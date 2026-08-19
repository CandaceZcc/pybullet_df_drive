"""阶段四 B2：v2 Dashboard 只读有界快照的合同测试。"""
from __future__ import annotations

from dataclasses import fields, replace
from fractions import Fraction
from importlib import import_module
from threading import Event, Thread, current_thread

import pytest

from slope_sim.interfaces.models import ImuAttitude, LidarPoint, LidarPointCloud, WheelState
from slope_sim.interfaces.transport import TransportSnapshot, TransportTopicQuality
from slope_sim.model_registry import get_robot_model
from slope_sim.truth_sensors import Stage4RtkState


class _ControllerTransport:
    """为真实 v2 controller 提供本单元不涉及的最小传输边界。"""

    def close(self) -> None:
        """测试不持有外部资源。"""


class _CenterLidar:
    """按请求时间生成中心 LiDAR，避免测试伪造 v2 身份字段。"""

    def scan(self, timestamp_ns: int) -> LidarPointCloud:
        return LidarPointCloud(
            timestamp_ns,
            "lidar_link",
            1,
            1,
            (LidarPoint(0, 1.0, 0.0, 0.1, 100, 1, 0),),
        )


class _TruthSensors:
    """提供真实工厂所需的同帧三点 RTK 和 IMU 输入。"""

    def read_rtk(self, timestamp_ns: int) -> Stage4RtkState:
        return Stage4RtkState(
            timestamp_ns,
            (1.2, 2.3, 0.4),
            (1.0, 2.0, 0.4),
            (0.8, 1.7, 0.4),
            -0.25,
        )

    def read_imu(self, timestamp_ns: int) -> ImuAttitude:
        return ImuAttitude(timestamp_ns, 0.1, -0.2)


class _RecordingTransport:
    """记录 runtime 的 raw 输出，保持 Dashboard 测试不触碰 eCAL。"""

    def publish(
        self,
        _topic: str,
        _payload: bytes,
        _type_name: str,
        _sim_time_ns: int,
        *,
        wall_time: float,
    ) -> bool:
        assert wall_time >= 0.0
        return True


class _DeferredCenterLidarService:
    """按一次 poll 延迟回传 child 编码帧，模拟正式异步 worker。"""

    def __init__(self, descriptor: object) -> None:
        self._codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
        self._capture: dict[str, object] | None = None
        self._polls_after_capture = 0

    def capture(self, **kwargs: object) -> bool:
        self._capture = dict(kwargs)
        return True

    def poll(self) -> object | None:
        if self._capture is None:
            return None
        self._polls_after_capture += 1
        if self._polls_after_capture == 1:
            return None
        capture = self._capture
        self._capture = None
        identity = capture["output_identity"]
        model = import_module("slope_sim.interfaces.v2.models").LidarPointCloudV2(
            capture["timestamp_ns"],
            "lidar_link",
            0,
            1,
            (),
            identity.sequence,
            identity.world_generation,
            identity.simulation_session_id,
            identity.descriptor_sha256,
        )
        return type("Prepared", (), {
            "topic": "lidar_link",
            "timestamp_ns": capture["timestamp_ns"],
            "protobuf_payload": self._codec.encode(model).payload,
        })()

    def drain_events(self) -> tuple[object, ...]:
        """匹配异步工厂的事件接口；本 Dashboard mock 不产生 worker 告警。"""
        return ()


def _coherent_frames_and_wheel() -> tuple[object, object]:
    """通过正式 factory 生成不可篡改的同一 v2 session/world 输入。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    controller_type = import_module("slope_sim.interfaces.v2.runtime_protocol").V2RuntimeProtocol
    frame_module = import_module("slope_sim.interfaces.v2.sensor_frames")
    controller = controller_type(
        get_robot_model("df_mid"),
        transport=_ControllerTransport(),
        descriptor=descriptor,
    )
    frames = frame_module.V2SensorFrameFactory(
        controller, _CenterLidar(), _TruthSensors()
    ).capture(100_000_000)
    wheel = frame_module.V2WheelStateFactory(controller, "df_mid").build(
        WheelState(100_000_000, (1.5, -1.5), ())
    )
    return frames, wheel


def test_v2_dashboard_store_keeps_only_one_session_world_and_rejects_mixed_generation() -> None:
    """GUI 只能读取同一 v2 session/world 的最后完整数据，混代不得覆盖。"""
    module = import_module("slope_sim.interfaces.v2.dashboard_snapshot")
    store_type = getattr(module, "V2DashboardSnapshotStore", None)
    assert store_type is not None, "v2 dashboard snapshot store must exist"
    frames, wheel = _coherent_frames_and_wheel()
    store = store_type()

    store.update_sensor_frames(frames)
    store.update_wheel_state(wheel)
    snapshot = store.snapshot()

    assert snapshot is not None
    assert snapshot.lidar_point_count == frames.lidar.point_num
    assert snapshot.lidar_timestamp_ns == frames.lidar.timebase_ns
    assert snapshot.lidar_sequence == frames.lidar.sequence
    assert snapshot.rtk is frames.rtk
    assert snapshot.imu is frames.imu
    assert snapshot.wheel_state is wheel
    assert snapshot.simulation_session_id == frames.lidar.simulation_session_id
    assert snapshot.world_generation == frames.lidar.world_generation

    with pytest.raises(ValueError, match="world_generation"):
        store.update_wheel_state(replace(wheel, world_generation=wheel.world_generation + 1))

    assert store.snapshot() is snapshot


def test_v2_dashboard_store_accepts_only_a_verified_immutable_ipc_snapshot() -> None:
    """GUI 进程只能接收同一 v2 identity 的完整不可变 snapshot，不能绕过边界校验。"""
    module = import_module("slope_sim.interfaces.v2.dashboard_snapshot")
    store_type = module.V2DashboardSnapshotStore
    frames, wheel = _coherent_frames_and_wheel()
    source = store_type()
    source.update_sensor_frames(frames)
    source.update_wheel_state(wheel)
    snapshot = source.snapshot()
    assert snapshot is not None
    receiver = store_type()

    receiver.update_snapshot(snapshot)

    assert receiver.snapshot() is snapshot
    with pytest.raises(ValueError, match="world_generation"):
        receiver.update_snapshot(replace(snapshot, world_generation=snapshot.world_generation + 1))
    assert receiver.snapshot() is snapshot


def test_v2_dashboard_store_notifies_after_each_complete_snapshot_replacement() -> None:
    """IPC producer 只在原子快照完成后收到通知，不能观察到半帧数据。"""
    module = import_module("slope_sim.interfaces.v2.dashboard_snapshot")
    frames, wheel = _coherent_frames_and_wheel()
    observed: list[object] = []
    store = module.V2DashboardSnapshotStore(on_update=observed.append)

    store.update_sensor_frames(frames)
    store.update_wheel_state(wheel)

    assert len(observed) == 2
    assert observed[0].wheel_state is None
    assert observed[0].lidar_point_count == frames.lidar.point_num
    assert observed[0].rtk is frames.rtk
    assert observed[0].imu is frames.imu
    assert tuple(item.topic for item in observed[0].topic_observations) == (
        "/sim/wheel/command", "/sim/wheel/state", "/sim/lidar/points",
        "/sim/rtk/state", "/sim/imu/attitude",
    )
    assert observed[1] is store.snapshot()


def test_v2_simulator_runtime_publishes_latest_successful_frames_to_dashboard_store() -> None:
    """物理 runtime 只在输出成功后单向更新 GUI 可读的 v2 快照。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    controller_type = import_module("slope_sim.interfaces.v2.runtime_protocol").V2RuntimeProtocol
    frame_module = import_module("slope_sim.interfaces.v2.sensor_frames")
    runtime_type = import_module("slope_sim.interfaces.v2.simulation_runtime").V2SimulatorRuntime
    store_type = import_module("slope_sim.interfaces.v2.dashboard_snapshot").V2DashboardSnapshotStore
    controller = controller_type(
        get_robot_model("df_mid"),
        transport=_ControllerTransport(),
        descriptor=descriptor,
    )
    store = store_type()
    runtime = runtime_type(
        controller=controller,
        wheel_feedback_reader=lambda timestamp_ns: WheelState(
            timestamp_ns, (1.5, -1.5), ()
        ),
        sensor_frames=frame_module.V2SensorFrameFactory(
            controller, _CenterLidar(), _TruthSensors()
        ),
        output_publisher=frame_module.V2OutputFramePublisher(
            _RecordingTransport(), descriptor
        ),
        wheel_state_factory=frame_module.V2WheelStateFactory(controller, "df_mid"),
        dashboard_snapshot_store=store,
    )

    for frame in range(24):
        runtime.after_physics_step(Fraction(1, 240), wall_time=frame / 240.0)

    snapshot = store.snapshot()
    assert snapshot is not None
    assert snapshot.wheel_state is not None
    assert snapshot.lidar_point_count == 1
    assert snapshot.rtk is not None
    assert snapshot.imu is not None
    assert snapshot.wheel_state.timestamp_ns == 100_000_000
    assert snapshot.lidar_timestamp_ns == snapshot.rtk.timestamp_ns == snapshot.imu.timestamp_ns == 100_000_000


def test_v2_async_runtime_keeps_only_lidar_metadata_in_dashboard_snapshot() -> None:
    """异步完成帧只把可显示元数据交给 Dashboard，绝不保留 child LiDAR bytes。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    controller_type = import_module("slope_sim.interfaces.v2.runtime_protocol").V2RuntimeProtocol
    frame_module = import_module("slope_sim.interfaces.v2.sensor_frames")
    runtime_type = import_module("slope_sim.interfaces.v2.simulation_runtime").V2SimulatorRuntime
    store_type = import_module("slope_sim.interfaces.v2.dashboard_snapshot").V2DashboardSnapshotStore
    controller = controller_type(
        get_robot_model("df_mid"),
        transport=_ControllerTransport(),
        descriptor=descriptor,
    )
    service = _DeferredCenterLidarService(descriptor)
    store = store_type()
    runtime = runtime_type(
        controller=controller,
        wheel_feedback_reader=lambda timestamp_ns: WheelState(
            timestamp_ns, (1.5, -1.5), ()
        ),
        sensor_frames=frame_module.V2AsyncSensorFrameFactory(
            controller,
            service,
            _TruthSensors(),
            lambda: ("mount", ()),
        ),
        output_publisher=frame_module.V2OutputFramePublisher(
            _RecordingTransport(), descriptor
        ),
        wheel_state_factory=frame_module.V2WheelStateFactory(controller, "df_mid"),
        dashboard_snapshot_store=store,
    )

    for frame in range(25):
        runtime.after_physics_step(Fraction(1, 240), wall_time=frame / 240.0)

    snapshot = store.snapshot()
    assert snapshot is not None
    assert tuple(field.name for field in fields(snapshot)) == (
        "simulation_session_id",
        "descriptor_sha256",
        "world_generation",
        "wheel_state",
        "lidar_timestamp_ns",
        "lidar_sequence",
        "lidar_point_count",
        "rtk",
        "imu",
        "topic_observations",
        "robot_model",
    )
    assert snapshot.lidar_timestamp_ns == 100_000_000
    assert snapshot.lidar_sequence == 0
    assert snapshot.lidar_point_count is None
    assert snapshot.rtk is not None
    assert snapshot.imu is not None


def test_v2_dashboard_live_observation_keeps_telemetry_and_transport_times_separate() -> None:
    """transport 刷新不能伪造遥测新鲜度，GUI age 只能使用 telemetry 时刻。"""
    module = import_module("slope_sim.interfaces.v2.dashboard_snapshot")
    frames, _wheel = _coherent_frames_and_wheel()
    store = module.V2DashboardSnapshotStore()

    store.update_sensor_frames(frames, observed_at=10.0)
    store.refresh_transport(
        TransportSnapshot("ecal", True, 0, 0, 0, 0, topic_quality=()),
        observed_at=15.0,
    )
    snapshot = store.snapshot()

    assert snapshot is not None
    lidar = snapshot.topic_observation("/sim/lidar/points")
    assert lidar.telemetry_observed_at == 10.0
    assert lidar.transport_observed_at == 15.0
    assert snapshot.telemetry_age(now=16.5, topic="/sim/lidar/points") == 6.5


def test_v2_dashboard_live_observation_uses_windowed_rates_and_exact_topic_drops() -> None:
    """五话题显示固定目标频率、单调窗口频率及独立 transport drops。"""
    module = import_module("slope_sim.interfaces.v2.dashboard_snapshot")
    frames, wheel = _coherent_frames_and_wheel()
    store = module.V2DashboardSnapshotStore()

    store.update_wheel_state(wheel, observed_at=1.0)
    store.update_wheel_state(replace(wheel, sequence=wheel.sequence + 1), observed_at=1.1)
    assert store.snapshot(now=1.1).topic_observation("/sim/wheel/state").actual_hz is None
    store.update_wheel_state(replace(wheel, sequence=wheel.sequence + 2), observed_at=1.2)
    store.update_sensor_frames(frames, observed_at=1.2)
    store.refresh_transport(
        TransportSnapshot(
            "ecal", True, 0, 0, 0, 0,
            topic_quality=(
                TransportTopicQuality(
                    topic="/sim/lidar/points", dropped_count=3, error_count=1,
                    peer_connected=True, peer_count=1, protocol_state="verified",
                    remote_type_names=("test.v2.Message",), remote_encodings=("proto",),
                    remote_descriptor_sha256=("0" * 64,),
                ),
            ),
        ),
        observed_at=1.3,
    )
    snapshot = store.snapshot(now=1.3)

    assert snapshot is not None
    wheel_observation = snapshot.topic_observation("/sim/wheel/state")
    lidar_observation = snapshot.topic_observation("/sim/lidar/points")
    assert wheel_observation.target_hz == 100
    assert wheel_observation.actual_hz == pytest.approx(10.0)
    assert lidar_observation.target_hz == 10
    assert lidar_observation.actual_hz is None
    assert lidar_observation.dropped_count == 3
    assert lidar_observation.error_count == 1
    assert lidar_observation.peer_count == 1
    assert lidar_observation.protocol_state == "verified"

    stale = store.snapshot(now=3.3)
    assert stale.topic_observation("/sim/wheel/state").actual_hz is None
    assert stale.topic_observation("/sim/wheel/state").telemetry_observed_at == 1.2


def test_v2_dashboard_ipc_snapshot_keeps_mature_rate_until_window_expires() -> None:
    """IPC 接收端无事件 deque，也须按随快照到达的 telemetry 时刻有界保留 Hz。"""
    module = import_module("slope_sim.interfaces.v2.dashboard_snapshot")
    _frames, wheel = _coherent_frames_and_wheel()
    source = module.V2DashboardSnapshotStore()
    source.update_wheel_state(wheel, observed_at=1.0)
    source.update_wheel_state(replace(wheel, sequence=wheel.sequence + 1), observed_at=1.1)
    source.update_wheel_state(replace(wheel, sequence=wheel.sequence + 2), observed_at=1.2)
    receiver = module.V2DashboardSnapshotStore()
    receiver.update_snapshot(source.snapshot(now=1.2))

    assert receiver.snapshot(now=1.3).topic_observation("/sim/wheel/state").actual_hz == pytest.approx(10.0)
    assert receiver.snapshot(now=3.3).topic_observation("/sim/wheel/state").actual_hz is None


def test_v2_dashboard_reader_never_overwrites_newer_writer_snapshot(monkeypatch) -> None:
    """GUI 派生旧快照与 writer 交错时，Store 最终值仍必须是最新 telemetry。"""
    module = import_module("slope_sim.interfaces.v2.dashboard_snapshot")
    _frames, wheel = _coherent_frames_and_wheel()
    store = module.V2DashboardSnapshotStore()
    store.update_wheel_state(wheel, observed_at=1.0)
    store.update_wheel_state(replace(wheel, sequence=wheel.sequence + 1), observed_at=1.1)
    store.update_wheel_state(replace(wheel, sequence=wheel.sequence + 2), observed_at=1.2)
    reader_has_old_base = Event()
    continue_reader = Event()
    original_make_snapshot = store._make_snapshot

    def gated_make_snapshot(*args, **kwargs):
        if current_thread().name == "dashboard-reader":
            reader_has_old_base.set()
            assert continue_reader.wait(timeout=2.0)
        return original_make_snapshot(*args, **kwargs)

    monkeypatch.setattr(store, "_make_snapshot", gated_make_snapshot)
    derived: list[object] = []
    reader = Thread(
        target=lambda: derived.append(store.snapshot(now=3.3)),
        name="dashboard-reader",
    )
    reader.start()
    assert reader_has_old_base.wait(timeout=2.0)
    newest = replace(wheel, sequence=wheel.sequence + 3, timestamp_ns=330_000_000)
    store.update_wheel_state(newest, observed_at=3.3)
    continue_reader.set()
    reader.join(timeout=2.0)

    assert not reader.is_alive()
    assert derived[0].topic_observation("/sim/wheel/state").actual_hz is None
    latest = store.snapshot()
    assert latest.wheel_state is newest
    assert latest.topic_observation("/sim/wheel/state").latest_sequence == newest.sequence
    assert latest.topic_observation("/sim/wheel/state").telemetry_observed_at == 3.3


def test_v2_dashboard_live_observation_records_only_accepted_command_metadata() -> None:
    """拒绝 command 不得污染 dashboard；接受后使用 authority/mailbox 的元数据。"""
    module = import_module("slope_sim.interfaces.v2.dashboard_snapshot")
    frames, _wheel = _coherent_frames_and_wheel()
    store = module.V2DashboardSnapshotStore()
    store.update_sensor_frames(frames, observed_at=1.0)

    store.record_accepted_command(
        sequence=7,
        timestamp_ns=900_000_000,
        received_at=2.0,
        accepted=False,
    )
    assert store.snapshot().topic_observation("/sim/wheel/command").latest_sequence is None

    store.record_accepted_command(
        sequence=7,
        timestamp_ns=900_000_000,
        received_at=2.0,
        accepted=True,
    )
    command = store.snapshot().topic_observation("/sim/wheel/command")
    assert command.latest_sequence == 7
    assert command.latest_timestamp_ns == 900_000_000
    assert command.telemetry_observed_at == 2.0
