"""阶段四 B2：正式 v2 Simulator runtime 的五话题物理步协调。"""
import inspect
from fractions import Fraction
from importlib import import_module
from pathlib import Path

import pytest

from slope_sim.interfaces.models import ImuAttitude, LidarPoint, LidarPointCloud, WheelState
from slope_sim.interfaces.transport import TransportSnapshot, TransportTopicQuality
from slope_sim.model_registry import get_robot_model
from slope_sim.realtime import DeadlinePacer, RuntimeObservationCadence
from slope_sim.truth_sensors import Stage4RtkState


class FakeControllerTransport:
    """controller 构造期间无需真实 transport，只保留关闭表面。"""

    def __init__(self) -> None:
        self.command_protocol_state = "waiting"
        self.command_peer_count = 0

    def set_command_verified(self) -> None:
        """让唯一 command publisher 通过 raw metadata 门。"""
        self.command_protocol_state = "verified"
        self.command_peer_count = 1

    def poll_peer_state(self) -> None:
        """测试 transport 仅暴露已经提交的 discovery 质量。"""

    def snapshot(self) -> TransportSnapshot:
        metadata = (
            ("slope_sim.interfaces.v2.WheelCommand",),
            ("proto",),
            ("0" * 64,),
        ) if self.command_protocol_state == "verified" else ((), (), ())
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=True,
            published_count=0,
            received_count=0,
            error_count=0,
            dropped_count=0,
            topic_quality=(
                TransportTopicQuality(
                    topic="/sim/wheel/command",
                    peer_connected=self.command_peer_count == 1,
                    peer_count=self.command_peer_count,
                    protocol_state=self.command_protocol_state,
                    protocol_detail="",
                    remote_type_names=metadata[0],
                    remote_encodings=metadata[1],
                    remote_descriptor_sha256=metadata[2],
                ),
            ),
        )

    def close(self) -> None:
        """测试不持有外部资源。"""


class RecordingTransport:
    """保留正式 runtime 发送的 raw payload 和完整 topic metadata。"""

    def __init__(self) -> None:
        self.calls: list[tuple[str, bytes, str, int, float]] = []

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float,
    ) -> bool:
        self.calls.append((topic, bytes(payload), type_name, sim_time_ns, wall_time))
        return True


class CenterLidar:
    """以请求时间构造中心 LiDAR 帧，确保 runtime 不代填采样时间。"""

    def scan(self, timestamp_ns: int) -> LidarPointCloud:
        return LidarPointCloud(
            timestamp_ns,
            "lidar_link",
            1,
            1,
            (LidarPoint(0, 1.0, 0.0, 0.1, 100, 1, 0),),
        )


class Stage4Truth:
    """为 10 Hz 同帧投影提供固定三点 RTK 与 IMU 真值。"""

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


def test_v2_simulator_runtime_publishes_five_topics_at_240_100_10_hz() -> None:
    """240 Hz 物理循环必须产出 100 Hz wheel 和同刻 10 Hz 三传感器 v2 帧。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    controller_type = import_module("slope_sim.interfaces.v2.runtime_protocol").V2RuntimeProtocol
    frames = import_module("slope_sim.interfaces.v2.sensor_frames")
    runtime_module = import_module("slope_sim.interfaces.v2.simulation_runtime")
    runtime_type = getattr(runtime_module, "V2SimulatorRuntime", None)
    assert runtime_type is not None, "v2 simulator runtime must exist"
    controller = controller_type(
        get_robot_model("df_mid"),
        transport=FakeControllerTransport(),
        descriptor=descriptor,
    )
    transport = RecordingTransport()
    feedback_requests: list[int] = []
    runtime = runtime_type(
        controller=controller,
        wheel_feedback_reader=lambda timestamp_ns: (
            feedback_requests.append(timestamp_ns)
            or WheelState(timestamp_ns, (1.5, -1.5), ())
        ),
        sensor_frames=frames.V2SensorFrameFactory(controller, CenterLidar(), Stage4Truth()),
        output_publisher=frames.V2OutputFramePublisher(transport, descriptor),
        wheel_state_factory=frames.V2WheelStateFactory(controller, "df_mid"),
    )

    for frame in range(240):
        runtime.after_physics_step(Fraction(1, 240), wall_time=frame / 240.0)

    assert feedback_requests == [index * 10_000_000 for index in range(1, 101)]
    assert len(transport.calls) == 130
    assert [topic for topic, *_rest in transport.calls].count("/sim/wheel/state") == 100
    sensor_calls = [call for call in transport.calls if call[0] != "/sim/wheel/state"]
    assert len(sensor_calls) == 30
    for index in range(10):
        group = sensor_calls[index * 3:(index + 1) * 3]
        assert [item[0] for item in group] == [
            "/sim/lidar/points", "/sim/rtk/state", "/sim/imu/attitude",
        ]
        assert {item[3] for item in group} == {(index + 1) * 100_000_000}


def test_v2_simulator_runtime_routes_verified_command_to_existing_authority() -> None:
    """正式 runtime 只能在 raw transport verified 后向既有 authority 提交 command。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    controller_type = import_module("slope_sim.interfaces.v2.runtime_protocol").V2RuntimeProtocol
    frames = import_module("slope_sim.interfaces.v2.sensor_frames")
    runtime_type = import_module("slope_sim.interfaces.v2.simulation_runtime").V2SimulatorRuntime
    controller_transport = FakeControllerTransport()
    controller_transport.set_command_verified()
    controller = controller_type(
        get_robot_model("df_mid"), transport=controller_transport, descriptor=descriptor
    )
    runtime = runtime_type(
        controller=controller,
        wheel_feedback_reader=lambda timestamp_ns: WheelState(timestamp_ns, (0.0, 0.0), ()),
        sensor_frames=frames.V2SensorFrameFactory(controller, CenterLidar(), Stage4Truth()),
        output_publisher=frames.V2OutputFramePublisher(RecordingTransport(), descriptor),
        wheel_state_factory=frames.V2WheelStateFactory(controller, "df_mid"),
    )

    runtime.refresh_transport()
    identity = controller.snapshot()
    command = import_module("slope_sim.interfaces.v2.models").WheelCommandV2(
        timestamp_ns=10_000_000,
        drive_wheel_speed_rad_s=(1.0, -1.0),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=identity.world_generation,
        command_generation=identity.command_generation,
        source_id="manual.tool-1",
        source_session_id=b"m" * 16,
        robot_model="df_mid",
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=descriptor.sha256,
    )
    payload = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor).encode(command).payload

    assert runtime.accept_command_payload(payload, received_at=1.0) is True
    assert runtime.command_decision(now=1.0).waiting is False


def test_v2_simulator_runtime_projects_authority_rejection_with_command_identity() -> None:
    """authority 拒绝必须以独立诊断域保留原始身份和原因。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    controller_type = import_module("slope_sim.interfaces.v2.runtime_protocol").V2RuntimeProtocol
    snapshot_type = import_module(
        "slope_sim.interfaces.v2.dashboard_snapshot"
    ).V2DashboardSnapshotStore
    frames = import_module("slope_sim.interfaces.v2.sensor_frames")
    runtime_type = import_module("slope_sim.interfaces.v2.simulation_runtime").V2SimulatorRuntime
    controller_transport = FakeControllerTransport()
    controller_transport.set_command_verified()
    controller = controller_type(
        get_robot_model("df_mid"), transport=controller_transport, descriptor=descriptor
    )
    dashboard = snapshot_type(robot_model="df_mid")
    runtime = runtime_type(
        controller=controller,
        wheel_feedback_reader=lambda timestamp_ns: WheelState(timestamp_ns, (0.0, 0.0), ()),
        sensor_frames=frames.V2SensorFrameFactory(controller, CenterLidar(), Stage4Truth()),
        output_publisher=frames.V2OutputFramePublisher(RecordingTransport(), descriptor),
        wheel_state_factory=frames.V2WheelStateFactory(controller, "df_mid"),
        dashboard_snapshot_store=dashboard,
    )
    runtime.refresh_transport()
    for frame in range(24):
        runtime.after_physics_step(Fraction(1, 240), wall_time=frame / 240.0)
    identity = controller.snapshot()
    command = import_module("slope_sim.interfaces.v2.models").WheelCommandV2(
        timestamp_ns=10_000_000,
        drive_wheel_speed_rad_s=(1.0, -1.0),
        steering_wheel_speed_rad_s=(),
        sequence=1,
        world_generation=identity.world_generation + 1,
        command_generation=identity.command_generation,
        source_id="manual.tool-1",
        source_session_id=b"m" * 16,
        robot_model="df_mid",
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=descriptor.sha256,
    )
    payload = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor).encode(command).payload

    assert runtime.accept_command_payload(payload, received_at=2.5) is False

    snapshot = dashboard.snapshot()
    assert snapshot is not None
    assert len(snapshot.authority_rejections) == 1
    rejection = snapshot.authority_rejections[0]
    assert rejection.topic == "/sim/wheel/command"
    assert rejection.source_id == "manual.tool-1"
    assert rejection.sequence == 1
    assert rejection.simulation_session_id == identity.simulation_session_id
    assert rejection.world_generation == identity.world_generation + 1
    assert rejection.reason == "world generation does not match"
    assert rejection.received_at == 2.5
    assert snapshot.observer_rejections == ()
    assert snapshot.topic_observation("/sim/wheel/command").authority_error_count == 1
    assert snapshot.topic_observation("/sim/wheel/command").observer_error_count == 0


def test_v2_dashboard_accepts_delayed_command_after_newer_physics_query() -> None:
    """延迟 raw callback 的观测时间不能回拨物理线程已推进的 mailbox query 时钟。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    controller_type = import_module("slope_sim.interfaces.v2.runtime_protocol").V2RuntimeProtocol
    dashboard_type = import_module(
        "slope_sim.interfaces.v2.dashboard_snapshot"
    ).V2DashboardSnapshotStore
    frames = import_module("slope_sim.interfaces.v2.sensor_frames")
    runtime_type = import_module("slope_sim.interfaces.v2.simulation_runtime").V2SimulatorRuntime
    controller_transport = FakeControllerTransport()
    controller_transport.set_command_verified()
    controller = controller_type(
        get_robot_model("df_mid"), transport=controller_transport, descriptor=descriptor
    )
    dashboard = dashboard_type(robot_model="df_mid")
    runtime = runtime_type(
        controller=controller,
        wheel_feedback_reader=lambda timestamp_ns: WheelState(timestamp_ns, (0.0, 0.0), ()),
        sensor_frames=frames.V2SensorFrameFactory(controller, CenterLidar(), Stage4Truth()),
        output_publisher=frames.V2OutputFramePublisher(RecordingTransport(), descriptor),
        wheel_state_factory=frames.V2WheelStateFactory(controller, "df_mid"),
        dashboard_snapshot_store=dashboard,
    )
    runtime.refresh_transport()
    identity = controller.snapshot()
    model_type = import_module("slope_sim.interfaces.v2.models").WheelCommandV2
    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)

    def payload(sequence: int) -> bytes:
        return codec.encode(
            model_type(
                timestamp_ns=10_000_000 + sequence,
                drive_wheel_speed_rad_s=(1.0, -1.0),
                steering_wheel_speed_rad_s=(),
                sequence=sequence,
                world_generation=identity.world_generation,
                command_generation=identity.command_generation,
                source_id="manual.tool-1",
                source_session_id=b"m" * 16,
                robot_model="df_mid",
                simulation_session_id=identity.simulation_session_id,
                descriptor_sha256=descriptor.sha256,
            )
        ).payload

    assert runtime.accept_command_payload(payload(0), received_at=8.0) is True
    assert runtime.command_decision(now=10.0).timed_out
    assert runtime.accept_command_payload(payload(1), received_at=9.0) is True


def test_v2_runtime_physics_loop_uses_20hz_observation_with_fresh_decisions(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """正式物理循环首帧观测一次，随后 20 Hz 刷新且逐帧用新的墙钟决策。"""
    runtime_script = import_module("scripts.stage4_v2_simulation_runtime")
    observe_then_decide = getattr(runtime_script, "_observe_transport_then_decide", None)
    assert callable(
        observe_then_decide
    ), "runtime needs a single observation-and-decision loop boundary"

    clock = {"value": 0.0}
    monkeypatch.setattr(runtime_script.time, "monotonic", lambda: clock["value"])

    class Runtime:
        def __init__(self) -> None:
            self.refresh_times: list[float] = []
            self.decision_times: list[float] = []

        def refresh_transport(self) -> None:
            self.refresh_times.append(clock["value"])

        def command_decision(self, *, now: float) -> object:
            self.decision_times.append(now)
            return object()

    runtime = Runtime()
    cadence = RuntimeObservationCadence(monotonic=lambda: clock["value"])
    for frame in range(240):
        clock["value"] = frame / 240.0
        observe_then_decide(runtime, observation_cadence=cadence)

    assert len(runtime.refresh_times) == 20
    assert runtime.refresh_times[0] == pytest.approx(0.0)
    assert all(
        later - earlier >= (1.0 / 20.0) - 1e-12
        for earlier, later in zip(runtime.refresh_times, runtime.refresh_times[1:])
    )
    assert runtime.decision_times == pytest.approx([frame / 240.0 for frame in range(240)])
    source = inspect.getsource(runtime_script.run_v2_simulation_runtime)
    assert "RuntimeObservationCadence()" in source
    assert "_observe_transport_then_decide(" in source


def test_v2_runtime_physics_loop_uses_deadline_pacer_for_overrun_yields() -> None:
    """正式循环须以绝对 240 Hz deadline 等待，超期也让出一次主线程。"""
    runtime_script = import_module("scripts.stage4_v2_simulation_runtime")
    pacer_type = getattr(runtime_script, "DeadlinePacer", None)
    assert pacer_type is DeadlinePacer, "runtime must reuse the shared deadline pacer"

    clock = [0.0]
    sleeps: list[float] = []

    def sleep(delay: float) -> None:
        sleeps.append(delay)
        clock[0] += delay

    pacer = pacer_type(
        1.0 / 240.0,
        monotonic=lambda: clock[0],
        sleep=sleep,
    )
    pacer.start()
    clock[0] += 0.001
    on_time = pacer.wait_for_next_deadline()
    clock[0] += 0.006
    overrun = pacer.wait_for_next_deadline()

    assert sleeps == pytest.approx([(1.0 / 240.0) - 0.001, 0.0])
    assert [on_time.deadline_sec, overrun.deadline_sec] == pytest.approx(
        [1.0 / 240.0, 2.0 / 240.0]
    )
    assert on_time.overrun is False
    assert overrun.overrun is True
    source = inspect.getsource(runtime_script.run_v2_simulation_runtime)
    assert "pacer = DeadlinePacer(config.time_step)" in source
    assert "pacer.start()" in source
    assert "pacer.wait_for_next_deadline()" in source


@pytest.mark.parametrize(
    ("previous_interval", "installed_interval"),
    ((0.005, 0.001), (0.0005, 0.0005)),
)
def test_v2_runtime_scopes_transport_lane_scheduling_and_restores_exact_interval(
    monkeypatch: pytest.MonkeyPatch,
    previous_interval: float,
    installed_interval: float,
) -> None:
    """实时窗口收紧 GIL 轮转，并在正常或异常清理路径恢复调用方值。"""
    runtime_script = import_module("scripts.stage4_v2_simulation_runtime")
    install = getattr(runtime_script, "_install_transport_lane_scheduling", None)
    restore = getattr(runtime_script, "_restore_transport_lane_scheduling", None)
    assert callable(install), "Stage4 runtime must install transport lane scheduling"
    assert callable(restore), "Stage4 runtime must restore transport lane scheduling"

    set_calls: list[float] = []
    monkeypatch.setattr(
        runtime_script.sys,
        "getswitchinterval",
        lambda: previous_interval,
    )
    monkeypatch.setattr(runtime_script.sys, "setswitchinterval", set_calls.append)

    saved_interval = install()
    restore(saved_interval)

    assert saved_interval == previous_interval
    assert set_calls == [installed_interval, previous_interval]
    source = inspect.getsource(runtime_script.run_v2_simulation_runtime)
    install_call = source.index("_install_transport_lane_scheduling()")
    physics_loop = source.index("for _ in range(physics_steps):")
    outer_finally = source.rfind("\n    finally:")
    restore_call = source.index(
        "_restore_transport_lane_scheduling(", outer_finally
    )
    assert install_call < physics_loop
    assert outer_finally > physics_loop
    assert restore_call > outer_finally


def test_verified_peer_wait_allows_two_output_consumers_but_requires_command_publisher() -> None:
    """四参与者门禁允许两个只读 output consumer，却仍严格限制唯一 command publisher。"""
    runtime_script = import_module("scripts.stage4_v2_simulation_runtime")
    topics = import_module("slope_sim.interfaces.v2.topics").V2_TOPICS

    def quality(*, topic: str, peer_count: int) -> TransportTopicQuality:
        return TransportTopicQuality(
            topic=topic,
            peer_connected=peer_count > 0,
            peer_count=peer_count,
            protocol_state="verified" if peer_count else "waiting",
            remote_type_names=("slope_sim.interfaces.v2.Test",) * peer_count,
            remote_encodings=("proto",) * peer_count,
            remote_descriptor_sha256=("0" * 64,) * peer_count,
        )

    def snapshot(*, command_peers: int, output_peers: int) -> TransportSnapshot:
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=True,
            published_count=0,
            received_count=0,
            error_count=0,
            dropped_count=0,
            topic_quality=tuple(
                quality(
                    topic=contract.topic,
                    peer_count=(
                        command_peers
                        if contract.direction == "subscribe"
                        else output_peers
                    ),
                )
                for contract in topics
            ),
        )

    class Runtime:
        def __init__(self, transport_snapshot: TransportSnapshot) -> None:
            self.transport_snapshot = transport_snapshot

        def refresh_transport(self) -> TransportSnapshot:
            return self.transport_snapshot

    runtime_script._wait_for_verified_peers(
        Runtime(snapshot(command_peers=1, output_peers=2)),
        timeout_sec=0.0,
    )
    with pytest.raises(TimeoutError, match="verified"):
        runtime_script._wait_for_verified_peers(
            Runtime(snapshot(command_peers=1, output_peers=0)),
            timeout_sec=0.0,
        )
    with pytest.raises(TimeoutError, match="verified"):
        runtime_script._wait_for_verified_peers(
            Runtime(snapshot(command_peers=2, output_peers=2)),
            timeout_sec=0.0,
        )


def test_runtime_ready_follows_verified_peers_and_shared_start_precedes_window_clock(
    tmp_path: Path,
) -> None:
    """协调 runtime 必须只在 verified 后独占 ready，并由共享 start 释放窗口。"""
    runtime_script = import_module("scripts.stage4_v2_simulation_runtime")
    validate = getattr(runtime_script, "_coordination_paths", None)
    write_marker = getattr(runtime_script, "_write_marker", None)
    wait_start = getattr(runtime_script, "_wait_for_start", None)
    assert callable(validate), "runtime needs paired ready/start coordination paths"
    assert callable(write_marker), "runtime needs exclusive ready marker creation"
    assert callable(wait_start), "runtime needs a shared start wait"

    ready = tmp_path / "runtime.ready"
    start = tmp_path / "start.signal"
    assert validate(ready, start) == (ready, start)
    write_marker(ready)
    assert ready.exists()
    with pytest.raises(FileExistsError):
        write_marker(ready)
    with pytest.raises(TimeoutError):
        wait_start(start, deadline=0.0)
    write_marker(start)
    wait_start(start, deadline=0.0)
    with pytest.raises(ValueError):
        validate(ready, None)


def test_scene_cli_accepts_path_without_world_selectors(tmp_path: Path) -> None:
    """显式 scene 必须成为唯一世界来源，CLI 不得暗中注入默认 selector。"""
    runtime_script = import_module("scripts.stage4_v2_simulation_runtime")
    scene_path = tmp_path / "golf-scene.yaml"
    result_path = tmp_path / "runtime.json"

    parsed = runtime_script._parse_args(
        ["--result-json", str(result_path), "--scene", str(scene_path)]
    )

    assert parsed.scene == scene_path
    assert parsed.robot_model is None
    assert parsed.terrain_model is None


def test_runtime_result_exposes_frozen_lidar_pattern_identity() -> None:
    """runtime JSON 必须提供独立于 descriptor SHA 的扫描表身份。"""
    runtime_script = import_module("scripts.stage4_v2_simulation_runtime")
    pattern_identity = getattr(runtime_script, "_lidar_pattern_result_identity", None)
    assert callable(pattern_identity), "runtime lidar pattern result identity is missing"

    assert pattern_identity() == {
        "lidar_pattern_version": "livox-mid360-800000-v1",
        "lidar_pattern_sha256": "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca",
    }
    source = inspect.getsource(runtime_script.run_v2_simulation_runtime)
    assert "**_lidar_pattern_result_identity()" in source


@pytest.mark.parametrize(
    ("selector", "value"),
    (("--robot-model", "df_mid"), ("--terrain-model", "golf_heightfield")),
)
def test_scene_cli_rejects_world_selector_conflicts(
    tmp_path: Path,
    selector: str,
    value: str,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """scene 与显式车型或地形同时出现时必须在启动 PyBullet 前拒绝。"""
    runtime_script = import_module("scripts.stage4_v2_simulation_runtime")

    with pytest.raises(SystemExit) as error:
        runtime_script._parse_args(
            [
                "--result-json",
                str(tmp_path / "runtime.json"),
                "--scene",
                str(tmp_path / "golf-scene.yaml"),
                selector,
                value,
            ]
        )
    assert error.value.code == 2
    assert "--scene cannot be combined" in capsys.readouterr().err
