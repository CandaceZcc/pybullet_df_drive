"""阶段四 B2：中心 LiDAR、三点 RTK 与 IMU 的 v2 同帧投影。"""
from importlib import import_module
from fractions import Fraction

import pytest

from slope_sim.interfaces.models import ImuAttitude, LidarPoint, LidarPointCloud, WheelState
from slope_sim.lidar_worker import LidarServiceEvent
from slope_sim.model_registry import get_robot_model
from slope_sim.truth_sensors import Stage4RtkState


class FakeV2Transport:
    """只提供传感器帧工厂测试所需的最小 controller transport。"""

    def close(self) -> None:
        """controller 关闭时无需释放真实资源。"""


class RecordingPublishTransport:
    """记录发送层交给 transport 的唯一原始 bytes。"""

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


class StubCenterLidar:
    """返回固定中心扫描，验证工厂不更改采样时间。"""

    def scan(self, timestamp_ns: int) -> LidarPointCloud:
        return LidarPointCloud(
            timestamp_ns,
            "lidar_link",
            2,
            1,
            (
                LidarPoint(0, 1.0, 0.0, 0.1, 100, 1, 0),
                LidarPoint(99_982_638, 0.8, -0.2, 0.3, 200, 2, 15),
            ),
        )


class StubStage4Truth:
    """返回同一采样时刻的三点 RTK 和 IMU 真值。"""

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


class DeferredCenterLidarService:
    """模拟 worker 的非阻塞单帧完成，记录父端冻结请求。"""

    def __init__(self, payload: bytes) -> None:
        self.payload = payload
        self.capture_calls: list[dict[str, object]] = []
        self.pending = True

    def capture(self, **kwargs: object) -> bool:
        self.capture_calls.append(dict(kwargs))
        return True

    def poll(self):
        if self.pending:
            self.pending = False
            return None
        request = self.capture_calls[0]
        return type("Prepared", (), {
            "topic": "lidar_link",
            "timestamp_ns": request["timestamp_ns"],
            "protobuf_payload": self.payload,
        })()

    def drain_events(self) -> tuple[object, ...]:
        """正常完成路径没有终态 worker 事件。"""
        return ()


class TerminalFailureCenterLidarService(DeferredCenterLidarService):
    """模拟 worker 已明确丢弃的单个中心扫描。"""

    def __init__(self, error_code: str) -> None:
        super().__init__(b"unused")
        self.error_code = error_code

    def poll(self):
        return None

    def drain_events(self) -> tuple[object, ...]:
        request = self.capture_calls[0]
        return (
            LidarServiceEvent(
                1,
                "frame_failed",
                "topic",
                "lidar_link",
                (
                    1,
                    0,
                    0,
                    "lidar_link",
                    request["timestamp_ns"],
                ),
                self.error_code,
                "worker returned a terminal lidar frame failure",
            ),
        )


def test_v2_async_sensor_factory_defers_center_scan_and_keeps_one_reserved_identity(
    controller,
) -> None:
    """异步 LiDAR 必须先提交冻结 job，完成后才组合原始 payload 与同刻 RTK/IMU。"""
    module = import_module("slope_sim.interfaces.v2.sensor_frames")
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    identity = controller.reserve_output("/sim/lidar/points")
    payload = codec.encode(
        import_module("slope_sim.interfaces.v2.models").LidarPointCloudV2(
            100_000_000,
            "lidar_link",
            0,
            1,
            (),
            identity.sequence,
            identity.world_generation,
            identity.simulation_session_id,
            identity.descriptor_sha256,
        )
    ).payload
    # 测试本身占用的 sequence 不能混入 factory；新 controller 保持正式从零开始语义。
    controller.close()
    controller_type = import_module("slope_sim.interfaces.v2.runtime_protocol").V2RuntimeProtocol
    controller = controller_type(
        get_robot_model("df_mid"), transport=FakeV2Transport(), descriptor=descriptor
    )
    service = DeferredCenterLidarService(payload)
    factory_type = getattr(module, "V2AsyncSensorFrameFactory", None)
    assert factory_type is not None, "v2 async sensor frame factory must exist"
    factory = factory_type(
        controller,
        service,
        StubStage4Truth(),
        lambda: ("mount", ()),
    )

    assert factory.capture(100_000_000) is None
    assert service.capture_calls[0]["topic"] == "lidar_link"
    assert service.capture_calls[0]["timestamp_ns"] == 100_000_000
    assert factory.poll_completed() == ()

    completed = factory.poll_completed()
    assert len(completed) == 1
    frame = completed[0]
    assert frame.lidar_payload == payload
    assert frame.rtk.timestamp_ns == frame.imu.timestamp_ns == 100_000_000
    assert frame.lidar_timestamp_ns == 100_000_000


@pytest.mark.parametrize("error_code", ("sensor_overrun", "raycast_failed"))
def test_v2_async_sensor_factory_discards_truth_pair_when_worker_reports_terminal_failure(
    controller,
    error_code: str,
) -> None:
    """已明确丢弃的 LiDAR 不得让同刻 RTK/IMU 永久阻塞关闭 drain。"""
    module = import_module("slope_sim.interfaces.v2.sensor_frames")
    service = TerminalFailureCenterLidarService(error_code)
    factory = module.V2AsyncSensorFrameFactory(
        controller,
        service,
        StubStage4Truth(),
        lambda: ("mount", ()),
    )

    factory.capture(100_000_000)

    assert factory.poll_completed() == ()
    assert factory.has_pending() is False


def test_v2_output_publisher_forwards_prepared_lidar_payload_without_reencoding(
    controller,
) -> None:
    """异步完成帧必须把 child bytes 原样发布，RTK/IMU 保持正式 codec 路径。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    module = import_module("slope_sim.interfaces.v2.sensor_frames")
    frames = module.V2SensorFrameFactory(
        controller, StubCenterLidar(), StubStage4Truth()
    ).capture(100_000_000)
    prepared = module.V2PreparedSensorFrames(
        b"prepared-worker-lidar-bytes",
        100_000_000,
        frames.rtk,
        frames.imu,
        import_module("slope_sim.interfaces.v2.session").OutputIdentity(
            "/sim/lidar/points",
            frames.lidar.simulation_session_id,
            frames.lidar.descriptor_sha256,
            frames.lidar.world_generation,
            frames.lidar.sequence,
        ),
    )
    transport = RecordingPublishTransport()
    publisher = module.V2OutputFramePublisher(transport, descriptor)

    publisher.publish_prepared(prepared, wall_time=12.5)

    assert [(topic, payload, type_name, timestamp_ns) for topic, payload, type_name, timestamp_ns, _wall in transport.calls] == [
        ("/sim/lidar/points", b"prepared-worker-lidar-bytes", "slope_sim.interfaces.v2.LidarPointCloud", 100_000_000),
        ("/sim/rtk/state", transport.calls[1][1], "slope_sim.interfaces.v2.RtkState", 100_000_000),
        ("/sim/imu/attitude", transport.calls[2][1], "slope_sim.interfaces.v2.ImuAttitude", 100_000_000),
    ]


@pytest.fixture
def controller():
    """复用正式 session/controller，禁止测试伪造输出 identity。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    controller_type = getattr(
        import_module("slope_sim.interfaces.v2.runtime_protocol"),
        "V2RuntimeProtocol",
    )
    return controller_type(
        get_robot_model("df_mid"),
        transport=FakeV2Transport(),
        descriptor=descriptor,
    )


def test_v2_sensor_frame_factory_builds_three_identity_bound_same_timestamp_frames(controller) -> None:
    """一次 10 Hz 采样必须让三条传感器消息共享 timestamp/session/world。"""
    module = import_module("slope_sim.interfaces.v2.sensor_frames")
    factory_type = getattr(module, "V2SensorFrameFactory", None)
    assert factory_type is not None, "v2 sensor frame factory must exist"
    factory = factory_type(controller, StubCenterLidar(), StubStage4Truth())

    frames = factory.capture(100_000_000)

    assert frames.lidar.timebase_ns == frames.rtk.timestamp_ns == frames.imu.timestamp_ns == 100_000_000
    assert frames.lidar.frame_id == "lidar_link"
    assert frames.lidar.point_num == len(frames.lidar.points) == 2
    assert frames.rtk.left.x_m == pytest.approx(1.2)
    assert frames.rtk.center.y_m == pytest.approx(2.0)
    assert frames.rtk.right.z_m == pytest.approx(0.4)
    assert frames.imu.roll_rad == pytest.approx(0.1)
    assert frames.imu.pitch_rad == pytest.approx(-0.2)
    assert (frames.lidar.sequence, frames.rtk.sequence, frames.imu.sequence) == (0, 0, 0)
    assert len({frames.lidar.simulation_session_id, frames.rtk.simulation_session_id, frames.imu.simulation_session_id}) == 1
    assert len({frames.lidar.descriptor_sha256, frames.rtk.descriptor_sha256, frames.imu.descriptor_sha256}) == 1
    assert {frames.lidar.world_generation, frames.rtk.world_generation, frames.imu.world_generation} == {1}


def test_v2_sensor_frame_publisher_sends_each_encoded_frame_once(controller) -> None:
    """同帧传感器必须逐条只编码一次，并将原始 bytes 交给固定 topic。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    frame_module = import_module("slope_sim.interfaces.v2.sensor_frames")
    publisher_type = getattr(frame_module, "V2OutputFramePublisher", None)
    assert publisher_type is not None, "v2 output frame publisher must exist"
    frames = frame_module.V2SensorFrameFactory(
        controller, StubCenterLidar(), StubStage4Truth()
    ).capture(100_000_000)
    transport = RecordingPublishTransport()
    publisher = publisher_type(transport, descriptor)

    publisher.publish(frames, wall_time=12.5)

    assert [(topic, type_name, sim_time_ns, wall_time) for topic, _payload, type_name, sim_time_ns, wall_time in transport.calls] == [
        ("/sim/lidar/points", "slope_sim.interfaces.v2.LidarPointCloud", 100_000_000, 12.5),
        ("/sim/rtk/state", "slope_sim.interfaces.v2.RtkState", 100_000_000, 12.5),
        ("/sim/imu/attitude", "slope_sim.interfaces.v2.ImuAttitude", 100_000_000, 12.5),
    ]
    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    assert codec.decode_lidar_point_cloud(transport.calls[0][1]) == frames.lidar
    assert codec.decode_rtk_state(transport.calls[1][1]) == frames.rtk
    assert codec.decode_imu_attitude(transport.calls[2][1]) == frames.imu


def test_v2_wheel_state_factory_binds_physics_feedback_to_controller_identity(controller) -> None:
    """wheel state 必须从物理反馈和同轮 authority 生成完整 v2 状态。"""
    module = import_module("slope_sim.interfaces.v2.sensor_frames")
    factory_type = getattr(module, "V2WheelStateFactory", None)
    assert factory_type is not None, "v2 wheel state factory must exist"
    frame = factory_type(controller, "df_mid").build(
        WheelState(10_000_000, (1.5, -1.5), ())
    )

    assert frame.timestamp_ns == 10_000_000
    assert frame.drive_wheel_speed_rad_s == pytest.approx((1.5, -1.5))
    assert frame.steering_wheel_angle_rad == ()
    assert frame.sequence == 0
    assert frame.world_generation == frame.command_generation == 1
    assert frame.robot_model == "df_mid"
    assert frame.command_authority_state.name == "WAITING"
    assert frame.command_peer_count == 0
    assert frame.command_owner_source_id == ""
    assert frame.command_owner_source_session_id == b""


def test_v2_publish_cadence_reuses_exact_240hz_clock_for_100hz_wheel_and_10hz_sensor() -> None:
    """240 次物理步必须恰好得到 100 次 wheel 和 10 次同刻传感器发布。"""
    module = import_module("slope_sim.interfaces.v2.sensor_frames")
    cadence_type = getattr(module, "V2PublishCadence", None)
    assert cadence_type is not None, "v2 publish cadence must exist"
    cadence = cadence_type()
    wheel_deadlines: list[int] = []
    sensor_deadlines: list[int] = []

    for _ in range(240):
        batch = cadence.advance(Fraction(1, 240))
        wheel_deadlines.extend(batch.wheel_timestamps_ns)
        sensor_deadlines.extend(batch.sensor_timestamps_ns)

    assert len(wheel_deadlines) == 100
    assert len(sensor_deadlines) == 10
    assert wheel_deadlines[0] == 10_000_000
    assert wheel_deadlines[-1] == 1_000_000_000
    assert sensor_deadlines == [index * 100_000_000 for index in range(1, 11)]
