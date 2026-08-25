"""阶段四 B2：将中心 LiDAR 与三点真值一次投影为同帧 v2 传感器消息。"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real

from slope_sim.interfaces.clock import PeriodicScheduler, SimulationClock
from slope_sim.interfaces.models import ImuAttitude, LidarPointCloud, WheelState
from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.models import (
    ImuAttitudeV2,
    LidarPointCloudV2,
    LidarPointV2,
    Point3dV2,
    RtkStateV2,
    WheelStateV2,
)
from slope_sim.interfaces.v2.runtime_protocol import V2RuntimeProtocol
from slope_sim.interfaces.v2.session import OutputIdentity
from slope_sim.interfaces.v2.topics import V2_OUTPUT_TOPICS


@dataclass(frozen=True)
class V2SensorFrames:
    """一次 10 Hz 采样的中心 LiDAR、三点 RTK 和 IMU 输出。"""

    lidar: LidarPointCloudV2
    rtk: RtkStateV2
    imu: ImuAttitudeV2


@dataclass(frozen=True)
class V2PreparedSensorFrames:
    """child raw bytes、其预留身份与父端冻结的同刻 RTK/IMU 输出。"""

    lidar_payload: bytes
    lidar_timestamp_ns: int
    rtk: RtkStateV2
    imu: ImuAttitudeV2
    lidar_identity: OutputIdentity


@dataclass(frozen=True)
class V2PublishBatch:
    """一次物理步跨过的 v2 wheel 与同步传感器发布期限。"""

    wheel_timestamps_ns: tuple[int, ...]
    sensor_timestamps_ns: tuple[int, ...]


class V2PublishCadence:
    """复用精确通用时钟，固定阶段四 100 Hz wheel 与 10 Hz 传感器频率。"""

    def __init__(self) -> None:
        self._clock = SimulationClock()
        self._wheel = PeriodicScheduler(100)
        self._sensor = PeriodicScheduler(10)

    def advance(self, dt: object) -> V2PublishBatch:
        """预览成功后原子提交一物理步，并返回全部跨越的发布期限。"""
        candidate_ns = self._clock.preview_advance(dt)
        wheel_due = self._wheel.preview_due(candidate_ns)
        sensor_due = self._sensor.preview_due(candidate_ns)
        if self._wheel.pop_due(candidate_ns) != wheel_due:
            raise RuntimeError("v2 wheel scheduler preview and commit diverged")
        if self._sensor.pop_due(candidate_ns) != sensor_due:
            raise RuntimeError("v2 sensor scheduler preview and commit diverged")
        if self._clock.advance(dt) != candidate_ns:
            raise RuntimeError("v2 simulation clock preview and commit diverged")
        return V2PublishBatch(wheel_due, sensor_due)


class V2SensorFrameFactory:
    """复用 B1 真值对象，以单个采样时间构造三条身份绑定 v2 输出。"""

    def __init__(
        self,
        controller: V2RuntimeProtocol,
        lidar: object,
        truth_sensors: object,
    ) -> None:
        if not isinstance(controller, V2RuntimeProtocol):
            raise ValueError("controller must be a V2RuntimeProtocol")
        if not callable(getattr(lidar, "scan", None)):
            raise ValueError("lidar must provide scan")
        if not callable(getattr(truth_sensors, "read_rtk", None)) or not callable(
            getattr(truth_sensors, "read_imu", None)
        ):
            raise ValueError("truth_sensors must provide read_rtk and read_imu")
        self._controller = controller
        self._lidar = lidar
        self._truth_sensors = truth_sensors

    def capture(self, timestamp_ns: int) -> V2SensorFrames:
        """预留同代际 identity 后读取一次 B1 真值，禁止消息跨采样时刻混合。"""
        identities = self._controller.reserve_outputs(V2_OUTPUT_TOPICS[1:])
        lidar_identity, rtk_identity, imu_identity = identities
        scan = self._lidar.scan(timestamp_ns)
        rtk = self._truth_sensors.read_rtk(timestamp_ns)
        imu = self._truth_sensors.read_imu(timestamp_ns)
        from slope_sim.truth_sensors import Stage4RtkState

        if not isinstance(scan, LidarPointCloud):
            raise RuntimeError("center lidar scan must return LidarPointCloud")
        if not isinstance(rtk, Stage4RtkState):
            raise RuntimeError("truth sensor suite must return Stage4RtkState")
        if not isinstance(imu, ImuAttitude):
            raise RuntimeError("truth sensor suite must return ImuAttitude")
        if scan.timebase_ns != rtk.timestamp_ns or scan.timebase_ns != imu.timestamp_ns:
            raise RuntimeError("v2 sensor sample timestamps must match")
        return V2SensorFrames(
            LidarPointCloudV2(
                scan.timebase_ns,
                scan.frame_id,
                scan.point_num,
                scan.lidar_id,
                tuple(
                    LidarPointV2(
                        point.offset_time_ns,
                        point.x,
                        point.y,
                        point.z,
                        point.reflectivity,
                        point.tag,
                        point.line,
                    )
                    for point in scan.points
                ),
                lidar_identity.sequence,
                lidar_identity.world_generation,
                lidar_identity.simulation_session_id,
                lidar_identity.descriptor_sha256,
            ),
            RtkStateV2(
                rtk.timestamp_ns,
                rtk_identity.sequence,
                rtk_identity.world_generation,
                "world",
                Point3dV2(*rtk.left),
                Point3dV2(*rtk.center),
                Point3dV2(*rtk.right),
                rtk.heading_rad,
                rtk_identity.simulation_session_id,
                rtk_identity.descriptor_sha256,
            ),
            ImuAttitudeV2(
                imu.timestamp_ns,
                imu.roll_rad,
                imu.pitch_rad,
                imu_identity.sequence,
                imu_identity.world_generation,
                "base_link",
                imu_identity.simulation_session_id,
                imu_identity.descriptor_sha256,
            ),
        )

    def poll_completed(self) -> tuple[V2PreparedSensorFrames, ...]:
        """同步 factory 没有 child 结果；保留与异步 factory 相同的 runtime 窄接口。"""
        return ()

    def has_pending(self) -> bool:
        """同步 factory 在 capture 返回时已经完成，不需要 runtime drain。"""
        return False


class V2AsyncSensorFrameFactory(V2SensorFrameFactory):
    """通过既有有界 LiDAR service 异步完成中心扫描，物理线程不执行射线查询。"""

    def __init__(
        self,
        controller: V2RuntimeProtocol,
        lidar_service: object,
        truth_sensors: object,
        capture_context: object,
    ) -> None:
        if not isinstance(controller, V2RuntimeProtocol):
            raise ValueError("controller must be a V2RuntimeProtocol")
        if not all(
            callable(getattr(lidar_service, method_name, None))
            for method_name in ("capture", "poll", "drain_events")
        ):
            raise ValueError("lidar_service must provide capture, poll, and drain_events")
        if not callable(getattr(truth_sensors, "read_rtk", None)) or not callable(
            getattr(truth_sensors, "read_imu", None)
        ):
            raise ValueError("truth_sensors must provide read_rtk and read_imu")
        if not callable(capture_context):
            raise ValueError("capture_context must be callable")
        self._controller = controller
        self._lidar_service = lidar_service
        self._truth_sensors = truth_sensors
        self._capture_context = capture_context
        self._pending: dict[int, tuple[OutputIdentity, RtkStateV2, ImuAttitudeV2]] = {}
        self._last_lidar_failure_detail: str | None = None

    def capture(self, timestamp_ns: int) -> None:
        """冻结一份 10 Hz 状态并提交 worker，成功帧稍后由 poll_completed 交付。"""
        identities = self._controller.reserve_outputs(V2_OUTPUT_TOPICS[1:])
        lidar_identity, rtk_identity, imu_identity = identities
        rtk = self._truth_sensors.read_rtk(timestamp_ns)
        imu = self._truth_sensors.read_imu(timestamp_ns)
        from slope_sim.truth_sensors import Stage4RtkState

        if not isinstance(rtk, Stage4RtkState) or not isinstance(imu, ImuAttitude):
            raise RuntimeError("truth sensor suite returned an invalid Stage4 sample")
        if rtk.timestamp_ns != timestamp_ns or imu.timestamp_ns != timestamp_ns:
            raise RuntimeError("v2 async sensor sample timestamps must match")
        if timestamp_ns in self._pending:
            raise RuntimeError("v2 async lidar timestamp is already pending")
        context = self._capture_context()
        if type(context) is not tuple or len(context) != 2:
            raise RuntimeError("capture_context must return world mount pose and snapshots")
        world_mount_pose, obstacle_snapshots = context
        accepted = self._lidar_service.capture(
            topic="lidar_link",
            timestamp_ns=timestamp_ns,
            world_mount_pose=world_mount_pose,
            optional_base_pose=None,
            complete_obstacle_snapshots_without_body_ids=obstacle_snapshots,
            output_identity=lidar_identity,
        )
        if accepted is not True and not self._capture_rejected_by_backpressure():
            raise RuntimeError(
                "Stage4 lidar worker rejected a due capture: "
                + self._capture_rejection_detail()
            )
        if accepted is not True:
            return None
        self._pending[timestamp_ns] = (
            lidar_identity,
            RtkStateV2(
                rtk.timestamp_ns,
                rtk_identity.sequence,
                rtk_identity.world_generation,
                "world",
                Point3dV2(*rtk.left),
                Point3dV2(*rtk.center),
                Point3dV2(*rtk.right),
                rtk.heading_rad,
                rtk_identity.simulation_session_id,
                rtk_identity.descriptor_sha256,
            ),
            ImuAttitudeV2(
                imu.timestamp_ns,
                imu.roll_rad,
                imu.pitch_rad,
                imu_identity.sequence,
                imu_identity.world_generation,
                "base_link",
                imu_identity.simulation_session_id,
                imu_identity.descriptor_sha256,
            ),
        )
        return None

    def _capture_rejected_by_backpressure(self) -> bool:
        """只把健康 service 的两级队列满视为可观测单帧降级。"""
        snapshot_reader = getattr(self._lidar_service, "snapshot", None)
        if not callable(snapshot_reader):
            return False
        snapshot = snapshot_reader()
        return (
            getattr(snapshot, "state", None) == "ready"
            and getattr(snapshot, "in_flight_identity", None) is not None
            and getattr(snapshot, "pending_capture_identity", None) is not None
        )

    def _capture_rejection_detail(self) -> str:
        """保留 service 既有的有界诊断，避免基础设施失败被泛化文案覆盖。"""
        snapshot_reader = getattr(self._lidar_service, "snapshot", None)
        if not callable(snapshot_reader):
            return "state=unknown"
        snapshot = snapshot_reader()
        state = getattr(snapshot, "state", "unknown")
        error_code = getattr(snapshot, "last_error_code", None) or "none"
        error_detail = getattr(snapshot, "last_error_detail", "") or "none"
        detail = f"state={state}, error={error_code}, detail={error_detail}"
        if self._last_lidar_failure_detail is not None:
            detail += f", previous_frame_failure={self._last_lidar_failure_detail}"
        return detail

    def poll_completed(self) -> tuple[V2PreparedSensorFrames, ...]:
        """非阻塞消费至多一条 child payload，禁止父端反序列化或二次编码 LiDAR。"""
        prepared = self._lidar_service.poll()
        self._discard_terminal_lidar_events()
        if prepared is None:
            return ()
        topic = getattr(prepared, "topic", None)
        timestamp_ns = getattr(prepared, "timestamp_ns", None)
        payload = getattr(prepared, "protobuf_payload", None)
        if topic != "lidar_link" or type(timestamp_ns) is not int or type(payload) is not bytes:
            raise RuntimeError("Stage4 lidar worker returned an invalid prepared payload")
        try:
            lidar_identity, rtk, imu = self._pending.pop(timestamp_ns)
        except KeyError as error:
            raise RuntimeError("Stage4 lidar worker returned an unknown timestamp") from error
        return (V2PreparedSensorFrames(payload, timestamp_ns, rtk, imu, lidar_identity),)

    def _discard_terminal_lidar_events(self) -> None:
        """worker 已明确丢弃的扫描必须连同同刻真值一起撤销，避免关闭等待幽灵帧。"""
        events = self._lidar_service.drain_events()
        from slope_sim.lidar_worker import LidarServiceEvent

        if type(events) is not tuple:
            raise RuntimeError("Stage4 lidar worker drain_events must return an exact tuple")
        for event in events:
            if type(event) is not LidarServiceEvent:
                raise RuntimeError("Stage4 lidar worker returned an invalid service event")
            if event.kind != "frame_failed":
                continue
            self._last_lidar_failure_detail = (
                f"{event.stable_error_code}: {event.bounded_detail}"
            )
            identity = event.optional_job_identity
            if (
                event.optional_topic == "lidar_link"
                and identity is not None
                and identity[3] == "lidar_link"
            ):
                self._pending.pop(identity[4], None)

    def has_pending(self) -> bool:
        """供关闭路径等待 child 已返回每个已预留的 10 Hz 采样。"""
        return bool(self._pending)


class V2OutputFramePublisher:
    """把 v2 wheel/传感器模型确定性编码一次，并直接交给 raw transport。"""

    def __init__(self, transport: object, descriptor: DescriptorIdentity) -> None:
        if not callable(getattr(transport, "publish", None)):
            raise ValueError("transport must provide publish")
        if not isinstance(descriptor, DescriptorIdentity):
            raise ValueError("descriptor must be a DescriptorIdentity")
        self._transport = transport
        self._codec = V2ProtoCodec(descriptor)

    def publish(self, frames: V2SensorFrames, *, wall_time: float) -> None:
        """按固定 topic 顺序发送三条已绑定身份的 10 Hz 传感器消息。"""
        if not isinstance(frames, V2SensorFrames):
            raise ValueError("frames must be a V2SensorFrames")
        if isinstance(wall_time, bool) or not isinstance(wall_time, Real) or not math.isfinite(float(wall_time)):
            raise ValueError("wall_time must be finite")
        for topic, model, timestamp_ns in (
            ("/sim/lidar/points", frames.lidar, frames.lidar.timebase_ns),
            ("/sim/rtk/state", frames.rtk, frames.rtk.timestamp_ns),
            ("/sim/imu/attitude", frames.imu, frames.imu.timestamp_ns),
        ):
            encoded = self._codec.encode(model)
            published = self._transport.publish(
                topic,
                encoded.payload,
                encoded.type_name,
                timestamp_ns,
                wall_time=float(wall_time),
            )
            if published is not True:
                raise RuntimeError(f"v2 transport rejected {topic} sensor frame")

    def publish_prepared(self, frames: V2PreparedSensorFrames, *, wall_time: float) -> None:
        """原样发布 child 已编码 LiDAR，并只在父端编码同刻 RTK/IMU。"""
        if not isinstance(frames, V2PreparedSensorFrames):
            raise ValueError("frames must be a V2PreparedSensorFrames")
        if isinstance(wall_time, bool) or not isinstance(wall_time, Real) or not math.isfinite(float(wall_time)):
            raise ValueError("wall_time must be finite")
        if type(frames.lidar_payload) is not bytes or not frames.lidar_payload:
            raise ValueError("prepared lidar payload must be nonempty exact bytes")
        if (
            frames.lidar_timestamp_ns != frames.rtk.timestamp_ns
            or frames.lidar_timestamp_ns != frames.imu.timestamp_ns
        ):
            raise ValueError("prepared v2 sensor timestamps must match")
        published = self._transport.publish(
            "/sim/lidar/points",
            frames.lidar_payload,
            "slope_sim.interfaces.v2.LidarPointCloud",
            frames.lidar_timestamp_ns,
            wall_time=float(wall_time),
        )
        if published is not True:
            raise RuntimeError("v2 transport rejected prepared lidar frame")
        for topic, model, timestamp_ns in (
            ("/sim/rtk/state", frames.rtk, frames.rtk.timestamp_ns),
            ("/sim/imu/attitude", frames.imu, frames.imu.timestamp_ns),
        ):
            encoded = self._codec.encode(model)
            published = self._transport.publish(
                topic,
                encoded.payload,
                encoded.type_name,
                timestamp_ns,
                wall_time=float(wall_time),
            )
            if published is not True:
                raise RuntimeError(f"v2 transport rejected {topic} sensor frame")

    def publish_wheel_state(self, frame: WheelStateV2, *, wall_time: float) -> None:
        """发送一条已绑定 authority 的 wheel state，禁止调用方重编码。"""
        if not isinstance(frame, WheelStateV2):
            raise ValueError("frame must be a WheelStateV2")
        if isinstance(wall_time, bool) or not isinstance(wall_time, Real) or not math.isfinite(float(wall_time)):
            raise ValueError("wall_time must be finite")
        encoded = self._codec.encode(frame)
        published = self._transport.publish(
            "/sim/wheel/state",
            encoded.payload,
            encoded.type_name,
            frame.timestamp_ns,
            wall_time=float(wall_time),
        )
        if published is not True:
            raise RuntimeError("v2 transport rejected wheel state frame")


class V2WheelStateFactory:
    """将物理主线程反馈投影为带当前 authority 回显的 v2 wheel state。"""

    def __init__(self, controller: V2RuntimeProtocol, robot_model: str) -> None:
        if not isinstance(controller, V2RuntimeProtocol):
            raise ValueError("controller must be a V2RuntimeProtocol")
        if not isinstance(robot_model, str) or not robot_model:
            raise ValueError("robot_model must be nonempty")
        self._controller = controller
        self._robot_model = robot_model

    def build(self, feedback: WheelState) -> WheelStateV2:
        """把当前轮子反馈和权属状态绑定到同一 session/world 输出 identity。"""
        if not isinstance(feedback, WheelState):
            raise ValueError("feedback must be a WheelState")
        identity = self._controller.reserve_output("/sim/wheel/state")
        snapshot = self._controller.snapshot()
        if (
            identity.simulation_session_id != snapshot.simulation_session_id
            or identity.descriptor_sha256 != snapshot.descriptor_sha256
            or identity.world_generation != snapshot.world_generation
        ):
            raise RuntimeError("v2 wheel state identity changed during construction")
        authority = snapshot.authority
        return WheelStateV2(
            feedback.timestamp_ns,
            feedback.drive_wheel_speed_rad_s,
            feedback.steering_wheel_angle_rad,
            identity.sequence,
            identity.world_generation,
            snapshot.command_generation,
            self._robot_model,
            identity.simulation_session_id,
            identity.descriptor_sha256,
            authority.state,
            authority.owner_source_id or "",
            authority.owner_source_session_id or b"",
            authority.peer_count,
        )
