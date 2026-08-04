# 企业接口运行时：串联六话题、物理帧钩子、日志与可线性化生命周期。
from __future__ import annotations

from collections.abc import Callable, Collection, Sequence
from contextlib import contextmanager
from dataclasses import dataclass, replace
from fractions import Fraction
import math
from numbers import Real
from threading import Condition, Lock, get_ident, local
import time
from typing import Protocol, runtime_checkable

from slope_sim.interfaces.clock import PeriodicScheduler, SimulationClock
from slope_sim.interfaces.codec import LIDAR_POINT_CLOUD_TYPE_NAME, ProtoCodec
from slope_sim.interfaces.config import ChannelConfig, InterfaceConfig
from slope_sim.interfaces.dashboard_snapshot import (
    InterfaceDashboardSnapshot,
    LidarTopViewFrame,
)
from slope_sim.interfaces.logging import InterfaceEventLogger, InterfaceLogRecord
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelCommandMechanicalLimitError,
    WheelCommandModelMismatchError,
    WheelState,
    validate_wheel_command,
)
from slope_sim.interfaces.status import (
    InterfaceStatusSnapshot,
    RollingFrequency,
    TopicStatus,
)
from slope_sim.interfaces.transport import (
    LocalTransport,
    Subscription,
    Transport,
    TransportSnapshot,
    TransportTopicQuality,
    transport_state_to_command_peer_state,
)
from slope_sim.interfaces.wheel import WheelCommandMailbox, WheelDecision
from slope_sim.lidar_pointcloud import LidarScanResult, MultiLineLidar
from slope_sim.lidar_worker import (
    LidarScanService,
    LidarServiceEvent,
    LidarServiceSnapshot,
    PreparedLidarFrame,
    PreparedLidarPayload,
    world_digest_for_document,
)
from slope_sim.model_registry import RobotModelSpec
from slope_sim.obstacles import ObstacleSnapshot, ObstacleSpec
from slope_sim.scene_config import SceneDocument
from slope_sim.sensor_backend import SensorBackend
from slope_sim.truth_sensors import TruthSensorSuite


_SAFE_STOP_DT = 1.0 / 240.0
_TERMINAL_LOG_TIMEOUT_SEC = 1.0
_SENSOR_FENCE_TIMEOUT_SEC = 0.250
_LIDAR_CLOSE_JOIN_TIMEOUT_SEC = 2.0
_UINT64_MAX = (1 << 64) - 1


@runtime_checkable
class WheelRobotPort(Protocol):
    """运行时依赖的最小机器人轮子控制与反馈端口。"""

    model_spec: RobotModelSpec

    def command_wheel_speeds(
        self,
        drive_wheel_speeds: tuple[float, ...],
        steering_wheel_speeds: tuple[float, ...] = (),
        dt: float = _SAFE_STOP_DT,
    ) -> tuple[float, ...]:
        ...

    def hold_current_steering_and_stop_drive(self, dt: float) -> object:
        ...

    def read_interface_wheel_state(self, timestamp_ns: int) -> WheelState:
        ...


@dataclass
class _TopicTracker:
    """保存单话题累计状态；所有写操作由 runtime 生命周期锁保护。"""

    channel: ChannelConfig
    frequency: RollingFrequency | None
    state: str
    detail: str = ""
    message_count: int = 0
    error_count: int = 0
    dropped_count: int = 0
    latest_timestamp_ns: int | None = None


@dataclass
class _TransportQualityCursor:
    """保存 runtime 已消费的单话题 transport revision 和活动覆盖状态。"""

    revision: int = -1
    error_count: int = 0
    dropped_count: int = 0
    state: str = "active"
    detail: str = ""
    last_error_detail: str | None = None
    last_drop_detail: str | None = None
    peer_connected: bool | None = None


@dataclass(frozen=True)
class _LocalTwist:
    """本地人工目标只保存纯数值，转换在物理主线程执行。"""

    linear: float
    angular: float
    dt: float


@dataclass(frozen=True, slots=True)
class SensorFence:
    """记录可恢复 sensor fence 的原 service 与进入前状态。"""

    _runtime: object
    _service: object | None
    _service_was_ready: bool


def _robot_model(robot: object) -> RobotModelSpec:
    """确认机器人来自注册表并实现运行时所需的轮子端口。"""
    model = getattr(robot, "model_spec", None)
    if not isinstance(model, RobotModelSpec):
        raise ValueError("robot.model_spec must be a RobotModelSpec")
    required_methods = (
        "command_wheel_speeds",
        "hold_current_steering_and_stop_drive",
        "read_interface_wheel_state",
    )
    if any(not callable(getattr(robot, name, None)) for name in required_methods):
        raise ValueError("robot must implement the WheelRobotPort")
    return model


def _error_detail(error: BaseException) -> str:
    """生成稳定且非空的状态错误说明。"""
    message = str(error)
    return f"{type(error).__name__}: {message}" if message else type(error).__name__


def _bounded_failure_detail(stage: str, error: BaseException) -> str:
    """生成不携带任意异常文本的单行有界生命周期诊断。"""
    detail = f"{stage} failed: {type(error).__name__}".replace("\r", " ").replace(
        "\n", " "
    )
    encoded = detail.encode("utf-8", errors="replace")
    if len(encoded) <= 512:
        return encoded.decode("utf-8")
    return encoded[:512].decode("utf-8", errors="ignore")


def _wheel_counts(model: RobotModelSpec) -> tuple[int, int]:
    if model.controller_kind == "differential":
        return 2, 0
    if model.controller_kind == "active_steering":
        return 4, 2
    raise ValueError(f"unsupported controller_kind: {model.controller_kind}")


def _waiting_decision(model: RobotModelSpec) -> WheelDecision:
    drive_count, steering_count = _wheel_counts(model)
    return WheelDecision((0.0,) * drive_count, (0.0,) * steering_count, waiting=True)


def _validate_wheel_state_lengths(state: WheelState, model: RobotModelSpec) -> None:
    drive_count, steering_count = _wheel_counts(model)
    if len(state.drive_wheel_speed_rad_s) != drive_count:
        raise ValueError(f"{model.name} requires {drive_count} drive wheel states")
    if len(state.steering_wheel_angle_rad) != steering_count:
        raise ValueError(f"{model.name} requires {steering_count} steering wheel angles")


def _finite_number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be a finite number")
    return normalized


def _positive_number(name: str, value: object) -> float:
    normalized = _finite_number(name, value)
    if normalized <= 0.0:
        raise ValueError(f"{name} must be positive")
    return normalized


def _close_initialization_resources(
    logger: object | None,
    transport: object,
    sensor_backend: object | None,
) -> None:
    """构造失败时按所有权顺序尽力关闭资源，次生错误不得覆盖原始异常。"""
    for resource in (logger, transport, sensor_backend):
        close_resource = None if resource is None else getattr(resource, "close", None)
        if not callable(close_resource):
            continue
        try:
            close_resource()
        except BaseException:
            pass


class InterfaceRuntime:
    """在物理主线程驱动六话题，并隔离异步命令回调和世界重建。"""

    def __init__(
        self,
        robot: WheelRobotPort,
        *,
        config: InterfaceConfig,
        transport: Transport,
        monotonic: Callable[[], float] = time.monotonic,
        sensor_backend: SensorBackend | None = None,
        scene_document: SceneDocument | None = None,
        logger: InterfaceEventLogger | None = None,
        capture_lidar_top_view: bool = True,
        lidar_scan_service: LidarScanService | None = None,
        lidar_scan_service_factory: Callable[
            [SceneDocument, int, str], LidarScanService
        ]
        | None = None,
    ) -> None:
        if not isinstance(config, InterfaceConfig):
            raise ValueError("config must be an InterfaceConfig")
        if not callable(monotonic):
            raise ValueError("monotonic must be callable")
        if (sensor_backend is None) != (scene_document is None):
            raise ValueError("sensor_backend and scene_document must be provided together")
        if scene_document is not None and not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        if not isinstance(capture_lidar_top_view, bool):
            raise ValueError("capture_lidar_top_view must be a bool")
        if lidar_scan_service_factory is not None and not callable(
            lidar_scan_service_factory
        ):
            raise ValueError("lidar_scan_service_factory must be callable")
        model = _robot_model(robot)

        self._config = config
        self._transport = transport
        self._monotonic = monotonic
        self._capture_lidar_top_view = capture_lidar_top_view
        self._lidar_scan_service: LidarScanService | None = None
        self._lidar_capture_fenced = False
        self._active_sensor_fence: SensorFence | None = None
        self._lidar_service_paused_by_runtime: LidarScanService | None = None
        self._lidar_scan_service_factory = lidar_scan_service_factory
        self._prepared_lidar_service: LidarScanService | None = None
        self._prepared_scene_document: SceneDocument | None = None
        self._prepared_lidar_world_digest: str | None = None
        self._retired_lidar_services: list[LidarScanService] = []
        self._retired_lidar_cleanup_claims: set[int] = set()
        self._retired_lidar_diagnostic_ids: set[int] = set()
        self._condition = Condition()
        self._state = "open"
        self._lifecycle_generation = 0
        self._next_subscription_token = 0
        self._active_subscription_token: int | None = None
        self._in_flight_publishes = 0
        self._callback_context = local()
        self._in_flight_world_operations = 0
        self._world_operation_context = local()
        self._lifecycle_owner_ident: int | None = None
        self._rebuild_candidate_owner_ident: int | None = None
        self._prepare_in_progress = False
        self._rebind_in_progress = False
        self._rebuild_candidate_in_progress = False
        self._rebuild_lidar_candidate_in_progress = False
        self._rebuild_prepared = False
        self._prepared_robot_parked = False
        self._accepting_commands = True
        self._world_ready = True
        self._paused = False
        self._safe_stop_latched = False
        self._safe_stop_mode: str | None = None
        self._command_epoch = 0
        self._timeout_event_epoch: int | None = None
        self._ecal_lifecycle_enabled = False
        self._peer_state = "active"
        self._peer_command_seen = False
        self._auto_fallback_detail = ""
        self._close_error: BaseException | None = None
        self._close_trace: list[str] = []

        self._robot = robot
        self._robot_model = model
        try:
            if lidar_scan_service is not None:
                actual_transport_mode = transport.snapshot().mode
                if type(actual_transport_mode) is not str or actual_transport_mode != "ecal":
                    raise ValueError(
                        "lidar_scan_service requires actual transport mode 'ecal'"
                    )
            self._mailbox = WheelCommandMailbox(
                model,
                timeout_sec=config.command_timeout_sec,
                frequency_window_sec=config.status_window_sec,
            )
            self._clock = SimulationClock()
            self._wheel_scheduler = PeriodicScheduler(config.wheel_state.rate_hz)
            self._local_command_scheduler = PeriodicScheduler(config.wheel_command.rate_hz)
            # 后雷达提前半周期，避免两批射线在同一物理帧集中执行。
            self._sensor_schedulers = {
                config.lidar_front.topic: PeriodicScheduler(config.lidar_front.rate_hz),
                config.lidar_rear.topic: PeriodicScheduler(
                    config.lidar_rear.rate_hz,
                    first_deadline_sec=Fraction(
                        1,
                        2 * config.lidar_rear.rate_hz,
                    ),
                ),
                config.rtk.topic: PeriodicScheduler(config.rtk.rate_hz),
                config.imu.topic: PeriodicScheduler(config.imu.rate_hz),
            }
            self._codec = ProtoCodec()
            self._wheel_frequency = RollingFrequency(config.status_window_sec)
            self._topics = {
                channel.topic: _TopicTracker(
                    channel,
                    (
                        None
                        if channel is config.wheel_command
                        else self._wheel_frequency
                        if channel is config.wheel_state
                        else RollingFrequency(config.status_window_sec)
                    ),
                    "active"
                    if channel in (config.wheel_command, config.wheel_state)
                    else "disconnected",
                    ""
                    if channel in (config.wheel_command, config.wheel_state)
                    else "sensor backend is not configured",
                )
                for channel in config.channels
            }
            self._transport_quality = {
                channel.topic: _TransportQualityCursor()
                for channel in config.channels
            }
            self._last_decision = self._mailbox.decision(now=self._monotonic())
            self._last_wheel_state: WheelState | None = None
            self._latest_wheel_command: WheelCommand | None = None
            self._latest_wheel_command_received_sim_time_ns: int | None = None
            self._latest_lidar_front: LidarPointCloud | None = None
            self._latest_lidar_rear: LidarPointCloud | None = None
            self._latest_rtk: RtkState | None = None
            self._latest_imu: ImuAttitude | None = None
            self._latest_lidar_front_view: LidarTopViewFrame | None = None
            self._latest_lidar_rear_view: LidarTopViewFrame | None = None
            self._local_twist: _LocalTwist | None = None
            self._connection_polls = 0

            self._logger = logger
            self._log_lock = Lock()
            self._log_sequence = 0
            self._last_log_wall_time_ns = 0
            self._pending_logger_drops = 0
            self._sensor_backend = sensor_backend
            self._scene_document = scene_document
            self._front_lidar: MultiLineLidar | None = None
            self._rear_lidar: MultiLineLidar | None = None
            self._truth_sensor_suite: TruthSensorSuite | None = None
            self._command_subscription: Subscription | None = None
            if sensor_backend is not None and scene_document is not None:
                sensors = self._build_sensor_objects(sensor_backend, scene_document)
                self._install_sensor_objects(*sensors)
            subscription, subscription_token = self._subscribe_wheel_command()
            self._command_subscription = subscription
            self._active_subscription_token = subscription_token
            self._lidar_scan_service = lidar_scan_service
        except BaseException:
            # 构造尚未成功，不伪造正常 close_trace，也不能触碰机器人控制端口。
            _close_initialization_resources(logger, transport, sensor_backend)
            raise

    @classmethod
    def local_for_robot(
        cls,
        robot: WheelRobotPort,
        *,
        config: InterfaceConfig | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        sensor_backend: SensorBackend | None = None,
        scene_document: SceneDocument | None = None,
        logger: InterfaceEventLogger | None = None,
        capture_lidar_top_view: bool = True,
    ) -> "InterfaceRuntime":
        """为机器人创建只使用进程内传输的运行时。"""
        selected = InterfaceConfig.default(transport_mode="local") if config is None else config
        if not isinstance(selected, InterfaceConfig):
            raise ValueError("config must be an InterfaceConfig")
        if selected.transport_mode != "local":
            raise ValueError("local_for_robot requires transport_mode='local'")
        transport = LocalTransport(monotonic=monotonic)
        try:
            return cls(
                robot,
                config=selected,
                transport=transport,
                monotonic=monotonic,
                sensor_backend=sensor_backend,
                scene_document=scene_document,
                logger=logger,
                capture_lidar_top_view=capture_lidar_top_view,
            )
        except BaseException:
            try:
                transport.close()
            except BaseException:
                pass
            raise

    @property
    def config(self) -> InterfaceConfig:
        return self._config

    @property
    def robot_model(self) -> RobotModelSpec:
        with self._condition:
            return self._robot_model

    @property
    def clock(self) -> SimulationClock:
        return self._clock

    @property
    def last_decision(self) -> WheelDecision:
        with self._condition:
            return self._last_decision

    @property
    def last_wheel_state(self) -> WheelState | None:
        with self._condition:
            return self._last_wheel_state

    @property
    def paused(self) -> bool:
        with self._condition:
            return self._paused

    @property
    def connection_polls(self) -> int:
        with self._condition:
            return self._connection_polls

    @property
    def bound_robot_id(self) -> int | None:
        with self._condition:
            robot_id = getattr(self._robot, "robot_id", None)
            return robot_id if isinstance(robot_id, int) and not isinstance(robot_id, bool) else None

    @property
    def scene_document(self) -> SceneDocument | None:
        with self._condition:
            return self._scene_document

    @property
    def close_trace(self) -> tuple[str, ...]:
        with self._condition:
            return tuple(self._close_trace)

    def _require_open_locked(self) -> None:
        if self._state != "open":
            raise RuntimeError(f"interface runtime is {self._state}")

    def _require_rebind_ready_locked(self) -> None:
        """仅允许 wheel-only rebind 在线且未参与世界事务时继续。"""
        self._require_open_locked()
        if (
            not self._world_ready
            or self._prepare_in_progress
            or self._rebind_in_progress
            or self._rebuild_prepared
        ):
            raise RuntimeError("world rebuild is in progress")

    def _enter_interface_callback(self, kind: str) -> int:
        """登记当前线程的接口回调类别，供生命周期入口识别同步重入。"""
        attribute = f"{kind}_depth"
        previous_depth = getattr(self._callback_context, attribute, 0)
        setattr(self._callback_context, attribute, previous_depth + 1)
        return previous_depth

    def _leave_interface_callback(self, kind: str, previous_depth: int) -> None:
        """恢复进入接口回调前的线程局部深度。"""
        attribute = f"{kind}_depth"
        if previous_depth == 0:
            delattr(self._callback_context, attribute)
        else:
            setattr(self._callback_context, attribute, previous_depth)

    def _in_interface_callback(self, *kinds: str) -> bool:
        """判断当前线程是否位于指定接口回调；省略类别时检查全部类别。"""
        selected = kinds or ("publish", "receive", "logger")
        return any(
            getattr(self._callback_context, f"{kind}_depth", 0) > 0
            for kind in selected
        )

    def _reject_lifecycle_owner_reentry_locked(self, operation: str) -> None:
        """生命周期 owner 调用外部端口时，同线程嵌套入口必须立即失败。"""
        current_ident = get_ident()
        if (
            self._lifecycle_owner_ident == current_ident
            or self._rebuild_candidate_owner_ident == current_ident
        ):
            raise RuntimeError(
                f"{operation} cannot reenter an active lifecycle transition"
            )

    def _build_sensor_objects(
        self,
        backend: SensorBackend,
        document: SceneDocument,
    ) -> tuple[MultiLineLidar, MultiLineLidar, TruthSensorSuite]:
        """完全构造新传感器组后再交给生命周期事务提交。"""
        mounts = document.sensors.mounts
        lidar_config = document.sensors.lidar
        front = MultiLineLidar(
            backend,
            lidar_config,
            mounts.lidar_front,
            frame_id="lidar_front",
            lidar_id=1,
        )
        rear = MultiLineLidar(
            backend,
            lidar_config,
            mounts.lidar_rear,
            frame_id="lidar_rear",
            lidar_id=2,
        )
        return front, rear, TruthSensorSuite(backend, mounts)

    def _install_sensor_objects(
        self,
        front: MultiLineLidar,
        rear: MultiLineLidar,
        truth: TruthSensorSuite,
    ) -> None:
        self._front_lidar = front
        self._rear_lidar = rear
        self._truth_sensor_suite = truth
        for channel in (
            self._config.lidar_front,
            self._config.lidar_rear,
            self._config.rtk,
            self._config.imu,
        ):
            tracker = self._topics[channel.topic]
            tracker.state = "active"
            tracker.detail = ""

    def _subscribe_wheel_command(self) -> tuple[Subscription, int]:
        """为每次订阅尝试分配永不复用的 token，返回前候选保持 inactive。"""
        with self._condition:
            subscription_token = self._next_subscription_token
            self._next_subscription_token += 1
        command_type = self._codec.type_name(WheelCommand(0, (), ()))

        def receive(payload: bytes, received_at: float) -> bool | None:
            previous_depth = self._enter_interface_callback("receive")
            try:
                return self._on_wheel_command_payload(
                    payload,
                    received_at,
                    subscription_token=subscription_token,
                )
            finally:
                self._leave_interface_callback("receive", previous_depth)

        return (
            self._transport.subscribe(
                self._config.wheel_command.topic,
                command_type,
                receive,
            ),
            subscription_token,
        )

    def _on_wheel_command_payload(
        self,
        payload: bytes,
        received_at: float,
        *,
        subscription_token: int,
    ) -> bool | None:
        """锁内捕获旧邮箱和 token，解析后只向该捕获引用提交。"""
        with self._condition:
            if (
                self._state != "open"
                or not self._accepting_commands
                or subscription_token != self._active_subscription_token
            ):
                return None
            mailbox = self._mailbox
            model = self._robot_model
            token = mailbox.capture_generation()
            lifecycle_generation = self._lifecycle_generation
            # 接收时刻必须在首次 ingress 守卫内冻结，解码耗时不得改写语义。
            received_sim_time_ns = self._clock.now_ns

        try:
            command = self._codec.decode_wheel_command(payload)
        except Exception as exc:
            with self._condition:
                if (
                    self._state != "open"
                    or not self._accepting_commands
                    or self._lifecycle_generation != lifecycle_generation
                    or self._mailbox is not mailbox
                    or subscription_token != self._active_subscription_token
                ):
                    return None
                self._record_topic_error_locked(self._config.wheel_command.topic, exc)
            self._record_event(
                "protobuf_parse_failed",
                self._config.wheel_command.topic,
                tracker_generation=lifecycle_generation,
                reason=_error_detail(exc),
                topic=self._config.wheel_command.topic,
            )
            return False

        if type(command) is not WheelCommand:
            error = ValueError("command must be an exact WheelCommand")
            with self._condition:
                if (
                    self._state != "open"
                    or not self._accepting_commands
                    or self._lifecycle_generation != lifecycle_generation
                    or self._mailbox is not mailbox
                    or subscription_token != self._active_subscription_token
                ):
                    return None
                self._record_topic_error_locked(self._config.wheel_command.topic, error)
            self._record_event(
                "invalid_command",
                self._config.wheel_command.topic,
                tracker_generation=lifecycle_generation,
                reason=_error_detail(error),
                topic=self._config.wheel_command.topic,
            )
            return False

        # mailbox 负责权威提交；这里仅保留结构化拒绝原因供事件归因。
        rejection_event = "invalid_command"
        try:
            validate_wheel_command(command, model)
        except WheelCommandModelMismatchError:
            rejection_event = "model_mismatch"
        except WheelCommandMechanicalLimitError:
            rejection_event = "mechanical_limit"
        except ValueError:
            pass

        try:
            accepted = mailbox.accept(
                command,
                received_at=received_at,
                generation=token,
            )
        except Exception as exc:
            with self._condition:
                if (
                    self._state != "open"
                    or not self._accepting_commands
                    or self._lifecycle_generation != lifecycle_generation
                    or self._mailbox is not mailbox
                    or subscription_token != self._active_subscription_token
                ):
                    return None
                self._record_topic_error_locked(self._config.wheel_command.topic, exc)
            self._record_event(
                "invalid_command",
                self._config.wheel_command.topic,
                tracker_generation=lifecycle_generation,
                reason=_error_detail(exc),
                topic=self._config.wheel_command.topic,
            )
            return False

        invalid_reason: str | None = None
        with self._condition:
            current = (
                self._state == "open"
                and self._accepting_commands
                and self._lifecycle_generation == lifecycle_generation
                and self._mailbox is mailbox
                and subscription_token == self._active_subscription_token
            )
            if not current:
                return None
            if not accepted:
                invalid_reason = (
                    mailbox.snapshot(now=received_at).last_error
                    or "invalid wheel command"
                )
                self._record_topic_error_locked(
                    self._config.wheel_command.topic,
                    ValueError(invalid_reason),
                )
            else:
                tracker = self._topics[self._config.wheel_command.topic]
                tracker.message_count += 1
                tracker.latest_timestamp_ns = command.timestamp_ns
                tracker.state = "active"
                tracker.detail = ""
                self._latest_wheel_command = command
                self._latest_wheel_command_received_sim_time_ns = received_sim_time_ns
                self._mark_command_accepted_locked()
                self._peer_command_seen = True
                if self._ecal_lifecycle_enabled and self._peer_state == "waiting_peer":
                    self._peer_state = "active"

        if not accepted:
            self._record_event(
                rejection_event,
                self._config.wheel_command.topic,
                tracker_generation=lifecycle_generation,
                reason=invalid_reason or "invalid wheel command",
                topic=self._config.wheel_command.topic,
            )
            return False
        self._record_message(
            self._config.wheel_command.topic,
            "receive",
            command.timestamp_ns,
            self._codec.type_name(command),
            bytes(payload),
            received_at,
            tracker_generation=lifecycle_generation,
        )
        return True

    def accept_local_command(
        self,
        command: WheelCommand,
        *,
        received_at: float | None = None,
    ) -> bool:
        """保留旧测试入口；正式本地 twist 必须走 transport 回环。"""
        with self._condition:
            if not self._accepting_commands:
                raise RuntimeError(
                    f"interface runtime is {self._state} and not accepting commands"
                )
            self._require_open_locked()
            event_time = self._monotonic() if received_at is None else received_at
            if type(command) is not WheelCommand:
                self._record_topic_error_locked(
                    self._config.wheel_command.topic,
                    ValueError("command must be an exact WheelCommand"),
                )
                return False
            # 测试入口与正式 callback 一致，在 mailbox 调用前冻结仿真接收时刻。
            received_sim_time_ns = self._clock.now_ns
            accepted = self._mailbox.accept(command, received_at=event_time)
            if accepted:
                tracker = self._topics[self._config.wheel_command.topic]
                tracker.message_count += 1
                tracker.latest_timestamp_ns = command.timestamp_ns
                tracker.state = "active"
                tracker.detail = ""
                self._latest_wheel_command = command
                self._latest_wheel_command_received_sim_time_ns = received_sim_time_ns
                self._mark_command_accepted_locked()
                self._peer_command_seen = True
                if self._ecal_lifecycle_enabled and self._peer_state == "waiting_peer":
                    self._peer_state = "active"
            else:
                error = self._mailbox.snapshot(now=event_time).last_error or "invalid wheel command"
                self._record_topic_error_locked(
                    self._config.wheel_command.topic,
                    ValueError(error),
                )
            return accepted

    def capture_command_ingress(self) -> tuple[WheelCommandMailbox, int]:
        with self._condition:
            if not self._accepting_commands:
                raise RuntimeError(
                    f"interface runtime is {self._state} and not accepting commands"
                )
            self._require_open_locked()
            return self._mailbox, self._mailbox.capture_generation()

    def submit_local_twist(self, linear: float, angular: float, dt: float) -> bool:
        """仅真实 local transport 接受人工 twist，目标转换留在 before hook。"""
        snapshot = self._transport.snapshot()
        if snapshot.mode != "local":
            return False
        target = _LocalTwist(
            _finite_number("linear", linear),
            _finite_number("angular", angular),
            _positive_number("dt", dt),
        )
        with self._condition:
            self._require_open_locked()
            if not self._accepting_commands:
                raise RuntimeError("interface runtime is not accepting commands")
            self._local_twist = target
        return True

    def poll_transport(self) -> object | None:
        """轮询连接后消费不可变逐话题质量快照，供生产主循环统一归因。"""
        with self._condition:
            self._connection_polls += 1
        poll = getattr(self._transport, "poll_peer_state", None)
        result = poll() if callable(poll) else None
        snapshot = self._transport.snapshot()
        with self._condition:
            lifecycle_enabled = self._ecal_lifecycle_enabled
        snapshot_state = getattr(snapshot, "state", None)
        if lifecycle_enabled and isinstance(snapshot_state, str):
            self.handle_peer_state(
                snapshot_state,
                ecal_connected=snapshot.ecal_connected,
            )
        self.consume_transport_snapshot(snapshot)
        return result

    def consume_transport_snapshot(self, snapshot: TransportSnapshot) -> None:
        """让入口 relay 与周期 poll 复用同一逐话题质量消费边界。"""
        self._consume_transport_snapshot(snapshot)

    def _consume_transport_snapshot(
        self,
        snapshot: TransportSnapshot,
        *,
        allow_closing: bool = False,
        terminal_deadline: float | None = None,
    ) -> None:
        """按 revision 原子合并 transport 累计量，旧并发快照不得回退状态。"""
        if not isinstance(snapshot, TransportSnapshot):
            raise ValueError("transport snapshot must be a TransportSnapshot")
        pending_events: list[tuple[str, str, str, int, int]] = []
        with self._condition:
            if self._state != "open" and not (
                allow_closing and self._state == "closing"
            ):
                return
            generation = self._lifecycle_generation
            sim_time_ns = self._clock.now_ns
            for quality in snapshot.topic_quality:
                if not isinstance(quality, TransportTopicQuality):
                    raise ValueError("transport topic quality has an invalid type")
                cursor = self._transport_quality.get(quality.topic)
                if cursor is None:
                    raise ValueError(f"unknown transport quality topic: {quality.topic}")
                if quality.revision < cursor.revision:
                    continue
                current_signature = (
                    cursor.error_count,
                    cursor.dropped_count,
                    cursor.state,
                    cursor.detail,
                    cursor.last_error_detail,
                    cursor.last_drop_detail,
                    cursor.peer_connected,
                )
                incoming_signature = (
                    quality.error_count,
                    quality.dropped_count,
                    quality.state,
                    quality.detail,
                    quality.last_error_detail,
                    quality.last_drop_detail,
                    quality.peer_connected,
                )
                if quality.revision == cursor.revision:
                    if incoming_signature != current_signature:
                        raise ValueError(
                            f"transport topic quality changed without revision: {quality.topic}"
                        )
                    continue
                if (
                    quality.error_count < cursor.error_count
                    or quality.dropped_count < cursor.dropped_count
                ):
                    raise ValueError(
                        f"transport topic quality counters moved backwards: {quality.topic}"
                    )

                error_delta = quality.error_count - cursor.error_count
                dropped_delta = quality.dropped_count - cursor.dropped_count
                tracker = self._topics[quality.topic]
                tracker.error_count += error_delta
                tracker.dropped_count += dropped_delta
                cursor.revision = quality.revision
                cursor.error_count = quality.error_count
                cursor.dropped_count = quality.dropped_count
                cursor.state = quality.state
                cursor.detail = quality.detail
                cursor.last_error_detail = quality.last_error_detail
                cursor.last_drop_detail = quality.last_drop_detail
                cursor.peer_connected = quality.peer_connected
                if error_delta:
                    pending_events.append(
                        (
                            "publish_failed",
                            quality.topic,
                            quality.last_error_detail
                            or quality.detail
                            or "asynchronous publish failed",
                            sim_time_ns,
                            error_delta,
                        )
                    )
                if dropped_delta:
                    pending_events.append(
                        (
                            "queue_dropped",
                            quality.topic,
                            quality.last_drop_detail
                            or quality.detail
                            or "output queue dropped a message",
                            sim_time_ns,
                            dropped_delta,
                        )
                    )

        for event, topic, reason, event_sim_time_ns, count in pending_events:
            event_fields: dict[str, object] = {
                "topic": topic,
                "reason": reason,
                "sim_time_ns": event_sim_time_ns,
                "count": count,
            }
            if event == "queue_dropped":
                event_fields["source"] = "transport"
            if terminal_deadline is None:
                self._record_event(
                    event,
                    topic,
                    tracker_generation=generation,
                    **event_fields,
                )
            else:
                self._record_terminal_event(
                    event,
                    topic,
                    tracker_generation=generation,
                    deadline=terminal_deadline,
                    **event_fields,
                )

    def initialize_peer_lifecycle(
        self,
        mode: str,
        state: str,
        *,
        detail: str = "",
        ecal_connected: bool | None = None,
    ) -> None:
        """session 接管 transport 后，把首个 eCAL 快照纳入同一生命周期锁。"""
        if mode not in {"local", "ecal"}:
            raise ValueError("mode must be 'local' or 'ecal'")
        if not isinstance(detail, str):
            raise ValueError("detail must be a string")
        if mode == "local":
            if self._config.transport_mode == "auto":
                fallback_detail = detail or "EcalUnavailableError"
                with self._condition:
                    self._require_open_locked()
                    self._auto_fallback_detail = fallback_detail
                self._record_runtime_event(
                    "ecal_disconnected",
                    reason=fallback_detail,
                )
            return
        with self._condition:
            self._require_open_locked()
            self._ecal_lifecycle_enabled = True
        connected = state != "disconnected" if ecal_connected is None else ecal_connected
        self.handle_peer_state(
            state,
            initial=True,
            ecal_connected=connected,
        )
        self._record_runtime_event(
            "ecal_initialized",
            reason=f"eCAL initialized in {state} state",
        )

    def handle_peer_state(
        self,
        state: str,
        *,
        initial: bool = False,
        ecal_connected: bool | None = None,
    ) -> None:
        """原子应用 discovery 边沿；断线同时封锁所有旧代命令。"""
        connected = state != "disconnected" if ecal_connected is None else ecal_connected
        peer_state = transport_state_to_command_peer_state(
            state,
            ecal_connected=connected,
        )
        event: str | None = None
        reason: str | None = None
        with self._condition:
            if self._state != "open" or not self._ecal_lifecycle_enabled:
                return
            previous = self._peer_state
            if peer_state == "disconnected":
                if previous != "disconnected" or initial:
                    self._lifecycle_generation += 1
                    if self._lidar_scan_service is not None:
                        self._lidar_scan_service.invalidate_generation(
                            self._lifecycle_generation
                        )
                    self._mailbox.clear()
                    self._clear_dashboard_payloads_locked()
                    self._local_twist = None
                    self._last_decision = _waiting_decision(self._robot_model)
                    self._safe_stop_latched = False
                    self._safe_stop_mode = None
                self._peer_command_seen = False
                self._peer_state = "disconnected"
                if previous != "disconnected" and not initial:
                    event = "ecal_disconnected"
                    reason = "eCAL peer disconnected"
            elif peer_state == "waiting_peer":
                if previous == "disconnected" or initial:
                    self._peer_command_seen = False
                self._peer_state = "waiting_peer"
                if previous == "disconnected" and not initial:
                    event = "ecal_reconnected"
                    reason = "eCAL peer discovered; waiting for a new command"
            else:
                # transport 的 active 只是候选；当前 runtime 代必须确实收过新命令。
                self._peer_state = "active" if self._peer_command_seen else "waiting_peer"
        if event is not None and reason is not None:
            self._record_runtime_event(event, reason=reason)

    def _publish_local_commands(self, generation: int) -> None:
        with self._condition:
            if self._state != "open" or self._paused or not self._world_ready:
                return
            deadlines = self._local_command_scheduler.pop_due(self._clock.now_ns)
            target = self._local_twist
            robot = self._robot
        if target is None:
            return
        converter = getattr(robot, "wheel_command_from_twist", None)
        if not callable(converter):
            raise ValueError("robot must implement wheel_command_from_twist for local twist")
        for timestamp_ns in deadlines:
            with self._condition:
                if not self._generation_is_publishable_locked(generation):
                    return
            try:
                command = converter(
                    target.linear,
                    target.angular,
                    timestamp_ns=timestamp_ns,
                    dt=target.dt,
                )
            except Exception as exc:
                with self._condition:
                    if self._lifecycle_generation == generation:
                        self._record_topic_error_locked(
                            self._config.wheel_command.topic,
                            exc,
                        )
                continue
            self._publish_message(
                self._config.wheel_command.topic,
                command,
                timestamp_ns,
                generation,
                count_topic=False,
                log_publish=False,
            )

    def before_physics_step(
        self,
        dt: float,
        wall_time: float | None = None,
    ) -> WheelDecision | None:
        """先按 100 Hz 回环本地目标，再消费邮箱并下发本帧轮子控制。"""
        self._poll_lidar_service()
        timeout_event = False
        with self._condition:
            self._require_open_locked()
            if self._paused or not self._world_ready:
                return None
            generation = self._lifecycle_generation
        if self._transport.snapshot().mode == "local":
            self._publish_local_commands(generation)

        with self._condition:
            self._require_open_locked()
            if self._paused or not self._world_ready or generation != self._lifecycle_generation:
                return None
            now = self._monotonic() if wall_time is None else wall_time
            decision = self._mailbox.decision(now=now)
            if decision.waiting or decision.timed_out:
                safe_mode = "waiting" if decision.waiting else "timed_out"
                if not self._safe_stop_latched or self._safe_stop_mode != safe_mode:
                    self._robot.hold_current_steering_and_stop_drive(dt)
                    self._safe_stop_latched = True
                    self._safe_stop_mode = safe_mode
                else:
                    self._robot.command_wheel_speeds(
                        decision.drive_wheel_speed_rad_s,
                        decision.steering_wheel_speed_rad_s,
                        dt=dt,
                    )
            else:
                self._safe_stop_latched = False
                self._safe_stop_mode = None
                self._robot.command_wheel_speeds(
                    decision.drive_wheel_speed_rad_s,
                    decision.steering_wheel_speed_rad_s,
                    dt=dt,
                )
            self._last_decision = decision
            if decision.timed_out:
                self._topics[self._config.wheel_command.topic].state = "timed_out"
                if self._timeout_event_epoch != self._command_epoch:
                    self._timeout_event_epoch = self._command_epoch
                    timeout_event = True
        if timeout_event:
            self._record_runtime_event(
                "command_timeout",
                reason=(
                    "wheel command exceeded "
                    f"{self._config.command_timeout_sec:.3f}s wall-clock timeout"
                ),
                wall_time=now,
            )
        return decision

    def next_physics_step_publish_topics(self, dt: float) -> tuple[str, ...]:
        """只读预览下一物理步跨过期限的可发布输出话题。"""
        with self._condition:
            self._require_open_locked()
            if self._paused or not self._world_ready:
                return ()
            candidate_ns = self._clock.preview_advance(dt)
            topics: list[str] = []
            if self._wheel_scheduler.preview_due(candidate_ns):
                topics.append(self._config.wheel_state.topic)
            sensors_available = (
                self._front_lidar is not None
                and self._rear_lidar is not None
                and self._truth_sensor_suite is not None
            )
            if sensors_available:
                topics.extend(
                    topic
                    for topic, scheduler in self._sensor_schedulers.items()
                    if scheduler.preview_due(candidate_ns)
                    and (
                        self._lidar_scan_service is None
                        or topic
                        not in {
                            self._config.lidar_front.topic,
                            self._config.lidar_rear.topic,
                        }
                    )
                )
            return tuple(topics)

    def after_physics_step(self, dt: float) -> tuple[WheelState, ...]:
        """仅在 Bullet 步进后读取物理反馈，并按独立期限发布五路状态。"""
        with self._condition:
            self._require_open_locked()
            if self._paused or not self._world_ready:
                return ()
            candidate_ns = self._clock.preview_advance(dt)
            wheel_due = self._wheel_scheduler.preview_due(candidate_ns)
            sensor_due = {
                topic: scheduler.preview_due(candidate_ns)
                for topic, scheduler in self._sensor_schedulers.items()
            }
            # 所有 preview 成功后才统一提交，后置 scheduler 失败不会污染前置状态。
            if self._wheel_scheduler.pop_due(candidate_ns) != wheel_due:
                raise RuntimeError("wheel scheduler preview and commit diverged")
            for topic, scheduler in self._sensor_schedulers.items():
                if scheduler.pop_due(candidate_ns) != sensor_due[topic]:
                    raise RuntimeError(f"sensor scheduler preview and commit diverged: {topic}")
            committed_ns = self._clock.advance(dt)
            if committed_ns != candidate_ns:
                raise RuntimeError("simulation clock preview and commit diverged")
            generation = self._lifecycle_generation

        states = self._publish_wheel_deadlines(wheel_due, generation)
        with self._condition:
            sensors_available = (
                self._generation_is_publishable_locked(generation)
                and self._front_lidar is not None
                and self._rear_lidar is not None
                and self._truth_sensor_suite is not None
            )
            front_lidar = self._front_lidar
            rear_lidar = self._rear_lidar
            truth_sensor_suite = self._truth_sensor_suite
            capture_lidar_top_view = self._capture_lidar_top_view
            lidar_scan_service = self._lidar_scan_service
        if sensors_available:
            if front_lidar is None or rear_lidar is None or truth_sensor_suite is None:
                raise RuntimeError("sensor availability snapshot is inconsistent")
            if lidar_scan_service is None:
                routes = (
                    (
                        self._config.lidar_front.topic,
                        lambda timestamp_ns: (
                            self._scan_lidar_for_dashboard(front_lidar, timestamp_ns)
                            if capture_lidar_top_view
                            else self._scan_lidar_message(front_lidar, timestamp_ns)
                        ),
                    ),
                    (
                        self._config.lidar_rear.topic,
                        lambda timestamp_ns: (
                            self._scan_lidar_for_dashboard(rear_lidar, timestamp_ns)
                            if capture_lidar_top_view
                            else self._scan_lidar_message(rear_lidar, timestamp_ns)
                        ),
                    ),
                    (self._config.rtk.topic, truth_sensor_suite.read_rtk),
                    (self._config.imu.topic, truth_sensor_suite.read_imu),
                )
            else:
                capture_routes = (
                    (
                        self._config.lidar_front.topic,
                        front_lidar,
                    ),
                    (
                        self._config.lidar_rear.topic,
                        rear_lidar,
                    ),
                )
                captures_current = True
                for topic, lidar in capture_routes:
                    if not self._capture_lidar_deadlines(
                        topic,
                        sensor_due[topic],
                        lidar,
                        lidar_scan_service,
                        generation,
                    ):
                        captures_current = False
                        break
                routes = (
                    (self._config.rtk.topic, truth_sensor_suite.read_rtk),
                    (self._config.imu.topic, truth_sensor_suite.read_imu),
                ) if captures_current else ()
            for topic, reader in routes:
                if not self._publish_sensor_deadlines(
                    topic,
                    sensor_due[topic],
                    reader,
                    generation,
                ):
                    break
        self._poll_lidar_service()
        return states

    @staticmethod
    def _scan_lidar_for_dashboard(lidar: object, timestamp_ns: int) -> LidarScanResult:
        """雷达必须直接返回同一次扫描原子生成的企业点云与俯视帧。"""
        scan_with_top_view = getattr(lidar, "scan_with_top_view", None)
        if not callable(scan_with_top_view):
            raise TypeError("lidar must implement scan_with_top_view")
        result = scan_with_top_view(timestamp_ns)
        if type(result) is not LidarScanResult:
            raise TypeError("lidar scan_with_top_view must return an exact LidarScanResult")
        return result

    @staticmethod
    def _scan_lidar_message(lidar: object, timestamp_ns: int) -> LidarPointCloud:
        """headless 路径只读取企业点云，禁止隐式构造 Dashboard 俯视数据。"""
        scan = getattr(lidar, "scan", None)
        if not callable(scan):
            raise TypeError("lidar must implement scan")
        message = scan(timestamp_ns)
        if type(message) is not LidarPointCloud:
            raise TypeError("lidar scan must return an exact LidarPointCloud")
        return message

    @staticmethod
    def _snapshot_bodyless_obstacle(obstacle: ObstacleSpec) -> ObstacleSnapshot:
        """把稳定逻辑障碍物复制成 worker 只接受的无物理 ID 快照。"""
        return ObstacleSnapshot(
            logical_id=obstacle.logical_id,
            body_id=None,
            mode=obstacle.mode,
            shape=obstacle.geometry.shape,
            position=obstacle.position,
            orientation=obstacle.orientation,
            path=obstacle.path,
            geometry=obstacle.geometry,
        )

    def _capture_lidar_deadlines(
        self,
        topic: str,
        deadlines: tuple[int, ...],
        lidar: object,
        service: LidarScanService,
        generation: int,
    ) -> bool:
        """在单个 world 屏障内冻结每帧输入，释放屏障后再提交 IPC。"""
        if topic == self._config.lidar_front.topic:
            service_topic = "lidar_front"
        elif topic == self._config.lidar_rear.topic:
            service_topic = "lidar_rear"
        else:
            raise ValueError("async lidar capture requires a configured lidar topic")

        for timestamp_ns in deadlines:
            with self._condition:
                if self._lidar_capture_fenced:
                    return self._generation_is_publishable_locked(generation)
                if not self._begin_world_operation_locked(generation):
                    return False
                backend = self._sensor_backend
                scene_document = self._scene_document
                capture_top_view = self._capture_lidar_top_view
            capture: dict[str, object] | None = None
            capture_error: Exception | None = None
            try:
                world_mount_reader = getattr(lidar, "_world_mount", None)
                if not callable(world_mount_reader):
                    raise TypeError("lidar must provide a frozen world mount")
                world_mount_pose = world_mount_reader()
                optional_base_pose = None
                if capture_top_view:
                    if backend is None:
                        raise RuntimeError("sensor backend is not configured")
                    optional_base_pose = backend.world_pose("base_link")
                if scene_document is None:
                    raise RuntimeError("scene document is not configured")
                snapshots = tuple(
                    self._snapshot_bodyless_obstacle(obstacle)
                    for obstacle in scene_document.obstacles
                )
                capture = {
                    "topic": service_topic,
                    "timestamp_ns": timestamp_ns,
                    "world_mount_pose": world_mount_pose,
                    "optional_base_pose": optional_base_pose,
                    "complete_obstacle_snapshots_without_body_ids": snapshots,
                }
            except Exception as exc:
                capture_error = exc
            finally:
                self._finish_world_operation()

            if capture_error is not None:
                with self._condition:
                    if self._lifecycle_generation != generation:
                        return False
                    self._record_topic_error_locked(topic, capture_error)
                self._record_event(
                    "sensor_failed",
                    topic,
                    tracker_generation=generation,
                    topic=topic,
                    reason=_error_detail(capture_error),
                    sim_time_ns=timestamp_ns,
                )
                continue
            if capture is None:
                raise RuntimeError("lidar capture completed without a snapshot")
            with self._condition:
                if not self._generation_is_publishable_locked(generation):
                    return False
                if self._lidar_capture_fenced:
                    return True
            try:
                service.capture(**capture)
            except Exception as exc:
                with self._condition:
                    if self._lifecycle_generation != generation:
                        return False
                    self._record_topic_error_locked(topic, exc)
                continue
        return True

    def _consume_lidar_service_events(
        self,
        service: LidarScanService,
        events: tuple[LidarServiceEvent, ...],
        *,
        allow_closing: bool = False,
    ) -> None:
        """只按一次性 typed event 更新当前 service 对应的话题质量。"""
        for event in events:
            if type(event) is not LidarServiceEvent:
                raise TypeError("lidar service returned an invalid event")
            if event.scope == "topic":
                if event.optional_topic == "lidar_front":
                    topics = (self._config.lidar_front.topic,)
                elif event.optional_topic == "lidar_rear":
                    topics = (self._config.lidar_rear.topic,)
                else:  # pragma: no cover - LidarServiceEvent 构造器已冻结该合同
                    raise ValueError("lidar service event used an unknown topic")
            else:
                topics = (
                    self._config.lidar_front.topic,
                    self._config.lidar_rear.topic,
                )
            with self._condition:
                state_is_publishable = self._state == "open" or (
                    allow_closing and self._state == "closing"
                )
                if not state_is_publishable or self._lidar_scan_service is not service:
                    return
                generation = self._lifecycle_generation
                for topic in topics:
                    tracker = self._topics[topic]
                    tracker.error_count += 1
                    tracker.dropped_count += 1
                    tracker.state = "error"
                    tracker.detail = event.bounded_detail
                sim_time_ns = (
                    self._clock.now_ns
                    if event.optional_job_identity is None
                    else event.optional_job_identity[4]
                )
            event_fields: dict[str, object] = {
                "scope": event.scope,
                "stable_error_code": event.stable_error_code,
                "service_event_kind": event.kind,
                "service_event_sequence": event.sequence,
                "reason": event.bounded_detail,
                "sim_time_ns": sim_time_ns,
            }
            if event.scope == "topic":
                event_fields["topic"] = topics[0]
            else:
                event_fields["topics"] = topics
            self._record_event(
                "sensor_failed",
                topics[0],
                tracker_generation=generation,
                **event_fields,
            )

    def _poll_lidar_service(self, *, allow_closing: bool = False) -> None:
        """非阻塞收取一帧及其 typed outcomes，并立即走既有发布通道。"""
        with self._condition:
            service = self._lidar_scan_service
            state_is_publishable = self._state == "open" or (
                allow_closing and self._state == "closing"
            )
            if service is None or not state_is_publishable:
                return
        prepared = service.poll()
        drain_events = getattr(service, "drain_events", None)
        events = () if not callable(drain_events) else drain_events()
        if type(events) is not tuple:
            raise TypeError("lidar service drain_events must return an exact tuple")
        self._consume_lidar_service_events(
            service,
            events,
            allow_closing=allow_closing,
        )
        if prepared is not None:
            if type(prepared) is PreparedLidarPayload:
                self._publish_prepared_lidar_payload(
                    prepared,
                    allow_closing=allow_closing,
                )
            else:
                self._publish_prepared_lidar_frame(
                    prepared,
                    allow_closing=allow_closing,
                )

    def begin_sensor_fence(self) -> SensorFence:
        """停止新 capture，并在 250 ms 总预算内排空已捕获帧。"""
        with self._condition:
            self._require_open_locked()
            if self._active_sensor_fence is not None:
                raise RuntimeError("sensor fence is already active")
            service = self._lidar_scan_service
            if service is not None:
                self._lidar_capture_fenced = True
            provisional = SensorFence(self, service, False)
            self._active_sensor_fence = provisional

        if service is None:
            return provisional

        deadline = self._monotonic() + _SENSOR_FENCE_TIMEOUT_SEC
        # snapshot/poll 都可能进入 IPC，不能持有 runtime 生命周期锁。
        snapshot = service.snapshot()
        if self._monotonic() >= deadline:
            raise TimeoutError("sensor fence did not become idle within 250 ms")
        fence = SensorFence(self, service, snapshot.state == "ready")
        with self._condition:
            self._require_open_locked()
            if self._active_sensor_fence is not provisional:
                raise RuntimeError("sensor fence ownership changed during startup")
            self._active_sensor_fence = fence

        while (
            snapshot.in_flight_identity is not None
            or snapshot.pending_capture_identity is not None
        ):
            self._poll_lidar_service()
            snapshot = service.snapshot()
            if self._monotonic() >= deadline:
                raise TimeoutError("sensor fence did not become idle within 250 ms")
            if (
                snapshot.in_flight_identity is None
                and snapshot.pending_capture_identity is None
            ):
                break
        return fence

    def complete_sensor_fence(
        self,
        fence: SensorFence,
        *,
        resume_capture: bool,
    ) -> None:
        """ACK 成功后只为进入前 ready 的同一 service 恢复 capture。"""
        if not isinstance(resume_capture, bool):
            raise ValueError("resume_capture must be a bool")
        with self._condition:
            self._require_open_locked()
            if fence is not self._active_sensor_fence or fence._runtime is not self:
                raise ValueError("sensor fence does not belong to this runtime")
            if (
                resume_capture
                and fence._service_was_ready
                and self._lidar_scan_service is fence._service
            ):
                self._lidar_capture_fenced = False
            self._active_sensor_fence = None

    def _publish_prepared_lidar_frame(
        self,
        frame: object,
        *,
        allow_closing: bool = False,
    ) -> bool:
        """校验 worker 帧身份后直接发布预编码 bytes，不在父进程重新编码。"""
        if type(frame) is not PreparedLidarFrame:
            return False
        if frame.topic == "lidar_front":
            topic = self._config.lidar_front.topic
            expected_lidar_id = 1
        elif frame.topic == "lidar_rear":
            topic = self._config.lidar_rear.topic
            expected_lidar_id = 2
        else:
            return False
        message = frame.message
        top_view = frame.optional_top_view
        if (
            type(message) is not LidarPointCloud
            or message.frame_id != frame.topic
            or message.lidar_id != expected_lidar_id
            or message.timebase_ns != frame.timestamp_ns
            or type(frame.protobuf_payload) is not bytes
            or not frame.protobuf_payload
        ):
            return False
        with self._condition:
            if not self._generation_is_publishable_locked(
                frame.lifecycle_generation,
                allow_closing=allow_closing,
            ):
                return False
            capture_top_view = self._capture_lidar_top_view
        if capture_top_view:
            if type(top_view) is not LidarTopViewFrame:
                return False
            try:
                LidarScanResult(message, top_view)
            except (TypeError, ValueError):
                return False
        elif top_view is not None:
            return False
        try:
            type_name = self._codec.type_name(message)
        except Exception as exc:
            with self._condition:
                if self._lifecycle_generation == frame.lifecycle_generation:
                    self._record_topic_error_locked(topic, exc)
            return False
        return self._publish_encoded_message(
            topic,
            message,
            frame.timestamp_ns,
            frame.lifecycle_generation,
            payload=frame.protobuf_payload,
            type_name=type_name,
            top_view=top_view,
            allow_closing=allow_closing,
        )

    def _publish_prepared_lidar_payload(
        self,
        payload_frame: object,
        *,
        allow_closing: bool = False,
    ) -> bool:
        """headless worker payload 不解码点云，直接复用 encoded publish 内核。"""
        if type(payload_frame) is not PreparedLidarPayload:
            return False
        if payload_frame.topic == "lidar_front":
            topic = self._config.lidar_front.topic
        elif payload_frame.topic == "lidar_rear":
            topic = self._config.lidar_rear.topic
        else:  # pragma: no cover - compact 合同构造器已冻结 topic
            return False
        if type(payload_frame.protobuf_payload) is not bytes or not payload_frame.protobuf_payload:
            return False
        with self._condition:
            if (
                self._capture_lidar_top_view
                or not self._generation_is_publishable_locked(
                    payload_frame.lifecycle_generation,
                    allow_closing=allow_closing,
                )
            ):
                return False
        return self._publish_encoded_message(
            topic,
            None,
            payload_frame.timestamp_ns,
            payload_frame.lifecycle_generation,
            payload=payload_frame.protobuf_payload,
            type_name=LIDAR_POINT_CLOUD_TYPE_NAME,
            store_dashboard_output=False,
            allow_closing=allow_closing,
        )

    def _publish_wheel_deadlines(
        self,
        deadlines: tuple[int, ...],
        generation: int,
    ) -> tuple[WheelState, ...]:
        states: list[WheelState] = []
        for timestamp_ns in deadlines:
            with self._condition:
                if not self._begin_world_operation_locked(generation):
                    break
                robot = self._robot
                model = self._robot_model
            state: WheelState | None = None
            read_error: Exception | None = None
            try:
                state = robot.read_interface_wheel_state(timestamp_ns)
                if not isinstance(state, WheelState):
                    raise TypeError("robot wheel feedback must be a WheelState")
                if state.timestamp_ns != timestamp_ns:
                    raise ValueError(
                        "robot wheel feedback timestamp does not match request"
                    )
                _validate_wheel_state_lengths(state, model)
            except Exception as exc:
                read_error = exc
            finally:
                self._finish_world_operation()
            if read_error is not None:
                with self._condition:
                    if self._lifecycle_generation == generation:
                        self._record_topic_error_locked(
                            self._config.wheel_state.topic,
                            read_error,
                        )
                continue
            if state is None:
                raise RuntimeError("wheel reader completed without a state")

            states.append(state)
            self._publish_message(
                self._config.wheel_state.topic,
                state,
                timestamp_ns,
                generation,
            )
            with self._condition:
                if self._paused or not self._generation_is_publishable_locked(generation):
                    break
        return tuple(states)

    def _publish_sensor_deadlines(
        self,
        topic: str,
        deadlines: tuple[int, ...],
        reader: Callable[[int], object],
        generation: int,
    ) -> bool:
        """单路扫描失败只污染本话题；代际变化则终止整个旧传感器批次。"""
        for timestamp_ns in deadlines:
            with self._condition:
                if not self._begin_world_operation_locked(generation):
                    return False
            reading: object | None = None
            read_error: Exception | None = None
            try:
                reading = reader(timestamp_ns)
            except Exception as exc:
                read_error = exc
            finally:
                self._finish_world_operation()
            if read_error is not None:
                with self._condition:
                    if self._lifecycle_generation != generation:
                        return False
                    self._record_topic_error_locked(topic, read_error)
                self._record_event(
                    "sensor_failed",
                    topic,
                    tracker_generation=generation,
                    topic=topic,
                    reason=_error_detail(read_error),
                    sim_time_ns=timestamp_ns,
                )
                continue
            with self._condition:
                if not self._generation_is_publishable_locked(generation):
                    return False
            if isinstance(reading, LidarScanResult):
                message = reading.message
                top_view = reading.top_view
            else:
                message = reading
                top_view = None
            self._publish_message(
                topic,
                message,
                timestamp_ns,
                generation,
                top_view=top_view,
            )
            with self._condition:
                if not self._generation_is_publishable_locked(generation):
                    return False
        return True

    def _generation_is_publishable_locked(
        self,
        generation: int,
        *,
        allow_closing: bool = False,
    ) -> bool:
        if allow_closing:
            return (
                self._state == "closing"
                and not self._paused
                and self._lifecycle_generation == generation
            )
        return (
            self._state == "open"
            and self._world_ready
            and not self._paused
            and self._lifecycle_generation == generation
        )

    def _generation_can_commit_publish_locked(
        self,
        generation: int,
        *,
        allow_closing: bool = False,
    ) -> bool:
        """允许已完成的 transport 调用提交当前代结果，不受回调内 pause 影响。"""
        if allow_closing:
            return (
                self._state == "closing"
                and self._lifecycle_generation == generation
            )
        return (
            self._state == "open"
            and self._world_ready
            and self._lifecycle_generation == generation
        )

    def _begin_world_operation_locked(
        self,
        generation: int,
        *,
        allow_paused: bool = False,
    ) -> bool:
        """在当前 world 仍可用时登记一次锁外读取或 backend 绑定。"""
        active = (
            self._state == "open"
            and self._world_ready
            and self._lifecycle_generation == generation
            and (allow_paused or not self._paused)
        )
        if not active:
            return False
        self._in_flight_world_operations += 1
        depth = getattr(self._world_operation_context, "depth", 0)
        self._world_operation_context.depth = depth + 1
        return True

    def _finish_world_operation(self) -> None:
        """无论操作成功与否都释放 lifecycle barrier，并唤醒生命周期线程。"""
        previous_depth = getattr(self._world_operation_context, "depth", 0)
        if previous_depth <= 0:
            raise RuntimeError("world-operation context depth became invalid")
        try:
            with self._condition:
                self._in_flight_world_operations -= 1
                self._condition.notify_all()
                if self._in_flight_world_operations < 0:
                    raise RuntimeError("in-flight world-operation counter became negative")
        finally:
            if previous_depth == 1:
                del self._world_operation_context.depth
            else:
                self._world_operation_context.depth = previous_depth - 1

    def _publish_message(
        self,
        topic: str,
        message: object,
        timestamp_ns: int,
        generation: int,
        *,
        count_topic: bool = True,
        log_publish: bool = True,
        top_view: LidarTopViewFrame | None = None,
    ) -> bool:
        """发布前后核对代际，成功后才统计并提交二进制日志。"""
        with self._condition:
            if not self._generation_is_publishable_locked(generation):
                return False
        try:
            payload = self._codec.encode(message)
            type_name = self._codec.type_name(message)
        except Exception as exc:
            with self._condition:
                if self._lifecycle_generation == generation:
                    self._record_topic_error_locked(topic, exc)
            return False
        return self._publish_encoded_message(
            topic,
            message,
            timestamp_ns,
            generation,
            payload=payload,
            type_name=type_name,
            count_topic=count_topic,
            log_publish=log_publish,
            top_view=top_view,
        )

    def _publish_encoded_message(
        self,
        topic: str,
        message: object | None,
        timestamp_ns: int,
        generation: int,
        *,
        payload: bytes,
        type_name: str,
        count_topic: bool = True,
        log_publish: bool = True,
        top_view: LidarTopViewFrame | None = None,
        store_dashboard_output: bool = True,
        allow_closing: bool = False,
    ) -> bool:
        """复用同一发布线性化内核提交已编码消息及其 Dashboard 副本。"""
        try:
            publish_time = self._monotonic()
        except Exception as exc:
            with self._condition:
                if self._lifecycle_generation == generation:
                    self._record_topic_error_locked(topic, exc)
            return False
        # 第二次检查与登记必须原子；transport 调用本身始终在 lifecycle 锁外。
        with self._condition:
            if not self._generation_is_publishable_locked(
                generation,
                allow_closing=allow_closing,
            ):
                return False
            self._in_flight_publishes += 1
        previous_publish_depth = self._enter_interface_callback("publish")

        publish_error: Exception | None = None
        try:
            try:
                published = self._transport.publish(
                    topic,
                    payload,
                    type_name,
                    timestamp_ns,
                    wall_time=publish_time,
                )
                if published is not True:
                    raise RuntimeError("transport publish returned False")
            except Exception as exc:
                publish_error = exc

            if publish_error is not None:
                with self._condition:
                    publish_still_current = self._generation_can_commit_publish_locked(
                        generation,
                        allow_closing=allow_closing,
                    )
                    if publish_still_current:
                        self._record_topic_error_locked(topic, publish_error)
                self._record_event(
                    "publish_failed",
                    topic,
                    tracker_generation=generation,
                    topic=topic,
                    reason=_error_detail(publish_error),
                    sim_time_ns=timestamp_ns,
                )
                return False

            with self._condition:
                publish_still_current = self._generation_can_commit_publish_locked(
                    generation,
                    allow_closing=allow_closing,
                )
                if publish_still_current and count_topic:
                    tracker = self._topics[topic]
                    tracker.message_count += 1
                    tracker.latest_timestamp_ns = timestamp_ns
                    if store_dashboard_output:
                        self._store_dashboard_output_locked(topic, message, top_view)
                    try:
                        frequency = self._wheel_frequency if topic == self._config.wheel_state.topic else tracker.frequency
                        if frequency is None:
                            raise RuntimeError("publish topic has no frequency tracker")
                        frequency.record(publish_time)
                    except Exception as exc:
                        self._record_topic_error_locked(topic, exc)
                    else:
                        tracker.state = "active"
                        tracker.detail = ""

            if log_publish:
                self._record_message(
                    topic,
                    "publish",
                    timestamp_ns,
                    type_name,
                    payload,
                    publish_time,
                    tracker_generation=generation,
                    allow_closing=allow_closing,
                )
            return publish_still_current
        finally:
            try:
                with self._condition:
                    self._in_flight_publishes -= 1
                    self._condition.notify_all()
                    if self._in_flight_publishes < 0:
                        raise RuntimeError("in-flight publish counter became negative")
            finally:
                self._leave_interface_callback("publish", previous_publish_depth)

    def _store_dashboard_output_locked(
        self,
        topic: str,
        message: object,
        top_view: LidarTopViewFrame | None,
    ) -> None:
        """仅在成功发布的当前代线性化点保存对应 Dashboard latest。"""
        if topic == self._config.wheel_state.topic:
            if not isinstance(message, WheelState):
                raise TypeError("wheel state topic must publish WheelState")
            self._last_wheel_state = message
        elif topic == self._config.lidar_front.topic:
            if not isinstance(message, LidarPointCloud):
                raise TypeError("front lidar topic must publish a cloud")
            if self._capture_lidar_top_view:
                if not isinstance(top_view, LidarTopViewFrame):
                    raise TypeError("front lidar topic must publish a cloud and top view")
                self._latest_lidar_front = message
                self._latest_lidar_front_view = top_view
            elif top_view is not None:
                raise TypeError("headless front lidar must not publish a top view")
        elif topic == self._config.lidar_rear.topic:
            if not isinstance(message, LidarPointCloud):
                raise TypeError("rear lidar topic must publish a cloud")
            if self._capture_lidar_top_view:
                if not isinstance(top_view, LidarTopViewFrame):
                    raise TypeError("rear lidar topic must publish a cloud and top view")
                self._latest_lidar_rear = message
                self._latest_lidar_rear_view = top_view
            elif top_view is not None:
                raise TypeError("headless rear lidar must not publish a top view")
        elif topic == self._config.rtk.topic:
            if not isinstance(message, RtkState):
                raise TypeError("RTK topic must publish RtkState")
            self._latest_rtk = message
        elif topic == self._config.imu.topic:
            if not isinstance(message, ImuAttitude):
                raise TypeError("IMU topic must publish ImuAttitude")
            self._latest_imu = message

    def _record_topic_error_locked(self, topic: str, error: BaseException) -> None:
        tracker = self._topics[topic]
        tracker.error_count += 1
        tracker.state = "error"
        tracker.detail = _error_detail(error)

    def _new_log_record(
        self,
        topic: str,
        direction: str,
        sim_time_ns: int,
        type_name: str,
        payload: bytes,
        wall_time: float,
    ) -> InterfaceLogRecord:
        with self._condition:
            sequence = self._log_sequence
            self._log_sequence += 1
            wall_time_ns = max(self._last_log_wall_time_ns, round(wall_time * 1_000_000_000))
            wall_time_ns = min(_UINT64_MAX, wall_time_ns)
            self._last_log_wall_time_ns = wall_time_ns
        return InterfaceLogRecord(
            sequence,
            topic,
            direction,
            sim_time_ns,
            wall_time_ns,
            type_name,
            payload,
        )

    def _record_message(
        self,
        topic: str,
        direction: str,
        sim_time_ns: int,
        type_name: str,
        payload: bytes,
        wall_time: float,
        *,
        tracker_generation: int | None = None,
        allow_closing: bool = False,
    ) -> None:
        logger = self._logger
        if logger is None:
            return
        # 聚合事件先于下一条普通记录；两次提交都保持非阻塞。
        with self._log_lock:
            # 等待日志锁期间 close 可能已完成；旧代消息不得触碰终结后的 logger。
            with self._condition:
                state_is_loggable = self._state == "open" or (
                    allow_closing and self._state == "closing"
                )
                if tracker_generation is not None and (
                    self._lifecycle_generation != tracker_generation
                    or not state_is_loggable
                ):
                    return
            self._flush_pending_logger_drops_locked(logger, terminal=False)
            previous_log_depth = self._enter_interface_callback("logger")
            try:
                try:
                    accepted = logger.record_message(
                        self._new_log_record(
                            topic,
                            direction,
                            sim_time_ns,
                            type_name,
                            payload,
                            wall_time,
                        )
                    )
                except Exception:
                    accepted = False
            finally:
                self._leave_interface_callback("logger", previous_log_depth)
            if accepted is not True:
                self._pending_logger_drops += 1
        if accepted is not True:
            with self._condition:
                if (
                    tracker_generation is not None
                    and self._lifecycle_generation != tracker_generation
                ):
                    return
                tracker = self._topics[topic]
                tracker.dropped_count += 1
                tracker.state = "degraded"
                tracker.detail = "interface logger rejected message"

    def _record_event(
        self,
        event: str,
        tracker_topic: str,
        *,
        tracker_generation: int | None = None,
        **fields: object,
    ) -> None:
        logger = self._logger
        if logger is None:
            return
        # 所有生产事件在同一边界补齐当前场景上下文，显式事件时间不得被覆盖。
        with self._condition:
            if (
                tracker_generation is not None
                and self._lifecycle_generation != tracker_generation
            ):
                return
            wall_time_ns = min(
                _UINT64_MAX,
                max(0, round(self._monotonic() * 1_000_000_000)),
            )
            defaults: dict[str, object] = {
                "wall_time_ns": wall_time_ns,
                "sim_time_ns": self._clock.now_ns,
                "robot_model": self._robot_model.name,
                "terrain_model": (
                    self._scene_document.terrain.terrain_model
                    if self._scene_document is not None
                    else "unbound"
                ),
                "topic": tracker_topic,
                "reason": event,
            }
        event_fields = dict(fields)
        for name, value in defaults.items():
            event_fields.setdefault(name, value)
        # 写锁等待期间可能发生重建；logger 调用前必须再次线性化代际。
        with self._log_lock:
            with self._condition:
                if (
                    tracker_generation is not None
                    and self._lifecycle_generation != tracker_generation
                ):
                    return
            self._flush_pending_logger_drops_locked(logger, terminal=False)
            previous_log_depth = self._enter_interface_callback("logger")
            try:
                try:
                    accepted = logger.record_event(event, **event_fields)
                except Exception:
                    accepted = False
            finally:
                self._leave_interface_callback("logger", previous_log_depth)
            if accepted is not True:
                self._pending_logger_drops += 1
        if accepted is not True:
            with self._condition:
                if (
                    tracker_generation is not None
                    and self._lifecycle_generation != tracker_generation
                ):
                    return
                tracker = self._topics[tracker_topic]
                tracker.dropped_count += 1
                tracker.state = "degraded"
                tracker.detail = "interface logger rejected event"

    def _record_terminal_event(
        self,
        event: str,
        tracker_topic: str,
        *,
        tracker_generation: int | None,
        deadline: float,
        **fields: object,
    ) -> bool:
        """关闭期按共享真实时钟预算提交质量事件，拒绝不得递归累计。"""
        logger = self._logger
        if logger is None:
            return True
        with self._condition:
            if (
                tracker_generation is not None
                and self._lifecycle_generation != tracker_generation
            ):
                return False
            wall_time_ns = min(
                _UINT64_MAX,
                max(0, round(self._monotonic() * 1_000_000_000)),
            )
            defaults: dict[str, object] = {
                "wall_time_ns": wall_time_ns,
                "sim_time_ns": self._clock.now_ns,
                "robot_model": self._robot_model.name,
                "terrain_model": (
                    self._scene_document.terrain.terrain_model
                    if self._scene_document is not None
                    else "unbound"
                ),
                "topic": tracker_topic,
                "reason": event,
            }
        event_fields = dict(fields)
        for name, value in defaults.items():
            event_fields.setdefault(name, value)

        # transport 归因先占终态容量；普通 logger 聚合留到所有 topic 之后。
        with self._log_lock:
            with self._condition:
                if (
                    tracker_generation is not None
                    and self._lifecycle_generation != tracker_generation
                ):
                    return False
            timeout_sec = max(0.0, deadline - time.monotonic())
            previous_log_depth = self._enter_interface_callback("logger")
            try:
                try:
                    return (
                        logger.record_terminal_event(
                            event,
                            timeout_sec=timeout_sec,
                            **event_fields,
                        )
                        is True
                    )
                except Exception:
                    return False
            finally:
                self._leave_interface_callback("logger", previous_log_depth)

    def _flush_pending_logger_drops_locked(
        self,
        logger: InterfaceEventLogger,
        *,
        terminal: bool,
        timeout_sec: float = _TERMINAL_LOG_TIMEOUT_SEC,
    ) -> bool:
        """在日志写锁内提交当前精确聚合；诊断自身拒绝不得递归累计。"""
        count = self._pending_logger_drops
        if count == 0:
            return True
        previous_log_depth = self._enter_interface_callback("logger")
        try:
            try:
                if terminal:
                    accepted = logger.record_terminal_event(
                        "queue_dropped",
                        timeout_sec=timeout_sec,
                        source="interface_logger",
                        count=count,
                    )
                else:
                    accepted = logger.record_event(
                        "queue_dropped",
                        source="interface_logger",
                        count=count,
                    )
            except Exception:
                accepted = False
        finally:
            self._leave_interface_callback("logger", previous_log_depth)
        if accepted is True:
            self._pending_logger_drops -= count
            return True
        return False

    def _record_runtime_event(
        self,
        event: str,
        *,
        reason: str,
        wall_time: float | None = None,
    ) -> None:
        """提交生命周期事件；通用上下文统一由日志边界补齐。"""
        with self._condition:
            generation = self._lifecycle_generation
            fields: dict[str, object] = {
                "topic": self._config.wheel_command.topic,
                "reason": reason,
            }
            if wall_time is not None:
                fields["wall_time_ns"] = min(
                    _UINT64_MAX,
                    max(0, round(wall_time * 1_000_000_000)),
                )
        self._record_event(
            event,
            self._config.wheel_command.topic,
            tracker_generation=generation,
            **fields,
        )

    def _cleanup_retired_lidar_service(
        self,
        service: LidarScanService,
        *,
        report_failure: bool,
    ) -> BaseException | None:
        """串行重试一个 retired worker；成功后才释放 runtime 所有权。"""
        identity = id(service)
        with self._condition:
            while identity in self._retired_lidar_cleanup_claims:
                self._condition.wait()
            if not any(item is service for item in self._retired_lidar_services):
                return None
            self._retired_lidar_cleanup_claims.add(identity)

        error: BaseException | None = None
        try:
            service.close_idle()
        except BaseException as exc:
            error = exc
            should_report = False
            with self._condition:
                if report_failure and identity not in self._retired_lidar_diagnostic_ids:
                    self._retired_lidar_diagnostic_ids.add(identity)
                    should_report = True
            if should_report:
                try:
                    self._record_event(
                        "retired_cleanup_failed",
                        self._config.wheel_command.topic,
                        scope="service",
                        stable_error_code="worker_shutdown_failed",
                        reason=_bounded_failure_detail(
                            "retired lidar service cleanup",
                            exc,
                        ),
                    )
                except BaseException:
                    pass
        else:
            with self._condition:
                self._retired_lidar_services = [
                    item for item in self._retired_lidar_services if item is not service
                ]
                self._retired_lidar_diagnostic_ids.discard(identity)
        finally:
            with self._condition:
                self._retired_lidar_cleanup_claims.discard(identity)
                self._condition.notify_all()
        return error

    def _register_retired_lidar_service_locked(
        self,
        service: LidarScanService,
    ) -> None:
        """在 runtime condition 内登记仍由本 runtime 负责终结的 service。"""
        if not any(item is service for item in self._retired_lidar_services):
            self._retired_lidar_services.append(service)

    def status_snapshot(self, wall_time: float | None = None) -> InterfaceStatusSnapshot:
        """在单一临界区复制六话题、命令邮箱和实际 transport 状态。"""
        with self._condition:
            captured_at = self._monotonic() if wall_time is None else wall_time
            transport = self._transport.snapshot()
            return self._status_snapshot_locked(captured_at, transport)

    def lidar_service_snapshot(self) -> LidarServiceSnapshot | None:
        """返回调用线性化时由 runtime 持有的当前 LiDAR service 诊断。"""
        with self._condition:
            service = self._lidar_scan_service
        if service is None:
            return None
        snapshot = service.snapshot()
        if type(snapshot) is not LidarServiceSnapshot:
            raise TypeError("lidar service snapshot must use the frozen contract")
        return snapshot

    def _status_snapshot_locked(
        self,
        captured_at: float,
        transport: TransportSnapshot,
    ) -> InterfaceStatusSnapshot:
        """调用方持生命周期锁时构造状态，供两种公开快照入口复用。"""
        command = self._mailbox.snapshot(now=captured_at)
        peer_state = self._peer_state if self._ecal_lifecycle_enabled else None
        if self._state != "open" or peer_state == "disconnected":
            command = replace(command, state="disconnected")
        topics: dict[str, TopicStatus] = {}
        for channel in self._config.channels:
            tracker = self._topics[channel.topic]
            if channel is self._config.wheel_command:
                actual_hz = command.valid_hz
                message_count = command.valid_count
                error_count = max(tracker.error_count, command.invalid_count)
            else:
                frequency = (
                    self._wheel_frequency
                    if channel is self._config.wheel_state
                    else tracker.frequency
                )
                if frequency is None:
                    raise RuntimeError("publish topic has no frequency tracker")
                actual_hz = frequency.hz(captured_at)
                message_count = tracker.message_count
                error_count = tracker.error_count
            topic_state = tracker.state if self._state == "open" else "disconnected"
            topic_detail = tracker.detail
            transport_quality = self._transport_quality[channel.topic]
            if self._state == "open" and transport_quality.state != "active":
                topic_state = transport_quality.state
                topic_detail = transport_quality.detail
            if self._state == "open" and channel is self._config.wheel_command:
                if (
                    transport_quality.state == "active"
                    and peer_state in {"waiting_peer", "disconnected"}
                ):
                    topic_state = peer_state
            elif (
                self._state == "open"
                and self._ecal_lifecycle_enabled
                and transport_quality.state == "active"
                and transport_quality.peer_connected is False
                and topic_state not in {"error", "degraded"}
            ):
                topic_state = "waiting_peer"
                topic_detail = "eCAL 话题对端未连接"
            if self._auto_fallback_detail and not topic_detail:
                topic_detail = self._auto_fallback_detail
            topics[channel.topic] = TopicStatus(
                topic=channel.topic,
                direction=channel.direction,
                state=topic_state,
                target_hz=float(channel.rate_hz),
                actual_hz=actual_hz,
                latest_timestamp_ns=tracker.latest_timestamp_ns,
                message_count=message_count,
                error_count=error_count,
                dropped_count=tracker.dropped_count,
                detail=topic_detail,
            )
        return InterfaceStatusSnapshot(
            captured_at=captured_at,
            transport_mode=transport.mode,
            ecal_connected=transport.ecal_connected,
            command=command,
            wheel_state=self._last_wheel_state,
            topics=topics,
        )

    def dashboard_snapshot(
        self,
        wall_time: float | None = None,
    ) -> InterfaceDashboardSnapshot:
        """锁内线性化冻结引用，锁外完成可能遍历满帧点云的模型校验。"""
        with self._condition:
            captured_at = self._monotonic() if wall_time is None else wall_time
            transport = self._transport.snapshot()
            status = self._status_snapshot_locked(captured_at, transport)
            captured = (
                self._lifecycle_generation,
                self._robot_model.name,
                self._clock.now_ns,
                status,
                self._latest_wheel_command,
                self._latest_wheel_command_received_sim_time_ns,
                self._last_wheel_state,
                self._latest_lidar_front,
                self._latest_lidar_rear,
                self._latest_rtk,
                self._latest_imu,
                self._latest_lidar_front_view,
                self._latest_lidar_rear_view,
            )
        (
            generation,
            robot_model,
            sim_time_ns,
            status,
            wheel_command,
            wheel_command_received_sim_time_ns,
            wheel_state,
            lidar_front,
            lidar_rear,
            rtk,
            imu,
            lidar_front_view,
            lidar_rear_view,
        ) = captured
        return InterfaceDashboardSnapshot(
            generation=generation,
            robot_model=robot_model,
            sim_time_ns=sim_time_ns,
            status=status,
            wheel_command=wheel_command,
            wheel_command_received_sim_time_ns=wheel_command_received_sim_time_ns,
            wheel_state=wheel_state,
            lidar_front=lidar_front,
            lidar_rear=lidar_rear,
            rtk=rtk,
            imu=imu,
            lidar_front_view=lidar_front_view,
            lidar_rear_view=lidar_rear_view,
        )

    def pause(self) -> None:
        """先挂起本 runtime 拥有的异步 LiDAR，再冻结物理调度。"""
        with self._condition:
            self._require_open_locked()
            if self._paused:
                return
            service = self._lidar_scan_service
            self._lidar_service_paused_by_runtime = None
            if service is not None and service.snapshot().state == "ready":
                service.pause()
                self._lidar_service_paused_by_runtime = service
            self._paused = True

    def resume(self, wall_time: float | None = None) -> WheelDecision:
        """先刷新安全决定并恢复本次 pause 的 LiDAR，再恢复物理发布。"""
        with self._condition:
            self._require_open_locked()
            now = self._monotonic() if wall_time is None else wall_time
            decision = self._mailbox.decision(now=now)
            service = self._lidar_service_paused_by_runtime
            if service is not None and self._lidar_scan_service is service:
                service.resume()
            self._lidar_service_paused_by_runtime = None
            self._last_decision = decision
            self._paused = False
            return decision

    def prepare_world_rebuild(self) -> None:
        """关闭命令回调入口、清空旧邮箱并安全停止当前机器人。"""
        if self._in_interface_callback():
            raise RuntimeError(
                "prepare_world_rebuild cannot run from an in-flight interface callback"
            )
        if getattr(self._world_operation_context, "depth", 0) > 0:
            raise RuntimeError(
                "prepare_world_rebuild cannot run from an in-flight world operation"
            )
        with self._condition:
            self._reject_lifecycle_owner_reentry_locked("prepare_world_rebuild")
            self._require_open_locked()
            if not self._world_ready:
                raise RuntimeError("world rebuild is already prepared")
            lidar_service = self._lidar_scan_service
            previous_document = self._scene_document
            previous_digest = None
            if lidar_service is not None:
                if previous_document is None:
                    raise RuntimeError("lidar service requires a scene document")
                previous_digest = world_digest_for_document(previous_document)
            self._prepare_in_progress = True
            self._lifecycle_owner_ident = get_ident()
            self._rebuild_prepared = False
            self._prepared_robot_parked = False
            self._accepting_commands = False
            self._world_ready = False
            self._active_subscription_token = None
            subscription = self._command_subscription
            self._command_subscription = None
            robot = self._robot
            if lidar_service is not None:
                self._lidar_capture_fenced = True
            self._prepared_lidar_service = lidar_service
            self._prepared_scene_document = previous_document
            self._prepared_lidar_world_digest = previous_digest

        first_error: BaseException | None = None
        if lidar_service is not None:
            try:
                deadline = self._monotonic() + _SENSOR_FENCE_TIMEOUT_SEC
                safety_deadline = time.monotonic() + _SENSOR_FENCE_TIMEOUT_SEC

                def fence_expired() -> bool:
                    return (
                        self._monotonic() >= deadline
                        or time.monotonic() >= safety_deadline
                    )

                lidar_service.pause()
                snapshot = lidar_service.snapshot()
                if fence_expired():
                    raise TimeoutError(
                        "rebuild lidar service did not become idle within 250 ms"
                    )
                # pause 已撤销 pending；这里只回收不可取消的旧 native 扫描结果。
                while snapshot.in_flight_identity is not None:
                    lidar_service.poll()
                    snapshot = lidar_service.snapshot()
                    if fence_expired():
                        raise TimeoutError(
                            "rebuild lidar service did not become idle within 250 ms"
                        )
                    if snapshot.in_flight_identity is not None:
                        time.sleep(0.001)
            except BaseException as exc:
                first_error = exc

        with self._condition:
            # 已登记读/发布先在旧代退出；world_ready=False 已阻止新任务准入。
            while (
                self._in_flight_world_operations > 0
                or self._in_flight_publishes > 0
            ):
                self._condition.wait()
            self._lifecycle_generation += 1
            lifecycle_generation = self._lifecycle_generation
            if lidar_service is not None:
                try:
                    lidar_service.invalidate_generation(lifecycle_generation)
                except BaseException as exc:
                    if first_error is None:
                        first_error = exc
            self._mailbox.clear()
            self._clear_dashboard_payloads_locked()
            self._begin_new_command_epoch_locked()
            self._last_decision = _waiting_decision(self._robot_model)
            self._local_twist = None
            self._safe_stop_latched = False
            self._safe_stop_mode = None
        # 订阅屏障先等已捕获回调离开，再接触可能被重建的 PyBullet 对象。
        if subscription is not None:
            try:
                subscription.close()
            except BaseException as exc:
                if first_error is None:
                    first_error = exc
        robot_parked = False
        try:
            robot.hold_current_steering_and_stop_drive(_SAFE_STOP_DT)
            robot_parked = True
        except BaseException as exc:
            if first_error is None:
                first_error = exc
        with self._condition:
            self._prepare_in_progress = False
            self._lifecycle_owner_ident = None
            self._rebuild_prepared = True
            self._prepared_robot_parked = robot_parked
            self._condition.notify_all()
        if first_error is not None:
            raise first_error

    def abort_world_rebuild(self) -> None:
        """prepare 失败且旧世界未删除时，恢复旧绑定和全新等待命令态。"""
        if self._in_interface_callback():
            raise RuntimeError(
                "abort_world_rebuild cannot run from an in-flight interface callback"
            )
        lidar_resolution_reserved = False
        with self._condition:
            self._reject_lifecycle_owner_reentry_locked("abort_world_rebuild")
            while self._prepare_in_progress and self._state == "open":
                self._condition.wait()
            self._require_open_locked()
            if self._world_ready and not self._rebuild_prepared:
                return
            if not self._rebuild_prepared or self._world_ready:
                raise RuntimeError("world rebuild is not abortable")
            lifecycle_generation = self._lifecycle_generation
            model = self._robot_model
            prepared_lidar_service = self._prepared_lidar_service
            if prepared_lidar_service is not None:
                if self._rebuild_candidate_in_progress:
                    raise RuntimeError(
                        "world rebuild candidate is already in progress"
                    )
                self._rebuild_candidate_in_progress = True
                self._rebuild_candidate_owner_ident = get_ident()
                lidar_resolution_reserved = True

        try:
            self._abort_world_rebuild_reserved(
                lifecycle_generation,
                model,
                prepared_lidar_service,
            )
        finally:
            if lidar_resolution_reserved:
                with self._condition:
                    self._rebuild_candidate_in_progress = False
                    self._rebuild_candidate_owner_ident = None
                    self._condition.notify_all()

    def _abort_world_rebuild_reserved(
        self,
        lifecycle_generation: int,
        model: RobotModelSpec,
        prepared_lidar_service: LidarScanService | None,
    ) -> None:
        """在 LiDAR 终结 reservation 下恢复 prepare 保存的旧绑定。"""

        if prepared_lidar_service is not None:
            prepared_snapshot = prepared_lidar_service.snapshot()
            if (
                prepared_snapshot.lifecycle_generation != lifecycle_generation
                or prepared_snapshot.state != "suspended"
                or prepared_snapshot.in_flight_identity is not None
                or prepared_snapshot.pending_capture_identity is not None
            ):
                raise RuntimeError(
                    "prepared lidar service must remain suspended and idle"
                )

        mailbox = WheelCommandMailbox(
            model,
            timeout_sec=self._config.command_timeout_sec,
            frequency_window_sec=self._config.status_window_sec,
        )
        decision = mailbox.decision(now=self._monotonic())
        subscription: Subscription | None = None
        try:
            subscription, subscription_token = self._subscribe_wheel_command()
            with self._condition:
                self._require_open_locked()
                if (
                    self._lifecycle_generation != lifecycle_generation
                    or not self._rebuild_prepared
                    or self._world_ready
                ):
                    raise RuntimeError("world changed while aborting rebuild")
                if prepared_lidar_service is not None:
                    prepared_lidar_service.resume()
                self._mailbox = mailbox
                self._reset_command_tracker_locked()
                self._begin_new_command_epoch_locked()
                self._last_decision = decision
                self._clear_dashboard_payloads_locked()
                self._command_subscription = subscription
                self._active_subscription_token = subscription_token
                self._accepting_commands = True
                self._world_ready = True
                self._rebuild_prepared = False
                self._prepared_robot_parked = False
                self._lidar_capture_fenced = False
                self._lidar_service_paused_by_runtime = None
                self._prepared_lidar_service = None
                self._prepared_scene_document = None
                self._prepared_lidar_world_digest = None
                self._safe_stop_latched = False
                self._safe_stop_mode = None
        except BaseException:
            if subscription is not None:
                try:
                    subscription.close()
                except BaseException:
                    pass
            with self._condition:
                if self._state == "open":
                    # 仅本次 abort 仍拥有 prepared 状态时才能写 fault，避免覆盖竞态赢家。
                    if (
                        self._lifecycle_generation == lifecycle_generation
                        and not self._world_ready
                        and self._rebuild_prepared
                    ):
                        self._state = "faulted"
                        self._accepting_commands = False
                        self._world_ready = False
                        self._rebuild_prepared = False
                        self._command_subscription = None
                        self._active_subscription_token = None
                        self._mailbox.clear()
                        self._clear_dashboard_payloads_locked()
                        self._last_decision = _waiting_decision(self._robot_model)
                        self._condition.notify_all()
            raise

    def fault_world_rebuild(self) -> None:
        """物理世界无法恢复时终结 prepared 状态，并保持统一关闭路径可用。"""
        if self._in_interface_callback():
            raise RuntimeError(
                "fault_world_rebuild cannot run from an in-flight interface callback"
            )
        with self._condition:
            self._reject_lifecycle_owner_reentry_locked("fault_world_rebuild")
            while self._prepare_in_progress and self._state == "open":
                self._condition.wait()
            if self._state == "faulted":
                return
            self._require_open_locked()
            if self._world_ready or not self._rebuild_prepared:
                raise RuntimeError("world rebuild is not prepared")
            self._state = "faulted"
            self._accepting_commands = False
            self._world_ready = False
            self._rebuild_prepared = False
            self._command_subscription = None
            self._active_subscription_token = None
            self._mailbox.clear()
            self._clear_dashboard_payloads_locked()
            self._last_decision = _waiting_decision(self._robot_model)
            self._local_twist = None
            self._safe_stop_latched = False
            self._safe_stop_mode = None
            self._condition.notify_all()

    @contextmanager
    def _reserve_world_rebuild_candidate(self):
        """只串行化候选所有权，不在锁内执行 factory 或预热。"""
        with self._condition:
            self._reject_lifecycle_owner_reentry_locked("commit_world_rebuild")
            while self._prepare_in_progress and self._state == "open":
                self._condition.wait()
            self._require_open_locked()
            if not self._rebuild_prepared or self._world_ready:
                raise RuntimeError("prepare_world_rebuild must be called first")
            if self._rebuild_candidate_in_progress:
                raise RuntimeError("world rebuild candidate is already in progress")
            self._rebuild_candidate_in_progress = True
        try:
            yield
        finally:
            with self._condition:
                self._rebuild_candidate_in_progress = False
                self._rebuild_lidar_candidate_in_progress = False
                self._rebuild_candidate_owner_ident = None
                self._condition.notify_all()

    def commit_world_rebuild(
        self,
        new_robot: WheelRobotPort,
        new_backend: SensorBackend,
        new_document: SceneDocument,
    ) -> None:
        """完整构造新邮箱和传感器后，一次恢复世界绑定与命令订阅。"""
        if self._in_interface_callback():
            raise RuntimeError(
                "commit_world_rebuild cannot run from an in-flight interface callback"
            )
        with self._reserve_world_rebuild_candidate():
            self._commit_world_rebuild_reserved(
                new_robot,
                new_backend,
                new_document,
            )

    def _commit_world_rebuild_reserved(
        self,
        new_robot: WheelRobotPort,
        new_backend: SensorBackend,
        new_document: SceneDocument,
    ) -> None:
        """在唯一 reservation 下构建候选并执行原子世界交换。"""
        with self._condition:
            while self._prepare_in_progress and self._state == "open":
                self._condition.wait()
            self._require_open_locked()
            if not self._rebuild_prepared or self._world_ready:
                raise RuntimeError("prepare_world_rebuild must be called first")
            old_backend = self._sensor_backend
            old_lidar_service = self._prepared_lidar_service
            previous_document = self._prepared_scene_document
            previous_digest = self._prepared_lidar_world_digest
            lidar_service_factory = self._lidar_scan_service_factory
            lifecycle_generation = self._lifecycle_generation
        model = _robot_model(new_robot)
        if not isinstance(new_document, SceneDocument):
            raise ValueError("new_document must be a SceneDocument")
        mailbox = WheelCommandMailbox(
            model,
            timeout_sec=self._config.command_timeout_sec,
            frequency_window_sec=self._config.status_window_sec,
        )
        decision = mailbox.decision(now=self._monotonic())
        sensors = self._build_sensor_objects(new_backend, new_document)
        candidate_lidar_service = old_lidar_service
        reuses_previous = False
        if old_lidar_service is not None:
            candidate_digest = world_digest_for_document(new_document)
            reuses_previous = (
                previous_document is not None
                and new_document == previous_document
                and candidate_digest == previous_digest
            )
            if not reuses_previous:
                if lidar_service_factory is None:
                    raise RuntimeError(
                        "lidar worker rebuild requires a service factory"
                    )
                with self._condition:
                    self._require_open_locked()
                    self._rebuild_lidar_candidate_in_progress = True
                    self._rebuild_candidate_owner_ident = get_ident()
                candidate_lidar_service = lidar_service_factory(
                    new_document,
                    lifecycle_generation,
                    candidate_digest,
                )
                if not isinstance(candidate_lidar_service, LidarScanService):
                    raise TypeError(
                        "lidar_scan_service_factory must return LidarScanService"
                    )
                try:
                    candidate_snapshot = candidate_lidar_service.snapshot()
                    if candidate_snapshot.lifecycle_generation != lifecycle_generation:
                        raise RuntimeError(
                            "candidate lidar service generation does not match rebuild"
                        )
                    if (
                        candidate_snapshot.state != "ready"
                        or candidate_snapshot.in_flight_identity is not None
                        or candidate_snapshot.pending_capture_identity is not None
                    ):
                        raise RuntimeError(
                            "candidate lidar service must be ready and idle"
                        )
                except BaseException:
                    with self._condition:
                        self._register_retired_lidar_service_locked(
                            candidate_lidar_service
                        )
                    self._cleanup_retired_lidar_service(
                        candidate_lidar_service,
                        report_failure=True,
                    )
                    raise
            else:
                previous_snapshot = old_lidar_service.snapshot()
                if (
                    previous_snapshot.lifecycle_generation != lifecycle_generation
                    or previous_snapshot.state != "suspended"
                    or previous_snapshot.in_flight_identity is not None
                    or previous_snapshot.pending_capture_identity is not None
                ):
                    raise RuntimeError(
                        "previous lidar service must remain suspended and idle"
                    )
        subscription: Subscription | None = None
        retired_lidar_service: LidarScanService | None = None
        committed = False
        try:
            subscription, subscription_token = self._subscribe_wheel_command()
            with self._condition:
                self._require_open_locked()
                if (
                    self._world_ready
                    or not self._rebuild_prepared
                    or self._lifecycle_generation != lifecycle_generation
                    or self._prepared_lidar_service is not old_lidar_service
                    or self._prepared_scene_document is not previous_document
                    or self._prepared_lidar_world_digest != previous_digest
                ):
                    raise RuntimeError("world changed while committing rebuild")
                # resume 不触碰 child；失败时必须发生在任何新世界字段发布前。
                if reuses_previous and candidate_lidar_service is not None:
                    candidate_lidar_service.resume()
                if (
                    old_lidar_service is not None
                    and candidate_lidar_service is not old_lidar_service
                ):
                    retired_lidar_service = old_lidar_service
                    self._register_retired_lidar_service_locked(old_lidar_service)
                self._robot = new_robot
                self._robot_model = model
                self._mailbox = mailbox
                self._reset_command_tracker_locked()
                self._begin_new_command_epoch_locked()
                self._last_decision = decision
                self._clear_dashboard_payloads_locked()
                self._sensor_backend = new_backend
                self._scene_document = new_document
                self._install_sensor_objects(*sensors)
                self._lidar_scan_service = candidate_lidar_service
                self._lidar_capture_fenced = False
                self._lidar_service_paused_by_runtime = None
                self._command_subscription = subscription
                self._active_subscription_token = subscription_token
                self._accepting_commands = True
                self._world_ready = True
                self._rebuild_prepared = False
                self._prepared_robot_parked = False
                self._prepared_lidar_service = None
                self._prepared_scene_document = None
                self._prepared_lidar_world_digest = None
                committed = True
        except BaseException:
            if subscription is not None:
                try:
                    subscription.close()
                except BaseException:
                    pass
            if retired_lidar_service is not None and not committed:
                with self._condition:
                    self._retired_lidar_services = [
                        item
                        for item in self._retired_lidar_services
                        if item is not retired_lidar_service
                    ]
            if (
                candidate_lidar_service is not None
                and candidate_lidar_service is not old_lidar_service
                and not committed
            ):
                with self._condition:
                    self._register_retired_lidar_service_locked(
                        candidate_lidar_service
                    )
                self._cleanup_retired_lidar_service(
                    candidate_lidar_service,
                    report_failure=True,
                )
            raise
        if retired_lidar_service is not None:
            self._cleanup_retired_lidar_service(
                retired_lidar_service,
                report_failure=True,
            )
        # 新绑定已发布后，旧 backend 的释放失败不能反转成功 commit。
        if old_backend is not None and old_backend is not new_backend:
            close_old_backend = getattr(old_backend, "close", None)
            if callable(close_old_backend):
                try:
                    close_old_backend()
                except BaseException:
                    pass

    def refresh_scene_bindings(
        self,
        terrain_ids: Collection[int],
        snapshots: Sequence[object],
        scene_document: SceneDocument | None = None,
    ) -> None:
        """结构变更后同时提交临时 body 分类和完整无 body-id 逻辑文档。"""
        if scene_document is not None and not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        with self._condition:
            self._require_open_locked()
            if not self._world_ready or self._sensor_backend is None:
                raise RuntimeError("sensor backend is not bound to an active world")
            backend = self._sensor_backend
            generation = self._lifecycle_generation
            if not self._begin_world_operation_locked(
                generation,
                allow_paused=True,
            ):
                raise RuntimeError("sensor backend is not bound to an active world")
        try:
            binder = getattr(backend, "bind_scene", None)
            if not callable(binder):
                raise ValueError("sensor backend must implement bind_scene")
            binder(terrain_ids, snapshots)
        finally:
            self._finish_world_operation()
        with self._condition:
            if self._lifecycle_generation != generation or not self._world_ready:
                raise RuntimeError("world changed while refreshing scene bindings")
            if scene_document is not None:
                self._scene_document = scene_document

    def update_scene_document(self, scene_document: SceneDocument) -> None:
        """移动障碍物推进后只刷新逻辑状态，不重复绑定稳定的 body 分类。"""
        if not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        with self._condition:
            self._require_open_locked()
            if not self._world_ready or self._scene_document is None:
                raise RuntimeError("sensor backend is not bound to an active world")
            if scene_document.robot_model != self._robot_model.name:
                raise ValueError("scene_document robot_model must match the active robot")
            if scene_document.sensors != self._scene_document.sensors:
                raise ValueError("scene_document sensors must match the active binding")
            self._scene_document = scene_document

    def rebind_robot(self, robot: WheelRobotPort) -> None:
        """兼容旧 wheel-only API：保留 transport 并原子替换机器人和邮箱。"""
        if self._in_interface_callback():
            raise RuntimeError(
                "rebind_robot cannot run from an in-flight interface callback"
            )
        if getattr(self._world_operation_context, "depth", 0) > 0:
            raise RuntimeError("rebind_robot cannot run from an in-flight world operation")
        model = _robot_model(robot)
        mailbox = WheelCommandMailbox(
            model,
            timeout_sec=self._config.command_timeout_sec,
            frequency_window_sec=self._config.status_window_sec,
        )
        decision = mailbox.decision(now=self._monotonic())
        with self._condition:
            self._reject_lifecycle_owner_reentry_locked("rebind_robot")
            self._require_rebind_ready_locked()
        candidate_subscription, candidate_token = self._subscribe_wheel_command()
        old_subscription: Subscription | None = None
        old_mailbox: WheelCommandMailbox | None = None
        old_robot: WheelRobotPort | None = None
        old_model: RobotModelSpec | None = None
        robot_parked = False
        try:
            with self._condition:
                self._reject_lifecycle_owner_reentry_locked("rebind_robot")
                self._require_rebind_ready_locked()
                old_accepting_commands = self._accepting_commands
                old_subscription_token = self._active_subscription_token
                self._rebind_in_progress = True
                self._lifecycle_owner_ident = get_ident()
                self._accepting_commands = False
                self._active_subscription_token = None
                self._world_ready = False
                try:
                    # wheel-only rebind 只等待旧世界访问；已进入 transport 的 publish 可自行退场。
                    while self._in_flight_world_operations > 0:
                        self._condition.wait()
                    self._require_open_locked()
                    old_mailbox = self._mailbox
                    old_robot = self._robot
                    old_model = self._robot_model
                    old_subscription = self._command_subscription
                    old_robot.hold_current_steering_and_stop_drive(_SAFE_STOP_DT)
                    robot_parked = True
                    old_mailbox.clear()
                    self._lifecycle_generation += 1
                    self._robot = robot
                    self._robot_model = model
                    self._mailbox = mailbox
                    self._reset_command_tracker_locked()
                    self._begin_new_command_epoch_locked()
                    self._last_decision = decision
                    self._clear_dashboard_payloads_locked()
                    self._local_twist = None
                    self._safe_stop_latched = False
                    self._safe_stop_mode = None
                    self._command_subscription = candidate_subscription
                    self._active_subscription_token = candidate_token
                    self._accepting_commands = True
                    self._world_ready = True
                except BaseException:
                    if not robot_parked:
                        self._accepting_commands = old_accepting_commands
                        self._active_subscription_token = old_subscription_token
                        self._world_ready = True
                    else:
                        # 停车副作用不可回滚；意外提交失败只能恢复旧引用并关闭准入。
                        if old_mailbox is not None:
                            try:
                                old_mailbox.clear()
                            except BaseException:
                                pass
                            self._mailbox = old_mailbox
                        if old_robot is not None:
                            self._robot = old_robot
                        if old_model is not None:
                            self._robot_model = old_model
                        self._command_subscription = old_subscription
                        self._active_subscription_token = None
                        self._accepting_commands = False
                        self._world_ready = False
                        self._state = "faulted"
                        self._lifecycle_generation += 1
                        tracker = self._topics[self._config.wheel_command.topic]
                        tracker.frequency = None
                        tracker.state = "disconnected"
                        tracker.detail = "wheel-only rebind commit failed after safe stop"
                        tracker.message_count = 0
                        tracker.error_count = 0
                        tracker.dropped_count = 0
                        tracker.latest_timestamp_ns = None
                        self._peer_command_seen = False
                        self._latest_wheel_command = None
                        self._latest_wheel_command_received_sim_time_ns = None
                        self._last_wheel_state = None
                        self._latest_lidar_front = None
                        self._latest_lidar_rear = None
                        self._latest_rtk = None
                        self._latest_imu = None
                        self._latest_lidar_front_view = None
                        self._latest_lidar_rear_view = None
                        self._last_decision = _waiting_decision(self._robot_model)
                        self._local_twist = None
                        self._safe_stop_latched = False
                        self._safe_stop_mode = None
                    raise
                finally:
                    self._rebind_in_progress = False
                    self._lifecycle_owner_ident = None
                    self._condition.notify_all()
        except BaseException:
            try:
                candidate_subscription.close()
            except BaseException:
                pass
            raise

        # 原子切换后旧 callback 已因 token 失效，关闭异常不反转新绑定。
        if old_subscription is not None and old_subscription is not candidate_subscription:
            try:
                old_subscription.close()
            except BaseException:
                pass

    def _reset_command_tracker_locked(self) -> None:
        """新邮箱从完整等待态开始，不继承旧命令的时间戳和质量计数。"""
        tracker = self._topics[self._config.wheel_command.topic]
        tracker.frequency = None
        tracker.state = "active"
        tracker.detail = ""
        tracker.message_count = 0
        tracker.error_count = 0
        tracker.dropped_count = 0
        tracker.latest_timestamp_ns = None

    def _clear_dashboard_payloads_locked(self) -> None:
        """generation 失效时一次清空命令、五路输出和两路俯视 latest。"""
        self._latest_wheel_command = None
        self._latest_wheel_command_received_sim_time_ns = None
        self._last_wheel_state = None
        self._latest_lidar_front = None
        self._latest_lidar_rear = None
        self._latest_rtk = None
        self._latest_imu = None
        self._latest_lidar_front_view = None
        self._latest_lidar_rear_view = None

    def _begin_new_command_epoch_locked(self) -> None:
        """重建或重绑后废弃旧 peer 激活依据，等待当前订阅的新命令。"""
        self._peer_command_seen = False
        if self._ecal_lifecycle_enabled and self._peer_state != "disconnected":
            self._peer_state = "waiting_peer"

    def _mark_command_accepted_locked(self) -> None:
        """为每条当前代有效命令分配独立超时事件 epoch。"""
        self._command_epoch += 1

    def close(self) -> None:
        """按固定逻辑顺序尽力释放资源，并让并发关闭者共享首错。"""
        if getattr(self._world_operation_context, "depth", 0) > 0:
            raise RuntimeError("close cannot run from an in-flight world operation")
        in_publish_callback = self._in_interface_callback("publish")
        if self._in_interface_callback("receive", "logger"):
            raise RuntimeError("close cannot run from an in-flight interface callback")
        with self._condition:
            self._reject_lifecycle_owner_reentry_locked("close")
            # 生命周期失败方必须重新竞争 owner；同步 publish 回调不能参与阻塞等待。
            while True:
                if self._state == "closed":
                    return
                if self._state == "closing":
                    if in_publish_callback:
                        raise RuntimeError(
                            "close cannot wait for an active lifecycle transition "
                            "from an in-flight publish callback"
                        )
                    while self._state != "closed":
                        self._condition.wait()
                    if self._close_error is not None:
                        raise self._close_error
                    return
                if (
                    self._prepare_in_progress
                    or self._rebind_in_progress
                    or self._rebuild_lidar_candidate_in_progress
                ):
                    if in_publish_callback:
                        raise RuntimeError(
                            "close cannot wait for an active lifecycle transition "
                            "from an in-flight publish callback"
                        )
                    self._condition.wait()
                    continue
                break
            self._state = "closing"
            self._lifecycle_owner_ident = get_ident()
            closing_lidar_generation = self._lifecycle_generation
            self._accepting_commands = False
            self._active_subscription_token = None
            skip_safe_stop = not self._world_ready and self._prepared_robot_parked
            active_lidar_service = self._lidar_scan_service
            if active_lidar_service is not None:
                self._lidar_capture_fenced = True
            self._world_ready = False
            while self._in_flight_world_operations > 0:
                self._condition.wait()
            self._mailbox.clear()
            self._clear_dashboard_payloads_locked()
            self._last_decision = _waiting_decision(self._robot_model)
            self._safe_stop_latched = False
            self._safe_stop_mode = None
            subscription = self._command_subscription
            self._command_subscription = None
            robot = self._robot
            logger = self._logger
            backend = self._sensor_backend

        first_error: BaseException | None = None

        def run_step(name: str, action: Callable[[], object] | None = None) -> None:
            nonlocal first_error
            with self._condition:
                self._close_trace.append(name)
            if action is None:
                return
            try:
                action()
            except BaseException as exc:
                if first_error is None:
                    first_error = exc

        run_step("stop_commands", None if subscription is None else subscription.close)
        run_step(
            "safe_stop",
            None
            if skip_safe_stop
            else lambda: robot.hold_current_steering_and_stop_drive(_SAFE_STOP_DT),
        )

        def drain_active_lidar_service() -> None:
            """在关闭代仍有效时排空 active worker，并在发布完成后正常 join。"""
            service = active_lidar_service
            if service is None:
                return
            begin_draining = getattr(service, "begin_draining", None)
            snapshot_service = getattr(service, "snapshot", None)
            close_idle = getattr(service, "close_idle", None)
            force_close = getattr(service, "force_close", None)
            if not all(
                callable(method)
                for method in (
                    begin_draining,
                    snapshot_service,
                    close_idle,
                    force_close,
                )
            ):
                return

            normal_close_error: BaseException | None = None
            service_failed_before_close = False
            try:
                begin_draining()
                drain_events = getattr(service, "drain_events", None)
                if callable(drain_events):
                    pending_events = drain_events()
                    if type(pending_events) is not tuple:
                        raise TypeError(
                            "lidar service drain_events must return an exact tuple"
                        )
                    self._consume_lidar_service_events(
                        service,
                        pending_events,
                        allow_closing=True,
                    )
                deadline = self._monotonic() + _SENSOR_FENCE_TIMEOUT_SEC
                hard_deadline = time.monotonic() + _SENSOR_FENCE_TIMEOUT_SEC
                snapshot = snapshot_service()
                service_failed_before_close = snapshot.state == "failed"
                while (
                    snapshot.in_flight_identity is not None
                    or snapshot.pending_capture_identity is not None
                ):
                    if snapshot.state == "failed":
                        raise RuntimeError("lidar service failed while closing")
                    if (
                        self._monotonic() >= deadline
                        or time.monotonic() >= hard_deadline
                    ):
                        raise TimeoutError(
                            "lidar service did not drain within 250 ms during close"
                        )
                    self._poll_lidar_service(allow_closing=True)
                    snapshot = snapshot_service()
                if snapshot.state == "failed":
                    raise RuntimeError("lidar service failed while closing")
                close_idle(timeout_sec=_LIDAR_CLOSE_JOIN_TIMEOUT_SEC)
            except BaseException as exc:
                normal_close_error = exc

            if normal_close_error is not None:
                self._record_event(
                    "worker_shutdown_failed",
                    self._config.wheel_command.topic,
                    tracker_generation=closing_lidar_generation,
                    scope="service",
                    stable_error_code="worker_shutdown_failed",
                    reason=_bounded_failure_detail(
                        "active lidar service shutdown",
                        normal_close_error,
                    ),
                )
                force_error: BaseException | None = None
                try:
                    force_close()
                except BaseException as exc:
                    force_error = exc
                with self._condition:
                    if self._lidar_scan_service is service:
                        self._lidar_scan_service = None
                if force_error is not None:
                    raise normal_close_error from force_error
                if not service_failed_before_close:
                    raise normal_close_error
                return
            with self._condition:
                if self._lidar_scan_service is service:
                    self._lidar_scan_service = None

        def cleanup_lidar_services() -> None:
            first_cleanup_error: BaseException | None = None
            try:
                drain_active_lidar_service()
            except BaseException as exc:
                first_cleanup_error = exc
            with self._condition:
                retired = tuple(self._retired_lidar_services)
            for service in retired:
                cleanup_error = self._cleanup_retired_lidar_service(
                    service,
                    report_failure=False,
                )
                if first_cleanup_error is None and cleanup_error is not None:
                    first_cleanup_error = cleanup_error
            if first_cleanup_error is not None:
                raise first_cleanup_error

        run_step("stop_sensors", cleanup_lidar_services)
        with self._condition:
            # 合法 closing generation 已完全排空后，才统一失效其余旧代输出。
            if self._lifecycle_generation == closing_lidar_generation:
                self._lifecycle_generation += 1
            self._world_ready = False
            self._clear_dashboard_payloads_locked()
        terminal_log_deadline: float | None = None

        def quiesce_transport() -> None:
            nonlocal terminal_log_deadline
            final_snapshot = self._transport.quiesce()
            # quiesce 耗时不应侵占 final snapshot 的终态日志恢复窗口。
            terminal_log_deadline = time.monotonic() + _TERMINAL_LOG_TIMEOUT_SEC
            self._consume_transport_snapshot(
                final_snapshot,
                allow_closing=True,
                terminal_deadline=terminal_log_deadline,
            )

        run_step("quiesce_transport", quiesce_transport)
        if self._ecal_lifecycle_enabled:
            self._record_runtime_event(
                "ecal_closed",
                reason="interface runtime closing",
            )

        def close_logger() -> None:
            if logger is None:
                return
            deadline = terminal_log_deadline
            if deadline is None:
                # transport quiesce 失败时，logger 聚合仍使用独立有界预算。
                deadline = time.monotonic() + _TERMINAL_LOG_TIMEOUT_SEC
            with self._log_lock:
                self._flush_pending_logger_drops_locked(
                    logger,
                    terminal=True,
                    timeout_sec=max(0.0, deadline - time.monotonic()),
                )
                logger.close()

        run_step("close_log", close_logger)
        run_step("close_transport", self._transport.close)
        close_backend = None if backend is None else getattr(backend, "close", None)
        run_step("close_sensors", close_backend if callable(close_backend) else None)

        with self._condition:
            self._close_error = first_error
            self._state = "closed"
            self._lifecycle_owner_ident = None
            self._condition.notify_all()
        if first_error is not None:
            raise first_error
