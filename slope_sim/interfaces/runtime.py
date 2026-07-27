# 企业接口运行时：串联六话题、物理帧钩子、日志与可线性化生命周期。
from __future__ import annotations

from collections.abc import Callable, Collection, Sequence
from dataclasses import dataclass, replace
import math
from numbers import Real
from threading import Condition, Lock, local
import time
from typing import Protocol, runtime_checkable

from slope_sim.interfaces.clock import PeriodicScheduler, SimulationClock
from slope_sim.interfaces.codec import ProtoCodec
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
from slope_sim.model_registry import RobotModelSpec
from slope_sim.scene_config import SceneDocument
from slope_sim.sensor_backend import SensorBackend
from slope_sim.truth_sensors import TruthSensorSuite


_SAFE_STOP_DT = 1.0 / 240.0
_TERMINAL_LOG_TIMEOUT_SEC = 1.0
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
    ) -> None:
        if not isinstance(config, InterfaceConfig):
            raise ValueError("config must be an InterfaceConfig")
        if not callable(monotonic):
            raise ValueError("monotonic must be callable")
        if (sensor_backend is None) != (scene_document is None):
            raise ValueError("sensor_backend and scene_document must be provided together")
        if scene_document is not None and not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        model = _robot_model(robot)

        self._config = config
        self._transport = transport
        self._monotonic = monotonic
        self._condition = Condition()
        self._state = "open"
        self._lifecycle_generation = 0
        self._next_subscription_token = 0
        self._active_subscription_token: int | None = None
        self._in_flight_publishes = 0
        self._publish_context = local()
        self._prepare_in_progress = False
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
            self._mailbox = WheelCommandMailbox(
                model,
                timeout_sec=config.command_timeout_sec,
                frequency_window_sec=config.status_window_sec,
            )
            self._clock = SimulationClock()
            self._wheel_scheduler = PeriodicScheduler(config.wheel_state.rate_hz)
            self._local_command_scheduler = PeriodicScheduler(config.wheel_command.rate_hz)
            self._sensor_schedulers = {
                config.lidar_front.topic: PeriodicScheduler(config.lidar_front.rate_hz),
                config.lidar_rear.topic: PeriodicScheduler(config.lidar_rear.rate_hz),
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
            or self._rebuild_prepared
        ):
            raise RuntimeError("world rebuild is in progress")

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
            return self._on_wheel_command_payload(
                payload,
                received_at,
                subscription_token=subscription_token,
            )

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
                        self._record_topic_error_locked(self._config.wheel_command.topic, exc)
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
        if sensors_available:
            if front_lidar is None or rear_lidar is None or truth_sensor_suite is None:
                raise RuntimeError("sensor availability snapshot is inconsistent")
            routes = (
                (
                    self._config.lidar_front.topic,
                    lambda timestamp_ns: self._scan_lidar_for_dashboard(
                        front_lidar,
                        timestamp_ns,
                    ),
                ),
                (
                    self._config.lidar_rear.topic,
                    lambda timestamp_ns: self._scan_lidar_for_dashboard(
                        rear_lidar,
                        timestamp_ns,
                    ),
                ),
                (self._config.rtk.topic, truth_sensor_suite.read_rtk),
                (self._config.imu.topic, truth_sensor_suite.read_imu),
            )
            for topic, reader in routes:
                if not self._publish_sensor_deadlines(
                    topic,
                    sensor_due[topic],
                    reader,
                    generation,
                ):
                    break
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

    def _publish_wheel_deadlines(
        self,
        deadlines: tuple[int, ...],
        generation: int,
    ) -> tuple[WheelState, ...]:
        states: list[WheelState] = []
        for timestamp_ns in deadlines:
            with self._condition:
                if not self._generation_is_publishable_locked(generation):
                    break
                robot = self._robot
                model = self._robot_model
            try:
                state = robot.read_interface_wheel_state(timestamp_ns)
                if not isinstance(state, WheelState):
                    raise TypeError("robot wheel feedback must be a WheelState")
                if state.timestamp_ns != timestamp_ns:
                    raise ValueError("robot wheel feedback timestamp does not match request")
                _validate_wheel_state_lengths(state, model)
            except Exception as exc:
                with self._condition:
                    if self._lifecycle_generation == generation:
                        self._record_topic_error_locked(self._config.wheel_state.topic, exc)
                continue

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
                if not self._generation_is_publishable_locked(generation):
                    return False
            try:
                reading = reader(timestamp_ns)
            except Exception as exc:
                with self._condition:
                    if self._lifecycle_generation != generation:
                        return False
                    self._record_topic_error_locked(topic, exc)
                self._record_event(
                    "sensor_failed",
                    topic,
                    tracker_generation=generation,
                    topic=topic,
                    reason=_error_detail(exc),
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

    def _generation_is_publishable_locked(self, generation: int) -> bool:
        return (
            self._state == "open"
            and self._world_ready
            and not self._paused
            and self._lifecycle_generation == generation
        )

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
            publish_time = self._monotonic()
        except Exception as exc:
            with self._condition:
                if self._lifecycle_generation == generation:
                    self._record_topic_error_locked(topic, exc)
            return False

        # 第二次检查与登记必须原子；transport 调用本身始终在 lifecycle 锁外。
        with self._condition:
            if not self._generation_is_publishable_locked(generation):
                return False
            self._in_flight_publishes += 1
        previous_publish_depth = getattr(self._publish_context, "depth", 0)
        self._publish_context.depth = previous_publish_depth + 1

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
                    publish_still_current = (
                        self._lifecycle_generation == generation
                        and self._state == "open"
                        and self._world_ready
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
                publish_still_current = (
                    self._state == "open"
                    and self._lifecycle_generation == generation
                    and self._world_ready
                )
                if publish_still_current and count_topic:
                    tracker = self._topics[topic]
                    tracker.message_count += 1
                    tracker.latest_timestamp_ns = timestamp_ns
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
                if previous_publish_depth == 0:
                    del self._publish_context.depth
                else:
                    self._publish_context.depth = previous_publish_depth

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
            if not isinstance(message, LidarPointCloud) or not isinstance(
                top_view,
                LidarTopViewFrame,
            ):
                raise TypeError("front lidar topic must publish a cloud and top view")
            self._latest_lidar_front = message
            self._latest_lidar_front_view = top_view
        elif topic == self._config.lidar_rear.topic:
            if not isinstance(message, LidarPointCloud) or not isinstance(
                top_view,
                LidarTopViewFrame,
            ):
                raise TypeError("rear lidar topic must publish a cloud and top view")
            self._latest_lidar_rear = message
            self._latest_lidar_rear_view = top_view
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
    ) -> None:
        logger = self._logger
        if logger is None:
            return
        # 聚合事件先于下一条普通记录；两次提交都保持非阻塞。
        with self._log_lock:
            # 等待日志锁期间 close 可能已完成；旧代消息不得触碰终结后的 logger。
            with self._condition:
                if tracker_generation is not None and (
                    self._lifecycle_generation != tracker_generation
                    or self._state != "open"
                ):
                    return
            self._flush_pending_logger_drops_locked(logger, terminal=False)
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
            try:
                accepted = logger.record_event(event, **event_fields)
            except Exception:
                accepted = False
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

    def status_snapshot(self, wall_time: float | None = None) -> InterfaceStatusSnapshot:
        """在单一临界区复制六话题、命令邮箱和实际 transport 状态。"""
        with self._condition:
            captured_at = self._monotonic() if wall_time is None else wall_time
            transport = self._transport.snapshot()
            return self._status_snapshot_locked(captured_at, transport)

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
            if self._state == "open" and channel is self._config.wheel_command:
                if peer_state in {"waiting_peer", "disconnected"}:
                    topic_state = peer_state
            elif (
                self._state == "open"
                and self._ecal_lifecycle_enabled
                and transport_quality.peer_connected is False
            ):
                topic_state = "waiting_peer"
                topic_detail = "eCAL 话题对端未连接"
            if self._state == "open" and transport_quality.state != "active":
                topic_state = transport_quality.state
                topic_detail = transport_quality.detail
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
        with self._condition:
            self._require_open_locked()
            self._paused = True

    def resume(self, wall_time: float | None = None) -> WheelDecision:
        """先按 100 ms 墙钟期限刷新安全决定，再恢复物理发布。"""
        with self._condition:
            self._require_open_locked()
            now = self._monotonic() if wall_time is None else wall_time
            decision = self._mailbox.decision(now=now)
            self._last_decision = decision
            self._paused = False
            return decision

    def prepare_world_rebuild(self) -> None:
        """关闭命令回调入口、清空旧邮箱并安全停止当前机器人。"""
        if getattr(self._publish_context, "depth", 0) > 0:
            raise RuntimeError(
                "prepare_world_rebuild cannot run from an in-flight publish callback"
            )
        with self._condition:
            self._require_open_locked()
            if not self._world_ready:
                raise RuntimeError("world rebuild is already prepared")
            self._prepare_in_progress = True
            self._rebuild_prepared = False
            self._prepared_robot_parked = False
            self._accepting_commands = False
            self._world_ready = False
            self._active_subscription_token = None
            subscription = self._command_subscription
            self._command_subscription = None
            robot = self._robot
            # 已登记 publish 先在旧代完成日志线性化；world_ready=False 已阻止新发布。
            while self._in_flight_publishes > 0:
                self._condition.wait()
            self._lifecycle_generation += 1
            self._mailbox.clear()
            self._clear_dashboard_payloads_locked()
            self._begin_new_command_epoch_locked()
            self._last_decision = _waiting_decision(self._robot_model)
            self._local_twist = None
            self._safe_stop_latched = False
            self._safe_stop_mode = None
        # 订阅屏障先等已捕获回调离开，再接触可能被重建的 PyBullet 对象。
        first_error: BaseException | None = None
        if subscription is not None:
            try:
                subscription.close()
            except BaseException as exc:
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
            self._rebuild_prepared = True
            self._prepared_robot_parked = robot_parked
            self._condition.notify_all()
        if first_error is not None:
            raise first_error

    def abort_world_rebuild(self) -> None:
        """prepare 失败且旧世界未删除时，恢复旧绑定和全新等待命令态。"""
        with self._condition:
            while self._prepare_in_progress and self._state == "open":
                self._condition.wait()
            self._require_open_locked()
            if self._world_ready and not self._rebuild_prepared:
                return
            if not self._rebuild_prepared or self._world_ready:
                raise RuntimeError("world rebuild is not abortable")
            lifecycle_generation = self._lifecycle_generation
            model = self._robot_model

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
                        self._lifecycle_generation += 1
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
        with self._condition:
            while self._prepare_in_progress and self._state == "open":
                self._condition.wait()
            if self._state == "faulted":
                return
            self._require_open_locked()
            if self._world_ready or not self._rebuild_prepared:
                raise RuntimeError("world rebuild is not prepared")
            self._state = "faulted"
            self._lifecycle_generation += 1
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

    def commit_world_rebuild(
        self,
        new_robot: WheelRobotPort,
        new_backend: SensorBackend,
        new_document: SceneDocument,
    ) -> None:
        """完整构造新邮箱和传感器后，一次恢复世界绑定与命令订阅。"""
        with self._condition:
            while self._prepare_in_progress and self._state == "open":
                self._condition.wait()
            self._require_open_locked()
            if not self._rebuild_prepared or self._world_ready:
                raise RuntimeError("prepare_world_rebuild must be called first")
            old_backend = self._sensor_backend
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
        subscription, subscription_token = self._subscribe_wheel_command()
        try:
            with self._condition:
                self._require_open_locked()
                if self._world_ready:
                    raise RuntimeError("prepare_world_rebuild must be called first")
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
                self._command_subscription = subscription
                self._active_subscription_token = subscription_token
                self._accepting_commands = True
                self._world_ready = True
                self._rebuild_prepared = False
                self._prepared_robot_parked = False
        except BaseException:
            try:
                subscription.close()
            except BaseException:
                pass
            raise
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
        binder = getattr(backend, "bind_scene", None)
        if not callable(binder):
            raise ValueError("sensor backend must implement bind_scene")
        binder(terrain_ids, snapshots)
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
        model = _robot_model(robot)
        mailbox = WheelCommandMailbox(
            model,
            timeout_sec=self._config.command_timeout_sec,
            frequency_window_sec=self._config.status_window_sec,
        )
        decision = mailbox.decision(now=self._monotonic())
        with self._condition:
            self._require_rebind_ready_locked()
        candidate_subscription, candidate_token = self._subscribe_wheel_command()
        try:
            with self._condition:
                self._require_rebind_ready_locked()
                old_mailbox = self._mailbox
                old_robot = self._robot
                old_subscription = self._command_subscription
                old_robot.hold_current_steering_and_stop_drive(_SAFE_STOP_DT)
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
        with self._condition:
            if self._state == "closed":
                return
            if self._state == "closing":
                while self._state != "closed":
                    self._condition.wait()
                if self._close_error is not None:
                    raise self._close_error
                return
            self._state = "closing"
            self._lifecycle_generation += 1
            self._accepting_commands = False
            self._active_subscription_token = None
            # prepare 在锁外停车；close 必须等结果后才能决定是否需要重试。
            while self._prepare_in_progress:
                self._condition.wait()
            skip_safe_stop = not self._world_ready and self._prepared_robot_parked
            self._world_ready = False
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
        run_step("stop_sensors")
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
            self._condition.notify_all()
        if first_error is not None:
            raise first_error
