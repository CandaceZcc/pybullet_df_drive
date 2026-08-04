# 真实 eCAL 传输适配器：隔离版本绑定、六话题资源、最新帧队列与重连生命周期。
from __future__ import annotations

from collections import OrderedDict
from collections.abc import Callable
from contextlib import contextmanager
from dataclasses import dataclass
from importlib import import_module
import math
from numbers import Real
from threading import Condition, Event, Lock, RLock, Thread, current_thread, local
import time
from typing import Any

from google.protobuf.message import Message

from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.generated import slope_sim_interfaces_pb2 as pb
from slope_sim.interfaces.transport import (
    LocalTransport,
    TRANSPORT_STATES,
    TransportCallback,
    TransportSnapshot,
    TransportTopicQuality,
)


_UINT32_MAX = (1 << 32) - 1
_UINT64_MAX = (1 << 64) - 1
_ROLES = frozenset({"simulation", "peer"})
_STOPPING_STATES = frozenset({"quiescing", "quiesced", "closing", "closed"})
_CALLBACK_CONTEXT = local()


class EcalUnavailableError(RuntimeError):
    """表示已知的 eCAL Python binding 均无法导入。"""


@contextmanager
def _transport_callback_context(transport: object):
    """标记当前用户回调及其所属 transport，供重入生命周期操作识别。"""
    previous_depth = getattr(_CALLBACK_CONTEXT, "depth", 0)
    previous_transports = getattr(_CALLBACK_CONTEXT, "transports", ())
    _CALLBACK_CONTEXT.depth = previous_depth + 1
    _CALLBACK_CONTEXT.transports = (*previous_transports, transport)
    try:
        yield
    finally:
        if previous_depth == 0:
            del _CALLBACK_CONTEXT.depth
        else:
            _CALLBACK_CONTEXT.depth = previous_depth
        if previous_transports:
            _CALLBACK_CONTEXT.transports = previous_transports
        else:
            del _CALLBACK_CONTEXT.transports


def _inside_transport_callback(transport: object) -> bool:
    """判断当前线程是否正在该 transport 的 delivery gate 内执行用户代码。"""
    return transport in getattr(_CALLBACK_CONTEXT, "transports", ())


@dataclass(frozen=True)
class EcalTransportSnapshot(TransportSnapshot):
    """在通用传输计数之外暴露 eCAL 连接详情和连接代际。"""

    detail: str
    state: str
    generation: int


@dataclass(frozen=True)
class _ChannelBinding:
    """保存集中配置话题对应的完整类型名与 Protobuf 类。"""

    topic: str
    direction: str
    type_name: str
    message_type: type[Message]


@dataclass(frozen=True)
class _PendingMessage:
    """保存异步发布线程尚未发送的单话题最新负载。"""

    payload: bytes
    sim_time_ns: int
    wall_time: float


class _CoreParticipant:
    """把 eCAL 的进程级 initialize/finalize 包装成幂等 participant 资源。"""

    def __init__(self, core: object) -> None:
        self._core: object | None = core
        self._closed = False

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        core = self._core
        self._core = None
        finalize = getattr(core, "finalize", None)
        if not callable(finalize):
            raise RuntimeError("eCAL core.finalize is unavailable")
        result = finalize()
        if result is False:
            raise RuntimeError("eCAL core.finalize returned False")


class _ProtoResource:
    """保留官方 v6 Protobuf 资源，并在关闭时清除全部 Python 引用。"""

    def __init__(
        self,
        raw: object,
        *,
        direction: str,
        callback: Callable[..., None] | None = None,
    ) -> None:
        self.raw: object | None = raw
        self.direction = direction
        self.callback = callback
        self._closed = False

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        raw = self.raw
        self.raw = None
        self.callback = None
        if self.direction != "subscriber":
            return
        remove_callback = getattr(raw, "remove_receive_callback", None)
        if not callable(remove_callback):
            raise RuntimeError("eCAL Subscriber.remove_receive_callback is unavailable")
        remove_callback()


def _resource_raw(resource: _ProtoResource) -> object:
    """拒绝访问已清引用的底层官方资源。"""
    if resource.raw is None:
        raise RuntimeError(f"eCAL {resource.direction} resource is closed")
    return resource.raw


def _resource_peer_connected(resource: _ProtoResource) -> bool:
    """严格读取官方 v6 discovery count，拒绝 bool 和伪整数。"""
    raw = _resource_raw(resource)
    method_name = (
        "get_publisher_count"
        if resource.direction == "subscriber"
        else "get_subscriber_count"
    )
    count_method = getattr(raw, method_name, None)
    if not callable(count_method):
        raise RuntimeError(f"eCAL resource.{method_name} is unavailable")
    count = count_method()
    if isinstance(count, bool) or not isinstance(count, int) or count < 0:
        raise RuntimeError("eCAL peer count must be a nonnegative integer")
    return count > 0


@dataclass(frozen=True)
class EcalBindings:
    """封装项目唯一支持的 Eclipse eCAL 6.1 官方 Protobuf API。"""

    api: str
    core: object
    proto_core: object
    common_core: object

    @classmethod
    def v61(
        cls,
        core: object,
        proto_core: object,
        common_core: object,
    ) -> "EcalBindings":
        """绑定 `nanobind_core` 和 `ecal.msg.proto.core` 官方路径。"""
        cls._validate_modules(core, proto_core, common_core)
        return cls("v6.1", core, proto_core, common_core)

    @staticmethod
    def _validate_modules(
        core: object,
        proto_core: object,
        common_core: object,
    ) -> None:
        """导入成功后立即拒绝不完整 API，且不把兼容错误伪装成缺包。"""
        required = (
            (core, "initialize", "eCAL core.initialize"),
            (core, "finalize", "eCAL core.finalize"),
            (
                core,
                "get_publisher_configuration",
                "eCAL core.get_publisher_configuration",
            ),
            (proto_core, "Publisher", "eCAL proto Publisher"),
            (proto_core, "Subscriber", "eCAL proto Subscriber"),
            (common_core, "ReceiveCallbackData", "eCAL ReceiveCallbackData"),
        )
        for owner, attribute, label in required:
            if not callable(getattr(owner, attribute, None)):
                raise RuntimeError(f"{label} is unavailable")

    def create_participant(self, name: str) -> _CoreParticipant:
        """初始化当前进程的 eCAL participant，并检查显式失败返回值。"""
        result = self.core.initialize(name)
        if result is False:
            raise RuntimeError("eCAL core.initialize returned False")
        return _CoreParticipant(self.core)

    def create_publisher(
        self,
        topic: str,
        message_type: type[Message],
        *,
        shm_buffer_count: int,
        acknowledge_timeout_ms: int,
    ) -> _ProtoResource:
        """创建使用有界 SHM ring 与 subscriber ACK 的真实 publisher。"""
        buffer_count = _require_queue_size(shm_buffer_count)
        ack_timeout_ms = _require_uint32(
            "acknowledge_timeout_ms",
            acknowledge_timeout_ms,
        )
        publisher_config = self.core.get_publisher_configuration()
        try:
            publisher_config.layer.shm.acknowledge_timeout_ms = ack_timeout_ms
            publisher_config.layer.shm.memfile_buffer_count = buffer_count
        except AttributeError as error:
            raise RuntimeError(
                "eCAL publisher SHM configuration is unavailable"
            ) from error
        publisher_type = self.proto_core.Publisher
        return _ProtoResource(
            publisher_type(message_type, topic, publisher_config),
            direction="publisher",
        )

    def create_subscriber(
        self,
        topic: str,
        message_type: type[Message],
        callback: Callable[[bytes], None],
    ) -> _ProtoResource:
        """创建 v6 subscriber，并只从 ReceiveCallbackData.message 复制负载。"""
        subscriber_type = self.proto_core.Subscriber
        subscriber = subscriber_type(message_type, topic)
        resource = _ProtoResource(subscriber, direction="subscriber")

        def protobuf_callback(_topic_id: object, data: object) -> None:
            message = getattr(data, "message", None)
            if not isinstance(message, Message):
                raise RuntimeError(
                    "eCAL ReceiveCallbackData.message must be a protobuf message"
                )
            callback(message.SerializeToString(deterministic=True))

        resource.callback = protobuf_callback
        set_callback = getattr(subscriber, "set_receive_callback", None)
        if not callable(set_callback):
            resource.close()
            raise RuntimeError("eCAL Subscriber.set_receive_callback is unavailable")
        try:
            set_callback(protobuf_callback)
        except BaseException:
            resource.close()
            raise
        return resource

    @staticmethod
    def send(
        publisher: _ProtoResource,
        payload: bytes,
        message_type: type[Message],
    ) -> None:
        """解析稳定线负载后交给 eCAL Protobuf publisher。"""
        message = message_type()
        message.ParseFromString(payload)
        send = getattr(_resource_raw(publisher), "send", None)
        if not callable(send):
            raise RuntimeError("eCAL ProtoPublisher.send is unavailable")
        result = send(message)
        if result is False:
            raise RuntimeError("eCAL ProtoPublisher.send returned False")

    @staticmethod
    def is_peer_connected(resource: _ProtoResource) -> bool:
        """通过官方 count API 判断对应远端端点是否存在。"""
        return _resource_peer_connected(resource)

    @staticmethod
    def close(resource: object) -> None:
        """关闭由 binding 创建的 participant、publisher 或 subscriber。"""
        close = getattr(resource, "close", None)
        if not callable(close):
            raise RuntimeError("eCAL resource.close is unavailable")
        close()


def _copy_ecal_payload(payload: object) -> bytes:
    """把 eCAL 回调值复制成与上层 TransportCallback 一致的 bytes。"""
    if isinstance(payload, Message):
        return payload.SerializeToString(deterministic=True)
    if isinstance(payload, (bytes, bytearray, memoryview)):
        return bytes(payload)
    raise ValueError("eCAL payload must be a protobuf message or bytes-like")


def load_ecal_bindings() -> EcalBindings:
    """只导入项目固定的 Eclipse eCAL 6.1 官方 API。"""
    try:
        core = import_module("ecal.nanobind_core")
        proto_core = import_module("ecal.msg.proto.core")
        common_core = import_module("ecal.msg.common.core")
    except ImportError as error:
        raise EcalUnavailableError(
            f"official Eclipse eCAL 6.1 bindings unavailable: {error}"
        ) from error
    return EcalBindings.v61(core, proto_core, common_core)


def _channel_bindings(config: InterfaceConfig) -> tuple[_ChannelBinding, ...]:
    """按集中配置固定顺序组装六个话题和五种 Protobuf 类型。"""
    return (
        _ChannelBinding(
            config.wheel_command.topic,
            config.wheel_command.direction,
            "slope_sim.interfaces.v1.WheelCommand",
            pb.WheelCommand,
        ),
        _ChannelBinding(
            config.wheel_state.topic,
            config.wheel_state.direction,
            "slope_sim.interfaces.v1.WheelState",
            pb.WheelState,
        ),
        _ChannelBinding(
            config.lidar_front.topic,
            config.lidar_front.direction,
            "slope_sim.interfaces.v1.LidarPointCloud",
            pb.LidarPointCloud,
        ),
        _ChannelBinding(
            config.lidar_rear.topic,
            config.lidar_rear.direction,
            "slope_sim.interfaces.v1.LidarPointCloud",
            pb.LidarPointCloud,
        ),
        _ChannelBinding(
            config.rtk.topic,
            config.rtk.direction,
            "slope_sim.interfaces.v1.RtkState",
            pb.RtkState,
        ),
        _ChannelBinding(
            config.imu.topic,
            config.imu.direction,
            "slope_sim.interfaces.v1.ImuAttitude",
            pb.ImuAttitude,
        ),
    )


def _require_nonempty_text(name: str, value: object) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _require_uint64(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= _UINT64_MAX:
        raise ValueError(f"{name} must be a uint64 integer")
    return value


def _require_wall_time(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a nonnegative finite number")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized < 0.0:
        raise ValueError(f"{name} must be a nonnegative finite number")
    return normalized


def _require_queue_size(value: object) -> int:
    return _require_positive_uint32("queue_size", value)


def _require_positive_uint32(name: str, value: object) -> int:
    """校验写入 eCAL 原生 unsigned int 配置的正整数。"""
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 < value <= _UINT32_MAX
    ):
        raise ValueError(f"{name} must be a positive uint32 integer")
    return value


def _require_uint32(name: str, value: object) -> int:
    """校验允许零值关闭功能的 eCAL unsigned int 配置。"""
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= _UINT32_MAX
    ):
        raise ValueError(f"{name} must be a uint32 integer")
    return value


def publisher_acknowledge_timeout_ms(config: InterfaceConfig) -> int:
    """以命令 watchdog 为 ACK 等待上限，并向上取整到原生毫秒。"""
    if not isinstance(config, InterfaceConfig):
        raise ValueError("config must be an InterfaceConfig")
    return _require_positive_uint32(
        "acknowledge_timeout_ms",
        math.ceil(config.command_timeout_sec * 1_000.0),
    )


def _error_detail(error: BaseException) -> str:
    """生成供逐话题质量快照消费的稳定非空错误说明。"""
    message = str(error)
    return f"{type(error).__name__}: {message}" if message else type(error).__name__


class _DetailedLocalTransport(LocalTransport):
    """为显式 local/auto 降级补充不会误报 eCAL 的状态详情。"""

    def __init__(
        self,
        *,
        monotonic: Callable[[], float] | None = None,
        detail: str = "eCAL 未连接",
    ) -> None:
        super().__init__(monotonic=monotonic)
        self._detail = _require_nonempty_text("detail", detail)

    def snapshot(self) -> EcalTransportSnapshot:
        base = super().snapshot()
        return EcalTransportSnapshot(
            mode=base.mode,
            ecal_connected=base.ecal_connected,
            published_count=base.published_count,
            received_count=base.received_count,
            error_count=base.error_count,
            dropped_count=base.dropped_count,
            topic_quality=base.topic_quality,
            detail=self._detail,
            state="disconnected",
            generation=0,
        )


class EcalSubscription:
    """逻辑订阅句柄；底层 eCAL subscriber 由 transport 集中持有。"""

    def __init__(
        self,
        transport: "EcalTransport",
        topic: str,
        callback: TransportCallback,
    ) -> None:
        self._transport = transport
        self._topic = topic
        self._callback = callback
        self._active = True

    def close(self) -> None:
        self._transport._close_subscription(self)


class EcalTransport:
    """集中创建六话题真实资源，并异步发送每话题最新 Protobuf 帧。"""

    def __init__(
        self,
        config: InterfaceConfig | None = None,
        *,
        bindings: EcalBindings | Any | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        start_worker: bool = True,
        queue_size: int | None = None,
        participant_name: str = "slope-sim",
        role: str = "simulation",
        peer_state_callback: Callable[[str], None] | None = None,
    ) -> None:
        selected_config = InterfaceConfig.default(transport_mode="ecal") if config is None else config
        if not isinstance(selected_config, InterfaceConfig):
            raise ValueError("config must be an InterfaceConfig")
        if not callable(monotonic):
            raise ValueError("monotonic must be callable")
        if not isinstance(start_worker, bool):
            raise ValueError("start_worker must be a bool")
        if not isinstance(role, str) or role not in _ROLES:
            raise ValueError("role must be 'simulation' or 'peer'")
        if peer_state_callback is not None and not callable(peer_state_callback):
            raise ValueError("peer_state_callback must be callable")
        normalized_name = _require_nonempty_text("participant_name", participant_name)
        selected_queue_size = (
            selected_config.outgoing_queue_size if queue_size is None else queue_size
        )

        self._config = selected_config
        self._bindings = load_ecal_bindings() if bindings is None else bindings
        self._monotonic = monotonic
        self._queue_size = _require_queue_size(selected_queue_size)
        self._acknowledge_timeout_ms = (
            publisher_acknowledge_timeout_ms(selected_config)
            if role == "peer"
            else 0
        )
        self._role = role
        self._peer_state_callback = peer_state_callback
        self._condition = Condition()
        self._delivery_gate = RLock()
        self._discovery_gate = RLock()
        self._state = "initializing"
        self._stop_worker = False
        self._in_flight_deliveries = 0
        self._in_flight_discoveries = 0
        self._cleanup_thread: Thread | None = None
        self._cleanup_claimed = False
        self._close_error: BaseException | None = None
        self._quiesce_started = False
        self._quiesce_complete = False
        self._peer_connected = False
        self._deferred_peer_connected: tuple[int, dict[str, bool]] | None = None
        self._next_discovery_revision = 0
        self._applied_discovery_revision = -1
        self._generation = 0
        self._detail = ""
        self._published_count = 0
        self._received_count = 0
        self._error_count = 0
        self._dropped_count = 0
        self._pending: OrderedDict[str, _PendingMessage] = OrderedDict()
        self._claimed_pending: dict[str, _PendingMessage] = {}
        self._claimed_send_started: set[str] = set()
        self._subscriptions: dict[str, list[EcalSubscription]] = {}
        self._subscriber_resources: list[tuple[_ChannelBinding, object]] = []
        self._publisher_resources: list[tuple[_ChannelBinding, object]] = []
        self._publisher_by_topic: dict[str, tuple[_ChannelBinding, object]] = {}
        self._subscriber_by_topic: dict[str, tuple[_ChannelBinding, object]] = {}
        self._send_locks: dict[str, Lock] = {}
        self._worker_events: dict[str, Event] = {}
        self._participant: object | None = None
        self._workers: dict[str, Thread] = {}
        self._worker_exited_topics: set[str] = set()
        self._fatal_worker_fault: tuple[str, BaseException] | None = None

        channels = _channel_bindings(selected_config)
        if role == "peer":
            channels = tuple(
                _ChannelBinding(
                    channel.topic,
                    "publish" if channel.direction == "subscribe" else "subscribe",
                    channel.type_name,
                    channel.message_type,
                )
                for channel in channels
            )
        self._channels = channels
        self._topic_quality = {
            channel.topic: TransportTopicQuality(channel.topic)
            for channel in channels
        }

        # 初始化是一个事务：任何阶段失败都逆序释放此前已经创建的真实资源。
        try:
            self._participant = self._bindings.create_participant(normalized_name)
            for channel in channels:
                if channel.direction != "subscribe":
                    continue
                resource = self._bindings.create_subscriber(
                    channel.topic,
                    channel.message_type,
                    lambda payload, topic=channel.topic: self._on_payload(topic, payload),
                )
                self._subscriber_resources.append((channel, resource))
                self._subscriber_by_topic[channel.topic] = (channel, resource)
                self._subscriptions[channel.topic] = []
            for channel in channels:
                if channel.direction != "publish":
                    continue
                resource = self._bindings.create_publisher(
                    channel.topic,
                    channel.message_type,
                    shm_buffer_count=self._queue_size,
                    acknowledge_timeout_ms=self._acknowledge_timeout_ms,
                )
                self._publisher_resources.append((channel, resource))
                self._publisher_by_topic[channel.topic] = (channel, resource)
                self._send_locks[channel.topic] = Lock()
                self._worker_events[channel.topic] = Event()
            self._state = "waiting_peer"
            if start_worker:
                for index, (channel, _resource) in enumerate(
                    self._publisher_resources,
                    start=1,
                ):
                    worker = Thread(
                        target=self._worker_main,
                        args=(channel.topic,),
                        name=f"ecal-{role}-publisher-{index}",
                        daemon=True,
                    )
                    self._workers[channel.topic] = worker
                    worker.start()
        except BaseException:
            self._cleanup_partial_initialization()
            raise

    @property
    def worker_alive(self) -> bool:
        return any(
            worker.is_alive() and topic not in self._worker_exited_topics
            for topic, worker in self._workers.items()
        )

    @property
    def role(self) -> str:
        return self._role

    def _is_worker_thread(self) -> bool:
        """判断当前调用是否来自任一 publisher lane。"""
        caller = current_thread()
        return any(caller is worker for worker in self._workers.values())

    def _cleanup_partial_initialization(self) -> None:
        """初始化失败时继续清理全部资源，并保留最初的创建异常。"""
        with self._condition:
            self._stop_worker = True
            self._pending.clear()
            self._claimed_pending.clear()
            self._claimed_send_started.clear()
            for worker_event in self._worker_events.values():
                worker_event.set()
        for worker in tuple(self._workers.values()):
            try:
                is_alive = getattr(worker, "is_alive", None)
                join = getattr(worker, "join", None)
                if callable(is_alive) and is_alive() and callable(join):
                    join()
            except BaseException:
                pass
        self._workers.clear()
        self._worker_exited_topics.clear()
        for _channel, resource in reversed(self._publisher_resources):
            try:
                self._bindings.close(resource)
            except BaseException:
                pass
        for _channel, resource in reversed(self._subscriber_resources):
            try:
                self._bindings.close(resource)
            except BaseException:
                pass
        participant = self._participant
        self._participant = None
        self._publisher_by_topic.clear()
        self._subscriber_by_topic.clear()
        self._send_locks.clear()
        self._worker_events.clear()
        self._publisher_resources.clear()
        self._subscriber_resources.clear()
        if participant is not None:
            try:
                self._bindings.close(participant)
            except BaseException:
                pass

    def _require_open_locked(self) -> None:
        if self._state in _STOPPING_STATES:
            raise RuntimeError("transport is closed")
        fatal_fault = self._fatal_worker_fault
        if fatal_fault is not None:
            topic, error = fatal_fault
            raise RuntimeError(
                self._worker_fatal_detail(topic, error)
            ) from error

    @staticmethod
    def _worker_fatal_detail(topic: str, error: BaseException) -> str:
        """生成可定位 publisher lane 与原始异常的全局致命故障说明。"""
        return (
            f"publisher worker fatal fault on topic {topic!r}: "
            f"{_error_detail(error)}"
        )

    def _latch_worker_fatal_locked(
        self,
        topic: str,
        error: BaseException,
    ) -> None:
        """调用方持锁时只锁存首个 publisher worker 致命退出。"""
        if self._fatal_worker_fault is not None:
            return
        self._fatal_worker_fault = (topic, error)
        detail = self._worker_fatal_detail(topic, error)
        self._error_count += 1
        self._update_topic_quality_locked(
            topic,
            error_delta=1,
            state="error",
            detail=detail,
        )
        if self._state not in _STOPPING_STATES:
            self._state = "error"
            self._detail = detail

        # 任一 lane 丢失后全局传输已不可恢复，唤醒其余 lane 统一退出。
        self._stop_worker = True
        for worker_event in self._worker_events.values():
            worker_event.set()

    def subscribe(
        self,
        topic: str,
        type_name: str,
        callback: TransportCallback,
    ) -> EcalSubscription:
        """把上层回调挂到已集中创建的真实 subscriber。"""
        normalized_topic = _require_nonempty_text("topic", topic)
        normalized_type = _require_nonempty_text("type_name", type_name)
        if not callable(callback):
            raise ValueError("callback must be callable")
        with self._condition:
            self._require_open_locked()
            channel_resource = self._subscriber_by_topic.get(normalized_topic)
            if channel_resource is None:
                raise ValueError(f"topic {normalized_topic!r} is not configured for subscribe")
            channel, _resource = channel_resource
            if normalized_type != channel.type_name:
                raise ValueError(
                    f"topic {normalized_topic!r} requires type {channel.type_name!r}"
                )
            subscription = EcalSubscription(self, normalized_topic, callback)
            self._subscriptions[normalized_topic].append(subscription)
            return subscription

    def _close_subscription(self, subscription: EcalSubscription) -> None:
        """仅移除逻辑回调；真实 subscriber 随 transport 生命周期关闭。"""
        with self._condition:
            if not subscription._active:
                return
            subscription._active = False
            callbacks = self._subscriptions.get(subscription._topic)
            if callbacks is not None:
                try:
                    callbacks.remove(subscription)
                except ValueError:
                    pass

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float | None = None,
    ) -> bool:
        """非阻塞提交单话题最新帧；覆盖旧帧时准确累计 dropped。"""
        normalized_topic = _require_nonempty_text("topic", topic)
        normalized_type = _require_nonempty_text("type_name", type_name)
        if not isinstance(payload, (bytes, bytearray, memoryview)):
            raise ValueError("payload must be bytes-like")
        copied_payload = bytes(payload)
        normalized_sim_time = _require_uint64("sim_time_ns", sim_time_ns)
        clock_value = self._monotonic() if wall_time is None else wall_time
        normalized_wall_time = _require_wall_time("wall_time", clock_value)

        with self._condition:
            self._require_open_locked()
            channel_resource = self._publisher_by_topic.get(normalized_topic)
            if channel_resource is None:
                raise ValueError(f"topic {normalized_topic!r} is not configured for publish")
            channel, _resource = channel_resource
            if normalized_type != channel.type_name:
                raise ValueError(
                    f"topic {normalized_topic!r} requires type {channel.type_name!r}"
                )

            message = _PendingMessage(
                copied_payload,
                normalized_sim_time,
                normalized_wall_time,
            )
            dropped_topic: str | None = None
            if (
                normalized_topic in self._workers
                and normalized_topic not in self._claimed_pending
            ):
                if normalized_topic in self._pending:
                    raise RuntimeError("publisher lane pending ownership is inconsistent")
                # 生产线程只完成 lane 所有权交接，不在物理线程调用 native send。
                self._claimed_pending[normalized_topic] = message
            else:
                if normalized_topic in self._pending:
                    del self._pending[normalized_topic]
                    dropped_topic = normalized_topic
                elif len(self._pending) >= self._queue_size:
                    dropped_topic, _dropped_message = self._pending.popitem(last=False)
                self._pending[normalized_topic] = message
            if dropped_topic is not None:
                self._dropped_count += 1
                self._state = "degraded"
                self._detail = "输出队列覆盖旧消息"
                self._update_topic_quality_locked(
                    dropped_topic,
                    dropped_delta=1,
                    state="degraded",
                    detail="output queue replaced a pending message",
                )
            worker_event = self._worker_events.get(normalized_topic)
            if worker_event is not None:
                worker_event.set()
            return True

    def pending_payload(self, topic: str) -> bytes | None:
        """返回尚未发送的单话题负载副本，供队列诊断使用。"""
        normalized_topic = _require_nonempty_text("topic", topic)
        with self._condition:
            pending = self._pending.get(normalized_topic)
            return None if pending is None else bytes(pending.payload)

    def is_idle(self) -> bool:
        """只读判断所有 publisher lane 是否没有 ready、latest 或 native in-flight。"""
        with self._condition:
            self._require_open_locked()
            return not self._pending and not self._claimed_pending

    def is_topic_idle(self, topic: str) -> bool:
        """只读判断指定 publisher lane 是否没有 pending 或 native in-flight。"""
        normalized_topic = _require_nonempty_text("topic", topic)
        with self._condition:
            self._require_open_locked()
            if normalized_topic not in self._publisher_by_topic:
                raise ValueError(
                    f"topic {normalized_topic!r} is not configured for publish"
                )
            return (
                normalized_topic not in self._pending
                and normalized_topic not in self._claimed_pending
            )

    def wait_idle(self, *, timeout_sec: float) -> None:
        """有界等待 pending 与 worker 已认领帧全部发送完成。"""
        timeout = _require_wall_time("timeout_sec", timeout_sec)
        deadline = time.monotonic() + timeout
        with self._condition:
            self._require_open_locked()
            while self._pending or self._claimed_pending:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        f"transport did not become idle within {timeout:g}s"
                    )
                self._condition.wait(timeout=remaining)
                self._require_open_locked()

    def _on_payload(self, topic: str, payload: object) -> None:
        """锁外交付负载；至少一次成功且没有明确拒绝时才激活当前代。"""
        copied_payload = _copy_ecal_payload(payload)
        received_at = _require_wall_time("received_at", self._monotonic())
        with self._condition:
            if self._state in _STOPPING_STATES:
                return
            callback_generation = self._generation
            callbacks = tuple(
                subscription
                for subscription in self._subscriptions.get(topic, ())
                if subscription._active
            )
            self._received_count += 1
            self._in_flight_deliveries += 1

        self._delivery_gate.acquire()
        transition: str | None = None
        try:
            accepted_current_generation = False
            rejected_current_generation = False
            for subscription in callbacks:
                # 调用前先挡住已失效 delivery；用户 callback 始终在生命周期锁外执行。
                with self._condition:
                    if (
                        self._state in _STOPPING_STATES
                        or callback_generation != self._generation
                    ):
                        break
                    if not subscription._active:
                        continue

                callback_error: Exception | None = None
                accepted: bool | None = None
                with _transport_callback_context(self):
                    try:
                        accepted = subscription._callback(copied_payload, received_at)
                    except Exception as exc:
                        callback_error = exc

                # 返回后复核是结果的线性化点：旧代结果和异常不得污染新代。
                with self._condition:
                    if (
                        self._state in _STOPPING_STATES
                        or callback_generation != self._generation
                    ):
                        break
                    if callback_error is not None:
                        self._record_error_locked(callback_error)
                    elif accepted is False:
                        rejected_current_generation = True
                    elif accepted is True or accepted is None:
                        accepted_current_generation = True
                    else:
                        self._record_error_locked(
                            TypeError("transport callback must return bool or None")
                        )

            with self._condition:
                if (
                    self._state not in _STOPPING_STATES
                    and callback_generation == self._generation
                ):
                    self._set_topic_peer_connected_locked(topic, True)
                    if topic == self._config.wheel_command.topic:
                        self._peer_connected = True
                        if (
                            accepted_current_generation
                            and not rejected_current_generation
                        ):
                            transition = self._set_state_locked("active", self._detail)
                        elif self._state == "disconnected":
                            transition = self._set_state_locked(
                                "waiting_peer", self._detail
                            )
            self._notify_peer_transition(transition)
            self._drain_deferred_peer_state()
        finally:
            self._delivery_gate.release()
            with self._condition:
                self._in_flight_deliveries -= 1
                self._condition.notify_all()
            # 仍位于底层 subscriber callback 栈，绝不能在此同步销毁 subscriber。

    def _set_state_locked(self, state: str, detail: str) -> str | None:
        """调用方持锁时提交状态，并返回需要在锁外通知的变化。"""
        if state not in TRANSPORT_STATES:
            raise RuntimeError(f"invalid eCAL state: {state}")
        changed = self._state != state
        self._state = state
        self._detail = detail
        return state if changed else None

    def _notify_peer_transition(self, transition: str | None) -> None:
        """在 transport 锁外通知上层，以便断连时清邮箱和归零目标。"""
        if transition is None or self._peer_state_callback is None:
            return
        try:
            with _transport_callback_context(self):
                self._peer_state_callback(transition)
        except Exception as exc:
            self._record_error(exc)
        finally:
            self._run_deferred_cleanup_if_safe()

    def _apply_peer_state_locked(
        self,
        connected: bool,
        *,
        previously_known: bool,
    ) -> str | None:
        """只用命令话题 discovery 驱动安全代际和全局连接状态。"""
        def publish_lifecycle_state(state: str, detail: str) -> str | None:
            if any(
                quality.state != "active"
                for quality in self._topic_quality.values()
            ):
                # 健康错误保留在快照中，但连接边沿仍必须通知 runtime 清命令。
                return state
            return self._set_state_locked(state, detail)

        if not previously_known:
            self._peer_connected = connected
            if connected:
                initial_state = "active" if self._role == "peer" else "waiting_peer"
                return publish_lifecycle_state(initial_state, "")
            return publish_lifecycle_state(
                "disconnected", "eCAL 命令对端未连接"
            )
        if connected and not self._peer_connected:
            self._peer_connected = True
            reconnected_state = "active" if self._role == "peer" else "waiting_peer"
            return publish_lifecycle_state(reconnected_state, "")
        if not connected and self._peer_connected:
            self._peer_connected = False
            self._generation += 1
            return publish_lifecycle_state("disconnected", "eCAL 对端已断开")
        return None

    def _apply_peer_states_locked(
        self,
        observed: dict[str, bool],
    ) -> str | None:
        """原子提交六话题 discovery，命令边沿单独驱动 mailbox 生命周期。"""
        command_topic = self._config.wheel_command.topic
        command_quality = self._topic_quality[command_topic]
        command_previously_known = command_quality.peer_connected is not None
        for topic, connected in observed.items():
            self._set_topic_peer_connected_locked(topic, connected)
        return self._apply_peer_state_locked(
            observed[command_topic],
            previously_known=command_previously_known,
        )

    def _apply_discovery_observation_locked(
        self,
        revision: int,
        observed: dict[str, bool],
    ) -> str | None:
        """只提交最新 discovery revision，迟到旧观察不得回退连接状态。"""
        if revision <= self._applied_discovery_revision:
            return None
        self._applied_discovery_revision = revision
        return self._apply_peer_states_locked(observed)

    def _drain_deferred_peer_state(self) -> None:
        """在 delivery gate 内处理用户回调重入时暂存的最新 discovery 结果。"""
        while True:
            with self._condition:
                if self._state in _STOPPING_STATES:
                    self._deferred_peer_connected = None
                    return
                deferred = self._deferred_peer_connected
                self._deferred_peer_connected = None
                if deferred is None:
                    return
                revision, observed = deferred
                transition = self._apply_discovery_observation_locked(
                    revision,
                    observed,
                )
            self._notify_peer_transition(transition)

    def _record_error(self, error: Exception) -> None:
        with self._condition:
            self._record_error_locked(error)

    def _record_error_locked(self, error: Exception) -> None:
        """调用方持生命周期锁时原子提交当前代错误。"""
        if self._state in _STOPPING_STATES:
            return
        self._error_count += 1
        detail = str(error)
        self._state = "error"
        self._detail = (
            f"{type(error).__name__}: {detail}" if detail else type(error).__name__
        )

    def _record_publish_error_locked(self, topic: str, error: Exception) -> None:
        """同步累计全局和话题发送错误，关闭期不得改写 lifecycle 状态。"""
        detail = _error_detail(error)
        self._error_count += 1
        self._update_topic_quality_locked(
            topic,
            error_delta=1,
            state="error",
            detail=detail,
        )
        if self._state not in _STOPPING_STATES:
            self._state = "error"
            self._detail = detail

    def _update_topic_quality_locked(
        self,
        topic: str,
        *,
        error_delta: int = 0,
        dropped_delta: int = 0,
        state: str,
        detail: str,
    ) -> None:
        """调用方持锁时原子累计单话题质量并推进 revision。"""
        current = self._topic_quality[topic]
        self._topic_quality[topic] = TransportTopicQuality(
            topic=topic,
            error_count=current.error_count + error_delta,
            dropped_count=current.dropped_count + dropped_delta,
            state=state,
            detail=detail,
            revision=current.revision + 1,
            last_error_detail=(
                detail if error_delta else current.last_error_detail
            ),
            last_drop_detail=(
                detail if dropped_delta else current.last_drop_detail
            ),
            peer_connected=current.peer_connected,
        )

    def _set_topic_peer_connected_locked(self, topic: str, connected: bool) -> None:
        """只在单话题 discovery 值变化时推进质量 revision。"""
        current = self._topic_quality[topic]
        if current.peer_connected is connected:
            return
        self._topic_quality[topic] = TransportTopicQuality(
            topic=topic,
            error_count=current.error_count,
            dropped_count=current.dropped_count,
            state=current.state,
            detail=current.detail,
            revision=current.revision + 1,
            last_error_detail=current.last_error_detail,
            last_drop_detail=current.last_drop_detail,
            peer_connected=connected,
        )

    def _recover_topic_quality_locked(self, topic: str) -> None:
        """成功发送清除该话题活动故障，但保留累计 error/drop。"""
        current = self._topic_quality[topic]
        if current.state == "active":
            return
        self._topic_quality[topic] = TransportTopicQuality(
            topic=topic,
            error_count=current.error_count,
            dropped_count=current.dropped_count,
            state="active",
            detail="",
            revision=current.revision + 1,
            last_error_detail=current.last_error_detail,
            last_drop_detail=current.last_drop_detail,
            peer_connected=current.peer_connected,
        )

    def poll_peer_state(self) -> str:
        """逐话题轮询 discovery；只有命令边沿改变安全代际。"""
        discovery_registered = False
        try:
            with self._condition:
                self._require_open_locked()
            # native count API 彼此串行，但不占 publisher lane 或 transport 状态锁。
            with self._discovery_gate:
                with self._condition:
                    self._require_open_locked()
                    monitored = self._discovery_resources_locked()
                    revision = self._next_discovery_revision
                    self._next_discovery_revision += 1
                    self._in_flight_discoveries += 1
                    discovery_registered = True
                try:
                    observed = {
                        topic: self._bindings.is_peer_connected(resource)
                        for topic, resource in monitored
                    }
                except Exception as exc:
                    self._record_error(exc)
                    raise

            # 同线程用户回调已经持有 delivery gate，结果留到整个 delivery 返回前处理。
            if _inside_transport_callback(self):
                with self._condition:
                    if self._state in _STOPPING_STATES:
                        return "disconnected"
                    deferred = self._deferred_peer_connected
                    if deferred is None or revision > deferred[0]:
                        self._deferred_peer_connected = (revision, observed)
                    return self._state

            # delivery 与 discovery 共用独立门，确保断线清邮箱后旧 payload 不再进入上层。
            with self._delivery_gate:
                with self._condition:
                    if self._state in _STOPPING_STATES:
                        return "disconnected"
                    transition = self._apply_discovery_observation_locked(
                        revision,
                        observed,
                    )
                self._notify_peer_transition(transition)
                self._drain_deferred_peer_state()
                with self._condition:
                    if self._state in _STOPPING_STATES:
                        return "disconnected"
                    return self._state
        finally:
            if discovery_registered:
                with self._condition:
                    self._in_flight_discoveries -= 1
                    self._condition.notify_all()
                    if self._in_flight_discoveries < 0:
                        raise RuntimeError(
                            "in-flight discovery counter became negative"
                        )

    def _discovery_resources_locked(self) -> tuple[tuple[str, object], ...]:
        """按集中配置顺序返回本角色六个 publisher/subscriber 资源。"""
        resources: list[tuple[str, object]] = []
        for channel in self._channels:
            resource_map = (
                self._subscriber_by_topic
                if channel.direction == "subscribe"
                else self._publisher_by_topic
            )
            channel_resource = resource_map.get(channel.topic)
            if channel_resource is None:
                raise RuntimeError(
                    f"eCAL discovery resource is unavailable: {channel.topic}"
                )
            resources.append((channel.topic, channel_resource[1]))
        return tuple(resources)

    def _worker_main(self, topic: str) -> None:
        """运行单话题发送 lane，并在退出安全点接管延期清理。"""
        escaped_error: BaseException | None = None
        try:
            self._worker_loop(topic)
        except BaseException as exc:
            escaped_error = exc
            raise
        finally:
            with self._condition:
                # BaseException 会绕过普通 send completion；先恢复 ready 所有权。
                self._claimed_send_started.discard(topic)
                if escaped_error is not None:
                    self._latch_worker_fatal_locked(topic, escaped_error)
                if self._quiesce_started:
                    self._reclaim_unsent_locked()
                self._worker_exited_topics.add(topic)
                if (
                    self._quiesce_started
                    and len(self._worker_exited_topics) == len(self._workers)
                ):
                    self._quiesce_complete = True
                    if self._state == "quiescing":
                        self._state = "quiesced"
                self._condition.notify_all()
            self._run_deferred_cleanup_if_safe(worker_epilogue=True)

    def _worker_loop(self, topic: str) -> None:
        """持续排空单话题 ready/latest，并把 native send 与 ready 明确区分。"""
        worker_event = self._worker_events[topic]
        channel, publisher = self._publisher_by_topic[topic]
        while True:
            worker_event.wait()
            worker_event.clear()
            with self._send_locks[topic]:
                while True:
                    with self._condition:
                        if self._stop_worker or self._state in _STOPPING_STATES:
                            return
                        message = self._claim_next_send_locked(topic)
                        if message is None:
                            break

                    send_error: Exception | None = None
                    try:
                        self._bindings.send(
                            publisher,
                            message.payload,
                            channel.message_type,
                        )
                    except Exception as exc:
                        send_error = exc

                    with self._condition:
                        has_next = self._complete_claimed_send_locked(topic)
                        if send_error is not None:
                            self._record_publish_error_locked(topic, send_error)
                        else:
                            self._published_count += 1
                            if self._state not in _STOPPING_STATES:
                                self._recover_topic_quality_locked(topic)
                            if (
                                self._state == "degraded"
                                and not self._pending
                                and not self._claimed_pending
                                and all(
                                    quality.state == "active"
                                    for quality in self._topic_quality.values()
                                )
                            ):
                                recovered = (
                                    "active" if self._peer_connected else "waiting_peer"
                                )
                                self._set_state_locked(recovered, "")
                        if not has_next:
                            break

    def _claim_next_send_locked(self, topic: str) -> _PendingMessage | None:
        """调用方持锁时把 ready 标为 native in-flight，兼容无预交接旧状态。"""
        message = self._claimed_pending.get(topic)
        if message is None:
            message = self._pending.pop(topic, None)
            if message is None:
                return None
            self._claimed_pending[topic] = message
        if topic in self._claimed_send_started:
            raise RuntimeError("publisher lane already owns an in-flight message")
        self._claimed_send_started.add(topic)
        return message

    def _complete_claimed_send_locked(self, topic: str) -> bool:
        """结束 native send，并原子提升该话题当前 latest 供同次唤醒继续发送。"""
        self._require_claimed_pending_locked(topic)
        if topic not in self._claimed_send_started:
            raise RuntimeError("publisher lane send completion has no active send")
        self._claimed_send_started.remove(topic)
        del self._claimed_pending[topic]
        next_message = self._pending.pop(topic, None)
        if next_message is not None:
            self._claimed_pending[topic] = next_message
        self._condition.notify_all()
        return next_message is not None

    def _require_claimed_pending_locked(self, topic: str) -> None:
        """调用方持锁时确认 worker 仍唯一拥有指定待发送帧。"""
        if topic not in self._claimed_pending:
            raise RuntimeError("worker claimed pending ownership changed unexpectedly")

    def snapshot(self) -> EcalTransportSnapshot:
        """在一个临界区复制连接语义、代际和全部累计计数。"""
        with self._condition:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> EcalTransportSnapshot:
        """调用方持生命周期锁时构造完整 eCAL 快照。"""
        fatal_fault = self._fatal_worker_fault
        stopping = self._state in _STOPPING_STATES
        return EcalTransportSnapshot(
            mode="ecal",
            ecal_connected=self._peer_connected,
            published_count=self._published_count,
            received_count=self._received_count,
            error_count=self._error_count,
            dropped_count=self._dropped_count,
            topic_quality=tuple(self._topic_quality.values()),
            detail=(
                self._worker_fatal_detail(*fatal_fault)
                if fatal_fault is not None and not stopping
                else self._detail
            ),
            state=(
                "disconnected"
                if stopping
                else "error"
                if fatal_fault is not None
                else self._state
            ),
            generation=self._generation,
        )

    def _begin_quiesce_locked(self) -> None:
        """原子关闭入口，并把尚未由 worker 认领的帧恰好计数一次。"""
        if self._quiesce_started:
            return
        self._quiesce_started = True
        self._state = "quiescing"
        self._peer_connected = False
        self._deferred_peer_connected = None
        self._generation += 1
        for subscriptions in self._subscriptions.values():
            for subscription in subscriptions:
                subscription._active = False
            subscriptions.clear()
        self._reclaim_unsent_locked()
        self._stop_worker = True
        for worker_event in self._worker_events.values():
            worker_event.set()
        self._condition.notify_all()

    def _reclaim_unsent_locked(self) -> None:
        """回收未进入 native send 的 ready/latest，并恰好累计一次 terminal drop。"""
        unsent_by_topic: dict[str, int] = {}
        for pending_topic in self._pending:
            unsent_by_topic[pending_topic] = unsent_by_topic.get(pending_topic, 0) + 1
        for claimed_topic in tuple(self._claimed_pending):
            if claimed_topic in self._claimed_send_started:
                continue
            unsent_by_topic[claimed_topic] = unsent_by_topic.get(claimed_topic, 0) + 1
            del self._claimed_pending[claimed_topic]
        for pending_topic, count in unsent_by_topic.items():
            self._dropped_count += count
            self._update_topic_quality_locked(
                pending_topic,
                dropped_delta=count,
                state="degraded",
                detail="transport closed before pending message was sent",
            )
        self._pending.clear()

    def _finish_quiesce(self, *, wait_for_deliveries: bool) -> EcalTransportSnapshot:
        """停止并汇合全部 publisher lane；外部 quiesce 还等待既有 delivery。"""
        callback_context = _inside_transport_callback(self)
        caller = current_thread()
        worker_context = self._is_worker_thread()
        with self._condition:
            if self._state == "closed":
                return self._snapshot_locked()
            self._begin_quiesce_locked()
            workers = tuple(self._workers.values())

        for worker in workers:
            if worker is not caller:
                worker.join()

        with self._condition:
            if all(worker is caller or not worker.is_alive() for worker in workers):
                self._quiesce_complete = True
                if self._state == "quiescing":
                    self._state = "quiesced"
                self._condition.notify_all()
            if wait_for_deliveries and not callback_context and not worker_context:
                while (
                    self._in_flight_deliveries > 0
                    or self._in_flight_discoveries > 0
                ):
                    self._condition.wait()
            return self._snapshot_locked()

    def quiesce(self) -> EcalTransportSnapshot:
        """停止新 payload 和发布 worker，并返回资源关闭前的最终质量。"""
        return self._finish_quiesce(wait_for_deliveries=True)

    def close(self) -> None:
        """关闭资源；callback 内只发起异步清理，外部调用等待同一最终结果。"""
        callback_context = _inside_transport_callback(self)
        worker_context = self._is_worker_thread()
        cleanup_thread: Thread | None = None
        run_cleanup_inline = False

        with self._condition:
            if self._state == "closed":
                close_error = self._close_error
            elif self._state == "closing":
                if callback_context or worker_context:
                    return
                if not self._cleanup_claimed:
                    self._cleanup_claimed = True
                    run_cleanup_inline = True
                    close_error = None
                else:
                    while self._state != "closed":
                        self._condition.wait()
                    close_error = self._close_error
            else:
                self._begin_quiesce_locked()
                self._state = "closing"

                if callback_context or worker_context:
                    cleanup_thread = Thread(
                        target=self._claim_and_cleanup,
                        name=f"ecal-{self._role}-cleanup",
                        daemon=True,
                    )
                    self._cleanup_thread = cleanup_thread
                    close_error = None
                else:
                    self._cleanup_claimed = True
                    run_cleanup_inline = True
                    close_error = None

        if cleanup_thread is not None:
            try:
                cleanup_thread.start()
            except BaseException as exc:
                self._remember_close_error(exc)
                self._run_deferred_cleanup_if_safe()
                raise
            return
        if run_cleanup_inline:
            self._cleanup_resources()
            with self._condition:
                close_error = self._close_error
        if close_error is not None:
            raise close_error

    def _claim_and_cleanup(self) -> None:
        """让第一个安全执行者取得唯一 cleanup 所有权。"""
        with self._condition:
            if self._state != "closing" or self._cleanup_claimed:
                return
            self._cleanup_claimed = True
        self._cleanup_resources()

    def _run_deferred_cleanup_if_safe(self, *, worker_epilogue: bool = False) -> None:
        """callback/worker 退出后接管未成功启动的异步 cleanup。"""
        if _inside_transport_callback(self):
            return
        if self._is_worker_thread() and not worker_epilogue:
            return
        with self._condition:
            if self._in_flight_deliveries != 0 and not worker_epilogue:
                return
        self._claim_and_cleanup()

    def _remember_close_error(self, error: BaseException) -> None:
        """并发 cleanup 阶段只保留按锁顺序观察到的第一个错误。"""
        with self._condition:
            if self._close_error is None:
                self._close_error = error
            self._condition.notify_all()

    def _cleanup_resources(self) -> None:
        """唯一清理所有者按固定顺序关闭资源，并发布共享完成结果。"""
        self._finish_quiesce(wait_for_deliveries=False)

        # discovery 已持有 native 资源引用，必须先等 count API 全部返回。
        with self._condition:
            while self._in_flight_discoveries > 0:
                self._condition.wait()

        for _channel, resource in reversed(self._subscriber_resources):
            try:
                self._bindings.close(resource)
            except BaseException as exc:
                self._remember_close_error(exc)

        # 某些 binding 的 subscriber.close 不带回调屏障，因此再等 transport delivery。
        with self._condition:
            while self._in_flight_deliveries > 0:
                self._condition.wait()

        # `_finish_quiesce` 已汇合全部 lane，此后 publisher 不会再进入 send。
        for _channel, resource in reversed(self._publisher_resources):
            try:
                self._bindings.close(resource)
            except BaseException as exc:
                self._remember_close_error(exc)

        participant = self._participant
        self._participant = None
        self._publisher_by_topic.clear()
        self._subscriber_by_topic.clear()
        self._send_locks.clear()
        self._worker_events.clear()
        self._publisher_resources.clear()
        self._subscriber_resources.clear()
        if participant is not None:
            try:
                self._bindings.close(participant)
            except BaseException as exc:
                self._remember_close_error(exc)

        with self._condition:
            self._quiesce_complete = True
            self._state = "closed"
            self._detail = ""
            self._condition.notify_all()


def create_transport(
    mode: str,
    *,
    config: InterfaceConfig | None = None,
    bindings: EcalBindings | Any | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    start_worker: bool = True,
    queue_size: int | None = None,
    participant_name: str = "slope-sim",
    role: str = "simulation",
    peer_state_callback: Callable[[str], None] | None = None,
) -> LocalTransport | EcalTransport:
    """按严格 ecal、auto 降级或显式 local 模式创建传输。"""
    if not isinstance(mode, str) or mode not in {"auto", "ecal", "local"}:
        raise ValueError("mode must be 'auto', 'ecal', or 'local'")
    if not callable(monotonic):
        raise ValueError("monotonic must be callable")
    if mode == "local":
        return _DetailedLocalTransport(monotonic=monotonic)

    selected_bindings = bindings
    if selected_bindings is None:
        try:
            selected_bindings = load_ecal_bindings()
        except EcalUnavailableError as exc:
            if mode == "ecal":
                raise
            return _DetailedLocalTransport(
                monotonic=monotonic,
                detail=_error_detail(exc),
            )

    selected_config = InterfaceConfig.default(transport_mode=mode) if config is None else config
    return EcalTransport(
        selected_config,
        bindings=selected_bindings,
        monotonic=monotonic,
        start_worker=start_worker,
        queue_size=queue_size,
        participant_name=participant_name,
        role=role,
        peer_state_callback=peer_state_callback,
    )
