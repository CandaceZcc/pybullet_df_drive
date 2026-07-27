# 本地传输接口：提供确定性进程内发布订阅、生命周期和只读计数快照。
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
import math
from numbers import Real
from threading import Condition, local
import time
from typing import Protocol, runtime_checkable


_UINT64_MAX = (1 << 64) - 1
_TOPIC_QUALITY_STATES = frozenset({"active", "degraded", "error"})
TRANSPORT_STATES = frozenset(
    {"active", "waiting_peer", "degraded", "disconnected", "error"}
)
TransportCallback = Callable[[bytes, float], bool | None]
_CALLBACK_CONTEXT = local()


def _require_nonempty_text(name: str, value: object) -> str:
    """校验话题和消息类型使用非空字符串。"""
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _require_uint64(name: str, value: object) -> int:
    """校验无符号 64 位整数并排除 bool。"""
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= _UINT64_MAX:
        raise ValueError(f"{name} must be a uint64 integer")
    return value


def _require_wall_time(name: str, value: object) -> float:
    """校验单调墙钟参数为非负有限实数。"""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a nonnegative finite number")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized < 0.0:
        raise ValueError(f"{name} must be a nonnegative finite number")
    return normalized


def transport_state_to_command_peer_state(
    state: str,
    *,
    ecal_connected: bool,
) -> str:
    """把 transport 健康态映射为不误激活旧命令的 peer 生命周期状态。"""
    if not isinstance(state, str) or state not in TRANSPORT_STATES:
        raise ValueError("state must be a valid transport state")
    if not isinstance(ecal_connected, bool):
        raise ValueError("ecal_connected must be a bool")
    if state in {"error", "degraded"}:
        return "active" if ecal_connected else "disconnected"
    return state


@runtime_checkable
class Subscription(Protocol):
    """传输订阅仅暴露幂等关闭能力。"""

    def close(self) -> None:
        """停止后续消息交付。"""
        ...


@runtime_checkable
class Transport(Protocol):
    """企业接口运行时依赖的窄传输契约。"""

    def subscribe(
        self,
        topic: str,
        type_name: str,
        callback: TransportCallback,
    ) -> Subscription:
        """订阅指定话题；抛错必须异常原子，不能留下可交付的 callback。"""
        ...

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float | None = None,
    ) -> bool:
        """发布序列化负载并返回交付过程是否无回调错误。"""
        ...

    def snapshot(self) -> "TransportSnapshot":
        """返回当前传输计数。"""
        ...

    def quiesce(self) -> "TransportSnapshot":
        """停止新交付、等待外部回调收敛，并返回最终质量快照。"""
        ...

    def close(self) -> None:
        """在 quiesce 后幂等释放传输资源。"""
        ...


@dataclass(frozen=True)
class TransportTopicQuality:
    """传输层单话题累计质量和当前活动故障的不可变快照。"""

    topic: str
    error_count: int = 0
    dropped_count: int = 0
    state: str = "active"
    detail: str = ""
    revision: int = 0
    last_error_detail: str | None = None
    last_drop_detail: str | None = None
    peer_connected: bool | None = None

    def __post_init__(self) -> None:
        _require_nonempty_text("topic", self.topic)
        for name, value in (
            ("error_count", self.error_count),
            ("dropped_count", self.dropped_count),
            ("revision", self.revision),
        ):
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        if self.state not in _TOPIC_QUALITY_STATES:
            raise ValueError("state must be active, degraded, or error")
        if not isinstance(self.detail, str):
            raise ValueError("detail must be a string")
        if self.state == "active" and self.detail:
            raise ValueError("active topic quality detail must be empty")
        if self.state != "active" and not self.detail:
            raise ValueError("unhealthy topic quality detail must be nonempty")
        for name, value in (
            ("last_error_detail", self.last_error_detail),
            ("last_drop_detail", self.last_drop_detail),
        ):
            if value is not None and (not isinstance(value, str) or not value):
                raise ValueError(f"{name} must be None or a nonempty string")
        if self.peer_connected is not None and not isinstance(
            self.peer_connected, bool
        ):
            raise ValueError("peer_connected must be a bool or None")


@dataclass(frozen=True)
class TransportSnapshot:
    """传输模式、连接语义和累计计数的不可变快照。"""

    mode: str
    ecal_connected: bool
    published_count: int
    received_count: int
    error_count: int
    dropped_count: int
    topic_quality: tuple[TransportTopicQuality, ...] = field(
        default_factory=tuple,
        kw_only=True,
    )

    def __post_init__(self) -> None:
        if not isinstance(self.topic_quality, tuple):
            raise ValueError("topic_quality must be a tuple")
        if any(not isinstance(item, TransportTopicQuality) for item in self.topic_quality):
            raise ValueError("topic_quality must contain TransportTopicQuality values")
        topics = tuple(item.topic for item in self.topic_quality)
        if len(set(topics)) != len(topics):
            raise ValueError("topic_quality must not contain duplicate topics")


class LocalSubscription:
    """由 LocalTransport 管理的单个进程内订阅。"""

    def __init__(
        self,
        transport: "LocalTransport",
        topic: str,
        type_name: str,
        callback: TransportCallback,
    ) -> None:
        self._transport = transport
        self._topic = topic
        self._type_name = type_name
        self._callback = callback
        self._active = True
        self._in_flight = 0

    def close(self) -> None:
        """停用订阅；外部线程等待已开始的回调完成。"""
        self._transport._close_subscription(self)

    def _deliver(self, payload: bytes, received_at: float) -> bool | None:
        """在统一锁内登记交付，并始终在锁外执行用户回调。"""
        if not self._transport._begin_delivery(self):
            return None

        previous_transports = getattr(_CALLBACK_CONTEXT, "transports", ())
        _CALLBACK_CONTEXT.transports = (*previous_transports, self._transport)
        try:
            callback_result = self._callback(payload, received_at)
            if callback_result is False:
                return False
            if callback_result is not True and callback_result is not None:
                self._transport._record_error()
                return False
        except Exception:
            self._transport._record_error()
            return False
        finally:
            if previous_transports:
                _CALLBACK_CONTEXT.transports = previous_transports
            else:
                del _CALLBACK_CONTEXT.transports
            self._transport._finish_delivery(self)
        return True


class LocalTransport:
    """同步、线程安全且不声称 eCAL 连接的进程内传输。"""

    def __init__(self, monotonic: Callable[[], float] | None = None) -> None:
        if monotonic is not None and not callable(monotonic):
            raise ValueError("monotonic must be callable")
        self._monotonic = monotonic if monotonic is not None else time.monotonic
        self._condition = Condition()
        self._state = "open"
        self._topic_types: dict[str, str] = {}
        self._subscriptions: list[LocalSubscription] = []
        self._total_in_flight = 0
        self._published_count = 0
        self._received_count = 0
        self._error_count = 0
        self._dropped_count = 0

    def _require_open(self) -> None:
        """要求 transport 仍可接受新操作；调用方必须持锁。"""
        if self._state != "open":
            raise RuntimeError("transport is closed")

    def _bind_topic_type(self, topic: str, type_name: str) -> None:
        """首次绑定消息类型，后续冲突在同一临界区原子拒绝。"""
        existing = self._topic_types.get(topic)
        if existing is not None and existing != type_name:
            raise ValueError(f"topic {topic!r} already uses type {existing!r}")
        self._topic_types.setdefault(topic, type_name)

    def subscribe(
        self,
        topic: str,
        type_name: str,
        callback: TransportCallback,
    ) -> LocalSubscription:
        """注册同步回调，并保持一个话题在 transport 生命周期内类型稳定。"""
        with self._condition:
            self._require_open()
        normalized_topic = _require_nonempty_text("topic", topic)
        normalized_type = _require_nonempty_text("type_name", type_name)
        if not callable(callback):
            raise ValueError("callback must be callable")

        with self._condition:
            self._require_open()
            self._bind_topic_type(normalized_topic, normalized_type)
            subscription = LocalSubscription(self, normalized_topic, normalized_type, callback)
            self._subscriptions.append(subscription)
            return subscription

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float | None = None,
    ) -> bool:
        """复制负载和订阅快照，在 transport 锁外按注册顺序同步交付。"""
        with self._condition:
            self._require_open()
        normalized_topic = _require_nonempty_text("topic", topic)
        normalized_type = _require_nonempty_text("type_name", type_name)
        if not isinstance(payload, (bytes, bytearray, memoryview)):
            raise ValueError("payload must be bytes-like")
        copied_payload = bytes(payload)
        _require_uint64("sim_time_ns", sim_time_ns)
        clock_value = self._monotonic() if wall_time is None else wall_time
        received_at = _require_wall_time("wall_time", clock_value)

        with self._condition:
            self._require_open()
            self._bind_topic_type(normalized_topic, normalized_type)
            self._published_count += 1
            subscriptions = tuple(
                subscription
                for subscription in self._subscriptions
                if subscription._topic == normalized_topic
                and subscription._type_name == normalized_type
            )

        delivered_without_error = True
        for subscription in subscriptions:
            result = subscription._deliver(copied_payload, received_at)
            if result is False:
                delivered_without_error = False
        return delivered_without_error

    def snapshot(self) -> TransportSnapshot:
        """在一个临界区复制所有累计计数。"""
        with self._condition:
            return self._snapshot_locked()

    def _snapshot_locked(self) -> TransportSnapshot:
        """调用方持生命周期锁时构造本地累计计数快照。"""
        return TransportSnapshot(
            mode="local",
            ecal_connected=False,
            published_count=self._published_count,
            received_count=self._received_count,
            error_count=self._error_count,
            dropped_count=self._dropped_count,
        )

    def quiesce(self) -> TransportSnapshot:
        """原子禁止新交付；外部调用等待已开始回调，callback 内立即返回。"""
        callback_context = self._in_callback_context()
        with self._condition:
            if self._state == "open":
                self._state = "quiescing"
                for subscription in self._subscriptions:
                    subscription._active = False
                self._subscriptions.clear()
            if self._state == "quiescing" and self._total_in_flight == 0:
                self._state = "quiesced"
                self._condition.notify_all()
            if not callback_context:
                while self._state == "quiescing":
                    self._condition.wait()
                while self._state == "closing":
                    self._condition.wait()
        return self.snapshot()

    def close(self) -> None:
        """在 quiesce 屏障后发布 closed；callback 内保留防死锁语义。"""
        self.quiesce()
        callback_context = self._in_callback_context()
        with self._condition:
            if self._state == "quiescing":
                self._state = "closing"
            elif self._state == "quiesced":
                self._state = "closed"
                self._condition.notify_all()
            if self._state == "closed":
                return
            if self._total_in_flight == 0:
                self._state = "closed"
                self._condition.notify_all()
                return
            if callback_context:
                return
            while self._state != "closed":
                self._condition.wait()

    def _close_subscription(self, subscription: LocalSubscription) -> None:
        """停用单个订阅，并为外部关闭者提供 per-subscription 屏障。"""
        callback_context = self._in_callback_context()
        with self._condition:
            subscription._active = False
            try:
                self._subscriptions.remove(subscription)
            except ValueError:
                pass
            if callback_context:
                return
            while subscription._in_flight > 0:
                self._condition.wait()

    def _begin_delivery(self, subscription: LocalSubscription) -> bool:
        """原子检查生命周期并登记一次即将启动的用户回调。"""
        with self._condition:
            if self._state != "open" or not subscription._active:
                return False
            subscription._in_flight += 1
            self._total_in_flight += 1
            self._received_count += 1
            return True

    def _finish_delivery(self, subscription: LocalSubscription) -> None:
        """结束 in-flight 交付，并在全局屏障完成时关闭 transport。"""
        with self._condition:
            subscription._in_flight -= 1
            self._total_in_flight -= 1
            if self._total_in_flight == 0:
                if self._state == "quiescing":
                    self._state = "quiesced"
                elif self._state == "closing":
                    self._state = "closed"
            self._condition.notify_all()

    def _in_callback_context(self) -> bool:
        """仅识别当前实例的回调栈，避免跨 transport 误跳过外部屏障。"""
        return self in getattr(_CALLBACK_CONTEXT, "transports", ())

    def _record_error(self) -> None:
        """记录被隔离的订阅回调异常。"""
        with self._condition:
            self._error_count += 1
