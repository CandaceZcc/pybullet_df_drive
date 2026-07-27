# 企业接口状态：定义不可变状态快照和单调时间窗口频率统计。
from __future__ import annotations

from collections import deque
from collections.abc import Mapping
from dataclasses import dataclass
import math
from numbers import Real
from threading import Lock
from types import MappingProxyType

from slope_sim.interfaces.models import WheelState


TOPIC_STATES = frozenset(
    {"active", "waiting_peer", "timed_out", "degraded", "disconnected", "error"}
)
COMMAND_STATES = frozenset(
    {"waiting_command", "active", "invalid_command", "timed_out", "disconnected"}
)

_DIRECTIONS = frozenset({"publish", "subscribe"})
_TRANSPORT_MODES = frozenset({"auto", "ecal", "local"})
_UINT64_MAX = (1 << 64) - 1


def _require_finite_float(name: str, value: object, *, positive: bool) -> float:
    """校验有限实数，并按字段要求限制为正数或非负数。"""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    try:
        normalized = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be a finite number")
    if positive and normalized <= 0.0:
        raise ValueError(f"{name} must be positive")
    if not positive and normalized < 0.0:
        raise ValueError(f"{name} must be nonnegative")
    return normalized


def _require_optional_uint64(name: str, value: object) -> int | None:
    """校验可缺省的 uint64 纳秒时间戳。"""
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= _UINT64_MAX:
        raise ValueError(f"{name} must be None or a uint64 integer")
    return value


def _require_nonnegative_int(name: str, value: object) -> int:
    """校验计数值并显式排除 bool。"""
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


@dataclass(frozen=True)
class TopicStatus:
    """单个发布或订阅话题的不可变运行状态。"""

    topic: str
    direction: str
    state: str
    target_hz: float
    actual_hz: float
    latest_timestamp_ns: int | None
    message_count: int
    error_count: int = 0
    dropped_count: int = 0
    detail: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.topic, str) or not self.topic:
            raise ValueError("topic must be a nonempty string")
        if not isinstance(self.direction, str) or self.direction not in _DIRECTIONS:
            raise ValueError("direction must be 'publish' or 'subscribe'")
        if not isinstance(self.state, str) or self.state not in TOPIC_STATES:
            raise ValueError("state must be a valid topic state")
        object.__setattr__(self, "target_hz", _require_finite_float("target_hz", self.target_hz, positive=True))
        object.__setattr__(self, "actual_hz", _require_finite_float("actual_hz", self.actual_hz, positive=False))
        object.__setattr__(
            self,
            "latest_timestamp_ns",
            _require_optional_uint64("latest_timestamp_ns", self.latest_timestamp_ns),
        )
        _require_nonnegative_int("message_count", self.message_count)
        _require_nonnegative_int("error_count", self.error_count)
        _require_nonnegative_int("dropped_count", self.dropped_count)
        if not isinstance(self.detail, str):
            raise ValueError("detail must be a string")


@dataclass(frozen=True)
class WheelCommandStatus:
    """轮速命令接收质量和最近有效命令的不可变状态。"""

    state: str
    valid_hz: float
    latest_timestamp_ns: int | None
    valid_count: int
    invalid_count: int
    last_error: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.state, str) or self.state not in COMMAND_STATES:
            raise ValueError("state must be a valid command state")
        object.__setattr__(self, "valid_hz", _require_finite_float("valid_hz", self.valid_hz, positive=False))
        object.__setattr__(
            self,
            "latest_timestamp_ns",
            _require_optional_uint64("latest_timestamp_ns", self.latest_timestamp_ns),
        )
        _require_nonnegative_int("valid_count", self.valid_count)
        _require_nonnegative_int("invalid_count", self.invalid_count)
        if self.last_error is not None and not isinstance(self.last_error, str):
            raise ValueError("last_error must be None or a string")


@dataclass(frozen=True)
class InterfaceStatusSnapshot:
    """供界面线程只读消费的完整企业接口状态快照。"""

    captured_at: float
    transport_mode: str
    ecal_connected: bool
    command: WheelCommandStatus
    wheel_state: WheelState | None
    topics: Mapping[str, TopicStatus]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "captured_at",
            _require_finite_float("captured_at", self.captured_at, positive=False),
        )
        if not isinstance(self.transport_mode, str) or self.transport_mode not in _TRANSPORT_MODES:
            raise ValueError("transport_mode must be 'auto', 'ecal', or 'local'")
        if not isinstance(self.ecal_connected, bool):
            raise ValueError("ecal_connected must be a bool")
        if not isinstance(self.command, WheelCommandStatus):
            raise ValueError("command must be a WheelCommandStatus")
        if self.wheel_state is not None and not isinstance(self.wheel_state, WheelState):
            raise ValueError("wheel_state must be None or a WheelState")
        if not isinstance(self.topics, Mapping):
            raise ValueError("topics must be a mapping")

        # 复制调用方映射，避免冻结快照仍被外部字典间接修改。
        copied_topics: dict[str, TopicStatus] = {}
        for key, status in self.topics.items():
            if not isinstance(key, str):
                raise ValueError("topics key must be a string")
            if not isinstance(status, TopicStatus):
                raise ValueError("topics values must be TopicStatus instances")
            if key != status.topic:
                raise ValueError("topics key must equal TopicStatus.topic")
            copied_topics[key] = status
        object.__setattr__(self, "topics", MappingProxyType(copied_topics))


class RollingFrequency:
    """使用单调墙钟事件在滑动窗口内估算实际频率。"""

    def __init__(self, window_sec: float = 2.0) -> None:
        self.window_sec = _require_finite_float("window_sec", window_sec, positive=True)
        self._events: deque[float] = deque()
        self._latest_event_timestamp: float | None = None
        self._latest_query_time: float | None = None
        self._lock = Lock()

    def _evict_before(self, now: float) -> None:
        """在调用方持锁时，仅保留闭区间 [now-window, now] 内的事件。"""
        cutoff = now - self.window_sec
        while self._events and self._events[0] < cutoff:
            self._events.popleft()

    def record(self, timestamp: float) -> None:
        """记录一个非递减的单调墙钟事件。"""
        normalized = _require_finite_float("timestamp", timestamp, positive=False)
        with self._lock:
            if self._latest_event_timestamp is not None and normalized < self._latest_event_timestamp:
                raise ValueError("timestamp must not move backwards")
            self._events.append(normalized)
            self._latest_event_timestamp = normalized
            horizon = normalized
            if self._latest_query_time is not None:
                horizon = max(horizon, self._latest_query_time)
            self._evict_before(horizon)

    def hz(self, now: float) -> float:
        """返回当前窗口频率；不足两个事件或零跨度时返回零。"""
        normalized = _require_finite_float("now", now, positive=False)
        with self._lock:
            if self._latest_event_timestamp is not None and normalized < self._latest_event_timestamp:
                raise ValueError("now must not be earlier than the latest event")
            if self._latest_query_time is not None and normalized < self._latest_query_time:
                raise ValueError("now must not move backwards")
            self._latest_query_time = normalized
            self._evict_before(normalized)
            if len(self._events) < 2:
                return 0.0
            elapsed = normalized - self._events[0]
            if elapsed == 0.0:
                return 0.0
            return (len(self._events) - 1) / elapsed
