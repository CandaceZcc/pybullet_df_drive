# 仿真接口时钟：提供精确仿真时间和基于累计期限的周期调度。
from __future__ import annotations

from fractions import Fraction
import math
from numbers import Real


_NANOSECONDS_PER_SECOND = 1_000_000_000
_UINT64_MAX = (1 << 64) - 1

# 单次最多补发一万个期限；超过时拒绝且不推进调度器状态。
MAX_CATCH_UP_DEADLINES = 10_000


def _positive_time_fraction(dt: object) -> Fraction:
    """把正有限时间步规范为有界分母的有理数。"""
    if isinstance(dt, bool) or not isinstance(dt, Real):
        raise ValueError("dt must be a positive finite number")

    if isinstance(dt, Fraction):
        step = dt
    elif isinstance(dt, int):
        step = Fraction(dt)
    else:
        try:
            normalized = float(dt)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("dt must be a positive finite number") from exc
        if not math.isfinite(normalized):
            raise ValueError("dt must be a positive finite number")
        step = Fraction(normalized).limit_denominator(_NANOSECONDS_PER_SECOND)

    if step <= 0:
        raise ValueError("dt must be a positive finite number")
    return step


def _require_uint64_now(now_ns: object) -> int:
    """校验调度器使用的单调 uint64 纳秒时间。"""
    if isinstance(now_ns, bool) or not isinstance(now_ns, int) or not 0 <= now_ns <= _UINT64_MAX:
        raise ValueError("now_ns must be a uint64 integer")
    return now_ns


def _round_deadline_ns(deadline_index: int, rate_hz: int) -> int:
    """用整数商余数精确实现正 Fraction 的 ties-to-even 舍入。"""
    numerator = deadline_index * _NANOSECONDS_PER_SECOND
    quotient, remainder = divmod(numerator, rate_hz)
    doubled_remainder = remainder * 2
    if doubled_remainder > rate_hz or (
        doubled_remainder == rate_hz and quotient % 2 == 1
    ):
        return quotient + 1
    return quotient


class SimulationClock:
    """仅在显式推进时变化的精确仿真时钟。"""

    def __init__(self) -> None:
        self._seconds = Fraction(0)

    @property
    def now_ns(self) -> int:
        """返回四舍五入到整数纳秒的当前仿真时间。"""
        return round(self._seconds * _NANOSECONDS_PER_SECOND)

    def _candidate_seconds(self, dt: Real | Fraction) -> Fraction:
        """校验步长和 uint64 上界，但不提交内部时钟。"""
        candidate_seconds = self._seconds + _positive_time_fraction(dt)
        if candidate_seconds * _NANOSECONDS_PER_SECOND > _UINT64_MAX:
            raise ValueError("simulation time must fit in uint64 nanoseconds")
        return candidate_seconds

    def preview_advance(self, dt: Real | Fraction) -> int:
        """返回推进候选纳秒值，任何成功或失败都不修改当前时间。"""
        return round(self._candidate_seconds(dt) * _NANOSECONDS_PER_SECOND)

    def advance(self, dt: Real | Fraction) -> int:
        """推进一个正时间步，并返回推进后的仿真纳秒时间。"""
        candidate_seconds = self._candidate_seconds(dt)
        self._seconds = candidate_seconds
        return round(candidate_seconds * _NANOSECONDS_PER_SECOND)


class PeriodicScheduler:
    """按有理数期限累计并一次弹出所有已到期时间戳。"""

    def __init__(self, rate_hz: int) -> None:
        if (
            isinstance(rate_hz, bool)
            or not isinstance(rate_hz, int)
            or not 1 <= rate_hz <= _NANOSECONDS_PER_SECOND
        ):
            raise ValueError("rate_hz must be an integer in range 1..1000000000")
        self._rate_hz = rate_hz
        self._period_seconds = Fraction(1, rate_hz)
        self._next_deadline_seconds = self._period_seconds
        self._next_deadline_index = 1
        self._last_now_ns: int | None = None

    def _calculate_due(self, now_ns: int) -> tuple[int, tuple[int, ...]]:
        """复用同一套期限计算，返回规范时间和不修改状态的到期序列。"""
        normalized_now = _require_uint64_now(now_ns)
        if self._last_now_ns is not None and normalized_now < self._last_now_ns:
            raise ValueError("now_ns must not move backwards")

        now_seconds = Fraction(normalized_now, _NANOSECONDS_PER_SECOND)
        due_count = 0
        if self._next_deadline_seconds <= now_seconds:
            due_count = (now_seconds - self._next_deadline_seconds) // self._period_seconds + 1
        if due_count > MAX_CATCH_UP_DEADLINES:
            raise ValueError(
                f"scheduler catch-up limit exceeded: {due_count} deadlines "
                f"is greater than {MAX_CATCH_UP_DEADLINES}"
            )

        # 用整数生成时间戳，完整构造结果后再原子提交 Fraction 期限。
        first_deadline_index = self._next_deadline_index
        due = tuple(
            _round_deadline_ns(deadline_index, self._rate_hz)
            for deadline_index in range(
                first_deadline_index,
                first_deadline_index + due_count,
            )
        )
        return normalized_now, due

    def preview_due(self, now_ns: int) -> tuple[int, ...]:
        """预览当前到期期限；成功或失败都不改变 scheduler 状态。"""
        _normalized_now, due = self._calculate_due(now_ns)
        return due

    def pop_due(self, now_ns: int) -> tuple[int, ...]:
        """返回当前时间跨过的全部期限，并原子提交 scheduler 状态。"""
        normalized_now, due = self._calculate_due(now_ns)
        due_count = len(due)
        self._next_deadline_index += due_count
        self._next_deadline_seconds += due_count * self._period_seconds

        self._last_now_ns = normalized_now
        return due
