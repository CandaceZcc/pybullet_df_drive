# 墙钟实时节拍模块：用绝对 deadline 驱动物理循环并记录超期证据。
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from numbers import Real
import time


_DEFAULT_OVERRUN_YIELD_SEC = 0.0
RUNTIME_OBSERVATION_PERIOD_SEC = 0.050


def _finite_number(name: str, value: Real, *, allow_zero: bool) -> float:
    """把公开数值参数收窄为有限浮点数。"""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized < 0.0:
        raise ValueError(f"{name} must be a finite number")
    if not allow_zero and normalized == 0.0:
        raise ValueError(f"{name} must be positive")
    return normalized


@dataclass(frozen=True, slots=True)
class PacingSample:
    """一次帧尾等待的 deadline、工作耗时和超期证据。"""

    frame_index: int
    deadline_sec: float
    work_sec: float
    sleep_requested_sec: float
    sleep_elapsed_sec: float
    lateness_sec: float
    yield_requested_sec: float
    overrun: bool


@dataclass(frozen=True, slots=True)
class PacingStatistics:
    """从最近一次 start 起累计的墙钟节拍统计。"""

    wait_count: int
    overrun_count: int
    max_lateness_sec: float
    total_sleep_requested_sec: float
    total_sleep_elapsed_sec: float
    total_yield_requested_sec: float


@dataclass(frozen=True, slots=True)
class ControlPathSample:
    """开发期开关下的有界控制链节拍快照。"""

    loop_count: int
    max_loop_gap_sec: float
    max_send_elapsed_sec: float


class ControlPathDiagnostics:
    """汇总 GUI 主循环与 Command socket 续租耗时，不在默认路径创建或输出。"""

    def __init__(self, period_sec: Real = 1.0) -> None:
        self._period_sec = _finite_number("period_sec", period_sec, allow_zero=False)
        self._next_sample_at: float | None = None
        self._last_loop_at: float | None = None
        self._loop_count = 0
        self._max_loop_gap_sec = 0.0
        self._max_send_elapsed_sec = 0.0

    def record_loop(self, *, now: Real) -> None:
        observed_at = _finite_number("now", now, allow_zero=True)
        if self._last_loop_at is not None:
            self._max_loop_gap_sec = max(self._max_loop_gap_sec, observed_at - self._last_loop_at)
        self._last_loop_at = observed_at
        self._loop_count += 1
        if self._next_sample_at is None:
            self._next_sample_at = observed_at + self._period_sec

    def record_send(self, *, elapsed_sec: Real) -> None:
        self._max_send_elapsed_sec = max(
            self._max_send_elapsed_sec,
            _finite_number("elapsed_sec", elapsed_sec, allow_zero=True),
        )

    def sample_if_due(self, *, now: Real) -> ControlPathSample | None:
        observed_at = _finite_number("now", now, allow_zero=True)
        if self._next_sample_at is None or observed_at < self._next_sample_at:
            return None
        sample = ControlPathSample(
            loop_count=self._loop_count,
            max_loop_gap_sec=self._max_loop_gap_sec,
            max_send_elapsed_sec=self._max_send_elapsed_sec,
        )
        self._next_sample_at = observed_at + self._period_sec
        self._loop_count = 0
        self._max_loop_gap_sec = 0.0
        self._max_send_elapsed_sec = 0.0
        return sample


class RuntimeObservationCadence:
    """以低频推进 discovery，同时为每个物理帧返回新墙钟。"""

    def __init__(
        self,
        period_sec: Real = RUNTIME_OBSERVATION_PERIOD_SEC,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.period_sec = _finite_number(
            "period_sec",
            period_sec,
            allow_zero=False,
        )
        if not callable(monotonic):
            raise ValueError("monotonic must be callable")
        self._monotonic = monotonic
        self._next_observation_at: float | None = None

    @property
    def next_observation_at(self) -> float | None:
        """返回当前绝对期限，供门禁输出和测试核对。"""
        return self._next_observation_at

    def reset(self) -> None:
        """结构重建或新会话后，让下一物理帧立即重新观测。"""
        self._next_observation_at = None

    def poll_if_due(self, runtime: object) -> tuple[bool, float]:
        """到期时先 poll，并从 poll 完成墙钟重建下一期限。"""
        observed_at = _finite_number(
            "monotonic()",
            self._monotonic(),
            allow_zero=True,
        )
        next_observation_at = self._next_observation_at
        if (
            next_observation_at is not None
            and observed_at < next_observation_at
        ):
            return False, observed_at

        poll_transport = getattr(runtime, "poll_transport", None)
        if not callable(poll_transport):
            raise ValueError("runtime must implement poll_transport")
        poll_transport()
        completed_at = _finite_number(
            "monotonic()",
            self._monotonic(),
            allow_zero=True,
        )
        self._next_observation_at = completed_at + self.period_sec
        return True, completed_at


class DeadlinePacer:
    """按绝对墙钟期限节流循环，帧工作时间不会叠加成漂移。"""

    def __init__(
        self,
        period_sec: Real,
        *,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        overrun_yield_sec: Real | None = None,
    ) -> None:
        self.period_sec = _finite_number("period_sec", period_sec, allow_zero=False)
        if not callable(monotonic) or not callable(sleep):
            raise ValueError("monotonic and sleep must be callable")
        self.overrun_yield_sec = (
            _DEFAULT_OVERRUN_YIELD_SEC
            if overrun_yield_sec is None
            else _finite_number(
                "overrun_yield_sec",
                overrun_yield_sec,
                allow_zero=True,
            )
        )
        self._monotonic = monotonic
        self._sleep = sleep
        self._next_deadline: float | None = None
        self._last_wake: float | None = None
        self._wait_count = 0
        self._overrun_count = 0
        self._max_lateness_sec = 0.0
        self._total_sleep_requested_sec = 0.0
        self._total_sleep_elapsed_sec = 0.0
        self._total_yield_requested_sec = 0.0

    def start(self) -> None:
        """建立新的绝对时间轴并清空上一轮累计统计。"""
        started_at = float(self._monotonic())
        self._next_deadline = started_at + self.period_sec
        self._last_wake = started_at
        self._wait_count = 0
        self._overrun_count = 0
        self._max_lateness_sec = 0.0
        self._total_sleep_requested_sec = 0.0
        self._total_sleep_elapsed_sec = 0.0
        self._total_yield_requested_sec = 0.0

    def reset_deadline(self) -> None:
        """暂停恢复后从当前墙钟重建下一期限，不追赶暂停区间。"""
        if self._next_deadline is None:
            raise RuntimeError("deadline pacer has not started")
        resumed_at = float(self._monotonic())
        self._next_deadline = resumed_at + self.period_sec
        self._last_wake = resumed_at

    def wait_for_next_deadline(self) -> PacingSample:
        """等待当前绝对期限，并返回实际工作、休眠和迟到量。"""
        if self._next_deadline is None or self._last_wake is None:
            raise RuntimeError("deadline pacer has not started")

        deadline = self._next_deadline
        before_wait = float(self._monotonic())
        delay = deadline - before_wait
        lateness = max(0.0, -delay)
        overrun = delay < 0.0
        yield_requested = self.overrun_yield_sec if overrun else 0.0
        sleep_requested = delay if delay > 0.0 else yield_requested
        if overrun:
            # sleep(0) 让出 Python 调度权，但不会为每个超期帧追加固定欠债。
            self._sleep(sleep_requested)
        elif sleep_requested > 0.0:
            self._sleep(sleep_requested)
        after_wait = float(self._monotonic())

        frame_index = self._wait_count
        self._wait_count += 1
        if overrun:
            self._overrun_count += 1
        self._max_lateness_sec = max(self._max_lateness_sec, lateness)
        self._total_sleep_requested_sec += sleep_requested
        self._total_sleep_elapsed_sec += max(0.0, after_wait - before_wait)
        self._total_yield_requested_sec += yield_requested
        self._next_deadline = deadline + self.period_sec
        work_sec = max(0.0, before_wait - self._last_wake)
        self._last_wake = after_wait

        return PacingSample(
            frame_index=frame_index,
            deadline_sec=deadline,
            work_sec=work_sec,
            sleep_requested_sec=sleep_requested,
            sleep_elapsed_sec=max(0.0, after_wait - before_wait),
            lateness_sec=lateness,
            yield_requested_sec=yield_requested,
            overrun=overrun,
        )

    @property
    def statistics(self) -> PacingStatistics:
        """返回不可变累计值，调用方不能反向修改 pacer 状态。"""
        return PacingStatistics(
            wait_count=self._wait_count,
            overrun_count=self._overrun_count,
            max_lateness_sec=self._max_lateness_sec,
            total_sleep_requested_sec=self._total_sleep_requested_sec,
            total_sleep_elapsed_sec=self._total_sleep_elapsed_sec,
            total_yield_requested_sec=self._total_yield_requested_sec,
        )
