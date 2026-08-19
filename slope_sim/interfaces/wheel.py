# 轮子命令邮箱：在线程间原子传递已校验命令并执行 100 ms 墙钟失效保护。
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from threading import Lock
import time

from slope_sim.interfaces.models import WheelCommand, validate_wheel_command
from slope_sim.interfaces.status import RollingFrequency, WheelCommandStatus
from slope_sim.model_registry import RobotModelSpec


def _require_wall_time(name: str, value: object, *, positive: bool = False) -> float:
    """校验墙钟和超时参数，显式拒绝 bool、NaN 与无穷值。"""
    if isinstance(value, bool) or not isinstance(value, Real):
        qualifier = "positive" if positive else "nonnegative"
        raise ValueError(f"{name} must be a {qualifier} finite number")
    normalized = float(value)
    invalid_sign = normalized <= 0.0 if positive else normalized < 0.0
    if not math.isfinite(normalized) or invalid_sign:
        qualifier = "positive" if positive else "nonnegative"
        raise ValueError(f"{name} must be a {qualifier} finite number")
    return normalized


@dataclass(frozen=True)
class WheelDecision:
    """物理主线程可直接消费的不可变轮速和安全状态。"""

    drive_wheel_speed_rad_s: tuple[float, ...]
    steering_wheel_speed_rad_s: tuple[float, ...]
    waiting: bool = False
    timed_out: bool = False


class WheelCommandMailbox:
    """串行化回调写入和物理线程读取的最新有效轮子命令。"""

    def __init__(
        self,
        model: RobotModelSpec,
        timeout_sec: float = 0.100,
        frequency_window_sec: float = 2.0,
    ) -> None:
        if not isinstance(model, RobotModelSpec):
            raise ValueError("model must be a RobotModelSpec")
        if model.controller_kind == "differential":
            drive_count, steering_count = 2, 0
        elif model.controller_kind == "active_steering":
            drive_count, steering_count = 4, 2
        else:
            raise ValueError(f"unsupported controller_kind: {model.controller_kind}")

        self._model = model
        self._timeout_sec = _require_wall_time("timeout_sec", timeout_sec, positive=True)
        normalized_frequency_window = _require_wall_time(
            "frequency_window_sec",
            frequency_window_sec,
            positive=True,
        )
        self._zero_drive = (0.0,) * drive_count
        self._zero_steering = (0.0,) * steering_count
        self._lock = Lock()
        self._latest: WheelCommand | None = None
        self._last_valid_received_at: float | None = None
        self._latest_valid_event_time: float | None = None
        self._latest_accept_time: float | None = None
        self._latest_query_time: float | None = None
        self._generation = 0
        self._valid_count = 0
        self._invalid_count = 0
        self._last_error: str | None = None
        self._state = "waiting_command"
        self._frequency = RollingFrequency(normalized_frequency_window)

    def capture_generation(self) -> int:
        """在线程安全快照中返回当前清空代际。"""
        with self._lock:
            return self._generation

    def accept(
        self,
        command: WheelCommand,
        *,
        received_at: float,
        generation: int | None = None,
    ) -> bool:
        """先完成整条命令和墙钟校验，再在单一临界区提交全部状态。"""
        if generation is not None and (
            isinstance(generation, bool) or not isinstance(generation, int) or generation < 0
        ):
            raise ValueError("generation must be a nonnegative integer")
        with self._lock:
            accepted_generation = self._generation if generation is None else generation
            if accepted_generation != self._generation:
                return False

        normalized_time = _require_wall_time("received_at", received_at)
        validation_error: str | None = None
        if not isinstance(command, WheelCommand):
            validation_error = "command must be a WheelCommand"
        else:
            try:
                validate_wheel_command(command, self._model)
            except ValueError as exc:
                validation_error = str(exc)

        with self._lock:
            if accepted_generation != self._generation:
                return False
            if self._latest_accept_time is not None and normalized_time < self._latest_accept_time:
                raise ValueError("received_at must not move backwards")

            if validation_error is not None:
                self._latest_accept_time = normalized_time
                self._invalid_count += 1
                self._last_error = validation_error
                self._state = "invalid_command"
                return False

            # RollingFrequency 只接收有效事件，并与邮箱复合状态一起受外层锁保护。
            self._frequency.record(normalized_time)
            self._latest_accept_time = normalized_time
            self._latest = command
            self._last_valid_received_at = normalized_time
            self._latest_valid_event_time = normalized_time
            self._valid_count += 1
            self._last_error = None
            self._state = "active"
            return True

    def decision(self, *, now: float | None = None) -> WheelDecision:
        """按当前墙钟返回最新命令，达到超时边界时立即输出车型零数组。"""
        query_time = time.monotonic() if now is None else now
        normalized_now = _require_wall_time("now", query_time)
        with self._lock:
            self._validate_query_time(normalized_now)
            decision = self._decision_locked(normalized_now)
            self._latest_query_time = normalized_now
            return decision

    def latest_timestamp_ns(self) -> int | None:
        """只读最新有效命令的 sender timestamp，不参与 safety query 时钟。"""
        with self._lock:
            return None if self._latest is None else self._latest.timestamp_ns

    def clear(self) -> None:
        """移除可执行命令并回到等待态，同时保留累计质量计数和频率历史。"""
        with self._lock:
            self._generation += 1
            self._latest = None
            self._last_valid_received_at = None
            self._last_error = None
            self._state = "waiting_command"

    def snapshot(self, *, now: float | None = None) -> WheelCommandStatus:
        """在同一临界区更新超时、频率并复制不可变命令状态。"""
        query_time = time.monotonic() if now is None else now
        normalized_now = _require_wall_time("now", query_time)
        with self._lock:
            self._validate_query_time(normalized_now)
            frequency_horizon = normalized_now
            if self._latest_valid_event_time is not None:
                frequency_horizon = max(frequency_horizon, self._latest_valid_event_time)
            valid_hz = self._frequency.hz(frequency_horizon)
            self._refresh_state_locked(normalized_now)
            self._latest_query_time = normalized_now
            return WheelCommandStatus(
                state=self._state,
                valid_hz=valid_hz,
                latest_timestamp_ns=(
                    self._latest.timestamp_ns if self._latest is not None else None
                ),
                valid_count=self._valid_count,
                invalid_count=self._invalid_count,
                last_error=self._last_error,
            )

    def _validate_query_time(self, now: float) -> None:
        """查询只与此前查询顺序比较，不把并发事件时间混入主循环。"""
        if self._latest_query_time is not None and now < self._latest_query_time:
            raise ValueError("now must not move backwards")

    def _is_timed_out_locked(self, now: float) -> bool:
        """严格按计划的墙钟年龄差判断 100 ms 安全边界。"""
        if self._last_valid_received_at is None:
            return False
        return now - self._last_valid_received_at >= self._timeout_sec

    def _refresh_state_locked(self, now: float) -> None:
        """按等待、非法、超时优先级刷新可观测状态。"""
        if self._latest is None:
            if self._state != "invalid_command":
                self._state = "waiting_command"
        elif self._is_timed_out_locked(now):
            self._state = "timed_out"
        elif self._state != "invalid_command":
            self._state = "active"

    def _decision_locked(self, now: float) -> WheelDecision:
        """调用方持锁时构造一个与邮箱状态一致的执行决定。"""
        self._refresh_state_locked(now)
        if self._latest is None:
            return WheelDecision(self._zero_drive, self._zero_steering, True, False)
        if self._is_timed_out_locked(now):
            return WheelDecision(self._zero_drive, self._zero_steering, False, True)
        return WheelDecision(
            self._latest.drive_wheel_speed_rad_s,
            self._latest.steering_wheel_speed_rad_s,
            False,
            False,
        )
