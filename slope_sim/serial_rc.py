"""阶段五 SBUS 遥控器：有界字节流解析与安全控制门禁。"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
import time
from threading import Event, RLock, Thread
from typing import Callable

from slope_sim.interfaces.v2.runsim_session import (
    MAX_ANGULAR_VELOCITY_RAD_S,
    MAX_LINEAR_VELOCITY_M_S,
)


_FRAME_HEADER = 0x0F
_FRAME_BYTES = 25
_CHANNELS = 16
# SBUS 协议的标准有效模拟量范围；摇杆校准端点在其内，超出校准端点时饱和而非停车。
_CHANNEL_INPUT_MIN = 172
_CHANNEL_INPUT_MAX = 1811
_STICK_MIN = 282
_STICK_CENTER = 1002
_STICK_MAX = 1722
_RECOVERABLE_RC_REASONS = frozenset({"waiting_frame", "recovering_frames", "frame_timeout"})


def _normalize_stick(value: int, *, deadzone: float = 0.0) -> float:
    """按实测中位分段映射，避免物理极值被理论 SBUS 量程压缩。"""
    if value <= _STICK_CENTER:
        scaled = (value - _STICK_CENTER) / (_STICK_CENTER - _STICK_MIN)
    else:
        scaled = (value - _STICK_CENTER) / (_STICK_MAX - _STICK_CENTER)
    if abs(scaled) <= deadzone:
        return 0.0
    return max(-1.0, min(1.0, scaled))


class _StickAxisFilter:
    """短窗口中值滤波叠加变化率限制，安全停车由门禁直接绕过。"""

    def __init__(self, *, deadzone: float, max_rate_per_sec: float = 4.0) -> None:
        self._deadzone = deadzone
        self._max_rate_per_sec = max_rate_per_sec
        self._samples: deque[int] = deque(maxlen=5)
        self._stable_raw: int | None = None
        self._output: float | None = None
        self._last_at: float | None = None

    def update(self, value: int, *, now: float) -> float:
        self._samples.append(value)
        ordered = sorted(self._samples)
        median = ordered[len(ordered) // 2]
        if self._stable_raw is None or abs(median - self._stable_raw) >= 4:
            self._stable_raw = median
        target = _normalize_stick(self._stable_raw, deadzone=self._deadzone)
        if self._output is None:
            self._output = target
        elif self._last_at is None:
            # 故障后以零为起点；首帧只建立时间基准，不瞬间跳回旧速度。
            pass
        else:
            limit = self._max_rate_per_sec * max(0.0, float(now) - self._last_at)
            delta = max(-limit, min(limit, target - self._output))
            self._output += delta
        self._last_at = float(now)
        return self._output

    def reset(self, *, safe_zero: bool = False) -> None:
        self._samples.clear()
        self._stable_raw = None
        self._output = 0.0 if safe_zero else None
        self._last_at = None


@dataclass(frozen=True, slots=True)
class RcStickReadout:
    """只读遥控器测试的两个操纵杆状态，不参与解锁或车辆控制。"""

    throttle_channel: int
    steering_channel: int
    throttle_raw: int
    steering_raw: int
    throttle_normalized: float
    steering_normalized: float


def rc_stick_readout(channels: tuple[int, ...]) -> RcStickReadout:
    """提取 CH3 左杆前后和 CH1 右杆转向，并按实测中位分段归一化。"""
    if (
        not isinstance(channels, tuple)
        or len(channels) != _CHANNELS
        or any(
            type(value) is not int
            or not _CHANNEL_INPUT_MIN <= value <= _CHANNEL_INPUT_MAX
            for value in channels
        )
    ):
        raise ValueError("channels must be 16 SBUS channel values in range")

    return RcStickReadout(
        throttle_channel=3,
        steering_channel=1,
        throttle_raw=channels[2],
        steering_raw=channels[0],
        throttle_normalized=_normalize_stick(channels[2]),
        steering_normalized=_normalize_stick(channels[0]),
    )


def pyserial_opener(
    *,
    serial_import: Callable[[str], object] = import_module,
) -> Callable[[Path], object]:
    """延迟加载 pyserial，避免未启用 RC 的桌面会话受可选依赖影响。"""
    try:
        serial_module = serial_import("serial")
    except ImportError as exc:
        raise RuntimeError(
            "pyserial is required for RC input; update the slope-sim environment"
        ) from exc
    serial_constructor = getattr(serial_module, "Serial", None)
    if not callable(serial_constructor):
        raise RuntimeError("pyserial Serial constructor is unavailable")

    def open_port(path: Path) -> object:
        if not isinstance(path, Path) or not path.is_absolute():
            raise ValueError("RC serial path must be an absolute Path")
        return serial_constructor(str(path), baudrate=115200, timeout=0.02)

    return open_port


class SbusFrameParser:
    """保留跨 read 残留字节，按帧头重同步并拒绝越界通道。"""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._discarded_frame_count = 0

    @property
    def discarded_frame_count(self) -> int:
        """返回因失步或通道越界丢弃的候选帧数，供端口资格判定重置连续计数。"""
        return self._discarded_frame_count

    def feed(self, payload: bytes | bytearray | memoryview) -> tuple[tuple[int, ...], ...]:
        if not isinstance(payload, (bytes, bytearray, memoryview)):
            raise ValueError("serial payload must be bytes-like")
        self._buffer.extend(payload)
        decoded: list[tuple[int, ...]] = []
        while True:
            try:
                start = self._buffer.index(_FRAME_HEADER)
            except ValueError:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < _FRAME_BYTES:
                break
            frame = bytes(self._buffer[:_FRAME_BYTES])
            channels = self._decode(frame)
            if channels is None:
                self._discarded_frame_count += 1
                del self._buffer[0]
                continue
            del self._buffer[:_FRAME_BYTES]
            decoded.append(channels)
        return tuple(decoded)

    @staticmethod
    def _decode(frame: bytes) -> tuple[int, ...] | None:
        if len(frame) != _FRAME_BYTES or frame[0] != _FRAME_HEADER:
            return None
        bits = int.from_bytes(frame[1:23], "little")
        channels = tuple((bits >> (11 * index)) & 0x07FF for index in range(_CHANNELS))
        if any(
            value < _CHANNEL_INPUT_MIN or value > _CHANNEL_INPUT_MAX
            for value in channels
        ):
            return None
        return channels


def serial_by_id_candidates(
    by_id_directory: Path = Path("/dev/serial/by-id"),
) -> tuple[Path, ...]:
    """只返回稳定 by-id 候选，禁止把瞬态 ttyUSB 编号当成设备身份。"""
    if not isinstance(by_id_directory, Path):
        raise ValueError("by_id_directory must be a Path")
    if not by_id_directory.is_dir():
        return ()
    return tuple(sorted(
        (path for path in by_id_directory.iterdir() if not path.is_dir()),
        key=lambda path: path.name,
    ))


def qualify_rc_ports(
    candidates: tuple[Path, ...],
    *,
    opener: Callable[[Path], object],
    duration_sec: float = 2.0,
    min_valid_frames: int = 20,
    monotonic: Callable[[], float] = time.monotonic,
    ignore_io_errors: bool = True,
) -> tuple[Path, ...]:
    """逐个端口验证连续 SBUS 帧；读取超时由 opener 配置，函数不假设 pyserial 可用。"""
    if not isinstance(candidates, tuple) or any(not isinstance(path, Path) for path in candidates):
        raise ValueError("candidates must be a tuple of Path values")
    if not callable(opener) or not callable(monotonic):
        raise ValueError("opener and monotonic must be callable")
    if not isinstance(ignore_io_errors, bool):
        raise ValueError("ignore_io_errors must be a bool")
    if duration_sec <= 0.0 or min_valid_frames <= 0:
        raise ValueError("duration_sec and min_valid_frames must be positive")
    qualified: list[Path] = []
    for path in candidates:
        reader = None
        close = None
        parser = SbusFrameParser()
        consecutive = 0
        discarded = 0
        deadline = float(monotonic()) + duration_sec
        try:
            reader = opener(path)
            read = getattr(reader, "read", None)
            close = getattr(reader, "close", None)
            if not callable(read):
                raise ValueError("serial opener result must provide read")
            while float(monotonic()) < deadline:
                payload = read(256)
                frames = parser.feed(payload)
                if parser.discarded_frame_count != discarded:
                    discarded = parser.discarded_frame_count
                    consecutive = 0
                consecutive += len(frames)
                if consecutive >= min_valid_frames:
                    qualified.append(path)
                    break
        except OSError:
            if not ignore_io_errors:
                raise
        finally:
            if callable(close):
                try:
                    close()
                except OSError:
                    if not ignore_io_errors:
                        raise
    return tuple(qualified)


def select_rc_port(
    candidates: tuple[Path, ...],
    *,
    opener: Callable[[Path], object],
    explicit_path: Path | None = None,
    duration_sec: float = 2.0,
    min_valid_frames: int = 20,
    monotonic: Callable[[], float] = time.monotonic,
) -> Path | None:
    """返回唯一合格端口；显式路径也必须通过相同协议资格判定。"""
    targets = (explicit_path,) if explicit_path is not None else candidates
    if explicit_path is not None and not isinstance(explicit_path, Path):
        raise ValueError("explicit_path must be a Path or None")
    qualified = qualify_rc_ports(
        targets,
        opener=opener,
        duration_sec=duration_sec,
        min_valid_frames=min_valid_frames,
        monotonic=monotonic,
        ignore_io_errors=explicit_path is None,
    )
    if not qualified:
        return None
    if len(qualified) > 1:
        names = ", ".join(path.name for path in qualified)
        raise RuntimeError(f"multiple qualified RC ports: {names}")
    return qualified[0]


@dataclass(frozen=True, slots=True)
class RcCommand:
    """串口 worker 交给受监管 IPC 的归一化候选命令，未接管时恒为零。"""

    linear_velocity_m_s: float
    angular_velocity_rad_s: float
    active: bool
    reason: str


@dataclass(frozen=True, slots=True)
class CommandSourceSnapshot:
    """唯一 Command IPC 前的控制源状态，供 Dashboard 显示和故障追溯。"""

    active_source: str | None
    failure_reason: str | None
    latest_target: tuple[float, float] = (0.0, 0.0)
    mailbox_update_count: int = 0
    command_send_count: int = 0
    renewal_count: int = 0
    last_renewal_age_sec: float | None = None
    max_renewal_gap_sec: float | None = None
    renewal_hz: float | None = None
    zero_reason: str | None = None


class CommandSourceArbiter:
    """把多个候选收敛到唯一 Command socket，切源和撤销总是先归零。"""

    _SOURCES = frozenset(("keyboard", "rc", "external"))

    def __init__(
        self,
        command_client: object,
        *,
        renewal_hz: float | None = None,
        start_renewer: bool = True,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if not callable(getattr(command_client, "send_target", None)):
            raise ValueError("command_client must provide send_target")
        if renewal_hz is not None and (
            isinstance(renewal_hz, bool)
            or not isinstance(renewal_hz, (int, float))
            or renewal_hz <= 0.0
        ):
            raise ValueError("renewal_hz must be positive or None")
        if not isinstance(start_renewer, bool) or not callable(monotonic):
            raise ValueError("start_renewer and monotonic must be valid")
        self._command_client = command_client
        self._lock = RLock()
        self._active_source: str | None = None
        self._failure_reason: str | None = None
        self._latest_target = (0.0, 0.0)
        self._mailbox_update_count = 0
        self._command_send_count = 0
        self._renewal_count = 0
        self._last_renewal_at: float | None = None
        self._max_renewal_gap_sec: float | None = None
        self._zero_reason: str | None = None
        self._renewal_hz = None if renewal_hz is None else float(renewal_hz)
        self._renewal_period_sec = (
            None if self._renewal_hz is None else 1.0 / self._renewal_hz
        )
        self._monotonic = monotonic
        self._renewal_stop = Event()
        self._renewal_thread: Thread | None = None
        if self._renewal_period_sec is not None and start_renewer:
            self._renewal_thread = Thread(
                target=self._run_renewer,
                name="command-target-renewer",
                daemon=True,
            )
            self._renewal_thread.start()

    def select_source(self, source: str, *, now: float) -> None:
        """用户显式选择输入源；不保留上一个源的运动目标。"""
        if source not in self._SOURCES:
            raise ValueError(f"unsupported command source: {source}")
        with self._lock:
            previous_source = self._active_source or "none"
            self._zero_reason = f"source_switch:{previous_source}->{source}"
            self._send_zero(now)
            self._active_source = source
            self._failure_reason = None

    def submit_keyboard(
        self,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
        *,
        now: float,
    ) -> bool:
        """仅在键盘拥有控制权时续租其目标。"""
        return self._submit(
            "keyboard", linear_velocity_m_s, angular_velocity_rad_s, now=now
        )

    def submit_external(
        self,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
        *,
        now: float,
    ) -> bool:
        """仅在外部命令拥有控制权时续租其目标。"""
        return self._submit(
            "external", linear_velocity_m_s, angular_velocity_rad_s, now=now
        )

    def submit_rc(self, command: RcCommand, *, now: float) -> bool:
        """RC 软超时只停车不撤销选择，新鲜帧恢复后自动续行。"""
        if not isinstance(command, RcCommand):
            raise ValueError("command must be an RcCommand")
        with self._lock:
            if self._active_source != "rc":
                return False
            if not command.active:
                if command.reason in _RECOVERABLE_RC_REASONS:
                    if self._failure_reason != command.reason:
                        self._zero_reason = command.reason
                        self._send_zero(now)
                    else:
                        self._latest_target = (0.0, 0.0)
                    self._failure_reason = command.reason
                else:
                    self._revoke(command.reason, now=now)
                return False
            self._publish_or_store_target(
                command.linear_velocity_m_s,
                command.angular_velocity_rad_s,
                now=now,
            )
            self._failure_reason = None
            return True

    def submit_fault(self, reason: str, *, now: float) -> None:
        """worker、串口或本机 IPC 故障立即撤销控制权。"""
        if not reason:
            raise ValueError("reason must be non-empty")
        with self._lock:
            self._revoke(reason, now=now)

    def snapshot(self, *, now: float | None = None) -> CommandSourceSnapshot:
        """返回不可变状态，避免 Dashboard 读取时竞态。"""
        observed_at = float(self._monotonic() if now is None else now)
        with self._lock:
            last_renewal_age = (
                None
                if self._last_renewal_at is None
                else max(0.0, observed_at - self._last_renewal_at)
            )
            return CommandSourceSnapshot(
                self._active_source,
                self._failure_reason,
                self._latest_target,
                self._mailbox_update_count,
                self._command_send_count,
                self._renewal_count,
                last_renewal_age,
                self._max_renewal_gap_sec,
                self._renewal_hz,
                self._zero_reason,
            )

    def renew_once(self, *, now: float) -> None:
        """按固定节拍发送容量 1 mailbox 中的最新目标。"""
        with self._lock:
            if self._renewal_period_sec is None:
                return
            self._send_target(*self._latest_target, now=now)
            if self._last_renewal_at is not None:
                gap = max(0.0, float(now) - self._last_renewal_at)
                self._max_renewal_gap_sec = (
                    gap
                    if self._max_renewal_gap_sec is None
                    else max(self._max_renewal_gap_sec, gap)
                )
            self._last_renewal_at = float(now)
            self._renewal_count += 1

    def close(self, *, now: float | None = None) -> None:
        """停止续租线程并同步发送最终零命令。"""
        if self._renewal_stop.is_set():
            return
        self._renewal_stop.set()
        if self._renewal_thread is not None:
            self._renewal_thread.join(timeout=1.0)
        observed_at = float(self._monotonic() if now is None else now)
        with self._lock:
            self._zero_reason = "closed"
            self._send_zero(observed_at)
            self._active_source = None

    def _submit(
        self,
        source: str,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
        *,
        now: float,
    ) -> bool:
        with self._lock:
            if self._active_source != source:
                return False
            self._publish_or_store_target(
                linear_velocity_m_s,
                angular_velocity_rad_s,
                now=now,
            )
            return True

    def _revoke(self, reason: str, *, now: float) -> None:
        try:
            self._zero_reason = reason
            self._send_zero(now)
        finally:
            self._active_source = None
            self._failure_reason = reason

    def _send_zero(self, now: float) -> None:
        self._latest_target = (0.0, 0.0)
        self._send_target(0.0, 0.0, now=now)

    def _publish_or_store_target(
        self,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
        *,
        now: float,
    ) -> None:
        self._latest_target = (linear_velocity_m_s, angular_velocity_rad_s)
        self._mailbox_update_count += 1
        self._zero_reason = None
        if self._renewal_period_sec is None:
            self._send_target(linear_velocity_m_s, angular_velocity_rad_s, now=now)

    def _run_renewer(self) -> None:
        assert self._renewal_period_sec is not None
        next_deadline = float(self._monotonic()) + self._renewal_period_sec
        while not self._renewal_stop.wait(
            max(0.0, next_deadline - float(self._monotonic()))
        ):
            observed_at = float(self._monotonic())
            try:
                self.renew_once(now=observed_at)
            except Exception:
                self._renewal_stop.set()
                return
            next_deadline += self._renewal_period_sec
            if next_deadline <= observed_at:
                next_deadline = observed_at + self._renewal_period_sec

    def _send_target(
        self,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
        *,
        now: float,
    ) -> None:
        try:
            self._command_client.send_target(
                linear_velocity_m_s, angular_velocity_rad_s, now=now
            )
            self._command_send_count += 1
        except Exception:
            self._active_source = None
            self._failure_reason = "ipc_interrupted"
            raise


@dataclass(frozen=True, slots=True)
class RcWorkerSnapshot:
    """Dashboard/IPC 可读取的串口健康摘要，不暴露 serial 对象或可变 buffer。"""

    path: Path
    actual_hz: float | None
    last_frame_age_sec: float | None
    last_channels: tuple[int, ...] | None
    command: RcCommand
    failure_reason: str | None
    watchdog_timeout_count: int = 0


class RcCommandGate:
    """把新鲜 CH1/CH3 帧映射为控制目标，并在持续失联时安全停车。"""

    def __init__(
        self,
        *,
        timeout_sec: float = 0.2,
        deadzone: float = 0.05,
        recovery_frames: int = 3,
    ) -> None:
        if timeout_sec <= 0.0 or not 0.0 <= deadzone < 1.0:
            raise ValueError("timeout_sec and deadzone must be valid")
        if isinstance(recovery_frames, bool) or not isinstance(recovery_frames, int) or recovery_frames <= 0:
            raise ValueError("recovery_frames must be a positive integer")
        self._timeout_sec = timeout_sec
        self._deadzone = deadzone
        self._recovery_frames = recovery_frames
        self._last_frame_at: float | None = None
        # 端口资格检查已经验证至少 20 帧；只有真实故障后才重新累计恢复帧。
        self._fresh_frame_count = recovery_frames
        self._throttle_filter = _StickAxisFilter(deadzone=deadzone)
        self._steering_filter = _StickAxisFilter(deadzone=deadzone)
        self._active = False
        self._last = self._zero("waiting_frame")

    def observe(self, channels: tuple[int, ...], *, now: float) -> RcCommand:
        if not isinstance(channels, tuple) or len(channels) != _CHANNELS:
            return self.fault("invalid_channels")
        if any(
            type(value) is not int
            or not _CHANNEL_INPUT_MIN <= value <= _CHANNEL_INPUT_MAX
            for value in channels
        ):
            return self.fault("channel_out_of_range")
        self._last_frame_at = float(now)
        self._fresh_frame_count = min(self._recovery_frames, self._fresh_frame_count + 1)
        filtered_throttle = self._throttle_filter.update(channels[2], now=now)
        filtered_steering = self._steering_filter.update(channels[0], now=now)
        if self._fresh_frame_count < self._recovery_frames:
            self._active = False
            self._last = self._zero("recovering_frames")
            return self._last
        self._active = True
        self._last = RcCommand(
            MAX_LINEAR_VELOCITY_M_S
            * filtered_throttle,
            -MAX_ANGULAR_VELOCITY_RAD_S
            * filtered_steering,
            True,
            "active",
        )
        return self._last

    def decision(self, *, now: float) -> RcCommand:
        if self._last_frame_at is None:
            return self._last
        if float(now) - self._last_frame_at >= self._timeout_sec - 1e-12:
            self._active = False
            self._fresh_frame_count = 0
            self._reset_filters()
            self._last = self._zero("frame_timeout")
        return self._last

    def fault(self, reason: str) -> RcCommand:
        self._active = False
        self._fresh_frame_count = 0
        self._reset_filters()
        self._last = self._zero(reason)
        return self._last

    def _reset_filters(self) -> None:
        self._throttle_filter.reset(safe_zero=True)
        self._steering_filter.reset(safe_zero=True)

    def _zero(self, reason: str) -> RcCommand:
        return RcCommand(0.0, 0.0, False, reason)


class SerialRcWorker:
    """串口读取线程只生成候选命令；sink 必须是受监管的本机 Command IPC。"""

    def __init__(
        self,
        path: Path,
        *,
        reader: object,
        command_sink: Callable[[RcCommand, float], None],
        monotonic: Callable[[], float] = time.monotonic,
        start_worker: bool = True,
    ) -> None:
        if not isinstance(path, Path):
            raise ValueError("path must be a Path")
        if not callable(getattr(reader, "read", None)):
            raise ValueError("reader must provide read")
        if not callable(command_sink) or not callable(monotonic):
            raise ValueError("command_sink and monotonic must be callable")
        if not isinstance(start_worker, bool):
            raise ValueError("start_worker must be a bool")
        self._path = path
        self._reader = reader
        self._command_sink = command_sink
        self._monotonic = monotonic
        self._parser = SbusFrameParser()
        self._gate = RcCommandGate()
        self._lock = RLock()
        self._frame_times: deque[float] = deque()
        self._last_frame_at: float | None = None
        self._last_channels: tuple[int, ...] | None = None
        self._last_command = self._gate.decision(now=0.0)
        self._failure_reason: str | None = None
        self._watchdog_timeout_count = 0
        self._stop = Event()
        self._thread: Thread | None = None
        if start_worker:
            self._thread = Thread(target=self._run, name="serial-rc-worker", daemon=True)
            self._thread.start()

    def process_once(self, *, now: float | None = None) -> RcCommand:
        """读取一批字节并更新 watchdog；可由 worker 和确定性测试共用。"""
        observed_at = float(self._monotonic() if now is None else now)
        try:
            payload = self._reader.read(256)
        except Exception as error:
            return self._submit_fault("serial_read_failed", observed_at, error)
        with self._lock:
            discarded_before = self._parser.discarded_frame_count
            try:
                frames = self._parser.feed(payload)
            except ValueError as error:
                return self._submit_fault("serial_payload_invalid", observed_at, error)
            if frames:
                for channels in frames:
                    self._last_channels = channels
                    self._last_frame_at = observed_at
                    self._frame_times.append(observed_at)
                    command = self._gate.observe(channels, now=observed_at)
                self._failure_reason = None if command.active else command.reason
            elif self._parser.discarded_frame_count != discarded_before:
                command = self._gate.fault("parser_desynchronized")
                self._failure_reason = command.reason
            else:
                command = self._gate.decision(now=observed_at)
                self._failure_reason = None if command.active else command.reason
            if command.reason == "frame_timeout" and self._last_command.reason != "frame_timeout":
                self._watchdog_timeout_count += 1
            while self._frame_times and self._frame_times[0] < observed_at - 2.0:
                self._frame_times.popleft()
            self._last_command = command
        try:
            self._command_sink(command, observed_at)
        except Exception as error:
            return self._submit_fault("ipc_interrupted", observed_at, error, send=False)
        return command

    def snapshot(self, *, now: float | None = None) -> RcWorkerSnapshot:
        """返回有界诊断状态；读取不触发新的控制命令。"""
        observed_at = float(self._monotonic() if now is None else now)
        with self._lock:
            events = tuple(timestamp for timestamp in self._frame_times if timestamp >= observed_at - 2.0)
            actual_hz = (
                (len(events) - 1) / (events[-1] - events[0])
                if len(events) >= 3 and events[-1] > events[0]
                else None
            )
            age = None if self._last_frame_at is None else max(0.0, observed_at - self._last_frame_at)
            return RcWorkerSnapshot(
                self._path, actual_hz, age, self._last_channels,
                self._last_command, self._failure_reason, self._watchdog_timeout_count,
            )

    def _submit_fault(
        self,
        reason: str,
        observed_at: float,
        error: Exception,
        *,
        send: bool = True,
    ) -> RcCommand:
        with self._lock:
            command = self._gate.fault(reason)
            self._last_command = command
            self._failure_reason = f"{reason}: {type(error).__name__}: {error}"
        if send:
            try:
                self._command_sink(command, observed_at)
            except Exception:
                pass
        return command

    def _run(self) -> None:
        while not self._stop.is_set():
            self.process_once()

    def close(self) -> None:
        """先撤销 control candidate，再关闭 reader 以解除可能阻塞的 read。"""
        if self._stop.is_set():
            return
        self._stop.set()
        now = float(self._monotonic())
        self._submit_fault("worker_closed", now, RuntimeError("serial RC worker closed"))
        close = getattr(self._reader, "close", None)
        if callable(close):
            close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


def start_rc_worker(
    *,
    command_sink: Callable[[RcCommand, float], None],
    opener: Callable[[Path], object],
    explicit_path: Path | None = None,
    by_id_directory: Path = Path("/dev/serial/by-id"),
    monotonic: Callable[[], float] = time.monotonic,
    start_worker: bool = True,
) -> SerialRcWorker:
    """在唯一稳定端口通过 SBUS 资格判定后，才启动真实 RC worker。"""
    if explicit_path is not None and not isinstance(explicit_path, Path):
        raise ValueError("explicit_path must be a Path or None")
    selected = select_rc_port(
        serial_by_id_candidates(by_id_directory),
        opener=opener,
        explicit_path=explicit_path,
        monotonic=monotonic,
    )
    if selected is None:
        target = explicit_path if explicit_path is not None else by_id_directory
        raise RuntimeError(f"no qualified RC SBUS port: {target}")
    reader = opener(selected)
    try:
        return SerialRcWorker(
            selected,
            reader=reader,
            command_sink=command_sink,
            monotonic=monotonic,
            start_worker=start_worker,
        )
    except BaseException:
        close = getattr(reader, "close", None)
        if callable(close):
            close()
        raise
