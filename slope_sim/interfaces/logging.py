# 企业接口日志：保存内部 Protobuf 二进制帧，并提供可验证的读取模型。
from __future__ import annotations

from collections.abc import Callable, Iterator
from contextlib import suppress
from dataclasses import dataclass
import errno
import json
import math
from numbers import Real
import os
from pathlib import Path
from queue import Empty, Queue
import struct
from threading import Condition, Lock, Semaphore, Thread, current_thread
from time import time_ns
from typing import BinaryIO, TextIO
from uuid import uuid4

from google.protobuf.message import DecodeError

from slope_sim.interfaces.generated import slope_sim_internal_pb2 as internal_pb


_UINT64_MAX = (1 << 64) - 1
_UINT32_MAX = (1 << 32) - 1
_FRAME_PREFIX = struct.Struct("<I")
_DIRECTIONS = frozenset({"receive", "publish"})
MAX_INTERFACE_LOG_FRAME_BYTES = 64 * 1024 * 1024
DEFAULT_INTERFACE_LOG_MAX_RECORDS = 1_000_000
DEFAULT_INTERFACE_LOG_MAX_TOTAL_BYTES = 1024 * 1024 * 1024
INTERFACE_EVENT_TYPES = frozenset(
    {
        "protobuf_parse_failed",
        "invalid_command",
        "command_timeout",
        "model_mismatch",
        "mechanical_limit",
        "ecal_initialized",
        "ecal_disconnected",
        "ecal_reconnected",
        "ecal_closed",
        "sensor_failed",
        "publish_failed",
        "queue_dropped",
    }
)
_EVENT_UINT64_FIELDS = frozenset({"wall_time_ns", "sim_time_ns"})
_EVENT_TEXT_FIELDS = frozenset(
    {"robot_model", "terrain_model", "topic", "reason"}
)
_UNSAFE_PREFIX_CHARS = frozenset('/\\\x00<>:"|?*')
_WINDOWS_RESERVED_NAMES = frozenset(
    {
        "CON",
        "PRN",
        "AUX",
        "NUL",
        *(f"COM{i}" for i in range(1, 10)),
        *(f"LPT{i}" for i in range(1, 10)),
    }
)
_UNSETTLED_STREAMS: list[object] = []
_UNSETTLED_STREAMS_LOCK = Lock()


def _require_uint64(name: str, value: object) -> int:
    """严格校验内部 envelope 的 uint64 字段。"""
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= _UINT64_MAX:
        raise ValueError(f"{name} must be a uint64 integer")
    return value


def _require_nonempty_text(name: str, value: object) -> str:
    """拒绝空文本和非字符串接口标识。"""
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a nonempty string")
    return value


def _require_nonnegative_int(name: str, value: object) -> int:
    """校验日志累计计数并排除 bool。"""
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _require_nonnegative_timeout(value: object) -> float:
    """校验关闭期等待边界，拒绝 bool、负数和非有限值。"""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError("timeout_sec must be a nonnegative finite number")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized < 0.0:
        raise ValueError("timeout_sec must be a nonnegative finite number")
    return normalized


def _require_positive_limit(
    name: str,
    value: object,
    *,
    maximum: int | None = None,
) -> int:
    """严格校验 reader 的有限正整数边界。"""
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"interface log {name} must be a positive integer")
    if maximum is not None and value > maximum:
        raise ValueError(f"interface log {name} exceeds its supported maximum")
    return value


def _varint_size(value: int) -> int:
    """返回非负整数在 Protobuf varint 中占用的字节数。"""
    return max(1, (value.bit_length() + 6) // 7)


def _uint64_field_size(value: int) -> int:
    """计算单字节 tag 的 proto3 uint64 字段实际 wire size。"""
    return 0 if value == 0 else 1 + _varint_size(value)


def _length_delimited_field_size(length: int) -> int:
    """计算单字节 tag 的非空 string/bytes 字段 wire size。"""
    return 0 if length == 0 else 1 + _varint_size(length) + length


def _record_envelope_size(record: InterfaceLogRecord) -> int:
    """不复制 payload，精确计算内部 envelope 的序列化长度。"""
    return sum(
        (
            _uint64_field_size(record.sequence),
            _length_delimited_field_size(len(record.topic.encode("utf-8"))),
            _length_delimited_field_size(len(record.direction.encode("utf-8"))),
            _uint64_field_size(record.sim_time_ns),
            _uint64_field_size(record.wall_time_ns),
            _length_delimited_field_size(len(record.type_name.encode("utf-8"))),
            _length_delimited_field_size(len(record.payload)),
        )
    )


def _require_safe_prefix(value: object) -> str:
    """把 prefix 限制为单个跨平台安全文件名前缀。"""
    if not isinstance(value, str):
        raise ValueError("prefix must be a safe nonempty string")
    try:
        encoded = value.encode("utf-8")
    except UnicodeError as exc:
        raise ValueError("prefix must be valid UTF-8 text") from exc
    reserved_stem = value.split(".", 1)[0].upper()
    if (
        not value
        or value != value.strip()
        or value in {".", ".."}
        or value.endswith(".")
        or len(encoded) > 128
        or reserved_stem in _WINDOWS_RESERVED_NAMES
        or any(character in _UNSAFE_PREFIX_CHARS or ord(character) < 32 for character in value)
    ):
        raise ValueError("prefix must be a safe nonempty filename prefix")
    return value


def _require_log_directory(value: object) -> Path:
    """仅接受显式非空字符串或 Path 日志目录。"""
    if isinstance(value, str):
        if not value:
            raise ValueError("interface log directory must be a nonempty path")
        return Path(value)
    if isinstance(value, Path):
        return value
    raise ValueError("interface log directory must be a string or Path")


def _ensure_log_directory(directory: Path) -> None:
    """原子创建可共享目录；不推断所有权，也不在实例失败时删除。"""
    try:
        directory.mkdir(parents=True, exist_ok=True)
    except FileExistsError as exc:
        raise ValueError("interface log directory must be a directory") from exc
    if not directory.is_dir():
        raise ValueError("interface log directory must be a directory")


def _new_log_paths(directory: Path, prefix: str) -> InterfaceLogPaths:
    """生成不覆盖旧运行日志的同 stem 成对路径。"""
    run_id = f"{time_ns()}_{uuid4().hex[:8]}"
    stem = f"{prefix}_{run_id}"
    return InterfaceLogPaths(
        directory / f"{stem}.interfaces.bin",
        directory / f"{stem}.events.jsonl",
    )


@dataclass(frozen=True)
class _CapturedStreamFd:
    """文件描述符及私有副本，用于识别同一次文件打开。"""

    number: int
    guard_number: int
    device: int
    inode: int


def _capture_stream_fd(stream: BinaryIO | TextIO) -> _CapturedStreamFd:
    """复制一个不对外暴露的 fd，稳定持有原 open-file-description。"""
    fd = stream.fileno()
    if isinstance(fd, bool) or not isinstance(fd, int) or fd < 0:
        raise OSError("interface log stream has no usable file descriptor")
    guard_fd = os.dup(fd)
    try:
        stat = os.fstat(guard_fd)
    except BaseException:
        with suppress(OSError):
            os.close(guard_fd)
        raise
    return _CapturedStreamFd(fd, guard_fd, int(stat.st_dev), int(stat.st_ino))


def _captured_fd_is_current(captured: _CapturedStreamFd) -> bool:
    """用两次共享偏移探针确认候选 fd 仍是原打开实例。"""
    guard_offset: int | None = None
    try:
        guard_stat = os.fstat(captured.guard_number)
        candidate_stat = os.fstat(captured.number)
        expected_identity = (captured.device, captured.inode)
        if (
            (int(guard_stat.st_dev), int(guard_stat.st_ino)) != expected_identity
            or (int(candidate_stat.st_dev), int(candidate_stat.st_ino))
            != expected_identity
        ):
            return False

        # dup 共享文件偏移；同 inode 的独立 open 不会跟随两次不同探针。
        guard_offset = os.lseek(captured.guard_number, 0, os.SEEK_CUR)
        first_probe = 1 if guard_offset != 1 else 2
        second_probe = 3 if first_probe != 3 else 4
        os.lseek(captured.guard_number, first_probe, os.SEEK_SET)
        if os.lseek(captured.number, 0, os.SEEK_CUR) != first_probe:
            return False
        os.lseek(captured.guard_number, second_probe, os.SEEK_SET)
        return os.lseek(captured.number, 0, os.SEEK_CUR) == second_probe
    except OSError:
        # 无法证明所有权时宁可保留候选 fd，也不能误关复用后的资源。
        return False
    finally:
        if guard_offset is not None:
            with suppress(OSError):
                os.lseek(captured.guard_number, guard_offset, os.SEEK_SET)


def _stream_is_closed(stream: BinaryIO | TextIO) -> bool:
    """安全读取流关闭状态；不合作的包装流按未关闭处理。"""
    try:
        return bool(stream.closed)
    except BaseException:
        return False


def _run_stream_finalizer(stream: BinaryIO | TextIO) -> None:
    """在 fd 尚未复用时让底层文件对象稳定进入 closed 状态。"""
    try:
        finalizer = getattr(stream, "__del__", None)
        if callable(finalizer):
            finalizer()
    except BaseException:
        pass


def _close_stream_with_fd(
    stream: BinaryIO | TextIO,
    fd: _CapturedStreamFd | None,
) -> BaseException | None:
    """先关闭高层流，仅在所有权确认后兜底，并始终释放私有 guard。"""
    close_error: BaseException | None = None
    try:
        try:
            stream.close()
        except BaseException as exc:
            close_error = exc

        if not _stream_is_closed(stream):
            _run_stream_finalizer(stream)
        needs_fd_close = not _stream_is_closed(stream)
        if needs_fd_close and fd is not None and _captured_fd_is_current(fd):
            try:
                os.close(fd.number)
            except OSError as exc:
                if exc.errno != errno.EBADF and close_error is None:
                    close_error = exc
        if not _stream_is_closed(stream):
            # 隔离不合作包装流，禁止其析构误关未来复用的同号 fd。
            with _UNSETTLED_STREAMS_LOCK:
                _UNSETTLED_STREAMS.append(stream)
    finally:
        if fd is not None:
            try:
                os.close(fd.guard_number)
            except OSError as exc:
                if exc.errno != errno.EBADF and close_error is None:
                    close_error = exc
    return close_error


@dataclass(frozen=True)
class InterfaceLogRecord:
    """一条企业消息及其方向、双时钟和原始 Protobuf 字节。"""

    sequence: int
    topic: str
    direction: str
    sim_time_ns: int
    wall_time_ns: int
    type_name: str
    payload: bytes

    def __post_init__(self) -> None:
        object.__setattr__(self, "sequence", _require_uint64("sequence", self.sequence))
        object.__setattr__(self, "topic", _require_nonempty_text("topic", self.topic))
        if not isinstance(self.direction, str) or self.direction not in _DIRECTIONS:
            raise ValueError("direction must be 'receive' or 'publish'")
        object.__setattr__(
            self,
            "sim_time_ns",
            _require_uint64("sim_time_ns", self.sim_time_ns),
        )
        object.__setattr__(
            self,
            "wall_time_ns",
            _require_uint64("wall_time_ns", self.wall_time_ns),
        )
        object.__setattr__(
            self,
            "type_name",
            _require_nonempty_text("type_name", self.type_name),
        )
        if not isinstance(self.payload, (bytes, bytearray, memoryview)):
            raise ValueError("payload must be bytes-like")
        object.__setattr__(self, "payload", bytes(self.payload))


@dataclass(frozen=True)
class InterfaceLogPaths:
    """成对返回二进制消息与 UTF-8 事件日志路径。"""

    binary_path: Path
    event_path: Path

    def __post_init__(self) -> None:
        if not isinstance(self.binary_path, Path):
            raise ValueError("binary_path must be a Path")
        if not isinstance(self.event_path, Path):
            raise ValueError("event_path must be a Path")
        if self.binary_path == self.event_path:
            raise ValueError("interface log paths must be distinct")


@dataclass(frozen=True)
class InterfaceLogSnapshot:
    """接口日志接受、丢弃及终态计数的不可变快照。"""

    accepted_messages: int
    accepted_events: int
    dropped_messages: int
    dropped_events: int
    closed: bool
    writer_failed: bool

    def __post_init__(self) -> None:
        for name in (
            "accepted_messages",
            "accepted_events",
            "dropped_messages",
            "dropped_events",
        ):
            _require_nonnegative_int(name, getattr(self, name))
        if not isinstance(self.closed, bool):
            raise ValueError("closed must be a bool")
        if not isinstance(self.writer_failed, bool):
            raise ValueError("writer_failed must be a bool")


def _serialize_record(record: InterfaceLogRecord) -> bytes:
    """把不可变记录编码为固定小端长度前缀帧。"""
    envelope = internal_pb.InterfaceLogEnvelope(
        sequence=record.sequence,
        topic=record.topic,
        direction=record.direction,
        sim_time_ns=record.sim_time_ns,
        wall_time_ns=record.wall_time_ns,
        type_name=record.type_name,
        payload=record.payload,
    )
    payload = envelope.SerializeToString(deterministic=True)
    if len(payload) > MAX_INTERFACE_LOG_FRAME_BYTES:
        raise ValueError("interface log frame exceeds the configured frame limit")
    return _FRAME_PREFIX.pack(len(payload)) + payload


def _record_from_envelope(envelope: internal_pb.InterfaceLogEnvelope) -> InterfaceLogRecord:
    """由解析后的内部 envelope 重新执行模型边界校验。"""
    return InterfaceLogRecord(
        sequence=envelope.sequence,
        topic=envelope.topic,
        direction=envelope.direction,
        sim_time_ns=envelope.sim_time_ns,
        wall_time_ns=envelope.wall_time_ns,
        type_name=envelope.type_name,
        payload=envelope.payload,
    )


def _validate_event_header(event: object, fields: dict[str, object]) -> None:
    """在容量预留前仅校验固定成本的事件名和常用上下文字段。"""
    if not isinstance(event, str) or not event:
        raise ValueError("interface log event must be a nonempty string")

    try:
        for name in _EVENT_UINT64_FIELDS & fields.keys():
            _require_uint64(name, fields[name])
        for name in _EVENT_TEXT_FIELDS & fields.keys():
            _require_nonempty_text(name, fields[name])
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("interface log event fields must be JSON serializable") from exc


def _require_string_json_keys(value: object, active: set[int]) -> None:
    """递归拒绝 JSON 对象中的非字符串键，并避开循环容器无限递归。"""
    if isinstance(value, dict):
        identity = id(value)
        if identity in active:
            return
        active.add(identity)
        try:
            for key, child in value.items():
                if not isinstance(key, str):
                    raise ValueError(
                        "interface log event dict keys must all be strings"
                    )
                _require_string_json_keys(child, active)
        finally:
            active.remove(identity)
    elif isinstance(value, (list, tuple)):
        identity = id(value)
        if identity in active:
            return
        active.add(identity)
        try:
            for child in value:
                _require_string_json_keys(child, active)
        finally:
            active.remove(identity)


def _serialize_event(event: object, fields: dict[str, object]) -> str:
    """深度序列化事件字段，并生成稳定 UTF-8 JSON 行。"""
    _validate_event_header(event, fields)
    _require_string_json_keys(fields, set())
    try:
        return json.dumps(
            {"event": event, **fields},
            ensure_ascii=False,
            sort_keys=True,
            allow_nan=False,
        ) + "\n"
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("interface log event fields must be JSON serializable") from exc


@dataclass(frozen=True)
class _MessageItem:
    """已序列化的二进制消息队列项。"""

    frame: bytes


@dataclass(frozen=True)
class _EventItem:
    """已验证且自带换行符的 JSONL 队列项。"""

    line: str


_STOP = object()
_Writer = Callable[[BinaryIO | TextIO, bytes | str], object]


def _default_writer(stream: BinaryIO | TextIO, data: bytes | str) -> object:
    """执行真实文件写入；可注入 writer 仅用于隔离阻塞和失败测试。"""
    return stream.write(data)  # type: ignore[arg-type]


def _write_all(
    writer: _Writer,
    stream: BinaryIO | TextIO,
    data: bytes | str,
) -> None:
    """循环处理合法短写，并把零进展或非法返回值升级为 writer 失败。"""
    offset = 0
    while offset < len(data):
        remaining = data[offset:]
        written = writer(stream, remaining)
        if (
            isinstance(written, bool)
            or not isinstance(written, int)
            or not 0 < written <= len(remaining)
        ):
            raise OSError("interface log writer made no progress")
        offset += written


def _iter_interface_log(
    path: Path,
    *,
    max_frame_bytes: int,
    max_records: int,
    max_total_bytes: int,
) -> Iterator[InterfaceLogRecord]:
    """流式解析已验证限制下的帧，避免一次性积累全部记录。"""
    record_count = 0
    total_bytes = 0
    with path.open("rb") as stream:
        while True:
            prefix = stream.read(_FRAME_PREFIX.size)
            if not prefix:
                break
            if len(prefix) != _FRAME_PREFIX.size:
                raise ValueError("interface log has a short prefix")
            (length,) = _FRAME_PREFIX.unpack(prefix)
            if length > max_frame_bytes:
                raise ValueError("interface log has an oversized length")
            if record_count >= max_records:
                raise ValueError("interface log exceeds max_records")
            frame_bytes = _FRAME_PREFIX.size + length
            if total_bytes + frame_bytes > max_total_bytes:
                raise ValueError("interface log exceeds max_total_bytes")
            payload = stream.read(length)
            if len(payload) != length:
                raise ValueError("interface log has a short payload")
            envelope = internal_pb.InterfaceLogEnvelope()
            try:
                envelope.ParseFromString(payload)
                record = _record_from_envelope(envelope)
            except (DecodeError, ValueError) as exc:
                raise ValueError("interface log contains a bad protobuf") from exc
            record_count += 1
            total_bytes += frame_bytes
            yield record


def iter_interface_log(
    path: str | Path,
    *,
    max_frame_bytes: int = MAX_INTERFACE_LOG_FRAME_BYTES,
    max_records: int = DEFAULT_INTERFACE_LOG_MAX_RECORDS,
    max_total_bytes: int = DEFAULT_INTERFACE_LOG_MAX_TOTAL_BYTES,
) -> Iterator[InterfaceLogRecord]:
    """返回带单帧、记录数及累计字节边界的流式日志 iterator。"""
    normalized_frame_bytes = _require_positive_limit(
        "max_frame_bytes",
        max_frame_bytes,
        maximum=_UINT32_MAX,
    )
    normalized_records = _require_positive_limit("max_records", max_records)
    normalized_total_bytes = _require_positive_limit(
        "max_total_bytes",
        max_total_bytes,
    )
    return _iter_interface_log(
        Path(path),
        max_frame_bytes=normalized_frame_bytes,
        max_records=normalized_records,
        max_total_bytes=normalized_total_bytes,
    )


def read_interface_log(
    path: str | Path,
    *,
    max_frame_bytes: int = MAX_INTERFACE_LOG_FRAME_BYTES,
    max_records: int = DEFAULT_INTERFACE_LOG_MAX_RECORDS,
    max_total_bytes: int = DEFAULT_INTERFACE_LOG_MAX_TOTAL_BYTES,
) -> tuple[InterfaceLogRecord, ...]:
    """在有限总量边界内按文件输入顺序恢复全部消息。"""
    return tuple(
        iter_interface_log(
            path,
            max_frame_bytes=max_frame_bytes,
            max_records=max_records,
            max_total_bytes=max_total_bytes,
        )
    )


class InterfaceEventLogger:
    """用单一后台线程按共享队列入队顺序异步保存消息和事件。"""

    def __init__(
        self,
        log_dir: str | Path,
        *,
        prefix: str = "interfaces",
        queue_size: int = 1024,
        writer: _Writer | None = None,
    ) -> None:
        safe_prefix = _require_safe_prefix(prefix)
        if isinstance(queue_size, bool) or not isinstance(queue_size, int) or queue_size <= 0:
            raise ValueError("queue_size must be a positive integer")
        if writer is not None and not callable(writer):
            raise ValueError("writer must be callable")
        directory = _require_log_directory(log_dir)
        _ensure_log_directory(directory)
        created_paths: list[Path] = []
        binary_file: BinaryIO | None = None
        event_file: TextIO | None = None
        binary_fd: _CapturedStreamFd | None = None
        event_fd: _CapturedStreamFd | None = None
        worker: Thread | None = None

        try:
            self._paths = _new_log_paths(directory, safe_prefix)
            binary_file = self._paths.binary_path.open("xb")
            created_paths.append(self._paths.binary_path)
            binary_fd = _capture_stream_fd(binary_file)
            event_file = self._paths.event_path.open(
                "x",
                encoding="utf-8",
                newline="\n",
            )
            created_paths.append(self._paths.event_path)
            event_fd = _capture_stream_fd(event_file)
            self._binary_file = binary_file
            self._event_file = event_file
            self._binary_fd = binary_fd
            self._event_fd = event_fd
            self._writer = _default_writer if writer is None else writer
            self._condition = Condition()
            self._capacity = Semaphore(queue_size)
            self._queue: Queue[object] = Queue()
            self._state = "open"
            self._accepting = True
            self._worker_error: BaseException | None = None
            self._accepted_messages = 0
            self._accepted_events = 0
            self._dropped_messages = 0
            self._dropped_events = 0
            worker = Thread(
                target=self._run_writer,
                name=f"interface-log-{safe_prefix}",
                daemon=True,
            )
            self._worker = worker
            worker.start()
        except BaseException:
            # start 可能在 target 已运行后才抛错，此时必须先让 worker 退出。
            worker_started = worker is not None and worker.ident is not None
            can_unlink = not worker_started
            if worker_started:
                try:
                    with self._condition:
                        self._accepting = False
                except BaseException:
                    pass
                try:
                    self._queue.put_nowait(_STOP)
                except BaseException:
                    pass
                try:
                    worker.join()
                except BaseException:
                    pass
                try:
                    can_unlink = not worker.is_alive()
                except BaseException:
                    can_unlink = False
            else:
                # 未启动时没有线程拥有文件，由构造线程直接且仅终结一次。
                for stream, fd in (
                    (binary_file, binary_fd),
                    (event_file, event_fd),
                ):
                    if stream is not None:
                        try:
                            _close_stream_with_fd(stream, fd)
                        except BaseException:
                            pass

            if can_unlink:
                for path in created_paths:
                    try:
                        path.unlink()
                    except BaseException:
                        pass
            # 目录可能已被并发构造器共享；这里只清理本实例创建的文件。
            raise

    @property
    def paths(self) -> InterfaceLogPaths:
        """返回构造时确定且不会变化的成对日志路径。"""
        return self._paths

    def _increment_locked(self, kind: str, *, accepted: bool) -> None:
        """在生命周期锁内更新对应消息或事件累计计数。"""
        attribute = f"_{'accepted' if accepted else 'dropped'}_{kind}s"
        setattr(self, attribute, getattr(self, attribute) + 1)

    def _reserve_capacity(
        self,
        kind: str,
        *,
        timeout_sec: float | None = None,
    ) -> bool:
        """在序列化前预留 queued/in-flight 名额；默认路径绝不等待。"""
        acquired = (
            self._capacity.acquire(blocking=False)
            if timeout_sec is None
            else self._capacity.acquire(timeout=timeout_sec)
        )
        if not acquired:
            with self._condition:
                self._increment_locked(kind, accepted=False)
            return False

        with self._condition:
            if not self._accepting:
                self._capacity.release()
                self._increment_locked(kind, accepted=False)
                return False
            return True

    def _commit_reserved(self, item: object, kind: str) -> bool:
        """把已准备项目在线性化边界排队，或在关闭竞态中释放名额。"""
        with self._condition:
            if not self._accepting:
                self._capacity.release()
                self._increment_locked(kind, accepted=False)
                return False
            try:
                self._queue.put_nowait(item)
            except BaseException:
                self._capacity.release()
                raise
            self._increment_locked(kind, accepted=True)
            return True

    def record_message(self, record: InterfaceLogRecord) -> bool:
        """非阻塞提交一条已严格校验的消息记录。"""
        if not isinstance(record, InterfaceLogRecord):
            raise ValueError("record must be an InterfaceLogRecord")
        if _record_envelope_size(record) > MAX_INTERFACE_LOG_FRAME_BYTES:
            raise ValueError("interface log frame exceeds the configured frame limit")
        if not self._reserve_capacity("message"):
            return False
        try:
            item = _MessageItem(_serialize_record(record))
        except BaseException:
            self._capacity.release()
            raise
        return self._commit_reserved(item, "message")

    def record_event(self, event: str, **fields: object) -> bool:
        """非阻塞提交一条 UTF-8、键排序的可读事件。"""
        _validate_event_header(event, fields)
        if not self._reserve_capacity("event"):
            return False
        try:
            item = _EventItem(_serialize_event(event, fields))
        except BaseException:
            self._capacity.release()
            raise
        return self._commit_reserved(item, "event")

    def record_terminal_event(
        self,
        event: str,
        *,
        timeout_sec: float = 1.0,
        **fields: object,
    ) -> bool:
        """只供关闭路径在有限时间内等待容量并提交最终质量事件。"""
        _validate_event_header(event, fields)
        normalized_timeout = _require_nonnegative_timeout(timeout_sec)
        # 终态入口允许等待容量，但非法 JSON 必须在任何等待前立即失败。
        item = _EventItem(_serialize_event(event, fields))
        if not self._reserve_capacity("event", timeout_sec=normalized_timeout):
            return False
        return self._commit_reserved(item, "event")

    def snapshot(self) -> InterfaceLogSnapshot:
        """在一个临界区复制容量计数和关闭终态。"""
        with self._condition:
            return InterfaceLogSnapshot(
                accepted_messages=self._accepted_messages,
                accepted_events=self._accepted_events,
                dropped_messages=self._dropped_messages,
                dropped_events=self._dropped_events,
                closed=self._state in {"closed", "failed"},
                writer_failed=self._worker_error is not None,
            )

    def _record_worker_failure(self, error: BaseException) -> None:
        """保留首个后台错误并原子停止接收后续项目。"""
        with self._condition:
            if self._worker_error is None:
                self._worker_error = error
            self._accepting = False
            self._condition.notify_all()

    def _discard_pending_items(self) -> None:
        """writer 失败后释放所有仍排队项目占用的容量名额。"""
        while True:
            try:
                item = self._queue.get_nowait()
            except Empty:
                return
            if item is not _STOP:
                self._capacity.release()

    def _finalize_files(self) -> BaseException | None:
        """尽力 flush、fsync、close 两个句柄，并返回首个终结错误。"""
        first_error: BaseException | None = None
        for stream, fd in (
            (self._binary_file, self._binary_fd),
            (self._event_file, self._event_fd),
        ):
            try:
                stream.flush()
            except BaseException as exc:  # 后台边界必须保存所有落盘失败
                if first_error is None:
                    first_error = exc
            try:
                # guard 始终引用日志原打开实例，不受高层 fd 复用影响。
                os.fsync(fd.guard_number)
            except BaseException as exc:  # flush 失败后仍独立尝试 fsync
                if first_error is None:
                    first_error = exc
            close_error = _close_stream_with_fd(stream, fd)
            if close_error is not None and first_error is None:
                first_error = close_error
        return first_error

    def _run_writer(self) -> None:
        """按共享队列入队顺序写入，并在每项真正完成后释放容量。"""
        try:
            while True:
                item = self._queue.get()
                if item is _STOP:
                    break
                try:
                    if isinstance(item, _MessageItem):
                        _write_all(self._writer, self._binary_file, item.frame)
                    elif isinstance(item, _EventItem):
                        _write_all(self._writer, self._event_file, item.line)
                    else:  # pragma: no cover - 队列仅由本模块构造
                        raise RuntimeError("unknown interface log queue item")
                except BaseException:
                    self._capacity.release()
                    raise
                self._capacity.release()
        except BaseException as exc:
            self._record_worker_failure(exc)
            self._discard_pending_items()
        finally:
            finalization_error = self._finalize_files()
            if finalization_error is not None:
                self._record_worker_failure(finalization_error)

    def close(self) -> InterfaceLogPaths:
        """停止接收、等待全部已接受项落盘，并发布稳定关闭终态。"""
        if current_thread() is self._worker:
            raise RuntimeError("interface log writer thread cannot close itself")

        owns_close = False
        with self._condition:
            if self._state == "open":
                self._state = "closing"
                self._accepting = False
                self._queue.put_nowait(_STOP)
                owns_close = True
            while not owns_close and self._state == "closing":
                self._condition.wait()
            if not owns_close:
                if self._state == "failed":
                    raise RuntimeError("interface log writer failed") from self._worker_error
                return self._paths

        self._worker.join()
        with self._condition:
            self._state = "failed" if self._worker_error is not None else "closed"
            self._condition.notify_all()
            if self._state == "failed":
                raise RuntimeError("interface log writer failed") from self._worker_error
            return self._paths
