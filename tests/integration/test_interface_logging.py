# 接口日志集成测试：锁定内部 Protobuf 契约与长度前缀消息的有序往返行为。
from dataclasses import FrozenInstanceError
import errno
import gc
from importlib import import_module
import inspect
import json
import math
import os
from pathlib import Path
import struct
from threading import Barrier, Event, Lock, Thread
import time

import pytest


INTERNAL_PACKAGE = "slope_sim.internal.v1"
UINT64_MAX = (1 << 64) - 1
REQUIRED_EVENTS = (
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
)
INTERNAL_FIELDS = (
    ("sequence", 1, "TYPE_UINT64", "LABEL_OPTIONAL"),
    ("topic", 2, "TYPE_STRING", "LABEL_OPTIONAL"),
    ("direction", 3, "TYPE_STRING", "LABEL_OPTIONAL"),
    ("sim_time_ns", 4, "TYPE_UINT64", "LABEL_OPTIONAL"),
    ("wall_time_ns", 5, "TYPE_UINT64", "LABEL_OPTIONAL"),
    ("type_name", 6, "TYPE_STRING", "LABEL_OPTIONAL"),
    ("payload", 7, "TYPE_BYTES", "LABEL_OPTIONAL"),
)


interface_logging = import_module("slope_sim.interfaces.logging")
InterfaceEventLogger = interface_logging.InterfaceEventLogger
InterfaceLogPaths = interface_logging.InterfaceLogPaths
InterfaceLogRecord = interface_logging.InterfaceLogRecord
read_interface_log = interface_logging.read_interface_log


def sample_record(sequence=1, **changes):
    values = {
        "sequence": sequence,
        "topic": "/sim/wheel/state",
        "direction": "publish",
        "sim_time_ns": 10,
        "wall_time_ns": 20,
        "type_name": "slope_sim.interfaces.v1.WheelState",
        "payload": b"payload",
    }
    values.update(changes)
    return InterfaceLogRecord(**values)


class BlockingWriter:
    """阻塞首个后台写入，供容量和关闭屏障测试使用。"""

    def __init__(self, *, fail=False):
        self.started = Event()
        self.release = Event()
        self.fail = fail
        self.calls = []
        self._lock = Lock()

    def __call__(self, stream, data):
        with self._lock:
            self.calls.append("message" if isinstance(data, bytes) else "event")
        self.started.set()
        if not self.release.wait(timeout=5.0):
            raise TimeoutError("test writer was not released")
        if self.fail:
            raise OSError("injected writer failure")
        return stream.write(data)


class CapturingWriter:
    """记录单后台线程看到的混合项顺序，再执行真实文件写入。"""

    def __init__(self):
        self.order = []

    def __call__(self, stream, data):
        if isinstance(data, bytes):
            self.order.append("message")
        else:
            self.order.append(json.loads(data)["event"])
        return stream.write(data)


class ChunkedWriter:
    """每次只写固定长度，可选择在若干次调用后停止前进。"""

    def __init__(self, chunk_size, *, zero_after=None):
        self.chunk_size = chunk_size
        self.zero_after = zero_after
        self.calls = 0

    def __call__(self, stream, data):
        self.calls += 1
        if self.zero_after is not None and self.calls > self.zero_after:
            return 0
        return stream.write(data[: self.chunk_size])


class CapacitySpy:
    """记录容量 acquire/release 顺序，同时委托给真实 semaphore。"""

    def __init__(self, delegate, trace):
        self.delegate = delegate
        self.trace = trace

    def acquire(self, *args, **kwargs):
        self.trace.append("reserve")
        return self.delegate.acquire(*args, **kwargs)

    def release(self):
        self.trace.append("release")
        return self.delegate.release()


class CloseRaisingStream:
    """close 抛错但故意不关闭底层真实文件描述符。"""

    def __init__(self, stream, label):
        self.stream = stream
        self.label = label
        self.fd = stream.fileno()
        self.close_calls = 0

    @property
    def closed(self):
        return self.stream.closed

    def close(self):
        self.close_calls += 1
        raise OSError(f"injected {self.label} close failure")

    def __getattr__(self, name):
        return getattr(self.stream, name)


class FinalizationTrackingStream:
    """记录 flush/close，并可注入 flush 失败。"""

    def __init__(self, stream, label, *, fail_flush=False):
        self.stream = stream
        self.label = label
        self.fd = stream.fileno()
        self.fail_flush = fail_flush
        self.flush_calls = 0
        self.close_calls = 0

    @property
    def closed(self):
        return self.stream.closed

    def flush(self):
        self.flush_calls += 1
        if self.fail_flush:
            raise OSError(f"injected {self.label} flush failure")
        return self.stream.flush()

    def close(self):
        self.close_calls += 1
        return self.stream.close()

    def __getattr__(self, name):
        return getattr(self.stream, name)


class DelayedFinalizerStream:
    """第一次析构不动作，第二次会关闭已捕获的描述符。"""

    def __init__(self, stream):
        self.stream = stream
        self.fd = stream.fileno()
        self.finalizer_calls = 0

    @property
    def closed(self):
        return False

    def fileno(self):
        return self.fd

    def close(self):
        raise OSError("injected close failure")

    def __del__(self):
        self.finalizer_calls += 1
        if self.finalizer_calls >= 2:
            try:
                _REAL_OS_CLOSE(self.fd)
            except OSError:
                pass


class CloseReusesFdStream:
    """close 先关闭旧 fd 并复用同号，但故意继续报告未关闭。"""

    def __init__(self, stream, replacement_path):
        self.stream = stream
        self.fd = stream.fileno()
        self.replacement_path = replacement_path
        self.replacement_fd = None

    @property
    def closed(self):
        return False

    def fileno(self):
        return self.fd

    def close(self):
        _REAL_OS_CLOSE(self.fd)
        self.replacement_fd = os.open(
            self.replacement_path,
            os.O_CREAT | os.O_RDWR,
            0o600,
        )
        assert self.replacement_fd == self.fd
        raise OSError("injected close-after-reuse failure")

    def __del__(self):
        pass


_REAL_OS_CLOSE = os.close


def assert_fd_is_closed(fd):
    with pytest.raises(OSError) as error:
        os.fstat(fd)
    assert error.value.errno == errno.EBADF


def force_close_test_streams(streams):
    for stream in streams.values():
        try:
            os.close(stream.fd)
        except OSError as error:
            if error.errno != errno.EBADF:
                raise


def test_internal_proto_descriptor_is_exact():
    descriptor_pb2 = import_module("google.protobuf.descriptor_pb2")
    generated = import_module(
        "slope_sim.interfaces.generated.slope_sim_internal_pb2"
    )
    file_descriptor = descriptor_pb2.FileDescriptorProto()
    file_descriptor.ParseFromString(generated.DESCRIPTOR.serialized_pb)

    assert file_descriptor.name == "slope_sim_internal.proto"
    assert file_descriptor.syntax == "proto3"
    assert file_descriptor.package == INTERNAL_PACKAGE
    assert tuple(message.name for message in file_descriptor.message_type) == (
        "InterfaceLogEnvelope",
    )
    assert tuple(
        (
            field.name,
            field.number,
            descriptor_pb2.FieldDescriptorProto.Type.Name(field.type),
            descriptor_pb2.FieldDescriptorProto.Label.Name(field.label),
        )
        for field in file_descriptor.message_type[0].field
    ) == INTERNAL_FIELDS


def test_binary_log_round_trips_records_in_input_order(tmp_path):
    records = (
        InterfaceLogRecord(
            2,
            "/sim/wheel/state",
            "publish",
            30,
            40,
            "WheelState",
            b"bb",
        ),
        InterfaceLogRecord(
            1,
            "/sim/wheel/command",
            "receive",
            10,
            20,
            "WheelCommand",
            b"a",
        ),
    )
    logger = InterfaceEventLogger(tmp_path, prefix="gate", queue_size=8)

    assert all(logger.record_message(record) for record in records)
    paths = logger.close()

    assert read_interface_log(paths.binary_path) == records


def test_log_record_is_immutable_and_copies_bytes_like_payload():
    source = bytearray(b"payload")
    record = sample_record(payload=source)
    source[:] = b"changed"

    assert record.payload == b"payload"
    assert isinstance(record.payload, bytes)
    with pytest.raises(FrozenInstanceError):
        record.sequence = 2


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("sequence", True),
        ("sequence", -1),
        ("sequence", UINT64_MAX + 1),
        ("topic", ""),
        ("topic", 1),
        ("direction", "subscribe"),
        ("sim_time_ns", -1),
        ("wall_time_ns", 1.0),
        ("type_name", ""),
        ("payload", "payload"),
    ),
)
def test_log_record_rejects_invalid_fields(field, value):
    with pytest.raises(ValueError, match=field):
        sample_record(**{field: value})


def test_log_paths_are_distinct_immutable_path_objects(tmp_path):
    paths = InterfaceLogPaths(tmp_path / "messages.bin", tmp_path / "events.jsonl")

    with pytest.raises(FrozenInstanceError):
        paths.binary_path = tmp_path / "other.bin"
    with pytest.raises(ValueError, match="binary_path"):
        InterfaceLogPaths("messages.bin", tmp_path / "events.jsonl")
    with pytest.raises(ValueError, match="distinct"):
        InterfaceLogPaths(paths.binary_path, paths.binary_path)


def test_log_snapshot_is_immutable_and_strictly_validated():
    InterfaceLogSnapshot = interface_logging.InterfaceLogSnapshot
    snapshot = InterfaceLogSnapshot(1, 2, 3, 4, False, False)

    assert snapshot.pending_count == 0
    with pytest.raises(FrozenInstanceError):
        snapshot.dropped_messages = 5
    with pytest.raises(ValueError, match="accepted_messages"):
        InterfaceLogSnapshot(True, 2, 3, 4, False, False)
    with pytest.raises(ValueError, match="closed"):
        InterfaceLogSnapshot(1, 2, 3, 4, 0, False)
    with pytest.raises(ValueError, match="pending_count"):
        InterfaceLogSnapshot(1, 2, 3, 4, False, False, pending_count=True)


@pytest.mark.parametrize(
    ("corruption", "contents", "max_frame_bytes", "reason"),
    (
        ("short_prefix", b"\x01\x00", 64, "short prefix"),
        (
            "oversized_length",
            struct.pack("<I", 65),
            64,
            "oversized length",
        ),
        (
            "short_payload",
            struct.pack("<I", 2) + b"\x08",
            64,
            "short payload",
        ),
        (
            "bad_protobuf",
            struct.pack("<I", 1) + b"\x80",
            64,
            "bad protobuf",
        ),
    ),
)
def test_reader_rejects_each_corrupt_binary_frame(
    tmp_path,
    corruption,
    contents,
    max_frame_bytes,
    reason,
):
    path = tmp_path / f"{corruption}.bin"
    path.write_bytes(contents)

    with pytest.raises(ValueError, match=rf"interface log.*{reason}"):
        read_interface_log(path, max_frame_bytes=max_frame_bytes)


@pytest.mark.parametrize("maximum", (True, 0, -1, 1.5, (1 << 32)))
def test_reader_rejects_invalid_maximum_frame_size(tmp_path, maximum):
    path = tmp_path / "empty.bin"
    path.write_bytes(b"")

    with pytest.raises(ValueError, match="interface log.*max_frame_bytes"):
        read_interface_log(path, max_frame_bytes=maximum)


def test_streaming_reader_yields_valid_records_before_later_corruption(tmp_path):
    logger = InterfaceEventLogger(tmp_path, prefix="streaming-reader", queue_size=4)
    first = sample_record(1)
    second = sample_record(2)
    assert logger.record_message(first)
    assert logger.record_message(second)
    path = logger.close().binary_path
    with path.open("ab") as stream:
        stream.write(struct.pack("<I", 1) + b"\x80")

    iterator = interface_logging.iter_interface_log(path)

    assert next(iterator) == first
    assert next(iterator) == second
    with pytest.raises(ValueError, match="interface log.*bad protobuf"):
        next(iterator)


def test_reader_enforces_configurable_record_and_total_byte_limits(tmp_path):
    logger = InterfaceEventLogger(tmp_path, prefix="reader-totals", queue_size=4)
    records = tuple(sample_record(sequence) for sequence in range(3))
    assert all(logger.record_message(record) for record in records)
    path = logger.close().binary_path
    total_bytes = path.stat().st_size

    with pytest.raises(ValueError, match="interface log.*max_records"):
        read_interface_log(
            path,
            max_records=2,
            max_total_bytes=total_bytes,
        )
    with pytest.raises(ValueError, match="interface log.*max_total_bytes"):
        read_interface_log(
            path,
            max_records=3,
            max_total_bytes=total_bytes - 1,
        )

    assert read_interface_log(
        path,
        max_records=3,
        max_total_bytes=total_bytes,
    ) == records


@pytest.mark.parametrize(
    ("name", "value"),
    (
        ("max_records", True),
        ("max_records", 0),
        ("max_records", -1),
        ("max_records", 1.5),
        ("max_total_bytes", True),
        ("max_total_bytes", 0),
        ("max_total_bytes", -1),
        ("max_total_bytes", 1.5),
    ),
)
def test_reader_rejects_invalid_total_limit_configuration(tmp_path, name, value):
    path = tmp_path / "empty.bin"
    path.write_bytes(b"")
    kwargs = {name: value}

    with pytest.raises(ValueError, match=rf"interface log.*{name}"):
        read_interface_log(path, **kwargs)


def test_reader_defaults_have_finite_record_and_total_byte_limits():
    parameters = inspect.signature(read_interface_log).parameters

    assert isinstance(parameters["max_records"].default, int)
    assert parameters["max_records"].default > 0
    assert isinstance(parameters["max_total_bytes"].default, int)
    assert parameters["max_total_bytes"].default > 0


def test_public_frame_limit_is_the_reader_default():
    maximum = interface_logging.MAX_INTERFACE_LOG_FRAME_BYTES

    assert maximum == 64 * 1024 * 1024
    assert inspect.signature(read_interface_log).parameters[
        "max_frame_bytes"
    ].default == maximum


def test_logger_rejects_obviously_oversized_payload_before_serialization(
    tmp_path,
    monkeypatch,
):
    monkeypatch.setattr(
        interface_logging,
        "MAX_INTERFACE_LOG_FRAME_BYTES",
        256,
        raising=False,
    )
    logger = InterfaceEventLogger(tmp_path, prefix="oversized-payload", queue_size=1)

    def unexpected_serialization(_record):
        raise AssertionError("oversized payload reached protobuf serialization")

    monkeypatch.setattr(interface_logging, "_serialize_record", unexpected_serialization)
    try:
        with pytest.raises(ValueError, match="interface log.*frame.*limit"):
            logger.record_message(sample_record(payload=b"x" * 257))
        assert logger.snapshot().accepted_messages == 0
        assert logger.snapshot().dropped_messages == 0
    finally:
        paths = logger.close()

    assert paths.binary_path.read_bytes() == b""


def test_logger_rejects_payload_equal_to_limit_before_payload_copy(
    tmp_path,
    monkeypatch,
):
    monkeypatch.setattr(
        interface_logging,
        "MAX_INTERFACE_LOG_FRAME_BYTES",
        256,
    )
    logger = InterfaceEventLogger(tmp_path, prefix="equal-limit", queue_size=1)

    def unexpected_serialization(_record):
        raise AssertionError("equal-limit payload reached protobuf serialization")

    monkeypatch.setattr(interface_logging, "_serialize_record", unexpected_serialization)
    try:
        with pytest.raises(ValueError, match="interface log.*frame.*limit"):
            logger.record_message(sample_record(payload=b"x" * 256))
        assert logger.snapshot().accepted_messages == 0
    finally:
        logger.close()


def test_logger_rejects_envelope_whose_metadata_pushes_it_over_frame_limit(
    tmp_path,
    monkeypatch,
):
    monkeypatch.setattr(
        interface_logging,
        "MAX_INTERFACE_LOG_FRAME_BYTES",
        128,
        raising=False,
    )
    logger = InterfaceEventLogger(tmp_path, prefix="oversized-envelope", queue_size=1)
    record = sample_record(topic="/" + "t" * 120, payload=b"x")
    try:
        with pytest.raises(ValueError, match="interface log.*frame.*limit"):
            logger.record_message(record)
        assert logger.snapshot().accepted_messages == 0
    finally:
        paths = logger.close()

    assert paths.binary_path.read_bytes() == b""


@pytest.mark.parametrize("event", REQUIRED_EVENTS)
def test_all_required_event_categories_are_json_serializable(tmp_path, event):
    logger = InterfaceEventLogger(tmp_path, prefix=event, queue_size=2)

    assert logger.record_event(
        event,
        wall_time_ns=1,
        sim_time_ns=2,
        robot_model="df_back",
        terrain_model="flat",
    )
    payload = json.loads(logger.close().event_path.read_text(encoding="utf-8"))

    assert payload == {
        "event": event,
        "robot_model": "df_back",
        "sim_time_ns": 2,
        "terrain_model": "flat",
        "wall_time_ns": 1,
    }


def test_event_jsonl_uses_utf8_sorted_keys_and_copies_mutable_fields(tmp_path):
    logger = InterfaceEventLogger(tmp_path, prefix="readable", queue_size=4)
    details = {"轮位": ["左前", "右前"]}
    fields = {
        "wall_time_ns": 20,
        "sim_time_ns": 10,
        "robot_model": "df_back",
        "terrain_model": "高尔夫场",
        "topic": "/sim/wheel/command",
        "reason": "驱动轮[0]越界",
        "details": details,
    }
    expected_fields = {
        **fields,
        "details": {"轮位": ["左前", "右前"]},
    }

    assert logger.record_event("invalid_command", **fields)
    details["轮位"].append("已修改")
    paths = logger.close()

    expected = json.dumps(
        {"event": "invalid_command", **expected_fields},
        ensure_ascii=False,
        sort_keys=True,
    ) + "\n"
    assert paths.event_path.read_text(encoding="utf-8") == expected


@pytest.mark.parametrize("event", ("", 1))
def test_event_logger_rejects_invalid_event_names(tmp_path, event):
    logger = InterfaceEventLogger(tmp_path, prefix="invalid-event")
    try:
        with pytest.raises(ValueError, match="event"):
            logger.record_event(event)
    finally:
        logger.close()


def test_event_logger_accepts_additional_nonempty_event_categories(tmp_path):
    logger = InterfaceEventLogger(tmp_path, prefix="custom-event")
    try:
        assert logger.record_event("custom_diagnostic", reason="future extension")
    finally:
        paths = logger.close()
    event = json.loads(paths.event_path.read_text(encoding="utf-8"))

    assert event["event"] == "custom_diagnostic"


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("wall_time_ns", True),
        ("sim_time_ns", -1),
        ("robot_model", ""),
        ("terrain_model", 1),
        ("topic", ""),
        ("reason", None),
        ("details", object()),
        ("measurement", math.nan),
        ("measurement", math.inf),
        ("measurement", -math.inf),
    ),
)
def test_event_logger_rejects_invalid_or_non_json_fields(tmp_path, field, value):
    logger = InterfaceEventLogger(tmp_path, prefix="invalid-field")
    try:
        with pytest.raises(ValueError, match="interface log event"):
            logger.record_event("sensor_failed", **{field: value})
    finally:
        logger.close()


@pytest.mark.parametrize(
    "details",
    (
        {1: "integer key"},
        {"nested": [{True: "boolean key"}]},
        {"nested": {1.5: "float key"}},
    ),
)
def test_event_logger_recursively_rejects_non_string_dict_keys_and_releases_capacity(
    tmp_path,
    details,
):
    logger = InterfaceEventLogger(tmp_path, prefix="invalid-json-key", queue_size=1)
    try:
        with pytest.raises(ValueError, match="interface log event.*dict key"):
            logger.record_event("sensor_failed", details=details)
        assert logger.snapshot().accepted_events == 0
        assert logger.snapshot().dropped_events == 0
        assert logger.record_event(
            "sensor_failed",
            details={"nested": [{"valid": True}]},
        )
    finally:
        paths = logger.close()

    rows = paths.event_path.read_text(encoding="utf-8").splitlines()
    assert len(rows) == 1
    assert json.loads(rows[0])["details"] == {"nested": [{"valid": True}]}


def test_full_queue_never_blocks_and_counts_each_dropped_message(tmp_path):
    writer = BlockingWriter()
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="overload",
        queue_size=2,
        writer=writer,
    )
    accepted: list[bool] = []
    submission_errors: list[BaseException] = []
    submission_completed = Event()

    def submit_batch() -> None:
        try:
            accepted.extend(
                logger.record_message(sample_record(sequence))
                for sequence in range(10)
            )
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            submission_errors.append(exc)
        finally:
            submission_completed.set()

    submitter = Thread(target=submit_batch, daemon=True)
    try:
        submitter.start()
        assert writer.started.wait(timeout=1.0)
        assert submission_completed.wait(timeout=1.0)
        submitter.join(timeout=1.0)

        assert accepted == [True, True] + [False] * 8
        assert submission_errors == []
        assert not submitter.is_alive()
        assert not writer.release.is_set()
        assert logger.snapshot().pending_count == 2
        assert logger.snapshot() == interface_logging.InterfaceLogSnapshot(
            accepted_messages=2,
            accepted_events=0,
            dropped_messages=8,
            dropped_events=0,
            closed=False,
            writer_failed=False,
            pending_count=2,
        )
    finally:
        writer.release.set()
        submitter.join(timeout=1.0)

    paths = logger.close()
    assert logger.snapshot().pending_count == 0
    assert [record.sequence for record in read_interface_log(paths.binary_path)] == [0, 1]
    assert writer.calls == ["message", "message"]


def test_paths_property_is_read_only_and_stable_across_close(tmp_path):
    logger = InterfaceEventLogger(tmp_path, prefix="readonly-paths")

    paths = logger.paths
    with pytest.raises(AttributeError):
        logger.paths = InterfaceLogPaths(
            tmp_path / "replacement.bin",
            tmp_path / "replacement.jsonl",
        )

    assert logger.close() is paths


def test_terminal_event_timeout_is_bounded_and_does_not_leak_capacity(tmp_path):
    writer = BlockingWriter()
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="terminal-capacity",
        queue_size=1,
        writer=writer,
    )
    try:
        assert logger.record_message(sample_record(1))
        assert writer.started.wait(timeout=1.0)

        started = time.perf_counter()
        assert not logger.record_terminal_event(
            "queue_dropped",
            timeout_sec=0.02,
            source="interface_logger",
            count=1,
        )
        elapsed = time.perf_counter() - started

        assert elapsed < 0.5
        assert logger.snapshot().dropped_events == 1

        # 超时不能消费 semaphore 名额；容量恢复后同一路径仍应成功。
        writer.release.set()
        assert logger.record_terminal_event(
            "queue_dropped",
            timeout_sec=1.0,
            source="interface_logger",
            count=1,
        )
    finally:
        writer.release.set()
        paths = logger.close()

    events = [
        json.loads(line)
        for line in paths.event_path.read_text(encoding="utf-8").splitlines()
    ]
    assert events == [
        {"count": 1, "event": "queue_dropped", "source": "interface_logger"}
    ]


@pytest.mark.parametrize(
    ("details", "case_name"),
    (
        ({1: "non-string key"}, "non-string-key"),
        (float("nan"), "nan"),
        (object(), "non-serializable"),
    ),
)
def test_terminal_event_fully_validates_before_waiting_for_capacity(
    tmp_path,
    details,
    case_name,
):
    writer = BlockingWriter()
    logger = InterfaceEventLogger(
        tmp_path,
        prefix=f"terminal-validation-{case_name}",
        queue_size=1,
        writer=writer,
    )
    try:
        assert logger.record_message(sample_record(1))
        assert writer.started.wait(timeout=1.0)

        started = time.perf_counter()
        with pytest.raises(ValueError, match="JSON|dict keys"):
            logger.record_terminal_event(
                "queue_dropped",
                timeout_sec=0.5,
                source="interface_logger",
                count=1,
                details=details,
            )
        elapsed = time.perf_counter() - started

        assert elapsed < 0.2
        assert logger.snapshot().dropped_events == 0

        # 校验失败不能占用容量；原有写入完成后终态入口仍可取得唯一名额。
        writer.release.set()
        assert logger.record_terminal_event(
            "queue_dropped",
            timeout_sec=1.0,
            source="interface_logger",
            count=1,
        )
    finally:
        writer.release.set()
        paths = logger.close()

    events = [
        json.loads(line)
        for line in paths.event_path.read_text(encoding="utf-8").splitlines()
    ]
    assert events == [
        {"count": 1, "event": "queue_dropped", "source": "interface_logger"}
    ]


def test_terminal_event_after_writer_failure_is_rejected_and_counted(tmp_path):
    writer = BlockingWriter(fail=True)
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="terminal-writer-failure",
        queue_size=1,
        writer=writer,
    )
    assert logger.record_message(sample_record(1))
    assert writer.started.wait(timeout=1.0)
    writer.release.set()

    deadline = time.monotonic() + 1.0
    while not logger.snapshot().writer_failed and time.monotonic() < deadline:
        time.sleep(0.005)
    assert logger.snapshot().writer_failed

    assert not logger.record_terminal_event(
        "queue_dropped",
        timeout_sec=0.1,
        source="interface_logger",
        count=1,
    )
    assert logger.snapshot().dropped_events == 1
    with pytest.raises(RuntimeError, match="^interface log writer failed$"):
        logger.close()


def test_full_capacity_skips_large_message_and_event_serialization(
    tmp_path,
    monkeypatch,
):
    writer = BlockingWriter()
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="large-overload",
        queue_size=1,
        writer=writer,
    )
    assert logger.record_message(sample_record(1))
    assert writer.started.wait(timeout=1.0)
    large_record = sample_record(2, payload=b"x" * (8 * 1024 * 1024))
    large_event_value = "界" * (8 * 1024 * 1024)

    def unexpected_record_serialization(_record):
        raise AssertionError("full queue serialized a large record")

    def unexpected_event_serialization(_event, _fields):
        raise AssertionError("full queue serialized a large event")

    monkeypatch.setattr(
        interface_logging,
        "_serialize_record",
        unexpected_record_serialization,
    )
    monkeypatch.setattr(
        interface_logging,
        "_serialize_event",
        unexpected_event_serialization,
    )
    try:
        started = time.perf_counter()
        assert not logger.record_message(large_record)
        assert not logger.record_event("sensor_failed", details=large_event_value)
        elapsed = time.perf_counter() - started

        assert elapsed < 0.050
        snapshot = logger.snapshot()
        assert snapshot.dropped_messages == 1
        assert snapshot.dropped_events == 1
    finally:
        writer.release.set()
        logger.close()


@pytest.mark.parametrize("item_kind", ("message", "event"))
def test_serialization_failure_releases_reserved_capacity(
    tmp_path,
    monkeypatch,
    item_kind,
):
    logger = InterfaceEventLogger(
        tmp_path,
        prefix=f"serialization-failure-{item_kind}",
        queue_size=1,
    )
    trace = []
    logger._capacity = CapacitySpy(logger._capacity, trace)

    if item_kind == "message":
        original_serializer = interface_logging._serialize_record

        def fail_serialization(_record):
            trace.append("serialize")
            raise ValueError("injected record serialization failure")

        monkeypatch.setattr(interface_logging, "_serialize_record", fail_serialization)
        call = lambda: logger.record_message(sample_record(1))
    else:
        original_serializer = interface_logging._serialize_event

        def fail_serialization(_event, _fields):
            trace.append("serialize")
            raise ValueError("injected event serialization failure")

        monkeypatch.setattr(interface_logging, "_serialize_event", fail_serialization)
        call = lambda: logger.record_event("sensor_failed", reason="injected")

    try:
        with pytest.raises(ValueError, match="injected .* serialization failure"):
            call()
        assert trace == ["reserve", "serialize", "release"]
        snapshot = logger.snapshot()
        assert snapshot.accepted_messages == 0
        assert snapshot.accepted_events == 0
        assert snapshot.dropped_messages == 0
        assert snapshot.dropped_events == 0

        if item_kind == "message":
            monkeypatch.setattr(
                interface_logging,
                "_serialize_record",
                original_serializer,
            )
            assert logger.record_message(sample_record(2))
        else:
            monkeypatch.setattr(
                interface_logging,
                "_serialize_event",
                original_serializer,
            )
            assert logger.record_event("sensor_failed", reason="recovered")
    finally:
        logger.close()


def test_messages_and_events_share_capacity_without_overwriting_accepted_items(tmp_path):
    writer = BlockingWriter()
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="mixed-capacity",
        queue_size=2,
        writer=writer,
    )
    try:
        assert logger.record_message(sample_record(1))
        assert logger.record_event("ecal_initialized", wall_time_ns=2)
        assert not logger.record_message(sample_record(3))
        assert logger.snapshot() == interface_logging.InterfaceLogSnapshot(
            accepted_messages=1,
            accepted_events=1,
            dropped_messages=1,
            dropped_events=0,
            closed=False,
            writer_failed=False,
            pending_count=2,
        )
    finally:
        writer.release.set()

    paths = logger.close()
    assert [record.sequence for record in read_interface_log(paths.binary_path)] == [1]
    assert json.loads(paths.event_path.read_text(encoding="utf-8"))["event"] == "ecal_initialized"
    assert writer.calls == ["message", "event"]


def test_single_worker_executes_mixed_items_in_enqueue_order(tmp_path):
    writer = CapturingWriter()
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="mixed-order",
        queue_size=8,
        writer=writer,
    )

    assert logger.record_message(sample_record(1))
    assert logger.record_event("invalid_command", reason="limit")
    assert logger.record_message(sample_record(2))
    assert logger.record_event("publish_failed", topic="/sim/wheel/state")
    paths = logger.close()

    # 这里只锁定单 worker 的执行顺序；两个独立文件不承诺恢复跨文件全局顺序。
    assert writer.order == [
        "message",
        "invalid_command",
        "message",
        "publish_failed",
    ]
    assert [record.sequence for record in read_interface_log(paths.binary_path)] == [1, 2]
    assert [
        json.loads(line)["event"]
        for line in paths.event_path.read_text(encoding="utf-8").splitlines()
    ] == ["invalid_command", "publish_failed"]


def test_close_is_idempotent_and_flushes_every_accepted_item(tmp_path):
    logger = InterfaceEventLogger(tmp_path, prefix="close", queue_size=64)
    for sequence in range(20):
        assert logger.record_message(sample_record(sequence))
        assert logger.record_event("sensor_failed", sim_time_ns=sequence)

    first = logger.close()
    second = logger.close()

    assert second is first
    assert [record.sequence for record in read_interface_log(first.binary_path)] == list(range(20))
    assert len(first.event_path.read_text(encoding="utf-8").splitlines()) == 20
    assert logger.snapshot().closed
    assert not logger.record_message(sample_record(21))
    assert not logger.record_event("ecal_closed", wall_time_ns=22)


def test_concurrent_close_calls_return_one_stable_result(tmp_path):
    writer = BlockingWriter()
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="concurrent-close",
        queue_size=2,
        writer=writer,
    )
    assert logger.record_message(sample_record(1))
    assert writer.started.wait(timeout=1.0)
    assert logger.record_message(sample_record(2))

    results = []
    errors = []

    def close_logger():
        try:
            results.append(logger.close())
        except BaseException as exc:  # pragma: no cover - 线程异常带回主测试线程
            errors.append(exc)

    threads = [Thread(target=close_logger) for _ in range(4)]
    for thread in threads:
        thread.start()
    writer.release.set()
    for thread in threads:
        thread.join(timeout=2.0)

    assert all(not thread.is_alive() for thread in threads)
    assert errors == []
    assert len(results) == 4
    assert all(result is results[0] for result in results)
    assert [record.sequence for record in read_interface_log(results[0].binary_path)] == [1, 2]


def test_writer_failure_has_stable_close_error_and_never_deadlocks(tmp_path):
    writer = BlockingWriter(fail=True)
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="writer-failure",
        queue_size=2,
        writer=writer,
    )
    assert logger.record_message(sample_record(1))
    assert writer.started.wait(timeout=1.0)
    assert logger.record_event("publish_failed", topic="/sim/wheel/state")
    writer.release.set()

    errors = []

    def close_logger():
        try:
            logger.close()
        except BaseException as exc:  # pragma: no cover - 线程异常带回主测试线程
            errors.append(exc)

    thread = Thread(target=close_logger)
    thread.start()
    thread.join(timeout=2.0)

    assert not thread.is_alive()
    assert len(errors) == 1
    assert isinstance(errors[0], RuntimeError)
    assert str(errors[0]) == "interface log writer failed"
    assert logger.snapshot().closed
    assert logger.snapshot().writer_failed
    with pytest.raises(RuntimeError, match="^interface log writer failed$"):
        logger.close()


@pytest.mark.parametrize("item_kind", ("message", "event"))
def test_worker_retries_consecutive_short_writes_until_item_is_complete(
    tmp_path,
    item_kind,
):
    writer = ChunkedWriter(3)
    logger = InterfaceEventLogger(
        tmp_path,
        prefix=f"short-write-{item_kind}",
        writer=writer,
    )
    if item_kind == "message":
        assert logger.record_message(sample_record(payload=b"abcdefghij"))
    else:
        assert logger.record_event("sensor_failed", reason="abcdefghij")

    paths = logger.close()

    assert writer.calls > 2
    if item_kind == "message":
        assert read_interface_log(paths.binary_path) == (
            sample_record(payload=b"abcdefghij"),
        )
    else:
        assert json.loads(paths.event_path.read_text(encoding="utf-8")) == {
            "event": "sensor_failed",
            "reason": "abcdefghij",
        }


@pytest.mark.parametrize("item_kind", ("message", "event"))
def test_zero_progress_after_short_write_has_stable_writer_failure(
    tmp_path,
    item_kind,
):
    writer = ChunkedWriter(3, zero_after=1)
    logger = InterfaceEventLogger(
        tmp_path,
        prefix=f"zero-write-{item_kind}",
        writer=writer,
    )
    if item_kind == "message":
        assert logger.record_message(sample_record(payload=b"abcdefghij"))
    else:
        assert logger.record_event("sensor_failed", reason="abcdefghij")

    with pytest.raises(RuntimeError, match="^interface log writer failed$"):
        logger.close()
    with pytest.raises(RuntimeError, match="^interface log writer failed$"):
        logger.close()


@pytest.mark.parametrize(
    "prefix",
    (
        "",
        ".",
        "..",
        "../escape",
        "sub/directory",
        "sub\\directory",
        " leading",
        "trailing ",
        "bad\x00name",
        "bad:name",
        1,
    ),
)
def test_logger_rejects_unsafe_prefixes_without_leaking_files(tmp_path, prefix):
    logger = None
    try:
        with pytest.raises(ValueError, match="prefix"):
            logger = InterfaceEventLogger(tmp_path, prefix=prefix)
    finally:
        if logger is not None:
            paths = logger.close()
            paths.binary_path.unlink(missing_ok=True)
            paths.event_path.unlink(missing_ok=True)


@pytest.mark.parametrize("log_dir", ("", object()))
def test_logger_rejects_invalid_log_directories(tmp_path, monkeypatch, log_dir):
    monkeypatch.chdir(tmp_path)
    logger = None
    try:
        with pytest.raises(ValueError, match="interface log directory"):
            logger = InterfaceEventLogger(log_dir)
    finally:
        if logger is not None:
            paths = logger.close()
            paths.binary_path.unlink(missing_ok=True)
            paths.event_path.unlink(missing_ok=True)


def test_same_prefix_creates_distinct_files_without_overwriting_old_log(tmp_path):
    first_logger = InterfaceEventLogger(tmp_path, prefix="repeat")
    assert first_logger.record_message(sample_record(1))
    first = first_logger.close()

    second_logger = InterfaceEventLogger(tmp_path, prefix="repeat")
    assert second_logger.record_message(sample_record(2))
    second = second_logger.close()

    assert second != first
    assert read_interface_log(first.binary_path) == (sample_record(1),)
    assert read_interface_log(second.binary_path) == (sample_record(2),)


@pytest.mark.parametrize("preexisting", (False, True))
def test_partial_file_initialization_cleans_created_artifacts(
    tmp_path,
    monkeypatch,
    preexisting,
):
    directory = tmp_path / "created-parent" / "logs"
    if preexisting:
        directory.mkdir(parents=True)
    real_open = Path.open

    def fail_event_open(path, *args, **kwargs):
        if path.name.endswith(".events.jsonl"):
            raise OSError("injected event open failure")
        return real_open(path, *args, **kwargs)

    monkeypatch.setattr(Path, "open", fail_event_open)

    with pytest.raises(OSError, match="event open failure"):
        InterfaceEventLogger(directory, prefix="partial-open")

    if directory.exists():
        assert list(directory.iterdir()) == []
    assert directory.is_dir()


def test_open_failure_cannot_remove_directory_shared_by_concurrent_constructor(
    tmp_path,
    monkeypatch,
):
    directory = tmp_path / "shared-open-race"
    mkdir_barrier = Barrier(2)
    failure_done = Event()
    real_mkdir = Path.mkdir
    real_open = Path.open
    successes = []
    errors = []

    def synchronized_mkdir(path, *args, **kwargs):
        if path == directory:
            mkdir_barrier.wait(timeout=2.0)
        return real_mkdir(path, *args, **kwargs)

    def controlled_open(path, *args, **kwargs):
        if "race-open-fail" in path.name and path.name.endswith(".interfaces.bin"):
            raise OSError("injected race open failure")
        if "race-open-success" in path.name and path.name.endswith(".interfaces.bin"):
            assert failure_done.wait(timeout=2.0)
        return real_open(path, *args, **kwargs)

    def construct_failure():
        try:
            InterfaceEventLogger(directory, prefix="race-open-fail")
        except BaseException as exc:  # pragma: no cover - 线程异常带回主测试线程
            errors.append(exc)
        finally:
            failure_done.set()

    def construct_success():
        try:
            successes.append(
                InterfaceEventLogger(directory, prefix="race-open-success")
            )
        except BaseException as exc:  # pragma: no cover - 线程异常带回主测试线程
            errors.append(exc)

    monkeypatch.setattr(Path, "mkdir", synchronized_mkdir)
    monkeypatch.setattr(Path, "open", controlled_open)
    threads = [Thread(target=construct_failure), Thread(target=construct_success)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=3.0)

    try:
        assert all(not thread.is_alive() for thread in threads)
        assert len(successes) == 1
        assert len(errors) == 1
        assert str(errors[0]) == "injected race open failure"
    finally:
        for logger in successes:
            logger.close()


def test_thread_start_failure_cannot_remove_shared_constructor_directory(
    tmp_path,
    monkeypatch,
):
    directory = tmp_path / "shared-thread-race"
    mkdir_barrier = Barrier(2)
    failure_done = Event()
    real_mkdir = Path.mkdir
    real_open = Path.open
    real_thread_start = interface_logging.Thread.start
    successes = []
    errors = []

    def synchronized_mkdir(path, *args, **kwargs):
        if path == directory:
            mkdir_barrier.wait(timeout=2.0)
        return real_mkdir(path, *args, **kwargs)

    def controlled_open(path, *args, **kwargs):
        if "race-thread-success" in path.name and path.name.endswith(".interfaces.bin"):
            assert failure_done.wait(timeout=2.0)
        return real_open(path, *args, **kwargs)

    def controlled_thread_start(thread):
        if thread.name == "interface-log-race-thread-fail":
            raise RuntimeError("injected race thread start failure")
        return real_thread_start(thread)

    def construct_failure():
        try:
            InterfaceEventLogger(directory, prefix="race-thread-fail")
        except BaseException as exc:  # pragma: no cover - 线程异常带回主测试线程
            errors.append(exc)
        finally:
            failure_done.set()

    def construct_success():
        try:
            successes.append(
                InterfaceEventLogger(directory, prefix="race-thread-success")
            )
        except BaseException as exc:  # pragma: no cover - 线程异常带回主测试线程
            errors.append(exc)

    monkeypatch.setattr(Path, "mkdir", synchronized_mkdir)
    monkeypatch.setattr(Path, "open", controlled_open)
    monkeypatch.setattr(interface_logging.Thread, "start", controlled_thread_start)
    threads = [Thread(target=construct_failure), Thread(target=construct_success)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=3.0)

    try:
        assert all(not thread.is_alive() for thread in threads)
        assert len(successes) == 1
        assert len(errors) == 1
        assert str(errors[0]) == "injected race thread start failure"
    finally:
        for logger in successes:
            logger.close()


def test_thread_start_failure_cleans_files_but_leaves_shareable_directory(
    tmp_path,
    monkeypatch,
):
    directory = tmp_path / "created-parent" / "logs"

    def fail_start(_thread):
        raise RuntimeError("injected thread start failure")

    monkeypatch.setattr(interface_logging.Thread, "start", fail_start)

    with pytest.raises(RuntimeError, match="thread start failure"):
        InterfaceEventLogger(directory, prefix="partial-thread")

    assert directory.is_dir()
    assert list(directory.iterdir()) == []


def test_thread_start_failure_after_target_entry_rolls_back_transactionally(
    tmp_path,
    monkeypatch,
):
    directory = tmp_path / "post-start-rollback"
    real_open = Path.open
    real_unlink = Path.unlink
    real_queue = interface_logging.Queue
    real_run_writer = InterfaceEventLogger._run_writer
    real_thread_start = interface_logging.Thread.start
    real_thread_join = interface_logging.Thread.join
    start_error = RuntimeError("injected post-start failure")
    target_started = Event()
    owners = []
    workers = []
    constructor_errors = []
    worker_alive_at_return = []
    unlink_worker_states = []
    streams = {}

    class StopThenRaiseQueue:
        """先实际投递 stop，再模拟通知操作自身抛错。"""

        def __init__(self, *args, **kwargs):
            self.delegate = real_queue(*args, **kwargs)

        def put_nowait(self, item):
            self.delegate.put_nowait(item)
            if item is interface_logging._STOP:
                raise OSError("injected stop failure")

        def get(self, *args, **kwargs):
            return self.delegate.get(*args, **kwargs)

        def get_nowait(self):
            return self.delegate.get_nowait()

    def wrap_open(path, *args, **kwargs):
        stream = real_open(path, *args, **kwargs)
        if "post-start-rollback" not in path.name:
            return stream
        label = "binary" if path.name.endswith(".interfaces.bin") else "event"
        wrapped = CloseRaisingStream(stream, label)
        streams[label] = wrapped
        return wrapped

    def observe_writer_target(logger):
        owners.append(logger)
        target_started.set()
        return real_run_writer(logger)

    def start_then_raise(thread):
        if thread.name != "interface-log-post-start-rollback":
            return real_thread_start(thread)
        workers.append(thread)
        real_thread_start(thread)
        assert target_started.wait(timeout=1.0)
        raise start_error

    def join_then_raise(thread, *args, **kwargs):
        result = real_thread_join(thread, *args, **kwargs)
        if thread in workers:
            raise OSError("injected join failure")
        return result

    def unlink_then_raise(path, *args, **kwargs):
        if "post-start-rollback" not in path.name:
            return real_unlink(path, *args, **kwargs)
        unlink_worker_states.append(workers[0].is_alive())
        real_unlink(path, *args, **kwargs)
        raise OSError("injected unlink failure")

    def construct_logger():
        try:
            InterfaceEventLogger(directory, prefix="post-start-rollback")
        except BaseException as exc:  # 线程内异常带回主测试线程断言
            constructor_errors.append(exc)
        finally:
            worker_alive_at_return.append(bool(workers and workers[0].is_alive()))

    monkeypatch.setattr(interface_logging, "Queue", StopThenRaiseQueue)
    monkeypatch.setattr(Path, "open", wrap_open)
    monkeypatch.setattr(Path, "unlink", unlink_then_raise)
    monkeypatch.setattr(InterfaceEventLogger, "_run_writer", observe_writer_target)
    monkeypatch.setattr(interface_logging.Thread, "start", start_then_raise)
    monkeypatch.setattr(interface_logging.Thread, "join", join_then_raise)

    constructor_thread = Thread(
        target=construct_logger,
        name="test-post-start-constructor",
        daemon=True,
    )
    real_thread_start(constructor_thread)
    real_thread_join(constructor_thread, timeout=2.0)
    constructor_finished_in_time = not constructor_thread.is_alive()

    # 即使旧实现留下 worker，也先有界回收，再执行任何可能失败的断言。
    for owner, worker in zip(owners, workers):
        if worker.is_alive():
            with owner._condition:
                owner._accepting = False
            try:
                owner._queue.put_nowait(interface_logging._STOP)
            except BaseException:
                pass
            real_thread_join(worker, timeout=2.0)
    real_thread_join(constructor_thread, timeout=2.0)
    residual_threads = [
        thread
        for thread in (constructor_thread, *workers)
        if thread.is_alive()
    ]
    for path in directory.iterdir() if directory.exists() else ():
        real_unlink(path)
    force_close_test_streams(streams)

    assert constructor_finished_in_time
    assert target_started.is_set()
    assert len(constructor_errors) == 1
    assert constructor_errors[0] is start_error
    assert worker_alive_at_return == [False]
    assert unlink_worker_states == [False, False]
    assert {name: stream.close_calls for name, stream in streams.items()} == {
        "binary": 1,
        "event": 1,
    }
    assert residual_threads == []


def test_partial_initialization_force_closes_fd_when_stream_close_raises(
    tmp_path,
    monkeypatch,
):
    real_open = Path.open
    streams = {}

    def fail_after_binary_open(path, *args, **kwargs):
        if path.name.endswith(".events.jsonl"):
            raise OSError("injected event open failure")
        wrapped = CloseRaisingStream(
            real_open(path, *args, **kwargs),
            "binary",
        )
        streams["binary"] = wrapped
        return wrapped

    monkeypatch.setattr(Path, "open", fail_after_binary_open)

    try:
        with pytest.raises(OSError, match="event open failure"):
            InterfaceEventLogger(tmp_path / "logs", prefix="fd-partial-open")
        assert streams["binary"].close_calls == 1
        assert_fd_is_closed(streams["binary"].fd)
    finally:
        force_close_test_streams(streams)


def test_thread_start_failure_force_closes_both_open_file_descriptors(
    tmp_path,
    monkeypatch,
):
    real_open = Path.open
    streams = {}

    def wrap_open(path, *args, **kwargs):
        label = "binary" if path.name.endswith(".interfaces.bin") else "event"
        wrapped = CloseRaisingStream(real_open(path, *args, **kwargs), label)
        streams[label] = wrapped
        return wrapped

    def fail_start(_thread):
        raise RuntimeError("injected thread start failure")

    monkeypatch.setattr(Path, "open", wrap_open)
    monkeypatch.setattr(interface_logging.Thread, "start", fail_start)

    try:
        with pytest.raises(RuntimeError, match="thread start failure"):
            InterfaceEventLogger(tmp_path / "logs", prefix="fd-thread-start")
        assert {name: stream.close_calls for name, stream in streams.items()} == {
            "binary": 1,
            "event": 1,
        }
        for stream in streams.values():
            assert_fd_is_closed(stream.fd)
    finally:
        force_close_test_streams(streams)


def test_worker_close_failure_force_closes_both_fds_and_has_stable_error(
    tmp_path,
    monkeypatch,
):
    real_open = Path.open
    streams = {}

    def wrap_open(path, *args, **kwargs):
        label = "binary" if path.name.endswith(".interfaces.bin") else "event"
        wrapped = CloseRaisingStream(real_open(path, *args, **kwargs), label)
        streams[label] = wrapped
        return wrapped

    monkeypatch.setattr(Path, "open", wrap_open)
    logger = InterfaceEventLogger(tmp_path, prefix="fd-worker-close", queue_size=2)
    assert logger.record_message(sample_record(1))
    assert logger.record_event("sensor_failed", reason="fd test")

    try:
        with pytest.raises(RuntimeError, match="^interface log writer failed$"):
            logger.close()
        with pytest.raises(RuntimeError, match="^interface log writer failed$"):
            logger.close()
        assert {name: stream.close_calls for name, stream in streams.items()} == {
            "binary": 1,
            "event": 1,
        }
        assert logger.snapshot().closed
        assert logger.snapshot().writer_failed
        for stream in streams.values():
            assert_fd_is_closed(stream.fd)
    finally:
        force_close_test_streams(streams)


def test_forced_fd_close_cannot_later_close_a_reused_descriptor(
    tmp_path,
    monkeypatch,
):
    real_open = Path.open
    streams = {}

    def fail_after_binary_open(path, *args, **kwargs):
        if path.name.endswith(".events.jsonl"):
            raise OSError("injected event open failure")
        wrapped = CloseRaisingStream(real_open(path, *args, **kwargs), "binary")
        streams["binary"] = wrapped
        return wrapped

    monkeypatch.setattr(Path, "open", fail_after_binary_open)
    with pytest.raises(OSError, match="event open failure"):
        InterfaceEventLogger(tmp_path / "logs", prefix="fd-reuse")

    stale_fd = streams["binary"].fd
    replacement_fd = os.open(
        tmp_path / "replacement.bin",
        os.O_CREAT | os.O_RDWR,
        0o600,
    )
    assert replacement_fd == stale_fd
    try:
        streams.clear()
        gc.collect()
        os.fstat(replacement_fd)
    finally:
        try:
            os.close(replacement_fd)
        except OSError as error:
            if error.errno != errno.EBADF:
                raise


def test_fd_fallback_never_finalizes_stream_after_descriptor_is_reused(
    tmp_path,
    monkeypatch,
):
    stream = DelayedFinalizerStream((tmp_path / "stale.bin").open("wb"))
    replacement_path = tmp_path / "replacement-after-close.bin"
    replacement_fd = None

    def close_then_reuse(fd):
        nonlocal replacement_fd
        if fd != stream.fd:
            _REAL_OS_CLOSE(fd)
            return
        _REAL_OS_CLOSE(fd)
        replacement_fd = os.open(replacement_path, os.O_CREAT | os.O_RDWR, 0o600)
        assert replacement_fd == fd

    monkeypatch.setattr(interface_logging.os, "close", close_then_reuse)
    try:
        captured_fd = interface_logging._capture_stream_fd(stream)
        error = interface_logging._close_stream_with_fd(stream, captured_fd)

        assert isinstance(error, OSError)
        assert replacement_fd == stream.fd
        os.fstat(replacement_fd)
        assert stream.finalizer_calls == 1
        assert_fd_is_closed(captured_fd.guard_number)
    finally:
        if replacement_fd is not None:
            try:
                _REAL_OS_CLOSE(replacement_fd)
            except OSError as error:
                if error.errno != errno.EBADF:
                    raise


def test_fd_fallback_skips_descriptor_reused_during_high_level_close(tmp_path):
    stale_path = tmp_path / "stale-before-fallback.bin"
    stream = CloseReusesFdStream(
        stale_path.open("wb"),
        stale_path,
    )
    captured_fd = interface_logging._capture_stream_fd(stream)
    try:
        error = interface_logging._close_stream_with_fd(stream, captured_fd)

        assert isinstance(error, OSError)
        assert stream.replacement_fd == stream.fd
        os.fstat(stream.replacement_fd)
        assert_fd_is_closed(captured_fd.guard_number)
    finally:
        if stream.replacement_fd is not None:
            try:
                _REAL_OS_CLOSE(stream.replacement_fd)
            except OSError as error:
                if error.errno != errno.EBADF:
                    raise


@pytest.mark.parametrize("failed_operation", ("flush", "fsync"))
def test_flush_or_fsync_failure_still_finalizes_both_streams(
    tmp_path,
    monkeypatch,
    failed_operation,
):
    real_open = Path.open
    real_fsync = interface_logging.os.fsync
    streams = {}
    fsync_calls = []

    def wrap_open(path, *args, **kwargs):
        label = "binary" if path.name.endswith(".interfaces.bin") else "event"
        wrapped = FinalizationTrackingStream(
            real_open(path, *args, **kwargs),
            label,
            fail_flush=failed_operation == "flush" and label == "binary",
        )
        streams[label] = wrapped
        return wrapped

    def controlled_fsync(fd):
        fsync_calls.append(fd)
        if failed_operation == "fsync" and fd == logger._binary_fd.guard_number:
            raise OSError("injected binary fsync failure")
        return real_fsync(fd)

    monkeypatch.setattr(Path, "open", wrap_open)
    monkeypatch.setattr(interface_logging.os, "fsync", controlled_fsync)
    logger = InterfaceEventLogger(
        tmp_path,
        prefix=f"finalize-{failed_operation}",
        queue_size=2,
    )
    assert logger.record_message(sample_record(1))
    assert logger.record_event("sensor_failed", reason="finalize test")

    with pytest.raises(RuntimeError, match="^interface log writer failed$"):
        logger.close()

    assert {name: stream.flush_calls for name, stream in streams.items()} == {
        "binary": 1,
        "event": 1,
    }
    assert fsync_calls == [
        logger._binary_fd.guard_number,
        logger._event_fd.guard_number,
    ]
    assert {name: stream.close_calls for name, stream in streams.items()} == {
        "binary": 1,
        "event": 1,
    }
    for stream in streams.values():
        assert_fd_is_closed(stream.fd)
