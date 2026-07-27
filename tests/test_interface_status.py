# 接口状态测试：锁定不可变快照、状态边界和滚动频率统计。
from __future__ import annotations

from dataclasses import FrozenInstanceError
import math
from threading import Barrier, Event, Lock, Thread

import pytest

from slope_sim.interfaces.models import WheelState
from slope_sim.interfaces.status import (
    COMMAND_STATES,
    TOPIC_STATES,
    InterfaceStatusSnapshot,
    RollingFrequency,
    TopicStatus,
    WheelCommandStatus,
)


UINT64_MAX = (1 << 64) - 1


def _topic(**changes) -> TopicStatus:
    values = {
        "topic": "/sim/wheel/state",
        "direction": "publish",
        "state": "active",
        "target_hz": 100.0,
        "actual_hz": 99.5,
        "latest_timestamp_ns": 10,
        "message_count": 20,
    }
    values.update(changes)
    return TopicStatus(**values)


def _command(**changes) -> WheelCommandStatus:
    values = {
        "state": "active",
        "valid_hz": 100.0,
        "latest_timestamp_ns": 10,
        "valid_count": 20,
        "invalid_count": 1,
    }
    values.update(changes)
    return WheelCommandStatus(**values)


def test_state_constants_and_every_allowed_state_are_exact():
    assert TOPIC_STATES == {
        "active",
        "waiting_peer",
        "timed_out",
        "degraded",
        "disconnected",
        "error",
    }
    assert COMMAND_STATES == {
        "waiting_command",
        "active",
        "invalid_command",
        "timed_out",
        "disconnected",
    }
    assert {_topic(state=state).state for state in TOPIC_STATES} == TOPIC_STATES
    assert {_command(state=state).state for state in COMMAND_STATES} == COMMAND_STATES


def test_status_dataclasses_are_frozen_and_apply_defaults():
    topic = _topic(latest_timestamp_ns=None)
    command = _command(latest_timestamp_ns=None)
    snapshot = InterfaceStatusSnapshot(0.0, "local", False, command, None, {topic.topic: topic})

    assert (topic.error_count, topic.dropped_count, topic.detail) == (0, 0, "")
    assert command.last_error is None
    with pytest.raises(FrozenInstanceError):
        topic.state = "error"
    with pytest.raises(FrozenInstanceError):
        command.state = "timed_out"
    with pytest.raises(FrozenInstanceError):
        snapshot.transport_mode = "auto"


@pytest.mark.parametrize(
    "changes",
    (
        {"topic": ""},
        {"topic": 1},
        {"direction": "unknown"},
        {"direction": 1},
        {"state": "unknown"},
        {"state": 1},
        {"detail": None},
    ),
)
def test_topic_status_rejects_invalid_text_fields(changes):
    with pytest.raises(ValueError):
        _topic(**changes)


@pytest.mark.parametrize(
    ("field", "invalid"),
    (
        ("target_hz", True),
        ("target_hz", 0.0),
        ("target_hz", -1.0),
        ("target_hz", math.nan),
        ("target_hz", math.inf),
        ("actual_hz", True),
        ("actual_hz", -1.0),
        ("actual_hz", math.nan),
        ("actual_hz", math.inf),
    ),
)
def test_topic_status_rejects_invalid_frequencies(field, invalid):
    with pytest.raises(ValueError, match=field):
        _topic(**{field: invalid})


@pytest.mark.parametrize("timestamp", (None, 0, UINT64_MAX))
def test_topic_and_command_status_accept_uint64_timestamp_boundaries(timestamp):
    assert _topic(latest_timestamp_ns=timestamp).latest_timestamp_ns == timestamp
    assert _command(latest_timestamp_ns=timestamp).latest_timestamp_ns == timestamp


@pytest.mark.parametrize("invalid", (True, -1, 1.0, UINT64_MAX + 1))
def test_topic_and_command_status_reject_invalid_timestamps(invalid):
    with pytest.raises(ValueError, match="latest_timestamp_ns"):
        _topic(latest_timestamp_ns=invalid)
    with pytest.raises(ValueError, match="latest_timestamp_ns"):
        _command(latest_timestamp_ns=invalid)


@pytest.mark.parametrize("field", ("message_count", "error_count", "dropped_count"))
@pytest.mark.parametrize("invalid", (True, -1, 1.0))
def test_topic_status_rejects_invalid_counts(field, invalid):
    with pytest.raises(ValueError, match=field):
        _topic(**{field: invalid})


@pytest.mark.parametrize("invalid", ("unknown", 1, None))
def test_wheel_command_status_rejects_invalid_state(invalid):
    with pytest.raises(ValueError, match="state"):
        _command(state=invalid)


@pytest.mark.parametrize("invalid", (True, -1.0, math.nan, math.inf, "100"))
def test_wheel_command_status_rejects_invalid_frequency(invalid):
    with pytest.raises(ValueError, match="valid_hz"):
        _command(valid_hz=invalid)


@pytest.mark.parametrize("field", ("valid_count", "invalid_count"))
@pytest.mark.parametrize("invalid", (True, -1, 1.0))
def test_wheel_command_status_rejects_invalid_counts(field, invalid):
    with pytest.raises(ValueError, match=field):
        _command(**{field: invalid})


def test_wheel_command_status_requires_optional_string_error():
    assert _command(last_error="bad command").last_error == "bad command"
    with pytest.raises(ValueError, match="last_error"):
        _command(last_error=1)


def test_snapshot_accepts_none_wheel_state_while_waiting_for_command():
    command = _command(
        state="waiting_command",
        valid_hz=0.0,
        latest_timestamp_ns=None,
        valid_count=0,
        invalid_count=0,
    )

    snapshot = InterfaceStatusSnapshot(0.0, "local", False, command, None, {})

    assert snapshot.wheel_state is None


def test_snapshot_exposes_frozen_wheel_state_and_defensively_copies_topics():
    topic = _topic()
    wheel_state = WheelState(10, (1.0, 2.0), ())
    source = {topic.topic: topic}
    snapshot = InterfaceStatusSnapshot(12.5, "ecal", True, _command(), wheel_state, source)

    source.clear()
    assert snapshot.wheel_state is wheel_state
    assert snapshot.wheel_state.drive_wheel_speed_rad_s == (1.0, 2.0)
    assert dict(snapshot.topics) == {topic.topic: topic}
    with pytest.raises(TypeError):
        snapshot.topics[topic.topic] = topic
    with pytest.raises(FrozenInstanceError):
        snapshot.command.valid_count = 21
    with pytest.raises(FrozenInstanceError):
        snapshot.wheel_state.timestamp_ns = 11


@pytest.mark.parametrize("invalid", (object(), []))
def test_snapshot_rejects_invalid_wheel_state(invalid):
    with pytest.raises(ValueError, match="wheel_state"):
        InterfaceStatusSnapshot(0.0, "local", False, _command(), invalid, {})


@pytest.mark.parametrize("invalid", (True, -1.0, math.nan, math.inf, "1"))
def test_snapshot_rejects_invalid_captured_at(invalid):
    with pytest.raises(ValueError, match="captured_at"):
        InterfaceStatusSnapshot(invalid, "local", False, _command(), None, {})


@pytest.mark.parametrize("invalid", ("unknown", 1, None))
def test_snapshot_rejects_invalid_transport_mode(invalid):
    with pytest.raises(ValueError, match="transport_mode"):
        InterfaceStatusSnapshot(0.0, invalid, False, _command(), None, {})


def test_snapshot_rejects_invalid_connection_command_and_topic_mapping():
    topic = _topic()
    with pytest.raises(ValueError, match="ecal_connected"):
        InterfaceStatusSnapshot(0.0, "local", 0, _command(), None, {})
    with pytest.raises(ValueError, match="command"):
        InterfaceStatusSnapshot(0.0, "local", False, object(), None, {})
    with pytest.raises(ValueError, match="topics"):
        InterfaceStatusSnapshot(0.0, "local", False, _command(), None, [])
    with pytest.raises(ValueError, match="key"):
        InterfaceStatusSnapshot(0.0, "local", False, _command(), None, {1: topic})
    with pytest.raises(ValueError, match="TopicStatus"):
        InterfaceStatusSnapshot(0.0, "local", False, _command(), None, {topic.topic: object()})
    with pytest.raises(ValueError, match="topic"):
        InterfaceStatusSnapshot(0.0, "local", False, _command(), None, {"/other": topic})


def test_rolling_frequency_uses_two_second_window_and_decays():
    frequency = RollingFrequency(window_sec=2.0)
    for index in range(201):
        frequency.record(index * 0.01)

    assert frequency.hz(2.0) == pytest.approx(100.0)
    assert frequency.hz(2.5) == pytest.approx(75.0)
    assert frequency.hz(4.0) == 0.0
    assert frequency.hz(4.01) == 0.0


def test_rolling_frequency_retains_window_boundary_and_handles_zero_span():
    frequency = RollingFrequency(window_sec=2.0)
    frequency.record(0.0)
    frequency.record(2.0)
    assert frequency.hz(2.0) == pytest.approx(0.5)

    duplicate = RollingFrequency()
    duplicate.record(1.0)
    duplicate.record(1.0)
    assert duplicate.hz(1.0) == 0.0


def test_rolling_frequency_rejects_out_of_order_events_and_early_now():
    frequency = RollingFrequency()
    frequency.record(1.0)
    with pytest.raises(ValueError, match="timestamp"):
        frequency.record(0.5)
    with pytest.raises(ValueError, match="now"):
        frequency.hz(0.5)


def test_rolling_frequency_allows_captured_event_after_query_without_revival():
    frequency = RollingFrequency()
    frequency.record(1.0)
    timestamp_captured = Event()
    resume_callback = Event()
    callback_errors: list[BaseException] = []

    def delayed_callback() -> None:
        event_timestamp = 2.0
        timestamp_captured.set()
        try:
            if not resume_callback.wait(timeout=5.0):
                raise TimeoutError("callback was not resumed")
            frequency.record(event_timestamp)
        except BaseException as exc:  # pragma: no cover - 仅用于把线程异常带回主测试线程
            callback_errors.append(exc)

    callback = Thread(target=delayed_callback)
    callback.start()
    try:
        assert timestamp_captured.wait(timeout=5.0)
        assert frequency.hz(10.0) == 0.0
    finally:
        resume_callback.set()
        callback.join(timeout=5.0)

    assert not callback.is_alive()
    assert callback_errors == []
    assert tuple(frequency._events) == ()
    assert frequency.hz(10.0) == 0.0
    with pytest.raises(ValueError, match="now"):
        frequency.hz(9.0)


def test_rolling_frequency_serializes_callback_and_main_thread_access():
    class TrackingLock:
        """记录两个线程进入同一真实互斥锁临界区的次数。"""

        def __init__(self) -> None:
            self._lock = Lock()
            self.entry_count = 0
            self.exit_count = 0
            self.active_count = 0
            self.max_active_count = 0

        def __enter__(self):
            self._lock.acquire()
            self.entry_count += 1
            self.active_count += 1
            self.max_active_count = max(self.max_active_count, self.active_count)
            return self

        def __exit__(self, exc_type, exc_value, traceback) -> None:
            self.active_count -= 1
            self.exit_count += 1
            self._lock.release()

    frequency = RollingFrequency()
    assert hasattr(frequency._lock, "acquire")
    tracking_lock = TrackingLock()
    frequency._lock = tracking_lock
    start_barrier = Barrier(3)
    errors: list[BaseException] = []

    def invoke(operation) -> None:
        try:
            start_barrier.wait(timeout=5.0)
            operation()
        except BaseException as exc:  # pragma: no cover - 仅用于把线程异常带回主测试线程
            errors.append(exc)

    threads = (
        Thread(target=invoke, args=(lambda: frequency.record(0.0),)),
        Thread(target=invoke, args=(lambda: frequency.hz(0.0),)),
    )
    for thread in threads:
        thread.start()
    start_barrier.wait(timeout=5.0)

    for thread in threads:
        thread.join(timeout=5.0)
    assert not any(thread.is_alive() for thread in threads)
    assert errors == []
    assert tracking_lock.entry_count == tracking_lock.exit_count == 2
    assert tracking_lock.max_active_count == 1


@pytest.mark.parametrize("invalid", (True, 0.0, -1.0, math.nan, math.inf, "2"))
def test_rolling_frequency_rejects_invalid_window(invalid):
    with pytest.raises(ValueError, match="window_sec"):
        RollingFrequency(invalid)


@pytest.mark.parametrize("invalid", (True, -1.0, math.nan, math.inf, "1"))
def test_rolling_frequency_rejects_invalid_record_and_now_values(invalid):
    frequency = RollingFrequency()
    with pytest.raises(ValueError, match="timestamp"):
        frequency.record(invalid)
    with pytest.raises(ValueError, match="now"):
        frequency.hz(invalid)
