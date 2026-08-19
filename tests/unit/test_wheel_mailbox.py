# 轮子命令邮箱单元测试：锁定原子校验、100 ms 安全超时和并发墙钟顺序。
from __future__ import annotations

from dataclasses import FrozenInstanceError, replace
import math
from threading import Event, Thread

import pytest

from slope_sim.interfaces.models import WheelCommand
from slope_sim.interfaces.wheel import WheelCommandMailbox, WheelDecision
from slope_sim.model_registry import get_robot_model


def _differential_command(timestamp_ns: int, left: float = 1.0, right: float = 2.0) -> WheelCommand:
    return WheelCommand(timestamp_ns, (left, right), ())


def _active_command(timestamp_ns: int) -> WheelCommand:
    return WheelCommand(timestamp_ns, (1.0, 2.0, 3.0, 4.0), (0.5, -0.5))


def test_invalid_command_does_not_replace_or_refresh_last_valid_command_timeout():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    valid = _differential_command(100)
    invalid = _differential_command(200, left=20.01)

    assert mailbox.accept(valid, received_at=10.0)
    assert not mailbox.accept(invalid, received_at=10.08)

    before_timeout = mailbox.decision(now=10.08)
    assert before_timeout.drive_wheel_speed_rad_s == valid.drive_wheel_speed_rad_s
    assert not before_timeout.waiting
    assert not before_timeout.timed_out
    assert mailbox.snapshot(now=10.08).state == "invalid_command"

    decision = mailbox.decision(now=10.101)
    status = mailbox.snapshot(now=10.101)
    assert decision.drive_wheel_speed_rad_s == (0.0, 0.0)
    assert decision.steering_wheel_speed_rad_s == ()
    assert not decision.waiting
    assert decision.timed_out
    assert status.state == "timed_out"
    assert status.latest_timestamp_ns == 100
    assert status.valid_count == 1
    assert status.invalid_count == 1
    assert status.last_error is not None


def test_active_steering_clear_returns_to_waiting_with_four_plus_two_zero_arrays():
    mailbox = WheelCommandMailbox(get_robot_model("active_steering_4wd"))
    assert mailbox.accept(_active_command(10), received_at=1.0)
    assert not mailbox.accept(WheelCommand(11, (1.0,), ()), received_at=1.01)

    mailbox.clear()

    decision = mailbox.decision(now=1.02)
    status = mailbox.snapshot(now=1.02)
    assert decision.waiting
    assert not decision.timed_out
    assert decision.drive_wheel_speed_rad_s == (0.0, 0.0, 0.0, 0.0)
    assert decision.steering_wheel_speed_rad_s == (0.0, 0.0)
    assert status.state == "waiting_command"
    assert status.latest_timestamp_ns is None
    assert status.valid_count == 1
    assert status.invalid_count == 1
    assert status.last_error is None


def test_two_valid_commands_use_latest_and_count_every_accept():
    mailbox = WheelCommandMailbox(get_robot_model("df_back"))
    first = _differential_command(1, 1.0, 2.0)
    latest = _differential_command(2, 3.0, 4.0)

    assert mailbox.accept(first, received_at=1.0)
    assert mailbox.accept(latest, received_at=1.01)

    decision = mailbox.decision(now=1.02)
    status = mailbox.snapshot(now=1.02)
    assert decision.drive_wheel_speed_rad_s == latest.drive_wheel_speed_rad_s
    assert status.state == "active"
    assert status.latest_timestamp_ns == latest.timestamp_ns
    assert status.valid_count == 2
    assert status.invalid_count == 0
    assert status.valid_hz == pytest.approx(50.0)


def test_latest_timestamp_read_does_not_advance_query_time():
    """Dashboard 元数据读取最新 sender 时间，不得参与 mailbox 的 safety query 时钟。"""
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))

    assert mailbox.decision(now=10.0).waiting
    assert mailbox.latest_timestamp_ns() is None
    command = _differential_command(123)
    assert mailbox.accept(command, received_at=9.0)
    before = mailbox.snapshot(now=10.0)

    assert mailbox.latest_timestamp_ns() == command.timestamp_ns
    assert mailbox.snapshot(now=10.0) == before
    assert mailbox.decision(now=10.1).timed_out
    assert mailbox.snapshot(now=10.1).state == "timed_out"
    with pytest.raises(ValueError, match="now"):
        mailbox.decision(now=10.05)


def test_command_frequency_uses_configurable_window_and_default_remains_two_seconds() -> None:
    one_second = WheelCommandMailbox(
        get_robot_model("df_mid"),
        frequency_window_sec=1.0,
    )
    default_window = WheelCommandMailbox(get_robot_model("df_mid"))
    for mailbox in (one_second, default_window):
        assert mailbox.accept(_differential_command(1), received_at=0.0)
        assert mailbox.accept(_differential_command(2), received_at=1.0)

    assert one_second.snapshot(now=1.5).valid_hz == 0.0
    assert default_window.snapshot(now=1.5).valid_hz == pytest.approx(2.0 / 3.0)


@pytest.mark.parametrize(
    ("now", "timed_out"),
    ((0.099999, False), (0.100000, True), (0.100001, True)),
)
def test_timeout_boundary_is_exactly_one_hundred_milliseconds(now: float, timed_out: bool):
    mailbox = WheelCommandMailbox(get_robot_model("df_front"), timeout_sec=0.100)
    assert mailbox.accept(_differential_command(123), received_at=0.0)

    decision = mailbox.decision(now=now)

    assert decision.timed_out is timed_out
    assert decision.waiting is False
    expected = (0.0, 0.0) if timed_out else (1.0, 2.0)
    assert decision.drive_wheel_speed_rad_s == expected


@pytest.mark.parametrize(
    ("received_at", "now", "timed_out"),
    (
        (0.008, 0.108, True),
        (10.0, 10.1, False),
        (1e16, 1e16, False),
    ),
)
def test_timeout_uses_direct_elapsed_subtraction_review_counterexamples(
    received_at: float,
    now: float,
    timed_out: bool,
):
    mailbox = WheelCommandMailbox(get_robot_model("df_front"), timeout_sec=0.100)
    assert mailbox.accept(_differential_command(123), received_at=received_at)

    decision = mailbox.decision(now=now)

    assert decision.timed_out is timed_out


def test_sender_timestamp_does_not_participate_in_safety_timeout():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    command = _differential_command((1 << 64) - 1)

    assert mailbox.accept(command, received_at=5.0)

    decision = mailbox.decision(now=5.05)
    assert decision.drive_wheel_speed_rad_s == command.drive_wheel_speed_rad_s
    assert not decision.timed_out


def test_waiting_and_timed_out_decisions_are_frozen():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    waiting = mailbox.decision(now=0.0)

    with pytest.raises(FrozenInstanceError):
        waiting.waiting = False

    assert mailbox.accept(_differential_command(1), received_at=0.0)
    timed_out = mailbox.decision(now=0.1)
    with pytest.raises(FrozenInstanceError):
        timed_out.timed_out = False


def test_active_decision_defaults_to_nonwaiting_and_not_timed_out_and_is_frozen():
    decision = WheelDecision((1.0, 2.0), ())

    assert not decision.waiting
    assert not decision.timed_out
    with pytest.raises(FrozenInstanceError):
        decision.waiting = True


@pytest.mark.parametrize("timeout_sec", (True, 0.0, -0.1, math.nan, math.inf, -math.inf, "0.1"))
def test_mailbox_rejects_invalid_timeout(timeout_sec):
    with pytest.raises(ValueError, match="timeout_sec"):
        WheelCommandMailbox(get_robot_model("df_mid"), timeout_sec=timeout_sec)


@pytest.mark.parametrize(
    "frequency_window_sec",
    (True, 0.0, -1.0, math.nan, math.inf, -math.inf, "2.0"),
)
def test_mailbox_rejects_invalid_frequency_window(frequency_window_sec) -> None:
    with pytest.raises(ValueError, match="frequency_window_sec"):
        WheelCommandMailbox(
            get_robot_model("df_mid"),
            frequency_window_sec=frequency_window_sec,
        )


def test_mailbox_rejects_invalid_or_unsupported_model():
    with pytest.raises(ValueError, match="model"):
        WheelCommandMailbox(object())

    unsupported = replace(get_robot_model("df_mid"), controller_kind="unknown")
    with pytest.raises(ValueError, match="controller_kind"):
        WheelCommandMailbox(unsupported)


@pytest.mark.parametrize("received_at", (True, -1.0, math.nan, math.inf, -math.inf, "1"))
def test_invalid_accept_wall_time_raises_without_counting_or_replacing(received_at):
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))

    with pytest.raises(ValueError, match="received_at"):
        mailbox.accept(_differential_command(1), received_at=received_at)

    status = mailbox.snapshot(now=0.0)
    assert status.state == "waiting_command"
    assert status.valid_count == 0
    assert status.invalid_count == 0


@pytest.mark.parametrize("now", (True, -1.0, math.nan, math.inf, -math.inf, "1"))
def test_invalid_decision_and_snapshot_wall_times_leave_state_unchanged(now):
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    assert mailbox.accept(_differential_command(1), received_at=0.0)

    with pytest.raises(ValueError, match="now"):
        mailbox.decision(now=now)
    with pytest.raises(ValueError, match="now"):
        mailbox.snapshot(now=now)

    status = mailbox.snapshot(now=0.0)
    assert status.state == "active"
    assert status.valid_count == 1


def test_non_wheel_command_is_invalid_without_replacing_latest():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    valid = _differential_command(1)
    assert mailbox.accept(valid, received_at=1.0)

    assert not mailbox.accept(object(), received_at=1.01)

    decision = mailbox.decision(now=1.02)
    status = mailbox.snapshot(now=1.02)
    assert decision.drive_wheel_speed_rad_s == valid.drive_wheel_speed_rad_s
    assert status.valid_count == 1
    assert status.invalid_count == 1
    assert status.state == "invalid_command"
    assert "WheelCommand" in status.last_error


def test_invalid_event_time_does_not_block_decision_at_an_earlier_captured_query_time():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    valid = _differential_command(1)
    invalid = _differential_command(2, left=20.01)
    assert mailbox.accept(valid, received_at=1.95)
    assert not mailbox.accept(invalid, received_at=2.001)

    decision = mailbox.decision(now=2.0)
    status = mailbox.snapshot(now=2.0)

    assert decision.drive_wheel_speed_rad_s == valid.drive_wheel_speed_rad_s
    assert not decision.timed_out
    assert status.state == "invalid_command"
    assert status.valid_count == 1
    assert status.invalid_count == 1


def test_invalid_command_without_a_previous_valid_command_remains_visible_in_status():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))

    assert not mailbox.accept(_differential_command(1, left=20.01), received_at=1.0)

    decision = mailbox.decision(now=1.0)
    status = mailbox.snapshot(now=1.0)
    assert decision.waiting
    assert not decision.timed_out
    assert status.state == "invalid_command"
    assert status.latest_timestamp_ns is None
    assert status.valid_count == 0
    assert status.invalid_count == 1


def test_wall_clock_rollback_is_rejected_without_partial_state():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    first = _differential_command(1)
    assert mailbox.accept(first, received_at=10.0)
    assert mailbox.snapshot(now=10.0).state == "active"

    with pytest.raises(ValueError, match="received_at"):
        mailbox.accept(_differential_command(2), received_at=9.0)
    with pytest.raises(ValueError, match="now"):
        mailbox.decision(now=9.0)

    status = mailbox.snapshot(now=10.0)
    assert status.latest_timestamp_ns == first.timestamp_ns
    assert status.valid_count == 1
    assert status.invalid_count == 0
    assert status.state == "active"


def test_invalid_command_with_invalid_time_does_not_increment_invalid_count():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    invalid = _differential_command(1, left=20.01)

    with pytest.raises(ValueError, match="received_at"):
        mailbox.accept(invalid, received_at=math.nan)

    assert mailbox.snapshot(now=0.0).invalid_count == 0


def test_clear_invalidates_an_inflight_callback_generation_without_mutating_state():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    initial = _differential_command(1)
    stale_invalid = _differential_command(2, left=20.01)
    fresh = _differential_command(3, left=3.0, right=4.0)
    assert mailbox.accept(initial, received_at=8.0)
    stale_generation = mailbox.capture_generation()
    callback_waiting = Event()
    resume_callback = Event()
    callback_results: list[bool] = []

    def delayed_stale_callback() -> None:
        callback_waiting.set()
        assert resume_callback.wait(timeout=5.0)
        callback_results.append(
            mailbox.accept(stale_invalid, received_at=9.0, generation=stale_generation)
        )

    callback = Thread(target=delayed_stale_callback)
    callback.start()
    assert callback_waiting.wait(timeout=2.0)

    mailbox.clear()
    new_generation = mailbox.capture_generation()
    waiting_status = mailbox.snapshot(now=8.0)
    resume_callback.set()
    callback.join(timeout=2.0)

    assert not callback.is_alive()
    assert new_generation == stale_generation + 1
    assert callback_results == [False]
    assert waiting_status.state == "waiting_command"
    assert waiting_status.valid_count == 1
    assert waiting_status.invalid_count == 0
    assert waiting_status.last_error is None
    assert mailbox.accept(fresh, received_at=8.5, generation=new_generation)
    decision = mailbox.decision(now=8.5)
    status = mailbox.snapshot(now=8.5)
    assert decision.drive_wheel_speed_rad_s == fresh.drive_wheel_speed_rad_s
    assert status.latest_timestamp_ns == fresh.timestamp_ns
    assert status.valid_count == 2
    assert status.invalid_count == 0


def test_delayed_callback_timestamp_captured_before_query_remains_a_legal_interleaving():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    timestamp_captured = Event()
    resume_callback = Event()
    callback_errors: list[BaseException] = []

    def delayed_accept() -> None:
        received_at = 2.0
        timestamp_captured.set()
        try:
            if not resume_callback.wait(timeout=5.0):
                raise TimeoutError("callback was not resumed")
            mailbox.accept(_differential_command(2), received_at=received_at)
        except BaseException as exc:  # pragma: no cover - 线程异常带回主测试线程
            callback_errors.append(exc)

    callback = Thread(target=delayed_accept)
    callback.start()
    try:
        assert timestamp_captured.wait(timeout=5.0)
        assert mailbox.snapshot(now=10.0).state == "waiting_command"
    finally:
        resume_callback.set()
        callback.join(timeout=5.0)

    assert not callback.is_alive()
    assert callback_errors == []
    decision = mailbox.decision(now=10.0)
    status = mailbox.snapshot(now=10.0)
    assert decision.timed_out
    assert decision.drive_wheel_speed_rad_s == (0.0, 0.0)
    assert status.state == "timed_out"
    assert status.valid_count == 1
    assert status.valid_hz == 0.0


def test_valid_callback_after_query_capture_uses_frequency_horizon_without_changing_safety_now():
    mailbox = WheelCommandMailbox(get_robot_model("df_mid"))
    query_now = 2.0
    query_captured = Event()
    callback_finished = Event()
    callback_errors: list[BaseException] = []

    def callback() -> None:
        try:
            assert query_captured.wait(timeout=5.0)
            mailbox.accept(_differential_command(10), received_at=2.001)
        except BaseException as exc:  # pragma: no cover - 线程异常带回主测试线程
            callback_errors.append(exc)
        finally:
            callback_finished.set()

    callback_thread = Thread(target=callback)
    callback_thread.start()
    query_captured.set()
    assert callback_finished.wait(timeout=2.0)
    callback_thread.join(timeout=2.0)

    assert not callback_thread.is_alive()
    assert callback_errors == []
    decision = mailbox.decision(now=query_now)
    status = mailbox.snapshot(now=query_now)
    assert decision.drive_wheel_speed_rad_s == (1.0, 2.0)
    assert not decision.timed_out
    assert status.state == "active"
    assert status.valid_hz == 0.0
