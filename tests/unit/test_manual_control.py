# 手动控制单元测试：保护方向键、空格和退出键的速度命令映射。
import pybullet as p
import pytest

from slope_sim.manual_control import (
    ESCAPE_KEY,
    KeyboardEventTracker,
    ManualControlSettings,
    command_from_keyboard,
)
from slope_sim.serial_rc import CommandSourceArbiter


def _down(*keys: int) -> dict[int, int]:
    return {key: p.KEY_IS_DOWN for key in keys}


def _triggered(*keys: int) -> dict[int, int]:
    return {key: p.KEY_WAS_TRIGGERED for key in keys}


def test_arrow_keys_map_to_forward_and_left_turn_command():
    settings = ManualControlSettings(max_linear_speed=0.7, max_angular_speed=1.2)

    command = command_from_keyboard(_down(p.B3G_UP_ARROW, p.B3G_LEFT_ARROW), settings)

    assert command.linear_velocity == pytest.approx(0.7)
    assert command.angular_velocity == pytest.approx(1.2)
    assert command.should_exit is False


def test_wasd_keys_map_to_forward_and_left_turn_command():
    """手动验收规定的 W/S/A/D 必须与方向键产生相同的差速目标。"""
    settings = ManualControlSettings(max_linear_speed=0.7, max_angular_speed=1.2)

    command = command_from_keyboard(_down(ord("w"), ord("a")), settings)

    assert command.linear_velocity == pytest.approx(0.7)
    assert command.angular_velocity == pytest.approx(1.2)
    assert command.should_exit is False


def test_opposite_arrow_keys_cancel_each_other():
    settings = ManualControlSettings(max_linear_speed=0.7, max_angular_speed=1.2)

    command = command_from_keyboard(
        _down(p.B3G_UP_ARROW, p.B3G_DOWN_ARROW, p.B3G_LEFT_ARROW, p.B3G_RIGHT_ARROW),
        settings,
    )

    assert command.linear_velocity == 0.0
    assert command.angular_velocity == 0.0


def test_space_stops_and_q_or_escape_requests_exit():
    settings = ManualControlSettings(max_linear_speed=0.7, max_angular_speed=1.2)

    stop = command_from_keyboard(_triggered(ord(" ")), settings)
    quit_with_q = command_from_keyboard(_triggered(ord("q")), settings)
    quit_with_escape = command_from_keyboard(_triggered(ESCAPE_KEY), settings)

    assert stop.linear_velocity == 0.0
    assert stop.angular_velocity == 0.0
    assert stop.should_exit is False
    assert quit_with_q.should_exit is True
    assert quit_with_escape.should_exit is True


def test_exit_keys_can_be_disabled_while_dashboard_owns_window_events():
    """Dashboard 操作产生的 Esc/Q 不能让 PyBullet 主循环意外收束。"""
    settings = ManualControlSettings(max_linear_speed=0.7, max_angular_speed=1.2)

    command = command_from_keyboard(
        _triggered(ESCAPE_KEY),
        settings,
        allow_exit=False,
    )

    assert command.linear_velocity == 0.0
    assert command.angular_velocity == 0.0
    assert command.should_exit is False


def test_keyboard_tracker_bridges_x11_auto_repeat_release_but_stops_after_grace() -> None:
    """PyBullet 无法标记 X11 伪释放；短 release/press 间隙必须连续，真松键须有界停车。"""
    tracker = KeyboardEventTracker(release_grace_sec=0.05)
    settings = ManualControlSettings(max_linear_speed=0.7, max_angular_speed=1.2)

    pressed = tracker.command(
        {p.B3G_UP_ARROW: p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED}, settings, now=1.00
    )
    repeat_release = tracker.command(
        {p.B3G_UP_ARROW: p.KEY_WAS_RELEASED}, settings, now=1.01
    )
    repeat_gap = tracker.command({}, settings, now=1.03)
    repeated = tracker.command(
        {p.B3G_UP_ARROW: p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED}, settings, now=1.04
    )
    real_release = tracker.command(
        {p.B3G_UP_ARROW: p.KEY_WAS_RELEASED}, settings, now=2.00
    )
    stopped = tracker.command({}, settings, now=2.051)

    assert pressed.linear_velocity == pytest.approx(0.7)
    assert repeat_release.linear_velocity == pytest.approx(0.7)
    assert repeat_gap.linear_velocity == pytest.approx(0.7)
    assert repeated.linear_velocity == pytest.approx(0.7)
    assert real_release.linear_velocity == pytest.approx(0.7)
    assert stopped.linear_velocity == 0.0


def test_keyboard_auto_repeat_stays_nonzero_through_command_arbiter() -> None:
    """输入跟踪后的每帧目标必须穿过唯一仲裁器，伪释放期间不得提交零值。"""
    sent: list[tuple[float, float]] = []

    class Client:
        @staticmethod
        def send_target(linear: float, angular: float, *, now: float) -> None:
            del now
            sent.append((linear, angular))

    tracker = KeyboardEventTracker(release_grace_sec=0.05)
    arbiter = CommandSourceArbiter(Client())
    settings = ManualControlSettings(max_linear_speed=0.7, max_angular_speed=1.2)
    arbiter.select_source("keyboard", now=0.0)

    events = (
        (1.00, {p.B3G_UP_ARROW: p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED}),
        (1.01, {p.B3G_UP_ARROW: p.KEY_WAS_RELEASED}),
        (1.03, {}),
        (1.04, {p.B3G_UP_ARROW: p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED}),
        (2.00, {p.B3G_UP_ARROW: p.KEY_WAS_RELEASED}),
        (2.051, {}),
    )
    for observed_at, keyboard_events in events:
        command = tracker.command(keyboard_events, settings, now=observed_at)
        assert arbiter.submit_keyboard(
            command.linear_velocity,
            command.angular_velocity,
            now=observed_at,
        )
    arbiter.close(now=3.0)

    assert sent[0] == (0.0, 0.0)
    assert sent[1:6] == [(0.7, 0.0)] * 5
    assert sent[6:] == [(0.0, 0.0), (0.0, 0.0)]
