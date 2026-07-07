# 手动控制测试：保护方向键、空格和退出键的速度命令映射。
import pybullet as p
import pytest

from slope_sim.manual_control import ESCAPE_KEY, ManualControlSettings, command_from_keyboard


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
