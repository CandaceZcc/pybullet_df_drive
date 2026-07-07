# 手动控制模块：把 PyBullet GUI 键盘事件转换成差速车速度命令。
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import pybullet as p

ESCAPE_KEY = 27


@dataclass(frozen=True)
class ManualControlSettings:
    """手动控制的速度上限，由 GUI 滑条实时调整。"""

    max_linear_speed: float = 0.4
    max_angular_speed: float = 0.8


@dataclass(frozen=True)
class ManualCommand:
    """一次键盘读取后的车体速度命令。"""

    linear_velocity: float
    angular_velocity: float
    should_exit: bool = False


def command_from_keyboard(
    keyboard_events: Mapping[int, int],
    settings: ManualControlSettings,
) -> ManualCommand:
    """把 PyBullet 键盘事件转换成差速车的线速度和角速度命令。"""
    if _pressed(keyboard_events, ord("q")) or _pressed(keyboard_events, ESCAPE_KEY):
        return ManualCommand(0.0, 0.0, should_exit=True)

    if _pressed(keyboard_events, ord(" ")):
        return ManualCommand(0.0, 0.0)

    forward = _held(keyboard_events, p.B3G_UP_ARROW)
    backward = _held(keyboard_events, p.B3G_DOWN_ARROW)
    left = _held(keyboard_events, p.B3G_LEFT_ARROW)
    right = _held(keyboard_events, p.B3G_RIGHT_ARROW)

    linear_direction = int(forward) - int(backward)
    angular_direction = int(left) - int(right)
    return ManualCommand(
        linear_velocity=linear_direction * settings.max_linear_speed,
        angular_velocity=angular_direction * settings.max_angular_speed,
    )


def _held(keyboard_events: Mapping[int, int], key: int) -> bool:
    """判断某个键当前是否处于按住状态。"""
    state = keyboard_events.get(key, 0)
    return bool(state & p.KEY_IS_DOWN)


def _pressed(keyboard_events: Mapping[int, int], key: int) -> bool:
    """判断某个键是否被按下或刚刚触发。"""
    state = keyboard_events.get(key, 0)
    return bool(state & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED))
