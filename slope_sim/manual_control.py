# 手动控制模块：把 PyBullet GUI 键盘事件转换成差速车速度命令。
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import time

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


class KeyboardEventTracker:
    """跨过 X11 自动重复的伪释放间隙，并在真实松键后有界清除方向键。"""

    _DIRECTION_KEYS = frozenset(
        (
            p.B3G_UP_ARROW,
            p.B3G_DOWN_ARROW,
            p.B3G_LEFT_ARROW,
            p.B3G_RIGHT_ARROW,
            ord("w"),
            ord("s"),
            ord("a"),
            ord("d"),
        )
    )

    def __init__(self, *, release_grace_sec: float = 0.05) -> None:
        if isinstance(release_grace_sec, bool) or release_grace_sec <= 0.0:
            raise ValueError("release_grace_sec must be positive")
        self._release_grace_sec = float(release_grace_sec)
        self._held_keys: set[int] = set()
        self._pending_release_at: dict[int, float] = {}

    def command(
        self,
        keyboard_events: Mapping[int, int],
        settings: ManualControlSettings,
        allow_exit: bool = True,
        *,
        now: float | None = None,
    ) -> ManualCommand:
        if not isinstance(keyboard_events, Mapping):
            raise ValueError("keyboard_events must be a mapping")
        observed_at = time.monotonic() if now is None else float(now)
        for key, state in keyboard_events.items():
            if key not in self._DIRECTION_KEYS:
                continue
            if state & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                self._held_keys.add(key)
                self._pending_release_at.pop(key, None)
            elif state & p.KEY_WAS_RELEASED and key in self._held_keys:
                self._pending_release_at[key] = observed_at + self._release_grace_sec
        for key, deadline in tuple(self._pending_release_at.items()):
            if observed_at >= deadline:
                self._held_keys.discard(key)
                del self._pending_release_at[key]

        normalized = {
            key: state
            for key, state in keyboard_events.items()
            if key not in self._DIRECTION_KEYS
        }
        normalized.update({key: p.KEY_IS_DOWN for key in self._held_keys})
        return command_from_keyboard(normalized, settings, allow_exit)


def command_from_keyboard(
    keyboard_events: Mapping[int, int],
    settings: ManualControlSettings,
    allow_exit: bool = True,
) -> ManualCommand:
    """把 PyBullet 键盘事件转换成差速车的线速度和角速度命令。"""
    if allow_exit and (_pressed(keyboard_events, ord("q")) or _pressed(keyboard_events, ESCAPE_KEY)):
        return ManualCommand(0.0, 0.0, should_exit=True)

    if _pressed(keyboard_events, ord(" ")):
        return ManualCommand(0.0, 0.0)

    forward = _held(keyboard_events, p.B3G_UP_ARROW) or _held(keyboard_events, ord("w"))
    backward = _held(keyboard_events, p.B3G_DOWN_ARROW) or _held(keyboard_events, ord("s"))
    left = _held(keyboard_events, p.B3G_LEFT_ARROW) or _held(keyboard_events, ord("a"))
    right = _held(keyboard_events, p.B3G_RIGHT_ARROW) or _held(keyboard_events, ord("d"))

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
