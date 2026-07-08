# Dashboard 模块：提供实时遥测侧窗和可测试的数据格式化函数。
from __future__ import annotations

import math
import time
from dataclasses import dataclass, fields, replace

from slope_sim.telemetry import RobotTelemetry


SMOOTHED_TELEMETRY_FIELDS = frozenset(
    {
        "x",
        "y",
        "z",
        "roll",
        "pitch",
        "yaw",
        "body_forward_speed",
        "yaw_rate",
        "left_actual_drive_speed",
        "right_actual_drive_speed",
        "left_track_surface_speed",
        "right_track_surface_speed",
        "left_body_track_speed",
        "right_body_track_speed",
        "left_contact_normal_force",
        "right_contact_normal_force",
        "left_slip_ratio",
        "right_slip_ratio",
        "lidar_min_distance",
    }
)


def _fmt(value: float, unit: str = "", digits: int = 2) -> str:
    if isinstance(value, float) and math.isnan(value):
        return "--"
    return f"{value:.{digits}f}{unit}"


def should_refresh_dashboard(last_update_time: float | None, now: float, update_hz: float) -> bool:
    """判断侧窗是否到达下一次刷新时间，避免 240Hz 数字闪烁。"""
    if update_hz <= 0:
        raise ValueError("update_hz must be positive")
    if last_update_time is None:
        return True
    return now - last_update_time >= (1.0 / update_hz) - 1e-9


def smooth_telemetry(previous: RobotTelemetry | None, current: RobotTelemetry, alpha: float) -> RobotTelemetry:
    """只平滑物理反馈字段，命令/目标值保持当前值，避免控制显示滞后。"""
    if not 0.0 < alpha <= 1.0:
        raise ValueError("alpha must be in (0, 1]")
    if previous is None:
        return current

    smoothed_values = {}
    for field in fields(RobotTelemetry):
        value = getattr(current, field.name)
        if field.name not in SMOOTHED_TELEMETRY_FIELDS:
            continue
        previous_value = getattr(previous, field.name)
        # NaN 表示该传感器当前不可用，此时保留 NaN，让界面显示 "--"。
        if isinstance(value, float) and math.isnan(value):
            continue
        if isinstance(previous_value, float) and math.isnan(previous_value):
            smoothed_values[field.name] = value
        else:
            smoothed_values[field.name] = previous_value + alpha * (value - previous_value)
    return replace(current, **smoothed_values)


def dashboard_rows(telemetry: RobotTelemetry) -> list[tuple[str, str]]:
    """把遥测数据转换成侧边栏显示行，便于单元测试和 GUI 复用。"""
    return [
        ("位置 x/y/z", f"{_fmt(telemetry.x)} / {_fmt(telemetry.y)} / {_fmt(telemetry.z)} m"),
        (
            "姿态 roll/pitch/yaw",
            f"{_fmt(math.degrees(telemetry.roll), digits=1)} / "
            f"{_fmt(math.degrees(telemetry.pitch), digits=1)} / "
            f"{_fmt(math.degrees(telemetry.yaw), digits=1)} deg",
        ),
        ("车体速度 / yaw_rate", f"{_fmt(telemetry.body_forward_speed)} m/s / {_fmt(telemetry.yaw_rate)} rad/s"),
        (
            "左右实际驱动速度",
            f"{_fmt(telemetry.left_actual_drive_speed)} / {_fmt(telemetry.right_actual_drive_speed)} rad/s",
        ),
        (
            "驱动表面速度",
            f"{_fmt(telemetry.left_track_surface_speed)} / {_fmt(telemetry.right_track_surface_speed)} m/s",
        ),
        (
            "驱动局部车速",
            f"{_fmt(telemetry.left_body_track_speed)} / {_fmt(telemetry.right_body_track_speed)} m/s",
        ),
        ("左右打滑率", f"{_fmt(telemetry.left_slip_ratio)} / {_fmt(telemetry.right_slip_ratio)}"),
        ("接触法向力", f"{_fmt(telemetry.left_contact_normal_force)} / {_fmt(telemetry.right_contact_normal_force)} N"),
        ("最近障碍距离", f"{_fmt(telemetry.lidar_min_distance)} m"),
        (
            "当前命令 v/w",
            f"{_fmt(telemetry.command_linear_velocity)} m/s / {_fmt(telemetry.command_angular_velocity)} rad/s",
        ),
    ]


@dataclass(frozen=True)
class DashboardCommand:
    """Dashboard 输出给仿真循环的手动速度命令。"""

    linear_velocity: float
    angular_velocity: float
    should_exit: bool = False


class TelemetryDashboard:
    """PySide6 实时侧边栏；导入 PySide6 延迟到实例化时，避免影响 DIRECT 测试。"""

    def __init__(
        self,
        max_linear_speed: float,
        max_angular_speed: float,
        update_hz: float = 5.0,
        smoothing_alpha: float = 0.35,
    ) -> None:
        """创建遥测窗口、控制按钮和按键状态。"""
        from PySide6 import QtCore, QtGui, QtWidgets

        self.QtCore = QtCore
        self.QtGui = QtGui
        self.QtWidgets = QtWidgets
        self._pressed_keys: set[int] = set()
        self._button_keys: set[int] = set()
        self._should_exit = False
        self.update_hz = update_hz
        self.smoothing_alpha = smoothing_alpha
        self._last_update_time: float | None = None
        self._smoothed_telemetry: RobotTelemetry | None = None

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        self.window = QtWidgets.QWidget()
        self.window.setWindowTitle("Step 2 Telemetry Dashboard")
        self.window.setMinimumWidth(390)
        self.window.setFocusPolicy(QtCore.Qt.StrongFocus)

        layout = QtWidgets.QVBoxLayout(self.window)
        title = QtWidgets.QLabel("实时小车数据反馈")
        title_font = QtGui.QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        self.labels: dict[str, object] = {}
        grid = QtWidgets.QGridLayout()
        label_font = QtGui.QFont()
        label_font.setPointSize(12)
        value_font = QtGui.QFont("Monospace")
        value_font.setPointSize(12)
        value_font.setStyleHint(QtGui.QFont.Monospace)
        for row, (name, value) in enumerate(dashboard_rows(RobotTelemetry())):
            name_label = QtWidgets.QLabel(name)
            value_label = QtWidgets.QLabel(value)
            name_label.setFont(label_font)
            value_label.setFont(value_font)
            grid.addWidget(name_label, row, 0)
            grid.addWidget(value_label, row, 1)
            self.labels[name] = value_label
        layout.addLayout(grid)

        self.linear_spin = QtWidgets.QDoubleSpinBox()
        self.linear_spin.setRange(0.0, 2.0)
        self.linear_spin.setSingleStep(0.05)
        self.linear_spin.setValue(max_linear_speed)
        self.angular_spin = QtWidgets.QDoubleSpinBox()
        self.angular_spin.setRange(0.0, 4.0)
        self.angular_spin.setSingleStep(0.05)
        self.angular_spin.setValue(max_angular_speed)
        layout.addWidget(QtWidgets.QLabel("最大线速度 m/s"))
        layout.addWidget(self.linear_spin)
        layout.addWidget(QtWidgets.QLabel("最大角速度 rad/s"))
        layout.addWidget(self.angular_spin)

        button_grid = QtWidgets.QGridLayout()
        self._add_button(button_grid, "↑", 0, 1, QtCore.Qt.Key_Up)
        self._add_button(button_grid, "←", 1, 0, QtCore.Qt.Key_Left)
        self._add_button(button_grid, "■", 1, 1, QtCore.Qt.Key_Space)
        self._add_button(button_grid, "→", 1, 2, QtCore.Qt.Key_Right)
        self._add_button(button_grid, "↓", 2, 1, QtCore.Qt.Key_Down)
        quit_button = QtWidgets.QPushButton("退出")
        quit_button.clicked.connect(self.request_exit)
        button_grid.addWidget(quit_button, 3, 0, 1, 3)
        layout.addLayout(button_grid)

        self.window.keyPressEvent = self._key_press_event
        self.window.keyReleaseEvent = self._key_release_event
        self.window.show()
        self.window.activateWindow()
        self.window.setFocus()

    def _add_button(self, grid: object, text: str, row: int, col: int, key: int) -> None:
        button = self.QtWidgets.QPushButton(text)
        button.setMinimumHeight(48)
        button.pressed.connect(lambda key=key: self._button_keys.add(key))
        button.released.connect(lambda key=key: self._button_keys.discard(key))
        grid.addWidget(button, row, col)

    def _key_press_event(self, event: object) -> None:
        key = event.key()
        if key in {self.QtCore.Qt.Key_Q, self.QtCore.Qt.Key_Escape}:
            self.request_exit()
        else:
            self._pressed_keys.add(key)

    def _key_release_event(self, event: object) -> None:
        self._pressed_keys.discard(event.key())

    def request_exit(self) -> None:
        self._should_exit = True

    def process_events(self) -> None:
        self.app.processEvents()
        if not self.window.isVisible():
            self._should_exit = True

    def current_command(self) -> DashboardCommand:
        keys = self._pressed_keys | self._button_keys
        if self.QtCore.Qt.Key_Space in keys:
            return DashboardCommand(0.0, 0.0, self._should_exit)
        linear = 0.0
        angular = 0.0
        if self.QtCore.Qt.Key_Up in keys:
            linear += self.linear_spin.value()
        if self.QtCore.Qt.Key_Down in keys:
            linear -= self.linear_spin.value()
        if self.QtCore.Qt.Key_Left in keys:
            angular += self.angular_spin.value()
        if self.QtCore.Qt.Key_Right in keys:
            angular -= self.angular_spin.value()
        return DashboardCommand(linear, angular, self._should_exit)

    def update(self, telemetry: RobotTelemetry) -> bool:
        """按较低频率刷新显示，并对物理反馈做一阶平滑。"""
        now = time.monotonic()
        if not should_refresh_dashboard(self._last_update_time, now, self.update_hz):
            return False
        display_telemetry = smooth_telemetry(self._smoothed_telemetry, telemetry, self.smoothing_alpha)
        self._smoothed_telemetry = display_telemetry
        self._last_update_time = now
        for name, value in dashboard_rows(display_telemetry):
            self.labels[name].setText(value)
        return True

    def close(self) -> None:
        self.window.close()
        self.app.processEvents()
