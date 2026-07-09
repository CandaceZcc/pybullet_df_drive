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
        "velocity_sensor_vx",
        "velocity_sensor_vy",
        "velocity_sensor_vz",
        "velocity_sensor_body_forward_speed",
        "velocity_sensor_yaw_rate",
        "linear_acceleration_x",
        "linear_acceleration_y",
        "linear_acceleration_z",
        "angular_acceleration_z",
        "left_actual_drive_speed",
        "right_actual_drive_speed",
        "left_track_surface_speed",
        "right_track_surface_speed",
        "left_body_track_speed",
        "right_body_track_speed",
        "left_contact_normal_force",
        "right_contact_normal_force",
        "left_contact_friction_force",
        "right_contact_friction_force",
        "left_slip_ratio",
        "right_slip_ratio",
        "left_slip_speed",
        "right_slip_speed",
        "local_ground_height",
        "local_terrain_normal_x",
        "local_terrain_normal_y",
        "local_terrain_normal_z",
        "lidar_min_distance",
    }
)


def _fmt(value: float, unit: str = "", digits: int = 2) -> str:
    if isinstance(value, float) and math.isnan(value):
        return "--"
    return f"{value:.{digits}f}{unit}"


def _fmt_signed(value: float, unit: str = "", digits: int = 2) -> str:
    if isinstance(value, float) and math.isnan(value):
        return "--"
    return f"{value:+.{digits}f}{unit}"


def _fmt_slip(value: float, valid: bool) -> str:
    if not valid:
        return "低速"
    return _fmt_signed(value)


def dashboard_window_size(available_width: int, available_height: int) -> tuple[int, int]:
    """根据屏幕可用区域给侧边栏一个保守初始大小。"""
    width = min(520, max(320, int(available_width * 0.45)))
    height = min(1200, max(280, int(available_height * 0.90)))
    return width, height


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


def dashboard_groups(telemetry: RobotTelemetry) -> list[tuple[str, list[tuple[str, str]]]]:
    """把遥测数据按主题分组，侧窗和单元测试共用同一套显示规则。"""
    return [
        (
            "位姿",
            [
                ("位置 x/y/z", f"{_fmt(telemetry.x)} / {_fmt(telemetry.y)} / {_fmt(telemetry.z)} m"),
                (
                    "姿态 roll/pitch/yaw",
                    f"{_fmt(math.degrees(telemetry.roll), digits=1)} / "
                    f"{_fmt(math.degrees(telemetry.pitch), digits=1)} / "
                    f"{_fmt(math.degrees(telemetry.yaw), digits=1)} deg",
                ),
            ],
        ),
        (
            "速度",
            [
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
            ],
        ),
        (
            "速度传感",
            [
                (
                    "速度传感 vx/vy/vz",
                    f"{_fmt(telemetry.velocity_sensor_vx)} / {_fmt(telemetry.velocity_sensor_vy)} / {_fmt(telemetry.velocity_sensor_vz)} m/s",
                ),
                (
                    "速度传感前向/yaw",
                    f"{_fmt(telemetry.velocity_sensor_body_forward_speed)} m/s / {_fmt(telemetry.velocity_sensor_yaw_rate)} rad/s",
                ),
                (
                    "加速度 xyz",
                    f"{_fmt(telemetry.linear_acceleration_x)} / {_fmt(telemetry.linear_acceleration_y)} / {_fmt(telemetry.linear_acceleration_z)} m/s^2",
                ),
                ("角加速度 z", f"{_fmt(telemetry.angular_acceleration_z)} rad/s^2"),
            ],
        ),
        (
            "接触 / 打滑",
            [
                ("签名打滑率", f"{_fmt_slip(telemetry.left_slip_ratio, telemetry.left_slip_valid)} / {_fmt_slip(telemetry.right_slip_ratio, telemetry.right_slip_valid)}"),
                ("打滑速度差", f"{_fmt_signed(telemetry.left_slip_speed)} / {_fmt_signed(telemetry.right_slip_speed)} m/s"),
                ("接触法向力", f"{_fmt(telemetry.left_contact_normal_force)} / {_fmt(telemetry.right_contact_normal_force)} N"),
                ("接触摩擦力", f"{_fmt(telemetry.left_contact_friction_force)} / {_fmt(telemetry.right_contact_friction_force)} N"),
                ("有效接触点", f"{telemetry.left_contact_count} / {telemetry.right_contact_count}"),
            ],
        ),
        (
            "地形 / 摩擦",
            [
                ("地形类型", telemetry.terrain_type),
                ("地面高度 / 法向 z", f"{_fmt(telemetry.local_ground_height)} m / {_fmt(telemetry.local_terrain_normal_z)}"),
                (
                    "地形法向 x/y/z",
                    f"{_fmt(telemetry.local_terrain_normal_x)} / {_fmt(telemetry.local_terrain_normal_y)} / {_fmt(telemetry.local_terrain_normal_z)}",
                ),
                (
                    "地面摩擦 lat/roll/spin",
                    f"{_fmt(telemetry.ground_lateral_friction)} / {_fmt(telemetry.ground_rolling_friction)} / {_fmt(telemetry.ground_spinning_friction)}",
                ),
                ("驱动/支撑摩擦", f"{_fmt(telemetry.drive_lateral_friction)} / {_fmt(telemetry.support_lateral_friction)}"),
                (
                    "履带各向异性摩擦",
                    f"{_fmt(telemetry.track_anisotropic_friction_x)} / "
                    f"{_fmt(telemetry.track_anisotropic_friction_y)} / "
                    f"{_fmt(telemetry.track_anisotropic_friction_z)}",
                ),
            ],
        ),
        (
            "传感器 / 命令",
            [
                ("最近障碍距离", f"{_fmt(telemetry.lidar_min_distance)} m"),
                (
                    "当前命令 v/w",
                    f"{_fmt(telemetry.command_linear_velocity)} m/s / {_fmt(telemetry.command_angular_velocity)} rad/s",
                ),
            ],
        ),
    ]


def dashboard_rows(telemetry: RobotTelemetry) -> list[tuple[str, str]]:
    """把分组遥测展平成旧接口需要的行列表。"""
    rows: list[tuple[str, str]] = []
    for _group_name, group_rows in dashboard_groups(telemetry):
        rows.extend(group_rows)
    return rows


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
        self.window.setMinimumWidth(320)
        self.window.setFocusPolicy(QtCore.Qt.StrongFocus)
        screen = self.app.primaryScreen()
        if screen is not None:
            available = screen.availableGeometry()
            width, height = dashboard_window_size(available.width(), available.height())
            self.window.resize(width, height)

        layout = QtWidgets.QVBoxLayout(self.window)
        title = QtWidgets.QLabel("实时小车数据反馈")
        title_font = QtGui.QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        self.labels: dict[str, object] = {}
        label_font = QtGui.QFont()
        label_font.setPointSize(11)
        value_font = QtGui.QFont("Monospace")
        value_font.setPointSize(11)
        value_font.setStyleHint(QtGui.QFont.Monospace)
        group_font = QtGui.QFont()
        group_font.setPointSize(12)
        group_font.setBold(True)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        for group_name, rows in dashboard_groups(RobotTelemetry()):
            group_label = QtWidgets.QLabel(group_name)
            group_label.setFont(group_font)
            content_layout.addWidget(group_label)
            grid = QtWidgets.QGridLayout()
            grid.setColumnStretch(1, 1)
            for row, (name, value) in enumerate(rows):
                name_label = QtWidgets.QLabel(name)
                value_label = QtWidgets.QLabel(value)
                name_label.setFont(label_font)
                value_label.setFont(value_font)
                name_label.setWordWrap(True)
                value_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
                grid.addWidget(name_label, row, 0)
                grid.addWidget(value_label, row, 1)
                self.labels[name] = value_label
            content_layout.addLayout(grid)
        content_layout.addStretch(1)
        scroll.setWidget(content)
        layout.addWidget(scroll, stretch=1)

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
