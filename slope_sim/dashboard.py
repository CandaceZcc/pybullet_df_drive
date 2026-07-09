# Dashboard 模块：提供实时遥测侧窗和可测试的数据格式化函数。
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, fields, replace
from pathlib import Path

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

DASHBOARD_MAIN_AREA_STRETCH = 2
DASHBOARD_CONTROL_BAR_STRETCH = 0
DASHBOARD_DIRECTION_BUTTON_SIZE = 36
DASHBOARD_CONTROL_SPINBOX_WIDTH = 104
DASHBOARD_CONTROL_BAR_MAX_HEIGHT = 160
DASHBOARD_BUTTON_PULSE_SEC = 0.75


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


def _fmt_abs_slip(value: float, valid: bool) -> str:
    if not valid:
        return "低速"
    return _fmt(abs(value))


def dashboard_window_size(available_width: int, available_height: int) -> tuple[int, int]:
    """根据屏幕可用区域给实验监视窗口一个偏宽的初始大小。"""
    width = min(1180, max(820, int(available_width * 0.65)))
    width = min(width, max(320, int(available_width * 0.95)))
    height = min(1100, max(560, int(available_height * 0.88)))
    height = min(height, max(280, int(available_height * 0.95)))
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


class TelemetryPlotBuffer:
    """保存最近一段时间的遥测样本，供实时曲线和单元测试共用。"""

    def __init__(self, window_sec: float = 20.0) -> None:
        if window_sec <= 0:
            raise ValueError("window_sec must be positive")
        self.window_sec = window_sec
        self._samples: deque[RobotTelemetry] = deque()

    def append(self, telemetry: RobotTelemetry) -> None:
        """加入新样本，并裁掉窗口外的旧样本。"""
        self._samples.append(telemetry)
        cutoff = telemetry.t - self.window_sec
        while self._samples and self._samples[0].t < cutoff:
            self._samples.popleft()

    def clear(self) -> None:
        """清空当前曲线缓存。"""
        self._samples.clear()

    def series(self) -> dict[str, list[float]]:
        """返回 Matplotlib line.set_data 可直接使用的字段序列。"""
        fields_to_plot = (
            "t",
            "x",
            "y",
            "command_linear_velocity",
            "body_forward_speed",
            "command_angular_velocity",
            "yaw_rate",
            "left_slip_ratio",
            "right_slip_ratio",
            "left_slip_speed",
            "right_slip_speed",
            "left_contact_normal_force",
            "right_contact_normal_force",
            "left_contact_friction_force",
            "right_contact_friction_force",
            "left_contact_count",
            "right_contact_count",
        )
        series = {field: [float(getattr(sample, field)) for sample in self._samples] for field in fields_to_plot}
        # 曲线页用绝对值表达打滑严重度，带符号速度差仍用于判断驱动/制动方向。
        series["left_abs_slip_ratio"] = [abs(value) for value in series["left_slip_ratio"]]
        series["right_abs_slip_ratio"] = [abs(value) for value in series["right_slip_ratio"]]
        return series


@dataclass(frozen=True)
class DashboardPlotLine:
    """实时曲线中的一条线。"""

    key: str
    x_field: str
    y_field: str
    label: str


@dataclass(frozen=True)
class DashboardPlotSpec:
    """一个 Dashboard 曲线标签页，对应一张 Matplotlib 图。"""

    tab_label: str
    title: str
    x_label: str
    y_label: str
    lines: tuple[DashboardPlotLine, ...]
    equal_aspect: bool = False


DASHBOARD_PLOT_LEGEND_STYLE = {
    "fontsize": 7,
    "framealpha": 0.65,
    "borderpad": 0.25,
    "handlelength": 1.2,
    "loc": "upper right",
}


def dashboard_plot_specs() -> list[DashboardPlotSpec]:
    """定义 Dashboard 曲线页：一个标签页只放一张图。"""
    return [
        DashboardPlotSpec(
            tab_label="轨迹",
            title="x/y trajectory",
            x_label="x [m]",
            y_label="y [m]",
            lines=(DashboardPlotLine("trajectory", "x", "y", "xy"),),
            equal_aspect=True,
        ),
        DashboardPlotSpec(
            tab_label="速度/命令",
            title="command vs actual",
            x_label="t [s]",
            y_label="value",
            lines=(
                DashboardPlotLine("command_linear_velocity", "t", "command_linear_velocity", "cmd v"),
                DashboardPlotLine("body_forward_speed", "t", "body_forward_speed", "body v"),
                DashboardPlotLine("command_angular_velocity", "t", "command_angular_velocity", "cmd yaw"),
                DashboardPlotLine("yaw_rate", "t", "yaw_rate", "yaw_rate"),
            ),
        ),
        DashboardPlotSpec(
            tab_label="打滑",
            title="slip severity",
            x_label="t [s]",
            y_label="|ratio| / signed m/s",
            lines=(
                DashboardPlotLine("left_abs_slip_ratio", "t", "left_abs_slip_ratio", "L |ratio|"),
                DashboardPlotLine("right_abs_slip_ratio", "t", "right_abs_slip_ratio", "R |ratio|"),
                DashboardPlotLine("left_slip_speed", "t", "left_slip_speed", "L signed speed"),
                DashboardPlotLine("right_slip_speed", "t", "right_slip_speed", "R signed speed"),
            ),
        ),
        DashboardPlotSpec(
            tab_label="接触",
            title="contact",
            x_label="t [s]",
            y_label="force / count",
            lines=(
                DashboardPlotLine("left_contact_normal_force", "t", "left_contact_normal_force", "L N"),
                DashboardPlotLine("right_contact_normal_force", "t", "right_contact_normal_force", "R N"),
                DashboardPlotLine("left_contact_friction_force", "t", "left_contact_friction_force", "L F"),
                DashboardPlotLine("right_contact_friction_force", "t", "right_contact_friction_force", "R F"),
                DashboardPlotLine("left_contact_count", "t", "left_contact_count", "L cnt"),
                DashboardPlotLine("right_contact_count", "t", "right_contact_count", "R cnt"),
            ),
        ),
    ]


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
                ("打滑严重度 |率|", f"{_fmt_abs_slip(telemetry.left_slip_ratio, telemetry.left_slip_valid)} / {_fmt_abs_slip(telemetry.right_slip_ratio, telemetry.right_slip_valid)}"),
                ("带符号打滑率", f"{_fmt_slip(telemetry.left_slip_ratio, telemetry.left_slip_valid)} / {_fmt_slip(telemetry.right_slip_ratio, telemetry.right_slip_valid)}"),
                ("打滑速度差", f"{_fmt_signed(telemetry.left_slip_speed)} / {_fmt_signed(telemetry.right_slip_speed)} m/s"),
                ("接触法向力", f"{_fmt(telemetry.left_contact_normal_force)} / {_fmt(telemetry.right_contact_normal_force)} N"),
                ("接触摩擦力", f"{_fmt(telemetry.left_contact_friction_force)} / {_fmt(telemetry.right_contact_friction_force)} N"),
                ("有效接触点", f"{telemetry.left_contact_count} / {telemetry.right_contact_count}"),
            ],
        ),
        (
            "地形 / 摩擦",
            [
                ("车型", telemetry.robot_model),
                ("地形类型", telemetry.terrain_type),
                ("地形探测", "有效" if telemetry.terrain_probe_valid else "无效"),
                ("越界保护", "越界" if telemetry.out_of_bounds else "正常"),
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
                ("驱动电机力", f"{_fmt(telemetry.drive_motor_force)}"),
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
    requested_robot_model: str | None = None
    reset_requested: bool = False


class TelemetryDashboard:
    """PySide6 实时侧边栏；导入 PySide6 延迟到实例化时，避免影响 DIRECT 测试。"""

    def __init__(
        self,
        max_linear_speed: float,
        max_angular_speed: float,
        update_hz: float = 5.0,
        smoothing_alpha: float = 0.35,
        robot_model: str = "diff_drive",
        model_switch_enabled: bool = False,
        plot_update_hz: float = 5.0,
        plot_window_sec: float = 20.0,
        plot_snapshot_dir: str | Path = "results/figures",
    ) -> None:
        """创建遥测窗口、控制按钮和按键状态。"""
        from PySide6 import QtCore, QtGui, QtWidgets

        self.QtCore = QtCore
        self.QtGui = QtGui
        self.QtWidgets = QtWidgets
        self._pressed_keys: set[int] = set()
        self._button_keys: set[int] = set()
        self._button_pulses: dict[int, float] = {}
        self._should_exit = False
        self._reset_requested = False
        self.update_hz = update_hz
        self.smoothing_alpha = smoothing_alpha
        self.plot_update_hz = plot_update_hz
        self.plot_snapshot_dir = Path(plot_snapshot_dir)
        self.plot_buffer = TelemetryPlotBuffer(plot_window_sec)
        self._last_update_time: float | None = None
        self._last_plot_update_time: float | None = None
        self._smoothed_telemetry: RobotTelemetry | None = None

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        dashboard = self

        class DashboardKeyEventFilter(QtCore.QObject):
            """捕获子控件焦点下的方向键，避免 spinbox/tab 吃掉驾驶按键。"""

            def eventFilter(self, _watched: object, event: object) -> bool:
                if event.type() == QtCore.QEvent.KeyPress:
                    return dashboard._handle_key_press(event.key())
                if event.type() == QtCore.QEvent.KeyRelease:
                    return dashboard._handle_key_release(event.key())
                return False

        self._key_event_filter = DashboardKeyEventFilter()
        self.app.installEventFilter(self._key_event_filter)
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

        tabs = QtWidgets.QTabWidget()
        layout.addWidget(tabs, stretch=DASHBOARD_MAIN_AREA_STRETCH)
        data_tab = QtWidgets.QWidget()
        data_layout = QtWidgets.QVBoxLayout(data_tab)

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
        data_layout.addWidget(scroll, stretch=1)
        tabs.addTab(data_tab, "数据")
        self.tabs = tabs
        self._add_plot_tab(tabs)
        tabs.currentChanged.connect(lambda _index: self._draw_current_plot_canvas())

        control_bar = QtWidgets.QWidget()
        control_bar.setMaximumHeight(DASHBOARD_CONTROL_BAR_MAX_HEIGHT)
        control_layout = QtWidgets.QHBoxLayout(control_bar)
        control_layout.setContentsMargins(4, 4, 4, 4)
        control_layout.setSpacing(8)

        parameter_group = QtWidgets.QGroupBox("参数")
        parameter_layout = QtWidgets.QGridLayout(parameter_group)
        parameter_layout.setContentsMargins(8, 6, 8, 6)
        parameter_layout.setHorizontalSpacing(6)
        parameter_layout.setVerticalSpacing(4)
        self.linear_spin = QtWidgets.QDoubleSpinBox()
        self.linear_spin.setRange(0.0, 2.0)
        self.linear_spin.setSingleStep(0.05)
        self.linear_spin.setValue(max_linear_speed)
        self.linear_spin.setFixedWidth(DASHBOARD_CONTROL_SPINBOX_WIDTH)
        self.angular_spin = QtWidgets.QDoubleSpinBox()
        self.angular_spin.setRange(0.0, 4.0)
        self.angular_spin.setSingleStep(0.05)
        self.angular_spin.setValue(max_angular_speed)
        self.angular_spin.setFixedWidth(DASHBOARD_CONTROL_SPINBOX_WIDTH)
        parameter_layout.addWidget(QtWidgets.QLabel("最大线速度"), 0, 0)
        parameter_layout.addWidget(self.linear_spin, 0, 1)
        parameter_layout.addWidget(QtWidgets.QLabel("最大角速度"), 1, 0)
        parameter_layout.addWidget(self.angular_spin, 1, 1)
        self.robot_combo = None
        if model_switch_enabled:
            self.robot_combo = QtWidgets.QComboBox()
            self.robot_combo.addItem("diff_drive", "diff_drive")
            self.robot_combo.addItem("tracked_proxy", "tracked_proxy")
            self.robot_combo.setFixedWidth(DASHBOARD_CONTROL_SPINBOX_WIDTH)
            index = self.robot_combo.findData(robot_model)
            if index >= 0:
                self.robot_combo.setCurrentIndex(index)
            reload_button = QtWidgets.QPushButton("重载车辆")
            reload_button.clicked.connect(self.request_reset)
            reload_button.setFixedHeight(30)
            parameter_layout.addWidget(QtWidgets.QLabel("车型"), 2, 0)
            parameter_layout.addWidget(self.robot_combo, 2, 1)
            parameter_layout.addWidget(reload_button, 3, 0, 1, 2)
        control_layout.addWidget(parameter_group, stretch=0)
        control_layout.addStretch(1)

        control_group = QtWidgets.QGroupBox("控制")
        button_grid = QtWidgets.QGridLayout()
        button_grid.setContentsMargins(8, 6, 8, 6)
        button_grid.setHorizontalSpacing(4)
        button_grid.setVerticalSpacing(4)
        self._add_button(button_grid, "↑", 0, 1, QtCore.Qt.Key_Up)
        self._add_button(button_grid, "←", 1, 0, QtCore.Qt.Key_Left)
        self._add_button(button_grid, "■", 1, 1, QtCore.Qt.Key_Space)
        self._add_button(button_grid, "→", 1, 2, QtCore.Qt.Key_Right)
        self._add_button(button_grid, "↓", 2, 1, QtCore.Qt.Key_Down)
        quit_button = QtWidgets.QPushButton("退出")
        quit_button.setFixedHeight(28)
        quit_button.clicked.connect(self.request_exit)
        button_grid.addWidget(quit_button, 3, 0, 1, 3)
        control_group.setLayout(button_grid)
        control_layout.addWidget(control_group, stretch=0)
        layout.addWidget(control_bar, stretch=DASHBOARD_CONTROL_BAR_STRETCH)

        self.window.keyPressEvent = self._key_press_event
        self.window.keyReleaseEvent = self._key_release_event
        self.window.show()
        self.window.activateWindow()
        self.window.setFocus()

    def _add_button(self, grid: object, text: str, row: int, col: int, key: int) -> None:
        button = self.QtWidgets.QPushButton(text)
        button.setFixedSize(DASHBOARD_DIRECTION_BUTTON_SIZE, DASHBOARD_DIRECTION_BUTTON_SIZE)
        button.pressed.connect(lambda key=key: self._button_keys.add(self._normalize_key(key)))
        button.released.connect(lambda key=key: self._button_keys.discard(self._normalize_key(key)))
        button.clicked.connect(lambda _checked=False, key=key: self._pulse_button_key(key))
        grid.addWidget(button, row, col)

    def _add_plot_tab(self, tabs: object) -> None:
        """按曲线规格创建多个 Matplotlib 实时曲线页。"""
        from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
        from matplotlib.figure import Figure

        self.plot_specs = dashboard_plot_specs()
        self.plot_figures = {}
        self.plot_canvases = {}
        self.plot_axes = {}
        self.plot_lines = {}
        for spec in self.plot_specs:
            plot_tab = self.QtWidgets.QWidget()
            plot_layout = self.QtWidgets.QVBoxLayout(plot_tab)
            figure = Figure(figsize=(7.0, 4.8), tight_layout=True)
            canvas = FigureCanvas(figure)
            axis = figure.subplots(1, 1)
            self.plot_figures[spec.tab_label] = figure
            self.plot_canvases[spec.tab_label] = canvas
            self.plot_axes[spec.tab_label] = axis
            for line_spec in spec.lines:
                self.plot_lines[line_spec.key] = axis.plot([], [], label=line_spec.label)[0]
            axis.set_title(spec.title)
            axis.set_xlabel(spec.x_label)
            axis.set_ylabel(spec.y_label)
            axis.grid(True, alpha=0.3)
            axis.legend(**DASHBOARD_PLOT_LEGEND_STYLE)
            if spec.equal_aspect:
                axis.axis("equal")
            plot_layout.addWidget(canvas, stretch=1)
            button_row = self.QtWidgets.QHBoxLayout()
            clear_button = self.QtWidgets.QPushButton("清空曲线")
            clear_button.clicked.connect(self.clear_plots)
            save_button = self.QtWidgets.QPushButton("保存当前图")
            save_button.clicked.connect(lambda _checked=False, label=spec.tab_label: self.save_plot_snapshot(label))
            button_row.addWidget(clear_button)
            button_row.addWidget(save_button)
            plot_layout.addLayout(button_row)
            tabs.addTab(plot_tab, spec.tab_label)

    def _key_press_event(self, event: object) -> None:
        if self._handle_key_press(event.key()):
            event.accept()
        else:
            event.ignore()

    def _key_release_event(self, event: object) -> None:
        if self._handle_key_release(event.key()):
            event.accept()
        else:
            event.ignore()

    def _normalize_key(self, key: object) -> int:
        """把 PySide enum/int 统一为 int，保证集合比较稳定。"""
        value = getattr(key, "value", key)
        return int(value)

    def _control_keys(self) -> set[int]:
        """Dashboard 需要接管的驾驶/退出按键。"""
        return {
            self._normalize_key(self.QtCore.Qt.Key_Up),
            self._normalize_key(self.QtCore.Qt.Key_Down),
            self._normalize_key(self.QtCore.Qt.Key_Left),
            self._normalize_key(self.QtCore.Qt.Key_Right),
            self._normalize_key(self.QtCore.Qt.Key_Space),
            self._normalize_key(self.QtCore.Qt.Key_Q),
            self._normalize_key(self.QtCore.Qt.Key_Escape),
        }

    def _handle_key_press(self, key: object) -> bool:
        """处理键盘按下事件；返回 True 表示事件已用于驾驶控制。"""
        key_code = self._normalize_key(key)
        if key_code not in self._control_keys():
            return False
        if key_code in {self._normalize_key(self.QtCore.Qt.Key_Q), self._normalize_key(self.QtCore.Qt.Key_Escape)}:
            self.request_exit()
        else:
            self._pressed_keys.add(key_code)
        return True

    def _handle_key_release(self, key: object) -> bool:
        """处理键盘释放事件；返回 True 表示事件已用于驾驶控制。"""
        key_code = self._normalize_key(key)
        if key_code not in self._control_keys():
            return False
        self._pressed_keys.discard(key_code)
        return True

    def _pulse_button_key(self, key: object) -> None:
        """让一次短点击也至少持续几个仿真帧，避免 press/release 同轮被吃掉。"""
        self._button_pulses[self._normalize_key(key)] = time.monotonic() + DASHBOARD_BUTTON_PULSE_SEC

    def request_exit(self) -> None:
        self._should_exit = True

    def request_reset(self) -> None:
        self._reset_requested = True

    def clear_plots(self) -> None:
        """清空实时曲线数据和画布。"""
        self.plot_buffer.clear()
        self._apply_plot_series()
        for canvas in self.plot_canvases.values():
            canvas.draw_idle()

    def save_plot_snapshot(self, tab_label: str | None = None) -> Path:
        """保存当前曲线页截图到配置的图像目录。"""
        if tab_label is None:
            tab_label = self.tabs.tabText(self.tabs.currentIndex())
        if tab_label not in self.plot_figures:
            tab_label = self.plot_specs[0].tab_label
        self.plot_snapshot_dir.mkdir(parents=True, exist_ok=True)
        safe_label = tab_label.replace("/", "_")
        path = self.plot_snapshot_dir / f"dashboard_plot_{safe_label}_{time.strftime('%Y%m%d_%H%M%S')}.png"
        self.plot_figures[tab_label].savefig(path, dpi=150)
        return path

    def process_events(self) -> None:
        self.app.processEvents()
        if not self.window.isVisible():
            self._should_exit = True

    def current_command(self) -> DashboardCommand:
        now = time.monotonic()
        self._button_pulses = {key: expires_at for key, expires_at in self._button_pulses.items() if expires_at >= now}
        keys = self._pressed_keys | self._button_keys | set(self._button_pulses)
        requested_robot_model = None
        if self.robot_combo is not None:
            requested_robot_model = self.robot_combo.currentData()
        reset_requested = self._reset_requested
        self._reset_requested = False
        if self._normalize_key(self.QtCore.Qt.Key_Space) in keys:
            return DashboardCommand(0.0, 0.0, self._should_exit, requested_robot_model, reset_requested)
        linear = 0.0
        angular = 0.0
        if self._normalize_key(self.QtCore.Qt.Key_Up) in keys:
            linear += self.linear_spin.value()
        if self._normalize_key(self.QtCore.Qt.Key_Down) in keys:
            linear -= self.linear_spin.value()
        if self._normalize_key(self.QtCore.Qt.Key_Left) in keys:
            angular += self.angular_spin.value()
        if self._normalize_key(self.QtCore.Qt.Key_Right) in keys:
            angular -= self.angular_spin.value()
        return DashboardCommand(linear, angular, self._should_exit, requested_robot_model, reset_requested)

    def update(self, telemetry: RobotTelemetry) -> bool:
        """按较低频率刷新显示，并对物理反馈做一阶平滑。"""
        now = time.monotonic()
        plot_updated = self._maybe_update_plots(telemetry, now)
        if not should_refresh_dashboard(self._last_update_time, now, self.update_hz):
            return plot_updated
        display_telemetry = smooth_telemetry(self._smoothed_telemetry, telemetry, self.smoothing_alpha)
        self._smoothed_telemetry = display_telemetry
        self._last_update_time = now
        for name, value in dashboard_rows(display_telemetry):
            self.labels[name].setText(value)
        return True

    def _maybe_update_plots(self, telemetry: RobotTelemetry, now: float) -> bool:
        """按独立频率更新曲线，避免每个物理步都重绘。"""
        if not should_refresh_dashboard(self._last_plot_update_time, now, self.plot_update_hz):
            return False
        self.plot_buffer.append(telemetry)
        self._last_plot_update_time = now
        self._apply_plot_series()
        self._draw_current_plot_canvas()
        return True

    def _draw_current_plot_canvas(self) -> None:
        """只重绘当前可见曲线页，避免数据页也被 Matplotlib 绘图拖慢。"""
        tab_label = self.tabs.tabText(self.tabs.currentIndex())
        canvas = self.plot_canvases.get(tab_label)
        if canvas is not None:
            canvas.draw_idle()

    def _apply_plot_series(self) -> None:
        """把 buffer 中的数据写入 Matplotlib line。"""
        series = self.plot_buffer.series()
        for spec in self.plot_specs:
            for line_spec in spec.lines:
                self.plot_lines[line_spec.key].set_data(series[line_spec.x_field], series[line_spec.y_field])
        for spec in self.plot_specs:
            axis = self.plot_axes[spec.tab_label]
            axis.relim()
            axis.autoscale_view()
            if spec.equal_aspect:
                axis.axis("equal")

    def close(self) -> None:
        self.app.removeEventFilter(self._key_event_filter)
        self.window.close()
        self.app.processEvents()
