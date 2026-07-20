# Dashboard 模块：提供实时遥测侧窗和可测试的数据格式化函数。
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, fields, replace
from pathlib import Path

from slope_sim.model_registry import robot_model_names
from slope_sim.scene import terrain_model_names
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
        "front_left_actual_drive_speed",
        "front_right_actual_drive_speed",
        "rear_left_actual_drive_speed",
        "rear_right_actual_drive_speed",
        "front_left_actual_steering_angle",
        "front_right_actual_steering_angle",
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

DASHBOARD_FIXED_WIDTH = 420
DASHBOARD_AVAILABLE_HEIGHT_RATIO = 0.95
DASHBOARD_TOP_AREA_STRETCH = 45
DASHBOARD_CONTROL_AREA_STRETCH = 55
DASHBOARD_TOP_TABS_MIN_HEIGHT = 320
DASHBOARD_DIRECTION_BUTTON_SIZE = 36
DASHBOARD_CONTROL_SPINBOX_WIDTH = 104
DASHBOARD_PLOT_FIGURE_SIZE = (4.0, 3.2)
DASHBOARD_PLOT_MARGINS = {"left": 0.24, "right": 0.96, "bottom": 0.20, "top": 0.86}
DASHBOARD_COMPACT_HEIGHT_PLOT_MARGINS = {"left": 0.24, "right": 0.96, "bottom": 0.35, "top": 0.69}
DASHBOARD_COMPACT_CONTROL_SCROLL_HEIGHT = 20
DASHBOARD_BUTTON_PULSE_SEC = 0.75
DASHBOARD_MAX_PLOT_DRAW_HZ = 2.0
DASHBOARD_MIN_PLOT_DRAW_COOLDOWN_SEC = 0.5
DASHBOARD_PLOT_DRAW_COOLDOWN_FACTOR = 2.0


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
    """返回固定宽度及当前屏幕可用高度 95% 的 Dashboard 尺寸。"""
    del available_width
    return DASHBOARD_FIXED_WIDTH, max(1, int(available_height * DASHBOARD_AVAILABLE_HEIGHT_RATIO))


def dashboard_top_tabs_min_height(window_height: int, reserved_height: int) -> int:
    """优先保留 320px 顶部区域，小窗口则缩至布局实际可用高度。"""
    return min(DASHBOARD_TOP_TABS_MIN_HEIGHT, max(0, int(window_height) - int(reserved_height)))


def should_refresh_dashboard(last_update_time: float | None, now: float, update_hz: float) -> bool:
    """判断侧窗是否到达下一次刷新时间，避免 240Hz 数字闪烁。"""
    if update_hz <= 0:
        raise ValueError("update_hz must be positive")
    if last_update_time is None:
        return True
    return now - last_update_time >= (1.0 / update_hz) - 1e-9


def plot_draw_cooldown_sec(draw_duration_sec: float, plot_update_hz: float) -> float:
    """根据绘制耗时计算下一次重绘前的冷却时间，避免慢绘图连续占满控制循环。"""
    if draw_duration_sec < 0:
        raise ValueError("draw_duration_sec must be non-negative")
    if plot_update_hz <= 0:
        raise ValueError("plot_update_hz must be positive")
    requested_interval = 1.0 / min(plot_update_hz, DASHBOARD_MAX_PLOT_DRAW_HZ)
    return max(
        DASHBOARD_MIN_PLOT_DRAW_COOLDOWN_SEC,
        requested_interval,
        draw_duration_sec * DASHBOARD_PLOT_DRAW_COOLDOWN_FACTOR,
    )


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
                    "四轮实际驱动速度 FL/FR/RL/RR",
                    f"{_fmt(telemetry.front_left_actual_drive_speed)} / "
                    f"{_fmt(telemetry.front_right_actual_drive_speed)} / "
                    f"{_fmt(telemetry.rear_left_actual_drive_speed)} / "
                    f"{_fmt(telemetry.rear_right_actual_drive_speed)} rad/s",
                ),
                (
                    "前轮实际转角 FL/FR",
                    f"{_fmt(telemetry.front_left_actual_steering_angle)} / "
                    f"{_fmt(telemetry.front_right_actual_steering_angle)} rad",
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
class TerrainSelection:
    """Dashboard 提交给手动仿真循环的一组完整场地参数。"""

    terrain_model: str
    slope_deg: float = 0.0
    golf_seed: int = 0
    golf_relief: str = "medium"

    def __post_init__(self) -> None:
        """规范化选择值，并阻止非法参数进入 PyBullet 场景重建。"""
        terrain_model = self.terrain_model.lower()
        golf_relief = self.golf_relief.lower()
        if terrain_model not in terrain_model_names():
            raise ValueError(f"terrain_model must be one of: {', '.join(terrain_model_names())}")
        if golf_relief not in {"low", "medium", "high"}:
            raise ValueError("golf_relief must be 'low', 'medium', or 'high'")
        if not math.isfinite(float(self.slope_deg)):
            raise ValueError("slope_deg must be finite")
        if not math.isfinite(float(self.golf_seed)) or int(self.golf_seed) != float(self.golf_seed):
            raise ValueError("golf_seed must be a finite integer")
        object.__setattr__(self, "terrain_model", terrain_model)
        object.__setattr__(self, "slope_deg", float(self.slope_deg))
        object.__setattr__(self, "golf_seed", int(self.golf_seed))
        object.__setattr__(self, "golf_relief", golf_relief)


@dataclass(frozen=True)
class DashboardCommand:
    """Dashboard 输出给仿真循环的手动速度和一次性场景命令。"""

    linear_velocity: float
    angular_velocity: float
    should_exit: bool = False
    requested_robot_model: str | None = None
    reset_requested: bool = False
    requested_terrain: TerrainSelection | None = None
    camera_follow_enabled: bool = False
    camera_follow_view: str = "front"


class TelemetryDashboard:
    """PySide6 实时侧边栏；导入 PySide6 延迟到实例化时，避免影响 DIRECT 测试。"""

    def __init__(
        self,
        max_linear_speed: float,
        max_angular_speed: float,
        update_hz: float = 5.0,
        smoothing_alpha: float = 0.35,
        robot_model: str = "df_back",
        model_switch_enabled: bool = False,
        terrain_model: str = "flat",
        slope_deg: float = 0.0,
        golf_seed: int = 0,
        golf_relief: str = "medium",
        terrain_switch_enabled: bool = False,
        plot_update_hz: float = 5.0,
        plot_window_sec: float = 20.0,
        plot_snapshot_dir: str | Path = "results/figures",
        camera_follow_enabled: bool = True,
        camera_follow_view: str = "front",
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
        self._requested_robot_model: str | None = None
        self._requested_terrain: TerrainSelection | None = None
        self._switch_busy = False
        self.update_hz = update_hz
        self.smoothing_alpha = smoothing_alpha
        self.plot_update_hz = plot_update_hz
        self.plot_snapshot_dir = Path(plot_snapshot_dir)
        self.plot_buffer = TelemetryPlotBuffer(plot_window_sec)
        self._last_update_time: float | None = None
        self._last_plot_update_time: float | None = None
        self._smoothed_telemetry: RobotTelemetry | None = None
        self._plot_dirty_tabs: set[str] = set()
        self._plot_next_draw_time: dict[str, float] = {}
        self._plot_draw_started_at: dict[str, float] = {}

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
        self.window.setWindowTitle("Stage 1 Robot Evaluation Dashboard")
        self.window.setFocusPolicy(QtCore.Qt.StrongFocus)
        screen = self.app.primaryScreen()
        if screen is not None:
            available = screen.availableGeometry()
            width, height = dashboard_window_size(available.width(), available.height())
        else:
            width, height = dashboard_window_size(DASHBOARD_FIXED_WIDTH, 700)
        # 固定侧栏尺寸，避免布局提示或用户拖拽改变仿真期间的窗口占用。
        self.window.setFixedSize(width, height)

        layout = QtWidgets.QVBoxLayout(self.window)
        title = QtWidgets.QLabel("实时小车数据反馈")
        title_font = QtGui.QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        layout.addWidget(title)

        tabs = QtWidgets.QTabWidget()
        layout.addWidget(tabs, stretch=DASHBOARD_TOP_AREA_STRETCH)
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
        self.telemetry_scroll = scroll
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
        tabs.currentChanged.connect(self._handle_tab_changed)

        self.control_scroll = QtWidgets.QScrollArea()
        self.control_scroll.setWidgetResizable(True)
        self.control_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.control_scroll.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Preferred,
            QtWidgets.QSizePolicy.Policy.Ignored,
        )
        self.control_scroll.setMinimumHeight(DASHBOARD_COMPACT_CONTROL_SCROLL_HEIGHT)
        self.control_content = QtWidgets.QWidget()
        self.control_layout = QtWidgets.QVBoxLayout(self.control_content)
        self.control_layout.setContentsMargins(4, 4, 4, 4)
        self.control_layout.setSpacing(8)

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

        vehicle_group = QtWidgets.QGroupBox("车型")
        vehicle_layout = QtWidgets.QGridLayout(vehicle_group)
        vehicle_layout.setContentsMargins(8, 6, 8, 6)
        vehicle_layout.setHorizontalSpacing(6)
        vehicle_layout.setVerticalSpacing(4)
        self.robot_combo = None
        self.apply_robot_button = None
        if model_switch_enabled:
            self.robot_combo = QtWidgets.QComboBox()
            for model_name in robot_model_names():
                self.robot_combo.addItem(model_name, model_name)
            self.robot_combo.setFixedWidth(DASHBOARD_CONTROL_SPINBOX_WIDTH)
            index = self.robot_combo.findData(robot_model)
            if index >= 0:
                self.robot_combo.setCurrentIndex(index)
            vehicle_layout.addWidget(self.robot_combo, 0, 0, 1, 2)
            self.apply_robot_button = QtWidgets.QPushButton("应用车型")
            self.apply_robot_button.clicked.connect(self.request_robot_switch)
            vehicle_layout.addWidget(self.apply_robot_button, 1, 0, 1, 2)
        self.reset_button = QtWidgets.QPushButton("复位车辆")
        self.reset_button.clicked.connect(self.request_reset)
        self.reset_button.setFixedHeight(30)
        reset_row = 2 if model_switch_enabled else 0
        vehicle_layout.addWidget(self.reset_button, reset_row, 0, 1, 2)

        self.terrain_combo = None
        self.slope_spin = None
        self.golf_seed_spin = None
        self.golf_relief_combo = None
        self.apply_terrain_button = None
        self.switch_status_label = None
        terrain_group = None
        if terrain_switch_enabled:
            terrain_group = QtWidgets.QGroupBox("场地")
            terrain_layout = QtWidgets.QGridLayout(terrain_group)
            terrain_layout.setContentsMargins(8, 6, 8, 6)
            terrain_layout.setHorizontalSpacing(6)
            terrain_layout.setVerticalSpacing(2)
            self.terrain_combo = QtWidgets.QComboBox()
            for terrain_name in terrain_model_names():
                self.terrain_combo.addItem(terrain_name, terrain_name)
            terrain_index = self.terrain_combo.findData(terrain_model)
            if terrain_index >= 0:
                self.terrain_combo.setCurrentIndex(terrain_index)
            self.terrain_combo.setMinimumWidth(140)

            self.slope_spin = QtWidgets.QDoubleSpinBox()
            self.slope_spin.setRange(-30.0, 30.0)
            self.slope_spin.setSingleStep(1.0)
            self.slope_spin.setSuffix(" deg")
            self.slope_spin.setValue(slope_deg)
            self.slope_spin.setFixedWidth(DASHBOARD_CONTROL_SPINBOX_WIDTH)

            self.golf_seed_spin = QtWidgets.QSpinBox()
            self.golf_seed_spin.setRange(-2_147_483_648, 2_147_483_647)
            self.golf_seed_spin.setValue(golf_seed)
            self.golf_seed_spin.setFixedWidth(DASHBOARD_CONTROL_SPINBOX_WIDTH)

            self.golf_relief_combo = QtWidgets.QComboBox()
            for relief_name in ("low", "medium", "high"):
                self.golf_relief_combo.addItem(relief_name, relief_name)
            relief_index = self.golf_relief_combo.findData(golf_relief)
            if relief_index >= 0:
                self.golf_relief_combo.setCurrentIndex(relief_index)
            self.golf_relief_combo.setFixedWidth(DASHBOARD_CONTROL_SPINBOX_WIDTH)

            terrain_layout.addWidget(QtWidgets.QLabel("场地"), 0, 0)
            terrain_layout.addWidget(self.terrain_combo, 0, 1)
            terrain_layout.addWidget(QtWidgets.QLabel("坡度"), 1, 0)
            terrain_layout.addWidget(self.slope_spin, 1, 1)
            terrain_layout.addWidget(QtWidgets.QLabel("随机种子"), 2, 0)
            terrain_layout.addWidget(self.golf_seed_spin, 2, 1)
            terrain_layout.addWidget(QtWidgets.QLabel("起伏"), 3, 0)
            terrain_layout.addWidget(self.golf_relief_combo, 3, 1)
            self.apply_terrain_button = QtWidgets.QPushButton("应用场地")
            self.apply_terrain_button.clicked.connect(self.request_terrain_switch)
            terrain_layout.addWidget(self.apply_terrain_button, 4, 0, 1, 2)
            self.switch_status_label = QtWidgets.QLabel("切换状态：就绪")
            terrain_layout.addWidget(self.switch_status_label, 5, 0, 1, 2)
            self.terrain_combo.currentIndexChanged.connect(self._update_terrain_parameter_state)
            self._update_terrain_parameter_state()
        elif model_switch_enabled:
            self.switch_status_label = QtWidgets.QLabel("切换状态：就绪")
            vehicle_layout.addWidget(self.switch_status_label, reset_row + 1, 0, 1, 2)

        camera_group = QtWidgets.QGroupBox("相机")
        camera_layout = QtWidgets.QGridLayout(camera_group)
        camera_layout.setContentsMargins(8, 6, 8, 6)
        camera_layout.setHorizontalSpacing(6)
        camera_layout.setVerticalSpacing(4)
        self.camera_follow_checkbox = QtWidgets.QCheckBox("启用跟随")
        self.camera_follow_checkbox.setChecked(camera_follow_enabled)
        self.camera_view_combo = QtWidgets.QComboBox()
        for view_label, view_name in (("车后", "front"), ("侧面", "side"), ("固定", "custom")):
            self.camera_view_combo.addItem(view_label, view_name)
        camera_view_index = self.camera_view_combo.findData(camera_follow_view)
        if camera_view_index >= 0:
            self.camera_view_combo.setCurrentIndex(camera_view_index)
        self.camera_view_combo.setFixedWidth(DASHBOARD_CONTROL_SPINBOX_WIDTH)
        camera_layout.addWidget(self.camera_follow_checkbox, 0, 0, 1, 2)
        camera_layout.addWidget(QtWidgets.QLabel("视角"), 1, 0)
        camera_layout.addWidget(self.camera_view_combo, 1, 1)
        self.camera_follow_checkbox.toggled.connect(self._update_camera_follow_state)
        self._update_camera_follow_state()

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
        # 控制组按操作顺序纵向排列，仅控制区滚动，顶部遥测与曲线保持固定可见。
        self.control_groups = [parameter_group, vehicle_group]
        if terrain_group is not None:
            self.control_groups.append(terrain_group)
        self.control_groups.extend((camera_group, control_group))
        for group in self.control_groups:
            self.control_layout.addWidget(group)
        self.control_layout.addStretch(1)
        self.control_scroll.setWidget(self.control_content)
        layout.addWidget(self.control_scroll, stretch=DASHBOARD_CONTROL_AREA_STRETCH)

        # 先为标题和控制区保留 Qt 的真实最小空间，再给顶部 tabs 分配优先最小高。
        margins = layout.contentsMargins()
        reserved_height = (
            margins.top()
            + margins.bottom()
            + title.sizeHint().height()
            + max(0, layout.spacing()) * 2
            + self.control_scroll.minimumHeight()
        )
        tabs_min_height = dashboard_top_tabs_min_height(height, reserved_height)
        tabs.setMinimumHeight(tabs_min_height)
        if tabs_min_height < DASHBOARD_TOP_TABS_MIN_HEIGHT:
            # 紧凑高度增加上下留白比例，让短画布中的标题、轴标签和图例仍完整。
            for figure in self.plot_figures.values():
                figure.subplots_adjust(**DASHBOARD_COMPACT_HEIGHT_PLOT_MARGINS)

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
            figure = Figure(figsize=DASHBOARD_PLOT_FIGURE_SIZE)
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
                # 只在创建时设置等比例，避免每个遥测样本都重新调整坐标轴。
                axis.set_aspect("equal", adjustable="datalim")
            figure.subplots_adjust(**DASHBOARD_PLOT_MARGINS)
            canvas.mpl_connect("draw_event", lambda _event, label=spec.tab_label: self._record_plot_draw(label))
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
            self._plot_next_draw_time[spec.tab_label] = 0.0

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
        if self._switch_busy:
            return
        self._reset_requested = True

    def request_robot_switch(self) -> None:
        """把下拉框中的车型保存为一次性应用请求。"""
        if self._switch_busy or self.robot_combo is None:
            return
        self._requested_robot_model = str(self.robot_combo.currentData())
        self.set_switch_busy(True, "等待应用")

    def request_terrain_switch(self) -> None:
        """把当前场地控件值保存为一次性应用请求。"""
        if self._switch_busy or self.terrain_combo is None:
            return
        self._requested_terrain = TerrainSelection(
            terrain_model=str(self.terrain_combo.currentData()),
            slope_deg=self.slope_spin.value(),
            golf_seed=self.golf_seed_spin.value(),
            golf_relief=str(self.golf_relief_combo.currentData()),
        )
        self.set_switch_busy(True, "等待应用")

    def _update_terrain_parameter_state(self, _index: int | None = None) -> None:
        """只启用待应用场地真正使用的参数，减少误操作。"""
        if self.terrain_combo is None:
            return
        terrain_model = str(self.terrain_combo.currentData())
        self.slope_spin.setEnabled(terrain_model == "slope")
        golf_enabled = terrain_model == "golf_heightfield"
        self.golf_seed_spin.setEnabled(golf_enabled)
        self.golf_relief_combo.setEnabled(golf_enabled)

    def _update_camera_follow_state(self, _checked: bool | None = None) -> None:
        """跟随关闭时禁用视角选择，避免编辑不生效的控件。"""
        self.camera_view_combo.setEnabled(self.camera_follow_checkbox.isChecked())

    def sync_active_selection(self, robot_model: str, terrain: TerrainSelection) -> None:
        """在切换成功或回滚后，让控件重新反映真实活动世界。"""
        if self.robot_combo is not None:
            index = self.robot_combo.findData(robot_model)
            if index >= 0:
                self.robot_combo.setCurrentIndex(index)
        if self.terrain_combo is None:
            return
        index = self.terrain_combo.findData(terrain.terrain_model)
        if index >= 0:
            self.terrain_combo.setCurrentIndex(index)
        self.slope_spin.setValue(terrain.slope_deg)
        self.golf_seed_spin.setValue(terrain.golf_seed)
        relief_index = self.golf_relief_combo.findData(terrain.golf_relief)
        if relief_index >= 0:
            self.golf_relief_combo.setCurrentIndex(relief_index)
        self._update_terrain_parameter_state()

    def show_switch_status(self, message: str, *, is_error: bool = False) -> None:
        """显示最近一次切换结果；错误使用醒目颜色但不弹阻塞窗口。"""
        self.set_switch_busy(False, message, is_error=is_error)

    def set_switch_busy(self, busy: bool, message: str, *, is_error: bool = False) -> None:
        """切换事务期间禁用会产生重建请求的按钮，完成后统一恢复。"""
        self._switch_busy = busy
        for button in (self.apply_robot_button, self.apply_terrain_button, self.reset_button):
            if button is not None:
                button.setEnabled(not busy)
        if self.switch_status_label is None:
            return
        self.switch_status_label.setText(f"切换状态：{message}")
        self.switch_status_label.setStyleSheet("color: #b00020;" if is_error else "")

    def _active_plot_label(self) -> str | None:
        """返回当前可见的曲线标签；数据页不参与绘图。"""
        if not hasattr(self, "tabs"):
            return None
        label = self.tabs.tabText(self.tabs.currentIndex())
        return label if label in self.plot_canvases else None

    def _handle_tab_changed(self, _index: int) -> None:
        """切换曲线页时只标记当前页，避免一次切换触发所有图重绘。"""
        label = self._active_plot_label()
        if label is None:
            return
        self._plot_dirty_tabs.add(label)

    def _record_plot_draw(self, tab_label: str) -> None:
        """记录一次实际绘制耗时，并为慢速绘图增加冷却时间。"""
        started_at = self._plot_draw_started_at.pop(tab_label, None)
        if started_at is None:
            return
        completed_at = time.monotonic()
        duration = max(0.0, completed_at - started_at)
        next_allowed = completed_at + plot_draw_cooldown_sec(duration, self.plot_update_hz)
        self._plot_next_draw_time[tab_label] = max(self._plot_next_draw_time.get(tab_label, 0.0), next_allowed)

    def clear_plots(self) -> None:
        """清空实时曲线数据和画布。"""
        self.plot_buffer.clear()
        self._plot_dirty_tabs.update(self.plot_canvases)
        label = self._active_plot_label()
        if label is not None:
            self._request_current_plot_draw(time.monotonic(), force=True)

    def reset_feedback_history(self) -> None:
        """车辆重载后清空旧车的平滑状态、刷新节流和曲线数据。"""
        self._smoothed_telemetry = None
        self._last_update_time = None
        self._last_plot_update_time = None
        self._plot_next_draw_time.clear()
        self._plot_draw_started_at.clear()
        self.clear_plots()

    def save_plot_snapshot(self, tab_label: str | None = None) -> Path:
        """保存当前曲线页截图到配置的图像目录。"""
        if tab_label is None:
            tab_label = self.tabs.tabText(self.tabs.currentIndex())
        if tab_label not in self.plot_figures:
            tab_label = self.plot_specs[0].tab_label
        self._apply_plot_series(tab_label)
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
        requested_robot_model = self._requested_robot_model
        requested_terrain = self._requested_terrain
        reset_requested = self._reset_requested
        camera_follow_enabled = self.camera_follow_checkbox.isChecked()
        camera_follow_view = str(self.camera_view_combo.currentData())
        self._requested_robot_model = None
        self._requested_terrain = None
        self._reset_requested = False
        if self._normalize_key(self.QtCore.Qt.Key_Space) in keys:
            return DashboardCommand(
                0.0,
                0.0,
                should_exit=self._should_exit,
                requested_robot_model=requested_robot_model,
                reset_requested=reset_requested,
                requested_terrain=requested_terrain,
                camera_follow_enabled=camera_follow_enabled,
                camera_follow_view=camera_follow_view,
            )
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
        return DashboardCommand(
            linear,
            angular,
            should_exit=self._should_exit,
            requested_robot_model=requested_robot_model,
            reset_requested=reset_requested,
            requested_terrain=requested_terrain,
            camera_follow_enabled=camera_follow_enabled,
            camera_follow_view=camera_follow_view,
        )

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
            return self._request_current_plot_draw(now)
        self.plot_buffer.append(telemetry)
        self._last_plot_update_time = now
        self._plot_dirty_tabs.update(self.plot_canvases)
        self._request_current_plot_draw(now)
        return True

    def _request_current_plot_draw(self, now: float, *, force: bool = False) -> bool:
        """按可见页、绘制冷却和 Qt pending 状态合并重绘请求。"""
        tab_label = self._active_plot_label()
        if tab_label is None or tab_label not in self._plot_dirty_tabs:
            return False
        if not force and now < self._plot_next_draw_time.get(tab_label, 0.0):
            return False
        canvas = self.plot_canvases[tab_label]
        # QtAgg 自己会合并 pending 请求；这里提前丢弃，避免不断刷新绘图队列。
        if getattr(canvas, "_draw_pending", False) or getattr(canvas, "_is_drawing", False):
            return False
        self._apply_plot_series(tab_label)
        self._plot_draw_started_at[tab_label] = now
        self._plot_next_draw_time[tab_label] = now + plot_draw_cooldown_sec(0.0, self.plot_update_hz)
        self._plot_dirty_tabs.discard(tab_label)
        canvas.draw_idle()
        return True

    def _apply_plot_series(self, tab_label: str | None = None) -> None:
        """只把 buffer 数据写入指定的可见曲线页。"""
        tab_label = tab_label or self._active_plot_label()
        if tab_label is None:
            return
        spec = next(spec for spec in self.plot_specs if spec.tab_label == tab_label)
        series = self.plot_buffer.series()
        for line_spec in spec.lines:
            self.plot_lines[line_spec.key].set_data(series[line_spec.x_field], series[line_spec.y_field])
        axis = self.plot_axes[spec.tab_label]
        axis.relim()
        axis.autoscale_view()

    def close(self) -> None:
        self.app.removeEventFilter(self._key_event_filter)
        self.window.close()
        self.app.processEvents()
