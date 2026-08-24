# Dashboard 模块：提供实时遥测侧窗和可测试的数据格式化函数。
from __future__ import annotations

import json
import math
import os
import time
from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass, fields, replace
from functools import lru_cache
from pathlib import Path

from slope_sim.dashboard_charts import (
    DASHBOARD_PLOT_LEGEND_STYLE,
    DashboardPlotLine,
    DashboardPlotSpec,
    InterfaceChartBuffer,
    TelemetryPlotBuffer,
    dashboard_plot_specs,
    interface_chart_specs,
)
from slope_sim.interfaces.config import ChannelConfig, InterfaceConfig
from slope_sim.interfaces.dashboard_snapshot import InterfaceDashboardSnapshot
from slope_sim.interfaces.models import LidarPointCloud
from slope_sim.interfaces.status import InterfaceStatusSnapshot
from slope_sim.manual_capture import ManualCaptureAction, ManualCaptureUiState
from slope_sim.model_registry import get_robot_model, robot_model_names
from slope_sim.obstacles import (
    OBSTACLE_MODES,
    OBSTACLE_SHAPES,
    ObstacleGenerationRequest,
    ObstacleSnapshot,
)
from slope_sim.pointcloud_export import export_lidar_point_clouds
from slope_sim.resource_monitor import ResourceSnapshot
from slope_sim.runtime_actions import (
    AddObstaclesAction,
    ClearObstaclesAction,
    DeleteObstacleAction,
    ResetRobotAction,
    RuntimeAction,
    SwitchRobotAction,
    SwitchTerrainAction,
    TerrainSelection,
)
from slope_sim.scene import terrain_model_names
from slope_sim.telemetry import RobotTelemetry
from slope_sim.window_layout import (
    DisplayMetrics,
    DASHBOARD_WIDTH_RATIO,
    FrameExtents,
    Rect,
    WindowLayoutError,
    logical_client_rect_for_outer,
    wait_for_x11_outer_geometry,
    x11_window_manager_available,
)


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

DASHBOARD_WINDOW_TITLE = "3D仿真Dashboard"
DASHBOARD_LAYOUT_REPORT_ENV = "SLOPE_SIM_DASHBOARD_LAYOUT_REPORT_PATH"
DASHBOARD_LAYOUT_REPORT_VERSION = 4
DASHBOARD_CONTENT_TAB_LABELS = frozenset({"接口状态", "障碍物", "v2 eCAL"})
DASHBOARD_REQUIRED_CRITICAL_CONTROLS = (
    "暂停",
    "复位车辆",
    "线速度",
    "角速度",
    "退出",
    "车型",
    "应用车型",
    "场地",
    "坡度",
    "场地随机种子",
    "起伏",
    "应用场地",
    "障碍模式",
    "障碍形状",
    "障碍数量",
    "障碍随机种子",
    "障碍速度",
    "障碍移动占比",
    "添加障碍",
    "删除选中",
    "清空障碍",
    "结构状态",
)
DASHBOARD_DEFAULT_WIDTH_RATIO = DASHBOARD_WIDTH_RATIO
DASHBOARD_TOP_AREA_STRETCH = 50
DASHBOARD_CONTROL_AREA_STRETCH = 50
DASHBOARD_TOP_TABS_MIN_HEIGHT = 320
DASHBOARD_CONTROL_SPINBOX_WIDTH = 104
DASHBOARD_PLOT_FIGURE_SIZE = (4.0, 3.2)
DASHBOARD_PLOT_MARGINS = {"left": 0.26, "right": 0.96, "bottom": 0.20, "top": 0.86}
DASHBOARD_COMPACT_HEIGHT_PLOT_MARGINS = {"left": 0.24, "right": 0.96, "bottom": 0.35, "top": 0.69}
DASHBOARD_VERY_COMPACT_PLOT_MARGINS = {"left": 0.24, "right": 0.96, "bottom": 0.33, "top": 0.68}
DASHBOARD_COMPACT_CONTROL_SCROLL_HEIGHT = 20
DASHBOARD_VERY_COMPACT_SCROLL_HEIGHT = 10
DASHBOARD_VERY_COMPACT_PLOT_BUTTON_HEIGHT = 22
DASHBOARD_MAX_PLOT_DRAW_HZ = 2.0
DASHBOARD_MIN_PLOT_DRAW_COOLDOWN_SEC = 0.5
DASHBOARD_PLOT_DRAW_COOLDOWN_FACTOR = 2.0
DASHBOARD_LIDAR_X_LIMITS = (-48.0, 48.0)
DASHBOARD_LIDAR_Y_LIMITS = (-30.0, 30.0)
DASHBOARD_LIDAR_TAG_COLORS = ("#7a7f85", "#2f8f46", "#2878b5", "#d1495b")
DASHBOARD_LIDAR_TAG_LABELS = (
    "unknown",
    "terrain",
    "static obstacle",
    "moving obstacle",
)
DASHBOARD_CRITICAL_CONTROL_SCROLL_MARGIN = 8
DASHBOARD_MIN_FULL_WIDTH_PLOT = 300
DASHBOARD_NARROW_PLOT_LEFT_MARGIN = 0.28

INTERFACE_STATE_TEXT = {
    "active": "活动",
    "waiting_peer": "等待对端",
    "waiting_command": "等待命令",
    "invalid_command": "命令无效",
    "timed_out": "超时",
    "degraded": "降级",
    "disconnected": "未连接",
    "error": "错误",
}

INTERFACE_STATE_COLORS = {
    "active": "#147d3f",
    "waiting_peer": "#8a5b00",
    "waiting_command": "#8a5b00",
    "invalid_command": "#b42318",
    "timed_out": "#b42318",
    "degraded": "#9a3412",
    "disconnected": "#5f6368",
    "error": "#b42318",
}


@lru_cache(maxsize=1)
def _dashboard_cjk_font() -> object | None:
    """选择系统现有中文字体，避免 Matplotlib 画布显示方框或反复告警。"""
    from matplotlib import font_manager

    candidates = sorted(
        path
        for path in font_manager.findSystemFonts()
        if "NotoSansCJK-Regular" in path
    )
    if not candidates:
        return None
    return font_manager.FontProperties(fname=candidates[0])


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
    """返回屏幕可用区域右侧 33% 的默认 Dashboard 尺寸。"""
    width = (
        int(available_width) * DASHBOARD_DEFAULT_WIDTH_RATIO.numerator
        + DASHBOARD_DEFAULT_WIDTH_RATIO.denominator // 2
    ) // DASHBOARD_DEFAULT_WIDTH_RATIO.denominator
    return max(1, width), max(1, int(available_height))


def _interface_state_text(state: str) -> str:
    """把接口层稳定状态映射为企业界面中文。"""
    return INTERFACE_STATE_TEXT.get(state, f"未知状态（{state}）")


def _transport_mode_text(mode: str) -> str:
    """显示实际传输模式，不把本地测试误报为 eCAL。"""
    return {
        "auto": "自动选择模式",
        "ecal": "eCAL 正式模式",
        "local": "本地测试模式",
    }.get(mode, f"未知模式（{mode}）")


def _format_frequency(value: float) -> str:
    return f"{value:.1f} Hz"


def _format_timestamp(value: int | None) -> str:
    return "--" if value is None else str(value)


def _format_array(values: tuple[float, ...], unit: str) -> str:
    return f"[{', '.join(f'{value:.2f}' for value in values)}] {unit}"


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


DASHBOARD_VERY_COMPACT_LEGEND_FONT_SIZE = 6
DASHBOARD_VERY_COMPACT_AXIS_LABEL_FONT_SIZE = 6
DASHBOARD_VERY_COMPACT_TICK_FONT_SIZE = 6


def wait_for_dashboard_frame_extents(
    *,
    frame_extents_getter: Callable[[], FrameExtents | None],
    process_events: Callable[[], None],
    window_manager_expected: bool,
    timeout_sec: float = 1.0,
    poll_interval_sec: float = 0.02,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> FrameExtents:
    """有 WM 时等待 Qt 标题栏边距连续两次稳定，无 WM 时返回零。"""
    if not window_manager_expected:
        return FrameExtents(0, 0, 0, 0)
    deadline = clock() + timeout_sec
    previous: FrameExtents | None = None
    while True:
        process_events()
        current = frame_extents_getter()
        if current is None:
            previous = None
            remaining = deadline - clock()
            if remaining <= 0.0:
                raise WindowLayoutError("Dashboard frame extents did not stabilize")
            sleeper(min(poll_interval_sec, remaining))
            continue
        if not isinstance(current, FrameExtents):
            raise TypeError("frame_extents_getter must return FrameExtents or None")
        if current == previous and any(
            (current.left, current.right, current.top, current.bottom)
        ):
            return current
        previous = current if any(
            (current.left, current.right, current.top, current.bottom)
        ) else None
        remaining = deadline - clock()
        if remaining <= 0.0:
            raise WindowLayoutError("Dashboard frame extents did not stabilize")
        sleeper(min(poll_interval_sec, remaining))


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
class DashboardCommand:
    """Dashboard 输出给仿真循环的手动速度和一次性场景命令。"""

    linear_velocity: float
    angular_velocity: float
    should_exit: bool = False
    structural_action: RuntimeAction | None = None
    camera_follow_enabled: bool = False
    camera_follow_view: str = "front"
    paused: bool = False
    control_source: str = "keyboard"


@dataclass
class InterfaceStatusRow:
    """一个企业话题的 Qt 标签集合，便于只读快照逐字段更新。"""

    container: object
    name_label: object
    state_label: object
    target_label: object
    actual_label: object
    timestamp_label: object
    error_label: object
    drop_label: object
    detail_label: object
    command_state_label: object | None = None
    command_frequency_label: object | None = None
    command_timestamp_label: object | None = None
    drive_values_label: object | None = None
    steering_values_label: object | None = None

    @property
    def frequency_label(self) -> object:
        """兼容 Task13 计划中的实际频率属性名。"""
        return self.actual_label

    @property
    def target_frequency_label(self) -> object:
        return self.target_label

    @property
    def actual_frequency_label(self) -> object:
        return self.actual_label

    @property
    def error_count_label(self) -> object:
        return self.error_label

    @property
    def dropped_count_label(self) -> object:
        return self.drop_label

    @property
    def value_labels(self) -> tuple[object, ...]:
        """返回需要在窄宽下换行的全部值标签。"""
        optional = (
            self.command_state_label,
            self.command_frequency_label,
            self.command_timestamp_label,
            self.drive_values_label,
            self.steering_values_label,
        )
        return (
            self.state_label,
            self.target_label,
            self.actual_label,
            self.timestamp_label,
            self.error_label,
            self.drop_label,
            self.detail_label,
            *(label for label in optional if label is not None),
        )


class TelemetryDashboard:
    """PySide6 实时侧边栏；导入 PySide6 延迟到实例化时，避免影响 DIRECT 测试。"""

    def __init__(
        self,
        max_linear_speed: float = 0.4,
        max_angular_speed: float = 0.8,
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
        interface_config: InterfaceConfig | None = None,
        interface_enabled: bool = False,
        developer_diagnostics_enabled: bool = False,
        show_lidar_tools: bool = True,
        v2_dashboard_enabled: bool = False,
    ) -> None:
        """创建企业接口侧窗；内部诊断仅在显式开启时构建。"""
        from PySide6 import QtCore, QtGui, QtWidgets

        if interface_config is None:
            interface_config = InterfaceConfig.default()
        if not isinstance(interface_config, InterfaceConfig):
            raise ValueError("interface_config must be an InterfaceConfig")
        if not isinstance(developer_diagnostics_enabled, bool):
            raise ValueError("developer_diagnostics_enabled must be a bool")
        if not isinstance(interface_enabled, bool):
            raise ValueError("interface_enabled must be a bool")
        if not isinstance(show_lidar_tools, bool):
            raise ValueError("show_lidar_tools must be a bool")
        if not isinstance(v2_dashboard_enabled, bool):
            raise ValueError("v2_dashboard_enabled must be a bool")
        if v2_dashboard_enabled and show_lidar_tools:
            raise ValueError("v2 dashboard requires legacy lidar tools to be disabled")

        self.QtCore = QtCore
        self.QtGui = QtGui
        self.QtWidgets = QtWidgets
        self.interface_config = interface_config
        self.interface_enabled = interface_enabled
        self.developer_diagnostics_enabled = developer_diagnostics_enabled
        self.show_lidar_tools = show_lidar_tools
        self.v2_dashboard_enabled = v2_dashboard_enabled
        self._model_switch_enabled = bool(model_switch_enabled)
        self._terrain_switch_enabled = bool(terrain_switch_enabled)
        self._paused = False
        self._camera_follow_enabled = bool(camera_follow_enabled)
        self._camera_follow_view = str(camera_follow_view)
        self._pressed_keys: set[int] = set()
        self._should_exit = False
        self._pending_actions: deque[RuntimeAction] = deque()
        self._pending_capture_actions: deque[ManualCaptureAction] = deque()
        self._capture_state = ManualCaptureUiState.IDLE
        self._capture_start_pending = False
        self._capture_mcap_path: Path | None = None
        self._capture_lvx2_path: Path | None = None
        self._capture_compressed_path: Path | None = None
        self._capture_compression_pending = False
        self._structure_busy = False
        self._switch_busy = False
        self._last_obstacle_refresh_time: float | None = None
        self.update_hz = update_hz
        self.smoothing_alpha = smoothing_alpha
        self.plot_update_hz = plot_update_hz
        self.plot_snapshot_dir = Path(plot_snapshot_dir)
        self.pointcloud_export_dir = self.plot_snapshot_dir.parent / "pointcloud"
        self.plot_buffer = TelemetryPlotBuffer(plot_window_sec)
        self.interface_plot_buffer = InterfaceChartBuffer(plot_window_sec, interface_config)
        self._interface_generation: int | None = None
        self._interface_robot_model = robot_model
        self._latest_lidar_views: dict[str, object | None] = {"front": None, "rear": None}
        self._latest_lidar_clouds: dict[str, LidarPointCloud | None] = {"front": None, "rear": None}
        self._lidar_view_enabled = True
        self._last_update_time: float | None = None
        self._actual_update_hz: float | None = None
        self._last_plot_update_time: float | None = None
        self._smoothed_telemetry: RobotTelemetry | None = None
        self._interface_status: InterfaceStatusSnapshot | None = None
        self._latest_interface_snapshot: InterfaceDashboardSnapshot | None = None
        self._last_interface_status_update_time: float | None = None
        report_path = os.environ.get(DASHBOARD_LAYOUT_REPORT_ENV)
        self._layout_report_path = Path(report_path) if report_path else None
        self._last_layout_report_tab_index: int | None = None
        self._last_layout_report_revisions: dict[int, int] = {}
        self._plot_dirty_tabs: set[str] = set()
        self._plot_data_revisions: dict[str, int] = {}
        self._plot_rendered_revisions: dict[str, int] = {}
        self._plot_next_draw_time: dict[str, float] = {}
        self._plot_draw_started_at: dict[str, float] = {}
        self.labels: dict[str, object] = {}
        self.plot_specs: list[object] = []
        self.plot_figures: dict[str, object] = {}
        self.plot_canvases: dict[str, object] = {}
        self.plot_axes: dict[str, object] = {}
        self.plot_lines: dict[str, object] = {}
        self.plot_legends: dict[str, object] = {}
        self.plot_texts: dict[str, object] = {}
        self.no_steering_texts: dict[str, object] = {}
        self.plot_layouts: dict[str, object] = {}
        self.plot_buttons: list[object] = []
        self.plot_actions: dict[str, object] = {}
        self.plot_selector_button = None
        self.plot_selector_menu = None
        self._plot_spec_by_label: dict[str, object] = {}
        self.interface_plot_specs = interface_chart_specs(get_robot_model(robot_model))
        self._interface_line_keys: set[str] = set()
        self.lidar_collection = None
        self.lidar_vehicle_marker = None
        self.lidar_range_ring = None
        self.lidar_front_time_text = None
        self.lidar_rear_time_text = None
        self.lidar_view_checkbox = None
        self.lidar_status_label = None
        self.lidar_open_button = None
        self.lidar_export_button = None
        self.developer_layout = None
        self.diagnostic_tabs = None
        self.telemetry_scroll = None
        self.diagnostic_control_scroll = None
        self.diagnostic_control_content = None
        self.diagnostic_control_groups: list[object] = []
        self.resource_labels: dict[str, object] = {}
        self.resource_layout = None
        self.linear_spin = None
        self.angular_spin = None
        self.control_source_combo = None
        self.control_owner_label = None
        self.rc_status_label = None
        self.camera_follow_checkbox = None
        self.camera_view_combo = None
        self.direction_buttons: list[object] = []
        self.capture_duration_combo = None
        self.capture_start_button = None
        self.capture_stop_button = None
        self.capture_viewer_button = None
        self.capture_compress_button = None
        self.capture_status_label = None
        self.capture_path_label = None
        self.v2_dashboard_widget = None

        self.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        dashboard = self

        class DashboardKeyEventFilter(QtCore.QObject):
            """捕获子控件焦点下的方向键，避免 spinbox/tab 吃掉驾驶按键。"""

            def eventFilter(self, _watched: object, event: object) -> bool:
                if event.type() == QtCore.QEvent.KeyPress:
                    return dashboard._handle_tab_cycle(event) or dashboard._handle_key_press(event.key())
                if event.type() == QtCore.QEvent.KeyRelease:
                    return dashboard._handle_key_release(event.key())
                return False

        class DashboardSpinBoxWheelFilter(QtCore.QObject):
            """阻止滚轮掠过参数框时无意修改数值，保留键盘和箭头按钮操作。"""

            def eventFilter(self, _watched: object, event: object) -> bool:
                return event.type() == QtCore.QEvent.Wheel

        self._key_event_filter = DashboardKeyEventFilter()
        self._spinbox_wheel_filter = DashboardSpinBoxWheelFilter()
        self._key_event_filter_installed = False
        self._mpl_connections: list[tuple[object, str, int]] = []
        self._disposed = False
        self.window = QtWidgets.QWidget()
        try:
            self._initialize_window(
                max_linear_speed=max_linear_speed,
                max_angular_speed=max_angular_speed,
                robot_model=robot_model,
                terrain_model=terrain_model,
                slope_deg=slope_deg,
                golf_seed=golf_seed,
                golf_relief=golf_relief,
                camera_follow_enabled=camera_follow_enabled,
                camera_follow_view=camera_follow_view,
                developer_diagnostics_enabled=developer_diagnostics_enabled,
            )
        except BaseException:
            self._dispose()
            raise

    def _initialize_window(
        self,
        *,
        max_linear_speed: float,
        max_angular_speed: float,
        robot_model: str,
        terrain_model: str,
        slope_deg: float,
        golf_seed: int,
        golf_relief: str,
        camera_follow_enabled: bool,
        camera_follow_view: str,
        developer_diagnostics_enabled: bool,
    ) -> None:
        """完成窗口构造；任一步异常都由调用方统一释放已创建资源。"""
        QtCore = self.QtCore
        QtGui = self.QtGui
        QtWidgets = self.QtWidgets
        self.window.setWindowTitle(DASHBOARD_WINDOW_TITLE)
        self.window.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.window.setStyleSheet(
            "QGroupBox { border: 1px solid #c9ced3; border-radius: 6px; "
            "margin-top: 10px; padding-top: 7px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 3px; }"
            "QPushButton { min-height: 26px; border-radius: 4px; padding: 2px 6px; }"
            "QComboBox, QSpinBox, QDoubleSpinBox { min-height: 24px; }"
        )

        self.window_layout = QtWidgets.QVBoxLayout(self.window)
        self.window_layout.setContentsMargins(8, 8, 8, 8)
        self.window_layout.setSpacing(6)
        self.title_label = QtWidgets.QLabel(DASHBOARD_WINDOW_TITLE)
        title_font = QtGui.QFont()
        title_font.setPointSize(15)
        title_font.setBold(True)
        self.title_label.setFont(title_font)
        self.title_label.setWordWrap(True)
        self.window_layout.addWidget(self.title_label)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setUsesScrollButtons(True)
        self._add_interface_status_tab(self.tabs)
        self._add_obstacle_tab(self.tabs)
        self._add_plot_tabs(self.tabs)
        if developer_diagnostics_enabled:
            self._add_developer_diagnostics_tab(
                self.tabs,
                camera_follow_enabled=camera_follow_enabled,
                camera_follow_view=camera_follow_view,
            )
        self.tabs.currentChanged.connect(self._handle_tab_changed)
        self.window_layout.addWidget(self.tabs, stretch=DASHBOARD_TOP_AREA_STRETCH)

        self._add_enterprise_control_area(
            max_linear_speed=max_linear_speed,
            max_angular_speed=max_angular_speed,
            robot_model=robot_model,
            terrain_model=terrain_model,
            slope_deg=slope_deg,
            golf_seed=golf_seed,
            golf_relief=golf_relief,
        )
        if not self.interface_enabled and not self.v2_dashboard_enabled:
            self._show_realtime_interface_inactive()
        self.window_layout.addWidget(self.control_scroll, stretch=DASHBOARD_CONTROL_AREA_STRETCH)
        self._disable_spinbox_wheel_adjustment()

        screen = self.app.primaryScreen()
        if screen is not None:
            available = screen.availableGeometry()
            width, height = dashboard_window_size(available.width(), available.height())
        else:
            width, height = dashboard_window_size(1366, 768)
        self.window.resize(width, height)
        self._update_layout_for_size(width, height)

        self.window.keyPressEvent = self._key_press_event
        self.window.keyReleaseEvent = self._key_release_event
        self.app.installEventFilter(self._key_event_filter)
        self._key_event_filter_installed = True
        self.window.show()
        self.window.activateWindow()
        self.window.setFocus()

    def _disable_spinbox_wheel_adjustment(self) -> None:
        """所有参数框统一禁用滚轮，避免侧栏滚动时误改仿真配置。"""
        for spinbox in self.window.findChildren(self.QtWidgets.QAbstractSpinBox):
            spinbox.installEventFilter(self._spinbox_wheel_filter)

    def attach_v2_dashboard_widget(self, widget: object) -> None:
        """把正式 v2 状态与 MID-360 工作台分别嵌入对应的顶层页面。"""
        from slope_sim.interfaces.v2.dashboard_adapter import V2DashboardWidget

        if type(widget) is not V2DashboardWidget:
            raise ValueError("widget must be an exact V2DashboardWidget")
        if self.show_lidar_tools:
            raise ValueError("v2 dashboard requires legacy lidar tools to be disabled")
        if not self.v2_dashboard_enabled:
            raise ValueError("v2 dashboard widget requires v2_dashboard_enabled")
        if self.v2_dashboard_widget is not None:
            raise RuntimeError("v2 dashboard widget is already attached")
        take_panels = getattr(widget, "take_dashboard_panels", None)
        if not callable(take_panels):
            raise ValueError("v2 dashboard widget must provide take_dashboard_panels")
        interface_panel, lidar_panel = take_panels()

        def scroll_page(panel: object) -> object:
            page = self.QtWidgets.QWidget(self.tabs)
            layout = self.QtWidgets.QVBoxLayout(page)
            layout.setContentsMargins(0, 0, 0, 0)
            scroll = self.QtWidgets.QScrollArea(page)
            scroll.setWidgetResizable(True)
            scroll.setHorizontalScrollBarPolicy(self.QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
            scroll.setVerticalScrollBarPolicy(self.QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded)
            scroll.setFrameShape(self.QtWidgets.QFrame.Shape.NoFrame)
            scroll.setWidget(panel)
            layout.addWidget(scroll)
            return page

        legacy_index = self.tabs.indexOf(self.interface_status_tab)
        if legacy_index >= 0:
            self.tabs.removeTab(legacy_index)
        interface_index = self.tabs.insertTab(0, scroll_page(interface_panel), "接口状态")
        obstacle_index = next(
            index
            for index in range(self.tabs.count())
            if self.tabs.tabText(index) == "障碍物"
        )
        self.tabs.insertTab(obstacle_index + 1, scroll_page(lidar_panel), "LiDAR点云")
        self.tabs.setCurrentIndex(interface_index)
        self.v2_dashboard_widget = widget

    def _show_realtime_interface_inactive(self) -> None:
        """无通信启动明确标记为“已禁用”，不能伪装为网络故障或等待数据。"""
        self.transport_mode_label.setText("已关闭（流畅手动模式）")
        self.ecal_status_label.setText("已禁用")
        self.transport_detail_label.setText(
            "正常 runSim 为保持流畅已关闭实时接口；MID-360 在结束采集后离线重建。"
        )
        for row in self.interface_rows.values():
            row.state_label.setText("已禁用")
            row.target_label.setText("--")
            row.actual_label.setText("--")
            row.timestamp_label.setText("--")
            row.error_label.setText("0")
            row.drop_label.setText("0")
            row.detail_label.setText("此启动模式不运行实时传感器")

    def _add_interface_status_tab(self, tabs: object) -> None:
        """创建六话题企业状态页，所有值均由不可变快照更新。"""
        interface_tab = self.QtWidgets.QWidget()
        self.interface_status_tab = interface_tab
        interface_layout = self.QtWidgets.QVBoxLayout(interface_tab)
        interface_layout.setContentsMargins(4, 4, 4, 4)

        self.interface_scroll = self.QtWidgets.QScrollArea()
        self.interface_scroll.setWidgetResizable(True)
        self.interface_scroll.setHorizontalScrollBarPolicy(self.QtCore.Qt.ScrollBarAlwaysOff)
        self.interface_content = self.QtWidgets.QWidget()
        self.interface_content.setMinimumWidth(0)
        content_layout = self.QtWidgets.QVBoxLayout(self.interface_content)
        content_layout.setContentsMargins(4, 4, 4, 4)
        content_layout.setSpacing(6)

        overview = self.QtWidgets.QGroupBox("连接概览")
        overview_layout = self.QtWidgets.QGridLayout(overview)
        overview_layout.setColumnStretch(1, 1)
        self.transport_mode_label = self._add_form_value(
            overview_layout,
            0,
            "传输模式",
            _transport_mode_text(self.interface_config.transport_mode),
        )
        self.ecal_status_label = self._add_form_value(overview_layout, 1, "eCAL 连接", "eCAL 未连接")
        self.transport_detail_label = self._add_form_value(overview_layout, 2, "模式详情", "等待接口状态")
        content_layout.addWidget(overview)

        channel_rows = (
            (self.interface_config.wheel_command, "轮子命令"),
            (self.interface_config.wheel_state, "轮子状态"),
            (self.interface_config.lidar_front, "前雷达"),
            (self.interface_config.lidar_rear, "后雷达"),
            (self.interface_config.rtk, "RTK"),
            (self.interface_config.imu, "IMU"),
        )
        self.interface_rows: dict[str, InterfaceStatusRow] = {}
        for channel, display_name in channel_rows:
            row = self._create_interface_row(channel, display_name)
            self.interface_rows[channel.topic] = row
            content_layout.addWidget(row.container)
        content_layout.addStretch(1)

        self.interface_scroll.setWidget(self.interface_content)
        interface_layout.addWidget(self.interface_scroll)
        tabs.addTab(interface_tab, "接口状态")

    def _add_form_value(self, layout: object, row: int, name: str, value: str) -> object:
        """向窄宽友好的两列表单加入名称和值标签。"""
        name_label = self.QtWidgets.QLabel(name)
        name_label.setWordWrap(True)
        value_label = self.QtWidgets.QLabel(value)
        value_label.setWordWrap(True)
        value_label.setMinimumWidth(0)
        value_label.setSizePolicy(
            self.QtWidgets.QSizePolicy.Policy.Ignored,
            self.QtWidgets.QSizePolicy.Policy.Preferred,
        )
        value_label.setTextInteractionFlags(self.QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(name_label, row, 0, alignment=self.QtCore.Qt.AlignTop)
        layout.addWidget(value_label, row, 1)
        return value_label

    def _create_interface_row(self, channel: ChannelConfig, display_name: str) -> InterfaceStatusRow:
        """创建一个话题状态组，并为轮子话题追加企业字段。"""
        group = self.QtWidgets.QGroupBox(display_name)
        group.setSizePolicy(
            self.QtWidgets.QSizePolicy.Policy.Expanding,
            self.QtWidgets.QSizePolicy.Policy.Preferred,
        )
        grid = self.QtWidgets.QGridLayout(group)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(3)
        grid.setColumnStretch(1, 1)

        topic_label = self.QtWidgets.QLabel(channel.topic)
        topic_label.setWordWrap(True)
        topic_label.setMinimumWidth(0)
        topic_label.setStyleSheet("color: #5f6368;")
        grid.addWidget(topic_label, 0, 0, 1, 2)
        state_label = self._add_form_value(grid, 1, "状态", "等待状态")
        target_label = self._add_form_value(grid, 2, "目标频率", _format_frequency(float(channel.rate_hz)))
        actual_label = self._add_form_value(grid, 3, "实际频率", "--")
        timestamp_label = self._add_form_value(grid, 4, "最近时间戳", "--")
        error_label = self._add_form_value(grid, 5, "错误数", "0")
        drop_label = self._add_form_value(grid, 6, "丢帧数", "0")
        detail_label = self._add_form_value(grid, 7, "详情", "--")

        row = InterfaceStatusRow(
            container=group,
            name_label=topic_label,
            state_label=state_label,
            target_label=target_label,
            actual_label=actual_label,
            timestamp_label=timestamp_label,
            error_label=error_label,
            drop_label=drop_label,
            detail_label=detail_label,
        )
        if channel.topic == self.interface_config.wheel_command.topic:
            row.command_state_label = self._add_form_value(grid, 8, "命令状态", "等待命令")
            row.command_frequency_label = self._add_form_value(grid, 9, "有效频率", "0.0 Hz")
            row.command_timestamp_label = self._add_form_value(grid, 10, "最近输入", "--")
        elif channel.topic == self.interface_config.wheel_state.topic:
            row.drive_values_label = self._add_form_value(grid, 8, "实际轮速", "--")
            row.steering_values_label = self._add_form_value(grid, 9, "实际转角", "--")
        return row

    def _add_enterprise_control_area(
        self,
        *,
        max_linear_speed: float,
        max_angular_speed: float,
        robot_model: str,
        terrain_model: str,
        slope_deg: float,
        golf_seed: int,
        golf_relief: str,
    ) -> None:
        """创建企业允许的仿真、机器人、场地和障碍物控制。"""
        self.control_scroll = self.QtWidgets.QScrollArea()
        self.control_scroll.setWidgetResizable(True)
        self.control_scroll.setHorizontalScrollBarPolicy(self.QtCore.Qt.ScrollBarAlwaysOff)
        self.control_scroll.setSizePolicy(
            self.QtWidgets.QSizePolicy.Policy.Preferred,
            self.QtWidgets.QSizePolicy.Policy.Ignored,
        )
        self.control_scroll.setMinimumHeight(DASHBOARD_COMPACT_CONTROL_SCROLL_HEIGHT)
        self.control_content = self.QtWidgets.QWidget()
        self.control_content.setMinimumWidth(0)
        self.control_layout = self.QtWidgets.QVBoxLayout(self.control_content)
        self.control_layout.setContentsMargins(4, 4, 4, 4)
        self.control_layout.setSpacing(6)

        simulation_group = self._create_simulation_group(
            max_linear_speed=max_linear_speed,
            max_angular_speed=max_angular_speed,
        )
        robot_group = self._create_robot_group(robot_model)
        terrain_group = self._create_terrain_group(terrain_model, slope_deg, golf_seed, golf_relief)
        obstacle_group = self._create_obstacle_group()
        capture_group = self._create_capture_group()
        self.control_groups = [simulation_group, robot_group, terrain_group, obstacle_group, capture_group]
        for group in self.control_groups:
            self.control_layout.addWidget(group)
        self.control_layout.addStretch(1)
        self.control_scroll.setWidget(self.control_content)

    def _create_simulation_group(
        self,
        *,
        max_linear_speed: float,
        max_angular_speed: float,
    ) -> object:
        """创建暂停、速度上限、复位、退出和键盘驾驶控制。"""
        group = self.QtWidgets.QGroupBox("仿真控制")
        grid = self.QtWidgets.QGridLayout(group)
        grid.setContentsMargins(8, 7, 8, 7)
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(4)
        grid.setColumnStretch(1, 1)

        grid.addWidget(self.QtWidgets.QLabel("运行状态"), 0, 0)
        self.pause_status_label = self.QtWidgets.QLabel("运行中")
        self.pause_status_label.setWordWrap(True)
        grid.addWidget(self.pause_status_label, 0, 1, 1, 2)
        self.pause_button = self.QtWidgets.QPushButton("暂停")
        self._configure_button(
            self.pause_button,
            "暂停物理步进；接口连接状态继续刷新",
            self.QtWidgets.QStyle.StandardPixmap.SP_MediaPause,
        )
        self.pause_button.clicked.connect(self._toggle_paused)
        grid.addWidget(self.pause_button, 1, 0, 1, 3)

        self.reset_button = self.QtWidgets.QPushButton("复位车辆")
        self._configure_button(
            self.reset_button,
            "复位当前车辆",
            self.QtWidgets.QStyle.StandardPixmap.SP_BrowserReload,
        )
        self.reset_button.clicked.connect(self.request_reset)
        grid.addWidget(self.reset_button, 2, 0, 1, 3)

        linear_limit = 2.0
        angular_limit = 4.0
        if self.v2_dashboard_enabled:
            from slope_sim.interfaces.v2.runsim_session import (
                MAX_ANGULAR_VELOCITY_RAD_S,
                MAX_LINEAR_VELOCITY_M_S,
            )

            linear_limit = MAX_LINEAR_VELOCITY_M_S
            angular_limit = MAX_ANGULAR_VELOCITY_RAD_S
        self.linear_spin = self.QtWidgets.QDoubleSpinBox()
        self.linear_spin.setRange(0.0, linear_limit)
        self.linear_spin.setSingleStep(0.05)
        self.linear_spin.setSuffix(" m/s")
        self.linear_spin.setValue(max_linear_speed)
        self.linear_spin.setMinimumWidth(DASHBOARD_CONTROL_SPINBOX_WIDTH)
        self.angular_spin = self.QtWidgets.QDoubleSpinBox()
        self.angular_spin.setRange(0.0, angular_limit)
        self.angular_spin.setSingleStep(0.05)
        self.angular_spin.setSuffix(" rad/s")
        self.angular_spin.setValue(max_angular_speed)
        self.angular_spin.setMinimumWidth(DASHBOARD_CONTROL_SPINBOX_WIDTH)
        grid.addWidget(self.QtWidgets.QLabel("线速度"), 3, 0)
        grid.addWidget(self.linear_spin, 3, 1, 1, 2)
        grid.addWidget(self.QtWidgets.QLabel("角速度"), 4, 0)
        grid.addWidget(self.angular_spin, 4, 1, 1, 2)

        self.control_source_combo = self.QtWidgets.QComboBox()
        for label, source in (("键盘", "keyboard"), ("遥控器", "rc"), ("外部命令", "external")):
            self.control_source_combo.addItem(label, source)
        grid.addWidget(self.QtWidgets.QLabel("控制源"), 5, 0)
        grid.addWidget(self.control_source_combo, 5, 1, 1, 2)
        self.control_owner_label = self.QtWidgets.QLabel("当前所有者：键盘（等待接管）")
        self.control_owner_label.setWordWrap(True)
        grid.addWidget(self.control_owner_label, 6, 0, 1, 3)
        self.control_source_combo.currentIndexChanged.connect(
            self._control_source_selection_changed
        )
        self.rc_status_label = self.QtWidgets.QLabel("遥控器：未启用")
        self.rc_status_label.setWordWrap(True)
        grid.addWidget(self.rc_status_label, 7, 0, 1, 3)

        keyboard_hint = self.QtWidgets.QLabel(
            "驾驶：↑↓ 前后，←→ 转向，Space 暂停；退出请用本窗口“退出”按钮或关闭窗口"
        )
        keyboard_hint.setWordWrap(True)
        grid.addWidget(keyboard_hint, 8, 0, 1, 3)
        quit_row = 9
        if self.show_lidar_tools:
            self.lidar_view_checkbox = self.QtWidgets.QCheckBox("显示实时 LiDAR 点云")
            self.lidar_view_checkbox.setChecked(self._lidar_view_enabled)
            self.lidar_view_checkbox.setToolTip("只切换点云显示；传感器采集和接口发布保持运行")
            self.lidar_view_checkbox.toggled.connect(self._set_lidar_view_enabled)
            grid.addWidget(self.lidar_view_checkbox, 9, 0, 1, 3)
            self.lidar_open_button = self.QtWidgets.QPushButton("打开实时点云")
            self._configure_button(
                self.lidar_open_button,
                "打开 LiDAR点云 页面查看前后雷达的实时俯视投影",
                self.QtWidgets.QStyle.StandardPixmap.SP_DesktopIcon,
            )
            self.lidar_open_button.clicked.connect(self.show_lidar_point_cloud)
            grid.addWidget(self.lidar_open_button, 10, 0, 1, 3)
            self.lidar_export_button = self.QtWidgets.QPushButton("导出当前点云")
            self._configure_button(
                self.lidar_export_button,
                "导出当前前后 LiDAR 帧为可导入的 PCD 和 PLY 文件",
                self.QtWidgets.QStyle.StandardPixmap.SP_DialogSaveButton,
            )
            self.lidar_export_button.clicked.connect(self._export_current_lidar_point_clouds)
            grid.addWidget(self.lidar_export_button, 11, 0, 1, 3)
            self.lidar_status_label = self.QtWidgets.QLabel("点云：等待前/后雷达帧")
            self.lidar_status_label.setWordWrap(True)
            self.lidar_status_label.setTextInteractionFlags(
                self.QtCore.Qt.TextSelectableByMouse
            )
            grid.addWidget(self.lidar_status_label, 12, 0, 1, 3)
            quit_row = 13

        self.quit_button = self.QtWidgets.QPushButton("退出")
        self._configure_button(
            self.quit_button,
            "退出仿真程序",
            self.QtWidgets.QStyle.StandardPixmap.SP_DialogCloseButton,
        )
        self.quit_button.clicked.connect(self.request_exit)
        grid.addWidget(self.quit_button, quit_row, 0, 1, 3)
        return group

    def _create_robot_group(self, robot_model: str) -> object:
        group = self.QtWidgets.QGroupBox("机器人")
        layout = self.QtWidgets.QGridLayout(group)
        layout.setContentsMargins(8, 7, 8, 7)
        layout.setColumnStretch(1, 1)
        self.robot_combo = self.QtWidgets.QComboBox()
        for model_name in robot_model_names():
            self.robot_combo.addItem(model_name, model_name)
        index = self.robot_combo.findData(robot_model)
        if index >= 0:
            self.robot_combo.setCurrentIndex(index)
        self.robot_combo.setMinimumWidth(0)
        layout.addWidget(self.QtWidgets.QLabel("车型"), 0, 0)
        layout.addWidget(self.robot_combo, 0, 1)
        self.apply_robot_button = self.QtWidgets.QPushButton("应用车型")
        self._configure_button(
            self.apply_robot_button,
            "应用选中的机器人车型",
            self.QtWidgets.QStyle.StandardPixmap.SP_DialogApplyButton,
        )
        self.apply_robot_button.clicked.connect(self.request_robot_switch)
        layout.addWidget(self.apply_robot_button, 1, 0, 1, 2)
        self.robot_combo.setEnabled(self._model_switch_enabled)
        self.apply_robot_button.setEnabled(self._model_switch_enabled)
        return group

    def _create_terrain_group(
        self,
        terrain_model: str,
        slope_deg: float,
        golf_seed: int,
        golf_relief: str,
    ) -> object:
        group = self.QtWidgets.QGroupBox("场地")
        layout = self.QtWidgets.QGridLayout(group)
        layout.setContentsMargins(8, 7, 8, 7)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(3)
        layout.setColumnStretch(1, 1)

        self.terrain_combo = self.QtWidgets.QComboBox()
        for terrain_name in terrain_model_names():
            self.terrain_combo.addItem(terrain_name, terrain_name)
        terrain_index = self.terrain_combo.findData(terrain_model)
        if terrain_index >= 0:
            self.terrain_combo.setCurrentIndex(terrain_index)
        self.terrain_combo.setMinimumWidth(0)

        self.slope_spin = self.QtWidgets.QDoubleSpinBox()
        self.slope_spin.setRange(-30.0, 30.0)
        self.slope_spin.setSingleStep(1.0)
        self.slope_spin.setSuffix(" deg")
        self.slope_spin.setValue(slope_deg)

        self.golf_seed_spin = self.QtWidgets.QSpinBox()
        self.golf_seed_spin.setRange(-2_147_483_648, 2_147_483_647)
        self.golf_seed_spin.setValue(golf_seed)

        self.golf_relief_combo = self.QtWidgets.QComboBox()
        for relief_name in ("low", "medium", "high"):
            self.golf_relief_combo.addItem(relief_name, relief_name)
        relief_index = self.golf_relief_combo.findData(golf_relief)
        if relief_index >= 0:
            self.golf_relief_combo.setCurrentIndex(relief_index)

        layout.addWidget(self.QtWidgets.QLabel("场地"), 0, 0)
        layout.addWidget(self.terrain_combo, 0, 1)
        layout.addWidget(self.QtWidgets.QLabel("坡度"), 1, 0)
        layout.addWidget(self.slope_spin, 1, 1)
        layout.addWidget(self.QtWidgets.QLabel("随机种子"), 2, 0)
        layout.addWidget(self.golf_seed_spin, 2, 1)
        layout.addWidget(self.QtWidgets.QLabel("起伏"), 3, 0)
        layout.addWidget(self.golf_relief_combo, 3, 1)

        self.apply_terrain_button = self.QtWidgets.QPushButton("应用场地")
        self._configure_button(
            self.apply_terrain_button,
            "应用场地参数；高尔夫场地会重新生成",
            self.QtWidgets.QStyle.StandardPixmap.SP_DialogApplyButton,
        )
        self.apply_terrain_button.clicked.connect(self.request_terrain_switch)
        layout.addWidget(self.apply_terrain_button, 4, 0, 1, 2)
        self.terrain_combo.currentIndexChanged.connect(self._update_terrain_parameter_state)
        self._update_terrain_parameter_state()
        return group

    def _create_obstacle_group(self) -> object:
        group = self.QtWidgets.QGroupBox("障碍物")
        layout = self.QtWidgets.QGridLayout(group)
        layout.setContentsMargins(8, 7, 8, 7)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(3)
        layout.setColumnStretch(1, 1)
        self.obstacle_group = group

        self.obstacle_mode_combo = self.QtWidgets.QComboBox()
        mode_labels = {"static": "静态", "moving": "移动", "mixed": "混合"}
        for mode_name in OBSTACLE_MODES:
            self.obstacle_mode_combo.addItem(mode_labels[mode_name], mode_name)
        self.obstacle_shape_combo = self.QtWidgets.QComboBox()
        shape_labels = {"box": "方块", "cylinder": "圆柱", "sphere": "球体"}
        for shape_name in OBSTACLE_SHAPES:
            self.obstacle_shape_combo.addItem(shape_labels[shape_name], shape_name)
        self.obstacle_count_spin = self.QtWidgets.QSpinBox()
        self.obstacle_count_spin.setRange(1, 50)
        self.obstacle_count_spin.setValue(1)
        self.obstacle_seed_spin = self.QtWidgets.QSpinBox()
        self.obstacle_seed_spin.setRange(-2_147_483_648, 2_147_483_647)
        self.obstacle_seed_spin.setValue(0)
        self.obstacle_speed_spin = self.QtWidgets.QDoubleSpinBox()
        self.obstacle_speed_spin.setRange(0.01, 3.0)
        self.obstacle_speed_spin.setSingleStep(0.05)
        self.obstacle_speed_spin.setValue(0.35)
        self.obstacle_ratio_spin = self.QtWidgets.QSpinBox()
        self.obstacle_ratio_spin.setRange(0, 100)
        self.obstacle_ratio_spin.setSingleStep(5)
        self.obstacle_ratio_spin.setSuffix("%")
        self.obstacle_ratio_spin.setValue(30)

        controls = (
            ("模式", self.obstacle_mode_combo),
            ("形状", self.obstacle_shape_combo),
            ("数量", self.obstacle_count_spin),
            ("随机种子", self.obstacle_seed_spin),
            ("速度", self.obstacle_speed_spin),
            ("移动占比", self.obstacle_ratio_spin),
        )
        for row, (name, control) in enumerate(controls):
            layout.addWidget(self.QtWidgets.QLabel(name), row, 0)
            layout.addWidget(control, row, 1)

        self.add_obstacles_button = self.QtWidgets.QPushButton("添加障碍")
        self._configure_button(
            self.add_obstacles_button,
            "按当前参数随机添加障碍物",
            self.QtWidgets.QStyle.StandardPixmap.SP_FileDialogNewFolder,
        )
        self.add_obstacles_button.clicked.connect(self.request_add_obstacles)
        self.delete_obstacle_button = self.QtWidgets.QPushButton("删除选中")
        self._configure_button(
            self.delete_obstacle_button,
            "删除表格中选中的障碍物",
            self.QtWidgets.QStyle.StandardPixmap.SP_TrashIcon,
        )
        self.delete_obstacle_button.clicked.connect(self.request_delete_obstacle)
        self.clear_obstacles_button = self.QtWidgets.QPushButton("清空障碍")
        self._configure_button(
            self.clear_obstacles_button,
            "清空当前场地的全部障碍物",
            self.QtWidgets.QStyle.StandardPixmap.SP_DialogResetButton,
        )
        self.clear_obstacles_button.clicked.connect(self.request_clear_obstacles)
        layout.addWidget(self.add_obstacles_button, 6, 0, 1, 2)
        layout.addWidget(self.delete_obstacle_button, 7, 0, 1, 2)
        layout.addWidget(self.clear_obstacles_button, 8, 0, 1, 2)
        self.structure_status_label = self.QtWidgets.QLabel("结构状态：就绪")
        self.structure_status_label.setWordWrap(True)
        layout.addWidget(self.structure_status_label, 9, 0, 1, 2)
        self.switch_status_label = self.structure_status_label
        self.obstacle_mode_combo.currentIndexChanged.connect(self._update_obstacle_parameter_state)
        self._update_obstacle_parameter_state()
        self._update_delete_obstacle_button_state()
        return group

    def _create_capture_group(self) -> object:
        """创建手动 MID-360 采集入口；v2 由 C++ Recorder/Export 完成真实边界。"""
        group = self.QtWidgets.QGroupBox("MID-360 采集")
        layout = self.QtWidgets.QGridLayout(group)
        layout.setContentsMargins(8, 7, 8, 7)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)
        layout.setColumnStretch(1, 1)

        self.capture_duration_combo = self.QtWidgets.QComboBox()
        for label, duration in (
            ("1 分钟", 60),
            ("1.5 分钟", 90),
            ("3 分钟", 180),
            ("不限", None),
        ):
            self.capture_duration_combo.addItem(label, duration)
        layout.addWidget(self.QtWidgets.QLabel("最长采集"), 0, 0)
        layout.addWidget(self.capture_duration_combo, 0, 1)

        self.capture_status_label = self.QtWidgets.QLabel("状态：未采集")
        self.capture_status_label.setWordWrap(True)
        layout.addWidget(self.capture_status_label, 1, 0, 1, 2)
        self.capture_path_label = self.QtWidgets.QLabel("输出：--")
        self.capture_path_label.setWordWrap(True)
        self.capture_path_label.setTextInteractionFlags(
            self.QtCore.Qt.TextSelectableByMouse
        )
        layout.addWidget(self.capture_path_label, 2, 0, 1, 2)

        self.capture_start_button = self.QtWidgets.QPushButton("启用采集")
        self._configure_button(
            self.capture_start_button,
            (
                "请求 C++ Recorder 在下一完整五 topic 边界开始采集"
                if self.v2_dashboard_enabled
                else "冻结当前车辆、地形和障碍物，再开始记录驾驶轨迹"
            ),
            self.QtWidgets.QStyle.StandardPixmap.SP_MediaPlay,
        )
        self.capture_start_button.clicked.connect(self.request_capture_start)
        self.capture_stop_button = self.QtWidgets.QPushButton("结束采集")
        self._configure_button(
            self.capture_stop_button,
            (
                "请求 C++ Recorder 在下一完整五 topic 边界结束，并交给 C++ Export"
                if self.v2_dashboard_enabled
                else "结束轨迹记录并开始离线 MID-360 高保真重建"
            ),
            self.QtWidgets.QStyle.StandardPixmap.SP_MediaStop,
        )
        self.capture_stop_button.clicked.connect(self.request_capture_stop)
        self.capture_viewer_button = self.QtWidgets.QPushButton("导入 Livox Viewer")
        self._configure_button(
            self.capture_viewer_button,
            "打开已验证的本次 LVX2 文件",
            self.QtWidgets.QStyle.StandardPixmap.SP_DialogOpenButton,
        )
        self.capture_viewer_button.clicked.connect(self.request_livox_viewer_import)
        self.capture_compress_button = self.QtWidgets.QPushButton("压缩 MCAP（离线）")
        self._configure_button(
            self.capture_compress_button,
            "以 Zstandard 生成独立压缩副本；原始 MCAP 保留不变",
            self.QtWidgets.QStyle.StandardPixmap.SP_DriveFDIcon,
        )
        self.capture_compress_button.clicked.connect(self.request_capture_compression)
        layout.addWidget(self.capture_start_button, 3, 0, 1, 2)
        layout.addWidget(self.capture_stop_button, 4, 0, 1, 2)
        layout.addWidget(self.capture_viewer_button, 5, 0, 1, 2)
        layout.addWidget(self.capture_compress_button, 6, 0, 1, 2)
        self._refresh_capture_controls()
        return group

    def _add_developer_diagnostics_tab(
        self,
        tabs: object,
        *,
        camera_follow_enabled: bool,
        camera_follow_view: str,
    ) -> None:
        """创建仅含内部遥测和相机参数的开发者页。"""
        developer_tab = self.QtWidgets.QWidget()
        developer_layout = self.QtWidgets.QVBoxLayout(developer_tab)
        self.developer_layout = developer_layout
        developer_layout.setContentsMargins(4, 4, 4, 4)
        developer_layout.setSpacing(4)

        self.telemetry_scroll = self.QtWidgets.QScrollArea()
        self.telemetry_scroll.setWidgetResizable(True)
        content = self.QtWidgets.QWidget()
        content_layout = self.QtWidgets.QVBoxLayout(content)

        label_font = self.QtGui.QFont()
        label_font.setPointSize(11)
        value_font = self.QtGui.QFont("Monospace")
        value_font.setPointSize(11)
        value_font.setStyleHint(self.QtGui.QFont.Monospace)
        group_font = self.QtGui.QFont()
        group_font.setPointSize(12)
        group_font.setBold(True)
        for group_name, rows in dashboard_groups(RobotTelemetry()):
            group_label = self.QtWidgets.QLabel(group_name)
            group_label.setFont(group_font)
            content_layout.addWidget(group_label)
            grid = self.QtWidgets.QGridLayout()
            grid.setColumnStretch(1, 1)
            for row, (name, value) in enumerate(rows):
                name_label = self.QtWidgets.QLabel(name)
                value_label = self.QtWidgets.QLabel(value)
                name_label.setFont(label_font)
                value_label.setFont(value_font)
                name_label.setWordWrap(True)
                value_label.setWordWrap(True)
                value_label.setTextInteractionFlags(self.QtCore.Qt.TextSelectableByMouse)
                grid.addWidget(name_label, row, 0)
                grid.addWidget(value_label, row, 1)
                self.labels[name] = value_label
            content_layout.addLayout(grid)
        content_layout.addStretch(1)
        self.telemetry_scroll.setWidget(content)
        developer_layout.addWidget(self.telemetry_scroll, stretch=3)

        self.diagnostic_control_scroll = self.QtWidgets.QScrollArea()
        self.diagnostic_control_scroll.setWidgetResizable(True)
        self.diagnostic_control_scroll.setHorizontalScrollBarPolicy(self.QtCore.Qt.ScrollBarAlwaysOff)
        self.diagnostic_control_scroll.setSizePolicy(
            self.QtWidgets.QSizePolicy.Policy.Preferred,
            self.QtWidgets.QSizePolicy.Policy.Ignored,
        )
        self.diagnostic_control_scroll.setMinimumHeight(DASHBOARD_VERY_COMPACT_SCROLL_HEIGHT)
        self.diagnostic_control_content = self.QtWidgets.QWidget()
        diagnostic_layout = self.QtWidgets.QVBoxLayout(self.diagnostic_control_content)
        diagnostic_layout.setContentsMargins(4, 4, 4, 4)
        diagnostic_layout.setSpacing(6)

        resource_group = self.QtWidgets.QGroupBox("资源状态（1 Hz）")
        self.resource_layout = self.QtWidgets.QFormLayout(resource_group)
        resource_hint = self.QtWidgets.QLabel("等待资源采样")
        resource_hint.setWordWrap(True)
        self.resource_layout.addRow(resource_hint)

        camera_group = self.QtWidgets.QGroupBox("相机")
        camera_layout = self.QtWidgets.QGridLayout(camera_group)
        camera_layout.setColumnStretch(1, 1)
        self.camera_follow_checkbox = self.QtWidgets.QCheckBox("启用跟随")
        self.camera_follow_checkbox.setChecked(camera_follow_enabled)
        self.camera_view_combo = self.QtWidgets.QComboBox()
        for view_label, view_name in (("车后", "front"), ("侧面", "side"), ("固定", "custom")):
            self.camera_view_combo.addItem(view_label, view_name)
        camera_view_index = self.camera_view_combo.findData(camera_follow_view)
        if camera_view_index >= 0:
            self.camera_view_combo.setCurrentIndex(camera_view_index)
        camera_layout.addWidget(self.camera_follow_checkbox, 0, 0, 1, 2)
        camera_layout.addWidget(self.QtWidgets.QLabel("视角"), 1, 0)
        camera_layout.addWidget(self.camera_view_combo, 1, 1)
        self.camera_follow_checkbox.toggled.connect(self._update_camera_follow_state)
        self._update_camera_follow_state()

        self.diagnostic_control_groups = [resource_group, camera_group]
        for group in self.diagnostic_control_groups:
            diagnostic_layout.addWidget(group)
        diagnostic_layout.addStretch(1)
        self.diagnostic_control_scroll.setWidget(self.diagnostic_control_content)
        developer_layout.addWidget(self.diagnostic_control_scroll, stretch=1)
        tabs.addTab(developer_tab, "开发者诊断")

    def _configure_button(self, button: object, tooltip: str, standard_pixmap: object) -> None:
        """统一使用 Qt 标准图标，并为命令按钮提供可访问提示。"""
        button.setToolTip(tooltip)
        button.setIcon(self.window.style().standardIcon(standard_pixmap))

    def _add_obstacle_tab(self, tabs: object) -> None:
        """创建障碍物快照表；表格只保存逻辑字段，不接触 PyBullet body。"""
        obstacle_tab = self.QtWidgets.QWidget()
        obstacle_layout = self.QtWidgets.QVBoxLayout(obstacle_tab)
        self.obstacle_table = self.QtWidgets.QTableWidget(0, 4)
        self.obstacle_table.setHorizontalHeaderLabels(["逻辑ID", "模式", "形状", "位置"])
        self.obstacle_table.setEditTriggers(self.QtWidgets.QAbstractItemView.NoEditTriggers)
        self.obstacle_table.setSelectionBehavior(self.QtWidgets.QAbstractItemView.SelectRows)
        self.obstacle_table.setSelectionMode(self.QtWidgets.QAbstractItemView.SingleSelection)
        self.obstacle_table.verticalHeader().setVisible(False)
        self.obstacle_table.setHorizontalScrollBarPolicy(self.QtCore.Qt.ScrollBarAlwaysOff)
        self.obstacle_table.setWordWrap(True)
        self.obstacle_table.horizontalHeader().setSectionResizeMode(
            self.QtWidgets.QHeaderView.Stretch
        )
        self.obstacle_table.itemSelectionChanged.connect(self._update_delete_obstacle_button_state)
        obstacle_layout.addWidget(self.obstacle_table, stretch=1)
        tabs.addTab(obstacle_tab, "障碍物")

    def _add_plot_tabs(self, tabs: object) -> None:
        """只登记折线图；用户在下拉菜单勾选后才创建 Matplotlib 资源。"""
        legacy_specs = dashboard_plot_specs()
        self.plot_specs = [*legacy_specs, *self.interface_plot_specs]
        self._plot_spec_by_label = {
            spec.tab_label: spec for spec in self.plot_specs
        }
        self.plot_figures = {}
        self.plot_canvases = {}
        self.plot_axes = {}
        self.plot_lines = {}
        self.plot_legends = {}
        self.plot_texts = {}
        self.plot_layouts = {}
        self.plot_buttons = []
        self.plot_actions = {}
        self.plot_selector_button = self.QtWidgets.QToolButton()
        self.plot_selector_button.setText("图表（按需）")
        self.plot_selector_button.setPopupMode(
            self.QtWidgets.QToolButton.ToolButtonPopupMode.InstantPopup
        )
        self.plot_selector_menu = self.QtWidgets.QMenu(self.plot_selector_button)
        self.plot_selector_button.setMenu(self.plot_selector_menu)
        for spec in self.plot_specs:
            action = self.plot_selector_menu.addAction(spec.tab_label)
            action.setCheckable(True)
            action.setChecked(False)
            action.toggled.connect(
                lambda checked, label=spec.tab_label: self._set_plot_enabled(
                    label, checked
                )
            )
            self.plot_actions[spec.tab_label] = action
        tabs.setCornerWidget(
            self.plot_selector_button,
            self.QtCore.Qt.Corner.TopRightCorner,
        )
        if self.show_lidar_tools:
            from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
            from matplotlib.figure import Figure

            self._add_lidar_plot_tab(tabs, Figure=Figure, FigureCanvas=FigureCanvas)
        self._set_steering_tab_state(self._interface_robot_model)

    def _set_plot_enabled(self, tab_label: str, enabled: bool) -> None:
        """按勾选状态创建或释放单张折线图，未启用图表不保留后台缓存。"""
        if tab_label not in self._plot_spec_by_label:
            raise ValueError(f"unknown dashboard plot: {tab_label}")
        if enabled:
            if tab_label in self.plot_canvases:
                return
            from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
            from matplotlib.figure import Figure

            self._add_line_plot_tab(
                self.tabs,
                self._plot_spec_by_label[tab_label],
                Figure=Figure,
                FigureCanvas=FigureCanvas,
            )
            self.tabs.setCurrentWidget(self.plot_canvases[tab_label].parentWidget())
            # 延迟创建发生在窗口已经定型之后，必须立即套用当前紧凑布局。
            self._update_layout_for_size(self.window.width(), self.window.height())
            self._apply_plot_series(tab_label)
            self.plot_canvases[tab_label].draw()
            self._plot_rendered_revisions[tab_label] = self._plot_data_revisions[
                tab_label
            ]
            self._plot_dirty_tabs.discard(tab_label)
            return
        self._release_plot_tab(tab_label)

    def _release_plot_tab(self, tab_label: str) -> None:
        """释放取消勾选图表的 Qt/Matplotlib 对象及其回调。"""
        canvas = self.plot_canvases.pop(tab_label, None)
        if canvas is None:
            return
        plot_tab = canvas.parentWidget()
        index = self.tabs.indexOf(plot_tab)
        if index >= 0:
            self.tabs.removeTab(index)
        retained_connections = []
        for registered_canvas, event_name, callback_id in self._mpl_connections:
            if registered_canvas is canvas:
                registered_canvas.mpl_disconnect(callback_id)
            else:
                retained_connections.append(
                    (registered_canvas, event_name, callback_id)
                )
        self._mpl_connections = retained_connections
        spec = self._plot_spec_by_label[tab_label]
        for line_spec in spec.lines:
            self.plot_lines.pop(line_spec.key, None)
            self._interface_line_keys.discard(line_spec.key)
        self.plot_buttons = [
            button for button in self.plot_buttons
            if not plot_tab.isAncestorOf(button)
        ]
        self.plot_figures.pop(tab_label, None)
        self.plot_axes.pop(tab_label, None)
        self.plot_legends.pop(tab_label, None)
        self.plot_layouts.pop(tab_label, None)
        self.no_steering_texts.pop(tab_label, None)
        self.plot_texts.pop(f"{tab_label}:status", None)
        self._plot_data_revisions.pop(tab_label, None)
        self._plot_rendered_revisions.pop(tab_label, None)
        self._plot_next_draw_time.pop(tab_label, None)
        self._plot_draw_started_at.pop(tab_label, None)
        self._plot_dirty_tabs.discard(tab_label)
        figure = getattr(canvas, "figure", None)
        if figure is not None:
            figure.clear()
            figure.set_canvas(None)
        canvas.close()
        canvas.figure = None
        canvas.deleteLater()
        plot_tab.deleteLater()
        legacy_labels = {spec.tab_label for spec in dashboard_plot_specs()}
        interface_labels = {
            spec.tab_label for spec in self.interface_plot_specs
        }
        if not legacy_labels.intersection(self.plot_canvases):
            self.plot_buffer.clear()
        if not interface_labels.intersection(self.plot_canvases):
            self.interface_plot_buffer.clear()

    def _set_steering_tab_state(self, robot_model: str) -> None:
        """差速车型没有转向关节，禁用对应图表选项。"""
        applicable = bool(get_robot_model(robot_model).steering_joint_names)
        for label in ("转向命令", "转向反馈"):
            action = self.plot_actions.get(label)
            if action is not None:
                if not applicable and action.isChecked():
                    action.setChecked(False)
                action.setEnabled(applicable)
                action.setToolTip("" if applicable else "不适用于差速车型")
            canvas = self.plot_canvases.get(label)
            if canvas is not None:
                index = self.tabs.indexOf(canvas.parentWidget())
                self.tabs.setTabEnabled(index, applicable)
                self.tabs.setTabToolTip(
                    index, "" if applicable else "不适用于差速车型"
                )

    def _register_plot_canvas(
        self,
        tab_label: str,
        title: str,
        x_label: str,
        y_label: str,
        *,
        Figure: object,
        FigureCanvas: object,
    ) -> tuple[object, object, object, object, object]:
        """创建一个固定 Figure/Axis/Canvas，并登记统一绘制门禁。"""
        plot_tab = self.QtWidgets.QWidget()
        plot_layout = self.QtWidgets.QVBoxLayout(plot_tab)
        figure = Figure(figsize=DASHBOARD_PLOT_FIGURE_SIZE)
        canvas = FigureCanvas(figure)
        # Canvas 尚未加入 Qt 布局时也必须先登记，确保后续构造异常可统一释放。
        self.plot_figures[tab_label] = figure
        self.plot_canvases[tab_label] = canvas
        axis = figure.subplots(1, 1)
        axis.set_title(title, fontsize=7)
        axis.set_xlabel(x_label, fontsize=9)
        axis.set_ylabel(y_label, fontsize=9)
        if tab_label == "轨迹":
            # 大坐标轨迹直接显示完整刻度，避免科学计数 offset 挤占 xlabel 空间。
            axis.ticklabel_format(axis="both", style="plain", useOffset=False)
        else:
            # 时间轴标签靠左、科学计数 offset 靠右，窄画布也不会互相遮挡。
            axis.xaxis.set_label_coords(0.0, -0.17)
            axis.xaxis.label.set_horizontalalignment("left")
        # 窄侧栏不显示轴端最外层刻度，避免完整文字落到 Figure 边界之外。
        axis.xaxis.get_major_locator().set_params(prune="both")
        axis.yaxis.get_major_locator().set_params(prune="both")
        axis.tick_params(axis="both", labelsize=8)
        axis.grid(True, alpha=0.3)
        figure.subplots_adjust(**DASHBOARD_PLOT_MARGINS)
        event_name = "draw_event"
        callback_id = canvas.mpl_connect(
            event_name,
            lambda _event, label=tab_label: self._record_plot_draw(label),
        )
        self._mpl_connections.append((canvas, event_name, callback_id))
        self.plot_layouts[tab_label] = plot_layout
        self.plot_axes[tab_label] = axis
        self._plot_data_revisions[tab_label] = 0
        self._plot_rendered_revisions[tab_label] = -1
        self._plot_next_draw_time[tab_label] = 0.0
        plot_layout.addWidget(canvas, stretch=1)
        return plot_tab, plot_layout, figure, canvas, axis

    def _add_line_plot_tab(
        self,
        tabs: object,
        spec: object,
        *,
        Figure: object,
        FigureCanvas: object,
    ) -> None:
        """创建一个折线页，后续刷新只允许调用 `set_data`。"""
        plot_tab, plot_layout, _figure, _canvas, axis = self._register_plot_canvas(
            spec.tab_label,
            spec.title,
            spec.x_label,
            spec.y_label,
            Figure=Figure,
            FigureCanvas=FigureCanvas,
        )
        for line_spec in spec.lines:
            self.plot_lines[line_spec.key] = axis.plot([], [], label=line_spec.label)[0]
            if spec in self.interface_plot_specs:
                self._interface_line_keys.add(line_spec.key)
        handles = [self.plot_lines[line_spec.key] for line_spec in spec.lines]
        self.plot_legends[spec.tab_label] = self._create_plot_legend(
            axis,
            handles,
            [line_spec.label for line_spec in spec.lines],
        )
        if getattr(spec, "equal_aspect", False):
            axis.set_aspect("equal", adjustable="datalim")
        if spec.tab_label in ("转向命令", "转向反馈"):
            text_artist = axis.text(
                0.5,
                0.5,
                "不适用于差速车型" if not spec.lines else "",
                transform=axis.transAxes,
                ha="center",
                va="center",
                fontproperties=_dashboard_cjk_font(),
            )
            self.no_steering_texts[spec.tab_label] = text_artist
            self.plot_texts[f"{spec.tab_label}:status"] = text_artist

        button_row = self.QtWidgets.QHBoxLayout()
        clear_button = self.QtWidgets.QPushButton("清空曲线")
        self._configure_button(
            clear_button,
            "清空当前曲线数据",
            self.QtWidgets.QStyle.StandardPixmap.SP_DialogResetButton,
        )
        clear_button.clicked.connect(self.clear_plots)
        save_button = self.QtWidgets.QPushButton("保存当前图")
        self._configure_button(
            save_button,
            "保存当前图到图片目录",
            self.QtWidgets.QStyle.StandardPixmap.SP_DialogSaveButton,
        )
        save_button.clicked.connect(
            lambda _checked=False, label=spec.tab_label: self.save_plot_snapshot(label)
        )
        self.plot_buttons.extend((clear_button, save_button))
        button_row.addWidget(clear_button)
        button_row.addWidget(save_button)
        plot_layout.addLayout(button_row)
        tabs.addTab(plot_tab, spec.tab_label)

    @staticmethod
    def _create_plot_legend(axis: object, handles: list[object], labels: list[str]) -> object:
        """空转向页也创建稳定 Legend，且不制造虚假零值 Line2D。"""
        legend_columns = max(1, min(2, len(labels)))
        if handles:
            return axis.legend(
                handles=handles,
                labels=labels,
                ncols=legend_columns,
                columnspacing=1.0,
                handletextpad=0.6,
                **DASHBOARD_PLOT_LEGEND_STYLE,
            )
        from matplotlib.legend import Legend

        legend = Legend(
            axis,
            [],
            [],
            ncols=legend_columns,
            columnspacing=1.0,
            handletextpad=0.6,
            **DASHBOARD_PLOT_LEGEND_STYLE,
        )
        axis.add_artist(legend)
        return legend

    def _add_lidar_plot_tab(
        self,
        tabs: object,
        *,
        Figure: object,
        FigureCanvas: object,
    ) -> None:
        """创建单个可复用点云 artist，并提供显式的标准格式导出。"""
        import numpy as np

        plot_tab, plot_layout, _figure, _canvas, axis = self._register_plot_canvas(
            "LiDAR点云",
            "LiDAR obstacles (base_link)",
            "forward x [m]",
            "left y [m]",
            Figure=Figure,
            FigureCanvas=FigureCanvas,
        )
        axis.set_aspect("equal", adjustable="box")
        self.lidar_collection = axis.scatter([], [], s=4, zorder=3)
        self.lidar_collection.set_offsets(np.empty((0, 2)))
        self._set_lidar_limits(np.empty((0, 2)))
        axis.axhline(0.0, color="#687078", linewidth=0.6, alpha=0.35, zorder=1)
        axis.axvline(0.0, color="#687078", linewidth=0.6, alpha=0.35, zorder=1)
        from matplotlib.lines import Line2D
        from matplotlib.patches import Circle

        self.lidar_range_ring = Circle(
            (0.0, 0.0),
            radius=30.0,
            fill=False,
            edgecolor="#687078",
            linewidth=0.7,
            linestyle="--",
            alpha=0.45,
            zorder=1,
        )
        axis.add_patch(self.lidar_range_ring)
        self.lidar_vehicle_marker = axis.plot(
            [0.0],
            [0.0],
            marker=">",
            markersize=8,
            color="#111827",
            linestyle="None",
            zorder=4,
        )[0]
        legend_handles = [
            Line2D(
                [],
                [],
                marker="o",
                markersize=4,
                markerfacecolor=color,
                markeredgecolor="none",
                linestyle="None",
            )
            for color in DASHBOARD_LIDAR_TAG_COLORS
        ] + [self.lidar_vehicle_marker]
        self.plot_legends["LiDAR点云"] = self._create_plot_legend(
            axis,
            legend_handles,
            [*DASHBOARD_LIDAR_TAG_LABELS, "vehicle forward"],
        )
        self.lidar_front_time_text = axis.text(
            0.02,
            0.98,
            "前: --",
            transform=axis.transAxes,
            va="top",
            fontproperties=_dashboard_cjk_font(),
        )
        self.lidar_rear_time_text = axis.text(
            0.02,
            0.90,
            "后: --",
            transform=axis.transAxes,
            va="top",
            fontproperties=_dashboard_cjk_font(),
        )
        self.plot_texts["LiDAR点云:front"] = self.lidar_front_time_text
        self.plot_texts["LiDAR点云:rear"] = self.lidar_rear_time_text

        button_row = self.QtWidgets.QHBoxLayout()
        save_button = self.QtWidgets.QPushButton("保存当前图")
        self._configure_button(
            save_button,
            "保存当前图到图片目录",
            self.QtWidgets.QStyle.StandardPixmap.SP_DialogSaveButton,
        )
        save_button.clicked.connect(
            lambda _checked=False: self.save_plot_snapshot("LiDAR点云")
        )
        self.plot_buttons.append(save_button)
        button_row.addWidget(save_button)
        plot_layout.addLayout(button_row)
        tabs.addTab(plot_tab, "LiDAR点云")

    def _key_press_event(self, event: object) -> None:
        if self._handle_tab_cycle(event) or self._handle_key_press(event.key()):
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
        """Dashboard 需要接管的驾驶按键。"""
        return {
            self._normalize_key(self.QtCore.Qt.Key_Up),
            self._normalize_key(self.QtCore.Qt.Key_Down),
            self._normalize_key(self.QtCore.Qt.Key_Left),
            self._normalize_key(self.QtCore.Qt.Key_Right),
            self._normalize_key(self.QtCore.Qt.Key_W),
            self._normalize_key(self.QtCore.Qt.Key_S),
            self._normalize_key(self.QtCore.Qt.Key_A),
            self._normalize_key(self.QtCore.Qt.Key_D),
            self._normalize_key(self.QtCore.Qt.Key_Space),
        }

    def _handle_tab_cycle(self, event: object) -> bool:
        """在任意子控件焦点下统一处理顶层 Ctrl+Tab 页签切换。"""
        if event.key() != self.QtCore.Qt.Key_Tab:
            return False
        modifiers = event.modifiers()
        if not (modifiers & self.QtCore.Qt.ControlModifier):
            return False
        direction = -1 if modifiers & self.QtCore.Qt.ShiftModifier else 1
        count = self.tabs.count()
        if count:
            self.tabs.setCurrentIndex((self.tabs.currentIndex() + direction) % count)
        return True

    def _handle_key_press(self, key: object) -> bool:
        """处理键盘按下事件；返回 True 表示事件已用于驾驶控制。"""
        key_code = self._normalize_key(key)
        if key_code not in self._control_keys():
            return False
        self._pressed_keys.add(key_code)
        return True

    def _handle_key_release(self, key: object) -> bool:
        """处理键盘释放事件；返回 True 表示事件已用于驾驶控制。"""
        key_code = self._normalize_key(key)
        if key_code not in self._control_keys():
            return False
        self._pressed_keys.discard(key_code)
        return True

    @property
    def paused(self) -> bool:
        """返回持续暂停状态；调用方只能通过界面命令切换。"""
        return self._paused

    def _toggle_paused(self) -> None:
        """切换持续暂停，并同步只读状态和按钮图标。"""
        self._paused = not self._paused
        if self._paused:
            self.pause_status_label.setText("已暂停")
            self.pause_button.setText("继续")
            self.pause_button.setToolTip("继续物理步进")
            icon = self.QtWidgets.QStyle.StandardPixmap.SP_MediaPlay
        else:
            self.pause_status_label.setText("运行中")
            self.pause_button.setText("暂停")
            self.pause_button.setToolTip("暂停物理步进；接口连接状态继续刷新")
            icon = self.QtWidgets.QStyle.StandardPixmap.SP_MediaPause
        self.pause_button.setIcon(self.window.style().standardIcon(icon))

    def _update_layout_for_size(self, width: int, height: int) -> None:
        """按固定窗口尺寸分配区域，并仅为异常超窄画布补足文字边距。"""
        compact_height = height < 600
        very_compact_height = height <= 320
        margins = self.window_layout.contentsMargins()
        reserved_height = (
            margins.top()
            + margins.bottom()
            + self.title_label.sizeHint().height()
            + 2 * self.window_layout.spacing()
        )
        # 最小高度不能超过 pane 可用预算的一半，否则会覆盖上下 1:1 stretch。
        half_pane_budget = max(0, (int(height) - reserved_height) // 2)
        tabs_min_height = min(DASHBOARD_TOP_TABS_MIN_HEIGHT, half_pane_budget)
        self.tabs.setMinimumHeight(tabs_min_height)
        # 企业控制区始终参与 1:1 stretch，小窗口依靠自身滚动条保留全部控件。
        self.control_scroll.setMaximumHeight(16_777_215)
        if self.diagnostic_control_scroll is not None:
            if very_compact_height:
                self.diagnostic_control_scroll.setMaximumHeight(DASHBOARD_VERY_COMPACT_SCROLL_HEIGHT)
                self.developer_layout.setContentsMargins(0, 0, 0, 0)
                self.developer_layout.setSpacing(0)
            elif compact_height:
                self.diagnostic_control_scroll.setMaximumHeight(
                    max(DASHBOARD_COMPACT_CONTROL_SCROLL_HEIGHT, min(64, height // 14))
                )
                self.developer_layout.setContentsMargins(0, 0, 0, 0)
                self.developer_layout.setSpacing(0)
            else:
                self.diagnostic_control_scroll.setMaximumHeight(16_777_215)
                self.developer_layout.setContentsMargins(4, 4, 4, 4)
                self.developer_layout.setSpacing(4)
        for plot_layout in self.plot_layouts.values():
            if very_compact_height:
                plot_layout.setContentsMargins(0, 0, 0, 0)
                plot_layout.setSpacing(0)
            elif compact_height:
                plot_layout.setContentsMargins(2, 2, 2, 2)
                plot_layout.setSpacing(2)
            else:
                plot_layout.setContentsMargins(9, 9, 9, 9)
                plot_layout.setSpacing(6)
        for button in self.plot_buttons:
            if very_compact_height:
                button.setStyleSheet("min-height: 18px; padding: 0 4px;")
                button.setMaximumHeight(DASHBOARD_VERY_COMPACT_PLOT_BUTTON_HEIGHT)
            else:
                button.setStyleSheet("")
                button.setMaximumHeight(16_777_215)
        legend_font_size = (
            DASHBOARD_VERY_COMPACT_LEGEND_FONT_SIZE
            if very_compact_height
            else DASHBOARD_PLOT_LEGEND_STYLE["fontsize"]
        )
        axis_label_font_size = DASHBOARD_VERY_COMPACT_AXIS_LABEL_FONT_SIZE if very_compact_height else 9
        tick_font_size = DASHBOARD_VERY_COMPACT_TICK_FONT_SIZE if very_compact_height else 8
        for axis in self.plot_axes.values():
            legend = axis.get_legend()
            if legend is not None:
                for text in legend.get_texts():
                    text.set_fontsize(legend_font_size)
            # 极矮画布只压缩外围文字，不侵占数据 axes 的相对高度。
            axis.xaxis.label.set_fontsize(axis_label_font_size)
            axis.yaxis.label.set_fontsize(axis_label_font_size)
            axis.xaxis.labelpad = 0.0 if very_compact_height else 4.0
            axis.tick_params(
                axis="x",
                labelsize=tick_font_size,
                pad=0.0 if very_compact_height else 3.5,
            )
            axis.tick_params(axis="y", labelsize=tick_font_size)
        if height <= 320:
            plot_margins = DASHBOARD_VERY_COMPACT_PLOT_MARGINS
        elif compact_height:
            plot_margins = DASHBOARD_COMPACT_HEIGHT_PLOT_MARGINS
        else:
            plot_margins = DASHBOARD_PLOT_MARGINS
        if width < DASHBOARD_MIN_FULL_WIDTH_PLOT:
            plot_margins = {
                **plot_margins,
                "left": max(plot_margins["left"], DASHBOARD_NARROW_PLOT_LEFT_MARGIN),
            }
        for figure in self.plot_figures.values():
            figure.subplots_adjust(**plot_margins)

    @staticmethod
    def _rect_attribute(rect: object, name: str) -> int:
        """按属性读取并校验布局矩形，同时兼容 Qt 的零参数 getter。"""
        if not hasattr(rect, name):
            raise ValueError(f"window rect is missing {name}")
        value = getattr(rect, name)
        if callable(value):
            value = value()
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"window rect {name} must be an integer")
        return value

    def apply_window_rect(
        self,
        rect: object,
        *,
        display_metrics: DisplayMetrics | None = None,
    ) -> None:
        """把 X11 物理外框目标换算为 Qt 逻辑客户区并固定应用。"""
        x = self._rect_attribute(rect, "x")
        y = self._rect_attribute(rect, "y")
        width = self._rect_attribute(rect, "width")
        height = self._rect_attribute(rect, "height")
        if width <= 0 or height <= 0:
            raise ValueError("window rect width and height must be positive")

        # 窗口已 show；先让窗口管理器回填标题栏边距，再换算固定客户区。
        screen_object = self.window.screen() or self.app.primaryScreen()
        handle = self.window.windowHandle()
        if screen_object is None or handle is None:
            raise ValueError("dashboard window screen or handle is unavailable")
        if display_metrics is None:
            screen_geometry = screen_object.geometry()
            available_geometry = screen_object.availableGeometry()
            display_metrics = DisplayMetrics(
                screen=Rect(
                    screen_geometry.x(),
                    screen_geometry.y(),
                    screen_geometry.width(),
                    screen_geometry.height(),
                ),
                available=Rect(
                    available_geometry.x(),
                    available_geometry.y(),
                    available_geometry.width(),
                    available_geometry.height(),
                ),
                device_pixel_ratio=float(screen_object.devicePixelRatio()),
            )
        elif not isinstance(display_metrics, DisplayMetrics):
            raise TypeError("display_metrics must be DisplayMetrics or None")

        # 先用目标主屏指标移动外框，避免从副屏读取错误 DPR。
        preliminary = logical_client_rect_for_outer(
            Rect(x, y, width, height),
            screen=display_metrics.screen,
            device_pixel_ratio=display_metrics.device_pixel_ratio,
            frame_extents=FrameExtents(0, 0, 0, 0),
        )
        self.window.move(preliminary.x, preliminary.y)
        self.app.processEvents()

        def frame_extents() -> FrameExtents | None:
            margins = handle.frameMargins()
            values = (
                margins.left(),
                margins.right(),
                margins.top(),
                margins.bottom(),
            )
            # Qt 在 X11 装饰尚未建立时以 -1 表示未知边距。
            if any(value < 0 for value in values):
                return None
            return FrameExtents(
                *values,
            )

        window_manager_expected = (
            self.app.platformName().lower() == "xcb"
            and x11_window_manager_available()
        )
        stable_frame_extents = wait_for_dashboard_frame_extents(
            frame_extents_getter=frame_extents,
            process_events=self.app.processEvents,
            window_manager_expected=window_manager_expected,
        )
        target = logical_client_rect_for_outer(
            Rect(x, y, width, height),
            screen=display_metrics.screen,
            device_pixel_ratio=display_metrics.device_pixel_ratio,
            frame_extents=stable_frame_extents,
        )
        self.window.setFixedSize(target.width, target.height)
        self.window.move(target.x, target.y)
        self.app.processEvents()
        if self.app.platformName().lower() == "xcb":
            wait_for_x11_outer_geometry(
                str(int(self.window.winId())),
                Rect(x, y, width, height),
            )
        self._update_layout_for_size(target.width, target.height)

    @property
    def capture_state(self) -> ManualCaptureUiState:
        """返回当前采集 UI 状态；主循环据此决定是否写轨迹。"""
        return self._capture_state

    def _capture_locks_structure(self) -> bool:
        return self._capture_start_pending or self._capture_state in {
            ManualCaptureUiState.RECORDING,
            ManualCaptureUiState.GENERATING,
        }

    def _refresh_capture_controls(self) -> None:
        """只按状态切换采集控件，避免 GUI 线程推测后端重建结果。"""
        if self.capture_start_button is None:
            return
        start_allowed = (
            self._capture_state
            in {
                ManualCaptureUiState.IDLE,
                ManualCaptureUiState.COMPLETED,
                ManualCaptureUiState.FAILED,
            }
            and not self._structure_busy
        )
        self.capture_duration_combo.setEnabled(start_allowed)
        self.capture_start_button.setEnabled(start_allowed)
        self.capture_stop_button.setEnabled(
            self._capture_state is ManualCaptureUiState.RECORDING
        )
        self.capture_viewer_button.setEnabled(
            self._capture_state is ManualCaptureUiState.COMPLETED
            and self._capture_lvx2_path is not None
        )
        self.capture_compress_button.setEnabled(
            self._capture_state is ManualCaptureUiState.COMPLETED
            and self._capture_mcap_path is not None
            and not self._capture_compression_pending
        )

    def _refresh_structure_controls(self) -> None:
        """结构事务和采集冻结共用一处按钮/输入框可用性判定。"""
        locked = self._structure_busy or self._capture_locks_structure()
        self.robot_combo.setEnabled(self._model_switch_enabled and not locked)
        self.apply_robot_button.setEnabled(self._model_switch_enabled and not locked)
        self.reset_button.setEnabled(not locked)
        self.obstacle_group.setEnabled(not locked)
        self._update_terrain_parameter_state()
        if not locked:
            self._update_obstacle_parameter_state()
            self._update_delete_obstacle_button_state()

    def take_capture_request(self) -> ManualCaptureAction | None:
        """取走一个采集请求；与场景 RuntimeAction 队列保持独立。"""
        if not self._pending_capture_actions:
            return None
        return self._pending_capture_actions.popleft()

    def request_capture_start(self) -> None:
        if (
            self._capture_state
            not in {
                ManualCaptureUiState.IDLE,
                ManualCaptureUiState.COMPLETED,
                ManualCaptureUiState.FAILED,
            }
            or self._structure_busy
        ):
            return
        duration = self.capture_duration_combo.currentData()
        self._pending_capture_actions.append(ManualCaptureAction.start(duration))
        self._capture_start_pending = True
        self.capture_status_label.setText("状态：正在启用采集")
        self._refresh_structure_controls()
        self._refresh_capture_controls()

    def set_capture_recording(self, *, duration_limit_sec: int | None) -> None:
        ManualCaptureAction.start(duration_limit_sec)
        self._capture_start_pending = False
        self._capture_state = ManualCaptureUiState.RECORDING
        limit_text = "不限" if duration_limit_sec is None else f"{duration_limit_sec} 秒"
        self.capture_status_label.setText(f"状态：采集中（上限：{limit_text}）")
        self.capture_path_label.setText("输出：等待结束采集后生成")
        self._refresh_structure_controls()
        self._refresh_capture_controls()

    def request_capture_stop(self) -> None:
        if self._capture_state is ManualCaptureUiState.RECORDING:
            self._pending_capture_actions.append(ManualCaptureAction.stop())
            self.capture_stop_button.setEnabled(False)
            self.capture_status_label.setText("状态：正在结束采集")

    def set_capture_generating(self, detail: str) -> None:
        if not isinstance(detail, str) or not detail:
            raise ValueError("detail must be nonempty text")
        self._capture_state = ManualCaptureUiState.GENERATING
        self.capture_status_label.setText(f"状态：{detail}")
        self._refresh_structure_controls()
        self._refresh_capture_controls()

    def set_capture_completed(self, mcap_path: Path, lvx2_path: Path) -> None:
        if (
            not isinstance(mcap_path, Path)
            or not mcap_path.is_absolute()
            or not isinstance(lvx2_path, Path)
            or not lvx2_path.is_absolute()
        ):
            raise ValueError("completed capture paths must be absolute Paths")
        self._capture_state = ManualCaptureUiState.COMPLETED
        self._capture_mcap_path = mcap_path
        self._capture_lvx2_path = lvx2_path
        self._capture_compressed_path = None
        self._capture_compression_pending = False
        self.capture_status_label.setText("状态：采集完成，LVX2 已验证")
        self.capture_path_label.setText(f"MCAP：{mcap_path}\nLVX2：{lvx2_path}")
        self._refresh_structure_controls()
        self._refresh_capture_controls()

    def request_capture_compression(self) -> None:
        """请求主循环在线程中压缩已关闭的 MCAP，不阻塞 Qt。"""
        if (
            self._capture_state is not ManualCaptureUiState.COMPLETED
            or self._capture_mcap_path is None
            or self._capture_compression_pending
        ):
            return
        self._pending_capture_actions.append(
            ManualCaptureAction.compress_mcap(self._capture_mcap_path)
        )
        self._capture_compression_pending = True
        self.capture_status_label.setText("状态：正在离线压缩 MCAP，原始文件保留")
        self._refresh_capture_controls()

    def set_capture_compression_result(
        self,
        *,
        success: bool,
        compressed_path: Path | None,
        detail: str,
    ) -> None:
        """回显离线压缩结果；失败不影响已完成会话和 Viewer 重试。"""
        if type(success) is not bool or not isinstance(detail, str) or not detail:
            raise ValueError("compression result requires success and detail")
        if compressed_path is not None and (
            not isinstance(compressed_path, Path) or not compressed_path.is_absolute()
        ):
            raise ValueError("compressed_path must be an absolute Path or None")
        self._capture_compression_pending = False
        if self._capture_state is not ManualCaptureUiState.COMPLETED:
            return
        if success and compressed_path is not None:
            self._capture_compressed_path = compressed_path
            self.capture_status_label.setText(f"状态：MCAP 已完成 Zstandard 离线压缩（{detail}）")
            self.capture_path_label.setText(
                f"MCAP：{self._capture_mcap_path}\n压缩：{compressed_path}\nLVX2：{self._capture_lvx2_path}"
            )
        else:
            self.capture_status_label.setText(f"状态：MCAP 压缩失败，可重试：{detail}")
        self._refresh_capture_controls()

    def set_capture_failed(self, detail: str, *, output_dir: Path | None = None) -> None:
        if not isinstance(detail, str) or not detail:
            raise ValueError("detail must be nonempty text")
        if output_dir is not None and (not isinstance(output_dir, Path) or not output_dir.is_absolute()):
            raise ValueError("output_dir must be an absolute Path or None")
        self._capture_state = ManualCaptureUiState.FAILED
        self._capture_start_pending = False
        self._capture_mcap_path = None
        self._capture_lvx2_path = None
        self._capture_compressed_path = None
        self._capture_compression_pending = False
        self.capture_status_label.setText(f"状态：失败：{detail}")
        self.capture_path_label.setText(
            "输出：--" if output_dir is None else f"输出目录：{output_dir}"
        )
        self._refresh_structure_controls()
        self._refresh_capture_controls()

    def request_livox_viewer_import(self) -> None:
        if (
            self._capture_state is ManualCaptureUiState.COMPLETED
            and self._capture_lvx2_path is not None
        ):
            self._pending_capture_actions.append(
                ManualCaptureAction.open_viewer(self._capture_lvx2_path)
            )
            self.capture_viewer_button.setEnabled(False)
            self.capture_status_label.setText("状态：正在打开 Livox Viewer")

    def set_capture_viewer_result(self, *, success: bool, detail: str) -> None:
        """回显 Viewer 导入结果，失败时保留已验证 LVX2 供用户重试。"""
        if type(success) is not bool or not isinstance(detail, str) or not detail:
            raise ValueError("Viewer result requires a boolean and nonempty detail")
        if self._capture_state is not ManualCaptureUiState.COMPLETED:
            return
        self.capture_status_label.setText(
            "状态：Livox Viewer 已打开文件" if success else f"状态：导入失败，可重试：{detail}"
        )
        self._refresh_capture_controls()

    def update_rc_status(self, snapshot: object, *, source_snapshot: object | None = None) -> None:
        """显示 RC by-id、频率、年龄、原始通道、归一化命令和故障。"""
        if self.rc_status_label is None:
            return
        path = getattr(snapshot, "path", None)
        command = getattr(snapshot, "command", None)
        channels = getattr(snapshot, "last_channels", None)
        source = getattr(source_snapshot, "active_source", None)
        reason = getattr(snapshot, "failure_reason", None)
        text = (
            f"遥控器：{path or '未启用'} | {getattr(snapshot, 'actual_hz', None) or 0.0:.1f} Hz "
            f"| age {getattr(snapshot, 'last_frame_age_sec', None) or 0.0:.3f}s\n"
            f"ch1/ch3/ch6={channels[0]}/{channels[2]}/{channels[5]} "
            f"v/w={getattr(command, 'linear_velocity_m_s', 0.0):.2f}/"
            f"{getattr(command, 'angular_velocity_rad_s', 0.0):.2f} "
            f"unlock={'是' if getattr(command, 'unlocked', False) else '否'} "
            f"source={source or '无'} fault={reason or getattr(command, 'reason', '无')}"
            if isinstance(channels, tuple) and len(channels) == 16 and command is not None
            else "遥控器：等待有效 SBUS 帧"
        )
        fell_back_to_keyboard = False
        if (
            command is not None
            and not getattr(command, "active", False)
            and self.control_source_combo is not None
            and self.control_source_combo.currentData() == "rc"
        ):
            keyboard_index = self.control_source_combo.findData("keyboard")
            if keyboard_index >= 0:
                self.control_source_combo.setCurrentIndex(keyboard_index)
                fell_back_to_keyboard = True
        if fell_back_to_keyboard:
            text += "；遥控器未就绪，已退回键盘（先将 CH6 拨到低位再拨到高位）"
        self.rc_status_label.setText(text)

    def set_rc_available(self, available: bool) -> None:
        """仅在串口 worker 已启动时保留遥控器控制源，避免无输入的假接管。"""
        if type(available) is not bool:
            raise ValueError("rc availability must be a bool")
        if available or self.control_source_combo is None:
            return
        rc_index = self.control_source_combo.findData("rc")
        if rc_index >= 0:
            self.control_source_combo.removeItem(rc_index)
        if self.rc_status_label is not None:
            self.rc_status_label.setText("遥控器：未启用（请使用 --rc-port 启动）")
        if self.control_owner_label is not None:
            self.control_owner_label.setText("当前所有者：键盘（遥控器未启用）")

    def _control_source_selection_changed(self, _index: int) -> None:
        """在下一次仲裁快照到达前，明确所选 source 仍未获得实际控制权。"""
        if self.control_owner_label is None or self.control_source_combo is None:
            return
        source = str(self.control_source_combo.currentData())
        label = {"keyboard": "键盘", "rc": "遥控器", "external": "外部命令"}.get(source, source)
        self.control_owner_label.setText(f"当前所有者：{label}（等待接管）")

    def update_control_owner(self, source_snapshot: object | None) -> None:
        """用 Command 仲裁器快照投影实际 owner，避免 GUI 选择状态伪称已接管。"""
        if self.control_owner_label is None:
            return
        source = getattr(source_snapshot, "active_source", None)
        label = {"keyboard": "键盘", "rc": "遥控器", "external": "外部命令"}.get(source)
        if label is not None:
            self.control_owner_label.setText(f"当前所有者：{label}")
            return
        selected = (
            "键盘"
            if self.control_source_combo is None
            else {"keyboard": "键盘", "rc": "遥控器", "external": "外部命令"}.get(
                str(self.control_source_combo.currentData()), "未知"
            )
        )
        reason = getattr(source_snapshot, "failure_reason", None)
        detail = f"；故障：{reason}" if isinstance(reason, str) and reason else ""
        self.control_owner_label.setText(f"当前所有者：无（已选择：{selected}{detail}）")

    def request_exit(self) -> None:
        self._should_exit = True

    def _enqueue_structural_action(self, action: RuntimeAction) -> None:
        """把结构动作放入 FIFO，确保同一帧多次请求不会互相覆盖。"""
        if self._capture_locks_structure():
            return
        self._pending_actions.append(action)
        self.set_structure_busy(True, "等待应用")

    def request_reset(self) -> None:
        if self._structure_busy and not self._pending_actions:
            return
        self._enqueue_structural_action(ResetRobotAction())

    def request_robot_switch(self) -> None:
        """把下拉框中的车型保存为一次性应用请求。"""
        if not self._model_switch_enabled or (self._structure_busy and not self._pending_actions):
            return
        self._enqueue_structural_action(SwitchRobotAction(str(self.robot_combo.currentData())))

    def request_terrain_switch(self) -> None:
        """把当前场地控件值保存为一次性应用请求。"""
        if not self._terrain_switch_enabled or (self._structure_busy and not self._pending_actions):
            return
        self._enqueue_structural_action(
            SwitchTerrainAction(
                TerrainSelection(
                    terrain_model=str(self.terrain_combo.currentData()),
                    slope_deg=self.slope_spin.value(),
                    golf_seed=self.golf_seed_spin.value(),
                    golf_relief=str(self.golf_relief_combo.currentData()),
                )
            )
        )

    def request_add_obstacles(self) -> None:
        """读取障碍物控件并生成纯数据添加请求，实际 PyBullet 操作交给主循环。"""
        if self._structure_busy and not self._pending_actions:
            return
        request = ObstacleGenerationRequest(
            mode=str(self.obstacle_mode_combo.currentData()),
            shape=str(self.obstacle_shape_combo.currentData()),
            count=self.obstacle_count_spin.value(),
            seed=self.obstacle_seed_spin.value(),
            moving_speed=self.obstacle_speed_spin.value(),
            moving_ratio=self.obstacle_ratio_spin.value() / 100.0,
        )
        self._enqueue_structural_action(AddObstaclesAction(request))

    def request_delete_obstacle(self) -> None:
        """按表格选中行的逻辑 ID 删除障碍物，不依赖临时 body id。"""
        if self._structure_busy and not self._pending_actions:
            return
        logical_id = self._selected_obstacle_logical_id()
        if logical_id is None:
            return
        self._enqueue_structural_action(DeleteObstacleAction(logical_id))

    def request_clear_obstacles(self) -> None:
        """清空障碍物直接入队，不弹出会阻塞仿真循环的确认框。"""
        if self._structure_busy and not self._pending_actions:
            return
        self._enqueue_structural_action(ClearObstaclesAction())

    def _update_terrain_parameter_state(self, _index: int | None = None) -> None:
        """只启用待应用场地真正使用的参数，减少误操作。"""
        if not self._terrain_switch_enabled or self._capture_locks_structure():
            for control in (
                self.terrain_combo,
                self.slope_spin,
                self.golf_seed_spin,
                self.golf_relief_combo,
                self.apply_terrain_button,
            ):
                control.setEnabled(False)
            return
        self.terrain_combo.setEnabled(True)
        self.apply_terrain_button.setEnabled(not self._structure_busy)
        terrain_model = str(self.terrain_combo.currentData())
        self.slope_spin.setEnabled(terrain_model == "slope")
        golf_enabled = terrain_model == "golf_heightfield"
        self.golf_seed_spin.setEnabled(golf_enabled)
        self.golf_relief_combo.setEnabled(golf_enabled)

    def _update_camera_follow_state(self, _checked: bool | None = None) -> None:
        """跟随关闭时禁用视角选择，避免编辑不生效的控件。"""
        if self.camera_follow_checkbox is None or self.camera_view_combo is None:
            return
        self.camera_view_combo.setEnabled(self.camera_follow_checkbox.isChecked())

    def _update_obstacle_parameter_state(self, _index: int | None = None) -> None:
        """按生成模式启用移动参数，避免静态/全移动模式显示无效输入。"""
        mode = str(self.obstacle_mode_combo.currentData())
        self.obstacle_speed_spin.setEnabled(mode in {"moving", "mixed"})
        self.obstacle_ratio_spin.setEnabled(mode == "mixed")

    def _selected_obstacle_logical_id(self) -> int | None:
        """返回当前选中障碍物的稳定逻辑 ID；无选择时返回 None。"""
        rows = self.obstacle_table.selectionModel().selectedRows()
        if not rows:
            return None
        item = self.obstacle_table.item(rows[0].row(), 0)
        if item is None:
            return None
        value = item.data(self.QtCore.Qt.UserRole)
        return None if value is None else int(value)

    def _update_delete_obstacle_button_state(self) -> None:
        """删除按钮只在非忙碌且存在选中逻辑 ID 时可用。"""
        button = getattr(self, "delete_obstacle_button", None)
        if button is None:
            return
        button.setEnabled(not self._structure_busy and self._selected_obstacle_logical_id() is not None)

    def update_obstacle_snapshots(
        self,
        snapshots: tuple[ObstacleSnapshot, ...] | Callable[[], tuple[ObstacleSnapshot, ...]],
        *,
        force: bool = False,
    ) -> bool:
        """节流判定通过后才求值懒快照，并按逻辑 ID 恢复选中行。"""
        now = time.monotonic()
        if not force and not should_refresh_dashboard(self._last_obstacle_refresh_time, now, self.update_hz):
            return False
        if callable(snapshots):
            snapshots = snapshots()
        selected_logical_id = self._selected_obstacle_logical_id()
        self.obstacle_table.setRowCount(len(snapshots))
        selected_row: int | None = None
        for row, snapshot in enumerate(snapshots):
            values = (
                str(snapshot.logical_id),
                snapshot.mode,
                snapshot.shape,
                f"{snapshot.position[0]:.2f}, {snapshot.position[1]:.2f}, {snapshot.position[2]:.2f}",
            )
            for column, value in enumerate(values):
                item = self.obstacle_table.item(row, column)
                if item is None:
                    item = self.QtWidgets.QTableWidgetItem()
                    self.obstacle_table.setItem(row, column, item)
                item.setText(value)
                if column == 0:
                    item.setData(self.QtCore.Qt.UserRole, snapshot.logical_id)
            if snapshot.logical_id == selected_logical_id:
                selected_row = row
        # 位置列允许换行，刷新后必须按当前列宽重算每行高度。
        self.obstacle_table.resizeRowsToContents()
        self.obstacle_table.clearSelection()
        if selected_row is not None:
            self.obstacle_table.selectRow(selected_row)
        self._update_delete_obstacle_button_state()
        self._last_obstacle_refresh_time = now
        return True

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
        """显示最近一次结构操作结果；错误使用醒目颜色但不弹阻塞窗口。"""
        self.set_structure_busy(False, message, is_error=is_error)

    def set_switch_busy(self, busy: bool, message: str, *, is_error: bool = False) -> None:
        """兼容旧场景切换 API，实际委托给统一结构状态。"""
        self.set_structure_busy(busy, message, is_error=is_error)

    def set_structure_busy(self, busy: bool, message: str, *, is_error: bool = False) -> None:
        """结构事务期间禁用相关按钮，完成后按选择状态恢复删除按钮。"""
        self._structure_busy = busy
        self._switch_busy = busy
        self._refresh_structure_controls()
        self._refresh_capture_controls()
        status_label = getattr(self, "structure_status_label", None)
        if status_label is None:
            return
        status_label.setText(f"结构状态：{message}")
        status_label.setStyleSheet("color: #b00020;" if is_error else "")

    def _active_plot_label(self) -> str | None:
        """返回顶层标签栏当前可见的图表名称。"""
        label = self.tabs.tabText(self.tabs.currentIndex())
        return label if label in self.plot_canvases else None

    def _handle_tab_changed(self, _index: int) -> None:
        """切换曲线页时只标记当前页，避免一次切换触发所有图重绘。"""
        label = self._active_plot_label()
        if label is None:
            return
        self._plot_dirty_tabs.add(label)
        self._request_current_plot_draw(time.monotonic())

    def _mark_plot_data_changed(self, tab_labels: Iterable[str]) -> None:
        """推进受影响页的数据修订，并把它们加入下一次绘制集合。"""
        labels = tuple(tab_labels)
        for label in labels:
            if label not in self._plot_data_revisions:
                continue
            self._plot_data_revisions[label] += 1
        self._plot_dirty_tabs.update(labels)

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
        """清空所有折线缓存，并让当前页在门禁允许时同步空数据。"""
        self.plot_buffer.clear()
        self.interface_plot_buffer.clear()
        line_tabs = {spec.tab_label for spec in self.plot_specs}
        self._mark_plot_data_changed(line_tabs)
        self._request_current_plot_draw(time.monotonic())

    def reset_feedback_history(self) -> None:
        """车辆重载后清空旧车的平滑状态、刷新节流和曲线数据。"""
        self._smoothed_telemetry = None
        self._last_update_time = None
        self._actual_update_hz = None
        self._last_plot_update_time = None
        self._latest_interface_snapshot = None
        self._interface_status = None
        self._last_interface_status_update_time = None
        self._plot_draw_started_at.clear()
        self._plot_dirty_tabs.clear()
        self.clear_plots()
        self._interface_generation = None
        self._latest_lidar_views = {"front": None, "rear": None}
        self._latest_lidar_clouds = {"front": None, "rear": None}
        if self.show_lidar_tools:
            import numpy as np

            self.lidar_collection.set_offsets(np.empty((0, 2)))
            self.lidar_collection.set_facecolors([])
            self._set_lidar_limits(np.empty((0, 2)))
            self.lidar_front_time_text.set_text("前: --")
            self.lidar_rear_time_text.set_text("后: --")
            self._refresh_lidar_status()
            self._mark_plot_data_changed(("LiDAR点云",))

    def save_plot_snapshot(self, tab_label: str | None = None) -> Path:
        """保存当前曲线页截图到配置的图像目录。"""
        if not self.plot_specs:
            raise RuntimeError("dashboard plots are unavailable")
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
        self._request_current_plot_draw(time.monotonic())
        self._emit_layout_report_if_requested()
        if not self.window.isVisible():
            self._should_exit = True

    def _widget_global_rect(self, widget: object) -> list[int]:
        """把 Qt 控件客户区转换为可 JSON 序列化的全局逻辑矩形。"""
        point = widget.mapToGlobal(self.QtCore.QPoint(0, 0))
        return [point.x(), point.y(), widget.width(), widget.height()]

    @staticmethod
    def _matplotlib_bounds_logical_rect(
        canvas: object,
        bounds: object,
        scale: float,
    ) -> tuple[int, int, int, int]:
        """向外包围 Matplotlib 物理 bbox，保留亚像素越界和重叠证据。"""
        left = math.floor(bounds.x0 / scale)
        right = math.ceil(bounds.x1 / scale)
        top = math.floor(canvas.height() - bounds.y1 / scale)
        bottom = math.ceil(canvas.height() - bounds.y0 / scale)
        return left, top, max(1, right - left), max(1, bottom - top)

    def _legend_global_rect(self, tab_label: str) -> list[int] | None:
        """读取现有 renderer 的图例边界，不主动触发 Matplotlib 绘制。"""
        canvas = self.plot_canvases.get(tab_label)
        legend = self.plot_legends.get(tab_label)
        if canvas is None or legend is None:
            return None
        if getattr(canvas, "_draw_pending", False) or getattr(canvas, "_is_drawing", False):
            return None
        try:
            renderer = canvas.get_renderer()
            bounds = legend.get_window_extent(renderer=renderer)
            scale = float(getattr(canvas, "device_pixel_ratio", 1.0))
            if scale <= 0.0:
                return None
            left, top, width, height = self._matplotlib_bounds_logical_rect(
                canvas,
                bounds,
                scale,
            )
            point = canvas.mapToGlobal(self.QtCore.QPoint(left, top))
            return [point.x(), point.y(), width, height]
        except Exception:
            return None

    def _axis_global_rect(self, tab_label: str) -> list[int] | None:
        """读取 active axes 的全局逻辑矩形，供外部门禁检查画布覆盖率。"""
        canvas = self.plot_canvases.get(tab_label)
        axis = self.plot_axes.get(tab_label)
        if canvas is None or axis is None:
            return None
        try:
            bounds = axis.get_window_extent(renderer=canvas.get_renderer())
            scale = float(getattr(canvas, "device_pixel_ratio", 1.0))
            if scale <= 0.0:
                return None
            left, top, width, height = self._matplotlib_bounds_logical_rect(
                canvas,
                bounds,
                scale,
            )
            point = canvas.mapToGlobal(self.QtCore.QPoint(left, top))
            return [point.x(), point.y(), width, height]
        except Exception:
            return None

    def _plot_artist_global_rect(
        self,
        tab_label: str,
        artist: object,
    ) -> dict[str, object] | None:
        """把非空 Matplotlib 文本 artist 转成 Qt 全局逻辑矩形。"""
        canvas = self.plot_canvases.get(tab_label)
        if canvas is None or not artist.get_visible():
            return None
        text = str(artist.get_text())
        if not text.strip():
            return None
        try:
            renderer = canvas.get_renderer()
            bounds = artist.get_window_extent(renderer=renderer)
            scale = float(getattr(canvas, "device_pixel_ratio", 1.0))
            if scale <= 0.0:
                return None
            left, top, width, height = self._matplotlib_bounds_logical_rect(
                canvas,
                bounds,
                scale,
            )
            point = canvas.mapToGlobal(self.QtCore.QPoint(left, top))
            return {
                "text": text,
                "rect": [point.x(), point.y(), width, height],
            }
        except Exception:
            return None

    def _plot_artist_rects(self, tab_label: str) -> dict[str, object] | None:
        """报告坐标轴标题、标签与 offset，供外部门禁独立检查包含和重叠。"""
        axis = self.plot_axes.get(tab_label)
        if axis is None:
            return None
        artists = {
            "title": axis.title,
            "x_label": axis.xaxis.label,
            "y_label": axis.yaxis.label,
            "x_offset": axis.xaxis.get_offset_text(),
            "y_offset": axis.yaxis.get_offset_text(),
        }
        return {
            name: self._plot_artist_global_rect(tab_label, artist)
            for name, artist in artists.items()
        }

    def _plot_tick_rects(self, tab_label: str) -> dict[str, list[object]] | None:
        """报告当前实际可见的横纵轴 tick 文本，禁止窄画布静默遮挡。"""
        axis = self.plot_axes.get(tab_label)
        if axis is None:
            return None
        result: dict[str, list[object]] = {"x": [], "y": []}
        for axis_name, artists in (
            ("x", axis.get_xticklabels()),
            ("y", axis.get_yticklabels()),
        ):
            for artist in artists:
                value = self._plot_artist_global_rect(tab_label, artist)
                if value is not None:
                    result[axis_name].append(value)
        return result

    def _legend_text_rects(self, tab_label: str) -> list[object] | None:
        """报告图例内部文字，供 verifier 检查图例自身包含和互斥。"""
        legend = self.plot_legends.get(tab_label)
        if legend is None:
            return None
        return [
            value
            for artist in legend.get_texts()
            if (value := self._plot_artist_global_rect(tab_label, artist)) is not None
        ]

    def _qt_text_global_rect(self, widget: object) -> dict[str, object] | None:
        """按控件真实字体计算其显示文字所需矩形和可用内容矩形。"""
        QtCore = self.QtCore
        QtWidgets = self.QtWidgets
        if isinstance(widget, QtWidgets.QComboBox):
            text = widget.currentText()
        elif isinstance(widget, QtWidgets.QAbstractSpinBox):
            text = widget.text()
        elif isinstance(widget, (QtWidgets.QAbstractButton, QtWidgets.QLabel)):
            text = widget.text()
        else:
            return None
        text = str(text)
        if not text.strip():
            return None

        container = widget.contentsRect()
        if isinstance(widget, (QtWidgets.QComboBox, QtWidgets.QAbstractSpinBox)):
            # 为下拉箭头或步进按钮预留原生样式区域，避免数字被覆盖。
            container = container.adjusted(4, 1, -24, -1)
            alignment = QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter
        elif isinstance(widget, QtWidgets.QAbstractButton):
            icon_width = 0 if widget.icon().isNull() else widget.iconSize().width() + 6
            container = container.adjusted(icon_width + 4, 1, -4, -1)
            alignment = QtCore.Qt.AlignCenter
        else:
            # QLabel 的字体 bbox 可触及 contentsRect 上下边，不能把合法像素误报为裁切。
            container = container.adjusted(2, 0, -2, 0)
            alignment = widget.alignment()
        if container.width() <= 0 or container.height() <= 0:
            return None

        flags = int(alignment | QtCore.Qt.TextSingleLine)
        bounds = widget.fontMetrics().boundingRect(container, flags, text)
        text_point = widget.mapToGlobal(bounds.topLeft())
        container_point = widget.mapToGlobal(container.topLeft())
        return {
            "text": text,
            "rect": [text_point.x(), text_point.y(), bounds.width(), bounds.height()],
            "container_rect": [
                container_point.x(),
                container_point.y(),
                container.width(),
                container.height(),
            ],
        }

    def _tab_qt_text_rect(self, tab_index: int) -> dict[str, object] | None:
        """计算当前页签完整文字所需矩形，不能依赖 Qt 自动裁切。"""
        tab_bar = self.tabs.tabBar()
        text = self.tabs.tabText(tab_index)
        container = tab_bar.tabRect(tab_index).adjusted(6, 2, -6, -2)
        if not text.strip() or container.width() <= 0 or container.height() <= 0:
            return None
        flags = int(self.QtCore.Qt.AlignCenter | self.QtCore.Qt.TextSingleLine)
        bounds = tab_bar.fontMetrics().boundingRect(container, flags, text)
        text_point = tab_bar.mapToGlobal(bounds.topLeft())
        container_point = tab_bar.mapToGlobal(container.topLeft())
        return {
            "text": text,
            "rect": [text_point.x(), text_point.y(), bounds.width(), bounds.height()],
            "container_rect": [
                container_point.x(),
                container_point.y(),
                container.width(),
                container.height(),
            ],
        }

    def _tab_scroll_button_rects(self) -> dict[str, list[int]]:
        """读取 Qt 标签栏真实左右滚动按钮，禁止验收脚本猜测固定坐标。"""
        tab_bar = self.tabs.tabBar()
        result: dict[str, list[int]] = {}
        directions = {
            self.QtCore.Qt.LeftArrow: "left",
            self.QtCore.Qt.RightArrow: "right",
        }
        for button in tab_bar.findChildren(self.QtWidgets.QToolButton):
            name = directions.get(button.arrowType())
            if name is not None and button.isVisibleTo(tab_bar):
                result[name] = self._widget_global_rect(button)
        return result

    def _ensure_control_fully_visible(self, widget: object) -> None:
        """按控件真实外框补偿 Qt 滚动结果，保证 QRect 完整落入 viewport。"""
        margin = DASHBOARD_CRITICAL_CONTROL_SCROLL_MARGIN
        scrollbar = self.control_scroll.verticalScrollBar()
        viewport = self.control_scroll.viewport()
        self.control_scroll.ensureWidgetVisible(widget, margin, margin)
        for _attempt in range(3):
            top_left = widget.mapTo(viewport, self.QtCore.QPoint(0, 0))
            top = top_left.y()
            bottom = top + widget.height()
            if top < margin:
                delta = top - margin
            elif bottom > viewport.height() - margin:
                delta = bottom - (viewport.height() - margin)
            else:
                return
            previous = scrollbar.value()
            scrollbar.setValue(
                max(scrollbar.minimum(), min(scrollbar.maximum(), previous + delta))
            )
            if scrollbar.value() == previous:
                return

    def _layout_report(self) -> dict[str, object] | None:
        """构造当前页只读布局快照；绘图未完成时等待下一次事件循环。"""
        tab_index = self.tabs.currentIndex()
        page = self.tabs.currentWidget()
        if tab_index < 0 or page is None:
            return None
        tab_label = self.tabs.tabText(tab_index)
        canvas = self.plot_canvases.get(tab_label)
        if canvas is not None and (
            getattr(canvas, "_draw_pending", False)
            or getattr(canvas, "_is_drawing", False)
        ):
            return None
        page_kind = "content" if tab_label in DASHBOARD_CONTENT_TAB_LABELS else "plot"
        if page_kind == "content":
            required_plot_buttons: list[str] = []
            rendered_data_revision: int | None = None
        elif tab_label == "LiDAR点云":
            required_plot_buttons = ["保存当前图"]
            rendered_data_revision = self._plot_rendered_revisions.get(tab_label, -1)
        else:
            required_plot_buttons = ["清空曲线", "保存当前图"]
            rendered_data_revision = self._plot_rendered_revisions.get(tab_label, -1)
        if page_kind == "plot":
            current_data_revision = self._plot_data_revisions.get(tab_label, -1)
            if (
                rendered_data_revision < 0
                or rendered_data_revision != current_data_revision
            ):
                return None
        plot_button_widgets = (
            {
                button.text(): button
                for button in page.findChildren(self.QtWidgets.QPushButton)
                if button.isVisibleTo(page)
            }
            if page_kind == "plot"
            else {}
        )
        plot_buttons = {
            name: self._widget_global_rect(button)
            for name, button in plot_button_widgets.items()
        }
        legend_rect = self._legend_global_rect(tab_label)
        if page_kind == "plot" and legend_rect is None:
            return None
        plot_artist_rects = self._plot_artist_rects(tab_label)
        plot_tick_rects = self._plot_tick_rects(tab_label)
        axes_rect = self._axis_global_rect(tab_label)
        legend_text_rects = (
            self._legend_text_rects(tab_label) if legend_rect is not None else None
        )
        if page_kind == "plot" and (
            plot_artist_rects is None
            or axes_rect is None
            or any(plot_artist_rects[name] is None for name in ("title", "x_label", "y_label"))
            or plot_tick_rects is None
            or any(not plot_tick_rects[axis_name] for axis_name in ("x", "y"))
        ):
            return None
        if tab_label == "接口状态":
            content_widget_rects = {
                "接口状态滚动区": self._widget_global_rect(self.interface_scroll)
            }
        elif tab_label == "障碍物":
            content_widget_rects = {
                "障碍物表格": self._widget_global_rect(self.obstacle_table)
            }
        else:
            content_widget_rects = {}

        tab_scroll_button_rects = self._tab_scroll_button_rects()
        if set(tab_scroll_button_rects) not in (set(), {"left", "right"}):
            return None
        tab_qt_text = self._tab_qt_text_rect(tab_index)
        plot_button_qt_texts = {
            name: self._qt_text_global_rect(button)
            for name, button in plot_button_widgets.items()
        }
        if tab_qt_text is None or any(
            value is None for value in plot_button_qt_texts.values()
        ):
            return None

        # 验收模式逐个滚动关键控件，记录其真正完全进入视口时的几何。
        scrollbar = self.control_scroll.verticalScrollBar()
        original_scroll_value = scrollbar.value()
        viewport_rect = self._widget_global_rect(self.control_scroll.viewport())
        critical_widgets = {
            "暂停": self.pause_button,
            "复位车辆": self.reset_button,
            "线速度": self.linear_spin,
            "角速度": self.angular_spin,
            "退出": self.quit_button,
            "车型": self.robot_combo,
            "应用车型": self.apply_robot_button,
            "场地": self.terrain_combo,
            "坡度": self.slope_spin,
            "场地随机种子": self.golf_seed_spin,
            "起伏": self.golf_relief_combo,
            "应用场地": self.apply_terrain_button,
            "障碍模式": self.obstacle_mode_combo,
            "障碍形状": self.obstacle_shape_combo,
            "障碍数量": self.obstacle_count_spin,
            "障碍随机种子": self.obstacle_seed_spin,
            "障碍速度": self.obstacle_speed_spin,
            "障碍移动占比": self.obstacle_ratio_spin,
            "添加障碍": self.add_obstacles_button,
            "删除选中": self.delete_obstacle_button,
            "清空障碍": self.clear_obstacles_button,
            "结构状态": self.structure_status_label,
        }
        critical_controls: dict[str, object] = {}
        critical_control_qt_texts: dict[str, object] = {}
        try:
            for name in DASHBOARD_REQUIRED_CRITICAL_CONTROLS:
                widget = critical_widgets[name]
                self._ensure_control_fully_visible(widget)
                qt_text = self._qt_text_global_rect(widget)
                if qt_text is None:
                    return None
                critical_controls[name] = {
                    "rect": self._widget_global_rect(widget),
                    "viewport_rect": self._widget_global_rect(
                        self.control_scroll.viewport()
                    ),
                    "scroll_value": scrollbar.value(),
                }
                critical_control_qt_texts[name] = qt_text
        finally:
            scrollbar.setValue(original_scroll_value)

        screen = self.window.screen()
        screen_rect = None
        if screen is not None:
            geometry = screen.geometry()
            screen_rect = [
                geometry.x(),
                geometry.y(),
                geometry.width(),
                geometry.height(),
            ]
        return {
            "report_version": DASHBOARD_LAYOUT_REPORT_VERSION,
            "tab_index": tab_index,
            "tab_count": self.tabs.count(),
            "tab_label": tab_label,
            "tab_order": [
                self.tabs.tabText(index) for index in range(self.tabs.count())
            ],
            "page_kind": page_kind,
            "required_plot_buttons": required_plot_buttons,
            "rendered_data_revision": rendered_data_revision,
            "device_pixel_ratio": float(self.window.devicePixelRatioF()),
            "screen_rect": screen_rect,
            "window_rect": self._widget_global_rect(self.window),
            "title_rect": self._widget_global_rect(self.title_label),
            "tabs_rect": self._widget_global_rect(self.tabs),
            "tab_bar_rect": self._widget_global_rect(self.tabs.tabBar()),
            "tab_scroll_button_rects": tab_scroll_button_rects,
            "controls_rect": self._widget_global_rect(self.control_scroll),
            "page_rect": self._widget_global_rect(page),
            "canvas_rect": None if canvas is None else self._widget_global_rect(canvas),
            "axes_rect": axes_rect,
            "legend_rect": legend_rect,
            "legend_text_rects": legend_text_rects,
            "plot_button_rects": plot_buttons,
            "content_widget_rects": content_widget_rects,
            "plot_artist_rects": plot_artist_rects,
            "plot_tick_rects": plot_tick_rects,
            "qt_text_rects": {
                "tab": tab_qt_text,
                "plot_buttons": plot_button_qt_texts,
                "critical_controls": critical_control_qt_texts,
            },
            "control_viewport_rect": viewport_rect,
            "control_content_rect": self._widget_global_rect(self.control_content),
            "control_scroll_range": [scrollbar.minimum(), scrollbar.maximum()],
            "critical_control_rects": critical_controls,
        }

    def _emit_layout_report_if_requested(self) -> None:
        """验收模式只追加新访问的内容页或完成新数据绘制的图表页。"""
        if self._layout_report_path is None:
            return
        tab_index = self.tabs.currentIndex()
        report = self._layout_report()
        if report is None:
            return
        revision = report["rendered_data_revision"]
        if revision is None:
            if tab_index == self._last_layout_report_tab_index:
                return
        elif self._last_layout_report_revisions.get(tab_index, -1) >= revision:
            return
        self._layout_report_path.parent.mkdir(parents=True, exist_ok=True)
        with self._layout_report_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(report, ensure_ascii=False, sort_keys=True))
            handle.write("\n")
        self._last_layout_report_tab_index = tab_index
        if revision is not None:
            self._last_layout_report_revisions[tab_index] = revision

    def current_command(self) -> DashboardCommand:
        keys = self._pressed_keys
        structural_action = self._pending_actions.popleft() if self._pending_actions else None
        if self.camera_follow_checkbox is None or self.camera_view_combo is None:
            camera_follow_enabled = self._camera_follow_enabled
            camera_follow_view = self._camera_follow_view
        else:
            camera_follow_enabled = self.camera_follow_checkbox.isChecked()
            camera_follow_view = str(self.camera_view_combo.currentData())
        paused = self._paused or self._normalize_key(self.QtCore.Qt.Key_Space) in keys
        control_source = (
            "keyboard"
            if self.control_source_combo is None
            else str(self.control_source_combo.currentData())
        )
        if paused:
            return DashboardCommand(
                0.0,
                0.0,
                paused=True,
                should_exit=self._should_exit,
                structural_action=structural_action,
                camera_follow_enabled=camera_follow_enabled,
                camera_follow_view=camera_follow_view,
                control_source=control_source,
            )
        if control_source != "keyboard":
            return DashboardCommand(
                0.0,
                0.0,
                should_exit=self._should_exit,
                structural_action=structural_action,
                camera_follow_enabled=camera_follow_enabled,
                camera_follow_view=camera_follow_view,
                control_source=control_source,
            )
        linear = 0.0
        angular = 0.0
        if (
            self._normalize_key(self.QtCore.Qt.Key_Up) in keys
            or self._normalize_key(self.QtCore.Qt.Key_W) in keys
        ):
            linear += self.linear_spin.value()
        if (
            self._normalize_key(self.QtCore.Qt.Key_Down) in keys
            or self._normalize_key(self.QtCore.Qt.Key_S) in keys
        ):
            linear -= self.linear_spin.value()
        if (
            self._normalize_key(self.QtCore.Qt.Key_Left) in keys
            or self._normalize_key(self.QtCore.Qt.Key_A) in keys
        ):
            angular += self.angular_spin.value()
        if (
            self._normalize_key(self.QtCore.Qt.Key_Right) in keys
            or self._normalize_key(self.QtCore.Qt.Key_D) in keys
        ):
            angular -= self.angular_spin.value()
        return DashboardCommand(
            linear,
            angular,
            paused=paused,
            should_exit=self._should_exit,
            structural_action=structural_action,
            camera_follow_enabled=camera_follow_enabled,
            camera_follow_view=camera_follow_view,
            control_source=control_source,
        )

    def update(self, telemetry: RobotTelemetry) -> bool:
        """按较低频率刷新显示，并对物理反馈做一阶平滑。"""
        now = time.monotonic()
        plot_updated = self._maybe_update_plots(telemetry, now)
        if not should_refresh_dashboard(self._last_update_time, now, self.update_hz):
            return plot_updated
        previous_update_time = self._last_update_time
        self._last_update_time = now
        if previous_update_time is not None and now > previous_update_time:
            self._actual_update_hz = 1.0 / (now - previous_update_time)
        if not self.developer_diagnostics_enabled:
            return True
        display_telemetry = smooth_telemetry(self._smoothed_telemetry, telemetry, self.smoothing_alpha)
        self._smoothed_telemetry = display_telemetry
        for name, value in dashboard_rows(display_telemetry):
            self.labels[name].setText(value)
        return True

    @property
    def actual_update_hz(self) -> float | None:
        """返回最近两次实际 UI 刷新的频率，不把配置目标当成观测值。"""
        return self._actual_update_hz

    def update_resource_status(self, snapshot: ResourceSnapshot) -> None:
        """渲染资源采样器的只读 1 Hz 快照，Qt 线程不自行读取系统状态。"""
        if type(snapshot) is not ResourceSnapshot:
            raise ValueError("snapshot must be an exact ResourceSnapshot")
        if self.resource_layout is None:
            return
        for process in snapshot.processes:
            label = self.resource_labels.get(process.name)
            if label is None:
                label = self.QtWidgets.QLabel()
                label.setWordWrap(True)
                label.setTextInteractionFlags(self.QtCore.Qt.TextSelectableByMouse)
                self.resource_labels[process.name] = label
                self.resource_layout.addRow(process.name, label)
            cpu = "--" if process.cpu_percent is None else f"{process.cpu_percent:.1f}%"
            rss_mib = process.rss_bytes / (1024.0 * 1024.0)
            label.setText(
                f"pid={process.pid} | CPU {cpu} | RSS {rss_mib:.1f} MiB | {process.state}"
            )
        for name, value in snapshot.metrics:
            label = self.resource_labels.get(name)
            if label is None:
                label = self.QtWidgets.QLabel()
                label.setWordWrap(True)
                self.resource_labels[name] = label
                self.resource_layout.addRow(name, label)
            label.setText(value)

    def update_interface_status(self, snapshot: InterfaceStatusSnapshot) -> None:
        """Qt 线程只渲染不可变快照，不访问 PyBullet、eCAL 或传输工厂。"""
        if not isinstance(snapshot, InterfaceStatusSnapshot):
            raise ValueError("snapshot must be an InterfaceStatusSnapshot")
        self._interface_status = snapshot
        self.transport_mode_label.setText(_transport_mode_text(snapshot.transport_mode))
        self.ecal_status_label.setText("eCAL 已连接" if snapshot.ecal_connected else "eCAL 未连接")

        details = [
            status.detail
            for channel in self.interface_config.channels
            if (status := snapshot.topics.get(channel.topic)) is not None and status.detail
        ]
        if self.interface_config.transport_mode == "auto" and snapshot.transport_mode == "local":
            reason = details[0] if details else "未提供回退原因"
            self.transport_detail_label.setText(f"自动回退：{reason}")
        elif details:
            self.transport_detail_label.setText(details[0])
        else:
            self.transport_detail_label.setText("--")

        for channel in self.interface_config.channels:
            row = self.interface_rows[channel.topic]
            status = snapshot.topics.get(channel.topic)
            if status is None:
                state = "error"
                row.state_label.setText(_interface_state_text(state))
                row.target_label.setText(_format_frequency(float(channel.rate_hz)))
                row.actual_label.setText("--")
                row.timestamp_label.setText("--")
                row.error_label.setText("1")
                row.drop_label.setText("0")
                row.detail_label.setText(f"状态快照缺少话题：{channel.topic}")
            else:
                state = status.state
                row.state_label.setText(_interface_state_text(state))
                row.target_label.setText(_format_frequency(status.target_hz))
                row.actual_label.setText(_format_frequency(status.actual_hz))
                row.timestamp_label.setText(_format_timestamp(status.latest_timestamp_ns))
                row.error_label.setText(str(status.error_count))
                row.drop_label.setText(str(status.dropped_count))
                row.detail_label.setText(status.detail or "--")
            row.state_label.setStyleSheet(f"color: {INTERFACE_STATE_COLORS.get(state, '#5f6368')}; font-weight: 600;")

        command_row = self.interface_rows[self.interface_config.wheel_command.topic]
        command_row.command_state_label.setText(_interface_state_text(snapshot.command.state))
        command_row.command_state_label.setStyleSheet(
            f"color: {INTERFACE_STATE_COLORS.get(snapshot.command.state, '#5f6368')}; font-weight: 600;"
        )
        command_row.command_frequency_label.setText(_format_frequency(snapshot.command.valid_hz))
        command_row.command_timestamp_label.setText(_format_timestamp(snapshot.command.latest_timestamp_ns))

        wheel_row = self.interface_rows[self.interface_config.wheel_state.topic]
        if snapshot.wheel_state is None:
            wheel_row.drive_values_label.setText("--")
            wheel_row.steering_values_label.setText("--")
        else:
            wheel_row.drive_values_label.setText(
                _format_array(snapshot.wheel_state.drive_wheel_speed_rad_s, "rad/s")
            )
            wheel_row.steering_values_label.setText(
                _format_array(snapshot.wheel_state.steering_wheel_angle_rad, "rad")
            )

    def _rebuild_interface_line_artists(self, robot_model: str) -> None:
        """车型变化时只替换轮组 Line2D 和 Legend，不重建画布或坐标轴。"""
        for key in self._interface_line_keys:
            line = self.plot_lines.pop(key, None)
            if line is not None:
                line.remove()
        self._interface_line_keys.clear()

        self.interface_plot_specs = interface_chart_specs(get_robot_model(robot_model))
        legacy_specs = dashboard_plot_specs()
        self.plot_specs = [*legacy_specs, *self.interface_plot_specs]
        self._plot_spec_by_label = {
            spec.tab_label: spec for spec in self.plot_specs
        }
        for spec in self.interface_plot_specs:
            axis = self.plot_axes.get(spec.tab_label)
            if axis is None:
                continue
            old_legend = self.plot_legends.pop(spec.tab_label, None)
            if old_legend is not None:
                old_legend.remove()
            handles = []
            for line_spec in spec.lines:
                line = axis.plot([], [], label=line_spec.label)[0]
                self.plot_lines[line_spec.key] = line
                self._interface_line_keys.add(line_spec.key)
                handles.append(line)
            self.plot_legends[spec.tab_label] = self._create_plot_legend(
                axis,
                handles,
                [line_spec.label for line_spec in spec.lines],
            )
            if spec.tab_label in self.no_steering_texts:
                self.no_steering_texts[spec.tab_label].set_text(
                    "" if spec.lines else "不适用于差速车型"
                )
        self._interface_robot_model = robot_model
        self._set_steering_tab_state(robot_model)

    def _reset_interface_generation(self, snapshot: InterfaceDashboardSnapshot | object) -> None:
        """代际切换时原子丢弃旧业务、质量基线和 LiDAR 最后帧。"""
        self.interface_plot_buffer.clear()
        self._latest_lidar_views = {"front": None, "rear": None}
        self._latest_lidar_clouds = {"front": None, "rear": None}
        self._last_interface_status_update_time = None
        if snapshot.robot_model != self._interface_robot_model:
            self._rebuild_interface_line_artists(snapshot.robot_model)
        generation = getattr(snapshot, "generation", None)
        if generation is None:
            generation = getattr(snapshot, "world_generation", None)
        if type(generation) is not int:
            raise ValueError("interface snapshot has no generation")
        self._interface_generation = generation
        self._mark_plot_data_changed(
            {spec.tab_label for spec in self.interface_plot_specs} | {"LiDAR点云"}
        )

    def _capture_latest_lidar_views(self, snapshot: InterfaceDashboardSnapshot) -> bool:
        """仅替换各雷达更新的成功帧，失败或旧帧继续显示上次成功结果。"""
        changed = False
        for side, cloud, view in (
            ("front", snapshot.lidar_front, snapshot.lidar_front_view),
            ("rear", snapshot.lidar_rear, snapshot.lidar_rear_view),
        ):
            previous = self._latest_lidar_views[side]
            if view is not None and (
                previous is None or view.timestamp_ns > previous.timestamp_ns
            ):
                self._latest_lidar_views[side] = view
                self._latest_lidar_clouds[side] = cloud
                changed = True
        self._refresh_lidar_status()
        return changed

    def _refresh_lidar_status(self) -> None:
        """在控制区给出点云是否到达及当前可导出点数。"""
        if self.lidar_status_label is None:
            return
        front = self._latest_lidar_clouds["front"]
        rear = self._latest_lidar_clouds["rear"]
        if front is None and rear is None:
            self.lidar_status_label.setText("点云：等待前/后雷达帧")
            return
        front_text = "--" if front is None else str(front.point_num)
        rear_text = "--" if rear is None else str(rear.point_num)
        self.lidar_status_label.setText(
            f"点云：前 {front_text} 点，后 {rear_text} 点；可在“LiDAR点云”页导出 PCD/PLY"
        )

    def update_interface_snapshot(self, snapshot: InterfaceDashboardSnapshot) -> None:
        """消费一次组合快照，缓存所有新数据但只请求当前图绘制。"""
        if type(snapshot) is not InterfaceDashboardSnapshot:
            raise ValueError("snapshot must be an exact InterfaceDashboardSnapshot")
        now = time.monotonic()
        if (
            snapshot.generation != self._interface_generation
            or snapshot.robot_model != self._interface_robot_model
        ):
            self._reset_interface_generation(snapshot)
        self._latest_interface_snapshot = snapshot
        self._interface_status = snapshot.status
        if should_refresh_dashboard(
            self._last_interface_status_update_time,
            now,
            self.update_hz,
        ):
            self.update_interface_status(snapshot.status)
            self._last_interface_status_update_time = now
        space = self._normalize_key(self.QtCore.Qt.Key_Space)
        paused = self._paused or space in self._pressed_keys
        interface_labels = {
            spec.tab_label for spec in self.interface_plot_specs
        }
        if interface_labels.intersection(self.plot_canvases):
            changed = self.interface_plot_buffer.append(snapshot, paused=paused)
            self._mark_plot_data_changed(changed)
        if not paused and self._capture_latest_lidar_views(snapshot):
            self._mark_plot_data_changed(("LiDAR点云",))
        self._request_current_plot_draw(now)

    def update_v2_chart_snapshot(self, snapshot: object) -> None:
        """将 eCAL receiver 的 v2 快照直接写入企业图表缓存。"""
        from slope_sim.interfaces.v2.dashboard_snapshot import V2DashboardSnapshot

        if type(snapshot) is not V2DashboardSnapshot:
            raise ValueError("snapshot must be an exact V2DashboardSnapshot")
        if snapshot.robot_model == "unknown":
            return
        now = time.monotonic()
        if (
            snapshot.world_generation != self._interface_generation
            or snapshot.robot_model != self._interface_robot_model
        ):
            self._reset_interface_generation(snapshot)
        space = self._normalize_key(self.QtCore.Qt.Key_Space)
        paused = self._paused or space in self._pressed_keys
        interface_labels = {
            spec.tab_label for spec in self.interface_plot_specs
        }
        if interface_labels.intersection(self.plot_canvases):
            changed = self.interface_plot_buffer.append_v2(snapshot, paused=paused)
            self._mark_plot_data_changed(changed)
        self._request_current_plot_draw(now)

    def _maybe_update_plots(self, telemetry: RobotTelemetry, now: float) -> bool:
        """按独立频率更新曲线，避免每个物理步都重绘。"""
        legacy_labels = {
            spec.tab_label for spec in dashboard_plot_specs()
        }
        if not legacy_labels.intersection(self.plot_canvases):
            return self._request_current_plot_draw(now)
        if not should_refresh_dashboard(self._last_plot_update_time, now, self.plot_update_hz):
            return self._request_current_plot_draw(now)
        self.plot_buffer.append(telemetry)
        self._last_plot_update_time = now
        self._mark_plot_data_changed(
            spec.tab_label for spec in dashboard_plot_specs()
        )
        self._request_current_plot_draw(now)
        return True

    def _request_current_plot_draw(self, now: float) -> bool:
        """按可见页、绘制冷却和 Qt pending 状态合并重绘请求。"""
        tab_label = self._active_plot_label()
        if tab_label is None or tab_label not in self._plot_dirty_tabs:
            return False
        if now < self._plot_next_draw_time.get(tab_label, 0.0):
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
        """只用 `set_data/set_offsets` 同步指定可见页，不创建新 artist。"""
        tab_label = tab_label or self._active_plot_label()
        if tab_label is None:
            return
        if tab_label == "LiDAR点云":
            self._apply_lidar_series()
            self._plot_rendered_revisions[tab_label] = self._plot_data_revisions[
                tab_label
            ]
            return
        spec = next(spec for spec in self.plot_specs if spec.tab_label == tab_label)
        if spec in self.interface_plot_specs:
            series = self.interface_plot_buffer.series(tab_label)
            x_field = "t"
        else:
            series = self.plot_buffer.series()
        for line_spec in spec.lines:
            if spec in self.interface_plot_specs:
                self.plot_lines[line_spec.key].set_data(
                    series[x_field],
                    series.get(line_spec.key, []),
                )
            else:
                self.plot_lines[line_spec.key].set_data(
                    series[line_spec.x_field],
                    series[line_spec.y_field],
                )
        axis = self.plot_axes[spec.tab_label]
        axis.relim()
        axis.autoscale_view()
        self._plot_rendered_revisions[tab_label] = self._plot_data_revisions[
            tab_label
        ]

    def _apply_lidar_series(self) -> None:
        """把前后最后成功帧合并为一个 base_link 俯视散点集。"""
        import numpy as np

        if not self._lidar_view_enabled:
            self.lidar_collection.set_offsets(np.empty((0, 2)))
            self.lidar_collection.set_facecolors([])
            self.lidar_front_time_text.set_text("前: 已隐藏")
            self.lidar_rear_time_text.set_text("后: 已隐藏")
            self._set_lidar_limits(np.empty((0, 2)))
            return
        points = []
        colors = []
        for side in ("front", "rear"):
            view = self._latest_lidar_views[side]
            if view is None:
                continue
            for point in view.points:
                points.append((point.x, point.y))
                colors.append(DASHBOARD_LIDAR_TAG_COLORS[point.tag])
        offsets = np.asarray(points, dtype=float) if points else np.empty((0, 2))
        self.lidar_collection.set_offsets(offsets)
        self.lidar_collection.set_facecolors(colors)
        front = self._latest_lidar_views["front"]
        rear = self._latest_lidar_views["rear"]
        self.lidar_front_time_text.set_text(
            "前: --" if front is None else f"前: {front.timestamp_ns / 1_000_000_000.0:.3f} s"
        )
        self.lidar_rear_time_text.set_text(
            "后: --" if rear is None else f"后: {rear.timestamp_ns / 1_000_000_000.0:.3f} s"
        )
        self._set_lidar_limits(offsets)

    def _set_lidar_view_enabled(self, enabled: bool) -> None:
        """切换可视化而不影响运行时传感器采集、接口消息或当前帧缓存。"""
        self._lidar_view_enabled = bool(enabled)
        self._mark_plot_data_changed(("LiDAR点云",))
        self._request_current_plot_draw(time.monotonic())

    def show_lidar_point_cloud(self, _checked: bool = False) -> None:
        """从控制区直接跳转到实时点云页，不要求用户滚动顶层标签栏。"""
        for index in range(self.tabs.count()):
            if self.tabs.tabText(index) == "LiDAR点云":
                self.tabs.setCurrentIndex(index)
                return
        raise RuntimeError("LiDAR point-cloud tab is unavailable")

    def export_current_lidar_point_clouds(self) -> tuple[Path, ...]:
        """按需导出当前前后 LiDAR 原始帧，并在界面显示绝对目录。"""
        clouds = tuple(
            cloud
            for cloud in (
                self._latest_lidar_clouds["front"],
                self._latest_lidar_clouds["rear"],
            )
            if cloud is not None
        )
        paths = export_lidar_point_clouds(self.pointcloud_export_dir, clouds)
        if self.lidar_status_label is not None:
            self.lidar_status_label.setText(
                f"点云：已导出 {len(paths)} 个 PCD/PLY 文件到 {self.pointcloud_export_dir.resolve()}"
            )
        return paths

    def _export_current_lidar_point_clouds(self) -> None:
        """把按钮动作的无帧错误转换为可见状态，不终止仿真循环。"""
        try:
            self.export_current_lidar_point_clouds()
        except ValueError:
            if self.lidar_status_label is not None:
                self.lidar_status_label.setText("点云：尚未收到可导出的前/后雷达帧")

    def _set_lidar_limits(self, offsets: object) -> None:
        """保持固定 base_link 视野；点云更新不能改变物理比例或 axes 几何。"""
        del offsets
        axis = self.plot_axes["LiDAR点云"]
        axis.set_xlim(*DASHBOARD_LIDAR_X_LIMITS)
        axis.set_ylim(*DASHBOARD_LIDAR_Y_LIMITS)

    def _dispose(self) -> None:
        """幂等释放 Qt 全局钩子、Matplotlib 回调和全部窗口资源。"""
        if self._disposed:
            return
        self._disposed = True

        if self._key_event_filter_installed:
            try:
                self.app.removeEventFilter(self._key_event_filter)
            except Exception:
                pass
            finally:
                self._key_event_filter_installed = False
        try:
            self._key_event_filter.deleteLater()
        except Exception:
            pass

        connections = tuple(self._mpl_connections)
        self._mpl_connections.clear()
        for canvas, _event_name, callback_id in connections:
            try:
                canvas.mpl_disconnect(callback_id)
            except Exception:
                pass

        canvases = list({id(canvas): canvas for canvas, _event, _cid in connections}.values())
        for canvas in getattr(self, "plot_canvases", {}).values():
            if all(existing is not canvas for existing in canvases):
                canvases.append(canvas)
        for canvas in canvases:
            figure = getattr(canvas, "figure", None)
            if figure is not None:
                try:
                    figure.clear()
                    figure.set_canvas(None)
                except Exception:
                    pass
            try:
                canvas.close()
            except Exception:
                pass
            try:
                canvas.figure = None
                canvas.deleteLater()
            except Exception:
                pass

        for mapping_name in (
            "labels",
            "interface_rows",
            "plot_figures",
            "plot_canvases",
            "plot_axes",
            "plot_lines",
            "plot_legends",
            "plot_texts",
            "plot_layouts",
        ):
            mapping = getattr(self, mapping_name, None)
            if mapping is not None:
                mapping.clear()
        for collection_name in (
            "no_steering_texts",
            "plot_buttons",
            "direction_buttons",
            "diagnostic_control_groups",
            "control_groups",
        ):
            collection = getattr(self, collection_name, None)
            if collection is not None:
                collection.clear()
        self._interface_line_keys.clear()
        self._latest_lidar_views = {"front": None, "rear": None}
        self._latest_interface_snapshot = None
        self._interface_status = None
        self.lidar_collection = None
        self.lidar_vehicle_marker = None
        self.lidar_range_ring = None
        self.lidar_front_time_text = None
        self.lidar_rear_time_text = None

        v2_dashboard_widget = getattr(self, "v2_dashboard_widget", None)
        if v2_dashboard_widget is not None:
            try:
                shutdown = getattr(v2_dashboard_widget, "shutdown", None)
                if callable(shutdown):
                    shutdown()
                v2_dashboard_widget.close()
            except Exception:
                pass
            self.v2_dashboard_widget = None

        try:
            self.window.close()
        except Exception:
            pass
        try:
            self.window.deleteLater()
        except Exception:
            pass
        try:
            self.QtCore.QCoreApplication.sendPostedEvents(
                None,
                self.QtCore.QEvent.DeferredDelete,
            )
            self.app.processEvents()
        except Exception:
            pass

    def close(self) -> None:
        """关闭 Dashboard；重复调用不会再次操作已释放对象。"""
        self._dispose()
