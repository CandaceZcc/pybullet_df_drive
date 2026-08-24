# Dashboard 手动验收脚本：打开 GUI，操作 Dashboard 控制区，并用日志确认车辆移动。
from __future__ import annotations

import argparse
from collections import Counter
from fractions import Fraction
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping, Sequence

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.dashboard import DASHBOARD_LAYOUT_REPORT_ENV, DASHBOARD_WINDOW_TITLE
from slope_sim.window_layout import (
    AmbiguousWindowError,
    OwnedX11Window,
    PYBULLET_WINDOW_TITLE,
    PYBULLET_WINDOW_TOKEN_ENV,
    Rect,
    WindowLayoutError,
    find_owned_x11_window,
    primary_display_metrics,
    read_x11_outer_geometry,
    resolve_x11_client_window,
    search_x11_window_ids,
    x11_available_geometry,
)

DEFAULT_MANUAL_PREFIX = "manual_golf_heightfield_active_steering_4wd_0_"
DASHBOARD_TAB_ORDER = (
    "接口状态",
    "障碍物",
    "轨迹",
    "速度/命令",
    "驱动命令",
    "驱动反馈",
    "转向命令",
    "转向反馈",
    "LiDAR点云",
    "RTK位置",
    "RTK航向",
    "IMU姿态",
    "轮组频率",
    "传感频率",
    "接口异常",
)
V2_DASHBOARD_TAB_ORDER = (
    "v2 eCAL",
    *(label for label in DASHBOARD_TAB_ORDER if label != "LiDAR点云"),
)
DASHBOARD_LAYOUT_REPORT_VERSION = 4
DASHBOARD_CONTENT_TABS = frozenset({"接口状态", "障碍物", "v2 eCAL"})
DASHBOARD_CONTENT_WIDGETS = {
    "接口状态": ("接口状态滚动区",),
    "障碍物": ("障碍物表格",),
}
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
DASHBOARD_LINE_PLOT_BUTTONS = ("清空曲线", "保存当前图")
DASHBOARD_LIDAR_PLOT_BUTTONS = ("保存当前图",)
DASHBOARD_PLOT_ARTISTS = ("title", "x_label", "y_label", "x_offset", "y_offset")
DASHBOARD_STABLE_LAYOUT_RECTS = (
    "tabs_rect",
    "controls_rect",
    "page_rect",
    "canvas_rect",
    "axes_rect",
)
LEGACY_PLOT_TAB_ORDER = ("data", "obstacles", "trajectory", "speed")
DEVELOPER_DIAGNOSTIC_TAB_LABEL = "开发者诊断"
DASHBOARD_TAB_SCROLL_SETTLE_SEC = 0.05
DASHBOARD_PLOT_BUTTON_SETTLE_SEC = 0.05
DASHBOARD_TAB_SCROLL_MIN_CHANGE_RATIO = 0.005
DASHBOARD_TAB_SCROLL_MAX_RESTORE_RATIO = 0.001
DASHBOARD_TAB_SCROLL_MAX_RESTORE_CLICKS = len(DASHBOARD_TAB_ORDER)
WINDOW_LAYOUT_POLL_INTERVAL_SEC = 0.05
MANUAL_PROCESS_SHUTDOWN_GRACE_SEC = 20.0
DASHBOARD_INTERACTION_STARTUP_GRACE_SEC = 20.0
DASHBOARD_LAYOUT_REPORT_TIMEOUT_SEC = 3.0
DASHBOARD_RATIO_NUMERATOR = 33
DASHBOARD_RATIO_DENOMINATOR = 100
DASHBOARD_ROOT_MARGIN_LOGICAL_PX = 8
DASHBOARD_ROOT_SPACING_LOGICAL_PX = 6
DASHBOARD_MIN_CANVAS_PAGE_WIDTH_PERCENT = 85
DASHBOARD_MIN_CANVAS_PAGE_HEIGHT_PERCENT = 70
DASHBOARD_FORMAL_MIN_CLIENT_HEIGHT = 600
DASHBOARD_COVERAGE_TOLERANCE_PHYSICAL_PX = 1


WindowGeometry = Rect


@dataclass(frozen=True)
class ManualMotionSummary:
    """一次手动 GUI 日志的移动摘要。"""

    log_path: Path
    dx: float
    max_command_linear_velocity: float
    tail_body_forward_speed: float
    max_body_forward_speed: float
    out_of_bounds: bool


@dataclass(frozen=True, slots=True)
class DashboardTabVerification:
    """汇总 15 个 Dashboard 页面截图的非空门禁。"""

    passed: bool
    visited_tabs: int
    min_nonbackground_ratio: float
    detail: str


@dataclass(frozen=True, slots=True)
class DashboardLayoutVerification:
    """汇总单个页面布局报告的边界检查。"""

    passed: bool
    detail: str


@dataclass(frozen=True, slots=True)
class DashboardLayoutReportOccurrence:
    """一条 JSONL 布局报告及其零基物理行号。"""

    line_number: int
    report: Mapping[str, object]


def display_is_available(env: Mapping[str, str] | None = None) -> bool:
    """判断当前 shell 是否提供本验收工具所需的 X11/XWayland DISPLAY。"""
    values = os.environ if env is None else env
    return bool(values.get("DISPLAY"))


def dashboard_window_ids(search_output: str) -> list[str]:
    """从 xdotool search 输出中提取真实窗口 id，忽略调试行。"""
    return [line.strip() for line in search_output.splitlines() if line.strip().isdigit()]


def newest_manual_log(log_dir: Path, *, prefix: str = DEFAULT_MANUAL_PREFIX, after: float = 0.0) -> Path | None:
    """找到脚本启动后生成的最新手动日志。"""
    candidates = [path for path in log_dir.glob(f"{prefix}*.csv") if path.stat().st_mtime >= after]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.name)


def summarize_manual_motion(log_path: Path, tail_samples: int = 60) -> ManualMotionSummary:
    """从手动日志中提取位移、命令和尾段速度，作为验收依据。"""
    frame = pd.read_csv(log_path)
    if frame.empty:
        raise ValueError(f"manual log is empty: {log_path}")
    required = {"x", "command_linear_velocity", "body_forward_speed", "out_of_bounds"}
    missing = required - set(frame.columns)
    if missing:
        raise ValueError(f"manual log missing columns: {', '.join(sorted(missing))}")
    tail = frame.tail(min(tail_samples, len(frame)))
    return ManualMotionSummary(
        log_path=log_path,
        dx=float(frame["x"].iloc[-1] - frame["x"].iloc[0]),
        max_command_linear_velocity=float(frame["command_linear_velocity"].abs().max()),
        tail_body_forward_speed=float(tail["body_forward_speed"].mean()),
        max_body_forward_speed=float(frame["body_forward_speed"].abs().max()),
        out_of_bounds=bool(frame["out_of_bounds"].any()),
    )


def motion_passed(
    summary: ManualMotionSummary,
    *,
    min_dx: float = 0.08,
    min_command: float = 0.20,
    min_peak_speed: float = 0.08,
) -> bool:
    """判断 Dashboard 操作是否已经让车辆产生可见移动。"""
    return (
        summary.dx >= min_dx
        and summary.max_command_linear_velocity >= min_command
        and summary.max_body_forward_speed >= min_peak_speed
        and not summary.out_of_bounds
    )


def process_wait_timeout(duration_sec: float) -> float:
    """为驾驶时长附加软件渲染、绘图和日志落盘的退出宽限。"""
    return duration_sec + MANUAL_PROCESS_SHUTDOWN_GRACE_SEC


def build_child_command(
    args: argparse.Namespace,
    *,
    log_dir: Path,
    layout_report_path: Path,
) -> tuple[list[str], dict[str, str]]:
    """构造独立手动仿真命令，并注入窗口 token 与布局报告路径。"""
    command = [
        sys.executable,
        "main.py",
        "--config",
        args.config,
        "--gui",
        "--manual",
        "--duration-sec",
        str(child_run_duration(args)),
        "--interface-mode",
        "ecal",
    ]
    if args.plot_tab is not None:
        command.append("--developer-diagnostics")
    command.extend(("--figure-dir", str(log_dir / "figures")))
    command.extend(("--log-dir", str(log_dir)))
    child_env = os.environ.copy()
    child_env[PYBULLET_WINDOW_TOKEN_ENV] = (
        f"pybullet-main-verifier-{os.getpid()}-{uuid.uuid4().hex}"
    )
    child_env[DASHBOARD_LAYOUT_REPORT_ENV] = str(layout_report_path)
    return command, child_env


def child_run_duration(args: argparse.Namespace) -> float:
    """为需等待窗口的交互门禁预留启动预算，避免侵占按住时长。"""
    if args.verify_dashboard_tabs:
        return (
            max(args.duration_sec, args.hold_sec)
            + DASHBOARD_INTERACTION_STARTUP_GRACE_SEC
        )
    return args.duration_sec


def _frame_statistics(
    image: object,
) -> tuple[tuple[int, int, int], float, float, float]:
    """忽略外沿后计算颜色跨度、有效像素及其行列覆盖率。"""
    rgb = image.convert("RGB")
    width, height = rgb.size
    if width > 4 and height > 4:
        rgb = rgb.crop((2, 2, width - 2, height - 2))
    raw = rgb.tobytes()
    pixels = list(zip(raw[0::3], raw[1::3], raw[2::3]))
    if not pixels:
        return (0, 0, 0), 0.0, 0.0, 0.0
    minimum = tuple(min(pixel[channel] for pixel in pixels) for channel in range(3))
    maximum = tuple(max(pixel[channel] for pixel in pixels) for channel in range(3))
    ranges = tuple(maximum[channel] - minimum[channel] for channel in range(3))
    background = Counter(pixels).most_common(1)[0][0]
    active = [
        max(abs(pixel[channel] - background[channel]) for channel in range(3)) >= 6
        for pixel in pixels
    ]
    active_count = sum(active)
    inner_width, inner_height = rgb.size
    minimum_row_pixels = max(2, math.ceil(inner_width * 0.01))
    minimum_column_pixels = max(2, math.ceil(inner_height * 0.01))
    active_rows = sum(
        sum(active[row * inner_width : (row + 1) * inner_width])
        >= minimum_row_pixels
        for row in range(inner_height)
    )
    active_columns = sum(
        sum(
            active[row * inner_width + column]
            for row in range(inner_height)
        )
        >= minimum_column_pixels
        for column in range(inner_width)
    )
    return (
        ranges,
        active_count / len(pixels),
        active_rows / inner_height,
        active_columns / inner_width,
    )


def _frame_difference_ratio(first: object, second: object) -> float:
    """计算两张同尺寸截图中发生可见 RGB 变化的像素比例。"""
    first_rgb = first.convert("RGB")
    second_rgb = second.convert("RGB")
    if first_rgb.size != second_rgb.size:
        raise ValueError(
            f"frame size mismatch: first={first_rgb.size!r}, second={second_rgb.size!r}"
        )
    first_raw = first_rgb.tobytes()
    second_raw = second_rgb.tobytes()
    pixel_count = first_rgb.size[0] * first_rgb.size[1]
    if pixel_count <= 0:
        return 0.0
    changed = sum(
        max(
            abs(first_raw[offset + channel] - second_raw[offset + channel])
            for channel in range(3)
        )
        >= 6
        for offset in range(0, len(first_raw), 3)
    )
    return changed / pixel_count


def verify_dashboard_frames(
    frames: Sequence[object],
    tab_labels: Sequence[str],
) -> DashboardTabVerification:
    """逐页验证截图具有至少两个变化通道和可见非背景内容。"""
    if len(frames) != len(tab_labels):
        return DashboardTabVerification(
            False,
            len(frames),
            0.0,
            f"frame count mismatch: frames={len(frames)} tabs={len(tab_labels)}",
        )
    minimum_ratio = 1.0
    for index, (frame, label) in enumerate(zip(frames, tab_labels), start=1):
        channel_ranges, ratio, row_coverage, column_coverage = _frame_statistics(frame)
        minimum_ratio = min(minimum_ratio, ratio)
        varied_channels = sum(value >= 8 for value in channel_ranges)
        if (
            varied_channels < 2
            or ratio < 0.005
            or min(row_coverage, column_coverage) < 0.03
        ):
            return DashboardTabVerification(
                False,
                index,
                minimum_ratio,
                (
                    f"tab {index} {label!r} has insufficient pixel density: "
                    f"ranges={channel_ranges}, ratio={ratio:.6f}, "
                    f"coverage=({row_coverage:.6f},{column_coverage:.6f})"
                ),
            )
    return DashboardTabVerification(
        True,
        len(frames),
        minimum_ratio,
        "all dashboard tabs are nonblank",
    )


def _layout_rect(value: object, name: str) -> Rect:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError(f"{name} must contain x, y, width, height")
    if any(type(item) is not int for item in value):
        raise ValueError(f"{name} must contain integer coordinates")
    return Rect(*value)


def _dashboard_report_geometry(
    report: Mapping[str, object],
    client_rect: Rect,
) -> tuple[Rect, float]:
    """校验 Qt 逻辑窗口与 X11 物理 client 的 DPR 尺寸关系。"""
    raw_scale = report.get("device_pixel_ratio")
    if isinstance(raw_scale, bool) or not isinstance(raw_scale, (int, float)):
        raise ValueError("device_pixel_ratio must be a positive finite number")
    scale = float(raw_scale)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("device_pixel_ratio must be a positive finite number")
    logical_window = _layout_rect(report.get("window_rect"), "window_rect")
    expected_width = round(logical_window.width * scale)
    expected_height = round(logical_window.height * scale)
    tolerance = max(1, math.ceil(scale))
    if (
        abs(expected_width - client_rect.width) > tolerance
        or abs(expected_height - client_rect.height) > tolerance
    ):
        raise ValueError(
            "window_rect DPR size does not match Dashboard client area"
        )
    return logical_window, scale


def _dashboard_report_rect_to_x11(
    report: Mapping[str, object],
    value: object,
    name: str,
    client_rect: Rect,
) -> Rect:
    """以 Qt client 左上角为锚，把逻辑全局矩形映射到 X11 物理像素。"""
    logical_window, scale = _dashboard_report_geometry(report, client_rect)
    logical = _layout_rect(value, name)
    left = client_rect.x + round((logical.x - logical_window.x) * scale)
    top = client_rect.y + round((logical.y - logical_window.y) * scale)
    right = client_rect.x + round((logical.right - logical_window.x) * scale)
    bottom = client_rect.y + round((logical.bottom - logical_window.y) * scale)
    return Rect(left, top, right - left, bottom - top)


def _rect_inside(inner: Rect, outer: Rect) -> bool:
    return (
        inner.x >= outer.x
        and inner.y >= outer.y
        and inner.right <= outer.right
        and inner.bottom <= outer.bottom
    )


def _rects_overlap(first: Rect, second: Rect) -> bool:
    """只把具有正面积的交叠视为 artist/控件重叠。"""
    return (
        max(first.x, second.x) < min(first.right, second.right)
        and max(first.y, second.y) < min(first.bottom, second.bottom)
    )


def _dashboard_expected_page_contract(tab_label: str) -> tuple[str, tuple[str, ...]]:
    """返回独立于 Dashboard 实现的固定页面类型和命令按钮契约。"""
    if tab_label in DASHBOARD_CONTENT_TABS:
        return "content", ()
    if tab_label == "LiDAR点云":
        return "plot", DASHBOARD_LIDAR_PLOT_BUTTONS
    return "plot", DASHBOARD_LINE_PLOT_BUTTONS


def _integer_pair(value: object, name: str) -> tuple[int, int]:
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 2
        or any(type(item) is not int for item in value)
    ):
        raise ValueError(f"{name} must contain two integers")
    return value[0], value[1]


def validate_dashboard_layout_report(
    report: Mapping[str, object],
    client_rect: Rect,
    *,
    expected_tab_order: Sequence[str] = DASHBOARD_TAB_ORDER,
) -> DashboardLayoutVerification:
    """严格校验 v4 页面、axes、artist、页签箭头和关键控件契约。"""
    try:
        if not isinstance(report, Mapping):
            raise TypeError("dashboard layout report must be a mapping")
        frozen_tab_order = tuple(expected_tab_order)
        allowed_tab_labels = set(DASHBOARD_TAB_ORDER) | {"v2 eCAL", DEVELOPER_DIAGNOSTIC_TAB_LABEL}
        if (
            not frozen_tab_order
            or len(set(frozen_tab_order)) != len(frozen_tab_order)
            or any(label not in allowed_tab_labels for label in frozen_tab_order)
            or frozen_tab_order[0] not in {"接口状态", "v2 eCAL"}
            or "障碍物" not in frozen_tab_order
            or (
                DEVELOPER_DIAGNOSTIC_TAB_LABEL in frozen_tab_order
                and frozen_tab_order[-1] != DEVELOPER_DIAGNOSTIC_TAB_LABEL
            )
        ):
            raise ValueError("expected_tab_order is not a supported Dashboard contract")
        report_version = report.get("report_version")
        if (
            type(report_version) is not int
            or report_version != DASHBOARD_LAYOUT_REPORT_VERSION
        ):
            raise ValueError(
                f"report_version must be {DASHBOARD_LAYOUT_REPORT_VERSION}"
            )
        tab_count = report.get("tab_count")
        if type(tab_count) is not int or tab_count != len(frozen_tab_order):
            raise ValueError(
                f"tab count mismatch: {tab_count!r}"
            )
        tab_label = report.get("tab_label")
        if tab_label not in frozen_tab_order:
            raise ValueError(f"unknown tab label: {tab_label!r}")
        tab_index = report.get("tab_index")
        if (
            type(tab_index) is not int
            or not 0 <= tab_index < len(frozen_tab_order)
            or frozen_tab_order[tab_index] != tab_label
        ):
            raise ValueError(
                f"tab_index does not match tab_label {tab_label!r}"
            )
        tab_order = report.get("tab_order")
        if (
            not isinstance(tab_order, (list, tuple))
            or tuple(tab_order) != frozen_tab_order
        ):
            raise ValueError("tab_order does not match the required top-level order")
        expected_kind, expected_buttons = _dashboard_expected_page_contract(tab_label)
        if report.get("page_kind") != expected_kind:
            raise ValueError(
                f"page_kind mismatch for {tab_label!r}: "
                f"expected={expected_kind!r}"
            )
        required_buttons = report.get("required_plot_buttons")
        if (
            not isinstance(required_buttons, (list, tuple))
            or tuple(required_buttons) != expected_buttons
        ):
            raise ValueError(
                f"required_plot_buttons mismatch for {tab_label!r}: "
                f"expected={expected_buttons!r}"
            )
        logical_window, scale = _dashboard_report_geometry(report, client_rect)
        if logical_window.height < DASHBOARD_FORMAL_MIN_CLIENT_HEIGHT:
            raise ValueError(
                "logical window height must be at least "
                f"{DASHBOARD_FORMAL_MIN_CLIENT_HEIGHT} pixels for formal layout verification"
            )
        tolerance = max(1, math.ceil(scale))
        coverage_tolerance = (
            0
            if scale.is_integer()
            else DASHBOARD_COVERAGE_TOLERANCE_PHYSICAL_PX
        )

        def report_rect(value: object, name: str) -> Rect:
            return _dashboard_report_rect_to_x11(
                report,
                value,
                name,
                client_rect,
            )

        title = report_rect(report.get("title_rect"), "title_rect")
        tabs = report_rect(report.get("tabs_rect"), "tabs_rect")
        tab_bar = report_rect(report.get("tab_bar_rect"), "tab_bar_rect")
        controls = report_rect(report.get("controls_rect"), "controls_rect")
        page = report_rect(report.get("page_rect"), "page_rect")
        if tabs.bottom > controls.y:
            raise ValueError("top tabs and controls overlap")
        for name, rect in (
            ("title_rect", title),
            ("tabs_rect", tabs),
            ("controls_rect", controls),
        ):
            if not _rect_inside(rect, client_rect):
                raise ValueError(f"{name} is outside Dashboard client area")
        # 独立按物理像素核对 1:1；标题、边距和间距不属于两个 pane 的分母。
        if (
            abs(tabs.height - controls.height) > tolerance
            or abs(tabs.x - controls.x) > tolerance
            or abs(tabs.right - controls.right) > tolerance
        ):
            raise ValueError(
                "top tabs and controls do not satisfy the 50:50 split"
            )
        physical_margin = round(DASHBOARD_ROOT_MARGIN_LOGICAL_PX * scale)
        physical_spacing = round(DASHBOARD_ROOT_SPACING_LOGICAL_PX * scale)
        # 用实际 Qt 矩形独立锁定根布局，防止两个 pane 等高却共同缩小。
        if any(
            abs(actual - expected) > tolerance
            for actual, expected in (
                (title.x, client_rect.x + physical_margin),
                (title.y, client_rect.y + physical_margin),
                (title.right, client_rect.right - physical_margin),
                (tabs.x, client_rect.x + physical_margin),
                (tabs.y - title.bottom, physical_spacing),
                (tabs.right, client_rect.right - physical_margin),
                (controls.x, client_rect.x + physical_margin),
                (controls.y - tabs.bottom, physical_spacing),
                (controls.right, client_rect.right - physical_margin),
                (controls.bottom, client_rect.bottom - physical_margin),
            )
        ):
            raise ValueError(
                "title_rect and panes do not fill available vertical space "
                "with 8px margins and 6px spacing"
            )
        if not _rect_inside(page, tabs):
            raise ValueError("page_rect is outside tabs_rect")
        if not _rect_inside(tab_bar, tabs):
            raise ValueError("tab_bar_rect is outside tabs_rect")
        tab_scroll_buttons = report.get("tab_scroll_button_rects")
        if not isinstance(tab_scroll_buttons, Mapping) or set(tab_scroll_buttons) not in (
            set(),
            {"left", "right"},
        ):
            raise ValueError(
                "tab_scroll_button_rects must be empty or contain exactly left and right"
            )
        tab_button_rects = {
            name: report_rect(
                tab_scroll_buttons[name],
                f"tab_scroll_button_rects[{name!r}]",
            )
            for name in tab_scroll_buttons
        }
        for name, button in tab_button_rects.items():
            if not _rect_inside(button, tab_bar):
                raise ValueError(f"tab scroll button {name!r} is outside tab_bar_rect")
        if tab_button_rects:
            left_button = tab_button_rects["left"]
            right_button = tab_button_rects["right"]
            if (
                left_button.x >= right_button.x
                or left_button.right - right_button.x > tolerance
            ):
                raise ValueError("tab scroll buttons overlap")

        canvas_value = report.get("canvas_rect")
        canvas: Rect | None = None
        if expected_kind == "plot":
            if canvas_value is None:
                raise ValueError("canvas_rect is required for plot pages")
            canvas = report_rect(canvas_value, "canvas_rect")
            if not _rect_inside(canvas, page):
                raise ValueError("canvas_rect is outside page_rect")
            # 分数 DPR 的端点映射最多允许一个物理像素取整余量。
            if (
                canvas.width * 100
                + coverage_tolerance * 100
                < page.width * DASHBOARD_MIN_CANVAS_PAGE_WIDTH_PERCENT
                or canvas.height * 100
                + coverage_tolerance * 100
                < page.height * DASHBOARD_MIN_CANVAS_PAGE_HEIGHT_PERCENT
            ):
                raise ValueError(
                    "canvas_rect does not fill at least 85% width and 70% height "
                    "of page_rect"
                )
        elif canvas_value is not None:
            raise ValueError("canvas_rect must be null for content pages")

        axes_value = report.get("axes_rect")
        rendered_data_revision = report.get("rendered_data_revision")
        if expected_kind == "plot":
            if (
                type(rendered_data_revision) is not int
                or rendered_data_revision < 0
            ):
                raise ValueError(
                    "rendered_data_revision must be a non-negative integer for plot pages"
                )
            if axes_value is None:
                raise ValueError("axes_rect is required for plot pages")
            assert canvas is not None
            axes_rect = report_rect(axes_value, "axes_rect")
            if not _rect_inside(axes_rect, canvas):
                raise ValueError("axes_rect is outside canvas_rect")
            if (
                axes_rect.width * 100 < canvas.width * 60
                or axes_rect.height * 100 < canvas.height * 50
            ):
                raise ValueError(
                    "axes_rect does not fill at least 60% width and 50% height of canvas_rect"
                )
        else:
            if axes_value is not None:
                raise ValueError("axes_rect must be null for content pages")
            if rendered_data_revision is not None:
                raise ValueError(
                    "rendered_data_revision must be null for content pages"
                )

        legend_value = report.get("legend_rect")
        legend_required = expected_kind == "plot"
        legend: Rect | None = None
        if legend_required:
            if legend_value is None:
                raise ValueError("legend_rect is required for plot pages")
            assert canvas is not None
            legend = report_rect(legend_value, "legend_rect")
            if not _rect_inside(legend, canvas):
                raise ValueError("legend_rect is outside canvas_rect")
        elif legend_value is not None:
            raise ValueError(f"legend_rect is unexpected for {tab_label!r}")

        plot_buttons = report.get("plot_button_rects")
        if not isinstance(plot_buttons, Mapping):
            raise ValueError("plot_button_rects must be a mapping")
        if set(plot_buttons) != set(expected_buttons):
            raise ValueError(
                f"plot_button_rects mismatch for {tab_label!r}: "
                f"expected={expected_buttons!r}"
            )
        plot_button_rects: dict[str, Rect] = {}
        for name in expected_buttons:
            button = report_rect(
                plot_buttons[name],
                f"plot_button_rects[{name!r}]",
            )
            if not _rect_inside(button, page):
                raise ValueError(f"plot button {name!r} is outside page_rect")
            for other_name, other_button in plot_button_rects.items():
                if _rects_overlap(button, other_button):
                    raise ValueError(
                        f"plot buttons {other_name!r} and {name!r} overlap"
                    )
            plot_button_rects[name] = button

        expected_content_widgets = DASHBOARD_CONTENT_WIDGETS.get(tab_label, ())
        content_widgets = report.get("content_widget_rects")
        if not isinstance(content_widgets, Mapping) or set(content_widgets) != set(
            expected_content_widgets
        ):
            raise ValueError(
                f"content_widget_rects mismatch for {tab_label!r}: "
                f"expected={expected_content_widgets!r}"
            )
        for name in expected_content_widgets:
            widget = report_rect(
                content_widgets[name],
                f"content_widget_rects[{name!r}]",
            )
            if not _rect_inside(widget, page):
                raise ValueError(f"content widget {name!r} is outside page_rect")

        plot_artists = report.get("plot_artist_rects")
        tick_rects_by_axis = report.get("plot_tick_rects")
        legend_text_values = report.get("legend_text_rects")
        if expected_kind == "content":
            if plot_artists is not None:
                raise ValueError("plot_artist_rects must be null for content pages")
            if tick_rects_by_axis is not None:
                raise ValueError("plot_tick_rects must be null for content pages")
            if legend_text_values is not None:
                raise ValueError("legend_text_rects must be null for content pages")
        else:
            if not isinstance(plot_artists, Mapping) or set(plot_artists) != set(
                DASHBOARD_PLOT_ARTISTS
            ):
                raise ValueError(
                    "plot_artist_rects must contain the complete title/label/offset contract"
                )
            assert canvas is not None
            artist_rects: dict[str, Rect | None] = {}
            for name in DASHBOARD_PLOT_ARTISTS:
                value = plot_artists[name]
                if value is None:
                    if name in {"title", "x_label", "y_label"}:
                        raise ValueError(f"plot artist {name!r} is required")
                    artist_rects[name] = None
                    continue
                if not isinstance(value, Mapping) or set(value) != {"text", "rect"}:
                    raise ValueError(f"plot artist {name!r} has invalid fields")
                text_value = value.get("text")
                if not isinstance(text_value, str) or not text_value.strip():
                    raise ValueError(f"plot artist {name!r} has empty text")
                artist = report_rect(value.get("rect"), f"plot artist {name}")
                if not _rect_inside(artist, canvas):
                    raise ValueError(f"plot artist {name!r} is outside canvas_rect")
                artist_rects[name] = artist
            visible_artists = [
                (name, rect)
                for name, rect in artist_rects.items()
                if rect is not None
            ]
            for first_index, (first_name, first_rect) in enumerate(visible_artists):
                for second_name, second_rect in visible_artists[first_index + 1 :]:
                    if _rects_overlap(first_rect, second_rect):
                        raise ValueError(
                            f"plot artists {first_name!r} and {second_name!r} overlap"
                        )

            if not isinstance(tick_rects_by_axis, Mapping) or set(
                tick_rects_by_axis
            ) != {"x", "y"}:
                raise ValueError("plot_tick_rects must contain exactly x and y")
            visible_ticks: list[tuple[str, Rect]] = []
            for axis_name in ("x", "y"):
                values = tick_rects_by_axis[axis_name]
                if not isinstance(values, (list, tuple)) or not values:
                    raise ValueError(
                        f"plot_tick_rects[{axis_name!r}] must contain visible ticks"
                    )
                for tick_index, value in enumerate(values):
                    tick_name = f"{axis_name}[{tick_index}]"
                    if not isinstance(value, Mapping) or set(value) != {"text", "rect"}:
                        raise ValueError(f"plot tick {tick_name} has invalid fields")
                    text_value = value.get("text")
                    if not isinstance(text_value, str) or not text_value.strip():
                        raise ValueError(f"plot tick {tick_name} has empty text")
                    tick = report_rect(value.get("rect"), f"plot tick {tick_name}")
                    if not _rect_inside(tick, canvas):
                        raise ValueError(f"plot tick {tick_name} is outside canvas_rect")
                    for other_name, other_tick in visible_ticks:
                        if _rects_overlap(tick, other_tick):
                            raise ValueError(
                                f"plot ticks {other_name!r} and {tick_name!r} overlap"
                            )
                    for artist_name, artist_rect in visible_artists:
                        if _rects_overlap(tick, artist_rect):
                            raise ValueError(
                                f"plot tick {tick_name!r} and artist {artist_name!r} overlap"
                            )
                    visible_ticks.append((tick_name, tick))

            if legend_required:
                if not isinstance(legend_text_values, (list, tuple)):
                    raise ValueError("legend_text_rects must be a list for plot pages")
                assert legend is not None
                legend_texts: list[tuple[str, Rect]] = []
                for text_index, value in enumerate(legend_text_values):
                    text_name = f"legend[{text_index}]"
                    if not isinstance(value, Mapping) or set(value) != {"text", "rect"}:
                        raise ValueError(f"legend text {text_name} has invalid fields")
                    text_value = value.get("text")
                    if not isinstance(text_value, str) or not text_value.strip():
                        raise ValueError(f"legend text {text_name} has empty text")
                    text_rect = report_rect(value.get("rect"), f"legend text {text_name}")
                    if not _rect_inside(text_rect, legend):
                        raise ValueError(f"legend text {text_name} is outside legend_rect")
                    for other_name, other_rect in legend_texts:
                        if _rects_overlap(text_rect, other_rect):
                            raise ValueError(
                                f"legend texts {other_name!r} and {text_name!r} overlap"
                            )
                    legend_texts.append((text_name, text_rect))
                for name, rect in (*visible_artists, *visible_ticks):
                    if _rects_overlap(legend, rect):
                        raise ValueError(f"legend_rect overlaps plot text {name!r}")
            elif legend_text_values is not None:
                raise ValueError(f"legend_text_rects is unexpected for {tab_label!r}")

        viewport = report_rect(
            report.get("control_viewport_rect"),
            "control_viewport_rect",
        )
        content = report_rect(
            report.get("control_content_rect"),
            "control_content_rect",
        )
        if not _rect_inside(viewport, controls):
            raise ValueError("control_viewport_rect is outside controls_rect")
        if (
            content.width + tolerance < viewport.width
            or content.height + tolerance < viewport.height
        ):
            raise ValueError("control_content_rect is smaller than its viewport")
        scroll_min, scroll_max = _integer_pair(
            report.get("control_scroll_range"),
            "control_scroll_range",
        )
        if scroll_min != 0 or scroll_max < scroll_min:
            raise ValueError("control_scroll_range is invalid")

        critical = report.get("critical_control_rects", {})
        if not isinstance(critical, Mapping):
            raise ValueError("critical_control_rects must be a mapping")
        if set(critical) != set(DASHBOARD_REQUIRED_CRITICAL_CONTROLS):
            raise ValueError(
                "critical_control_rects must contain exactly "
                f"{DASHBOARD_REQUIRED_CRITICAL_CONTROLS!r}"
            )
        critical_control_rects: dict[str, Rect] = {}
        for name in DASHBOARD_REQUIRED_CRITICAL_CONTROLS:
            value = critical[name]
            if not isinstance(value, Mapping):
                raise ValueError(f"critical control {name!r} must be a mapping")
            if set(value) != {"rect", "viewport_rect", "scroll_value"}:
                raise ValueError(f"critical control {name!r} has invalid fields")
            control = report_rect(value.get("rect"), f"critical control {name}")
            recorded_viewport = report_rect(
                value.get("viewport_rect"),
                f"critical control {name} viewport",
            )
            if recorded_viewport != viewport:
                raise ValueError(
                    f"critical control {name!r} reported a different viewport"
                )
            scroll_value = value.get("scroll_value")
            if type(scroll_value) is not int or not (
                scroll_min <= scroll_value <= scroll_max
            ):
                raise ValueError(
                    f"critical control {name!r} has invalid scroll_value"
                )
            if not _rect_inside(control, recorded_viewport):
                raise ValueError(
                    f"critical control {name!r} is outside its viewport"
                )
            critical_control_rects[name] = control

        qt_texts = report.get("qt_text_rects")
        if not isinstance(qt_texts, Mapping) or set(qt_texts) != {
            "tab",
            "plot_buttons",
            "critical_controls",
        }:
            raise ValueError(
                "qt_text_rects must contain tab, plot_buttons and critical_controls"
            )

        def qt_text_entry(
            value: object,
            name: str,
            *,
            expected_container: Rect | None = None,
        ) -> tuple[Rect, Rect]:
            if not isinstance(value, Mapping) or set(value) != {
                "text",
                "rect",
                "container_rect",
            }:
                raise ValueError(f"Qt text {name!r} has invalid fields")
            text_value = value.get("text")
            if not isinstance(text_value, str) or not text_value.strip():
                raise ValueError(f"Qt text {name!r} is empty")
            text_rect = report_rect(value.get("rect"), f"Qt text {name}")
            container_rect = report_rect(
                value.get("container_rect"),
                f"Qt text {name} container",
            )
            if expected_container is not None and not _rect_inside(
                container_rect,
                expected_container,
            ):
                raise ValueError(f"Qt text {name!r} container is outside its widget")
            if not _rect_inside(text_rect, container_rect):
                raise ValueError(f"Qt text {name!r} is outside its container")
            return text_rect, container_rect

        tab_text, tab_container = qt_text_entry(qt_texts["tab"], "tab")
        if not _rect_inside(tab_container, tab_bar):
            raise ValueError("Qt text 'tab' container is outside tab_bar_rect")

        plot_button_texts = qt_texts["plot_buttons"]
        if not isinstance(plot_button_texts, Mapping) or set(
            plot_button_texts
        ) != set(expected_buttons):
            raise ValueError("Qt plot button texts do not match required_plot_buttons")
        visible_qt_page_texts = [("tab", tab_text)]
        for name in expected_buttons:
            text_rect, _container = qt_text_entry(
                plot_button_texts[name],
                f"plot button {name}",
                expected_container=plot_button_rects[name],
            )
            visible_qt_page_texts.append((f"plot button {name}", text_rect))
        for first_index, (first_name, first_rect) in enumerate(visible_qt_page_texts):
            for second_name, second_rect in visible_qt_page_texts[first_index + 1 :]:
                if _rects_overlap(first_rect, second_rect):
                    raise ValueError(
                        f"Qt texts {first_name!r} and {second_name!r} overlap"
                    )

        critical_texts = qt_texts["critical_controls"]
        if not isinstance(critical_texts, Mapping) or set(critical_texts) != set(
            DASHBOARD_REQUIRED_CRITICAL_CONTROLS
        ):
            raise ValueError("Qt critical control texts do not match required controls")
        for name in DASHBOARD_REQUIRED_CRITICAL_CONTROLS:
            qt_text_entry(
                critical_texts[name],
                f"critical control {name}",
                expected_container=critical_control_rects[name],
            )
    except (TypeError, ValueError) as exc:
        return DashboardLayoutVerification(False, str(exc))
    return DashboardLayoutVerification(True, "dashboard layout report passed")


def validate_dashboard_layout_stability(
    before: Mapping[str, object],
    after: Mapping[str, object],
    client_rect: Rect,
) -> DashboardLayoutVerification:
    """校验同一页新数据完成绘制后，五个稳定布局矩形保持不变。"""
    expected_tab_order = tuple(before.get("tab_order", ()))
    before_result = validate_dashboard_layout_report(
        before,
        client_rect,
        expected_tab_order=expected_tab_order,
    )
    if not before_result.passed:
        return DashboardLayoutVerification(
            False,
            f"first layout report failed: {before_result.detail}",
        )
    after_result = validate_dashboard_layout_report(
        after,
        client_rect,
        expected_tab_order=expected_tab_order,
    )
    if not after_result.passed:
        return DashboardLayoutVerification(
            False,
            f"second layout report failed: {after_result.detail}",
        )
    try:
        for field in (
            "report_version",
            "tab_index",
            "tab_label",
            "page_kind",
            "device_pixel_ratio",
        ):
            if before.get(field) != after.get(field):
                raise ValueError(f"{field} changed between layout reports")
        if tuple(before.get("tab_order", ())) != tuple(after.get("tab_order", ())):
            raise ValueError("tab_order changed between layout reports")
        if _layout_rect(before.get("window_rect"), "window_rect") != _layout_rect(
            after.get("window_rect"),
            "window_rect",
        ):
            raise ValueError("window_rect changed between layout reports")

        for field in DASHBOARD_STABLE_LAYOUT_RECTS:
            before_value = before.get(field)
            after_value = after.get(field)
            if before_value is None or after_value is None:
                if before_value is not None or after_value is not None:
                    raise ValueError(f"{field} changed between layout reports")
                continue
            if _layout_rect(before_value, field) != _layout_rect(after_value, field):
                raise ValueError(f"{field} changed between layout reports")

        if before.get("page_kind") == "plot":
            before_revision = before.get("rendered_data_revision")
            after_revision = after.get("rendered_data_revision")
            if not isinstance(before_revision, int) or not isinstance(after_revision, int):
                raise ValueError("rendered_data_revision is invalid")
            if after_revision <= before_revision:
                raise ValueError(
                    "rendered_data_revision did not advance between layout reports"
                )
    except (TypeError, ValueError) as exc:
        return DashboardLayoutVerification(False, str(exc))
    return DashboardLayoutVerification(
        True,
        "dashboard layout remained stable after new data was rendered",
    )


def wait_for_dashboard_layout_report(
    path: Path,
    tab_index: int,
    tab_label: str,
    timeout_sec: float,
    after_line_number: int = -1,
) -> DashboardLayoutReportOccurrence:
    """有界等待指定页签在给定 JSONL 行游标之后追加新报告。"""
    if type(after_line_number) is not int or after_line_number < -1:
        raise ValueError("after_line_number must be an integer greater than or equal to -1")
    deadline = time.monotonic() + timeout_sec
    last_detail = "report file does not exist"
    while time.monotonic() < deadline:
        if path.exists():
            try:
                lines = path.read_text(encoding="utf-8").splitlines()
            except OSError as exc:
                last_detail = str(exc)
            else:
                for line_number, line in enumerate(lines):
                    if line_number <= after_line_number:
                        continue
                    try:
                        report = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    if not isinstance(report, Mapping):
                        continue
                    if (
                        report.get("tab_index") == tab_index
                        and report.get("tab_label") == tab_label
                    ):
                        return DashboardLayoutReportOccurrence(
                            line_number=line_number,
                            report=report,
                        )
                last_detail = (
                    f"no fresh report after line {after_line_number} for "
                    f"tab_index={tab_index}, tab_label={tab_label!r}"
                )
        time.sleep(0.05)
    raise RuntimeError(f"dashboard layout report timeout: {last_detail}")


def verify_dashboard_tabs(
    window_id: str,
    *,
    display: str,
    client_rect: Rect,
    layout_report_path: Path,
    hold_drive_sec: float,
    expected_tab_order: Sequence[str] = DASHBOARD_TAB_ORDER,
    report_reader: Callable[
        [Path, int, str, float, int],
        DashboardLayoutReportOccurrence,
    ] = wait_for_dashboard_layout_report,
    capture: Callable[..., object] | None = None,
    command_runner: Callable[[Sequence[str]], object] | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> DashboardTabVerification:
    """按住前进键遍历两轮 15 页，验证像素、交互和数据后布局稳定性。"""
    if not isinstance(client_rect, Rect):
        raise TypeError("client_rect must be a Rect")
    if not math.isfinite(hold_drive_sec) or hold_drive_sec <= 0.0:
        raise ValueError("hold_drive_sec must be positive and finite")
    frozen_tab_order = tuple(expected_tab_order)
    if frozen_tab_order not in {
        DASHBOARD_TAB_ORDER,
        V2_DASHBOARD_TAB_ORDER,
    }:
        raise ValueError("expected_tab_order is not a supported Dashboard contract")
    if capture is None:
        from PIL import ImageGrab

        capture = ImageGrab.grab
    run_command = _run_xdotool if command_runner is None else command_runner

    def capture_client() -> object:
        image = capture(
            bbox=(
                client_rect.x,
                client_rect.y,
                client_rect.right,
                client_rect.bottom,
            ),
            xdisplay=display,
        )
        if getattr(image, "size", None) != (
            client_rect.width,
            client_rect.height,
        ):
            raise ValueError(
                "client capture size mismatch: "
                f"actual={getattr(image, 'size', None)!r}"
            )
        return image

    def crop_client(image: object, rect: Rect) -> object:
        return image.crop(
            (
                rect.x - client_rect.x,
                rect.y - client_rect.y,
                rect.right - client_rect.x,
                rect.bottom - client_rect.y,
            )
        )

    frames: list[object] = []
    first_reports: dict[str, Mapping[str, object]] = {}
    report_line_cursor = -1
    run_command(("windowfocus", window_id))
    try:
        run_command(("keydown", "Up"))
        started_at = clock()
        for index, label in enumerate(frozen_tab_order):
            if index:
                run_command(("keydown", "ctrl"))
                try:
                    run_command(("key", "Tab"))
                finally:
                    run_command(("keyup", "ctrl"))
            occurrence = report_reader(
                layout_report_path,
                index,
                label,
                DASHBOARD_LAYOUT_REPORT_TIMEOUT_SEC,
                report_line_cursor,
            )
            report_line_cursor = occurrence.line_number
            report = occurrence.report
            if report.get("tab_index") != index or report.get("tab_label") != label:
                return DashboardTabVerification(
                    False,
                    index + 1,
                    0.0,
                    f"tab {index + 1} report order mismatch: expected={label!r}",
                )
            reported_order = report.get("tab_order")
            if reported_order is not None and tuple(reported_order) != frozen_tab_order:
                return DashboardTabVerification(
                    False,
                    index + 1,
                    0.0,
                    f"tab {index + 1} reported a different top-level order",
                )
            layout_result = validate_dashboard_layout_report(
                report,
                client_rect,
                expected_tab_order=frozen_tab_order,
            )
            if not layout_result.passed:
                return DashboardTabVerification(
                    False,
                    index + 1,
                    0.0,
                    f"tab {index + 1} {label!r} layout failed: {layout_result.detail}",
                )
            first_reports[label] = report
            if index == 0:
                # 把鼠标移出标签栏，避免 hover 像素掩盖真实滚动状态。
                stable_pointer = (
                    str(client_rect.x + 2),
                    str(client_rect.bottom - 2),
                )
                scroll_buttons = report["tab_scroll_button_rects"]
                tab_bar = _dashboard_report_rect_to_x11(
                    report,
                    report["tab_bar_rect"],
                    "tab_bar_rect",
                    client_rect,
                )
                left_button = _dashboard_report_rect_to_x11(
                    report,
                    scroll_buttons["left"],
                    "tab_scroll_button_rects['left']",
                    client_rect,
                )
                # 只比较按钮左侧的标签内容；按钮焦点框不能算作滚动证据。
                tab_scroll_content = Rect(
                    tab_bar.x,
                    tab_bar.y,
                    left_button.x - tab_bar.x,
                    tab_bar.height,
                )
                run_command(("mousemove", *stable_pointer))
                sleeper(DASHBOARD_TAB_SCROLL_SETTLE_SEC)
                try:
                    initial_tab_bar = crop_client(
                        capture_client(),
                        tab_scroll_content,
                    )
                except ValueError as exc:
                    return DashboardTabVerification(False, 1, 0.0, str(exc))
                right_button = _dashboard_report_rect_to_x11(
                    report,
                    scroll_buttons["right"],
                    "tab_scroll_button_rects['right']",
                    client_rect,
                )
                run_command(
                    (
                        "mousemove",
                        str(right_button.x + right_button.width // 2),
                        str(right_button.y + right_button.height // 2),
                    )
                )
                run_command(("click", "1"))
                run_command(("mousemove", *stable_pointer))
                sleeper(DASHBOARD_TAB_SCROLL_SETTLE_SEC)
                try:
                    shifted_tab_bar = crop_client(
                        capture_client(),
                        tab_scroll_content,
                    )
                except ValueError as exc:
                    return DashboardTabVerification(False, 1, 0.0, str(exc))
                changed_ratio = _frame_difference_ratio(
                    initial_tab_bar,
                    shifted_tab_bar,
                )
                if changed_ratio < DASHBOARD_TAB_SCROLL_MIN_CHANGE_RATIO:
                    return DashboardTabVerification(
                        False,
                        1,
                        0.0,
                        (
                            "tab scroll right click did not change the tab bar: "
                            f"ratio={changed_ratio:.6f}"
                        ),
                    )

                left_button = _dashboard_report_rect_to_x11(
                    report,
                    scroll_buttons["left"],
                    "tab_scroll_button_rects['left']",
                    client_rect,
                )
                restored_ratio = changed_ratio
                # Qt 会按下一段被遮挡标签滚动，左右单击不保证像素位移对称。
                for _attempt in range(DASHBOARD_TAB_SCROLL_MAX_RESTORE_CLICKS):
                    run_command(
                        (
                            "mousemove",
                            str(left_button.x + left_button.width // 2),
                            str(left_button.y + left_button.height // 2),
                        )
                    )
                    run_command(("click", "1"))
                    run_command(("mousemove", *stable_pointer))
                    sleeper(DASHBOARD_TAB_SCROLL_SETTLE_SEC)
                    try:
                        restored_tab_bar = crop_client(
                            capture_client(),
                            tab_scroll_content,
                        )
                    except ValueError as exc:
                        return DashboardTabVerification(False, 1, 0.0, str(exc))
                    restored_ratio = _frame_difference_ratio(
                        initial_tab_bar,
                        restored_tab_bar,
                    )
                    if restored_ratio <= DASHBOARD_TAB_SCROLL_MAX_RESTORE_RATIO:
                        break
                else:
                    return DashboardTabVerification(
                        False,
                        1,
                        0.0,
                        (
                            "tab scroll left clicks did not restore the tab bar "
                            f"after {DASHBOARD_TAB_SCROLL_MAX_RESTORE_CLICKS} "
                            "left clicks: "
                            f"ratio={restored_ratio:.6f}"
                        ),
                    )
            target_value = report.get("canvas_rect") or report.get("page_rect")
            target = _dashboard_report_rect_to_x11(
                report,
                target_value,
                "capture_rect",
                client_rect,
            )
            try:
                full_client = capture_client()
            except ValueError as exc:
                return DashboardTabVerification(
                    False,
                    index + 1,
                    0.0,
                    f"tab {index + 1} {label!r} {exc}",
                )
            frames.append(crop_client(full_client, target))

            # 先保存当前非空图，再清空曲线；两类按钮都必须走真实点击路径。
            for button_name in reversed(tuple(report["required_plot_buttons"])):
                button = _dashboard_report_rect_to_x11(
                    report,
                    report["plot_button_rects"][button_name],
                    f"plot_button_rects[{button_name!r}]",
                    client_rect,
                )
                run_command(
                    (
                        "mousemove",
                        str(button.x + button.width // 2),
                        str(button.y + button.height // 2),
                    )
                )
                run_command(("click", "1"))
                sleeper(DASHBOARD_PLOT_BUTTON_SETTLE_SEC)

        # 第二轮从上一条物理 JSONL 行之后读取，证明新数据绘制未改变布局。
        for index, label in enumerate(frozen_tab_order):
            run_command(("keydown", "ctrl"))
            try:
                run_command(("key", "Tab"))
            finally:
                run_command(("keyup", "ctrl"))
            occurrence = report_reader(
                layout_report_path,
                index,
                label,
                DASHBOARD_LAYOUT_REPORT_TIMEOUT_SEC,
                report_line_cursor,
            )
            report_line_cursor = occurrence.line_number
            stability = validate_dashboard_layout_stability(
                first_reports[label],
                occurrence.report,
                client_rect,
            )
            if not stability.passed:
                return DashboardTabVerification(
                    False,
                    index + 1,
                    0.0,
                    (
                        f"tab {index + 1} {label!r} data-layout stability failed: "
                        f"{stability.detail}"
                    ),
                )
        remaining = hold_drive_sec - (clock() - started_at)
        if remaining > 0.0:
            sleeper(remaining)
        return verify_dashboard_frames(frames, frozen_tab_order)
    finally:
        try:
            run_command(("keyup", "ctrl"))
        finally:
            run_command(("keyup", "Up"))


def _format_rect(rect: Rect) -> str:
    """把窗口矩形格式化为稳定、适合验收日志阅读的文本。"""
    return f"(x={rect.x},y={rect.y},width={rect.width},height={rect.height})"


def _round_positive_fraction_half_up(value: Fraction) -> int:
    """在 verifier 内独立用交叉乘积计算正有理数 half-up。"""
    return (2 * value.numerator + value.denominator) // (2 * value.denominator)


def validate_window_layout(
    available: Rect,
    main: Rect,
    dashboard: Rect,
    *,
    device_pixel_ratio: float = 1.0,
) -> None:
    """用独立 33/100 oracle 核对比例、覆盖、公共边和 DPR 对齐。"""
    if isinstance(device_pixel_ratio, bool) or not isinstance(
        device_pixel_ratio,
        (int, float),
    ):
        raise ValueError("device_pixel_ratio must be a positive finite number")
    scale = float(device_pixel_ratio)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("device_pixel_ratio must be a positive finite number")

    ratio = Fraction(DASHBOARD_RATIO_NUMERATOR, DASHBOARD_RATIO_DENOMINATOR)
    scale_ratio = Fraction(str(scale))
    ideal_logical_width = available.width * ratio / scale_ratio
    logical_dashboard_width = max(
        1,
        _round_positive_fraction_half_up(ideal_logical_width),
    )
    expected_dashboard_width = max(
        1,
        _round_positive_fraction_half_up(logical_dashboard_width * scale_ratio),
    )
    if expected_dashboard_width >= available.width:
        raise ValueError("available geometry is too narrow for 33/100 Dashboard")
    expected_main = Rect(
        available.x,
        available.y,
        available.width - expected_dashboard_width,
        available.height,
    )
    expected_dashboard = Rect(
        expected_main.right,
        available.y,
        expected_dashboard_width,
        available.height,
    )

    reasons: list[str] = []
    if main.right > dashboard.x:
        reasons.append("windows overlap")
    elif main.right < dashboard.x:
        reasons.append("windows leave a horizontal gap")
    if main.x != available.x or dashboard.right != available.right:
        reasons.append("windows do not cover available width")
    if (
        main.y != available.y
        or dashboard.y != available.y
        or main.bottom != available.bottom
        or dashboard.bottom != available.bottom
    ):
        reasons.append("windows are not vertically aligned to available geometry")
    if dashboard.width != expected_dashboard_width:
        reasons.append(
            "dashboard width is not the exact DPR-aligned 33/100 of available width"
        )

    if reasons:
        raise ValueError(
            "window layout verification failed "
            f"({'; '.join(reasons)}): "
            f"expected_main={_format_rect(expected_main)} "
            f"actual_main={_format_rect(main)} "
            f"expected_dashboard={_format_rect(expected_dashboard)} "
            f"actual_dashboard={_format_rect(dashboard)}"
        )


def wait_for_window_layout(
    available: Rect,
    *,
    geometry_getter: Callable[[], tuple[Rect, Rect]],
    timeout_sec: float,
    device_pixel_ratio: float = 1.0,
    poll_interval_sec: float = WINDOW_LAYOUT_POLL_INTERVAL_SEC,
    clock: Callable[[], float] | None = None,
    sleeper: Callable[[float], None] | None = None,
) -> tuple[Rect, Rect]:
    """有界等待两个完整外框稳定到目标布局，超时保留最后一次实际几何。"""
    if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be a positive finite number")
    if not math.isfinite(poll_interval_sec) or poll_interval_sec <= 0.0:
        raise ValueError("poll_interval_sec must be a positive finite number")

    monotonic = time.monotonic if clock is None else clock
    sleep = time.sleep if sleeper is None else sleeper
    deadline = monotonic() + timeout_sec

    while True:
        main, dashboard = geometry_getter()
        try:
            validate_window_layout(
                available,
                main,
                dashboard,
                device_pixel_ratio=device_pixel_ratio,
            )
        except ValueError as exc:
            remaining = deadline - monotonic()
            if remaining <= 0.0:
                raise RuntimeError(
                    "window layout did not stabilize before timeout: "
                    f"available={_format_rect(available)} "
                    f"main={_format_rect(main)} "
                    f"dashboard={_format_rect(dashboard)}; "
                    f"last_error={exc}"
                ) from exc
            sleep(min(poll_interval_sec, remaining))
        else:
            return main, dashboard


def parse_geometry(output: str) -> WindowGeometry:
    """解析 xdotool getwindowgeometry --shell 的输出。"""
    values: dict[str, int] = {}
    for line in output.splitlines():
        if "=" not in line:
            continue
        key, raw_value = line.split("=", 1)
        if key in {"X", "Y", "WIDTH", "HEIGHT"}:
            values[key] = int(raw_value)
    return WindowGeometry(
        x=values["X"],
        y=values["Y"],
        width=values["WIDTH"],
        height=values["HEIGHT"],
    )


def parse_xwininfo_geometry(output: str) -> WindowGeometry:
    """解析 xwininfo 客户区的绝对原点和物理尺寸。"""
    fields = {
        "Absolute upper-left X": "x",
        "Absolute upper-left Y": "y",
        "Width": "width",
        "Height": "height",
    }
    values: dict[str, int] = {}
    for line in output.splitlines():
        if ":" not in line:
            continue
        key, raw_value = line.strip().split(":", 1)
        field = fields.get(key)
        if field is not None:
            values[field] = int(raw_value.strip())
    return WindowGeometry(
        x=values["x"],
        y=values["y"],
        width=values["width"],
        height=values["height"],
    )


def _run_xdotool(args: Sequence[str]) -> subprocess.CompletedProcess[str]:
    """调用 xdotool，并把失败信息保留给调用方打印。"""
    return subprocess.run(["xdotool", *args], check=True, text=True, capture_output=True)


def _get_window_geometry(window_id: str) -> WindowGeometry:
    """用 xwininfo 获取 Qt 客户区几何，避开 XWayland 重父化坐标。"""
    result = subprocess.run(
        ["xwininfo", "-id", window_id],
        check=True,
        text=True,
        capture_output=True,
    )
    return parse_xwininfo_geometry(result.stdout)


def _get_window_outer_geometry(window_id: str) -> WindowGeometry:
    """回读带窗口管理器标题栏的完整物理窗口矩形。"""
    return read_x11_outer_geometry(window_id)


def _find_window(
    window_title: str,
    process_id: int | None,
    timeout_sec: float,
    *,
    excluded_window_ids: tuple[str, ...] = (),
) -> str:
    """按精确标题和必选进程 PID 等待唯一 Qt 客户窗口。"""
    if type(process_id) is not int or process_id <= 0:
        raise ValueError("process_id must be a positive integer")
    deadline = time.monotonic() + timeout_sec
    last_error = ""
    title_pattern = rf"^{re.escape(window_title)}$"
    command = ["xdotool", "search", "--all", "--onlyvisible"]
    command.extend(("--pid", str(process_id)))
    command.extend(("--name", title_pattern))
    while time.monotonic() < deadline:
        result = subprocess.run(
            command,
            text=True,
            capture_output=True,
        )
        ids = [
            window_id
            for window_id in dashboard_window_ids(result.stdout)
            if window_id not in excluded_window_ids
        ]
        if result.returncode == 0 and ids:
            try:
                return resolve_x11_client_window(
                    tuple(ids),
                    runner=subprocess.run,
                )
            except AmbiguousWindowError as exc:
                raise RuntimeError(
                    "window search failed: "
                    f"title={window_title!r}, pid={process_id!r}, {exc}"
                ) from exc
            except WindowLayoutError as exc:
                last_error = str(exc)
                time.sleep(0.2)
                continue
        last_error = result.stderr.strip()
        time.sleep(0.2)
    raise RuntimeError(
        f"window not found: title={window_title!r}, pid={process_id}, detail={last_error or 'no match'}"
    )


def _select_dashboard_plot_tab(window_id: str, plot_tab: str) -> None:
    """从冻结的首个页签按索引切换旧页面，避免固定像素坐标。"""
    if plot_tab not in LEGACY_PLOT_TAB_ORDER:
        raise KeyError(f"unknown dashboard tab: {plot_tab}")
    target_indices = {
        "data": len(DASHBOARD_TAB_ORDER),
        "obstacles": DASHBOARD_TAB_ORDER.index("障碍物"),
        "trajectory": DASHBOARD_TAB_ORDER.index("轨迹"),
        "speed": DASHBOARD_TAB_ORDER.index("速度/命令"),
    }
    _run_xdotool(("windowfocus", window_id))
    for _index in range(target_indices[plot_tab]):
        _run_xdotool(("key", "ctrl+Tab"))
    time.sleep(0.5)


def _send_dashboard_up_key(window_id: str, hold_sec: float) -> None:
    """直接设置 X11 键盘焦点，不依赖窗口管理器的 activate 语义。"""
    _run_xdotool(["windowfocus", window_id])
    _run_xdotool(["keydown", "Up"])
    time.sleep(hold_sec)
    _run_xdotool(["keyup", "Up"])


def _positive_float(value: str) -> float:
    """为 argparse 提供清晰的正数错误信息。"""
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a positive number") from exc
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a positive number")
    return parsed


def _available_size(value: str) -> tuple[int, int]:
    """解析 WIDTHxHEIGHT，并拒绝零值、负值和含糊格式。"""
    match = re.fullmatch(r"([1-9][0-9]*)[xX]([1-9][0-9]*)", value)
    if match is None:
        raise argparse.ArgumentTypeError(
            "expected available size must use positive WIDTHxHEIGHT"
        )
    return int(match.group(1)), int(match.group(2))


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析 Dashboard 屏幕验收参数。"""
    parser = argparse.ArgumentParser(
        description="Verify enterprise Dashboard manual control and optional window layout."
    )
    parser.add_argument("--config", default="configs/stage1_golf_gui.yaml", help="GUI manual config to run.")
    parser.add_argument("--duration-sec", type=_positive_float, default=4.0, help="Manual run duration.")
    parser.add_argument("--hold-sec", type=_positive_float, default=4.0, help="How long to hold the dashboard up key.")
    parser.add_argument("--input-method", choices=["key"], default="key", help="Dashboard keyboard control method.")
    parser.add_argument(
        "--plot-tab",
        choices=list(LEGACY_PLOT_TAB_ORDER),
        default=None,
        help="Opt in to developer diagnostics and select a legacy data/plot tab.",
    )
    parser.add_argument(
        "--verify-window-layout",
        action="store_true",
        default=True,
        help="Verify PyBullet and Dashboard client rectangles against the primary available geometry.",
    )
    parser.add_argument(
        "--verify-dashboard-tabs",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Cycle and capture all 15 default enterprise Dashboard tabs (enabled by default).",
    )
    parser.add_argument(
        "--expected-available-size",
        type=_available_size,
        default=None,
        metavar="WIDTHxHEIGHT",
        help="Require the Qt primary available area to have this size.",
    )
    parser.add_argument("--window-timeout-sec", type=_positive_float, default=12.0, help="Seconds to wait for both GUI windows.")
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=None,
        help="Dedicated manual log directory; omitted uses a temporary directory.",
    )
    args = parser.parse_args(argv)
    if args.expected_available_size is not None and not args.verify_window_layout:
        parser.error("--expected-available-size requires --verify-window-layout")
    if args.verify_dashboard_tabs and args.plot_tab is not None:
        parser.error("--verify-dashboard-tabs cannot be combined with --plot-tab")
    return args


def _run_verification(args: argparse.Namespace, log_dir: Path) -> int:
    """在已确定的独立日志目录中执行一次真实 GUI 验收。"""
    log_dir.mkdir(parents=True, exist_ok=True)
    layout_report_path = log_dir / f"dashboard-layout-{uuid.uuid4().hex}.jsonl"
    started_at = time.time()
    command, child_env = build_child_command(
        args,
        log_dir=log_dir,
        layout_report_path=layout_report_path,
    )
    try:
        process = subprocess.Popen(command, env=child_env)
    except OSError as exc:
        print(f"failed to start manual GUI process: {exc}", file=sys.stderr)
        return 2

    qt_application = None
    try:
        claim_token = child_env[PYBULLET_WINDOW_TOKEN_ENV]
        owned_main = find_owned_x11_window(
            claim_token,
            expected_pid=process.pid,
            timeout_sec=args.window_timeout_sec,
            runner=subprocess.run,
        )
        main_window_id = owned_main.window_id
        print(
            "xres_owner "
            f"pid={owned_main.owner_pid} main_id={main_window_id} token={claim_token}"
        )
        dashboard_window_id = _find_window(
            DASHBOARD_WINDOW_TITLE,
            process.pid,
            args.window_timeout_sec,
        )
        main_geometry = _get_window_outer_geometry(main_window_id)
        dashboard_geometry = _get_window_outer_geometry(dashboard_window_id)

        if args.verify_window_layout:
            # 父进程也保留 QApplication，确保 availableGeometry 使用当前真实屏幕。
            from PySide6 import QtWidgets

            qt_application = (
                QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
            )
            display_metrics = primary_display_metrics()
            available = x11_available_geometry(display_metrics)
            print(
                "window_layout "
                f"available={_format_rect(available)} "
                f"main={_format_rect(main_geometry)} "
                f"dashboard={_format_rect(dashboard_geometry)}"
            )
            if args.expected_available_size is not None:
                expected_width, expected_height = args.expected_available_size
                actual_size = (available.width, available.height)
                if actual_size != args.expected_available_size:
                    raise ValueError(
                        "available geometry size mismatch: "
                        f"expected={expected_width}x{expected_height}, "
                        f"actual={available.width}x{available.height}, "
                        f"available={_format_rect(available)}"
                    )

            def geometry_getter() -> tuple[Rect, Rect]:
                return (
                    _get_window_outer_geometry(main_window_id),
                    _get_window_outer_geometry(dashboard_window_id),
                )

            verified_main, verified_dashboard = wait_for_window_layout(
                available,
                geometry_getter=geometry_getter,
                timeout_sec=args.window_timeout_sec,
                device_pixel_ratio=display_metrics.device_pixel_ratio,
            )
            if (verified_main, verified_dashboard) != (
                main_geometry,
                dashboard_geometry,
            ):
                print(
                    "window_layout "
                    f"available={_format_rect(available)} "
                    f"main={_format_rect(verified_main)} "
                    f"dashboard={_format_rect(verified_dashboard)}"
                )
            main_geometry = verified_main
            dashboard_geometry = verified_dashboard

        dashboard_client = _get_window_geometry(dashboard_window_id)
        if args.verify_dashboard_tabs:
            tab_result = verify_dashboard_tabs(
                dashboard_window_id,
                display=os.environ["DISPLAY"],
                client_rect=dashboard_client,
                layout_report_path=layout_report_path,
                hold_drive_sec=args.hold_sec,
                expected_tab_order=V2_DASHBOARD_TAB_ORDER,
            )
            print(
                "dashboard_tabs "
                f"tabs={tab_result.visited_tabs} "
                f"nonblank={tab_result.visited_tabs if tab_result.passed else tab_result.visited_tabs - 1} "
                f"min_ratio={tab_result.min_nonbackground_ratio:.6f} "
                f"detail={tab_result.detail}"
            )
            if not tab_result.passed:
                raise ValueError(tab_result.detail)
            _run_xdotool(("windowfocus", dashboard_window_id))
            _run_xdotool(("key", "Escape"))
        else:
            initial_report = wait_for_dashboard_layout_report(
                layout_report_path,
                0,
                V2_DASHBOARD_TAB_ORDER[0],
                DASHBOARD_LAYOUT_REPORT_TIMEOUT_SEC,
            ).report
            initial_layout = validate_dashboard_layout_report(
                initial_report,
                dashboard_client,
                expected_tab_order=(
                    (*V2_DASHBOARD_TAB_ORDER, DEVELOPER_DIAGNOSTIC_TAB_LABEL)
                    if args.plot_tab is not None
                    else V2_DASHBOARD_TAB_ORDER
                ),
            )
            if not initial_layout.passed:
                raise ValueError(
                    f"initial Dashboard layout failed: {initial_layout.detail}"
                )
            if args.plot_tab is not None:
                _select_dashboard_plot_tab(dashboard_window_id, args.plot_tab)
            _send_dashboard_up_key(dashboard_window_id, args.hold_sec)
        return_code = process.wait(timeout=process_wait_timeout(child_run_duration(args)))
        if return_code != 0:
            print(f"manual GUI process exited with {return_code}", file=sys.stderr)
            return return_code
    except Exception as exc:
        print(f"dashboard verification failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5.0)

    log_path = newest_manual_log(log_dir, after=started_at)
    if log_path is None:
        print("No new manual log was generated.", file=sys.stderr)
        return 1
    summary = summarize_manual_motion(log_path)
    print(
        "dashboard_manual_motion "
        f"log={summary.log_path} "
        f"dx={summary.dx:.4f} "
        f"max_cmd={summary.max_command_linear_velocity:.4f} "
        f"tail_v={summary.tail_body_forward_speed:.4f} "
        f"max_v={summary.max_body_forward_speed:.4f} "
        f"out_of_bounds={summary.out_of_bounds}"
    )
    return 0 if motion_passed(summary) else 1


def main(argv: Sequence[str] | None = None) -> int:
    """启动 GUI 手动模式，验证真实窗口后按日志结果返回验收状态。"""
    args = parse_args(argv)
    if not display_is_available():
        print("No X11/XWayland DISPLAY is available; pure Wayland is unsupported.", file=sys.stderr)
        return 2
    if shutil.which("xdotool") is None:
        print("xdotool is required for screen control.", file=sys.stderr)
        return 2
    if shutil.which("xwininfo") is None:
        print("xwininfo is required for Dashboard client-area geometry.", file=sys.stderr)
        return 2
    if shutil.which("xprop") is None:
        print("xprop is required for desktop workarea and window-frame geometry.", file=sys.stderr)
        return 2
    if args.verify_dashboard_tabs:
        try:
            from PIL import ImageGrab  # noqa: F401
        except ImportError as exc:
            print(f"Pillow ImageGrab is required for tab verification: {exc}", file=sys.stderr)
            return 2

    if args.log_dir is not None:
        return _run_verification(args, args.log_dir)
    with tempfile.TemporaryDirectory(prefix="slope-sim-dashboard-") as directory:
        return _run_verification(args, Path(directory))


if __name__ == "__main__":
    raise SystemExit(main())
