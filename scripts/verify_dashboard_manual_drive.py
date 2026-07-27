# Dashboard 手动验收脚本：打开 GUI，操作 Dashboard 控制区，并用日志确认车辆移动。
from __future__ import annotations

import argparse
from collections import Counter
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
    align_window_layout_to_scale,
    calculate_window_layout,
    find_owned_x11_window,
    primary_display_metrics,
    read_x11_outer_geometry,
    resolve_x11_client_window,
    search_x11_window_ids,
    x11_available_geometry,
)

DASHBOARD_LOGICAL_WIDTH = 420
DEFAULT_MANUAL_PREFIX = "manual_golf_heightfield_active_steering_4wd_0_"
DASHBOARD_TAB_ORDER = (
    "接口状态",
    "障碍物",
    "轨迹",
    "速度/命令",
    "打滑",
    "接触",
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
LEGACY_PLOT_TAB_ORDER = ("data", "obstacles", "trajectory", "speed", "slip", "contact")
DASHBOARD_TAB_X_OFFSETS = {
    "data": 34,
    "obstacles": 92,
    "trajectory": 155,
    "speed": 218,
    "slip": 280,
    "contact": 328,
}
DASHBOARD_TAB_Y_OFFSET = 62
DASHBOARD_CONTROL_SCROLL_BOTTOM_OFFSET = 80
DASHBOARD_UP_BUTTON_BOTTOM_OFFSET = 156
DASHBOARD_CONTROL_SCROLL_DOWN_STEPS = 20
DASHBOARD_CONTROL_SCROLL_DELAY_MS = 20
DASHBOARD_CONTROL_SCROLL_SETTLE_SEC = 0.2
WINDOW_LAYOUT_POLL_INTERVAL_SEC = 0.05
MANUAL_PROCESS_SHUTDOWN_GRACE_SEC = 20.0
DASHBOARD_LAYOUT_REPORT_TIMEOUT_SEC = 3.0


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
    """汇总 17 个 Dashboard 页面截图的非空门禁。"""

    passed: bool
    visited_tabs: int
    min_nonbackground_ratio: float
    detail: str


@dataclass(frozen=True, slots=True)
class DashboardLayoutVerification:
    """汇总单个页面布局报告的边界检查。"""

    passed: bool
    detail: str


def display_is_available(env: Mapping[str, str] | None = None) -> bool:
    """判断当前 shell 是否提供本验收工具所需的 X11/XWayland DISPLAY。"""
    values = os.environ if env is None else env
    return bool(values.get("DISPLAY"))


def dashboard_geometry_scale(geometry: WindowGeometry) -> float:
    """由物理窗口宽度推导 Qt 逻辑像素到 xdotool 坐标的缩放率。"""
    return geometry.width / DASHBOARD_LOGICAL_WIDTH


def dashboard_up_button_point(geometry: WindowGeometry) -> tuple[int, int]:
    """返回控制区滚到底后，侧栏中轴上的上箭头中心点。"""
    scale = dashboard_geometry_scale(geometry)
    bottom_offset = round(DASHBOARD_UP_BUTTON_BOTTOM_OFFSET * scale)
    return geometry.x + geometry.width // 2, geometry.y + geometry.height - bottom_offset


def dashboard_control_scroll_point(geometry: WindowGeometry) -> tuple[int, int]:
    """返回下方控制滚动视口内用于发送滚轮事件的稳定位置。"""
    scale = dashboard_geometry_scale(geometry)
    bottom_offset = round(DASHBOARD_CONTROL_SCROLL_BOTTOM_OFFSET * scale)
    return (
        geometry.x + geometry.width // 2,
        geometry.y + geometry.height - bottom_offset,
    )


def dashboard_plot_tab_point(geometry: WindowGeometry, plot_tab: str) -> tuple[int, int]:
    """按命名标签布局估算数据、障碍物或曲线页的标签中心点。"""
    if plot_tab not in LEGACY_PLOT_TAB_ORDER:
        raise KeyError(f"unknown dashboard tab: {plot_tab}")
    scale = dashboard_geometry_scale(geometry)
    return (
        geometry.x + round(DASHBOARD_TAB_X_OFFSETS[plot_tab] * scale),
        geometry.y + round(DASHBOARD_TAB_Y_OFFSET * scale),
    )


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
        "local",
    ]
    if args.plot_tab is not None:
        command.append("--developer-diagnostics")
    command.extend(("--log-dir", str(log_dir)))
    child_env = os.environ.copy()
    child_env[PYBULLET_WINDOW_TOKEN_ENV] = (
        f"pybullet-main-verifier-{os.getpid()}-{uuid.uuid4().hex}"
    )
    child_env[DASHBOARD_LAYOUT_REPORT_ENV] = str(layout_report_path)
    return command, child_env


def child_run_duration(args: argparse.Namespace) -> float:
    """页签验收留出交互预算；完成后 verifier 会主动发送退出键。"""
    if args.verify_dashboard_tabs:
        return max(args.duration_sec, args.hold_sec) + 20.0
    return args.duration_sec


def _frame_statistics(image: object) -> tuple[tuple[int, int, int], float]:
    """计算 RGB 通道跨度与相对主背景色的非背景像素比例。"""
    rgb = image.convert("RGB")
    raw = rgb.tobytes()
    pixels = list(zip(raw[0::3], raw[1::3], raw[2::3]))
    if not pixels:
        return (0, 0, 0), 0.0
    minimum = tuple(min(pixel[channel] for pixel in pixels) for channel in range(3))
    maximum = tuple(max(pixel[channel] for pixel in pixels) for channel in range(3))
    ranges = tuple(maximum[channel] - minimum[channel] for channel in range(3))
    background_count = Counter(pixels).most_common(1)[0][1]
    return ranges, 1.0 - background_count / len(pixels)


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
        channel_ranges, ratio = _frame_statistics(frame)
        minimum_ratio = min(minimum_ratio, ratio)
        varied_channels = sum(value >= 8 for value in channel_ranges)
        if varied_channels < 2 or ratio < 0.005:
            return DashboardTabVerification(
                False,
                index,
                minimum_ratio,
                f"tab {index} {label!r} is blank: ranges={channel_ranges}, ratio={ratio:.6f}",
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


def validate_dashboard_layout_report(
    report: Mapping[str, object],
    client_rect: Rect,
) -> DashboardLayoutVerification:
    """拒绝上下区域重叠、控件越界和不完整的 17 页布局报告。"""
    try:
        if report.get("tab_count") != len(DASHBOARD_TAB_ORDER):
            raise ValueError(
                f"tab count mismatch: {report.get('tab_count')!r}"
            )
        tab_label = report.get("tab_label")
        if tab_label not in DASHBOARD_TAB_ORDER:
            raise ValueError(f"unknown tab label: {tab_label!r}")
        _dashboard_report_geometry(report, client_rect)

        def report_rect(value: object, name: str) -> Rect:
            return _dashboard_report_rect_to_x11(
                report,
                value,
                name,
                client_rect,
            )

        tabs = report_rect(report.get("tabs_rect"), "tabs_rect")
        controls = report_rect(report.get("controls_rect"), "controls_rect")
        page = report_rect(report.get("page_rect"), "page_rect")
        if tabs.bottom > controls.y:
            raise ValueError("top tabs and controls overlap")
        for name, rect in (("tabs_rect", tabs), ("controls_rect", controls)):
            if not _rect_inside(rect, client_rect):
                raise ValueError(f"{name} is outside Dashboard client area")
        if not _rect_inside(page, tabs):
            raise ValueError("page_rect is outside tabs_rect")
        for key in ("canvas_rect", "legend_rect"):
            value = report.get(key)
            if value is not None and not _rect_inside(report_rect(value, key), page):
                raise ValueError(f"{key} is outside page_rect")
        for index, value in enumerate(report.get("plot_button_rects", ())):
            if not _rect_inside(report_rect(value, f"plot_button_rects[{index}]"), page):
                raise ValueError("plot button is outside page_rect")
        critical = report.get("critical_control_rects", {})
        if not isinstance(critical, Mapping):
            raise ValueError("critical_control_rects must be a mapping")
        for name, value in critical.items():
            if not _rect_inside(report_rect(value, f"critical control {name}"), controls):
                raise ValueError(f"critical control {name!r} is outside controls_rect")
    except (TypeError, ValueError) as exc:
        return DashboardLayoutVerification(False, str(exc))
    return DashboardLayoutVerification(True, "dashboard layout report passed")


def wait_for_dashboard_layout_report(
    path: Path,
    tab_index: int,
    tab_label: str,
    timeout_sec: float,
) -> Mapping[str, object]:
    """有界等待 Dashboard 为指定页签追加完整 JSONL 报告。"""
    deadline = time.monotonic() + timeout_sec
    last_detail = "report file does not exist"
    while time.monotonic() < deadline:
        if path.exists():
            try:
                lines = path.read_text(encoding="utf-8").splitlines()
            except OSError as exc:
                last_detail = str(exc)
            else:
                for line in reversed(lines):
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
                        return report
                last_detail = (
                    f"no report for tab_index={tab_index}, tab_label={tab_label!r}"
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
    report_reader: Callable[[Path, int, str, float], Mapping[str, object]] = wait_for_dashboard_layout_report,
    capture: Callable[..., object] | None = None,
    command_runner: Callable[[Sequence[str]], object] | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
) -> DashboardTabVerification:
    """按住前进键遍历 17 页，逐页验证布局报告和实际客户区像素。"""
    if not isinstance(client_rect, Rect):
        raise TypeError("client_rect must be a Rect")
    if not math.isfinite(hold_drive_sec) or hold_drive_sec <= 0.0:
        raise ValueError("hold_drive_sec must be positive and finite")
    if capture is None:
        from PIL import ImageGrab

        capture = ImageGrab.grab
    run_command = _run_xdotool if command_runner is None else command_runner

    frames: list[object] = []
    started_at = clock()
    run_command(("windowfocus", window_id))
    try:
        run_command(("keydown", "Up"))
        for index, label in enumerate(DASHBOARD_TAB_ORDER):
            if index:
                run_command(("keydown", "ctrl"))
                try:
                    run_command(("key", "Tab"))
                finally:
                    run_command(("keyup", "ctrl"))
            report = report_reader(
                layout_report_path,
                index,
                label,
                DASHBOARD_LAYOUT_REPORT_TIMEOUT_SEC,
            )
            if report.get("tab_index") != index or report.get("tab_label") != label:
                return DashboardTabVerification(
                    False,
                    index + 1,
                    0.0,
                    f"tab {index + 1} report order mismatch: expected={label!r}",
                )
            reported_order = report.get("tab_order")
            if reported_order is not None and tuple(reported_order) != DASHBOARD_TAB_ORDER:
                return DashboardTabVerification(
                    False,
                    index + 1,
                    0.0,
                    f"tab {index + 1} reported a different top-level order",
                )
            layout_result = validate_dashboard_layout_report(report, client_rect)
            if not layout_result.passed:
                return DashboardTabVerification(
                    False,
                    index + 1,
                    0.0,
                    f"tab {index + 1} {label!r} layout failed: {layout_result.detail}",
                )
            target_value = report.get("canvas_rect") or report.get("page_rect")
            target = _dashboard_report_rect_to_x11(
                report,
                target_value,
                "capture_rect",
                client_rect,
            )
            frames.append(
                capture(
                    bbox=(target.x, target.y, target.right, target.bottom),
                    xdisplay=display,
                )
            )
        remaining = hold_drive_sec - (clock() - started_at)
        if remaining > 0.0:
            sleeper(remaining)
        return verify_dashboard_frames(frames, DASHBOARD_TAB_ORDER)
    finally:
        try:
            run_command(("keyup", "ctrl"))
        finally:
            run_command(("keyup", "Up"))


def _format_rect(rect: Rect) -> str:
    """把窗口矩形格式化为稳定、适合验收日志阅读的文本。"""
    return f"(x={rect.x},y={rect.y},width={rect.width},height={rect.height})"


def validate_window_layout(
    available: Rect,
    main: Rect,
    dashboard: Rect,
    *,
    device_pixel_ratio: float = 1.0,
) -> None:
    """核对两窗精确命中共享布局，并显式检查覆盖、对齐和重叠。"""
    expected = align_window_layout_to_scale(
        calculate_window_layout(available, True),
        device_pixel_ratio,
    )
    expected_dashboard = expected.dashboard
    if expected_dashboard is None:
        raise ValueError("window layout verification failed: expected dashboard is missing")

    reasons: list[str] = []
    if main != expected.main:
        reasons.append("main mismatch")
    if dashboard != expected_dashboard:
        reasons.append("dashboard mismatch")
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

    if reasons:
        raise ValueError(
            "window layout verification failed "
            f"({'; '.join(reasons)}): "
            f"expected_main={_format_rect(expected.main)} "
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


def _click_dashboard_up(window_id: str, hold_sec: float) -> None:
    """先滚到底部控制组，再按住 Dashboard 上箭头一小段时间。"""
    geometry = _get_window_geometry(window_id)
    scroll_x, scroll_y = dashboard_control_scroll_point(geometry)
    up_x, up_y = dashboard_up_button_point(geometry)
    _run_xdotool(["mousemove", str(scroll_x), str(scroll_y)])
    _run_xdotool(
        [
            "click",
            "--repeat",
            str(DASHBOARD_CONTROL_SCROLL_DOWN_STEPS),
            "--delay",
            str(DASHBOARD_CONTROL_SCROLL_DELAY_MS),
            "5",
        ]
    )
    time.sleep(DASHBOARD_CONTROL_SCROLL_SETTLE_SEC)
    _run_xdotool(["mousemove", str(up_x), str(up_y)])
    _run_xdotool(["mousedown", "1"])
    time.sleep(hold_sec)
    _run_xdotool(["mouseup", "1"])


def _select_dashboard_plot_tab(window_id: str, plot_tab: str) -> None:
    """激活 Dashboard 并点击指定曲线标签，复现曲线可见时的控制链路。"""
    geometry = _get_window_geometry(window_id)
    x, y = dashboard_plot_tab_point(geometry, plot_tab)
    _run_xdotool(["mousemove", str(x), str(y)])
    _run_xdotool(["click", "1"])
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
    parser.add_argument("--hold-sec", type=_positive_float, default=4.0, help="How long to hold the dashboard up button.")
    parser.add_argument("--input-method", choices=["key", "button"], default="key", help="Dashboard screen control method.")
    parser.add_argument(
        "--plot-tab",
        choices=list(DASHBOARD_TAB_X_OFFSETS),
        default=None,
        help="Opt in to developer diagnostics and select a legacy data/plot tab.",
    )
    parser.add_argument(
        "--verify-window-layout",
        action="store_true",
        help="Verify PyBullet and Dashboard client rectangles against the primary available geometry.",
    )
    parser.add_argument(
        "--verify-dashboard-tabs",
        action="store_true",
        help="Cycle and capture all 17 enterprise Dashboard tabs.",
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

        if args.verify_dashboard_tabs:
            dashboard_client = _get_window_geometry(dashboard_window_id)
            tab_result = verify_dashboard_tabs(
                dashboard_window_id,
                display=os.environ["DISPLAY"],
                client_rect=dashboard_client,
                layout_report_path=layout_report_path,
                hold_drive_sec=args.hold_sec,
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
        elif args.plot_tab is not None:
            _select_dashboard_plot_tab(dashboard_window_id, args.plot_tab)
        if not args.verify_dashboard_tabs:
            if args.input_method == "button":
                _click_dashboard_up(dashboard_window_id, args.hold_sec)
            else:
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
