# 窗口布局适配：计算主窗与 Dashboard 几何，并隔离 Qt、PyBullet 和 X11 副作用。
from __future__ import annotations

from collections.abc import Callable
import ctypes
import ctypes.util
from dataclasses import dataclass
import math
import os
import re
import subprocess
import time
import uuid


PYBULLET_WINDOW_TITLE = (
    "Bullet Physics ExampleBrowser using OpenGL3+ [btgl] Release build"
)
PYBULLET_WINDOW_TOKEN_ENV = "SLOPE_SIM_PYBULLET_WINDOW_TOKEN"
_XRES_CLIENT_ID_PID_MASK = 0x02
_XRES_CLIENT_ID_PID_TYPE = 1


class WindowLayoutError(RuntimeError):
    """表示屏幕读取、GUI 连接或窗口几何应用失败。"""


class AmbiguousWindowError(WindowLayoutError):
    """表示候选包含多个独立窗口族，继续操作可能误控其他进程。"""


@dataclass(frozen=True)
class Rect:
    """使用绝对坐标和正客户区尺寸描述一个矩形。"""

    x: int
    y: int
    width: int
    height: int

    def __post_init__(self) -> None:
        for name in ("x", "y", "width", "height"):
            value = getattr(self, name)
            if type(value) is not int:
                raise TypeError(f"{name} must be an integer")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be positive")

    @property
    def right(self) -> int:
        """返回矩形右侧的排他坐标。"""
        return self.x + self.width

    @property
    def bottom(self) -> int:
        """返回矩形底部的排他坐标。"""
        return self.y + self.height


@dataclass(frozen=True)
class WindowLayout:
    """保存 PyBullet 主窗口和可选 Dashboard 的初始矩形。"""

    main: Rect
    dashboard: Rect | None

    def __post_init__(self) -> None:
        if not isinstance(self.main, Rect):
            raise TypeError("main must be a Rect")
        if self.dashboard is not None and not isinstance(self.dashboard, Rect):
            raise TypeError("dashboard must be a Rect or None")


@dataclass(frozen=True)
class FrameExtents:
    """描述客户区四周由窗口管理器添加的非负边框尺寸。"""

    left: int
    right: int
    top: int
    bottom: int

    def __post_init__(self) -> None:
        for name in ("left", "right", "top", "bottom"):
            value = getattr(self, name)
            if type(value) is not int:
                raise TypeError(f"{name} must be an integer")
            if value < 0:
                raise ValueError(f"{name} must be non-negative")


@dataclass(frozen=True)
class DisplayMetrics:
    """保存 Qt 主屏逻辑几何及其到 X11 物理像素的缩放率。"""

    screen: Rect
    available: Rect
    device_pixel_ratio: float

    def __post_init__(self) -> None:
        if not isinstance(self.screen, Rect) or not isinstance(self.available, Rect):
            raise TypeError("screen and available must be Rect values")
        _positive_scale(self.device_pixel_ratio, "device_pixel_ratio")


@dataclass(frozen=True)
class _CommandResult:
    """将可注入 runner 的结果收敛为稳定的文本命令结果。"""

    returncode: int
    stdout: str
    stderr: str


@dataclass(frozen=True)
class OwnedX11Window:
    """记录经过 XRes client PID 验证的唯一 X11 客户窗。"""

    window_id: str
    owner_pid: int
    title: str


class _XResClientIdSpec(ctypes.Structure):
    """对应 libXRes 的 XResClientIdSpec ABI。"""

    _fields_ = [
        ("client", ctypes.c_ulong),
        ("mask", ctypes.c_uint),
    ]


class _XResClientIdValue(ctypes.Structure):
    """对应 libXRes 返回的 XResClientIdValue ABI。"""

    _fields_ = [
        ("spec", _XResClientIdSpec),
        ("length", ctypes.c_long),
        ("value", ctypes.c_void_p),
    ]


Runner = Callable[..., object]
Clock = Callable[[], float]
Sleeper = Callable[[float], None]


def _positive_scale(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a positive finite number")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return normalized


def calculate_window_layout(available: Rect, dashboard_enabled: bool) -> WindowLayout:
    """按可用工作区计算左侧 80% 主窗和右侧 20% Dashboard。"""
    if not isinstance(available, Rect):
        raise TypeError("available must be a Rect")
    if type(dashboard_enabled) is not bool:
        raise TypeError("dashboard_enabled must be a bool")
    if not dashboard_enabled:
        return WindowLayout(main=available, dashboard=None)

    dashboard_width = available.width // 5
    if dashboard_width <= 0:
        raise WindowLayoutError("available geometry is too narrow for a dashboard")
    main_width = available.width - dashboard_width
    return WindowLayout(
        main=Rect(available.x, available.y, main_width, available.height),
        dashboard=Rect(
            available.x + main_width,
            available.y,
            dashboard_width,
            available.height,
        ),
    )


def align_window_layout_to_scale(
    layout: WindowLayout,
    device_pixel_ratio: float,
) -> WindowLayout:
    """把右栏宽度对齐到 Qt 可表示的物理像素，同时保持无缝覆盖。"""
    if not isinstance(layout, WindowLayout):
        raise TypeError("layout must be a WindowLayout")
    scale = _positive_scale(device_pixel_ratio, "device_pixel_ratio")
    if layout.dashboard is None:
        return layout

    available_width = layout.main.width + layout.dashboard.width
    target_dashboard_width = available_width / 5.0
    logical_dashboard_width = max(1, round(target_dashboard_width / scale))
    dashboard_width = max(1, int(round(logical_dashboard_width * scale)))
    if dashboard_width >= available_width:
        raise WindowLayoutError("available geometry is too narrow for dashboard scale")
    dashboard_right = layout.dashboard.right
    dashboard_x = dashboard_right - dashboard_width
    return WindowLayout(
        main=Rect(
            layout.main.x,
            layout.main.y,
            dashboard_x - layout.main.x,
            layout.main.height,
        ),
        dashboard=Rect(
            dashboard_x,
            layout.dashboard.y,
            dashboard_width,
            layout.dashboard.height,
        ),
    )


def logical_client_rect_for_outer(
    outer: Rect,
    *,
    screen: Rect,
    device_pixel_ratio: float,
    frame_extents: FrameExtents,
) -> Rect:
    """把 X11 物理外框换算成 Qt 逻辑客户区位置和固定尺寸。"""
    if not isinstance(outer, Rect) or not isinstance(screen, Rect):
        raise TypeError("outer and screen must be Rect values")
    if not isinstance(frame_extents, FrameExtents):
        raise TypeError("frame_extents must be FrameExtents")
    scale = _positive_scale(device_pixel_ratio, "device_pixel_ratio")
    frame_x = screen.x + round((outer.x - screen.x) / scale)
    frame_y = screen.y + round((outer.y - screen.y) / scale)
    frame_right = screen.x + round((outer.right - screen.x) / scale)
    frame_bottom = screen.y + round((outer.bottom - screen.y) / scale)
    outer_width = frame_right - frame_x
    outer_height = frame_bottom - frame_y
    return Rect(
        frame_x,
        frame_y,
        outer_width - frame_extents.left - frame_extents.right,
        outer_height - frame_extents.top - frame_extents.bottom,
    )


def primary_available_geometry() -> Rect:
    """懒读取主屏排除任务栏后的可用区域，不负责创建 Qt application。"""
    try:
        from PySide6 import QtGui
    except Exception as exc:
        raise WindowLayoutError(f"PySide6 is unavailable: {exc}") from exc

    try:
        application = QtGui.QGuiApplication.instance()
    except Exception as exc:
        raise WindowLayoutError(f"cannot access the Qt GUI application: {exc}") from exc
    if application is None:
        raise WindowLayoutError("Qt GUI application is not initialized")

    try:
        screen = application.primaryScreen()
    except Exception as exc:
        raise WindowLayoutError(f"cannot read the primary screen: {exc}") from exc
    if screen is None:
        raise WindowLayoutError("Qt primary screen is unavailable")

    try:
        geometry = screen.availableGeometry()
        values = {
            "x": geometry.x(),
            "y": geometry.y(),
            "width": geometry.width(),
            "height": geometry.height(),
        }
        return Rect(**values)
    except (AttributeError, TypeError, ValueError) as exc:
        raise WindowLayoutError(f"invalid primary available geometry: {exc}") from exc


def primary_display_metrics() -> DisplayMetrics:
    """读取主屏逻辑几何、逻辑可用区和设备像素比。"""
    try:
        from PySide6 import QtGui
    except Exception as exc:
        raise WindowLayoutError(f"PySide6 is unavailable: {exc}") from exc

    application = QtGui.QGuiApplication.instance()
    if application is None:
        raise WindowLayoutError("Qt GUI application is not initialized")
    screen = application.primaryScreen()
    if screen is None:
        raise WindowLayoutError("Qt primary screen is unavailable")
    try:
        geometry = screen.geometry()
        available = screen.availableGeometry()
        return DisplayMetrics(
            screen=Rect(
                geometry.x(),
                geometry.y(),
                geometry.width(),
                geometry.height(),
            ),
            available=Rect(
                available.x(),
                available.y(),
                available.width(),
                available.height(),
            ),
            device_pixel_ratio=float(screen.devicePixelRatio()),
        )
    except (AttributeError, TypeError, ValueError) as exc:
        raise WindowLayoutError(f"invalid primary display metrics: {exc}") from exc


def connect_pybullet_gui(rect: Rect, pybullet_module: object | None = None) -> int:
    """用确定的客户区宽高连接 PyBullet GUI，并返回有效 client id。"""
    if not isinstance(rect, Rect):
        raise TypeError("rect must be a Rect")

    module = pybullet_module
    if module is None:
        try:
            import pybullet as module
        except Exception as exc:
            raise WindowLayoutError(f"cannot import PyBullet: {exc}") from exc

    options = f"--width={rect.width} --height={rect.height}"
    try:
        gui_mode = getattr(module, "GUI")
        connect = getattr(module, "connect")
        client_id = connect(gui_mode, options=options)
    except Exception as exc:
        raise WindowLayoutError(f"failed to connect to PyBullet GUI: {exc}") from exc
    if type(client_id) is not int or client_id < 0:
        raise WindowLayoutError(
            f"failed to connect to PyBullet GUI: invalid client id {client_id!r}"
        )
    return client_id


def parse_xdotool_window_ids(output: str) -> tuple[str, ...]:
    """从 xdotool search 文本中提取去重后的正十进制窗口 id。"""
    if type(output) is not str:
        raise TypeError("xdotool output must be text")
    window_ids: list[str] = []
    seen: set[str] = set()
    for line in output.splitlines():
        candidate = line.strip()
        if re.fullmatch(r"[0-9]+", candidate) is None or int(candidate) <= 0:
            continue
        if candidate not in seen:
            seen.add(candidate)
            window_ids.append(candidate)
    return tuple(window_ids)


def parse_xwininfo_geometry(output: str) -> Rect:
    """严格解析 xwininfo 报告的客户区绝对原点、宽度和高度。"""
    if type(output) is not str:
        raise WindowLayoutError("invalid xwininfo output: expected text")
    field_names = {
        "Absolute upper-left X": "x",
        "Absolute upper-left Y": "y",
        "Width": "width",
        "Height": "height",
    }
    values: dict[str, int] = {}
    for line in output.splitlines():
        key, separator, raw_value = line.strip().partition(":")
        name = field_names.get(key)
        if not separator or name is None:
            continue
        value = raw_value.strip()
        if name in values:
            raise WindowLayoutError(f"invalid xwininfo output: duplicate {key}")
        if re.fullmatch(r"[+-]?[0-9]+", value) is None:
            raise WindowLayoutError(f"invalid xwininfo output: non-integer {key}")
        values[name] = int(value)

    missing = tuple(name for name in ("x", "y", "width", "height") if name not in values)
    if missing:
        raise WindowLayoutError(
            f"invalid xwininfo output: missing {', '.join(missing)}"
        )
    try:
        return Rect(**values)
    except (TypeError, ValueError) as exc:
        raise WindowLayoutError(f"invalid xwininfo geometry: {exc}") from exc


def parse_xwininfo_parent_id(output: str) -> str:
    """解析 xwininfo 的父窗口 ID，并统一为十进制文本。"""
    if type(output) is not str:
        raise WindowLayoutError("invalid xwininfo output: expected text")
    match = re.search(
        r"^\s*Parent window id:\s+(0x[0-9a-fA-F]+|[0-9]+)\b",
        output,
        flags=re.MULTILINE,
    )
    if match is None:
        raise WindowLayoutError("invalid xwininfo output: missing Parent window id")
    return str(int(match.group(1), 0))


def parse_xprop_frame_extents(output: str) -> FrameExtents:
    """解析 EWMH 客户窗边框；无窗口管理器时返回零边框。"""
    if type(output) is not str:
        raise WindowLayoutError("invalid xprop frame extents: expected text")
    if "no such atom" in output.lower() or "not found" in output.lower():
        return FrameExtents(0, 0, 0, 0)
    match = re.search(
        r"^_NET_FRAME_EXTENTS(?:\([^)]*\))?\s*=\s*([^\n]+)$",
        output,
        flags=re.MULTILINE,
    )
    if match is None:
        raise WindowLayoutError("invalid xprop frame extents: missing property")
    values = [part.strip() for part in match.group(1).split(",")]
    if len(values) != 4 or any(re.fullmatch(r"[0-9]+", value) is None for value in values):
        raise WindowLayoutError("invalid xprop frame extents: expected four integers")
    try:
        return FrameExtents(*(int(value) for value in values))
    except (TypeError, ValueError) as exc:
        raise WindowLayoutError(f"invalid xprop frame extents: {exc}") from exc


def _frame_extents_property_is_missing(output: str) -> bool:
    lowered = output.lower()
    return "no such atom" in lowered or "not found" in lowered


def outer_rect_from_client(client: Rect, frame_extents: FrameExtents) -> Rect:
    """按 EWMH 边框把客户区回读转换为用户看到的完整窗口外框。"""
    if not isinstance(client, Rect) or not isinstance(frame_extents, FrameExtents):
        raise TypeError("client and frame_extents have invalid types")
    return Rect(
        client.x - frame_extents.left,
        client.y - frame_extents.top,
        client.width + frame_extents.left + frame_extents.right,
        client.height + frame_extents.top + frame_extents.bottom,
    )


def _parse_xprop_workarea(output: str) -> Rect | None:
    """解析当前桌面的 EWMH 工作区；无窗口管理器时返回 None。"""
    if type(output) is not str:
        raise WindowLayoutError("invalid xprop workarea: expected text")
    lowered = output.lower()
    if "_net_workarea" not in lowered or "no such atom" in lowered or "not found" in lowered:
        return None

    desktop_match = re.search(
        r"^_NET_CURRENT_DESKTOP(?:\([^)]*\))?\s*=\s*([0-9]+)\s*$",
        output,
        flags=re.MULTILINE,
    )
    workarea_match = re.search(
        r"^_NET_WORKAREA(?:\([^)]*\))?\s*=\s*([^\n]+)$",
        output,
        flags=re.MULTILINE,
    )
    if desktop_match is None or workarea_match is None:
        raise WindowLayoutError("invalid xprop workarea: missing desktop or workarea")
    raw_values = [part.strip() for part in workarea_match.group(1).split(",")]
    if not raw_values or any(re.fullmatch(r"[+-]?[0-9]+", value) is None for value in raw_values):
        raise WindowLayoutError("invalid xprop workarea: expected integers")
    values = [int(value) for value in raw_values]
    desktop_index = int(desktop_match.group(1))
    start = desktop_index * 4
    if len(values) % 4 != 0 or start + 4 > len(values):
        raise WindowLayoutError("invalid xprop workarea: desktop index is out of range")
    try:
        return Rect(*values[start : start + 4])
    except (TypeError, ValueError) as exc:
        raise WindowLayoutError(f"invalid xprop workarea: {exc}") from exc


def _parse_current_desktop(output: str) -> int:
    match = re.search(
        r"^_NET_CURRENT_DESKTOP(?:\([^)]*\))?\s*=\s*([0-9]+)\s*$",
        output,
        flags=re.MULTILINE,
    )
    if match is None:
        raise WindowLayoutError("invalid xprop workarea: missing current desktop")
    return int(match.group(1))


def _parse_gtk_workareas(output: str, property_name: str) -> tuple[Rect, ...]:
    lowered = output.lower()
    if "no such atom" in lowered or "not found" in lowered:
        return ()
    match = re.search(
        rf"^{re.escape(property_name)}(?:\([^)]*\))?\s*=\s*([^\n]+)$",
        output,
        flags=re.MULTILINE,
    )
    if match is None:
        return ()
    raw_values = [part.strip() for part in match.group(1).split(",")]
    if len(raw_values) % 4 != 0 or any(
        re.fullmatch(r"[+-]?[0-9]+", value) is None for value in raw_values
    ):
        raise WindowLayoutError("invalid GTK workareas: expected rectangle tuples")
    values = [int(value) for value in raw_values]
    try:
        return tuple(
            Rect(*values[index : index + 4])
            for index in range(0, len(values), 4)
        )
    except (TypeError, ValueError) as exc:
        raise WindowLayoutError(f"invalid GTK workareas: {exc}") from exc


def _run_command(command: list[str], runner: Runner) -> _CommandResult:
    """统一执行外部工具，并把工具缺失及 runner 契约错误转为领域异常。"""
    try:
        completed = runner(
            command,
            check=False,
            text=True,
            capture_output=True,
        )
    except FileNotFoundError as exc:
        raise WindowLayoutError(
            f"required window tool {command[0]!r} was not found"
        ) from exc
    except (OSError, subprocess.SubprocessError) as exc:
        raise WindowLayoutError(
            f"failed to execute window command {' '.join(command)}: {exc}"
        ) from exc

    try:
        returncode = completed.returncode
        stdout = completed.stdout
        stderr = completed.stderr
    except AttributeError as exc:
        raise WindowLayoutError("window command runner returned an invalid result") from exc
    if type(returncode) is not int:
        raise WindowLayoutError("window command runner returned an invalid return code")
    if stdout is None:
        stdout = ""
    if stderr is None:
        stderr = ""
    if type(stdout) is not str or type(stderr) is not str:
        raise WindowLayoutError("window command runner returned non-text output")
    return _CommandResult(returncode, stdout, stderr)


def _command_failure(command: list[str], result: _CommandResult) -> WindowLayoutError:
    detail = result.stderr.strip() or result.stdout.strip() or "no diagnostic output"
    return WindowLayoutError(
        f"window command failed ({' '.join(command)}), "
        f"returncode={result.returncode}: {detail}"
    )


def _with_unapplied_geometry_context(
    error: WindowLayoutError,
    expected: Rect,
) -> WindowLayoutError:
    """保留早期失败诊断，并补充尚未取得实际几何时的目标上下文。"""
    return WindowLayoutError(f"{error}; expected={expected!r}, actual=None")


def _run_required(command: list[str], runner: Runner) -> _CommandResult:
    result = _run_command(command, runner)
    if result.returncode != 0:
        raise _command_failure(command, result)
    return result


def search_x11_window_ids(
    window_title: str,
    *,
    only_visible: bool,
    runner: Runner | None = None,
) -> tuple[str, ...]:
    """按精确标题读取窗口 ID，可用于启动前记录已有窗口。"""
    if type(window_title) is not str or not window_title:
        raise ValueError("window_title must be non-empty text")
    if type(only_visible) is not bool:
        raise TypeError("only_visible must be a bool")
    command = ["xdotool", "search", "--all"]
    if only_visible:
        command.append("--onlyvisible")
    command.extend(("--name", rf"^{re.escape(window_title)}$"))
    command_runner = subprocess.run if runner is None else runner
    result = _run_command(command, command_runner)
    if result.returncode == 0:
        return parse_xdotool_window_ids(result.stdout)
    if result.returncode == 1:
        return ()
    raise _command_failure(command, result)


def _intersect_rect(first: Rect, second: Rect) -> Rect:
    x = max(first.x, second.x)
    y = max(first.y, second.y)
    right = min(first.right, second.right)
    bottom = min(first.bottom, second.bottom)
    if right <= x or bottom <= y:
        raise WindowLayoutError("X11 workarea does not intersect the primary screen")
    return Rect(x, y, right - x, bottom - y)


def x11_available_geometry(
    metrics: DisplayMetrics,
    *,
    runner: Runner | None = None,
) -> Rect:
    """读取当前桌面物理工作区，并裁剪到 Qt 识别的主显示器。"""
    if not isinstance(metrics, DisplayMetrics):
        raise TypeError("metrics must be DisplayMetrics")
    command_runner = subprocess.run if runner is None else runner
    result = _run_required(
        ["xprop", "-root", "_NET_CURRENT_DESKTOP", "_NET_WORKAREA"],
        command_runner,
    )
    workarea = _parse_xprop_workarea(result.stdout)
    scale = metrics.device_pixel_ratio
    physical_screen = Rect(
        metrics.screen.x,
        metrics.screen.y,
        round(metrics.screen.width * scale),
        round(metrics.screen.height * scale),
    )
    if workarea is None:
        return Rect(
            metrics.available.x,
            metrics.available.y,
            round(metrics.available.width * scale),
            round(metrics.available.height * scale),
        )
    desktop_index = _parse_current_desktop(result.stdout)
    gtk_property = f"_GTK_WORKAREAS_D{desktop_index}"
    gtk_result = _run_required(
        ["xprop", "-root", gtk_property],
        command_runner,
    )
    gtk_workareas = _parse_gtk_workareas(gtk_result.stdout, gtk_property)
    intersections: list[Rect] = []
    for candidate in gtk_workareas:
        try:
            intersections.append(_intersect_rect(candidate, physical_screen))
        except WindowLayoutError:
            continue
    if intersections:
        return max(intersections, key=lambda rect: rect.width * rect.height)
    return _intersect_rect(workarea, physical_screen)


def read_x11_frame_extents(
    window_id: str,
    *,
    runner: Runner | None = None,
) -> FrameExtents:
    """读取客户窗的 EWMH 外框尺寸，无窗口管理器时返回零。"""
    command_runner = subprocess.run if runner is None else runner
    result = _run_required(
        ["xprop", "-id", window_id, "_NET_FRAME_EXTENTS"],
        command_runner,
    )
    return parse_xprop_frame_extents(result.stdout)


def x11_window_manager_available(*, runner: Runner | None = None) -> bool:
    """通过 EWMH 根窗口属性判断当前 X11 是否存在窗口管理器。"""
    command_runner = subprocess.run if runner is None else runner
    result = _run_required(
        ["xprop", "-root", "_NET_SUPPORTING_WM_CHECK"],
        command_runner,
    )
    if _frame_extents_property_is_missing(result.stdout):
        return False
    if re.search(r"^_NET_SUPPORTING_WM_CHECK(?:\([^)]*\))?\s*[:=]", result.stdout, re.MULTILINE):
        return True
    raise WindowLayoutError("invalid X11 window-manager property")


def wait_for_x11_frame_extents(
    window_id: str,
    *,
    timeout_sec: float,
    poll_interval_sec: float,
    runner: Runner,
    clock: Clock,
    sleeper: Sleeper,
) -> FrameExtents:
    """有 WM 时等待边框属性连续两次一致；无 WM 时立即返回零。"""
    deadline = clock() + timeout_sec
    previous: FrameExtents | None = None
    must_wait = False

    while True:
        result = _run_required(
            ["xprop", "-id", window_id, "_NET_FRAME_EXTENTS"],
            runner,
        )
        if _frame_extents_property_is_missing(result.stdout):
            if previous is None and not must_wait:
                if not x11_window_manager_available(runner=runner):
                    return FrameExtents(0, 0, 0, 0)
                must_wait = True
            previous = None
        else:
            current = parse_xprop_frame_extents(result.stdout)
            if current == previous:
                return current
            previous = current

        if not _sleep_before_retry(
            deadline=deadline,
            poll_interval_sec=poll_interval_sec,
            clock=clock,
            sleeper=sleeper,
        ):
            raise WindowLayoutError(
                f"X11 frame extents did not stabilize for window_id={window_id}"
            )


def read_x11_outer_geometry(
    window_id: str,
    *,
    runner: Runner | None = None,
) -> Rect:
    """回读客户区与外框尺寸，并返回完整窗口的 X11 物理矩形。"""
    command_runner = subprocess.run if runner is None else runner
    geometry = _run_required(["xwininfo", "-id", window_id], command_runner)
    client = parse_xwininfo_geometry(geometry.stdout)
    extents = read_x11_frame_extents(window_id, runner=command_runner)
    return outer_rect_from_client(client, extents)


def wait_for_x11_outer_geometry(
    window_id: str,
    expected: Rect,
    *,
    timeout_sec: float = 1.0,
    poll_interval_sec: float = 0.02,
    runner: Runner | None = None,
    clock: Clock = time.monotonic,
    sleeper: Sleeper = time.sleep,
) -> Rect:
    """有界等待完整外框命中目标，重试窗口管理器的短暂过渡状态。"""
    if not isinstance(expected, Rect):
        raise TypeError("expected must be a Rect")
    timeout = _positive_seconds(timeout_sec, "timeout_sec")
    poll_interval = _positive_seconds(poll_interval_sec, "poll_interval_sec")
    command_runner = subprocess.run if runner is None else runner
    deadline = clock() + timeout
    last_actual: Rect | None = None
    last_error: WindowLayoutError | None = None
    while True:
        try:
            last_actual = read_x11_outer_geometry(
                window_id,
                runner=command_runner,
            )
            last_error = None
        except WindowLayoutError as exc:
            last_error = exc
        if last_actual == expected and last_error is None:
            return last_actual
        if not _sleep_before_retry(
            deadline=deadline,
            poll_interval_sec=poll_interval,
            clock=clock,
            sleeper=sleeper,
        ):
            detail = (
                f"last_error={last_error}"
                if last_error is not None
                else f"actual={last_actual!r}"
            )
            raise WindowLayoutError(
                "X11 outer window geometry did not stabilize: "
                f"expected={expected!r}, {detail}"
            )


def _positive_seconds(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a positive finite number")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return normalized


def _sleep_before_retry(
    *,
    deadline: float,
    poll_interval_sec: float,
    clock: Clock,
    sleeper: Sleeper,
) -> bool:
    """仅在条件尚未满足且截止时间未到时等待下一次检查。"""
    remaining = deadline - clock()
    if remaining <= 0.0:
        return False
    sleeper(min(poll_interval_sec, remaining))
    return True


def resolve_x11_client_window(
    window_ids: tuple[str, ...],
    *,
    runner: Runner | None = None,
) -> str:
    """从同名候选中识别被窗口管理器重父化的唯一客户窗。"""
    clients = _resolve_x11_client_windows(window_ids, runner=runner)
    if len(clients) == 1:
        return clients[0]
    raise AmbiguousWindowError(
        "ambiguous X11 window candidates: "
        f"candidate_ids={', '.join(window_ids)}, "
        f"client_ids={', '.join(clients) or 'none'}"
    )


def _resolve_x11_client_windows(
    window_ids: tuple[str, ...],
    *,
    runner: Runner | None = None,
) -> tuple[str, ...]:
    """移除同一重父化窗口族中的 WM frame，保留可做所有权查询的 client。"""
    if not window_ids:
        raise WindowLayoutError("X11 window candidates must not be empty")
    if len(window_ids) == 1:
        return window_ids

    command_runner = subprocess.run if runner is None else runner
    candidate_set = set(window_ids)
    frame_ids: set[str] = set()
    for window_id in window_ids:
        result = _run_required(
            ["xwininfo", "-id", window_id, "-tree"],
            command_runner,
        )
        parent_id = parse_xwininfo_parent_id(result.stdout)
        if parent_id in candidate_set and parent_id != window_id:
            frame_ids.add(parent_id)

    return tuple(window_id for window_id in window_ids if window_id not in frame_ids)


def _query_xres_client_pid(client_xid: int) -> int:
    """通过独立 X11 连接查询 XRes 1.2 local-client PID。"""
    x11_name = ctypes.util.find_library("X11")
    xres_name = ctypes.util.find_library("XRes")
    if not x11_name or not xres_name:
        raise OSError("libX11 or libXRes was not found")
    x11 = ctypes.CDLL(x11_name)
    xres = ctypes.CDLL(xres_name)

    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.restype = ctypes.c_int
    xres.XResQueryExtension.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]
    xres.XResQueryExtension.restype = ctypes.c_int
    xres.XResQueryVersion.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]
    xres.XResQueryVersion.restype = ctypes.c_int
    xres.XResQueryClientIds.argtypes = [
        ctypes.c_void_p,
        ctypes.c_long,
        ctypes.POINTER(_XResClientIdSpec),
        ctypes.POINTER(ctypes.c_long),
        ctypes.POINTER(ctypes.POINTER(_XResClientIdValue)),
    ]
    xres.XResQueryClientIds.restype = ctypes.c_int
    xres.XResGetClientIdType.argtypes = [ctypes.POINTER(_XResClientIdValue)]
    xres.XResGetClientIdType.restype = ctypes.c_int
    xres.XResGetClientPid.argtypes = [ctypes.POINTER(_XResClientIdValue)]
    xres.XResGetClientPid.restype = ctypes.c_int
    xres.XResClientIdsDestroy.argtypes = [
        ctypes.c_long,
        ctypes.POINTER(_XResClientIdValue),
    ]
    xres.XResClientIdsDestroy.restype = None

    display_name = os.environ.get("DISPLAY")
    display = x11.XOpenDisplay(
        None if display_name is None else display_name.encode("utf-8")
    )
    if not display:
        raise OSError(f"cannot open X11 display {display_name!r}")

    values = ctypes.POINTER(_XResClientIdValue)()
    count = ctypes.c_long(0)
    try:
        event_base = ctypes.c_int()
        error_base = ctypes.c_int()
        if not xres.XResQueryExtension(
            display,
            ctypes.byref(event_base),
            ctypes.byref(error_base),
        ):
            raise OSError("XRes extension is unavailable on the X11 server")
        major = ctypes.c_int()
        minor = ctypes.c_int()
        if not xres.XResQueryVersion(display, ctypes.byref(major), ctypes.byref(minor)):
            raise OSError("XRes version query failed")
        if (major.value, minor.value) < (1, 2):
            raise OSError(
                f"XRes 1.2 is required, server provides {major.value}.{minor.value}"
            )

        spec = _XResClientIdSpec(client_xid, _XRES_CLIENT_ID_PID_MASK)
        # 该接口返回 X11 Status：0 是 Success，不是常见的 Xlib Bool。
        query_status = xres.XResQueryClientIds(
            display,
            1,
            ctypes.byref(spec),
            ctypes.byref(count),
            ctypes.byref(values),
        )
        if query_status != 0:
            raise OSError(
                f"XRes client PID query failed for XID {client_xid}: "
                f"status={query_status}"
            )
        for index in range(count.value):
            value = ctypes.pointer(values[index])
            if xres.XResGetClientIdType(value) == _XRES_CLIENT_ID_PID_TYPE:
                return int(xres.XResGetClientPid(value))
        raise OSError(f"XRes returned no local client PID for XID {client_xid}")
    finally:
        if values:
            xres.XResClientIdsDestroy(count, values)
        x11.XCloseDisplay(display)


def xres_client_pid(
    window_id: str,
    *,
    query: Callable[[int], int] | None = None,
) -> int:
    """严格查询十进制 client XID 的 XRes PID，不允许标题降级。"""
    if type(window_id) is not str or re.fullmatch(r"[1-9][0-9]*", window_id) is None:
        raise ValueError("window_id must be a positive decimal XID")
    query_pid = _query_xres_client_pid if query is None else query
    try:
        owner_pid = query_pid(int(window_id))
    except Exception as exc:
        raise WindowLayoutError(
            f"XRes client PID query failed for window_id={window_id}: {exc}"
        ) from exc
    if type(owner_pid) is not int or owner_pid <= 0:
        raise WindowLayoutError(
            "XRes returned an invalid client PID: "
            f"window_id={window_id}, pid={owner_pid!r}"
        )
    return owner_pid


def parse_xprop_window_pid(output: str) -> int | None:
    """解析可选 `_NET_WM_PID`；缺失时仍由 XRes 作为唯一所有权依据。"""
    if type(output) is not str:
        raise WindowLayoutError("invalid _NET_WM_PID output: expected text")
    lowered = output.lower()
    if "no such atom" in lowered or "not found" in lowered:
        return None
    match = re.search(
        r"^_NET_WM_PID(?:\([^)]*\))?\s*=\s*([0-9]+)\s*$",
        output,
        flags=re.MULTILINE,
    )
    if match is None:
        raise WindowLayoutError("invalid _NET_WM_PID output")
    owner_pid = int(match.group(1))
    if owner_pid <= 0:
        raise WindowLayoutError("invalid _NET_WM_PID value")
    return owner_pid


def read_x11_window_pid(
    window_id: str,
    *,
    runner: Runner | None = None,
) -> int | None:
    """读取 client 的可选 EWMH PID，供 XRes 一致性检查使用。"""
    command_runner = subprocess.run if runner is None else runner
    result = _run_required(
        ["xprop", "-id", window_id, "_NET_WM_PID"],
        command_runner,
    )
    return parse_xprop_window_pid(result.stdout)


def find_owned_x11_window(
    window_title: str,
    *,
    expected_pid: int,
    timeout_sec: float,
    poll_interval_sec: float = 0.05,
    excluded_window_ids: tuple[str, ...] = (),
    runner: Runner | None = None,
    xres_pid_getter: Callable[[str], int] | None = None,
    wm_pid_getter: Callable[[str], int | None] | None = None,
    clock: Clock = time.monotonic,
    sleeper: Sleeper = time.sleep,
) -> OwnedX11Window:
    """按精确标题、启动后新增集合和 XRes PID 等待唯一客户窗。"""
    if type(window_title) is not str or not window_title:
        raise ValueError("window_title must be non-empty text")
    if type(expected_pid) is not int or expected_pid <= 0:
        raise ValueError("expected_pid must be a positive integer")
    timeout = _positive_seconds(timeout_sec, "timeout_sec")
    poll_interval = _positive_seconds(poll_interval_sec, "poll_interval_sec")
    command_runner = subprocess.run if runner is None else runner
    pid_getter = xres_client_pid if xres_pid_getter is None else xres_pid_getter
    ewmh_pid_getter = (
        (lambda window_id: read_x11_window_pid(window_id, runner=command_runner))
        if wm_pid_getter is None
        else wm_pid_getter
    )
    excluded = frozenset(excluded_window_ids)
    command = [
        "xdotool",
        "search",
        "--all",
        "--onlyvisible",
        "--name",
        rf"^{re.escape(window_title)}$",
    ]
    deadline = clock() + timeout
    last_detail = "no matching window id"

    while True:
        result = _run_command(command, command_runner)
        if result.returncode == 0:
            candidates = tuple(
                window_id
                for window_id in parse_xdotool_window_ids(result.stdout)
                if window_id not in excluded
            )
            if candidates:
                try:
                    clients = _resolve_x11_client_windows(
                        candidates,
                        runner=command_runner,
                    )
                except WindowLayoutError as exc:
                    # WM 重父化期间可能短暂 BadWindow；只重试 client 解析。
                    last_detail = str(exc)
                else:
                    owned: list[OwnedX11Window] = []
                    observations: list[str] = []
                    for window_id in clients:
                        owner_pid = pid_getter(window_id)
                        if type(owner_pid) is not int or owner_pid <= 0:
                            raise WindowLayoutError(
                                "XRes returned an invalid client PID: "
                                f"window_id={window_id}, pid={owner_pid!r}"
                            )
                        observations.append(f"{window_id}:{owner_pid}")
                        if owner_pid == expected_pid:
                            ewmh_pid = ewmh_pid_getter(window_id)
                            if ewmh_pid is not None and ewmh_pid != owner_pid:
                                raise WindowLayoutError(
                                    "process ownership evidence disagrees between XRes and "
                                    f"_NET_WM_PID: window_id={window_id}, "
                                    f"XRes={owner_pid}, _NET_WM_PID={ewmh_pid}"
                                )
                            owned.append(OwnedX11Window(window_id, owner_pid, window_title))
                    if len(owned) == 1:
                        return owned[0]
                    if len(owned) > 1:
                        raise AmbiguousWindowError(
                            "ambiguous owned X11 windows: "
                            f"title={window_title!r}, expected_pid={expected_pid}, "
                            f"window_ids={', '.join(item.window_id for item in owned)}"
                        )
                    last_detail = (
                        "XRes process ownership mismatch: "
                        f"expected_pid={expected_pid}, "
                        f"observed={','.join(observations) or 'none'}"
                    )
            else:
                last_detail = "search returned no new window id"
        elif result.returncode == 1:
            last_detail = result.stderr.strip() or "no matching window"
        else:
            raise _command_failure(command, result)

        if not _sleep_before_retry(
            deadline=deadline,
            poll_interval_sec=poll_interval,
            clock=clock,
            sleeper=sleeper,
        ):
            raise WindowLayoutError(
                "owned X11 window search failed for "
                f"title={window_title!r}: {last_detail}"
            )


def _find_pybullet_window(
    *,
    timeout_sec: float,
    poll_interval_sec: float,
    runner: Runner,
    clock: Clock,
    sleeper: Sleeper,
    excluded_window_ids: frozenset[str],
) -> str:
    """按精确标题等待唯一可见的 PyBullet 原生顶层窗口。"""
    title_pattern = rf"^{re.escape(PYBULLET_WINDOW_TITLE)}$"
    command = [
        "xdotool",
        "search",
        "--all",
        "--onlyvisible",
        "--name",
        title_pattern,
    ]
    deadline = clock() + timeout_sec
    last_detail = "no matching window id"

    while True:
        result = _run_command(command, runner)
        if result.returncode == 0:
            window_ids = tuple(
                window_id
                for window_id in parse_xdotool_window_ids(result.stdout)
                if window_id not in excluded_window_ids
            )
            if not window_ids:
                last_detail = "search returned no valid window id"
            else:
                try:
                    return resolve_x11_client_window(window_ids, runner=runner)
                except AmbiguousWindowError:
                    raise
                except WindowLayoutError as exc:
                    last_detail = str(exc)
        elif result.returncode == 1:
            last_detail = result.stderr.strip() or "no matching window"
        else:
            raise _command_failure(command, result)

        if not _sleep_before_retry(
            deadline=deadline,
            poll_interval_sec=poll_interval_sec,
            clock=clock,
            sleeper=sleeper,
        ):
            raise WindowLayoutError(
                "PyBullet window search failed for "
                f"title={PYBULLET_WINDOW_TITLE!r}: {last_detail}"
            )


def _verify_window_geometry(
    window_id: str,
    expected: Rect,
    *,
    frame_extents: FrameExtents,
    timeout_sec: float,
    poll_interval_sec: float,
    runner: Runner,
    clock: Clock,
    sleeper: Sleeper,
) -> None:
    """轮询客户区实际几何，直到窗口系统确认目标已生效。"""
    command = ["xwininfo", "-id", window_id]
    deadline = clock() + timeout_sec
    while True:
        try:
            result = _run_required(command, runner)
            client = parse_xwininfo_geometry(result.stdout)
            actual = outer_rect_from_client(client, frame_extents)
        except WindowLayoutError as exc:
            raise _with_unapplied_geometry_context(exc, expected) from exc
        if actual == expected:
            return
        if not _sleep_before_retry(
            deadline=deadline,
            poll_interval_sec=poll_interval_sec,
            clock=clock,
            sleeper=sleeper,
        ):
            raise WindowLayoutError(
                "PyBullet main window geometry mismatch: "
                f"expected={expected!r}, actual={actual!r}"
            )


def apply_main_window_rect(
    rect: Rect,
    *,
    timeout_sec: float = 5.0,
    poll_interval_sec: float = 0.05,
    excluded_window_ids: tuple[str, ...] = (),
    expected_pid: int | None = None,
    claim_token: str | None = None,
    runner: Runner | None = None,
    clock: Clock | None = None,
    sleeper: Sleeper | None = None,
) -> OwnedX11Window:
    """以 XRes 所有权认领 Main，再设置并验证完整外框矩形。"""
    if not isinstance(rect, Rect):
        raise TypeError("rect must be a Rect")
    if not isinstance(excluded_window_ids, tuple) or any(
        type(window_id) is not str
        or re.fullmatch(r"[1-9][0-9]*", window_id) is None
        for window_id in excluded_window_ids
    ):
        raise TypeError("excluded_window_ids must be a tuple of positive decimal IDs")
    timeout = _positive_seconds(timeout_sec, "timeout_sec")
    poll_interval = _positive_seconds(poll_interval_sec, "poll_interval_sec")
    process_pid = os.getpid() if expected_pid is None else expected_pid
    if type(process_pid) is not int or process_pid <= 0:
        raise ValueError("expected_pid must be a positive integer")
    token = (
        f"pybullet-main-{process_pid}-{uuid.uuid4().hex}"
        if claim_token is None
        else claim_token
    )
    if type(token) is not str or not token or "\n" in token or "\r" in token:
        raise ValueError("claim_token must be non-empty single-line text")
    command_runner = subprocess.run if runner is None else runner
    monotonic = time.monotonic if clock is None else clock
    sleep = time.sleep if sleeper is None else sleeper

    try:
        owned = find_owned_x11_window(
            PYBULLET_WINDOW_TITLE,
            expected_pid=process_pid,
            timeout_sec=timeout,
            poll_interval_sec=poll_interval,
            excluded_window_ids=excluded_window_ids,
            runner=command_runner,
            clock=monotonic,
            sleeper=sleep,
        )
        window_id = owned.window_id
        # 认领后立即移除公共标题，后续操作只使用本次唯一 client ID/token。
        _run_required(
            ["xdotool", "set_window", "--name", token, window_id],
            command_runner,
        )
        claimed = OwnedX11Window(window_id, owned.owner_pid, token)
        frame_extents = wait_for_x11_frame_extents(
            window_id,
            timeout_sec=timeout,
            poll_interval_sec=poll_interval,
            runner=command_runner,
            clock=monotonic,
            sleeper=sleep,
        )
        client_width = rect.width - frame_extents.left - frame_extents.right
        client_height = rect.height - frame_extents.top - frame_extents.bottom
        if client_width <= 0 or client_height <= 0:
            raise WindowLayoutError(
                "PyBullet target outer geometry is smaller than its window frame: "
                f"expected={rect!r}, frame_extents={frame_extents!r}"
            )

        # 先缩小客户区再移动外框，避免大初始窗触发 WM 最大化约束。
        _run_required(
            [
                "xdotool",
                "windowsize",
                window_id,
                str(client_width),
                str(client_height),
            ],
            command_runner,
        )
        _run_required(
            [
                "xdotool",
                "windowmove",
                window_id,
                str(rect.x),
                str(rect.y),
            ],
            command_runner,
        )
    except WindowLayoutError as exc:
        raise _with_unapplied_geometry_context(exc, rect) from exc

    _verify_window_geometry(
        window_id,
        rect,
        frame_extents=frame_extents,
        timeout_sec=timeout,
        poll_interval_sec=poll_interval,
        runner=command_runner,
        clock=monotonic,
        sleeper=sleep,
    )
    return claimed


__all__ = [
    "AmbiguousWindowError",
    "DisplayMetrics",
    "FrameExtents",
    "OwnedX11Window",
    "PYBULLET_WINDOW_TITLE",
    "PYBULLET_WINDOW_TOKEN_ENV",
    "Rect",
    "WindowLayout",
    "WindowLayoutError",
    "apply_main_window_rect",
    "align_window_layout_to_scale",
    "calculate_window_layout",
    "connect_pybullet_gui",
    "find_owned_x11_window",
    "logical_client_rect_for_outer",
    "parse_xdotool_window_ids",
    "parse_xprop_frame_extents",
    "parse_xprop_window_pid",
    "parse_xwininfo_geometry",
    "parse_xwininfo_parent_id",
    "primary_available_geometry",
    "primary_display_metrics",
    "read_x11_frame_extents",
    "read_x11_outer_geometry",
    "read_x11_window_pid",
    "resolve_x11_client_window",
    "search_x11_window_ids",
    "wait_for_x11_frame_extents",
    "wait_for_x11_outer_geometry",
    "x11_available_geometry",
    "x11_window_manager_available",
    "xres_client_pid",
]
