# 窗口布局单元测试：锁定纯几何、Qt 懒读取、PyBullet GUI 参数和 X11 回读语义。
from __future__ import annotations

import builtins
from dataclasses import FrozenInstanceError
from types import ModuleType, SimpleNamespace
import os
import re
import sys

import pytest

import slope_sim.window_layout as window_layout_module
from slope_sim.window_layout import (
    PYBULLET_WINDOW_TITLE,
    Rect,
    WindowLayout,
    WindowLayoutError,
    apply_main_window_rect,
    calculate_window_layout,
    connect_pybullet_gui,
    parse_xdotool_window_ids,
    parse_xwininfo_geometry,
    primary_available_geometry,
)


def _completed(returncode: int = 0, stdout: str = "", stderr: str = ""):
    return SimpleNamespace(returncode=returncode, stdout=stdout, stderr=stderr)


def _xwininfo(rect: Rect, *, parent_id: int = 1) -> str:
    return (
        "xwininfo: Window id: 0x4d (has no name)\n"
        f"  Parent window id: 0x{parent_id:x} (has no name)\n"
        f"  Absolute upper-left X:  {rect.x}\n"
        f"  Absolute upper-left Y:  {rect.y}\n"
        "  Relative upper-left X:  0\n"
        "  Relative upper-left Y:  0\n"
        f"  Width: {rect.width}\n"
        f"  Height: {rect.height}\n"
    )


class _ScriptedRunner:
    """按顺序返回命令结果，并保留完整 subprocess 参数。"""

    def __init__(
        self,
        responses: list[object],
        *,
        frame_extents_output: str = "_NET_FRAME_EXTENTS:  no such atom on any window.\n",
    ) -> None:
        self._responses = iter(responses)
        self._frame_extents_output = frame_extents_output
        self.calls: list[tuple[list[str], dict[str, object]]] = []

    def __call__(self, command, **kwargs):
        self.calls.append((list(command), dict(kwargs)))
        if command[:3] == ["xdotool", "set_window", "--name"]:
            return _completed()
        if command[0] == "xprop" and command[-1] == "_NET_FRAME_EXTENTS":
            return _completed(stdout=self._frame_extents_output)
        if command == ["xprop", "-root", "_NET_SUPPORTING_WM_CHECK"]:
            return _completed(
                stdout="_NET_SUPPORTING_WM_CHECK: no such atom on any window.\n"
            )
        response = next(self._responses)
        if isinstance(response, BaseException):
            raise response
        return response


class _FakeClock:
    """让窗口等待测试只按条件推进虚拟时间。"""

    def __init__(self) -> None:
        self.now = 0.0
        self.sleeps: list[float] = []

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration_sec: float) -> None:
        assert duration_sec > 0.0
        self.sleeps.append(duration_sec)
        self.now += duration_sec


@pytest.fixture(autouse=True)
def _isolate_xres_and_claim_token(monkeypatch):
    """几何单测注入可信 XRes 结果；真实 libXRes 由 Xvfb 门禁覆盖。"""
    monkeypatch.setattr(
        window_layout_module,
        "_query_xres_client_pid",
        lambda _client_xid: os.getpid(),
    )
    monkeypatch.setattr(
        window_layout_module,
        "read_x11_window_pid",
        lambda _window_id, **_kwargs: None,
    )
    monkeypatch.setattr(
        window_layout_module.uuid,
        "uuid4",
        lambda: SimpleNamespace(hex="unit-test-token"),
    )


def _install_fake_qt(monkeypatch, app) -> None:
    class FakeQGuiApplication:
        @classmethod
        def instance(cls):
            return app

        @classmethod
        def primaryScreen(cls):
            return None if app is None else app.primaryScreen()

    qt_gui = ModuleType("PySide6.QtGui")
    qt_gui.QGuiApplication = FakeQGuiApplication
    package = ModuleType("PySide6")
    package.QtGui = qt_gui
    monkeypatch.setitem(sys.modules, "PySide6", package)
    monkeypatch.setitem(sys.modules, "PySide6.QtGui", qt_gui)


def _search_command() -> list[str]:
    exact_title = rf"^{re.escape(PYBULLET_WINDOW_TITLE)}$"
    return [
        "xdotool",
        "search",
        "--all",
        "--onlyvisible",
        "--name",
        exact_title,
    ]


def _all_search_command() -> list[str]:
    exact_title = rf"^{re.escape(PYBULLET_WINDOW_TITLE)}$"
    return ["xdotool", "search", "--all", "--name", exact_title]


def _claim_command(window_id: str) -> list[str]:
    return [
        "xdotool",
        "set_window",
        "--name",
        f"pybullet-main-{os.getpid()}-unit-test-token",
        window_id,
    ]


def _assert_unapplied_geometry_context(error: WindowLayoutError, expected: Rect) -> None:
    """早期失败尚无可用回读几何，但必须保留目标矩形上下文。"""
    message = str(error)
    assert f"expected={expected!r}" in message
    assert "actual=None" in message


def test_rect_is_frozen_and_exposes_exclusive_edges():
    rect = Rect(-20, 30, 1366, 768)

    assert rect.right == 1346
    assert rect.bottom == 798
    with pytest.raises(FrozenInstanceError):
        rect.width = 100


@pytest.mark.parametrize("field", ("x", "y", "width", "height"))
@pytest.mark.parametrize("value", (True, False, 1.0, "1"))
def test_rect_requires_strict_non_boolean_integers(field, value):
    values = {"x": 0, "y": 0, "width": 100, "height": 80}
    values[field] = value

    with pytest.raises(TypeError, match=field):
        Rect(**values)


@pytest.mark.parametrize(
    "width,height",
    ((0, 10), (-1, 10), (10, 0), (10, -1)),
)
def test_rect_requires_positive_dimensions(width, height):
    with pytest.raises(ValueError, match="positive"):
        Rect(0, 0, width, height)


def test_window_layout_is_frozen():
    layout = WindowLayout(main=Rect(0, 0, 100, 80), dashboard=None)

    with pytest.raises(FrozenInstanceError):
        layout.dashboard = Rect(80, 0, 20, 80)


@pytest.mark.parametrize(
    "width,height,main_width,dashboard_width",
    (
        (1366, 768, 915, 451),
        (1920, 1080, 1286, 634),
        (2560, 1440, 1715, 845),
    ),
)
def test_layout_fills_available_geometry_at_exact_sixty_seven_thirty_three_split(
    width,
    height,
    main_width,
    dashboard_width,
):
    layout = calculate_window_layout(Rect(0, 0, width, height), dashboard_enabled=True)

    assert layout.main == Rect(0, 0, main_width, height)
    assert layout.dashboard == Rect(
        width - dashboard_width,
        0,
        dashboard_width,
        height,
    )
    assert layout.main.right == layout.dashboard.x
    assert layout.dashboard.right == width
    assert layout.main.bottom == layout.dashboard.bottom == height


def test_layout_preserves_nonzero_origin_and_nondivisible_width_without_a_seam():
    available = Rect(37, 53, 1367, 701)

    layout = calculate_window_layout(available, dashboard_enabled=True)

    assert layout.main == Rect(37, 53, 916, 701)
    assert layout.dashboard == Rect(953, 53, 451, 701)
    assert layout.main.right == layout.dashboard.x
    assert layout.dashboard.right == available.right
    assert layout.dashboard.bottom == available.bottom


def test_dashboard_disabled_gives_main_entire_available_geometry():
    available = Rect(20, 30, 1801, 1000)

    layout = calculate_window_layout(available, dashboard_enabled=False)

    assert layout.main is available
    assert layout.dashboard is None


def test_dashboard_split_aligns_to_two_physical_pixels_without_changing_coverage():
    available = Rect(112, 64, 2448, 1376)
    layout = calculate_window_layout(available, dashboard_enabled=True)

    aligned = window_layout_module.align_window_layout_to_scale(layout, 2.0)

    assert aligned.main == Rect(112, 64, 1640, 1376)
    assert aligned.dashboard == Rect(1752, 64, 808, 1376)
    assert aligned.main.right == aligned.dashboard.x
    assert aligned.dashboard.right == available.right


@pytest.mark.parametrize("available_width", (50, 1250))
def test_dashboard_scale_alignment_uses_half_up_at_exact_half_pixel(available_width):
    """DPR 对齐不得用 ties-to-even 改写 33% 的 half-up 契约。"""
    layout = calculate_window_layout(
        Rect(0, 0, available_width, 700),
        dashboard_enabled=True,
    )

    aligned = window_layout_module.align_window_layout_to_scale(layout, 1.0)

    assert aligned == layout


def test_dashboard_scale_alignment_uses_exact_ratio_at_decimal_dpr_half_boundary():
    """十进制 DPR 的精确 .5 边界不得被二进制浮点向下扰动。"""
    layout = calculate_window_layout(
        Rect(0, 0, 1365, 700),
        dashboard_enabled=True,
    )

    aligned = window_layout_module.align_window_layout_to_scale(layout, 1.1)

    assert aligned.main == Rect(0, 0, 914, 700)
    assert aligned.dashboard == Rect(914, 0, 451, 700)


def test_x11_available_geometry_uses_current_desktop_workarea_and_primary_screen():
    metrics = window_layout_module.DisplayMetrics(
        screen=Rect(0, 0, 1280, 720),
        available=Rect(112, 64, 1224, 688),
        device_pixel_ratio=2.0,
    )
    runner = _ScriptedRunner(
        [
            _completed(
                stdout=(
                    "_NET_CURRENT_DESKTOP(CARDINAL) = 1\n"
                    "_NET_WORKAREA(CARDINAL) = 0, 0, 2560, 1440, "
                    "112, 64, 2448, 1376\n"
                )
            ),
            _completed(
                stdout="_GTK_WORKAREAS_D1(CARDINAL) = 112, 64, 2448, 1376\n"
            ),
        ]
    )

    actual = window_layout_module.x11_available_geometry(metrics, runner=runner)

    assert actual == Rect(112, 64, 2448, 1376)
    assert runner.calls[0][0] == [
        "xprop",
        "-root",
        "_NET_CURRENT_DESKTOP",
        "_NET_WORKAREA",
    ]


def test_x11_available_geometry_scales_qt_size_when_window_manager_is_absent():
    metrics = window_layout_module.DisplayMetrics(
        screen=Rect(0, 0, 1280, 720),
        available=Rect(0, 0, 1280, 720),
        device_pixel_ratio=2.0,
    )
    runner = _ScriptedRunner(
        [_completed(stdout="_NET_WORKAREA:  no such atom on any window.\n")]
    )

    assert window_layout_module.x11_available_geometry(metrics, runner=runner) == Rect(
        0,
        0,
        2560,
        1440,
    )


def test_x11_available_geometry_prefers_per_monitor_gtk_workarea():
    metrics = window_layout_module.DisplayMetrics(
        screen=Rect(1920, 0, 1920, 1080),
        available=Rect(1920, 0, 1920, 1080),
        device_pixel_ratio=1.0,
    )
    runner = _ScriptedRunner(
        [
            _completed(
                stdout=(
                    "_NET_CURRENT_DESKTOP(CARDINAL) = 0\n"
                    "_NET_WORKAREA(CARDINAL) = 0, 40, 3840, 1040\n"
                )
            ),
            _completed(
                stdout=(
                    "_GTK_WORKAREAS_D0(CARDINAL) = "
                    "0, 40, 1920, 1040, 1920, 0, 1920, 1080\n"
                )
            ),
        ]
    )

    assert window_layout_module.x11_available_geometry(metrics, runner=runner) == Rect(
        1920,
        0,
        1920,
        1080,
    )
    assert runner.calls[1][0] == ["xprop", "-root", "_GTK_WORKAREAS_D0"]


def test_logical_client_rect_converts_physical_outer_rect_and_qt_frame_margins():
    actual = window_layout_module.logical_client_rect_for_outer(
        Rect(2070, 64, 490, 1376),
        screen=Rect(0, 0, 1280, 720),
        device_pixel_ratio=2.0,
        frame_extents=window_layout_module.FrameExtents(0, 0, 37, 0),
    )

    assert actual == Rect(1035, 32, 245, 651)


def test_logical_client_rect_uses_shared_edges_for_fractional_dpr_and_negative_origin():
    actual = window_layout_module.logical_client_rect_for_outer(
        Rect(-100, 11, 491, 777),
        screen=Rect(-1920, -100, 1280, 720),
        device_pixel_ratio=1.5,
        frame_extents=window_layout_module.FrameExtents(1, 2, 3, 4),
    )

    assert actual == Rect(-707, -26, 325, 511)


def test_direct_geometry_and_injected_pybullet_paths_never_import_qt(monkeypatch):
    real_import = builtins.__import__

    def reject_qt_import(name, *args, **kwargs):
        if name == "PySide6" or name.startswith("PySide6."):
            raise AssertionError("DIRECT-compatible path imported Qt")
        return real_import(name, *args, **kwargs)

    fake_pybullet = SimpleNamespace(GUI=7, connect=lambda mode, **kwargs: 3)
    monkeypatch.setattr(builtins, "__import__", reject_qt_import)

    layout = calculate_window_layout(Rect(0, 0, 1000, 600), True)
    assert connect_pybullet_gui(layout.main, pybullet_module=fake_pybullet) == 3


def test_primary_available_geometry_lazily_reads_qt_available_area(monkeypatch):
    geometry = SimpleNamespace(
        x=lambda: -1920,
        y=lambda: 24,
        width=lambda: 1920,
        height=lambda: 1056,
    )
    calls: list[str] = []
    screen = SimpleNamespace(
        availableGeometry=lambda: calls.append("availableGeometry") or geometry
    )
    app = SimpleNamespace(primaryScreen=lambda: calls.append("primaryScreen") or screen)
    _install_fake_qt(monkeypatch, app)

    assert primary_available_geometry() == Rect(-1920, 24, 1920, 1056)
    assert calls == ["primaryScreen", "availableGeometry"]


@pytest.mark.parametrize("case", ("no_app", "no_screen"))
def test_primary_available_geometry_rejects_missing_application_or_screen(monkeypatch, case):
    app = None if case == "no_app" else SimpleNamespace(primaryScreen=lambda: None)
    _install_fake_qt(monkeypatch, app)

    with pytest.raises(WindowLayoutError, match="application|screen"):
        primary_available_geometry()


@pytest.mark.parametrize(
    "geometry",
    (
        SimpleNamespace(x=lambda: 0, y=lambda: 0, width=lambda: 0, height=lambda: 768),
        SimpleNamespace(x=lambda: 0.5, y=lambda: 0, width=lambda: 1366, height=lambda: 768),
    ),
)
def test_primary_available_geometry_wraps_invalid_qt_geometry(monkeypatch, geometry):
    screen = SimpleNamespace(availableGeometry=lambda: geometry)
    app = SimpleNamespace(primaryScreen=lambda: screen)
    _install_fake_qt(monkeypatch, app)

    with pytest.raises(WindowLayoutError, match="invalid.*geometry"):
        primary_available_geometry()


def test_primary_available_geometry_reports_missing_pyside6(monkeypatch):
    real_import = builtins.__import__
    monkeypatch.delitem(sys.modules, "PySide6", raising=False)
    monkeypatch.delitem(sys.modules, "PySide6.QtGui", raising=False)

    def fail_pyside_import(name, *args, **kwargs):
        if name == "PySide6" or name.startswith("PySide6."):
            raise ImportError("not installed")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fail_pyside_import)

    with pytest.raises(WindowLayoutError, match="PySide6"):
        primary_available_geometry()


def test_connect_pybullet_gui_passes_deterministic_size_options_without_qt():
    calls: list[tuple[int, dict[str, object]]] = []

    def connect(mode, **kwargs):
        calls.append((mode, kwargs))
        return 8

    fake_pybullet = SimpleNamespace(GUI=11, connect=connect)

    client_id = connect_pybullet_gui(
        Rect(20, 30, 1093, 768),
        pybullet_module=fake_pybullet,
    )

    assert client_id == 8
    assert calls == [(11, {"options": "--width=1093 --height=768"})]


@pytest.mark.parametrize("client_id", (-1, -20, None, True))
def test_connect_pybullet_gui_rejects_failed_or_invalid_connection_ids(client_id):
    fake_pybullet = SimpleNamespace(GUI=1, connect=lambda *args, **kwargs: client_id)

    with pytest.raises(WindowLayoutError, match="connect.*PyBullet GUI"):
        connect_pybullet_gui(Rect(0, 0, 800, 600), pybullet_module=fake_pybullet)


def test_connect_pybullet_gui_wraps_pybullet_connection_errors():
    def fail_connect(*args, **kwargs):
        raise RuntimeError("display unavailable")

    fake_pybullet = SimpleNamespace(GUI=1, connect=fail_connect)

    with pytest.raises(WindowLayoutError, match="display unavailable"):
        connect_pybullet_gui(Rect(0, 0, 800, 600), pybullet_module=fake_pybullet)


def test_parse_xdotool_window_ids_ignores_non_id_lines_and_preserves_order():
    output = "debug: searching\n101\n\nnot-an-id\n205\n101\n0\n"

    assert parse_xdotool_window_ids(output) == ("101", "205")


def test_owned_window_lookup_resolves_frame_then_filters_by_xres_pid():
    """同标题窗口必须先归并 WM frame，再由 XRes client PID 选定所有者。"""
    calls: list[list[str]] = []
    xres_calls: list[str] = []

    def runner(command, **_kwargs):
        calls.append(list(command))
        if command[0] == "xdotool":
            return _completed(stdout="41\n77\n90\n")
        parent_id = {"41": 1, "77": 41, "90": 1}[command[2]]
        return _completed(stdout=_xwininfo(Rect(0, 0, 800, 600), parent_id=parent_id))

    def xres_pid_getter(window_id: str) -> int:
        xres_calls.append(window_id)
        return {"77": 9001, "90": 4321}[window_id]

    owned = window_layout_module.find_owned_x11_window(
        PYBULLET_WINDOW_TITLE,
        expected_pid=4321,
        timeout_sec=1.0,
        runner=runner,
        xres_pid_getter=xres_pid_getter,
        wm_pid_getter=lambda _window_id: None,
    )

    assert owned.window_id == "90"
    assert owned.owner_pid == 4321
    assert xres_calls == ["77", "90"]
    assert calls[:4] == [
        _search_command(),
        ["xwininfo", "-id", "41", "-tree"],
        ["xwininfo", "-id", "77", "-tree"],
        ["xwininfo", "-id", "90", "-tree"],
    ]


def test_xres_client_pid_uses_decimal_client_xid_and_rejects_query_failure():
    """XRes 包装必须查询 client XID，并把扩展不可用变成明确领域错误。"""
    calls: list[int] = []

    assert window_layout_module.xres_client_pid(
        "77",
        query=lambda xid: calls.append(xid) or 4321,
    ) == 4321
    assert calls == [77]

    def unavailable(_xid: int) -> int:
        raise OSError("libXRes.so.1 missing")

    with pytest.raises(WindowLayoutError, match="XRes.*missing"):
        window_layout_module.xres_client_pid("77", query=unavailable)


def test_owned_window_lookup_rejects_xres_and_net_wm_pid_disagreement():
    """可选的 _NET_WM_PID 一旦存在，就不能与 XRes 所有权证据冲突。"""

    def runner(command, **_kwargs):
        assert command == _search_command()
        return _completed(stdout="77\n")

    with pytest.raises(WindowLayoutError, match="ownership.*XRes.*_NET_WM_PID"):
        window_layout_module.find_owned_x11_window(
            PYBULLET_WINDOW_TITLE,
            expected_pid=4321,
            timeout_sec=1.0,
            runner=runner,
            xres_pid_getter=lambda _window_id: 4321,
            wm_pid_getter=lambda _window_id: 9001,
        )


def test_apply_main_window_rect_claims_owned_client_before_geometry_commands(monkeypatch):
    """Main 必须先完成 PID 认领并改成唯一 token，之后才允许移动或缩放。"""
    target = Rect(20, 30, 800, 600)
    owned = window_layout_module.OwnedX11Window("77", 4321, PYBULLET_WINDOW_TITLE)
    find_calls: list[dict[str, object]] = []
    commands: list[list[str]] = []

    def find_owned(_title, **kwargs):
        find_calls.append(kwargs)
        return owned

    def runner(command, **_kwargs):
        commands.append(list(command))
        if command == ["xprop", "-id", "77", "_NET_FRAME_EXTENTS"]:
            return _completed(stdout="_NET_FRAME_EXTENTS: not found.\n")
        if command == ["xprop", "-root", "_NET_SUPPORTING_WM_CHECK"]:
            return _completed(stdout="_NET_SUPPORTING_WM_CHECK: not found.\n")
        if command == ["xwininfo", "-id", "77"]:
            return _completed(stdout=_xwininfo(target))
        return _completed()

    monkeypatch.setattr(window_layout_module, "find_owned_x11_window", find_owned)

    claimed = apply_main_window_rect(
        target,
        expected_pid=4321,
        claim_token="pybullet-main-test-token",
        excluded_window_ids=("11", "12"),
        runner=runner,
    )

    assert claimed == window_layout_module.OwnedX11Window(
        "77",
        4321,
        "pybullet-main-test-token",
    )
    assert find_calls == [
        {
            "expected_pid": 4321,
            "timeout_sec": 5.0,
            "poll_interval_sec": 0.05,
            "excluded_window_ids": ("11", "12"),
            "runner": runner,
            "clock": window_layout_module.time.monotonic,
            "sleeper": window_layout_module.time.sleep,
        }
    ]
    claim_index = commands.index(
        ["xdotool", "set_window", "--name", "pybullet-main-test-token", "77"]
    )
    assert claim_index < commands.index(["xdotool", "windowsize", "77", "800", "600"])
    assert claim_index < commands.index(["xdotool", "windowmove", "77", "20", "30"])


def test_search_x11_window_ids_can_snapshot_hidden_and_visible_windows():
    runner = _ScriptedRunner([_completed(stdout="41\n77\n")])

    actual = window_layout_module.search_x11_window_ids(
        PYBULLET_WINDOW_TITLE,
        only_visible=False,
        runner=runner,
    )

    assert actual == ("41", "77")
    assert runner.calls[0][0] == _all_search_command()


def test_parse_xwininfo_geometry_reads_absolute_client_area():
    output = (
        "xwininfo: Window id: 0x65 \"pybullet\"\n"
        "  Absolute upper-left X:  -1366\n"
        "  Absolute upper-left Y:  24\n"
        "  Relative upper-left X:  5\n"
        "  Relative upper-left Y:  31\n"
        "  Width: 1093\n"
        "  Height: 744\n"
        "  Depth: 24\n"
    )

    assert parse_xwininfo_geometry(output) == Rect(-1366, 24, 1093, 744)


@pytest.mark.parametrize(
    "output",
    (
        "Absolute upper-left X: 0\nAbsolute upper-left Y: 0\nWidth: 100\n",
        "Absolute upper-left X: zero\nAbsolute upper-left Y: 0\nWidth: 100\nHeight: 80\n",
        "Absolute upper-left X: 0\nAbsolute upper-left Y: 0\nWidth: 0\nHeight: 80\n",
    ),
)
def test_parse_xwininfo_geometry_rejects_missing_noninteger_or_invalid_fields(output):
    with pytest.raises(WindowLayoutError, match="xwininfo"):
        parse_xwininfo_geometry(output)


def test_installed_pybullet_title_drives_exact_search_before_xres_ownership():
    assert (
        PYBULLET_WINDOW_TITLE
        == "Bullet Physics ExampleBrowser using OpenGL3+ [btgl] Release build"
    )
    target = Rect(0, 0, 800, 600)
    runner = _ScriptedRunner(
        [
            _completed(stdout="77\n"),
            _completed(),
            _completed(),
            _completed(stdout=_xwininfo(target)),
        ]
    )
    apply_main_window_rect(target, runner=runner)

    search_command = runner.calls[0][0]
    assert search_command == _search_command()
    assert "--pid" not in search_command
    assert runner.calls[1][0] == _claim_command("77")


def test_apply_main_window_rect_locks_title_filtered_x11_command_sequence():
    target = Rect(20, 30, 1093, 768)
    runner = _ScriptedRunner(
        [
            _completed(stdout="debug line\n77\n"),
            _completed(stdout=_xwininfo(target)),
            _completed(stdout=_xwininfo(target)),
            _completed(stdout=_xwininfo(target)),
        ]
    )
    clock = _FakeClock()
    apply_main_window_rect(
        target,
        runner=runner,
        clock=clock.monotonic,
        sleeper=clock.sleep,
    )

    assert [command for command, _kwargs in runner.calls] == [
        _search_command(),
        _claim_command("77"),
        ["xprop", "-id", "77", "_NET_FRAME_EXTENTS"],
        ["xprop", "-root", "_NET_SUPPORTING_WM_CHECK"],
        ["xdotool", "windowsize", "77", "1093", "768"],
        ["xdotool", "windowmove", "77", "20", "30"],
        ["xwininfo", "-id", "77"],
    ]
    assert all(
        kwargs == {"check": False, "text": True, "capture_output": True}
        for _command, kwargs in runner.calls
    )
    assert clock.sleeps == []


def test_apply_main_window_rect_retries_search_until_matching_window_exists():
    target = Rect(0, 0, 800, 600)
    runner = _ScriptedRunner(
        [
            _completed(returncode=1, stderr="no window"),
            _completed(stdout="diagnostic only\n"),
            _completed(stdout="55\n"),
            _completed(),
            _completed(),
            _completed(stdout=_xwininfo(target)),
        ]
    )
    clock = _FakeClock()
    apply_main_window_rect(
        target,
        timeout_sec=1.0,
        poll_interval_sec=0.1,
        runner=runner,
        clock=clock.monotonic,
        sleeper=clock.sleep,
    )

    assert [command for command, _kwargs in runner.calls[:3]] == [
        _search_command(),
        _search_command(),
        _search_command(),
    ]
    assert clock.sleeps == pytest.approx([0.1, 0.1])


def test_apply_main_window_rect_selects_reparented_client_from_frame_pair():
    expected = Rect(0, 0, 800, 600)
    client = Rect(0, 74, 800, 526)
    runner = _ScriptedRunner(
        [
            _completed(stdout="41\n77\n"),
            _completed(stdout=_xwininfo(expected, parent_id=1)),
            _completed(stdout=_xwininfo(expected, parent_id=41)),
            _completed(),
            _completed(),
            _completed(stdout=_xwininfo(client, parent_id=41)),
        ],
        frame_extents_output="_NET_FRAME_EXTENTS(CARDINAL) = 0, 0, 74, 0\n",
    )

    apply_main_window_rect(expected, runner=runner)

    assert [command for command, _kwargs in runner.calls] == [
        _search_command(),
        ["xwininfo", "-id", "41", "-tree"],
        ["xwininfo", "-id", "77", "-tree"],
        _claim_command("77"),
        ["xprop", "-id", "77", "_NET_FRAME_EXTENTS"],
        ["xprop", "-id", "77", "_NET_FRAME_EXTENTS"],
        ["xdotool", "windowsize", "77", "800", "526"],
        ["xdotool", "windowmove", "77", "0", "0"],
        ["xwininfo", "-id", "77"],
    ]


def test_apply_main_window_rect_ignores_preexisting_window_family():
    expected = Rect(0, 0, 800, 600)
    runner = _ScriptedRunner(
        [
            _completed(stdout="11\n12\n41\n77\n"),
            _completed(stdout=_xwininfo(expected, parent_id=1)),
            _completed(stdout=_xwininfo(expected, parent_id=41)),
            _completed(),
            _completed(),
            _completed(stdout=_xwininfo(expected, parent_id=41)),
        ]
    )

    apply_main_window_rect(
        expected,
        excluded_window_ids=("11", "12"),
        runner=runner,
    )

    assert [command for command, _kwargs in runner.calls[:5]] == [
        _search_command(),
        ["xwininfo", "-id", "41", "-tree"],
        ["xwininfo", "-id", "77", "-tree"],
        _claim_command("77"),
        ["xprop", "-id", "77", "_NET_FRAME_EXTENTS"],
    ]


def test_apply_main_window_rect_rejects_independent_matching_pybullet_windows():
    expected = Rect(0, 0, 800, 600)
    runner = _ScriptedRunner(
        [
            _completed(stdout="41\n77\n"),
            _completed(stdout=_xwininfo(expected, parent_id=1)),
            _completed(stdout=_xwininfo(expected, parent_id=1)),
        ]
    )

    with pytest.raises(WindowLayoutError, match=r"ambiguous.*41.*77") as captured:
        apply_main_window_rect(expected, runner=runner)

    _assert_unapplied_geometry_context(captured.value, expected)
    assert [command for command, _kwargs in runner.calls] == [
        _search_command(),
        ["xwininfo", "-id", "41", "-tree"],
        ["xwininfo", "-id", "77", "-tree"],
    ]


def test_apply_main_window_rect_retries_transient_reparenting_lookup_failure():
    expected = Rect(0, 0, 800, 600)
    runner = _ScriptedRunner(
        [
            _completed(stdout="41\n77\n"),
            _completed(returncode=1, stderr="BadWindow during reparent"),
            _completed(stdout="41\n77\n"),
            _completed(stdout=_xwininfo(expected, parent_id=1)),
            _completed(stdout=_xwininfo(expected, parent_id=41)),
            _completed(),
            _completed(),
            _completed(stdout=_xwininfo(expected, parent_id=41)),
        ]
    )
    clock = _FakeClock()

    apply_main_window_rect(
        expected,
        timeout_sec=0.5,
        poll_interval_sec=0.05,
        runner=runner,
        clock=clock.monotonic,
        sleeper=clock.sleep,
    )

    assert [command for command, _kwargs in runner.calls[:3]] == [
        _search_command(),
        ["xwininfo", "-id", "41", "-tree"],
        _search_command(),
    ]
    assert clock.sleeps == pytest.approx([0.05])


def test_apply_main_window_rect_waits_for_verified_geometry_not_a_fixed_delay():
    target = Rect(10, 20, 1000, 700)
    initial = Rect(12, 44, 900, 650)
    runner = _ScriptedRunner(
        [
            _completed(stdout="88\n"),
            _completed(),
            _completed(),
            _completed(stdout=_xwininfo(initial)),
            _completed(stdout=_xwininfo(target)),
        ]
    )
    clock = _FakeClock()
    apply_main_window_rect(
        target,
        timeout_sec=1.0,
        poll_interval_sec=0.05,
        runner=runner,
        clock=clock.monotonic,
        sleeper=clock.sleep,
    )

    assert [command for command, _kwargs in runner.calls[-2:]] == [
        ["xwininfo", "-id", "88"],
        ["xwininfo", "-id", "88"],
    ]
    assert clock.sleeps == pytest.approx([0.05])


def test_apply_main_window_rect_waits_for_delayed_frame_extents_with_window_manager():
    expected = Rect(0, 0, 800, 600)
    client = Rect(0, 74, 800, 526)
    clock = _FakeClock()
    extents_reads = 0

    def runner(command, **_kwargs):
        nonlocal extents_reads
        if command == _search_command():
            return _completed(stdout="77\n")
        if command == ["xprop", "-id", "77", "_NET_FRAME_EXTENTS"]:
            extents_reads += 1
            if extents_reads == 1:
                return _completed(stdout="_NET_FRAME_EXTENTS: not found.\n")
            return _completed(stdout="_NET_FRAME_EXTENTS(CARDINAL) = 0, 0, 74, 0\n")
        if command == ["xprop", "-root", "_NET_SUPPORTING_WM_CHECK"]:
            return _completed(
                stdout="_NET_SUPPORTING_WM_CHECK(WINDOW): window id # 0x400001\n"
            )
        if command[0] == "xwininfo":
            return _completed(stdout=_xwininfo(client))
        return _completed()

    apply_main_window_rect(
        expected,
        timeout_sec=1.0,
        poll_interval_sec=0.05,
        runner=runner,
        clock=clock.monotonic,
        sleeper=clock.sleep,
    )

    assert extents_reads == 3
    assert clock.sleeps == pytest.approx([0.05, 0.05])


def test_wait_for_x11_outer_geometry_rereads_transient_frame_extents():
    expected = Rect(0, 0, 800, 600)
    client = Rect(0, 74, 800, 526)
    clock = _FakeClock()
    extents_reads = 0

    def runner(command, **_kwargs):
        nonlocal extents_reads
        if command[0] == "xwininfo":
            return _completed(stdout=_xwininfo(client))
        extents_reads += 1
        if extents_reads == 1:
            return _completed(stdout="_NET_FRAME_EXTENTS: not found.\n")
        return _completed(stdout="_NET_FRAME_EXTENTS(CARDINAL) = 0, 0, 74, 0\n")

    actual = window_layout_module.wait_for_x11_outer_geometry(
        "77",
        expected,
        timeout_sec=0.5,
        poll_interval_sec=0.05,
        runner=runner,
        clock=clock.monotonic,
        sleeper=clock.sleep,
    )

    assert actual == expected
    assert extents_reads == 2
    assert clock.sleeps == pytest.approx([0.05])


def test_apply_main_window_rect_reports_missing_window_tool():
    expected = Rect(0, 0, 800, 600)
    runner = _ScriptedRunner([FileNotFoundError("xdotool")])

    with pytest.raises(WindowLayoutError, match="xdotool.*not found") as captured:
        apply_main_window_rect(expected, runner=runner)

    _assert_unapplied_geometry_context(captured.value, expected)


def test_apply_main_window_rect_reports_search_timeout():
    expected = Rect(0, 0, 800, 600)
    calls: list[list[str]] = []
    clock = _FakeClock()

    def runner(command, **kwargs):
        calls.append(list(command))
        return _completed(returncode=1, stderr="no matching window")

    with pytest.raises(
        WindowLayoutError,
        match="search.*Bullet Physics ExampleBrowser",
    ) as captured:
        apply_main_window_rect(
            expected,
            timeout_sec=0.2,
            poll_interval_sec=0.1,
            runner=runner,
            clock=clock.monotonic,
            sleeper=clock.sleep,
        )

    _assert_unapplied_geometry_context(captured.value, expected)
    assert len(calls) == 3
    assert all(command == _search_command() for command in calls)
    assert clock.sleeps == pytest.approx([0.1, 0.1])


@pytest.mark.parametrize(
    "command_name,diagnostic",
    (("windowmove", "move denied"), ("windowsize", "size denied")),
)
def test_apply_main_window_rect_reports_non_search_command_failure(command_name, diagnostic):
    expected = Rect(0, 0, 800, 600)
    responses = [_completed(stdout="77\n")]
    if command_name == "windowmove":
        responses.append(_completed())
    responses.append(_completed(returncode=2, stderr=diagnostic))
    runner = _ScriptedRunner(responses)
    with pytest.raises(
        WindowLayoutError,
        match=rf"{command_name}.*{diagnostic}",
    ) as captured:
        apply_main_window_rect(expected, runner=runner)

    _assert_unapplied_geometry_context(captured.value, expected)
    assert runner.calls[-1][0][1] == command_name


def test_apply_main_window_rect_reports_expected_and_actual_after_geometry_timeout():
    expected = Rect(20, 30, 800, 600)
    actual = Rect(21, 54, 799, 576)
    clock = _FakeClock()

    def runner(command, **kwargs):
        if list(command) == _search_command():
            return _completed(stdout="90\n")
        if command[0] == "xprop":
            return _completed(stdout="_NET_FRAME_EXTENTS: not found.\n")
        if command[0] == "xwininfo":
            return _completed(stdout=_xwininfo(actual))
        return _completed()

    with pytest.raises(WindowLayoutError) as captured:
        apply_main_window_rect(
            expected,
            timeout_sec=0.1,
            poll_interval_sec=0.05,
            runner=runner,
            clock=clock.monotonic,
            sleeper=clock.sleep,
        )

    message = str(captured.value)
    assert f"expected={expected!r}" in message
    assert f"actual={actual!r}" in message
    assert clock.sleeps == pytest.approx([0.05, 0.05])


def test_apply_main_window_rect_rejects_malformed_xwininfo_output():
    expected = Rect(0, 0, 800, 600)
    runner = _ScriptedRunner(
        [
            _completed(stdout="91\n"),
            _completed(),
            _completed(),
            _completed(stdout="Width: 800\nHeight: 600\n"),
        ]
    )
    with pytest.raises(WindowLayoutError, match="xwininfo") as captured:
        apply_main_window_rect(expected, runner=runner)

    _assert_unapplied_geometry_context(captured.value, expected)
