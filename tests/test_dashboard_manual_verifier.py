# Dashboard 屏幕验收脚本测试：保护日志判定和按钮点击坐标，避免手工验收口径漂移。
import argparse
from pathlib import Path
import os
import subprocess
import sys
from types import ModuleType, SimpleNamespace

import pandas as pd
from PIL import Image, ImageDraw
import pytest
import scripts.verify_dashboard_manual_drive as verifier_module
from slope_sim.window_layout import Rect, calculate_window_layout

from scripts.verify_dashboard_manual_drive import (
    ManualMotionSummary,
    WindowGeometry,
    dashboard_up_button_point,
    dashboard_window_ids,
    display_is_available,
    motion_passed,
    newest_manual_log,
    summarize_manual_motion,
)


def test_direct_script_help_bootstraps_repo_root_without_pythonpath():
    """直接运行脚本时应自行发现仓库包，不依赖外部 PYTHONPATH。"""
    root = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env.pop("PYTHONPATH", None)

    completed = subprocess.run(
        [
            sys.executable,
            str(root / "scripts" / "verify_dashboard_manual_drive.py"),
            "--help",
        ],
        cwd=root,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert "--verify-window-layout" in completed.stdout
    assert "--verify-dashboard-tabs" in completed.stdout
    assert "--expected-available-size" in completed.stdout


def test_display_is_available_requires_x11_or_xwayland_display():
    assert display_is_available({"DISPLAY": ":0"}) is True
    assert display_is_available({"WAYLAND_DISPLAY": "wayland-0"}) is False
    assert display_is_available({}) is False


def test_dashboard_up_button_point_uses_sidebar_center_and_bottom_offset():
    normal_point = dashboard_up_button_point(WindowGeometry(x=10, y=20, width=420, height=760))
    shorter_point = dashboard_up_button_point(WindowGeometry(x=10, y=20, width=420, height=600))

    assert normal_point == (220, 624)
    assert shorter_point == (220, 464)


def test_dashboard_geometry_scale_uses_fixed_logical_width():
    geometry = WindowGeometry(x=240, y=236, width=840, height=1306)

    assert verifier_module.dashboard_geometry_scale(geometry) == pytest.approx(2.0)


def test_parse_xwininfo_geometry_uses_absolute_client_origin():
    output = """
xwininfo: Window id: 0x4a00007 "Stage 1 Robot Evaluation Dashboard"

  Absolute upper-left X:  212
  Absolute upper-left Y:  138
  Relative upper-left X:  28
  Relative upper-left Y:  98
  Width: 840
  Height: 1306
"""

    assert verifier_module.parse_xwininfo_geometry(output) == WindowGeometry(212, 138, 840, 1306)


def test_get_window_geometry_calls_xwininfo(monkeypatch):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return type(
            "Result",
            (),
            {
                "stdout": (
                    "Absolute upper-left X:  212\n"
                    "Absolute upper-left Y:  138\n"
                    "Width: 840\n"
                    "Height: 1306\n"
                )
            },
        )()

    monkeypatch.setattr(verifier_module.subprocess, "run", fake_run)

    geometry = verifier_module._get_window_geometry("dashboard-id")

    assert geometry == WindowGeometry(212, 138, 840, 1306)
    assert calls == [
        (
            ["xwininfo", "-id", "dashboard-id"],
            {"check": True, "text": True, "capture_output": True},
        )
    ]


def test_dashboard_hidpi_points_scale_all_logical_offsets():
    geometry = WindowGeometry(x=240, y=236, width=840, height=1306)

    assert dashboard_up_button_point(geometry) == (660, 1230)
    assert verifier_module.dashboard_control_scroll_point(geometry) == (660, 1382)
    assert verifier_module.dashboard_plot_tab_point(geometry, "trajectory") == (550, 360)
    assert verifier_module.dashboard_plot_tab_point(geometry, "speed") == (676, 360)
    assert verifier_module.dashboard_plot_tab_point(geometry, "slip") == (800, 360)
    assert verifier_module.dashboard_plot_tab_point(geometry, "contact") == (896, 360)


def test_click_dashboard_up_scrolls_controls_before_holding_button(monkeypatch):
    """按钮路径必须先把控制区滚到底，再点击滚动后的上箭头。"""
    events = []

    def fake_geometry(window_id):
        events.append(("geometry", window_id))
        return WindowGeometry(x=10, y=20, width=420, height=760)

    def fake_xdotool(args):
        events.append(("xdotool", tuple(args)))
        return type(
            "Result",
            (),
            {"stdout": "X=10\nY=20\nWIDTH=420\nHEIGHT=760\n"},
        )()

    monkeypatch.setattr(verifier_module, "_get_window_geometry", fake_geometry, raising=False)
    monkeypatch.setattr(verifier_module, "_run_xdotool", fake_xdotool)
    monkeypatch.setattr(verifier_module.time, "sleep", lambda duration: events.append(("sleep", duration)))

    verifier_module._click_dashboard_up("dashboard-id", 1.25)

    assert events == [
        ("geometry", "dashboard-id"),
        ("xdotool", ("mousemove", "220", "700")),
        ("xdotool", ("click", "--repeat", "20", "--delay", "20", "5")),
        ("sleep", 0.2),
        ("xdotool", ("mousemove", "220", "624")),
        ("xdotool", ("mousedown", "1")),
        ("sleep", 1.25),
        ("xdotool", ("mouseup", "1")),
    ]


def test_select_dashboard_plot_tab_uses_client_geometry(monkeypatch):
    """标签点击必须与按钮路径共用 xwininfo 客户区原点。"""
    events = []

    def fake_geometry(window_id):
        events.append(("geometry", window_id))
        return WindowGeometry(x=212, y=138, width=840, height=1306)

    def fake_xdotool(args):
        events.append(("xdotool", tuple(args)))
        return type(
            "Result",
            (),
            {"stdout": "X=240\nY=236\nWIDTH=840\nHEIGHT=1306\n"},
        )()

    monkeypatch.setattr(verifier_module, "_get_window_geometry", fake_geometry, raising=False)
    monkeypatch.setattr(verifier_module, "_run_xdotool", fake_xdotool)
    monkeypatch.setattr(verifier_module.time, "sleep", lambda duration: events.append(("sleep", duration)))

    verifier_module._select_dashboard_plot_tab("dashboard-id", "trajectory")

    assert events == [
        ("geometry", "dashboard-id"),
        ("xdotool", ("mousemove", "522", "262")),
        ("xdotool", ("click", "1")),
        ("sleep", 0.5),
    ]


def test_dashboard_plot_tab_point_targets_named_curve_tabs():
    geometry = WindowGeometry(x=10, y=20, width=420, height=760)

    assert verifier_module.DASHBOARD_TAB_ORDER == (
        "接口状态", "障碍物", "轨迹", "速度/命令", "打滑", "接触",
        "驱动命令", "驱动反馈", "转向命令", "转向反馈", "LiDAR点云",
        "RTK位置", "RTK航向", "IMU姿态", "轮组频率", "传感频率", "接口异常",
    )
    assert verifier_module.LEGACY_PLOT_TAB_ORDER == (
        "data", "obstacles", "trajectory", "speed", "slip", "contact"
    )
    assert verifier_module.dashboard_plot_tab_point(geometry, "obstacles") == (102, 82)
    assert verifier_module.dashboard_plot_tab_point(geometry, "trajectory") == (165, 82)
    assert verifier_module.dashboard_plot_tab_point(geometry, "speed") == (228, 82)
    assert verifier_module.dashboard_plot_tab_point(geometry, "slip") == (290, 82)
    assert verifier_module.dashboard_plot_tab_point(geometry, "contact") == (338, 82)


def test_dashboard_manual_verifier_accepts_plot_tab_argument():
    args = verifier_module.parse_args(["--plot-tab", "speed"])

    assert args.plot_tab == "speed"


def test_dashboard_manual_verifier_accepts_window_layout_options_without_default_plot_tab():
    args = verifier_module.parse_args(
        [
            "--verify-window-layout",
            "--expected-available-size",
            "1366x768",
        ]
    )

    assert args.verify_window_layout is True
    assert args.expected_available_size == (1366, 768)
    assert args.plot_tab is None


def test_dashboard_manual_verifier_accepts_seventeen_tab_gate():
    args = verifier_module.parse_args(["--verify-dashboard-tabs"])

    assert args.verify_dashboard_tabs is True


def test_build_child_command_forwards_explicit_log_dir_and_layout_report(tmp_path):
    args = verifier_module.parse_args(["--duration-sec", "4"])
    report_path = tmp_path / "dashboard-layout.jsonl"

    command, child_env = verifier_module.build_child_command(
        args,
        log_dir=tmp_path,
        layout_report_path=report_path,
    )

    assert command[-2:] == ["--log-dir", str(tmp_path)]
    assert command[command.index("--interface-mode") + 1] == "local"
    assert child_env[verifier_module.DASHBOARD_LAYOUT_REPORT_ENV] == str(report_path)
    assert child_env[verifier_module.PYBULLET_WINDOW_TOKEN_ENV]


def test_verify_dashboard_frames_visits_all_tabs_and_rejects_blank_last_frame():
    visible = Image.new("RGB", (120, 80), (242, 242, 242))
    ImageDraw.Draw(visible).rectangle((10, 10, 90, 50), fill=(30, 110, 190))
    blank = Image.new("RGB", (120, 80), (242, 242, 242))

    result = verifier_module.verify_dashboard_frames(
        [visible] * 16 + [blank],
        verifier_module.DASHBOARD_TAB_ORDER,
    )

    assert result.passed is False
    assert result.visited_tabs == 17
    assert "tab 17" in result.detail
    assert "接口异常" in result.detail


def test_validate_dashboard_layout_report_rejects_top_control_overlap():
    client = Rect(100, 50, 400, 700)
    report = {
        "tab_count": 17,
        "tab_label": "轨迹",
        "device_pixel_ratio": 1.0,
        "window_rect": [100, 50, 400, 700],
        "tabs_rect": [100, 90, 400, 360],
        "controls_rect": [100, 450, 400, 300],
        "page_rect": [108, 125, 384, 315],
        "canvas_rect": [116, 135, 368, 250],
        "legend_rect": [330, 145, 130, 70],
        "plot_button_rects": [[116, 395, 120, 30], [250, 395, 120, 30]],
        "critical_control_rects": {"暂停": [110, 470, 80, 28]},
    }

    assert verifier_module.validate_dashboard_layout_report(report, client).passed
    report["controls_rect"] = [100, 430, 400, 320]
    failed = verifier_module.validate_dashboard_layout_report(report, client)
    assert failed.passed is False
    assert "overlap" in failed.detail


def test_validate_dashboard_layout_report_maps_qt_logical_rects_to_x11_pixels():
    """真实高 DPI 桌面必须以 Qt client 为锚点映射到 X11 物理坐标。"""
    client = Rect(2070, 138, 490, 1302)
    report = {
        "tab_count": 17,
        "tab_label": "接口状态",
        "device_pixel_ratio": 2.0,
        "window_rect": [1035, 69, 245, 651],
        "tabs_rect": [1043, 113, 229, 320],
        "controls_rect": [1043, 439, 229, 273],
        "page_rect": [1045, 142, 225, 289],
        "canvas_rect": None,
        "legend_rect": None,
        "plot_button_rects": [],
        "critical_control_rects": {"暂停": [1071, 491, 156, 30]},
    }

    assert verifier_module.validate_dashboard_layout_report(report, client).passed


def test_verify_dashboard_tabs_cycles_all_pages_while_holding_drive_key(tmp_path):
    visible = Image.new("RGB", (120, 80), (242, 242, 242))
    ImageDraw.Draw(visible).rectangle((10, 10, 90, 50), fill=(30, 110, 190))
    reports = []
    for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER):
        reports.append(
            {
                "tab_index": index,
                "tab_count": 17,
                "tab_label": label,
                "device_pixel_ratio": 1.0,
                "window_rect": [100, 50, 400, 700],
                "tabs_rect": [100, 90, 400, 360],
                "controls_rect": [100, 450, 400, 300],
                "page_rect": [108, 125, 384, 315],
                "canvas_rect": [116, 135, 368, 250] if index >= 2 else None,
                "legend_rect": None,
                "plot_button_rects": [],
                "critical_control_rects": {"暂停": [110, 470, 80, 28]},
            }
        )
    commands = []
    sleeps = []
    now = {"value": 10.0}

    def sleeper(duration):
        sleeps.append(duration)
        now["value"] += duration

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=Rect(100, 50, 400, 700),
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=lambda _path, index, label, _timeout: reports[index],
        capture=lambda **_kwargs: visible,
        command_runner=lambda args: commands.append(tuple(args)),
        clock=lambda: now["value"],
        sleeper=sleeper,
    )

    assert result.passed is True
    assert result.visited_tabs == 17
    assert commands[0:2] == [("windowfocus", "dashboard-id"), ("keydown", "Up")]
    assert commands.count(("key", "Tab")) == 16
    assert commands[-2:] == [("keyup", "ctrl"), ("keyup", "Up")]
    assert sum(sleeps) >= 4.0


def test_verify_dashboard_tabs_releases_keys_when_report_wait_fails(tmp_path):
    commands = []

    def fail_report(*_args):
        raise RuntimeError("layout report timeout")

    with pytest.raises(RuntimeError, match="layout report timeout"):
        verifier_module.verify_dashboard_tabs(
            "dashboard-id",
            display=":99",
            client_rect=Rect(100, 50, 400, 700),
            layout_report_path=tmp_path / "layout.jsonl",
            hold_drive_sec=4.0,
            report_reader=fail_report,
            capture=lambda **_kwargs: Image.new("RGB", (10, 10)),
            command_runner=lambda args: commands.append(tuple(args)),
            clock=lambda: 10.0,
            sleeper=lambda _duration: None,
        )

    assert ("keyup", "ctrl") in commands
    assert commands[-1] == ("keyup", "Up")


def test_verify_dashboard_tabs_releases_keys_when_up_keydown_fails(tmp_path):
    commands = []

    def fail_after_keydown(args):
        command = tuple(args)
        commands.append(command)
        if command == ("keydown", "Up"):
            raise RuntimeError("keydown failed after sending event")

    with pytest.raises(RuntimeError, match="keydown failed after sending event"):
        verifier_module.verify_dashboard_tabs(
            "dashboard-id",
            display=":99",
            client_rect=Rect(100, 50, 400, 700),
            layout_report_path=tmp_path / "layout.jsonl",
            hold_drive_sec=4.0,
            report_reader=lambda *_args: {},
            capture=lambda **_kwargs: Image.new("RGB", (10, 10)),
            command_runner=fail_after_keydown,
            clock=lambda: 10.0,
            sleeper=lambda _duration: None,
        )

    assert commands[-2:] == [("keyup", "ctrl"), ("keyup", "Up")]


def test_run_verification_uses_fresh_layout_report_for_reused_log_dir(
    monkeypatch,
    tmp_path,
):
    report_paths = []
    args = verifier_module.parse_args(["--log-dir", str(tmp_path)])

    def capture_report_path(_args, *, log_dir, layout_report_path):
        assert log_dir == tmp_path
        report_paths.append(layout_report_path)
        return ["manual-gui"], {}

    monkeypatch.setattr(verifier_module, "build_child_command", capture_report_path)
    monkeypatch.setattr(
        verifier_module.subprocess,
        "Popen",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("stop after build")),
    )

    assert verifier_module._run_verification(args, tmp_path) == 2
    assert verifier_module._run_verification(args, tmp_path) == 2

    assert len(set(report_paths)) == 2
    assert all(path.parent == tmp_path for path in report_paths)
    assert all(path.name.startswith("dashboard-layout-") for path in report_paths)
    assert all(path.suffix == ".jsonl" for path in report_paths)


@pytest.mark.parametrize("value", ("nan", "inf"))
def test_positive_float_rejects_non_finite_values(value):
    with pytest.raises(argparse.ArgumentTypeError, match="positive"):
        verifier_module._positive_float(value)


def test_default_hold_covers_slow_xvfb_frames_during_task13_two_second_runs():
    defaults = verifier_module.parse_args([])
    task13 = verifier_module.parse_args(["--duration-sec", "2"])

    assert defaults.hold_sec == task13.hold_sec == 4.0
    assert task13.duration_sec == 2.0


def test_process_wait_timeout_preserves_xvfb_shutdown_grace():
    """软件 OpenGL 落盘较慢，2 秒驾驶不能再被旧的 10 秒预算误杀。"""
    assert verifier_module.process_wait_timeout(2.0) == pytest.approx(22.0)


def test_validate_window_layout_accepts_exact_shared_layout():
    available = Rect(10, 20, 1001, 700)
    layout = calculate_window_layout(available, True)

    assert (
        verifier_module.validate_window_layout(
            available,
            layout.main,
            layout.dashboard,
        )
        is None
    )


def test_validate_window_layout_accepts_dpr_aligned_physical_outer_rects():
    available = Rect(112, 64, 2448, 1376)
    layout = verifier_module.align_window_layout_to_scale(
        calculate_window_layout(available, True),
        2.0,
    )

    assert (
        verifier_module.validate_window_layout(
            available,
            layout.main,
            layout.dashboard,
            device_pixel_ratio=2.0,
        )
        is None
    )


@pytest.mark.parametrize(
    "main,dashboard,expected_reason",
    (
        (Rect(10, 21, 801, 699), Rect(811, 20, 200, 700), "mismatch"),
        (Rect(10, 20, 802, 700), Rect(811, 20, 200, 700), "overlap"),
    ),
)
def test_validate_window_layout_reports_shift_or_overlap(
    main,
    dashboard,
    expected_reason,
):
    available = Rect(10, 20, 1001, 700)

    with pytest.raises(ValueError) as excinfo:
        verifier_module.validate_window_layout(available, main, dashboard)

    message = str(excinfo.value)
    assert expected_reason in message
    assert "expected_main=" in message
    assert "actual_main=" in message
    assert "expected_dashboard=" in message
    assert "actual_dashboard=" in message


class _FakeClock:
    """用虚拟时间验证几何轮询有界且只按条件等待。"""

    def __init__(self):
        self.now = 0.0
        self.sleeps = []

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.sleeps.append(duration)
        self.now += duration


def test_wait_for_window_layout_rereads_until_both_rects_match():
    available = Rect(10, 20, 1000, 700)
    layout = calculate_window_layout(available, True)
    samples = iter(
        (
            (Rect(11, 20, 799, 700), Rect(810, 21, 200, 699)),
            (layout.main, layout.dashboard),
        )
    )
    reads = []
    clock = _FakeClock()

    def geometry_getter():
        sample = next(samples)
        reads.append(sample)
        return sample

    actual = verifier_module.wait_for_window_layout(
        available,
        geometry_getter=geometry_getter,
        timeout_sec=0.5,
        poll_interval_sec=0.1,
        clock=clock.monotonic,
        sleeper=clock.sleep,
    )

    assert actual == (layout.main, layout.dashboard)
    assert reads == [
        (Rect(11, 20, 799, 700), Rect(810, 21, 200, 699)),
        (layout.main, layout.dashboard),
    ]
    assert clock.sleeps == [pytest.approx(0.1)]


def test_wait_for_window_layout_timeout_reports_last_three_rects():
    available = Rect(10, 20, 1000, 700)
    first = (Rect(10, 21, 800, 699), Rect(810, 20, 200, 700))
    last = (Rect(10, 20, 801, 700), Rect(810, 20, 200, 700))
    reads = 0
    clock = _FakeClock()

    def geometry_getter():
        nonlocal reads
        reads += 1
        return first if reads == 1 else last

    with pytest.raises(RuntimeError, match="did not stabilize") as excinfo:
        verifier_module.wait_for_window_layout(
            available,
            geometry_getter=geometry_getter,
            timeout_sec=0.2,
            poll_interval_sec=0.1,
            clock=clock.monotonic,
            sleeper=clock.sleep,
        )

    message = str(excinfo.value)
    assert f"available={verifier_module._format_rect(available)}" in message
    assert f"main={verifier_module._format_rect(last[0])}" in message
    assert f"dashboard={verifier_module._format_rect(last[1])}" in message
    assert reads == 3
    assert clock.sleeps == [pytest.approx(0.1), pytest.approx(0.1)]


def test_send_dashboard_up_key_uses_windowfocus_without_window_manager(monkeypatch):
    events = []
    monkeypatch.setattr(
        verifier_module,
        "_run_xdotool",
        lambda args: events.append(("xdotool", tuple(args))),
    )
    monkeypatch.setattr(
        verifier_module.time,
        "sleep",
        lambda duration: events.append(("sleep", duration)),
    )

    verifier_module._send_dashboard_up_key("dashboard-id", 1.25)

    assert events == [
        ("xdotool", ("windowfocus", "dashboard-id")),
        ("xdotool", ("keydown", "Up")),
        ("sleep", 1.25),
        ("xdotool", ("keyup", "Up")),
    ]


def test_find_window_rejects_title_only_lookup_without_pid():
    with pytest.raises(ValueError, match="process_id"):
        verifier_module._find_window("PyBullet [main]", None, 1.0)


def test_find_window_with_pid_keeps_dashboard_process_filter(monkeypatch):
    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        return type("Result", (), {"returncode": 0, "stdout": "88\n", "stderr": ""})()

    monkeypatch.setattr(verifier_module.subprocess, "run", fake_run)

    assert verifier_module._find_window("Dashboard", 4321, 1.0) == "88"
    assert calls == [
        [
            "xdotool",
            "search",
            "--all",
            "--onlyvisible",
            "--pid",
            "4321",
            "--name",
            r"^Dashboard$",
        ]
    ]


def test_find_window_selects_reparented_client_from_frame_pair(monkeypatch):
    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        if command[0] == "xdotool":
            stdout = "41\n77\n"
        elif command[2] == "41":
            stdout = "Parent window id: 0x1 (the root window)\n"
        else:
            stdout = "Parent window id: 0x29 (frame window)\n"
        return type("Result", (), {"returncode": 0, "stdout": stdout, "stderr": ""})()

    monkeypatch.setattr(verifier_module.subprocess, "run", fake_run)

    assert verifier_module._find_window("PyBullet", 4321, 1.0) == "77"
    assert calls == [
        [
            "xdotool", "search", "--all", "--onlyvisible",
            "--pid", "4321", "--name", r"^PyBullet$",
        ],
        ["xwininfo", "-id", "41", "-tree"],
        ["xwininfo", "-id", "77", "-tree"],
    ]


def test_find_window_rejects_multiple_independent_exact_matches(monkeypatch):
    def fake_run(command, **_kwargs):
        stdout = (
            "41\n77\n"
            if command[0] == "xdotool"
            else "Parent window id: 0x1 (the root window)\n"
        )
        return type("Result", (), {"returncode": 0, "stdout": stdout, "stderr": ""})()

    monkeypatch.setattr(verifier_module.subprocess, "run", fake_run)

    with pytest.raises(RuntimeError, match=r"ambiguous.*41.*77"):
        verifier_module._find_window("PyBullet", 4321, 1.0)


def test_find_window_excludes_preexisting_ids_before_resolving_family(monkeypatch):
    calls = []

    def fake_run(command, **_kwargs):
        calls.append(command)
        if command[0] == "xdotool":
            stdout = "11\n12\n41\n77\n"
        elif command[2] == "41":
            stdout = "Parent window id: 0x1 (the root window)\n"
        else:
            stdout = "Parent window id: 0x29 (frame window)\n"
        return type("Result", (), {"returncode": 0, "stdout": stdout, "stderr": ""})()

    monkeypatch.setattr(verifier_module.subprocess, "run", fake_run)

    assert verifier_module._find_window(
        "PyBullet",
        4321,
        1.0,
        excluded_window_ids=("11", "12"),
    ) == "77"
    assert all("11" not in command and "12" not in command for command in calls[1:])


def test_dashboard_manual_verifier_requires_xwininfo(monkeypatch, capsys):
    monkeypatch.setattr(verifier_module, "display_is_available", lambda: True)
    monkeypatch.setattr(
        verifier_module.shutil,
        "which",
        lambda command: "/usr/bin/xdotool" if command == "xdotool" else None,
    )

    assert verifier_module.main([]) == 2
    assert "xwininfo is required" in capsys.readouterr().err


def test_main_prints_all_actual_rects_before_expected_size_failure(monkeypatch, capsys, tmp_path):
    """布局门禁失败前也必须留下三个真实矩形，便于定位桌面差异。"""
    available = Rect(10, 20, 1000, 700)
    layout = calculate_window_layout(available, True)

    class FakeQApplication:
        @classmethod
        def instance(cls):
            return None

        def __init__(self, _args):
            pass

    qt_widgets = ModuleType("PySide6.QtWidgets")
    qt_widgets.QApplication = FakeQApplication
    package = ModuleType("PySide6")
    package.QtWidgets = qt_widgets
    monkeypatch.setitem(sys.modules, "PySide6", package)
    monkeypatch.setitem(sys.modules, "PySide6.QtWidgets", qt_widgets)

    class FakeProcess:
        pid = 4321

        def __init__(self):
            self.terminated = False

        def poll(self):
            return None

        def terminate(self):
            self.terminated = True

        def wait(self, timeout):
            return 0

        def kill(self):
            raise AssertionError("process should terminate without kill")

    process = FakeProcess()
    geometries = {
        "main-id": layout.main,
        "dashboard-id": layout.dashboard,
    }
    monkeypatch.setattr(verifier_module, "display_is_available", lambda: True)
    monkeypatch.setattr(verifier_module.shutil, "which", lambda command: f"/usr/bin/{command}")
    popen_calls = []

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        return process

    monkeypatch.setattr(verifier_module.subprocess, "Popen", fake_popen)
    owned_calls = []

    def fake_find_owned(title, **kwargs):
        owned_calls.append((title, kwargs))
        return verifier_module.OwnedX11Window("main-id", process.pid, title)

    monkeypatch.setattr(verifier_module, "find_owned_x11_window", fake_find_owned)
    find_calls = []

    def fake_find_window(title, process_id, _timeout, *, excluded_window_ids=()):
        find_calls.append((title, process_id, excluded_window_ids))
        if title == verifier_module.PYBULLET_WINDOW_TITLE:
            raise AssertionError("Main must be resolved by token plus XRes PID")
        return "dashboard-id"

    monkeypatch.setattr(
        verifier_module,
        "_find_window",
        fake_find_window,
    )
    monkeypatch.setattr(
        verifier_module,
        "_get_window_outer_geometry",
        lambda window_id: geometries[window_id],
    )
    monkeypatch.setattr(
        verifier_module,
        "primary_display_metrics",
        lambda: SimpleNamespace(device_pixel_ratio=1.0),
    )
    monkeypatch.setattr(
        verifier_module,
        "x11_available_geometry",
        lambda _metrics: available,
    )
    monkeypatch.setattr(
        verifier_module,
        "_send_dashboard_up_key",
        lambda *_args: (_ for _ in ()).throw(AssertionError("input sent before validation")),
    )

    return_code = verifier_module.main(
        [
            "--verify-window-layout",
            "--expected-available-size",
            "999x700",
            "--duration-sec",
            "2",
            "--log-dir",
            str(tmp_path),
        ]
    )

    captured = capsys.readouterr()
    assert return_code == 1
    assert f"available={verifier_module._format_rect(available)}" in captured.out
    assert f"main={verifier_module._format_rect(layout.main)}" in captured.out
    assert f"dashboard={verifier_module._format_rect(layout.dashboard)}" in captured.out
    assert "expected=999x700" in captured.err
    assert "actual=1000x700" in captured.err
    command, popen_kwargs = popen_calls[0]
    assert command[-2:] == ["--log-dir", str(tmp_path)]
    token = popen_kwargs["env"][verifier_module.PYBULLET_WINDOW_TOKEN_ENV]
    assert owned_calls == [
        (
            token,
            {
                "expected_pid": process.pid,
                "timeout_sec": 12.0,
                "runner": verifier_module.subprocess.run,
            },
        )
    ]
    assert find_calls == [
        (verifier_module.DASHBOARD_WINDOW_TITLE, process.pid, ()),
    ]
    assert process.terminated is True


def test_dashboard_window_ids_ignores_xdotool_debug_lines():
    output = "command: search\n6292287\n67108871\n"

    assert dashboard_window_ids(output) == ["6292287", "67108871"]


def test_newest_manual_log_filters_by_prefix_and_start_time(tmp_path: Path):
    old = tmp_path / "manual_golf_heightfield_active_steering_4wd_0_20260709_163134.csv"
    new = tmp_path / "manual_golf_heightfield_active_steering_4wd_0_20260709_163201.csv"
    other = tmp_path / "manual_flat_df_back_0_new.csv"
    for path in (old, new, other):
        path.write_text("x\n0\n", encoding="utf-8")
    os.utime(old, (300.0, 300.0))
    os.utime(other, (300.0, 300.0))
    os.utime(new, (200.0, 200.0))
    after = 150.0

    assert newest_manual_log(tmp_path, after=after) == new


def test_summarize_manual_motion_extracts_dashboard_success_metrics(tmp_path: Path):
    log_path = tmp_path / "manual_golf_heightfield_active_steering_4wd_0_run.csv"
    pd.DataFrame(
        {
            "x": [1.0, 1.05, 1.14],
            "command_linear_velocity": [0.0, 0.25, 0.25],
            "body_forward_speed": [0.0, 0.20, 0.23],
            "out_of_bounds": [False, False, False],
        }
    ).to_csv(log_path, index=False)

    summary = summarize_manual_motion(log_path, tail_samples=2)

    assert summary.dx == pytest.approx(0.14)
    assert summary.max_command_linear_velocity == pytest.approx(0.25)
    assert summary.tail_body_forward_speed == pytest.approx(0.215)
    assert summary.max_body_forward_speed == pytest.approx(0.23)
    assert summary.out_of_bounds is False
    assert motion_passed(summary) is True


def test_motion_passed_rejects_no_command_no_motion_or_out_of_bounds(tmp_path: Path):
    base = ManualMotionSummary(
        log_path=tmp_path / "run.csv",
        dx=0.12,
        max_command_linear_velocity=0.25,
        tail_body_forward_speed=0.00,
        max_body_forward_speed=0.20,
        out_of_bounds=False,
    )

    assert motion_passed(base) is True
    assert motion_passed(ManualMotionSummary(base.log_path, 0.01, 0.25, 0.00, 0.20, False)) is False
    assert motion_passed(ManualMotionSummary(base.log_path, 0.12, 0.00, 0.00, 0.20, False)) is False
    assert motion_passed(ManualMotionSummary(base.log_path, 0.12, 0.25, 0.00, 0.01, False)) is False
    assert motion_passed(ManualMotionSummary(base.log_path, 0.12, 0.25, 0.00, 0.20, True)) is False
