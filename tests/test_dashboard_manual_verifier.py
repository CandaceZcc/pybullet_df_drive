# Dashboard 屏幕验收脚本测试：保护日志判定和按钮点击坐标，避免手工验收口径漂移。
from pathlib import Path
import os

import pandas as pd
import pytest
import scripts.verify_dashboard_manual_drive as verifier_module

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


def test_display_is_available_checks_display_or_wayland():
    assert display_is_available({"DISPLAY": ":0"}) is True
    assert display_is_available({"WAYLAND_DISPLAY": "wayland-0"}) is True
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
    assert verifier_module.dashboard_plot_tab_point(geometry, "trajectory") == (404, 360)
    assert verifier_module.dashboard_plot_tab_point(geometry, "speed") == (530, 360)
    assert verifier_module.dashboard_plot_tab_point(geometry, "slip") == (654, 360)
    assert verifier_module.dashboard_plot_tab_point(geometry, "contact") == (750, 360)


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
        ("xdotool", ("windowactivate", "dashboard-id")),
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
        ("xdotool", ("windowactivate", "dashboard-id")),
        ("xdotool", ("mousemove", "376", "262")),
        ("xdotool", ("click", "1")),
        ("sleep", 0.5),
    ]


def test_dashboard_plot_tab_point_targets_named_curve_tabs():
    geometry = WindowGeometry(x=10, y=20, width=420, height=760)

    assert verifier_module.dashboard_plot_tab_point(geometry, "trajectory") == (92, 82)
    assert verifier_module.dashboard_plot_tab_point(geometry, "speed") == (155, 82)
    assert verifier_module.dashboard_plot_tab_point(geometry, "slip") == (217, 82)
    assert verifier_module.dashboard_plot_tab_point(geometry, "contact") == (265, 82)


def test_dashboard_manual_verifier_accepts_plot_tab_argument():
    args = verifier_module.parse_args(["--plot-tab", "speed"])

    assert args.plot_tab == "speed"


def test_dashboard_manual_verifier_requires_xwininfo(monkeypatch, capsys):
    monkeypatch.setattr(verifier_module, "display_is_available", lambda: True)
    monkeypatch.setattr(
        verifier_module.shutil,
        "which",
        lambda command: "/usr/bin/xdotool" if command == "xdotool" else None,
    )

    assert verifier_module.main([]) == 2
    assert "xwininfo is required" in capsys.readouterr().err


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
