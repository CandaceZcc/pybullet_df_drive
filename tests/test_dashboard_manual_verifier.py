# Dashboard 屏幕验收脚本测试：保护日志判定和按钮点击坐标，避免手工验收口径漂移。
from pathlib import Path
import os

import pandas as pd
import pytest

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


def test_dashboard_up_button_point_targets_bottom_right_control_pad():
    point = dashboard_up_button_point(WindowGeometry(x=10, y=20, width=900, height=700))

    assert point == (774, 545)


def test_dashboard_window_ids_ignores_xdotool_debug_lines():
    output = "command: search\n6292287\n67108871\n"

    assert dashboard_window_ids(output) == ["6292287", "67108871"]


def test_newest_manual_log_filters_by_prefix_and_start_time(tmp_path: Path):
    old = tmp_path / "manual_dam_slope_tracked_proxy_10_20260709_163134.csv"
    new = tmp_path / "manual_dam_slope_tracked_proxy_10_20260709_163201.csv"
    other = tmp_path / "manual_box_slope_diff_drive_10_new.csv"
    for path in (old, new, other):
        path.write_text("x\n0\n", encoding="utf-8")
    os.utime(old, (300.0, 300.0))
    os.utime(other, (300.0, 300.0))
    os.utime(new, (200.0, 200.0))
    after = 150.0

    assert newest_manual_log(tmp_path, after=after) == new


def test_summarize_manual_motion_extracts_dashboard_success_metrics(tmp_path: Path):
    log_path = tmp_path / "manual_dam_slope_tracked_proxy_10_run.csv"
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
