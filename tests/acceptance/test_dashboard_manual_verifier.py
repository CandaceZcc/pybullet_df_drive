# Dashboard 屏幕验收测试：保护日志判定和按钮点击坐标，避免手工验收口径漂移。
import argparse
import copy
import json
from pathlib import Path
import os
import subprocess
import sys
from types import ModuleType, SimpleNamespace

import pandas as pd
from PIL import Image, ImageDraw
import pytest
import scripts.verify_dashboard_manual_drive as verifier_module
from slope_sim.window_layout import (
    Rect,
    align_window_layout_to_scale,
    calculate_window_layout,
)

from scripts.verify_dashboard_manual_drive import (
    ManualMotionSummary,
    WindowGeometry,
    dashboard_window_ids,
    display_is_available,
    motion_passed,
    newest_manual_log,
    summarize_manual_motion,
)


_TEST_REQUIRED_CRITICAL_CONTROLS = (
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


def _valid_dashboard_report_fixture(
    *,
    index: int,
    label: str,
    client_rect: Rect,
) -> dict[str, object]:
    """构造 schema v4 的最小有效布局报告，供 verifier 行为测试复用。"""
    page_kind = "content" if label in {"接口状态", "障碍物"} else "plot"
    if page_kind == "content":
        required_buttons: list[str] = []
    elif label == "LiDAR点云":
        required_buttons = ["保存当前图"]
    else:
        required_buttons = ["清空曲线", "保存当前图"]

    root_margin = 8
    root_spacing = 6
    title_height = 26
    panes_height = client_rect.height - 2 * root_margin - title_height - 2 * root_spacing
    tabs_height = panes_height // 2
    controls_height = panes_height - tabs_height
    tabs_top = client_rect.y + root_margin + title_height + root_spacing
    controls_top = tabs_top + tabs_height + root_spacing
    page_top = tabs_top + 35
    page_height = tabs_height - 38
    canvas_top = page_top + 10
    button_top = page_top + page_height - 35
    canvas_height = button_top - 10 - canvas_top
    axes_top = canvas_top + 30
    axes_height = canvas_height - 80
    viewport_top = controls_top + 39
    viewport = [
        client_rect.x + 8,
        viewport_top,
        client_rect.width - 16,
        controls_height - 39,
    ]
    return {
        "report_version": 4,
        "tab_index": index,
        "tab_count": len(verifier_module.DASHBOARD_TAB_ORDER),
        "tab_label": label,
        "tab_order": list(verifier_module.DASHBOARD_TAB_ORDER),
        "page_kind": page_kind,
        "required_plot_buttons": required_buttons,
        "device_pixel_ratio": 1.0,
        "window_rect": [
            client_rect.x,
            client_rect.y,
            client_rect.width,
            client_rect.height,
        ],
        "title_rect": [
            client_rect.x + 8,
            client_rect.y + 8,
            client_rect.width - 16,
            26,
        ],
        "tabs_rect": [
            client_rect.x + 8,
            tabs_top,
            client_rect.width - 16,
            tabs_height,
        ],
        "tab_bar_rect": [
            client_rect.x + 8,
            client_rect.y + 42,
            client_rect.width - 16,
            30,
        ],
        "tab_scroll_button_rects": {
            "left": [client_rect.right - 56, client_rect.y + 42, 20, 30],
            "right": [client_rect.right - 36, client_rect.y + 42, 20, 30],
        },
        "controls_rect": [
            client_rect.x + 8,
            controls_top,
            client_rect.width - 16,
            controls_height,
        ],
        "page_rect": [
            client_rect.x + 8,
            page_top,
            client_rect.width - 16,
            page_height,
        ],
        "canvas_rect": (
            [
                client_rect.x + 16,
                canvas_top,
                client_rect.width - 32,
                canvas_height,
            ]
            if page_kind == "plot"
            else None
        ),
        "axes_rect": (
            [
                client_rect.x + 50,
                axes_top,
                260,
                axes_height,
            ]
            if page_kind == "plot"
            else None
        ),
        "legend_rect": (
            [client_rect.x + 250, canvas_top + 10, 120, 60]
            if page_kind == "plot"
            else None
        ),
        "plot_button_rects": {
            name: [
                client_rect.x + 16 + button_index * 130,
                button_top,
                120,
                30,
            ]
            for button_index, name in enumerate(required_buttons)
        },
        "content_widget_rects": (
            {
                "接口状态滚动区" if label == "接口状态" else "障碍物表格": [
                    client_rect.x + 16,
                    canvas_top,
                    client_rect.width - 32,
                    page_height - 15,
                ]
            }
            if page_kind == "content"
            else {}
        ),
        "plot_artist_rects": (
            {
                "title": {
                    "text": "plot title",
                    "rect": [client_rect.x + 170, canvas_top + 5, 80, 18],
                },
                "x_label": {
                    "text": "x [m]",
                    "rect": [client_rect.x + 180, canvas_top + canvas_height - 30, 40, 18],
                },
                "y_label": {
                    "text": "y [m]",
                    "rect": [client_rect.x + 20, canvas_top + 95, 18, 40],
                },
                "x_offset": None,
                "y_offset": None,
            }
            if page_kind == "plot"
            else None
        ),
        "plot_tick_rects": (
            {
                "x": [
                    {
                        "text": "0",
                        "rect": [client_rect.x + 80, canvas_top + canvas_height - 45, 14, 16],
                    },
                    {
                        "text": "1",
                        "rect": [client_rect.x + 280, canvas_top + canvas_height - 45, 14, 16],
                    },
                ],
                "y": [
                    {
                        "text": "0",
                        "rect": [client_rect.x + 42, axes_top + axes_height - 40, 14, 16],
                    },
                    {
                        "text": "1",
                        "rect": [client_rect.x + 42, axes_top + 10, 14, 16],
                    },
                ],
            }
            if page_kind == "plot"
            else None
        ),
        "legend_text_rects": (
            [
                {
                    "text": "series",
                    "rect": [client_rect.x + 268, canvas_top + 25, 48, 16],
                }
            ]
            if page_kind == "plot"
            else None
        ),
        "qt_text_rects": {
            "tab": {
                "text": label,
                "rect": [client_rect.x + 22, client_rect.y + 48, 70, 16],
                "container_rect": [
                    client_rect.x + 8,
                    client_rect.y + 42,
                    100,
                    30,
                ],
            },
            "plot_buttons": {
                name: {
                    "text": name,
                    "rect": [
                        client_rect.x + 36 + button_index * 130,
                        button_top + 7,
                        80,
                        16,
                    ],
                    "container_rect": [
                        client_rect.x + 16 + button_index * 130,
                        button_top,
                        120,
                        30,
                    ],
                }
                for button_index, name in enumerate(required_buttons)
            },
            "critical_controls": {
                name: {
                    "text": name,
                    "rect": [
                        client_rect.x + 32,
                        viewport_top + 18,
                        100,
                        16,
                    ],
                    "container_rect": [
                        client_rect.x + 20,
                        viewport_top + 12,
                        180,
                        28,
                    ],
                }
                for name in _TEST_REQUIRED_CRITICAL_CONTROLS
            },
        },
        "control_viewport_rect": list(viewport),
        "control_content_rect": [
            client_rect.x + 8,
            viewport_top,
            client_rect.width - 16,
            900,
        ],
        "control_scroll_range": [0, 900 - viewport[3]],
        "critical_control_rects": {
            name: {
                "rect": [
                    client_rect.x + 20,
                    viewport_top + 12,
                    180,
                    28,
                ],
                "viewport_rect": list(viewport),
                "scroll_value": 0,
            }
            for name in _TEST_REQUIRED_CRITICAL_CONTROLS
        },
        "rendered_data_revision": 10 if page_kind == "plot" else None,
    }


def _scroll_aware_capture(frame: Image.Image):
    """前四次提供原始/右移/部分恢复/完全恢复，随后返回稳定截图。"""
    scrolled = frame.copy()
    ImageDraw.Draw(scrolled).rectangle((30, 42, 210, 70), fill=(170, 70, 40))
    partially_restored = frame.copy()
    ImageDraw.Draw(partially_restored).rectangle(
        (30, 42, 80, 70),
        fill=(170, 70, 40),
    )
    frames = [frame, scrolled, partially_restored, frame]
    calls = []

    def capture(**kwargs):
        calls.append(kwargs)
        if frames:
            return frames.pop(0)
        return frame

    return capture, calls


def _two_round_report_reader(report_rounds, calls):
    """按全局 JSONL 行游标返回两轮页签报告，拒绝复用旧 occurrence。"""

    def read(_path, index, label, _timeout, after_line_number):
        line_number = after_line_number + 1
        round_index, expected_index = divmod(
            line_number,
            len(verifier_module.DASHBOARD_TAB_ORDER),
        )
        assert expected_index == index
        report = report_rounds[round_index][index]
        assert report["tab_label"] == label
        calls.append((line_number, index, label, after_line_number))
        return verifier_module.DashboardLayoutReportOccurrence(
            line_number=line_number,
            report=report,
        )

    return read


def _fresh_two_round_report_reader(reports):
    """为既有交互测试生成几何不变、绘制修订前进的第二轮报告。"""
    second_round = copy.deepcopy(reports)
    for report in second_round:
        if report["rendered_data_revision"] is not None:
            report["rendered_data_revision"] += 1
    return _two_round_report_reader((reports, second_round), [])


def test_direct_script_help_bootstraps_repo_root_without_pythonpath():
    """直接运行脚本时应自行发现仓库包，不依赖外部 PYTHONPATH。"""
    root = Path(__file__).resolve().parents[2]
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


def test_button_drive_helper_is_removed():
    """正式 GUI 门禁只保留键盘驾驶，不再维护已删除按钮的坐标路径。"""
    assert not hasattr(verifier_module, "_click_dashboard_up")


def test_select_dashboard_plot_tab_uses_indexed_navigation_without_fixed_coordinates(monkeypatch):
    """旧图选择使用冻结索引，不再假设 420px Dashboard 或固定 DPR。"""
    events = []
    monkeypatch.setattr(
        verifier_module,
        "_run_xdotool",
        lambda args: events.append(("xdotool", tuple(args))),
    )
    monkeypatch.setattr(verifier_module.time, "sleep", lambda duration: events.append(("sleep", duration)))

    verifier_module._select_dashboard_plot_tab("dashboard-id", "trajectory")

    assert events == [
        ("xdotool", ("windowfocus", "dashboard-id")),
        ("xdotool", ("key", "ctrl+Tab")),
        ("xdotool", ("key", "ctrl+Tab")),
        ("sleep", 0.5),
    ]


def test_select_dashboard_data_tab_uses_explicit_diagnostic_page_index(monkeypatch):
    """显式旧数据页位于 15 个默认页之后，不能误停在企业接口状态页。"""
    events = []
    monkeypatch.setattr(
        verifier_module,
        "_run_xdotool",
        lambda args: events.append(tuple(args)),
    )
    monkeypatch.setattr(verifier_module.time, "sleep", lambda _duration: None)

    verifier_module._select_dashboard_plot_tab("dashboard-id", "data")

    assert events[0] == ("windowfocus", "dashboard-id")
    assert events.count(("key", "ctrl+Tab")) == len(
        verifier_module.DASHBOARD_TAB_ORDER
    )


def test_dashboard_tab_orders_keep_legacy_plot_mapping():
    assert verifier_module.DASHBOARD_TAB_ORDER == (
        "接口状态", "障碍物", "轨迹", "速度/命令",
        "驱动命令", "驱动反馈", "转向命令", "转向反馈", "LiDAR点云",
        "RTK位置", "RTK航向", "IMU姿态", "轮组频率", "传感频率", "接口异常",
    )
    assert verifier_module.LEGACY_PLOT_TAB_ORDER == (
        "data", "obstacles", "trajectory", "speed"
    )


def test_dashboard_manual_verifier_accepts_plot_tab_argument():
    args = verifier_module.parse_args(
        ["--no-verify-dashboard-tabs", "--plot-tab", "speed"]
    )

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


def test_dashboard_manual_verifier_enables_geometry_gate_by_default():
    args = verifier_module.parse_args([])

    assert args.verify_window_layout is True


def test_dashboard_manual_verifier_enables_complete_tab_gate_by_default():
    args = verifier_module.parse_args([])

    assert args.verify_dashboard_tabs is True


def test_button_input_is_rejected_even_when_tab_gate_is_disabled(capsys):
    with pytest.raises(SystemExit):
        verifier_module.parse_args(
            ["--input-method", "button", "--no-verify-dashboard-tabs"]
        )

    captured = capsys.readouterr()
    assert "invalid choice" in captured.err


def test_expected_available_size_uses_default_geometry_gate():
    args = verifier_module.parse_args(
        ["--expected-available-size", "1366x768"]
    )

    assert args.verify_window_layout is True
    assert args.expected_available_size == (1366, 768)


def test_dashboard_manual_verifier_accepts_fifteen_tab_gate():
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
    assert command[command.index("--figure-dir") + 1] == str(tmp_path / "figures")
    assert command[command.index("--interface-mode") + 1] == "local"
    assert child_env[verifier_module.DASHBOARD_LAYOUT_REPORT_ENV] == str(report_path)
    assert child_env[verifier_module.PYBULLET_WINDOW_TOKEN_ENV]


def test_verify_dashboard_frames_visits_all_tabs_and_rejects_blank_last_frame():
    visible = Image.new("RGB", (120, 80), (242, 242, 242))
    ImageDraw.Draw(visible).rectangle((10, 10, 90, 50), fill=(30, 110, 190))
    blank = Image.new("RGB", (120, 80), (242, 242, 242))

    result = verifier_module.verify_dashboard_frames(
        [visible] * 14 + [blank],
        verifier_module.DASHBOARD_TAB_ORDER,
    )

    assert result.passed is False
    assert result.visited_tabs == 15
    assert "tab 15" in result.detail
    assert "接口异常" in result.detail


@pytest.mark.parametrize(
    ("logical_height", "device_pixel_ratio", "expected_passed"),
    (
        (304, 1.0, False),
        (320, 2.0, False),
        (599, 1.25, False),
        (600, 1.0, True),
        (651, 2.0, True),
        (768, 1.0, True),
    ),
)
def test_validate_dashboard_layout_report_uses_logical_formal_client_height(
    logical_height,
    device_pixel_ratio,
    expected_passed,
):
    """正式高度按 Qt 逻辑像素判断，不能被高 DPR 物理尺寸绕过。"""
    assert getattr(verifier_module, "DASHBOARD_FORMAL_MIN_CLIENT_HEIGHT", None) == 600
    logical_client = Rect(100, 50, 400, logical_height)
    physical_client = Rect(
        200,
        100,
        round(logical_client.width * device_pixel_ratio),
        round(logical_client.height * device_pixel_ratio),
    )
    report = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=logical_client,
    )
    report["device_pixel_ratio"] = device_pixel_ratio

    result = verifier_module.validate_dashboard_layout_report(report, physical_client)

    assert result.passed is expected_passed
    if not expected_passed:
        assert "at least 600" in result.detail


def test_validate_dashboard_layout_report_rejects_top_control_overlap():
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )

    assert verifier_module.validate_dashboard_layout_report(report, client).passed
    report["controls_rect"] = [100, 410, 400, 340]
    failed = verifier_module.validate_dashboard_layout_report(report, client)
    assert failed.passed is False
    assert "overlap" in failed.detail


def test_validate_dashboard_layout_report_rejects_legacy_sixty_forty_split():
    """正式默认路径必须直接拒绝旧 360:300，而非只检查上下区不重叠。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=client,
    )
    report["tabs_rect"] = [client.x, client.y + 40, client.width, 360]
    report["controls_rect"] = [client.x, client.y + 400, client.width, 300]

    result = verifier_module.validate_dashboard_layout_report(report, client)

    assert result.passed is False
    assert "50:50" in result.detail


@pytest.mark.parametrize(
    "title_rect",
    (
        None,
        [100, 58, 384, 26],
    ),
)
def test_validate_dashboard_layout_report_requires_real_title_geometry(title_rect):
    """缺失或未遵守根边距的标题矩形都不能通过正式布局门禁。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=client,
    )
    if title_rect is None:
        report.pop("title_rect")
    else:
        report["title_rect"] = title_rect

    result = verifier_module.validate_dashboard_layout_report(report, client)

    assert result.passed is False
    assert "title_rect" in result.detail


def test_validate_dashboard_layout_report_rejects_equal_panes_that_jointly_shrink():
    """上下 pane 即使等高，也必须共同铺满标题下方的全部可用高度。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=client,
    )
    report["tabs_rect"][3] = 303
    report["controls_rect"][1] = client.y + 349
    report["controls_rect"][3] = 303
    report["page_rect"][3] = 265
    report["content_widget_rects"]["接口状态滚动区"][3] = 250
    report["control_viewport_rect"][3] = 244
    for control in report["critical_control_rects"].values():
        control["viewport_rect"][3] = 244

    result = verifier_module.validate_dashboard_layout_report(report, client)

    assert result.passed is False
    assert "available vertical space" in result.detail


def test_validate_dashboard_layout_report_allows_one_physical_dpr_alignment_step():
    """50:50 允许不超过一个 DPR 对齐步的整数像素余数。"""
    logical_client = Rect(1035, 69, 245, 700)
    physical_client = Rect(2070, 138, 490, 1400)
    report = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=logical_client,
    )
    report["device_pixel_ratio"] = 2.0
    report["tabs_rect"][3] -= 1

    result = verifier_module.validate_dashboard_layout_report(report, physical_client)

    assert result.passed, result.detail


@pytest.mark.parametrize(
    ("field", "coordinate", "delta"),
    (
        ("tabs_rect", 3, -1),
        ("controls_rect", 3, -1),
        ("page_rect", 0, 1),
        ("canvas_rect", 0, 1),
        ("axes_rect", 0, 1),
    ),
)
def test_validate_dashboard_layout_stability_rejects_geometry_drift(
    field,
    coordinate,
    delta,
):
    """数据重绘前后五个稳定矩形中任一个漂移都必须失败。"""
    client = Rect(100, 50, 400, 700)
    before = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    after = copy.deepcopy(before)
    after["rendered_data_revision"] += 1
    after[field][coordinate] += delta

    result = verifier_module.validate_dashboard_layout_stability(
        before,
        after,
        client,
    )

    assert result.passed is False
    assert field in result.detail


def test_validate_dashboard_layout_stability_allows_new_data_with_same_geometry():
    """artist 内容与数据修订变化合法，但五个稳定矩形必须保持不变。"""
    client = Rect(100, 50, 400, 700)
    before = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    after = copy.deepcopy(before)
    after["rendered_data_revision"] += 1
    after["plot_tick_rects"]["x"][0]["text"] = "2"

    result = verifier_module.validate_dashboard_layout_stability(
        before,
        after,
        client,
    )

    assert result.passed, result.detail


def test_wait_for_dashboard_layout_report_returns_only_after_line_cursor(tmp_path):
    """reader 必须返回游标后的新 occurrence，不能倒序复用第一轮旧报告。"""
    client = Rect(100, 50, 400, 700)
    first = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    unrelated = _valid_dashboard_report_fixture(
        index=3,
        label="速度/命令",
        client_rect=client,
    )
    second = copy.deepcopy(first)
    second["rendered_data_revision"] += 1
    report_path = tmp_path / "layout.jsonl"
    report_path.write_text(
        "\n".join(
            json.dumps(report, ensure_ascii=False)
            for report in (first, unrelated, second)
        ),
        encoding="utf-8",
    )

    first_occurrence = verifier_module.wait_for_dashboard_layout_report(
        report_path,
        2,
        "轨迹",
        0.2,
        -1,
    )
    second_occurrence = verifier_module.wait_for_dashboard_layout_report(
        report_path,
        2,
        "轨迹",
        0.2,
        first_occurrence.line_number,
    )

    assert first_occurrence.line_number == 0
    assert second_occurrence.line_number == 2
    assert second_occurrence.report["rendered_data_revision"] == 11


def test_validate_dashboard_layout_report_maps_qt_logical_rects_to_x11_pixels():
    """真实高 DPI 桌面必须以 Qt client 为锚点映射到 X11 物理坐标。"""
    client = Rect(2070, 138, 490, 1400)
    report = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=Rect(1035, 69, 245, 700),
    )
    report["device_pixel_ratio"] = 2.0

    assert verifier_module.validate_dashboard_layout_report(report, client).passed


def test_validate_dashboard_layout_report_requires_explicit_page_contract():
    """验收报告不能靠缺失 canvas/按钮字段把绘图页伪装成普通内容页。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    report.pop("page_kind")

    failed = verifier_module.validate_dashboard_layout_report(report, client)

    assert failed.passed is False
    assert "page_kind" in failed.detail


def test_validate_dashboard_layout_report_rejects_empty_or_unreachable_critical_controls():
    """下半区必须覆盖完整关键控件，且逐个滚动后真实进入 viewport。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=client,
    )
    report["critical_control_rects"] = {}

    empty = verifier_module.validate_dashboard_layout_report(report, client)
    assert empty.passed is False
    assert "critical" in empty.detail

    report["critical_control_rects"] = {
        name: {
            "rect": [120, 760, 180, 28],
            "viewport_rect": [108, 458, 376, 284],
            "scroll_value": 616,
        }
        for name in verifier_module.DASHBOARD_REQUIRED_CRITICAL_CONTROLS
    }
    unreachable = verifier_module.validate_dashboard_layout_report(report, client)
    assert unreachable.passed is False
    assert "viewport" in unreachable.detail


def test_validate_dashboard_layout_report_requires_plot_canvas_legend_and_buttons():
    """折线页的画布、图例和两个命令按钮都是强制契约。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    report["canvas_rect"] = None

    failed = verifier_module.validate_dashboard_layout_report(report, client)

    assert failed.passed is False
    assert "canvas_rect" in failed.detail


def test_validate_dashboard_layout_report_rejects_undersized_active_axes():
    """正式门禁必须直接拒绝未铺满画布主体的 active axes。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=8,
        label="LiDAR点云",
        client_rect=client,
    )
    report["axes_rect"] = [150, 165, 100, 100]

    failed = verifier_module.validate_dashboard_layout_report(report, client)

    assert failed.passed is False
    assert "axes_rect" in failed.detail


@pytest.mark.parametrize("device_pixel_ratio", (1.0, 1.25, 2.0))
def test_validate_dashboard_layout_report_accepts_exact_canvas_page_coverage(
    device_pixel_ratio,
):
    """85%/70% 边界在分数 DPR 取整后仍应保留一个物理像素余量。"""
    logical_client = Rect(100, 50, 400, 700)
    physical_client = Rect(
        200,
        100,
        round(logical_client.width * device_pixel_ratio),
        round(logical_client.height * device_pixel_ratio),
    )
    report = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=logical_client,
    )
    report["device_pixel_ratio"] = device_pixel_ratio
    report["page_rect"] = [108, 125, 360, 280]
    report["canvas_rect"] = [108, 135, 306, 196]
    report["legend_rect"] = [280, 180, 120, 60]
    report["legend_text_rects"][0]["rect"] = [298, 195, 48, 16]
    report["plot_artist_rects"]["x_label"]["rect"] = [280, 305, 40, 18]
    for tick in report["plot_tick_rects"]["x"]:
        tick["rect"][1] = 280

    result = verifier_module.validate_dashboard_layout_report(report, physical_client)

    assert result.passed, result.detail


@pytest.mark.parametrize(
    ("field", "value"),
    (("width", 305), ("height", 195)),
)
@pytest.mark.parametrize("device_pixel_ratio", (1.0, 2.0))
def test_validate_dashboard_layout_report_rejects_canvas_just_below_coverage(
    field,
    value,
    device_pixel_ratio,
):
    """canvas 比 85%/70% 逻辑边界少 1px 时必须失败。"""
    logical_client = Rect(100, 50, 400, 700)
    physical_client = Rect(
        200,
        100,
        round(logical_client.width * device_pixel_ratio),
        round(logical_client.height * device_pixel_ratio),
    )
    report = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=logical_client,
    )
    report["device_pixel_ratio"] = device_pixel_ratio
    report["page_rect"] = [108, 125, 360, 280]
    report["canvas_rect"] = [108, 135, 306, 196]
    report["canvas_rect"][2 if field == "width" else 3] = value

    result = verifier_module.validate_dashboard_layout_report(report, physical_client)

    assert result.passed is False
    assert "canvas_rect" in result.detail and "page_rect" in result.detail


def test_validate_dashboard_layout_report_rejects_canvas_with_low_page_coverage():
    """画布即使完整位于页面内，明显缩矮后也不能通过正式门禁。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    report["canvas_rect"][3] = 190
    report["plot_artist_rects"]["x_label"]["rect"][1] = client.y + 255
    for tick in report["plot_tick_rects"]["x"]:
        tick["rect"][1] = client.y + 245

    result = verifier_module.validate_dashboard_layout_report(report, client)

    assert result.passed is False
    assert "canvas_rect" in result.detail
    assert "page_rect" in result.detail


def test_validate_dashboard_layout_report_enforces_line_lidar_and_content_contracts():
    """三类页面必须分别锁定图例、按钮和禁止出现的图表字段。"""
    client = Rect(100, 50, 400, 700)

    line = _valid_dashboard_report_fixture(index=2, label="轨迹", client_rect=client)
    line["legend_rect"] = None
    assert "legend_rect" in verifier_module.validate_dashboard_layout_report(
        line,
        client,
    ).detail

    line = _valid_dashboard_report_fixture(index=2, label="轨迹", client_rect=client)
    line["plot_button_rects"].pop("保存当前图")
    assert "plot_button_rects" in verifier_module.validate_dashboard_layout_report(
        line,
        client,
    ).detail

    lidar = _valid_dashboard_report_fixture(
        index=8,
        label="LiDAR点云",
        client_rect=client,
    )
    assert verifier_module.validate_dashboard_layout_report(lidar, client).passed
    lidar["legend_rect"] = None
    assert "legend_rect" in verifier_module.validate_dashboard_layout_report(
        lidar,
        client,
    ).detail

    content = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=client,
    )
    assert verifier_module.validate_dashboard_layout_report(content, client).passed
    content["canvas_rect"] = [116, 135, 368, 250]
    assert "canvas_rect" in verifier_module.validate_dashboard_layout_report(
        content,
        client,
    ).detail


def test_validate_dashboard_layout_report_rejects_missing_content_controls_and_artist_overlap():
    """内容页控件和绘图文字必须完全包含，xlabel 不能与 offset 重叠。"""
    client = Rect(100, 50, 400, 700)

    content = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=client,
    )
    content["content_widget_rects"] = {}
    missing = verifier_module.validate_dashboard_layout_report(content, client)
    assert missing.passed is False
    assert "content_widget_rects" in missing.detail

    trajectory = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    trajectory["plot_artist_rects"]["x_offset"] = {
        "text": "+1e6",
        "rect": [295, 335, 40, 18],
    }
    trajectory["plot_artist_rects"]["x_label"]["rect"] = [280, 335, 40, 18]
    overlap = verifier_module.validate_dashboard_layout_report(trajectory, client)
    assert overlap.passed is False
    assert "overlap" in overlap.detail


def test_validate_dashboard_layout_report_rejects_tick_legend_and_qt_text_defects():
    """schema v4 必须能直接抓住 tick、legend 与 Qt 文字的越界或重叠。"""
    client = Rect(100, 50, 400, 700)

    missing_ticks = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    missing_ticks.pop("plot_tick_rects")
    result = verifier_module.validate_dashboard_layout_report(missing_ticks, client)
    assert result.passed is False
    assert "plot_tick_rects" in result.detail

    overlapping_ticks = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    overlapping_ticks["plot_tick_rects"]["x"][1]["rect"] = [180, 320, 14, 16]
    result = verifier_module.validate_dashboard_layout_report(overlapping_ticks, client)
    assert result.passed is False
    assert "tick" in result.detail and "overlap" in result.detail

    legend_text_outside = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    legend_text_outside["legend_text_rects"][0]["rect"] = [120, 160, 48, 16]
    result = verifier_module.validate_dashboard_layout_report(legend_text_outside, client)
    assert result.passed is False
    assert "legend text" in result.detail

    clipped_qt_text = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    clipped_qt_text["qt_text_rects"]["plot_buttons"]["保存当前图"]["rect"] = [
        350,
        402,
        120,
        16,
    ]
    result = verifier_module.validate_dashboard_layout_report(clipped_qt_text, client)
    assert result.passed is False
    assert "Qt text" in result.detail


def test_validate_dashboard_layout_report_requires_both_tab_scroll_buttons():
    """正式 15 页门禁必须具备可实际点击的左右标签栏滚动按钮。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=client,
    )
    report["tab_scroll_button_rects"].pop("left")

    failed = verifier_module.validate_dashboard_layout_report(report, client)

    assert failed.passed is False
    assert "tab_scroll_button_rects" in failed.detail


def test_validate_dashboard_layout_report_allows_one_pixel_native_tab_button_border():
    """Qt 原生左右箭头共享 1px 边框合法，更深的物理重叠仍必须失败。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=client,
    )
    report["tab_scroll_button_rects"]["left"] = [444, 92, 21, 30]

    assert verifier_module.validate_dashboard_layout_report(report, client).passed

    report["tab_scroll_button_rects"]["left"] = [444, 92, 22, 30]
    failed = verifier_module.validate_dashboard_layout_report(report, client)
    assert failed.passed is False
    assert "overlap" in failed.detail


def test_validate_dashboard_layout_report_requires_strict_schema_and_scroll_evidence():
    """v3 标量类型、页签顺序和滚动证据都不能被宽松相等或缺省绕过。"""
    client = Rect(100, 50, 400, 700)

    for field, invalid_value in (
        ("report_version", 3.0),
        ("tab_count", 15.0),
        ("tab_index", 3),
        ("tab_order", list(reversed(verifier_module.DASHBOARD_TAB_ORDER))),
    ):
        report = _valid_dashboard_report_fixture(
            index=2,
            label="轨迹",
            client_rect=client,
        )
        report[field] = invalid_value
        assert verifier_module.validate_dashboard_layout_report(
            report,
            client,
        ).passed is False

    report = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    report["control_scroll_range"] = [1, 616]
    assert "control_scroll_range" in verifier_module.validate_dashboard_layout_report(
        report,
        client,
    ).detail

    report = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    report["critical_control_rects"]["暂停"]["scroll_value"] = 617
    assert "scroll_value" in verifier_module.validate_dashboard_layout_report(
        report,
        client,
    ).detail

    report = _valid_dashboard_report_fixture(
        index=2,
        label="轨迹",
        client_rect=client,
    )
    report["critical_control_rects"]["暂停"]["viewport_rect"][0] += 1
    assert "viewport" in verifier_module.validate_dashboard_layout_report(
        report,
        client,
    ).detail


def test_layout_report_allows_explicit_diagnostics_without_weakening_default_gate():
    """旧图显式诊断可多一个末页，默认正式契约仍必须严格保持 15 页。"""
    client = Rect(100, 50, 400, 700)
    report = _valid_dashboard_report_fixture(
        index=0,
        label="接口状态",
        client_rect=client,
    )
    diagnostic_order = (*verifier_module.DASHBOARD_TAB_ORDER, "开发者诊断")
    report["tab_count"] = len(diagnostic_order)
    report["tab_order"] = list(diagnostic_order)

    assert verifier_module.validate_dashboard_layout_report(report, client).passed is False
    result = verifier_module.validate_dashboard_layout_report(
        report,
        client,
        expected_tab_order=diagnostic_order,
    )
    assert result.passed, result.detail


def test_verify_dashboard_tabs_cycles_all_pages_while_holding_drive_key(tmp_path):
    client = Rect(100, 50, 400, 700)
    visible = Image.new("RGB", (client.width, client.height), (242, 242, 242))
    ImageDraw.Draw(visible).rectangle((28, 95, 280, 240), fill=(30, 110, 190))
    reports = [
        _valid_dashboard_report_fixture(
            index=index,
            label=label,
            client_rect=client,
        )
        for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER)
    ]
    commands = []
    sleeps = []
    now = {"value": 10.0}
    capture, _capture_calls = _scroll_aware_capture(visible)

    def sleeper(duration):
        sleeps.append(duration)
        now["value"] += duration

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=client,
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=_fresh_two_round_report_reader(reports),
        capture=capture,
        command_runner=lambda args: commands.append(tuple(args)),
        clock=lambda: now["value"],
        sleeper=sleeper,
    )

    assert result.passed is True
    assert result.visited_tabs == 15
    assert commands[0:2] == [("windowfocus", "dashboard-id"), ("keydown", "Up")]
    assert commands.count(("key", "Tab")) == 29
    expected_plot_clicks = sum(
        len(report["required_plot_buttons"])
        for report in reports
    )
    assert commands.count(("click", "1")) == 3 + expected_plot_clicks
    right_move = commands.index(("mousemove", "474", "107"))
    left_move = commands.index(("mousemove", "454", "107"))
    assert commands[right_move + 1] == ("click", "1")
    assert commands[left_move + 1] == ("click", "1")
    assert commands.count(("mousemove", "454", "107")) == 2
    assert commands[-2:] == [("keyup", "ctrl"), ("keyup", "Up")]
    assert sum(sleeps) == pytest.approx(4.0)


def test_verify_dashboard_tabs_reads_two_fresh_occurrences_by_default(tmp_path):
    """正式默认路径必须遍历两轮，并让第二轮跳过每页的旧 JSONL 行。"""
    client = Rect(100, 50, 400, 700)
    visible = Image.new("RGB", (client.width, client.height), (242, 242, 242))
    ImageDraw.Draw(visible).rectangle((28, 95, 280, 240), fill=(30, 110, 190))
    first_round = [
        _valid_dashboard_report_fixture(index=index, label=label, client_rect=client)
        for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER)
    ]
    second_round = copy.deepcopy(first_round)
    for report in second_round:
        if report["rendered_data_revision"] is not None:
            report["rendered_data_revision"] += 1
            report["plot_tick_rects"]["x"][0]["text"] = "2"
    calls = []
    capture, _capture_calls = _scroll_aware_capture(visible)

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=client,
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=_two_round_report_reader(
            (first_round, second_round),
            calls,
        ),
        capture=capture,
        command_runner=lambda _args: None,
        clock=lambda: 10.0,
        sleeper=lambda _duration: None,
    )

    assert result.passed, result.detail
    assert len(calls) == 2 * len(verifier_module.DASHBOARD_TAB_ORDER)
    assert [call[0] for call in calls] == list(range(30))
    assert [call[3] for call in calls] == list(range(-1, 29))


def test_verify_dashboard_tabs_fails_when_second_round_axes_geometry_changes(tmp_path):
    """两轮默认门禁必须实际消费稳定性结果，不能只新增未调用的 helper。"""
    client = Rect(100, 50, 400, 700)
    visible = Image.new("RGB", (client.width, client.height), (242, 242, 242))
    ImageDraw.Draw(visible).rectangle((28, 95, 280, 240), fill=(30, 110, 190))
    first_round = [
        _valid_dashboard_report_fixture(index=index, label=label, client_rect=client)
        for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER)
    ]
    second_round = copy.deepcopy(first_round)
    for report in second_round:
        if report["rendered_data_revision"] is not None:
            report["rendered_data_revision"] += 1
    second_round[2]["axes_rect"][2] += 8
    capture, _capture_calls = _scroll_aware_capture(visible)

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=client,
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=_two_round_report_reader((first_round, second_round), []),
        capture=capture,
        command_runner=lambda _args: None,
        clock=lambda: 10.0,
        sleeper=lambda _duration: None,
    )

    assert result.passed is False
    assert "axes_rect" in result.detail


def test_verify_dashboard_tabs_rejects_scroll_buttons_without_visual_state_change(tmp_path):
    """左右按钮即使收到了 click，标签栏未右移也不能通过正式门禁。"""
    client = Rect(100, 50, 400, 700)
    frame = Image.new("RGB", (client.width, client.height), (242, 242, 242))
    ImageDraw.Draw(frame).rectangle((28, 95, 280, 240), fill=(30, 110, 190))
    reports = [
        _valid_dashboard_report_fixture(index=index, label=label, client_rect=client)
        for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER)
    ]

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=client,
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=_fresh_two_round_report_reader(reports),
        capture=lambda **_kwargs: frame,
        command_runner=lambda _args: None,
        clock=lambda: 10.0,
        sleeper=lambda _duration: None,
    )

    assert result.passed is False
    assert "scroll" in result.detail


def test_verify_dashboard_tabs_bounds_left_clicks_when_tab_bar_never_restores(tmp_path):
    """左移必须有界；像素始终未恢复时不能循环或放宽恢复判据。"""
    client = Rect(100, 50, 400, 700)
    frame = Image.new("RGB", (client.width, client.height), (242, 242, 242))
    ImageDraw.Draw(frame).rectangle((28, 95, 280, 240), fill=(30, 110, 190))
    scrolled = frame.copy()
    ImageDraw.Draw(scrolled).rectangle((30, 42, 210, 70), fill=(170, 70, 40))
    partially_restored = frame.copy()
    ImageDraw.Draw(partially_restored).rectangle(
        (30, 42, 80, 70),
        fill=(170, 70, 40),
    )
    captures = [frame, scrolled]
    commands = []
    reports = [
        _valid_dashboard_report_fixture(index=index, label=label, client_rect=client)
        for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER)
    ]

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=client,
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=_fresh_two_round_report_reader(reports),
        capture=lambda **_kwargs: captures.pop(0) if captures else partially_restored,
        command_runner=lambda args: commands.append(tuple(args)),
        clock=lambda: 10.0,
        sleeper=lambda _duration: None,
    )

    assert result.passed is False
    expected_attempts = len(verifier_module.DASHBOARD_TAB_ORDER)
    assert f"after {expected_attempts} left clicks" in result.detail
    assert commands.count(("click", "1")) == 1 + expected_attempts


def test_verify_dashboard_tabs_ignores_scroll_button_focus_pixels(tmp_path):
    """按钮自身的 hover/focus 变化不能冒充标签内容发生了滚动。"""
    client = Rect(100, 50, 400, 700)
    frame = Image.new("RGB", (client.width, client.height), (242, 242, 242))
    ImageDraw.Draw(frame).rectangle((28, 95, 280, 240), fill=(30, 110, 190))
    button_focused = frame.copy()
    ImageDraw.Draw(button_focused).rectangle((345, 42, 383, 70), fill=(170, 70, 40))
    captures = [frame, button_focused, frame]
    reports = [
        _valid_dashboard_report_fixture(index=index, label=label, client_rect=client)
        for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER)
    ]

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=client,
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=_fresh_two_round_report_reader(reports),
        capture=lambda **_kwargs: captures.pop(0) if captures else frame,
        command_runner=lambda _args: None,
        clock=lambda: 10.0,
        sleeper=lambda _duration: None,
    )

    assert result.passed is False
    assert "did not change" in result.detail


def test_verify_dashboard_tabs_clicks_every_reported_plot_button(tmp_path):
    """15 页路径必须真实点击每个图表页报告出的完整按钮集合。"""
    client = Rect(100, 50, 400, 700)
    frame = Image.new("RGB", (client.width, client.height), (242, 242, 242))
    ImageDraw.Draw(frame).rectangle((28, 95, 280, 240), fill=(30, 110, 190))
    reports = [
        _valid_dashboard_report_fixture(index=index, label=label, client_rect=client)
        for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER)
    ]
    capture, _capture_calls = _scroll_aware_capture(frame)
    commands = []

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=client,
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=_fresh_two_round_report_reader(reports),
        capture=capture,
        command_runner=lambda args: commands.append(tuple(args)),
        clock=lambda: 10.0,
        sleeper=lambda _duration: None,
    )

    expected_plot_clicks = sum(
        len(report["required_plot_buttons"])
        for report in reports
    )
    assert result.passed is True
    assert commands.count(("click", "1")) == 3 + expected_plot_clicks


def test_verify_dashboard_tabs_starts_hold_timer_after_up_keydown(tmp_path):
    """聚焦窗口的耗时不能侵占成功按下 Up 后的四秒驾驶预算。"""
    client = Rect(100, 50, 400, 700)
    frame = Image.new("RGB", (client.width, client.height), (242, 242, 242))
    ImageDraw.Draw(frame).rectangle((28, 95, 280, 240), fill=(30, 110, 190))
    reports = [
        _valid_dashboard_report_fixture(
            index=index,
            label=label,
            client_rect=client,
        )
        for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER)
    ]
    now = {"value": 0.0}
    sleeps = []
    capture, _capture_calls = _scroll_aware_capture(frame)

    def run_command(args):
        command = tuple(args)
        if command == ("windowfocus", "dashboard-id"):
            now["value"] = 1.5
        elif command == ("keydown", "Up"):
            now["value"] = 2.0

    def sleeper(duration):
        sleeps.append(duration)
        now["value"] += duration

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=client,
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=_fresh_two_round_report_reader(reports),
        capture=capture,
        command_runner=run_command,
        clock=lambda: now["value"],
        sleeper=sleeper,
    )

    assert result.passed is True
    assert sleeps[:3] == [0.05, 0.05, 0.05]
    assert sum(sleeps) == pytest.approx(4.0)


def test_verify_dashboard_tabs_captures_full_client_then_crops_reported_page(tmp_path):
    """ImageGrab 只能截完整 client，页内容必须从同一张客户区图中二次裁剪。"""
    client = Rect(100, 50, 400, 700)
    full_client = Image.new("RGB", (400, 700), (242, 242, 242))
    ImageDraw.Draw(full_client).rectangle((28, 95, 280, 240), fill=(30, 110, 190))
    capture, capture_calls = _scroll_aware_capture(full_client)
    reports = []
    for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER):
        reports.append(
            _valid_dashboard_report_fixture(
                index=index,
                label=label,
                client_rect=client,
            )
        )

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=client,
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=_fresh_two_round_report_reader(reports),
        capture=capture,
        command_runner=lambda _args: None,
        clock=lambda: 10.0,
        sleeper=lambda _duration: None,
    )

    assert result.passed is True
    assert capture_calls == [
        {"bbox": (100, 50, 500, 750), "xdisplay": ":99"}
    ] * (len(verifier_module.DASHBOARD_TAB_ORDER) + 4)


def test_verify_dashboard_tabs_ignores_content_outside_reported_page(tmp_path):
    """客户区其他区域再丰富，也不能掩盖报告目标页本身为空。"""
    client = Rect(100, 50, 400, 700)
    full_client = Image.new("RGB", (400, 700), (242, 242, 242))
    ImageDraw.Draw(full_client).rectangle((20, 500, 360, 650), fill=(30, 110, 190))
    reports = [
        _valid_dashboard_report_fixture(
            index=index,
            label=label,
            client_rect=client,
        )
        for index, label in enumerate(verifier_module.DASHBOARD_TAB_ORDER)
    ]
    capture, _capture_calls = _scroll_aware_capture(full_client)

    result = verifier_module.verify_dashboard_tabs(
        "dashboard-id",
        display=":99",
        client_rect=client,
        layout_report_path=tmp_path / "layout.jsonl",
        hold_drive_sec=4.0,
        report_reader=_fresh_two_round_report_reader(reports),
        capture=capture,
        command_runner=lambda _args: None,
        clock=lambda: 10.0,
        sleeper=lambda _duration: None,
    )

    assert result.passed is False
    assert result.visited_tabs == 1
    assert "density" in result.detail


def test_verify_dashboard_frames_rejects_single_pixel_border_and_separator():
    """背景加 1px 边框/分隔线不能冒充真实文字、折线或点云内容。"""
    border = Image.new("RGB", (120, 80), (242, 242, 242))
    draw = ImageDraw.Draw(border)
    draw.rectangle((0, 0, 119, 79), outline=(30, 110, 190), width=1)
    draw.line((0, 40, 119, 40), fill=(30, 110, 190), width=1)

    result = verifier_module.verify_dashboard_frames(
        [border] * len(verifier_module.DASHBOARD_TAB_ORDER),
        verifier_module.DASHBOARD_TAB_ORDER,
    )

    assert result.passed is False
    assert "coverage" in result.detail or "density" in result.detail


def test_verify_dashboard_frames_rejects_single_pixel_cross():
    """贯穿整页的一横一竖仍只是分隔线，不能冒充页面内容。"""
    cross = Image.new("RGB", (120, 80), (242, 242, 242))
    draw = ImageDraw.Draw(cross)
    draw.line((0, 40, 119, 40), fill=(30, 110, 190), width=1)
    draw.line((60, 0, 60, 79), fill=(30, 110, 190), width=1)

    result = verifier_module.verify_dashboard_frames(
        [cross] * len(verifier_module.DASHBOARD_TAB_ORDER),
        verifier_module.DASHBOARD_TAB_ORDER,
    )

    assert result.passed is False
    assert "coverage" in result.detail


def test_verify_dashboard_frames_accepts_text_curve_and_sparse_point_content():
    """阈值必须保留文字、折线和稀疏点云三类真实页面。"""
    text = Image.new("RGB", (120, 80), (242, 242, 242))
    ImageDraw.Draw(text).text((12, 20), "status 100 Hz", fill=(30, 110, 190))

    curve = Image.new("RGB", (120, 80), (242, 242, 242))
    curve_draw = ImageDraw.Draw(curve)
    curve_draw.line((10, 65, 10, 10, 110, 10), fill=(80, 80, 80), width=1)
    curve_draw.line((12, 58, 35, 45, 58, 52, 82, 25, 108, 34), fill=(30, 110, 190), width=2)

    points = Image.new("RGB", (120, 80), (242, 242, 242))
    points_draw = ImageDraw.Draw(points)
    for x, y in ((18, 20), (34, 55), (52, 35), (71, 62), (88, 24), (104, 46)):
        points_draw.ellipse((x - 2, y - 2, x + 2, y + 2), fill=(30, 110, 190))

    for frame in (text, curve, points):
        result = verifier_module.verify_dashboard_frames(
            [frame] * len(verifier_module.DASHBOARD_TAB_ORDER),
            verifier_module.DASHBOARD_TAB_ORDER,
        )
        assert result.passed is True, result.detail


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


def test_validate_window_layout_uses_independent_thirty_three_percent_oracle(
    monkeypatch,
):
    """验收比例不得复用生产布局 helper，否则同一错误会双向自洽。"""
    available = Rect(10, 20, 1000, 700)
    main = Rect(10, 20, 670, 700)
    dashboard = Rect(680, 20, 330, 700)
    forbidden = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        AssertionError("production layout helper must not be called")
    )
    monkeypatch.setattr(
        verifier_module,
        "calculate_window_layout",
        forbidden,
        raising=False,
    )
    monkeypatch.setattr(
        verifier_module,
        "align_window_layout_to_scale",
        forbidden,
        raising=False,
    )

    assert verifier_module.validate_window_layout(
        available,
        main,
        dashboard,
    ) is None


@pytest.mark.parametrize(
    ("available_width", "device_pixel_ratio", "dashboard_width"),
    (
        (1366, 1.0, 451),
        (1920, 1.0, 634),
        (2560, 1.0, 845),
        (1366, 1.25, 451),
        (1366, 1.5, 452),
        (1366, 2.0, 450),
        (2448, 2.0, 808),
        (1365, 1.1, 451),
    ),
)
def test_validate_window_layout_accepts_only_literal_dpr_aligned_width(
    available_width,
    device_pixel_ratio,
    dashboard_width,
):
    """独立 oracle 应把规格比例和 DPR 对齐收敛为唯一物理宽度。"""
    available = Rect(0, 0, available_width, 768)
    main_width = available_width - dashboard_width

    assert verifier_module.validate_window_layout(
        available,
        Rect(0, 0, main_width, 768),
        Rect(main_width, 0, dashboard_width, 768),
        device_pixel_ratio=device_pixel_ratio,
    ) is None


@pytest.mark.parametrize(
    ("available_width", "device_pixel_ratio", "dashboard_width"),
    (
        (1366, 1.0, 450),
        (1920, 1.0, 633),
        (2560, 1.0, 844),
        (2448, 2.0, 806),
        (2448, 2.0, 807),
        (2448, 2.0, 809),
    ),
)
def test_validate_window_layout_rejects_nearby_but_wrong_aligned_width(
    available_width,
    device_pixel_ratio,
    dashboard_width,
):
    """比例近似或落在容差内，也不能替代唯一的 DPR 对齐边界。"""
    available = Rect(0, 0, available_width, 768)
    main_width = available_width - dashboard_width

    with pytest.raises(ValueError, match="33/100"):
        verifier_module.validate_window_layout(
            available,
            Rect(0, 0, main_width, 768),
            Rect(main_width, 0, dashboard_width, 768),
            device_pixel_ratio=device_pixel_ratio,
        )


@pytest.mark.parametrize(
    ("available_width", "dashboard_width"),
    (
        (1366, 273),
        (1366, 410),
        (1366, 492),
        (1366, 420),
    ),
)
def test_validate_window_layout_rejects_legacy_or_fixed_dashboard_widths(
    available_width,
    dashboard_width,
):
    available = Rect(0, 0, available_width, 768)
    main_width = available_width - dashboard_width

    with pytest.raises(ValueError, match="33/100"):
        verifier_module.validate_window_layout(
            available,
            Rect(0, 0, main_width, 768),
            Rect(main_width, 0, dashboard_width, 768),
        )


def test_validate_window_layout_accepts_dpr_aligned_physical_outer_rects():
    available = Rect(112, 64, 2448, 1376)
    layout = align_window_layout_to_scale(
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
        (Rect(10, 21, 801, 699), Rect(811, 20, 200, 700), "vertically"),
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
