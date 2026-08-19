# 企业 Dashboard 单元测试：锁定主页面白名单、接口快照渲染和窄宽布局。
from __future__ import annotations

from dataclasses import dataclass, replace
import json

import pytest

import slope_sim.dashboard as dashboard_module
from slope_sim.dashboard import TelemetryDashboard
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.dashboard_snapshot import (
    InterfaceDashboardSnapshot,
    LidarTopViewFrame,
    LidarTopViewPoint,
)
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPoint,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.interfaces.status import InterfaceStatusSnapshot, TopicStatus, WheelCommandStatus
from slope_sim.telemetry import RobotTelemetry


TOPIC_LABELS = ("轮子命令", "轮子状态", "前雷达", "后雷达", "RTK", "IMU")
EXPECTED_DEFAULT_TABS = [
    "接口状态", "障碍物", "轨迹", "速度/命令",
    "驱动命令", "驱动反馈", "转向命令", "转向反馈", "LiDAR点云",
    "RTK位置", "RTK航向", "IMU姿态", "轮组频率", "传感频率", "接口异常",
]


@dataclass(frozen=True)
class RectLike:
    """模拟并行窗口布局模块产出的只读矩形。"""

    x: int
    y: int
    width: int
    height: int


def _tab_names(dashboard: TelemetryDashboard) -> list[str]:
    return [dashboard.tabs.tabText(index) for index in range(dashboard.tabs.count())]


def _all_widget_text(dashboard: TelemetryDashboard) -> str:
    """收集所有已创建控件文本，隐藏页也不能夹带默认禁用字段。"""
    from PySide6 import QtWidgets

    texts: list[str] = []
    for label in dashboard.window.findChildren(QtWidgets.QLabel):
        texts.append(label.text())
    for button in dashboard.window.findChildren(QtWidgets.QAbstractButton):
        texts.append(button.text())
        texts.append(button.toolTip())
    for group in dashboard.window.findChildren(QtWidgets.QGroupBox):
        texts.append(group.title())
    for combo in dashboard.window.findChildren(QtWidgets.QComboBox):
        texts.extend(combo.itemText(index) for index in range(combo.count()))
    for tabs in dashboard.window.findChildren(QtWidgets.QTabWidget):
        texts.extend(tabs.tabText(index) for index in range(tabs.count()))
    return "\n".join(texts)


def _snapshot(
    config: InterfaceConfig,
    *,
    state: str = "active",
    transport_mode: str | None = None,
    ecal_connected: bool = True,
    actual_hz: float = 10.0,
    timestamp_ns: int | None = 123,
    detail: str = "链路正常",
) -> InterfaceStatusSnapshot:
    topics = {
        channel.topic: TopicStatus(
            topic=channel.topic,
            direction=channel.direction,
            state=state,
            target_hz=float(channel.rate_hz),
            actual_hz=actual_hz,
            latest_timestamp_ns=timestamp_ns,
            message_count=17,
            error_count=2,
            dropped_count=3,
            detail=detail,
        )
        for channel in config.channels
    }
    return InterfaceStatusSnapshot(
        captured_at=1.0,
        transport_mode=transport_mode or config.transport_mode,
        ecal_connected=ecal_connected,
        command=WheelCommandStatus(
            state="active",
            valid_hz=98.5,
            latest_timestamp_ns=122,
            valid_count=16,
            invalid_count=1,
        ),
        wheel_state=WheelState(
            timestamp_ns=123,
            drive_wheel_speed_rad_s=(1.25, -2.5),
            steering_wheel_angle_rad=(0.1, -0.2),
        ),
        topics=topics,
    )


def _dashboard_snapshot(
    config: InterfaceConfig,
    *,
    generation: int = 1,
    timestamp_ns: int = 1_000_000_000,
    captured_at: float = 1.0,
    robot_model: str = "active_steering_4wd",
    points: tuple[tuple[float, float, int, int], ...] = (),
) -> InterfaceDashboardSnapshot:
    """构造包含企业业务消息和可选前后点云的完整组合快照。"""
    status = replace(_snapshot(config), captured_at=captured_at)
    drive_count = 4 if robot_model == "active_steering_4wd" else 2
    steering_count = 2 if robot_model == "active_steering_4wd" else 0
    front_points = tuple(point for point in points if point[3] == 1)
    rear_points = tuple(point for point in points if point[3] == 2)

    def lidar_side(side: str, values: tuple[tuple[float, float, int, int], ...]):
        if not values:
            return None, None
        lidar_id = 1 if side == "front" else 2
        side_time = timestamp_ns if lidar_id == 1 else timestamp_ns + 1_000_000_000
        cloud_points = tuple(
            LidarPoint(index, x, y, 0.0, 1, tag, index % 16)
            for index, (x, y, tag, _lidar_id) in enumerate(values)
        )
        cloud = LidarPointCloud(
            side_time,
            f"lidar_{side}",
            len(cloud_points),
            lidar_id,
            cloud_points,
        )
        view = LidarTopViewFrame(
            side_time,
            tuple(LidarTopViewPoint(x, y, tag, lidar_id) for x, y, tag, _ in values),
        )
        return cloud, view

    front_cloud, front_view = lidar_side("front", front_points)
    rear_cloud, rear_view = lidar_side("rear", rear_points)
    return InterfaceDashboardSnapshot(
        generation=generation,
        robot_model=robot_model,
        sim_time_ns=timestamp_ns,
        status=status,
        wheel_command=WheelCommand(
            99,
            tuple(float(index + 1) for index in range(drive_count)),
            tuple(float(index + 1) / 10.0 for index in range(steering_count)),
        ),
        wheel_command_received_sim_time_ns=timestamp_ns,
        wheel_state=WheelState(
            timestamp_ns,
            tuple(float(index + 5) for index in range(drive_count)),
            tuple(float(index + 1) / 20.0 for index in range(steering_count)),
        ),
        lidar_front=front_cloud,
        lidar_rear=rear_cloud,
        rtk=RtkState(timestamp_ns, 1.0, 2.0, 3.0, 0.4),
        imu=ImuAttitude(timestamp_ns, 0.1, -0.2),
        lidar_front_view=front_view,
        lidar_rear_view=rear_view,
    )


@pytest.fixture
def enterprise_dashboard(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        interface_config=InterfaceConfig.default(),
        model_switch_enabled=True,
        terrain_switch_enabled=True,
    )
    try:
        yield dashboard
    finally:
        dashboard.close()


def test_default_dashboard_contains_only_enterprise_tabs_and_controls(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        assert dashboard.window.windowTitle() == "3D仿真Dashboard"
        assert _tab_names(dashboard) == EXPECTED_DEFAULT_TABS
        assert dashboard.tabs.usesScrollButtons()
        assert dashboard.diagnostic_tabs is None
        assert len(dashboard.window.findChildren(dashboard.QtWidgets.QTabWidget)) == 1
        assert all(dashboard.tabs.isAncestorOf(canvas) for canvas in dashboard.plot_canvases.values())
        visible_text = _all_widget_text(dashboard)
        for required in (
            "仿真控制",
            "线速度",
            "角速度",
            "机器人",
            "场地",
            "障碍物",
            *TOPIC_LABELS,
        ):
            assert required in visible_text
        for forbidden in (
            "摩擦",
            "地形法向",
            "导航",
            "自动避障",
            "相机参数",
            "打滑",
            "接触",
        ):
            assert forbidden not in visible_text
    finally:
        dashboard.close()


def test_dashboard_hides_stage3_lidar_controls_when_interactive_lidar_is_disabled(
    monkeypatch,
):
    """纯手动驾驶不展示无数据的圆形调试点云入口。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(show_lidar_tools=False)
    try:
        assert "LiDAR点云" not in [
            dashboard.tabs.tabText(index) for index in range(dashboard.tabs.count())
        ]
        assert dashboard.lidar_view_checkbox is None
        assert dashboard.lidar_open_button is None
        assert dashboard.lidar_export_button is None
    finally:
        dashboard.close()


def test_layout_report_serializes_current_plot_and_control_rectangles(monkeypatch, tmp_path):
    """v4 验收报告必须覆盖 active axes、图表文字和全部关键控制。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    report_path = tmp_path / "dashboard-layout.jsonl"
    monkeypatch.setenv(dashboard_module.DASHBOARD_LAYOUT_REPORT_ENV, str(report_path))
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        scrollbar = dashboard.control_scroll.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        original_scroll_value = scrollbar.value()
        dashboard.tabs.setCurrentIndex(EXPECTED_DEFAULT_TABS.index("轨迹"))
        dashboard.process_events()
        dashboard.process_events()

        rows = [json.loads(line) for line in report_path.read_text(encoding="utf-8").splitlines()]
        report = rows[-1]
        assert report["report_version"] == 4
        assert report["tab_count"] == 15
        assert report["tab_label"] == "轨迹"
        assert report["tab_order"] == EXPECTED_DEFAULT_TABS
        assert report["page_kind"] == "plot"
        assert report["required_plot_buttons"] == ["清空曲线", "保存当前图"]
        assert type(report["rendered_data_revision"]) is int
        assert report["rendered_data_revision"] >= 0
        assert len(report["tabs_rect"]) == len(report["controls_rect"]) == 4
        assert len(report["tab_bar_rect"]) == 4
        assert set(report["tab_scroll_button_rects"]) == {"left", "right"}
        assert len(report["page_rect"]) == len(report["canvas_rect"]) == 4
        assert len(report["axes_rect"]) == 4
        assert len(report["legend_rect"]) == 4
        assert set(report["plot_button_rects"]) == {"清空曲线", "保存当前图"}
        assert report["content_widget_rects"] == {}
        assert set(report["plot_artist_rects"]) == {
            "title",
            "x_label",
            "y_label",
            "x_offset",
            "y_offset",
        }
        assert report["plot_artist_rects"]["x_offset"] is None
        assert report["plot_artist_rects"]["y_offset"] is None
        assert len(report["control_viewport_rect"]) == 4
        assert len(report["control_content_rect"]) == 4
        assert len(report["control_scroll_range"]) == 2
        assert set(report["critical_control_rects"]) == {
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
        }
        for control in report["critical_control_rects"].values():
            assert set(control) == {"rect", "viewport_rect", "scroll_value"}
            assert len(control["rect"]) == len(control["viewport_rect"]) == 4
        assert scrollbar.value() == original_scroll_value
    finally:
        dashboard.close()


def test_layout_report_waits_for_a_new_rendered_revision_before_reemitting_plot(
    monkeypatch,
    tmp_path,
):
    """切回图表时不能先写新行旧修订；完成新数据绘制后才能追加。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    report_path = tmp_path / "dashboard-layout.jsonl"
    monkeypatch.setenv(dashboard_module.DASHBOARD_LAYOUT_REPORT_ENV, str(report_path))
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        trajectory_index = EXPECTED_DEFAULT_TABS.index("轨迹")
        dashboard.tabs.setCurrentIndex(trajectory_index)
        for _attempt in range(6):
            dashboard.process_events()
        first_rows = [
            json.loads(line)
            for line in report_path.read_text(encoding="utf-8").splitlines()
            if json.loads(line)["tab_label"] == "轨迹"
        ]
        assert len(first_rows) == 1

        dashboard.tabs.setCurrentIndex(EXPECTED_DEFAULT_TABS.index("速度/命令"))
        for _attempt in range(6):
            dashboard.process_events()
        dashboard.tabs.setCurrentIndex(trajectory_index)
        for _attempt in range(3):
            dashboard.process_events()
        unchanged_rows = [
            json.loads(line)
            for line in report_path.read_text(encoding="utf-8").splitlines()
            if json.loads(line)["tab_label"] == "轨迹"
        ]
        assert len(unchanged_rows) == 1

        dashboard._plot_next_draw_time["轨迹"] = 0.0
        dashboard._mark_plot_data_changed(("轨迹",))
        for _attempt in range(6):
            dashboard.process_events()
        final_rows = [
            json.loads(line)
            for line in report_path.read_text(encoding="utf-8").splitlines()
            if json.loads(line)["tab_label"] == "轨迹"
        ]
        assert len(final_rows) == 2
        assert (
            final_rows[-1]["rendered_data_revision"]
            > first_rows[-1]["rendered_data_revision"]
        )
    finally:
        dashboard.close()


def test_trajectory_disables_scientific_offsets_and_keeps_axis_artists_separate(monkeypatch):
    """大坐标轨迹不能让科学计数 offset 与坐标轴标签争用同一位置。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        dashboard.apply_window_rect(RectLike(0, 0, 404, 651))
        dashboard.tabs.setCurrentIndex(EXPECTED_DEFAULT_TABS.index("轨迹"))
        for index in range(3):
            dashboard.plot_buffer.append(
                RobotTelemetry(
                    t=float(index),
                    x=1_000_000.0 + index,
                    y=2_000_000.0 + index,
                )
            )
        dashboard._apply_plot_series("轨迹")
        canvas = dashboard.plot_canvases["轨迹"]
        canvas.draw()

        axis = dashboard.plot_axes["轨迹"]
        renderer = canvas.get_renderer()
        x_label = axis.xaxis.label.get_window_extent(renderer=renderer)
        y_label = axis.yaxis.label.get_window_extent(renderer=renderer)
        x_offset_artist = axis.xaxis.get_offset_text()
        y_offset_artist = axis.yaxis.get_offset_text()

        assert x_offset_artist.get_text() == ""
        assert y_offset_artist.get_text() == ""
        assert not x_label.overlaps(
            x_offset_artist.get_window_extent(renderer=renderer)
        )
        assert not y_label.overlaps(
            y_offset_artist.get_window_extent(renderer=renderer)
        )
    finally:
        dashboard.close()


def test_wall_time_xlabel_stays_separate_from_scientific_offset(monkeypatch):
    """窄画布的大墙钟横轴保留 offset，但长 xlabel 必须与其分居左右。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        dashboard.apply_window_rect(RectLike(0, 0, 404, 651))
        tab_label = "轮组频率"
        dashboard.tabs.setCurrentIndex(EXPECTED_DEFAULT_TABS.index(tab_label))
        dashboard.plot_lines["command_hz"].set_data(
            [17_828.2, 17_828.3, 17_828.4],
            [100.0, 99.0, 101.0],
        )
        axis = dashboard.plot_axes[tab_label]
        axis.relim()
        axis.autoscale_view()
        canvas = dashboard.plot_canvases[tab_label]
        canvas.draw()

        renderer = canvas.get_renderer()
        x_label = axis.xaxis.label.get_window_extent(renderer=renderer)
        x_offset_artist = axis.xaxis.get_offset_text()

        assert x_offset_artist.get_text()
        assert not x_label.overlaps(
            x_offset_artist.get_window_extent(renderer=renderer)
        )
    finally:
        dashboard.close()


def test_plot_axes_prune_outermost_ticks_for_narrow_dashboard(enterprise_dashboard):
    """窄画布必须裁掉两端主刻度，避免完整 tick 文字越出 Figure。"""
    for axis in enterprise_dashboard.plot_axes.values():
        assert axis.xaxis.get_major_locator()._prune == "both"
        assert axis.yaxis.get_major_locator()._prune == "both"


def test_qt_label_text_report_preserves_full_vertical_contents_rect(
    enterprise_dashboard,
):
    """QLabel 合法字体 bbox 可触及上下边，报告不能人为缩短内容区。"""
    from PySide6 import QtWidgets

    label = QtWidgets.QLabel("运行状态", enterprise_dashboard.window)
    label.resize(120, 30)
    label.show()
    enterprise_dashboard.process_events()

    report = enterprise_dashboard._qt_text_global_rect(label)
    contents = label.contentsRect()

    assert report is not None
    assert report["container_rect"][2:] == [contents.width() - 4, contents.height()]


def test_layout_report_fully_contains_all_required_controls_at_real_client_size(monkeypatch):
    """真实 404x651 客户区中，六个障碍物控件也必须逐个完整滚入 viewport。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtCore

    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        dashboard.apply_window_rect(RectLike(0, 0, 404, 651))
        dashboard.process_events()

        report = dashboard._layout_report()

        assert report is not None
        assert {
            "障碍模式",
            "障碍形状",
            "障碍数量",
            "障碍随机种子",
            "障碍速度",
            "障碍移动占比",
        } <= set(report["critical_control_rects"])
        for name, control in report["critical_control_rects"].items():
            viewport = QtCore.QRect(*control["viewport_rect"])
            rect = QtCore.QRect(*control["rect"])
            assert viewport.contains(rect), (name, rect, viewport)
    finally:
        dashboard.close()


def test_all_fifteen_default_layout_reports_pass_content_and_artist_containment(monkeypatch):
    """15 个默认页面都必须由正式 verifier 证明控件、文字 artist 和画布完整包含。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from scripts.verify_dashboard_manual_drive import validate_dashboard_layout_report
    from slope_sim.window_layout import Rect

    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        dashboard.apply_window_rect(RectLike(0, 0, 404, 651))
        for index, label in enumerate(EXPECTED_DEFAULT_TABS):
            dashboard.tabs.setCurrentIndex(index)
            report = None
            for _attempt in range(12):
                dashboard.process_events()
                report = dashboard._layout_report()
                if report is not None:
                    break

            assert report is not None, label
            x, y, width, height = report["window_rect"]
            result = validate_dashboard_layout_report(
                report,
                Rect(x, y, width, height),
            )
            assert result.passed, (label, result.detail)
    finally:
        dashboard.close()


def test_real_dashboard_report_stays_stable_after_new_trajectory_data(monkeypatch):
    """真实 Dashboard 完成新数据绘制后必须推进修订且不改变五个布局矩形。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from scripts.verify_dashboard_manual_drive import validate_dashboard_layout_stability
    from slope_sim.window_layout import Rect

    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        dashboard.apply_window_rect(RectLike(0, 0, 404, 651))
        dashboard.tabs.setCurrentIndex(EXPECTED_DEFAULT_TABS.index("轨迹"))
        for _attempt in range(6):
            dashboard.process_events()
        before = dashboard._layout_report()
        assert before is not None

        next_draw_time = dashboard._plot_next_draw_time["轨迹"] + 1.0
        monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: next_draw_time)
        dashboard._last_plot_update_time = None
        dashboard.update(RobotTelemetry(t=1.0, x=1.0, y=2.0))
        for _attempt in range(6):
            dashboard.process_events()
        after = dashboard._layout_report()
        assert after is not None

        x, y, width, height = before["window_rect"]
        result = validate_dashboard_layout_stability(
            before,
            after,
            Rect(x, y, width, height),
        )
        assert after["rendered_data_revision"] > before["rendered_data_revision"]
        assert result.passed, result.detail
    finally:
        dashboard.close()


def test_developer_diagnostics_tab_is_opt_in(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    normal = TelemetryDashboard(developer_diagnostics_enabled=False)
    diagnostic = TelemetryDashboard(developer_diagnostics_enabled=True)
    try:
        assert _tab_names(normal) == EXPECTED_DEFAULT_TABS
        assert _tab_names(diagnostic) == [*EXPECTED_DEFAULT_TABS, "开发者诊断"]
        diagnostic_text = _all_widget_text(diagnostic)
        assert "打滑严重度" in diagnostic_text
        assert "有效接触点" in diagnostic_text
        assert "接触法向力" in diagnostic_text
        expected_plot_tabs = set(EXPECTED_DEFAULT_TABS) - {"接口状态", "障碍物"}
        assert normal.tabs.count() == 15
        assert diagnostic.tabs.count() == 16
        assert len(normal.plot_specs) == len(diagnostic.plot_specs) == 12
        assert set(normal.plot_canvases) == expected_plot_tabs
        assert set(diagnostic.plot_canvases) == expected_plot_tabs
        assert len(normal.plot_canvases) == len(diagnostic.plot_canvases) == 13
        assert normal.diagnostic_tabs is diagnostic.diagnostic_tabs is None
        assert len(diagnostic.window.findChildren(diagnostic.QtWidgets.QTabWidget)) == 1
        assert all(diagnostic.tabs.isAncestorOf(canvas) for canvas in diagnostic.plot_canvases.values())
    finally:
        normal.close()
        diagnostic.close()


def test_plot_buttons_use_icons_tooltips_and_keep_callbacks(monkeypatch):
    """一级折线页保留清空、保存命令及标准图标。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(developer_diagnostics_enabled=True)
    saved_tabs: list[str] = []
    dashboard.save_plot_snapshot = lambda tab_label: saved_tabs.append(tab_label)
    try:
        for index in range(dashboard.tabs.count()):
            tab_label = dashboard.tabs.tabText(index)
            if tab_label not in dashboard.plot_canvases:
                continue
            dashboard.tabs.setCurrentIndex(index)
            buttons = dashboard.tabs.widget(index).findChildren(
                dashboard.QtWidgets.QPushButton
            )

            expected = ["保存当前图"] if tab_label == "LiDAR点云" else ["清空曲线", "保存当前图"]
            assert [button.text() for button in buttons] == expected
            assert all(not button.icon().isNull() for button in buttons)
            assert all(button.toolTip() for button in buttons)

            if tab_label == "LiDAR点云":
                buttons[0].click()
                assert saved_tabs[-1] == tab_label
                continue
            dashboard.plot_buffer.append(RobotTelemetry(t=float(index)))
            buttons[0].click()
            assert dashboard.plot_buffer.series()["t"] == []
            buttons[1].click()
            assert saved_tabs[-1] == tab_label
    finally:
        dashboard.close()


def test_hidden_interface_tabs_buffer_without_drawing_then_draw_only_selected(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    now = {"value": 100.0}
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: now["value"])
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        draw_counts = {label: 0 for label in dashboard.plot_canvases}
        for label, canvas in dashboard.plot_canvases.items():
            canvas.draw_idle = lambda label=label: draw_counts.__setitem__(label, draw_counts[label] + 1)
            canvas._draw_pending = False
        dashboard.tabs.setCurrentIndex(0)

        dashboard.update_interface_snapshot(_dashboard_snapshot(dashboard.interface_config))

        assert dashboard.interface_plot_buffer.series("RTK位置")["t"] == [1.0]
        assert all(count == 0 for count in draw_counts.values())

        rtk_index = EXPECTED_DEFAULT_TABS.index("RTK位置")
        dashboard.tabs.setCurrentIndex(rtk_index)
        assert draw_counts["RTK位置"] >= 1
        assert all(count == 0 for label, count in draw_counts.items() if label != "RTK位置")

        first_draw_count = draw_counts["RTK位置"]
        now["value"] = 100.2
        dashboard.tabs.setCurrentIndex(EXPECTED_DEFAULT_TABS.index("RTK航向"))
        dashboard.tabs.setCurrentIndex(rtk_index)
        assert draw_counts["RTK位置"] == first_draw_count

        now["value"] = 100.5
        dashboard.update_interface_snapshot(
            _dashboard_snapshot(dashboard.interface_config, timestamp_ns=2_000_000_000, captured_at=2.0)
        )
        assert draw_counts["RTK位置"] == first_draw_count + 1
    finally:
        dashboard.close()


def test_same_generation_reuses_all_plot_artists_and_lidar_latest_success(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    now = {"value": 100.0}
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: now["value"])
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        lidar_index = EXPECTED_DEFAULT_TABS.index("LiDAR点云")
        dashboard.tabs.setCurrentIndex(lidar_index)
        dashboard.process_events()
        for canvas in dashboard.plot_canvases.values():
            canvas.draw_idle = lambda: None
            canvas._draw_pending = False
        first = _dashboard_snapshot(
            dashboard.interface_config,
            points=((1.0, 2.0, 0, 1), (2.0, 3.0, 1, 1), (3.0, 4.0, 2, 2), (4.0, 5.0, 3, 2)),
        )
        now["value"] = 100.5
        dashboard.update_interface_snapshot(first)
        identities = {
            "figures": {key: id(value) for key, value in dashboard.plot_figures.items()},
            "axes": {key: id(value) for key, value in dashboard.plot_axes.items()},
            "canvases": {key: id(value) for key, value in dashboard.plot_canvases.items()},
            "lines": {key: id(value) for key, value in dashboard.plot_lines.items()},
            "legends": {key: id(value) for key, value in dashboard.plot_legends.items()},
            "texts": tuple(id(value) for value in dashboard.plot_texts.values()),
            "collection": id(dashboard.lidar_collection),
        }

        second = _dashboard_snapshot(
            dashboard.interface_config,
            timestamp_ns=3_000_000_000,
            captured_at=3.0,
            points=((8.0, 9.0, 3, 2),),
        )
        now["value"] = 101.0
        dashboard.update_interface_snapshot(second)

        assert identities["figures"] == {key: id(value) for key, value in dashboard.plot_figures.items()}
        assert identities["axes"] == {key: id(value) for key, value in dashboard.plot_axes.items()}
        assert identities["canvases"] == {key: id(value) for key, value in dashboard.plot_canvases.items()}
        assert identities["lines"] == {key: id(value) for key, value in dashboard.plot_lines.items()}
        assert identities["legends"] == {key: id(value) for key, value in dashboard.plot_legends.items()}
        assert identities["texts"] == tuple(id(value) for value in dashboard.plot_texts.values())
        assert identities["collection"] == id(dashboard.lidar_collection)
        assert dashboard.lidar_collection.get_offsets().tolist() == [
            [1.0, 2.0],
            [2.0, 3.0],
            [8.0, 9.0],
        ]
        assert len({tuple(color) for color in dashboard.lidar_collection.get_facecolors()}) == 3
        assert "1.000" in dashboard.lidar_front_time_text.get_text()
        assert "4.000" in dashboard.lidar_rear_time_text.get_text()

        dashboard.update_interface_snapshot(
            replace(second, lidar_rear=None, lidar_rear_view=None, status=replace(second.status, captured_at=4.0))
        )
        assert dashboard.lidar_collection.get_offsets().tolist() == [
            [1.0, 2.0],
            [2.0, 3.0],
            [8.0, 9.0],
        ]
        dashboard.reset_feedback_history()
        assert dashboard.lidar_collection.get_offsets().shape == (0, 2)
        with pytest.raises(ValueError, match="at least one LiDAR point cloud"):
            dashboard.export_current_lidar_point_clouds()
    finally:
        dashboard.close()


def test_lidar_viewport_and_artist_geometry_stay_fixed_across_frames(monkeypatch):
    """点云内容只能更新 artist，不能改变固定车体坐标窗口或绘图区几何。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    now = {"value": 100.0}
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: now["value"])
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        dashboard.tabs.setCurrentIndex(EXPECTED_DEFAULT_TABS.index("LiDAR点云"))
        dashboard.process_events()
        canvas = dashboard.plot_canvases["LiDAR点云"]
        canvas._draw_pending = False
        canvas.draw()
        initial_canvas_size = canvas.get_width_height()
        initial_axis_bounds = dashboard.plot_axes["LiDAR点云"].get_window_extent(
            canvas.get_renderer()
        ).bounds
        identities = (
            id(dashboard.plot_axes["LiDAR点云"]),
            id(dashboard.lidar_collection),
            id(dashboard.lidar_front_time_text),
            id(dashboard.lidar_rear_time_text),
        )

        now["value"] = 100.5
        dashboard.update_interface_snapshot(
            _dashboard_snapshot(
                dashboard.interface_config,
                generation=1,
                timestamp_ns=1_000_000_000,
                points=((30.0, 30.0, 1, 1),),
            )
        )
        canvas.draw()
        assert dashboard.plot_axes["LiDAR点云"].get_xlim() == pytest.approx((-48.0, 48.0))
        assert dashboard.plot_axes["LiDAR点云"].get_ylim() == pytest.approx((-30.0, 30.0))

        canvas._draw_pending = False
        now["value"] = 101.0
        dashboard.update_interface_snapshot(
            _dashboard_snapshot(
                dashboard.interface_config,
                generation=1,
                timestamp_ns=2_000_000_000,
                captured_at=2.0,
                points=((1.0, 1.0, 2, 1),),
            )
        )
        canvas.draw()
        assert dashboard.plot_axes["LiDAR点云"].get_xlim() == pytest.approx((-48.0, 48.0))
        assert dashboard.plot_axes["LiDAR点云"].get_ylim() == pytest.approx((-30.0, 30.0))

        canvas._draw_pending = False
        now["value"] = 101.5
        empty_snapshot = _dashboard_snapshot(
            dashboard.interface_config,
            generation=2,
            timestamp_ns=3_000_000_000,
            captured_at=3.0,
            points=(),
        )
        dashboard.update_interface_snapshot(
            replace(
                empty_snapshot,
                lidar_front=LidarPointCloud(
                    3_000_000_000,
                    "lidar_front",
                    0,
                    1,
                    (),
                ),
                lidar_front_view=LidarTopViewFrame(3_000_000_000, ()),
            )
        )
        canvas.draw()
        assert dashboard.plot_axes["LiDAR点云"].get_xlim() == pytest.approx((-48.0, 48.0))
        assert dashboard.plot_axes["LiDAR点云"].get_ylim() == pytest.approx((-30.0, 30.0))

        canvas._draw_pending = False
        now["value"] = 102.0
        dashboard.update_interface_snapshot(
            _dashboard_snapshot(
                dashboard.interface_config,
                generation=2,
                timestamp_ns=4_000_000_000,
                captured_at=4.0,
                points=((2.0, -2.0, 3, 1),),
            )
        )
        canvas.draw()
        dashboard.reset_feedback_history()
        canvas.draw()
        assert dashboard.plot_axes["LiDAR点云"].get_xlim() == pytest.approx((-48.0, 48.0))
        assert dashboard.plot_axes["LiDAR点云"].get_ylim() == pytest.approx((-30.0, 30.0))
        assert canvas.get_width_height() == initial_canvas_size
        final_axis_bounds = dashboard.plot_axes["LiDAR点云"].get_window_extent(
            canvas.get_renderer()
        ).bounds
        assert final_axis_bounds == pytest.approx(initial_axis_bounds, abs=1.0)
        assert identities == (
            id(dashboard.plot_axes["LiDAR点云"]),
            id(dashboard.lidar_collection),
            id(dashboard.lidar_front_time_text),
            id(dashboard.lidar_rear_time_text),
        )
    finally:
        dashboard.close()


def test_lidar_plot_explains_vehicle_frame_and_obstacle_classes(monkeypatch):
    """LiDAR 页应直接标明车体方向，并用固定图例解释障碍点颜色。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        axis = dashboard.plot_axes["LiDAR点云"]
        legend = dashboard.plot_legends["LiDAR点云"]

        assert axis.get_xlabel() == "forward x [m]"
        assert axis.get_ylabel() == "left y [m]"
        assert list(dashboard.lidar_vehicle_marker.get_xdata()) == [0.0]
        assert list(dashboard.lidar_vehicle_marker.get_ydata()) == [0.0]
        assert dashboard.lidar_vehicle_marker.get_marker() == ">"
        assert [text.get_text() for text in legend.get_texts()] == [
            "unknown",
            "terrain",
            "static obstacle",
            "moving obstacle",
            "vehicle forward",
        ]
        assert legend._ncols == 2
        assert legend.columnspacing == pytest.approx(1.0)
        assert legend.handletextpad == pytest.approx(0.6)
    finally:
        dashboard.close()


def test_lidar_tag_colors_and_no_steering_status(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default(), robot_model="df_back")
    try:
        dashboard.process_events()
        for canvas in dashboard.plot_canvases.values():
            canvas.draw_idle = lambda: None
            canvas._draw_pending = False
        dashboard.update_interface_snapshot(
            _dashboard_snapshot(
                dashboard.interface_config,
                robot_model="df_back",
                points=((1.0, 1.0, 0, 1), (2.0, 2.0, 1, 1), (3.0, 3.0, 2, 2), (4.0, 4.0, 3, 2)),
            )
        )
        dashboard.tabs.setCurrentIndex(EXPECTED_DEFAULT_TABS.index("LiDAR点云"))
        assert dashboard.lidar_collection.get_offsets().shape == (4, 2)
        assert len({tuple(color) for color in dashboard.lidar_collection.get_facecolors()}) == 4

        dashboard.tabs.setCurrentIndex(EXPECTED_DEFAULT_TABS.index("转向命令"))
        dashboard.process_events()
        assert dashboard.no_steering_texts["转向命令"].get_text() == "当前车型无转向数据"
        assert all(len(line.get_xdata()) == 0 for key, line in dashboard.plot_lines.items() if key.startswith("steering_command_"))
    finally:
        dashboard.close()


def test_dashboard_exposes_and_exports_the_latest_lidar_frames(monkeypatch, tmp_path):
    """点云页必须易发现，并仅在用户点击时导出当前前后帧。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        interface_config=InterfaceConfig.default(),
        plot_snapshot_dir=tmp_path / "figures",
    )
    try:
        assert dashboard.lidar_open_button.text() == "打开实时点云"
        dashboard.lidar_open_button.click()
        assert dashboard.tabs.tabText(dashboard.tabs.currentIndex()) == "LiDAR点云"
        assert dashboard.lidar_view_checkbox.isChecked()
        dashboard.update_interface_snapshot(
            _dashboard_snapshot(
                dashboard.interface_config,
                points=((1.0, 2.0, 1, 1), (3.0, 4.0, 2, 2)),
            )
        )

        paths = dashboard.export_current_lidar_point_clouds()

        assert tuple(path.name for path in paths) == (
            "lidar_front_1000000000.pcd",
            "lidar_front_1000000000.ply",
            "lidar_rear_2000000000.pcd",
            "lidar_rear_2000000000.ply",
        )
        assert all(path.parent == tmp_path / "pointcloud" for path in paths)
        assert "已导出" in dashboard.lidar_status_label.text()
        dashboard.lidar_view_checkbox.setChecked(False)
        dashboard._apply_lidar_series()
        assert dashboard.lidar_collection.get_offsets().shape == (0, 2)
    finally:
        dashboard.close()


def test_same_generation_model_change_rebuilds_wheel_artists_and_legend(monkeypatch):
    """异常同代车型变化也必须清历史并替换真实轮组 artist。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default(), robot_model="df_back")
    try:
        first = _dashboard_snapshot(
            dashboard.interface_config,
            generation=1,
            timestamp_ns=1_000_000_000,
            captured_at=1.0,
            robot_model="df_back",
        )
        dashboard.update_interface_snapshot(first)
        old_drive_lines = tuple(dashboard.plot_axes["驱动命令"].lines)
        old_drive_legend = dashboard.plot_legends["驱动命令"]

        replacement = _dashboard_snapshot(
            dashboard.interface_config,
            generation=1,
            timestamp_ns=2_000_000_000,
            captured_at=2.0,
            robot_model="active_steering_4wd",
        )
        dashboard.update_interface_snapshot(replacement)

        assert dashboard._interface_robot_model == "active_steering_4wd"
        assert dashboard.interface_plot_buffer.series("驱动命令")["t"] == [2.0]
        assert len(dashboard.plot_axes["驱动命令"].lines) == 4
        assert len(dashboard.plot_axes["转向命令"].lines) == 2
        assert all(line not in dashboard.plot_axes["驱动命令"].lines for line in old_drive_lines)
        assert dashboard.plot_legends["驱动命令"] is not old_drive_legend
        assert dashboard.no_steering_texts["转向命令"].get_text() == ""
    finally:
        dashboard.close()


@pytest.mark.parametrize("paused", (False, True))
def test_interface_snapshot_throttles_qt_status_writes_without_dropping_data(
    monkeypatch,
    paused,
):
    """240Hz 快照全部进入缓存，状态 QLabel 只按 Dashboard 刷新频率写入。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    now = {"value": 100.0}
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: now["value"])
    dashboard = TelemetryDashboard(
        interface_config=InterfaceConfig.default(),
        robot_model="active_steering_4wd",
        update_hz=5.0,
    )
    dashboard._paused = paused
    writes = {"value": 0}
    original_set_text = dashboard.transport_mode_label.setText

    def counted_set_text(value):
        writes["value"] += 1
        original_set_text(value)

    monkeypatch.setattr(dashboard.transport_mode_label, "setText", counted_set_text)
    latest = None
    try:
        for index in range(240):
            now["value"] = 100.0 + index / 240.0
            timestamp_ns = 1_000_000_000 + index * 4_166_667
            latest = _dashboard_snapshot(
                dashboard.interface_config,
                generation=1,
                timestamp_ns=timestamp_ns,
                captured_at=index / 240.0,
                robot_model="active_steering_4wd",
            )
            dashboard.update_interface_snapshot(latest)

        assert 1 <= writes["value"] <= 6
        assert dashboard._latest_interface_snapshot is latest
        assert dashboard._interface_status is latest.status
        if paused:
            assert len(dashboard.interface_plot_buffer.series("轮组频率")["t"]) == 240
            assert dashboard.interface_plot_buffer.series("驱动命令")["t"] == []
        else:
            assert len(dashboard.interface_plot_buffer.series("驱动命令")["t"]) == 240
    finally:
        dashboard.close()


def test_reset_feedback_history_invalidates_latest_status_and_next_generation_renders(
    monkeypatch,
):
    """结构重建必须原子清 latest/status/门禁，新代首帧不能受旧节流时间阻塞。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    now = {"value": 100.0}
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: now["value"])
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default(), update_hz=5.0)
    writes = {"value": 0}
    original_set_text = dashboard.transport_mode_label.setText

    def counted_set_text(value):
        writes["value"] += 1
        original_set_text(value)

    monkeypatch.setattr(dashboard.transport_mode_label, "setText", counted_set_text)
    try:
        dashboard.update_interface_snapshot(
            _dashboard_snapshot(
                dashboard.interface_config,
                generation=1,
                timestamp_ns=1_000_000_000,
                captured_at=1.0,
                robot_model="df_back",
            )
        )
        assert writes["value"] == 1

        dashboard.reset_feedback_history()

        assert dashboard._latest_interface_snapshot is None
        assert dashboard._interface_status is None
        assert dashboard._last_interface_status_update_time is None

        now["value"] = 100.01
        dashboard.update_interface_snapshot(
            _dashboard_snapshot(
                dashboard.interface_config,
                generation=2,
                timestamp_ns=2_000_000_000,
                captured_at=2.0,
                robot_model="df_back",
            )
        )
        assert writes["value"] == 2
    finally:
        dashboard.close()


def test_constructor_failure_before_completion_does_not_install_global_filter(monkeypatch):
    """控件构造中途失败时，不能把半构造 Dashboard 挂到全局事件链。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    original_install = QtWidgets.QApplication.installEventFilter
    installed_filters: list[object] = []
    existing_widget_ids = {id(widget) for widget in app.topLevelWidgets()}

    def track_install(application, event_filter):
        if type(event_filter).__name__ == "DashboardKeyEventFilter":
            installed_filters.append(event_filter)
        return original_install(application, event_filter)

    def fail_plot_construction(*_args, **_kwargs):
        raise RuntimeError("injected plot construction failure")

    monkeypatch.setattr(QtWidgets.QApplication, "installEventFilter", track_install)
    monkeypatch.setattr(TelemetryDashboard, "_add_plot_tabs", fail_plot_construction)
    with pytest.raises(RuntimeError, match="injected plot construction failure"):
        TelemetryDashboard(developer_diagnostics_enabled=True)
    app.processEvents()

    assert installed_filters == []
    assert not any(
        id(widget) not in existing_widget_ids
        and widget.windowTitle() == dashboard_module.DASHBOARD_WINDOW_TITLE
        for widget in app.topLevelWidgets()
    )


def test_constructor_failure_after_canvas_creation_disposes_mpl_and_qt(monkeypatch):
    """canvas 创建后的构造异常必须断开回调、释放 Figure 并删除顶层窗口。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
    from PySide6 import QtWidgets
    import shiboken6

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    existing_widget_ids = {id(widget) for widget in app.topLevelWidgets()}
    original_connect = FigureCanvasQTAgg.mpl_connect
    connections: list[tuple[object, str, int, object, object]] = []

    def track_connect(canvas, event_name, callback):
        cid = original_connect(canvas, event_name, callback)
        connections.append(
            (canvas, event_name, cid, canvas.figure, canvas.callbacks)
        )
        return cid

    def fail_control_construction(*_args, **_kwargs):
        raise RuntimeError("injected control construction failure")

    monkeypatch.setattr(FigureCanvasQTAgg, "mpl_connect", track_connect)
    monkeypatch.setattr(
        TelemetryDashboard,
        "_add_enterprise_control_area",
        fail_control_construction,
    )

    with pytest.raises(RuntimeError, match="injected control construction failure"):
        TelemetryDashboard()
    app.processEvents()

    assert len(connections) == 13
    assert all(
        cid not in registry.callbacks.get(event_name, {})
        for _canvas, event_name, cid, _figure, registry in connections
    )
    assert all(
        figure.axes == []
        for _canvas, _event, _cid, figure, _registry in connections
    )
    assert all(
        not shiboken6.isValid(canvas)
        for canvas, _event, _cid, _figure, _registry in connections
    )
    assert not any(
        id(widget) not in existing_widget_ids
        and widget.windowTitle() == dashboard_module.DASHBOARD_WINDOW_TITLE
        for widget in app.topLevelWidgets()
    )


def test_constructor_failure_before_canvas_registration_disposes_orphan_canvas(monkeypatch):
    """FigureCanvas 创建后若 axis 构造失败，也不能留下无 parent 的顶层控件。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
    from matplotlib.figure import Figure
    from PySide6 import QtWidgets
    import shiboken6

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    original_canvas_init = FigureCanvasQTAgg.__init__
    canvases = []

    def track_canvas_init(canvas, *args, **kwargs):
        original_canvas_init(canvas, *args, **kwargs)
        canvases.append(canvas)

    def fail_subplots(*_args, **_kwargs):
        raise RuntimeError("injected subplots failure")

    monkeypatch.setattr(FigureCanvasQTAgg, "__init__", track_canvas_init)
    monkeypatch.setattr(Figure, "subplots", fail_subplots)

    with pytest.raises(RuntimeError, match="injected subplots failure"):
        TelemetryDashboard()
    app.processEvents()

    assert len(canvases) == 1
    assert not shiboken6.isValid(canvases[0])


def test_final_window_setup_failure_removes_filter_and_closes_window(monkeypatch):
    """最终显示阶段失败时，必须撤销全局 filter 并关闭已显示窗口。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    original_install = QtWidgets.QApplication.installEventFilter
    original_remove = QtWidgets.QApplication.removeEventFilter
    original_activate = QtWidgets.QWidget.activateWindow
    original_close = QtWidgets.QWidget.close
    installed_filters: list[object] = []
    removed_filters: list[object] = []
    closed_windows: list[object] = []
    existing_widget_ids = {id(widget) for widget in app.topLevelWidgets()}

    def track_install(application, event_filter):
        if type(event_filter).__name__ == "DashboardKeyEventFilter":
            installed_filters.append(event_filter)
        return original_install(application, event_filter)

    def track_remove(application, event_filter):
        if type(event_filter).__name__ == "DashboardKeyEventFilter":
            removed_filters.append(event_filter)
        return original_remove(application, event_filter)

    def fail_dashboard_activation(widget):
        if widget.windowTitle() == dashboard_module.DASHBOARD_WINDOW_TITLE:
            raise RuntimeError("injected activation failure")
        return original_activate(widget)

    def track_close(widget):
        if widget.windowTitle() == dashboard_module.DASHBOARD_WINDOW_TITLE:
            closed_windows.append(widget)
        return original_close(widget)

    monkeypatch.setattr(QtWidgets.QApplication, "installEventFilter", track_install)
    monkeypatch.setattr(QtWidgets.QApplication, "removeEventFilter", track_remove)
    monkeypatch.setattr(QtWidgets.QWidget, "activateWindow", fail_dashboard_activation)
    monkeypatch.setattr(QtWidgets.QWidget, "close", track_close)
    with pytest.raises(RuntimeError, match="injected activation failure"):
        TelemetryDashboard()
    app.processEvents()

    assert len(installed_filters) == 1
    assert removed_filters == installed_filters
    assert len(closed_windows) == 1
    assert not any(
        id(widget) not in existing_widget_ids
        and widget.windowTitle() == dashboard_module.DASHBOARD_WINDOW_TITLE
        for widget in app.topLevelWidgets()
    )


def test_close_is_idempotent_and_disposes_all_mpl_and_qt_resources(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets
    import shiboken6

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    existing_widget_ids = {id(widget) for widget in app.topLevelWidgets()}
    dashboard = TelemetryDashboard()
    window = dashboard.window
    connections = list(dashboard._mpl_connections)
    callback_registries = [
        (canvas.callbacks, event_name, cid)
        for canvas, event_name, cid in connections
    ]
    figures = list(dashboard.plot_figures.values())

    dashboard.close()
    dashboard.close()

    assert dashboard._key_event_filter_installed is False
    assert all(
        cid not in registry.callbacks.get(event_name, {})
        for registry, event_name, cid in callback_registries
    )
    assert all(figure.axes == [] for figure in figures)
    assert all(not shiboken6.isValid(canvas) for canvas, _event, _cid in connections)
    assert not shiboken6.isValid(window)
    assert dashboard.lidar_collection is None
    assert dashboard.lidar_front_time_text is None
    assert dashboard.lidar_rear_time_text is None
    assert dashboard.no_steering_texts == {}
    assert dashboard.plot_buttons == []
    assert dashboard._interface_line_keys == set()
    assert not any(
        id(widget) not in existing_widget_ids
        and widget.windowTitle() == dashboard_module.DASHBOARD_WINDOW_TITLE
        for widget in app.topLevelWidgets()
    )


def test_default_title_and_command_buttons_use_icons_and_tooltips(enterprise_dashboard):
    dashboard = enterprise_dashboard
    assert dashboard.window.windowTitle() == dashboard_module.DASHBOARD_WINDOW_TITLE == "3D仿真Dashboard"

    icon_command_buttons = (
        dashboard.pause_button,
        dashboard.reset_button,
        dashboard.quit_button,
        dashboard.apply_robot_button,
        dashboard.apply_terrain_button,
        dashboard.add_obstacles_button,
        dashboard.delete_obstacle_button,
        dashboard.clear_obstacles_button,
    )
    assert all(button.toolTip() for button in icon_command_buttons)
    assert all(not button.icon().isNull() for button in icon_command_buttons)


def test_default_dashboard_has_no_direction_buttons(
    enterprise_dashboard,
):
    """方向驾驶统一使用键盘，默认 Dashboard 不再显示遮挡控件的按钮。"""
    dashboard = enterprise_dashboard

    assert dashboard.direction_buttons == []
    assert not {
        button.text()
        for button in dashboard.window.findChildren(dashboard.QtWidgets.QPushButton)
    } & {"↑", "←", "■", "→", "↓"}


def test_pause_button_exposes_read_only_state_and_persists_in_commands(enterprise_dashboard):
    dashboard = enterprise_dashboard
    assert dashboard.paused is False
    assert dashboard.pause_status_label.text() == "运行中"

    dashboard.pause_button.click()

    assert dashboard.paused is True
    assert dashboard.pause_status_label.text() == "已暂停"
    assert dashboard.pause_button.text() == "继续"
    assert dashboard.current_command().paused is True
    with pytest.raises(AttributeError):
        dashboard.paused = False

    dashboard.pause_button.click()
    assert dashboard.paused is False
    assert dashboard.current_command().paused is False


def test_space_key_keeps_existing_hold_to_pause_semantics(enterprise_dashboard):
    dashboard = enterprise_dashboard
    space = dashboard._normalize_key(dashboard.QtCore.Qt.Key_Space)
    dashboard._pressed_keys.add(space)
    assert dashboard.current_command().paused is True

    dashboard._pressed_keys.clear()
    assert dashboard.current_command().paused is False


def test_ctrl_tab_cycles_top_level_pages_from_window_focus(enterprise_dashboard):
    """验收和人工操作都应能在顶层窗口焦点下用 Ctrl+Tab 切换一级页。"""
    dashboard = enterprise_dashboard
    dashboard.tabs.setCurrentIndex(0)
    dashboard.window.setFocus()
    event = dashboard.QtGui.QKeyEvent(
        dashboard.QtCore.QEvent.KeyPress,
        dashboard.QtCore.Qt.Key_Tab,
        dashboard.QtCore.Qt.ControlModifier,
    )

    dashboard.app.sendEvent(dashboard.window, event)

    assert dashboard.tabs.currentIndex() == 1


def test_local_snapshot_explicitly_displays_ecal_disconnected(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    config = InterfaceConfig.default(transport_mode="local")
    dashboard = TelemetryDashboard(interface_config=config)
    try:
        dashboard.update_interface_status(
            _snapshot(config, transport_mode="local", ecal_connected=False, detail="")
        )

        assert dashboard.transport_mode_label.text() == "本地测试模式"
        assert dashboard.ecal_status_label.text() == "eCAL 未连接"
    finally:
        dashboard.close()


def test_auto_fallback_detail_remains_visible(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    config = InterfaceConfig.default(transport_mode="auto")
    dashboard = TelemetryDashboard(interface_config=config)
    try:
        dashboard.update_interface_status(
            _snapshot(
                config,
                transport_mode="local",
                ecal_connected=False,
                detail="EcalUnavailableError: import failed",
            )
        )

        assert dashboard.transport_mode_label.text() == "本地测试模式"
        assert "EcalUnavailableError: import failed" in dashboard.transport_detail_label.text()
    finally:
        dashboard.close()


@pytest.mark.parametrize(
    ("state", "expected"),
    (
        ("active", "活动"),
        ("waiting_peer", "等待对端"),
        ("timed_out", "超时"),
        ("degraded", "降级"),
        ("disconnected", "未连接"),
        ("error", "错误"),
    ),
)
def test_dashboard_maps_every_interface_state(state, expected, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    config = InterfaceConfig.default()
    dashboard = TelemetryDashboard(interface_config=config)
    try:
        dashboard.update_interface_status(_snapshot(config, state=state))

        assert all(row.state_label.text() == expected for row in dashboard.interface_rows.values())
        assert all(row.actual_label.text() == "10.0 Hz" for row in dashboard.interface_rows.values())
        assert all(row.timestamp_label.text() == "123" for row in dashboard.interface_rows.values())
    finally:
        dashboard.close()


@pytest.mark.parametrize(
    ("state", "expected"),
    (("waiting_command", "等待命令"), ("invalid_command", "命令无效")),
)
def test_wheel_command_compatibility_states_are_localized(state, expected, monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    config = InterfaceConfig.default()
    dashboard = TelemetryDashboard(interface_config=config)
    try:
        snapshot = _snapshot(config)
        dashboard.update_interface_status(
            replace(snapshot, command=replace(snapshot.command, state=state))
        )

        command_row = dashboard.interface_rows[config.wheel_command.topic]
        assert command_row.command_state_label.text() == expected
    finally:
        dashboard.close()


def test_interface_rows_render_all_status_fields_and_wheel_details(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    config = InterfaceConfig.default()
    dashboard = TelemetryDashboard(interface_config=config)
    try:
        dashboard.update_interface_status(_snapshot(config))

        for channel in config.channels:
            row = dashboard.interface_rows[channel.topic]
            assert row.target_label.text() == f"{channel.rate_hz:.1f} Hz"
            assert row.actual_label.text() == "10.0 Hz"
            assert row.timestamp_label.text() == "123"
            assert row.error_label.text() == "2"
            assert row.drop_label.text() == "3"
            assert row.detail_label.text() == "链路正常"

        command_row = dashboard.interface_rows[config.wheel_command.topic]
        assert command_row.command_state_label.text() == "活动"
        assert command_row.command_frequency_label.text() == "98.5 Hz"
        assert command_row.command_timestamp_label.text() == "122"

        wheel_row = dashboard.interface_rows[config.wheel_state.topic]
        assert wheel_row.drive_values_label.text() == "[1.25, -2.50] rad/s"
        assert wheel_row.steering_values_label.text() == "[0.10, -0.20] rad"
    finally:
        dashboard.close()


def test_missing_topic_renders_error_without_crashing(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    config = InterfaceConfig.default()
    dashboard = TelemetryDashboard(interface_config=config)
    try:
        snapshot = _snapshot(config)
        topics = dict(snapshot.topics)
        topics.pop(config.imu.topic)

        dashboard.update_interface_status(replace(snapshot, topics=topics))

        missing_row = dashboard.interface_rows[config.imu.topic]
        assert missing_row.state_label.text() == "错误"
        assert config.imu.topic in missing_row.detail_label.text()
    finally:
        dashboard.close()


def test_status_rendering_does_not_call_ecal_or_pybullet(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    import pybullet
    from slope_sim.interfaces import ecal_transport

    def forbidden_call(*_args, **_kwargs):
        raise AssertionError("Dashboard status rendering crossed the snapshot boundary")

    monkeypatch.setattr(pybullet, "getBasePositionAndOrientation", forbidden_call)
    monkeypatch.setattr(ecal_transport, "create_transport", forbidden_call)
    config = InterfaceConfig.default(transport_mode="local")
    dashboard = TelemetryDashboard(interface_config=config)
    try:
        dashboard.update_interface_status(
            _snapshot(config, transport_mode="local", ecal_connected=False)
        )
    finally:
        dashboard.close()


def test_apply_window_rect_uses_rect_attributes_and_fixes_client_without_window_manager(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard()
    try:
        dashboard.apply_window_rect(RectLike(17, 23, 273, 768))
        dashboard.process_events()

        assert (dashboard.window.x(), dashboard.window.y()) == (17, 23)
        assert (dashboard.window.width(), dashboard.window.height()) == (273, 768)
        assert dashboard.window.minimumSize() == dashboard.window.maximumSize()
        assert (
            dashboard.window.minimumWidth(),
            dashboard.window.minimumHeight(),
        ) == (273, 768)
    finally:
        dashboard.close()


def test_dashboard_frame_extents_wait_for_two_stable_managed_samples():
    samples = iter(
        (
            dashboard_module.FrameExtents(0, 0, 0, 0),
            dashboard_module.FrameExtents(0, 0, 37, 0),
            dashboard_module.FrameExtents(0, 0, 37, 0),
        )
    )
    events = []
    now = 0.0

    def clock():
        return now

    def sleep(duration):
        nonlocal now
        events.append(("sleep", duration))
        now += duration

    actual = dashboard_module.wait_for_dashboard_frame_extents(
        frame_extents_getter=lambda: next(samples),
        process_events=lambda: events.append("events"),
        window_manager_expected=True,
        timeout_sec=0.5,
        poll_interval_sec=0.05,
        clock=clock,
        sleeper=sleep,
    )

    assert actual == dashboard_module.FrameExtents(0, 0, 37, 0)
    assert events == [
        "events",
        ("sleep", 0.05),
        "events",
        ("sleep", 0.05),
        "events",
    ]


@pytest.mark.parametrize(
    ("screen_width", "height"),
    ((1366, 768), (1920, 1080), (2560, 1440)),
)
def test_enterprise_controls_are_fully_reachable_at_thirty_three_percent_width(
    screen_width,
    height,
    enterprise_dashboard,
):
    from PySide6 import QtCore

    dashboard = enterprise_dashboard
    dashboard_width = (screen_width * 33 + 50) // 100
    dashboard.apply_window_rect(RectLike(0, 0, dashboard_width, height))
    dashboard.process_events()

    assert dashboard.control_scroll.horizontalScrollBar().maximum() == 0
    assert dashboard.interface_scroll.horizontalScrollBar().maximum() == 0
    assert dashboard.obstacle_table.horizontalScrollBar().maximum() == 0

    enterprise_controls = (
        ("pause", dashboard.pause_button),
        ("reset", dashboard.reset_button),
        ("linear", dashboard.linear_spin),
        ("angular", dashboard.angular_spin),
        ("quit", dashboard.quit_button),
        ("robot", dashboard.robot_combo),
        ("apply_robot", dashboard.apply_robot_button),
        ("terrain", dashboard.terrain_combo),
        ("slope", dashboard.slope_spin),
        ("golf_seed", dashboard.golf_seed_spin),
        ("golf_relief", dashboard.golf_relief_combo),
        ("apply_terrain", dashboard.apply_terrain_button),
        ("obstacle_mode", dashboard.obstacle_mode_combo),
        ("obstacle_shape", dashboard.obstacle_shape_combo),
        ("obstacle_count", dashboard.obstacle_count_spin),
        ("obstacle_seed", dashboard.obstacle_seed_spin),
        ("obstacle_speed", dashboard.obstacle_speed_spin),
        ("obstacle_ratio", dashboard.obstacle_ratio_spin),
        ("add_obstacles", dashboard.add_obstacles_button),
        ("delete_obstacle", dashboard.delete_obstacle_button),
        ("clear_obstacles", dashboard.clear_obstacles_button),
    )
    viewport_rect = dashboard.control_scroll.viewport().rect()
    for name, control in enterprise_controls:
        dashboard._ensure_control_fully_visible(control)
        dashboard.process_events()
        top_left = control.mapTo(dashboard.control_scroll.viewport(), control.rect().topLeft())
        assert viewport_rect.contains(
            QtCore.QRect(top_left, control.size())
        ), name

    group_rects = [group.geometry() for group in dashboard.control_groups]
    assert all(not upper.intersects(lower) for upper, lower in zip(group_rects, group_rects[1:]))
    interface_rects = [row.container.geometry() for row in dashboard.interface_rows.values()]
    assert all(not upper.intersects(lower) for upper, lower in zip(interface_rects, interface_rects[1:]))
    assert all(label.wordWrap() for row in dashboard.interface_rows.values() for label in row.value_labels)
