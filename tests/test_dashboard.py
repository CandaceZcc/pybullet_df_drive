# Dashboard 测试：只验证数据显示格式，不要求测试环境打开真实窗口。
import pytest

import slope_sim.dashboard as dashboard_module
from slope_sim.dashboard import (
    DashboardCommand,
    DASHBOARD_CONTROL_BAR_MAX_HEIGHT,
    DASHBOARD_CONTROL_BAR_STRETCH,
    DASHBOARD_CONTROL_SPINBOX_WIDTH,
    DASHBOARD_DIRECTION_BUTTON_SIZE,
    DASHBOARD_MAIN_AREA_STRETCH,
    DASHBOARD_PLOT_LEGEND_STYLE,
    TelemetryDashboard,
    TelemetryPlotBuffer,
    dashboard_plot_specs,
    dashboard_groups,
    dashboard_rows,
    dashboard_window_size,
    should_refresh_dashboard,
    smooth_telemetry,
)
from slope_sim.telemetry import RobotTelemetry


def test_dashboard_rows_format_core_telemetry_values():
    rows = dashboard_rows(
        RobotTelemetry(
            x=1.2345,
            y=-2.0,
            z=0.45,
            roll=0.1,
            pitch=-0.2,
            yaw=0.3,
            body_forward_speed=0.4,
            yaw_rate=0.5,
            velocity_sensor_vx=0.41,
            velocity_sensor_vy=0.02,
            velocity_sensor_vz=-0.01,
            velocity_sensor_body_forward_speed=0.40,
            velocity_sensor_yaw_rate=0.50,
            linear_acceleration_x=0.60,
            linear_acceleration_y=0.10,
            linear_acceleration_z=-0.20,
            angular_acceleration_z=0.03,
            left_actual_drive_speed=3.2,
            right_actual_drive_speed=3.4,
            left_track_surface_speed=0.26,
            right_track_surface_speed=0.27,
            left_body_track_speed=0.25,
            right_body_track_speed=0.30,
            left_slip_ratio=0.11,
            right_slip_ratio=0.12,
            left_slip_speed=0.01,
            right_slip_speed=-0.03,
            left_slip_valid=True,
            right_slip_valid=False,
            ground_lateral_friction=1.10,
            ground_rolling_friction=0.03,
            ground_spinning_friction=0.04,
            drive_lateral_friction=2.00,
            support_lateral_friction=0.02,
            track_anisotropic_friction_x=2.0,
            track_anisotropic_friction_y=0.05,
            track_anisotropic_friction_z=0.05,
            left_contact_friction_force=1.2,
            right_contact_friction_force=1.4,
            left_contact_count=2,
            right_contact_count=3,
            terrain_type="twr_slope_5deg",
            terrain_probe_valid=True,
            out_of_bounds=False,
            robot_model="tracked_proxy",
            local_ground_height=0.12,
            local_terrain_normal_x=-0.087,
            local_terrain_normal_z=0.996,
            lidar_min_distance=2.5,
            command_linear_velocity=0.6,
            command_angular_velocity=0.7,
        )
    )

    labels = {label: value for label, value in rows}
    assert labels["位置 x/y/z"] == "1.23 / -2.00 / 0.45 m"
    assert labels["姿态 roll/pitch/yaw"] == "5.7 / -11.5 / 17.2 deg"
    assert labels["车体速度 / yaw_rate"] == "0.40 m/s / 0.50 rad/s"
    assert labels["速度传感 vx/vy/vz"] == "0.41 / 0.02 / -0.01 m/s"
    assert labels["速度传感前向/yaw"] == "0.40 m/s / 0.50 rad/s"
    assert labels["加速度 xyz"] == "0.60 / 0.10 / -0.20 m/s^2"
    assert labels["角加速度 z"] == "0.03 rad/s^2"
    assert labels["驱动表面速度"] == "0.26 / 0.27 m/s"
    assert labels["驱动局部车速"] == "0.25 / 0.30 m/s"
    assert labels["打滑严重度 |率|"] == "0.11 / 低速"
    assert labels["带符号打滑率"] == "+0.11 / 低速"
    assert labels["打滑速度差"] == "+0.01 / -0.03 m/s"
    assert labels["接触摩擦力"] == "1.20 / 1.40 N"
    assert labels["有效接触点"] == "2 / 3"
    assert labels["地形类型"] == "twr_slope_5deg"
    assert labels["车型"] == "tracked_proxy"
    assert labels["地形探测"] == "有效"
    assert labels["越界保护"] == "正常"
    assert labels["地面高度 / 法向 z"] == "0.12 m / 1.00"
    assert labels["地形法向 x/y/z"] == "-0.09 / 0.00 / 1.00"
    assert labels["地面摩擦 lat/roll/spin"] == "1.10 / 0.03 / 0.04"
    assert labels["驱动/支撑摩擦"] == "2.00 / 0.02"
    assert labels["履带各向异性摩擦"] == "2.00 / 0.05 / 0.05"
    assert labels["最近障碍距离"] == "2.50 m"


def test_dashboard_groups_split_rows_by_topic():
    groups = dashboard_groups(RobotTelemetry())
    group_names = [name for name, _rows in groups]

    assert group_names == ["位姿", "速度", "速度传感", "接触 / 打滑", "地形 / 摩擦", "传感器 / 命令"]


def test_dashboard_rows_show_out_of_bounds_status():
    labels = {
        label: value
        for label, value in dashboard_rows(
            RobotTelemetry(
                terrain_probe_valid=False,
                out_of_bounds=True,
            )
        )
    }

    assert labels["地形探测"] == "无效"
    assert labels["越界保护"] == "越界"


def test_dashboard_command_can_request_model_switch_or_reset():
    command = DashboardCommand(
        linear_velocity=0.0,
        angular_velocity=0.0,
        requested_robot_model="tracked_proxy",
        reset_requested=True,
    )

    assert command.requested_robot_model == "tracked_proxy"
    assert command.reset_requested is True


def test_telemetry_plot_buffer_keeps_recent_window_and_series():
    buffer = TelemetryPlotBuffer(window_sec=1.0)
    buffer.append(
        RobotTelemetry(
            t=0.0,
            x=0.0,
            y=0.0,
            command_linear_velocity=0.2,
            body_forward_speed=0.1,
            command_angular_velocity=0.0,
            yaw_rate=0.01,
            left_slip_ratio=0.1,
            right_slip_ratio=0.2,
            left_contact_normal_force=3.0,
            right_contact_normal_force=4.0,
        )
    )
    buffer.append(
        RobotTelemetry(
            t=0.5,
            x=0.1,
            y=0.0,
            command_linear_velocity=0.2,
            body_forward_speed=0.15,
            left_slip_ratio=-0.3,
            right_slip_ratio=0.25,
        )
    )
    buffer.append(RobotTelemetry(t=1.5, x=0.2, y=0.1, command_linear_velocity=0.2, body_forward_speed=0.18))

    series = buffer.series()

    assert series["t"] == [0.5, 1.5]
    assert series["x"] == [0.1, 0.2]
    assert series["y"] == [0.0, 0.1]
    assert series["command_linear_velocity"] == [0.2, 0.2]
    assert series["body_forward_speed"] == [0.15, 0.18]
    assert series["left_abs_slip_ratio"] == [0.3, 0.0]
    assert series["right_abs_slip_ratio"] == [0.25, 0.0]

    buffer.clear()

    assert buffer.series()["t"] == []


def test_dashboard_plot_specs_use_one_chart_per_tab_with_compact_legend():
    specs = dashboard_plot_specs()

    assert [spec.tab_label for spec in specs] == ["轨迹", "速度/命令", "打滑", "接触"]
    assert all(spec.tab_label != "曲线" for spec in specs)
    assert all(len(spec.lines) >= 1 for spec in specs)
    slip_spec = next(spec for spec in specs if spec.tab_label == "打滑")
    assert slip_spec.title == "slip severity"
    assert [line.y_field for line in slip_spec.lines[:2]] == ["left_abs_slip_ratio", "right_abs_slip_ratio"]
    assert DASHBOARD_PLOT_LEGEND_STYLE == {
        "fontsize": 7,
        "framealpha": 0.65,
        "borderpad": 0.25,
        "handlelength": 1.2,
        "loc": "upper right",
    }


def test_dashboard_window_size_clamps_to_available_screen():
    assert dashboard_window_size(available_width=900, available_height=700) == (820, 616)
    assert dashboard_window_size(available_width=5000, available_height=3000) == (1180, 1100)
    assert dashboard_window_size(available_width=600, available_height=320) == (570, 304)


def test_dashboard_layout_constants_keep_plots_dominant_and_controls_compact():
    assert DASHBOARD_MAIN_AREA_STRETCH == 2
    assert DASHBOARD_CONTROL_BAR_STRETCH == 0
    assert 34 <= DASHBOARD_DIRECTION_BUTTON_SIZE <= 38
    assert 90 <= DASHBOARD_CONTROL_SPINBOX_WIDTH <= 110
    assert DASHBOARD_CONTROL_BAR_MAX_HEIGHT <= 170


def test_dashboard_keyboard_controls_work_when_child_widget_has_focus(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtCore, QtGui

    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.linear_spin.setFocus()
        press = QtGui.QKeyEvent(QtCore.QEvent.KeyPress, QtCore.Qt.Key_Up, QtCore.Qt.NoModifier)
        dashboard.app.sendEvent(dashboard.linear_spin, press)
        dashboard.process_events()

        assert dashboard.current_command().linear_velocity == 0.4

        release = QtGui.QKeyEvent(QtCore.QEvent.KeyRelease, QtCore.Qt.Key_Up, QtCore.Qt.NoModifier)
        dashboard.app.sendEvent(dashboard.linear_spin, release)
        dashboard.process_events()

        assert dashboard.current_command().linear_velocity == 0.0
    finally:
        dashboard.close()


def test_dashboard_button_click_pulse_lasts_long_enough_for_visible_motion(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtCore

    now = {"value": 100.0}
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: now["value"])
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard._pulse_button_key(QtCore.Qt.Key_Up)

        # 鼠标短点击也要跨过多帧仿真，避免 10 度坡面上位移太小看不出来。
        now["value"] = 100.5
        assert dashboard.current_command().linear_velocity == pytest.approx(0.4)

        now["value"] = 101.2
        assert dashboard.current_command().linear_velocity == 0.0
    finally:
        dashboard.close()


def test_dashboard_direction_button_click_sends_forward_command(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtCore, QtTest, QtWidgets

    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        up_buttons = [button for button in dashboard.window.findChildren(QtWidgets.QPushButton) if button.text() == "↑"]
        assert len(up_buttons) == 1

        QtTest.QTest.mouseClick(up_buttons[0], QtCore.Qt.LeftButton)
        dashboard.process_events()

        assert dashboard.current_command().linear_velocity == pytest.approx(0.4)
    finally:
        dashboard.close()


def test_dashboard_skips_hidden_plot_draws_when_data_tab_is_current(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        draw_counts = {label: 0 for label in dashboard.plot_canvases}
        for label, canvas in dashboard.plot_canvases.items():
            canvas.draw_idle = lambda label=label: draw_counts.__setitem__(label, draw_counts[label] + 1)

        dashboard.tabs.setCurrentIndex(0)
        dashboard.update(RobotTelemetry(t=0.0, x=1.0, y=0.0))

        assert draw_counts == {label: 0 for label in dashboard.plot_canvases}
    finally:
        dashboard.close()


def test_dashboard_direction_button_outputs_command_while_pressed(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        up_button = next(button for button in dashboard.window.findChildren(QtWidgets.QPushButton) if button.text() == "↑")
        up_button.pressed.emit()

        assert dashboard.current_command().linear_velocity == 0.4

        up_button.released.emit()

        assert dashboard.current_command().linear_velocity == 0.0
    finally:
        dashboard.close()


def test_dashboard_direction_button_click_outputs_short_command_pulse(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        up_button = next(button for button in dashboard.window.findChildren(QtWidgets.QPushButton) if button.text() == "↑")
        up_button.clicked.emit()

        assert dashboard.current_command().linear_velocity == 0.4
    finally:
        dashboard.close()


def test_should_refresh_dashboard_respects_update_rate():
    assert should_refresh_dashboard(last_update_time=None, now=10.0, update_hz=5.0) is True
    assert should_refresh_dashboard(last_update_time=10.0, now=10.1, update_hz=5.0) is False
    assert should_refresh_dashboard(last_update_time=10.0, now=10.2, update_hz=5.0) is True


def test_smooth_telemetry_smooths_feedback_but_keeps_command_current():
    previous = RobotTelemetry(
        left_actual_drive_speed=4.0,
        right_actual_drive_speed=4.0,
        left_track_surface_speed=0.32,
        right_track_surface_speed=0.32,
        left_body_track_speed=0.30,
        right_body_track_speed=0.30,
        left_contact_normal_force=20.0,
        left_contact_friction_force=4.0,
        left_slip_speed=0.02,
        command_linear_velocity=0.1,
    )
    current = RobotTelemetry(
        left_actual_drive_speed=8.0,
        right_actual_drive_speed=12.0,
        left_track_surface_speed=0.64,
        right_track_surface_speed=0.96,
        left_body_track_speed=0.50,
        right_body_track_speed=0.70,
        left_contact_normal_force=40.0,
        left_contact_friction_force=8.0,
        left_slip_speed=0.10,
        command_linear_velocity=0.5,
    )

    smoothed = smooth_telemetry(previous, current, alpha=0.25)

    assert smoothed.left_actual_drive_speed == 5.0
    assert smoothed.right_actual_drive_speed == 6.0
    assert smoothed.left_track_surface_speed == 0.4
    assert smoothed.right_body_track_speed == pytest.approx(0.4)
    assert smoothed.left_contact_normal_force == 25.0
    assert smoothed.left_contact_friction_force == 5.0
    assert smoothed.left_slip_speed == pytest.approx(0.04)
    assert smoothed.command_linear_velocity == 0.5
