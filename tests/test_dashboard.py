# Dashboard 测试：只验证数据显示格式，不要求测试环境打开真实窗口。
import pytest

import slope_sim.dashboard as dashboard_module
from slope_sim.dashboard import (
    DashboardCommand,
    DASHBOARD_CONTROL_SPINBOX_WIDTH,
    DASHBOARD_DIRECTION_BUTTON_SIZE,
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
from slope_sim.obstacles import ObstacleGenerationRequest, ObstacleSnapshot
from slope_sim.runtime_actions import (
    AddObstaclesAction,
    ClearObstaclesAction,
    DeleteObstacleAction,
    ResetRobotAction,
    SwitchRobotAction,
    SwitchTerrainAction,
    TerrainSelection,
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
            terrain_type="slope",
            terrain_probe_valid=True,
            out_of_bounds=False,
            robot_model="df_mid",
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
    assert labels["地形类型"] == "slope"
    assert labels["车型"] == "df_mid"
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


def test_dashboard_rows_show_active_steering_four_wheel_feedback():
    labels = {
        label: value
        for label, value in dashboard_rows(
            RobotTelemetry(
                robot_model="active_steering_4wd",
                front_left_actual_drive_speed=3.0,
                front_right_actual_drive_speed=4.0,
                rear_left_actual_drive_speed=5.0,
                rear_right_actual_drive_speed=6.0,
                front_left_actual_steering_angle=0.1,
                front_right_actual_steering_angle=0.2,
            )
        )
    }

    assert labels["四轮实际驱动速度 FL/FR/RL/RR"] == "3.00 / 4.00 / 5.00 / 6.00 rad/s"
    assert labels["前轮实际转角 FL/FR"] == "0.10 / 0.20 rad"


def test_dashboard_command_carries_at_most_one_structural_action():
    command = DashboardCommand(linear_velocity=0.0, angular_velocity=0.0, structural_action=SwitchRobotAction("df_mid"))

    assert command.structural_action == SwitchRobotAction("df_mid")
    assert DashboardCommand(0.0, 0.0, structural_action=ResetRobotAction()).structural_action == ResetRobotAction()


def test_dashboard_camera_controls_use_initial_state_and_emit_current_state(monkeypatch):
    """相机控件应展示配置初值，并把每帧状态写入命令。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        camera_follow_enabled=True,
        camera_follow_view="side",
    )
    try:
        assert dashboard.camera_follow_checkbox.isChecked() is True
        assert [dashboard.camera_view_combo.itemData(index) for index in range(dashboard.camera_view_combo.count())] == [
            "front",
            "side",
            "custom",
        ]
        assert [dashboard.camera_view_combo.itemText(index) for index in range(dashboard.camera_view_combo.count())] == [
            "车后",
            "侧面",
            "固定",
        ]
        assert dashboard.camera_view_combo.currentData() == "side"
        assert dashboard.camera_view_combo.isEnabled() is True

        dashboard.camera_view_combo.setCurrentIndex(dashboard.camera_view_combo.findData("custom"))
        dashboard.camera_follow_checkbox.setChecked(False)
        command = dashboard.current_command()

        assert dashboard.camera_view_combo.isEnabled() is False
        assert command.camera_follow_enabled is False
        assert command.camera_follow_view == "custom"
    finally:
        dashboard.close()


def test_dashboard_stop_exit_and_scene_requests_keep_camera_state(monkeypatch):
    """停车、退出和一次性场景请求不能吞掉持续相机状态。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        terrain_switch_enabled=True,
        camera_follow_enabled=True,
        camera_follow_view="side",
    )
    try:
        dashboard._pressed_keys.add(dashboard._normalize_key(dashboard.QtCore.Qt.Key_Space))
        stopped = dashboard.current_command()
        dashboard._pressed_keys.clear()
        dashboard.request_exit()
        exiting = dashboard.current_command()
        dashboard.terrain_combo.setCurrentIndex(dashboard.terrain_combo.findData("slope"))
        dashboard.request_terrain_switch()
        requested = dashboard.current_command()

        assert (stopped.camera_follow_enabled, stopped.camera_follow_view) == (True, "side")
        assert (exiting.camera_follow_enabled, exiting.camera_follow_view) == (True, "side")
        assert (requested.camera_follow_enabled, requested.camera_follow_view) == (True, "side")
        assert requested.structural_action == SwitchTerrainAction(TerrainSelection("slope"))
    finally:
        dashboard.close()


def test_dashboard_model_switch_requires_apply_and_is_one_shot(monkeypatch):
    """只改变车型下拉框不能修改仿真，点击应用后请求只发送一次。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        robot_model="df_back",
        model_switch_enabled=True,
    )
    try:
        dashboard.robot_combo.setCurrentIndex(dashboard.robot_combo.findData("df_mid"))
        assert dashboard.current_command().structural_action is None

        dashboard.request_robot_switch()
        assert dashboard.current_command().structural_action == SwitchRobotAction("df_mid")
        assert dashboard.current_command().structural_action is None
    finally:
        dashboard.close()


def test_dashboard_terrain_switch_requires_apply_and_captures_parameters(monkeypatch):
    """应用场地请求需要完整携带地形类型及对应参数。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        terrain_model="flat",
        terrain_switch_enabled=True,
    )
    try:
        dashboard.terrain_combo.setCurrentIndex(dashboard.terrain_combo.findData("slope"))
        dashboard.slope_spin.setValue(9.5)
        assert dashboard.current_command().structural_action is None

        dashboard.request_terrain_switch()
        request = dashboard.current_command().structural_action
        assert request == SwitchTerrainAction(TerrainSelection("slope", slope_deg=9.5))
        assert dashboard.current_command().structural_action is None
    finally:
        dashboard.close()


def test_dashboard_top_tabs_include_obstacle_table(monkeypatch):
    """顶部标签页应新增障碍物表格，并只展示稳定逻辑字段。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        assert [dashboard.tabs.tabText(index) for index in range(dashboard.tabs.count())] == [
            "数据",
            "障碍物",
            "轨迹",
            "速度/命令",
            "打滑",
            "接触",
        ]
        assert dashboard.obstacle_table.selectionBehavior() == QtWidgets.QAbstractItemView.SelectRows
        assert dashboard.obstacle_table.selectionMode() == QtWidgets.QAbstractItemView.SingleSelection
        assert [
            dashboard.obstacle_table.horizontalHeaderItem(column).text()
            for column in range(dashboard.obstacle_table.columnCount())
        ] == ["逻辑ID", "模式", "形状", "位置"]
    finally:
        dashboard.close()


def test_dashboard_obstacle_snapshot_refresh_preserves_selection_by_logical_id(monkeypatch):
    """障碍物位置刷新后，应按逻辑 ID 恢复当前选中行。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.update_obstacle_snapshots(
            (
                ObstacleSnapshot(1, None, "static", "box", (1.0, 2.0, 0.3), (0.0, 0.0, 0.0, 1.0)),
                ObstacleSnapshot(2, None, "moving", "sphere", (3.0, 4.0, 0.5), (0.0, 0.0, 0.0, 1.0)),
            )
        )
        dashboard.obstacle_table.selectRow(1)

        dashboard.update_obstacle_snapshots(
            (
                ObstacleSnapshot(2, None, "moving", "sphere", (3.5, 4.5, 0.5), (0.0, 0.0, 0.0, 1.0)),
                ObstacleSnapshot(3, None, "static", "cylinder", (-1.0, 0.0, 0.4), (0.0, 0.0, 0.0, 1.0)),
            ),
            force=True,
        )

        selected_rows = dashboard.obstacle_table.selectionModel().selectedRows()
        assert len(selected_rows) == 1
        assert dashboard.obstacle_table.item(selected_rows[0].row(), 0).data(dashboard.QtCore.Qt.UserRole) == 2
        assert dashboard.obstacle_table.item(selected_rows[0].row(), 3).text() == "3.50, 4.50, 0.50"
    finally:
        dashboard.close()


def test_dashboard_obstacle_snapshot_refresh_is_throttled(monkeypatch):
    """障碍物表格刷新频率不能跟 240Hz 物理循环绑定。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    now = {"value": 100.0}
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: now["value"])
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8, update_hz=5.0)
    try:
        first = (ObstacleSnapshot(1, None, "static", "box", (1.0, 2.0, 0.3), (0.0, 0.0, 0.0, 1.0)),)
        second = (ObstacleSnapshot(1, None, "static", "box", (9.0, 2.0, 0.3), (0.0, 0.0, 0.0, 1.0)),)

        assert dashboard.update_obstacle_snapshots(first) is True
        now["value"] = 100.05
        assert dashboard.update_obstacle_snapshots(second) is False
        assert dashboard.obstacle_table.item(0, 3).text() == "1.00, 2.00, 0.30"

        now["value"] = 100.20
        assert dashboard.update_obstacle_snapshots(second) is True
        assert dashboard.obstacle_table.item(0, 3).text() == "9.00, 2.00, 0.30"
    finally:
        dashboard.close()


def test_dashboard_control_scroll_includes_obstacle_group(monkeypatch):
    """障碍物控件组应位于下方滚动控制区，不挤占顶部 tabs。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        model_switch_enabled=True,
        terrain_switch_enabled=True,
    )
    try:
        assert [group.title() for group in dashboard.control_groups] == ["参数", "车型", "场地", "障碍物", "相机", "控制"]
        assert dashboard.obstacle_group.parentWidget() is dashboard.control_content
        assert dashboard.control_scroll.isAncestorOf(dashboard.obstacle_group)
        assert not dashboard.tabs.isAncestorOf(dashboard.obstacle_group)
    finally:
        dashboard.close()


@pytest.mark.parametrize(
    ("mode", "speed_enabled", "ratio_enabled"),
    [
        ("static", False, False),
        ("moving", True, False),
        ("mixed", True, True),
    ],
)
def test_dashboard_obstacle_mode_enables_relevant_controls(monkeypatch, mode, speed_enabled, ratio_enabled):
    """不同障碍物模式只开放真正会参与生成请求的参数。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.obstacle_mode_combo.setCurrentIndex(dashboard.obstacle_mode_combo.findData(mode))

        assert dashboard.obstacle_speed_spin.isEnabled() is speed_enabled
        assert dashboard.obstacle_ratio_spin.isEnabled() is ratio_enabled
        assert dashboard.obstacle_ratio_spin.value() == 30
        assert dashboard.obstacle_count_spin.minimum() == 1
        assert dashboard.obstacle_count_spin.maximum() == 50
    finally:
        dashboard.close()


def test_dashboard_add_obstacle_command_captures_all_controls(monkeypatch):
    """添加请求必须完整携带模式、形状、数量、种子、速度和混合比例。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.obstacle_mode_combo.setCurrentIndex(dashboard.obstacle_mode_combo.findData("mixed"))
        dashboard.obstacle_shape_combo.setCurrentIndex(dashboard.obstacle_shape_combo.findData("cylinder"))
        dashboard.obstacle_count_spin.setValue(7)
        dashboard.obstacle_seed_spin.setValue(123)
        dashboard.obstacle_speed_spin.setValue(0.65)
        dashboard.obstacle_ratio_spin.setValue(45)

        dashboard.request_add_obstacles()
        action = dashboard.current_command().structural_action

        assert action == AddObstaclesAction(
            ObstacleGenerationRequest(
                mode="mixed",
                shape="cylinder",
                count=7,
                seed=123,
                moving_speed=0.65,
                moving_ratio=0.45,
            )
        )
    finally:
        dashboard.close()


def test_dashboard_delete_obstacle_uses_selected_logical_id(monkeypatch):
    """未选择行时删除按钮禁用；选择后按逻辑 ID 生成删除请求。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.update_obstacle_snapshots(
            (ObstacleSnapshot(7, None, "static", "box", (0.0, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0)),)
        )
        assert dashboard.delete_obstacle_button.isEnabled() is False

        dashboard.obstacle_table.selectRow(0)
        assert dashboard.delete_obstacle_button.isEnabled() is True
        dashboard.request_delete_obstacle()

        assert dashboard.current_command().structural_action == DeleteObstacleAction(7)
    finally:
        dashboard.close()


def test_dashboard_clear_obstacles_queues_directly_without_modal(monkeypatch):
    """清空障碍物不应弹阻塞确认框，直接排入结构动作队列。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    def fail_modal(*_args, **_kwargs):
        raise AssertionError("clear obstacles must not open a modal dialog")

    monkeypatch.setattr(QtWidgets.QMessageBox, "question", fail_modal, raising=False)
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.clear_obstacles_button.click()

        assert dashboard.current_command().structural_action == ClearObstaclesAction()
    finally:
        dashboard.close()


def test_dashboard_structural_actions_emit_fifo_without_loss(monkeypatch):
    """Dashboard 每帧最多输出一个结构动作，多次排队保持 FIFO 且不重复。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        actions = [
            AddObstaclesAction(ObstacleGenerationRequest("static", 1, seed=1)),
            DeleteObstacleAction(3),
            ClearObstaclesAction(),
        ]
        for action in actions:
            dashboard._enqueue_structural_action(action)

        assert [dashboard.current_command().structural_action for _ in range(4)] == [*actions, None]
    finally:
        dashboard.close()


def test_dashboard_public_obstacle_requests_queue_fifo_before_current_command(monkeypatch):
    """同一轮 Qt 事件里多个公开请求应全部入队，不能被 busy 状态吞掉。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.update_obstacle_snapshots(
            (ObstacleSnapshot(3, None, "static", "box", (0.0, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0)),)
        )
        dashboard.obstacle_table.selectRow(0)

        dashboard.request_add_obstacles()
        dashboard.request_delete_obstacle()
        dashboard.request_clear_obstacles()

        actions = [dashboard.current_command().structural_action for _ in range(4)]
        assert isinstance(actions[0], AddObstaclesAction)
        assert actions[1:] == [DeleteObstacleAction(3), ClearObstaclesAction(), None]
    finally:
        dashboard.close()


def test_dashboard_structure_busy_disables_buttons_and_restores_status(monkeypatch):
    """结构操作忙碌期间禁用相关按钮，成功或失败都恢复并显示统一状态。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        model_switch_enabled=True,
        terrain_switch_enabled=True,
    )
    try:
        dashboard.set_structure_busy(True, "应用中")

        related_buttons = (
            dashboard.apply_robot_button,
            dashboard.apply_terrain_button,
            dashboard.reset_button,
            dashboard.add_obstacles_button,
            dashboard.delete_obstacle_button,
            dashboard.clear_obstacles_button,
        )
        assert all(button.isEnabled() is False for button in related_buttons)
        assert dashboard.structure_status_label.text() == "结构状态：应用中"

        dashboard.show_switch_status("切换失败: boom", is_error=True)
        assert all(button.isEnabled() is True for button in related_buttons if button is not dashboard.delete_obstacle_button)
        assert dashboard.delete_obstacle_button.isEnabled() is False
        assert dashboard.structure_status_label.text() == "结构状态：切换失败: boom"
        assert "#b00020" in dashboard.structure_status_label.styleSheet()
    finally:
        dashboard.close()


def test_dashboard_enables_only_parameters_for_pending_terrain(monkeypatch):
    """场地参数的可编辑状态跟随待应用选择，而不是当前活动场地。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        terrain_switch_enabled=True,
    )
    try:
        dashboard.terrain_combo.setCurrentIndex(dashboard.terrain_combo.findData("slope"))
        assert dashboard.slope_spin.isEnabled() is True
        assert dashboard.golf_seed_spin.isEnabled() is False
        assert dashboard.golf_relief_combo.isEnabled() is False

        dashboard.terrain_combo.setCurrentIndex(dashboard.terrain_combo.findData("golf_heightfield"))
        assert dashboard.slope_spin.isEnabled() is False
        assert dashboard.golf_seed_spin.isEnabled() is True
        assert dashboard.golf_relief_combo.isEnabled() is True
    finally:
        dashboard.close()


def test_dashboard_syncs_controls_to_the_active_world(monkeypatch):
    """场景回滚后，待应用控件必须恢复为真实活动世界。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        model_switch_enabled=True,
        terrain_switch_enabled=True,
    )
    try:
        terrain = TerrainSelection(
            "golf_heightfield",
            slope_deg=7.0,
            golf_seed=41,
            golf_relief="high",
        )
        dashboard.sync_active_selection("df_mid", terrain)

        assert dashboard.robot_combo.currentData() == "df_mid"
        assert dashboard.terrain_combo.currentData() == "golf_heightfield"
        assert dashboard.slope_spin.value() == pytest.approx(7.0)
        assert dashboard.golf_seed_spin.value() == 41
        assert dashboard.golf_relief_combo.currentData() == "high"
    finally:
        dashboard.close()


def test_dashboard_shows_non_blocking_switch_error(monkeypatch):
    """切换失败只更新统一状态标签，不弹出阻塞物理循环的对话框。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        terrain_switch_enabled=True,
    )
    try:
        dashboard.show_switch_status("已恢复旧场地", is_error=True)

        assert dashboard.switch_status_label is dashboard.structure_status_label
        assert dashboard.structure_status_label.text() == "结构状态：已恢复旧场地"
        assert "#b00020" in dashboard.structure_status_label.styleSheet()
    finally:
        dashboard.close()


def test_dashboard_disables_scene_buttons_until_switch_finishes(monkeypatch):
    """一次请求待处理期间必须禁止车型、场地和复位重复提交。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        model_switch_enabled=True,
        terrain_switch_enabled=True,
    )
    try:
        dashboard.robot_combo.setCurrentIndex(dashboard.robot_combo.findData("df_mid"))
        dashboard.apply_robot_button.click()

        assert dashboard.apply_robot_button.isEnabled() is False
        assert dashboard.apply_terrain_button.isEnabled() is False
        assert dashboard.reset_button.isEnabled() is False
        assert dashboard.current_command().structural_action == SwitchRobotAction("df_mid")

        dashboard.request_terrain_switch()
        assert dashboard.current_command().structural_action is None

        dashboard.show_switch_status("车型已切换为 df_mid")
        assert dashboard.apply_robot_button.isEnabled() is True
        assert dashboard.apply_terrain_button.isEnabled() is True
        assert dashboard.reset_button.isEnabled() is True
    finally:
        dashboard.close()


def test_dashboard_reset_request_marks_structure_busy_immediately(monkeypatch):
    """复位也是安全停车结构操作，请求发出后不能被同帧其他按钮覆盖。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        model_switch_enabled=True,
        terrain_switch_enabled=True,
    )
    try:
        dashboard.request_reset()

        assert dashboard.apply_robot_button.isEnabled() is False
        assert dashboard.apply_terrain_button.isEnabled() is False
        assert dashboard.reset_button.isEnabled() is False

        dashboard.robot_combo.setCurrentIndex(dashboard.robot_combo.findData("df_mid"))
        dashboard.request_robot_switch()
        assert dashboard.current_command().structural_action == ResetRobotAction()
    finally:
        dashboard.close()


def test_default_stage1_dashboard_exposes_current_robot_reset(monkeypatch):
    """阶段一不启用车型切换时，仍必须给用户一个可达的车辆复位按钮。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtWidgets

    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        reset_button = next(
            button for button in dashboard.window.findChildren(QtWidgets.QPushButton) if button.text() == "复位车辆"
        )
        reset_button.click()

        assert dashboard.current_command().structural_action == ResetRobotAction()
    finally:
        dashboard.close()


def test_dashboard_reset_feedback_history_clears_smoothing_and_plots(monkeypatch):
    """车辆重载后不能继续混合显示旧车的平滑值和曲线。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard._smoothed_telemetry = RobotTelemetry(x=8.0)
        dashboard._last_update_time = 10.0
        dashboard._last_plot_update_time = 10.0
        dashboard.plot_buffer.append(RobotTelemetry(t=1.0, x=8.0))

        dashboard.reset_feedback_history()

        assert dashboard._smoothed_telemetry is None
        assert dashboard._last_update_time is None
        assert dashboard._last_plot_update_time is None
        assert dashboard.plot_buffer.series()["x"] == []
    finally:
        dashboard.close()


def _dashboard_plot_tab_indices(dashboard):
    """按标签名定位曲线页，避免新增非曲线页后测试依赖裸索引。"""
    return [
        index
        for index in range(dashboard.tabs.count())
        if dashboard.tabs.tabText(index) in dashboard.plot_canvases
    ]


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


def test_dashboard_window_size_uses_fixed_sidebar_width_and_screen_height():
    assert dashboard_window_size(available_width=900, available_height=700) == (420, 665)
    assert dashboard_window_size(available_width=5000, available_height=3000) == (420, 2850)
    assert dashboard_window_size(available_width=320, available_height=320) == (420, 304)


def test_dashboard_fixed_sidebar_uses_vertical_control_scroll(monkeypatch):
    """固定侧栏应把五组控件纵向放入独立滚动区。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtCore, QtWidgets

    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        model_switch_enabled=True,
        terrain_switch_enabled=True,
        camera_follow_enabled=True,
    )
    try:
        dashboard.process_events()

        screen = dashboard.app.primaryScreen().availableGeometry()
        expected_size = dashboard_window_size(screen.width(), screen.height())
        assert (dashboard.window.width(), dashboard.window.height()) == expected_size
        assert dashboard.window.minimumWidth() == dashboard.window.maximumWidth() == dashboard_module.DASHBOARD_FIXED_WIDTH
        assert dashboard.window.minimumHeight() == dashboard.window.maximumHeight() == expected_size[1]

        dashboard.window.resize(dashboard_module.DASHBOARD_FIXED_WIDTH + 200, expected_size[1] + 200)
        dashboard.process_events()
        assert (dashboard.window.width(), dashboard.window.height()) == expected_size

        assert dashboard.control_scroll.widget() is dashboard.control_content
        assert dashboard.control_scroll.horizontalScrollBarPolicy() == QtCore.Qt.ScrollBarAlwaysOff
        assert dashboard.control_scroll.verticalScrollBar().maximum() > 0
        assert [group.title() for group in dashboard.control_groups] == ["参数", "车型", "场地", "障碍物", "相机", "控制"]
        assert all(group.parentWidget() is dashboard.control_content for group in dashboard.control_groups)
        assert not dashboard.control_scroll.isAncestorOf(dashboard.tabs)
        assert all(not dashboard.control_scroll.isAncestorOf(canvas) for canvas in dashboard.plot_canvases.values())
        assert dashboard.telemetry_scroll is not dashboard.control_scroll
        assert dashboard.tabs.isAncestorOf(dashboard.telemetry_scroll)

        def window_rect(widget):
            """把嵌套控件坐标统一到 Dashboard 窗口坐标系。"""
            top_left = widget.mapTo(dashboard.window, widget.rect().topLeft())
            return QtCore.QRect(top_left, widget.size())

        groups = {group.title(): group for group in dashboard.control_groups}
        group_rects = [window_rect(group) for group in dashboard.control_groups]
        assert all(upper.bottom() < lower.top() for upper, lower in zip(group_rects, group_rects[1:]))

        key_widgets = (
            ("参数", dashboard.linear_spin),
            ("参数", dashboard.angular_spin),
            ("车型", dashboard.robot_combo),
            ("车型", dashboard.apply_robot_button),
            ("车型", dashboard.reset_button),
            ("场地", dashboard.terrain_combo),
            ("场地", dashboard.slope_spin),
            ("场地", dashboard.golf_seed_spin),
            ("场地", dashboard.golf_relief_combo),
            ("场地", dashboard.apply_terrain_button),
            ("障碍物", dashboard.obstacle_mode_combo),
            ("障碍物", dashboard.obstacle_shape_combo),
            ("障碍物", dashboard.obstacle_count_spin),
            ("障碍物", dashboard.obstacle_seed_spin),
            ("障碍物", dashboard.obstacle_speed_spin),
            ("障碍物", dashboard.obstacle_ratio_spin),
            ("障碍物", dashboard.add_obstacles_button),
            ("障碍物", dashboard.delete_obstacle_button),
            ("障碍物", dashboard.clear_obstacles_button),
            ("障碍物", dashboard.structure_status_label),
            ("相机", dashboard.camera_follow_checkbox),
            ("相机", dashboard.camera_view_combo),
        )
        label_texts = {
            "参数": ("最大线速度", "最大角速度"),
            "场地": ("场地", "坡度", "随机种子", "起伏"),
            "障碍物": ("模式", "形状", "数量", "随机种子", "速度", "移动占比"),
            "相机": ("视角",),
        }
        for group_name, texts in label_texts.items():
            for text in texts:
                label = next(label for label in groups[group_name].findChildren(QtWidgets.QLabel) if label.text() == text)
                key_widgets += ((group_name, label),)

        for group_name, widget in key_widgets:
            assert groups[group_name].isAncestorOf(widget)
    finally:
        dashboard.close()


def _assert_plot_artists_inside_canvas(label, axis, canvas):
    """真实绘制后检查标题、轴标签和图例均未越出画布。"""
    canvas.draw()
    renderer = canvas.get_renderer()
    canvas_width, canvas_height = canvas.get_width_height()
    artists = [("title", axis.title), ("xlabel", axis.xaxis.label), ("ylabel", axis.yaxis.label)]
    legend = axis.get_legend()
    if legend is not None:
        artists.append(("legend", legend))

    for artist_name, artist in artists:
        bounds = artist.get_window_extent(renderer)
        assert bounds.x0 >= 0, f"{label} {artist_name} left={bounds.x0}"
        assert bounds.y0 >= 0, f"{label} {artist_name} bottom={bounds.y0}"
        assert bounds.x1 <= canvas_width, f"{label} {artist_name} right={bounds.x1} > {canvas_width}"
        assert bounds.y1 <= canvas_height, f"{label} {artist_name} top={bounds.y1} > {canvas_height}"


def test_dashboard_small_fixed_window_keeps_plot_content_and_controls_visible(monkeypatch):
    """极小屏幕不能重叠区域，也不能裁掉图表控件或实际绘制内容。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(dashboard_module, "dashboard_window_size", lambda _width, _height: (420, 304))
    from PySide6 import QtCore, QtWidgets

    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        model_switch_enabled=True,
        terrain_switch_enabled=True,
    )
    try:
        dashboard.process_events()

        def window_rect(widget):
            """把嵌套控件坐标统一到 Dashboard 窗口坐标系。"""
            top_left = widget.mapTo(dashboard.window, widget.rect().topLeft())
            return QtCore.QRect(top_left, widget.size())

        tabs_rect = window_rect(dashboard.tabs)
        controls_rect = window_rect(dashboard.control_scroll)
        assert tabs_rect.bottom() < controls_rect.top()
        assert dashboard.control_scroll.viewport().height() > 0
        assert dashboard.control_scroll.verticalScrollBar().maximum() > 0
        assert [group.title() for group in dashboard.control_groups] == ["参数", "车型", "场地", "障碍物", "相机", "控制"]

        dashboard.control_scroll.verticalScrollBar().setValue(dashboard.control_scroll.verticalScrollBar().maximum())
        dashboard.process_events()
        assert window_rect(dashboard.control_groups[-1]).intersects(window_rect(dashboard.control_scroll.viewport()))

        for index in _dashboard_plot_tab_indices(dashboard):
            dashboard.tabs.setCurrentIndex(index)
            dashboard.process_events()
            plot_tab = dashboard.tabs.widget(index)
            label = dashboard.tabs.tabText(index)
            buttons = plot_tab.findChildren(QtWidgets.QPushButton)
            assert [button.text() for button in buttons] == ["清空曲线", "保存当前图"]
            for button in buttons:
                assert dashboard.window.rect().contains(window_rect(button))
                assert not button.visibleRegion().isEmpty()
            _assert_plot_artists_inside_canvas(
                label,
                dashboard.plot_axes[label],
                dashboard.plot_canvases[label],
            )
    finally:
        dashboard.close()


def test_dashboard_layout_constants_describe_fixed_vertical_sidebar():
    assert dashboard_module.DASHBOARD_FIXED_WIDTH == 420
    assert dashboard_module.DASHBOARD_TOP_AREA_STRETCH == 45
    assert dashboard_module.DASHBOARD_CONTROL_AREA_STRETCH == 55
    assert dashboard_module.DASHBOARD_TOP_TABS_MIN_HEIGHT == 320
    assert dashboard_module.DASHBOARD_PLOT_FIGURE_SIZE == (4.0, 3.2)
    assert dashboard_module.DASHBOARD_PLOT_MARGINS == {
        "left": 0.24,
        "right": 0.96,
        "bottom": 0.20,
        "top": 0.86,
    }
    assert dashboard_module.DASHBOARD_COMPACT_HEIGHT_PLOT_MARGINS == {
        "left": 0.24,
        "right": 0.96,
        "bottom": 0.35,
        "top": 0.69,
    }
    assert dashboard_module.DASHBOARD_COMPACT_CONTROL_SCROLL_HEIGHT == 20
    assert 34 <= DASHBOARD_DIRECTION_BUTTON_SIZE <= 38
    assert 90 <= DASHBOARD_CONTROL_SPINBOX_WIDTH <= 110


def test_dashboard_narrow_plot_layout_keeps_axes_and_buttons_in_top_tabs(monkeypatch):
    """窄画布应保留轴标签边距，图表按钮也必须留在顶部标签页。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtCore, QtWidgets

    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.process_events()

        for label, figure in dashboard.plot_figures.items():
            assert tuple(figure.get_size_inches()) == pytest.approx(dashboard_module.DASHBOARD_PLOT_FIGURE_SIZE)
            subplotpars = figure.subplotpars
            assert subplotpars.left >= 0.23
            assert subplotpars.bottom >= 0.18
            assert subplotpars.right <= 0.97
            assert subplotpars.top <= 0.90

            index = next(index for index in _dashboard_plot_tab_indices(dashboard) if dashboard.tabs.tabText(index) == label)
            dashboard.tabs.setCurrentIndex(index)
            dashboard.process_events()
            plot_tab = dashboard.tabs.widget(index)
            canvas = dashboard.plot_canvases[label]
            buttons = plot_tab.findChildren(QtWidgets.QPushButton)
            assert [button.text() for button in buttons] == ["清空曲线", "保存当前图"]
            assert plot_tab.isAncestorOf(canvas)
            assert not dashboard.control_scroll.isAncestorOf(canvas)

            for widget in (canvas, *buttons):
                top_left = widget.mapTo(plot_tab, widget.rect().topLeft())
                widget_rect = QtCore.QRect(top_left, widget.size())
                assert plot_tab.rect().contains(widget_rect)
    finally:
        dashboard.close()


def test_dashboard_plot_text_bounds_stay_inside_real_narrow_canvas(monkeypatch):
    """正常窄侧栏中，标题、轴标签和图例都必须留在画布内。"""
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        for label, axis in dashboard.plot_axes.items():
            index = next(index for index in _dashboard_plot_tab_indices(dashboard) if dashboard.tabs.tabText(index) == label)
            dashboard.tabs.setCurrentIndex(index)
            dashboard.process_events()
            canvas = dashboard.plot_canvases[label]
            _assert_plot_artists_inside_canvas(label, axis, canvas)
    finally:
        dashboard.close()


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


def test_dashboard_updates_only_the_visible_plot_axis(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.tabs.setCurrentIndex(next(index for index in _dashboard_plot_tab_indices(dashboard) if dashboard.tabs.tabText(index) == "轨迹"))
        dashboard.process_events()
        relim_counts = {label: 0 for label in dashboard.plot_axes}
        for label, axis in dashboard.plot_axes.items():
            axis.relim = lambda label=label: relim_counts.__setitem__(label, relim_counts[label] + 1)
            axis.autoscale_view = lambda **_kwargs: None
        for canvas in dashboard.plot_canvases.values():
            canvas.draw_idle = lambda: None

        dashboard.update(RobotTelemetry(t=0.0, x=1.0, y=0.0))

        assert relim_counts == {"轨迹": 1, "速度/命令": 0, "打滑": 0, "接触": 0}
    finally:
        dashboard.close()


def test_dashboard_caps_visible_plot_redraw_requests_at_two_hz(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    now = {"value": 100.0}
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: now["value"])
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8, plot_update_hz=5.0)
    try:
        dashboard.tabs.setCurrentIndex(next(index for index in _dashboard_plot_tab_indices(dashboard) if dashboard.tabs.tabText(index) == "轨迹"))
        dashboard.process_events()
        draw_count = {"value": 0}
        dashboard.plot_canvases["轨迹"].draw_idle = lambda: draw_count.__setitem__("value", draw_count["value"] + 1)

        for timestamp in (100.0, 100.21, 100.42):
            now["value"] = timestamp
            dashboard.update(RobotTelemetry(t=timestamp - 100.0, x=timestamp, y=0.0))

        assert draw_count["value"] == 1

        now["value"] = 100.5
        dashboard.update(RobotTelemetry(t=0.5, x=100.5, y=0.0))
        assert draw_count["value"] == 2
    finally:
        dashboard.close()


def test_dashboard_drops_redraw_request_while_qt_canvas_is_pending(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.tabs.setCurrentIndex(next(index for index in _dashboard_plot_tab_indices(dashboard) if dashboard.tabs.tabText(index) == "轨迹"))
        dashboard.process_events()
        canvas = dashboard.plot_canvases["轨迹"]
        draw_count = {"value": 0}
        canvas.draw_idle = lambda: draw_count.__setitem__("value", draw_count["value"] + 1)
        canvas._draw_pending = True

        dashboard.update(RobotTelemetry(t=0.0, x=1.0, y=0.0))

        assert draw_count["value"] == 0
        assert "轨迹" in dashboard._plot_dirty_tabs
    finally:
        dashboard.close()


def test_dashboard_slow_draw_completion_extends_redraw_cooldown(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    now = {"value": 100.0}
    monkeypatch.setattr(dashboard_module.time, "monotonic", lambda: now["value"])
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8, plot_update_hz=5.0)
    try:
        dashboard.tabs.setCurrentIndex(next(index for index in _dashboard_plot_tab_indices(dashboard) if dashboard.tabs.tabText(index) == "轨迹"))
        dashboard.process_events()
        canvas = dashboard.plot_canvases["轨迹"]
        draw_count = {"value": 0}
        canvas.draw_idle = lambda: draw_count.__setitem__("value", draw_count["value"] + 1)

        dashboard.update(RobotTelemetry(t=0.0, x=1.0, y=0.0))
        now["value"] = 100.4
        dashboard._record_plot_draw("轨迹")

        now["value"] = 100.9
        dashboard.update(RobotTelemetry(t=0.9, x=1.1, y=0.0))
        assert draw_count["value"] == 1

        now["value"] = 101.21
        dashboard.update(RobotTelemetry(t=1.21, x=1.2, y=0.0))
        assert draw_count["value"] == 2
    finally:
        dashboard.close()


def test_plot_draw_cooldown_leaves_control_time_after_a_slow_draw():
    assert dashboard_module.plot_draw_cooldown_sec(draw_duration_sec=0.05, plot_update_hz=5.0) == pytest.approx(0.5)
    assert dashboard_module.plot_draw_cooldown_sec(draw_duration_sec=0.4, plot_update_hz=5.0) == pytest.approx(0.8)


def test_dashboard_plot_figures_do_not_recompute_tight_layout_on_every_draw(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        assert all(figure.get_layout_engine() is None for figure in dashboard.plot_figures.values())
    finally:
        dashboard.close()


def test_dashboard_clear_plots_redraws_only_the_visible_canvas(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        dashboard.tabs.setCurrentIndex(next(index for index in _dashboard_plot_tab_indices(dashboard) if dashboard.tabs.tabText(index) == "速度/命令"))
        dashboard.process_events()
        draw_counts = {label: 0 for label in dashboard.plot_canvases}
        for label, canvas in dashboard.plot_canvases.items():
            canvas.draw_idle = lambda label=label: draw_counts.__setitem__(label, draw_counts[label] + 1)

        dashboard.clear_plots()

        assert draw_counts == {"轨迹": 0, "速度/命令": 1, "打滑": 0, "接触": 0}
    finally:
        dashboard.close()


def test_dashboard_controls_still_work_after_each_plot_tab_is_selected(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtCore, QtGui, QtWidgets

    dashboard = TelemetryDashboard(max_linear_speed=0.4, max_angular_speed=0.8)
    try:
        up_button = next(button for button in dashboard.window.findChildren(QtWidgets.QPushButton) if button.text() == "↑")
        for index in _dashboard_plot_tab_indices(dashboard):
            dashboard.tabs.setCurrentIndex(index)
            canvas = dashboard.plot_canvases[dashboard.tabs.tabText(index)]
            press = QtGui.QKeyEvent(QtCore.QEvent.KeyPress, QtCore.Qt.Key_Up, QtCore.Qt.NoModifier)
            dashboard.app.sendEvent(canvas, press)
            assert dashboard.current_command().linear_velocity == pytest.approx(0.4)

            release = QtGui.QKeyEvent(QtCore.QEvent.KeyRelease, QtCore.Qt.Key_Up, QtCore.Qt.NoModifier)
            dashboard.app.sendEvent(canvas, release)
            up_button.pressed.emit()
            assert dashboard.current_command().linear_velocity == pytest.approx(0.4)
            up_button.released.emit()
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
