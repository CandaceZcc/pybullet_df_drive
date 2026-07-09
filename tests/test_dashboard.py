# Dashboard 测试：只验证数据显示格式，不要求测试环境打开真实窗口。
import pytest

from slope_sim.dashboard import dashboard_groups, dashboard_rows, dashboard_window_size, should_refresh_dashboard, smooth_telemetry
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
    assert labels["签名打滑率"] == "+0.11 / 低速"
    assert labels["打滑速度差"] == "+0.01 / -0.03 m/s"
    assert labels["接触摩擦力"] == "1.20 / 1.40 N"
    assert labels["有效接触点"] == "2 / 3"
    assert labels["地形类型"] == "twr_slope_5deg"
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


def test_dashboard_window_size_clamps_to_available_screen():
    assert dashboard_window_size(available_width=900, available_height=700) == (405, 630)
    assert dashboard_window_size(available_width=5000, available_height=3000) == (520, 1200)
    assert dashboard_window_size(available_width=600, available_height=320) == (320, 288)


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
