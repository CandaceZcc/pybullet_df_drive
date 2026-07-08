# Dashboard 测试：只验证数据显示格式，不要求测试环境打开真实窗口。
import pytest

from slope_sim.dashboard import dashboard_rows, should_refresh_dashboard, smooth_telemetry
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
            left_actual_drive_speed=3.2,
            right_actual_drive_speed=3.4,
            left_track_surface_speed=0.26,
            right_track_surface_speed=0.27,
            left_body_track_speed=0.25,
            right_body_track_speed=0.30,
            left_slip_ratio=0.11,
            right_slip_ratio=0.12,
            lidar_min_distance=2.5,
            command_linear_velocity=0.6,
            command_angular_velocity=0.7,
        )
    )

    labels = {label: value for label, value in rows}
    assert labels["位置 x/y/z"] == "1.23 / -2.00 / 0.45 m"
    assert labels["姿态 roll/pitch/yaw"] == "5.7 / -11.5 / 17.2 deg"
    assert labels["车体速度 / yaw_rate"] == "0.40 m/s / 0.50 rad/s"
    assert labels["驱动表面速度"] == "0.26 / 0.27 m/s"
    assert labels["驱动局部车速"] == "0.25 / 0.30 m/s"
    assert labels["最近障碍距离"] == "2.50 m"


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
        command_linear_velocity=0.5,
    )

    smoothed = smooth_telemetry(previous, current, alpha=0.25)

    assert smoothed.left_actual_drive_speed == 5.0
    assert smoothed.right_actual_drive_speed == 6.0
    assert smoothed.left_track_surface_speed == 0.4
    assert smoothed.right_body_track_speed == pytest.approx(0.4)
    assert smoothed.left_contact_normal_force == 25.0
    assert smoothed.command_linear_velocity == 0.5
