# 遥测测试：保护车体系速度、打滑率和状态行格式。
import math

import pytest

from slope_sim.telemetry import RobotTelemetry, body_frame_velocity, slip_ratio, track_body_speeds


def test_body_frame_velocity_rotates_world_velocity_by_yaw():
    forward, lateral, vertical = body_frame_velocity((0.0, 1.0, 0.2), yaw=math.pi / 2.0)

    assert forward == pytest.approx(1.0)
    assert lateral == pytest.approx(0.0)
    assert vertical == pytest.approx(0.2)


def test_slip_ratio_is_zero_safe_and_signed():
    assert slip_ratio(drive_surface_speed=0.0, body_forward_speed=0.4) == 0.0
    assert slip_ratio(drive_surface_speed=1.0, body_forward_speed=0.8) == pytest.approx(0.2)
    assert slip_ratio(drive_surface_speed=-1.0, body_forward_speed=-0.8) == pytest.approx(-0.2)


def test_track_body_speeds_use_left_and_right_track_positions():
    left_speed, right_speed = track_body_speeds(body_forward_speed=0.4, yaw_rate=0.8, track_width=0.5)

    assert left_speed == pytest.approx(0.2)
    assert right_speed == pytest.approx(0.6)


def test_robot_telemetry_exports_csv_row_with_core_fields():
    telemetry = RobotTelemetry(
        t=1.0,
        x=2.0,
        y=3.0,
        z=0.4,
        pitch=0.1,
        left_track_surface_speed=0.8,
        right_body_track_speed=0.7,
        lidar_min_distance=5.0,
    )

    row = telemetry.to_row(reference_x=2.5, reference_y=3.5, estimated_x=2.1, estimated_y=3.1)

    assert row["t"] == 1.0
    assert row["x"] == 2.0
    assert row["pitch"] == 0.1
    assert row["left_track_surface_speed"] == 0.8
    assert row["right_body_track_speed"] == 0.7
    assert row["lidar_min_distance"] == 5.0
    assert row["reference_y"] == 3.5
