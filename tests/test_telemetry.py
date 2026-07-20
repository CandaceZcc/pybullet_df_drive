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
    assert slip_ratio(drive_surface_speed=0.0, body_forward_speed=0.4) == pytest.approx(-1.0)
    assert slip_ratio(drive_surface_speed=1.0, body_forward_speed=0.8) == pytest.approx(0.2)
    assert slip_ratio(drive_surface_speed=-1.0, body_forward_speed=-0.8) == pytest.approx(-0.2)


def test_slip_ratio_uses_larger_reference_speed_and_clamps_spikes():
    assert slip_ratio(drive_surface_speed=0.01, body_forward_speed=0.4) == pytest.approx(-0.975)
    assert slip_ratio(drive_surface_speed=3.0, body_forward_speed=-3.0) == pytest.approx(1.0)


def test_slip_ratio_treats_both_low_speeds_as_invalid_zero():
    assert slip_ratio(drive_surface_speed=0.0, body_forward_speed=0.0) == 0.0
    assert slip_ratio(drive_surface_speed=0.02, body_forward_speed=-0.01) == 0.0


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
        velocity_sensor_vx=0.31,
        velocity_sensor_body_forward_speed=0.32,
        velocity_sensor_yaw_rate=0.12,
        linear_acceleration_x=0.5,
        linear_acceleration_z=-0.1,
        angular_acceleration_z=0.03,
        left_track_surface_speed=0.8,
        right_body_track_speed=0.7,
        ground_rolling_friction=0.03,
        ground_spinning_friction=0.04,
        support_lateral_friction=0.02,
        track_anisotropic_friction_x=2.0,
        track_anisotropic_friction_y=0.05,
        track_anisotropic_friction_z=0.05,
        left_slip_speed=-0.05,
        right_slip_valid=False,
        left_contact_friction_force=1.2,
        right_contact_count=2,
        terrain_type="slope",
        local_ground_height=0.2,
        local_terrain_normal_x=-0.087,
        local_terrain_normal_z=0.996,
        lidar_min_distance=5.0,
    )

    row = telemetry.to_row(reference_x=2.5, reference_y=3.5, estimated_x=2.1, estimated_y=3.1)

    assert row["t"] == 1.0
    assert row["x"] == 2.0
    assert row["pitch"] == 0.1
    assert row["velocity_sensor_vx"] == 0.31
    assert row["velocity_sensor_body_forward_speed"] == 0.32
    assert row["velocity_sensor_yaw_rate"] == 0.12
    assert row["linear_acceleration_x"] == 0.5
    assert row["linear_acceleration_z"] == -0.1
    assert row["angular_acceleration_z"] == 0.03
    assert row["left_track_surface_speed"] == 0.8
    assert row["right_body_track_speed"] == 0.7
    assert row["ground_rolling_friction"] == 0.03
    assert row["ground_spinning_friction"] == 0.04
    assert row["support_lateral_friction"] == 0.02
    assert row["track_anisotropic_friction_x"] == 2.0
    assert row["left_slip_speed"] == -0.05
    assert row["right_slip_valid"] is False
    assert row["left_contact_friction_force"] == 1.2
    assert row["right_contact_count"] == 2
    assert row["terrain_type"] == "slope"
    assert row["local_ground_height"] == 0.2
    assert row["local_terrain_normal_x"] == -0.087
    assert row["local_terrain_normal_z"] == 0.996
    assert row["lidar_min_distance"] == 5.0
    assert row["reference_y"] == 3.5
