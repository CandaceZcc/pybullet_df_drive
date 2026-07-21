# 日志测试：确保 CSV 输出字段稳定，并单独记录阶段二障碍物事件。
import json
from pathlib import Path

import pandas as pd
import pytest

import slope_sim.logger as logger_module
from slope_sim.logger import CsvSimulationLogger
from slope_sim.robot import RobotState
from slope_sim.runtime_actions import TerrainSelection


def test_csv_logger_writes_robot_state_rows(tmp_path: Path):
    logger = CsvSimulationLogger(tmp_path, prefix="unit")
    state = RobotState(
        t=0.5,
        x=1.0,
        y=2.0,
        z=0.3,
        roll=0.1,
        pitch=0.2,
        yaw=0.4,
        linear_velocity=0.6,
        angular_velocity=0.7,
        vx=0.3,
        vy=0.4,
        vz=0.0,
        velocity_sensor_vx=0.3,
        velocity_sensor_vy=0.4,
        velocity_sensor_vz=0.0,
        velocity_sensor_body_forward_speed=0.3,
        velocity_sensor_yaw_rate=0.7,
        linear_acceleration_x=0.8,
        linear_acceleration_y=0.1,
        linear_acceleration_z=-0.2,
        angular_acceleration_z=0.05,
        body_forward_speed=0.3,
        yaw_rate=0.7,
        left_target_drive_speed=6.0,
        right_target_drive_speed=7.0,
        left_actual_drive_speed=5.5,
        right_actual_drive_speed=6.5,
        front_left_actual_drive_speed=5.0,
        front_right_actual_drive_speed=5.5,
        rear_left_actual_drive_speed=6.0,
        rear_right_actual_drive_speed=6.5,
        front_left_actual_steering_angle=0.1,
        front_right_actual_steering_angle=0.12,
        left_track_surface_speed=0.44,
        right_track_surface_speed=0.52,
        left_body_track_speed=0.40,
        right_body_track_speed=0.50,
        left_slip_ratio=0.1,
        right_slip_ratio=0.2,
        ground_rolling_friction=0.03,
        ground_spinning_friction=0.04,
        support_lateral_friction=0.02,
        track_anisotropic_friction_x=2.0,
        track_anisotropic_friction_y=0.05,
        track_anisotropic_friction_z=0.05,
        left_slip_speed=0.03,
        right_slip_speed=0.04,
        left_slip_valid=True,
        right_slip_valid=False,
        left_contact_friction_force=1.5,
        right_contact_friction_force=1.6,
        left_contact_count=2,
        right_contact_count=3,
        terrain_type="slope",
        local_ground_height=0.2,
        local_terrain_normal_x=-0.087,
        local_terrain_normal_y=0.0,
        local_terrain_normal_z=0.996,
        lidar_min_distance=2.5,
    )

    logger.record(state, reference_x=1.5, reference_y=2.5, estimated_x=0.9, estimated_y=1.9)
    log_path = logger.close()

    frame = pd.read_csv(log_path)
    assert {"t", "x", "y", "z", "roll", "pitch", "yaw"}.issubset(frame.columns)
    assert {"vx", "vy", "vz", "body_forward_speed", "yaw_rate"}.issubset(frame.columns)
    assert {
        "velocity_sensor_vx",
        "velocity_sensor_body_forward_speed",
        "velocity_sensor_yaw_rate",
        "linear_acceleration_x",
        "angular_acceleration_z",
    }.issubset(frame.columns)
    assert {"left_actual_drive_speed", "right_actual_drive_speed", "lidar_min_distance"}.issubset(frame.columns)
    assert {
        "front_left_actual_drive_speed",
        "front_right_actual_drive_speed",
        "rear_left_actual_drive_speed",
        "rear_right_actual_drive_speed",
        "front_left_actual_steering_angle",
        "front_right_actual_steering_angle",
    }.issubset(frame.columns)
    assert {"left_track_surface_speed", "right_body_track_speed"}.issubset(frame.columns)
    assert {"ground_rolling_friction", "ground_spinning_friction", "support_lateral_friction"}.issubset(frame.columns)
    assert {"track_anisotropic_friction_x", "track_anisotropic_friction_y", "track_anisotropic_friction_z"}.issubset(frame.columns)
    assert {"left_slip_speed", "right_slip_speed", "left_slip_valid", "right_slip_valid"}.issubset(frame.columns)
    assert {"left_contact_friction_force", "right_contact_friction_force"}.issubset(frame.columns)
    assert {"left_contact_count", "right_contact_count"}.issubset(frame.columns)
    assert {"terrain_type", "local_ground_height", "local_terrain_normal_x", "local_terrain_normal_z"}.issubset(frame.columns)
    assert frame.iloc[0]["x"] == 1.0
    assert frame.iloc[0]["reference_y"] == 2.5
    assert frame.iloc[0]["left_track_surface_speed"] == 0.44
    assert frame.iloc[0]["velocity_sensor_body_forward_speed"] == 0.3
    assert frame.iloc[0]["ground_rolling_friction"] == 0.03
    assert frame.iloc[0]["left_slip_ratio"] == 0.1
    assert frame.iloc[0]["left_slip_speed"] == 0.03
    assert bool(frame.iloc[0]["right_slip_valid"]) is False
    assert frame.iloc[0]["left_contact_count"] == 2
    assert frame.iloc[0]["terrain_type"] == "slope"


def test_obstacle_event_logger_writes_stable_jsonl_fields_and_flushes(tmp_path: Path):
    """障碍物事件必须写独立 JSONL，避免把结构事件混入遥测 CSV。"""
    logger = logger_module.ObstacleEventLogger(tmp_path, prefix="obstacles")

    logger.record_event(
        sim_time=1.25,
        event_type="add",
        logical_id=7,
        request_params={"mode": "mixed", "count": 3, "shape": "box"},
        seed=42,
        robot_model="df_back",
        terrain=TerrainSelection("golf_heightfield", golf_seed=9, golf_relief="high"),
        success=False,
        error_reason="no valid placement",
    )
    path = logger.close()

    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert len(rows) == 1
    assert set(rows[0]) == {
        "sim_time",
        "event_type",
        "logical_id",
        "request_params",
        "seed",
        "robot_model",
        "terrain",
        "success",
        "error_reason",
    }
    assert rows[0]["sim_time"] == 1.25
    assert rows[0]["event_type"] == "add"
    assert rows[0]["logical_id"] == 7
    assert rows[0]["request_params"] == {"mode": "mixed", "count": 3, "shape": "box"}
    assert rows[0]["seed"] == 42
    assert rows[0]["robot_model"] == "df_back"
    assert rows[0]["terrain"] == {
        "terrain_model": "golf_heightfield",
        "slope_deg": 0.0,
        "golf_seed": 9,
        "golf_relief": "high",
    }
    assert rows[0]["success"] is False
    assert rows[0]["error_reason"] == "no valid placement"


def test_obstacle_event_logger_close_is_idempotent(tmp_path: Path):
    logger = logger_module.ObstacleEventLogger(tmp_path, prefix="obstacles")

    first_path = logger.close()
    second_path = logger.close()

    assert first_path == second_path
    assert first_path.exists()


def test_obstacle_event_logger_rejects_record_after_close(tmp_path: Path):
    logger = logger_module.ObstacleEventLogger(tmp_path, prefix="obstacles")
    logger.close()

    with pytest.raises(ValueError, match="closed"):
        logger.record_event(
            sim_time=0.0,
            event_type="add",
            logical_id=None,
            request_params={},
            seed=None,
            robot_model="df_back",
            terrain=TerrainSelection("flat"),
            success=True,
            error_reason=None,
        )
