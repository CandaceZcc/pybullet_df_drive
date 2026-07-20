# 仿真 smoke 测试：覆盖阶段一 4×3 组合、日志生成和两类控制器物理响应。
from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pandas as pd
import pybullet as p
import pytest

from scripts.verify_stage1_matrix import verify_combination
import scripts.verify_stage1_matrix as stage1_verifier
from slope_sim.config import ExperimentConfig
from slope_sim.model_registry import get_robot_model, robot_model_names
from slope_sim.robot import create_robot
import slope_sim.scene as scene_module
from slope_sim.scene import terrain_model_names
from slope_sim.simulation import _probe_terrain_for_robot, run_experiment


def test_run_experiment_direct_generates_log_and_figure(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="df_back",
            terrain_model="flat",
            slope_deg=0.0,
            duration_sec=0.2,
            time_step=1.0 / 120.0,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )
    assert result.log_path.exists()
    assert result.figure_path.exists()
    assert result.metrics["endpoint_error"] >= 0.0
    frame = pd.read_csv(result.log_path)
    assert len(frame) > 0
    assert set(frame["robot_model"]) == {"df_back"}
    assert set(frame["terrain_type"]) <= {"", "flat"}


@pytest.mark.parametrize("terrain_model", terrain_model_names())
@pytest.mark.parametrize("robot_model", robot_model_names())
def test_stage1_robot_terrain_matrix_stays_upright_and_moves(robot_model: str, terrain_model: str):
    client_id = p.connect(p.DIRECT)
    try:
        distance, clearance, roll, pitch = verify_combination(client_id, robot_model, terrain_model)
        assert distance >= 0.05
        assert 0.05 <= clearance <= 0.45
        assert roll < 0.7
        assert pitch < 0.7
    finally:
        p.disconnect(client_id)


def test_stage1_pose_validator_rejects_robot_without_ground_contact():
    """矩阵验收不能把尚未接触地面的悬空车辆判为通过。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = stage1_verifier.create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=1.0 / 240.0,
            terrain_model="flat",
        )
        robot = stage1_verifier.create_robot(client_id, "df_back", base_height=2.0)
        with pytest.raises(AssertionError, match="ground contact"):
            stage1_verifier.validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("robot_model", robot_model_names())
def test_all_robot_models_drive_from_upper_flat_across_ramp_to_lower_flat(robot_model: str):
    """四种车型必须真实驶过高位平台、下坡和低位平台。"""
    client_id = p.connect(p.DIRECT)
    try:
        time_step = 1.0 / 240.0
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=8.0,
            time_step=time_step,
            terrain_model="slope",
        )
        spec = get_robot_model(robot_model)
        robot = create_robot(
            client_id,
            robot_model,
            start_x=scene.spawn_position[0],
            start_y=scene.spawn_position[1],
            base_height=scene.spawn_position[2] + spec.base_height,
            start_orientation=scene.spawn_orientation,
        )
        robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
        for settle_step in range(120):
            robot.command_twist(0.0, 0.0, dt=time_step)
            p.stepSimulation(physicsClientId=client_id)
            stage1_verifier.validate_robot_pose(
                client_id,
                robot,
                scene,
                require_ground_contact=settle_step >= 30,
            )

        ramp_half_x = scene_module.SLOPE_RAMP_LENGTH * math.cos(math.radians(8.0)) / 2.0
        samples: list[tuple[float, float]] = []
        entered_lower = False
        for _ in range(7200):
            robot.command_twist(0.7, 0.0, dt=time_step)
            p.stepSimulation(physicsClientId=client_id)
            stage1_verifier.validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
            position, orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
            x = float(position[0])
            pitch = float(p.getEulerFromQuaternion(orientation)[1])
            samples.append((x, pitch))
            if x > ramp_half_x + 0.8:
                entered_lower = True
                break

        final_x = samples[-1][0]
        upper_pitch = [abs(pitch) for x, pitch in samples if x < -ramp_half_x - 0.30]
        ramp_pitch = [abs(pitch) for x, pitch in samples if -ramp_half_x + 0.30 < x < ramp_half_x - 0.30]
        lower_pitch = [abs(pitch) for x, pitch in samples if x > ramp_half_x + 0.30]
        assert entered_lower, f"{robot_model} did not enter lower flat: final_x={final_x:.3f}"
        assert upper_pitch, f"{robot_model} had no upper-flat pitch samples"
        assert ramp_pitch, f"{robot_model} had no ramp pitch samples"
        assert lower_pitch, f"{robot_model} had no lower-flat pitch samples"
        assert math.degrees(sum(upper_pitch) / len(upper_pitch)) < 2.0
        assert math.degrees(max(ramp_pitch)) > 5.0
        assert math.degrees(sum(lower_pitch) / len(lower_pitch)) < 2.0
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(
    ("linear_velocity", "angular_velocity", "expected_x_sign", "expected_yaw_sign"),
    [
        (0.3, 0.0, 1, 0),
        (-0.3, 0.0, -1, 0),
        (0.3, 0.7, 1, 1),
        (0.3, -0.7, 1, -1),
        (0.0, 0.7, 0, 1),
    ],
)
@pytest.mark.parametrize("robot_model", ["df_front", "df_mid", "df_back"])
def test_differential_models_cover_stage1_motion_matrix(
    robot_model: str,
    linear_velocity: float,
    angular_velocity: float,
    expected_x_sign: int,
    expected_yaw_sign: int,
):
    """三种差速布局都自动覆盖前进、后退、左右转和差速转向。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = stage1_verifier.create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=1.0 / 240.0,
            terrain_model="flat",
        )
        robot = stage1_verifier.create_robot(
            client_id,
            robot_model,
            start_x=scene.spawn_position[0],
            start_y=scene.spawn_position[1],
            base_height=scene.spawn_position[2] + stage1_verifier.create_robot_base_height(robot_model),
            start_orientation=scene.spawn_orientation,
        )
        robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
        for _ in range(120):
            robot.command_twist(0.0, 0.0, dt=1.0 / 240.0)
            p.stepSimulation(physicsClientId=client_id)
        start_position, start_orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
        start_drive_axle, _ = p.multiplyTransforms(
            start_position,
            start_orientation,
            (robot.drive_center_x, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        start_yaw = p.getEulerFromQuaternion(start_orientation)[2]

        for _ in range(360):
            robot.command_twist(linear_velocity, angular_velocity, dt=1.0 / 240.0)
            p.stepSimulation(physicsClientId=client_id)
            stage1_verifier.validate_robot_pose(client_id, robot, scene, require_ground_contact=True)

        end_position, end_orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
        end_drive_axle, _ = p.multiplyTransforms(
            end_position,
            end_orientation,
            (robot.drive_center_x, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        yaw_delta = p.getEulerFromQuaternion(end_orientation)[2] - start_yaw
        if expected_x_sign:
            assert expected_x_sign * (float(end_position[0]) - float(start_position[0])) > 0.20
        else:
            drive_axle_displacement = math.hypot(
                float(end_drive_axle[0]) - float(start_drive_axle[0]),
                float(end_drive_axle[1]) - float(start_drive_axle[1]),
            )
            assert drive_axle_displacement < 0.08
        if expected_yaw_sign:
            assert expected_yaw_sign * yaw_delta > 0.50
        else:
            assert abs(yaw_delta) < 0.05
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("robot_model", ["df_front", "df_mid", "df_back"])
def test_differential_models_turn_in_place_on_flat(robot_model: str, tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model=robot_model,
            drive_model="physics",
            terrain_model="flat",
            slope_deg=0.0,
            duration_sec=1.2,
            time_step=1.0 / 240.0,
            target_linear_velocity=0.0,
            target_angular_velocity=0.7,
            log_dir=tmp_path / f"{robot_model}_logs",
            figure_dir=tmp_path / f"{robot_model}_figures",
        )
    )
    frame = pd.read_csv(result.log_path)
    assert frame["yaw"].iloc[-1] > 0.35
    assert frame.tail(100)["yaw_rate"].mean() > 0.35


def test_active_steering_4wd_forward_turn_has_drive_and_yaw_response(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="active_steering_4wd",
            drive_model="physics",
            terrain_model="flat",
            slope_deg=0.0,
            duration_sec=1.8,
            time_step=1.0 / 240.0,
            target_linear_velocity=0.35,
            target_angular_velocity=0.6,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )
    frame = pd.read_csv(result.log_path)
    assert frame["x"].iloc[-1] - frame["x"].iloc[0] > 0.25
    assert frame["yaw"].iloc[-1] > 0.15
    assert frame.tail(120)["body_forward_speed"].mean() > 0.15


def test_physics_log_keeps_existing_internal_diagnostics(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="df_mid",
            drive_model="physics",
            terrain_model="golf_heightfield",
            golf_seed=5,
            golf_relief="low",
            duration_sec=0.4,
            lidar_enabled=True,
            lidar_ray_count=9,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )
    frame = pd.read_csv(result.log_path)
    expected = {
        "left_actual_drive_speed",
        "right_actual_drive_speed",
        "velocity_sensor_body_forward_speed",
        "local_ground_height",
        "local_terrain_normal_z",
        "lidar_min_distance",
    }
    assert expected.issubset(frame.columns)
    assert frame["terrain_probe_valid"].all()
    assert set(frame["terrain_type"]) == {"golf_heightfield"}


def test_robot_terrain_probe_filters_obstacle_on_offset_ray():
    """运行时遥测的侧向探测线被障碍物覆盖时，仍应采到真实地形。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        robot_id = p.createMultiBody(
            baseMass=0.0,
            basePosition=(0.0, 0.0, 0.20),
            physicsClientId=client_id,
        )
        collision_shape_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=(0.25, 0.25, 0.30),
            physicsClientId=client_id,
        )
        obstacle_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=(0.0, 0.45, 0.30),
            physicsClientId=client_id,
        )
        first_hit = p.rayTest((0.0, 0.45, 2.0), (0.0, 0.45, -2.0), physicsClientId=client_id)[0]

        probe = _probe_terrain_for_robot(client_id, SimpleNamespace(robot_id=robot_id), scene)

        assert first_hit[0] == obstacle_id
        assert probe.terrain_probe_valid is True
        assert probe.local_ground_height == pytest.approx(0.0, abs=1e-6)
        assert probe.local_terrain_normal_z == pytest.approx(1.0, abs=1e-6)
    finally:
        p.disconnect(client_id)
