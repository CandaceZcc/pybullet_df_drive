# 真值传感器 DIRECT 门禁：在四车型和三地形中对照 PyBullet 官方变换结果。
from __future__ import annotations

from dataclasses import dataclass
import math

import pybullet as p
import pytest

from scripts.verify_stage1_matrix import MAX_CONTACT_PENETRATION_M, validate_robot_pose
from slope_sim.model_registry import get_robot_model, robot_model_names
from slope_sim.robot import create_robot
from slope_sim.scene import create_slope_scene
from slope_sim.sensor_backend import PyBulletSensorBackend
from slope_sim.truth_sensors import SensorMounts, TruthSensorSuite, wrap_angle


TIME_STEP = 1.0 / 240.0
STABLE_CONTACT_FRAMES = 60
MAX_SETTLED_LINEAR_SPEED_MPS = 0.05
MAX_SETTLED_ANGULAR_SPEED_RAD_S = 0.20


@dataclass(frozen=True)
class TruthGateResult:
    """三地形门禁的四项真值误差。"""

    rtk_position_error_m: float
    rtk_yaw_error_rad: float
    imu_roll_error_rad: float
    imu_pitch_error_rad: float


def _angle_error(actual: float, expected: float) -> float:
    return abs(wrap_angle(actual - expected))


def _world_base_link_pose(client_id: int, robot_id: int):
    """按 Bullet 的惯性 frame 关系独立恢复 URDF base_link 世界位姿。"""
    world_inertial_position, world_inertial_orientation = p.getBasePositionAndOrientation(
        robot_id,
        physicsClientId=client_id,
    )
    dynamics = p.getDynamicsInfo(robot_id, -1, physicsClientId=client_id)
    inverse_position, inverse_orientation = p.invertTransform(
        dynamics[3],
        dynamics[4],
    )
    return p.multiplyTransforms(
        world_inertial_position,
        world_inertial_orientation,
        inverse_position,
        inverse_orientation,
    )


def run_truth_sensor_gate(robot_model: str, terrain_model: str) -> TruthGateResult:
    """创建独立 DIRECT client，按 URDF frame 比较真值并保证断开连接。"""
    client_id = p.connect(p.DIRECT)
    result = None
    try:
        slope_deg = 8.0 if terrain_model == "slope" else 0.0
        scene = create_slope_scene(
            client_id,
            slope_deg=slope_deg,
            time_step=TIME_STEP,
            terrain_model=terrain_model,
            golf_seed=31,
            golf_relief="medium",
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
        robot.apply_drive_friction(
            lateral_friction=1.4,
            support_lateral_friction=0.03,
        )
        zero_drive = (0.0,) * len(spec.drive_joint_names)
        zero_steering = (0.0,) * len(spec.steering_joint_names)
        # 先让车辆按真实接触稳定，再读取当前场地姿态，避免悬空硬编码姿态门禁。
        stable_contact_streak = 0
        max_linear_speed = 0.0
        max_angular_speed = 0.0
        max_penetration = 0.0
        stable_start = 180 - STABLE_CONTACT_FRAMES
        for settle_step in range(180):
            robot.command_wheel_speeds(
                zero_drive,
                zero_steering,
                dt=TIME_STEP,
            )
            p.stepSimulation(physicsClientId=client_id)
            validate_robot_pose(
                client_id,
                robot,
                scene,
                require_ground_contact=settle_step >= stable_start,
            )
            if settle_step >= stable_start:
                contacts = [
                    contact
                    for terrain_body_id in scene.body_ids
                    for contact in p.getContactPoints(
                        bodyA=robot.robot_id,
                        bodyB=terrain_body_id,
                        physicsClientId=client_id,
                    )
                ]
                stable_contact_streak = stable_contact_streak + 1 if contacts else 0
                linear_velocity, angular_velocity = p.getBaseVelocity(
                    robot.robot_id,
                    physicsClientId=client_id,
                )
                velocity_values = tuple(
                    float(value) for value in (*linear_velocity, *angular_velocity)
                )
                assert all(math.isfinite(value) for value in velocity_values)
                max_linear_speed = max(
                    max_linear_speed,
                    math.sqrt(sum(float(value) ** 2 for value in linear_velocity)),
                )
                max_angular_speed = max(
                    max_angular_speed,
                    math.sqrt(sum(float(value) ** 2 for value in angular_velocity)),
                )
                max_penetration = max(
                    max_penetration,
                    max(max(0.0, -float(contact[8])) for contact in contacts),
                )
        assert stable_contact_streak >= STABLE_CONTACT_FRAMES
        assert max_linear_speed <= MAX_SETTLED_LINEAR_SPEED_MPS
        assert max_angular_speed <= MAX_SETTLED_ANGULAR_SPEED_RAD_S
        assert max_penetration <= MAX_CONTACT_PENETRATION_M
        mounts = SensorMounts.default()
        suite = TruthSensorSuite(PyBulletSensorBackend(client_id, robot.robot_id), mounts)
        rtk = suite.read_rtk(123_000_000)
        imu = suite.read_imu(123_000_000)

        base_position, base_orientation = _world_base_link_pose(client_id, robot.robot_id)
        primary_position, _primary_orientation = p.multiplyTransforms(
            base_position,
            base_orientation,
            mounts.rtk_primary.position,
            mounts.rtk_primary.orientation,
        )
        secondary_position, _secondary_orientation = p.multiplyTransforms(
            base_position,
            base_orientation,
            mounts.rtk_secondary.position,
            mounts.rtk_secondary.orientation,
        )
        _imu_position, imu_orientation = p.multiplyTransforms(
            base_position,
            base_orientation,
            mounts.imu.position,
            mounts.imu.orientation,
        )
        expected_roll, expected_pitch, _expected_yaw = p.getEulerFromQuaternion(imu_orientation)
        expected_rtk_yaw = wrap_angle(
            math.atan2(
                secondary_position[1] - primary_position[1],
                secondary_position[0] - primary_position[0],
            )
        )
        result = TruthGateResult(
            rtk_position_error_m=math.dist(
                (rtk.main_x, rtk.main_y, rtk.main_z),
                primary_position,
            ),
            rtk_yaw_error_rad=_angle_error(rtk.baseline_yaw_rad, expected_rtk_yaw),
            imu_roll_error_rad=_angle_error(imu.roll_rad, expected_roll),
            imu_pitch_error_rad=_angle_error(imu.pitch_rad, expected_pitch),
        )
    finally:
        p.disconnect(client_id)

    assert p.isConnected(client_id) == 0
    assert result is not None
    return result


@pytest.mark.parametrize("robot_model", robot_model_names())
def test_default_mount_parent_links_exist_on_every_deliverable_model(robot_model):
    client_id = p.connect(p.DIRECT)
    try:
        robot = create_robot(client_id, robot_model)
        backend = PyBulletSensorBackend(client_id, robot.robot_id)

        suite = TruthSensorSuite(backend, SensorMounts.default())

        assert suite.mounts == SensorMounts.default()
    finally:
        p.disconnect(client_id)
    assert p.isConnected(client_id) == 0


@pytest.mark.parametrize("robot_model", robot_model_names())
@pytest.mark.parametrize("terrain_model", ("flat", "slope", "golf_heightfield"))
def test_direct_rtk_and_imu_match_pybullet_truth_within_1e_4(
    robot_model,
    terrain_model,
):
    result = run_truth_sensor_gate(robot_model, terrain_model)

    assert result.rtk_position_error_m <= 1e-4
    assert result.rtk_yaw_error_rad <= 1e-4
    assert result.imu_roll_error_rad <= 1e-4
    assert result.imu_pitch_error_rad <= 1e-4
