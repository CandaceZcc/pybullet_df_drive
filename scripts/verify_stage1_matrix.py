#!/usr/bin/env python3
# 阶段一 DIRECT 验证：快速遍历四种车型和三类场地，不依赖桌面环境。
from __future__ import annotations

import math
from pathlib import Path
import sys

import pybullet as p

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.model_registry import robot_model_names
from slope_sim.robot import DifferentialDriveRobot, create_robot
from slope_sim.scene import SceneInfo, create_slope_scene, terrain_model_names
from slope_sim.simulation import _probe_terrain_for_robot


MAX_STAGE1_TILT_RAD = math.radians(40.0)
MAX_CONTACT_PENETRATION_M = 0.02


def validate_robot_pose(
    client_id: int,
    robot: DifferentialDriveRobot,
    scene: SceneInfo,
    *,
    require_ground_contact: bool,
) -> None:
    """逐帧检查有限位姿、场地边界、翻转和异常穿透。"""
    position, orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
    roll, pitch, _yaw = p.getEulerFromQuaternion(orientation)
    pose_values = tuple(float(value) for value in (*position, *orientation, roll, pitch))
    if not all(math.isfinite(value) for value in pose_values):
        raise AssertionError(f"non-finite robot pose: {robot.model_spec.name}")
    if abs(roll) > MAX_STAGE1_TILT_RAD or abs(pitch) > MAX_STAGE1_TILT_RAD:
        raise AssertionError(f"robot tipped excessively: {robot.model_spec.name}/{scene.terrain_type}")

    terrain_probe = _probe_terrain_for_robot(client_id, robot, scene)
    if not terrain_probe.terrain_probe_valid:
        raise AssertionError(f"robot left terrain bounds: {robot.model_spec.name}/{scene.terrain_type}")

    contacts = [
        contact
        for body_id in scene.body_ids
        for contact in p.getContactPoints(
            bodyA=robot.robot_id,
            bodyB=body_id,
            physicsClientId=client_id,
        )
    ]
    if require_ground_contact and not contacts:
        raise AssertionError(f"robot has no ground contact: {robot.model_spec.name}/{scene.terrain_type}")
    if contacts:
        deepest_penetration = min(float(contact[8]) for contact in contacts)
        if deepest_penetration < -MAX_CONTACT_PENETRATION_M:
            raise AssertionError(
                f"robot penetrated terrain: {robot.model_spec.name}/{scene.terrain_type}: "
                f"{deepest_penetration:.4f} m"
            )


def verify_combination(client_id: int, robot_model: str, terrain_model: str) -> tuple[float, float, float, float]:
    """加载一个组合，先静置再前进，检查落地、姿态和基本驱动响应。"""
    slope_deg = 8.0 if terrain_model == "slope" else 0.0
    scene = create_slope_scene(
        client_id,
        slope_deg=slope_deg,
        time_step=1.0 / 240.0,
        terrain_model=terrain_model,
        golf_seed=37,
        golf_relief="medium",
    )
    robot = create_robot(
        client_id,
        robot_model,
        start_x=scene.spawn_position[0],
        start_y=scene.spawn_position[1],
        base_height=scene.spawn_position[2] + create_robot_base_height(robot_model),
        start_orientation=scene.spawn_orientation,
    )
    robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
    validate_robot_pose(client_id, robot, scene, require_ground_contact=False)
    for settle_step in range(120):
        robot.command_twist(0.0, 0.0, dt=1.0 / 240.0)
        p.stepSimulation(physicsClientId=client_id)
        validate_robot_pose(client_id, robot, scene, require_ground_contact=settle_step >= 30)
    initial_position, _ = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
    for _ in range(180):
        robot.command_twist(0.25, 0.0, dt=1.0 / 240.0)
        p.stepSimulation(physicsClientId=client_id)
        validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
    position, orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
    roll, pitch, _yaw = p.getEulerFromQuaternion(orientation)
    local_ground = _probe_terrain_for_robot(client_id, robot, scene)
    distance = math.hypot(float(position[0]) - float(initial_position[0]), float(position[1]) - float(initial_position[1]))
    clearance = float(position[2]) - local_ground.local_ground_height

    values = (distance, clearance, abs(float(roll)), abs(float(pitch)))
    if not all(math.isfinite(value) for value in values):
        raise AssertionError(f"non-finite state: {robot_model}/{terrain_model}: {values}")
    if distance < 0.05:
        raise AssertionError(f"robot did not move enough: {robot_model}/{terrain_model}: {distance:.3f} m")
    if not 0.05 <= clearance <= 0.45:
        raise AssertionError(
            f"invalid ground clearance: {robot_model}/{terrain_model}: {clearance:.3f} m "
            f"base_z={float(position[2]):.3f} ground_z={local_ground.local_ground_height:.3f}"
        )
    return values


def create_robot_base_height(robot_model: str) -> float:
    """延迟导入注册表高度，保持验证主流程易读。"""
    from slope_sim.model_registry import get_robot_model

    return get_robot_model(robot_model).base_height


def main() -> int:
    """运行 4×3 矩阵并打印便于阶段报告引用的结果。"""
    client_id = p.connect(p.DIRECT)
    if client_id < 0:
        raise RuntimeError("Failed to connect to PyBullet DIRECT")
    try:
        for terrain_model in terrain_model_names():
            for robot_model in robot_model_names():
                distance, clearance, roll, pitch = verify_combination(client_id, robot_model, terrain_model)
                print(
                    f"PASS {robot_model:22s} {terrain_model:16s} "
                    f"distance={distance:.3f} clearance={clearance:.3f} "
                    f"roll_deg={math.degrees(roll):.2f} pitch_deg={math.degrees(pitch):.2f}"
                )
    finally:
        p.disconnect(client_id)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
