# 场景模块：创建平地或单一斜坡，并设置基础物理参数。
from __future__ import annotations

import math

import pybullet as p


def create_slope_scene(
    client_id: int,
    slope_deg: float,
    time_step: float,
    ground_lateral_friction: float = 1.0,
) -> int:
    """创建一个可设置坡度的简单地面场景。"""
    p.resetSimulation(physicsClientId=client_id)
    p.setGravity(0.0, 0.0, -9.81, physicsClientId=client_id)
    p.setTimeStep(time_step, physicsClientId=client_id)

    slope_rad = math.radians(slope_deg)
    # 使用一个静态长方体作为地面；0 度时就是平地，非 0 度时就是斜坡。
    length = 64.0
    width = 32.0
    thickness = 0.08
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[length / 2.0, width / 2.0, thickness / 2.0],
        physicsClientId=client_id,
    )
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[length / 2.0, width / 2.0, thickness / 2.0],
        rgbaColor=[0.35, 0.55, 0.28, 1.0],
        physicsClientId=client_id,
    )
    orientation = p.getQuaternionFromEuler([0.0, -slope_rad, 0.0])
    # 静态刚体质量为 0，用作不会被机器人推走的地面；地面居中放置，避免机器人出生在边缘。
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[0.0, 0.0, -thickness / 2.0],
        baseOrientation=orientation,
        physicsClientId=client_id,
    )
    p.changeDynamics(body_id, -1, lateralFriction=ground_lateral_friction, rollingFriction=0.02, physicsClientId=client_id)
    return body_id


def configure_gui_visualizer(
    client_id: int,
    camera_distance: float,
    camera_yaw: float,
    camera_pitch: float,
    camera_target: tuple[float, float, float],
) -> None:
    """放大 PyBullet 主视图并关闭默认的小预览窗口。"""
    for flag in (
        p.COV_ENABLE_RGB_BUFFER_PREVIEW,
        p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
        p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
    ):
        p.configureDebugVisualizer(flag, 0, physicsClientId=client_id)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=client_id)
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=camera_target,
        physicsClientId=client_id,
    )
