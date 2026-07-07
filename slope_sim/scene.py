# 场景模块：创建平地或单一斜坡，并设置基础物理参数。
from __future__ import annotations

import math

import pybullet as p


def create_slope_scene(client_id: int, slope_deg: float, time_step: float) -> int:
    """创建一个可设置坡度的简单地面场景。"""
    p.resetSimulation(physicsClientId=client_id)
    p.setGravity(0.0, 0.0, -9.81, physicsClientId=client_id)
    p.setTimeStep(time_step, physicsClientId=client_id)

    slope_rad = math.radians(slope_deg)
    # 使用一个静态长方体作为地面；0 度时就是平地，非 0 度时就是斜坡。
    length = 12.0
    width = 4.0
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
    # 静态刚体质量为 0，用作不会被机器人推走的地面。
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[length / 2.0, 0.0, length * math.sin(slope_rad) / 2.0 - thickness / 2.0],
        baseOrientation=orientation,
        physicsClientId=client_id,
    )
    p.changeDynamics(body_id, -1, lateralFriction=1.0, rollingFriction=0.02, physicsClientId=client_id)
    return body_id
