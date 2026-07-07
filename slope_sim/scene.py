from __future__ import annotations

import math

import pybullet as p


def create_slope_scene(client_id: int, slope_deg: float, time_step: float) -> int:
    p.resetSimulation(physicsClientId=client_id)
    p.setGravity(0.0, 0.0, -9.81, physicsClientId=client_id)
    p.setTimeStep(time_step, physicsClientId=client_id)

    slope_rad = math.radians(slope_deg)
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

