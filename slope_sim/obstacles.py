# 障碍物基础模块：创建质量为零的箱体，并在物理步进前同步运动学位姿与速度。
from __future__ import annotations

import math

import pybullet as p


QUATERNION_NORM_EPSILON = 1e-12


def _require_finite_vector(name: str, values: tuple[float, ...], *, length: int) -> tuple[float, ...]:
    """把 PyBullet 向量参数规范为浮点数，并在进入物理引擎前拒绝非法值。"""
    if len(values) != length:
        raise ValueError(f"{name} must contain {length} values")
    normalized = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in normalized):
        raise ValueError(f"{name} must contain only finite values")
    return normalized


def _normalize_orientation(orientation: tuple[float, ...]) -> tuple[float, float, float, float]:
    """拒绝会产生 NaN 的退化四元数，并在调用 PyBullet 前统一为单位四元数。"""
    quaternion = _require_finite_vector("orientation", orientation, length=4)
    norm = math.hypot(*quaternion)
    if not math.isfinite(norm):
        raise ValueError("orientation norm must be finite")
    if norm <= QUATERNION_NORM_EPSILON:
        raise ValueError("orientation norm must be greater than zero")
    return tuple(value / norm for value in quaternion)


def _body_ids(client_id: int) -> set[int]:
    """读取当前客户端刚体集合，供创建失败时只清理本次新增对象。"""
    return {
        p.getBodyUniqueId(index, physicsClientId=client_id)
        for index in range(p.getNumBodies(physicsClientId=client_id))
    }


def create_box_obstacle(
    client_id: int,
    *,
    half_extents: tuple[float, float, float],
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    color: tuple[float, float, float, float] = (0.85, 0.32, 0.12, 1.0),
) -> int:
    """创建带碰撞和可视形状的质量零箱体，异常时删除本次产生的半成品刚体。"""
    box_half_extents = _require_finite_vector("half_extents", half_extents, length=3)
    if any(value <= 0.0 for value in box_half_extents):
        raise ValueError("half_extents must be positive")
    base_position = _require_finite_vector("position", position, length=3)
    base_orientation = _normalize_orientation(orientation)
    rgba_color = _require_finite_vector("color", color, length=4)
    existing_body_ids = _body_ids(client_id)
    collision_shape_id: int | None = None

    try:
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=box_half_extents,
            physicsClientId=client_id,
        )
        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=box_half_extents,
            rgbaColor=rgba_color,
            physicsClientId=client_id,
        )
        return p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
            basePosition=base_position,
            baseOrientation=base_orientation,
            physicsClientId=client_id,
        )
    except Exception:
        # 创建调用可能在抛错前已注册 body，按差集清理可避免污染仍在运行的场景。
        for body_id in _body_ids(client_id) - existing_body_ids:
            p.removeBody(body_id, physicsClientId=client_id)
        if collision_shape_id is not None:
            p.removeCollisionShape(collision_shape_id, physicsClientId=client_id)
        # PyBullet 没有独立 visual-shape 删除 API；未绑定的 visual 只能随 resetSimulation 清理。
        raise


def update_kinematic_obstacle(
    client_id: int,
    body_id: int,
    *,
    position: tuple[float, float, float],
    linear_velocity: tuple[float, float, float],
    orientation: tuple[float, float, float, float] | None = None,
) -> None:
    """在 stepSimulation 前写入受控位姿和路径切向速度，使碰撞不能反推障碍物轨迹。"""
    base_position = _require_finite_vector("position", position, length=3)
    tangent_velocity = _require_finite_vector("linear_velocity", linear_velocity, length=3)
    if orientation is None:
        _, current_orientation = p.getBasePositionAndOrientation(body_id, physicsClientId=client_id)
        base_orientation = _normalize_orientation(tuple(current_orientation))
    else:
        base_orientation = _normalize_orientation(orientation)

    # 质量为零的 body 不受求解器反推；每帧重置则显式维持规划轨迹和接触表面速度。
    p.resetBasePositionAndOrientation(
        body_id,
        base_position,
        base_orientation,
        physicsClientId=client_id,
    )
    p.resetBaseVelocity(
        body_id,
        linearVelocity=tangent_velocity,
        angularVelocity=(0.0, 0.0, 0.0),
        physicsClientId=client_id,
    )
