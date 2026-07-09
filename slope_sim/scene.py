# 场景模块：创建平地、简单斜坡或参考仓库坡面，并设置基础物理参数。
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import pybullet as p

from slope_sim.telemetry import TerrainProbe


TWR_SLOPE_URDF = Path("urdf/terrain/twr_slope_5deg.urdf")
TWR_SLOPE_SCALE = 2.5
TWR_SLOPE_BASE_POSITION = (6.0, 0.0, -1.194)


@dataclass(frozen=True)
class SceneInfo:
    """当前仿真场景的核心信息，供日志和地形探测使用。"""

    body_id: int
    terrain_type: str
    slope_deg: float


def create_slope_scene(
    client_id: int,
    slope_deg: float,
    time_step: float,
    ground_lateral_friction: float = 1.0,
    ground_rolling_friction: float = 0.02,
    ground_spinning_friction: float = 0.0,
    terrain_model: str = "box_slope",
) -> SceneInfo:
    """创建一个地面场景，并返回地形元信息。"""
    p.resetSimulation(physicsClientId=client_id)
    p.setGravity(0.0, 0.0, -9.81, physicsClientId=client_id)
    p.setTimeStep(time_step, physicsClientId=client_id)

    if terrain_model == "twr_slope_5deg":
        if abs(slope_deg - 5.0) > 1e-9:
            raise ValueError("terrain_model 'twr_slope_5deg' requires slope_deg: 5.0")
        body_id = _create_twr_slope_scene(
            client_id,
            ground_lateral_friction,
            ground_rolling_friction,
            ground_spinning_friction,
        )
        return SceneInfo(body_id=body_id, terrain_type=terrain_model, slope_deg=slope_deg)
    if terrain_model != "box_slope":
        raise ValueError("terrain_model must be 'box_slope' or 'twr_slope_5deg'")
    body_id = _create_box_slope_scene(
        client_id,
        slope_deg,
        ground_lateral_friction,
        ground_rolling_friction,
        ground_spinning_friction,
    )
    return SceneInfo(body_id=body_id, terrain_type=terrain_model, slope_deg=slope_deg)


def _create_box_slope_scene(
    client_id: int,
    slope_deg: float,
    ground_lateral_friction: float,
    ground_rolling_friction: float,
    ground_spinning_friction: float,
) -> int:
    """创建项目原有的单个长方体坡面。"""
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
    _apply_terrain_friction(
        client_id,
        body_id,
        ground_lateral_friction,
        ground_rolling_friction,
        ground_spinning_friction,
    )
    return body_id


def _create_twr_slope_scene(
    client_id: int,
    ground_lateral_friction: float,
    ground_rolling_friction: float,
    ground_spinning_friction: float,
) -> int:
    """加载 Two-Wheel-Robot-DeepRL 风格的 5 度双坡 URDF。"""
    body_id = p.loadURDF(
        str(TWR_SLOPE_URDF),
        TWR_SLOPE_BASE_POSITION,
        [0.0, 0.0, 0.0, 1.0],
        useFixedBase=True,
        globalScaling=TWR_SLOPE_SCALE,
        physicsClientId=client_id,
    )
    _apply_terrain_friction(
        client_id,
        body_id,
        ground_lateral_friction,
        ground_rolling_friction,
        ground_spinning_friction,
    )
    return body_id


def _apply_terrain_friction(
    client_id: int,
    body_id: int,
    lateral_friction: float,
    rolling_friction: float,
    spinning_friction: float,
) -> None:
    """把地面摩擦参数应用到 base 和所有固定子 link。"""
    for link_index in range(-1, p.getNumJoints(body_id, physicsClientId=client_id)):
        p.changeDynamics(
            body_id,
            link_index,
            lateralFriction=lateral_friction,
            rollingFriction=rolling_friction,
            spinningFriction=spinning_friction,
            physicsClientId=client_id,
        )


def probe_terrain(client_id: int, x: float, y: float, ray_height: float = 8.0) -> TerrainProbe:
    """从机器人当前位置向下发射射线，估计局部地面高度和法向。"""
    hit = p.rayTest(
        (x, y, ray_height),
        (x, y, -ray_height),
        physicsClientId=client_id,
    )[0]
    if hit[0] < 0:
        return TerrainProbe()
    hit_position = hit[3]
    normal = hit[4]
    return TerrainProbe(
        local_ground_height=float(hit_position[2]),
        local_terrain_normal_x=float(normal[0]),
        local_terrain_normal_y=float(normal[1]),
        local_terrain_normal_z=float(normal[2]),
    )


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
