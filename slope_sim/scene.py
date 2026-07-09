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
DAM_EDGE_GUARD_MARGIN = 0.5


@dataclass(frozen=True)
class TerrainBounds:
    """地形可行驶矩形范围，用于把 raycast miss 和驶出场地分开记录。"""

    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def contains(self, x: float, y: float) -> bool:
        """判断水平坐标是否仍在有限地形范围内。"""
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y


@dataclass(frozen=True)
class SceneInfo:
    """当前仿真场景的核心信息，供日志和地形探测使用。"""

    body_id: int
    terrain_type: str
    slope_deg: float
    body_ids: tuple[int, ...] = ()
    spawn_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    bounds: TerrainBounds | None = None

    def __post_init__(self) -> None:
        """兼容旧调用：未显式传 body_ids 时，主地形就是唯一地形体。"""
        if not self.body_ids:
            object.__setattr__(self, "body_ids", (self.body_id,))


def create_slope_scene(
    client_id: int,
    slope_deg: float,
    time_step: float,
    ground_lateral_friction: float = 1.0,
    ground_rolling_friction: float = 0.02,
    ground_spinning_friction: float = 0.0,
    terrain_model: str = "box_slope",
    dam_toe_length: float = 2.0,
    dam_slope_length: float = 8.0,
    dam_crest_length: float = 3.0,
    dam_exit_length: float = 2.0,
    dam_width: float = 4.0,
    dam_wall_height: float = 0.35,
    terrain_guard_enabled: bool = True,
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
    if terrain_model == "dam_slope":
        return _create_dam_slope_scene(
            client_id,
            slope_deg,
            ground_lateral_friction,
            ground_rolling_friction,
            ground_spinning_friction,
            dam_toe_length,
            dam_slope_length,
            dam_crest_length,
            dam_exit_length,
            dam_width,
            dam_wall_height,
            terrain_guard_enabled,
        )
    if terrain_model != "box_slope":
        raise ValueError("terrain_model must be 'box_slope', 'twr_slope_5deg', or 'dam_slope'")
    body_id = _create_box_slope_scene(
        client_id,
        slope_deg,
        ground_lateral_friction,
        ground_rolling_friction,
        ground_spinning_friction,
    )
    return SceneInfo(body_id=body_id, terrain_type=terrain_model, slope_deg=slope_deg)


def _create_dam_slope_scene(
    client_id: int,
    slope_deg: float,
    ground_lateral_friction: float,
    ground_rolling_friction: float,
    ground_spinning_friction: float,
    dam_toe_length: float,
    dam_slope_length: float,
    dam_crest_length: float,
    dam_exit_length: float,
    dam_width: float,
    dam_wall_height: float,
    terrain_guard_enabled: bool,
) -> SceneInfo:
    """用多个静态 box 拼出入口、上坡、坝顶、下坡和出口平地。"""
    slope_rad = math.radians(abs(slope_deg))
    crest_height = dam_slope_length * math.tan(slope_rad)
    x0 = 0.0
    x1 = x0 + dam_toe_length
    x2 = x1 + dam_slope_length
    x3 = x2 + dam_crest_length
    x4 = x3 + dam_slope_length
    x5 = x4 + dam_exit_length
    thickness = 0.08

    body_ids = [
        _create_dam_surface_segment(client_id, x0, x1, 0.0, 0.0, dam_width, thickness, [0.32, 0.50, 0.28, 1.0]),
        _create_dam_surface_segment(client_id, x1, x2, 0.0, crest_height, dam_width, thickness, [0.38, 0.56, 0.32, 1.0]),
        _create_dam_surface_segment(client_id, x2, x3, crest_height, crest_height, dam_width, thickness, [0.40, 0.58, 0.34, 1.0]),
        _create_dam_surface_segment(client_id, x3, x4, crest_height, 0.0, dam_width, thickness, [0.38, 0.56, 0.32, 1.0]),
        _create_dam_surface_segment(client_id, x4, x5, 0.0, 0.0, dam_width, thickness, [0.32, 0.50, 0.28, 1.0]),
    ]
    if terrain_guard_enabled:
        body_ids.extend(_create_dam_guard_rails(client_id, x5, dam_width, crest_height, dam_wall_height))

    for body_id in body_ids:
        _apply_terrain_friction(
            client_id,
            body_id,
            ground_lateral_friction,
            ground_rolling_friction,
            ground_spinning_friction,
        )

    # 越界保护比物理边缘提前触发，避免手动模式中车体中心刚越过边缘才停车。
    edge_guard_margin = min(DAM_EDGE_GUARD_MARGIN, dam_toe_length / 2.0, dam_exit_length / 2.0)
    bounds = TerrainBounds(
        min_x=x0 + edge_guard_margin,
        max_x=x5 - edge_guard_margin,
        min_y=-dam_width / 2.0,
        max_y=dam_width / 2.0,
    )
    spawn_position = (dam_toe_length / 2.0, 0.0, 0.0)
    return SceneInfo(
        body_id=body_ids[0],
        body_ids=tuple(body_ids),
        terrain_type="dam_slope",
        slope_deg=slope_deg,
        spawn_position=spawn_position,
        bounds=bounds,
    )


def _create_dam_surface_segment(
    client_id: int,
    x_start: float,
    x_end: float,
    z_start: float,
    z_end: float,
    width: float,
    thickness: float,
    color: list[float],
) -> int:
    """创建一个顶面连接两端高度的静态地形块。"""
    dx = x_end - x_start
    dz = z_end - z_start
    surface_angle = math.atan2(dz, dx)
    surface_length = math.hypot(dx, dz)
    center_x = (x_start + x_end) / 2.0 + math.sin(surface_angle) * thickness / 2.0
    center_z = (z_start + z_end) / 2.0 - math.cos(surface_angle) * thickness / 2.0
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[surface_length / 2.0, width / 2.0, thickness / 2.0],
        physicsClientId=client_id,
    )
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[surface_length / 2.0, width / 2.0, thickness / 2.0],
        rgbaColor=color,
        physicsClientId=client_id,
    )
    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[center_x, 0.0, center_z],
        baseOrientation=p.getQuaternionFromEuler([0.0, -surface_angle, 0.0]),
        physicsClientId=client_id,
    )


def _create_dam_guard_rails(
    client_id: int,
    total_length: float,
    width: float,
    crest_height: float,
    wall_height: float,
) -> list[int]:
    """创建左右静态护栏，终点保持开放以便越界保护接管。"""
    rail_thickness = 0.12
    side_height = crest_height + wall_height
    side_collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=[total_length / 2.0, rail_thickness / 2.0, side_height / 2.0],
        physicsClientId=client_id,
    )
    side_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[total_length / 2.0, rail_thickness / 2.0, side_height / 2.0],
        rgbaColor=[0.45, 0.45, 0.45, 1.0],
        physicsClientId=client_id,
    )
    body_ids: list[int] = []
    for side in (-1.0, 1.0):
        body_ids.append(
            p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=side_collision,
                baseVisualShapeIndex=side_visual,
                basePosition=[total_length / 2.0, side * (width / 2.0 + rail_thickness / 2.0), side_height / 2.0],
                physicsClientId=client_id,
            )
        )
    return body_ids


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


def probe_terrain(
    client_id: int,
    x: float,
    y: float,
    ray_height: float = 8.0,
    bounds: TerrainBounds | None = None,
) -> TerrainProbe:
    """从机器人当前位置向下发射射线，估计局部地面高度和法向。"""
    if bounds is not None and not bounds.contains(x, y):
        return TerrainProbe(terrain_probe_valid=False, out_of_bounds=True)
    hit = p.rayTest(
        (x, y, ray_height),
        (x, y, -ray_height),
        physicsClientId=client_id,
    )[0]
    if hit[0] < 0:
        return TerrainProbe(terrain_probe_valid=False, out_of_bounds=True)
    hit_position = hit[3]
    normal = hit[4]
    return TerrainProbe(
        terrain_probe_valid=True,
        out_of_bounds=False,
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


def camera_follow_yaw(camera_follow_view: str, camera_yaw: float) -> float:
    """根据跟随视角选择 debug camera yaw；custom 保留配置值。"""
    view = camera_follow_view.lower()
    if view == "front":
        return -90.0
    if view == "side":
        return 0.0
    return camera_yaw


def update_follow_camera(
    client_id: int,
    robot_id: int,
    camera_distance: float,
    camera_pitch: float,
    camera_yaw: float,
    camera_follow_view: str,
) -> None:
    """把 PyBullet GUI debug camera 的 target 绑定到机器人 base 位置。"""
    position, _orientation = p.getBasePositionAndOrientation(robot_id, physicsClientId=client_id)
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_follow_yaw(camera_follow_view, camera_yaw),
        cameraPitch=camera_pitch,
        cameraTargetPosition=(float(position[0]), float(position[1]), float(position[2])),
        physicsClientId=client_id,
    )
