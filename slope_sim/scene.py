# 场景模块：创建阶段一平面、三段式下坡和高尔夫 heightfield，并设置物理参数。
from __future__ import annotations

import math
import random
from dataclasses import dataclass

import pybullet as p

from slope_sim.telemetry import TerrainProbe


STAGE1_TERRAIN_MODELS = ("flat", "slope", "golf_heightfield")
STAGE1_TERRAIN_LENGTH = 20.0
STAGE1_TERRAIN_WIDTH = 12.0
SLOPE_UPPER_LENGTH = 4.0
SLOPE_RAMP_LENGTH = 8.0
SLOPE_LOWER_LENGTH = 6.0
SLOPE_SEAM_OVERLAP = 0.04
TERRAIN_THICKNESS = 0.08
GOLF_HEIGHTFIELD_ROWS = 96
GOLF_HEIGHTFIELD_COLUMNS = 96
GOLF_HEIGHTFIELD_CELL_SIZE = 0.14
GOLF_RELIEF_AMPLITUDES = {"low": 0.10, "medium": 0.20, "high": 0.32}
# 廊道中心仍保留大部分丘洼，避免把高尔夫地形变成无语义的平面。
GOLF_CORRIDOR_FEATURE_FLOOR = 0.813
GOLF_CORRIDOR_DETAIL_FLOOR = 0.18


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
    spawn_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    bounds: TerrainBounds | None = None

    def __post_init__(self) -> None:
        """兼容旧调用：未显式传 body_ids 时，主地形就是唯一地形体。"""
        if not self.body_ids:
            object.__setattr__(self, "body_ids", (self.body_id,))


def terrain_model_names() -> tuple[str, ...]:
    """返回阶段一可从配置和命令行选择的三类场地。"""
    return STAGE1_TERRAIN_MODELS


def golf_corridor_center(seed: int, normalized_x: float) -> float:
    """返回指定横坐标处的平滑行车廊道中心，且不受其他地形抽样影响。"""
    corridor_rng = random.Random(int(seed) ^ 0x5F3759DF)
    phase = corridor_rng.uniform(-math.pi, math.pi)
    return 0.24 * math.sin(0.85 * math.pi * normalized_x + phase)


def generate_golf_heightfield(
    seed: int,
    relief: str,
    *,
    rows: int = GOLF_HEIGHTFIELD_ROWS,
    columns: int = GOLF_HEIGHTFIELD_COLUMNS,
) -> tuple[float, ...]:
    """生成含多尺度丘洼和平滑行车廊道的可复现连续高度数组。"""
    relief_name = relief.lower()
    if relief_name not in GOLF_RELIEF_AMPLITUDES:
        raise ValueError("golf_relief must be 'low', 'medium', or 'high'")
    if rows < 4 or columns < 4:
        raise ValueError("heightfield rows and columns must be at least 4")
    amplitude = GOLF_RELIEF_AMPLITUDES[relief_name]
    rng = random.Random(int(seed))
    phase_x = rng.uniform(-math.pi, math.pi)
    phase_y = rng.uniform(-math.pi, math.pi)
    corridor_phase = random.Random(int(seed) ^ 0x5F3759DF).uniform(-math.pi, math.pi)
    hills = [
        (
            rng.uniform(-0.78, 0.78),
            rng.uniform(-0.78, 0.78),
            rng.uniform(0.16, 0.40),
            rng.uniform(0.14, 0.34),
            rng.uniform(0.28, 0.72),
        )
        for _ in range(8)
    ]
    basins = [
        (
            rng.uniform(-0.72, 0.72),
            rng.uniform(-0.72, 0.72),
            rng.uniform(0.18, 0.38),
            rng.uniform(0.16, 0.34),
            rng.uniform(0.18, 0.42),
        )
        for _ in range(4)
    ]
    heights: list[float] = []
    for row in range(rows):
        y = -1.0 + 2.0 * row / (rows - 1)
        for column in range(columns):
            x = -1.0 + 2.0 * column / (columns - 1)
            broad = 0.30 * math.sin(0.85 * math.pi * x + phase_x)
            broad += 0.24 * math.cos(0.75 * math.pi * y + phase_y)
            broad += 0.12 * x * y

            # 椭圆高斯丘洼提供不同方向的局部尺度，避免规则波纹感。
            features = 0.0
            for center_x, center_y, sigma_x, sigma_y, height_scale in hills:
                radius_sq = ((x - center_x) / sigma_x) ** 2 + ((y - center_y) / sigma_y) ** 2
                features += height_scale * math.exp(-0.5 * radius_sq)
            for center_x, center_y, sigma_x, sigma_y, depth_scale in basins:
                radius_sq = ((x - center_x) / sigma_x) ** 2 + ((y - center_y) / sigma_y) ** 2
                features -= depth_scale * math.exp(-0.5 * radius_sq)

            detail = 0.10 * math.sin(2.4 * math.pi * x + 0.5 * phase_y)
            detail += 0.08 * math.cos(2.1 * math.pi * y - 0.5 * phase_x)
            corridor_center = 0.24 * math.sin(0.85 * math.pi * x + corridor_phase)
            corridor_distance = abs(y - corridor_center)
            corridor_weight = 1.0 - math.exp(-((corridor_distance / 0.20) ** 2))

            # 廊道保留 broad 和横坡，仅适度减弱局部丘洼与小尺度波。
            feature_weight = GOLF_CORRIDOR_FEATURE_FLOOR + (1.0 - GOLF_CORRIDOR_FEATURE_FLOOR) * corridor_weight
            detail_weight = GOLF_CORRIDOR_DETAIL_FLOOR + (1.0 - GOLF_CORRIDOR_DETAIL_FLOOR) * corridor_weight
            rolling = broad + features * feature_weight + detail * detail_weight
            heights.append(amplitude * rolling)
    # 整体平移不改变坡形，把最低点抬到零以上便于 GUI 观察和出生高度计算。
    minimum = min(heights)
    return tuple(height - minimum for height in heights)


def _create_golf_heightfield_scene(
    client_id: int,
    ground_lateral_friction: float,
    ground_rolling_friction: float,
    ground_spinning_friction: float,
    golf_seed: int,
    golf_relief: str,
) -> SceneInfo:
    """用 PyBullet GEOM_HEIGHTFIELD 创建高尔夫球场式连续缓坡和小丘。"""
    heightfield_data = generate_golf_heightfield(golf_seed, golf_relief)
    collision = p.createCollisionShape(
        shapeType=p.GEOM_HEIGHTFIELD,
        meshScale=[GOLF_HEIGHTFIELD_CELL_SIZE, GOLF_HEIGHTFIELD_CELL_SIZE, 1.0],
        # heightfieldData 保持 y-major；PyBullet 的 Rows 是 x 快轴样本数。
        heightfieldData=heightfield_data,
        numHeightfieldRows=GOLF_HEIGHTFIELD_COLUMNS,
        numHeightfieldColumns=GOLF_HEIGHTFIELD_ROWS,
        flags=p.GEOM_CONCAVE_INTERNAL_EDGE,
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        basePosition=[0.0, 0.0, 0.0],
        physicsClientId=client_id,
    )
    p.changeVisualShape(
        body_id,
        -1,
        rgbaColor=[0.18, 0.58, 0.16, 1.0],
        physicsClientId=client_id,
    )
    _apply_terrain_friction(
        client_id,
        body_id,
        ground_lateral_friction,
        ground_rolling_friction,
        ground_spinning_friction,
    )
    half_x = (GOLF_HEIGHTFIELD_COLUMNS - 1) * GOLF_HEIGHTFIELD_CELL_SIZE / 2.0
    half_y = (GOLF_HEIGHTFIELD_ROWS - 1) * GOLF_HEIGHTFIELD_CELL_SIZE / 2.0
    bounds = TerrainBounds(-half_x, half_x, -half_y, half_y)
    spawn_x = -3.5
    spawn_z = _raycast_ground_height(client_id, spawn_x, 0.0)
    spawn_normal = _raycast_ground_normal(client_id, spawn_x, 0.0)
    return SceneInfo(
        body_id=body_id,
        terrain_type="golf_heightfield",
        slope_deg=0.0,
        spawn_position=(spawn_x, 0.0, spawn_z),
        spawn_orientation=_orientation_from_ground_normal(spawn_normal),
        bounds=bounds,
    )


def _raycast_ground_height(client_id: int, x: float, y: float) -> float:
    """场地创建后用真实碰撞面计算安全出生高度。"""
    hit = p.rayTest((x, y, 20.0), (x, y, -20.0), physicsClientId=client_id)[0]
    if hit[0] < 0:
        raise RuntimeError(f"terrain has no collision surface at spawn ({x}, {y})")
    return float(hit[3][2])


def _raycast_ground_normal(client_id: int, x: float, y: float) -> tuple[float, float, float]:
    """读取出生点碰撞三角面的世界法向。"""
    hit = p.rayTest((x, y, 20.0), (x, y, -20.0), physicsClientId=client_id)[0]
    if hit[0] < 0:
        raise RuntimeError(f"terrain has no collision normal at spawn ({x}, {y})")
    return tuple(float(value) for value in hit[4])


def _orientation_from_ground_normal(normal: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """在车头保持世界 +X 的前提下，让车体 +Z 轴贴合局部地形法向。"""
    nx, ny, nz = normal
    pitch = math.atan2(nx, nz)
    roll = -math.atan2(ny, math.sqrt(nx * nx + nz * nz))
    return tuple(float(value) for value in p.getQuaternionFromEuler([roll, pitch, 0.0]))


def create_slope_scene(
    client_id: int,
    slope_deg: float,
    time_step: float,
    ground_lateral_friction: float = 1.0,
    ground_rolling_friction: float = 0.02,
    ground_spinning_friction: float = 0.0,
    terrain_model: str = "flat",
    golf_seed: int = 0,
    golf_relief: str = "medium",
) -> SceneInfo:
    """创建一个地面场景，并返回地形元信息。"""
    p.resetSimulation(physicsClientId=client_id)
    p.setGravity(0.0, 0.0, -9.81, physicsClientId=client_id)
    p.setTimeStep(time_step, physicsClientId=client_id)

    terrain_model = terrain_model.lower()
    if terrain_model not in STAGE1_TERRAIN_MODELS:
        choices = ", ".join(STAGE1_TERRAIN_MODELS)
        raise ValueError(f"terrain_model must be one of: {choices}")
    if terrain_model == "golf_heightfield":
        return _create_golf_heightfield_scene(
            client_id,
            ground_lateral_friction,
            ground_rolling_friction,
            ground_spinning_friction,
            golf_seed,
            golf_relief,
        )
    if terrain_model == "slope":
        return _create_segmented_slope_scene(
            client_id,
            slope_deg,
            ground_lateral_friction,
            ground_rolling_friction,
            ground_spinning_friction,
        )
    actual_slope_deg = 0.0
    body_id = _create_planar_scene(
        client_id,
        actual_slope_deg,
        ground_lateral_friction,
        ground_rolling_friction,
        ground_spinning_friction,
    )
    bounds = TerrainBounds(
        min_x=-STAGE1_TERRAIN_LENGTH / 2.0,
        max_x=STAGE1_TERRAIN_LENGTH / 2.0,
        min_y=-STAGE1_TERRAIN_WIDTH / 2.0,
        max_y=STAGE1_TERRAIN_WIDTH / 2.0,
    )
    spawn_x = -3.5
    spawn_z = _raycast_ground_height(client_id, spawn_x, 0.0)
    spawn_orientation = p.getQuaternionFromEuler([0.0, -math.radians(actual_slope_deg), 0.0])
    return SceneInfo(
        body_id=body_id,
        terrain_type=terrain_model,
        slope_deg=actual_slope_deg,
        spawn_position=(spawn_x, 0.0, spawn_z),
        spawn_orientation=tuple(float(value) for value in spawn_orientation),
        bounds=bounds,
    )


def _create_static_terrain_box(
    client_id: int,
    half_extents: tuple[float, float, float],
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    color: tuple[float, float, float, float],
    ground_lateral_friction: float,
    ground_rolling_friction: float,
    ground_spinning_friction: float,
) -> int:
    """创建统一的静态地形箱；摩擦设置失败时不留半成品刚体。"""
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        physicsClientId=client_id,
    )
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=color,
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position,
        baseOrientation=orientation,
        physicsClientId=client_id,
    )
    try:
        _apply_terrain_friction(
            client_id,
            body_id,
            ground_lateral_friction,
            ground_rolling_friction,
            ground_spinning_friction,
        )
    except Exception:
        p.removeBody(body_id, physicsClientId=client_id)
        raise
    return body_id


def _create_segmented_slope_scene(
    client_id: int,
    slope_deg: float,
    ground_lateral_friction: float,
    ground_rolling_friction: float,
    ground_spinning_friction: float,
) -> SceneInfo:
    """沿 +X 创建高位平台、直线下坡和低位平台。"""
    slope_rad = math.radians(slope_deg)
    drop = SLOPE_RAMP_LENGTH * math.sin(slope_rad)
    horizontal_ramp = SLOPE_RAMP_LENGTH * math.cos(slope_rad)
    ramp_start_x = -horizontal_ramp / 2.0
    ramp_end_x = horizontal_ramp / 2.0
    upper_center_x = ramp_start_x - SLOPE_UPPER_LENGTH / 2.0 + SLOPE_SEAM_OVERLAP / 2.0
    lower_center_x = ramp_end_x + SLOPE_LOWER_LENGTH / 2.0 - SLOPE_SEAM_OVERLAP / 2.0
    flat_orientation = (0.0, 0.0, 0.0, 1.0)
    ramp_orientation = tuple(float(value) for value in p.getQuaternionFromEuler((0.0, slope_rad, 0.0)))
    friction = (
        ground_lateral_friction,
        ground_rolling_friction,
        ground_spinning_friction,
    )
    created_body_ids: list[int] = []

    try:
        # 薄箱的上表面是可行驶面，接缝处轻微重叠避免射线和车轮落空。
        upper_id = _create_static_terrain_box(
            client_id,
            (
                (SLOPE_UPPER_LENGTH + SLOPE_SEAM_OVERLAP) / 2.0,
                STAGE1_TERRAIN_WIDTH / 2.0,
                TERRAIN_THICKNESS / 2.0,
            ),
            (upper_center_x, 0.0, drop - TERRAIN_THICKNESS / 2.0),
            flat_orientation,
            (0.32, 0.56, 0.25, 1.0),
            *friction,
        )
        created_body_ids.append(upper_id)
        ramp_id = _create_static_terrain_box(
            client_id,
            (SLOPE_RAMP_LENGTH / 2.0, STAGE1_TERRAIN_WIDTH / 2.0, TERRAIN_THICKNESS / 2.0),
            (
                -math.sin(slope_rad) * TERRAIN_THICKNESS / 2.0,
                0.0,
                drop / 2.0 - math.cos(slope_rad) * TERRAIN_THICKNESS / 2.0,
            ),
            ramp_orientation,
            (0.48, 0.63, 0.27, 1.0),
            *friction,
        )
        created_body_ids.append(ramp_id)
        lower_id = _create_static_terrain_box(
            client_id,
            (
                (SLOPE_LOWER_LENGTH + SLOPE_SEAM_OVERLAP) / 2.0,
                STAGE1_TERRAIN_WIDTH / 2.0,
                TERRAIN_THICKNESS / 2.0,
            ),
            (lower_center_x, 0.0, -TERRAIN_THICKNESS / 2.0),
            flat_orientation,
            (0.27, 0.49, 0.22, 1.0),
            *friction,
        )
        created_body_ids.append(lower_id)
    except Exception:
        # 后续段失败时按创建反序回滚，避免场景内留下孤立地形。
        for body_id in reversed(created_body_ids):
            p.removeBody(body_id, physicsClientId=client_id)
        raise

    bounds = TerrainBounds(
        min_x=ramp_start_x - SLOPE_UPPER_LENGTH,
        max_x=ramp_end_x + SLOPE_LOWER_LENGTH,
        min_y=-STAGE1_TERRAIN_WIDTH / 2.0,
        max_y=STAGE1_TERRAIN_WIDTH / 2.0,
    )
    return SceneInfo(
        body_id=ramp_id,
        body_ids=(upper_id, ramp_id, lower_id),
        terrain_type="slope",
        slope_deg=slope_deg,
        spawn_position=(upper_center_x, 0.0, drop),
        spawn_orientation=flat_orientation,
        bounds=bounds,
    )


def _create_planar_scene(
    client_id: int,
    slope_deg: float,
    ground_lateral_friction: float,
    ground_rolling_friction: float,
    ground_spinning_friction: float,
) -> int:
    """创建水平或按指定角度倾斜的连续静态场地。"""
    slope_rad = math.radians(slope_deg)
    orientation = p.getQuaternionFromEuler([0.0, -slope_rad, 0.0])
    # flat 仍保持单一静态箱体，但复用与分段坡面相同的安全工厂。
    return _create_static_terrain_box(
        client_id,
        (STAGE1_TERRAIN_LENGTH / 2.0, STAGE1_TERRAIN_WIDTH / 2.0, TERRAIN_THICKNESS / 2.0),
        (0.0, 0.0, -TERRAIN_THICKNESS / 2.0),
        tuple(float(value) for value in orientation),
        (0.35, 0.55, 0.28, 1.0),
        ground_lateral_friction,
        ground_rolling_friction,
        ground_spinning_friction,
    )


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
    ray_start_z: float | None = None,
) -> TerrainProbe:
    """从机器人当前位置向下发射射线，估计局部地面高度和法向。"""
    if bounds is not None and not bounds.contains(x, y):
        return TerrainProbe(terrain_probe_valid=False, out_of_bounds=True)
    hit = p.rayTest(
        (x, y, ray_height if ray_start_z is None else ray_start_z),
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


def camera_follow_yaw(camera_follow_view: str, camera_yaw: float, robot_yaw: float) -> float:
    """按车体朝向计算跟随相机 yaw；custom 保留配置的世界坐标 yaw。"""
    view = camera_follow_view.lower()
    heading_deg = math.degrees(robot_yaw)
    if view == "front":
        return heading_deg - 90.0
    if view == "side":
        return heading_deg
    return camera_yaw


def update_follow_camera(
    client_id: int,
    robot_id: int,
    camera_distance: float,
    camera_pitch: float,
    camera_yaw: float,
    camera_follow_view: str,
) -> None:
    """按活动车体的实时位置和 yaw 更新 PyBullet GUI 跟随相机。"""
    position, orientation = p.getBasePositionAndOrientation(robot_id, physicsClientId=client_id)
    # PyBullet 返回四元数，欧拉角第三项是绕世界 Z 轴的车体 yaw。
    _roll, _pitch, robot_yaw = p.getEulerFromQuaternion(orientation)
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_follow_yaw(camera_follow_view, camera_yaw, robot_yaw),
        cameraPitch=camera_pitch,
        cameraTargetPosition=(float(position[0]), float(position[1]), float(position[2])),
        physicsClientId=client_id,
    )
