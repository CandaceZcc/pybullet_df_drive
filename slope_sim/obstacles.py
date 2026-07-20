# 障碍物基础模块：创建质量为零的箱体，并在物理步进前同步运动学位姿与速度。
from __future__ import annotations

import math
import random
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from numbers import Integral

import pybullet as p

from slope_sim.scene import TerrainBounds
from slope_sim.telemetry import TerrainProbe


QUATERNION_NORM_EPSILON = 1e-12
OBSTACLE_MODES = ("static", "moving", "mixed")
OBSTACLE_SHAPES = ("box", "cylinder", "sphere")
Aabb3D = tuple[tuple[float, float, float], tuple[float, float, float]]
TerrainSampler = Callable[[float, float], TerrainProbe]


class ObstaclePlanningError(RuntimeError):
    """障碍物整批规划失败；调用方不得发布半批结果。"""


def _require_finite_float(name: str, value: object) -> float:
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be finite")
    return normalized


def _require_integral(name: str, value: object) -> int:
    """拒绝 bool 和小数截断，避免逻辑 ID、数量和方向在边界处静默变形。"""
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{name} must be an integer")
    return int(value)


def _require_positive_float(name: str, value: object) -> float:
    normalized = _require_finite_float(name, value)
    if normalized <= 0.0:
        raise ValueError(f"{name} must be greater than zero")
    return normalized


def _normalize_range_pair(name: str, values: Sequence[object]) -> tuple[float, float]:
    if len(values) != 2:
        raise ValueError(f"{name} must contain min and max values")
    minimum = _require_positive_float(f"{name}[0]", values[0])
    maximum = _require_positive_float(f"{name}[1]", values[1])
    if minimum > maximum:
        raise ValueError(f"{name} minimum must not exceed maximum")
    return minimum, maximum


def _normalize_xy(name: str, values: tuple[float, ...]) -> tuple[float, float]:
    x, y = _require_finite_vector(name, values, length=2)
    return x, y


def _normalize_aabb(aabb: Aabb3D | None) -> Aabb3D | None:
    if aabb is None:
        return None
    if len(aabb) != 2:
        raise ValueError("vehicle_aabb must contain min and max corners")
    minimum = _require_finite_vector("vehicle_aabb minimum", tuple(aabb[0]), length=3)
    maximum = _require_finite_vector("vehicle_aabb maximum", tuple(aabb[1]), length=3)
    if any(minimum[index] > maximum[index] for index in range(3)):
        raise ValueError("vehicle_aabb minimum corner must not exceed maximum corner")
    return minimum, maximum


def _validate_bounds(bounds: TerrainBounds) -> TerrainBounds:
    for name in ("min_x", "max_x", "min_y", "max_y"):
        _require_finite_float(f"bounds.{name}", getattr(bounds, name))
    if bounds.min_x >= bounds.max_x or bounds.min_y >= bounds.max_y:
        raise ValueError("bounds must have positive width and height")
    return bounds


def _lower_choice(name: str, value: str, choices: tuple[str, ...]) -> str:
    normalized = str(value).lower()
    if normalized not in choices:
        joined = ", ".join(choices)
        raise ValueError(f"{name} must be one of: {joined}")
    return normalized


@dataclass(frozen=True)
class ObstacleGenerationSettings:
    """障碍物规划的场地、车辆保护区和候选采样上限。"""

    bounds: TerrainBounds
    spawn_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    spawn_protection_radius: float = 0.8
    vehicle_aabb: Aabb3D | None = None
    minimum_clearance: float = 0.10
    half_extent_ranges: tuple[tuple[float, float], tuple[float, float], tuple[float, float]] = (
        (0.12, 0.35),
        (0.12, 0.35),
        (0.20, 0.55),
    )
    moving_path_length_range: tuple[float, float] = (0.8, 2.0)
    max_candidate_attempts: int = 300
    max_batch_obstacles: int = 50
    max_scene_obstacles: int = 100

    def __post_init__(self) -> None:
        object.__setattr__(self, "bounds", _validate_bounds(self.bounds))
        object.__setattr__(self, "spawn_position", _require_finite_vector("spawn_position", self.spawn_position, length=3))
        object.__setattr__(
            self,
            "spawn_protection_radius",
            _require_finite_float("spawn_protection_radius", self.spawn_protection_radius),
        )
        if self.spawn_protection_radius < 0.0:
            raise ValueError("spawn_protection_radius must not be negative")
        object.__setattr__(self, "vehicle_aabb", _normalize_aabb(self.vehicle_aabb))
        object.__setattr__(self, "minimum_clearance", _require_finite_float("minimum_clearance", self.minimum_clearance))
        if self.minimum_clearance < 0.0:
            raise ValueError("minimum_clearance must not be negative")
        if len(self.half_extent_ranges) != 3:
            raise ValueError("half_extent_ranges must contain three axis ranges")
        object.__setattr__(
            self,
            "half_extent_ranges",
            tuple(
                _normalize_range_pair(f"half_extent_ranges[{index}]", axis_range)
                for index, axis_range in enumerate(self.half_extent_ranges)
            ),
        )
        object.__setattr__(
            self,
            "moving_path_length_range",
            _normalize_range_pair("moving_path_length_range", self.moving_path_length_range),
        )
        max_candidate_attempts = _require_integral("max_candidate_attempts", self.max_candidate_attempts)
        max_batch_obstacles = _require_integral("max_batch_obstacles", self.max_batch_obstacles)
        max_scene_obstacles = _require_integral("max_scene_obstacles", self.max_scene_obstacles)
        if max_candidate_attempts <= 0:
            raise ValueError("max_candidate_attempts must be greater than zero")
        if max_batch_obstacles <= 0:
            raise ValueError("max_batch_obstacles must be greater than zero")
        if max_scene_obstacles <= 0:
            raise ValueError("max_scene_obstacles must be greater than zero")
        if max_scene_obstacles > 100:
            raise ValueError("max_scene_obstacles must not exceed 100")
        object.__setattr__(self, "max_candidate_attempts", max_candidate_attempts)
        object.__setattr__(self, "max_batch_obstacles", max_batch_obstacles)
        object.__setattr__(self, "max_scene_obstacles", max_scene_obstacles)


@dataclass(frozen=True)
class ObstacleGenerationRequest:
    """Dashboard 一次随机添加操作对应的纯领域请求。"""

    mode: str
    count: int
    seed: int
    shape: str = "box"
    moving_ratio: float = 0.30
    moving_speed: float = 0.35

    def __post_init__(self) -> None:
        mode = _lower_choice("mode", self.mode, OBSTACLE_MODES)
        shape = _lower_choice("shape", self.shape, OBSTACLE_SHAPES)
        count = _require_integral("count", self.count)
        if count < 1 or count > 50:
            raise ValueError("count must be in the range 1..50")
        ratio = _require_finite_float("moving_ratio", self.moving_ratio)
        if ratio < 0.0 or ratio > 1.0:
            raise ValueError("moving_ratio must be in the range 0..1")
        speed = _require_finite_float("moving_speed", self.moving_speed)
        if mode in {"moving", "mixed"} and speed <= 0.0:
            raise ValueError("moving_speed must be greater than zero for moving obstacles")
        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "shape", shape)
        object.__setattr__(self, "count", count)
        object.__setattr__(self, "seed", _require_integral("seed", self.seed))
        object.__setattr__(self, "moving_ratio", ratio)
        object.__setattr__(self, "moving_speed", speed)


@dataclass(frozen=True)
class ObstacleGeometry:
    """障碍物碰撞外形的逻辑尺寸，规划阶段用外接圆做快速排斥。"""

    shape: str
    half_extents: tuple[float, float, float]

    def __post_init__(self) -> None:
        shape = _lower_choice("shape", self.shape, OBSTACLE_SHAPES)
        half_extents = _require_finite_vector("half_extents", self.half_extents, length=3)
        if any(value <= 0.0 for value in half_extents):
            raise ValueError("half_extents must be positive")
        if shape == "sphere":
            radius = max(half_extents)
            half_extents = (radius, radius, radius)
        elif shape == "cylinder":
            radius = max(half_extents[0], half_extents[1])
            half_extents = (radius, radius, half_extents[2])
        object.__setattr__(self, "shape", shape)
        object.__setattr__(self, "half_extents", half_extents)

    @property
    def bounding_radius(self) -> float:
        """返回 XY 平面的保守占地半径。"""
        if self.shape == "sphere":
            return max(self.half_extents[0], self.half_extents[1])
        if self.shape == "cylinder":
            return max(self.half_extents[0], self.half_extents[1])
        return math.hypot(self.half_extents[0], self.half_extents[1])


@dataclass(frozen=True)
class ObstaclePath:
    """移动障碍物的贴地直线路径和往返进度。"""

    start_xy: tuple[float, float]
    end_xy: tuple[float, float]
    speed: float
    progress: float = 0.0
    direction: int = 1

    def __post_init__(self) -> None:
        start_xy = _normalize_xy("start_xy", self.start_xy)
        end_xy = _normalize_xy("end_xy", self.end_xy)
        speed = _require_positive_float("speed", self.speed)
        progress = _require_finite_float("progress", self.progress)
        if progress < 0.0 or progress > 1.0:
            raise ValueError("progress must be in the range 0..1")
        direction = _require_integral("direction", self.direction)
        if direction not in {-1, 1}:
            raise ValueError("direction must be -1 or 1")
        if math.dist(start_xy, end_xy) <= QUATERNION_NORM_EPSILON:
            raise ValueError("path endpoints must be distinct")
        object.__setattr__(self, "start_xy", start_xy)
        object.__setattr__(self, "end_xy", end_xy)
        object.__setattr__(self, "speed", speed)
        object.__setattr__(self, "progress", progress)
        object.__setattr__(self, "direction", direction)

    @property
    def length(self) -> float:
        return math.dist(self.start_xy, self.end_xy)


@dataclass(frozen=True)
class ObstacleSpec:
    """不含 PyBullet body id 的稳定逻辑障碍物规格。"""

    logical_id: int
    mode: str
    geometry: ObstacleGeometry
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    path: ObstaclePath | None = None

    def __post_init__(self) -> None:
        logical_id = _require_integral("logical_id", self.logical_id)
        if logical_id < 1:
            raise ValueError("logical_id must be positive")
        mode = _lower_choice("mode", self.mode, ("static", "moving"))
        if mode == "static" and self.path is not None:
            raise ValueError("static obstacle must not have a path")
        if mode == "moving" and self.path is None:
            raise ValueError("moving obstacle must have a path")
        object.__setattr__(self, "logical_id", logical_id)
        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "position", _require_finite_vector("position", self.position, length=3))
        object.__setattr__(self, "orientation", _normalize_orientation(self.orientation))


@dataclass(frozen=True)
class ObstacleSnapshot:
    """运行时只读快照；body_id 是临时物理标识，逻辑 id 用于恢复和 UI。"""

    logical_id: int
    body_id: int | None
    mode: str
    shape: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    path: ObstaclePath | None = None

    def __post_init__(self) -> None:
        logical_id = _require_integral("logical_id", self.logical_id)
        if logical_id < 1:
            raise ValueError("logical_id must be positive")
        body_id = None if self.body_id is None else _require_integral("body_id", self.body_id)
        if body_id is not None and body_id < 0:
            raise ValueError("body_id must be non-negative when present")
        object.__setattr__(self, "logical_id", logical_id)
        object.__setattr__(self, "body_id", body_id)
        object.__setattr__(self, "mode", _lower_choice("mode", self.mode, ("static", "moving")))
        object.__setattr__(self, "shape", _lower_choice("shape", self.shape, OBSTACLE_SHAPES))
        object.__setattr__(self, "position", _require_finite_vector("position", self.position, length=3))
        object.__setattr__(self, "orientation", _normalize_orientation(self.orientation))


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


def split_mixed_obstacle_counts(count: int, moving_ratio: float) -> tuple[int, int]:
    """按 half-up 规则拆分混合批次，并在可行时保证静态和移动各至少一个。"""
    normalized_count = _require_integral("count", count)
    if normalized_count < 1:
        raise ValueError("count must be positive")
    ratio = _require_finite_float("moving_ratio", moving_ratio)
    if ratio < 0.0 or ratio > 1.0:
        raise ValueError("moving_ratio must be in the range 0..1")
    moving_count = math.floor(normalized_count * ratio + 0.5)
    if normalized_count >= 2:
        moving_count = min(normalized_count - 1, max(1, moving_count))
    else:
        moving_count = min(normalized_count, max(0, moving_count))
    return normalized_count - moving_count, moving_count


def point_segment_distance_2d(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    """计算二维点到线段的最短距离，供移动障碍物扫掠走廊排斥使用。"""
    px, py = _normalize_xy("point", point)
    ax, ay = _normalize_xy("start", start)
    bx, by = _normalize_xy("end", end)
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= QUATERNION_NORM_EPSILON:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    closest_x = ax + t * dx
    closest_y = ay + t * dy
    return math.hypot(px - closest_x, py - closest_y)


def _orientation_2d(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect_2d(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> bool:
    eps = 1e-12
    o1 = _orientation_2d(a0, a1, b0)
    o2 = _orientation_2d(a0, a1, b1)
    o3 = _orientation_2d(b0, b1, a0)
    o4 = _orientation_2d(b0, b1, a1)
    if o1 * o2 < -eps and o3 * o4 < -eps:
        return True

    def on_segment(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> bool:
        return (
            min(start[0], end[0]) - eps <= point[0] <= max(start[0], end[0]) + eps
            and min(start[1], end[1]) - eps <= point[1] <= max(start[1], end[1]) + eps
        )

    return (
        abs(o1) <= eps
        and on_segment(b0, a0, a1)
        or abs(o2) <= eps
        and on_segment(b1, a0, a1)
        or abs(o3) <= eps
        and on_segment(a0, b0, b1)
        or abs(o4) <= eps
        and on_segment(a1, b0, b1)
    )


def segment_distance_2d(
    a0: tuple[float, float],
    a1: tuple[float, float],
    b0: tuple[float, float],
    b1: tuple[float, float],
) -> float:
    """计算两条二维线段的中心线距离；交叉时距离为零。"""
    segment_a0 = _normalize_xy("a0", a0)
    segment_a1 = _normalize_xy("a1", a1)
    segment_b0 = _normalize_xy("b0", b0)
    segment_b1 = _normalize_xy("b1", b1)
    if _segments_intersect_2d(segment_a0, segment_a1, segment_b0, segment_b1):
        return 0.0
    return min(
        point_segment_distance_2d(segment_a0, segment_b0, segment_b1),
        point_segment_distance_2d(segment_a1, segment_b0, segment_b1),
        point_segment_distance_2d(segment_b0, segment_a0, segment_a1),
        point_segment_distance_2d(segment_b1, segment_a0, segment_a1),
    )


def circle_aabb_distance_2d(center: tuple[float, float], aabb: Aabb3D | None) -> float:
    """返回圆心到车辆 AABB 的 XY 距离；圆心在矩形内时为零。"""
    if aabb is None:
        return math.inf
    x, y = _normalize_xy("center", center)
    normalized_aabb = _normalize_aabb(aabb)
    assert normalized_aabb is not None
    minimum, maximum = normalized_aabb
    closest_x = min(max(x, minimum[0]), maximum[0])
    closest_y = min(max(y, minimum[1]), maximum[1])
    return math.hypot(x - closest_x, y - closest_y)


def _segment_aabb_distance_2d(
    start: tuple[float, float],
    end: tuple[float, float],
    aabb: Aabb3D | None,
) -> float:
    if aabb is None:
        return math.inf
    normalized_aabb = _normalize_aabb(aabb)
    assert normalized_aabb is not None
    minimum, maximum = normalized_aabb
    corners = (
        (minimum[0], minimum[1]),
        (maximum[0], minimum[1]),
        (maximum[0], maximum[1]),
        (minimum[0], maximum[1]),
    )
    if circle_aabb_distance_2d(start, normalized_aabb) <= 0.0 or circle_aabb_distance_2d(end, normalized_aabb) <= 0.0:
        return 0.0
    edges = tuple(zip(corners, (*corners[1:], corners[0])))
    return min(segment_distance_2d(start, end, edge_start, edge_end) for edge_start, edge_end in edges)


def advance_ping_pong_progress(
    *,
    progress: float,
    direction: int,
    segment_length: float,
    speed: float,
    dt: float,
) -> tuple[float, int]:
    """按剩余位移推进往返路径，跨端点时翻转并继续消费位移。"""
    current_progress = _require_finite_float("progress", progress)
    if current_progress < 0.0 or current_progress > 1.0:
        raise ValueError("progress must be in the range 0..1")
    current_direction = _require_integral("direction", direction)
    if current_direction not in {-1, 1}:
        raise ValueError("direction must be -1 or 1")
    length = _require_positive_float("segment_length", segment_length)
    remaining = _require_positive_float("speed", speed) * _require_finite_float("dt", dt)
    if remaining < 0.0:
        raise ValueError("dt must not be negative")
    if remaining == 0.0:
        return current_progress, current_direction

    # 三角波相位等价于无限长折线路径，O(1) 消费任意大的剩余位移。
    phase = current_progress if current_direction > 0 else 2.0 - current_progress
    phase = (phase + remaining / length) % 2.0
    if math.isclose(phase, 0.0, abs_tol=1e-12) or math.isclose(phase, 2.0, abs_tol=1e-12):
        return 0.0, 1
    if math.isclose(phase, 1.0, abs_tol=1e-12):
        return 1.0, -1
    if phase < 1.0:
        return phase, 1
    return 2.0 - phase, -1


def _vector_length(vector: tuple[float, float, float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _normalize_vector3(name: str, vector: tuple[float, float, float]) -> tuple[float, float, float]:
    values = _require_finite_vector(name, vector, length=3)
    length = _vector_length(values)
    if length <= QUATERNION_NORM_EPSILON:
        raise ValueError(f"{name} must be non-zero")
    return tuple(value / length for value in values)


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _quaternion_from_matrix_rows(rows: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float, float]:
    m00, m01, m02 = rows[0]
    m10, m11, m12 = rows[1]
    m20, m21, m22 = rows[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / scale
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / scale
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / scale
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale
    return _normalize_orientation((qx, qy, qz, qw))


def _orientation_from_heading_and_normal(
    heading_xy: tuple[float, float],
    normal: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    """组合路径航向和地形法向：局部 X 尽量沿路径，局部 Z 贴合法向。"""
    up = _normalize_vector3("terrain_normal", normal)
    heading_x, heading_y = _normalize_xy("heading_xy", heading_xy)
    heading_length = math.hypot(heading_x, heading_y)
    if heading_length <= QUATERNION_NORM_EPSILON:
        raise ValueError("heading_xy must be non-zero")
    desired_forward = (heading_x / heading_length, heading_y / heading_length, 0.0)
    projected = tuple(desired_forward[index] - _dot(desired_forward, up) * up[index] for index in range(3))
    if _vector_length(projected) <= QUATERNION_NORM_EPSILON:
        projected = _cross((0.0, 1.0, 0.0), up)
    forward = _normalize_vector3("forward", projected)
    left = _normalize_vector3("left", _cross(up, forward))
    forward = _normalize_vector3("forward", _cross(left, up))
    rows = (
        (forward[0], left[0], up[0]),
        (forward[1], left[1], up[1]),
        (forward[2], left[2], up[2]),
    )
    return _quaternion_from_matrix_rows(rows)


def _normal_from_probe(probe: TerrainProbe) -> tuple[float, float, float]:
    return (probe.local_terrain_normal_x, probe.local_terrain_normal_y, probe.local_terrain_normal_z)


def _modes_for_request(request: ObstacleGenerationRequest) -> tuple[str, ...]:
    if request.mode == "static":
        return ("static",) * request.count
    if request.mode == "moving":
        return ("moving",) * request.count
    static_count, moving_count = split_mixed_obstacle_counts(request.count, request.moving_ratio)
    return ("static",) * static_count + ("moving",) * moving_count


def _sample_geometry(shape: str, settings: ObstacleGenerationSettings, rng: random.Random) -> ObstacleGeometry:
    # 尺寸只来自请求局部 RNG，保证同 seed 和同逻辑集合下可复现。
    half_extents = tuple(rng.uniform(axis_min, axis_max) for axis_min, axis_max in settings.half_extent_ranges)
    return ObstacleGeometry(shape=shape, half_extents=half_extents)


def _circle_inside_bounds(center: tuple[float, float], radius: float, bounds: TerrainBounds) -> bool:
    x, y = center
    return bounds.min_x + radius <= x <= bounds.max_x - radius and bounds.min_y + radius <= y <= bounds.max_y - radius


def _path_clearance_distance(a: ObstacleSpec, b: ObstacleSpec) -> float:
    if a.path is None and b.path is None:
        return math.hypot(a.position[0] - b.position[0], a.position[1] - b.position[1])
    if a.path is not None and b.path is not None:
        return segment_distance_2d(a.path.start_xy, a.path.end_xy, b.path.start_xy, b.path.end_xy)
    moving = a if a.path is not None else b
    static = b if a.path is not None else a
    assert moving.path is not None
    return point_segment_distance_2d((static.position[0], static.position[1]), moving.path.start_xy, moving.path.end_xy)


def _candidate_avoids_protected_spaces(spec: ObstacleSpec, settings: ObstacleGenerationSettings) -> bool:
    radius = spec.geometry.bounding_radius
    required_clearance = radius + settings.minimum_clearance
    spawn_xy = (settings.spawn_position[0], settings.spawn_position[1])
    if spec.path is None:
        center_xy = (spec.position[0], spec.position[1])
        if math.hypot(center_xy[0] - spawn_xy[0], center_xy[1] - spawn_xy[1]) < (
            radius + settings.spawn_protection_radius + settings.minimum_clearance
        ):
            return False
        return circle_aabb_distance_2d(center_xy, settings.vehicle_aabb) >= required_clearance
    if point_segment_distance_2d(spawn_xy, spec.path.start_xy, spec.path.end_xy) < (
        radius + settings.spawn_protection_radius + settings.minimum_clearance
    ):
        return False
    return _segment_aabb_distance_2d(spec.path.start_xy, spec.path.end_xy, settings.vehicle_aabb) >= required_clearance


def _candidate_is_valid(
    spec: ObstacleSpec,
    settings: ObstacleGenerationSettings,
    occupied_specs: tuple[ObstacleSpec, ...],
) -> bool:
    radius = spec.geometry.bounding_radius
    if spec.path is None:
        if not _circle_inside_bounds((spec.position[0], spec.position[1]), radius, settings.bounds):
            return False
    else:
        if not _circle_inside_bounds(spec.path.start_xy, radius, settings.bounds):
            return False
        if not _circle_inside_bounds(spec.path.end_xy, radius, settings.bounds):
            return False
    if not _candidate_avoids_protected_spaces(spec, settings):
        return False
    for other in occupied_specs:
        required = radius + other.geometry.bounding_radius + settings.minimum_clearance
        if _path_clearance_distance(spec, other) < required:
            return False
    return True


def _sample_valid_probe(terrain_sampler: TerrainSampler, x: float, y: float) -> TerrainProbe | None:
    probe = terrain_sampler(x, y)
    if not probe.terrain_probe_valid or probe.out_of_bounds:
        return None
    _require_finite_float("local_ground_height", probe.local_ground_height)
    _normalize_vector3("terrain_normal", _normal_from_probe(probe))
    return probe


def _make_candidate_spec(
    *,
    logical_id: int,
    mode: str,
    request: ObstacleGenerationRequest,
    settings: ObstacleGenerationSettings,
    rng: random.Random,
    terrain_sampler: TerrainSampler,
) -> ObstacleSpec | None:
    geometry = _sample_geometry(request.shape, settings, rng)
    radius = geometry.bounding_radius
    if settings.bounds.min_x + radius > settings.bounds.max_x - radius:
        return None
    if settings.bounds.min_y + radius > settings.bounds.max_y - radius:
        return None
    x = rng.uniform(settings.bounds.min_x + radius, settings.bounds.max_x - radius)
    y = rng.uniform(settings.bounds.min_y + radius, settings.bounds.max_y - radius)
    heading_xy = (1.0, 0.0)
    path: ObstaclePath | None = None

    if mode == "moving":
        heading_angle = rng.uniform(-math.pi, math.pi)
        path_length = rng.uniform(*settings.moving_path_length_range)
        heading_xy = (math.cos(heading_angle), math.sin(heading_angle))
        end_xy = (x + heading_xy[0] * path_length, y + heading_xy[1] * path_length)
        if not _circle_inside_bounds(end_xy, radius, settings.bounds):
            return None
        if _sample_valid_probe(terrain_sampler, end_xy[0], end_xy[1]) is None:
            return None
        path = ObstaclePath(start_xy=(x, y), end_xy=end_xy, speed=request.moving_speed)

    probe = _sample_valid_probe(terrain_sampler, x, y)
    if probe is None:
        return None
    z = probe.local_ground_height + geometry.half_extents[2]
    orientation = _orientation_from_heading_and_normal(heading_xy, _normal_from_probe(probe))
    return ObstacleSpec(
        logical_id=logical_id,
        mode=mode,
        geometry=geometry,
        position=(x, y, z),
        orientation=orientation,
        path=path,
    )


def plan_obstacle_batch(
    settings: ObstacleGenerationSettings,
    request: ObstacleGenerationRequest,
    terrain_sampler: TerrainSampler,
    *,
    existing_specs: Sequence[ObstacleSpec] = (),
) -> tuple[ObstacleSpec, ...]:
    """确定性规划整批逻辑障碍物；失败时抛错且不返回半批。"""
    if request.count > settings.max_batch_obstacles:
        raise ValueError(f"count must not exceed per-batch limit {settings.max_batch_obstacles}")
    existing = tuple(existing_specs)
    if len(existing) + request.count > settings.max_scene_obstacles:
        raise ValueError(f"scene obstacle count must not exceed {settings.max_scene_obstacles}")
    next_logical_id = (max((spec.logical_id for spec in existing), default=0) + 1)
    rng = random.Random(request.seed)
    planned: list[ObstacleSpec] = []
    modes = _modes_for_request(request)

    for offset, mode in enumerate(modes):
        logical_id = next_logical_id + offset
        occupied = (*existing, *planned)
        accepted: ObstacleSpec | None = None
        for _ in range(settings.max_candidate_attempts):
            candidate = _make_candidate_spec(
                logical_id=logical_id,
                mode=mode,
                request=request,
                settings=settings,
                rng=rng,
                terrain_sampler=terrain_sampler,
            )
            if candidate is None:
                continue
            if _candidate_is_valid(candidate, settings, occupied):
                accepted = candidate
                break
        if accepted is None:
            raise ObstaclePlanningError(
                f"Unable to plan complete obstacle batch of {request.count}; "
                f"candidate attempts exhausted while placing logical id {logical_id}"
            )
        planned.append(accepted)
    return tuple(planned)


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
    obstacle_body_id = _require_integral("body_id", body_id)
    if obstacle_body_id < 0:
        raise ValueError("body_id must be non-negative")
    base_position = _require_finite_vector("position", position, length=3)
    tangent_velocity = _require_finite_vector("linear_velocity", linear_velocity, length=3)
    if orientation is None:
        _, current_orientation = p.getBasePositionAndOrientation(obstacle_body_id, physicsClientId=client_id)
        base_orientation = _normalize_orientation(tuple(current_orientation))
    else:
        base_orientation = _normalize_orientation(orientation)

    # 质量为零的 body 不受求解器反推；每帧重置则显式维持规划轨迹和接触表面速度。
    p.resetBasePositionAndOrientation(
        obstacle_body_id,
        base_position,
        base_orientation,
        physicsClientId=client_id,
    )
    p.resetBaseVelocity(
        obstacle_body_id,
        linearVelocity=tangent_velocity,
        angularVelocity=(0.0, 0.0, 0.0),
        physicsClientId=client_id,
    )
