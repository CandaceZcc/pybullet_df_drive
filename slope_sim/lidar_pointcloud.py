# 多线点云模块：用冻结安装外参和可分片 16x180 射线生成原子雷达点云。
from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
import math
from numbers import Integral, Real

from slope_sim.interfaces.dashboard_snapshot import LidarTopViewFrame, LidarTopViewPoint
from slope_sim.interfaces.models import LidarPoint, LidarPointCloud
from slope_sim.scene import LIDAR_VISIBLE_GROUP
from slope_sim.sensor_backend import Pose, RayHit, SensorBackend, Vec3
from slope_sim.truth_sensors import MountPose, SensorMounts


LIDAR_SCAN_PERIOD_NS = 100_000_000
_UINT64_MAX = (1 << 64) - 1
_SUPPORTED_VERTICAL_LINES = 16
_SUPPORTED_HORIZONTAL_SAMPLES = 180
_SUPPORTED_HORIZONTAL_FOV_DEG = 180.0
_SUPPORTED_VERTICAL_FOV_DEG = (-15.0, 15.0)
_SUPPORTED_MIN_RANGE_M = 0.10
_SUPPORTED_MAX_RANGE_M = 30.0
_SUPPORTED_RAY_COUNT = _SUPPORTED_VERTICAL_LINES * _SUPPORTED_HORIZONTAL_SAMPLES
_RANGE_REL_TOLERANCE = 1e-6
_RANGE_ABS_TOLERANCE_M = 1e-5
_POINT_SEMANTICS = {
    "unknown": (0, 80),
    "terrain": (1, 100),
    "static_obstacle": (2, 160),
    "moving_obstacle": (3, 200),
}


def _require_integral(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{name} must be an integer")
    return int(value)


def _require_finite_number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be finite")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be finite")
    return normalized


def _require_vertical_fov(value: object) -> tuple[float, float]:
    invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
    if (
        isinstance(value, invalid_types)
        or not isinstance(value, Sequence)
        or len(value) != 2
    ):
        raise ValueError("vertical_fov_deg must contain lower and upper finite angles")
    lower = _require_finite_number("vertical_fov_deg[0]", value[0])
    upper = _require_finite_number("vertical_fov_deg[1]", value[1])
    return lower, upper


@dataclass(frozen=True, slots=True)
class LidarConfig:
    """阶段三第一版固定扫描几何；拒绝隐式转换和尚未支持的变体。"""

    vertical_lines: int
    horizontal_samples: int
    horizontal_fov_deg: float
    vertical_fov_deg: tuple[float, float]
    min_range_m: float
    max_range_m: float

    def __post_init__(self) -> None:
        vertical_lines = _require_integral("vertical_lines", self.vertical_lines)
        horizontal_samples = _require_integral("horizontal_samples", self.horizontal_samples)
        horizontal_fov = _require_finite_number("horizontal_fov_deg", self.horizontal_fov_deg)
        vertical_fov = _require_vertical_fov(self.vertical_fov_deg)
        minimum_range = _require_finite_number("min_range_m", self.min_range_m)
        maximum_range = _require_finite_number("max_range_m", self.max_range_m)

        if vertical_lines <= 0:
            raise ValueError("vertical_lines must be greater than zero")
        if horizontal_samples <= 0:
            raise ValueError("horizontal_samples must be greater than zero")
        if not 0.0 < horizontal_fov <= 360.0:
            raise ValueError("horizontal_fov_deg must be in range (0, 360]")
        if not -90.0 <= vertical_fov[0] < vertical_fov[1] <= 90.0:
            raise ValueError("vertical_fov_deg must be increasing within [-90, 90]")
        if minimum_range <= 0.0:
            raise ValueError("min_range_m must be greater than zero")
        if maximum_range <= minimum_range:
            raise ValueError("max_range_m must be greater than min_range_m")

        normalized = (
            vertical_lines,
            horizontal_samples,
            horizontal_fov,
            vertical_fov,
            minimum_range,
            maximum_range,
        )
        supported = (
            _SUPPORTED_VERTICAL_LINES,
            _SUPPORTED_HORIZONTAL_SAMPLES,
            _SUPPORTED_HORIZONTAL_FOV_DEG,
            _SUPPORTED_VERTICAL_FOV_DEG,
            _SUPPORTED_MIN_RANGE_M,
            _SUPPORTED_MAX_RANGE_M,
        )
        if normalized != supported:
            raise ValueError(
                "unsupported lidar geometry; only the fixed 16x180 "
                "stage-three geometry is supported"
            )

        object.__setattr__(self, "vertical_lines", vertical_lines)
        object.__setattr__(self, "horizontal_samples", horizontal_samples)
        object.__setattr__(self, "horizontal_fov_deg", horizontal_fov)
        object.__setattr__(self, "vertical_fov_deg", vertical_fov)
        object.__setattr__(self, "min_range_m", minimum_range)
        object.__setattr__(self, "max_range_m", maximum_range)

    @classmethod
    def default(cls) -> "LidarConfig":
        """返回阶段三冻结的 16 线、180 度、30 米扫描参数。"""
        return cls(
            vertical_lines=_SUPPORTED_VERTICAL_LINES,
            horizontal_samples=_SUPPORTED_HORIZONTAL_SAMPLES,
            horizontal_fov_deg=_SUPPORTED_HORIZONTAL_FOV_DEG,
            vertical_fov_deg=_SUPPORTED_VERTICAL_FOV_DEG,
            min_range_m=_SUPPORTED_MIN_RANGE_M,
            max_range_m=_SUPPORTED_MAX_RANGE_M,
        )

    @property
    def ray_count(self) -> int:
        """返回每帧固定射线总数。"""
        return self.vertical_lines * self.horizontal_samples


def _inclusive_angles(lower: float, upper: float, count: int) -> tuple[float, ...]:
    """生成包含两端点的等间隔角度，避免累计加法导致末端漂移。"""
    step = (upper - lower) / (count - 1)
    return tuple(lower + step * index for index in range(count - 1)) + (upper,)


def _direction_from_degrees(azimuth_deg: float, elevation_deg: float) -> Vec3:
    azimuth = math.radians(azimuth_deg)
    elevation = math.radians(elevation_deg)
    horizontal = math.cos(elevation)
    return (
        horizontal * math.cos(azimuth),
        horizontal * math.sin(azimuth),
        math.sin(elevation),
    )


def build_unit_rays(config: LidarConfig) -> tuple[Vec3, ...]:
    """按 line 再 azimuth 构造稳定单位射线表，水平和垂直端点均包含。"""
    if not isinstance(config, LidarConfig):
        raise ValueError("config must be a LidarConfig value")
    elevations = _inclusive_angles(
        config.vertical_fov_deg[0],
        config.vertical_fov_deg[1],
        config.vertical_lines,
    )
    half_horizontal_fov = config.horizontal_fov_deg / 2.0
    azimuths = _inclusive_angles(
        -half_horizontal_fov,
        half_horizontal_fov,
        config.horizontal_samples,
    )
    return tuple(
        _direction_from_degrees(azimuth, elevation)
        for elevation in elevations
        for azimuth in azimuths
    )


def _require_uint64(name: str, value: object) -> int:
    """严格匹配接口层 uint64：拒绝 bool、浮点和范围外整数。"""
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value <= _UINT64_MAX
    ):
        raise ValueError(f"{name} must be a uint64")
    return value


def _rotation_matrix(orientation: tuple[float, float, float, float]) -> tuple[float, ...]:
    """把单位四元数展开为行主序旋转矩阵，单帧 5760 个端点可复用。"""
    x, y, z, w = orientation
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        1.0 - 2.0 * (yy + zz),
        2.0 * (xy - wz),
        2.0 * (xz + wy),
        2.0 * (xy + wz),
        1.0 - 2.0 * (xx + zz),
        2.0 * (yz - wx),
        2.0 * (xz - wy),
        2.0 * (yz + wx),
        1.0 - 2.0 * (xx + yy),
    )


def _transform_points(pose: Pose, points: tuple[Vec3, ...]) -> tuple[Vec3, ...]:
    matrix = _rotation_matrix(pose.orientation)
    px, py, pz = pose.position
    return tuple(
        (
            px + matrix[0] * x + matrix[1] * y + matrix[2] * z,
            py + matrix[3] * x + matrix[4] * y + matrix[5] * z,
            pz + matrix[6] * x + matrix[7] * y + matrix[8] * z,
        )
        for x, y, z in points
    )


def _require_local_point(value: object, ray_index: int) -> Vec3:
    # 正式后端返回精确 tuple[float, float, float]；先走等价快速校验避开 ABC 热路径。
    if (
        type(value) is tuple
        and len(value) == 3
        and type(value[0]) is float
        and type(value[1]) is float
        and type(value[2]) is float
        and math.isfinite(value[0])
        and math.isfinite(value[1])
        and math.isfinite(value[2])
    ):
        return value
    invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
    if isinstance(value, invalid_types) or not isinstance(value, Sequence) or len(value) != 3:
        raise RuntimeError(f"inverse transformed hit {ray_index} must contain 3 finite values")
    coordinates: list[float] = []
    for coordinate in value:
        if isinstance(coordinate, bool) or not isinstance(coordinate, Real):
            raise RuntimeError(f"inverse transformed hit {ray_index} must contain 3 finite values")
        normalized = float(coordinate)
        if not math.isfinite(normalized):
            raise RuntimeError(f"inverse transformed hit {ray_index} must contain 3 finite values")
        coordinates.append(normalized)
    return tuple(coordinates)  # type: ignore[return-value]


def _require_inverse_points(raw_points: object, expected_length: int) -> tuple[Vec3, ...]:
    """统一校验后端批量逆变换的有序性、长度和有限三维坐标。"""
    invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
    if isinstance(raw_points, invalid_types) or not isinstance(raw_points, Sequence):
        raise RuntimeError(
            "sensor backend inverse_transform_points must return an ordered sequence"
        )
    points = tuple(raw_points)
    if len(points) != expected_length:
        raise RuntimeError(
            "sensor backend inverse_transform_points returned an unexpected length"
        )
    return tuple(
        _require_local_point(point, point_index)
        for point_index, point in enumerate(points)
    )


@dataclass(frozen=True, slots=True)
class LidarScanResult:
    """一次射线扫描原子生成的企业点云和 base_link 俯视帧。"""

    message: LidarPointCloud
    top_view: LidarTopViewFrame

    def __post_init__(self) -> None:
        if type(self.message) is not LidarPointCloud:
            raise ValueError("message must be an exact LidarPointCloud")
        if type(self.top_view) is not LidarTopViewFrame:
            raise ValueError("top_view must be an exact LidarTopViewFrame")
        if self.message.timebase_ns != self.top_view.timestamp_ns:
            raise ValueError("message and top_view timestamps must match")
        expected = {
            "lidar_front": 1,
            "lidar_rear": 2,
        }.get(self.message.frame_id)
        if expected is None or self.message.lidar_id != expected:
            raise ValueError("message frame_id and lidar_id must identify a lidar side")
        if len(self.message.points) != len(self.top_view.points):
            raise ValueError("message and top_view point counts must match")
        if any(point.lidar_id != expected for point in self.top_view.points):
            raise ValueError("top_view point lidar_id must match message lidar_id")
        for message_point, view_point in zip(
            self.message.points,
            self.top_view.points,
            strict=True,
        ):
            if message_point.tag != view_point.tag:
                raise ValueError("message and top_view point tags must match")


class MultiLineLidar:
    """一台固定安装的多线雷达；每次扫描只读取当前 parent 世界位姿。"""

    def __init__(
        self,
        backend: SensorBackend,
        config: LidarConfig,
        mount: MountPose,
        *,
        frame_id: str,
        lidar_id: int,
    ) -> None:
        if not isinstance(config, LidarConfig):
            raise ValueError("config must be a LidarConfig value")
        if not isinstance(mount, MountPose):
            raise ValueError("mount must be a MountPose value")
        if (
            type(frame_id) is not str
            or type(lidar_id) is not int
            or (frame_id, lidar_id)
            not in {("lidar_front", 1), ("lidar_rear", 2)}
        ):
            raise ValueError("frame_id and lidar_id must identify the front or rear lidar")
        try:
            link_names = tuple(backend.link_names())
        except (AttributeError, TypeError) as exc:
            raise ValueError("sensor backend must provide semantic link names") from exc
        if mount.parent_link not in link_names:
            raise ValueError(f"lidar parent link {mount.parent_link!r} does not exist on the current robot")

        self._backend = backend
        self.config = config
        self._mount = mount
        self.frame_id = frame_id
        self.lidar_id = lidar_id
        unit_rays = build_unit_rays(config)
        if len(unit_rays) != config.ray_count:
            raise RuntimeError("fixed lidar ray table length does not match config.ray_count")
        self._local_starts = tuple(
            tuple(component * config.min_range_m for component in ray)
            for ray in unit_rays
        )
        self._local_ends = tuple(
            tuple(component * config.max_range_m for component in ray)
            for ray in unit_rays
        )

    @classmethod
    def front(cls, backend: SensorBackend, config: LidarConfig) -> "MultiLineLidar":
        """使用正式车型 URDF 的前雷达语义 link 和 Task 8 冻结外参。"""
        return cls(
            backend,
            config,
            SensorMounts.default().lidar_front,
            frame_id="lidar_front",
            lidar_id=1,
        )

    @classmethod
    def rear(cls, backend: SensorBackend, config: LidarConfig) -> "MultiLineLidar":
        """使用冻结的后雷达 yaw=pi 外参，射线算法不另设反向分支。"""
        return cls(
            backend,
            config,
            SensorMounts.default().lidar_rear,
            frame_id="lidar_rear",
            lidar_id=2,
        )

    def _world_mount(self) -> Pose:
        parent = self._backend.world_pose(self._mount.parent_link)
        if not isinstance(parent, Pose):
            raise RuntimeError("sensor backend world_pose must return Pose")
        world_mount = self._backend.transform_pose(
            parent,
            Pose(self._mount.position, self._mount.orientation),
        )
        if not isinstance(world_mount, Pose):
            raise RuntimeError("sensor backend transform_pose must return Pose")
        return world_mount

    def _world_rays(self, mount: Pose) -> tuple[tuple[Vec3, ...], tuple[Vec3, ...]]:
        """用同一发布时刻安装位姿变换整张固定射线表，不模拟运动畸变。"""
        return _transform_points(mount, self._local_starts), _transform_points(
            mount,
            self._local_ends,
        )

    def _point_from_hit(
        self,
        ray_index: int,
        hit: RayHit,
        local_point: Vec3,
    ) -> LidarPoint | None:
        """把已完成后端边界校验的局部命中转换为企业点。"""
        distance = math.hypot(*local_point)
        # PyBullet 变换返回单精度坐标，边界比较只吸收其数值回差。
        below_minimum = distance < self.config.min_range_m and not math.isclose(
            distance,
            self.config.min_range_m,
            rel_tol=_RANGE_REL_TOLERANCE,
            abs_tol=_RANGE_ABS_TOLERANCE_M,
        )
        above_maximum = distance > self.config.max_range_m and not math.isclose(
            distance,
            self.config.max_range_m,
            rel_tol=_RANGE_REL_TOLERANCE,
            abs_tol=_RANGE_ABS_TOLERANCE_M,
        )
        if below_minimum or above_maximum:
            return None
        try:
            tag, reflectivity = _POINT_SEMANTICS[hit.category]
        except KeyError as exc:
            raise RuntimeError(f"ray {ray_index} has unsupported hit category {hit.category!r}") from exc
        return LidarPoint(
            ray_index * LIDAR_SCAN_PERIOD_NS // _SUPPORTED_RAY_COUNT,
            local_point[0],
            local_point[1],
            local_point[2],
            reflectivity,
            tag,
            ray_index // self.config.horizontal_samples,
        )

    def _indexed_hits(
        self,
        starts: tuple[Vec3, ...],
        ends: tuple[Vec3, ...],
    ) -> tuple[tuple[tuple[int, RayHit], ...], bool]:
        """优先读取生产后端紧凑命中；测试替身继续使用完整定长结果。"""
        compact_query = getattr(self._backend, "ray_test_indexed_hits", None)
        compact_declared = "ray_test_indexed_hits" in type(self._backend).__dict__
        if compact_declared and callable(compact_query):
            raw_indexed = compact_query(
                starts,
                ends,
                collision_mask=LIDAR_VISIBLE_GROUP,
            )
            invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
            if isinstance(raw_indexed, invalid_types) or not isinstance(
                raw_indexed,
                Sequence,
            ):
                raise RuntimeError(
                    "sensor backend ray_test_indexed_hits must return an ordered sequence"
                )
            indexed = tuple(raw_indexed)
            previous_index = -1
            for item_index, item in enumerate(indexed):
                if type(item) is not tuple or len(item) != 2:
                    raise RuntimeError(
                        f"sensor backend indexed hit {item_index} must contain index and RayHit"
                    )
                ray_index, hit = item
                if (
                    type(ray_index) is not int
                    or not previous_index < ray_index < self.config.ray_count
                    or type(hit) is not RayHit
                    or not hit.hit
                ):
                    raise RuntimeError(
                        f"sensor backend indexed hit {item_index} is invalid"
                    )
                previous_index = ray_index
            return indexed, True

        raw_hits = self._backend.ray_test_batch(
            starts,
            ends,
            collision_mask=LIDAR_VISIBLE_GROUP,
        )
        invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
        if isinstance(raw_hits, invalid_types) or not isinstance(raw_hits, Sequence):
            raise RuntimeError("sensor backend ray_test_batch must return an ordered sequence")
        hits = tuple(raw_hits)
        if len(hits) != self.config.ray_count:
            raise RuntimeError(
                f"sensor backend returned {len(hits)} results for {self.config.ray_count} rays"
            )
        for ray_index, hit in enumerate(hits):
            if not isinstance(hit, RayHit):
                raise RuntimeError(
                    f"sensor backend ray result {ray_index} must be a RayHit"
                )
        return tuple(
            (ray_index, hit)
            for ray_index, hit in enumerate(hits)
            if hit.hit
        ), False

    def _scan_message_at_mount(
        self,
        scan_time: int,
        mount: Pose,
        *,
        capture_world_points: bool,
    ) -> tuple[LidarPointCloud, tuple[Vec3, ...]]:
        """执行一次射线批次，并按需保留构建俯视帧所需的世界命中点。"""
        starts, ends = self._world_rays(mount)
        if len(starts) != self.config.ray_count or len(ends) != self.config.ray_count:
            raise RuntimeError("lidar ray input batches must match config.ray_count")
        indexed_hits, used_compact_query = self._indexed_hits(starts, ends)
        world_hit_points = tuple(hit.hit_position for _ray_index, hit in indexed_hits)
        prevalidated_inverse = getattr(
            self._backend,
            "inverse_transform_points_prevalidated",
            None,
        )
        if used_compact_query and callable(prevalidated_inverse):
            raw_local_points = prevalidated_inverse(mount, world_hit_points)
            if type(raw_local_points) is not tuple or len(raw_local_points) != len(
                indexed_hits
            ):
                raise RuntimeError(
                    "prevalidated inverse transform returned an unexpected result"
                )
            local_points = raw_local_points
        else:
            raw_local_points = self._backend.inverse_transform_points(
                mount,
                world_hit_points,
            )
            local_points = _require_inverse_points(raw_local_points, len(indexed_hits))

        points: list[LidarPoint] = []
        accepted_world_points: list[Vec3] | None = [] if capture_world_points else None
        for (ray_index, hit), local_point in zip(
            indexed_hits,
            local_points,
            strict=True,
        ):
            point = self._point_from_hit(ray_index, hit, local_point)
            if point is not None:
                points.append(point)
                if accepted_world_points is not None:
                    accepted_world_points.append(hit.hit_position)
        frozen_points = tuple(points)
        message = LidarPointCloud(
            scan_time,
            self.frame_id,
            len(frozen_points),
            self.lidar_id,
            frozen_points,
        )
        return message, (
            () if accepted_world_points is None else tuple(accepted_world_points)
        )

    def _scan_frozen(
        self,
        timebase_ns: int,
        world_mount_pose: Pose,
        base_pose: Pose | None = None,
    ) -> LidarPointCloud | LidarScanResult:
        """使用调用方冻结的世界位姿扫描，避免读取后端当前姿态。"""
        scan_time = _require_uint64("timebase_ns", timebase_ns)
        if type(world_mount_pose) is not Pose:
            raise ValueError("world_mount_pose must be an exact Pose")
        if base_pose is not None and type(base_pose) is not Pose:
            raise ValueError("base_pose must be an exact Pose when provided")
        message, accepted_world_points = self._scan_message_at_mount(
            scan_time,
            world_mount_pose,
            capture_world_points=base_pose is not None,
        )
        if base_pose is None:
            return message

        # 只转换企业点云已接受的同一批 world hit，天然保持 message 严格同序。
        raw_base_points = self._backend.inverse_transform_points(
            base_pose,
            accepted_world_points,
        )
        base_points = _require_inverse_points(raw_base_points, len(message.points))
        top_view = LidarTopViewFrame(
            scan_time,
            tuple(
                LidarTopViewPoint(
                    base_point[0],
                    base_point[1],
                    message_point.tag,
                    self.lidar_id,
                )
                for message_point, base_point in zip(
                    message.points,
                    base_points,
                    strict=True,
                )
            ),
        )
        return LidarScanResult(message, top_view)

    def scan_with_top_view(self, timebase_ns: int) -> LidarScanResult:
        """用一次射线批次同步生成雷达局部点云和 base_link 俯视帧。"""
        scan_time = _require_uint64("timebase_ns", timebase_ns)
        mount = self._world_mount()
        base_pose = self._backend.world_pose("base_link")
        if not isinstance(base_pose, Pose):
            raise RuntimeError("sensor backend world_pose must return Pose")
        result = self._scan_frozen(scan_time, mount, base_pose)
        if type(result) is not LidarScanResult:
            raise RuntimeError("frozen lidar dashboard scan returned an unexpected result")
        return result

    def scan(self, timebase_ns: int) -> LidarPointCloud:
        """生成企业点云，不构造仅供 Dashboard 使用的 base_link 俯视帧。"""
        scan_time = _require_uint64("timebase_ns", timebase_ns)
        result = self._scan_frozen(scan_time, self._world_mount())
        if type(result) is not LidarPointCloud:
            raise RuntimeError("frozen lidar message scan returned an unexpected result")
        return result
