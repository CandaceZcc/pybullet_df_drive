"""Golf MCAP 回放的姿态恢复、插值与逐点去畸变。"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
from typing import Sequence

import numpy as np

from slope_sim.interfaces.v2.models import (
    ImuAttitudeV2,
    LidarPointCloudV2,
    RtkStateV2,
)
from slope_sim.lidar_pointcloud import LIDAR_SCAN_PERIOD_NS, mid360_offset_time_ns
from slope_sim.sensor_backend import Pose, Quaternion, Vec3


_RTK_HALF_BASELINE_M = 0.20
_RTK_CENTER_HEIGHT_M = 0.18
_LIDAR_HEIGHT_M = 0.105
_BASELINE_EPSILON_M = 1e-9
_BASELINE_DIRECTION_TOLERANCE = 5e-6
_RTK_GEOMETRY_TOLERANCE_M = 5e-6
_LIDAR_FRAME_PERIOD_NS = 100_000_000
_LIDAR_FIRING_SLOT_COUNT = 5_760
_LIDAR_LAST_OFFSET_NS = mid360_offset_time_ns(_LIDAR_FIRING_SLOT_COUNT - 1)
_MAX_STATIC_DISPLAY_POINTS = 500_000
_PLAYBACK_RATES = (0.25, 0.5, 1.0, 2.0, 4.0)
_UINT64_MAX = (1 << 64) - 1


class MissingPoseLookaheadError(ValueError):
    """点的采样时刻超出已记录姿态节点时抛出。"""


@dataclass(frozen=True, slots=True)
class RecoveredPoseNode:
    """同一 RTK/IMU 时刻恢复出的 base 与 LiDAR 世界位姿。"""

    timestamp_ns: int
    base_pose: Pose
    lidar_pose: Pose

    def __post_init__(self) -> None:
        if (
            isinstance(self.timestamp_ns, bool)
            or not isinstance(self.timestamp_ns, int)
            or not 0 <= self.timestamp_ns <= _UINT64_MAX
        ):
            raise ValueError("timestamp_ns must fit uint64")
        if type(self.base_pose) is not Pose or type(self.lidar_pose) is not Pose:
            raise ValueError("base_pose and lidar_pose must be exact Pose values")


@dataclass(frozen=True, slots=True)
class DeskewedPoint:
    """保留采样时刻与 MID-360 语义的世界坐标点。"""

    timestamp_ns: int
    position: Vec3
    reflectivity: int
    tag: int
    line: int

    def __post_init__(self) -> None:
        if (
            isinstance(self.timestamp_ns, bool)
            or not isinstance(self.timestamp_ns, int)
            or not 0 <= self.timestamp_ns <= _UINT64_MAX
        ):
            raise ValueError("timestamp_ns must fit uint64")
        object.__setattr__(
            self,
            "position",
            _require_finite_vector("position", self.position),
        )
        for name, maximum in (("reflectivity", (1 << 32) - 1), ("tag", 3), ("line", 15)):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or not 0 <= value <= maximum
            ):
                raise ValueError(f"{name} is outside its unsigned range")


@dataclass(frozen=True, slots=True)
class WorldMapSnapshot:
    """可直接交给渲染线程的只读世界地图数组。"""

    permanent_positions: np.ndarray
    permanent_tags: np.ndarray
    moving_positions: np.ndarray
    moving_tags: np.ndarray
    permanent_voxel_count: int


@dataclass(frozen=True, slots=True)
class PlaybackFrameRequest:
    """容量 1 后台队列中的单个逻辑帧请求。"""

    generation: int
    frame_index: int
    rebuild_from_start: bool


def _require_finite_vector(name: str, value: object) -> Vec3:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError(f"{name} must contain three finite numbers")
    if any(
        isinstance(component, bool)
        or not isinstance(component, Real)
        or not math.isfinite(float(component))
        for component in value
    ):
        raise ValueError(f"{name} must contain three finite numbers")
    return tuple(float(component) for component in value)  # type: ignore[return-value]


def _normalize_vector(vector: Vec3, *, name: str) -> Vec3:
    length = math.hypot(*vector)
    if length <= _BASELINE_EPSILON_M:
        raise ValueError(f"{name} is degenerate")
    return tuple(component / length for component in vector)  # type: ignore[return-value]


def _normalize_quaternion(value: object, *, name: str) -> Quaternion:
    if not isinstance(value, (tuple, list)) or len(value) != 4:
        raise ValueError(f"{name} must contain four finite numbers")
    if any(
        isinstance(component, bool)
        or not isinstance(component, Real)
        or not math.isfinite(float(component))
        for component in value
    ):
        raise ValueError(f"{name} must contain four finite numbers")
    copied = tuple(float(component) for component in value)
    length = math.hypot(*copied)
    if not math.isfinite(length) or length <= 0.0:
        raise ValueError(f"{name} norm must be finite and nonzero")
    return tuple(component / length for component in copied)  # type: ignore[return-value]


def _quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return _normalize_quaternion(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ),
        name="recovered orientation",
    )


def _rotate(orientation: Quaternion, point: Vec3) -> Vec3:
    x, y, z, w = orientation
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    px, py, pz = point
    return (
        (1.0 - 2.0 * (yy + zz)) * px + 2.0 * (xy - wz) * py + 2.0 * (xz + wy) * pz,
        2.0 * (xy + wz) * px + (1.0 - 2.0 * (xx + zz)) * py + 2.0 * (yz - wx) * pz,
        2.0 * (xz - wy) * px + 2.0 * (yz + wx) * py + (1.0 - 2.0 * (xx + yy)) * pz,
    )


def _transform(pose: Pose, point: Vec3) -> Vec3:
    rotated = _rotate(pose.orientation, point)
    return tuple(
        pose.position[index] + rotated[index] for index in range(3)
    )  # type: ignore[return-value]


def _require_matching_pose_messages(rtk: RtkStateV2, imu: ImuAttitudeV2) -> None:
    if type(rtk) is not RtkStateV2 or type(imu) is not ImuAttitudeV2:
        raise ValueError("rtk and imu must be exact v2 pose messages")
    if rtk.timestamp_ns != imu.timestamp_ns:
        raise ValueError("RTK and IMU timestamps must match")
    if rtk.world_generation != imu.world_generation:
        raise ValueError("RTK and IMU world generations must match")
    if rtk.simulation_session_id != imu.simulation_session_id:
        raise ValueError("RTK and IMU simulation sessions must match")
    if rtk.descriptor_sha256 != imu.descriptor_sha256:
        raise ValueError("RTK and IMU descriptors must match")
    if rtk.frame_id != "world" or imu.frame_id != "base_link":
        raise ValueError("RTK/IMU frame_id must be world/base_link")


def recover_pose_node(
    rtk: RtkStateV2,
    imu: ImuAttitudeV2,
    *,
    previous_orientation: Quaternion | None = None,
) -> RecoveredPoseNode:
    """由三维 +Y RTK 基线和 IMU roll/pitch 恢复唯一世界位姿。"""
    _require_matching_pose_messages(rtk, imu)
    baseline = (
        rtk.left.x_m - rtk.right.x_m,
        rtk.left.y_m - rtk.right.y_m,
        rtk.left.z_m - rtk.right.z_m,
    )
    baseline_length = math.hypot(*baseline)
    expected_length = 2.0 * _RTK_HALF_BASELINE_M
    if not math.isclose(
        baseline_length,
        expected_length,
        rel_tol=0.0,
        abs_tol=_RTK_GEOMETRY_TOLERANCE_M,
    ):
        raise ValueError("LEFT-RIGHT RTK baseline must be 0.40 m")
    midpoint = (
        (rtk.left.x_m + rtk.right.x_m) / 2.0,
        (rtk.left.y_m + rtk.right.y_m) / 2.0,
        (rtk.left.z_m + rtk.right.z_m) / 2.0,
    )
    center = (rtk.center.x_m, rtk.center.y_m, rtk.center.z_m)
    if max(abs(midpoint[index] - center[index]) for index in range(3)) > _RTK_GEOMETRY_TOLERANCE_M:
        raise ValueError("RTK CENTER must be the LEFT-RIGHT midpoint")
    base_y_world = _normalize_vector(baseline, name="LEFT-RIGHT RTK baseline")

    # ZYX 旋转下，base +Y 的水平投影角还包含 roll/pitch 项，不能直接当 yaw。
    roll = imu.roll_rad
    pitch = imu.pitch_rad
    horizontal_norm = math.hypot(base_y_world[0], base_y_world[1])
    if horizontal_norm <= _BASELINE_EPSILON_M:
        raise ValueError("LEFT-RIGHT RTK baseline has no horizontal yaw information")
    baseline_heading = math.atan2(base_y_world[1], base_y_world[0])
    expected_heading = math.remainder(
        baseline_heading - math.pi / 2.0,
        2.0 * math.pi,
    )
    if abs(math.remainder(rtk.heading_rad - expected_heading, 2.0 * math.pi)) > 1e-7:
        raise ValueError("RTK heading_rad is inconsistent with the LEFT-RIGHT baseline")
    yaw = baseline_heading - math.atan2(
        math.cos(roll),
        math.sin(pitch) * math.sin(roll),
    )
    orientation = _quaternion_from_euler(roll, pitch, yaw)
    recovered_base_y = _rotate(orientation, (0.0, 1.0, 0.0))
    direction_error = max(
        abs(recovered_base_y[index] - base_y_world[index]) for index in range(3)
    )
    if direction_error > _BASELINE_DIRECTION_TOLERANCE:
        raise ValueError(
            "RTK baseline is inconsistent with IMU roll/pitch: "
            f"maximum direction error {direction_error:.17g}"
        )

    if previous_orientation is not None:
        previous = _normalize_quaternion(
            previous_orientation,
            name="previous_orientation",
        )
        if sum(orientation[index] * previous[index] for index in range(4)) < 0.0:
            orientation = tuple(-value for value in orientation)  # type: ignore[assignment]

    center_offset = _rotate(orientation, (0.0, 0.0, _RTK_CENTER_HEIGHT_M))
    base_position = tuple(
        center[index] - center_offset[index] for index in range(3)
    )
    base_pose = Pose(base_position, orientation)  # type: ignore[arg-type]
    lidar_position = _transform(base_pose, (0.0, 0.0, _LIDAR_HEIGHT_M))
    lidar_pose = Pose(lidar_position, orientation)
    return RecoveredPoseNode(rtk.timestamp_ns, base_pose, lidar_pose)


def slerp_shortest(
    start: Quaternion,
    end: Quaternion,
    fraction: float,
) -> Quaternion:
    """在两个单位四元数之间做符号稳定的最短弧 SLERP。"""
    if isinstance(fraction, bool) or not isinstance(fraction, Real):
        raise ValueError("fraction must be a finite number in range 0..1")
    normalized_fraction = float(fraction)
    if not math.isfinite(normalized_fraction) or not 0.0 <= normalized_fraction <= 1.0:
        raise ValueError("fraction must be a finite number in range 0..1")
    first = _normalize_quaternion(start, name="start quaternion")
    second = _normalize_quaternion(end, name="end quaternion")
    dot = sum(first[index] * second[index] for index in range(4))
    if dot < 0.0:
        second = tuple(-value for value in second)  # type: ignore[assignment]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return _normalize_quaternion(
            tuple(
                first[index]
                + normalized_fraction * (second[index] - first[index])
                for index in range(4)
            ),
            name="interpolated quaternion",
        )
    angle = math.acos(dot)
    scale = math.sin(angle)
    start_weight = math.sin((1.0 - normalized_fraction) * angle) / scale
    end_weight = math.sin(normalized_fraction * angle) / scale
    return _normalize_quaternion(
        tuple(
            start_weight * first[index] + end_weight * second[index]
            for index in range(4)
        ),
        name="interpolated quaternion",
    )


def interpolate_pose(start: Pose, end: Pose, fraction: float) -> Pose:
    """线性插值位置并以最短弧插值姿态。"""
    if type(start) is not Pose or type(end) is not Pose:
        raise ValueError("start and end must be exact Pose values")
    if isinstance(fraction, bool) or not isinstance(fraction, Real):
        raise ValueError("fraction must be a finite number in range 0..1")
    normalized_fraction = float(fraction)
    if not math.isfinite(normalized_fraction) or not 0.0 <= normalized_fraction <= 1.0:
        raise ValueError("fraction must be a finite number in range 0..1")
    position = tuple(
        start.position[index]
        + normalized_fraction * (end.position[index] - start.position[index])
        for index in range(3)
    )
    return Pose(
        position,  # type: ignore[arg-type]
        slerp_shortest(start.orientation, end.orientation, normalized_fraction),
    )


def deskew_lidar_frame(
    cloud: LidarPointCloudV2,
    start: RecoveredPoseNode,
    lookahead: RecoveredPoseNode,
) -> tuple[DeskewedPoint, ...]:
    """按每点 firing 时刻插值 LiDAR 位姿并转换到世界坐标。"""
    if type(cloud) is not LidarPointCloudV2:
        raise ValueError("cloud must be an exact LidarPointCloudV2")
    if type(start) is not RecoveredPoseNode or type(lookahead) is not RecoveredPoseNode:
        raise ValueError("start and lookahead must be exact RecoveredPoseNode values")
    if cloud.frame_id != "lidar_link" or cloud.lidar_id != 1:
        raise ValueError("cloud frame_id/lidar_id must identify the center lidar_link")
    if start.timestamp_ns != cloud.timebase_ns:
        raise ValueError("start pose timestamp must equal the LiDAR timebase")
    expected_lookahead_ns = cloud.timebase_ns + _LIDAR_FRAME_PERIOD_NS
    if expected_lookahead_ns > _UINT64_MAX:
        raise ValueError("LiDAR frame timestamps must fit uint64")
    previous_offset = -1
    for point in cloud.points:
        if not _is_realtime_mid360_offset(point.offset_time_ns) or point.offset_time_ns <= previous_offset:
            raise ValueError(
                "offset_time_ns must be strictly increasing on the realtime MID-360 firing grid"
            )
        if cloud.timebase_ns + point.offset_time_ns > _UINT64_MAX:
            raise ValueError("LiDAR point timestamps must fit uint64")
        previous_offset = point.offset_time_ns
    if lookahead.timestamp_ns != expected_lookahead_ns:
        raise MissingPoseLookaheadError(
            "recorded pose look-ahead must be the next 100 ms pose node"
        )
    duration_ns = lookahead.timestamp_ns - start.timestamp_ns
    deskewed: list[DeskewedPoint] = []
    for point in cloud.points:
        timestamp_ns = cloud.timebase_ns + point.offset_time_ns
        fraction = (
            0.0
            if duration_ns == 0
            else (timestamp_ns - start.timestamp_ns) / duration_ns
        )
        pose = interpolate_pose(start.lidar_pose, lookahead.lidar_pose, fraction)
        position = _transform(pose, (point.x, point.y, point.z))
        deskewed.append(
            DeskewedPoint(
                timestamp_ns,
                position,
                point.reflectivity,
                point.tag,
                point.line,
            )
        )
    return tuple(deskewed)


def _is_realtime_mid360_offset(offset: int) -> bool:
    """验证稀疏命中仍来自正式 5,760 slot 的均分 firing 时间表。"""
    if not 0 <= offset <= _LIDAR_LAST_OFFSET_NS:
        return False
    slot = (
        offset * _LIDAR_FIRING_SLOT_COUNT + LIDAR_SCAN_PERIOD_NS - 1
    ) // LIDAR_SCAN_PERIOD_NS
    return slot < _LIDAR_FIRING_SLOT_COUNT and mid360_offset_time_ns(slot) == offset


class WorldMapAccumulator:
    """按 5 cm 体素累计永久层，并保留短时运动层。"""

    def __init__(
        self,
        *,
        minimum: Vec3,
        maximum: Vec3,
        voxel_size_m: float = 0.05,
        static_display_limit: int = _MAX_STATIC_DISPLAY_POINTS,
        moving_ttl_ns: int = 300_000_000,
    ) -> None:
        self._minimum = _require_finite_vector("minimum", minimum)
        self._maximum = _require_finite_vector("maximum", maximum)
        if any(
            self._minimum[index] >= self._maximum[index] for index in range(3)
        ):
            raise ValueError("minimum must be strictly below maximum")
        if (
            isinstance(voxel_size_m, bool)
            or not isinstance(voxel_size_m, Real)
            or not math.isfinite(float(voxel_size_m))
            or float(voxel_size_m) <= 0.0
        ):
            raise ValueError("voxel_size_m must be positive and finite")
        if (
            isinstance(static_display_limit, bool)
            or not isinstance(static_display_limit, int)
            or not 1 <= static_display_limit <= _MAX_STATIC_DISPLAY_POINTS
        ):
            raise ValueError("static_display_limit must be in range 1..500000")
        if (
            isinstance(moving_ttl_ns, bool)
            or not isinstance(moving_ttl_ns, int)
            or moving_ttl_ns <= 0
        ):
            raise ValueError("moving_ttl_ns must be a positive integer")
        self._voxel_size_m = float(voxel_size_m)
        self._static_display_limit = static_display_limit
        self._moving_ttl_ns = moving_ttl_ns
        self._permanent: dict[tuple[int, int, int], DeskewedPoint] = {}
        self._moving: dict[tuple[int, int, int], DeskewedPoint] = {}
        self._static_cache: tuple[np.ndarray, np.ndarray] | None = None
        self._latest_frame_time_ns: int | None = None

    @staticmethod
    def _require_frame_time(frame_time_ns: object) -> int:
        if (
            isinstance(frame_time_ns, bool)
            or not isinstance(frame_time_ns, int)
            or frame_time_ns < 0
        ):
            raise ValueError("frame_time_ns must be a nonnegative integer")
        return frame_time_ns

    def _inside_bounds(self, position: Vec3) -> bool:
        return all(
            self._minimum[index] <= position[index] <= self._maximum[index]
            for index in range(3)
        )

    def _voxel_key(self, position: Vec3) -> tuple[int, int, int]:
        return tuple(
            math.floor(
                (position[index] - self._minimum[index]) / self._voxel_size_m
            )
            for index in range(3)
        )  # type: ignore[return-value]

    @staticmethod
    def _voxel_score(key: tuple[int, int, int]) -> int:
        """固定整数散列用于显示抽样，结果不受 Python hash seed 影响。"""
        x, y, z = key
        return (
            (x * 73_856_093) ^ (y * 19_349_663) ^ (z * 83_492_791)
        ) & ((1 << 64) - 1)

    def add_frame(
        self,
        points: Sequence[DeskewedPoint],
        *,
        frame_time_ns: int,
    ) -> None:
        """按稳定输入顺序加入一帧，不让重复播放扩大永久层。"""
        timestamp = self._require_frame_time(frame_time_ns)
        if not isinstance(points, (tuple, list)):
            raise ValueError("points must be an ordered finite sequence")
        if any(type(point) is not DeskewedPoint for point in points):
            raise ValueError("points must contain only exact DeskewedPoint values")
        permanent_changed = False
        for point in points:
            if not self._inside_bounds(point.position):
                continue
            key = self._voxel_key(point.position)
            if point.tag in (1, 2):
                if key not in self._permanent:
                    self._permanent[key] = point
                    permanent_changed = True
            elif point.tag == 3:
                existing = self._moving.get(key)
                if existing is None or point.timestamp_ns >= existing.timestamp_ns:
                    self._moving[key] = point
        if permanent_changed:
            self._static_cache = None
        self._latest_frame_time_ns = (
            timestamp
            if self._latest_frame_time_ns is None
            else max(timestamp, self._latest_frame_time_ns)
        )
        self._expire_moving(timestamp)

    def _expire_moving(self, timestamp_ns: int) -> None:
        expired = tuple(
            key
            for key, point in self._moving.items()
            if timestamp_ns - point.timestamp_ns > self._moving_ttl_ns
        )
        for key in expired:
            del self._moving[key]

    @staticmethod
    def _readonly_array(values: object, *, dtype: object, shape: tuple[int, ...]) -> np.ndarray:
        array = np.asarray(values, dtype=dtype)
        if array.size == 0:
            array = np.empty(shape, dtype=dtype)
        else:
            array = np.ascontiguousarray(array.reshape(shape))
        array.setflags(write=False)
        return array

    def _static_arrays(self) -> tuple[np.ndarray, np.ndarray]:
        if self._static_cache is not None:
            return self._static_cache
        keys = tuple(self._permanent)
        if len(keys) > self._static_display_limit:
            keys = tuple(
                sorted(keys, key=lambda key: (self._voxel_score(key), key))[
                    : self._static_display_limit
                ]
            )
        points = tuple(self._permanent[key] for key in keys)
        positions = self._readonly_array(
            tuple(point.position for point in points),
            dtype=np.float32,
            shape=(len(points), 3),
        )
        tags = self._readonly_array(
            tuple(point.tag for point in points),
            dtype=np.uint8,
            shape=(len(points),),
        )
        self._static_cache = (positions, tags)
        return self._static_cache

    def snapshot(self, *, frame_time_ns: int) -> WorldMapSnapshot:
        """在给定仿真时刻清理 TTL 并生成确定性只读显示数组。"""
        timestamp = self._require_frame_time(frame_time_ns)
        self._expire_moving(timestamp)
        permanent_positions, permanent_tags = self._static_arrays()
        moving_points = tuple(self._moving[key] for key in sorted(self._moving))
        moving_positions = self._readonly_array(
            tuple(point.position for point in moving_points),
            dtype=np.float32,
            shape=(len(moving_points), 3),
        )
        moving_tags = self._readonly_array(
            tuple(point.tag for point in moving_points),
            dtype=np.uint8,
            shape=(len(moving_points),),
        )
        return WorldMapSnapshot(
            permanent_positions,
            permanent_tags,
            moving_positions,
            moving_tags,
            len(self._permanent),
        )

    def clear(self) -> None:
        """定位回退前清空全部派生状态。"""
        self._permanent.clear()
        self._moving.clear()
        self._static_cache = None
        self._latest_frame_time_ns = None


class PlaybackClock:
    """不跳逻辑帧的回放时钟与容量 1 后台请求状态机。"""

    def __init__(self, frame_times_ns: Sequence[int]) -> None:
        if not isinstance(frame_times_ns, (tuple, list)) or not frame_times_ns:
            raise ValueError("frame_times_ns must be a nonempty ordered sequence")
        times = tuple(frame_times_ns)
        if any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0
            for value in times
        ):
            raise ValueError("frame_times_ns must contain nonnegative integers")
        if any(
            current - previous != _LIDAR_FRAME_PERIOD_NS
            for previous, current in zip(times, times[1:])
        ):
            raise ValueError("LiDAR frame times must be exactly 100 ms apart")
        self._frame_times_ns = times
        self._frame_index = 0
        self._desired_index = 0
        self._paused = True
        self._rate = 1.0
        self._generation = 0
        self._pending_rebuild = False
        self._in_flight: PlaybackFrameRequest | None = None

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def frame_index(self) -> int:
        return self._frame_index

    @property
    def frame_interval_ns(self) -> int:
        return round(_LIDAR_FRAME_PERIOD_NS / self._rate)

    @property
    def rate(self) -> float:
        return self._rate

    def play(self) -> None:
        self._paused = False

    def pause(self) -> None:
        self._paused = True

    def set_rate(self, rate: float) -> None:
        if isinstance(rate, bool) or not isinstance(rate, Real):
            raise ValueError("rate must be one of 0.25, 0.5, 1, 2, 4")
        normalized = float(rate)
        if normalized not in _PLAYBACK_RATES:
            raise ValueError("rate must be one of 0.25, 0.5, 1, 2, 4")
        self._rate = normalized

    def _request_if_idle(self) -> PlaybackFrameRequest | None:
        if self._in_flight is not None or self._desired_index == self._frame_index:
            return None
        request = PlaybackFrameRequest(
            self._generation,
            self._desired_index,
            self._pending_rebuild,
        )
        self._in_flight = request
        return request

    def _schedule(self, target: int, *, rebuild: bool) -> PlaybackFrameRequest | None:
        self._generation += 1
        self._desired_index = target
        self._pending_rebuild = rebuild
        return self._request_if_idle()

    def begin_next_frame(self) -> PlaybackFrameRequest | None:
        """播放时仅在无 in-flight 结果时请求相邻下一帧。"""
        if self._paused or self._in_flight is not None:
            return None
        if self._frame_index >= len(self._frame_times_ns) - 1:
            self._paused = True
            return None
        return self._schedule(self._frame_index + 1, rebuild=False)

    def step(self, delta: int) -> PlaybackFrameRequest | None:
        if isinstance(delta, bool) or not isinstance(delta, int) or delta not in (-1, 1):
            raise ValueError("delta must be -1 or 1")
        self._paused = True
        target = min(
            len(self._frame_times_ns) - 1,
            max(0, self._frame_index + delta),
        )
        return self._schedule(target, rebuild=target < self._frame_index)

    def seek(self, frame_index: int) -> PlaybackFrameRequest | None:
        if (
            isinstance(frame_index, bool)
            or not isinstance(frame_index, int)
            or not 0 <= frame_index < len(self._frame_times_ns)
        ):
            raise ValueError("frame_index is outside the recording")
        self._paused = True
        # 定位总是从开头确定性重建，避免复用错误的未来地图状态。
        return self._schedule(frame_index, rebuild=True)

    def begin_pending_frame(self) -> PlaybackFrameRequest | None:
        return self._request_if_idle()

    def complete(self, request: PlaybackFrameRequest) -> bool:
        """只接受当前 generation 的结果；过期结果仅释放队列容量。"""
        if type(request) is not PlaybackFrameRequest or request != self._in_flight:
            raise ValueError("request does not match the in-flight playback frame")
        self._in_flight = None
        accepted = (
            request.generation == self._generation
            and request.frame_index == self._desired_index
        )
        if not accepted:
            return False
        self._frame_index = request.frame_index
        self._pending_rebuild = False
        if self._frame_index == len(self._frame_times_ns) - 1:
            self._paused = True
        return True
