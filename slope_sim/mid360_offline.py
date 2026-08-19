"""离线 MID-360 高保真 firing schedule 与分时扫描执行器。"""

from __future__ import annotations

from dataclasses import dataclass

import math
import numpy as np

from slope_sim.lidar_pointcloud import (
    MID360_PATTERN_VERSION,
    _POINT_SEMANTICS,
    _RANGE_ABS_TOLERANCE_M,
    _RANGE_REL_TOLERANCE,
    _mid360_pattern_base,
    _mid360_unit_directions_from_rows,
    _rotation_matrix,
)
from slope_sim.interfaces.models import LidarPoint, LidarPointCloud
from slope_sim.scene import LIDAR_VISIBLE_GROUP
from slope_sim.sensor_backend import Pose, RayHit


_UINT64_MAX = (1 << 64) - 1
_PATTERN_ROW_COUNT = 800_000
_OFFLINE_FIRING_SLOT_COUNT = 20_000
_OFFLINE_FIRING_INTERVAL_NS = 5_000
_OFFLINE_FRAME_PERIOD_NS = 100_000_000
_PHYSICS_STEPS_PER_FRAME = 24
_OFFLINE_MIN_RANGE_M = 0.1
# 离线重建与实时中心 MID-360 同步扩大 50%，保留 20,000 slot 的高保真质量。
_OFFLINE_MAX_RANGE_M = 60.0


def _require_slot(slot: object) -> int:
    if type(slot) is not int or not 0 <= slot < _OFFLINE_FIRING_SLOT_COUNT:
        raise ValueError("slot must be an integer in [0, 19999]")
    return slot


def _require_sequence(sequence: object) -> int:
    if type(sequence) is not int or not 0 <= sequence <= _UINT64_MAX:
        raise ValueError("sequence must be a uint64")
    return sequence


@dataclass(frozen=True, slots=True)
class OfflineMid360Profile:
    """只供 Golf 离线采集使用的 200 kHz MID-360 合同。"""

    @classmethod
    def high_fidelity(cls) -> "OfflineMid360Profile":
        """返回唯一受支持的离线高保真 profile。"""
        return cls()

    @property
    def firing_slot_count(self) -> int:
        return _OFFLINE_FIRING_SLOT_COUNT

    @property
    def firing_interval_ns(self) -> int:
        return _OFFLINE_FIRING_INTERVAL_NS

    @property
    def frame_period_ns(self) -> int:
        return _OFFLINE_FRAME_PERIOD_NS

    @property
    def min_range_m(self) -> float:
        return _OFFLINE_MIN_RANGE_M

    @property
    def max_range_m(self) -> float:
        return _OFFLINE_MAX_RANGE_M

    @property
    def physics_step_slot_ranges(self) -> tuple[range, ...]:
        """按 firing offset 把一帧精确分入 24 个 240 Hz 状态区间。"""
        return tuple(
            range(
                (step * self.firing_slot_count + _PHYSICS_STEPS_PER_FRAME - 1)
                // _PHYSICS_STEPS_PER_FRAME,
                (
                    (step + 1) * self.firing_slot_count
                    + _PHYSICS_STEPS_PER_FRAME
                    - 1
                )
                // _PHYSICS_STEPS_PER_FRAME,
            )
            for step in range(_PHYSICS_STEPS_PER_FRAME)
        )


@dataclass(frozen=True, slots=True)
class OfflineMid360Schedule:
    """把离线帧 sequence/slot 映射到官方 pattern 行与仿真时间。"""

    profile: OfflineMid360Profile
    pattern_version: str
    world_generation: int

    def __post_init__(self) -> None:
        if type(self.profile) is not OfflineMid360Profile:
            raise ValueError("profile must be an OfflineMid360Profile")
        if self.pattern_version != MID360_PATTERN_VERSION:
            raise ValueError(
                f"pattern_version must be {MID360_PATTERN_VERSION!r}"
            )
        # 复用实时路径已经冻结的 version + generation phase 规则及校验。
        _mid360_pattern_base(self.pattern_version, self.world_generation)

    def offset_time_ns(self, slot: object) -> int:
        """返回帧内精确 5 us firing offset。"""
        return _require_slot(slot) * self.profile.firing_interval_ns

    def pattern_row_index(self, *, sequence: object, slot: object) -> int:
        """离线帧以 20,000 行为步幅连续遍历官方 4 s pattern。"""
        normalized_sequence = _require_sequence(sequence)
        normalized_slot = _require_slot(slot)
        base = _mid360_pattern_base(self.pattern_version, self.world_generation)
        return (
            base
            + normalized_sequence * self.profile.firing_slot_count
            + normalized_slot
        ) % _PATTERN_ROW_COUNT


@dataclass(frozen=True, slots=True)
class OfflineMid360AcceptanceTruth:
    """与最终原始点云严格同序的本帧 PyBullet 命中真值。"""

    world_positions: np.ndarray
    body_ids: np.ndarray
    hit_body_positions: np.ndarray

    def __post_init__(self) -> None:
        positions = np.array(self.world_positions, dtype=np.float64, order="C", copy=True)
        body_ids = np.array(self.body_ids, dtype=np.int32, order="C", copy=True)
        hit_body_positions = np.array(
            self.hit_body_positions,
            dtype=np.float64,
            order="C",
            copy=True,
        )
        if positions.ndim != 2 or positions.shape[1:] != (3,):
            raise ValueError("world_positions must have shape (N, 3)")
        if body_ids.ndim != 1 or body_ids.shape[0] != positions.shape[0]:
            raise ValueError("body_ids must have shape (N,)")
        if hit_body_positions.shape != positions.shape:
            raise ValueError("hit_body_positions must have shape (N, 3)")
        if not np.isfinite(positions).all() or np.any(body_ids < 0):
            raise ValueError("acceptance truth contains invalid hits")
        if hit_body_positions.size:
            finite_rows = np.isfinite(hit_body_positions).all(axis=1)
            missing_rows = np.isnan(hit_body_positions).all(axis=1)
            if not np.logical_or(finite_rows, missing_rows).all():
                raise ValueError("hit body positions must be finite rows or all-NaN rows")
        positions.setflags(write=False)
        body_ids.setflags(write=False)
        hit_body_positions.setflags(write=False)
        object.__setattr__(self, "world_positions", positions)
        object.__setattr__(self, "body_ids", body_ids)
        object.__setattr__(self, "hit_body_positions", hit_body_positions)

def _world_rays(
    directions: np.ndarray,
    pose: Pose,
    profile: OfflineMid360Profile,
) -> tuple[np.ndarray, np.ndarray]:
    """把一批局部单位方向变换成同一冻结 pose 下的世界射线。"""
    matrix = np.asarray(_rotation_matrix(pose.orientation), dtype=np.float64).reshape(3, 3)
    world_directions = np.ascontiguousarray(directions @ matrix.T, dtype=np.float64)
    origin = np.asarray(pose.position, dtype=np.float64)
    starts = np.ascontiguousarray(
        origin + world_directions * profile.min_range_m,
        dtype=np.float64,
    )
    ends = np.ascontiguousarray(
        origin + world_directions * profile.max_range_m,
        dtype=np.float64,
    )
    starts.setflags(write=False)
    ends.setflags(write=False)
    return starts, ends


def _measurement_from_hit(
    global_slot: int,
    hit: RayHit,
    local_point: tuple[float, float, float],
    profile: OfflineMid360Profile,
) -> tuple[float, float, float, int, int] | None:
    """复用现有点语义，并在 0.1..40 m 边界吸收单精度回差。"""
    distance = math.hypot(*local_point)
    if (
        distance < profile.min_range_m
        and not math.isclose(
            distance,
            profile.min_range_m,
            rel_tol=_RANGE_REL_TOLERANCE,
            abs_tol=_RANGE_ABS_TOLERANCE_M,
        )
    ) or (
        distance > profile.max_range_m
        and not math.isclose(
            distance,
            profile.max_range_m,
            rel_tol=_RANGE_REL_TOLERANCE,
            abs_tol=_RANGE_ABS_TOLERANCE_M,
        )
    ):
        return None
    try:
        tag, reflectivity = _POINT_SEMANTICS[hit.category]
    except KeyError as error:
        raise RuntimeError(
            f"ray {global_slot} has unsupported hit category {hit.category!r}"
        ) from error
    return (*local_point, reflectivity, tag)


class OfflineMid360FrameScanner:
    """在 24 个物理状态上累积一个保持运动畸变的原始 LiDAR 帧。"""

    def __init__(
        self,
        backend: object,
        schedule: OfflineMid360Schedule,
        *,
        sequence: object,
    ) -> None:
        if type(schedule) is not OfflineMid360Schedule:
            raise ValueError("schedule must be an OfflineMid360Schedule")
        for method_name in (
            "world_pose",
            "_ray_test_indexed_hits_ndarray",
            "inverse_transform_points_prevalidated",
        ):
            if not callable(getattr(backend, method_name, None)):
                raise ValueError(f"backend must provide {method_name}()")
        self._backend = backend
        self._schedule = schedule
        self._sequence = _require_sequence(sequence)
        self._next_step = 0
        self._points: list[LidarPoint] = []
        self._truth_positions: list[tuple[float, float, float]] = []
        self._truth_body_ids: list[int] = []
        self._truth_hit_body_positions: list[tuple[float, float, float]] = []
        self._acceptance_truth: OfflineMid360AcceptanceTruth | None = None
        self._finalized = False

    def capture_step(
        self,
        step: object,
        *,
        body_positions_by_id: object,
    ) -> int:
        """冻结本步 `lidar_link` pose，并执行唯一一批 833/834 条射线。"""
        if type(step) is not int or step != self._next_step or not 0 <= step < 24:
            raise ValueError(f"step must be the next physics step {self._next_step}")
        if self._finalized:
            raise RuntimeError("offline MID-360 frame is already finalized")
        if type(body_positions_by_id) is not dict:
            raise ValueError("body_positions_by_id must be a dict")
        normalized_body_positions: dict[int, tuple[float, float, float]] = {}
        for body_id, position in body_positions_by_id.items():
            if (
                type(body_id) is not int
                or body_id < 0
                or not isinstance(position, (tuple, list))
                or len(position) != 3
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    for value in position
                )
            ):
                raise ValueError("body position mappings must contain finite Vec3 values")
            normalized_body_positions[body_id] = tuple(float(value) for value in position)
        slot_range = self._schedule.profile.physics_step_slot_ranges[step]
        rows = tuple(
            self._schedule.pattern_row_index(sequence=self._sequence, slot=slot)
            for slot in slot_range
        )
        directions = _mid360_unit_directions_from_rows(rows)
        pose = self._backend.world_pose("lidar_link")
        if type(pose) is not Pose:
            raise RuntimeError("backend world_pose must return Pose")
        starts, ends = _world_rays(directions, pose, self._schedule.profile)
        indexed_hits = self._backend._ray_test_indexed_hits_ndarray(
            starts,
            ends,
            collision_mask=LIDAR_VISIBLE_GROUP,
            num_threads=0,
        )
        if type(indexed_hits) is not tuple:
            raise RuntimeError("backend indexed hits must be a tuple")
        previous_index = -1
        normalized_hits: list[tuple[int, RayHit]] = []
        for batch_index, hit in indexed_hits:
            if (
                type(batch_index) is not int
                or not previous_index < batch_index < len(slot_range)
                or type(hit) is not RayHit
                or not hit.hit
            ):
                raise RuntimeError("backend returned invalid indexed hits")
            normalized_hits.append((batch_index, hit))
            previous_index = batch_index
        if normalized_hits:
            world_points = tuple(hit.hit_position for _, hit in normalized_hits)
            local_points = self._backend.inverse_transform_points_prevalidated(
                pose,
                world_points,
            )
            if type(local_points) is not tuple or len(local_points) != len(normalized_hits):
                raise RuntimeError("backend inverse transform returned the wrong point count")
            for (batch_index, hit), local_point in zip(
                normalized_hits,
                local_points,
                strict=True,
            ):
                global_slot = slot_range.start + batch_index
                measurement = _measurement_from_hit(
                    global_slot,
                    hit,
                    local_point,
                    self._schedule.profile,
                )
                if measurement is None:
                    continue
                self._points.append(
                    LidarPoint(
                        self._schedule.offset_time_ns(global_slot),
                        *measurement,
                        self._schedule.pattern_row_index(
                            sequence=self._sequence,
                            slot=global_slot,
                        )
                        % 4,
                        )
                    )
                self._truth_positions.append(hit.hit_position)
                self._truth_body_ids.append(hit.body_id)
                body_position = normalized_body_positions.get(hit.body_id)
                if measurement[4] in (2, 3) and body_position is None:
                    raise RuntimeError("obstacle hit is missing its frozen body position")
                self._truth_hit_body_positions.append(
                    (math.nan, math.nan, math.nan)
                    if body_position is None
                    else body_position
                )
        self._next_step += 1
        return len(normalized_hits)

    def finalize(self, *, timebase_ns: object) -> LidarPointCloud:
        """只在 24 步完整后冻结消息；扫描失败或缺步不得伪装成功。"""
        if self._finalized:
            raise RuntimeError("offline MID-360 frame is already finalized")
        if self._next_step != 24:
            raise RuntimeError("offline MID-360 frame requires all 24 physics steps")
        self._finalized = True
        points = tuple(self._points)
        self._acceptance_truth = OfflineMid360AcceptanceTruth(
            np.asarray(self._truth_positions, dtype=np.float64).reshape((-1, 3)),
            np.asarray(self._truth_body_ids, dtype=np.int32),
            np.asarray(self._truth_hit_body_positions, dtype=np.float64).reshape((-1, 3)),
        )
        return LidarPointCloud(
            timebase_ns,
            "lidar_link",
            len(points),
            1,
            points,
        )

    def acceptance_truth(self) -> OfflineMid360AcceptanceTruth:
        """返回最终点云同序真值；未完成帧不得泄露半帧状态。"""
        if self._acceptance_truth is None:
            raise RuntimeError("offline MID-360 frame is not finalized")
        return self._acceptance_truth
