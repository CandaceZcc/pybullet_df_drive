"""MID-360 Golf 采集真值与严格 MCAP 的有界数值验收。"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from numbers import Real
from typing import Iterator, Protocol, Sequence

import numpy as np

from slope_sim.interfaces.v2.models import LidarPointCloudV2
from slope_sim.mapping_replay import (
    RecoveredPoseNode,
    WorldMapAccumulator,
    deskew_lidar_frame,
)
from slope_sim.mid360_offline import OfflineMid360AcceptanceTruth
from slope_sim.obstacles import ObstacleSnapshot, ObstacleSpec
from slope_sim.scene import TerrainBounds


_ACTIVE_SPEED_THRESHOLD_M_S = 0.1
_DESKEW_THRESHOLD_M = 0.05
_DESKEW_BIN_WIDTH_M = 0.0001
_MOVING_POSITION_BUCKET_M = 0.05
_LIDAR_FRAME_PERIOD_NS = 100_000_000
_MAX_STATIC_DISPLAY_POINTS = 500_000


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be finite")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be finite")
    return normalized


def _nearest_rank_p95(values: Sequence[float]) -> float:
    if not values:
        raise ValueError("P95 requires at least one sample")
    ordered = sorted(_finite("P95 sample", value) for value in values)
    return ordered[math.ceil(0.95 * len(ordered)) - 1]


@dataclass(frozen=True, slots=True)
class ObstacleAcceptanceMetrics:
    logical_id: int
    mode: str
    hit_frame_count: int
    position_bucket_count: int
    position_span_m: float


@dataclass(frozen=True, slots=True)
class CaptureAcceptanceMetrics:
    eligible_motion_frames: int
    moving_motion_frames: int
    active_speed_ratio: float | None
    deskew_point_count: int
    deskew_within_5cm_count: int
    deskew_error_p95_upper_bound_m: float | None
    obstacles: tuple[ObstacleAcceptanceMetrics, ...]


def capture_acceptance_document(metrics: CaptureAcceptanceMetrics) -> dict[str, object]:
    """把内存统计转换为 simulator 可持久化的紧凑稳定结构。"""
    if type(metrics) is not CaptureAcceptanceMetrics:
        raise ValueError("metrics must be exact CaptureAcceptanceMetrics")
    return {
        "motion": {
            "eligible_frame_count": metrics.eligible_motion_frames,
            "speed_above_0_1_m_s_frame_count": metrics.moving_motion_frames,
            "speed_above_0_1_m_s_ratio": metrics.active_speed_ratio,
        },
        "deskew": {
            "point_count": metrics.deskew_point_count,
            "within_0_05_m_count": metrics.deskew_within_5cm_count,
            "error_p95_upper_bound_m": metrics.deskew_error_p95_upper_bound_m,
        },
        "obstacles": [
            {
                "logical_id": item.logical_id,
                "mode": item.mode,
                "hit_frame_count": item.hit_frame_count,
                "position_bucket_count": item.position_bucket_count,
                "position_span_m": item.position_span_m,
            }
            for item in metrics.obstacles
        ],
    }


def _nonnegative_int(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def parse_capture_acceptance(document: object) -> CaptureAcceptanceMetrics:
    """严格恢复 simulator 真值统计，拒绝缺字段、NaN 和自相矛盾比率。"""
    if not isinstance(document, dict) or set(document) != {
        "motion",
        "deskew",
        "obstacles",
    }:
        raise ValueError("truth_acceptance fields are invalid")
    motion = document["motion"]
    deskew = document["deskew"]
    obstacles = document["obstacles"]
    if not isinstance(motion, dict) or set(motion) != {
        "eligible_frame_count",
        "speed_above_0_1_m_s_frame_count",
        "speed_above_0_1_m_s_ratio",
    }:
        raise ValueError("truth_acceptance motion fields are invalid")
    if not isinstance(deskew, dict) or set(deskew) != {
        "point_count",
        "within_0_05_m_count",
        "error_p95_upper_bound_m",
    }:
        raise ValueError("truth_acceptance deskew fields are invalid")
    eligible = _nonnegative_int("eligible_frame_count", motion["eligible_frame_count"])
    moving = _nonnegative_int(
        "speed_above_0_1_m_s_frame_count",
        motion["speed_above_0_1_m_s_frame_count"],
    )
    if moving > eligible:
        raise ValueError("moving motion frames exceed eligible frames")
    ratio_value = motion["speed_above_0_1_m_s_ratio"]
    if eligible == 0:
        if moving != 0 or ratio_value is not None:
            raise ValueError("empty motion statistics must use a null ratio")
        ratio = None
    else:
        ratio = _finite("speed_above_0_1_m_s_ratio", ratio_value)
        if not 0.0 <= ratio <= 1.0 or not math.isclose(
            ratio,
            moving / eligible,
            rel_tol=0.0,
            abs_tol=1e-15,
        ):
            raise ValueError("motion ratio differs from its counts")
    point_count = _nonnegative_int("deskew point_count", deskew["point_count"])
    within_count = _nonnegative_int(
        "within_0_05_m_count",
        deskew["within_0_05_m_count"],
    )
    if within_count > point_count:
        raise ValueError("deskew within-threshold count exceeds point count")
    p95_value = deskew["error_p95_upper_bound_m"]
    if point_count == 0:
        if within_count != 0 or p95_value is not None:
            raise ValueError("empty deskew statistics must use a null P95")
        p95_upper = None
    else:
        p95_upper = _finite("error_p95_upper_bound_m", p95_value)
        if p95_upper < 0.0:
            raise ValueError("deskew P95 must be nonnegative")

    if not isinstance(obstacles, list) or not obstacles:
        raise ValueError("truth_acceptance obstacles must be a nonempty list")
    parsed_obstacles: list[ObstacleAcceptanceMetrics] = []
    seen_ids: set[int] = set()
    expected_keys = {
        "logical_id",
        "mode",
        "hit_frame_count",
        "position_bucket_count",
        "position_span_m",
    }
    for item in obstacles:
        if not isinstance(item, dict) or set(item) != expected_keys:
            raise ValueError("truth_acceptance obstacle fields are invalid")
        logical_id = _nonnegative_int("obstacle logical_id", item["logical_id"])
        if logical_id < 1 or logical_id in seen_ids:
            raise ValueError("obstacle logical ids must be unique and positive")
        seen_ids.add(logical_id)
        mode = item["mode"]
        if mode not in {"static", "moving"}:
            raise ValueError("obstacle mode is invalid")
        hit_frames = _nonnegative_int("hit_frame_count", item["hit_frame_count"])
        bucket_count = _nonnegative_int(
            "position_bucket_count",
            item["position_bucket_count"],
        )
        span = _finite("position_span_m", item["position_span_m"])
        if span < 0.0 or (mode == "static" and (bucket_count != 0 or span != 0.0)):
            raise ValueError("obstacle position statistics are invalid")
        parsed_obstacles.append(
            ObstacleAcceptanceMetrics(
                logical_id,
                mode,
                hit_frames,
                bucket_count,
                span,
            )
        )
    if tuple(item.logical_id for item in parsed_obstacles) != tuple(sorted(seen_ids)):
        raise ValueError("truth_acceptance obstacles must be ordered by logical id")
    return CaptureAcceptanceMetrics(
        eligible,
        moving,
        ratio,
        point_count,
        within_count,
        p95_upper,
        tuple(parsed_obstacles),
    )


class GolfCaptureAcceptance:
    """逐帧消费瞬时真值，不保留跨帧原始点或世界坐标。"""

    def __init__(self, obstacle_snapshots: Sequence[ObstacleSnapshot]) -> None:
        snapshots = tuple(obstacle_snapshots)
        if not snapshots or any(type(item) is not ObstacleSnapshot for item in snapshots):
            raise ValueError("obstacle_snapshots must contain exact snapshots")
        if any(item.body_id is None for item in snapshots):
            raise ValueError("acceptance obstacles must include body ids")
        self._obstacle_by_body = {int(item.body_id): item for item in snapshots}
        if len(self._obstacle_by_body) != len(snapshots):
            raise ValueError("acceptance obstacle body ids must be unique")
        self._hit_frames = {item.logical_id: 0 for item in snapshots}
        self._position_buckets = {
            item.logical_id: set() for item in snapshots if item.mode == "moving"
        }
        self._first_positions: dict[int, tuple[float, float, float]] = {}
        self._position_spans = {logical_id: 0.0 for logical_id in self._position_buckets}
        self._eligible_motion_frames = 0
        self._moving_motion_frames = 0
        self._deskew_point_count = 0
        self._deskew_within_threshold_count = 0
        self._deskew_error_bins: Counter[int] = Counter()

    def observe_motion_frame(self, *, eligible: bool, actual_speed_m_s: float) -> None:
        """只对已排除启动、圆弧和停车尾段的 LiDAR 帧统计速度。"""
        if type(eligible) is not bool:
            raise ValueError("eligible must be a bool")
        speed = _finite("actual_speed_m_s", actual_speed_m_s)
        if speed < 0.0:
            raise ValueError("actual_speed_m_s must be nonnegative")
        if not eligible:
            return
        self._eligible_motion_frames += 1
        if speed > _ACTIVE_SPEED_THRESHOLD_M_S:
            self._moving_motion_frames += 1

    def observe_lidar_frame(
        self,
        *,
        cloud: LidarPointCloudV2,
        truth: OfflineMid360AcceptanceTruth,
        start: RecoveredPoseNode,
        lookahead: RecoveredPoseNode,
        obstacle_snapshots: Sequence[ObstacleSnapshot],
    ) -> None:
        """比较逐点 deskew，并把同一障碍在一帧内最多计数一次。"""
        if type(cloud) is not LidarPointCloudV2:
            raise ValueError("cloud must be an exact LidarPointCloudV2")
        if type(truth) is not OfflineMid360AcceptanceTruth:
            raise ValueError("truth must be exact OfflineMid360AcceptanceTruth")
        if len(cloud.points) != truth.world_positions.shape[0]:
            raise ValueError("truth must stay aligned with the finalized cloud")
        current = tuple(obstacle_snapshots)
        if any(type(item) is not ObstacleSnapshot or item.body_id is None for item in current):
            raise ValueError("current obstacle snapshots must include body ids")
        current_by_body = {int(item.body_id): item for item in current}
        if set(current_by_body) != set(self._obstacle_by_body):
            raise ValueError("obstacle body identity changed during capture")

        deskewed = deskew_lidar_frame(cloud, start, lookahead)
        if deskewed:
            positions = np.asarray(
                tuple(point.position for point in deskewed),
                dtype=np.float64,
            )
            errors = np.linalg.norm(positions - truth.world_positions, axis=1)
            if not np.isfinite(errors).all():
                raise RuntimeError("deskew comparison produced non-finite errors")
            self._deskew_point_count += int(errors.size)
            self._deskew_within_threshold_count += int(
                np.count_nonzero(errors <= _DESKEW_THRESHOLD_M)
            )
            bins = np.ceil(errors / _DESKEW_BIN_WIDTH_M).astype(np.int64)
            unique, counts = np.unique(bins, return_counts=True)
            self._deskew_error_bins.update(
                {int(index): int(count) for index, count in zip(unique, counts, strict=True)}
            )

        hit_logical_ids: set[int] = set()
        moving_hit_positions: dict[int, list[tuple[float, float, float]]] = {}
        for point, body_id_value, body_position_value in zip(
            cloud.points,
            truth.body_ids,
            truth.hit_body_positions,
            strict=True,
        ):
            body_id = int(body_id_value)
            obstacle = self._obstacle_by_body.get(body_id)
            if obstacle is None:
                if point.tag in (2, 3):
                    raise RuntimeError("obstacle-tagged point has no canonical body identity")
                continue
            expected_tag = 2 if obstacle.mode == "static" else 3
            if point.tag != expected_tag:
                raise RuntimeError("obstacle truth disagrees with the LiDAR point tag")
            hit_logical_ids.add(obstacle.logical_id)
            if not np.isfinite(body_position_value).all():
                raise RuntimeError("obstacle hit is missing its frozen body position")
            if obstacle.mode == "moving":
                moving_hit_positions.setdefault(obstacle.logical_id, []).append(
                    tuple(float(value) for value in body_position_value)
                )

        for logical_id in hit_logical_ids:
            self._hit_frames[logical_id] += 1
            initial = next(
                item for item in self._obstacle_by_body.values() if item.logical_id == logical_id
            )
            if initial.mode != "moving":
                continue
            for position in moving_hit_positions[logical_id]:
                bucket = tuple(
                    round(position[index] / _MOVING_POSITION_BUCKET_M)
                    for index in range(2)
                )
                self._position_buckets[logical_id].add(bucket)
                first = self._first_positions.setdefault(logical_id, position)
                self._position_spans[logical_id] = max(
                    self._position_spans[logical_id],
                    math.dist(first, position),
                )

    def snapshot(self) -> CaptureAcceptanceMetrics:
        ratio = (
            None
            if self._eligible_motion_frames == 0
            else self._moving_motion_frames / self._eligible_motion_frames
        )
        p95_upper = None
        if self._deskew_point_count:
            target = math.ceil(0.95 * self._deskew_point_count)
            cumulative = 0
            for bin_index in sorted(self._deskew_error_bins):
                cumulative += self._deskew_error_bins[bin_index]
                if cumulative >= target:
                    p95_upper = bin_index * _DESKEW_BIN_WIDTH_M
                    break
        obstacles = tuple(
            ObstacleAcceptanceMetrics(
                logical_id=item.logical_id,
                mode=item.mode,
                hit_frame_count=self._hit_frames[item.logical_id],
                position_bucket_count=len(self._position_buckets.get(item.logical_id, ())),
                position_span_m=self._position_spans.get(item.logical_id, 0.0),
            )
            for item in sorted(self._obstacle_by_body.values(), key=lambda value: value.logical_id)
        )
        return CaptureAcceptanceMetrics(
            self._eligible_motion_frames,
            self._moving_motion_frames,
            ratio,
            self._deskew_point_count,
            self._deskew_within_threshold_count,
            p95_upper,
            obstacles,
        )


class _MappingIndex(Protocol):
    lidar_frame_times_ns: tuple[int, ...]
    pose_nodes: tuple[RecoveredPoseNode, ...]

    def iter_lidar_frames(self) -> Iterator[LidarPointCloudV2]: ...


class _Route(Protocol):
    length: float
    duration_s: float

    def project(self, x: float, y: float) -> object: ...


@dataclass(frozen=True, slots=True)
class MappingAcceptanceMetrics:
    route_sample_count: int
    route_error_p95_m: float
    route_final_remaining_m: float
    terrain_eligible_cell_count: int
    terrain_covered_cell_count: int
    terrain_coverage_ratio: float
    permanent_voxel_count: int
    displayed_static_point_count: int


ACCEPTANCE_THRESHOLDS = {
    "route_error_p95_max_m": 0.35,
    "route_final_remaining_max_m": 0.75,
    "active_speed_ratio_min": 0.95,
    "terrain_coverage_ratio_min": 0.95,
    "moving_obstacle_min_hit_frames": 10,
    "moving_obstacle_min_position_buckets": 2,
    "deskew_error_p95_max_m": 0.05,
    "static_display_point_max": _MAX_STATIC_DISPLAY_POINTS,
}


def acceptance_failures(
    capture: CaptureAcceptanceMetrics,
    mapping: MappingAcceptanceMetrics,
) -> tuple[str, ...]:
    """按设计冻结阈值返回稳定、可写入证据文件的失败键。"""
    if type(capture) is not CaptureAcceptanceMetrics or type(mapping) is not MappingAcceptanceMetrics:
        raise ValueError("acceptance metrics must use exact metric types")
    failures: list[str] = []
    if mapping.route_sample_count <= 0 or mapping.route_error_p95_m > 0.35:
        failures.append("route_error_p95")
    if mapping.route_final_remaining_m > 0.75:
        failures.append("route_incomplete")
    if (
        capture.eligible_motion_frames <= 0
        or capture.active_speed_ratio is None
        or capture.active_speed_ratio < 0.95
    ):
        failures.append("active_speed_ratio")
    if (
        mapping.terrain_eligible_cell_count <= 0
        or mapping.terrain_coverage_ratio < 0.95
    ):
        failures.append("terrain_coverage_ratio")
    static = tuple(item for item in capture.obstacles if item.mode == "static")
    moving = tuple(item for item in capture.obstacles if item.mode == "moving")
    if (
        tuple(item.logical_id for item in static) != (1, 2, 3, 4, 5, 6)
        or any(item.hit_frame_count < 1 for item in static)
    ):
        failures.append("static_obstacle_hits")
    if (
        tuple(item.logical_id for item in moving) != (7, 8, 9)
        or any(item.hit_frame_count < 10 for item in moving)
    ):
        failures.append("moving_obstacle_hit_frames")
    if any(
        item.position_bucket_count < 2 or item.position_span_m <= 0.0
        for item in moving
    ):
        failures.append("moving_obstacle_positions")
    if (
        capture.deskew_point_count <= 0
        or capture.deskew_error_p95_upper_bound_m is None
        or capture.deskew_error_p95_upper_bound_m > 0.05
        or capture.deskew_within_5cm_count / capture.deskew_point_count < 0.95
    ):
        failures.append("deskew_error_p95")
    if mapping.displayed_static_point_count > _MAX_STATIC_DISPLAY_POINTS:
        failures.append("static_display_point_count")
    return tuple(failures)


def _obstacle_footprint(obstacle: ObstacleSpec) -> tuple[float, float, float, float]:
    half_x, half_y, _half_z = obstacle.geometry.half_extents
    points = (
        ((obstacle.position[0], obstacle.position[1]),)
        if obstacle.path is None
        else (obstacle.path.start_xy, obstacle.path.end_xy)
    )
    return (
        min(point[0] for point in points) - half_x,
        max(point[0] for point in points) + half_x,
        min(point[1] for point in points) - half_y,
        max(point[1] for point in points) + half_y,
    )


def _eligible_terrain_cells(
    bounds: TerrainBounds,
    obstacles: Sequence[ObstacleSpec],
    *,
    safety_edge_m: float,
    terrain_grid_m: float,
) -> tuple[set[tuple[int, int]], float, float, int, int]:
    minimum_x = bounds.min_x + safety_edge_m
    maximum_x = bounds.max_x - safety_edge_m
    minimum_y = bounds.min_y + safety_edge_m
    maximum_y = bounds.max_y - safety_edge_m
    x_count = math.floor((maximum_x - minimum_x) / terrain_grid_m + 1e-12)
    y_count = math.floor((maximum_y - minimum_y) / terrain_grid_m + 1e-12)
    if x_count <= 0 or y_count <= 0:
        raise ValueError("terrain bounds leave no acceptance cells")
    footprints = tuple(_obstacle_footprint(item) for item in obstacles)
    cells = {
        (x_index, y_index)
        for y_index in range(y_count)
        for x_index in range(x_count)
        if not any(
            footprint[0]
            <= minimum_x + (x_index + 0.5) * terrain_grid_m
            <= footprint[1]
            and footprint[2]
            <= minimum_y + (y_index + 0.5) * terrain_grid_m
            <= footprint[3]
            for footprint in footprints
        )
    }
    if not cells:
        raise ValueError("obstacle footprints remove every terrain acceptance cell")
    return cells, minimum_x, minimum_y, x_count, y_count


def evaluate_mapping_session(
    index: _MappingIndex,
    *,
    route: _Route,
    bounds: TerrainBounds,
    obstacles: Sequence[ObstacleSpec],
    safety_edge_m: float = 1.0,
    terrain_grid_m: float = 0.25,
) -> MappingAcceptanceMetrics:
    """单遍 deskew 严格 LiDAR 流并派生路线、覆盖与最终地图计数。"""
    if not isinstance(bounds, TerrainBounds):
        raise ValueError("bounds must be TerrainBounds")
    edge = _finite("safety_edge_m", safety_edge_m)
    grid = _finite("terrain_grid_m", terrain_grid_m)
    if edge < 0.0 or grid <= 0.0:
        raise ValueError("acceptance edge/grid must be positive")
    obstacle_values = tuple(obstacles)
    if any(type(item) is not ObstacleSpec for item in obstacle_values):
        raise ValueError("obstacles must contain exact ObstacleSpec values")
    frame_times = tuple(index.lidar_frame_times_ns)
    pose_nodes = tuple(index.pose_nodes)
    if not frame_times:
        raise ValueError("mapping acceptance requires LiDAR frames")
    pose_by_time = {node.timestamp_ns: node for node in pose_nodes}
    if len(pose_by_time) != len(pose_nodes):
        raise ValueError("mapping acceptance pose timestamps must be unique")
    route_duration_s = _finite("route duration", getattr(route, "duration_s", None))
    route_length_m = _finite("route length", getattr(route, "length", None))
    if route_duration_s < 0.0 or route_length_m <= 0.0:
        raise ValueError("mapping acceptance route duration/length must be positive")
    route_deadline_ns = (
        math.ceil(route_duration_s * 10.0 - 1e-12) * _LIDAR_FRAME_PERIOD_NS
    )
    final_pose = pose_by_time.get(route_deadline_ns)
    if final_pose is None:
        raise ValueError("mapping acceptance requires the route-deadline pose")
    final_projection = route.project(
        final_pose.base_pose.position[0],
        final_pose.base_pose.position[1],
    )
    final_route_distance_m = _finite(
        "final route distance",
        getattr(final_projection, "route_distance_m", None),
    )
    if not 0.0 <= final_route_distance_m <= route_length_m + 1e-9:
        raise ValueError("final route distance must lie on the route")
    route_final_remaining_m = max(0.0, route_length_m - final_route_distance_m)
    eligible, grid_min_x, grid_min_y, x_count, y_count = _eligible_terrain_cells(
        bounds,
        obstacle_values,
        safety_edge_m=edge,
        terrain_grid_m=grid,
    )
    covered: set[tuple[int, int]] = set()
    route_errors: list[float] = []
    world_map = WorldMapAccumulator(
        minimum=(bounds.min_x, bounds.min_y, -2.0),
        maximum=(bounds.max_x, bounds.max_y, 5.0),
    )
    frame_count = 0
    last_time_ns = frame_times[0]
    for cloud in index.iter_lidar_frames():
        if frame_count >= len(frame_times) or cloud.timebase_ns != frame_times[frame_count]:
            raise ValueError("mapping acceptance LiDAR order differs from its index")
        start = pose_by_time.get(cloud.timebase_ns)
        lookahead = pose_by_time.get(cloud.timebase_ns + _LIDAR_FRAME_PERIOD_NS)
        if start is None or lookahead is None:
            raise ValueError("mapping acceptance requires same-time and look-ahead poses")
        projection = route.project(start.base_pose.position[0], start.base_pose.position[1])
        distance = _finite("route projection distance", getattr(projection, "distance_m", None))
        if distance < 0.0:
            raise ValueError("route projection distance must be nonnegative")
        route_errors.append(distance)
        points = deskew_lidar_frame(cloud, start, lookahead)
        world_map.add_frame(points, frame_time_ns=cloud.timebase_ns)
        for point in points:
            if point.tag != 1:
                continue
            x_index = math.floor((point.position[0] - grid_min_x) / grid)
            y_index = math.floor((point.position[1] - grid_min_y) / grid)
            cell = (x_index, y_index)
            if 0 <= x_index < x_count and 0 <= y_index < y_count and cell in eligible:
                covered.add(cell)
        frame_count += 1
        last_time_ns = cloud.timebase_ns
    if frame_count != len(frame_times):
        raise ValueError("mapping acceptance LiDAR count differs from its index")
    snapshot = world_map.snapshot(frame_time_ns=last_time_ns)
    displayed_count = len(snapshot.permanent_positions)
    if displayed_count > _MAX_STATIC_DISPLAY_POINTS:
        raise RuntimeError("world map display exceeded its frozen point limit")
    return MappingAcceptanceMetrics(
        route_sample_count=len(route_errors),
        route_error_p95_m=_nearest_rank_p95(route_errors),
        route_final_remaining_m=route_final_remaining_m,
        terrain_eligible_cell_count=len(eligible),
        terrain_covered_cell_count=len(covered),
        terrain_coverage_ratio=len(covered) / len(eligible),
        permanent_voxel_count=snapshot.permanent_voxel_count,
        displayed_static_point_count=displayed_count,
    )
