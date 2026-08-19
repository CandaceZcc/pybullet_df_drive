"""MID-360 Golf 数值验收的流式统计与阈值合同。"""

from __future__ import annotations

from dataclasses import replace
from importlib import import_module
from types import SimpleNamespace

import numpy as np
import pytest

from slope_sim.interfaces.v2.models import LidarPointCloudV2, LidarPointV2
from slope_sim.mapping_replay import RecoveredPoseNode
from slope_sim.obstacles import ObstacleGeometry, ObstacleSnapshot, ObstacleSpec
from slope_sim.scene import TerrainBounds
from slope_sim.sensor_backend import Pose


_SESSION_ID = bytes.fromhex("00112233445566778899aabbccddeeff")
_DESCRIPTOR = bytes.fromhex("11" * 32)


def _cloud(
    timebase_ns: int,
    values: tuple[tuple[float, float, float, int], ...],
) -> LidarPointCloudV2:
    return LidarPointCloudV2(
        timebase_ns=timebase_ns,
        frame_id="lidar_link",
        point_num=len(values),
        lidar_id=1,
        points=tuple(
            LidarPointV2(index * 5_000, x, y, z, 100, tag, index % 4)
            for index, (x, y, z, tag) in enumerate(values)
        ),
        sequence=timebase_ns // 100_000_000,
        world_generation=1,
        simulation_session_id=_SESSION_ID,
        descriptor_sha256=_DESCRIPTOR,
    )


def _node(timestamp_ns: int, *, base_xy: tuple[float, float] = (0.0, 0.0)) -> RecoveredPoseNode:
    identity = (0.0, 0.0, 0.0, 1.0)
    return RecoveredPoseNode(
        timestamp_ns,
        Pose((base_xy[0], base_xy[1], 0.0), identity),
        Pose((0.0, 0.0, 0.0), identity),
    )


def _obstacle_snapshot(
    logical_id: int,
    body_id: int,
    mode: str,
    position: tuple[float, float, float],
) -> ObstacleSnapshot:
    return ObstacleSnapshot(
        logical_id=logical_id,
        body_id=body_id,
        mode=mode,
        shape="box",
        position=position,
        orientation=(0.0, 0.0, 0.0, 1.0),
        geometry=ObstacleGeometry("box", (0.1, 0.1, 0.1)),
    )


def test_capture_acceptance_keeps_bounded_truth_metrics_per_frame() -> None:
    acceptance = import_module("slope_sim.mapping_acceptance")
    offline = import_module("slope_sim.mid360_offline")
    initial = (
        _obstacle_snapshot(1, 10, "static", (1.0, 0.0, 0.0)),
        _obstacle_snapshot(7, 11, "moving", (2.0, 0.0, 0.0)),
    )
    accumulator = acceptance.GolfCaptureAcceptance(initial)
    accumulator.observe_motion_frame(eligible=True, actual_speed_m_s=0.2)
    accumulator.observe_motion_frame(eligible=True, actual_speed_m_s=0.05)

    for frame_index, moving_x in enumerate((2.0, 2.1)):
        timebase_ns = frame_index * 100_000_000
        cloud = _cloud(
            timebase_ns,
            ((1.0, 0.0, 0.0, 2), (2.0, 0.0, 0.0, 3)),
        )
        truth = offline.OfflineMid360AcceptanceTruth(
            np.asarray(((1.0, 0.0, 0.0), (2.0, 0.0, 0.0))),
            np.asarray((10, 11)),
            np.asarray(((1.0, 0.0, 0.0), (moving_x, 0.0, 0.0))),
        )
        accumulator.observe_lidar_frame(
            cloud=cloud,
            truth=truth,
            start=_node(timebase_ns),
            lookahead=_node(timebase_ns + 100_000_000),
            obstacle_snapshots=(
                initial[0],
                _obstacle_snapshot(7, 11, "moving", (9.0, 0.0, 0.0)),
            ),
        )

    metrics = accumulator.snapshot()

    assert metrics.eligible_motion_frames == 2
    assert metrics.moving_motion_frames == 1
    assert metrics.active_speed_ratio == pytest.approx(0.5)
    assert metrics.deskew_point_count == 4
    assert metrics.deskew_within_5cm_count == 4
    assert metrics.deskew_error_p95_upper_bound_m == pytest.approx(0.0)
    assert metrics.obstacles[0].logical_id == 1
    assert metrics.obstacles[0].hit_frame_count == 2
    assert metrics.obstacles[1].logical_id == 7
    assert metrics.obstacles[1].hit_frame_count == 2
    assert metrics.obstacles[1].position_bucket_count == 2
    assert metrics.obstacles[1].position_span_m == pytest.approx(0.1)


class _Route:
    length = 1.0
    duration_s = 0.1

    def project(self, x: float, y: float) -> SimpleNamespace:
        return SimpleNamespace(
            distance_m=abs(y),
            route_distance_m=min(self.length, max(0.0, x)),
        )


class _Index:
    def __init__(self, cloud: LidarPointCloudV2) -> None:
        self.lidar_frame_times_ns = (cloud.timebase_ns,)
        self.pose_nodes = (
            _node(0, base_xy=(0.25, 0.0)),
            _node(100_000_000, base_xy=(0.75, 0.0)),
        )
        self._cloud = cloud
        self.iterations = 0

    def iter_lidar_frames(self):
        self.iterations += 1
        yield self._cloud


def test_mapping_session_acceptance_derives_route_coverage_and_map_counts() -> None:
    acceptance = import_module("slope_sim.mapping_acceptance")
    bounds = TerrainBounds(0.0, 1.0, 0.0, 1.0)
    values = tuple(
        (x, y, 0.0, 1)
        for y in (0.375, 0.625)
        for x in (0.375, 0.625)
    )
    index = _Index(_cloud(0, values))
    excluded = ObstacleSpec(
        logical_id=1,
        mode="static",
        geometry=ObstacleGeometry("box", (0.01, 0.01, 0.1)),
        position=(0.375, 0.375, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )

    metrics = acceptance.evaluate_mapping_session(
        index,
        route=_Route(),
        bounds=bounds,
        obstacles=(excluded,),
        safety_edge_m=0.25,
        terrain_grid_m=0.25,
    )

    assert index.iterations == 1
    assert metrics.route_sample_count == 1
    assert metrics.route_error_p95_m == pytest.approx(0.0)
    assert metrics.route_final_remaining_m == pytest.approx(0.25)
    assert metrics.terrain_eligible_cell_count == 3
    assert metrics.terrain_covered_cell_count == 3
    assert metrics.terrain_coverage_ratio == pytest.approx(1.0)
    assert metrics.permanent_voxel_count == 4
    assert metrics.displayed_static_point_count == 4


def test_acceptance_failures_checks_every_frozen_threshold() -> None:
    acceptance = import_module("slope_sim.mapping_acceptance")
    obstacles = tuple(
        acceptance.ObstacleAcceptanceMetrics(
            logical_id=logical_id,
            mode="static" if logical_id <= 6 else "moving",
            hit_frame_count=1 if logical_id <= 6 else 10,
            position_bucket_count=0 if logical_id <= 6 else 2,
            position_span_m=0.0 if logical_id <= 6 else 0.1,
        )
        for logical_id in range(1, 10)
    )
    capture = acceptance.CaptureAcceptanceMetrics(
        eligible_motion_frames=100,
        moving_motion_frames=95,
        active_speed_ratio=0.95,
        deskew_point_count=100,
        deskew_within_5cm_count=95,
        deskew_error_p95_upper_bound_m=0.05,
        obstacles=obstacles,
    )
    mapping = acceptance.MappingAcceptanceMetrics(
        route_sample_count=1,
        route_error_p95_m=0.35,
        route_final_remaining_m=0.75,
        terrain_eligible_cell_count=100,
        terrain_covered_cell_count=95,
        terrain_coverage_ratio=0.95,
        permanent_voxel_count=600_000,
        displayed_static_point_count=500_000,
    )

    assert acceptance.acceptance_failures(capture, mapping) == ()
    assert acceptance.acceptance_failures(
        capture,
        replace(mapping, route_error_p95_m=0.351),
    ) == ("route_error_p95",)
    assert acceptance.acceptance_failures(
        capture,
        replace(mapping, route_final_remaining_m=0.751),
    ) == ("route_incomplete",)
    assert acceptance.acceptance_failures(
        replace(
            capture,
            moving_motion_frames=94,
            active_speed_ratio=0.94,
        ),
        mapping,
    ) == ("active_speed_ratio",)
    assert acceptance.acceptance_failures(
        capture,
        replace(mapping, terrain_coverage_ratio=0.949),
    ) == ("terrain_coverage_ratio",)
    assert acceptance.acceptance_failures(
        replace(capture, obstacles=(replace(obstacles[0], hit_frame_count=0), *obstacles[1:])),
        mapping,
    ) == ("static_obstacle_hits",)
    assert acceptance.acceptance_failures(
        replace(capture, obstacles=(*obstacles[:6], replace(obstacles[6], hit_frame_count=9), *obstacles[7:])),
        mapping,
    ) == ("moving_obstacle_hit_frames",)
    assert acceptance.acceptance_failures(
        replace(
            capture,
            obstacles=(
                *obstacles[:6],
                replace(obstacles[6], position_bucket_count=1, position_span_m=0.0),
                *obstacles[7:],
            ),
        ),
        mapping,
    ) == ("moving_obstacle_positions",)
    assert acceptance.acceptance_failures(
        replace(capture, deskew_error_p95_upper_bound_m=0.0501),
        mapping,
    ) == ("deskew_error_p95",)
    assert acceptance.acceptance_failures(
        capture,
        replace(mapping, displayed_static_point_count=500_001),
    ) == ("static_display_point_count",)
