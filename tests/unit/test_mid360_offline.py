"""离线 MID-360 高保真 schedule 的冻结合同。"""

from importlib import import_module

import numpy as np
import pytest

from slope_sim.lidar_pointcloud import (
    MID360_PATTERN_VERSION,
    Stage4LidarProfile,
)
from slope_sim.sensor_backend import Pose, RayHit


def test_offline_schedule_uses_20000_five_us_slots_without_changing_realtime() -> None:
    """离线 100 ms 帧必须独立分配 20,000 firing，不能改写实时 profile。"""
    offline = import_module("slope_sim.mid360_offline")
    profile = offline.OfflineMid360Profile.high_fidelity()
    schedule = offline.OfflineMid360Schedule(
        profile,
        pattern_version=MID360_PATTERN_VERSION,
        world_generation=7,
    )

    assert profile.firing_slot_count == 20_000
    assert profile.firing_interval_ns == 5_000
    assert profile.frame_period_ns == 100_000_000
    assert profile.min_range_m == pytest.approx(0.1)
    assert profile.max_range_m == pytest.approx(40.0)
    assert tuple(len(slot_range) for slot_range in profile.physics_step_slot_ranges) == (
        834,
        833,
        833,
    ) * 8
    assert profile.physics_step_slot_ranges[0].start == 0
    assert profile.physics_step_slot_ranges[-1].stop == 20_000
    assert sum(map(len, profile.physics_step_slot_ranges)) == 20_000

    assert schedule.offset_time_ns(0) == 0
    assert schedule.offset_time_ns(19_999) == 99_995_000
    with pytest.raises(ValueError, match="slot"):
        schedule.offset_time_ns(20_000)

    first_row = schedule.pattern_row_index(sequence=0, slot=0)
    assert schedule.pattern_row_index(sequence=39, slot=19_999) == (
        first_row + 799_999
    ) % 800_000
    assert schedule.pattern_row_index(sequence=40, slot=0) == first_row
    assert Stage4LidarProfile.realtime().firing_slot_count == 5_760


class _MovingPoseBackend:
    """记录每个物理步的冻结 pose、射线批次和局部逆变换。"""

    def __init__(self) -> None:
        self.world_pose_calls: list[str] = []
        self.batch_sizes: list[int] = []
        self.inverse_poses: list[Pose] = []
        self._step = -1

    def world_pose(self, parent_link: str) -> Pose:
        self.world_pose_calls.append(parent_link)
        self._step += 1
        return Pose((self._step * 0.1, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    def _ray_test_indexed_hits_ndarray(
        self,
        starts: object,
        ends: object,
        *,
        collision_mask: int,
        num_threads: int = 0,
    ) -> tuple[tuple[int, RayHit], ...]:
        assert isinstance(starts, np.ndarray)
        assert isinstance(ends, np.ndarray)
        assert starts.flags.writeable is False
        assert ends.flags.writeable is False
        assert collision_mask == 0x10
        assert num_threads == 0
        self.batch_sizes.append(len(starts))
        pose_x = self._step * 0.1
        assert np.linalg.norm(starts[0] - np.array((pose_x, 0.0, 0.0))) == pytest.approx(0.1)
        assert np.linalg.norm(ends[0] - np.array((pose_x, 0.0, 0.0))) == pytest.approx(40.0)
        if self._step == 23:
            return ()
        return ((0, RayHit((10.0, 0.0, 0.0), 7, -1, "moving_obstacle")),)

    def inverse_transform_points_prevalidated(
        self,
        pose: Pose,
        points: tuple[tuple[float, float, float], ...],
    ) -> tuple[tuple[float, float, float], ...]:
        self.inverse_poses.append(pose)
        return tuple(
            (
                point[0] - pose.position[0],
                point[1] - pose.position[1],
                point[2] - pose.position[2],
            )
            for point in points
        )


def test_offline_scanner_freezes_each_physics_step_and_keeps_raw_motion_distortion() -> None:
    """batch-local 命中必须映射回全局 firing，并按所属步 pose 保留 raw 形变。"""
    offline = import_module("slope_sim.mid360_offline")
    profile = offline.OfflineMid360Profile.high_fidelity()
    schedule = offline.OfflineMid360Schedule(
        profile,
        pattern_version=MID360_PATTERN_VERSION,
        world_generation=3,
    )
    backend = _MovingPoseBackend()
    scanner = offline.OfflineMid360FrameScanner(
        backend,
        schedule,
        sequence=2,
    )

    with pytest.raises(RuntimeError, match="finalized"):
        scanner.acceptance_truth()

    for step in range(24):
        assert scanner.capture_step(
            step,
            body_positions_by_id={7: (step * 0.05, 1.0, 0.0)},
        ) == (0 if step == 23 else 1)
    cloud = scanner.finalize(timebase_ns=500_000_000)
    truth = scanner.acceptance_truth()

    assert backend.world_pose_calls == ["lidar_link"] * 24
    assert tuple(backend.batch_sizes) == (834, 833, 833) * 8
    assert len(backend.inverse_poses) == 23
    assert cloud.timebase_ns == 500_000_000
    assert cloud.frame_id == "lidar_link"
    assert cloud.lidar_id == 1
    assert cloud.point_num == len(cloud.points) == 23
    assert tuple(point.offset_time_ns for point in cloud.points) == tuple(
        slot_range.start * 5_000
        for slot_range in profile.physics_step_slot_ranges[:-1]
    )
    assert tuple(point.x for point in cloud.points) == pytest.approx(
        tuple(10.0 - step * 0.1 for step in range(23))
    )
    assert all((point.reflectivity, point.tag) == (200, 3) for point in cloud.points)
    assert truth.world_positions.shape == (23, 3)
    assert truth.world_positions.dtype == np.float64
    assert truth.world_positions.flags.writeable is False
    assert truth.body_ids.shape == (23,)
    assert truth.body_ids.dtype == np.int32
    assert truth.body_ids.flags.writeable is False
    assert truth.hit_body_positions.shape == (23, 3)
    assert truth.hit_body_positions.dtype == np.float64
    assert truth.hit_body_positions.flags.writeable is False
    assert truth.world_positions == pytest.approx(np.tile((10.0, 0.0, 0.0), (23, 1)))
    assert tuple(truth.body_ids) == (7,) * 23
    assert truth.hit_body_positions == pytest.approx(
        np.asarray(tuple((step * 0.05, 1.0, 0.0) for step in range(23)))
    )
