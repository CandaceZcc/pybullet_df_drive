"""Golf 世界回放的姿态恢复、插值与逐点去畸变合同。"""

from dataclasses import replace
from importlib import import_module
import math

import numpy as np
import pytest

from slope_sim.interfaces.v2.models import (
    ImuAttitudeV2,
    LidarPointCloudV2,
    LidarPointV2,
    Point3dV2,
    RtkStateV2,
)
from slope_sim.sensor_backend import Pose


_SESSION_ID = bytes.fromhex("00112233445566778899aabbccddeeff")
_DESCRIPTOR = bytes.fromhex("11" * 32)


def _quaternion_from_euler(
    roll: float,
    pitch: float,
    yaw: float,
) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _rotate(
    quaternion: tuple[float, float, float, float],
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    matrix = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )
    return tuple(
        sum(matrix[row][column] * point[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _transform(
    pose: Pose,
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotated = _rotate(pose.orientation, point)
    return tuple(pose.position[index] + rotated[index] for index in range(3))  # type: ignore[return-value]


def _inverse_transform(
    pose: Pose,
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    translated = tuple(point[index] - pose.position[index] for index in range(3))
    x, y, z, w = pose.orientation
    return _rotate((-x, -y, -z, w), translated)  # type: ignore[arg-type]


def test_recover_pose_uses_full_rtk_baseline_and_frozen_mounts() -> None:
    """倾斜地面上的 RTK heading 不等于 yaw，仍须恢复唯一 base/lidar pose。"""
    replay = import_module("slope_sim.mapping_replay")
    timestamp_ns = 1_000_000_000
    roll, pitch, yaw = 0.23, -0.17, 0.71
    expected_orientation = _quaternion_from_euler(roll, pitch, yaw)
    base_position = (1.2, -0.7, 0.4)

    def mounted(local: tuple[float, float, float]) -> Point3dV2:
        rotated = _rotate(expected_orientation, local)
        return Point3dV2(
            *(base_position[index] + rotated[index] for index in range(3))
        )

    left = mounted((0.0, 0.20, 0.18))
    center = mounted((0.0, 0.0, 0.18))
    right = mounted((0.0, -0.20, 0.18))
    heading = math.remainder(
        math.atan2(left.y_m - right.y_m, left.x_m - right.x_m) - math.pi / 2.0,
        2.0 * math.pi,
    )
    assert abs(math.remainder(heading - yaw, 2.0 * math.pi)) > 0.01
    rtk = RtkStateV2(
        timestamp_ns,
        3,
        1,
        "world",
        left,
        center,
        right,
        heading,
        _SESSION_ID,
        _DESCRIPTOR,
    )
    imu = ImuAttitudeV2(
        timestamp_ns,
        roll,
        pitch,
        3,
        1,
        "base_link",
        _SESSION_ID,
        _DESCRIPTOR,
    )
    previous = tuple(-value for value in expected_orientation)

    node = replay.recover_pose_node(
        rtk,
        imu,
        previous_orientation=previous,
    )

    assert node.timestamp_ns == timestamp_ns
    assert node.base_pose.position == pytest.approx(base_position, abs=1e-9)
    expected_lidar_position = _transform(
        Pose(base_position, expected_orientation),
        (0.0, 0.0, 0.105),
    )
    assert node.lidar_pose.position == pytest.approx(expected_lidar_position, abs=1e-9)
    assert abs(
        sum(
            node.base_pose.orientation[index] * expected_orientation[index]
            for index in range(4)
        )
    ) == pytest.approx(1.0, abs=1e-9)
    assert sum(
        node.base_pose.orientation[index] * previous[index]
        for index in range(4)
    ) >= 0.0


def test_recover_pose_accepts_pybullet_scale_baseline_quantization() -> None:
    """世界端点的微米级量化不能把同一真值姿态误判为 RTK/IMU 冲突。"""
    replay = import_module("slope_sim.mapping_replay")
    roll, pitch, yaw = 0.0485, 0.0005, 0.0022
    orientation = _quaternion_from_euler(roll, pitch, yaw)
    center = _transform(Pose((-2.7, 0.0, 0.106), orientation), (0.0, 0.0, 0.18))
    exact_base_y = _rotate(orientation, (0.0, 1.0, 0.0))

    def pose_messages(direction_z_error: float) -> tuple[RtkStateV2, ImuAttitudeV2]:
        perturbed = (
            exact_base_y[0],
            exact_base_y[1],
            exact_base_y[2] + direction_z_error,
        )
        length = math.hypot(*perturbed)
        direction = tuple(value / length for value in perturbed)
        left = Point3dV2(
            *(center[index] + 0.20 * direction[index] for index in range(3))
        )
        right = Point3dV2(
            *(center[index] - 0.20 * direction[index] for index in range(3))
        )
        heading = math.remainder(
            math.atan2(left.y_m - right.y_m, left.x_m - right.x_m)
            - math.pi / 2.0,
            2.0 * math.pi,
        )
        return (
            RtkStateV2(
                1_500_000_000,
                15,
                1,
                "world",
                left,
                Point3dV2(*center),
                right,
                heading,
                _SESSION_ID,
                _DESCRIPTOR,
            ),
            ImuAttitudeV2(
                1_500_000_000,
                roll,
                pitch,
                15,
                1,
                "base_link",
                _SESSION_ID,
                _DESCRIPTOR,
            ),
        )

    rtk, imu = pose_messages(2.0e-6)
    node = replay.recover_pose_node(rtk, imu)
    assert abs(
        sum(node.base_pose.orientation[index] * orientation[index] for index in range(4))
    ) == pytest.approx(1.0, abs=1e-9)

    incompatible_rtk, incompatible_imu = pose_messages(1.0e-4)
    with pytest.raises(ValueError, match="inconsistent with IMU"):
        replay.recover_pose_node(incompatible_rtk, incompatible_imu)


def test_recover_pose_accepts_pybullet_scale_baseline_length_quantization() -> None:
    """大世界坐标的端点量化不得把微米级基线误差当成安装错误。"""
    replay = import_module("slope_sim.mapping_replay")

    def pose_messages(length_error_m: float) -> tuple[RtkStateV2, ImuAttitudeV2]:
        center = Point3dV2(-9.75, -6.4, 0.2)
        half_length = (0.40 + length_error_m) / 2.0
        return (
            RtkStateV2(
                1_500_000_000,
                15,
                1,
                "world",
                Point3dV2(center.x_m, center.y_m + half_length, center.z_m),
                center,
                Point3dV2(center.x_m, center.y_m - half_length, center.z_m),
                0.0,
                _SESSION_ID,
                _DESCRIPTOR,
            ),
            ImuAttitudeV2(
                1_500_000_000,
                0.0,
                0.0,
                15,
                1,
                "base_link",
                _SESSION_ID,
                _DESCRIPTOR,
            ),
        )

    rtk, imu = pose_messages(4.0e-6)
    node = replay.recover_pose_node(rtk, imu)
    assert node.base_pose.position == pytest.approx((-9.75, -6.4, 0.02), abs=1e-12)

    incompatible_rtk, incompatible_imu = pose_messages(1.0e-4)
    with pytest.raises(ValueError, match="0.40"):
        replay.recover_pose_node(incompatible_rtk, incompatible_imu)


def test_recover_pose_rejects_noncanonical_mount_geometry_and_frames() -> None:
    """回放不得从错误 RTK 杆臂或坐标 frame 猜测车辆位姿。"""
    replay = import_module("slope_sim.mapping_replay")
    rtk = RtkStateV2(
        0,
        0,
        1,
        "world",
        Point3dV2(0.0, 0.20, 0.18),
        Point3dV2(0.0, 0.0, 0.18),
        Point3dV2(0.0, -0.20, 0.18),
        0.0,
        _SESSION_ID,
        _DESCRIPTOR,
    )
    imu = ImuAttitudeV2(
        0,
        0.0,
        0.0,
        0,
        1,
        "base_link",
        _SESSION_ID,
        _DESCRIPTOR,
    )

    with pytest.raises(ValueError, match="0.40"):
        replay.recover_pose_node(
            replace(
                rtk,
                left=Point3dV2(0.0, 2.0, 0.18),
                right=Point3dV2(0.0, -2.0, 0.18),
            ),
            imu,
        )
    with pytest.raises(ValueError, match="CENTER"):
        replay.recover_pose_node(
            replace(rtk, center=Point3dV2(0.1, 0.0, 0.18)),
            imu,
        )
    with pytest.raises(ValueError, match="frame_id"):
        replay.recover_pose_node(replace(rtk, frame_id="map"), imu)
    with pytest.raises(ValueError, match="frame_id"):
        replay.recover_pose_node(rtk, replace(imu, frame_id="imu_link"))


def test_slerp_normalizes_large_finite_quaternions_without_overflow() -> None:
    """有限四元数即使量级很大，也应稳定归一化而不是产生 NaN。"""
    replay = import_module("slope_sim.mapping_replay")
    orientation = replay.slerp_shortest(
        (1e308, 0.0, 0.0, 0.0),
        (0.0, 1e308, 0.0, 0.0),
        0.5,
    )

    assert all(math.isfinite(value) for value in orientation)
    assert math.hypot(*orientation) == pytest.approx(1.0)


def test_slerp_rejects_an_unrepresentable_quaternion_norm() -> None:
    """范数本身溢出的有限输入必须在对应端点被明确拒绝。"""
    replay = import_module("slope_sim.mapping_replay")

    with pytest.raises(ValueError, match="start quaternion norm"):
        replay.slerp_shortest(
            (1e308, 1e308, 1e308, 1e308),
            (0.0, 0.0, 0.0, 1.0),
            0.0,
        )


def test_deskew_slerps_each_raw_point_back_to_one_world_hit() -> None:
    """平移加转弯产生的 raw 形变必须按点时刻恢复为相同世界命中。"""
    replay = import_module("slope_sim.mapping_replay")
    start = replay.RecoveredPoseNode(
        0,
        Pose((0.0, 0.0, 0.0), _quaternion_from_euler(0.0, 0.0, 0.0)),
        Pose((0.0, 0.0, 0.0), _quaternion_from_euler(0.0, 0.0, 0.0)),
    )
    end = replay.RecoveredPoseNode(
        100_000_000,
        Pose((1.0, 0.0, 0.0), _quaternion_from_euler(0.0, 0.0, math.pi / 2.0)),
        Pose((1.0, 0.0, 0.0), _quaternion_from_euler(0.0, 0.0, math.pi / 2.0)),
    )
    middle_pose = Pose(
        (0.5, 0.0, 0.0),
        _quaternion_from_euler(0.0, 0.0, math.pi / 4.0),
    )
    world_hit = (5.0, 2.0, 0.5)
    raw_start = _inverse_transform(start.lidar_pose, world_hit)
    raw_middle = _inverse_transform(middle_pose, world_hit)
    cloud = LidarPointCloudV2(
        0,
        "lidar_link",
        2,
        1,
        (
            LidarPointV2(0, *raw_start, 100, 1, 0),
            LidarPointV2(50_000_000, *raw_middle, 160, 2, 1),
        ),
        0,
        1,
        _SESSION_ID,
        _DESCRIPTOR,
    )

    points = replay.deskew_lidar_frame(cloud, start, end)

    assert len(points) == 2
    for point in points:
        assert point.position == pytest.approx(world_hit, abs=2e-6)
    assert tuple(point.timestamp_ns for point in points) == (0, 50_000_000)
    assert tuple(point.tag for point in points) == (1, 2)
    short_lookahead = replay.RecoveredPoseNode(
        49_999_999,
        end.base_pose,
        end.lidar_pose,
    )
    with pytest.raises(replay.MissingPoseLookaheadError, match="look-ahead"):
        replay.deskew_lidar_frame(cloud, start, short_lookahead)


def test_deskew_requires_full_frame_lookahead_and_official_firing_grid() -> None:
    """稀疏命中或空帧也不能绕过 5 us/100 ms 的离线 firing 合同。"""
    replay = import_module("slope_sim.mapping_replay")
    pose = Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    start = replay.RecoveredPoseNode(0, pose, pose)
    short = replay.RecoveredPoseNode(99_994_999, pose, pose)
    complete = replay.RecoveredPoseNode(100_000_000, pose, pose)
    empty = LidarPointCloudV2(
        0,
        "lidar_link",
        0,
        1,
        (),
        0,
        1,
        _SESSION_ID,
        _DESCRIPTOR,
    )
    with pytest.raises(replay.MissingPoseLookaheadError, match="look-ahead"):
        replay.deskew_lidar_frame(empty, start, short)

    def cloud_with_offsets(*offsets: int, timebase_ns: int = 0) -> LidarPointCloudV2:
        points = tuple(
            LidarPointV2(offset, 1.0, 0.0, 0.0, 100, 1, 0)
            for offset in offsets
        )
        return replace(
            empty,
            timebase_ns=timebase_ns,
            point_num=len(points),
            points=points,
        )

    for invalid in (
        cloud_with_offsets(1),
        cloud_with_offsets(10_000, 5_000),
        cloud_with_offsets(100_000_000),
    ):
        with pytest.raises(ValueError, match="offset_time_ns"):
            replay.deskew_lidar_frame(invalid, start, complete)

    maximum = (1 << 64) - 1
    overflowing = cloud_with_offsets(5_000, timebase_ns=maximum - 4_999)
    late_start = replay.RecoveredPoseNode(maximum - 4_999, pose, pose)
    late_lookahead = replay.RecoveredPoseNode(maximum, pose, pose)
    with pytest.raises(ValueError, match="uint64"):
        replay.deskew_lidar_frame(overflowing, late_start, late_lookahead)


def test_world_map_voxels_static_and_moving_layers_deterministically() -> None:
    """永久层去重，运动层按仿真时间过期，显示上限不改变底层地图。"""
    replay = import_module("slope_sim.mapping_replay")

    def point(
        timestamp_ns: int,
        position: tuple[float, float, float],
        tag: int,
    ) -> object:
        return replay.DeskewedPoint(timestamp_ns, position, 100, tag, 0)

    first_frame = (
        point(100, (0.001, 0.001, 0.001), 1),
        point(100, (0.049, 0.049, 0.049), 1),
        point(100, (0.101, 0.001, 0.001), 2),
        point(100, (0.201, 0.001, 0.001), 0),
        point(100, (0.301, 0.001, 0.001), 3),
        point(100, (2.0, 0.0, 0.0), 1),
    )
    second_frame = (point(200, (0.201, 0.001, 0.001), 1),)

    def build() -> object:
        accumulator = replay.WorldMapAccumulator(
            minimum=(-1.0, -1.0, -1.0),
            maximum=(1.0, 1.0, 1.0),
            voxel_size_m=0.05,
            static_display_limit=2,
            moving_ttl_ns=300_000_000,
        )
        accumulator.add_frame(first_frame, frame_time_ns=100)
        accumulator.add_frame(second_frame, frame_time_ns=200)
        return accumulator

    accumulator = build()
    snapshot = accumulator.snapshot(frame_time_ns=300_000_100)

    assert snapshot.permanent_voxel_count == 3
    assert snapshot.permanent_positions.shape == (2, 3)
    assert snapshot.permanent_tags.shape == (2,)
    assert snapshot.moving_positions.shape == (1, 3)
    assert snapshot.moving_tags.tolist() == [3]
    for values in (
        snapshot.permanent_positions,
        snapshot.permanent_tags,
        snapshot.moving_positions,
        snapshot.moving_tags,
    ):
        assert isinstance(values, np.ndarray)
        assert values.flags.writeable is False

    repeated = build().snapshot(frame_time_ns=300_000_100)
    np.testing.assert_array_equal(
        snapshot.permanent_positions,
        repeated.permanent_positions,
    )
    np.testing.assert_array_equal(snapshot.permanent_tags, repeated.permanent_tags)

    accumulator.add_frame(first_frame, frame_time_ns=300_000_100)
    assert accumulator.snapshot(frame_time_ns=300_000_100).permanent_voxel_count == 3
    expired = accumulator.snapshot(frame_time_ns=600_000_101)
    assert expired.moving_positions.shape == (0, 3)
    accumulator.clear()
    cleared = accumulator.snapshot(frame_time_ns=600_000_101)
    assert cleared.permanent_voxel_count == 0
    assert cleared.permanent_positions.shape == (0, 3)


def test_playback_clock_waits_for_one_result_and_rebuilds_after_seek() -> None:
    """后台未完成时不推进；回退与定位使用 generation 丢弃过期结果。"""
    replay = import_module("slope_sim.mapping_replay")
    clock = replay.PlaybackClock((0, 100_000_000, 200_000_000))

    assert clock.paused is True
    assert clock.frame_index == 0
    clock.play()
    first = clock.begin_next_frame()
    assert first is not None
    assert first.frame_index == 1
    assert first.rebuild_from_start is False
    assert clock.begin_next_frame() is None
    assert clock.frame_index == 0
    assert clock.complete(first) is True
    assert clock.frame_index == 1

    clock.set_rate(2.0)
    assert clock.frame_interval_ns == 50_000_000
    backward = clock.step(-1)
    assert backward is not None
    assert backward.frame_index == 0
    assert backward.rebuild_from_start is True
    assert clock.paused is True

    # 用户定位会使尚未返回的回退结果过期，但容量 1 队列仍须先排空。
    assert clock.seek(2) is None
    assert clock.complete(backward) is False
    assert clock.frame_index == 1
    target = clock.begin_pending_frame()
    assert target is not None
    assert target.frame_index == 2
    assert target.rebuild_from_start is True
    assert clock.complete(target) is True
    assert clock.frame_index == 2
