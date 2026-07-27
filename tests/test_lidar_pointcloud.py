# 多线点云单元测试：锁定 16x180 射线、安装外参、字段顺序和严格输入边界。
from __future__ import annotations

from collections import deque
from dataclasses import FrozenInstanceError, fields, replace
import math

import pybullet as p
import pytest

from slope_sim.interfaces.models import LidarPoint, LidarPointCloud
from slope_sim.interfaces.dashboard_snapshot import LidarTopViewFrame, LidarTopViewPoint
from slope_sim.lidar_pointcloud import (
    LIDAR_SCAN_PERIOD_NS,
    LIDAR_VISIBLE_GROUP,
    LidarConfig,
    LidarScanResult,
    MultiLineLidar,
    build_unit_rays,
)
from slope_sim.sensor_backend import Pose, RayHit
from slope_sim.truth_sensors import SensorMounts


IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)
UINT64_MAX = (1 << 64) - 1


class FakeLidarBackend:
    """以真实刚体变换数学记录点云算法调用，不替代 DIRECT 物理门禁。"""

    def __init__(self) -> None:
        self.parent_poses = {
            "base_link": Pose((0.0, 0.0, 0.0), IDENTITY_QUATERNION),
            "lidar_front_mount": Pose((0.0, 0.0, 0.0), IDENTITY_QUATERNION),
            "lidar_rear_mount": Pose((0.0, 0.0, 0.0), IDENTITY_QUATERNION),
        }
        self.local_hits: dict[int, tuple[tuple[float, float, float], str]] = {}
        self.world_pose_calls: list[str] = []
        self.mount_locals: list[Pose] = []
        self.ray_batches: list[
            tuple[
                tuple[tuple[float, float, float], ...],
                tuple[tuple[float, float, float], ...],
                int,
            ]
        ] = []
        self.active_mount: Pose | None = None
        self.truncate_results = False
        self.scan_error: Exception | None = None
        self.inverse_batches: list[tuple[tuple[float, float, float], ...]] = []
        self.inverse_overrides: deque[object | None] = deque()

    def link_names(self) -> tuple[str, ...]:
        return ("base_link", "lidar_front_mount", "lidar_rear_mount")

    def world_pose(self, parent_link: str) -> Pose:
        self.world_pose_calls.append(parent_link)
        return self.parent_poses[parent_link]

    def transform_pose(self, parent: Pose, local: Pose) -> Pose:
        position, orientation = p.multiplyTransforms(
            parent.position,
            parent.orientation,
            local.position,
            local.orientation,
        )
        transformed = Pose(tuple(position), tuple(orientation))
        if local.position == (0.0, 0.0, 0.0):
            self.mount_locals.append(local)
            self.active_mount = transformed
        return transformed

    def inverse_transform_point(
        self,
        pose: Pose,
        point: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        inverse_position, inverse_orientation = p.invertTransform(
            pose.position,
            pose.orientation,
        )
        local_position, _ = p.multiplyTransforms(
            inverse_position,
            inverse_orientation,
            point,
            IDENTITY_QUATERNION,
        )
        return tuple(float(value) for value in local_position)

    def inverse_transform_points(
        self,
        pose: Pose,
        points: tuple[tuple[float, float, float], ...],
    ) -> object:
        items = tuple(points)
        self.inverse_batches.append(items)
        if self.inverse_overrides:
            override = self.inverse_overrides.popleft()
            if isinstance(override, BaseException):
                raise override
            if override is not None:
                return override
        return tuple(self.inverse_transform_point(pose, point) for point in items)

    def euler_from_quaternion(self, orientation):
        return tuple(float(value) for value in p.getEulerFromQuaternion(orientation))

    def ray_test_batch(self, starts, ends, *, collision_mask: int) -> tuple[RayHit, ...]:
        if self.scan_error is not None:
            raise self.scan_error
        start_items = tuple(starts)
        end_items = tuple(ends)
        self.ray_batches.append((start_items, end_items, collision_mask))
        assert self.active_mount is not None

        hits: list[RayHit] = []
        for ray_index in range(len(start_items)):
            configured = self.local_hits.get(ray_index)
            if configured is None:
                hits.append(RayHit((0.0, 0.0, 0.0), -1, -1, "unknown"))
                continue
            local_point, category = configured
            world_position, _ = p.multiplyTransforms(
                self.active_mount.position,
                self.active_mount.orientation,
                local_point,
                IDENTITY_QUATERNION,
            )
            hits.append(
                RayHit(
                    tuple(float(value) for value in world_position),
                    100 + ray_index,
                    -1,
                    category,
                )
            )
        result = tuple(hits)
        return result[:-1] if self.truncate_results else result

    def hit_ray(
        self,
        ray_index: int,
        *,
        local_point: tuple[float, float, float],
        category: str,
    ) -> None:
        self.local_hits[ray_index] = (local_point, category)

    def set_all_rays_to_miss(self) -> None:
        self.local_hits.clear()


def test_default_lidar_scan_geometry_is_stable_and_immutable():
    config = LidarConfig.default()

    assert config.vertical_lines == 16
    assert config.horizontal_samples == 180
    assert config.horizontal_fov_deg == 180.0
    assert config.vertical_fov_deg == (-15.0, 15.0)
    assert config.min_range_m == 0.10
    assert config.max_range_m == 30.0
    assert config.ray_count == 2880
    with pytest.raises(AttributeError):
        config.vertical_lines = 8


@pytest.mark.parametrize(
    ("field_name", "invalid_value"),
    [
        ("vertical_lines", True),
        ("vertical_lines", 16.0),
        ("horizontal_samples", False),
        ("horizontal_samples", 180.0),
        ("horizontal_fov_deg", True),
        ("horizontal_fov_deg", math.nan),
        ("vertical_fov_deg", (-15.0, math.inf)),
        ("vertical_fov_deg", (False, 15.0)),
        ("min_range_m", False),
        ("min_range_m", math.nan),
        ("max_range_m", True),
        ("max_range_m", math.inf),
    ],
)
def test_lidar_config_rejects_bool_wrong_integer_types_and_non_finite_values(
    field_name,
    invalid_value,
):
    with pytest.raises(ValueError):
        replace(LidarConfig.default(), **{field_name: invalid_value})


@pytest.mark.parametrize(
    "changes",
    [
        {"vertical_lines": 0},
        {"horizontal_samples": 0},
        {"horizontal_fov_deg": 0.0},
        {"horizontal_fov_deg": 361.0},
        {"vertical_fov_deg": (-15.0,)},
        {"vertical_fov_deg": (15.0, -15.0)},
        {"vertical_fov_deg": (-91.0, 15.0)},
        {"min_range_m": 0.0},
        {"min_range_m": 30.0},
        {"max_range_m": 0.10},
    ],
)
def test_lidar_config_rejects_illegal_ranges(changes):
    with pytest.raises(ValueError):
        replace(LidarConfig.default(), **changes)


@pytest.mark.parametrize(
    "changes",
    [
        {"vertical_lines": 8},
        {"horizontal_samples": 90},
        {"horizontal_fov_deg": 90.0},
        {"vertical_fov_deg": (-10.0, 10.0)},
        {"min_range_m": 0.20},
        {"max_range_m": 20.0},
    ],
)
def test_lidar_config_rejects_valid_but_unsupported_first_release_geometry(changes):
    with pytest.raises(ValueError, match="unsupported"):
        replace(LidarConfig.default(), **changes)


def test_unit_rays_are_line_then_azimuth_with_inclusive_fov_endpoints():
    rays = build_unit_rays(LidarConfig.default())
    cos_15 = math.cos(math.radians(15.0))
    sin_15 = math.sin(math.radians(15.0))

    assert len(rays) == 2880
    assert rays[0] == pytest.approx((0.0, -cos_15, -sin_15), abs=1e-12)
    assert rays[179] == pytest.approx((0.0, cos_15, -sin_15), abs=1e-12)
    assert rays[180][2] > rays[0][2]
    assert rays[-1] == pytest.approx((0.0, cos_15, sin_15), abs=1e-12)
    assert all(math.sqrt(sum(value * value for value in ray)) == pytest.approx(1.0) for ray in rays)


def test_front_and_rear_use_frozen_mounts_and_transform_the_same_fixed_ray_table():
    backend = FakeLidarBackend()
    config = LidarConfig.default()

    MultiLineLidar.front(backend, config).scan(1)
    front_starts, front_ends, front_mask = backend.ray_batches[-1]
    MultiLineLidar.rear(backend, config).scan(2)
    rear_starts, rear_ends, rear_mask = backend.ray_batches[-1]

    assert backend.world_pose_calls == [
        "lidar_front_mount",
        "base_link",
        "lidar_rear_mount",
        "base_link",
    ]
    assert backend.mount_locals[0] == Pose((0.0, 0.0, 0.0), IDENTITY_QUATERNION)
    assert backend.mount_locals[1] == Pose((0.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0))
    assert front_mask == rear_mask == LIDAR_VISIBLE_GROUP == 0x10
    assert len(front_starts) == len(front_ends) == len(rear_starts) == len(rear_ends) == 2880

    for ray_index in (0, 89, 90, 179, 180, 2879):
        assert math.dist((0.0, 0.0, 0.0), front_starts[ray_index]) == pytest.approx(0.10)
        assert math.dist((0.0, 0.0, 0.0), front_ends[ray_index]) == pytest.approx(30.0)
        assert rear_starts[ray_index] == pytest.approx(
            (-front_starts[ray_index][0], -front_starts[ray_index][1], front_starts[ray_index][2]),
            abs=1e-7,
        )
        assert rear_ends[ray_index] == pytest.approx(
            (-front_ends[ray_index][0], -front_ends[ray_index][1], front_ends[ray_index][2]),
            abs=1e-6,
        )


@pytest.mark.parametrize(
    ("frame_id", "lidar_id"),
    (
        ("lidar_front", 2),
        ("lidar_rear", 1),
        ("other", 1),
        ("lidar_front", True),
    ),
)
def test_constructor_rejects_invalid_front_rear_metadata_pair(frame_id, lidar_id):
    with pytest.raises(ValueError, match="frame_id.*lidar_id"):
        MultiLineLidar(
            FakeLidarBackend(),
            LidarConfig.default(),
            SensorMounts.default().lidar_front,
            frame_id=frame_id,
            lidar_id=lidar_id,
        )


def test_cloud_preserves_field_order_ray_order_line_categories_and_offsets():
    backend = FakeLidarBackend()
    backend.parent_poses["lidar_front_mount"] = Pose(
        (3.0, -4.0, 2.0),
        tuple(p.getQuaternionFromEuler((0.2, -0.1, 0.7))),
    )
    backend.hit_ray(0, local_point=(0.10, 0.0, 0.0), category="terrain")
    backend.hit_ray(180, local_point=(2.0, -0.5, 0.25), category="static_obstacle")
    backend.hit_ray(2879, local_point=(29.0, 1.0, 0.5), category="moving_obstacle")

    cloud = MultiLineLidar.front(backend, LidarConfig.default()).scan(1_000_000_000)

    assert tuple(field.name for field in fields(LidarPointCloud)) == (
        "timebase_ns",
        "frame_id",
        "point_num",
        "lidar_id",
        "points",
    )
    assert tuple(field.name for field in fields(LidarPoint)) == (
        "offset_time_ns",
        "x",
        "y",
        "z",
        "reflectivity",
        "tag",
        "line",
    )
    assert (cloud.timebase_ns, cloud.frame_id, cloud.lidar_id) == (
        1_000_000_000,
        "lidar_front",
        1,
    )
    assert cloud.point_num == len(cloud.points) == 3
    assert tuple(point.line for point in cloud.points) == (0, 1, 15)
    assert tuple(point.offset_time_ns for point in cloud.points) == (
        0,
        180 * LIDAR_SCAN_PERIOD_NS // 2880,
        2879 * LIDAR_SCAN_PERIOD_NS // 2880,
    )
    assert tuple((point.tag, point.reflectivity) for point in cloud.points) == (
        (1, 100),
        (2, 160),
        (3, 200),
    )
    assert len(backend.inverse_batches) == 2
    assert len(backend.inverse_batches[0]) == 3
    assert len(backend.inverse_batches[1]) == 3
    for point, expected in zip(
        cloud.points,
        ((0.10, 0.0, 0.0), (2.0, -0.5, 0.25), (29.0, 1.0, 0.5)),
        strict=True,
    ):
        assert (point.x, point.y, point.z) == pytest.approx(expected, abs=2e-6)
    assert all(
        math.isfinite(value)
        for point in cloud.points
        for value in (point.x, point.y, point.z)
    )


def test_empty_cloud_keeps_scan_time_and_zero_count():
    backend = FakeLidarBackend()
    backend.set_all_rays_to_miss()

    cloud = MultiLineLidar.front(backend, LidarConfig.default()).scan(1_000_000_000)

    assert cloud.timebase_ns == 1_000_000_000
    assert cloud.point_num == 0
    assert cloud.points == ()


def test_range_boundaries_are_inclusive_and_out_of_range_hits_are_removed():
    backend = FakeLidarBackend()
    backend.hit_ray(0, local_point=(0.10, 0.0, 0.0), category="static_obstacle")
    backend.hit_ray(1, local_point=(0.099, 0.0, 0.0), category="terrain")
    backend.hit_ray(2, local_point=(30.0, 0.0, 0.0), category="terrain")
    backend.hit_ray(3, local_point=(30.001, 0.0, 0.0), category="moving_obstacle")

    cloud = MultiLineLidar.front(backend, LidarConfig.default()).scan(5)

    assert len(cloud.points) == 2
    assert (cloud.points[0].x, cloud.points[0].y, cloud.points[0].z) == pytest.approx(
        (0.10, 0.0, 0.0)
    )
    assert (cloud.points[1].x, cloud.points[1].y, cloud.points[1].z) == pytest.approx(
        (30.0, 0.0, 0.0)
    )
    assert tuple(point.offset_time_ns for point in cloud.points) == (
        0,
        2 * LIDAR_SCAN_PERIOD_NS // 2880,
    )


def test_translated_rotated_exact_range_boundaries_survive_pybullet_round_trip():
    backend = FakeLidarBackend()
    backend.parent_poses["lidar_front_mount"] = Pose(
        (10.0, -10.0, 3.0),
        tuple(p.getQuaternionFromEuler((1.1, -0.7, 2.4))),
    )
    rays = build_unit_rays(LidarConfig.default())
    minimum_index = 1075
    maximum_index = 2879
    backend.hit_ray(
        minimum_index,
        local_point=tuple(value * 0.10 for value in rays[minimum_index]),
        category="terrain",
    )
    backend.hit_ray(
        maximum_index,
        local_point=tuple(value * 30.0 for value in rays[maximum_index]),
        category="terrain",
    )

    cloud = MultiLineLidar.front(backend, LidarConfig.default()).scan(10)

    assert tuple(point.offset_time_ns for point in cloud.points) == (
        minimum_index * LIDAR_SCAN_PERIOD_NS // 2880,
        maximum_index * LIDAR_SCAN_PERIOD_NS // 2880,
    )
    assert math.hypot(cloud.points[0].x, cloud.points[0].y, cloud.points[0].z) == pytest.approx(
        0.10,
        abs=1e-5,
    )
    assert math.hypot(cloud.points[1].x, cloud.points[1].y, cloud.points[1].z) == pytest.approx(
        30.0,
        abs=1e-5,
    )


def test_unknown_body_hit_uses_confirmed_unknown_tag_and_reflectivity():
    backend = FakeLidarBackend()
    backend.hit_ray(90, local_point=(1.0, 0.0, 0.0), category="unknown")

    cloud = MultiLineLidar.front(backend, LidarConfig.default()).scan(20)

    assert cloud.point_num == 1
    assert (cloud.points[0].tag, cloud.points[0].reflectivity) == (0, 80)


@pytest.mark.parametrize("invalid_timebase", [True, False, -1, UINT64_MAX + 1, 1.0, math.nan, "1"])
def test_scan_rejects_non_uint64_before_reading_backend(invalid_timebase):
    backend = FakeLidarBackend()
    lidar = MultiLineLidar.front(backend, LidarConfig.default())

    with pytest.raises(ValueError, match="timebase_ns.*uint64"):
        lidar.scan(invalid_timebase)

    assert backend.world_pose_calls == []
    assert backend.ray_batches == []


def test_scan_accepts_uint64_endpoints():
    backend = FakeLidarBackend()
    lidar = MultiLineLidar.rear(backend, LidarConfig.default())

    assert lidar.scan(0).timebase_ns == 0
    assert lidar.scan(UINT64_MAX).timebase_ns == UINT64_MAX


def test_scan_rejects_backend_result_length_mismatch_and_propagates_errors():
    backend = FakeLidarBackend()
    lidar = MultiLineLidar.front(backend, LidarConfig.default())
    backend.truncate_results = True

    with pytest.raises(RuntimeError, match="returned 2879 results.*2880 rays"):
        lidar.scan(1)

    backend.truncate_results = False
    backend.scan_error = LookupError("injected ray failure")
    with pytest.raises(LookupError, match="injected ray failure"):
        lidar.scan(2)


def _world_pose_from_base(
    base: Pose,
    local_position: tuple[float, float, float],
    local_orientation: tuple[float, float, float, float] = IDENTITY_QUATERNION,
) -> Pose:
    position, orientation = p.multiplyTransforms(
        base.position,
        base.orientation,
        local_position,
        local_orientation,
    )
    return Pose(tuple(position), tuple(orientation))


@pytest.mark.parametrize(
    ("side", "mount_offset", "mount_yaw", "local_hit", "expected_base_xy", "lidar_id"),
    (
        ("front", (0.5, 0.2, 0.1), 0.0, (2.0, 1.0, 0.0), (2.5, 1.2), 1),
        ("rear", (-0.4, 0.3, 0.1), math.pi, (1.0, -0.5, 0.0), (-1.4, 0.8), 2),
    ),
)
def test_scan_with_top_view_projects_same_accepted_hit_into_rotated_translated_base(
    side: str,
    mount_offset: tuple[float, float, float],
    mount_yaw: float,
    local_hit: tuple[float, float, float],
    expected_base_xy: tuple[float, float],
    lidar_id: int,
) -> None:
    backend = FakeLidarBackend()
    base = Pose(
        (8.0, -3.0, 1.2),
        tuple(p.getQuaternionFromEuler((0.0, 0.0, 0.65))),
    )
    mount_orientation = tuple(p.getQuaternionFromEuler((0.0, 0.0, mount_yaw)))
    backend.parent_poses["base_link"] = base
    backend.parent_poses[f"lidar_{side}_mount"] = _world_pose_from_base(
        base,
        mount_offset,
    )
    backend.hit_ray(91, local_point=local_hit, category="static_obstacle")
    lidar = MultiLineLidar.front(backend, LidarConfig.default()) if side == "front" else MultiLineLidar.rear(backend, LidarConfig.default())

    result = lidar.scan_with_top_view(123)

    assert isinstance(result, LidarScanResult)
    assert isinstance(result.message, LidarPointCloud)
    assert isinstance(result.top_view, LidarTopViewFrame)
    assert len(backend.ray_batches) == 1
    assert result.message.point_num == len(result.top_view.points) == 1
    assert result.message.points[0].tag == result.top_view.points[0].tag == 2
    assert result.top_view.points[0].lidar_id == lidar_id
    assert (result.top_view.points[0].x, result.top_view.points[0].y) == pytest.approx(
        expected_base_xy,
        abs=3e-6,
    )
    assert mount_orientation == pytest.approx(
        SensorMounts.default().lidar_front.orientation
        if side == "front"
        else SensorMounts.default().lidar_rear.orientation,
        abs=1e-7,
    )


def test_scan_with_top_view_keeps_empty_frames_and_scan_compatibility_uses_one_batch() -> None:
    backend = FakeLidarBackend()
    lidar = MultiLineLidar.front(backend, LidarConfig.default())

    result = lidar.scan_with_top_view(10)
    assert result.message.points == result.top_view.points == ()
    assert result.top_view.timestamp_ns == 10
    assert not hasattr(result, "__dict__")
    with pytest.raises(FrozenInstanceError):
        result.message = result.message
    assert len(backend.ray_batches) == 1

    backend.ray_batches.clear()
    cloud = lidar.scan(20)
    assert type(cloud) is LidarPointCloud
    assert cloud.timebase_ns == 20
    assert len(backend.ray_batches) == 1


@pytest.mark.parametrize("subclass_field", ("message", "top_view"))
def test_lidar_scan_result_rejects_payload_subclasses(subclass_field: str) -> None:
    class MutableCloud(LidarPointCloud):
        pass

    class MutableFrame(LidarTopViewFrame):
        pass

    cloud_type = MutableCloud if subclass_field == "message" else LidarPointCloud
    frame_type = MutableFrame if subclass_field == "top_view" else LidarTopViewFrame
    cloud = cloud_type(1, "lidar_front", 0, 1, ())
    frame = frame_type(1, ())

    with pytest.raises(ValueError, match=subclass_field):
        LidarScanResult(cloud, frame)


def test_lidar_scan_result_rejects_cloud_view_point_tag_mismatch() -> None:
    message = LidarPointCloud(
        1,
        "lidar_front",
        1,
        1,
        (LidarPoint(0, 1.0, 0.0, 0.0, 100, 1, 0),),
    )
    top_view = LidarTopViewFrame(
        1,
        (LidarTopViewPoint(1.0, 0.0, 2, 1),),
    )

    with pytest.raises(ValueError, match="tag"):
        LidarScanResult(message, top_view)


def test_filtered_hits_keep_message_and_rotated_base_view_in_strict_same_order() -> None:
    backend = FakeLidarBackend()
    base = Pose(
        (6.0, -4.0, 1.0),
        tuple(p.getQuaternionFromEuler((0.0, 0.0, 0.8))),
    )
    mount_offset = (0.5, -0.25, 0.1)
    backend.parent_poses["base_link"] = base
    backend.parent_poses["lidar_front_mount"] = _world_pose_from_base(
        base,
        mount_offset,
    )
    backend.hit_ray(10, local_point=(1.0, 0.2, 0.0), category="terrain")
    backend.hit_ray(11, local_point=(0.05, 0.0, 0.0), category="static_obstacle")
    backend.hit_ray(12, local_point=(31.0, 0.0, 0.0), category="static_obstacle")
    backend.hit_ray(13, local_point=(2.0, -0.4, 0.0), category="moving_obstacle")

    result = MultiLineLidar.front(backend, LidarConfig.default()).scan_with_top_view(9)

    assert len(backend.ray_batches) == 1
    assert tuple(point.offset_time_ns for point in result.message.points) == (
        10 * LIDAR_SCAN_PERIOD_NS // 2880,
        13 * LIDAR_SCAN_PERIOD_NS // 2880,
    )
    assert tuple(point.tag for point in result.message.points) == (1, 3)
    assert tuple(point.tag for point in result.top_view.points) == (1, 3)
    for point, expected in zip(
        result.top_view.points,
        ((1.5, -0.05), (2.5, -0.65)),
        strict=True,
    ):
        assert (point.x, point.y) == pytest.approx(expected, abs=3e-6)


def test_front_and_rear_full_hits_produce_5760_ordered_top_view_points() -> None:
    backend = FakeLidarBackend()
    for ray_index in range(LidarConfig.default().ray_count):
        backend.hit_ray(
            ray_index,
            local_point=(1.0 + ray_index / 10_000.0, 0.25, 0.0),
            category="unknown",
        )

    front = MultiLineLidar.front(backend, LidarConfig.default()).scan_with_top_view(1)
    rear = MultiLineLidar.rear(backend, LidarConfig.default()).scan_with_top_view(2)

    assert len(front.top_view.points) + len(rear.top_view.points) == 5760
    assert len(backend.ray_batches) == 2
    for result in (front, rear):
        assert len(result.message.points) == len(result.top_view.points) == 2880
        assert tuple(point.tag for point in result.top_view.points) == tuple(
            point.tag for point in result.message.points
        )
        assert all(point.tag == 0 for point in result.top_view.points)


@pytest.mark.parametrize(
    "invalid_base_inverse",
    (
        "point",
        set(),
        (),
        ((1.0, 2.0),),
        ((math.nan, 0.0, 0.0),),
        ((True, 0.0, 0.0),),
    ),
)
def test_scan_with_top_view_rejects_invalid_base_inverse_transform(
    invalid_base_inverse: object,
) -> None:
    backend = FakeLidarBackend()
    backend.hit_ray(0, local_point=(1.0, 0.0, 0.0), category="terrain")
    backend.inverse_overrides.extend((None, invalid_base_inverse))

    with pytest.raises(RuntimeError, match="inverse transformed hit|ordered sequence|unexpected length"):
        MultiLineLidar.front(backend, LidarConfig.default()).scan_with_top_view(1)

    assert len(backend.ray_batches) == 1
