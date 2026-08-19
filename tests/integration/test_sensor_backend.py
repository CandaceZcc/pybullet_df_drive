# 传感器后端集成测试：锁定位姿变换、语义 link、射线命中分类和 PyBullet 边界校验。
from __future__ import annotations

from dataclasses import FrozenInstanceError
import math
import statistics
import time
from types import SimpleNamespace

import numpy as np
import pybullet as p
import pytest

import slope_sim.sensor_backend as sensor_backend_module
from slope_sim import lidar_pointcloud as lidar_pointcloud_module
from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb
from slope_sim.model_registry import get_robot_model
from slope_sim.obstacles import ObstacleSnapshot
from slope_sim.robot import create_robot
from slope_sim.scene import TERRAIN_FILTER_GROUP, create_slope_scene
from slope_sim.sensor_backend import Pose, PyBulletSensorBackend, RayHit
from slope_sim.lidar_pointcloud import MultiLineLidar, Stage4LidarProfile


IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)
LIDAR_RAY_COUNT = 16 * 180
DUAL_LIDAR_BACKEND_BUDGET_MS = 80.0


def _assert_same_orientation(actual, expected, *, abs_tol: float = 1e-7) -> None:
    """四元数 q 与 -q 表示同一旋转，比较绝对内积避免符号歧义。"""
    dot = sum(float(left) * float(right) for left, right in zip(actual, expected))
    assert abs(dot) == pytest.approx(1.0, abs=abs_tol)


def _create_backend(model_name: str = "df_back") -> tuple[int, PyBulletSensorBackend]:
    client_id = p.connect(p.DIRECT)
    robot = create_robot(client_id, model_name, start_x=10.0, start_y=10.0)
    return client_id, PyBulletSensorBackend(client_id, robot.robot_id)


def _create_static_box(
    client_id: int,
    position: tuple[float, float, float],
) -> int:
    collision_id = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=(0.25, 0.25, 0.25),
        physicsClientId=client_id,
    )
    return p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision_id,
        basePosition=position,
        physicsClientId=client_id,
    )


def _build_lidar_batch(
    origin: tuple[float, float, float],
    *,
    rear: bool,
) -> tuple[tuple[tuple[float, float, float], ...], tuple[tuple[float, float, float], ...]]:
    """生成固定 16x180 前向或后向真实性能射线。"""
    directions = []
    yaw_offset = math.pi if rear else 0.0
    for line in range(16):
        elevation = math.radians(-15.0 + line * 30.0 / 15.0)
        for sample in range(180):
            azimuth = yaw_offset + math.radians(-90.0 + sample * 180.0 / 179.0)
            directions.append(
                (
                    math.cos(elevation) * math.cos(azimuth),
                    math.cos(elevation) * math.sin(azimuth),
                    math.sin(elevation),
                )
            )
    starts = (origin,) * len(directions)
    ends = tuple(
        (
            origin[0] + 30.0 * direction[0],
            origin[1] + 30.0 * direction[1],
            origin[2] + 30.0 * direction[2],
        )
        for direction in directions
    )
    return starts, ends


def test_pose_and_ray_hit_are_immutable_and_pose_normalizes_quaternion():
    pose = Pose((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 2.0))
    hit = RayHit(
        position=(4.0, 5.0, 6.0),
        body_id=7,
        link_index=-1,
        category="terrain",
    )

    assert pose.orientation == IDENTITY_QUATERNION
    assert hit.hit is True
    with pytest.raises(FrozenInstanceError):
        pose.position = (0.0, 0.0, 0.0)
    with pytest.raises(FrozenInstanceError):
        hit.category = "unknown"


def test_pose_quaternion_normalization_is_idempotent():
    first = Pose((1.0, 2.0, 3.0), (1.0, 2.0, 3.0, 4.0))

    second = Pose(first.position, first.orientation)

    assert second.orientation == first.orientation


@pytest.mark.parametrize(
    ("position", "orientation", "message"),
    [
        ((math.nan, 0.0, 0.0), IDENTITY_QUATERNION, "position"),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), "quaternion"),
        ((0.0, 0.0, 0.0), (0.0, math.inf, 0.0, 1.0), "quaternion"),
    ],
)
def test_pose_rejects_non_finite_position_and_invalid_quaternion(position, orientation, message):
    with pytest.raises(ValueError, match=message):
        Pose(position, orientation)


def test_pose_rejects_unordered_and_one_shot_position_values():
    with pytest.raises(ValueError, match="position"):
        Pose({0.0, 1.0, 2.0}, IDENTITY_QUATERNION)
    with pytest.raises(ValueError, match="position"):
        Pose(iter((0.0, 1.0, 2.0)), IDENTITY_QUATERNION)


def test_backend_exposes_semantic_links_and_recovers_base_link_frame():
    client_id, backend = _create_backend("df_front")
    try:
        names = backend.link_names()
        base_pose = backend.world_pose("base_link")
        world_inertial_position, world_inertial_orientation = p.getBasePositionAndOrientation(
            backend.robot_id,
            physicsClientId=client_id,
        )
        dynamics = p.getDynamicsInfo(
            backend.robot_id,
            -1,
            physicsClientId=client_id,
        )
        inverse_position, inverse_orientation = p.invertTransform(
            dynamics[3],
            dynamics[4],
        )
        expected_position, expected_orientation = p.multiplyTransforms(
            world_inertial_position,
            world_inertial_orientation,
            inverse_position,
            inverse_orientation,
        )

        assert names[0] == "base_link"
        assert "lidar_front_mount" in names
        assert "lidar_rear_mount" in names
        assert base_pose.position == pytest.approx(expected_position)
        _assert_same_orientation(base_pose.orientation, expected_orientation)
        assert math.dist(base_pose.position, world_inertial_position) > 0.03
        with pytest.raises(ValueError, match="parent link"):
            backend.world_pose("missing_link")
    finally:
        p.disconnect(client_id)


def test_base_link_recovery_handles_nonzero_inertial_position_and_rotation():
    client_id = p.connect(p.DIRECT)
    expected_position = (1.1, -2.2, 0.8)
    expected_orientation = tuple(p.getQuaternionFromEuler((-0.4, 0.25, 0.7)))
    local_inertial_position = (0.13, -0.07, 0.05)
    local_inertial_orientation = tuple(p.getQuaternionFromEuler((0.2, -0.3, 0.4)))
    try:
        body_id = p.createMultiBody(
            baseMass=1.0,
            basePosition=expected_position,
            baseOrientation=expected_orientation,
            baseInertialFramePosition=local_inertial_position,
            baseInertialFrameOrientation=local_inertial_orientation,
            physicsClientId=client_id,
        )

        pose = PyBulletSensorBackend(client_id, body_id).world_pose("base_link")

        assert pose.position == pytest.approx(expected_position, abs=1e-7)
        _assert_same_orientation(pose.orientation, expected_orientation)
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(
    ("local_position", "local_orientation", "message"),
    [
        ((math.nan, 0.0, 0.0), IDENTITY_QUATERNION, "finite"),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), "quaternion"),
        ((0.0, 0.0, 0.0), (0.0, math.inf, 0.0, 1.0), "quaternion"),
    ],
)
def test_base_link_recovery_rejects_invalid_local_inertial_pose(
    monkeypatch,
    local_position,
    local_orientation,
    message,
):
    client_id, backend = _create_backend()
    original_get_dynamics_info = sensor_backend_module.p.getDynamicsInfo

    def invalid_dynamics_info(*args, **kwargs):
        fields = list(original_get_dynamics_info(*args, **kwargs))
        fields[3] = local_position
        fields[4] = local_orientation
        return tuple(fields)

    try:
        monkeypatch.setattr(
            sensor_backend_module.p,
            "getDynamicsInfo",
            invalid_dynamics_info,
        )
        with pytest.raises(RuntimeError, match=message):
            backend.world_pose("base_link")
    finally:
        p.disconnect(client_id)


def test_non_base_world_pose_uses_world_link_frame_state(monkeypatch):
    client_id, backend = _create_backend()
    world_link_position = (1.0, 2.0, 3.0)
    world_link_orientation = tuple(p.getQuaternionFromEuler((0.1, -0.2, 0.3)))

    def fake_get_link_state(*_args, **_kwargs):
        return (
            (91.0, 92.0, 93.0),
            IDENTITY_QUATERNION,
            (0.0, 0.0, 0.0),
            IDENTITY_QUATERNION,
            world_link_position,
            world_link_orientation,
        )

    try:
        monkeypatch.setattr(sensor_backend_module.p, "getLinkState", fake_get_link_state)
        pose = backend.world_pose("lidar_front_mount")

        assert pose.position == pytest.approx(world_link_position)
        assert pose.orientation == pytest.approx(world_link_orientation)
    finally:
        p.disconnect(client_id)


def test_backend_transforms_pose_point_and_quaternion_coordinates():
    client_id, backend = _create_backend()
    try:
        parent = Pose(
            (1.0, 2.0, 3.0),
            tuple(p.getQuaternionFromEuler((0.0, 0.0, math.pi / 2.0))),
        )
        local = Pose(
            (1.0, 0.0, 0.5),
            tuple(p.getQuaternionFromEuler((0.2, 0.0, 0.0))),
        )

        world = backend.transform_pose(parent, local)
        local_again = backend.inverse_transform_point(world, world.position)
        local_batch = backend.inverse_transform_points(
            world,
            (
                world.position,
                backend.transform_pose(
                    world,
                    Pose((0.5, -0.25, 1.0), IDENTITY_QUATERNION),
                ).position,
            ),
        )
        roll, pitch, yaw = backend.euler_from_quaternion(world.orientation)

        assert world.position == pytest.approx((1.0, 3.0, 3.5), abs=1e-7)
        assert local_again == pytest.approx((0.0, 0.0, 0.0), abs=1e-7)
        assert local_batch[0] == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
        assert local_batch[1] == pytest.approx((0.5, -0.25, 1.0), abs=1e-6)
        assert (roll, pitch, yaw) == pytest.approx((0.2, 0.0, math.pi / 2.0), abs=1e-7)
    finally:
        p.disconnect(client_id)


def test_bind_scene_maps_temporary_body_ids_to_stable_hit_categories():
    client_id, backend = _create_backend()
    try:
        terrain_id = _create_static_box(client_id, (0.0, 0.0, 0.0))
        static_id = _create_static_box(client_id, (0.0, 2.0, 0.0))
        moving_id = _create_static_box(client_id, (0.0, 4.0, 0.0))
        unknown_id = _create_static_box(client_id, (0.0, 6.0, 0.0))
        backend.bind_scene(
            (terrain_id,),
            (
                ObstacleSnapshot(1, static_id, "static", "box", (0.0, 2.0, 0.0), IDENTITY_QUATERNION),
                ObstacleSnapshot(2, moving_id, "moving", "box", (0.0, 4.0, 0.0), IDENTITY_QUATERNION),
            ),
        )
        starts = tuple((-1.0, y, 0.0) for y in (0.0, 2.0, 4.0, 6.0, 8.0))
        ends = tuple((1.0, y, 0.0) for y in (0.0, 2.0, 4.0, 6.0, 8.0))

        hits = backend.ray_test_batch(starts, ends, collision_mask=0xFFFF)

        assert tuple(hit.category for hit in hits) == (
            "terrain",
            "static_obstacle",
            "moving_obstacle",
            "unknown",
            "unknown",
        )
        assert tuple(hit.body_id for hit in hits[:4]) == (terrain_id, static_id, moving_id, unknown_id)
        assert all(hit.hit for hit in hits[:4])
        assert hits[-1].hit is False
        assert hits[-1].body_id == -1
    finally:
        p.disconnect(client_id)


def test_bind_scene_rejects_generators_and_mappings_before_tuple_conversion():
    client_id, backend = _create_backend()
    try:
        with pytest.raises(ValueError, match="terrain_body_ids"):
            backend.bind_scene(iter((1,)), ())
        with pytest.raises(ValueError, match="terrain_body_ids"):
            backend.bind_scene({1: "terrain"}, ())
        with pytest.raises(ValueError, match="obstacles"):
            backend.bind_scene((1,), iter(()))
        with pytest.raises(ValueError, match="obstacles"):
            backend.bind_scene((1,), {"item": object()})
    finally:
        p.disconnect(client_id)


def test_bind_scene_validates_mode_even_when_obstacle_has_no_body_id():
    client_id, backend = _create_backend()
    try:
        invalid_obstacle = SimpleNamespace(body_id=None, mode="teleporting")
        with pytest.raises(ValueError, match="mode"):
            backend.bind_scene((), (invalid_obstacle,))
    finally:
        p.disconnect(client_id)


def test_bind_scene_rejects_unhashable_mode_with_stable_value_error():
    client_id, backend = _create_backend()
    try:
        invalid_obstacle = SimpleNamespace(body_id=None, mode=["static"])
        with pytest.raises(ValueError, match="mode.*string"):
            backend.bind_scene((), (invalid_obstacle,))
    finally:
        p.disconnect(client_id)


def test_two_real_2880_ray_batches_fit_10hz_backend_budget_with_stable_semantics():
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=1.0 / 240.0,
            terrain_model="flat",
        )
        spec = get_robot_model("df_back")
        robot = create_robot(
            client_id,
            "df_back",
            start_x=scene.spawn_position[0],
            start_y=scene.spawn_position[1],
            base_height=scene.spawn_position[2] + spec.base_height,
        )
        backend = PyBulletSensorBackend(client_id, robot.robot_id)
        backend.bind_scene(scene.body_ids, ())
        sensor_z = scene.spawn_position[2] + 0.40
        front_rays = _build_lidar_batch(
            (scene.spawn_position[0] + 0.34, scene.spawn_position[1], sensor_z),
            rear=False,
        )
        rear_rays = _build_lidar_batch(
            (scene.spawn_position[0] - 0.34, scene.spawn_position[1], sensor_z),
            rear=True,
        )

        def scan_pair():
            front_hits = backend.ray_test_batch(
                *front_rays,
                collision_mask=TERRAIN_FILTER_GROUP,
            )
            rear_hits = backend.ray_test_batch(
                *rear_rays,
                collision_mask=TERRAIN_FILTER_GROUP,
            )
            return front_hits, rear_hits

        for _ in range(2):
            scan_pair()
        elapsed_ms = []
        for _ in range(5):
            started = time.perf_counter()
            front_hits, rear_hits = scan_pair()
            elapsed_ms.append((time.perf_counter() - started) * 1000.0)

        assert len(front_hits) == len(rear_hits) == LIDAR_RAY_COUNT
        for hits in (front_hits, rear_hits):
            assert any(hit.hit for hit in hits)
            assert any(not hit.hit for hit in hits)
            for hit in hits:
                if hit.hit:
                    assert hit.body_id in scene.body_ids
                    assert hit.link_index == -1
                    assert hit.category == "terrain"
                else:
                    assert (hit.body_id, hit.link_index, hit.category) == (-1, -1, "unknown")
        assert statistics.median(elapsed_ms) < DUAL_LIDAR_BACKEND_BUDGET_MS
    finally:
        p.disconnect(client_id)


def test_stage4_numpy_batch_matches_scalar_oracle():
    """Stage4 ndarray 批次保持标量端点、命中和确定性 protobuf 语义。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=1.0 / 240.0,
            terrain_model="golf_heightfield",
            golf_seed=41,
            golf_relief="medium",
        )
        spec = get_robot_model("df_back")
        robot = create_robot(
            client_id,
            "df_back",
            start_x=scene.spawn_position[0],
            start_y=scene.spawn_position[1],
            base_height=scene.spawn_position[2] + spec.base_height,
        )
        backend = PyBulletSensorBackend(client_id, robot.robot_id)
        backend.bind_scene(scene.body_ids, ())
        scanner = MultiLineLidar.stage4(backend, Stage4LidarProfile.realtime())
        mount = scanner._world_mount()
        pattern_version = "livox-mid360-800000-v1"
        world_generation = 3
        sequence = 7
        global_slots = tuple(range(5_760))
        scalar_directions = tuple(
            lidar_pointcloud_module.mid360_direction_for_slot(
                pattern_version,
                world_generation,
                sequence,
                global_slot,
            )
            for global_slot in global_slots
        )
        scalar_starts = lidar_pointcloud_module._transform_points(
            mount,
            tuple(
                tuple(component * 0.10 for component in direction)
                for direction in scalar_directions
            ),
        )
        scalar_ends = lidar_pointcloud_module._transform_points(
            mount,
            tuple(
                tuple(component * 45.0 for component in direction)
                for direction in scalar_directions
            ),
        )
        ndarray_starts, ndarray_ends = scanner._stage4_world_rays_for_slots(
            mount,
            pattern_version=pattern_version,
            world_generation=world_generation,
            sequence=sequence,
            global_slots=global_slots,
        )

        assert ndarray_starts.dtype == ndarray_ends.dtype == np.dtype("float64")
        assert ndarray_starts.flags.c_contiguous and ndarray_ends.flags.c_contiguous
        assert ndarray_starts.flags.writeable is False
        assert ndarray_ends.flags.writeable is False
        assert ndarray_starts.shape == ndarray_ends.shape == (5760, 3)
        np.testing.assert_allclose(ndarray_starts, scalar_starts, rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(ndarray_ends, scalar_ends, rtol=0.0, atol=1e-12)

        scalar_hits = backend.ray_test_batch(
            scalar_starts,
            scalar_ends,
            collision_mask=TERRAIN_FILTER_GROUP,
        )
        indexed_hits = backend._ray_test_indexed_hits_ndarray(
            ndarray_starts,
            ndarray_ends,
            collision_mask=TERRAIN_FILTER_GROUP,
        )
        indexed_by_ray = dict(indexed_hits)
        assert tuple(indexed_by_ray) == tuple(sorted(indexed_by_ray))
        assert len(indexed_by_ray) == len(set(indexed_by_ray))

        scalar_values = []
        ndarray_values = []
        for ray_index, scalar_hit in enumerate(scalar_hits):
            global_slot = global_slots[ray_index]
            ndarray_hit = indexed_by_ray.get(ray_index)
            assert (ndarray_hit is not None) is scalar_hit.hit
            if not scalar_hit.hit:
                continue
            assert ndarray_hit is not None
            assert (
                ndarray_hit.body_id,
                ndarray_hit.link_index,
                ndarray_hit.category,
            ) == (
                scalar_hit.body_id,
                scalar_hit.link_index,
                scalar_hit.category,
            )
            assert ndarray_hit.hit_position == pytest.approx(
                scalar_hit.hit_position,
                abs=0.001,
            )
            scalar_local = backend.inverse_transform_point(mount, scalar_hit.hit_position)
            ndarray_local = backend.inverse_transform_point(mount, ndarray_hit.hit_position)
            scalar_value = scanner._stage4_point_values_from_hit(
                global_slot,
                scalar_hit,
                scalar_local,
                pattern_version=pattern_version,
                world_generation=world_generation,
                sequence=sequence,
            )
            ndarray_value = scanner._stage4_point_values_from_hit(
                global_slot,
                ndarray_hit,
                ndarray_local,
                pattern_version=pattern_version,
                world_generation=world_generation,
                sequence=sequence,
            )
            assert (ndarray_value is None) is (scalar_value is None)
            if scalar_value is not None:
                assert ndarray_value is not None
                assert ndarray_value[:1] + ndarray_value[4:] == scalar_value[:1] + scalar_value[4:]
                assert ndarray_value[1:4] == pytest.approx(scalar_value[1:4], abs=0.001)
                scalar_values.append(scalar_value)
                ndarray_values.append(ndarray_value)

        identity = dict(
            sequence=sequence,
            world_generation=world_generation,
            simulation_session_id=b"\x01" * 16,
            descriptor_sha256=b"\x02" * 32,
        )
        scalar_payload = pb.LidarPointCloud(
            timebase_ns=1_000_000_000,
            frame_id="lidar_link",
            lidar_id=1,
            **identity,
        )
        for value in scalar_values:
            scalar_payload.points.add(
                offset_time_ns=value[0],
                x=value[1],
                y=value[2],
                z=value[3],
                reflectivity=value[4],
                tag=value[5],
                line=value[6],
            )
        scalar_payload.point_num = len(scalar_values)
        ndarray_payload = pb.LidarPointCloud(
            timebase_ns=1_000_000_000,
            frame_id="lidar_link",
            lidar_id=1,
            **identity,
        )
        for value in ndarray_values:
            ndarray_payload.points.add(
                offset_time_ns=value[0],
                x=value[1],
                y=value[2],
                z=value[3],
                reflectivity=value[4],
                tag=value[5],
                line=value[6],
            )
        ndarray_payload.point_num = len(ndarray_values)
        assert ndarray_payload.SerializeToString(deterministic=True) == scalar_payload.SerializeToString(
            deterministic=True
        )
    finally:
        p.disconnect(client_id)


def test_stage4_ndarray_batch_uses_explicit_private_thread_count(monkeypatch) -> None:
    """Stage4 shard 只能通过私有 ndarray 入口指定单个 PyBullet 内部线程。"""
    client_id, backend = _create_backend()
    starts = np.zeros((2, 3), dtype=np.float64)
    ends = np.ones((2, 3), dtype=np.float64)
    starts.setflags(write=False)
    ends.setflags(write=False)
    observed: dict[str, object] = {}

    def ray_test_batch(starts, ends, *, numThreads, collisionFilterMask, physicsClientId):
        observed.update(
            starts=starts,
            ends=ends,
            num_threads=numThreads,
            collision_mask=collisionFilterMask,
            client_id=physicsClientId,
        )
        return (
            (-1, -1, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            (-1, -1, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        )

    try:
        monkeypatch.setattr(sensor_backend_module.p, "rayTestBatch", ray_test_batch)

        assert backend._ray_test_indexed_hits_ndarray(
            starts,
            ends,
            collision_mask=0x10,
            num_threads=1,
        ) == ()

        assert observed == {
            "starts": starts,
            "ends": ends,
            "num_threads": 1,
            "collision_mask": 0x10,
            "client_id": client_id,
        }
    finally:
        p.disconnect(client_id)


def test_ray_batch_rejects_bad_lengths_non_finite_points_and_invalid_mask(monkeypatch):
    client_id, backend = _create_backend()
    calls = 0

    def record_call(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        return ()

    try:
        monkeypatch.setattr(sensor_backend_module.p, "rayTestBatch", record_call)
        with pytest.raises(ValueError, match="same length"):
            backend.ray_test_batch(((0.0, 0.0, 0.0),), (), collision_mask=1)
        with pytest.raises(ValueError, match="finite"):
            backend.ray_test_batch(
                ((0.0, math.nan, 0.0),),
                ((1.0, 0.0, 0.0),),
                collision_mask=1,
            )
        with pytest.raises(ValueError, match="sequence"):
            backend.ray_test_batch(
                {(0.0, 0.0, 0.0)},
                ((1.0, 0.0, 0.0),),
                collision_mask=1,
            )
        for invalid_mask in (True, -1, 1.5):
            with pytest.raises(ValueError, match="collision_mask"):
                backend.ray_test_batch((), (), collision_mask=invalid_mask)
        assert calls == 0
        assert backend.ray_test_batch((), (), collision_mask=0) == ()
        assert backend.ray_test_batch((), (), collision_mask=0x7FFFFFFF) == ()
        assert calls == 0
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("invalid_mask", (True, 0x80000000, 0xFFFFFFFF, 2**32))
def test_ray_batch_rejects_masks_outside_signed_c_int_for_empty_and_nonempty_batches(
    monkeypatch,
    invalid_mask,
):
    client_id, backend = _create_backend()
    calls = 0

    def record_call(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        return ()

    try:
        monkeypatch.setattr(sensor_backend_module.p, "rayTestBatch", record_call)
        with pytest.raises(ValueError, match="collision_mask"):
            backend.ray_test_batch((), (), collision_mask=invalid_mask)
        with pytest.raises(ValueError, match="collision_mask"):
            backend.ray_test_batch(
                ((0.0, 0.0, 0.0),),
                ((1.0, 0.0, 0.0),),
                collision_mask=invalid_mask,
            )
        assert calls == 0
    finally:
        p.disconnect(client_id)


def test_ray_batch_rejects_more_than_pybullet_maximum_before_native_call(monkeypatch):
    client_id, backend = _create_backend()
    too_many_points = ((0.0, 0.0, 0.0),) * 16384
    calls = 0

    def record_call(*_args, **_kwargs):
        nonlocal calls
        calls += 1
        return ()

    try:
        monkeypatch.setattr(sensor_backend_module.p, "rayTestBatch", record_call)
        with pytest.raises(ValueError, match="16383"):
            backend.ray_test_batch(
                too_many_points,
                too_many_points,
                collision_mask=1,
            )
        assert calls == 0
    finally:
        p.disconnect(client_id)


def test_ray_batch_accepts_real_pybullet_maximum_size():
    client_id, backend = _create_backend()
    starts = ((100.0, 100.0, 100.0),) * 16383
    ends = ((101.0, 100.0, 100.0),) * 16383
    try:
        hits = backend.ray_test_batch(starts, ends, collision_mask=0)

        assert len(hits) == 16383
        assert all(not hit.hit for hit in hits)
        assert all(hit.category == "unknown" for hit in hits)
    finally:
        p.disconnect(client_id)


def test_ray_batch_passes_collision_mask_and_rejects_return_length_mismatch(monkeypatch):
    client_id, backend = _create_backend()
    observed_mask = None

    def mismatched_result(_starts, _ends, *, collisionFilterMask, physicsClientId):
        nonlocal observed_mask
        observed_mask = collisionFilterMask
        assert physicsClientId == client_id
        return ()

    try:
        monkeypatch.setattr(sensor_backend_module.p, "rayTestBatch", mismatched_result)
        with pytest.raises(RuntimeError, match="returned 0 results for 1 rays"):
            backend.ray_test_batch(
                ((0.0, 0.0, 0.0),),
                ((1.0, 0.0, 0.0),),
                collision_mask=0x10,
            )
        assert observed_mask == 0x10
    finally:
        p.disconnect(client_id)


def test_indexed_hit_batch_uses_parallel_pybullet_and_omits_missed_rays(monkeypatch):
    client_id, backend = _create_backend()
    backend.bind_scene((7,), ())
    observed: dict[str, object] = {}
    raw_results = (
        (-1, -1, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        (7, -1, 0.5, (1.0, 2.0, 3.0), (0.0, 0.0, 1.0)),
    )

    def indexed_results(
        starts,
        ends,
        *,
        numThreads,
        collisionFilterMask,
        physicsClientId,
    ):
        observed.update(
            starts=starts,
            ends=ends,
            num_threads=numThreads,
            collision_mask=collisionFilterMask,
            client_id=physicsClientId,
        )
        return raw_results

    starts = ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    ends = ((2.0, 0.0, 0.0), (2.0, 1.0, 0.0))
    try:
        monkeypatch.setattr(sensor_backend_module.p, "rayTestBatch", indexed_results)

        indexed_hits = backend.ray_test_indexed_hits(
            starts,
            ends,
            collision_mask=0x10,
        )

        assert indexed_hits == ((1, RayHit((1.0, 2.0, 3.0), 7, -1, "terrain")),)
        assert observed == {
            "starts": starts,
            "ends": ends,
            "num_threads": 0,
            "collision_mask": 0x10,
            "client_id": client_id,
        }
    finally:
        p.disconnect(client_id)


def test_ray_batch_rejects_result_with_extra_fields(monkeypatch):
    client_id, backend = _create_backend()
    raw_hit = (1, -1, 0.5, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), "extra")
    try:
        monkeypatch.setattr(
            sensor_backend_module.p,
            "rayTestBatch",
            lambda *_args, **_kwargs: (raw_hit,),
        )
        with pytest.raises(RuntimeError, match="exactly 5 fields"):
            backend.ray_test_batch(
                ((0.0, 0.0, 0.0),),
                ((1.0, 0.0, 0.0),),
                collision_mask=1,
            )
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(
    "raw_hit",
    [
        (1, -1, math.nan, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        (1, -1, 0.5, (math.inf, 0.0, 0.0), (0.0, 0.0, 1.0)),
        (1, -1, 0.5, (0.0, 0.0, 0.0), (0.0, math.nan, 1.0)),
    ],
)
def test_ray_batch_rejects_non_finite_pybullet_results(monkeypatch, raw_hit):
    client_id, backend = _create_backend()
    try:
        monkeypatch.setattr(
            sensor_backend_module.p,
            "rayTestBatch",
            lambda *_args, **_kwargs: (raw_hit,),
        )
        with pytest.raises(RuntimeError, match="finite"):
            backend.ray_test_batch(
                ((0.0, 0.0, 0.0),),
                ((1.0, 0.0, 0.0),),
                collision_mask=1,
            )
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(
    ("raw_hit", "message"),
    [
        ((True, -1, 0.5, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)), "body id"),
        ((1, True, 0.5, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)), "link index"),
        ((1, -1, True, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)), "hit fraction"),
        ((-2, -1, 0.5, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)), "body id"),
        ((1, -2, 0.5, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)), "link index"),
        (
            (1, -1, math.nextafter(0.0, -math.inf), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            "range 0..1",
        ),
        (
            (1, -1, math.nextafter(1.0, math.inf), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
            "range 0..1",
        ),
        ((1, -1, 0.5, (0.0, 0.0), (0.0, 0.0, 1.0)), "position"),
        ((1, -1, 0.5, (0.0, 0.0, 0.0), (0.0, 1.0)), "normal"),
        ((1, -1, 0.5, (True, 0.0, 0.0), (0.0, 0.0, 1.0)), "position"),
        ((1, -1, 0.5, (0.0, 0.0, 0.0), (False, 0.0, 1.0)), "normal"),
    ],
)
def test_trusted_ray_conversion_rejects_invalid_pybullet_fields(
    monkeypatch,
    raw_hit,
    message,
):
    client_id, backend = _create_backend()
    try:
        monkeypatch.setattr(
            sensor_backend_module.p,
            "rayTestBatch",
            lambda *_args, **_kwargs: (raw_hit,),
        )
        with pytest.raises(RuntimeError, match=message):
            backend.ray_test_batch(
                ((0.0, 0.0, 0.0),),
                ((1.0, 0.0, 0.0),),
                collision_mask=1,
            )
    finally:
        p.disconnect(client_id)
