# 多线点云 DIRECT 门禁：验证三地形、自身过滤、前后障碍标签和连续移动扫描。
from __future__ import annotations

from dataclasses import dataclass
import itertools
import math
import statistics
import time

import pybullet as p
import pytest

from slope_sim.lidar_pointcloud import LidarConfig, MultiLineLidar
from slope_sim.model_registry import get_robot_model
from slope_sim.obstacles import ObstacleSnapshot, create_box_obstacle, update_kinematic_obstacle
from slope_sim.robot import create_robot
from slope_sim.scene import create_slope_scene
from slope_sim.sensor_backend import Pose, PyBulletSensorBackend, RayHit
from slope_sim.truth_sensors import MountPose, SensorMounts


TIME_STEP = 1.0 / 240.0
IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)
ROBOT_MODEL = "df_back"
DUAL_LIDAR_SCAN_BUDGET_MS = 100.0


class RecordingPyBulletSensorBackend(PyBulletSensorBackend):
    """保留真实批量射线结果，门禁可直接核对命中 body 而非猜测点标签。"""

    def __init__(self, client_id: int, robot_id: int) -> None:
        super().__init__(client_id, robot_id)
        self.ray_history: list[tuple[RayHit, ...]] = []

    def ray_test_batch(self, starts, ends, *, collision_mask: int) -> tuple[RayHit, ...]:
        hits = super().ray_test_batch(starts, ends, collision_mask=collision_mask)
        self.ray_history.append(hits)
        return hits


@dataclass(frozen=True)
class LidarDirectGateResult:
    """三地形扫描的自身命中、地形命中和点云字段摘要。"""

    front_self_hits: int
    rear_self_hits: int
    front_terrain_hits: int
    rear_terrain_hits: int
    front_tags: tuple[int, ...]
    rear_tags: tuple[int, ...]
    front_point_count: int
    rear_point_count: int


@dataclass(frozen=True)
class LidarObstacleGateResult:
    """前后视场中的障碍标签及真实 body 命中集合。"""

    front_tags: frozenset[int]
    rear_tags: frozenset[int]
    front_body_ids: frozenset[int]
    rear_body_ids: frozenset[int]
    expected_front_body_id: int
    expected_rear_body_id: int


@dataclass(frozen=True)
class MovingObstacleSequenceResult:
    """连续扫描中移动障碍物点的最近局部量程。"""

    closest_ranges_m: tuple[float, ...]


def _create_settled_world(client_id: int, terrain_model: str):
    """创建正式地形和车型并稳定接触，随后才读取安装 link 世界位姿。"""
    scene = create_slope_scene(
        client_id,
        slope_deg=8.0 if terrain_model == "slope" else 0.0,
        time_step=TIME_STEP,
        terrain_model=terrain_model,
        golf_seed=31,
        golf_relief="medium",
    )
    spec = get_robot_model(ROBOT_MODEL)
    robot = create_robot(
        client_id,
        ROBOT_MODEL,
        start_x=scene.spawn_position[0],
        start_y=scene.spawn_position[1],
        base_height=scene.spawn_position[2] + spec.base_height,
        start_orientation=scene.spawn_orientation,
    )
    robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
    for _ in range(180):
        robot.command_wheel_speeds((0.0, 0.0), (), dt=TIME_STEP)
        p.stepSimulation(physicsClientId=client_id)
    return scene, robot


def _world_mount(backend: PyBulletSensorBackend, mount: MountPose) -> Pose:
    """按 Task 8 正式接口组合 parent link 与冻结安装外参。"""
    return backend.transform_pose(
        backend.world_pose(mount.parent_link),
        Pose(mount.position, mount.orientation),
    )


def _point_from_mount(
    backend: PyBulletSensorBackend,
    mount: Pose,
    local_point: tuple[float, float, float],
) -> tuple[float, float, float]:
    return backend.transform_pose(mount, Pose(local_point, IDENTITY_QUATERNION)).position


def run_lidar_direct_gate(terrain_model: str) -> LidarDirectGateResult:
    """在独立 DIRECT client 中生成前后真实点云并核对原始 body id。"""
    client_id = p.connect(p.DIRECT)
    result = None
    try:
        scene, robot = _create_settled_world(client_id, terrain_model)
        backend = RecordingPyBulletSensorBackend(client_id, robot.robot_id)
        backend.bind_scene(scene.body_ids, ())
        front_cloud = MultiLineLidar.front(backend, LidarConfig.default()).scan(100_000_000)
        rear_cloud = MultiLineLidar.rear(backend, LidarConfig.default()).scan(100_000_000)
        front_hits, rear_hits = backend.ray_history
        result = LidarDirectGateResult(
            front_self_hits=sum(hit.body_id == robot.robot_id for hit in front_hits),
            rear_self_hits=sum(hit.body_id == robot.robot_id for hit in rear_hits),
            front_terrain_hits=sum(hit.body_id in scene.body_ids for hit in front_hits),
            rear_terrain_hits=sum(hit.body_id in scene.body_ids for hit in rear_hits),
            front_tags=tuple(point.tag for point in front_cloud.points),
            rear_tags=tuple(point.tag for point in rear_cloud.points),
            front_point_count=front_cloud.point_num,
            rear_point_count=rear_cloud.point_num,
        )
        assert front_cloud.point_num == len(front_cloud.points)
        assert rear_cloud.point_num == len(rear_cloud.points)
        assert all(
            math.isfinite(value)
            for cloud in (front_cloud, rear_cloud)
            for point in cloud.points
            for value in (point.x, point.y, point.z)
        )
    finally:
        p.disconnect(client_id)

    assert p.isConnected(client_id) == 0
    assert result is not None
    return result


@pytest.mark.parametrize("terrain_model", ("flat", "slope", "golf_heightfield"))
def test_front_and_rear_clouds_hit_terrain_without_hitting_robot(terrain_model):
    result = run_lidar_direct_gate(terrain_model)

    assert result.front_self_hits == 0
    assert result.rear_self_hits == 0
    assert result.front_terrain_hits > 0
    assert result.rear_terrain_hits > 0
    assert result.front_point_count == result.front_terrain_hits
    assert result.rear_point_count == result.rear_terrain_hits
    assert set(result.front_tags) == {1}
    assert set(result.rear_tags) == {1}


def test_two_complete_lidar_scans_fit_the_shared_10hz_period():
    """整帧预算包含世界射线、物理 raycast、局部点和不可变消息构造。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene, robot = _create_settled_world(client_id, "flat")
        backend = PyBulletSensorBackend(client_id, robot.robot_id)
        backend.bind_scene(scene.body_ids, ())
        config = LidarConfig.default()
        front = MultiLineLidar.front(backend, config)
        rear = MultiLineLidar.rear(backend, config)
        front.scan(0)
        rear.scan(0)
        elapsed_ms = []

        for scan_index in range(5):
            started_at = time.perf_counter()
            front.scan(scan_index)
            rear.scan(scan_index)
            elapsed_ms.append((time.perf_counter() - started_at) * 1000.0)

        assert statistics.median(elapsed_ms) < DUAL_LIDAR_SCAN_BUDGET_MS
    finally:
        p.disconnect(client_id)


def run_lidar_obstacle_gate(mode: str) -> LidarObstacleGateResult:
    """分别把正式质量零障碍物放到前后局部 +X，验证 mount 四元数决定视场。"""
    client_id = p.connect(p.DIRECT)
    result = None
    try:
        scene, robot = _create_settled_world(client_id, "flat")
        backend = RecordingPyBulletSensorBackend(client_id, robot.robot_id)
        mounts = SensorMounts.default()
        front_mount = _world_mount(backend, mounts.lidar_front)
        rear_mount = _world_mount(backend, mounts.lidar_rear)
        front_position = _point_from_mount(backend, front_mount, (2.0, 0.0, 0.05))
        rear_position = _point_from_mount(backend, rear_mount, (2.0, 0.0, 0.05))
        front_body_id = create_box_obstacle(
            client_id,
            half_extents=(0.25, 0.25, 0.30),
            position=front_position,
        )
        rear_body_id = create_box_obstacle(
            client_id,
            half_extents=(0.25, 0.25, 0.30),
            position=rear_position,
        )
        snapshots = (
            ObstacleSnapshot(1, front_body_id, mode, "box", front_position, IDENTITY_QUATERNION),
            ObstacleSnapshot(2, rear_body_id, mode, "box", rear_position, IDENTITY_QUATERNION),
        )
        backend.bind_scene(scene.body_ids, snapshots)
        front_cloud = MultiLineLidar.front(backend, LidarConfig.default()).scan(200_000_000)
        rear_cloud = MultiLineLidar.rear(backend, LidarConfig.default()).scan(200_000_000)
        front_hits, rear_hits = backend.ray_history
        result = LidarObstacleGateResult(
            front_tags=frozenset(point.tag for point in front_cloud.points),
            rear_tags=frozenset(point.tag for point in rear_cloud.points),
            front_body_ids=frozenset(hit.body_id for hit in front_hits if hit.hit),
            rear_body_ids=frozenset(hit.body_id for hit in rear_hits if hit.hit),
            expected_front_body_id=front_body_id,
            expected_rear_body_id=rear_body_id,
        )
    finally:
        p.disconnect(client_id)

    assert result is not None
    return result


@pytest.mark.parametrize(("mode", "expected_tag"), (("static", 2), ("moving", 3)))
def test_obstacle_in_each_field_of_view_changes_corresponding_cloud(mode, expected_tag):
    result = run_lidar_obstacle_gate(mode)

    assert expected_tag in result.front_tags
    assert expected_tag in result.rear_tags
    assert result.expected_front_body_id in result.front_body_ids
    assert result.expected_rear_body_id in result.rear_body_ids
    assert result.expected_rear_body_id not in result.front_body_ids
    assert result.expected_front_body_id not in result.rear_body_ids


def run_lidar_moving_obstacle_sequence(scan_count: int) -> MovingObstacleSequenceResult:
    """沿前雷达局部 +X 小步移动真实 body，并连续读取五帧相对量程。"""
    client_id = p.connect(p.DIRECT)
    result = None
    try:
        scene, robot = _create_settled_world(client_id, "flat")
        backend = RecordingPyBulletSensorBackend(client_id, robot.robot_id)
        mount = _world_mount(backend, SensorMounts.default().lidar_front)
        initial_position = _point_from_mount(backend, mount, (1.70, 0.0, 0.05))
        obstacle_id = create_box_obstacle(
            client_id,
            half_extents=(0.25, 0.25, 0.30),
            position=initial_position,
        )
        backend.bind_scene(
            scene.body_ids,
            (ObstacleSnapshot(1, obstacle_id, "moving", "box", initial_position, IDENTITY_QUATERNION),),
        )
        next_position = _point_from_mount(backend, mount, (1.73, 0.0, 0.05))
        velocity = tuple((next_position[index] - initial_position[index]) / 0.1 for index in range(3))
        closest_ranges: list[float] = []

        for scan_index in range(scan_count):
            position = _point_from_mount(
                backend,
                mount,
                (1.70 + 0.03 * scan_index, 0.0, 0.05),
            )
            update_kinematic_obstacle(
                client_id,
                obstacle_id,
                position=position,
                orientation=IDENTITY_QUATERNION,
                linear_velocity=velocity,
            )
            p.performCollisionDetection(physicsClientId=client_id)
            cloud = MultiLineLidar.front(backend, LidarConfig.default()).scan(
                scan_index * 100_000_000
            )
            obstacle_points = [point for point in cloud.points if point.tag == 3]
            assert obstacle_points
            closest_ranges.append(
                min(math.sqrt(point.x * point.x + point.y * point.y + point.z * point.z) for point in obstacle_points)
            )
        result = MovingObstacleSequenceResult(tuple(closest_ranges))
    finally:
        p.disconnect(client_id)

    assert result is not None
    return result


def test_moving_obstacle_changes_cloud_continuously_across_five_scans():
    result = run_lidar_moving_obstacle_sequence(scan_count=5)
    adjacent = tuple(itertools.pairwise(result.closest_ranges_m))

    assert len(result.closest_ranges_m) == 5
    assert all(left != right for left, right in adjacent)
    assert max(abs(right - left) for left, right in adjacent) < 0.10
