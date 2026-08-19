"""离线 MID-360 执行器在真实 PyBullet DIRECT 世界中的聚焦验证。"""

import math

import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import build_world_from_scene_document
from slope_sim.lidar_pointcloud import MID360_PATTERN_VERSION
from slope_sim.mid360_offline import (
    OfflineMid360FrameScanner,
    OfflineMid360Profile,
    OfflineMid360Schedule,
)
from slope_sim.sensor_backend import PyBulletSensorBackend
from slope_sim.simulation import initial_scene_document


def test_offline_scanner_uses_real_df_mid_wheel_motion_and_pybullet_hits() -> None:
    """真实轮关节运动期间的 24 批射线必须生成有界、按时间排序的 raw 命中。"""
    config = ExperimentConfig(
        mode="direct",
        duration_sec=0.1,
        robot_model="df_mid",
        terrain_model="flat",
        interface_enabled=False,
        dashboard_enabled=False,
    )
    document = initial_scene_document(config)
    client_id = p.connect(p.DIRECT)
    assert client_id >= 0
    try:
        world, obstacle_manager = build_world_from_scene_document(
            client_id,
            config,
            document,
        )
        robot = world.active_robot.robot
        backend = PyBulletSensorBackend(client_id, robot.robot_id)
        backend.bind_scene(
            world.scene.body_ids,
            obstacle_manager.snapshot(include_body_id=True),
        )
        profile = OfflineMid360Profile.high_fidelity()
        assert profile.max_range_m == 60.0
        scanner = OfflineMid360FrameScanner(
            backend,
            OfflineMid360Schedule(
                profile,
                pattern_version=MID360_PATTERN_VERSION,
                world_generation=1,
            ),
            sequence=0,
        )
        start_position = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]
        robot.command_twist(0.6, 0.0, dt=config.time_step)
        for step in range(24):
            snapshots = obstacle_manager.snapshot(include_body_id=True)
            scanner.capture_step(
                step,
                body_positions_by_id={
                    int(snapshot.body_id): snapshot.position
                    for snapshot in snapshots
                    if snapshot.body_id is not None
                },
            )
            obstacle_manager.update_moving(config.time_step)
            p.stepSimulation(physicsClientId=client_id)
        cloud = scanner.finalize(timebase_ns=0)
        end_position = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]

        assert math.dist(start_position, end_position) > 0.001
        assert 0 < cloud.point_num <= 20_000
        offsets = tuple(point.offset_time_ns for point in cloud.points)
        assert offsets == tuple(sorted(offsets))
        assert offsets[-1] <= 99_995_000
        assert all(
            0.1 - 1e-5 <= math.hypot(point.x, point.y, point.z) <= 60.0 + 1e-5
            for point in cloud.points
        )
    finally:
        p.disconnect(client_id)
