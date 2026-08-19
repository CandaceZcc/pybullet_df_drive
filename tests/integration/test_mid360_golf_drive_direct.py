"""MID-360 Golf 路线在真实 PyBullet DIRECT 世界中的聚焦验证。"""

from __future__ import annotations

import math
from pathlib import Path

import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import build_world_from_scene_document
import slope_sim.mid360_golf_drive as golf_drive
from slope_sim.scene_config import load_scene


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TIME_STEP = 1.0 / 240.0


def test_physics_step_orders_command_then_moving_obstacles_then_bullet(monkeypatch) -> None:
    events: list[str] = []

    class RecordingManager:
        def update_moving(self, dt: float) -> None:
            assert dt == TIME_STEP
            events.append("moving")

    monkeypatch.setattr(
        "slope_sim.mid360_golf_drive.p.stepSimulation",
        lambda *, physicsClientId: events.append(f"step:{physicsClientId}"),
    )

    golf_drive.advance_golf_physics_step(
        client_id=7,
        obstacle_manager=RecordingManager(),
        dt=TIME_STEP,
        apply_command=lambda: events.append("command"),
    )

    assert events == ["command", "moving", "step:7"]


def test_real_df_mid_wheels_follow_approach_without_robot_pose_reset(monkeypatch) -> None:
    config = ExperimentConfig(
        mode="direct",
        duration_sec=1.0,
        time_step=TIME_STEP,
        robot_model="df_mid",
        terrain_model="golf_heightfield",
        golf_seed=41,
        golf_relief="medium",
        interface_enabled=False,
        dashboard_enabled=False,
    )
    document = load_scene(PROJECT_ROOT / "configs/mid360_golf_mapping.yaml")
    client_id = p.connect(p.DIRECT)
    assert client_id >= 0
    try:
        world, obstacle_manager = build_world_from_scene_document(client_id, config, document)
        robot = world.active_robot.robot
        assert world.scene.bounds is not None
        route = golf_drive.build_canonical_golf_route(
            world.scene.bounds,
            spawn_xy=(world.scene.spawn_position[0], world.scene.spawn_position[1]),
        )
        controller = golf_drive.GolfRouteController(route)

        # 先让车辆落稳；之后拦截 pose reset，仅允许运动障碍按既有合同更新位姿。
        for _ in range(48):
            golf_drive.advance_golf_physics_step(
                client_id=client_id,
                obstacle_manager=obstacle_manager,
                dt=TIME_STEP,
            )
        start_position = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]
        start_joint_positions = tuple(
            p.getJointState(robot.robot_id, joint, physicsClientId=client_id)[0]
            for joint in robot.drive_wheel_joint_indices
        )
        moving_before = tuple(
            snapshot.path.progress
            for snapshot in obstacle_manager.snapshot()
            if snapshot.path is not None
        )

        robot_pose_resets: list[int] = []
        original_reset = p.resetBasePositionAndOrientation

        def record_reset(body_id, *args, **kwargs):
            if body_id == robot.robot_id:
                robot_pose_resets.append(body_id)
            return original_reset(body_id, *args, **kwargs)

        monkeypatch.setattr(p, "resetBasePositionAndOrientation", record_reset)
        next_command_ns = 0
        for physics_step in range(240):
            physics_time_ns = round(physics_step * 1_000_000_000 / 240)

            def apply_due_command() -> None:
                nonlocal next_command_ns
                if physics_time_ns < next_command_ns:
                    return
                position, orientation = p.getBasePositionAndOrientation(
                    robot.robot_id,
                    physicsClientId=client_id,
                )
                yaw = p.getEulerFromQuaternion(orientation)[2]
                command = controller.update(
                    timestamp_ns=next_command_ns,
                    x=position[0],
                    y=position[1],
                    yaw=yaw,
                )
                assert command is not None
                robot.command_wheel_speeds(command.drive_wheel_speeds, dt=TIME_STEP)
                next_command_ns += 10_000_000

            golf_drive.advance_golf_physics_step(
                client_id=client_id,
                obstacle_manager=obstacle_manager,
                dt=TIME_STEP,
                apply_command=apply_due_command,
            )

        end_position = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]
        end_joint_positions = tuple(
            p.getJointState(robot.robot_id, joint, physicsClientId=client_id)[0]
            for joint in robot.drive_wheel_joint_indices
        )
        moving_after = tuple(
            snapshot.path.progress
            for snapshot in obstacle_manager.snapshot()
            if snapshot.path is not None
        )
        committed_obstacle_ids = {
            snapshot.physics_body_id
            for snapshot in obstacle_manager.snapshot()
            if snapshot.physics_body_id is not None
        }

        assert math.dist(start_position[:2], end_position[:2]) > 0.03
        assert any(
            abs(end - start) > 0.1
            for start, end in zip(start_joint_positions, end_joint_positions, strict=True)
        )
        assert moving_after != moving_before
        assert robot_pose_resets == []
        assert golf_drive.obstacle_contact_body_ids(
            client_id,
            robot.robot_id,
            committed_obstacle_ids,
        ) == ()
    finally:
        p.disconnect(client_id)


def test_real_df_mid_keeps_drive_contact_at_canonical_golf_bridge_pose() -> None:
    config = ExperimentConfig(
        mode="direct",
        duration_sec=2.0,
        time_step=TIME_STEP,
        robot_model="df_mid",
        terrain_model="golf_heightfield",
        golf_seed=41,
        golf_relief="medium",
        interface_enabled=False,
        dashboard_enabled=False,
    )
    document = load_scene(PROJECT_ROOT / "configs/mid360_golf_mapping.yaml")
    client_id = p.connect(p.DIRECT)
    assert client_id >= 0
    try:
        world, obstacle_manager = build_world_from_scene_document(client_id, config, document)
        robot = world.active_robot.robot
        # canonical 第 7 段曾在此处由前后支撑球架空两只中置驱动轮。
        start_position = (8.219030996444147, 0.05955211555175353, 0.21787083740791988)
        start_orientation = (
            -0.002405362148360623,
            0.008033430588021782,
            0.07973806812913584,
            0.996780577016304,
        )
        p.resetBasePositionAndOrientation(
            robot.robot_id,
            start_position,
            start_orientation,
            physicsClientId=client_id,
        )
        p.resetBaseVelocity(
            robot.robot_id,
            linearVelocity=(0.0, 0.0, 0.0),
            angularVelocity=(0.0, 0.0, 0.0),
            physicsClientId=client_id,
        )

        left_normal_forces: list[float] = []
        right_normal_forces: list[float] = []
        for _ in range(480):
            golf_drive.advance_golf_physics_step(
                client_id=client_id,
                obstacle_manager=obstacle_manager,
                dt=TIME_STEP,
                apply_command=lambda: robot.command_wheel_speeds(
                    (0.0, 6.0),
                    dt=TIME_STEP,
                ),
            )
            contacts = p.getContactPoints(
                bodyA=robot.robot_id,
                physicsClientId=client_id,
            )
            left_normal_forces.append(
                sum(
                    float(contact[9])
                    for contact in contacts
                    if contact[3] in robot.left_contact_links
                )
            )
            right_normal_forces.append(
                sum(
                    float(contact[9])
                    for contact in contacts
                    if contact[3] in robot.right_contact_links
                )
            )

        end_position = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]
        linear_velocity = p.getBaseVelocity(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]

        assert sum(left_normal_forces[-120:]) / 120.0 > 5.0
        assert sum(right_normal_forces[-120:]) / 120.0 > 5.0
        assert math.dist(start_position[:2], end_position[:2]) > 0.10
        assert math.hypot(*linear_velocity[:2]) > 0.05
    finally:
        p.disconnect(client_id)


def test_real_df_mid_reaches_final_route_progress_by_frozen_deadline() -> None:
    """固定物理时间表必须让真实车轮在停车截止前跑完最后一条扫描带。"""
    config = ExperimentConfig(
        mode="direct",
        duration_sec=1.0,
        time_step=TIME_STEP,
        robot_model="df_mid",
        terrain_model="golf_heightfield",
        golf_seed=41,
        golf_relief="medium",
        interface_enabled=False,
        dashboard_enabled=False,
    )
    document = load_scene(PROJECT_ROOT / "configs/mid360_golf_mapping.yaml")
    client_id = p.connect(p.DIRECT)
    assert client_id >= 0
    try:
        world, obstacle_manager = build_world_from_scene_document(
            client_id,
            config,
            document,
        )
        robot = world.active_robot.robot
        assert world.scene.bounds is not None
        route = golf_drive.build_canonical_golf_route(
            world.scene.bounds,
            spawn_xy=(world.scene.spawn_position[0], world.scene.spawn_position[1]),
        )
        controller = golf_drive.GolfRouteController(route)
        deadline_s = math.ceil(route.duration_s * 10.0 - 1e-12) / 10.0
        physics_steps = round(deadline_s / TIME_STEP)
        next_command_ns = 0

        for physics_step in range(physics_steps):
            physics_time_ns = round(physics_step * 1_000_000_000 / 240)

            def apply_due_command() -> None:
                nonlocal next_command_ns
                if physics_time_ns < next_command_ns:
                    return
                position, orientation = p.getBasePositionAndOrientation(
                    robot.robot_id,
                    physicsClientId=client_id,
                )
                command = controller.update(
                    timestamp_ns=next_command_ns,
                    x=position[0],
                    y=position[1],
                    yaw=p.getEulerFromQuaternion(orientation)[2],
                )
                assert command is not None
                robot.command_wheel_speeds(command.drive_wheel_speeds, dt=TIME_STEP)
                next_command_ns += 10_000_000

            golf_drive.advance_golf_physics_step(
                client_id=client_id,
                obstacle_manager=obstacle_manager,
                dt=TIME_STEP,
                apply_command=apply_due_command,
            )

        position = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]
        projection = route.project(position[0], position[1])

        assert projection.segment_index == len(route.segments) - 1
        assert route.length - projection.route_distance_m <= 0.75
    finally:
        p.disconnect(client_id)
