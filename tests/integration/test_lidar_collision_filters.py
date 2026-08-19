# 雷达碰撞过滤集成回归：锁定可见位数值，并证明它不改变车辆实体接触和最终位姿。
from __future__ import annotations

from dataclasses import dataclass

import pybullet as p
import pytest

import slope_sim.obstacles as obstacle_module
import slope_sim.scene as scene_module
from slope_sim.model_registry import get_robot_model
from slope_sim.obstacles import ObstacleGeometry, ObstacleSpec, create_box_obstacle
from slope_sim.robot import create_robot


TIME_STEP = 1.0 / 240.0
BASELINE_TERRAIN_GROUP = 0x2 | 0x8
BASELINE_OBSTACLE_GROUP = 0x2
PHYSICAL_MASK = 0x3


@dataclass(frozen=True)
class CollisionRun:
    """一次确定性物理运行的累计接触数和最终完整 base 位姿。"""

    vehicle_terrain_contacts: int
    vehicle_obstacle_contacts: int
    final_pose: tuple[float, ...]


@dataclass(frozen=True)
class CollisionFilterGateResult:
    """旧碰撞组基线与增加雷达可见位后的成对结果。"""

    baseline: CollisionRun
    visible: CollisionRun


def test_lidar_visible_collision_constants_keep_original_physical_masks():
    assert scene_module.LIDAR_VISIBLE_GROUP == 0x10
    assert scene_module.STATIC_COLLISION_GROUP == 0x2
    assert scene_module.TERRAIN_FILTER_GROUP == 0x8
    assert scene_module.TERRAIN_COLLISION_GROUP == 0x2 | 0x8 | 0x10
    assert scene_module.TERRAIN_COLLISION_MASK == PHYSICAL_MASK
    assert obstacle_module.OBSTACLE_COLLISION_GROUP == 0x2 | 0x10
    assert obstacle_module.OBSTACLE_COLLISION_MASK == PHYSICAL_MASK


@pytest.mark.parametrize("terrain_model", ("flat", "slope", "golf_heightfield"))
def test_every_terrain_body_adds_only_lidar_group_and_keeps_mask(monkeypatch, terrain_model):
    client_id = p.connect(p.DIRECT)
    calls: list[tuple[int, int, int, int]] = []
    original = scene_module.p.setCollisionFilterGroupMask

    def capture(body_id, link_index, group, mask, *, physicsClientId):
        calls.append((int(body_id), int(link_index), int(group), int(mask)))
        return original(body_id, link_index, group, mask, physicsClientId=physicsClientId)

    monkeypatch.setattr(scene_module.p, "setCollisionFilterGroupMask", capture)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=8.0,
            time_step=TIME_STEP,
            terrain_model=terrain_model,
            golf_seed=7,
        )
        expected_links = {
            (body_id, link_index)
            for body_id in scene.body_ids
            for link_index in range(-1, p.getNumJoints(body_id, physicsClientId=client_id))
        }

        assert {(body_id, link_index) for body_id, link_index, _, _ in calls} == expected_links
        assert all(
            (group, mask)
            == (scene_module.TERRAIN_COLLISION_GROUP, scene_module.TERRAIN_COLLISION_MASK)
            for _, _, group, mask in calls
        )
    finally:
        p.disconnect(client_id)


def test_formal_obstacles_are_lidar_visible_while_temporary_planning_body_stays_disabled(
    monkeypatch,
):
    client_id = p.connect(p.DIRECT)
    calls: list[tuple[int, int, int, int]] = []
    original = obstacle_module.p.setCollisionFilterGroupMask

    def capture(body_id, link_index, group, mask, *, physicsClientId):
        calls.append((int(body_id), int(link_index), int(group), int(mask)))
        return original(body_id, link_index, group, mask, physicsClientId=physicsClientId)

    monkeypatch.setattr(obstacle_module.p, "setCollisionFilterGroupMask", capture)
    try:
        public_body_id = create_box_obstacle(
            client_id,
            half_extents=(0.20, 0.25, 0.30),
            position=(-1.0, 0.0, 0.30),
        )
        spec = ObstacleSpec(
            logical_id=1,
            mode="static",
            geometry=ObstacleGeometry("box", (0.20, 0.25, 0.30)),
            position=(0.0, 0.0, 0.30),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )
        formal_body_id = obstacle_module._create_obstacle_body(client_id, spec, temporary=False)
        temporary_body_id = obstacle_module._create_obstacle_body(client_id, spec, temporary=True)

        by_body = {body_id: (link_index, group, mask) for body_id, link_index, group, mask in calls}
        assert by_body[public_body_id] == (
            -1,
            obstacle_module.OBSTACLE_COLLISION_GROUP,
            obstacle_module.OBSTACLE_COLLISION_MASK,
        )
        assert by_body[formal_body_id] == (
            -1,
            obstacle_module.OBSTACLE_COLLISION_GROUP,
            obstacle_module.OBSTACLE_COLLISION_MASK,
        )
        assert by_body[temporary_body_id] == (-1, 0, 0)
        assert p.getDynamicsInfo(public_body_id, -1, physicsClientId=client_id)[0] == 0.0
        assert p.getDynamicsInfo(formal_body_id, -1, physicsClientId=client_id)[0] == 0.0
        assert p.getCollisionShapeData(temporary_body_id, -1, physicsClientId=client_id) == ()
    finally:
        p.disconnect(client_id)


def _run_collision_case(*, lidar_visible: bool) -> CollisionRun:
    """从同一正式初态运行碰撞，唯一变量是地形和障碍物 group 的 0x10 位。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=TIME_STEP,
            terrain_model="flat",
        )
        spec = get_robot_model("df_back")
        robot = create_robot(
            client_id,
            "df_back",
            start_x=-1.25,
            start_y=0.0,
            base_height=scene.spawn_position[2] + spec.base_height,
            start_orientation=scene.spawn_orientation,
        )
        robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
        for _ in range(120):
            robot.command_twist(0.0, 0.0, dt=TIME_STEP)
            p.stepSimulation(physicsClientId=client_id)

        obstacle_id = create_box_obstacle(
            client_id,
            half_extents=(0.12, 0.65, 0.35),
            position=(0.0, 0.0, 0.35),
        )
        terrain_group = (
            scene_module.TERRAIN_COLLISION_GROUP
            if lidar_visible
            else BASELINE_TERRAIN_GROUP
        )
        obstacle_group = (
            obstacle_module.OBSTACLE_COLLISION_GROUP
            if lidar_visible
            else BASELINE_OBSTACLE_GROUP
        )
        for terrain_body_id in scene.body_ids:
            p.setCollisionFilterGroupMask(
                terrain_body_id,
                -1,
                terrain_group,
                PHYSICAL_MASK,
                physicsClientId=client_id,
            )
        p.setCollisionFilterGroupMask(
            obstacle_id,
            -1,
            obstacle_group,
            PHYSICAL_MASK,
            physicsClientId=client_id,
        )

        terrain_contacts = 0
        obstacle_contacts = 0
        for _ in range(720):
            robot.command_twist(0.6, 0.0, dt=TIME_STEP)
            p.stepSimulation(physicsClientId=client_id)
            terrain_contacts += sum(
                len(
                    p.getContactPoints(
                        bodyA=robot.robot_id,
                        bodyB=terrain_body_id,
                        physicsClientId=client_id,
                    )
                )
                for terrain_body_id in scene.body_ids
            )
            obstacle_contacts += len(
                p.getContactPoints(
                    bodyA=robot.robot_id,
                    bodyB=obstacle_id,
                    physicsClientId=client_id,
                )
            )
        position, orientation = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )
        return CollisionRun(
            terrain_contacts,
            obstacle_contacts,
            tuple(float(value) for value in (*position, *orientation)),
        )
    finally:
        p.disconnect(client_id)


def run_lidar_collision_filter_gate() -> CollisionFilterGateResult:
    """成对运行旧 group 和新增可见位，避免用单次无接触场景作空洞比较。"""
    return CollisionFilterGateResult(
        baseline=_run_collision_case(lidar_visible=False),
        visible=_run_collision_case(lidar_visible=True),
    )


def test_lidar_visibility_bit_does_not_change_physical_contacts_or_final_pose():
    result = run_lidar_collision_filter_gate()

    assert result.baseline.vehicle_terrain_contacts > 0
    assert result.baseline.vehicle_obstacle_contacts > 0
    assert result.visible.vehicle_terrain_contacts == result.baseline.vehicle_terrain_contacts
    assert result.visible.vehicle_obstacle_contacts == result.baseline.vehicle_obstacle_contacts
    assert result.visible.final_pose == pytest.approx(result.baseline.final_pose, abs=1e-6)
