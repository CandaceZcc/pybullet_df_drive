# 阶段二障碍物门禁测试：用正式车辆验证质量零箱体的创建、运动学碰撞和阻挡效果。
from __future__ import annotations

import math

import pybullet as p
import pytest

import slope_sim.obstacles as obstacle_module
from slope_sim.model_registry import get_robot_model
from slope_sim.obstacles import create_box_obstacle, update_kinematic_obstacle
from slope_sim.robot import create_robot
from slope_sim.scene import create_slope_scene, probe_terrain


TIME_STEP = 1.0 / 240.0
ROBOT_MODEL = "df_back"
DRIVE_LINEAR_VELOCITY = 0.6


def _create_flat_robot(client_id: int, *, start_x: float, start_y: float = 0.0):
    """复用正式场景、车型工厂和阶段一验收摩擦参数创建测试车辆。"""
    scene = create_slope_scene(
        client_id,
        slope_deg=0.0,
        time_step=TIME_STEP,
        terrain_model="flat",
    )
    spec = get_robot_model(ROBOT_MODEL)
    robot = create_robot(
        client_id,
        ROBOT_MODEL,
        start_x=start_x,
        start_y=start_y,
        base_height=scene.spawn_position[2] + spec.base_height,
        start_orientation=scene.spawn_orientation,
    )
    robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
    for _ in range(120):
        robot.command_twist(0.0, 0.0, dt=TIME_STEP)
        p.stepSimulation(physicsClientId=client_id)
    return robot


def _body_ids(client_id: int) -> set[int]:
    return {
        p.getBodyUniqueId(index, physicsClientId=client_id)
        for index in range(p.getNumBodies(physicsClientId=client_id))
    }


def _assert_finite_robot_state(client_id: int, robot_id: int) -> tuple[float, float]:
    """检查碰撞帧位姿与速度均有限，并返回线、角速度模长。"""
    position, orientation = p.getBasePositionAndOrientation(robot_id, physicsClientId=client_id)
    linear_velocity, angular_velocity = p.getBaseVelocity(robot_id, physicsClientId=client_id)
    values = tuple(float(value) for value in (*position, *orientation, *linear_velocity, *angular_velocity))
    assert all(math.isfinite(value) for value in values)
    return math.sqrt(sum(float(value) ** 2 for value in linear_velocity)), math.sqrt(
        sum(float(value) ** 2 for value in angular_velocity)
    )


def test_create_box_obstacle_rejects_zero_norm_orientation():
    client_id = p.connect(p.DIRECT)
    try:
        before_ids = _body_ids(client_id)
        with pytest.raises(ValueError, match="orientation.*norm"):
            create_box_obstacle(
                client_id,
                half_extents=(0.20, 0.25, 0.30),
                position=(0.0, 0.0, 0.30),
                orientation=(0.0, 0.0, 0.0, 0.0),
            )
        assert _body_ids(client_id) == before_ids
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(
    "current_orientation",
    [
        (0.0, 0.0, 0.0, 0.0),
        (math.nan, 0.0, 0.0, 1.0),
    ],
)
def test_update_rejects_invalid_explicit_or_current_orientation(monkeypatch, current_orientation):
    client_id = p.connect(p.DIRECT)
    try:
        body_id = create_box_obstacle(
            client_id,
            half_extents=(0.20, 0.25, 0.30),
            position=(0.0, 0.0, 0.30),
        )
        with pytest.raises(ValueError, match="orientation.*norm"):
            update_kinematic_obstacle(
                client_id,
                body_id,
                position=(0.1, 0.0, 0.30),
                linear_velocity=(0.1, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 0.0),
            )

        monkeypatch.setattr(
            obstacle_module.p,
            "getBasePositionAndOrientation",
            lambda *_args, **_kwargs: ((0.0, 0.0, 0.30), current_orientation),
        )
        expected_message = "finite" if math.isnan(current_orientation[0]) else "norm"
        with pytest.raises(ValueError, match=f"orientation.*{expected_message}"):
            update_kinematic_obstacle(
                client_id,
                body_id,
                position=(0.1, 0.0, 0.30),
                linear_velocity=(0.1, 0.0, 0.0),
            )
    finally:
        p.disconnect(client_id)


def test_non_unit_orientations_are_normalized_before_pybullet_calls(monkeypatch):
    client_id = p.connect(p.DIRECT)
    try:
        create_orientations: list[tuple[float, ...]] = []
        original_create_multibody = obstacle_module.p.createMultiBody

        def capture_create_orientation(*args, **kwargs):
            create_orientations.append(tuple(kwargs["baseOrientation"]))
            return original_create_multibody(*args, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "createMultiBody", capture_create_orientation)
        body_id = create_box_obstacle(
            client_id,
            half_extents=(0.20, 0.25, 0.30),
            position=(0.0, 0.0, 0.30),
            orientation=(0.0, 0.0, 0.0, 2.0),
        )
        assert create_orientations == [(0.0, 0.0, 0.0, 1.0)]

        update_orientations: list[tuple[float, ...]] = []
        original_reset_pose = obstacle_module.p.resetBasePositionAndOrientation

        def capture_update_orientation(*args, **kwargs):
            update_orientations.append(tuple(args[2]))
            return original_reset_pose(*args, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "resetBasePositionAndOrientation", capture_update_orientation)
        update_kinematic_obstacle(
            client_id,
            body_id,
            position=(0.1, 0.0, 0.30),
            linear_velocity=(0.1, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 3.0),
        )
        assert update_orientations == [(0.0, 0.0, 0.0, 1.0)]
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("failure_stage", ["visual", "multibody"])
def test_create_releases_collision_shape_when_later_creation_fails(monkeypatch, failure_stage):
    client_id = p.connect(p.DIRECT)
    try:
        removed_collision_ids: list[int] = []
        original_remove_collision_shape = obstacle_module.p.removeCollisionShape

        def track_collision_removal(shape_id, *args, **kwargs):
            removed_collision_ids.append(int(shape_id))
            return original_remove_collision_shape(shape_id, *args, **kwargs)

        def fail_creation(*_args, **_kwargs):
            raise RuntimeError(f"injected {failure_stage} creation failure")

        monkeypatch.setattr(obstacle_module.p, "removeCollisionShape", track_collision_removal)
        if failure_stage == "visual":
            monkeypatch.setattr(obstacle_module.p, "createVisualShape", fail_creation)
        else:
            monkeypatch.setattr(obstacle_module.p, "createMultiBody", fail_creation)

        with pytest.raises(RuntimeError, match=f"injected {failure_stage} creation failure"):
            create_box_obstacle(
                client_id,
                half_extents=(0.20, 0.25, 0.30),
                position=(0.0, 0.0, 0.30),
            )
        assert len(removed_collision_ids) == 1
    finally:
        p.disconnect(client_id)


def test_create_box_obstacle_has_zero_mass_and_cleans_partial_body_on_failure(monkeypatch):
    client_id = p.connect(p.DIRECT)
    try:
        body_id = create_box_obstacle(
            client_id,
            half_extents=(0.20, 0.25, 0.30),
            position=(0.0, 0.0, 0.30),
        )
        assert p.getDynamicsInfo(body_id, -1, physicsClientId=client_id)[0] == 0.0

        before_ids = _body_ids(client_id)
        original_create_multibody = obstacle_module.p.createMultiBody

        def create_then_fail(*args, **kwargs):
            original_create_multibody(*args, **kwargs)
            raise RuntimeError("injected obstacle construction failure")

        monkeypatch.setattr(obstacle_module.p, "createMultiBody", create_then_fail)
        with pytest.raises(RuntimeError, match="injected obstacle construction failure"):
            create_box_obstacle(
                client_id,
                half_extents=(0.20, 0.25, 0.30),
                position=(1.0, 0.0, 0.30),
            )
        assert _body_ids(client_id) == before_ids
    finally:
        p.disconnect(client_id)


def test_moving_zero_mass_box_keeps_path_and_collision_state_bounded():
    client_id = p.connect(p.DIRECT)
    try:
        robot = _create_flat_robot(client_id, start_x=0.0)
        obstacle_id = create_box_obstacle(
            client_id,
            half_extents=(0.25, 0.25, 0.30),
            position=(0.0, -1.0, 0.30),
        )
        obstacle_speed = 0.20
        commanded_y = -1.0
        maximum_lateral_error = 0.0
        maximum_penetration = 0.0
        maximum_robot_linear_speed = 0.0
        maximum_robot_angular_speed = 0.0
        contact_frames = 0

        for _ in range(960):
            # 运动学体必须先更新位姿和路径切向速度，再执行本帧物理步进。
            update_kinematic_obstacle(
                client_id,
                obstacle_id,
                position=(0.0, commanded_y, 0.30),
                linear_velocity=(0.0, obstacle_speed, 0.0),
            )
            pre_step_velocity, _ = p.getBaseVelocity(obstacle_id, physicsClientId=client_id)
            assert pre_step_velocity == pytest.approx((0.0, obstacle_speed, 0.0), abs=1e-12)
            p.stepSimulation(physicsClientId=client_id)
            commanded_y += obstacle_speed * TIME_STEP

            obstacle_position, _ = p.getBasePositionAndOrientation(obstacle_id, physicsClientId=client_id)
            maximum_lateral_error = max(maximum_lateral_error, abs(float(obstacle_position[0])))
            assert float(obstacle_position[1]) == pytest.approx(commanded_y, abs=1e-9)

            contacts = p.getContactPoints(
                bodyA=robot.robot_id,
                bodyB=obstacle_id,
                physicsClientId=client_id,
            )
            if not contacts:
                continue
            contact_frames += 1
            maximum_penetration = max(
                maximum_penetration,
                max(max(0.0, -float(contact[8])) for contact in contacts),
            )
            linear_speed, angular_speed = _assert_finite_robot_state(client_id, robot.robot_id)
            maximum_robot_linear_speed = max(maximum_robot_linear_speed, linear_speed)
            maximum_robot_angular_speed = max(maximum_robot_angular_speed, angular_speed)

        assert contact_frames > 0
        assert maximum_lateral_error <= 1e-6
        assert maximum_penetration <= 0.03
        assert maximum_robot_linear_speed <= 3.0
        assert maximum_robot_angular_speed <= 10.0
    finally:
        p.disconnect(client_id)


def test_probe_terrain_ignores_vehicle_above_flat_ground():
    """车辆覆盖采样点时，只接受 SceneInfo 声明的地形 body。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        spec = get_robot_model(ROBOT_MODEL)
        robot = create_robot(
            client_id,
            ROBOT_MODEL,
            start_x=0.0,
            start_y=0.0,
            base_height=scene.spawn_position[2] + spec.base_height,
            start_orientation=scene.spawn_orientation,
        )
        first_hit = p.rayTest((0.0, 0.0, 2.0), (0.0, 0.0, -2.0), physicsClientId=client_id)[0]

        probe = probe_terrain(client_id, 0.0, 0.0, ray_height=2.0, terrain_body_ids=scene.body_ids)

        assert first_hit[0] == robot.robot_id
        assert probe.terrain_probe_valid is True
        assert probe.local_ground_height == pytest.approx(0.0, abs=1e-6)
        assert (
            probe.local_terrain_normal_x,
            probe.local_terrain_normal_y,
            probe.local_terrain_normal_z,
        ) == pytest.approx((0.0, 0.0, 1.0), abs=1e-6)
    finally:
        p.disconnect(client_id)


def test_probe_terrain_starting_above_moving_obstacle_skips_itself():
    """从运动障碍物上方起射时，不把障碍物顶面误当作逐帧抬升的地表。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        obstacle_id = create_box_obstacle(
            client_id,
            half_extents=(0.25, 0.25, 0.30),
            position=(0.0, 0.0, 0.30),
        )
        update_kinematic_obstacle(
            client_id,
            obstacle_id,
            position=(0.0, 0.0, 0.30),
            linear_velocity=(0.20, 0.0, 0.0),
        )
        ray_start_z = 0.61
        first_hit = p.rayTest((0.0, 0.0, ray_start_z), (0.0, 0.0, -2.0), physicsClientId=client_id)[0]

        probe = probe_terrain(
            client_id,
            0.0,
            0.0,
            ray_height=2.0,
            ray_start_z=ray_start_z,
            terrain_body_ids=scene.body_ids,
        )

        assert first_hit[0] == obstacle_id
        assert probe.terrain_probe_valid is True
        assert probe.local_ground_height == pytest.approx(0.0, abs=1e-6)
        assert probe.local_terrain_normal_z == pytest.approx(1.0, abs=1e-6)
    finally:
        p.disconnect(client_id)


def test_terrain_collision_group_preserves_default_dynamic_contacts():
    """地形专用组仍须与默认组车辆和动态箱体产生物理接触。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        spec = get_robot_model(ROBOT_MODEL)
        robot = create_robot(
            client_id,
            ROBOT_MODEL,
            start_x=-1.0,
            start_y=0.0,
            base_height=scene.spawn_position[2] + spec.base_height,
            start_orientation=scene.spawn_orientation,
        )
        collision_shape_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=(0.18, 0.18, 0.18),
            physicsClientId=client_id,
        )
        dynamic_obstacle_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=(1.0, 0.0, 0.65),
            physicsClientId=client_id,
        )
        robot_contact_max = 0
        obstacle_contact_max = 0
        for _ in range(480):
            robot.command_twist(0.0, 0.0, dt=TIME_STEP)
            p.stepSimulation(physicsClientId=client_id)
            robot_contact_max = max(
                robot_contact_max,
                len(p.getContactPoints(bodyA=robot.robot_id, bodyB=scene.body_id, physicsClientId=client_id)),
            )
            obstacle_contact_max = max(
                obstacle_contact_max,
                len(p.getContactPoints(bodyA=dynamic_obstacle_id, bodyB=scene.body_id, physicsClientId=client_id)),
            )

        assert robot_contact_max > 0
        assert obstacle_contact_max > 0
    finally:
        p.disconnect(client_id)


def _run_forward_displacement(*, with_obstacle: bool) -> tuple[float, int, float, float, float]:
    """从相同初态运行正式差速车，返回位移与碰撞稳定性实测值。"""
    client_id = p.connect(p.DIRECT)
    try:
        robot = _create_flat_robot(client_id, start_x=-1.25)
        obstacle_id = None
        if with_obstacle:
            obstacle_id = create_box_obstacle(
                client_id,
                half_extents=(0.12, 0.65, 0.35),
                position=(0.0, 0.0, 0.35),
            )
        start_position, _ = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
        contact_frames = 0
        maximum_penetration = 0.0
        maximum_linear_speed = 0.0
        maximum_angular_speed = 0.0

        for _ in range(720):
            robot.command_twist(DRIVE_LINEAR_VELOCITY, 0.0, dt=TIME_STEP)
            p.stepSimulation(physicsClientId=client_id)
            if obstacle_id is None:
                continue
            contacts = p.getContactPoints(
                bodyA=robot.robot_id,
                bodyB=obstacle_id,
                physicsClientId=client_id,
            )
            if not contacts:
                continue
            contact_frames += 1
            maximum_penetration = max(
                maximum_penetration,
                max(max(0.0, -float(contact[8])) for contact in contacts),
            )
            linear_speed, angular_speed = _assert_finite_robot_state(client_id, robot.robot_id)
            maximum_linear_speed = max(maximum_linear_speed, linear_speed)
            maximum_angular_speed = max(maximum_angular_speed, angular_speed)

        end_position, _ = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
        displacement = float(end_position[0]) - float(start_position[0])
        return displacement, contact_frames, maximum_penetration, maximum_linear_speed, maximum_angular_speed
    finally:
        p.disconnect(client_id)


def test_static_box_limits_forward_displacement_without_unstable_collision():
    baseline_displacement, _, _, _, _ = _run_forward_displacement(with_obstacle=False)
    obstacle_displacement, contact_frames, penetration, linear_speed, angular_speed = _run_forward_displacement(
        with_obstacle=True
    )

    assert baseline_displacement > 0.5
    assert contact_frames > 0
    assert penetration <= 0.03
    assert linear_speed <= 3.0
    assert angular_speed <= 10.0
    assert obstacle_displacement <= baseline_displacement * 0.50
