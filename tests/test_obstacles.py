# 阶段二障碍物门禁测试：用正式车辆验证质量零箱体的创建、运动学碰撞和阻挡效果。
from __future__ import annotations

import math
import random
from dataclasses import FrozenInstanceError, replace
from types import SimpleNamespace

import pybullet as p
import pytest

import slope_sim.obstacles as obstacle_module
from slope_sim.model_registry import get_robot_model
from slope_sim.obstacles import create_box_obstacle, update_kinematic_obstacle
from slope_sim.robot import create_robot
from slope_sim.scene import TerrainBounds, create_slope_scene, probe_terrain
from slope_sim.telemetry import TerrainProbe


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


def _manager_with_static_obstacles(
    client_id: int,
    *,
    count: int,
    soft_budget_seconds: float = 0.002,
):
    """创建带正式静态 body 的管理器，供删除一致性回归复用。"""
    scene = create_slope_scene(
        client_id,
        slope_deg=0.0,
        time_step=TIME_STEP,
        terrain_model="flat",
    )
    manager = obstacle_module.ObstacleManager(
        client_id,
        obstacle_module.ObstacleGenerationSettings(
            bounds=scene.bounds or TerrainBounds(-4.0, 4.0, -3.0, 3.0),
            spawn_position=scene.spawn_position,
            spawn_protection_radius=0.4,
        ),
        terrain_body_ids=scene.body_ids,
        soft_budget_seconds=soft_budget_seconds,
    )
    snapshots = tuple(
        obstacle_module.ObstacleSnapshot(
            logical_id=index + 1,
            body_id=None,
            mode="static",
            shape="box",
            position=(-2.0 + index * 0.7, -1.0, 0.3),
            orientation=(0.0, 0.0, 0.0, 1.0),
            geometry=obstacle_module.ObstacleGeometry("box", (0.2, 0.2, 0.3)),
        )
        for index in range(count)
    )
    assert manager.restore(snapshots).succeeded is True
    return manager


def _assert_finite_robot_state(client_id: int, robot_id: int) -> tuple[float, float]:
    """检查碰撞帧位姿与速度均有限，并返回线、角速度模长。"""
    position, orientation = p.getBasePositionAndOrientation(robot_id, physicsClientId=client_id)
    linear_velocity, angular_velocity = p.getBaseVelocity(robot_id, physicsClientId=client_id)
    values = tuple(float(value) for value in (*position, *orientation, *linear_velocity, *angular_velocity))
    assert all(math.isfinite(value) for value in values)
    return math.sqrt(sum(float(value) ** 2 for value in linear_velocity)), math.sqrt(
        sum(float(value) ** 2 for value in angular_velocity)
    )


@pytest.mark.parametrize("failure_mode", ("raises", "remains"))
def test_delete_keeps_record_until_physical_body_is_confirmed_absent(
    monkeypatch,
    failure_mode,
):
    """removeBody 抛错或假成功时，逻辑记录必须继续指向仍存在的活动 body。"""
    client_id = p.connect(p.DIRECT)
    try:
        manager = _manager_with_static_obstacles(client_id, count=1)
        before = manager.snapshot()
        body_id = before[0].physics_body_id
        assert body_id is not None
        original_remove = obstacle_module.p.removeBody

        def fail_or_leave_body(candidate_body_id: int, **kwargs) -> None:
            if candidate_body_id == body_id:
                if failure_mode == "raises":
                    raise p.error("injected committed body removal failure")
                return
            original_remove(candidate_body_id, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "removeBody", fail_or_leave_body)

        result = manager.delete(before[0].logical_id)

        assert result.done is True
        assert result.succeeded is False
        assert result.deleted_count == 0
        assert manager.snapshot() == before
        assert body_id in _body_ids(client_id)
        expected_reason = "injected committed body removal failure" if failure_mode == "raises" else "remained"
        assert expected_reason in result.message
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("failure_mode", ("raises", "remains"))
def test_cross_frame_clear_stops_at_unconfirmed_body_and_preserves_fifo_tail(
    monkeypatch,
    failure_mode,
):
    """清空部分成功后遇到失败，应保留失败项和尚未处理的 FIFO 记录。"""
    client_id = p.connect(p.DIRECT)
    try:
        manager = _manager_with_static_obstacles(
            client_id,
            count=3,
            soft_budget_seconds=0.0,
        )
        before = manager.snapshot()
        body_ids = tuple(snapshot.physics_body_id for snapshot in before)
        assert all(body_id is not None for body_id in body_ids)
        failed_body_id = body_ids[1]
        original_remove = obstacle_module.p.removeBody

        def fail_or_leave_body(candidate_body_id: int, **kwargs) -> None:
            if candidate_body_id == failed_body_id:
                if failure_mode == "raises":
                    raise p.error("injected clear removal failure")
                return
            original_remove(candidate_body_id, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "removeBody", fail_or_leave_body)
        manager.begin_clear()

        first_slice = manager.advance_pending_operation()
        failed_slice = manager.advance_pending_operation()

        assert first_slice.done is False
        assert first_slice.deleted_count == 1
        assert failed_slice.done is True
        assert failed_slice.succeeded is False
        assert failed_slice.deleted_count == 1
        assert [snapshot.logical_id for snapshot in manager.snapshot()] == [2, 3]
        remaining_body_ids = _body_ids(client_id)
        assert body_ids[0] not in remaining_body_ids
        assert body_ids[1] in remaining_body_ids
        assert body_ids[2] in remaining_body_ids
    finally:
        p.disconnect(client_id)


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


@pytest.mark.parametrize("body_id", [True, 1.0, -1])
def test_update_rejects_invalid_body_id_before_touching_pybullet(body_id):
    client_id = p.connect(p.DIRECT)
    try:
        first_id = create_box_obstacle(
            client_id,
            half_extents=(0.20, 0.25, 0.30),
            position=(0.0, 0.0, 0.30),
        )
        second_id = create_box_obstacle(
            client_id,
            half_extents=(0.20, 0.25, 0.30),
            position=(1.0, 0.0, 0.30),
        )
        before_position, _ = p.getBasePositionAndOrientation(second_id, physicsClientId=client_id)

        with pytest.raises(ValueError, match="body_id"):
            update_kinematic_obstacle(
                client_id,
                body_id,
                position=(2.0, 0.0, 0.30),
                linear_velocity=(0.0, 0.0, 0.0),
            )

        after_position, _ = p.getBasePositionAndOrientation(second_id, physicsClientId=client_id)
        assert first_id == 0
        assert second_id == 1
        assert after_position == pytest.approx(before_position)
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


def test_obstacle_spec_quaternion_normalization_is_idempotent():
    first = obstacle_module.ObstacleSpec(
        logical_id=1,
        mode="static",
        geometry=obstacle_module.ObstacleGeometry("box", (0.2, 0.2, 0.3)),
        position=(0.0, 0.0, 0.3),
        orientation=(1.0, 2.0, 3.0, 4.0),
    )

    second = replace(first, orientation=first.orientation)

    assert second.orientation == first.orientation


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


def _flat_terrain_sampler(x: float, y: float) -> TerrainProbe:
    """规划器单元测试只注入轻量地表回调，不依赖 PyBullet GUI 或真实场景。"""
    return TerrainProbe(
        terrain_probe_valid=True,
        local_ground_height=0.05 * x - 0.03 * y,
        local_terrain_normal_x=0.0,
        local_terrain_normal_y=0.0,
        local_terrain_normal_z=1.0,
    )


def _domain_settings(**overrides):
    """创建可复用的领域规划设置，默认尺寸固定便于断言空间边界。"""
    values = {
        "bounds": TerrainBounds(-4.0, 4.0, -3.0, 3.0),
        "spawn_position": (0.0, 0.0, 0.0),
        "spawn_protection_radius": 0.65,
        "vehicle_aabb": ((-0.45, -0.35, -0.10), (0.45, 0.35, 0.65)),
        "minimum_clearance": 0.08,
        "half_extent_ranges": ((0.18, 0.18), (0.22, 0.22), (0.30, 0.30)),
        "moving_path_length_range": (0.85, 1.10),
        "max_candidate_attempts": 600,
    }
    values.update(overrides)
    return obstacle_module.ObstacleGenerationSettings(**values)


def _existing_static(logical_id: int, x: float, y: float, radius: float = 0.28):
    geometry = obstacle_module.ObstacleGeometry(shape="box", half_extents=(radius, radius, 0.30))
    return obstacle_module.ObstacleSpec(
        logical_id=logical_id,
        mode="static",
        geometry=geometry,
        position=(x, y, 0.30),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )


def _existing_moving(logical_id: int, start_xy: tuple[float, float], end_xy: tuple[float, float]):
    geometry = obstacle_module.ObstacleGeometry(shape="box", half_extents=(0.20, 0.20, 0.30))
    path = obstacle_module.ObstaclePath(start_xy=start_xy, end_xy=end_xy, speed=0.35)
    return obstacle_module.ObstacleSpec(
        logical_id=logical_id,
        mode="moving",
        geometry=geometry,
        position=(start_xy[0], start_xy[1], 0.30),
        orientation=(0.0, 0.0, 0.0, 1.0),
        path=path,
    )


@pytest.mark.parametrize(
    "factory, message",
    [
        (lambda: obstacle_module.ObstacleGenerationRequest(mode="bad", count=1, seed=1), "mode"),
        (lambda: obstacle_module.ObstacleGenerationRequest(mode="static", count=0, seed=1), "count"),
        (lambda: obstacle_module.ObstacleGenerationRequest(mode="static", count=51, seed=1), "count"),
        (lambda: obstacle_module.ObstacleGenerationRequest(mode="static", count=1, seed=1, shape="mesh"), "shape"),
        (lambda: obstacle_module.ObstacleGenerationRequest(mode="mixed", count=2, seed=1, moving_ratio=-0.1), "ratio"),
        (lambda: obstacle_module.ObstacleGenerationRequest(mode="mixed", count=2, seed=1, moving_ratio=1.1), "ratio"),
        (lambda: obstacle_module.ObstacleGenerationRequest(mode="moving", count=1, seed=1, moving_speed=0.0), "speed"),
        (lambda: _domain_settings(max_scene_obstacles=101), "scene.*100"),
    ],
)
def test_obstacle_generation_request_rejects_invalid_parameters(factory, message):
    with pytest.raises(ValueError, match=message):
        factory()


@pytest.mark.parametrize(
    "factory, message",
    [
        (lambda: obstacle_module.ObstacleGenerationRequest(mode="static", count=2.9, seed=1), "count.*integer"),
        (lambda: obstacle_module.ObstacleGenerationRequest(mode="static", count=True, seed=1), "count.*integer"),
        (lambda: obstacle_module.ObstacleGenerationRequest(mode="static", count=2, seed=1.5), "seed.*integer"),
        (lambda: _domain_settings(max_candidate_attempts=4.2), "max_candidate_attempts.*integer"),
        (lambda: _domain_settings(max_batch_obstacles=True), "max_batch_obstacles.*integer"),
        (
            lambda: obstacle_module.ObstaclePath(
                start_xy=(0.0, 0.0),
                end_xy=(1.0, 0.0),
                speed=0.2,
                direction=0.5,
            ),
            "direction.*integer",
        ),
        (lambda: _existing_static(1.9, 2.0, 2.0), "logical_id.*integer"),
        (
            lambda: obstacle_module.ObstacleSnapshot(
                logical_id=1,
                body_id=2.2,
                mode="static",
                shape="box",
                position=(1.0, 2.0, 0.3),
                orientation=(0.0, 0.0, 0.0, 1.0),
            ),
            "body_id.*integer",
        ),
    ],
)
def test_generation_domain_objects_reject_non_integral_identifiers(factory, message):
    with pytest.raises(ValueError, match=message):
        factory()


@pytest.mark.parametrize("nested_name", ("geometry", "path"))
def test_obstacle_spec_rejects_foreign_nested_domain_values(nested_name):
    values = {
        "logical_id": 1,
        "mode": "static",
        "geometry": obstacle_module.ObstacleGeometry("box", (0.2, 0.2, 0.3)),
        "position": (0.0, 0.0, 0.3),
        "orientation": (0.0, 0.0, 0.0, 1.0),
        "path": None,
    }
    if nested_name == "geometry":
        values["geometry"] = SimpleNamespace(
            shape="mesh",
            half_extents=(0.2, 0.2, 0.3),
        )
    else:
        values["mode"] = "moving"
        values["path"] = SimpleNamespace(
            start_xy=(0.0, 0.0),
            end_xy=(1.0, 0.0),
            speed=0.2,
            progress=2.0,
            direction=0,
        )

    with pytest.raises(ValueError, match=f"{nested_name}.*Obstacle{nested_name.title()}"):
        obstacle_module.ObstacleSpec(**values)


@pytest.mark.parametrize(
    ("nested_name", "field_name", "invalid_value", "message"),
    (
        ("geometry", "shape", "mesh", "geometry.*shape"),
        ("geometry", "half_extents", (0.0, 0.2, 0.3), "geometry.*half_extents"),
        ("geometry", "half_extents", (math.inf, 0.2, 0.3), "geometry.*half_extents"),
        ("geometry", "half_extents", (True, 0.2, 0.3), "geometry.*half_extents"),
        ("path", "start_xy", (math.nan, 0.0), "path.*start_xy"),
        ("path", "end_xy", (0.0, 0.0), "path.*endpoints"),
        ("path", "speed", 0.0, "path.*speed"),
        ("path", "speed", True, "path.*speed"),
        ("path", "progress", 2.0, "path.*progress"),
        ("path", "progress", False, "path.*progress"),
        ("path", "direction", 0, "path.*direction"),
    ),
)
def test_obstacle_spec_revalidates_forged_nested_domain_values(
    nested_name,
    field_name,
    invalid_value,
    message,
):
    geometry = obstacle_module.ObstacleGeometry("box", (0.2, 0.2, 0.3))
    path = obstacle_module.ObstaclePath((0.0, 0.0), (1.0, 0.0), 0.2)
    nested = geometry if nested_name == "geometry" else path
    object.__setattr__(nested, field_name, invalid_value)

    with pytest.raises(ValueError, match=message):
        obstacle_module.ObstacleSpec(
            logical_id=1,
            mode="static" if nested_name == "geometry" else "moving",
            geometry=geometry,
            position=(0.0, 0.0, 0.3),
            orientation=(0.0, 0.0, 0.0, 1.0),
            path=None if nested_name == "geometry" else path,
        )


def test_non_box_geometry_uses_canonical_radius_dimensions():
    sphere = obstacle_module.ObstacleGeometry(shape="sphere", half_extents=(0.20, 0.35, 0.30))
    cylinder = obstacle_module.ObstacleGeometry(shape="cylinder", half_extents=(0.20, 0.35, 0.60))

    assert sphere.half_extents == pytest.approx((0.35, 0.35, 0.35))
    assert sphere.bounding_radius == pytest.approx(0.35)
    assert cylinder.half_extents == pytest.approx((0.35, 0.35, 0.60))
    assert cylinder.bounding_radius == pytest.approx(0.35)


@pytest.mark.parametrize(
    ("count", "ratio", "expected"),
    [
        (5, 0.30, (3, 2)),
        (10, 0.30, (7, 3)),
        (2, 0.00, (1, 1)),
        (2, 0.01, (1, 1)),
        (2, 0.99, (1, 1)),
        (2, 1.00, (1, 1)),
        (1, 0.20, (1, 0)),
        (1, 0.80, (0, 1)),
    ],
)
def test_mixed_mode_uses_half_up_rounding_and_clamps_when_possible(count, ratio, expected):
    assert obstacle_module.split_mixed_obstacle_counts(count, ratio) == expected


def test_generation_objects_are_immutable_and_body_id_is_not_logical_id():
    request = obstacle_module.ObstacleGenerationRequest(mode="static", count=1, seed=12)
    snapshot = obstacle_module.ObstacleSnapshot(
        logical_id=7,
        body_id=42,
        mode="static",
        shape="box",
        position=(1.0, 2.0, 0.3),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )

    with pytest.raises(FrozenInstanceError):
        request.count = 2
    assert snapshot.logical_id == 7
    assert snapshot.body_id == 42


def test_generation_rejects_scene_total_above_limit_before_sampling():
    settings = _domain_settings(max_scene_obstacles=100)
    existing = tuple(_existing_static(index + 1, -3.5 + index * 0.01, -2.5) for index in range(99))
    request = obstacle_module.ObstacleGenerationRequest(mode="static", count=2, seed=3)
    calls: list[tuple[float, float]] = []

    def sampler(x: float, y: float) -> TerrainProbe:
        calls.append((x, y))
        return _flat_terrain_sampler(x, y)

    with pytest.raises(ValueError, match="scene.*100"):
        obstacle_module.plan_obstacle_batch(settings, request, sampler, existing_specs=existing)
    assert calls == []


def test_same_inputs_and_seed_generate_identical_batch_without_global_rng_mutation():
    settings = _domain_settings()
    existing = (_existing_static(3, 2.6, -1.8), _existing_moving(8, (-2.8, 1.8), (-2.0, 1.8)))
    request = obstacle_module.ObstacleGenerationRequest(
        mode="mixed",
        count=6,
        seed=1234,
        moving_ratio=0.30,
        moving_speed=0.40,
    )
    random.seed(20260720)
    before_state = random.getstate()

    first = obstacle_module.plan_obstacle_batch(settings, request, _flat_terrain_sampler, existing_specs=existing)
    after_state = random.getstate()
    second = obstacle_module.plan_obstacle_batch(settings, request, _flat_terrain_sampler, existing_specs=existing)

    random.setstate(before_state)
    expected_next = random.random()
    random.setstate(after_state)
    actual_next = random.random()
    assert actual_next == expected_next
    assert first == second
    assert [spec.logical_id for spec in first] == list(range(9, 15))
    assert [spec.mode for spec in first].count("static") == 4
    assert [spec.mode for spec in first].count("moving") == 2


def test_planner_keeps_bounding_radius_inside_bounds_and_away_from_occupied_space():
    settings = _domain_settings()
    existing = (_existing_static(1, 2.4, 1.8), _existing_moving(2, (-3.0, -2.3), (-2.0, -2.3)))
    request = obstacle_module.ObstacleGenerationRequest(mode="mixed", count=8, seed=91, moving_ratio=0.5)

    batch = obstacle_module.plan_obstacle_batch(settings, request, _flat_terrain_sampler, existing_specs=existing)
    all_specs = (*existing, *batch)

    for index, spec in enumerate(batch):
        radius = spec.geometry.bounding_radius
        x, y, z = spec.position
        assert settings.bounds.min_x + radius <= x <= settings.bounds.max_x - radius
        assert settings.bounds.min_y + radius <= y <= settings.bounds.max_y - radius
        assert math.hypot(x - settings.spawn_position[0], y - settings.spawn_position[1]) >= (
            radius + settings.spawn_protection_radius + settings.minimum_clearance
        )
        assert obstacle_module.circle_aabb_distance_2d((x, y), settings.vehicle_aabb) >= (
            radius + settings.minimum_clearance
        )
        assert z == pytest.approx(_flat_terrain_sampler(x, y).local_ground_height + spec.geometry.half_extents[2])

        for other in all_specs[: len(existing) + index]:
            if spec.path is None and other.path is None:
                distance = math.hypot(x - other.position[0], y - other.position[1])
            elif spec.path is not None and other.path is not None:
                distance = obstacle_module.segment_distance_2d(
                    spec.path.start_xy,
                    spec.path.end_xy,
                    other.path.start_xy,
                    other.path.end_xy,
                )
            else:
                moving = spec if spec.path is not None else other
                static = other if spec.path is not None else spec
                distance = obstacle_module.point_segment_distance_2d(
                    (static.position[0], static.position[1]),
                    moving.path.start_xy,
                    moving.path.end_xy,
                )
            assert distance >= spec.geometry.bounding_radius + other.geometry.bounding_radius + settings.minimum_clearance


def test_moving_paths_stay_inside_bounds_and_swept_corridors_do_not_intersect():
    settings = _domain_settings(bounds=TerrainBounds(-5.0, 5.0, -4.0, 4.0), moving_path_length_range=(1.4, 1.8))
    request = obstacle_module.ObstacleGenerationRequest(mode="moving", count=5, seed=2048, moving_speed=0.55)

    batch = obstacle_module.plan_obstacle_batch(settings, request, _flat_terrain_sampler)

    for index, spec in enumerate(batch):
        assert spec.path is not None
        for point in (spec.path.start_xy, spec.path.end_xy):
            assert settings.bounds.min_x + spec.geometry.bounding_radius <= point[0] <= (
                settings.bounds.max_x - spec.geometry.bounding_radius
            )
            assert settings.bounds.min_y + spec.geometry.bounding_radius <= point[1] <= (
                settings.bounds.max_y - spec.geometry.bounding_radius
            )
        for other in batch[:index]:
            assert other.path is not None
            distance = obstacle_module.segment_distance_2d(
                spec.path.start_xy,
                spec.path.end_xy,
                other.path.start_xy,
                other.path.end_xy,
            )
            assert distance >= spec.geometry.bounding_radius + other.geometry.bounding_radius + settings.minimum_clearance


def test_candidate_attempt_exhaustion_is_atomic_and_reports_clear_error():
    settings = _domain_settings(
        bounds=TerrainBounds(-0.30, 0.30, -0.30, 0.30),
        spawn_protection_radius=0.0,
        vehicle_aabb=None,
        half_extent_ranges=((0.28, 0.28), (0.28, 0.28), (0.30, 0.30)),
        max_candidate_attempts=4,
    )
    request = obstacle_module.ObstacleGenerationRequest(mode="static", count=2, seed=7)

    with pytest.raises(obstacle_module.ObstaclePlanningError, match="Unable to plan complete obstacle batch.*2"):
        obstacle_module.plan_obstacle_batch(settings, request, _flat_terrain_sampler)


def test_ping_pong_progress_consumes_remaining_displacement_without_leaving_segment():
    progress, direction = obstacle_module.advance_ping_pong_progress(
        progress=0.80,
        direction=1,
        segment_length=1.0,
        speed=1.0,
        dt=0.70,
    )
    assert progress == pytest.approx(0.50)
    assert direction == -1

    progress, direction = obstacle_module.advance_ping_pong_progress(
        progress=0.10,
        direction=-1,
        segment_length=1.0,
        speed=1.0,
        dt=4.75,
    )
    assert 0.0 <= progress <= 1.0
    assert direction in {-1, 1}
    assert progress == pytest.approx(0.65)
    assert direction == 1

    progress, direction = obstacle_module.advance_ping_pong_progress(
        progress=0.50,
        direction=1,
        segment_length=1.0,
        speed=1.0,
        dt=0.50,
    )
    assert progress == pytest.approx(1.0)
    assert direction == -1

    progress, direction = obstacle_module.advance_ping_pong_progress(
        progress=0.50,
        direction=-1,
        segment_length=1.0,
        speed=1.0,
        dt=0.50,
    )
    assert progress == pytest.approx(0.0)
    assert direction == 1


def test_segment_distance_helpers_cover_crossing_parallel_and_point_cases():
    assert obstacle_module.segment_distance_2d((0.0, 0.0), (2.0, 0.0), (1.0, -1.0), (1.0, 1.0)) == pytest.approx(0.0)
    assert obstacle_module.segment_distance_2d((0.0, 0.0), (2.0, 0.0), (0.0, 1.5), (2.0, 1.5)) == pytest.approx(1.5)
    assert obstacle_module.point_segment_distance_2d((1.0, 2.0), (0.0, 0.0), (2.0, 0.0)) == pytest.approx(2.0)


def test_planned_orientation_preserves_path_heading_and_sampled_terrain_normal():
    normal = (0.20, -0.10, math.sqrt(0.95))

    def tilted_sampler(x: float, y: float) -> TerrainProbe:
        return TerrainProbe(
            terrain_probe_valid=True,
            local_ground_height=0.0,
            local_terrain_normal_x=normal[0],
            local_terrain_normal_y=normal[1],
            local_terrain_normal_z=normal[2],
        )

    settings = _domain_settings(bounds=TerrainBounds(-3.0, 3.0, -3.0, 3.0))
    request = obstacle_module.ObstacleGenerationRequest(mode="moving", count=1, seed=5, moving_speed=0.4)

    (spec,) = obstacle_module.plan_obstacle_batch(settings, request, tilted_sampler)
    assert spec.path is not None
    matrix = p.getMatrixFromQuaternion(spec.orientation)
    local_x = (matrix[0], matrix[3], matrix[6])
    local_z = (matrix[2], matrix[5], matrix[8])
    path_dx = spec.path.end_xy[0] - spec.path.start_xy[0]
    path_dy = spec.path.end_xy[1] - spec.path.start_xy[1]
    path_heading = (path_dx / math.hypot(path_dx, path_dy), path_dy / math.hypot(path_dx, path_dy), 0.0)

    assert local_z == pytest.approx(normal, abs=1e-6)
    assert local_x[0] * path_heading[0] + local_x[1] * path_heading[1] > 0.97
