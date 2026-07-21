# ObstacleManager 生命周期测试：覆盖 PyBullet body 创建、跨帧事务、删除和快照恢复。
from __future__ import annotations

import math

import pybullet as p
import pytest

import slope_sim.obstacles as obstacle_module
from slope_sim.obstacles import (
    ObstacleGeometry,
    ObstacleGenerationRequest,
    ObstacleGenerationSettings,
    ObstacleManager,
    ObstaclePath,
    ObstacleSnapshot,
)
from slope_sim.scene import TerrainBounds, create_slope_scene


TIME_STEP = 1.0 / 240.0


class FakeClock:
    """测试用可控时钟，便于稳定触发 2 ms 跨帧预算。"""

    def __init__(self) -> None:
        self.current = 0.0

    def __call__(self) -> float:
        return self.current

    def advance(self, seconds: float) -> None:
        self.current += seconds


def _settings(**overrides) -> ObstacleGenerationSettings:
    values = {
        "bounds": TerrainBounds(-4.0, 4.0, -3.0, 3.0),
        "spawn_position": (0.0, 0.0, 0.0),
        "spawn_protection_radius": 0.40,
        "vehicle_aabb": None,
        "minimum_clearance": 0.05,
        "half_extent_ranges": ((0.20, 0.20), (0.30, 0.30), (0.40, 0.40)),
        "moving_path_length_range": (1.0, 1.0),
        "max_candidate_attempts": 900,
    }
    values.update(overrides)
    return ObstacleGenerationSettings(**values)


def _finish(manager: ObstacleManager):
    """推进当前事务直到完成，测试断言可聚焦最终状态。"""
    result = manager.advance_pending_operation()
    guard = 0
    while not result.done:
        guard += 1
        assert guard < 100
        result = manager.advance_pending_operation()
    return result


def _body_ids(client_id: int) -> set[int]:
    return {
        p.getBodyUniqueId(index, physicsClientId=client_id)
        for index in range(p.getNumBodies(physicsClientId=client_id))
    }


def _add_one(
    manager: ObstacleManager,
    *,
    shape: str = "box",
    mode: str = "static",
    seed: int = 11,
    moving_speed: float = 0.40,
):
    manager.begin_add(ObstacleGenerationRequest(mode=mode, count=1, seed=seed, shape=shape, moving_speed=moving_speed))
    result = _finish(manager)
    assert result.done
    assert result.succeeded
    assert result.published_count == 1
    return manager.snapshot()[-1]


def _assert_ground_attached(client_id: int, snapshot) -> None:
    aabb_min, aabb_max = p.getAABB(snapshot.physics_body_id, -1, physicsClientId=client_id)
    assert aabb_min[2] == pytest.approx(0.0, abs=0.04)
    position, _orientation = p.getBasePositionAndOrientation(snapshot.physics_body_id, physicsClientId=client_id)
    assert tuple(position) == pytest.approx(snapshot.position, abs=1e-9)
    assert aabb_max[2] - aabb_min[2] > 0.0


@pytest.mark.parametrize(
    ("shape", "expected_type"),
    [
        ("box", p.GEOM_BOX),
        ("cylinder", p.GEOM_CYLINDER),
        ("sphere", p.GEOM_SPHERE),
    ],
)
def test_manager_creates_real_shapes_with_zero_mass_and_ground_pose(shape, expected_type):
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)

        snapshot = _add_one(manager, shape=shape, seed=100 + expected_type)
        body_id = snapshot.physics_body_id
        assert body_id is not None
        assert p.getDynamicsInfo(body_id, -1, physicsClientId=client_id)[0] == 0.0
        assert p.getCollisionShapeData(body_id, -1, physicsClientId=client_id)[0][2] == expected_type
        assert p.getVisualShapeData(body_id, physicsClientId=client_id)[0][2] == expected_type
        _assert_ground_attached(client_id, snapshot)
    finally:
        p.disconnect(client_id)


def test_static_objects_do_not_move_and_moving_objects_reverse_at_endpoints():
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(
            client_id,
            _settings(bounds=scene.bounds, moving_path_length_range=(0.60, 0.60)),
            terrain_body_ids=scene.body_ids,
        )
        static_snapshot = _add_one(manager, mode="static", seed=21)
        moving_snapshot = _add_one(manager, mode="moving", seed=22, moving_speed=0.60)
        static_before = static_snapshot.position
        assert moving_snapshot.path is not None

        manager.update_moving(1.0)
        first_update = {snapshot.logical_id: snapshot for snapshot in manager.snapshot()}
        assert first_update[static_snapshot.logical_id].position == pytest.approx(static_before)
        assert first_update[moving_snapshot.logical_id].path.direction == -1

        manager.update_moving(0.5)
        second_update = {snapshot.logical_id: snapshot for snapshot in manager.snapshot()}
        assert second_update[moving_snapshot.logical_id].path.direction == -1
        assert second_update[moving_snapshot.logical_id].path.progress < first_update[moving_snapshot.logical_id].path.progress

        velocity, _angular = p.getBaseVelocity(moving_snapshot.physics_body_id, physicsClientId=client_id)
        assert math.sqrt(sum(component * component for component in velocity)) == pytest.approx(0.60, abs=1e-6)
    finally:
        p.disconnect(client_id)


def test_dashboard_snapshot_is_immutable_copy_without_body_id():
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        _add_one(manager, seed=31)

        dashboard = manager.snapshot(include_body_id=False)
        assert dashboard[0].body_id is None
        with pytest.raises(AttributeError):
            dashboard[0].position = (99.0, 99.0, 99.0)
        assert manager.snapshot()[0].position != (99.0, 99.0, 99.0)
    finally:
        p.disconnect(client_id)


def test_delete_one_delete_missing_and_clear_all():
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        first = _add_one(manager, seed=41)
        second = _add_one(manager, seed=42)

        delete_result = manager.delete(first.logical_id)
        assert delete_result.done and delete_result.succeeded
        assert delete_result.deleted_count == 1
        assert [snapshot.logical_id for snapshot in manager.snapshot()] == [second.logical_id]

        missing_result = manager.delete(9999)
        assert missing_result.done and not missing_result.succeeded
        assert missing_result.deleted_count == 0

        manager.begin_clear()
        clear_result = _finish(manager)
        assert clear_result.done and clear_result.succeeded
        assert clear_result.deleted_count == 1
        assert manager.snapshot() == ()
    finally:
        p.disconnect(client_id)


def test_pending_clear_snapshot_does_not_expose_removed_body_ids():
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(
            client_id,
            _settings(bounds=scene.bounds),
            terrain_body_ids=scene.body_ids,
            soft_budget_seconds=0.0,
        )
        first = _add_one(manager, seed=43)
        second = _add_one(manager, seed=44)

        manager.begin_clear()
        result = manager.advance_pending_operation()

        assert not result.done
        assert result.deleted_count == 1
        assert first.physics_body_id not in _body_ids(client_id)
        remaining = manager.snapshot()
        assert [snapshot.logical_id for snapshot in remaining] == [second.logical_id]
        assert remaining[0].physics_body_id in _body_ids(client_id)

        final = _finish(manager)
        assert final.done and final.succeeded
        assert manager.snapshot() == ()
    finally:
        p.disconnect(client_id)


def test_creation_failure_rolls_back_only_this_batch_and_keeps_existing_logical_collection(monkeypatch):
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        existing = _add_one(manager, seed=51)
        before_ids = _body_ids(client_id)
        before_snapshot = manager.snapshot()

        original_create_multibody = obstacle_module.p.createMultiBody
        created_obstacle_bodies = 0

        def fail_second_real_obstacle(*args, **kwargs):
            nonlocal created_obstacle_bodies
            if kwargs.get("baseCollisionShapeIndex", -1) >= 0:
                created_obstacle_bodies += 1
                if created_obstacle_bodies == 2:
                    body_id = original_create_multibody(*args, **kwargs)
                    raise RuntimeError(f"injected body failure after {body_id}")
            return original_create_multibody(*args, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "createMultiBody", fail_second_real_obstacle)
        manager.begin_add(ObstacleGenerationRequest(mode="static", count=2, seed=52, shape="box"))
        result = _finish(manager)

        assert result.done and not result.succeeded
        assert "injected body failure" in result.message
        assert manager.snapshot() == before_snapshot
        assert _body_ids(client_id) == before_ids
        assert manager.snapshot()[0].logical_id == existing.logical_id
    finally:
        p.disconnect(client_id)


def test_moving_velocity_follows_slope_height_change():
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=12.0, time_step=TIME_STEP, terrain_model="slope")
        manager = ObstacleManager(
            client_id,
            _settings(bounds=scene.bounds),
            terrain_body_ids=scene.body_ids,
        )
        snapshot = ObstacleSnapshot(
            logical_id=1,
            body_id=None,
            mode="moving",
            shape="box",
            position=(-1.0, 0.0, 0.30),
            orientation=(0.0, 0.0, 0.0, 1.0),
            path=ObstaclePath(start_xy=(-1.0, 0.0), end_xy=(1.0, 0.0), speed=1.0),
            geometry=ObstacleGeometry(shape="box", half_extents=(0.20, 0.20, 0.30)),
        )
        manager.restore((snapshot,))
        before = manager.snapshot()[0]

        manager.update_moving(0.5)
        after = manager.snapshot()[0]
        velocity, _angular = p.getBaseVelocity(after.physics_body_id, physicsClientId=client_id)

        assert after.position[2] != pytest.approx(before.position[2])
        assert velocity[0] == pytest.approx((after.position[0] - before.position[0]) / 0.5, abs=1e-6)
        assert velocity[1] == pytest.approx((after.position[1] - before.position[1]) / 0.5, abs=1e-6)
        assert velocity[2] == pytest.approx((after.position[2] - before.position[2]) / 0.5, abs=1e-6)
    finally:
        p.disconnect(client_id)


def test_add_planning_and_hidden_temporary_creation_yield_across_frames(monkeypatch):
    client_id = p.connect(p.DIRECT)
    try:
        clock = FakeClock()
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(
            client_id,
            _settings(bounds=scene.bounds),
            terrain_body_ids=scene.body_ids,
            monotonic_clock=clock,
            soft_budget_seconds=0.002,
        )
        original_plan = obstacle_module.plan_obstacle_batch
        original_create_multibody = obstacle_module.p.createMultiBody

        def slow_plan(*args, **kwargs):
            value = original_plan(*args, **kwargs)
            clock.advance(0.003)
            return value

        def slow_create_multibody(*args, **kwargs):
            body_id = original_create_multibody(*args, **kwargs)
            if kwargs.get("baseCollisionShapeIndex", -1) == -1:
                clock.advance(0.003)
            return body_id

        monkeypatch.setattr(obstacle_module, "plan_obstacle_batch", slow_plan)
        monkeypatch.setattr(obstacle_module.p, "createMultiBody", slow_create_multibody)
        manager.begin_add(ObstacleGenerationRequest(mode="static", count=3, seed=61, shape="box"))

        after_planning = manager.advance_pending_operation()
        assert not after_planning.done
        assert manager.snapshot() == ()

        after_first_temp = manager.advance_pending_operation()
        assert not after_first_temp.done
        assert manager.snapshot() == ()
        temporary_ids = tuple(manager.pending_body_ids())
        assert len(temporary_ids) == 1
        assert p.getCollisionShapeData(temporary_ids[0], -1, physicsClientId=client_id) == ()
        assert p.getVisualShapeData(temporary_ids[0], physicsClientId=client_id)[0][7][3] == pytest.approx(0.0)

        final = _finish(manager)
        assert final.succeeded
        assert manager.pending_body_ids() == ()
        assert len(manager.snapshot()) == 3
    finally:
        p.disconnect(client_id)


def test_stage2_batch_add_50_and_clear_100_yield_across_frames():
    """阶段二验收上限：50 个添加和 100 个清空都要跨帧让出控制。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(
            client_id,
            _settings(
                bounds=scene.bounds,
                max_scene_obstacles=100,
                half_extent_ranges=((0.08, 0.08), (0.08, 0.08), (0.12, 0.12)),
                minimum_clearance=0.01,
                spawn_protection_radius=0.20,
                max_candidate_attempts=3000,
            ),
            terrain_body_ids=scene.body_ids,
            soft_budget_seconds=0.0,
        )
        manager.begin_add(ObstacleGenerationRequest(mode="mixed", count=50, seed=101))
        first_add = manager.advance_pending_operation()
        assert not first_add.done
        assert manager.snapshot() == ()
        add_result = _finish(manager)
        assert add_result.done and add_result.succeeded
        assert add_result.published_count == 50

        manager.begin_add(ObstacleGenerationRequest(mode="mixed", count=50, seed=202))
        second_add = _finish(manager)
        assert second_add.done and second_add.succeeded
        assert second_add.published_count == 50
        assert len(manager.snapshot()) == 100

        manager.begin_clear()
        first_clear = manager.advance_pending_operation()
        assert not first_clear.done
        assert first_clear.deleted_count == 1
        clear_result = _finish(manager)
        assert clear_result.done and clear_result.succeeded
        assert clear_result.deleted_count == 100
        assert manager.snapshot() == ()
    finally:
        p.disconnect(client_id)


def test_temporary_bodies_are_non_colliding_and_logical_list_publishes_once_after_batch_success():
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        obstacle_collision = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=(0.30, 0.30, 0.30),
            physicsClientId=client_id,
        )
        probe_body = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=obstacle_collision,
            basePosition=(0.0, 0.0, 0.30),
            physicsClientId=client_id,
        )
        manager = ObstacleManager(
            client_id,
            _settings(bounds=scene.bounds),
            terrain_body_ids=scene.body_ids,
            monotonic_clock=FakeClock(),
            soft_budget_seconds=0.0,
        )
        manager.begin_add(ObstacleGenerationRequest(mode="static", count=2, seed=71, shape="box"))

        first = manager.advance_pending_operation()
        assert not first.done
        assert manager.snapshot() == ()
        second = manager.advance_pending_operation()
        assert not second.done
        assert manager.snapshot() == ()
        for temporary_id in manager.pending_body_ids():
            assert p.getContactPoints(probe_body, temporary_id, physicsClientId=client_id) == ()

        final = _finish(manager)
        assert final.done and final.succeeded
        assert len(manager.snapshot()) == 2
    finally:
        p.disconnect(client_id)


def test_final_commit_rereads_vehicle_aabb_and_cancels_if_vehicle_enters_candidate_area():
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        current_aabb = {"value": None}
        manager = ObstacleManager(
            client_id,
            _settings(bounds=scene.bounds, vehicle_aabb=None),
            terrain_body_ids=scene.body_ids,
            vehicle_aabb_getter=lambda: current_aabb["value"],
            soft_budget_seconds=0.0,
        )
        manager.begin_add(ObstacleGenerationRequest(mode="static", count=1, seed=81, shape="box"))
        first = manager.advance_pending_operation()
        assert not first.done
        assert len(manager.pending_specs()) == 1
        candidate = manager.pending_specs()[0]
        radius = candidate.geometry.bounding_radius + 0.01
        current_aabb["value"] = (
            (candidate.position[0] - radius, candidate.position[1] - radius, -0.10),
            (candidate.position[0] + radius, candidate.position[1] + radius, 0.80),
        )

        final = _finish(manager)
        assert final.done and not final.succeeded
        assert "vehicle AABB" in final.message
        assert manager.snapshot() == ()
        assert manager.pending_body_ids() == ()
    finally:
        p.disconnect(client_id)


def test_restore_from_snapshot_preserves_logical_id_xy_path_progress_and_direction():
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(
            client_id,
            _settings(bounds=scene.bounds, moving_path_length_range=(0.80, 0.80)),
            terrain_body_ids=scene.body_ids,
        )
        original = _add_one(manager, mode="moving", seed=91, moving_speed=0.40)
        manager.update_moving(0.25)
        saved = manager.snapshot(include_body_id=False)
        saved_obstacle = saved[0]
        assert saved_obstacle.path is not None
        old_body_id = manager.snapshot()[0].physics_body_id

        p.removeBody(old_body_id, physicsClientId=client_id)
        manager.restore(saved)
        restored = manager.snapshot()[0]

        assert restored.logical_id == saved_obstacle.logical_id == original.logical_id
        assert restored.position[:2] == pytest.approx(saved_obstacle.position[:2], abs=1e-9)
        assert restored.position[2] == pytest.approx(saved_obstacle.geometry.half_extents[2], abs=0.04)
        assert math.hypot(*restored.orientation) == pytest.approx(1.0, abs=1e-9)
        assert restored.physics_body_id is not None
        assert restored.physics_body_id in _body_ids(client_id)
        assert restored.path.start_xy == pytest.approx(saved_obstacle.path.start_xy)
        assert restored.path.end_xy == pytest.approx(saved_obstacle.path.end_xy)
        assert restored.path.progress == pytest.approx(saved_obstacle.path.progress)
        assert restored.path.direction == saved_obstacle.path.direction
    finally:
        p.disconnect(client_id)


def test_restore_failure_or_duplicate_ids_preserves_existing_collection():
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        existing = _add_one(manager, seed=92)
        before_snapshot = manager.snapshot()
        before_body_ids = _body_ids(client_id)

        missing_geometry = ObstacleSnapshot(
            logical_id=99,
            body_id=None,
            mode="static",
            shape="box",
            position=(1.0, 1.0, 0.30),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )
        with pytest.raises(ValueError, match="geometry"):
            manager.restore((missing_geometry,))
        assert manager.snapshot() == before_snapshot
        assert _body_ids(client_id) == before_body_ids

        saved = manager.snapshot(include_body_id=False)
        with pytest.raises(ValueError, match="duplicate"):
            manager.restore((saved[0], saved[0]))
        assert manager.snapshot()[0].logical_id == existing.logical_id
        assert _body_ids(client_id) == before_body_ids
    finally:
        p.disconnect(client_id)
