# ObstacleManager 集成测试：覆盖 PyBullet body 创建、跨帧事务、删除和快照恢复。
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


def test_update_moving_does_not_search_records_by_dataclass_equality(monkeypatch):
    """移动更新必须按稳定索引发布，不能对已更新列表逐项执行 index 搜索。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        for offset in range(5):
            _add_one(
                manager,
                mode="moving",
                seed=230 + offset,
                moving_speed=0.3,
            )
        original_eq = obstacle_module._ObstacleRecord.__eq__
        equality_calls = 0

        def count_equality(left, right):
            nonlocal equality_calls
            equality_calls += 1
            return original_eq(left, right)

        monkeypatch.setattr(obstacle_module._ObstacleRecord, "__eq__", count_equality)

        manager.update_moving(0.01)

        assert equality_calls == 0
    finally:
        p.disconnect(client_id)


def test_update_moving_batches_all_terrain_rays_once(monkeypatch):
    """同一帧全部移动体必须共享一次 rayTestBatch，不重复标量地形探测。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        for offset in range(6):
            _add_one(
                manager,
                mode="moving",
                seed=250 + offset,
                moving_speed=0.3,
            )
        original_probe = obstacle_module.probe_terrain
        original_batch = obstacle_module.p.rayTestBatch
        scalar_calls = 0
        batch_sizes: list[int] = []

        def count_scalar_probe(*args, **kwargs):
            nonlocal scalar_calls
            scalar_calls += 1
            return original_probe(*args, **kwargs)

        def count_batch(ray_from, ray_to, **kwargs):
            batch_sizes.append(len(ray_from))
            return original_batch(ray_from, ray_to, **kwargs)

        monkeypatch.setattr(obstacle_module, "probe_terrain", count_scalar_probe)
        monkeypatch.setattr(obstacle_module.p, "rayTestBatch", count_batch)

        manager.update_moving(0.01)

        assert scalar_calls == 0
        assert batch_sizes == [6]
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(
    ("terrain_model", "slope_deg"),
    (("flat", 0.0), ("slope", 8.0), ("golf_heightfield", 0.0)),
)
def test_batch_terrain_sampling_matches_scalar_probe_pose(
    terrain_model: str,
    slope_deg: float,
):
    """batch 射线须与旧逐点探测生成相同地高、法向和障碍物姿态。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(
            client_id,
            slope_deg=slope_deg,
            time_step=TIME_STEP,
            terrain_model=terrain_model,
            golf_seed=37,
            golf_relief="medium",
        )
        manager = ObstacleManager(
            client_id,
            _settings(bounds=scene.bounds),
            terrain_body_ids=scene.body_ids,
        )
        positions = ((-2.25, -1.10), (0.15, 0.20), (2.40, 1.25))
        batch_probes = manager._sample_moving_terrain_batch(positions)
        scalar_probes = tuple(manager._terrain_sampler(x, y) for x, y in positions)
        geometry = ObstacleGeometry("box", (0.20, 0.20, 0.30))

        for index, (batch_probe, scalar_probe) in enumerate(
            zip(batch_probes, scalar_probes, strict=True),
            start=1,
        ):
            assert batch_probe.local_ground_height == pytest.approx(
                scalar_probe.local_ground_height,
                abs=2e-4,
            )
            batch_normal = (
                batch_probe.local_terrain_normal_x,
                batch_probe.local_terrain_normal_y,
                batch_probe.local_terrain_normal_z,
            )
            scalar_normal = (
                scalar_probe.local_terrain_normal_x,
                scalar_probe.local_terrain_normal_y,
                scalar_probe.local_terrain_normal_z,
            )
            assert batch_normal == pytest.approx(scalar_normal, abs=1e-5)
            base_spec = obstacle_module.ObstacleSpec(
                logical_id=index,
                mode="static",
                geometry=geometry,
                position=(positions[index - 1][0], positions[index - 1][1], 0.30),
                orientation=(0.0, 0.0, 0.0, 1.0),
            )
            batch_spec = manager._spec_with_sampled_pose(
                base_spec,
                positions[index - 1],
                (1.0, 0.0),
                batch_probe,
            )
            scalar_spec = manager._spec_with_sampled_pose(
                base_spec,
                positions[index - 1],
                (1.0, 0.0),
                scalar_probe,
            )
            assert batch_spec.position == pytest.approx(scalar_spec.position, abs=2e-4)
            assert batch_spec.orientation == pytest.approx(scalar_spec.orientation, abs=1e-5)
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


def test_manager_exposes_read_only_specs_revision_and_constant_time_moving_flag():
    """热路径元数据不能暴露 body id 或可变内部 records。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)

        assert hasattr(manager, "has_moving")
        assert hasattr(manager, "revision")
        assert hasattr(manager, "logical_specs")
        initial_revision = manager.revision
        assert manager.has_moving is False

        _add_one(manager, mode="static", seed=240)
        static_revision = manager.revision
        specs = manager.logical_specs()
        assert static_revision > initial_revision
        assert manager.has_moving is False
        assert isinstance(specs, tuple)
        assert all(isinstance(spec, obstacle_module.ObstacleSpec) for spec in specs)
        assert all(not hasattr(spec, "body_id") for spec in specs)

        _add_one(manager, mode="moving", seed=241, moving_speed=0.3)
        moving_revision = manager.revision
        assert manager.has_moving is True
        manager.update_moving(0.01)
        assert manager.revision > moving_revision
        assert manager.logical_specs()[0] is specs[0]
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


def test_create_body_propagates_residual_id_when_filter_failure_cleanup_is_fake(
    monkeypatch,
):
    """createMultiBody 后置失败且清理假成功时，异常必须携带残留 body ID。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        spec = obstacle_module.ObstacleSpec(
            logical_id=70,
            mode="static",
            geometry=ObstacleGeometry("box", (0.20, 0.20, 0.30)),
            position=(2.0, 1.0, 0.30),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )
        original_filter = obstacle_module.p.setCollisionFilterGroupMask
        original_remove = obstacle_module.p.removeBody
        residual_body_id: int | None = None

        def fail_real_filter(body_id, link_id, group, mask, **kwargs):
            nonlocal residual_body_id
            if group == obstacle_module.OBSTACLE_COLLISION_GROUP:
                residual_body_id = int(body_id)
                raise RuntimeError("injected obstacle filter failure")
            return original_filter(body_id, link_id, group, mask, **kwargs)

        def leave_residual_body(body_id: int, **kwargs) -> None:
            if body_id == residual_body_id:
                return
            original_remove(body_id, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "setCollisionFilterGroupMask", fail_real_filter)
        monkeypatch.setattr(obstacle_module.p, "removeBody", leave_residual_body)

        with pytest.raises(RuntimeError) as excinfo:
            obstacle_module._create_obstacle_body(client_id, spec, temporary=False)

        assert "injected obstacle filter failure" in str(excinfo.value)
        assert residual_body_id is not None
        assert getattr(excinfo.value, "residual_body_ids", ()) == (residual_body_id,)
        assert residual_body_id in _body_ids(client_id)
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("operation", ("add", "restore"))
def test_manager_registers_post_creation_residual_as_cleanup_debt(
    monkeypatch,
    operation: str,
):
    """add/restore 都必须接管创建异常留下的候选 body，不能丢失所有权。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(
            client_id,
            _settings(bounds=scene.bounds),
            terrain_body_ids=scene.body_ids,
        )
        original_filter = obstacle_module.p.setCollisionFilterGroupMask
        original_remove = obstacle_module.p.removeBody
        residual_body_id: int | None = None

        def fail_real_filter(body_id, link_id, group, mask, **kwargs):
            nonlocal residual_body_id
            if group == obstacle_module.OBSTACLE_COLLISION_GROUP:
                residual_body_id = int(body_id)
                raise RuntimeError(f"{operation} filter failure")
            return original_filter(body_id, link_id, group, mask, **kwargs)

        def leave_residual_body(body_id: int, **kwargs) -> None:
            if body_id == residual_body_id:
                return
            original_remove(body_id, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "setCollisionFilterGroupMask", fail_real_filter)
        monkeypatch.setattr(obstacle_module.p, "removeBody", leave_residual_body)

        if operation == "add":
            manager.begin_add(ObstacleGenerationRequest("static", 1, seed=71))
            result = _finish(manager)
            assert result.succeeded is False
        else:
            target = ObstacleSnapshot(
                71,
                None,
                "static",
                "box",
                (2.0, 1.0, 0.30),
                (0.0, 0.0, 0.0, 1.0),
                geometry=ObstacleGeometry("box", (0.20, 0.20, 0.30)),
            )
            with pytest.raises(RuntimeError, match="restore filter failure"):
                manager.restore((target,))

        assert residual_body_id is not None
        assert manager.pending_body_ids() == (residual_body_id,)
        formal_body_ids = {
            snapshot.physics_body_id for snapshot in manager.snapshot()
        }
        assert residual_body_id not in formal_body_ids
    finally:
        p.disconnect(client_id)


def test_cleanup_debt_retries_strictly_and_transient_failure_recovers(monkeypatch):
    """新操作门禁每次都重试 debt；瞬时失败消失后应自动清债并继续。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        spec = obstacle_module.ObstacleSpec(
            81,
            "static",
            ObstacleGeometry("box", (0.20, 0.20, 0.30)),
            (2.0, 1.0, 0.30),
            (0.0, 0.0, 0.0, 1.0),
        )
        debt_body_id = obstacle_module._create_obstacle_body(client_id, spec, temporary=False)
        manager._register_cleanup_debt(((debt_body_id, RuntimeError("seed debt")),))
        original_remove = obstacle_module.p.removeBody
        attempts = 0

        def fail_debt_once(body_id: int, **kwargs) -> None:
            nonlocal attempts
            if body_id == debt_body_id:
                attempts += 1
                if attempts == 1:
                    raise RuntimeError("transient cleanup failure")
            original_remove(body_id, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "removeBody", fail_debt_once)

        with pytest.raises(RuntimeError) as excinfo:
            manager.begin_clear()
        assert str(debt_body_id) in str(excinfo.value)
        assert manager.pending_body_ids() == (debt_body_id,)

        started = manager.begin_clear()
        assert started.done is False
        assert manager.pending_body_ids() == ()
        assert debt_body_id not in _body_ids(client_id)
        assert _finish(manager).succeeded is True
    finally:
        p.disconnect(client_id)


def test_cleanup_debt_fake_success_stays_blocked_with_concrete_body_id(monkeypatch):
    """debt 删除正常返回但 body 仍在时，门禁必须保留债务并指出具体 ID。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        spec = obstacle_module.ObstacleSpec(
            82,
            "static",
            ObstacleGeometry("box", (0.20, 0.20, 0.30)),
            (2.0, 1.0, 0.30),
            (0.0, 0.0, 0.0, 1.0),
        )
        debt_body_id = obstacle_module._create_obstacle_body(client_id, spec, temporary=False)
        manager._register_cleanup_debt(((debt_body_id, RuntimeError("seed debt")),))
        original_remove = obstacle_module.p.removeBody

        def leave_debt_body(body_id: int, **kwargs) -> None:
            if body_id == debt_body_id:
                return
            original_remove(body_id, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "removeBody", leave_debt_body)

        with pytest.raises(RuntimeError) as excinfo:
            manager.begin_clear()

        assert str(debt_body_id) in str(excinfo.value)
        assert "remained" in str(excinfo.value)
        assert manager.pending_body_ids() == (debt_body_id,)
        assert debt_body_id in _body_ids(client_id)
        assert manager.snapshot() == ()
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


def test_add_planning_yields_after_each_budgeted_candidate(monkeypatch):
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
        original_make_candidate = obstacle_module._make_candidate_spec

        def slow_make_candidate(*args, **kwargs):
            value = original_make_candidate(*args, **kwargs)
            clock.advance(0.003)
            return value

        monkeypatch.setattr(obstacle_module, "_make_candidate_spec", slow_make_candidate)
        manager.begin_add(ObstacleGenerationRequest(mode="static", count=3, seed=61, shape="box"))

        first_candidate = manager.advance_pending_operation()
        assert not first_candidate.done
        assert manager.snapshot() == ()
        assert len(manager.pending_specs()) == 1

        second_candidate = manager.advance_pending_operation()
        assert not second_candidate.done
        assert manager.snapshot() == ()
        assert len(manager.pending_specs()) == 2

        final = _finish(manager)
        assert final.succeeded
        assert manager.pending_body_ids() == ()
        assert len(manager.snapshot()) == 3
    finally:
        p.disconnect(client_id)


def test_final_candidate_creation_yields_before_atomic_publication(monkeypatch):
    """正式候选创建同样受时间片约束，不能在一次推进中阻塞整批。"""
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
        original_create_multibody = obstacle_module.p.createMultiBody

        def slow_final_create(*args, **kwargs):
            body_id = original_create_multibody(*args, **kwargs)
            if kwargs.get("baseCollisionShapeIndex", -1) != -1:
                clock.advance(0.003)
            return body_id

        monkeypatch.setattr(obstacle_module.p, "createMultiBody", slow_final_create)
        manager.begin_add(ObstacleGenerationRequest(mode="static", count=3, seed=62, shape="box"))

        pending = manager.advance_pending_operation()

        assert not pending.done
        assert manager.snapshot() == ()
        final = _finish(manager)

        assert final.succeeded
        assert len(manager.snapshot()) == 3
        assert manager.pending_body_ids() == ()
    finally:
        p.disconnect(client_id)


def test_temporary_candidate_cleanup_yields_before_atomic_publication(monkeypatch):
    """提交前的临时候选清理不能在一个时间片内删除整批 body。"""
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
        original_remove_body = obstacle_module.p.removeBody

        def slow_remove_body(*args, **kwargs):
            original_remove_body(*args, **kwargs)
            clock.advance(0.003)

        monkeypatch.setattr(obstacle_module.p, "removeBody", slow_remove_body)
        manager.begin_add(ObstacleGenerationRequest(mode="static", count=3, seed=63, shape="box"))

        pending = manager.advance_pending_operation()

        assert not pending.done
        assert manager.snapshot() == ()
        # 候选正式 body 与尚未清理的临时 body 都仍由 manager 管理。
        assert len(manager.pending_body_ids()) == 5
        final = _finish(manager)

        assert final.succeeded
        assert len(manager.snapshot()) == 3
        assert manager.pending_body_ids() == ()
    finally:
        p.disconnect(client_id)


def test_final_temporary_cleanup_failure_keeps_body_as_cleanup_debt(monkeypatch):
    """提交前临时 body 删除失败时，所有权必须转入 debt 而非丢失。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(
            client_id,
            _settings(bounds=scene.bounds),
            terrain_body_ids=scene.body_ids,
        )
        original_remove_body = obstacle_module.p.removeBody
        failed_body_id: int | None = None

        def fail_first_temporary_remove(body_id, **kwargs):
            nonlocal failed_body_id
            if failed_body_id is None:
                failed_body_id = int(body_id)
                raise p.error("injected temporary cleanup failure")
            original_remove_body(body_id, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "removeBody", fail_first_temporary_remove)
        manager.begin_add(ObstacleGenerationRequest(mode="static", count=2, seed=64, shape="box"))

        result = _finish(manager)

        assert result.done is True
        assert result.succeeded is False
        assert failed_body_id is not None
        assert manager.snapshot() == ()
        assert manager.pending_body_ids() == (failed_body_id,)
        assert failed_body_id in _body_ids(client_id)
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
        add_result = first_add
        for _ in range(400):
            if add_result.done:
                break
            add_result = manager.advance_pending_operation()
        assert add_result.done
        assert add_result.done and add_result.succeeded
        assert add_result.published_count == 50

        manager.begin_add(ObstacleGenerationRequest(mode="mixed", count=50, seed=202))
        second_add = manager.advance_pending_operation()
        for _ in range(400):
            if second_add.done:
                break
            second_add = manager.advance_pending_operation()
        assert second_add.done
        assert second_add.done and second_add.succeeded
        assert second_add.published_count == 50
        assert len(manager.snapshot()) == 100

        manager.begin_clear()
        first_clear = manager.advance_pending_operation()
        assert not first_clear.done
        assert first_clear.deleted_count == 1
        clear_result = first_clear
        for _ in range(400):
            if clear_result.done:
                break
            clear_result = manager.advance_pending_operation()
        assert clear_result.done
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

        result = manager.restore(saved)
        restored = manager.snapshot()[0]

        assert result.succeeded is True
        assert restored.logical_id == saved_obstacle.logical_id == original.logical_id
        assert restored.position[:2] == pytest.approx(saved_obstacle.position[:2], abs=1e-9)
        assert restored.position[2] == pytest.approx(saved_obstacle.geometry.half_extents[2], abs=0.04)
        assert math.hypot(*restored.orientation) == pytest.approx(1.0, abs=1e-9)
        assert restored.physics_body_id is not None
        assert restored.physics_body_id != old_body_id
        assert restored.physics_body_id in _body_ids(client_id)
        assert restored.path.start_xy == pytest.approx(saved_obstacle.path.start_xy)
        assert restored.path.end_xy == pytest.approx(saved_obstacle.path.end_xy)
        assert restored.path.progress == pytest.approx(saved_obstacle.path.progress)
        assert restored.path.direction == saved_obstacle.path.direction
    finally:
        p.disconnect(client_id)


def test_restore_rejects_missing_original_before_creating_any_candidate(monkeypatch):
    """正式记录缺 body 时立即进入 fault，不能让新候选复用缺失 ID。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        _add_one(manager, seed=181)
        _add_one(manager, seed=182)
        before = manager.snapshot()
        missing_body_id = before[0].physics_body_id
        survivor_body_id = before[1].physics_body_id
        assert missing_body_id is not None and survivor_body_id is not None
        p.removeBody(missing_body_id, physicsClientId=client_id)
        original_create = obstacle_module._create_obstacle_body
        creation_calls: list[int] = []

        def record_creation(client_id_arg, spec, **kwargs):
            creation_calls.append(spec.logical_id)
            return original_create(client_id_arg, spec, **kwargs)

        monkeypatch.setattr(obstacle_module, "_create_obstacle_body", record_creation)
        target = ObstacleSnapshot(
            90,
            None,
            "static",
            "box",
            (2.0, 1.0, 0.30),
            (0.0, 0.0, 0.0, 1.0),
            geometry=ObstacleGeometry("box", (0.20, 0.20, 0.30)),
        )

        with pytest.raises(RuntimeError) as excinfo:
            manager.restore((target,))

        assert str(missing_body_id) in str(excinfo.value)
        assert "missing" in str(excinfo.value)
        assert creation_calls == []
        assert manager.faulted is True
        assert manager.snapshot() == ()
        assert manager.pending_body_ids() == (survivor_body_id,)
    finally:
        p.disconnect(client_id)


def test_restore_missing_id_reuse_combination_never_publishes_candidate_as_old(
    monkeypatch,
):
    """预缺失、ID 复用、另一旧体失败和候选清理失败组合下不得错配所有权。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        _add_one(manager, seed=183)
        _add_one(manager, seed=184)
        original_records = manager.snapshot()
        missing_body_id = original_records[0].physics_body_id
        blocked_body_id = original_records[1].physics_body_id
        assert missing_body_id is not None and blocked_body_id is not None
        p.removeBody(missing_body_id, physicsClientId=client_id)
        original_create = obstacle_module._create_obstacle_body
        original_remove = obstacle_module.p.removeBody
        candidate_ids: list[int] = []

        def record_candidate(client_id_arg, spec, **kwargs):
            body_id = original_create(client_id_arg, spec, **kwargs)
            if spec.logical_id == 91:
                candidate_ids.append(body_id)
            return body_id

        def fail_old_and_candidate_cleanup(body_id: int, **kwargs) -> None:
            if body_id == blocked_body_id:
                raise RuntimeError("other old body removal failed")
            if body_id in candidate_ids:
                raise RuntimeError("candidate cleanup failed")
            original_remove(body_id, **kwargs)

        monkeypatch.setattr(obstacle_module, "_create_obstacle_body", record_candidate)
        monkeypatch.setattr(
            obstacle_module.p,
            "removeBody",
            fail_old_and_candidate_cleanup,
        )
        target = ObstacleSnapshot(
            91,
            None,
            "static",
            "sphere",
            (2.0, -1.0, 0.20),
            (0.0, 0.0, 0.0, 1.0),
            geometry=ObstacleGeometry("sphere", (0.20, 0.20, 0.20)),
        )

        with pytest.raises(RuntimeError):
            manager.restore((target,))

        assert candidate_ids == []
        assert manager.faulted is True
        assert manager.snapshot() == ()
        assert blocked_body_id in manager.pending_body_ids()
        assert not any(
            snapshot.logical_id == original_records[0].logical_id
            and snapshot.physics_body_id in candidate_ids
            for snapshot in manager.snapshot()
        )
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("failure_mode", ("raises", "remains"))
def test_restore_nonempty_collection_rolls_back_when_first_old_body_is_not_removed(
    monkeypatch,
    failure_mode,
):
    """旧 body 抛非 p.error 或假成功时，候选必须清空且原集合保持有效。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        _add_one(manager, mode="moving", seed=191, moving_speed=0.4)
        _add_one(manager, mode="static", seed=192)
        before_logical = manager.snapshot(include_body_id=False)
        before_physical = manager.snapshot()
        failed_body_id = before_physical[0].physics_body_id
        original_remove = obstacle_module.p.removeBody
        target = (
            ObstacleSnapshot(
                logical_id=101,
                body_id=None,
                mode="static",
                shape="box",
                position=(2.0, 1.0, 0.3),
                orientation=(0.0, 0.0, 0.0, 1.0),
                geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
            ),
        )

        def fail_or_leave_old_body(body_id: int, **kwargs) -> None:
            if body_id == failed_body_id:
                if failure_mode == "raises":
                    raise RuntimeError("old committed body removal exploded")
                return
            original_remove(body_id, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "removeBody", fail_or_leave_old_body)

        result = manager.restore(target)

        assert result.done is True
        assert result.succeeded is False
        expected_reason = "old committed body removal exploded" if failure_mode == "raises" else "remained"
        assert expected_reason in result.message
        assert manager.snapshot(include_body_id=False) == before_logical
        assert manager.snapshot() == before_physical
        expected_body_ids = set(scene.body_ids) | {
            snapshot.physics_body_id for snapshot in manager.snapshot()
        }
        assert _body_ids(client_id) == expected_body_ids
    finally:
        p.disconnect(client_id)


def test_restore_partial_old_removal_recreates_deleted_specs_in_original_order(
    monkeypatch,
):
    """第一旧体已删、第二旧体失败时，要从原 spec 重建并恢复完整顺序。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        _add_one(manager, mode="moving", seed=201, moving_speed=0.35)
        _add_one(manager, mode="static", seed=202)
        _add_one(manager, mode="moving", seed=203, moving_speed=0.45)
        manager.update_moving(0.2)
        before_logical = manager.snapshot(include_body_id=False)
        before_physical = manager.snapshot()
        failed_body_id = before_physical[1].physics_body_id
        original_remove = obstacle_module.p.removeBody
        original_create = obstacle_module._create_obstacle_body
        created_logical_ids: list[int] = []
        target = (
            ObstacleSnapshot(
                logical_id=301,
                body_id=None,
                mode="static",
                shape="sphere",
                position=(2.0, -1.0, 0.2),
                orientation=(0.0, 0.0, 0.0, 1.0),
                geometry=ObstacleGeometry("sphere", (0.2, 0.2, 0.2)),
            ),
        )

        def fail_second_old_body(body_id: int, **kwargs) -> None:
            if body_id == failed_body_id:
                raise RuntimeError("second old body removal failed")
            original_remove(body_id, **kwargs)

        def record_created_spec(client_id_arg, spec, **kwargs):
            created_logical_ids.append(spec.logical_id)
            return original_create(client_id_arg, spec, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "removeBody", fail_second_old_body)
        monkeypatch.setattr(obstacle_module, "_create_obstacle_body", record_created_spec)

        result = manager.restore(target)

        assert result.done is True
        assert result.succeeded is False
        assert "second old body removal failed" in result.message
        assert manager.snapshot(include_body_id=False) == before_logical
        after_physical = manager.snapshot()
        assert [snapshot.logical_id for snapshot in after_physical] == [1, 2, 3]
        assert created_logical_ids == [301, 1]
        assert after_physical[0].physics_body_id in _body_ids(client_id)
        expected_body_ids = set(scene.body_ids) | {
            snapshot.physics_body_id for snapshot in after_physical
        }
        assert _body_ids(client_id) == expected_body_ids
    finally:
        p.disconnect(client_id)


def test_restore_reports_primary_and_candidate_cleanup_failures(monkeypatch):
    """候选无法清理时必须同时报告旧体删除首错和候选回滚错。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        _add_one(manager, mode="static", seed=211)
        before = manager.snapshot(include_body_id=False)
        old_body_id = manager.snapshot()[0].physics_body_id
        target = (
            ObstacleSnapshot(
                401,
                None,
                "static",
                "box",
                (2.0, 1.0, 0.3),
                (0.0, 0.0, 0.0, 1.0),
                geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
            ),
        )

        def fail_old_and_candidate(body_id: int, **_kwargs) -> None:
            if body_id == old_body_id:
                raise RuntimeError("primary old removal failure")
            raise RuntimeError("candidate cleanup failure")

        monkeypatch.setattr(obstacle_module.p, "removeBody", fail_old_and_candidate)

        with pytest.raises(RuntimeError) as excinfo:
            manager.restore(target)

        assert "primary old removal failure" in str(excinfo.value)
        assert "candidate cleanup failure" in str(excinfo.value)
        assert manager.snapshot(include_body_id=False) == before
        logical_body_ids = {
            snapshot.physics_body_id for snapshot in manager.snapshot()
        }
        cleanup_debt = _body_ids(client_id) - set(scene.body_ids) - logical_body_ids
        assert cleanup_debt
        assert set(manager.pending_body_ids()) == cleanup_debt
    finally:
        p.disconnect(client_id)


def test_restore_reports_primary_and_old_spec_rebuild_failures(monkeypatch):
    """部分旧体已删且补建失败时，异常必须包含两段原因且不得伪装完整回滚。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings(bounds=scene.bounds), terrain_body_ids=scene.body_ids)
        _add_one(manager, mode="moving", seed=221, moving_speed=0.3)
        _add_one(manager, mode="static", seed=222)
        before = manager.snapshot()
        failed_body_id = before[1].physics_body_id
        original_remove = obstacle_module.p.removeBody
        original_create = obstacle_module._create_obstacle_body
        target = (
            ObstacleSnapshot(
                501,
                None,
                "static",
                "sphere",
                (2.0, -1.0, 0.2),
                (0.0, 0.0, 0.0, 1.0),
                geometry=ObstacleGeometry("sphere", (0.2, 0.2, 0.2)),
            ),
        )

        def fail_second_old_body(body_id: int, **kwargs) -> None:
            if body_id == failed_body_id:
                raise RuntimeError("primary second-old removal failure")
            original_remove(body_id, **kwargs)

        def fail_rebuilding_first_old(client_id_arg, spec, **kwargs):
            if spec.logical_id == before[0].logical_id:
                raise RuntimeError("old spec rebuild failure")
            return original_create(client_id_arg, spec, **kwargs)

        monkeypatch.setattr(obstacle_module.p, "removeBody", fail_second_old_body)
        monkeypatch.setattr(
            obstacle_module,
            "_create_obstacle_body",
            fail_rebuilding_first_old,
        )

        with pytest.raises(RuntimeError) as excinfo:
            manager.restore(target)

        assert "primary second-old removal failure" in str(excinfo.value)
        assert "old spec rebuild failure" in str(excinfo.value)
        assert manager.faulted is True
        assert manager.snapshot() == ()
        owned_body_ids = _body_ids(client_id) - set(scene.body_ids)
        assert owned_body_ids
        assert set(manager.pending_body_ids()) == owned_body_ids
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
