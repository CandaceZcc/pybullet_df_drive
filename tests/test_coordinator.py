# 场景协调器测试：验证运行时结构操作、障碍物事务和物理步进的主线程编排。
from __future__ import annotations

from types import SimpleNamespace

import pybullet as p
import pytest

import slope_sim.coordinator as coordinator_module
import slope_sim.obstacles as obstacle_module
from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import SimulationCoordinator, load_manual_world
from slope_sim.obstacles import (
    ObstacleGenerationRequest,
    ObstacleGenerationSettings,
    ObstacleGeometry,
    ObstacleManager,
    ObstaclePath,
    ObstacleSpec,
    ObstacleSnapshot,
)
from slope_sim.runtime_actions import (
    AddObstaclesAction,
    ClearObstaclesAction,
    DeleteObstacleAction,
    ResetRobotAction,
    SwitchRobotAction,
    SwitchTerrainAction,
    TerrainSelection,
)
from slope_sim.scene import TerrainBounds


def _body_ids(client_id: int) -> set[int]:
    return {p.getBodyUniqueId(index, physicsClientId=client_id) for index in range(p.getNumBodies(client_id))}


def _manager(client_id: int, world, *, soft_budget_seconds: float = 0.0) -> ObstacleManager:
    bounds = world.scene.bounds or TerrainBounds(-8.0, 8.0, -4.0, 4.0)
    return ObstacleManager(
        client_id,
        ObstacleGenerationSettings(
            bounds=bounds,
            spawn_position=world.scene.spawn_position,
            spawn_protection_radius=0.4,
            max_candidate_attempts=1000,
        ),
        terrain_body_ids=world.scene.body_ids,
        soft_budget_seconds=soft_budget_seconds,
    )


def _restore_pair(manager: ObstacleManager) -> tuple[ObstacleSnapshot, ObstacleSnapshot]:
    static = ObstacleSnapshot(
        logical_id=1,
        body_id=None,
        mode="static",
        shape="box",
        position=(-2.0, -1.0, 0.3),
        orientation=(0.0, 0.0, 0.0, 1.0),
        geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
    )
    moving = ObstacleSnapshot(
        logical_id=2,
        body_id=None,
        mode="moving",
        shape="sphere",
        position=(1.0, 0.5, 0.2),
        orientation=(0.0, 0.0, 0.0, 1.0),
        path=ObstaclePath(start_xy=(1.0, 0.5), end_xy=(2.0, 0.5), speed=0.4, progress=0.35, direction=-1),
        geometry=ObstacleGeometry("sphere", (0.2, 0.2, 0.2)),
    )
    result = manager.restore((static, moving))
    assert result.succeeded is True
    return static, moving


def test_coordinator_fifo_starts_one_structural_operation_at_a_time(monkeypatch):
    """跨帧障碍物事务未完成前，后续结构操作不能插队执行。"""
    started: list[str] = []

    class SlowObstacleManager:
        def __init__(self) -> None:
            self.advances = 0

        def begin_add(self, _request):
            started.append("add")
            return SimpleNamespace(done=False, succeeded=False, operation="add", message="pending")

        def advance_pending_operation(self):
            self.advances += 1
            done = self.advances >= 2
            return SimpleNamespace(done=done, succeeded=done, operation="add", message="pending")

        def update_moving(self, _dt):
            pass

    world = SimpleNamespace(active_robot=SimpleNamespace(robot=SimpleNamespace(command_twist=lambda *_args, **_kwargs: None)))
    coordinator = SimulationCoordinator(
        client_id=0,
        config=SimpleNamespace(time_step=0.01),
        world=world,
        obstacle_manager=SlowObstacleManager(),
        step_physics=lambda _client_id: started.append("step"),
    )
    def fake_replace(_client_id, _config, current_world, _robot_model):
        started.append("robot")
        return SimpleNamespace(world=current_world, state_changed=True, world_reset=False, status_message="robot")

    monkeypatch.setattr(coordinator_module, "replace_manual_world_robot", fake_replace)

    coordinator.enqueue(AddObstaclesAction(ObstacleGenerationRequest("static", 1, seed=1)))
    coordinator.enqueue(SwitchRobotAction("df_mid"))

    coordinator.step(0.01)
    assert started == ["add", "step"]

    coordinator.step(0.01)
    assert started == ["add", "step", "step"]

    coordinator.step(0.01)
    assert started[-2:] == ["robot", "step"]
    assert started.count("robot") == 1


def test_coordinator_reports_pending_after_immediate_action_when_queue_remains(monkeypatch):
    """真实协调器完成一个即时结构动作后，后续 FIFO 未执行前仍应报告 pending。"""
    events: list[str] = []
    robot = SimpleNamespace(command_twist=lambda *_args, **_kwargs: None)
    world = SimpleNamespace(active_robot=SimpleNamespace(robot=robot))
    manager = SimpleNamespace(update_moving=lambda _dt: events.append("moving"))
    coordinator = SimulationCoordinator(
        client_id=0,
        config=SimpleNamespace(time_step=0.01),
        world=world,
        obstacle_manager=manager,
        step_physics=lambda _client_id: events.append("step"),
    )

    def finish_immediately(action):
        events.append(type(action).__name__)
        return SimpleNamespace(world=world, state_changed=True, world_reset=False, status_message="done")

    monkeypatch.setattr(coordinator, "_apply_immediate_action", finish_immediately)
    coordinator.enqueue(SwitchRobotAction("df_mid"))
    coordinator.enqueue(DeleteObstacleAction(99))

    result = coordinator.step(0.01)

    assert getattr(result, "obstacle_result", None) is None
    assert coordinator.has_pending_action is True
    assert events == ["SwitchRobotAction", "moving", "step"]


def test_robot_switch_and_reset_preserve_obstacle_manager_and_bodies():
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        _restore_pair(manager)
        obstacle_body_ids = {snapshot.physics_body_id for snapshot in manager.snapshot()}
        coordinator = SimulationCoordinator(client_id, config, world, manager)

        switch_result = coordinator.apply_action(SwitchRobotAction("df_mid"))
        reset_result = coordinator.apply_action(ResetRobotAction())

        assert switch_result.state_changed is True
        assert reset_result.state_changed is True
        assert coordinator.obstacle_manager is manager
        assert {snapshot.physics_body_id for snapshot in manager.snapshot()} == obstacle_body_ids
        assert obstacle_body_ids <= _body_ids(client_id)
    finally:
        p.disconnect(client_id)


def test_coordinator_add_rechecks_current_robot_aabb_after_robot_switch(monkeypatch):
    """通过协调器添加障碍物时，提交前必须避开当前车辆 AABB。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        coordinator = SimulationCoordinator(client_id, config, world, manager)
        coordinator.apply_action(SwitchRobotAction("df_mid"))

        robot_id = coordinator.world.active_robot.robot.robot_id
        aabb_min, aabb_max = p.getAABB(robot_id, -1, physicsClientId=client_id)
        center = ((aabb_min[0] + aabb_max[0]) / 2.0, (aabb_min[1] + aabb_max[1]) / 2.0)

        def plan_on_robot(_settings, _request, terrain_sampler, **_kwargs):
            probe = terrain_sampler(center[0], center[1])
            geometry = ObstacleGeometry("box", (0.25, 0.25, 0.25))
            return (
                ObstacleSpec(
                    logical_id=1,
                    mode="static",
                    geometry=geometry,
                    position=(center[0], center[1], probe.local_ground_height + geometry.half_extents[2]),
                    orientation=(0.0, 0.0, 0.0, 1.0),
                ),
            )

        monkeypatch.setattr(obstacle_module, "plan_obstacle_batch", plan_on_robot)

        coordinator.enqueue(AddObstaclesAction(ObstacleGenerationRequest("static", 1, seed=20)))
        result = coordinator.step(config.time_step)
        guard = 0
        while result is not None and result.obstacle_result is not None and not result.obstacle_result.done:
            guard += 1
            assert guard < 10
            result = coordinator.step(config.time_step)

        assert result is not None
        assert result.obstacle_result is not None
        assert result.obstacle_result.done is True
        assert result.obstacle_result.succeeded is False
        assert "vehicle AABB" in result.obstacle_result.message
        assert coordinator.obstacle_manager.snapshot() == ()
    finally:
        p.disconnect(client_id)


def test_robot_switch_rolls_back_replacement_when_old_robot_removal_fails(monkeypatch):
    """旧车删除失败时不能提交新车型，也不能留下重复车辆。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        coordinator = SimulationCoordinator(client_id, config, world, manager)
        old_robot_id = world.active_robot.robot.robot_id
        before_ids = _body_ids(client_id)
        original_remove_body = coordinator_module.p.removeBody

        def fail_old_robot(body_id, **kwargs):
            if body_id == old_robot_id:
                raise RuntimeError("old robot stuck")
            return original_remove_body(body_id, **kwargs)

        monkeypatch.setattr(coordinator_module.p, "removeBody", fail_old_robot)

        result = coordinator.apply_action(SwitchRobotAction("df_mid"))

        assert result.state_changed is False
        assert result.world.active_robot.robot.robot_id == old_robot_id
        assert result.error_message is not None
        assert "old robot stuck" in result.error_message
        assert _body_ids(client_id) == before_ids
    finally:
        p.disconnect(client_id)


def test_terrain_switch_snapshots_obstacles_and_restores_identity_xy_and_path_state():
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_mid", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_mid")
        manager = _manager(client_id, world)
        _restore_pair(manager)
        before = manager.snapshot()
        coordinator = SimulationCoordinator(client_id, config, world, manager)

        result = coordinator.apply_action(SwitchTerrainAction(TerrainSelection("slope", slope_deg=6.0)))

        after = coordinator.obstacle_manager.snapshot()
        assert result.state_changed is True
        assert coordinator.world.terrain == TerrainSelection("slope", slope_deg=6.0)
        assert [item.logical_id for item in after] == [item.logical_id for item in before]
        assert [(item.position[0], item.position[1]) for item in after] == [
            (item.position[0], item.position[1]) for item in before
        ]
        assert after[1].path is not None
        assert before[1].path is not None
        assert after[1].path.progress == pytest.approx(before[1].path.progress)
        assert after[1].path.direction == before[1].path.direction
    finally:
        p.disconnect(client_id)


def test_target_terrain_cannot_contain_snapshot_rolls_back_world_robot_and_obstacles():
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        far_snapshot = ObstacleSnapshot(
            logical_id=7,
            body_id=None,
            mode="static",
            shape="box",
            position=(8.5, 0.0, 0.2),
            orientation=(0.0, 0.0, 0.0, 1.0),
            geometry=ObstacleGeometry("box", (0.2, 0.2, 0.2)),
        )
        assert manager.restore((far_snapshot,)).succeeded is True
        coordinator = SimulationCoordinator(client_id, config, world, manager)

        result = coordinator.apply_action(SwitchTerrainAction(TerrainSelection("golf_heightfield", golf_seed=3)))

        restored = coordinator.obstacle_manager.snapshot()
        assert result.state_changed is True
        assert result.world_reset is True
        assert result.error_message is not None
        assert coordinator.world.terrain == TerrainSelection("flat")
        assert coordinator.world.active_robot.robot_model == "df_back"
        assert [(item.logical_id, item.position[0], item.position[1]) for item in restored] == [(7, 8.5, 0.0)]
    finally:
        p.disconnect(client_id)


def test_target_failure_plus_rollback_failure_reports_both_reasons(monkeypatch):
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        coordinator = SimulationCoordinator(client_id, config, world, manager)

        def fail_all(_client_id, _config, terrain, _robot_model):
            if terrain.terrain_model == "slope":
                raise RuntimeError("target exploded")
            raise RuntimeError("rollback exploded")

        monkeypatch.setattr(coordinator_module, "load_manual_world", fail_all)

        with pytest.raises(RuntimeError) as excinfo:
            coordinator.apply_action(SwitchTerrainAction(TerrainSelection("slope", slope_deg=5.0)))

        assert "target exploded" in str(excinfo.value)
        assert "rollback exploded" in str(excinfo.value)
    finally:
        p.disconnect(client_id)


def test_pending_structural_task_still_updates_moving_obstacles_and_steps_physics():
    events: list[str] = []

    class PendingManager:
        def begin_add(self, _request):
            events.append("begin")
            return SimpleNamespace(done=False, succeeded=False, operation="add", message="pending")

        def advance_pending_operation(self):
            events.append("advance")
            return SimpleNamespace(done=False, succeeded=False, operation="add", message="pending")

        def update_moving(self, dt):
            events.append(f"moving:{dt:g}")

    world = SimpleNamespace(active_robot=SimpleNamespace(robot=SimpleNamespace(command_twist=lambda *_args, **_kwargs: None)))
    coordinator = SimulationCoordinator(
        client_id=11,
        config=SimpleNamespace(time_step=0.02),
        world=world,
        obstacle_manager=PendingManager(),
        step_physics=lambda client_id: events.append(f"step:{client_id}"),
    )
    coordinator.enqueue(AddObstaclesAction(ObstacleGenerationRequest("static", 1, seed=1)))

    coordinator.step(0.02)

    assert events == ["begin", "advance", "moving:0.02", "step:11"]


@pytest.mark.parametrize(
    ("action", "should_stop"),
    [
        (SwitchTerrainAction(TerrainSelection("slope", slope_deg=4.0)), True),
        (SwitchRobotAction("df_mid"), True),
        (ResetRobotAction(), True),
        (AddObstaclesAction(ObstacleGenerationRequest("static", 1, seed=10)), False),
        (DeleteObstacleAction(1), False),
        (ClearObstaclesAction(), False),
    ],
)
def test_coordinator_safe_stop_only_for_world_and_robot_rebuild_actions(monkeypatch, action, should_stop):
    """协调器只在重建车辆/场地前停车，障碍物增删清不能清零持续驾驶命令。"""
    stop_calls: list[tuple[float, float]] = []
    robot = SimpleNamespace(command_twist=lambda linear, angular, **_kwargs: stop_calls.append((linear, angular)))
    world = SimpleNamespace(active_robot=SimpleNamespace(robot=robot))
    manager = SimpleNamespace(update_moving=lambda _dt: None)
    coordinator = SimulationCoordinator(
        client_id=0,
        config=SimpleNamespace(time_step=0.01),
        world=world,
        obstacle_manager=manager,
    )

    def finish_immediately(_action):
        return SimpleNamespace(world=world, state_changed=True, world_reset=False, status_message="done")

    monkeypatch.setattr(coordinator, "_apply_immediate_action", finish_immediately)

    coordinator.apply_action(action)

    assert stop_calls == ([(0.0, 0.0)] if should_stop else [])


def test_rebuild_leaves_no_duplicate_terrain_robot_or_obstacle_bodies():
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        _restore_pair(manager)
        coordinator = SimulationCoordinator(client_id, config, world, manager)

        coordinator.apply_action(SwitchTerrainAction(TerrainSelection("slope", slope_deg=7.0)))

        terrain_body_ids = set(coordinator.world.scene.body_ids)
        robot_body_id = coordinator.world.active_robot.robot.robot_id
        obstacle_body_ids = {snapshot.physics_body_id for snapshot in coordinator.obstacle_manager.snapshot()}
        current_ids = _body_ids(client_id)
        expected_ids = terrain_body_ids | {robot_body_id} | obstacle_body_ids
        assert current_ids == expected_ids
        assert len(current_ids) == len(terrain_body_ids) + 1 + len(obstacle_body_ids)
    finally:
        p.disconnect(client_id)
