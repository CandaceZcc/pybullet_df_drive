# SceneDocument 场景事务集成测试：验证协调器与企业接口运行时的重建、回滚和动态绑定。
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import replace

import pybullet as p
import pytest

import slope_sim.coordinator as coordinator_module
from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import (
    SimulationCoordinator,
    build_world_from_scene_document,
    load_manual_world,
)
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.models import WheelCommand
from slope_sim.interfaces.runtime import InterfaceRuntime
from slope_sim.interfaces.transport import LocalTransport
from slope_sim.obstacles import (
    ObstacleGenerationRequest,
    ObstacleGenerationSettings,
    ObstacleGeometry,
    ObstacleManager,
    ObstacleOperationResult,
    ObstaclePath,
    ObstacleSnapshot,
    ObstacleSpec,
)
from slope_sim.runtime_actions import (
    AddObstaclesAction,
    ClearObstaclesAction,
    DeleteObstacleAction,
    LoadSceneAction,
    ResetRobotAction,
    SwitchRobotAction,
    SwitchTerrainAction,
    TerrainSelection,
)
from slope_sim.scene import TerrainBounds
from slope_sim.scene_config import SceneDocument, SensorDocument, TerrainDocument
from slope_sim.sensor_backend import PyBulletSensorBackend

from .test_interface_runtime_integration import Clock, Transport


def _body_ids(client_id: int) -> set[int]:
    return {
        p.getBodyUniqueId(index, physicsClientId=client_id)
        for index in range(p.getNumBodies(client_id))
    }


def _manager(
    client_id: int,
    world,
    *,
    soft_budget_seconds: float = 0.0,
) -> ObstacleManager:
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


def _obstacle_snapshots() -> tuple[ObstacleSnapshot, ObstacleSnapshot]:
    return (
        ObstacleSnapshot(
            logical_id=11,
            body_id=None,
            mode="static",
            shape="box",
            position=(-2.0, -1.0, 0.3),
            orientation=(0.0, 0.0, 0.0, 1.0),
            geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
        ),
        ObstacleSnapshot(
            logical_id=12,
            body_id=None,
            mode="moving",
            shape="sphere",
            position=(1.35, 0.5, 0.2),
            orientation=(0.0, 0.0, 0.0, 1.0),
            path=ObstaclePath(
                start_xy=(1.0, 0.5),
                end_xy=(2.0, 0.5),
                speed=0.4,
                progress=0.35,
                direction=-1,
            ),
            geometry=ObstacleGeometry("sphere", (0.2, 0.2, 0.2)),
        ),
    )


def _edge_obstacle_snapshots() -> tuple[ObstacleSnapshot, ObstacleSnapshot]:
    """构造 flat 合法且靠近 +X 边界的静态/移动障碍布局。"""
    return (
        ObstacleSnapshot(
            logical_id=31,
            body_id=None,
            mode="static",
            shape="box",
            position=(8.5, -1.0, 0.25),
            orientation=(0.0, 0.0, 0.0, 1.0),
            geometry=ObstacleGeometry("box", (0.2, 0.2, 0.25)),
        ),
        ObstacleSnapshot(
            logical_id=32,
            body_id=None,
            mode="moving",
            shape="sphere",
            position=(8.6, 1.0, 0.2),
            orientation=(0.0, 0.0, 0.0, 1.0),
            path=ObstaclePath(
                start_xy=(8.0, 1.0),
                end_xy=(9.5, 1.0),
                speed=0.4,
                progress=0.4,
                direction=-1,
            ),
            geometry=ObstacleGeometry("sphere", (0.2, 0.2, 0.2)),
        ),
    )


def _twenty_obstacle_snapshots() -> tuple[ObstacleSnapshot, ...]:
    """构造含 flat X 边界障碍的 20 个可恢复逻辑对象。"""
    interior = tuple(
        ObstacleSnapshot(
            logical_id=100 + row * 9 + column,
            body_id=None,
            mode="static",
            shape="box",
            position=(x, y, 0.2),
            orientation=(0.0, 0.0, 0.0, 1.0),
            geometry=ObstacleGeometry("box", (0.15, 0.15, 0.2)),
        )
        for row, y in enumerate((-3.0, 3.0))
        for column, x in enumerate((-7.2, -5.4, -3.6, -1.8, 0.0, 1.8, 3.6, 5.4, 7.2))
    )
    return (*interior, *_edge_obstacle_snapshots())


def _target_obstacles() -> tuple[ObstacleSpec, ObstacleSpec]:
    return (
        ObstacleSpec(
            logical_id=21,
            mode="static",
            geometry=ObstacleGeometry("cylinder", (0.25, 0.25, 0.35)),
            position=(-1.0, 1.0, 0.35),
            orientation=(0.0, 0.0, 0.0, 1.0),
        ),
        ObstacleSpec(
            logical_id=22,
            mode="moving",
            geometry=ObstacleGeometry("box", (0.2, 0.2, 0.25)),
            position=(1.5, -1.0, 0.25),
            orientation=(0.0, 0.0, 0.0, 1.0),
            path=ObstaclePath(
                start_xy=(1.0, -1.0),
                end_xy=(2.0, -1.0),
                speed=0.5,
                progress=0.5,
                direction=1,
            ),
        ),
    )


def _custom_sensors() -> SensorDocument:
    sensors = SensorDocument.default()
    mounts = replace(
        sensors.mounts,
        imu=replace(sensors.mounts.imu, position=(0.0, 0.0, 0.12)),
    )
    return SensorDocument(mounts, sensors.lidar)


def _runtime_document(
    world,
    sensors: SensorDocument,
    obstacles=(),
) -> SceneDocument:
    """从测试世界构造与 coordinator 对应的完整无 body-id 文档。"""
    return SceneDocument.from_runtime(
        world.active_robot.robot_model,
        world.terrain,
        obstacles,
        sensors.mounts,
        lidar_config=sensors.lidar,
    )


@contextmanager
def _real_runtime_coordinator(
    *,
    initial_robot_model: str = "df_back",
    initial_obstacles: tuple[ObstacleSnapshot, ...] = (),
    sensors: SensorDocument | None = None,
):
    """创建真实 DIRECT 世界和 InterfaceRuntime，并统一释放运行时资源。"""
    client_id = p.connect(p.DIRECT)
    runtime: InterfaceRuntime | None = None
    try:
        config = ExperimentConfig(
            mode="gui",
            robot_model=initial_robot_model,
            terrain_model="flat",
        )
        world = load_manual_world(
            client_id,
            config,
            TerrainSelection("flat"),
            initial_robot_model,
        )
        manager = _manager(client_id, world)
        if initial_obstacles:
            assert manager.restore(initial_obstacles).succeeded is True
        selected_sensors = SensorDocument.default() if sensors is None else sensors
        backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
        backend.bind_scene(world.scene.body_ids, manager.snapshot(include_body_id=True))
        runtime = InterfaceRuntime.local_for_robot(
            world.active_robot.robot,
            sensor_backend=backend,
            scene_document=_runtime_document(
                world,
                selected_sensors,
                manager.snapshot(include_body_id=False),
            ),
        )
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        yield client_id, config, coordinator, runtime
    finally:
        if runtime is not None:
            runtime.close()
        p.disconnect(client_id)


class RecordingRuntime:
    """记录协调器生命周期调用，可让首个 commit 定点失败。"""

    def __init__(self, document: SceneDocument, *, fail_commit_count: int = 0) -> None:
        self.scene_document = document
        self.bound_robot_id: int | None = None
        self.transport = object()
        self.prepare_calls = 0
        self.prepared = False
        self.faulted = False
        self.commit_calls: list[tuple[object, object, SceneDocument]] = []
        self.refresh_calls: list[tuple[tuple[int, ...], tuple[ObstacleSnapshot, ...]]] = []
        self._remaining_commit_failures = fail_commit_count

    def prepare_world_rebuild(self) -> None:
        self.prepare_calls += 1
        self.prepared = True

    def abort_world_rebuild(self) -> None:
        self.prepared = False

    def fault_world_rebuild(self) -> None:
        self.prepared = False
        self.faulted = True

    def commit_world_rebuild(self, robot, backend, document: SceneDocument) -> None:
        self.commit_calls.append((robot, backend, document))
        if self._remaining_commit_failures:
            self._remaining_commit_failures -= 1
            raise RuntimeError("runtime target commit failed")
        self.bound_robot_id = robot.robot_id
        self.scene_document = document
        self.prepared = False

    def refresh_scene_bindings(
        self,
        terrain_ids,
        snapshots,
        scene_document: SceneDocument | None = None,
    ) -> None:
        self.refresh_calls.append((tuple(terrain_ids), tuple(snapshots)))
        if scene_document is not None:
            self.scene_document = scene_document

    def update_scene_document(self, scene_document: SceneDocument) -> None:
        self.scene_document = scene_document


class StickyCloseSubscription:
    """close 抛错且仍可 emit，用于验证旧订阅 generation 隔离。"""

    def __init__(self, callback) -> None:
        self.callback = callback
        self.closed = False
        self.close_calls = 0

    def close(self) -> None:
        self.close_calls += 1
        self.closed = True
        raise RuntimeError("subscription close failed")


class StickyFirstSubscriptionTransport(Transport):
    """首个订阅无法关闭，恢复时创建的后续订阅正常。"""

    def __init__(self) -> None:
        super().__init__()
        self.sticky_subscription: StickyCloseSubscription | None = None

    def subscribe(self, topic: str, type_name: str, callback):
        if self.sticky_subscription is None:
            subscription = StickyCloseSubscription(callback)
            self.sticky_subscription = subscription
            self.subscriptions.append((topic, type_name, subscription))
            return subscription
        return super().subscribe(topic, type_name, callback)


class AbortSubscribeFailTransport(Transport):
    """初始订阅成功，但 abort 重建订阅时定点失败。"""

    def __init__(self) -> None:
        super().__init__()
        self.subscribe_calls = 0

    def subscribe(self, topic: str, type_name: str, callback):
        self.subscribe_calls += 1
        if self.subscribe_calls == 2:
            raise RuntimeError("abort subscription failed")
        return super().subscribe(topic, type_name, callback)


class RegisteredThenRaisedSubscription:
    """transport 已保存 callback，但 subscribe 调用方拿不到句柄。"""

    def __init__(self, callback) -> None:
        self.callback = callback
        self.closed = False

    def close(self) -> None:
        self.closed = True


class TargetCommitRegisterThenRaiseTransport(Transport):
    """目标 commit 注册第 2 个订阅后抛错，rollback 的第 3 个订阅成功。"""

    def __init__(self) -> None:
        super().__init__()
        self.subscribe_calls = 0
        self.uncertain_subscription: RegisteredThenRaisedSubscription | None = None

    def subscribe(self, topic: str, type_name: str, callback):
        self.subscribe_calls += 1
        if self.subscribe_calls == 2:
            subscription = RegisteredThenRaisedSubscription(callback)
            self.uncertain_subscription = subscription
            self.subscriptions.append((topic, type_name, subscription))
            raise RuntimeError("target subscribe registered then failed")
        return super().subscribe(topic, type_name, callback)


class TargetCommitRegisterThenRaiseLocalTransport(LocalTransport):
    """真实本地传输保留失败候选，用于验证 stale 回调为聚合中性。"""

    def __init__(self) -> None:
        super().__init__()
        self.subscribe_calls = 0

    def subscribe(self, topic: str, type_name: str, callback):
        self.subscribe_calls += 1
        subscription = super().subscribe(topic, type_name, callback)
        if self.subscribe_calls == 2:
            raise RuntimeError("target local subscribe registered then failed")
        return subscription


def _coordinator_with_transport(client_id: int, transport: Transport):
    """创建绑定真实 PyBullet 车辆、可注入订阅故障的 runtime coordinator。"""
    config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
    world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
    manager = _manager(client_id, world)
    backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
    backend.bind_scene(world.scene.body_ids, ())
    clock = Clock()
    runtime = InterfaceRuntime(
        world.active_robot.robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
        sensor_backend=backend,
        scene_document=_runtime_document(world, SensorDocument.default()),
    )
    coordinator = SimulationCoordinator(
        client_id,
        config,
        world,
        manager,
        interface_runtime=runtime,
    )
    return config, world, manager, clock, runtime, coordinator


def _assert_single_world_partition(client_id: int, coordinator: SimulationCoordinator) -> None:
    obstacle_ids = {
        snapshot.physics_body_id
        for snapshot in coordinator.obstacle_manager.snapshot(include_body_id=True)
    }
    assert None not in obstacle_ids
    expected = (
        set(coordinator.world.scene.body_ids)
        | {coordinator.world.active_robot.robot.robot_id}
        | obstacle_ids
    )
    assert _body_ids(client_id) == expected


def _assert_runtime_categories(runtime: InterfaceRuntime, coordinator: SimulationCoordinator) -> None:
    expected = {body_id: "terrain" for body_id in coordinator.world.scene.body_ids}
    for snapshot in coordinator.obstacle_manager.snapshot(include_body_id=True):
        expected[snapshot.physics_body_id] = (
            "moving_obstacle" if snapshot.mode == "moving" else "static_obstacle"
        )
    assert runtime._sensor_backend._hit_categories == expected


def test_initial_scene_restore_failure_removes_every_new_body(monkeypatch) -> None:
    """首次恢复障碍物失败时不能留下 terrain、robot 或半成品障碍 body。"""
    client_id = p.connect(p.DIRECT)
    try:
        before = _body_ids(client_id)
        sensors = SensorDocument.default()
        document = SceneDocument.from_runtime(
            "df_back",
            TerrainSelection("flat"),
            (),
            sensors.mounts,
            lidar_config=sensors.lidar,
        )

        def fail_after_creating_body(self, _snapshots):
            p.createMultiBody(
                baseMass=0.0,
                basePosition=(2.0, 0.0, 0.2),
                physicsClientId=self._client_id,
            )
            return ObstacleOperationResult(
                done=True,
                succeeded=False,
                operation="restore",
                message="injected initial restore failure",
            )

        monkeypatch.setattr(ObstacleManager, "restore", fail_after_creating_body)

        with pytest.raises(RuntimeError, match="initial restore failure"):
            build_world_from_scene_document(
                client_id,
                ExperimentConfig(mode="direct", interface_enabled=False),
                document,
            )

        assert _body_ids(client_id) == before
    finally:
        p.disconnect(client_id)


def test_interface_disabled_scene_load_never_constructs_sensor_backend(monkeypatch) -> None:
    """禁用接口后的场景重建也只能创建物理 world，不能偷偷创建传感器后端。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(
            mode="direct",
            robot_model="df_back",
            terrain_model="flat",
            interface_enabled=False,
        )
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        coordinator = SimulationCoordinator(client_id, config, world, manager)
        sensors = SensorDocument.default()
        target = SceneDocument.from_runtime(
            "df_mid",
            TerrainSelection("flat"),
            (),
            sensors.mounts,
            lidar_config=sensors.lidar,
        )

        monkeypatch.setattr(
            coordinator_module,
            "PyBulletSensorBackend",
            lambda *_args, **_kwargs: (_ for _ in ()).throw(
                AssertionError("disabled coordinator constructed a sensor backend")
            ),
        )

        result = coordinator.apply_scene_document(target)

        assert result.error_message is None
        assert coordinator.world.active_robot.robot_model == "df_mid"
        assert coordinator.interface_runtime is None
    finally:
        p.disconnect(client_id)


def test_logical_scene_export_tracks_latest_moving_state_without_body_ids() -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        assert manager.restore(_obstacle_snapshots()).succeeded is True
        coordinator = SimulationCoordinator(client_id, config, world, manager)

        before = coordinator.logical_scene_document()
        manager.update_moving(0.5)
        after = coordinator.logical_scene_document()

        assert before.obstacles[1].path is not None
        assert after.obstacles[1].path is not None
        assert after.obstacles[1].path.progress != before.obstacles[1].path.progress
        assert after.obstacles[1].position != before.obstacles[1].position
        assert all(not hasattr(obstacle, "body_id") for obstacle in after.obstacles)
    finally:
        p.disconnect(client_id)


def test_load_scene_rebuilds_all_domains_and_preserves_runtime_transport() -> None:
    with _real_runtime_coordinator() as (client_id, config, coordinator, runtime):
        transport = runtime._transport
        assert runtime.accept_local_command(WheelCommand(1, (1.0, 1.0), ()))
        decision = runtime.before_physics_step(config.time_step)
        assert decision is not None and not decision.waiting
        target = SceneDocument(
            1,
            "df_mid",
            TerrainDocument("slope", 6.0, 17, "high"),
            _target_obstacles(),
            _custom_sensors(),
        )

        result = coordinator.apply_action(LoadSceneAction(target))

        logical = coordinator.logical_scene_document()
        assert result.state_changed is True
        assert result.world_reset is True
        assert result.error_message is None
        assert logical.robot_model == "df_mid"
        assert logical.terrain == target.terrain
        assert logical.sensors == target.sensors
        assert [item.logical_id for item in logical.obstacles] == [21, 22]
        assert logical.obstacles[1].path is not None
        assert logical.obstacles[1].path.progress == pytest.approx(0.5)
        assert runtime._transport is transport
        assert runtime.bound_robot_id == coordinator.world.active_robot.robot.robot_id
        assert runtime.last_decision.waiting is True
        assert runtime.scene_document is not None
        assert runtime.scene_document.robot_model == target.robot_model
        assert runtime.scene_document.terrain == target.terrain
        assert runtime.scene_document.sensors == target.sensors
        assert runtime.scene_document == logical
        _assert_runtime_categories(runtime, coordinator)
        _assert_single_world_partition(client_id, coordinator)


def test_flat_edge_layout_switches_to_golf_and_commits_runtime_document() -> None:
    """带接口运行时切换 golf 时，边界障碍布局不能触发 flat 回滚。"""
    with _real_runtime_coordinator(
        initial_obstacles=_edge_obstacle_snapshots(),
    ) as (client_id, _config, coordinator, runtime):
        before = coordinator.logical_scene_document()

        result = coordinator.apply_action(
            SwitchTerrainAction(
                TerrainSelection("golf_heightfield", golf_seed=13, golf_relief="high")
            )
        )

        logical = coordinator.logical_scene_document()
        assert result.state_changed is True
        assert result.world_reset is True
        assert result.error_message is None
        assert logical.terrain == TerrainDocument("golf_heightfield", 0.0, 13, "high")
        assert runtime.scene_document == logical
        assert [item.logical_id for item in logical.obstacles] == [31, 32]
        assert [(item.position[0], item.position[1]) for item in logical.obstacles] == [
            (item.position[0], item.position[1]) for item in before.obstacles
        ]
        assert logical.obstacles[1].path == before.obstacles[1].path
        _assert_runtime_categories(runtime, coordinator)
        _assert_single_world_partition(client_id, coordinator)


@pytest.mark.parametrize(
    "initial_obstacles",
    ((), _twenty_obstacle_snapshots()),
    ids=("empty", "twenty-retained"),
)
def test_active_steering_four_wheel_switches_from_flat_to_golf_without_rollback(
    initial_obstacles: tuple[ObstacleSnapshot, ...],
) -> None:
    """精确覆盖 GUI 反馈：四驱车型切高尔夫后不得回弹到 flat。"""
    with _real_runtime_coordinator(
        initial_robot_model="active_steering_4wd",
        initial_obstacles=initial_obstacles,
    ) as (client_id, _config, coordinator, runtime):
        before = coordinator.logical_scene_document()
        transport = runtime._transport

        result = coordinator.apply_action(
            SwitchTerrainAction(
                TerrainSelection(
                    "golf_heightfield",
                    golf_seed=73,
                    golf_relief="medium",
                )
            )
        )

        logical = coordinator.logical_scene_document()
        assert result.state_changed is True
        assert result.world_reset is True
        assert result.error_message is None
        assert coordinator.world.active_robot.robot_model == "active_steering_4wd"
        assert coordinator.world.terrain == TerrainSelection(
            "golf_heightfield",
            golf_seed=73,
            golf_relief="medium",
        )
        assert logical.robot_model == "active_steering_4wd"
        assert logical.terrain == TerrainDocument(
            "golf_heightfield",
            0.0,
            73,
            "medium",
        )
        assert runtime._transport is transport
        assert runtime.scene_document == logical
        assert runtime.bound_robot_id == coordinator.world.active_robot.robot.robot_id
        assert [
            (
                item.logical_id,
                item.mode,
                item.geometry,
                item.position[:2],
                item.path,
            )
            for item in logical.obstacles
        ] == [
            (
                item.logical_id,
                item.mode,
                item.geometry,
                item.position[:2],
                item.path,
            )
            for item in before.obstacles
        ]
        _assert_runtime_categories(runtime, coordinator)
        _assert_single_world_partition(client_id, coordinator)


def test_target_restore_failure_rebuilds_previous_scene_and_runtime(monkeypatch) -> None:
    sensors = _custom_sensors()
    with _real_runtime_coordinator(
        initial_obstacles=_obstacle_snapshots(),
        sensors=sensors,
    ) as (client_id, _config, coordinator, runtime):
        previous = coordinator.logical_scene_document()
        transport = runtime._transport
        original_restore = ObstacleManager.restore
        restore_calls = 0

        def fail_target_restore(manager, snapshots):
            nonlocal restore_calls
            restore_calls += 1
            if restore_calls == 1:
                raise RuntimeError("target restore failed")
            return original_restore(manager, snapshots)

        monkeypatch.setattr(ObstacleManager, "restore", fail_target_restore)
        target = SceneDocument(
            1,
            "active_steering_4wd",
            TerrainDocument("slope", 5.0, 9, "medium"),
            _target_obstacles(),
            SensorDocument.default(),
        )

        result = coordinator.apply_scene_document(target)

        assert result.state_changed is True
        assert result.world_reset is True
        assert result.error_message is not None
        assert "target restore failed" in result.error_message
        assert coordinator.logical_scene_document() == previous
        assert runtime._transport is transport
        assert runtime.bound_robot_id == coordinator.world.active_robot.robot.robot_id
        assert runtime.scene_document is not None
        assert runtime.scene_document == previous
        _assert_runtime_categories(runtime, coordinator)
        _assert_single_world_partition(client_id, coordinator)


def test_runtime_commit_failure_rolls_back_and_commits_recovered_binding() -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        assert manager.restore(_obstacle_snapshots()).succeeded is True
        sensors = _custom_sensors()
        runtime = RecordingRuntime(
            _runtime_document(
                world,
                sensors,
                manager.snapshot(include_body_id=False),
            ),
            fail_commit_count=1,
        )
        runtime.bound_robot_id = world.active_robot.robot.robot_id
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        previous = coordinator.logical_scene_document()
        target = SceneDocument(
            1,
            "df_mid",
            TerrainDocument("slope", 4.0, 3, "low"),
            _target_obstacles(),
            SensorDocument.default(),
        )

        result = coordinator.apply_scene_document(target)

        assert result.state_changed is True
        assert result.world_reset is True
        assert result.error_message is not None
        assert "runtime target commit failed" in result.error_message
        assert runtime.prepare_calls == 1
        assert len(runtime.commit_calls) == 2
        assert runtime.bound_robot_id == coordinator.world.active_robot.robot.robot_id
        assert runtime.scene_document == previous
        assert coordinator.logical_scene_document() == previous
        _assert_single_world_partition(client_id, coordinator)
    finally:
        p.disconnect(client_id)


def test_target_failure_without_message_still_returns_nonempty_error(monkeypatch) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        coordinator = SimulationCoordinator(client_id, config, world, manager)
        original_restore = ObstacleManager.restore
        restore_calls = 0

        def fail_without_message(target_manager, snapshots):
            nonlocal restore_calls
            restore_calls += 1
            if restore_calls == 1:
                raise RuntimeError()
            return original_restore(target_manager, snapshots)

        monkeypatch.setattr(ObstacleManager, "restore", fail_without_message)
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 2.0, 0, "medium"),
        )

        result = coordinator.apply_scene_document(target)

        assert result.error_message
        assert "RuntimeError" in result.error_message
    finally:
        p.disconnect(client_id)


def test_scene_factory_post_create_failure_cleans_target_before_rollback(
    monkeypatch,
) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        coordinator = SimulationCoordinator(client_id, config, world, manager)
        original_create_scene = coordinator_module.create_slope_scene
        create_calls = 0
        bodies_before_rollback: set[int] | None = None

        def fail_first_after_create(*args, **kwargs):
            nonlocal bodies_before_rollback, create_calls
            if create_calls == 1:
                bodies_before_rollback = _body_ids(client_id)
            scene = original_create_scene(*args, **kwargs)
            create_calls += 1
            if create_calls == 1:
                raise RuntimeError("target failed after terrain creation")
            return scene

        monkeypatch.setattr(
            coordinator_module,
            "create_slope_scene",
            fail_first_after_create,
        )
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 4.0, 0, "medium"),
        )

        result = coordinator.apply_scene_document(target)

        assert result.error_message == "target failed after terrain creation"
        expected = (
            set(coordinator.world.scene.body_ids)
            | {coordinator.world.active_robot.robot.robot_id}
        )
        assert bodies_before_rollback == set()
        assert _body_ids(client_id) == expected
    finally:
        p.disconnect(client_id)


def test_scene_factory_post_create_target_and_rollback_failures_leave_no_new_bodies(
    monkeypatch,
) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        coordinator = SimulationCoordinator(client_id, config, world, manager)
        original_create_scene = coordinator_module.create_slope_scene
        create_calls = 0

        def fail_every_time_after_create(*args, **kwargs):
            nonlocal create_calls
            original_create_scene(*args, **kwargs)
            create_calls += 1
            reason = "target post-create" if create_calls == 1 else "rollback post-create"
            raise RuntimeError(reason)

        monkeypatch.setattr(
            coordinator_module,
            "create_slope_scene",
            fail_every_time_after_create,
        )
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 4.0, 0, "medium"),
        )

        with pytest.raises(RuntimeError) as excinfo:
            coordinator.apply_scene_document(target)

        assert "target post-create" in str(excinfo.value)
        assert "rollback post-create" in str(excinfo.value)
        assert _body_ids(client_id) == set()
    finally:
        p.disconnect(client_id)


def test_build_scene_failure_body_diff_preserves_preexisting_unrelated_body(
    monkeypatch,
) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        coordinator = SimulationCoordinator(client_id, config, world, manager)
        unrelated_body_id = p.createMultiBody(
            baseMass=0.0,
            basePosition=(100.0, 100.0, 100.0),
            physicsClientId=client_id,
        )
        create_calls = 0

        def create_one_body_then_fail(client_id: int, *_args, **_kwargs):
            nonlocal create_calls
            p.createMultiBody(
                baseMass=0.0,
                basePosition=(float(create_calls), 0.0, 10.0),
                physicsClientId=client_id,
            )
            create_calls += 1
            reason = "target body leaked" if create_calls == 1 else "rollback body leaked"
            raise RuntimeError(reason)

        monkeypatch.setattr(
            coordinator_module,
            "create_slope_scene",
            create_one_body_then_fail,
        )
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 4.0, 0, "medium"),
        )

        with pytest.raises(RuntimeError):
            coordinator.apply_scene_document(target)

        assert _body_ids(client_id) == {unrelated_body_id}
    finally:
        p.disconnect(client_id)


def test_target_and_rollback_failure_raise_both_reasons(monkeypatch) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        runtime = RecordingRuntime(_runtime_document(world, SensorDocument.default()))
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        load_calls = 0

        def fail_target_and_rollback(*_args, **_kwargs):
            nonlocal load_calls
            load_calls += 1
            reason = "target build failed" if load_calls == 1 else "rollback build failed"
            raise RuntimeError(reason)

        monkeypatch.setattr(coordinator_module, "load_manual_world", fail_target_and_rollback)
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 5.0, 1, "medium"),
        )

        with pytest.raises(RuntimeError) as excinfo:
            coordinator.apply_scene_document(target)

        assert "target build failed" in str(excinfo.value)
        assert "rollback build failed" in str(excinfo.value)
        assert runtime.prepare_calls == 1
        assert runtime.commit_calls == []
    finally:
        p.disconnect(client_id)


def test_double_scene_failure_then_real_runtime_close_does_not_repark_removed_robot(
    monkeypatch,
) -> None:
    client_id = p.connect(p.DIRECT)
    runtime: InterfaceRuntime | None = None
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
        backend.bind_scene(world.scene.body_ids, ())
        runtime = InterfaceRuntime.local_for_robot(
            world.active_robot.robot,
            sensor_backend=backend,
            scene_document=_runtime_document(world, SensorDocument.default()),
        )
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        load_calls = 0

        def fail_target_and_rollback(*_args, **_kwargs):
            nonlocal load_calls
            load_calls += 1
            reason = "target world failed" if load_calls == 1 else "rollback world failed"
            raise RuntimeError(reason)

        monkeypatch.setattr(coordinator_module, "load_manual_world", fail_target_and_rollback)
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 5.0, 0, "medium"),
        )

        with pytest.raises(RuntimeError) as excinfo:
            coordinator.apply_scene_document(target)

        assert "target world failed" in str(excinfo.value)
        assert "rollback world failed" in str(excinfo.value)
        runtime.close()
        assert runtime.close_trace == (
            "stop_commands",
            "safe_stop",
            "stop_sensors",
            "quiesce_transport",
            "close_log",
            "close_transport",
            "close_sensors",
        )
    finally:
        if runtime is not None:
            try:
                runtime.close()
            except RuntimeError:
                pass
        p.disconnect(client_id)


def test_prepare_subscription_close_failure_aborts_without_old_callback_leak() -> None:
    client_id = p.connect(p.DIRECT)
    runtime: InterfaceRuntime | None = None
    try:
        transport = StickyFirstSubscriptionTransport()
        config, world, manager, clock, runtime, coordinator = _coordinator_with_transport(
            client_id,
            transport,
        )
        before_ids = _body_ids(client_id)
        before_document = coordinator.logical_scene_document()

        result = coordinator.apply_action(
            SwitchTerrainAction(TerrainSelection("slope", slope_deg=4.0))
        )

        assert result.state_changed is False
        assert result.world_reset is False
        assert result.error_message == "subscription close failed"
        assert coordinator.world is world
        assert coordinator.obstacle_manager is manager
        assert coordinator.logical_scene_document() == before_document
        assert _body_ids(client_id) == before_ids
        sticky = transport.sticky_subscription
        assert sticky is not None and sticky.closed is True
        sticky.closed = False
        payload = ProtoCodec().encode(WheelCommand(101, (1.0, 1.0), ()))
        before_count = runtime.status_snapshot(wall_time=clock()).command.valid_count
        assert sticky.callback(payload, clock()) is None
        assert runtime.status_snapshot(wall_time=clock()).command.valid_count == before_count
        recovered_subscription = transport.subscriptions[-1][2]
        assert recovered_subscription is not sticky
        assert recovered_subscription.callback(payload, clock()) is True
        decision = runtime.before_physics_step(config.time_step, wall_time=clock())
        assert decision is not None and decision.waiting is False
        assert runtime.status_snapshot(wall_time=clock()).command.valid_count == before_count + 1
        runtime.close()
        runtime.close()
    finally:
        if runtime is not None:
            runtime.close()
        p.disconnect(client_id)


def test_prepare_parking_failure_aborts_and_old_world_remains_operational(
    monkeypatch,
) -> None:
    client_id = p.connect(p.DIRECT)
    runtime: InterfaceRuntime | None = None
    try:
        config, world, manager, clock, runtime, coordinator = _coordinator_with_transport(
            client_id,
            Transport(),
        )
        before_ids = _body_ids(client_id)
        before_document = coordinator.logical_scene_document()
        robot = world.active_robot.robot
        original_park = robot.hold_current_steering_and_stop_drive
        park_calls = 0

        def fail_first_parking(dt: float) -> None:
            nonlocal park_calls
            park_calls += 1
            if park_calls == 1:
                raise RuntimeError("prepare parking failed")
            original_park(dt)

        monkeypatch.setattr(robot, "hold_current_steering_and_stop_drive", fail_first_parking)

        result = coordinator.apply_action(SwitchRobotAction("df_mid"))

        assert result.state_changed is False
        assert result.world_reset is False
        assert result.error_message == "prepare parking failed"
        assert coordinator.world is world
        assert coordinator.obstacle_manager is manager
        assert coordinator.logical_scene_document() == before_document
        assert _body_ids(client_id) == before_ids
        assert runtime.accept_local_command(WheelCommand(102, (1.0, 1.0), ())) is True
        decision = runtime.before_physics_step(config.time_step, wall_time=clock())
        assert decision is not None and decision.waiting is False
        runtime.close()
        runtime.close()
    finally:
        if runtime is not None:
            runtime.close()
        p.disconnect(client_id)


def test_prepare_and_abort_failure_faults_runtime_but_keeps_close_available(
    monkeypatch,
) -> None:
    client_id = p.connect(p.DIRECT)
    runtime: InterfaceRuntime | None = None
    try:
        config, world, manager, clock, runtime, coordinator = _coordinator_with_transport(
            client_id,
            AbortSubscribeFailTransport(),
        )
        before_ids = _body_ids(client_id)
        before_document = coordinator.logical_scene_document()
        robot = world.active_robot.robot
        original_park = robot.hold_current_steering_and_stop_drive
        park_calls = 0

        def fail_first_parking(dt: float) -> None:
            nonlocal park_calls
            park_calls += 1
            if park_calls == 1:
                raise RuntimeError("prepare parking failed")
            original_park(dt)

        monkeypatch.setattr(robot, "hold_current_steering_and_stop_drive", fail_first_parking)

        with pytest.raises(RuntimeError) as excinfo:
            coordinator.apply_action(ResetRobotAction())

        assert "prepare parking failed" in str(excinfo.value)
        assert "abort subscription failed" in str(excinfo.value)
        assert coordinator.world is world
        assert coordinator.obstacle_manager is manager
        assert coordinator.logical_scene_document() == before_document
        assert _body_ids(client_id) == before_ids
        assert runtime.status_snapshot(wall_time=clock()).command.state == "disconnected"
        with pytest.raises(RuntimeError, match="faulted"):
            runtime.before_physics_step(config.time_step, wall_time=clock())
        runtime.close()
        runtime.close()
    finally:
        if runtime is not None:
            runtime.close()
        p.disconnect(client_id)


def test_failed_target_commit_subscription_cannot_enter_rollback_mailbox() -> None:
    client_id = p.connect(p.DIRECT)
    runtime: InterfaceRuntime | None = None
    try:
        transport = TargetCommitRegisterThenRaiseTransport()
        config, _world, _manager, clock, runtime, coordinator = _coordinator_with_transport(
            client_id,
            transport,
        )
        previous = coordinator.logical_scene_document()
        target = replace(
            previous,
            robot_model="df_mid",
            terrain=TerrainDocument("slope", 4.0, 0, "medium"),
        )

        result = coordinator.apply_action(LoadSceneAction(target))

        assert result.state_changed is True
        assert result.world_reset is True
        assert result.error_message == "target subscribe registered then failed"
        assert coordinator.logical_scene_document() == previous
        uncertain = transport.uncertain_subscription
        assert uncertain is not None and uncertain.closed is False
        recovered = transport.subscriptions[-1][2]
        assert recovered is not uncertain
        payload = ProtoCodec().encode(WheelCommand(201, (1.0, 1.0), ()))
        before_count = runtime.status_snapshot(wall_time=clock()).command.valid_count
        assert uncertain.callback(payload, clock()) is None
        assert runtime.status_snapshot(wall_time=clock()).command.valid_count == before_count
        assert recovered.callback(payload, clock()) is True
        decision = runtime.before_physics_step(config.time_step, wall_time=clock())
        assert decision is not None and decision.waiting is False
        assert runtime.status_snapshot(wall_time=clock()).command.valid_count == before_count + 1
    finally:
        if runtime is not None:
            runtime.close()
        p.disconnect(client_id)


def test_local_transport_stale_target_callback_is_neutral_after_rollback() -> None:
    client_id = p.connect(p.DIRECT)
    runtime: InterfaceRuntime | None = None
    try:
        transport = TargetCommitRegisterThenRaiseLocalTransport()
        config, _world, _manager, clock, runtime, coordinator = _coordinator_with_transport(
            client_id,
            transport,
        )
        previous = coordinator.logical_scene_document()
        target = replace(
            previous,
            robot_model="df_mid",
            terrain=TerrainDocument("slope", 4.0, 0, "medium"),
        )

        result = coordinator.apply_action(LoadSceneAction(target))

        assert result.error_message == "target local subscribe registered then failed"
        assert coordinator.logical_scene_document() == previous
        command = WheelCommand(202, (1.0, 1.0), ())
        payload = ProtoCodec().encode(command)
        before_count = runtime.status_snapshot(wall_time=clock()).command.valid_count
        assert transport.publish(
            runtime.config.wheel_command.topic,
            payload,
            ProtoCodec().type_name(command),
            command.timestamp_ns,
            wall_time=clock(),
        ) is True
        assert runtime.status_snapshot(wall_time=clock()).command.valid_count == before_count + 1
    finally:
        if runtime is not None:
            runtime.close()
        p.disconnect(client_id)


def test_partial_active_world_removal_failure_rebuilds_previous_without_duplicates(
    monkeypatch,
) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        runtime = RecordingRuntime(_runtime_document(world, SensorDocument.default()))
        runtime.bound_robot_id = world.active_robot.robot.robot_id
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        previous = coordinator.logical_scene_document()
        old_ids = _body_ids(client_id)
        failed_body_id = world.scene.body_ids[0]
        original_remove = p.removeBody
        failures = 0

        def fail_terrain_once(body_id: int, **kwargs) -> None:
            nonlocal failures
            if body_id == failed_body_id and failures == 0:
                failures += 1
                raise RuntimeError("old terrain removal failed once")
            original_remove(body_id, **kwargs)

        monkeypatch.setattr(coordinator_module.p, "removeBody", fail_terrain_once)
        target = replace(
            previous,
            robot_model="df_mid",
            terrain=TerrainDocument("slope", 4.0, 0, "medium"),
        )

        result = coordinator.apply_scene_document(target)

        assert result.error_message is not None
        assert "old terrain removal failed once" in result.error_message
        assert coordinator.logical_scene_document() == previous
        assert runtime.scene_document == previous
        assert runtime.prepared is False
        assert runtime.faulted is False
        assert len(runtime.commit_calls) == 1
        assert len(_body_ids(client_id)) == len(old_ids)
        _assert_single_world_partition(client_id, coordinator)
    finally:
        p.disconnect(client_id)


def test_active_body_enumeration_failure_aborts_prepared_runtime(monkeypatch) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        runtime = RecordingRuntime(_runtime_document(world, SensorDocument.default()))
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        before_ids = _body_ids(client_id)
        original_snapshot = manager.snapshot

        def fail_physical_snapshot(*, include_body_id=True):
            if include_body_id:
                raise RuntimeError("active body enumeration failed")
            return original_snapshot(include_body_id=False)

        monkeypatch.setattr(manager, "snapshot", fail_physical_snapshot)
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 2.0, 0, "medium"),
        )

        result = coordinator.apply_scene_document(target)

        assert result.state_changed is False
        assert result.error_message is not None
        assert "active body enumeration failed" in result.error_message
        assert runtime.prepared is False
        assert runtime.faulted is False
        assert _body_ids(client_id) == before_ids
    finally:
        p.disconnect(client_id)


def test_removal_failure_plus_diagnostic_failure_faults_prepared_runtime(
    monkeypatch,
) -> None:
    """删除已开始后若无法判断部分删除，必须故障收口并同时报告两次异常。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        runtime = RecordingRuntime(_runtime_document(world, SensorDocument.default()))
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        original_current_body_ids = coordinator_module._current_body_ids
        removal_started = False

        def fail_active_removal(_body_id: int, **_kwargs) -> None:
            nonlocal removal_started
            removal_started = True
            raise RuntimeError("primary active removal failure")

        def fail_removal_diagnosis(current_client_id: int) -> set[int]:
            if removal_started:
                raise RuntimeError("secondary body diagnosis failure")
            return original_current_body_ids(current_client_id)

        monkeypatch.setattr(coordinator_module.p, "removeBody", fail_active_removal)
        monkeypatch.setattr(
            coordinator_module,
            "_current_body_ids",
            fail_removal_diagnosis,
        )
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 3.0, 0, "medium"),
        )

        with pytest.raises(RuntimeError) as excinfo:
            coordinator.apply_scene_document(target)

        assert "primary active removal failure" in str(excinfo.value)
        assert "secondary body diagnosis failure" in str(excinfo.value)
        assert runtime.prepared is False
        assert runtime.faulted is True
        assert runtime.commit_calls == []
    finally:
        p.disconnect(client_id)


def test_permanent_partial_active_world_removal_failure_faults_and_raises(
    monkeypatch,
) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        runtime = RecordingRuntime(_runtime_document(world, SensorDocument.default()))
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        before_ids = _body_ids(client_id)
        failed_body_id = world.scene.body_ids[0]
        original_remove = p.removeBody

        def fail_terrain_forever(body_id: int, **kwargs) -> None:
            if body_id == failed_body_id:
                raise RuntimeError("old terrain permanently stuck")
            original_remove(body_id, **kwargs)

        monkeypatch.setattr(coordinator_module.p, "removeBody", fail_terrain_forever)
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 3.0, 0, "medium"),
        )

        with pytest.raises(RuntimeError, match="old terrain permanently stuck"):
            coordinator.apply_scene_document(target)

        assert runtime.prepared is False
        assert runtime.faulted is True
        assert runtime.commit_calls == []
        assert _body_ids(client_id) < before_ids
        assert failed_body_id in _body_ids(client_id)
    finally:
        p.disconnect(client_id)


def test_corrupted_frozen_document_is_rejected_before_prepare_or_delete(monkeypatch) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        runtime_document = _runtime_document(world, SensorDocument.default())
        runtime = RecordingRuntime(runtime_document)
        runtime.bound_robot_id = world.active_robot.robot.robot_id
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        before_ids = _body_ids(client_id)
        corrupted = replace(runtime_document)
        object.__setattr__(corrupted, "robot_model", "corrupted-model")
        monkeypatch.setattr(
            coordinator,
            "_remove_active_world_strict",
            lambda *_args, **_kwargs: pytest.fail("prevalidation must precede deletion"),
        )

        with pytest.raises(ValueError, match="robot_model"):
            coordinator.apply_scene_document(corrupted)

        assert runtime.prepare_calls == 0
        assert runtime.commit_calls == []
        assert runtime.scene_document is runtime_document
        assert coordinator.world is world
        assert coordinator.obstacle_manager is manager
        assert _body_ids(client_id) == before_ids
    finally:
        p.disconnect(client_id)


def test_coordinator_rejects_sensor_document_that_disagrees_with_runtime() -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        runtime_document = _runtime_document(world, SensorDocument.default())
        runtime = RecordingRuntime(runtime_document)
        before_ids = _body_ids(client_id)
        before_aabb_getter = manager._vehicle_aabb_getter

        with pytest.raises(ValueError, match="sensor_document.*runtime"):
            SimulationCoordinator(
                client_id,
                config,
                world,
                manager,
                interface_runtime=runtime,
                sensor_document=_custom_sensors(),
            )

        assert runtime.prepare_calls == 0
        assert runtime.scene_document is runtime_document
        assert manager._vehicle_aabb_getter is before_aabb_getter
        assert _body_ids(client_id) == before_ids
    finally:
        p.disconnect(client_id)


def test_coordinator_rejects_runtime_document_that_disagrees_with_obstacles() -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        runtime_document = replace(
            _runtime_document(world, SensorDocument.default()),
            obstacles=_target_obstacles(),
        )
        runtime = RecordingRuntime(runtime_document)
        before_ids = _body_ids(client_id)
        before_aabb_getter = manager._vehicle_aabb_getter

        with pytest.raises(ValueError, match="runtime scene_document.*logical scene"):
            SimulationCoordinator(
                client_id,
                config,
                world,
                manager,
                interface_runtime=runtime,
            )

        assert runtime.prepare_calls == 0
        assert runtime.scene_document is runtime_document
        assert manager._vehicle_aabb_getter is before_aabb_getter
        assert _body_ids(client_id) == before_ids
    finally:
        p.disconnect(client_id)


def test_coordinator_accepts_matching_explicit_and_runtime_sensor_documents() -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        sensors = _custom_sensors()
        runtime = RecordingRuntime(_runtime_document(world, sensors))

        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
            sensor_document=SensorDocument(sensors.mounts, sensors.lidar),
        )

        assert coordinator.sensor_document == sensors
        assert coordinator.logical_scene_document().sensors == sensors
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("action_name", ("robot", "reset", "terrain", "load"))
def test_interface_rebuild_actions_use_prepare_commit_without_command_twist(
    monkeypatch,
    action_name: str,
) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        runtime = RecordingRuntime(_runtime_document(world, SensorDocument.default()))
        runtime.bound_robot_id = world.active_robot.robot.robot_id
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        twist_calls: list[tuple[object, ...]] = []
        monkeypatch.setattr(
            world.active_robot.robot,
            "command_twist",
            lambda *args, **_kwargs: twist_calls.append(args),
        )
        if action_name == "robot":
            action = SwitchRobotAction("df_mid")
        elif action_name == "reset":
            action = ResetRobotAction()
        elif action_name == "terrain":
            action = SwitchTerrainAction(TerrainSelection("slope", slope_deg=3.0))
        else:
            action = LoadSceneAction(
                replace(
                    coordinator.logical_scene_document(),
                    terrain=TerrainDocument("golf_heightfield", 0.0, 31, "low"),
                )
            )

        result = coordinator.apply_action(action)

        assert result.state_changed is True
        assert result.world_reset is True
        assert runtime.prepare_calls == 1
        assert len(runtime.commit_calls) == 1
        assert runtime.commit_calls[0][2] == coordinator.logical_scene_document()
        assert twist_calls == []
    finally:
        p.disconnect(client_id)


def _rebuild_action(coordinator: SimulationCoordinator, action_name: str):
    """为 pending 串行测试构造四种会重建完整世界的动作。"""
    if action_name == "robot":
        return SwitchRobotAction("df_mid")
    if action_name == "reset":
        return ResetRobotAction()
    if action_name == "terrain":
        return SwitchTerrainAction(TerrainSelection("slope", slope_deg=3.0))
    return LoadSceneAction(
        replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("golf_heightfield", 0.0, 41, "low"),
        )
    )


@pytest.mark.parametrize("pending_operation", ("add", "clear"))
@pytest.mark.parametrize("action_name", ("robot", "reset", "terrain", "load"))
def test_apply_action_finishes_existing_pending_operation_before_rebuild(
    pending_operation: str,
    action_name: str,
) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world, soft_budget_seconds=0.0)
        if pending_operation == "clear":
            assert manager.restore(_obstacle_snapshots()).succeeded is True
        runtime = RecordingRuntime(
            _runtime_document(
                world,
                SensorDocument.default(),
                manager.snapshot(include_body_id=False),
            )
        )
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        if pending_operation == "add":
            coordinator.enqueue(
                AddObstaclesAction(ObstacleGenerationRequest("static", 2, seed=81))
            )
        else:
            coordinator.enqueue(ClearObstaclesAction())

        coordinator.step(config.time_step)
        if pending_operation == "add":
            guard = 0
            while not manager.pending_body_ids():
                guard += 1
                assert guard < 10
                coordinator.step(config.time_step)
        assert coordinator._active_action is not None
        action = _rebuild_action(coordinator, action_name)

        result = coordinator.apply_action(action)

        assert result.world_reset is True
        assert result.obstacle_result is None
        assert coordinator._active_action is None
        assert manager.pending_body_ids() == ()
        _assert_single_world_partition(client_id, coordinator)
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("incoming_action", ("load", "switch"))
def test_synchronous_action_drains_active_and_queued_actions_in_fifo_order(
    incoming_action: str,
) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world, soft_budget_seconds=0.0)
        runtime = RecordingRuntime(_runtime_document(world, SensorDocument.default()))
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        coordinator.enqueue(
            AddObstaclesAction(ObstacleGenerationRequest("static", 2, seed=91))
        )
        coordinator.step(config.time_step)
        guard = 0
        while not manager.pending_body_ids():
            guard += 1
            assert guard < 10
            coordinator.step(config.time_step)

        golf_document = replace(
            coordinator.logical_scene_document(),
            robot_model="active_steering_4wd",
            terrain=TerrainDocument("golf_heightfield", 0.0, 52, "low"),
            obstacles=(),
        )
        if incoming_action == "load":
            coordinator.enqueue(SwitchRobotAction("df_mid"))
            action = LoadSceneAction(golf_document)
            expected_commit_models = ["df_mid", "active_steering_4wd"]
            expected_robot = "active_steering_4wd"
        else:
            coordinator.enqueue(LoadSceneAction(golf_document))
            action = SwitchRobotAction("df_mid")
            expected_commit_models = ["active_steering_4wd", "df_mid"]
            expected_robot = "df_mid"

        result = coordinator.apply_action(action)

        assert result.world_reset is True
        assert coordinator.has_pending_action is False
        assert [call[2].robot_model for call in runtime.commit_calls] == expected_commit_models
        assert coordinator.world.active_robot.robot_model == expected_robot
        assert coordinator.world.terrain == TerrainSelection(
            "golf_heightfield",
            golf_seed=52,
            golf_relief="low",
        )
        _assert_single_world_partition(client_id, coordinator)
    finally:
        p.disconnect(client_id)


def test_apply_scene_document_rejects_direct_call_during_active_action() -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world, soft_budget_seconds=0.0)
        coordinator = SimulationCoordinator(client_id, config, world, manager)
        coordinator.enqueue(
            AddObstaclesAction(ObstacleGenerationRequest("static", 2, seed=82))
        )
        coordinator.step(config.time_step)
        assert coordinator._active_action is not None
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 2.0, 0, "medium"),
        )

        with pytest.raises(RuntimeError, match="pending structural action"):
            coordinator.apply_scene_document(target)

        assert coordinator.world is world
        assert coordinator.obstacle_manager is manager
        assert coordinator._active_action is not None
    finally:
        p.disconnect(client_id)


def test_apply_scene_document_rejects_direct_call_when_fifo_queue_is_not_empty() -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        runtime = RecordingRuntime(_runtime_document(world, SensorDocument.default()))
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        coordinator.enqueue(SwitchRobotAction("df_mid"))
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 2.0, 0, "medium"),
        )

        with pytest.raises(RuntimeError, match="pending structural action"):
            coordinator.apply_scene_document(target)

        assert coordinator.world is world
        assert coordinator.obstacle_manager is manager
        assert coordinator.has_pending_action is True
        assert runtime.prepare_calls == 0
    finally:
        p.disconnect(client_id)


def test_scene_rebuild_removes_manager_pending_bodies_started_outside_coordinator(
    monkeypatch,
) -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world, soft_budget_seconds=0.0)
        coordinator = SimulationCoordinator(client_id, config, world, manager)
        manager.begin_add(ObstacleGenerationRequest("static", 2, seed=83))
        guard = 0
        while not manager.pending_body_ids():
            guard += 1
            assert guard < 10
            result = manager.advance_pending_operation()
            assert result.done is False
        pending_ids = set(manager.pending_body_ids())
        removed_ids: list[int] = []
        original_remove_body = p.removeBody

        def record_remove(body_id: int, **kwargs) -> None:
            removed_ids.append(body_id)
            original_remove_body(body_id, **kwargs)

        monkeypatch.setattr(coordinator_module.p, "removeBody", record_remove)
        target = replace(
            coordinator.logical_scene_document(),
            terrain=TerrainDocument("slope", 2.0, 0, "medium"),
        )

        result = coordinator.apply_scene_document(target)

        assert result.world_reset is True
        assert pending_ids <= set(removed_ids)
        _assert_single_world_partition(client_id, coordinator)
    finally:
        p.disconnect(client_id)


def test_obstacle_body_changes_refresh_runtime_only_after_successful_completion() -> None:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world, soft_budget_seconds=0.0)
        runtime = RecordingRuntime(_runtime_document(world, SensorDocument.default()))
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        coordinator.enqueue(
            AddObstaclesAction(ObstacleGenerationRequest("static", 1, seed=73))
        )

        result = coordinator.step(config.time_step)
        guard = 0
        while result is not None and result.obstacle_result is not None and not result.obstacle_result.done:
            assert runtime.refresh_calls == []
            guard += 1
            assert guard < 20
            result = coordinator.step(config.time_step)

        assert result is not None and result.obstacle_result is not None
        assert result.obstacle_result.succeeded is True
        assert len(runtime.refresh_calls) == 1
        terrain_ids, snapshots = runtime.refresh_calls[-1]
        assert terrain_ids == tuple(world.scene.body_ids)
        assert len(snapshots) == 1
        assert snapshots[0].physics_body_id is not None
        assert runtime.scene_document == coordinator.logical_scene_document()

        refresh_count = len(runtime.refresh_calls)
        missing = coordinator.apply_action(DeleteObstacleAction(9999))
        assert missing.state_changed is False
        assert len(runtime.refresh_calls) == refresh_count

        logical_id = coordinator.obstacle_manager.snapshot()[0].logical_id
        deleted = coordinator.apply_action(DeleteObstacleAction(logical_id))
        assert deleted.state_changed is True
        assert len(runtime.refresh_calls) == refresh_count + 1
        assert runtime.refresh_calls[-1][1] == ()

        empty_clear = coordinator.apply_action(ClearObstaclesAction())
        assert empty_clear.state_changed is True
        assert len(runtime.refresh_calls) == refresh_count + 1

        added = coordinator.apply_action(
            AddObstaclesAction(ObstacleGenerationRequest("moving", 1, seed=74))
        )
        assert added.state_changed is True
        assert len(runtime.refresh_calls) == refresh_count + 2
        assert runtime.scene_document == coordinator.logical_scene_document()
        before_progress = runtime.scene_document.obstacles[0].path.progress
        coordinator.step(config.time_step)
        assert len(runtime.refresh_calls) == refresh_count + 2
        assert runtime.scene_document == coordinator.logical_scene_document()
        assert runtime.scene_document.obstacles[0].path.progress != before_progress
        cleared = coordinator.apply_action(ClearObstaclesAction())
        assert cleared.state_changed is True
        assert len(runtime.refresh_calls) == refresh_count + 3
        assert runtime.refresh_calls[-1][1] == ()
    finally:
        p.disconnect(client_id)


def test_partially_failed_clear_refreshes_runtime_bindings_and_full_document(
    monkeypatch,
) -> None:
    """清空失败但已删 body 时，分类和完整 SceneDocument 仍必须同步。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world, soft_budget_seconds=1.0)
        assert manager.restore(_obstacle_snapshots()).succeeded is True
        runtime = RecordingRuntime(
            _runtime_document(
                world,
                SensorDocument.default(),
                manager.snapshot(include_body_id=False),
            )
        )
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        failed_body_id = manager.snapshot()[1].physics_body_id
        original_remove = p.removeBody

        def fail_second_obstacle(body_id: int, **kwargs) -> None:
            if body_id == failed_body_id:
                raise p.error("second obstacle removal failed")
            original_remove(body_id, **kwargs)

        monkeypatch.setattr(coordinator_module.p, "removeBody", fail_second_obstacle)

        result = coordinator.apply_action(ClearObstaclesAction())

        assert result.state_changed is False
        assert result.obstacle_result is not None
        assert result.obstacle_result.done is True
        assert result.obstacle_result.succeeded is False
        assert result.obstacle_result.deleted_count == 1
        remaining = manager.snapshot()
        assert [snapshot.logical_id for snapshot in remaining] == [12]
        assert len(runtime.refresh_calls) == 1
        terrain_ids, bound_snapshots = runtime.refresh_calls[0]
        assert terrain_ids == tuple(world.scene.body_ids)
        assert bound_snapshots == remaining
        assert runtime.scene_document == coordinator.logical_scene_document()
        assert [obstacle.logical_id for obstacle in runtime.scene_document.obstacles] == [12]
    finally:
        p.disconnect(client_id)


def test_cross_frame_clear_refreshes_runtime_after_each_deleted_slice() -> None:
    """每个清空切片删掉 body 后，都要在下一次物理步进前同步运行时场景。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world, soft_budget_seconds=0.0)
        first, moving = _obstacle_snapshots()
        snapshots = (first, replace(moving, mode="static", path=None))
        assert manager.restore(snapshots).succeeded is True
        runtime = RecordingRuntime(
            _runtime_document(
                world,
                SensorDocument.default(),
                manager.snapshot(include_body_id=False),
            )
        )
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        coordinator.enqueue(ClearObstaclesAction())

        first_slice = coordinator.step(config.time_step)

        assert first_slice is not None and first_slice.obstacle_result is not None
        assert first_slice.obstacle_result.done is False
        assert first_slice.obstacle_result.deleted_count == 1
        assert len(runtime.refresh_calls) == 1
        assert runtime.scene_document == coordinator.logical_scene_document()

        second_slice = coordinator.step(config.time_step)

        assert second_slice is not None and second_slice.obstacle_result is not None
        assert second_slice.obstacle_result.done is False
        assert second_slice.obstacle_result.deleted_count == 2
        assert len(runtime.refresh_calls) == 2
        assert runtime.scene_document == coordinator.logical_scene_document()

        completed = coordinator.step(config.time_step)

        assert completed is not None and completed.obstacle_result is not None
        assert completed.obstacle_result.done is True
        assert completed.obstacle_result.succeeded is True
        assert len(runtime.refresh_calls) == 2
    finally:
        p.disconnect(client_id)


def test_moving_step_uses_manager_metadata_and_reuses_scene_document_cache(
    monkeypatch,
) -> None:
    """每帧移动同步不得重复快照或重新深度校验整份场景文档。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = _manager(client_id, world)
        moving = _obstacle_snapshots()[1]
        assert manager.restore((moving,)).succeeded is True
        runtime = RecordingRuntime(
            _runtime_document(
                world,
                SensorDocument.default(),
                manager.snapshot(include_body_id=False),
            )
        )
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
        )
        original_snapshot = manager.snapshot
        original_builder = coordinator_module._logical_document_for_world
        snapshot_calls = 0
        document_build_calls = 0

        def count_snapshot(*, include_body_id=True):
            nonlocal snapshot_calls
            snapshot_calls += 1
            return original_snapshot(include_body_id=include_body_id)

        def count_document_build(*args, **kwargs):
            nonlocal document_build_calls
            document_build_calls += 1
            return original_builder(*args, **kwargs)

        monkeypatch.setattr(manager, "snapshot", count_snapshot)
        monkeypatch.setattr(
            coordinator_module,
            "_logical_document_for_world",
            count_document_build,
        )

        coordinator.step(config.time_step)

        assert snapshot_calls == 0
        assert document_build_calls == 0
        assert runtime.scene_document == coordinator.logical_scene_document()
        assert snapshot_calls == 0
        assert document_build_calls == 0
    finally:
        p.disconnect(client_id)
