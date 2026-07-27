# 场景协调器：在 PyBullet 主线程串行处理车辆、场地和障碍物结构操作。
from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING

import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.model_registry import get_robot_model
from slope_sim.obstacles import (
    Aabb3D,
    ObstacleGenerationSettings,
    ObstacleManager,
    ObstacleOperationResult,
    ObstacleSnapshot,
    ObstacleSpec,
)
from slope_sim.robot import DifferentialDriveRobot, create_robot
from slope_sim.runtime_actions import (
    AddObstaclesAction,
    ClearObstaclesAction,
    DeleteObstacleAction,
    LoadSceneAction,
    ResetRobotAction,
    RuntimeAction,
    SwitchRobotAction,
    SwitchTerrainAction,
    TerrainSelection,
    is_safe_stop_action,
)
from slope_sim.scene import SceneInfo, TerrainBounds, create_slope_scene
from slope_sim.scene_config import SceneDocument, SensorDocument, TerrainDocument
from slope_sim.sensor_backend import PyBulletSensorBackend

if TYPE_CHECKING:
    from slope_sim.interfaces.runtime import InterfaceRuntime


@dataclass(frozen=True)
class ActiveManualRobot:
    """GUI 手动模式中当前活动车辆及其模型参数。"""

    robot: DifferentialDriveRobot
    robot_model: str
    wheel_radius: float


@dataclass(frozen=True)
class ActiveManualWorld:
    """手动模式当前有效的场景、车辆和完整场地参数。"""

    scene: SceneInfo
    active_robot: ActiveManualRobot
    terrain: TerrainSelection


@dataclass(frozen=True)
class ManualSwitchResult:
    """一次结构操作后的有效世界和可供 Dashboard 显示的结果。"""

    world: ActiveManualWorld
    state_changed: bool
    world_reset: bool
    status_message: str
    error_message: str | None = None
    obstacle_result: ObstacleOperationResult | None = None


@dataclass(frozen=True)
class _BuiltScene:
    """一次完整场景构建的候选物理对象，提交前不改协调器当前引用。"""

    world: ActiveManualWorld
    obstacle_manager: ObstacleManager
    sensor_backend: PyBulletSensorBackend | None
    runtime_document: SceneDocument


def _logical_document_for_world(
    world: ActiveManualWorld,
    obstacle_manager: ObstacleManager,
    sensors: SensorDocument,
) -> SceneDocument:
    """从已提交物理世界生成不含 body id 的完整运行时场景文档。"""
    logical_specs = getattr(obstacle_manager, "logical_specs", None)
    obstacles = (
        logical_specs()
        if callable(logical_specs)
        else obstacle_manager.snapshot(include_body_id=False)
    )
    return SceneDocument.from_runtime(
        world.active_robot.robot_model,
        world.terrain,
        obstacles,
        sensors.mounts,
        lidar_config=sensors.lidar,
    )


def manual_wheel_radius_for_model(robot_model: str) -> float:
    """GUI 车型切换时从注册表读取稳定轮径。"""
    return get_robot_model(robot_model).wheel_radius


def load_manual_robot(
    client_id: int,
    config: ExperimentConfig,
    scene: SceneInfo,
    robot_model: str | None = None,
) -> ActiveManualRobot:
    """按场景出生点加载 GUI 手动模式车辆，并应用驱动摩擦。"""
    model = (robot_model or config.robot_model).lower()
    wheel_radius = manual_wheel_radius_for_model(model)
    robot = None
    try:
        robot = create_robot(
            client_id=client_id,
            robot_model=model,
            start_x=scene.spawn_position[0],
            start_y=scene.spawn_position[1],
            base_height=scene.spawn_position[2] + get_robot_model(model).base_height,
            start_orientation=scene.spawn_orientation,
            drive_motor_force=config.drive_motor_force,
        )
        robot.apply_drive_friction(config.drive_lateral_friction, config.support_lateral_friction)
    except Exception:
        # 加载后续步骤失败时及时删除半成品，避免切换失败留下幽灵车体。
        if robot is not None:
            _remove_body_safely(client_id, robot.robot_id)
        raise
    return ActiveManualRobot(robot=robot, robot_model=model, wheel_radius=wheel_radius)


def reload_manual_robot(
    client_id: int,
    current: ActiveManualRobot | None,
    config: ExperimentConfig,
    scene: SceneInfo,
    robot_model: str,
) -> ActiveManualRobot:
    """先加载新车再移除旧车，保证车型加载失败时旧车仍可使用。"""
    replacement = load_manual_robot(client_id, config, scene, robot_model=robot_model)
    return _commit_manual_robot_replacement(client_id, current, replacement)


def _commit_manual_robot_replacement(
    client_id: int,
    current: ActiveManualRobot | None,
    replacement: ActiveManualRobot,
) -> ActiveManualRobot:
    """按删除后的物理事实提交新车；旧体未确认消失时严格回收 replacement。"""
    if current is None:
        return replacement
    try:
        _remove_body_strict(client_id, current.robot.robot_id)
    except Exception as removal_exc:
        try:
            _remove_body_strict(client_id, replacement.robot.robot_id)
        except Exception as cleanup_exc:
            raise RuntimeError(
                f"old robot removal failed ({_exception_reason(removal_exc)}); "
                f"replacement cleanup also failed ({_exception_reason(cleanup_exc)})"
            ) from cleanup_exc
        raise
    return replacement


def create_manual_scene(client_id: int, config: ExperimentConfig, terrain: TerrainSelection) -> SceneInfo:
    """按运行时选择复用统一场景工厂，避免启动和切换使用两套参数。"""
    return create_slope_scene(
        client_id,
        slope_deg=terrain.slope_deg,
        time_step=config.time_step,
        ground_lateral_friction=config.ground_lateral_friction,
        ground_rolling_friction=config.ground_rolling_friction,
        ground_spinning_friction=config.ground_spinning_friction,
        terrain_model=terrain.terrain_model,
        golf_seed=terrain.golf_seed,
        golf_relief=terrain.golf_relief,
    )


def load_manual_world(
    client_id: int,
    config: ExperimentConfig,
    terrain: TerrainSelection,
    robot_model: str,
) -> ActiveManualWorld:
    """创建一套完整场地，并在其安全出生点加载指定车型。"""
    scene = create_manual_scene(client_id, config, terrain)
    try:
        active_robot = load_manual_robot(client_id, config, scene, robot_model=robot_model)
    except Exception:
        for body_id in scene.body_ids:
            _remove_body_safely(client_id, body_id)
        raise
    return ActiveManualWorld(scene=scene, active_robot=active_robot, terrain=terrain)


def create_obstacle_manager(
    client_id: int,
    world: ActiveManualWorld,
    *,
    template: ObstacleManager | None = None,
    vehicle_aabb_getter: Callable[[], Aabb3D | None] | None = None,
) -> ObstacleManager:
    """为当前场地创建障碍物管理器；重建场地时沿用旧事务预算等参数。"""
    bounds = world.scene.bounds or TerrainBounds(-8.0, 8.0, -4.0, 4.0)
    if template is None:
        settings = ObstacleGenerationSettings(
            bounds=bounds,
            spawn_position=world.scene.spawn_position,
        )
        return ObstacleManager(
            client_id,
            settings,
            terrain_body_ids=world.scene.body_ids,
            vehicle_aabb_getter=vehicle_aabb_getter,
        )

    settings = replace(
        template._settings,
        bounds=bounds,
        spawn_position=world.scene.spawn_position,
    )
    return ObstacleManager(
        client_id,
        settings,
        terrain_body_ids=world.scene.body_ids,
        vehicle_aabb_getter=vehicle_aabb_getter,
        monotonic_clock=template._clock,
        soft_budget_seconds=template._soft_budget_seconds,
    )


def build_world_from_scene_document(
    client_id: int,
    config: ExperimentConfig,
    document: SceneDocument,
    *,
    template: ObstacleManager | None = None,
) -> tuple[ActiveManualWorld, ObstacleManager]:
    """从完整逻辑文档构建初始物理世界，任一失败都回收本次新增 body。"""
    if not isinstance(document, SceneDocument):
        raise ValueError("document must be a SceneDocument")
    validated = SceneDocument(
        document.schema_version,
        document.robot_model,
        document.terrain,
        document.obstacles,
        document.sensors,
    )
    terrain = TerrainSelection(
        validated.terrain.terrain_model,
        slope_deg=validated.terrain.slope_deg,
        golf_seed=validated.terrain.golf_seed,
        golf_relief=validated.terrain.golf_relief,
    )
    existing_body_ids = _current_body_ids(client_id)
    world: ActiveManualWorld | None = None
    manager: ObstacleManager | None = None
    try:
        world = load_manual_world(client_id, config, terrain, validated.robot_model)
        manager = create_obstacle_manager(client_id, world, template=template)
        restore_result = manager.restore(
            tuple(
                SimulationCoordinator._snapshot_from_document(obstacle)
                for obstacle in validated.obstacles
            )
        )
        if not restore_result.succeeded:
            raise RuntimeError(restore_result.message or "scene obstacle restore failed")
        return world, manager
    except Exception:
        # 工厂可能在返回对象前抛错，body 差集是启动阶段最可靠的回收边界。
        for body_id in _current_body_ids(client_id) - existing_body_ids:
            _remove_body_safely(client_id, body_id)
        raise


def replace_manual_world_robot(
    client_id: int,
    config: ExperimentConfig,
    world: ActiveManualWorld,
    robot_model: str,
    *,
    force: bool = False,
) -> ManualSwitchResult:
    """在保留当前地形和障碍物管理器的前提下事务式替换活动车辆。"""
    target_model = robot_model.lower()
    if not force and target_model == world.active_robot.robot_model:
        return ManualSwitchResult(world, False, False, "当前车型已生效")
    try:
        replacement = load_manual_robot(client_id, config, world.scene, robot_model=target_model)
    except Exception as exc:
        message = f"应用车型失败，已保留 {world.active_robot.robot_model}: {exc}"
        return ManualSwitchResult(world, False, False, message, error_message=str(exc))

    try:
        replacement = _commit_manual_robot_replacement(
            client_id,
            world.active_robot,
            replacement,
        )
    except Exception as exc:
        if isinstance(exc, _BodyRemovalError) and exc.body_remains is True:
            message = f"应用车型失败，旧车型未删除: {exc}"
            return ManualSwitchResult(
                world,
                False,
                False,
                message,
                error_message=str(exc),
            )
        raise RuntimeError(
            f"old robot removal state is unknown: {_exception_reason(exc)}"
        ) from exc
    updated = ActiveManualWorld(scene=world.scene, active_robot=replacement, terrain=world.terrain)
    action = "车辆已复位" if force else f"车型已切换为 {target_model}"
    return ManualSwitchResult(updated, True, False, action)


class SimulationCoordinator:
    """协调结构操作 FIFO，并保证待处理事务期间仍推进障碍物和物理世界。"""

    def __init__(
        self,
        client_id: int,
        config: ExperimentConfig,
        world: ActiveManualWorld,
        obstacle_manager: ObstacleManager,
        *,
        step_physics: Callable[[int], None] | None = None,
        interface_runtime: InterfaceRuntime | None = None,
        sensor_document: SensorDocument | None = None,
    ) -> None:
        self.client_id = client_id
        self.config = config
        self.world = world
        self.obstacle_manager = obstacle_manager
        self.interface_runtime = interface_runtime
        runtime_document = (
            None if interface_runtime is None else getattr(interface_runtime, "scene_document", None)
        )
        if sensor_document is not None and not isinstance(sensor_document, SensorDocument):
            raise ValueError("sensor_document must be a SensorDocument")
        explicit_sensors = (
            None
            if sensor_document is None
            else SensorDocument(sensor_document.mounts, sensor_document.lidar)
        )
        if isinstance(runtime_document, SceneDocument):
            runtime_sensors = SensorDocument(
                runtime_document.sensors.mounts,
                runtime_document.sensors.lidar,
            )
            if explicit_sensors is not None and explicit_sensors != runtime_sensors:
                raise ValueError("sensor_document must match runtime scene_document sensors")
            selected_sensors = runtime_sensors
        else:
            selected_sensors = explicit_sensors or SensorDocument.default()
        # 协调器只持有重构后的唯一传感器配置，不保存第二份可漂移来源。
        self.sensor_document = selected_sensors
        if isinstance(runtime_document, SceneDocument):
            expected_document = _logical_document_for_world(
                world,
                obstacle_manager,
                selected_sensors,
            )
            if runtime_document != expected_document:
                raise ValueError(
                    "runtime scene_document must match coordinator logical scene"
                )
        self.last_result: ManualSwitchResult | None = None
        self._queue: deque[RuntimeAction] = deque()
        self._active_action: RuntimeAction | None = None
        self._active_clear_synced_deleted_count = 0
        self._scene_document_cache_token: tuple[int, int, int, int] | None = None
        self._scene_document_cache: SceneDocument | None = None
        self._step_physics = step_physics or (lambda client_id: p.stepSimulation(physicsClientId=client_id))
        self._bind_obstacle_manager_to_current_robot()

    def enqueue(self, action: RuntimeAction) -> None:
        """把 Dashboard 的一次性结构操作放入 FIFO。"""
        self._queue.append(action)

    @property
    def has_pending_action(self) -> bool:
        return self._active_action is not None or bool(self._queue)

    def apply_action(self, action: RuntimeAction) -> ManualSwitchResult:
        """同步执行单个结构操作，供测试和立即完成的场景事务复用。"""
        # 同步入口先按 FIFO 排空既有事务；Dashboard 的 step 跨帧路径保持不变。
        while self._active_action is not None or self._queue:
            if self._active_action is not None:
                self.last_result = self._advance_active_action()
            else:
                self._start_next_action()
        if self.interface_runtime is None and is_safe_stop_action(action):
            self.world.active_robot.robot.command_twist(0.0, 0.0, dt=self.config.time_step)
        result = self._apply_immediate_action(action)
        while self._active_action is not None:
            result = self._advance_active_action()
        self.last_result = result
        return result

    def step(self, dt: float) -> ManualSwitchResult | None:
        """推进一个物理帧：先推进一个结构事务时间片，再更新障碍物并步进物理。"""
        if self._active_action is None and self._queue:
            self._start_next_action()
        if self._active_action is not None:
            self.last_result = self._advance_active_action()

        self.obstacle_manager.update_moving(dt)
        if self.interface_runtime is not None and self._manager_has_moving_obstacles():
            self.interface_runtime.update_scene_document(self.logical_scene_document())
        self._step_physics(self.client_id)
        return self.last_result

    def _start_next_action(self) -> None:
        action = self._queue.popleft()
        if self.interface_runtime is None and is_safe_stop_action(action):
            self.world.active_robot.robot.command_twist(0.0, 0.0, dt=self.config.time_step)
        if isinstance(action, AddObstaclesAction):
            self.obstacle_manager.begin_add(action.request)
            self._active_action = action
            return
        if isinstance(action, ClearObstaclesAction):
            self.obstacle_manager.begin_clear()
            self._active_clear_synced_deleted_count = 0
            self._active_action = action
            return
        self.last_result = self._apply_immediate_action(action)

    def _advance_active_action(self) -> ManualSwitchResult:
        assert self._active_action is not None
        result = self.obstacle_manager.advance_pending_operation()
        if result.operation == "clear":
            # 跨帧清空每确认一个 body 消失，就在本帧物理步进前同步运行时。
            deleted_count = result.deleted_count
            if deleted_count > self._active_clear_synced_deleted_count:
                self._refresh_runtime_scene_bindings()
                self._active_clear_synced_deleted_count = deleted_count
        if result.done:
            self._active_action = None
            if result.operation != "clear" and self._obstacle_result_changed_body_set(result):
                self._refresh_runtime_scene_bindings()
            self._active_clear_synced_deleted_count = 0
        return ManualSwitchResult(
            self.world,
            state_changed=result.succeeded,
            world_reset=False,
            status_message=result.message or f"{result.operation} done",
            error_message=None if result.succeeded else result.message,
            obstacle_result=result,
        )

    def _apply_immediate_action(self, action: RuntimeAction) -> ManualSwitchResult:
        if isinstance(action, SwitchRobotAction):
            if self.interface_runtime is not None:
                target_model = action.robot_model.lower()
                if target_model == self.world.active_robot.robot_model:
                    return ManualSwitchResult(self.world, False, False, "当前车型已生效")
                document = replace(
                    self.logical_scene_document(),
                    robot_model=target_model,
                )
                result = self._apply_scene_document_transaction(document)
                if result.error_message is None:
                    return replace(result, status_message=f"车型已切换为 {target_model}")
                return result
            result = replace_manual_world_robot(self.client_id, self.config, self.world, action.robot_model)
            self.world = result.world
            self._bind_obstacle_manager_to_current_robot()
            return result
        if isinstance(action, ResetRobotAction):
            if self.interface_runtime is not None:
                result = self._apply_scene_document_transaction(self.logical_scene_document())
                if result.error_message is None:
                    return replace(result, status_message="车辆已复位")
                return result
            result = replace_manual_world_robot(
                self.client_id,
                self.config,
                self.world,
                self.world.active_robot.robot_model,
                force=True,
            )
            self.world = result.world
            self._bind_obstacle_manager_to_current_robot()
            return result
        if isinstance(action, SwitchTerrainAction):
            if action.terrain == self.world.terrain:
                return ManualSwitchResult(self.world, False, False, "当前场地已生效")
            document = replace(
                self.logical_scene_document(),
                terrain=TerrainDocument.from_selection(action.terrain),
            )
            result = self._apply_scene_document_transaction(document)
            if result.error_message is None:
                return replace(
                    result,
                    status_message=f"场地已切换为 {action.terrain.terrain_model}",
                )
            return result
        if isinstance(action, LoadSceneAction):
            return self._apply_scene_document_transaction(action.document)
        if isinstance(action, DeleteObstacleAction):
            obstacle_result = self.obstacle_manager.delete(action.logical_id)
            if self._obstacle_result_changed_body_set(obstacle_result):
                self._refresh_runtime_scene_bindings()
            return ManualSwitchResult(
                self.world,
                state_changed=obstacle_result.succeeded,
                world_reset=False,
                status_message=obstacle_result.message or "障碍物已删除",
                error_message=None if obstacle_result.succeeded else obstacle_result.message,
                obstacle_result=obstacle_result,
            )
        if isinstance(action, AddObstaclesAction):
            self.obstacle_manager.begin_add(action.request)
            self._active_action = action
            return ManualSwitchResult(self.world, False, False, "障碍物添加中")
        if isinstance(action, ClearObstaclesAction):
            self.obstacle_manager.begin_clear()
            self._active_clear_synced_deleted_count = 0
            self._active_action = action
            return ManualSwitchResult(self.world, False, False, "障碍物清空中")
        raise TypeError(f"unsupported runtime action: {action!r}")

    def logical_scene_document(self) -> SceneDocument:
        """从当前物理世界动态导出唯一逻辑真相，永不携带临时 body id。"""
        revision = getattr(self.obstacle_manager, "revision", None)
        cache_token = (
            id(self.world),
            id(self.obstacle_manager),
            revision,
            id(self.sensor_document),
        ) if isinstance(revision, int) else None
        if (
            cache_token is not None
            and cache_token == self._scene_document_cache_token
            and self._scene_document_cache is not None
        ):
            return self._scene_document_cache
        document = _logical_document_for_world(
            self.world,
            self.obstacle_manager,
            self.sensor_document,
        )
        if cache_token is not None:
            self._scene_document_cache_token = cache_token
            self._scene_document_cache = document
        return document

    def _manager_has_moving_obstacles(self) -> bool:
        """优先读取管理器 O(1) 标志；兼容测试替身和旧实现。"""
        has_moving = getattr(self.obstacle_manager, "has_moving", None)
        if isinstance(has_moving, bool):
            return has_moving
        return any(
            snapshot.path is not None
            for snapshot in self.obstacle_manager.snapshot(include_body_id=False)
        )

    def apply_scene_document(self, document: SceneDocument) -> ManualSwitchResult:
        """直接场景入口不得绕过已调度的结构操作。"""
        if self._active_action is not None or self._queue:
            raise RuntimeError(
                "pending structural action must finish before applying a scene document"
            )
        return self._apply_scene_document_transaction(document)

    def _apply_scene_document_transaction(
        self,
        document: SceneDocument,
    ) -> ManualSwitchResult:
        """完整重建场景；目标失败时破坏性恢复之前的逻辑文档。"""
        validated = self._revalidate_scene_document(document)
        previous = self.logical_scene_document()
        old_world = self.world
        old_manager = self.obstacle_manager

        # prepare 必须位于任何 PyBullet 删除之前，由运行时独占安全停车和命令屏障。
        if self.interface_runtime is not None:
            try:
                self.interface_runtime.prepare_world_rebuild()
            except Exception as prepare_exc:
                prepare_reason = _exception_reason(prepare_exc)
                try:
                    self.interface_runtime.abort_world_rebuild()
                except Exception as abort_exc:
                    abort_reason = _exception_reason(abort_exc)
                    raise RuntimeError(
                        f"prepare world rebuild failed ({prepare_reason}); "
                        f"abort also failed ({abort_reason})"
                    ) from abort_exc
                message = f"准备场景重建失败，旧场景保持有效: {prepare_reason}"
                return ManualSwitchResult(
                    self.world,
                    False,
                    False,
                    message,
                    error_message=prepare_reason,
                )
        # body 枚举失败发生在删除前，可证明旧世界尚未被破坏并安全 abort。
        try:
            owned_body_ids = self._world_body_ids(old_world, old_manager)
            current_body_ids = _current_body_ids(self.client_id)
        except Exception as enumeration_exc:
            return self._abort_unchanged_active_world(
                old_world,
                _exception_reason(enumeration_exc),
            )

        active_body_ids = tuple(
            body_id for body_id in owned_body_ids if body_id in current_body_ids
        )
        old_body_ids = set(active_body_ids)
        try:
            self._remove_active_world_strict(
                old_world,
                old_manager,
                known_present_body_ids=active_body_ids,
            )
        except Exception as removal_exc:
            removal_reason = _exception_reason(removal_exc)
            try:
                remaining_body_ids = _current_body_ids(self.client_id)
            except Exception as diagnosis_exc:
                diagnosis_reason = _exception_reason(diagnosis_exc)
                if self.interface_runtime is not None:
                    try:
                        self.interface_runtime.fault_world_rebuild()
                    except Exception as fault_exc:
                        fault_reason = _exception_reason(fault_exc)
                        raise RuntimeError(
                            f"active world removal failed ({removal_reason}); "
                            f"removal diagnosis also failed ({diagnosis_reason}); "
                            f"fault transition also failed ({fault_reason})"
                        ) from fault_exc
                raise RuntimeError(
                    f"active world removal failed ({removal_reason}); "
                    f"removal diagnosis also failed ({diagnosis_reason})"
                ) from diagnosis_exc
            removed_body_ids = old_body_ids - remaining_body_ids
            if not removed_body_ids:
                return self._abort_unchanged_active_world(
                    old_world,
                    removal_reason,
                )
            return self._recover_after_partial_removal(
                previous,
                old_world,
                old_manager,
                removal_reason,
            )

        target: _BuiltScene | None = None
        try:
            target = self._build_scene(validated, template=old_manager)
            if self.interface_runtime is not None:
                if target.sensor_backend is None:
                    raise RuntimeError("interface scene rebuild has no sensor backend")
                self.interface_runtime.commit_world_rebuild(
                    target.world.active_robot.robot,
                    target.sensor_backend,
                    target.runtime_document,
                )
        except Exception as target_exc:
            target_reason = _exception_reason(target_exc)
            if target is not None:
                self._cleanup_world_best_effort(target.world, target.obstacle_manager)
            rollback: _BuiltScene | None = None
            try:
                rollback = self._build_scene(previous, template=old_manager)
                if self.interface_runtime is not None:
                    if rollback.sensor_backend is None:
                        raise RuntimeError("interface scene rollback has no sensor backend")
                    self.interface_runtime.commit_world_rebuild(
                        rollback.world.active_robot.robot,
                        rollback.sensor_backend,
                        rollback.runtime_document,
                    )
            except Exception as rollback_exc:
                rollback_reason = _exception_reason(rollback_exc)
                if rollback is not None:
                    self._cleanup_world_best_effort(
                        rollback.world,
                        rollback.obstacle_manager,
                    )
                if self.interface_runtime is not None:
                    self.interface_runtime.fault_world_rebuild()
                raise RuntimeError(
                    f"target scene failed ({target_reason}); "
                    f"rollback also failed ({rollback_reason})"
                ) from rollback_exc

            self._install_built_scene(rollback, previous.sensors)
            message = f"应用场景失败，已恢复原场景: {target_reason}"
            return ManualSwitchResult(
                rollback.world,
                True,
                True,
                message,
                error_message=target_reason,
            )

        self._install_built_scene(target, validated.sensors)
        return ManualSwitchResult(
            target.world,
            True,
            True,
            "场景已加载",
        )

    @staticmethod
    def _revalidate_scene_document(document: SceneDocument) -> SceneDocument:
        """逐层重构 frozen 文档，阻止绕过构造器的非法值进入物理事务。"""
        if not isinstance(document, SceneDocument):
            raise ValueError("document must be a SceneDocument")
        try:
            return SceneDocument(
                document.schema_version,
                document.robot_model,
                document.terrain,
                document.obstacles,
                document.sensors,
            )
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"scene document: {exc}") from exc

    def _build_scene(
        self,
        document: SceneDocument,
        *,
        template: ObstacleManager,
    ) -> _BuiltScene:
        """构建并分类候选 world/manager/backend，异常时清理全部候选 body。"""
        world: ActiveManualWorld | None = None
        manager: ObstacleManager | None = None
        existing_body_ids = _current_body_ids(self.client_id)
        try:
            world, manager = build_world_from_scene_document(
                self.client_id,
                self.config,
                document,
                template=template,
            )
            backend: PyBulletSensorBackend | None = None
            if self.interface_runtime is not None:
                backend = PyBulletSensorBackend(
                    self.client_id,
                    world.active_robot.robot.robot_id,
                )
                backend.bind_scene(
                    world.scene.body_ids,
                    manager.snapshot(include_body_id=True),
                )
            runtime_document = _logical_document_for_world(
                world,
                manager,
                document.sensors,
            )
            return _BuiltScene(world, manager, backend, runtime_document)
        except Exception:
            if world is not None:
                self._cleanup_world_best_effort(world, manager)
            # 场景工厂可能在返回 SceneInfo 前抛错，只能按主线程 body 差集回收。
            for body_id in _current_body_ids(self.client_id) - existing_body_ids:
                _remove_body_safely(self.client_id, body_id)
            raise

    @staticmethod
    def _snapshot_from_document(obstacle: ObstacleSpec) -> ObstacleSnapshot:
        """把稳定逻辑障碍物转换成 restore 唯一接受的无 body-id 快照。"""
        return ObstacleSnapshot(
            logical_id=obstacle.logical_id,
            body_id=None,
            mode=obstacle.mode,
            shape=obstacle.geometry.shape,
            position=obstacle.position,
            orientation=obstacle.orientation,
            path=obstacle.path,
            geometry=obstacle.geometry,
        )

    def _install_built_scene(
        self,
        built: _BuiltScene,
        sensors: SensorDocument,
    ) -> None:
        """运行时提交成功后再发布协调器引用，并重绑车辆 AABB。"""
        self.world = built.world
        self.obstacle_manager = built.obstacle_manager
        self.sensor_document = sensors
        self._bind_obstacle_manager_to_current_robot()

    def _recover_after_partial_removal(
        self,
        previous: SceneDocument,
        old_world: ActiveManualWorld,
        old_manager: ObstacleManager,
        removal_reason: str,
    ) -> ManualSwitchResult:
        """部分删除后只允许清尽旧 body 并重建 previous，绝不安装 target。"""
        try:
            self._remove_active_world_strict(old_world, old_manager)
        except Exception as cleanup_exc:
            cleanup_reason = _exception_reason(cleanup_exc)
            if self.interface_runtime is not None:
                self.interface_runtime.fault_world_rebuild()
            raise RuntimeError(
                f"active world removal failed ({removal_reason}); "
                f"recovery removal also failed ({cleanup_reason})"
            ) from cleanup_exc

        rollback: _BuiltScene | None = None
        try:
            rollback = self._build_scene(previous, template=old_manager)
            if self.interface_runtime is not None:
                if rollback.sensor_backend is None:
                    raise RuntimeError("interface scene rollback has no sensor backend")
                self.interface_runtime.commit_world_rebuild(
                    rollback.world.active_robot.robot,
                    rollback.sensor_backend,
                    rollback.runtime_document,
                )
        except Exception as rollback_exc:
            rollback_reason = _exception_reason(rollback_exc)
            if rollback is not None:
                self._cleanup_world_best_effort(
                    rollback.world,
                    rollback.obstacle_manager,
                )
            if self.interface_runtime is not None:
                self.interface_runtime.fault_world_rebuild()
            raise RuntimeError(
                f"active world removal failed ({removal_reason}); "
                f"rollback also failed ({rollback_reason})"
            ) from rollback_exc

        self._install_built_scene(rollback, previous.sensors)
        message = f"删除旧场景失败，已恢复原场景: {removal_reason}"
        return ManualSwitchResult(
            rollback.world,
            True,
            True,
            message,
            error_message=removal_reason,
        )

    def _abort_unchanged_active_world(
        self,
        old_world: ActiveManualWorld,
        removal_reason: str,
    ) -> ManualSwitchResult:
        """确认活动 body 未被删除时，退出 prepared 并继续使用原世界。"""
        if self.interface_runtime is not None:
            try:
                self.interface_runtime.abort_world_rebuild()
            except Exception as abort_exc:
                abort_reason = _exception_reason(abort_exc)
                raise RuntimeError(
                    f"active world removal failed ({removal_reason}); "
                    f"abort also failed ({abort_reason})"
                ) from abort_exc
        message = f"删除旧场景失败，旧场景保持有效: {removal_reason}"
        return ManualSwitchResult(
            old_world,
            False,
            False,
            message,
            error_message=removal_reason,
        )

    @staticmethod
    def _obstacle_result_changed_body_set(result: object) -> bool:
        """已完成操作只要确实改变 body 集，就刷新射线分类和逻辑文档。"""
        if not getattr(result, "done", False):
            return False
        operation = getattr(result, "operation", "")
        if operation == "add":
            return (
                getattr(result, "succeeded", False)
                and getattr(result, "published_count", 0) > 0
            )
        if operation in {"delete", "clear"}:
            return getattr(result, "deleted_count", 0) > 0
        return False

    def _refresh_runtime_scene_bindings(self) -> None:
        """障碍物 body 集提交后，把当前临时物理分类刷新给既有 backend。"""
        if self.interface_runtime is None:
            return
        self.interface_runtime.refresh_scene_bindings(
            self.world.scene.body_ids,
            self.obstacle_manager.snapshot(include_body_id=True),
            self.logical_scene_document(),
        )

    def _bind_obstacle_manager_to_current_robot(self) -> None:
        """让障碍物提交前始终读取当前车辆 AABB，避免车型/场地切换后引用旧车。"""
        setter = getattr(self.obstacle_manager, "set_vehicle_aabb_getter", None)
        if setter is None:
            return
        setter(
            lambda: robot_body_aabb(self.client_id, self.world.active_robot.robot.robot_id)
        )

    def _world_body_ids(
        self,
        world: ActiveManualWorld,
        obstacle_manager: ObstacleManager | None,
    ) -> tuple[int, ...]:
        """按依赖顺序收集一次世界事务拥有的全部 body，并稳定去重。"""
        body_ids: list[int] = []
        if obstacle_manager is not None:
            pending_body_ids = getattr(obstacle_manager, "pending_body_ids", None)
            if callable(pending_body_ids):
                body_ids.extend(pending_body_ids())
            for snapshot in obstacle_manager.snapshot(include_body_id=True):
                if snapshot.physics_body_id is not None:
                    body_ids.append(snapshot.physics_body_id)
        body_ids.append(world.active_robot.robot.robot_id)
        body_ids.extend(world.scene.body_ids)
        return tuple(dict.fromkeys(body_ids))

    def _remove_active_world_strict(
        self,
        world: ActiveManualWorld,
        obstacle_manager: ObstacleManager | None,
        *,
        known_present_body_ids: tuple[int, ...] | None = None,
    ) -> None:
        """严格删除已提交活动世界；异常或残留都必须进入事务恢复。"""
        body_ids = (
            self._world_body_ids(world, obstacle_manager)
            if known_present_body_ids is None
            else known_present_body_ids
        )
        for body_id in body_ids:
            if (
                known_present_body_ids is None
                and body_id not in _current_body_ids(self.client_id)
            ):
                continue
            try:
                p.removeBody(body_id, physicsClientId=self.client_id)
            except Exception as exc:
                raise RuntimeError(
                    f"failed to remove active body {body_id}: {_exception_reason(exc)}"
                ) from exc
            if body_id in _current_body_ids(self.client_id):
                raise RuntimeError(f"active body {body_id} remained after removeBody")

    def _cleanup_world_best_effort(
        self,
        world: ActiveManualWorld,
        obstacle_manager: ObstacleManager | None,
    ) -> None:
        """候选或回滚失败后的清理不覆盖正在传播的事务首错。"""
        try:
            body_ids = self._world_body_ids(world, obstacle_manager)
        except Exception:
            body_ids = (
                world.active_robot.robot.robot_id,
                *world.scene.body_ids,
            )
        for body_id in body_ids:
            _remove_body_safely(self.client_id, body_id)


def _remove_body_safely(client_id: int, body_id: int) -> None:
    """删除 PyBullet body；已被清理的 body 忽略，避免回滚清理二次失败。"""
    try:
        p.removeBody(body_id, physicsClientId=client_id)
    except Exception:
        pass


def _current_body_ids(client_id: int) -> set[int]:
    """读取当前 client 的 body ID 集，供场景工厂异常路径做精确差集清理。"""
    return {
        p.getBodyUniqueId(index, physicsClientId=client_id)
        for index in range(p.getNumBodies(client_id))
    }


def _exception_reason(error: BaseException) -> str:
    """异常无文本时回退到类型名，保证事务结果始终提供非空原因。"""
    return str(error) or type(error).__name__


class _BodyRemovalError(RuntimeError):
    """携带后验存在性；None 表示诊断失败，调用方不得选择任一 world。"""

    def __init__(self, message: str, *, body_remains: bool | None) -> None:
        super().__init__(message)
        self.body_remains = body_remains


def _remove_body_strict(client_id: int, body_id: int) -> None:
    """删除后复核 body 集；删后抛错仍按物理事实提交。"""
    removal_error: Exception | None = None
    try:
        p.removeBody(body_id, physicsClientId=client_id)
    except Exception as exc:
        removal_error = exc

    try:
        body_remains = body_id in _current_body_ids(client_id)
    except Exception as diagnosis_exc:
        diagnosis_reason = _exception_reason(diagnosis_exc)
        if removal_error is None:
            message = f"failed to diagnose body {body_id} removal: {diagnosis_reason}"
        else:
            message = (
                f"body {body_id} removal failed ({_exception_reason(removal_error)}); "
                f"diagnosis also failed ({diagnosis_reason})"
            )
        raise _BodyRemovalError(message, body_remains=None) from diagnosis_exc

    if not body_remains:
        return
    if removal_error is not None:
        message = (
            f"failed to remove body {body_id} "
            f"({_exception_reason(removal_error)}); body remained"
        )
        raise _BodyRemovalError(message, body_remains=True) from removal_error
    raise _BodyRemovalError(
        f"body {body_id} remained after removeBody",
        body_remains=True,
    )


def robot_body_aabb(client_id: int, robot_id: int) -> Aabb3D | None:
    """合并机器人 base 与所有 link 的 AABB，供障碍物规划提交前避让当前车辆。"""
    aabbs: list[Aabb3D] = []
    for link_index in range(-1, p.getNumJoints(robot_id, physicsClientId=client_id)):
        try:
            aabb_min, aabb_max = p.getAABB(robot_id, link_index, physicsClientId=client_id)
        except Exception:
            return None
        aabbs.append((tuple(aabb_min), tuple(aabb_max)))
    if not aabbs:
        return None
    return (
        tuple(min(aabb[0][axis] for aabb in aabbs) for axis in range(3)),
        tuple(max(aabb[1][axis] for aabb in aabbs) for axis in range(3)),
    )
