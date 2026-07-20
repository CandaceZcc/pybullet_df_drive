# 场景协调器：在 PyBullet 主线程串行处理车辆、场地和障碍物结构操作。
from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, replace

import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.obstacles import Aabb3D, ObstacleGenerationSettings, ObstacleManager, ObstacleOperationResult
from slope_sim.robot import DifferentialDriveRobot, create_robot
from slope_sim.model_registry import get_robot_model
from slope_sim.runtime_actions import (
    AddObstaclesAction,
    ClearObstaclesAction,
    DeleteObstacleAction,
    ResetRobotAction,
    RuntimeAction,
    SwitchRobotAction,
    SwitchTerrainAction,
    TerrainSelection,
    is_safe_stop_action,
)
from slope_sim.scene import SceneInfo, TerrainBounds, create_slope_scene
from slope_sim.simulation import _robot_base_height


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
            base_height=scene.spawn_position[2] + _robot_base_height(model),
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
    if current is not None:
        _remove_body_strict(client_id, current.robot.robot_id)
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
        _remove_body_strict(client_id, world.active_robot.robot.robot_id)
    except Exception as exc:
        _remove_body_safely(client_id, replacement.robot.robot_id)
        message = f"应用车型失败，旧车型未删除: {exc}"
        return ManualSwitchResult(world, False, False, message, error_message=str(exc))
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
    ) -> None:
        self.client_id = client_id
        self.config = config
        self.world = world
        self.obstacle_manager = obstacle_manager
        self.last_result: ManualSwitchResult | None = None
        self._queue: deque[RuntimeAction] = deque()
        self._active_action: RuntimeAction | None = None
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
        if is_safe_stop_action(action):
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
        self._step_physics(self.client_id)
        return self.last_result

    def _start_next_action(self) -> None:
        action = self._queue.popleft()
        if is_safe_stop_action(action):
            self.world.active_robot.robot.command_twist(0.0, 0.0, dt=self.config.time_step)
        if isinstance(action, AddObstaclesAction):
            self.obstacle_manager.begin_add(action.request)
            self._active_action = action
            return
        if isinstance(action, ClearObstaclesAction):
            self.obstacle_manager.begin_clear()
            self._active_action = action
            return
        self.last_result = self._apply_immediate_action(action)

    def _advance_active_action(self) -> ManualSwitchResult:
        assert self._active_action is not None
        result = self.obstacle_manager.advance_pending_operation()
        if result.done:
            self._active_action = None
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
            result = replace_manual_world_robot(self.client_id, self.config, self.world, action.robot_model)
            self.world = result.world
            self._bind_obstacle_manager_to_current_robot()
            return result
        if isinstance(action, ResetRobotAction):
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
            return self._switch_terrain(action.terrain)
        if isinstance(action, DeleteObstacleAction):
            obstacle_result = self.obstacle_manager.delete(action.logical_id)
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
            self._active_action = action
            return ManualSwitchResult(self.world, False, False, "障碍物清空中")
        raise TypeError(f"unsupported runtime action: {action!r}")

    def _switch_terrain(self, target_terrain: TerrainSelection) -> ManualSwitchResult:
        """按快照、重建、恢复、提交流程切换场地；失败时重建旧世界回滚。"""
        if target_terrain == self.world.terrain:
            return ManualSwitchResult(self.world, False, False, "当前场地已生效")

        old_world = self.world
        old_manager = self.obstacle_manager
        old_model = old_world.active_robot.robot_model
        snapshots = old_manager.snapshot(include_body_id=False)
        self._remove_world_and_obstacles(old_world, old_manager)

        target_world: ActiveManualWorld | None = None
        target_manager: ObstacleManager | None = None
        try:
            target_world = load_manual_world(self.client_id, self.config, target_terrain, old_model)
            target_manager = create_obstacle_manager(self.client_id, target_world, template=old_manager)
            target_manager.restore(snapshots)
        except Exception as target_exc:
            if target_world is not None:
                self._remove_world_and_obstacles(target_world, target_manager)
            try:
                restored_world = load_manual_world(self.client_id, self.config, old_world.terrain, old_model)
                restored_manager = create_obstacle_manager(self.client_id, restored_world, template=old_manager)
                restored_manager.restore(snapshots)
            except Exception as rollback_exc:
                raise RuntimeError(
                    f"target terrain failed ({target_exc}); rollback also failed ({rollback_exc})"
                ) from rollback_exc
            self.world = restored_world
            self.obstacle_manager = restored_manager
            self._bind_obstacle_manager_to_current_robot()
            message = f"应用场地失败，已恢复 {old_world.terrain.terrain_model}: {target_exc}"
            return ManualSwitchResult(restored_world, True, True, message, error_message=str(target_exc))

        assert target_world is not None and target_manager is not None
        self.world = target_world
        self.obstacle_manager = target_manager
        self._bind_obstacle_manager_to_current_robot()
        return ManualSwitchResult(target_world, True, True, f"场地已切换为 {target_terrain.terrain_model}")

    def _bind_obstacle_manager_to_current_robot(self) -> None:
        """让障碍物提交前始终读取当前车辆 AABB，避免车型/场地切换后引用旧车。"""
        setter = getattr(self.obstacle_manager, "set_vehicle_aabb_getter", None)
        if setter is None:
            return
        setter(
            lambda: robot_body_aabb(self.client_id, self.world.active_robot.robot.robot_id)
        )

    def _remove_world_and_obstacles(
        self,
        world: ActiveManualWorld,
        obstacle_manager: ObstacleManager | None,
    ) -> None:
        if obstacle_manager is not None:
            for snapshot in obstacle_manager.snapshot(include_body_id=True):
                if snapshot.physics_body_id is not None:
                    _remove_body_safely(self.client_id, snapshot.physics_body_id)
        _remove_body_safely(self.client_id, world.active_robot.robot.robot_id)
        for body_id in world.scene.body_ids:
            _remove_body_safely(self.client_id, body_id)


def _remove_body_safely(client_id: int, body_id: int) -> None:
    """删除 PyBullet body；已被清理的 body 忽略，避免回滚清理二次失败。"""
    try:
        p.removeBody(body_id, physicsClientId=client_id)
    except Exception:
        pass


def _remove_body_strict(client_id: int, body_id: int) -> None:
    """事务关键路径使用严格删除，失败必须反馈给上层回滚。"""
    p.removeBody(body_id, physicsClientId=client_id)


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
