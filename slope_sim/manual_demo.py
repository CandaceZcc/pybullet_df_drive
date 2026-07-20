# GUI 手动演示模块：打开 PyBullet 窗口，用方向键控制车辆在当前地形中运动。
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time

import pandas as pd
import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.dashboard import DashboardCommand, TelemetryDashboard, TerrainSelection
from slope_sim.diagnostics import compute_diagnostic_summary, write_diagnostic_summary
from slope_sim.logger import CsvSimulationLogger
from slope_sim.manual_control import ManualCommand, ManualControlSettings, command_from_keyboard
from slope_sim.metrics import compute_tracking_metrics
from slope_sim.model_registry import get_robot_model
from slope_sim.robot import DifferentialDriveRobot, create_robot
from slope_sim.scene import SceneInfo, configure_gui_visualizer, create_slope_scene, update_follow_camera
from slope_sim.simulation import (
    SimulationResult,
    _probe_terrain_for_robot,
    _read_lidar_for_robot,
    _robot_base_height,
    plot_feedback_figures,
    plot_trajectory,
)


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
    """一次切换后的有效世界和可供 Dashboard 显示的结果。"""

    world: ActiveManualWorld
    state_changed: bool
    world_reset: bool
    status_message: str
    error_message: str | None = None


def manual_step_limit(duration_limit_sec: float | None, time_step: float) -> int | None:
    """把显式运行时长转换为循环步数；None 表示一直运行到按退出键。"""
    if time_step <= 0:
        raise ValueError("time_step must be positive")
    if duration_limit_sec is None:
        return None
    if duration_limit_sec <= 0:
        raise ValueError("duration_limit_sec must be positive")
    return max(1, int(duration_limit_sec / time_step))


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
            p.removeBody(robot.robot_id, physicsClientId=client_id)
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
        p.removeBody(current.robot.robot_id, physicsClientId=client_id)
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
    active_robot = load_manual_robot(client_id, config, scene, robot_model=robot_model)
    return ActiveManualWorld(scene=scene, active_robot=active_robot, terrain=terrain)


def replace_manual_world_robot(
    client_id: int,
    config: ExperimentConfig,
    world: ActiveManualWorld,
    robot_model: str,
    *,
    force: bool = False,
) -> ManualSwitchResult:
    """在保留当前地形的前提下事务式替换活动车辆。"""
    target_model = robot_model.lower()
    if not force and target_model == world.active_robot.robot_model:
        return ManualSwitchResult(world, False, False, "当前车型已生效")
    try:
        replacement = load_manual_robot(client_id, config, world.scene, robot_model=target_model)
    except Exception as exc:
        message = f"应用车型失败，已保留 {world.active_robot.robot_model}: {exc}"
        return ManualSwitchResult(world, False, False, message, error_message=str(exc))

    try:
        p.removeBody(world.active_robot.robot.robot_id, physicsClientId=client_id)
    except Exception as exc:
        p.removeBody(replacement.robot.robot_id, physicsClientId=client_id)
        message = f"应用车型失败，旧车型未删除: {exc}"
        return ManualSwitchResult(world, False, False, message, error_message=str(exc))

    updated = ActiveManualWorld(scene=world.scene, active_robot=replacement, terrain=world.terrain)
    action = "车辆已复位" if force else f"车型已切换为 {target_model}"
    return ManualSwitchResult(updated, True, False, action)


def rebuild_manual_world(
    client_id: int,
    config: ExperimentConfig,
    world: ActiveManualWorld,
    target_terrain: TerrainSelection,
) -> ManualSwitchResult:
    """重建目标场地；失败时用上一个有效选择恢复完整世界。"""
    robot_model = world.active_robot.robot_model
    try:
        updated = load_manual_world(client_id, config, target_terrain, robot_model)
    except Exception as target_exc:
        try:
            restored = load_manual_world(client_id, config, world.terrain, robot_model)
        except Exception as rollback_exc:
            raise RuntimeError(
                f"target terrain failed ({target_exc}); rollback also failed ({rollback_exc})"
            ) from rollback_exc
        message = f"应用场地失败，已恢复 {world.terrain.terrain_model}: {target_exc}"
        return ManualSwitchResult(restored, True, True, message, error_message=str(target_exc))

    return ManualSwitchResult(
        updated,
        True,
        True,
        f"场地已切换为 {target_terrain.terrain_model}",
    )


def apply_manual_switch_request(
    client_id: int,
    config: ExperimentConfig,
    world: ActiveManualWorld,
    command: DashboardCommand,
) -> ManualSwitchResult:
    """在物理主线程内按场地、车型、复位顺序执行至多一次变更。"""
    if command.requested_terrain is not None:
        if command.requested_terrain == world.terrain:
            return ManualSwitchResult(world, False, False, "当前场地已生效")
        return rebuild_manual_world(client_id, config, world, command.requested_terrain)
    if command.requested_robot_model is not None:
        return replace_manual_world_robot(client_id, config, world, command.requested_robot_model)
    if command.reset_requested:
        return replace_manual_world_robot(
            client_id,
            config,
            world,
            world.active_robot.robot_model,
            force=True,
        )
    return ManualSwitchResult(world, False, False, "就绪")


def process_manual_scene_action(
    client_id: int,
    config: ExperimentConfig,
    world: ActiveManualWorld,
    command: DashboardCommand,
    dashboard: TelemetryDashboard | None = None,
) -> ManualSwitchResult:
    """在单个物理帧内先停旧车，再执行切换并同步 GUI 状态。"""
    if dashboard is not None:
        dashboard.set_switch_busy(True, "应用中")
        dashboard.process_events()

    # 切换前显式撤销旧车轮速，避免移除车体或重建世界时残留驱动力。
    world.active_robot.robot.command_twist(0.0, 0.0, dt=config.time_step)
    try:
        result = apply_manual_switch_request(client_id, config, world, command)
    except Exception as exc:
        if dashboard is not None:
            dashboard.show_switch_status(f"切换失败: {exc}", is_error=True)
        raise

    if result.world_reset:
        configure_gui_visualizer(
            client_id,
            config.camera_distance,
            config.camera_yaw,
            config.camera_pitch,
            config.camera_target,
        )
    if dashboard is not None:
        dashboard.sync_active_selection(result.world.active_robot.robot_model, result.world.terrain)
        if result.state_changed:
            dashboard.reset_feedback_history()
        dashboard.show_switch_status(
            result.status_message,
            is_error=result.error_message is not None,
        )
    return result


def merge_manual_commands(dashboard_command: DashboardCommand, keyboard_command: ManualCommand) -> DashboardCommand:
    """Dashboard 无输入时，允许 PyBullet 窗口焦点下的物理键盘接管控制。"""
    if keyboard_command.should_exit:
        return DashboardCommand(
            0.0,
            0.0,
            should_exit=True,
            requested_robot_model=dashboard_command.requested_robot_model,
            reset_requested=dashboard_command.reset_requested,
            requested_terrain=dashboard_command.requested_terrain,
            camera_follow_enabled=dashboard_command.camera_follow_enabled,
            camera_follow_view=dashboard_command.camera_follow_view,
        )
    dashboard_has_scene_action = (
        dashboard_command.requested_terrain is not None
        or dashboard_command.requested_robot_model is not None
        or dashboard_command.reset_requested
    )
    if dashboard_has_scene_action:
        return DashboardCommand(
            0.0,
            0.0,
            should_exit=dashboard_command.should_exit,
            requested_robot_model=dashboard_command.requested_robot_model,
            reset_requested=dashboard_command.reset_requested,
            requested_terrain=dashboard_command.requested_terrain,
            camera_follow_enabled=dashboard_command.camera_follow_enabled,
            camera_follow_view=dashboard_command.camera_follow_view,
        )
    dashboard_has_motion = abs(dashboard_command.linear_velocity) > 1e-12 or abs(dashboard_command.angular_velocity) > 1e-12
    if dashboard_has_motion or dashboard_command.should_exit:
        return dashboard_command
    keyboard_has_motion = abs(keyboard_command.linear_velocity) > 1e-12 or abs(keyboard_command.angular_velocity) > 1e-12
    if not keyboard_has_motion:
        return dashboard_command
    return DashboardCommand(
        keyboard_command.linear_velocity,
        keyboard_command.angular_velocity,
        should_exit=dashboard_command.should_exit,
        requested_robot_model=dashboard_command.requested_robot_model,
        reset_requested=dashboard_command.reset_requested,
        requested_terrain=dashboard_command.requested_terrain,
        camera_follow_enabled=dashboard_command.camera_follow_enabled,
        camera_follow_view=dashboard_command.camera_follow_view,
    )


def limit_manual_command_step(
    previous_command: DashboardCommand,
    target_command: DashboardCommand,
    dt: float,
    linear_acceleration_limit: float,
    angular_acceleration_limit: float,
) -> DashboardCommand:
    """限制手动速度命令变化率，减少急停或反向造成的轮胎冲击尖峰。"""
    if dt <= 0:
        raise ValueError("dt must be positive")
    if linear_acceleration_limit <= 0:
        raise ValueError("linear_acceleration_limit must be positive")
    if angular_acceleration_limit <= 0:
        raise ValueError("angular_acceleration_limit must be positive")
    if (
        target_command.should_exit
        or target_command.reset_requested
        or target_command.requested_robot_model is not None
        or target_command.requested_terrain is not None
    ):
        return DashboardCommand(
            0.0,
            0.0,
            should_exit=target_command.should_exit,
            requested_robot_model=target_command.requested_robot_model,
            reset_requested=target_command.reset_requested,
            requested_terrain=target_command.requested_terrain,
            camera_follow_enabled=target_command.camera_follow_enabled,
            camera_follow_view=target_command.camera_follow_view,
        )
    return DashboardCommand(
        _step_toward(previous_command.linear_velocity, target_command.linear_velocity, linear_acceleration_limit * dt),
        _step_toward(previous_command.angular_velocity, target_command.angular_velocity, angular_acceleration_limit * dt),
        should_exit=target_command.should_exit,
        requested_robot_model=target_command.requested_robot_model,
        reset_requested=target_command.reset_requested,
        requested_terrain=target_command.requested_terrain,
        camera_follow_enabled=target_command.camera_follow_enabled,
        camera_follow_view=target_command.camera_follow_view,
    )


def _step_toward(current: float, target: float, max_delta: float) -> float:
    """把一个标量按最大步长推向目标值。"""
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + max_delta * (1.0 if delta > 0.0 else -1.0)


def run_manual_demo(config: ExperimentConfig, *, duration_limit_sec: float | None = None) -> SimulationResult:
    """启动 PyBullet GUI，使用方向键手动控制当前地形中的车辆。"""
    if config.mode != "gui":
        raise ValueError("manual demo requires GUI mode; use --gui --manual")

    client_id = p.connect(p.GUI)
    if client_id < 0:
        raise RuntimeError("Failed to connect to PyBullet GUI")

    logger: CsvSimulationLogger | None = None
    dashboard: TelemetryDashboard | None = None
    try:
        # 场景和机器人仍复用自动仿真的构建逻辑，避免两套世界配置不一致。
        terrain = TerrainSelection(
            terrain_model=config.terrain_model,
            slope_deg=config.slope_deg,
            golf_seed=config.golf_seed,
            golf_relief=config.golf_relief,
        )
        world = load_manual_world(client_id, config, terrain, config.robot_model)
        scene = world.scene
        active_robot = world.active_robot
        robot = active_robot.robot
        configure_gui_visualizer(
            client_id,
            config.camera_distance,
            config.camera_yaw,
            config.camera_pitch,
            config.camera_target,
        )
        manual_prefix = f"manual_{config.terrain_model}_{active_robot.robot_model}_{config.slope_deg:g}"
        logger = CsvSimulationLogger(config.log_dir, prefix=manual_prefix)

        linear_slider = None
        angular_slider = None
        if config.dashboard_enabled:
            try:
                dashboard = TelemetryDashboard(
                    max_linear_speed=config.target_linear_velocity,
                    max_angular_speed=0.8,
                    update_hz=config.dashboard_update_hz,
                    smoothing_alpha=config.dashboard_smoothing_alpha,
                    robot_model=active_robot.robot_model,
                    model_switch_enabled=True,
                    terrain_model=terrain.terrain_model,
                    slope_deg=terrain.slope_deg,
                    golf_seed=terrain.golf_seed,
                    golf_relief=terrain.golf_relief,
                    terrain_switch_enabled=True,
                    plot_update_hz=config.dashboard_plot_update_hz,
                    plot_window_sec=config.dashboard_plot_window_sec,
                    plot_snapshot_dir=config.figure_dir,
                    camera_follow_enabled=config.camera_follow_enabled,
                    camera_follow_view=config.camera_follow_view,
                )
            except Exception as exc:
                print(f"Dashboard unavailable, fallback to PyBullet debug sliders: {exc}")

        if dashboard is None:
            # PyBullet 自带调参滑条作为兜底，不额外依赖侧窗。
            linear_slider = p.addUserDebugParameter("max linear speed [m/s]", 0.0, 1.2, config.target_linear_velocity)
            angular_slider = p.addUserDebugParameter("max angular speed [rad/s]", 0.0, 2.0, 0.8)

        max_steps = manual_step_limit(duration_limit_sec, config.time_step)
        step = 0
        out_of_bounds_latched = False
        # Dashboard 不可用时，持续沿用配置相机状态，不能被命令默认值覆盖。
        camera_follow_enabled = config.camera_follow_enabled
        camera_follow_view = config.camera_follow_view
        limited_command = DashboardCommand(
            0.0,
            0.0,
            camera_follow_enabled=camera_follow_enabled,
            camera_follow_view=camera_follow_view,
        )
        while max_steps is None or step < max_steps:
            if dashboard is not None:
                dashboard.process_events()
                command = dashboard.current_command()
                # Dashboard 窗口和 PyBullet 窗口谁拿到焦点，都能用物理方向键控制。
                settings = ManualControlSettings(
                    max_linear_speed=dashboard.linear_spin.value(),
                    max_angular_speed=dashboard.angular_spin.value(),
                )
                keyboard_command = command_from_keyboard(p.getKeyboardEvents(), settings)
                command = merge_manual_commands(command, keyboard_command)
                camera_follow_enabled = command.camera_follow_enabled
                camera_follow_view = command.camera_follow_view
            else:
                # 每一步都读取滑条值，这样运行中可以即时调整速度上限。
                max_linear_speed = p.readUserDebugParameter(linear_slider)
                max_angular_speed = p.readUserDebugParameter(angular_slider)
                settings = ManualControlSettings(max_linear_speed=max_linear_speed, max_angular_speed=max_angular_speed)
                # 方向键只在 PyBullet 窗口获得焦点时生效。
                keyboard_command = command_from_keyboard(p.getKeyboardEvents(), settings)
                command = DashboardCommand(
                    keyboard_command.linear_velocity,
                    keyboard_command.angular_velocity,
                    should_exit=keyboard_command.should_exit,
                    camera_follow_enabled=camera_follow_enabled,
                    camera_follow_view=camera_follow_view,
                )
            if command.should_exit:
                break

            scene_action_requested = (
                command.requested_terrain is not None
                or command.requested_robot_model is not None
                or command.reset_requested
            )
            if scene_action_requested:
                switch_result = process_manual_scene_action(client_id, config, world, command, dashboard)
                world = switch_result.world
                scene = world.scene
                terrain = world.terrain
                active_robot = world.active_robot
                robot = active_robot.robot
                out_of_bounds_latched = False
                limited_command = DashboardCommand(
                    0.0,
                    0.0,
                    camera_follow_enabled=camera_follow_enabled,
                    camera_follow_view=camera_follow_view,
                )

            target_command = DashboardCommand(
                0.0 if out_of_bounds_latched or scene_action_requested else command.linear_velocity,
                0.0 if out_of_bounds_latched or scene_action_requested else command.angular_velocity,
                should_exit=command.should_exit,
                camera_follow_enabled=camera_follow_enabled,
                camera_follow_view=camera_follow_view,
            )
            if out_of_bounds_latched or scene_action_requested:
                limited_command = target_command
            else:
                limited_command = limit_manual_command_step(
                    limited_command,
                    target_command,
                    config.time_step,
                    config.manual_linear_acceleration_limit,
                    config.manual_angular_acceleration_limit,
                )
            linear_velocity = limited_command.linear_velocity
            angular_velocity = limited_command.angular_velocity
            robot.command_twist(linear_velocity, angular_velocity, dt=config.time_step)
            t = step * config.time_step
            if config.drive_model == "physics":
                p.stepSimulation(physicsClientId=client_id)
                lidar_summary = _read_lidar_for_robot(client_id, robot, config)
                state = robot.read_physics_state(
                    t=t,
                    command_linear_velocity=linear_velocity,
                    command_angular_velocity=angular_velocity,
                    ground_lateral_friction=config.ground_lateral_friction,
                    drive_lateral_friction=config.drive_lateral_friction,
                    ground_rolling_friction=config.ground_rolling_friction,
                    ground_spinning_friction=config.ground_spinning_friction,
                    support_lateral_friction=config.support_lateral_friction,
                    robot_model=active_robot.robot_model,
                    terrain_type=scene.terrain_type,
                    terrain_probe=_probe_terrain_for_robot(client_id, robot, scene),
                    lidar_summary=lidar_summary,
                )
            else:
                # 当前阶段保留运动学推进，保证初学阶段的轨迹稳定、容易观察。
                state = robot.step_kinematic(dt=config.time_step, slope_deg=scene.slope_deg, t=t)
                p.stepSimulation(physicsClientId=client_id)
            out_of_bounds_latched = state.out_of_bounds
            if camera_follow_enabled:
                update_follow_camera(
                    client_id,
                    robot.robot_id,
                    config.camera_distance,
                    config.camera_pitch,
                    config.camera_yaw,
                    camera_follow_view,
                )
            if dashboard is not None:
                dashboard.update(state)
            logger.record(
                state,
                reference_x=0.0,
                reference_y=0.0,
                estimated_x=state.x,
                estimated_y=state.y,
            )
            time.sleep(config.time_step)
            step += 1

        log_path = logger.close()
        logger = None
    finally:
        if logger is not None:
            log_path = logger.close()
        if dashboard is not None:
            dashboard.close()
        # 不管中途是否退出，都要断开 PyBullet，避免 GUI/物理客户端残留。
        p.disconnect(client_id)

    frame = pd.read_csv(log_path)
    diagnostic_summary = compute_diagnostic_summary(frame)
    diagnostic_summary_path = write_diagnostic_summary(log_path, diagnostic_summary)
    metrics = compute_tracking_metrics(frame, final_reference_yaw=0.0)
    figure_prefix = Path(log_path).stem
    figure_path = plot_trajectory(frame, config.figure_dir, prefix=figure_prefix)
    feedback_figure_paths = plot_feedback_figures(frame, config.figure_dir, prefix=figure_prefix)
    return SimulationResult(
        log_path=log_path,
        figure_path=figure_path,
        metrics=metrics,
        feedback_figure_paths=feedback_figure_paths,
        diagnostic_summary=diagnostic_summary.to_dict(),
        diagnostic_summary_path=diagnostic_summary_path,
    )
