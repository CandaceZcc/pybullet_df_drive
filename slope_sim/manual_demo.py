# GUI 手动演示模块：打开 PyBullet 窗口，用方向键控制车辆在当前地形中运动。
from __future__ import annotations

from pathlib import Path
import time

import pandas as pd
import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import SimulationCoordinator, create_obstacle_manager, load_manual_robot, load_manual_world, reload_manual_robot
from slope_sim.dashboard import DashboardCommand, TelemetryDashboard
from slope_sim.diagnostics import compute_diagnostic_summary, write_diagnostic_summary
from slope_sim.logger import CsvSimulationLogger
from slope_sim.manual_control import ManualCommand, ManualControlSettings, command_from_keyboard
from slope_sim.metrics import compute_tracking_metrics
from slope_sim.runtime_actions import TerrainSelection, is_safe_stop_action
from slope_sim.scene import configure_gui_visualizer, update_follow_camera
from slope_sim.simulation import (
    SimulationResult,
    _probe_terrain_for_robot,
    _read_lidar_for_robot,
    plot_feedback_figures,
    plot_trajectory,
)


def manual_step_limit(duration_limit_sec: float | None, time_step: float) -> int | None:
    """把显式运行时长转换为循环步数；None 表示一直运行到按退出键。"""
    if time_step <= 0:
        raise ValueError("time_step must be positive")
    if duration_limit_sec is None:
        return None
    if duration_limit_sec <= 0:
        raise ValueError("duration_limit_sec must be positive")
    return max(1, int(duration_limit_sec / time_step))




def merge_manual_commands(dashboard_command: DashboardCommand, keyboard_command: ManualCommand) -> DashboardCommand:
    """Dashboard 无输入时，允许 PyBullet 窗口焦点下的物理键盘接管控制。"""
    if keyboard_command.should_exit:
        return DashboardCommand(
            0.0,
            0.0,
            should_exit=True,
            structural_action=dashboard_command.structural_action,
            camera_follow_enabled=dashboard_command.camera_follow_enabled,
            camera_follow_view=dashboard_command.camera_follow_view,
        )
    dashboard_has_safe_stop_action = is_safe_stop_action(dashboard_command.structural_action)
    if dashboard_has_safe_stop_action:
        return DashboardCommand(
            0.0,
            0.0,
            should_exit=dashboard_command.should_exit,
            structural_action=dashboard_command.structural_action,
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
        structural_action=dashboard_command.structural_action,
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
        or is_safe_stop_action(target_command.structural_action)
    ):
        return DashboardCommand(
            0.0,
            0.0,
            should_exit=target_command.should_exit,
            structural_action=target_command.structural_action,
            camera_follow_enabled=target_command.camera_follow_enabled,
            camera_follow_view=target_command.camera_follow_view,
        )
    return DashboardCommand(
        _step_toward(previous_command.linear_velocity, target_command.linear_velocity, linear_acceleration_limit * dt),
        _step_toward(previous_command.angular_velocity, target_command.angular_velocity, angular_acceleration_limit * dt),
        should_exit=target_command.should_exit,
        structural_action=target_command.structural_action,
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
        coordinator = SimulationCoordinator(client_id, config, world, create_obstacle_manager(client_id, world))
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
        handled_result = None
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

            structural_action = command.structural_action
            safe_stop_requested = is_safe_stop_action(structural_action)
            if structural_action is not None:
                coordinator.enqueue(structural_action)
                if dashboard is not None:
                    dashboard.set_switch_busy(True, "应用中")

            target_command = DashboardCommand(
                0.0 if out_of_bounds_latched or safe_stop_requested else command.linear_velocity,
                0.0 if out_of_bounds_latched or safe_stop_requested else command.angular_velocity,
                should_exit=command.should_exit,
                structural_action=structural_action,
                camera_follow_enabled=camera_follow_enabled,
                camera_follow_view=camera_follow_view,
            )
            if out_of_bounds_latched or safe_stop_requested:
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
                try:
                    result = coordinator.step(config.time_step)
                except Exception as exc:
                    if dashboard is not None:
                        dashboard.show_switch_status(f"切换失败: {exc}", is_error=True)
                    raise
                world = coordinator.world
                scene = world.scene
                terrain = world.terrain
                active_robot = world.active_robot
                robot = active_robot.robot
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
                try:
                    result = coordinator.step(config.time_step)
                except Exception as exc:
                    if dashboard is not None:
                        dashboard.show_switch_status(f"切换失败: {exc}", is_error=True)
                    raise
                world = coordinator.world
                scene = world.scene
                terrain = world.terrain
                active_robot = world.active_robot
                robot = active_robot.robot
            if result is not None and result is not handled_result:
                handled_result = result
                if result.world_reset:
                    configure_gui_visualizer(
                        client_id,
                        config.camera_distance,
                        config.camera_yaw,
                        config.camera_pitch,
                        config.camera_target,
                    )
                    out_of_bounds_latched = False
                    limited_command = DashboardCommand(
                        0.0,
                        0.0,
                        camera_follow_enabled=camera_follow_enabled,
                        camera_follow_view=camera_follow_view,
                    )
                if dashboard is not None:
                    dashboard.sync_active_selection(result.world.active_robot.robot_model, result.world.terrain)
                    if result.state_changed:
                        dashboard.reset_feedback_history()
                    dashboard.show_switch_status(
                        result.status_message,
                        is_error=result.error_message is not None,
                    )
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
