# GUI 手动演示模块：打开 PyBullet 窗口，用方向键控制差速车在平地中运动。
from __future__ import annotations

import time

import pandas as pd
import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.dashboard import TelemetryDashboard
from slope_sim.logger import CsvSimulationLogger
from slope_sim.manual_control import ManualControlSettings, command_from_keyboard
from slope_sim.metrics import compute_tracking_metrics
from slope_sim.robot import DifferentialDriveRobot
from slope_sim.scene import configure_gui_visualizer, create_slope_scene
from slope_sim.simulation import (
    SimulationResult,
    _probe_terrain_for_robot,
    _read_lidar_for_robot,
    _robot_base_height,
    _robot_urdf_path,
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


def run_manual_demo(config: ExperimentConfig, *, duration_limit_sec: float | None = None) -> SimulationResult:
    """启动 PyBullet GUI，使用方向键手动控制平地差速车。"""
    if config.mode != "gui":
        raise ValueError("manual demo requires GUI mode; use --gui --manual")

    client_id = p.connect(p.GUI)
    if client_id < 0:
        raise RuntimeError("Failed to connect to PyBullet GUI")

    logger: CsvSimulationLogger | None = None
    dashboard: TelemetryDashboard | None = None
    try:
        # 场景和机器人仍复用自动仿真的构建逻辑，避免两套世界配置不一致。
        scene = create_slope_scene(
            client_id,
            config.slope_deg,
            config.time_step,
            config.ground_lateral_friction,
            config.ground_rolling_friction,
            config.ground_spinning_friction,
            config.terrain_model,
        )
        configure_gui_visualizer(
            client_id,
            config.camera_distance,
            config.camera_yaw,
            config.camera_pitch,
            config.camera_target,
        )
        robot = DifferentialDriveRobot(
            client_id=client_id,
            urdf_path=_robot_urdf_path(config.robot_model),
            wheel_base=config.wheel_base,
            wheel_radius=config.wheel_radius,
            base_height=_robot_base_height(config.robot_model),
        )
        robot.apply_drive_friction(config.drive_lateral_friction, config.support_lateral_friction)
        logger = CsvSimulationLogger(config.log_dir, prefix=f"manual_flat_{config.slope_deg:g}")

        linear_slider = None
        angular_slider = None
        if config.dashboard_enabled:
            try:
                dashboard = TelemetryDashboard(
                    max_linear_speed=config.target_linear_velocity,
                    max_angular_speed=0.8,
                    update_hz=config.dashboard_update_hz,
                    smoothing_alpha=config.dashboard_smoothing_alpha,
                )
            except Exception as exc:
                print(f"Dashboard unavailable, fallback to PyBullet debug sliders: {exc}")

        if dashboard is None:
            # PyBullet 自带调参滑条作为兜底，不额外依赖侧窗。
            linear_slider = p.addUserDebugParameter("max linear speed [m/s]", 0.0, 1.2, config.target_linear_velocity)
            angular_slider = p.addUserDebugParameter("max angular speed [rad/s]", 0.0, 2.0, 0.8)

        max_steps = manual_step_limit(duration_limit_sec, config.time_step)
        step = 0
        while max_steps is None or step < max_steps:
            if dashboard is not None:
                dashboard.process_events()
                command = dashboard.current_command()
            else:
                # 每一步都读取滑条值，这样运行中可以即时调整速度上限。
                max_linear_speed = p.readUserDebugParameter(linear_slider)
                max_angular_speed = p.readUserDebugParameter(angular_slider)
                settings = ManualControlSettings(max_linear_speed=max_linear_speed, max_angular_speed=max_angular_speed)
                # 方向键只在 PyBullet 窗口获得焦点时生效。
                command = command_from_keyboard(p.getKeyboardEvents(), settings)
            if command.should_exit:
                break

            robot.command_twist(command.linear_velocity, command.angular_velocity)
            t = step * config.time_step
            if config.drive_model == "physics":
                p.stepSimulation(physicsClientId=client_id)
                lidar_summary = _read_lidar_for_robot(client_id, robot, config)
                state = robot.read_physics_state(
                    t=t,
                    command_linear_velocity=command.linear_velocity,
                    command_angular_velocity=command.angular_velocity,
                    ground_lateral_friction=config.ground_lateral_friction,
                    drive_lateral_friction=config.drive_lateral_friction,
                    ground_rolling_friction=config.ground_rolling_friction,
                    ground_spinning_friction=config.ground_spinning_friction,
                    support_lateral_friction=config.support_lateral_friction,
                    terrain_type=scene.terrain_type,
                    terrain_probe=_probe_terrain_for_robot(client_id, robot),
                    lidar_summary=lidar_summary,
                )
            else:
                # 当前阶段保留运动学推进，保证初学阶段的轨迹稳定、容易观察。
                state = robot.step_kinematic(dt=config.time_step, slope_deg=config.slope_deg, t=t)
                p.stepSimulation(physicsClientId=client_id)
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
    metrics = compute_tracking_metrics(frame, final_reference_yaw=0.0)
    figure_path = plot_trajectory(frame, config.figure_dir, prefix=f"manual_flat_{config.slope_deg:g}")
    feedback_figure_paths = plot_feedback_figures(frame, config.figure_dir, prefix=f"manual_flat_{config.slope_deg:g}")
    return SimulationResult(log_path=log_path, figure_path=figure_path, metrics=metrics, feedback_figure_paths=feedback_figure_paths)
