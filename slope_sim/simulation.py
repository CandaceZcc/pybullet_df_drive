# 自动仿真模块：运行一次固定速度实验，记录轨迹、绘图并计算误差。
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.diagnostics import compute_diagnostic_summary, write_diagnostic_summary
from slope_sim.logger import CsvSimulationLogger
from slope_sim.metrics import compute_tracking_metrics
from slope_sim.robot import DifferentialDriveRobot
from slope_sim.scene import SceneInfo, configure_gui_visualizer, create_slope_scene, probe_terrain, update_follow_camera
from slope_sim.sensors import LidarSummary, read_lidar


@dataclass(frozen=True)
class SimulationResult:
    """一次仿真的输出结果，包含日志、图像和误差指标。"""

    log_path: Path
    figure_path: Path
    metrics: dict[str, float]
    feedback_figure_paths: tuple[Path, ...] = ()
    diagnostic_summary: dict[str, float] | None = None
    diagnostic_summary_path: Path | None = None


def run_experiment(config: ExperimentConfig) -> SimulationResult:
    """运行一次自动仿真实验，并返回日志、图像和误差指标。"""
    connection_mode = p.GUI if config.mode == "gui" else p.DIRECT
    client_id = p.connect(connection_mode)
    if client_id < 0:
        raise RuntimeError(f"Failed to connect to PyBullet in {config.mode} mode")

    try:
        # PyBullet 客户端内创建场景和机器人，DIRECT 模式也能无窗口运行。
        scene = create_slope_scene(
            client_id,
            config.slope_deg,
            config.time_step,
            config.ground_lateral_friction,
            config.ground_rolling_friction,
            config.ground_spinning_friction,
            config.terrain_model,
            config.dam_toe_length,
            config.dam_slope_length,
            config.dam_crest_length,
            config.dam_exit_length,
            config.dam_width,
            config.dam_wall_height,
            config.terrain_guard_enabled,
        )
        if config.mode == "gui":
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
            start_x=scene.spawn_position[0],
            start_y=scene.spawn_position[1],
            base_height=scene.spawn_position[2] + _robot_base_height(config.robot_model),
            drive_motor_force=config.drive_motor_force,
            track_anisotropic_friction=config.track_anisotropic_friction,
            track_drive_mode=config.track_drive_mode,
        )
        robot.apply_drive_friction(config.drive_lateral_friction, config.support_lateral_friction)
        logger = CsvSimulationLogger(config.log_dir, prefix=f"slope_{config.slope_deg:g}")
        steps = max(1, int(config.duration_sec / config.time_step))

        # 当前阶段使用固定 v/w 命令；后续路径跟踪会在这里替换成控制器输出。
        for step in range(steps):
            t = step * config.time_step
            robot.command_twist(config.target_linear_velocity, config.target_angular_velocity)
            if config.drive_model == "physics":
                p.stepSimulation(physicsClientId=client_id)
                lidar_summary = _read_lidar_for_robot(client_id, robot, config)
                state = robot.read_physics_state(
                    t=t,
                    command_linear_velocity=config.target_linear_velocity,
                    command_angular_velocity=config.target_angular_velocity,
                    ground_lateral_friction=config.ground_lateral_friction,
                    drive_lateral_friction=config.drive_lateral_friction,
                    ground_rolling_friction=config.ground_rolling_friction,
                    ground_spinning_friction=config.ground_spinning_friction,
                    support_lateral_friction=config.support_lateral_friction,
                    robot_model=config.robot_model,
                    terrain_type=scene.terrain_type,
                    terrain_probe=_probe_terrain_for_robot(client_id, robot, scene),
                    lidar_summary=lidar_summary,
                )
            else:
                state = robot.step_kinematic(dt=config.time_step, slope_deg=config.slope_deg, t=t)
                p.stepSimulation(physicsClientId=client_id)
            if config.mode == "gui" and config.camera_follow_enabled:
                update_follow_camera(
                    client_id,
                    robot.robot_id,
                    config.camera_distance,
                    config.camera_pitch,
                    config.camera_yaw,
                    config.camera_follow_view,
                )
            reference_x = scene.spawn_position[0] + config.target_linear_velocity * t
            reference_y = scene.spawn_position[1]
            logger.record(
                state,
                reference_x=reference_x,
                reference_y=reference_y,
                estimated_x=state.x,
                estimated_y=state.y,
            )

        log_path = logger.close()
    finally:
        # 确保异常时也释放 PyBullet 连接。
        p.disconnect(client_id)

    frame = pd.read_csv(log_path)
    diagnostic_summary = compute_diagnostic_summary(frame)
    diagnostic_summary_path = write_diagnostic_summary(log_path, diagnostic_summary)
    metrics = compute_tracking_metrics(frame, final_reference_yaw=0.0)
    figure_path = plot_trajectory(frame, config.figure_dir, prefix=f"slope_{config.slope_deg:g}")
    feedback_figure_paths = plot_feedback_figures(frame, config.figure_dir, prefix=f"slope_{config.slope_deg:g}")
    return SimulationResult(
        log_path=log_path,
        figure_path=figure_path,
        metrics=metrics,
        feedback_figure_paths=feedback_figure_paths,
        diagnostic_summary=diagnostic_summary.to_dict(),
        diagnostic_summary_path=diagnostic_summary_path,
    )


def _robot_urdf_path(robot_model: str) -> Path:
    """根据配置选择机器人 URDF。"""
    if robot_model == "tracked_proxy":
        return Path("urdf/tracked_proxy.urdf")
    return Path("urdf/diff_drive.urdf")


def _robot_base_height(robot_model: str) -> float:
    """根据机器人模型选择贴近地面的初始车体高度。"""
    if robot_model == "tracked_proxy":
        return 0.16
    return 0.14


def _read_lidar_for_robot(client_id: int, robot: DifferentialDriveRobot, config: ExperimentConfig) -> LidarSummary:
    """从机器人车体附近发射简化 LiDAR 射线。"""
    if not config.lidar_enabled:
        return LidarSummary()
    position, orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
    yaw = p.getEulerFromQuaternion(orientation)[2]
    origin = (float(position[0]), float(position[1]), float(position[2]) + 0.12)
    return read_lidar(
        client_id,
        origin,
        yaw,
        config.lidar_ray_count,
        config.lidar_max_distance,
        config.lidar_fov_deg,
        draw_debug=config.mode == "gui" and config.lidar_debug_draw,
        life_time=max(config.time_step * 2.0, 0.05),
    )


def _probe_terrain_for_robot(client_id: int, robot: DifferentialDriveRobot, scene: SceneInfo | None = None):
    """用机器人当前位置做一次向下地形探测。"""
    position, _orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
    return probe_terrain(client_id, float(position[0]), float(position[1]), bounds=None if scene is None else scene.bounds)


def plot_trajectory(frame: pd.DataFrame, figure_dir: str | Path, prefix: str = "run") -> Path:
    """将参考轨迹、真实轨迹和估计轨迹画到同一张图中。"""
    figure_dir = Path(figure_dir)
    figure_dir.mkdir(parents=True, exist_ok=True)
    figure_path = figure_dir / f"{prefix}_trajectory.png"

    fig, ax = plt.subplots(figsize=(6.0, 4.0))
    ax.plot(frame["reference_x"], frame["reference_y"], label="reference", linestyle="--")
    ax.plot(frame["x"], frame["y"], label="actual")
    ax.plot(frame["estimated_x"], frame["estimated_y"], label="estimated", alpha=0.75)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Differential-drive slope tracking")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(figure_path, dpi=150)
    plt.close(fig)
    return figure_path


def plot_feedback_figures(frame: pd.DataFrame, figure_dir: str | Path, prefix: str = "run") -> tuple[Path, ...]:
    """生成阶段三反馈图：打滑曲线和接触/摩擦力曲线。"""
    figure_dir = Path(figure_dir)
    figure_dir.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    time_axis = frame["t"] if "t" in frame.columns else pd.Series(range(len(frame)))

    if {"left_slip_ratio", "right_slip_ratio", "left_slip_speed", "right_slip_speed"}.issubset(frame.columns):
        slip_path = figure_dir / f"{prefix}_slip.png"
        fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.0), sharex=True)
        axes[0].plot(time_axis, frame["left_slip_ratio"], label="left ratio")
        axes[0].plot(time_axis, frame["right_slip_ratio"], label="right ratio")
        axes[0].set_ylabel("slip ratio")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        axes[1].plot(time_axis, frame["left_slip_speed"], label="left speed")
        axes[1].plot(time_axis, frame["right_slip_speed"], label="right speed")
        axes[1].set_xlabel("t [s]" if "t" in frame.columns else "sample")
        axes[1].set_ylabel("slip speed [m/s]")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        fig.tight_layout()
        fig.savefig(slip_path, dpi=150)
        plt.close(fig)
        paths.append(slip_path)

    contact_columns = {
        "left_contact_normal_force",
        "right_contact_normal_force",
        "left_contact_friction_force",
        "right_contact_friction_force",
    }
    if contact_columns.issubset(frame.columns):
        contact_path = figure_dir / f"{prefix}_contact.png"
        fig, axes = plt.subplots(2, 1, figsize=(7.0, 5.0), sharex=True)
        axes[0].plot(time_axis, frame["left_contact_normal_force"], label="left normal")
        axes[0].plot(time_axis, frame["right_contact_normal_force"], label="right normal")
        axes[0].set_ylabel("normal force [N]")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()
        axes[1].plot(time_axis, frame["left_contact_friction_force"], label="left friction")
        axes[1].plot(time_axis, frame["right_contact_friction_force"], label="right friction")
        axes[1].set_xlabel("t [s]" if "t" in frame.columns else "sample")
        axes[1].set_ylabel("friction force [N]")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        fig.tight_layout()
        fig.savefig(contact_path, dpi=150)
        plt.close(fig)
        paths.append(contact_path)

    return tuple(paths)
