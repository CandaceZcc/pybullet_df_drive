# 项目主入口：解析命令行参数，选择自动仿真或 GUI 手动控制模式。
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from slope_sim.config import load_config
from slope_sim.manual_demo import run_manual_demo
from slope_sim.model_registry import robot_model_names
from slope_sim.scene import terrain_model_names
from slope_sim.simulation import run_experiment


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """定义命令行参数，支持配置覆盖、GUI 和手动控制开关。"""
    parser = argparse.ArgumentParser(description="Run a PyBullet differential-drive slope simulation.")
    parser.add_argument("--config", default="configs/experiment.yaml", help="Path to an experiment YAML file.")
    parser.add_argument("--mode", choices=["direct", "gui"], default=None, help="PyBullet connection mode.")
    parser.add_argument("--gui", action="store_true", help="Shortcut for --mode gui.")
    parser.add_argument("--manual", action="store_true", help="Use PyBullet GUI keyboard control.")
    parser.add_argument("--slope-deg", type=float, default=None, help="Slope angle in degrees.")
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=None,
        help="Simulation duration in seconds; manual mode runs until q/Esc unless this is passed.",
    )
    parser.add_argument("--time-step", type=float, default=None, help="Simulation time step in seconds.")
    parser.add_argument("--target-linear-velocity", type=float, default=None, help="Target body velocity in m/s.")
    parser.add_argument("--target-angular-velocity", type=float, default=None, help="Target yaw rate in rad/s.")
    parser.add_argument("--robot-model", choices=robot_model_names(), default=None, help="Robot URDF model.")
    parser.add_argument("--drive-model", choices=["physics"], default=None, help="PyBullet joint physics model.")
    parser.add_argument("--interface-mode", choices=["auto", "ecal", "local"], default=None, help="Interface transport mode.")
    parser.add_argument("--no-interface", action="store_true", help="Disable the enterprise interface.")
    parser.add_argument("--no-interface-log", action="store_true", help="Disable enterprise interface logs.")
    parser.add_argument("--scene-in", type=Path, default=None, help="Input scene document path.")
    parser.add_argument("--scene-out", type=Path, default=None, help="Exported scene document path.")
    parser.add_argument("--developer-diagnostics", action="store_true", help="Enable developer diagnostics.")
    parser.add_argument("--no-dashboard", action="store_true", help="Disable the PySide6 telemetry dashboard.")
    parser.add_argument("--dashboard-update-hz", type=float, default=None, help="Telemetry dashboard display refresh rate.")
    parser.add_argument("--dashboard-smoothing-alpha", type=float, default=None, help="Dashboard feedback smoothing alpha.")
    parser.add_argument("--lidar", action="store_true", help="Enable the simple ray-cast LiDAR.")
    parser.add_argument("--lidar-debug-draw", action="store_true", help="Draw LiDAR rays in the PyBullet GUI.")
    parser.add_argument("--terrain-model", choices=terrain_model_names(), default=None, help="Terrain model.")
    parser.add_argument("--golf-seed", type=int, default=None, help="Reproducible golf heightfield seed.")
    parser.add_argument("--golf-relief", choices=["low", "medium", "high"], default=None, help="Golf terrain relief preset.")
    parser.add_argument("--ground-friction", type=float, default=None, help="Ground lateral friction coefficient.")
    parser.add_argument("--ground-rolling-friction", type=float, default=None, help="Ground rolling friction coefficient.")
    parser.add_argument("--ground-spinning-friction", type=float, default=None, help="Ground spinning friction coefficient.")
    parser.add_argument("--wheel-friction", type=float, default=None, help="Drive-wheel lateral friction coefficient.")
    parser.add_argument("--support-friction", type=float, default=None, help="Caster/support lateral friction coefficient.")
    parser.add_argument("--drive-motor-force", type=float, default=None, help="Velocity motor force for each driven wheel.")
    parser.add_argument("--log-dir", type=Path, default=None, help="Directory for CSV logs.")
    parser.add_argument("--figure-dir", type=Path, default=None, help="Directory for generated figures.")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """程序主流程：加载配置、运行仿真、打印输出文件和误差指标。"""
    args = parse_args(argv)
    overrides = {
        "mode": args.mode,
        "gui": args.gui,
        "slope_deg": args.slope_deg,
        "duration_sec": args.duration_sec,
        "time_step": args.time_step,
        "target_linear_velocity": args.target_linear_velocity,
        "target_angular_velocity": args.target_angular_velocity,
        "robot_model": args.robot_model,
        "drive_model": args.drive_model,
        "interface_mode": args.interface_mode,
        "no_interface": args.no_interface,
        "no_interface_log": args.no_interface_log,
        "scene_in": args.scene_in,
        "scene_out": args.scene_out,
        "developer_diagnostics": args.developer_diagnostics,
        "no_dashboard": args.no_dashboard,
        "dashboard_update_hz": args.dashboard_update_hz,
        "dashboard_smoothing_alpha": args.dashboard_smoothing_alpha,
        "lidar": args.lidar,
        "lidar_debug_draw": args.lidar_debug_draw if args.lidar_debug_draw else None,
        "terrain_model": args.terrain_model,
        "golf_seed": args.golf_seed,
        "golf_relief": args.golf_relief,
        "ground_friction": args.ground_friction,
        "ground_rolling_friction": args.ground_rolling_friction,
        "ground_spinning_friction": args.ground_spinning_friction,
        "wheel_friction": args.wheel_friction,
        "support_friction": args.support_friction,
        "drive_motor_force": args.drive_motor_force,
        "log_dir": args.log_dir,
        "figure_dir": args.figure_dir,
    }
    config = load_config(args.config, overrides=overrides)
    # 手动模式必须使用 PyBullet GUI；普通实验仍走 DIRECT/GUI 自动仿真路径。
    if args.manual:
        result = run_manual_demo(config, duration_limit_sec=args.duration_sec)
    else:
        result = run_experiment(config)
    print(f"log: {result.log_path}")
    print(f"figure: {result.figure_path}")
    for figure_path in result.feedback_figure_paths:
        print(f"feedback_figure: {figure_path}")
    if result.diagnostic_summary_path is not None:
        print(f"diagnostic_summary: {result.diagnostic_summary_path}")
    if result.obstacle_event_log_path is not None:
        print(f"obstacle_event_log: {result.obstacle_event_log_path}")
    if result.interface_binary_log is not None:
        print(f"interface_binary_log: {result.interface_binary_log}")
    if result.interface_event_log is not None:
        print(f"interface_event_log: {result.interface_event_log}")
    if result.scene_export is not None:
        print(f"scene_export: {result.scene_export}")
    for name, value in result.metrics.items():
        print(f"{name}: {value:.6f}")
    if result.diagnostic_summary is not None:
        for name, value in result.diagnostic_summary.items():
            print(f"diagnostic_{name}: {value:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
