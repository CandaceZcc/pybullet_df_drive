# 项目主入口：解析命令行参数，选择自动仿真或 GUI 手动控制模式。
from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
from typing import Mapping, Sequence

from slope_sim.config import load_config
from slope_sim.manual_demo import run_manual_demo
from slope_sim.model_registry import robot_model_names
from slope_sim.scene import terrain_model_names
from slope_sim.simulation import run_experiment
from slope_sim.interfaces.v2.ecal_preflight import (
    EcalPreflightReport,
    LegacyInterfaceModeError,
    require_v2_interface_mode,
    run_v2_ecal_preflight,
)
from slope_sim.interfaces.v2.runsim_v2_command import RunSimV2Command
from slope_sim.serial_rc import CommandSourceArbiter, pyserial_opener, start_rc_worker


def resolve_v2_runtime_root(environment: Mapping[str, str] | None = None) -> Path:
    """定位 Command 与 eCAL 资源；源码验收可显式指定非发行运行根。"""
    env = os.environ if environment is None else environment
    configured = env.get("SLOPE_SIM_V2_RUNTIME_ROOT")
    if not configured:
        return Path(__file__).resolve().parent
    root = Path(configured).expanduser().resolve()
    if not root.is_dir():
        raise ValueError("SLOPE_SIM_V2_RUNTIME_ROOT must be an existing directory")
    return root


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
    parser.add_argument("--v2-realtime", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--ecal-config", type=Path, default=None, help="正式 v2 eCAL 配置文件路径（覆盖 ECAL_CONFIG_PATH）。")
    parser.add_argument("--ecal-time-plugin-path", type=Path, default=None, help="正式 v2 eCAL time-sync 插件目录（覆盖 ECAL_TIME_PLUGIN_PATH）。")
    parser.add_argument("--capture-output-dir", type=Path, default=None, help="正式 v2 C++ Recorder/Export 采集输出目录。")
    parser.add_argument("--capture-duration-sec", type=int, choices=(60, 90, 180), default=None, help="启动正式 v2 采集并在 60、90 或 180 秒后自动导出。")
    parser.add_argument("--viewer-root", type=Path, default=None, help="本地 Livox Viewer 2 安装根目录（用于打开导出的 LVX2）。")
    parser.add_argument("--open-ros-rviz", action="store_true", help="启动时打开独立 ROS Bridge/RViz2 实时点云显示。")
    parser.add_argument("--rc-port", type=Path, default=None, help="稳定 /dev/serial/by-id 遥控器路径；通过 SBUS 资格检查后启用。")
    parser.add_argument("--no-interface", action="store_true", help="Disable the enterprise interface.")
    parser.add_argument("--no-interface-log", action="store_true", help="Disable enterprise interface logs.")
    parser.add_argument("--scene-in", type=Path, default=None, help="Input scene document path.")
    parser.add_argument("--scene-out", type=Path, default=None, help="Exported scene document path.")
    parser.add_argument("--developer-diagnostics", action="store_true", help="Enable developer diagnostics.")
    parser.add_argument("--no-dashboard", action="store_true", help="Disable the PySide6 telemetry dashboard.")
    parser.add_argument("--dashboard-update-hz", type=float, default=None, help="Telemetry dashboard display refresh rate.")
    parser.add_argument("--dashboard-smoothing-alpha", type=float, default=None, help="Dashboard feedback smoothing alpha.")
    parser.add_argument("--lidar", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--lidar-debug-draw", action="store_true", help=argparse.SUPPRESS)
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


def show_v2_preflight_failure_dashboard(report: EcalPreflightReport) -> None:
    """在未创建 PyBullet 世界前展示可操作的 v2 Dashboard 启动诊断。"""
    from PySide6 import QtWidgets

    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    dialog = QtWidgets.QMessageBox()
    dialog.setWindowTitle("Slope Sim Dashboard · eCAL v2 启动诊断")
    dialog.setIcon(QtWidgets.QMessageBox.Icon.Critical)
    dialog.setText("eCAL v2 预检失败，实时仿真未启动")
    dialog.setInformativeText(report.format_dashboard())
    dialog.setStandardButtons(QtWidgets.QMessageBox.StandardButton.Ok)
    dialog.exec()
    # 保留 QApplication 引用直到模态诊断窗口关闭。
    _ = application


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
    formal_v2_requested = (
        args.v2_realtime
        or args.interface_mode is not None
        or config.interface_mode == "ecal"
    )
    if args.rc_port is not None and not (
        args.manual and config.interface_enabled and formal_v2_requested
    ):
        print("runSim startup blocked: --rc-port requires formal v2 manual interface", file=sys.stderr)
        return 2
    if args.manual and config.interface_enabled and formal_v2_requested:
        if args.no_dashboard and (args.capture_duration_sec is not None or args.open_ros_rviz):
            print("runSim v2 startup blocked: capture and ROS/RViz controls require Dashboard", file=sys.stderr)
            return 2
        try:
            require_v2_interface_mode(config.interface_mode)
        except LegacyInterfaceModeError as error:
            print(f"runSim v2 startup blocked: {error}", file=sys.stderr)
            return 2
        # CLI 覆盖必须同时进入预检与随后创建的 eCAL participant/子进程，不能只复制给预检。
        runtime_environment_overrides: dict[str, str] = {}
        if args.ecal_config is not None:
            runtime_environment_overrides["ECAL_CONFIG_PATH"] = str(args.ecal_config.resolve())
        if args.ecal_time_plugin_path is not None:
            runtime_environment_overrides["ECAL_TIME_PLUGIN_PATH"] = str(args.ecal_time_plugin_path.resolve())
        environment = dict(os.environ)
        environment.update(runtime_environment_overrides)
        try:
            v2_runtime_root = resolve_v2_runtime_root(environment)
        except ValueError as error:
            print(f"runSim v2 startup blocked: {error}", file=sys.stderr)
            return 2
        if "ECAL_CONFIG_PATH" not in environment and "ECAL_DATA" not in environment:
            data_dir = v2_runtime_root / "etc" / "ecal"
            if (data_dir / "ecal.yaml").is_file():
                runtime_environment_overrides["ECAL_DATA"] = str(data_dir)
                environment["ECAL_DATA"] = str(data_dir)
        if "ECAL_TIME_PLUGIN_PATH" not in environment:
            plugin_dir = v2_runtime_root / "lib"
            if (plugin_dir / "libecaltime-localtime.so").is_file():
                runtime_environment_overrides["ECAL_TIME_PLUGIN_PATH"] = str(plugin_dir)
                environment["ECAL_TIME_PLUGIN_PATH"] = str(plugin_dir)
        report = run_v2_ecal_preflight(environment=environment)
        if not report.ok:
            print(report.format_terminal(), file=sys.stderr)
            if not args.no_dashboard:
                show_v2_preflight_failure_dashboard(report)
            return 2
    # 手动模式必须使用 PyBullet GUI；普通实验仍走 DIRECT/GUI 自动仿真路径。
    if args.manual and config.interface_enabled and formal_v2_requested:
        previous_environment = {
            name: os.environ.get(name) for name in runtime_environment_overrides
        }
        os.environ.update(runtime_environment_overrides)
        try:
            try:
                command = RunSimV2Command.launch(
                    release_root=v2_runtime_root,
                    robot_model=config.robot_model,
                )
            except RuntimeError as error:
                print(f"runSim v2 Command startup blocked: {error}", file=sys.stderr)
                return 2
            command_closed = False

            def close_command() -> None:
                """在 eCAL world 退订前停止唯一 Command，避免后续 publish 误报失败。"""
                nonlocal command_closed
                if not command_closed:
                    command.close()
                    command_closed = True

            try:
                command_arbiter = (
                    CommandSourceArbiter(command.client)
                    if callable(getattr(command.client, "send_target", None))
                    else None
                )
                rc_worker = None
                if args.rc_port is not None:
                    if command_arbiter is None:
                        print("runSim RC startup blocked: v2 Command client has no controlled target ingress", file=sys.stderr)
                        return 2
                    try:
                        rc_worker = start_rc_worker(
                            command_sink=lambda candidate, now: command_arbiter.submit_rc(candidate, now=now),
                            opener=pyserial_opener(),
                            explicit_path=args.rc_port.resolve(),
                        )
                    except (RuntimeError, ValueError) as error:
                        print(f"runSim RC startup blocked: {error}", file=sys.stderr)
                        return 2
                result = run_manual_demo(
                    config,
                    duration_limit_sec=args.duration_sec,
                    v2_session_id_factory=command.session_id_factory,
                    v2_command_client=command.client,
                    v2_command_shutdown=close_command,
                    v2_command_arbiter=command_arbiter,
                    v2_command_pid=getattr(command, "process_pid", None),
                    rc_worker=rc_worker,
                    v2_capture_release_root=v2_runtime_root,
                    v2_capture_output_root=(
                        None
                        if args.capture_output_dir is None
                        else args.capture_output_dir.resolve()
                    ),
                    v2_viewer_root=(
                        None if args.viewer_root is None else args.viewer_root.resolve()
                    ),
                    v2_capture_duration_sec=args.capture_duration_sec,
                    v2_open_live_viewer=args.open_ros_rviz,
                )
            finally:
                close_command()
        finally:
            for name, previous in previous_environment.items():
                if previous is None:
                    os.environ.pop(name, None)
                else:
                    os.environ[name] = previous
    elif args.manual:
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
