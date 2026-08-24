# GUI 手动演示模块：打开 PyBullet 窗口，用方向键控制车辆在当前地形中运动。
from __future__ import annotations

from collections import deque
from collections.abc import Callable
import os
from pathlib import Path
import multiprocessing
import sys
import threading
import time

import pandas as pd
import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import (
    SimulationCoordinator,
    _logical_document_for_world,
    build_world_from_scene_document,
    load_manual_robot,
    reload_manual_robot,
)
from slope_sim.dashboard import DashboardCommand, TelemetryDashboard, should_refresh_dashboard
from slope_sim.diagnostics import compute_diagnostic_summary, write_diagnostic_summary
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.v2.runsim_session import (
    MAX_ANGULAR_VELOCITY_RAD_S,
    MAX_LINEAR_VELOCITY_M_S,
)
from slope_sim.interfaces.v2.world_runtime import V2ManualWorldRuntime
from slope_sim.logger import CsvSimulationLogger, ObstacleEventLogger
from slope_sim.resource_monitor import ResourceMonitor
from slope_sim.manual_capture import ManualCaptureRecorder, ManualCaptureSession
from slope_sim.manual_control import ManualCommand, ManualControlSettings, command_from_keyboard
from slope_sim.manual_mid360_reconstruction import reconstruction_worker_entrypoint
from slope_sim.metrics import compute_tracking_metrics
from slope_sim.realtime import DeadlinePacer, RuntimeObservationCadence
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
from slope_sim.scene import configure_gui_visualizer, update_follow_camera
from slope_sim.scene_config import dump_scene_atomic
from slope_sim.sensor_backend import PyBulletSensorBackend
from slope_sim.sensors import LidarSummary
from slope_sim.simulation import (
    InterfaceSession,
    SimulationResult,
    _probe_terrain_for_robot,
    _read_lidar_for_robot,
    create_interface_session,
    initial_scene_document,
    plot_feedback_figures,
    plot_trajectory,
)
from slope_sim.window_layout import (
    PYBULLET_WINDOW_TITLE,
    PYBULLET_WINDOW_TOKEN_ENV,
    WindowLayoutError,
    align_window_layout_to_scale,
    apply_main_window_rect,
    calculate_window_layout,
    connect_pybullet_gui,
    primary_display_metrics,
    search_x11_window_ids,
    x11_available_geometry,
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




def _create_v2_dashboard_widget(
    runtime: V2ManualWorldRuntime,
    *,
    live_viewer_launcher: Callable[[], Callable[[], None]] | None = None,
) -> object:
    """延迟导入 Qt v2 视图，确保非 v2 手动入口不依赖该可选 GUI 模块。"""
    from slope_sim.interfaces.v2.dashboard_adapter import V2DashboardWidget

    return V2DashboardWidget(
        runtime.descriptor,
        live_viewer_launcher=live_viewer_launcher,
    )


def _refresh_v2_dashboard_from_receiver(
    widget: object,
    receiver: object,
    *,
    chart_sink: object | None = None,
) -> bool:
    """GUI 线程只渲染 eCAL receiver 已校验的 snapshot 与 world 点云。"""
    refresh = getattr(widget, "refresh_from_store", None)
    cloud_frame = getattr(receiver, "cloud_frame", None)
    update_cloud = getattr(widget, "update_cloud_frame", None)
    store = getattr(receiver, "snapshot_store", None)
    if not callable(refresh) or not callable(cloud_frame) or not callable(update_cloud):
        raise ValueError("v2 Dashboard receiver/widget contract is incomplete")
    if store is None:
        raise ValueError("v2 Dashboard receiver has no snapshot_store")
    refreshed = bool(refresh(store))
    if chart_sink is not None:
        update_chart = getattr(chart_sink, "update_v2_chart_snapshot", None)
        snapshot = getattr(store, "snapshot", None)
        if callable(update_chart):
            if not callable(snapshot):
                raise ValueError("v2 Dashboard receiver store has no snapshot")
            current = snapshot()
            if current is not None:
                update_chart(current)
    frame = cloud_frame()
    if frame is not None:
        update_cloud(frame)
    update_diagnostics = getattr(widget, "update_receiver_diagnostics", None)
    render_dropped_count = getattr(receiver, "render_dropped_count", None)
    diagnostics = getattr(receiver, "diagnostics", None)
    if (
        callable(update_diagnostics)
        and type(render_dropped_count) is int
        and isinstance(diagnostics, tuple)
    ):
        update_diagnostics(
            render_dropped_count=render_dropped_count,
            diagnostics=diagnostics,
        )
    return refreshed


def _refresh_v2_dashboard_if_due(
    widget: object,
    receiver: object,
    *,
    chart_sink: object | None,
    last_refresh_at: float | None,
    now: float,
    update_hz: float,
) -> tuple[bool, float | None]:
    """以独立 UI 节拍拉取 v2 快照，不能跟随物理循环重复复制。"""
    if not should_refresh_dashboard(last_refresh_at, now, update_hz):
        return False, last_refresh_at
    _refresh_v2_dashboard_from_receiver(widget, receiver, chart_sink=chart_sink)
    return True, now


def _update_resource_dashboard(
    dashboard: object,
    monitor: object,
    *,
    children: dict[str, int],
    metrics: dict[str, str] | Callable[[], dict[str, str]],
    storage_paths: dict[str, Path],
) -> bool:
    """仅把到期的资源快照转交 Qt；采样与系统读取始终留在主循环边界。"""
    sample = getattr(monitor, "sample", None)
    update = getattr(dashboard, "update_resource_status", None)
    if not callable(sample) or not callable(update):
        raise ValueError("resource monitor/dashboard contract is incomplete")
    snapshot = sample(
        children=children,
        metrics=metrics,
        storage_paths=storage_paths,
    )
    if snapshot is None:
        return False
    update(snapshot)
    return True


def _resource_scalar_metrics(
    *,
    pacer: object,
    dashboard: object,
    rc_snapshot: object | None,
) -> dict[str, str]:
    """整理已有只读统计；未运行的可选组件不显示虚构的健康指标。"""
    statistics = getattr(pacer, "statistics", None)
    overrun_count = getattr(statistics, "overrun_count", None)
    metrics = {"物理超期": str(overrun_count) if isinstance(overrun_count, int) else "--"}
    actual_update_hz = getattr(dashboard, "actual_update_hz", None)
    if isinstance(actual_update_hz, (int, float)) and not isinstance(actual_update_hz, bool):
        metrics["Dashboard 刷新"] = f"{actual_update_hz:.1f} Hz"
    if rc_snapshot is None:
        return metrics
    actual_hz = getattr(rc_snapshot, "actual_hz", None)
    if isinstance(actual_hz, (int, float)) and not isinstance(actual_hz, bool):
        metrics["串口帧率"] = f"{actual_hz:.1f} Hz"
    frame_age_sec = getattr(rc_snapshot, "last_frame_age_sec", None)
    if isinstance(frame_age_sec, (int, float)) and not isinstance(frame_age_sec, bool):
        metrics["串口帧年龄"] = f"{frame_age_sec * 1_000.0:.0f} ms"
    watchdog_timeout_count = getattr(rc_snapshot, "watchdog_timeout_count", None)
    if isinstance(watchdog_timeout_count, int) and not isinstance(watchdog_timeout_count, bool):
        metrics["串口 watchdog"] = str(watchdog_timeout_count)
    return metrics


def _create_v2_dashboard_receiver(descriptor: object) -> tuple[object, object]:
    """创建附着既有 eCAL core 的 Dashboard 接收链，暂不启动 worker。"""
    from slope_sim.interfaces.v2.dashboard_receiver import (
        V2DashboardEcalReceiver,
        V2DashboardRawObserverTransport,
    )

    observer = V2DashboardRawObserverTransport(descriptor, start_worker=False)
    receiver = V2DashboardEcalReceiver(descriptor, transport=observer)
    observer.set_diagnostic_callback(receiver.record_transport_error)
    return observer, receiver


def _renew_v2_command_target(
    command_client: object | None,
    linear_velocity: float,
    angular_velocity: float,
    *,
    now: float,
    arbiter: object | None = None,
    source: str = "keyboard",
) -> None:
    """GUI 仅经已认证本机 socket 续租目标，不能直接发布 WheelCommand。"""
    if arbiter is not None:
        snapshot = getattr(arbiter, "snapshot", None)
        select_source = getattr(arbiter, "select_source", None)
        if not callable(snapshot) or not callable(select_source):
            raise ValueError("v2_command_arbiter must provide snapshot and select_source")
        if getattr(snapshot(), "active_source", None) != source:
            select_source(source, now=now)
        if source != "keyboard":
            return
        submit = getattr(arbiter, "submit_keyboard", None)
        if not callable(submit):
            raise ValueError("v2_command_arbiter is missing the selected source submitter")
        submit(linear_velocity, angular_velocity, now=now)
        return
    if source != "keyboard":
        return
    if command_client is None:
        return
    send_target = getattr(command_client, "send_target", None)
    if not callable(send_target):
        raise ValueError("v2_command_client must provide send_target")
    send_target(linear_velocity, angular_velocity, now=now)


def _sync_v2_command_generation(
    command_client: object | None,
    runtime: V2ManualWorldRuntime,
    *,
    robot_model: str,
    now: float,
) -> None:
    """world 提交后同步 C++ Command 的新身份和车型，禁止形状漂移。"""
    if command_client is None:
        return
    snapshot = runtime.protocol_snapshot()
    world_generation = getattr(snapshot, "world_generation", None)
    command_generation = getattr(snapshot, "command_generation", None)
    if type(world_generation) is not int or world_generation <= 0:
        raise ValueError("v2 runtime snapshot has invalid world_generation")
    if type(command_generation) is not int or command_generation <= 0:
        raise ValueError("v2 runtime snapshot has invalid command_generation")
    synchronize = getattr(command_client, "sync_generation", None)
    if not callable(synchronize):
        raise ValueError("v2_command_client must provide sync_generation")
    synchronize(
        world_generation,
        command_generation,
        robot_model=robot_model,
        now=now,
    )


def _start_v2_capture(*, release_root: Path, runtime: V2ManualWorldRuntime, output_root: Path) -> tuple[object, Path]:
    """以当前 v2 session/world 启动唯一 C++ Recorder，并请求完整边界开始。"""
    from slope_sim.interfaces.v2.runsim_v2_recorder import (
        RunSimV2Recorder,
        prepare_capture_output_dirs,
    )

    output_dirs = prepare_capture_output_dirs(output_root)
    recorder = RunSimV2Recorder.launch(
        release_root=release_root,
        snapshot=runtime.protocol_snapshot(),
        scene_id=output_dirs.published_dir.name,
        output_dir=output_dirs.staging_dir,
        published_output_dir=output_dirs.published_dir,
    )
    try:
        recorder.start()
    except BaseException:
        recorder.close()
        raise
    return recorder, output_dirs.published_dir


def merge_manual_commands(dashboard_command: DashboardCommand, keyboard_command: ManualCommand) -> DashboardCommand:
    """Dashboard 无输入时，允许 PyBullet 窗口焦点下的物理键盘接管控制。"""
    if keyboard_command.should_exit:
        return DashboardCommand(
            0.0,
            0.0,
            paused=dashboard_command.paused,
            should_exit=True,
            structural_action=dashboard_command.structural_action,
            camera_follow_enabled=dashboard_command.camera_follow_enabled,
            camera_follow_view=dashboard_command.camera_follow_view,
            control_source=dashboard_command.control_source,
        )
    dashboard_has_safe_stop_action = is_safe_stop_action(dashboard_command.structural_action)
    if dashboard_has_safe_stop_action:
        return DashboardCommand(
            0.0,
            0.0,
            paused=dashboard_command.paused,
            should_exit=dashboard_command.should_exit,
            structural_action=dashboard_command.structural_action,
            camera_follow_enabled=dashboard_command.camera_follow_enabled,
            camera_follow_view=dashboard_command.camera_follow_view,
            control_source=dashboard_command.control_source,
        )
    dashboard_has_motion = abs(dashboard_command.linear_velocity) > 1e-12 or abs(dashboard_command.angular_velocity) > 1e-12
    if dashboard_command.paused:
        return DashboardCommand(
            0.0,
            0.0,
            paused=True,
            should_exit=dashboard_command.should_exit,
            structural_action=dashboard_command.structural_action,
            camera_follow_enabled=dashboard_command.camera_follow_enabled,
            camera_follow_view=dashboard_command.camera_follow_view,
            control_source=dashboard_command.control_source,
        )
    if dashboard_command.control_source != "keyboard":
        return DashboardCommand(
            0.0,
            0.0,
            should_exit=dashboard_command.should_exit,
            structural_action=dashboard_command.structural_action,
            camera_follow_enabled=dashboard_command.camera_follow_enabled,
            camera_follow_view=dashboard_command.camera_follow_view,
            control_source=dashboard_command.control_source,
        )
    if dashboard_has_motion or dashboard_command.should_exit:
        return dashboard_command
    keyboard_has_motion = abs(keyboard_command.linear_velocity) > 1e-12 or abs(keyboard_command.angular_velocity) > 1e-12
    if not keyboard_has_motion:
        return dashboard_command
    return DashboardCommand(
        keyboard_command.linear_velocity,
        keyboard_command.angular_velocity,
        paused=dashboard_command.paused,
        should_exit=dashboard_command.should_exit,
        structural_action=dashboard_command.structural_action,
        camera_follow_enabled=dashboard_command.camera_follow_enabled,
        camera_follow_view=dashboard_command.camera_follow_view,
        control_source=dashboard_command.control_source,
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
        or target_command.paused
        or is_safe_stop_action(target_command.structural_action)
    ):
        return DashboardCommand(
            0.0,
            0.0,
            paused=target_command.paused,
            should_exit=target_command.should_exit,
            structural_action=target_command.structural_action,
            camera_follow_enabled=target_command.camera_follow_enabled,
            camera_follow_view=target_command.camera_follow_view,
            control_source=target_command.control_source,
        )
    return DashboardCommand(
        _step_toward(previous_command.linear_velocity, target_command.linear_velocity, linear_acceleration_limit * dt),
        _step_toward(previous_command.angular_velocity, target_command.angular_velocity, angular_acceleration_limit * dt),
        paused=target_command.paused,
        should_exit=target_command.should_exit,
        structural_action=target_command.structural_action,
        camera_follow_enabled=target_command.camera_follow_enabled,
        camera_follow_view=target_command.camera_follow_view,
        control_source=target_command.control_source,
    )


def _step_toward(current: float, target: float, max_delta: float) -> float:
    """把一个标量按最大步长推向目标值。"""
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + max_delta * (1.0 if delta > 0.0 else -1.0)


def _request_params_for_event(action: RuntimeAction | None) -> dict[str, object]:
    """把结构动作转换成 JSON 友好的请求参数，供障碍物事件日志复现操作来源。"""
    if isinstance(action, AddObstaclesAction):
        request = action.request
        return {
            "mode": request.mode,
            "count": request.count,
            "shape": request.shape,
            "moving_ratio": request.moving_ratio,
            "seed": request.seed,
            "speed": request.moving_speed,
        }
    if isinstance(action, DeleteObstacleAction):
        return {"logical_id": action.logical_id}
    if isinstance(action, ClearObstaclesAction):
        return {}
    if isinstance(action, SwitchTerrainAction):
        return _terrain_params(action.terrain)
    if isinstance(action, SwitchRobotAction):
        return {"robot_model": action.robot_model}
    if isinstance(action, ResetRobotAction):
        return {}
    return {}


def _terrain_params(terrain: TerrainSelection) -> dict[str, object]:
    """把场地选择转换成稳定 JSON 字段，供重建和回滚事件复用。"""
    return {
        "terrain_model": terrain.terrain_model,
        "slope_deg": terrain.slope_deg,
        "golf_seed": terrain.golf_seed,
        "golf_relief": terrain.golf_relief,
    }


def _event_type_for_action(action: RuntimeAction | None, result_operation: str | None = None) -> str:
    """统一事件类型名称，避免 Dashboard 文案变化影响日志解析。"""
    if result_operation in {"add", "delete", "clear"}:
        return result_operation
    if isinstance(action, AddObstaclesAction):
        return "add"
    if isinstance(action, DeleteObstacleAction):
        return "delete"
    if isinstance(action, ClearObstaclesAction):
        return "clear"
    if isinstance(action, SwitchTerrainAction):
        return "terrain_rebuild"
    if isinstance(action, SwitchRobotAction):
        return "robot_switch"
    if isinstance(action, ResetRobotAction):
        return "robot_reset"
    return result_operation or "structural_action"


def _seed_for_event(action: RuntimeAction | None) -> int | None:
    """提取可复现随机种子；非随机操作用 None。"""
    if isinstance(action, AddObstaclesAction):
        return action.request.seed
    return None


def _logical_id_for_event(action: RuntimeAction | None) -> int | None:
    """删除单个障碍物时记录逻辑 ID，批量事件保留为 null。"""
    if isinstance(action, DeleteObstacleAction):
        return action.logical_id
    return None


def _record_manual_structural_event(
    logger: ObstacleEventLogger,
    *,
    sim_time: float,
    action: RuntimeAction | None,
    result,
) -> None:
    """把协调器结果写入障碍物事件日志；pending 事务不写，避免重复刷屏。"""
    obstacle_result = getattr(result, "obstacle_result", None)
    if obstacle_result is not None and not obstacle_result.done:
        return
    if obstacle_result is None and not isinstance(action, SwitchTerrainAction):
        return
    operation = None if obstacle_result is None else obstacle_result.operation
    success = bool(getattr(result, "error_message", None) is None)
    if obstacle_result is not None:
        success = bool(obstacle_result.succeeded)
    error_reason = getattr(result, "error_message", None)
    if obstacle_result is not None and not obstacle_result.succeeded:
        error_reason = obstacle_result.message or error_reason
    event_type = _event_type_for_action(action, operation)
    logger.record_event(
        sim_time=sim_time,
        event_type=event_type,
        logical_id=_logical_id_for_event(action),
        request_params=_request_params_for_event(action),
        seed=_seed_for_event(action),
        robot_model=result.world.active_robot.robot_model,
        terrain=result.world.terrain,
        success=success,
        error_reason=error_reason,
    )
    if isinstance(action, SwitchTerrainAction) and getattr(result, "error_message", None) is not None:
        logger.record_event(
            sim_time=sim_time,
            event_type="rollback",
            logical_id=None,
            request_params=_terrain_params(result.world.terrain),
            seed=None,
            robot_model=result.world.active_robot.robot_model,
            terrain=result.world.terrain,
            success=True,
            error_reason=None,
        )


def run_manual_demo(
    config: ExperimentConfig,
    *,
    duration_limit_sec: float | None = None,
    monotonic: Callable[[], float] | None = None,
    sleep: Callable[[float], None] | None = None,
    v2_session_id_factory: Callable[[], bytes] | None = None,
    v2_command_client: object | None = None,
    v2_command_shutdown: Callable[[], None] | None = None,
    v2_command_arbiter: object | None = None,
    v2_command_pid: int | None = None,
    rc_worker: object | None = None,
    v2_capture_release_root: Path | None = None,
    v2_capture_output_root: Path | None = None,
    v2_viewer_root: Path | None = None,
    v2_capture_duration_sec: int | None = None,
    v2_open_live_viewer: bool = False,
) -> SimulationResult:
    """启动 PyBullet GUI，使用方向键手动控制当前地形中的车辆。"""
    if config.mode != "gui":
        raise ValueError("manual demo requires GUI mode; use --gui --manual")
    if v2_capture_output_root is not None and not v2_capture_output_root.is_absolute():
        raise ValueError("v2_capture_output_root must be an absolute Path")
    if v2_viewer_root is not None and not v2_viewer_root.is_absolute():
        raise ValueError("v2_viewer_root must be an absolute Path")
    if v2_capture_duration_sec not in (None, 60, 90, 180):
        raise ValueError("v2_capture_duration_sec must be 60, 90, or 180 seconds")
    if v2_command_pid is not None and (
        isinstance(v2_command_pid, bool) or not isinstance(v2_command_pid, int) or v2_command_pid <= 0
    ):
        raise ValueError("v2_command_pid must be a positive int or None")
    capture_root = (
        (config.log_dir.parent / "manual-mid360").resolve()
        if v2_capture_output_root is None
        else v2_capture_output_root
    )
    runtime_monotonic = time.monotonic if monotonic is None else monotonic

    # 配置文件先完成全量验证，失败时不能创建任何 PyBullet scene/robot body。
    document = initial_scene_document(config)

    # GUI 手动入口才创建 QApplication；局部引用需保留到两个窗口均关闭。
    from PySide6 import QtWidgets

    qt_application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    display_metrics = primary_display_metrics()
    available = x11_available_geometry(display_metrics)
    layout = align_window_layout_to_scale(
        calculate_window_layout(available, config.dashboard_enabled),
        display_metrics.device_pixel_ratio,
    )
    existing_pybullet_windows = search_x11_window_ids(
        PYBULLET_WINDOW_TITLE,
        only_visible=False,
    )
    client_id = connect_pybullet_gui(layout.main, pybullet_module=p)

    logger: CsvSimulationLogger | None = None
    obstacle_event_logger: ObstacleEventLogger | None = None
    dashboard: TelemetryDashboard | None = None
    v2_dashboard: object | None = None
    interface_session: InterfaceSession | None = None
    v2_manual_runtime: V2ManualWorldRuntime | None = None
    v2_dashboard_receiver: object | None = None
    v2_dashboard_observer: object | None = None
    interface_log_paths = None
    log_path: Path | None = None
    event_log_path: Path | None = None
    scene_export: Path | None = None
    capture_session: ManualCaptureSession | None = None
    capture_process: multiprocessing.Process | None = None
    capture_receiver = None
    capture_output_dir: Path | None = None
    viewer_import_results: deque[tuple[bool, str]] = deque()
    compression_results: deque[tuple[bool, Path | None, str]] = deque()
    v2_capture_recorder: object | None = None
    v2_capture_exporting = False
    v2_capture_results: deque[tuple[bool, Path | None, Path | None, str]] = deque()
    v2_capture_stop_deadline: float | None = None
    resource_monitor: ResourceMonitor | None = None
    try:
        claim_token = os.environ.get(PYBULLET_WINDOW_TOKEN_ENV)
        claim_kwargs = {} if claim_token is None else {"claim_token": claim_token}
        if existing_pybullet_windows:
            apply_main_window_rect(
                layout.main,
                excluded_window_ids=existing_pybullet_windows,
                **claim_kwargs,
            )
        else:
            apply_main_window_rect(layout.main, **claim_kwargs)
        world, obstacle_manager = build_world_from_scene_document(
            client_id,
            config,
            document,
        )
        if config.interface_enabled and config.interface_mode == "ecal":
            # 障碍物恢复会按当前地形重采样高度和姿态；v2 worker 必须接收已提交物理世界的规范文档。
            runtime_document = _logical_document_for_world(
                world,
                obstacle_manager,
                document.sensors,
            )
            # 正式 runSim v2 只复用这个 GUI client；worker 只做异步中心 LiDAR 复制世界。
            sensor_backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
            sensor_backend.bind_scene(
                world.scene.body_ids,
                obstacle_manager.snapshot(include_body_id=True),
            )
            v2_manual_runtime = V2ManualWorldRuntime(
                config=config,
                scene_document=runtime_document,
                robot=world.active_robot.robot,
                sensor_backend=sensor_backend,
                obstacle_manager=obstacle_manager,
                session_id_factory=v2_session_id_factory,
            )
        elif config.interface_enabled:
            interface_session = create_interface_session(
                config,
                client_id=client_id,
                coordinator_world=world,
                obstacle_manager=obstacle_manager,
                document=document,
                monotonic=runtime_monotonic,
            )
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            obstacle_manager,
            interface_runtime=(
                v2_manual_runtime
                if v2_manual_runtime is not None
                else None if interface_session is None else interface_session.runtime
            ),
            sensor_document=document.sensors,
        )
        terrain = world.terrain
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
        manual_prefix = (
            f"manual_{terrain.terrain_model}_{active_robot.robot_model}_{terrain.slope_deg:g}"
        )
        logger = CsvSimulationLogger(
            config.log_dir,
            prefix=manual_prefix,
            rate_hz=config.telemetry_log_hz,
        )
        obstacle_event_logger = ObstacleEventLogger(config.log_dir, prefix=f"{manual_prefix}_obstacles")

        linear_slider = None
        angular_slider = None
        if config.dashboard_enabled:
            interface_config = InterfaceConfig.default(
                transport_mode=config.interface_mode
            )
            try:
                dashboard = TelemetryDashboard(
                    max_linear_speed=config.target_linear_velocity,
                    max_angular_speed=config.target_angular_velocity,
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
                    interface_config=interface_config,
                    interface_enabled=config.interface_enabled,
                    developer_diagnostics_enabled=config.developer_diagnostics_enabled,
                    show_lidar_tools=v2_manual_runtime is None,
                    v2_dashboard_enabled=v2_manual_runtime is not None,
                )
            except Exception as exc:
                raise RuntimeError(f"dashboard construction failed: {exc}") from exc
            set_rc_available = getattr(dashboard, "set_rc_available", None)
            if callable(set_rc_available):
                set_rc_available(rc_worker is not None)
            if config.developer_diagnostics_enabled:
                resource_monitor = ResourceMonitor(monotonic=runtime_monotonic)
            if layout.dashboard is None:
                raise WindowLayoutError(
                    "dashboard layout is unavailable while Dashboard is enabled"
                )
            dashboard.apply_window_rect(
                layout.dashboard,
                display_metrics=display_metrics,
            )
            if v2_manual_runtime is not None:
                live_viewer_launcher = None
                if v2_capture_release_root is not None:
                    from slope_sim.interfaces.v2.runsim_v2_live_viewer import (
                        build_live_viewer_launcher,
                    )

                    live_viewer_launcher = build_live_viewer_launcher(
                        v2_capture_release_root
                    )
                # eCAL core 已由 simulator transport 初始化；Dashboard 只能附着只读 observer。
                (
                    v2_dashboard_observer,
                    v2_dashboard_receiver,
                ) = _create_v2_dashboard_receiver(
                    v2_manual_runtime.descriptor,
                )
                v2_dashboard_observer.start()
                v2_dashboard = _create_v2_dashboard_widget(
                    v2_manual_runtime,
                    live_viewer_launcher=live_viewer_launcher,
                )
                dashboard.attach_v2_dashboard_widget(v2_dashboard)
                if v2_open_live_viewer:
                    launch_live_viewer = getattr(v2_dashboard, "launch_live_viewer", None)
                    if not callable(launch_live_viewer):
                        raise RuntimeError("v2 dashboard does not provide live viewer startup")
                    launch_live_viewer()
                if v2_capture_duration_sec is not None:
                    capture_index = dashboard.capture_duration_combo.findData(
                        v2_capture_duration_sec
                    )
                    if capture_index < 0:
                        raise RuntimeError("v2 dashboard does not support requested capture duration")
                    dashboard.capture_duration_combo.setCurrentIndex(capture_index)
                    dashboard.request_capture_start()
                if v2_capture_release_root is not None:
                    from slope_sim.interfaces.v2.runsim_v2_recorder import (
                        load_latest_successful_lvx2_path,
                    )

                    latest_lvx2 = load_latest_successful_lvx2_path(
                        capture_root
                    )
                    if latest_lvx2 is not None:
                        latest_mcap = latest_lvx2.parent.parent / "session.mcap"
                        if latest_mcap.is_file():
                            dashboard.set_capture_completed(latest_mcap, latest_lvx2)

        if dashboard is None:
            # 仅 Dashboard 明确禁用时保留 PyBullet 自带调参滑条。
            linear_slider = p.addUserDebugParameter(
                "max linear speed [m/s]", 0.0, MAX_LINEAR_VELOCITY_M_S, config.target_linear_velocity
            )
            angular_slider = p.addUserDebugParameter(
                "max angular speed [rad/s]", 0.0, MAX_ANGULAR_VELOCITY_RAD_S, config.target_angular_velocity
            )

        max_steps = manual_step_limit(duration_limit_sec, config.time_step)
        manual_deadline = (
            None
            if duration_limit_sec is None
            else runtime_monotonic() + duration_limit_sec
        )
        step = 0
        out_of_bounds_latched = False
        # Dashboard 不可用时，持续沿用配置相机状态，不能被命令默认值覆盖。
        camera_follow_enabled = config.camera_follow_enabled
        camera_follow_view = config.camera_follow_view
        limited_command = DashboardCommand(
            0.0,
            0.0,
            paused=False,
            camera_follow_enabled=camera_follow_enabled,
            camera_follow_view=camera_follow_view,
        )
        handled_result = None
        manual_paused = False
        pending_event_actions: deque[RuntimeAction] = deque()
        pacer = DeadlinePacer(
            config.time_step,
            monotonic=runtime_monotonic,
            sleep=time.sleep if sleep is None else sleep,
        )
        observation_cadence = RuntimeObservationCadence(
            monotonic=runtime_monotonic,
        )
        last_v2_dashboard_refresh_at: float | None = None
        pacer.start()
        while (
            (max_steps is None or step < max_steps)
            and (manual_deadline is None or runtime_monotonic() < manual_deadline)
        ):
            if dashboard is not None:
                dashboard.process_events()
                if resource_monitor is not None:
                    resource_children: dict[str, int] = {}
                    if v2_command_pid is not None:
                        resource_children["Command"] = v2_command_pid
                    capture_pid = None if capture_process is None else capture_process.pid
                    if isinstance(capture_pid, int) and not isinstance(capture_pid, bool) and capture_pid > 0:
                        resource_children["离线重建"] = capture_pid
                    _update_resource_dashboard(
                        dashboard,
                        resource_monitor,
                        children=resource_children,
                        metrics=lambda: _resource_scalar_metrics(
                            pacer=pacer,
                            dashboard=dashboard,
                            rc_snapshot=(
                                rc_worker.snapshot()
                                if rc_worker is not None
                                and callable(getattr(rc_worker, "snapshot", None))
                                else None
                            ),
                        ),
                        storage_paths={
                            **({"CSV 日志": logger.path} if logger is not None else {}),
                            **({"采集目录": capture_output_dir} if capture_output_dir is not None else {}),
                        },
                    )
                if v2_command_arbiter is not None:
                    update_control_owner = getattr(dashboard, "update_control_owner", None)
                    snapshot = getattr(v2_command_arbiter, "snapshot", None)
                    if callable(update_control_owner) and callable(snapshot):
                        update_control_owner(snapshot())
                if v2_dashboard is not None and v2_manual_runtime is not None:
                    if v2_dashboard_receiver is not None:
                        _refreshed, last_v2_dashboard_refresh_at = _refresh_v2_dashboard_if_due(
                            v2_dashboard,
                            v2_dashboard_receiver,
                            chart_sink=dashboard,
                            last_refresh_at=last_v2_dashboard_refresh_at,
                            now=runtime_monotonic(),
                            update_hz=config.dashboard_update_hz,
                        )
                    else:
                        if should_refresh_dashboard(
                            last_v2_dashboard_refresh_at,
                            runtime_monotonic(),
                            config.dashboard_update_hz,
                        ):
                            v2_dashboard.refresh_from_store(
                                v2_manual_runtime.dashboard_snapshot_store
                            )
                            snapshot = v2_manual_runtime.dashboard_snapshot_store.snapshot()
                            if snapshot is not None:
                                dashboard.update_v2_chart_snapshot(snapshot)
                            last_v2_dashboard_refresh_at = runtime_monotonic()
                if rc_worker is not None and dashboard is not None:
                    snapshot = getattr(rc_worker, "snapshot", None)
                    update_rc_status = getattr(dashboard, "update_rc_status", None)
                    if callable(snapshot) and callable(update_rc_status):
                        update_rc_status(
                            snapshot(),
                            source_snapshot=(
                                v2_command_arbiter.snapshot()
                                if v2_command_arbiter is not None
                                else None
                            ),
                        )
                while viewer_import_results:
                    viewer_success, viewer_detail = viewer_import_results.popleft()
                    dashboard.set_capture_viewer_result(
                        success=viewer_success,
                        detail=viewer_detail,
                    )
                while compression_results:
                    compression_success, compressed_path, compression_detail = compression_results.popleft()
                    dashboard.set_capture_compression_result(
                        success=compression_success,
                        compressed_path=compressed_path,
                        detail=compression_detail,
                    )
                while v2_capture_results:
                    capture_success, mcap_path, lvx2_path, detail = v2_capture_results.popleft()
                    v2_capture_exporting = False
                    if capture_success and mcap_path is not None and lvx2_path is not None:
                        dashboard.set_capture_completed(mcap_path, lvx2_path)
                    else:
                        dashboard.set_capture_failed(detail, output_dir=capture_output_dir)
                if capture_process is not None and not capture_process.is_alive():
                    if capture_receiver is not None and capture_receiver.poll():
                        capture_result = capture_receiver.recv()
                        if capture_result.get("ok") is True:
                            dashboard.set_capture_completed(
                                Path(capture_result["mcap_path"]),
                                Path(capture_result["lvx2_path"]),
                            )
                        else:
                            dashboard.set_capture_failed(
                                str(capture_result.get("error", "离线重建失败")),
                                output_dir=capture_output_dir,
                            )
                    else:
                        dashboard.set_capture_failed(
                            "离线重建进程未返回结果",
                            output_dir=capture_output_dir,
                        )
                    capture_process.join()
                    capture_process = None
                    if capture_receiver is not None:
                        capture_receiver.close()
                        capture_receiver = None
                take_capture_request = getattr(dashboard, "take_capture_request", None)
                if (
                    v2_capture_recorder is not None
                    and v2_capture_stop_deadline is not None
                    and runtime_monotonic() >= v2_capture_stop_deadline
                ):
                    dashboard.request_capture_stop()
                    v2_capture_stop_deadline = None
                capture_request = (
                    None if not callable(take_capture_request) else take_capture_request()
                )
                if capture_request is not None:
                    if (
                        capture_request.kind == "start"
                        and v2_manual_runtime is not None
                        and v2_capture_release_root is not None
                        and v2_capture_recorder is None
                        and not v2_capture_exporting
                    ):
                        v2_capture_recorder, capture_output_dir = _start_v2_capture(
                            release_root=v2_capture_release_root,
                            runtime=v2_manual_runtime,
                            output_root=capture_root,
                        )
                        dashboard.set_capture_recording(
                            duration_limit_sec=capture_request.duration_limit_sec
                        )
                        v2_capture_stop_deadline = (
                            None
                            if capture_request.duration_limit_sec is None
                            else runtime_monotonic() + capture_request.duration_limit_sec
                        )
                    elif (
                        capture_request.kind == "stop"
                        and v2_capture_recorder is not None
                        and v2_capture_release_root is not None
                        and capture_output_dir is not None
                    ):
                        recorder = v2_capture_recorder
                        output_dir = capture_output_dir
                        v2_capture_recorder = None
                        v2_capture_exporting = True
                        v2_capture_stop_deadline = None
                        recorder.stop()
                        dashboard.set_capture_generating("正在导出 C++ Recorder MCAP 与 MID-360 点云")

                        def finalize_v2_capture() -> None:
                            try:
                                mcap_path = recorder.wait_for_success(timeout_sec=15.0)
                                lvx2_path, _export_result = recorder.export(
                                    release_root=v2_capture_release_root,
                                    mcap_path=mcap_path,
                                )
                                v2_capture_results.append((True, mcap_path, lvx2_path, ""))
                            except Exception as error:
                                v2_capture_results.append((False, None, None, str(error) or type(error).__name__))

                        threading.Thread(
                            target=finalize_v2_capture,
                            name="runsim-v2-capture-export",
                            daemon=True,
                        ).start()
                    elif capture_request.kind == "start" and capture_session is None and capture_process is None:
                        capture_session = ManualCaptureRecorder(
                            (config.log_dir.parent / "manual-mid360").resolve()
                        ).start(
                            scene_document=coordinator.logical_scene_document(),
                            world_generation=1,
                            duration_limit_sec=capture_request.duration_limit_sec,
                            started_sim_time_ns=round(step * config.time_step * 1_000_000_000),
                        )
                        dashboard.set_capture_recording(
                            duration_limit_sec=capture_request.duration_limit_sec
                        )
                    elif capture_request.kind == "stop" and capture_session is not None:
                        receipt = capture_session.finish(
                            finished_sim_time_ns=round(step * config.time_step * 1_000_000_000)
                        )
                        capture_output_dir = receipt.output_dir
                        dashboard.set_capture_generating("正在离线重建 MID-360 点云")
                        receiver, sender = multiprocessing.get_context("spawn").Pipe(duplex=False)
                        capture_process = multiprocessing.get_context("spawn").Process(
                            target=reconstruction_worker_entrypoint,
                            args=(receipt, config, sender),
                            daemon=True,
                        )
                        capture_process.start()
                        sender.close()
                        capture_receiver = receiver
                        capture_session = None
                    elif capture_request.kind == "open_viewer" and capture_request.lvx2_path is not None:
                        def import_lvx2(path: Path = capture_request.lvx2_path) -> None:
                            try:
                                from scripts.verify_livox_viewer2_linux import import_lvx2_in_livox_viewer

                                _pid, log_path = import_lvx2_in_livox_viewer(
                                    path,
                                    viewer_root=v2_viewer_root,
                                )
                                viewer_import_results.append(
                                    (True, f"已确认打开：{log_path}")
                                )
                            except Exception as error:
                                viewer_import_results.append(
                                    (False, str(error) or type(error).__name__)
                                )

                        threading.Thread(
                            target=import_lvx2,
                            name="livox-viewer-import",
                            daemon=True,
                        ).start()
                    elif capture_request.kind == "compress_mcap" and capture_request.mcap_path is not None:
                        def compress_mcap(path: Path = capture_request.mcap_path) -> None:
                            try:
                                from slope_sim.mcap_compression import compress_mcap_zstd

                                result = compress_mcap_zstd(path)
                                compression_results.append(
                                    (
                                        True,
                                        result.output_path,
                                        f"{result.source_bytes / 1024.0:.1f} KiB → "
                                        f"{result.output_bytes / 1024.0:.1f} KiB",
                                    )
                                )
                            except Exception as error:
                                compression_results.append(
                                    (False, None, str(error) or type(error).__name__)
                                )

                        threading.Thread(
                            target=compress_mcap,
                            name="mcap-zstd-compression",
                            daemon=True,
                        ).start()
                command = dashboard.current_command()
                # Dashboard 窗口和 PyBullet 窗口谁拿到焦点，都能用物理方向键控制。
                settings = ManualControlSettings(
                    max_linear_speed=dashboard.linear_spin.value(),
                    max_angular_speed=dashboard.angular_spin.value(),
                )
                # Dashboard 的下拉框取消键会被 PyBullet 捕获；退出只接受 Dashboard
                # 明确请求，避免切换控制源时把 Esc/Q 误当作关闭仿真。
                keyboard_command = command_from_keyboard(
                    p.getKeyboardEvents(),
                    settings,
                    False,
                )
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
                    paused=False,
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
                pending_event_actions.append(structural_action)
                if dashboard is not None:
                    dashboard.set_switch_busy(True, "应用中")

            target_command = DashboardCommand(
                0.0 if out_of_bounds_latched or safe_stop_requested or command.paused or capture_process is not None else command.linear_velocity,
                0.0 if out_of_bounds_latched or safe_stop_requested or command.paused or capture_process is not None else command.angular_velocity,
                paused=command.paused,
                should_exit=command.should_exit,
                structural_action=structural_action,
                camera_follow_enabled=camera_follow_enabled,
                camera_follow_view=camera_follow_view,
                control_source=command.control_source,
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
            t = step * config.time_step
            interface_snapshot_due = False
            interface_snapshot_wall_time = runtime_monotonic()
            if v2_manual_runtime is not None:
                if command.paused:
                    _renew_v2_command_target(
                        v2_command_client,
                        0.0,
                        0.0,
                        now=runtime_monotonic(),
                        arbiter=v2_command_arbiter,
                        source=command.control_source,
                    )
                    observation_cadence.poll_if_due(v2_manual_runtime)
                    manual_paused = True
                    pacer.wait_for_next_deadline()
                    continue
                if manual_paused:
                    observation_cadence.reset()
                    manual_paused = False
                    pacer.reset_deadline()
                (
                    _interface_snapshot_due,
                    interface_snapshot_wall_time,
                ) = observation_cadence.poll_if_due(v2_manual_runtime)
                _renew_v2_command_target(
                    v2_command_client,
                    linear_velocity,
                    angular_velocity,
                    now=interface_snapshot_wall_time,
                    arbiter=v2_command_arbiter,
                    source=limited_command.control_source,
                )
                v2_decision = v2_manual_runtime.command_decision(
                    now=interface_snapshot_wall_time,
                )
                robot.command_wheel_speeds(
                    v2_decision.drive_wheel_speed_rad_s,
                    v2_decision.steering_wheel_speed_rad_s,
                    dt=config.time_step,
                )
                # GUI/Dashboard 不能绕过唯一 C++ Command 写入 wheel command。
                reported_linear_velocity = 0.0
                reported_angular_velocity = 0.0
            elif interface_session is not None:
                runtime = interface_session.runtime
                if command.paused:
                    (
                        interface_snapshot_due,
                        interface_snapshot_wall_time,
                    ) = observation_cadence.poll_if_due(runtime)
                    if not manual_paused:
                        runtime.pause()
                        manual_paused = True
                    if dashboard is not None and interface_snapshot_due:
                        snapshot_wall_time = runtime_monotonic()
                        dashboard.update_interface_snapshot(
                            runtime.dashboard_snapshot(
                                wall_time=snapshot_wall_time,
                            )
                        )
                    pacer.wait_for_next_deadline()
                    continue
                if manual_paused:
                    observation_cadence.reset()
                (
                    interface_snapshot_due,
                    interface_snapshot_wall_time,
                ) = observation_cadence.poll_if_due(runtime)
                if manual_paused:
                    runtime.resume(wall_time=interface_snapshot_wall_time)
                    manual_paused = False
                    pacer.reset_deadline()
                if interface_session.actual_transport_mode == "local":
                    runtime.submit_local_twist(
                        linear_velocity,
                        angular_velocity,
                        config.time_step,
                    )
                    reported_linear_velocity = linear_velocity
                    reported_angular_velocity = angular_velocity
                else:
                    # 真实 eCAL 只消费外部 WheelCommand，Dashboard 速度不构成控制输入。
                    reported_linear_velocity = 0.0
                    reported_angular_velocity = 0.0
                runtime.before_physics_step(
                    config.time_step,
                    wall_time=interface_snapshot_wall_time,
                )
            else:
                if command.paused:
                    manual_paused = True
                    pacer.wait_for_next_deadline()
                    continue
                if manual_paused:
                    manual_paused = False
                    pacer.reset_deadline()
                robot.command_twist(linear_velocity, angular_velocity, dt=config.time_step)
                reported_linear_velocity = linear_velocity
                reported_angular_velocity = angular_velocity

            try:
                result = coordinator.step(config.time_step)
            except Exception as exc:
                if dashboard is not None:
                    dashboard.show_switch_status(f"切换失败: {exc}", is_error=True)
                raise
            if v2_manual_runtime is not None:
                v2_manual_runtime.after_physics_step(
                    config.time_step,
                    wall_time=interface_snapshot_wall_time,
                )
                if rc_worker is not None and dashboard is not None:
                    snapshot = getattr(rc_worker, "snapshot", None)
                    update_rc_status = getattr(dashboard, "update_rc_status", None)
                    if callable(snapshot) and callable(update_rc_status):
                        update_rc_status(
                            snapshot(),
                            source_snapshot=(
                                v2_command_arbiter.snapshot()
                                if v2_command_arbiter is not None
                                else None
                            ),
                        )
            elif interface_session is not None:
                interface_session.runtime.after_physics_step(config.time_step)

            # 结构事务可能替换整个 world，后续读取必须重新取得当前 robot 引用。
            world = coordinator.world
            scene = world.scene
            terrain = world.terrain
            active_robot = world.active_robot
            robot = active_robot.robot
            if config.drive_model == "physics":
                # v2 已由异步中心 worker 承担实时扫描；禁止回退 v1 同步射线。
                lidar_summary = (
                    LidarSummary()
                    if v2_manual_runtime is not None
                    else _read_lidar_for_robot(client_id, robot, config)
                )
                state = robot.read_physics_state(
                    t=t,
                    command_linear_velocity=reported_linear_velocity,
                    command_angular_velocity=reported_angular_velocity,
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
                state = robot.step_kinematic(
                    dt=config.time_step,
                    slope_deg=scene.slope_deg,
                    t=t,
                )
            if result is not None and result is not handled_result:
                handled_result = result
                if result.state_changed and v2_manual_runtime is not None:
                    _sync_v2_command_generation(
                        v2_command_client,
                        v2_manual_runtime,
                        robot_model=result.world.active_robot.robot_model,
                        now=runtime_monotonic(),
                    )
                event_action = pending_event_actions[0] if pending_event_actions else None
                _record_manual_structural_event(
                    obstacle_event_logger,
                    sim_time=t,
                    action=event_action,
                    result=result,
                )
                obstacle_result = getattr(result, "obstacle_result", None)
                if obstacle_result is None or obstacle_result.done:
                    if pending_event_actions:
                        pending_event_actions.popleft()
                if result.world_reset:
                    # 新 world 已切换 runtime generation，下一帧必须立即刷新 discovery。
                    observation_cadence.reset()
                    if v2_manual_runtime is not None:
                        v2_manual_runtime.bind_obstacle_manager(
                            coordinator.obstacle_manager
                        )
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
                        paused=False,
                        camera_follow_enabled=camera_follow_enabled,
                        camera_follow_view=camera_follow_view,
                    )
                if dashboard is not None:
                    dashboard.sync_active_selection(result.world.active_robot.robot_model, result.world.terrain)
                    # 障碍物增删清不替换车辆或 runtime generation，需保留监控历史。
                    if result.state_changed and result.obstacle_result is None:
                        dashboard.reset_feedback_history()
                    queue_still_busy = getattr(coordinator, "has_pending_action", False)
                    obstacle_still_busy = result.obstacle_result is not None and not result.obstacle_result.done
                    if queue_still_busy or obstacle_still_busy:
                        # 结构 FIFO 未清空时保持按钮锁定，不把 pending 当成失败。
                        dashboard.set_structure_busy(True, result.status_message)
                    else:
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
                if interface_session is not None and interface_snapshot_due:
                    snapshot_wall_time = runtime_monotonic()
                    dashboard.update_interface_snapshot(
                        interface_session.runtime.dashboard_snapshot(
                            wall_time=snapshot_wall_time,
                        )
                    )
                dashboard.update_obstacle_snapshots(
                    lambda manager=coordinator.obstacle_manager: manager.snapshot(
                        include_body_id=False
                    )
                )
                dashboard.update(state)
            if capture_session is not None:
                position, orientation = p.getBasePositionAndOrientation(
                    robot.robot_id,
                    physicsClientId=client_id,
                )
                simulation_time_ns = round((step + 1) * config.time_step * 1_000_000_000)
                capture_session.record_pose(
                    sim_time_ns=simulation_time_ns,
                    position=tuple(float(value) for value in position),
                    orientation=tuple(float(value) for value in orientation),
                )
                elapsed_ns = simulation_time_ns - capture_session.started_sim_time_ns
                if (
                    capture_session.duration_limit_sec is not None
                    and elapsed_ns >= capture_session.duration_limit_sec * 1_000_000_000
                    and dashboard is not None
                ):
                    dashboard.request_capture_stop()
            logger.record(
                state,
                reference_x=0.0,
                reference_y=0.0,
                estimated_x=state.x,
                estimated_y=state.y,
            )
            step += 1
            pacer.wait_for_next_deadline()

        if config.scene_out is not None:
            scene_export = dump_scene_atomic(
                coordinator.logical_scene_document(),
                config.scene_out,
            )
    finally:
        # 正常和异常共用同一清理路径；主异常存续时只记录次生清理错误。
        primary_exception_active = sys.exc_info()[0] is not None
        cleanup_error: BaseException | None = None
        if capture_session is not None:
            try:
                capture_session.abort(reason="manual simulation exited before capture finished")
            except BaseException as exc:
                cleanup_error = exc
        if v2_capture_recorder is not None:
            try:
                close_recorder = getattr(v2_capture_recorder, "close", None)
                if not callable(close_recorder):
                    raise RuntimeError("v2 capture recorder does not provide close")
                close_recorder()
            except BaseException as exc:
                if cleanup_error is None:
                    cleanup_error = exc
        if capture_process is not None and capture_process.is_alive():
            capture_process.terminate()
            capture_process.join(timeout=2.0)
        if capture_receiver is not None:
            capture_receiver.close()
        if interface_session is not None:
            try:
                interface_log_paths = interface_session.close()
            except BaseException as exc:
                cleanup_error = exc
        if v2_dashboard_receiver is not None:
            try:
                v2_dashboard_receiver.close()
            except BaseException as exc:
                cleanup_error = exc
        if v2_dashboard_observer is not None:
            try:
                v2_dashboard_observer.close()
            except BaseException as exc:
                if cleanup_error is None:
                    cleanup_error = exc
        if rc_worker is not None:
            try:
                close_rc_worker = getattr(rc_worker, "close", None)
                if callable(close_rc_worker):
                    close_rc_worker()
            except BaseException as exc:
                if cleanup_error is None:
                    cleanup_error = exc
        if v2_manual_runtime is not None:
            if v2_command_shutdown is not None:
                try:
                    v2_command_shutdown()
                except BaseException as exc:
                    cleanup_error = exc
            else:
                close_v2_command_client = getattr(v2_command_client, "close", None)
                if v2_command_client is not None and not callable(close_v2_command_client):
                    cleanup_error = ValueError("v2_command_client must provide close")
                elif callable(close_v2_command_client):
                    try:
                        close_v2_command_client()
                    except BaseException as exc:
                        cleanup_error = exc
            try:
                v2_manual_runtime.close()
            except BaseException as exc:
                if cleanup_error is None:
                    cleanup_error = exc
        if dashboard is not None:
            try:
                dashboard.close()
            except BaseException as exc:
                if cleanup_error is None:
                    cleanup_error = exc
        if logger is not None:
            try:
                log_path = logger.close()
            except BaseException as exc:
                if cleanup_error is None:
                    cleanup_error = exc
        if obstacle_event_logger is not None:
            try:
                event_log_path = obstacle_event_logger.close()
            except BaseException as exc:
                if cleanup_error is None:
                    cleanup_error = exc
        try:
            # PyBullet 必须最后断开，runtime.close 的安全停车和传感器读取仍依赖 client。
            p.disconnect(client_id)
        except BaseException as exc:
            if cleanup_error is None:
                cleanup_error = exc
        if not primary_exception_active and cleanup_error is not None:
            raise cleanup_error

    if log_path is None:
        raise RuntimeError("manual CSV logger did not return a path")
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
        obstacle_event_log_path=event_log_path,
        interface_binary_log=(
            None if interface_log_paths is None else interface_log_paths.binary_path
        ),
        interface_event_log=(
            None if interface_log_paths is None else interface_log_paths.event_path
        ),
        scene_export=scene_export,
    )
