# 自动仿真模块：运行一次固定速度实验，记录轨迹、绘图并计算误差。
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from numbers import Real
from pathlib import Path
import sys
from threading import Lock
import time

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import (
    SimulationCoordinator,
    build_world_from_scene_document,
)
from slope_sim.diagnostics import compute_diagnostic_summary, write_diagnostic_summary
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.ecal_transport import create_transport
from slope_sim.interfaces.logging import InterfaceEventLogger, InterfaceLogPaths
from slope_sim.interfaces.runtime import InterfaceRuntime
from slope_sim.logger import CsvSimulationLogger
from slope_sim.metrics import compute_tracking_metrics
from slope_sim.model_registry import get_robot_model
from slope_sim.robot import DifferentialDriveRobot
from slope_sim.runtime_actions import TerrainSelection
from slope_sim.scene import SceneInfo, configure_gui_visualizer, probe_terrain, update_follow_camera
from slope_sim.scene_config import (
    SceneDocument,
    SensorDocument,
    dump_scene_atomic,
    load_scene,
)
from slope_sim.sensor_backend import PyBulletSensorBackend
from slope_sim.sensors import LidarSummary, read_lidar


def run_interface_physics_frame(
    runtime,
    coordinator,
    *,
    actual_transport_mode: str,
    linear_velocity: float,
    angular_velocity: float,
    dt: float,
):
    """按企业接口约定顺序推进一个未暂停物理帧。"""
    runtime.poll_transport()
    if actual_transport_mode == "local":
        # 本地控制也必须经过 codec、transport 和 mailbox，不能直达机器人。
        runtime.submit_local_twist(linear_velocity, angular_velocity, dt)
    runtime.before_physics_step(dt)
    result = coordinator.step(dt)
    runtime.after_physics_step(dt)
    return result


@dataclass
class _DeadlinePacer:
    """按绝对墙钟期限节流真实 eCAL 循环，超期时不追加漂移。"""

    period_sec: float
    monotonic: Callable[[], float] = time.monotonic
    sleep: Callable[[float], None] = time.sleep
    _next_deadline: float | None = None

    def __post_init__(self) -> None:
        if (
            isinstance(self.period_sec, bool)
            or not isinstance(self.period_sec, Real)
            or not math.isfinite(float(self.period_sec))
            or float(self.period_sec) <= 0.0
        ):
            raise ValueError("period_sec must be a positive finite number")
        if not callable(self.monotonic) or not callable(self.sleep):
            raise ValueError("monotonic and sleep must be callable")
        self.period_sec = float(self.period_sec)

    def start(self) -> None:
        """只读取一次起点，后续期限始终从上一 deadline 累加。"""
        self._next_deadline = float(self.monotonic()) + self.period_sec

    def wait_for_next_deadline(self) -> None:
        if self._next_deadline is None:
            raise RuntimeError("deadline pacer has not started")
        deadline = self._next_deadline
        self._next_deadline = deadline + self.period_sec
        delay = deadline - float(self.monotonic())
        if delay > 0.0:
            self.sleep(delay)


class InterfaceSession:
    """入口层持有运行时及日志路径，关闭时仍由 runtime 先释放全部接口资源。"""

    def __init__(
        self,
        runtime: InterfaceRuntime,
        logger: InterfaceEventLogger | None,
        actual_transport_mode: str,
    ) -> None:
        self.runtime = runtime
        self.logger = logger
        self.actual_transport_mode = actual_transport_mode
        self._closed = False
        self._log_paths: InterfaceLogPaths | None = None

    def close(self) -> InterfaceLogPaths | None:
        """先执行 runtime 固定关闭序列，再幂等读取 logger 的成对路径。"""
        if self._closed:
            return self._log_paths
        first_error: BaseException | None = None
        try:
            self.runtime.close()
        except BaseException as exc:
            first_error = exc
        if self.logger is not None:
            try:
                self._log_paths = self.logger.close()
            except BaseException as exc:
                if first_error is None:
                    first_error = exc
        self._closed = True
        if first_error is not None:
            raise first_error
        return self._log_paths


class _PeerStateRelay:
    """在 transport 先于 runtime 创建时，串行转交 discovery 生命周期边沿。"""

    def __init__(self) -> None:
        self._lock = Lock()
        self._runtime: InterfaceRuntime | None = None
        self._transport: object | None = None

    def __call__(self, state: str) -> None:
        with self._lock:
            if self._runtime is not None and self._transport is not None:
                snapshot = self._transport.snapshot()
                self._runtime.handle_peer_state(
                    state,
                    ecal_connected=snapshot.ecal_connected,
                )
                self._runtime.consume_transport_snapshot(snapshot)

    def attach(self, runtime: InterfaceRuntime, transport: object) -> object:
        """锁住并发 callback 后读取最新快照，旧边沿只会重复而不会倒序。"""
        with self._lock:
            snapshot = transport.snapshot()
            state = getattr(
                snapshot,
                "state",
                "waiting_peer" if snapshot.ecal_connected else "disconnected",
            )
            detail = getattr(snapshot, "detail", "")
            runtime.initialize_peer_lifecycle(
                snapshot.mode,
                state,
                detail=detail,
                ecal_connected=snapshot.ecal_connected,
            )
            runtime.consume_transport_snapshot(snapshot)
            self._runtime = runtime
            self._transport = transport
            return snapshot


def initial_scene_document(config: ExperimentConfig) -> SceneDocument:
    """在连接 PyBullet 前加载并验证场景，缺省时从配置构造逻辑文档。"""
    if config.scene_in is not None:
        return load_scene(config.scene_in)
    terrain = TerrainSelection(
        config.terrain_model,
        slope_deg=config.slope_deg,
        golf_seed=config.golf_seed,
        golf_relief=config.golf_relief,
    )
    sensors = SensorDocument.default()
    return SceneDocument.from_runtime(
        config.robot_model,
        terrain,
        (),
        sensors.mounts,
        lidar_config=sensors.lidar,
    )


def _close_resource_quietly(resource: object | None) -> None:
    """初始化回滚只做尽力关闭，不能用次生错误覆盖原始异常。"""
    close = None if resource is None else getattr(resource, "close", None)
    if callable(close):
        try:
            close()
        except BaseException:
            pass


def create_interface_session(
    config: ExperimentConfig,
    *,
    client_id: int,
    coordinator_world,
    obstacle_manager,
    document: SceneDocument,
) -> InterfaceSession | None:
    """按配置创建唯一 transport/runtime/logger/backend 所有权链。"""
    if not config.interface_enabled:
        return None

    interface_config = InterfaceConfig.default(transport_mode=config.interface_mode)
    backend = None
    transport = None
    interface_logger = None
    runtime = None
    peer_state_relay = _PeerStateRelay()
    try:
        runtime_document = SceneDocument.from_runtime(
            document.robot_model,
            document.terrain,
            obstacle_manager.snapshot(include_body_id=False),
            document.sensors.mounts,
            lidar_config=document.sensors.lidar,
        )
        backend = PyBulletSensorBackend(
            client_id,
            coordinator_world.active_robot.robot.robot_id,
        )
        backend.bind_scene(
            coordinator_world.scene.body_ids,
            obstacle_manager.snapshot(include_body_id=True),
        )
        # auto 的唯一降级点在 create_transport 内；其余初始化异常一律向上传播。
        transport = create_transport(
            config.interface_mode,
            config=interface_config,
            peer_state_callback=peer_state_relay,
        )
        if config.interface_log_enabled:
            interface_logger = InterfaceEventLogger(
                config.log_dir,
                queue_size=interface_config.log_queue_size,
            )
        runtime = InterfaceRuntime(
            coordinator_world.active_robot.robot,
            config=interface_config,
            transport=transport,
            sensor_backend=backend,
            scene_document=runtime_document,
            logger=interface_logger,
        )
        actual_transport_mode = peer_state_relay.attach(runtime, transport).mode
    except BaseException:
        if runtime is not None:
            # runtime 构造成功后已经取得完整所有权，后续失败不能绕过其关闭顺序。
            _close_resource_quietly(runtime)
        else:
            # 构造失败自身也会清理；幂等关闭同时覆盖 transport/logger 的更早失败点。
            _close_resource_quietly(interface_logger)
            _close_resource_quietly(transport)
            _close_resource_quietly(backend)
        raise
    return InterfaceSession(runtime, interface_logger, actual_transport_mode)


@dataclass(frozen=True)
class SimulationResult:
    """一次仿真的输出结果，包含日志、图像和误差指标。"""

    log_path: Path
    figure_path: Path
    metrics: dict[str, float]
    feedback_figure_paths: tuple[Path, ...] = ()
    diagnostic_summary: dict[str, float] | None = None
    diagnostic_summary_path: Path | None = None
    obstacle_event_log_path: Path | None = None
    interface_binary_log: Path | None = None
    interface_event_log: Path | None = None
    scene_export: Path | None = None


def run_experiment(
    config: ExperimentConfig,
    *,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> SimulationResult:
    """运行一次自动仿真实验，并返回日志、图像和误差指标。"""
    # 场景文件必须在任何 PyBullet body 出现前完成全量解析与领域校验。
    document = initial_scene_document(config)
    connection_mode = p.GUI if config.mode == "gui" else p.DIRECT
    client_id = p.connect(connection_mode)
    if client_id < 0:
        raise RuntimeError(f"Failed to connect to PyBullet in {config.mode} mode")

    logger: CsvSimulationLogger | None = None
    interface_session: InterfaceSession | None = None
    interface_log_paths: InterfaceLogPaths | None = None
    log_path: Path | None = None
    scene_export: Path | None = None
    try:
        world, obstacle_manager = build_world_from_scene_document(
            client_id,
            config,
            document,
        )
        if config.interface_enabled:
            interface_session = create_interface_session(
                config,
                client_id=client_id,
                coordinator_world=world,
                obstacle_manager=obstacle_manager,
                document=document,
            )
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            obstacle_manager,
            interface_runtime=None if interface_session is None else interface_session.runtime,
            sensor_document=document.sensors,
        )
        scene = world.scene
        if config.mode == "gui":
            configure_gui_visualizer(
                client_id,
                config.camera_distance,
                config.camera_yaw,
                config.camera_pitch,
                config.camera_target,
            )
        logger = CsvSimulationLogger(config.log_dir, prefix=f"slope_{world.terrain.slope_deg:g}")
        steps = max(1, int(config.duration_sec / config.time_step))
        pacer: _DeadlinePacer | None = None
        if (
            interface_session is not None
            and interface_session.actual_transport_mode == "ecal"
        ):
            pacer = _DeadlinePacer(
                config.time_step,
                monotonic=monotonic,
                sleep=sleep,
            )
            pacer.start()

        # 自动目标在 local 下进入接口回环；严格 eCAL 只接受外部 WheelCommand。
        for step in range(steps):
            t = step * config.time_step
            robot = coordinator.world.active_robot.robot
            if interface_session is None:
                robot.command_twist(
                    config.target_linear_velocity,
                    config.target_angular_velocity,
                    dt=config.time_step,
                )
                coordinator.step(config.time_step)
                reported_linear_velocity = config.target_linear_velocity
                reported_angular_velocity = config.target_angular_velocity
            else:
                run_interface_physics_frame(
                    interface_session.runtime,
                    coordinator,
                    actual_transport_mode=interface_session.actual_transport_mode,
                    linear_velocity=config.target_linear_velocity,
                    angular_velocity=config.target_angular_velocity,
                    dt=config.time_step,
                )
                local_control = interface_session.actual_transport_mode == "local"
                reported_linear_velocity = config.target_linear_velocity if local_control else 0.0
                reported_angular_velocity = config.target_angular_velocity if local_control else 0.0

            world = coordinator.world
            scene = world.scene
            robot = world.active_robot.robot
            if config.drive_model == "physics":
                lidar_summary = _read_lidar_for_robot(client_id, robot, config)
                state = robot.read_physics_state(
                    t=t,
                    command_linear_velocity=reported_linear_velocity,
                    command_angular_velocity=reported_angular_velocity,
                    ground_lateral_friction=config.ground_lateral_friction,
                    drive_lateral_friction=config.drive_lateral_friction,
                    ground_rolling_friction=config.ground_rolling_friction,
                    ground_spinning_friction=config.ground_spinning_friction,
                    support_lateral_friction=config.support_lateral_friction,
                    robot_model=world.active_robot.robot_model,
                    terrain_type=scene.terrain_type,
                    terrain_probe=_probe_terrain_for_robot(client_id, robot, scene),
                    lidar_summary=lidar_summary,
                )
            else:
                state = robot.step_kinematic(dt=config.time_step, slope_deg=scene.slope_deg, t=t)
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
            if pacer is not None:
                pacer.wait_for_next_deadline()

        if config.scene_out is not None:
            scene_export = dump_scene_atomic(
                coordinator.logical_scene_document(),
                config.scene_out,
            )
    finally:
        # 清理异常不能覆盖正在传播的主异常；PyBullet 始终最后断开。
        primary_exception_active = sys.exc_info()[0] is not None
        cleanup_error: BaseException | None = None
        if interface_session is not None:
            try:
                interface_log_paths = interface_session.close()
            except BaseException as exc:
                cleanup_error = exc
        if logger is not None:
            try:
                log_path = logger.close()
            except BaseException as exc:
                if cleanup_error is None:
                    cleanup_error = exc
        try:
            p.disconnect(client_id)
        except BaseException as exc:
            if cleanup_error is None:
                cleanup_error = exc
        if not primary_exception_active and cleanup_error is not None:
            raise cleanup_error

    if log_path is None:
        raise RuntimeError("simulation CSV logger did not return a path")
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
        interface_binary_log=(
            None if interface_log_paths is None else interface_log_paths.binary_path
        ),
        interface_event_log=(
            None if interface_log_paths is None else interface_log_paths.event_path
        ),
        scene_export=scene_export,
    )


def _robot_urdf_path(robot_model: str) -> Path:
    """根据配置选择机器人 URDF。"""
    return get_robot_model(robot_model).urdf_path


def _robot_base_height(robot_model: str) -> float:
    """根据机器人模型选择贴近地面的初始车体高度。"""
    return get_robot_model(robot_model).base_height


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
    # 有 scene 时按地形 body 过滤，侧向偏移只作为旧调用和边界保护。
    probe_y = float(position[1]) + 0.45
    if scene is not None and scene.bounds is not None and probe_y > scene.bounds.max_y:
        probe_y = float(position[1]) - 0.45
    return probe_terrain(
        client_id,
        float(position[0]),
        probe_y,
        bounds=None if scene is None else scene.bounds,
        terrain_body_ids=None if scene is None else scene.body_ids,
    )


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
