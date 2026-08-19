#!/usr/bin/env python3
"""阶段四 B2：真实 PyBullet 五话题 v2 Simulator runtime 的最小 DIRECT 入口。"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from numbers import Real
import os
from pathlib import Path
import sys
import time
from typing import Callable, Sequence

import pybullet as p


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import build_world_from_scene_document
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity, load_v2_descriptor
from slope_sim.interfaces.v2.dashboard_snapshot import V2DashboardSnapshotStore
from slope_sim.interfaces.v2.runtime_protocol import V2RuntimeProtocol
from slope_sim.interfaces.v2.sensor_frames import (
    V2AsyncSensorFrameFactory,
    V2OutputFramePublisher,
    V2WheelStateFactory,
)
from slope_sim.interfaces.v2.simulation_runtime import V2SimulatorRuntime
from slope_sim.interfaces.v2.topics import V2_OUTPUT_TOPICS, V2_TOPICS
from slope_sim.interfaces.v2.transport import create_v2_ecal_transport
from slope_sim.interfaces.v2.world_runtime import start_stage4_lidar_service
from slope_sim.lidar_worker import LidarScanService, start_lidar_worker
from slope_sim.lidar_pointcloud import MID360_PATTERN_SHA256, MID360_PATTERN_VERSION
from slope_sim.model_registry import get_robot_model, robot_model_names
from slope_sim.realtime import DeadlinePacer, RuntimeObservationCadence
from slope_sim.sensor_backend import PyBulletSensorBackend
from slope_sim.scene import terrain_model_names
from slope_sim.simulation import initial_scene_document
from slope_sim.truth_sensors import Stage4SensorMounts, Stage4TruthSensorSuite


_TRANSPORT_LANE_SWITCH_INTERVAL_SEC = 0.001


def _lidar_pattern_result_identity() -> dict[str, str]:
    """返回独立于 v2 wire descriptor 的冻结扫描表身份。"""
    return {
        "lidar_pattern_version": MID360_PATTERN_VERSION,
        "lidar_pattern_sha256": MID360_PATTERN_SHA256,
    }


def _positive_duration(duration_sec: object) -> float:
    """校验最小 DIRECT 窗口长度，拒绝 bool、NaN、无穷和零。"""
    if isinstance(duration_sec, bool) or not isinstance(duration_sec, Real):
        raise ValueError("duration_sec must be positive and finite")
    value = float(duration_sec)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("duration_sec must be positive and finite")
    return value


def _install_transport_lane_scheduling() -> float:
    """实时窗口收紧 GIL 轮转，避免 PyBullet 主线程饿死 transport lane。"""
    previous = sys.getswitchinterval()
    sys.setswitchinterval(min(previous, _TRANSPORT_LANE_SWITCH_INTERVAL_SEC))
    return previous


def _restore_transport_lane_scheduling(previous: float) -> None:
    """恢复调用方线程切换策略，避免同进程后续运行继承全局改动。"""
    sys.setswitchinterval(previous)


def _wait_for_verified_peers(
    runtime: V2SimulatorRuntime,
    *,
    timeout_sec: float | None = None,
    deadline: float | None = None,
) -> None:
    """验收前等待唯一 command publisher 与已验证的 output consumer。"""
    if deadline is None:
        if timeout_sec is None:
            raise ValueError("timeout_sec or deadline is required")
        deadline = time.monotonic() + timeout_sec
    expected_contracts = {contract.topic: contract for contract in V2_TOPICS}
    while True:
        snapshot = runtime.refresh_transport()
        qualities = {item.topic: item for item in snapshot.topic_quality}
        if (
            snapshot.ecal_connected
            and set(qualities) == set(expected_contracts)
            and all(
                quality.protocol_state == "verified"
                and (
                    quality.peer_count == 1
                    if contract.direction == "subscribe"
                    else quality.peer_count >= 1
                )
                for topic, contract in expected_contracts.items()
                for quality in (qualities[topic],)
            )
        ):
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("v2 eCAL peers did not reach verified directional state")
        time.sleep(0.01)


def _coordination_paths(
    ready_file: Path | None,
    start_file: Path | None,
) -> tuple[Path, Path] | None:
    """校验每次运行私有的成对协调 marker，拒绝复用或相对路径。"""
    if (ready_file is None) != (start_file is None):
        raise ValueError("ready_file and start_file must be provided together")
    if ready_file is None:
        return None
    if not isinstance(ready_file, Path) or not isinstance(start_file, Path):
        raise ValueError("ready_file and start_file must be Paths")
    if not ready_file.is_absolute() or not start_file.is_absolute():
        raise ValueError("coordination marker paths must be absolute")
    ready = ready_file.resolve(strict=False)
    start = start_file.resolve(strict=False)
    if ready == start or not ready.parent.is_dir() or not start.parent.is_dir():
        raise ValueError("coordination marker parents must exist and paths must differ")
    if ready.exists() or start.exists():
        raise ValueError("coordination markers must not already exist")
    return ready, start


def _write_marker(path: Path) -> None:
    """排他发布 ready/start，防止多个参与者误共享同一窗口。"""
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    os.close(descriptor)


def _wait_for_start(path: Path, *, deadline: float) -> None:
    """等待 parent 的共享 start；超时预算由调用者统一拥有。"""
    while not path.exists():
        if time.monotonic() >= deadline:
            raise TimeoutError("shared start marker was not created before startup deadline")
        time.sleep(0.01)


class _RuntimeTransportObserver:
    """把 v2 runtime 的刷新表面对接到通用低频观测节拍。"""

    def __init__(self, runtime: V2SimulatorRuntime) -> None:
        self._runtime = runtime

    def poll_transport(self) -> None:
        """只在观测到期时推进 native discovery 和 dashboard transport 快照。"""
        self._runtime.refresh_transport()


def _observe_transport_then_decide(
    runtime: V2SimulatorRuntime,
    *,
    observation_cadence: RuntimeObservationCadence,
) -> tuple[object, float]:
    """低频完成 native 观测后，以本物理帧的新墙钟读取 command 决策。"""
    observation_cadence.poll_if_due(_RuntimeTransportObserver(runtime))
    wall_time = time.monotonic()
    return runtime.command_decision(now=wall_time), wall_time


def run_v2_simulation_runtime(
    *,
    result_json: Path,
    duration_sec: float,
    robot_model: str | None = None,
    terrain_model: str | None = None,
    scene: Path | None = None,
    transport_factory: Callable[[DescriptorIdentity], object] | None = None,
    require_verified_peers: bool = False,
    peer_timeout_sec: float = 5.0,
    ready_file: Path | None = None,
    start_file: Path | None = None,
    dashboard_snapshot_store: V2DashboardSnapshotStore | None = None,
    session_id_factory: Callable[[], bytes] | None = None,
) -> dict[str, object]:
    """构造真实 DIRECT 世界，在每个物理步安全执行 command 并发布五个 v2 topic。"""
    duration = _positive_duration(duration_sec)
    if type(require_verified_peers) is not bool:
        raise ValueError("require_verified_peers must be a bool")
    timeout = _positive_duration(peer_timeout_sec)
    coordination = _coordination_paths(ready_file, start_file)
    if coordination is not None and not require_verified_peers:
        raise ValueError("coordinated startup requires verified peers")
    if not isinstance(result_json, Path):
        raise ValueError("result_json must be a Path")
    if dashboard_snapshot_store is not None and type(dashboard_snapshot_store) is not V2DashboardSnapshotStore:
        raise ValueError("dashboard_snapshot_store must be None or an exact V2DashboardSnapshotStore")
    if session_id_factory is not None and not callable(session_id_factory):
        raise ValueError("session_id_factory must be callable or None")
    if scene is not None and not isinstance(scene, Path):
        raise ValueError("scene must be None or a Path")
    if scene is not None and (robot_model is not None or terrain_model is not None):
        raise ValueError("scene cannot be combined with robot_model or terrain_model")
    selected_model = get_robot_model(robot_model or "df_mid").name
    config = ExperimentConfig(
        mode="direct",
        duration_sec=duration,
        robot_model=selected_model,
        terrain_model=terrain_model or "flat",
        scene_in=scene,
        interface_enabled=False,
        dashboard_enabled=False,
    )
    document = initial_scene_document(config)
    config = replace(
        config,
        robot_model=document.robot_model,
        terrain_model=document.terrain.terrain_model,
        slope_deg=document.terrain.slope_deg,
        golf_seed=document.terrain.golf_seed,
        golf_relief=document.terrain.golf_relief,
    )
    client_id = p.connect(p.DIRECT)
    if client_id < 0:
        raise RuntimeError("failed to connect PyBullet DIRECT")
    controller: V2RuntimeProtocol | None = None
    lidar_service: LidarScanService | None = None
    robot = None
    disconnected = False
    previous_thread_switch_interval_sec: float | None = None
    try:
        world, obstacle_manager = build_world_from_scene_document(client_id, config, document)
        robot = world.active_robot.robot
        backend = PyBulletSensorBackend(client_id, robot.robot_id)
        backend.bind_scene(
            world.scene.body_ids,
            obstacle_manager.snapshot(include_body_id=True),
        )
        worker_handle, lidar_service = start_stage4_lidar_service(
            config,
            document,
            world_generation=1,
            worker_starter=start_lidar_worker,
            service_factory=LidarScanService.from_worker_handle,
        )

        def capture_context() -> tuple[object, tuple[object, ...]]:
            """在 parent 物理步后冻结中心 mount 和无 body-id 障碍物快照。"""
            return (
                backend.world_pose("lidar_link"),
                tuple(
                    replace(snapshot, body_id=None)
                    for snapshot in obstacle_manager.snapshot(include_body_id=True)
                ),
            )

        descriptor = load_v2_descriptor()
        transport = (
            create_v2_ecal_transport(descriptor=descriptor)
            if transport_factory is None
            else transport_factory(descriptor)
        )
        controller = V2RuntimeProtocol(
            robot.model_spec,
            transport=transport,
            descriptor=descriptor,
            session_id_factory=session_id_factory,
        )
        if dashboard_snapshot_store is None:
            dashboard_snapshot_store = V2DashboardSnapshotStore()
        runtime = V2SimulatorRuntime(
            controller=controller,
            wheel_feedback_reader=robot.read_interface_wheel_state,
            sensor_frames=V2AsyncSensorFrameFactory(
                controller,
                lidar_service,
                Stage4TruthSensorSuite(backend, Stage4SensorMounts.default()),
                capture_context,
            ),
            output_publisher=V2OutputFramePublisher(transport, descriptor),
            wheel_state_factory=V2WheelStateFactory(controller, robot.model_spec.name),
            dashboard_snapshot_store=dashboard_snapshot_store,
        )
        subscribe = getattr(transport, "subscribe", None)
        if not callable(subscribe):
            raise RuntimeError("v2 transport must provide subscribe")
        subscribe(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            lambda payload, received_at: runtime.accept_command_payload(
                payload,
                received_at=received_at,
            ),
        )
        if require_verified_peers:
            startup_deadline = time.monotonic() + timeout
            _wait_for_verified_peers(runtime, deadline=startup_deadline)
            if coordination is not None:
                ready, start = coordination
                _write_marker(ready)
                _wait_for_start(start, deadline=startup_deadline)
                _wait_for_verified_peers(runtime, deadline=startup_deadline)

        previous_thread_switch_interval_sec = _install_transport_lane_scheduling()
        published_frames = {topic: 0 for topic in V2_OUTPUT_TOPICS}
        physics_steps = math.ceil(duration / config.time_step)
        window_started_at = time.monotonic()
        pacer = DeadlinePacer(config.time_step)
        pacer.start()
        observation_cadence = RuntimeObservationCadence()
        for _ in range(physics_steps):
            decision, wall_time = _observe_transport_then_decide(
                runtime,
                observation_cadence=observation_cadence,
            )
            robot.command_wheel_speeds(
                decision.drive_wheel_speed_rad_s,
                decision.steering_wheel_speed_rad_s,
                dt=config.time_step,
            )
            obstacle_manager.update_moving(config.time_step)
            p.stepSimulation(physicsClientId=client_id)
            batch = runtime.after_physics_step(config.time_step, wall_time=wall_time)
            published_frames["/sim/wheel/state"] += len(batch.wheel_timestamps_ns)
            for topic in V2_OUTPUT_TOPICS[1:]:
                published_frames[topic] += len(batch.sensor_timestamps_ns)
            pacer.wait_for_next_deadline()
        wall_duration = time.monotonic() - window_started_at

        # 关闭前先物理安全停车，确保 native transport 最后看到的是无活动驱动的反馈。
        robot.hold_current_steering_and_stop_drive(config.time_step)
        runtime.drain_sensor_outputs(timeout_sec=timeout)
        dashboard_snapshot = dashboard_snapshot_store.snapshot()
        lidar_service.begin_draining()
        lidar_service.close_idle()
        lidar_service = None
        if require_verified_peers:
            wait_idle = getattr(transport, "wait_idle", None)
            if not callable(wait_idle):
                raise RuntimeError("verified v2 eCAL transport must provide wait_idle")
            # eCAL close 会回收未发送的 pending latest；验收路径必须先排空。
            wait_idle(timeout_sec=timeout)
        # 在 close 前冻结 transport 计数，供性能门与运行结果交叉审计。
        transport_snapshot = runtime.refresh_transport()
        controller.close()
        controller = None
        p.disconnect(client_id)
        disconnected = True
        result = {
            "transport": "ecal",
            **_lidar_pattern_result_identity(),
            "robot_model": document.robot_model,
            "terrain_model": document.terrain.terrain_model,
            "slope_deg": document.terrain.slope_deg,
            "golf_seed": document.terrain.golf_seed,
            "golf_relief": document.terrain.golf_relief,
            "physics_steps": physics_steps,
            "sim_duration_sec": physics_steps * config.time_step,
            "wall_duration_sec": wall_duration,
            "published_frames": published_frames,
            "transport_metrics": {
                "published_count": transport_snapshot.published_count,
                "error_count": transport_snapshot.error_count,
                "dropped_count": transport_snapshot.dropped_count,
            },
            "lidar_worker": {
                "prewarmed_topics": list(worker_handle.ready.prewarmed_topics),
                "clean_shutdown": True,
            },
            # 只留下 GUI 可验证的有界元数据，禁止结果文件重载 child 点云 bytes。
            "dashboard_snapshot": (
                None
                if dashboard_snapshot is None
                else {
                    "wheel_timestamp_ns": (
                        None
                        if dashboard_snapshot.wheel_state is None
                        else dashboard_snapshot.wheel_state.timestamp_ns
                    ),
                    "lidar_timestamp_ns": dashboard_snapshot.lidar_timestamp_ns,
                    "lidar_sequence": dashboard_snapshot.lidar_sequence,
                    "rtk_timestamp_ns": (
                        None if dashboard_snapshot.rtk is None else dashboard_snapshot.rtk.timestamp_ns
                    ),
                    "imu_timestamp_ns": (
                        None if dashboard_snapshot.imu is None else dashboard_snapshot.imu.timestamp_ns
                    ),
                }
            ),
            "clean_shutdown": True,
        }
        result_json.parent.mkdir(parents=True, exist_ok=True)
        result_json.write_text(json.dumps(result, indent=2), encoding="utf-8")
        return result
    finally:
        if previous_thread_switch_interval_sec is not None:
            _restore_transport_lane_scheduling(previous_thread_switch_interval_sec)
        if robot is not None and controller is not None:
            try:
                robot.hold_current_steering_and_stop_drive(config.time_step)
            finally:
                controller.close()
        if lidar_service is not None:
            lidar_service.force_close()
        if not disconnected:
            p.disconnect(client_id)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析最小五话题 DIRECT runtime 的命令行参数。"""
    parser = argparse.ArgumentParser(description="Run the Stage 4 v2 PyBullet runtime")
    parser.add_argument("--result-json", type=Path, required=True)
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--robot-model", choices=robot_model_names(), default=None)
    parser.add_argument("--terrain-model", choices=terrain_model_names(), default=None)
    parser.add_argument("--scene", type=Path, default=None)
    parser.add_argument("--ready-file", type=Path, default=None)
    parser.add_argument("--start-file", type=Path, default=None)
    args = parser.parse_args(argv)
    if args.scene is not None and (
        args.robot_model is not None or args.terrain_model is not None
    ):
        parser.error("--scene cannot be combined with --robot-model or --terrain-model")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    """执行入口并把环境或 runtime 故障转换为可审计的非零退出码。"""
    args = _parse_args(argv)
    try:
        run_v2_simulation_runtime(
            result_json=args.result_json,
            duration_sec=args.duration_sec,
            robot_model=args.robot_model,
            terrain_model=args.terrain_model,
            scene=args.scene,
            ready_file=args.ready_file,
            start_file=args.start_file,
        )
    except Exception as error:
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
