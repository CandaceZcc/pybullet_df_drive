#!/usr/bin/env python3
# LiDAR worker 本地实时门禁：用生产 spawn service 在 DIRECT 中验证双雷达负载与回收。
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path
import sys
from typing import Sequence
import time

import pybullet as p

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import SimulationCoordinator, build_world_from_scene_document
from slope_sim.lidar_worker import (
    LidarScanService,
    LidarWorkerWorldSpec,
    start_lidar_worker,
    world_digest_for_document,
)
from slope_sim.realtime import DeadlinePacer
from slope_sim.runtime_actions import AddObstaclesAction, ObstacleGenerationRequest
from slope_sim.sensor_backend import Pose, PyBulletSensorBackend
from slope_sim.simulation import initial_scene_document


_TIME_STEP_SEC = 1.0 / 240.0
_CAPTURE_PERIOD_SEC = 0.050
_HEARTBEAT_LIMIT_NS = 20_000_000
_DRAIN_TIMEOUT_SEC = 2.0
_WORKER_STARTUP_TIMEOUT_SEC = 15.0
_SIM_WALL_RATIO_MIN = 0.95


@dataclass(frozen=True, slots=True)
class RealtimeVerifierResult:
    """本地实时门禁的最小可审计结果，不依赖 eCAL 或同步替身。"""

    window_count: int
    capture_count: int
    completed_count: int
    overrun_count: int
    failure_count: int
    max_heartbeat_ns: int
    sim_wall_ratio: float
    worker_exitcode: int | None


def _bodyless_obstacle_snapshots(records: Sequence[object]) -> tuple[object, ...]:
    """从 parent 世界快照去除临时 body id，保持 worker IPC 只接收逻辑状态。"""
    return tuple(replace(record, body_id=None) for record in records)


def _world_mount_pose(
    backend: PyBulletSensorBackend,
    mount: object,
) -> Pose:
    """在单个物理帧内冻结 mount 与 base 的同源世界位姿。"""
    parent_pose = backend.world_pose(mount.parent_link)
    return backend.transform_pose(
        parent_pose,
        Pose(mount.position, mount.orientation),
    )


def _bootstrap_twenty_obstacle_world(client_id: int, config: ExperimentConfig):
    """正式 worker 启动前先生成固定 seed 的完整 20 障碍逻辑文档。"""
    initial_document = initial_scene_document(config)
    world, obstacle_manager = build_world_from_scene_document(
        client_id,
        config,
        initial_document,
    )
    coordinator = SimulationCoordinator(
        client_id,
        config,
        world,
        obstacle_manager,
        sensor_document=initial_document.sensors,
    )
    result = coordinator.apply_action(
        AddObstaclesAction(ObstacleGenerationRequest("mixed", 20, seed=7301))
    )
    if result.obstacle_result is None or not result.obstacle_result.succeeded:
        raise RuntimeError("failed to bootstrap twenty lidar verifier obstacles")
    document = coordinator.logical_scene_document()
    records = obstacle_manager.snapshot(include_body_id=True)
    if len(document.obstacles) != 20 or len(records) != 20:
        raise RuntimeError("twenty-obstacle bootstrap did not produce a complete scene")
    return world, obstacle_manager, document, records


def _drain_and_close_service(service: LidarScanService) -> int | None:
    """排空真实 child 的最后两级请求，再由 service 自己完成 typed Stop 回收。"""
    service.begin_draining()
    deadline = time.monotonic() + _DRAIN_TIMEOUT_SEC
    while True:
        snapshot = service.snapshot()
        if snapshot.in_flight_identity is None and snapshot.pending_capture_identity is None:
            break
        if time.monotonic() >= deadline:
            raise TimeoutError("lidar worker verifier drain timed out")
        service.poll()
        time.sleep(0.001)
    handle = service._owned_worker_handle
    if handle is None:
        raise RuntimeError("realtime verifier service lost its owned worker handle")
    service.close_idle(timeout_sec=_DRAIN_TIMEOUT_SEC)
    return handle.process.exitcode


def _require_sim_wall_ratio(physics_step_count: int, wall_duration_sec: float) -> float:
    """沿用 P0 的 sim/wall 下限，物理步仅统计正式测量窗口。"""
    if type(physics_step_count) is not int or physics_step_count < 0:
        raise ValueError("physics_step_count must be a nonnegative int")
    if type(wall_duration_sec) not in {int, float} or wall_duration_sec <= 0.0:
        raise ValueError("wall_duration_sec must be positive")
    ratio = physics_step_count * _TIME_STEP_SEC / float(wall_duration_sec)
    if ratio < _SIM_WALL_RATIO_MIN:
        raise RuntimeError(
            "lidar verifier sim/wall ratio fell below P0 oracle: "
            f"{ratio:.6f} < {_SIM_WALL_RATIO_MIN:.2f}"
        )
    return ratio


def _require_window_sim_wall_ratio(
    starting_step_count: int,
    ending_step_count: int,
    started_at: float,
    ended_at: float,
) -> float:
    """逐个 P0 窗口核对 sim/wall，禁止健康窗口掩盖早期违例。"""
    if type(starting_step_count) is not int or starting_step_count < 0:
        raise ValueError("starting_step_count must be a nonnegative int")
    if type(ending_step_count) is not int or ending_step_count < starting_step_count:
        raise ValueError("ending_step_count must not precede starting_step_count")
    if type(started_at) not in {int, float} or type(ended_at) not in {int, float}:
        raise ValueError("window timestamps must be real numbers")
    return _require_sim_wall_ratio(
        ending_step_count - starting_step_count,
        float(ended_at) - float(started_at),
    )


def _require_clean_worker_exitcode(worker_exitcode: int | None) -> None:
    """正常 Stop/Stopped 后只接受 owned worker 的精确零退出。"""
    if type(worker_exitcode) is not int or worker_exitcode != 0:
        raise RuntimeError(
            f"lidar worker exited with unexpected exitcode: {worker_exitcode!r}"
        )


def run_lidar_worker_realtime_verifier(
    *,
    windows: int = 10,
    duration_sec: float = 5.0,
) -> RealtimeVerifierResult:
    """执行连续本地实时窗口；任一 20 ms heartbeat 或 worker 合同异常立即失败。"""
    if type(windows) is not int or windows <= 0:
        raise ValueError("windows must be a positive int")
    if type(duration_sec) not in {int, float} or duration_sec <= 0.0:
        raise ValueError("duration_sec must be positive")

    config = ExperimentConfig(
        mode="direct",
        robot_model="df_back",
        terrain_model="flat",
        time_step=_TIME_STEP_SEC,
        interface_enabled=False,
        interface_log_enabled=False,
        dashboard_enabled=False,
    )
    client_id = p.connect(p.DIRECT)
    if client_id < 0:
        raise RuntimeError("failed to connect PyBullet DIRECT")

    service: LidarScanService | None = None
    worker_exitcode: int | None = None
    try:
        world, obstacle_manager, document, records = _bootstrap_twenty_obstacle_world(
            client_id,
            config,
        )
        world_spec = LidarWorkerWorldSpec(
            1,
            config,
            document,
            world_digest_for_document(document),
        )
        handle = start_lidar_worker(world_spec, startup_timeout_sec=_WORKER_STARTUP_TIMEOUT_SEC)
        service = LidarScanService.from_worker_handle(
            handle,
            lifecycle_generation=0,
        )
        backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
        backend.bind_scene(world.scene.body_ids, records)
        bodyless_records = _bodyless_obstacle_snapshots(records)
        mounts = document.sensors.mounts
        pacer = DeadlinePacer(_TIME_STEP_SEC)
        pacer.start()
        capture_count = 0
        max_heartbeat_ns = 0
        next_capture_at = time.monotonic()
        topic_index = 0
        physics_step_count = 0
        window_sim_wall_ratios: list[float] = []

        for _window in range(windows):
            window_started_at = time.monotonic()
            window_start_step_count = physics_step_count
            window_deadline = window_started_at + float(duration_sec)
            while time.monotonic() < window_deadline:
                heartbeat_started_ns = time.monotonic_ns()
                poll_started_ns = heartbeat_started_ns
                service.poll()
                poll_duration_ns = time.monotonic_ns() - poll_started_ns
                now = time.monotonic()
                capture_duration_ns = 0
                if now >= next_capture_at:
                    capture_started_ns = time.monotonic_ns()
                    topic = ("lidar_front", "lidar_rear")[topic_index % 2]
                    mount = (
                        mounts.lidar_front
                        if topic == "lidar_front"
                        else mounts.lidar_rear
                    )
                    if not service.capture(
                        topic=topic,
                        timestamp_ns=time.monotonic_ns(),
                        world_mount_pose=_world_mount_pose(backend, mount),
                        optional_base_pose=None,
                        complete_obstacle_snapshots_without_body_ids=bodyless_records,
                    ):
                        raise RuntimeError("production lidar service rejected realtime capture")
                    capture_count += 1
                    topic_index += 1
                    next_capture_at = now + _CAPTURE_PERIOD_SEC
                    capture_duration_ns = time.monotonic_ns() - capture_started_ns
                physics_started_ns = time.monotonic_ns()
                p.stepSimulation(physicsClientId=client_id)
                physics_step_count += 1
                physics_duration_ns = time.monotonic_ns() - physics_started_ns
                heartbeat_ns = time.monotonic_ns() - heartbeat_started_ns
                max_heartbeat_ns = max(max_heartbeat_ns, heartbeat_ns)
                if heartbeat_ns > _HEARTBEAT_LIMIT_NS:
                    raise RuntimeError(
                        "lidar verifier heartbeat exceeded 20 ms: "
                        f"{heartbeat_ns} ns "
                        f"(poll={poll_duration_ns} capture={capture_duration_ns} "
                        f"physics={physics_duration_ns})"
                )
                pacer.wait_for_next_deadline()

            window_sim_wall_ratios.append(
                _require_window_sim_wall_ratio(
                    window_start_step_count,
                    physics_step_count,
                    window_started_at,
                    time.monotonic(),
                )
            )

        sim_wall_ratio = min(window_sim_wall_ratios)
        worker_exitcode = _drain_and_close_service(service)
        _require_clean_worker_exitcode(worker_exitcode)
        snapshot = service.snapshot()
        if (
            snapshot.completed_count != capture_count
            or snapshot.overrun_count != 0
            or snapshot.failed_count != 0
            or snapshot.stale_count != 0
            or snapshot.last_error_code is not None
        ):
            raise RuntimeError("lidar verifier observed worker drop, failure, or overrun")
        return RealtimeVerifierResult(
            windows,
            capture_count,
            snapshot.completed_count,
            snapshot.overrun_count,
            snapshot.failed_count,
            max_heartbeat_ns,
            sim_wall_ratio,
            worker_exitcode,
        )
    except BaseException:
        if service is not None:
            try:
                service.force_close()
            except BaseException:
                pass
        raise
    finally:
        p.disconnect(client_id)


def _parse_args() -> argparse.Namespace:
    """解析独立命令行参数，默认执行计划要求的 10 个 5 秒窗口。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--windows", type=int, default=10)
    parser.add_argument("--duration-sec", type=float, default=5.0)
    return parser.parse_args()


if __name__ == "__main__":
    arguments = _parse_args()
    result = run_lidar_worker_realtime_verifier(
        windows=arguments.windows,
        duration_sec=arguments.duration_sec,
    )
    print(json.dumps(asdict(result), sort_keys=True))
