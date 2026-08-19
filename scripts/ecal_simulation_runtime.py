#!/usr/bin/env python3
# 真实 eCAL 仿真子进程：以 PyBullet DIRECT 和正式 InterfaceRuntime 驱动六话题。
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import sys
import time
from typing import Callable, Collection, Mapping, Sequence

import pybullet as p


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import SimulationCoordinator, build_world_from_scene_document
from slope_sim.interfaces.backlog import _has_sustained_backlog
from slope_sim.interfaces.ecal_transport import (
    EcalTransport,
    EcalTransportSnapshot,
)
from slope_sim.interfaces.logging import (
    InterfaceEventLogger,
    InterfaceLogRecord,
    InterfaceLogSnapshot,
    read_interface_log,
)
from slope_sim.interfaces.wheel import WheelDecision
from slope_sim.lidar_worker import LidarServiceSnapshot
from slope_sim.model_registry import get_robot_model, robot_model_names
from slope_sim.obstacles import ObstacleGenerationRequest
from slope_sim.realtime import RuntimeObservationCadence
from slope_sim.runtime_actions import AddObstaclesAction
from slope_sim.simulation import (
    _DeadlinePacer,
    _restore_cyclic_gc,
    _suspend_cyclic_gc,
    create_interface_session,
    initial_scene_document,
)


_STOP_SPEED_TOLERANCE_RAD_S = 0.05
_STEERING_HOLD_TOLERANCE_RAD = 0.02
_TRANSPORT_IDLE_TIMEOUT_SEC = 2.0
_LOGGER_IDLE_TIMEOUT_SEC = 2.0
_LOGGER_SAMPLE_PERIOD_SEC = 0.100
_ACTIVE_DRIVE_THRESHOLD_RAD_S = 0.1
_ACTIVE_FEEDBACK_THRESHOLD_RAD_S = 0.5
_COMMAND_CALLBACK_SWITCH_INTERVAL_SEC = 0.001


def _install_command_callback_scheduling() -> float:
    """实时窗口收紧 GIL 轮转，避免 LiDAR 后处理饿死命令回调。"""
    previous = sys.getswitchinterval()
    sys.setswitchinterval(min(previous, _COMMAND_CALLBACK_SWITCH_INTERVAL_SEC))
    return previous


def _restore_command_callback_scheduling(previous: float) -> None:
    """恢复调用方线程切换策略，保证同进程重复运行不泄漏全局状态。"""
    sys.setswitchinterval(previous)


def _horizontal_distance(
    start: Sequence[float],
    end: Sequence[float],
) -> float:
    """仅按世界坐标 xy 计算底盘水平距离，坡面高度变化不计入前进证据。"""
    return math.hypot(float(end[0]) - float(start[0]), float(end[1]) - float(start[1]))


def _is_active_drive_control(decision: WheelDecision) -> bool:
    """等待、超时或阈值内目标均不计入正常负载有效控制时长。"""
    return (
        not decision.waiting
        and not decision.timed_out
        and any(
            abs(value) > _ACTIVE_DRIVE_THRESHOLD_RAD_S
            for value in decision.drive_wheel_speed_rad_s
        )
    )


def _is_controlled_motion_step(
    decision: WheelDecision,
    wheel_state: object,
) -> bool:
    """要求有效命令与同向真实关节反馈同时存在，排除命令回显。"""
    if not _is_active_drive_control(decision):
        return False
    actual = getattr(wheel_state, "drive_wheel_speed_rad_s", None)
    targets = decision.drive_wheel_speed_rad_s
    if not isinstance(actual, (tuple, list)) or len(actual) != len(targets):
        return False
    commanded_pairs = tuple(
        (float(target), float(measured))
        for target, measured in zip(targets, actual, strict=True)
        if abs(target) > _ACTIVE_DRIVE_THRESHOLD_RAD_S
    )
    return bool(commanded_pairs) and all(
        abs(measured) >= _ACTIVE_FEEDBACK_THRESHOLD_RAD_S
        and target * measured > 0.0
        for target, measured in commanded_pairs
    )


def _has_obstacle_contact(
    contacts: Sequence[Sequence[object]],
    *,
    robot_id: int,
    obstacle_body_ids: Collection[int],
) -> bool:
    """只识别机器人与已提交障碍物的接触，忽略地形和其他物体。"""
    obstacle_ids = frozenset(int(body_id) for body_id in obstacle_body_ids)
    for contact in contacts:
        if len(contact) < 3:
            continue
        body_a = int(contact[1])
        body_b = int(contact[2])
        if body_a == robot_id and body_b in obstacle_ids:
            return True
        if body_b == robot_id and body_a in obstacle_ids:
            return True
    return False


@dataclass
class _NormalLoadMotionTracker:
    """在两个 normal-load marker 之间累计物理步、双时钟和真实底盘运动。"""

    start_step_count: int
    start_sim_time_ns: int
    start_wall_time: float
    start_position_m: tuple[float, float, float]
    physics_time_step_sec: float
    _previous_position_m: tuple[float, float, float] = field(init=False)
    _horizontal_path_length_m: float = field(default=0.0, init=False)
    _max_horizontal_speed_m_s: float = field(default=0.0, init=False)
    _control_step_count: int = field(default=0, init=False)
    _control_wall_duration_sec: float = field(default=0.0, init=False)
    _previous_decision_wall_time: float = field(init=False)
    _previous_active_control: bool = field(default=False, init=False)
    _controlled_motion_step_count: int = field(default=0, init=False)
    _controlled_displacement_x_m: float = field(default=0.0, init=False)
    _controlled_displacement_y_m: float = field(default=0.0, init=False)
    _controlled_path_length_m: float = field(default=0.0, init=False)
    _controlled_max_speed_m_s: float = field(default=0.0, init=False)
    _obstacle_contact_step_count: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        self._previous_position_m = tuple(float(value) for value in self.start_position_m)
        self.physics_time_step_sec = _positive_finite(
            "physics_time_step_sec",
            self.physics_time_step_sec,
        )
        self._previous_decision_wall_time = float(self.start_wall_time)

    def observe_step(
        self,
        *,
        position_m: Sequence[float],
        linear_velocity_m_s: Sequence[float],
        active_control: bool,
        controlled_motion: bool,
        obstacle_contact: bool = False,
        decision_wall_time: float,
    ) -> None:
        """每个已完成物理步只调用一次，让路径和控制共享采样基准。"""
        decision_time = float(decision_wall_time)
        if decision_time < self._previous_decision_wall_time:
            raise RuntimeError("normal-load control wall clock moved backwards")
        if self._previous_active_control:
            self._control_wall_duration_sec += (
                decision_time - self._previous_decision_wall_time
            )
        self._previous_decision_wall_time = decision_time
        self._previous_active_control = bool(active_control)

        current_position = tuple(float(value) for value in position_m)
        delta_x = current_position[0] - self._previous_position_m[0]
        delta_y = current_position[1] - self._previous_position_m[1]
        step_distance = math.hypot(delta_x, delta_y)
        self._horizontal_path_length_m += step_distance
        self._previous_position_m = current_position
        horizontal_speed = math.hypot(
            float(linear_velocity_m_s[0]),
            float(linear_velocity_m_s[1]),
        )
        self._max_horizontal_speed_m_s = max(
            self._max_horizontal_speed_m_s,
            horizontal_speed,
        )
        if active_control:
            self._control_step_count += 1
        if obstacle_contact:
            self._obstacle_contact_step_count += 1
        if active_control and controlled_motion and not obstacle_contact:
            self._controlled_motion_step_count += 1
            self._controlled_displacement_x_m += delta_x
            self._controlled_displacement_y_m += delta_y
            self._controlled_path_length_m += step_distance
            self._controlled_max_speed_m_s = max(
                self._controlled_max_speed_m_s,
                horizontal_speed,
            )

    def finish(
        self,
        *,
        end_step_count: int,
        end_sim_time_ns: int,
        end_wall_time: float,
        end_position_m: Sequence[float],
    ) -> dict[str, int | float]:
        """冻结正常负载证据；所有时长直接来自对应时钟。"""
        step_count = end_step_count - self.start_step_count
        sim_duration_sec = (end_sim_time_ns - self.start_sim_time_ns) / 1.0e9
        wall_duration_sec = end_wall_time - self.start_wall_time
        if step_count < 0 or sim_duration_sec < 0.0 or wall_duration_sec <= 0.0:
            raise RuntimeError("normal-load motion clocks moved backwards")
        if end_wall_time < self._previous_decision_wall_time:
            raise RuntimeError("normal-load control wall clock moved backwards")
        if self._previous_active_control:
            self._control_wall_duration_sec += (
                end_wall_time - self._previous_decision_wall_time
            )
        self._horizontal_path_length_m += _horizontal_distance(
            self._previous_position_m,
            end_position_m,
        )
        base_displacement_m = _horizontal_distance(
            self.start_position_m,
            end_position_m,
        )
        control_sim_duration_sec = self._control_step_count * self.physics_time_step_sec
        controlled_sim_duration_sec = (
            self._controlled_motion_step_count * self.physics_time_step_sec
        )
        return {
            "normal_load_physics_time_step_sec": self.physics_time_step_sec,
            "normal_load_step_count": step_count,
            "normal_load_sim_duration_sec": sim_duration_sec,
            "normal_load_wall_duration_sec": wall_duration_sec,
            "normal_load_sim_wall_ratio": sim_duration_sec / wall_duration_sec,
            "normal_load_control_step_count": self._control_step_count,
            "normal_load_control_sim_duration_sec": control_sim_duration_sec,
            "normal_load_control_wall_duration_sec": self._control_wall_duration_sec,
            "normal_load_controlled_motion_step_count": (
                self._controlled_motion_step_count
            ),
            "normal_load_controlled_motion_sim_duration_sec": (
                controlled_sim_duration_sec
            ),
            "normal_load_obstacle_contact_step_count": (
                self._obstacle_contact_step_count
            ),
            "normal_load_controlled_displacement_m": math.hypot(
                self._controlled_displacement_x_m,
                self._controlled_displacement_y_m,
            ),
            "normal_load_controlled_path_length_m": (
                self._controlled_path_length_m
            ),
            "normal_load_controlled_mean_speed_m_s": (
                self._controlled_path_length_m / controlled_sim_duration_sec
                if controlled_sim_duration_sec > 0.0
                else 0.0
            ),
            "normal_load_controlled_max_speed_m_s": (
                self._controlled_max_speed_m_s
            ),
            "normal_load_base_displacement_m": base_displacement_m,
            "normal_load_base_path_length_m": self._horizontal_path_length_m,
            "normal_load_base_mean_speed_m_s": (
                self._horizontal_path_length_m / sim_duration_sec
                if sim_duration_sec > 0.0
                else 0.0
            ),
            "normal_load_base_max_speed_m_s": self._max_horizontal_speed_m_s,
        }


@dataclass(frozen=True)
class _NormalLoadStartCapture:
    """把正式窗口起点 ACK 前的日志与传输快照绑定。"""

    log_snapshot: InterfaceLogSnapshot
    transport_snapshot: EcalTransportSnapshot


@dataclass(frozen=True)
class _NormalLoadEndCapture:
    """把正式窗口终点与同一屏障内的日志、传输快照绑定。"""

    step_count: int
    sim_time_ns: int
    wall_time: float
    position_m: tuple[float, ...]
    log_queue_sample: tuple[int, int]
    log_snapshot: InterfaceLogSnapshot
    transport_snapshot: EcalTransportSnapshot


@dataclass
class _WheelDrainFenceGate:
    """窗口结束后只放行到首条 wheel 数据帧，再等待下一协议阶段。"""

    end_sim_time_ns: int
    delivered: bool = field(default=False, init=False)
    released: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        if (
            isinstance(self.end_sim_time_ns, bool)
            or not isinstance(self.end_sim_time_ns, int)
            or self.end_sim_time_ns < 0
        ):
            raise ValueError("end_sim_time_ns must be a nonnegative integer")

    def physics_step_due(self, *, next_protocol_ready: bool) -> bool:
        """fence 未产生时继续推进；产生后只由下一 marker 解除暂停。"""
        if not isinstance(next_protocol_ready, bool):
            raise ValueError("next_protocol_ready must be a bool")
        if self.released or not self.delivered:
            return True
        if next_protocol_ready:
            self.released = True
            return True
        return False

    def observe_wheel_states(self, states: Sequence[object]) -> None:
        """识别首条跨过正式窗口尾的 wheel 数据帧。"""
        if self.released:
            return
        post_window_count = 0
        for state in states:
            timestamp_ns = getattr(state, "timestamp_ns", None)
            if (
                isinstance(timestamp_ns, bool)
                or not isinstance(timestamp_ns, int)
                or timestamp_ns < 0
            ):
                raise RuntimeError("wheel drain state has an invalid timestamp")
            post_window_count += int(timestamp_ns > self.end_sim_time_ns)
        if self.delivered and post_window_count:
            raise RuntimeError("wheel drain fence emitted more than one data frame")
        if post_window_count > 1:
            raise RuntimeError("wheel drain step emitted more than one data frame")
        self.delivered = self.delivered or post_window_count == 1


@dataclass(frozen=True)
class _ScheduledPhysicsFrame:
    """一次调度轮次的安全决策、观测边界和实际物理推进结果。"""

    decision: object
    decision_wall_time: float
    observation_due: bool
    next_observation_at: float
    advanced: bool
    published_wheel_states: tuple[object, ...]


def _begin_normal_load_motion_window(
    *,
    pacer: _DeadlinePacer,
    start_step_count: int,
    start_sim_time_ns: int,
    physics_time_step_sec: float,
    start_position_m: tuple[float, float, float],
    monotonic: Callable[[], float] = time.monotonic,
) -> _NormalLoadMotionTracker:
    """屏障完成后重建绝对 deadline，再捕获正式窗口墙钟起点。"""
    pacer.reset_deadline()
    return _NormalLoadMotionTracker(
        start_step_count=start_step_count,
        start_sim_time_ns=start_sim_time_ns,
        start_wall_time=float(monotonic()),
        start_position_m=start_position_m,
        physics_time_step_sec=physics_time_step_sec,
    )


def _positive_finite(name: str, value: float) -> float:
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return normalized


def _write_ack(path: Path, payload: object = None) -> None:
    """原子语义只要求存在性；内容保留诊断结果。"""
    if path.exists():
        return
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


def _write_final_protocol_ack(
    runtime: object,
    transport: EcalTransport,
    logger: InterfaceEventLogger,
    path: Path,
) -> None:
    """排空传感器、日志和发送 lane 后保持冻结并写最终 ACK。"""
    fence = runtime.begin_sensor_fence()
    _wait_for_logger_idle(logger, timeout_sec=_LOGGER_IDLE_TIMEOUT_SEC)
    transport.wait_idle(timeout_sec=_TRANSPORT_IDLE_TIMEOUT_SEC)
    snapshot = transport.snapshot()
    if any(
        quality.state != "active" or quality.peer_connected is not True
        for quality in snapshot.topic_quality
    ):
        raise RuntimeError("final protocol transport snapshot is not active")
    _write_ack(path, {"active": True})
    runtime.complete_sensor_fence(fence, resume_capture=False)


def _drive_is_stopped(values: tuple[float, ...]) -> bool:
    return bool(values) and max(abs(value) for value in values) <= _STOP_SPEED_TOLERANCE_RAD_S


def _read_marker(path: Path) -> dict[str, object] | None:
    if not path.exists():
        return None
    try:
        loaded = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        # 对端可能仍在写 JSON，也可能收到 ack 后并发删除；本轮按无 marker 处理。
        return None
    if not isinstance(loaded, dict):
        raise ValueError(f"scenario marker must contain an object: {path}")
    return loaded


def _wheel_drain_physics_step_due(
    gate: _WheelDrainFenceGate,
    *,
    marker: Path,
    pacer: _DeadlinePacer,
) -> bool:
    """只在下一阶段 marker 完整可解析时解除 fence，并重建节拍期限。"""
    was_released = gate.released
    step_due = gate.physics_step_due(
        next_protocol_ready=_read_marker(marker) is not None,
    )
    if gate.released and not was_released:
        pacer.start()
    return step_due


def _marker_duration_sec(marker: Mapping[str, object], description: str) -> float:
    """严格读取 peer 写入的正式窗口时长，拒绝布尔值和宽松转换。"""
    value = marker.get("duration_sec")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{description} duration_sec must be a finite number")
    return _positive_finite(f"{description} duration_sec", float(value))


def _normal_load_physics_step_due(
    tracker: _NormalLoadMotionTracker,
    duration_sec: float,
    *,
    now: float,
) -> bool:
    """正式墙钟 deadline 一到即停步，complete marker 延迟不能扩展仿真窗。"""
    duration = _positive_finite("normal-load duration_sec", duration_sec)
    observed_at = float(now)
    if not math.isfinite(observed_at):
        raise ValueError("normal-load wall time must be finite")
    return observed_at < tracker.start_wall_time + duration


def _final_protocol_physics_step_due(final_ack: Path) -> bool:
    """最终协议 ACK 完整落盘后停步，给 peer 留出静默关闭窗口。"""
    return _read_marker(final_ack) is None


def _wait_for_normal_load_completion_marker(
    marker: Path,
    stop_file: Path,
    *,
    timeout_sec: float,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> tuple[dict[str, object], float]:
    """物理停步后有界等待完整 complete marker，并返回 post-poll 墙钟。"""
    timeout = _positive_finite("completion marker timeout_sec", timeout_sec)
    deadline = float(monotonic()) + timeout
    while True:
        payload = _read_marker(marker)
        observed_at = float(monotonic())
        if payload is not None:
            return payload, observed_at
        if stop_file.exists():
            raise RuntimeError("simulation runtime stopped before measurement completion")
        if observed_at >= deadline:
            raise TimeoutError(f"measurement completion marker not received: {marker}")
        sleep(min(0.005, deadline - observed_at))


def _wait_for_start_signal(
    start_file: Path,
    stop_file: Path,
    *,
    timeout_sec: float,
) -> None:
    """等待双进程共同门闩，避免 discovery 预热期提前发布。"""
    deadline = time.monotonic() + timeout_sec
    while not start_file.exists():
        if stop_file.exists():
            raise RuntimeError("simulation runtime stopped before start signal")
        if time.monotonic() >= deadline:
            raise TimeoutError(f"start signal not received: {start_file}")
        time.sleep(0.005)


def _prepare_physics_step(
    runtime: object,
    time_step_sec: float,
    *,
    observation_cadence: RuntimeObservationCadence,
) -> tuple[object, float, bool]:
    """到期时先推进 discovery；每帧都用新墙钟执行命令安全决策。"""
    observation_due, decision_wall_time = observation_cadence.poll_if_due(runtime)
    decision = runtime.before_physics_step(
        time_step_sec,
        wall_time=decision_wall_time,
    )
    return decision, decision_wall_time, observation_due


def _prepare_scheduled_physics_step(
    runtime: object,
    time_step_sec: float,
    *,
    observation_cadence: RuntimeObservationCadence,
    normal_load_tracker: _NormalLoadMotionTracker | None = None,
    normal_load_duration_sec: float | None = None,
) -> tuple[object, float, bool, float, bool]:
    """执行安全决策，并从 discovery 完成时刻重建下一观测期限。"""
    if (normal_load_tracker is None) != (normal_load_duration_sec is None):
        raise ValueError("normal-load tracker and duration must be provided together")
    decision, decision_wall_time, observation_due = _prepare_physics_step(
        runtime,
        time_step_sec,
        observation_cadence=observation_cadence,
    )
    scheduled_next = observation_cadence.next_observation_at
    if scheduled_next is None:
        raise RuntimeError("runtime observation cadence did not establish a deadline")
    physics_step_due = (
        True
        if normal_load_tracker is None or normal_load_duration_sec is None
        else _normal_load_physics_step_due(
            normal_load_tracker,
            normal_load_duration_sec,
            now=decision_wall_time,
        )
    )
    return (
        decision,
        decision_wall_time,
        observation_due,
        scheduled_next,
        physics_step_due,
    )


def _run_scheduled_physics_frame(
    runtime: object,
    coordinator: object,
    time_step_sec: float,
    *,
    observation_cadence: RuntimeObservationCadence,
    allow_physics_step: bool,
    normal_load_tracker: _NormalLoadMotionTracker | None = None,
    normal_load_duration_sec: float | None = None,
) -> _ScheduledPhysicsFrame:
    """持续执行安全/连接观测，仅在两层门禁都允许时推进并发布物理帧。"""
    if not isinstance(allow_physics_step, bool):
        raise ValueError("allow_physics_step must be a bool")
    (
        decision,
        decision_wall_time,
        observation_due,
        scheduled_next,
        deadline_step_due,
    ) = _prepare_scheduled_physics_step(
        runtime,
        time_step_sec,
        observation_cadence=observation_cadence,
        normal_load_tracker=normal_load_tracker,
        normal_load_duration_sec=normal_load_duration_sec,
    )
    advanced = allow_physics_step and deadline_step_due
    published_wheel_states: tuple[object, ...] = ()
    if advanced:
        coordinator.step(time_step_sec)
        published_wheel_states = tuple(runtime.after_physics_step(time_step_sec))
    return _ScheduledPhysicsFrame(
        decision=decision,
        decision_wall_time=decision_wall_time,
        observation_due=observation_due,
        next_observation_at=scheduled_next,
        advanced=advanced,
        published_wheel_states=published_wheel_states,
    )


def _finish_gated_physics_iteration(
    frame: _ScheduledPhysicsFrame,
    *,
    allow_physics_step: bool,
    pacer: _DeadlinePacer,
) -> None:
    """observation 处理完成后，为关闭的外部门消费本轮墙钟期限。"""
    if not frame.advanced and not allow_physics_step:
        pacer.wait_for_next_deadline()


def _transport_allows_physics_step(
    runtime: object,
    transport: EcalTransport,
    time_step_sec: float,
) -> bool:
    """发送 lane 忙时只挡住会跨发布期限的下一物理步。"""
    due_topics = runtime.next_physics_step_publish_topics(time_step_sec)
    return all(transport.is_topic_idle(topic) for topic in due_topics)


def _capture_runtime_observation(
    runtime: object,
    transport: EcalTransport,
    *,
    wall_time: float,
) -> tuple[object, object, EcalTransportSnapshot]:
    """复用 Dashboard 内含 status，并保留门禁所需逐话题 discovery 快照。"""
    dashboard = runtime.dashboard_snapshot(wall_time=wall_time)
    return dashboard.status, dashboard, transport.snapshot()


def _transport_snapshot_payload(
    snapshot: EcalTransportSnapshot,
) -> dict[str, object]:
    """把累计量和逐话题质量复制为可诊断的稳定 JSON 对象。"""
    return {
        "published_count": snapshot.published_count,
        "received_count": snapshot.received_count,
        "dropped_count": snapshot.dropped_count,
        "error_count": snapshot.error_count,
        "topic_quality": {
            quality.topic: {
                "dropped_count": quality.dropped_count,
                "error_count": quality.error_count,
                "state": quality.state,
                "detail": quality.detail,
                "peer_connected": quality.peer_connected,
            }
            for quality in snapshot.topic_quality
        },
    }


def _lidar_service_snapshot_result(runtime: object) -> dict[str, object] | None:
    """把当前 worker 的固定诊断合同复制进 P0 JSON 结果。"""
    capture = getattr(runtime, "lidar_service_snapshot", None)
    if not callable(capture):
        raise TypeError("runtime must expose lidar_service_snapshot()")
    snapshot = capture()
    if snapshot is None:
        return None
    if type(snapshot) is not LidarServiceSnapshot:
        raise TypeError("runtime returned an invalid LiDAR service snapshot")
    return {
        "state": snapshot.state,
        "child_pid": snapshot.child_pid,
        "lifecycle_generation": snapshot.lifecycle_generation,
        "pause_epoch": snapshot.pause_epoch,
        "next_job_id": snapshot.next_job_id,
        "in_flight_identity": snapshot.in_flight_identity,
        "pending_capture_identity": snapshot.pending_capture_identity,
        "completed_count": snapshot.completed_count,
        "failed_count": snapshot.failed_count,
        "overrun_count": snapshot.overrun_count,
        "stale_count": snapshot.stale_count,
        "max_capture_to_response_ns": snapshot.max_capture_to_response_ns,
        "last_error_code": snapshot.last_error_code,
        "last_error_detail": snapshot.last_error_detail,
    }


def _transport_snapshot_delta(
    start: EcalTransportSnapshot,
    end: EcalTransportSnapshot,
) -> dict[str, object]:
    """计算同一 transport 两个累计快照之间的严格非负增量。"""
    start_quality = {quality.topic: quality for quality in start.topic_quality}
    end_quality = {quality.topic: quality for quality in end.topic_quality}
    if start_quality.keys() != end_quality.keys():
        raise RuntimeError("transport topic set changed during measurement")

    published_count = end.published_count - start.published_count
    received_count = end.received_count - start.received_count
    dropped_count = end.dropped_count - start.dropped_count
    error_count = end.error_count - start.error_count
    if any(
        value < 0
        for value in (published_count, received_count, dropped_count, error_count)
    ):
        raise RuntimeError("transport counters moved backwards during measurement")

    topic_payload: dict[str, object] = {}
    for topic, end_item in end_quality.items():
        start_item = start_quality[topic]
        topic_dropped = end_item.dropped_count - start_item.dropped_count
        topic_errors = end_item.error_count - start_item.error_count
        if topic_dropped < 0 or topic_errors < 0:
            raise RuntimeError(
                f"transport topic counters moved backwards during measurement: {topic}"
            )
        topic_payload[topic] = {
            "dropped_count": topic_dropped,
            "error_count": topic_errors,
            "state": end_item.state,
            "detail": end_item.detail,
            "peer_connected": end_item.peer_connected,
        }
    return {
        "published_count": published_count,
        "received_count": received_count,
        "dropped_count": dropped_count,
        "error_count": error_count,
        "topic_quality": topic_payload,
    }


def _output_peer_isolated(
    snapshot: EcalTransportSnapshot,
    target_topic: str,
    output_topics: Sequence[str],
) -> bool:
    """直接读取 discovery 位，避免 Dashboard 故障优先级遮蔽断连事实。"""
    expected = set(output_topics)
    if target_topic not in expected:
        return False
    peer_states = {
        quality.topic: quality.peer_connected
        for quality in snapshot.topic_quality
        if quality.topic in expected
    }
    return (
        set(peer_states) == expected
        and peer_states[target_topic] is False
        and all(
            peer_states[topic] is True
            for topic in expected
            if topic != target_topic
        )
    )


def _topic_peer_connected(
    snapshot: EcalTransportSnapshot,
    topic: str,
) -> bool:
    """严格读取一个话题的 discovery 位，缺失或重复均不算已连接。"""
    matches = tuple(
        quality.peer_connected
        for quality in snapshot.topic_quality
        if quality.topic == topic
    )
    return matches == (True,)


def _wait_for_logger_idle(
    logger: InterfaceEventLogger,
    *,
    timeout_sec: float,
) -> InterfaceLogSnapshot:
    """在 marker 屏障内有界等待已接受日志全部落盘。"""
    deadline = time.monotonic() + timeout_sec
    while True:
        snapshot = logger.snapshot()
        if snapshot.writer_failed:
            raise RuntimeError("interface logger writer failed during normal load")
        if snapshot.closed:
            raise RuntimeError("interface logger is closed at marker boundary")
        if snapshot.pending_count == 0:
            return snapshot
        if time.monotonic() >= deadline:
            raise TimeoutError("interface logger did not become idle")
        time.sleep(0.005)


def _logger_snapshot_delta(
    start: InterfaceLogSnapshot,
    end: InterfaceLogSnapshot,
) -> dict[str, object]:
    """统计 normal-load marker 之间日志接受、丢弃和最终积压增量。"""
    accepted_messages = end.accepted_messages - start.accepted_messages
    accepted_events = end.accepted_events - start.accepted_events
    dropped_messages = end.dropped_messages - start.dropped_messages
    dropped_events = end.dropped_events - start.dropped_events
    if any(
        value < 0
        for value in (
            accepted_messages,
            accepted_events,
            dropped_messages,
            dropped_events,
        )
    ):
        raise RuntimeError("interface logger counters moved backwards")
    return {
        "accepted_messages": accepted_messages,
        "accepted_events": accepted_events,
        "dropped_messages": dropped_messages,
        "dropped_events": dropped_events,
        "writer_failed": end.writer_failed,
        "final_pending": end.pending_count,
    }


def _measurement_log_events(
    records: Sequence[InterfaceLogRecord],
    *,
    measurement_start: float,
    measurement_end: float,
    start_sim_time_ns: int,
    end_sim_time_ns: int,
    command_topic: str,
    output_topics: Sequence[str],
) -> tuple[list[dict[str, object]], dict[str, list[dict[str, object]]], bool]:
    """命令按墙钟、输出按共享仿真边界提取，并检查全局消息序列连续。"""
    start_ns = round(measurement_start * 1_000_000_000)
    end_ns = round(measurement_end * 1_000_000_000)
    if (
        isinstance(start_sim_time_ns, bool)
        or not isinstance(start_sim_time_ns, int)
        or isinstance(end_sim_time_ns, bool)
        or not isinstance(end_sim_time_ns, int)
        or start_sim_time_ns < 0
        or end_sim_time_ns <= start_sim_time_ns
    ):
        raise ValueError("simulation measurement window must be increasing uint64 values")
    selected = tuple(
        record
        for record in records
        if (
            record.topic == command_topic
            and record.direction == "receive"
            and start_ns <= record.wall_time_ns < end_ns
        )
        or (
            record.topic in output_topics
            and record.direction == "publish"
            and start_sim_time_ns < record.sim_time_ns <= end_sim_time_ns
        )
    )
    contiguous = bool(selected) and all(
        current.sequence == previous.sequence + 1
        for previous, current in zip(selected, selected[1:])
    )

    def payload(record: InterfaceLogRecord) -> dict[str, object]:
        return {
            "wall_time": record.wall_time_ns / 1_000_000_000.0,
            "timestamp_ns": record.sim_time_ns,
            "type": record.type_name,
        }

    commands = [
        payload(record)
        for record in selected
        if record.topic == command_topic and record.direction == "receive"
    ]
    published = {
        topic: [
            payload(record)
            for record in selected
            if record.topic == topic and record.direction == "publish"
        ]
        for topic in output_topics
    }
    return commands, published, contiguous


def _capture_marked_transport_snapshot(
    transport: EcalTransport,
    marker: Path,
    ack: Path,
    *,
    before_ack: Callable[[], None] | None = None,
    ack_fields: Mapping[str, object] | None = None,
) -> EcalTransportSnapshot | None:
    """只在 discovery 稳定后捕获计数，并把正式窗口边界写入回执。"""
    if not marker.exists():
        return None
    # 当前仿真线程在此不再提交新帧，确保快照覆盖 marker 前的全部发送。
    transport.wait_idle(timeout_sec=_TRANSPORT_IDLE_TIMEOUT_SEC)
    snapshot = transport.snapshot()
    if any(
        quality.state != "active" or quality.peer_connected is not True
        for quality in snapshot.topic_quality
    ):
        return None
    if before_ack is not None:
        before_ack()
    ack_payload = _transport_snapshot_payload(snapshot)
    for key, value in dict(ack_fields or {}).items():
        if key in ack_payload:
            raise ValueError(f"ack field conflicts with transport snapshot: {key}")
        ack_payload[key] = value
    _write_ack(ack, ack_payload)
    return snapshot


def _capture_normal_load_start(
    runtime: object,
    transport: EcalTransport,
    logger: InterfaceEventLogger,
    marker: Path,
    ack: Path,
    *,
    before_ack: Callable[[], None] | None = None,
    ack_fields: Mapping[str, object] | None = None,
) -> _NormalLoadStartCapture | None:
    """先收敛传感器和日志，再捕获 start transport 快照并恢复 capture。"""
    if not marker.exists():
        return None
    fence = runtime.begin_sensor_fence()
    log_snapshot = _wait_for_logger_idle(
        logger,
        timeout_sec=_LOGGER_IDLE_TIMEOUT_SEC,
    )
    transport_snapshot = _capture_marked_transport_snapshot(
        transport,
        marker,
        ack,
        before_ack=before_ack,
        ack_fields=ack_fields,
    )
    if transport_snapshot is None:
        raise RuntimeError("normal-load start transport snapshot is not active")
    # 只有成功 ACK 后才解除可恢复 fence；异常路径保持停止。
    runtime.complete_sensor_fence(fence, resume_capture=True)
    return _NormalLoadStartCapture(log_snapshot, transport_snapshot)


def _capture_normal_load_end(
    transport: EcalTransport,
    logger: InterfaceEventLogger,
    marker: Path,
    ack: Path,
    *,
    end_step_count: int,
    end_sim_time_ns: int,
    end_wall_time: float,
    end_position_m: Sequence[float],
    runtime: object | None = None,
) -> _NormalLoadEndCapture:
    """在单一结束屏障内冻结仿真边界、日志和 active transport 快照。"""
    fence = None if runtime is None else runtime.begin_sensor_fence()
    window_log_snapshot = logger.snapshot()
    accepted = (
        window_log_snapshot.accepted_messages
        + window_log_snapshot.accepted_events
    )
    completed = accepted - window_log_snapshot.pending_count
    if completed < 0:
        raise RuntimeError("interface logger pending exceeds accepted records")
    log_snapshot = _wait_for_logger_idle(
        logger,
        timeout_sec=_LOGGER_IDLE_TIMEOUT_SEC,
    )
    transport_snapshot = _capture_marked_transport_snapshot(
        transport,
        marker,
        ack,
        ack_fields={"window_end_sim_time_ns": end_sim_time_ns},
    )
    if transport_snapshot is None:
        raise RuntimeError("normal-load end transport snapshot is not active")
    if runtime is not None:
        # transport helper 已成功写 ACK，随后才允许后测协议继续 capture。
        runtime.complete_sensor_fence(fence, resume_capture=True)
    return _NormalLoadEndCapture(
        step_count=end_step_count,
        sim_time_ns=end_sim_time_ns,
        wall_time=end_wall_time,
        position_m=tuple(float(value) for value in end_position_m),
        log_queue_sample=(window_log_snapshot.pending_count, completed),
        log_snapshot=log_snapshot,
        transport_snapshot=transport_snapshot,
    )


def _bootstrap_normal_load_scene(
    client_id: int,
    config: ExperimentConfig,
    document: object,
) -> tuple[object, object, object, int, frozenset[int]]:
    """在 session/worker 启动前完成 P0 20 障碍事务，并冻结完整逻辑文档。"""
    world, obstacle_manager = build_world_from_scene_document(client_id, config, document)
    bootstrap_coordinator = SimulationCoordinator(
        client_id,
        config,
        world,
        obstacle_manager,
        sensor_document=document.sensors,
    )
    add_result = bootstrap_coordinator.apply_action(
        AddObstaclesAction(ObstacleGenerationRequest("mixed", 20, seed=7301))
    )
    obstacle_count = len(obstacle_manager.snapshot())
    if (
        add_result.obstacle_result is None
        or not add_result.obstacle_result.succeeded
        or obstacle_count != 20
    ):
        raise RuntimeError("failed to install exactly twenty normal-load obstacles")
    obstacle_body_ids = frozenset(
        snapshot.body_id
        for snapshot in obstacle_manager.snapshot(include_body_id=True)
        if snapshot.body_id is not None
    )
    if len(obstacle_body_ids) != obstacle_count:
        raise RuntimeError("normal-load obstacle body IDs are incomplete")
    runtime_document = bootstrap_coordinator.logical_scene_document()
    if len(runtime_document.obstacles) != obstacle_count:
        raise RuntimeError("normal-load logical scene does not match installed obstacles")
    return (
        world,
        obstacle_manager,
        runtime_document,
        obstacle_count,
        obstacle_body_ids,
    )


def run_simulation_runtime(
    *,
    result_json: Path,
    scenario_dir: Path,
    ready_file: Path,
    start_file: Path,
    stop_file: Path,
    participant_name: str,
    max_runtime_sec: float,
    robot_model: str = "active_steering_4wd",
) -> None:
    """创建正式 DIRECT 世界，实时推进物理并记录安全场景证据。"""
    max_runtime = _positive_finite("max_runtime_sec", max_runtime_sec)
    selected_robot_model = get_robot_model(robot_model).name
    config = ExperimentConfig(
        mode="direct",
        duration_sec=max_runtime,
        robot_model=selected_robot_model,
        interface_mode="ecal",
        interface_enabled=True,
        interface_log_enabled=True,
        dashboard_enabled=False,
        log_dir=result_json.parent / "interface-logs",
    )
    client_id = p.connect(p.DIRECT)
    if client_id < 0:
        raise RuntimeError("failed to connect PyBullet DIRECT")

    session = None
    transport = None
    runtime = None
    logger = None
    normal_load_start_snapshot = None
    normal_load_end_snapshot = None
    normal_load_log_start_snapshot = None
    normal_load_log_end_snapshot = None
    normal_load_motion_tracker: _NormalLoadMotionTracker | None = None
    wheel_drain_fence: _WheelDrainFenceGate | None = None
    normal_load_motion_evidence: dict[str, int | float] | None = None
    normal_load_log_samples: list[tuple[float, int, int]] = []
    normal_load_started_at: float | None = None
    next_log_sample_at: float | None = None
    fault_injection_transport_snapshot = None
    measurement_window: tuple[float, float] | None = None
    log_paths = None
    result: dict[str, object] = {}
    cyclic_gc_was_enabled: bool | None = None
    previous_thread_switch_interval_sec: float | None = None
    try:
        document = initial_scene_document(config)
        (
            world,
            obstacle_manager,
            document,
            obstacle_count,
            obstacle_body_ids,
        ) = _bootstrap_normal_load_scene(
            client_id,
            config,
            document,
        )
        session = create_interface_session(
            config,
            client_id=client_id,
            coordinator_world=world,
            obstacle_manager=obstacle_manager,
            document=document,
            participant_name=participant_name,
        )
        if session is None:
            raise RuntimeError("strict simulation runtime did not create an interface session")
        transport = session.transport
        if not isinstance(transport, EcalTransport):
            raise RuntimeError("strict simulation runtime did not create EcalTransport")
        if session.actual_transport_mode != "ecal":
            raise RuntimeError("strict simulation session did not retain ecal mode")
        logger = session.logger
        if logger is None:
            raise RuntimeError("strict simulation session did not create interface logging")
        runtime = session.runtime
        interface_config = runtime.config
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            obstacle_manager,
            interface_runtime=runtime,
            sensor_document=document.sensors,
        )
        robot = coordinator.world.active_robot.robot

        scenario_dir.mkdir(parents=True, exist_ok=True)
        ready_file.parent.mkdir(parents=True, exist_ok=True)
        ready_file.write_text("ready\n", encoding="utf-8")
        _wait_for_start_signal(
            start_file,
            stop_file,
            timeout_sec=max_runtime,
        )

        channels = interface_config.channels
        output_topics = tuple(
            channel.topic for channel in channels if channel.direction == "publish"
        )
        state_history: dict[str, list[dict[str, object]]] = {
            channel.topic: [] for channel in channels
        }
        last_states: dict[str, str] = {}
        output_disconnect_isolated = {topic: False for topic in output_topics}
        feedback_is_not_command_echo = False
        invalid_command_rejected = False
        timeout_stopped_vehicle = False
        timeout_preserved_steering = False
        timeout_reference: tuple[float, ...] | None = None
        timeout_reference_at: float | None = None
        disconnected_stopped = False
        reconnect_wait_seen = False
        reconnect_required_new_command = False
        reconnect_generation_advanced = False
        mailbox_generation_before_disconnect: int | None = None
        mailbox_generation_after_disconnect: int | None = None
        accepted_peer_states: dict[str, str] = {}

        physics_steps = 0
        warmup_started_at = time.monotonic()
        warmup_started_sim_ns = runtime.clock.now_ns
        warmup_started_log = logger.snapshot()
        warmup_started_status = runtime.status_snapshot(wall_time=warmup_started_at)
        warmup_requested_sec: float | None = None
        warmup_wall_duration_sec = 0.0
        warmup_sim_duration_sec = 0.0
        warmup_physics_steps = 0
        warmup_log_accepted_messages = 0
        warmup_topic_counts: dict[str, int] = {}
        normal_load_requested_duration_sec: float | None = None
        normal_load_sim_started_ns: int | None = None
        normal_load_sim_ended_ns: int | None = None
        normal_load_ended_at: float | None = None
        normal_load_command_baseline_count: int | None = None
        normal_load_command_states: set[str] = set()
        normal_load_base_start: tuple[float, float] | None = None
        normal_load_base_last: tuple[float, float] | None = None
        normal_load_base_trajectory_m = 0.0
        normal_load_rtk_start: tuple[float, float] | None = None
        normal_load_rtk_last: tuple[float, float] | None = None
        normal_load_rtk_timestamp_ns: int | None = None
        normal_load_wheel_timestamp_ns: int | None = None
        normal_load_drive_peaks: list[float] = []
        normal_load_steering_peaks: list[float] = []
        normal_load_steering_same_sign = False
        observation_cadence = RuntimeObservationCadence()
        status = None
        dashboard = None
        transport_status = None

        started_at = time.monotonic()
        pacer = _DeadlinePacer(config.time_step)
        pacer.start()
        cyclic_gc_was_enabled = _suspend_cyclic_gc()
        previous_thread_switch_interval_sec = _install_command_callback_scheduling()
        while not stop_file.exists():
            loop_wall_time = time.monotonic()
            if loop_wall_time - started_at >= max_runtime:
                raise TimeoutError("simulation runtime exceeded its process budget")

            if (
                normal_load_motion_tracker is not None
                and normal_load_end_snapshot is None
                and normal_load_requested_duration_sec is not None
                and not _normal_load_physics_step_due(
                    normal_load_motion_tracker,
                    normal_load_requested_duration_sec,
                    now=loop_wall_time,
                )
            ):
                measurement_complete_payload, end_barrier_wall_time = (
                    _wait_for_normal_load_completion_marker(
                        scenario_dir / "measurement_complete.active",
                        stop_file,
                        timeout_sec=_TRANSPORT_IDLE_TIMEOUT_SEC,
                    )
                )
                completed_duration_sec = _marker_duration_sec(
                    measurement_complete_payload,
                    "measurement complete marker",
                )
                if not math.isclose(
                    completed_duration_sec,
                    normal_load_requested_duration_sec,
                    rel_tol=0.0,
                    abs_tol=1.0e-9,
                ):
                    raise ValueError("measurement marker durations do not match")
                measurement_start = measurement_complete_payload.get(
                    "measurement_start"
                )
                measurement_end = measurement_complete_payload.get("measurement_end")
                if (
                    isinstance(measurement_start, bool)
                    or not isinstance(measurement_start, (int, float))
                    or isinstance(measurement_end, bool)
                    or not isinstance(measurement_end, (int, float))
                ):
                    raise ValueError(
                        "measurement complete marker must contain numeric window"
                    )
                completed_wall_window = (
                    float(measurement_start),
                    float(measurement_end),
                )
                if (
                    not all(math.isfinite(value) for value in completed_wall_window)
                    or completed_wall_window[1] <= completed_wall_window[0]
                ):
                    raise ValueError("measurement window must be finite and increasing")
                end_position, _orientation = p.getBasePositionAndOrientation(
                    robot.robot_id,
                    physicsClientId=robot.client_id,
                )
                end_capture = _capture_normal_load_end(
                    transport,
                    logger,
                    scenario_dir / "measurement_complete.active",
                    scenario_dir / "measurement_complete.ack",
                    end_step_count=physics_steps,
                    end_sim_time_ns=runtime.clock.now_ns,
                    end_wall_time=end_barrier_wall_time,
                    end_position_m=end_position,
                    runtime=runtime,
                )
                normal_load_motion_evidence = normal_load_motion_tracker.finish(
                    end_step_count=end_capture.step_count,
                    end_sim_time_ns=end_capture.sim_time_ns,
                    end_wall_time=end_capture.wall_time,
                    end_position_m=end_capture.position_m,
                )
                measurement_window = completed_wall_window
                normal_load_end_snapshot = end_capture.transport_snapshot
                normal_load_log_end_snapshot = end_capture.log_snapshot
                normal_load_ended_at = end_capture.wall_time
                normal_load_sim_ended_ns = end_capture.sim_time_ns
                normal_load_base_last = end_capture.position_m[:2]
                if normal_load_started_at is None:
                    raise RuntimeError("normal-load logger window did not start")
                normal_load_log_samples.append(
                    (
                        end_capture.wall_time - normal_load_started_at,
                        *end_capture.log_queue_sample,
                    )
                )
                # 继续推进到首条跨界 wheel 数据帧，随后等待下一协议 marker。
                wheel_drain_fence = _WheelDrainFenceGate(
                    end_sim_time_ns=end_capture.sim_time_ns,
                )
                pacer.start()
                observation_cadence.reset()

            fence_step_due = True
            if wheel_drain_fence is not None:
                fence_step_due = _wheel_drain_physics_step_due(
                    wheel_drain_fence,
                    marker=scenario_dir / "invalid.active",
                    pacer=pacer,
                )
            physics_step_due = (
                fence_step_due
                and _final_protocol_physics_step_due(
                    scenario_dir / "new_command.ack"
                )
                and _transport_allows_physics_step(
                    runtime,
                    transport,
                    config.time_step,
                )
            )

            active_tracker = (
                normal_load_motion_tracker
                if normal_load_end_snapshot is None
                else None
            )
            active_duration = (
                normal_load_requested_duration_sec
                if normal_load_end_snapshot is None
                else None
            )
            frame = _run_scheduled_physics_frame(
                runtime,
                coordinator,
                config.time_step,
                observation_cadence=observation_cadence,
                allow_physics_step=physics_step_due,
                normal_load_tracker=active_tracker,
                normal_load_duration_sec=active_duration,
            )
            decision = frame.decision
            decision_wall_time = frame.decision_wall_time
            observation_due = frame.observation_due
            if frame.advanced:
                physics_steps += 1
                if wheel_drain_fence is not None:
                    wheel_drain_fence.observe_wheel_states(
                        frame.published_wheel_states
                    )

            # discovery、Dashboard 与 transport 快照共享 20 Hz 观测边界。
            if observation_due:
                snapshot_time = time.monotonic()
                status, dashboard, transport_status = _capture_runtime_observation(
                    runtime,
                    transport,
                    wall_time=snapshot_time,
                )
                for topic, topic_status in status.topics.items():
                    if last_states.get(topic) != topic_status.state:
                        state_history[topic].append(
                            {
                                "wall_time": snapshot_time,
                                "state": topic_status.state,
                            }
                        )
                        last_states[topic] = topic_status.state
            _finish_gated_physics_iteration(
                frame,
                allow_physics_step=physics_step_due,
                pacer=pacer,
            )
            if not frame.advanced:
                continue
            if status is None or dashboard is None or transport_status is None:
                raise RuntimeError("runtime observation was not initialized")

            command = dashboard.wheel_command
            wheel_state = dashboard.wheel_state

            direct_wheel_state = None
            direct_base_position: tuple[float, ...] | None = None
            if normal_load_motion_tracker is not None and normal_load_end_snapshot is None:
                direct_wheel_state = robot.read_interface_wheel_state(runtime.clock.now_ns)
                base_position, _orientation = p.getBasePositionAndOrientation(
                    robot.robot_id,
                    physicsClientId=robot.client_id,
                )
                linear_velocity, _angular_velocity = p.getBaseVelocity(
                    robot.robot_id,
                    physicsClientId=robot.client_id,
                )
                contacts = p.getContactPoints(
                    bodyA=robot.robot_id,
                    physicsClientId=robot.client_id,
                )
                direct_base_position = tuple(float(value) for value in base_position)
                active_control = (
                    decision is not None and _is_active_drive_control(decision)
                )
                controlled_motion = (
                    decision is not None
                    and _is_controlled_motion_step(decision, direct_wheel_state)
                )
                if (
                    controlled_motion
                    and decision is not None
                    and len(decision.drive_wheel_speed_rad_s)
                    == len(direct_wheel_state.drive_wheel_speed_rad_s)
                    and any(
                        abs(actual - requested) > 1.0e-3
                        for actual, requested in zip(
                            direct_wheel_state.drive_wheel_speed_rad_s,
                            decision.drive_wheel_speed_rad_s,
                            strict=True,
                        )
                    )
                ):
                    feedback_is_not_command_echo = True
                normal_load_motion_tracker.observe_step(
                    position_m=direct_base_position,
                    linear_velocity_m_s=linear_velocity,
                    active_control=active_control,
                    controlled_motion=controlled_motion,
                    obstacle_contact=_has_obstacle_contact(
                        contacts,
                        robot_id=robot.robot_id,
                        obstacle_body_ids=obstacle_body_ids,
                    ),
                    decision_wall_time=decision_wall_time,
                )

                current_base = direct_base_position[:2]
                if normal_load_base_last is not None:
                    normal_load_base_trajectory_m += math.dist(
                        normal_load_base_last,
                        current_base,
                    )
                normal_load_base_last = current_base

                if (
                    normal_load_command_baseline_count is not None
                    and status.command.valid_count > normal_load_command_baseline_count
                ):
                    normal_load_command_states.add(status.command.state)
                if (
                    dashboard.rtk is not None
                    and dashboard.rtk.timestamp_ns != normal_load_rtk_timestamp_ns
                ):
                    normal_load_rtk_timestamp_ns = dashboard.rtk.timestamp_ns
                    current_rtk = (dashboard.rtk.main_x, dashboard.rtk.main_y)
                    if normal_load_rtk_start is None:
                        normal_load_rtk_start = current_rtk
                    normal_load_rtk_last = current_rtk

                normal_load_wheel_timestamp_ns = direct_wheel_state.timestamp_ns
                if not normal_load_drive_peaks:
                    normal_load_drive_peaks = [
                        0.0 for _value in direct_wheel_state.drive_wheel_speed_rad_s
                    ]
                if not normal_load_steering_peaks:
                    normal_load_steering_peaks = [
                        0.0 for _value in direct_wheel_state.steering_wheel_angle_rad
                    ]
                for index, value in enumerate(
                    direct_wheel_state.drive_wheel_speed_rad_s
                ):
                    normal_load_drive_peaks[index] = max(
                        normal_load_drive_peaks[index],
                        abs(value),
                    )
                for index, value in enumerate(
                    direct_wheel_state.steering_wheel_angle_rad
                ):
                    normal_load_steering_peaks[index] = max(
                        normal_load_steering_peaks[index],
                        abs(value),
                    )
                if (
                    len(direct_wheel_state.steering_wheel_angle_rad) == 2
                    and all(
                        abs(value) >= 0.1
                        for value in direct_wheel_state.steering_wheel_angle_rad
                    )
                    and math.prod(direct_wheel_state.steering_wheel_angle_rad) > 0.0
                ):
                    normal_load_steering_same_sign = True

            measurement_start_payload = _read_marker(
                scenario_dir / "measurement_start.active"
            )
            if normal_load_start_snapshot is None and measurement_start_payload is not None:
                requested = measurement_start_payload.get("warmup_sec")
                if isinstance(requested, bool) or not isinstance(requested, (int, float)):
                    raise ValueError("measurement start marker must contain warmup_sec")
                requested_warmup_sec = float(requested)
                if not math.isfinite(requested_warmup_sec) or requested_warmup_sec <= 0.0:
                    raise ValueError("measurement start warmup_sec must be positive")
                requested_duration_sec = _marker_duration_sec(
                    measurement_start_payload,
                    "measurement start marker",
                )
                start_step_count = physics_steps
                start_sim_time_ns = runtime.clock.now_ns
                tracker_holder: list[_NormalLoadMotionTracker] = []

                def begin_motion_window() -> None:
                    observation_cadence.reset()
                    base_position, _orientation = p.getBasePositionAndOrientation(
                        robot.robot_id,
                        physicsClientId=robot.client_id,
                    )
                    tracker_holder.append(
                        _begin_normal_load_motion_window(
                            pacer=pacer,
                            start_step_count=start_step_count,
                            start_sim_time_ns=start_sim_time_ns,
                            physics_time_step_sec=config.time_step,
                            start_position_m=tuple(
                                float(value) for value in base_position
                            ),
                        )
                    )

                start_capture = _capture_normal_load_start(
                    runtime,
                    transport,
                    logger,
                    scenario_dir / "measurement_start.active",
                    scenario_dir / "measurement_start.ack",
                    before_ack=begin_motion_window,
                    ack_fields={"window_start_sim_time_ns": start_sim_time_ns},
                )
                if start_capture is not None:
                    if len(tracker_holder) != 1:
                        raise RuntimeError("normal-load motion window did not start once")
                    normal_load_motion_tracker = tracker_holder[0]
                    normal_load_requested_duration_sec = requested_duration_sec
                    warmup_requested_sec = requested_warmup_sec
                    normal_load_start_snapshot = start_capture.transport_snapshot
                    normal_load_log_start_snapshot = start_capture.log_snapshot
                    normal_load_started_at = normal_load_motion_tracker.start_wall_time
                    next_log_sample_at = normal_load_started_at
                    normal_load_sim_started_ns = normal_load_motion_tracker.start_sim_time_ns
                    normal_load_command_baseline_count = status.command.valid_count
                    warmup_wall_duration_sec = normal_load_started_at - warmup_started_at
                    warmup_sim_duration_sec = (
                        normal_load_sim_started_ns - warmup_started_sim_ns
                    ) / 1_000_000_000.0
                    warmup_physics_steps = physics_steps
                    warmup_log_accepted_messages = (
                        start_capture.log_snapshot.accepted_messages
                        - warmup_started_log.accepted_messages
                    )
                    warmup_topic_counts = {
                        topic: topic_status.message_count
                        - warmup_started_status.topics[topic].message_count
                        for topic, topic_status in status.topics.items()
                    }
                    normal_load_base_start = (
                        normal_load_motion_tracker.start_position_m[0],
                        normal_load_motion_tracker.start_position_m[1],
                    )
                    normal_load_base_last = normal_load_base_start
                    baseline_accepted = (
                        start_capture.log_snapshot.accepted_messages
                        + start_capture.log_snapshot.accepted_events
                    )
                    normal_load_log_samples.append(
                        (
                            0.0,
                            start_capture.log_snapshot.pending_count,
                            baseline_accepted
                            - start_capture.log_snapshot.pending_count,
                        )
                    )

            if (
                normal_load_start_snapshot is not None
                and normal_load_end_snapshot is None
                and normal_load_started_at is not None
                and next_log_sample_at is not None
            ):
                sampled_at = time.monotonic()
                if sampled_at >= next_log_sample_at:
                    log_sample = logger.snapshot()
                    accepted = (
                        log_sample.accepted_messages + log_sample.accepted_events
                    )
                    normal_load_log_samples.append(
                        (
                            sampled_at - normal_load_started_at,
                            log_sample.pending_count,
                            accepted - log_sample.pending_count,
                        )
                    )
                    next_log_sample_at += _LOGGER_SAMPLE_PERIOD_SEC
                    if next_log_sample_at <= sampled_at:
                        next_log_sample_at = sampled_at + _LOGGER_SAMPLE_PERIOD_SEC

            invalid = _read_marker(scenario_dir / "invalid.active")
            if invalid is not None:
                invalid_timestamp = invalid.get("timestamp_ns")
                latest_timestamp = None if command is None else command.timestamp_ns
                if (
                    status.command.invalid_count > 0
                    and latest_timestamp != invalid_timestamp
                ):
                    invalid_command_rejected = True
                    _write_ack(scenario_dir / "invalid.ack", {"rejected": True})

            if (scenario_dir / "timeout.active").exists() and wheel_state is not None:
                if runtime.last_decision.timed_out:
                    if timeout_reference is None:
                        timeout_reference = wheel_state.steering_wheel_angle_rad
                        timeout_reference_at = decision_wall_time
                    timeout_stopped_vehicle = _drive_is_stopped(
                        wheel_state.drive_wheel_speed_rad_s
                    )
                    if (
                        timeout_reference_at is not None
                        and decision_wall_time - timeout_reference_at >= 0.05
                        and len(timeout_reference)
                        == len(wheel_state.steering_wheel_angle_rad)
                    ):
                        timeout_preserved_steering = all(
                            abs(current - reference)
                            <= _STEERING_HOLD_TOLERANCE_RAD
                            for current, reference in zip(
                                wheel_state.steering_wheel_angle_rad,
                                timeout_reference,
                                strict=True,
                            )
                        )
                    if timeout_stopped_vehicle and timeout_preserved_steering:
                        _write_ack(
                            scenario_dir / "timeout.ack",
                            {"stopped": True, "steering_held": True},
                        )

            for index, topic in enumerate(output_topics):
                marker = _read_marker(scenario_dir / f"drop_{index}.active")
                if (
                    marker is not None
                    and marker.get("topic") == topic
                    and _output_peer_isolated(
                        transport_status,
                        topic,
                        output_topics,
                    )
                ):
                    output_disconnect_isolated[topic] = True
                    _write_ack(
                        scenario_dir / f"drop_{index}.ack",
                        {"topic": topic, "isolated": True},
                    )
                restored = _read_marker(
                    scenario_dir / f"drop_{index}.restored"
                )
                if (
                    restored is not None
                    and restored.get("topic") == topic
                    and status.topics[topic].state == "active"
                    and all(
                        status.topics[other].state == "active"
                        for other in output_topics
                    )
                ):
                    _write_ack(
                        scenario_dir / f"drop_{index}.restored.ack",
                        {"topic": topic, "active": True},
                    )

            if (scenario_dir / "command_disconnect_prepare.active").exists():
                if (
                    mailbox_generation_before_disconnect is None
                    and status.topics[interface_config.wheel_command.topic].state
                    == "active"
                    and any(
                        abs(value) > 0.1
                        for value in runtime.last_decision.drive_wheel_speed_rad_s
                    )
                ):
                    _mailbox, generation = runtime.capture_command_ingress()
                    mailbox_generation_before_disconnect = generation
                    _write_ack(
                        scenario_dir / "command_disconnect_prepare.ack",
                        {"mailbox_generation": generation},
                    )

            if (scenario_dir / "command_disconnected.active").exists():
                if (
                    status.command.state == "disconnected"
                    and wheel_state is not None
                    and _drive_is_stopped(wheel_state.drive_wheel_speed_rad_s)
                ):
                    _mailbox, generation = runtime.capture_command_ingress()
                    if (
                        mailbox_generation_before_disconnect is not None
                        and generation > mailbox_generation_before_disconnect
                    ):
                        mailbox_generation_after_disconnect = generation
                        reconnect_generation_advanced = True
                        disconnected_stopped = True
                        _write_ack(
                            scenario_dir / "command_disconnected.ack",
                            {
                                "stopped": True,
                                "mailbox_generation": generation,
                            },
                        )

            if (scenario_dir / "command_reconnected_wait.active").exists():
                if (
                    _topic_peer_connected(
                        transport_status,
                        interface_config.wheel_command.topic,
                    )
                    and runtime.last_decision.waiting
                    and wheel_state is not None
                    and _drive_is_stopped(wheel_state.drive_wheel_speed_rad_s)
                ):
                    reconnect_wait_seen = True
                    _write_ack(
                        scenario_dir / "command_reconnected_wait.ack",
                        {"old_command_stayed_stopped": True},
                    )

            if (scenario_dir / "new_command.active").exists():
                topic_states = {
                    topic: topic_status.state
                    for topic, topic_status in status.topics.items()
                }
                if (
                    reconnect_wait_seen
                    and status.topics[interface_config.wheel_command.topic].state
                    == "active"
                    and any(
                        abs(value) > 0.1
                        for value in runtime.last_decision.drive_wheel_speed_rad_s
                    )
                    and all(state == "active" for state in topic_states.values())
                ):
                    reconnect_required_new_command = (
                        disconnected_stopped and reconnect_generation_advanced
                    )
                    accepted_peer_states = topic_states
                    _write_final_protocol_ack(
                        runtime,
                        transport,
                        logger,
                        scenario_dir / "new_command.ack",
                    )

            pacer.wait_for_next_deadline()

        if (
            normal_load_start_snapshot is None
            or normal_load_end_snapshot is None
            or normal_load_log_start_snapshot is None
            or normal_load_log_end_snapshot is None
        ):
            raise RuntimeError("normal-load transport/logger window was not captured")
        if (
            mailbox_generation_before_disconnect is None
            or mailbox_generation_after_disconnect is None
        ):
            raise RuntimeError("reconnect mailbox generations were not captured")
        if (
            warmup_requested_sec is None
            or normal_load_started_at is None
            or normal_load_ended_at is None
            or normal_load_sim_started_ns is None
            or normal_load_sim_ended_ns is None
            or measurement_window is None
            or normal_load_base_start is None
            or normal_load_base_last is None
            or normal_load_motion_evidence is None
            or normal_load_requested_duration_sec is None
        ):
            raise RuntimeError("normal-load timing and motion evidence was not captured")
        fault_injection_transport_snapshot = transport.snapshot()
        log_delta = _logger_snapshot_delta(
            normal_load_log_start_snapshot,
            normal_load_log_end_snapshot,
        )
        normal_load_wall_duration_sec = normal_load_ended_at - normal_load_started_at
        normal_load_sim_duration_sec = (
            normal_load_sim_ended_ns - normal_load_sim_started_ns
        ) / 1_000_000_000.0
        rtk_displacement_m = (
            math.dist(normal_load_rtk_start, normal_load_rtk_last)
            if normal_load_rtk_start is not None and normal_load_rtk_last is not None
            else 0.0
        )
        base_displacement_m = math.dist(
            normal_load_base_start,
            normal_load_base_last,
        )
        left_steering_peak = (
            normal_load_steering_peaks[0]
            if len(normal_load_steering_peaks) >= 1
            else 0.0
        )
        right_steering_peak = (
            normal_load_steering_peaks[1]
            if len(normal_load_steering_peaks) >= 2
            else 0.0
        )
        lidar_service_snapshot_result = _lidar_service_snapshot_result(runtime)
        result = {
            **normal_load_motion_evidence,
            "transport": "ecal",
            "runtime": "simulation",
            "robot_model": selected_robot_model,
            "feedback_is_not_command_echo": feedback_is_not_command_echo,
            "invalid_command_rejected": invalid_command_rejected,
            "timeout_stopped_vehicle": timeout_stopped_vehicle,
            "timeout_preserved_steering": timeout_preserved_steering,
            "output_disconnect_isolated": output_disconnect_isolated,
            "per_topic_peer_states": accepted_peer_states,
            "reconnect_required_new_command": reconnect_required_new_command,
            "reconnect_generation_advanced": reconnect_generation_advanced,
            "mailbox_generation_before_disconnect": mailbox_generation_before_disconnect,
            "mailbox_generation_after_disconnect": mailbox_generation_after_disconnect,
            "lidar_service_snapshot": lidar_service_snapshot_result,
            "normal_load_obstacle_count": obstacle_count,
            "normal_load_log_sample_count": len(normal_load_log_samples),
            "normal_load_log_samples": normal_load_log_samples,
            "normal_load_log_accepted_messages": log_delta["accepted_messages"],
            "normal_load_log_accepted_events": log_delta["accepted_events"],
            "normal_load_log_max_pending": max(
                (
                    depth
                    for _sampled_at, depth, _completed in normal_load_log_samples
                ),
                default=0,
            ),
            "normal_load_log_final_pending": log_delta["final_pending"],
            "normal_load_log_sustained_backlog": _has_sustained_backlog(
                normal_load_log_samples
            ),
            "normal_load_log_dropped_messages": log_delta["dropped_messages"],
            "normal_load_log_dropped_events": log_delta["dropped_events"],
            "normal_load_log_writer_failed": log_delta["writer_failed"],
            "normal_load_warmup_requested_sec": warmup_requested_sec,
            "normal_load_warmup_wall_duration_sec": warmup_wall_duration_sec,
            "normal_load_warmup_sim_duration_sec": warmup_sim_duration_sec,
            "normal_load_warmup_physics_steps": warmup_physics_steps,
            "normal_load_warmup_log_accepted_messages": (
                warmup_log_accepted_messages
            ),
            "normal_load_warmup_topic_counts": warmup_topic_counts,
            "normal_load_command_states": sorted(normal_load_command_states),
            "normal_load_requested_duration_sec": normal_load_requested_duration_sec,
            "normal_load_window_start_sim_time_ns": normal_load_sim_started_ns,
            "normal_load_window_end_sim_time_ns": normal_load_sim_ended_ns,
            "normal_load_measurement_wall_duration_sec": (
                measurement_window[1] - measurement_window[0]
            ),
            "normal_load_sim_wall_ratio": (
                normal_load_sim_duration_sec / normal_load_wall_duration_sec
                if normal_load_wall_duration_sec > 0.0
                else 0.0
            ),
            "normal_load_control_duration_sec": 0.0,
            "normal_load_rtk_displacement_m": rtk_displacement_m,
            "normal_load_base_displacement_m": base_displacement_m,
            "normal_load_trajectory_distance_m": normal_load_base_trajectory_m,
            "normal_load_average_speed_m_s": (
                normal_load_base_trajectory_m / normal_load_wall_duration_sec
                if normal_load_wall_duration_sec > 0.0
                else 0.0
            ),
            "normal_load_nonzero_drive_feedback_wheels": sum(
                peak >= 0.1 for peak in normal_load_drive_peaks
            ),
            "normal_load_peak_left_steering_angle_rad": left_steering_peak,
            "normal_load_peak_right_steering_angle_rad": right_steering_peak,
            "normal_load_steering_same_sign": normal_load_steering_same_sign,
            "normal_load_peak_steering_angle_rad": max(
                left_steering_peak,
                right_steering_peak,
            ),
            "state_history": state_history,
            "transport_snapshot": _transport_snapshot_delta(
                normal_load_start_snapshot,
                normal_load_end_snapshot,
            ),
            "fault_injection_transport_snapshot": _transport_snapshot_delta(
                normal_load_end_snapshot,
                fault_injection_transport_snapshot,
            ),
        }
    finally:
        if previous_thread_switch_interval_sec is not None:
            _restore_command_callback_scheduling(
                previous_thread_switch_interval_sec,
            )
        if cyclic_gc_was_enabled is not None:
            _restore_cyclic_gc(cyclic_gc_was_enabled)
        close_error: BaseException | None = None
        close_trace: tuple[str, ...] = ()
        if session is not None:
            try:
                log_paths = session.close()
                close_trace = runtime.close_trace
            except BaseException as exc:
                close_error = exc
        p.disconnect(client_id)
        if close_error is not None:
            raise close_error

    result["close_trace"] = close_trace
    result["clean_shutdown"] = close_trace == (
        "stop_commands",
        "safe_stop",
        "stop_sensors",
        "quiesce_transport",
        "close_log",
        "close_transport",
        "close_sensors",
    )
    if log_paths is None or measurement_window is None or runtime is None:
        raise RuntimeError("normal-load interface log paths were not captured")
    command_topic = runtime.config.wheel_command.topic
    output_topics = tuple(
        channel.topic for channel in runtime.config.channels if channel.direction == "publish"
    )
    commands, published, sequence_contiguous = _measurement_log_events(
        read_interface_log(log_paths.binary_path),
        measurement_start=measurement_window[0],
        measurement_end=measurement_window[1],
        start_sim_time_ns=normal_load_sim_started_ns,
        end_sim_time_ns=normal_load_sim_ended_ns,
        command_topic=command_topic,
        output_topics=output_topics,
    )
    result["normal_load_received_commands"] = commands
    result["normal_load_published"] = published
    result["normal_load_log_sequence_contiguous"] = sequence_contiguous
    result["normal_load_control_duration_sec"] = (
        float(commands[-1]["wall_time"]) - float(commands[0]["wall_time"])
        if len(commands) >= 2
        else 0.0
    )
    evidence_root = result_json.parent.resolve(strict=True)
    try:
        binary_log_relative = log_paths.binary_path.resolve(strict=True).relative_to(
            evidence_root
        )
        event_log_relative = log_paths.event_path.resolve(strict=True).relative_to(
            evidence_root
        )
    except ValueError as exc:
        raise RuntimeError("interface log escaped the result evidence directory") from exc
    result["interface_log_files"] = {
        "binary": binary_log_relative.as_posix(),
        "events": event_log_relative.as_posix(),
    }
    result["interface_binary_log"] = str(log_paths.binary_path)
    result["interface_event_log"] = str(log_paths.event_path)
    result_json.parent.mkdir(parents=True, exist_ok=True)
    result_json.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the real eCAL PyBullet runtime")
    parser.add_argument("--result-json", type=Path, required=True)
    parser.add_argument("--scenario-dir", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--start-file", type=Path, required=True)
    parser.add_argument("--stop-file", type=Path, required=True)
    parser.add_argument("--participant-name", default="slope-sim-runtime")
    parser.add_argument("--max-runtime-sec", type=float, default=18.0)
    parser.add_argument(
        "--robot-model",
        choices=robot_model_names(),
        default="active_steering_4wd",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        run_simulation_runtime(
            result_json=args.result_json,
            scenario_dir=args.scenario_dir,
            ready_file=args.ready_file,
            start_file=args.start_file,
            stop_file=args.stop_file,
            participant_name=args.participant_name,
            max_runtime_sec=args.max_runtime_sec,
            robot_model=args.robot_model,
        )
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
