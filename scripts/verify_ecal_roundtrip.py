#!/usr/bin/env python3
# 真实 eCAL 双进程门禁：验证六话题时序、物理运动、安全断连与原始日志链。
from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from threading import Lock
import time
from typing import Callable, Iterable, Mapping, Sequence
from uuid import uuid4


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.config import ChannelConfig, InterfaceConfig
from slope_sim.interfaces.ecal_transport import EcalTransport, create_transport
from slope_sim.interfaces.logging import InterfaceLogRecord, iter_interface_log
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.interfaces.wheel import WheelCommandMailbox
from slope_sim.model_registry import get_robot_model, robot_model_names
from scripts.ecal_roundtrip_peer import (
    _SCENARIO_ACK_TIMEOUT_SEC,
    _channel_period_sec,
    _command_topic_type,
    _message_timestamp_ns,
    _output_topic_types,
)


_PEER_SCRIPT = Path(__file__).with_name("ecal_roundtrip_peer.py")
_SIMULATION_RUNTIME_SCRIPT = Path(__file__).with_name("ecal_simulation_runtime.py")
_SUPPORTED_RUNTIMES = frozenset({"transport", "simulation"})
_DEFAULT_WARMUP_SEC = 1.0
_DEFAULT_PROCESS_TIMEOUT_SEC = 60.0
_MIN_COUNT_RATIO = 0.85
_MAX_COUNT_RATIO = 1.15
_MIN_WINDOW_COVERAGE_RATIO = 0.80
_SILENCE_SAMPLE_PERIOD_SEC = 0.005
_FIRST_RECONNECT_PEER_DURATION_SEC = 5.0
_ECAL_REGISTRATION_TIMEOUT_SEC = 10.0
_ECAL_REGISTRATION_MARGIN_SEC = 2.0
_CHILD_TERMINATE_TIMEOUT_SEC = 3.0
_SIMULATION_FIXED_PROTOCOL_WAIT_COUNT = 10
_SIMULATION_PER_OUTPUT_WAIT_COUNT = 3
_SIMULATION_SCENARIO_MARGIN_SEC = 2.0
_MAX_INTERARRIVAL_PERIODS = 2.5
_MIN_WARMUP_SIM_WALL_RATIO = 0.95
_MIN_SIM_WALL_RATIO = 0.98
_MAX_SIM_WALL_RATIO = 1.02
_MIN_MOTION_DISPLACEMENT_M = 0.50
_MIN_AVERAGE_SPEED_M_S = 0.10
_MIN_STEERING_ANGLE_RAD = 0.10


@dataclass(frozen=True)
class RoundtripResult:
    """2.5 秒真实进程环回的六话题统计。"""

    transport_name: str
    peer_returncode: int
    wall_clock_hz: Mapping[str, float]
    message_timestamp_hz: Mapping[str, float]
    received_topics: set[str]
    topic_types: Mapping[str, str]
    message_counts: Mapping[str, int]
    dropped_count: int
    duration_sec: float = 2.5
    event_span_sec: Mapping[str, float] = field(default_factory=dict)
    max_interarrival_gap_sec: Mapping[str, float] = field(default_factory=dict)
    end_to_end_timestamp_match: Mapping[str, bool] = field(default_factory=dict)
    peer_dropped_count: int = 0
    transport_error_count: int = 0
    peer_error_count: int = 0
    runtime_name: str = "transport"
    robot_model: str = ""
    feedback_is_not_command_echo: bool = False
    invalid_command_rejected: bool = False
    timeout_stopped_vehicle: bool = False
    timeout_preserved_steering: bool = False
    output_disconnect_isolated: Mapping[str, bool] = field(default_factory=dict)
    per_topic_peer_states: Mapping[str, str] = field(default_factory=dict)
    reconnect_required_new_command: bool = False
    reconnect_generation_advanced: bool = False
    mailbox_generation_before_disconnect: int = 0
    mailbox_generation_after_disconnect: int = 0
    normal_load_obstacle_count: int = 0
    normal_load_log_sample_count: int = 0
    normal_load_log_accepted_messages: int = 0
    normal_load_log_accepted_events: int = 0
    normal_load_log_max_pending: int = 0
    normal_load_log_final_pending: int = 0
    normal_load_log_sustained_backlog: bool = True
    normal_load_log_dropped_messages: int = 0
    normal_load_log_dropped_events: int = 0
    normal_load_log_writer_failed: bool = True
    normal_load_log_sequence_contiguous: bool = False
    normal_load_requested_duration_sec: float = 0.0
    peer_measurement_duration_sec: float = 0.0
    normal_load_physics_time_step_sec: float = 0.0
    normal_load_step_count: int = 0
    normal_load_sim_duration_sec: float = 0.0
    normal_load_wall_duration_sec: float = 0.0
    normal_load_control_step_count: int = 0
    normal_load_control_sim_duration_sec: float = 0.0
    normal_load_control_wall_duration_sec: float = 0.0
    normal_load_controlled_motion_step_count: int = 0
    normal_load_controlled_motion_sim_duration_sec: float = 0.0
    normal_load_obstacle_contact_step_count: int = 0
    normal_load_controlled_displacement_m: float = 0.0
    normal_load_controlled_path_length_m: float = 0.0
    normal_load_controlled_mean_speed_m_s: float = 0.0
    normal_load_controlled_max_speed_m_s: float = 0.0
    normal_load_warmup_requested_sec: float = 0.0
    normal_load_warmup_wall_duration_sec: float = 0.0
    normal_load_warmup_sim_duration_sec: float = 0.0
    normal_load_warmup_physics_steps: int = 0
    normal_load_warmup_log_accepted_messages: int = 0
    normal_load_warmup_topic_counts: Mapping[str, int] = field(default_factory=dict)
    normal_load_command_states: tuple[str, ...] = ()
    normal_load_measurement_wall_duration_sec: float = 0.0
    normal_load_sim_wall_ratio: float = 0.0
    normal_load_control_duration_sec: float = 0.0
    normal_load_rtk_displacement_m: float = 0.0
    normal_load_base_displacement_m: float = 0.0
    normal_load_base_path_length_m: float = 0.0
    normal_load_base_mean_speed_m_s: float = 0.0
    normal_load_base_max_speed_m_s: float = 0.0
    normal_load_trajectory_distance_m: float = 0.0
    normal_load_average_speed_m_s: float = 0.0
    normal_load_nonzero_drive_feedback_wheels: int = 0
    normal_load_peak_left_steering_angle_rad: float = 0.0
    normal_load_peak_right_steering_angle_rad: float = 0.0
    normal_load_steering_same_sign: bool = False
    normal_load_peak_steering_angle_rad: float = 0.0
    peer_rtk_displacement_m: float = 0.0
    logged_rtk_displacement_m: float = 0.0
    rtk_log_match_count: int = 0
    rtk_log_max_position_error_m: float = math.inf
    normal_load_window_start_sim_time_ns: int = 0
    normal_load_window_end_sim_time_ns: int = 0
    wheel_drain_timestamp_ns: int = 0
    wheel_log_publish_count: int = 0
    wheel_peer_receive_count: int = 0
    wheel_log_match_count: int = 0
    wheel_drain_complete: bool = False
    clean_shutdown: bool = False


@dataclass(frozen=True)
class RtkLogChainEvidence:
    """同一批 RTK 在真实 peer 与原始接口日志中的交叉校验结果。"""

    peer_displacement_m: float
    logged_displacement_m: float
    match_count: int
    max_position_error_m: float


@dataclass(frozen=True)
class WheelLogDeliveryEvidence:
    """正式仿真窗口内 wheel 原始 publish 与真实 peer 的精确对齐结果。"""

    logged_count: int
    peer_count: int
    match_count: int


@dataclass(frozen=True)
class ReconnectResult:
    """真实 peer 退出和重启期间的连接状态与驱动目标快照。"""

    transport_name: str
    states: tuple[str, ...]
    drive_target_before_disconnect: tuple[float, float]
    mailbox_generation_before_disconnect: int
    mailbox_generation_after_disconnect: int
    first_peer_terminated: bool
    first_peer_returncode: int
    first_peer_runtime_sec: float
    first_peer_planned_duration_sec: float
    drive_target_while_disconnected: tuple[float, float]
    drive_target_after_peer_restart_before_new_command: tuple[float, float]
    silence_observed_sec: float
    silence_sample_count: int
    silence_all_zero: bool
    drive_target_after_new_command: tuple[float, float]


def _positive_finite(name: str, value: float) -> float:
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return normalized


def _nonnegative_finite(name: str, value: float) -> float:
    normalized = float(value)
    if not math.isfinite(normalized) or normalized < 0.0:
        raise ValueError(f"{name} must be a nonnegative finite number")
    return normalized


def _simulation_scenario_budget_sec(
    *,
    duration_sec: float,
    warmup_sec: float,
    startup_timeout_sec: float,
) -> float:
    """计算串行故障注入协议的总寿命，不把单次启动上限误作总时限。"""
    duration = _positive_finite("duration_sec", duration_sec)
    warmup = _positive_finite("warmup_sec", warmup_sec)
    startup_timeout = _positive_finite(
        "startup_timeout_sec",
        startup_timeout_sec,
    )
    output_count = sum(
        channel.direction == "publish"
        for channel in InterfaceConfig.default(transport_mode="ecal").channels
    )
    serial_wait_count = (
        _SIMULATION_FIXED_PROTOCOL_WAIT_COUNT
        + _SIMULATION_PER_OUTPUT_WAIT_COUNT * output_count
    )
    return (
        startup_timeout
        + warmup
        + duration
        + serial_wait_count * _SCENARIO_ACK_TIMEOUT_SEC
        + _SIMULATION_SCENARIO_MARGIN_SEC
    )


def _frequency_hz(values: Sequence[float]) -> float:
    """按首末事件间隔估算墙钟频率，少于两条时明确返回零。"""
    if len(values) < 2:
        return 0.0
    elapsed = float(values[-1]) - float(values[0])
    if elapsed <= 0.0:
        return 0.0
    return (len(values) - 1) / elapsed


def _timestamp_frequency_hz(values: Sequence[int]) -> float:
    """按消息自身纳秒时间戳估算仿真消息频率。"""
    if len(values) < 2:
        return 0.0
    elapsed_ns = int(values[-1]) - int(values[0])
    if elapsed_ns <= 0:
        return 0.0
    return (len(values) - 1) * 1_000_000_000.0 / elapsed_ns


def _configured_topics(
    config: InterfaceConfig,
) -> tuple[str, str, tuple[str, ...], tuple[str, ...]]:
    """从集中配置返回命令、轮态、四传感器及五输出话题。"""
    command_topic = config.wheel_command.topic
    wheel_state_topic = config.wheel_state.topic
    sensor_topics = (
        config.lidar_front.topic,
        config.lidar_rear.topic,
        config.rtk.topic,
        config.imu.topic,
    )
    return command_topic, wheel_state_topic, sensor_topics, (
        wheel_state_topic,
        *sensor_topics,
    )


def _expected_topic_types(
    config: InterfaceConfig,
    codec: ProtoCodec,
) -> dict[str, str]:
    """由默认配置和 codec 生成六话题期望类型。"""
    return {
        config.wheel_command.topic: _command_topic_type(config, codec),
        **_output_topic_types(config, codec),
    }


def _event_span_sec(values: Sequence[float]) -> float:
    """返回事件覆盖的墙钟窗口；不足两条时覆盖为零。"""
    if len(values) < 2:
        return 0.0
    return max(0.0, float(values[-1]) - float(values[0]))


def _maximum_event_gap_sec(
    values: Sequence[float],
    window_start: float,
    window_end: float,
) -> float:
    """返回含窗口首尾的最大事件空档；越界或倒序直接判为无穷。"""
    start = float(window_start)
    end = float(window_end)
    normalized = [float(value) for value in values]
    if (
        not math.isfinite(start)
        or not math.isfinite(end)
        or end <= start
        or any(not math.isfinite(value) for value in normalized)
        or any(value < start or value > end for value in normalized)
        or any(current < previous for previous, current in zip(normalized, normalized[1:]))
    ):
        return math.inf
    points = [start, *normalized, end]
    return max(current - previous for previous, current in zip(points, points[1:]))


def _timestamp_sequences_match(
    produced: Sequence[int],
    consumed: Sequence[int],
) -> bool:
    """逐条匹配端到端 timestamp，并拒绝空、重复和乱序序列。"""
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value < 0
        for value in (*produced, *consumed)
    ):
        raise AssertionError("timestamp sequences must contain nonnegative integers")
    source = tuple(produced)
    sink = tuple(consumed)
    return (
        bool(source)
        and source == sink
        and all(current > previous for previous, current in zip(source, source[1:]))
    )


def _all_topic_peers_connected(transport: EcalTransport) -> bool:
    """只有六个端点均完成 discovery 才允许进入测量窗口。"""
    transport.poll_peer_state()
    quality = transport.snapshot().topic_quality
    return bool(quality) and all(item.peer_connected is True for item in quality)


def _polled_transport_state(transport: EcalTransport) -> str:
    """主动推进 discovery 后读取 lifecycle，避免消费发送线程留下的旧状态。"""
    transport.poll_peer_state()
    return transport.snapshot().state


def _start_peer(
    *,
    result_json: Path,
    ready_file: Path,
    start_file: Path,
    duration_sec: float,
    participant_name: str,
    command: tuple[float, float] = (4.0, 4.0),
    command_delay_sec: float = 0.0,
) -> subprocess.Popen[str]:
    """只用当前 slope-sim Python 启动独立真实 eCAL peer。"""
    return subprocess.Popen(
        [
            sys.executable,
            str(_PEER_SCRIPT),
            "--result-json",
            str(result_json),
            "--ready-file",
            str(ready_file),
            "--start-file",
            str(start_file),
            "--duration-sec",
            str(duration_sec),
            "--participant-name",
            participant_name,
            "--drive-command",
            str(command[0]),
            str(command[1]),
            "--command-delay-sec",
            str(command_delay_sec),
        ],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )


def _start_simulation_runtime(
    *,
    result_json: Path,
    scenario_dir: Path,
    ready_file: Path,
    start_file: Path,
    stop_file: Path,
    participant_name: str,
    max_runtime_sec: float,
    robot_model: str = "active_steering_4wd",
) -> subprocess.Popen[str]:
    """启动正式 PyBullet DIRECT runtime 子进程。"""
    return subprocess.Popen(
        [
            sys.executable,
            str(_SIMULATION_RUNTIME_SCRIPT),
            "--result-json",
            str(result_json),
            "--scenario-dir",
            str(scenario_dir),
            "--ready-file",
            str(ready_file),
            "--start-file",
            str(start_file),
            "--stop-file",
            str(stop_file),
            "--participant-name",
            participant_name,
            "--max-runtime-sec",
            str(max_runtime_sec),
            "--robot-model",
            robot_model,
        ],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )


def _start_simulation_peer(
    *,
    result_json: Path,
    scenario_dir: Path,
    ready_file: Path,
    start_file: Path,
    duration_sec: float,
    warmup_sec: float,
    participant_name: str,
    start_timeout_sec: float,
    robot_model: str = "active_steering_4wd",
) -> subprocess.Popen[str]:
    """启动执行完整物理与安全场景的官方 eCAL peer。"""
    return subprocess.Popen(
        [
            sys.executable,
            str(_PEER_SCRIPT),
            "--result-json",
            str(result_json),
            "--scenario-dir",
            str(scenario_dir),
            "--ready-file",
            str(ready_file),
            "--start-file",
            str(start_file),
            "--duration-sec",
            str(duration_sec),
            "--warmup-sec",
            str(warmup_sec),
            "--participant-name",
            participant_name,
            "--start-timeout-sec",
            str(start_timeout_sec),
            "--robot-model",
            robot_model,
            "--simulation-scenario",
        ],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )


def _wait_until(
    predicate: Callable[[], bool],
    *,
    timeout_sec: float,
    description: str,
    process: subprocess.Popen[str] | None = None,
    process_name: str = "peer",
) -> None:
    """按条件等待 discovery/消息，不用固定睡眠冒充连接成功。"""
    deadline = time.monotonic() + timeout_sec
    while not predicate():
        if process is not None and process.poll() is not None:
            stdout, stderr = process.communicate()
            raise RuntimeError(
                f"{process_name} exited before {description} (code={process.returncode}): "
                f"{stdout.strip()} {stderr.strip()}"
            )
        if time.monotonic() >= deadline:
            raise TimeoutError(f"timed out waiting for {description}")
        time.sleep(0.010)


def _finish_peer(
    process: subprocess.Popen[str],
    *,
    timeout_sec: float,
    process_name: str = "peer",
) -> tuple[int, str, str]:
    """等待 peer 正常退出；超时时终止并把证据带回门禁。"""
    try:
        stdout, stderr = process.communicate(timeout=timeout_sec)
    except subprocess.TimeoutExpired as exc:
        process.terminate()
        try:
            stdout, stderr = process.communicate(timeout=_CHILD_TERMINATE_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            process.kill()
            stdout, stderr = process.communicate(timeout=_CHILD_TERMINATE_TIMEOUT_SEC)
        raise TimeoutError(
            f"{process_name} did not exit: {stdout.strip()} {stderr.strip()}"
        ) from exc
    if process.returncode != 0:
        raise RuntimeError(
            f"{process_name} exited with code {process.returncode}: "
            f"{stdout.strip()} {stderr.strip()}"
        )
    return process.returncode, stdout, stderr


def _reap_process(process: subprocess.Popen[str] | None) -> None:
    """异常路径也有界终止并回收子进程，避免残留 eCAL participant。"""
    if process is None:
        return
    if process.poll() is None:
        process.terminate()
        try:
            process.communicate(timeout=_CHILD_TERMINATE_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=_CHILD_TERMINATE_TIMEOUT_SEC)
    else:
        process.communicate()


def _terminate_running_peer(
    process: subprocess.Popen[str],
    *,
    timeout_sec: float,
) -> int:
    """主动终止仍在运行的 peer，并要求操作产生非零退出码。"""
    timeout = _positive_finite("timeout_sec", timeout_sec)
    if process.poll() is not None:
        raise AssertionError(
            f"peer exited naturally before forced termination: {process.returncode}"
        )
    process.terminate()
    try:
        process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate(timeout=timeout)
    if process.returncode is None or process.returncode == 0:
        raise AssertionError(
            f"forced peer termination returned invalid code: {process.returncode}"
        )
    return int(process.returncode)


def _publish_output(
    transport: EcalTransport,
    codec: ProtoCodec,
    topic: str,
    message: object,
    timestamp_ns: int,
    *,
    wall_time: float,
) -> None:
    payload = codec.encode(message)
    transport.publish(
        topic,
        payload,
        codec.type_name(message),
        timestamp_ns,
        wall_time=wall_time,
    )


def _run_output_schedule(
    transport: EcalTransport,
    *,
    duration_sec: float,
    config: InterfaceConfig,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> None:
    """无 PyBullet 地按五个输出通道各自配置的墙钟频率发布。"""
    codec = ProtoCodec()
    started_at = monotonic()
    ended_at = started_at + duration_sec
    outputs: tuple[tuple[ChannelConfig, Callable[[int], object]], ...] = (
        (
            config.wheel_state,
            lambda timestamp_ns: WheelState(timestamp_ns, (4.0, 4.0), ()),
        ),
        (
            config.lidar_front,
            lambda timestamp_ns: LidarPointCloud(
                timestamp_ns, "lidar_front", 0, 1, ()
            ),
        ),
        (
            config.lidar_rear,
            lambda timestamp_ns: LidarPointCloud(
                timestamp_ns, "lidar_rear", 0, 2, ()
            ),
        ),
        (
            config.rtk,
            lambda timestamp_ns: RtkState(
                timestamp_ns, 1.0, 2.0, 3.0, 0.25
            ),
        ),
        (
            config.imu,
            lambda timestamp_ns: ImuAttitude(timestamp_ns, 0.05, -0.10),
        ),
    )
    next_publish = {channel.topic: started_at for channel, _factory in outputs}
    message_indexes = {channel.topic: 0 for channel, _factory in outputs}

    while True:
        now = monotonic()
        if now >= ended_at:
            break
        for channel, message_factory in outputs:
            due_at = next_publish[channel.topic]
            while due_at <= now and due_at < ended_at:
                message_indexes[channel.topic] += 1
                timestamp_ns = _message_timestamp_ns(
                    message_indexes[channel.topic],
                    channel,
                )
                _publish_output(
                    transport,
                    codec,
                    channel.topic,
                    message_factory(timestamp_ns),
                    timestamp_ns,
                    wall_time=monotonic(),
                )
                due_at = (
                    started_at
                    + message_indexes[channel.topic] * _channel_period_sec(channel)
                )
            next_publish[channel.topic] = due_at

        sleep_until = min(*next_publish.values(), ended_at)
        remaining = sleep_until - monotonic()
        if remaining > 0.0:
            sleep(remaining)


def _single_type(topic: str, events: Sequence[Mapping[str, object]]) -> str:
    raw_types = [event.get("type") for event in events]
    if any(not isinstance(value, str) or not value for value in raw_types):
        raise AssertionError(f"{topic} event type must be a nonempty string")
    observed = set(raw_types)
    if len(observed) != 1:
        raise AssertionError(f"{topic} observed message types: {sorted(observed)}")
    return observed.pop()


def _assert_roundtrip_result(
    result: RoundtripResult,
    *,
    config: InterfaceConfig | None = None,
    codec: ProtoCodec | None = None,
) -> None:
    """执行脚本自身硬门禁，确保命令行 PASS 与 pytest 判据一致。"""
    selected_config = (
        InterfaceConfig.default(transport_mode="ecal") if config is None else config
    )
    selected_codec = ProtoCodec() if codec is None else codec
    command_topic, wheel_state_topic, _sensor_topics, output_topics = _configured_topics(
        selected_config
    )
    expected_types = _expected_topic_types(selected_config, selected_codec)
    if result.transport_name != "ecal":
        raise AssertionError("transport must be ecal")
    if result.peer_returncode != 0:
        raise AssertionError(f"peer returned {result.peer_returncode}")
    if result.received_topics != set(output_topics):
        raise AssertionError(
            f"missing output topics: {set(output_topics) - result.received_topics}"
        )
    if dict(result.topic_types) != expected_types:
        raise AssertionError(f"message type mismatch: {dict(result.topic_types)}")
    if result.dropped_count != 0 or result.peer_dropped_count != 0:
        raise AssertionError(
            "normal load dropped messages: "
            f"transport={result.dropped_count}, peer={result.peer_dropped_count}"
        )
    if result.transport_error_count != 0 or result.peer_error_count != 0:
        raise AssertionError(
            "normal load transport errors: "
            f"transport={result.transport_error_count}, peer={result.peer_error_count}"
        )

    duration = _positive_finite("result.duration_sec", result.duration_sec)
    target_rates = {
        channel.topic: float(channel.rate_hz) for channel in selected_config.channels
    }
    for topic, target_hz in target_rates.items():
        expected_count = duration * target_hz
        minimum_count = max(2, math.floor(expected_count * _MIN_COUNT_RATIO))
        maximum_count = math.ceil(expected_count * _MAX_COUNT_RATIO)
        count = result.message_counts.get(topic, 0)
        if not minimum_count <= count <= maximum_count:
            raise AssertionError(
                f"{topic} count {count} outside {minimum_count}..{maximum_count}"
            )
        coverage = result.event_span_sec.get(topic, 0.0)
        minimum_coverage = duration * _MIN_WINDOW_COVERAGE_RATIO
        if coverage < minimum_coverage:
            raise AssertionError(
                f"{topic} coverage {coverage:.3f}s below {minimum_coverage:.3f}s"
            )
        maximum_gap = result.max_interarrival_gap_sec.get(topic, math.inf)
        allowed_gap = _MAX_INTERARRIVAL_PERIODS / target_hz
        if not math.isfinite(maximum_gap) or maximum_gap > allowed_gap:
            raise AssertionError(
                f"{topic} maximum gap {maximum_gap:.6f}s exceeds "
                f"{allowed_gap:.6f}s"
            )

    wheel_topics = {command_topic, wheel_state_topic}
    for channel in selected_config.channels:
        topic = channel.topic
        target_hz = float(channel.rate_hz)
        wall_tolerance = 0.05 if topic in wheel_topics else 0.10
        wall_lower = target_hz * (1.0 - wall_tolerance)
        wall_upper = target_hz * (1.0 + wall_tolerance)
        if not wall_lower <= result.wall_clock_hz[topic] <= wall_upper:
            raise AssertionError(
                f"{topic} wall frequency {result.wall_clock_hz[topic]:.3f} Hz "
                f"outside {wall_lower:.3f}..{wall_upper:.3f} Hz"
            )

        timestamp_lower = target_hz * (1.0 - 0.01)
        timestamp_upper = target_hz * (1.0 + 0.01)
        if not (
            timestamp_lower
            <= result.message_timestamp_hz[topic]
            <= timestamp_upper
        ):
            raise AssertionError(
                f"{topic} timestamp frequency "
                f"{result.message_timestamp_hz[topic]:.3f} Hz outside "
                f"{timestamp_lower:.3f}..{timestamp_upper:.3f} Hz"
            )


def _run_ecal_transport_roundtrip(
    *,
    duration_sec: float,
    process_timeout_sec: float,
) -> RoundtripResult:
    """启动独立 peer，并在当前进程运行无物理真实 eCAL harness。"""
    duration = duration_sec
    token = uuid4().hex[:10]
    config = InterfaceConfig.default(transport_mode="ecal")
    codec = ProtoCodec()
    command_topic, _wheel_state_topic, _sensor_topics, output_topics = (
        _configured_topics(config)
    )
    expected_types = _expected_topic_types(config, codec)
    command_lock = Lock()
    command_events: list[dict[str, object]] = []

    with tempfile.TemporaryDirectory(prefix="slope-ecal-roundtrip-") as temp_dir:
        temp_path = Path(temp_dir)
        result_path = temp_path / "peer-result.json"
        ready_path = temp_path / "peer.ready"
        start_path = temp_path / "start.signal"
        transport = create_transport(
            "ecal",
            config=config,
            participant_name=f"slope-sim-harness-{token}",
        )
        if not isinstance(transport, EcalTransport) or transport.snapshot().mode != "ecal":
            transport.close()
            raise RuntimeError("strict harness did not create an ecal transport")

        def record_command(payload: bytes, received_at: float) -> bool:
            command = codec.decode_wheel_command(payload)
            with command_lock:
                command_events.append(
                    {
                        "wall_time": received_at,
                        "timestamp_ns": command.timestamp_ns,
                        "type": codec.type_name(command),
                    }
                )
            return True

        subscription = transport.subscribe(
            command_topic,
            expected_types[command_topic],
            record_command,
        )
        peer = _start_peer(
            result_json=result_path,
            ready_file=ready_path,
            start_file=start_path,
            duration_sec=duration,
            participant_name=f"slope-sim-peer-{token}",
        )
        try:
            _wait_until(
                ready_path.exists,
                timeout_sec=process_timeout_sec,
                description="peer readiness",
                process=peer,
            )
            _wait_until(
                lambda: _all_topic_peers_connected(transport),
                timeout_sec=process_timeout_sec,
                description="all six eCAL topic peers",
                process=peer,
            )
            start_path.write_text("start\n", encoding="utf-8")
            _run_output_schedule(transport, duration_sec=duration, config=config)
            peer_returncode, _stdout, _stderr = _finish_peer(
                peer,
                timeout_sec=process_timeout_sec,
            )
            if not result_path.exists():
                raise RuntimeError("peer exited without a JSON result")
            peer_result = _load_json_object(result_path, "transport peer")
            if peer_result.get("transport") != "ecal":
                raise AssertionError("peer did not report transport=ecal")
            transport_snapshot = transport.snapshot()
            peer_snapshot = peer_result.get("snapshot")
            if not isinstance(peer_snapshot, dict):
                raise AssertionError("peer result snapshot field must be an object")
        finally:
            _reap_process(peer)
            subscription.close()
            transport.close()

    with command_lock:
        events_by_topic: dict[str, list[Mapping[str, object]]] = {
            command_topic: list(command_events)
        }
    received_json = peer_result.get("received")
    if not isinstance(received_json, dict):
        raise AssertionError("peer result received field must be an object")
    for topic in output_topics:
        events_by_topic[topic] = _event_list(
            received_json.get(topic),
            f"peer result events for {topic}",
        )

    event_evidence = {
        topic: _strict_event_evidence(events, f"events for {topic}")
        for topic, events in events_by_topic.items()
    }

    wall_clock_hz = {
        topic: _frequency_hz(wall_times)
        for topic, (wall_times, _timestamps) in event_evidence.items()
    }
    timestamp_hz = {
        topic: _timestamp_frequency_hz(timestamps)
        for topic, (_wall_times, timestamps) in event_evidence.items()
    }
    message_counts = {topic: len(events) for topic, events in events_by_topic.items()}
    received_topics = {
        topic for topic in output_topics if message_counts.get(topic, 0) > 0
    }
    topic_types = {
        topic: _single_type(topic, events)
        for topic, events in events_by_topic.items()
    }
    event_spans = {
        topic: _event_span_sec(wall_times)
        for topic, (wall_times, _timestamps) in event_evidence.items()
    }
    max_gaps = {}
    for topic, (wall_times, _timestamps) in event_evidence.items():
        max_gaps[topic] = (
            _maximum_event_gap_sec(wall_times, wall_times[0], wall_times[-1])
            if wall_times
            else math.inf
        )
    result = RoundtripResult(
        transport_name="ecal",
        peer_returncode=peer_returncode,
        wall_clock_hz=wall_clock_hz,
        message_timestamp_hz=timestamp_hz,
        received_topics=received_topics,
        topic_types=topic_types,
        message_counts=message_counts,
        dropped_count=transport_snapshot.dropped_count,
        duration_sec=duration,
        event_span_sec=event_spans,
        max_interarrival_gap_sec=max_gaps,
        peer_dropped_count=_strict_nonnegative_int(
            peer_snapshot, "dropped_count", "transport peer snapshot"
        ),
        transport_error_count=transport_snapshot.error_count,
        peer_error_count=_strict_nonnegative_int(
            peer_snapshot, "error_count", "transport peer snapshot"
        ),
        runtime_name="transport",
        clean_shutdown=True,
    )
    _assert_roundtrip_result(result, config=config, codec=codec)
    return result


def _wait_for_live_processes(
    duration_sec: float,
    processes: Sequence[tuple[str, subprocess.Popen[str]]],
) -> None:
    """显式 warmup 期间持续确认两个子进程仍存活。"""
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        for name, process in processes:
            if process.poll() is not None:
                stdout, stderr = process.communicate()
                raise RuntimeError(
                    f"{name} exited during warmup (code={process.returncode}): "
                    f"{stdout.strip()} {stderr.strip()}"
                )
        remaining = deadline - time.monotonic()
        if remaining > 0.0:
            time.sleep(min(0.010, remaining))


def _load_json_object(path: Path, description: str) -> dict[str, object]:
    if not path.exists():
        raise RuntimeError(f"{description} exited without a JSON result")
    loaded = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise AssertionError(f"{description} result must be an object")
    return loaded


def _event_list(value: object, description: str) -> list[Mapping[str, object]]:
    if not isinstance(value, list) or any(not isinstance(item, dict) for item in value):
        raise AssertionError(f"{description} must be a list of event objects")
    return value


def _validate_simulation_command_cardinality(
    events: Sequence[Mapping[str, object]],
    *,
    robot_model: str,
) -> None:
    """独立核对 peer 命令事件的驱动/转向数组基数。"""
    model = get_robot_model(robot_model)
    expected = (
        len(model.drive_joint_names),
        len(model.steering_joint_names),
    )
    if not events:
        raise AssertionError("simulation peer command cardinality evidence is empty")
    for event in events:
        observed = (
            event.get("drive_wheel_count"),
            event.get("steering_wheel_count"),
        )
        if any(type(value) is not int for value in observed) or observed != expected:
            raise AssertionError(
                "simulation peer command cardinality does not match robot_model"
            )


def _simulation_measurement_events(
    peer_result: Mapping[str, object],
    runtime_result: Mapping[str, object],
    *,
    command_topic: str,
    output_topics: Sequence[str],
) -> dict[str, list[Mapping[str, object]]]:
    """命令使用 runtime 实收日志，输出使用 peer 实收事件。"""
    received = peer_result.get("received")
    if not isinstance(received, dict):
        raise AssertionError("simulation peer received field must be an object")
    events = {
        command_topic: _event_list(
            runtime_result.get("normal_load_received_commands"),
            "runtime received command events",
        )
    }
    for topic in output_topics:
        events[topic] = _event_list(
            received.get(topic),
            f"peer received events for {topic}",
        )
    return events


def _strict_finite_float(
    mapping: Mapping[str, object],
    key: str,
    description: str,
) -> float:
    """严格读取外部进程有限浮点证据，拒绝 bool 和字符串。"""
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AssertionError(f"{description} must report {key} as a finite number")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise AssertionError(f"{description} must report {key} as a finite number")
    return normalized


def _strict_string_tuple(
    mapping: Mapping[str, object],
    key: str,
    description: str,
) -> tuple[str, ...]:
    value = mapping.get(key)
    if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
        raise AssertionError(f"{description} must report {key} as a string list")
    return tuple(value)


def _strict_count_mapping(
    mapping: Mapping[str, object],
    key: str,
    description: str,
) -> dict[str, int]:
    value = mapping.get(key)
    if not isinstance(value, dict) or any(
        not isinstance(topic, str)
        or isinstance(count, bool)
        or not isinstance(count, int)
        or count < 0
        for topic, count in value.items()
    ):
        raise AssertionError(f"{description} must report {key} as nonnegative counts")
    return dict(value)


def _strict_bool_mapping(
    mapping: Mapping[str, object],
    key: str,
    description: str,
) -> dict[str, bool]:
    """严格读取字符串到 JSON bool 的映射，拒绝 0/1 等相等替代值。"""
    value = mapping.get(key)
    if not isinstance(value, dict) or any(
        not isinstance(topic, str) or not isinstance(state, bool)
        for topic, state in value.items()
    ):
        raise AssertionError(f"{description} must report {key} as a boolean mapping")
    return dict(value)


def _strict_string_mapping(
    mapping: Mapping[str, object],
    key: str,
    description: str,
) -> dict[str, str]:
    """严格读取字符串映射，避免外部状态被 str() 隐式规范化。"""
    value = mapping.get(key)
    if not isinstance(value, dict) or any(
        not isinstance(topic, str) or not isinstance(state, str) or not state
        for topic, state in value.items()
    ):
        raise AssertionError(f"{description} must report {key} as a string mapping")
    return dict(value)


def _strict_true(mapping: Mapping[str, object], key: str, description: str) -> bool:
    if mapping.get(key) is not True:
        raise AssertionError(f"{description} must report {key}=true")
    return True


def _strict_false(mapping: Mapping[str, object], key: str, description: str) -> bool:
    """只接受 JSON false，拒绝缺字段、零值和其他假值替代。"""
    if mapping.get(key) is not False:
        raise AssertionError(f"{description} must report {key}=false")
    return False


def _strict_model_steering_same_sign(
    mapping: Mapping[str, object],
    *,
    robot_model: str,
    description: str,
) -> bool:
    """按车型严格读取转向同向证据，差速车型必须明确为 false。"""
    key = "normal_load_steering_same_sign"
    if get_robot_model(robot_model).steering_joint_names:
        return _strict_true(mapping, key, description)
    return _strict_false(mapping, key, description)


def _strict_nonnegative_int(
    mapping: Mapping[str, object],
    key: str,
    description: str,
) -> int:
    """严格读取外部进程的非负整数证据，排除 bool 和字符串。"""
    value = mapping.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise AssertionError(f"{description} must report {key} as a nonnegative integer")
    return value


def _strict_nonnegative_float(
    mapping: Mapping[str, object],
    key: str,
    description: str,
) -> float:
    """严格读取外部进程的有限非负浮点证据，拒绝 bool 和字符串。"""
    value = mapping.get(key)
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0.0
    ):
        raise AssertionError(
            f"{description} must report {key} as a finite nonnegative number"
        )
    return float(value)


def _strict_event_evidence(
    events: Sequence[Mapping[str, object]],
    description: str,
) -> tuple[tuple[float, ...], tuple[int, ...]]:
    """在频率统计前统一校验外部事件的墙钟、时间戳和类型字段。"""
    wall_times: list[float] = []
    timestamps: list[int] = []
    for index, event in enumerate(events):
        event_description = f"{description}[{index}]"
        wall_times.append(
            _strict_finite_float(event, "wall_time", event_description)
        )
        timestamps.append(
            _strict_nonnegative_int(event, "timestamp_ns", event_description)
        )
        type_name = event.get("type")
        if not isinstance(type_name, str) or not type_name:
            raise AssertionError(
                f"{event_description} must report type as a nonempty string"
            )
    return tuple(wall_times), tuple(timestamps)


def _simulation_max_interarrival_gaps(
    evidence_by_topic: Mapping[
        str,
        tuple[Sequence[float], Sequence[int]],
    ],
    *,
    command_topic: str,
    command_wall_window: tuple[float, float],
    output_sim_window_ns: tuple[int, int],
) -> dict[str, float]:
    """命令按墙钟边界，输出以内段墙钟和两侧仿真时间共同计算空档。"""
    sim_start_ns, sim_end_ns = output_sim_window_ns
    if (
        isinstance(sim_start_ns, bool)
        or not isinstance(sim_start_ns, int)
        or isinstance(sim_end_ns, bool)
        or not isinstance(sim_end_ns, int)
        or sim_start_ns < 0
        or sim_end_ns <= sim_start_ns
    ):
        raise AssertionError("output simulation gap window must be increasing integers")
    gaps: dict[str, float] = {}
    for topic, (raw_wall_times, raw_timestamps) in evidence_by_topic.items():
        wall_times = tuple(raw_wall_times)
        timestamps = tuple(raw_timestamps)
        if not wall_times or len(wall_times) != len(timestamps):
            gaps[topic] = math.inf
            continue
        if topic == command_topic:
            gaps[topic] = _maximum_event_gap_sec(
                wall_times,
                *command_wall_window,
            )
            continue
        if (
            any(
                isinstance(value, bool) or not isinstance(value, int) or value < 0
                for value in timestamps
            )
            or any(
                current <= previous
                for previous, current in zip(timestamps, timestamps[1:])
            )
            or not sim_start_ns < timestamps[0] <= timestamps[-1] <= sim_end_ns
        ):
            gaps[topic] = math.inf
            continue
        interior_gap = (
            _maximum_event_gap_sec(wall_times, wall_times[0], wall_times[-1])
            if len(wall_times) >= 2
            else 0.0
        )
        leading_gap = (timestamps[0] - sim_start_ns) / 1_000_000_000.0
        trailing_gap = (sim_end_ns - timestamps[-1]) / 1_000_000_000.0
        gaps[topic] = max(interior_gap, leading_gap, trailing_gap)
    return gaps


def _resolve_child_evidence_path(
    root: Path,
    value: object,
    *,
    suffix: str,
) -> Path:
    """只接受证据目录内已存在的相对普通文件，拒绝路径和软链接逃逸。"""
    if not isinstance(value, str) or not value:
        raise AssertionError("evidence path must be a nonempty relative string")
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise AssertionError("evidence path must stay below its result directory")
    if not isinstance(suffix, str) or not suffix or not relative.name.endswith(suffix):
        raise AssertionError(f"evidence path must end with {suffix!r}")
    try:
        resolved_root = root.resolve(strict=True)
        resolved = (root / relative).resolve(strict=True)
        resolved.relative_to(resolved_root)
    except (FileNotFoundError, ValueError, OSError) as exc:
        raise AssertionError("evidence path escaped or does not exist") from exc
    if not resolved.is_file():
        raise AssertionError("evidence path must name a regular file")
    return resolved


def _prepare_evidence_directory(path: Path) -> Path:
    """创建或复用一个空的专用目录，绝不覆盖已有验收证据。"""
    if path.exists():
        if path.is_symlink() or not path.is_dir():
            raise ValueError("evidence_dir must be a real directory")
        if any(path.iterdir()):
            raise ValueError("evidence_dir must be empty")
    else:
        path.mkdir(parents=True)
    return path.resolve(strict=True)


def _strict_position_m(value: object, description: str) -> tuple[float, float, float]:
    """严格读取 JSON 三维坐标，排除 bool、字符串和非有限数。"""
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise AssertionError(f"{description} position_m must contain three numbers")
    if any(
        isinstance(item, bool)
        or not isinstance(item, (int, float))
        or not math.isfinite(item)
        for item in value
    ):
        raise AssertionError(f"{description} position_m must contain finite numbers")
    return tuple(float(item) for item in value)


def _horizontal_displacement(
    positions: Sequence[tuple[float, float, float]],
) -> float:
    if len(positions) < 2:
        return 0.0
    start = positions[0]
    end = positions[-1]
    return math.hypot(end[0] - start[0], end[1] - start[1])


def _strict_sha256(value: object, description: str) -> str:
    """严格读取小写十六进制 SHA-256，拒绝缺失或宽松字符串。"""
    if (
        not isinstance(value, str)
        or len(value) != 64
        or value.lower() != value
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise AssertionError(f"{description} payload_sha256 must be lowercase hex")
    return value


def _summarize_wheel_log_delivery(
    peer_events: Sequence[Mapping[str, object]],
    log_records: Iterable[InterfaceLogRecord],
    *,
    start_sim_time_ns: int,
    end_sim_time_ns: int,
    config: InterfaceConfig,
    codec: ProtoCodec,
) -> WheelLogDeliveryEvidence:
    """在 `(start, end]` 内双向精确核对 wheel 日志、类型和负载摘要。"""
    start = _strict_nonnegative_int(
        {"value": start_sim_time_ns},
        "value",
        "wheel delivery start",
    )
    end = _strict_nonnegative_int(
        {"value": end_sim_time_ns},
        "value",
        "wheel delivery end",
    )
    if start >= end:
        raise AssertionError("wheel delivery simulation window must be increasing")

    expected_type = codec.type_name(WheelState(0, (), ()))
    peer_by_timestamp: dict[int, tuple[str, str]] = {}
    for event in peer_events:
        timestamp_ns = _strict_nonnegative_int(
            event,
            "timestamp_ns",
            "peer wheel-state",
        )
        if not start < timestamp_ns <= end:
            raise AssertionError("peer wheel-state event is outside the simulation window")
        event_type = event.get("type")
        if event_type != expected_type:
            raise AssertionError("peer wheel-state event has the wrong protobuf type")
        if timestamp_ns in peer_by_timestamp:
            raise AssertionError("peer wheel-state events contain a duplicate timestamp")
        peer_by_timestamp[timestamp_ns] = (
            expected_type,
            _strict_sha256(event.get("payload_sha256"), "peer wheel-state"),
        )

    logged_by_timestamp: dict[int, tuple[str, str]] = {}
    for record in log_records:
        if record.topic != config.wheel_state.topic:
            continue
        if not start < record.sim_time_ns <= end:
            continue
        if record.direction != "publish":
            raise AssertionError("logged wheel-state record must be a publish record")
        if record.type_name != expected_type:
            raise AssertionError("logged wheel-state record has the wrong protobuf type")
        message = codec.decode_wheel_state(record.payload)
        if record.sim_time_ns != message.timestamp_ns:
            raise AssertionError(
                "logged wheel-state envelope and payload timestamps disagree"
            )
        if message.timestamp_ns in logged_by_timestamp:
            raise AssertionError("logged wheel-state records contain a duplicate timestamp")
        logged_by_timestamp[message.timestamp_ns] = (
            expected_type,
            hashlib.sha256(record.payload).hexdigest(),
        )

    missing_timestamps = logged_by_timestamp.keys() - peer_by_timestamp.keys()
    if missing_timestamps:
        raise AssertionError(
            "peer is missing logged wheel-state timestamps: "
            f"{sorted(missing_timestamps)[:5]}"
        )
    extra_timestamps = peer_by_timestamp.keys() - logged_by_timestamp.keys()
    if extra_timestamps:
        raise AssertionError(
            "unexpected peer wheel-state timestamps: "
            f"{sorted(extra_timestamps)[:5]}"
        )
    mismatched_timestamps = tuple(
        timestamp_ns
        for timestamp_ns in sorted(logged_by_timestamp)
        if logged_by_timestamp[timestamp_ns] != peer_by_timestamp[timestamp_ns]
    )
    if mismatched_timestamps:
        raise AssertionError(
            "wheel-state payload hash mismatch at timestamps: "
            f"{list(mismatched_timestamps[:5])}"
        )
    return WheelLogDeliveryEvidence(
        logged_count=len(logged_by_timestamp),
        peer_count=len(peer_by_timestamp),
        match_count=len(logged_by_timestamp),
    )


def _summarize_rtk_log_chain(
    peer_events: Sequence[Mapping[str, object]],
    log_records: Iterable[InterfaceLogRecord],
    *,
    config: InterfaceConfig,
    codec: ProtoCodec,
) -> RtkLogChainEvidence:
    """按仿真时间戳对齐 peer RTK 与原始 publish 日志，拒绝缺帧和重复帧。"""
    expected_type = codec.type_name(RtkState(0, 0.0, 0.0, 0.0, 0.0))
    peer_by_timestamp: dict[int, tuple[float, float, float]] = {}
    for event in peer_events:
        timestamp_ns = _strict_nonnegative_int(event, "timestamp_ns", "peer RTK")
        if event.get("type") != expected_type:
            raise AssertionError("peer RTK event has the wrong protobuf type")
        if timestamp_ns in peer_by_timestamp:
            raise AssertionError("peer RTK events contain a duplicate timestamp")
        peer_by_timestamp[timestamp_ns] = _strict_position_m(
            event.get("position_m"),
            "peer RTK",
        )

    logged_by_timestamp: dict[int, tuple[float, float, float]] = {}
    for record in log_records:
        if record.topic != config.rtk.topic:
            continue
        if record.direction != "publish":
            raise AssertionError("logged RTK record must be a publish record")
        if record.type_name != expected_type:
            raise AssertionError("logged RTK record has the wrong protobuf type")
        message = codec.decode_rtk_state(record.payload)
        if record.sim_time_ns != message.timestamp_ns:
            raise AssertionError("logged RTK envelope and payload timestamps disagree")
        if message.timestamp_ns in logged_by_timestamp:
            raise AssertionError("logged RTK records contain a duplicate timestamp")
        logged_by_timestamp[message.timestamp_ns] = (
            message.main_x,
            message.main_y,
            message.main_z,
        )

    missing_timestamps = peer_by_timestamp.keys() - logged_by_timestamp.keys()
    if missing_timestamps:
        raise AssertionError(
            "raw interface log is missing peer RTK timestamps: "
            f"{sorted(missing_timestamps)[:5]}"
        )
    matched_timestamps = sorted(peer_by_timestamp)
    peer_positions = [peer_by_timestamp[value] for value in matched_timestamps]
    logged_positions = [logged_by_timestamp[value] for value in matched_timestamps]
    position_errors = [
        math.dist(peer_position, logged_position)
        for peer_position, logged_position in zip(
            peer_positions,
            logged_positions,
            strict=True,
        )
    ]
    return RtkLogChainEvidence(
        peer_displacement_m=_horizontal_displacement(peer_positions),
        logged_displacement_m=_horizontal_displacement(logged_positions),
        match_count=len(matched_timestamps),
        max_position_error_m=max(position_errors, default=math.inf),
    )


def _assert_simulation_result(
    result: RoundtripResult,
    *,
    config: InterfaceConfig,
) -> None:
    """物理与安全场景字段必须全部由两个真实子进程给出肯定证据。"""
    expected_topics = {channel.topic for channel in config.channels}
    output_topics = {
        channel.topic for channel in config.channels if channel.direction == "publish"
    }
    if result.runtime_name != "simulation":
        raise AssertionError("runtime must be simulation")
    for field_name in (
        "feedback_is_not_command_echo",
        "invalid_command_rejected",
        "timeout_stopped_vehicle",
        "timeout_preserved_steering",
        "reconnect_required_new_command",
        "clean_shutdown",
    ):
        if getattr(result, field_name) is not True:
            raise AssertionError(f"simulation gate failed: {field_name}")
    if dict(result.output_disconnect_isolated) != {
        topic: True for topic in output_topics
    }:
        raise AssertionError("simulation output disconnect isolation failed")
    if dict(result.per_topic_peer_states) != {
        topic: "active" for topic in expected_topics
    }:
        raise AssertionError("simulation per-topic peer states are not all active")
    if dict(result.end_to_end_timestamp_match) != {
        topic: True for topic in expected_topics
    }:
        raise AssertionError("simulation end-to-end timestamp sequence mismatch")

    # 联合负载字段必须证明同一个真实 eCAL 窗口同时包含 20 障碍物和日志。
    integer_fields = {
        "mailbox_generation_before_disconnect": (
            result.mailbox_generation_before_disconnect
        ),
        "mailbox_generation_after_disconnect": (
            result.mailbox_generation_after_disconnect
        ),
        "normal_load_obstacle_count": result.normal_load_obstacle_count,
        "normal_load_log_sample_count": result.normal_load_log_sample_count,
        "normal_load_log_accepted_messages": result.normal_load_log_accepted_messages,
        "normal_load_log_accepted_events": result.normal_load_log_accepted_events,
        "normal_load_log_max_pending": result.normal_load_log_max_pending,
        "normal_load_log_final_pending": result.normal_load_log_final_pending,
        "normal_load_log_dropped_messages": result.normal_load_log_dropped_messages,
        "normal_load_log_dropped_events": result.normal_load_log_dropped_events,
        "normal_load_step_count": result.normal_load_step_count,
        "normal_load_control_step_count": result.normal_load_control_step_count,
        "normal_load_controlled_motion_step_count": (
            result.normal_load_controlled_motion_step_count
        ),
        "normal_load_obstacle_contact_step_count": (
            result.normal_load_obstacle_contact_step_count
        ),
        "normal_load_warmup_physics_steps": result.normal_load_warmup_physics_steps,
        "normal_load_warmup_log_accepted_messages": (
            result.normal_load_warmup_log_accepted_messages
        ),
        "normal_load_nonzero_drive_feedback_wheels": (
            result.normal_load_nonzero_drive_feedback_wheels
        ),
        "rtk_log_match_count": result.rtk_log_match_count,
        "normal_load_window_start_sim_time_ns": (
            result.normal_load_window_start_sim_time_ns
        ),
        "normal_load_window_end_sim_time_ns": (
            result.normal_load_window_end_sim_time_ns
        ),
        "wheel_drain_timestamp_ns": result.wheel_drain_timestamp_ns,
        "wheel_log_publish_count": result.wheel_log_publish_count,
        "wheel_peer_receive_count": result.wheel_peer_receive_count,
        "wheel_log_match_count": result.wheel_log_match_count,
    }
    if any(
        isinstance(value, bool) or not isinstance(value, int) or value < 0
        for value in integer_fields.values()
    ):
        raise AssertionError("simulation gate integer evidence is invalid")
    if result.reconnect_generation_advanced is not True:
        raise AssertionError("simulation reconnect generation did not advance")
    if (
        result.mailbox_generation_after_disconnect
        <= result.mailbox_generation_before_disconnect
    ):
        raise AssertionError("simulation mailbox generation did not advance")
    if result.normal_load_obstacle_count != 20:
        raise AssertionError("simulation normal load must contain exactly 20 obstacles")
    if result.wheel_drain_complete is not True:
        raise AssertionError("simulation wheel-state delivery fence is incomplete")
    if (
        result.normal_load_window_start_sim_time_ns
        >= result.normal_load_window_end_sim_time_ns
    ):
        raise AssertionError("simulation wheel delivery window did not advance")
    if (
        result.wheel_drain_timestamp_ns
        <= result.normal_load_window_end_sim_time_ns
    ):
        raise AssertionError("simulation wheel delivery fence did not cross window end")
    if not (
        result.wheel_log_publish_count
        == result.wheel_peer_receive_count
        == result.wheel_log_match_count
    ):
        raise AssertionError("simulation wheel log/peer delivery counts disagree")
    minimum_wheel_matches = math.floor(
        result.duration_sec * config.wheel_state.rate_hz * 0.90
    )
    if result.wheel_log_match_count < minimum_wheel_matches:
        raise AssertionError("simulation wheel log/peer delivery window is incomplete")
    minimum_log_samples = max(2, math.floor(result.duration_sec * 10.0 * 0.90))
    if result.normal_load_log_sample_count < minimum_log_samples:
        raise AssertionError("simulation normal-load log sampling window is incomplete")
    minimum_accepted_messages = math.floor(
        result.duration_sec
        * sum(channel.rate_hz for channel in config.channels)
        * 0.90
    )
    if result.normal_load_log_accepted_messages < minimum_accepted_messages:
        raise AssertionError("simulation normal-load logger accepted too few messages")
    if result.normal_load_log_final_pending != 0:
        raise AssertionError("simulation normal-load logger did not become idle")
    if result.normal_load_log_sustained_backlog is not False:
        raise AssertionError("simulation normal-load logger has sustained backlog")
    if (
        result.normal_load_log_dropped_messages != 0
        or result.normal_load_log_dropped_events != 0
    ):
        raise AssertionError("simulation normal-load logger dropped records")
    if result.normal_load_log_writer_failed is not False:
        raise AssertionError("simulation normal-load logger writer failed")
    if result.normal_load_log_sequence_contiguous is not True:
        raise AssertionError("simulation normal-load binary sequence is not contiguous")

    warmup = result.normal_load_warmup_requested_sec
    warmup_wall = result.normal_load_warmup_wall_duration_sec
    warmup_sim = result.normal_load_warmup_sim_duration_sec
    if not math.isfinite(warmup) or warmup <= 0.0:
        raise AssertionError("simulation production warmup request is invalid")
    if not math.isfinite(warmup_wall) or warmup_wall < warmup:
        raise AssertionError("simulation production warmup wall window is incomplete")
    if (
        not math.isfinite(warmup_sim)
        or warmup_sim < warmup * _MIN_WARMUP_SIM_WALL_RATIO
    ):
        raise AssertionError("simulation production warmup did not advance sim time")
    minimum_warmup_steps = math.floor(
        warmup * 240.0 * _MIN_WARMUP_SIM_WALL_RATIO
    )
    if result.normal_load_warmup_physics_steps < minimum_warmup_steps:
        raise AssertionError("simulation production warmup advanced too few physics steps")
    minimum_warmup_messages = math.floor(
        warmup
        * sum(channel.rate_hz for channel in config.channels)
        * _MIN_WINDOW_COVERAGE_RATIO
    )
    if result.normal_load_warmup_log_accepted_messages < minimum_warmup_messages:
        raise AssertionError("simulation production warmup logged too few messages")
    warmup_counts = dict(result.normal_load_warmup_topic_counts)
    if set(warmup_counts) != expected_topics or any(
        isinstance(count, bool) or not isinstance(count, int) or count <= 0
        for count in warmup_counts.values()
    ):
        raise AssertionError("simulation production warmup did not exercise all topics")
    if tuple(result.normal_load_command_states) != ("active",):
        raise AssertionError("simulation normal-load command entered a non-active state")

    # 双时钟和步数证据必须共同证明 240 Hz 物理时钟与墙钟同速。
    float_fields = {
        "normal_load_requested_duration_sec": (
            result.normal_load_requested_duration_sec
        ),
        "peer_measurement_duration_sec": result.peer_measurement_duration_sec,
        "normal_load_physics_time_step_sec": (
            result.normal_load_physics_time_step_sec
        ),
        "normal_load_sim_duration_sec": result.normal_load_sim_duration_sec,
        "normal_load_wall_duration_sec": result.normal_load_wall_duration_sec,
        "normal_load_sim_wall_ratio": result.normal_load_sim_wall_ratio,
        "normal_load_control_sim_duration_sec": (
            result.normal_load_control_sim_duration_sec
        ),
        "normal_load_control_wall_duration_sec": (
            result.normal_load_control_wall_duration_sec
        ),
        "normal_load_controlled_motion_sim_duration_sec": (
            result.normal_load_controlled_motion_sim_duration_sec
        ),
        "normal_load_controlled_displacement_m": (
            result.normal_load_controlled_displacement_m
        ),
        "normal_load_controlled_path_length_m": (
            result.normal_load_controlled_path_length_m
        ),
        "normal_load_controlled_mean_speed_m_s": (
            result.normal_load_controlled_mean_speed_m_s
        ),
        "normal_load_controlled_max_speed_m_s": (
            result.normal_load_controlled_max_speed_m_s
        ),
        "normal_load_base_displacement_m": result.normal_load_base_displacement_m,
        "normal_load_base_path_length_m": result.normal_load_base_path_length_m,
        "normal_load_base_mean_speed_m_s": result.normal_load_base_mean_speed_m_s,
        "normal_load_base_max_speed_m_s": result.normal_load_base_max_speed_m_s,
        "peer_rtk_displacement_m": result.peer_rtk_displacement_m,
        "logged_rtk_displacement_m": result.logged_rtk_displacement_m,
        "rtk_log_max_position_error_m": result.rtk_log_max_position_error_m,
    }
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0.0
        for value in float_fields.values()
    ):
        raise AssertionError("simulation gate floating-point evidence is invalid")
    if result.normal_load_step_count <= 0:
        raise AssertionError("simulation normal-load physics step count is empty")
    time_step_sec = result.normal_load_physics_time_step_sec
    if time_step_sec <= 0.0:
        raise AssertionError("simulation physics time step must be positive")
    if not math.isclose(
        result.normal_load_requested_duration_sec,
        result.duration_sec,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise AssertionError("simulation runtime measurement request does not match")
    if not math.isclose(
        result.peer_measurement_duration_sec,
        result.duration_sec,
        rel_tol=1.0e-12,
        abs_tol=1.0e-9,
    ):
        raise AssertionError("simulation peer measurement duration does not match")

    expected_sim_duration = result.normal_load_step_count * time_step_sec
    bounded_sim_duration = (
        result.normal_load_window_end_sim_time_ns
        - result.normal_load_window_start_sim_time_ns
    ) / 1.0e9
    if not math.isclose(
        bounded_sim_duration,
        result.normal_load_sim_duration_sec,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise AssertionError("simulation window bounds disagree with runtime duration")
    if not math.isclose(
        result.normal_load_sim_duration_sec,
        expected_sim_duration,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise AssertionError("simulation duration does not match physics step count")
    if result.normal_load_wall_duration_sec <= 0.0:
        raise AssertionError("simulation normal-load wall duration is empty")
    recomputed_ratio = (
        result.normal_load_sim_duration_sec / result.normal_load_wall_duration_sec
    )
    if not math.isclose(
        result.normal_load_sim_wall_ratio,
        recomputed_ratio,
        rel_tol=1.0e-9,
        abs_tol=1.0e-9,
    ):
        raise AssertionError("simulation reported sim/wall ratio is inconsistent")
    if not _MIN_SIM_WALL_RATIO <= recomputed_ratio <= _MAX_SIM_WALL_RATIO:
        raise AssertionError("simulation clock did not track wall clock in real time")
    window_tolerance_sec = max(
        4.0 * time_step_sec,
        min(0.05, result.duration_sec * 0.02),
    )
    if (
        result.normal_load_sim_duration_sec
        > result.duration_sec + window_tolerance_sec
    ):
        raise AssertionError("simulation measurement window is too long")
    if (
        result.normal_load_sim_duration_sec
        < result.duration_sec - window_tolerance_sec
    ):
        raise AssertionError("simulation normal-load duration is incomplete")
    if (
        abs(result.normal_load_wall_duration_sec - result.duration_sec)
        > window_tolerance_sec
    ):
        raise AssertionError("simulation wall-clock measurement window does not match")

    nominal_step_count = result.duration_sec / time_step_sec
    minimum_control_steps = math.ceil(nominal_step_count * 0.90)
    if result.normal_load_control_step_count < minimum_control_steps:
        raise AssertionError("simulation active-control step window is incomplete")
    if result.normal_load_control_step_count > result.normal_load_step_count:
        raise AssertionError("simulation active-control steps exceed physics steps")
    expected_control_duration = result.normal_load_control_step_count * time_step_sec
    if not math.isclose(
        result.normal_load_control_sim_duration_sec,
        expected_control_duration,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise AssertionError("active-control duration does not match its step count")
    if result.normal_load_control_wall_duration_sec < result.duration_sec * 0.90:
        raise AssertionError("simulation wall-clock control window is incomplete")
    if (
        result.normal_load_control_wall_duration_sec
        > result.normal_load_wall_duration_sec + 1.0e-9
    ):
        raise AssertionError("simulation control wall time exceeds measurement window")

    minimum_controlled_steps = math.ceil(nominal_step_count * 0.85)
    if result.normal_load_controlled_motion_step_count < minimum_controlled_steps:
        raise AssertionError("simulation controlled-motion step window is incomplete")
    if (
        result.normal_load_controlled_motion_step_count
        > result.normal_load_control_step_count
    ):
        raise AssertionError("controlled-motion steps exceed active-control steps")
    expected_controlled_duration = (
        result.normal_load_controlled_motion_step_count * time_step_sec
    )
    if not math.isclose(
        result.normal_load_controlled_motion_sim_duration_sec,
        expected_controlled_duration,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise AssertionError(
            "controlled-motion duration does not match its step count"
        )
    if result.normal_load_obstacle_contact_step_count != 0:
        raise AssertionError("simulation vehicle contacted an obstacle during measurement")
    if result.normal_load_controlled_displacement_m < 0.25:
        raise AssertionError("simulation controlled displacement is too small")
    if (
        result.normal_load_controlled_path_length_m + 1.0e-9
        < result.normal_load_controlled_displacement_m
    ):
        raise AssertionError("controlled path is shorter than controlled displacement")
    expected_controlled_mean_speed = (
        result.normal_load_controlled_path_length_m
        / result.normal_load_controlled_motion_sim_duration_sec
    )
    if not math.isclose(
        result.normal_load_controlled_mean_speed_m_s,
        expected_controlled_mean_speed,
        rel_tol=1.0e-9,
        abs_tol=1.0e-9,
    ):
        raise AssertionError("controlled-motion mean speed is inconsistent")
    if result.normal_load_controlled_mean_speed_m_s < 0.05:
        raise AssertionError("simulation controlled mean speed is too small")
    if result.normal_load_controlled_max_speed_m_s < 0.10:
        raise AssertionError("simulation controlled maximum speed is too small")

    # 底盘整窗、peer RTK 与原始日志共同排除命令回显假阳性。
    if result.normal_load_base_displacement_m < 0.25:
        raise AssertionError("simulation base displacement is too small")
    if (
        result.normal_load_base_path_length_m + 1.0e-9
        < result.normal_load_base_displacement_m
    ):
        raise AssertionError("simulation base path is shorter than displacement")
    expected_mean_speed = (
        result.normal_load_base_path_length_m / result.normal_load_sim_duration_sec
    )
    if not math.isclose(
        result.normal_load_base_mean_speed_m_s,
        expected_mean_speed,
        rel_tol=1.0e-9,
        abs_tol=1.0e-9,
    ):
        raise AssertionError("simulation reported mean speed is inconsistent")
    if result.normal_load_base_mean_speed_m_s < 0.05:
        raise AssertionError("simulation base mean speed is too small")
    if result.normal_load_base_max_speed_m_s < 0.10:
        raise AssertionError("simulation base maximum speed is too small")
    if result.peer_rtk_displacement_m < 0.25:
        raise AssertionError("peer RTK displacement is too small")
    if result.logged_rtk_displacement_m < 0.25:
        raise AssertionError("logged RTK displacement is too small")
    minimum_rtk_matches = math.floor(result.duration_sec * config.rtk.rate_hz * 0.85)
    if result.rtk_log_match_count < minimum_rtk_matches:
        raise AssertionError("RTK peer/log timestamp matches are incomplete")
    if result.rtk_log_max_position_error_m > 1.0e-6:
        raise AssertionError("RTK peer and raw interface log positions disagree")

    measurement_wall = result.normal_load_measurement_wall_duration_sec
    if not math.isfinite(measurement_wall) or not math.isclose(
        measurement_wall,
        result.duration_sec,
        rel_tol=0.0,
        abs_tol=1e-6,
    ):
        raise AssertionError("simulation measurement wall window is not exact")
    ratio = result.normal_load_sim_wall_ratio
    if not math.isfinite(ratio) or not (
        _MIN_SIM_WALL_RATIO <= ratio <= _MAX_SIM_WALL_RATIO
    ):
        raise AssertionError("simulation sim/wall ratio is outside 0.98..1.02")
    control_duration = result.normal_load_control_duration_sec
    if not math.isfinite(control_duration) or not (
        result.duration_sec - 0.02 <= control_duration <= result.duration_sec
    ):
        raise AssertionError("simulation control window is incomplete")

    motion_fields = {
        "rtk displacement": result.normal_load_rtk_displacement_m,
        "base displacement": result.normal_load_base_displacement_m,
        "trajectory distance": result.normal_load_trajectory_distance_m,
    }
    if any(
        not math.isfinite(value) or value < _MIN_MOTION_DISPLACEMENT_M
        for value in motion_fields.values()
    ):
        raise AssertionError("simulation motion evidence is incomplete")
    if (
        not math.isfinite(result.normal_load_average_speed_m_s)
        or result.normal_load_average_speed_m_s < _MIN_AVERAGE_SPEED_M_S
    ):
        raise AssertionError("simulation average speed evidence is incomplete")
    robot_model = getattr(result, "robot_model", "active_steering_4wd")
    try:
        model = get_robot_model(robot_model)
    except ValueError as exc:
        raise AssertionError("simulation robot model evidence is invalid") from exc
    expected_drive_wheels = len(model.drive_joint_names)
    if result.normal_load_nonzero_drive_feedback_wheels != expected_drive_wheels:
        raise AssertionError("simulation drive feedback did not move every wheel")
    steering_peaks = (
        result.normal_load_peak_left_steering_angle_rad,
        result.normal_load_peak_right_steering_angle_rad,
    )
    if model.steering_joint_names:
        if any(
            not math.isfinite(value)
            or value < _MIN_STEERING_ANGLE_RAD
            or value > model.max_steering_angle + 0.02
            for value in steering_peaks
        ):
            raise AssertionError(
                "simulation steering feedback is incomplete or out of limit"
            )
        if result.normal_load_steering_same_sign is not True:
            raise AssertionError("simulation front steering feedback did not turn together")
    elif (
        any(not math.isfinite(value) or abs(value) > 1.0e-9 for value in steering_peaks)
        or result.normal_load_steering_same_sign is not False
    ):
        raise AssertionError("differential simulation unexpectedly reported steering joints")
    if result.normal_load_peak_steering_angle_rad != max(steering_peaks):
        raise AssertionError("simulation peak steering evidence is inconsistent")


def _run_ecal_simulation_roundtrip(
    *,
    duration_sec: float,
    warmup_sec: float,
    process_timeout_sec: float,
    evidence_dir: Path | None,
    robot_model: str,
) -> RoundtripResult:
    """编排真实 PyBullet runtime 与官方 eCAL peer，并合并双方证据。"""
    selected_robot_model = get_robot_model(robot_model).name
    token = uuid4().hex[:10]
    config = InterfaceConfig.default(transport_mode="ecal")
    codec = ProtoCodec()
    command_topic, _wheel_state_topic, _sensor_topics, output_topics = (
        _configured_topics(config)
    )
    expected_types = _expected_topic_types(config, codec)
    scenario_budget_sec = _simulation_scenario_budget_sec(
        duration_sec=duration_sec,
        warmup_sec=warmup_sec,
        startup_timeout_sec=process_timeout_sec,
    )
    runtime_process: subprocess.Popen[str] | None = None
    peer_process: subprocess.Popen[str] | None = None

    evidence_context = (
        tempfile.TemporaryDirectory(prefix="slope-ecal-simulation-")
        if evidence_dir is None
        else nullcontext(_prepare_evidence_directory(evidence_dir))
    )
    with evidence_context as temp_dir:
        temp_path = Path(temp_dir)
        scenario_path = temp_path / "scenario"
        start_path = temp_path / "start.signal"
        stop_path = temp_path / "stop.signal"
        runtime_result_path = temp_path / "runtime-result.json"
        peer_result_path = temp_path / "peer-result.json"
        runtime_ready_path = temp_path / "runtime.ready"
        peer_ready_path = temp_path / "peer.ready"
        try:
            runtime_process = _start_simulation_runtime(
                result_json=runtime_result_path,
                scenario_dir=scenario_path,
                ready_file=runtime_ready_path,
                start_file=start_path,
                stop_file=stop_path,
                participant_name=f"slope-sim-runtime-{token}",
                max_runtime_sec=scenario_budget_sec,
                robot_model=selected_robot_model,
            )
            _wait_until(
                runtime_ready_path.exists,
                timeout_sec=process_timeout_sec,
                description="simulation runtime readiness",
                process=runtime_process,
                process_name="simulation runtime",
            )
            peer_process = _start_simulation_peer(
                result_json=peer_result_path,
                scenario_dir=scenario_path,
                ready_file=peer_ready_path,
                start_file=start_path,
                duration_sec=duration_sec,
                warmup_sec=warmup_sec,
                participant_name=f"slope-sim-physics-peer-{token}",
                start_timeout_sec=process_timeout_sec,
                robot_model=selected_robot_model,
            )
            _wait_until(
                peer_ready_path.exists,
                timeout_sec=process_timeout_sec,
                description="simulation peer readiness",
                process=peer_process,
                process_name="simulation peer",
            )
            start_path.write_text("start\n", encoding="utf-8")
            try:
                peer_returncode, _peer_stdout, _peer_stderr = _finish_peer(
                    peer_process,
                    timeout_sec=scenario_budget_sec,
                    process_name="simulation peer",
                )
            except BaseException as peer_exc:
                if runtime_process.poll() is not None:
                    runtime_stdout, runtime_stderr = runtime_process.communicate()
                    raise RuntimeError(
                        "simulation runtime exited before peer protocol completed: "
                        f"code={runtime_process.returncode} "
                        f"stdout={runtime_stdout.strip()} "
                        f"stderr={runtime_stderr.strip()}"
                    ) from peer_exc
                raise
            stop_path.write_text("stop\n", encoding="utf-8")
            _finish_peer(
                runtime_process,
                timeout_sec=process_timeout_sec,
                process_name="simulation runtime",
            )
            peer_result = _load_json_object(peer_result_path, "simulation peer")
            runtime_result = _load_json_object(runtime_result_path, "simulation runtime")
        finally:
            stop_path.write_text("stop\n", encoding="utf-8")
            _reap_process(peer_process)
            _reap_process(runtime_process)

        # 原始日志必须在临时目录释放前完成解析和端到端交叉校验。
        interface_log_files = runtime_result.get("interface_log_files")
        if not isinstance(interface_log_files, dict):
            raise AssertionError(
                "simulation runtime interface_log_files must be an object"
            )
        binary_log_path = _resolve_child_evidence_path(
            temp_path,
            interface_log_files.get("binary"),
            suffix=".interfaces.bin",
        )
        peer_received = peer_result.get("received")
        if not isinstance(peer_received, dict):
            raise AssertionError("simulation peer received field must be an object")
        window_start_sim_time_ns = _strict_nonnegative_int(
            peer_result,
            "window_start_sim_time_ns",
            "simulation peer",
        )
        window_end_sim_time_ns = _strict_nonnegative_int(
            peer_result,
            "window_end_sim_time_ns",
            "simulation peer",
        )
        runtime_window_start = _strict_nonnegative_int(
            runtime_result,
            "normal_load_window_start_sim_time_ns",
            "simulation runtime",
        )
        runtime_window_end = _strict_nonnegative_int(
            runtime_result,
            "normal_load_window_end_sim_time_ns",
            "simulation runtime",
        )
        if (
            window_start_sim_time_ns != runtime_window_start
            or window_end_sim_time_ns != runtime_window_end
        ):
            raise AssertionError("simulation peer/runtime window bounds disagree")
        if window_start_sim_time_ns >= window_end_sim_time_ns:
            raise AssertionError("simulation window bounds did not advance")
        wheel_drain_complete = _strict_true(
            peer_result,
            "wheel_drain_complete",
            "simulation peer",
        )
        wheel_drain_timestamp_ns = _strict_nonnegative_int(
            peer_result,
            "wheel_drain_timestamp_ns",
            "simulation peer",
        )
        if wheel_drain_timestamp_ns <= window_end_sim_time_ns:
            raise AssertionError("wheel-state delivery fence did not cross window end")

        log_records = tuple(iter_interface_log(binary_log_path))
        wheel_log_evidence = _summarize_wheel_log_delivery(
            _event_list(
                peer_received.get(config.wheel_state.topic),
                f"received events for {config.wheel_state.topic}",
            ),
            log_records,
            start_sim_time_ns=window_start_sim_time_ns,
            end_sim_time_ns=window_end_sim_time_ns,
            config=config,
            codec=codec,
        )
        rtk_log_evidence = _summarize_rtk_log_chain(
            _event_list(
                peer_received.get(config.rtk.topic),
                f"received events for {config.rtk.topic}",
            ),
            log_records,
            config=config,
            codec=codec,
        )

    for description, payload in (
        ("simulation peer", peer_result),
        ("simulation runtime", runtime_result),
    ):
        if payload.get("transport") != "ecal":
            raise AssertionError(f"{description} did not report transport=ecal")
        if payload.get("runtime") != "simulation":
            raise AssertionError(f"{description} did not report runtime=simulation")
        if payload.get("robot_model") != selected_robot_model:
            raise AssertionError(
                f"{description} robot_model does not match the requested model"
            )
        _strict_true(payload, "clean_shutdown", description)

    peer_snapshot = peer_result.get("snapshot")
    runtime_snapshot = runtime_result.get("transport_snapshot")
    if not isinstance(peer_snapshot, dict):
        raise AssertionError("simulation peer snapshot must be an object")
    if not isinstance(runtime_snapshot, dict):
        raise AssertionError("simulation runtime transport_snapshot must be an object")

    final_peer_states = _strict_bool_mapping(
        peer_result,
        "final_peer_states",
        "simulation peer",
    )
    if final_peer_states != {
        topic: True for topic in expected_types
    }:
        raise AssertionError("simulation peer final states are not all connected")
    peer_requested_duration = _strict_nonnegative_float(
        peer_result,
        "requested_duration_sec",
        "simulation peer",
    )
    if not math.isclose(
        peer_requested_duration,
        duration_sec,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise AssertionError("simulation peer requested duration does not match")
    peer_measurement_duration = _strict_nonnegative_float(
        peer_result,
        "peer_measurement_duration_sec",
        "simulation peer",
    )

    measurement_start = _strict_finite_float(
        peer_result,
        "measurement_start",
        "simulation peer",
    )
    measurement_end = _strict_finite_float(
        peer_result,
        "measurement_end",
        "simulation peer",
    )
    if measurement_end <= measurement_start:
        raise AssertionError("simulation peer measurement window must be increasing")
    events_by_topic = _simulation_measurement_events(
        peer_result,
        runtime_result,
        command_topic=command_topic,
        output_topics=output_topics,
    )
    event_evidence = {
        topic: _strict_event_evidence(events, f"measurement events for {topic}")
        for topic, events in events_by_topic.items()
    }
    peer_commands = _event_list(peer_result.get("commands"), "peer command events")
    _validate_simulation_command_cardinality(
        peer_commands,
        robot_model=selected_robot_model,
    )
    _peer_command_walls, peer_command_timestamps = _strict_event_evidence(
        peer_commands,
        "peer command events",
    )
    runtime_published = runtime_result.get("normal_load_published")
    peer_received = peer_result.get("received")
    if not isinstance(runtime_published, dict):
        raise AssertionError("simulation runtime normal_load_published must be an object")
    if not isinstance(peer_received, dict):
        raise AssertionError("simulation peer received field must be an object")
    end_to_end_match = {
        command_topic: _timestamp_sequences_match(
            peer_command_timestamps,
            event_evidence[command_topic][1],
        )
    }
    for topic in output_topics:
        produced = _event_list(
            runtime_published.get(topic),
            f"runtime published events for {topic}",
        )
        consumed = _event_list(
            peer_received.get(topic),
            f"peer received events for {topic}",
        )
        _produced_walls, produced_timestamps = _strict_event_evidence(
            produced,
            f"runtime published events for {topic}",
        )
        _consumed_walls, consumed_timestamps = _strict_event_evidence(
            consumed,
            f"peer received events for {topic}",
        )
        end_to_end_match[topic] = _timestamp_sequences_match(
            produced_timestamps,
            consumed_timestamps,
        )

    wall_clock_hz = {
        topic: _frequency_hz(wall_times)
        for topic, (wall_times, _timestamps) in event_evidence.items()
    }
    timestamp_hz = {
        topic: _timestamp_frequency_hz(timestamps)
        for topic, (_wall_times, timestamps) in event_evidence.items()
    }
    message_counts = {topic: len(events) for topic, events in events_by_topic.items()}
    event_spans = {
        topic: _event_span_sec(wall_times)
        for topic, (wall_times, _timestamps) in event_evidence.items()
    }
    max_gaps = _simulation_max_interarrival_gaps(
        event_evidence,
        command_topic=command_topic,
        command_wall_window=(measurement_start, measurement_end),
        output_sim_window_ns=(
            window_start_sim_time_ns,
            window_end_sim_time_ns,
        ),
    )
    output_isolation = _strict_bool_mapping(
        runtime_result,
        "output_disconnect_isolated",
        "simulation runtime",
    )
    topic_states = _strict_string_mapping(
        runtime_result,
        "per_topic_peer_states",
        "simulation runtime",
    )

    result = RoundtripResult(
        transport_name="ecal",
        peer_returncode=peer_returncode,
        wall_clock_hz=wall_clock_hz,
        message_timestamp_hz=timestamp_hz,
        received_topics={
            topic for topic in output_topics if message_counts.get(topic, 0) > 0
        },
        topic_types={
            topic: _single_type(topic, events)
            for topic, events in events_by_topic.items()
        },
        message_counts=message_counts,
        dropped_count=_strict_nonnegative_int(
            runtime_snapshot,
            "dropped_count",
            "simulation runtime transport snapshot",
        ),
        duration_sec=duration_sec,
        event_span_sec=event_spans,
        max_interarrival_gap_sec=max_gaps,
        end_to_end_timestamp_match=end_to_end_match,
        peer_dropped_count=_strict_nonnegative_int(
            peer_snapshot,
            "dropped_count",
            "simulation peer snapshot",
        ),
        transport_error_count=_strict_nonnegative_int(
            runtime_snapshot,
            "error_count",
            "simulation runtime transport snapshot",
        ),
        peer_error_count=_strict_nonnegative_int(
            peer_snapshot,
            "error_count",
            "simulation peer snapshot",
        ),
        runtime_name="simulation",
        robot_model=selected_robot_model,
        feedback_is_not_command_echo=_strict_true(
            runtime_result, "feedback_is_not_command_echo", "simulation runtime"
        ),
        invalid_command_rejected=_strict_true(
            runtime_result, "invalid_command_rejected", "simulation runtime"
        ),
        timeout_stopped_vehicle=_strict_true(
            runtime_result, "timeout_stopped_vehicle", "simulation runtime"
        ),
        timeout_preserved_steering=_strict_true(
            runtime_result, "timeout_preserved_steering", "simulation runtime"
        ),
        output_disconnect_isolated=output_isolation,
        per_topic_peer_states=topic_states,
        reconnect_required_new_command=_strict_true(
            runtime_result, "reconnect_required_new_command", "simulation runtime"
        ),
        reconnect_generation_advanced=_strict_true(
            runtime_result, "reconnect_generation_advanced", "simulation runtime"
        ),
        mailbox_generation_before_disconnect=_strict_nonnegative_int(
            runtime_result,
            "mailbox_generation_before_disconnect",
            "simulation runtime",
        ),
        mailbox_generation_after_disconnect=_strict_nonnegative_int(
            runtime_result,
            "mailbox_generation_after_disconnect",
            "simulation runtime",
        ),
        normal_load_obstacle_count=_strict_nonnegative_int(
            runtime_result,
            "normal_load_obstacle_count",
            "simulation runtime",
        ),
        normal_load_log_sample_count=_strict_nonnegative_int(
            runtime_result,
            "normal_load_log_sample_count",
            "simulation runtime",
        ),
        normal_load_log_accepted_messages=_strict_nonnegative_int(
            runtime_result,
            "normal_load_log_accepted_messages",
            "simulation runtime",
        ),
        normal_load_log_accepted_events=_strict_nonnegative_int(
            runtime_result,
            "normal_load_log_accepted_events",
            "simulation runtime",
        ),
        normal_load_log_max_pending=_strict_nonnegative_int(
            runtime_result,
            "normal_load_log_max_pending",
            "simulation runtime",
        ),
        normal_load_log_final_pending=_strict_nonnegative_int(
            runtime_result,
            "normal_load_log_final_pending",
            "simulation runtime",
        ),
        normal_load_log_sustained_backlog=_strict_false(
            runtime_result,
            "normal_load_log_sustained_backlog",
            "simulation runtime "
            f"samples={runtime_result.get('normal_load_log_samples')!r}",
        ),
        normal_load_log_dropped_messages=_strict_nonnegative_int(
            runtime_result,
            "normal_load_log_dropped_messages",
            "simulation runtime",
        ),
        normal_load_log_dropped_events=_strict_nonnegative_int(
            runtime_result,
            "normal_load_log_dropped_events",
            "simulation runtime",
        ),
        normal_load_log_writer_failed=_strict_false(
            runtime_result,
            "normal_load_log_writer_failed",
            "simulation runtime",
        ),
        normal_load_log_sequence_contiguous=_strict_true(
            runtime_result,
            "normal_load_log_sequence_contiguous",
            "simulation runtime",
        ),
        normal_load_requested_duration_sec=_strict_nonnegative_float(
            runtime_result,
            "normal_load_requested_duration_sec",
            "simulation runtime",
        ),
        peer_measurement_duration_sec=peer_measurement_duration,
        normal_load_physics_time_step_sec=_strict_nonnegative_float(
            runtime_result,
            "normal_load_physics_time_step_sec",
            "simulation runtime",
        ),
        normal_load_step_count=_strict_nonnegative_int(
            runtime_result,
            "normal_load_step_count",
            "simulation runtime",
        ),
        normal_load_sim_duration_sec=_strict_nonnegative_float(
            runtime_result,
            "normal_load_sim_duration_sec",
            "simulation runtime",
        ),
        normal_load_wall_duration_sec=_strict_nonnegative_float(
            runtime_result,
            "normal_load_wall_duration_sec",
            "simulation runtime",
        ),
        normal_load_control_step_count=_strict_nonnegative_int(
            runtime_result,
            "normal_load_control_step_count",
            "simulation runtime",
        ),
        normal_load_control_sim_duration_sec=_strict_nonnegative_float(
            runtime_result,
            "normal_load_control_sim_duration_sec",
            "simulation runtime",
        ),
        normal_load_control_wall_duration_sec=_strict_nonnegative_float(
            runtime_result,
            "normal_load_control_wall_duration_sec",
            "simulation runtime",
        ),
        normal_load_controlled_motion_step_count=_strict_nonnegative_int(
            runtime_result,
            "normal_load_controlled_motion_step_count",
            "simulation runtime",
        ),
        normal_load_controlled_motion_sim_duration_sec=_strict_nonnegative_float(
            runtime_result,
            "normal_load_controlled_motion_sim_duration_sec",
            "simulation runtime",
        ),
        normal_load_obstacle_contact_step_count=_strict_nonnegative_int(
            runtime_result,
            "normal_load_obstacle_contact_step_count",
            "simulation runtime",
        ),
        normal_load_controlled_displacement_m=_strict_nonnegative_float(
            runtime_result,
            "normal_load_controlled_displacement_m",
            "simulation runtime",
        ),
        normal_load_controlled_path_length_m=_strict_nonnegative_float(
            runtime_result,
            "normal_load_controlled_path_length_m",
            "simulation runtime",
        ),
        normal_load_controlled_mean_speed_m_s=_strict_nonnegative_float(
            runtime_result,
            "normal_load_controlled_mean_speed_m_s",
            "simulation runtime",
        ),
        normal_load_controlled_max_speed_m_s=_strict_nonnegative_float(
            runtime_result,
            "normal_load_controlled_max_speed_m_s",
            "simulation runtime",
        ),
        normal_load_warmup_requested_sec=_strict_finite_float(
            runtime_result,
            "normal_load_warmup_requested_sec",
            "simulation runtime",
        ),
        normal_load_warmup_wall_duration_sec=_strict_finite_float(
            runtime_result,
            "normal_load_warmup_wall_duration_sec",
            "simulation runtime",
        ),
        normal_load_warmup_sim_duration_sec=_strict_finite_float(
            runtime_result,
            "normal_load_warmup_sim_duration_sec",
            "simulation runtime",
        ),
        normal_load_warmup_physics_steps=_strict_nonnegative_int(
            runtime_result,
            "normal_load_warmup_physics_steps",
            "simulation runtime",
        ),
        normal_load_warmup_log_accepted_messages=_strict_nonnegative_int(
            runtime_result,
            "normal_load_warmup_log_accepted_messages",
            "simulation runtime",
        ),
        normal_load_warmup_topic_counts=_strict_count_mapping(
            runtime_result,
            "normal_load_warmup_topic_counts",
            "simulation runtime",
        ),
        normal_load_command_states=_strict_string_tuple(
            runtime_result,
            "normal_load_command_states",
            "simulation runtime",
        ),
        normal_load_measurement_wall_duration_sec=_strict_finite_float(
            runtime_result,
            "normal_load_measurement_wall_duration_sec",
            "simulation runtime",
        ),
        normal_load_sim_wall_ratio=_strict_nonnegative_float(
            runtime_result,
            "normal_load_sim_wall_ratio",
            "simulation runtime",
        ),
        normal_load_control_duration_sec=_strict_finite_float(
            runtime_result,
            "normal_load_control_duration_sec",
            "simulation runtime",
        ),
        normal_load_rtk_displacement_m=_strict_finite_float(
            runtime_result,
            "normal_load_rtk_displacement_m",
            "simulation runtime",
        ),
        normal_load_base_displacement_m=_strict_nonnegative_float(
            runtime_result,
            "normal_load_base_displacement_m",
            "simulation runtime",
        ),
        normal_load_base_path_length_m=_strict_nonnegative_float(
            runtime_result,
            "normal_load_base_path_length_m",
            "simulation runtime",
        ),
        normal_load_base_mean_speed_m_s=_strict_nonnegative_float(
            runtime_result,
            "normal_load_base_mean_speed_m_s",
            "simulation runtime",
        ),
        normal_load_base_max_speed_m_s=_strict_nonnegative_float(
            runtime_result,
            "normal_load_base_max_speed_m_s",
            "simulation runtime",
        ),
        normal_load_trajectory_distance_m=_strict_finite_float(
            runtime_result,
            "normal_load_trajectory_distance_m",
            "simulation runtime",
        ),
        normal_load_average_speed_m_s=_strict_finite_float(
            runtime_result,
            "normal_load_average_speed_m_s",
            "simulation runtime",
        ),
        normal_load_nonzero_drive_feedback_wheels=_strict_nonnegative_int(
            runtime_result,
            "normal_load_nonzero_drive_feedback_wheels",
            "simulation runtime",
        ),
        normal_load_peak_left_steering_angle_rad=_strict_finite_float(
            runtime_result,
            "normal_load_peak_left_steering_angle_rad",
            "simulation runtime",
        ),
        normal_load_peak_right_steering_angle_rad=_strict_finite_float(
            runtime_result,
            "normal_load_peak_right_steering_angle_rad",
            "simulation runtime",
        ),
        normal_load_steering_same_sign=_strict_model_steering_same_sign(
            runtime_result,
            robot_model=selected_robot_model,
            description="simulation runtime",
        ),
        normal_load_peak_steering_angle_rad=_strict_finite_float(
            runtime_result,
            "normal_load_peak_steering_angle_rad",
            "simulation runtime",
        ),
        peer_rtk_displacement_m=rtk_log_evidence.peer_displacement_m,
        logged_rtk_displacement_m=rtk_log_evidence.logged_displacement_m,
        rtk_log_match_count=rtk_log_evidence.match_count,
        rtk_log_max_position_error_m=rtk_log_evidence.max_position_error_m,
        normal_load_window_start_sim_time_ns=window_start_sim_time_ns,
        normal_load_window_end_sim_time_ns=window_end_sim_time_ns,
        wheel_drain_timestamp_ns=wheel_drain_timestamp_ns,
        wheel_log_publish_count=wheel_log_evidence.logged_count,
        wheel_peer_receive_count=wheel_log_evidence.peer_count,
        wheel_log_match_count=wheel_log_evidence.match_count,
        wheel_drain_complete=wheel_drain_complete,
        clean_shutdown=True,
    )
    _assert_roundtrip_result(result, config=config, codec=codec)
    _assert_simulation_result(result, config=config)
    return result


def run_ecal_process_roundtrip(
    duration_sec: float = 2.5,
    *,
    runtime: str = "transport",
    warmup_sec: float = _DEFAULT_WARMUP_SEC,
    process_timeout_sec: float = _DEFAULT_PROCESS_TIMEOUT_SEC,
    evidence_dir: str | Path | None = None,
    robot_model: str = "active_steering_4wd",
) -> RoundtripResult:
    """按指定 runtime 运行真实 eCAL 双进程验收，不允许本地替代。"""
    duration = _positive_finite("duration_sec", duration_sec)
    if runtime not in _SUPPORTED_RUNTIMES:
        raise ValueError(f"runtime must be one of {sorted(_SUPPORTED_RUNTIMES)}")
    warmup = _nonnegative_finite("warmup_sec", warmup_sec)
    process_timeout = _positive_finite("process_timeout_sec", process_timeout_sec)
    if evidence_dir is not None and not isinstance(evidence_dir, (str, Path)):
        raise ValueError("evidence_dir must be a path")
    retained_evidence_dir = None if evidence_dir is None else Path(evidence_dir)
    if runtime != "simulation" and retained_evidence_dir is not None:
        raise ValueError("evidence_dir is only supported by the simulation runtime")
    if runtime == "transport":
        return _run_ecal_transport_roundtrip(
            duration_sec=duration,
            process_timeout_sec=process_timeout,
        )
    if warmup <= 0.0:
        raise ValueError("warmup_sec must be a positive finite number for simulation")
    selected_robot_model = get_robot_model(robot_model).name
    return _run_ecal_simulation_roundtrip(
        duration_sec=duration,
        warmup_sec=warmup,
        process_timeout_sec=process_timeout,
        evidence_dir=retained_evidence_dir,
        robot_model=selected_robot_model,
    )


def _assert_reconnect_result(
    result: ReconnectResult,
    *,
    command: tuple[float, float],
    silence_sec: float,
) -> None:
    """验证强制断线前后均有非零控制证据，并覆盖完整静默窗口。"""
    silence = _positive_finite("silence_sec", silence_sec)
    if result.transport_name != "ecal":
        raise AssertionError("transport must be ecal")
    if result.states != ("active", "disconnected", "waiting_peer", "active"):
        raise AssertionError(f"unexpected reconnect states: {result.states}")
    if result.drive_target_before_disconnect != command:
        raise AssertionError("first peer command was not applied before disconnect")
    for name, generation in (
        (
            "mailbox_generation_before_disconnect",
            result.mailbox_generation_before_disconnect,
        ),
        (
            "mailbox_generation_after_disconnect",
            result.mailbox_generation_after_disconnect,
        ),
    ):
        if isinstance(generation, bool) or not isinstance(generation, int) or generation < 0:
            raise AssertionError(f"{name} must be a nonnegative integer")
    if (
        result.mailbox_generation_after_disconnect
        <= result.mailbox_generation_before_disconnect
    ):
        raise AssertionError("mailbox generation did not advance on disconnect")
    if result.first_peer_terminated is not True:
        raise AssertionError("first peer was not explicitly terminated")
    if result.first_peer_returncode == 0:
        raise AssertionError("first peer exited naturally")
    if not 0.0 <= result.first_peer_runtime_sec < result.first_peer_planned_duration_sec:
        raise AssertionError("first peer was not terminated before its planned duration")
    if result.drive_target_while_disconnected != (0.0, 0.0):
        raise AssertionError("drive target was not zero while disconnected")
    if result.drive_target_after_peer_restart_before_new_command != (0.0, 0.0):
        raise AssertionError("stale drive target returned before a new command")
    minimum_samples = max(2, math.floor(silence / 0.02))
    if result.silence_observed_sec < silence:
        raise AssertionError("reconnect silence window was not fully observed")
    if result.silence_sample_count < minimum_samples:
        raise AssertionError("reconnect silence window had too few samples")
    if result.silence_all_zero is not True:
        raise AssertionError("drive target changed during reconnect silence")
    if result.drive_target_after_new_command != command:
        raise AssertionError("new generation command was not applied")


def run_ecal_reconnect_gate(
    command: tuple[float, float] = (4.0, 4.0),
    silence_sec: float = 0.15,
) -> ReconnectResult:
    """强制终止并重启真实 peer，验证 mailbox 代际和控制目标。"""
    silence = _positive_finite("silence_sec", silence_sec)
    if (
        len(command) != 2
        or any(not math.isfinite(value) for value in command)
        or command == (0.0, 0.0)
    ):
        raise ValueError("command must contain two finite nonzero-control values")
    token = uuid4().hex[:10]
    config = InterfaceConfig.default(transport_mode="ecal")
    codec = ProtoCodec()
    command_topic = config.wheel_command.topic
    command_type = _command_topic_type(config, codec)
    mailbox = WheelCommandMailbox(get_robot_model("df_back"))
    state_lock = Lock()
    states: list[str] = []

    def peer_state_changed(state: str) -> None:
        with state_lock:
            if not states or states[-1] != state:
                states.append(state)
        if state == "disconnected":
            mailbox.clear()

    transport = create_transport(
        "ecal",
        config=config,
        participant_name=f"slope-sim-reconnect-{token}",
        peer_state_callback=peer_state_changed,
    )
    if not isinstance(transport, EcalTransport) or transport.snapshot().mode != "ecal":
        transport.close()
        raise RuntimeError("strict reconnect harness did not create an ecal transport")

    def record_command(payload: bytes, received_at: float) -> bool:
        # 回调入口固定 mailbox 引用和 token；解码完成后只向同一引用提交。
        mailbox_ref = mailbox
        generation = mailbox_ref.capture_generation()
        decoded = codec.decode_wheel_command(payload)
        return mailbox_ref.accept(
            decoded,
            received_at=received_at,
            generation=generation,
        )

    def drive_target() -> tuple[float, float]:
        decision = mailbox.decision(now=time.monotonic())
        return tuple(decision.drive_wheel_speed_rad_s)

    subscription = transport.subscribe(command_topic, command_type, record_command)
    first_peer: subprocess.Popen[str] | None = None
    second_peer: subprocess.Popen[str] | None = None
    try:
        with tempfile.TemporaryDirectory(prefix="slope-ecal-reconnect-") as temp_dir:
            temp_path = Path(temp_dir)
            first_started_at = time.monotonic()
            first_peer = _start_peer(
                result_json=temp_path / "first.json",
                ready_file=temp_path / "first.ready",
                start_file=temp_path / "first.start",
                duration_sec=_FIRST_RECONNECT_PEER_DURATION_SEC,
                participant_name=f"slope-sim-reconnect-peer-a-{token}",
                command=command,
            )
            _wait_until(
                (temp_path / "first.ready").exists,
                timeout_sec=10.0,
                description="first peer readiness",
                process=first_peer,
            )
            _wait_until(
                lambda: _all_topic_peers_connected(transport),
                timeout_sec=10.0,
                description="first peer six-topic discovery",
                process=first_peer,
            )
            (temp_path / "first.start").write_text("start\n", encoding="utf-8")
            _wait_until(
                lambda: _polled_transport_state(transport) == "active",
                timeout_sec=10.0,
                description="first active command",
                process=first_peer,
            )
            _wait_until(
                lambda: drive_target() == command,
                timeout_sec=2.0,
                description="first nonzero drive target",
                process=first_peer,
            )
            target_before_disconnect = drive_target()
            generation_before_disconnect = mailbox.capture_generation()
            with state_lock:
                states[:] = ["active"]
            first_peer_returncode = _terminate_running_peer(
                first_peer,
                timeout_sec=3.0,
            )
            first_peer_runtime = time.monotonic() - first_started_at
            _wait_until(
                lambda: (
                    _polled_transport_state(transport) == "disconnected"
                    and drive_target() == (0.0, 0.0)
                ),
                timeout_sec=(
                    _ECAL_REGISTRATION_TIMEOUT_SEC + _ECAL_REGISTRATION_MARGIN_SEC
                ),
                description="peer disconnection and zero drive target",
            )
            disconnected_target = drive_target()
            generation_after_disconnect = mailbox.capture_generation()

            second_duration = silence + 0.60
            second_peer = _start_peer(
                result_json=temp_path / "second.json",
                ready_file=temp_path / "second.ready",
                start_file=temp_path / "second.start",
                duration_sec=second_duration,
                participant_name=f"slope-sim-reconnect-peer-b-{token}",
                command=command,
                command_delay_sec=silence,
            )
            _wait_until(
                (temp_path / "second.ready").exists,
                timeout_sec=10.0,
                description="second peer readiness",
                process=second_peer,
            )
            _wait_until(
                lambda: (
                    _polled_transport_state(transport) == "waiting_peer"
                    and _all_topic_peers_connected(transport)
                ),
                timeout_sec=10.0,
                description="restarted peer six-topic discovery",
                process=second_peer,
            )
            restarted_target = drive_target()
            (temp_path / "second.start").write_text("start\n", encoding="utf-8")

            silence_started_at = time.monotonic()
            silence_targets: list[tuple[float, float]] = []
            silence_deadline = silence_started_at + silence
            while time.monotonic() < silence_deadline:
                if second_peer.poll() is not None:
                    raise RuntimeError("second peer exited during reconnect silence")
                current_target = drive_target()
                silence_targets.append(current_target)
                if current_target != (0.0, 0.0):
                    raise AssertionError("drive target changed during reconnect silence")
                if _polled_transport_state(transport) != "waiting_peer":
                    raise AssertionError("transport left waiting_peer during silence")
                remaining = silence_deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(min(_SILENCE_SAMPLE_PERIOD_SEC, remaining))
            silence_observed = time.monotonic() - silence_started_at

            _wait_until(
                lambda: _polled_transport_state(transport) == "active",
                timeout_sec=10.0,
                description="new generation command",
                process=second_peer,
            )
            _wait_until(
                lambda: drive_target() == command,
                timeout_sec=2.0,
                description="new generation drive target",
                process=second_peer,
            )
            target_after_new_command = drive_target()
            with state_lock:
                accepted_states = tuple(states)
            _finish_peer(second_peer, timeout_sec=second_duration + 5.0)
    finally:
        for process in (first_peer, second_peer):
            _reap_process(process)
        subscription.close()
        transport.close()

    result = ReconnectResult(
        transport_name="ecal",
        states=accepted_states,
        drive_target_before_disconnect=target_before_disconnect,
        mailbox_generation_before_disconnect=generation_before_disconnect,
        mailbox_generation_after_disconnect=generation_after_disconnect,
        first_peer_terminated=True,
        first_peer_returncode=first_peer_returncode,
        first_peer_runtime_sec=first_peer_runtime,
        first_peer_planned_duration_sec=_FIRST_RECONNECT_PEER_DURATION_SEC,
        drive_target_while_disconnected=disconnected_target,
        drive_target_after_peer_restart_before_new_command=restarted_target,
        silence_observed_sec=silence_observed,
        silence_sample_count=len(silence_targets),
        silence_all_zero=all(target == (0.0, 0.0) for target in silence_targets),
        drive_target_after_new_command=target_after_new_command,
    )
    _assert_reconnect_result(result, command=command, silence_sec=silence)
    return result


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Verify the real eCAL process roundtrip")
    parser.add_argument(
        "--runtime",
        choices=sorted(_SUPPORTED_RUNTIMES),
        default="transport",
    )
    parser.add_argument("--warmup-sec", type=float, default=_DEFAULT_WARMUP_SEC)
    parser.add_argument("--duration-sec", type=float, default=2.5)
    parser.add_argument(
        "--process-timeout-sec",
        type=float,
        default=_DEFAULT_PROCESS_TIMEOUT_SEC,
    )
    parser.add_argument("--reconnect-silence-sec", type=float, default=0.15)
    parser.add_argument("--evidence-dir", type=Path, default=None)
    parser.add_argument(
        "--robot-model",
        choices=robot_model_names(),
        default="active_steering_4wd",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    config = InterfaceConfig.default(transport_mode="ecal")
    expected_types = _expected_topic_types(config, ProtoCodec())
    try:
        roundtrip_kwargs = {
            "runtime": args.runtime,
            "warmup_sec": args.warmup_sec,
            "process_timeout_sec": args.process_timeout_sec,
            "evidence_dir": args.evidence_dir,
        }
        if args.runtime == "simulation":
            roundtrip_kwargs["robot_model"] = args.robot_model
        result = run_ecal_process_roundtrip(args.duration_sec, **roundtrip_kwargs)
        reconnect = (
            run_ecal_reconnect_gate(silence_sec=args.reconnect_silence_sec)
            if args.runtime == "transport"
            else None
        )
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1

    if args.runtime == "simulation":
        print("runtime=simulation transport=ecal")
    else:
        print("transport=ecal")
    for topic in expected_types:
        peer_state = (
            f" peer_state={result.per_topic_peer_states[topic]}"
            if args.runtime == "simulation"
            else ""
        )
        print(
            f"topic={topic} type={result.topic_types[topic]} "
            f"count={result.message_counts[topic]} "
            f"wall_hz={result.wall_clock_hz[topic]:.3f} "
            f"timestamp_hz={result.message_timestamp_hz[topic]:.3f}"
            f"{peer_state}"
        )
    if reconnect is not None:
        print(f"reconnect_states={','.join(reconnect.states)}")
        print(
            "reconnect_targets="
            f"{reconnect.drive_target_before_disconnect}->"
            f"{reconnect.drive_target_while_disconnected}->"
            f"{reconnect.drive_target_after_new_command}"
        )
        print(
            "reconnect_mailbox_generation="
            f"{reconnect.mailbox_generation_before_disconnect}->"
            f"{reconnect.mailbox_generation_after_disconnect}"
        )
    else:
        print(f"physics_feedback={result.feedback_is_not_command_echo}")
        print(f"invalid_command_rejected={result.invalid_command_rejected}")
        print(
            "timeout_safe_stop="
            f"{result.timeout_stopped_vehicle and result.timeout_preserved_steering}"
        )
        print(f"reconnect_new_command={result.reconnect_required_new_command}")
        print(
            "reconnect_mailbox_generation="
            f"{result.mailbox_generation_before_disconnect}->"
            f"{result.mailbox_generation_after_disconnect}"
        )
        print(
            "normal_load="
            f"obstacles={result.normal_load_obstacle_count} "
            f"log_samples={result.normal_load_log_sample_count} "
            f"log_accepted={result.normal_load_log_accepted_messages} "
            f"max_pending={result.normal_load_log_max_pending} "
            f"final_pending={result.normal_load_log_final_pending} "
            f"log_dropped="
            f"{result.normal_load_log_dropped_messages + result.normal_load_log_dropped_events}"
        )
        print(
            "window="
            f"requested_sec={result.normal_load_requested_duration_sec:.6f} "
            f"peer_sec={result.peer_measurement_duration_sec:.6f} "
            f"physics_dt_sec={result.normal_load_physics_time_step_sec:.9f}"
        )
        print(
            "realtime="
            f"steps={result.normal_load_step_count} "
            f"sim_sec={result.normal_load_sim_duration_sec:.6f} "
            f"wall_sec={result.normal_load_wall_duration_sec:.6f} "
            f"sim_wall_ratio={result.normal_load_sim_wall_ratio:.6f} "
            f"control_steps={result.normal_load_control_step_count} "
            f"control_sim_sec={result.normal_load_control_sim_duration_sec:.6f} "
            f"control_wall_sec={result.normal_load_control_wall_duration_sec:.6f}"
        )
        print(
            "controlled_motion="
            f"steps={result.normal_load_controlled_motion_step_count} "
            f"sim_sec={result.normal_load_controlled_motion_sim_duration_sec:.6f} "
            f"displacement_m={result.normal_load_controlled_displacement_m:.6f} "
            f"path_m={result.normal_load_controlled_path_length_m:.6f} "
            f"mean_m_s={result.normal_load_controlled_mean_speed_m_s:.6f} "
            f"max_m_s={result.normal_load_controlled_max_speed_m_s:.6f}"
        )
        print(
            "obstacle_contacts="
            f"steps={result.normal_load_obstacle_contact_step_count}"
        )
        print(
            "motion="
            f"displacement_m={result.normal_load_base_displacement_m:.6f} "
            f"path_m={result.normal_load_base_path_length_m:.6f} "
            f"mean_m_s={result.normal_load_base_mean_speed_m_s:.6f} "
            f"max_m_s={result.normal_load_base_max_speed_m_s:.6f}"
        )
        print(
            "rtk_log_chain="
            f"peer_displacement_m={result.peer_rtk_displacement_m:.6f} "
            f"logged_displacement_m={result.logged_rtk_displacement_m:.6f} "
            f"matches={result.rtk_log_match_count} "
            f"max_error_m={result.rtk_log_max_position_error_m:.9f}"
        )
        print(
            "wheel_log_delivery="
            f"logged={result.wheel_log_publish_count} "
            f"peer={result.wheel_peer_receive_count} "
            f"matches={result.wheel_log_match_count} "
            f"fence_timestamp_ns={result.wheel_drain_timestamp_ns}"
        )
        if args.evidence_dir is not None:
            print(f"evidence_dir={args.evidence_dir.resolve()}")
        print(f"clean_shutdown={result.clean_shutdown}")
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
