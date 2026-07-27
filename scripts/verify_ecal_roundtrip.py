#!/usr/bin/env python3
# 真实 eCAL 双进程门禁：验证六话题频率、类型以及断连重启后的安全清零。
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
from threading import Lock
import time
from typing import Callable, Mapping, Sequence
from uuid import uuid4


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.config import ChannelConfig, InterfaceConfig
from slope_sim.interfaces.ecal_transport import EcalTransport, create_transport
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.interfaces.wheel import WheelCommandMailbox
from slope_sim.model_registry import get_robot_model
from scripts.ecal_roundtrip_peer import (
    _channel_period_sec,
    _command_topic_type,
    _message_timestamp_ns,
    _output_topic_types,
)


_PEER_SCRIPT = Path(__file__).with_name("ecal_roundtrip_peer.py")
_SIMULATION_RUNTIME_SCRIPT = Path(__file__).with_name("ecal_simulation_runtime.py")
_SUPPORTED_RUNTIMES = frozenset({"transport", "simulation"})
_DEFAULT_WARMUP_SEC = 1.0
_DEFAULT_PROCESS_TIMEOUT_SEC = 20.0
_MIN_COUNT_RATIO = 0.85
_MAX_COUNT_RATIO = 1.15
_MIN_WINDOW_COVERAGE_RATIO = 0.80
_SILENCE_SAMPLE_PERIOD_SEC = 0.005
_FIRST_RECONNECT_PEER_DURATION_SEC = 5.0
_ECAL_REGISTRATION_TIMEOUT_SEC = 10.0
_ECAL_REGISTRATION_MARGIN_SEC = 2.0
_CHILD_TERMINATE_TIMEOUT_SEC = 3.0


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
    peer_dropped_count: int = 0
    transport_error_count: int = 0
    peer_error_count: int = 0
    runtime_name: str = "transport"
    feedback_is_not_command_echo: bool = False
    invalid_command_rejected: bool = False
    timeout_stopped_vehicle: bool = False
    timeout_preserved_steering: bool = False
    output_disconnect_isolated: Mapping[str, bool] = field(default_factory=dict)
    per_topic_peer_states: Mapping[str, str] = field(default_factory=dict)
    reconnect_required_new_command: bool = False
    clean_shutdown: bool = False


@dataclass(frozen=True)
class ReconnectResult:
    """真实 peer 退出和重启期间的连接状态与驱动目标快照。"""

    transport_name: str
    states: tuple[str, ...]
    drive_target_before_disconnect: tuple[float, float]
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


def _all_topic_peers_connected(transport: EcalTransport) -> bool:
    """只有六个端点均完成 discovery 才允许进入测量窗口。"""
    quality = transport.snapshot().topic_quality
    return bool(quality) and all(item.peer_connected is True for item in quality)


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
    participant_name: str,
    start_timeout_sec: float,
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
            "--participant-name",
            participant_name,
            "--start-timeout-sec",
            str(start_timeout_sec),
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
    observed = {str(event["type"]) for event in events}
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

    wheel_topics = {command_topic, wheel_state_topic}
    for channel in selected_config.channels:
        topic = channel.topic
        target_hz = float(channel.rate_hz)
        wall_tolerance = 0.05 if topic in wheel_topics else 0.10
        wall_lower = target_hz * (1.0 - wall_tolerance)
        wall_upper = target_hz * (1.0 + wall_tolerance)
        if not wall_lower <= result.wall_clock_hz[topic] <= wall_upper:
            raise AssertionError(f"{topic} wall frequency out of range")

        timestamp_lower = target_hz * (1.0 - 0.01)
        timestamp_upper = target_hz * (1.0 + 0.01)
        if not (
            timestamp_lower
            <= result.message_timestamp_hz[topic]
            <= timestamp_upper
        ):
            raise AssertionError(f"{topic} timestamp frequency out of range")


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
            peer_result = json.loads(result_path.read_text(encoding="utf-8"))
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
        events = received_json.get(topic)
        if not isinstance(events, list):
            raise AssertionError(f"peer result missing event list for {topic}")
        events_by_topic[topic] = events

    wall_clock_hz = {
        topic: _frequency_hz([float(event["wall_time"]) for event in events])
        for topic, events in events_by_topic.items()
    }
    timestamp_hz = {
        topic: _timestamp_frequency_hz(
            [int(event["timestamp_ns"]) for event in events]
        )
        for topic, events in events_by_topic.items()
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
        topic: _event_span_sec([float(event["wall_time"]) for event in events])
        for topic, events in events_by_topic.items()
    }
    result = RoundtripResult(
        transport_name="ecal",
        peer_returncode=peer_returncode,
        wall_clock_hz=wall_clock_hz,
        message_timestamp_hz=timestamp_hz,
        received_topics=received_topics,
        topic_types=topic_types,
        message_counts=message_counts,
        dropped_count=int(transport_snapshot.dropped_count),
        duration_sec=duration,
        event_span_sec=event_spans,
        peer_dropped_count=int(peer_snapshot.get("dropped_count", -1)),
        transport_error_count=int(transport_snapshot.error_count),
        peer_error_count=int(peer_snapshot.get("error_count", -1)),
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


def _strict_true(mapping: Mapping[str, object], key: str, description: str) -> bool:
    if mapping.get(key) is not True:
        raise AssertionError(f"{description} must report {key}=true")
    return True


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


def _run_ecal_simulation_roundtrip(
    *,
    duration_sec: float,
    warmup_sec: float,
    process_timeout_sec: float,
) -> RoundtripResult:
    """编排真实 PyBullet runtime 与官方 eCAL peer，并合并双方证据。"""
    token = uuid4().hex[:10]
    config = InterfaceConfig.default(transport_mode="ecal")
    codec = ProtoCodec()
    command_topic, _wheel_state_topic, _sensor_topics, output_topics = (
        _configured_topics(config)
    )
    expected_types = _expected_topic_types(config, codec)
    runtime_process: subprocess.Popen[str] | None = None
    peer_process: subprocess.Popen[str] | None = None

    with tempfile.TemporaryDirectory(prefix="slope-ecal-simulation-") as temp_dir:
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
                max_runtime_sec=process_timeout_sec,
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
                participant_name=f"slope-sim-physics-peer-{token}",
                start_timeout_sec=process_timeout_sec,
            )
            _wait_until(
                peer_ready_path.exists,
                timeout_sec=process_timeout_sec,
                description="simulation peer readiness",
                process=peer_process,
                process_name="simulation peer",
            )
            _wait_for_live_processes(
                warmup_sec,
                (
                    ("simulation runtime", runtime_process),
                    ("simulation peer", peer_process),
                ),
            )
            start_path.write_text("start\n", encoding="utf-8")
            peer_returncode, _peer_stdout, _peer_stderr = _finish_peer(
                peer_process,
                timeout_sec=process_timeout_sec,
                process_name="simulation peer",
            )
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

    for description, payload in (
        ("simulation peer", peer_result),
        ("simulation runtime", runtime_result),
    ):
        if payload.get("transport") != "ecal":
            raise AssertionError(f"{description} did not report transport=ecal")
        if payload.get("runtime") != "simulation":
            raise AssertionError(f"{description} did not report runtime=simulation")
        _strict_true(payload, "clean_shutdown", description)

    peer_snapshot = peer_result.get("snapshot")
    runtime_snapshot = runtime_result.get("transport_snapshot")
    if not isinstance(peer_snapshot, dict):
        raise AssertionError("simulation peer snapshot must be an object")
    if not isinstance(runtime_snapshot, dict):
        raise AssertionError("simulation runtime transport_snapshot must be an object")

    final_peer_states = peer_result.get("final_peer_states")
    if not isinstance(final_peer_states, dict) or final_peer_states != {
        topic: True for topic in expected_types
    }:
        raise AssertionError("simulation peer final states are not all connected")

    events_by_topic: dict[str, list[Mapping[str, object]]] = {
        command_topic: _event_list(peer_result.get("commands"), "command events")
    }
    received = peer_result.get("received")
    if not isinstance(received, dict):
        raise AssertionError("simulation peer received field must be an object")
    for topic in output_topics:
        events_by_topic[topic] = _event_list(
            received.get(topic), f"received events for {topic}"
        )

    wall_clock_hz = {
        topic: _frequency_hz([float(event["wall_time"]) for event in events])
        for topic, events in events_by_topic.items()
    }
    timestamp_hz = {
        topic: _timestamp_frequency_hz(
            [int(event["timestamp_ns"]) for event in events]
        )
        for topic, events in events_by_topic.items()
    }
    message_counts = {topic: len(events) for topic, events in events_by_topic.items()}
    event_spans = {
        topic: _event_span_sec([float(event["wall_time"]) for event in events])
        for topic, events in events_by_topic.items()
    }
    output_isolation = runtime_result.get("output_disconnect_isolated")
    topic_states = runtime_result.get("per_topic_peer_states")
    if not isinstance(output_isolation, dict):
        raise AssertionError("simulation output_disconnect_isolated must be an object")
    if not isinstance(topic_states, dict):
        raise AssertionError("simulation per_topic_peer_states must be an object")

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
        dropped_count=int(runtime_snapshot.get("dropped_count", -1)),
        duration_sec=duration_sec,
        event_span_sec=event_spans,
        peer_dropped_count=int(peer_snapshot.get("dropped_count", -1)),
        transport_error_count=int(runtime_snapshot.get("error_count", -1)),
        peer_error_count=int(peer_snapshot.get("error_count", -1)),
        runtime_name="simulation",
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
) -> RoundtripResult:
    """按指定 runtime 运行真实 eCAL 双进程验收，不允许本地替代。"""
    duration = _positive_finite("duration_sec", duration_sec)
    if runtime not in _SUPPORTED_RUNTIMES:
        raise ValueError(f"runtime must be one of {sorted(_SUPPORTED_RUNTIMES)}")
    warmup = _nonnegative_finite("warmup_sec", warmup_sec)
    process_timeout = _positive_finite("process_timeout_sec", process_timeout_sec)
    if runtime == "transport":
        return _run_ecal_transport_roundtrip(
            duration_sec=duration,
            process_timeout_sec=process_timeout,
        )
    return _run_ecal_simulation_roundtrip(
        duration_sec=duration,
        warmup_sec=warmup,
        process_timeout_sec=process_timeout,
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
                lambda: transport.snapshot().state == "active",
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
            with state_lock:
                states[:] = ["active"]
            first_peer_returncode = _terminate_running_peer(
                first_peer,
                timeout_sec=3.0,
            )
            first_peer_runtime = time.monotonic() - first_started_at
            _wait_until(
                lambda: (
                    transport.snapshot().state == "disconnected"
                    and drive_target() == (0.0, 0.0)
                ),
                timeout_sec=(
                    _ECAL_REGISTRATION_TIMEOUT_SEC + _ECAL_REGISTRATION_MARGIN_SEC
                ),
                description="peer disconnection and zero drive target",
            )
            disconnected_target = drive_target()

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
                    transport.snapshot().state == "waiting_peer"
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
                if transport.snapshot().state != "waiting_peer":
                    raise AssertionError("transport left waiting_peer during silence")
                remaining = silence_deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(min(_SILENCE_SAMPLE_PERIOD_SEC, remaining))
            silence_observed = time.monotonic() - silence_started_at

            _wait_until(
                lambda: transport.snapshot().state == "active",
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
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    config = InterfaceConfig.default(transport_mode="ecal")
    expected_types = _expected_topic_types(config, ProtoCodec())
    try:
        result = run_ecal_process_roundtrip(
            args.duration_sec,
            runtime=args.runtime,
            warmup_sec=args.warmup_sec,
            process_timeout_sec=args.process_timeout_sec,
        )
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
    else:
        print(f"physics_feedback={result.feedback_is_not_command_echo}")
        print(f"invalid_command_rejected={result.invalid_command_rejected}")
        print(
            "timeout_safe_stop="
            f"{result.timeout_stopped_vehicle and result.timeout_preserved_steering}"
        )
        print(f"reconnect_new_command={result.reconnect_required_new_command}")
        print(f"clean_shutdown={result.clean_shutdown}")
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
