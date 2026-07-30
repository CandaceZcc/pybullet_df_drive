#!/usr/bin/env python3
# eCAL 环回对端：发布轮子命令、订阅五个仿真输出并记录逐消息 JSON。
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
from threading import Lock
import time
from typing import Callable, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.config import ChannelConfig, InterfaceConfig
from slope_sim.interfaces.ecal_transport import EcalTransport, create_transport
from slope_sim.interfaces.ecal_transport import load_ecal_bindings
from slope_sim.interfaces.generated import slope_sim_interfaces_pb2 as pb
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.model_registry import get_robot_model, robot_model_names


_NANOSECONDS_PER_SECOND = 1_000_000_000
# 官方默认 10 秒判定注册失联；额外两秒只用于跨进程调度余量。
_SCENARIO_ACK_TIMEOUT_SEC = 12.0


def _positive_finite(name: str, value: float) -> float:
    """校验命令行持续时间，避免静默生成空门禁结果。"""
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return normalized


def _nonnegative_finite(name: str, value: float) -> float:
    normalized = float(value)
    if not math.isfinite(normalized) or normalized < 0.0:
        raise ValueError(f"{name} must be a nonnegative finite number")
    return normalized


def _wait_for_start(start_file: Path | None, timeout_sec: float) -> None:
    """用文件门闩让两个独立进程从同一测量阶段开始。"""
    if start_file is None:
        return
    deadline = time.monotonic() + timeout_sec
    while not start_file.exists():
        if time.monotonic() >= deadline:
            raise TimeoutError(f"start signal not received: {start_file}")
        time.sleep(0.005)


def _output_topic_types(
    config: InterfaceConfig,
    codec: ProtoCodec,
) -> dict[str, str]:
    """由集中配置和 codec 生成五个输出 registration 类型。"""
    return {
        config.wheel_state.topic: codec.type_name(WheelState(0, (), ())),
        config.lidar_front.topic: codec.type_name(
            LidarPointCloud(0, "lidar_front", 0, 1, ())
        ),
        config.lidar_rear.topic: codec.type_name(
            LidarPointCloud(0, "lidar_rear", 0, 2, ())
        ),
        config.rtk.topic: codec.type_name(RtkState(0, 0.0, 0.0, 0.0, 0.0)),
        config.imu.topic: codec.type_name(ImuAttitude(0, 0.0, 0.0)),
    }


def _decode_output_event(
    config: InterfaceConfig,
    codec: ProtoCodec,
    topic: str,
    payload: bytes,
) -> tuple[int, str]:
    """按配置选择 decoder，并从真实解码模型提取时间戳和类型。"""
    if topic == config.wheel_state.topic:
        message = codec.decode_wheel_state(payload)
        timestamp_ns = message.timestamp_ns
    elif topic in {config.lidar_front.topic, config.lidar_rear.topic}:
        message = codec.decode_lidar_point_cloud(payload)
        timestamp_ns = message.timebase_ns
    elif topic == config.rtk.topic:
        message = codec.decode_rtk_state(payload)
        timestamp_ns = message.timestamp_ns
    elif topic == config.imu.topic:
        message = codec.decode_imu_attitude(payload)
        timestamp_ns = message.timestamp_ns
    else:
        raise KeyError(f"topic {topic!r} is not configured as an output")
    return timestamp_ns, codec.type_name(message)


def _rtk_position_evidence(
    config: InterfaceConfig,
    codec: ProtoCodec,
    topic: str,
    payload: bytes,
) -> dict[str, object]:
    """仅为真实解码的 RTK 事件附加三维位置，保持通用解码契约不变。"""
    if topic != config.rtk.topic:
        return {}
    message = codec.decode_rtk_state(payload)
    return {"position_m": [message.main_x, message.main_y, message.main_z]}


def _payload_evidence(payload: bytes) -> dict[str, str]:
    """保存确定性 Protobuf 负载摘要，供原始日志到 peer 的逐帧核验。"""
    return {"payload_sha256": hashlib.sha256(payload).hexdigest()}


def _command_topic_type(config: InterfaceConfig, codec: ProtoCodec) -> str:
    """由 codec 生成轮子命令 registration 类型。"""
    return codec.type_name(WheelCommand(0, (), ()))


def _channel_period_sec(channel: ChannelConfig) -> float:
    """由通道目标频率计算唯一墙钟周期。"""
    return 1.0 / float(channel.rate_hz)


def _message_timestamp_ns(index: int, channel: ChannelConfig) -> int:
    """由消息序号和通道频率计算仿真时间戳。"""
    return round(index * _NANOSECONDS_PER_SECOND / channel.rate_hz)


def _simulation_command_for_model(
    robot_model: str,
    timestamp_ns: int,
    *,
    drive_speed_rad_s: float,
) -> WheelCommand:
    """按车型构造正式场景命令，锁定差速 2+0 与主动转向 4+2。"""
    speed = float(drive_speed_rad_s)
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("drive_speed_rad_s must be a positive finite number")
    model = get_robot_model(robot_model)
    if model.controller_kind == "differential":
        drive = (speed, speed * 0.75)
    else:
        drive = (speed,) * len(model.drive_joint_names)
    steering = (0.6,) * len(model.steering_joint_names)
    return WheelCommand(timestamp_ns, drive, steering)


def _run_command_schedule(
    transport: EcalTransport,
    *,
    duration_sec: float,
    command_delay_sec: float,
    drive_command: tuple[float, float],
    config: InterfaceConfig,
    codec: ProtoCodec,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> list[dict[str, object]]:
    """按 wheel_command 配置调度命令，并返回实际发送事件。"""
    channel = config.wheel_command
    period_sec = _channel_period_sec(channel)
    command_type = _command_topic_type(config, codec)
    measurement_start = monotonic()
    measurement_end = measurement_start + duration_sec
    schedule_start = measurement_start + command_delay_sec
    next_command = schedule_start
    command_index = 0
    command_events: list[dict[str, object]] = []

    while True:
        now = monotonic()
        if now >= measurement_end:
            break
        while next_command <= now and next_command < measurement_end:
            command_index += 1
            timestamp_ns = _message_timestamp_ns(command_index, channel)
            command = WheelCommand(timestamp_ns, drive_command, ())
            payload = codec.encode(command)
            sent_at = monotonic()
            transport.publish(
                channel.topic,
                payload,
                command_type,
                timestamp_ns,
                wall_time=sent_at,
            )
            command_events.append(
                {
                    "wall_time": sent_at,
                    "timestamp_ns": timestamp_ns,
                    "type": codec.type_name(command),
                }
            )
            next_command = schedule_start + command_index * period_sec

        remaining = min(next_command, measurement_end) - monotonic()
        if remaining > 0.0:
            sleep(remaining)

    return command_events


def _official_output_message_types() -> dict[str, type]:
    """返回 simulation peer 创建五个官方 Subscriber 所需的生成类型。"""
    config = InterfaceConfig.default(transport_mode="ecal")
    return {
        config.wheel_state.topic: pb.WheelState,
        config.lidar_front.topic: pb.LidarPointCloud,
        config.lidar_rear.topic: pb.LidarPointCloud,
        config.rtk.topic: pb.RtkState,
        config.imu.topic: pb.ImuAttitude,
    }


def _wait_for_file(path: Path, timeout_sec: float, description: str) -> None:
    """等待另一验收进程给出明确确认，超时即失败。"""
    deadline = time.monotonic() + timeout_sec
    while not path.exists():
        if time.monotonic() >= deadline:
            raise TimeoutError(f"timed out waiting for {description}: {path}")
        time.sleep(0.005)


def _wait_for_json_object(
    path: Path,
    timeout_sec: float,
    description: str,
    *,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    """等待完整 JSON 回执；文件存在但仍在写入时继续有界重试。"""
    deadline = monotonic() + _positive_finite("timeout_sec", timeout_sec)
    while True:
        try:
            loaded = json.loads(path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            loaded = None
        if loaded is not None:
            if not isinstance(loaded, dict):
                raise ValueError(f"{description} must contain a JSON object")
            return loaded
        now = monotonic()
        if now >= deadline:
            raise TimeoutError(f"timed out waiting for {description}: {path}")
        sleep(min(0.005, deadline - now))


def _ack_sim_time_ns(payload: Mapping[str, object], key: str) -> int:
    """从屏障回执严格读取 uint64 仿真边界。"""
    value = payload.get(key)
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 0 <= value < 1 << 64
    ):
        raise ValueError(f"{key} must be a uint64 integer")
    return value


def _events_in_sim_window(
    events: Sequence[Mapping[str, object]],
    *,
    start_sim_time_ns: int,
    end_sim_time_ns: int,
) -> list[Mapping[str, object]]:
    """按 runtime 共享仿真边界选择 `(start, end]` 的输出事件。"""
    start = _ack_sim_time_ns(
        {"start_sim_time_ns": start_sim_time_ns},
        "start_sim_time_ns",
    )
    end = _ack_sim_time_ns(
        {"end_sim_time_ns": end_sim_time_ns},
        "end_sim_time_ns",
    )
    if start >= end:
        raise ValueError("simulation measurement window must be increasing")
    selected: list[Mapping[str, object]] = []
    for event in events:
        timestamp_ns = _ack_sim_time_ns(event, "timestamp_ns")
        if start < timestamp_ns <= end:
            selected.append(event)
    return selected


def _wait_for_output_fence(
    received: Mapping[str, list[dict[str, object]]],
    event_lock: object,
    *,
    topic: str,
    after_timestamp_ns: int,
    timeout_sec: float,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """等待同 publisher 的 post-window 帧，证明此前 callback 已越过窗口尾。"""
    boundary = _ack_sim_time_ns(
        {"after_timestamp_ns": after_timestamp_ns},
        "after_timestamp_ns",
    )
    deadline = monotonic() + _positive_finite("timeout_sec", timeout_sec)
    while True:
        with event_lock:  # type: ignore[attr-defined]
            timestamps = tuple(
                _ack_sim_time_ns(event, "timestamp_ns")
                for event in received.get(topic, ())
            )
        candidates = tuple(value for value in timestamps if value > boundary)
        if candidates:
            return min(candidates)
        now = monotonic()
        if now >= deadline:
            raise TimeoutError("timed out waiting for wheel-state delivery fence")
        sleep(min(0.005, deadline - now))


def _wait_for_all_transport_peers(
    transport: EcalTransport,
    timeout_sec: float,
) -> None:
    """命令和五个输出端点全部发现后才开始正式测量。"""
    deadline = time.monotonic() + timeout_sec
    while True:
        transport.poll_peer_state()
        quality = transport.snapshot().topic_quality
        if quality and all(item.peer_connected is True for item in quality):
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("all six eCAL peers were not discovered before measurement")
        time.sleep(0.01)


def _wait_for_v61_resource_peer(
    bindings: object,
    resource: object,
    timeout_sec: float,
    description: str,
) -> None:
    """有界等待一个重建后的官方 v6 资源重新发现对应端点。"""
    deadline = time.monotonic() + timeout_sec
    while not bindings.is_peer_connected(resource):
        if time.monotonic() >= deadline:
            raise TimeoutError(f"timed out waiting for {description}")
        time.sleep(0.01)


def _write_marker(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


def _complete_measurement_window(
    scenario_dir: Path,
    *,
    duration_sec: float,
    measurement_start: float,
    measurement_end: float,
) -> dict[str, object]:
    """冻结 runtime 正常窗口，并返回含仿真尾边界的完整 JSON 回执。"""
    duration = _positive_finite("duration_sec", duration_sec)
    start = float(measurement_start)
    end = float(measurement_end)
    if not math.isfinite(start) or not math.isfinite(end) or end <= start:
        raise ValueError("measurement window must be finite and increasing")
    marker = scenario_dir / "measurement_complete.active"
    _write_marker(
        marker,
        {
            "duration_sec": duration,
            "measurement_start": start,
            "measurement_end": end,
        },
    )
    payload = _wait_for_json_object(
        scenario_dir / "measurement_complete.ack",
        _SCENARIO_ACK_TIMEOUT_SEC,
        "normal-load transport snapshot",
    )
    marker.unlink(missing_ok=True)
    return payload


def _send_v61_command(bindings, publisher, command: WheelCommand, codec: ProtoCodec) -> None:
    """通过官方 Publisher 发送已由领域模型校验/编码的 WheelCommand。"""
    bindings.send(publisher, codec.encode(command), pb.WheelCommand)


def _run_v61_command_schedule(
    bindings,
    publisher,
    *,
    duration_sec: float,
    config: InterfaceConfig,
    codec: ProtoCodec,
    robot_model: str = "active_steering_4wd",
    start_index: int = 1,
) -> tuple[list[dict[str, object]], float, float]:
    """以 100 Hz 发送当前车型命令，并返回精确测量窗口。"""
    if isinstance(start_index, bool) or not isinstance(start_index, int) or start_index <= 0:
        raise ValueError("start_index must be a positive integer")
    channel = config.wheel_command
    period_sec = _channel_period_sec(channel)
    started_at = time.monotonic()
    ended_at = started_at + duration_sec
    next_send = started_at
    sent_count = 0
    events: list[dict[str, object]] = []
    while True:
        now = time.monotonic()
        if now >= ended_at:
            break
        while next_send <= now and next_send < ended_at:
            index = start_index + sent_count
            timestamp_ns = _message_timestamp_ns(index, channel)
            command = _simulation_command_for_model(
                robot_model,
                timestamp_ns,
                drive_speed_rad_s=4.0,
            )
            sent_at = time.monotonic()
            _send_v61_command(bindings, publisher, command, codec)
            events.append(
                {
                    "wall_time": sent_at,
                    "timestamp_ns": timestamp_ns,
                    "type": codec.type_name(command),
                    "drive_wheel_count": len(command.drive_wheel_speed_rad_s),
                    "steering_wheel_count": len(
                        command.steering_wheel_speed_rad_s
                    ),
                }
            )
            sent_count += 1
            next_send = started_at + sent_count * period_sec
        delay = min(next_send, ended_at) - time.monotonic()
        if delay > 0.0:
            time.sleep(delay)
    return events, started_at, ended_at


def run_simulation_peer(
    *,
    result_json: Path,
    scenario_dir: Path,
    duration_sec: float,
    warmup_sec: float,
    participant_name: str,
    ready_file: Path,
    start_file: Path,
    start_timeout_sec: float = 15.0,
    robot_model: str = "active_steering_4wd",
) -> None:
    """驱动真实 PyBullet 对端，并逐个断开/恢复六个官方 eCAL 端点。"""
    duration = _positive_finite("duration_sec", duration_sec)
    warmup = _positive_finite("warmup_sec", warmup_sec)
    timeout = _positive_finite("start_timeout_sec", start_timeout_sec)
    selected_robot_model = get_robot_model(robot_model).name
    config = InterfaceConfig.default(transport_mode="ecal")
    codec = ProtoCodec()
    bindings = load_ecal_bindings()
    participant = None
    command_publisher = None
    subscribers: dict[str, object] = {}
    event_lock = Lock()
    received: dict[str, list[dict[str, object]]] = {
        topic: [] for topic in _official_output_message_types()
    }

    def make_callback(topic: str):
        def record(payload: bytes) -> None:
            timestamp_ns, decoded_type = _decode_output_event(
                config, codec, topic, payload
            )
            event = {
                "wall_time": time.monotonic(),
                "timestamp_ns": timestamp_ns,
                "type": decoded_type,
                **_payload_evidence(payload),
                **_rtk_position_evidence(config, codec, topic, payload),
            }
            with event_lock:
                received[topic].append(event)

        return record

    try:
        participant = bindings.create_participant(participant_name)
        command_publisher = bindings.create_publisher(
            config.wheel_command.topic, pb.WheelCommand
        )
        for topic, message_type in _official_output_message_types().items():
            subscribers[topic] = bindings.create_subscriber(
                topic, message_type, make_callback(topic)
            )

        scenario_dir.mkdir(parents=True, exist_ok=True)
        ready_file.parent.mkdir(parents=True, exist_ok=True)
        ready_file.write_text("ready\n", encoding="utf-8")
        _wait_for_start(start_file, timeout)

        discovery_deadline = time.monotonic() + timeout
        while True:
            resources = [command_publisher, *subscribers.values()]
            if all(bindings.is_peer_connected(resource) for resource in resources):
                break
            if time.monotonic() >= discovery_deadline:
                raise TimeoutError("all six eCAL peers were not discovered before measurement")
            time.sleep(0.01)

        # 共享 start 门闩之后用同一正式 peer 驱动真实生产预热。
        warmup_events, _warmup_start, _warmup_end = _run_v61_command_schedule(
            bindings,
            command_publisher,
            duration_sec=warmup,
            config=config,
            codec=codec,
            robot_model=selected_robot_model,
        )

        measurement_start_marker = scenario_dir / "measurement_start.active"
        _write_marker(
            measurement_start_marker,
            {"warmup_sec": warmup, "duration_sec": duration},
        )
        measurement_start_ack = _wait_for_json_object(
            scenario_dir / "measurement_start.ack",
            _SCENARIO_ACK_TIMEOUT_SEC,
            "normal-load transport baseline",
        )
        window_start_sim_time_ns = _ack_sim_time_ns(
            measurement_start_ack,
            "window_start_sim_time_ns",
        )
        measurement_start_marker.unlink(missing_ok=True)

        command_events, measurement_start, measurement_end = _run_v61_command_schedule(
            bindings,
            command_publisher,
            duration_sec=duration,
            config=config,
            codec=codec,
            robot_model=selected_robot_model,
            start_index=len(warmup_events) + 1,
        )
        measurement_complete_ack = _complete_measurement_window(
            scenario_dir,
            duration_sec=duration,
            measurement_start=measurement_start,
            measurement_end=measurement_end,
        )
        window_end_sim_time_ns = _ack_sim_time_ns(
            measurement_complete_ack,
            "window_end_sim_time_ns",
        )
        if window_end_sim_time_ns <= window_start_sim_time_ns:
            raise ValueError("simulation measurement window must be increasing")
        delivery_fence_timestamp_ns = _wait_for_output_fence(
            received,
            event_lock,
            topic=config.wheel_state.topic,
            after_timestamp_ns=window_end_sim_time_ns,
            timeout_sec=_SCENARIO_ACK_TIMEOUT_SEC,
        )
        with event_lock:
            measured_received = {
                topic: [
                    dict(event)
                    for event in _events_in_sim_window(
                        events,
                        start_sim_time_ns=window_start_sim_time_ns,
                        end_sim_time_ns=window_end_sim_time_ns,
                    )
                ]
                for topic, events in received.items()
            }

        invalid_timestamp = 9_000_000_000_000
        invalid_marker = scenario_dir / "invalid.active"
        _write_marker(invalid_marker, {"timestamp_ns": invalid_timestamp})
        _send_v61_command(
            bindings,
            command_publisher,
            WheelCommand(invalid_timestamp, (19.0,), ()),
            codec,
        )
        _wait_for_file(
            scenario_dir / "invalid.ack",
            _SCENARIO_ACK_TIMEOUT_SEC,
            "invalid command rejection",
        )
        invalid_marker.unlink(missing_ok=True)

        timeout_marker = scenario_dir / "timeout.active"
        _write_marker(timeout_marker, {})
        _wait_for_file(
            scenario_dir / "timeout.ack",
            _SCENARIO_ACK_TIMEOUT_SEC,
            "100 ms timeout stop",
        )
        timeout_marker.unlink(missing_ok=True)

        output_topics = tuple(_official_output_message_types())
        for index, topic in enumerate(output_topics):
            marker = scenario_dir / f"drop_{index}.active"
            _write_marker(marker, {"topic": topic})
            resource = subscribers.pop(topic)
            bindings.close(resource)
            _wait_for_file(
                scenario_dir / f"drop_{index}.ack",
                _SCENARIO_ACK_TIMEOUT_SEC,
                f"isolated output disconnect {topic}",
            )
            subscribers[topic] = bindings.create_subscriber(
                topic,
                _official_output_message_types()[topic],
                make_callback(topic),
            )
            _wait_for_v61_resource_peer(
                bindings,
                subscribers[topic],
                _SCENARIO_ACK_TIMEOUT_SEC,
                f"restored subscriber discovery {topic}",
            )
            restored_marker = scenario_dir / f"drop_{index}.restored"
            _write_marker(restored_marker, {"topic": topic})
            _wait_for_file(
                scenario_dir / f"drop_{index}.restored.ack",
                _SCENARIO_ACK_TIMEOUT_SEC,
                f"restored output discovery {topic}",
            )
            restored_marker.unlink(missing_ok=True)
            marker.unlink(missing_ok=True)

        prepare_marker = scenario_dir / "command_disconnect_prepare.active"
        prepare_ack = scenario_dir / "command_disconnect_prepare.ack"
        _write_marker(prepare_marker, {})
        prepare_started = time.monotonic()
        prepare_index = 0
        while not prepare_ack.exists():
            if time.monotonic() - prepare_started >= _SCENARIO_ACK_TIMEOUT_SEC:
                raise TimeoutError(
                    f"timed out waiting for active command generation: {prepare_ack}"
                )
            prepare_index += 1
            _send_v61_command(
                bindings,
                command_publisher,
                _simulation_command_for_model(
                    selected_robot_model,
                    invalid_timestamp + prepare_index * 10_000_000,
                    drive_speed_rad_s=3.0,
                ),
                codec,
            )
            time.sleep(0.01)
        prepare_marker.unlink(missing_ok=True)

        disconnected_marker = scenario_dir / "command_disconnected.active"
        _write_marker(disconnected_marker, {})
        bindings.close(command_publisher)
        command_publisher = None
        _wait_for_file(
            scenario_dir / "command_disconnected.ack",
            _SCENARIO_ACK_TIMEOUT_SEC,
            "command peer disconnection",
        )
        disconnected_marker.unlink(missing_ok=True)

        command_publisher = bindings.create_publisher(
            config.wheel_command.topic, pb.WheelCommand
        )
        reconnect_marker = scenario_dir / "command_reconnected_wait.active"
        _write_marker(reconnect_marker, {})
        _wait_for_file(
            scenario_dir / "command_reconnected_wait.ack",
            _SCENARIO_ACK_TIMEOUT_SEC,
            "reconnected peer waiting for a new command",
        )
        _wait_for_v61_resource_peer(
            bindings,
            command_publisher,
            _SCENARIO_ACK_TIMEOUT_SEC,
            "reconnected command publisher discovery",
        )
        reconnect_marker.unlink(missing_ok=True)

        new_marker = scenario_dir / "new_command.active"
        _write_marker(new_marker, {})
        new_started = time.monotonic()
        new_index = 0
        while time.monotonic() - new_started < 0.25:
            new_index += 1
            _send_v61_command(
                bindings,
                command_publisher,
                _simulation_command_for_model(
                    selected_robot_model,
                    invalid_timestamp + new_index * 10_000_000,
                    drive_speed_rad_s=3.0,
                ),
                codec,
            )
            time.sleep(0.01)
        _wait_for_file(
            scenario_dir / "new_command.ack",
            _SCENARIO_ACK_TIMEOUT_SEC,
            "new generation command activation",
        )
        new_marker.unlink(missing_ok=True)

        final_peer_states = {
            config.wheel_command.topic: bindings.is_peer_connected(command_publisher),
            **{
                topic: bindings.is_peer_connected(resource)
                for topic, resource in subscribers.items()
            },
        }
        result = {
            "transport": "ecal",
            "runtime": "simulation",
            "robot_model": selected_robot_model,
            "commands": command_events,
            "warmup_command_count": len(warmup_events),
            "received": measured_received,
            "requested_duration_sec": duration,
            "peer_measurement_duration_sec": measurement_end - measurement_start,
            "measurement_start": measurement_start,
            "measurement_end": measurement_end,
            "window_start_sim_time_ns": window_start_sim_time_ns,
            "window_end_sim_time_ns": window_end_sim_time_ns,
            "measurement_sim_duration_sec": (
                window_end_sim_time_ns - window_start_sim_time_ns
            )
            / _NANOSECONDS_PER_SECOND,
            "wheel_drain_complete": True,
            "wheel_drain_timestamp_ns": delivery_fence_timestamp_ns,
            "wheel_delivery_fence_timestamp_ns": delivery_fence_timestamp_ns,
            "final_peer_states": final_peer_states,
            "snapshot": {"dropped_count": 0, "error_count": 0},
        }
    finally:
        for resource in reversed(tuple(subscribers.values())):
            bindings.close(resource)
        if command_publisher is not None:
            bindings.close(command_publisher)
        if participant is not None:
            bindings.close(participant)

    result["clean_shutdown"] = True
    result_json.parent.mkdir(parents=True, exist_ok=True)
    result_json.write_text(
        json.dumps(result, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def run_peer(
    *,
    result_json: Path,
    duration_sec: float,
    drive_command: tuple[float, float],
    participant_name: str,
    ready_file: Path | None = None,
    start_file: Path | None = None,
    command_delay_sec: float = 0.0,
    start_timeout_sec: float = 15.0,
    settle_sec: float = 0.20,
) -> None:
    """运行真实 peer，并在退出前完整写入命令与五输出事件。"""
    duration = _positive_finite("duration_sec", duration_sec)
    command_delay = _nonnegative_finite("command_delay_sec", command_delay_sec)
    start_timeout = _positive_finite("start_timeout_sec", start_timeout_sec)
    settle = _nonnegative_finite("settle_sec", settle_sec)
    if command_delay >= duration:
        raise ValueError("command_delay_sec must be smaller than duration_sec")
    if len(drive_command) != 2 or any(not math.isfinite(value) for value in drive_command):
        raise ValueError("drive_command must contain two finite values")

    config = InterfaceConfig.default(transport_mode="ecal")
    codec = ProtoCodec()
    output_types = _output_topic_types(config, codec)
    transport = create_transport(
        "ecal",
        config=config,
        role="peer",
        participant_name=participant_name,
    )
    if not isinstance(transport, EcalTransport) or transport.snapshot().mode != "ecal":
        transport.close()
        raise RuntimeError("strict peer did not create an ecal transport")

    event_lock = Lock()
    received: dict[str, list[dict[str, object]]] = {
        topic: [] for topic in output_types
    }
    command_events: list[dict[str, object]] = []
    subscriptions = []

    for topic, type_name in output_types.items():
        def record_output(
            payload: bytes,
            received_at: float,
            *,
            event_topic: str = topic,
        ) -> bool:
            timestamp_ns, decoded_type = _decode_output_event(
                config,
                codec,
                event_topic,
                payload,
            )
            with event_lock:
                received[event_topic].append(
                    {
                        "wall_time": received_at,
                        "timestamp_ns": timestamp_ns,
                        "type": decoded_type,
                    }
                )
            return True

        subscriptions.append(transport.subscribe(topic, type_name, record_output))

    try:
        if ready_file is not None:
            ready_file.parent.mkdir(parents=True, exist_ok=True)
            ready_file.write_text("ready\n", encoding="utf-8")
        _wait_for_start(start_file, start_timeout)
        _wait_for_all_transport_peers(transport, start_timeout)

        command_events = _run_command_schedule(
            transport,
            duration_sec=duration,
            command_delay_sec=command_delay,
            drive_command=drive_command,
            config=config,
            codec=codec,
        )

        if settle > 0.0:
            time.sleep(settle)
        with event_lock:
            result = {
                "transport": "ecal",
                "commands": list(command_events),
                "received": {
                    topic: list(events) for topic, events in received.items()
                },
                "snapshot": asdict(transport.snapshot()),
            }
    finally:
        for subscription in subscriptions:
            subscription.close()
        transport.close()

    result_json.parent.mkdir(parents=True, exist_ok=True)
    result_json.write_text(
        json.dumps(result, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the real eCAL roundtrip peer")
    parser.add_argument("--result-json", type=Path, required=True)
    parser.add_argument("--duration-sec", type=float, default=2.5)
    parser.add_argument("--warmup-sec", type=float, default=1.0)
    parser.add_argument("--drive-command", type=float, nargs=2, default=(4.0, 4.0))
    parser.add_argument("--participant-name", default="slope-sim-roundtrip-peer")
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--start-file", type=Path)
    parser.add_argument("--command-delay-sec", type=float, default=0.0)
    parser.add_argument("--start-timeout-sec", type=float, default=15.0)
    parser.add_argument("--settle-sec", type=float, default=0.20)
    parser.add_argument("--simulation-scenario", action="store_true")
    parser.add_argument("--scenario-dir", type=Path)
    parser.add_argument(
        "--robot-model",
        choices=robot_model_names(),
        default="active_steering_4wd",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        if args.simulation_scenario:
            if args.scenario_dir is None or args.ready_file is None or args.start_file is None:
                raise ValueError(
                    "simulation scenario requires --scenario-dir, --ready-file, and --start-file"
                )
            run_simulation_peer(
                result_json=args.result_json,
                scenario_dir=args.scenario_dir,
                duration_sec=args.duration_sec,
                warmup_sec=args.warmup_sec,
                participant_name=args.participant_name,
                ready_file=args.ready_file,
                start_file=args.start_file,
                start_timeout_sec=args.start_timeout_sec,
                robot_model=args.robot_model,
            )
        else:
            run_peer(
                result_json=args.result_json,
                duration_sec=args.duration_sec,
                drive_command=tuple(args.drive_command),
                participant_name=args.participant_name,
                ready_file=args.ready_file,
                start_file=args.start_file,
                command_delay_sec=args.command_delay_sec,
                start_timeout_sec=args.start_timeout_sec,
                settle_sec=args.settle_sec,
            )
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
