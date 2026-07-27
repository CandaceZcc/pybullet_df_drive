#!/usr/bin/env python3
# eCAL 环回对端：发布轮子命令、订阅五个仿真输出并记录逐消息 JSON。
from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
from threading import Lock
import time
from typing import Callable, Sequence


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


_NANOSECONDS_PER_SECOND = 1_000_000_000
_SCENARIO_ACK_TIMEOUT_SEC = 1.5


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


def _command_topic_type(config: InterfaceConfig, codec: ProtoCodec) -> str:
    """由 codec 生成轮子命令 registration 类型。"""
    return codec.type_name(WheelCommand(0, (), ()))


def _channel_period_sec(channel: ChannelConfig) -> float:
    """由通道目标频率计算唯一墙钟周期。"""
    return 1.0 / float(channel.rate_hz)


def _message_timestamp_ns(index: int, channel: ChannelConfig) -> int:
    """由消息序号和通道频率计算仿真时间戳。"""
    return round(index * _NANOSECONDS_PER_SECOND / channel.rate_hz)


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


def _write_marker(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


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
) -> tuple[list[dict[str, object]], float, float]:
    """以 100 Hz 发送主动转向命令，并返回精确测量窗口。"""
    channel = config.wheel_command
    period_sec = _channel_period_sec(channel)
    started_at = time.monotonic()
    ended_at = started_at + duration_sec
    next_send = started_at
    index = 0
    events: list[dict[str, object]] = []
    while True:
        now = time.monotonic()
        if now >= ended_at:
            break
        while next_send <= now and next_send < ended_at:
            index += 1
            timestamp_ns = _message_timestamp_ns(index, channel)
            command = WheelCommand(
                timestamp_ns,
                (4.0, 4.0, 4.0, 4.0),
                (0.6, 0.6),
            )
            sent_at = time.monotonic()
            _send_v61_command(bindings, publisher, command, codec)
            events.append(
                {
                    "wall_time": sent_at,
                    "timestamp_ns": timestamp_ns,
                    "type": codec.type_name(command),
                }
            )
            next_send = started_at + index * period_sec
        delay = min(next_send, ended_at) - time.monotonic()
        if delay > 0.0:
            time.sleep(delay)
    return events, started_at, ended_at


def run_simulation_peer(
    *,
    result_json: Path,
    scenario_dir: Path,
    duration_sec: float,
    participant_name: str,
    ready_file: Path,
    start_file: Path,
    start_timeout_sec: float = 15.0,
) -> None:
    """驱动真实 PyBullet 对端，并逐个断开/恢复六个官方 eCAL 端点。"""
    duration = _positive_finite("duration_sec", duration_sec)
    timeout = _positive_finite("start_timeout_sec", start_timeout_sec)
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
            with event_lock:
                received[topic].append(
                    {
                        "wall_time": time.monotonic(),
                        "timestamp_ns": timestamp_ns,
                        "type": decoded_type,
                    }
                )

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

        with event_lock:
            for events in received.values():
                events.clear()
        command_events, measurement_start, measurement_end = _run_v61_command_schedule(
            bindings,
            command_publisher,
            duration_sec=duration,
            config=config,
            codec=codec,
        )
        time.sleep(0.03)
        with event_lock:
            measured_received = {
                topic: [
                    event
                    for event in events
                    if measurement_start <= float(event["wall_time"]) < measurement_end
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
            marker.unlink(missing_ok=True)
            time.sleep(0.05)

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
                WheelCommand(
                    invalid_timestamp + new_index * 10_000_000,
                    (3.0, 3.0, 3.0, 3.0),
                    (0.0, 0.0),
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
            "commands": command_events,
            "received": measured_received,
            "measurement_start": measurement_start,
            "measurement_end": measurement_end,
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
    parser.add_argument("--drive-command", type=float, nargs=2, default=(4.0, 4.0))
    parser.add_argument("--participant-name", default="slope-sim-roundtrip-peer")
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--start-file", type=Path)
    parser.add_argument("--command-delay-sec", type=float, default=0.0)
    parser.add_argument("--start-timeout-sec", type=float, default=15.0)
    parser.add_argument("--settle-sec", type=float, default=0.20)
    parser.add_argument("--simulation-scenario", action="store_true")
    parser.add_argument("--scenario-dir", type=Path)
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
                participant_name=args.participant_name,
                ready_file=args.ready_file,
                start_file=args.start_file,
                start_timeout_sec=args.start_timeout_sec,
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
