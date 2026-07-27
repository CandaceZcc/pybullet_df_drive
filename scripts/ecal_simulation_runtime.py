#!/usr/bin/env python3
# 真实 eCAL 仿真子进程：以 PyBullet DIRECT 和正式 InterfaceRuntime 驱动六话题。
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time
from typing import Sequence

import pybullet as p


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import SimulationCoordinator, build_world_from_scene_document
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.ecal_transport import EcalTransport, create_transport
from slope_sim.interfaces.runtime import InterfaceRuntime
from slope_sim.scene_config import SceneDocument
from slope_sim.sensor_backend import PyBulletSensorBackend
from slope_sim.simulation import _PeerStateRelay, initial_scene_document


_STOP_SPEED_TOLERANCE_RAD_S = 0.05
_STEERING_HOLD_TOLERANCE_RAD = 0.02


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


def _drive_is_stopped(values: tuple[float, ...]) -> bool:
    return bool(values) and max(abs(value) for value in values) <= _STOP_SPEED_TOLERANCE_RAD_S


def _read_marker(path: Path) -> dict[str, object] | None:
    if not path.exists():
        return None
    loaded = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ValueError(f"scenario marker must contain an object: {path}")
    return loaded


def run_simulation_runtime(
    *,
    result_json: Path,
    scenario_dir: Path,
    ready_file: Path,
    start_file: Path,
    stop_file: Path,
    participant_name: str,
    max_runtime_sec: float,
) -> None:
    """创建正式 DIRECT 世界，实时推进物理并记录安全场景证据。"""
    max_runtime = _positive_finite("max_runtime_sec", max_runtime_sec)
    config = ExperimentConfig(
        mode="direct",
        duration_sec=max_runtime,
        robot_model="active_steering_4wd",
        interface_mode="ecal",
        interface_enabled=True,
        interface_log_enabled=False,
        dashboard_enabled=False,
    )
    interface_config = InterfaceConfig.default(transport_mode="ecal")
    client_id = p.connect(p.DIRECT)
    if client_id < 0:
        raise RuntimeError("failed to connect PyBullet DIRECT")

    transport = None
    backend = None
    runtime = None
    final_transport_snapshot = None
    result: dict[str, object] = {}
    try:
        document = initial_scene_document(config)
        world, obstacle_manager = build_world_from_scene_document(
            client_id, config, document
        )
        backend = PyBulletSensorBackend(
            client_id, world.active_robot.robot.robot_id
        )
        backend.bind_scene(
            world.scene.body_ids,
            obstacle_manager.snapshot(include_body_id=True),
        )
        runtime_document = SceneDocument.from_runtime(
            document.robot_model,
            document.terrain,
            obstacle_manager.snapshot(include_body_id=False),
            document.sensors.mounts,
            lidar_config=document.sensors.lidar,
        )
        relay = _PeerStateRelay()
        transport = create_transport(
            "ecal",
            config=interface_config,
            participant_name=participant_name,
            peer_state_callback=relay,
        )
        if not isinstance(transport, EcalTransport):
            raise RuntimeError("strict simulation runtime did not create EcalTransport")
        runtime = InterfaceRuntime(
            world.active_robot.robot,
            config=interface_config,
            transport=transport,
            sensor_backend=backend,
            scene_document=runtime_document,
        )
        relay.attach(runtime, transport)
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            obstacle_manager,
            interface_runtime=runtime,
            sensor_document=document.sensors,
        )

        scenario_dir.mkdir(parents=True, exist_ok=True)
        ready_file.parent.mkdir(parents=True, exist_ok=True)
        ready_file.write_text("ready\n", encoding="utf-8")

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
        accepted_peer_states: dict[str, str] = {}

        started_at = time.monotonic()
        next_deadline = started_at
        while not stop_file.exists():
            now = time.monotonic()
            if now - started_at >= max_runtime:
                raise TimeoutError("simulation runtime exceeded its process budget")

            runtime.poll_transport()
            runtime.before_physics_step(config.time_step, wall_time=now)
            coordinator.step(config.time_step)
            runtime.after_physics_step(config.time_step)

            # 物理后钩子会发布新事件，快照查询时刻必须晚于这些发布事件。
            snapshot_time = time.monotonic()
            status = runtime.status_snapshot(wall_time=snapshot_time)
            dashboard = runtime.dashboard_snapshot(wall_time=snapshot_time)
            for topic, topic_status in status.topics.items():
                if last_states.get(topic) != topic_status.state:
                    state_history[topic].append(
                        {
                            "wall_time": snapshot_time,
                            "state": topic_status.state,
                        }
                    )
                    last_states[topic] = topic_status.state

            command = dashboard.wheel_command
            wheel_state = dashboard.wheel_state
            if command is not None and wheel_state is not None:
                if len(command.drive_wheel_speed_rad_s) == len(
                    wheel_state.drive_wheel_speed_rad_s
                ) and any(
                    abs(actual - requested) > 1e-3
                    for actual, requested in zip(
                        wheel_state.drive_wheel_speed_rad_s,
                        command.drive_wheel_speed_rad_s,
                        strict=True,
                    )
                ):
                    feedback_is_not_command_echo = True

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
                        timeout_reference_at = now
                    timeout_stopped_vehicle = _drive_is_stopped(
                        wheel_state.drive_wheel_speed_rad_s
                    )
                    if (
                        timeout_reference_at is not None
                        and now - timeout_reference_at >= 0.05
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
                if marker is None or marker.get("topic") != topic:
                    continue
                if status.topics[topic].state == "waiting_peer":
                    isolated = all(
                        status.topics[other].state != "waiting_peer"
                        for other in output_topics
                        if other != topic
                    )
                    output_disconnect_isolated[topic] = isolated
                    _write_ack(
                        scenario_dir / f"drop_{index}.ack",
                        {"topic": topic, "isolated": isolated},
                    )

            if (scenario_dir / "command_disconnected.active").exists():
                if (
                    status.topics[interface_config.wheel_command.topic].state
                    == "disconnected"
                    and wheel_state is not None
                    and _drive_is_stopped(wheel_state.drive_wheel_speed_rad_s)
                ):
                    disconnected_stopped = True
                    _write_ack(
                        scenario_dir / "command_disconnected.ack",
                        {"stopped": True},
                    )

            if (scenario_dir / "command_reconnected_wait.active").exists():
                if (
                    status.topics[interface_config.wheel_command.topic].state
                    == "waiting_peer"
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
                    reconnect_required_new_command = disconnected_stopped
                    accepted_peer_states = topic_states
                    _write_ack(
                        scenario_dir / "new_command.ack",
                        {"active": True},
                    )

            next_deadline += config.time_step
            delay = next_deadline - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)

        final_transport_snapshot = transport.snapshot()
        result = {
            "transport": "ecal",
            "runtime": "simulation",
            "feedback_is_not_command_echo": feedback_is_not_command_echo,
            "invalid_command_rejected": invalid_command_rejected,
            "timeout_stopped_vehicle": timeout_stopped_vehicle,
            "timeout_preserved_steering": timeout_preserved_steering,
            "output_disconnect_isolated": output_disconnect_isolated,
            "per_topic_peer_states": accepted_peer_states,
            "reconnect_required_new_command": reconnect_required_new_command,
            "state_history": state_history,
            "transport_snapshot": {
                "dropped_count": final_transport_snapshot.dropped_count,
                "error_count": final_transport_snapshot.error_count,
            },
        }
    finally:
        close_error: BaseException | None = None
        close_trace: tuple[str, ...] = ()
        if runtime is not None:
            try:
                runtime.close()
                close_trace = runtime.close_trace
            except BaseException as exc:
                close_error = exc
        else:
            for resource in (transport, backend):
                close = None if resource is None else getattr(resource, "close", None)
                if callable(close):
                    try:
                        close()
                    except BaseException as exc:
                        if close_error is None:
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
        )
    except Exception as exc:
        print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
