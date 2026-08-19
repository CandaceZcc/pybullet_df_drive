#!/usr/bin/env python3
"""MID-360 Golf 离线采集、严格校验与三维回放的唯一公开入口。"""

from __future__ import annotations

import argparse
from hashlib import sha256
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Sequence
from uuid import uuid4


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.interfaces.v2.descriptor import DescriptorIdentity, load_v2_descriptor
from slope_sim.mapping_acceptance import (
    ACCEPTANCE_THRESHOLDS,
    CaptureAcceptanceMetrics,
    MappingAcceptanceMetrics,
    acceptance_failures,
    capture_acceptance_document,
    evaluate_mapping_session,
    parse_capture_acceptance,
)
from slope_sim.mapping_mcap import MappingSessionIndex, load_mapping_session
from slope_sim.mid360_golf_drive import build_canonical_golf_route
from slope_sim.scene import TerrainBounds
from slope_sim.scene_config import load_scene


_SCENE_PATH = ROOT / "configs/mid360_golf_mapping.yaml"
_PATTERN_PATH = ROOT / "slope_sim/assets/mid360_pattern.bin"
_DESCRIPTOR_PATH = (
    ROOT / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
)
_SCENE_ID = "mid360-golf-mapping-v1"
_COMMAND_SOURCE_ID = "mid360.golf.command-peer"
_PATTERN_VERSION = "livox-mid360-800000-v1"
_PATTERN_SHA256 = (
    "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca"
)
_WORLD_GENERATION = 1
_COMMAND_GENERATION = 1
_SIMULATION_DURATION_NS = 208_200_000_000
_PHYSICS_STEPS = 49_968
_FRAME_PERIOD_NS = 100_000_000
_RECORDER_DEADLINE_MS = 6 * 60 * 60 * 1_000
_PEER_TIMEOUT_SEC = 120.0
_STARTUP_TIMEOUT_SEC = 180.0
_FAULT_GRACE_SEC = 5.0
_PROCESS_TERM_TIMEOUT_SEC = 2.0
_POLL_INTERVAL_SEC = 0.01
_GOLF_BOUNDS = TerrainBounds(-10.01, 10.01, -6.65, 6.65)

_EXPECTED_COUNTS = {
    "/sim/wheel/command": 20_820,
    "/sim/wheel/state": 20_820,
    "/sim/lidar/points": 2_082,
    "/sim/rtk/state": 2_083,
    "/sim/imu/attitude": 2_083,
}
_SIMULATOR_COUNTS = {
    topic: count
    for topic, count in _EXPECTED_COUNTS.items()
    if topic != "/sim/wheel/command"
}
_SIMULATOR_RESULT_KEYS = frozenset(
    {
        "role",
        "clean_shutdown",
        "fault_reason",
        "simulation_session_id",
        "descriptor_sha256",
        "world_generation",
        "command_generation",
        "scene_id",
        "client_id",
        "connection_mode",
        "physics_steps",
        "sim_duration_ns",
        "robot_model",
        "terrain_model",
        "golf_seed",
        "golf_relief",
        "static_obstacle_count",
        "moving_obstacle_count",
        "published_frames",
        "expected_topic_counts",
        "active_command_steps",
        "truth_acceptance",
        "transport_metrics",
    }
)
_COMMAND_RESULT_KEYS = frozenset(
    {
        "role",
        "clean_shutdown",
        "fault_reason",
        "source_id",
        "source_session_id",
        "descriptor_sha256",
        "published_frames",
        "last_wheel_timestamp_ns",
        "latest_pose_timestamp_ns",
        "normal_stop_started",
        "finished",
    }
)
_RECORDER_RESULT_KEYS = frozenset(
    {"clean_shutdown", "mcap", "recorded_count", "role", "topics"}
)


def _is_normalized_absolute(path: Path) -> bool:
    return path.is_absolute() and Path(os.path.normpath(str(path))) == path


def _require_new_output_directory(path: Path) -> None:
    if (
        not _is_normalized_absolute(path)
        or path.exists()
        or not path.parent.is_dir()
    ):
        raise ValueError(
            "output_dir must be a new absolute directory below an existing directory"
        )


def _require_recorder(path: Path) -> None:
    if (
        not _is_normalized_absolute(path)
        or not path.is_file()
        or not os.access(path, os.X_OK)
        or path.name != "slope_sim_stage4_recorder"
    ):
        raise ValueError(
            "recorder must be the absolute normalized slope_sim_stage4_recorder executable"
        )


def _require_exporter(path: Path) -> None:
    if (
        not _is_normalized_absolute(path)
        or not path.is_file()
        or not os.access(path, os.X_OK)
        or path.name != "slope_sim_stage4_export"
    ):
        raise ValueError(
            "exporter must be the absolute normalized slope_sim_stage4_export executable"
        )


def _preflight(
    recorder: Path,
    output_dir: Path,
    exporter: Path | None,
) -> DescriptorIdentity:
    """在创建任何结果目录前锁定正式场景、pattern、descriptor 和 Recorder。"""
    _require_recorder(recorder)
    if exporter is not None:
        _require_exporter(exporter)
    _require_new_output_directory(output_dir)
    for name, path in (
        ("scene", _SCENE_PATH),
        ("MID-360 pattern", _PATTERN_PATH),
        ("v2 descriptor", _DESCRIPTOR_PATH),
    ):
        if not path.is_absolute() or not path.is_file():
            raise RuntimeError(f"fixed {name} file is unavailable")

    scene = load_scene(_SCENE_PATH)
    static_ids = tuple(
        sorted(obstacle.logical_id for obstacle in scene.obstacles if obstacle.mode == "static")
    )
    moving_ids = tuple(
        sorted(obstacle.logical_id for obstacle in scene.obstacles if obstacle.mode == "moving")
    )
    if (
        scene.robot_model != "df_mid"
        or scene.terrain.terrain_model != "golf_heightfield"
        or scene.terrain.golf_seed != 41
        or scene.terrain.golf_relief != "medium"
        or static_ids != (1, 2, 3, 4, 5, 6)
        or moving_ids != (7, 8, 9)
    ):
        raise RuntimeError("fixed MID-360 Golf scene differs from the canonical contract")

    with _PATTERN_PATH.open("rb") as stream:
        pattern_digest = sha256(stream.read()).hexdigest()
    if pattern_digest != _PATTERN_SHA256:
        raise RuntimeError("fixed MID-360 pattern digest does not match")
    descriptor = load_v2_descriptor()
    if not descriptor.serialized_file_descriptor_set or len(descriptor.sha256) != 32:
        raise RuntimeError("fixed v2 descriptor identity is invalid")
    return descriptor


def _export_mapping_lidar(
    *,
    exporter: Path,
    mcap_path: Path,
    output_dir: Path,
) -> tuple[Path, Path]:
    """只把已验收的完整 MCAP 交给 C++ Export，并复核可导入 LVX2 已发布。"""
    export_dir = output_dir / "export"
    export_result = output_dir / "export.json"
    command = [
        str(exporter),
        "--input", str(mcap_path),
        "--descriptor-set", str(_DESCRIPTOR_PATH),
        "--output-dir", str(export_dir),
        "--result", str(export_result),
    ]
    completed = subprocess.run(command, text=True, capture_output=True, check=False)
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(f"mapping export failed ({completed.returncode}): {detail}")
    lvx2_path = export_dir / "lidar.lvx2"
    if not lvx2_path.is_file() or lvx2_path.stat().st_size == 0:
        raise RuntimeError("mapping export did not publish lidar.lvx2")
    receipt = _read_json(export_result, name="mapping export result")
    if (
        receipt.get("clean_shutdown") is not True
        or receipt.get("role") != "export"
        or receipt.get("lvx2") != "lidar.lvx2"
        or any(type(receipt.get(name)) is not int or receipt[name] <= 0 for name in (
            "lidar_frames", "lvx2_frames", "lvx2_packages", "lvx2_points",
        ))
    ):
        raise RuntimeError("mapping export result is incomplete")
    return lvx2_path, export_result


def _unique_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    document: dict[str, object] = {}
    for key, value in pairs:
        if key in document:
            raise ValueError(f"duplicate JSON key: {key}")
        document[key] = value
    return document


def _read_json(path: Path, *, name: str) -> dict[str, object]:
    if not path.is_file():
        raise RuntimeError(f"{name} is missing")
    try:
        document = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_unique_json_object,
        )
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        raise RuntimeError(f"{name} is not valid unique-key UTF-8 JSON") from error
    if not isinstance(document, dict):
        raise RuntimeError(f"{name} must be a JSON object")
    return document


def _write_all(descriptor: int, payload: bytes) -> None:
    offset = 0
    while offset < len(payload):
        written = os.write(descriptor, payload[offset:])
        if written <= 0:
            raise OSError("atomic marker write made no progress")
        offset += written


def _sync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY | os.O_CLOEXEC)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _write_atomic_json(path: Path, document: dict[str, object]) -> None:
    """先同步同目录临时 inode，再用硬链接排他发布不可见半成品。"""
    payload = (
        json.dumps(
            document,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n"
    ).encode("utf-8")
    temporary = path.with_name(f".{path.name}.{os.getpid()}.atomic")
    descriptor = os.open(
        temporary,
        os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_CLOEXEC,
        0o600,
    )
    try:
        _write_all(descriptor, payload)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    try:
        os.link(temporary, path)
        _sync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)
        _sync_directory(path.parent)


def _fault_reason(path: Path) -> str | None:
    if not path.exists():
        return None
    document = _read_json(path, name="fault marker")
    reason = document.get("fault")
    if set(document) != {"fault"} or not isinstance(reason, str) or not reason:
        raise RuntimeError("fault marker fields are invalid")
    return reason


def _publish_fault(path: Path, reason: str) -> None:
    if path.exists():
        _fault_reason(path)
        return
    try:
        _write_atomic_json(path, {"fault": reason})
    except FileExistsError:
        _fault_reason(path)


def _process_error(role: str, returncode: int) -> RuntimeError:
    return RuntimeError(f"{role} exited with code {returncode}")


def _wait_for_ready(
    *,
    processes: dict[str, subprocess.Popen[str]],
    simulator_ready: Path,
    command_ready: Path,
    fault_path: Path,
    simulation_session_hex: str,
    source_session_hex: str,
    descriptor_hex: str,
) -> None:
    expected = {
        simulator_ready: {
            "role": "simulator",
            "ready": True,
            "simulation_session_id": simulation_session_hex,
            "descriptor_sha256": descriptor_hex,
            "world_generation": _WORLD_GENERATION,
        },
        command_ready: {
            "role": "command_peer",
            "ready": True,
            "source_id": _COMMAND_SOURCE_ID,
            "source_session_id": source_session_hex,
            "descriptor_sha256": descriptor_hex,
        },
    }
    deadline = time.monotonic() + _STARTUP_TIMEOUT_SEC
    while True:
        reason = _fault_reason(fault_path)
        if reason is not None:
            raise RuntimeError(f"Golf startup fault: {reason}")
        for role, process in processes.items():
            returncode = process.poll()
            if returncode is not None:
                raise _process_error(role, returncode)
        if all(path.is_file() for path in expected):
            for path, expected_document in expected.items():
                document = _read_json(
                    path,
                    name=f"{expected_document['role']} ready marker",
                )
                if (
                    document != expected_document
                    or type(document.get("ready")) is not bool
                    or (
                        document.get("role") == "simulator"
                        and type(document.get("world_generation")) is not int
                    )
                ):
                    raise RuntimeError(
                        f"{expected_document['role']} ready marker fields are invalid"
                    )
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("MID-360 Golf peers did not reach the ready barrier")
        time.sleep(_POLL_INTERVAL_SEC)


def _wait_for_completion(
    processes: dict[str, subprocess.Popen[str]],
    fault_path: Path,
) -> None:
    while True:
        reason = _fault_reason(fault_path)
        if reason is not None:
            raise RuntimeError(f"MID-360 Golf session fault: {reason}")
        complete = True
        for role, process in processes.items():
            returncode = process.poll()
            if returncode is None:
                complete = False
            elif returncode != 0:
                raise _process_error(role, returncode)
        if complete:
            return
        time.sleep(_POLL_INTERVAL_SEC)


def _stop_process_group(process: subprocess.Popen[str]) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=_PROCESS_TERM_TIMEOUT_SEC)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait(timeout=_PROCESS_TERM_TIMEOUT_SEC)


def _cleanup_after_fault(processes: Sequence[subprocess.Popen[str]]) -> None:
    """先给 topic 安全停车留出有界时间，再回收各自独立的进程组。"""
    deadline = time.monotonic() + _FAULT_GRACE_SEC
    while any(process.poll() is None for process in processes):
        if time.monotonic() >= deadline:
            break
        time.sleep(_POLL_INTERVAL_SEC)
    for process in reversed(processes):
        _stop_process_group(process)


def _recorder_argv(
    recorder: Path,
    *,
    simulation_session_hex: str,
    mcap_path: Path,
    result_path: Path,
) -> list[str]:
    return [
        str(recorder),
        "--descriptor-set",
        str(_DESCRIPTOR_PATH),
        "--scene-id",
        _SCENE_ID,
        "--simulation-session-id",
        simulation_session_hex,
        "--world-generation",
        str(_WORLD_GENERATION),
        "--lidar-pattern-version",
        _PATTERN_VERSION,
        "--lidar-pattern-sha256",
        _PATTERN_SHA256,
        "--output",
        str(mcap_path),
        "--expected-wheel-command-count",
        str(_EXPECTED_COUNTS["/sim/wheel/command"]),
        "--expected-wheel-state-count",
        str(_EXPECTED_COUNTS["/sim/wheel/state"]),
        "--expected-lidar-points-count",
        str(_EXPECTED_COUNTS["/sim/lidar/points"]),
        "--expected-rtk-state-count",
        str(_EXPECTED_COUNTS["/sim/rtk/state"]),
        "--expected-imu-attitude-count",
        str(_EXPECTED_COUNTS["/sim/imu/attitude"]),
        "--deadline-ms",
        str(_RECORDER_DEADLINE_MS),
        "--result",
        str(result_path),
    ]


def _simulator_argv(
    *,
    ready_path: Path,
    start_path: Path,
    result_path: Path,
    fault_path: Path,
    simulation_session_hex: str,
    direct: bool,
) -> list[str]:
    argv = [
        sys.executable,
        "-m",
        "scripts.mid360_golf_simulation",
        "--ready-path",
        str(ready_path),
        "--start-path",
        str(start_path),
        "--result-path",
        str(result_path),
        "--fault-path",
        str(fault_path),
        "--simulation-session-id",
        simulation_session_hex,
        "--peer-timeout-sec",
        str(_PEER_TIMEOUT_SEC),
    ]
    if direct:
        argv.append("--direct")
    return argv


def _command_argv(
    *,
    ready_path: Path,
    start_path: Path,
    result_path: Path,
    fault_path: Path,
    source_session_hex: str,
) -> list[str]:
    return [
        sys.executable,
        "-m",
        "scripts.mid360_golf_command_peer",
        "--ready-path",
        str(ready_path),
        "--start-path",
        str(start_path),
        "--result-path",
        str(result_path),
        "--fault-path",
        str(fault_path),
        "--source-session-id",
        source_session_hex,
        "--peer-timeout-sec",
        str(_PEER_TIMEOUT_SEC),
    ]


def _spawn(argv: Sequence[str]) -> subprocess.Popen[str]:
    return subprocess.Popen(
        list(argv),
        cwd=str(ROOT),
        start_new_session=True,
        text=True,
    )


def _validate_simulator_result(
    document: dict[str, object],
    *,
    simulation_session_hex: str,
    descriptor_hex: str,
    direct: bool,
) -> CaptureAcceptanceMetrics:
    expected_scalars = {
        "role": "simulator",
        "clean_shutdown": True,
        "fault_reason": None,
        "simulation_session_id": simulation_session_hex,
        "descriptor_sha256": descriptor_hex,
        "world_generation": _WORLD_GENERATION,
        "command_generation": _COMMAND_GENERATION,
        "scene_id": _SCENE_ID,
        "connection_mode": "direct" if direct else "gui",
        "physics_steps": _PHYSICS_STEPS,
        "sim_duration_ns": _SIMULATION_DURATION_NS,
        "robot_model": "df_mid",
        "terrain_model": "golf_heightfield",
        "golf_seed": 41,
        "golf_relief": "medium",
        "static_obstacle_count": 6,
        "moving_obstacle_count": 3,
        "published_frames": _SIMULATOR_COUNTS,
        "expected_topic_counts": _EXPECTED_COUNTS,
        "transport_metrics": {
            "published_count": sum(_SIMULATOR_COUNTS.values()),
            "received_count": _EXPECTED_COUNTS["/sim/wheel/command"],
            "error_count": 0,
            "dropped_count": 0,
        },
    }
    if set(document) != _SIMULATOR_RESULT_KEYS or any(
        document.get(key) != value for key, value in expected_scalars.items()
    ):
        raise RuntimeError("simulator result differs from the frozen Golf contract")
    client_id = document.get("client_id")
    active_steps = document.get("active_command_steps")
    integer_fields = (
        "world_generation",
        "command_generation",
        "physics_steps",
        "sim_duration_ns",
        "golf_seed",
        "static_obstacle_count",
        "moving_obstacle_count",
    )
    metrics = document["transport_metrics"]
    if (
        type(document.get("clean_shutdown")) is not bool
        or any(type(document.get(name)) is not int for name in integer_fields)
        or type(client_id) is not int
        or client_id < 0
        or type(active_steps) is not int
        or not 0 < active_steps <= _PHYSICS_STEPS
        or not isinstance(metrics, dict)
        or any(type(value) is not int for value in metrics.values())
    ):
        raise RuntimeError("simulator result contains invalid runtime metrics")
    try:
        return parse_capture_acceptance(document.get("truth_acceptance"))
    except ValueError as error:
        raise RuntimeError("simulator truth acceptance is invalid") from error


def _validate_command_result(
    document: dict[str, object],
    *,
    source_session_hex: str,
    descriptor_hex: str,
) -> None:
    expected = {
        "role": "command_peer",
        "clean_shutdown": True,
        "fault_reason": None,
        "source_id": _COMMAND_SOURCE_ID,
        "source_session_id": source_session_hex,
        "descriptor_sha256": descriptor_hex,
        "published_frames": {
            "/sim/wheel/command": _EXPECTED_COUNTS["/sim/wheel/command"]
        },
        "last_wheel_timestamp_ns": _SIMULATION_DURATION_NS,
        "latest_pose_timestamp_ns": _SIMULATION_DURATION_NS,
        "normal_stop_started": True,
        "finished": True,
    }
    if (
        set(document) != _COMMAND_RESULT_KEYS
        or document != expected
        or type(document.get("clean_shutdown")) is not bool
        or type(document.get("last_wheel_timestamp_ns")) is not int
        or type(document.get("latest_pose_timestamp_ns")) is not int
        or type(document.get("normal_stop_started")) is not bool
        or type(document.get("finished")) is not bool
    ):
        raise RuntimeError("command peer result differs from the frozen Golf contract")


def _validate_recorder_result(
    document: dict[str, object],
    *,
    mcap_path: Path,
) -> None:
    expected = {
        "clean_shutdown": True,
        "mcap": str(mcap_path),
        "recorded_count": sum(_EXPECTED_COUNTS.values()),
        "role": "recorder",
        "topics": _EXPECTED_COUNTS,
    }
    if (
        set(document) != _RECORDER_RESULT_KEYS
        or document != expected
        or type(document.get("clean_shutdown")) is not bool
        or type(document.get("recorded_count")) is not int
    ):
        raise RuntimeError("Recorder result differs from the frozen Golf contract")


def _validate_mapping_index(
    index: MappingSessionIndex,
    *,
    simulation_session_hex: str,
    descriptor: DescriptorIdentity,
) -> None:
    identity = index.identity
    if (
        identity.simulation_session_id != bytes.fromhex(simulation_session_hex)
        or identity.descriptor_sha256 != descriptor.sha256
        or identity.world_generation != _WORLD_GENERATION
        or identity.scene_id != _SCENE_ID
        or identity.lidar_pattern_version != _PATTERN_VERSION
        or identity.lidar_pattern_sha256 != bytes.fromhex(_PATTERN_SHA256)
        or dict(index.topic_counts) != _EXPECTED_COUNTS
        or index.message_count != sum(_EXPECTED_COUNTS.values())
        or index.lidar_frame_times_ns
        != tuple(range(0, _SIMULATION_DURATION_NS, _FRAME_PERIOD_NS))
        or tuple(node.timestamp_ns for node in index.pose_nodes)
        != tuple(
            range(
                0,
                _SIMULATION_DURATION_NS + _FRAME_PERIOD_NS,
                _FRAME_PERIOD_NS,
            )
        )
    ):
        raise RuntimeError("strict MCAP index differs from the frozen Golf session")


def _file_sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _acceptance_document(
    *,
    capture: CaptureAcceptanceMetrics,
    mapping: MappingAcceptanceMetrics,
    simulation_session_hex: str,
    mcap_path: Path,
) -> dict[str, object]:
    capture_document = capture_acceptance_document(capture)
    failures = acceptance_failures(capture, mapping)
    return {
        "schema": "mid360-golf-mapping-acceptance-v2",
        "simulation_session_id": simulation_session_hex,
        "mcap": {
            "path": str(mcap_path),
            "size_bytes": mcap_path.stat().st_size,
            "sha256": _file_sha256(mcap_path),
        },
        "thresholds": dict(ACCEPTANCE_THRESHOLDS),
        "metrics": {
            "route": {
                "sample_count": mapping.route_sample_count,
                "error_p95_m": mapping.route_error_p95_m,
                "final_remaining_m": mapping.route_final_remaining_m,
            },
            "motion": capture_document["motion"],
            "terrain": {
                "eligible_cell_count": mapping.terrain_eligible_cell_count,
                "covered_cell_count": mapping.terrain_covered_cell_count,
                "coverage_ratio": mapping.terrain_coverage_ratio,
            },
            "obstacles": capture_document["obstacles"],
            "deskew": capture_document["deskew"],
            "world_map": {
                "permanent_voxel_count": mapping.permanent_voxel_count,
                "displayed_static_point_count": mapping.displayed_static_point_count,
            },
        },
        "failures": list(failures),
        "passed": not failures,
    }


def _open_mapping_replay(index: MappingSessionIndex) -> None:
    """只在所有持久化校验通过后才导入 Qt/OpenGL 并进入事件循环。"""
    from PySide6 import QtWidgets

    from slope_sim.mapping_replay_gui import (
        MappingMcapReplaySource,
        MappingReplayWindow,
    )

    application = QtWidgets.QApplication.instance()
    if application is None:
        application = QtWidgets.QApplication([sys.argv[0]])
    window = MappingReplayWindow(MappingMcapReplaySource(index))
    window.show()
    exit_code = application.exec()
    if exit_code != 0:
        raise RuntimeError(f"mapping replay exited with code {exit_code}")


def _run_mapping_replay_qa(
    *,
    index: MappingSessionIndex,
    output_dir: Path,
    simulation_session_hex: str,
) -> dict[str, object]:
    """让最终 GUI 证据严格复用本次已验收的 MCAP session。"""
    from scripts.verify_mid360_golf_mapping_replay import run_mapping_replay_qa

    return run_mapping_replay_qa(
        index=index,
        output_dir=output_dir,
        simulation_session_hex=simulation_session_hex,
    )


def run_mid360_golf_mapping(
    *,
    recorder: Path,
    exporter: Path | None = None,
    output_dir: Path,
    direct: bool = False,
    open_replay: bool = True,
    replay_qa: bool = False,
) -> dict[str, object]:
    """执行唯一固定会话；任何失败都不会把不完整 MCAP 交给回放。"""
    if (
        not isinstance(recorder, Path)
        or not isinstance(output_dir, Path)
        or (exporter is not None and not isinstance(exporter, Path))
    ):
        raise ValueError("recorder, exporter and output_dir must be Paths")
    if (
        type(direct) is not bool
        or type(open_replay) is not bool
        or type(replay_qa) is not bool
    ):
        raise ValueError("direct, open_replay and replay_qa must be bools")
    descriptor = _preflight(recorder, output_dir, exporter)
    output_dir.mkdir(mode=0o700)

    simulator_ready = output_dir / "simulator.ready.json"
    command_ready = output_dir / "command.ready.json"
    start_path = output_dir / "start.json"
    fault_path = output_dir / "fault.json"
    simulator_result = output_dir / "simulator.json"
    command_result = output_dir / "command.json"
    recorder_result = output_dir / "recorder.json"
    acceptance_result = output_dir / "acceptance.json"
    mcap_path = output_dir / "session.mcap"
    simulation_session_hex = uuid4().hex
    source_session_hex = uuid4().hex
    descriptor_hex = descriptor.sha256.hex()
    processes: dict[str, subprocess.Popen[str]] = {}
    try:
        # Recorder 必须先订阅，两个 Python peer 随后各自验证拓扑并发布 ready。
        processes["recorder"] = _spawn(
            _recorder_argv(
                recorder,
                simulation_session_hex=simulation_session_hex,
                mcap_path=mcap_path,
                result_path=recorder_result,
            )
        )
        processes["simulator"] = _spawn(
            _simulator_argv(
                ready_path=simulator_ready,
                start_path=start_path,
                result_path=simulator_result,
                fault_path=fault_path,
                simulation_session_hex=simulation_session_hex,
                direct=direct,
            )
        )
        processes["command_peer"] = _spawn(
            _command_argv(
                ready_path=command_ready,
                start_path=start_path,
                result_path=command_result,
                fault_path=fault_path,
                source_session_hex=source_session_hex,
            )
        )
        _wait_for_ready(
            processes=processes,
            simulator_ready=simulator_ready,
            command_ready=command_ready,
            fault_path=fault_path,
            simulation_session_hex=simulation_session_hex,
            source_session_hex=source_session_hex,
            descriptor_hex=descriptor_hex,
        )
        _write_atomic_json(start_path, {"start": True})
        _wait_for_completion(processes, fault_path)

        capture_metrics = _validate_simulator_result(
            _read_json(simulator_result, name="simulator result"),
            simulation_session_hex=simulation_session_hex,
            descriptor_hex=descriptor_hex,
            direct=direct,
        )
        _validate_command_result(
            _read_json(command_result, name="command peer result"),
            source_session_hex=source_session_hex,
            descriptor_hex=descriptor_hex,
        )
        _validate_recorder_result(
            _read_json(recorder_result, name="Recorder result"),
            mcap_path=mcap_path,
        )
        index = load_mapping_session(mcap_path, recorder_result)
        _validate_mapping_index(
            index,
            simulation_session_hex=simulation_session_hex,
            descriptor=descriptor,
        )
        scene = load_scene(_SCENE_PATH)
        mapping_metrics = evaluate_mapping_session(
            index,
            route=build_canonical_golf_route(_GOLF_BOUNDS),
            bounds=_GOLF_BOUNDS,
            obstacles=scene.obstacles,
        )
        acceptance_document = _acceptance_document(
            capture=capture_metrics,
            mapping=mapping_metrics,
            simulation_session_hex=simulation_session_hex,
            mcap_path=mcap_path,
        )
        _write_atomic_json(acceptance_result, acceptance_document)
        persisted_acceptance = _read_json(
            acceptance_result,
            name="mapping acceptance result",
        )
        if persisted_acceptance != acceptance_document:
            raise RuntimeError("persisted mapping acceptance result changed")
        failures = persisted_acceptance["failures"]
        if failures:
            raise RuntimeError(
                "mapping acceptance failed: " + ", ".join(failures)
            )
        lvx2_path = None
        export_result = None
        if exporter is not None:
            lvx2_path, export_result = _export_mapping_lidar(
                exporter=exporter,
                mcap_path=mcap_path,
                output_dir=output_dir,
            )
        replay_qa_result = None
        if replay_qa:
            replay_qa_result = _run_mapping_replay_qa(
                index=index,
                output_dir=output_dir / "gui-qa",
                simulation_session_hex=simulation_session_hex,
            )
            if replay_qa_result.get("passed") is not True:
                raise RuntimeError("mapping replay QA did not pass")
        elif open_replay:
            _open_mapping_replay(index)
        result = {
            "clean_shutdown": True,
            "output_dir": str(output_dir),
            "mcap": str(mcap_path),
            "acceptance": str(acceptance_result),
            "simulation_session_id": simulation_session_hex,
            "source_session_id": source_session_hex,
            "topic_counts": dict(_EXPECTED_COUNTS),
            "replay_opened": open_replay or replay_qa,
        }
        if lvx2_path is not None and export_result is not None:
            result["lvx2"] = str(lvx2_path)
            result["export"] = str(export_result)
        if replay_qa_result is not None:
            qa_path = replay_qa_result.get("qa")
            if not isinstance(qa_path, str):
                raise RuntimeError("mapping replay QA result is invalid")
            result["replay_qa"] = qa_path
        return result
    except BaseException as error:
        reason = str(error).strip() or type(error).__name__
        try:
            _publish_fault(fault_path, reason)
        finally:
            _cleanup_after_fault(tuple(processes.values()))
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recorder", required=True, type=Path)
    parser.add_argument("--exporter", type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--direct", action="store_true", help="use PyBullet DIRECT")
    parser.add_argument(
        "--no-replay",
        action="store_true",
        help="validate the MCAP without opening the 3D replay window",
    )
    parser.add_argument(
        "--replay-qa",
        action="store_true",
        help="run automated GUI replay QA against this accepted simulation session",
    )
    arguments = parser.parse_args(argv)
    try:
        run_mid360_golf_mapping(
            recorder=arguments.recorder,
            exporter=arguments.exporter,
            output_dir=arguments.output_dir,
            direct=arguments.direct,
            open_replay=not arguments.no_replay,
            replay_qa=arguments.replay_qa,
        )
    except (OSError, RuntimeError, ValueError) as error:
        print(f"MID-360 Golf mapping failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
