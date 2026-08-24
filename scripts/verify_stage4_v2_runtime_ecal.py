#!/usr/bin/env python3
"""阶段四 B2：持续五话题 raw eCAL runtime 的真实验收入口。"""
from __future__ import annotations

import argparse
from fractions import Fraction
from hashlib import sha256
import json
import math
from numbers import Real
import os
from pathlib import Path
import subprocess
import sys
from threading import Lock
import time
from typing import Callable, Sequence

from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings, RawReceivedFrame, process_raw_frame
from slope_sim.interfaces.v2.topics import V2_TOPICS


_PHYSICS_HZ = 240
_WHEEL_HZ = 100
_SENSOR_HZ = 10
_OUTPUT_TYPE_NAMES = {
    contract.topic: contract.type_name
    for contract in V2_TOPICS
    if contract.direction == "publish"
}


def expected_v2_frame_counts(*, duration_sec: object) -> dict[str, int]:
    """按正式 DIRECT runtime 的上取整物理窗口推导四个输出 topic 的精确帧数。"""
    if isinstance(duration_sec, bool) or not isinstance(duration_sec, Real):
        raise ValueError("duration_sec must be positive and finite")
    duration = float(duration_sec)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration_sec must be positive and finite")
    physics_steps = math.ceil(Fraction(str(duration)) * _PHYSICS_HZ)
    return {
        "/sim/wheel/state": physics_steps * _WHEEL_HZ // _PHYSICS_HZ,
        "/sim/lidar/points": physics_steps * _SENSOR_HZ // _PHYSICS_HZ,
        "/sim/rtk/state": physics_steps * _SENSOR_HZ // _PHYSICS_HZ,
        "/sim/imu/attitude": physics_steps * _SENSOR_HZ // _PHYSICS_HZ,
    }


def observe_command_peer_count(previous: int, current: object) -> int:
    """累计测量窗口内唯一 command peer，避免正常关闭后的 discovery 归零污染证据。"""
    if type(previous) is not int or previous not in {0, 1}:
        raise ValueError("previous command peer count must be zero or one")
    if type(current) is not int or current not in {0, 1}:
        raise ValueError("command peer count must be exactly zero or one")
    return 1 if previous == 1 or current == 1 else 0


def build_collector_command(
    *,
    descriptor_path: Path,
    ready_path: Path,
    result_path: Path,
    duration_sec: float,
    timeout_sec: float,
) -> list[str]:
    """构造独立 collector 的完整模块命令，禁止依赖相对路径或隐式默认值。"""
    paths = (descriptor_path, ready_path, result_path)
    if any(not isinstance(path, Path) for path in paths):
        raise ValueError("collector evidence paths must be Path values")
    duration = _positive_finite("duration_sec", duration_sec)
    timeout = _positive_finite("timeout_sec", timeout_sec)
    return [
        sys.executable,
        "-m",
        "scripts.verify_stage4_v2_runtime_ecal",
        "--participant",
        "collector",
        "--descriptor-path",
        str(descriptor_path.resolve()),
        "--ready-path",
        str(ready_path.resolve()),
        "--result-path",
        str(result_path.resolve()),
        "--duration-sec",
        str(duration),
        "--timeout-sec",
        str(timeout),
    ]


def build_runtime_evidence_paths(evidence_dir: Path) -> dict[str, Path]:
    """为一次真实五话题运行分配唯一且尚未创建的证据路径。"""
    if not isinstance(evidence_dir, Path):
        raise ValueError("evidence_dir must be a Path")
    root = evidence_dir.resolve()
    if root.exists():
        raise FileExistsError("runtime eCAL evidence directory already exists")
    return {
        "descriptor": root / "descriptor" / "slope_sim_interfaces_v2.desc",
        "collector_ready": root / "collector-ready.json",
        "collector_result": root / "collector-result.json",
        "runtime_result": root / "runtime-result.json",
        "process": root / "collector-process.json",
    }


def _positive_finite(name: str, value: object) -> float:
    """规范正有限时间参数，拒绝 bool、NaN、无穷与零。"""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be positive and finite")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return normalized


def require_real_desktop_environment() -> None:
    """拒绝把 Qt offscreen/minimal 或无显示会话误记为真实桌面 GUI 验收。"""
    if os.environ.get("QT_QPA_PLATFORM", "").strip().lower() in {"offscreen", "minimal"}:
        raise RuntimeError("real desktop dashboard gate rejects offscreen Qt backend")
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise RuntimeError("real desktop dashboard gate requires DISPLAY or WAYLAND_DISPLAY")


def verify_peer_evidence(evidence: object, *, duration_sec: object) -> None:
    """逐条复算 peer 的 raw 接收证据，拒绝数量、时序或输出身份不一致。"""
    if type(evidence) is not dict:
        raise ValueError("peer evidence must be an object")
    if evidence.get("command_peer_count") != 1:
        raise ValueError("command peer count must be exactly one")
    if evidence.get("callback_errors") != 0:
        raise ValueError("peer callback errors must be zero")
    if evidence.get("clean_shutdown") is not True:
        raise ValueError("peer must report clean shutdown")
    outputs = evidence.get("outputs")
    expected_counts = expected_v2_frame_counts(duration_sec=duration_sec)
    if type(outputs) is not dict or set(outputs) != set(expected_counts):
        raise ValueError("peer outputs must cover exactly the four v2 output topics")

    all_identities: set[tuple[str, str, int]] = set()
    for topic, expected_count in expected_counts.items():
        item = outputs[topic]
        if type(item) is not dict or item.get("count") != expected_count:
            raise ValueError(f"{topic} count does not match the runtime window")
        sequences = item.get("sequences")
        timestamps = item.get("timestamps_ns")
        identities = item.get("identities")
        if (
            type(sequences) is not list
            or type(timestamps) is not list
            or type(identities) is not list
            or len(sequences) != expected_count
            or len(timestamps) != expected_count
            or len(identities) != expected_count
        ):
            raise ValueError(f"{topic} evidence length does not match count")
        if sequences != list(range(expected_count)):
            raise ValueError(f"{topic} sequence is not contiguous from zero")
        interval_ns = 10_000_000 if topic == "/sim/wheel/state" else 100_000_000
        if timestamps != [index * interval_ns for index in range(1, expected_count + 1)]:
            raise ValueError(f"{topic} timestamps do not match its cadence")
        for identity in identities:
            if (
                type(identity) is not list
                or len(identity) != 3
                or type(identity[0]) is not str
                or len(identity[0]) != 32
                or type(identity[1]) is not str
                or len(identity[1]) != 64
                or type(identity[2]) is not int
                or identity[2] < 0
            ):
                raise ValueError(f"{topic} contains an invalid v2 output identity")
            all_identities.add((identity[0], identity[1], identity[2]))
    if len(all_identities) != 1:
        raise ValueError("all v2 outputs must share one session/descriptor/world identity")


def verify_runtime_evidence(runtime: object, *, duration_sec: object) -> None:
    """复核 Simulator 端的实时、零 drop 与关闭门，不能只信 collector 帧数。"""
    duration = _positive_finite("duration_sec", duration_sec)
    if type(runtime) is not dict:
        raise ValueError("runtime evidence must be an object")
    expected_frames = expected_v2_frame_counts(duration_sec=duration)
    expected_steps = math.ceil(duration * _PHYSICS_HZ)
    if runtime.get("published_frames") != expected_frames:
        raise ValueError("runtime published frames do not match the window")
    if runtime.get("physics_steps") != expected_steps:
        raise ValueError("runtime physics steps do not match the window")
    if runtime.get("sim_duration_sec") != expected_steps / _PHYSICS_HZ:
        raise ValueError("runtime simulation duration does not match physics steps")
    wall = runtime.get("wall_duration_sec")
    if not isinstance(wall, Real) or not duration <= wall <= duration * 1.1:
        raise ValueError("runtime wall pacing does not match the window")
    metrics = runtime.get("transport_metrics")
    if type(metrics) is not dict or metrics.get("published_count") != sum(expected_frames.values()):
        raise ValueError("runtime transport published count does not match frames")
    if metrics.get("error_count") != 0 or metrics.get("dropped_count") != 0:
        raise ValueError("runtime transport error or dropped count is nonzero")
    if runtime.get("clean_shutdown") is not True or runtime.get("lidar_worker", {}).get("clean_shutdown") is not True:
        raise ValueError("runtime or lidar worker did not cleanly shut down")


class RawV2OutputCollector:
    """持续 peer 的 callback 汇集器，只保存已验证 v2 输出的审计字段。"""

    def __init__(self, descriptor: DescriptorIdentity) -> None:
        if not isinstance(descriptor, DescriptorIdentity):
            raise ValueError("descriptor must be a DescriptorIdentity")
        self._descriptor = descriptor
        self._codec = V2ProtoCodec(descriptor)
        self._lock = Lock()
        self._records: dict[str, list[tuple[int, int, float, list[object], dict[str, object]]]] = {
            topic: [] for topic in _OUTPUT_TYPE_NAMES
        }

    def _parser_for_topic(self, topic: str):
        """将 topic 唯一映射到冻结 codec 的对应 decoder。"""
        parsers = {
            "/sim/wheel/state": self._codec.decode_wheel_state,
            "/sim/lidar/points": self._codec.decode_lidar_point_cloud,
            "/sim/rtk/state": self._codec.decode_rtk_state,
            "/sim/imu/attitude": self._codec.decode_imu_attitude,
        }
        return parsers[topic]

    def record(self, topic: str, frame: RawReceivedFrame) -> None:
        """验证一个 owned raw frame，并记录其 codec 还原的时序与绑定身份。"""
        expected_type = _OUTPUT_TYPE_NAMES.get(topic)
        if expected_type is None:
            raise ValueError("topic is not a v2 runtime output")
        processed = process_raw_frame(
            frame,
            expected_type=expected_type,
            descriptor=self._descriptor,
            parser=self._parser_for_topic(topic),
        )
        model = processed.parsed
        timestamp_ns = (
            model.timebase_ns if topic == "/sim/lidar/points" else model.timestamp_ns
        )
        identity = [
            model.simulation_session_id.hex(),
            model.descriptor_sha256.hex(),
            model.world_generation,
        ]
        publisher = {
            "entity_id": frame.remote_publisher_entity_id,
            "process_id": frame.remote_publisher_process_id,
            "host_name": frame.remote_publisher_host_name,
        }
        with self._lock:
            self._records[topic].append(
                (model.sequence, timestamp_ns, frame.received_at, identity, publisher)
            )

    def output_evidence(self, topic: str) -> dict[str, object]:
        """返回一个 topic 的独立 JSON 兼容记录，不泄露内部可变列表。"""
        if topic not in _OUTPUT_TYPE_NAMES:
            raise ValueError("topic is not a v2 runtime output")
        with self._lock:
            records = tuple(self._records[topic])
        return {
            "count": len(records),
            "sequences": [record[0] for record in records],
            "timestamps_ns": [record[1] for record in records],
            "received_at_sec": [record[2] for record in records],
            "identities": [list(record[3]) for record in records],
            "publishers": [dict(record[4]) for record in records],
        }


def _write_new_json(path: Path, value: object) -> None:
    """以排他创建写入 participant 证据，拒绝覆盖任意旧运行结果。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as handle:
        json.dump(value, handle, sort_keys=True)
        handle.write("\n")


def run_collector_participant(
    *,
    descriptor_path: Path,
    ready_path: Path,
    result_path: Path,
    duration_sec: float,
    timeout_sec: float,
) -> int:
    """运行持续 raw eCAL collector，等待正式窗口的全部四类 v2 输出。"""
    if not all(isinstance(path, Path) for path in (descriptor_path, ready_path, result_path)):
        raise ValueError("collector evidence paths must be Path values")
    duration = _positive_finite("duration_sec", duration_sec)
    timeout = _positive_finite("timeout_sec", timeout_sec)
    descriptor_bytes = descriptor_path.read_bytes()
    descriptor = DescriptorIdentity(descriptor_bytes, sha256(descriptor_bytes).digest())
    bindings = EcalRawBindings()
    core = bindings._core
    initialize = getattr(core, "initialize", None)
    if not callable(initialize) or initialize(f"stage4-v2-runtime-peer-{os.getpid()}", 0x3F) is False:
        raise RuntimeError("eCAL collector initialize returned False")
    collector = RawV2OutputCollector(descriptor)
    callback_errors: list[str] = []
    error_lock = Lock()
    subscribers: list[object] = []
    finalized = False
    try:
        for topic, type_name in _OUTPUT_TYPE_NAMES.items():
            def receive(frame: RawReceivedFrame, *, captured_topic: str = topic) -> None:
                try:
                    collector.record(captured_topic, frame)
                except (TypeError, ValueError, RuntimeError) as error:
                    with error_lock:
                        callback_errors.append(f"{captured_topic}: {type(error).__name__}")

            subscribers.append(
                bindings.create_subscriber(topic, type_name, descriptor, receive)
            )
        command_publisher = bindings.create_publisher(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            descriptor,
        )
        _write_new_json(
            ready_path,
            {
                "command_publisher": True,
                "output_topics": sorted(_OUTPUT_TYPE_NAMES),
                "pid": os.getpid(),
            },
        )
        expected = expected_v2_frame_counts(duration_sec=duration)
        deadline = time.monotonic() + timeout
        command_peer_count = 0
        while time.monotonic() < deadline:
            command_peer_count = observe_command_peer_count(
                command_peer_count,
                command_publisher.get_subscriber_count(),
            )
            with error_lock:
                if callback_errors:
                    break
            if all(
                collector.output_evidence(topic)["count"] == count
                for topic, count in expected.items()
            ):
                break
            time.sleep(0.002)
        outputs = {topic: collector.output_evidence(topic) for topic in expected}
        with error_lock:
            errors = tuple(callback_errors)
        evidence = {
            "callback_errors": len(errors),
            "callback_error_details": list(errors),
            "clean_shutdown": False,
            "command_peer_count": command_peer_count,
            "outputs": outputs,
        }
    finally:
        for subscriber in subscribers:
            remove_callback = getattr(subscriber, "remove_receive_callback", None)
            if callable(remove_callback):
                remove_callback()
        finalize = getattr(core, "finalize", None)
        if not callable(finalize) or finalize() is False:
            raise RuntimeError("eCAL collector finalize returned False")
        finalized = True
    evidence["clean_shutdown"] = finalized
    _write_new_json(result_path, evidence)
    verify_peer_evidence(evidence, duration_sec=duration)
    return 0


def _wait_for_collector_ready(
    process: subprocess.Popen[str],
    ready_path: Path,
    *,
    timeout_sec: float,
) -> None:
    """等待 child 已持有四个 subscriber 与 command publisher，提前退出立即失败。"""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if ready_path.is_file():
            ready = json.loads(ready_path.read_text(encoding="utf-8"))
            if ready.get("command_publisher") is not True or ready.get("output_topics") != sorted(_OUTPUT_TYPE_NAMES):
                raise RuntimeError("collector ready evidence is incomplete")
            return
        if process.poll() is not None:
            stderr = process.stderr.read() if process.stderr is not None else ""
            raise RuntimeError(f"collector exited before ready: {stderr.strip()}")
        time.sleep(0.01)
    raise TimeoutError("collector did not become ready")


def _run_v2_ecal_gate(
    *,
    evidence_dir: Path,
    duration_sec: float,
    robot_model: str = "df_mid",
    peer_timeout_sec: float = 20.0,
    runtime_runner: Callable[..., dict[str, object]],
) -> dict[str, object]:
    """复用唯一 collector 运行任一正式 eCAL runtime，并复核两端证据。"""
    duration = _positive_finite("duration_sec", duration_sec)
    timeout = _positive_finite("peer_timeout_sec", peer_timeout_sec)
    if not isinstance(evidence_dir, Path):
        raise ValueError("evidence_dir must be a Path")
    paths = build_runtime_evidence_paths(evidence_dir)
    root = evidence_dir.resolve()
    root.mkdir(parents=True)
    descriptor = __import__("slope_sim.interfaces.v2.descriptor", fromlist=["load_v2_descriptor"]).load_v2_descriptor()
    paths["descriptor"].parent.mkdir(parents=True)
    with paths["descriptor"].open("xb") as handle:
        handle.write(descriptor.serialized_file_descriptor_set)
    environment = os.environ.copy()
    environment.pop("STAGE4_ECAL_TEST_SHIM", None)
    environment.pop("LD_PRELOAD", None)
    command = build_collector_command(
        descriptor_path=paths["descriptor"],
        ready_path=paths["collector_ready"],
        result_path=paths["collector_result"],
        duration_sec=duration,
        timeout_sec=timeout,
    )
    process = subprocess.Popen(
        command,
        cwd=str(Path(__file__).resolve().parents[1]),
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        _wait_for_collector_ready(process, paths["collector_ready"], timeout_sec=timeout)
        from slope_sim.interfaces.v2.transport import create_v2_ecal_transport

        runtime = runtime_runner(
            result_json=paths["runtime_result"],
            duration_sec=duration,
            robot_model=robot_model,
            transport_factory=lambda runtime_descriptor: create_v2_ecal_transport(
                descriptor=runtime_descriptor,
                participant_name=f"stage4-v2-runtime-{os.getpid()}",
            ),
            peer_timeout_sec=timeout,
        )
        try:
            returncode = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired as error:
            process.terminate()
            process.wait(timeout=5)
            raise TimeoutError("collector did not finish after runtime shutdown") from error
        stdout = process.stdout.read() if process.stdout is not None else ""
        stderr = process.stderr.read() if process.stderr is not None else ""
        _write_new_json(
            paths["process"],
            {
                "returncode": returncode,
                "stderr_sha256": sha256(stderr.encode()).hexdigest(),
                "stdout_sha256": sha256(stdout.encode()).hexdigest(),
            },
        )
        if returncode != 0:
            raise RuntimeError(f"collector exited nonzero: {stderr.strip()}")
        collector = json.loads(paths["collector_result"].read_text(encoding="utf-8"))
        verify_peer_evidence(collector, duration_sec=duration)
        verify_runtime_evidence(runtime, duration_sec=duration)
        return {"evidence_dir": str(root), "runtime": runtime, "collector": collector}
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def run_v2_runtime_ecal_gate(
    *,
    evidence_dir: Path,
    duration_sec: float,
    robot_model: str = "df_mid",
    peer_timeout_sec: float = 20.0,
) -> dict[str, object]:
    """启动唯一 collector 与正式 headless runtime，并复核五话题 raw eCAL 证据。"""
    from scripts.stage4_v2_simulation_runtime import run_v2_simulation_runtime

    def run_headless_runtime(**kwargs: object) -> dict[str, object]:
        """保留 headless runtime 的 verified peer 与关闭排空合同。"""
        return run_v2_simulation_runtime(require_verified_peers=True, **kwargs)

    return _run_v2_ecal_gate(
        evidence_dir=evidence_dir,
        duration_sec=duration_sec,
        robot_model=robot_model,
        peer_timeout_sec=peer_timeout_sec,
        runtime_runner=run_headless_runtime,
    )


def run_v2_dashboard_ecal_gate(
    *,
    evidence_dir: Path,
    duration_sec: float,
    robot_model: str = "df_mid",
    peer_timeout_sec: float = 20.0,
    screenshot_png: Path,
) -> dict[str, object]:
    """在真实桌面展示已验证的五话题 eCAL runtime，并保存完整遥测截图。"""
    require_real_desktop_environment()
    if not isinstance(screenshot_png, Path):
        raise ValueError("screenshot_png must be a Path")
    from scripts.stage4_v2_dashboard import run_v2_dashboard_session

    def run_dashboard_runtime(**kwargs: object) -> dict[str, object]:
        """仅替换 Qt 展示层；底层仍由共用 collector gate 验证 raw eCAL。"""
        return run_v2_dashboard_session(
            result_json=kwargs["result_json"],
            duration_sec=kwargs["duration_sec"],
            robot_model=kwargs["robot_model"],
            peer_timeout_sec=kwargs["peer_timeout_sec"],
            screenshot_png=screenshot_png,
            isolate_runtime=True,
        )

    return _run_v2_ecal_gate(
        evidence_dir=evidence_dir,
        duration_sec=duration_sec,
        robot_model=robot_model,
        peer_timeout_sec=peer_timeout_sec,
        runtime_runner=run_dashboard_runtime,
    )


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析内部 collector CLI；父 gate 才负责启动 PyBullet runtime。"""
    parser = argparse.ArgumentParser(description="Collect Stage 4 v2 eCAL runtime frames")
    parser.add_argument("--participant", choices=("collector",), required=True)
    parser.add_argument("--descriptor-path", type=Path, required=True)
    parser.add_argument("--ready-path", type=Path, required=True)
    parser.add_argument("--result-path", type=Path, required=True)
    parser.add_argument("--duration-sec", type=float, required=True)
    parser.add_argument("--timeout-sec", type=float, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """以非零退出码暴露 collector 任何 discovery、callback 或证据故障。"""
    args = _parse_args(argv)
    try:
        return run_collector_participant(
            descriptor_path=args.descriptor_path,
            ready_path=args.ready_path,
            result_path=args.result_path,
            duration_sec=args.duration_sec,
            timeout_sec=args.timeout_sec,
        )
    except (OSError, RuntimeError, TypeError, ValueError) as error:
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
