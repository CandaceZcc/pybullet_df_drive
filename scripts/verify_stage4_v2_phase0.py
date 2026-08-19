"""阶段四 A：验证真实 eCAL Phase-0 汇总结果，不创建 participant。"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from hashlib import sha256
import json
import os
from pathlib import Path
import shutil
from subprocess import CompletedProcess, Popen, PIPE, TimeoutExpired
import subprocess
import sys
from time import monotonic, sleep
from typing import Callable, NoReturn

from google.protobuf import descriptor_pb2

from slope_sim.interfaces.generated import slope_sim_interfaces_pb2 as v1_pb
from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity, load_v2_descriptor
from slope_sim.interfaces.v2.ecal_raw import (
    EcalRawBindings,
    ProtocolVerificationState,
    process_raw_frame,
)
from slope_sim.interfaces.v2.models import WheelCommandV2
from slope_sim.interfaces.v2.session import ProtocolSession


SCENARIOS = (
    "python_to_python_raw",
    "python_to_cpp_raw",
    "cpp_to_python_raw",
    "v1_v2_same_topic_conflict",
)
SUCCESS_SCENARIOS = SCENARIOS[:3]
_WORKER_ORDER = [
    "payload_sha256",
    "remote_metadata_verified",
    "protobuf_parsed",
    "in_band_identity_validated",
]
_ECAL_INIT_ALL_COMPONENTS = 0x3F


def build_phase0_runtime_paths(evidence_dir: Path) -> dict[str, dict[str, Path]]:
    """为本次 Phase-0 运行分配绝对且互不复用的证据目标路径。"""
    root = evidence_dir.resolve()
    if root.exists():
        raise ValueError("Phase-0 evidence directory must not already exist")
    paths: dict[str, dict[str, Path]] = {
        "descriptor": {"copy": root / "descriptor" / "slope_sim_interfaces_v2.desc"},
    }
    for scenario in SCENARIOS:
        scenario_root = root / scenario
        paths[scenario] = {
            "send_payload": scenario_root / "send.bin",
            "receive_payload": scenario_root / "received.bin",
            "publisher_result": scenario_root / "publisher.json",
            "subscriber_result": scenario_root / "subscriber.json",
            "publisher_stdout": scenario_root / "publisher.stdout",
            "publisher_stderr": scenario_root / "publisher.stderr",
            "subscriber_stdout": scenario_root / "subscriber.stdout",
            "subscriber_stderr": scenario_root / "subscriber.stderr",
            "publisher_process": scenario_root / "publisher-process.json",
            "subscriber_process": scenario_root / "subscriber-process.json",
            "scenario_result": scenario_root / "scenario.json",
        }
    return paths


def build_python_participant_command(
    *,
    role: str,
    topic: str,
    type_name: str,
    descriptor_set: Path,
    payload: Path,
    result: Path,
    deadline_ms: int,
) -> list[str]:
    """构造 Python raw participant 的完整子进程参数，不依赖隐式默认值。"""
    if role not in {"publish", "subscribe", "announce"}:
        raise ValueError("role must be publish, subscribe, or announce")
    if not all(isinstance(value, str) and value for value in (topic, type_name)):
        raise ValueError("topic and type_name must be nonempty")
    if isinstance(deadline_ms, bool) or not isinstance(deadline_ms, int) or not 1 <= deadline_ms <= 60000:
        raise ValueError("deadline_ms must be in 1..60000")
    return [
        "--participant", role,
        "--topic", topic,
        "--type-name", type_name,
        "--descriptor-set", str(descriptor_set.resolve()),
        "--payload", str(payload.resolve()),
        "--result", str(result.resolve()),
        "--deadline-ms", str(deadline_ms),
    ]


def build_python_participant_process_command(**kwargs: object) -> list[str]:
    """以模块入口启动 participant，保留仓库根在 Python 导入路径中。"""
    return [
        sys.executable,
        "-m",
        "scripts.verify_stage4_v2_phase0",
        *build_python_participant_command(**kwargs),
    ]


def initialize_phase0_core(core: object, name: str) -> None:
    """以 eCAL Init::All 初始化，确保 Phase-0 monitoring API 可用。"""
    initialize = getattr(core, "initialize", None)
    if not callable(initialize) or initialize(name, _ECAL_INIT_ALL_COMPONENTS) is False:
        raise RuntimeError("eCAL core.initialize returned False")


def build_phase0_child_environment(
    parent_environment: dict[str, str],
    dependency_prefix: Path,
) -> dict[str, str]:
    """为真实 participant 固定私有 eCAL 配置和 time plugin 搜索目录。"""
    prefix = dependency_prefix.resolve()
    data_dir = prefix / "etc" / "ecal"
    plugin_dir = prefix / "lib"
    if not (data_dir / "ecal.yaml").is_file() or not plugin_dir.is_dir():
        raise RuntimeError("frozen eCAL prefix is incomplete")
    environment = dict(parent_environment)
    environment["ECAL_DATA"] = str(data_dir)
    environment["ECAL_TIME_PLUGIN_PATH"] = str(plugin_dir)
    environment.pop("STAGE4_ECAL_TEST_SHIM", None)
    environment.pop("LD_PRELOAD", None)
    return environment


def _read_evidence_object(path: Path, label: str) -> dict[str, object]:
    """读取 participant 的落盘 JSON，拒绝汇总结果替代原始进程证据。"""
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise AssertionError(f"{label} evidence is unavailable") from error
    assert type(value) is dict, f"{label} evidence must be a JSON object"
    return value


def _verify_success_evidence(
    *,
    evidence_dir: Path,
    name: str,
    scenario: dict[str, object],
    publisher: dict[str, object],
    subscriber: dict[str, object],
    expected_descriptor_sha256: str,
) -> None:
    """将成功场景汇总绑定到本次 descriptor、原始帧和 participant 文件。"""
    scenario_dir = evidence_dir / name
    assert _read_evidence_object(scenario_dir / "publisher.json", f"{name} publisher") == publisher
    assert _read_evidence_object(scenario_dir / "subscriber.json", f"{name} subscriber") == subscriber
    assert _read_evidence_object(scenario_dir / "scenario.json", f"{name} scenario") == scenario
    try:
        sent = (scenario_dir / "send.bin").read_bytes()
        received = (scenario_dir / "received.bin").read_bytes()
    except OSError as error:
        raise AssertionError(f"{name} raw payload evidence is unavailable") from error
    assert sent == received, f"{name} received payload differs from sent payload"
    payload_sha256 = sha256(sent).hexdigest()
    assert publisher.get("payload_sha256") == payload_sha256, (
        f"{name} publisher payload SHA-256 differs from send bytes"
    )
    assert subscriber.get("payload_sha256") == payload_sha256, (
        f"{name} subscriber payload SHA-256 differs from received bytes"
    )
    assert publisher.get("descriptor_sha256") == expected_descriptor_sha256, (
        f"{name} publisher descriptor SHA-256 differs from descriptor bytes"
    )
    assert subscriber.get("descriptor_sha256") == expected_descriptor_sha256, (
        f"{name} subscriber descriptor SHA-256 differs from descriptor bytes"
    )


def _verify_conflict_evidence(
    *,
    evidence_dir: Path,
    conflict: dict[str, object],
) -> None:
    """将同 topic 冲突汇总绑定到 announce、subscriber 与场景原始证据。"""
    name = "v1_v2_same_topic_conflict"
    scenario_dir = evidence_dir / name
    publisher = _read_evidence_object(scenario_dir / "publisher.json", f"{name} publisher")
    assert publisher == {
        "clean_shutdown": True,
        "finalized": True,
        "mode": "announce",
    }, f"{name} publisher evidence differs from required announcer result"
    expected_subscriber = dict(conflict)
    del expected_subscriber["exit_code"]
    assert _read_evidence_object(
        scenario_dir / "subscriber.json", f"{name} subscriber"
    ) == expected_subscriber, f"{name} conflict subscriber evidence differs from summary"
    assert _read_evidence_object(
        scenario_dir / "scenario.json", f"{name} scenario"
    ) == conflict, f"{name} conflict scenario evidence differs from summary"
    publisher_returncode = _verify_process_evidence(
        process_path=scenario_dir / "publisher-process.json",
        stdout_path=scenario_dir / "publisher.stdout",
        stderr_path=scenario_dir / "publisher.stderr",
        label=f"{name} publisher process",
    )
    subscriber_returncode = _verify_process_evidence(
        process_path=scenario_dir / "subscriber-process.json",
        stdout_path=scenario_dir / "subscriber.stdout",
        stderr_path=scenario_dir / "subscriber.stderr",
        label=f"{name} subscriber process",
    )
    assert publisher_returncode == 0, f"{name} publisher process must exit zero"
    assert subscriber_returncode != 0, f"{name} conflict subscriber process must exit nonzero"
    assert conflict.get("exit_code") == subscriber_returncode, (
        f"{name} conflict exit_code differs from subscriber process evidence"
    )


def _verify_process_evidence(
    *,
    process_path: Path,
    stdout_path: Path,
    stderr_path: Path,
    label: str,
) -> int:
    """回读 parent 排他写入的退出状态，并重算关联 stdout/stderr 的摘要。"""
    document = _read_evidence_object(process_path, label)
    assert set(document) == {"returncode", "stderr_sha256", "stdout_sha256"}, (
        f"{label} fields differ from process evidence contract"
    )
    returncode = document["returncode"]
    assert isinstance(returncode, int) and not isinstance(returncode, bool), (
        f"{label} returncode must be an integer"
    )
    for path, field in ((stdout_path, "stdout_sha256"), (stderr_path, "stderr_sha256")):
        try:
            payload = path.read_bytes()
        except OSError as error:
            raise AssertionError(f"{label} log evidence is unavailable") from error
        assert document[field] == sha256(payload).hexdigest(), (
            f"{label} {field} differs from log bytes"
        )
    return returncode


def verify_phase0_result(result: dict[str, object]) -> None:
    """严格验证四个 Phase-0 场景，不接受本地声明替代远端证据。"""
    assert type(result) is dict
    assert result.get("topic") == "/sim/wheel/command"
    expected_type = "slope_sim.interfaces.v2.WheelCommand"
    assert result.get("expected_type") == expected_type
    assert result.get("expected_encoding") == "proto"
    descriptor = result.get("expected_descriptor_sha256")
    assert isinstance(descriptor, str) and len(descriptor) == 64
    scenarios = result.get("scenarios")
    assert type(scenarios) is dict and set(scenarios) == set(SCENARIOS)

    conflict = scenarios["v1_v2_same_topic_conflict"]
    assert type(conflict) is dict
    assert conflict.get("protocol_state") == "conflict"
    assert conflict.get("accepted_count") == 0
    assert conflict.get("exit_code") != 0, "v1 same-topic peer must hard fail"
    assert conflict.get("clean_shutdown") is True
    assert conflict.get("finalized") is True

    for name in SUCCESS_SCENARIOS:
        scenario = scenarios[name]
        assert type(scenario) is dict
        publisher = scenario.get("publisher")
        subscriber = scenario.get("subscriber")
        assert type(publisher) is dict and type(subscriber) is dict
        assert publisher.get("payload_sha256") == subscriber.get("payload_sha256"), (
            f"payload SHA-256 mismatch in {name}"
        )
        assert publisher.get("descriptor_sha256") == subscriber.get("descriptor_sha256")
        assert subscriber.get("remote_type_name") == expected_type
        assert subscriber.get("remote_encoding") == "proto"
        assert subscriber.get("remote_descriptor_sha256") == descriptor, (
            f"remote descriptor SHA-256 mismatch in {name}"
        )
        assert subscriber.get("peer_count") == 1
        assert subscriber.get("protocol_state") == "verified"
        assert subscriber.get("worker_order") == _WORKER_ORDER
        assert scenario.get("clean_shutdown") is True
        assert scenario.get("finalized") is True

    evidence_value = result.get("evidence_dir")
    assert isinstance(evidence_value, str) and evidence_value, "Phase-0 evidence_dir is required"
    evidence_dir = Path(evidence_value)
    assert evidence_dir.is_absolute(), "Phase-0 evidence_dir must be absolute"
    frozen_descriptor = load_v2_descriptor()
    assert descriptor == frozen_descriptor.sha256.hex(), "result descriptor differs from frozen v2 descriptor"
    try:
        copied_descriptor = (evidence_dir / "descriptor" / "slope_sim_interfaces_v2.desc").read_bytes()
    except OSError as error:
        raise AssertionError("Phase-0 descriptor evidence is unavailable") from error
    assert sha256(copied_descriptor).hexdigest() == descriptor, (
        "Phase-0 descriptor evidence differs from result descriptor"
    )
    _verify_conflict_evidence(evidence_dir=evidence_dir, conflict=conflict)

    for name in SUCCESS_SCENARIOS:
        scenario = scenarios[name]
        assert type(scenario) is dict
        publisher = scenario.get("publisher")
        subscriber = scenario.get("subscriber")
        assert type(publisher) is dict and type(subscriber) is dict
        _verify_success_evidence(
            evidence_dir=evidence_dir,
            name=name,
            scenario=scenario,
            publisher=publisher,
            subscriber=subscriber,
            expected_descriptor_sha256=descriptor,
        )


def build_cpp_probe_commands(
    *,
    probe: str,
    role: str,
    topic: str,
    type_name: str,
    descriptor_set: str,
    result: str,
    deadline_ms: int,
    payload: str | None = None,
    payload_out: str | None = None,
    expected_peer_count: int | None = None,
) -> tuple[list[str], list[str]]:
    """按 Task 9 已冻结的字段顺序构造 dry-run 与真实 C++ probe argv。"""
    if role not in {"publish", "subscribe"}:
        raise ValueError("role must be publish or subscribe")
    if not all(isinstance(value, str) and value for value in (probe, topic, type_name, descriptor_set, result)):
        raise ValueError("probe, topic, type_name, descriptor_set, and result must be nonempty")
    if isinstance(deadline_ms, bool) or not isinstance(deadline_ms, int) or not 1 <= deadline_ms <= 60000:
        raise ValueError("deadline_ms must be in 1..60000")
    command = [
        probe,
        role,
        "--topic",
        topic,
        "--type-name",
        type_name,
        "--descriptor-set",
        descriptor_set,
    ]
    if role == "publish":
        if not isinstance(payload, str) or not payload or payload_out is not None or expected_peer_count is not None:
            raise ValueError("publish requires only payload")
        command.extend(["--payload", payload])
    else:
        if not isinstance(payload_out, str) or not payload_out or payload is not None or expected_peer_count != 1:
            raise ValueError("subscribe requires payload_out and exact peer count 1")
        command.extend(["--payload-out", payload_out, "--expected-peer-count", "1"])
    command.extend(["--result", result, "--deadline-ms", str(deadline_ms)])
    return ([probe, "--dry-run", *command[1:]], command)


def run_cpp_probe_commands(
    commands: tuple[list[str], list[str]],
    *,
    runner: Callable[[list[str]], CompletedProcess],
) -> CompletedProcess:
    """严格先运行 dry-run；其失败时绝不执行同一角色的真实命令。"""
    dry_run, real = commands
    dry_result = runner(list(dry_run))
    if dry_result.returncode != 0:
        raise RuntimeError("probe dry-run failed")
    real_result = runner(list(real))
    if real_result.returncode != 0:
        raise RuntimeError("probe real command failed")
    return real_result


def _write_new_json(path: Path, value: object) -> None:
    """以稳定 JSON 写入从未存在过的本次运行证据。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n"
    with path.open("x", encoding="utf-8") as stream:
        stream.write(payload)


def _read_result(path: Path, label: str) -> dict[str, object]:
    """读取 participant 写出的单一 JSON 结果，拒绝缺失或非对象内容。"""
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"{label} result is unavailable") from error
    if type(value) is not dict:
        raise RuntimeError(f"{label} result must be a JSON object")
    return value


def _wait_exact_count(count: Callable[[], object], deadline_ms: int) -> int:
    """等待精确一个远端 peer；0 和大于 1 都不能作为通过条件。"""
    deadline = monotonic() + deadline_ms / 1000
    while monotonic() < deadline:
        current = count()
        if type(current) is int and current == 1:
            return current
        if type(current) is int and current > 1:
            raise RuntimeError("raw eCAL peer count exceeds one")
        sleep(0.01)
    current = count()
    if type(current) is int and current == 1:
        return current
    raise RuntimeError("raw eCAL peer count did not reach exactly one")


def _run_python_participant(
    *,
    role: str,
    topic: str,
    type_name: str,
    descriptor_set: Path,
    payload: Path,
    result: Path,
    deadline_ms: int,
) -> int:
    """在独立进程执行一个 Python raw eCAL participant，并只输出本帧证据。"""
    descriptor_bytes = descriptor_set.read_bytes()
    descriptor = DescriptorIdentity(descriptor_bytes, sha256(descriptor_bytes).digest())
    bindings = EcalRawBindings()
    core = bindings._core
    initialize_phase0_core(core, f"stage4-phase0-python-{role}-{os.getpid()}")
    subscriber: object | None = None
    try:
        if role == "publish":
            publisher = bindings.create_publisher(topic, type_name, descriptor)
            peer_count = _wait_exact_count(publisher.get_subscriber_count, deadline_ms)
            wire = payload.read_bytes()
            bindings.send(publisher, wire)
            output = {
                "clean_shutdown": True,
                "descriptor_sha256": descriptor.sha256.hex(),
                "mode": "publish",
                "payload_sha256": sha256(wire).hexdigest(),
                "peer_count": peer_count,
            }
            exit_code = 0
        elif role == "announce":
            # 冲突场景只需让远端 monitoring 观察到 v1 metadata，禁止投递 payload。
            publisher = bindings.create_publisher(topic, type_name, descriptor)
            sleep(deadline_ms / 1000)
            del publisher
            output = {"clean_shutdown": True, "mode": "announce"}
            exit_code = 0
        else:
            received = []
            subscriber = bindings.create_subscriber(topic, type_name, descriptor, received.append)
            deadline = monotonic() + deadline_ms / 1000
            snapshot = None
            while monotonic() < deadline:
                peer_count = subscriber.get_publisher_count()
                snapshot = bindings.snapshot_remote_endpoints(
                    topic=topic,
                    remote_direction="publisher",
                    peer_count=peer_count,
                    expected_type=type_name,
                    descriptor=descriptor,
                )
                if snapshot.verification.state is ProtocolVerificationState.CONFLICT:
                    output = {
                        "accepted_count": 0,
                        "clean_shutdown": True,
                        "detail": snapshot.verification.detail,
                        "finalized": True,
                        "protocol_state": "conflict",
                    }
                    _write_new_json(result, output)
                    return 1
                if snapshot.verification.state is ProtocolVerificationState.VERIFIED:
                    break
                sleep(0.01)
            else:
                raise RuntimeError("raw eCAL monitoring did not verify one matching peer")
            assert snapshot is not None
            while not received and monotonic() < deadline:
                sleep(0.01)
            if not received:
                raise RuntimeError("raw eCAL Python subscriber did not receive a frame")
            frame = received.pop(0)

            def parse_wheel_command(wire: bytes) -> object:
                message = __import__(
                    "slope_sim.interfaces.generated.slope_sim_interfaces_v2_pb2",
                    fromlist=["WheelCommand"],
                ).WheelCommand()
                if not message.ParseFromString(wire):
                    raise ValueError("raw payload protobuf parse failed")
                if bytes(message.descriptor_sha256) != descriptor.sha256 or len(message.simulation_session_id) != 16:
                    raise ValueError("raw payload in-band identity mismatch")
                return message

            processed = process_raw_frame(
                frame,
                expected_type=type_name,
                descriptor=descriptor,
                parser=parse_wheel_command,
            )
            payload.parent.mkdir(parents=True, exist_ok=True)
            payload.write_bytes(frame.payload)
            output = {
                "clean_shutdown": True,
                "descriptor_sha256": descriptor.sha256.hex(),
                "mode": "subscribe",
                "payload_sha256": processed.payload_sha256.hex(),
                "peer_count": snapshot.verification.peer_count,
                "protocol_state": "verified",
                "remote_descriptor_sha256": sha256(frame.remote_descriptor).hexdigest(),
                "remote_encoding": frame.remote_encoding,
                "remote_type_name": frame.remote_type_name,
                "worker_order": list(_WORKER_ORDER),
            }
            exit_code = 0
    finally:
        if subscriber is not None:
            remove_callback = getattr(subscriber, "remove_receive_callback", None)
            if callable(remove_callback):
                remove_callback()
        core.finalize()
    output["finalized"] = True
    _write_new_json(result, output)
    return exit_code


@dataclass(frozen=True)
class _ProcessEvidence:
    """一个 participant 子进程的退出状态和捕获日志位置。"""

    returncode: int
    stdout_path: Path
    stderr_path: Path


def _finish_process(
    process: Popen[bytes],
    *,
    timeout_sec: float,
    stdout_path: Path,
    stderr_path: Path,
) -> _ProcessEvidence:
    """以绝对 deadline 收口子进程，超时依次 TERM、KILL 并保留日志。"""
    try:
        stdout, stderr = process.communicate(timeout=timeout_sec)
    except TimeoutExpired:
        process.terminate()
        try:
            stdout, stderr = process.communicate(timeout=2)
        except TimeoutExpired:
            process.kill()
            stdout, stderr = process.communicate()
            stderr += b"\nphase0_parent_killed_after_timeout\n"
    stdout_path.write_bytes(stdout)
    stderr_path.write_bytes(stderr)
    return _ProcessEvidence(process.returncode, stdout_path, stderr_path)


def _write_process_evidence(path: Path, evidence: _ProcessEvidence) -> None:
    """将 parent 观察的退出码与已落盘日志摘要排他固化为独立证据。"""
    _write_new_json(
        path,
        {
            "returncode": evidence.returncode,
            "stderr_sha256": sha256(evidence.stderr_path.read_bytes()).hexdigest(),
            "stdout_sha256": sha256(evidence.stdout_path.read_bytes()).hexdigest(),
        },
    )


def _start_process(command: list[str], environment: dict[str, str]) -> Popen[bytes]:
    """所有 participant 禁止继承测试 shim 或动态预加载替代品。"""
    return Popen(command, stdin=subprocess.DEVNULL, stdout=PIPE, stderr=PIPE, env=environment)


def _run_pair(
    *,
    subscriber_command: list[str],
    publisher_command: list[str],
    paths: dict[str, Path],
    environment: dict[str, str],
    conflict: bool = False,
) -> tuple[_ProcessEvidence, _ProcessEvidence]:
    """严格先起订阅端再起发布端，不做失败自动重试。"""
    subscriber = _start_process(subscriber_command, environment)
    sleep(0.25)
    publisher = _start_process(publisher_command, environment)
    publisher_evidence = _finish_process(
        publisher, timeout_sec=12, stdout_path=paths["publisher_stdout"], stderr_path=paths["publisher_stderr"]
    )
    subscriber_evidence = _finish_process(
        subscriber, timeout_sec=12, stdout_path=paths["subscriber_stdout"], stderr_path=paths["subscriber_stderr"]
    )
    if conflict:
        if publisher_evidence.returncode != 0 or subscriber_evidence.returncode == 0:
            raise RuntimeError("v1 same-topic conflict participant exited unexpectedly")
    elif publisher_evidence.returncode != 0 or subscriber_evidence.returncode != 0:
        raise RuntimeError("Phase-0 participant exited nonzero")
    return publisher_evidence, subscriber_evidence


def _assert_cpp_dry_run(command: list[str]) -> None:
    """真实 C++ probe 前逐条验证 Task 9 冻结的 canonical dry-run。"""
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode != 0 or completed.stderr:
        raise RuntimeError("C++ probe dry-run failed")
    try:
        plan = json.loads(completed.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError("C++ probe dry-run did not return canonical JSON") from error
    if plan.get("dry_run") is not True or plan.get("topic") != "/sim/wheel/command":
        raise RuntimeError("C++ probe dry-run plan differs from Phase-0 contract")


def _fixed_v2_payload(descriptor: DescriptorIdentity) -> bytes:
    """使用固定会话 fixture 产生一次确定性 WheelCommand 原始 bytes。"""
    session = ProtocolSession(descriptor, session_id_factory=lambda: b"s" * 16)
    model = WheelCommandV2(
        timestamp_ns=1,
        drive_wheel_speed_rad_s=(1.0, 1.0),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=session.world_generation,
        command_generation=session.command_generation,
        source_id="phase0",
        source_session_id=b"p" * 16,
        robot_model="df_back",
        simulation_session_id=session.simulation_session_id,
        descriptor_sha256=session.descriptor_sha256,
    )
    return V2ProtoCodec(descriptor).encode(model).payload


def _copy_v1_descriptor(path: Path) -> None:
    """仅为同名冲突 participant 制作 v1 raw metadata，不作为 v2 验收输入。"""
    descriptor = descriptor_pb2.FileDescriptorSet()
    descriptor.file.add().ParseFromString(v1_pb.DESCRIPTOR.serialized_pb)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(descriptor.SerializeToString(deterministic=True))


def _scenario_success(
    name: str,
    paths: dict[str, Path],
    publisher_result: dict[str, object],
    subscriber_result: dict[str, object],
) -> dict[str, object]:
    """归一化 Python/C++ participant 的公共 Phase-0 成功结果。"""
    assert publisher_result.get("clean_shutdown") is True, "publisher must report clean_shutdown"
    assert subscriber_result.get("clean_shutdown") is True, "subscriber must report clean_shutdown"
    assert publisher_result.get("finalized") is True, "publisher must report finalized"
    assert subscriber_result.get("finalized") is True, "subscriber must report finalized"
    assert subscriber_result.get("protocol_state") == "verified", "subscriber must report verified protocol_state"
    assert subscriber_result.get("worker_order") == _WORKER_ORDER, "subscriber must report worker_order"
    scenario = {
        "clean_shutdown": True,
        "finalized": True,
        "publisher": publisher_result,
        "subscriber": subscriber_result,
    }
    _write_new_json(paths["scenario_result"], scenario)
    return scenario


def run_phase0_gate(*, evidence_dir: Path, phase0_build_dir: Path) -> dict[str, object]:
    """执行一次严格串行的 Python/C++ raw eCAL Phase-0，并返回唯一汇总。"""
    if os.environ.get("STAGE4_ECAL_TEST_SHIM") or os.environ.get("LD_PRELOAD"):
        raise RuntimeError("real Phase-0 must not run with eCAL shim or LD_PRELOAD")
    probe = (phase0_build_dir / "ecal_v2_raw_probe").resolve()
    if not probe.is_file() or not os.access(probe, os.X_OK):
        raise RuntimeError("Phase-0 C++ probe is unavailable")
    paths = build_phase0_runtime_paths(evidence_dir)
    root = evidence_dir.resolve()
    root.mkdir(parents=True)
    descriptor = load_v2_descriptor()
    prefix = os.environ.get("STAGE4_DEPENDENCY_PREFIX")
    if not prefix:
        raise RuntimeError("STAGE4_DEPENDENCY_PREFIX is required for real Phase-0")
    child_environment = build_phase0_child_environment(os.environ, Path(prefix))
    descriptor_path = paths["descriptor"]["copy"]
    descriptor_path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(Path("slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"), descriptor_path)
    v1_descriptor_path = root / "descriptor" / "slope_sim_interfaces_v1.desc"
    _copy_v1_descriptor(v1_descriptor_path)
    type_name = "slope_sim.interfaces.v2.WheelCommand"
    payload = _fixed_v2_payload(descriptor)
    scenarios: dict[str, object] = {}

    def python_command(role: str, scenario: str, *, raw_type: str = type_name, raw_descriptor: Path = descriptor_path) -> list[str]:
        item = paths[scenario]
        payload_path = item["send_payload"] if role in {"publish", "announce"} else item["receive_payload"]
        return build_python_participant_process_command(
            role=role, topic="/sim/wheel/command", type_name=raw_type, descriptor_set=raw_descriptor,
            payload=payload_path, result=item["publisher_result"] if role in {"publish", "announce"} else item["subscriber_result"], deadline_ms=10000,
        )

    # Python raw -> Python raw.
    item = paths["python_to_python_raw"]
    item["send_payload"].parent.mkdir(parents=True, exist_ok=True)
    item["send_payload"].write_bytes(payload)
    _run_pair(subscriber_command=python_command("subscribe", "python_to_python_raw"), publisher_command=python_command("publish", "python_to_python_raw"), paths=item, environment=child_environment)
    scenarios["python_to_python_raw"] = _scenario_success("python_to_python_raw", item, _read_result(item["publisher_result"], "Python publisher"), _read_result(item["subscriber_result"], "Python subscriber"))

    # Python raw -> C++ raw; the C++ subscriber's dry-run is mandatory before it starts.
    item = paths["python_to_cpp_raw"]
    item["send_payload"].parent.mkdir(parents=True, exist_ok=True)
    item["send_payload"].write_bytes(payload)
    cpp_subscribe = build_cpp_probe_commands(probe=str(probe), role="subscribe", topic="/sim/wheel/command", type_name=type_name, descriptor_set=str(descriptor_path), payload_out=str(item["receive_payload"]), result=str(item["subscriber_result"]), expected_peer_count=1, deadline_ms=10000)
    _assert_cpp_dry_run(cpp_subscribe[0])
    _run_pair(subscriber_command=cpp_subscribe[1], publisher_command=python_command("publish", "python_to_cpp_raw"), paths=item, environment=child_environment)
    scenarios["python_to_cpp_raw"] = _scenario_success("python_to_cpp_raw", item, _read_result(item["publisher_result"], "Python publisher"), _read_result(item["subscriber_result"], "C++ subscriber"))

    # C++ raw -> Python raw; the C++ publisher's dry-run is mandatory before it starts.
    item = paths["cpp_to_python_raw"]
    item["send_payload"].parent.mkdir(parents=True, exist_ok=True)
    item["send_payload"].write_bytes(payload)
    cpp_publish = build_cpp_probe_commands(probe=str(probe), role="publish", topic="/sim/wheel/command", type_name=type_name, descriptor_set=str(descriptor_path), payload=str(item["send_payload"]), result=str(item["publisher_result"]), deadline_ms=10000)
    _assert_cpp_dry_run(cpp_publish[0])
    _run_pair(subscriber_command=python_command("subscribe", "cpp_to_python_raw"), publisher_command=cpp_publish[1], paths=item, environment=child_environment)
    scenarios["cpp_to_python_raw"] = _scenario_success("cpp_to_python_raw", item, _read_result(item["publisher_result"], "C++ publisher"), _read_result(item["subscriber_result"], "Python subscriber"))

    # A v1 raw publisher on the same final topic must cause a v2 monitoring conflict before delivery.
    item = paths["v1_v2_same_topic_conflict"]
    item["send_payload"].parent.mkdir(parents=True, exist_ok=True)
    item["send_payload"].write_bytes(v1_pb.WheelCommand(timestamp_ns=1).SerializeToString(deterministic=True))
    publisher_evidence, subscriber_evidence = _run_pair(
        subscriber_command=python_command("subscribe", "v1_v2_same_topic_conflict"),
        publisher_command=python_command("announce", "v1_v2_same_topic_conflict", raw_type="slope_sim.interfaces.v1.WheelCommand", raw_descriptor=v1_descriptor_path),
        paths=item,
        environment=child_environment,
        conflict=True,
    )
    _write_process_evidence(item["publisher_process"], publisher_evidence)
    _write_process_evidence(item["subscriber_process"], subscriber_evidence)
    publisher_result = _read_result(item["publisher_result"], "v1 metadata announcer")
    conflict_result = _read_result(item["subscriber_result"], "v1/v2 conflict subscriber")
    conflict_result["exit_code"] = subscriber_evidence.returncode
    conflict_result["clean_shutdown"] = conflict_result.get("clean_shutdown") is True and publisher_result.get("clean_shutdown") is True
    conflict_result["finalized"] = conflict_result.get("finalized") is True
    _write_new_json(item["scenario_result"], conflict_result)
    scenarios["v1_v2_same_topic_conflict"] = conflict_result

    result = {
        "evidence_dir": str(root),
        "expected_descriptor_sha256": descriptor.sha256.hex(),
        "expected_encoding": "proto",
        "expected_type": type_name,
        "scenarios": scenarios,
        "topic": "/sim/wheel/command",
    }
    _write_new_json(root / "phase0-result.json", result)
    return result


def _participant_main(argv: list[str]) -> int:
    """解析仅供内部子进程使用的 Python raw participant CLI。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--participant", choices=("publish", "subscribe", "announce"), required=True)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--type-name", required=True)
    parser.add_argument("--descriptor-set", type=Path, required=True)
    parser.add_argument("--payload", type=Path, required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--deadline-ms", type=int, required=True)
    args = parser.parse_args(argv)
    return _run_python_participant(
        role=args.participant, topic=args.topic, type_name=args.type_name,
        descriptor_set=args.descriptor_set.resolve(), payload=args.payload.resolve(),
        result=args.result.resolve(), deadline_ms=args.deadline_ms,
    )


def main(argv: list[str] | None = None) -> int:
    """脚本入口只暴露内部 participant 子命令，主 gate 始终由 pytest 驱动。"""
    try:
        return _participant_main(sys.argv[1:] if argv is None else argv)
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
