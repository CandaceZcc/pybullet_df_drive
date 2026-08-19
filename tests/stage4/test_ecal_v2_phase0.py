"""阶段四 A：真实 eCAL Phase-0 结果的纯判定器合同。"""
from importlib import import_module
import json
import os
from pathlib import Path
import sys
from time import strftime
import weakref

import pytest


def require_wished_module(name: str):
    """让尚未创建的判定器表现为明确 RED，而不是 pytest 收集错误。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


def valid_phase0_result() -> dict[str, object]:
    """构造不包含 participant 的最小已验证 Phase-0 结果 oracle。"""
    descriptor = "ab" * 32
    payload = "cd" * 32
    subscriber = {
        "payload_sha256": payload,
        "descriptor_sha256": descriptor,
        "remote_type_name": "slope_sim.interfaces.v2.WheelCommand",
        "remote_encoding": "proto",
        "remote_descriptor_sha256": descriptor,
        "peer_count": 1,
        "protocol_state": "verified",
        "worker_order": [
            "payload_sha256",
            "remote_metadata_verified",
            "protobuf_parsed",
            "in_band_identity_validated",
        ],
    }
    return {
        "topic": "/sim/wheel/command",
        "expected_type": "slope_sim.interfaces.v2.WheelCommand",
        "expected_encoding": "proto",
        "expected_descriptor_sha256": descriptor,
        "scenarios": {
            name: {
                "publisher": {"payload_sha256": payload, "descriptor_sha256": descriptor},
                "subscriber": dict(subscriber),
                "clean_shutdown": True,
                "finalized": True,
            }
            for name in (
                "python_to_python_raw",
                "python_to_cpp_raw",
                "cpp_to_python_raw",
            )
        }
        | {
            "v1_v2_same_topic_conflict": {
                "protocol_state": "conflict",
                "accepted_count": 0,
                "exit_code": 1,
                "clean_shutdown": True,
                "finalized": True,
            }
        },
    }


def phase0_result_with_evidence(tmp_path: Path) -> tuple[object, dict[str, object]]:
    """写入完整的最小 Phase-0 证据树，供汇总与 participant 绑定测试复用。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    descriptor = module.load_v2_descriptor()
    payload = b"phase0 evidence payload"
    evidence_dir = tmp_path / "phase0-evidence"
    descriptor_path = evidence_dir / "descriptor" / "slope_sim_interfaces_v2.desc"
    descriptor_path.parent.mkdir(parents=True)
    descriptor_path.write_bytes(descriptor.serialized_file_descriptor_set)
    result = valid_phase0_result()
    result["evidence_dir"] = str(evidence_dir)
    result["expected_descriptor_sha256"] = descriptor.sha256.hex()
    scenarios = result["scenarios"]
    assert isinstance(scenarios, dict)
    for name in ("python_to_python_raw", "python_to_cpp_raw", "cpp_to_python_raw"):
        scenario = scenarios[name]
        assert isinstance(scenario, dict)
        scenario_dir = evidence_dir / name
        scenario_dir.mkdir()
        (scenario_dir / "send.bin").write_bytes(payload)
        (scenario_dir / "received.bin").write_bytes(payload)
        digest = module.sha256(payload).hexdigest()
        publisher = scenario["publisher"]
        subscriber = scenario["subscriber"]
        assert isinstance(publisher, dict) and isinstance(subscriber, dict)
        publisher["payload_sha256"] = digest
        publisher["descriptor_sha256"] = descriptor.sha256.hex()
        subscriber["payload_sha256"] = digest
        subscriber["descriptor_sha256"] = descriptor.sha256.hex()
        subscriber["remote_descriptor_sha256"] = descriptor.sha256.hex()
        (scenario_dir / "publisher.json").write_text(json.dumps(publisher), encoding="utf-8")
        (scenario_dir / "subscriber.json").write_text(json.dumps(subscriber), encoding="utf-8")
        (scenario_dir / "scenario.json").write_text(json.dumps(scenario), encoding="utf-8")

    conflict = scenarios["v1_v2_same_topic_conflict"]
    assert isinstance(conflict, dict)
    conflict_dir = evidence_dir / "v1_v2_same_topic_conflict"
    conflict_dir.mkdir()
    publisher = {"clean_shutdown": True, "finalized": True, "mode": "announce"}
    subscriber = {key: value for key, value in conflict.items() if key != "exit_code"}
    (conflict_dir / "publisher.json").write_text(json.dumps(publisher), encoding="utf-8")
    (conflict_dir / "subscriber.json").write_text(json.dumps(subscriber), encoding="utf-8")
    (conflict_dir / "scenario.json").write_text(json.dumps(conflict), encoding="utf-8")
    for role, returncode in (("publisher", 0), ("subscriber", conflict["exit_code"])):
        assert isinstance(returncode, int)
        stdout = conflict_dir / f"{role}.stdout"
        stderr = conflict_dir / f"{role}.stderr"
        stdout.write_bytes(b"")
        stderr.write_bytes(b"")
        (conflict_dir / f"{role}-process.json").write_text(
            json.dumps(
                {
                    "returncode": returncode,
                    "stderr_sha256": module.sha256(b"").hexdigest(),
                    "stdout_sha256": module.sha256(b"").hexdigest(),
                }
            ),
            encoding="utf-8",
        )
    return module, result


def test_phase0_rejects_payload_hash_mismatch() -> None:
    """任一跨语言场景的 payload SHA-256 不同必须硬失败。"""
    verifier = require_wished_module("scripts.verify_stage4_v2_phase0").verify_phase0_result
    result = valid_phase0_result()
    scenarios = result["scenarios"]
    assert isinstance(scenarios, dict)
    scenarios["python_to_cpp_raw"]["subscriber"]["payload_sha256"] = "00" * 32
    with pytest.raises(AssertionError, match="payload SHA-256"):
        verifier(result)


def test_phase0_rejects_received_bytes_that_do_not_match_evidence(tmp_path: Path) -> None:
    """汇总 JSON 即使自洽，也必须与本次收发原始 bytes 完全一致。"""
    module, result = phase0_result_with_evidence(tmp_path)
    evidence_dir = Path(result["evidence_dir"])
    (evidence_dir / "cpp_to_python_raw" / "received.bin").write_bytes(b"tampered")

    with pytest.raises(AssertionError, match="received payload differs"):
        module.verify_phase0_result(result)


def test_phase0_rejects_conflict_subscriber_that_does_not_match_evidence(tmp_path: Path) -> None:
    """v1/v2 冲突汇总也必须逐项绑定 subscriber 和 scenario 落盘证据。"""
    module, result = phase0_result_with_evidence(tmp_path)
    evidence_dir = Path(result["evidence_dir"])
    subscriber_path = evidence_dir / "v1_v2_same_topic_conflict" / "subscriber.json"
    subscriber = json.loads(subscriber_path.read_text(encoding="utf-8"))
    subscriber["accepted_count"] = 1
    subscriber_path.write_text(json.dumps(subscriber), encoding="utf-8")

    with pytest.raises(AssertionError, match="conflict subscriber"):
        module.verify_phase0_result(result)


def test_phase0_rejects_conflict_exit_code_that_does_not_match_process_evidence(
    tmp_path: Path,
) -> None:
    """冲突的非零退出状态必须与 parent 排他写入的 process 证据一致。"""
    module, result = phase0_result_with_evidence(tmp_path)
    evidence_dir = Path(result["evidence_dir"])
    process_path = evidence_dir / "v1_v2_same_topic_conflict" / "subscriber-process.json"
    process = json.loads(process_path.read_text(encoding="utf-8"))
    process["returncode"] = 0
    process_path.write_text(json.dumps(process), encoding="utf-8")

    with pytest.raises(AssertionError, match="conflict subscriber process"):
        module.verify_phase0_result(result)


def test_cpp_probe_argv_is_exact_and_dry_run_precedes_real() -> None:
    """编排器必须先验证冻结 CLI，再允许启动真实 probe。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    probe = "/opt/stage4/bin/ecal_v2_raw_probe"
    descriptor = "/tmp/stage4/v2.desc"
    common = {
        "probe": probe,
        "topic": "/sim/wheel/command",
        "type_name": "slope_sim.interfaces.v2.WheelCommand",
        "descriptor_set": descriptor,
        "deadline_ms": 10000,
    }
    publish = module.build_cpp_probe_commands(
        role="publish",
        payload="/tmp/stage4/send.bin",
        result="/tmp/stage4/publish.json",
        **common,
    )
    assert publish == (
        [
            probe, "--dry-run", "publish", "--topic", "/sim/wheel/command",
            "--type-name", common["type_name"], "--descriptor-set", descriptor,
            "--payload", "/tmp/stage4/send.bin", "--result", "/tmp/stage4/publish.json",
            "--deadline-ms", "10000",
        ],
        [
            probe, "publish", "--topic", "/sim/wheel/command",
            "--type-name", common["type_name"], "--descriptor-set", descriptor,
            "--payload", "/tmp/stage4/send.bin", "--result", "/tmp/stage4/publish.json",
            "--deadline-ms", "10000",
        ],
    )
    calls: list[list[str]] = []
    module.run_cpp_probe_commands(
        publish,
        runner=lambda argv: calls.append(argv) or __import__("subprocess").CompletedProcess(argv, 0),
    )
    assert calls == list(publish)
    calls.clear()
    with pytest.raises(RuntimeError, match="probe dry-run failed"):
        module.run_cpp_probe_commands(
            publish,
            runner=lambda argv: calls.append(argv) or __import__("subprocess").CompletedProcess(argv, 1),
        )
    assert calls == [publish[0]]


def test_phase0_requires_v1_same_topic_hard_failure() -> None:
    """v1 peer 和 v2 使用同一 topic 时不得保留任何通过路径。"""
    verifier = require_wished_module("scripts.verify_stage4_v2_phase0").verify_phase0_result
    result = valid_phase0_result()
    scenarios = result["scenarios"]
    assert isinstance(scenarios, dict)
    scenarios["v1_v2_same_topic_conflict"]["exit_code"] = 0
    with pytest.raises(AssertionError, match="v1 same-topic peer"):
        verifier(result)


def test_phase0_rejects_remote_descriptor_metadata_mismatch() -> None:
    """callback 本帧远端 descriptor 不同必须硬失败，不能只看带内摘要。"""
    verifier = require_wished_module("scripts.verify_stage4_v2_phase0").verify_phase0_result
    result = valid_phase0_result()
    scenarios = result["scenarios"]
    assert isinstance(scenarios, dict)
    scenarios["cpp_to_python_raw"]["subscriber"]["remote_descriptor_sha256"] = "11" * 32
    with pytest.raises(AssertionError, match="remote descriptor"):
        verifier(result)


def test_phase0_success_refuses_to_invent_missing_cpp_evidence(tmp_path: Path) -> None:
    """C++ participant 未写出的 worker/finalize 字段不得由汇总器默认成通过。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    result = valid_phase0_result()
    scenarios = result["scenarios"]
    assert isinstance(scenarios, dict)
    scenario = scenarios["python_to_cpp_raw"]
    assert isinstance(scenario, dict)
    subscriber = scenario["subscriber"]
    assert isinstance(subscriber, dict)
    publisher = scenario["publisher"]
    assert isinstance(publisher, dict)
    publisher.update({"clean_shutdown": True, "finalized": True})
    subscriber.update({"clean_shutdown": True, "finalized": True})
    subscriber.pop("protocol_state")
    subscriber.pop("worker_order")

    with pytest.raises(AssertionError, match="must report"):
        module._scenario_success(
            "python_to_cpp_raw",
            {"scenario_result": tmp_path / "scenario.json"},
            publisher,
            subscriber,
        )


def test_phase0_runtime_paths_are_unique_and_absolute(tmp_path: Path) -> None:
    """真实 gate 只能写入本次新建证据目录，不能复用历史结果。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    paths = module.build_phase0_runtime_paths(tmp_path / "evidence")
    assert set(paths) == {
        "descriptor",
        "python_to_python_raw",
        "python_to_cpp_raw",
        "cpp_to_python_raw",
        "v1_v2_same_topic_conflict",
    }
    all_paths = [path for scenario in paths.values() for path in scenario.values()]
    assert all(path.is_absolute() for path in all_paths)
    assert len(all_paths) == len(set(all_paths))
    assert not any(path.exists() for path in all_paths)


def test_phase0_json_evidence_uses_exclusive_create_after_race(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """竞争者在存在性检查后占位时，证据写入绝不能覆盖它。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    target = tmp_path / "phase0.json"
    target.write_text("existing\n", encoding="utf-8")
    original_exists = Path.exists
    monkeypatch.setattr(
        Path,
        "exists",
        lambda self: False if self == target else original_exists(self),
    )

    with pytest.raises(FileExistsError):
        module._write_new_json(target, {"new": True})
    assert target.read_text(encoding="utf-8") == "existing\n"


def test_python_participant_argv_is_explicit_and_frozen(tmp_path: Path) -> None:
    """Python raw participant 也必须以固定、完整的跨进程 CLI 启动。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    command = module.build_python_participant_command(
        role="subscribe",
        topic="/sim/wheel/command",
        type_name="slope_sim.interfaces.v2.WheelCommand",
        descriptor_set=tmp_path / "v2.desc",
        payload=tmp_path / "received.bin",
        result=tmp_path / "subscriber.json",
        deadline_ms=10000,
    )
    assert command == [
        "--participant", "subscribe",
        "--topic", "/sim/wheel/command",
        "--type-name", "slope_sim.interfaces.v2.WheelCommand",
        "--descriptor-set", str((tmp_path / "v2.desc").resolve()),
        "--payload", str((tmp_path / "received.bin").resolve()),
        "--result", str((tmp_path / "subscriber.json").resolve()),
        "--deadline-ms", "10000",
    ]


def test_python_participant_process_uses_project_module(tmp_path: Path) -> None:
    """子进程必须以模块方式启动，确保项目根可导入 slope_sim。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    command = module.build_python_participant_process_command(
        role="publish",
        topic="/sim/wheel/command",
        type_name="slope_sim.interfaces.v2.WheelCommand",
        descriptor_set=tmp_path / "v2.desc",
        payload=tmp_path / "send.bin",
        result=tmp_path / "publisher.json",
        deadline_ms=10000,
    )
    assert command[:3] == [sys.executable, "-m", "scripts.verify_stage4_v2_phase0"]
    assert command[3:] == module.build_python_participant_command(
        role="publish",
        topic="/sim/wheel/command",
        type_name="slope_sim.interfaces.v2.WheelCommand",
        descriptor_set=tmp_path / "v2.desc",
        payload=tmp_path / "send.bin",
        result=tmp_path / "publisher.json",
        deadline_ms=10000,
    )


def test_phase0_initializes_all_ecal_components_for_monitoring() -> None:
    """Phase-0 discovery 依赖 monitoring，不能使用默认的 0x37 组件集。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")

    class Core:
        calls: list[tuple[str, int]] = []

        def initialize(self, name: str, components: int) -> bool:
            self.calls.append((name, components))
            return True

    core = Core()
    module.initialize_phase0_core(core, "phase0-test")
    assert core.calls == [("phase0-test", 0x3F)]


def test_phase0_child_environment_uses_frozen_ecal_prefix(tmp_path: Path) -> None:
    """participant 必须显式加载私有 eCAL 配置和 time plugin，不依赖系统路径。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    prefix = tmp_path / "prefix"
    (prefix / "etc" / "ecal").mkdir(parents=True)
    (prefix / "etc" / "ecal" / "ecal.yaml").write_text("# test\n", encoding="ascii")
    (prefix / "lib").mkdir()
    environment = module.build_phase0_child_environment(
        {"STAGE4_ECAL_TEST_SHIM": "shim", "LD_PRELOAD": "preload", "KEEP": "yes"},
        prefix,
    )
    assert environment["ECAL_DATA"] == str((prefix / "etc" / "ecal").resolve())
    assert environment["ECAL_TIME_PLUGIN_PATH"] == str((prefix / "lib").resolve())
    assert environment["KEEP"] == "yes"
    assert "STAGE4_ECAL_TEST_SHIM" not in environment
    assert "LD_PRELOAD" not in environment


def test_metadata_only_participant_announces_without_peer_wait_or_send(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """同名冲突用 v1 端只能公布 metadata，不能等待 v2 接收或投递 payload。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    calls: list[str] = []

    class Core:
        def initialize(self, _name: str, _components: int) -> bool:
            calls.append("initialize")
            return True

        def finalize(self) -> None:
            calls.append("finalize")

    class Publisher:
        def get_subscriber_count(self) -> int:
            pytest.fail("metadata-only participant must not wait for a peer")

        def send(self, _payload: bytes) -> None:
            pytest.fail("metadata-only participant must not send payload")

    publisher_refs: list[weakref.ReferenceType[Publisher]] = []

    class Bindings:
        _core = Core()

        def create_publisher(self, *_args: object) -> Publisher:
            calls.append("create_publisher")
            publisher = Publisher()
            publisher_refs.append(weakref.ref(publisher))
            return publisher

    holds: list[float] = []
    monkeypatch.setattr(module, "EcalRawBindings", lambda: Bindings())

    def hold(seconds: float) -> None:
        assert publisher_refs and publisher_refs[0]() is not None, (
            "announcer must retain publisher through metadata window"
        )
        holds.append(seconds)

    monkeypatch.setattr(module, "sleep", hold)
    descriptor = tmp_path / "v1.desc"
    descriptor.write_bytes(b"v1 descriptor")
    result = tmp_path / "announcer.json"

    assert module._run_python_participant(
        role="announce",
        topic="/sim/wheel/command",
        type_name="slope_sim.interfaces.v1.WheelCommand",
        descriptor_set=descriptor,
        payload=tmp_path / "must-not-be-read.bin",
        result=result,
        deadline_ms=10000,
    ) == 0
    assert calls == ["initialize", "create_publisher", "finalize"]
    assert holds == [10.0]
    assert json.loads(result.read_text(encoding="utf-8")) == {
        "clean_shutdown": True,
        "finalized": True,
        "mode": "announce",
    }


def test_participant_cli_accepts_metadata_only_announce(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """内部 participant CLI 必须允许冲突场景的 metadata-only role。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    called: dict[str, object] = {}
    monkeypatch.setattr(
        module,
        "_run_python_participant",
        lambda **kwargs: called.update(kwargs) or 0,
    )
    assert module.main([
        "--participant", "announce",
        "--topic", "/sim/wheel/command",
        "--type-name", "slope_sim.interfaces.v1.WheelCommand",
        "--descriptor-set", str(tmp_path / "v1.desc"),
        "--payload", str(tmp_path / "unused.bin"),
        "--result", str(tmp_path / "announcer.json"),
        "--deadline-ms", "10000",
    ]) == 0
    assert called["role"] == "announce"


@pytest.fixture(scope="module")
def real_phase0_result() -> dict[str, object]:
    """本模块四项真实验收共享一次受授权的串行 eCAL gate。"""
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    build_dir = Path(os.environ["STAGE4_PHASE0_BUILD_DIR"])
    evidence_dir = Path(
        os.environ.get(
            "STAGE4_PHASE0_EVIDENCE_DIR",
            f"results/stage4/phase0-real-{strftime('%Y%m%dT%H%M%S')}",
        )
    )
    result = module.run_phase0_gate(evidence_dir=evidence_dir, phase0_build_dir=build_dir)
    module.verify_phase0_result(result)
    return result


@pytest.mark.ecal
def test_real_phase0_python_to_python_raw(real_phase0_result: dict[str, object]) -> None:
    """真实 Python raw publisher/subscriber 必须交付同一原始 payload。"""
    scenarios = real_phase0_result["scenarios"]
    assert isinstance(scenarios, dict)
    assert scenarios["python_to_python_raw"]["subscriber"]["protocol_state"] == "verified"


@pytest.mark.ecal
def test_real_phase0_python_to_cpp_raw(real_phase0_result: dict[str, object]) -> None:
    """真实 Python publisher 到 C++ subscriber 必须验证 callback metadata。"""
    scenarios = real_phase0_result["scenarios"]
    assert isinstance(scenarios, dict)
    assert scenarios["python_to_cpp_raw"]["subscriber"]["protocol_state"] == "verified"


@pytest.mark.ecal
def test_real_phase0_cpp_to_python_raw(real_phase0_result: dict[str, object]) -> None:
    """真实 C++ publisher 到 Python subscriber 必须验证 callback metadata。"""
    scenarios = real_phase0_result["scenarios"]
    assert isinstance(scenarios, dict)
    assert scenarios["cpp_to_python_raw"]["subscriber"]["protocol_state"] == "verified"


@pytest.mark.ecal
def test_real_phase0_rejects_v1_same_topic(real_phase0_result: dict[str, object]) -> None:
    """同名 v1 publisher 必须让 v2 subscriber 硬拒绝且不接收 payload。"""
    scenarios = real_phase0_result["scenarios"]
    assert isinstance(scenarios, dict)
    conflict = scenarios["v1_v2_same_topic_conflict"]
    assert conflict["protocol_state"] == "conflict"
    assert conflict["accepted_count"] == 0
