"""阶段四 A：冻结 C++17 Phase-0 raw probe 的 ABI 和 dry-run CLI 合同。"""
from hashlib import sha256
import json
import os
from pathlib import Path
import re
import subprocess

import pytest

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb


def test_phase0_creates_the_protobuf_generated_output_directory() -> None:
    """Phase-0 必须在 protoc 写入前创建其受控生成目录。"""
    cmake = (Path(__file__).resolve().parents[2] / "cpp" / "phase0" / "CMakeLists.txt").read_text(encoding="utf-8")

    assert 'file(MAKE_DIRECTORY "${V2_GENERATED_DIR}")' in cmake


def test_phase0_uses_jazzy_python_in_the_current_rosidl_scope() -> None:
    """ROSIDL 必须读取 Jazzy Python 的普通变量，而不只依赖 cache。"""
    cmake = (Path(__file__).resolve().parents[2] / "cpp" / "phase0" / "CMakeLists.txt").read_text(encoding="utf-8")

    scope_assignment = 'set(Python3_EXECUTABLE "${STAGE4_ROSIDL_PYTHON_EXECUTABLE}")'
    cache_assignment = 'set(Python3_EXECUTABLE "${STAGE4_ROSIDL_PYTHON_EXECUTABLE}" CACHE FILEPATH'
    assert scope_assignment in cmake
    assert cache_assignment in cmake
    assert cmake.index(scope_assignment) < cmake.index(cache_assignment)


def test_phase0_retargets_the_existing_python_interpreter_for_ament_package_xml() -> None:
    """ament_package_xml 必须从已创建的 Python3 target 读取 Jazzy 解释器。"""
    cmake = (Path(__file__).resolve().parents[2] / "cpp" / "phase0" / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "set_property(TARGET Python3::Interpreter PROPERTY IMPORTED_LOCATION" in cmake
    assert '"${STAGE4_ROSIDL_PYTHON_EXECUTABLE}")' in cmake


def test_phase0_cmake_resolves_protoc_through_active_configuration_mapping() -> None:
    """Protobuf 导出必须遵循 eCAL 给当前 build type 的配置映射。"""
    cmake_lists = (
        Path(__file__).resolve().parents[2] / "cpp" / "phase0" / "CMakeLists.txt"
    ).read_text(encoding="utf-8")

    assert 'string(TOUPPER "${CMAKE_BUILD_TYPE}" STAGE4_IMPORTED_CONFIG)' in cmake_lists
    assert '"CMAKE_MAP_IMPORTED_CONFIG_${STAGE4_IMPORTED_CONFIG}"' in cmake_lists
    assert "IMPORTED_LOCATION_${STAGE4_IMPORTED_CONFIG}" in cmake_lists
    assert "IMPORTED_LOCATION_NOCONFIG" not in cmake_lists


def _phase0_executable(name: str) -> Path:
    """从 CTest 注入的绝对 build root 获取尚未实现或已构建的工具。"""
    raw_build = os.environ.get("STAGE4_PHASE0_BUILD_DIR")
    assert raw_build, "CTest must provide STAGE4_PHASE0_BUILD_DIR"
    build_dir = Path(raw_build)
    assert build_dir.is_absolute(), "Phase-0 build directory must be absolute"
    assert build_dir.is_dir(), "configured Phase-0 build directory is missing"
    executable = build_dir / name
    assert executable.is_file(), f"Phase-0 {name} behavior is not implemented"
    assert os.access(executable, os.X_OK), f"Phase-0 {name} is not executable"
    return executable


@pytest.mark.stage4_artifact
def test_phase0_install_publishes_core_tools_and_sdk(tmp_path: Path) -> None:
    """release setup 依赖 CMake 实际安装五个工具和公共 SDK。"""
    build_dir = _phase0_executable("slope_sim_stage4_command").parent
    source = Path(__file__).resolve().parents[2] / "cpp" / "phase0"
    subprocess.run(["cmake", "-S", str(source), "-B", str(build_dir)], check=True)
    subprocess.run(["cmake", "--build", str(build_dir), "--parallel"], check=True)
    prefix = tmp_path / "release"
    subprocess.run(["cmake", "--install", str(build_dir), "--prefix", str(prefix)], check=True)

    for executable in (
        "slope_sim_stage4_command",
        "slope_sim_stage4_subscriber",
        "slope_sim_stage4_recorder",
        "slope_sim_stage4_replay",
        "slope_sim_stage4_export",
    ):
        path = prefix / "bin" / executable
        assert path.is_file() and os.access(path, os.X_OK)
    assert (prefix / "lib" / "libslope_sim_client.a").is_file()


def _run_abi_inspection(probe: Path, tool: str, *args: str) -> str:
    """以稳定的英文 ELF 输出检查 raw probe 的运行时 ABI。"""
    result = subprocess.run(
        [tool, *args, str(probe)],
        check=True,
        capture_output=True,
        text=True,
        env={**os.environ, "LC_ALL": "C", "LANG": "C"},
    )
    return result.stdout


def test_raw_probe_abi_inspection_forces_c_locale(monkeypatch) -> None:
    """ABI 工具输出必须固定为 C locale，避免系统界面语言改变断言语义。"""
    calls: list[tuple[list[str], dict[str, object]]] = []

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, stdout="inspection\n", stderr="")

    monkeypatch.setattr(subprocess, "run", fake_run)

    assert _run_abi_inspection(Path("/tmp/ecal_v2_raw_probe"), "readelf", "-d") == "inspection\n"
    assert calls[0][0] == ["readelf", "-d", "/tmp/ecal_v2_raw_probe"]
    assert calls[0][1]["env"]["LC_ALL"] == "C"
    assert calls[0][1]["env"]["LANG"] == "C"


def _valid_probe_inputs(tmp_path: Path) -> tuple[Path, Path]:
    """生成由 Python v2 codec 合同可解析的确定性 wheel command 输入。"""
    frozen = Path(__file__).resolve().parents[2] / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    assert frozen.is_file(), "frozen v2 descriptor is missing"
    descriptor = tmp_path / "v2.desc"
    descriptor.write_bytes(frozen.read_bytes())
    payload = tmp_path / "command.bin"
    message = pb.WheelCommand(
        timestamp_ns=1,
        drive_wheel_speed_rad_s=(1.0, 1.0),
        sequence=0,
        world_generation=1,
        command_generation=1,
        source_id="phase0",
        source_session_id=b"p" * 16,
        robot_model="df_back",
        simulation_session_id=b"s" * 16,
        descriptor_sha256=sha256(descriptor.read_bytes()).digest(),
    )
    payload.write_bytes(message.SerializeToString(deterministic=True))
    return descriptor, payload


@pytest.mark.stage4_artifact
def test_phase0_tools_report_frozen_abi() -> None:
    """golden 工具必须明确报告 C++ ABI、compiler、eCAL 和 Protobuf 身份。"""
    result = subprocess.run(
        [str(_phase0_executable("v2_golden")), "--version"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stdout.splitlines() == [
        "cxx=17",
        "compiler=gcc-13",
        "ecal=6.1.1",
        "protobuf=33.6",
        "glibcxx_cxx11_abi=1",
    ]


@pytest.mark.stage4_artifact
def test_raw_probe_retains_frozen_ecal_runtime_boundary() -> None:
    """raw probe 即使 dry-run 也必须保留已冻结的 eCAL ABI 依赖。"""
    probe = _phase0_executable("ecal_v2_raw_probe")
    dynamic = _run_abi_inspection(probe, "readelf", "-d")
    runtime = _run_abi_inspection(probe, "ldd")

    assert "Shared library: [libecal_core.so.6]" in dynamic
    assert "RUNPATH" in dynamic
    assert "conda" not in dynamic
    runpath = re.search(r"Library runpath: \[([^]]+)\]", dynamic)
    assert runpath is not None
    assert f"libecal_core.so.6 => {runpath.group(1)}/libecal_core.so.6" in runtime


def test_raw_probe_declares_callback_owned_envelope() -> None:
    """native callback 必须只复制 raw 输入，不得借用 eCAL 临时缓冲区。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "#include <ecal/ecal.h>" in source
    assert "struct RawEnvelope final" in source
    assert "std::vector<std::byte> payload" in source
    assert "std::chrono::steady_clock::time_point received_at" in source
    assert "RawEnvelope CopyEnvelope(" in source
    assert "const eCAL::SReceiveCallbackData& data" in source
    assert "data.buffer_size > 0 && data.buffer == nullptr" in source


def test_cpp_phase0_evidence_writers_use_posix_exclusive_create() -> None:
    """C++ 结果与 golden fixture 都必须以 O_EXCL 防止检查后覆盖。"""
    root = Path(__file__).resolve().parents[2] / "cpp" / "phase0"
    for source_path in (root / "ecal_v2_raw_probe.cpp", root / "v2_golden.cpp"):
        source = source_path.read_text(encoding="utf-8")
        assert "#include <fcntl.h>" in source
        assert "O_CREAT | O_EXCL" in source
        assert "::open(path.c_str()" in source
        assert "::fsync(" in source


def test_raw_probe_worker_hashes_before_metadata_and_protobuf_parse() -> None:
    """worker 必须先 hash，再验证本帧 metadata，最后才解析 Protobuf。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    start = source.index("ProcessedEnvelope ProcessEnvelope(")
    worker = source[start:]
    assert "stage4::Sha256" in worker
    assert "remote type/encoding/descriptor mismatch" in worker
    assert "ParseFromString" in worker
    assert worker.index("stage4::Sha256") < worker.index("remote type/encoding/descriptor mismatch")
    assert worker.index("remote type/encoding/descriptor mismatch") < worker.index("ParseFromString")


def test_raw_probe_uses_a_bounded_nonblocking_receive_lane() -> None:
    """callback lane 必须容量受限，满时不阻塞 native eCAL callback。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "class ReceiveLane final" in source
    assert "bool Push(RawEnvelope envelope)" in source
    assert "if (slot_)" in source
    assert "return false;" in source
    assert "std::optional<RawEnvelope> Take()" in source


def test_raw_probe_constructs_complete_ecal_raw_type_metadata() -> None:
    """raw pub/sub 必须把冻结 descriptor 作为 eCAL 的远端 metadata 发布。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "eCAL::SDataTypeInformation TypeInfo(" in source
    assert "info.name = type_name" in source
    assert 'info.encoding = "proto"' in source
    assert "info.descriptor = descriptor" in source


def test_raw_probe_reserves_one_raii_ecal_lifecycle_for_real_mode() -> None:
    """真实模式必须显式初始化并在所有退出路径 finalize eCAL。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "class EcalLifecycle final" in source
    assert "eCAL::Initialize(" in source
    assert "eCAL::Finalize()" in source
    assert "int RunRealProbe(const ProbePlan& plan)" in source
    assert "return RunRealProbe(plan);" in source


def test_raw_probe_wires_ecal_raw_publisher_and_subscriber() -> None:
    """真实路径必须使用 eCAL 原始 pub/sub 和 callback-owned envelope。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "eCAL::CPublisher publisher(" in source
    assert "eCAL::CSubscriber subscriber(" in source
    assert "subscriber.SetReceiveCallback(" in source
    assert "CopyEnvelope(type_info, data" in source
    assert "lane.Push(" in source


def test_raw_probe_publishes_the_original_payload_bytes() -> None:
    """publish 不能重新序列化 Protobuf，必须发送 Python 写入的原始文件 bytes。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "const std::string payload = ReadFile(*plan.payload);" in source
    assert "publisher.Send(payload.data(), payload.size())" in source
    assert "raw eCAL publisher send failed" in source


def test_raw_probe_requires_exactly_one_peer_before_real_io() -> None:
    """真实 publish/subscribe 必须在 deadline 内等待精确一个 peer。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "bool WaitForExactPeerCount(" in source
    assert "publisher.GetSubscriberCount()" in source
    assert "subscriber.GetPublisherCount()" in source
    assert "raw eCAL peer count did not reach exactly one" in source


def test_raw_probe_subscriber_waits_for_owned_envelope_then_runs_worker() -> None:
    """subscriber 只能由 worker 消费 callback 已复制的 envelope。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "std::optional<RawEnvelope> WaitForEnvelope(" in source
    assert "const auto envelope = WaitForEnvelope(lane, plan.deadline_ms);" in source
    assert "raw eCAL subscriber did not receive a frame" in source
    assert "ProcessEnvelope(*envelope, plan.type_name, descriptor)" in source


def test_raw_probe_writes_the_callback_owned_payload_without_reserializing() -> None:
    """subscriber 输出必须是 callback 复制的原始 bytes，而非解析后的消息重编码。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "void WriteNewFile(const fs::path& path, const std::string& payload)" in source
    assert "WriteNewFile(*plan.payload_out, received_payload);" in source
    assert "received_payload = std::string(" in source


def test_raw_probe_emits_result_with_payload_descriptor_and_peer_identity() -> None:
    """真实结果必须记录原始 payload、descriptor 摘要和精确 peer count。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "std::string BuildResultJson(" in source
    assert "payload_sha256" in source
    assert "descriptor_sha256" in source
    assert "peer_count" in source
    assert "WriteNewFile(plan.result, result_json);" in source
    assert "clean_shutdown" in source
    assert source.index("WriteNewFile(plan.result, result_json);") > source.index("// eCAL lifecycle")


def test_raw_probe_result_reports_actual_finalize_and_worker_verification() -> None:
    """C++ result 必须显式证明 finalize 与 subscriber worker 校验，禁止由 Python 默认补齐。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "bool Finalize()" in source
    assert ',\\"finalized\\":true' in source
    assert ',\\"protocol_state\\":\\"verified\\"' in source
    assert ',\\"worker_order\\":[\\"payload_sha256\\"' in source


def test_raw_probe_subscriber_result_records_callback_remote_metadata() -> None:
    """subscriber result 必须来自 callback 的远端 metadata 与传输时间字段。"""
    source = (
        Path(__file__).resolve().parents[2]
        / "cpp"
        / "phase0"
        / "ecal_v2_raw_probe.cpp"
    ).read_text(encoding="utf-8")
    assert "BuildSubscribeResultJson(" in source
    assert "envelope.remote_type_name" in source
    assert "envelope.remote_encoding" in source
    assert "envelope.remote_descriptor" in source
    assert "envelope.send_timestamp_us" in source
    assert "envelope.send_clock" in source


@pytest.mark.stage4_artifact
def test_v2_golden_decodes_python_wheel_command_bytes(tmp_path) -> None:
    """C++ golden 必须解析 Python 确定性编码的 WheelCommand 原始 bytes。"""
    golden = _phase0_executable("v2_golden")
    descriptor, payload = _valid_probe_inputs(tmp_path)
    result = subprocess.run(
        [
            str(golden),
            "decode",
            "--descriptor-set",
            str(descriptor),
            "WheelCommand",
            str(payload),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    document = json.loads(result.stdout)
    assert document["message_name"] == "WheelCommand"
    assert document["message"]["source_id"] == "phase0"
    assert document["message"]["simulation_session_id"] == "c3Nzc3Nzc3Nzc3Nzc3Nzcw=="
    assert document["descriptor_sha256"] == sha256(descriptor.read_bytes()).hexdigest()
    assert document["payload_sha256"] == sha256(payload.read_bytes()).hexdigest()


@pytest.mark.stage4_artifact
def test_v2_golden_decodes_every_top_level_v2_message(tmp_path) -> None:
    """C++ golden 的 decode 入口必须覆盖 v2 的全部五类顶层消息。"""
    golden = _phase0_executable("v2_golden")
    descriptor, _payload = _valid_probe_inputs(tmp_path)
    descriptor_sha256 = sha256(descriptor.read_bytes()).digest()
    for name in ("WheelCommand", "WheelState", "LidarPointCloud", "RtkState", "ImuAttitude"):
        message = getattr(pb, name)(
            simulation_session_id=b"s" * 16,
            descriptor_sha256=descriptor_sha256,
        )
        payload = tmp_path / f"{name}.bin"
        payload.write_bytes(message.SerializeToString(deterministic=True))
        result = subprocess.run(
            [
                str(golden),
                "decode",
                "--descriptor-set",
                str(descriptor),
                name,
                str(payload),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        document = json.loads(result.stdout)
        assert document["message_name"] == name
        assert document["descriptor_sha256"] == descriptor_sha256.hex()
        assert document["payload_sha256"] == sha256(payload.read_bytes()).hexdigest()


@pytest.mark.stage4_artifact
def test_v2_golden_emits_five_deterministic_fixtures(tmp_path) -> None:
    """C++ fixtures 必须覆盖五类顶层消息，且重复生成的 wire bytes 完全相同。"""
    golden = _phase0_executable("v2_golden")
    descriptor, _payload = _valid_probe_inputs(tmp_path)
    first = tmp_path / "first"
    second = tmp_path / "second"
    first.mkdir()
    second.mkdir()
    for output_dir in (first, second):
        subprocess.run(
            [
                str(golden),
                "encode-fixtures",
                "--descriptor-set",
                str(descriptor),
                "--output-dir",
                str(output_dir),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    expected = {f"{name}.bin" for name in ("WheelCommand", "WheelState", "LidarPointCloud", "RtkState", "ImuAttitude")}
    assert {path.name for path in first.glob("*.bin")} == expected
    assert (first / "manifest.json").is_file()
    assert {path.name: path.read_bytes() for path in first.iterdir()} == {
        path.name: path.read_bytes() for path in second.iterdir()
    }
    descriptor_sha256 = sha256(descriptor.read_bytes()).digest()
    for name in expected:
        message = getattr(pb, name.removesuffix(".bin"))()
        message.ParseFromString((first / name).read_bytes())
        assert bytes(message.descriptor_sha256) == descriptor_sha256


@pytest.mark.stage4_artifact
def test_raw_probe_accepts_complete_publish_and_subscribe_dry_runs(tmp_path) -> None:
    """dry-run 必须完成全部输入验证却不创建 output 或 participant。"""
    probe = _phase0_executable("ecal_v2_raw_probe")
    descriptor, payload = _valid_probe_inputs(tmp_path)
    publish_result = tmp_path / "publish.json"
    subscribe_payload = tmp_path / "received.bin"
    subscribe_result = tmp_path / "subscribe.json"
    cases = (
        (
            [str(probe), "--dry-run", "publish", "--topic", "/sim/wheel/command", "--type-name", "slope_sim.interfaces.v2.WheelCommand", "--descriptor-set", str(descriptor), "--payload", str(payload), "--result", str(publish_result), "--deadline-ms", "10000"],
            {"deadline_ms": 10000, "descriptor_set": str(descriptor), "dry_run": True, "mode": "publish", "payload": str(payload), "result": str(publish_result), "topic": "/sim/wheel/command", "type_name": "slope_sim.interfaces.v2.WheelCommand"},
        ),
        (
            [str(probe), "--dry-run", "subscribe", "--topic", "/sim/wheel/command", "--type-name", "slope_sim.interfaces.v2.WheelCommand", "--descriptor-set", str(descriptor), "--payload-out", str(subscribe_payload), "--result", str(subscribe_result), "--expected-peer-count", "1", "--deadline-ms", "10000"],
            {"deadline_ms": 10000, "descriptor_set": str(descriptor), "dry_run": True, "expected_peer_count": 1, "mode": "subscribe", "payload_out": str(subscribe_payload), "result": str(subscribe_result), "topic": "/sim/wheel/command", "type_name": "slope_sim.interfaces.v2.WheelCommand"},
        ),
    )
    for argv, expected in cases:
        completed = subprocess.run(argv, check=False, capture_output=True, text=True)
        assert completed.returncode == 0, completed.stderr
        assert completed.stderr == ""
        assert completed.stdout == json.dumps(expected, sort_keys=True, separators=(",", ":"), ensure_ascii=True) + "\n"
    assert not publish_result.exists()
    assert not subscribe_payload.exists()
    assert not subscribe_result.exists()


@pytest.mark.stage4_artifact
def test_raw_probe_rejects_bad_cli_matrix(tmp_path) -> None:
    """参数、路径与角色错误要在 eCAL 初始化前返回稳定类别退出码。"""
    probe = _phase0_executable("ecal_v2_raw_probe")
    descriptor, payload = _valid_probe_inputs(tmp_path)
    result = tmp_path / "result.json"
    common = ["--topic", "/sim/wheel/command", "--type-name", "slope_sim.interfaces.v2.WheelCommand", "--descriptor-set", str(descriptor)]
    cases = (
        ("missing", ["--dry-run", "publish", *common[2:], "--payload", str(payload), "--result", str(result), "--deadline-ms", "10000"], 64),
        ("duplicate", ["--dry-run", "publish", *common, "--topic", "/duplicate", "--payload", str(payload), "--result", str(result), "--deadline-ms", "10000"], 64),
        ("unknown", ["--dry-run", "publish", *common, "--payload", str(payload), "--result", str(result), "--deadline-ms", "10000", "--unknown"], 64),
        ("relative", ["--dry-run", "publish", *common[:4], "--descriptor-set", "relative.desc", "--payload", str(payload), "--result", str(result), "--deadline-ms", "10000"], 64),
        ("deadline", ["--dry-run", "publish", *common, "--payload", str(payload), "--result", str(result), "--deadline-ms", "0"], 64),
        ("wrong-role", ["--dry-run", "publish", *common, "--payload", str(payload), "--payload-out", str(tmp_path / "out.bin"), "--result", str(result), "--deadline-ms", "10000"], 64),
        ("output-alias", ["--dry-run", "subscribe", *common, "--payload-out", str(result), "--result", str(result), "--expected-peer-count", "1", "--deadline-ms", "10000"], 73),
        ("existing-output", ["--dry-run", "publish", *common, "--payload", str(payload), "--result", str(descriptor), "--deadline-ms", "10000"], 73),
    )
    for label, args, expected_rc in cases:
        completed = subprocess.run([str(probe), *args], check=False, capture_output=True, text=True)
        assert completed.returncode == expected_rc, label
        assert completed.stdout == "", label
        assert completed.stderr.startswith("error: "), label
