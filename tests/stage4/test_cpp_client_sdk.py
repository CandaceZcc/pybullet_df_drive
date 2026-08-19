# 阶段四 C1：验证 C++ SDK 复用冻结的五话题线协议元数据。
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb


def _phase0_build_directory() -> Path:
    """读取已验证的 C++ 构建根，避免测试自行猜测依赖前缀。"""
    raw_directory = os.environ.get("STAGE4_PHASE0_BUILD_DIR")
    assert raw_directory, "STAGE4_PHASE0_BUILD_DIR must name the verified C++ build"
    build_directory = Path(raw_directory)
    assert build_directory.is_absolute(), "C++ build directory must be absolute"
    assert build_directory.is_dir(), "verified C++ build directory is missing"
    return build_directory


def _client_tool(name: str) -> Path:
    """取得由已验证 CMake 根编译的 C++ SDK 工具。"""
    build_directory = _phase0_build_directory()
    subprocess.run(
        ["cmake", "--build", str(build_directory), "--target", name],
        check=True,
        capture_output=True,
        text=True,
    )
    executable = build_directory / name
    assert executable.is_file(), f"C++ SDK tool is missing: {executable}"
    assert os.access(executable, os.X_OK), f"C++ SDK tool is not executable: {executable}"
    return executable


def _command_tool() -> Path:
    """允许缺失 canonical 依赖时显式选择本轮隔离的 Command 测试二进制。"""
    override = os.environ.get("STAGE4_COMMAND_EXECUTABLE")
    if override is None:
        return _client_tool("slope_sim_stage4_command")
    executable = Path(override)
    assert executable.is_absolute() and executable.is_file()
    assert os.access(executable, os.X_OK)
    return executable


def _recorder_tool() -> Path:
    """允许缺失历史 C++ 安装前缀时选择本轮聚焦 Recorder 二进制。"""
    override = os.environ.get("STAGE4_RECORDER_EXECUTABLE")
    if override is None:
        return _client_tool("slope_sim_stage4_recorder")
    executable = Path(override)
    assert executable.is_absolute() and executable.is_file()
    assert os.access(executable, os.X_OK)
    return executable


def _gated_command_tool() -> Path:
    """本轮 gated Command 优先使用隔离二进制，RED 保持复用旧 fallback。"""
    candidate = Path("/tmp/stage4-command-gated-20260814/slope_sim_stage4_command")
    fallback = Path("/tmp/stage4-command-fallback-20260814/slope_sim_stage4_command")
    return candidate if candidate.is_file() else fallback


def _ros_build_directory() -> Path:
    """读取独立 Jazzy 构建根，保持核心 CMake 根不依赖 ROS。"""
    raw_directory = os.environ.get("STAGE4_ROS_BUILD_DIR")
    assert raw_directory, "STAGE4_ROS_BUILD_DIR must name the verified Jazzy C++ build"
    build_directory = Path(raw_directory)
    assert build_directory.is_absolute(), "Jazzy C++ build directory must be absolute"
    assert build_directory.is_dir(), "verified Jazzy C++ build directory is missing"
    return build_directory


def _ros2_bridge_tool() -> Path:
    """构建并取得独立 Jazzy Bridge，禁止让核心构建隐式拉入 ROS。"""
    build_directory = _ros_build_directory()
    subprocess.run(
        ["cmake", "--build", str(build_directory), "--target", "slope_sim_stage4_ros2_bridge"],
        check=True,
        capture_output=True,
        text=True,
    )
    executable = build_directory / "slope_sim_stage4_ros2_bridge"
    assert executable.is_file() and os.access(executable, os.X_OK)
    return executable


def _phase0_tool(name: str) -> Path:
    """构建并取得阶段 A raw eCAL participant，避免测试使用旧二进制。"""
    build_directory = _phase0_build_directory()
    subprocess.run(
        ["cmake", "--build", str(build_directory), "--target", name],
        check=True,
        capture_output=True,
        text=True,
    )
    executable = build_directory / name
    assert executable.is_file() and os.access(executable, os.X_OK)
    return executable


@pytest.mark.stage4_artifact
def test_cpp_recorder_rejects_missing_or_invalid_lidar_pattern_identity(
    tmp_path: Path,
) -> None:
    """Recorder CLI 必须在 eCAL 初始化前拒绝缺失、畸形或错误的冻结 pattern 身份。"""
    root = Path(__file__).resolve().parents[2]
    recorder = _recorder_tool()
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    base = [
        str(recorder),
        "--descriptor-set", str(descriptor),
        "--scene-id", "pattern-cli-test",
        "--simulation-session-id", "00112233445566778899aabbccddeeff",
        "--world-generation", "7",
        "--output", str(tmp_path / "session.mcap"),
        "--expected-count", "1",
        "--deadline-ms", "1000",
        "--result", str(tmp_path / "result.json"),
    ]
    cases = (
        (["--lidar-pattern-version", "livox-mid360-800000-v1"], "lidar pattern identity"),
        (["--lidar-pattern-sha256", "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca"], "lidar pattern identity"),
        (["--lidar-pattern-version", "unknown", "--lidar-pattern-sha256", "0" * 64], "lidar-pattern-version"),
        (["--lidar-pattern-version", "livox-mid360-800000-v1", "--lidar-pattern-sha256", "short"], "lidar-pattern-sha256"),
        (["--lidar-pattern-version", "livox-mid360-800000-v1", "--lidar-pattern-sha256", "0" * 64], "lidar-pattern-sha256"),
    )
    for extra, expected_error in cases:
        completed = subprocess.run(
            [*base, *extra],
            check=False,
            capture_output=True,
            text=True,
        )
        assert completed.returncode == 64
        assert expected_error in completed.stderr
        assert not (tmp_path / "session.mcap").exists()
        assert not (tmp_path / "result.json").exists()


@pytest.mark.stage4_artifact
def test_cpp_recorder_accepts_explicit_per_topic_counts_and_binds_result(
    tmp_path: Path,
) -> None:
    """离线 Golf 窗口可逐 topic 定数，结果必须绑定唯一 MCAP 路径。"""
    root = Path(__file__).resolve().parents[2]
    recorder = _recorder_tool()
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    output = tmp_path / "session.mcap"
    result = tmp_path / "result.json"
    completed = subprocess.run(
        [
            str(recorder),
            "--descriptor-set", str(descriptor),
            "--scene-id", "mid360-golf-explicit-counts",
            "--simulation-session-id", "00112233445566778899aabbccddeeff",
            "--world-generation", "1",
            "--lidar-pattern-version", "livox-mid360-800000-v1",
            "--lidar-pattern-sha256", "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca",
            "--output", str(output),
            "--expected-wheel-command-count", "1",
            "--expected-wheel-state-count", "2",
            "--expected-lidar-points-count", "3",
            "--expected-rtk-state-count", "4",
            "--expected-imu-attitude-count", "5",
            "--deadline-ms", "1",
            "--result", str(result),
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )

    # 无 publisher 的 1 ms 窗口会按预期失败，但必须通过 CLI 并留下安全结果。
    assert completed.returncode == 1, completed.stderr
    document = json.loads(result.read_text(encoding="utf-8"))
    assert document["clean_shutdown"] is False
    assert document["mcap"] == str(output)
    assert document["topics"] == {
        "/sim/wheel/command": 0,
        "/sim/wheel/state": 0,
        "/sim/lidar/points": 0,
        "/sim/rtk/state": 0,
        "/sim/imu/attitude": 0,
    }


@pytest.mark.stage4_artifact
def test_cpp_replay_tool_is_registered() -> None:
    """隔离 Replay 必须作为独立 C++ participant 构建，不能借用实时 Simulator。"""
    replay = _client_tool("slope_sim_stage4_replay")
    assert replay.name == "slope_sim_stage4_replay"


@pytest.mark.stage4_artifact
def test_cpp_ros2_bridge_tool_is_registered() -> None:
    """独立 Jazzy Bridge 初始化后必须报告 ready，且不进入核心构建。"""
    bridge = _ros2_bridge_tool()
    completed = subprocess.run(
        [
            "zsh",
            "-lc",
            'source /opt/ros/jazzy/setup.zsh && exec "$1" --dry-run',
            "stage4_ros2_bridge",
            str(bridge),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == "ros2_bridge=ready\n"
    assert completed.stderr == ""


@pytest.mark.stage4_artifact
def test_cpp_client_sdk_exposes_exact_five_v2_topic_contracts() -> None:
    """SDK 的唯一 topic 表必须编译并固定匹配 Python v2 线协议。"""
    build_directory = _phase0_build_directory()
    subprocess.run(
        [
            "cmake",
            "--build",
            str(build_directory),
            "--target",
            "slope_sim_client_topic_contract_test",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    executable = build_directory / "slope_sim_client_topic_contract_test"
    assert executable.is_file(), "C++ SDK topic-contract test executable is missing"
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


@pytest.mark.stage4_artifact
def test_cpp_recorder_queue_rejects_overflow_without_dropping_prior_frame() -> None:
    """Recorder 队列满时必须显式拒绝新帧，保留已接受的原始 bytes。"""
    build_directory = _phase0_build_directory()
    subprocess.run(
        [
            "cmake",
            "--build",
            str(build_directory),
            "--target",
            "slope_sim_client_recorder_queue_test",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    executable = build_directory / "slope_sim_client_recorder_queue_test"
    assert executable.is_file(), "C++ Recorder queue test executable is missing"
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


@pytest.mark.stage4_artifact
def test_cpp_recorder_atomic_segment_hides_partial_output_until_finalize() -> None:
    """Recorder 临时 segment 只能在 fsync 后原子发布为最终文件。"""
    build_directory = _phase0_build_directory()
    subprocess.run(
        [
            "cmake",
            "--build",
            str(build_directory),
            "--target",
            "slope_sim_client_atomic_segment_test",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    executable = build_directory / "slope_sim_client_atomic_segment_test"
    assert executable.is_file(), "C++ atomic segment test executable is missing"
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


@pytest.mark.stage4_artifact
def test_cpp_recorder_writes_readable_mcap_with_five_raw_topics() -> None:
    """Recorder 必须以官方 MCAP reader 读回五 topic 原始 bytes 和会话 manifest。"""
    build_directory = _phase0_build_directory()
    subprocess.run(
        [
            "cmake",
            "--build",
            str(build_directory),
            "--target",
            "slope_sim_client_mcap_session_writer_test",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    executable = build_directory / "slope_sim_client_mcap_session_writer_test"
    assert executable.is_file(), "C++ MCAP session writer test executable is missing"
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


@pytest.mark.stage4_artifact
def test_cpp_recorder_session_defers_io_and_requires_safe_stop_on_overflow() -> None:
    """Recorder callback 仅入队；溢出必须阻止继续持久化并要求安全停车。"""
    build_directory = _phase0_build_directory()
    subprocess.run(
        [
            "cmake",
            "--build",
            str(build_directory),
            "--target",
            "slope_sim_client_recorder_session_test",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    executable = build_directory / "slope_sim_client_recorder_session_test"
    assert executable.is_file(), "C++ Recorder session test executable is missing"
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


@pytest.mark.stage4_artifact
def test_cpp_recorder_participant_records_all_five_v2_topics(tmp_path: Path) -> None:
    """独立 Recorder 必须接受离线期限并以真实 eCAL 原子录制五 topic。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    recorder = _client_tool("slope_sim_stage4_recorder")
    probe = _phase0_tool("ecal_v2_raw_probe")
    recording = tmp_path / "session.mcap"
    recorder_result = tmp_path / "recorder.json"
    recorder_process = subprocess.Popen(
        [
            str(recorder),
            "--descriptor-set",
            str(descriptor),
            "--scene-id",
            "recorder-integration-scene",
            "--simulation-session-id",
            "00112233445566778899aabbccddeeff",
            "--world-generation",
            "7",
            "--lidar-pattern-version",
            "livox-mid360-800000-v1",
            "--lidar-pattern-sha256",
            "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca",
            "--output",
            str(recording),
            "--expected-count",
            "1",
            "--deadline-ms",
            "21600000",
            "--result",
            str(recorder_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    topic_inputs = (
        ("/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand", "WheelCommand.bin"),
        ("/sim/wheel/state", "slope_sim.interfaces.v2.WheelState", "WheelState.bin"),
        ("/sim/lidar/points", "slope_sim.interfaces.v2.LidarPointCloud", "LidarPointCloud.bin"),
        ("/sim/rtk/state", "slope_sim.interfaces.v2.RtkState", "RtkState.bin"),
        ("/sim/imu/attitude", "slope_sim.interfaces.v2.ImuAttitude", "ImuAttitude.bin"),
    )
    try:
        # eCAL discovery is asynchronous; do not publish before all five subscribers have registered.
        time.sleep(1.0)
        assert recorder_process.poll() is None, recorder_process.stderr.read()
        for index, (topic, type_name, fixture) in enumerate(topic_inputs):
            published = subprocess.run(
                [
                    str(probe),
                    "publish",
                    "--topic",
                    topic,
                    "--type-name",
                    type_name,
                    "--descriptor-set",
                    str(descriptor),
                    "--payload",
                    str(root / "tests/fixtures/stage4/v2" / fixture),
                    "--result",
                    str(tmp_path / f"publisher-{index}.json"),
                    "--deadline-ms",
                    "3000",
                ],
                capture_output=True,
                text=True,
                timeout=8,
            )
            assert published.returncode == 0, f"topic={topic}\n{published.stderr}"
        stdout, stderr = recorder_process.communicate(timeout=8)
    finally:
        if recorder_process.poll() is None:
            recorder_process.kill()
            recorder_process.communicate()

    assert recorder_process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
    assert recording.read_bytes().startswith(b"\x89MCAP0\r\n")
    assert json.loads(recorder_result.read_text(encoding="utf-8")) == {
        "clean_shutdown": True,
        "mcap": str(recording),
        "recorded_count": 5,
        "role": "recorder",
        "topics": {topic: 1 for topic, _, _ in topic_inputs},
    }


@pytest.mark.stage4_artifact
def test_cpp_recorder_fault_result_requires_safe_stop_and_hides_partial_mcap(tmp_path: Path) -> None:
    """重复 sequence 的 raw frame 必须在入队前安全停车，不能发布部分 MCAP。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    recorder = _client_tool("slope_sim_stage4_recorder")
    probe = _phase0_tool("ecal_v2_raw_probe")
    recording = tmp_path / "faulted.mcap"
    recorder_result = tmp_path / "recorder-fault.json"
    recorder_process = subprocess.Popen(
        [
            str(recorder),
            "--descriptor-set",
            str(descriptor),
            "--scene-id",
            "recorder-fault-scene",
            "--simulation-session-id",
            "00112233445566778899aabbccddeeff",
            "--world-generation",
            "7",
            "--lidar-pattern-version",
            "livox-mid360-800000-v1",
            "--lidar-pattern-sha256",
            "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca",
            "--output",
            str(recording),
            "--expected-count",
            "1",
            "--deadline-ms",
            "20000",
            "--result",
            str(recorder_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        time.sleep(1.0)
        for index in range(2):
            published = subprocess.run(
                [
                    str(probe),
                    "publish",
                    "--topic",
                    "/sim/wheel/command",
                    "--type-name",
                    "slope_sim.interfaces.v2.WheelCommand",
                    "--descriptor-set",
                    str(descriptor),
                    "--payload",
                    str(root / "tests/fixtures/stage4/v2/WheelCommand.bin"),
                    "--result",
                    str(tmp_path / f"duplicate-publisher-{index}.json"),
                    "--deadline-ms",
                    "3000",
                ],
                capture_output=True,
                text=True,
                timeout=8,
            )
            assert published.returncode == 0, published.stderr
        stdout, stderr = recorder_process.communicate(timeout=8)
    finally:
        if recorder_process.poll() is None:
            recorder_process.kill()
            recorder_process.communicate()

    assert recorder_process.returncode != 0, f"stdout={stdout}\nstderr={stderr}"
    assert not recording.exists()
    assert json.loads(recorder_result.read_text(encoding="utf-8")) == {
        "clean_shutdown": False,
        "fault_reason": "/sim/wheel/command sequence is not continuous",
        "mcap": str(recording),
        "recorded_count": 1,
        "role": "recorder",
        "safe_stop_required": True,
        "topics": {
            "/sim/wheel/command": 1,
            "/sim/wheel/state": 0,
            "/sim/lidar/points": 0,
            "/sim/rtk/state": 0,
            "/sim/imu/attitude": 0,
        },
    }


@pytest.mark.stage4_artifact
def test_cpp_command_allows_runtime_and_recorder_to_subscribe_to_command_topic(tmp_path: Path) -> None:
    """唯一 Command 必须允许 runtime 与只读 Recorder 同时消费 command topic。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    command = _client_tool("slope_sim_stage4_command")
    recorder = _client_tool("slope_sim_stage4_recorder")
    probe = _phase0_tool("ecal_v2_raw_probe")
    recorder_result = tmp_path / "recorder-fault.json"
    recorder_process = subprocess.Popen(
        [
            str(recorder),
            "--descriptor-set",
            str(descriptor),
            "--scene-id",
            "command-consumer-scene",
            "--simulation-session-id",
            "00112233445566778899aabbccddeeff",
            "--world-generation",
            "7",
            "--lidar-pattern-version",
            "livox-mid360-800000-v1",
            "--lidar-pattern-sha256",
            "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca",
            "--output",
            str(tmp_path / "unfinished.mcap"),
            "--expected-count",
            "15",
            "--deadline-ms",
            "10000",
            "--result",
            str(recorder_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    received = tmp_path / "runtime-received.bin"
    subscriber_result = tmp_path / "runtime-subscriber.json"
    subscriber_process = subprocess.Popen(
        [
            str(probe),
            "subscribe",
            "--topic",
            "/sim/wheel/command",
            "--type-name",
            "slope_sim.interfaces.v2.WheelCommand",
            "--descriptor-set",
            str(descriptor),
            "--payload-out",
            str(received),
            "--result",
            str(subscriber_result),
            "--expected-peer-count",
            "1",
            "--deadline-ms",
            "10000",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        time.sleep(3.0)
        command_result = tmp_path / "command.json"
        completed = subprocess.run(
            [
                str(command),
                "--descriptor-set",
                str(descriptor),
                "--payload",
                str(payload),
                "--duration-ms",
                "150",
                "--deadline-ms",
                "3000",
                "--result",
                str(command_result),
            ],
            capture_output=True,
            text=True,
            timeout=8,
        )
        assert completed.returncode == 0, completed.stderr
        subscriber_stdout, subscriber_stderr = subscriber_process.communicate(timeout=8)
    finally:
        for process in (subscriber_process, recorder_process):
            if process.poll() is None:
                process.kill()
                process.communicate()

    assert subscriber_process.returncode == 0, (
        f"stdout={subscriber_stdout}\nstderr={subscriber_stderr}"
    )
    assert received.read_bytes() == payload.read_bytes()


@pytest.mark.stage4_artifact
def test_cpp_client_validates_python_wheel_command_raw_bytes_and_descriptor(
    tmp_path: Path,
) -> None:
    """SDK 必须只接受冻结 descriptor 身份绑定的 Python WheelCommand 原始 bytes。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    tool = _client_tool("slope_sim_client_validate_wheel_command")

    accepted = subprocess.run(
        [str(tool), "--descriptor-set", str(descriptor), "--payload", str(payload)],
        capture_output=True,
        text=True,
    )
    assert accepted.returncode == 0, accepted.stderr
    assert accepted.stdout == "valid\n"

    tampered_descriptor = tmp_path / "tampered.desc"
    tampered_descriptor.write_bytes(descriptor.read_bytes() + b"x")
    rejected = subprocess.run(
        [
            str(tool),
            "--descriptor-set",
            str(tampered_descriptor),
            "--payload",
            str(payload),
        ],
        capture_output=True,
        text=True,
    )
    assert rejected.returncode == 66
    assert "descriptor digest differs" in rejected.stderr


@pytest.mark.parametrize(
    ("topic", "fixture"),
    (
        ("/sim/wheel/command", "WheelCommand.bin"),
        ("/sim/wheel/state", "WheelState.bin"),
        ("/sim/lidar/points", "LidarPointCloud.bin"),
        ("/sim/rtk/state", "RtkState.bin"),
        ("/sim/imu/attitude", "ImuAttitude.bin"),
    ),
)
@pytest.mark.stage4_artifact
def test_cpp_client_validates_all_python_v2_topic_payloads(topic: str, fixture: str) -> None:
    """SDK 必须按五 topic 的各自顶层类型验证冻结 Python raw bytes。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2" / fixture
    tool = _client_tool("slope_sim_client_validate_v2_payload")
    completed = subprocess.run(
        [str(tool), "--topic", topic, "--descriptor-set", str(descriptor), "--payload", str(payload)],
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == "valid\n"


@pytest.mark.stage4_artifact
def test_cpp_client_rejects_sparse_wheel_state_bytes_on_lidar_topic(tmp_path: Path) -> None:
    """SDK 不得把只含共同身份字段的 WheelState 误作 LiDAR 原始 payload。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = tmp_path / "sparse-wheel-state.bin"
    payload.write_bytes(
        pb.WheelState(
            simulation_session_id=bytes.fromhex("00112233445566778899aabbccddeeff"),
            descriptor_sha256=hashlib.sha256(descriptor.read_bytes()).digest(),
        ).SerializeToString(deterministic=True)
    )
    tool = _client_tool("slope_sim_client_validate_v2_payload")

    rejected = subprocess.run(
        [
            str(tool),
            "--topic",
            "/sim/lidar/points",
            "--descriptor-set",
            str(descriptor),
            "--payload",
            str(payload),
        ],
        capture_output=True,
        text=True,
    )

    assert rejected.returncode == 66
    assert "payload does not match the v2 topic type" in rejected.stderr


@pytest.mark.stage4_artifact
def test_cpp_client_rejects_v2_payload_with_unknown_field(tmp_path: Path) -> None:
    """SDK 必须拒绝 protobuf 解析后仍保留未知字段的原始 bytes。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = tmp_path / "lidar-with-unknown-field.bin"
    payload.write_bytes(
        (root / "tests/fixtures/stage4/v2/LidarPointCloud.bin").read_bytes()
        + b"\xf8\x07\x01"
    )
    tool = _client_tool("slope_sim_client_validate_v2_payload")

    rejected = subprocess.run(
        [
            str(tool),
            "--topic",
            "/sim/lidar/points",
            "--descriptor-set",
            str(descriptor),
            "--payload",
            str(payload),
        ],
        capture_output=True,
        text=True,
    )

    assert rejected.returncode == 66
    assert "payload does not match the v2 topic type" in rejected.stderr


@pytest.mark.parametrize(
    ("topic", "fixture", "nested_member"),
    (
        ("/sim/lidar/points", "LidarPointCloud.bin", "point"),
        ("/sim/rtk/state", "RtkState.bin", "left"),
    ),
)
@pytest.mark.stage4_artifact
def test_cpp_client_rejects_v2_payload_with_nested_unknown_field(
    tmp_path: Path,
    topic: str,
    fixture: str,
    nested_member: str,
) -> None:
    """SDK 必须拒绝任一嵌套 protobuf message 保留未知字段的原始 bytes。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = tmp_path / f"{Path(fixture).stem}-nested-unknown.bin"
    if nested_member == "point":
        message = pb.LidarPointCloud()
        message.ParseFromString((root / "tests/fixtures/stage4/v2" / fixture).read_bytes())
        message.points[0].MergeFromString(b"\xf8\x07\x01")
    else:
        message = pb.RtkState()
        message.ParseFromString((root / "tests/fixtures/stage4/v2" / fixture).read_bytes())
        message.left.MergeFromString(b"\xf8\x07\x01")
    payload.write_bytes(message.SerializeToString(deterministic=True))
    tool = _client_tool("slope_sim_client_validate_v2_payload")

    rejected = subprocess.run(
        [
            str(tool),
            "--topic",
            topic,
            "--descriptor-set",
            str(descriptor),
            "--payload",
            str(payload),
        ],
        capture_output=True,
        text=True,
    )

    assert rejected.returncode == 66
    assert "payload does not match the v2 topic type" in rejected.stderr


@pytest.mark.stage4_artifact
def test_cpp_client_rejects_lidar_point_count_mismatch(tmp_path: Path) -> None:
    """SDK 必须拒绝 point_num 与嵌套点数不一致的 LiDAR 原始 payload。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = tmp_path / "lidar-point-count-mismatch.bin"
    message = pb.LidarPointCloud()
    message.ParseFromString((root / "tests/fixtures/stage4/v2/LidarPointCloud.bin").read_bytes())
    message.point_num += 1
    payload.write_bytes(message.SerializeToString(deterministic=True))
    tool = _client_tool("slope_sim_client_validate_v2_payload")

    rejected = subprocess.run(
        [
            str(tool),
            "--topic",
            "/sim/lidar/points",
            "--descriptor-set",
            str(descriptor),
            "--payload",
            str(payload),
        ],
        capture_output=True,
        text=True,
    )

    assert rejected.returncode == 66
    assert "payload does not match the v2 topic type" in rejected.stderr


@pytest.mark.stage4_artifact
def test_cpp_client_rejects_v2_payload_with_short_simulation_session(tmp_path: Path) -> None:
    """SDK 必须拒绝未绑定 16-byte 仿真 session 的 v2 原始 payload。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = tmp_path / "lidar-short-session.bin"
    message = pb.LidarPointCloud()
    message.ParseFromString((root / "tests/fixtures/stage4/v2/LidarPointCloud.bin").read_bytes())
    message.simulation_session_id = b"short-session-i"
    payload.write_bytes(message.SerializeToString(deterministic=True))
    tool = _client_tool("slope_sim_client_validate_v2_payload")

    rejected = subprocess.run(
        [
            str(tool),
            "--topic",
            "/sim/lidar/points",
            "--descriptor-set",
            str(descriptor),
            "--payload",
            str(payload),
        ],
        capture_output=True,
        text=True,
    )

    assert rejected.returncode == 66
    assert "payload does not match the v2 topic type" in rejected.stderr


@pytest.mark.stage4_artifact
def test_cpp_client_rejects_v2_payload_with_zero_world_generation(tmp_path: Path) -> None:
    """SDK 必须拒绝未绑定已建立 world generation 的 v2 原始 payload。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = tmp_path / "lidar-zero-world-generation.bin"
    message = pb.LidarPointCloud()
    message.ParseFromString((root / "tests/fixtures/stage4/v2/LidarPointCloud.bin").read_bytes())
    message.world_generation = 0
    payload.write_bytes(message.SerializeToString(deterministic=True))
    tool = _client_tool("slope_sim_client_validate_v2_payload")

    rejected = subprocess.run(
        [
            str(tool),
            "--topic",
            "/sim/lidar/points",
            "--descriptor-set",
            str(descriptor),
            "--payload",
            str(payload),
        ],
        capture_output=True,
        text=True,
    )

    assert rejected.returncode == 66
    assert "payload does not match the v2 topic type" in rejected.stderr


@pytest.mark.parametrize("invalid_case", ("short-session", "zero-generation", "unknown-field"))
@pytest.mark.stage4_artifact
def test_cpp_command_dry_run_rejects_payload_outside_shared_v2_contract(
    tmp_path: Path,
    invalid_case: str,
) -> None:
    """Command dry-run 必须和共享 WheelCommand topic 判据拒绝相同的非法 payload。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = tmp_path / f"wheel-command-{invalid_case}.bin"
    raw_payload = (root / "tests/fixtures/stage4/v2/WheelCommand.bin").read_bytes()
    if invalid_case == "unknown-field":
        payload.write_bytes(raw_payload + b"\xf8\x07\x01")
    else:
        message = pb.WheelCommand()
        message.ParseFromString(raw_payload)
        if invalid_case == "short-session":
            message.simulation_session_id = b"short-session-i"
        else:
            message.world_generation = 0
        payload.write_bytes(message.SerializeToString(deterministic=True))
    command = _client_tool("slope_sim_stage4_command")

    rejected = subprocess.run(
        [
            str(command),
            "--dry-run",
            "--descriptor-set",
            str(descriptor),
            "--payload",
            str(payload),
            "--duration-ms",
            "100",
        ],
        capture_output=True,
        text=True,
    )

    assert rejected.returncode == 66
    assert "payload is not a valid WheelCommand" in rejected.stderr


@pytest.mark.stage4_artifact
def test_cpp_client_command_lease_stops_active_steering_at_100ms_boundary() -> None:
    """SDK 命令租约到达 100 ms 时必须输出完整 4+2 零命令。"""
    executable = _client_tool("slope_sim_client_command_lease_test")
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


@pytest.mark.stage4_artifact
def test_cpp_client_command_lock_rejects_second_process() -> None:
    """SDK 单实例锁必须在另一进程已持有时立即拒绝第二个 Command。"""
    executable = _client_tool("slope_sim_client_command_lock_test")
    subprocess.run([str(executable)], check=True, capture_output=True, text=True)


@pytest.mark.stage4_artifact
def test_cpp_command_dry_run_accepts_only_validated_raw_wheel_command() -> None:
    """正式 Command 的 dry-run 必须先复用 SDK raw-byte/descriptor 判据。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    command = _client_tool("slope_sim_stage4_command")

    accepted = subprocess.run(
        [
            str(command),
            "--dry-run",
            "--descriptor-set",
            str(descriptor),
            "--payload",
            str(payload),
            "--duration-ms",
            "100",
        ],
        capture_output=True,
        text=True,
    )
    assert accepted.returncode == 0, accepted.stderr
    assert accepted.stdout == "validated command: /sim/wheel/command 100Hz\n"


@pytest.mark.stage4_artifact
def test_cpp_command_schedule_requires_all_three_options(tmp_path: Path) -> None:
    """Schedule 参数必须完整出现，不能默默退回单 payload 租约模式。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"

    rejected = subprocess.run(
        [
            str(_command_tool()),
            "--dry-run",
            "--descriptor-set", str(descriptor),
            "--payload", str(payload),
            "--turn-payload", str(payload),
            "--turn-at-ms", "100",
            "--duration-ms", "300",
        ],
        capture_output=True,
        text=True,
    )

    assert rejected.returncode == 66
    assert "schedule options must be provided together" in rejected.stderr


@pytest.mark.parametrize(
    ("turn_at_ms", "stop_at_ms", "message"),
    (
        ("0", "200", "turn-at-ms must be a positive 10 ms multiple"),
        ("105", "200", "turn-at-ms must be a positive 10 ms multiple"),
        ("200", "100", "schedule must satisfy 0 < turn-at-ms < stop-at-ms < duration-ms"),
        ("100", "300", "schedule must satisfy 0 < turn-at-ms < stop-at-ms < duration-ms"),
    ),
)
@pytest.mark.stage4_artifact
def test_cpp_command_schedule_rejects_invalid_offsets(
    tmp_path: Path,
    turn_at_ms: str,
    stop_at_ms: str,
    message: str,
) -> None:
    """Schedule 边界必须按 10 ms tick 对齐并严格落在运行窗口内。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"

    rejected = subprocess.run(
        [
            str(_command_tool()),
            "--dry-run",
            "--descriptor-set", str(descriptor),
            "--payload", str(payload),
            "--turn-payload", str(payload),
            "--turn-at-ms", turn_at_ms,
            "--stop-at-ms", stop_at_ms,
            "--duration-ms", "300",
        ],
        capture_output=True,
        text=True,
    )

    assert rejected.returncode == 66
    assert message in rejected.stderr


@pytest.mark.stage4_artifact
def test_cpp_command_schedule_rejects_mismatched_payload_identity(tmp_path: Path) -> None:
    """转向模板只能改变轮速，不能切换 command owner 或仿真身份。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    straight = pb.WheelCommand()
    straight.ParseFromString(
        (root / "tests/fixtures/stage4/v2/WheelCommand.bin").read_bytes()
    )
    turn = pb.WheelCommand()
    turn.CopyFrom(straight)
    turn.source_id = f"{straight.source_id}.other"
    straight_path = tmp_path / "straight.bin"
    turn_path = tmp_path / "turn.bin"
    straight_path.write_bytes(straight.SerializeToString(deterministic=True))
    turn_path.write_bytes(turn.SerializeToString(deterministic=True))

    rejected = subprocess.run(
        [
            str(_command_tool()),
            "--dry-run",
            "--descriptor-set", str(descriptor),
            "--payload", str(straight_path),
            "--turn-payload", str(turn_path),
            "--turn-at-ms", "100",
            "--stop-at-ms", "200",
            "--duration-ms", "300",
        ],
        capture_output=True,
        text=True,
    )

    assert rejected.returncode == 66
    assert "schedule payload identity differs" in rejected.stderr


@pytest.mark.stage4_artifact
def test_cpp_command_gated_start_requires_exact_runtime_and_recorder_subscribers(
    tmp_path: Path,
) -> None:
    """gated Command 只接受完整 marker 三元组和精确两个 command subscribers。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    ready = tmp_path / "command.ready"
    start = tmp_path / "start.signal"
    command = _gated_command_tool()
    base = [
        str(command), "--dry-run", "--descriptor-set", str(descriptor),
        "--payload", str(payload), "--duration-ms", "100",
    ]
    default = subprocess.run(base, capture_output=True, text=True)
    assert default.returncode == 0, default.stderr
    incomplete = subprocess.run(
        [*base, "--ready-file", str(ready)], capture_output=True, text=True
    )
    assert incomplete.returncode == 66
    coordinated = subprocess.run(
        [*base, "--ready-file", str(ready), "--start-file", str(start),
         "--expected-subscriber-count", "2"],
        capture_output=True, text=True,
    )
    assert coordinated.returncode == 0, coordinated.stderr
    invalid_count = subprocess.run(
        [*base, "--ready-file", str(ready), "--start-file", str(start),
         "--expected-subscriber-count", "1"],
        capture_output=True, text=True,
    )
    assert invalid_count.returncode == 66


@pytest.mark.stage4_artifact
def test_cpp_command_shared_start_precedes_schedule_clock(tmp_path: Path) -> None:
    """精确两个 endpoint 就绪后，Command 必须等 shared start 才开始 100 Hz 窗口。"""
    from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
    from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings

    root = Path(__file__).resolve().parents[2]
    descriptor_path = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    ready = tmp_path / "command.ready"
    start = tmp_path / "start.signal"
    result = tmp_path / "command.json"
    bindings = EcalRawBindings()
    assert bindings._core.initialize("stage4-command-gated-start-test", 0x3F) is not False
    descriptor = load_v2_descriptor()
    first = bindings.create_subscriber(
        "/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand", descriptor, lambda _frame: None
    )
    second = bindings.create_subscriber(
        "/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand", descriptor, lambda _frame: None
    )
    process = subprocess.Popen(
        [
            str(_gated_command_tool()), "--descriptor-set", str(descriptor_path),
            "--payload", str(payload), "--duration-ms", "100", "--deadline-ms", "3000",
            "--result", str(result), "--ready-file", str(ready), "--start-file", str(start),
            "--expected-subscriber-count", "2",
        ],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        deadline = time.monotonic() + 2.0
        while not ready.exists() and process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert ready.exists(), process.stderr.read()
        assert process.poll() is None
        start.touch(exist_ok=False)
        stdout, stderr = process.communicate(timeout=5)
    finally:
        first.remove_receive_callback()
        second.remove_receive_callback()
        assert bindings._core.finalize() is not False
        if process.poll() is None:
            process.kill()
            process.communicate()

    assert process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
    assert json.loads(result.read_text(encoding="utf-8"))["published_count"] == 10


@pytest.mark.stage4_artifact
def test_cpp_command_continuous_forward_turn_stop_schedule(tmp_path: Path) -> None:
    """显式 schedule 必须逐个 10 ms tick 续租并发布直行、转向、零速三段。"""
    from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
    from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings

    root = Path(__file__).resolve().parents[2]
    descriptor_path = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    descriptor = load_v2_descriptor()
    straight = pb.WheelCommand()
    straight.ParseFromString(
        (root / "tests/fixtures/stage4/v2/WheelCommand.bin").read_bytes()
    )
    straight.sequence = 0
    straight.robot_model = "df_mid"
    del straight.drive_wheel_speed_rad_s[:]
    straight.drive_wheel_speed_rad_s.extend((4.0, 4.0))
    del straight.steering_wheel_speed_rad_s[:]
    turn = pb.WheelCommand()
    turn.CopyFrom(straight)
    del turn.drive_wheel_speed_rad_s[:]
    turn.drive_wheel_speed_rad_s.extend((2.875, 4.125))
    straight_path = tmp_path / "straight.bin"
    turn_path = tmp_path / "turn.bin"
    straight_path.write_bytes(straight.SerializeToString(deterministic=True))
    turn_path.write_bytes(turn.SerializeToString(deterministic=True))

    received = []
    bindings = EcalRawBindings()
    assert bindings._core.initialize("stage4-command-schedule-test", 0x3F) is not False
    subscriber = bindings.create_subscriber(
        "/sim/wheel/command",
        "slope_sim.interfaces.v2.WheelCommand",
        descriptor,
        received.append,
    )
    command_result = tmp_path / "command.json"
    try:
        completed = subprocess.run(
            [
                str(_command_tool()),
                "--descriptor-set", str(descriptor_path),
                "--payload", str(straight_path),
                "--turn-payload", str(turn_path),
                "--turn-at-ms", "100",
                "--stop-at-ms", "200",
                "--duration-ms", "300",
                "--deadline-ms", "3000",
                "--result", str(command_result),
            ],
            capture_output=True,
            text=True,
            timeout=8,
        )
    finally:
        subscriber.remove_receive_callback()
        assert bindings._core.finalize() is not False

    assert completed.returncode == 0, completed.stderr
    frames = []
    for envelope in received:
        frame = pb.WheelCommand()
        frame.ParseFromString(envelope.payload)
        frames.append(frame)
    assert [frame.sequence for frame in frames] == list(range(30))
    assert [tuple(frame.drive_wheel_speed_rad_s) for frame in frames] == (
        [(4.0, 4.0)] * 10
        + [(2.875, 4.125)] * 10
        + [(0.0, 0.0)] * 10
    )
    assert all(tuple(frame.steering_wheel_speed_rad_s) == () for frame in frames)
    assert {
        (
            bytes(frame.simulation_session_id),
            frame.world_generation,
            frame.command_generation,
            frame.source_id,
            bytes(frame.source_session_id),
            frame.robot_model,
        )
        for frame in frames
    } == {
        (
            bytes(straight.simulation_session_id),
            straight.world_generation,
            straight.command_generation,
            straight.source_id,
            bytes(straight.source_session_id),
            "df_mid",
        )
    }
    assert json.loads(command_result.read_text(encoding="utf-8")) == {
        "active_published_count": 30,
        "clean_shutdown": True,
        "published_count": 30,
        "safe_stop_published_count": 0,
        "transport": "ecal",
    }


@pytest.mark.stage4_artifact
def test_cpp_command_publishes_python_raw_command_to_exactly_one_ecal_peer(
    tmp_path: Path,
) -> None:
    """真实 Command 必须以冻结 metadata 向一个 C++ raw subscriber 发送原始 bytes。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    command = _client_tool("slope_sim_stage4_command")
    probe = _phase0_tool("ecal_v2_raw_probe")
    received = tmp_path / "received.bin"
    subscriber_result = tmp_path / "subscriber.json"
    subscriber = subprocess.Popen(
        [
            str(probe),
            "subscribe",
            "--topic",
            "/sim/wheel/command",
            "--type-name",
            "slope_sim.interfaces.v2.WheelCommand",
            "--descriptor-set",
            str(descriptor),
            "--payload-out",
            str(received),
            "--result",
            str(subscriber_result),
            "--expected-peer-count",
            "1",
            "--deadline-ms",
            "3000",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    command_result = tmp_path / "command.json"
    try:
        completed = subprocess.run(
            [
                str(command),
                "--descriptor-set",
                str(descriptor),
                "--payload",
                str(payload),
                "--duration-ms",
                "150",
                "--deadline-ms",
                "3000",
                "--result",
                str(command_result),
            ],
            capture_output=True,
            text=True,
            timeout=8,
        )
        stdout, stderr = subscriber.communicate(timeout=8)
    finally:
        if subscriber.poll() is None:
            subscriber.kill()
            subscriber.communicate()

    assert completed.returncode == 0, completed.stderr
    assert subscriber.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
    assert received.read_bytes() == payload.read_bytes()
    assert json.loads(command_result.read_text(encoding="utf-8")) == {
        "active_published_count": 10,
        "clean_shutdown": True,
        "published_count": 15,
        "safe_stop_published_count": 5,
        "transport": "ecal",
    }


@pytest.mark.stage4_artifact
def test_cpp_subscriber_reads_exact_command_window_without_publishing(tmp_path: Path) -> None:
    """只读 Subscriber 必须验证并收齐 C++ Command 的完整 15 帧窗口。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    command = _client_tool("slope_sim_stage4_command")
    subscriber = _client_tool("slope_sim_stage4_subscriber")
    subscriber_result = tmp_path / "subscriber.json"
    subscriber_process = subprocess.Popen(
        [
            str(subscriber),
            "--topic",
            "/sim/wheel/command",
            "--descriptor-set",
            str(descriptor),
            "--expected-count",
            "15",
            "--deadline-ms",
            "3000",
            "--result",
            str(subscriber_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    command_result = tmp_path / "command.json"
    try:
        completed = subprocess.run(
            [
                str(command),
                "--descriptor-set",
                str(descriptor),
                "--payload",
                str(payload),
                "--duration-ms",
                "150",
                "--deadline-ms",
                "3000",
                "--result",
                str(command_result),
            ],
            capture_output=True,
            text=True,
            timeout=8,
        )
        stdout, stderr = subscriber_process.communicate(timeout=8)
    finally:
        if subscriber_process.poll() is None:
            subscriber_process.kill()
            subscriber_process.communicate()

    assert completed.returncode == 0, completed.stderr
    assert subscriber_process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
    assert json.loads(subscriber_result.read_text(encoding="utf-8")) == {
        "clean_shutdown": True,
        "received_count": 15,
        "role": "subscriber",
        "topic": "/sim/wheel/command",
    }


@pytest.mark.stage4_artifact
def test_cpp_subscriber_rejects_a_window_that_observed_multiple_publishers(
    tmp_path: Path,
) -> None:
    """只读 Subscriber 曾观察到第二个同型 publisher 时必须 fail-closed。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    command = _client_tool("slope_sim_stage4_command")
    subscriber = _client_tool("slope_sim_stage4_subscriber")
    subscriber_result = tmp_path / "subscriber.json"
    command_result = tmp_path / "command.json"
    announcer_result = tmp_path / "announcer.json"
    subscriber_process = subprocess.Popen(
        [
            str(subscriber),
            "--topic", "/sim/wheel/command",
            "--descriptor-set", str(descriptor),
            "--expected-count", "500",
            "--deadline-ms", "9000",
            "--result", str(subscriber_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    command_process = subprocess.Popen(
        [
            str(command),
            "--descriptor-set", str(descriptor),
            "--payload", str(payload),
            "--duration-ms", "5000",
            "--deadline-ms", "9000",
            "--result", str(command_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    announcer_process: subprocess.Popen[str] | None = None
    try:
        time.sleep(1.0)
        announcer_process = subprocess.Popen(
            [
                sys.executable,
                "-m", "scripts.verify_stage4_v2_phase0",
                "--participant", "announce",
                "--topic", "/sim/wheel/command",
                "--type-name", "slope_sim.interfaces.v2.WheelCommand",
                "--descriptor-set", str(descriptor),
                "--payload", str(payload),
                "--result", str(announcer_result),
                "--deadline-ms", "3000",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        subscriber_stdout, subscriber_stderr = subscriber_process.communicate(timeout=9)
        command_stdout, command_stderr = command_process.communicate(timeout=9)
        announcer_stdout, announcer_stderr = announcer_process.communicate(timeout=9)
    finally:
        for process in (subscriber_process, command_process, announcer_process):
            if process is not None and process.poll() is None:
                process.kill()
                process.communicate()

    assert command_process.returncode == 0, f"stdout={command_stdout}\nstderr={command_stderr}"
    assert announcer_process.returncode == 0, (
        f"stdout={announcer_stdout}\nstderr={announcer_stderr}"
    )
    assert subscriber_process.returncode != 0, (
        f"stdout={subscriber_stdout}\nstderr={subscriber_stderr}"
    )
    assert not subscriber_result.exists()


@pytest.mark.parametrize(
    ("topic", "type_name", "fixture"),
    (
        ("/sim/wheel/state", "slope_sim.interfaces.v2.WheelState", "WheelState.bin"),
        ("/sim/lidar/points", "slope_sim.interfaces.v2.LidarPointCloud", "LidarPointCloud.bin"),
        ("/sim/rtk/state", "slope_sim.interfaces.v2.RtkState", "RtkState.bin"),
        ("/sim/imu/attitude", "slope_sim.interfaces.v2.ImuAttitude", "ImuAttitude.bin"),
    ),
)
@pytest.mark.stage4_artifact
def test_cpp_subscriber_reads_each_v2_output_topic_without_publishing(
    tmp_path: Path,
    topic: str,
    type_name: str,
    fixture: str,
) -> None:
    """只读 Subscriber 必须收齐每条输出 topic 的一帧冻结 raw bytes。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2" / fixture
    subscriber = _client_tool("slope_sim_stage4_subscriber")
    probe = _phase0_tool("ecal_v2_raw_probe")
    subscriber_result = tmp_path / "subscriber.json"
    process = subprocess.Popen(
        [
            str(subscriber),
            "--topic",
            topic,
            "--descriptor-set",
            str(descriptor),
            "--expected-count",
            "1",
            "--deadline-ms",
            "3000",
            "--result",
            str(subscriber_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    publisher_result = tmp_path / "publisher.json"
    try:
        published = subprocess.run(
            [
                str(probe),
                "publish",
                "--topic",
                topic,
                "--type-name",
                type_name,
                "--descriptor-set",
                str(descriptor),
                "--payload",
                str(payload),
                "--result",
                str(publisher_result),
                "--deadline-ms",
                "3000",
            ],
            capture_output=True,
            text=True,
            timeout=8,
        )
        stdout, stderr = process.communicate(timeout=8)
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()

    assert published.returncode == 0, published.stderr
    assert process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
    assert json.loads(subscriber_result.read_text(encoding="utf-8")) == {
        "clean_shutdown": True,
        "received_count": 1,
        "role": "subscriber",
        "topic": topic,
    }


@pytest.mark.stage4_artifact
def test_cpp_subscriber_and_command_complete_verified_python_runtime_window(tmp_path: Path) -> None:
    """一个 C++ 输出 Subscriber 与唯一 C++ Command 必须完成正式五话题 runtime 窗口。"""
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    command_payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    subscriber = _client_tool("slope_sim_stage4_subscriber")
    command = _client_tool("slope_sim_stage4_command")
    subscriber_result = tmp_path / "subscriber.json"
    subscriber_process = subprocess.Popen(
        [
            str(subscriber),
            "--all-outputs",
            "true",
            "--descriptor-set",
            str(descriptor),
            "--duration-ms",
            "100",
            "--deadline-ms",
            "5000",
            "--result",
            str(subscriber_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    command_result = tmp_path / "command.json"
    command_process = subprocess.Popen(
        [
            str(command),
            "--descriptor-set",
            str(descriptor),
            "--payload",
            str(command_payload),
            "--duration-ms",
            "150",
            "--deadline-ms",
            "5000",
            "--result",
            str(command_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        time.sleep(0.1)
        assert subscriber_process.poll() is None, subscriber_process.stderr.read()
        from scripts.stage4_v2_simulation_runtime import run_v2_simulation_runtime

        runtime = run_v2_simulation_runtime(
            result_json=tmp_path / "runtime.json",
            duration_sec=0.1,
            require_verified_peers=True,
            peer_timeout_sec=5.0,
        )
        subscriber_stdout, subscriber_stderr = subscriber_process.communicate(timeout=8)
        command_stdout, command_stderr = command_process.communicate(timeout=8)
    finally:
        for process in (subscriber_process, command_process):
            if process.poll() is None:
                process.kill()
                process.communicate()

    assert subscriber_process.returncode == 0, (
        f"stdout={subscriber_stdout}\nstderr={subscriber_stderr}"
    )
    assert command_process.returncode == 0, f"stdout={command_stdout}\nstderr={command_stderr}"
    assert runtime["published_frames"] == {
        "/sim/wheel/state": 10,
        "/sim/lidar/points": 1,
        "/sim/rtk/state": 1,
        "/sim/imu/attitude": 1,
    }
    assert json.loads(subscriber_result.read_text(encoding="utf-8")) == {
        "clean_shutdown": True,
        "role": "subscriber",
        "topics": {
            "/sim/wheel/state": 10,
            "/sim/lidar/points": 1,
            "/sim/rtk/state": 1,
            "/sim/imu/attitude": 1,
        },
    }


@pytest.mark.stage4_artifact
def test_cpp_all_output_subscriber_rejects_competing_runtime_publisher_after_verified_peer(
    tmp_path: Path,
) -> None:
    """AllOutputs 在已确认唯一 runtime publisher 后遇到竞争者必须失败。"""
    from scripts.verify_stage4_v2_phase0 import initialize_phase0_core
    from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
    from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings

    root = Path(__file__).resolve().parents[2]
    descriptor_path = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelState.bin"
    command_payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    subscriber = _client_tool("slope_sim_stage4_subscriber")
    command = _client_tool("slope_sim_stage4_command")
    subscriber_result = tmp_path / "subscriber.json"
    runtime_result = tmp_path / "runtime.json"
    command_result = tmp_path / "command.json"
    announcer_result = tmp_path / "announcer.json"
    subscriber_process = subprocess.Popen(
        [
            str(subscriber),
            "--all-outputs", "true",
            "--descriptor-set", str(descriptor_path),
            "--duration-ms", "5000",
            "--deadline-ms", "9000",
            "--result", str(subscriber_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    command_process = subprocess.Popen(
        [
            str(command),
            "--descriptor-set", str(descriptor_path),
            "--payload", str(command_payload),
            "--duration-ms", "5000",
            "--deadline-ms", "9000",
            "--result", str(command_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    runtime_process = subprocess.Popen(
        [
            sys.executable,
            "-c",
            "from pathlib import Path\n"
            "from scripts.stage4_v2_simulation_runtime import run_v2_simulation_runtime\n"
            f"run_v2_simulation_runtime(result_json=Path({str(runtime_result)!r}), duration_sec=5.0, "
            "require_verified_peers=True, peer_timeout_sec=10.0)",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    bindings = EcalRawBindings()
    core = bindings._core
    announcer_process: subprocess.Popen[str] | None = None
    try:
        initialize_phase0_core(core, f"stage4-all-output-watcher-{os.getpid()}")
        watcher = bindings.create_subscriber(
            "/sim/wheel/state",
            "slope_sim.interfaces.v2.WheelState",
            load_v2_descriptor(),
            lambda _frame: None,
        )
        deadline = time.monotonic() + 12.0
        while watcher.get_publisher_count() != 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert watcher.get_publisher_count() == 1, "runtime publisher did not become uniquely visible"
        assert runtime_process.poll() is None, runtime_process.stderr.read()
        announcer_process = subprocess.Popen(
            [
                sys.executable,
                "-m", "scripts.verify_stage4_v2_phase0",
                "--participant", "announce",
                "--topic", "/sim/wheel/state",
                "--type-name", "slope_sim.interfaces.v2.WheelState",
                "--descriptor-set", str(descriptor_path),
                "--payload", str(payload),
                "--result", str(announcer_result),
                "--deadline-ms", "3000",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        subscriber_stdout, subscriber_stderr = subscriber_process.communicate(timeout=15)
        runtime_stdout, runtime_stderr = runtime_process.communicate(timeout=15)
        announcer_stdout, announcer_stderr = announcer_process.communicate(timeout=15)
    finally:
        finalize = getattr(core, "finalize", None)
        if callable(finalize):
            finalize()
        for process in (subscriber_process, command_process, runtime_process, announcer_process):
            if process is not None and process.poll() is None:
                process.kill()
                process.communicate()

    command_stdout, command_stderr = command_process.communicate(timeout=1)
    assert command_process.returncode == 0, f"stdout={command_stdout}\nstderr={command_stderr}"
    assert runtime_process.returncode == 0, f"stdout={runtime_stdout}\nstderr={runtime_stderr}"
    assert announcer_process.returncode == 0, (
        f"stdout={announcer_stdout}\nstderr={announcer_stderr}"
    )
    assert subscriber_process.returncode != 0, (
        f"stdout={subscriber_stdout}\nstderr={subscriber_stderr}"
    )
    assert not subscriber_result.exists()
