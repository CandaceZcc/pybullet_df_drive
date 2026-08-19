"""阶段四 v2 Python/C++ 冻结 Protobuf 原始字节互操作测试。"""
from __future__ import annotations

from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess

import pytest

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb


_MESSAGE_NAMES = (
    "WheelCommand",
    "WheelState",
    "LidarPointCloud",
    "RtkState",
    "ImuAttitude",
)
_REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
_GOLDEN_DIRECTORY = _REPOSITORY_ROOT / "tests" / "fixtures" / "stage4" / "v2"
_DESCRIPTOR_PATH = (
    _REPOSITORY_ROOT
    / "slope_sim"
    / "interfaces"
    / "generated"
    / "slope_sim_interfaces_v2.desc"
)


def _phase0_golden_tool() -> Path:
    """从显式 Task 9 GREEN build 取得冻结的 C++ golden 工具。"""
    raw_build = os.environ.get("STAGE4_PHASE0_BUILD_DIR")
    assert raw_build, "STAGE4_PHASE0_BUILD_DIR must name the Task 9 GREEN build"
    build_directory = Path(raw_build)
    assert build_directory.is_absolute(), "Task 9 build directory must be absolute"
    executable = build_directory / "v2_golden"
    assert executable.is_file(), "Task 9 v2_golden build is missing"
    assert os.access(executable, os.X_OK), "Task 9 v2_golden is not executable"
    return executable


def _load_manifest() -> dict[str, object]:
    """读取与 fixture 一起冻结的摘要清单，避免测试自证。"""
    manifest_path = _GOLDEN_DIRECTORY / "manifest.json"
    assert manifest_path.is_file(), f"golden fixture is not implemented: {manifest_path}"
    return json.loads(manifest_path.read_text(encoding="utf-8"))


def _run_v2_golden(*arguments: str) -> subprocess.CompletedProcess[str]:
    """调用 C++ golden 工具，保留不同子命令各自的输出合同。"""
    return subprocess.run(
        [str(_phase0_golden_tool()), *arguments],
        check=True,
        capture_output=True,
        text=True,
    )


def _decode_v2_golden(*arguments: str) -> dict[str, object]:
    """只解析 decode 子命令保证输出的稳定 JSON 文档。"""
    return json.loads(_run_v2_golden(*arguments).stdout)


@pytest.fixture
def golden_directory() -> Path:
    """提供仓库内只读的 v2 fixture 目录。"""
    return _GOLDEN_DIRECTORY


@pytest.fixture
def v2_descriptor_path() -> Path:
    """提供阶段四冻结的 FileDescriptorSet 原始文件。"""
    assert _DESCRIPTOR_PATH.is_file(), "frozen v2 descriptor is missing"
    return _DESCRIPTOR_PATH


@pytest.mark.parametrize("message_name", _MESSAGE_NAMES)
@pytest.mark.stage4_artifact
def test_python_golden_decodes_in_cpp_without_byte_change(
    message_name: str, golden_directory: Path, v2_descriptor_path: Path
) -> None:
    """Python fixture 必须被 C++ 解码且保持相同 payload 摘要。"""
    golden_file = golden_directory / f"{message_name}.bin"
    assert golden_file.is_file(), f"golden fixture is not implemented: {golden_file}"
    manifest = _load_manifest()
    document = _decode_v2_golden(
        "decode",
        "--descriptor-set",
        str(v2_descriptor_path),
        message_name,
        str(golden_file),
    )

    expected = manifest["messages"][message_name]
    assert document["message_name"] == message_name
    assert document["payload_sha256"] == expected["payload_sha256"]
    assert document["payload_sha256"] == sha256(golden_file.read_bytes()).hexdigest()
    assert document["descriptor_sha256"] == manifest["descriptor_sha256"]


@pytest.mark.parametrize("message_name", _MESSAGE_NAMES)
@pytest.mark.stage4_artifact
def test_cpp_encodes_exact_python_golden_bytes(
    message_name: str,
    golden_directory: Path,
    v2_descriptor_path: Path,
    tmp_path: Path,
) -> None:
    """冻结 C++ encoder 重新产生的五种 payload 必须逐 byte 等于 Python fixture。"""
    golden_file = golden_directory / f"{message_name}.bin"
    assert golden_file.is_file(), f"golden fixture is not implemented: {golden_file}"
    output_directory = tmp_path / "cpp-goldens"
    output_directory.mkdir()
    _run_v2_golden(
        "encode-fixtures",
        "--descriptor-set",
        str(v2_descriptor_path),
        "--output-dir",
        str(output_directory),
    )

    cpp_payload = (output_directory / f"{message_name}.bin").read_bytes()
    python_payload = golden_file.read_bytes()
    assert cpp_payload == python_payload
    message = getattr(pb, message_name)()
    message.ParseFromString(cpp_payload)
    assert message.SerializeToString(deterministic=True) == python_payload


def test_task13_goldens_cover_full_robot_and_sensor_values(golden_directory: Path) -> None:
    """golden 必须覆盖计划指定的车型、轮组、三维点与 RTK 三点字段。"""
    command = pb.WheelCommand()
    command.ParseFromString((golden_directory / "WheelCommand.bin").read_bytes())
    assert command.timestamp_ns == 1_000_000_000
    assert tuple(command.drive_wheel_speed_rad_s) == (1.5, -2.25)
    assert tuple(command.steering_wheel_speed_rad_s) == ()
    assert command.sequence == 3
    assert command.command_generation == 11
    assert command.source_id == "golden.command"
    assert command.robot_model == "df_mid"

    state = pb.WheelState()
    state.ParseFromString((golden_directory / "WheelState.bin").read_bytes())
    assert tuple(state.drive_wheel_speed_rad_s) == (1.5, -2.25, 3.75, -4.5)
    assert tuple(state.steering_wheel_angle_rad) == (0.25, -0.5)
    assert state.command_authority_state == pb.ACTIVE
    assert state.command_owner_source_id == "golden.command"
    assert state.command_peer_count == 1

    lidar = pb.LidarPointCloud()
    lidar.ParseFromString((golden_directory / "LidarPointCloud.bin").read_bytes())
    assert lidar.point_num == 2
    assert [(point.offset_time_ns, point.x, point.y, point.z, point.reflectivity, point.tag, point.line) for point in lidar.points] == [
        (0, 1.0, 2.0, 3.0, 4, 5, 6),
        (100, -1.0, -2.0, -3.0, 7, 8, 9),
    ]

    rtk = pb.RtkState()
    rtk.ParseFromString((golden_directory / "RtkState.bin").read_bytes())
    assert (rtk.left.x_m, rtk.left.y_m, rtk.left.z_m) == (1.0, 0.5, 0.2)
    assert (rtk.center.x_m, rtk.center.y_m, rtk.center.z_m) == (1.0, 0.0, 0.2)
    assert (rtk.right.x_m, rtk.right.y_m, rtk.right.z_m) == (1.0, -0.5, 0.2)
    assert rtk.heading_rad == 0.25
