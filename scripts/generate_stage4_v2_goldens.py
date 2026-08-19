"""阶段四 v2 Python/C++ 互操作 golden payload 的创建与只读校验工具。"""
from __future__ import annotations

import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys
from typing import Mapping

from google.protobuf.message import Message

_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPOSITORY_ROOT))

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb


_DESCRIPTOR_PATH = (
    _REPOSITORY_ROOT
    / "slope_sim"
    / "interfaces"
    / "generated"
    / "slope_sim_interfaces_v2.desc"
)
_OUTPUT_DIRECTORY = _REPOSITORY_ROOT / "tests" / "fixtures" / "stage4" / "v2"
_SIMULATION_SESSION_ID = bytes.fromhex("00112233445566778899aabbccddeeff")
_SOURCE_SESSION_ID = bytes.fromhex("ffeeddccbbaa99887766554433221100")
_MESSAGE_NAMES = (
    "WheelCommand",
    "WheelState",
    "LidarPointCloud",
    "RtkState",
    "ImuAttitude",
)


def _descriptor_sha256() -> bytes:
    """读取唯一 v2 descriptor 并返回其固定 SHA-256。"""
    if not _DESCRIPTOR_PATH.is_file():
        raise RuntimeError(f"frozen v2 descriptor is missing: {_DESCRIPTOR_PATH}")
    return sha256(_DESCRIPTOR_PATH.read_bytes()).digest()


def _set_identity(message: Message, descriptor_sha256: bytes) -> None:
    """为五类顶层消息写入同一会话、descriptor 与 world generation。"""
    message.simulation_session_id = _SIMULATION_SESSION_ID
    message.descriptor_sha256 = descriptor_sha256
    message.world_generation = 7


def _build_messages() -> Mapping[str, Message]:
    """复现冻结 C++ v2_golden encoder 的五条 canonical 消息。"""
    descriptor_sha256 = _descriptor_sha256()

    command = pb.WheelCommand()
    _set_identity(command, descriptor_sha256)
    command.timestamp_ns = 1_000_000_000
    command.drive_wheel_speed_rad_s.extend((1.5, -2.25))
    command.sequence = 3
    command.command_generation = 11
    command.source_id = "golden.command"
    command.source_session_id = _SOURCE_SESSION_ID
    command.robot_model = "df_mid"

    state = pb.WheelState()
    _set_identity(state, descriptor_sha256)
    state.timestamp_ns = 1_000_000_000
    state.drive_wheel_speed_rad_s.extend((1.5, -2.25, 3.75, -4.5))
    state.steering_wheel_angle_rad.extend((0.25, -0.5))
    state.sequence = 4
    state.command_generation = 11
    state.robot_model = "df_mid"
    state.command_authority_state = pb.ACTIVE
    state.command_owner_source_id = command.source_id
    state.command_owner_source_session_id = command.source_session_id
    state.command_peer_count = 1

    lidar = pb.LidarPointCloud()
    _set_identity(lidar, descriptor_sha256)
    lidar.timebase_ns = 1_000_000_000
    lidar.frame_id = "lidar_front"
    lidar.lidar_id = 1
    lidar.sequence = 6
    first = lidar.points.add()
    first.x, first.y, first.z = 1.0, 2.0, 3.0
    first.reflectivity, first.tag, first.line = 4, 5, 6
    second = lidar.points.add()
    second.offset_time_ns = 100
    second.x, second.y, second.z = -1.0, -2.0, -3.0
    second.reflectivity, second.tag, second.line = 7, 8, 9
    lidar.point_num = len(lidar.points)

    rtk = pb.RtkState()
    _set_identity(rtk, descriptor_sha256)
    rtk.timestamp_ns = 1_000_000_000
    rtk.sequence = 5
    rtk.frame_id = "world"
    rtk.left.x_m, rtk.left.y_m, rtk.left.z_m = 1.0, 0.5, 0.2
    rtk.center.x_m, rtk.center.y_m, rtk.center.z_m = 1.0, 0.0, 0.2
    rtk.right.x_m, rtk.right.y_m, rtk.right.z_m = 1.0, -0.5, 0.2
    rtk.heading_rad = 0.25

    imu = pb.ImuAttitude()
    _set_identity(imu, descriptor_sha256)
    imu.timestamp_ns = 1_000_000_000
    imu.roll_rad = 0.1
    imu.pitch_rad = 0.2
    imu.sequence = 7
    imu.frame_id = "base_link"

    return {
        "WheelCommand": command,
        "WheelState": state,
        "LidarPointCloud": lidar,
        "RtkState": rtk,
        "ImuAttitude": imu,
    }


def _serialized_messages() -> Mapping[str, bytes]:
    """一次确定性序列化所有 canonical 消息，供创建和验证共用。"""
    return {
        name: message.SerializeToString(deterministic=True)
        for name, message in _build_messages().items()
    }


def _manifest(payloads: Mapping[str, bytes]) -> dict[str, object]:
    """构造含身份和摘要的稳定 manifest，供变更审查与离线验收。"""
    descriptor_hex = _descriptor_sha256().hex()
    return {
        "descriptor_sha256": descriptor_hex,
        "messages": {
            name: {
                "descriptor_sha256": descriptor_hex,
                "payload_sha256": sha256(payloads[name]).hexdigest(),
                "simulation_session_id": _SIMULATION_SESSION_ID.hex(),
                "type_name": f"slope_sim.interfaces.v2.{name}",
                "world_generation": 7,
            }
            for name in _MESSAGE_NAMES
        },
    }


def _manifest_bytes(payloads: Mapping[str, bytes]) -> bytes:
    """以稳定排序序列化清单，避免格式噪声掩盖 payload 变更。"""
    return (json.dumps(_manifest(payloads), indent=2, sort_keys=True) + "\n").encode("ascii")


def _write_new_file(path: Path, payload: bytes) -> None:
    """用 exclusive create 写入冻结文件，拒绝任何覆盖。"""
    with path.open("xb") as stream:
        stream.write(payload)


def create_goldens() -> None:
    """只在全新目标目录创建完整 fixture 集，防止静默更新 golden。"""
    if _OUTPUT_DIRECTORY.exists():
        raise RuntimeError(f"golden output already exists: {_OUTPUT_DIRECTORY}")
    _OUTPUT_DIRECTORY.mkdir(parents=True)
    payloads = _serialized_messages()
    for name in _MESSAGE_NAMES:
        _write_new_file(_OUTPUT_DIRECTORY / f"{name}.bin", payloads[name])
    _write_new_file(_OUTPUT_DIRECTORY / "manifest.json", _manifest_bytes(payloads))


def verify_goldens() -> None:
    """逐 byte 验证 fixture、manifest、schema 与 Python 确定性编码均未漂移。"""
    payloads = _serialized_messages()
    for name in _MESSAGE_NAMES:
        path = _OUTPUT_DIRECTORY / f"{name}.bin"
        if not path.is_file():
            raise RuntimeError(f"golden payload is missing: {path}")
        if path.read_bytes() != payloads[name]:
            raise RuntimeError(f"golden payload drifted: {path}")
    manifest_path = _OUTPUT_DIRECTORY / "manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError(f"golden manifest is missing: {manifest_path}")
    if manifest_path.read_bytes() != _manifest_bytes(payloads):
        raise RuntimeError(f"golden manifest drifted: {manifest_path}")


def main(argv: list[str] | None = None) -> int:
    """提供一次性创建和默认只读验证两种明确模式。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--create",
        action="store_true",
        help="only create a new golden directory; never overwrite an existing one",
    )
    arguments = parser.parse_args(argv)
    try:
        if arguments.create:
            create_goldens()
        else:
            verify_goldens()
    except RuntimeError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    if arguments.create:
        print("5 golden payloads created")
    else:
        print("5 golden payloads verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
