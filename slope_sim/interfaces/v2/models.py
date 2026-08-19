"""阶段四 wheel 协议模型：冻结会话、代际、命令来源和命令权回显。"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
from numbers import Real
import re
from struct import pack, unpack

from slope_sim.interfaces.models import WheelCommand


_UINT32_MAX = (1 << 32) - 1
_UINT64_MAX = (1 << 64) - 1
_SOURCE_ID = re.compile(r"[A-Za-z0-9._-]{1,64}\Z")


class CommandAuthorityState(IntEnum):
    """v2 wheel state 的精确命令权状态。"""

    WAITING = 1
    CLAIMABLE = 2
    ACTIVE = 3
    CONFLICT = 4


def require_fixed_bytes(name: str, value: object, length: int) -> bytes:
    """复制并验证固定长度的二进制身份字段。"""
    if not isinstance(value, (bytes, bytearray, memoryview)):
        raise ValueError(f"{name} must be bytes-like")
    copied = bytes(value)
    if len(copied) != length:
        raise ValueError(f"{name} must be exactly {length} bytes")
    return copied


def require_uint(name: str, value: object, maximum: int = _UINT64_MAX) -> int:
    """拒绝 bool 和越界值，保持 Protobuf uint 语义精确。"""
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ValueError(f"{name} is outside its unsigned integer range")
    return value


def require_float_tuple(name: str, value: object) -> tuple[float, ...]:
    """将有限实数序列复制成不可变 float tuple。"""
    if not isinstance(value, (tuple, list)):
        raise ValueError(f"{name} must be an ordered sequence")
    if any(isinstance(item, bool) or not isinstance(item, Real) or not math.isfinite(float(item)) for item in value):
        raise ValueError(f"{name} must contain finite numbers")
    return tuple(float(item) for item in value)


def require_finite_float(name: str, value: object) -> float:
    """规范有限实数，阻止 bool、NaN 和无穷值进入 v2 数据面。"""
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
    return float(value)


def require_float32(name: str, value: object) -> float:
    """规范 Protobuf float 字段，确保 encode/decode 的领域值逐值稳定。"""
    normalized = require_finite_float(name, value)
    try:
        return unpack("<f", pack("<f", normalized))[0]
    except OverflowError as error:
        raise ValueError(f"{name} is outside float32 range") from error


@dataclass(frozen=True)
class WheelCommandV2:
    timestamp_ns: int
    drive_wheel_speed_rad_s: tuple[float, ...]
    steering_wheel_speed_rad_s: tuple[float, ...]
    sequence: int
    world_generation: int
    command_generation: int
    source_id: str
    source_session_id: bytes
    robot_model: str
    simulation_session_id: bytes
    descriptor_sha256: bytes

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", require_uint("timestamp_ns", self.timestamp_ns))
        object.__setattr__(self, "sequence", require_uint("sequence", self.sequence))
        for name in ("world_generation", "command_generation"):
            value = require_uint(name, getattr(self, name))
            if value == 0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)
        if not isinstance(self.source_id, str) or _SOURCE_ID.fullmatch(self.source_id) is None:
            raise ValueError("source_id must match [A-Za-z0-9._-]{1,64}")
        if not isinstance(self.robot_model, str) or not self.robot_model:
            raise ValueError("robot_model must be nonempty")
        object.__setattr__(self, "source_session_id", require_fixed_bytes("source_session_id", self.source_session_id, 16))
        object.__setattr__(self, "simulation_session_id", require_fixed_bytes("simulation_session_id", self.simulation_session_id, 16))
        object.__setattr__(self, "descriptor_sha256", require_fixed_bytes("descriptor_sha256", self.descriptor_sha256, 32))
        object.__setattr__(self, "drive_wheel_speed_rad_s", require_float_tuple("drive_wheel_speed_rad_s", self.drive_wheel_speed_rad_s))
        object.__setattr__(self, "steering_wheel_speed_rad_s", require_float_tuple("steering_wheel_speed_rad_s", self.steering_wheel_speed_rad_s))

    def to_v1_motion(self) -> WheelCommand:
        """仅转换运动值供既有 mailbox 复用，保留已完成的 v2 校验。"""
        return WheelCommand(self.timestamp_ns, self.drive_wheel_speed_rad_s, self.steering_wheel_speed_rad_s)


@dataclass(frozen=True)
class WheelStateV2:
    timestamp_ns: int
    drive_wheel_speed_rad_s: tuple[float, ...]
    steering_wheel_angle_rad: tuple[float, ...]
    sequence: int
    world_generation: int
    command_generation: int
    robot_model: str
    simulation_session_id: bytes
    descriptor_sha256: bytes
    command_authority_state: CommandAuthorityState
    command_owner_source_id: str
    command_owner_source_session_id: bytes
    command_peer_count: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", require_uint("timestamp_ns", self.timestamp_ns))
        object.__setattr__(self, "sequence", require_uint("sequence", self.sequence))
        for name in ("world_generation", "command_generation"):
            value = require_uint(name, getattr(self, name))
            if value == 0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)
        object.__setattr__(self, "command_peer_count", require_uint("command_peer_count", self.command_peer_count, _UINT32_MAX))
        object.__setattr__(self, "simulation_session_id", require_fixed_bytes("simulation_session_id", self.simulation_session_id, 16))
        object.__setattr__(self, "descriptor_sha256", require_fixed_bytes("descriptor_sha256", self.descriptor_sha256, 32))
        object.__setattr__(self, "drive_wheel_speed_rad_s", require_float_tuple("drive_wheel_speed_rad_s", self.drive_wheel_speed_rad_s))
        object.__setattr__(self, "steering_wheel_angle_rad", require_float_tuple("steering_wheel_angle_rad", self.steering_wheel_angle_rad))
        if not isinstance(self.robot_model, str) or not self.robot_model:
            raise ValueError("robot_model must be nonempty")
        if type(self.command_authority_state) is not CommandAuthorityState:
            raise ValueError("command_authority_state must be a CommandAuthorityState")
        expected = {CommandAuthorityState.WAITING: 0, CommandAuthorityState.CLAIMABLE: 1, CommandAuthorityState.ACTIVE: 1}
        if (self.command_authority_state in expected and self.command_peer_count != expected[self.command_authority_state]) or (self.command_authority_state is CommandAuthorityState.CONFLICT and self.command_peer_count <= 1):
            raise ValueError("command authority state does not match exact peer count")
        owner_session = bytes(self.command_owner_source_session_id)
        if self.command_authority_state is CommandAuthorityState.ACTIVE:
            if not isinstance(self.command_owner_source_id, str) or _SOURCE_ID.fullmatch(self.command_owner_source_id) is None:
                raise ValueError("ACTIVE requires a valid owner source_id")
            owner_session = require_fixed_bytes("command_owner_source_session_id", owner_session, 16)
        elif self.command_owner_source_id or owner_session:
            raise ValueError("non-ACTIVE state must not expose an owner")
        object.__setattr__(self, "command_owner_source_session_id", owner_session)


@dataclass(frozen=True)
class ImuAttitudeV2:
    """v2 IMU 的同帧姿态、身份和代际字段。"""

    timestamp_ns: int
    roll_rad: float
    pitch_rad: float
    sequence: int
    world_generation: int
    frame_id: str
    simulation_session_id: bytes
    descriptor_sha256: bytes

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", require_uint("timestamp_ns", self.timestamp_ns))
        object.__setattr__(self, "sequence", require_uint("sequence", self.sequence))
        generation = require_uint("world_generation", self.world_generation)
        if generation == 0:
            raise ValueError("world_generation must be positive")
        object.__setattr__(self, "world_generation", generation)
        for name in ("roll_rad", "pitch_rad"):
            object.__setattr__(self, name, require_finite_float(name, getattr(self, name)))
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be nonempty")
        object.__setattr__(self, "simulation_session_id", require_fixed_bytes("simulation_session_id", self.simulation_session_id, 16))
        object.__setattr__(self, "descriptor_sha256", require_fixed_bytes("descriptor_sha256", self.descriptor_sha256, 32))


@dataclass(frozen=True)
class Point3dV2:
    """RTK 天线在冻结坐标系中的三维位置。"""

    x_m: float
    y_m: float
    z_m: float

    def __post_init__(self) -> None:
        for name in ("x_m", "y_m", "z_m"):
            object.__setattr__(self, name, require_finite_float(name, getattr(self, name)))


@dataclass(frozen=True)
class RtkStateV2:
    """v2 三点 RTK、基线航向与会话身份字段。"""

    timestamp_ns: int
    sequence: int
    world_generation: int
    frame_id: str
    left: Point3dV2
    center: Point3dV2
    right: Point3dV2
    heading_rad: float
    simulation_session_id: bytes
    descriptor_sha256: bytes

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", require_uint("timestamp_ns", self.timestamp_ns))
        object.__setattr__(self, "sequence", require_uint("sequence", self.sequence))
        generation = require_uint("world_generation", self.world_generation)
        if generation == 0:
            raise ValueError("world_generation must be positive")
        object.__setattr__(self, "world_generation", generation)
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be nonempty")
        for name in ("left", "center", "right"):
            if type(getattr(self, name)) is not Point3dV2:
                raise ValueError(f"{name} must be a Point3dV2")
        object.__setattr__(self, "heading_rad", require_finite_float("heading_rad", self.heading_rad))
        object.__setattr__(self, "simulation_session_id", require_fixed_bytes("simulation_session_id", self.simulation_session_id, 16))
        object.__setattr__(self, "descriptor_sha256", require_fixed_bytes("descriptor_sha256", self.descriptor_sha256, 32))


@dataclass(frozen=True)
class LidarPointV2:
    """MID-360 风格单点，沿用冻结的 v2 点字段范围。"""

    offset_time_ns: int
    x: float
    y: float
    z: float
    reflectivity: int
    tag: int
    line: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "offset_time_ns", require_uint("offset_time_ns", self.offset_time_ns, _UINT32_MAX))
        for name in ("x", "y", "z"):
            object.__setattr__(self, name, require_float32(name, getattr(self, name)))
        object.__setattr__(self, "reflectivity", require_uint("reflectivity", self.reflectivity, _UINT32_MAX))
        tag = require_uint("tag", self.tag, _UINT32_MAX)
        line = require_uint("line", self.line, _UINT32_MAX)
        if tag > 3:
            raise ValueError("tag must be in range 0..3")
        if line > 15:
            raise ValueError("line must be in range 0..15")
        object.__setattr__(self, "tag", tag)
        object.__setattr__(self, "line", line)


@dataclass(frozen=True)
class LidarPointCloudV2:
    """v2 中心 LiDAR 的一帧点云、代际和会话身份。"""

    timebase_ns: int
    frame_id: str
    point_num: int
    lidar_id: int
    points: tuple[LidarPointV2, ...]
    sequence: int
    world_generation: int
    simulation_session_id: bytes
    descriptor_sha256: bytes

    def __post_init__(self) -> None:
        object.__setattr__(self, "timebase_ns", require_uint("timebase_ns", self.timebase_ns))
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be nonempty")
        object.__setattr__(self, "point_num", require_uint("point_num", self.point_num, _UINT32_MAX))
        object.__setattr__(self, "lidar_id", require_uint("lidar_id", self.lidar_id, _UINT32_MAX))
        if not isinstance(self.points, (tuple, list)):
            raise ValueError("points must be an ordered sequence of LidarPointV2")
        points = tuple(self.points)
        if any(type(point) is not LidarPointV2 for point in points):
            raise ValueError("points must contain only LidarPointV2 values")
        if self.point_num != len(points):
            raise ValueError("point_num must equal len(points)")
        object.__setattr__(self, "points", points)
        object.__setattr__(self, "sequence", require_uint("sequence", self.sequence))
        generation = require_uint("world_generation", self.world_generation)
        if generation == 0:
            raise ValueError("world_generation must be positive")
        object.__setattr__(self, "world_generation", generation)
        object.__setattr__(self, "simulation_session_id", require_fixed_bytes("simulation_session_id", self.simulation_session_id, 16))
        object.__setattr__(self, "descriptor_sha256", require_fixed_bytes("descriptor_sha256", self.descriptor_sha256, 32))
