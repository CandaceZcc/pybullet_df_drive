# 企业接口模型：定义与传输框架解耦的不可变消息及轮子命令校验。
from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
import math
from numbers import Real

from slope_sim.model_registry import RobotModelSpec


_UINT32_MAX = (1 << 32) - 1
_UINT64_MAX = (1 << 64) - 1


class WheelCommandValidationError(ValueError):
    """轮子命令可归因的领域校验错误基类。"""


class WheelCommandModelMismatchError(WheelCommandValidationError):
    """轮子数组长度与当前车型不匹配。"""


class WheelCommandMechanicalLimitError(WheelCommandValidationError):
    """轮速超过当前车型机械限位。"""


def _require_uint(name: str, value: object, maximum: int, type_name: str) -> int:
    """校验 Protobuf 无符号整数字段，显式排除 Python 的 bool 子类型。"""
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ValueError(f"{name} must be a {type_name}")
    return value


def _require_finite_float(name: str, value: object) -> float:
    """把实数规范为 float，并阻止 bool、NaN 和无穷值进入接口层。"""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be a finite number")
    return normalized


def _normalize_float_sequence(name: str, values: object) -> tuple[float, ...]:
    """复制数值序列，确保冻结消息不再引用调用方的可变列表。"""
    invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
    if isinstance(values, invalid_types) or not isinstance(values, Sequence):
        raise ValueError(f"{name} must be an ordered sequence of finite numbers")
    items = tuple(values)
    return tuple(_require_finite_float(f"{name}[{index}]", value) for index, value in enumerate(items))


@dataclass(frozen=True)
class WheelCommand:
    """按轮子语义排列的速度命令；车型长度由独立校验器检查。"""

    timestamp_ns: int
    drive_wheel_speed_rad_s: tuple[float, ...]
    steering_wheel_speed_rad_s: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", _require_uint("timestamp_ns", self.timestamp_ns, _UINT64_MAX, "uint64"))
        object.__setattr__(
            self,
            "drive_wheel_speed_rad_s",
            _normalize_float_sequence("drive_wheel_speed_rad_s", self.drive_wheel_speed_rad_s),
        )
        object.__setattr__(
            self,
            "steering_wheel_speed_rad_s",
            _normalize_float_sequence("steering_wheel_speed_rad_s", self.steering_wheel_speed_rad_s),
        )


@dataclass(frozen=True)
class WheelState:
    """从物理引擎读取的实际驱动轮速度和转向轮角度。"""

    timestamp_ns: int
    drive_wheel_speed_rad_s: tuple[float, ...]
    steering_wheel_angle_rad: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", _require_uint("timestamp_ns", self.timestamp_ns, _UINT64_MAX, "uint64"))
        object.__setattr__(
            self,
            "drive_wheel_speed_rad_s",
            _normalize_float_sequence("drive_wheel_speed_rad_s", self.drive_wheel_speed_rad_s),
        )
        object.__setattr__(
            self,
            "steering_wheel_angle_rad",
            _normalize_float_sequence("steering_wheel_angle_rad", self.steering_wheel_angle_rad),
        )


@dataclass(frozen=True)
class LidarPoint:
    """单个多线雷达点及其扫描顺序和命中语义。"""

    offset_time_ns: int
    x: float
    y: float
    z: float
    reflectivity: int
    tag: int
    line: int

    def __post_init__(self) -> None:
        # LiDAR 热路径只产生精确内建类型；等价快速校验避免逐字段 ABC 分派。
        if (
            type(self.offset_time_ns) is int
            and 0 <= self.offset_time_ns <= _UINT32_MAX
            and type(self.x) is float
            and math.isfinite(self.x)
            and type(self.y) is float
            and math.isfinite(self.y)
            and type(self.z) is float
            and math.isfinite(self.z)
            and type(self.reflectivity) is int
            and 0 <= self.reflectivity <= _UINT32_MAX
            and type(self.tag) is int
            and 0 <= self.tag <= 3
            and type(self.line) is int
            and 0 <= self.line <= 15
        ):
            return
        object.__setattr__(self, "offset_time_ns", _require_uint("offset_time_ns", self.offset_time_ns, _UINT32_MAX, "uint32"))
        object.__setattr__(self, "x", _require_finite_float("x", self.x))
        object.__setattr__(self, "y", _require_finite_float("y", self.y))
        object.__setattr__(self, "z", _require_finite_float("z", self.z))
        object.__setattr__(self, "reflectivity", _require_uint("reflectivity", self.reflectivity, _UINT32_MAX, "uint32"))
        tag = _require_uint("tag", self.tag, _UINT32_MAX, "uint32")
        line = _require_uint("line", self.line, _UINT32_MAX, "uint32")
        if tag > 3:
            raise ValueError("tag must be in range 0..3")
        if line > 15:
            raise ValueError("line must be in range 0..15")
        object.__setattr__(self, "tag", tag)
        object.__setattr__(self, "line", line)


@dataclass(frozen=True)
class LidarPointCloud:
    """一台雷达的一帧不可变点云。"""

    timebase_ns: int
    frame_id: str
    point_num: int
    lidar_id: int
    points: tuple[LidarPoint, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "timebase_ns", _require_uint("timebase_ns", self.timebase_ns, _UINT64_MAX, "uint64"))
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("frame_id must be a nonempty string")
        object.__setattr__(self, "point_num", _require_uint("point_num", self.point_num, _UINT32_MAX, "uint32"))
        object.__setattr__(self, "lidar_id", _require_uint("lidar_id", self.lidar_id, _UINT32_MAX, "uint32"))
        invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
        if isinstance(self.points, invalid_types) or not isinstance(self.points, Sequence):
            raise ValueError("points must be an ordered sequence of LidarPoint")
        points = tuple(self.points)
        if any(not isinstance(point, LidarPoint) for point in points):
            raise ValueError("points must contain only LidarPoint values")
        if self.point_num != len(points):
            raise ValueError("point_num must equal len(points)")
        object.__setattr__(self, "points", points)


@dataclass(frozen=True)
class RtkState:
    """RTK 主天线位置与双天线基线偏航角。"""

    timestamp_ns: int
    main_x: float
    main_y: float
    main_z: float
    baseline_yaw_rad: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", _require_uint("timestamp_ns", self.timestamp_ns, _UINT64_MAX, "uint64"))
        object.__setattr__(self, "main_x", _require_finite_float("main_x", self.main_x))
        object.__setattr__(self, "main_y", _require_finite_float("main_y", self.main_y))
        object.__setattr__(self, "main_z", _require_finite_float("main_z", self.main_z))
        object.__setattr__(
            self,
            "baseline_yaw_rad",
            _require_finite_float("baseline_yaw_rad", self.baseline_yaw_rad),
        )


@dataclass(frozen=True)
class ImuAttitude:
    """IMU 输出的车体横滚角和俯仰角。"""

    timestamp_ns: int
    roll_rad: float
    pitch_rad: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", _require_uint("timestamp_ns", self.timestamp_ns, _UINT64_MAX, "uint64"))
        object.__setattr__(self, "roll_rad", _require_finite_float("roll_rad", self.roll_rad))
        object.__setattr__(self, "pitch_rad", _require_finite_float("pitch_rad", self.pitch_rad))


def _require_bounded(
    name: str,
    values: tuple[float, ...],
    limit_name: str,
    limit: object,
) -> None:
    """原子检查整组轮速的有限性和对称机械限位。"""
    normalized_limit = _require_finite_float(limit_name, limit)
    if normalized_limit < 0.0:
        raise ValueError(f"{limit_name} must be non-negative")
    for index, value in enumerate(values):
        normalized = _require_finite_float(f"{name}[{index}]", value)
        if abs(normalized) > normalized_limit:
            raise WheelCommandMechanicalLimitError(
                f"{name} wheel speed exceeds limit {normalized_limit} rad/s"
            )


def validate_wheel_command(command: WheelCommand, model: RobotModelSpec) -> WheelCommand:
    """按当前车型整条校验轮子数量和速度限位，成功时返回原对象。"""
    if model.controller_kind == "differential":
        expected_drive, expected_steering = 2, 0
    elif model.controller_kind == "active_steering":
        expected_drive, expected_steering = 4, 2
    else:
        raise ValueError(f"unsupported controller_kind: {model.controller_kind}")

    if len(command.drive_wheel_speed_rad_s) != expected_drive:
        raise WheelCommandModelMismatchError(
            f"{model.name} requires {expected_drive} drive wheel speeds"
        )
    if len(command.steering_wheel_speed_rad_s) != expected_steering:
        raise WheelCommandModelMismatchError(
            f"{model.name} requires {expected_steering} steering wheel speeds"
        )
    _require_bounded(
        "drive",
        command.drive_wheel_speed_rad_s,
        "max_drive_wheel_speed_rad_s",
        model.max_drive_wheel_speed_rad_s,
    )
    _require_bounded(
        "steering",
        command.steering_wheel_speed_rad_s,
        "max_steering_speed_rad_s",
        model.max_steering_speed_rad_s,
    )
    return command
