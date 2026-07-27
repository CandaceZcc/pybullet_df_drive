# Dashboard 快照模型：定义 GUI 唯一可读取的不可变接口状态与 LiDAR 俯视点。
from __future__ import annotations

from collections.abc import Iterator, Mapping, Sequence
from dataclasses import dataclass
import math
from numbers import Real

from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.interfaces.status import InterfaceStatusSnapshot
from slope_sim.model_registry import robot_model_names


_UINT64_MAX = (1 << 64) - 1
_ROBOT_MODELS = frozenset(robot_model_names())


def _require_finite_number(name: str, value: object) -> float:
    """规范俯视坐标，并显式拒绝 bool、NaN 和无穷值。"""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be a finite number")
    return normalized


def _require_strict_int(name: str, value: object, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in range {minimum}..{maximum}")
    return value


def _require_uint64(name: str, value: object) -> int:
    if type(value) is not int or not 0 <= value <= _UINT64_MAX:
        raise ValueError(f"{name} must be a uint64 integer")
    return value


@dataclass(frozen=True, slots=True)
class LidarTopViewPoint:
    """单个 base_link 俯视投影点，仅保留平面坐标与企业语义。"""

    x: float
    y: float
    tag: int
    lidar_id: int

    def __post_init__(self) -> None:
        # 扫描器生成精确内建类型；等价快速校验保留公开边界并缩短逐点热路径。
        if (
            type(self.x) is float
            and math.isfinite(self.x)
            and type(self.y) is float
            and math.isfinite(self.y)
            and type(self.tag) is int
            and 0 <= self.tag <= 3
            and type(self.lidar_id) is int
            and 1 <= self.lidar_id <= 2
        ):
            return
        object.__setattr__(self, "x", _require_finite_number("x", self.x))
        object.__setattr__(self, "y", _require_finite_number("y", self.y))
        object.__setattr__(self, "tag", _require_strict_int("tag", self.tag, 0, 3))
        object.__setattr__(
            self,
            "lidar_id",
            _require_strict_int("lidar_id", self.lidar_id, 1, 2),
        )


@dataclass(frozen=True, slots=True)
class LidarTopViewFrame:
    """一台雷达当前帧的有序 base_link 俯视投影。"""

    timestamp_ns: int
    points: tuple[LidarTopViewPoint, ...]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "timestamp_ns",
            _require_uint64("timestamp_ns", self.timestamp_ns),
        )
        invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
        if isinstance(self.points, invalid_types) or not isinstance(self.points, Sequence):
            raise ValueError("points must be an ordered sequence of LidarTopViewPoint")
        points = tuple(self.points)
        if any(type(point) is not LidarTopViewPoint for point in points):
            raise ValueError("points must contain only LidarTopViewPoint values")
        # 立即复制调用方序列，冻结对象不能间接引用可变列表。
        object.__setattr__(self, "points", points)


def _validate_lidar_side(
    side: str,
    cloud: LidarPointCloud | None,
    view: LidarTopViewFrame | None,
) -> None:
    """校验可选点云和俯视帧各自属于声明侧，存在时还必须同帧。"""
    expected_frame = f"lidar_{side}"
    expected_id = 1 if side == "front" else 2
    cloud_name = f"lidar_{side}"
    view_name = f"lidar_{side}_view"
    if cloud is not None and type(cloud) is not LidarPointCloud:
        raise ValueError(f"{cloud_name} must be None or an exact LidarPointCloud")
    if view is not None and type(view) is not LidarTopViewFrame:
        raise ValueError(f"{view_name} must be None or an exact LidarTopViewFrame")
    if (cloud is None) != (view is None):
        raise ValueError(f"{cloud_name} and {view_name} must be present together")
    if cloud is None or view is None:
        return
    if cloud.frame_id != expected_frame or cloud.lidar_id != expected_id:
        raise ValueError(f"{cloud_name} frame_id and lidar_id do not match its side")
    if any(point.lidar_id != expected_id for point in view.points):
        raise ValueError(f"{view_name} lidar_id does not match its side")
    if cloud.timebase_ns != view.timestamp_ns:
        raise ValueError(f"{cloud_name} and {view_name} timestamps must match")
    if len(cloud.points) != len(view.points):
        raise ValueError(f"{cloud_name} and {view_name} point counts must match")
    for cloud_point, view_point in zip(cloud.points, view.points, strict=True):
        if cloud_point.tag != view_point.tag:
            raise ValueError(f"{cloud_name} and {view_name} point tags must match")


@dataclass(frozen=True, slots=True)
class InterfaceDashboardSnapshot:
    """同一 runtime 生命周期代内供 Qt 线程只读消费的组合快照。"""

    generation: int
    robot_model: str
    sim_time_ns: int
    status: InterfaceStatusSnapshot
    wheel_command: WheelCommand | None
    wheel_command_received_sim_time_ns: int | None
    wheel_state: WheelState | None
    lidar_front: LidarPointCloud | None
    lidar_rear: LidarPointCloud | None
    rtk: RtkState | None
    imu: ImuAttitude | None
    lidar_front_view: LidarTopViewFrame | None
    lidar_rear_view: LidarTopViewFrame | None

    def __post_init__(self) -> None:
        if type(self.generation) is not int or self.generation < 0:
            raise ValueError("generation must be a nonnegative integer")
        if type(self.robot_model) is not str or self.robot_model not in _ROBOT_MODELS:
            raise ValueError("robot_model must identify a supported robot model")
        _require_uint64("sim_time_ns", self.sim_time_ns)
        if type(self.status) is not InterfaceStatusSnapshot:
            raise ValueError("status must be an exact InterfaceStatusSnapshot")

        if self.wheel_command is not None and type(self.wheel_command) is not WheelCommand:
            raise ValueError("wheel_command must be None or an exact WheelCommand")
        command_time = self.wheel_command_received_sim_time_ns
        if (self.wheel_command is None) != (command_time is None):
            raise ValueError(
                "wheel_command_received_sim_time_ns must be present exactly when wheel_command is present"
            )
        if command_time is not None:
            _require_uint64("wheel_command_received_sim_time_ns", command_time)

        optional_types = (
            ("wheel_state", self.wheel_state, WheelState),
            ("rtk", self.rtk, RtkState),
            ("imu", self.imu, ImuAttitude),
        )
        for name, value, expected_type in optional_types:
            if value is not None and type(value) is not expected_type:
                raise ValueError(f"{name} must be None or an exact {expected_type.__name__}")

        _validate_lidar_side("front", self.lidar_front, self.lidar_front_view)
        _validate_lidar_side("rear", self.lidar_rear, self.lidar_rear_view)
