# 企业接口配置：集中定义传输模式、六个固定通道和有界运行参数。
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real


_TRANSPORT_MODES = frozenset({"auto", "ecal", "local"})
_DIRECTIONS = frozenset({"publish", "subscribe"})


def _require_positive_float(name: str, value: object) -> float:
    """校验需要严格为正且有限的时间参数。"""
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a positive finite number")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return normalized


def _require_positive_int(name: str, value: object) -> int:
    """校验频率和队列容量，避免 bool 被当作整数一使用。"""
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


@dataclass(frozen=True)
class ChannelConfig:
    """单个企业话题的名称、目标频率和数据方向。"""

    topic: str
    rate_hz: int
    direction: str

    def __post_init__(self) -> None:
        if not isinstance(self.topic, str) or not self.topic:
            raise ValueError("topic must be a nonempty string")
        _require_positive_int("rate_hz", self.rate_hz)
        if not isinstance(self.direction, str) or self.direction not in _DIRECTIONS:
            raise ValueError("direction must be 'publish' or 'subscribe'")


@dataclass(frozen=True)
class InterfaceConfig:
    """阶段三接口运行时共享的唯一通道和容量配置。"""

    transport_mode: str
    wheel_command: ChannelConfig
    wheel_state: ChannelConfig
    lidar_front: ChannelConfig
    lidar_rear: ChannelConfig
    rtk: ChannelConfig
    imu: ChannelConfig
    command_timeout_sec: float = 0.100
    status_window_sec: float = 2.0
    outgoing_queue_size: int = 32
    log_queue_size: int = 256

    def __post_init__(self) -> None:
        """集中拒绝未知模式、重复话题和无效的有界运行参数。"""
        if not isinstance(self.transport_mode, str) or self.transport_mode not in _TRANSPORT_MODES:
            raise ValueError("transport_mode must be 'auto', 'ecal', or 'local'")
        if any(not isinstance(channel, ChannelConfig) for channel in self.channels):
            raise ValueError("all interface channels must be ChannelConfig values")
        topics = tuple(channel.topic for channel in self.channels)
        if len(set(topics)) != len(topics):
            raise ValueError("duplicate interface topic")
        object.__setattr__(
            self,
            "command_timeout_sec",
            _require_positive_float("command_timeout_sec", self.command_timeout_sec),
        )
        object.__setattr__(
            self,
            "status_window_sec",
            _require_positive_float("status_window_sec", self.status_window_sec),
        )
        _require_positive_int("outgoing_queue_size", self.outgoing_queue_size)
        _require_positive_int("log_queue_size", self.log_queue_size)

    @property
    def channels(self) -> tuple[ChannelConfig, ...]:
        """按命令、状态、前后雷达、RTK、IMU 的固定顺序返回六个通道。"""
        return (
            self.wheel_command,
            self.wheel_state,
            self.lidar_front,
            self.lidar_rear,
            self.rtk,
            self.imu,
        )

    @classmethod
    def default(cls, *, transport_mode: str = "auto") -> "InterfaceConfig":
        """创建阶段三设计约定的六个默认企业通道。"""
        return cls(
            transport_mode=transport_mode,
            wheel_command=ChannelConfig("/sim/wheel/command", 100, "subscribe"),
            wheel_state=ChannelConfig("/sim/wheel/state", 100, "publish"),
            lidar_front=ChannelConfig("/sim/lidar/front/points", 10, "publish"),
            lidar_rear=ChannelConfig("/sim/lidar/rear/points", 10, "publish"),
            rtk=ChannelConfig("/sim/rtk/state", 10, "publish"),
            imu=ChannelConfig("/sim/imu/attitude", 10, "publish"),
        )
