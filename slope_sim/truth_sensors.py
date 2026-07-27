# 真值传感器：定义语义安装外参，并从后端当前位姿生成双天线 RTK 与 IMU 输出。
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real

from slope_sim.interfaces.models import ImuAttitude, RtkState
from slope_sim.sensor_backend import Pose, Quaternion, SensorBackend, Vec3


RTK_HORIZONTAL_BASELINE_EPSILON_M = 1e-6
_EXACT_FLOAT_INTEGER_LIMIT = float(1 << 53)


@dataclass(frozen=True)
class MountPose:
    """传感器相对语义 parent link 的不可变安装位姿。"""

    parent_link: str
    position: Vec3
    orientation: Quaternion

    def __post_init__(self) -> None:
        if not isinstance(self.parent_link, str) or not self.parent_link:
            raise ValueError("parent link must be a nonempty semantic name")
        # 复用 Pose 的有限值与单位四元数约束，避免两套安装校验规则漂移。
        pose = Pose(self.position, self.orientation)
        object.__setattr__(self, "position", pose.position)
        object.__setattr__(self, "orientation", pose.orientation)


@dataclass(frozen=True)
class SensorMounts:
    """当前车型共享的前后雷达、双 RTK 天线和 IMU 安装配置。"""

    lidar_front: MountPose
    lidar_rear: MountPose
    rtk_primary: MountPose
    rtk_secondary: MountPose
    imu: MountPose

    def __post_init__(self) -> None:
        for name, mount in self.named_mounts():
            if not isinstance(mount, MountPose):
                raise ValueError(f"{name} must be a MountPose")

    @classmethod
    def default(cls) -> "SensorMounts":
        """返回阶段三确认的四车型共用默认安装外参。"""
        identity = (0.0, 0.0, 0.0, 1.0)
        return cls(
            lidar_front=MountPose(
                "lidar_front_mount",
                (0.0, 0.0, 0.0),
                identity,
            ),
            lidar_rear=MountPose(
                "lidar_rear_mount",
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
            ),
            rtk_primary=MountPose(
                "base_link",
                (-0.20, 0.0, 0.18),
                identity,
            ),
            rtk_secondary=MountPose(
                "base_link",
                (0.20, 0.0, 0.18),
                identity,
            ),
            imu=MountPose(
                "base_link",
                (0.0, 0.0, 0.08),
                identity,
            ),
        )

    def named_mounts(self) -> tuple[tuple[str, MountPose], ...]:
        """按稳定顺序返回字段名和安装点，供加载时整组校验。"""
        return (
            ("lidar_front", self.lidar_front),
            ("lidar_rear", self.lidar_rear),
            ("rtk_primary", self.rtk_primary),
            ("rtk_secondary", self.rtk_secondary),
            ("imu", self.imu),
        )


def wrap_angle(angle: float) -> float:
    """把有限角度精确归一化到半开区间 [-pi, pi)。"""
    if isinstance(angle, bool) or not isinstance(angle, Real):
        raise ValueError("angle must be finite")
    normalized = float(angle)
    if not math.isfinite(normalized):
        raise ValueError("angle must be finite")
    wrapped = math.atan2(math.sin(normalized), math.cos(normalized))
    quotient = round(normalized / math.pi)
    # 在整数仍可由 float 精确区分的范围内，规范化由 float pi 构造的奇数倍。
    is_exact_odd_pi = (
        normalized != 0.0
        and abs(normalized) < _EXACT_FLOAT_INTEGER_LIMIT
        and normalized == quotient * math.pi
        and quotient % 2 != 0
    )
    if wrapped >= math.pi or is_exact_odd_pi:
        wrapped = -math.pi
    return wrapped


class TruthSensorSuite:
    """从后端当前世界真值即时生成 RTK 和 IMU，不缓存、不滤波、不加噪声。"""

    def __init__(self, backend: SensorBackend, mounts: SensorMounts) -> None:
        if not isinstance(mounts, SensorMounts):
            raise ValueError("mounts must be a SensorMounts value")
        self._backend = backend
        self.mounts = mounts
        self._validate_parent_links()

    def _validate_parent_links(self) -> None:
        """加载或车型切换时一次验证全部语义 parent link。"""
        try:
            link_names = tuple(self._backend.link_names())
        except (AttributeError, TypeError) as exc:
            raise ValueError("sensor backend must provide semantic link names") from exc
        if any(not isinstance(name, str) or not name for name in link_names):
            raise ValueError("sensor backend link names must be nonempty strings")
        known_links = set(link_names)
        for mount_name, mount in self.mounts.named_mounts():
            if mount.parent_link not in known_links:
                raise ValueError(
                    f"{mount_name} parent link {mount.parent_link!r} "
                    "does not exist on the current robot"
                )

    def _world_mount(self, mount: MountPose) -> Pose:
        """每次读取都重新组合当前 parent 真值和固定局部外参。"""
        parent_pose = self._backend.world_pose(mount.parent_link)
        local_pose = Pose(mount.position, mount.orientation)
        return self._backend.transform_pose(parent_pose, local_pose)

    def read_rtk(self, timestamp_ns: int) -> RtkState:
        """输出主天线世界位置与主天线指向副天线的水平 yaw。"""
        primary = self._world_mount(self.mounts.rtk_primary).position
        secondary = self._world_mount(self.mounts.rtk_secondary).position
        baseline_dx = secondary[0] - primary[0]
        baseline_dy = secondary[1] - primary[1]
        horizontal_baseline = math.hypot(baseline_dx, baseline_dy)
        if horizontal_baseline <= RTK_HORIZONTAL_BASELINE_EPSILON_M:
            raise ValueError(
                "RTK horizontal baseline must be greater than "
                f"{RTK_HORIZONTAL_BASELINE_EPSILON_M} m; "
                f"got {horizontal_baseline:.17g} m"
            )
        baseline_yaw = wrap_angle(math.atan2(baseline_dy, baseline_dx))
        return RtkState(
            timestamp_ns,
            primary[0],
            primary[1],
            primary[2],
            baseline_yaw,
        )

    def read_imu(self, timestamp_ns: int) -> ImuAttitude:
        """从 IMU 世界四元数直接输出 roll/pitch 真值。"""
        imu_pose = self._world_mount(self.mounts.imu)
        roll, pitch, _yaw = self._backend.euler_from_quaternion(
            imu_pose.orientation
        )
        return ImuAttitude(timestamp_ns, roll, pitch)
