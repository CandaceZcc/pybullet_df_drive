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


@dataclass(frozen=True)
class Stage4SensorMounts:
    """阶段四单中心 LiDAR、三点 RTK 和 IMU 的独立安装合同。"""

    lidar: MountPose
    rtk_left: MountPose
    rtk_center: MountPose
    rtk_right: MountPose
    imu: MountPose

    def __post_init__(self) -> None:
        for name, mount in self.named_mounts():
            if not isinstance(mount, MountPose):
                raise ValueError(f"{name} must be a MountPose")
        if self.rtk_left.parent_link != self.rtk_right.parent_link:
            raise ValueError("RTK local baseline endpoints must share a parent link")
        local_dx = self.rtk_left.position[0] - self.rtk_right.position[0]
        local_dy = self.rtk_left.position[1] - self.rtk_right.position[1]
        if math.hypot(local_dx, local_dy) <= RTK_HORIZONTAL_BASELINE_EPSILON_M:
            raise ValueError(
                "RTK local horizontal baseline must be greater than "
                f"{RTK_HORIZONTAL_BASELINE_EPSILON_M} m"
            )

    @classmethod
    def default(cls) -> "Stage4SensorMounts":
        """返回四车型共享的中心 LiDAR 与三点 RTK canonical 几何。"""
        identity = (0.0, 0.0, 0.0, 1.0)
        return cls(
            lidar=MountPose("lidar_link", (0.0, 0.0, 0.0), identity),
            rtk_left=MountPose("base_link", (0.0, 0.20, 0.18), identity),
            rtk_center=MountPose("base_link", (0.0, 0.0, 0.18), identity),
            rtk_right=MountPose("base_link", (0.0, -0.20, 0.18), identity),
            imu=MountPose("base_link", (0.0, 0.0, 0.08), identity),
        )

    def named_mounts(self) -> tuple[tuple[str, MountPose], ...]:
        """以稳定顺序暴露阶段四安装角色，供新 runtime 一次性校验。"""
        return (
            ("lidar", self.lidar),
            ("rtk_left", self.rtk_left),
            ("rtk_center", self.rtk_center),
            ("rtk_right", self.rtk_right),
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


def heading_from_rtk_baseline(left: Vec3, right: Vec3) -> float:
    """由世界坐标 RIGHT 到 LEFT 的水平基线恢复车体 +X 航向。"""
    identity = (0.0, 0.0, 0.0, 1.0)
    left_point = Pose(left, identity).position
    right_point = Pose(right, identity).position
    baseline_dx = left_point[0] - right_point[0]
    baseline_dy = left_point[1] - right_point[1]
    horizontal_baseline = math.hypot(baseline_dx, baseline_dy)
    if horizontal_baseline <= RTK_HORIZONTAL_BASELINE_EPSILON_M:
        raise ValueError(
            "RTK RIGHT-to-LEFT horizontal baseline must be greater than "
            f"{RTK_HORIZONTAL_BASELINE_EPSILON_M} m; "
            f"got {horizontal_baseline:.17g} m"
        )
    return wrap_angle(math.atan2(baseline_dy, baseline_dx) - math.pi / 2.0)


@dataclass(frozen=True)
class Stage4RtkState:
    """阶段四 RTK 的三个世界点与由基线恢复的车体航向。"""

    timestamp_ns: int
    left: Vec3
    center: Vec3
    right: Vec3
    heading_rad: float

    def __post_init__(self) -> None:
        if isinstance(self.timestamp_ns, bool) or not isinstance(self.timestamp_ns, int):
            raise ValueError("timestamp_ns must be an integer")
        if not 0 <= self.timestamp_ns <= (1 << 64) - 1:
            raise ValueError("timestamp_ns must fit uint64")
        identity = (0.0, 0.0, 0.0, 1.0)
        object.__setattr__(self, "left", Pose(self.left, identity).position)
        object.__setattr__(self, "center", Pose(self.center, identity).position)
        object.__setattr__(self, "right", Pose(self.right, identity).position)
        object.__setattr__(self, "heading_rad", wrap_angle(self.heading_rad))


class Stage4TruthSensorSuite:
    """为 v2 runtime 提供单中心 LiDAR 配套的三点 RTK 与 IMU 真值。"""

    def __init__(self, backend: SensorBackend, mounts: Stage4SensorMounts) -> None:
        if not isinstance(mounts, Stage4SensorMounts):
            raise ValueError("mounts must be a Stage4SensorMounts value")
        self._backend = backend
        self.mounts = mounts
        self._validate_parent_links()

    def _validate_parent_links(self) -> None:
        """加载或车型切换时校验阶段四全部语义安装点。"""
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

    def _world_mounts(self, mounts: tuple[MountPose, ...]) -> tuple[Pose, ...]:
        """按 parent link 冻结一次世界位姿，再批量组合该帧全部局部外参。"""
        parent_poses: dict[str, Pose] = {}
        world_mounts: list[Pose] = []
        for mount in mounts:
            if mount.parent_link not in parent_poses:
                parent_pose = self._backend.world_pose(mount.parent_link)
                if not isinstance(parent_pose, Pose):
                    raise RuntimeError("sensor backend world_pose must return Pose")
                parent_poses[mount.parent_link] = parent_pose
            world_mount = self._backend.transform_pose(
                parent_poses[mount.parent_link],
                Pose(mount.position, mount.orientation),
            )
            if not isinstance(world_mount, Pose):
                raise RuntimeError("sensor backend transform_pose must return Pose")
            world_mounts.append(world_mount)
        return tuple(world_mounts)

    def _world_mount(self, mount: MountPose) -> Pose:
        """组合单个安装点，供 IMU 等非批量读取复用同一校验路径。"""
        return self._world_mounts((mount,))[0]

    def read_rtk(self, timestamp_ns: int) -> Stage4RtkState:
        """同次读取三个 RTK 世界点，并用 RIGHT 到 LEFT 基线恢复航向。"""
        left_pose, _center_pose, right_pose = self._world_mounts(
            (
                self.mounts.rtk_left,
                self.mounts.rtk_center,
                self.mounts.rtk_right,
            )
        )
        left = left_pose.position
        right = right_pose.position
        # 三个 mount 理论上共线；以同批端点中点消除后端逐点 float 回差。
        center = tuple(
            (left[index] + right[index]) / 2.0 for index in range(3)
        )
        return Stage4RtkState(
            timestamp_ns,
            left,
            center,
            right,
            heading_from_rtk_baseline(left, right),
        )

    def read_imu(self, timestamp_ns: int) -> ImuAttitude:
        """阶段四 IMU 继续直接输出当前安装点的 roll/pitch 真值。"""
        imu_pose = self._world_mount(self.mounts.imu)
        roll, pitch, _yaw = self._backend.euler_from_quaternion(
            imu_pose.orientation
        )
        return ImuAttitude(timestamp_ns, roll, pitch)


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
