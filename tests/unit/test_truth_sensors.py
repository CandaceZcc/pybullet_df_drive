# 真值传感器单元测试：用纯 Python fake backend 验证安装外参、双天线 RTK 和 IMU 数学。
from __future__ import annotations

import dataclasses
from dataclasses import FrozenInstanceError
import math

import pytest

import slope_sim.truth_sensors as truth_sensors
from slope_sim.sensor_backend import Pose
from slope_sim.truth_sensors import (
    MountPose,
    SensorMounts,
    TruthSensorSuite,
    wrap_angle,
)


IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)
RTK_BASELINE_EPSILON_M = 1e-6


def _quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(orientation, vector):
    """用单位四元数旋转三维向量，fake backend 不依赖 PyBullet。"""
    conjugate = (-orientation[0], -orientation[1], -orientation[2], orientation[3])
    rotated = _quaternion_multiply(
        _quaternion_multiply(orientation, (vector[0], vector[1], vector[2], 0.0)),
        conjugate,
    )
    return rotated[:3]


def _quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _euler_from_quaternion(orientation):
    x, y, z, w = orientation
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _trigonometric_angle_reference(angle: float) -> float:
    """用 libm sin/cos 的高精度范围约简建立独立大角度参考。"""
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    return -math.pi if wrapped >= math.pi else wrapped


class FakeSensorBackend:
    """只实现传感器窄协议，算法单测不得接触 PyBullet。"""

    def __init__(self) -> None:
        self.base_pose = Pose((0.0, 0.0, 0.0), IDENTITY_QUATERNION)
        self.world_pose_calls = []
        self._links = (
            "base_link",
            "lidar_link",
            "lidar_front_mount",
            "lidar_rear_mount",
        )

    def link_names(self):
        return self._links

    def world_pose(self, parent_link):
        if parent_link not in self._links:
            raise ValueError(f"unknown parent link: {parent_link}")
        self.world_pose_calls.append(parent_link)
        return self.base_pose

    def transform_pose(self, parent, local):
        rotated = _rotate_vector(parent.orientation, local.position)
        return Pose(
            tuple(parent.position[index] + rotated[index] for index in range(3)),
            _quaternion_multiply(parent.orientation, local.orientation),
        )

    def inverse_transform_point(self, pose, point):
        relative = tuple(point[index] - pose.position[index] for index in range(3))
        conjugate = (-pose.orientation[0], -pose.orientation[1], -pose.orientation[2], pose.orientation[3])
        return _rotate_vector(conjugate, relative)

    def euler_from_quaternion(self, orientation):
        return _euler_from_quaternion(orientation)

    def ray_test_batch(self, starts, ends, *, collision_mask):
        raise AssertionError("RTK/IMU truth must not perform ray tests")


@pytest.fixture
def fake_backend():
    return FakeSensorBackend()


def test_default_mounts_match_confirmed_stage3_geometry():
    mounts = SensorMounts.default()

    assert mounts.lidar_front == MountPose("lidar_front_mount", (0.0, 0.0, 0.0), IDENTITY_QUATERNION)
    assert mounts.lidar_rear == MountPose("lidar_rear_mount", (0.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0))
    assert mounts.rtk_primary == MountPose("base_link", (-0.20, 0.0, 0.18), IDENTITY_QUATERNION)
    assert mounts.rtk_secondary == MountPose("base_link", (0.20, 0.0, 0.18), IDENTITY_QUATERNION)
    assert mounts.imu == MountPose("base_link", (0.0, 0.0, 0.08), IDENTITY_QUATERNION)


def test_stage4_default_mounts_define_one_lidar_and_three_rtk_points():
    """阶段四安装组不能沿用阶段三前后双雷达、双天线命名。"""
    stage4_mounts_type = getattr(truth_sensors, "Stage4SensorMounts", None)
    assert stage4_mounts_type is not None, "Stage4 sensor mount contract must exist"

    mounts = stage4_mounts_type.default()

    assert mounts.lidar == MountPose("lidar_link", (0.0, 0.0, 0.0), IDENTITY_QUATERNION)
    assert mounts.rtk_left == MountPose("base_link", (0.0, 0.20, 0.18), IDENTITY_QUATERNION)
    assert mounts.rtk_center == MountPose("base_link", (0.0, 0.0, 0.18), IDENTITY_QUATERNION)
    assert mounts.rtk_right == MountPose("base_link", (0.0, -0.20, 0.18), IDENTITY_QUATERNION)
    assert mounts.imu == MountPose("base_link", (0.0, 0.0, 0.08), IDENTITY_QUATERNION)
    assert tuple(name for name, _mount in mounts.named_mounts()) == (
        "lidar",
        "rtk_left",
        "rtk_center",
        "rtk_right",
        "imu",
    )


def test_stage4_mounts_reject_a_degenerate_local_rtk_baseline():
    """LEFT/RIGHT 局部位置退化时不能等到运行中才暴露无航向。"""
    mounts = truth_sensors.Stage4SensorMounts.default()

    with pytest.raises(ValueError, match="RTK local horizontal baseline"):
        dataclasses.replace(mounts, rtk_right=mounts.rtk_left)


def test_stage4_truth_suite_reads_three_world_rtk_points_and_baseline_heading(fake_backend):
    """三点 RTK 必须来自同一真值姿态，并由世界基线计算 heading。"""
    suite_type = getattr(truth_sensors, "Stage4TruthSensorSuite", None)
    assert suite_type is not None, "Stage4 truth sensor suite must exist"
    stage4_mounts = truth_sensors.Stage4SensorMounts.default()
    roll, pitch, yaw = 0.37, -0.29, 0.81
    fake_backend.base_pose = Pose(
        (4.0, -3.0, 1.2),
        _quaternion_from_euler(roll, pitch, yaw),
    )
    suite = suite_type(fake_backend, stage4_mounts)

    state = suite.read_rtk(timestamp_ns=20)

    assert state.timestamp_ns == 20
    expected_center = fake_backend.transform_pose(
        fake_backend.base_pose,
        Pose(stage4_mounts.rtk_center.position, stage4_mounts.rtk_center.orientation),
    ).position
    assert state.center == pytest.approx(expected_center)
    assert state.left[2] != pytest.approx(state.right[2])
    assert state.heading_rad == pytest.approx(
        heading_from_baseline := truth_sensors.heading_from_rtk_baseline(
            state.left,
            state.right,
        ),
        abs=1e-12,
    )
    assert state.heading_rad != pytest.approx(yaw, abs=1e-4)


def test_stage4_rtk_center_is_constructed_as_the_exact_world_midpoint() -> None:
    """后端逐点变换的单精度回差不得破坏冻结的三点 RTK 几何。"""

    class QuantizedCenterBackend(FakeSensorBackend):
        def transform_pose(self, parent, local):
            transformed = super().transform_pose(parent, local)
            if local.position == (0.0, 0.0, 0.18):
                return Pose(
                    (
                        transformed.position[0] + 2.0e-7,
                        transformed.position[1],
                        transformed.position[2],
                    ),
                    transformed.orientation,
                )
            return transformed

    backend = QuantizedCenterBackend()
    backend.base_pose = Pose(
        (-3.499, -0.009, 0.107),
        _quaternion_from_euler(0.048, 0.005, 0.01),
    )
    state = truth_sensors.Stage4TruthSensorSuite(
        backend,
        truth_sensors.Stage4SensorMounts.default(),
    ).read_rtk(0)

    assert state.center == tuple(
        (state.left[index] + state.right[index]) / 2.0 for index in range(3)
    )


def test_stage4_truth_suite_freezes_one_parent_pose_for_a_three_point_rtk_sample(
    fake_backend,
):
    """同一 RTK 样本只能读取一次共享 parent，防止三个点跨物理帧。"""
    suite = truth_sensors.Stage4TruthSensorSuite(
        fake_backend,
        truth_sensors.Stage4SensorMounts.default(),
    )
    fake_backend.world_pose_calls.clear()

    suite.read_rtk(timestamp_ns=30)

    assert fake_backend.world_pose_calls == ["base_link"]


def test_mount_pose_is_immutable_and_normalizes_quaternion():
    mount = MountPose("base_link", (0, 0, 0), (0, 0, 0, 2))

    assert mount.position == (0.0, 0.0, 0.0)
    assert mount.orientation == IDENTITY_QUATERNION
    with pytest.raises(FrozenInstanceError):
        mount.parent_link = "other"


@pytest.mark.parametrize(
    ("parent_link", "position", "orientation", "message"),
    [
        ("", (0.0, 0.0, 0.0), IDENTITY_QUATERNION, "parent link"),
        ("base_link", (0.0, math.nan, 0.0), IDENTITY_QUATERNION, "position"),
        ("base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0), "quaternion"),
        ("base_link", (0.0, 0.0, 0.0), (math.inf, 0.0, 0.0, 1.0), "quaternion"),
    ],
)
def test_mount_pose_rejects_invalid_parent_position_and_quaternion(
    parent_link,
    position,
    orientation,
    message,
):
    with pytest.raises(ValueError, match=message):
        MountPose(parent_link, position, orientation)


def test_suite_rejects_unknown_semantic_parent_link(fake_backend):
    mounts = SensorMounts.default()
    invalid_parent = dataclasses.replace(
        mounts,
        imu=MountPose("missing", mounts.imu.position, mounts.imu.orientation),
    )

    with pytest.raises(ValueError, match="parent link.*missing"):
        TruthSensorSuite(fake_backend, invalid_parent)


def test_sensor_mounts_are_immutable(fake_backend):
    mounts = SensorMounts.default()
    TruthSensorSuite(fake_backend, mounts)

    with pytest.raises(FrozenInstanceError):
        mounts.imu = mounts.rtk_primary


def test_rtk_uses_primary_world_position_and_primary_to_secondary_yaw(fake_backend):
    yaw = math.pi - 0.1
    fake_backend.base_pose = Pose((3.0, 4.0, 1.0), _quaternion_from_euler(0.0, 0.0, yaw))
    suite = TruthSensorSuite(fake_backend, SensorMounts.default())

    state = suite.read_rtk(timestamp_ns=10)

    assert state.timestamp_ns == 10
    assert (state.main_x, state.main_y, state.main_z) == pytest.approx(
        (
            3.0 - 0.20 * math.cos(yaw),
            4.0 - 0.20 * math.sin(yaw),
            1.18,
        )
    )
    assert -math.pi <= state.baseline_yaw_rad < math.pi
    assert state.baseline_yaw_rad == pytest.approx(yaw)


def test_stage4_heading_uses_right_to_left_world_baseline_under_roll_and_pitch():
    """三点 RTK 的航向由水平基线恢复，不能错误复用 Euler yaw。"""
    roll, pitch, yaw = 0.37, -0.29, 0.81
    orientation = _quaternion_from_euler(roll, pitch, yaw)
    center = (4.0, -3.0, 1.2)
    right_local = (0.0, -0.31, 0.0)
    left_local = (0.0, 0.31, 0.0)
    right = tuple(
        center[index] + _rotate_vector(orientation, right_local)[index]
        for index in range(3)
    )
    left = tuple(
        center[index] + _rotate_vector(orientation, left_local)[index]
        for index in range(3)
    )

    heading_from_baseline = getattr(
        truth_sensors,
        "heading_from_rtk_baseline",
        None,
    )
    assert callable(heading_from_baseline), "stage4 RTK baseline heading function must exist"
    heading = heading_from_baseline(left, right)

    expected = math.atan2(left[1] - right[1], left[0] - right[0]) - math.pi / 2.0
    assert heading == pytest.approx(wrap_angle(expected), abs=1e-12)
    assert heading != pytest.approx(yaw, abs=1e-4)


def test_rtk_rejects_coincident_primary_and_secondary_antennas(fake_backend):
    mounts = SensorMounts.default()
    coincident_mounts = dataclasses.replace(
        mounts,
        rtk_secondary=mounts.rtk_primary,
    )

    with pytest.raises(ValueError, match="horizontal baseline"):
        TruthSensorSuite(fake_backend, coincident_mounts).read_rtk(1)


@pytest.mark.parametrize(
    "pitch",
    [
        math.pi / 2.0,
        math.pi / 2.0 - 0.5 * RTK_BASELINE_EPSILON_M / 0.4,
        math.pi / 2.0 + 0.5 * RTK_BASELINE_EPSILON_M / 0.4,
    ],
)
def test_default_rtk_rejects_vertical_or_sub_epsilon_horizontal_baseline(
    fake_backend,
    pitch,
):
    fake_backend.base_pose = Pose(
        (0.0, 0.0, 0.0),
        _quaternion_from_euler(0.0, pitch, 0.7),
    )

    with pytest.raises(ValueError, match="horizontal baseline"):
        TruthSensorSuite(fake_backend, SensorMounts.default()).read_rtk(1)


@pytest.mark.parametrize("pitch_side", (-1.0, 1.0))
def test_default_rtk_accepts_horizontal_baseline_just_above_epsilon(
    fake_backend,
    pitch_side,
):
    pitch = math.pi / 2.0 + pitch_side * 2.0 * RTK_BASELINE_EPSILON_M / 0.4
    fake_backend.base_pose = Pose(
        (0.0, 0.0, 0.0),
        _quaternion_from_euler(0.0, pitch, -0.4),
    )

    state = TruthSensorSuite(fake_backend, SensorMounts.default()).read_rtk(1)

    assert math.isfinite(state.baseline_yaw_rad)
    assert -math.pi <= state.baseline_yaw_rad < math.pi


@pytest.mark.parametrize(
    ("baseline_length", "rejected"),
    [
        (math.nextafter(RTK_BASELINE_EPSILON_M, 0.0), True),
        (RTK_BASELINE_EPSILON_M, True),
        (math.nextafter(RTK_BASELINE_EPSILON_M, math.inf), False),
    ],
)
def test_rtk_horizontal_baseline_nextafter_contract(
    fake_backend,
    baseline_length,
    rejected,
):
    mounts = SensorMounts.default()
    boundary_mounts = dataclasses.replace(
        mounts,
        rtk_primary=MountPose("base_link", (0.0, 0.0, 0.0), IDENTITY_QUATERNION),
        rtk_secondary=MountPose(
            "base_link",
            (baseline_length, 0.0, 0.0),
            IDENTITY_QUATERNION,
        ),
    )
    suite = TruthSensorSuite(fake_backend, boundary_mounts)

    if rejected:
        with pytest.raises(ValueError, match="greater than 1e-06 m"):
            suite.read_rtk(1)
    else:
        assert suite.read_rtk(1).baseline_yaw_rad == pytest.approx(0.0)


@pytest.mark.parametrize(
    ("yaw", "expected"),
    [
        (math.pi, -math.pi),
        (-math.pi, -math.pi),
        (math.pi + 1e-9, -math.pi + 1e-9),
    ],
)
def test_rtk_yaw_wraps_exact_pi_boundary(yaw, expected, fake_backend):
    fake_backend.base_pose = Pose((0.0, 0.0, 0.0), _quaternion_from_euler(0.0, 0.0, yaw))

    state = TruthSensorSuite(fake_backend, SensorMounts.default()).read_rtk(1)

    assert state.baseline_yaw_rad == pytest.approx(expected, abs=1e-12)


def test_wrap_angle_returns_half_open_interval_and_rejects_non_finite_values():
    assert wrap_angle(math.pi) == -math.pi
    assert wrap_angle(-math.pi) == -math.pi
    assert wrap_angle(3.0 * math.pi) == -math.pi
    assert wrap_angle(11.0 * math.pi) == -math.pi
    assert wrap_angle(-11.0 * math.pi) == -math.pi
    assert wrap_angle(101.0 * math.pi) == -math.pi
    assert -math.pi <= wrap_angle(-8.2) < math.pi
    with pytest.raises(ValueError, match="finite"):
        wrap_angle(math.nan)


@pytest.mark.parametrize(
    "angle",
    (1e12 + 0.125, 1e16, 1e20, -1e20),
)
def test_wrap_angle_matches_trigonometric_reference_for_large_floats(angle):
    expected = _trigonometric_angle_reference(angle)

    actual = wrap_angle(angle)

    assert -math.pi <= actual < math.pi
    assert actual == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize(
    "angle",
    [
        math.pi,
        -math.pi,
        math.nextafter(math.pi, 0.0),
        math.nextafter(math.pi, math.inf),
        math.nextafter(-math.pi, 0.0),
        math.nextafter(-math.pi, -math.inf),
    ],
)
def test_wrap_angle_preserves_half_open_boundary_at_pi_and_nextafter(angle):
    expected = _trigonometric_angle_reference(angle)

    actual = wrap_angle(angle)

    assert -math.pi <= actual < math.pi
    assert actual == pytest.approx(expected, abs=1e-15)


def test_imu_uses_current_world_mount_quaternion_without_noise(fake_backend):
    roll, pitch, yaw = 0.31, -0.27, 1.2
    fake_backend.base_pose = Pose((0.0, 0.0, 2.0), _quaternion_from_euler(roll, pitch, yaw))
    suite = TruthSensorSuite(fake_backend, SensorMounts.default())

    state = suite.read_imu(timestamp_ns=20)

    assert state.timestamp_ns == 20
    assert state.roll_rad == pytest.approx(roll, abs=1e-12)
    assert state.pitch_rad == pytest.approx(pitch, abs=1e-12)


def test_each_read_recomputes_from_current_backend_truth(fake_backend):
    suite = TruthSensorSuite(fake_backend, SensorMounts.default())
    first_rtk = suite.read_rtk(1)
    first_imu = suite.read_imu(1)

    fake_backend.base_pose = Pose(
        (5.0, -2.0, 0.5),
        _quaternion_from_euler(-0.2, 0.15, -0.7),
    )
    second_rtk = suite.read_rtk(2)
    second_imu = suite.read_imu(2)

    assert (second_rtk.main_x, second_rtk.main_y, second_rtk.main_z) != pytest.approx(
        (first_rtk.main_x, first_rtk.main_y, first_rtk.main_z)
    )
    assert second_rtk.baseline_yaw_rad != pytest.approx(first_rtk.baseline_yaw_rad)
    assert second_imu.roll_rad != pytest.approx(first_imu.roll_rad)
    assert second_imu.pitch_rad != pytest.approx(first_imu.pitch_rad)
