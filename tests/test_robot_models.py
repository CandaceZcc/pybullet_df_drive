# 阶段一车型测试：先锁定四种模型的结构、控制语义和主动转向边界。
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pybullet as p
import pytest

import slope_sim.robot as robot_module
from slope_sim.model_registry import ROBOT_MODELS, get_robot_model, robot_model_names
from slope_sim.interfaces.models import WheelCommand
from slope_sim.robot import ActiveSteeringRobot, DifferentialDriveRobot, create_robot


def _load_robot(model_name: str):
    client_id = p.connect(p.DIRECT)
    robot = create_robot(client_id, model_name)
    return client_id, robot


def _step(client_id: int, count: int = 60) -> None:
    for _ in range(count):
        p.stepSimulation(physicsClientId=client_id)


def test_stage1_exposes_only_four_robot_models_and_existing_track_is_rejected():
    assert robot_model_names() == ("df_front", "df_mid", "df_back", "active_steering_4wd")
    assert set(ROBOT_MODELS) == set(robot_model_names())
    with pytest.raises(ValueError, match="robot_model"):
        get_robot_model("tracked_proxy")


def test_robot_factory_removes_body_when_constructor_fails_after_urdf_load(monkeypatch):
    """URDF 已加载后若关节解析失败，工厂不能遗留半成品刚体。"""
    client_id = p.connect(p.DIRECT)
    try:
        before_ids = {
            p.getBodyUniqueId(index, physicsClientId=client_id)
            for index in range(p.getNumBodies(physicsClientId=client_id))
        }

        def fail_joint_resolution(_robot):
            raise RuntimeError("injected joint resolution failure")

        monkeypatch.setattr(robot_module.DifferentialDriveRobot, "_find_wheel_joints", fail_joint_resolution)
        with pytest.raises(RuntimeError, match="injected joint resolution failure"):
            create_robot(client_id, "df_back")

        after_ids = {
            p.getBodyUniqueId(index, physicsClientId=client_id)
            for index in range(p.getNumBodies(physicsClientId=client_id))
        }
        assert after_ids == before_ids
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(
    ("model_name", "expected_drive_x", "expected_support_count"),
    [
        ("df_front", 0.22, 1),
        ("df_mid", 0.0, 2),
        ("df_back", -0.22, 1),
    ],
)
def test_differential_model_metadata_and_joint_layout(model_name: str, expected_drive_x: float, expected_support_count: int):
    spec = get_robot_model(model_name)
    assert spec.controller_kind == "differential"
    assert len(spec.drive_joint_names) == 2
    assert len(spec.support_link_names) == expected_support_count
    assert spec.drive_center_x == pytest.approx(expected_drive_x)
    assert Path(spec.urdf_path).exists()


def test_active_steering_metadata_has_four_drive_and_two_steering_joints():
    spec = get_robot_model("active_steering_4wd")
    assert spec.controller_kind == "active_steering"
    assert len(spec.drive_joint_names) == 4
    assert spec.steering_joint_names == ("front_left_steering_joint", "front_right_steering_joint")


@pytest.mark.parametrize("model_name", robot_model_names())
def test_all_robot_models_keep_stage3_mechanical_speed_limits(model_name: str):
    """四种车型共享稳定的驱动轮与转向轮速度机械限位。"""
    spec = get_robot_model(model_name)
    assert spec.max_drive_wheel_speed_rad_s == pytest.approx(20.0)
    assert spec.max_steering_speed_rad_s == pytest.approx(2.0)


@pytest.mark.parametrize("model_name", ["df_front", "df_mid", "df_back"])
def test_differential_robot_resolves_named_drive_joints_and_supports_twist(model_name: str):
    client_id, robot = _load_robot(model_name)
    try:
        assert isinstance(robot, DifferentialDriveRobot)
        assert len(robot.drive_wheel_joint_indices) == 2
        left_speed, right_speed = robot.command_twist(0.4, 0.8)
        _step(client_id)
        assert left_speed < right_speed
        assert robot.read_drive_wheel_speeds() == pytest.approx((left_speed, right_speed), abs=0.05)
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("model_name", ["df_front", "df_mid", "df_back"])
def test_task12_differential_wheel_command_from_twist_is_pure(model_name: str, monkeypatch):
    client_id, robot = _load_robot(model_name)
    try:
        before = (robot.left_wheel_speed, robot.right_wheel_speed, robot.linear_velocity, robot.angular_velocity)
        monkeypatch.setattr(
            robot_module.p,
            "setJointMotorControl2",
            lambda *args, **kwargs: pytest.fail("pure conversion called PyBullet"),
        )

        command = robot.wheel_command_from_twist(0.4, 0.8, timestamp_ns=123, dt=0.1)

        assert isinstance(command, WheelCommand)
        assert command.timestamp_ns == 123
        assert command.drive_wheel_speed_rad_s == pytest.approx((2.0, 6.0))
        assert command.steering_wheel_speed_rad_s == ()
        assert (robot.left_wheel_speed, robot.right_wheel_speed, robot.linear_velocity, robot.angular_velocity) == before
    finally:
        p.disconnect(client_id)


def test_task12_active_steering_wheel_command_from_twist_is_pure_and_limited(monkeypatch):
    client_id, robot = _load_robot("active_steering_4wd")
    try:
        before = (tuple(robot._steering_targets), robot.left_wheel_speed, robot.right_wheel_speed)
        monkeypatch.setattr(
            robot_module.p,
            "setJointMotorControl2",
            lambda *args, **kwargs: pytest.fail("pure conversion called PyBullet"),
        )

        command = robot.wheel_command_from_twist(0.4, 0.1, timestamp_ns=456, dt=0.1)

        assert command.timestamp_ns == 456
        assert command.drive_wheel_speed_rad_s == pytest.approx((4.0, 4.0, 4.0, 4.0))
        expected_angle = math.atan(robot.model_spec.axle_distance * 0.1 / 0.4)
        expected_rate = min(robot.MAX_STEERING_RATE, expected_angle / 0.1)
        assert command.steering_wheel_speed_rad_s == pytest.approx((expected_rate, expected_rate))
        assert (tuple(robot._steering_targets), robot.left_wheel_speed, robot.right_wheel_speed) == before
    finally:
        p.disconnect(client_id)


def test_active_steering_integrates_front_steering_speed_and_limits_angle():
    client_id, robot = _load_robot("active_steering_4wd")
    try:
        assert isinstance(robot, ActiveSteeringRobot)
        robot.command_wheel_speeds((4.0, 4.0, 4.0, 4.0), (1.0, -1.0), dt=0.2)
        _step(client_id)
        assert robot.read_drive_wheel_speeds() == pytest.approx((4.0, 4.0, 4.0, 4.0), abs=0.05)
        assert robot.read_steering_wheel_angles() == pytest.approx((0.2, -0.2), abs=0.03)

        robot.command_wheel_speeds((0.0, 0.0, 0.0, 0.0), (100.0, -100.0), dt=10.0)
        _step(client_id)
        left_angle, right_angle = robot.read_steering_wheel_angles()
        assert left_angle == pytest.approx(robot.max_steering_angle, abs=0.03)
        assert right_angle == pytest.approx(-robot.max_steering_angle, abs=0.03)
    finally:
        p.disconnect(client_id)


def test_active_steering_twist_drives_all_four_wheels_and_turns_front_wheels():
    client_id, robot = _load_robot("active_steering_4wd")
    try:
        assert isinstance(robot, ActiveSteeringRobot)
        robot.command_twist(0.4, 0.6, dt=0.1)
        _step(client_id)
        assert robot.read_drive_wheel_speeds() == pytest.approx((4.0, 4.0, 4.0, 4.0), abs=0.2)
        left_angle, right_angle = robot.read_steering_wheel_angles()
        assert left_angle > 0.0
        assert right_angle > 0.0
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("invalid_speed", [math.nan, math.inf, -math.inf])
def test_differential_control_rejects_non_finite_commands(invalid_speed: float):
    client_id, robot = _load_robot("df_back")
    try:
        with pytest.raises(ValueError, match="finite"):
            robot.command_wheel_speeds((invalid_speed, 1.0))
        with pytest.raises(ValueError, match="finite"):
            robot.command_twist(0.2, invalid_speed)
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("invalid_speed", [math.nan, math.inf, -math.inf])
def test_active_steering_control_rejects_non_finite_commands(invalid_speed: float):
    client_id, robot = _load_robot("active_steering_4wd")
    try:
        with pytest.raises(ValueError, match="finite"):
            robot.command_wheel_speeds((1.0, 1.0, invalid_speed, 1.0), (0.1, 0.1), dt=0.1)
        with pytest.raises(ValueError, match="finite"):
            robot.command_wheel_speeds((1.0, 1.0, 1.0, 1.0), (0.1, invalid_speed), dt=0.1)
        with pytest.raises(ValueError, match="finite"):
            robot.command_twist(invalid_speed, 0.2, dt=0.1)
        with pytest.raises(ValueError, match="finite"):
            robot.command_twist(0.2, 0.1, dt=invalid_speed)
    finally:
        p.disconnect(client_id)


def test_active_steering_physics_telemetry_contains_four_wheels_and_two_steering_angles():
    """主动转向车运行遥测必须暴露 4 个真实轮速和 2 个真实前轮转角。"""
    client_id, robot = _load_robot("active_steering_4wd")
    try:
        robot.command_wheel_speeds((3.0, 4.0, 5.0, 6.0), (0.5, 0.5), dt=0.2)
        _step(client_id)
        state = robot.read_physics_state(
            t=0.25,
            command_linear_velocity=0.4,
            command_angular_velocity=0.5,
            ground_lateral_friction=1.0,
            drive_lateral_friction=1.0,
            robot_model="active_steering_4wd",
            terrain_type="flat",
        )

        assert state.front_left_actual_drive_speed == pytest.approx(3.0, abs=0.1)
        assert state.front_right_actual_drive_speed == pytest.approx(4.0, abs=0.1)
        assert state.rear_left_actual_drive_speed == pytest.approx(5.0, abs=0.1)
        assert state.rear_right_actual_drive_speed == pytest.approx(6.0, abs=0.1)
        assert state.front_left_actual_steering_angle == pytest.approx(0.1, abs=0.03)
        assert state.front_right_actual_steering_angle == pytest.approx(0.1, abs=0.03)
    finally:
        p.disconnect(client_id)


def test_stage1_robot_bodies_share_geometry_and_total_mass_baseline():
    """四种车型使用同一车体尺寸和近似相同总质量，便于公平比较驱动布局。"""
    expected_body_size = (0.72, 0.44, 0.14)
    total_masses = []
    client_id = p.connect(p.DIRECT)
    try:
        for model_name in robot_model_names():
            spec = get_robot_model(model_name)
            root = ET.parse(spec.urdf_path).getroot()
            base_collision = root.find("./link[@name='base_link']/collision/geometry/box")
            assert base_collision is not None
            assert tuple(float(value) for value in base_collision.attrib["size"].split()) == pytest.approx(
                expected_body_size
            )

            robot = create_robot(client_id, model_name)
            total_mass = sum(
                float(p.getDynamicsInfo(robot.robot_id, link_index, physicsClientId=client_id)[0])
                for link_index in range(-1, p.getNumJoints(robot.robot_id, physicsClientId=client_id))
            )
            total_masses.append(total_mass)
            p.removeBody(robot.robot_id, physicsClientId=client_id)
    finally:
        p.disconnect(client_id)

    assert max(total_masses) - min(total_masses) < 0.05
