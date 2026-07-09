# 二轮支撑测试：保护单前支撑轮的低摩擦设置，避免它参与牵引或卡住转向。
import pybullet as p
import pytest

from slope_sim.robot import DifferentialDriveRobot


def test_diff_drive_support_wheel_uses_separate_low_friction():
    client_id = p.connect(p.DIRECT)
    try:
        robot = DifferentialDriveRobot(
            client_id=client_id,
            urdf_path="urdf/diff_drive.urdf",
            wheel_base=0.5,
            wheel_radius=0.1,
            base_height=0.14,
        )

        robot.apply_drive_friction(lateral_friction=1.2, support_lateral_friction=0.03)

        left_friction = p.getDynamicsInfo(robot.robot_id, robot.left_joint, physicsClientId=client_id)[1]
        caster_friction = p.getDynamicsInfo(
            robot.robot_id,
            robot.joint_name_to_index["caster_joint"],
            physicsClientId=client_id,
        )[1]
        assert left_friction == pytest.approx(1.2)
        assert caster_friction == pytest.approx(0.03)
    finally:
        p.disconnect(client_id)


def test_diff_drive_uses_rear_drive_front_support_geometry():
    client_id = p.connect(p.DIRECT)
    try:
        robot = DifferentialDriveRobot(
            client_id=client_id,
            urdf_path="urdf/diff_drive.urdf",
            wheel_base=0.5,
            wheel_radius=0.1,
            base_height=0.14,
        )

        left_info = p.getJointInfo(robot.robot_id, robot.left_joint, physicsClientId=client_id)
        caster_info = p.getJointInfo(
            robot.robot_id,
            robot.joint_name_to_index["caster_joint"],
            physicsClientId=client_id,
        )

        assert left_info[14][0] == pytest.approx(-0.12)
        assert caster_info[14][0] == pytest.approx(0.30)
        assert caster_info[14][0] - left_info[14][0] >= 0.40
    finally:
        p.disconnect(client_id)
