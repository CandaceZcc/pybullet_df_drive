# 履带驱动测试：保护 tracked_proxy 的同侧滚轮联动和有效半径换算。
import pybullet as p
import pytest

from slope_sim.robot import DifferentialDriveRobot, drive_targets_from_track_surface_speed


def test_drive_targets_keep_track_surface_speed_consistent():
    targets = drive_targets_from_track_surface_speed(surface_speed=0.56, joint_radii=[0.08, 0.07])

    assert targets == pytest.approx([7.0, 8.0])
    assert targets[0] * 0.08 == pytest.approx(targets[1] * 0.07)


def test_tracked_proxy_center_only_mode_keeps_tracked_friction_metadata():
    client_id = p.connect(p.DIRECT)
    try:
        robot = DifferentialDriveRobot(
            client_id=client_id,
            urdf_path="urdf/tracked_proxy.urdf",
            wheel_base=0.5,
            wheel_radius=0.08,
            drive_motor_force=2.5,
            track_anisotropic_friction=(2.0, 0.2, 0.05),
            track_drive_mode="center_only",
        )

        assert robot.uses_tracked_proxy is True
        assert robot.drive_motor_force == 2.5
        assert robot.track_anisotropic_friction == (2.0, 0.2, 0.05)
        assert len(robot.left_drive_joints) == 1
        assert len(robot.right_drive_joints) == 1
    finally:
        p.disconnect(client_id)
