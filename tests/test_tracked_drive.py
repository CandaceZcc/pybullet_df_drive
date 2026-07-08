# 履带驱动测试：保护 tracked_proxy 的同侧滚轮联动和有效半径换算。
import pytest

from slope_sim.robot import drive_targets_from_track_surface_speed


def test_drive_targets_keep_track_surface_speed_consistent():
    targets = drive_targets_from_track_surface_speed(surface_speed=0.56, joint_radii=[0.08, 0.07])

    assert targets == pytest.approx([7.0, 8.0])
    assert targets[0] * 0.08 == pytest.approx(targets[1] * 0.07)

