# 差速支撑集成测试：保护前/中/后驱模型的球形支撑轮数量、位置和低摩擦设置。
import pybullet as p
import pytest

from slope_sim.model_registry import get_robot_model
from slope_sim.robot import create_robot


@pytest.mark.parametrize(
    ("model_name", "support_names", "drive_x"),
    [
        ("df_front", ("rear_support",), 0.22),
        ("df_mid", ("front_support", "rear_support"), 0.0),
        ("df_back", ("front_support",), -0.22),
    ],
)
def test_differential_support_wheels_use_separate_low_friction(model_name, support_names, drive_x):
    client_id = p.connect(p.DIRECT)
    try:
        robot = create_robot(client_id, model_name)
        robot.apply_drive_friction(lateral_friction=1.2, support_lateral_friction=0.03)

        assert get_robot_model(model_name).drive_center_x == pytest.approx(drive_x)
        assert len(robot.support_links) == len(support_names)
        left_friction = p.getDynamicsInfo(robot.robot_id, robot.left_joint, physicsClientId=client_id)[1]
        assert left_friction == pytest.approx(1.2)
        for support_name in support_names:
            support_joint = robot.joint_name_to_index[f"{support_name}_joint"]
            support_friction = p.getDynamicsInfo(robot.robot_id, support_joint, physicsClientId=client_id)[1]
            assert support_friction == pytest.approx(0.03)
    finally:
        p.disconnect(client_id)


def test_front_mid_back_drive_axles_are_physically_ordered():
    positions = {}
    client_id = p.connect(p.DIRECT)
    try:
        for model_name in ("df_front", "df_mid", "df_back"):
            robot = create_robot(client_id, model_name)
            info = p.getJointInfo(robot.robot_id, robot.left_joint, physicsClientId=client_id)
            positions[model_name] = float(info[14][0])
            p.removeBody(robot.robot_id, physicsClientId=client_id)
    finally:
        p.disconnect(client_id)

    assert positions["df_front"] > positions["df_mid"] > positions["df_back"]
