# 手动演示测试：保护 GUI 手动模式的退出时长策略。
import pybullet as p
import pytest

from slope_sim.config import ExperimentConfig
from slope_sim.dashboard import DashboardCommand
from slope_sim.manual_control import ManualCommand
from slope_sim.manual_demo import limit_manual_command_step, load_manual_robot, manual_step_limit, merge_manual_commands, reload_manual_robot
from slope_sim.scene import create_slope_scene
from slope_sim.simulation import _probe_terrain_for_robot, _read_lidar_for_robot


def test_manual_step_limit_is_unbounded_without_explicit_duration():
    assert manual_step_limit(duration_limit_sec=None, time_step=1.0 / 240.0) is None


def test_manual_step_limit_uses_explicit_duration_when_given():
    assert manual_step_limit(duration_limit_sec=1.0, time_step=0.25) == 4
    assert manual_step_limit(duration_limit_sec=0.01, time_step=0.25) == 1


def test_manual_step_limit_rejects_invalid_duration_or_time_step():
    with pytest.raises(ValueError):
        manual_step_limit(duration_limit_sec=0.0, time_step=0.25)
    with pytest.raises(ValueError):
        manual_step_limit(duration_limit_sec=1.0, time_step=0.0)


def test_limit_manual_command_step_slews_drive_commands():
    previous = DashboardCommand(1.0, -0.8, requested_robot_model="tracked_proxy")
    target = DashboardCommand(0.0, 0.0, requested_robot_model="tracked_proxy")

    limited = limit_manual_command_step(
        previous,
        target,
        dt=0.1,
        linear_acceleration_limit=2.0,
        angular_acceleration_limit=4.0,
    )

    assert limited.linear_velocity == pytest.approx(0.8)
    assert limited.angular_velocity == pytest.approx(-0.4)
    assert limited.requested_robot_model == "tracked_proxy"


def test_limit_manual_command_step_bypasses_slew_for_reset_or_exit():
    previous = DashboardCommand(1.0, 0.8)
    reset_target = DashboardCommand(0.0, 0.0, reset_requested=True)
    exit_target = DashboardCommand(0.0, 0.0, should_exit=True)

    reset = limit_manual_command_step(previous, reset_target, dt=0.1, linear_acceleration_limit=2.0, angular_acceleration_limit=4.0)
    exiting = limit_manual_command_step(previous, exit_target, dt=0.1, linear_acceleration_limit=2.0, angular_acceleration_limit=4.0)

    assert reset.linear_velocity == 0.0
    assert reset.angular_velocity == 0.0
    assert reset.reset_requested is True
    assert exiting.linear_velocity == 0.0
    assert exiting.angular_velocity == 0.0
    assert exiting.should_exit is True


def test_manual_robot_reload_switches_model_and_default_radius():
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(
            mode="gui",
            terrain_model="dam_slope",
            gui_model_switch_enabled=True,
        )
        scene = create_slope_scene(
            client_id,
            slope_deg=config.slope_deg,
            time_step=config.time_step,
            terrain_model=config.terrain_model,
        )

        active = load_manual_robot(client_id, config, scene, robot_model="diff_drive")
        switched = reload_manual_robot(client_id, active, config, scene, robot_model="tracked_proxy")

        assert active.robot_model == "diff_drive"
        assert active.wheel_radius == pytest.approx(0.10)
        assert switched.robot_model == "tracked_proxy"
        assert switched.wheel_radius == pytest.approx(0.08)
        assert switched.robot.uses_tracked_proxy is True
    finally:
        p.disconnect(client_id)


def test_merge_manual_commands_uses_pybullet_keyboard_when_dashboard_is_idle():
    dashboard = DashboardCommand(0.0, 0.0, requested_robot_model="tracked_proxy")
    keyboard = ManualCommand(0.4, 0.0)

    merged = merge_manual_commands(dashboard, keyboard)

    assert merged.linear_velocity == 0.4
    assert merged.angular_velocity == 0.0
    assert merged.requested_robot_model == "tracked_proxy"


def test_manual_dashboard_command_moves_tracked_proxy_on_dam_slope():
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(
            mode="gui",
            robot_model="tracked_proxy",
            drive_model="physics",
            terrain_model="dam_slope",
            slope_deg=10.0,
            wheel_radius=0.08,
            target_linear_velocity=0.25,
            gui_model_switch_enabled=True,
            lidar_enabled=True,
            lidar_ray_count=9,
        )
        scene = create_slope_scene(
            client_id,
            slope_deg=config.slope_deg,
            time_step=config.time_step,
            ground_lateral_friction=config.ground_lateral_friction,
            ground_rolling_friction=config.ground_rolling_friction,
            ground_spinning_friction=config.ground_spinning_friction,
            terrain_model=config.terrain_model,
            dam_toe_length=config.dam_toe_length,
            dam_slope_length=config.dam_slope_length,
            dam_crest_length=config.dam_crest_length,
            dam_exit_length=config.dam_exit_length,
            dam_width=config.dam_width,
            dam_wall_height=config.dam_wall_height,
            terrain_guard_enabled=config.terrain_guard_enabled,
        )
        active = load_manual_robot(client_id, config, scene)
        robot = active.robot

        states = []
        for step in range(180):
            # 等价于 dashboard 上箭头持续输出的手动命令，直接覆盖 UI 到物理模型的关键链路。
            robot.command_twist(config.target_linear_velocity, 0.0)
            p.stepSimulation(physicsClientId=client_id)
            states.append(
                robot.read_physics_state(
                    t=step * config.time_step,
                    command_linear_velocity=config.target_linear_velocity,
                    command_angular_velocity=0.0,
                    ground_lateral_friction=config.ground_lateral_friction,
                    drive_lateral_friction=config.drive_lateral_friction,
                    ground_rolling_friction=config.ground_rolling_friction,
                    ground_spinning_friction=config.ground_spinning_friction,
                    support_lateral_friction=config.support_lateral_friction,
                    robot_model=active.robot_model,
                    terrain_type=scene.terrain_type,
                    terrain_probe=_probe_terrain_for_robot(client_id, robot, scene),
                    lidar_summary=_read_lidar_for_robot(client_id, robot, config),
                )
            )

        assert states[-1].x - states[0].x > 0.10
        assert sum(state.body_forward_speed for state in states[-60:]) / 60.0 > 0.18
        assert not any(state.out_of_bounds for state in states)
    finally:
        p.disconnect(client_id)
