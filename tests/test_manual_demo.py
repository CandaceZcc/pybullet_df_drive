# 手动演示测试：保护 GUI 手动模式的退出时长策略。
from pathlib import Path
from types import SimpleNamespace

import pybullet as p
import pytest

import slope_sim.manual_demo as manual_demo_module
from slope_sim.config import ExperimentConfig
from slope_sim.dashboard import DashboardCommand, TerrainSelection
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
    previous = DashboardCommand(1.0, -0.8)
    target = DashboardCommand(0.0, 0.0)

    limited = limit_manual_command_step(
        previous,
        target,
        dt=0.1,
        linear_acceleration_limit=2.0,
        angular_acceleration_limit=4.0,
    )

    assert limited.linear_velocity == pytest.approx(0.8)
    assert limited.angular_velocity == pytest.approx(-0.4)
    assert limited.requested_robot_model is None


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
            terrain_model="flat",
        )
        scene = create_slope_scene(
            client_id,
            slope_deg=config.slope_deg,
            time_step=config.time_step,
            terrain_model=config.terrain_model,
        )

        active = load_manual_robot(client_id, config, scene, robot_model="df_back")
        switched = reload_manual_robot(client_id, active, config, scene, robot_model="active_steering_4wd")

        assert active.robot_model == "df_back"
        assert active.wheel_radius == pytest.approx(0.10)
        assert switched.robot_model == "active_steering_4wd"
        assert switched.wheel_radius == pytest.approx(0.10)
        assert len(switched.robot.drive_wheel_joint_indices) == 4
    finally:
        p.disconnect(client_id)


def test_apply_manual_switch_replaces_robot_without_rebuilding_terrain():
    """车型切换只替换车辆，当前地形刚体必须保持不变。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = manual_demo_module.load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        old_terrain_id = world.scene.body_id
        old_robot_id = world.active_robot.robot.robot_id

        result = manual_demo_module.apply_manual_switch_request(
            client_id,
            config,
            world,
            DashboardCommand(0.0, 0.0, requested_robot_model="active_steering_4wd"),
        )

        assert result.world.scene.body_id == old_terrain_id
        assert result.world.active_robot.robot_model == "active_steering_4wd"
        assert result.world.active_robot.robot.robot_id != old_robot_id
        assert old_robot_id not in [p.getBodyUniqueId(index, physicsClientId=client_id) for index in range(p.getNumBodies(client_id))]
        assert result.state_changed is True
        assert result.world_reset is False
    finally:
        p.disconnect(client_id)


def test_apply_manual_switch_rebuilds_terrain_and_keeps_robot_model():
    """场地切换会重建世界，但不能悄悄改变当前车型。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_mid", terrain_model="flat")
        world = manual_demo_module.load_manual_world(client_id, config, TerrainSelection("flat"), "df_mid")
        result = manual_demo_module.apply_manual_switch_request(
            client_id,
            config,
            world,
            DashboardCommand(0.0, 0.0, requested_terrain=TerrainSelection("slope", slope_deg=8.0)),
        )

        assert result.world.scene.terrain_type == "slope"
        assert result.world.scene.slope_deg == pytest.approx(8.0)
        assert result.world.active_robot.robot_model == "df_mid"
        assert result.state_changed is True
        assert result.world_reset is True
    finally:
        p.disconnect(client_id)


def test_failed_terrain_switch_rolls_back_to_previous_world(monkeypatch):
    """目标场地创建失败后必须恢复有效旧世界，而不是留下无地面状态。"""
    client_id = p.connect(p.DIRECT)
    original = manual_demo_module.create_slope_scene
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = manual_demo_module.load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")

        def fail_target(*args, **kwargs):
            if kwargs.get("terrain_model") == "golf_heightfield":
                raise RuntimeError("target terrain failed")
            return original(*args, **kwargs)

        monkeypatch.setattr(manual_demo_module, "create_slope_scene", fail_target)
        result = manual_demo_module.apply_manual_switch_request(
            client_id,
            config,
            world,
            DashboardCommand(
                0.0,
                0.0,
                requested_terrain=TerrainSelection("golf_heightfield", golf_seed=7),
            ),
        )

        assert result.world.terrain == TerrainSelection("flat")
        assert result.world.scene.terrain_type == "flat"
        assert result.world.active_robot.robot_model == "df_back"
        assert "target terrain failed" in result.error_message
        assert result.state_changed is True
        assert result.world_reset is True
    finally:
        p.disconnect(client_id)


def test_failed_robot_switch_keeps_current_robot(monkeypatch):
    """新车型加载失败时旧车仍然有效，不能先删旧车再尝试加载。"""
    client_id = p.connect(p.DIRECT)
    original = manual_demo_module.load_manual_robot
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = manual_demo_module.load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        old_robot_id = world.active_robot.robot.robot_id

        def fail_target(client_id, config, scene, robot_model=None):
            if robot_model == "df_mid":
                raise RuntimeError("target robot failed")
            return original(client_id, config, scene, robot_model)

        monkeypatch.setattr(manual_demo_module, "load_manual_robot", fail_target)
        result = manual_demo_module.apply_manual_switch_request(
            client_id,
            config,
            world,
            DashboardCommand(0.0, 0.0, requested_robot_model="df_mid"),
        )

        assert result.world.active_robot.robot.robot_id == old_robot_id
        assert result.world.active_robot.robot_model == "df_back"
        assert "target robot failed" in result.error_message
        assert result.state_changed is False
    finally:
        p.disconnect(client_id)


def test_merge_manual_commands_uses_pybullet_keyboard_when_dashboard_is_idle():
    dashboard = DashboardCommand(0.0, 0.0)
    keyboard = ManualCommand(0.4, 0.0)

    merged = merge_manual_commands(dashboard, keyboard)

    assert merged.linear_velocity == 0.4
    assert merged.angular_velocity == 0.0
    assert merged.requested_robot_model is None


def test_merge_manual_commands_keeps_camera_state_when_keyboard_takes_control():
    dashboard = DashboardCommand(0.0, 0.0, camera_follow_enabled=True, camera_follow_view="side")

    merged = merge_manual_commands(dashboard, ManualCommand(0.4, 0.0))

    assert merged.linear_velocity == 0.4
    assert merged.camera_follow_enabled is True
    assert merged.camera_follow_view == "side"


def test_merge_manual_commands_preserves_terrain_switch_and_blocks_keyboard_motion():
    """场地切换帧不能被 PyBullet 窗口键盘重新注入驾驶速度。"""
    request = TerrainSelection("slope", slope_deg=6.0)
    merged = merge_manual_commands(
        DashboardCommand(0.0, 0.0, requested_terrain=request),
        ManualCommand(0.4, 0.0),
    )

    assert merged.requested_terrain == request
    assert merged.linear_velocity == 0.0
    assert merged.angular_velocity == 0.0


def test_limit_manual_command_step_bypasses_slew_for_scene_switch():
    """场景切换必须立即清零，不允许加速度限制器残留上一帧速度。"""
    request = TerrainSelection("golf_heightfield", golf_seed=9)
    limited = limit_manual_command_step(
        DashboardCommand(1.0, 0.8),
        DashboardCommand(
            0.0,
            0.0,
            requested_terrain=request,
            camera_follow_enabled=True,
            camera_follow_view="front",
        ),
        dt=0.1,
        linear_acceleration_limit=2.0,
        angular_acceleration_limit=4.0,
    )

    assert limited.linear_velocity == 0.0
    assert limited.angular_velocity == 0.0
    assert limited.requested_terrain == request
    assert limited.camera_follow_enabled is True
    assert limited.camera_follow_view == "front"


def test_manual_demo_dashboard_fallback_keeps_configured_camera_state(monkeypatch):
    """侧窗不可用时，PyBullet 回退循环仍沿用配置的相机跟随状态。"""
    client_id = 17
    robot_id = 23
    camera_calls = []
    state = SimpleNamespace(x=0.0, y=0.0, out_of_bounds=False)
    robot = SimpleNamespace(
        robot_id=robot_id,
        command_twist=lambda *_args, **_kwargs: None,
        read_physics_state=lambda **_kwargs: state,
    )
    world = SimpleNamespace(
        scene=SimpleNamespace(terrain_type="flat", slope_deg=0.0),
        active_robot=SimpleNamespace(robot=robot, robot_model="df_back"),
        terrain=TerrainSelection("flat"),
    )

    class FakeLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def record(self, *_args, **_kwargs):
            pass

        def close(self):
            return Path("manual_fallback.csv")

    class FakeDiagnosticSummary:
        def to_dict(self):
            return {}

    monkeypatch.setattr(manual_demo_module.p, "connect", lambda _mode: client_id)
    monkeypatch.setattr(manual_demo_module.p, "addUserDebugParameter", lambda *_args: 1)
    monkeypatch.setattr(manual_demo_module.p, "readUserDebugParameter", lambda _slider: 0.0)
    monkeypatch.setattr(manual_demo_module.p, "getKeyboardEvents", lambda: {})
    monkeypatch.setattr(manual_demo_module.p, "stepSimulation", lambda **_kwargs: None)
    monkeypatch.setattr(manual_demo_module.p, "disconnect", lambda _client_id: None)
    monkeypatch.setattr(manual_demo_module, "TelemetryDashboard", lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("offscreen")))
    monkeypatch.setattr(manual_demo_module, "load_manual_world", lambda *_args: world)
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeLogger)
    monkeypatch.setattr(manual_demo_module, "command_from_keyboard", lambda *_args: ManualCommand(0.0, 0.0))
    monkeypatch.setattr(manual_demo_module, "_read_lidar_for_robot", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "_probe_terrain_for_robot", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "update_follow_camera", lambda *args: camera_calls.append(args))
    monkeypatch.setattr(manual_demo_module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(manual_demo_module.pd, "read_csv", lambda _path: object())
    monkeypatch.setattr(manual_demo_module, "compute_diagnostic_summary", lambda _frame: FakeDiagnosticSummary())
    monkeypatch.setattr(manual_demo_module, "write_diagnostic_summary", lambda *_args: Path("diagnostics.json"))
    monkeypatch.setattr(manual_demo_module, "compute_tracking_metrics", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(manual_demo_module, "plot_trajectory", lambda *_args, **_kwargs: Path("trajectory.png"))
    monkeypatch.setattr(manual_demo_module, "plot_feedback_figures", lambda *_args, **_kwargs: [])

    manual_demo_module.run_manual_demo(
        ExperimentConfig(
            mode="gui",
            time_step=0.01,
            dashboard_enabled=True,
            camera_follow_enabled=True,
            camera_follow_view="side",
        ),
        duration_limit_sec=0.01,
    )

    assert camera_calls == [(client_id, robot_id, 6.0, -35.0, 45.0, "side")]


def test_terrain_switch_takes_priority_over_model_switch_and_reset():
    """同一帧最多重建一次，优先应用会重置整个世界的场地请求。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = manual_demo_module.load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        result = manual_demo_module.apply_manual_switch_request(
            client_id,
            config,
            world,
            DashboardCommand(
                0.4,
                0.8,
                requested_robot_model="df_mid",
                reset_requested=True,
                requested_terrain=TerrainSelection("slope", slope_deg=5.0),
            ),
        )

        assert result.world.scene.terrain_type == "slope"
        assert result.world.active_robot.robot_model == "df_back"
    finally:
        p.disconnect(client_id)


def test_process_manual_scene_action_stops_old_robot_before_switch_and_updates_dashboard(monkeypatch):
    """单帧协调必须先停旧车，再重建并同步相机与 Dashboard。"""
    client_id = p.connect(p.DIRECT)
    events = []
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = manual_demo_module.load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        original_command_twist = world.active_robot.robot.command_twist

        def record_command_twist(linear_velocity, angular_velocity, dt):
            events.append(("stop", linear_velocity, angular_velocity))
            return original_command_twist(linear_velocity, angular_velocity, dt=dt)

        monkeypatch.setattr(world.active_robot.robot, "command_twist", record_command_twist)

        expected_result = manual_demo_module.ManualSwitchResult(
            world=world,
            state_changed=True,
            world_reset=True,
            status_message="场地已切换",
        )

        def fake_apply(_client_id, _config, _world, _command):
            events.append(("apply",))
            return expected_result

        monkeypatch.setattr(manual_demo_module, "apply_manual_switch_request", fake_apply)
        monkeypatch.setattr(
            manual_demo_module,
            "configure_gui_visualizer",
            lambda *_args: events.append(("camera",)),
        )

        class DashboardProbe:
            def set_switch_busy(self, busy, message, **_kwargs):
                events.append(("busy", busy, message))

            def process_events(self):
                events.append(("events",))

            def sync_active_selection(self, robot_model, terrain):
                events.append(("sync", robot_model, terrain.terrain_model))

            def reset_feedback_history(self):
                events.append(("reset_history",))

            def show_switch_status(self, message, **_kwargs):
                events.append(("status", message))

        result = manual_demo_module.process_manual_scene_action(
            client_id,
            config,
            world,
            DashboardCommand(
                0.4,
                0.8,
                requested_terrain=TerrainSelection("slope", slope_deg=5.0),
            ),
            DashboardProbe(),
        )

        assert result is expected_result
        assert events.index(("stop", 0.0, 0.0)) < events.index(("apply",))
        assert ("busy", True, "应用中") in events
        assert ("camera",) in events
        assert ("sync", "df_back", "flat") in events
        assert ("reset_history",) in events
        assert ("status", "场地已切换") in events
    finally:
        p.disconnect(client_id)


def test_manual_dashboard_command_moves_mid_drive_on_slope():
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(
            mode="gui",
            robot_model="df_mid",
            drive_model="physics",
            terrain_model="slope",
            slope_deg=8.0,
            target_linear_velocity=0.25,
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
            golf_seed=config.golf_seed,
            golf_relief=config.golf_relief,
        )
        active = load_manual_robot(client_id, config, scene)
        robot = active.robot

        states = []
        for step in range(180):
            # 等价于 dashboard 上箭头持续输出的手动命令，直接覆盖 UI 到物理模型的关键链路。
            robot.command_twist(config.target_linear_velocity, 0.0, dt=config.time_step)
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
