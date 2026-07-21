# 手动演示测试：保护 GUI 手动模式的退出时长策略和阶段二结构操作主循环。
import json
from pathlib import Path
from types import SimpleNamespace

import pybullet as p
import pytest

import slope_sim.manual_demo as manual_demo_module
from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import load_manual_robot, reload_manual_robot
from slope_sim.dashboard import DashboardCommand
from slope_sim.manual_control import ManualCommand
from slope_sim.manual_demo import limit_manual_command_step, manual_step_limit, merge_manual_commands
from slope_sim.obstacles import ObstacleOperationResult, ObstacleSnapshot
from slope_sim.runtime_actions import (
    AddObstaclesAction,
    ClearObstaclesAction,
    DeleteObstacleAction,
    ResetRobotAction,
    SwitchRobotAction,
    SwitchTerrainAction,
    TerrainSelection,
)
from slope_sim.obstacles import ObstacleGenerationRequest
from slope_sim.scene import create_slope_scene
from slope_sim.simulation import _probe_terrain_for_robot, _read_lidar_for_robot


class FakeObstacleEventLogger:
    """不关心事件日志的手动循环测试用空日志器，避免污染 results 目录。"""

    def __init__(self, *_args, **_kwargs):
        pass

    def record_event(self, **_kwargs):
        pass

    def close(self):
        return Path("manual_obstacles.jsonl")


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
    assert limited.structural_action is None


def test_limit_manual_command_step_bypasses_slew_for_reset_or_exit():
    previous = DashboardCommand(1.0, 0.8)
    reset_target = DashboardCommand(0.0, 0.0, structural_action=ResetRobotAction())
    exit_target = DashboardCommand(0.0, 0.0, should_exit=True)

    reset = limit_manual_command_step(previous, reset_target, dt=0.1, linear_acceleration_limit=2.0, angular_acceleration_limit=4.0)
    exiting = limit_manual_command_step(previous, exit_target, dt=0.1, linear_acceleration_limit=2.0, angular_acceleration_limit=4.0)

    assert reset.linear_velocity == 0.0
    assert reset.angular_velocity == 0.0
    assert reset.structural_action == ResetRobotAction()
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


def test_merge_manual_commands_uses_pybullet_keyboard_when_dashboard_is_idle():
    dashboard = DashboardCommand(0.0, 0.0)
    keyboard = ManualCommand(0.4, 0.0)

    merged = merge_manual_commands(dashboard, keyboard)

    assert merged.linear_velocity == 0.4
    assert merged.angular_velocity == 0.0
    assert merged.structural_action is None


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
        DashboardCommand(0.0, 0.0, structural_action=SwitchTerrainAction(request)),
        ManualCommand(0.4, 0.0),
    )

    assert merged.structural_action == SwitchTerrainAction(request)
    assert merged.linear_velocity == 0.0
    assert merged.angular_velocity == 0.0


@pytest.mark.parametrize(
    "action",
    [
        AddObstaclesAction(ObstacleGenerationRequest("static", 1, seed=4)),
        DeleteObstacleAction(1),
        ClearObstaclesAction(),
    ],
)
def test_merge_manual_commands_allows_obstacle_actions_during_drive(action):
    """障碍物增删不属于安全停车操作，不能吞掉当前驾驶命令。"""
    merged = merge_manual_commands(
        DashboardCommand(0.2, 0.1, structural_action=action),
        ManualCommand(0.4, 0.0),
    )

    assert merged.structural_action == action
    assert merged.linear_velocity == pytest.approx(0.2)
    assert merged.angular_velocity == pytest.approx(0.1)


def test_limit_manual_command_step_bypasses_slew_for_scene_switch():
    """场景切换必须立即清零，不允许加速度限制器残留上一帧速度。"""
    request = TerrainSelection("golf_heightfield", golf_seed=9)
    limited = limit_manual_command_step(
        DashboardCommand(1.0, 0.8),
        DashboardCommand(
            0.0,
            0.0,
            structural_action=SwitchTerrainAction(request),
            camera_follow_enabled=True,
            camera_follow_view="front",
        ),
        dt=0.1,
        linear_acceleration_limit=2.0,
        angular_acceleration_limit=4.0,
    )

    assert limited.linear_velocity == 0.0
    assert limited.angular_velocity == 0.0
    assert limited.structural_action == SwitchTerrainAction(request)
    assert limited.camera_follow_enabled is True
    assert limited.camera_follow_view == "front"


@pytest.mark.parametrize(
    "action",
    [
        AddObstaclesAction(ObstacleGenerationRequest("static", 1, seed=5)),
        DeleteObstacleAction(1),
        ClearObstaclesAction(),
    ],
)
def test_limit_manual_command_step_keeps_slew_for_obstacle_actions(action):
    """障碍物操作可以和驾驶命令同帧排队，速度仍按常规限幅。"""
    limited = limit_manual_command_step(
        DashboardCommand(0.0, 0.0),
        DashboardCommand(0.6, 0.4, structural_action=action),
        dt=0.1,
        linear_acceleration_limit=2.0,
        angular_acceleration_limit=4.0,
    )

    assert limited.linear_velocity == pytest.approx(0.2)
    assert limited.angular_velocity == pytest.approx(0.4)
    assert limited.structural_action == action


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
    monkeypatch.setattr(
        manual_demo_module,
        "create_obstacle_manager",
        lambda *_args, **_kwargs: SimpleNamespace(update_moving=lambda _dt: None),
    )
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeLogger)
    monkeypatch.setattr(manual_demo_module, "ObstacleEventLogger", FakeObstacleEventLogger)
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


def test_manual_demo_keeps_dashboard_busy_while_obstacle_operation_is_pending(monkeypatch):
    """跨帧障碍物事务未完成时，Dashboard 应保持忙碌且不显示错误。"""
    client_id = 18
    robot_id = 24
    events = []
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
    pending_result = SimpleNamespace(
        world=world,
        state_changed=False,
        world_reset=False,
        status_message="pending",
        error_message=None,
        obstacle_result=ObstacleOperationResult(done=False, succeeded=False, operation="add", message="pending"),
    )

    class FakeDashboard:
        def __init__(self, *_args, **_kwargs):
            self.linear_spin = SimpleNamespace(value=lambda: 0.4)
            self.angular_spin = SimpleNamespace(value=lambda: 0.8)
            self._calls = 0

        def process_events(self):
            pass

        def current_command(self):
            self._calls += 1
            return DashboardCommand(0.0, 0.0, should_exit=self._calls > 1)

        def set_switch_busy(self, busy, message, **_kwargs):
            events.append(("busy", busy, message))

        def set_structure_busy(self, busy, message, **_kwargs):
            events.append(("structure", busy, message))

        def sync_active_selection(self, *_args):
            events.append(("sync",))

        def reset_feedback_history(self):
            events.append(("reset_history",))

        def show_switch_status(self, message, **kwargs):
            events.append(("status", message, kwargs.get("is_error")))

        def update(self, _state):
            pass

        def update_obstacle_snapshots(self, _snapshots):
            pass

        def close(self):
            pass

    class FakeManager:
        def snapshot(self, *, include_body_id=False):
            assert include_body_id is False
            return ()

    class FakeCoordinator:
        def __init__(self, *_args, **_kwargs):
            self.world = world
            self.obstacle_manager = FakeManager()

        def enqueue(self, _action):
            pass

        def step(self, _dt):
            return pending_result

    class FakeLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def record(self, *_args, **_kwargs):
            pass

        def close(self):
            return Path("manual_pending.csv")

    class FakeDiagnosticSummary:
        def to_dict(self):
            return {}

    monkeypatch.setattr(manual_demo_module.p, "connect", lambda _mode: client_id)
    monkeypatch.setattr(manual_demo_module.p, "disconnect", lambda _client_id: None)
    monkeypatch.setattr(manual_demo_module.p, "getKeyboardEvents", lambda: {})
    monkeypatch.setattr(manual_demo_module, "TelemetryDashboard", FakeDashboard)
    monkeypatch.setattr(manual_demo_module, "SimulationCoordinator", FakeCoordinator)
    monkeypatch.setattr(manual_demo_module, "load_manual_world", lambda *_args: world)
    monkeypatch.setattr(manual_demo_module, "create_obstacle_manager", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeLogger)
    monkeypatch.setattr(manual_demo_module, "ObstacleEventLogger", FakeObstacleEventLogger)
    monkeypatch.setattr(manual_demo_module, "command_from_keyboard", lambda *_args: ManualCommand(0.0, 0.0))
    monkeypatch.setattr(manual_demo_module, "update_follow_camera", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(manual_demo_module.pd, "read_csv", lambda _path: object())
    monkeypatch.setattr(manual_demo_module, "compute_diagnostic_summary", lambda _frame: FakeDiagnosticSummary())
    monkeypatch.setattr(manual_demo_module, "write_diagnostic_summary", lambda *_args: Path("diagnostics.json"))
    monkeypatch.setattr(manual_demo_module, "compute_tracking_metrics", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(manual_demo_module, "plot_trajectory", lambda *_args, **_kwargs: Path("trajectory.png"))
    monkeypatch.setattr(manual_demo_module, "plot_feedback_figures", lambda *_args, **_kwargs: [])
    monkeypatch.setattr(manual_demo_module, "_read_lidar_for_robot", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "_probe_terrain_for_robot", lambda *_args: None)

    manual_demo_module.run_manual_demo(
        ExperimentConfig(mode="gui", time_step=0.01, dashboard_enabled=True),
        duration_limit_sec=0.02,
    )

    assert ("structure", True, "pending") in events
    assert not any(event[0] == "status" and event[2] for event in events)


def test_manual_demo_keeps_dashboard_busy_after_immediate_action_when_queue_remains(monkeypatch):
    """即时结构动作完成后，如果 FIFO 里还有动作，Dashboard 不能提前解锁。"""
    client_id = 22
    robot_id = 28
    events = []
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
    result = SimpleNamespace(
        world=world,
        state_changed=True,
        world_reset=False,
        status_message="车型已切换为 df_mid",
        error_message=None,
        obstacle_result=None,
    )

    class FakeDashboard:
        def __init__(self, *_args, **_kwargs):
            self.linear_spin = SimpleNamespace(value=lambda: 0.4)
            self.angular_spin = SimpleNamespace(value=lambda: 0.8)
            self._calls = 0

        def process_events(self):
            pass

        def current_command(self):
            self._calls += 1
            return DashboardCommand(0.0, 0.0, should_exit=self._calls > 1)

        def set_structure_busy(self, busy, message, **_kwargs):
            events.append(("structure", busy, message))

        def sync_active_selection(self, *_args):
            pass

        def reset_feedback_history(self):
            pass

        def show_switch_status(self, message, **kwargs):
            events.append(("status", message, kwargs.get("is_error")))

        def update(self, _state):
            pass

        def update_obstacle_snapshots(self, _snapshots):
            pass

        def close(self):
            pass

    class FakeManager:
        def snapshot(self, *, include_body_id=False):
            assert include_body_id is False
            return ()

    class FakeCoordinator:
        def __init__(self, *_args, **_kwargs):
            self.world = world
            self.obstacle_manager = FakeManager()
            self.has_pending_action = True

        def enqueue(self, _action):
            pass

        def step(self, _dt):
            return result

    class FakeLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def record(self, *_args, **_kwargs):
            pass

        def close(self):
            return Path("manual_queue_busy.csv")

    class FakeDiagnosticSummary:
        def to_dict(self):
            return {}

    monkeypatch.setattr(manual_demo_module.p, "connect", lambda _mode: client_id)
    monkeypatch.setattr(manual_demo_module.p, "disconnect", lambda _client_id: None)
    monkeypatch.setattr(manual_demo_module.p, "getKeyboardEvents", lambda: {})
    monkeypatch.setattr(manual_demo_module, "TelemetryDashboard", FakeDashboard)
    monkeypatch.setattr(manual_demo_module, "SimulationCoordinator", FakeCoordinator)
    monkeypatch.setattr(manual_demo_module, "load_manual_world", lambda *_args: world)
    monkeypatch.setattr(manual_demo_module, "create_obstacle_manager", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeLogger)
    monkeypatch.setattr(manual_demo_module, "ObstacleEventLogger", FakeObstacleEventLogger)
    monkeypatch.setattr(manual_demo_module, "command_from_keyboard", lambda *_args: ManualCommand(0.0, 0.0))
    monkeypatch.setattr(manual_demo_module, "update_follow_camera", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(manual_demo_module.pd, "read_csv", lambda _path: object())
    monkeypatch.setattr(manual_demo_module, "compute_diagnostic_summary", lambda _frame: FakeDiagnosticSummary())
    monkeypatch.setattr(manual_demo_module, "write_diagnostic_summary", lambda *_args: Path("diagnostics.json"))
    monkeypatch.setattr(manual_demo_module, "compute_tracking_metrics", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(manual_demo_module, "plot_trajectory", lambda *_args, **_kwargs: Path("trajectory.png"))
    monkeypatch.setattr(manual_demo_module, "plot_feedback_figures", lambda *_args, **_kwargs: [])
    monkeypatch.setattr(manual_demo_module, "_read_lidar_for_robot", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "_probe_terrain_for_robot", lambda *_args: None)

    manual_demo_module.run_manual_demo(
        ExperimentConfig(mode="gui", time_step=0.01, dashboard_enabled=True),
        duration_limit_sec=0.02,
    )

    assert ("structure", True, "车型已切换为 df_mid") in events
    assert not any(event[0] == "status" for event in events)


def test_manual_demo_refreshes_dashboard_obstacle_snapshots(monkeypatch):
    """手动循环应把管理器快照同步到 Dashboard 障碍物表格。"""
    client_id = 19
    robot_id = 25
    snapshots = (
        ObstacleSnapshot(5, None, "static", "box", (1.0, 2.0, 0.3), (0.0, 0.0, 0.0, 1.0)),
    )
    refreshed = []
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

    class FakeDashboard:
        def __init__(self, *_args, **_kwargs):
            self.linear_spin = SimpleNamespace(value=lambda: 0.4)
            self.angular_spin = SimpleNamespace(value=lambda: 0.8)
            self._calls = 0

        def process_events(self):
            pass

        def current_command(self):
            self._calls += 1
            return DashboardCommand(0.0, 0.0, should_exit=self._calls > 1)

        def update_obstacle_snapshots(self, value):
            refreshed.append(value)

        def update(self, _state):
            pass

        def close(self):
            pass

    class FakeManager:
        def snapshot(self, *, include_body_id=False):
            assert include_body_id is False
            return snapshots

    class FakeCoordinator:
        def __init__(self, *_args, **_kwargs):
            self.world = world
            self.obstacle_manager = FakeManager()

        def enqueue(self, _action):
            pass

        def step(self, _dt):
            return None

    class FakeLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def record(self, *_args, **_kwargs):
            pass

        def close(self):
            return Path("manual_obstacles.csv")

    class FakeDiagnosticSummary:
        def to_dict(self):
            return {}

    monkeypatch.setattr(manual_demo_module.p, "connect", lambda _mode: client_id)
    monkeypatch.setattr(manual_demo_module.p, "disconnect", lambda _client_id: None)
    monkeypatch.setattr(manual_demo_module.p, "getKeyboardEvents", lambda: {})
    monkeypatch.setattr(manual_demo_module, "TelemetryDashboard", FakeDashboard)
    monkeypatch.setattr(manual_demo_module, "SimulationCoordinator", FakeCoordinator)
    monkeypatch.setattr(manual_demo_module, "load_manual_world", lambda *_args: world)
    monkeypatch.setattr(manual_demo_module, "create_obstacle_manager", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeLogger)
    monkeypatch.setattr(manual_demo_module, "ObstacleEventLogger", FakeObstacleEventLogger)
    monkeypatch.setattr(manual_demo_module, "command_from_keyboard", lambda *_args: ManualCommand(0.0, 0.0))
    monkeypatch.setattr(manual_demo_module, "update_follow_camera", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(manual_demo_module.pd, "read_csv", lambda _path: object())
    monkeypatch.setattr(manual_demo_module, "compute_diagnostic_summary", lambda _frame: FakeDiagnosticSummary())
    monkeypatch.setattr(manual_demo_module, "write_diagnostic_summary", lambda *_args: Path("diagnostics.json"))
    monkeypatch.setattr(manual_demo_module, "compute_tracking_metrics", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(manual_demo_module, "plot_trajectory", lambda *_args, **_kwargs: Path("trajectory.png"))
    monkeypatch.setattr(manual_demo_module, "plot_feedback_figures", lambda *_args, **_kwargs: [])
    monkeypatch.setattr(manual_demo_module, "_read_lidar_for_robot", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "_probe_terrain_for_robot", lambda *_args: None)

    manual_demo_module.run_manual_demo(
        ExperimentConfig(mode="gui", time_step=0.01, dashboard_enabled=True),
        duration_limit_sec=0.02,
    )

    assert refreshed == [snapshots]


def test_manual_structural_event_logger_records_delete_clear_and_terrain_rollback():
    """事件转换层要覆盖删除、清空、场地目标失败和回滚结果。"""
    rows = []
    terrain = TerrainSelection("flat")
    world = SimpleNamespace(
        active_robot=SimpleNamespace(robot_model="df_back"),
        terrain=terrain,
    )

    class CapturingEventLogger:
        def record_event(self, **kwargs):
            rows.append(kwargs)

    manual_demo_module._record_manual_structural_event(
        CapturingEventLogger(),
        sim_time=0.5,
        action=DeleteObstacleAction(7),
        result=SimpleNamespace(
            world=world,
            error_message=None,
            obstacle_result=ObstacleOperationResult(done=True, succeeded=True, operation="delete", deleted_count=1),
        ),
    )
    manual_demo_module._record_manual_structural_event(
        CapturingEventLogger(),
        sim_time=0.6,
        action=ClearObstaclesAction(),
        result=SimpleNamespace(
            world=world,
            error_message=None,
            obstacle_result=ObstacleOperationResult(done=True, succeeded=True, operation="clear", deleted_count=3),
        ),
    )
    manual_demo_module._record_manual_structural_event(
        CapturingEventLogger(),
        sim_time=0.7,
        action=SwitchTerrainAction(TerrainSelection("slope", slope_deg=6.0)),
        result=SimpleNamespace(
            world=world,
            error_message="target exploded",
            obstacle_result=None,
        ),
    )

    assert [row["event_type"] for row in rows] == ["delete", "clear", "terrain_rebuild", "rollback"]
    assert rows[0]["logical_id"] == 7
    assert rows[0]["request_params"] == {"logical_id": 7}
    assert rows[1]["request_params"] == {}
    assert rows[2]["success"] is False
    assert rows[2]["error_reason"] == "target exploded"
    assert rows[3]["success"] is True
    assert rows[3]["request_params"] == {
        "terrain_model": "flat",
        "slope_deg": 0.0,
        "golf_seed": 0,
        "golf_relief": "medium",
    }
    assert rows[3]["error_reason"] is None
    assert rows[3]["terrain"] == terrain


@pytest.mark.parametrize("action", [SwitchRobotAction("df_mid"), ResetRobotAction()])
def test_manual_structural_event_logger_skips_non_obstacle_scene_events(action):
    """障碍物事件日志只记录障碍和场地重建，不混入车型切换或复位。"""
    rows = []
    world = SimpleNamespace(
        active_robot=SimpleNamespace(robot_model="df_back"),
        terrain=TerrainSelection("flat"),
    )

    class CapturingEventLogger:
        def record_event(self, **kwargs):
            rows.append(kwargs)

    manual_demo_module._record_manual_structural_event(
        CapturingEventLogger(),
        sim_time=0.8,
        action=action,
        result=SimpleNamespace(world=world, error_message=None, obstacle_result=None),
    )

    assert rows == []


def test_manual_demo_logs_obstacle_failure_and_keeps_drive_active(monkeypatch, tmp_path: Path):
    """添加失败也要写事件日志；障碍物操作期间驾驶命令仍进入机器人控制。"""
    client_id = 20
    robot_id = 26
    csv_path = tmp_path / "manual_events.csv"
    command_calls = []
    request = ObstacleGenerationRequest("mixed", 3, shape="box", seed=42)
    state = SimpleNamespace(x=0.0, y=0.0, out_of_bounds=False)
    robot = SimpleNamespace(
        robot_id=robot_id,
        command_twist=lambda linear, angular, **_kwargs: command_calls.append((linear, angular)),
        read_physics_state=lambda **_kwargs: state,
    )
    world = SimpleNamespace(
        scene=SimpleNamespace(terrain_type="flat", slope_deg=0.0),
        active_robot=SimpleNamespace(robot=robot, robot_model="df_back"),
        terrain=TerrainSelection("flat"),
    )
    failed_result = SimpleNamespace(
        world=world,
        state_changed=False,
        world_reset=False,
        status_message="no valid placement",
        error_message="no valid placement",
        obstacle_result=ObstacleOperationResult(
            done=True,
            succeeded=False,
            operation="add",
            message="no valid placement",
        ),
    )

    class FakeDashboard:
        def __init__(self, *_args, **_kwargs):
            self.linear_spin = SimpleNamespace(value=lambda: 0.4)
            self.angular_spin = SimpleNamespace(value=lambda: 0.8)
            self._calls = 0

        def process_events(self):
            pass

        def current_command(self):
            self._calls += 1
            if self._calls == 1:
                return DashboardCommand(0.4, 0.2, structural_action=AddObstaclesAction(request))
            return DashboardCommand(0.0, 0.0, should_exit=True)

        def set_switch_busy(self, *_args, **_kwargs):
            pass

        def sync_active_selection(self, *_args):
            pass

        def reset_feedback_history(self):
            pass

        def show_switch_status(self, *_args, **_kwargs):
            pass

        def update_obstacle_snapshots(self, _snapshots):
            pass

        def update(self, _state):
            pass

        def close(self):
            pass

    class FakeManager:
        def snapshot(self, *, include_body_id=False):
            assert include_body_id is False
            return ()

    class FakeCoordinator:
        def __init__(self, *_args, **_kwargs):
            self.world = world
            self.obstacle_manager = FakeManager()

        def enqueue(self, action):
            assert action == AddObstaclesAction(request)

        def step(self, _dt):
            return failed_result

    class FakeCsvLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def record(self, *_args, **_kwargs):
            pass

        def close(self):
            return csv_path

    class FakeDiagnosticSummary:
        def to_dict(self):
            return {}

    monkeypatch.setattr(manual_demo_module.p, "connect", lambda _mode: client_id)
    monkeypatch.setattr(manual_demo_module.p, "disconnect", lambda _client_id: None)
    monkeypatch.setattr(manual_demo_module.p, "getKeyboardEvents", lambda: {})
    monkeypatch.setattr(manual_demo_module, "TelemetryDashboard", FakeDashboard)
    monkeypatch.setattr(manual_demo_module, "SimulationCoordinator", FakeCoordinator)
    monkeypatch.setattr(manual_demo_module, "load_manual_world", lambda *_args: world)
    monkeypatch.setattr(manual_demo_module, "create_obstacle_manager", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeCsvLogger)
    monkeypatch.setattr(manual_demo_module, "command_from_keyboard", lambda *_args: ManualCommand(0.0, 0.0))
    monkeypatch.setattr(manual_demo_module, "update_follow_camera", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(manual_demo_module.pd, "read_csv", lambda _path: object())
    monkeypatch.setattr(manual_demo_module, "compute_diagnostic_summary", lambda _frame: FakeDiagnosticSummary())
    monkeypatch.setattr(manual_demo_module, "write_diagnostic_summary", lambda *_args: Path("diagnostics.json"))
    monkeypatch.setattr(manual_demo_module, "compute_tracking_metrics", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(manual_demo_module, "plot_trajectory", lambda *_args, **_kwargs: Path("trajectory.png"))
    monkeypatch.setattr(manual_demo_module, "plot_feedback_figures", lambda *_args, **_kwargs: [])
    monkeypatch.setattr(manual_demo_module, "_read_lidar_for_robot", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "_probe_terrain_for_robot", lambda *_args: None)

    result = manual_demo_module.run_manual_demo(
        ExperimentConfig(mode="gui", time_step=0.01, dashboard_enabled=True, log_dir=tmp_path),
        duration_limit_sec=0.02,
    )

    assert command_calls[0][0] > 0.0
    assert result.obstacle_event_log_path is not None
    rows = [json.loads(line) for line in result.obstacle_event_log_path.read_text(encoding="utf-8").splitlines()]
    assert rows == [
        {
            "sim_time": 0.0,
            "event_type": "add",
            "logical_id": None,
            "request_params": {
                "mode": "mixed",
                "count": 3,
                "shape": "box",
                "moving_ratio": 0.3,
                "seed": 42,
                "speed": 0.35,
            },
            "seed": 42,
            "robot_model": "df_back",
            "terrain": {
                "terrain_model": "flat",
                "slope_deg": 0.0,
                "golf_seed": 0,
                "golf_relief": "medium",
            },
            "success": False,
            "error_reason": "no valid placement",
        }
    ]


def test_manual_demo_logs_stateful_fifo_events_without_duplicates(monkeypatch):
    """多动作 FIFO 下，事件日志要按真实完成顺序配对且不重复写 last_result。"""
    client_id = 23
    rows = []
    add_request = ObstacleGenerationRequest("static", 1, shape="box", seed=11)
    state = SimpleNamespace(x=0.0, y=0.0, out_of_bounds=False)
    robot = SimpleNamespace(
        robot_id=29,
        command_twist=lambda *_args, **_kwargs: None,
        read_physics_state=lambda **_kwargs: state,
    )
    world = SimpleNamespace(
        scene=SimpleNamespace(terrain_type="flat", slope_deg=0.0),
        active_robot=SimpleNamespace(robot=robot, robot_model="df_back"),
        terrain=TerrainSelection("flat"),
    )
    actions = [
        AddObstaclesAction(add_request),
        DeleteObstacleAction(7),
        ClearObstaclesAction(),
        None,
        None,
        None,
    ]

    class FakeDashboard:
        def __init__(self, *_args, **_kwargs):
            self.linear_spin = SimpleNamespace(value=lambda: 0.4)
            self.angular_spin = SimpleNamespace(value=lambda: 0.8)
            self._calls = 0

        def process_events(self):
            pass

        def current_command(self):
            action = actions[self._calls] if self._calls < len(actions) else None
            self._calls += 1
            return DashboardCommand(0.0, 0.0, structural_action=action)

        def set_switch_busy(self, *_args, **_kwargs):
            pass

        def set_structure_busy(self, *_args, **_kwargs):
            pass

        def sync_active_selection(self, *_args):
            pass

        def reset_feedback_history(self):
            pass

        def show_switch_status(self, *_args, **_kwargs):
            pass

        def update_obstacle_snapshots(self, _snapshots):
            pass

        def update(self, _state):
            pass

        def close(self):
            pass

    class FakeManager:
        def snapshot(self, *, include_body_id=False):
            assert include_body_id is False
            return ()

    class StatefulCoordinator:
        def __init__(self, *_args, **_kwargs):
            self.world = world
            self.obstacle_manager = FakeManager()
            self._queue = []
            self._active = None
            self._active_steps = 0
            self.last_result = None

        @property
        def has_pending_action(self):
            return self._active is not None or bool(self._queue)

        def enqueue(self, action):
            self._queue.append(action)

        def step(self, _dt):
            if self._active is None and self._queue:
                self._active = self._queue.pop(0)
                self._active_steps = 0
            if isinstance(self._active, AddObstaclesAction):
                self._active_steps += 1
                if self._active_steps == 1:
                    self.last_result = SimpleNamespace(
                        world=world,
                        state_changed=False,
                        world_reset=False,
                        status_message="pending add",
                        error_message=None,
                        obstacle_result=ObstacleOperationResult(done=False, succeeded=False, operation="add", message="pending"),
                    )
                    return self.last_result
                self._active = None
                self.last_result = SimpleNamespace(
                    world=world,
                    state_changed=True,
                    world_reset=False,
                    status_message="added",
                    error_message=None,
                    obstacle_result=ObstacleOperationResult(done=True, succeeded=True, operation="add", published_count=1),
                )
                return self.last_result
            if isinstance(self._active, DeleteObstacleAction):
                self._active = None
                self.last_result = SimpleNamespace(
                    world=world,
                    state_changed=True,
                    world_reset=False,
                    status_message="deleted",
                    error_message=None,
                    obstacle_result=ObstacleOperationResult(done=True, succeeded=True, operation="delete", deleted_count=1),
                )
                return self.last_result
            if isinstance(self._active, ClearObstaclesAction):
                self._active_steps += 1
                if self._active_steps == 1:
                    self.last_result = SimpleNamespace(
                        world=world,
                        state_changed=False,
                        world_reset=False,
                        status_message="pending clear",
                        error_message=None,
                        obstacle_result=ObstacleOperationResult(done=False, succeeded=False, operation="clear", message="pending"),
                    )
                    return self.last_result
                self._active = None
                self.last_result = SimpleNamespace(
                    world=world,
                    state_changed=True,
                    world_reset=False,
                    status_message="cleared",
                    error_message=None,
                    obstacle_result=ObstacleOperationResult(done=True, succeeded=True, operation="clear", deleted_count=4),
                )
                return self.last_result
            return self.last_result

    class CapturingObstacleEventLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def record_event(self, **kwargs):
            rows.append(kwargs)

        def close(self):
            return Path("manual_fifo_obstacles.jsonl")

    class FakeCsvLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def record(self, *_args, **_kwargs):
            pass

        def close(self):
            return Path("manual_fifo.csv")

    class FakeDiagnosticSummary:
        def to_dict(self):
            return {}

    monkeypatch.setattr(manual_demo_module.p, "connect", lambda _mode: client_id)
    monkeypatch.setattr(manual_demo_module.p, "disconnect", lambda _client_id: None)
    monkeypatch.setattr(manual_demo_module.p, "getKeyboardEvents", lambda: {})
    monkeypatch.setattr(manual_demo_module, "TelemetryDashboard", FakeDashboard)
    monkeypatch.setattr(manual_demo_module, "SimulationCoordinator", StatefulCoordinator)
    monkeypatch.setattr(manual_demo_module, "load_manual_world", lambda *_args: world)
    monkeypatch.setattr(manual_demo_module, "create_obstacle_manager", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeCsvLogger)
    monkeypatch.setattr(manual_demo_module, "ObstacleEventLogger", CapturingObstacleEventLogger)
    monkeypatch.setattr(manual_demo_module, "command_from_keyboard", lambda *_args: ManualCommand(0.0, 0.0))
    monkeypatch.setattr(manual_demo_module, "update_follow_camera", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(manual_demo_module.pd, "read_csv", lambda _path: object())
    monkeypatch.setattr(manual_demo_module, "compute_diagnostic_summary", lambda _frame: FakeDiagnosticSummary())
    monkeypatch.setattr(manual_demo_module, "write_diagnostic_summary", lambda *_args: Path("diagnostics.json"))
    monkeypatch.setattr(manual_demo_module, "compute_tracking_metrics", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(manual_demo_module, "plot_trajectory", lambda *_args, **_kwargs: Path("trajectory.png"))
    monkeypatch.setattr(manual_demo_module, "plot_feedback_figures", lambda *_args, **_kwargs: [])
    monkeypatch.setattr(manual_demo_module, "_read_lidar_for_robot", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "_probe_terrain_for_robot", lambda *_args: None)

    manual_demo_module.run_manual_demo(
        ExperimentConfig(mode="gui", time_step=0.01, dashboard_enabled=True),
        duration_limit_sec=0.06,
    )

    assert [row["event_type"] for row in rows] == ["add", "delete", "clear"]
    assert rows[0]["request_params"]["seed"] == 11
    assert rows[1]["logical_id"] == 7
    assert rows[1]["request_params"] == {"logical_id": 7}
    assert rows[2]["request_params"] == {}


def test_manual_demo_closes_obstacle_event_logger_on_exception(monkeypatch):
    """物理循环中途异常时，障碍物事件日志也必须和 CSV 一样关闭。"""
    client_id = 21
    closed = []
    state = SimpleNamespace(x=0.0, y=0.0, out_of_bounds=False)
    robot = SimpleNamespace(
        robot_id=27,
        command_twist=lambda *_args, **_kwargs: None,
        read_physics_state=lambda **_kwargs: state,
    )
    world = SimpleNamespace(
        scene=SimpleNamespace(terrain_type="flat", slope_deg=0.0),
        active_robot=SimpleNamespace(robot=robot, robot_model="df_back"),
        terrain=TerrainSelection("flat"),
    )

    class FakeDashboard:
        def __init__(self, *_args, **_kwargs):
            self.linear_spin = SimpleNamespace(value=lambda: 0.4)
            self.angular_spin = SimpleNamespace(value=lambda: 0.8)

        def process_events(self):
            pass

        def current_command(self):
            return DashboardCommand(0.0, 0.0)

        def show_switch_status(self, *_args, **_kwargs):
            pass

        def close(self):
            pass

    class FakeCoordinator:
        def __init__(self, *_args, **_kwargs):
            self.world = world
            self.obstacle_manager = SimpleNamespace(snapshot=lambda **_kwargs: ())

        def enqueue(self, _action):
            pass

        def step(self, _dt):
            raise RuntimeError("physics failed")

    class FakeCsvLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def close(self):
            return Path("manual_exception.csv")

    class FakeObstacleEventLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def close(self):
            closed.append(True)
            return Path("manual_obstacles.jsonl")

    monkeypatch.setattr(manual_demo_module.p, "connect", lambda _mode: client_id)
    monkeypatch.setattr(manual_demo_module.p, "disconnect", lambda _client_id: None)
    monkeypatch.setattr(manual_demo_module.p, "getKeyboardEvents", lambda: {})
    monkeypatch.setattr(manual_demo_module, "TelemetryDashboard", FakeDashboard)
    monkeypatch.setattr(manual_demo_module, "SimulationCoordinator", FakeCoordinator)
    monkeypatch.setattr(manual_demo_module, "load_manual_world", lambda *_args: world)
    monkeypatch.setattr(manual_demo_module, "create_obstacle_manager", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeCsvLogger)
    monkeypatch.setattr(manual_demo_module, "ObstacleEventLogger", FakeObstacleEventLogger, raising=False)
    monkeypatch.setattr(manual_demo_module, "command_from_keyboard", lambda *_args: ManualCommand(0.0, 0.0))

    with pytest.raises(RuntimeError, match="physics failed"):
        manual_demo_module.run_manual_demo(ExperimentConfig(mode="gui", time_step=0.01), duration_limit_sec=0.01)

    assert closed == [True]


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
