# 手动演示测试：保护 GUI 手动模式的退出时长策略和阶段二结构操作主循环。
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pybullet as p
import pytest

import slope_sim.coordinator as coordinator_module
import slope_sim.manual_demo as manual_demo_module
from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import load_manual_robot, reload_manual_robot
from slope_sim.dashboard import DashboardCommand
from slope_sim.interfaces.config import InterfaceConfig
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
    LoadSceneAction,
    is_safe_stop_action,
)
from slope_sim.obstacles import ObstacleGenerationRequest
from slope_sim.scene import create_slope_scene
from slope_sim.simulation import _probe_terrain_for_robot, _read_lidar_for_robot
from slope_sim.window_layout import (
    DisplayMetrics,
    Rect,
    WindowLayout,
    WindowLayoutError,
    calculate_window_layout,
)


class FakeObstacleEventLogger:
    """不关心事件日志的手动循环测试用空日志器，避免污染 results 目录。"""

    def __init__(self, *_args, **_kwargs):
        pass

    def record_event(self, **_kwargs):
        pass

    def close(self):
        return Path("manual_obstacles.jsonl")


class _WindowedFakeDashboard:
    """为既有手动循环替身补齐 Task 13 的窗口矩形契约。"""

    def apply_window_rect(self, _rect, **_kwargs):
        pass


def _install_fake_manual_qt(monkeypatch, trace: list[object]) -> None:
    """安装最小 QtWidgets 替身，用于验证 QApplication 的懒创建顺序。"""

    class FakeQApplication:
        _instance = None

        @classmethod
        def instance(cls):
            trace.append("qapp.instance")
            return cls._instance

        def __init__(self, _args):
            trace.append("qapp.create")
            type(self)._instance = self

    qt_widgets = ModuleType("PySide6.QtWidgets")
    qt_widgets.QApplication = FakeQApplication
    package = ModuleType("PySide6")
    package.QtWidgets = qt_widgets
    monkeypatch.setitem(sys.modules, "PySide6", package)
    monkeypatch.setitem(sys.modules, "PySide6.QtWidgets", qt_widgets)


@pytest.fixture(autouse=True)
def _isolate_manual_window_services(monkeypatch):
    """让手动循环测试不访问真实 Qt 屏幕、X11 窗口或 GUI 连接参数。"""
    _install_fake_manual_qt(monkeypatch, [])
    available = Rect(0, 0, 1000, 700)
    monkeypatch.setattr(
        manual_demo_module,
        "primary_available_geometry",
        lambda: available,
        raising=False,
    )
    def display_metrics():
        current = manual_demo_module.primary_available_geometry()
        return DisplayMetrics(
            screen=current,
            available=current,
            device_pixel_ratio=1.0,
        )

    monkeypatch.setattr(
        manual_demo_module,
        "primary_display_metrics",
        display_metrics,
        raising=False,
    )
    monkeypatch.setattr(
        manual_demo_module,
        "x11_available_geometry",
        lambda metrics: metrics.available,
        raising=False,
    )
    monkeypatch.setattr(
        manual_demo_module,
        "search_x11_window_ids",
        lambda *_args, **_kwargs: (),
        raising=False,
    )
    monkeypatch.setattr(
        manual_demo_module,
        "apply_main_window_rect",
        lambda _rect: None,
        raising=False,
    )
    monkeypatch.setattr(
        manual_demo_module,
        "connect_pybullet_gui",
        lambda _rect, *, pybullet_module: pybullet_module.connect(pybullet_module.GUI),
        raising=False,
    )


def _patch_manual_dashboard_startup(monkeypatch, events: list[object], available: Rect) -> None:
    """把手动入口推进到 Dashboard 构造点，同时隔离场景和日志副作用。"""
    document = SimpleNamespace(sensors=object())
    robot = SimpleNamespace(robot_id=23)
    world = SimpleNamespace(
        scene=SimpleNamespace(terrain_type="flat", slope_deg=0.0),
        active_robot=SimpleNamespace(robot=robot, robot_model="df_back"),
        terrain=TerrainSelection("flat"),
    )

    class StartupLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def close(self):
            events.append("logger.close")
            return Path("manual_startup.csv")

    monkeypatch.setattr(manual_demo_module, "initial_scene_document", lambda _config: document)
    monkeypatch.setattr(manual_demo_module, "primary_available_geometry", lambda: available)
    monkeypatch.setattr(
        manual_demo_module,
        "connect_pybullet_gui",
        lambda rect, *, pybullet_module: events.append(("connect", rect, pybullet_module)) or 72,
    )
    monkeypatch.setattr(
        manual_demo_module.p,
        "connect",
        lambda *_args, **_kwargs: events.append("direct p.connect") or 72,
    )
    monkeypatch.setattr(
        manual_demo_module.p,
        "disconnect",
        lambda client_id: events.append(("disconnect", client_id)),
    )
    monkeypatch.setattr(
        manual_demo_module,
        "build_world_from_scene_document",
        lambda *_args, **_kwargs: (world, object()),
    )
    monkeypatch.setattr(manual_demo_module, "SimulationCoordinator", lambda *_args, **_kwargs: object())
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", StartupLogger)
    monkeypatch.setattr(manual_demo_module, "ObstacleEventLogger", StartupLogger)


def _body_ids(client_id: int) -> set[int]:
    """读取 DIRECT 世界 body 集，验证 reload 失败后没有重复车辆。"""
    return {
        p.getBodyUniqueId(index, physicsClientId=client_id)
        for index in range(p.getNumBodies(physicsClientId=client_id))
    }


def test_manual_demo_calculates_full_layout_before_connect_and_disconnects_on_apply_error(
    monkeypatch,
):
    """禁用 Dashboard 时先按主屏可用区连接，主窗应用失败后必须断连。"""
    trace: list[object] = []
    available = Rect(20, 30, 1000, 700)
    layout = WindowLayout(main=available, dashboard=None)
    _install_fake_manual_qt(monkeypatch, trace)

    monkeypatch.setattr(
        manual_demo_module,
        "initial_scene_document",
        lambda _config: trace.append("validate") or object(),
    )
    monkeypatch.setattr(
        manual_demo_module,
        "primary_available_geometry",
        lambda: trace.append("available") or available,
        raising=False,
    )
    monkeypatch.setattr(
        manual_demo_module,
        "calculate_window_layout",
        lambda rect, enabled: trace.append(("calculate", rect, enabled)) or layout,
        raising=False,
    )
    monkeypatch.setattr(
        manual_demo_module,
        "connect_pybullet_gui",
        lambda rect, *, pybullet_module: trace.append(("connect", rect, pybullet_module)) or 71,
        raising=False,
    )
    monkeypatch.setattr(
        manual_demo_module,
        "search_x11_window_ids",
        lambda title, *, only_visible: (
            trace.append(("snapshot", title, only_visible)) or ("41", "77")
        ),
        raising=False,
    )

    def fail_apply(rect, *, excluded_window_ids):
        trace.append(("apply_main", rect, excluded_window_ids))
        raise WindowLayoutError("main placement failed")

    monkeypatch.setattr(
        manual_demo_module,
        "apply_main_window_rect",
        fail_apply,
        raising=False,
    )
    monkeypatch.setattr(
        manual_demo_module.p,
        "connect",
        lambda *_args, **_kwargs: trace.append("direct p.connect") or 71,
    )
    monkeypatch.setattr(
        manual_demo_module,
        "build_world_from_scene_document",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("world built before layout")),
    )
    monkeypatch.setattr(
        manual_demo_module.p,
        "disconnect",
        lambda client_id: trace.append(("disconnect", client_id)),
    )

    with pytest.raises(WindowLayoutError, match="main placement failed"):
        manual_demo_module.run_manual_demo(
            ExperimentConfig(
                mode="gui",
                dashboard_enabled=False,
                interface_enabled=False,
            )
        )

    assert trace == [
        "validate",
        "qapp.instance",
        "qapp.create",
        "available",
        ("calculate", available, False),
        ("snapshot", manual_demo_module.PYBULLET_WINDOW_TITLE, False),
        ("connect", available, manual_demo_module.p),
        ("apply_main", available, ("41", "77")),
        ("disconnect", 71),
    ]


def test_manual_demo_applies_dashboard_rect_with_enterprise_interface_options(monkeypatch):
    """Dashboard 构造成功后立即应用右侧矩形，布局错误不能进入 fallback。"""
    events: list[object] = []
    available = Rect(20, 30, 1000, 700)
    layout = calculate_window_layout(available, True)
    constructor_kwargs: dict[str, object] = {}
    _patch_manual_dashboard_startup(monkeypatch, events, available)

    class FakeDashboard(_WindowedFakeDashboard):
        def __init__(self, *_args, **kwargs):
            constructor_kwargs.update(kwargs)
            events.append("dashboard.construct")

        def apply_window_rect(self, rect, *, display_metrics):
            events.append(("dashboard.apply", rect, display_metrics))
            raise WindowLayoutError("dashboard placement failed")

        def close(self):
            events.append("dashboard.close")

    monkeypatch.setattr(
        manual_demo_module,
        "apply_main_window_rect",
        lambda rect: events.append(("main.apply", rect)),
    )
    monkeypatch.setattr(manual_demo_module, "TelemetryDashboard", FakeDashboard)
    monkeypatch.setattr(
        manual_demo_module.p,
        "addUserDebugParameter",
        lambda *_args: (_ for _ in ()).throw(AssertionError("layout error entered fallback")),
    )

    config = ExperimentConfig(
        mode="gui",
        dashboard_enabled=True,
        interface_enabled=False,
        interface_mode="local",
        developer_diagnostics_enabled=True,
    )
    with pytest.raises(WindowLayoutError, match="dashboard placement failed"):
        manual_demo_module.run_manual_demo(config)

    assert constructor_kwargs["interface_config"] == InterfaceConfig.default(
        transport_mode="local"
    )
    assert constructor_kwargs["developer_diagnostics_enabled"] is True
    assert (
        "dashboard.apply",
        layout.dashboard,
        DisplayMetrics(screen=available, available=available, device_pixel_ratio=1.0),
    ) in events
    assert ("main.apply", layout.main) in events
    assert "dashboard.close" in events
    assert events[-1] == ("disconnect", 72)


@pytest.mark.parametrize(
    "constructor_error",
    (
        RuntimeError("dashboard constructor failed"),
        WindowLayoutError("dashboard constructor layout failed"),
    ),
    ids=("runtime-error", "window-layout-error"),
)
def test_manual_demo_dashboard_constructor_failure_is_fatal_and_cleans_up(
    monkeypatch,
    constructor_error,
):
    """请求 Dashboard 时构造异常必须终止，不能扩展 Main 或启用滑条。"""
    events: list[object] = []
    available = Rect(20, 30, 1000, 700)
    initial_layout = calculate_window_layout(available, True)
    calculate_calls: list[bool] = []
    _patch_manual_dashboard_startup(monkeypatch, events, available)

    def calculate(rect, dashboard_enabled):
        calculate_calls.append(dashboard_enabled)
        return calculate_window_layout(rect, dashboard_enabled)

    def fail_constructor(*_args, **_kwargs):
        raise constructor_error

    monkeypatch.setattr(manual_demo_module, "calculate_window_layout", calculate)
    monkeypatch.setattr(
        manual_demo_module,
        "apply_main_window_rect",
        lambda rect: events.append(("main.apply", rect)),
    )
    monkeypatch.setattr(manual_demo_module, "TelemetryDashboard", fail_constructor)
    monkeypatch.setattr(
        manual_demo_module.p,
        "addUserDebugParameter",
        lambda *args: events.append(("slider", args)) or len(events),
    )
    with pytest.raises(RuntimeError, match="dashboard construction failed") as excinfo:
        manual_demo_module.run_manual_demo(
            ExperimentConfig(
                mode="gui",
                dashboard_enabled=True,
                interface_enabled=False,
            )
        )

    assert excinfo.value.__cause__ is constructor_error
    assert calculate_calls == [True]
    assert [event for event in events if event[0] == "main.apply"] == [
        ("main.apply", initial_layout.main),
    ]
    assert [event for event in events if event[0] == "slider"] == []
    assert events[-1] == ("disconnect", 72)


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


def test_manual_robot_reload_commits_replacement_when_old_remove_raises_after_delete(
    monkeypatch,
):
    """旧车已实际删除时，即使 removeBody 随后抛错也应提交 replacement。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", terrain_model="flat")
        scene = create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=config.time_step,
            terrain_model="flat",
        )
        active = load_manual_robot(client_id, config, scene, robot_model="df_back")
        old_robot_id = active.robot.robot_id
        original_remove = coordinator_module.p.removeBody

        def remove_old_then_raise(body_id: int, **kwargs) -> None:
            original_remove(body_id, **kwargs)
            if body_id == old_robot_id:
                raise RuntimeError("old reload robot removed before injected error")

        monkeypatch.setattr(coordinator_module.p, "removeBody", remove_old_then_raise)

        replacement = reload_manual_robot(
            client_id,
            active,
            config,
            scene,
            robot_model="df_mid",
        )

        assert replacement.robot_model == "df_mid"
        assert old_robot_id not in _body_ids(client_id)
        assert _body_ids(client_id) == set(scene.body_ids) | {replacement.robot.robot_id}
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("failure_mode", ("raises", "remains"))
def test_manual_robot_reload_cleans_replacement_when_old_body_remains(
    monkeypatch,
    failure_mode: str,
):
    """旧车仍在时 reload 必须清理 replacement 并显式失败。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", terrain_model="flat")
        scene = create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=config.time_step,
            terrain_model="flat",
        )
        active = load_manual_robot(client_id, config, scene, robot_model="df_back")
        old_robot_id = active.robot.robot_id
        before_body_ids = _body_ids(client_id)
        original_remove = coordinator_module.p.removeBody

        def fail_or_leave_old(body_id: int, **kwargs) -> None:
            if body_id == old_robot_id:
                if failure_mode == "raises":
                    raise RuntimeError("old reload robot removal failed")
                return
            original_remove(body_id, **kwargs)

        monkeypatch.setattr(coordinator_module.p, "removeBody", fail_or_leave_old)

        with pytest.raises(RuntimeError) as excinfo:
            reload_manual_robot(
                client_id,
                active,
                config,
                scene,
                robot_model="df_mid",
            )

        expected_reason = (
            "old reload robot removal failed"
            if failure_mode == "raises"
            else "remained"
        )
        assert expected_reason in str(excinfo.value)
        assert _body_ids(client_id) == before_body_ids
    finally:
        p.disconnect(client_id)


def test_manual_robot_reload_diagnosis_failure_cleans_replacement_and_raises(
    monkeypatch,
):
    """无法诊断旧体是否删除时不能返回悬空引用，也不能遗留 replacement。"""
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", terrain_model="flat")
        scene = create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=config.time_step,
            terrain_model="flat",
        )
        active = load_manual_robot(client_id, config, scene, robot_model="df_back")
        old_robot_id = active.robot.robot_id
        before_body_ids = _body_ids(client_id)
        original_remove = coordinator_module.p.removeBody
        original_current_body_ids = coordinator_module._current_body_ids
        diagnosis_calls = 0

        def leave_old_body(body_id: int, **kwargs) -> None:
            if body_id == old_robot_id:
                return
            original_remove(body_id, **kwargs)

        def fail_first_diagnosis(current_client_id: int) -> set[int]:
            nonlocal diagnosis_calls
            diagnosis_calls += 1
            if diagnosis_calls == 1:
                raise RuntimeError("reload robot diagnosis unavailable")
            return original_current_body_ids(current_client_id)

        monkeypatch.setattr(coordinator_module.p, "removeBody", leave_old_body)
        monkeypatch.setattr(
            coordinator_module,
            "_current_body_ids",
            fail_first_diagnosis,
        )

        with pytest.raises(RuntimeError, match="reload robot diagnosis unavailable"):
            reload_manual_robot(
                client_id,
                active,
                config,
                scene,
                robot_model="df_mid",
            )

        assert _body_ids(client_id) == before_body_ids
    finally:
        p.disconnect(client_id)


def test_merge_manual_commands_uses_pybullet_keyboard_when_dashboard_is_idle():
    dashboard = DashboardCommand(0.0, 0.0)
    keyboard = ManualCommand(0.4, 0.0)

    merged = merge_manual_commands(dashboard, keyboard)

    assert merged.linear_velocity == 0.4
    assert merged.angular_velocity == 0.0
    assert merged.structural_action is None


def test_task12_paused_command_is_zero_and_blocks_keyboard_motion():
    dashboard = DashboardCommand(
        0.7,
        -0.2,
        paused=True,
        camera_follow_enabled=True,
        camera_follow_view="side",
    )

    merged = merge_manual_commands(dashboard, ManualCommand(0.4, 0.8))

    assert merged.linear_velocity == 0.0
    assert merged.angular_velocity == 0.0
    assert merged.paused is True
    assert merged.camera_follow_enabled is True
    assert merged.camera_follow_view == "side"


def test_task12_limit_propagates_paused_and_stops_immediately():
    target = DashboardCommand(0.8, 0.4, paused=True, should_exit=True)

    limited = limit_manual_command_step(
        DashboardCommand(1.0, 1.0),
        target,
        dt=0.1,
        linear_acceleration_limit=0.1,
        angular_acceleration_limit=0.1,
    )

    assert limited.linear_velocity == 0.0
    assert limited.angular_velocity == 0.0
    assert limited.paused is True
    assert limited.should_exit is True


def test_task12_load_scene_action_validates_document_and_is_safe_stop():
    from slope_sim.scene_config import SceneDocument
    from slope_sim.truth_sensors import SensorMounts

    document = SceneDocument.from_runtime(
        "df_back",
        TerrainSelection("flat"),
        (),
        SensorMounts.default(),
    )

    action = LoadSceneAction(document)

    assert action.document is document
    assert is_safe_stop_action(action) is True
    with pytest.raises(ValueError, match="SceneDocument"):
        LoadSceneAction(object())


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


def test_manual_demo_uses_absolute_deadline_and_keeps_configured_camera_state(monkeypatch):
    """手动循环只等待当前帧余量，且禁用 Dashboard 时沿用配置相机状态。"""
    client_id = 17
    robot_id = 23
    camera_calls = []
    sleep_delays = []
    clock = {"now": 0.0}
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
    monkeypatch.setattr(
        manual_demo_module,
        "build_world_from_scene_document",
        lambda *_args, **_kwargs: (
            world,
            SimpleNamespace(update_moving=lambda _dt: None),
        ),
    )
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeLogger)
    monkeypatch.setattr(manual_demo_module, "ObstacleEventLogger", FakeObstacleEventLogger)
    monkeypatch.setattr(manual_demo_module, "command_from_keyboard", lambda *_args: ManualCommand(0.0, 0.0))
    monkeypatch.setattr(manual_demo_module, "_read_lidar_for_robot", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "_probe_terrain_for_robot", lambda *_args: None)
    def update_follow_camera(*args):
        camera_calls.append(args)
        clock["now"] += 0.004

    def sleep(seconds):
        sleep_delays.append(seconds)
        clock["now"] += seconds

    monkeypatch.setattr(manual_demo_module, "update_follow_camera", update_follow_camera)
    monkeypatch.setattr(manual_demo_module.time, "monotonic", lambda: clock["now"])
    monkeypatch.setattr(manual_demo_module.time, "sleep", sleep)
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
                dashboard_enabled=False,
            interface_enabled=False,
            camera_follow_enabled=True,
            camera_follow_view="side",
        ),
        duration_limit_sec=0.01,
    )

    assert camera_calls == [(client_id, robot_id, 6.0, -35.0, 45.0, "side")]
    assert sleep_delays == pytest.approx([0.006])
    assert clock["now"] == pytest.approx(0.01)


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

    class FakeDashboard(_WindowedFakeDashboard):
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
    monkeypatch.setattr(manual_demo_module, "build_world_from_scene_document", lambda *_args, **_kwargs: (world, object()))
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
        ExperimentConfig(mode="gui", time_step=0.01, dashboard_enabled=True, interface_enabled=False),
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

    class FakeDashboard(_WindowedFakeDashboard):
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
    monkeypatch.setattr(manual_demo_module, "build_world_from_scene_document", lambda *_args, **_kwargs: (world, object()))
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
        ExperimentConfig(mode="gui", time_step=0.01, dashboard_enabled=True, interface_enabled=False),
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
    lazy_values = []
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

    class FakeDashboard(_WindowedFakeDashboard):
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
            lazy_values.append(callable(value))
            if callable(value):
                value = value()
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
    monkeypatch.setattr(manual_demo_module, "build_world_from_scene_document", lambda *_args, **_kwargs: (world, object()))
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
        ExperimentConfig(mode="gui", time_step=0.01, dashboard_enabled=True, interface_enabled=False),
        duration_limit_sec=0.02,
    )

    assert refreshed == [snapshots]
    assert lazy_values == [True]


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

    class FakeDashboard(_WindowedFakeDashboard):
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
    monkeypatch.setattr(manual_demo_module, "build_world_from_scene_document", lambda *_args, **_kwargs: (world, object()))
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
        ExperimentConfig(
            mode="gui",
            time_step=0.01,
            dashboard_enabled=True,
            interface_enabled=False,
            log_dir=tmp_path,
        ),
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
    history_resets = []
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

    class FakeDashboard(_WindowedFakeDashboard):
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
            history_resets.append(True)

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
    monkeypatch.setattr(manual_demo_module, "build_world_from_scene_document", lambda *_args, **_kwargs: (world, object()))
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
        ExperimentConfig(mode="gui", time_step=0.01, dashboard_enabled=True, interface_enabled=False),
        duration_limit_sec=0.06,
    )

    assert [row["event_type"] for row in rows] == ["add", "delete", "clear"]
    assert rows[0]["request_params"]["seed"] == 11
    assert rows[1]["logical_id"] == 7
    assert rows[1]["request_params"] == {"logical_id": 7}
    assert rows[2]["request_params"] == {}
    assert history_resets == []


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

    class FakeDashboard(_WindowedFakeDashboard):
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
    monkeypatch.setattr(manual_demo_module, "build_world_from_scene_document", lambda *_args, **_kwargs: (world, object()))
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeCsvLogger)
    monkeypatch.setattr(manual_demo_module, "ObstacleEventLogger", FakeObstacleEventLogger, raising=False)
    monkeypatch.setattr(manual_demo_module, "command_from_keyboard", lambda *_args: ManualCommand(0.0, 0.0))

    with pytest.raises(RuntimeError, match="physics failed"):
        manual_demo_module.run_manual_demo(
            ExperimentConfig(mode="gui", time_step=0.01, interface_enabled=False),
            duration_limit_sec=0.01,
        )

    assert closed == [True]


def test_manual_enabled_local_loop_pauses_polls_rebinds_and_returns_interface_logs(
    monkeypatch,
    tmp_path: Path,
) -> None:
    """手动入口低频观测，暂停仍发现对端，恢复后立即重建观测边界。"""
    from slope_sim.interfaces.logging import InterfaceLogPaths
    from slope_sim.interfaces.status import InterfaceStatusSnapshot, WheelCommandStatus
    from slope_sim.scene_config import SceneDocument, SensorDocument, TerrainDocument

    trace: list[str] = []
    direct_twists: list[tuple[str, float, float]] = []
    controlled_robot_ids: list[int] = []
    statuses: list[InterfaceStatusSnapshot] = []
    decision_wall_times: list[float] = []
    resume_wall_times: list[float] = []
    clock = [0.0]
    sleeps: list[float] = []
    state = SimpleNamespace(x=0.0, y=0.0, out_of_bounds=False)

    def sleep(delay: float) -> None:
        sleeps.append(delay)
        clock[0] += delay

    def make_robot(robot_id: int):
        return SimpleNamespace(
            robot_id=robot_id,
            command_twist=lambda linear, angular, **_kwargs: direct_twists.append(
                (str(robot_id), linear, angular)
            ),
            read_physics_state=lambda **_kwargs: state,
        )

    old_robot = make_robot(31)
    new_robot = make_robot(32)
    terrain = TerrainSelection("flat")
    scene = SimpleNamespace(
        terrain_type="flat",
        slope_deg=0.0,
        body_ids=(10,),
        spawn_position=(0.0, 0.0, 0.0),
    )
    world = SimpleNamespace(
        scene=scene,
        active_robot=SimpleNamespace(robot=old_robot, robot_model="df_back"),
        terrain=terrain,
    )
    document = SceneDocument(
        1,
        "df_back",
        TerrainDocument("flat", 0.0, 0, "medium"),
        (),
        SensorDocument.default(),
    )
    snapshot = InterfaceStatusSnapshot(
        captured_at=0.0,
        transport_mode="local",
        ecal_connected=False,
        command=WheelCommandStatus("waiting_command", 0.0, None, 0, 0),
        wheel_state=None,
        topics={},
    )
    dashboard_snapshot = object()

    class FakeManager:
        def snapshot(self, *, include_body_id=False):
            return ()

    manager = FakeManager()
    cadence_resets: list[float] = []
    real_cadence_type = manual_demo_module.RuntimeObservationCadence

    class RecordingCadence(real_cadence_type):
        def reset(self) -> None:
            cadence_resets.append(clock[0])
            super().reset()

    monkeypatch.setattr(
        manual_demo_module,
        "RuntimeObservationCadence",
        RecordingCadence,
    )

    class FakeRuntime:
        def __init__(self):
            self.bound_robot = old_robot

        def poll_transport(self):
            trace.append("poll")

        def submit_local_twist(self, linear, angular, dt):
            trace.append(("submit", linear, angular, dt))
            return True

        def before_physics_step(self, _dt, *, wall_time):
            trace.append("before")
            decision_wall_times.append(wall_time)
            controlled_robot_ids.append(self.bound_robot.robot_id)

        def after_physics_step(self, _dt):
            trace.append("after")

        def pause(self):
            trace.append("pause")

        def resume(self, *, wall_time):
            trace.append("resume")
            resume_wall_times.append(wall_time)

        def dashboard_snapshot(self, *, wall_time):
            assert wall_time == pytest.approx(clock[0])
            trace.append("dashboard_snapshot")
            return dashboard_snapshot

        def status_snapshot(self):
            raise AssertionError("manual loop must not request a separate status snapshot")

    runtime = FakeRuntime()

    class FakeSession:
        actual_transport_mode = "local"

        def __init__(self):
            self.runtime = runtime

        def close(self):
            trace.append("runtime.close")
            return InterfaceLogPaths(
                tmp_path / "interfaces.bin",
                tmp_path / "interfaces.jsonl",
            )

    class FakeCoordinator:
        def __init__(self, *_args, **_kwargs):
            self.world = world
            self.obstacle_manager = manager
            self.steps = 0

        @property
        def has_pending_action(self):
            return False

        def enqueue(self, _action):
            pass

        def step(self, _dt):
            trace.append("coordinator.step")
            self.steps += 1
            clock[0] += 0.002
            if self.steps == 1:
                self.world = SimpleNamespace(
                    scene=scene,
                    active_robot=SimpleNamespace(robot=new_robot, robot_model="df_mid"),
                    terrain=terrain,
                )
                runtime.bound_robot = new_robot
            return persistent_result

        def logical_scene_document(self):
            return document

    commands = iter(
        (
            DashboardCommand(0.4, 0.1),
            DashboardCommand(0.0, 0.0, paused=True),
            DashboardCommand(0.0, 0.0, paused=True),
            DashboardCommand(0.2, -0.1),
        )
    )
    persistent_result = SimpleNamespace(
        world=SimpleNamespace(
            scene=scene,
            active_robot=SimpleNamespace(robot=new_robot, robot_model="df_mid"),
            terrain=terrain,
        ),
        state_changed=True,
        world_reset=True,
        status_message="车型已切换为 df_mid",
        error_message=None,
        obstacle_result=None,
    )

    class FakeDashboard(_WindowedFakeDashboard):
        def __init__(self, *_args, **_kwargs):
            self.linear_spin = SimpleNamespace(value=lambda: 0.4)
            self.angular_spin = SimpleNamespace(value=lambda: 0.8)

        def process_events(self):
            trace.append("events")
            clock[0] += 0.030

        def current_command(self):
            return next(commands)

        def update_interface_snapshot(self, value):
            statuses.append(value)

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
            trace.append("dashboard.close")

    class FakeLogger:
        def __init__(self, *_args, **_kwargs):
            pass

        def record(self, *_args, **_kwargs):
            pass

        def close(self):
            return tmp_path / "manual.csv"

    class FakeDiagnosticSummary:
        def to_dict(self):
            return {}

    monkeypatch.setattr(manual_demo_module, "initial_scene_document", lambda _config: (trace.append("load_scene"), document)[1], raising=False)
    monkeypatch.setattr(manual_demo_module.p, "connect", lambda _mode: (trace.append("connect"), 27)[1])
    monkeypatch.setattr(manual_demo_module.p, "disconnect", lambda _client: trace.append("disconnect"))
    monkeypatch.setattr(manual_demo_module.p, "getKeyboardEvents", lambda: {})
    monkeypatch.setattr(manual_demo_module, "build_world_from_scene_document", lambda *_args, **_kwargs: (world, manager), raising=False)
    monkeypatch.setattr(manual_demo_module, "create_interface_session", lambda *_args, **_kwargs: FakeSession(), raising=False)
    monkeypatch.setattr(manual_demo_module, "SimulationCoordinator", FakeCoordinator)
    monkeypatch.setattr(manual_demo_module, "TelemetryDashboard", FakeDashboard)
    monkeypatch.setattr(manual_demo_module, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "CsvSimulationLogger", FakeLogger)
    monkeypatch.setattr(manual_demo_module, "ObstacleEventLogger", FakeObstacleEventLogger)
    monkeypatch.setattr(manual_demo_module, "command_from_keyboard", lambda *_args: ManualCommand(0.0, 0.0))
    monkeypatch.setattr(manual_demo_module, "_read_lidar_for_robot", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "_probe_terrain_for_robot", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "update_follow_camera", lambda *_args: None)
    monkeypatch.setattr(manual_demo_module, "dump_scene_atomic", lambda value, path: (trace.append(("export", value)), path)[1], raising=False)
    monkeypatch.setattr(manual_demo_module.pd, "read_csv", lambda _path: object())
    monkeypatch.setattr(manual_demo_module, "compute_diagnostic_summary", lambda _frame: FakeDiagnosticSummary())
    monkeypatch.setattr(manual_demo_module, "write_diagnostic_summary", lambda *_args: tmp_path / "diagnostics.json")
    monkeypatch.setattr(manual_demo_module, "compute_tracking_metrics", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(manual_demo_module, "plot_trajectory", lambda *_args, **_kwargs: tmp_path / "trajectory.png")
    monkeypatch.setattr(manual_demo_module, "plot_feedback_figures", lambda *_args, **_kwargs: ())

    result = manual_demo_module.run_manual_demo(
        ExperimentConfig(
            mode="gui",
            time_step=0.01,
            interface_mode="local",
            interface_enabled=True,
            interface_log_enabled=True,
            scene_out=tmp_path / "scene.yaml",
        ),
        duration_limit_sec=0.02,
        monotonic=lambda: clock[0],
        sleep=sleep,
    )

    assert direct_twists == []
    assert controlled_robot_ids == [31, 32]
    assert trace.count("poll") == 3
    assert trace.count("coordinator.step") == 2
    assert trace.count("before") == trace.count("after") == 2
    assert trace.count("pause") == trace.count("resume") == 1
    assert decision_wall_times == pytest.approx([0.030, 0.122])
    assert resume_wall_times == pytest.approx([0.122])
    assert cadence_resets == pytest.approx([0.032, 0.122])
    assert max(index for index, value in enumerate(trace) if value == "poll") < trace.index(
        "resume"
    ) < max(index for index, value in enumerate(trace) if value == "before")
    assert statuses == [dashboard_snapshot] * 3
    assert trace.count("dashboard_snapshot") == 3
    assert sleeps == pytest.approx([0.0, 0.0, 0.0, 0.008])
    assert clock[0] == pytest.approx(0.132)
    assert result.interface_binary_log == tmp_path / "interfaces.bin"
    assert result.interface_event_log == tmp_path / "interfaces.jsonl"
    assert result.scene_export == tmp_path / "scene.yaml"
    assert trace.index("load_scene") < trace.index("connect")
    assert trace[-1] == "disconnect"


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
