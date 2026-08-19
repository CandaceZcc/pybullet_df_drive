"""runSim v2：共享 GUI 物理世界的 runtime/worker 生命周期回归。"""
from __future__ import annotations

from collections import deque
import sys
from types import SimpleNamespace

import pytest

from slope_sim.config import ExperimentConfig
from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
from slope_sim.interfaces.v2.models import WheelCommandV2
from slope_sim.interfaces.v2.runtime_protocol import V2RuntimeProtocol
from slope_sim.interfaces.transport import TransportSnapshot, TransportTopicQuality
from slope_sim.model_registry import get_robot_model
from slope_sim.obstacles import ObstacleGeometry, ObstacleSnapshot, ObstacleSpec
from slope_sim.simulation import initial_scene_document


class _Transport:
    def __init__(self) -> None:
        self.closed = False

    def subscribe(self, *_args: object) -> None:
        return None

    def poll_peer_state(self) -> None:
        return None

    def snapshot(self) -> TransportSnapshot:
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=True,
            published_count=0,
            received_count=0,
            error_count=0,
            dropped_count=0,
            topic_quality=(
                TransportTopicQuality(
                    topic="/sim/wheel/command",
                    peer_connected=True,
                    peer_count=1,
                    protocol_state="verified",
                    protocol_detail="",
                    remote_type_names=("slope_sim.interfaces.v2.WheelCommand",),
                    remote_encodings=("proto",),
                    remote_descriptor_sha256=("0" * 64,),
                ),
            ),
        )

    def close(self) -> None:
        self.closed = True


class _Service:
    def __init__(self) -> None:
        self.closed = False

    def begin_draining(self) -> None:
        return None

    def close_idle(self) -> None:
        self.closed = True

    def force_close(self) -> None:
        self.closed = True


def test_moving_obstacle_frame_keeps_v2_worker_generation_and_freezes_latest_snapshot() -> None:
    """移动位姿只进入 capture 快照，不能逐物理帧重建异步 worker。"""
    from slope_sim.coordinator import SimulationCoordinator
    from slope_sim.interfaces.v2.world_runtime import V2ManualWorldRuntime, V2WorldRuntime

    document = initial_scene_document(ExperimentConfig(mode="gui", interface_enabled=True))
    descriptor = load_v2_descriptor()
    controller = V2RuntimeProtocol(
        get_robot_model(document.robot_model), transport=_Transport(), descriptor=descriptor
    )
    services: list[_Service] = []

    class PendingCaptureService(_Service):
        def __init__(self) -> None:
            super().__init__()
            self.pending_capture = True
            self.close_attempts = 0

        def close_idle(self) -> None:
            self.close_attempts += 1
            raise AssertionError("moving-frame worker must not close a pending capture")

    def start_worker(_document: object, _generation: int) -> _Service:
        service = PendingCaptureService()
        services.append(service)
        return service

    class MovingManager:
        has_moving = True

        def __init__(self) -> None:
            self._snapshot = ObstacleSnapshot(
                logical_id=1,
                body_id=17,
                mode="moving",
                shape="box",
                position=(1.0, 0.0, 0.3),
                orientation=(0.0, 0.0, 0.0, 1.0),
            )

        def update_moving(self, _dt: float) -> None:
            self._snapshot = ObstacleSnapshot(
                logical_id=1,
                body_id=17,
                mode="moving",
                shape="box",
                position=(2.0, 0.0, 0.3),
                orientation=(0.0, 0.0, 0.0, 1.0),
            )

        def snapshot(self, *, include_body_id: bool) -> tuple[ObstacleSnapshot, ...]:
            assert include_body_id is True
            return (self._snapshot,)

    class Backend:
        def world_pose(self, frame_id: str) -> tuple[float, float, float]:
            assert frame_id == "lidar_link"
            return (0.0, 0.0, 0.0)

    manager = MovingManager()
    manual_runtime = object.__new__(V2ManualWorldRuntime)
    manual_runtime._world_runtime = V2WorldRuntime(
        controller=controller,
        scene_document=document,
        start_worker=start_worker,
    )
    manual_runtime._sensor_backend = Backend()
    manual_runtime._obstacle_manager = manager
    manual_runtime._runtime = object()
    manual_runtime._make_runtime = lambda: object()

    coordinator = object.__new__(SimulationCoordinator)
    coordinator._active_action = None
    coordinator._queue = deque()
    coordinator.last_result = None
    coordinator.obstacle_manager = manager
    coordinator.interface_runtime = manual_runtime
    coordinator.client_id = 1
    coordinator._step_physics = lambda _client_id: None
    coordinator.logical_scene_document = lambda: document._replace_validated_runtime_obstacles(
        (
            ObstacleSpec(
                logical_id=9,
                mode="static",
                geometry=ObstacleGeometry("box", (0.3, 0.3, 0.3)),
                position=(3.0, 0.0, 0.3),
                orientation=(0.0, 0.0, 0.0, 1.0),
            ),
        )
    )

    before = controller.snapshot()
    coordinator.step(0.01)
    _mount_pose, snapshots = manual_runtime._capture_context()

    assert controller.snapshot().world_generation == before.world_generation
    assert len(services) == 1
    assert services[0].close_attempts == 0
    assert services[0].closed is False
    assert snapshots[0].position == (2.0, 0.0, 0.3)
    assert snapshots[0].body_id is None


def test_worker_close_failure_aborts_prepared_rebuild_and_rejects_stale_token() -> None:
    """有在途采集时 close 失败也必须退出 prepared，后续结构重建仍可恢复。"""
    from slope_sim.interfaces.v2.world_runtime import V2WorldRuntime

    document = initial_scene_document(ExperimentConfig(mode="gui", interface_enabled=True))
    changed_document = document._replace_validated_runtime_obstacles(
        (
            ObstacleSpec(
                logical_id=1,
                mode="static",
                geometry=ObstacleGeometry("box", (0.3, 0.3, 0.3)),
                position=(2.0, 0.0, 0.3),
                orientation=(0.0, 0.0, 0.0, 1.0),
            ),
        )
    )
    descriptor = load_v2_descriptor()
    controller = V2RuntimeProtocol(
        get_robot_model(document.robot_model), transport=_Transport(), descriptor=descriptor
    )
    controller.refresh_transport()
    services: list[_Service] = []

    class PendingService(_Service):
        def __init__(self, *, fail_close: bool) -> None:
            super().__init__()
            self.pending_capture = True
            self.close_attempts = 0
            self._fail_close = fail_close

        def close_idle(self) -> None:
            self.close_attempts += 1
            if self._fail_close:
                raise RuntimeError("pending lidar capture cannot close")
            self.closed = True

    def start_worker(_document: object, _generation: int) -> PendingService:
        service = PendingService(fail_close=not services)
        services.append(service)
        return service

    runtime = V2WorldRuntime(
        controller=controller,
        scene_document=document,
        start_worker=start_worker,
    )
    before = controller.snapshot()
    stale_ingress = controller.capture_ingress()
    stale_command = WheelCommandV2(
        timestamp_ns=10_000_000,
        drive_wheel_speed_rad_s=(1.0, -1.0),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=before.world_generation,
        command_generation=before.command_generation,
        source_id="manual.tool-1",
        source_session_id=b"m" * 16,
        robot_model="df_mid",
        simulation_session_id=before.simulation_session_id,
        descriptor_sha256=descriptor.sha256,
    )

    with pytest.raises(RuntimeError, match="pending lidar capture"):
        runtime.update_scene_document(changed_document)

    assert services[0].close_attempts == 1
    assert len(services) == 2
    assert controller.snapshot().world_generation == before.world_generation
    with pytest.raises(RuntimeError, match="not prepared"):
        controller.commit_world_rebuild()
    assert controller.accept_decoded_command(
        stale_command, received_at=1.0, ingress=stale_ingress
    ) is False

    runtime.update_scene_document(changed_document)

    assert controller.snapshot().world_generation == before.world_generation + 1
    assert runtime.scene_document == changed_document


def test_v2_world_runtime_rebuilds_worker_and_invalidates_old_command_token() -> None:
    """同一 GUI world 的障碍物事务必须换 worker、推进 world/command generation。"""
    from slope_sim.interfaces.v2.world_runtime import V2WorldRuntime

    config = ExperimentConfig(mode="gui", interface_enabled=True)
    document = initial_scene_document(config)
    descriptor = load_v2_descriptor()
    transport = _Transport()
    controller = V2RuntimeProtocol(
        get_robot_model(document.robot_model), transport=transport, descriptor=descriptor
    )
    services: list[_Service] = []

    def start_worker(_document: object, _generation: int) -> _Service:
        service = _Service()
        services.append(service)
        return service

    runtime = V2WorldRuntime(
        controller=controller,
        scene_document=document,
        start_worker=start_worker,
    )
    controller.refresh_transport()
    before = controller.snapshot()
    old_ingress = controller.capture_ingress()
    old_command = WheelCommandV2(
        timestamp_ns=10_000_000,
        drive_wheel_speed_rad_s=(1.0, -1.0),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=before.world_generation,
        command_generation=before.command_generation,
        source_id="manual.tool-1",
        source_session_id=b"m" * 16,
        robot_model="df_mid",
        simulation_session_id=before.simulation_session_id,
        descriptor_sha256=descriptor.sha256,
    )

    runtime.update_scene_document(document)

    after = controller.snapshot()
    assert len(services) == 2
    assert services[0].closed is True
    assert after.world_generation == before.world_generation + 1
    assert after.command_generation > before.command_generation
    assert controller.accept_decoded_command(
        old_command, received_at=1.0, ingress=old_ingress
    ) is False
    assert controller.mailbox.decision(now=1.0).waiting is True
    assert runtime.scene_document == document


def test_manual_demo_ecal_branch_injects_v2_runtime_into_existing_gui_coordinator(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """GUI eCAL 复用已连接 client，并把唯一 v2 runtime 注入 coordinator。"""
    import slope_sim.manual_demo as manual_demo

    client_id = 47
    robot = SimpleNamespace(robot_id=73)
    world = SimpleNamespace(
        scene=SimpleNamespace(body_ids=(101,)),
        active_robot=SimpleNamespace(robot=robot, robot_model="df_mid"),
        terrain=SimpleNamespace(
            terrain_model="flat",
            slope_deg=0.0,
            golf_seed=0,
            golf_relief="medium",
        ),
    )
    calls: dict[str, object] = {}

    class FakeQApplication:
        @staticmethod
        def instance() -> None:
            return None

        def __init__(self, _args: object) -> None:
            pass

    class FakeSensorBackend:
        def __init__(self, actual_client_id: int, robot_id: int) -> None:
            calls["backend"] = (actual_client_id, robot_id)

        def bind_scene(self, terrain_body_ids: tuple[int, ...], snapshots: tuple[object, ...]) -> None:
            calls["bindings"] = (terrain_body_ids, snapshots)

    class FakeV2ManualWorldRuntime:
        def __init__(self, **kwargs: object) -> None:
            calls["v2_init"] = kwargs
            self.obstacle_manager = kwargs["obstacle_manager"]
            self.descriptor = object()
            self.dashboard_snapshot_store = object()

        def close(self) -> None:
            calls["v2_closed"] = True

    class FakeV2Dashboard:
        def refresh_from_store(self, store: object) -> None:
            calls.setdefault("v2_refreshes", []).append(store)

        def launch_live_viewer(self) -> None:
            calls["live_viewer_opened"] = True

    class CaptureDurationCombo:
        def findData(self, duration: int) -> int:
            calls["capture_duration_requested"] = duration
            return 2

        def setCurrentIndex(self, index: int) -> None:
            calls["capture_duration_index"] = index

    class FakeCommandClient:
        def send_target(self, *_args: object, **_kwargs: object) -> None:
            raise AssertionError("exit before the first manual target renewal")

        def close(self) -> None:
            calls["command_client_closed"] = True

    class FakeDashboard:
        def __init__(self, **kwargs: object) -> None:
            calls["dashboard_init"] = kwargs
            self.linear_spin = SimpleNamespace(value=lambda: 0.4)
            self.angular_spin = SimpleNamespace(value=lambda: 0.8)
            self.capture_duration_combo = CaptureDurationCombo()

        def apply_window_rect(self, *_args: object, **_kwargs: object) -> None:
            return None

        def attach_v2_dashboard_widget(self, widget: object) -> None:
            calls["attached_v2_dashboard"] = widget

        def process_events(self) -> None:
            return None

        def current_command(self) -> object:
            return manual_demo.DashboardCommand(0.0, 0.0, should_exit=True)

        def request_capture_start(self) -> None:
            calls["capture_start_requested"] = True

        def close(self) -> None:
            calls["dashboard_closed"] = True

    class FakeCoordinator:
        def __init__(self, *args: object, **kwargs: object) -> None:
            calls["coordinator"] = (args, kwargs)
            self.world = world
            self.obstacle_manager = kwargs["interface_runtime"].obstacle_manager

    class FakeLogger:
        def __init__(self, *_args: object, **_kwargs: object) -> None:
            pass

        def close(self) -> str:
            return "manual.csv"

    class FakeObstacleLogger(FakeLogger):
        pass

    fake_qt = SimpleNamespace(QtWidgets=SimpleNamespace(QApplication=FakeQApplication))
    monkeypatch.setitem(sys.modules, "PySide6", fake_qt)
    monkeypatch.setattr(
        manual_demo,
        "primary_display_metrics",
        lambda: SimpleNamespace(device_pixel_ratio=1.0),
    )
    monkeypatch.setattr(manual_demo, "x11_available_geometry", lambda _metrics: object())
    monkeypatch.setattr(
        manual_demo,
        "calculate_window_layout",
        lambda _available, _dashboard_enabled: SimpleNamespace(main=object(), dashboard=object()),
    )
    monkeypatch.setattr(manual_demo, "align_window_layout_to_scale", lambda layout, _scale: layout)
    monkeypatch.setattr(manual_demo, "search_x11_window_ids", lambda *_args, **_kwargs: ())
    monkeypatch.setattr(manual_demo, "connect_pybullet_gui", lambda *_args, **_kwargs: client_id)
    monkeypatch.setattr(manual_demo, "apply_main_window_rect", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(manual_demo.p, "disconnect", lambda actual_client_id: calls.setdefault("disconnect", actual_client_id))
    monkeypatch.setattr(manual_demo.p, "addUserDebugParameter", lambda *_args: 1)
    monkeypatch.setattr(manual_demo.p, "readUserDebugParameter", lambda _slider: 0.0)
    monkeypatch.setattr(manual_demo.p, "getKeyboardEvents", lambda: {})
    monkeypatch.setattr(manual_demo, "build_world_from_scene_document", lambda *_args: (world, SimpleNamespace(snapshot=lambda **_kwargs: ("obstacle",))))
    monkeypatch.setattr(manual_demo, "PyBulletSensorBackend", FakeSensorBackend)
    monkeypatch.setattr(manual_demo, "V2ManualWorldRuntime", FakeV2ManualWorldRuntime)
    monkeypatch.setattr(manual_demo, "TelemetryDashboard", FakeDashboard)
    v2_dashboard = FakeV2Dashboard()
    monkeypatch.setattr(
        manual_demo,
        "_create_v2_dashboard_widget",
        lambda _runtime, **_kwargs: v2_dashboard,
    )
    monkeypatch.setattr(manual_demo, "SimulationCoordinator", FakeCoordinator)
    monkeypatch.setattr(manual_demo, "create_interface_session", lambda *_args, **_kwargs: (_ for _ in ()).throw(AssertionError("v1 interface session must not be created")))
    monkeypatch.setattr(manual_demo, "configure_gui_visualizer", lambda *_args: None)
    monkeypatch.setattr(manual_demo, "CsvSimulationLogger", FakeLogger)
    monkeypatch.setattr(manual_demo, "ObstacleEventLogger", FakeObstacleLogger)
    monkeypatch.setattr(manual_demo, "command_from_keyboard", lambda *_args: manual_demo.ManualCommand(0.0, 0.0, should_exit=True))
    monkeypatch.setattr(manual_demo.pd, "read_csv", lambda _path: object())
    monkeypatch.setattr(manual_demo, "compute_diagnostic_summary", lambda _frame: SimpleNamespace(to_dict=lambda: {}))
    monkeypatch.setattr(manual_demo, "write_diagnostic_summary", lambda *_args: "diagnostics.json")
    monkeypatch.setattr(manual_demo, "compute_tracking_metrics", lambda *_args, **_kwargs: {})
    monkeypatch.setattr(manual_demo, "plot_trajectory", lambda *_args, **_kwargs: "trajectory.png")
    monkeypatch.setattr(manual_demo, "plot_feedback_figures", lambda *_args, **_kwargs: [])

    config = ExperimentConfig(
        mode="gui",
        time_step=0.01,
        dashboard_enabled=True,
        interface_enabled=True,
        interface_mode="ecal",
    )
    session_id_factory = lambda: b"s" * 16
    command_client = FakeCommandClient()
    manual_demo.run_manual_demo(
        config,
        duration_limit_sec=0.01,
        v2_session_id_factory=session_id_factory,
        v2_command_client=command_client,
        v2_capture_release_root=tmp_path,
        v2_capture_output_root=(tmp_path / "captures").resolve(),
        v2_viewer_root=(tmp_path / "viewer").resolve(),
        v2_capture_duration_sec=90,
        v2_open_live_viewer=True,
    )

    runtime = calls["v2_init"]
    coordinator_args, coordinator_kwargs = calls["coordinator"]
    assert calls["backend"] == (client_id, robot.robot_id)
    assert runtime["config"] is config
    assert runtime["robot"] is robot
    assert runtime["session_id_factory"] is session_id_factory
    assert coordinator_args[:4] == (client_id, config, world, runtime["obstacle_manager"])
    assert coordinator_kwargs["interface_runtime"].__class__ is FakeV2ManualWorldRuntime
    assert calls["dashboard_init"]["show_lidar_tools"] is False
    assert calls["dashboard_init"]["v2_dashboard_enabled"] is True
    assert calls["attached_v2_dashboard"] is v2_dashboard
    assert calls["live_viewer_opened"] is True
    assert calls["capture_duration_requested"] == 90
    assert calls["capture_duration_index"] == 2
    assert calls["capture_start_requested"] is True
    assert calls["v2_refreshes"] == [
        coordinator_kwargs["interface_runtime"].dashboard_snapshot_store
    ]
    assert calls["dashboard_closed"] is True
    assert calls["v2_closed"] is True
    assert calls["command_client_closed"] is True
    assert calls["disconnect"] == client_id


def test_manual_demo_renews_v2_target_only_through_authenticated_client() -> None:
    """GUI 的速度意图只能交给 socket client，不能直接写入 eCAL。"""
    import slope_sim.manual_demo as manual_demo

    sent: list[tuple[float, float, float]] = []

    class Client:
        def send_target(self, linear: float, angular: float, *, now: float) -> None:
            sent.append((linear, angular, now))

    manual_demo._renew_v2_command_target(Client(), 0.6, -0.2, now=12.5)

    assert sent == [(0.6, -0.2, 12.5)]


def test_v2_manual_runtime_exposes_one_atomic_protocol_snapshot() -> None:
    """Recorder 编排只能读取同一时刻的 session/world identity。"""
    import slope_sim.interfaces.v2.world_runtime as world_runtime_module

    snapshot = object()

    class Controller:
        def snapshot(self) -> object:
            return snapshot

    runtime = object.__new__(world_runtime_module.V2ManualWorldRuntime)
    runtime._controller = Controller()

    assert runtime.protocol_snapshot() is snapshot


def test_manual_demo_starts_v2_capture_from_current_protocol_snapshot(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Dashboard start 必须启动 C++ Recorder，不再创建 Python 轨迹记录器。"""
    import slope_sim.manual_demo as manual_demo

    snapshot = object()
    calls: dict[str, object] = {}

    class Runtime:
        def protocol_snapshot(self) -> object:
            return snapshot

    class Recorder:
        def start(self) -> None:
            calls["started"] = True

    def launch(**kwargs: object) -> Recorder:
        calls.update(kwargs)
        return Recorder()

    monkeypatch.setattr(
        "slope_sim.interfaces.v2.runsim_v2_recorder.create_capture_output_dir",
        lambda _root: tmp_path / "capture-20260819-143012",
    )
    monkeypatch.setattr(
        "slope_sim.interfaces.v2.runsim_v2_recorder.RunSimV2Recorder.launch", launch,
    )

    recorder, output_dir = manual_demo._start_v2_capture(
        release_root=tmp_path / "release",
        runtime=Runtime(),
        output_root=tmp_path,
    )

    assert isinstance(recorder, Recorder)
    assert output_dir == tmp_path / "capture-20260819-143012"
    assert calls["snapshot"] is snapshot
    assert calls["scene_id"] == output_dir.name
    assert calls["output_dir"] == output_dir
    assert calls["started"] is True


def test_v2_manual_runtime_rebuild_and_obstacle_refresh_delegate_one_worker_transaction(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """车辆/场地重建与障碍物提交共用 worker 事务，刷新同时推进 generation。"""
    import slope_sim.interfaces.v2.world_runtime as world_runtime_module

    document = initial_scene_document(ExperimentConfig(mode="gui", interface_enabled=True))
    changed_document = document._replace_validated_runtime_obstacles(
        (
            ObstacleSpec(
                logical_id=1,
                mode="static",
                geometry=ObstacleGeometry("box", (0.3, 0.3, 0.3)),
                position=(2.0, 0.0, 0.3),
                orientation=(0.0, 0.0, 0.0, 1.0),
            ),
        )
    )
    events: list[tuple[object, ...]] = []

    class FakeRobot:
        model_spec = SimpleNamespace(name="df_mid")

    class FakeBackend:
        def bind_scene(self, terrain_body_ids: tuple[int, ...], snapshots: tuple[object, ...]) -> None:
            events.append(("bind", terrain_body_ids, snapshots))

    class FakeWorldRuntime:
        def __init__(self) -> None:
            self.scene_document = document

        def prepare_world_rebuild(self) -> None:
            events.append(("prepare",))

        def commit_world_rebuild(self, scene_document: object) -> None:
            events.append(("commit", scene_document))
            self.scene_document = scene_document

        def update_scene_document(self, scene_document: object) -> None:
            events.append(("invalidate_generation", scene_document))
            self.scene_document = scene_document

    monkeypatch.setattr(world_runtime_module, "DifferentialDriveRobot", FakeRobot)
    monkeypatch.setattr(world_runtime_module, "PyBulletSensorBackend", FakeBackend)
    runtime = object.__new__(world_runtime_module.V2ManualWorldRuntime)
    runtime._world_runtime = FakeWorldRuntime()
    runtime._robot = FakeRobot()
    runtime._sensor_backend = FakeBackend()
    runtime._runtime = object()
    monkeypatch.setattr(runtime, "_make_runtime", lambda: object())

    runtime.prepare_world_rebuild()
    runtime.commit_world_rebuild(FakeRobot(), FakeBackend(), document)
    runtime.refresh_scene_bindings((11,), ("new-obstacle",), changed_document)

    assert events == [
        ("prepare",),
        ("commit", document),
        ("bind", (11,), ("new-obstacle",)),
        ("invalidate_generation", changed_document),
    ]


def test_v2_manual_runtime_exposes_one_snapshot_store_to_its_simulator(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """手动 GUI 与物理 runtime 必须共享同一份不可变 v2 Dashboard store。"""
    import slope_sim.interfaces.v2.world_runtime as world_runtime_module

    captured: dict[str, object] = {}

    class FakeSensorFrames:
        def __init__(self, *_args: object) -> None:
            pass

    class FakePublisher:
        def __init__(self, *_args: object) -> None:
            pass

    class FakeWheelStateFactory:
        def __init__(self, *_args: object) -> None:
            pass

    class FakeTruthSuite:
        def __init__(self, *_args: object) -> None:
            pass

    class FakeMounts:
        @staticmethod
        def default() -> object:
            return object()

    class FakeSimulator:
        def __init__(self, **kwargs: object) -> None:
            captured.update(kwargs)

    store = object()
    descriptor = object()
    runtime = object.__new__(world_runtime_module.V2ManualWorldRuntime)
    runtime._dashboard_snapshot_store = store
    runtime._descriptor = descriptor
    runtime._controller = object()
    runtime._robot = SimpleNamespace(
        read_interface_wheel_state=object(),
        model_spec=SimpleNamespace(name="df_mid"),
    )
    runtime._world_runtime = SimpleNamespace(worker=object())
    runtime._transport = object()
    runtime._sensor_backend = object()
    runtime._capture_context = lambda: ()

    monkeypatch.setattr(world_runtime_module, "V2AsyncSensorFrameFactory", FakeSensorFrames)
    monkeypatch.setattr(world_runtime_module, "V2OutputFramePublisher", FakePublisher)
    monkeypatch.setattr(world_runtime_module, "V2WheelStateFactory", FakeWheelStateFactory)
    monkeypatch.setattr(world_runtime_module, "Stage4TruthSensorSuite", FakeTruthSuite)
    monkeypatch.setattr(world_runtime_module, "Stage4SensorMounts", FakeMounts)
    monkeypatch.setattr(world_runtime_module, "V2SimulatorRuntime", FakeSimulator)

    assert runtime.dashboard_snapshot_store is store
    assert runtime.descriptor is descriptor
    runtime._make_runtime()
    assert captured["dashboard_snapshot_store"] is store


def test_v2_manual_runtime_replaces_dashboard_store_after_world_generation_change(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """world 重建后 GUI 绝不能把旧 generation 与新帧混进同一 snapshot store。"""
    import slope_sim.interfaces.v2.world_runtime as world_runtime_module

    document = initial_scene_document(ExperimentConfig(mode="gui", interface_enabled=True))

    class FakeRobot:
        model_spec = SimpleNamespace(name="df_mid")

    class FakeBackend:
        pass

    class FakeWorldRuntime:
        def commit_world_rebuild(self, committed: object) -> None:
            assert committed is document

    monkeypatch.setattr(world_runtime_module, "DifferentialDriveRobot", FakeRobot)
    monkeypatch.setattr(world_runtime_module, "PyBulletSensorBackend", FakeBackend)
    runtime = object.__new__(world_runtime_module.V2ManualWorldRuntime)
    runtime._world_runtime = FakeWorldRuntime()
    runtime._robot = FakeRobot()
    runtime._sensor_backend = FakeBackend()
    runtime._dashboard_snapshot_store = (
        world_runtime_module.V2DashboardSnapshotStore(robot_model="df_mid")
    )
    runtime._make_runtime = lambda: object()
    previous = runtime.dashboard_snapshot_store

    runtime.commit_world_rebuild(FakeRobot(), FakeBackend(), document)

    assert runtime.dashboard_snapshot_store is not previous
