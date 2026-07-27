# InterfaceRuntime 集成测试：覆盖六话题、命令回调、本地 twist、传感器和日志。
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import json
from threading import Event, Lock, Thread
import time
from types import SimpleNamespace

import pytest

import slope_sim.interfaces.runtime as runtime_module
import slope_sim.simulation as simulation_module
from slope_sim.config import ExperimentConfig
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.ecal_transport import EcalUnavailableError
from slope_sim.interfaces.dashboard_snapshot import LidarTopViewFrame
from slope_sim.interfaces.logging import InterfaceEventLogger, InterfaceLogRecord
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.interfaces.transport import TransportSnapshot, TransportTopicQuality
from slope_sim.lidar_pointcloud import LidarScanResult, MultiLineLidar
from slope_sim.model_registry import get_robot_model
from slope_sim.obstacles import ObstacleGeometry, ObstacleSnapshot, ObstacleSpec
from slope_sim.scene_config import SceneDocument, SensorDocument, TerrainDocument
from slope_sim.truth_sensors import SensorMounts, TruthSensorSuite


class Clock:
    """可显式推进的单调墙钟。"""

    def __init__(self, value: float = 0.0) -> None:
        self.value = value

    def __call__(self) -> float:
        return self.value

    def advance(self, dt: float) -> float:
        self.value += dt
        return self.value


class Subscription:
    """记录关闭并支持测试直接触发回调。"""

    def __init__(self, callback, trace: list[str]) -> None:
        self.callback = callback
        self.trace = trace
        self.closed = False
        self.close_error: BaseException | None = None

    def close(self) -> None:
        self.trace.append("subscription.close")
        self.closed = True
        if self.close_error is not None:
            raise self.close_error


class Transport:
    """同步交付命令话题且可按话题注入发布结果。"""

    def __init__(self, *, mode: str = "local") -> None:
        self.mode = mode
        self.trace: list[str] = []
        self.subscriptions: list[tuple[str, str, Subscription]] = []
        self.published: list[tuple[str, bytes, str, int, float | None]] = []
        self.outcomes: dict[str, deque[object]] = {}
        self.poll_count = 0
        self.quiesce_count = 0
        self.close_count = 0

    def subscribe(self, topic: str, type_name: str, callback) -> Subscription:
        subscription = Subscription(callback, self.trace)
        self.subscriptions.append((topic, type_name, subscription))
        return subscription

    def emit(self, topic: str, payload: bytes, received_at: float) -> object:
        result = None
        for bound_topic, _type_name, subscription in tuple(self.subscriptions):
            if bound_topic == topic and not subscription.closed:
                result = subscription.callback(payload, received_at)
        return result

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float | None = None,
    ) -> bool:
        self.published.append((topic, bytes(payload), type_name, sim_time_ns, wall_time))
        queue = self.outcomes.get(topic)
        outcome = queue.popleft() if queue else True
        if isinstance(outcome, BaseException):
            raise outcome
        if outcome is not True:
            return False
        self.emit(topic, payload, 0.0 if wall_time is None else wall_time)
        return True

    def poll_peer_state(self) -> str:
        self.poll_count += 1
        return "active"

    def snapshot(self) -> TransportSnapshot:
        return TransportSnapshot(
            mode=self.mode,
            ecal_connected=self.mode == "ecal",
            published_count=len(self.published),
            received_count=0,
            error_count=0,
            dropped_count=0,
        )

    def quiesce(self) -> TransportSnapshot:
        """模拟 transport 停止新交付后的最终质量快照。"""
        self.quiesce_count += 1
        return self.snapshot()

    def close(self) -> None:
        self.trace.append("transport.close")
        self.close_count += 1


class Robot:
    """实现运行时需要的轮子端口，并记录纯 twist 转换调用。"""

    def __init__(self, robot_id: int = 1) -> None:
        self.robot_id = robot_id
        self.model_spec = get_robot_model("df_mid")
        self.commands: list[tuple[tuple[float, ...], tuple[float, ...], float]] = []
        self.twists: list[tuple[float, float, int, float]] = []
        self.safe_stops = 0

    def command_wheel_speeds(self, drive, steering=(), dt=1.0 / 240.0):
        self.commands.append((tuple(drive), tuple(steering), dt))
        return tuple(drive)

    def hold_current_steering_and_stop_drive(self, _dt: float) -> None:
        self.safe_stops += 1

    def read_interface_wheel_state(self, timestamp_ns: int) -> WheelState:
        return WheelState(timestamp_ns, (1.0, 2.0), ())

    def wheel_command_from_twist(
        self,
        linear: float,
        angular: float,
        timestamp_ns: int,
        dt: float,
    ) -> WheelCommand:
        self.twists.append((linear, angular, timestamp_ns, dt))
        return WheelCommand(timestamp_ns, (linear - angular, linear + angular), ())


class Backend:
    """只在构造阶段提供语义 link，并记录动态场景绑定。"""

    def __init__(self, trace: list[str] | None = None) -> None:
        self.trace = [] if trace is None else trace
        self.bind_calls: list[tuple[tuple[int, ...], tuple[object, ...]]] = []

    def link_names(self):
        return ("base_link", "lidar_front_mount", "lidar_rear_mount")

    def bind_scene(self, terrain_ids, snapshots) -> None:
        self.bind_calls.append((tuple(terrain_ids), tuple(snapshots)))

    def close(self) -> None:
        self.trace.append("backend.close")


class Logger:
    """同步收集运行时日志，并可拒绝消息入队。"""

    def __init__(self, *, accept_messages: bool = True, trace: list[str] | None = None) -> None:
        self.accept_messages = accept_messages
        self.messages: list[InterfaceLogRecord] = []
        self.events: list[tuple[str, dict[str, object]]] = []
        self.trace = [] if trace is None else trace

    def record_message(self, record: InterfaceLogRecord) -> bool:
        self.messages.append(record)
        return self.accept_messages

    def record_event(self, event: str, **fields: object) -> bool:
        self.events.append((event, fields))
        return True

    def record_terminal_event(
        self,
        event: str,
        *,
        timeout_sec: float = 1.0,
        **fields: object,
    ) -> bool:
        """fake 终态入口复用同步事件收集，不模拟容量等待。"""
        assert timeout_sec >= 0.0
        return self.record_event(event, **fields)

    def close(self) -> None:
        self.trace.append("logger.close")


class RecoveringQueueWriter:
    """阻塞首批写入，并在指定数量完成后暴露容量已恢复。"""

    def __init__(self, drain_count: int) -> None:
        self.drain_count = drain_count
        self.started = Event()
        self.release = Event()
        self.drained = Event()
        self._lock = Lock()
        self._completed = 0

    def __call__(self, stream, data):
        self.started.set()
        if not self.release.wait(timeout=5.0):
            raise TimeoutError("test logger writer was not released")
        written = stream.write(data)
        with self._lock:
            self._completed += 1
            if self._completed >= self.drain_count:
                self.drained.set()
        return written


class SlowSuccessfulWriter:
    """让关闭期终态事件必须短暂等待容量，但最终可以落盘。"""

    def __init__(self, delay_sec: float) -> None:
        self.delay_sec = delay_sec
        self.started = Event()

    def __call__(self, stream, data):
        self.started.set()
        time.sleep(self.delay_sec)
        return stream.write(data)


class SingleCapacityOrderingWriter:
    """分段释放唯一容量，并阻塞普通恢复事件以锁定提交先后。"""

    def __init__(self) -> None:
        self.initial_started = Event()
        self.release_initial = Event()
        self.recovery_started = Event()
        self.release_recovery = Event()
        self._lock = Lock()
        self._events: list[dict[str, object]] = []

    def __call__(self, stream, data):
        if isinstance(data, str):
            event = json.loads(data)
            reason = event.get("reason")
            if reason == "occupy capacity":
                self.initial_started.set()
                if not self.release_initial.wait(timeout=5.0):
                    raise TimeoutError("initial logger write was not released")
            elif reason == "capacity recovered":
                self.recovery_started.set()
                if not self.release_recovery.wait(timeout=5.0):
                    raise TimeoutError("recovery logger write was not released")
            written = stream.write(data)
            with self._lock:
                self._events.append(event)
            return written
        return stream.write(data)

    def wait_for_event(self, event_name: str, **fields: object) -> bool:
        """等待 writer 实际完成指定事件，避免依赖线程调度时序。"""
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with self._lock:
                if any(
                    event.get("event") == event_name
                    and all(event.get(name) == value for name, value in fields.items())
                    for event in self._events
                ):
                    return True
            time.sleep(0.005)
        return False


class InitCleanupTransport(Transport):
    """为构造事务注入 subscribe 与 close 异常，并记录资源释放顺序。"""

    def __init__(
        self,
        trace: list[str],
        *,
        subscribe_error: BaseException | None = None,
    ) -> None:
        super().__init__()
        self.cleanup_trace = trace
        self.subscribe_error = subscribe_error

    def subscribe(self, topic: str, type_name: str, callback) -> Subscription:
        if self.subscribe_error is not None:
            raise self.subscribe_error
        return super().subscribe(topic, type_name, callback)

    def close(self) -> None:
        self.cleanup_trace.append("close_transport")
        raise RuntimeError("transport cleanup failed")


class InitCleanupLogger(Logger):
    """构造失败清理 logger；close 异常不得截断后续资源释放。"""

    def __init__(self, trace: list[str]) -> None:
        super().__init__()
        self.cleanup_trace = trace

    def close(self) -> None:
        self.cleanup_trace.append("close_log")
        raise RuntimeError("logger cleanup failed")


class InitCleanupBackend(Backend):
    """可让传感器构造失败，并记录最终 backend 释放。"""

    def __init__(self, trace: list[str], *, fail_build: bool) -> None:
        super().__init__()
        self.cleanup_trace = trace
        self.fail_build = fail_build

    def link_names(self):
        if self.fail_build:
            return ("base_link",)
        return super().link_names()

    def close(self) -> None:
        self.cleanup_trace.append("close_sensors")
        raise RuntimeError("sensor cleanup failed")


class OrderedBlockingLogger(Logger):
    """阻塞首条记录，暴露并发调用是否按 sequence 串行提交。"""

    def __init__(self) -> None:
        super().__init__()
        self.first_entered = Event()
        self.release_first = Event()
        self._append_lock = Lock()

    def record_message(self, record: InterfaceLogRecord) -> bool:
        if record.sequence == 0:
            self.first_entered.set()
            assert self.release_first.wait(timeout=3.0)
        with self._append_lock:
            self.messages.append(record)
        return True


class RejectingEventLogger(Logger):
    """保留事件内容但拒绝入队，用于验证 topic 降级统计。"""

    def record_event(self, event: str, **fields: object) -> bool:
        self.events.append((event, fields))
        return False


class BlockingCodec(ProtoCodec):
    """让命令解析停在生命周期锁外，复现迟到旧 payload。"""

    def __init__(self) -> None:
        self.entered = Event()
        self.release = Event()

    def decode_wheel_command(self, payload: object) -> WheelCommand:
        self.entered.set()
        assert self.release.wait(timeout=3.0)
        return super().decode_wheel_command(payload)


@dataclass
class StubLidar:
    """返回空点云或抛出单路扫描错误。"""

    frame_id: str
    lidar_id: int
    error: Exception | None = None

    def scan_with_top_view(self, timestamp_ns: int) -> LidarScanResult:
        if self.error is not None:
            raise self.error
        message = LidarPointCloud(timestamp_ns, self.frame_id, 0, self.lidar_id, ())
        return LidarScanResult(message, LidarTopViewFrame(timestamp_ns, ()))

    def scan(self, timestamp_ns: int) -> LidarPointCloud:
        return self.scan_with_top_view(timestamp_ns).message


class StubTruth:
    """生成确定性 RTK/IMU 真值模型。"""

    def read_rtk(self, timestamp_ns: int) -> RtkState:
        return RtkState(timestamp_ns, 1.0, 2.0, 3.0, 0.25)

    def read_imu(self, timestamp_ns: int) -> ImuAttitude:
        return ImuAttitude(timestamp_ns, 0.1, -0.2)


def scene_document() -> SceneDocument:
    """构造不含动态 body id 的绑定配置。"""
    return SceneDocument(
        1,
        "df_mid",
        TerrainDocument("flat", 0.0, 0, "low"),
        (),
        SensorDocument.default(),
    )


def make_runtime(
    *,
    mode: str = "local",
    backend: Backend | None = None,
    document: SceneDocument | None = None,
    logger: Logger | None = None,
    transport: Transport | None = None,
):
    from slope_sim.interfaces.runtime import InterfaceRuntime

    clock = Clock()
    robot = Robot()
    selected_transport = Transport(mode=mode) if transport is None else transport
    runtime = InterfaceRuntime(
        robot,
        config=InterfaceConfig.default(transport_mode=mode),
        transport=selected_transport,
        monotonic=clock,
        sensor_backend=backend,
        scene_document=document,
        logger=logger,
    )
    return runtime, robot, selected_transport, clock


@pytest.mark.parametrize("failure_stage", ("sensor_build", "subscribe"))
def test_constructor_failure_closes_all_owned_resources_and_preserves_original_error(
    failure_stage: str,
) -> None:
    from slope_sim.interfaces.runtime import InterfaceRuntime

    trace: list[str] = []
    robot = Robot()
    backend = InitCleanupBackend(trace, fail_build=failure_stage == "sensor_build")
    subscribe_error = (
        RuntimeError("original subscribe failure")
        if failure_stage == "subscribe"
        else None
    )
    transport = InitCleanupTransport(trace, subscribe_error=subscribe_error)
    logger = InitCleanupLogger(trace)
    expected_message = (
        "lidar parent link" if failure_stage == "sensor_build" else "original subscribe failure"
    )

    with pytest.raises((ValueError, RuntimeError), match=expected_message):
        InterfaceRuntime(
            robot,
            config=InterfaceConfig.default(transport_mode="local"),
            transport=transport,
            monotonic=Clock(),
            sensor_backend=backend,
            scene_document=scene_document(),
            logger=logger,
        )

    assert trace == ["close_log", "close_transport", "close_sensors"]
    assert robot.safe_stops == 0


def test_entrypoint_actual_mode_snapshot_failure_closes_constructed_runtime(
    monkeypatch,
    tmp_path,
) -> None:
    """runtime 成功后读取实际模式失败，也必须只经 runtime 所有权链完成清理。"""
    trace: list[str] = []
    robot = Robot()
    world = SimpleNamespace(
        scene=SimpleNamespace(body_ids=(10,)),
        active_robot=SimpleNamespace(robot=robot),
    )

    class Manager:
        def snapshot(self, *, include_body_id=False):
            return ()

    class Backend:
        def __init__(self, *_args):
            trace.append("backend")

        def bind_scene(self, *_args):
            trace.append("backend.bind")

        def close(self):
            trace.append("backend.close")

    class SnapshotFailTransport:
        def snapshot(self):
            trace.append("transport.snapshot")
            raise RuntimeError("actual mode snapshot failed")

        def close(self):
            trace.append("transport.close")

    class EntryLogger:
        def __init__(self, *_args, **_kwargs):
            trace.append("logger")

        def close(self):
            trace.append("logger.close")

    class ConstructedRuntime:
        def __init__(self, _robot, *, transport, sensor_backend, logger, **_kwargs):
            trace.append("runtime")
            self.transport = transport
            self.backend = sensor_backend
            self.logger = logger

        def close(self):
            trace.append("runtime.close")
            self.logger.close()
            self.transport.close()
            self.backend.close()

    transport = SnapshotFailTransport()
    monkeypatch.setattr(simulation_module, "PyBulletSensorBackend", Backend)
    monkeypatch.setattr(simulation_module, "create_transport", lambda *_args, **_kwargs: transport)
    monkeypatch.setattr(simulation_module, "InterfaceEventLogger", EntryLogger)
    monkeypatch.setattr(simulation_module, "InterfaceRuntime", ConstructedRuntime)

    with pytest.raises(RuntimeError, match="actual mode snapshot failed"):
        simulation_module.create_interface_session(
            ExperimentConfig(
                interface_mode="local",
                interface_enabled=True,
                interface_log_enabled=True,
                log_dir=tmp_path,
            ),
            client_id=3,
            coordinator_world=world,
            obstacle_manager=Manager(),
            document=scene_document(),
        )

    assert trace[-4:] == [
        "runtime.close",
        "logger.close",
        "transport.close",
        "backend.close",
    ]


def test_entrypoint_backend_bind_failure_closes_backend_and_preserves_primary_error(
    monkeypatch,
) -> None:
    """backend 已构造但场景绑定失败时，必须回收 backend 且不启动后续接口资源。"""
    trace: list[str] = []
    world = SimpleNamespace(
        scene=SimpleNamespace(body_ids=(10, 11)),
        active_robot=SimpleNamespace(robot=Robot()),
    )

    class Manager:
        def snapshot(self, *, include_body_id=False):
            if include_body_id:
                return (SimpleNamespace(body_id=12, mode="static"),)
            return ()

    class BindFailBackend:
        def __init__(self, *_args):
            trace.append("backend")

        def bind_scene(self, terrain_ids, snapshots):
            trace.append(("bind", tuple(terrain_ids), tuple(snapshots)))
            raise RuntimeError("primary backend bind failure")

        def close(self):
            trace.append("backend.close")
            raise RuntimeError("secondary backend close failure")

    def reject_transport(*_args, **_kwargs):
        trace.append("transport")
        raise AssertionError("transport must not start after backend bind failure")

    monkeypatch.setattr(simulation_module, "PyBulletSensorBackend", BindFailBackend)
    monkeypatch.setattr(simulation_module, "create_transport", reject_transport)

    with pytest.raises(RuntimeError, match="primary backend bind failure"):
        simulation_module.create_interface_session(
            ExperimentConfig(
                interface_mode="local",
                interface_enabled=True,
                interface_log_enabled=False,
            ),
            client_id=3,
            coordinator_world=world,
            obstacle_manager=Manager(),
            document=scene_document(),
        )

    assert trace[0] == "backend"
    assert trace[-1] == "backend.close"
    assert "transport" not in trace


def test_production_session_keeps_complete_scene_document_without_body_ids(monkeypatch) -> None:
    logical_obstacle = ObstacleSpec(
        logical_id=7,
        mode="static",
        geometry=ObstacleGeometry("box", (0.2, 0.3, 0.4)),
        position=(1.0, 2.0, 0.4),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    document = replace(scene_document(), obstacles=(logical_obstacle,))
    physical_snapshot = ObstacleSnapshot(
        logical_id=7,
        body_id=12,
        mode=logical_obstacle.mode,
        shape=logical_obstacle.geometry.shape,
        position=logical_obstacle.position,
        orientation=logical_obstacle.orientation,
        geometry=logical_obstacle.geometry,
    )
    world = SimpleNamespace(
        scene=SimpleNamespace(body_ids=(10, 11)),
        active_robot=SimpleNamespace(robot=Robot()),
    )

    class Manager:
        def snapshot(self, *, include_body_id=False):
            return (
                physical_snapshot
                if include_body_id
                else replace(physical_snapshot, body_id=None),
            )

    backend = Backend()
    monkeypatch.setattr(simulation_module, "PyBulletSensorBackend", lambda *_args: backend)

    session = simulation_module.create_interface_session(
        ExperimentConfig(
            interface_mode="local",
            interface_enabled=True,
            interface_log_enabled=False,
        ),
        client_id=3,
        coordinator_world=world,
        obstacle_manager=Manager(),
        document=document,
    )
    assert session is not None
    try:
        assert session.runtime.scene_document == document
        assert all(
            not hasattr(obstacle, "body_id")
            for obstacle in session.runtime.scene_document.obstacles
        )
        assert backend.bind_calls == [((10, 11), (physical_snapshot,))]
    finally:
        session.close()


def test_production_session_wires_peer_lifecycle_to_runtime_generation(monkeypatch) -> None:
    """session 必须把真实 discovery 边沿接到 runtime，而非只更新 transport。"""
    transport = Transport(mode="ecal")
    peer_callback = None
    peer_state = "waiting_peer"

    def snapshot() -> TransportSnapshot:
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=peer_state != "disconnected",
            published_count=len(transport.published),
            received_count=0,
            error_count=0,
            dropped_count=0,
        )

    transport.snapshot = snapshot

    def create_peer_transport(_mode, *, config, peer_state_callback):
        nonlocal peer_callback
        assert config.transport_mode == "ecal"
        peer_callback = peer_state_callback
        return transport

    def transition(state: str) -> None:
        nonlocal peer_state
        peer_state = state
        assert peer_callback is not None
        peer_callback(state)

    world = SimpleNamespace(
        scene=SimpleNamespace(body_ids=(10,)),
        active_robot=SimpleNamespace(robot=Robot()),
    )

    class Manager:
        def snapshot(self, *, include_body_id=False):
            return ()

    monkeypatch.setattr(simulation_module, "PyBulletSensorBackend", lambda *_args: Backend())
    monkeypatch.setattr(simulation_module, "create_transport", create_peer_transport)

    session = simulation_module.create_interface_session(
        ExperimentConfig(
            interface_mode="ecal",
            interface_enabled=True,
            interface_log_enabled=False,
        ),
        client_id=3,
        coordinator_world=world,
        obstacle_manager=Manager(),
        document=scene_document(),
    )
    assert session is not None
    runtime = session.runtime
    topic = runtime.config.wheel_command.topic
    codec = ProtoCodec()
    try:
        received_at = runtime._monotonic()
        assert transport.emit(
            topic,
            codec.encode(WheelCommand(1, (3.0, 4.0), ())),
            received_at,
        ) is True
        transition("active")
        old_mailbox, old_generation = runtime.capture_command_ingress()
        assert runtime.status_snapshot(wall_time=received_at).command.state == "active"

        transition("disconnected")
        disconnected = runtime.status_snapshot(wall_time=received_at)
        assert disconnected.command.state == "disconnected"
        assert disconnected.topics[topic].state == "disconnected"
        assert not old_mailbox.accept(
            WheelCommand(2, (8.0, 9.0), ()),
            received_at=received_at,
            generation=old_generation,
        )

        transition("waiting_peer")
        transition("active")
        waiting = runtime.status_snapshot(wall_time=received_at)
        assert waiting.command.state == "waiting_command"
        assert waiting.topics[topic].state == "waiting_peer"

        assert transport.emit(
            topic,
            codec.encode(WheelCommand(3, (5.0, 6.0), ())),
            received_at,
        ) is True
        transition("active")
        restored = runtime.status_snapshot(wall_time=received_at)
        assert restored.command.state == "active"
        assert restored.topics[topic].state == "active"

        runtime.prepare_world_rebuild()
        runtime.commit_world_rebuild(Robot(2), Backend(), scene_document())
        rebuilt_at = runtime._monotonic()
        rebuilt = runtime.status_snapshot(wall_time=rebuilt_at)
        assert rebuilt.command.state == "waiting_command"
        assert rebuilt.topics[topic].state == "waiting_peer"
        transition("active")
        assert runtime.status_snapshot(wall_time=rebuilt_at).topics[topic].state == "waiting_peer"

        assert transport.emit(
            topic,
            codec.encode(WheelCommand(4, (7.0, 8.0), ())),
            rebuilt_at,
        ) is True
        transition("active")
        assert runtime.status_snapshot(wall_time=rebuilt_at).topics[topic].state == "active"
    finally:
        session.close()


def test_production_auto_fallback_logs_unavailable_once_but_explicit_local_does_not(
    monkeypatch,
) -> None:
    reason = "modern import failed; legacy import failed"
    loggers = deque((Logger(), Logger()))
    world = SimpleNamespace(
        scene=SimpleNamespace(body_ids=(10,)),
        active_robot=SimpleNamespace(robot=Robot()),
    )

    class Manager:
        def snapshot(self, *, include_body_id=False):
            return ()

    def unavailable():
        raise EcalUnavailableError(reason)

    monkeypatch.setattr(
        "slope_sim.interfaces.ecal_transport.load_ecal_bindings",
        unavailable,
    )
    monkeypatch.setattr(
        simulation_module,
        "PyBulletSensorBackend",
        lambda *_args: Backend(),
    )
    monkeypatch.setattr(
        simulation_module,
        "InterfaceEventLogger",
        lambda *_args, **_kwargs: loggers.popleft(),
    )

    auto_logger = loggers[0]
    auto_session = simulation_module.create_interface_session(
        ExperimentConfig(
            interface_mode="auto",
            interface_enabled=True,
            interface_log_enabled=True,
        ),
        client_id=3,
        coordinator_world=world,
        obstacle_manager=Manager(),
        document=scene_document(),
    )
    assert auto_session is not None
    try:
        assert auto_session.actual_transport_mode == "local"
        assert [event for event, _fields in auto_logger.events] == [
            "ecal_disconnected"
        ]
        assert auto_session.runtime._transport.snapshot().detail == (
            f"EcalUnavailableError: {reason}"
        )
        fallback_status = auto_session.runtime.status_snapshot()
        assert all(
            status.detail == f"EcalUnavailableError: {reason}"
            for status in fallback_status.topics.values()
        )
        fields = auto_logger.events[0][1]
        assert isinstance(fields["wall_time_ns"], int)
        assert fields["sim_time_ns"] == 0
        assert fields["robot_model"] == "df_mid"
        assert fields["terrain_model"] == "flat"
        assert fields["topic"] == auto_session.runtime.config.wheel_command.topic
        assert fields["reason"] == f"EcalUnavailableError: {reason}"
    finally:
        auto_session.close()

    local_logger = loggers[0]
    local_session = simulation_module.create_interface_session(
        ExperimentConfig(
            interface_mode="local",
            interface_enabled=True,
            interface_log_enabled=True,
        ),
        client_id=3,
        coordinator_world=world,
        obstacle_manager=Manager(),
        document=scene_document(),
    )
    assert local_session is not None
    try:
        assert local_session.actual_transport_mode == "local"
        assert local_logger.events == []
    finally:
        local_session.close()


@pytest.mark.parametrize("failure_stage", ("scheduler", "initial_monotonic"))
def test_early_constructor_failure_uses_same_owned_resource_cleanup_transaction(
    failure_stage: str,
) -> None:
    from slope_sim.interfaces.runtime import InterfaceRuntime

    trace: list[str] = []
    robot = Robot()
    backend = InitCleanupBackend(trace, fail_build=False)
    transport = InitCleanupTransport(trace)
    logger = InitCleanupLogger(trace)
    base_config = InterfaceConfig.default(transport_mode="local")
    config = (
        replace(
            base_config,
            wheel_state=replace(
                base_config.wheel_state,
                rate_hz=1_000_000_001,
            ),
        )
        if failure_stage == "scheduler"
        else base_config
    )

    def monotonic() -> float:
        if failure_stage == "initial_monotonic":
            raise RuntimeError("original initial monotonic failure")
        return 0.0

    expected_message = (
        "rate_hz" if failure_stage == "scheduler" else "original initial monotonic failure"
    )
    with pytest.raises((ValueError, RuntimeError), match=expected_message):
        InterfaceRuntime(
            robot,
            config=config,
            transport=transport,
            monotonic=monotonic,
            sensor_backend=backend,
            scene_document=scene_document(),
            logger=logger,
        )

    assert trace == ["close_log", "close_transport", "close_sensors"]
    assert robot.safe_stops == 0


def test_delayed_subscription_payload_cannot_enter_rebuilt_mailbox() -> None:
    runtime, _robot, transport, clock = make_runtime()
    codec = BlockingCodec()
    runtime._codec = codec
    payload = ProtoCodec().encode(WheelCommand(7, (3.0, 4.0), ()))
    result: list[object] = []
    worker = Thread(
        target=lambda: result.append(
            transport.emit(runtime.config.wheel_command.topic, payload, clock())
        ),
        daemon=True,
    )
    try:
        worker.start()
        assert codec.entered.wait(timeout=2.0)
        runtime.prepare_world_rebuild()
        runtime.commit_world_rebuild(Robot(2), Backend(), scene_document())
        codec.release.set()
        worker.join(timeout=2.0)

        assert not worker.is_alive()
        assert result == [None]
        assert runtime.status_snapshot(wall_time=clock()).command.state == "waiting_command"
        assert runtime.bound_robot_id == 2
    finally:
        codec.release.set()
        worker.join(timeout=2.0)
        runtime.close()


def test_local_twist_uses_codec_transport_subscription_and_100hz_deadline() -> None:
    runtime, robot, transport, clock = make_runtime()
    try:
        assert runtime.submit_local_twist(2.0, 0.5, 0.01)
        runtime.after_physics_step(0.01)
        decision = runtime.before_physics_step(0.01, wall_time=clock())

        command_messages = [
            item for item in transport.published if item[0] == runtime.config.wheel_command.topic
        ]
        assert robot.twists == [(2.0, 0.5, 10_000_000, 0.01)]
        assert len(command_messages) == 1
        assert ProtoCodec().decode_wheel_command(command_messages[0][1]) == WheelCommand(
            10_000_000, (1.5, 2.5), ()
        )
        assert runtime.status_snapshot(wall_time=clock()).command.valid_count == 1
        assert decision is not None and decision.drive_wheel_speed_rad_s == (1.5, 2.5)
    finally:
        runtime.close()


def test_local_interface_frame_uses_canonical_runtime_order_without_twist_bypass() -> None:
    """入口单帧必须让本地目标经过 transport/mailbox 后再推进唯一物理步。"""
    runtime, robot, transport, clock = make_runtime()
    trace: list[str] = []
    original_poll = runtime.poll_transport
    original_submit = runtime.submit_local_twist
    original_before = runtime.before_physics_step
    original_after = runtime.after_physics_step
    direct_twists: list[tuple[float, float, float]] = []

    runtime.poll_transport = lambda: (trace.append("poll"), original_poll())[1]
    runtime.submit_local_twist = lambda linear, angular, dt: (
        trace.append("submit_local"),
        original_submit(linear, angular, dt),
    )[1]
    runtime.before_physics_step = lambda dt: (
        trace.append("before"),
        original_before(dt, wall_time=clock()),
    )[1]
    runtime.after_physics_step = lambda dt: (
        trace.append("after"),
        original_after(dt),
    )[1]
    robot.command_twist = lambda linear, angular, *, dt: direct_twists.append(
        (linear, angular, dt)
    )

    class Coordinator:
        def step(self, dt: float) -> str:
            trace.append("coordinator.step")
            clock.advance(dt)
            return "advanced"

    try:
        result = simulation_module.run_interface_physics_frame(
            runtime,
            Coordinator(),
            actual_transport_mode="local",
            linear_velocity=0.6,
            angular_velocity=0.2,
            dt=0.01,
        )
        simulation_module.run_interface_physics_frame(
            runtime,
            Coordinator(),
            actual_transport_mode="local",
            linear_velocity=0.6,
            angular_velocity=0.2,
            dt=0.01,
        )

        assert result == "advanced"
        assert trace == [
            "poll",
            "submit_local",
            "before",
            "coordinator.step",
            "after",
        ] * 2
        assert transport.poll_count == 2
        assert robot.twists == [(0.6, 0.2, 10_000_000, 0.01)]
        assert robot.commands[-1][0] == pytest.approx((0.4, 0.8))
        assert direct_twists == []
    finally:
        runtime.close()


def test_local_twist_logs_only_canonical_command_receive_record() -> None:
    logger = Logger()
    runtime, _robot, _transport, clock = make_runtime(logger=logger)
    command_topic = runtime.config.wheel_command.topic
    try:
        assert runtime.submit_local_twist(2.0, 0.5, 0.01)
        clock.advance(0.01)
        runtime.after_physics_step(0.01)
        runtime.before_physics_step(0.01, wall_time=clock())

        command_records = [
            record for record in logger.messages if record.topic == command_topic
        ]
        assert len(command_records) == 1
        assert command_records[0].direction == "receive"
        assert command_records[0].sim_time_ns == 10_000_000
    finally:
        runtime.close()


def test_ecal_ignores_local_twist_and_polling_reports_actual_transport() -> None:
    runtime, robot, transport, _clock = make_runtime(mode="ecal")
    try:
        assert runtime.submit_local_twist(1.0, 0.2, 0.01) is False
        assert runtime.poll_transport() == "active"
        assert runtime.poll_transport() == "active"
        assert runtime.connection_polls == transport.poll_count == 2
        assert runtime.status_snapshot().transport_mode == "ecal"
        assert robot.twists == []
    finally:
        runtime.close()


def test_poll_transport_consumes_per_topic_quality_and_ignores_stale_revision() -> None:
    logger = Logger()
    transport = Transport(mode="ecal")
    quality: tuple[TransportTopicQuality, ...] = ()

    def snapshot() -> TransportSnapshot:
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=True,
            published_count=len(transport.published),
            received_count=0,
            error_count=sum(item.error_count for item in quality),
            dropped_count=sum(item.dropped_count for item in quality),
            topic_quality=quality,
        )

    transport.snapshot = snapshot
    runtime, _robot, _transport, clock = make_runtime(
        mode="ecal",
        backend=Backend(),
        document=scene_document(),
        logger=logger,
        transport=transport,
    )
    wheel_topic = runtime.config.wheel_state.topic
    front_topic = runtime.config.lidar_front.topic
    failed_quality = (
        TransportTopicQuality(
            wheel_topic,
            error_count=1,
            state="error",
            detail="RuntimeError: async wheel send failed",
            revision=1,
        ),
        TransportTopicQuality(
            front_topic,
            dropped_count=2,
            state="degraded",
            detail="output queue replaced a pending message",
            revision=1,
        ),
    )
    try:
        quality = failed_quality
        assert runtime.poll_transport() == "active"
        failed = runtime.status_snapshot(wall_time=clock()).topics
        assert failed[wheel_topic].error_count == 1
        assert failed[wheel_topic].state == "error"
        assert "async wheel send failed" in failed[wheel_topic].detail
        assert failed[front_topic].dropped_count == 2
        assert failed[front_topic].state == "degraded"
        assert [event for event, _fields in logger.events].count("publish_failed") == 1
        assert [event for event, _fields in logger.events].count("queue_dropped") == 1
        publish_event = next(
            fields for event, fields in logger.events if event == "publish_failed"
        )
        assert publish_event["count"] == 1
        queue_events = [
            fields for event, fields in logger.events if event == "queue_dropped"
        ]
        assert all(
            fields == {
                "wall_time_ns": 0,
                "sim_time_ns": 0,
                "robot_model": "df_mid",
                "terrain_model": "flat",
                "topic": front_topic,
                "reason": "output queue replaced a pending message",
                "count": 2,
                "source": "transport",
            }
            for fields in queue_events
        )

        quality = (
            TransportTopicQuality(
                wheel_topic,
                error_count=1,
                revision=2,
            ),
            TransportTopicQuality(
                front_topic,
                dropped_count=2,
                revision=2,
            ),
        )
        runtime.poll_transport()
        recovered = runtime.status_snapshot(wall_time=clock()).topics
        assert recovered[wheel_topic].state == "active"
        assert recovered[wheel_topic].detail == ""
        assert recovered[wheel_topic].error_count == 1
        assert recovered[front_topic].state == "active"
        assert recovered[front_topic].detail == ""
        assert recovered[front_topic].dropped_count == 2

        quality = failed_quality
        runtime.poll_transport()
        stale = runtime.status_snapshot(wall_time=clock()).topics
        assert stale[wheel_topic] == recovered[wheel_topic]
        assert stale[front_topic] == recovered[front_topic]
        assert [event for event, _fields in logger.events].count("publish_failed") == 1
        assert [event for event, _fields in logger.events].count("queue_dropped") == 1
    finally:
        runtime.close()


def test_poll_transport_aggregates_huge_quality_deltas_without_linear_expansion(
    monkeypatch,
) -> None:
    logger = Logger()
    transport = Transport(mode="ecal")
    quality: tuple[TransportTopicQuality, ...] = ()

    def snapshot() -> TransportSnapshot:
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=True,
            published_count=0,
            received_count=0,
            error_count=sum(item.error_count for item in quality),
            dropped_count=sum(item.dropped_count for item in quality),
            topic_quality=quality,
        )

    transport.snapshot = snapshot
    runtime, _robot, _transport, clock = make_runtime(
        mode="ecal",
        backend=Backend(),
        document=scene_document(),
        logger=logger,
        transport=transport,
    )
    wheel_topic = runtime.config.wheel_state.topic
    front_topic = runtime.config.lidar_front.topic
    delta = 10_000_000

    def reject_linear_range(*_args):
        raise AssertionError("quality delta must not be expanded with range")

    monkeypatch.setattr(runtime_module, "range", reject_linear_range, raising=False)
    quality = (
        TransportTopicQuality(
            wheel_topic,
            error_count=delta,
            state="error",
            detail="bulk async send failures",
            revision=1,
        ),
        TransportTopicQuality(
            front_topic,
            dropped_count=delta,
            state="degraded",
            detail="bulk queue replacements",
            revision=1,
        ),
    )
    try:
        runtime.poll_transport()

        status = runtime.status_snapshot(wall_time=clock()).topics
        assert status[wheel_topic].error_count == delta
        assert status[front_topic].dropped_count == delta
        assert [
            (event, fields["topic"], fields["count"])
            for event, fields in logger.events
        ] == [
            ("publish_failed", wheel_topic, delta),
            ("queue_dropped", front_topic, delta),
        ]
    finally:
        runtime.close()


def test_ecal_interface_frame_never_submits_dashboard_twist() -> None:
    """入口得知实际 transport 为 eCAL 后，不得把本地速度值送入 runtime。"""
    runtime, robot, transport, clock = make_runtime(mode="ecal")
    runtime.submit_local_twist = lambda *_args, **_kwargs: (_ for _ in ()).throw(
        AssertionError("eCAL frame submitted a local twist")
    )

    class Coordinator:
        def __init__(self) -> None:
            self.steps = 0

        def step(self, dt: float) -> None:
            self.steps += 1
            clock.advance(dt)

    coordinator = Coordinator()
    try:
        simulation_module.run_interface_physics_frame(
            runtime,
            coordinator,
            actual_transport_mode="ecal",
            linear_velocity=9.0,
            angular_velocity=-4.0,
            dt=0.01,
        )

        assert coordinator.steps == 1
        assert transport.poll_count == 1
        assert robot.twists == []
        assert runtime.status_snapshot(wall_time=clock()).command.state == "waiting_command"
    finally:
        runtime.close()


def test_sensor_failure_isolated_and_status_contains_all_six_channels() -> None:
    backend = Backend()
    runtime, _robot, transport, clock = make_runtime(
        backend=backend,
        document=scene_document(),
    )
    assert isinstance(runtime._front_lidar, MultiLineLidar)
    assert isinstance(runtime._truth_sensor_suite, TruthSensorSuite)
    runtime._front_lidar = StubLidar("lidar_front", 1, RuntimeError("front ray failed"))
    runtime._rear_lidar = StubLidar("lidar_rear", 2)
    runtime._truth_sensor_suite = StubTruth()
    try:
        clock.advance(0.1)
        runtime.after_physics_step(0.1)
        snapshot = runtime.status_snapshot(wall_time=clock())

        assert tuple(snapshot.topics) == tuple(channel.topic for channel in runtime.config.channels)
        front = snapshot.topics[runtime.config.lidar_front.topic]
        assert front.state == "error" and front.message_count == 0
        assert "front ray failed" in front.detail
        for channel in (runtime.config.lidar_rear, runtime.config.rtk, runtime.config.imu):
            status = snapshot.topics[channel.topic]
            assert status.state == "active"
            assert status.message_count == 1
            assert status.latest_timestamp_ns == 100_000_000
        assert runtime.config.lidar_front.topic not in [item[0] for item in transport.published]
    finally:
        runtime.close()


def test_sensor_reader_failure_records_event_and_rejection_degrades_topic() -> None:
    logger = RejectingEventLogger()
    runtime, _robot, _transport, clock = make_runtime(
        backend=Backend(),
        document=scene_document(),
        logger=logger,
    )
    runtime._front_lidar = StubLidar(
        "lidar_front",
        1,
        RuntimeError("front sensor failed"),
    )
    runtime._rear_lidar = StubLidar("lidar_rear", 2)
    runtime._truth_sensor_suite = StubTruth()
    topic = runtime.config.lidar_front.topic
    try:
        clock.advance(0.2)
        runtime.after_physics_step(0.2)
        status = runtime.status_snapshot(wall_time=clock()).topics[topic]

        sensor_events = [item for item in logger.events if item[0] == "sensor_failed"]
        assert [fields["sim_time_ns"] for _event, fields in sensor_events] == [
            100_000_000,
            200_000_000,
        ]
        for _event, fields in sensor_events:
            assert fields["topic"] == topic
            assert "front sensor failed" in str(fields["reason"])
            assert fields["wall_time_ns"] == 200_000_000
            assert fields["robot_model"] == "df_mid"
            assert fields["terrain_model"] == "flat"
        assert status.error_count == 2
        assert status.dropped_count == 2
        assert status.state == "degraded"
    finally:
        runtime.close()


def test_command_timeout_event_is_recorded_once_per_timeout_edge() -> None:
    logger = Logger()
    runtime, _robot, _transport, clock = make_runtime(
        backend=Backend(),
        document=scene_document(),
        logger=logger,
    )
    topic = runtime.config.wheel_command.topic
    try:
        assert runtime.accept_local_command(
            WheelCommand(1, (1.0, 2.0), ()),
            received_at=clock(),
        )
        runtime.before_physics_step(0.01, wall_time=clock())
        clock.advance(runtime.config.command_timeout_sec)
        runtime.before_physics_step(0.01, wall_time=clock())
        runtime.before_physics_step(0.01, wall_time=clock())

        first_epoch = [item for item in logger.events if item[0] == "command_timeout"]
        assert len(first_epoch) == 1
        fields = first_epoch[0][1]
        assert fields["robot_model"] == "df_mid"
        assert fields["terrain_model"] == "flat"
        assert fields["topic"] == topic
        assert fields["wall_time_ns"] == 100_000_000
        assert fields["sim_time_ns"] == 0
        assert fields["reason"]

        assert runtime.accept_local_command(
            WheelCommand(2, (2.0, 3.0), ()),
            received_at=clock(),
        )
        runtime.before_physics_step(0.01, wall_time=clock())
        clock.advance(runtime.config.command_timeout_sec)
        runtime.before_physics_step(0.01, wall_time=clock())

        assert len([item for item in logger.events if item[0] == "command_timeout"]) == 2
    finally:
        runtime.close()


def test_new_command_epoch_logs_timeout_without_intermediate_active_frame() -> None:
    logger = Logger()
    runtime, _robot, _transport, clock = make_runtime(
        backend=Backend(),
        document=scene_document(),
        logger=logger,
    )
    try:
        assert runtime.accept_local_command(
            WheelCommand(1, (1.0, 2.0), ()),
            received_at=clock(),
        )
        clock.advance(runtime.config.command_timeout_sec)
        runtime.before_physics_step(0.01, wall_time=clock())

        assert runtime.accept_local_command(
            WheelCommand(2, (2.0, 3.0), ()),
            received_at=clock(),
        )
        clock.advance(runtime.config.command_timeout_sec)
        runtime.before_physics_step(0.01, wall_time=clock())
        runtime.before_physics_step(0.01, wall_time=clock())

        timeout_events = [event for event, _fields in logger.events if event == "command_timeout"]
        assert timeout_events == ["command_timeout", "command_timeout"]
    finally:
        runtime.close()


def test_production_session_logs_ecal_lifecycle_before_logger_close(monkeypatch) -> None:
    trace: list[str] = []
    logger = Logger(trace=trace)
    transport = Transport(mode="ecal")
    peer_callback = None
    peer_state = "waiting_peer"

    def record_event(event: str, **fields: object) -> bool:
        trace.append(f"event:{event}")
        logger.events.append((event, fields))
        return True

    logger.record_event = record_event

    def snapshot() -> TransportSnapshot:
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=peer_state != "disconnected",
            published_count=0,
            received_count=0,
            error_count=0,
            dropped_count=0,
        )

    transport.snapshot = snapshot

    def create_peer_transport(_mode, *, config, peer_state_callback):
        nonlocal peer_callback
        assert config.transport_mode == "ecal"
        peer_callback = peer_state_callback
        return transport

    def transition(state: str) -> None:
        nonlocal peer_state
        peer_state = state
        assert peer_callback is not None
        peer_callback(state)

    world = SimpleNamespace(
        scene=SimpleNamespace(body_ids=(10,)),
        active_robot=SimpleNamespace(robot=Robot()),
    )

    class Manager:
        def snapshot(self, *, include_body_id=False):
            return ()

    monkeypatch.setattr(simulation_module, "PyBulletSensorBackend", lambda *_args: Backend())
    monkeypatch.setattr(simulation_module, "create_transport", create_peer_transport)
    monkeypatch.setattr(simulation_module, "InterfaceEventLogger", lambda *_args, **_kwargs: logger)

    session = simulation_module.create_interface_session(
        ExperimentConfig(
            interface_mode="ecal",
            interface_enabled=True,
            interface_log_enabled=True,
        ),
        client_id=3,
        coordinator_world=world,
        obstacle_manager=Manager(),
        document=scene_document(),
    )
    assert session is not None
    transition("disconnected")
    transition("disconnected")
    transition("waiting_peer")
    transition("waiting_peer")
    session.close()

    event_names = [event for event, _fields in logger.events]
    assert event_names == [
        "ecal_initialized",
        "ecal_disconnected",
        "ecal_reconnected",
        "ecal_closed",
    ]
    assert trace.index("event:ecal_closed") < trace.index("logger.close")
    for _event, fields in logger.events:
        assert fields["robot_model"] == "df_mid"
        assert fields["terrain_model"] == "flat"
        assert fields["topic"] == session.runtime.config.wheel_command.topic


def test_logger_queue_rejections_are_persisted_after_capacity_recovers(tmp_path) -> None:
    writer = RecoveringQueueWriter(drain_count=3)
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="runtime-recovered-drops",
        queue_size=3,
        writer=writer,
    )
    runtime, _robot, _transport, _clock = make_runtime(logger=logger)
    try:
        for index in range(3):
            runtime._record_runtime_event("sensor_failed", reason=f"queued-{index}")
        assert writer.started.wait(timeout=1.0)

        runtime._record_runtime_event("sensor_failed", reason="rejected-0")
        runtime._record_runtime_event("sensor_failed", reason="rejected-1")
        # 第二条普通事件前的聚合探测也会被 logger 计为一次 dropped event。
        assert logger.snapshot().dropped_events == 3

        writer.release.set()
        assert writer.drained.wait(timeout=2.0)
        runtime._record_runtime_event("sensor_failed", reason="capacity recovered")
        paths = logger.paths
    finally:
        writer.release.set()
        runtime.close()

    events = [
        json.loads(line)
        for line in paths.event_path.read_text(encoding="utf-8").splitlines()
    ]
    logger_drops = [
        event
        for event in events
        if event["event"] == "queue_dropped"
        and event.get("source") == "interface_logger"
    ]
    assert logger_drops == [
        {
            "count": 2,
            "event": "queue_dropped",
            "source": "interface_logger",
        }
    ]


def test_pending_logger_drop_precedes_next_ordinary_event_with_one_recovered_slot(
    tmp_path,
) -> None:
    writer = SingleCapacityOrderingWriter()
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="runtime-drop-order",
        queue_size=1,
        writer=writer,
    )
    runtime, _robot, _transport, _clock = make_runtime(logger=logger)
    real_record_event = logger.record_event
    real_terminal_event = logger.record_terminal_event
    terminal_calls = 0

    def wait_for_capacity() -> None:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if logger._capacity.acquire(blocking=False):
                logger._capacity.release()
                return
            time.sleep(0.005)
        raise AssertionError("logger capacity did not recover")

    def synchronized_record_event(event: str, **fields: object) -> bool:
        accepted = real_record_event(event, **fields)
        if (
            accepted
            and event == "queue_dropped"
            and fields.get("source") == "interface_logger"
        ):
            assert writer.wait_for_event(
                "queue_dropped",
                source="interface_logger",
                count=1,
            )
            wait_for_capacity()
        return accepted

    def observed_terminal_event(
        event: str,
        *,
        timeout_sec: float = 1.0,
        **fields: object,
    ) -> bool:
        nonlocal terminal_calls
        terminal_calls += 1
        return real_terminal_event(event, timeout_sec=timeout_sec, **fields)

    logger.record_event = synchronized_record_event
    logger.record_terminal_event = observed_terminal_event
    paths = logger.paths
    try:
        runtime._record_runtime_event("sensor_failed", reason="occupy capacity")
        assert writer.initial_started.wait(timeout=1.0)
        runtime._record_runtime_event("sensor_failed", reason="create pending drop")
        assert logger.snapshot().dropped_events == 1

        writer.release_initial.set()
        assert writer.wait_for_event("sensor_failed", reason="occupy capacity")
        wait_for_capacity()

        # 唯一恢复名额必须先给聚合事件；writer 随后释放它供普通事件使用。
        runtime._record_runtime_event("sensor_failed", reason="capacity recovered")
        assert writer.recovery_started.wait(timeout=1.0)
        writer.release_recovery.set()
        assert writer.wait_for_event("sensor_failed", reason="capacity recovered")
    finally:
        writer.release_initial.set()
        writer.release_recovery.set()
        runtime.close()

    events = [
        json.loads(line)
        for line in paths.event_path.read_text(encoding="utf-8").splitlines()
    ]
    logger_drop_index = next(
        index
        for index, event in enumerate(events)
        if event["event"] == "queue_dropped"
        and event.get("source") == "interface_logger"
        and event.get("count") == 1
    )
    recovery_index = next(
        index
        for index, event in enumerate(events)
        if event["event"] == "sensor_failed"
        and event.get("reason") == "capacity recovered"
    )
    assert logger_drop_index < recovery_index
    assert terminal_calls == 0


def test_runtime_close_uses_bounded_terminal_path_for_pending_logger_drop(tmp_path) -> None:
    writer = SlowSuccessfulWriter(delay_sec=0.1)
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="runtime-terminal-drop",
        queue_size=1,
        writer=writer,
    )
    runtime, _robot, _transport, _clock = make_runtime(logger=logger)
    paths = logger.paths

    runtime._record_runtime_event("sensor_failed", reason="occupy capacity")
    assert writer.started.wait(timeout=1.0)
    runtime._record_runtime_event("sensor_failed", reason="must persist as drop")
    assert logger.snapshot().dropped_events == 1

    started = time.monotonic()
    runtime.close()
    elapsed = time.monotonic() - started

    assert elapsed < 1.5
    events = [
        json.loads(line)
        for line in paths.event_path.read_text(encoding="utf-8").splitlines()
    ]
    logger_drops = [
        event
        for event in events
        if event["event"] == "queue_dropped"
        and event.get("source") == "interface_logger"
    ]
    assert logger_drops == [
        {
            "count": 1,
            "event": "queue_dropped",
            "source": "interface_logger",
        }
    ]


def test_runtime_close_starts_terminal_log_budget_after_quiesce_returns(
    tmp_path,
    monkeypatch,
) -> None:
    wheel_topic = InterfaceConfig.default(transport_mode="local").wheel_state.topic

    class BlockingFinalDropTransport(Transport):
        """由测试推进 quiesce，并在返回时发布一个最终 transport drop。"""

        def __init__(self) -> None:
            super().__init__()
            self.quiesce_started = Event()
            self.release_quiesce = Event()

        def quiesce(self) -> TransportSnapshot:
            self.quiesce_count += 1
            self.quiesce_started.set()
            if not self.release_quiesce.wait(timeout=5.0):
                raise TimeoutError("test transport quiesce was not released")
            return TransportSnapshot(
                mode="local",
                ecal_connected=False,
                published_count=0,
                received_count=0,
                error_count=0,
                dropped_count=1,
                topic_quality=(
                    TransportTopicQuality(
                        wheel_topic,
                        dropped_count=1,
                        state="degraded",
                        detail="pending frame dropped during quiesce",
                        revision=1,
                    ),
                ),
            )

    deadline_clock = Clock(10.0)
    monkeypatch.setattr(
        runtime_module,
        "time",
        SimpleNamespace(monotonic=deadline_clock),
    )
    writer = RecoveringQueueWriter(drain_count=1)
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="runtime-post-quiesce-deadline",
        queue_size=1,
        writer=writer,
    )
    transport = BlockingFinalDropTransport()
    runtime, _robot, _transport, _clock = make_runtime(
        logger=logger,
        transport=transport,
    )
    terminal_started = Event()
    terminal_calls: list[tuple[dict[str, object], float]] = []
    real_terminal_event = logger.record_terminal_event

    def observed_terminal_event(
        event: str,
        *,
        timeout_sec: float = 1.0,
        **fields: object,
    ) -> bool:
        terminal_calls.append(({"event": event, **fields}, timeout_sec))
        terminal_started.set()
        return real_terminal_event(event, timeout_sec=timeout_sec, **fields)

    logger.record_terminal_event = observed_terminal_event
    paths = logger.paths
    close_completed = Event()
    close_errors: list[BaseException] = []

    def close_runtime() -> None:
        try:
            runtime.close()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            close_errors.append(exc)
        finally:
            close_completed.set()

    runtime._record_runtime_event("sensor_failed", reason="occupy logger capacity")
    assert writer.started.wait(timeout=1.0)
    closer = Thread(target=close_runtime, daemon=True)
    closer.start()
    try:
        assert transport.quiesce_started.wait(timeout=1.0)
        # 只推进终态预算时钟，确定性跨过旧实现提前创建的 1 秒 deadline。
        deadline_clock.advance(1.1)
        transport.release_quiesce.set()
        assert terminal_started.wait(timeout=1.0)
        assert not close_completed.is_set()
    finally:
        transport.release_quiesce.set()
        writer.release.set()
    closer.join(timeout=2.0)

    assert not closer.is_alive()
    assert close_errors == []
    assert close_completed.is_set()
    assert len(terminal_calls) == 1
    assert terminal_calls[0][0]["source"] == "transport"
    assert terminal_calls[0][1] == pytest.approx(1.0)
    events = [
        json.loads(line)
        for line in paths.event_path.read_text(encoding="utf-8").splitlines()
    ]
    transport_drops = [
        event
        for event in events
        if event["event"] == "queue_dropped"
        and event.get("source") == "transport"
    ]
    assert [(event["topic"], event["count"]) for event in transport_drops] == [
        (wheel_topic, 1)
    ]
    assert logger.snapshot().dropped_events == 0


def test_rejected_terminal_drop_event_is_not_counted_recursively() -> None:
    class RejectingTerminalLogger(Logger):
        """拒绝消息和终态诊断，用调用次数锁定非递归语义。"""

        def __init__(self) -> None:
            super().__init__(accept_messages=False)
            self.terminal_calls: list[tuple[str, dict[str, object]]] = []

        def record_terminal_event(
            self,
            event: str,
            *,
            timeout_sec: float = 1.0,
            **fields: object,
        ) -> bool:
            self.terminal_calls.append((event, fields))
            return False

    logger = RejectingTerminalLogger()
    runtime, _robot, transport, clock = make_runtime(logger=logger)
    payload = ProtoCodec().encode(WheelCommand(1, (1.0, 2.0), ()))

    assert transport.emit(runtime.config.wheel_command.topic, payload, clock()) is True
    runtime.close()

    assert logger.terminal_calls == [
        ("queue_dropped", {"source": "interface_logger", "count": 1})
    ]


def test_logging_only_records_success_and_rejection_degrades_topic() -> None:
    logger = Logger(accept_messages=False)
    runtime, _robot, transport, clock = make_runtime(logger=logger)
    codec = ProtoCodec()
    command_topic = runtime.config.wheel_command.topic
    wheel_topic = runtime.config.wheel_state.topic
    transport.outcomes[wheel_topic] = deque((False, True))
    try:
        assert transport.emit(command_topic, b"not protobuf", clock()) is False
        valid_payload = codec.encode(WheelCommand(9, (1.0, 2.0), ()))
        assert transport.emit(command_topic, valid_payload, clock()) is True

        clock.advance(0.02)
        runtime.after_physics_step(0.01)
        runtime.after_physics_step(0.01)
        snapshot = runtime.status_snapshot(wall_time=clock())
        command_status = snapshot.topics[command_topic]

        assert any(event == "protobuf_parse_failed" for event, _fields in logger.events)
        assert command_status.dropped_count == 1
        assert command_status.state == "degraded"
        wheel_records = [
            record for record in logger.messages
            if record.topic == wheel_topic and record.direction == "publish"
        ]
        assert len(wheel_records) == 1
        assert snapshot.topics[wheel_topic].message_count == 1
        assert snapshot.topics[wheel_topic].error_count == 1
        assert snapshot.topics[wheel_topic].dropped_count == 1
        assert snapshot.topics[wheel_topic].state == "degraded"
        receive_records = [record for record in logger.messages if record.direction == "receive"]
        assert len(receive_records) == 1
        assert [record.sequence for record in logger.messages] == sorted(
            record.sequence for record in logger.messages
        )
        assert [record.wall_time_ns for record in logger.messages] == sorted(
            record.wall_time_ns for record in logger.messages
        )
    finally:
        runtime.close()


def test_concurrent_receive_logs_are_enqueued_in_monotonic_sequence_order() -> None:
    logger = OrderedBlockingLogger()
    runtime, _robot, transport, clock = make_runtime(logger=logger)
    codec = ProtoCodec()
    topic = runtime.config.wheel_command.topic
    payloads = (
        codec.encode(WheelCommand(1, (1.0, 2.0), ())),
        codec.encode(WheelCommand(2, (2.0, 3.0), ())),
    )
    results: list[object] = []
    first = Thread(target=lambda: results.append(transport.emit(topic, payloads[0], clock())), daemon=True)
    second = Thread(target=lambda: results.append(transport.emit(topic, payloads[1], clock())), daemon=True)
    try:
        first.start()
        assert logger.first_entered.wait(timeout=2.0)
        second.start()
        second.join(timeout=0.2)
        logger.release_first.set()
        first.join(timeout=2.0)
        second.join(timeout=2.0)

        assert not first.is_alive() and not second.is_alive()
        assert results == [True, True]
        assert [record.sequence for record in logger.messages] == [0, 1]
        assert [record.wall_time_ns for record in logger.messages] == sorted(
            record.wall_time_ns for record in logger.messages
        )
    finally:
        logger.release_first.set()
        first.join(timeout=2.0)
        second.join(timeout=2.0)
        runtime.close()


def test_invalid_command_callback_reads_mailbox_error_only_once() -> None:
    runtime, _robot, transport, clock = make_runtime()
    mailbox = runtime._mailbox
    original_snapshot = mailbox.snapshot
    snapshot_calls = 0

    def single_snapshot(*, now=None):
        nonlocal snapshot_calls
        snapshot_calls += 1
        if snapshot_calls > 1:
            raise RuntimeError("mailbox error was read twice")
        return original_snapshot(now=now)

    mailbox.snapshot = single_snapshot
    payload = ProtoCodec().encode(WheelCommand(3, (1.0, 2.0, 3.0), ()))
    try:
        assert transport.emit(runtime.config.wheel_command.topic, payload, clock()) is False
        assert snapshot_calls == 1
    finally:
        runtime.close()


@pytest.mark.parametrize(
    ("command", "expected_event"),
    (
        (WheelCommand(31, (1.0, 2.0, 3.0), ()), "model_mismatch"),
        (WheelCommand(32, (21.0, 0.0), ()), "mechanical_limit"),
    ),
)
def test_structured_wheel_rejections_emit_one_specific_event(
    command: WheelCommand,
    expected_event: str,
) -> None:
    logger = Logger()
    runtime, _robot, transport, clock = make_runtime(
        backend=Backend(),
        document=scene_document(),
        logger=logger,
    )
    runtime.update_scene_document(
        replace(
            scene_document(),
            terrain=TerrainDocument("slope", 12.0, 0, "low"),
        )
    )
    payload = ProtoCodec().encode(command)
    try:
        assert transport.emit(
            runtime.config.wheel_command.topic,
            payload,
            clock(),
        ) is False

        assert [event for event, _fields in logger.events] == [expected_event]
        assert logger.events[0][1] == {
            "wall_time_ns": 0,
            "sim_time_ns": 0,
            "robot_model": "df_mid",
            "terrain_model": "slope",
            "topic": runtime.config.wheel_command.topic,
            "reason": logger.events[0][1]["reason"],
        }
        assert logger.events[0][1]["reason"]
    finally:
        runtime.close()


def test_event_waiting_to_write_is_dropped_after_world_generation_changes() -> None:
    logger = Logger()
    runtime, _robot, transport, clock = make_runtime(
        backend=Backend(),
        document=scene_document(),
        logger=logger,
    )
    context_captured = Event()
    callback_results: list[object] = []
    payload = ProtoCodec().encode(WheelCommand(51, (1.0, 2.0, 3.0), ()))
    original_monotonic = runtime._monotonic

    def signal_context_capture() -> float:
        context_captured.set()
        return original_monotonic()

    runtime._monotonic = signal_context_capture
    runtime._log_lock.acquire()
    log_lock_held = True
    callback_thread = Thread(
        target=lambda: callback_results.append(
            transport.emit(runtime.config.wheel_command.topic, payload, clock())
        ),
        daemon=True,
    )
    try:
        callback_thread.start()
        assert context_captured.wait(timeout=1.0)
        new_document = replace(
            scene_document(),
            terrain=TerrainDocument("slope", 8.0, 0, "low"),
        )
        runtime.prepare_world_rebuild()
        runtime.commit_world_rebuild(Robot(52), Backend(), new_document)

        runtime._log_lock.release()
        log_lock_held = False
        callback_thread.join(timeout=2.0)

        assert not callback_thread.is_alive()
        assert callback_results == [False]
        assert logger.events == []
        assert runtime.scene_document == new_document
        command_status = runtime.status_snapshot(wall_time=clock()).topics[
            runtime.config.wheel_command.topic
        ]
        assert command_status.error_count == 0
        assert command_status.dropped_count == 0
    finally:
        if log_lock_held:
            runtime._log_lock.release()
        callback_thread.join(timeout=2.0)
        runtime.close()


def test_nonclassified_command_failure_remains_invalid_command() -> None:
    logger = Logger()
    runtime, _robot, transport, _clock = make_runtime(logger=logger)
    codec = ProtoCodec()
    topic = runtime.config.wheel_command.topic
    try:
        assert transport.emit(topic, codec.encode(WheelCommand(41, (1.0, 2.0), ())), 1.0)
        assert transport.emit(topic, codec.encode(WheelCommand(42, (2.0, 3.0), ())), 0.5) is False
        assert [event for event, _fields in logger.events] == ["invalid_command"]
    finally:
        runtime.close()
