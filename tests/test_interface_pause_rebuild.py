# InterfaceRuntime 生命周期测试：覆盖暂停、重建代际、场景刷新和关闭顺序。
from __future__ import annotations

from threading import Event, Lock, Thread

import pytest

from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.dashboard_snapshot import LidarTopViewFrame
from slope_sim.interfaces.models import LidarPointCloud, WheelCommand
from slope_sim.lidar_pointcloud import LidarScanResult

from test_interface_runtime_integration import (
    Backend,
    Clock,
    Logger,
    Robot,
    StubLidar,
    StubTruth,
    Transport,
    make_runtime,
    scene_document,
)


class BlockingFrontPublishTransport(Transport):
    """在 front payload 被 transport 接受前阻塞，并记录跨线程线性化顺序。"""

    def __init__(self, *, publish_error: BaseException | None = None) -> None:
        super().__init__()
        self.publish_error = publish_error
        self.front_started = Event()
        self.release_front = Event()
        self.timeline: list[str] = []

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float | None = None,
    ) -> bool:
        if topic == "/sim/lidar/front/points":
            self.front_started.set()
            assert self.release_front.wait(timeout=3.0)
            if self.publish_error is not None:
                self.timeline.append("front_raised")
                raise self.publish_error
            self.timeline.append("front_accepted")
        return super().publish(
            topic,
            payload,
            type_name,
            sim_time_ns,
            wall_time=wall_time,
        )


class TimelineLogger(Logger):
    """把旧 publish 的消息/事件处理点写入 transport 共用时间线。"""

    def __init__(self, timeline: list[str]) -> None:
        super().__init__()
        self.timeline = timeline

    def record_message(self, record) -> bool:
        accepted = super().record_message(record)
        if record.topic == "/sim/lidar/front/points":
            self.timeline.append("front_logged")
        return accepted

    def record_event(self, event: str, **fields: object) -> bool:
        accepted = super().record_event(event, **fields)
        if event == "publish_failed":
            self.timeline.append("publish_failed_logged")
        return accepted


class BlockingRejectingLogger(Logger):
    """只阻塞并拒绝指定旧代消息或事件，其余记录正常接受。"""

    def __init__(
        self,
        *,
        message_topic: str | None = None,
        event_name: str | None = None,
    ) -> None:
        super().__init__()
        self.message_topic = message_topic
        self.event_name = event_name
        self.entered = Event()
        self.release = Event()

    def record_message(self, record) -> bool:
        self.messages.append(record)
        if record.topic != self.message_topic:
            return True
        self.entered.set()
        assert self.release.wait(timeout=3.0)
        return False

    def record_event(self, event: str, **fields: object) -> bool:
        self.events.append((event, fields))
        if event != self.event_name:
            return True
        self.entered.set()
        assert self.release.wait(timeout=3.0)
        return False

def test_pause_freezes_clock_local_scheduler_and_physical_topics_but_poll_continues() -> None:
    backend = Backend()
    runtime, robot, transport, clock = make_runtime(
        backend=backend,
        document=scene_document(),
    )
    runtime._front_lidar = StubLidar("lidar_front", 1)
    runtime._rear_lidar = StubLidar("lidar_rear", 2)
    runtime._truth_sensor_suite = StubTruth()
    try:
        assert runtime.submit_local_twist(1.0, 0.0, 0.01)
        runtime.pause()
        before = tuple(transport.published)
        for _ in range(3):
            clock.advance(0.1)
            assert runtime.before_physics_step(0.1, wall_time=clock()) is None
            assert runtime.after_physics_step(0.1) == ()
            runtime.poll_transport()

        assert runtime.clock.now_ns == 0
        assert tuple(transport.published) == before
        assert robot.twists == []
        assert runtime.connection_polls == 3

        runtime.accept_local_command(WheelCommand(1, (2.0, 2.0), ()), received_at=0.0)
        decision = runtime.resume(wall_time=clock())
        assert decision.timed_out
    finally:
        runtime.close()


def test_prepare_commit_rebuild_keeps_transport_and_refreshes_dynamic_bindings() -> None:
    old_backend = Backend()
    runtime, old_robot, transport, clock = make_runtime(
        backend=old_backend,
        document=scene_document(),
    )
    new_robot = Robot(42)
    new_backend = Backend()
    document = scene_document()
    try:
        assert runtime.accept_local_command(WheelCommand(1, (1.0, 2.0), ()), received_at=clock())
        old_mailbox, old_generation = runtime.capture_command_ingress()
        old_subscription = transport.subscriptions[-1][2]

        runtime.prepare_world_rebuild()
        assert old_subscription.closed
        with pytest.raises(RuntimeError, match="not accepting"):
            runtime.accept_local_command(WheelCommand(2, (2.0, 3.0), ()), received_at=clock())
        runtime.commit_world_rebuild(new_robot, new_backend, document)

        assert runtime.bound_robot_id == 42
        assert runtime.scene_document is document
        assert runtime.last_decision.waiting
        rebuilt_status = runtime.status_snapshot(wall_time=clock())
        assert rebuilt_status.command.valid_count == 0
        assert rebuilt_status.topics[runtime.config.wheel_command.topic].latest_timestamp_ns is None
        assert runtime._transport is transport
        assert not old_mailbox.accept(
            WheelCommand(3, (3.0, 4.0), ()),
            received_at=clock(),
            generation=old_generation,
        )
        snapshots = (object(), object())
        runtime.refresh_scene_bindings((5, 6), snapshots)
        assert new_backend.bind_calls == [((5, 6), snapshots)]
        assert document.obstacles == ()
        assert old_robot.safe_stops == 1
    finally:
        runtime.close()


class OwnedBackend(Backend):
    """记录 backend 所有权释放次数，并可注入 close 或 link 校验失败。"""

    def __init__(
        self,
        name: str,
        trace: list[str],
        *,
        close_error: BaseException | None = None,
        valid_links: bool = True,
    ) -> None:
        super().__init__(trace)
        self.name = name
        self.close_error = close_error
        self.valid_links = valid_links
        self.close_count = 0

    def link_names(self):
        if not self.valid_links:
            return ("base_link",)
        return super().link_names()

    def close(self) -> None:
        self.close_count += 1
        self.trace.append(f"{self.name}.close")
        if self.close_error is not None:
            raise self.close_error


@pytest.mark.parametrize("old_close_raises", (False, True))
def test_successful_rebuild_closes_old_backend_once_without_reversing_commit(
    old_close_raises: bool,
) -> None:
    trace: list[str] = []
    old_backend = OwnedBackend(
        "old",
        trace,
        close_error=RuntimeError("old backend close failed") if old_close_raises else None,
    )
    runtime, _old_robot, _transport, _clock = make_runtime(
        backend=old_backend,
        document=scene_document(),
    )
    new_backend = OwnedBackend("new", trace)
    new_robot = Robot(61)
    try:
        runtime.prepare_world_rebuild()
        runtime.commit_world_rebuild(new_robot, new_backend, scene_document())

        assert old_backend.close_count == 1
        assert new_backend.close_count == 0
        assert runtime.bound_robot_id == 61
        runtime.refresh_scene_bindings((7,), ())
        assert new_backend.bind_calls == [((7,), ())]
    finally:
        runtime.close()

    assert old_backend.close_count == 1
    assert new_backend.close_count == 1


def test_rebuild_with_same_backend_instance_does_not_close_active_backend() -> None:
    trace: list[str] = []
    backend = OwnedBackend("shared", trace)
    runtime, _old_robot, _transport, _clock = make_runtime(
        backend=backend,
        document=scene_document(),
    )

    runtime.prepare_world_rebuild()
    runtime.commit_world_rebuild(Robot(62), backend, scene_document())

    assert backend.close_count == 0
    runtime.close()
    assert backend.close_count == 1


def test_failed_rebuild_does_not_close_old_or_candidate_backend() -> None:
    trace: list[str] = []
    old_backend = OwnedBackend("old", trace)
    runtime, _old_robot, _transport, _clock = make_runtime(
        backend=old_backend,
        document=scene_document(),
    )
    invalid_candidate = OwnedBackend("candidate", trace, valid_links=False)
    try:
        runtime.prepare_world_rebuild()
        with pytest.raises(ValueError, match="lidar parent link"):
            runtime.commit_world_rebuild(
                Robot(63),
                invalid_candidate,
                scene_document(),
            )

        assert old_backend.close_count == 0
        assert invalid_candidate.close_count == 0
    finally:
        runtime.close()

    assert old_backend.close_count == 1
    assert invalid_candidate.close_count == 0


class RegisteredRebindSubscription:
    """记录 rebind 候选 callback，但调用方无法取得订阅句柄。"""

    def __init__(self, callback) -> None:
        self.callback = callback
        self.closed = False

    def close(self) -> None:
        self.closed = True


class RebindRegisterThenRaiseTransport(Transport):
    """初始订阅成功，rebind 候选注册后抛错。"""

    def __init__(self) -> None:
        super().__init__()
        self.subscribe_calls = 0
        self.uncertain_subscription: RegisteredRebindSubscription | None = None

    def subscribe(self, topic: str, type_name: str, callback):
        self.subscribe_calls += 1
        if self.subscribe_calls == 2:
            subscription = RegisteredRebindSubscription(callback)
            self.uncertain_subscription = subscription
            self.subscriptions.append((topic, type_name, subscription))
            raise RuntimeError("rebind subscribe registered then failed")
        return super().subscribe(topic, type_name, callback)


def test_rebind_replaces_transport_subscription_and_routes_only_to_new_robot() -> None:
    runtime, old_robot, transport, clock = make_runtime()
    old_subscription = transport.subscriptions[-1][2]
    new_robot = Robot(71)
    try:
        runtime.rebind_robot(new_robot)

        new_subscription = transport.subscriptions[-1][2]
        assert new_subscription is not old_subscription
        payload = ProtoCodec().encode(WheelCommand(301, (2.0, 3.0), ()))
        assert old_subscription.callback(payload, clock()) is None
        assert new_subscription.callback(payload, clock()) is True
        assert runtime.status_snapshot(wall_time=clock()).command.valid_count == 1
        decision = runtime.before_physics_step(1.0 / 240.0, wall_time=clock())
        assert decision is not None and decision.waiting is False
        assert old_robot.commands == []
        assert new_robot.commands == [((2.0, 3.0), (), 1.0 / 240.0)]
    finally:
        runtime.close()


def test_rebind_registered_then_failed_candidate_never_activates() -> None:
    transport = RebindRegisterThenRaiseTransport()
    runtime, old_robot, _selected_transport, clock = make_runtime(transport=transport)
    old_subscription = transport.subscriptions[-1][2]
    new_robot = Robot(72)
    try:
        with pytest.raises(RuntimeError, match="rebind subscribe registered then failed"):
            runtime.rebind_robot(new_robot)

        uncertain = transport.uncertain_subscription
        assert uncertain is not None and uncertain.closed is False
        payload = ProtoCodec().encode(WheelCommand(302, (4.0, 5.0), ()))
        assert uncertain.callback(payload, clock()) is None
        assert old_subscription.callback(payload, clock()) is True
        assert runtime.bound_robot_id == old_robot.robot_id
        assert runtime.status_snapshot(wall_time=clock()).command.valid_count == 1
        decision = runtime.before_physics_step(1.0 / 240.0, wall_time=clock())
        assert decision is not None and decision.waiting is False
        assert old_robot.commands == [((4.0, 5.0), (), 1.0 / 240.0)]
        assert new_robot.commands == []
    finally:
        runtime.close()


def test_prepare_rebuild_parks_old_robot_after_subscription_barrier_failure() -> None:
    runtime, robot, transport, _clock = make_runtime()
    subscription = transport.subscriptions[-1][2]
    subscription.close_error = RuntimeError("subscription barrier failed")
    try:
        with pytest.raises(RuntimeError, match="subscription barrier failed"):
            runtime.prepare_world_rebuild()

        assert subscription.closed
        assert robot.safe_stops == 1
        assert runtime.last_decision.waiting
    finally:
        runtime.close()


def test_close_after_successful_prepare_records_safe_stop_without_reparking() -> None:
    runtime, robot, _transport, _clock = make_runtime()

    runtime.prepare_world_rebuild()
    assert robot.safe_stops == 1
    runtime.close()

    assert robot.safe_stops == 1
    assert runtime.close_trace == (
        "stop_commands",
        "safe_stop",
        "stop_sensors",
        "quiesce_transport",
        "close_log",
        "close_transport",
        "close_sensors",
    )


class FailFirstParkingRobot(Robot):
    """首次停车失败，close 重试时成功。"""

    def hold_current_steering_and_stop_drive(self, _dt: float) -> None:
        self.safe_stops += 1
        if self.safe_stops == 1:
            raise RuntimeError("prepare parking failed")


def test_close_retries_parking_when_prepare_parking_failed() -> None:
    from slope_sim.interfaces.config import InterfaceConfig
    from slope_sim.interfaces.runtime import InterfaceRuntime

    robot = FailFirstParkingRobot()
    transport = Transport()
    runtime = InterfaceRuntime(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
    )
    try:
        with pytest.raises(RuntimeError, match="prepare parking failed"):
            runtime.prepare_world_rebuild()

        runtime.close()

        assert robot.safe_stops == 2
        assert runtime.close_trace[1] == "safe_stop"
    finally:
        runtime.close()


class BlockingParkingRobot(Robot):
    """阻塞 prepare 停车，暴露 close 是否并行重复触碰机器人。"""

    def __init__(self) -> None:
        super().__init__()
        self.parking_started = Event()
        self.second_parking_started = Event()
        self.release_parking = Event()

    def hold_current_steering_and_stop_drive(self, _dt: float) -> None:
        self.safe_stops += 1
        if self.safe_stops > 1:
            self.second_parking_started.set()
        self.parking_started.set()
        assert self.release_parking.wait(timeout=3.0)


def test_close_waits_for_in_progress_prepare_before_deciding_safe_stop() -> None:
    from slope_sim.interfaces.config import InterfaceConfig
    from slope_sim.interfaces.runtime import InterfaceRuntime

    robot = BlockingParkingRobot()
    transport = Transport()
    runtime = InterfaceRuntime(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
    )
    prepare_errors: list[BaseException] = []
    close_errors: list[BaseException] = []
    close_started = Event()

    def run_prepare() -> None:
        try:
            runtime.prepare_world_rebuild()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            prepare_errors.append(exc)

    def run_close() -> None:
        close_started.set()
        try:
            runtime.close()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            close_errors.append(exc)

    prepare_thread = Thread(target=run_prepare, daemon=True)
    close_thread = Thread(target=run_close, daemon=True)
    try:
        prepare_thread.start()
        assert robot.parking_started.wait(timeout=2.0)
        close_thread.start()
        assert close_started.wait(timeout=2.0)
        assert not robot.second_parking_started.wait(timeout=0.2)

        robot.release_parking.set()
        prepare_thread.join(timeout=2.0)
        close_thread.join(timeout=2.0)
        assert not prepare_thread.is_alive() and not close_thread.is_alive()
        assert prepare_errors == close_errors == []
        assert robot.safe_stops == 1
    finally:
        robot.release_parking.set()
        prepare_thread.join(timeout=2.0)
        close_thread.join(timeout=2.0)
        runtime.close()


def test_rebind_rejects_while_prepare_parking_is_in_progress() -> None:
    from slope_sim.interfaces.config import InterfaceConfig
    from slope_sim.interfaces.runtime import InterfaceRuntime

    robot = BlockingParkingRobot()
    transport = Transport()
    runtime = InterfaceRuntime(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
    )
    new_robot = Robot(81)
    prepare_errors: list[BaseException] = []
    rebind_errors: list[BaseException] = []

    def run_prepare() -> None:
        try:
            runtime.prepare_world_rebuild()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            prepare_errors.append(exc)

    def run_rebind() -> None:
        try:
            runtime.rebind_robot(new_robot)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            rebind_errors.append(exc)

    prepare_thread = Thread(target=run_prepare)
    rebind_thread = Thread(target=run_rebind)
    try:
        prepare_thread.start()
        assert robot.parking_started.wait(timeout=2.0)
        rebind_thread.start()
        rebind_thread.join(timeout=0.5)

        assert not rebind_thread.is_alive()
        assert len(rebind_errors) == 1
        assert "world rebuild is in progress" in str(rebind_errors[0])
        assert not robot.second_parking_started.is_set()
        assert len(transport.subscriptions) == 1

        robot.release_parking.set()
        prepare_thread.join(timeout=2.0)
        assert not prepare_thread.is_alive()
        assert prepare_errors == []
        assert runtime.bound_robot_id == robot.robot_id
        runtime.abort_world_rebuild()
        assert runtime.status_snapshot().command.state == "waiting_command"
    finally:
        robot.release_parking.set()
        prepare_thread.join(timeout=2.0)
        rebind_thread.join(timeout=2.0)
        runtime.close()


class ConcurrentAbortTransport(Transport):
    """让两个 abort 候选订阅都注册后再同时竞争原子提交。"""

    def __init__(self) -> None:
        super().__init__()
        self._attempt_lock = Lock()
        self.abort_attempts = 0
        self.both_abort_subscribed = Event()
        self.release_abort_subscribe = Event()

    def subscribe(self, topic: str, type_name: str, callback):
        subscription = super().subscribe(topic, type_name, callback)
        with self._attempt_lock:
            if len(self.subscriptions) > 1:
                self.abort_attempts += 1
                if self.abort_attempts == 2:
                    self.both_abort_subscribed.set()
                should_block = True
            else:
                should_block = False
        if should_block:
            assert self.release_abort_subscribe.wait(timeout=3.0)
        return subscription


class BlockingCommitCloseErrorTransport(Transport):
    """阻塞 commit 候选订阅，并让候选清理抛出次生异常。"""

    def __init__(self) -> None:
        super().__init__()
        self.candidate_registered = Event()
        self.release_candidate = Event()
        self.candidate = None

    def subscribe(self, topic: str, type_name: str, callback):
        subscription = super().subscribe(topic, type_name, callback)
        if len(self.subscriptions) == 2:
            self.candidate = subscription
            subscription.close_error = RuntimeError("candidate close failed")
            self.candidate_registered.set()
            assert self.release_candidate.wait(timeout=3.0)
        return subscription


def test_commit_candidate_close_failure_preserves_primary_commit_error() -> None:
    transport = BlockingCommitCloseErrorTransport()
    runtime, _robot, _selected_transport, _clock = make_runtime(
        backend=Backend(),
        document=scene_document(),
        transport=transport,
    )
    errors: list[BaseException] = []

    def run_commit() -> None:
        try:
            runtime.commit_world_rebuild(Robot(82), Backend(), scene_document())
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            errors.append(exc)

    commit_thread = Thread(target=run_commit)
    try:
        runtime.prepare_world_rebuild()
        commit_thread.start()
        assert transport.candidate_registered.wait(timeout=2.0)

        runtime.close()
        transport.release_candidate.set()
        commit_thread.join(timeout=2.0)

        assert not commit_thread.is_alive()
        assert len(errors) == 1
        assert str(errors[0]) == "interface runtime is closed"
        assert transport.candidate is not None and transport.candidate.closed
    finally:
        transport.release_candidate.set()
        commit_thread.join(timeout=2.0)
        runtime.close()


def test_concurrent_abort_loser_does_not_fault_successfully_recovered_runtime() -> None:
    runtime, robot, transport, clock = make_runtime(
        transport=ConcurrentAbortTransport(),
    )
    assert isinstance(transport, ConcurrentAbortTransport)
    runtime.prepare_world_rebuild()
    completed: list[str] = []
    errors: list[BaseException] = []

    def run_abort() -> None:
        try:
            runtime.abort_world_rebuild()
            completed.append("ok")
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            errors.append(exc)

    first = Thread(target=run_abort)
    second = Thread(target=run_abort)
    try:
        first.start()
        second.start()
        assert transport.both_abort_subscribed.wait(timeout=2.0)
        transport.release_abort_subscribe.set()
        first.join(timeout=2.0)
        second.join(timeout=2.0)

        assert not first.is_alive() and not second.is_alive()
        assert completed == ["ok"]
        assert len(errors) == 1
        assert "world changed while aborting rebuild" in str(errors[0])
        assert runtime.bound_robot_id == robot.robot_id
        assert runtime.status_snapshot(wall_time=clock()).command.state == "waiting_command"
        active_subscriptions = [
            subscription
            for _topic, _type_name, subscription in transport.subscriptions
            if not subscription.closed
        ]
        assert len(active_subscriptions) == 1
        payload = ProtoCodec().encode(WheelCommand(401, (1.0, 1.0), ()))
        assert active_subscriptions[0].callback(payload, clock()) is True
        assert runtime.status_snapshot(wall_time=clock()).command.valid_count == 1
    finally:
        transport.release_abort_subscribe.set()
        first.join(timeout=2.0)
        second.join(timeout=2.0)
        runtime.close()


def test_old_valid_command_log_rejection_cannot_degrade_rebuilt_tracker() -> None:
    command_topic = "/sim/wheel/command"
    logger = BlockingRejectingLogger(message_topic=command_topic)
    runtime, _robot, transport, clock = make_runtime(logger=logger)
    payload = ProtoCodec().encode(WheelCommand(7, (1.0, 2.0), ()))
    callback_results: list[object] = []
    callback_thread = Thread(
        target=lambda: callback_results.append(
            transport.emit(command_topic, payload, clock())
        ),
        daemon=True,
    )
    try:
        callback_thread.start()
        assert logger.entered.wait(timeout=2.0)
        runtime.prepare_world_rebuild()
        runtime.commit_world_rebuild(Robot(21), Backend(), scene_document())

        logger.release.set()
        callback_thread.join(timeout=2.0)
        assert not callback_thread.is_alive()
        assert callback_results == [True]
        snapshot = runtime.status_snapshot(wall_time=clock())
        assert snapshot.command.state == "waiting_command"
        command_status = snapshot.topics[command_topic]
        assert command_status.state == "active"
        assert command_status.dropped_count == 0
        assert command_status.message_count == 0
    finally:
        logger.release.set()
        callback_thread.join(timeout=2.0)
        runtime.close()


@pytest.mark.parametrize(
    ("payload", "event_name"),
    (
        (b"not protobuf", "protobuf_parse_failed"),
        (
            ProtoCodec().encode(WheelCommand(8, (1.0, 2.0, 3.0), ())),
            "model_mismatch",
        ),
    ),
)
def test_old_command_event_rejection_cannot_degrade_rebuilt_tracker(
    payload: bytes,
    event_name: str,
) -> None:
    command_topic = "/sim/wheel/command"
    logger = BlockingRejectingLogger(event_name=event_name)
    runtime, _robot, transport, clock = make_runtime(logger=logger)
    callback_results: list[object] = []
    callback_thread = Thread(
        target=lambda: callback_results.append(
            transport.emit(command_topic, payload, clock())
        ),
        daemon=True,
    )
    try:
        callback_thread.start()
        assert logger.entered.wait(timeout=2.0)
        runtime.prepare_world_rebuild()
        runtime.commit_world_rebuild(Robot(22), Backend(), scene_document())

        logger.release.set()
        callback_thread.join(timeout=2.0)
        assert not callback_thread.is_alive()
        assert callback_results == [False]
        assert any(event == event_name for event, _fields in logger.events)
        snapshot = runtime.status_snapshot(wall_time=clock())
        assert snapshot.command.state == "waiting_command"
        command_status = snapshot.topics[command_topic]
        assert command_status.state == "active"
        assert command_status.dropped_count == 0
        assert command_status.error_count == 0
    finally:
        logger.release.set()
        callback_thread.join(timeout=2.0)
        runtime.close()


def test_old_sensor_event_rejection_cannot_degrade_rebuilt_tracker() -> None:
    logger = BlockingRejectingLogger(event_name="sensor_failed")
    runtime, _robot, _transport, clock = make_runtime(
        backend=Backend(),
        document=scene_document(),
        logger=logger,
    )
    runtime._front_lidar = StubLidar(
        "lidar_front",
        1,
        RuntimeError("old front sensor failed"),
    )
    runtime._rear_lidar = StubLidar("lidar_rear", 2)
    runtime._truth_sensor_suite = StubTruth()
    after_errors: list[BaseException] = []

    def run_after() -> None:
        try:
            runtime.after_physics_step(0.1)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            after_errors.append(exc)

    after_thread = Thread(target=run_after, daemon=True)
    try:
        clock.advance(0.1)
        after_thread.start()
        assert logger.entered.wait(timeout=2.0)
        runtime.prepare_world_rebuild()
        runtime.commit_world_rebuild(Robot(23), Backend(), scene_document())

        logger.release.set()
        after_thread.join(timeout=2.0)
        assert not after_thread.is_alive()
        assert after_errors == []
        front_topic = runtime.config.lidar_front.topic
        front_status = runtime.status_snapshot(wall_time=clock()).topics[front_topic]
        assert front_status.state == "active"
        assert front_status.dropped_count == 0
        assert any(event == "sensor_failed" for event, _fields in logger.events)
    finally:
        logger.release.set()
        after_thread.join(timeout=2.0)
        runtime.close()


class BlockingLidar:
    """阻塞昂贵扫描，允许测试在扫描期间切换生命周期代际。"""

    def __init__(self) -> None:
        self.entered = Event()
        self.release = Event()

    def scan_with_top_view(self, timestamp_ns: int) -> LidarScanResult:
        self.entered.set()
        assert self.release.wait(timeout=3.0)
        message = LidarPointCloud(timestamp_ns, "lidar_front", 0, 1, ())
        return LidarScanResult(message, LidarTopViewFrame(timestamp_ns, ()))

    def scan(self, timestamp_ns: int) -> LidarPointCloud:
        return self.scan_with_top_view(timestamp_ns).message


def test_expensive_old_sensor_scan_cannot_publish_after_rebuild() -> None:
    runtime, _robot, transport, clock = make_runtime(
        backend=Backend(),
        document=scene_document(),
    )
    blocking = BlockingLidar()
    runtime._front_lidar = blocking
    runtime._rear_lidar = StubLidar("lidar_rear", 2)
    runtime._truth_sensor_suite = StubTruth()
    errors: list[BaseException] = []

    def run_after() -> None:
        try:
            runtime.after_physics_step(0.1)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            errors.append(exc)

    worker = Thread(target=run_after, daemon=True)
    try:
        clock.advance(0.1)
        worker.start()
        assert blocking.entered.wait(timeout=2.0)
        runtime.prepare_world_rebuild()
        runtime.commit_world_rebuild(Robot(8), Backend(), scene_document())
        blocking.release.set()
        worker.join(timeout=2.0)

        assert not worker.is_alive()
        assert errors == []
        assert runtime.config.lidar_front.topic not in [item[0] for item in transport.published]
    finally:
        blocking.release.set()
        worker.join(timeout=2.0)
        runtime.close()


def test_prepare_commit_waits_for_registered_old_front_publish() -> None:
    transport = BlockingFrontPublishTransport()
    logger = TimelineLogger(transport.timeline)
    runtime, _robot, _selected_transport, clock = make_runtime(
        backend=Backend(),
        document=scene_document(),
        logger=logger,
        transport=transport,
    )
    runtime._front_lidar = StubLidar("lidar_front", 1)
    runtime._rear_lidar = StubLidar("lidar_rear", 2)
    runtime._truth_sensor_suite = StubTruth()
    after_errors: list[BaseException] = []
    rebuild_errors: list[BaseException] = []
    prepare_returned = Event()
    commit_returned = Event()

    def run_after() -> None:
        try:
            runtime.after_physics_step(0.1)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            after_errors.append(exc)

    def run_rebuild() -> None:
        try:
            runtime.prepare_world_rebuild()
            transport.timeline.append("prepare_returned")
            prepare_returned.set()
            runtime.commit_world_rebuild(Robot(12), Backend(), scene_document())
            transport.timeline.append("commit_returned")
            commit_returned.set()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            rebuild_errors.append(exc)

    after_thread = Thread(target=run_after, daemon=True)
    rebuild_thread = Thread(target=run_rebuild, daemon=True)
    try:
        clock.advance(0.1)
        after_thread.start()
        assert transport.front_started.wait(timeout=2.0)
        rebuild_thread.start()

        assert not prepare_returned.wait(timeout=0.2)
        assert not commit_returned.is_set()
        assert runtime.bound_robot_id == 1

        transport.release_front.set()
        after_thread.join(timeout=2.0)
        rebuild_thread.join(timeout=2.0)
        assert not after_thread.is_alive() and not rebuild_thread.is_alive()
        assert after_errors == rebuild_errors == []
        assert transport.timeline.index("front_accepted") < transport.timeline.index(
            "commit_returned"
        )
        snapshot = runtime.status_snapshot(wall_time=clock())
        front_topic = runtime.config.lidar_front.topic
        assert snapshot.topics[front_topic].message_count == 0
        assert snapshot.topics[front_topic].latest_timestamp_ns is None
        front_records = [
            record for record in logger.messages if record.topic == front_topic
        ]
        assert len(front_records) == 1
        assert front_records[0].direction == "publish"
        assert front_records[0].sim_time_ns == 100_000_000
        assert transport.timeline.index("front_accepted") < transport.timeline.index(
            "front_logged"
        ) < transport.timeline.index("commit_returned")
    finally:
        transport.release_front.set()
        after_thread.join(timeout=2.0)
        rebuild_thread.join(timeout=2.0)
        runtime.close()


def test_prepare_barrier_unregisters_front_publish_that_raises() -> None:
    transport = BlockingFrontPublishTransport(
        publish_error=RuntimeError("front publish failed"),
    )
    logger = TimelineLogger(transport.timeline)
    runtime, _robot, _selected_transport, clock = make_runtime(
        backend=Backend(),
        document=scene_document(),
        logger=logger,
        transport=transport,
    )
    runtime._front_lidar = StubLidar("lidar_front", 1)
    runtime._rear_lidar = StubLidar("lidar_rear", 2)
    runtime._truth_sensor_suite = StubTruth()
    prepare_returned = Event()
    commit_returned = Event()
    after_errors: list[BaseException] = []
    rebuild_errors: list[BaseException] = []

    def run_after() -> None:
        try:
            runtime.after_physics_step(0.1)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            after_errors.append(exc)

    def run_rebuild() -> None:
        try:
            runtime.prepare_world_rebuild()
            prepare_returned.set()
            runtime.commit_world_rebuild(Robot(13), Backend(), scene_document())
            transport.timeline.append("commit_returned")
            commit_returned.set()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            rebuild_errors.append(exc)

    after_thread = Thread(target=run_after, daemon=True)
    prepare_thread = Thread(target=run_rebuild, daemon=True)
    try:
        clock.advance(0.1)
        after_thread.start()
        assert transport.front_started.wait(timeout=2.0)
        prepare_thread.start()
        assert not prepare_returned.wait(timeout=0.2)

        transport.release_front.set()
        after_thread.join(timeout=2.0)
        prepare_thread.join(timeout=2.0)
        assert not after_thread.is_alive() and not prepare_thread.is_alive()
        assert prepare_returned.is_set()
        assert commit_returned.is_set()
        assert after_errors == rebuild_errors == []
        topic = runtime.config.lidar_front.topic
        status = runtime.status_snapshot(wall_time=clock()).topics[topic]
        assert status.state == "active"
        assert status.error_count == 0
        assert status.message_count == 0
        assert status.latest_timestamp_ns is None
        publish_events = [item for item in logger.events if item[0] == "publish_failed"]
        assert len(publish_events) == 1
        assert publish_events[0][1]["topic"] == topic
        assert "front publish failed" in str(publish_events[0][1]["reason"])
        assert publish_events[0][1]["sim_time_ns"] == 100_000_000
        assert transport.timeline.index("front_raised") < transport.timeline.index(
            "publish_failed_logged"
        ) < transport.timeline.index("commit_returned")
    finally:
        transport.release_front.set()
        after_thread.join(timeout=2.0)
        prepare_thread.join(timeout=2.0)
        runtime.close()


class FailingRobot(Robot):
    def __init__(self, trace: list[str]) -> None:
        super().__init__()
        self.trace = trace

    def hold_current_steering_and_stop_drive(self, _dt: float) -> None:
        self.trace.append("robot.safe_stop")
        raise RuntimeError("first close failure")


class FailingBackend(Backend):
    def close(self) -> None:
        self.trace.append("backend.close")
        raise RuntimeError("sensor close failure")


class FailingLogger(Logger):
    def close(self) -> None:
        self.trace.append("logger.close")
        raise RuntimeError("logger close failure")


def test_close_is_idempotent_best_effort_and_preserves_strict_logical_trace() -> None:
    from slope_sim.interfaces.runtime import InterfaceRuntime
    from slope_sim.interfaces.config import InterfaceConfig

    trace: list[str] = []
    transport = Transport()
    clock = Clock()
    robot = FailingRobot(trace)
    backend = FailingBackend(trace)
    logger = FailingLogger(trace=trace)
    runtime = InterfaceRuntime(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
        sensor_backend=backend,
        scene_document=scene_document(),
        logger=logger,
    )
    transport.close = lambda: trace.append("transport.close")
    try:
        with pytest.raises(RuntimeError, match="first close failure"):
            runtime.close()
        runtime.close()

        assert runtime.close_trace == (
            "stop_commands",
            "safe_stop",
            "stop_sensors",
            "quiesce_transport",
            "close_log",
            "close_transport",
            "close_sensors",
        )
        assert trace == [
            "robot.safe_stop",
            "logger.close",
            "transport.close",
            "backend.close",
        ]
        assert transport.trace[-1] == "subscription.close"
    finally:
        try:
            runtime.close()
        except RuntimeError:
            pass
