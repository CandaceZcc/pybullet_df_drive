# 接口运行时单元测试：锁定轮子闭环、状态统计及并发生命周期边界。
from __future__ import annotations

from collections import deque
from dataclasses import replace
import json
from threading import Event, Lock, Thread
import time

import pytest

import slope_sim.interfaces.runtime as runtime_module
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.dashboard_snapshot import LidarTopViewFrame, LidarTopViewPoint
from slope_sim.interfaces.logging import InterfaceEventLogger
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPoint,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.interfaces.transport import (
    LocalTransport,
    TransportSnapshot,
    TransportTopicQuality,
)
from slope_sim.lidar_pointcloud import LidarScanResult
from slope_sim.model_registry import RobotModelSpec, get_robot_model
from slope_sim.scene_config import SceneDocument, SensorDocument, TerrainDocument
from slope_sim.sensor_backend import Pose


class FakeMonotonic:
    """提供可显式推进的确定性墙钟。"""

    def __init__(self, initial: float = 0.0) -> None:
        self.value = initial

    def __call__(self) -> float:
        return self.value

    def advance(self, dt: float) -> float:
        self.value += dt
        return self.value


class FakeRobot:
    """记录运行时下发，并用独立实际值模拟关节反馈。"""

    def __init__(
        self,
        model_name: str = "df_mid",
        *,
        actual_drive: tuple[float, ...] | None = None,
        actual_steering: tuple[float, ...] | None = None,
    ) -> None:
        self.model_spec: RobotModelSpec = get_robot_model(model_name)
        drive_count = len(self.model_spec.drive_joint_names)
        steering_count = len(self.model_spec.steering_joint_names)
        self.actual_drive = actual_drive if actual_drive is not None else (0.0,) * drive_count
        self.actual_steering = (
            actual_steering if actual_steering is not None else (0.0,) * steering_count
        )
        self.command_calls: list[tuple[tuple[float, ...], tuple[float, ...], float]] = []
        self.read_timestamps: list[int] = []
        self.read_outcomes: deque[BaseException | None] = deque()
        self.safe_stop_count = 0
        self.safe_stop_actuals: list[tuple[float, ...]] = []
        self.safe_stop_started: Event | None = None
        self.release_safe_stop: Event | None = None
        self.safe_stop_error: BaseException | None = None

    def command_wheel_speeds(
        self,
        drive_wheel_speeds: tuple[float, ...],
        steering_wheel_speeds: tuple[float, ...] = (),
        dt: float = 1.0 / 240.0,
    ) -> tuple[float, ...]:
        self.command_calls.append((drive_wheel_speeds, steering_wheel_speeds, dt))
        return drive_wheel_speeds

    def hold_current_steering_and_stop_drive(self, dt: float) -> None:
        self.safe_stop_count += 1
        self.safe_stop_actuals.append(self.actual_steering)
        if self.safe_stop_started is not None:
            self.safe_stop_started.set()
        if self.release_safe_stop is not None:
            assert self.release_safe_stop.wait(timeout=5.0)
        if self.safe_stop_error is not None:
            raise self.safe_stop_error

    def read_interface_wheel_state(self, timestamp_ns: int) -> WheelState:
        self.read_timestamps.append(timestamp_ns)
        if self.read_outcomes:
            outcome = self.read_outcomes.popleft()
            if outcome is not None:
                raise outcome
        return WheelState(timestamp_ns, self.actual_drive, self.actual_steering)


class FakeTransport:
    """可注入发布和关闭结果的线程安全窄传输。"""

    def __init__(
        self,
        publish_outcomes: tuple[object, ...] = (),
        *,
        mode: str = "local",
    ) -> None:
        self.mode = mode
        self.publish_outcomes = deque(publish_outcomes)
        self.topic_publish_outcomes: dict[str, deque[object]] = {}
        self.published: list[tuple[str, bytes, str, int, float | None]] = []
        self.quiesce_count = 0
        self.close_count = 0
        self.close_started: Event | None = None
        self.release_close: Event | None = None
        self.close_error: BaseException | None = None
        self.publish_started: Event | None = None
        self.release_publish: Event | None = None
        self._lock = Lock()
        self.subscriptions: list[FakeSubscription] = []

    def subscribe(self, topic: str, type_name: str, callback):
        """保存运行时命令回调，并返回可重复关闭的订阅。"""
        subscription = FakeSubscription(topic, type_name, callback)
        self.subscriptions.append(subscription)
        return subscription

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
        if self.publish_started is not None:
            self.publish_started.set()
        if self.release_publish is not None:
            assert self.release_publish.wait(timeout=5.0)
        topic_outcomes = self.topic_publish_outcomes.get(topic)
        outcome = (
            topic_outcomes.popleft()
            if topic_outcomes
            else self.publish_outcomes.popleft()
            if self.publish_outcomes
            else True
        )
        if isinstance(outcome, BaseException):
            raise outcome
        return bool(outcome)

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
        """记录运行时关闭屏障并返回最终 fake 快照。"""
        with self._lock:
            self.quiesce_count += 1
        return self.snapshot()

    def close(self) -> None:
        with self._lock:
            self.close_count += 1
        if self.close_started is not None:
            self.close_started.set()
        if self.release_close is not None:
            assert self.release_close.wait(timeout=5.0)
        if self.close_error is not None:
            raise self.close_error


class FakeSubscription:
    """测试传输使用的最小订阅屏障。"""

    def __init__(self, topic: str, type_name: str, callback) -> None:
        self.topic = topic
        self.type_name = type_name
        self.callback = callback
        self.close_count = 0

    def close(self) -> None:
        self.close_count += 1


class DashboardLidar:
    """为 runtime 快照测试生成同一时刻、同一顺序的点云和俯视点。"""

    def __init__(self, frame_id: str, lidar_id: int) -> None:
        self.frame_id = frame_id
        self.lidar_id = lidar_id

    def scan_with_top_view(self, timestamp_ns: int) -> LidarScanResult:
        value = float(timestamp_ns)
        message = LidarPointCloud(
            timestamp_ns,
            self.frame_id,
            1,
            self.lidar_id,
            (LidarPoint(0, value, 0.0, 0.0, 100, 1, 0),),
        )
        top_view = LidarTopViewFrame(
            timestamp_ns,
            (LidarTopViewPoint(value, 0.0, 1, self.lidar_id),),
        )
        return LidarScanResult(message, top_view)

    def scan(self, timestamp_ns: int) -> LidarPointCloud:
        return self.scan_with_top_view(timestamp_ns).message


class LegacyOnlyLidar:
    """仅暴露旧 scan 的非空雷达，用于拒绝伪造俯视帧的兼容路径。"""

    def __init__(self) -> None:
        self.scan_calls = 0

    def scan(self, timestamp_ns: int) -> LidarPointCloud:
        self.scan_calls += 1
        point = LidarPoint(0, 1.0, 0.0, 0.0, 100, 1, 0)
        return LidarPointCloud(timestamp_ns, "lidar_front", 1, 1, (point,))


class MessageOnlyLidar:
    """记录 headless runtime 是否只调用企业点云入口。"""

    def __init__(self, frame_id: str, lidar_id: int) -> None:
        self.frame_id = frame_id
        self.lidar_id = lidar_id
        self.scan_timestamps: list[int] = []
        self.top_view_timestamps: list[int] = []

    def scan(self, timestamp_ns: int) -> LidarPointCloud:
        self.scan_timestamps.append(timestamp_ns)
        point = LidarPoint(0, 1.0, 0.0, 0.0, 100, 1, 0)
        return LidarPointCloud(
            timestamp_ns,
            self.frame_id,
            1,
            self.lidar_id,
            (point,),
        )

    def scan_with_top_view(self, timestamp_ns: int) -> LidarScanResult:
        self.top_view_timestamps.append(timestamp_ns)
        raise AssertionError("headless runtime must not build a LiDAR top view")


class AtomicPreferredLidar(MessageOnlyLidar):
    """即使旧增量入口仍存在，生产 runtime 也必须选择原子整帧扫描。"""

    def begin_incremental_scan(self, _timestamp_ns: int, *, capture_top_view: bool):
        raise AssertionError(
            f"runtime must not split one lidar frame across physics steps: {capture_top_view}"
        )


class DashboardTruthSensors:
    """为 runtime 快照测试提供随仿真时间变化的 RTK 和 IMU。"""

    def read_rtk(self, timestamp_ns: int) -> RtkState:
        return RtkState(timestamp_ns, float(timestamp_ns), 2.0, 3.0, 0.25)

    def read_imu(self, timestamp_ns: int) -> ImuAttitude:
        return ImuAttitude(timestamp_ns, 0.1, -0.2)


class DashboardBackend:
    """仅供 rebuild commit 构造新传感器绑定的语义后端。"""

    def link_names(self) -> tuple[str, ...]:
        return ("base_link", "lidar_front_mount", "lidar_rear_mount")

    def close(self) -> None:
        pass


class HeldLidarService:
    """用事件控制 prepared 结果可见性，模拟仍在扫描的异步 worker。"""

    def __init__(self) -> None:
        self.capture_entered = Event()
        self.result_release = Event()
        self.captures: list[dict[str, object]] = []
        self.poll_count = 0
        self.close_count = 0

    def capture(self, **capture: object) -> bool:
        self.captures.append(capture)
        self.capture_entered.set()
        return True

    def poll(self) -> None:
        self.poll_count += 1
        return None

    def close(self) -> None:
        self.close_count += 1


class FrozenMountLidar:
    """process-mode 测试只暴露冻结安装位姿，不允许父进程执行扫描。"""

    def __init__(self, pose: Pose) -> None:
        self.pose = pose

    def _world_mount(self) -> Pose:
        return self.pose


def _dashboard_scene_document() -> SceneDocument:
    return SceneDocument(
        1,
        "df_mid",
        TerrainDocument("flat", 0.0, 0, "low"),
        (),
        SensorDocument.default(),
    )


def _attach_dashboard_sensors(runtime) -> None:
    runtime._front_lidar = DashboardLidar("lidar_front", 1)
    runtime._rear_lidar = DashboardLidar("lidar_rear", 2)
    runtime._truth_sensor_suite = DashboardTruthSensors()


class BlockingDecodeCodec:
    """阻塞正式 ingress 解码，暴露接收时刻是否在首次守卫内冻结。"""

    def __init__(self, delegate: object) -> None:
        self.delegate = delegate
        self.entered = Event()
        self.release = Event()

    def decode_wheel_command(self, payload: bytes) -> WheelCommand:
        self.entered.set()
        assert self.release.wait(timeout=3.0)
        return self.delegate.decode_wheel_command(payload)

    def __getattr__(self, name: str) -> object:
        return getattr(self.delegate, name)


class MutableWheelCommand(WheelCommand):
    """模拟继承冻结命令后重新携带额外可变状态的不安全输入。"""


class SubclassDecodeCodec:
    """让正式 callback 收到 codec 异常返回的 WheelCommand 子类。"""

    def __init__(self, delegate: object, command: MutableWheelCommand) -> None:
        self.delegate = delegate
        self.command = command

    def decode_wheel_command(self, _payload: bytes) -> MutableWheelCommand:
        return self.command

    def __getattr__(self, name: str) -> object:
        return getattr(self.delegate, name)


def _runtime_type():
    """延迟导入，让 RED 同时暴露缺失 runtime 和机器人端口。"""
    from slope_sim.interfaces.runtime import InterfaceRuntime

    return InterfaceRuntime


def _make_runtime(
    robot: FakeRobot,
    clock: FakeMonotonic,
    transport: FakeTransport | None = None,
):
    return _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport if transport is not None else FakeTransport(),
        monotonic=clock,
    )


def _run_close(runtime, entered: Event, returned: Event, errors: list[BaseException]) -> None:
    entered.set()
    try:
        runtime.close()
    except BaseException as exc:  # pragma: no cover - 异常在线程汇合后断言
        errors.append(exc)
    finally:
        returned.set()


def test_three_240_hz_steps_apply_command_and_publish_first_actual_state_at_10_ms():
    clock = FakeMonotonic()
    robot = FakeRobot(actual_drive=(2.7, -1.8))
    runtime = _runtime_type().local_for_robot(robot, monotonic=clock)
    emitted: list[WheelState] = []
    try:
        command = WheelCommand(1, (3.0, -2.0), ())
        assert runtime.accept_local_command(command, received_at=clock())

        for _ in range(3):
            clock.advance(1.0 / 240.0)
            decision = runtime.before_physics_step(1.0 / 240.0)
            assert decision.drive_wheel_speed_rad_s == (3.0, -2.0)
            emitted.extend(runtime.after_physics_step(1.0 / 240.0))

        assert len(emitted) == 1
        state = emitted[0]
        assert state.timestamp_ns == 10_000_000
        assert state.drive_wheel_speed_rad_s == (2.7, -1.8)
        assert state.drive_wheel_speed_rad_s != command.drive_wheel_speed_rad_s
        assert state.steering_wheel_angle_rad == ()
        assert runtime.last_wheel_state is state
        assert robot.command_calls == [
            ((3.0, -2.0), (), 1.0 / 240.0),
            ((3.0, -2.0), (), 1.0 / 240.0),
            ((3.0, -2.0), (), 1.0 / 240.0),
        ]
    finally:
        runtime.close()


def test_next_physics_step_publish_topics_preview_is_non_mutating() -> None:
    """预览只报告下一步的到期话题，不得提前推进时钟或 scheduler。"""
    clock = FakeMonotonic()
    robot = FakeRobot(actual_drive=(2.7, -1.8))
    transport = FakeTransport()
    runtime = _make_runtime(robot, clock, transport)
    time_step_sec = 1.0 / 240.0
    try:
        assert runtime.next_physics_step_publish_topics(time_step_sec) == ()
        assert runtime.next_physics_step_publish_topics(time_step_sec) == ()
        assert runtime.clock.now_ns == 0

        assert runtime.after_physics_step(time_step_sec) == ()
        assert runtime.next_physics_step_publish_topics(time_step_sec) == ()
        assert runtime.after_physics_step(time_step_sec) == ()

        expected_wheel_topic = (runtime.config.wheel_state.topic,)
        assert runtime.next_physics_step_publish_topics(time_step_sec) == (
            expected_wheel_topic
        )
        assert runtime.next_physics_step_publish_topics(time_step_sec) == (
            expected_wheel_topic
        )
        assert runtime.clock.now_ns == 8_333_333
        assert robot.read_timestamps == []
        assert transport.published == []

        states = runtime.after_physics_step(time_step_sec)
        assert [state.timestamp_ns for state in states] == [10_000_000]
        assert runtime.clock.now_ns == 12_500_000

        assert runtime.next_physics_step_publish_topics(0.1) == expected_wheel_topic
        _attach_dashboard_sensors(runtime)
        expected_output_topics = tuple(
            channel.topic
            for channel in runtime.config.channels
            if channel.direction == "publish"
        )
        predicted_topics = runtime.next_physics_step_publish_topics(0.1)
        assert predicted_topics == expected_output_topics
        assert runtime.clock.now_ns == 12_500_000

        published_before = len(transport.published)
        runtime.after_physics_step(0.1)
        actual_topics = tuple(
            dict.fromkeys(
                event[0] for event in transport.published[published_before:]
            )
        )
        assert actual_topics == predicted_topics
    finally:
        runtime.close()


def test_async_lidar_capture_does_not_wait_before_next_wheel_deadline() -> None:
    """worker 结果未就绪时，下一次 100 Hz 轮子期限仍必须按帧返回。"""
    clock = FakeMonotonic()
    service = HeldLidarService()
    transport = FakeTransport(mode="ecal")
    runtime = _runtime_type()(
        FakeRobot(actual_drive=(1.0, 2.0)),
        config=InterfaceConfig.default(transport_mode="ecal"),
        transport=transport,
        monotonic=clock,
        sensor_backend=DashboardBackend(),
        scene_document=_dashboard_scene_document(),
        capture_lidar_top_view=False,
        lidar_scan_service=service,
    )
    runtime._front_lidar = FrozenMountLidar(
        Pose((1.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0))
    )
    runtime._rear_lidar = FrozenMountLidar(
        Pose((-1.0, 0.0, 0.8), (0.0, 0.0, 1.0, 0.0))
    )
    frame_returned = Event()
    emitted: list[WheelState] = []
    errors: list[BaseException] = []

    def run_through_next_wheel_deadline() -> None:
        try:
            runtime.after_physics_step(0.05)
            emitted.extend(runtime.after_physics_step(0.01))
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            errors.append(exc)
        finally:
            frame_returned.set()

    frame_thread = Thread(target=run_through_next_wheel_deadline, daemon=True)
    try:
        frame_thread.start()
        assert service.capture_entered.wait(timeout=2.0)
        assert not service.result_release.is_set()
        assert frame_returned.wait(timeout=0.5)
        assert errors == []
        assert emitted[-1].timestamp_ns == 60_000_000
        assert service.result_release.is_set() is False
    finally:
        service.result_release.set()
        frame_thread.join(timeout=2.0)
        runtime.close()


@pytest.mark.parametrize("requested_mode", ("local", "auto"))
def test_actual_local_transport_rejects_async_lidar_service_without_using_it(
    requested_mode: str,
) -> None:
    """异步 LiDAR 只属于实际 eCAL transport，构造失败前 service 仍归入口所有。"""
    service = HeldLidarService()
    transport = FakeTransport(mode="local")
    runtime = None
    try:
        with pytest.raises(
            ValueError,
            match="lidar_scan_service requires actual transport mode 'ecal'",
        ):
            runtime = _runtime_type()(
                FakeRobot(),
                config=InterfaceConfig.default(transport_mode=requested_mode),
                transport=transport,
                sensor_backend=DashboardBackend(),
                scene_document=_dashboard_scene_document(),
                lidar_scan_service=service,
            )
    finally:
        if runtime is not None:
            runtime.close()

    assert service.poll_count == 0
    assert service.captures == []
    assert service.close_count == 0
    assert transport.close_count == 1


def test_process_mode_preview_excludes_unprepared_lidar_topic() -> None:
    """process 模式预览只报告本帧能同步发布的 wheel/RTK/IMU。"""
    service = HeldLidarService()
    runtime = _runtime_type()(
        FakeRobot(),
        config=InterfaceConfig.default(transport_mode="ecal"),
        transport=FakeTransport(mode="ecal"),
        sensor_backend=DashboardBackend(),
        scene_document=_dashboard_scene_document(),
        lidar_scan_service=service,
    )
    try:
        assert runtime.next_physics_step_publish_topics(0.1) == (
            runtime.config.wheel_state.topic,
            runtime.config.rtk.topic,
            runtime.config.imu.topic,
        )
        assert service.poll_count == 0
    finally:
        runtime.close()


def test_async_lidar_polls_once_at_frame_head_and_tail() -> None:
    """一帧只在 before 开头和 after 结尾各执行一次非阻塞 poll。"""
    service = HeldLidarService()
    clock = FakeMonotonic()
    runtime = _runtime_type()(
        FakeRobot(),
        config=InterfaceConfig.default(transport_mode="ecal"),
        transport=FakeTransport(mode="ecal"),
        monotonic=clock,
        sensor_backend=DashboardBackend(),
        scene_document=_dashboard_scene_document(),
        lidar_scan_service=service,
    )
    try:
        runtime.before_physics_step(0.001, wall_time=clock())
        assert service.poll_count == 1

        runtime.after_physics_step(0.001)
        assert service.poll_count == 2
    finally:
        runtime.close()


def test_pause_makes_both_frame_hooks_noop_without_clock_control_or_publish():
    clock = FakeMonotonic()
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = FakeTransport()
    runtime = _make_runtime(robot, clock, transport)
    try:
        assert runtime.accept_local_command(WheelCommand(1, (3.0, 4.0), ()), received_at=0.0)
        runtime.pause()
        clock.advance(0.25)

        assert runtime.before_physics_step(0.25) is None
        assert runtime.after_physics_step(0.25) == ()
        assert runtime.clock.now_ns == 0
        assert robot.command_calls == []
        assert robot.read_timestamps == []
        assert transport.published == []

        resumed = runtime.resume(wall_time=clock())
        assert resumed.timed_out
        assert runtime.last_decision is resumed
        assert runtime.paused is False
    finally:
        runtime.close()


def test_resume_failure_leaves_pause_and_last_decision_unchanged():
    clock = FakeMonotonic(1.0)
    robot = FakeRobot()
    runtime = _make_runtime(robot, clock)
    try:
        assert runtime.accept_local_command(WheelCommand(1, (1.0, 2.0), ()), received_at=1.0)
        previous = runtime.before_physics_step(1.0 / 240.0, wall_time=1.0)
        runtime.pause()

        with pytest.raises(ValueError, match="now"):
            runtime.resume(wall_time=0.5)

        assert runtime.paused is True
        assert runtime.last_decision is previous
        assert runtime.before_physics_step(1.0 / 240.0, wall_time=1.0) is None
    finally:
        runtime.close()


@pytest.mark.parametrize("mode", ("auto", "ecal"))
def test_local_factory_rejects_non_local_config(mode: str):
    config = InterfaceConfig.default(transport_mode=mode)

    with pytest.raises(ValueError, match="local"):
        _runtime_type().local_for_robot(FakeRobot(), config=config)


def test_publish_false_and_exception_keep_last_successful_state_then_success_recovers_topic():
    clock = FakeMonotonic()
    robot = FakeRobot(actual_drive=(2.7, -1.8))
    transport = FakeTransport((False, RuntimeError("publish exploded"), True))
    runtime = _make_runtime(robot, clock, transport)
    topic_name = runtime.config.wheel_state.topic
    try:
        clock.advance(0.01)
        first = runtime.after_physics_step(0.01)
        first_status = runtime.status_snapshot(wall_time=clock()).topics[topic_name]
        assert first[0].timestamp_ns == 10_000_000
        assert runtime.last_wheel_state is None
        assert first_status.state == "error"
        assert first_status.message_count == 0
        assert first_status.error_count == 1
        assert first_status.latest_timestamp_ns is None
        assert first_status.detail

        robot.actual_drive = (2.6, -1.7)
        clock.advance(0.01)
        second = runtime.after_physics_step(0.01)
        second_status = runtime.status_snapshot(wall_time=clock()).topics[topic_name]
        assert second[0].timestamp_ns == 20_000_000
        assert runtime.last_wheel_state is None
        assert second_status.message_count == 0
        assert second_status.error_count == 2
        assert "publish exploded" in second_status.detail

        robot.actual_drive = (2.5, -1.6)
        clock.advance(0.01)
        third = runtime.after_physics_step(0.01)
        recovered = runtime.status_snapshot(wall_time=clock()).topics[topic_name]
        assert third[0] is runtime.last_wheel_state
        assert runtime.last_wheel_state.timestamp_ns == 30_000_000
        assert runtime.last_wheel_state.drive_wheel_speed_rad_s == (2.5, -1.6)
        assert recovered.state == "active"
        assert recovered.detail == ""
        assert recovered.message_count == 1
        assert recovered.error_count == 2
        assert recovered.dropped_count == 0
        assert recovered.latest_timestamp_ns == 30_000_000
        assert transport.published[-1][2] == "slope_sim.interfaces.v1.WheelState"
        assert transport.published[-1][3] == 30_000_000
    finally:
        runtime.close()


def test_joint_read_error_skips_half_message_and_later_success_recovers():
    clock = FakeMonotonic()
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    robot.read_outcomes.extend((RuntimeError("joint read failed"), None))
    transport = FakeTransport()
    runtime = _make_runtime(robot, clock, transport)
    topic_name = runtime.config.wheel_state.topic
    try:
        clock.advance(0.01)
        assert runtime.after_physics_step(0.01) == ()
        failed = runtime.status_snapshot(wall_time=clock()).topics[topic_name]
        assert runtime.last_wheel_state is None
        assert transport.published == []
        assert failed.state == "error"
        assert failed.error_count == 1
        assert "joint read failed" in failed.detail

        clock.advance(0.01)
        states = runtime.after_physics_step(0.01)
        recovered = runtime.status_snapshot(wall_time=clock()).topics[topic_name]
        assert len(states) == 1
        assert states[0].timestamp_ns == 20_000_000
        assert recovered.state == "active"
        assert recovered.message_count == 1
        assert recovered.error_count == 1
        assert recovered.detail == ""
    finally:
        runtime.close()


def test_initial_local_status_is_available_without_claiming_ecal_connection():
    clock = FakeMonotonic()
    runtime = _make_runtime(FakeRobot(), clock)
    try:
        snapshot = runtime.status_snapshot(wall_time=clock())
        topic = snapshot.topics[runtime.config.wheel_state.topic]

        assert snapshot.transport_mode == "local"
        assert snapshot.ecal_connected is False
        assert snapshot.command.state == "waiting_command"
        assert snapshot.wheel_state is None
        assert tuple(snapshot.topics) == tuple(
            channel.topic for channel in runtime.config.channels
        )
        assert topic.direction == "publish"
        assert topic.state == "active"
        assert topic.target_hz == 100.0
        assert topic.actual_hz == 0.0
        assert topic.latest_timestamp_ns is None
        assert topic.message_count == topic.error_count == topic.dropped_count == 0
    finally:
        runtime.close()


def test_runtime_applies_each_ecal_peer_state_only_to_its_own_topic():
    """命令 peer 只驱动安全代际，输出 peer 缺失只能影响对应输出。"""
    config = InterfaceConfig.default(transport_mode="ecal")

    class TopicPeerTransport(FakeTransport):
        def __init__(self) -> None:
            super().__init__()
            self.current_snapshot = TransportSnapshot(
                mode="ecal",
                ecal_connected=True,
                published_count=0,
                received_count=0,
                error_count=0,
                dropped_count=0,
                topic_quality=tuple(
                    TransportTopicQuality(
                        channel.topic,
                        peer_connected=channel is not config.lidar_front,
                    )
                    for channel in config.channels
                ),
            )

        def snapshot(self) -> TransportSnapshot:
            return self.current_snapshot

        def poll_peer_state(self) -> str:
            return "active"

    clock = FakeMonotonic(initial=1.0)
    transport = TopicPeerTransport()
    runtime = _runtime_type()(
        FakeRobot(),
        config=config,
        transport=transport,
        monotonic=clock,
    )
    try:
        runtime.initialize_peer_lifecycle(
            "ecal",
            "waiting_peer",
            ecal_connected=True,
        )
        payload = ProtoCodec().encode(WheelCommand(1, (2.0, 2.0), ()))
        assert transport.subscriptions[0].callback(payload, clock()) is True

        runtime.poll_transport()
        topics = runtime.status_snapshot().topics

        assert topics[config.wheel_command.topic].state == "active"
        assert topics[config.wheel_state.topic].state == "active"
        assert topics[config.lidar_front.topic].state == "waiting_peer"
    finally:
        runtime.close()


@pytest.mark.parametrize(
    ("target_name", "quality_state", "error_count", "dropped_count", "detail"),
    (
        (
            "wheel_command",
            "error",
            1,
            0,
            "eCAL command delivery failed",
        ),
        (
            "lidar_front",
            "error",
            1,
            0,
            "eCAL ProtoPublisher.send returned False",
        ),
        (
            "lidar_front",
            "degraded",
            0,
            1,
            "eCAL publisher queue replaced an older frame",
        ),
    ),
)
def test_runtime_active_transport_fault_outweighs_disconnected_topic_peer(
    target_name,
    quality_state,
    error_count,
    dropped_count,
    detail,
):
    """活动发送错误或队列降级必须优先于同话题的对端缺失提示。"""
    config = InterfaceConfig.default(transport_mode="ecal")
    target_topic = getattr(config, target_name).topic

    class DisconnectedErrorTransport(FakeTransport):
        def __init__(self) -> None:
            super().__init__()
            self.current_snapshot = TransportSnapshot(
                mode="ecal",
                ecal_connected=False,
                published_count=0,
                received_count=0,
                error_count=error_count,
                dropped_count=dropped_count,
                topic_quality=tuple(
                    TransportTopicQuality(
                        channel.topic,
                        error_count=error_count if channel.topic == target_topic else 0,
                        dropped_count=(
                            dropped_count if channel.topic == target_topic else 0
                        ),
                        state=quality_state if channel.topic == target_topic else "active",
                        detail=detail if channel.topic == target_topic else "",
                        revision=1,
                        peer_connected=channel.topic != target_topic,
                    )
                    for channel in config.channels
                ),
            )

        def snapshot(self) -> TransportSnapshot:
            return self.current_snapshot

        def poll_peer_state(self) -> str:
            return "active"

    clock = FakeMonotonic(initial=1.0)
    runtime = _runtime_type()(
        FakeRobot(),
        config=config,
        transport=DisconnectedErrorTransport(),
        monotonic=clock,
    )
    try:
        runtime.initialize_peer_lifecycle(
            "ecal",
            "disconnected",
            ecal_connected=False,
        )
        runtime.poll_transport()

        topic = runtime.status_snapshot().topics[target_topic]

        assert topic.state == quality_state
        assert topic.error_count == error_count
        assert topic.dropped_count == dropped_count
        assert topic.detail == detail
    finally:
        runtime.close()


def test_runtime_command_frequency_uses_interface_status_window() -> None:
    clock = FakeMonotonic()
    robot = FakeRobot()
    base_config = InterfaceConfig.default(transport_mode="local")
    config = replace(base_config, status_window_sec=1.0)
    runtime = _runtime_type()(
        robot,
        config=config,
        transport=FakeTransport(),
        monotonic=clock,
    )
    try:
        assert runtime.accept_local_command(
            WheelCommand(1, (1.0, 2.0), ()), received_at=0.0
        )
        assert runtime.accept_local_command(
            WheelCommand(2, (2.0, 3.0), ()), received_at=1.0
        )

        snapshot = runtime.status_snapshot(wall_time=1.5)
        assert snapshot.command.valid_hz == 0.0
        assert snapshot.topics[config.wheel_command.topic].actual_hz == 0.0
    finally:
        runtime.close()


def test_runtime_command_frequency_has_only_mailbox_source() -> None:
    runtime = _make_runtime(FakeRobot(), FakeMonotonic())
    try:
        command_tracker = runtime._topics[runtime.config.wheel_command.topic]
        assert command_tracker.frequency is None
    finally:
        runtime.close()


def test_same_model_rebind_invalidates_old_ingress_and_starts_fresh_waiting_mailbox():
    clock = FakeMonotonic()
    old_robot = FakeRobot("df_back")
    new_robot = FakeRobot("df_back")
    runtime = _make_runtime(old_robot, clock)
    old_mailbox, old_generation = runtime.capture_command_ingress()
    try:
        runtime.rebind_robot(new_robot)

        assert not old_mailbox.accept(
            WheelCommand(1, (1.0, 2.0), ()),
            received_at=clock(),
            generation=old_generation,
        )
        waiting = runtime.status_snapshot(wall_time=clock()).command
        assert waiting.state == "waiting_command"
        assert waiting.valid_count == 0
        assert waiting.invalid_count == 0
        assert waiting.latest_timestamp_ns is None
        assert runtime.last_wheel_state is None
        assert old_robot.safe_stop_count == 1

        assert runtime.accept_local_command(WheelCommand(2, (3.0, 4.0), ()), received_at=clock())
        runtime.before_physics_step(1.0 / 240.0, wall_time=clock())
        assert old_robot.command_calls == []
        assert new_robot.command_calls == [((3.0, 4.0), (), 1.0 / 240.0)]
    finally:
        runtime.close()


def test_rebind_construction_or_old_parking_failure_never_commits_mixed_robot_state():
    clock = FakeMonotonic()
    old_robot = FakeRobot("df_mid")
    runtime = _make_runtime(old_robot, clock)
    try:
        old_mailbox, old_generation = runtime.capture_command_ingress()
        invalid_robot = FakeRobot("df_mid")
        invalid_robot.model_spec = object()
        with pytest.raises(ValueError, match="model"):
            runtime.rebind_robot(invalid_robot)
        assert old_mailbox.accept(
            WheelCommand(1, (1.0, 2.0), ()),
            received_at=clock(),
            generation=old_generation,
        )
        previous_decision = runtime.before_physics_step(1.0 / 240.0, wall_time=clock())
        previous_status = runtime.status_snapshot(wall_time=clock()).command

        old_robot.safe_stop_error = RuntimeError("parking failed")
        new_robot = FakeRobot("active_steering_4wd")
        try:
            with pytest.raises(RuntimeError, match="parking failed"):
                runtime.rebind_robot(new_robot)
        finally:
            old_robot.safe_stop_error = None

        assert runtime.robot_model is old_robot.model_spec
        assert new_robot.command_calls == []
        assert old_mailbox.capture_generation() == old_generation
        assert runtime.last_decision is previous_decision
        assert runtime.status_snapshot(wall_time=clock()).command == previous_status
        assert old_mailbox.accept(
            WheelCommand(2, (3.0, 4.0), ()),
            received_at=clock(),
            generation=old_generation,
        )
        runtime.before_physics_step(1.0 / 240.0, wall_time=clock())
        assert old_robot.command_calls[-1][0] == (3.0, 4.0)
    finally:
        runtime.close()


def test_rebind_post_stop_commit_failure_faults_without_mixed_ingress():
    """停车后的意外提交异常必须 fail-closed，不能重新激活旧 token。"""

    class InjectedRebindFailure(BaseException):
        pass

    clock = FakeMonotonic()
    old_robot = FakeRobot("df_mid")
    new_robot = FakeRobot("active_steering_4wd")
    transport = FakeTransport()
    runtime = _make_runtime(old_robot, clock, transport)
    old_mailbox, old_generation = runtime.capture_command_ingress()
    old_subscription = transport.subscriptions[0]
    original_clear = runtime._clear_dashboard_payloads_locked

    def fail_after_dashboard_clear() -> None:
        original_clear()
        raise InjectedRebindFailure("injected post-stop commit failure")

    runtime._clear_dashboard_payloads_locked = fail_after_dashboard_clear
    try:
        with pytest.raises(InjectedRebindFailure, match="post-stop commit failure"):
            runtime.rebind_robot(new_robot)
    finally:
        runtime.__dict__.pop("_clear_dashboard_payloads_locked", None)

    candidate_subscription = transport.subscriptions[1]
    try:
        assert runtime._state == "faulted"
        assert runtime.robot_model is old_robot.model_spec
        assert runtime._robot is old_robot
        assert runtime._mailbox is old_mailbox
        assert runtime._command_subscription is old_subscription
        assert runtime._active_subscription_token is None
        assert not runtime._accepting_commands
        assert not runtime._world_ready
        assert old_mailbox.capture_generation() > old_generation
        assert old_subscription.close_count == 0
        assert candidate_subscription.close_count == 1

        payload = runtime._codec.encode(WheelCommand(2, (3.0, 4.0), ()))
        assert old_subscription.callback(payload, clock()) is None
        assert candidate_subscription.callback(payload, clock()) is None
        with pytest.raises(RuntimeError, match="faulted"):
            runtime.accept_local_command(
                WheelCommand(3, (1.0, 2.0), ()),
                received_at=clock(),
            )
    finally:
        runtime.close()

    assert old_subscription.close_count == 1
    assert candidate_subscription.close_count == 1


def test_rebind_partial_subscription_commit_faults_without_mixed_ingress(
    monkeypatch,
) -> None:
    """候选句柄已写但 token 未写时，任意异常也必须关闭两代准入。"""

    class InjectedPartialSubscriptionFailure(BaseException):
        pass

    class FailOnceAfterCandidateWrite:
        """精确在候选 subscription 落盘后注入一次提交异常。"""

        failed = False

        def __get__(self, instance, owner=None):
            if instance is None:
                return self
            return instance.__dict__["_command_subscription"]

        def __set__(self, instance, value) -> None:
            instance.__dict__["_command_subscription"] = value
            if not self.failed:
                self.failed = True
                raise InjectedPartialSubscriptionFailure(
                    "injected partial subscription commit failure"
                )

    clock = FakeMonotonic()
    old_robot = FakeRobot("df_mid")
    new_robot = FakeRobot("active_steering_4wd")
    transport = FakeTransport()
    runtime = _make_runtime(old_robot, clock, transport)
    old_mailbox, old_generation = runtime.capture_command_ingress()
    old_subscription = transport.subscriptions[0]
    descriptor = FailOnceAfterCandidateWrite()
    monkeypatch.setattr(
        type(runtime),
        "_command_subscription",
        descriptor,
        raising=False,
    )

    with pytest.raises(
        InjectedPartialSubscriptionFailure,
        match="partial subscription commit failure",
    ):
        runtime.rebind_robot(new_robot)

    candidate_subscription = transport.subscriptions[1]
    try:
        assert descriptor.failed is True
        assert runtime._state == "faulted"
        assert runtime._robot is old_robot
        assert runtime.robot_model is old_robot.model_spec
        assert runtime._mailbox is old_mailbox
        assert runtime._command_subscription is old_subscription
        assert runtime._active_subscription_token is None
        assert runtime._accepting_commands is False
        assert runtime._world_ready is False
        assert old_mailbox.capture_generation() > old_generation
        assert old_subscription.close_count == 0
        assert candidate_subscription.close_count == 1

        payload = runtime._codec.encode(WheelCommand(2, (3.0, 4.0), ()))
        assert old_subscription.callback(payload, clock()) is None
        assert candidate_subscription.callback(payload, clock()) is None
    finally:
        runtime.close()

    assert old_subscription.close_count == 1
    assert candidate_subscription.close_count == 1


def test_two_concurrent_closes_share_one_blocking_transport_barrier():
    clock = FakeMonotonic()
    robot = FakeRobot()
    transport = FakeTransport()
    transport.close_started = Event()
    transport.release_close = Event()
    runtime = _make_runtime(robot, clock, transport)
    first_entered, first_returned = Event(), Event()
    second_entered, second_returned = Event(), Event()
    errors: list[BaseException] = []
    first = Thread(target=_run_close, args=(runtime, first_entered, first_returned, errors), daemon=True)
    second = Thread(target=_run_close, args=(runtime, second_entered, second_returned, errors), daemon=True)
    try:
        first.start()
        assert first_entered.wait(timeout=2.0)
        assert transport.close_started.wait(timeout=2.0)
        second.start()
        assert second_entered.wait(timeout=2.0)

        assert not first_returned.is_set()
        assert not second_returned.wait(timeout=0.1)
        closing = runtime.status_snapshot(wall_time=clock())
        closing_topic = closing.topics[runtime.config.wheel_state.topic]
        assert closing.command.state == "disconnected"
        assert closing_topic.state == "disconnected"
        assert runtime.last_decision.waiting
        assert runtime.last_decision.drive_wheel_speed_rad_s == (0.0, 0.0)
        with pytest.raises(RuntimeError, match="closing"):
            runtime.accept_local_command(WheelCommand(1, (1.0, 2.0), ()), received_at=clock())

        transport.release_close.set()
        first.join(timeout=2.0)
        second.join(timeout=2.0)
        assert not first.is_alive() and not second.is_alive()
        assert errors == []
        assert transport.close_count == 1
        assert robot.safe_stop_count == 1
    finally:
        transport.release_close.set()
        first.join(timeout=2.0)
        second.join(timeout=2.0)
        runtime.close()


def test_parking_exception_still_closes_transport_and_wakes_same_result_waiter():
    clock = FakeMonotonic()
    robot = FakeRobot()
    robot.safe_stop_error = RuntimeError("safe stop failed")
    transport = FakeTransport()
    transport.close_started = Event()
    transport.release_close = Event()
    runtime = _make_runtime(robot, clock, transport)
    first_entered, first_returned = Event(), Event()
    second_entered, second_returned = Event(), Event()
    errors: list[BaseException] = []
    first = Thread(target=_run_close, args=(runtime, first_entered, first_returned, errors), daemon=True)
    second = Thread(target=_run_close, args=(runtime, second_entered, second_returned, errors), daemon=True)
    try:
        first.start()
        assert transport.close_started.wait(timeout=2.0)
        second.start()
        assert second_entered.wait(timeout=2.0)
        assert not second_returned.wait(timeout=0.1)

        transport.release_close.set()
        first.join(timeout=2.0)
        second.join(timeout=2.0)
        assert not first.is_alive() and not second.is_alive()
        assert len(errors) == 2
        assert all(isinstance(error, RuntimeError) for error in errors)
        assert all(str(error) == "safe stop failed" for error in errors)
        assert transport.close_count == 1
        assert robot.safe_stop_count == 1
    finally:
        transport.release_close.set()
        first.join(timeout=2.0)
        second.join(timeout=2.0)
        runtime.close()


def test_close_linearization_rejects_accept_and_rebind_without_mixed_resources():
    clock = FakeMonotonic()
    old_robot = FakeRobot("df_front")
    old_robot.safe_stop_started = Event()
    old_robot.release_safe_stop = Event()
    new_robot = FakeRobot("active_steering_4wd")
    transport = FakeTransport()
    runtime = _make_runtime(old_robot, clock, transport)
    mailbox, generation = runtime.capture_command_ingress()
    close_entered, close_returned = Event(), Event()
    errors: list[BaseException] = []
    closer = Thread(
        target=_run_close,
        args=(runtime, close_entered, close_returned, errors),
        daemon=True,
    )
    try:
        closer.start()
        assert old_robot.safe_stop_started.wait(timeout=2.0)

        with pytest.raises(RuntimeError, match="closing"):
            runtime.accept_local_command(WheelCommand(1, (1.0, 2.0), ()), received_at=clock())
        with pytest.raises(RuntimeError, match="closing"):
            runtime.rebind_robot(new_robot)
        assert not mailbox.accept(
            WheelCommand(2, (3.0, 4.0), ()),
            received_at=clock(),
            generation=generation,
        )
        assert runtime.robot_model is old_robot.model_spec
        assert new_robot.safe_stop_count == 0
        assert not close_returned.is_set()

        old_robot.release_safe_stop.set()
        closer.join(timeout=2.0)
        assert not closer.is_alive()
        assert errors == []
        assert old_robot.safe_stop_count == 1
        assert transport.close_count == 1
    finally:
        old_robot.release_safe_stop.set()
        closer.join(timeout=2.0)
        runtime.close()


def test_accept_waiting_behind_rebind_targets_only_the_new_mailbox_and_robot():
    clock = FakeMonotonic()
    old_robot = FakeRobot("df_back")
    old_robot.safe_stop_started = Event()
    old_robot.release_safe_stop = Event()
    new_robot = FakeRobot("df_back")
    runtime = _make_runtime(old_robot, clock)
    old_mailbox, old_generation = runtime.capture_command_ingress()
    rebind_errors: list[BaseException] = []
    accept_errors: list[BaseException] = []
    accept_result: list[bool] = []
    accept_entered = Event()
    accept_returned = Event()

    def rebind() -> None:
        try:
            runtime.rebind_robot(new_robot)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            rebind_errors.append(exc)

    def accept() -> None:
        accept_entered.set()
        try:
            accept_result.append(
                runtime.accept_local_command(
                    WheelCommand(5, (5.0, 6.0), ()),
                    received_at=clock(),
                )
            )
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            accept_errors.append(exc)
        finally:
            accept_returned.set()

    rebind_thread = Thread(target=rebind, daemon=True)
    accept_thread = Thread(target=accept, daemon=True)
    try:
        rebind_thread.start()
        assert old_robot.safe_stop_started.wait(timeout=2.0)
        accept_thread.start()
        assert accept_entered.wait(timeout=2.0)
        assert not accept_returned.wait(timeout=0.1)

        old_robot.release_safe_stop.set()
        rebind_thread.join(timeout=2.0)
        accept_thread.join(timeout=2.0)
        assert not rebind_thread.is_alive() and not accept_thread.is_alive()
        assert rebind_errors == accept_errors == []
        assert accept_result == [True]
        assert not old_mailbox.accept(
            WheelCommand(6, (1.0, 2.0), ()),
            received_at=clock(),
            generation=old_generation,
        )
        assert runtime.status_snapshot(wall_time=clock()).command.valid_count == 1

        runtime.before_physics_step(1.0 / 240.0, wall_time=clock())
        assert old_robot.command_calls == []
        assert new_robot.command_calls == [((5.0, 6.0), (), 1.0 / 240.0)]
    finally:
        old_robot.release_safe_stop.set()
        rebind_thread.join(timeout=2.0)
        accept_thread.join(timeout=2.0)
        runtime.close()


def test_after_scheduler_rejection_keeps_clock_atomic_and_next_small_step_recovers():
    clock = FakeMonotonic()
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = FakeTransport()
    runtime = _make_runtime(robot, clock, transport)
    try:
        with pytest.raises(ValueError, match="catch-up limit"):
            runtime.after_physics_step(100.01)

        assert runtime.clock.now_ns == 0
        assert robot.read_timestamps == []
        assert transport.published == []

        clock.advance(0.01)
        states = runtime.after_physics_step(0.01)
        assert tuple(state.timestamp_ns for state in states) == (10_000_000,)
        assert runtime.clock.now_ns == 10_000_000
    finally:
        runtime.close()


def test_late_imu_scheduler_rejection_does_not_mutate_earlier_schedulers():
    clock = FakeMonotonic()
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = FakeTransport()
    base_config = InterfaceConfig.default(transport_mode="local")
    config = replace(
        base_config,
        imu=replace(base_config.imu, rate_hz=1_000_000),
    )
    runtime = _runtime_type()(
        robot,
        config=config,
        transport=transport,
        monotonic=clock,
    )
    try:
        with pytest.raises(ValueError, match="catch-up limit"):
            runtime.after_physics_step(0.02)

        assert runtime.clock.now_ns == 0
        assert robot.read_timestamps == []
        assert transport.published == []

        assert runtime.after_physics_step(0.001) == ()
        assert runtime.clock.now_ns == 1_000_000
        assert robot.read_timestamps == []
        assert transport.published == []
    finally:
        runtime.close()


def test_wheel_state_callback_can_wait_for_status_then_close_without_stale_topic_writeback():
    clock = FakeMonotonic(0.03)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = LocalTransport(monotonic=clock)
    runtime = _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
    )
    status_entered = Event()
    status_finished = Event()
    callback_completed = Event()
    status_errors: list[BaseException] = []
    status_threads: list[Thread] = []

    def read_status() -> None:
        status_entered.set()
        try:
            runtime.status_snapshot(wall_time=clock())
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            status_errors.append(exc)
        finally:
            status_finished.set()

    def callback(_payload: bytes, _received_at: float) -> None:
        status_thread = Thread(target=read_status, daemon=True)
        status_threads.append(status_thread)
        status_thread.start()
        assert status_entered.wait(timeout=2.0)
        assert status_finished.wait(timeout=0.2)
        runtime.close()
        callback_completed.set()

    transport.subscribe(
        runtime.config.wheel_state.topic,
        "slope_sim.interfaces.v1.WheelState",
        callback,
    )
    try:
        states = runtime.after_physics_step(0.03)
        assert len(states) == 1
        assert states[0].timestamp_ns == 10_000_000
        assert robot.read_timestamps == [10_000_000]
        assert transport.snapshot().published_count == 1
        assert callback_completed.is_set()
        assert status_errors == []
        for status_thread in status_threads:
            status_thread.join(timeout=2.0)
            assert not status_thread.is_alive()

        snapshot = runtime.status_snapshot(wall_time=clock())
        topic = snapshot.topics[runtime.config.wheel_state.topic]
        assert snapshot.command.state == "disconnected"
        assert topic.state == "disconnected"
        assert topic.message_count == 0
        assert topic.error_count == 0
        assert topic.latest_timestamp_ns is None
    finally:
        runtime.close()
        for status_thread in status_threads:
            status_thread.join(timeout=2.0)


def test_wheel_state_callback_close_skips_stale_publish_log_after_logger_close(tmp_path):
    clock = FakeMonotonic(0.03)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = LocalTransport(monotonic=clock)
    logger = InterfaceEventLogger(tmp_path, prefix="runtime-callback-close")
    runtime = _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
        logger=logger,
    )
    callback_completed = Event()

    def callback(_payload: bytes, _received_at: float) -> None:
        runtime.close()
        callback_completed.set()

    transport.subscribe(
        runtime.config.wheel_state.topic,
        "slope_sim.interfaces.v1.WheelState",
        callback,
    )
    paths = logger.paths
    try:
        # 关闭线性化前已接受的事件必须由 logger.close 完整排空。
        runtime._record_runtime_event("sensor_failed", reason="before callback close")
        assert logger.snapshot().accepted_events == 1

        states = runtime.after_physics_step(0.03)

        assert len(states) == 1
        assert callback_completed.is_set()
        assert runtime.close_trace == (
            "stop_commands",
            "safe_stop",
            "stop_sensors",
            "quiesce_transport",
            "close_log",
            "close_transport",
            "close_sensors",
        )
        snapshot = logger.snapshot()
        assert snapshot.closed
        assert snapshot.accepted_messages == 0
        assert snapshot.dropped_messages == 0
        assert snapshot.dropped_events == 0
        assert runtime._pending_logger_drops == 0

        events = [
            json.loads(line)
            for line in paths.event_path.read_text(encoding="utf-8").splitlines()
        ]
        assert [(event["event"], event["reason"]) for event in events] == [
            ("sensor_failed", "before callback close")
        ]
        assert paths.binary_path.read_bytes() == b""
    finally:
        runtime.close()


def test_publish_callback_close_rejects_when_prepare_owns_lifecycle_transition():
    """prepare 等同步发布退出时，回调 close 必须失败而不能形成互等。"""
    clock = FakeMonotonic(0.01)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = LocalTransport(monotonic=clock)
    runtime = _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
    )
    callback_entered = Event()
    invoke_close = Event()
    callback_returned = Event()
    prepare_returned = Event()
    close_errors: list[BaseException] = []
    after_errors: list[BaseException] = []
    prepare_errors: list[BaseException] = []

    def callback(_payload: bytes, _received_at: float) -> None:
        callback_entered.set()
        assert invoke_close.wait(timeout=3.0)
        try:
            runtime.close()
        except BaseException as exc:
            close_errors.append(exc)
        finally:
            callback_returned.set()

    transport.subscribe(
        runtime.config.wheel_state.topic,
        "slope_sim.interfaces.v1.WheelState",
        callback,
    )

    def run_after() -> None:
        try:
            runtime.after_physics_step(0.01)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            after_errors.append(exc)

    def run_prepare() -> None:
        try:
            runtime.prepare_world_rebuild()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            prepare_errors.append(exc)
        finally:
            prepare_returned.set()

    after_thread = Thread(target=run_after, daemon=True)
    prepare_thread = Thread(target=run_prepare, daemon=True)
    try:
        after_thread.start()
        assert callback_entered.wait(timeout=2.0)
        prepare_thread.start()
        deadline = time.monotonic() + 2.0
        while True:
            with runtime._condition:
                transition_owned = (
                    runtime._prepare_in_progress and not runtime._world_ready
                )
                state_during_prepare = runtime._state
            if transition_owned:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                pytest.fail("prepare did not acquire the lifecycle transition")
            time.sleep(min(0.005, remaining))
        assert state_during_prepare == "open"

        invoke_close.set()
        returned_without_deadlock = callback_returned.wait(timeout=0.3)
        state_after_callback = runtime._state

        # 仅帮助旧 RED 实现退出互等，避免守护线程污染后续测试。
        if not returned_without_deadlock:
            with runtime._condition:
                runtime._prepare_in_progress = False
                runtime._condition.notify_all()

        if after_thread.ident is not None:
            after_thread.join(timeout=2.0)
        if prepare_thread.ident is not None:
            prepare_thread.join(timeout=2.0)

        assert returned_without_deadlock
        assert state_after_callback == "open"
        assert len(close_errors) == 1
        assert isinstance(close_errors[0], RuntimeError)
        assert "lifecycle transition" in str(close_errors[0])
        assert not after_thread.is_alive() and not prepare_thread.is_alive()
        assert after_errors == prepare_errors == []
        assert prepare_returned.is_set()
        assert runtime.close_trace == ()
    finally:
        invoke_close.set()
        with runtime._condition:
            if runtime._prepare_in_progress:
                runtime._prepare_in_progress = False
                runtime._condition.notify_all()
        after_thread.join(timeout=2.0)
        prepare_thread.join(timeout=2.0)
        runtime.close()


def test_publish_callback_close_rejects_when_external_close_already_owns_transition():
    """外部 close 等同步回调时，回调内嵌套 close 不能反向等待外部 owner。"""

    class ObservableLocalTransport(LocalTransport):
        def __init__(self) -> None:
            super().__init__(monotonic=clock)
            self.quiesce_entered = Event()

        def quiesce(self) -> TransportSnapshot:
            self.quiesce_entered.set()
            return super().quiesce()

    clock = FakeMonotonic(0.01)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = ObservableLocalTransport()
    runtime = _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
    )
    callback_entered = Event()
    invoke_nested_close = Event()
    callback_returned = Event()
    external_close_returned = Event()
    nested_close_errors: list[BaseException] = []
    after_errors: list[BaseException] = []
    external_close_errors: list[BaseException] = []

    def callback(_payload: bytes, _received_at: float) -> None:
        callback_entered.set()
        assert invoke_nested_close.wait(timeout=3.0)
        try:
            runtime.close()
        except BaseException as exc:
            nested_close_errors.append(exc)
        finally:
            callback_returned.set()

    transport.subscribe(
        runtime.config.wheel_state.topic,
        "slope_sim.interfaces.v1.WheelState",
        callback,
    )

    def run_after() -> None:
        try:
            runtime.after_physics_step(0.01)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            after_errors.append(exc)

    def run_external_close() -> None:
        try:
            runtime.close()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            external_close_errors.append(exc)
        finally:
            external_close_returned.set()

    after_thread = Thread(target=run_after, daemon=True)
    close_thread = Thread(target=run_external_close, daemon=True)
    try:
        after_thread.start()
        assert callback_entered.wait(timeout=2.0)
        close_thread.start()
        assert transport.quiesce_entered.wait(timeout=2.0)

        invoke_nested_close.set()
        returned_without_deadlock = callback_returned.wait(timeout=0.3)

        # 仅帮助旧 RED 实现退出互等，真正实现必须由 nested close 自行 fail-fast。
        if not returned_without_deadlock:
            with runtime._condition:
                runtime._state = "closed"
                runtime._condition.notify_all()

        after_thread.join(timeout=2.0)
        close_thread.join(timeout=2.0)

        assert returned_without_deadlock
        assert len(nested_close_errors) == 1
        assert isinstance(nested_close_errors[0], RuntimeError)
        assert "lifecycle transition" in str(nested_close_errors[0])
        assert not after_thread.is_alive() and not close_thread.is_alive()
        assert after_errors == external_close_errors == []
        assert external_close_returned.is_set()
        assert runtime._in_flight_publishes == 0
        assert runtime.close_trace == (
            "stop_commands",
            "safe_stop",
            "stop_sensors",
            "quiesce_transport",
            "close_log",
            "close_transport",
            "close_sensors",
        )
    finally:
        invoke_nested_close.set()
        with runtime._condition:
            if runtime._state == "closing":
                runtime._state = "closed"
                runtime._condition.notify_all()
        if after_thread.ident is not None:
            after_thread.join(timeout=2.0)
        if close_thread.ident is not None:
            close_thread.join(timeout=2.0)
        runtime.close()


@pytest.mark.parametrize("operation", ("prepare", "rebind", "close"))
def test_safe_stop_rejects_reentrant_close_from_lifecycle_owner(operation: str):
    """外部停车端口不得让当前生命周期 owner 同线程等待自己。"""

    class ReentrantSafeStopRobot(FakeRobot):
        def __init__(self) -> None:
            super().__init__("df_mid")
            self.runtime = None
            self.reentry_started = Event()
            self.reentry_errors: list[BaseException] = []
            self._reentry_attempted = False

        def hold_current_steering_and_stop_drive(self, dt: float) -> None:
            super().hold_current_steering_and_stop_drive(dt)
            if self._reentry_attempted:
                return
            self._reentry_attempted = True
            self.reentry_started.set()
            try:
                self.runtime.close()
            except BaseException as exc:
                self.reentry_errors.append(exc)

    clock = FakeMonotonic()
    robot = ReentrantSafeStopRobot()
    runtime = _make_runtime(robot, clock)
    robot.runtime = runtime
    operation_errors: list[BaseException] = []
    operation_returned = Event()

    def run_operation() -> None:
        try:
            if operation == "prepare":
                runtime.prepare_world_rebuild()
            elif operation == "rebind":
                runtime.rebind_robot(FakeRobot("df_mid"))
            else:
                runtime.close()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            operation_errors.append(exc)
        finally:
            operation_returned.set()

    worker = Thread(target=run_operation, daemon=True)
    returned_without_deadlock = False
    try:
        worker.start()
        assert robot.reentry_started.wait(timeout=2.0)
        returned_without_deadlock = operation_returned.wait(timeout=0.3)

        # 仅帮助旧 RED 实现退出同线程自等待，避免守护线程污染后续测试。
        if not returned_without_deadlock:
            with runtime._condition:
                runtime._state = "closed"
                runtime._prepare_in_progress = False
                runtime._rebind_in_progress = False
                runtime._condition.notify_all()
        worker.join(timeout=2.0)

        assert returned_without_deadlock
        assert not worker.is_alive()
        assert operation_errors == []
        assert len(robot.reentry_errors) == 1
        assert isinstance(robot.reentry_errors[0], RuntimeError)
        assert "lifecycle transition" in str(robot.reentry_errors[0])
    finally:
        with runtime._condition:
            forced_closed = runtime._state == "closed" and not returned_without_deadlock
        if operation == "prepare" and not forced_closed:
            runtime.abort_world_rebuild()
        worker.join(timeout=2.0)
        runtime.close()


def _release_self_waiting_publish_for_test(runtime, worker: Thread) -> None:
    """仅用于让旧 RED 实现退出自等待，避免守护线程残留到后续测试。"""
    if not worker.is_alive():
        return
    with runtime._condition:
        runtime._in_flight_publishes = 0
        runtime._condition.notify_all()
    worker.join(timeout=2.0)


def test_command_receive_logger_close_is_rejected_without_self_deadlock():
    """命令接收日志持有非重入锁时，嵌套 close 必须立即失败。"""

    class ReentrantCloseLogger:
        def __init__(self) -> None:
            self.runtime = None
            self.close_started = Event()
            self.close_errors: list[BaseException] = []

        def record_message(self, record) -> bool:
            if record.topic == "/sim/wheel/command" and record.direction == "receive":
                self.close_started.set()
                try:
                    self.runtime.close()
                except BaseException as exc:
                    self.close_errors.append(exc)
            return True

        def record_event(self, _event: str, **_fields: object) -> bool:
            return True

        def close(self) -> None:
            return None

    clock = FakeMonotonic(0.01)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = LocalTransport(monotonic=clock)
    logger = ReentrantCloseLogger()
    runtime = _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
        logger=logger,
    )
    logger.runtime = runtime
    command = WheelCommand(1, (2.0, 3.0), ())
    publish_errors: list[BaseException] = []
    publish_returned = Event()

    def publish_command() -> None:
        try:
            transport.publish(
                runtime.config.wheel_command.topic,
                runtime._codec.encode(command),
                runtime._codec.type_name(command),
                command.timestamp_ns,
                wall_time=clock(),
            )
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            publish_errors.append(exc)
        finally:
            publish_returned.set()

    worker = Thread(target=publish_command, daemon=True)
    returned_without_deadlock = False
    try:
        worker.start()
        assert logger.close_started.wait(timeout=2.0)
        returned_without_deadlock = publish_returned.wait(timeout=0.3)

        # 旧 RED 会在同一线程重取 _log_lock；测试线程只负责释放该死锁。
        if not returned_without_deadlock and runtime._log_lock.locked():
            runtime._log_lock.release()
        worker.join(timeout=2.0)

        assert returned_without_deadlock
        assert not worker.is_alive()
        assert publish_errors == []
        assert len(logger.close_errors) == 1
        assert isinstance(logger.close_errors[0], RuntimeError)
        assert "interface callback" in str(logger.close_errors[0])
        assert runtime.accept_local_command(
            WheelCommand(2, (3.0, 4.0), ()),
            received_at=clock(),
        )
    finally:
        if worker.is_alive() and runtime._log_lock.locked():
            runtime._log_lock.release()
        worker.join(timeout=2.0)
        runtime.close()


def test_prepare_from_local_transport_callback_is_rejected_without_partial_rebuild():
    clock = FakeMonotonic(0.01)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = LocalTransport(monotonic=clock)
    runtime = _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
    )
    prepare_errors: list[BaseException] = []
    after_errors: list[BaseException] = []

    def callback(_payload: bytes, _received_at: float) -> None:
        try:
            runtime.prepare_world_rebuild()
        except BaseException as exc:
            prepare_errors.append(exc)

    transport.subscribe(
        runtime.config.wheel_state.topic,
        "slope_sim.interfaces.v1.WheelState",
        callback,
    )

    def run_after() -> None:
        try:
            runtime.after_physics_step(0.01)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            after_errors.append(exc)

    worker = Thread(target=run_after, daemon=True)
    returned_in_time = False
    try:
        worker.start()
        worker.join(timeout=0.3)
        returned_in_time = not worker.is_alive()
        _release_self_waiting_publish_for_test(runtime, worker)

        assert returned_in_time
        assert after_errors == []
        assert len(prepare_errors) == 1
        assert isinstance(prepare_errors[0], RuntimeError)
        assert "in-flight interface callback" in str(prepare_errors[0])
        assert robot.safe_stop_count == 0
        assert runtime.accept_local_command(
            WheelCommand(1, (2.0, 3.0), ()), received_at=clock()
        )
    finally:
        _release_self_waiting_publish_for_test(runtime, worker)
        runtime.close()


def test_prepare_from_logger_callback_uses_same_reentrant_publish_guard():
    class PrepareLogger:
        """在 wheel publish 日志回调内尝试正式 prepare。"""

        def __init__(self) -> None:
            self.runtime = None
            self.prepare_errors: list[BaseException] = []

        def record_message(self, record) -> bool:
            if record.topic == "/sim/wheel/state":
                try:
                    self.runtime.prepare_world_rebuild()
                except BaseException as exc:
                    self.prepare_errors.append(exc)
            return True

        def record_event(self, _event: str, **_fields: object) -> bool:
            return True

        def close(self) -> None:
            return None

    clock = FakeMonotonic(0.01)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = LocalTransport(monotonic=clock)
    logger = PrepareLogger()
    runtime = _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
        logger=logger,
    )
    logger.runtime = runtime
    after_errors: list[BaseException] = []

    def run_after() -> None:
        try:
            runtime.after_physics_step(0.01)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            after_errors.append(exc)

    worker = Thread(target=run_after, daemon=True)
    returned_in_time = False
    try:
        worker.start()
        worker.join(timeout=0.3)
        returned_in_time = not worker.is_alive()
        _release_self_waiting_publish_for_test(runtime, worker)

        assert returned_in_time
        assert after_errors == []
        assert len(logger.prepare_errors) == 1
        assert isinstance(logger.prepare_errors[0], RuntimeError)
        assert "in-flight interface callback" in str(logger.prepare_errors[0])
        assert robot.safe_stop_count == 0
        assert runtime.accept_local_command(
            WheelCommand(1, (2.0, 3.0), ()), received_at=clock()
        )
    finally:
        _release_self_waiting_publish_for_test(runtime, worker)
        runtime.close()


@pytest.mark.parametrize("operation", ("commit", "abort", "fault"))
def test_publish_callback_rejects_remaining_world_lifecycle_operations(operation: str):
    """其余世界事务入口也不得从同步接口回调内启动或等待。"""
    clock = FakeMonotonic(0.01)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = LocalTransport(monotonic=clock)
    runtime = _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
    )
    lifecycle_errors: list[BaseException] = []

    def callback(_payload: bytes, _received_at: float) -> None:
        try:
            if operation == "commit":
                runtime.commit_world_rebuild(FakeRobot(), None, None)
            elif operation == "abort":
                runtime.abort_world_rebuild()
            else:
                runtime.fault_world_rebuild()
        except BaseException as exc:
            lifecycle_errors.append(exc)

    transport.subscribe(
        runtime.config.wheel_state.topic,
        "slope_sim.interfaces.v1.WheelState",
        callback,
    )
    try:
        states = runtime.after_physics_step(0.01)

        assert len(states) == 1
        assert len(lifecycle_errors) == 1
        assert isinstance(lifecycle_errors[0], RuntimeError)
        assert "in-flight interface callback" in str(lifecycle_errors[0])
        assert robot.safe_stop_count == 0
        assert runtime.accept_local_command(
            WheelCommand(1, (2.0, 3.0), ()),
            received_at=clock(),
        )
    finally:
        runtime.close()


def test_wheel_state_callback_pause_commits_current_message_then_stops_due_batch():
    clock = FakeMonotonic(0.03)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = LocalTransport(monotonic=clock)
    runtime = _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
    )
    callback_count = 0

    def callback(_payload: bytes, _received_at: float) -> None:
        nonlocal callback_count
        callback_count += 1
        runtime.pause()

    transport.subscribe(
        runtime.config.wheel_state.topic,
        "slope_sim.interfaces.v1.WheelState",
        callback,
    )
    try:
        states = runtime.after_physics_step(0.03)
        snapshot = runtime.status_snapshot(wall_time=clock())
        topic = snapshot.topics[runtime.config.wheel_state.topic]

        assert runtime.paused
        assert runtime.clock.now_ns == 30_000_000
        assert tuple(state.timestamp_ns for state in states) == (10_000_000,)
        assert robot.read_timestamps == [10_000_000]
        assert callback_count == 1
        assert transport.snapshot().published_count == 1
        assert topic.state == "active"
        assert topic.message_count == 1
        assert topic.latest_timestamp_ns == 10_000_000
        assert topic.error_count == 0
        assert topic.dropped_count == 0
    finally:
        runtime.close()


def test_paused_successful_publish_keeps_message_count_when_frequency_record_fails():
    class FailingFrequency:
        """仅在成功发布后的频率提交点注入错误。"""

        def record(self, _timestamp: float) -> None:
            raise RuntimeError("injected frequency failure")

        def hz(self, _now: float) -> float:
            return 0.0

    clock = FakeMonotonic(0.03)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = LocalTransport(monotonic=clock)
    runtime = _runtime_type()(
        robot,
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
    )
    runtime._wheel_frequency = FailingFrequency()

    def callback(_payload: bytes, _received_at: float) -> None:
        runtime.pause()

    transport.subscribe(
        runtime.config.wheel_state.topic,
        "slope_sim.interfaces.v1.WheelState",
        callback,
    )
    try:
        states = runtime.after_physics_step(0.03)
        snapshot = runtime.status_snapshot(wall_time=clock())
        topic = snapshot.topics[runtime.config.wheel_state.topic]

        assert tuple(state.timestamp_ns for state in states) == (10_000_000,)
        assert robot.read_timestamps == [10_000_000]
        assert transport.snapshot().published_count == 1
        assert topic.message_count == 1
        assert topic.latest_timestamp_ns == 10_000_000
        assert topic.error_count == 1
        assert topic.state == "error"
        assert "frequency failure" in topic.detail
        assert topic.dropped_count == 0
    finally:
        runtime.close()


def test_other_thread_pause_returns_during_publish_and_stops_remaining_due_batch():
    clock = FakeMonotonic(0.03)
    robot = FakeRobot(actual_drive=(1.0, 2.0))
    transport = FakeTransport()
    transport.publish_started = Event()
    transport.release_publish = Event()
    runtime = _make_runtime(robot, clock, transport)
    after_states: list[WheelState] = []
    after_errors: list[BaseException] = []
    pause_finished = Event()

    def run_after() -> None:
        try:
            after_states.extend(runtime.after_physics_step(0.03))
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            after_errors.append(exc)

    def run_pause() -> None:
        runtime.pause()
        pause_finished.set()

    after_thread = Thread(target=run_after, daemon=True)
    pause_thread = Thread(target=run_pause, daemon=True)
    try:
        after_thread.start()
        assert transport.publish_started.wait(timeout=2.0)
        pause_thread.start()
        assert pause_finished.wait(timeout=0.2)
        assert after_thread.is_alive()

        transport.release_publish.set()
        after_thread.join(timeout=2.0)
        pause_thread.join(timeout=2.0)
        assert not after_thread.is_alive() and not pause_thread.is_alive()
        assert after_errors == []
        assert tuple(state.timestamp_ns for state in after_states) == (10_000_000,)
        assert robot.read_timestamps == [10_000_000]
        snapshot = runtime.status_snapshot(wall_time=clock())
        topic = snapshot.topics[runtime.config.wheel_state.topic]
        assert topic.message_count == 1
        assert topic.latest_timestamp_ns == 10_000_000
        assert topic.dropped_count == 0
    finally:
        transport.release_publish.set()
        after_thread.join(timeout=2.0)
        pause_thread.join(timeout=2.0)
        runtime.close()


def test_rebind_completes_while_old_publish_is_blocked_and_old_result_cannot_commit():
    clock = FakeMonotonic(0.01)
    old_robot = FakeRobot("df_mid", actual_drive=(1.0, 2.0))
    new_robot = FakeRobot("active_steering_4wd")
    transport = FakeTransport()
    transport.publish_started = Event()
    transport.release_publish = Event()
    runtime = _make_runtime(old_robot, clock, transport)
    after_errors: list[BaseException] = []
    rebind_errors: list[BaseException] = []
    rebind_finished = Event()

    def run_after() -> None:
        try:
            runtime.after_physics_step(0.01)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            after_errors.append(exc)

    def run_rebind() -> None:
        try:
            runtime.rebind_robot(new_robot)
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            rebind_errors.append(exc)
        finally:
            rebind_finished.set()

    after_thread = Thread(target=run_after, daemon=True)
    rebind_thread = Thread(target=run_rebind, daemon=True)
    try:
        after_thread.start()
        assert transport.publish_started.wait(timeout=2.0)
        rebind_thread.start()
        assert rebind_finished.wait(timeout=0.2)

        transport.release_publish.set()
        after_thread.join(timeout=2.0)
        rebind_thread.join(timeout=2.0)
        assert not after_thread.is_alive() and not rebind_thread.is_alive()
        assert after_errors == rebind_errors == []
        assert runtime.robot_model is new_robot.model_spec
        assert runtime.last_wheel_state is None
        snapshot = runtime.status_snapshot(wall_time=clock())
        topic = snapshot.topics[runtime.config.wheel_state.topic]
        assert snapshot.command.state == "waiting_command"
        assert topic.message_count == 0
        assert topic.error_count == 0
        assert topic.latest_timestamp_ns is None
    finally:
        transport.release_publish.set()
        after_thread.join(timeout=2.0)
        rebind_thread.join(timeout=2.0)
        runtime.close()


@pytest.mark.parametrize("epoch", ("waiting", "timed_out"))
def test_safe_stop_samples_active_steering_angle_only_once_per_epoch(epoch: str):
    clock = FakeMonotonic()
    robot = FakeRobot(
        "active_steering_4wd",
        actual_steering=(0.2, -0.2),
    )
    runtime = _make_runtime(robot, clock)
    try:
        if epoch == "timed_out":
            assert runtime.accept_local_command(
                WheelCommand(1, (1.0, 2.0, 3.0, 4.0), (0.5, -0.5)),
                received_at=0.0,
            )
            runtime.before_physics_step(1.0 / 240.0, wall_time=0.0)
            first_wall_time = 0.1
        else:
            first_wall_time = 0.0

        first = runtime.before_physics_step(1.0 / 240.0, wall_time=first_wall_time)
        assert first.waiting or first.timed_out
        assert robot.safe_stop_count == 1
        assert robot.safe_stop_actuals == [(0.2, -0.2)]

        robot.actual_steering = (0.4, -0.4)
        second = runtime.before_physics_step(
            1.0 / 240.0,
            wall_time=first_wall_time + 1.0 / 240.0,
        )
        assert second.waiting or second.timed_out
        assert robot.safe_stop_count == 1
        assert robot.safe_stop_actuals == [(0.2, -0.2)]
        assert robot.command_calls[-1] == (
            (0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0),
            1.0 / 240.0,
        )
    finally:
        runtime.close()


def test_failed_first_safe_stop_is_retried_and_only_success_latches_epoch():
    clock = FakeMonotonic()
    robot = FakeRobot("active_steering_4wd", actual_steering=(0.2, -0.2))
    robot.safe_stop_error = RuntimeError("hold failed")
    runtime = _make_runtime(robot, clock)
    try:
        with pytest.raises(RuntimeError, match="hold failed"):
            runtime.before_physics_step(1.0 / 240.0, wall_time=0.0)
        assert robot.safe_stop_count == 1

        robot.safe_stop_error = None
        runtime.before_physics_step(1.0 / 240.0, wall_time=0.01)
        assert robot.safe_stop_count == 2
        robot.actual_steering = (0.4, -0.4)
        runtime.before_physics_step(1.0 / 240.0, wall_time=0.02)
        assert robot.safe_stop_count == 2
        assert robot.safe_stop_actuals == [
            (0.2, -0.2),
            (0.2, -0.2),
        ]
    finally:
        runtime.close()


def test_active_command_resets_safe_stop_latch_for_next_timeout_epoch():
    clock = FakeMonotonic()
    robot = FakeRobot("active_steering_4wd", actual_steering=(0.1, -0.1))
    runtime = _make_runtime(robot, clock)
    try:
        runtime.before_physics_step(1.0 / 240.0, wall_time=0.0)
        assert robot.safe_stop_actuals == [(0.1, -0.1)]

        assert runtime.accept_local_command(
            WheelCommand(1, (1.0, 2.0, 3.0, 4.0), (0.5, -0.5)),
            received_at=0.0,
        )
        runtime.before_physics_step(1.0 / 240.0, wall_time=0.0)
        robot.actual_steering = (0.3, -0.3)
        runtime.before_physics_step(1.0 / 240.0, wall_time=0.1)

        assert robot.safe_stop_count == 2
        assert robot.safe_stop_actuals[-1] == (0.3, -0.3)
    finally:
        runtime.close()


@pytest.mark.parametrize(
    ("model_name", "actual_drive", "actual_steering"),
    (
        ("df_back", (1.0,), ()),
        ("active_steering_4wd", (1.0, 2.0, 3.0, 4.0), (0.1,)),
    ),
)
def test_invalid_wheel_state_lengths_are_counted_without_publish_or_last_state(
    model_name: str,
    actual_drive: tuple[float, ...],
    actual_steering: tuple[float, ...],
):
    clock = FakeMonotonic(0.01)
    robot = FakeRobot(
        model_name,
        actual_drive=actual_drive,
        actual_steering=actual_steering,
    )
    transport = FakeTransport()
    runtime = _make_runtime(robot, clock, transport)
    try:
        assert runtime.after_physics_step(0.01) == ()
        snapshot = runtime.status_snapshot(wall_time=clock())
        topic = snapshot.topics[runtime.config.wheel_state.topic]
        assert runtime.last_wheel_state is None
        assert transport.published == []
        assert topic.state == "error"
        assert topic.error_count == 1
        assert topic.message_count == 0
        assert topic.latest_timestamp_ns is None
        assert "requires" in topic.detail
    finally:
        runtime.close()


def test_closed_runtime_exposes_zero_waiting_decision_and_disconnected_statuses():
    clock = FakeMonotonic()
    robot = FakeRobot("df_front")
    runtime = _make_runtime(robot, clock)
    try:
        assert runtime.accept_local_command(
            WheelCommand(1, (3.0, 4.0), ()),
            received_at=clock(),
        )
        runtime.before_physics_step(1.0 / 240.0, wall_time=clock())
        runtime.close()

        decision = runtime.last_decision
        snapshot = runtime.status_snapshot(wall_time=clock())
        topic = snapshot.topics[runtime.config.wheel_state.topic]
        assert decision.waiting
        assert not decision.timed_out
        assert decision.drive_wheel_speed_rad_s == (0.0, 0.0)
        assert decision.steering_wheel_speed_rad_s == ()
        assert snapshot.command.state == "disconnected"
        assert topic.state == "disconnected"
    finally:
        runtime.close()


def test_local_command_snapshot_uses_acceptance_sim_time_not_external_timestamp() -> None:
    clock = FakeMonotonic()
    runtime = _make_runtime(FakeRobot(), clock)
    try:
        runtime.after_physics_step(0.01)
        command = WheelCommand(999, (1.0, 2.0), ())

        assert runtime.accept_local_command(command, received_at=clock())
        snapshot = runtime.dashboard_snapshot(wall_time=clock())

        assert snapshot.wheel_command is command
        assert snapshot.wheel_command_received_sim_time_ns == 10_000_000
        assert snapshot.wheel_command_received_sim_time_ns != command.timestamp_ns
        assert snapshot.sim_time_ns == 10_000_000
        assert snapshot.status.wheel_state is snapshot.wheel_state
    finally:
        runtime.close()


def test_transport_callback_records_current_sim_time_and_stale_token_cannot_advance_it() -> None:
    clock = FakeMonotonic()
    transport = FakeTransport()
    runtime = _make_runtime(FakeRobot(), clock, transport)
    old_subscription = transport.subscriptions[-1]
    try:
        runtime.after_physics_step(0.02)
        command = WheelCommand(123, (3.0, 4.0), ())
        payload = runtime._codec.encode(command)

        assert old_subscription.callback(payload, clock()) is True
        accepted = runtime.dashboard_snapshot(wall_time=clock())
        assert accepted.wheel_command is command or accepted.wheel_command == command
        assert accepted.wheel_command_received_sim_time_ns == 20_000_000

        runtime.rebind_robot(FakeRobot())
        assert old_subscription.callback(payload, clock()) is None
        rebound = runtime.dashboard_snapshot(wall_time=clock())
        assert rebound.wheel_command is None
        assert rebound.wheel_command_received_sim_time_ns is None
    finally:
        runtime.close()


def test_transport_callback_freezes_sim_time_before_blocked_decode() -> None:
    clock = FakeMonotonic()
    transport = FakeTransport()
    runtime = _make_runtime(FakeRobot(), clock, transport)
    command = WheelCommand(123, (3.0, 4.0), ())
    payload = runtime._codec.encode(command)
    codec = BlockingDecodeCodec(runtime._codec)
    runtime._codec = codec
    callback_results: list[object] = []

    def run_callback() -> None:
        callback_results.append(transport.subscriptions[-1].callback(payload, clock()))

    worker = Thread(target=run_callback, daemon=True)
    try:
        worker.start()
        assert codec.entered.wait(timeout=2.0)
        runtime.after_physics_step(0.01)
        codec.release.set()
        worker.join(timeout=2.0)

        assert not worker.is_alive()
        assert callback_results == [True]
        snapshot = runtime.dashboard_snapshot(wall_time=clock())
        assert snapshot.sim_time_ns == 10_000_000
        assert snapshot.wheel_command_received_sim_time_ns == 0
    finally:
        codec.release.set()
        worker.join(timeout=2.0)
        runtime.close()


def test_invalid_local_command_does_not_advance_dashboard_command_or_acceptance_time() -> None:
    clock = FakeMonotonic()
    runtime = _make_runtime(FakeRobot(), clock)
    valid = WheelCommand(1, (1.0, 2.0), ())
    try:
        assert runtime.accept_local_command(valid, received_at=clock())
        runtime.after_physics_step(0.01)

        assert not runtime.accept_local_command(
            WheelCommand(2, (9.0,), ()),
            received_at=clock(),
        )
        snapshot = runtime.dashboard_snapshot(wall_time=clock())
        assert snapshot.wheel_command is valid
        assert snapshot.wheel_command_received_sim_time_ns == 0
    finally:
        runtime.close()


def test_local_command_subclass_is_rejected_without_polluting_dashboard_latest() -> None:
    clock = FakeMonotonic()
    runtime = _make_runtime(FakeRobot(), clock)
    valid = WheelCommand(1, (1.0, 2.0), ())
    subclass = MutableWheelCommand(2, (3.0, 4.0), ())
    object.__setattr__(subclass, "extra_state", [])
    try:
        assert runtime.accept_local_command(valid, received_at=clock())
        before = runtime.dashboard_snapshot(wall_time=clock())

        assert runtime.accept_local_command(subclass, received_at=clock()) is False

        after = runtime.dashboard_snapshot(wall_time=clock())
        assert after.status.command.valid_count == before.status.command.valid_count
        assert after.wheel_command is valid
        assert after.wheel_command_received_sim_time_ns == before.wheel_command_received_sim_time_ns
    finally:
        runtime.close()


def test_callback_command_subclass_is_rejected_without_polluting_dashboard_latest() -> None:
    clock = FakeMonotonic()
    transport = FakeTransport()
    runtime = _make_runtime(FakeRobot(), clock, transport)
    valid = WheelCommand(1, (1.0, 2.0), ())
    subclass = MutableWheelCommand(2, (3.0, 4.0), ())
    object.__setattr__(subclass, "extra_state", {})
    try:
        assert runtime.accept_local_command(valid, received_at=clock())
        before = runtime.dashboard_snapshot(wall_time=clock())
        runtime._codec = SubclassDecodeCodec(runtime._codec, subclass)

        assert transport.subscriptions[-1].callback(b"ignored", clock()) is False

        after = runtime.dashboard_snapshot(wall_time=clock())
        assert after.status.command.valid_count == before.status.command.valid_count
        assert after.wheel_command is valid
        assert after.wheel_command_received_sim_time_ns == before.wheel_command_received_sim_time_ns
    finally:
        runtime.close()


def test_dashboard_payloads_update_only_after_successful_current_publish_per_topic() -> None:
    clock = FakeMonotonic()
    transport = FakeTransport()
    runtime = _make_runtime(FakeRobot(actual_drive=(1.0, 2.0)), clock, transport)
    _attach_dashboard_sensors(runtime)
    front_topic = runtime.config.lidar_front.topic
    try:
        runtime.after_physics_step(0.1)
        first = runtime.dashboard_snapshot(wall_time=clock())
        assert first.wheel_state is not None
        assert first.lidar_front is not None and first.lidar_front_view is not None
        assert first.lidar_rear is not None and first.lidar_rear_view is not None
        assert first.rtk is not None and first.imu is not None
        assert first.lidar_front.timebase_ns == first.lidar_front_view.timestamp_ns == 100_000_000
        assert first.lidar_rear.timebase_ns == first.lidar_rear_view.timestamp_ns == 50_000_000

        transport.topic_publish_outcomes[front_topic] = deque((False,))
        runtime.after_physics_step(0.1)
        second = runtime.dashboard_snapshot(wall_time=clock())

        assert second.lidar_front is first.lidar_front
        assert second.lidar_front_view is first.lidar_front_view
        assert second.lidar_rear is not first.lidar_rear
        assert second.lidar_rear.timebase_ns == 150_000_000
        assert second.lidar_rear_view.timestamp_ns == 150_000_000
        assert second.rtk.timestamp_ns == 200_000_000
        assert second.imu.timestamp_ns == 200_000_000
        assert second.wheel_state.timestamp_ns == 200_000_000
        # 旧快照继续持有构造时的冻结 payload，不受后续发布影响。
        assert first.sim_time_ns == 100_000_000
        assert first.lidar_rear.timebase_ns == 50_000_000
    finally:
        runtime.close()


def test_headless_runtime_publishes_message_only_lidar_without_dashboard_payloads() -> None:
    """关闭俯视捕获后仍发布双点云，但不生成或保存 Dashboard 点云对。"""
    clock = FakeMonotonic()
    transport = FakeTransport()
    runtime = _runtime_type()(
        FakeRobot(),
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=clock,
        capture_lidar_top_view=False,
    )
    front = MessageOnlyLidar("lidar_front", 1)
    rear = MessageOnlyLidar("lidar_rear", 2)
    runtime._front_lidar = front
    runtime._rear_lidar = rear
    runtime._truth_sensor_suite = DashboardTruthSensors()
    try:
        runtime.after_physics_step(0.1)
        snapshot = runtime.dashboard_snapshot(wall_time=clock())

        assert front.scan_timestamps == [100_000_000]
        assert rear.scan_timestamps == [50_000_000]
        assert front.top_view_timestamps == rear.top_view_timestamps == []
        assert snapshot.lidar_front is snapshot.lidar_front_view is None
        assert snapshot.lidar_rear is snapshot.lidar_rear_view is None
        published_topics = tuple(item[0] for item in transport.published)
        assert runtime.config.lidar_front.topic in published_topics
        assert runtime.config.lidar_rear.topic in published_topics
        assert snapshot.status.topics[runtime.config.lidar_front.topic].message_count == 1
        assert snapshot.status.topics[runtime.config.lidar_rear.topic].message_count == 1
    finally:
        runtime.close()


def test_runtime_prefers_atomic_lidar_scan_over_legacy_incremental_entrypoint() -> None:
    """每个 deadline 在同一物理时刻完成整批射线，不能拼接跨帧世界状态。"""
    transport = FakeTransport()
    runtime = _runtime_type()(
        FakeRobot(),
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        capture_lidar_top_view=False,
    )
    front = AtomicPreferredLidar("lidar_front", 1)
    rear = AtomicPreferredLidar("lidar_rear", 2)
    runtime._front_lidar = front
    runtime._rear_lidar = rear
    runtime._truth_sensor_suite = DashboardTruthSensors()
    try:
        runtime.after_physics_step(0.1)

        assert front.scan_timestamps == [100_000_000]
        assert rear.scan_timestamps == [50_000_000]
        assert {
            topic
            for topic, _payload, _type_name, _timestamp_ns, _wall_time in transport.published
        } >= {runtime.config.lidar_front.topic, runtime.config.lidar_rear.topic}
    finally:
        runtime.close()


def test_runtime_phase_shifts_lidars_while_each_remains_exactly_ten_hz() -> None:
    """前后雷达错开半周期，长期仍分别服从统一仿真时钟。"""
    runtime = _runtime_type()(
        FakeRobot(),
        config=InterfaceConfig.default(transport_mode="local"),
        transport=FakeTransport(),
        capture_lidar_top_view=False,
    )
    front = MessageOnlyLidar("lidar_front", 1)
    rear = MessageOnlyLidar("lidar_rear", 2)
    runtime._front_lidar = front
    runtime._rear_lidar = rear
    runtime._truth_sensor_suite = DashboardTruthSensors()
    try:
        for _ in range(2_400):
            runtime.after_physics_step(1.0 / 240.0)

        assert front.scan_timestamps == [
            index * 100_000_000
            for index in range(1, 101)
        ]
        assert rear.scan_timestamps == [
            50_000_000 + index * 100_000_000
            for index in range(100)
        ]
    finally:
        runtime.close()


def test_lidar_phase_is_stable_across_pause_and_world_rebuild() -> None:
    """暂停和世界重建不推进仿真时钟，也不重置双雷达相位。"""
    clock = FakeMonotonic()
    runtime = _runtime_type()(
        FakeRobot(),
        config=InterfaceConfig.default(transport_mode="local"),
        transport=FakeTransport(),
        monotonic=clock,
        capture_lidar_top_view=False,
    )
    first_front = MessageOnlyLidar("lidar_front", 1)
    first_rear = MessageOnlyLidar("lidar_rear", 2)
    runtime._front_lidar = first_front
    runtime._rear_lidar = first_rear
    runtime._truth_sensor_suite = DashboardTruthSensors()
    try:
        runtime.after_physics_step(0.05)
        assert first_front.scan_timestamps == []
        assert first_rear.scan_timestamps == [50_000_000]

        runtime.pause()
        assert runtime.after_physics_step(1.0) == ()
        runtime.resume(wall_time=clock())
        runtime.after_physics_step(0.05)
        assert first_front.scan_timestamps == [100_000_000]
        assert first_rear.scan_timestamps == [50_000_000]

        runtime.prepare_world_rebuild()
        runtime.commit_world_rebuild(
            FakeRobot(),
            DashboardBackend(),
            _dashboard_scene_document(),
        )
        rebuilt_front = MessageOnlyLidar("lidar_front", 1)
        rebuilt_rear = MessageOnlyLidar("lidar_rear", 2)
        runtime._front_lidar = rebuilt_front
        runtime._rear_lidar = rebuilt_rear
        runtime._truth_sensor_suite = DashboardTruthSensors()

        runtime.after_physics_step(0.05)
        assert rebuilt_front.scan_timestamps == []
        assert rebuilt_rear.scan_timestamps == [150_000_000]

        runtime.after_physics_step(0.05)
        assert rebuilt_front.scan_timestamps == [200_000_000]
        assert rebuilt_rear.scan_timestamps == [150_000_000]
    finally:
        runtime.close()


def test_legacy_only_lidar_is_rejected_without_calling_scan_or_publishing() -> None:
    clock = FakeMonotonic()
    transport = FakeTransport()
    runtime = _make_runtime(FakeRobot(), clock, transport)
    legacy = LegacyOnlyLidar()
    runtime._front_lidar = legacy
    runtime._rear_lidar = DashboardLidar("lidar_rear", 2)
    runtime._truth_sensor_suite = DashboardTruthSensors()
    front_topic = runtime.config.lidar_front.topic
    try:
        runtime.after_physics_step(0.1)
        snapshot = runtime.dashboard_snapshot(wall_time=clock())

        assert legacy.scan_calls == 0
        assert snapshot.lidar_front is None
        assert snapshot.lidar_front_view is None
        assert snapshot.status.topics[front_topic].state == "error"
        assert front_topic not in tuple(item[0] for item in transport.published)
    finally:
        runtime.close()


@pytest.mark.parametrize(
    "operation",
    ("prepare", "commit", "abort", "fault", "rebind", "disconnect", "close"),
)
def test_generation_invalidating_paths_clear_every_dashboard_payload(operation: str) -> None:
    clock = FakeMonotonic()
    runtime = _make_runtime(FakeRobot(), clock)
    _attach_dashboard_sensors(runtime)
    closed = False
    try:
        runtime.after_physics_step(0.1)
        assert runtime.accept_local_command(
            WheelCommand(999, (1.0, 2.0), ()),
            received_at=clock(),
        )
        populated = runtime.dashboard_snapshot(wall_time=clock())
        assert all(
            value is not None
            for value in (
                populated.wheel_command,
                populated.wheel_command_received_sim_time_ns,
                populated.wheel_state,
                populated.lidar_front,
                populated.lidar_rear,
                populated.rtk,
                populated.imu,
                populated.lidar_front_view,
                populated.lidar_rear_view,
            )
        )

        if operation == "prepare":
            runtime.prepare_world_rebuild()
        elif operation == "commit":
            runtime.prepare_world_rebuild()
            runtime.commit_world_rebuild(
                FakeRobot(),
                DashboardBackend(),
                _dashboard_scene_document(),
            )
        elif operation == "abort":
            runtime.prepare_world_rebuild()
            runtime.abort_world_rebuild()
        elif operation == "fault":
            runtime.prepare_world_rebuild()
            runtime.fault_world_rebuild()
        elif operation == "rebind":
            runtime.rebind_robot(FakeRobot())
        elif operation == "disconnect":
            runtime.initialize_peer_lifecycle("ecal", "active")
            runtime.handle_peer_state("disconnected")
        else:
            runtime.close()
            closed = True

        cleared = runtime.dashboard_snapshot(wall_time=clock())
        assert cleared.generation > populated.generation
        assert (
            cleared.wheel_command,
            cleared.wheel_command_received_sim_time_ns,
            cleared.wheel_state,
            cleared.lidar_front,
            cleared.lidar_rear,
            cleared.rtk,
            cleared.imu,
            cleared.lidar_front_view,
            cleared.lidar_rear_view,
        ) == (None,) * 9
    finally:
        if not closed:
            runtime.close()


def test_pause_resume_preserves_generation_and_dashboard_payload_identity() -> None:
    clock = FakeMonotonic()
    runtime = _make_runtime(FakeRobot(), clock)
    _attach_dashboard_sensors(runtime)
    try:
        runtime.after_physics_step(0.1)
        assert runtime.accept_local_command(
            WheelCommand(999, (1.0, 2.0), ()),
            received_at=clock(),
        )
        before = runtime.dashboard_snapshot(wall_time=clock())

        runtime.pause()
        paused = runtime.dashboard_snapshot(wall_time=clock())
        runtime.resume(wall_time=clock())
        resumed = runtime.dashboard_snapshot(wall_time=clock())

        assert paused.generation == resumed.generation == before.generation
        for field_name in (
            "wheel_command",
            "wheel_command_received_sim_time_ns",
            "wheel_state",
            "lidar_front",
            "lidar_rear",
            "rtk",
            "imu",
            "lidar_front_view",
            "lidar_rear_view",
        ):
            assert getattr(paused, field_name) is getattr(before, field_name)
            assert getattr(resumed, field_name) is getattr(before, field_name)
    finally:
        runtime.close()


def test_candidate_rebind_parking_failure_preserves_generation_and_all_latest() -> None:
    clock = FakeMonotonic()
    robot = FakeRobot()
    runtime = _make_runtime(robot, clock)
    _attach_dashboard_sensors(runtime)
    try:
        runtime.after_physics_step(0.1)
        assert runtime.accept_local_command(
            WheelCommand(999, (1.0, 2.0), ()),
            received_at=clock(),
        )
        before = runtime.dashboard_snapshot(wall_time=clock())
        robot.safe_stop_error = RuntimeError("parking failed")

        with pytest.raises(RuntimeError, match="parking failed"):
            runtime.rebind_robot(FakeRobot())

        after = runtime.dashboard_snapshot(wall_time=clock())
        assert after.generation == before.generation
        for field_name in (
            "wheel_command",
            "wheel_command_received_sim_time_ns",
            "wheel_state",
            "lidar_front",
            "lidar_rear",
            "rtk",
            "imu",
            "lidar_front_view",
            "lidar_rear_view",
        ):
            assert getattr(after, field_name) is getattr(before, field_name)
    finally:
        robot.safe_stop_error = None
        runtime.close()


def test_dashboard_snapshot_constructs_after_releasing_lifecycle_lock(monkeypatch) -> None:
    clock = FakeMonotonic()
    runtime = _make_runtime(FakeRobot(), clock)
    real_snapshot_type = runtime_module.InterfaceDashboardSnapshot
    constructor_entered = Event()
    release_constructor = Event()
    snapshot_done = Event()
    pause_done = Event()
    errors: list[BaseException] = []

    def blocking_snapshot(**fields):
        constructor_entered.set()
        assert release_constructor.wait(timeout=3.0)
        return real_snapshot_type(**fields)

    def take_snapshot() -> None:
        try:
            runtime.dashboard_snapshot(wall_time=clock())
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            errors.append(exc)
        finally:
            snapshot_done.set()

    def pause_runtime() -> None:
        try:
            runtime.pause()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            errors.append(exc)
        finally:
            pause_done.set()

    monkeypatch.setattr(runtime_module, "InterfaceDashboardSnapshot", blocking_snapshot)
    snapshot_thread = Thread(target=take_snapshot, daemon=True)
    pause_thread = Thread(target=pause_runtime, daemon=True)
    try:
        snapshot_thread.start()
        assert constructor_entered.wait(timeout=2.0)
        pause_thread.start()

        assert pause_done.wait(timeout=0.2)
        assert not snapshot_done.is_set()
    finally:
        release_constructor.set()
        snapshot_thread.join(timeout=2.0)
        pause_thread.join(timeout=2.0)
        runtime.close()
    assert errors == []
