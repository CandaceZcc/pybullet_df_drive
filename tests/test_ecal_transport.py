# eCAL 传输适配器测试：用可控 binding 锁定导入、资源、队列和重连语义。
from __future__ import annotations

from dataclasses import FrozenInstanceError, dataclass, field, replace
import json
from threading import Event, Thread, get_ident
import time
from types import SimpleNamespace

import pytest

from scripts import ecal_roundtrip_peer as peer_script
from scripts import verify_ecal_roundtrip as verifier
import slope_sim.interfaces.ecal_transport as ecal_transport_module
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.config import ChannelConfig, InterfaceConfig
from slope_sim.interfaces.logging import InterfaceEventLogger
from slope_sim.interfaces.transport import LocalTransport, TransportTopicQuality
from slope_sim.interfaces.ecal_transport import (
    EcalBindings,
    EcalTransport,
    EcalUnavailableError,
    create_transport,
    load_ecal_bindings,
)
from slope_sim.interfaces.generated import slope_sim_interfaces_pb2 as pb
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.interfaces.runtime import InterfaceRuntime
from slope_sim.interfaces.wheel import WheelCommandMailbox
from slope_sim.model_registry import get_robot_model
from slope_sim.simulation import _PeerStateRelay


TEST_CONFIG = InterfaceConfig.default(transport_mode="ecal")
TEST_CODEC = ProtoCodec()
WHEEL_COMMAND_TOPIC = TEST_CONFIG.wheel_command.topic
WHEEL_STATE_TOPIC = TEST_CONFIG.wheel_state.topic
WHEEL_COMMAND_TYPE = TEST_CODEC.type_name(WheelCommand(0, (), ()))
WHEEL_STATE_TYPE = TEST_CODEC.type_name(WheelState(0, (), ()))


@dataclass
class FakeResource:
    """记录 fake eCAL 资源是否仍打开以及关闭顺序。"""

    kind: str
    topic: str
    close_log: list[str]
    connected: bool = False
    callback: object | None = None
    closed: bool = False
    close_count: int = 0
    wait_for_callback_on_close: bool = False
    callback_running: Event = field(default_factory=Event, repr=False)
    callback_returned: Event = field(default_factory=Event, repr=False)
    close_started: Event = field(default_factory=Event, repr=False)

    def close(self) -> None:
        self.close_started.set()
        if self.wait_for_callback_on_close and self.callback_running.is_set():
            self.callback_returned.wait()
        if self.closed:
            return
        self.closed = True
        self.close_count += 1
        self.close_log.append(f"{self.kind}:{self.topic}")

    def emit(self, payload: object) -> object:
        assert self.callback is not None
        self.callback_returned.clear()
        self.callback_running.set()
        try:
            return self.callback(payload)
        finally:
            self.callback_running.clear()
            self.callback_returned.set()


class FakeEcalBindings:
    """实现适配器依赖的最小 binding 端口，并支持精确故障注入。"""

    api = "fake"

    def __init__(self) -> None:
        self.close_log: list[str] = []
        self.participants: list[FakeResource] = []
        self.subscribers: list[FakeResource] = []
        self.publishers: list[FakeResource] = []
        self.sent: list[tuple[str, bytes]] = []
        self.fail_on_publisher_number: int | None = None

    def create_participant(self, name: str) -> FakeResource:
        resource = FakeResource("participant", name, self.close_log)
        self.participants.append(resource)
        return resource

    def create_subscriber(
        self,
        topic: str,
        _message_type: type,
        callback,
    ) -> FakeResource:
        resource = FakeResource("subscriber", topic, self.close_log, callback=callback)
        self.subscribers.append(resource)
        return resource

    def create_publisher(self, topic: str, _message_type: type) -> FakeResource:
        number = len(self.publishers) + 1
        if number == self.fail_on_publisher_number:
            raise RuntimeError(f"publisher {number} failed")
        resource = FakeResource("publisher", topic, self.close_log)
        self.publishers.append(resource)
        return resource

    def send(self, publisher: FakeResource, payload: bytes, _message_type: type) -> None:
        self.sent.append((publisher.topic, bytes(payload)))

    @staticmethod
    def is_peer_connected(resource: FakeResource) -> bool:
        return resource.connected

    @staticmethod
    def close(resource: FakeResource) -> None:
        resource.close()

    @property
    def open_participants(self) -> int:
        return sum(not resource.closed for resource in self.participants)

    @property
    def open_subscribers(self) -> int:
        return sum(not resource.closed for resource in self.subscribers)

    @property
    def open_publishers(self) -> int:
        return sum(not resource.closed for resource in self.publishers)


class _V61Publisher:
    """模拟官方 v6 Publisher；刻意不提供 close/destroy。"""

    instances: list["_V61Publisher"] = []
    fail_on_number: int | None = None

    def __init__(self, message_type: type, topic: str) -> None:
        number = len(type(self).instances) + 1
        if number == type(self).fail_on_number:
            raise RuntimeError(f"publisher {number} failed")
        self.constructor_args = (message_type, topic)
        self.subscriber_count: object = 0
        self.sent: list[object] = []
        type(self).instances.append(self)

    def send(self, message: object) -> bool:
        self.sent.append(message)
        return True

    def get_subscriber_count(self) -> object:
        return self.subscriber_count


class _V61Subscriber:
    """模拟官方 v6 Subscriber 回调与 discovery API。"""

    instances: list["_V61Subscriber"] = []

    def __init__(self, message_type: type, topic: str) -> None:
        self.constructor_args = (message_type, topic)
        self.publisher_count: object = 0
        self.receive_callback = None
        self.remove_count = 0
        type(self).instances.append(self)

    def set_receive_callback(self, callback) -> None:
        self.receive_callback = callback

    def remove_receive_callback(self) -> None:
        self.remove_count += 1
        self.receive_callback = None

    def get_publisher_count(self) -> object:
        return self.publisher_count

    def emit(self, message: object) -> None:
        assert self.receive_callback is not None
        self.receive_callback(
            SimpleNamespace(topic_name=self.constructor_args[1]),
            SimpleNamespace(message=message, send_timestamp=0, send_clock=0),
        )


class _V61Core:
    """记录官方进程级 initialize/finalize 调用。"""

    def __init__(self) -> None:
        self.initialize_calls: list[str] = []
        self.finalize_calls = 0

    def initialize(self, process_name: str) -> bool:
        self.initialize_calls.append(process_name)
        return True

    def finalize(self) -> bool:
        self.finalize_calls += 1
        return True


def _fake_v61_importer():
    """返回可记录官方三个模块导入顺序的 importer 与 core。"""
    _V61Publisher.instances = []
    _V61Publisher.fail_on_number = None
    _V61Subscriber.instances = []
    core = _V61Core()
    modules = {
        "ecal.nanobind_core": core,
        "ecal.msg.proto.core": SimpleNamespace(
            Publisher=_V61Publisher,
            Subscriber=_V61Subscriber,
        ),
        "ecal.msg.common.core": SimpleNamespace(ReceiveCallbackData=object),
    }
    imported: list[str] = []

    def importer(name: str):
        imported.append(name)
        return modules[name]

    return importer, imported, core


def _config(mode: str = "ecal") -> InterfaceConfig:
    return InterfaceConfig.default(transport_mode=mode)


def _command_subscriber(bindings: FakeEcalBindings) -> FakeResource:
    return next(
        resource
        for resource in bindings.subscribers
        if resource.topic == WHEEL_COMMAND_TOPIC
    )


def _expected_types(
    config: InterfaceConfig = TEST_CONFIG,
    codec: ProtoCodec = TEST_CODEC,
) -> dict[str, str]:
    """从集中配置和真实 codec 派生六话题类型，不复制生产常量。"""
    return {
        config.wheel_command.topic: codec.type_name(WheelCommand(0, (), ())),
        config.wheel_state.topic: codec.type_name(WheelState(0, (), ())),
        config.lidar_front.topic: codec.type_name(
            LidarPointCloud(0, "lidar_front", 0, 1, ())
        ),
        config.lidar_rear.topic: codec.type_name(
            LidarPointCloud(0, "lidar_rear", 0, 2, ())
        ),
        config.rtk.topic: codec.type_name(RtkState(0, 0.0, 0.0, 0.0, 0.0)),
        config.imu.topic: codec.type_name(ImuAttitude(0, 0.0, 0.0)),
    }


def _valid_command_payload(timestamp_ns: int = 10) -> bytes:
    return TEST_CODEC.encode(WheelCommand(timestamp_ns, (4.0, 4.0), ()))


class RuntimeRobot:
    """为 transport 关闭期集成测试提供最小轮式机器人端口。"""

    model_spec = get_robot_model("df_mid")

    def command_wheel_speeds(self, drive, steering=(), dt=1.0 / 240.0):
        return tuple(drive)

    def hold_current_steering_and_stop_drive(self, _dt: float) -> None:
        return None

    def read_interface_wheel_state(self, timestamp_ns: int) -> WheelState:
        return WheelState(timestamp_ns, (0.0, 0.0), ())


def _custom_rate_config() -> InterfaceConfig:
    """构造非默认频率，暴露任何脚本内重复编码的周期。"""
    rates = {
        TEST_CONFIG.wheel_command.topic: 50,
        TEST_CONFIG.wheel_state.topic: 80,
        TEST_CONFIG.lidar_front.topic: 5,
        TEST_CONFIG.lidar_rear.topic: 7,
        TEST_CONFIG.rtk.topic: 11,
        TEST_CONFIG.imu.topic: 13,
    }
    channels = {
        channel.topic: ChannelConfig(
            channel.topic,
            rates[channel.topic],
            channel.direction,
        )
        for channel in TEST_CONFIG.channels
    }
    return replace(
        TEST_CONFIG,
        wheel_command=channels[TEST_CONFIG.wheel_command.topic],
        wheel_state=channels[TEST_CONFIG.wheel_state.topic],
        lidar_front=channels[TEST_CONFIG.lidar_front.topic],
        lidar_rear=channels[TEST_CONFIG.lidar_rear.topic],
        rtk=channels[TEST_CONFIG.rtk.topic],
        imu=channels[TEST_CONFIG.imu.topic],
    )


class _FakeScheduleClock:
    """让频率调度测试不依赖真实等待。"""

    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, duration_sec: float) -> None:
        assert duration_sec > 0.0
        self.now += duration_sec


class _RecordingTransport:
    """记录调度器提交给 transport 的话题、时间戳和墙钟。"""

    def __init__(self) -> None:
        self.published: list[tuple[str, bytes, str, int, float]] = []

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float,
    ) -> bool:
        self.published.append(
            (topic, bytes(payload), type_name, sim_time_ns, wall_time)
        )
        return True


def _short_burst_roundtrip_result() -> verifier.RoundtripResult:
    """构造首末频率正确、但只有两条且覆盖窗口极短的假阳性。"""
    expected_types = _expected_types()
    rates = {channel.topic: float(channel.rate_hz) for channel in TEST_CONFIG.channels}
    wall_hz = {
        topic: rates[topic] for topic in expected_types
    }
    timestamp_hz = dict(wall_hz)
    result = verifier.RoundtripResult(
        transport_name="ecal",
        peer_returncode=0,
        wall_clock_hz=wall_hz,
        message_timestamp_hz=timestamp_hz,
        received_topics=set(expected_types) - {TEST_CONFIG.wheel_command.topic},
        topic_types=expected_types,
        message_counts={topic: 2 for topic in expected_types},
        dropped_count=0,
    )
    object.__setattr__(result, "duration_sec", 2.5)
    object.__setattr__(
        result,
        "event_span_sec",
        {topic: 1.0 / rates[topic] for topic in expected_types},
    )
    object.__setattr__(result, "peer_dropped_count", 0)
    object.__setattr__(result, "transport_error_count", 0)
    object.__setattr__(result, "peer_error_count", 0)
    return result


def test_strict_ecal_mode_raises_when_bindings_are_missing(monkeypatch):
    def missing(_name: str):
        raise ImportError("missing")

    monkeypatch.setattr("slope_sim.interfaces.ecal_transport.import_module", missing)

    with pytest.raises(EcalUnavailableError, match="eCAL"):
        create_transport("ecal")


def test_auto_mode_falls_back_to_local_with_exact_disconnected_detail(monkeypatch):
    reason = "modern import failed; legacy import failed"

    def unavailable():
        raise EcalUnavailableError(reason)

    monkeypatch.setattr(
        "slope_sim.interfaces.ecal_transport.load_ecal_bindings",
        unavailable,
    )

    transport = create_transport("auto")
    snapshot = transport.snapshot()

    assert isinstance(transport, LocalTransport)
    assert snapshot.mode == "local"
    assert snapshot.ecal_connected is False
    assert snapshot.detail == f"EcalUnavailableError: {reason}"


def test_auto_fallback_uses_nonempty_stable_detail_for_empty_error(monkeypatch):
    def unavailable():
        raise EcalUnavailableError("")

    monkeypatch.setattr(
        "slope_sim.interfaces.ecal_transport.load_ecal_bindings",
        unavailable,
    )

    transport = create_transport("auto")
    try:
        assert transport.snapshot().detail == "EcalUnavailableError"
    finally:
        transport.close()


def test_auto_mode_does_not_hide_real_ecal_initialization_errors():
    bindings = FakeEcalBindings()
    bindings.fail_on_publisher_number = 1

    with pytest.raises(RuntimeError, match="publisher 1"):
        create_transport("auto", bindings=bindings)


def test_load_bindings_uses_only_official_v61_module_paths(monkeypatch):
    importer, imported, _core = _fake_v61_importer()
    monkeypatch.setattr("slope_sim.interfaces.ecal_transport.import_module", importer)

    bindings = load_ecal_bindings()

    assert isinstance(bindings, EcalBindings)
    assert bindings.api == "v6.1"
    assert imported == [
        "ecal.nanobind_core",
        "ecal.msg.proto.core",
        "ecal.msg.common.core",
    ]


def test_load_bindings_never_swallows_non_import_errors(monkeypatch):
    imported: list[str] = []

    def broken_modern(name: str):
        imported.append(name)
        raise RuntimeError("modern initialization exploded")

    monkeypatch.setattr("slope_sim.interfaces.ecal_transport.import_module", broken_modern)

    with pytest.raises(RuntimeError, match="initialization exploded"):
        load_ecal_bindings()
    assert imported == ["ecal.nanobind_core"]


def test_v61_binding_uses_official_constructor_callback_and_cleanup_contract(monkeypatch):
    importer, _imported, core = _fake_v61_importer()
    monkeypatch.setattr("slope_sim.interfaces.ecal_transport.import_module", importer)
    bindings = load_ecal_bindings()
    received: list[bytes] = []

    participant = bindings.create_participant("official-v61")
    publisher = bindings.create_publisher("/out", pb.WheelState)
    subscriber = bindings.create_subscriber("/in", pb.WheelCommand, received.append)
    raw_publisher = publisher.raw
    raw_subscriber = subscriber.raw

    assert raw_publisher.constructor_args == (pb.WheelState, "/out")
    assert raw_subscriber.constructor_args == (pb.WheelCommand, "/in")
    message = pb.WheelCommand(timestamp_ns=7, drive_wheel_speed_rad_s=(1.0, 2.0))
    raw_subscriber.emit(message)
    bindings.send(
        publisher,
        pb.WheelState(timestamp_ns=8).SerializeToString(deterministic=True),
        pb.WheelState,
    )

    assert received == [message.SerializeToString(deterministic=True)]
    assert len(raw_publisher.sent) == 1
    bindings.close(subscriber)
    bindings.close(publisher)
    assert raw_subscriber.remove_count == 1
    assert subscriber.raw is None
    assert publisher.raw is None
    bindings.close(participant)
    assert core.initialize_calls == ["official-v61"]
    assert core.finalize_calls == 1


@pytest.mark.parametrize("count", (True, -1, 1.5, "1", None))
@pytest.mark.parametrize("direction", ("publisher", "subscriber"))
def test_v61_peer_count_rejects_bool_noninteger_and_negative(count, direction):
    raw = (
        SimpleNamespace(get_subscriber_count=lambda: count)
        if direction == "publisher"
        else SimpleNamespace(get_publisher_count=lambda: count)
    )
    resource = SimpleNamespace(raw=raw, direction=direction)

    with pytest.raises(RuntimeError, match="nonnegative integer"):
        ecal_transport_module._resource_peer_connected(resource)


def test_transport_centrally_creates_one_subscriber_and_five_publishers():
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=False)
    try:
        assert [resource.topic for resource in bindings.subscribers] == [
            TEST_CONFIG.wheel_command.topic
        ]
        assert [resource.topic for resource in bindings.publishers] == [
            TEST_CONFIG.wheel_state.topic,
            TEST_CONFIG.lidar_front.topic,
            TEST_CONFIG.lidar_rear.topic,
            TEST_CONFIG.rtk.topic,
            TEST_CONFIG.imu.topic,
        ]
    finally:
        transport.close()


def test_transport_topic_quality_peer_state_is_optional_and_strictly_boolean():
    assert TransportTopicQuality("/local").peer_connected is None
    assert TransportTopicQuality("/connected", peer_connected=True).peer_connected is True
    with pytest.raises(ValueError, match="peer_connected"):
        TransportTopicQuality("/invalid", peer_connected=1)


def test_simulation_role_reports_all_six_topic_peers_independently():
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=False)
    try:
        _command_subscriber(bindings).connected = True
        next(
            item for item in bindings.publishers if item.topic == WHEEL_STATE_TOPIC
        ).connected = True
        next(
            item
            for item in bindings.publishers
            if item.topic == TEST_CONFIG.lidar_front.topic
        ).connected = False

        transport.poll_peer_state()
        quality = {item.topic: item for item in transport.snapshot().topic_quality}

        assert set(quality) == set(_expected_types())
        assert quality[WHEEL_COMMAND_TOPIC].peer_connected is True
        assert quality[WHEEL_STATE_TOPIC].peer_connected is True
        assert quality[TEST_CONFIG.lidar_front.topic].peer_connected is False
        assert transport.snapshot().ecal_connected is True
    finally:
        transport.close()


def test_peer_role_uses_command_publisher_and_five_output_subscriber_counts():
    bindings = FakeEcalBindings()
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        start_worker=False,
        role="peer",
    )
    try:
        next(
            item for item in bindings.publishers if item.topic == WHEEL_COMMAND_TOPIC
        ).connected = True
        next(
            item for item in bindings.subscribers if item.topic == WHEEL_STATE_TOPIC
        ).connected = True
        next(
            item
            for item in bindings.subscribers
            if item.topic == TEST_CONFIG.lidar_front.topic
        ).connected = False

        transport.poll_peer_state()
        quality = {item.topic: item for item in transport.snapshot().topic_quality}

        assert quality[WHEEL_COMMAND_TOPIC].peer_connected is True
        assert quality[WHEEL_STATE_TOPIC].peer_connected is True
        assert quality[TEST_CONFIG.lidar_front.topic].peer_connected is False
        assert transport.snapshot().ecal_connected is True
    finally:
        transport.close()


def test_v61_partial_construction_removes_callback_before_finalize(monkeypatch):
    importer, _imported, core = _fake_v61_importer()
    monkeypatch.setattr("slope_sim.interfaces.ecal_transport.import_module", importer)
    bindings = load_ecal_bindings()
    _V61Publisher.fail_on_number = 3

    with pytest.raises(RuntimeError, match="publisher 3"):
        EcalTransport(_config(), bindings=bindings, start_worker=False)

    assert len(_V61Subscriber.instances) == 1
    assert _V61Subscriber.instances[0].remove_count == 1
    assert core.finalize_calls == 1


def test_partial_initialization_closes_every_resource_in_reverse_order():
    bindings = FakeEcalBindings()
    bindings.fail_on_publisher_number = 3

    with pytest.raises(RuntimeError, match="publisher 3"):
        EcalTransport(_config(), bindings=bindings, start_worker=False)

    assert bindings.open_subscribers == 0
    assert bindings.open_publishers == 0
    assert bindings.open_participants == 0
    assert bindings.close_log == [
        f"publisher:{TEST_CONFIG.lidar_front.topic}",
        f"publisher:{TEST_CONFIG.wheel_state.topic}",
        f"subscriber:{TEST_CONFIG.wheel_command.topic}",
        "participant:slope-sim",
    ]


def test_worker_start_failure_rolls_back_every_resource_and_preserves_error(monkeypatch):
    bindings = FakeEcalBindings()
    worker_targets: list[object] = []
    worker_joins: list[None] = []

    class StartFailingThread:
        def __init__(self, *, target, **_kwargs) -> None:
            worker_targets.append(target)
            self.alive = False

        def start(self) -> None:
            self.alive = True
            raise RuntimeError("worker start failed")

        def is_alive(self) -> bool:
            return self.alive

        def join(self) -> None:
            worker_joins.append(None)
            self.alive = False

    monkeypatch.setattr("slope_sim.interfaces.ecal_transport.Thread", StartFailingThread)

    with pytest.raises(RuntimeError, match="^worker start failed$"):
        EcalTransport(_config(), bindings=bindings)

    assert len(worker_targets) == 1
    assert worker_joins == [None]
    assert bindings.open_subscribers == 0
    assert bindings.open_publishers == 0
    assert bindings.open_participants == 0
    assert bindings.close_log == [
        *(f"publisher:{resource.topic}" for resource in reversed(bindings.publishers)),
        f"subscriber:{TEST_CONFIG.wheel_command.topic}",
        "participant:slope-sim",
    ]


def test_receive_callback_copies_payload_reads_clock_once_and_calls_upper_layer():
    bindings = FakeEcalBindings()
    clock_calls: list[None] = []

    def clock() -> float:
        clock_calls.append(None)
        return 12.5

    received: list[tuple[bytes, float]] = []
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        monotonic=clock,
        start_worker=False,
    )
    transport.subscribe(
        WHEEL_COMMAND_TOPIC,
        WHEEL_COMMAND_TYPE,
        lambda payload, received_at: received.append((payload, received_at)),
    )
    source = bytearray(b"command")
    try:
        _command_subscriber(bindings).emit(source)
        source[:] = b"changed"

        assert received == [(b"command", 12.5)]
        assert clock_calls == [None]
        assert transport.snapshot().received_count == 1
    finally:
        transport.close()


def test_outgoing_queue_keeps_latest_per_topic_and_counts_every_overwrite():
    bindings = FakeEcalBindings()
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        start_worker=False,
        queue_size=2,
    )
    try:
        assert transport.publish(
            WHEEL_STATE_TOPIC, b"old", WHEEL_STATE_TYPE, 10, wall_time=1.0
        )
        assert transport.publish(
            WHEEL_STATE_TOPIC, b"middle", WHEEL_STATE_TYPE, 20, wall_time=1.1
        )
        assert transport.publish(
            WHEEL_STATE_TOPIC, b"new", WHEEL_STATE_TYPE, 30, wall_time=1.2
        )

        snapshot = transport.snapshot()
        assert transport.pending_payload(WHEEL_STATE_TOPIC) == b"new"
        assert snapshot.dropped_count == 2
        assert snapshot.state == "degraded"
        assert snapshot.detail == "输出队列覆盖旧消息"
    finally:
        transport.close()


def test_full_queue_evicts_oldest_topic_but_preserves_each_remaining_latest_payload():
    bindings = FakeEcalBindings()
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        start_worker=False,
        queue_size=2,
    )
    try:
        lidar_type = TEST_CODEC.type_name(LidarPointCloud(0, "lidar_front", 0, 1, ()))
        transport.publish(WHEEL_STATE_TOPIC, b"wheel", WHEEL_STATE_TYPE, 1)
        transport.publish(
            TEST_CONFIG.lidar_front.topic,
            b"front",
            lidar_type,
            2,
        )
        transport.publish(
            TEST_CONFIG.lidar_rear.topic,
            b"rear",
            lidar_type,
            3,
        )

        assert transport.pending_payload(WHEEL_STATE_TOPIC) is None
        assert transport.pending_payload(TEST_CONFIG.lidar_front.topic) == b"front"
        assert transport.pending_payload(TEST_CONFIG.lidar_rear.topic) == b"rear"
        assert transport.snapshot().dropped_count == 1
    finally:
        transport.close()


def test_disconnect_reconnect_requires_current_generation_message_before_active():
    bindings = FakeEcalBindings()
    transitions: list[str] = []
    mailbox = WheelCommandMailbox(get_robot_model("df_back"))
    clock = [1.0]

    def peer_state_changed(state: str) -> None:
        transitions.append(state)
        if state == "disconnected":
            mailbox.clear()

    def accept_command(payload: bytes, received_at: float) -> bool:
        mailbox_ref = mailbox
        generation = mailbox_ref.capture_generation()
        command = TEST_CODEC.decode_wheel_command(payload)
        return mailbox_ref.accept(
            command,
            received_at=received_at,
            generation=generation,
        )

    transport = EcalTransport(
        _config(),
        bindings=bindings,
        monotonic=lambda: clock[0],
        peer_state_callback=peer_state_changed,
        start_worker=False,
    )
    subscriber = _command_subscriber(bindings)
    transport.subscribe(
        WHEEL_COMMAND_TOPIC,
        WHEEL_COMMAND_TYPE,
        accept_command,
    )
    try:
        subscriber.connected = True
        transport.poll_peer_state()
        subscriber.emit(_valid_command_payload(10))
        assert transport.snapshot().state == "active"
        assert mailbox.decision(now=clock[0]).drive_wheel_speed_rad_s == (4.0, 4.0)

        subscriber.connected = False
        transport.poll_peer_state()
        assert mailbox.decision(now=clock[0]).drive_wheel_speed_rad_s == (0.0, 0.0)

        subscriber.connected = True
        transport.poll_peer_state()
        assert transport.snapshot().state == "waiting_peer"
        assert mailbox.decision(now=clock[0]).drive_wheel_speed_rad_s == (0.0, 0.0)

        clock[0] = 1.01
        subscriber.emit(_valid_command_payload(20))
        assert transport.snapshot().state == "active"
        assert mailbox.decision(now=clock[0]).drive_wheel_speed_rad_s == (4.0, 4.0)
        assert transitions == ["active", "disconnected", "waiting_peer", "active"]
    finally:
        transport.close()


def test_entered_delivery_finishes_before_disconnect_clears_mailbox_generation():
    bindings = FakeEcalBindings()
    first_callback_started = Event()
    release_first_callback = Event()
    second_callback_started = Event()
    state_change_finished = Event()
    callback_errors: list[BaseException] = []
    accept_results: list[bool] = []
    mailbox = WheelCommandMailbox(get_robot_model("df_back"))

    def peer_state_changed(state: str) -> None:
        if state == "disconnected":
            mailbox.clear()

    transport = EcalTransport(
        _config(),
        bindings=bindings,
        monotonic=lambda: 1.0,
        peer_state_callback=peer_state_changed,
        start_worker=False,
    )
    subscriber = _command_subscriber(bindings)
    subscriber.connected = True
    transport.poll_peer_state()

    def delayed_callback(payload: bytes, received_at: float) -> bool:
        mailbox_ref = mailbox
        generation = mailbox_ref.capture_generation()
        first_callback_started.set()
        if not release_first_callback.wait(timeout=2.0):
            raise TimeoutError("callback release timed out")
        command = TEST_CODEC.decode_wheel_command(payload)
        accepted = mailbox_ref.accept(
            command,
            received_at=received_at,
            generation=generation,
        )
        accept_results.append(accepted)
        return accepted

    def later_callback(payload: bytes, received_at: float) -> bool:
        second_callback_started.set()
        mailbox_ref = mailbox
        generation = mailbox_ref.capture_generation()
        command = TEST_CODEC.decode_wheel_command(payload)
        accepted = mailbox_ref.accept(
            command,
            received_at=received_at,
            generation=generation,
        )
        accept_results.append(accepted)
        return accepted

    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, delayed_callback)
    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, later_callback)

    def emit() -> None:
        try:
            subscriber.emit(_valid_command_payload())
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            callback_errors.append(exc)

    def disconnect_and_reconnect() -> None:
        subscriber.connected = False
        transport.poll_peer_state()
        subscriber.connected = True
        transport.poll_peer_state()
        state_change_finished.set()

    callback_thread = Thread(target=emit, daemon=True)
    state_thread = Thread(target=disconnect_and_reconnect, daemon=True)
    try:
        callback_thread.start()
        assert first_callback_started.wait(timeout=2.0)
        state_thread.start()
        assert not state_change_finished.wait(timeout=0.1)
        release_first_callback.set()
        callback_thread.join(timeout=2.0)
        state_thread.join(timeout=2.0)

        assert not callback_thread.is_alive() and not state_thread.is_alive()
        assert callback_errors == []
        assert accept_results == [True, True]
        assert second_callback_started.is_set()
        assert mailbox.decision(now=1.0).drive_wheel_speed_rad_s == (0.0, 0.0)
        assert transport.snapshot().state == "waiting_peer"
    finally:
        release_first_callback.set()
        callback_thread.join(timeout=2.0)
        state_thread.join(timeout=2.0)
        transport.close()


def test_disconnect_waits_for_entered_delivery_before_clearing_mailbox_generation():
    """断线返回后，已进入的旧 payload 不得再向新代 mailbox 写命令。"""
    bindings = FakeEcalBindings()
    callback_entered = Event()
    release_callback = Event()
    state_change_finished = Event()
    mailbox = WheelCommandMailbox(get_robot_model("df_back"))
    accept_results: list[bool] = []

    def peer_state_changed(state: str) -> None:
        if state == "disconnected":
            mailbox.clear()

    transport = EcalTransport(
        _config(),
        bindings=bindings,
        monotonic=lambda: 1.0,
        peer_state_callback=peer_state_changed,
        start_worker=False,
    )
    subscriber = _command_subscriber(bindings)
    subscriber.connected = True
    transport.poll_peer_state()

    def delayed_entry(payload: bytes, received_at: float) -> bool:
        callback_entered.set()
        if not release_callback.wait(timeout=2.0):
            raise TimeoutError("callback release timed out")
        generation = mailbox.capture_generation()
        accepted = mailbox.accept(
            TEST_CODEC.decode_wheel_command(payload),
            received_at=received_at,
            generation=generation,
        )
        accept_results.append(accepted)
        return accepted

    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, delayed_entry)
    emit_thread = Thread(
        target=lambda: subscriber.emit(_valid_command_payload()),
        daemon=True,
    )

    def disconnect_and_reconnect() -> None:
        subscriber.connected = False
        transport.poll_peer_state()
        subscriber.connected = True
        transport.poll_peer_state()
        state_change_finished.set()

    state_thread = Thread(target=disconnect_and_reconnect, daemon=True)
    try:
        emit_thread.start()
        assert callback_entered.wait(timeout=2.0)
        state_thread.start()
        assert not state_change_finished.wait(timeout=0.1)

        release_callback.set()
        emit_thread.join(timeout=2.0)
        state_thread.join(timeout=2.0)

        assert not emit_thread.is_alive() and not state_thread.is_alive()
        assert accept_results == [True]
        assert mailbox.decision(now=1.0).drive_wheel_speed_rad_s == (0.0, 0.0)
        assert transport.snapshot().state == "waiting_peer"
    finally:
        release_callback.set()
        emit_thread.join(timeout=2.0)
        state_thread.join(timeout=2.0)
        transport.close()


def test_reentrant_disconnect_poll_is_deferred_until_delivery_returns():
    """同线程 discovery 轮询不得在当前 delivery 中途发布断线。"""
    bindings = FakeEcalBindings()
    mailbox = WheelCommandMailbox(get_robot_model("df_back"))
    callback_running = Event()
    disconnect_during_callback: list[bool] = []
    observed_states: list[str] = []
    accept_results: list[bool] = []

    def peer_state_changed(state: str) -> None:
        if state == "disconnected":
            disconnect_during_callback.append(callback_running.is_set())
            mailbox.clear()

    transport = EcalTransport(
        _config(),
        bindings=bindings,
        monotonic=lambda: 1.0,
        peer_state_callback=peer_state_changed,
        start_worker=False,
    )
    subscriber = _command_subscriber(bindings)
    subscriber.connected = True
    transport.poll_peer_state()

    def poll_then_accept(payload: bytes, received_at: float) -> bool:
        callback_running.set()
        try:
            subscriber.connected = False
            observed_states.append(transport.poll_peer_state())
            generation = mailbox.capture_generation()
            accepted = mailbox.accept(
                TEST_CODEC.decode_wheel_command(payload),
                received_at=received_at,
                generation=generation,
            )
            accept_results.append(accepted)
            return accepted
        finally:
            callback_running.clear()

    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, poll_then_accept)
    try:
        subscriber.emit(_valid_command_payload())

        assert observed_states == ["waiting_peer"]
        assert accept_results == [True]
        assert disconnect_during_callback == [False]
        assert transport.snapshot().state == "disconnected"
        assert mailbox.decision(now=1.0).drive_wheel_speed_rad_s == (0.0, 0.0)
    finally:
        transport.close()


def test_worker_sends_pending_payload_and_close_is_ordered_and_idempotent():
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=True)
    participant = bindings.participants[0]
    try:
        transport.publish(
            WHEEL_STATE_TOPIC, b"payload", WHEEL_STATE_TYPE, 10, wall_time=1.0
        )
        deadline = time.monotonic() + 2.0
        while not bindings.sent and time.monotonic() < deadline:
            time.sleep(0.005)
        assert bindings.sent == [(WHEEL_STATE_TOPIC, b"payload")]
        assert transport.snapshot().published_count == 1
    finally:
        transport.close()
        transport.close()

    assert transport.worker_alive is False
    assert participant.closed
    subscriber_indexes = [
        index
        for index, event in enumerate(bindings.close_log)
        if event.startswith("subscriber:")
    ]
    publisher_indexes = [
        index
        for index, event in enumerate(bindings.close_log)
        if event.startswith("publisher:")
    ]
    participant_index = bindings.close_log.index("participant:slope-sim")
    assert max(subscriber_indexes) < min(publisher_indexes) < participant_index


def test_queue_overwrite_snapshot_attributes_drop_to_evicted_topic_and_is_immutable():
    bindings = FakeEcalBindings()
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        start_worker=False,
        queue_size=1,
    )
    lidar_type = TEST_CODEC.type_name(LidarPointCloud(0, "lidar_front", 0, 1, ()))
    try:
        transport.publish(WHEEL_STATE_TOPIC, b"wheel-1", WHEEL_STATE_TYPE, 1)
        transport.publish(
            TEST_CONFIG.lidar_front.topic,
            b"front-1",
            lidar_type,
            2,
        )
        first = transport.snapshot()
        first_quality = {item.topic: item for item in first.topic_quality}

        assert first_quality[WHEEL_STATE_TOPIC].dropped_count == 1
        assert first_quality[WHEEL_STATE_TOPIC].state == "degraded"
        assert first_quality[WHEEL_STATE_TOPIC].detail
        assert first_quality[TEST_CONFIG.lidar_front.topic].dropped_count == 0
        with pytest.raises(FrozenInstanceError):
            first_quality[WHEEL_STATE_TOPIC].dropped_count = 9

        transport.publish(
            TEST_CONFIG.lidar_front.topic,
            b"front-2",
            lidar_type,
            3,
        )
        second_quality = {
            item.topic: item for item in transport.snapshot().topic_quality
        }
        assert first_quality[WHEEL_STATE_TOPIC].dropped_count == 1
        assert second_quality[TEST_CONFIG.lidar_front.topic].dropped_count == 1
    finally:
        transport.close()


def test_close_counts_each_unsent_pending_topic_once_without_publishing():
    bindings = FakeEcalBindings()
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        start_worker=False,
        queue_size=5,
    )
    types = _expected_types()
    pending_topics = (
        WHEEL_STATE_TOPIC,
        TEST_CONFIG.lidar_front.topic,
        TEST_CONFIG.rtk.topic,
    )
    for timestamp_ns, topic in enumerate(pending_topics, start=1):
        transport.publish(
            topic,
            f"pending-{timestamp_ns}".encode(),
            types[topic],
            timestamp_ns,
        )

    transport.close()
    first = transport.snapshot()
    first_quality = {quality.topic: quality for quality in first.topic_quality}

    assert first.state == "disconnected"
    assert first.published_count == 0
    assert first.dropped_count == len(pending_topics)
    assert bindings.sent == []
    for topic in pending_topics:
        quality = first_quality[topic]
        assert quality.dropped_count == 1
        assert quality.revision == 1
        assert quality.state == "degraded"
        assert quality.detail == "transport closed before pending message was sent"

    transport.close()
    second = transport.snapshot()
    assert second.dropped_count == first.dropped_count
    assert second.topic_quality == first.topic_quality


def test_quiesce_counts_pending_once_and_close_only_finalizes_resources():
    bindings = FakeEcalBindings()
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        start_worker=False,
        queue_size=5,
    )
    pending_topics = (WHEEL_STATE_TOPIC, TEST_CONFIG.rtk.topic)
    types = _expected_types()
    for timestamp_ns, topic in enumerate(pending_topics, start=1):
        transport.publish(topic, b"pending", types[topic], timestamp_ns)

    first = transport.quiesce()
    second = transport.quiesce()

    assert first.dropped_count == len(pending_topics)
    assert second.dropped_count == first.dropped_count
    assert second.topic_quality == first.topic_quality
    assert bindings.open_participants == 1
    assert bindings.open_subscribers == 1
    assert bindings.open_publishers == 5
    with pytest.raises(RuntimeError, match="closed"):
        transport.publish(WHEEL_STATE_TOPIC, b"late", WHEEL_STATE_TYPE, 3)

    transport.close()
    transport.close()
    final = transport.snapshot()
    assert final.dropped_count == first.dropped_count
    assert final.topic_quality == first.topic_quality
    assert bindings.open_participants == 0
    assert bindings.open_subscribers == 0
    assert bindings.open_publishers == 0


def test_runtime_logs_quiesced_ecal_pending_drop_before_logger_close(tmp_path):
    bindings = FakeEcalBindings()
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        start_worker=False,
        queue_size=1,
    )
    logger = InterfaceEventLogger(tmp_path, prefix="ecal-terminal-drop")
    runtime = InterfaceRuntime(
        RuntimeRobot(),
        config=_config(),
        transport=transport,
        monotonic=lambda: 1.0,
        logger=logger,
    )
    paths = logger.paths
    transport.publish(WHEEL_STATE_TOPIC, b"pending", WHEEL_STATE_TYPE, 1)

    runtime.close()

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
    assert len(transport_drops) == 1
    assert transport_drops[0]["count"] == 1
    assert transport_drops[0]["topic"] == WHEEL_STATE_TOPIC
    assert runtime.close_trace == (
        "stop_commands",
        "safe_stop",
        "stop_sensors",
        "quiesce_transport",
        "close_log",
        "close_transport",
        "close_sensors",
    )


def test_runtime_close_prioritizes_terminal_transport_drop_under_logger_backpressure(
    tmp_path,
):
    class BlockingLogWriter:
        """占住唯一 logger 名额，直到终态 transport 事件开始等待。"""

        def __init__(self) -> None:
            self.started = Event()
            self.release = Event()

        def __call__(self, stream, data):
            self.started.set()
            if not self.release.wait(timeout=5.0):
                raise TimeoutError("test logger writer was not released")
            return stream.write(data)

    bindings = FakeEcalBindings()
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        start_worker=False,
        queue_size=1,
    )
    writer = BlockingLogWriter()
    logger = InterfaceEventLogger(
        tmp_path,
        prefix="ecal-terminal-backpressure",
        queue_size=1,
        writer=writer,
    )
    runtime = InterfaceRuntime(
        RuntimeRobot(),
        config=_config(),
        transport=transport,
        monotonic=lambda: 1.0,
        logger=logger,
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
    runtime._record_runtime_event("sensor_failed", reason="create logger pending drop")
    assert runtime._pending_logger_drops == 1
    transport.publish(WHEEL_STATE_TOPIC, b"pending", WHEEL_STATE_TYPE, 1)

    closer = Thread(target=close_runtime, daemon=True)
    closer.start()
    try:
        assert terminal_started.wait(timeout=1.2)
        assert not close_completed.is_set()
    finally:
        writer.release.set()
    closer.join(timeout=2.0)

    assert not closer.is_alive()
    assert close_errors == []
    assert close_completed.is_set()
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
        (WHEEL_STATE_TOPIC, 1)
    ]
    logger_drops = [
        event
        for event in events
        if event["event"] == "queue_dropped"
        and event.get("source") == "interface_logger"
    ]
    assert [event["count"] for event in logger_drops] == [1]
    assert [call[0]["source"] for call in terminal_calls] == [
        "transport",
        "interface_logger",
    ]
    assert 0.0 <= terminal_calls[1][1] <= terminal_calls[0][1] <= 1.0
    assert logger.snapshot().dropped_events == 1


def test_close_counts_worker_claimed_frame_if_send_has_not_started() -> None:
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=True)
    transport._io_lock.acquire()
    io_lock_held = True
    close_errors: list[BaseException] = []

    def close_transport() -> None:
        try:
            transport.close()
        except BaseException as exc:
            close_errors.append(exc)

    closer = Thread(target=close_transport, daemon=True)
    try:
        transport.publish(WHEEL_STATE_TOPIC, b"claimed", WHEEL_STATE_TYPE, 1)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if transport.pending_payload(WHEEL_STATE_TOPIC) is None:
                break
            time.sleep(0.005)
        else:  # pragma: no cover - 超时路径由断言报告
            raise AssertionError("worker did not claim pending frame")

        closer.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with transport._condition:
                if transport._state == "closing":
                    break
            time.sleep(0.005)
        else:  # pragma: no cover - 超时路径由断言报告
            raise AssertionError("close did not enter closing state")

        transport._io_lock.release()
        io_lock_held = False
        closer.join(timeout=2.0)
        assert not closer.is_alive()
        assert close_errors == []

        first = transport.snapshot()
        quality = {
            item.topic: item for item in first.topic_quality
        }[WHEEL_STATE_TOPIC]
        assert bindings.sent == []
        assert first.published_count == 0
        assert first.dropped_count == 1
        assert quality.dropped_count == 1
        assert quality.revision == 1
        assert quality.state == "degraded"
        assert quality.detail == "transport closed before claimed message was sent"

        transport.close()
        second = transport.snapshot()
        assert second.dropped_count == first.dropped_count
        assert second.topic_quality == first.topic_quality
    finally:
        if io_lock_held:
            transport._io_lock.release()
        closer.join(timeout=2.0)
        if transport._state != "closed":
            transport.close()


def test_close_after_send_start_counts_successful_claim_as_published() -> None:
    bindings = FakeEcalBindings()
    send_started = Event()
    allow_send = Event()

    def blocked_send(publisher, payload: bytes, _message_type) -> None:
        send_started.set()
        assert allow_send.wait(timeout=2.0)
        bindings.sent.append((publisher.topic, bytes(payload)))

    bindings.send = blocked_send
    transport = EcalTransport(_config(), bindings=bindings, start_worker=True)
    close_errors: list[BaseException] = []

    def close_transport() -> None:
        try:
            transport.close()
        except BaseException as exc:
            close_errors.append(exc)

    closer = Thread(target=close_transport, daemon=True)
    try:
        transport.publish(WHEEL_STATE_TOPIC, b"sending", WHEEL_STATE_TYPE, 1)
        assert send_started.wait(timeout=2.0)
        closer.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with transport._condition:
                if transport._state == "closing":
                    break
            time.sleep(0.005)
        else:  # pragma: no cover - 超时路径由断言报告
            raise AssertionError("close did not overlap active send")

        allow_send.set()
        closer.join(timeout=2.0)
        assert not closer.is_alive()
        assert close_errors == []

        first = transport.snapshot()
        quality = {
            item.topic: item for item in first.topic_quality
        }[WHEEL_STATE_TOPIC]
        assert bindings.sent == [(WHEEL_STATE_TOPIC, b"sending")]
        assert first.published_count == 1
        assert first.dropped_count == 0
        assert quality.dropped_count == 0
        assert quality.revision == 0

        transport.close()
        second = transport.snapshot()
        assert second.published_count == first.published_count
        assert second.dropped_count == first.dropped_count
        assert second.topic_quality == first.topic_quality
    finally:
        allow_send.set()
        closer.join(timeout=2.0)
        if transport._state != "closed":
            transport.close()


def test_close_after_send_start_counts_failure_without_reopening_transport() -> None:
    bindings = FakeEcalBindings()
    send_started = Event()
    allow_send_failure = Event()

    def blocked_failing_send(_publisher, _payload: bytes, _message_type) -> None:
        send_started.set()
        if not allow_send_failure.wait(timeout=2.0):
            raise TimeoutError("failing send was not released")
        raise RuntimeError("send failed during close")

    bindings.send = blocked_failing_send
    transport = EcalTransport(_config(), bindings=bindings, start_worker=True)
    close_errors: list[BaseException] = []

    def close_transport() -> None:
        try:
            transport.close()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            close_errors.append(exc)

    closer = Thread(target=close_transport, daemon=True)
    try:
        transport.publish(WHEEL_STATE_TOPIC, b"failing", WHEEL_STATE_TYPE, 1)
        assert send_started.wait(timeout=2.0)
        closer.start()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            with transport._condition:
                if transport._state == "closing":
                    break
            time.sleep(0.005)
        else:  # pragma: no cover - 超时路径由断言报告
            raise AssertionError("close did not overlap failing send")

        allow_send_failure.set()
        closer.join(timeout=2.0)

        assert not closer.is_alive()
        assert close_errors == []
        snapshot = transport.snapshot()
        quality = {
            item.topic: item for item in snapshot.topic_quality
        }[WHEEL_STATE_TOPIC]
        assert bindings.sent == []
        assert snapshot.published_count == 0
        assert snapshot.dropped_count == 0
        assert snapshot.error_count == 1
        assert quality.error_count == 1
        assert quality.dropped_count == 0
        assert quality.state == "error"
        assert "send failed during close" in quality.detail
        assert snapshot.state == "disconnected"
        assert transport._state == "closed"
    finally:
        allow_send_failure.set()
        closer.join(timeout=2.0)
        if transport._state != "closed":
            transport.close()


def test_async_send_failure_and_success_update_only_its_topic_quality():
    bindings = FakeEcalBindings()
    attempts: list[bytes] = []
    first_failed = Event()
    recovered = Event()

    def fail_once(publisher, payload: bytes, message_type) -> None:
        attempts.append(bytes(payload))
        if len(attempts) == 1:
            first_failed.set()
            raise RuntimeError("wheel async send failed")
        bindings.sent.append((publisher.topic, bytes(payload)))
        recovered.set()

    bindings.send = fail_once
    transport = EcalTransport(_config(), bindings=bindings, start_worker=True)
    try:
        transport.publish(WHEEL_STATE_TOPIC, b"first", WHEEL_STATE_TYPE, 1)
        assert first_failed.wait(timeout=2.0)
        failed = {item.topic: item for item in transport.snapshot().topic_quality}
        assert failed[WHEEL_STATE_TOPIC].error_count == 1
        assert failed[WHEEL_STATE_TOPIC].state == "error"
        assert "wheel async send failed" in failed[WHEEL_STATE_TOPIC].detail
        assert all(
            item.error_count == 0
            for topic, item in failed.items()
            if topic != WHEEL_STATE_TOPIC
        )

        transport.publish(WHEEL_STATE_TOPIC, b"second", WHEEL_STATE_TYPE, 2)
        assert recovered.wait(timeout=2.0)
        quality = {item.topic: item for item in transport.snapshot().topic_quality}
        assert quality[WHEEL_STATE_TOPIC].error_count == 1
        assert quality[WHEEL_STATE_TOPIC].state == "active"
        assert quality[WHEEL_STATE_TOPIC].detail == ""
    finally:
        transport.close()


def test_worker_error_before_relay_attach_preserves_safe_state_and_recovers():
    bindings = FakeEcalBindings()
    send_failed = Event()

    def fail_send(_publisher, _payload: bytes, _message_type) -> None:
        send_failed.set()
        raise RuntimeError("worker failed before runtime attach")

    bindings.send = fail_send
    relay = _PeerStateRelay()
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        monotonic=lambda: 1.0,
        peer_state_callback=relay,
        start_worker=True,
    )
    transport.publish(WHEEL_STATE_TOPIC, b"first", WHEEL_STATE_TYPE, 1)
    assert send_failed.wait(timeout=2.0)
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        initial = transport.snapshot()
        wheel_quality = {
            quality.topic: quality for quality in initial.topic_quality
        }[WHEEL_STATE_TOPIC]
        if wheel_quality.error_count == 1:
            break
        time.sleep(0.005)
    else:  # pragma: no cover - 超时路径由断言报告
        raise AssertionError("worker error quality was not published")

    class RuntimeRobot:
        model_spec = get_robot_model("df_mid")

        def command_wheel_speeds(self, drive, steering=(), dt=1.0 / 240.0):
            return tuple(drive)

        def hold_current_steering_and_stop_drive(self, _dt: float) -> None:
            return None

        def read_interface_wheel_state(self, timestamp_ns: int) -> WheelState:
            return WheelState(timestamp_ns, (0.0, 0.0), ())

    runtime = InterfaceRuntime(
        RuntimeRobot(),
        config=_config(),
        transport=transport,
        monotonic=lambda: 1.0,
    )
    try:
        attached = relay.attach(runtime, transport)
        assert attached.state == "error"
        failed_status = runtime.status_snapshot(wall_time=1.0)
        assert failed_status.command.state == "disconnected"
        assert failed_status.topics[WHEEL_STATE_TOPIC].state == "error"
        assert failed_status.topics[WHEEL_STATE_TOPIC].error_count == 1
        assert "worker failed before runtime attach" in (
            failed_status.topics[WHEEL_STATE_TOPIC].detail
        )

        def send_success(publisher, payload: bytes, _message_type) -> None:
            bindings.sent.append((publisher.topic, bytes(payload)))

        bindings.send = send_success
        _command_subscriber(bindings).connected = True
        next(
            publisher
            for publisher in bindings.publishers
            if publisher.topic == WHEEL_STATE_TOPIC
        ).connected = True
        transport.publish(WHEEL_STATE_TOPIC, b"second", WHEEL_STATE_TYPE, 2)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            recovered_quality = {
                quality.topic: quality
                for quality in transport.snapshot().topic_quality
            }[WHEEL_STATE_TOPIC]
            if recovered_quality.state == "active":
                break
            time.sleep(0.005)
        else:  # pragma: no cover - 超时路径由断言报告
            raise AssertionError("worker topic quality did not recover")

        runtime.poll_transport()
        _command_subscriber(bindings).emit(_valid_command_payload(3))
        runtime.poll_transport()
        recovered = runtime.status_snapshot(wall_time=1.0)
        assert recovered.command.state == "active"
        assert recovered.topics[WHEEL_STATE_TOPIC].state == "active"
        assert recovered.topics[WHEEL_STATE_TOPIC].detail == ""
        assert recovered.topics[WHEEL_STATE_TOPIC].error_count == 1
    finally:
        runtime.close()


def test_rejected_callback_keeps_reconnected_transport_waiting_for_valid_command():
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=False)
    subscriber = _command_subscriber(bindings)
    transport.subscribe(
        WHEEL_COMMAND_TOPIC,
        WHEEL_COMMAND_TYPE,
        lambda _payload, _received_at: False,
    )
    try:
        subscriber.connected = True
        transport.poll_peer_state()
        subscriber.emit(_valid_command_payload())

        snapshot = transport.snapshot()
        assert snapshot.state == "waiting_peer"
        assert snapshot.ecal_connected is True
    finally:
        transport.close()


@pytest.mark.parametrize(
    ("callback_result", "expected_state"),
    ((True, "active"), (None, "active"), (False, "waiting_peer")),
)
def test_callback_result_contract_controls_current_generation_activation(
    callback_result,
    expected_state,
):
    """True/None 是成功，False 是明确拒绝且不能激活当前代。"""
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=False)
    subscriber = _command_subscriber(bindings)
    transport.subscribe(
        WHEEL_COMMAND_TOPIC,
        WHEEL_COMMAND_TYPE,
        lambda _payload, _received_at: callback_result,
    )
    try:
        subscriber.connected = True
        transport.poll_peer_state()
        subscriber.emit(_valid_command_payload())

        snapshot = transport.snapshot()
        assert snapshot.state == expected_state
        assert snapshot.error_count == 0
    finally:
        transport.close()


@pytest.mark.parametrize(
    ("callback_results", "expected_state"),
    (((True, None), "active"), ((False, True), "waiting_peer")),
)
def test_mixed_callback_results_require_success_without_explicit_rejection(
    callback_results,
    expected_state,
):
    """同次 delivery 至少一次成功且没有 False 才允许进入 active。"""
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=False)
    subscriber = _command_subscriber(bindings)
    for callback_result in callback_results:
        transport.subscribe(
            WHEEL_COMMAND_TOPIC,
            WHEEL_COMMAND_TYPE,
            lambda _payload, _received_at, result=callback_result: result,
        )
    try:
        subscriber.connected = True
        transport.poll_peer_state()
        subscriber.emit(_valid_command_payload())

        assert transport.snapshot().state == expected_state
    finally:
        transport.close()


def test_callback_exception_records_error_without_marking_transport_active():
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=False)
    subscriber = _command_subscriber(bindings)

    def fail_callback(_payload: bytes, _received_at: float) -> bool:
        raise RuntimeError("command validation failed")

    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, fail_callback)
    try:
        subscriber.connected = True
        transport.poll_peer_state()
        subscriber.emit(_valid_command_payload())

        snapshot = transport.snapshot()
        assert snapshot.state == "error"
        assert snapshot.error_count == 1
        assert "command validation failed" in snapshot.detail
    finally:
        transport.close()


def test_callback_error_before_disconnect_cannot_restore_post_disconnect_state():
    """已进入 callback 先结束，断线随后清命令并恢复 waiting_peer。"""
    bindings = FakeEcalBindings()
    callback_started = Event()
    release_callback = Event()
    later_callback_started = Event()
    state_change_finished = Event()
    stale_accept_results: list[bool] = []
    callback_errors: list[BaseException] = []
    mailbox = WheelCommandMailbox(get_robot_model("df_back"))

    def peer_state_changed(state: str) -> None:
        if state == "disconnected":
            mailbox.clear()

    transport = EcalTransport(
        _config(),
        bindings=bindings,
        monotonic=lambda: 1.0,
        peer_state_callback=peer_state_changed,
        start_worker=False,
    )
    subscriber = _command_subscriber(bindings)
    subscriber.connected = True
    transport.poll_peer_state()

    def delayed_failure(payload: bytes, received_at: float) -> bool:
        mailbox_ref = mailbox
        generation = mailbox_ref.capture_generation()
        callback_started.set()
        if not release_callback.wait(timeout=2.0):
            raise TimeoutError("callback release timed out")
        command = TEST_CODEC.decode_wheel_command(payload)
        stale_accept_results.append(
            mailbox_ref.accept(
                command,
                received_at=received_at,
                generation=generation,
            )
        )
        raise RuntimeError("stale callback failure")

    def later_callback(_payload: bytes, _received_at: float) -> bool:
        later_callback_started.set()
        return True

    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, delayed_failure)
    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, later_callback)

    def emit() -> None:
        try:
            subscriber.emit(_valid_command_payload())
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            callback_errors.append(exc)

    def disconnect_and_reconnect() -> None:
        subscriber.connected = False
        transport.poll_peer_state()
        subscriber.connected = True
        transport.poll_peer_state()
        state_change_finished.set()

    callback_thread = Thread(target=emit, daemon=True)
    state_thread = Thread(target=disconnect_and_reconnect, daemon=True)
    try:
        callback_thread.start()
        assert callback_started.wait(timeout=2.0)
        state_thread.start()
        assert not state_change_finished.wait(timeout=0.1)

        release_callback.set()
        callback_thread.join(timeout=2.0)
        state_thread.join(timeout=2.0)

        snapshot = transport.snapshot()
        assert not callback_thread.is_alive() and not state_thread.is_alive()
        assert callback_errors == []
        assert stale_accept_results == [True]
        assert later_callback_started.is_set()
        assert mailbox.decision(now=1.0).drive_wheel_speed_rad_s == (0.0, 0.0)
        assert snapshot.state == "waiting_peer"
        assert snapshot.error_count == 1
        assert snapshot.detail == ""
    finally:
        release_callback.set()
        callback_thread.join(timeout=2.0)
        state_thread.join(timeout=2.0)
        transport.close()


def test_successful_callback_can_activate_without_erasing_other_callback_error():
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=False)
    subscriber = _command_subscriber(bindings)

    def fail_callback(_payload: bytes, _received_at: float) -> bool:
        raise RuntimeError("observer failed")

    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, fail_callback)
    transport.subscribe(
        WHEEL_COMMAND_TOPIC,
        WHEEL_COMMAND_TYPE,
        lambda _payload, _received_at: True,
    )
    try:
        subscriber.connected = True
        transport.poll_peer_state()
        subscriber.emit(_valid_command_payload())

        snapshot = transport.snapshot()
        assert snapshot.state == "active"
        assert snapshot.error_count == 1
        assert "observer failed" in snapshot.detail
    finally:
        transport.close()


def test_short_frequency_burst_fails_count_and_window_coverage_gate():
    result = _short_burst_roundtrip_result()

    with pytest.raises(AssertionError, match="count|coverage"):
        verifier._assert_roundtrip_result(result)


def test_roundtrip_gate_rejects_peer_or_transport_quality_counters():
    result = _short_burst_roundtrip_result()
    object.__setattr__(result, "message_counts", {
        channel.topic: round(2.5 * channel.rate_hz)
        for channel in TEST_CONFIG.channels
    })
    object.__setattr__(result, "event_span_sec", {topic: 2.4 for topic in _expected_types()})
    object.__setattr__(result, "peer_dropped_count", 1)

    with pytest.raises(AssertionError, match="dropped"):
        verifier._assert_roundtrip_result(result)


def test_output_event_type_comes_from_decoded_message_and_configured_topic():
    decode_event = getattr(peer_script, "_decode_output_event", None)
    assert callable(decode_event), "peer must expose configured payload decoding"

    message = WheelState(10, (1.0, 2.0), ())
    timestamp_ns, type_name = decode_event(
        TEST_CONFIG,
        TEST_CODEC,
        TEST_CONFIG.wheel_state.topic,
        TEST_CODEC.encode(message),
    )
    assert timestamp_ns == 10
    assert type_name == TEST_CODEC.type_name(message)

    wrong_config = replace(
        TEST_CONFIG,
        wheel_state=ChannelConfig(
            "/wrong/wheel/state",
            TEST_CONFIG.wheel_state.rate_hz,
            "publish",
        ),
    )
    with pytest.raises(KeyError, match="topic"):
        decode_event(
            wrong_config,
            TEST_CODEC,
            TEST_CONFIG.wheel_state.topic,
            TEST_CODEC.encode(message),
        )
    with pytest.raises(ValueError, match="decode"):
        decode_event(
            TEST_CONFIG,
            TEST_CODEC,
            TEST_CONFIG.wheel_state.topic,
            b"\x80",
        )


def test_reconnect_evidence_requires_forced_exit_and_nonzero_targets():
    validate = getattr(verifier, "_assert_reconnect_result", None)
    assert callable(validate), "reconnect gate must validate control evidence"
    evidence = SimpleNamespace(
        transport_name="ecal",
        states=("active", "disconnected", "waiting_peer", "active"),
        drive_target_before_disconnect=(4.0, 4.0),
        first_peer_terminated=True,
        first_peer_returncode=-15,
        first_peer_runtime_sec=0.25,
        first_peer_planned_duration_sec=5.0,
        drive_target_while_disconnected=(0.0, 0.0),
        drive_target_after_peer_restart_before_new_command=(0.0, 0.0),
        silence_observed_sec=0.15,
        silence_sample_count=10,
        silence_all_zero=True,
        drive_target_after_new_command=(4.0, 4.0),
    )
    validate(evidence, command=(4.0, 4.0), silence_sec=0.15)

    for field_name, invalid_value in (
        ("drive_target_before_disconnect", (0.0, 0.0)),
        ("first_peer_returncode", 0),
        ("silence_observed_sec", 0.05),
        ("silence_all_zero", False),
        ("drive_target_after_new_command", (0.0, 0.0)),
    ):
        invalid = SimpleNamespace(**vars(evidence))
        setattr(invalid, field_name, invalid_value)
        with pytest.raises(AssertionError):
            validate(invalid, command=(4.0, 4.0), silence_sec=0.15)


def test_running_peer_is_explicitly_terminated_and_returns_nonzero():
    terminate = getattr(verifier, "_terminate_running_peer", None)
    assert callable(terminate), "reconnect gate must explicitly terminate its first peer"

    class FakeProcess:
        returncode: int | None = None
        terminate_count = 0

        def poll(self):
            return self.returncode

        def terminate(self) -> None:
            self.terminate_count += 1
            self.returncode = -15

        def communicate(self, timeout: float):
            assert timeout > 0.0
            return "", ""

    process = FakeProcess()
    returncode = terminate(process, timeout_sec=1.0)

    assert process.terminate_count == 1
    assert returncode == -15


def test_callback_on_other_transport_quiesce_waits_for_its_started_delivery():
    bindings_a = FakeEcalBindings()
    bindings_b = FakeEcalBindings()
    transport_a = EcalTransport(_config(), bindings=bindings_a, start_worker=False)
    transport_b = EcalTransport(_config(), bindings=bindings_b, start_worker=False)
    callback_b_started = Event()
    release_callback_b = Event()
    cross_quiesce_started = Event()
    cross_quiesce_returned = Event()
    emit_errors: list[BaseException] = []

    def callback_b(_payload: bytes, _received_at: float) -> bool:
        callback_b_started.set()
        if not release_callback_b.wait(timeout=5.0):
            raise TimeoutError("transport B callback was not released")
        return True

    def callback_a(_payload: bytes, _received_at: float) -> bool:
        cross_quiesce_started.set()
        transport_b.quiesce()
        cross_quiesce_returned.set()
        return True

    transport_b.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, callback_b)
    transport_a.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, callback_a)

    def emit(resource: FakeResource) -> None:
        try:
            resource.emit(_valid_command_payload())
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            emit_errors.append(exc)

    emitter_b = Thread(target=emit, args=(_command_subscriber(bindings_b),), daemon=True)
    emitter_a = Thread(target=emit, args=(_command_subscriber(bindings_a),), daemon=True)
    emitter_b.start()
    assert callback_b_started.wait(timeout=2.0)
    emitter_a.start()
    assert cross_quiesce_started.wait(timeout=2.0)
    returned_while_b_was_blocked = cross_quiesce_returned.wait(timeout=0.2)

    release_callback_b.set()
    emitter_b.join(timeout=2.0)
    emitter_a.join(timeout=2.0)
    transport_a.close()
    transport_b.close()

    assert not returned_while_b_was_blocked
    assert cross_quiesce_returned.is_set()
    assert not emitter_a.is_alive() and not emitter_b.is_alive()
    assert emit_errors == []


def test_callback_close_uses_async_cleanup_and_external_closers_share_completion():
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=True)
    subscriber = _command_subscriber(bindings)
    subscriber.wait_for_callback_on_close = True
    callback_close_returned = Event()
    allow_callback_return = Event()
    emitter_errors: list[BaseException] = []
    external_errors: list[BaseException] = []
    external_returned = [Event(), Event()]

    def close_in_callback(_payload: bytes, _received_at: float) -> bool:
        transport.close()
        callback_close_returned.set()
        if not allow_callback_return.wait(timeout=2.0):
            raise TimeoutError("callback return was not released")
        return True

    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, close_in_callback)

    def emit() -> None:
        try:
            subscriber.emit(_valid_command_payload())
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            emitter_errors.append(exc)

    def close_externally(index: int) -> None:
        try:
            transport.close()
        except BaseException as exc:  # pragma: no cover - 汇合后断言
            external_errors.append(exc)
        finally:
            external_returned[index].set()

    emitter = Thread(target=emit, daemon=True)
    closers = [
        Thread(target=close_externally, args=(index,), daemon=True)
        for index in range(2)
    ]
    emitter.start()
    try:
        assert callback_close_returned.wait(timeout=0.5)
        assert subscriber.close_started.wait(timeout=0.5)
        for closer in closers:
            closer.start()
        assert not any(event.wait(timeout=0.1) for event in external_returned)

        allow_callback_return.set()
        emitter.join(timeout=2.0)
        for closer in closers:
            closer.join(timeout=2.0)

        assert not emitter.is_alive()
        assert not any(closer.is_alive() for closer in closers)
        assert emitter_errors == []
        assert external_errors == []
        assert all(event.is_set() for event in external_returned)
        assert transport.worker_alive is False
        assert all(resource.close_count == 1 for resource in bindings.subscribers)
        assert all(resource.close_count == 1 for resource in bindings.publishers)
        assert all(resource.close_count == 1 for resource in bindings.participants)
        subscriber_indexes = [
            index for index, event in enumerate(bindings.close_log)
            if event.startswith("subscriber:")
        ]
        publisher_indexes = [
            index for index, event in enumerate(bindings.close_log)
            if event.startswith("publisher:")
        ]
        participant_index = bindings.close_log.index("participant:slope-sim")
        assert max(subscriber_indexes) < min(publisher_indexes) < participant_index
    finally:
        allow_callback_return.set()
        subscriber.callback_returned.set()
        emitter.join(timeout=2.0)
        for closer in closers:
            if closer.ident is not None:
                closer.join(timeout=2.0)
        transport.close()


@pytest.mark.parametrize("target_started", (False, True))
def test_cleanup_thread_start_failure_is_recovered_by_safe_close_owner(
    monkeypatch,
    target_started: bool,
) -> None:
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=False)
    subscriber = _command_subscriber(bindings)
    callback_close_finished = Event()
    spawned_threads: list[Thread] = []

    class FailingCleanupThread:
        def __init__(self, *, target, **_kwargs) -> None:
            self._target = target

        def start(self) -> None:
            if target_started:
                entered = Event()

                def run_target() -> None:
                    entered.set()
                    self._target()

                thread = Thread(target=run_target, daemon=True)
                spawned_threads.append(thread)
                thread.start()
                assert entered.wait(timeout=1.0)
            raise RuntimeError("cleanup thread start failed")

    monkeypatch.setattr(
        "slope_sim.interfaces.ecal_transport.Thread",
        FailingCleanupThread,
    )

    def close_in_callback(_payload: bytes, _received_at: float) -> bool:
        try:
            transport.close()
        finally:
            callback_close_finished.set()
        return True

    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, close_in_callback)
    emitter = Thread(target=lambda: subscriber.emit(_valid_command_payload()), daemon=True)
    external_errors: list[BaseException] = []
    external_done = Event()

    def close_externally() -> None:
        try:
            transport.close()
        except BaseException as exc:
            external_errors.append(exc)
        finally:
            external_done.set()

    closer = Thread(target=close_externally, daemon=True)
    try:
        emitter.start()
        assert callback_close_finished.wait(timeout=1.0)
        emitter.join(timeout=2.0)
        assert not emitter.is_alive()
        for thread in spawned_threads:
            thread.join(timeout=2.0)

        closer.start()
        assert external_done.wait(timeout=1.0)
        closer.join(timeout=1.0)
        assert not closer.is_alive()
        assert transport._state == "closed"
        assert len(external_errors) == 1
        assert str(external_errors[0]) == "cleanup thread start failed"
        with pytest.raises(RuntimeError, match="^cleanup thread start failed$"):
            transport.close()
        assert all(resource.close_count == 1 for resource in bindings.subscribers)
        assert all(resource.close_count == 1 for resource in bindings.publishers)
        assert all(resource.close_count == 1 for resource in bindings.participants)
    finally:
        if transport._state != "closed":
            transport._cleanup_resources()
        closer.join(timeout=2.0)
        for thread in spawned_threads:
            thread.join(timeout=2.0)


@pytest.mark.parametrize("start_worker", (False, True))
def test_native_callback_stack_never_synchronously_destroys_its_subscriber(
    monkeypatch,
    start_worker: bool,
) -> None:
    bindings = FakeEcalBindings()
    transport = EcalTransport(
        _config(),
        bindings=bindings,
        start_worker=start_worker,
    )
    subscriber = _command_subscriber(bindings)
    subscriber.wait_for_callback_on_close = True
    callback_close_returned = Event()
    emitter_returned = Event()
    external_returned = Event()
    external_errors: list[BaseException] = []

    class FailingCleanupThread:
        def __init__(self, *, target, **_kwargs) -> None:
            self._target = target

        def start(self) -> None:
            raise RuntimeError("native callback cleanup dispatch failed")

    monkeypatch.setattr(
        "slope_sim.interfaces.ecal_transport.Thread",
        FailingCleanupThread,
    )

    def close_in_callback(_payload: bytes, _received_at: float) -> bool:
        try:
            transport.close()
        except RuntimeError as exc:
            assert str(exc) == "native callback cleanup dispatch failed"
        callback_close_returned.set()
        return True

    transport.subscribe(WHEEL_COMMAND_TOPIC, WHEEL_COMMAND_TYPE, close_in_callback)

    def emit() -> None:
        try:
            subscriber.emit(_valid_command_payload())
        finally:
            emitter_returned.set()

    def close_externally() -> None:
        try:
            transport.close()
        except BaseException as exc:
            external_errors.append(exc)
        finally:
            external_returned.set()

    emitter = Thread(target=emit, daemon=True)
    closer = Thread(target=close_externally, daemon=True)
    closer_started = False
    try:
        emitter.start()
        assert callback_close_returned.wait(timeout=1.0)
        if not start_worker:
            closer.start()
            closer_started = True

        assert emitter_returned.wait(timeout=1.0)
        if start_worker:
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                with transport._condition:
                    if transport._state == "closed":
                        break
                time.sleep(0.005)
            else:  # pragma: no cover - 超时路径由断言报告
                raise AssertionError("publisher worker did not claim cleanup")
            closer.start()
            closer_started = True
        assert external_returned.wait(timeout=1.0)
        emitter.join(timeout=1.0)
        closer.join(timeout=1.0)

        assert not emitter.is_alive() and not closer.is_alive()
        assert transport._state == "closed"
        assert len(external_errors) == 1
        assert str(external_errors[0]) == "native callback cleanup dispatch failed"
        assert all(resource.close_count == 1 for resource in bindings.subscribers)
        assert all(resource.close_count == 1 for resource in bindings.publishers)
        assert all(resource.close_count == 1 for resource in bindings.participants)
    finally:
        # 仅用于让 RED 版本从自锁中退出，不参与成功路径断言。
        subscriber.callback_returned.set()
        emitter.join(timeout=2.0)
        if closer_started:
            closer.join(timeout=2.0)
        if transport._state != "closed":
            transport._cleanup_resources()


def test_worker_defers_failed_cleanup_dispatch_until_after_send_epilogue(
    monkeypatch,
) -> None:
    bindings = FakeEcalBindings()
    transport = EcalTransport(_config(), bindings=bindings, start_worker=True)
    original_cleanup = transport._cleanup_resources
    inside_send = Event()
    send_returned = Event()
    cleanup_contexts: list[bool] = []
    fallback_threads: list[Thread] = []

    class FailingCleanupThread:
        def __init__(self, *, target, **_kwargs) -> None:
            self._target = target

        def start(self) -> None:
            raise RuntimeError("worker cleanup dispatch failed")

    def observed_cleanup() -> None:
        called_inside_send = inside_send.is_set()
        cleanup_contexts.append(called_inside_send)
        if not called_inside_send:
            original_cleanup()
            return

        # 旧实现会在持有 _io_lock 时错误地同步进入这里；异步收尾仅防测试挂死。
        def finish_after_send() -> None:
            assert send_returned.wait(timeout=2.0)
            original_cleanup()

        fallback = Thread(target=finish_after_send, daemon=True)
        fallback_threads.append(fallback)
        fallback.start()

    def close_from_send(_publisher, _payload: bytes, _message_type) -> None:
        inside_send.set()
        try:
            transport.close()
        finally:
            inside_send.clear()
            send_returned.set()

    transport._cleanup_resources = observed_cleanup
    bindings.send = close_from_send
    monkeypatch.setattr(
        "slope_sim.interfaces.ecal_transport.Thread",
        FailingCleanupThread,
    )
    transport.publish(WHEEL_STATE_TOPIC, b"close", WHEEL_STATE_TYPE, 1)
    assert send_returned.wait(timeout=2.0)
    for thread in fallback_threads:
        thread.join(timeout=2.0)

    external_errors: list[BaseException] = []

    def close_externally() -> None:
        try:
            transport.close()
        except BaseException as exc:
            external_errors.append(exc)

    closer = Thread(target=close_externally, daemon=True)
    closer.start()
    closer.join(timeout=2.0)

    assert not closer.is_alive()
    assert cleanup_contexts == [False]
    assert transport._state == "closed"
    assert len(external_errors) == 1
    assert str(external_errors[0]) == "worker cleanup dispatch failed"
    assert all(resource.close_count == 1 for resource in bindings.subscribers)
    assert all(resource.close_count == 1 for resource in bindings.publishers)
    assert all(resource.close_count == 1 for resource in bindings.participants)


def test_peer_state_callback_close_dispatches_cleanup_off_callback_thread():
    """peer-state callback 关闭时不能同步等待当前 delivery。"""
    bindings = FakeEcalBindings()
    callback_thread_ids: list[int] = []
    cleanup_thread_ids: list[int] = []
    cleanup_finished = Event()
    transport = None

    def peer_state_changed(state: str) -> None:
        if state == "active":
            callback_thread_ids.append(get_ident())
            assert transport is not None
            transport.close()

    transport = EcalTransport(
        _config(),
        bindings=bindings,
        peer_state_callback=peer_state_changed,
        start_worker=False,
    )

    def controlled_cleanup() -> None:
        cleanup_thread_ids.append(get_ident())
        with transport._condition:
            transport._state = "closed"
            transport._condition.notify_all()
        cleanup_finished.set()

    transport._cleanup_resources = controlled_cleanup
    transport.subscribe(
        WHEEL_COMMAND_TOPIC,
        WHEEL_COMMAND_TYPE,
        lambda _payload, _received_at: True,
    )

    _command_subscriber(bindings).emit(_valid_command_payload())

    assert cleanup_finished.wait(timeout=1.0)
    assert callback_thread_ids
    assert cleanup_thread_ids
    assert cleanup_thread_ids != callback_thread_ids
    transport.close()


def test_custom_channel_rates_drive_command_and_output_schedules():
    """命令与五输出调度都只从各自 ChannelConfig 读取频率。"""
    config = _custom_rate_config()
    codec = ProtoCodec()

    command_clock = _FakeScheduleClock()
    command_transport = _RecordingTransport()
    run_commands = getattr(peer_script, "_run_command_schedule", None)
    assert callable(run_commands), "peer must expose a config-driven command schedule"
    command_events = run_commands(
        command_transport,
        duration_sec=1.0,
        command_delay_sec=0.0,
        drive_command=(4.0, 4.0),
        config=config,
        codec=codec,
        monotonic=command_clock.monotonic,
        sleep=command_clock.sleep,
    )

    output_clock = _FakeScheduleClock()
    output_transport = _RecordingTransport()
    verifier._run_output_schedule(
        output_transport,
        duration_sec=1.0,
        config=config,
        monotonic=output_clock.monotonic,
        sleep=output_clock.sleep,
    )

    assert len(command_events) == config.wheel_command.rate_hz
    assert len(command_transport.published) == config.wheel_command.rate_hz
    for channel in config.channels:
        published = (
            command_transport.published
            if channel is config.wheel_command
            else [
                event
                for event in output_transport.published
                if event[0] == channel.topic
            ]
        )
        assert len(published) == channel.rate_hz
        assert verifier._timestamp_frequency_hz(
            [event[3] for event in published]
        ) == pytest.approx(float(channel.rate_hz), rel=1e-9)


def test_roundtrip_gate_uses_custom_channel_rates_for_every_topic():
    """验收频率、计数和覆盖窗口随配置变化，不引用默认数值。"""
    config = _custom_rate_config()
    expected_types = _expected_types(config)
    rates = {channel.topic: float(channel.rate_hz) for channel in config.channels}
    duration_sec = 1.0
    result = verifier.RoundtripResult(
        transport_name="ecal",
        peer_returncode=0,
        wall_clock_hz=rates,
        message_timestamp_hz=rates,
        received_topics=set(expected_types) - {config.wheel_command.topic},
        topic_types=expected_types,
        message_counts={
            topic: round(duration_sec * rate) for topic, rate in rates.items()
        },
        dropped_count=0,
        duration_sec=duration_sec,
        event_span_sec={topic: 0.9 for topic in expected_types},
        peer_dropped_count=0,
        transport_error_count=0,
        peer_error_count=0,
    )

    verifier._assert_roundtrip_result(result, config=config, codec=TEST_CODEC)
