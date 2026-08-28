"""阶段四 A：验证 raw eCAL 回调复制、worker 顺序和远端 metadata gate。"""
from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from importlib import import_module

import pytest


@dataclass
class _EntityId:
    entity_id: int
    process_id: int
    host_name: str


@dataclass
class _TopicId:
    topic_id: _EntityId


@dataclass
class _DataTypeInformation:
    name: str
    encoding: str
    descriptor: object


@dataclass
class _ShmPublisherConfiguration:
    memfile_buffer_count: int = 1


@dataclass
class _PublisherLayerConfiguration:
    shm: _ShmPublisherConfiguration


@dataclass
class _PublisherConfiguration:
    layer: _PublisherLayerConfiguration


@dataclass
class _RawData:
    buffer: object
    send_timestamp: int
    send_clock: int


@dataclass
class _MonitoringTopic:
    topic_name: str
    datatype_information: _DataTypeInformation
    topic_id: int = 1


class _FakeSubscriber:
    """模拟原始三参数 receive callback，并记录注册签名。"""

    def __init__(self, topic: str, type_info: _DataTypeInformation) -> None:
        self.topic = topic
        self.type_info = type_info
        self.callback = None
        self.callback_argument_count = 0

    def set_receive_callback(self, callback) -> None:
        self.callback = callback
        self.callback_argument_count = callback.__code__.co_argcount

    def emit(
        self,
        payload: object,
        *,
        publisher_id: _TopicId,
        data_type_info: _DataTypeInformation,
        send_timestamp: int,
        send_clock: int,
    ) -> None:
        assert self.callback is not None
        self.callback(
            publisher_id,
            data_type_info,
            _RawData(payload, send_timestamp, send_clock),
        )


class _FakePublisher:
    def __init__(
        self,
        topic: str,
        type_info: _DataTypeInformation,
        configuration: _PublisherConfiguration,
    ) -> None:
        self.topic = topic
        self.type_info = type_info
        self.configuration = configuration
        self.sent: list[bytes] = []

    def send(self, payload: object) -> None:
        self.sent.append(bytes(payload))


class _FakeMonitoring:
    def __init__(self) -> None:
        self.publishers: list[_MonitoringTopic] = []
        self.subscribers: list[_MonitoringTopic] = []

    def get_monitoring(self):
        return self


class _FakeCore:
    """最小 eCAL raw surface，不包含 participant 或 native 状态。"""

    DataTypeInformation = _DataTypeInformation

    def __init__(self) -> None:
        self.subscribers: list[_FakeSubscriber] = []
        self.publishers: list[_FakePublisher] = []
        self.monitoring = _FakeMonitoring()

    def get_publisher_configuration(self) -> _PublisherConfiguration:
        return _PublisherConfiguration(_PublisherLayerConfiguration(_ShmPublisherConfiguration()))

    def Publisher(
        self,
        topic: str,
        type_info: _DataTypeInformation,
        configuration: _PublisherConfiguration,
    ) -> _FakePublisher:
        publisher = _FakePublisher(topic, type_info, configuration)
        self.publishers.append(publisher)
        return publisher

    def Subscriber(self, topic: str, type_info: _DataTypeInformation) -> _FakeSubscriber:
        subscriber = _FakeSubscriber(topic, type_info)
        self.subscribers.append(subscriber)
        return subscriber


def require_wished_module(name: str):
    """缺少 raw boundary 时保留可读 RED，而不破坏 pytest 收集。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


@pytest.fixture
def descriptor():
    """使用冻结 descriptor 验证远端 bytes 必须逐 byte 匹配。"""
    return require_wished_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()


@pytest.fixture
def fake_core():
    return _FakeCore()


@pytest.fixture
def remote_metadata(descriptor):
    return _DataTypeInformation(
        "slope_sim.interfaces.v2.WheelState",
        "proto",
        bytearray(descriptor.serialized_file_descriptor_set),
    )


def fake_topic(topic: str, type_name: str, descriptor: bytes, encoding: str = "proto") -> _MonitoringTopic:
    """构造 monitoring 端点；这里的 topic_id 故意保持与 raw TopicId 不同层级。"""
    return _MonitoringTopic(topic, _DataTypeInformation(type_name, encoding, descriptor))


def fake_topic_id(*, entity_id: int, process_id: int, host_name: str) -> _TopicId:
    """复刻 eCAL raw callback 的 TopicId -> EntityId 三层身份结构。"""
    return _TopicId(_EntityId(entity_id, process_id, host_name))


def test_raw_callback_only_copies_owned_envelope(fake_core, descriptor, remote_metadata) -> None:
    """native callback 只能复制当前帧，不得 hash、parse 或查 monitoring。"""
    bindings = require_wished_module("slope_sim.interfaces.v2.ecal_raw").EcalRawBindings(
        fake_core,
        monotonic=lambda: 4.5,
    )
    received = []
    bindings.create_subscriber(
        "/sim/wheel/state",
        "slope_sim.interfaces.v2.WheelState",
        descriptor,
        callback=received.append,
    )
    payload = bytearray(b"\x08\x01\x98\x01payload")
    fake_core.subscribers[0].emit(
        payload,
        publisher_id=fake_topic_id(entity_id=41, process_id=7, host_name="remote-host"),
        data_type_info=remote_metadata,
        send_timestamp=1234,
        send_clock=7,
    )
    payload[:] = b"changed"
    remote_metadata.name = "changed"
    remote_metadata.descriptor = b"changed"

    frame = received[0]
    assert frame.payload == b"\x08\x01\x98\x01payload"
    assert frame.remote_publisher_entity_id == 41
    assert frame.remote_publisher_process_id == 7
    assert frame.remote_publisher_host_name == "remote-host"
    assert frame.remote_type_name == "slope_sim.interfaces.v2.WheelState"
    assert frame.remote_descriptor == descriptor.serialized_file_descriptor_set
    assert frame.send_timestamp_us == 1234
    assert frame.send_clock == 7
    assert frame.received_at == 4.5
    assert not hasattr(frame, "payload_sha256")
    assert fake_core.subscribers[0].callback_argument_count == 3


def test_raw_publisher_sends_one_owned_payload_without_typed_reserialization(fake_core, descriptor) -> None:
    """发送边界只转交原始 bytes，禁止在 raw 层 ParseFromString。"""
    bindings = require_wished_module("slope_sim.interfaces.v2.ecal_raw").EcalRawBindings(fake_core)
    publisher = bindings.create_publisher(
        "/sim/wheel/state",
        "slope_sim.interfaces.v2.WheelState",
        descriptor,
    )
    bindings.send(publisher, bytearray(b"wire-bytes"))
    assert fake_core.publishers[0].sent == [b"wire-bytes"]


def test_raw_publisher_uses_a_bounded_shm_ring(fake_core, descriptor) -> None:
    """满负载 raw LiDAR 不得复用默认单缓冲 SHM 槽位。"""
    bindings = require_wished_module("slope_sim.interfaces.v2.ecal_raw").EcalRawBindings(fake_core)

    publisher = bindings.create_publisher(
        "/sim/lidar/points",
        "slope_sim.interfaces.v2.LidarPointCloud",
        descriptor,
    )

    assert publisher.configuration.layer.shm.memfile_buffer_count == 32


def test_worker_hashes_before_remote_validation_and_parse(descriptor) -> None:
    """worker 固定先 hash，再验证本帧 metadata，最后才调用 parser。"""
    module = require_wished_module("slope_sim.interfaces.v2.ecal_raw")
    raw_frame = module.RawReceivedFrame(
        payload=b"payload",
        remote_publisher_entity_id=41,
        remote_publisher_process_id=7,
        remote_publisher_host_name="remote-host",
        remote_type_name="slope_sim.interfaces.v2.WheelState",
        remote_encoding="proto",
        remote_descriptor=descriptor.serialized_file_descriptor_set,
        send_timestamp_us=1,
        send_clock=1,
        received_at=1.0,
    )
    order: list[str] = []

    def hash_payload(payload: bytes) -> bytes:
        order.append("hash")
        return sha256(payload).digest()

    def parse_payload(_payload: bytes) -> object:
        order.append("parse")
        return object()

    processed = module.process_raw_frame(
        raw_frame,
        expected_type="slope_sim.interfaces.v2.WheelState",
        descriptor=descriptor,
        parser=parse_payload,
        payload_hasher=hash_payload,
    )
    assert order == ["hash", "parse"]
    assert processed.payload_sha256 == sha256(raw_frame.payload).digest()


def test_worker_rejects_remote_metadata_before_parse(descriptor) -> None:
    """同名 topic 的错误 type、encoding 或 descriptor 都不能进入 parser。"""
    module = require_wished_module("slope_sim.interfaces.v2.ecal_raw")
    for type_name, encoding, raw_descriptor in (
        ("slope_sim.interfaces.v1.WheelState", "proto", descriptor.serialized_file_descriptor_set),
        ("slope_sim.interfaces.v2.WheelState", "json", descriptor.serialized_file_descriptor_set),
        ("slope_sim.interfaces.v2.WheelState", "proto", b"wrong"),
    ):
        frame = module.RawReceivedFrame(
            b"payload", 1, 2, "host", type_name, encoding, raw_descriptor, 1, 1, 1.0
        )
        with pytest.raises(ValueError, match="remote type/encoding/descriptor mismatch"):
            module.process_raw_frame(
                frame,
                expected_type="slope_sim.interfaces.v2.WheelState",
                descriptor=descriptor,
                parser=lambda _payload: pytest.fail("parser must not run"),
            )


def test_raw_metadata_classifies_empty_startup_metadata_as_pending(descriptor) -> None:
    """启动时 eCAL 尚未填充 metadata 只能 pending，完整非空不匹配才 conflict。"""
    module = require_wished_module("slope_sim.interfaces.v2.ecal_raw")
    expected = "slope_sim.interfaces.v2.WheelState"

    empty = module.RawReceivedFrame(b"wire", 1, 2, "host", "", "", b"", 1, 1, 1.0)
    valid = module.RawReceivedFrame(
        b"wire", 1, 2, "host", expected, "proto",
        descriptor.serialized_file_descriptor_set, 1, 1, 1.0,
    )
    conflict = module.RawReceivedFrame(
        b"wire", 1, 2, "host", "slope_sim.interfaces.v1.WheelState", "proto",
        descriptor.serialized_file_descriptor_set, 1, 1, 1.0,
    )

    classify = module.classify_raw_frame_metadata
    assert classify(empty, expected_type=expected, descriptor=descriptor) is module.ProtocolVerificationState.PENDING
    assert classify(valid, expected_type=expected, descriptor=descriptor) is module.ProtocolVerificationState.VERIFIED
    assert classify(conflict, expected_type=expected, descriptor=descriptor) is module.ProtocolVerificationState.CONFLICT


def test_raw_callback_preserves_none_metadata_as_empty_for_pending_gate(fake_core, descriptor) -> None:
    """native 暂态 None 需要复制为空字段交给 worker，而不是在 callback 内计协议错误。"""
    module = require_wished_module("slope_sim.interfaces.v2.ecal_raw")
    bindings = module.EcalRawBindings(fake_core, monotonic=lambda: 4.5)
    received: list[object] = []
    bindings.create_subscriber(
        "/sim/wheel/state", "slope_sim.interfaces.v2.WheelState", descriptor,
        callback=received.append,
    )

    fake_core.subscribers[0].emit(
        b"wire",
        publisher_id=fake_topic_id(entity_id=41, process_id=7, host_name="remote-host"),
        data_type_info=_DataTypeInformation(None, None, None),
        send_timestamp=1234,
        send_clock=7,
    )

    frame = received[0]
    assert frame.remote_type_name == ""
    assert frame.remote_encoding == ""
    assert frame.remote_descriptor == b""


def test_monitoring_maps_waiting_pending_verified_and_conflict(fake_core, descriptor) -> None:
    """远端 monitoring 按 exact count 做原子协议 gate，其他 topic 不污染结果。"""
    module = require_wished_module("slope_sim.interfaces.v2.ecal_raw")
    bindings = module.EcalRawBindings(fake_core)
    expected = "slope_sim.interfaces.v2.WheelCommand"
    snapshot = bindings.snapshot_remote_endpoints(
        topic="/sim/wheel/command",
        remote_direction="publisher",
        peer_count=0,
        expected_type=expected,
        descriptor=descriptor,
    )
    assert snapshot.verification.state is module.ProtocolVerificationState.WAITING

    fake_core.monitoring.publishers = [fake_topic("/other", expected, descriptor.serialized_file_descriptor_set)]
    snapshot = bindings.snapshot_remote_endpoints(
        topic="/sim/wheel/command",
        remote_direction="publisher",
        peer_count=1,
        expected_type=expected,
        descriptor=descriptor,
    )
    assert snapshot.verification.state is module.ProtocolVerificationState.PENDING

    fake_core.monitoring.publishers = [
        fake_topic("/sim/wheel/command", expected, descriptor.serialized_file_descriptor_set),
        fake_topic("/other", "wrong", b"wrong"),
    ]
    snapshot = bindings.snapshot_remote_endpoints(
        topic="/sim/wheel/command",
        remote_direction="publisher",
        peer_count=1,
        expected_type=expected,
        descriptor=descriptor,
    )
    assert snapshot.verification.state is module.ProtocolVerificationState.VERIFIED

    fake_core.monitoring.publishers.append(
        fake_topic("/sim/wheel/command", "slope_sim.interfaces.v1.WheelCommand", b"v1")
    )
    snapshot = bindings.snapshot_remote_endpoints(
        topic="/sim/wheel/command",
        remote_direction="publisher",
        peer_count=2,
        expected_type=expected,
        descriptor=descriptor,
    )
    assert snapshot.verification.state is module.ProtocolVerificationState.CONFLICT
    assert "mismatch" in snapshot.verification.detail


def test_monitoring_rejects_invalid_direction_and_peer_count(fake_core, descriptor) -> None:
    """monitoring 输入本身也必须严格，不能把 bool 或负数当 count。"""
    bindings = require_wished_module("slope_sim.interfaces.v2.ecal_raw").EcalRawBindings(fake_core)
    for peer_count in (True, -1):
        with pytest.raises(ValueError, match="peer_count"):
            bindings.snapshot_remote_endpoints(
                topic="/sim/wheel/command",
                remote_direction="publisher",
                peer_count=peer_count,
                expected_type="slope_sim.interfaces.v2.WheelCommand",
                descriptor=descriptor,
            )
    with pytest.raises(ValueError, match="remote_direction"):
        bindings.snapshot_remote_endpoints(
            topic="/sim/wheel/command",
            remote_direction="invalid",
            peer_count=0,
            expected_type="slope_sim.interfaces.v2.WheelCommand",
            descriptor=descriptor,
        )
