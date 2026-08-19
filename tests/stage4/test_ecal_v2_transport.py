"""阶段四 A Task 11：v2 raw eCAL transport 的有界发送与协议门禁。"""
from dataclasses import dataclass
from threading import Event, Thread
import time
import pytest

from slope_sim.interfaces.v2.ecal_raw import (
    ProtocolVerification,
    ProtocolVerificationState,
    RawReceivedFrame,
    RemoteEndpointSnapshot,
)


def require_wished_module(name: str):
    """让尚未实现的 v2 transport 表现为清晰 RED，而非收集错误。"""
    try:
        return __import__(name, fromlist=["*"])
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


@dataclass
class _FakeRawResource:
    """记录 raw channel 的方向、生命周期和原始发送内容。"""

    topic: str
    direction: str
    closed: bool = False


class FakeRawV2Bindings:
    """Task 11 首个 RED/GREEN 使用的最小 raw binding 端口。"""

    def __init__(self) -> None:
        self.sent_payloads: list[bytes] = []
        self.resources: list[_FakeRawResource] = []
        self.subscriber_callbacks: dict[str, object] = {}
        self.peer_counts: dict[str, int] = {}
        self.protocol_conflicts: dict[str, str] = {}
        self.protocol_verified: set[str] = set()

    def create_participant(self, _name: str) -> _FakeRawResource:
        resource = _FakeRawResource("participant", "participant")
        self.resources.append(resource)
        return resource

    def create_raw_subscriber(self, topic: str, _type_name: str, _descriptor: object, _callback) -> _FakeRawResource:
        resource = _FakeRawResource(topic, "subscriber")
        self.resources.append(resource)
        self.subscriber_callbacks[topic] = _callback
        return resource

    def create_raw_publisher(self, topic: str, _type_name: str, _descriptor: object) -> _FakeRawResource:
        resource = _FakeRawResource(topic, "publisher")
        self.resources.append(resource)
        return resource

    def send_raw(self, _publisher: _FakeRawResource, payload: bytes) -> None:
        self.sent_payloads.append(bytes(payload))

    def emit(self, topic: str, payload: object) -> None:
        """模拟 raw native callback 到达；测试端不伪造 metadata 已验证。"""
        callback = self.subscriber_callbacks[topic]
        callback(payload)

    def set_peer_count(self, topic: str, count: int) -> None:
        """为 discovery gate 提供精确 peer count。"""
        self.peer_counts[topic] = count

    def set_protocol_conflict(self, topic: str, remote_type_name: str) -> None:
        """模拟同名远端以 v1 metadata 占用 v2 topic。"""
        self.protocol_conflicts[topic] = remote_type_name

    def set_protocol_verified(self, topic: str) -> None:
        """模拟 monitoring 已对该 topic 的完整 metadata 达成一致。"""
        self.protocol_verified.add(topic)

    def snapshot_remote_endpoints(
        self,
        channel: object,
        resource: _FakeRawResource,
        peer_count: int,
    ) -> RemoteEndpointSnapshot:
        """返回与 count 对齐的 fake monitoring 结果。"""
        topic = resource.topic
        if peer_count == 0:
            return RemoteEndpointSnapshot(
                ProtocolVerification(ProtocolVerificationState.WAITING, 0, (), (), ())
            )
        remote_type = self.protocol_conflicts.get(topic, channel.type_name)
        state = (
            ProtocolVerificationState.CONFLICT
            if topic in self.protocol_conflicts
            else ProtocolVerificationState.VERIFIED
            if topic in self.protocol_verified
            else ProtocolVerificationState.PENDING
        )
        if state is ProtocolVerificationState.PENDING:
            return RemoteEndpointSnapshot(
                ProtocolVerification(state, peer_count, (), (), ())
            )
        descriptor_sha256 = channel.descriptor.sha256.hex()
        return RemoteEndpointSnapshot(
            ProtocolVerification(
                state,
                peer_count,
                (remote_type,),
                ("proto",),
                (descriptor_sha256,),
                "same-topic remote type/encoding/descriptor mismatch"
                if state is ProtocolVerificationState.CONFLICT
                else "",
            )
        )

    def peer_count(self, resource: _FakeRawResource) -> int:
        return self.peer_counts.get(resource.topic, 0)

    @staticmethod
    def close(resource: _FakeRawResource) -> None:
        resource.closed = True


@pytest.fixture
def descriptor():
    """使用冻结 v2 descriptor，避免 transport 自行生成另一份 metadata。"""
    module = require_wished_module("slope_sim.interfaces.v2.descriptor")
    return module.load_v2_descriptor()


def test_v2_transport_sends_exact_raw_bytes(descriptor) -> None:
    """v2 WheelState payload 必须直接进入 raw publisher，不能 typed 重序列化。"""
    module = require_wished_module("slope_sim.interfaces.v2.transport")
    factory = getattr(module, "create_v2_ecal_transport", None)
    assert callable(factory), "v2 raw transport factory is not implemented"
    bindings = FakeRawV2Bindings()
    transport = factory(descriptor=descriptor, bindings=bindings)
    payload = b"\x08\x01\x10\x02\x18\x03"
    try:
        transport.publish(
            "/sim/wheel/state",
            payload,
            "slope_sim.interfaces.v2.WheelState",
            10,
        )
        transport.wait_idle(timeout_sec=1.0)
        assert bindings.sent_payloads == [payload]
    finally:
        transport.close()


def test_v2_transport_rejects_raw_payload_while_metadata_is_pending(descriptor) -> None:
    """metadata 未 verified 时 raw callback 不得越过 transport delivery gate。"""
    module = require_wished_module("slope_sim.interfaces.v2.transport")
    bindings = FakeRawV2Bindings()
    transport = module.create_v2_ecal_transport(descriptor=descriptor, bindings=bindings)
    accepted: list[tuple[bytes, float]] = []
    try:
        transport.subscribe(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            lambda payload, _received_at: accepted.append(payload),
        )
        bindings.set_peer_count("/sim/wheel/command", 1)
        transport.poll_peer_state()
        bindings.emit("/sim/wheel/command", b"unverified-wire")
        assert accepted == []
        quality = next(
            item
            for item in transport.snapshot().topic_quality
            if item.topic == "/sim/wheel/command"
        )
        assert quality.protocol_state == "pending"
    finally:
        transport.close()


def test_v2_transport_latches_same_topic_protocol_conflict(descriptor) -> None:
    """v1 同名 metadata 冲突必须使 v2 transport 硬失败且拒绝后续交付。"""
    module = require_wished_module("slope_sim.interfaces.v2.transport")
    bindings = FakeRawV2Bindings()
    transport = module.create_v2_ecal_transport(descriptor=descriptor, bindings=bindings)
    try:
        bindings.set_peer_count("/sim/wheel/command", 1)
        bindings.set_protocol_conflict(
            "/sim/wheel/command", "slope_sim.interfaces.v1.WheelCommand"
        )
        ecal_module = require_wished_module("slope_sim.interfaces.ecal_transport")
        error_type = getattr(ecal_module, "ProtocolConflictError", None)
        assert error_type is not None, "ProtocolConflictError is not implemented"
        with pytest.raises(error_type, match="protocol conflict"):
            transport.poll_peer_state()
        quality = next(
            item
            for item in transport.snapshot().topic_quality
            if item.topic == "/sim/wheel/command"
        )
        assert quality.protocol_state == "conflict"
        assert quality.error_count == 1
    finally:
        transport.close()


def test_v2_transport_rejects_callback_frame_with_wrong_remote_metadata(descriptor) -> None:
    """verified discovery 不能替代 raw callback 本帧的 type/descriptor 校验。"""
    module = require_wished_module("slope_sim.interfaces.v2.transport")
    bindings = FakeRawV2Bindings()
    transport = module.create_v2_ecal_transport(descriptor=descriptor, bindings=bindings)
    accepted: list[tuple[bytes, float]] = []
    try:
        transport.subscribe(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            lambda payload, _received_at: accepted.append(payload),
        )
        bindings.set_peer_count("/sim/wheel/command", 1)
        bindings.set_protocol_verified("/sim/wheel/command")
        transport.poll_peer_state()
        bindings.emit(
            "/sim/wheel/command",
            RawReceivedFrame(
                payload=b"wrong-metadata",
                remote_publisher_entity_id=1,
                remote_publisher_process_id=2,
                remote_publisher_host_name="fixture",
                remote_type_name="slope_sim.interfaces.v1.WheelCommand",
                remote_encoding="proto",
                remote_descriptor=b"v1 descriptor",
                send_timestamp_us=1,
                send_clock=1,
                received_at=1.0,
            ),
        )
        assert accepted == []
        deadline = time.monotonic() + 1.0
        while True:
            quality = next(
                item
                for item in transport.snapshot().topic_quality
                if item.topic == "/sim/wheel/command"
            )
            if quality.error_count == 1 or time.monotonic() >= deadline:
                break
            time.sleep(0.01)
        assert quality.error_count == 1
    finally:
        transport.close()


def test_v2_transport_native_callback_only_enqueues_raw_frame(
    descriptor, monkeypatch
) -> None:
    """native callback 必须在 hash 前返回，耗时校验只能由 receive lane 执行。"""
    module = require_wished_module("slope_sim.interfaces.v2.transport")
    raw_module = require_wished_module("slope_sim.interfaces.v2.ecal_raw")
    bindings = FakeRawV2Bindings()
    transport = module.create_v2_ecal_transport(descriptor=descriptor, bindings=bindings)
    hasher_started = Event()
    release_hasher = Event()
    native_returned = Event()

    def blocking_metadata_validator(*_args, **_kwargs) -> bytes:
        hasher_started.set()
        assert release_hasher.wait(timeout=1.0)
        return b"h" * 32

    monkeypatch.setattr(raw_module, "validate_raw_frame_metadata", blocking_metadata_validator)
    try:
        bindings.set_peer_count("/sim/wheel/command", 1)
        bindings.set_protocol_verified("/sim/wheel/command")
        transport.poll_peer_state()
        frame = RawReceivedFrame(
            payload=b"wire",
            remote_publisher_entity_id=1,
            remote_publisher_process_id=2,
            remote_publisher_host_name="fixture",
            remote_type_name="slope_sim.interfaces.v2.WheelCommand",
            remote_encoding="proto",
            remote_descriptor=descriptor.serialized_file_descriptor_set,
            send_timestamp_us=1,
            send_clock=1,
            received_at=1.0,
        )
        native_thread = Thread(
            target=lambda: (bindings.emit("/sim/wheel/command", frame), native_returned.set()),
            daemon=True,
        )
        native_thread.start()
        assert hasher_started.wait(timeout=1.0)
        assert native_returned.wait(timeout=0.1), (
            "native raw callback must return before hash/parse/delivery work"
        )
    finally:
        release_hasher.set()
        native_thread.join(timeout=1.0) if "native_thread" in locals() else None
        transport.close()


def test_v2_transport_raw_receive_lane_keeps_owner_and_latest_frame(
    descriptor, monkeypatch
) -> None:
    """慢校验期间 command receive lane 只保留 owner 与最新帧，并准确计一次覆盖。"""
    module = require_wished_module("slope_sim.interfaces.v2.transport")
    raw_module = require_wished_module("slope_sim.interfaces.v2.ecal_raw")
    bindings = FakeRawV2Bindings()
    transport = module.create_v2_ecal_transport(descriptor=descriptor, bindings=bindings)
    first_started = Event()
    release_first = Event()
    latest_processed = Event()
    native_returned = [Event() for _ in range(3)]
    processed: list[bytes] = []

    def blocking_metadata_validator(frame, **_kwargs) -> bytes:
        processed.append(frame.payload)
        if frame.payload == b"first":
            first_started.set()
            assert release_first.wait(timeout=1.0)
        elif frame.payload == b"third":
            latest_processed.set()
        return b"h" * 32

    def frame(payload: bytes) -> RawReceivedFrame:
        return RawReceivedFrame(
            payload=payload,
            remote_publisher_entity_id=1,
            remote_publisher_process_id=2,
            remote_publisher_host_name="fixture",
            remote_type_name="slope_sim.interfaces.v2.WheelCommand",
            remote_encoding="proto",
            remote_descriptor=descriptor.serialized_file_descriptor_set,
            send_timestamp_us=1,
            send_clock=1,
            received_at=1.0,
        )

    monkeypatch.setattr(raw_module, "validate_raw_frame_metadata", blocking_metadata_validator)
    threads: list[Thread] = []
    try:
        bindings.set_peer_count("/sim/wheel/command", 1)
        bindings.set_protocol_verified("/sim/wheel/command")
        transport.poll_peer_state()
        for index, payload in enumerate((b"first", b"second", b"third")):
            thread = Thread(
                target=lambda payload=payload, index=index: (
                    bindings.emit("/sim/wheel/command", frame(payload)),
                    native_returned[index].set(),
                ),
                daemon=True,
            )
            threads.append(thread)
            thread.start()
            if index == 0:
                assert first_started.wait(timeout=1.0)
        assert native_returned[1].wait(timeout=0.1)
        assert native_returned[2].wait(timeout=0.1)
        snapshot = transport.snapshot()
        quality = next(
            item for item in snapshot.topic_quality if item.topic == "/sim/wheel/command"
        )
        assert snapshot.dropped_count == 1
        assert quality.dropped_count == 1
        release_first.set()
        assert latest_processed.wait(timeout=1.0)
        assert processed == [b"first", b"third"]
    finally:
        release_first.set()
        for thread in threads:
            thread.join(timeout=1.0)
        transport.close()


def test_v2_transport_delivers_verified_valid_raw_command(descriptor) -> None:
    """本帧 metadata 与 v2 codec 都通过后，才向上层交付原始 command bytes。"""
    module = require_wished_module("slope_sim.interfaces.v2.transport")
    codec_module = require_wished_module("slope_sim.interfaces.v2.codec")
    models = require_wished_module("slope_sim.interfaces.v2.models")
    bindings = FakeRawV2Bindings()
    transport = module.create_v2_ecal_transport(descriptor=descriptor, bindings=bindings)
    command = models.WheelCommandV2(
        timestamp_ns=20_000_000,
        drive_wheel_speed_rad_s=(1.25, -1.25),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=1,
        command_generation=1,
        source_id="manual.tool-1",
        source_session_id=bytes.fromhex("ffeeddccbbaa99887766554433221100"),
        robot_model="df_mid",
        simulation_session_id=bytes.fromhex("00112233445566778899aabbccddeeff"),
        descriptor_sha256=descriptor.sha256,
    )
    payload = codec_module.V2ProtoCodec(descriptor).encode(command).payload
    accepted: list[bytes] = []
    try:
        transport.subscribe(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            lambda wire, received_at: accepted.append((wire, received_at)),
        )
        bindings.set_peer_count("/sim/wheel/command", 1)
        bindings.set_protocol_verified("/sim/wheel/command")
        transport.poll_peer_state()
        bindings.emit(
            "/sim/wheel/command",
            RawReceivedFrame(
                payload=payload,
                remote_publisher_entity_id=1,
                remote_publisher_process_id=2,
                remote_publisher_host_name="fixture",
                remote_type_name="slope_sim.interfaces.v2.WheelCommand",
                remote_encoding="proto",
                remote_descriptor=descriptor.serialized_file_descriptor_set,
                send_timestamp_us=1,
                send_clock=1,
                received_at=1.0,
            ),
        )
        deadline = time.monotonic() + 1.0
        while accepted != [(payload, 1.0)] and time.monotonic() < deadline:
            time.sleep(0.01)
        assert accepted == [(payload, 1.0)]
        assert transport.snapshot().received_count == 1
    finally:
        transport.close()
