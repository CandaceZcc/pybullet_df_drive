"""阶段四 raw eCAL 边界：原字节收发、SHA-256 与远端类型元数据验证。"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from hashlib import sha256
from importlib import import_module
import time

from slope_sim.interfaces.v2.descriptor import DescriptorIdentity


@dataclass(frozen=True)
class RawReceivedFrame:
    """从 native callback 复制出的单帧 owned 数据，不在此层解析 payload。"""

    payload: bytes
    remote_publisher_entity_id: int
    remote_publisher_process_id: int
    remote_publisher_host_name: str
    remote_type_name: str
    remote_encoding: str
    remote_descriptor: bytes
    send_timestamp_us: int
    send_clock: int
    received_at: float


@dataclass(frozen=True)
class RemoteTypeMetadata:
    """本帧 eCAL type information 的 owned 复制。"""

    name: str
    encoding: str
    descriptor: bytes

    def __post_init__(self) -> None:
        if not isinstance(self.name, str):
            raise ValueError("remote type name must be a string")
        if not isinstance(self.encoding, str):
            raise ValueError("remote encoding must be a string")
        if not isinstance(self.descriptor, (bytes, bytearray, memoryview)):
            raise ValueError("remote descriptor must be bytes-like")
        object.__setattr__(self, "descriptor", bytes(self.descriptor))


@dataclass(frozen=True)
class ProcessedRawFrame:
    """worker 完成 hash、远端 metadata gate 和解析后的结果。"""

    envelope: RawReceivedFrame
    payload_sha256: bytes
    parsed: object


class ProtocolVerificationState(Enum):
    """monitoring 与 discovery count 对齐后的远端协议状态。"""

    WAITING = "waiting"
    PENDING = "pending"
    VERIFIED = "verified"
    CONFLICT = "conflict"


@dataclass(frozen=True)
class ProtocolVerification:
    """指定 topic 的全部远端 endpoint metadata 判定。"""

    state: ProtocolVerificationState
    peer_count: int
    type_names: tuple[str, ...]
    encodings: tuple[str, ...]
    descriptor_sha256: tuple[str, ...]
    detail: str = ""


@dataclass(frozen=True)
class RemoteEndpointSnapshot:
    """为后续 transport 质量快照提供单次原子验证结果。"""

    verification: ProtocolVerification


class EcalRawBindings:
    """仅封装 eCAL 6.1.1 raw Python surface，不承担业务解析或命令权。"""

    def __init__(
        self,
        core: object | None = None,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if not callable(monotonic):
            raise ValueError("monotonic must be callable")
        self._core = import_module("ecal.nanobind_core") if core is None else core
        self._monotonic = monotonic
        # 保持 native callback 与 lane callback 强引用，防止 nanobind 回调失效。
        self._subscriber_callbacks: list[tuple[object, Callable[..., None], Callable[[RawReceivedFrame], None]]] = []

    def _type_info(self, type_name: str, descriptor: DescriptorIdentity) -> object:
        """创建 raw publisher/subscriber 共享的 eCAL wire metadata。"""
        return self._core.DataTypeInformation(
            name=type_name,
            encoding="proto",
            descriptor=descriptor.serialized_file_descriptor_set,
        )

    def create_publisher(
        self,
        topic: str,
        type_name: str,
        descriptor: DescriptorIdentity,
    ) -> object:
        """创建 raw publisher；发送端只接受已生成的 bytes。"""
        return self._core.Publisher(topic, self._type_info(type_name, descriptor))

    def create_subscriber(
        self,
        topic: str,
        type_name: str,
        descriptor: DescriptorIdentity,
        callback: Callable[[RawReceivedFrame], None],
    ) -> object:
        """注册三参数 raw callback；native 栈只复制 owned envelope。"""
        if not callable(callback):
            raise ValueError("callback must be callable")
        subscriber = self._core.Subscriber(topic, self._type_info(type_name, descriptor))

        def receive(publisher_id: object, data_type_info: object, data: object) -> None:
            # raw callback 禁止 hash、monitoring、Protobuf parse 或领域校验。
            entity_id = publisher_id.topic_id
            metadata = RemoteTypeMetadata(
                _metadata_string(data_type_info.name),
                _metadata_string(data_type_info.encoding),
                _metadata_bytes(data_type_info.descriptor),
            )
            frame = RawReceivedFrame(
                payload=bytes(data.buffer),
                remote_publisher_entity_id=int(entity_id.entity_id),
                remote_publisher_process_id=int(entity_id.process_id),
                remote_publisher_host_name=str(entity_id.host_name),
                remote_type_name=metadata.name,
                remote_encoding=metadata.encoding,
                remote_descriptor=metadata.descriptor,
                send_timestamp_us=int(data.send_timestamp),
                send_clock=int(data.send_clock),
                received_at=float(self._monotonic()),
            )
            callback(frame)

        set_callback = getattr(subscriber, "set_receive_callback", None)
        if not callable(set_callback):
            raise RuntimeError("eCAL raw Subscriber.set_receive_callback is unavailable")
        set_callback(receive)
        self._subscriber_callbacks.append((subscriber, receive, callback))
        return subscriber

    @staticmethod
    def send(publisher: object, payload: object) -> None:
        """直接发送 owned raw bytes，禁止在 raw boundary 做 typed 重序列化。"""
        if not isinstance(payload, (bytes, bytearray, memoryview)):
            raise ValueError("payload must be bytes-like")
        send = getattr(publisher, "send", None)
        if not callable(send):
            raise RuntimeError("eCAL raw Publisher.send is unavailable")
        result = send(bytes(payload))
        if result is False:
            raise RuntimeError("eCAL raw Publisher.send returned False")

    def _remote_topics(self, remote_direction: str) -> tuple[object, ...]:
        """读取 monitoring 的指定远端方向，不从本地声明推断对端类型。"""
        monitoring = self._core.monitoring.get_monitoring()
        if remote_direction == "publisher":
            return tuple(monitoring.publishers)
        if remote_direction == "subscriber":
            return tuple(monitoring.subscribers)
        raise ValueError("remote_direction must be publisher or subscriber")

    def snapshot_remote_endpoints(
        self,
        *,
        topic: str,
        remote_direction: str,
        peer_count: int,
        expected_type: str,
        descriptor: DescriptorIdentity,
    ) -> RemoteEndpointSnapshot:
        """用 exact discovery count 与远端 monitoring metadata 建立协议 gate。"""
        if isinstance(peer_count, bool) or not isinstance(peer_count, int) or peer_count < 0:
            raise ValueError("peer_count must be a nonnegative integer")
        endpoints = tuple(
            endpoint
            for endpoint in self._remote_topics(remote_direction)
            if endpoint.topic_name == topic
        )
        if peer_count == 0:
            return RemoteEndpointSnapshot(
                ProtocolVerification(ProtocolVerificationState.WAITING, 0, (), (), ())
            )

        names = tuple(_metadata_string(item.datatype_information.name) for item in endpoints)
        encodings = tuple(_metadata_string(item.datatype_information.encoding) for item in endpoints)
        descriptors = tuple(_metadata_bytes(item.datatype_information.descriptor) for item in endpoints)
        descriptor_digests = tuple(sha256(raw).hexdigest() for raw in descriptors)
        if len(endpoints) != peer_count:
            return RemoteEndpointSnapshot(
                ProtocolVerification(
                    ProtocolVerificationState.PENDING,
                    peer_count,
                    names,
                    encodings,
                    descriptor_digests,
                    "discovery count and monitoring metadata are not yet aligned",
                )
            )
        if any(not name or not encoding or not raw for name, encoding, raw in zip(
            names, encodings, descriptors, strict=True,
        )):
            return RemoteEndpointSnapshot(
                ProtocolVerification(
                    ProtocolVerificationState.PENDING,
                    peer_count,
                    names,
                    encodings,
                    descriptor_digests,
                    "remote metadata is not yet populated",
                )
            )
        valid = all(
            name == expected_type
            and encoding == "proto"
            and raw == descriptor.serialized_file_descriptor_set
            for name, encoding, raw in zip(names, encodings, descriptors, strict=True)
        )
        return RemoteEndpointSnapshot(
            ProtocolVerification(
                ProtocolVerificationState.VERIFIED if valid else ProtocolVerificationState.CONFLICT,
                peer_count,
                names,
                encodings,
                descriptor_digests,
                "" if valid else "same-topic remote type/encoding/descriptor mismatch",
            )
        )


def _sha256_digest(payload: bytes) -> bytes:
    """默认 worker hash 函数，便于测试显式验证先后顺序。"""
    return sha256(payload).digest()


def _metadata_string(value: object) -> str:
    """保留 native 启动期 None 为缺失字段，交由 worker 的 pending gate 处理。"""
    return "" if value is None else str(value)


def _metadata_bytes(value: object) -> bytes:
    """保留 native 启动期 None 为缺失 descriptor。"""
    return b"" if value is None else bytes(value)


def classify_raw_frame_metadata(
    frame: RawReceivedFrame,
    *,
    expected_type: str,
    descriptor: DescriptorIdentity,
) -> ProtocolVerificationState:
    """仅完整的 wire metadata 才能判定协议通过或冲突。"""
    if not isinstance(frame, RawReceivedFrame):
        raise ValueError("frame must be a RawReceivedFrame")
    if not frame.remote_type_name or not frame.remote_encoding or not frame.remote_descriptor:
        return ProtocolVerificationState.PENDING
    if (
        frame.remote_type_name == expected_type
        and frame.remote_encoding == "proto"
        and frame.remote_descriptor == descriptor.serialized_file_descriptor_set
    ):
        return ProtocolVerificationState.VERIFIED
    return ProtocolVerificationState.CONFLICT


def validate_raw_frame_metadata(
    frame: RawReceivedFrame,
    *,
    expected_type: str,
    descriptor: DescriptorIdentity,
    payload_hasher: Callable[[bytes], bytes] = _sha256_digest,
) -> bytes:
    """先计算 payload 摘要并验证 callback 自带的远端 metadata。"""
    if not isinstance(frame, RawReceivedFrame):
        raise ValueError("frame must be a RawReceivedFrame")
    if not callable(payload_hasher):
        raise ValueError("payload_hasher must be callable")
    payload_sha256 = bytes(payload_hasher(frame.payload))
    if len(payload_sha256) != 32:
        raise RuntimeError("payload hasher must return exactly 32 bytes")
    if (
        frame.remote_type_name != expected_type
        or frame.remote_encoding != "proto"
        or frame.remote_descriptor != descriptor.serialized_file_descriptor_set
    ):
        raise ValueError("remote type/encoding/descriptor mismatch")
    return payload_sha256


def process_raw_frame(
    frame: RawReceivedFrame,
    *,
    expected_type: str,
    descriptor: DescriptorIdentity,
    parser: Callable[[bytes], object],
    payload_hasher: Callable[[bytes], bytes] = _sha256_digest,
) -> ProcessedRawFrame:
    """worker 中先 hash、再验本帧 metadata，最后才解析 payload。"""
    if not callable(parser) or not callable(payload_hasher):
        raise ValueError("parser and payload_hasher must be callable")
    payload_sha256 = validate_raw_frame_metadata(
        frame,
        expected_type=expected_type,
        descriptor=descriptor,
        payload_hasher=payload_hasher,
    )
    return ProcessedRawFrame(frame, payload_sha256, parser(frame.payload))
