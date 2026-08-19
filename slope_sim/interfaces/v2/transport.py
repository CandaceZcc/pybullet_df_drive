"""阶段四 v2 transport factory：通过既有有界 lane 使用 eCAL raw bytes。"""
from __future__ import annotations

from dataclasses import dataclass

from slope_sim.interfaces.ecal_transport import EcalTransport, _ChannelBinding
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings
from slope_sim.interfaces.v2.topics import V2_TOPICS


@dataclass
class _RawResource:
    """为 raw eCAL 资源保留方向，供统一 discovery 与关闭路径使用。"""

    raw: object
    direction: str
    closed: bool = False


class _RawV2Bindings:
    """将 Task 8 raw API 适配为既有 EcalTransport 资源端口。"""

    def __init__(self) -> None:
        self._raw = EcalRawBindings()
        self._finalized = False

    def create_participant(self, name: str) -> _RawResource:
        """以包含 monitoring 的完整组件集初始化本轮 v2 participant。"""
        initialize = getattr(self._raw._core, "initialize", None)
        if not callable(initialize) or initialize(name, 0x3F) is False:
            raise RuntimeError("eCAL raw core.initialize returned False")
        return _RawResource(self._raw._core, "participant")

    def create_raw_subscriber(
        self,
        topic: str,
        type_name: str,
        descriptor: DescriptorIdentity,
        callback,
    ) -> _RawResource:
        """注册只复制 owned frame 的 native raw callback。"""
        return _RawResource(
            self._raw.create_subscriber(topic, type_name, descriptor, callback),
            "subscriber",
        )

    def create_raw_publisher(
        self,
        topic: str,
        type_name: str,
        descriptor: DescriptorIdentity,
    ) -> _RawResource:
        """创建只接受预编码 bytes 的 raw publisher。"""
        return _RawResource(
            self._raw.create_publisher(topic, type_name, descriptor),
            "publisher",
        )

    def send_raw(self, publisher: _RawResource, payload: bytes) -> None:
        """不解析 payload，直接调用 Task 8 raw sender。"""
        self._raw.send(publisher.raw, payload)

    @staticmethod
    def peer_count(resource: _RawResource) -> int:
        """保留 discovery 的精确 raw publisher/subscriber count。"""
        method_name = (
            "get_publisher_count"
            if resource.direction == "subscriber"
            else "get_subscriber_count"
        )
        method = getattr(resource.raw, method_name, None)
        count = method() if callable(method) else None
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise RuntimeError("eCAL raw peer count must be a nonnegative integer")
        return count

    def snapshot_remote_endpoints(
        self,
        channel: object,
        resource: _RawResource,
        peer_count: int,
    ) -> object:
        """将 raw monitoring 快照按本地 channel 的相反方向传给 transport。"""
        remote_direction = (
            "publisher" if channel.direction == "subscribe" else "subscriber"
        )
        return self._raw.snapshot_remote_endpoints(
            topic=channel.topic,
            remote_direction=remote_direction,
            peer_count=peer_count,
            expected_type=channel.type_name,
            descriptor=channel.descriptor,
        )

    def close(self, resource: _RawResource) -> None:
        """先撤 raw receive callback，最后才 finalize 进程级 eCAL core。"""
        if resource.closed:
            return
        resource.closed = True
        if resource.direction == "subscriber":
            remove_callback = getattr(resource.raw, "remove_receive_callback", None)
            if callable(remove_callback):
                remove_callback()
            return
        if resource.direction == "participant" and not self._finalized:
            self._finalized = True
            finalize = getattr(resource.raw, "finalize", None)
            if not callable(finalize) or finalize() is False:
                raise RuntimeError("eCAL raw core.finalize returned False")


def _v2_channel_bindings(descriptor: DescriptorIdentity) -> tuple[_ChannelBinding, ...]:
    """从唯一 v2 topic 合同构造 raw channel，禁止混入阶段三消息类。"""
    codec = V2ProtoCodec(descriptor)
    parsers = {
        "slope_sim.interfaces.v2.WheelCommand": codec.decode_wheel_command,
        "slope_sim.interfaces.v2.WheelState": codec.decode_wheel_state,
        "slope_sim.interfaces.v2.LidarPointCloud": codec.decode_lidar_point_cloud,
        "slope_sim.interfaces.v2.RtkState": codec.decode_rtk_state,
        "slope_sim.interfaces.v2.ImuAttitude": codec.decode_imu_attitude,
    }
    return tuple(
        _ChannelBinding(
            topic=contract.topic,
            direction=contract.direction,
            type_name=contract.type_name,
            descriptor=descriptor,
            raw_wire=True,
            raw_parser=parsers[contract.type_name],
        )
        for contract in V2_TOPICS
    )


def create_v2_ecal_transport(
    *,
    descriptor: DescriptorIdentity,
    queue_size: int = 32,
    participant_name: str = "slope-sim-v2",
    role: str = "simulation",
    bindings: object | None = None,
) -> EcalTransport:
    """创建五话题 raw v2 transport；正式 runtime 切换仍由阶段 B 负责。"""
    if not isinstance(descriptor, DescriptorIdentity):
        raise ValueError("descriptor must be a DescriptorIdentity")
    selected = _RawV2Bindings() if bindings is None else bindings
    return EcalTransport(
        bindings=selected,
        queue_size=queue_size,
        participant_name=participant_name,
        role=role,
        channel_bindings=_v2_channel_bindings(descriptor),
    )
