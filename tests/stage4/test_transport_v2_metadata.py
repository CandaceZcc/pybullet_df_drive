"""阶段四 A：验证 transport 保留精确 peer count 和远端协议元数据。"""
from dataclasses import fields

import pytest

from slope_sim.interfaces.ecal_transport import EcalBindings, _ProtoResource
from slope_sim.interfaces.transport import TransportTopicQuality


class _RawSubscriber:
    """只实现 discovery count 的最小原始订阅资源。"""

    def __init__(self, publisher_count: object) -> None:
        self.publisher_count = publisher_count

    def get_publisher_count(self) -> object:
        return self.publisher_count


def test_topic_quality_preserves_exact_peer_count_and_conflict_metadata() -> None:
    """两个同名 peer 必须保留为 2，不能被压缩为一个布尔值。"""
    field_names = {field.name for field in fields(TransportTopicQuality)}
    assert {
        "peer_count",
        "protocol_state",
        "protocol_detail",
        "remote_type_names",
        "remote_encodings",
        "remote_descriptor_sha256",
    } <= field_names, "exact peer/protocol quality behavior is not implemented"
    quality = TransportTopicQuality(
        topic="/sim/wheel/command",
        peer_connected=True,
        peer_count=2,
        protocol_state="conflict",
        protocol_detail="unexpected slope_sim.interfaces.v1.WheelCommand",
        remote_type_names=(
            "slope_sim.interfaces.v1.WheelCommand",
            "slope_sim.interfaces.v2.WheelCommand",
        ),
        remote_encodings=("proto", "proto"),
        remote_descriptor_sha256=("11" * 32, "22" * 32),
    )
    assert quality.peer_count == 2
    assert quality.protocol_state == "conflict"


@pytest.mark.parametrize(
    "kwargs",
    (
        {"peer_connected": True, "peer_count": 0},
        {"peer_connected": False, "peer_count": 1},
        {"peer_connected": True, "peer_count": True},
        {"peer_connected": True, "peer_count": -1},
        {"peer_connected": True, "peer_count": 1, "protocol_state": "verified"},
        {"peer_connected": True, "peer_count": 1, "protocol_state": "conflict", "protocol_detail": ""},
        {
            "peer_connected": True,
            "peer_count": 1,
            "protocol_state": "verified",
            "remote_type_names": ("Type",),
            "remote_encodings": ("proto",),
            "remote_descriptor_sha256": ("not-a-digest",),
        },
    ),
)
def test_topic_quality_rejects_inconsistent_peer_and_protocol_metadata(kwargs) -> None:
    """质量快照必须拒绝不完整、矛盾或伪造的协议发现数据。"""
    with pytest.raises(ValueError):
        TransportTopicQuality("/sim/wheel/command", **kwargs)


def test_unchecked_local_quality_has_no_peer_count_or_remote_metadata() -> None:
    """本地 transport 不声称执行过 eCAL discovery 或协议验证。"""
    quality = TransportTopicQuality("/local")
    assert quality.peer_connected is None
    assert quality.peer_count is None
    assert quality.protocol_state == "not_checked"
    assert quality.remote_type_names == ()
    assert quality.remote_encodings == ()
    assert quality.remote_descriptor_sha256 == ()


def test_ecal_binding_exposes_exact_count_while_legacy_boolean_remains_derived() -> None:
    """eCAL wrapper 返回原始 count，旧 API 只作为兼容派生值。"""
    resource = _ProtoResource(_RawSubscriber(3), direction="subscriber")
    assert callable(getattr(EcalBindings, "peer_count", None)), (
        "exact eCAL peer_count behavior is not implemented"
    )
    assert EcalBindings.peer_count(resource) == 3
    assert EcalBindings.is_peer_connected(resource) is True


@pytest.mark.parametrize("count", (True, -1, 1.5))
def test_ecal_binding_rejects_invalid_native_count(count) -> None:
    """native discovery 的 bool、负数或浮点返回必须 fail closed。"""
    resource = _ProtoResource(_RawSubscriber(count), direction="subscriber")
    with pytest.raises(RuntimeError, match="peer count"):
        EcalBindings.peer_count(resource)
