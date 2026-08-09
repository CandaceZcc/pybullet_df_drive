"""阶段四 A 入口：冻结阶段三 v1 协议源与生成 descriptor。"""
from hashlib import sha256
from pathlib import Path

from slope_sim.interfaces.generated import slope_sim_interfaces_pb2 as v1_pb


V1_SOURCE_SHA256 = "9de0e629a6494ea9446893043c7e30ca9d6370868f23def4fcd4f2af5cd102d4"
V1_DESCRIPTOR_SHA256 = "6a524cce7b11ca72f73214394097407c2f8ddc50ea40ca6ffef7be1c248dc2e9"


def test_v1_source_and_descriptor_are_frozen() -> None:
    """v2 开工前必须精确锁定 v1 源和生成 descriptor 的原始字节。"""
    source = Path("proto/slope_sim_interfaces.proto").read_bytes()
    manifest_path = Path("proto/slope_sim_interfaces_v1.sha256")
    assert manifest_path.is_file(), "v1 SHA-256 manifest is not implemented"
    manifest = manifest_path.read_text(encoding="ascii")
    assert sha256(source).hexdigest() == V1_SOURCE_SHA256
    assert sha256(v1_pb.DESCRIPTOR.serialized_pb).hexdigest() == V1_DESCRIPTOR_SHA256
    assert manifest == (
        f"source {V1_SOURCE_SHA256}\n"
        f"descriptor {V1_DESCRIPTOR_SHA256}\n"
    )
