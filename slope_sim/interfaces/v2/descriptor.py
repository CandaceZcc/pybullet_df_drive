"""阶段四 descriptor 身份：集中校验 FileDescriptorSet 与冻结 SHA-256。"""
from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path


_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_DESCRIPTOR = _ROOT / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
_DEFAULT_MANIFEST = _ROOT / "proto/slope_sim_interfaces_v2.sha256"


@dataclass(frozen=True)
class DescriptorIdentity:
    """原始 FileDescriptorSet 及其二进制 SHA-256 身份。"""

    serialized_file_descriptor_set: bytes
    sha256: bytes


def load_v2_descriptor(
    descriptor_path: Path = _DEFAULT_DESCRIPTOR,
    manifest_path: Path = _DEFAULT_MANIFEST,
) -> DescriptorIdentity:
    """加载并校验冻结 descriptor；不一致时禁止启动 v2。"""
    payload = descriptor_path.read_bytes()
    digest = sha256(payload).digest()
    expected = bytes.fromhex(manifest_path.read_text(encoding="ascii").strip())
    if len(expected) != 32 or digest != expected:
        raise RuntimeError("v2 descriptor SHA-256 mismatch")
    return DescriptorIdentity(payload, digest)
