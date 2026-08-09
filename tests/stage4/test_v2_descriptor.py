"""阶段四 A：冻结并验证 v2 FileDescriptorSet 的唯一身份。"""
from hashlib import sha256
from importlib import import_module

import pytest


def require_wished_module(name: str):
    """让尚未实现的 descriptor loader 形成明确 RED。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


def test_descriptor_identity_matches_frozen_manifest() -> None:
    """默认运行时入口必须返回冻结 descriptor bytes 与 32-byte SHA-256。"""
    module = require_wished_module("slope_sim.interfaces.v2.descriptor")
    identity = module.load_v2_descriptor()
    assert len(identity.serialized_file_descriptor_set) > 0
    assert len(identity.sha256) == 32
    assert identity.sha256 == sha256(identity.serialized_file_descriptor_set).digest()


def test_descriptor_loader_rejects_manifest_mismatch(tmp_path) -> None:
    """descriptor 内容或 manifest 被替换后，v2 启动入口必须 fail closed。"""
    module = require_wished_module("slope_sim.interfaces.v2.descriptor")
    descriptor = tmp_path / "v2.desc"
    manifest = tmp_path / "v2.sha256"
    descriptor.write_bytes(b"descriptor")
    manifest.write_text("00" * 32 + "\n", encoding="ascii")
    with pytest.raises(RuntimeError, match="descriptor SHA-256 mismatch"):
        module.load_v2_descriptor(descriptor, manifest)
