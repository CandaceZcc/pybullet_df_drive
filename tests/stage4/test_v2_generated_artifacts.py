"""阶段四 A：独立验证 v2 生成模块和 FileDescriptorSet。"""
import os
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCHEMA = ROOT / "proto" / "slope_sim_interfaces_v2.proto"
GENERATOR = ROOT / "scripts" / "generate_v2_protos.py"
MODULE = ROOT / "slope_sim/interfaces/generated/slope_sim_interfaces_v2_pb2.py"
DESCRIPTOR = ROOT / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"


@pytest.mark.stage4_artifact
def test_v2_generated_artifacts_are_reproducible_with_frozen_protoc(tmp_path) -> None:
    """独立 protoc 33.6 重生成的 descriptor 必须逐 byte 等于跟踪产物。"""
    required = (SCHEMA, GENERATOR, MODULE, DESCRIPTOR)
    if not all(path.is_file() for path in required):
        pytest.fail("v2 generated artifacts are not implemented", pytrace=False)
    raw_protoc = os.environ.get("STAGE4_PROTOC")
    if not raw_protoc:
        pytest.fail("STAGE4_PROTOC is not configured", pytrace=False)
    protoc = Path(raw_protoc)
    assert subprocess.run([str(protoc), "--version"], check=True, capture_output=True, text=True).stdout.strip() == "libprotoc 33.6"
    generated = tmp_path / "generated"
    generated.mkdir()
    descriptor = generated / "v2.desc"
    subprocess.run(
        [str(protoc), f"--proto_path={SCHEMA.parent}", f"--python_out={generated}", f"--descriptor_set_out={descriptor}", "--include_imports", str(SCHEMA)],
        check=True,
    )
    assert descriptor.read_bytes() == DESCRIPTOR.read_bytes()
    assert (generated / "slope_sim_interfaces_v2_pb2.py").read_bytes() == MODULE.read_bytes()
