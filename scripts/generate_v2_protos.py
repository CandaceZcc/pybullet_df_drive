"""阶段四 v2 协议生成器：只使用 Task 2 冻结的 protoc 33.6。"""
from __future__ import annotations

import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
PROTO_DIR = ROOT / "proto"
OUTPUT_DIR = ROOT / "slope_sim" / "interfaces" / "generated"
V2_PROTO = PROTO_DIR / "slope_sim_interfaces_v2.proto"
V2_DESCRIPTOR_SET = OUTPUT_DIR / "slope_sim_interfaces_v2.desc"


def _stage4_protoc() -> Path:
    """读取且核验环境合同导出的唯一 protoc 33.6 可执行文件。"""
    raw = os.environ.get("STAGE4_PROTOC")
    if not raw:
        raise RuntimeError("STAGE4_PROTOC must point to the frozen protoc 33.6")
    executable = Path(raw).resolve(strict=True)
    version = subprocess.run(
        [str(executable), "--version"], check=True, capture_output=True, text=True
    ).stdout.strip()
    if version != "libprotoc 33.6":
        raise RuntimeError(f"expected libprotoc 33.6, got {version!r}")
    return executable


def _generate_v2(protoc: Path) -> None:
    """使用冻结编译器同时生成 Python 类型和原始 FileDescriptorSet。"""
    subprocess.run(
        [
            str(protoc), f"--proto_path={PROTO_DIR}", f"--python_out={OUTPUT_DIR}",
            f"--descriptor_set_out={V2_DESCRIPTOR_SET}", "--include_imports", str(V2_PROTO),
        ],
        check=True,
    )


def main() -> int:
    """生成 v2，保持历史 v1/internal 的 grpc_tools 流程不变。"""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    _generate_v2(_stage4_protoc())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
