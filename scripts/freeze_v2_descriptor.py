"""v2 descriptor 冻结工具：首次创建，后续只校验并拒绝覆盖。"""
from __future__ import annotations

from argparse import ArgumentParser
from hashlib import sha256
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DESCRIPTOR = ROOT / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
DEFAULT_MANIFEST = ROOT / "proto/slope_sim_interfaces_v2.sha256"


def freeze(descriptor: Path, manifest: Path, *, create: bool) -> str:
    """计算 descriptor 摘要，首次独占创建或严格验证既有冻结值。"""
    digest = sha256(descriptor.read_bytes()).hexdigest()
    if create:
        with manifest.open("x", encoding="ascii", newline="\n") as stream:
            stream.write(digest + "\n")
        return digest
    expected = manifest.read_text(encoding="ascii").strip()
    if expected != digest:
        raise RuntimeError(f"descriptor SHA-256 mismatch: {digest} != {expected}")
    return digest


def main() -> int:
    """解析稳定 CLI；无参数只校验，`--create` 只允许首次冻结。"""
    parser = ArgumentParser(description="Freeze or verify the Stage 4 v2 descriptor.")
    parser.add_argument("--create", action="store_true")
    args = parser.parse_args()
    print(freeze(DEFAULT_DESCRIPTOR, DEFAULT_MANIFEST, create=args.create))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
