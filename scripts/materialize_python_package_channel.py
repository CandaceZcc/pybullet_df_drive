#!/usr/bin/env python3
# 阶段四 Conda channel materializer：从 canonical package cache 生成本轮私有 file channel。
from __future__ import annotations

import argparse
import json
from pathlib import Path
import stat
from collections.abc import Sequence
from urllib.parse import urlsplit

from materialize_python_package_cache import _digest, _parse_archives


def materialize(manifest: Path, canonical_cache: Path, destination: Path, evidence: Path) -> None:
    """按冻结 URL 的子目录复制 archive，供离线 solver 读取本轮 repodata。"""
    records = _parse_archives(manifest)
    if destination.exists() or evidence.exists() or not evidence.parent.is_dir():
        raise ValueError("package channel outputs must be absent under existing directories")
    if destination == evidence or destination in evidence.parents or evidence in destination.parents:
        raise ValueError("package channel outputs must not overlap")
    canonical_root = canonical_cache.resolve()
    if not canonical_root.is_dir():
        raise ValueError("canonical package cache must be an existing directory")

    destination.mkdir(mode=0o755)
    copied: list[str] = []
    for record in records:
        source = canonical_root / str(record["relative_path"])
        source_stat = source.lstat()
        expected = (record["size"], record["md5"], record["sha256"])
        if (
            not stat.S_ISREG(source_stat.st_mode)
            or stat.S_ISLNK(source_stat.st_mode)
            or source_stat.st_nlink != 1
            or _digest(source) != expected
        ):
            raise ValueError("canonical archive differs from package cache manifest")
        subdir = Path(urlsplit(str(record["url"])).path).parent.name
        if subdir not in {"linux-64", "noarch"}:
            raise ValueError("package cache URL must end in a Conda subdir")
        target_directory = destination / subdir
        target_directory.mkdir(mode=0o755, exist_ok=True)
        target = target_directory / str(record["filename"])
        with source.open("rb") as input_stream, target.open("xb") as output_stream:
            while chunk := input_stream.read(1024 * 1024):
                output_stream.write(chunk)
        target.chmod(0o644)
        if target.stat().st_nlink != 1 or _digest(target) != expected:
            raise ValueError("private package channel archive differs after copy")
        copied.append(str(target.relative_to(destination)))
    evidence.write_text(
        json.dumps({"archives": copied, "channel": str(destination.resolve())}, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    evidence.chmod(0o644)


def main(argv: Sequence[str] | None = None) -> int:
    """物化一次全新的 file channel；索引由调用方的固定 Conda toolchain 创建。"""
    parser = argparse.ArgumentParser(description="Materialize a private Stage 4 Conda file channel.")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--canonical-cache", type=Path, required=True)
    parser.add_argument("--destination", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        materialize(
            args.manifest.resolve(),
            args.canonical_cache.resolve(),
            args.destination.resolve(),
            args.evidence.resolve(),
        )
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}")
        return 1
    print("PASS: private Conda file channel materialized")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
