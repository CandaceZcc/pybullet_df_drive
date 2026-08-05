#!/usr/bin/env python3
# 阶段四 eCAL wheel materializer：从 canonical cache 复制到本轮私有 wheel cache。
from __future__ import annotations

import argparse
from collections.abc import Sequence
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat


def _digest(path: Path) -> tuple[int, str]:
    """流式复算 wheel 的 size/SHA-256，复制前后均使用相同规则。"""
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            size += len(chunk)
            digest.update(chunk)
    return size, digest.hexdigest()


def _load_wheel(manifest: Path) -> dict[str, object]:
    """读取最小 wheel identity，拒绝路径猜测或宽松的哈希字段。"""
    document = json.loads(manifest.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("wheel cache manifest is invalid")
    wheel = document.get("wheel")
    if document.get("schema_version") != 1 or not isinstance(wheel, dict):
        raise ValueError("wheel cache manifest is invalid")
    filename = wheel.get("filename")
    sha256 = wheel.get("sha256")
    relative_path = wheel.get("relative_path")
    size = wheel.get("size")
    if not isinstance(filename, str) or not filename or Path(filename).name != filename:
        raise ValueError("wheel filename must be a basename")
    if not isinstance(sha256, str) or len(sha256) != 64 or any(
        character not in "0123456789abcdef" for character in sha256
    ):
        raise ValueError("wheel SHA-256 is invalid")
    if relative_path != f"wheels/{sha256}/{filename}":
        raise ValueError("wheel relative path is not canonical")
    relative = PurePosixPath(relative_path)
    if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
        raise ValueError("wheel relative path is unsafe")
    if not isinstance(size, int) or isinstance(size, bool) or size < 1:
        raise ValueError("wheel size is invalid")
    return {"filename": filename, "sha256": sha256, "relative_path": relative_path, "size": size}


def materialize(manifest: Path, canonical_cache: Path, destination: Path, evidence: Path) -> None:
    """exclusive-copy 唯一 eCAL wheel，并保存可供 builder 审计的私有路径证据。"""
    wheel = _load_wheel(manifest)
    if destination.exists() or evidence.exists() or not evidence.parent.is_dir():
        raise ValueError("wheel materializer outputs must be absent under existing directories")
    if destination == evidence or destination in evidence.parents or evidence in destination.parents:
        raise ValueError("wheel materializer outputs must not overlap")
    source = canonical_cache.resolve() / str(wheel["relative_path"])
    source_stat = source.lstat()
    if (
        not stat.S_ISREG(source_stat.st_mode)
        or stat.S_ISLNK(source_stat.st_mode)
        or source_stat.st_nlink != 1
    ):
        raise ValueError("canonical wheel must be a singly linked regular file")
    if _digest(source) != (wheel["size"], wheel["sha256"]):
        raise ValueError("canonical wheel digest differs from manifest")
    destination.mkdir(parents=True, mode=0o755)
    copied = destination / str(wheel["filename"])
    with source.open("rb") as input_stream, copied.open("xb") as output_stream:
        while chunk := input_stream.read(1024 * 1024):
            output_stream.write(chunk)
    copied.chmod(0o644)
    if _digest(copied) != (wheel["size"], wheel["sha256"]) or copied.stat().st_nlink != 1:
        raise ValueError("private wheel digest differs after copy")
    evidence.write_text(
        json.dumps(
            {
                "path": str(copied.resolve()),
                "sha256": wheel["sha256"],
                "size": wheel["size"],
            },
            ensure_ascii=True,
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    evidence.chmod(0o644)


def main(argv: Sequence[str] | None = None) -> int:
    """创建每轮独占的 eCAL wheel 副本，不运行 pip 或网络请求。"""
    parser = argparse.ArgumentParser(description="Materialize a private Stage 4 eCAL wheel cache.")
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
    print("PASS: private eCAL wheel materialized")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
