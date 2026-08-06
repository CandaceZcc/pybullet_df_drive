#!/usr/bin/env python3
# 阶段四 Python runtime 路径隔离验证：拒绝构建输入绝对路径残留在交付树中。
from __future__ import annotations

import argparse
import codecs
import json
import os
from pathlib import Path
import stat


def _member_kind(path: Path, relative: Path, is_text: bool) -> str:
    """按计划要求区分 conda、wheel metadata、文本和二进制泄漏来源。"""
    if path.name == "conda-unpack":
        return "conda-unpack"
    if any(part.endswith(".dist-info") for part in relative.parts):
        return "wheel-metadata"
    return "text" if is_text else "binary"


def _scan_file(path: Path, relative: Path, prefixes: tuple[bytes, ...]) -> None:
    """流式扫描单个常规文件，同时判定其 UTF-8 文本或二进制属性。"""
    decoder = codecs.getincrementaldecoder("utf-8")("strict")
    is_text = True
    longest_prefix = max(len(prefix) for prefix in prefixes)
    previous = b""
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            if b"\x00" in chunk:
                is_text = False
            try:
                decoder.decode(chunk)
            except UnicodeDecodeError:
                is_text = False
            payload = previous + chunk
            if any(prefix in payload for prefix in prefixes):
                kind = _member_kind(path, relative, is_text)
                raise ValueError(f"{kind} contains forbidden absolute prefix: {relative.as_posix()}")
            previous = payload[-(longest_prefix - 1) :] if longest_prefix > 1 else b""
    if is_text:
        try:
            decoder.decode(b"", final=True)
        except UnicodeDecodeError:
            is_text = False


def _verify_relative_link(path: Path, runtime_root: Path, relative: Path) -> None:
    """只接受解析后仍留在 runtime 内的相对链接，阻断打包树路径逃逸。"""
    target = Path(os.readlink(path))
    if target.is_absolute():
        raise ValueError(f"symbolic link escapes runtime root: {relative.as_posix()}")
    try:
        resolved = (path.parent / target).resolve(strict=True)
        resolved.relative_to(runtime_root.resolve())
    except (OSError, ValueError) as error:
        raise ValueError(f"symbolic link escapes runtime root: {relative.as_posix()}") from error
    target_metadata = resolved.lstat()
    if not (stat.S_ISDIR(target_metadata.st_mode) or stat.S_ISREG(target_metadata.st_mode)):
        raise ValueError(f"symbolic link target is unsafe: {relative.as_posix()}")


def verify(runtime_root: Path, forbidden_prefixes: tuple[Path, ...]) -> dict[str, int]:
    """扫描 staging tree 的文件名与内容，证明没有构建绝对路径泄漏。"""
    root_metadata = runtime_root.lstat()
    if not stat.S_ISDIR(root_metadata.st_mode) or stat.S_ISLNK(root_metadata.st_mode):
        raise ValueError("runtime root must be a real directory")
    if not forbidden_prefixes:
        raise ValueError("at least one forbidden prefix is required")
    prefixes = tuple(str(prefix).encode("utf-8") for prefix in forbidden_prefixes)
    counts = {"directories": 0, "files": 0, "links": 0, "filename_checks": 0}
    for path in sorted(runtime_root.rglob("*"), key=lambda candidate: candidate.as_posix()):
        relative = path.relative_to(runtime_root)
        metadata = path.lstat()
        if stat.S_ISLNK(metadata.st_mode):
            _verify_relative_link(path, runtime_root, relative)
            counts["links"] += 1
        elif stat.S_ISDIR(metadata.st_mode):
            counts["directories"] += 1
        elif stat.S_ISREG(metadata.st_mode):
            counts["files"] += 1
            _scan_file(path, relative, prefixes)
        else:
            raise ValueError(f"runtime tree contains unsafe member: {relative.as_posix()}")
        # 只扫描交付树内的规范相对文件名，避免把 runtime 自身的工作根误判为泄漏。
        filename = relative.as_posix().encode("utf-8")
        if any(prefix in filename for prefix in prefixes):
            raise ValueError(f"filename contains forbidden absolute prefix: {relative.as_posix()}")
        counts["filename_checks"] += 1
    return counts


def main() -> int:
    """提供 fail-closed CLI，并按需写入可审计的扫描统计证据。"""
    parser = argparse.ArgumentParser(description="Verify Stage 4 Python runtime path isolation.")
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--forbidden-prefix", type=Path, action="append", required=True)
    parser.add_argument("--evidence", type=Path)
    args = parser.parse_args()
    try:
        prefixes = tuple(
            prefix.resolve(strict=False) for prefix in args.forbidden_prefix
        )
        if any(not prefix.is_absolute() for prefix in prefixes):
            raise ValueError("forbidden prefix must be absolute")
        counts = verify(args.runtime_root.absolute(), prefixes)
        if args.evidence is not None:
            args.evidence.write_text(
                json.dumps({"runtime_root": str(args.runtime_root.absolute()), **counts}, sort_keys=True)
                + "\n",
                encoding="utf-8",
            )
    except (OSError, ValueError) as error:
        print(f"FAIL: {error}")
        return 1
    print("PASS: Python runtime contains no forbidden build prefixes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
