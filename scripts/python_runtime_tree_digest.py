#!/usr/bin/env python3
# 阶段四 Python runtime 树摘要：为双根可复现性比较固定成员元数据和内容。
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat


def _sha256(path: Path) -> str:
    """流式计算常规文件摘要，避免运行或加载 runtime 内的内容。"""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _record(
    relative: str,
    metadata: os.stat_result,
    content: str | None,
    link_target: str | None = None,
) -> bytes:
    """编码单个成员的稳定摘要记录；目录、文件和链接有不同类型字段。"""
    mode = stat.S_IMODE(metadata.st_mode)
    member_type = "L" if link_target is not None else "F" if content is not None else "D"
    fields = [member_type, relative, f"{mode:04o}", str(metadata.st_mtime_ns)]
    if link_target is not None:
        fields.append(link_target)
    elif content is not None:
        fields.extend((str(metadata.st_size), content))
    return "\0".join(fields).encode("utf-8") + b"\n"


def _safe_link_target(path: Path, runtime_root: Path, relative: str) -> str:
    """验证 conda-pack 相对链接不逃离 runtime tree，并返回原始 target。"""
    target = os.readlink(path)
    if Path(target).is_absolute():
        raise ValueError(f"runtime symbolic link escapes root: {relative}")
    try:
        resolved = (path.parent / target).resolve(strict=True)
        resolved.relative_to(runtime_root.resolve())
    except (OSError, ValueError) as error:
        raise ValueError(f"runtime symbolic link escapes root: {relative}") from error
    target_metadata = resolved.lstat()
    if not (stat.S_ISDIR(target_metadata.st_mode) or stat.S_ISREG(target_metadata.st_mode)):
        raise ValueError(f"runtime symbolic link target is unsafe: {relative}")
    return target


def digest_tree(runtime_root: Path) -> dict[str, int | str]:
    """校验封装 tree 类型并返回包含 mode/mtime 的稳定摘要。"""
    root_metadata = runtime_root.lstat()
    if not stat.S_ISDIR(root_metadata.st_mode) or stat.S_ISLNK(root_metadata.st_mode):
        raise ValueError("runtime root must be a real directory")
    records = [(".", _record(".", root_metadata, None))]
    directories = 1
    files = 0
    links = 0
    regular_bytes = 0
    for path in sorted(runtime_root.rglob("*"), key=lambda candidate: candidate.as_posix()):
        relative = path.relative_to(runtime_root).as_posix()
        metadata = path.lstat()
        if stat.S_ISLNK(metadata.st_mode):
            target = _safe_link_target(path, runtime_root, relative)
            records.append((relative, _record(relative, metadata, None, target)))
            links += 1
            continue
        if stat.S_ISDIR(metadata.st_mode):
            records.append((relative, _record(relative, metadata, None)))
            directories += 1
            continue
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise ValueError(f"runtime tree contains unsafe member: {relative}")
        content = _sha256(path)
        records.append((relative, _record(relative, metadata, content)))
        files += 1
        regular_bytes += metadata.st_size
    digest = hashlib.sha256()
    for _relative, record in sorted(records):
        digest.update(record)
    return {
        "directories": directories,
        "files": files,
        "links": links,
        "regular_bytes": regular_bytes,
        "tree_sha256": digest.hexdigest(),
    }


def main() -> int:
    """输出可机器比较的 JSON；调用者负责把 stdout 写入本轮 evidence。"""
    parser = argparse.ArgumentParser(description="Digest a normalized Stage 4 Python runtime tree.")
    parser.add_argument("--runtime-root", type=Path, required=True)
    args = parser.parse_args()
    try:
        document = digest_tree(args.runtime_root.absolute())
    except (OSError, ValueError) as error:
        print(f"FAIL: {error}")
        return 1
    print(json.dumps(document, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
