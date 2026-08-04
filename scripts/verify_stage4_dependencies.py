#!/usr/bin/env python3
# 阶段四依赖验证入口：后续集中校验冻结锁、缓存和构建环境，缺输入时严格失败。
from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass
import json
from pathlib import Path
import re


_COMMIT_PATTERN = re.compile(r"[0-9a-f]{40}")
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
_ARCHIVE_FORMATS = frozenset({"tar.gz", "tar.xz", "zip"})
_ARCHIVE_CONSUMERS = frozenset({"cpp_dependency", "validation", "ros_overlay"})


@dataclass(frozen=True)
class DependencyLockEntry:
    """单个源码依赖的不可变锁定身份与归档摘要。"""

    name: str
    url: str
    ref_kind: str
    ref: str
    commit: str
    archive_format: str
    archive_size: int
    archive_sha256: str
    consumers: tuple[str, ...]


def _require_text(record: dict[str, object], field: str) -> str:
    """读取结构化 lock 的非空文本字段，拒绝隐式类型转换。"""
    value = record.get(field)
    if not isinstance(value, str) or not value:
        raise ValueError(f"dependency {field} must be a nonempty string")
    return value


def _require_consumers(record: dict[str, object]) -> tuple[str, ...]:
    """读取归档的唯一构建消费者，拒绝无归属或未准入入口。"""
    value = record.get("consumers")
    if not isinstance(value, list) or not value or not all(
        isinstance(item, str) and item for item in value
    ):
        raise ValueError("dependency consumers must be a nonempty list")
    consumers = tuple(value)
    if len(set(consumers)) != len(consumers):
        raise ValueError("dependency consumers must not contain duplicates")
    if not set(consumers).issubset(_ARCHIVE_CONSUMERS):
        raise ValueError("dependency consumers include an unsupported value")
    return consumers


def load_dependency_lock(path: Path) -> tuple[DependencyLockEntry, ...]:
    """加载并校验依赖锁的不可变 ref、commit 与归档摘要关系。"""
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"dependency lock is not valid JSON: {path}") from error
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("dependency lock must use schema_version 1")
    records = document.get("dependencies")
    if not isinstance(records, list) or not records:
        raise ValueError("dependency lock must contain a nonempty dependencies list")

    entries: list[DependencyLockEntry] = []
    names: set[str] = set()
    for record in records:
        if not isinstance(record, dict):
            raise ValueError("dependency lock entries must be objects")
        name = _require_text(record, "name")
        if name in names:
            raise ValueError(f"duplicate dependency name: {name}")
        names.add(name)
        ref_kind = _require_text(record, "ref_kind")
        if ref_kind not in {"tag", "commit"}:
            raise ValueError("dependency ref_kind must be tag or commit")
        ref = _require_text(record, "ref")
        commit = _require_text(record, "commit")
        if _COMMIT_PATTERN.fullmatch(commit) is None:
            raise ValueError("dependency commit must be a 40-character lowercase SHA")
        if ref_kind == "commit" and ref != commit:
            raise ValueError("dependency commit ref must equal commit")

        archive = record.get("archive")
        if not isinstance(archive, dict):
            raise ValueError("dependency archive must be an object")
        archive_format = _require_text(archive, "format")
        if archive_format not in _ARCHIVE_FORMATS:
            raise ValueError("dependency archive format is not supported")
        archive_size = archive.get("size")
        if isinstance(archive_size, bool) or not isinstance(archive_size, int) or archive_size <= 0:
            raise ValueError("dependency archive size must be a positive integer")
        archive_sha256 = _require_text(archive, "sha256")
        if _SHA256_PATTERN.fullmatch(archive_sha256) is None:
            raise ValueError("dependency archive sha256 must be a lowercase SHA-256")
        consumers = _require_consumers(record)
        entries.append(
            DependencyLockEntry(
                name=name,
                url=_require_text(record, "url"),
                ref_kind=ref_kind,
                ref=ref,
                commit=commit,
                archive_format=archive_format,
                archive_size=archive_size,
                archive_sha256=archive_sha256,
                consumers=consumers,
            )
        )
    return tuple(entries)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析稳定的依赖验证入口，避免 builder 私自承担锁文件检查。"""
    parser = argparse.ArgumentParser(
        description="Verify stage 4 dependency locks and build environment."
    )
    parser.add_argument(
        "--locks-only",
        action="store_true",
        help="verify only frozen dependency locks and canonical caches",
    )
    parser.add_argument(
        "--lock",
        action="append",
        type=Path,
        default=[],
        help="structured dependency lock to validate; may be supplied more than once",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """验证调用方显式提供的冻结依赖锁，缺少输入时严格失败。"""
    args = _parse_args(argv)
    if args.locks_only:
        if not args.lock:
            print("FAIL: --locks-only requires at least one --lock")
            return 1
        try:
            # 每份锁均由同一 parser 验证，避免 CLI 与库入口规则漂移。
            entry_count = sum(len(load_dependency_lock(path)) for path in args.lock)
        except (OSError, ValueError) as error:
            print(f"FAIL: {error}")
            return 1
        print(f"PASS: {entry_count} dependency lock entries verified")
        return 0
    print("FAIL: select --locks-only once dependency locks are available")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
