#!/usr/bin/env python3
# 阶段四 C++ 依赖源码物化：将冻结 canonical archive 私有复制后安全展开为零链接树。
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import stat
from urllib.parse import urlsplit

from stage4_source_archive import materialize_archive, materialized_tree_digest
from verify_stage4_dependencies import load_dependency_lock


def _digest(path: Path) -> tuple[int, str]:
    """流式复算归档大小和 SHA-256，复制前后使用同一规则。"""
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            size += len(chunk)
            digest.update(chunk)
    return size, digest.hexdigest()


def materialize(manifest: Path, lock: Path, canonical_cache: Path, source_work: Path, evidence: Path) -> None:
    """为 C++ consumer 创建本轮独占 archive 副本、源码树及其可审计证据。"""
    if source_work.exists() or evidence.exists() or (
        evidence.parent != source_work and not evidence.parent.is_dir()
    ):
        raise ValueError("source materializer outputs must be absent under existing directories")
    document = json.loads(manifest.read_text(encoding="utf-8"))
    records = document.get("archives") if isinstance(document, dict) else None
    if document.get("schema_version") != 1 or not isinstance(records, list):
        raise ValueError("source cache manifest is invalid")
    entries = {entry.name: entry for entry in load_dependency_lock(lock)}
    selected = [record for record in records if isinstance(record, dict) and "cpp_dependency" in record.get("consumers", [])]
    if not selected:
        raise ValueError("source cache has no cpp_dependency archives")
    source_work.mkdir(mode=0o755)
    archives = source_work / "archives"
    trees = source_work / "trees"
    archives.mkdir(mode=0o755)
    trees.mkdir(mode=0o755)
    evidence_records = []
    for record in sorted(selected, key=lambda item: str(item["name"])):
        name = record.get("name")
        if not isinstance(name, str) or name not in entries:
            raise ValueError("source cache manifest identity differs from dependency lock")
        entry = entries[name]
        if record.get("archive") != {"format": entry.archive_format, "size": entry.archive_size, "sha256": entry.archive_sha256}:
            raise ValueError("source cache manifest archive differs from dependency lock")
        basename = Path(urlsplit(entry.url).path).name
        relative = f"archives/{entry.archive_sha256}/{basename}"
        if record.get("relative_path") != relative:
            raise ValueError("source cache manifest uses a noncanonical archive path")
        source = canonical_cache / relative
        metadata = source.lstat()
        if not stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode) or metadata.st_nlink != 1:
            raise ValueError("canonical source archive must be a singly linked regular file")
        if _digest(source) != (entry.archive_size, entry.archive_sha256):
            raise ValueError("canonical source archive digest differs from dependency lock")
        copied = archives / entry.archive_sha256 / basename
        copied.parent.mkdir(mode=0o755)
        with source.open("rb") as input_stream, copied.open("xb") as output_stream:
            shutil.copyfileobj(input_stream, output_stream, 1024 * 1024)
        copied.chmod(0o644)
        if _digest(copied) != (entry.archive_size, entry.archive_sha256) or copied.stat().st_nlink != 1:
            raise ValueError("private source archive digest differs after copy")
        output = trees / name
        root = materialize_archive(copied, output)
        tree = output / root
        actual = materialized_tree_digest(tree)
        expected = {key: record.get(key) for key in actual}
        if actual != expected:
            raise ValueError("materialized source tree differs from frozen manifest")
        evidence_records.append({"name": name, "archive": str(copied), "tree": str(tree), **actual})
    evidence.write_text(json.dumps({"schema_version": 1, "archives": evidence_records}, sort_keys=True) + "\n", encoding="utf-8")


def main() -> int:
    """提供显式 CLI，禁止使用隐式 source cache 或默认输出目录。"""
    parser = argparse.ArgumentParser(description="Materialize private Stage 4 C++ dependency sources.")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--lock", type=Path, required=True)
    parser.add_argument("--canonical-cache", type=Path, required=True)
    parser.add_argument("--source-work", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    try:
        materialize(args.manifest.resolve(), args.lock.resolve(), args.canonical_cache.resolve(), args.source_work.resolve(), args.evidence.resolve())
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}")
        return 1
    print("PASS: private C++ dependency sources materialized")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
