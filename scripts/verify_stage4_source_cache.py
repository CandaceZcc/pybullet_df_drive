#!/usr/bin/env python3
# 阶段四源码缓存校验入口：后续严格比对冻结锁、manifest 与 canonical 归档树。
from __future__ import annotations

import argparse
from collections.abc import Sequence
import hashlib
import json
from pathlib import Path
import stat
import tempfile
from urllib.parse import urlsplit

from stage4_source_archive import archive_census, materialize_archive, materialized_tree_digest
from verify_stage4_dependencies import load_dependency_lock


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析 source cache 的显式输入，禁止 builder 使用隐式默认缓存。"""
    parser = argparse.ArgumentParser(description="Verify the stage 4 canonical source cache.")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--lock", type=Path, action="append", required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """校验 lock、manifest 与 canonical cache 的同一不可变归档集合。"""
    args = parse_args(argv)
    try:
        document = json.loads(args.manifest.read_text(encoding="utf-8"))
        records = document.get("archives") if isinstance(document, dict) else None
        if document.get("schema_version") != 1 or not isinstance(records, list):
            raise ValueError("source cache manifest must use schema_version 1 with archives")
        entries = [entry for lock in args.lock for entry in load_dependency_lock(lock)]
        by_name = {entry.name: entry for entry in entries}
        if len(by_name) != len(entries):
            raise ValueError("source cache locks contain duplicate dependency names")
        if len(records) != len(by_name):
            raise ValueError("source cache manifest entries do not match dependency locks")
        seen: set[str] = set()
        expected_paths: set[str] = set()
        for record in records:
            if not isinstance(record, dict) or not isinstance(record.get("name"), str):
                raise ValueError("source cache manifest archive must have a name")
            name = record["name"]
            if name in seen or name not in by_name:
                raise ValueError("source cache manifest entries do not match dependency locks")
            seen.add(name)
            entry = by_name[name]
            archive = record.get("archive")
            if not isinstance(archive, dict):
                raise ValueError("source cache manifest archive metadata is required")
            expected = {
                "url": entry.url,
                "ref_kind": entry.ref_kind,
                "ref": entry.ref,
                "commit": entry.commit,
                "consumers": list(entry.consumers),
            }
            if any(record.get(field) != value for field, value in expected.items()):
                raise ValueError("source cache manifest identity differs from dependency lock")
            if archive != {
                "format": entry.archive_format,
                "size": entry.archive_size,
                "sha256": entry.archive_sha256,
            }:
                raise ValueError("source cache manifest archive differs from dependency lock")
            basename = Path(urlsplit(entry.url).path).name
            relative_path = f"archives/{entry.archive_sha256}/{basename}"
            if record.get("relative_path") != relative_path:
                raise ValueError("source cache manifest uses a noncanonical archive path")
            expected_paths.update(
                {
                    "archives",
                    f"archives/{entry.archive_sha256}",
                    relative_path,
                }
            )
            artifact = args.cache_root / relative_path
            metadata = artifact.lstat()
            if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
                raise ValueError("source cache artifact must be a singly linked regular file")
            if metadata.st_size != entry.archive_size:
                raise ValueError("source cache artifact size differs from dependency lock")
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
            if digest != entry.archive_sha256:
                raise ValueError("source cache artifact sha256 differs from dependency lock")
            census = archive_census(artifact)
            if record.get("top_level_root") != census["top_level_root"]:
                raise ValueError("source cache manifest top-level root differs from canonical archive")
            if record.get("member_count") != census["member_count"]:
                raise ValueError("source cache manifest member count differs from canonical archive")
            if record.get("regular_bytes") != census["regular_bytes"]:
                raise ValueError("source cache manifest regular bytes differ from canonical archive")
            if record.get("symlink_count") != census["symlink_count"]:
                raise ValueError("source cache manifest symlink count differs from canonical archive")
            # 归档成员合法后仍须物化链接，固定最终零链接文件树的内容合同。
            with tempfile.TemporaryDirectory(prefix="stage4-source-materialized-") as work:
                materialized_output = Path(work) / "tree"
                root = materialize_archive(artifact, materialized_output)
                materialized = materialized_tree_digest(materialized_output / root)
            if record.get("materialized_member_count") != materialized["materialized_member_count"]:
                raise ValueError(
                    "source cache manifest materialized member count differs from canonical archive"
                )
            if record.get("materialized_regular_bytes") != materialized[
                "materialized_regular_bytes"
            ]:
                raise ValueError(
                    "source cache manifest materialized regular bytes differs from canonical archive"
                )
            if record.get("materialized_tree_sha256") != materialized[
                "materialized_tree_sha256"
            ]:
                raise ValueError(
                    "source cache manifest materialized tree sha256 differs from canonical archive"
                )
        actual_paths = {
            candidate.relative_to(args.cache_root).as_posix()
            for candidate in args.cache_root.rglob("*")
        }
        if actual_paths != expected_paths:
            raise ValueError("source cache contains an unexpected entry")
        print(f"PASS: {len(entries)} canonical source archives verified")
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
