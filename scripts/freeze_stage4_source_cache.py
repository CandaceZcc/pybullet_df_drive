#!/usr/bin/env python3
# 阶段四源码缓存冻结入口：只从显式本地归档输入生成可复核的 canonical cache。
from __future__ import annotations

import argparse
from collections.abc import Sequence
import hashlib
import json
from pathlib import Path
import tempfile
from urllib.parse import urlsplit

from stage4_source_archive import archive_census, materialize_archive, materialized_tree_digest
from verify_stage4_dependencies import load_dependency_lock


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析冻结输入与全新输出位置，禁止隐式网络或 Git 工作树来源。"""
    parser = argparse.ArgumentParser(description="Freeze the stage 4 canonical source cache.")
    parser.add_argument("--lock", type=Path, action="append", required=True)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """从 lock 指定的本地归档生成 canonical cache 与结构化 manifest。"""
    args = parse_args(argv)
    try:
        if args.cache_root.exists() or args.manifest.exists():
            raise ValueError("source cache output and manifest must not already exist")
        entries = [entry for lock in args.lock for entry in load_dependency_lock(lock)]
        if len({entry.name for entry in entries}) != len(entries):
            raise ValueError("source cache locks contain duplicate dependency names")
        records: list[dict[str, object]] = []
        for entry in sorted(entries, key=lambda item: item.name):
            basename = Path(urlsplit(entry.url).path).name
            source = args.source_dir / basename
            metadata = source.stat()
            payload = source.read_bytes()
            if metadata.st_size != entry.archive_size:
                raise ValueError(f"source archive size differs from lock: {entry.name}")
            if hashlib.sha256(payload).hexdigest() != entry.archive_sha256:
                raise ValueError(f"source archive sha256 differs from lock: {entry.name}")
            census = archive_census(source)
            with tempfile.TemporaryDirectory(prefix="stage4-source-materialized-") as work:
                materialized_output = Path(work) / "tree"
                root = materialize_archive(source, materialized_output)
                materialized = materialized_tree_digest(materialized_output / root)
            relative_path = f"archives/{entry.archive_sha256}/{basename}"
            destination = args.cache_root / relative_path
            destination.parent.mkdir(parents=True, exist_ok=True)
            with destination.open("xb") as output:
                output.write(payload)
            records.append(
                {
                    "name": entry.name,
                    "url": entry.url,
                    "ref_kind": entry.ref_kind,
                    "ref": entry.ref,
                    "commit": entry.commit,
                    "consumers": list(entry.consumers),
                    "archive": {
                        "format": entry.archive_format,
                        "size": entry.archive_size,
                        "sha256": entry.archive_sha256,
                    },
                    "relative_path": relative_path,
                    **census,
                    **materialized,
                }
            )
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.write_text(
            json.dumps({"schema_version": 1, "archives": records}, indent=2, sort_keys=True)
            + "\n",
            encoding="utf-8",
        )
        print(f"PASS: {len(entries)} canonical source archives frozen")
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
