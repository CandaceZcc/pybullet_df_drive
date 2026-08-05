#!/usr/bin/env python3
# 阶段四 Python package materializer：将只读 canonical archive 转为本轮原生 flat cache。
from __future__ import annotations

import argparse
from collections.abc import Sequence
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat
from urllib.parse import urlsplit


def _digest(path: Path) -> tuple[int, str, str]:
    """流式复算 archive 的 size、MD5 和 SHA-256，不解包或执行制品。"""
    md5 = hashlib.md5()
    sha256 = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            size += len(chunk)
            md5.update(chunk)
            sha256.update(chunk)
    return size, md5.hexdigest(), sha256.hexdigest()


def _require_sha256(value: object, field: str) -> str:
    """读取固定小写 SHA-256 字段，拒绝宽松类型和大小写漂移。"""
    if not isinstance(value, str) or len(value) != 64 or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise ValueError(f"archive {field} must be a lowercase SHA-256")
    return value


def _require_md5(value: object) -> str:
    """读取显式 lock render 使用的 MD5，确保 native cache 来源仍可追溯。"""
    if not isinstance(value, str) or len(value) != 32 or any(
        character not in "0123456789abcdef" for character in value
    ):
        raise ValueError("archive md5 must be a lowercase MD5")
    return value


def _parse_archives(manifest: Path) -> list[dict[str, object]]:
    """解析 canonical cache manifest，保留 URL 的稳定排序作为 urls.txt 顺序。"""
    document = json.loads(manifest.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("package cache manifest is invalid")
    archives = document.get("archives")
    if document.get("schema_version") != 1 or not isinstance(archives, list) or not archives:
        raise ValueError("package cache manifest is invalid")
    parsed: list[dict[str, object]] = []
    filenames: dict[str, tuple[int, str, str]] = {}
    for record in archives:
        if not isinstance(record, dict):
            raise ValueError("package cache manifest archive must be an object")
        filename = record.get("filename")
        relative_path = record.get("relative_path")
        url = record.get("url")
        size = record.get("size")
        if not isinstance(filename, str) or not filename or Path(filename).name != filename:
            raise ValueError("archive filename must be a basename")
        if not isinstance(relative_path, str):
            raise ValueError("archive relative_path must be text")
        relative = PurePosixPath(relative_path)
        if relative.is_absolute() or any(part in {"", ".", ".."} for part in relative.parts):
            raise ValueError("archive relative_path is unsafe")
        if not isinstance(url, str) or urlsplit(url).scheme != "https" or not urlsplit(url).netloc:
            raise ValueError("archive URL must be HTTPS")
        if Path(urlsplit(url).path).name != filename:
            raise ValueError("archive URL basename differs from filename")
        if not isinstance(size, int) or isinstance(size, bool) or size < 1:
            raise ValueError("archive size must be positive")
        md5 = _require_md5(record.get("md5"))
        sha256 = _require_sha256(record.get("sha256"), "sha256")
        identity = (size, md5, sha256)
        previous = filenames.setdefault(filename, identity)
        if previous != identity:
            raise ValueError("archive basename collision has different content")
        parsed.append(
            {
                "filename": filename,
                "relative_path": relative_path,
                "url": url,
                "size": size,
                "md5": md5,
                "sha256": sha256,
            }
        )
    if len({record["url"] for record in parsed}) != len(parsed):
        raise ValueError("package cache manifest URLs must be unique")
    return sorted(parsed, key=lambda record: str(record["url"]))


def _validate_outputs(destination: Path, evidence: Path) -> None:
    """materializer 输出必须全新且互不重叠，避免复用可写 native cache。"""
    if destination == evidence or destination in evidence.parents or evidence in destination.parents:
        raise ValueError("materializer outputs must not overlap")
    if destination.exists() or evidence.exists() or not evidence.parent.is_dir():
        raise ValueError("materializer outputs must be absent under existing directories")


def _copy_archive(source: Path, target: Path) -> None:
    """以 exclusive create 写本轮普通文件，禁止链接或覆盖已有 archive。"""
    with source.open("rb") as input_stream, target.open("xb") as output_stream:
        while chunk := input_stream.read(1024 * 1024):
            output_stream.write(chunk)
    target.chmod(0o644)


def materialize(manifest: Path, canonical_cache: Path, destination: Path, evidence: Path) -> None:
    """将 manifest 精确声明的 canonical archive 物化为 root 级 native cache。"""
    records = _parse_archives(manifest)
    _validate_outputs(destination, evidence)
    canonical_root = canonical_cache.resolve()
    if not canonical_root.is_dir():
        raise ValueError("canonical package cache must be an existing directory")
    for record in records:
        source = canonical_root / str(record["relative_path"])
        source_stat = source.lstat()
        if (
            not stat.S_ISREG(source_stat.st_mode)
            or stat.S_ISLNK(source_stat.st_mode)
            or source_stat.st_nlink != 1
        ):
            raise ValueError("canonical archive must be a singly linked regular file")
        actual = _digest(source)
        expected = (record["size"], record["md5"], record["sha256"])
        if actual != expected:
            raise ValueError("canonical archive digest differs from manifest")

    destination.mkdir(parents=True, mode=0o755)
    copied: dict[str, dict[str, object]] = {}
    for record in records:
        filename = str(record["filename"])
        if filename in copied:
            continue
        source = canonical_root / str(record["relative_path"])
        target = destination / filename
        _copy_archive(source, target)
        if _digest(target) != (record["size"], record["md5"], record["sha256"]):
            raise ValueError("native archive digest differs after copy")
        copied[filename] = {
            "url": record["url"],
            "sha256": record["sha256"],
        }
    urls = "".join(f"{record['url']}\n" for record in records)
    urls_path = destination / "urls.txt"
    urls_path.write_text(urls, encoding="utf-8", newline="")
    urls_path.chmod(0o644)
    native_files = {path.name for path in destination.iterdir()}
    if native_files != set(copied) | {"urls.txt"}:
        raise ValueError("native cache contains unexpected files")
    evidence.write_text(
        json.dumps(
            {
                "archives": sorted(copied),
                "destination": str(destination.resolve()),
                "urls_sha256": hashlib.sha256(urls.encode("utf-8")).hexdigest(),
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
    """运行一次全新 native cache materialization，供隔离 builder 调用。"""
    parser = argparse.ArgumentParser(description="Materialize a Stage 4 native Conda cache.")
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
    print("PASS: Python native package cache materialized")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
