#!/usr/bin/env python3
"""阶段四 Python runtime 双根复现 gate：独立构建后比较规范 tree digest。"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile


_DIGEST_FIELDS = ("directories", "files", "links", "regular_bytes", "tree_sha256")


def _require_real_directory(path: Path, description: str) -> None:
    """拒绝目录链接和缺失目录，保证双根输出不会落入共享输入。"""
    metadata = path.lstat()
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise ValueError(f"{description} must be a real existing directory")


def _require_regular_executable(path: Path, description: str) -> None:
    """仅接受显式的普通可执行文件，禁止 PATH 或链接回退。"""
    metadata = path.lstat()
    if (
        not stat.S_ISREG(metadata.st_mode)
        or stat.S_ISLNK(metadata.st_mode)
        or not metadata.st_mode & stat.S_IXUSR
    ):
        raise ValueError(f"{description} must be an owner-executable regular file")


def _sha256_text(value: str) -> str:
    """固定 builder 文本输出的证据摘要，避免把大日志重复写入结果。"""
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def _load_digest(path: Path) -> dict[str, int | str]:
    """严格读取 builder 写入的 runtime tree digest，拒绝不完整或类型漂移。"""
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"runtime tree digest is unreadable: {path}") from error
    if not isinstance(document, dict) or tuple(sorted(document)) != _DIGEST_FIELDS:
        raise ValueError(f"runtime tree digest schema is invalid: {path}")
    for field in _DIGEST_FIELDS[:-1]:
        value = document[field]
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise ValueError(f"runtime tree digest field is invalid: {field}")
    tree_sha256 = document["tree_sha256"]
    if (
        not isinstance(tree_sha256, str)
        or len(tree_sha256) != 64
        or any(character not in "0123456789abcdef" for character in tree_sha256)
    ):
        raise ValueError("runtime tree digest sha256 is invalid")
    return document


def _write_evidence(path: Path, document: dict[str, object]) -> None:
    """独占写入最终比较证据，避免旧双根结果覆盖本轮结论。"""
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(document, stream, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def _builder_command(
    network_wrapper: Path,
    builder: Path,
    source: Path,
    work: Path,
    network_evidence: Path,
    micromamba: Path,
    package_cache: Path,
    wheel_cache: Path,
    source_date_epoch: int,
) -> list[str]:
    """构造每轮相同的断网 builder argv，仅替换私有 work 路径。"""
    return [
        str(network_wrapper),
        "--evidence-dir",
        str(network_evidence),
        "--",
        str(builder),
        "--source",
        str(source),
        "--work",
        str(work),
        "--root",
        str(work / "root"),
        "--micromamba",
        str(micromamba),
        "--package-cache",
        str(package_cache),
        "--wheel-cache",
        str(wheel_cache),
        "--source-date-epoch",
        str(source_date_epoch),
    ]


def verify_reproducibility(
    *,
    builder: Path,
    network_wrapper: Path,
    source: Path,
    work_parent: Path,
    micromamba: Path,
    package_cache: Path,
    wheel_cache: Path,
    source_date_epoch: int,
    evidence: Path,
) -> dict[str, object]:
    """运行两个新 work root，并比较纯 staging runtime 的规范 tree digest。"""
    for path, description in (
        (source, "source"),
        (work_parent, "work parent"),
        (package_cache, "package cache"),
        (wheel_cache, "wheel cache"),
    ):
        _require_real_directory(path, description)
    _require_regular_executable(builder, "builder")
    _require_regular_executable(network_wrapper, "network wrapper")
    _require_regular_executable(micromamba, "micromamba")
    if source_date_epoch < 0:
        raise ValueError("source date epoch must be nonnegative")
    if evidence.exists() or evidence.is_symlink() or not evidence.parent.is_dir():
        raise ValueError("evidence must be a new file under an existing directory")

    # 此隔离容器由本入口独占，A/B work 子目录在 builder 启动时仍不存在。
    run_root = Path(tempfile.mkdtemp(prefix="stage4-python-reproducibility-", dir=work_parent))
    work_paths = (run_root / "runtime-a", run_root / "runtime-b")
    runs: list[dict[str, object]] = []
    for work in work_paths:
        network_evidence = run_root / f"{work.name}-network-isolation"
        command = _builder_command(
            network_wrapper,
            builder,
            source,
            work,
            network_evidence,
            micromamba,
            package_cache,
            wheel_cache,
            source_date_epoch,
        )
        completed = subprocess.run(command, check=False, capture_output=True, text=True)
        if completed.returncode != 0:
            raise ValueError(f"runtime builder failed for {work.name} with exit code {completed.returncode}")
        digest_path = work / "python-runtime-tree-digest.json"
        digest = _load_digest(digest_path)
        runs.append(
            {
                "work": str(work),
                "network_evidence": str(network_evidence),
                "runtime_tree_digest": str(digest_path),
                "stdout_sha256": _sha256_text(completed.stdout),
                "stderr_sha256": _sha256_text(completed.stderr),
            }
        )
        if not (work / "root" / "runtime" / "python").is_dir():
            raise ValueError(f"runtime builder did not stage runtime tree: {work}")
        if work == work_paths[0]:
            digest_a = digest
        else:
            digest_b = digest
    if digest_a != digest_b:
        raise ValueError("normalized Python runtime tree digests differ across fresh roots")

    document: dict[str, object] = {
        "schema_version": 1,
        "runs": runs,
        "runtime_tree_a": digest_a,
        "runtime_tree_b": digest_b,
    }
    _write_evidence(evidence, document)
    return document


def main() -> int:
    """解析双根验证参数；成功时只输出可复现 builder 的比较结论。"""
    parser = argparse.ArgumentParser(description="Build and compare two fresh Stage 4 Python runtimes.")
    parser.add_argument("--builder", type=Path, required=True)
    parser.add_argument("--network-wrapper", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--work-parent", type=Path, required=True)
    parser.add_argument("--micromamba", type=Path, required=True)
    parser.add_argument("--package-cache", type=Path, required=True)
    parser.add_argument("--wheel-cache", type=Path, required=True)
    parser.add_argument("--source-date-epoch", type=int, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    try:
        verify_reproducibility(
            builder=args.builder.absolute(),
            network_wrapper=args.network_wrapper.absolute(),
            source=args.source.absolute(),
            work_parent=args.work_parent.absolute(),
            micromamba=args.micromamba.absolute(),
            package_cache=args.package_cache.absolute(),
            wheel_cache=args.wheel_cache.absolute(),
            source_date_epoch=args.source_date_epoch,
            evidence=args.evidence.absolute(),
        )
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"FAIL: {error}")
        return 1
    print("PASS: two fresh Python runtime trees have identical normalized digests")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
