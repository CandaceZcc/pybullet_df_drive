#!/usr/bin/env python3
# 阶段四 Python runtime staging 清理：仅在 pip 后移除可重建 metadata 和 bytecode。
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat


_CANONICAL_PREFIX_SHA256 = hashlib.sha256(b"stage4-python-runtime-prefix").hexdigest()


def _unlink_regular(path: Path, description: str) -> None:
    """只删除 staging 内单一普通文件，阻断链接或设备节点带来的越界风险。"""
    path_stat = path.lstat()
    if not stat.S_ISREG(path_stat.st_mode) or stat.S_ISLNK(path_stat.st_mode):
        raise ValueError(f"{description} must be a regular file")
    path.unlink()


def _normalize_metadata(path: Path, source_date_epoch: int) -> None:
    """固定 staging 成员的安全 mode 与 mtime，消除主机 umask 和构建时间影响。"""
    metadata = path.lstat()
    if stat.S_ISLNK(metadata.st_mode):
        # conda-pack 的相对链接是运行时格式的一部分；只规范链接自身 mtime。
        os.utime(path, (source_date_epoch, source_date_epoch), follow_symlinks=False)
        return
    if stat.S_ISDIR(metadata.st_mode):
        path.chmod(0o755)
    elif stat.S_ISREG(metadata.st_mode):
        path.chmod(0o755 if metadata.st_mode & 0o111 else 0o644)
    else:
        raise ValueError("runtime tree must contain only directories and regular files")
    os.utime(path, (source_date_epoch, source_date_epoch), follow_symlinks=False)


def _canonicalize_conda_prefix_hashes(runtime_root: Path) -> None:
    """保留 Conda package record，但移除 paths_data 中随 builder 根变化的前缀哈希。"""
    conda_meta = runtime_root / "conda-meta"
    if not conda_meta.exists():
        return
    metadata = conda_meta.lstat()
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise ValueError("conda-meta must be a real directory")
    for record_path in sorted(conda_meta.glob("*.json"), key=lambda path: path.name):
        record_metadata = record_path.lstat()
        if not stat.S_ISREG(record_metadata.st_mode) or stat.S_ISLNK(record_metadata.st_mode):
            raise ValueError("conda package record must be a regular file")
        try:
            document = json.loads(record_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as error:
            raise ValueError("conda package record must be valid JSON") from error
        if not isinstance(document, dict):
            raise ValueError("conda package record must be a JSON object")
        paths_data = document.get("paths_data")
        if paths_data is None:
            continue
        if not isinstance(paths_data, dict) or not isinstance(paths_data.get("paths"), list):
            raise ValueError("conda package record paths_data is invalid")
        changed = False
        for path_data in paths_data["paths"]:
            if not isinstance(path_data, dict) or "sha256_in_prefix" not in path_data:
                continue
            prefix_hash = path_data["sha256_in_prefix"]
            if prefix_hash is None:
                continue
            if (
                not isinstance(prefix_hash, str)
                or len(prefix_hash) != 64
                or any(character not in "0123456789abcdef" for character in prefix_hash)
            ):
                raise ValueError("conda package record prefix hash is invalid")
            if prefix_hash != _CANONICAL_PREFIX_SHA256:
                path_data["sha256_in_prefix"] = _CANONICAL_PREFIX_SHA256
                changed = True
        if changed:
            record_path.write_text(
                json.dumps(document, ensure_ascii=True, separators=(",", ":"), sort_keys=True) + "\n",
                encoding="utf-8",
            )


def sanitize(runtime_root: Path, source_date_epoch: int) -> None:
    """pack 和 pip 成功后清理 history、.pyc 与空 __pycache__ 并规范元数据。"""
    if not runtime_root.is_dir() or runtime_root.is_symlink():
        raise ValueError("runtime root must be a real directory")
    if source_date_epoch < 0:
        raise ValueError("source date epoch must be nonnegative")
    history = runtime_root / "conda-meta" / "history"
    if history.exists() or history.is_symlink():
        _unlink_regular(history, "conda history")
    for directory, subdirectories, filenames in os.walk(runtime_root, topdown=False, followlinks=False):
        current = Path(directory)
        for filename in filenames:
            if filename.endswith(".pyc"):
                _unlink_regular(current / filename, "Python bytecode")
        for name in subdirectories:
            if name != "__pycache__":
                continue
            cache_directory = current / name
            cache_stat = cache_directory.lstat()
            if not stat.S_ISDIR(cache_stat.st_mode) or stat.S_ISLNK(cache_stat.st_mode):
                raise ValueError("__pycache__ must be a real directory")
            cache_directory.rmdir()
    _canonicalize_conda_prefix_hashes(runtime_root)
    for directory, subdirectories, filenames in os.walk(runtime_root, topdown=False, followlinks=False):
        current = Path(directory)
        for filename in filenames:
            _normalize_metadata(current / filename, source_date_epoch)
        for name in subdirectories:
            _normalize_metadata(current / name, source_date_epoch)
    _normalize_metadata(runtime_root, source_date_epoch)


def main() -> int:
    """执行一次 staging 清理，不调用 conda-unpack 或 Python import。"""
    parser = argparse.ArgumentParser(description="Remove Stage 4 Python runtime bytecode and Conda history.")
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--source-date-epoch", type=int, required=True)
    args = parser.parse_args()
    try:
        sanitize(args.runtime_root.resolve(), args.source_date_epoch)
    except (OSError, ValueError) as error:
        print(f"FAIL: {error}")
        return 1
    print("PASS: Python runtime staging tree sanitized")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
