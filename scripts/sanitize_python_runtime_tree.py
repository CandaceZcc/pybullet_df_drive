#!/usr/bin/env python3
# 阶段四 Python runtime staging 清理：仅在 pip 后移除可重建 metadata 和 bytecode。
from __future__ import annotations

import argparse
import os
from pathlib import Path
import stat


def _unlink_regular(path: Path, description: str) -> None:
    """只删除 staging 内单一普通文件，阻断链接或设备节点带来的越界风险。"""
    path_stat = path.lstat()
    if not stat.S_ISREG(path_stat.st_mode) or stat.S_ISLNK(path_stat.st_mode):
        raise ValueError(f"{description} must be a regular file")
    path.unlink()


def sanitize(runtime_root: Path) -> None:
    """pack 和 pip 成功后清理 history、.pyc 与空 __pycache__ 目录。"""
    if not runtime_root.is_dir() or runtime_root.is_symlink():
        raise ValueError("runtime root must be a real directory")
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


def main() -> int:
    """执行一次 staging 清理，不调用 conda-unpack 或 Python import。"""
    parser = argparse.ArgumentParser(description="Remove Stage 4 Python runtime bytecode and Conda history.")
    parser.add_argument("--runtime-root", type=Path, required=True)
    args = parser.parse_args()
    try:
        sanitize(args.runtime_root.resolve())
    except (OSError, ValueError) as error:
        print(f"FAIL: {error}")
        return 1
    print("PASS: Python runtime staging tree sanitized")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
