#!/usr/bin/env python3
"""阶段四 Python runtime relocation smoke：只在随机副本执行 conda-unpack。"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import tempfile


def _load_tree_digest():
    """加载独立 tree digest，复用与双根 gate 完全相同的成员判定规则。"""
    helper = Path(__file__).with_name("python_runtime_tree_digest.py")
    spec = importlib.util.spec_from_file_location("stage4_python_runtime_tree_digest", helper)
    if spec is None or spec.loader is None:
        raise ValueError("runtime tree digest helper cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    digest_tree = getattr(module, "digest_tree", None)
    if not callable(digest_tree):
        raise ValueError("runtime tree digest helper is invalid")
    return digest_tree


def _sha256_text(value: str) -> str:
    """保存子进程输出摘要，避免将任意输出直接写入结构化 evidence。"""
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def _write_evidence(path: Path, document: dict[str, object]) -> None:
    """以独占创建写入本轮证据，防止旧 smoke 被静默覆盖。"""
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
        json.dump(document, stream, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def relocate_and_smoke(runtime_root: Path, copy_parent: Path, evidence: Path) -> dict[str, object]:
    """复制 staging 后运行副本 conda-unpack，并证明源 tree 在前后完全未变。"""
    source_metadata = runtime_root.lstat()
    if not stat.S_ISDIR(source_metadata.st_mode) or stat.S_ISLNK(source_metadata.st_mode):
        raise ValueError("runtime root must be a real directory")
    if not copy_parent.is_dir() or copy_parent.is_symlink():
        raise ValueError("copy parent must be a real existing directory")
    if evidence.exists() or evidence.is_symlink() or not evidence.parent.is_dir():
        raise ValueError("evidence must be a new file under an existing directory")

    digest_tree = _load_tree_digest()
    source_before = digest_tree(runtime_root)
    copy_root = Path(tempfile.mkdtemp(prefix="stage4-python-relocation-", dir=copy_parent))
    relocated_runtime = copy_root / "runtime"
    # 保留 relative symlink 与 metadata；copytree 绝不对源 staging 执行 conda-unpack。
    shutil.copytree(runtime_root, relocated_runtime, symlinks=True, copy_function=shutil.copy2)
    conda_unpack = relocated_runtime / "bin" / "conda-unpack"
    unpack_metadata = conda_unpack.lstat()
    if not stat.S_ISREG(unpack_metadata.st_mode) or stat.S_ISLNK(unpack_metadata.st_mode):
        raise ValueError("relocated conda-unpack must be a regular file")
    if not unpack_metadata.st_mode & stat.S_IXUSR:
        raise ValueError("relocated conda-unpack must be owner-executable")

    completed = subprocess.run(
        [str(conda_unpack)],
        check=False,
        capture_output=True,
        # conda-pack 的 conda-unpack 使用 /usr/bin/env python，必须只从副本 bin 解析它。
        env={
            "HOME": str(copy_root),
            "PATH": f"{relocated_runtime / 'bin'}:{os.defpath}",
            "LANG": "C.UTF-8",
            "LC_ALL": "C.UTF-8",
        },
        text=True,
    )
    source_after = digest_tree(runtime_root)
    if source_before != source_after:
        raise ValueError("source runtime tree changed during relocation smoke")
    if completed.returncode != 0:
        raise ValueError(f"relocated conda-unpack failed with exit code {completed.returncode}")

    document: dict[str, object] = {
        "schema_version": 1,
        "source_runtime": str(runtime_root),
        "relocated_runtime": str(relocated_runtime),
        "source_tree_before": source_before,
        "source_tree_after": source_after,
        "conda_unpack": {
            "argv": [str(conda_unpack)],
            "returncode": completed.returncode,
            "stdout_sha256": _sha256_text(completed.stdout),
            "stderr_sha256": _sha256_text(completed.stderr),
        },
    }
    _write_evidence(evidence, document)
    return document


def main() -> int:
    """解析显式路径并输出 relocation 结果；不在原 staging 执行任何程序。"""
    parser = argparse.ArgumentParser(description="Run conda-unpack only in a copied Stage 4 Python runtime.")
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--copy-parent", type=Path)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    sys.dont_write_bytecode = True
    runtime_root = args.runtime_root.absolute()
    copy_parent = args.copy_parent.absolute() if args.copy_parent else args.evidence.absolute().parent
    evidence = args.evidence.absolute()
    try:
        relocate_and_smoke(runtime_root, copy_parent, evidence)
    except (OSError, ValueError, shutil.Error) as error:
        print(f"FAIL: {error}")
        return 1
    print("PASS: Python runtime relocation smoke completed in a random copy")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
