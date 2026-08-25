"""阶段四 E：从仓库源树组装可嵌入 `.run` 的最小项目 payload。"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import tempfile


_REQUIRED_FILES = (
    "main.py",
    "runSim",
    "pyproject.toml",
    "scripts/stage4_v2_simulation_runtime.py",
    "scripts/stage4_v2_dashboard.py",
    "scripts/stage4_release_setup.py",
    "scripts/run_mid360_golf_mapping.py",
    "scripts/verify_mid360_golf_mapping_replay.py",
    "scripts/verify_livox_viewer2_linux.py",
    "scripts/verify_lvx2.py",
    "scripts/mid360_golf_simulation.py",
    "scripts/mid360_golf_command_peer.py",
    "scripts/test_rc_sticks.py",
    "packaging/python-environment.yml",
)
_REQUIRED_DIRECTORIES = (
    "slope_sim",
    "cpp",
    "proto",
    "urdf",
    "configs",
)
_LOCK_FILES = (
    "cpp-dependencies.lock",
    "ubuntu24-system-dependencies.lock",
    "ros2-dependencies.lock",
    "ros2-apt-packages.lock",
    "python.conda-lock.yml",
    "python-linux-64.lock",
)
_EXCLUDED_DIRECTORY_NAMES = frozenset({"__pycache__", "build", "results", "references"})


def _sha256_file(path: Path) -> str:
    """返回已进入 payload 的常规文件摘要。"""
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_regular_file(path: Path) -> None:
    """拒绝链接或特殊文件进入发布 payload。"""
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"required payload file is invalid: {path}")


def _require_directory(path: Path) -> None:
    """目录必须是实际目录，避免发布输入经链接漂移。"""
    if path.is_symlink() or not path.is_dir():
        raise ValueError(f"required payload directory is invalid: {path}")


def _copy_file(source: Path, destination: Path) -> None:
    """复制已验证的普通文件并保留入口权限。"""
    _require_regular_file(source)
    destination.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
    shutil.copy2(source, destination)


def _copy_directory(source: Path, destination: Path) -> None:
    """复制运行源树，排除测试、缓存和构建产物。"""
    _require_directory(source)
    destination.mkdir(mode=0o755, parents=True, exist_ok=False)
    for candidate in sorted(source.rglob("*"), key=lambda path: path.as_posix()):
        relative = candidate.relative_to(source)
        if relative.parts[0] == "tests" or any(
            part in _EXCLUDED_DIRECTORY_NAMES for part in relative.parts
        ):
            continue
        target = destination / relative
        if candidate.is_symlink():
            raise ValueError(f"payload directory contains a symlink: {candidate}")
        if candidate.is_dir():
            target.mkdir(mode=0o755, exist_ok=False)
        elif candidate.is_file():
            _copy_file(candidate, target)
        else:
            raise ValueError(f"payload directory contains a special file: {candidate}")


def _write_manifest(root: Path) -> None:
    """在 payload 内冻结实际复制文件的 SHA，供安装前后身份审计。"""
    files = {
        path.relative_to(root).as_posix(): _sha256_file(path)
        for path in sorted(root.rglob("*"), key=lambda path: path.as_posix())
        if path.is_file() and not path.is_symlink()
    }
    (root / "payload-manifest.json").write_text(
        json.dumps({"schema_version": 1, "files": files}, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def build_payload(source: Path, output: Path) -> None:
    """以唯一 staging 组装 release 源输入，成功后原子替换新目标。"""
    if not source.is_absolute() or source.is_symlink() or not source.is_dir():
        raise ValueError("source must be an absolute non-symlink directory")
    if not output.is_absolute() or output.exists() or not output.parent.is_dir():
        raise ValueError("output must be a new absolute path below an existing directory")

    for relative in _REQUIRED_FILES:
        _require_regular_file(source / relative)
    for relative in _REQUIRED_DIRECTORIES:
        _require_directory(source / relative)
    for name in _LOCK_FILES:
        _require_regular_file(source / "packaging" / "locks" / name)

    staging = Path(tempfile.mkdtemp(prefix=".stage4-project-payload-", dir=output.parent))
    try:
        for relative in _REQUIRED_FILES:
            _copy_file(source / relative, staging / relative)
        for relative in _REQUIRED_DIRECTORIES:
            _copy_directory(source / relative, staging / relative)
        for name in _LOCK_FILES:
            _copy_file(source / "packaging" / "locks" / name, staging / "packaging" / "locks" / name)
        _write_manifest(staging)
        os.replace(staging, output)
    except BaseException:
        if staging.exists() or staging.is_symlink():
            shutil.rmtree(staging)
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    build_payload(args.source, args.output)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
