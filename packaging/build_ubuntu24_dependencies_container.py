#!/usr/bin/env python3
# 阶段四 Ubuntu 24.04 私有构建容器：复制只读输入到容器 staging 后执行断网依赖构建。
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import stat
import subprocess
import sys
from typing import Sequence


_ROOT = Path(__file__).resolve().parents[1]
_STAGING_ROOT = Path("/opt/stage4-private")
_INPUT_ROOT = Path("/stage4-input")
_OUTPUT_PARENT = Path("/stage4-output-parent")


def _require_regular_file(path: Path, label: str) -> Path:
    """只接受真实普通输入文件，禁止链接把容器挂载带出受控根。"""
    metadata = path.lstat()
    if not stat.S_ISREG(metadata.st_mode) or path.is_symlink():
        raise ValueError(f"{label} must be a regular file")
    return path.resolve(strict=True)


def _require_real_directory(path: Path, label: str) -> Path:
    """只接受真实目录作为只读输入或结果父目录。"""
    metadata = path.lstat()
    if not stat.S_ISDIR(metadata.st_mode) or path.is_symlink():
        raise ValueError(f"{label} must be a real directory")
    return path.resolve(strict=True)


def _require_executable(path: Path, label: str) -> Path:
    """Docker 必须由调用方显式指定的普通可执行文件启动。"""
    normalized = _require_regular_file(path, label)
    if not os.access(normalized, os.X_OK):
        raise ValueError(f"{label} must be executable")
    return normalized


def _path_in_repository(path: Path, label: str) -> Path:
    """锁和 manifest 必须随私有 packaging staging 一起复制，拒绝仓库外路径。"""
    normalized = _require_regular_file(path, label)
    try:
        normalized.relative_to(_ROOT)
    except ValueError as error:
        raise ValueError(f"{label} must be inside the repository") from error
    return normalized


def _private_path(path: Path) -> Path:
    """将宿主仓库输入映射为容器内 root-owned staging 路径。"""
    return _STAGING_ROOT / path.relative_to(_ROOT)


def _require_new_output_root(path: Path) -> Path:
    """结果根必须是已有真实父目录下的新直接子目录，避免覆盖历史证据。"""
    if not path.is_absolute() or path.name in {"", ".", ".."}:
        raise ValueError("output-root must be an absolute direct child path")
    if path.exists() or path.is_symlink():
        raise ValueError("output-root must be absent")
    _require_real_directory(path.parent, "output-root parent")
    return path


def _load_system_lock(path: Path) -> dict[str, object]:
    """复用正式 verifier 的系统锁解析，避免 Docker 入口自行放宽 schema。"""
    sys.path.insert(0, str(_ROOT / "scripts"))
    try:
        from verify_stage4_dependencies import load_system_dependency_lock

        return load_system_dependency_lock(path)
    finally:
        sys.path.pop(0)


def _container_script(
    *,
    apt_specs: list[str],
    output_name: str,
    source_archive_manifest: Path,
    dependency_lock: Path,
    source_cache_locks: list[Path],
    source_date_epoch: int,
) -> str:
    """生成容器内唯一执行序列：联网 provision 完成后才进入可复核断网 child。"""
    result_root = _OUTPUT_PARENT / output_name
    build_log = result_root / "container-build.log"
    exit_code_record = result_root / "container-exit-code.txt"
    builder = _STAGING_ROOT / "packaging" / "build_dependencies.sh"
    wrapper = _STAGING_ROOT / "packaging" / "run_network_isolated.sh"
    command = [
        str(wrapper),
        "--evidence-dir",
        str(result_root / "network-evidence"),
        "--",
        str(builder),
        "--network-evidence",
        str(result_root / "network-evidence"),
        "--cmake",
        "/usr/bin/cmake",
        "--cc",
        "/usr/bin/x86_64-linux-gnu-gcc-13",
        "--cxx",
        "/usr/bin/x86_64-linux-gnu-g++-13",
        "--source-archive-cache",
        str(_STAGING_ROOT / "source-cache"),
        "--source-archive-manifest",
        str(_private_path(source_archive_manifest)),
        "--dependency-lock",
        str(_private_path(dependency_lock)),
    ]
    for lock in source_cache_locks:
        command.extend(("--source-cache-lock", str(_private_path(lock))))
    command.extend(
        (
            "--source-work",
            str(result_root / "source-work"),
            "--build-root",
            str(result_root / "build-root"),
            "--install-prefix",
            str(result_root / "install-prefix"),
            "--validation-prefix",
            str(result_root / "validation-prefix"),
            "--source-date-epoch",
            str(source_date_epoch),
        )
    )
    install_command = shlex.join(["apt-get", "install", "-y", "--no-install-recommends", *apt_specs])
    completion_trap = (
        "status=$?; "
        f"printf '%s\\n' \"$status\" > {shlex.quote(str(exit_code_record))}; "
        f"chmod -R a+rX -- {shlex.quote(str(result_root))}"
    )
    return "\n".join(
        (
            "set -euo pipefail",
            "export DEBIAN_FRONTEND=noninteractive",
            "apt-get update",
            install_command,
            "install -d -m 0700 /opt/stage4-private",
            "cp -a -- /stage4-input/packaging /opt/stage4-private/packaging",
            "cp -a -- /stage4-input/scripts /opt/stage4-private/scripts",
            "cp -a -- /stage4-input/source-cache /opt/stage4-private/source-cache",
            "chown -R root:root -- /opt/stage4-private",
            f"test ! -e {shlex.quote(str(result_root))}",
            # 容器 root 构建完成后，宿主必须能遍历产物以执行 ELF 与证据复核。
            f"mkdir -m 0755 -- {shlex.quote(str(result_root))}",
            # detached Docker 不保留 attached client 输出，因此结果根必须保存构建日志。
            f"exec > >(tee -a {shlex.quote(str(build_log))}) 2>&1",
            # 无论断网 child 成功或失败，均保留宿主可审计的退出码与全部证据。
            f"trap {shlex.quote(completion_trap)} EXIT",
            shlex.join(command),
        )
    )


def docker_command(
    *,
    docker: Path,
    system_lock: Path,
    source_archive_cache: Path,
    source_archive_manifest: Path,
    dependency_lock: Path,
    source_cache_locks: list[Path],
    output_root: Path,
    source_date_epoch: int,
    detach: bool = False,
) -> list[str]:
    """构造固定的 Docker argv；只挂载输入和结果父目录，不执行 bind-mount 脚本。"""
    document = _load_system_lock(system_lock)
    builder_image = document["builder_image"]
    assert isinstance(builder_image, dict)
    apt_records = document["apt_packages"]
    assert isinstance(apt_records, list)
    apt_specs = [f"{record['name']}={record['version']}" for record in apt_records]
    script = _container_script(
        apt_specs=apt_specs,
        output_name=output_root.name,
        source_archive_manifest=source_archive_manifest,
        dependency_lock=dependency_lock,
        source_cache_locks=source_cache_locks,
        source_date_epoch=source_date_epoch,
    )
    image = f"{builder_image['reference']}@{builder_image['digest']}"
    return [
        str(docker),
        "run",
        *( ["--detach"] if detach else [] ),
        "--rm",
        # Docker 默认 seccomp 拒绝 unshare；内层 wrapper 仍自行建立 user+netns 并写入证据。
        "--security-opt",
        "seccomp=unconfined",
        "--user",
        "0:0",
        "--mount",
        f"type=bind,src={_ROOT / 'packaging'},dst={_INPUT_ROOT / 'packaging'},readonly",
        "--mount",
        f"type=bind,src={_ROOT / 'scripts'},dst={_INPUT_ROOT / 'scripts'},readonly",
        "--mount",
        f"type=bind,src={source_archive_cache},dst={_INPUT_ROOT / 'source-cache'},readonly",
        "--mount",
        f"type=bind,src={output_root.parent},dst={_OUTPUT_PARENT}",
        image,
        "bash",
        "-ceu",
        script,
    ]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析完整的离线依赖构建输入，拒绝默认缓存、锁或结果目录。"""
    parser = argparse.ArgumentParser(
        description="Run the Stage 4 dependency build in a private Ubuntu 24.04 staging container."
    )
    parser.add_argument("--docker", type=Path, required=True)
    parser.add_argument("--system-lock", type=Path, required=True)
    parser.add_argument("--source-archive-cache", type=Path, required=True)
    parser.add_argument("--source-archive-manifest", type=Path, required=True)
    parser.add_argument("--dependency-lock", type=Path, required=True)
    parser.add_argument("--source-cache-lock", type=Path, action="append", default=[])
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--source-date-epoch", type=int, required=True)
    parser.add_argument(
        "--detach",
        action="store_true",
        help="launch the long-running container through the Docker daemon and print its ID",
    )
    parser.add_argument("--print-docker-command", action="store_true")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """在实际调用 Docker 前完成所有 fail-closed 输入与输出路径校验。"""
    args = parse_args(argv)
    try:
        if args.source_date_epoch < 0:
            raise ValueError("source-date-epoch must be nonnegative")
        docker = _require_executable(args.docker.absolute(), "docker")
        system_lock = _path_in_repository(args.system_lock.absolute(), "system-lock")
        source_archive_cache = _require_real_directory(
            args.source_archive_cache.absolute(), "source-archive-cache"
        )
        source_archive_manifest = _path_in_repository(
            args.source_archive_manifest.absolute(), "source-archive-manifest"
        )
        dependency_lock = _path_in_repository(
            args.dependency_lock.absolute(), "dependency-lock"
        )
        source_cache_locks = [
            _path_in_repository(path.absolute(), "source-cache-lock")
            for path in args.source_cache_lock
        ]
        output_root = _require_new_output_root(args.output_root.absolute())
        command = docker_command(
            docker=docker,
            system_lock=system_lock,
            source_archive_cache=source_archive_cache,
            source_archive_manifest=source_archive_manifest,
            dependency_lock=dependency_lock,
            source_cache_locks=source_cache_locks,
            output_root=output_root,
            source_date_epoch=args.source_date_epoch,
            detach=args.detach,
        )
    except (OSError, ValueError) as error:
        print(f"FAIL: {error}")
        return 1
    if args.print_docker_command:
        print(json.dumps({"command": command}, ensure_ascii=True))
        return 0
    if args.detach:
        result = subprocess.run(command, check=False, capture_output=True, text=True)
        if result.returncode != 0:
            sys.stderr.write(result.stderr)
            return result.returncode
        container_id = result.stdout.strip()
        if not re.fullmatch(r"[0-9a-f]{12,64}", container_id):
            print("FAIL: detached Docker launch did not return a container ID")
            return 1
        print(
            json.dumps(
                {"container_id": container_id, "output_root": str(output_root)},
                ensure_ascii=True,
            )
        )
        return 0
    result = subprocess.run(command, check=False)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
