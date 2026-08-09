#!/usr/bin/env python3
# 阶段四私有 Protobuf Conda 包入口：只从冻结 v33.6 源码生成本地可审计制品。
from __future__ import annotations

import argparse
from collections.abc import Sequence
import hashlib
import json
import os
from pathlib import Path
import subprocess

from materialize_python_package_cache import materialize as materialize_package_cache
from materialize_python_package_channel import materialize as materialize_package_channel
from stage4_source_archive import materialize_archive


PROTOBUF_SOURCE_SHA256 = "e825cac584256f88840ab6cf37add69ba0c6145811329d75642698a622d13498"
ABSEIL_SOURCE_SHA256 = "ed8f7d9f39139c449e79fd19765e23c96fdb774172d32d191323d3e3ea06e5ff"
SOURCE_DATE_EPOCH = "0"
RECIPE = Path(__file__).resolve().parents[1] / "packaging" / "recipes" / "protobuf-python"
CANONICALIZER = Path(__file__).with_name("canonicalize_private_conda_package.py")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析显式源码、构建和 channel 路径，禁止读取任意 checkout 或默认输出。"""
    parser = argparse.ArgumentParser(description="Build the private stage 4 Protobuf Conda package.")
    parser.add_argument("--source-archive", type=Path, required=True)
    parser.add_argument("--abseil-source-archive", type=Path, required=True)
    parser.add_argument("--work-root", type=Path, required=True)
    parser.add_argument("--channel-root", type=Path, required=True)
    parser.add_argument("--conda-build", type=Path, required=True)
    parser.add_argument("--package-cache", type=Path)
    parser.add_argument("--package-cache-manifest", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check-only", action="store_true")
    mode.add_argument("--materialize-only", action="store_true")
    mode.add_argument("--print-build-command", action="store_true")
    return parser.parse_args(argv)


def verify_source_archive(path: Path, *, label: str, expected_sha256: str) -> None:
    """校验冻结源码归档的安全形态与摘要，禁止 builder 接受未锁定输入。"""
    metadata = path.lstat()
    if not path.is_file() or path.is_symlink() or metadata.st_nlink != 1:
        raise ValueError(f"{label} source archive must be a singly linked regular file")
    if hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha256:
        raise ValueError(f"{label} source archive sha256 differs from frozen dependency lock")


def verify_output_roots(work_root: Path, channel_root: Path) -> None:
    """要求本轮私有源码树和 local channel 都从全新目录开始。"""
    if work_root.exists() or channel_root.exists():
        raise ValueError("private protobuf work and channel roots must not already exist")


def verify_executable(path: Path, label: str) -> None:
    """拒绝 PATH 回退、链接或不可执行的固定 toolchain 可执行文件。"""
    if not path.exists():
        raise ValueError(f"{label} must be an executable regular file")
    metadata = path.lstat()
    if (
        not path.is_file()
        or path.is_symlink()
        or metadata.st_nlink != 1
        or not metadata.st_mode & 0o111
    ):
        raise ValueError(f"{label} must be an executable regular file")


def verify_conda_build(path: Path) -> None:
    """校验私有 package producer，保留调用方可审计的明确报错。"""
    verify_executable(path, "conda-build")


def verify_conda_python(path: Path) -> None:
    """允许 Conda 前缀内的 Python 链接，但固定其可执行普通文件目标。"""
    if not path.exists():
        raise ValueError("conda-build Python must be an executable regular file")
    resolved = path.resolve(strict=True)
    if resolved.parent != path.parent.resolve():
        raise ValueError("conda-build Python must resolve within the toolchain bin directory")
    metadata = resolved.lstat()
    if (
        not resolved.is_file()
        or resolved.is_symlink()
        or metadata.st_nlink != 1
        or not metadata.st_mode & 0o111
    ):
        raise ValueError("conda-build Python must be an executable regular file")


def verify_package_cache(path: Path) -> None:
    """只接受调用方明确给出的真实 Conda package cache 目录。"""
    if not path.is_dir() or path.is_symlink():
        raise ValueError("package cache must be a real directory")


def reproducible_compiler_flags(work_root: Path) -> str:
    """将每轮私有构建根映射到稳定虚拟路径，避免 ELF 泄漏临时目录。"""
    source = str(work_root)
    target = "/stage4/protobuf-work"
    return " ".join(
        f"-{kind}-prefix-map={source}={target}"
        for kind in ("ffile", "fdebug", "fmacro")
    )


def producer_environment(work_root: Path) -> dict[str, str]:
    """移除宿主求解与网络配置，只保留本轮显式指定的 Conda 输入。"""
    environment = dict(os.environ)
    for name in tuple(environment):
        if (
            name.startswith(("CONDA_", "MAMBA_", "PIP_"))
            or name.lower()
            in {
                "all_proxy",
                "cflags",
                "cppflags",
                "cxxflags",
                "http_proxy",
                "https_proxy",
                "ldflags",
                "no_proxy",
            }
        ):
            environment.pop(name)
    environment["HOME"] = str(work_root / "empty-home")
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    environment["SOURCE_DATE_EPOCH"] = SOURCE_DATE_EPOCH
    flags = reproducible_compiler_flags(work_root)
    environment["CFLAGS"] = flags
    environment["CXXFLAGS"] = flags
    return environment


def build_command(
    conda_build: Path,
    work_root: Path,
    channel_root: Path,
    package_channel: Path | None = None,
) -> list[str]:
    """构造不依赖 PATH、使用独立构建根目录的 Conda package 命令。"""
    command = [
        str(conda_build),
        str(RECIPE),
        "--croot",
        str(work_root / "croot"),
        "--output-folder",
        str(channel_root),
    ]
    if package_channel is not None:
        command.extend(
            ["--override-channels", "--channel", package_channel.as_uri()]
        )
    return command


def verify_private_package_output(channel_root: Path) -> Path:
    """确认 producer 在 local channel 中只写出一个安全的目标 Protobuf 制品。"""
    packages = sorted(
        (channel_root / "linux-64").glob("protobuf-6.33.6-*.conda")
    )
    if len(packages) != 1:
        raise ValueError("private Protobuf producer did not create exactly one package")
    package = packages[0]
    metadata = package.lstat()
    if not package.is_file() or package.is_symlink() or metadata.st_nlink != 1:
        raise ValueError("private Protobuf producer did not create exactly one package")
    return package


def canonicalize_private_package(
    conda_build: Path,
    package: Path,
    work_root: Path,
    environment: dict[str, str],
) -> Path:
    """用同一锁定 toolchain 重封装 volatile metadata，再原子替换本轮输出包。"""
    python = conda_build.with_name("python")
    verify_conda_python(python)
    if not CANONICALIZER.is_file() or CANONICALIZER.is_symlink():
        raise ValueError("private Conda canonicalizer must be a regular file")
    canonical_root = work_root / "canonical-package"
    canonical_root.mkdir(mode=0o700)
    canonical_output = canonical_root / package.name
    command = [
        str(python),
        str(CANONICALIZER),
        "--input",
        str(package),
        "--output",
        str(canonical_output),
    ]
    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        env=environment,
        text=True,
    )
    (work_root / "canonicalizer.log").write_text(
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}\n",
        encoding="utf-8",
    )
    if result.returncode != 0:
        raise subprocess.CalledProcessError(
            result.returncode,
            command,
            output=result.stdout,
            stderr=result.stderr,
        )
    metadata = canonical_output.lstat()
    if (
        not canonical_output.is_file()
        or canonical_output.is_symlink()
        or metadata.st_nlink != 1
    ):
        raise ValueError("private Conda canonicalizer did not create a regular package")
    replacement = package.with_name(f".{package.name}.canonical")
    if replacement.exists():
        raise ValueError("private Conda package replacement path already exists")
    with canonical_output.open("rb") as source, replacement.open("xb") as destination:
        while chunk := source.read(1024 * 1024):
            destination.write(chunk)
        destination.flush()
        os.fsync(destination.fileno())
    replacement.chmod(0o644)
    replacement.replace(package)
    canonical_output.unlink()
    return verify_private_package_output(package.parents[1])


def run_producer(command: Sequence[str], environment: dict[str, str], log_path: Path) -> None:
    """执行外部 Conda producer，并把每轮标准输出与错误输出固定为工作证据。"""
    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        env=environment,
        text=True,
    )
    log_path.write_text(
        f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}\n",
        encoding="utf-8",
    )
    if result.returncode != 0:
        raise subprocess.CalledProcessError(
            result.returncode,
            command,
            output=result.stdout,
            stderr=result.stderr,
        )


def main(argv: Sequence[str] | None = None) -> int:
    """从冻结源码执行私有 Protobuf Conda producer，并保持输出根目录独立。"""
    args = parse_args(argv)
    args.source_archive = Path(os.path.abspath(args.source_archive))
    args.abseil_source_archive = Path(os.path.abspath(args.abseil_source_archive))
    args.work_root = Path(os.path.abspath(args.work_root))
    args.channel_root = Path(os.path.abspath(args.channel_root))
    if args.package_cache is not None:
        args.package_cache = Path(os.path.abspath(args.package_cache))
    if args.package_cache_manifest is not None:
        args.package_cache_manifest = Path(os.path.abspath(args.package_cache_manifest))
    try:
        verify_source_archive(
            args.source_archive,
            label="protobuf",
            expected_sha256=PROTOBUF_SOURCE_SHA256,
        )
        verify_source_archive(
            args.abseil_source_archive,
            label="abseil",
            expected_sha256=ABSEIL_SOURCE_SHA256,
        )
        verify_output_roots(args.work_root, args.channel_root)
        if args.materialize_only:
            args.work_root.mkdir()
            root = materialize_archive(args.source_archive, args.work_root / "source")
            materialize_archive(args.abseil_source_archive, args.work_root / "abseil-source")
            print(f"PASS: frozen Protobuf source materialized at {args.work_root / 'source' / root}")
            return 0
        if args.print_build_command:
            verify_conda_build(args.conda_build)
            print(
                json.dumps(
                    {
                        "recipe_metadata": str(RECIPE / "meta.yaml"),
                        "command": build_command(args.conda_build, args.work_root, args.channel_root),
                    }
                )
            )
            return 0
        if not args.check_only:
            verify_conda_build(args.conda_build)
            conda = args.conda_build.with_name("conda")
            verify_executable(conda, "conda")
            if (args.package_cache is None) != (args.package_cache_manifest is None):
                raise ValueError("package cache and manifest must be provided together")
            args.work_root.mkdir()
            source_root = args.work_root / "source" / materialize_archive(
                args.source_archive, args.work_root / "source"
            )
            abseil_root = args.work_root / "abseil-source" / materialize_archive(
                args.abseil_source_archive, args.work_root / "abseil-source"
            )
            (args.work_root / "empty-home").mkdir(mode=0o700)
            environment = producer_environment(args.work_root)
            environment.update(
                {
                "STAGE4_PROTOBUF_SOURCE_DIR": str(source_root),
                "STAGE4_ABSEIL_SOURCE_DIR": str(abseil_root),
                }
            )
            if args.package_cache is not None:
                verify_package_cache(args.package_cache)
                native_cache = args.work_root / "native-conda-pkgs"
                materialize_package_cache(
                    args.package_cache_manifest,
                    args.package_cache,
                    native_cache,
                    args.work_root / "native-conda-pkgs.json",
                )
                package_channel = args.work_root / "private-conda-channel"
                materialize_package_channel(
                    args.package_cache_manifest,
                    args.package_cache,
                    package_channel,
                    args.work_root / "private-conda-channel.json",
                )
                condarc = args.work_root / "producer.condarc"
                condarc.write_text(
                    "channels:\n"
                    f"  - {package_channel.as_uri()}\n"
                    "channel_priority: strict\n"
                    "offline: true\n",
                    encoding="utf-8",
                )
                environment.update(
                    {
                        "CONDA_OFFLINE": "true",
                        "CONDA_PKGS_DIRS": str(native_cache),
                        "CONDARC": str(condarc),
                    }
                )
                subprocess.run(
                    [str(conda), "index", str(package_channel)],
                    check=True,
                    env=environment,
                )
            run_producer(
                build_command(
                    args.conda_build,
                    args.work_root,
                    args.channel_root,
                    package_channel if args.package_cache is not None else None,
                ),
                environment,
                args.work_root / "producer.log",
            )
            package = verify_private_package_output(args.channel_root)
            canonicalize_private_package(
                args.conda_build,
                package,
                args.work_root,
                environment,
            )
            subprocess.run(
                [str(conda), "index", str(args.channel_root)],
                check=True,
                env=environment,
            )
            print(f"PASS: private Protobuf Conda producer completed at {args.channel_root}")
            return 0
        print("PASS: frozen Protobuf v33.6 source archive verified")
        return 0
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
