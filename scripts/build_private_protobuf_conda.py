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

from stage4_source_archive import materialize_archive


PROTOBUF_SOURCE_SHA256 = "e825cac584256f88840ab6cf37add69ba0c6145811329d75642698a622d13498"
ABSEIL_SOURCE_SHA256 = "ed8f7d9f39139c449e79fd19765e23c96fdb774172d32d191323d3e3ea06e5ff"
RECIPE = Path(__file__).resolve().parents[1] / "packaging" / "recipes" / "protobuf-python"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析显式源码、构建和 channel 路径，禁止读取任意 checkout 或默认输出。"""
    parser = argparse.ArgumentParser(description="Build the private stage 4 Protobuf Conda package.")
    parser.add_argument("--source-archive", type=Path, required=True)
    parser.add_argument("--abseil-source-archive", type=Path, required=True)
    parser.add_argument("--work-root", type=Path, required=True)
    parser.add_argument("--channel-root", type=Path, required=True)
    parser.add_argument("--conda-build", type=Path, required=True)
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


def build_command(conda_build: Path, work_root: Path, channel_root: Path) -> list[str]:
    """构造不依赖 PATH、使用独立构建根目录的 Conda package 命令。"""
    return [
        str(conda_build),
        str(RECIPE),
        "--croot",
        str(work_root / "croot"),
        "--output-folder",
        str(channel_root),
    ]


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
            args.work_root.mkdir()
            source_root = args.work_root / "source" / materialize_archive(
                args.source_archive, args.work_root / "source"
            )
            abseil_root = args.work_root / "abseil-source" / materialize_archive(
                args.abseil_source_archive, args.work_root / "abseil-source"
            )
            producer_environment = {
                **os.environ,
                "STAGE4_PROTOBUF_SOURCE_DIR": str(source_root),
                "STAGE4_ABSEIL_SOURCE_DIR": str(abseil_root),
            }
            run_producer(
                build_command(args.conda_build, args.work_root, args.channel_root),
                producer_environment,
                args.work_root / "producer.log",
            )
            verify_private_package_output(args.channel_root)
            subprocess.run(
                [str(conda), "index", str(args.channel_root)],
                check=True,
                env=producer_environment,
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
