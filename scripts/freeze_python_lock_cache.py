#!/usr/bin/env python3
# 阶段四 Python 锁/cache 联网 producer：所有下载前先验证固定工具链输入。
from __future__ import annotations

import argparse
from collections.abc import Sequence
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
from urllib.parse import urlsplit

from verify_python_lock_cache import (
    _archive_digests,
    _canonical_archive_relative_path,
    _expected_cache_records,
    _unified_conda_records,
    _verify_lock_pair,
)


MICROMAMBA_SHA256 = "77b7790ec97f64581118f103585b175df4306f95829b0fa6bfe4a19cc88a1182"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析 producer 的显式工具输入，禁止回退到调用机 PATH。"""
    parser = argparse.ArgumentParser(description="Freeze stage 4 Python locks and cache inputs.")
    parser.add_argument("--micromamba", type=Path, required=True)
    parser.add_argument("--lock-env", type=Path)
    parser.add_argument("--runtime-spec", type=Path)
    parser.add_argument("--toolchain-spec", type=Path)
    parser.add_argument("--virtual-packages", type=Path)
    parser.add_argument("--runtime-unified", type=Path)
    parser.add_argument("--runtime-explicit", type=Path)
    parser.add_argument("--toolchain-unified", type=Path)
    parser.add_argument("--toolchain-explicit", type=Path)
    parser.add_argument("--protobuf-build-spec", type=Path)
    parser.add_argument("--protobuf-build-unified", type=Path)
    parser.add_argument("--protobuf-build-explicit", type=Path)
    parser.add_argument("--seed", type=Path)
    parser.add_argument("--cache-root", type=Path)
    parser.add_argument("--cache-manifest", type=Path)
    parser.add_argument("--ca-bundle", type=Path)
    parser.add_argument("--check-only", action="store_true")
    return parser.parse_args(argv)


def verify_micromamba(path: Path) -> None:
    """复算唯一允许的 micromamba ELF 摘要，阻断未固定 producer 工具。"""
    metadata = path.lstat()
    if not path.is_file() or path.is_symlink() or metadata.st_nlink != 1:
        raise ValueError("micromamba must be a singly linked regular file")
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != MICROMAMBA_SHA256:
        raise ValueError("micromamba sha256 differs from pinned toolchain")


def _require_producer_paths(args: argparse.Namespace) -> tuple[Path, ...]:
    """要求正常 producer 的每个输入和输出都由调用方显式指定。"""
    names = (
        "lock_env",
        "runtime_spec",
        "toolchain_spec",
        "virtual_packages",
        "runtime_unified",
        "runtime_explicit",
        "toolchain_unified",
        "toolchain_explicit",
    )
    paths = tuple(getattr(args, name) for name in names)
    if any(path is None for path in paths):
        raise ValueError("lock producer requires explicit lock, spec, and output paths")
    return tuple(Path(path).resolve() for path in paths)


def _require_protobuf_build_paths(args: argparse.Namespace) -> tuple[Path, Path, Path] | None:
    """接受完整的私有 Protobuf build 锁三元组，拒绝半配置输入。"""
    paths = (
        args.protobuf_build_spec,
        args.protobuf_build_unified,
        args.protobuf_build_explicit,
    )
    if all(path is None for path in paths):
        return None
    if any(path is None for path in paths):
        raise ValueError("protobuf build spec and lock outputs must be supplied together")
    return tuple(Path(path).resolve() for path in paths)  # type: ignore[return-value]


def _verify_conda_lock(lock_env: Path) -> Path:
    """锁定工具只能来自已验证的独立 tool environment。"""
    conda_lock = lock_env / "bin" / "conda-lock"
    metadata = conda_lock.lstat()
    if (
        not conda_lock.is_file()
        or conda_lock.is_symlink()
        or metadata.st_nlink != 1
        or not metadata.st_mode & 0o111
    ):
        raise ValueError("lock environment must provide an executable regular conda-lock")
    return conda_lock


def _verify_lock_output_paths(paths: tuple[Path, ...]) -> None:
    """锁文件必须写入彼此独立、尚未存在的调用方路径。"""
    if len(set(paths)) != len(paths):
        raise ValueError("runtime and toolchain lock outputs must be distinct")
    for path in paths:
        if path.exists() or not path.parent.is_dir():
            raise ValueError("lock output paths must be absent under existing directories")


def _require_cache_outputs(args: argparse.Namespace) -> tuple[Path, Path, Path] | None:
    """要求下载 seed 与 canonical cache 三个输出成对出现且均为全新路径。"""
    supplied = (args.seed, args.cache_root, args.cache_manifest)
    if all(path is None for path in supplied):
        return None
    if any(path is None for path in supplied):
        raise ValueError("seed, cache root, and cache manifest must be supplied together")
    seed, cache_root, manifest_path = (Path(path).resolve() for path in supplied)
    if len({seed, cache_root, manifest_path}) != 3:
        raise ValueError("seed and canonical cache outputs must be distinct")
    for output in (seed, cache_root, manifest_path):
        if output.exists() or not output.parent.is_dir():
            raise ValueError("seed and canonical cache outputs must be absent under existing directories")
    return seed, cache_root, manifest_path


def _require_ca_bundle(args: argparse.Namespace) -> Path:
    """下载私有 channel 前验证调用方显式提供的 CA bundle。"""
    if args.ca_bundle is None:
        raise ValueError("cache producer requires an explicit CA bundle")
    ca_bundle = Path(args.ca_bundle).resolve()
    metadata = ca_bundle.lstat()
    if (
        not ca_bundle.is_file()
        or ca_bundle.is_symlink()
        or not stat.S_ISREG(metadata.st_mode)
        or metadata.st_nlink != 1
    ):
        raise ValueError("CA bundle must be a singly linked regular file")
    return ca_bundle


def _render_template(explicit_lock: Path) -> Path:
    """把唯一 linux-64 输出名还原为 conda-lock 所需的平台模板。"""
    if explicit_lock.name.count("linux-64") != 1:
        raise ValueError("explicit lock output name must contain linux-64 exactly once")
    return explicit_lock.with_name(explicit_lock.name.replace("linux-64", "{platform}"))


def _lock_commands(
    *,
    conda_lock: Path,
    micromamba: Path,
    runtime_spec: Path,
    toolchain_spec: Path,
    virtual_packages: Path,
    runtime_unified: Path,
    runtime_explicit: Path,
    toolchain_unified: Path,
    toolchain_explicit: Path,
    protobuf_build_spec: Path | None = None,
    protobuf_build_unified: Path | None = None,
    protobuf_build_explicit: Path | None = None,
) -> tuple[tuple[str, ...], ...]:
    """构造计划冻结的四条 argv，避免 shell 拼接或 PATH 回退。"""
    common_lock = (
        str(conda_lock),
        "lock",
        "--conda",
        str(micromamba),
        "--no-mamba",
        "--no-micromamba",
    )
    common_render = (
        str(conda_lock),
        "render",
        "--kind",
        "explicit",
        "--platform",
        "linux-64",
        "--no-dev-dependencies",
        "--filename-template",
    )
    commands = (
        common_lock
        + (
            "--file",
            str(runtime_spec),
            "--platform",
            "linux-64",
            "--kind",
            "lock",
            "--lockfile",
            str(runtime_unified),
            "--virtual-package-spec",
            str(virtual_packages),
            "--no-dev-dependencies",
        ),
        common_render + (str(_render_template(runtime_explicit)), str(runtime_unified)),
        common_lock
        + (
            "--file",
            str(toolchain_spec),
            "--platform",
            "linux-64",
            "--kind",
            "lock",
            "--lockfile",
            str(toolchain_unified),
            "--virtual-package-spec",
            str(virtual_packages),
            "--no-dev-dependencies",
        ),
        common_render + (str(_render_template(toolchain_explicit)), str(toolchain_unified)),
    )
    protobuf_paths = (
        protobuf_build_spec,
        protobuf_build_unified,
        protobuf_build_explicit,
    )
    if all(path is None for path in protobuf_paths):
        return commands
    if any(path is None for path in protobuf_paths):
        raise ValueError("protobuf build lock commands require complete paths")
    return commands + (
        common_lock
        + (
            "--file",
            str(protobuf_build_spec),
            "--platform",
            "linux-64",
            "--kind",
            "lock",
            "--lockfile",
            str(protobuf_build_unified),
            "--virtual-package-spec",
            str(virtual_packages),
            "--no-dev-dependencies",
        ),
        common_render
        + (
            str(_render_template(protobuf_build_explicit)),
            str(protobuf_build_unified),
        ),
    )


def _download_commands(
    *,
    micromamba: Path,
    ca_bundle: Path,
    seed: Path,
    runtime_explicit: Path,
    toolchain_explicit: Path,
    protobuf_build_explicit: Path | None = None,
) -> tuple[tuple[str, ...], ...]:
    """构造显式锁的隔离 download-only argv，禁止使用用户 cache。"""
    common = (
        str(micromamba),
        "create",
        "--no-rc",
        "--no-env",
        "--ssl-verify",
        str(ca_bundle),
        "--root-prefix",
        str(seed / "mamba-root"),
    )
    commands = (
        common
        + (
            "--prefix",
            str(seed / "runtime-download-prefix"),
            "--file",
            str(runtime_explicit),
            "--download-only",
            "--safety-checks",
            "enabled",
            "--yes",
        ),
        common
        + (
            "--prefix",
            str(seed / "toolchain-download-prefix"),
            "--file",
            str(toolchain_explicit),
            "--download-only",
            "--safety-checks",
            "enabled",
            "--yes",
        ),
    )
    if protobuf_build_explicit is None:
        return commands
    return commands + (
        common
        + (
            "--prefix",
            str(seed / "protobuf-build-download-prefix"),
            "--file",
            str(protobuf_build_explicit),
            "--download-only",
            "--safety-checks",
            "enabled",
            "--yes",
        ),
    )


def _seed_archive_path(seed_packages: Path, url: str) -> Path:
    """按 micromamba 2 的 URL 嵌套规则定位本轮下载归档。"""
    parsed = urlsplit(url)
    parts = tuple(part for part in parsed.path.split("/") if part)
    if (
        parsed.scheme != "https"
        or not parsed.netloc
        or parsed.query
        or parsed.fragment
        or not parts
        or any(part in {".", ".."} for part in parts)
    ):
        raise ValueError("seed archive URL must be a plain HTTPS package URL")
    return seed_packages.joinpath(
        parsed.scheme,
        parsed.netloc.replace(":", "_"),
        *parts,
    )


def _write_canonical_cache(
    *,
    seed: Path,
    cache_root: Path,
    manifest_path: Path,
    records: dict[str, tuple[str, str, frozenset[str]]],
) -> None:
    """从本轮 root 级下载归档重建只读 canonical cache 与 JSON 清单。"""
    if cache_root.exists() or manifest_path.exists():
        raise ValueError("canonical cache outputs must be absent")
    if not cache_root.parent.is_dir() or not manifest_path.parent.is_dir():
        raise ValueError("canonical cache output parents must exist")
    seed_packages = seed / "mamba-root" / "pkgs"
    if not seed_packages.is_dir() or seed_packages.is_symlink():
        raise ValueError("seed must contain a real mamba-root package cache")

    filename_hashes: dict[str, tuple[str, str]] = {}
    for url, (md5, sha256, _locks) in records.items():
        _relative_path, filename = _canonical_archive_relative_path(url)
        previous = filename_hashes.setdefault(filename, (md5, sha256))
        if previous != (md5, sha256):
            raise ValueError("seed archive basename maps to distinct lock hashes")

    cache_root.mkdir(mode=0o755)
    manifest_archives: list[dict[str, object]] = []
    for url in sorted(records):
        md5, sha256, locks = records[url]
        relative_path, filename = _canonical_archive_relative_path(url)
        source = _seed_archive_path(seed_packages, url)
        source_stat = source.lstat()
        if (
            not stat.S_ISREG(source_stat.st_mode)
            or stat.S_ISLNK(source_stat.st_mode)
            or source_stat.st_nlink != 1
        ):
            raise ValueError("seed package archive must be a singly linked regular file")
        size, actual_md5, actual_sha256 = _archive_digests(source)
        if actual_md5 != md5 or actual_sha256 != sha256:
            raise ValueError("seed package archive hashes differ from unified lock")

        target = cache_root / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.parent.chmod(0o755)
        with source.open("rb") as reader, target.open("xb") as writer:
            while chunk := reader.read(1024 * 1024):
                writer.write(chunk)
        target.chmod(0o644)
        manifest_archives.append(
            {
                "url": url,
                "filename": filename,
                "relative_path": relative_path,
                "size": size,
                "md5": md5,
                "sha256": sha256,
                "locks": sorted(locks),
            }
        )

    urls_txt = cache_root / "pkgs" / "urls.txt"
    urls_txt.parent.mkdir(parents=True, exist_ok=True)
    urls_txt.parent.chmod(0o755)
    urls_txt.write_text("\n".join(sorted(records)) + "\n", encoding="utf-8")
    urls_txt.chmod(0o644)
    manifest = {
        "schema_version": 1,
        "archives": manifest_archives,
        "urls_txt_sha256": hashlib.sha256(urls_txt.read_bytes()).hexdigest(),
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    manifest_path.chmod(0o644)


def main(argv: Sequence[str] | None = None) -> int:
    """验证固定工具后生成两组 Conda lock 与 explicit render。"""
    args = parse_args(argv)
    try:
        micromamba = args.micromamba.resolve()
        verify_micromamba(micromamba)
        if not args.check_only:
            (
                lock_env,
                runtime_spec,
                toolchain_spec,
                virtual_packages,
                runtime_unified,
                runtime_explicit,
                toolchain_unified,
                toolchain_explicit,
            ) = _require_producer_paths(args)
            protobuf_build_paths = _require_protobuf_build_paths(args)
            cache_outputs = _require_cache_outputs(args)
            conda_lock = _verify_conda_lock(lock_env)
            ca_bundle = (
                _require_ca_bundle(args) if args.ca_bundle is not None else None
            )
            producer_env = dict(os.environ)
            if ca_bundle is not None:
                # conda-lock 的求解器会另起 micromamba，需显式传递同一 CA。
                producer_env["MAMBA_SSL_VERIFY"] = str(ca_bundle)
            output_paths = (runtime_unified, runtime_explicit, toolchain_unified, toolchain_explicit)
            if protobuf_build_paths is not None:
                output_paths += protobuf_build_paths[1:]
            _verify_lock_output_paths(output_paths)
            for command in _lock_commands(
                conda_lock=conda_lock,
                micromamba=micromamba,
                runtime_spec=runtime_spec,
                toolchain_spec=toolchain_spec,
                virtual_packages=virtual_packages,
                runtime_unified=runtime_unified,
                runtime_explicit=runtime_explicit,
                toolchain_unified=toolchain_unified,
                toolchain_explicit=toolchain_explicit,
                protobuf_build_spec=(
                    None if protobuf_build_paths is None else protobuf_build_paths[0]
                ),
                protobuf_build_unified=(
                    None if protobuf_build_paths is None else protobuf_build_paths[1]
                ),
                protobuf_build_explicit=(
                    None if protobuf_build_paths is None else protobuf_build_paths[2]
                ),
            ):
                subprocess.run(command, check=True, env=producer_env)
            if cache_outputs is None:
                print("PASS: pinned runtime and toolchain Conda locks rendered")
                return 0
            if ca_bundle is None:
                ca_bundle = _require_ca_bundle(args)
            seed, cache_root, manifest_path = cache_outputs
            seed.mkdir(mode=0o755)
            for command in _download_commands(
                micromamba=micromamba,
                ca_bundle=ca_bundle,
                seed=seed,
                runtime_explicit=runtime_explicit,
                toolchain_explicit=toolchain_explicit,
                protobuf_build_explicit=(
                    None if protobuf_build_paths is None else protobuf_build_paths[2]
                ),
            ):
                subprocess.run(command, check=True, env=producer_env)
            _verify_lock_pair(runtime_unified, runtime_explicit, "runtime")
            _verify_lock_pair(toolchain_unified, toolchain_explicit, "toolchain")
            protobuf_build_records = None
            if protobuf_build_paths is not None:
                _verify_lock_pair(
                    protobuf_build_paths[1], protobuf_build_paths[2], "protobuf build"
                )
                protobuf_build_records = _unified_conda_records(
                    protobuf_build_paths[1], "protobuf build"
                )
            records = _expected_cache_records(
                _unified_conda_records(runtime_unified, "runtime"),
                _unified_conda_records(toolchain_unified, "toolchain"),
                protobuf_build_records=protobuf_build_records,
            )
            _write_canonical_cache(
                seed=seed,
                cache_root=cache_root,
                manifest_path=manifest_path,
                records=records,
            )
            print("PASS: pinned Conda locks and canonical package cache rendered")
            return 0
        print("PASS: pinned micromamba verified")
        return 0
    except (OSError, ValueError) as error:
        print(f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
