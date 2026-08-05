#!/usr/bin/env python3
# 阶段四 Python 锁与缓存验证入口：先严格检查人工 Conda 声明，再扩展到冻结产物。
from __future__ import annotations

import argparse
from collections.abc import Sequence
import hashlib
import json
from pathlib import Path
import stat
from urllib.parse import urlsplit

import yaml


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析显式静态输入，禁止 verifier 读取调用主机的 Conda 配置。"""
    parser = argparse.ArgumentParser(description="Verify stage 4 Python lock and cache inputs.")
    parser.add_argument("--runtime-spec", type=Path, required=True)
    parser.add_argument("--toolchain-spec", type=Path, required=True)
    parser.add_argument("--virtual-packages", type=Path, required=True)
    parser.add_argument("--runtime-unified", type=Path)
    parser.add_argument("--runtime-explicit", type=Path)
    parser.add_argument("--toolchain-unified", type=Path)
    parser.add_argument("--toolchain-explicit", type=Path)
    parser.add_argument("--cache-manifest", type=Path)
    parser.add_argument("--cache-root", type=Path)
    parser.add_argument("--static-only", action="store_true")
    return parser.parse_args(argv)


def _load_mapping(path: Path, label: str) -> dict[str, object]:
    """加载非空 YAML 映射，拒绝缺失和隐式顶层类型。"""
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not document:
        raise ValueError(f"{label} must be a nonempty YAML mapping")
    return document


def _dependencies(document: dict[str, object], label: str) -> tuple[object, ...]:
    """提取环境依赖并拒绝空列表，避免锁生成使用不完整声明。"""
    dependencies = document.get("dependencies")
    if not isinstance(dependencies, list) or not dependencies:
        raise ValueError(f"{label} dependencies must be a nonempty list")
    return tuple(dependencies)


def _verify_runtime_spec(document: dict[str, object]) -> None:
    """固定生产 runtime 为纯 Conda 依赖，wheel 只能在 conda-pack 后安装。"""
    dependencies = _dependencies(document, "runtime")
    if not all(isinstance(item, str) for item in dependencies):
        raise ValueError("runtime dependencies must not contain pip entries")
    required = {"python=3.10", "protobuf=6.33.6", "packaging"}
    if not required.issubset(dependencies):
        raise ValueError("runtime dependencies are missing required Conda packages")


def _verify_toolchain_spec(document: dict[str, object]) -> None:
    """确认 toolchain 是独立 Conda 环境，并包含构建与打包工具。"""
    dependencies = _dependencies(document, "toolchain")
    if not all(isinstance(item, str) for item in dependencies):
        raise ValueError("toolchain dependencies must be Conda package specifications")
    required = {
        "python=3.10",
        "python-build",
        "conda-build",
        "pip",
        "conda-lock=4.0.2",
        "conda-pack=0.9.2",
    }
    if not required.issubset(dependencies):
        raise ValueError("toolchain dependencies are missing required Conda packages")


def _verify_virtual_packages(document: dict[str, object]) -> None:
    """校验锁解析采用仓库内固定 linux-64 virtual package 视图。"""
    subdirs = document.get("subdirs")
    if not isinstance(subdirs, dict):
        raise ValueError("virtual package spec must define subdirs")
    linux = subdirs.get("linux-64")
    if not isinstance(linux, dict) or not isinstance(linux.get("packages"), dict):
        raise ValueError("virtual package spec must define linux-64 packages")
    packages = linux["packages"]
    if not {"__archspec", "__glibc", "__linux", "__unix"}.issubset(packages):
        raise ValueError("virtual package spec is missing required linux-64 packages")


def _is_hex_digest(value: object, length: int) -> bool:
    """限制锁摘要为固定长度十六进制，避免将缺失或非摘要字段带入缓存。"""
    return (
        isinstance(value, str)
        and len(value) == length
        and all(character in "0123456789abcdef" for character in value.lower())
    )


def _is_https_url(value: object) -> bool:
    """锁中的下载地址必须是无 fragment 的绝对 HTTPS URL。"""
    if not isinstance(value, str):
        return False
    parsed = urlsplit(value)
    return parsed.scheme == "https" and bool(parsed.netloc) and not parsed.fragment


def _unified_conda_records(path: Path, label: str) -> dict[str, tuple[str, str]]:
    """读取 linux-64 unified Conda 记录，固定 URL 到完整摘要的唯一映射。"""
    document = _load_mapping(path, f"{label} unified lock")
    packages = document.get("package")
    if not isinstance(packages, list) or not packages:
        raise ValueError(f"{label} unified lock must contain Conda packages")
    records: dict[str, tuple[str, str]] = {}
    for package in packages:
        if not isinstance(package, dict) or package.get("manager") != "conda":
            raise ValueError(f"{label} unified lock must contain only Conda packages")
        if package.get("platform") != "linux-64":
            raise ValueError(f"{label} unified lock must contain only linux-64 packages")
        url = package.get("url")
        hashes = package.get("hash")
        if not _is_https_url(url):
            raise ValueError(f"{label} unified lock package URL must use HTTPS")
        if not isinstance(hashes, dict) or not _is_hex_digest(hashes.get("md5"), 32):
            raise ValueError(f"{label} unified lock package MD5 must be a 32-digit hex digest")
        if not _is_hex_digest(hashes.get("sha256"), 64):
            raise ValueError(f"{label} unified lock package SHA-256 must be a 64-digit hex digest")
        if url in records:
            raise ValueError(f"{label} unified lock contains duplicate package URLs")
        records[url] = (hashes["md5"], hashes["sha256"])
    return records


def _explicit_conda_records(path: Path, label: str) -> dict[str, str]:
    """解析 explicit render，拒绝 pip 段、非 HTTPS 记录和重复 URL。"""
    records: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line == "@EXPLICIT":
            continue
        if line.lower().startswith("# pip"):
            raise ValueError(f"{label} explicit lock must not contain pip entries")
        if line.startswith("#"):
            continue
        url, separator, md5 = line.rpartition("#")
        if not separator or not _is_https_url(url) or not _is_hex_digest(md5, 32):
            raise ValueError(f"{label} explicit lock entry must use HTTPS URL#MD5")
        if url in records:
            raise ValueError(f"{label} explicit lock contains duplicate package URLs")
        records[url] = md5
    if not records:
        raise ValueError(f"{label} explicit lock must contain Conda package URLs")
    return records


def _verify_lock_pair(unified_path: Path, explicit_path: Path, label: str) -> None:
    """逐项比对 unified 与 explicit 锁，阻断 renderer URL 或 MD5 漂移。"""
    unified_records = _unified_conda_records(unified_path, label)
    explicit_records = _explicit_conda_records(explicit_path, label)
    for url, explicit_md5 in explicit_records.items():
        unified_hashes = unified_records.get(url)
        if unified_hashes is None:
            raise ValueError("explicit lock URL differs from unified record")
        if explicit_md5 != unified_hashes[0]:
            raise ValueError("explicit lock MD5 differs from unified record")
    if set(explicit_records) != set(unified_records):
        raise ValueError("explicit lock packages differ from unified records")


def _require_frozen_lock_paths(args: argparse.Namespace) -> tuple[Path, Path, Path, Path]:
    """仅在非静态模式接收四份明确锁文件，避免按目录或默认名猜测。"""
    names = (
        "runtime_unified",
        "runtime_explicit",
        "toolchain_unified",
        "toolchain_explicit",
    )
    paths = tuple(getattr(args, name) for name in names)
    if any(path is None for path in paths):
        raise ValueError("frozen lock verification requires all unified and explicit lock paths")
    return tuple(Path(path) for path in paths)  # type: ignore[arg-type,return-value]


def _canonical_archive_relative_path(url: str) -> tuple[str, str]:
    """由已验证 HTTPS URL 唯一推导 canonical cache 的 archive 相对路径。"""
    parsed = urlsplit(url)
    parts = tuple(part for part in parsed.path.split("/") if part)
    if not parts or any(part in {".", ".."} for part in parts):
        raise ValueError("unified lock package URL has an invalid archive path")
    relative_path = "/".join(("pkgs", "https", parsed.netloc, *parts))
    return relative_path, parts[-1]


def _archive_digests(path: Path) -> tuple[int, str, str]:
    """流式复算 package archive 摘要，避免把大包完整读入 verifier 内存。"""
    md5 = hashlib.md5()
    sha256 = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            size += len(chunk)
            md5.update(chunk)
            sha256.update(chunk)
    return size, md5.hexdigest(), sha256.hexdigest()


def _expected_cache_records(
    runtime_records: dict[str, tuple[str, str]],
    toolchain_records: dict[str, tuple[str, str]],
) -> dict[str, tuple[str, str, frozenset[str]]]:
    """合并两份 unified lock，并拒绝同 URL 的不一致摘要。"""
    expected: dict[str, tuple[str, str, frozenset[str]]] = {}
    for label, records in (("runtime", runtime_records), ("toolchain", toolchain_records)):
        for url, (md5, sha256) in records.items():
            previous = expected.get(url)
            if previous is not None and previous[:2] != (md5, sha256):
                raise ValueError("unified locks disagree on a shared package URL")
            locks = frozenset((label,)) if previous is None else previous[2] | {label}
            expected[url] = (md5, sha256, locks)
    return expected


def _cache_file_census(cache_root: Path) -> tuple[set[str], set[str]]:
    """枚举 cache 的普通文件和目录，拒绝链接、特殊节点与硬链接文件。"""
    root_stat = cache_root.lstat()
    if not stat.S_ISDIR(root_stat.st_mode) or stat.S_ISLNK(root_stat.st_mode):
        raise ValueError("package cache root must be a real directory")
    files: set[str] = set()
    directories: set[str] = set()
    for member in cache_root.rglob("*"):
        member_stat = member.lstat()
        relative_path = member.relative_to(cache_root).as_posix()
        if stat.S_ISLNK(member_stat.st_mode):
            raise ValueError("package cache must not contain links")
        if stat.S_ISDIR(member_stat.st_mode):
            directories.add(relative_path)
            continue
        if not stat.S_ISREG(member_stat.st_mode) or member_stat.st_nlink != 1:
            raise ValueError("package cache must contain only single-link regular files")
        files.add(relative_path)
    return files, directories


def _expected_cache_directories(files: set[str]) -> set[str]:
    """从允许文件集合推导唯一目录树，防止 cache 留下空目录或扁平 fallback。"""
    directories: set[str] = set()
    for filename in files:
        parent = Path(filename).parent
        while parent != Path("."):
            directories.add(parent.as_posix())
            parent = parent.parent
    return directories


def _verify_cache_manifest(
    manifest_path: Path,
    cache_root: Path,
    runtime_records: dict[str, tuple[str, str]],
    toolchain_records: dict[str, tuple[str, str]],
) -> None:
    """校验 canonical cache 清单、锁身份与 archive bytes 一一对应。"""
    document = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("package cache manifest must use schema_version 1")
    archives = document.get("archives")
    if not isinstance(archives, list):
        raise ValueError("package cache manifest must contain archives")
    expected = _expected_cache_records(runtime_records, toolchain_records)
    seen_urls: set[str] = set()
    seen_paths: set[str] = set()
    for record in archives:
        if not isinstance(record, dict):
            raise ValueError("package cache manifest archive must be a mapping")
        url = record.get("url")
        if not _is_https_url(url) or url in seen_urls or url not in expected:
            raise ValueError("package cache manifest archives do not match unified locks")
        seen_urls.add(url)
        relative_path, filename = _canonical_archive_relative_path(url)
        md5, sha256, locks = expected[url]
        if record.get("filename") != filename or record.get("relative_path") != relative_path:
            raise ValueError("package cache manifest uses a noncanonical archive path")
        if record.get("md5") != md5 or record.get("sha256") != sha256:
            raise ValueError("package cache manifest archive hashes differ from unified lock")
        if record.get("locks") != sorted(locks):
            raise ValueError("package cache manifest archive locks differ from unified lock")
        size = record.get("size")
        if not isinstance(size, int) or isinstance(size, bool) or size < 0:
            raise ValueError("package cache manifest archive size must be a nonnegative integer")
        if relative_path in seen_paths:
            raise ValueError("package cache manifest contains duplicate archive paths")
        seen_paths.add(relative_path)
        archive = cache_root / relative_path
        stat = archive.lstat()
        if not archive.is_file() or archive.is_symlink() or stat.st_nlink != 1:
            raise ValueError("package cache archive must be a single regular file")
        actual_size, actual_md5, actual_sha256 = _archive_digests(archive)
        if actual_size != size:
            raise ValueError("cache archive size differs from manifest")
        if actual_md5 != md5:
            raise ValueError("cache archive MD5 differs from manifest")
        if actual_sha256 != sha256:
            raise ValueError("cache archive SHA-256 differs from manifest")
    if seen_urls != set(expected):
        raise ValueError("package cache manifest archives do not match unified locks")
    urls_txt_sha256 = document.get("urls_txt_sha256")
    if not _is_hex_digest(urls_txt_sha256, 64):
        raise ValueError("package cache manifest must contain urls.txt SHA-256")
    urls_txt = cache_root / "pkgs" / "urls.txt"
    expected_urls = "\n".join(sorted(expected)) + "\n"
    if urls_txt.read_text(encoding="utf-8") != expected_urls:
        raise ValueError("package cache urls.txt differs from unified locks")
    if hashlib.sha256(urls_txt.read_bytes()).hexdigest() != urls_txt_sha256:
        raise ValueError("package cache urls.txt SHA-256 differs from manifest")
    expected_files = seen_paths | {"pkgs/urls.txt"}
    files, directories = _cache_file_census(cache_root)
    if files != expected_files or directories != _expected_cache_directories(expected_files):
        raise ValueError("package cache contains files outside the manifest")


def main(argv: Sequence[str] | None = None) -> int:
    """验证人工环境输入及已冻结锁的结构一致性，缺任何输入均 fail closed。"""
    args = parse_args(argv)
    try:
        runtime = _load_mapping(args.runtime_spec, "runtime spec")
        toolchain = _load_mapping(args.toolchain_spec, "toolchain spec")
        virtual_packages = _load_mapping(args.virtual_packages, "virtual package spec")
        _verify_runtime_spec(runtime)
        _verify_toolchain_spec(toolchain)
        _verify_virtual_packages(virtual_packages)
        if args.static_only:
            if any(
                getattr(args, name) is not None
                for name in (
                    "runtime_unified",
                    "runtime_explicit",
                    "toolchain_unified",
                    "toolchain_explicit",
                    "cache_manifest",
                    "cache_root",
                )
            ):
                raise ValueError("--static-only cannot be combined with frozen lock paths")
            print("PASS: stage 4 Python static environment inputs verified")
            return 0
        runtime_unified, runtime_explicit, toolchain_unified, toolchain_explicit = (
            _require_frozen_lock_paths(args)
        )
        _verify_lock_pair(runtime_unified, runtime_explicit, "runtime")
        _verify_lock_pair(toolchain_unified, toolchain_explicit, "toolchain")
        if (args.cache_manifest is None) != (args.cache_root is None):
            raise ValueError("cache verification requires both --cache-manifest and --cache-root")
        if args.cache_manifest is not None:
            _verify_cache_manifest(
                args.cache_manifest,
                args.cache_root,
                _unified_conda_records(runtime_unified, "runtime"),
                _unified_conda_records(toolchain_unified, "toolchain"),
            )
        print("PASS: stage 4 Python unified and explicit locks verified")
        return 0
    except (OSError, ValueError, json.JSONDecodeError, yaml.YAMLError) as error:
        print(f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
