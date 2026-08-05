#!/usr/bin/env python3
# 阶段四 Python wheel cache 验证入口：先锁定 canonical artifact 外壳，再扩展 ZIP/ELF 合同。
from __future__ import annotations

import argparse
import base64
from collections.abc import Sequence
import csv
from email.parser import BytesParser
import hashlib
import json
from pathlib import Path
import stat
from urllib.parse import urlsplit
import zipfile


PINNED_WHEEL_FIELDS = {
    "filename": "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl",
    "distribution": "eclipse-ecal",
    "version": "6.1.1",
    "python_tag": "cp310",
    "abi_tag": "cp310",
    "platform_tag": "manylinux_2_28_x86_64",
}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """只接收显式 manifest 与 cache 根，禁止从 pip cache 或环境变量猜输入。"""
    parser = argparse.ArgumentParser(description="Verify a stage 4 Python wheel cache.")
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    return parser.parse_args(argv)


def _is_https_url(value: object) -> bool:
    """wheel 下载地址必须是无 query/fragment 的绝对 HTTPS URL。"""
    if not isinstance(value, str):
        return False
    parsed = urlsplit(value)
    return (
        parsed.scheme == "https"
        and bool(parsed.netloc)
        and bool(parsed.path)
        and not parsed.query
        and not parsed.fragment
    )


def _is_sha256(value: object) -> bool:
    """只接受固定长度的小写十六进制 SHA-256。"""
    return (
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _expected_directories(files: set[str]) -> set[str]:
    """从允许文件集合推导 cache 的唯一目录树，拒绝遗留空目录。"""
    directories: set[str] = set()
    for filename in files:
        parent = Path(filename).parent
        while parent != Path("."):
            directories.add(parent.as_posix())
            parent = parent.parent
    return directories


def _cache_census(cache_root: Path) -> tuple[set[str], set[str]]:
    """枚举 cache 内全部成员，拒绝链接、特殊文件和硬链接 wheel。"""
    root_stat = cache_root.lstat()
    if not stat.S_ISDIR(root_stat.st_mode) or stat.S_ISLNK(root_stat.st_mode):
        raise ValueError("wheel cache root must be a real directory")
    files: set[str] = set()
    directories: set[str] = set()
    for member in cache_root.rglob("*"):
        member_stat = member.lstat()
        relative_path = member.relative_to(cache_root).as_posix()
        if stat.S_ISLNK(member_stat.st_mode):
            raise ValueError("wheel cache must not contain links")
        if stat.S_ISDIR(member_stat.st_mode):
            directories.add(relative_path)
            continue
        if not stat.S_ISREG(member_stat.st_mode) or member_stat.st_nlink != 1:
            raise ValueError("wheel cache must contain only single-link regular files")
        files.add(relative_path)
    return files, directories


def _wheel_digest(path: Path) -> tuple[int, str]:
    """流式复算 wheel 的 size/SHA-256，不执行 ZIP 内容。"""
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            size += len(chunk)
            digest.update(chunk)
    return size, digest.hexdigest()


def _verify_zip_wheel_contract(artifact: Path, wheel: dict[str, object]) -> None:
    """ABI 元数据一经写入 manifest，就必须能由真实 ZIP wheel 读取。"""
    contract_fields = (
        "distribution",
        "version",
        "requires_python",
        "python_tag",
        "abi_tag",
        "platform_tag",
    )
    present = tuple(field for field in contract_fields if field in wheel)
    if len(present) != len(contract_fields) or not all(
        isinstance(wheel[field], str) and wheel[field] for field in contract_fields
    ):
        raise ValueError("wheel manifest metadata contract is incomplete")
    try:
        with zipfile.ZipFile(artifact) as archive:
            member_names = tuple(info.filename for info in archive.infolist())
            metadata_paths = tuple(
                name for name in member_names if name.endswith(".dist-info/METADATA")
            )
            if len(metadata_paths) != 1:
                raise ValueError("wheel must contain exactly one dist-info METADATA member")
            metadata_path = metadata_paths[0]
            dist_info_prefix = metadata_path.removesuffix("METADATA")
            wheel_path = f"{dist_info_prefix}WHEEL"
            if wheel_path not in member_names:
                raise ValueError("wheel dist-info WHEEL member is missing")
            record_path = f"{dist_info_prefix}RECORD"
            if record_path not in member_names:
                raise ValueError("wheel dist-info RECORD member is missing")
            metadata = BytesParser().parsebytes(archive.read(metadata_path))
            wheel_metadata = BytesParser().parsebytes(archive.read(wheel_path))
            try:
                record_rows = tuple(
                    csv.reader(archive.read(record_path).decode("utf-8").splitlines())
                )
            except UnicodeDecodeError as error:
                raise ValueError("wheel RECORD must be UTF-8") from error
            if any(len(row) != 3 for row in record_rows):
                raise ValueError("wheel RECORD rows must contain path, hash, and size")
            record_paths = tuple(row[0] for row in record_rows)
            archive_paths = tuple(name for name in member_names if not name.endswith("/"))
            if set(record_paths) != set(archive_paths) or len(record_paths) != len(archive_paths):
                raise ValueError("wheel RECORD does not enumerate all archive members")
            for path, encoded_digest, declared_size in record_rows:
                if path == record_path:
                    if encoded_digest or declared_size:
                        raise ValueError("wheel RECORD self entry must be empty")
                    continue
                if not encoded_digest.startswith("sha256="):
                    raise ValueError("wheel RECORD must use SHA-256 member hashes")
                encoded_value = encoded_digest.removeprefix("sha256=")
                try:
                    expected_digest = base64.urlsafe_b64decode(
                        encoded_value + "=" * (-len(encoded_value) % 4)
                    )
                except ValueError as error:
                    raise ValueError("wheel RECORD SHA-256 is not base64url") from error
                payload = archive.read(path)
                if hashlib.sha256(payload).digest() != expected_digest:
                    raise ValueError("wheel RECORD SHA-256 differs from member")
                if not declared_size.isdecimal() or int(declared_size) != len(payload):
                    raise ValueError("wheel RECORD size differs from member")
    except zipfile.BadZipFile as error:
        raise ValueError("wheel artifact is not a valid ZIP file") from error

    if metadata.get("Name") != wheel["distribution"]:
        raise ValueError("wheel METADATA distribution differs from manifest")
    if metadata.get("Version") != wheel["version"]:
        raise ValueError("wheel METADATA version differs from manifest")
    if metadata.get("Requires-Python") != wheel["requires_python"]:
        raise ValueError("wheel METADATA Requires-Python differs from manifest")
    expected_tag = "-".join(
        (wheel["python_tag"], wheel["abi_tag"], wheel["platform_tag"])
    )
    if tuple(wheel_metadata.get_all("Tag", ())) != (expected_tag,):
        raise ValueError("wheel WHEEL tag differs from manifest")


def _verify_pinned_wheel_identity(wheel: dict[str, object]) -> None:
    """阻断内部 cache 以自洽 manifest 偷换计划锁定的 eCAL release。"""
    if any(wheel.get(field) != expected for field, expected in PINNED_WHEEL_FIELDS.items()):
        raise ValueError("wheel manifest does not match the pinned eCAL release")


def _verify_manifest(manifest_path: Path, cache_root: Path) -> None:
    """验证唯一 wheel 的 URL、canonical 路径、bytes 与 cache census。"""
    document = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("wheel cache manifest must use schema_version 1")
    wheel = document.get("wheel")
    if not isinstance(wheel, dict):
        raise ValueError("wheel cache manifest must contain a wheel mapping")
    url = wheel.get("url")
    filename = wheel.get("filename")
    relative_path = wheel.get("relative_path")
    size = wheel.get("size")
    sha256 = wheel.get("sha256")
    if not _is_https_url(url):
        raise ValueError("wheel manifest URL must use normalized HTTPS")
    if not isinstance(filename, str) or Path(filename).name != filename or not filename:
        raise ValueError("wheel manifest filename must be a basename")
    if Path(urlsplit(url).path).name != filename:
        raise ValueError("wheel manifest filename differs from URL basename")
    if not _is_sha256(sha256):
        raise ValueError("wheel manifest must contain a SHA-256")
    expected_relative_path = f"wheels/{sha256}/{filename}"
    if relative_path != expected_relative_path:
        raise ValueError("wheel manifest uses a noncanonical artifact path")
    if not isinstance(size, int) or isinstance(size, bool) or size < 0:
        raise ValueError("wheel manifest size must be a nonnegative integer")

    artifact = cache_root / relative_path
    artifact_stat = artifact.lstat()
    if (
        not stat.S_ISREG(artifact_stat.st_mode)
        or stat.S_ISLNK(artifact_stat.st_mode)
        or artifact_stat.st_nlink != 1
    ):
        raise ValueError("wheel artifact must be a singly linked regular file")
    actual_size, actual_sha256 = _wheel_digest(artifact)
    if actual_size != size:
        raise ValueError("wheel artifact size differs from manifest")
    if actual_sha256 != sha256:
        raise ValueError("wheel artifact SHA-256 differs from manifest")
    _verify_zip_wheel_contract(artifact, wheel)
    _verify_pinned_wheel_identity(wheel)
    files, directories = _cache_census(cache_root)
    if files != {relative_path} or directories != _expected_directories({relative_path}):
        raise ValueError("wheel cache contains files outside the manifest")


def main(argv: Sequence[str] | None = None) -> int:
    """在 pip 前 fail closed 验证 frozen wheel artifact。"""
    args = parse_args(argv)
    try:
        _verify_manifest(args.manifest, args.cache_root)
        print("PASS: stage 4 Python wheel cache artifact verified")
        return 0
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
