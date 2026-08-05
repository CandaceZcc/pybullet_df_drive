#!/usr/bin/env python3
# 阶段四 eCAL 安装态验证：将 staging 中的已安装文件重新绑定到冻结 wheel manifest。
from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat

from wheel_elf import _elf64_dynamic


def _sha256(path: Path) -> tuple[int, str]:
    """流式复算安装文件的 size/SHA-256，不加载或执行 eCAL 代码。"""
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            size += len(chunk)
            digest.update(chunk)
    return size, digest.hexdigest()


def _regular_file(site_packages: Path, relative_path: str) -> Path:
    """解析受限相对路径，拒绝 staging runtime 内的链接和路径逃逸。"""
    relative = PurePosixPath(relative_path)
    if relative.is_absolute() or not relative.parts or any(
        part in {"", ".", ".."} for part in relative.parts
    ):
        raise ValueError("installed eCAL member path is unsafe")
    path = site_packages.joinpath(*relative.parts)
    path_stat = path.lstat()
    if not stat.S_ISREG(path_stat.st_mode) or stat.S_ISLNK(path_stat.st_mode):
        raise ValueError("installed eCAL member is not a regular file")
    return path


def _site_packages_root(runtime_root: Path, metadata_path: str) -> Path:
    """由冻结 METADATA 相对路径反推唯一 site-packages 根，拒绝多份安装。"""
    matches = tuple(path for path in runtime_root.rglob(metadata_path) if path.is_file())
    if len(matches) != 1:
        raise ValueError("installed eCAL METADATA must appear exactly once")
    site_packages = matches[0]
    for _part in PurePosixPath(metadata_path).parts:
        site_packages = site_packages.parent
    if not site_packages.is_dir() or site_packages.is_symlink():
        raise ValueError("installed eCAL site-packages root is invalid")
    return site_packages


def _verify_record(
    site_packages: Path,
    expected_members: dict[str, object],
    record_path: str,
) -> None:
    """复核安装 RECORD；仅允许 pip 的 INSTALLER、REQUESTED 和 self 行增量。"""
    record = _regular_file(site_packages, record_path)
    try:
        rows = list(csv.reader(record.read_text(encoding="utf-8").splitlines()))
    except UnicodeDecodeError as error:
        raise ValueError("installed eCAL RECORD must be UTF-8") from error
    if any(len(row) != 3 for row in rows):
        raise ValueError("installed eCAL RECORD rows must have three columns")
    actual = {row[0]: row for row in rows}
    if len(actual) != len(rows):
        raise ValueError("installed eCAL RECORD contains duplicate paths")
    prefix = record_path.removesuffix("RECORD")
    generated = {record_path, f"{prefix}INSTALLER", f"{prefix}REQUESTED"}
    if set(actual) - set(expected_members) - generated:
        raise ValueError("installed eCAL RECORD contains unexpected members")
    if set(expected_members) - set(actual):
        raise ValueError("installed eCAL RECORD omits frozen members")
    if f"{prefix}direct_url.json" in actual:
        raise ValueError("installed eCAL RECORD retains direct_url.json")
    if actual.get(record_path) != [record_path, "", ""]:
        raise ValueError("installed eCAL RECORD self row is invalid")
    for relative_path, expected in expected_members.items():
        if not isinstance(expected, dict) or set(expected) != {"sha256", "size"}:
            raise ValueError("frozen eCAL member contract is invalid")
        if not isinstance(expected["sha256"], str) or not isinstance(expected["size"], int):
            raise ValueError("frozen eCAL member digest is invalid")
        path = _regular_file(site_packages, relative_path)
        size, digest = _sha256(path)
        if (size, digest) != (expected["size"], expected["sha256"]):
            raise ValueError("installed eCAL member differs from frozen wheel")
        encoded = base64.urlsafe_b64encode(bytes.fromhex(digest)).rstrip(b"=").decode("ascii")
        if actual[relative_path] != [relative_path, f"sha256={encoded}", str(size)]:
            raise ValueError("installed eCAL RECORD digest differs from member")
    for relative_path in generated - {record_path}:
        if relative_path not in actual:
            continue
        path = _regular_file(site_packages, relative_path)
        size, digest = _sha256(path)
        encoded = base64.urlsafe_b64encode(bytes.fromhex(digest)).rstrip(b"=").decode("ascii")
        if actual[relative_path] != [relative_path, f"sha256={encoded}", str(size)]:
            raise ValueError("installed eCAL generated RECORD row is invalid")


def _verify_licenses(site_packages: Path, files: object) -> None:
    """逐项重算冻结的 license/NOTICE，防止安装态被 pip 或后处理改写。"""
    if not isinstance(files, dict) or not files:
        raise ValueError("frozen eCAL license contract is invalid")
    for relative_path, expected_digest in files.items():
        if not isinstance(expected_digest, str):
            raise ValueError("frozen eCAL license digest is invalid")
        _size, digest = _sha256(_regular_file(site_packages, relative_path))
        if digest != expected_digest:
            raise ValueError("installed eCAL license differs from frozen wheel")


def _verify_elf(site_packages: Path, expected: object, members: dict[str, object]) -> None:
    """从安装态冻结成员重新解析全部 ELF，禁止出现第二套或篡改后的 ABI。"""
    if not isinstance(expected, list):
        raise ValueError("frozen eCAL ELF contract is invalid")
    actual = []
    for relative_path in members:
        path = _regular_file(site_packages, relative_path)
        payload = path.read_bytes()
        if not payload.startswith(b"\x7fELF"):
            continue
        basename = PurePosixPath(relative_path).name
        if basename == "libprotobuf.so" or basename.startswith("libprotobuf.so."):
            raise ValueError("installed eCAL must not bundle libprotobuf.so")
        actual.append(_elf64_dynamic(relative_path, payload))
    if sorted(actual, key=lambda member: str(member["path"])) != expected:
        raise ValueError("installed eCAL ELF inventory differs from frozen wheel")


def verify(runtime_root: Path, manifest_path: Path) -> None:
    """验证一份 staging runtime 中已安装 eCAL 的完整文件、license 与 ELF 合同。"""
    document = json.loads(manifest_path.read_text(encoding="utf-8"))
    wheel = document.get("wheel") if isinstance(document, dict) else None
    if not isinstance(wheel, dict):
        raise ValueError("wheel manifest is invalid")
    paths = wheel.get("dist_info_paths")
    members = wheel.get("members")
    licenses = wheel.get("licenses")
    elf = wheel.get("elf")
    if not isinstance(paths, dict) or not isinstance(members, dict) or not isinstance(licenses, dict) or not isinstance(elf, dict):
        raise ValueError("wheel manifest install contract is incomplete")
    metadata_path = paths.get("metadata")
    record_path = paths.get("record")
    record_members = members.get("record_members")
    if not isinstance(metadata_path, str) or not isinstance(record_path, str) or not isinstance(record_members, dict):
        raise ValueError("wheel manifest install paths are invalid")
    site_packages = _site_packages_root(runtime_root, metadata_path)
    _verify_record(site_packages, record_members, record_path)
    _verify_licenses(site_packages, licenses.get("files"))
    _verify_elf(site_packages, elf.get("members"), record_members)


def main() -> int:
    """以显式 runtime 根和 wheel manifest 运行安装态 eCAL 完整性验证。"""
    parser = argparse.ArgumentParser(description="Verify installed Stage 4 eCAL wheel contents.")
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    args = parser.parse_args()
    try:
        verify(args.runtime_root.resolve(), args.manifest.resolve())
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}")
        return 1
    print("PASS: installed eCAL runtime matches frozen wheel contract")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
