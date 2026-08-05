#!/usr/bin/env python3
# 阶段四 eCAL wheel 联网 producer：固定官方 release bytes 并生成 canonical cache。
from __future__ import annotations

import argparse
import base64
from collections.abc import Sequence
import csv
from email.parser import BytesParser
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat
from urllib.request import urlopen
import zipfile

from wheel_elf import elf_inventory


PINNED_WHEEL = {
    "url": "https://files.pythonhosted.org/packages/01/fe/af512872e33e8891b37007808d9c2b58a3bf7b7df3ad32cc03539254d7b2/eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl",
    "filename": "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl",
    "size": 6905517,
    "sha256": "57a23af7d83c077c04f01852db13f8cda7686a052d41659fafcbe6b3dbe9f6bc",
    "distribution": "eclipse-ecal",
    "version": "6.1.1",
    "requires_python": ">=3.8",
    "python_tag": "cp310",
    "abi_tag": "cp310",
    "platform_tag": "manylinux_2_28_x86_64",
}
PYPI_RELEASE_JSON_URL = "https://pypi.org/pypi/eclipse-ecal/6.1.1/json"


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析全新下载根、canonical cache 与 manifest 输出路径。"""
    parser = argparse.ArgumentParser(description="Freeze the stage 4 eCAL wheel cache.")
    parser.add_argument("--download-root", type=Path, required=True)
    parser.add_argument("--cache-root", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    return parser.parse_args(argv)


def _digest(path: Path) -> tuple[int, str]:
    """流式复算下载 wheel 的大小和 SHA-256。"""
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            size += len(chunk)
            digest.update(chunk)
    return size, digest.hexdigest()


def _require_safe_member_paths(member_names: tuple[str, ...]) -> None:
    """拒绝 ZIP 安装时可逃逸目标根的绝对、回退或 NUL 路径。"""
    for member_name in member_names:
        normalized = member_name.removesuffix("/")
        relative = PurePosixPath(normalized)
        if (
            not normalized
            or "\0" in member_name
            or "\\" in member_name
            or relative.is_absolute()
            or any(part in {"", ".", ".."} for part in relative.parts)
        ):
            raise ValueError("wheel ZIP member path is unsafe")


def _dist_info_contract(path: Path) -> tuple[dict[str, str], dict[str, str]]:
    """读取唯一 dist-info 三个核心成员，固定其路径和未压缩内容摘要。"""
    try:
        with zipfile.ZipFile(path) as archive:
            member_names = tuple(info.filename for info in archive.infolist())
            _require_safe_member_paths(member_names)
            if len(member_names) != len(set(member_names)):
                raise ValueError("wheel must not contain duplicate ZIP members")
            metadata_paths = tuple(
                name for name in member_names if name.endswith(".dist-info/METADATA")
            )
            if len(metadata_paths) != 1:
                raise ValueError("wheel must contain exactly one dist-info METADATA member")
            metadata_path = metadata_paths[0]
            prefix = metadata_path.removesuffix("METADATA")
            paths = {
                "metadata": metadata_path,
                "wheel": f"{prefix}WHEEL",
                "record": f"{prefix}RECORD",
            }
            if any(member_path not in member_names for member_path in paths.values()):
                raise ValueError("wheel dist-info core member is missing")
            digests = {
                name: hashlib.sha256(archive.read(member_path)).hexdigest()
                for name, member_path in paths.items()
            }
            return paths, digests
    except zipfile.BadZipFile as error:
        raise ValueError("downloaded wheel is not a valid ZIP file") from error


def _license_member_path(dist_info_prefix: str, value: str) -> str:
    """将 PEP 639 的相对 License-File 映射为受限的 wheel member 路径。"""
    relative = PurePosixPath(value)
    if (
        not value
        or relative.is_absolute()
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise ValueError("wheel License-File path is unsafe")
    return f"{dist_info_prefix}licenses/{relative.as_posix()}"


def _license_contract(path: Path, metadata_path: str) -> dict[str, object]:
    """冻结 SPDX 表达式和 METADATA 声明的所有 license/NOTICE 内容摘要。"""
    try:
        with zipfile.ZipFile(path) as archive:
            metadata = BytesParser().parsebytes(archive.read(metadata_path))
            expression = metadata.get("License-Expression")
            license_files = tuple(metadata.get_all("License-File", ()))
            if not isinstance(expression, str) or not expression:
                raise ValueError("wheel METADATA License-Expression is missing")
            if not license_files or len(license_files) != len(set(license_files)):
                raise ValueError("wheel METADATA License-File entries are invalid")
            prefix = metadata_path.removesuffix("METADATA")
            paths = tuple(
                _license_member_path(prefix, value) for value in license_files
            )
            member_names = {info.filename for info in archive.infolist()}
            if any(member_path not in member_names for member_path in paths):
                raise ValueError("wheel License-File member is missing")
            return {
                "expression": expression,
                "files": {
                    member_path: hashlib.sha256(archive.read(member_path)).hexdigest()
                    for member_path in paths
                },
            }
    except zipfile.BadZipFile as error:
        raise ValueError("downloaded wheel is not a valid ZIP file") from error


def _record_digest(value: str) -> str:
    """将 wheel RECORD 的 base64url SHA-256 转为 manifest 使用的十六进制摘要。"""
    if not value.startswith("sha256="):
        raise ValueError("wheel RECORD must use SHA-256 member hashes")
    encoded = value.removeprefix("sha256=")
    try:
        digest = base64.urlsafe_b64decode(encoded + "=" * (-len(encoded) % 4))
    except ValueError as error:
        raise ValueError("wheel RECORD SHA-256 is not base64url") from error
    if len(digest) != hashlib.sha256().digest_size:
        raise ValueError("wheel RECORD SHA-256 has an invalid length")
    return digest.hex()


def _member_contract(path: Path, record_path: str) -> dict[str, object]:
    """冻结完整 ZIP 文件树和 RECORD 声明的每个可安装成员。"""
    try:
        with zipfile.ZipFile(path) as archive:
            infos = tuple(info for info in archive.infolist() if not info.filename.endswith("/"))
            member_names = tuple(info.filename for info in infos)
            if len(member_names) != len(set(member_names)):
                raise ValueError("wheel must not contain duplicate ZIP members")
            try:
                rows = tuple(
                    csv.reader(archive.read(record_path).decode("utf-8").splitlines())
                )
            except UnicodeDecodeError as error:
                raise ValueError("wheel RECORD must be UTF-8") from error
            if any(len(row) != 3 for row in rows):
                raise ValueError("wheel RECORD rows must contain path, hash, and size")
            if {row[0] for row in rows} != set(member_names) or len(rows) != len(member_names):
                raise ValueError("wheel RECORD does not enumerate all archive members")
            record_members: dict[str, dict[str, object]] = {}
            tree = hashlib.sha256()
            for member_path in sorted(member_names):
                payload = archive.read(member_path)
                payload_sha256 = hashlib.sha256(payload).hexdigest()
                tree.update(member_path.encode("utf-8"))
                tree.update(b"\0")
                tree.update(str(len(payload)).encode("ascii"))
                tree.update(b"\0")
                tree.update(payload_sha256.encode("ascii"))
                tree.update(b"\n")
            for member_path, encoded_digest, declared_size in rows:
                if member_path == record_path:
                    if encoded_digest or declared_size:
                        raise ValueError("wheel RECORD self entry must be empty")
                    continue
                payload = archive.read(member_path)
                if _record_digest(encoded_digest) != hashlib.sha256(payload).hexdigest():
                    raise ValueError("wheel RECORD SHA-256 differs from member")
                if not declared_size.isdecimal() or int(declared_size) != len(payload):
                    raise ValueError("wheel RECORD size differs from member")
                record_members[member_path] = {
                    "sha256": hashlib.sha256(payload).hexdigest(),
                    "size": len(payload),
                }
            return {
                "count": len(member_names),
                "record_members": record_members,
                "tree_sha256": tree.hexdigest(),
            }
    except zipfile.BadZipFile as error:
        raise ValueError("downloaded wheel is not a valid ZIP file") from error


def _elf_contract(path: Path) -> dict[str, object]:
    """冻结 wheel 内所有动态 ELF 的 ABI 字段并阻断 bundled Protobuf。"""
    try:
        with zipfile.ZipFile(path) as archive:
            return {"forbidden_soname": "libprotobuf.so", "members": elf_inventory(archive)}
    except zipfile.BadZipFile as error:
        raise ValueError("downloaded wheel is not a valid ZIP file") from error


def _download(url: str, destination: Path) -> None:
    """下载唯一 wheel 到本轮私有根，不读取 pip 或用户 cache。"""
    with urlopen(url, timeout=60) as response, destination.open("xb") as output:
        while chunk := response.read(1024 * 1024):
            output.write(chunk)
    destination.chmod(0o644)


def _load_pypi_release() -> object:
    """读取固定 release 的官方 JSON，供下载前交叉核验 identity。"""
    with urlopen(PYPI_RELEASE_JSON_URL, timeout=60) as response:
        return json.loads(response.read().decode("utf-8"))


def _verify_pypi_release(document: object) -> None:
    """只接受与 pinned eCAL wheel 的上游 JSON 元数据完全一致的 release。"""
    if not isinstance(document, dict):
        raise ValueError("PyPI release metadata differs from pinned wheel")
    info = document.get("info")
    urls = document.get("urls")
    if not isinstance(info, dict) or not isinstance(urls, list):
        raise ValueError("PyPI release metadata differs from pinned wheel")
    expected_info = {
        "name": PINNED_WHEEL["distribution"],
        "version": PINNED_WHEEL["version"],
        "requires_python": PINNED_WHEEL["requires_python"],
    }
    if any(info.get(field) != value for field, value in expected_info.items()):
        raise ValueError("PyPI release metadata differs from pinned wheel")
    matches = [
        entry
        for entry in urls
        if isinstance(entry, dict) and entry.get("filename") == PINNED_WHEEL["filename"]
    ]
    if len(matches) != 1:
        raise ValueError("PyPI release metadata differs from pinned wheel")
    wheel = matches[0]
    expected_wheel = {
        "url": PINNED_WHEEL["url"],
        "size": PINNED_WHEEL["size"],
        "packagetype": "bdist_wheel",
        "requires_python": PINNED_WHEEL["requires_python"],
        "yanked": False,
    }
    digests = wheel.get("digests")
    if (
        any(wheel.get(field) != value for field, value in expected_wheel.items())
        or not isinstance(digests, dict)
        or digests.get("sha256") != PINNED_WHEEL["sha256"]
    ):
        raise ValueError("PyPI release metadata differs from pinned wheel")


def _require_new_outputs(args: argparse.Namespace) -> tuple[Path, Path, Path]:
    """所有 producer 输出均须是调用方提供的全新、互不重叠路径。"""
    download_root = args.download_root.resolve()
    cache_root = args.cache_root.resolve()
    manifest = args.manifest.resolve()
    if len({download_root, cache_root, manifest}) != 3:
        raise ValueError("wheel producer outputs must be distinct")
    for output in (download_root, cache_root, manifest):
        if output.exists() or not output.parent.is_dir():
            raise ValueError("wheel producer outputs must be absent under existing directories")
    return download_root, cache_root, manifest


def _freeze(download_root: Path, cache_root: Path, manifest: Path) -> None:
    """下载、校验并独占复制 wheel，生成其可验证的 ZIP 基线。"""
    filename = str(PINNED_WHEEL["filename"])
    download_root.mkdir(mode=0o700)
    downloaded = download_root / filename
    _verify_pypi_release(_load_pypi_release())
    _download(str(PINNED_WHEEL["url"]), downloaded)
    downloaded_stat = downloaded.lstat()
    if (
        not stat.S_ISREG(downloaded_stat.st_mode)
        or stat.S_ISLNK(downloaded_stat.st_mode)
        or downloaded_stat.st_nlink != 1
    ):
        raise ValueError("downloaded wheel must be a singly linked regular file")
    size, sha256 = _digest(downloaded)
    if size != PINNED_WHEEL["size"] or sha256 != PINNED_WHEEL["sha256"]:
        raise ValueError("downloaded wheel differs from pinned PyPI release")
    dist_info_paths, dist_info_digests = _dist_info_contract(downloaded)
    licenses = _license_contract(downloaded, dist_info_paths["metadata"])
    members = _member_contract(downloaded, dist_info_paths["record"])
    elf = _elf_contract(downloaded)

    relative_path = f"wheels/{sha256}/{filename}"
    artifact = cache_root / relative_path
    artifact.parent.mkdir(parents=True, mode=0o755)
    with downloaded.open("rb") as source, artifact.open("xb") as target:
        while chunk := source.read(1024 * 1024):
            target.write(chunk)
    artifact.chmod(0o644)
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "wheel": {
                    **PINNED_WHEEL,
                    "relative_path": relative_path,
                    "dist_info_paths": dist_info_paths,
                    "dist_info_digests": dist_info_digests,
                    "licenses": licenses,
                    "members": members,
                    "elf": elf,
                },
            },
            ensure_ascii=True,
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    manifest.chmod(0o644)


def main(argv: Sequence[str] | None = None) -> int:
    """运行唯一允许联网的 wheel freeze，并输出稳定状态。"""
    try:
        download_root, cache_root, manifest = _require_new_outputs(parse_args(argv))
        _freeze(download_root, cache_root, manifest)
        print("PASS: pinned eCAL wheel cache rendered")
        return 0
    except (OSError, ValueError) as error:
        print(f"FAIL: {error}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
