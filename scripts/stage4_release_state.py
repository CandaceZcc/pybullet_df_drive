"""阶段四 E：同版本 release 的只读完整性判据，供 .run 安装事务复用。"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
from typing import Literal, Mapping, cast


ReleaseDecision = Literal["install", "activate_only", "reject"]
_SAFE_COMPONENT = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]*")
_GIT_SHA = re.compile(r"[0-9a-f]{40}")


def sha256_file(path: Path) -> str:
    """流式计算普通 release 文件的 SHA-256，拒绝符号链接和目录。"""
    if not path.is_file() or path.is_symlink():
        raise ValueError("release member must be a regular file")
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validated_manifest(document: object) -> dict[str, object]:
    """只接受安装器生成的冻结 manifest 结构，防止宽松解析掩盖损坏。"""
    if not isinstance(document, dict) or set(document) != {
        "schema_version", "version", "git_sha", "payload_sha256", "with_ros", "files", "dependencies", "doctor"
    }:
        raise ValueError("release manifest shape is invalid")
    if document["schema_version"] != 1 or not isinstance(document["version"], str) or not document["version"]:
        raise ValueError("release manifest identity is invalid")
    if not isinstance(document["git_sha"], str) or not _GIT_SHA.fullmatch(document["git_sha"]):
        raise ValueError("release manifest Git SHA is invalid")
    payload_sha256 = document["payload_sha256"]
    if not isinstance(payload_sha256, str) or len(payload_sha256) != 64 or any(
        character not in "0123456789abcdef" for character in payload_sha256
    ):
        raise ValueError("release manifest payload hash is invalid")
    if not isinstance(document["with_ros"], bool):
        raise ValueError("release manifest with_ros is invalid")
    files = document["files"]
    if not isinstance(files, dict) or not files:
        raise ValueError("release manifest files are invalid")
    for relative_path, digest in files.items():
        candidate = Path(relative_path) if isinstance(relative_path, str) else None
        if (
            candidate is None
            or candidate.is_absolute()
            or candidate == Path(".")
            or ".." in candidate.parts
            or not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise ValueError("release manifest file entry is invalid")
    dependencies = document["dependencies"]
    if not isinstance(dependencies, list):
        raise ValueError("release manifest dependencies are invalid")
    dependency_names: set[str] = set()
    dependency_paths: set[str] = set()
    for dependency in dependencies:
        if not isinstance(dependency, dict) or set(dependency) != {
            "name", "license", "filename", "sha256"
        }:
            raise ValueError("release manifest dependency entry is invalid")
        name = dependency["name"]
        license_name = dependency["license"]
        filename = dependency["filename"]
        digest = dependency["sha256"]
        candidate = Path(filename) if isinstance(filename, str) else None
        if (
            not isinstance(name, str)
            or not _SAFE_COMPONENT.fullmatch(name)
            or not isinstance(license_name, str)
            or not license_name.strip()
            or candidate is None
            or candidate.is_absolute()
            or len(candidate.parts) != 2
            or candidate.parts[0] != "dependencies"
            or not _SAFE_COMPONENT.fullmatch(candidate.parts[1])
            or not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
            or filename not in files
            or files[filename] != digest
            or name in dependency_names
            or filename in dependency_paths
        ):
            raise ValueError("release manifest dependency entry is invalid")
        dependency_names.add(name)
        dependency_paths.add(filename)
    if document["doctor"] != {"files_verified": True}:
        raise ValueError("release manifest doctor result is invalid")
    return cast(dict[str, object], document)


def inspect_release(release: Path, expected_manifest: Mapping[str, object]) -> ReleaseDecision:
    """判定目标 release 应新装、只激活，还是因同版本漂移而拒绝覆盖。"""
    expected = _validated_manifest(dict(expected_manifest))
    if not release.exists():
        return "install"
    if not release.is_dir() or release.is_symlink():
        return "reject"
    manifest_path = release / "manifest.json"
    try:
        installed = _validated_manifest(json.loads(manifest_path.read_text(encoding="utf-8")))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError):
        return "reject"
    if installed != expected:
        return "reject"
    files = cast(dict[str, str], expected["files"])
    try:
        for relative_path, expected_digest in files.items():
            if sha256_file(release / relative_path) != expected_digest:
                return "reject"
    except ValueError:
        return "reject"
    return "activate_only"


def activate_current(install_root: Path, release: Path) -> None:
    """以同目录临时 symlink 加原子 replace 激活已完整验证的 release。"""
    root = install_root.resolve()
    releases = root / "releases"
    if not root.is_dir() or not releases.is_dir() or not release.is_dir() or release.is_symlink():
        raise ValueError("release activation paths are invalid")
    if release.parent.resolve() != releases.resolve():
        raise ValueError("release must be a direct child of install_root/releases")
    temporary = root / f".current-{os.getpid()}"
    if os.path.lexists(temporary):
        raise FileExistsError("current activation temporary path already exists")
    try:
        temporary.symlink_to(release.relative_to(root))
        os.replace(temporary, root / "current")
    except BaseException:
        if os.path.lexists(temporary):
            temporary.unlink()
        raise
