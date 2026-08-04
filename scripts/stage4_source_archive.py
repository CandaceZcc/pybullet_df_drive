#!/usr/bin/env python3
# 阶段四源码归档安全规则：freeze、verifier 与离线 builder 共用同一成员预检入口。
from __future__ import annotations

from collections.abc import Iterable
import hashlib
from pathlib import Path
import posixpath
import shutil
import stat
import tarfile


def validate_member_path(path: str) -> None:
    """拒绝归档成员名的歧义或根外路径，解包前必须先通过该检查。"""
    if not path:
        raise ValueError("archive member path must not be empty")
    if any(ord(character) < 32 or ord(character) == 127 for character in path):
        raise ValueError("archive member path must not contain control characters")
    if path.startswith("/"):
        raise ValueError("archive member path must be relative")
    parts = path.split("/")
    if "." in parts:
        raise ValueError("archive member path must not contain current-directory traversal")
    if ".." in parts:
        raise ValueError("archive member path must not contain parent traversal")


def validate_member_paths(paths: Iterable[str]) -> str:
    """校验成员集合共用唯一顶层根，并返回该冻结目录名。"""
    roots: set[str] = set()
    for path in paths:
        validate_member_path(path)
        roots.add(path.split("/", 1)[0])
    if len(roots) != 1:
        raise ValueError("archive members must use exactly one top-level root")
    return roots.pop()


def inspect_archive(path: Path) -> str:
    """在解包前读取 tar 成员并拒绝重复路径，返回唯一顶层根。"""
    with tarfile.open(path, mode="r:*") as archive:
        members = archive.getmembers()
    for member in members:
        if not (member.isreg() or member.isdir() or member.issym()):
            raise ValueError("archive contains an unsupported special member")
    paths = [member.name for member in members]
    declared_paths = set(paths)
    if len(declared_paths) != len(paths):
        raise ValueError("archive contains a duplicate member path")
    root = validate_member_paths(paths)
    regular_paths = {member.name for member in members if member.isreg()}
    for path in paths:
        parts = path.split("/")
        for index in range(1, len(parts)):
            if "/".join(parts[:index]) in regular_paths:
                raise ValueError("archive contains a file-directory conflict")
    member_by_path = {member.name: member for member in members}
    symlink_targets: dict[str, str] = {}
    for member in members:
        if member.issym():
            parent = member.name.rpartition("/")[0]
            resolved = posixpath.normpath(posixpath.join(parent, member.linkname))
            if resolved != root and not resolved.startswith(root + "/"):
                raise ValueError("archive symlink target escapes top-level root")
            if resolved not in declared_paths:
                raise ValueError("archive symlink target is not a declared member")
            symlink_targets[member.name] = resolved
    for source in symlink_targets:
        current = source
        visited: set[str] = set()
        while current in symlink_targets:
            if current in visited:
                raise ValueError("archive symlink chain contains a cycle")
            visited.add(current)
            current = symlink_targets[current]
        if not (member_by_path[current].isreg() or member_by_path[current].isdir()):
            raise ValueError("archive symlink target must resolve to a file or directory")
    return root


def archive_census(path: Path) -> dict[str, int | str]:
    """返回已通过安全预检的归档成员统计，供冻结与复核使用。"""
    root = inspect_archive(path)
    with tarfile.open(path, mode="r:*") as archive:
        members = archive.getmembers()
    return {
        "top_level_root": root,
        "member_count": len(members),
        "regular_bytes": sum(member.size for member in members if member.isreg()),
        "symlink_count": sum(1 for member in members if member.issym()),
    }


def materialized_tree_digest(root: Path) -> dict[str, int | str]:
    """验证零链接物化树并返回排序成员的稳定摘要。"""
    root_metadata = root.lstat()
    if not stat.S_ISDIR(root_metadata.st_mode) or stat.S_ISLNK(root_metadata.st_mode):
        raise ValueError("materialized tree root must be a directory without symlink")
    records: list[tuple[str, bytes]] = []
    pending = [root]
    member_count = 0
    regular_bytes = 0
    while pending:
        current = pending.pop()
        for child in current.iterdir():
            metadata = child.lstat()
            relative = child.relative_to(root).as_posix()
            if stat.S_ISDIR(metadata.st_mode):
                records.append((relative, b"D\0" + relative.encode() + b"\n"))
                pending.append(child)
            elif stat.S_ISREG(metadata.st_mode):
                if metadata.st_nlink != 1:
                    raise ValueError("materialized tree regular files must have st_nlink == 1")
                file_digest = hashlib.sha256(child.read_bytes()).hexdigest()
                records.append(
                    (
                        relative,
                        b"F\0" + relative.encode() + b"\0" + file_digest.encode() + b"\n",
                    )
                )
                member_count += 1
                regular_bytes += metadata.st_size
            else:
                raise ValueError("materialized tree contains a non-regular member")
    digest = hashlib.sha256()
    for _, record in sorted(records):
        digest.update(record)
    return {
        "materialized_member_count": member_count,
        "materialized_regular_bytes": regular_bytes,
        "materialized_tree_sha256": digest.hexdigest(),
    }


def materialize_archive(path: Path, output: Path) -> str:
    """将已预检的 tar 物化为无符号链接的新目录树。"""
    if output.exists():
        raise ValueError("archive materialization output must not already exist")
    root = inspect_archive(path)
    with tarfile.open(path, mode="r:*") as archive:
        members = archive.getmembers()
        member_by_path = {member.name: member for member in members}
        payloads: dict[str, bytes] = {}
        for member in members:
            if member.isreg():
                source = archive.extractfile(member)
                if source is None:
                    raise ValueError("archive regular member has no readable payload")
                payloads[member.name] = source.read()

    output.mkdir()
    materialized: set[str] = set()

    def resolve_link(member) -> str:
        """沿已通过预检的相对链接解析到最终普通成员。"""
        current = member.name
        while member_by_path[current].issym():
            link = member_by_path[current]
            parent = link.name.rpartition("/")[0]
            current = posixpath.normpath(posixpath.join(parent, link.linkname))
        return current

    def materialize_member(name: str) -> None:
        """递归写入普通成员或其 symlink 最终目标，绝不创建链接。"""
        if name in materialized:
            return
        member = member_by_path[name]
        destination = output / name
        if member.issym():
            materialize_member(resolve_link(member))
            target = output / resolve_link(member)
            if target.is_dir():
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copytree(target, destination, copy_function=shutil.copyfile)
            else:
                destination.parent.mkdir(parents=True, exist_ok=True)
                with target.open("rb") as source, destination.open("xb") as sink:
                    sink.write(source.read())
        elif member.isdir():
            destination.mkdir(parents=True, exist_ok=True)
        else:
            destination.parent.mkdir(parents=True, exist_ok=True)
            with destination.open("xb") as destination_file:
                destination_file.write(payloads[name])
        materialized.add(name)

    for member in members:
        if not member.issym():
            materialize_member(member.name)
    for member in members:
        if member.issym():
            materialize_member(member.name)
    return root
