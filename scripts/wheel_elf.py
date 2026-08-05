#!/usr/bin/env python3
# 阶段四 wheel ELF 解析：只用标准库读取动态链接 ABI，不执行制品代码。
from __future__ import annotations

import hashlib
from pathlib import PurePosixPath
import struct
import zipfile


_ELF_MAGIC = b"\x7fELF"
_ELF64_HEADER = struct.Struct("<16sHHIQQQIHHHHHH")
_ELF64_SECTION = struct.Struct("<IIQQQQIIQQ")
_ELF64_DYNAMIC = struct.Struct("<qQ")
_SHT_DYNAMIC = 6
_DT_NEEDED = 1
_DT_SONAME = 14
_DT_RPATH = 15
_DT_RUNPATH = 29
_MACHINES = {62: "EM_X86_64"}


def _slice(payload: bytes, offset: int, size: int, label: str) -> bytes:
    """读取经边界检查的 ELF 区段，拒绝截断或整数越界输入。"""
    if offset < 0 or size < 0 or offset > len(payload) or size > len(payload) - offset:
        raise ValueError(f"ELF {label} is outside the member")
    return payload[offset : offset + size]


def _cstring(table: bytes, offset: int, label: str) -> str:
    """从动态字符串表读取一个严格 UTF-8 的 NUL 终止字符串。"""
    if offset < 0 or offset >= len(table):
        raise ValueError(f"ELF {label} string offset is invalid")
    end = table.find(b"\0", offset)
    if end < 0:
        raise ValueError(f"ELF {label} string is not NUL terminated")
    try:
        return table[offset:end].decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError(f"ELF {label} string is not UTF-8") from error


def _elf64_dynamic(path: str, payload: bytes) -> dict[str, object]:
    """解析 little-endian ELF64 的 section/dynamic 表，提取加载合同。"""
    if len(payload) < _ELF64_HEADER.size:
        raise ValueError(f"ELF member is truncated: {path}")
    header = _ELF64_HEADER.unpack_from(payload)
    ident = header[0]
    if ident[:4] != _ELF_MAGIC or ident[4] != 2 or ident[5] != 1:
        raise ValueError(f"ELF member has unsupported class or byte order: {path}")
    machine = _MACHINES.get(header[2])
    if machine is None:
        raise ValueError(f"ELF member has unsupported machine: {path}")
    section_offset, section_size, section_count = header[6], header[11], header[12]
    if section_size != _ELF64_SECTION.size or section_count == 0:
        raise ValueError(f"ELF member has unsupported section table: {path}")
    sections = tuple(
        _ELF64_SECTION.unpack(_slice(payload, section_offset + index * section_size, section_size, "section"))
        for index in range(section_count)
    )
    dynamic_sections = tuple(section for section in sections if section[1] == _SHT_DYNAMIC)
    if len(dynamic_sections) != 1:
        raise ValueError(f"ELF member must contain exactly one dynamic section: {path}")
    dynamic = dynamic_sections[0]
    string_index = dynamic[6]
    if string_index >= len(sections):
        raise ValueError(f"ELF dynamic string table index is invalid: {path}")
    strings_section = sections[string_index]
    strings = _slice(payload, strings_section[4], strings_section[5], "dynamic string table")
    entry_size = dynamic[9]
    if entry_size != _ELF64_DYNAMIC.size or dynamic[5] % entry_size:
        raise ValueError(f"ELF dynamic table is malformed: {path}")
    needed: list[str] = []
    sonames: list[str] = []
    runpaths: list[str] = []
    for offset in range(dynamic[4], dynamic[4] + dynamic[5], entry_size):
        tag, value = _ELF64_DYNAMIC.unpack(_slice(payload, offset, entry_size, "dynamic entry"))
        if tag == _DT_NEEDED:
            needed.append(_cstring(strings, value, "DT_NEEDED"))
        elif tag == _DT_SONAME:
            sonames.append(_cstring(strings, value, "DT_SONAME"))
        elif tag in {_DT_RPATH, _DT_RUNPATH}:
            runpaths.append(_cstring(strings, value, "DT_RUNPATH"))
    if len(sonames) > 1 or len(runpaths) > 1:
        raise ValueError(f"ELF member has ambiguous SONAME or RUNPATH: {path}")
    return {
        "class": "ELF64",
        "machine": machine,
        "needed": needed,
        "path": path,
        "runpath": runpaths[0] if runpaths else None,
        "sha256": hashlib.sha256(payload).hexdigest(),
        "soname": sonames[0] if sonames else None,
    }


def elf_inventory(archive: zipfile.ZipFile) -> list[dict[str, object]]:
    """返回 wheel 内全部 ELF 的稳定 ABI 清单，拒绝 bundled libprotobuf。"""
    members: list[dict[str, object]] = []
    for info in archive.infolist():
        if info.filename.endswith("/"):
            continue
        payload = archive.read(info.filename)
        if not payload.startswith(_ELF_MAGIC):
            continue
        basename = PurePosixPath(info.filename).name
        if basename == "libprotobuf.so" or basename.startswith("libprotobuf.so."):
            raise ValueError("wheel must not bundle libprotobuf.so")
        members.append(_elf64_dynamic(info.filename, payload))
    return sorted(members, key=lambda member: str(member["path"]))
