"""阶段四 LVX2 oracle：独立、流式校验 Livox LVX2 v1.0 二进制布局。"""
from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import struct
from typing import BinaryIO


_PUBLIC_HEADER = struct.Struct("<16s4BI")
_PRIVATE_HEADER = struct.Struct("<IB")
_DEVICE_INFO = struct.Struct("<16s16sIBBBffffff")
_FRAME_HEADER = struct.Struct("<QQQ")
_PACKAGE_HEADER = struct.Struct("<BIBBQHBIB4s")
_POINT_TYPE_1 = struct.Struct("<iiiBB")
_SIGNATURE = b"livox_tech" + b"\0" * 6
_MAGIC = 0xAC0EA767
_DETAIL_FRAME_LIMIT = 2
_DETAIL_PACKAGE_LIMIT = 2


class Lvx2FormatError(ValueError):
    """LVX2 文件不满足冻结布局时的 fail-closed 错误。"""


@dataclass(frozen=True)
class DeviceInfo:
    """LVX2 device info 的 oracle 可审查视图。"""

    serial_number: bytes
    hub_serial_number: bytes
    lidar_id: int
    lidar_type: int
    device_type: int
    extrinsic_enabled: bool
    extrinsic: tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class Package:
    """一个已校验的 package 和其原始坐标单位点。"""

    version: int
    lidar_id: int
    lidar_type: int
    timestamp_type: int
    timestamp_raw: bytes
    timestamp_ns: int
    udp_counter: int
    data_type: int
    length: int
    frame_counter: int
    reserved: bytes
    points: tuple[tuple[int, int, int, int, int], ...]


@dataclass(frozen=True)
class Frame:
    """一个 frame 的文件 offset、连续 index 与 packages。"""

    current_offset: int
    next_offset: int
    index: int
    packages: tuple[Package, ...]


@dataclass(frozen=True)
class Lvx2File:
    """只保留校验所需的 LVX2 结构化事实。"""

    version: tuple[int, int, int, int]
    frame_duration_ms: int
    devices: tuple[DeviceInfo, ...]
    frames: tuple[Frame, ...]
    file_size: int
    frame_count: int
    package_count: int
    point_count: int
    complete: bool


def _read_exact(stream: BinaryIO, size: int, label: str) -> bytes:
    """以固定大小流式读取字段，截断立即失败且不读取整个文件。"""
    data = stream.read(size)
    if len(data) != size:
        raise Lvx2FormatError(f"truncated {label}")
    return data


def _parse_package(stream: BinaryIO, frame_end: int) -> Package:
    """在 frame 边界内解析一个 SDK normal-data package。"""
    header_offset = stream.tell()
    if header_offset + _PACKAGE_HEADER.size > frame_end:
        raise Lvx2FormatError("truncated package header")
    header = _read_exact(stream, _PACKAGE_HEADER.size, "package header")
    (
        version,
        lidar_id,
        lidar_type,
        timestamp_type,
        timestamp_ns,
        udp_counter,
        data_type,
        length,
        frame_counter,
        reserved,
    ) = _PACKAGE_HEADER.unpack(header)
    if version != 0:
        raise Lvx2FormatError(f"package version {version} is unsupported")
    if timestamp_type not in (0, 1, 2):
        raise Lvx2FormatError(f"timestamp_type {timestamp_type} is unsupported")
    if data_type != 1:
        raise Lvx2FormatError(f"data_type {data_type} is unsupported")
    point_struct = _POINT_TYPE_1
    if length % point_struct.size:
        raise Lvx2FormatError("package length is not a whole point count")
    if length // point_struct.size > 96:
        raise Lvx2FormatError("package contains more than 96 points")
    if stream.tell() + length > frame_end:
        raise Lvx2FormatError("truncated package points")
    payload = _read_exact(stream, length, "package points")
    return Package(
        version,
        lidar_id,
        lidar_type,
        timestamp_type,
        header[7:15],
        timestamp_ns,
        udp_counter,
        data_type,
        length,
        frame_counter,
        reserved,
        tuple(
            point_struct.unpack_from(payload, offset)
            for offset in range(0, length, point_struct.size)
        ),
    )


def parse_lvx2(path: Path, *, frame_limit: int | None = None) -> Lvx2File:
    """流式解析 LVX2；frame_limit 让大官方样例仅 seek 检查前若干帧。"""
    with path.open("rb") as stream:
        stream.seek(0, os.SEEK_END)
        file_size = stream.tell()
        stream.seek(0)
        signature, *version_and_magic = _PUBLIC_HEADER.unpack(
            _read_exact(stream, _PUBLIC_HEADER.size, "public header")
        )
        version = tuple(version_and_magic[:4])
        magic = version_and_magic[4]
        if signature != _SIGNATURE:
            raise Lvx2FormatError("signature is invalid")
        if version != (2, 0, 0, 0):
            raise Lvx2FormatError(f"version {version!r} is unsupported")
        if magic != _MAGIC:
            raise Lvx2FormatError("magic is invalid")
        frame_duration_ms, device_count = _PRIVATE_HEADER.unpack(
            _read_exact(stream, _PRIVATE_HEADER.size, "private header")
        )
        if frame_duration_ms != 50:
            raise Lvx2FormatError("frame_duration must be 50 ms")
        if device_count != 1:
            raise Lvx2FormatError("device_count must be 1")
        devices = []
        for index in range(device_count):
            (
                serial,
                hub_serial,
                lidar_id,
                lidar_type,
                device_type,
                extrinsic_enabled,
                *extrinsic,
            ) = _DEVICE_INFO.unpack(_read_exact(stream, _DEVICE_INFO.size, f"device {index}"))
            if device_type != 9:
                raise Lvx2FormatError(f"device_type {device_type} is not Mid-360")
            if extrinsic_enabled not in (0, 1):
                raise Lvx2FormatError("extrinsic enable must be 0 or 1")
            devices.append(
                DeviceInfo(
                    serial.rstrip(b"\0"),
                    hub_serial.rstrip(b"\0"),
                    lidar_id,
                    lidar_type,
                    device_type,
                    bool(extrinsic_enabled),
                    tuple(extrinsic),
                )
            )

        expected_offset = stream.tell()
        frames = []
        frame_count = 0
        package_count = 0
        point_count = 0
        while expected_offset < file_size and (
            frame_limit is None or frame_count < frame_limit
        ):
            stream.seek(expected_offset)
            current, next_offset, index = _FRAME_HEADER.unpack(
                _read_exact(stream, _FRAME_HEADER.size, "frame header")
            )
            if current != expected_offset:
                raise Lvx2FormatError("current_offset does not match frame position")
            if index != frame_count:
                raise Lvx2FormatError("frame index is not consecutive")
            if next_offset < current + _FRAME_HEADER.size or next_offset > file_size:
                raise Lvx2FormatError("next_offset is outside frame bounds")
            keep_details = len(frames) < _DETAIL_FRAME_LIMIT
            packages = []
            while stream.tell() < next_offset:
                package = _parse_package(stream, next_offset)
                package_count += 1
                point_count += len(package.points)
                if keep_details and len(packages) < _DETAIL_PACKAGE_LIMIT:
                    packages.append(package)
            if stream.tell() != next_offset:
                raise Lvx2FormatError("package does not end at next_offset")
            if keep_details:
                frames.append(Frame(current, next_offset, index, tuple(packages)))
            frame_count += 1
            expected_offset = next_offset
        complete = expected_offset == file_size
        if frame_limit is None and not complete:
            raise Lvx2FormatError("EOF is not a frame boundary")
    return Lvx2File(
        version,
        frame_duration_ms,
        tuple(devices),
        tuple(frames),
        file_size,
        frame_count,
        package_count,
        point_count,
        complete,
    )
