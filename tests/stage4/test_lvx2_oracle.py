# 阶段四 LVX2 oracle：以手写字节冻结官方 v1.0 的只读结构合同。
from __future__ import annotations

import io
import os
from pathlib import Path
import struct
from types import SimpleNamespace

import pytest


def _device(*, lidar_id: int = 7, device_type: int = 9) -> bytes:
    """手写 63-byte device info，避免 fixture 依赖待测 writer/parser。"""
    return struct.pack(
        "<16s16sIBBBffffff",
        b"SLOPESIM00000001",
        b"\0" * 16,
        lidar_id,
        0,
        device_type,
        0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def _package(
    *,
    data_type: int,
    timestamp_ns: int,
    points: bytes,
    reserved: bytes = b"\0" * 4,
) -> bytes:
    """手写 27-byte package header 与点负载。"""
    return struct.pack(
        "<BIBBQHBIB4s",
        0,
        7,
        0,
        0,
        timestamp_ns,
        3,
        data_type,
        len(points),
        0,
        reserved,
    ) + points


def _golden_bytes() -> bytes:
    """构造两帧 type-1 golden LVX2。"""
    public = struct.pack("<16s4BI", b"livox_tech\0\0\0\0\0\0", 2, 0, 0, 0, 0xAC0EA767)
    prefix = public + struct.pack("<IB", 50, 1) + _device()
    first_points = struct.pack("<iiiBB", 1000, -2000, 3000, 42, 5)
    second_points = struct.pack("<iiiBB", 11, -12, 13, 9, 2)
    first_package = _package(
        data_type=1,
        timestamp_ns=123456789,
        points=first_points,
        reserved=b"\0i\0n",
    )
    second_package = _package(data_type=1, timestamp_ns=223456789, points=second_points)
    first_offset = len(prefix)
    second_offset = first_offset + 24 + len(first_package)
    eof = second_offset + 24 + len(second_package)
    return b"".join(
        (
            prefix,
            struct.pack("<QQQ", first_offset, second_offset, 0),
            first_package,
            struct.pack("<QQQ", second_offset, eof, 1),
            second_package,
        )
    )


def _empty_frame_bytes() -> bytes:
    """手写一个只含 24-byte frame header 的合法空帧。"""
    public = struct.pack("<16s4BI", b"livox_tech\0\0\0\0\0\0", 2, 0, 0, 0, 0xAC0EA767)
    prefix = public + struct.pack("<IB", 50, 1) + _device()
    frame_offset = len(prefix)
    return prefix + struct.pack("<QQQ", frame_offset, frame_offset + 24, 0)


def _truncated_point_bytes() -> bytes:
    """手写声明 14-byte type-1 点、实际仅 13 bytes 的完整边界文件。"""
    public = struct.pack("<16s4BI", b"livox_tech\0\0\0\0\0\0", 2, 0, 0, 0, 0xAC0EA767)
    prefix = public + struct.pack("<IB", 50, 1) + _device()
    package_header = struct.pack(
        "<BIBBQHBIB4s", 0, 7, 0, 0, 123456789, 3, 1, 14, 0, b"\0" * 4
    )
    frame_offset = len(prefix)
    eof = frame_offset + 24 + len(package_header) + 13
    return prefix + struct.pack("<QQQ", frame_offset, eof, 0) + package_header + b"\0" * 13


def _empty_frames_bytes(frame_count: int) -> bytes:
    """手写多空帧文件，用小 fixture 验证结果对象不会随整文件增长。"""
    public = struct.pack("<16s4BI", b"livox_tech\0\0\0\0\0\0", 2, 0, 0, 0, 0xAC0EA767)
    prefix = public + struct.pack("<IB", 50, 1) + _device()
    frames = []
    offset = len(prefix)
    for index in range(frame_count):
        frames.append(struct.pack("<QQQ", offset, offset + 24, index))
        offset += 24
    return prefix + b"".join(frames)


def _many_packages_frame_bytes(package_count: int) -> bytes:
    """手写单帧多包文件，冻结全量计数与有界详情的分离。"""
    public = struct.pack(
        "<16s4BI", b"livox_tech\0\0\0\0\0\0", 2, 0, 0, 0, 0xAC0EA767
    )
    prefix = public + struct.pack("<IB", 50, 1) + _device()
    point = struct.pack("<iiiBB", 1, 2, 3, 4, 5)
    packages = b"".join(
        _package(data_type=1, timestamp_ns=index, points=point)
        for index in range(package_count)
    )
    frame_offset = len(prefix)
    eof = frame_offset + 24 + len(packages)
    return prefix + struct.pack("<QQQ", frame_offset, eof, 0) + packages


def _oversized_package_bytes() -> bytes:
    """手写 97 个 type-1 点，超过 Mid-360 每包最多 96 点合同。"""
    public = struct.pack("<16s4BI", b"livox_tech\0\0\0\0\0\0", 2, 0, 0, 0, 0xAC0EA767)
    prefix = public + struct.pack("<IB", 50, 1) + _device()
    package = _package(data_type=1, timestamp_ns=123456789, points=b"\0" * (97 * 14))
    frame_offset = len(prefix)
    eof = frame_offset + 24 + len(package)
    return prefix + struct.pack("<QQQ", frame_offset, eof, 0) + package


def _type_2_bytes() -> bytes:
    """手写规范 type-2 包，用于冻结本 oracle 的 type-1-only 合同。"""
    public = struct.pack(
        "<16s4BI", b"livox_tech\0\0\0\0\0\0", 2, 0, 0, 0, 0xAC0EA767
    )
    prefix = public + struct.pack("<IB", 50, 1) + _device()
    point = struct.pack("<hhhBB", 11, -12, 13, 9, 2)
    package = _package(data_type=2, timestamp_ns=123456789, points=point)
    frame_offset = len(prefix)
    eof = frame_offset + 24 + len(package)
    return prefix + struct.pack("<QQQ", frame_offset, eof, 0) + package


def _device_count_bytes(device_count: int) -> bytes:
    """手写零/多设备完整空帧文件，隔离 device count 合同。"""
    public = struct.pack(
        "<16s4BI", b"livox_tech\0\0\0\0\0\0", 2, 0, 0, 0, 0xAC0EA767
    )
    prefix = public + struct.pack("<IB", 50, device_count)
    prefix += b"".join(_device(lidar_id=index + 1) for index in range(device_count))
    frame_offset = len(prefix)
    return prefix + struct.pack("<QQQ", frame_offset, frame_offset + 24, 0)


class _GuardedReader(io.BytesIO):
    """拒绝无界或超过单个合法 package 的读取。"""

    def read(self, size: int = -1) -> bytes:
        assert 0 <= size <= 96 * 14
        return super().read(size)


class _GuardedPath:
    """向 parser 提供 Path 最小表面，并监测每次读取大小。"""

    def __init__(self, payload: bytes) -> None:
        self._payload = payload

    def stat(self) -> SimpleNamespace:
        return SimpleNamespace(st_size=len(self._payload))

    def open(self, mode: str) -> _GuardedReader:
        assert mode == "rb"
        return _GuardedReader(self._payload)


class _StaleStatPath(_GuardedPath):
    """模拟 stat 后文件增长，open 返回内容比旧 stat 大一帧。"""

    def stat(self) -> SimpleNamespace:
        return SimpleNamespace(st_size=len(self._payload) - 24)


def _write(tmp_path: Path, payload: bytes) -> Path:
    path = tmp_path / "golden.lvx2"
    path.write_bytes(payload)
    return path


def test_parse_handwritten_golden_headers_frames_packages_and_points(tmp_path: Path) -> None:
    """oracle 必须独立解析 little-endian header、type-1 点与 EOF offset。"""
    from scripts.verify_lvx2 import parse_lvx2

    parsed = parse_lvx2(_write(tmp_path, _golden_bytes()))

    assert parsed.version == (2, 0, 0, 0)
    assert parsed.frame_duration_ms == 50
    assert parsed.devices[0].lidar_id == 7
    assert parsed.devices[0].device_type == 9
    assert parsed.devices[0].serial_number == b"SLOPESIM00000001"
    assert parsed.devices[0].hub_serial_number == b""
    assert parsed.devices[0].lidar_type == 0
    assert parsed.devices[0].extrinsic_enabled is False
    assert parsed.devices[0].extrinsic == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert parsed.frames[0].current_offset == 92
    assert parsed.frames[0].next_offset == parsed.frames[1].current_offset
    assert parsed.frames[-1].next_offset == parsed.file_size
    assert parsed.complete is True
    assert (parsed.frame_count, parsed.package_count, parsed.point_count) == (2, 2, 2)
    assert [frame.index for frame in parsed.frames] == [0, 1]
    first_package = parsed.frames[0].packages[0]
    assert first_package.version == 0
    assert first_package.lidar_id == 7
    assert first_package.lidar_type == 0
    assert first_package.timestamp_type == 0
    assert first_package.timestamp_raw == struct.pack("<Q", 123456789)
    assert first_package.timestamp_ns == 123456789
    assert first_package.udp_counter == 3
    assert first_package.data_type == 1
    assert first_package.length == 14
    assert first_package.frame_counter == 0
    assert first_package.reserved == b"\0i\0n"
    assert first_package.points == ((1000, -2000, 3000, 42, 5),)
    second_package = parsed.frames[1].packages[0]
    assert second_package.data_type == 1
    assert second_package.length == 14
    assert second_package.points == ((11, -12, 13, 9, 2),)


def test_accepts_handwritten_empty_frame(tmp_path: Path) -> None:
    """空扫描对应的 frame 允许 header 后立即到达 next_offset/EOF。"""
    from scripts.verify_lvx2 import parse_lvx2

    parsed = parse_lvx2(_write(tmp_path, _empty_frame_bytes()))

    assert len(parsed.frames) == 1
    assert parsed.frames[0].next_offset == parsed.frames[0].current_offset + 24
    assert parsed.frames[0].next_offset == parsed.file_size
    assert parsed.frames[0].packages == ()


def test_streams_full_file_but_retains_only_bounded_details() -> None:
    """完整校验可扫描很多帧，但返回对象只保留前两帧的结构细节。"""
    from scripts.verify_lvx2 import parse_lvx2

    parsed = parse_lvx2(_GuardedPath(_empty_frames_bytes(1_000)))

    assert parsed.complete is True
    assert parsed.frame_count == 1_000
    assert len(parsed.frames) == 2
    assert [frame.index for frame in parsed.frames] == [0, 1]


def test_frame_limit_keeps_counts_but_not_unbounded_frame_details() -> None:
    """frame_limit 只限制扫描范围，不能放宽固定的详情保留上限。"""
    from scripts.verify_lvx2 import parse_lvx2

    parsed = parse_lvx2(_GuardedPath(_empty_frames_bytes(1_000)), frame_limit=1_000)

    assert parsed.complete is True
    assert parsed.frame_count == 1_000
    assert len(parsed.frames) == 2
    assert [frame.index for frame in parsed.frames] == [0, 1]


def test_uses_open_stream_eof_instead_of_stale_pre_open_stat() -> None:
    """完整解析必须扫描已打开 stream 的实际 EOF，不能信任旧 stat。"""
    from scripts.verify_lvx2 import parse_lvx2

    parsed = parse_lvx2(_StaleStatPath(_empty_frames_bytes(2)))

    assert parsed.file_size == 140
    assert parsed.complete is True
    assert parsed.frame_count == 2
    assert [frame.index for frame in parsed.frames] == [0, 1]


def test_single_frame_retains_bounded_details_but_counts_every_package() -> None:
    """单帧再多 package 也只保留固定详情，同时完整累计包数和点数。"""
    from scripts.verify_lvx2 import parse_lvx2

    parsed = parse_lvx2(_GuardedPath(_many_packages_frame_bytes(1_000)))

    assert parsed.complete is True
    assert (parsed.frame_count, parsed.package_count, parsed.point_count) == (
        1,
        1_000,
        1_000,
    )
    assert len(parsed.frames) == 1
    assert len(parsed.frames[0].packages) <= 2
    assert sum(len(package.points) for package in parsed.frames[0].packages) <= 4


def test_rejects_package_with_more_than_96_points(tmp_path: Path) -> None:
    """Mid-360 package 不得用超大 length 触发无界点负载读取。"""
    from scripts.verify_lvx2 import Lvx2FormatError, parse_lvx2

    with pytest.raises(Lvx2FormatError, match="96 points"):
        parse_lvx2(_write(tmp_path, _oversized_package_bytes()))


def test_rejects_data_type_2_even_when_structurally_valid(tmp_path: Path) -> None:
    """冻结 writer/oracle 合同仅接受 14-byte type-1 点。"""
    from scripts.verify_lvx2 import Lvx2FormatError, parse_lvx2

    with pytest.raises(Lvx2FormatError, match="data_type 2"):
        parse_lvx2(_write(tmp_path, _type_2_bytes()))


@pytest.mark.parametrize("device_count", (0, 2))
def test_rejects_device_count_other_than_one(
    tmp_path: Path, device_count: int
) -> None:
    """冻结 synthetic Mid-360 writer/oracle 合同恰有一个设备。"""
    from scripts.verify_lvx2 import Lvx2FormatError, parse_lvx2

    with pytest.raises(Lvx2FormatError, match="device_count must be 1"):
        parse_lvx2(_write(tmp_path, _device_count_bytes(device_count)))


@pytest.mark.parametrize(
    ("payload", "message"),
    (
        (b"livox_tech", "truncated"),
        (_golden_bytes()[:102], "truncated frame header"),
        (_truncated_point_bytes(), "truncated package points"),
        (b"x" + _golden_bytes()[1:], "signature"),
        (_golden_bytes()[:16] + b"\x03" + _golden_bytes()[17:], "version"),
        (_golden_bytes()[:20] + b"\0\0\0\0" + _golden_bytes()[24:], "magic"),
        (_golden_bytes()[:24] + struct.pack("<I", 49) + _golden_bytes()[28:], "frame_duration"),
        (_golden_bytes()[:92] + struct.pack("<Q", 93) + _golden_bytes()[100:], "current_offset"),
        (
            _golden_bytes()[:92]
            + struct.pack("<QQQ", 92, 93, 0)
            + _golden_bytes()[116:],
            "next_offset",
        ),
        (_golden_bytes()[:108] + struct.pack("<Q", 1) + _golden_bytes()[116:], "frame index"),
        (_golden_bytes()[:133] + b"\x03" + _golden_bytes()[134:], "data_type"),
        (_golden_bytes()[:66] + b"\x08" + _golden_bytes()[67:], "device_type"),
        (_golden_bytes()[:134] + struct.pack("<I", 13) + _golden_bytes()[138:], "length"),
    ),
)
def test_rejects_truncated_or_corrupt_handwritten_bytes(
    tmp_path: Path, payload: bytes, message: str
) -> None:
    """oracle 对长度、magic、offset 和未知类型 fail-closed。"""
    from scripts.verify_lvx2 import Lvx2FormatError, parse_lvx2

    with pytest.raises(Lvx2FormatError, match=message):
        parse_lvx2(_write(tmp_path, payload))


@pytest.mark.stage4_artifact
def test_reference_sample_header_device_and_first_two_frames_seek_based() -> None:
    """可选官方样例仅按环境变量读取，并只核验 header/device/首两帧。"""
    reference = os.environ.get("LIVOX_LVX2_REFERENCE")
    if not reference:
        pytest.skip("LIVOX_LVX2_REFERENCE is not set")
    from scripts.verify_lvx2 import parse_lvx2

    parsed = parse_lvx2(Path(reference), frame_limit=2)

    assert parsed.version == (2, 0, 0, 0)
    assert parsed.frame_duration_ms == 50
    assert parsed.file_size == 222_540_611
    assert len(parsed.devices) == 1
    device = parsed.devices[0]
    assert device.serial_number == b"47MDK9DF710030"
    assert device.lidar_id == 0x2C01A8C0
    assert device.lidar_type == 247
    assert device.device_type == 9
    assert device.extrinsic_enabled is True
    assert device.extrinsic == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert [frame.index for frame in parsed.frames] == [0, 1]
    assert parsed.frames[0].current_offset == 92
    assert parsed.frames[0].next_offset == 0x22D6C
    assert parsed.frames[1].current_offset == 0x22D6C
    assert parsed.frames[1].next_offset == 0x46A8D
    assert [len(frame.packages) for frame in parsed.frames] == [2, 2]
    assert parsed.complete is False
    assert (parsed.frame_count, parsed.package_count, parsed.point_count) == (
        2,
        211,
        20_256,
    )
    first_package = parsed.frames[0].packages[0]
    assert first_package.version == 0
    assert first_package.lidar_id == 0x2C01A8C0
    assert first_package.lidar_type == 8
    assert first_package.timestamp_type == 0
    assert first_package.timestamp_raw == struct.pack("<Q", 711_620_799_460)
    assert first_package.timestamp_ns == 711_620_799_460
    assert first_package.udp_counter == 17_473
    assert first_package.data_type == 1
    assert first_package.length == 1_344
    assert first_package.frame_counter == 0
    assert first_package.reserved == b"\0i\0n"
    assert len(first_package.points) == 96
    assert first_package.points[0] == (0, 0, 0, 0, 0)
    second_first_package = parsed.frames[1].packages[0]
    assert second_first_package.timestamp_ns == 711_670_719_460
    assert second_first_package.udp_counter == 17_577
