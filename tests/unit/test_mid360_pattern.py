"""MID-360 冻结扫描表的资产、phase、坐标和打包身份测试。"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import zipfile

import pytest

import slope_sim.lidar_pointcloud as lidar_pointcloud


ROOT = Path(__file__).resolve().parents[2]
ASSET_DIR = ROOT / "slope_sim" / "assets"
PATTERN = ASSET_DIR / "mid360_pattern.bin"
PROVENANCE = ASSET_DIR / "mid360_pattern.provenance.json"
NOTICE = ASSET_DIR / "livox_laser_simulation.LICENSE"
SOURCE = ROOT / "references" / "repos" / "livox_laser_simulation" / "scan_mode" / "mid360.csv"
GENERATOR = ROOT / "scripts" / "generate_mid360_pattern_asset.py"

PATTERN_VERSION = "livox-mid360-800000-v1"
SOURCE_SHA256 = "aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a"
PATTERN_SHA256 = "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca"
NOTICE_SHA256 = "4846843fe86275ad7a08915992fe69019608aebb783b3fb84e35a68d4611ad26"
ASSET_NAMES = (
    "mid360_pattern.bin",
    "mid360_pattern.provenance.json",
    "livox_laser_simulation.LICENSE",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_mid360_checked_in_asset_and_provenance_freeze_official_source() -> None:
    """派生角度表必须完整、可追溯，运行时资产不得以 synthetic fallback 代替。"""
    assert PATTERN.is_file(), "the frozen MID-360 binary asset is missing"
    assert PROVENANCE.is_file(), "the MID-360 provenance JSON is missing"
    assert NOTICE.is_file(), "the upstream MIT notice is missing"
    assert PATTERN.stat().st_size == 12_800_000
    assert _sha256(PATTERN) == PATTERN_SHA256
    assert _sha256(NOTICE) == NOTICE_SHA256

    provenance = json.loads(PROVENANCE.read_text(encoding="utf-8"))
    assert provenance == {
        "schema_version": 1,
        "pattern_version": PATTERN_VERSION,
        "source": {
            "repository": "https://github.com/Livox-SDK/livox_laser_simulation",
            "commit": "1cce1073633a062b92e30243a4c2920e45551bb5",
            "blob": "536686c17fc58d2900a585fefbb6bf21cd5acdee",
            "path": "scan_mode/mid360.csv",
            "header": "Time/s,Azimuth/deg,Zenith/deg",
            "row_count": 800_000,
            "sha256": SOURCE_SHA256,
        },
        "derivation": {
            "version": "mid360-angle-pairs-v1",
            "format": "little-endian <float64 azimuth_deg, float64 zenith_deg>",
            "row_count": 800_000,
            "byte_size": 12_800_000,
            "sha256": PATTERN_SHA256,
        },
    }
    assert "MIT License" in NOTICE.read_text(encoding="utf-8")
    assert "Copyright (c) 2021 livox" in NOTICE.read_text(encoding="utf-8")


def test_mid360_asset_generator_is_deterministic_and_validates_the_source(tmp_path: Path) -> None:
    """生成器从官方 CSV 确定性重建三项资产，并核对冻结 source identity。"""
    assert GENERATOR.is_file(), "the deterministic MID-360 asset generator is missing"
    generated = tmp_path / "assets"
    completed = subprocess.run(
        [
            sys.executable,
            str(GENERATOR),
            "--source",
            str(SOURCE),
            "--output-dir",
            str(generated),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert tuple(sorted(path.name for path in generated.iterdir())) == tuple(sorted(ASSET_NAMES))
    for name in ASSET_NAMES:
        assert (generated / name).read_bytes() == (ASSET_DIR / name).read_bytes()


@pytest.mark.parametrize(
    ("world_generation", "sequence", "global_slot", "expected"),
    (
        (1, 0, 0, 17_365),
        (1, 0, 5_759, 23_124),
        (1, 1, 0, 23_125),
        (2, 0, 0, 795_583),
        (2, 0, 5_759, 1_342),
    ),
)
def test_mid360_phase_matches_frozen_vectors(
    world_generation: int,
    sequence: int,
    global_slot: int,
    expected: int,
) -> None:
    phase = getattr(lidar_pointcloud, "mid360_pattern_row_index", None)
    assert callable(phase), "the frozen MID-360 phase function is missing"

    assert phase(PATTERN_VERSION, world_generation, sequence, global_slot) == expected


@pytest.mark.parametrize(
    ("pattern_version", "world_generation", "sequence", "global_slot"),
    (
        ("unknown", 1, 0, 0),
        (PATTERN_VERSION, 0, 0, 0),
        (PATTERN_VERSION, True, 0, 0),
        (PATTERN_VERSION, 1, -1, 0),
        (PATTERN_VERSION, 1, 1 << 64, 0),
        (PATTERN_VERSION, 1, 0, -1),
        (PATTERN_VERSION, 1, 0, 5_760),
    ),
)
def test_mid360_phase_rejects_non_frozen_identity_or_out_of_range_slots(
    pattern_version: object,
    world_generation: object,
    sequence: object,
    global_slot: object,
) -> None:
    phase = getattr(lidar_pointcloud, "mid360_pattern_row_index", None)
    assert callable(phase), "the frozen MID-360 phase function is missing"

    with pytest.raises(ValueError):
        phase(pattern_version, world_generation, sequence, global_slot)


def test_mid360_phase_base_is_computed_once_per_world_generation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """同一 world 的多 sequence/slot 必须共享 phase base，且缓存前仍严格验参。"""
    lidar_pointcloud._mid360_pattern_base_for_identity.cache_clear()
    calls = []
    actual_sha256 = lidar_pointcloud.hashlib.sha256

    def recording_sha256(value: bytes = b""):
        calls.append(value)
        return actual_sha256(value)

    monkeypatch.setattr(lidar_pointcloud.hashlib, "sha256", recording_sha256)
    for sequence, global_slot in ((1, 0), (1, 2), (2, 1), (2, 3)):
        lidar_pointcloud.mid360_line_for_slot(
            PATTERN_VERSION,
            41,
            sequence,
            global_slot,
        )

    assert len(calls) == 1
    with pytest.raises(ValueError):
        lidar_pointcloud.mid360_line_for_slot(PATTERN_VERSION, True, 1, 0)


def test_mid360_direction_uses_azimuth_and_zenith_in_x_forward_z_up_coordinates() -> None:
    """row 17365 的 zenith 小于 90 度，因此正确方向的 z 必须为正。"""
    direction_for_slot = getattr(lidar_pointcloud, "mid360_direction_for_slot", None)
    assert callable(direction_for_slot), "the MID-360 direction lookup is missing"

    actual = direction_for_slot(PATTERN_VERSION, 1, 0, 0)
    azimuth = math.radians(302.06)
    zenith = math.radians(55.504)
    expected = (
        math.sin(zenith) * math.cos(azimuth),
        math.sin(zenith) * math.sin(azimuth),
        math.cos(zenith),
    )

    assert actual == pytest.approx(expected, abs=1e-12)
    assert actual[2] > 0.0


def test_mid360_offsets_and_lines_follow_global_firing_slot_and_pattern_row() -> None:
    offset = getattr(lidar_pointcloud, "mid360_offset_time_ns", None)
    line = getattr(lidar_pointcloud, "mid360_line_for_slot", None)
    assert callable(offset), "the MID-360 firing offset function is missing"
    assert callable(line), "the MID-360 line lookup is missing"

    offsets = tuple(offset(slot) for slot in range(5_760))
    assert offsets[0] == 0
    assert offsets[2_880] == 50_000_000
    assert offsets[-1] == 99_982_638
    assert all(left < right for left, right in zip(offsets, offsets[1:]))
    assert tuple(line(PATTERN_VERSION, 1, 0, slot) for slot in range(8)) == (
        1,
        2,
        3,
        0,
        1,
        2,
        3,
        0,
    )


def test_mid360_binary_rows_are_little_endian_float64_angle_pairs() -> None:
    assert PATTERN.is_file(), "the frozen MID-360 binary asset is missing"
    raw = PATTERN.read_bytes()

    assert struct.unpack_from("<dd", raw, 0) == pytest.approx((268.99, 37.838))
    assert struct.unpack_from("<dd", raw, 17_365 * 16) == pytest.approx((302.06, 55.504))


def test_built_wheel_contains_all_frozen_mid360_assets(tmp_path: Path) -> None:
    """实际 wheel 必须携带 binary、provenance 和 MIT notice，而非仅源树可见。"""
    wheel_dir = tmp_path / "wheel"
    wheel_dir.mkdir()
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "pip",
            "wheel",
            "--no-deps",
            "--no-build-isolation",
            "--wheel-dir",
            str(wheel_dir),
            str(ROOT),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    wheels = tuple(wheel_dir.glob("*.whl"))
    assert len(wheels) == 1
    with zipfile.ZipFile(wheels[0]) as archive:
        names = set(archive.namelist())
        for asset_name in ASSET_NAMES:
            assert f"slope_sim/assets/{asset_name}" in names
