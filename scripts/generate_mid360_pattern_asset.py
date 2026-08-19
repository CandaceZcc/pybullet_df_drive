#!/usr/bin/env python3
"""从冻结的 Livox MID-360 CSV 确定性派生运行时角度资产。"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import struct


_SOURCE_REPOSITORY = "https://github.com/Livox-SDK/livox_laser_simulation"
_SOURCE_COMMIT = "1cce1073633a062b92e30243a4c2920e45551bb5"
_SOURCE_BLOB = "536686c17fc58d2900a585fefbb6bf21cd5acdee"
_SOURCE_PATH = "scan_mode/mid360.csv"
_SOURCE_HEADER = "Time/s,Azimuth/deg,Zenith/deg"
_SOURCE_SHA256 = "aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a"
_PATTERN_VERSION = "livox-mid360-800000-v1"
_PATTERN_SHA256 = "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca"
_ROW_COUNT = 800_000
_DERIVATION_VERSION = "mid360-angle-pairs-v1"
_DERIVATION_FORMAT = "little-endian <float64 azimuth_deg, float64 zenith_deg>"
_NOTICE = """MIT License

Copyright (c) 2021 livox

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _derive(source: Path) -> bytes:
    if source.is_symlink() or not source.is_file():
        raise ValueError("source must be a regular non-symlink file")
    raw_source = source.read_bytes()
    if _sha256(raw_source) != _SOURCE_SHA256:
        raise ValueError("source SHA-256 does not match the frozen MID-360 CSV")
    first_line = raw_source.splitlines()[0].decode("ascii")
    if first_line != _SOURCE_HEADER:
        raise ValueError("source CSV header does not match the frozen contract")

    derived = bytearray(_ROW_COUNT * 16)
    with source.open("r", encoding="ascii", newline="") as stream:
        reader = csv.reader(stream)
        if next(reader) != _SOURCE_HEADER.split(","):
            raise ValueError("source CSV header does not match the frozen contract")
        for expected_time, row in enumerate(reader, start=1):
            if len(row) != 3 or row[0] != str(expected_time):
                raise ValueError("source CSV time column is not the exact 1..800000 sequence")
            if expected_time > _ROW_COUNT:
                raise ValueError("source CSV contains more than 800000 data rows")
            azimuth = float(row[1])
            zenith = float(row[2])
            if not math.isfinite(azimuth) or not math.isfinite(zenith):
                raise ValueError("source CSV angles must be finite")
            struct.pack_into("<dd", derived, (expected_time - 1) * 16, azimuth, zenith)
        if expected_time != _ROW_COUNT:
            raise ValueError("source CSV must contain exactly 800000 data rows")
    frozen = bytes(derived)
    if _sha256(frozen) != _PATTERN_SHA256:
        raise ValueError("derived MID-360 asset SHA-256 does not match the frozen contract")
    return frozen


def generate(source: Path, output_dir: Path) -> None:
    """校验官方源身份，并一次写出冻结 binary、provenance 和 MIT notice。"""
    if output_dir.exists() or not output_dir.parent.is_dir():
        raise ValueError("output-dir must be a new directory below an existing parent")
    frozen = _derive(source)
    provenance = {
        "schema_version": 1,
        "pattern_version": _PATTERN_VERSION,
        "source": {
            "repository": _SOURCE_REPOSITORY,
            "commit": _SOURCE_COMMIT,
            "blob": _SOURCE_BLOB,
            "path": _SOURCE_PATH,
            "header": _SOURCE_HEADER,
            "row_count": _ROW_COUNT,
            "sha256": _SOURCE_SHA256,
        },
        "derivation": {
            "version": _DERIVATION_VERSION,
            "format": _DERIVATION_FORMAT,
            "row_count": _ROW_COUNT,
            "byte_size": len(frozen),
            "sha256": _PATTERN_SHA256,
        },
    }
    output_dir.mkdir(mode=0o755)
    (output_dir / "mid360_pattern.bin").write_bytes(frozen)
    (output_dir / "mid360_pattern.provenance.json").write_text(
        json.dumps(provenance, sort_keys=True, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    (output_dir / "livox_laser_simulation.LICENSE").write_text(
        _NOTICE,
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    generate(args.source, args.output_dir)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError) as error:
        raise SystemExit(f"error: {error}")
