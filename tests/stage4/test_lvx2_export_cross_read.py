"""阶段四：用独立 LVX2 oracle 回读 C++ Export 的最小夹具。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess

import pytest

from scripts.verify_lvx2 import parse_lvx2


def _export_test_executable() -> Path:
    raw = os.environ.get("STAGE4_EXPORT_TEST_EXECUTABLE")
    assert raw, "STAGE4_EXPORT_TEST_EXECUTABLE must name the verified C++ export test"
    executable = Path(raw)
    assert executable.is_absolute() and executable.is_file()
    return executable


@pytest.mark.stage4_artifact
def test_cpp_export_is_cross_read_by_independent_oracle_point_for_point(
    tmp_path: Path,
) -> None:
    """Python oracle 必须逐点读回 C++ 生成的标准 LVX2，而非复用 writer 解析器。"""
    completed = subprocess.run(
        [_export_test_executable(), "--cross-read-fixture", str(tmp_path)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr

    expected = json.loads((tmp_path / "expected.json").read_text(encoding="utf-8"))
    parsed = parse_lvx2(tmp_path / expected["lvx2"])
    assert parsed.complete
    assert parsed.frame_count == expected["frame_count"] == 2
    assert parsed.package_count == expected["package_count"]
    assert parsed.point_count == len(expected["points"])
    assert parsed.frames[-1].next_offset == parsed.file_size

    actual_points = []
    for frame in parsed.frames:
        for package in frame.packages:
            for point in package.points:
                actual_points.append(
                    {
                        "x_mm": point[0],
                        "y_mm": point[1],
                        "z_mm": point[2],
                        "reflectivity": point[3],
                        "tag": point[4],
                        "package_timestamp_ns": package.timestamp_ns,
                    }
                )
    expected_points = [
        {
            "x_mm": round(point["x_m"] * 1000),
            "y_mm": round(point["y_m"] * 1000),
            "z_mm": round(point["z_m"] * 1000),
            "reflectivity": point["reflectivity"],
            "tag": point["tag"],
            "package_timestamp_ns": point["package_timestamp_ns"],
        }
        for point in expected["points"]
    ]
    assert actual_points == expected_points
