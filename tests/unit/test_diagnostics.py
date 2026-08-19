# 诊断指标单元测试：把一次实验 CSV 压缩成便于调参的漂移、打滑和接触摘要。
from pathlib import Path

import json

import pandas as pd
import pytest

from slope_sim.diagnostics import compute_diagnostic_summary, write_diagnostic_summary


def test_compute_diagnostic_summary_measures_drift_slip_and_contact_balance():
    frame = pd.DataFrame(
        {
            "x": [0.0, 2.0],
            "y": [0.0, 1.0],
            "yaw": [0.1, -0.3],
            "left_slip_ratio": [0.1, -0.2],
            "right_slip_ratio": [-0.3, 0.0],
            "left_contact_normal_force": [10.0, 10.0],
            "right_contact_normal_force": [20.0, 20.0],
        }
    )

    summary = compute_diagnostic_summary(frame)

    assert summary.drift_slope == pytest.approx(0.5)
    assert summary.mean_abs_slip == pytest.approx(0.15)
    assert summary.max_abs_yaw == pytest.approx(0.3)
    assert summary.contact_imbalance == pytest.approx(1.0 / 3.0)


def test_write_diagnostic_summary_creates_json_next_to_log(tmp_path: Path):
    log_path = tmp_path / "run.csv"
    frame = pd.DataFrame(
        {
            "x": [0.0, 1.0],
            "y": [0.0, 0.0],
            "yaw": [0.0, 0.0],
            "left_slip_ratio": [0.1, 0.1],
            "right_slip_ratio": [0.2, 0.2],
        }
    )

    summary_path = write_diagnostic_summary(log_path, compute_diagnostic_summary(frame))

    assert summary_path == tmp_path / "run.summary.json"
    payload = json.loads(summary_path.read_text(encoding="utf-8"))
    assert payload["drift_slope"] == 0.0
    assert payload["mean_abs_slip"] == pytest.approx(0.15)
