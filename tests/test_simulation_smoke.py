# 仿真 smoke 测试：用一次短 DIRECT 仿真验证日志和图像能生成。
from pathlib import Path

import pandas as pd

from slope_sim.config import ExperimentConfig
from slope_sim.simulation import run_experiment


def test_run_experiment_direct_generates_log_and_figure(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            slope_deg=5.0,
            duration_sec=0.2,
            time_step=1.0 / 120.0,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    assert result.log_path.exists()
    assert result.figure_path.exists()
    assert result.metrics["endpoint_error"] >= 0.0

    frame = pd.read_csv(result.log_path)
    assert len(frame) > 0
    assert {"x", "y", "z", "roll", "pitch", "yaw"}.issubset(frame.columns)
