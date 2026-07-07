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


def test_run_experiment_physics_tracked_proxy_records_telemetry(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="tracked_proxy",
            drive_model="physics",
            lidar_enabled=True,
            lidar_ray_count=9,
            slope_deg=0.0,
            duration_sec=0.3,
            time_step=1.0 / 120.0,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    frame = pd.read_csv(result.log_path)
    assert len(frame) > 0
    assert {
        "left_actual_drive_speed",
        "right_actual_drive_speed",
        "left_slip_ratio",
        "right_slip_ratio",
        "lidar_min_distance",
    }.issubset(frame.columns)
    assert frame["lidar_min_distance"].notna().all()
    assert frame.iloc[-1]["x"] > frame.iloc[0]["x"]
