# 仿真 smoke 测试：用一次短 DIRECT 仿真验证日志和图像能生成。
from pathlib import Path

import pandas as pd
import pytest

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
            duration_sec=1.2,
            time_step=1.0 / 240.0,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    frame = pd.read_csv(result.log_path)
    assert len(frame) > 0
    assert {
        "left_actual_drive_speed",
        "right_actual_drive_speed",
        "left_track_surface_speed",
        "right_track_surface_speed",
        "left_body_track_speed",
        "right_body_track_speed",
        "left_slip_ratio",
        "right_slip_ratio",
        "lidar_min_distance",
    }.issubset(frame.columns)
    assert frame["lidar_min_distance"].notna().all()
    assert frame.iloc[-1]["x"] > frame.iloc[0]["x"]
    tail = frame.tail(120)
    assert abs(tail["yaw_rate"].mean()) < 0.12
    assert abs(tail["left_slip_ratio"].mean()) < 0.15
    assert abs(tail["right_slip_ratio"].mean()) < 0.15


def test_tracked_proxy_physics_turns_in_place_without_locking(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="tracked_proxy",
            drive_model="physics",
            slope_deg=0.0,
            duration_sec=1.5,
            time_step=1.0 / 240.0,
            wheel_base=0.5,
            wheel_radius=0.08,
            target_linear_velocity=0.0,
            target_angular_velocity=0.8,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    frame = pd.read_csv(result.log_path)
    tail_yaw_rate = frame.tail(120)["yaw_rate"].mean()

    assert 0.55 <= tail_yaw_rate <= 0.95
    assert frame["yaw"].iloc[-1] > 0.5


def test_tracked_proxy_physics_forward_turn_keeps_yaw_response(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="tracked_proxy",
            drive_model="physics",
            slope_deg=0.0,
            duration_sec=1.5,
            time_step=1.0 / 240.0,
            wheel_base=0.5,
            wheel_radius=0.08,
            target_linear_velocity=0.35,
            target_angular_velocity=0.8,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    frame = pd.read_csv(result.log_path)
    tail_yaw_rate = frame.tail(120)["yaw_rate"].mean()

    assert tail_yaw_rate > 0.35
    assert frame["yaw"].iloc[-1] > 0.35


def test_diff_drive_physics_starts_with_wheels_on_ground(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="diff_drive",
            drive_model="physics",
            slope_deg=0.0,
            duration_sec=0.05,
            time_step=1.0 / 240.0,
            target_linear_velocity=0.0,
            target_angular_velocity=0.0,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    frame = pd.read_csv(result.log_path)
    assert frame["z"].iloc[0] == pytest.approx(0.14, abs=0.02)


def test_diff_drive_physics_turns_near_commanded_yaw_rate(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="diff_drive",
            drive_model="physics",
            slope_deg=0.0,
            duration_sec=1.5,
            time_step=1.0 / 240.0,
            wheel_base=0.5,
            wheel_radius=0.1,
            target_linear_velocity=0.0,
            target_angular_velocity=0.8,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    frame = pd.read_csv(result.log_path)
    tail_yaw_rate = frame.tail(120)["yaw_rate"].mean()

    assert 0.65 <= tail_yaw_rate <= 0.95


def test_diff_drive_physics_forward_turn_is_smooth(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="diff_drive",
            drive_model="physics",
            slope_deg=0.0,
            duration_sec=1.5,
            time_step=1.0 / 240.0,
            wheel_base=0.5,
            wheel_radius=0.1,
            target_linear_velocity=0.35,
            target_angular_velocity=0.8,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    frame = pd.read_csv(result.log_path)
    tail = frame.tail(120)

    assert 0.65 <= tail["yaw_rate"].mean() <= 0.95
    assert abs(tail["body_lateral_speed"].mean()) < 0.05
    assert tail["yaw_rate"].std() < 0.12
