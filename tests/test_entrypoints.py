# 入口测试：覆盖 main/analysis 的参数解析和日志分析入口。
from pathlib import Path

import pandas as pd
import pytest

from analysis import analyze_log
import main as main_module
from main import parse_args
from slope_sim.simulation import SimulationResult


@pytest.mark.parametrize(
    "arguments",
    [["--robot-model", "tracked_proxy"], ["--terrain-model", "dam_slope"], ["--drive-model", "kinematic"]],
)
def test_main_parse_args_rejects_removed_stage1_options(arguments):
    with pytest.raises(SystemExit):
        parse_args(arguments)


def test_main_parse_args_supports_stage1_model_and_terrain_selection():
    args = parse_args(
        [
            "--gui",
            "--manual",
            "--slope-deg",
            "10",
            "--duration-sec",
            "2",
            "--robot-model",
            "active_steering_4wd",
            "--drive-model",
            "physics",
            "--no-dashboard",
            "--dashboard-update-hz",
            "4",
            "--dashboard-smoothing-alpha",
            "0.2",
            "--lidar",
            "--lidar-debug-draw",
            "--terrain-model",
            "golf_heightfield",
            "--golf-seed",
            "23",
            "--golf-relief",
            "high",
            "--ground-friction",
            "0.8",
            "--ground-rolling-friction",
            "0.03",
            "--ground-spinning-friction",
            "0.04",
            "--wheel-friction",
            "0.6",
            "--support-friction",
            "0.03",
            "--drive-motor-force",
            "2.5",
        ]
    )

    assert args.gui is True
    assert args.manual is True
    assert args.slope_deg == 10.0
    assert args.duration_sec == 2.0
    assert args.robot_model == "active_steering_4wd"
    assert args.drive_model == "physics"
    assert args.no_dashboard is True
    assert args.dashboard_update_hz == 4.0
    assert args.dashboard_smoothing_alpha == 0.2
    assert args.lidar is True
    assert args.lidar_debug_draw is True
    assert args.terrain_model == "golf_heightfield"
    assert args.golf_seed == 23
    assert args.golf_relief == "high"
    assert args.ground_friction == 0.8
    assert args.ground_rolling_friction == 0.03
    assert args.ground_spinning_friction == 0.04
    assert args.wheel_friction == 0.6
    assert args.support_friction == 0.03
    assert args.drive_motor_force == 2.5


def test_manual_mode_runs_until_quit_unless_duration_is_explicit(tmp_path: Path, monkeypatch):
    config_path = tmp_path / "flat_demo.yaml"
    config_path.write_text(
        "\n".join(
            [
                "mode: direct",
                "slope_deg: 0.0",
                "duration_sec: 5.0",
                "time_step: 0.01",
                f"log_dir: {tmp_path / 'logs'}",
                f"figure_dir: {tmp_path / 'figures'}",
            ]
        ),
        encoding="utf-8",
    )
    calls = []

    def fake_run_manual_demo(config, *, duration_limit_sec):
        calls.append((config, duration_limit_sec))
        return SimulationResult(
            log_path=tmp_path / "manual.csv",
            figure_path=tmp_path / "manual.png",
            metrics={"endpoint_error": 0.0},
        )

    monkeypatch.setattr(main_module, "run_manual_demo", fake_run_manual_demo)

    assert main_module.main(["--config", str(config_path), "--gui", "--manual"]) == 0
    assert calls[-1][0].duration_sec == 5.0
    assert calls[-1][1] is None

    assert main_module.main(["--config", str(config_path), "--gui", "--manual", "--duration-sec", "2"]) == 0
    assert calls[-1][0].duration_sec == 2.0
    assert calls[-1][1] == 2.0


def test_main_reports_obstacle_event_log_when_present(tmp_path: Path, monkeypatch, capsys):
    """CLI 输出要包含独立障碍物事件日志路径，方便 GUI 验收后定位结构操作。"""
    monkeypatch.setattr(
        main_module,
        "run_experiment",
        lambda _config: SimulationResult(
            log_path=tmp_path / "run.csv",
            figure_path=tmp_path / "run.png",
            metrics={"endpoint_error": 0.0},
            obstacle_event_log_path=tmp_path / "obstacles.jsonl",
        ),
    )

    assert main_module.main(["--mode", "direct"]) == 0

    assert f"obstacle_event_log: {tmp_path / 'obstacles.jsonl'}" in capsys.readouterr().out


def test_analyze_log_generates_metrics_and_figure(tmp_path: Path):
    log_path = tmp_path / "run.csv"
    pd.DataFrame(
        {
            "x": [0.0, 1.0],
            "y": [0.0, 0.0],
            "yaw": [0.0, 0.0],
            "reference_x": [0.0, 1.2],
            "reference_y": [0.0, 0.0],
            "estimated_x": [0.0, 1.0],
            "estimated_y": [0.0, 0.0],
            "left_slip_ratio": [0.0, 0.1],
            "right_slip_ratio": [0.0, -0.1],
            "left_slip_speed": [0.0, 0.02],
            "right_slip_speed": [0.0, -0.02],
            "left_contact_normal_force": [1.0, 2.0],
            "right_contact_normal_force": [1.1, 2.1],
            "left_contact_friction_force": [0.2, 0.3],
            "right_contact_friction_force": [0.2, 0.4],
        }
    ).to_csv(log_path, index=False)

    metrics, figure_path = analyze_log(log_path, tmp_path / "figures")

    assert metrics["endpoint_error"] > 0.0
    assert figure_path.exists()
    assert (tmp_path / "figures" / "run_slip.png").exists()
    assert (tmp_path / "figures" / "run_contact.png").exists()
