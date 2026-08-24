# 入口验收测试：覆盖 main/analysis 的参数解析和日志分析入口。
from pathlib import Path
import subprocess
import sys

import pandas as pd
import pytest

from analysis import analyze_log
import main as main_module
from main import parse_args
from slope_sim.simulation import SimulationResult


def test_simulation_and_coordinator_import_without_cycle_in_fresh_process():
    """入口模块必须保持单向依赖，不能依靠当前进程的模块缓存碰巧导入成功。"""
    repository_root = Path(__file__).resolve().parents[2]
    completed = subprocess.run(
        [
            sys.executable,
            "-c",
            "import slope_sim.simulation; import slope_sim.coordinator",
        ],
        cwd=repository_root,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr


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


def test_main_rejects_rc_port_outside_formal_v2_manual_mode(tmp_path: Path, capsys):
    config_path = tmp_path / "local.yaml"
    config_path.write_text("mode: direct\ninterface_mode: local\n", encoding="utf-8")

    assert main_module.main(
        ["--config", str(config_path), "--rc-port", "/dev/serial/by-id/usb-test"]
    ) == 2
    assert "--rc-port requires formal v2 manual interface" in capsys.readouterr().err


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


def test_task12_parse_args_and_main_propagate_interface_overrides(tmp_path: Path, monkeypatch):
    args = parse_args(
        [
            "--interface-mode",
            "ecal",
            "--no-interface",
            "--no-interface-log",
            "--scene-in",
            str(tmp_path / "in.yaml"),
            "--scene-out",
            str(tmp_path / "out.yaml"),
            "--developer-diagnostics",
        ]
    )
    assert args.interface_mode == "ecal"
    assert args.no_interface is True
    assert args.no_interface_log is True
    assert args.scene_in == tmp_path / "in.yaml"
    assert args.scene_out == tmp_path / "out.yaml"
    assert args.developer_diagnostics is True

    captured = {}
    from slope_sim.config import ExperimentConfig

    def fake_load_config(path, overrides):
        captured["path"] = path
        captured["overrides"] = overrides
        return ExperimentConfig()

    monkeypatch.setattr(main_module, "load_config", fake_load_config)
    monkeypatch.setattr(
        main_module,
        "run_experiment",
        lambda _config: SimulationResult(Path("run.csv"), Path("run.png"), {}),
    )

    assert main_module.main(
        [
            "--interface-mode",
            "local",
            "--no-interface",
            "--no-interface-log",
            "--scene-in",
            str(tmp_path / "in.yaml"),
            "--scene-out",
            str(tmp_path / "out.yaml"),
            "--developer-diagnostics",
        ]
    ) == 0

    overrides = captured["overrides"]
    assert overrides["interface_mode"] == "local"
    assert overrides["no_interface"] is True
    assert overrides["no_interface_log"] is True
    assert overrides["scene_in"] == tmp_path / "in.yaml"
    assert overrides["scene_out"] == tmp_path / "out.yaml"
    assert overrides["developer_diagnostics"] is True


def test_task12_main_reports_interface_and_scene_outputs(tmp_path: Path, monkeypatch, capsys):
    monkeypatch.setattr(
        main_module,
        "run_experiment",
        lambda _config: SimulationResult(
            tmp_path / "run.csv",
            tmp_path / "run.png",
            {},
            interface_binary_log=tmp_path / "interface.bin",
            interface_event_log=tmp_path / "interface.jsonl",
            scene_export=tmp_path / "scene.yaml",
        ),
    )

    assert main_module.main(["--mode", "direct"]) == 0

    output = capsys.readouterr().out
    assert f"interface_binary_log: {tmp_path / 'interface.bin'}" in output
    assert f"interface_event_log: {tmp_path / 'interface.jsonl'}" in output
    assert f"scene_export: {tmp_path / 'scene.yaml'}" in output


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
