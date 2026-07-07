# 入口测试：覆盖 main/analysis 的参数解析和日志分析入口。
from pathlib import Path

import pandas as pd

from analysis import analyze_log
import main as main_module
from main import parse_args
from slope_sim.simulation import SimulationResult


def test_main_parse_args_supports_gui_and_slope():
    args = parse_args(["--gui", "--manual", "--slope-deg", "10", "--duration-sec", "2"])

    assert args.gui is True
    assert args.manual is True
    assert args.slope_deg == 10.0
    assert args.duration_sec == 2.0


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
        }
    ).to_csv(log_path, index=False)

    metrics, figure_path = analyze_log(log_path, tmp_path / "figures")

    assert metrics["endpoint_error"] > 0.0
    assert figure_path.exists()
