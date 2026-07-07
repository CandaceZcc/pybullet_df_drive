from pathlib import Path

import pandas as pd

from analysis import analyze_log
from main import parse_args


def test_main_parse_args_supports_gui_and_slope():
    args = parse_args(["--gui", "--slope-deg", "10", "--duration-sec", "2"])

    assert args.gui is True
    assert args.slope_deg == 10.0
    assert args.duration_sec == 2.0


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

