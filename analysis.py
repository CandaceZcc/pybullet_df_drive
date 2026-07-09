# 日志分析入口：读取 CSV 轨迹日志，重新生成误差指标和轨迹图。
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

import pandas as pd

from slope_sim.metrics import compute_tracking_metrics
from slope_sim.simulation import plot_feedback_figures, plot_trajectory


def analyze_log(log_path: str | Path, figure_dir: str | Path = "results/figures") -> tuple[dict[str, float], Path]:
    """分析单个 CSV 日志，并输出指标字典和轨迹图路径。"""
    frame = pd.read_csv(log_path)
    metrics = compute_tracking_metrics(frame, final_reference_yaw=0.0)
    figure_path = plot_trajectory(frame, figure_dir, prefix=Path(log_path).stem)
    plot_feedback_figures(frame, figure_dir, prefix=Path(log_path).stem)
    return metrics, figure_path


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """定义日志分析脚本的命令行参数。"""
    parser = argparse.ArgumentParser(description="Analyze a slope simulation CSV log.")
    parser.add_argument("--log", required=True, type=Path, help="Path to a CSV log.")
    parser.add_argument("--figure-dir", default=Path("results/figures"), type=Path, help="Output figure directory.")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """命令行入口：读取日志、生成图表、打印误差指标。"""
    args = parse_args(argv)
    metrics, figure_path = analyze_log(args.log, args.figure_dir)
    print(f"figure: {figure_path}")
    for name, value in metrics.items():
        print(f"{name}: {value:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
