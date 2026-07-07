# 批量实验入口：按多个坡度重复运行 DIRECT 仿真，并汇总每次实验的误差指标。
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

import pandas as pd

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    # 允许直接执行 python experiments/run_slope_sweep.py 时仍能导入项目包。
    sys.path.insert(0, str(PROJECT_ROOT))

from slope_sim.config import load_config
from slope_sim.simulation import run_experiment


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """定义批量坡度实验的命令行参数。"""
    parser = argparse.ArgumentParser(description="Run repeated DIRECT simulations across slope angles.")
    parser.add_argument("--config", default="configs/experiment.yaml", help="Base experiment YAML file.")
    parser.add_argument("--slopes", nargs="+", type=float, default=[0.0, 5.0, 10.0, 15.0, 20.0])
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--duration-sec", type=float, default=None)
    parser.add_argument("--summary", type=Path, default=Path("results/logs/slope_sweep_summary.csv"))
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """逐个坡度和试验编号运行仿真，并写出汇总 CSV。"""
    args = parse_args(argv)
    rows: list[dict[str, float | int | str]] = []
    for slope in args.slopes:
        for trial in range(args.trials):
            # 每次实验都从基础配置加载，再用当前坡度和时长覆盖。
            config = load_config(
                args.config,
                overrides={
                    "mode": "direct",
                    "slope_deg": slope,
                    "duration_sec": args.duration_sec,
                },
            )
            result = run_experiment(config)
            rows.append(
                {
                    "slope_deg": slope,
                    "trial": trial,
                    "log_path": str(result.log_path),
                    "figure_path": str(result.figure_path),
                    **result.metrics,
                }
            )

    args.summary.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(args.summary, index=False)
    print(f"summary: {args.summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
