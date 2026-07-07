from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from slope_sim.config import load_config
from slope_sim.simulation import run_experiment


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a PyBullet differential-drive slope simulation.")
    parser.add_argument("--config", default="configs/experiment.yaml", help="Path to an experiment YAML file.")
    parser.add_argument("--mode", choices=["direct", "gui"], default=None, help="PyBullet connection mode.")
    parser.add_argument("--gui", action="store_true", help="Shortcut for --mode gui.")
    parser.add_argument("--slope-deg", type=float, default=None, help="Slope angle in degrees.")
    parser.add_argument("--duration-sec", type=float, default=None, help="Simulation duration in seconds.")
    parser.add_argument("--time-step", type=float, default=None, help="Simulation time step in seconds.")
    parser.add_argument("--target-linear-velocity", type=float, default=None, help="Target body velocity in m/s.")
    parser.add_argument("--target-angular-velocity", type=float, default=None, help="Target yaw rate in rad/s.")
    parser.add_argument("--log-dir", type=Path, default=None, help="Directory for CSV logs.")
    parser.add_argument("--figure-dir", type=Path, default=None, help="Directory for generated figures.")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    overrides = {
        "mode": args.mode,
        "gui": args.gui,
        "slope_deg": args.slope_deg,
        "duration_sec": args.duration_sec,
        "time_step": args.time_step,
        "target_linear_velocity": args.target_linear_velocity,
        "target_angular_velocity": args.target_angular_velocity,
        "log_dir": args.log_dir,
        "figure_dir": args.figure_dir,
    }
    config = load_config(args.config, overrides=overrides)
    result = run_experiment(config)
    print(f"log: {result.log_path}")
    print(f"figure: {result.figure_path}")
    for name, value in result.metrics.items():
        print(f"{name}: {value:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

