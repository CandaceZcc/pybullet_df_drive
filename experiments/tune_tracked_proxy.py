# 履带参数调优脚本：批量扫描电机力、各向异性摩擦和驱动模式，筛选可用候选。
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Sequence

import pandas as pd

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    # 允许直接执行 python experiments/tune_tracked_proxy.py。
    sys.path.insert(0, str(PROJECT_ROOT))

from slope_sim.config import load_config
from slope_sim.simulation import run_experiment


FORCE_VALUES = (1.5, 2.5, 3.5, 5.0)
TRACK_FRICTION_Y_VALUES = (0.05, 0.1, 0.2, 0.4)
TRACK_DRIVE_MODES = ("all_rollers", "center_only")
BASE_TRACK_FRICTION_XZ = (2.0, 0.05)


def parameter_grid() -> list[dict[str, float | str]]:
    """生成计划中固定的履带参数扫描网格。"""
    rows: list[dict[str, float | str]] = []
    for force in FORCE_VALUES:
        for friction_y in TRACK_FRICTION_Y_VALUES:
            for drive_mode in TRACK_DRIVE_MODES:
                rows.append(
                    {
                        "drive_motor_force": force,
                        "track_anisotropic_friction_y": friction_y,
                        "track_drive_mode": drive_mode,
                    }
                )
    return rows


def tuning_candidate_is_acceptable(straight_summary: dict[str, float], turn_summary: dict[str, float]) -> bool:
    """按计划中的漂移、打滑和转向响应阈值筛选候选参数。"""
    return (
        abs(float(straight_summary["drift_slope"])) < 0.02
        and float(straight_summary["mean_abs_slip"]) < 0.10
        and 0.55 <= abs(float(turn_summary["mean_yaw_rate"])) <= 0.95
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """定义履带参数扫描命令行参数。"""
    parser = argparse.ArgumentParser(description="Tune tracked_proxy drive force and friction parameters.")
    parser.add_argument("--config", default="configs/step3_feedback.yaml", help="Base YAML config.")
    parser.add_argument("--duration-sec", type=float, default=1.5)
    parser.add_argument("--summary", type=Path, default=Path("results/logs/tracked_proxy_tuning_summary.csv"))
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """运行直行和原地转向扫描，并输出候选汇总 CSV。"""
    args = parse_args(argv)
    rows: list[dict[str, float | str | bool]] = []
    for params in parameter_grid():
        base_overrides = {
            "mode": "direct",
            "robot_model": "tracked_proxy",
            "drive_model": "physics",
            "wheel_radius": 0.08,
            "duration_sec": args.duration_sec,
            "drive_motor_force": params["drive_motor_force"],
            "track_anisotropic_friction": [BASE_TRACK_FRICTION_XZ[0], params["track_anisotropic_friction_y"], BASE_TRACK_FRICTION_XZ[1]],
            "track_drive_mode": params["track_drive_mode"],
        }
        straight = run_experiment(
            load_config(
                args.config,
                overrides={
                    **base_overrides,
                    "target_linear_velocity": 0.25,
                    "target_angular_velocity": 0.0,
                },
            )
        )
        turn = run_experiment(
            load_config(
                args.config,
                overrides={
                    **base_overrides,
                    "target_linear_velocity": 0.0,
                    "target_angular_velocity": 0.8,
                },
            )
        )
        straight_summary = straight.diagnostic_summary or {}
        turn_frame = pd.read_csv(turn.log_path)
        turn_summary = {
            **(turn.diagnostic_summary or {}),
            "mean_yaw_rate": float(turn_frame.tail(min(120, len(turn_frame)))["yaw_rate"].mean()),
        }
        rows.append(
            {
                **params,
                "straight_log": str(straight.log_path),
                "turn_log": str(turn.log_path),
                "straight_drift_slope": straight_summary.get("drift_slope"),
                "straight_mean_abs_slip": straight_summary.get("mean_abs_slip"),
                "turn_mean_yaw_rate": turn_summary["mean_yaw_rate"],
                "acceptable": tuning_candidate_is_acceptable(straight_summary, turn_summary),
            }
        )

    args.summary.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(args.summary, index=False)
    print(f"summary: {args.summary}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
