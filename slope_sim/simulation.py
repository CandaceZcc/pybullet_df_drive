from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.logger import CsvSimulationLogger
from slope_sim.metrics import compute_tracking_metrics
from slope_sim.robot import DifferentialDriveRobot
from slope_sim.scene import create_slope_scene


@dataclass(frozen=True)
class SimulationResult:
    log_path: Path
    figure_path: Path
    metrics: dict[str, float]


def run_experiment(config: ExperimentConfig) -> SimulationResult:
    connection_mode = p.GUI if config.mode == "gui" else p.DIRECT
    client_id = p.connect(connection_mode)
    if client_id < 0:
        raise RuntimeError(f"Failed to connect to PyBullet in {config.mode} mode")

    try:
        create_slope_scene(client_id, config.slope_deg, config.time_step)
        robot = DifferentialDriveRobot(
            client_id=client_id,
            urdf_path=Path("urdf/diff_drive.urdf"),
            wheel_base=config.wheel_base,
            wheel_radius=config.wheel_radius,
        )
        logger = CsvSimulationLogger(config.log_dir, prefix=f"slope_{config.slope_deg:g}")
        steps = max(1, int(config.duration_sec / config.time_step))

        robot.command_twist(config.target_linear_velocity, config.target_angular_velocity)
        for step in range(steps):
            t = step * config.time_step
            state = robot.step_kinematic(dt=config.time_step, slope_deg=config.slope_deg, t=t)
            reference_x = config.target_linear_velocity * t
            reference_y = 0.0
            logger.record(
                state,
                reference_x=reference_x,
                reference_y=reference_y,
                estimated_x=state.x,
                estimated_y=state.y,
            )
            p.stepSimulation(physicsClientId=client_id)

        log_path = logger.close()
    finally:
        p.disconnect(client_id)

    frame = pd.read_csv(log_path)
    metrics = compute_tracking_metrics(frame, final_reference_yaw=0.0)
    figure_path = plot_trajectory(frame, config.figure_dir, prefix=f"slope_{config.slope_deg:g}")
    return SimulationResult(log_path=log_path, figure_path=figure_path, metrics=metrics)


def plot_trajectory(frame: pd.DataFrame, figure_dir: str | Path, prefix: str = "run") -> Path:
    figure_dir = Path(figure_dir)
    figure_dir.mkdir(parents=True, exist_ok=True)
    figure_path = figure_dir / f"{prefix}_trajectory.png"

    fig, ax = plt.subplots(figsize=(6.0, 4.0))
    ax.plot(frame["reference_x"], frame["reference_y"], label="reference", linestyle="--")
    ax.plot(frame["x"], frame["y"], label="actual")
    ax.plot(frame["estimated_x"], frame["estimated_y"], label="estimated", alpha=0.75)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Differential-drive slope tracking")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(figure_path, dpi=150)
    plt.close(fig)
    return figure_path

