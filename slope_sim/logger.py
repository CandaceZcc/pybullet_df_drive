from __future__ import annotations

import csv
from pathlib import Path
from time import strftime

from slope_sim.robot import RobotState


class CsvSimulationLogger:
    fieldnames = [
        "t",
        "x",
        "y",
        "z",
        "roll",
        "pitch",
        "yaw",
        "linear_velocity",
        "angular_velocity",
        "reference_x",
        "reference_y",
        "estimated_x",
        "estimated_y",
    ]

    def __init__(self, log_dir: str | Path, prefix: str = "run") -> None:
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.log_dir / f"{prefix}_{strftime('%Y%m%d_%H%M%S')}.csv"
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.fieldnames)
        self._writer.writeheader()

    def record(
        self,
        state: RobotState,
        reference_x: float,
        reference_y: float,
        estimated_x: float,
        estimated_y: float,
    ) -> None:
        self._writer.writerow(
            {
                "t": state.t,
                "x": state.x,
                "y": state.y,
                "z": state.z,
                "roll": state.roll,
                "pitch": state.pitch,
                "yaw": state.yaw,
                "linear_velocity": state.linear_velocity,
                "angular_velocity": state.angular_velocity,
                "reference_x": reference_x,
                "reference_y": reference_y,
                "estimated_x": estimated_x,
                "estimated_y": estimated_y,
            }
        )

    def close(self) -> Path:
        self._file.close()
        return self.path

