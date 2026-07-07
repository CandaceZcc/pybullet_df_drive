# 日志模块：把每一步仿真状态写入 CSV，供后续画图和误差分析使用。
from __future__ import annotations

import csv
from pathlib import Path
from time import strftime

from slope_sim.robot import RobotState


class CsvSimulationLogger:
    """按固定字段记录机器人状态、参考路径和估计轨迹。"""

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
        """创建日志文件并写入表头。"""
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
        """记录单个仿真时刻的状态和参考/估计位置。"""
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
        """关闭 CSV 文件，并返回日志路径。"""
        self._file.close()
        return self.path
