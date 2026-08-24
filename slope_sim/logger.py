# 日志模块：把仿真遥测写入 CSV，并把阶段二障碍物结构事件写入独立 JSONL。
from __future__ import annotations

import csv
import json
import math
from numbers import Real
from pathlib import Path
from time import strftime
from typing import Any

from slope_sim.robot import RobotState
from slope_sim.runtime_actions import TerrainSelection
from slope_sim.telemetry import TELEMETRY_FIELDNAMES


class CsvSimulationLogger:
    """按固定字段记录机器人状态、参考路径和估计轨迹。"""

    fieldnames = TELEMETRY_FIELDNAMES

    def __init__(
        self,
        log_dir: str | Path,
        prefix: str = "run",
        *,
        rate_hz: Real = 20.0,
    ) -> None:
        """创建日志文件；默认仅以 20 Hz 将遥测采样落盘。"""
        if isinstance(rate_hz, bool) or not isinstance(rate_hz, Real) or not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("rate_hz must be a positive finite number")
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.log_dir / f"{prefix}_{strftime('%Y%m%d_%H%M%S')}.csv"
        self._file = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=self.fieldnames)
        self._writer.writeheader()
        self._sample_period_sec = 1.0 / float(rate_hz)
        self._next_sample_time: float | None = None

    def record(
        self,
        state: RobotState,
        reference_x: float,
        reference_y: float,
        estimated_x: float,
        estimated_y: float,
    ) -> None:
        """到达采样时间才记录单个仿真状态，物理和控制循环不受影响。"""
        if (
            self._next_sample_time is not None
            and state.t + 1e-9 < self._next_sample_time
        ):
            return
        self._writer.writerow(state.to_row(reference_x, reference_y, estimated_x, estimated_y))
        self._next_sample_time = state.t + self._sample_period_sec

    def close(self) -> Path:
        """关闭 CSV 文件，并返回日志路径。"""
        self._file.close()
        return self.path


class ObstacleEventLogger:
    """逐行记录障碍物结构事件，和高频车辆遥测 CSV 分开存储。"""

    def __init__(self, log_dir: str | Path, prefix: str = "obstacles") -> None:
        """创建 JSONL 文件；每次写入后 flush，便于异常后检查最后操作。"""
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.log_dir / f"{prefix}_{strftime('%Y%m%d_%H%M%S')}.jsonl"
        self._file = self.path.open("w", encoding="utf-8")
        self._closed = False

    def record_event(
        self,
        *,
        sim_time: float,
        event_type: str,
        logical_id: int | None,
        request_params: dict[str, Any] | None,
        seed: int | None,
        robot_model: str,
        terrain: TerrainSelection,
        success: bool,
        error_reason: str | None,
    ) -> None:
        """写入一个稳定字段事件；None 会按 JSON null 保留，方便后续分析。"""
        if self._closed:
            raise ValueError("obstacle event logger is closed")
        row = {
            "sim_time": float(sim_time),
            "event_type": str(event_type),
            "logical_id": None if logical_id is None else int(logical_id),
            "request_params": request_params or {},
            "seed": None if seed is None else int(seed),
            "robot_model": str(robot_model),
            "terrain": {
                "terrain_model": terrain.terrain_model,
                "slope_deg": terrain.slope_deg,
                "golf_seed": terrain.golf_seed,
                "golf_relief": terrain.golf_relief,
            },
            "success": bool(success),
            "error_reason": error_reason,
        }
        self._file.write(json.dumps(row, ensure_ascii=False, sort_keys=False) + "\n")
        self._file.flush()

    def close(self) -> Path:
        """幂等关闭 JSONL 文件，并返回事件日志路径。"""
        if not self._closed:
            self._file.close()
            self._closed = True
        return self.path
