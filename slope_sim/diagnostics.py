# 诊断模块：把一次仿真日志压缩成漂移、打滑和接触平衡等调参指标。
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import pandas as pd


@dataclass(frozen=True)
class DiagnosticSummary:
    """一次实验的调参摘要指标。"""

    drift_slope: float = 0.0
    mean_abs_slip: float = math.nan
    max_abs_yaw: float = math.nan
    contact_imbalance: float = math.nan

    def to_dict(self) -> dict[str, float]:
        """转换为可写入 JSON/CSV 的普通字典。"""
        return asdict(self)


def compute_diagnostic_summary(frame: pd.DataFrame) -> DiagnosticSummary:
    """从 CSV DataFrame 计算用于判断漂移和打滑的摘要指标。"""
    drift_slope = _drift_slope(frame)
    mean_abs_slip = _mean_abs_columns(frame, ("left_slip_ratio", "right_slip_ratio"))
    max_abs_yaw = float(frame["yaw"].abs().max()) if "yaw" in frame else math.nan
    contact_imbalance = _contact_imbalance(frame)
    return DiagnosticSummary(
        drift_slope=drift_slope,
        mean_abs_slip=mean_abs_slip,
        max_abs_yaw=max_abs_yaw,
        contact_imbalance=contact_imbalance,
    )


def write_diagnostic_summary(log_path: str | Path, summary: DiagnosticSummary) -> Path:
    """把摘要指标写到日志旁边，便于一次实验结束后直接查看。"""
    path = Path(log_path).with_suffix(".summary.json")
    path.write_text(json.dumps(summary.to_dict(), indent=2, ensure_ascii=False), encoding="utf-8")
    return path


def _drift_slope(frame: pd.DataFrame) -> float:
    """用起终点 dy/dx 表示整体横向漂移斜率。"""
    if "x" not in frame or "y" not in frame or frame.empty:
        return math.nan
    dx = float(frame["x"].iloc[-1] - frame["x"].iloc[0])
    dy = float(frame["y"].iloc[-1] - frame["y"].iloc[0])
    if abs(dx) < 1e-9:
        return 0.0
    return dy / dx


def _mean_abs_columns(frame: pd.DataFrame, columns: tuple[str, ...]) -> float:
    """对存在的列计算平均绝对值；列缺失时返回 NaN。"""
    existing = [column for column in columns if column in frame]
    if not existing:
        return math.nan
    return float(frame[existing].abs().mean(axis=1).mean())


def _contact_imbalance(frame: pd.DataFrame) -> float:
    """用左右平均法向力差占总法向力比例衡量接触不平衡。"""
    columns = {"left_contact_normal_force", "right_contact_normal_force"}
    if not columns.issubset(frame.columns):
        return math.nan
    left = float(frame["left_contact_normal_force"].mean())
    right = float(frame["right_contact_normal_force"].mean())
    total = abs(left) + abs(right)
    if total < 1e-9:
        return 0.0
    return abs(left - right) / total
