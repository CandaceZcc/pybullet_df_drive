# 指标模块：根据 CSV 轨迹数据计算终点误差、平均误差和航向误差。
from __future__ import annotations

import math

import numpy as np
import pandas as pd


def compute_tracking_metrics(frame: pd.DataFrame, final_reference_yaw: float = 0.0) -> dict[str, float]:
    """从轨迹表中计算常用跟踪误差指标。"""
    required = {"x", "y", "yaw", "reference_x", "reference_y"}
    missing = sorted(required - set(frame.columns))
    if missing:
        raise ValueError(f"Missing metric columns: {', '.join(missing)}")
    if frame.empty:
        raise ValueError("Cannot compute metrics for an empty trajectory")

    # 每个时刻的平面位置误差，用于终点、平均和最大误差统计。
    dx = frame["x"].to_numpy() - frame["reference_x"].to_numpy()
    dy = frame["y"].to_numpy() - frame["reference_y"].to_numpy()
    errors = np.hypot(dx, dy)
    heading_error = abs(_wrap_angle(float(frame.iloc[-1]["yaw"]) - final_reference_yaw))
    return {
        "endpoint_error": float(errors[-1]),
        "mean_tracking_error": float(errors.mean()),
        "max_tracking_error": float(errors.max()),
        "heading_error": heading_error,
    }


def _wrap_angle(angle: float) -> float:
    """把角度归一化到 [-pi, pi)，方便计算最短航向误差。"""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
