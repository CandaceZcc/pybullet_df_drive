from __future__ import annotations

import math

import numpy as np
import pandas as pd


def compute_tracking_metrics(frame: pd.DataFrame, final_reference_yaw: float = 0.0) -> dict[str, float]:
    required = {"x", "y", "yaw", "reference_x", "reference_y"}
    missing = sorted(required - set(frame.columns))
    if missing:
        raise ValueError(f"Missing metric columns: {', '.join(missing)}")
    if frame.empty:
        raise ValueError("Cannot compute metrics for an empty trajectory")

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
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

