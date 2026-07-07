# 指标测试：确保轨迹误差和航向误差计算符合预期。
import pandas as pd
import pytest

from slope_sim.metrics import compute_tracking_metrics


def test_compute_tracking_metrics_from_logged_trajectory():
    frame = pd.DataFrame(
        {
            "x": [0.0, 1.0, 2.0],
            "y": [0.0, 1.0, 2.0],
            "yaw": [0.0, 0.1, 0.2],
            "reference_x": [0.0, 1.0, 3.0],
            "reference_y": [0.0, 1.0, 2.0],
        }
    )

    metrics = compute_tracking_metrics(frame, final_reference_yaw=0.3)

    assert metrics["endpoint_error"] == pytest.approx(1.0)
    assert metrics["mean_tracking_error"] == pytest.approx(1.0 / 3.0)
    assert metrics["max_tracking_error"] == pytest.approx(1.0)
    assert metrics["heading_error"] == pytest.approx(0.1)
