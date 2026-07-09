# 履带参数扫描测试：保护扫描网格和候选筛选规则，避免调参脚本变成手工判断。
from experiments.tune_tracked_proxy import parameter_grid, tuning_candidate_is_acceptable


def test_parameter_grid_covers_force_friction_and_drive_mode():
    grid = list(parameter_grid())

    assert len(grid) == 32
    assert {
        "drive_motor_force": 1.5,
        "track_anisotropic_friction_y": 0.05,
        "track_drive_mode": "all_rollers",
    } in grid
    assert {
        "drive_motor_force": 5.0,
        "track_anisotropic_friction_y": 0.4,
        "track_drive_mode": "center_only",
    } in grid


def test_tuning_candidate_requires_low_drift_low_slip_and_turning_response():
    straight = {"drift_slope": 0.01, "mean_abs_slip": 0.08}
    turn = {"mean_yaw_rate": 0.7}

    assert tuning_candidate_is_acceptable(straight, turn) is True
    assert tuning_candidate_is_acceptable({"drift_slope": 0.03, "mean_abs_slip": 0.08}, turn) is False
    assert tuning_candidate_is_acceptable({"drift_slope": 0.01, "mean_abs_slip": 0.12}, turn) is False
    assert tuning_candidate_is_acceptable(straight, {"mean_yaw_rate": 0.4}) is False
