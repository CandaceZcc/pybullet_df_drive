# 阶段一示例配置测试：保护平面、斜面和高尔夫 GUI 入口。
from pathlib import Path

from slope_sim.config import load_config


def test_flat_demo_config_runs_on_flat_ground():
    config = load_config(Path("configs/flat_demo.yaml"))
    assert config.mode == "direct"
    assert config.terrain_model == "flat"
    assert config.slope_deg == 0.0
    assert config.target_linear_velocity > 0.0


def test_slope_feedback_config_uses_mid_drive_on_continuous_slope():
    config = load_config(Path("configs/step3_feedback.yaml"))
    assert config.robot_model == "df_mid"
    assert config.terrain_model == "slope"
    assert config.slope_deg == 5.0


def test_golf_gui_config_uses_active_steering_and_reproducible_seed():
    config = load_config(Path("configs/stage1_golf_gui.yaml"))
    assert config.robot_model == "active_steering_4wd"
    assert config.terrain_model == "golf_heightfield"
    assert config.golf_seed == 41


def test_stage2_obstacle_gui_config_opens_dashboard_on_reproducible_golf_scene():
    config = load_config(Path("configs/stage2_obstacles_gui.yaml"))

    assert config.mode == "gui"
    assert config.dashboard_enabled is True
    assert config.robot_model == "df_back"
    assert config.terrain_model == "golf_heightfield"
    assert config.golf_seed == 2026
    assert config.golf_relief == "medium"
