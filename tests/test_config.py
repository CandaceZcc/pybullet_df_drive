# 配置测试：阶段一只允许四种车型、三类场地和高尔夫复现参数。
from __future__ import annotations

import math
from pathlib import Path

import pytest

from slope_sim.config import ExperimentConfig, load_config
from slope_sim.model_registry import robot_model_names
from slope_sim.scene import terrain_model_names


def test_load_config_reads_stage1_yaml_and_applies_overrides(tmp_path: Path):
    config_path = tmp_path / "stage1.yaml"
    config_path.write_text(
        "\n".join(
            [
                "mode: direct",
                "slope_deg: 8",
                "duration_sec: 3.0",
                "robot_model: df_front",
                "terrain_model: golf_heightfield",
                "golf_seed: 23",
                "golf_relief: high",
                "drive_model: physics",
                "dashboard_enabled: false",
                "log_dir: custom/logs",
                "figure_dir: custom/figures",
            ]
        ),
        encoding="utf-8",
    )

    config = load_config(config_path, overrides={"slope_deg": 12, "gui": True})

    assert config.mode == "gui"
    assert config.slope_deg == 12
    assert config.robot_model == "df_front"
    assert config.terrain_model == "golf_heightfield"
    assert config.golf_seed == 23
    assert config.golf_relief == "high"
    assert config.log_dir == Path("custom/logs")


def test_experiment_config_accepts_only_stage1_models():
    assert robot_model_names() == ("df_front", "df_mid", "df_back", "active_steering_4wd")
    assert terrain_model_names() == ("flat", "slope", "golf_heightfield")
    for robot_model in robot_model_names():
        assert ExperimentConfig(robot_model=robot_model).robot_model == robot_model
    for terrain_model in terrain_model_names():
        assert ExperimentConfig(terrain_model=terrain_model).terrain_model == terrain_model

    with pytest.raises(ValueError, match="robot_model"):
        ExperimentConfig(robot_model="tracked_proxy")
    with pytest.raises(ValueError, match="terrain_model"):
        ExperimentConfig(terrain_model="dam_slope")


def test_stage1_uses_real_physics_and_rejects_legacy_kinematic_mode():
    """阶段一公开入口只允许真实 PyBullet 关节反馈，避免运动学值冒充实测值。"""
    assert ExperimentConfig().drive_model == "physics"
    with pytest.raises(ValueError, match="drive_model"):
        ExperimentConfig(drive_model="kinematic")


def test_gui_config_defaults_to_front_camera_following():
    config = ExperimentConfig(mode="gui")

    assert config.camera_follow_enabled is True
    assert config.camera_follow_view == "front"


@pytest.mark.parametrize("camera_follow_view", ["front", "side", "custom"])
def test_experiment_config_keeps_legacy_camera_follow_views(camera_follow_view: str):
    assert ExperimentConfig(camera_follow_view=camera_follow_view).camera_follow_view == camera_follow_view


def test_experiment_config_rejects_invalid_golf_relief_and_physics():
    with pytest.raises(ValueError, match="golf_relief"):
        ExperimentConfig(golf_relief="spiky")
    with pytest.raises(ValueError, match="drive_motor_force"):
        ExperimentConfig(drive_motor_force=0.0)
    with pytest.raises(ValueError, match="manual_linear_acceleration_limit"):
        ExperimentConfig(manual_linear_acceleration_limit=0.0)
    with pytest.raises(ValueError, match="manual_angular_acceleration_limit"):
        ExperimentConfig(manual_angular_acceleration_limit=0.0)


@pytest.mark.parametrize(
    ("field_name", "value"),
    [
        ("slope_deg", math.nan),
        ("duration_sec", math.inf),
        ("time_step", math.nan),
        ("target_linear_velocity", math.inf),
        ("target_angular_velocity", -math.inf),
        ("dashboard_update_hz", math.nan),
        ("dashboard_smoothing_alpha", math.inf),
        ("dashboard_plot_update_hz", math.nan),
        ("dashboard_plot_window_sec", math.inf),
        ("manual_linear_acceleration_limit", math.nan),
        ("manual_angular_acceleration_limit", math.inf),
        ("camera_distance", math.nan),
        ("camera_yaw", math.inf),
        ("camera_pitch", -math.inf),
        ("lidar_ray_count", math.inf),
        ("lidar_max_distance", math.nan),
        ("lidar_fov_deg", math.inf),
        ("golf_seed", math.nan),
        ("ground_lateral_friction", math.nan),
        ("ground_rolling_friction", math.inf),
        ("ground_spinning_friction", math.nan),
        ("drive_lateral_friction", math.inf),
        ("support_lateral_friction", math.nan),
        ("drive_motor_force", math.inf),
    ],
)
def test_experiment_config_rejects_non_finite_numbers(field_name: str, value: float):
    with pytest.raises(ValueError, match="finite"):
        ExperimentConfig(**{field_name: value})


def test_experiment_config_rejects_non_finite_camera_target():
    with pytest.raises(ValueError, match="camera_target.*finite"):
        ExperimentConfig(camera_target=(0.0, math.nan, 0.0))


def test_old_tracked_and_dam_keys_are_not_runnable_config_keys(tmp_path: Path):
    path = tmp_path / "old.yaml"
    path.write_text("robot_model: tracked_proxy\nterrain_model: dam_slope\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_config(path)


@pytest.mark.parametrize(
    "path",
    [
        "configs/experiment.yaml",
        "configs/flat_demo.yaml",
        "configs/gui_step2_demo.yaml",
        "configs/step3_feedback.yaml",
        "configs/stage1_golf_gui.yaml",
    ],
)
def test_repository_configs_are_stage1_loadable(path: str):
    config = load_config(path)
    assert config.robot_model in robot_model_names()
    assert config.terrain_model in terrain_model_names()
    assert config.drive_model == "physics"
