# 配置测试：确保 YAML 配置、命令行覆盖和非法参数检查都可用。
from pathlib import Path

import pytest

from slope_sim.config import ExperimentConfig, load_config


def test_load_config_reads_yaml_and_applies_overrides(tmp_path: Path):
    config_path = tmp_path / "experiment.yaml"
    config_path.write_text(
        "\n".join(
            [
                "mode: direct",
                "slope_deg: 5",
                "duration_sec: 3.0",
                "time_step: 0.01",
                "wheel_base: 0.42",
                "wheel_radius: 0.08",
                "target_linear_velocity: 0.6",
                "target_angular_velocity: 0.1",
                "robot_model: tracked_proxy",
                "drive_model: physics",
                "dashboard_enabled: false",
                "dashboard_update_hz: 4.0",
                "dashboard_smoothing_alpha: 0.2",
                "camera_distance: 7.5",
                "camera_yaw: 35.0",
                "camera_pitch: -45.0",
                "camera_target: [1.0, 0.5, 0.2]",
                "lidar_enabled: true",
                "lidar_ray_count: 17",
                "lidar_max_distance: 4.0",
                "lidar_fov_deg: 120.0",
                "lidar_debug_draw: true",
                "terrain_model: box_slope",
                "dam_toe_length: 2.5",
                "dam_slope_length: 8.5",
                "dam_crest_length: 3.5",
                "dam_exit_length: 2.5",
                "dam_width: 4.5",
                "dam_wall_height: 0.4",
                "terrain_guard_enabled: false",
                "camera_follow_enabled: true",
                "camera_follow_view: side",
                "gui_model_switch_enabled: true",
                "dashboard_plot_update_hz: 6.0",
                "dashboard_plot_window_sec: 15.0",
                "manual_linear_acceleration_limit: 1.5",
                "manual_angular_acceleration_limit: 3.0",
                "drive_motor_force: 3.5",
                "track_anisotropic_friction: [2.5, 0.2, 0.1]",
                "track_drive_mode: center_only",
                "ground_lateral_friction: 0.9",
                "ground_rolling_friction: 0.03",
                "ground_spinning_friction: 0.04",
                "drive_lateral_friction: 0.7",
                "support_lateral_friction: 0.04",
                "log_dir: custom/logs",
                "figure_dir: custom/figures",
            ]
        ),
        encoding="utf-8",
    )

    config = load_config(config_path, overrides={"slope_deg": 12, "gui": True})

    assert isinstance(config, ExperimentConfig)
    assert config.mode == "gui"
    assert config.slope_deg == 12
    assert config.duration_sec == 3.0
    assert config.time_step == 0.01
    assert config.wheel_base == 0.42
    assert config.wheel_radius == 0.08
    assert config.target_linear_velocity == 0.6
    assert config.target_angular_velocity == 0.1
    assert config.robot_model == "tracked_proxy"
    assert config.drive_model == "physics"
    assert config.dashboard_enabled is False
    assert config.dashboard_update_hz == 4.0
    assert config.dashboard_smoothing_alpha == 0.2
    assert config.camera_distance == 7.5
    assert config.camera_yaw == 35.0
    assert config.camera_pitch == -45.0
    assert config.camera_target == (1.0, 0.5, 0.2)
    assert config.lidar_enabled is True
    assert config.lidar_ray_count == 17
    assert config.lidar_max_distance == 4.0
    assert config.lidar_fov_deg == 120.0
    assert config.lidar_debug_draw is True
    assert config.terrain_model == "box_slope"
    assert config.dam_toe_length == 2.5
    assert config.dam_slope_length == 8.5
    assert config.dam_crest_length == 3.5
    assert config.dam_exit_length == 2.5
    assert config.dam_width == 4.5
    assert config.dam_wall_height == 0.4
    assert config.terrain_guard_enabled is False
    assert config.camera_follow_enabled is True
    assert config.camera_follow_view == "side"
    assert config.gui_model_switch_enabled is True
    assert config.dashboard_plot_update_hz == 6.0
    assert config.dashboard_plot_window_sec == 15.0
    assert config.manual_linear_acceleration_limit == 1.5
    assert config.manual_angular_acceleration_limit == 3.0
    assert config.drive_motor_force == 3.5
    assert config.track_anisotropic_friction == (2.5, 0.2, 0.1)
    assert config.track_drive_mode == "center_only"
    assert config.ground_lateral_friction == 0.9
    assert config.ground_rolling_friction == 0.03
    assert config.ground_spinning_friction == 0.04
    assert config.drive_lateral_friction == 0.7
    assert config.support_lateral_friction == 0.04
    assert config.log_dir == Path("custom/logs")
    assert config.figure_dir == Path("custom/figures")


def test_experiment_config_rejects_invalid_mode():
    try:
        ExperimentConfig(mode="wayland")
    except ValueError as exc:
        assert "mode" in str(exc)
    else:
        raise AssertionError("invalid mode should fail")


def test_experiment_config_rejects_invalid_robot_or_drive_model():
    for kwargs in ({"robot_model": "tank"}, {"drive_model": "magic"}):
        try:
            ExperimentConfig(**kwargs)
        except ValueError as exc:
            assert "model" in str(exc)
        else:
            raise AssertionError(f"invalid config should fail: {kwargs}")


def test_experiment_config_rejects_invalid_terrain_model():
    with pytest.raises(ValueError, match="terrain_model"):
        ExperimentConfig(terrain_model="heightfield")


def test_experiment_config_accepts_dam_slope_and_step4_yaml():
    config = ExperimentConfig(terrain_model="dam_slope", slope_deg=10.0)

    assert config.terrain_model == "dam_slope"
    assert config.dam_toe_length == 2.0
    assert config.dam_slope_length == 8.0
    assert config.dam_crest_length == 3.0
    assert config.dam_exit_length == 2.0
    assert config.dam_width == 4.0
    assert config.dam_wall_height == 0.35
    assert config.terrain_guard_enabled is True

    step4 = load_config("configs/step4_dam_gui.yaml")

    assert step4.terrain_model == "dam_slope"
    assert step4.slope_deg == 10.0
    assert step4.ground_lateral_friction == 0.8
    assert step4.track_anisotropic_friction == (2.0, 0.2, 0.05)
    assert step4.manual_linear_acceleration_limit == 1.5
    assert step4.manual_angular_acceleration_limit == 4.0
    assert step4.camera_follow_enabled is True
    assert step4.gui_model_switch_enabled is True


def test_twr_slope_terrain_requires_five_degrees():
    with pytest.raises(ValueError, match="twr_slope_5deg"):
        ExperimentConfig(terrain_model="twr_slope_5deg", slope_deg=8.0)


def test_experiment_config_rejects_invalid_track_tuning():
    with pytest.raises(ValueError, match="drive_motor_force"):
        ExperimentConfig(drive_motor_force=0.0)
    with pytest.raises(ValueError, match="track_anisotropic_friction"):
        ExperimentConfig(track_anisotropic_friction=[2.0, 0.05])
    with pytest.raises(ValueError, match="track_drive_mode"):
        ExperimentConfig(track_drive_mode="magic")


def test_experiment_config_rejects_invalid_manual_acceleration_limits():
    with pytest.raises(ValueError, match="manual_linear_acceleration_limit"):
        ExperimentConfig(manual_linear_acceleration_limit=0.0)
    with pytest.raises(ValueError, match="manual_angular_acceleration_limit"):
        ExperimentConfig(manual_angular_acceleration_limit=0.0)
