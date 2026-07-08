# 配置测试：确保 YAML 配置、命令行覆盖和非法参数检查都可用。
from pathlib import Path

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
                "ground_lateral_friction: 0.9",
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
    assert config.ground_lateral_friction == 0.9
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
