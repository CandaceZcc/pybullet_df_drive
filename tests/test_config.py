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
    assert config.log_dir == Path("custom/logs")
    assert config.figure_dir == Path("custom/figures")


def test_experiment_config_rejects_invalid_mode():
    try:
        ExperimentConfig(mode="wayland")
    except ValueError as exc:
        assert "mode" in str(exc)
    else:
        raise AssertionError("invalid mode should fail")

