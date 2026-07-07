# 平地演示配置测试：确保阶段 1 配置确实是 0 度平地。
from pathlib import Path

from slope_sim.config import load_config


def test_flat_demo_config_runs_on_flat_ground():
    config = load_config(Path("configs/flat_demo.yaml"))

    assert config.mode == "direct"
    assert config.slope_deg == 0.0
    assert config.duration_sec > 0.0
    assert config.target_linear_velocity > 0.0
