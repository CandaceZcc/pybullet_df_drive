"""差速底盘斜坡仿真包，对外暴露常用配置和运行入口。"""

from slope_sim.config import ExperimentConfig
from slope_sim.simulation import SimulationResult, run_experiment

__all__ = ["ExperimentConfig", "SimulationResult", "run_experiment"]
