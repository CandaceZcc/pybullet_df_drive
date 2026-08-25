"""差速底盘斜坡仿真包，对外暴露常用配置和按需加载的运行入口。"""

from typing import TYPE_CHECKING

from slope_sim.config import ExperimentConfig

if TYPE_CHECKING:
    from slope_sim.simulation import SimulationResult

__all__ = ["ExperimentConfig", "SimulationResult", "run_experiment"]


def __getattr__(name: str) -> object:
    """仅在实际启动物理世界时导入 PyBullet，观察型 Dashboard 不加载它。"""
    if name in {"SimulationResult", "run_experiment"}:
        from slope_sim.simulation import SimulationResult, run_experiment

        return {"SimulationResult": SimulationResult, "run_experiment": run_experiment}[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
