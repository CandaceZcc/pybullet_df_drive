# 运行时动作协议：定义 Dashboard 与物理主线程共享的 Qt-free 结构操作命令。
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, TypeAlias

from slope_sim.obstacles import ObstacleGenerationRequest
from slope_sim.scene import terrain_model_names

if TYPE_CHECKING:
    from slope_sim.scene_config import SceneDocument


@dataclass(frozen=True)
class TerrainSelection:
    """Dashboard 提交给手动仿真循环的一组完整场地参数。"""

    terrain_model: str
    slope_deg: float = 0.0
    golf_seed: int = 0
    golf_relief: str = "medium"

    def __post_init__(self) -> None:
        """规范化选择值，并阻止非法参数进入 PyBullet 场景重建。"""
        terrain_model = self.terrain_model.lower()
        golf_relief = self.golf_relief.lower()
        if terrain_model not in terrain_model_names():
            raise ValueError(f"terrain_model must be one of: {', '.join(terrain_model_names())}")
        if golf_relief not in {"low", "medium", "high"}:
            raise ValueError("golf_relief must be 'low', 'medium', or 'high'")
        if not math.isfinite(float(self.slope_deg)):
            raise ValueError("slope_deg must be finite")
        if not math.isfinite(float(self.golf_seed)) or int(self.golf_seed) != float(self.golf_seed):
            raise ValueError("golf_seed must be a finite integer")
        object.__setattr__(self, "terrain_model", terrain_model)
        object.__setattr__(self, "slope_deg", float(self.slope_deg))
        object.__setattr__(self, "golf_seed", int(self.golf_seed))
        object.__setattr__(self, "golf_relief", golf_relief)


@dataclass(frozen=True)
class SwitchRobotAction:
    """请求把当前车辆替换为指定车型。"""

    robot_model: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "robot_model", self.robot_model.lower())


@dataclass(frozen=True)
class SwitchTerrainAction:
    """请求重建场地，并把当前障碍物逻辑快照恢复到新场地。"""

    terrain: TerrainSelection


@dataclass(frozen=True)
class ResetRobotAction:
    """请求在当前场地和车型下重新出生车辆。"""


@dataclass(frozen=True)
class AddObstaclesAction:
    """请求随机规划并添加一批障碍物。"""

    request: ObstacleGenerationRequest


@dataclass(frozen=True)
class DeleteObstacleAction:
    """请求删除指定逻辑 ID 的障碍物。"""

    logical_id: int


@dataclass(frozen=True)
class ClearObstaclesAction:
    """请求清空当前所有障碍物。"""


@dataclass(frozen=True)
class LoadSceneAction:
    """请求加载经过校验的完整逻辑场景文档。"""

    document: "SceneDocument"

    def __post_init__(self) -> None:
        """局部导入 SceneDocument，避免运行时模块循环依赖。"""
        from slope_sim.scene_config import SceneDocument

        if not isinstance(self.document, SceneDocument):
            raise ValueError("document must be a SceneDocument")


RuntimeAction: TypeAlias = (
    SwitchRobotAction
    | SwitchTerrainAction
    | ResetRobotAction
    | AddObstaclesAction
    | DeleteObstacleAction
    | ClearObstaclesAction
    | LoadSceneAction
)


def is_safe_stop_action(action: RuntimeAction | None) -> bool:
    """结构操作中，只有会重建车辆或场地的动作需要立即清零驾驶命令。"""
    return isinstance(action, (SwitchRobotAction, SwitchTerrainAction, ResetRobotAction, LoadSceneAction))
