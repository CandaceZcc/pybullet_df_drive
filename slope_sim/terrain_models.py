"""与物理引擎无关的场地选择合同，供 CLI 和观察进程安全导入。"""
from __future__ import annotations


STAGE1_TERRAIN_MODELS = ("flat", "slope", "golf_heightfield")


def terrain_model_names() -> tuple[str, ...]:
    """返回阶段一可从配置和命令行选择的三类场地。"""
    return STAGE1_TERRAIN_MODELS
