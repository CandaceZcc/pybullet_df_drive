# 传感器模块：提供基于 PyBullet rayTestBatch 的简化 LiDAR/超声波距离检测。
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import pybullet as p


@dataclass(frozen=True)
class LidarSummary:
    """LiDAR 扫描距离摘要，供 Dashboard、日志和避障控制使用。"""

    min_distance: float = math.nan
    front_distance: float = math.nan
    left_distance: float = math.nan
    right_distance: float = math.nan


def generate_lidar_rays(
    origin: tuple[float, float, float],
    yaw: float,
    ray_count: int,
    max_distance: float,
    fov_deg: float,
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]]]:
    """根据车体位置、航向角和视场角生成一组水平 LiDAR 射线。"""
    if ray_count <= 0:
        raise ValueError("ray_count must be positive")
    if max_distance <= 0:
        raise ValueError("max_distance must be positive")
    if fov_deg <= 0:
        raise ValueError("fov_deg must be positive")

    if ray_count == 1:
        angles = [yaw]
    else:
        half_fov = math.radians(fov_deg) / 2.0
        angles = [yaw - half_fov + index * (2.0 * half_fov / (ray_count - 1)) for index in range(ray_count)]

    ray_froms = [origin for _ in angles]
    ray_toes = [
        (
            origin[0] + max_distance * math.cos(angle),
            origin[1] + max_distance * math.sin(angle),
            origin[2],
        )
        for angle in angles
    ]
    return ray_froms, ray_toes


def summarize_lidar_hits(hits: Sequence[tuple], max_distance: float) -> LidarSummary:
    """把 PyBullet rayTestBatch 的命中结果压缩成前/左/右/最近距离。"""
    if not hits:
        return LidarSummary()

    distances = [
        max_distance if hit[0] == -1 else max(0.0, min(max_distance, float(hit[2]) * max_distance))
        for hit in hits
    ]
    center_index = len(distances) // 2
    left_distances = distances[center_index + 1 :] or [distances[center_index]]
    right_distances = distances[:center_index] or [distances[center_index]]
    return LidarSummary(
        min_distance=min(distances),
        front_distance=distances[center_index],
        left_distance=min(left_distances),
        right_distance=min(right_distances),
    )


def read_lidar(
    client_id: int,
    origin: tuple[float, float, float],
    yaw: float,
    ray_count: int,
    max_distance: float,
    fov_deg: float,
    draw_debug: bool = False,
    life_time: float = 0.1,
) -> LidarSummary:
    """执行一次 LiDAR 射线检测；GUI 模式下可画出短生命周期射线。"""
    ray_froms, ray_toes = generate_lidar_rays(origin, yaw, ray_count, max_distance, fov_deg)
    hits = p.rayTestBatch(ray_froms, ray_toes, physicsClientId=client_id)
    if draw_debug:
        for ray_from, ray_to, hit in zip(ray_froms, ray_toes, hits):
            hit_position = hit[3] if hit[0] != -1 else ray_to
            color = [1.0, 0.1, 0.1] if hit[0] != -1 else [0.1, 0.8, 0.2]
            p.addUserDebugLine(ray_from, hit_position, lineColorRGB=color, lifeTime=life_time, physicsClientId=client_id)
    return summarize_lidar_hits(hits, max_distance)
