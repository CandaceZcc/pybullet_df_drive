# 当前 LiDAR 帧导出：为 GUI 的显式保存操作生成标准 ASCII PCD/PLY 文件。
from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path

from slope_sim.interfaces.models import LidarPointCloud


def export_lidar_point_clouds(
    output_dir: Path,
    clouds: Iterable[LidarPointCloud],
) -> tuple[Path, ...]:
    """将当前帧写为 CloudCompare、MeshLab 可直接导入的 PCD 与 PLY。"""
    normalized = tuple(clouds)
    if not normalized:
        raise ValueError("at least one LiDAR point cloud is required")
    if any(type(cloud) is not LidarPointCloud for cloud in normalized):
        raise ValueError("clouds must contain exact LidarPointCloud values")
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    for cloud in normalized:
        stem = f"{cloud.frame_id}_{cloud.timebase_ns}"
        pcd_path = destination / f"{stem}.pcd"
        ply_path = destination / f"{stem}.ply"
        _write_pcd(pcd_path, cloud)
        _write_ply(ply_path, cloud)
        paths.extend((pcd_path, ply_path))
    return tuple(paths)


def _write_pcd(path: Path, cloud: LidarPointCloud) -> None:
    """写出保留射线时序和线号的 ASCII PCD 0.7。"""
    with path.open("w", encoding="utf-8", newline="\n") as output:
        output.write(
            "# .PCD v0.7 - Point Cloud Data file format\n"
            f"# frame_id {cloud.frame_id}\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity offset_time_ns line\n"
            "SIZE 4 4 4 4 4 2\n"
            "TYPE F F F F U U\n"
            "COUNT 1 1 1 1 1 1\n"
            f"WIDTH {cloud.point_num}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {cloud.point_num}\n"
            "DATA ascii\n"
        )
        for point in cloud.points:
            output.write(
                f"{point.x:.9g} {point.y:.9g} {point.z:.9g} "
                f"{point.reflectivity} {point.offset_time_ns} {point.line}\n"
            )


def _write_ply(path: Path, cloud: LidarPointCloud) -> None:
    """写出含语义 tag 的 ASCII PLY。"""
    with path.open("w", encoding="utf-8", newline="\n") as output:
        output.write(
            "ply\n"
            "format ascii 1.0\n"
            f"comment frame_id {cloud.frame_id}\n"
            f"element vertex {cloud.point_num}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uint intensity\n"
            "property uint offset_time_ns\n"
            "property uchar tag\n"
            "property uchar line\n"
            "end_header\n"
        )
        for point in cloud.points:
            output.write(
                f"{point.x:.9g} {point.y:.9g} {point.z:.9g} "
                f"{point.reflectivity} {point.offset_time_ns} {point.tag} {point.line}\n"
            )
