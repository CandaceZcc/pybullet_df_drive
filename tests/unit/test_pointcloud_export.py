"""普通 GUI 当前 LiDAR 帧的可导入点云导出合同。"""
from __future__ import annotations

from slope_sim.interfaces.models import LidarPoint, LidarPointCloud
from slope_sim.pointcloud_export import export_lidar_point_clouds


def test_export_lidar_point_clouds_writes_importable_pcd_and_ply(tmp_path) -> None:
    """一次显式导出必须为每个当前雷达帧写出 PCD 和 PLY。"""
    front = LidarPointCloud(
        123_000_000,
        "lidar_front",
        1,
        1,
        (LidarPoint(7, 1.25, -2.5, 0.75, 42, 2, 3),),
    )
    rear = LidarPointCloud(124_000_000, "lidar_rear", 0, 2, ())

    paths = export_lidar_point_clouds(tmp_path, (front, rear))

    assert tuple(path.name for path in paths) == (
        "lidar_front_123000000.pcd",
        "lidar_front_123000000.ply",
        "lidar_rear_124000000.pcd",
        "lidar_rear_124000000.ply",
    )
    assert "FIELDS x y z intensity offset_time_ns line" in paths[0].read_text(
        encoding="utf-8"
    )
    assert "1.25 -2.5 0.75 42 7 3" in paths[0].read_text(encoding="utf-8")
    assert "property uchar tag" in paths[1].read_text(encoding="utf-8")
    assert "1.25 -2.5 0.75 42 7 2 3" in paths[1].read_text(encoding="utf-8")
