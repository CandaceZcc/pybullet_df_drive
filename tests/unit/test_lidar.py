# LiDAR 单元测试：保护射线生成和距离摘要逻辑。
import pytest

from slope_sim.sensors import LidarSummary, generate_lidar_rays, summarize_lidar_hits


def test_generate_lidar_rays_spans_field_of_view():
    ray_froms, ray_toes = generate_lidar_rays(
        origin=(0.0, 0.0, 0.2),
        yaw=0.0,
        ray_count=3,
        max_distance=2.0,
        fov_deg=90.0,
    )

    assert ray_froms == [(0.0, 0.0, 0.2)] * 3
    assert ray_toes[1] == pytest.approx((2.0, 0.0, 0.2))
    assert ray_toes[0][1] < 0.0
    assert ray_toes[2][1] > 0.0


def test_summarize_lidar_hits_uses_fraction_or_max_distance():
    hits = [
        (-1, -1, 1.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        (2, -1, 0.25, (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        (3, -1, 0.5, (2.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ]

    summary = summarize_lidar_hits(hits, max_distance=4.0)

    assert isinstance(summary, LidarSummary)
    assert summary.min_distance == pytest.approx(1.0)
    assert summary.front_distance == pytest.approx(1.0)
    assert summary.left_distance == pytest.approx(2.0)
    assert summary.right_distance == pytest.approx(4.0)
