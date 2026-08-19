# 阶段一场地集成测试：保护平面、连续斜面和可复现高尔夫起伏地形。
from __future__ import annotations

import math

import pybullet as p
import pytest

from slope_sim.scene import create_slope_scene, generate_golf_heightfield, probe_terrain, terrain_model_names


def test_stage1_exposes_only_three_terrain_models():
    assert terrain_model_names() == ("flat", "slope", "golf_heightfield")


def test_golf_heightfield_is_reproducible_and_continuous():
    first = generate_golf_heightfield(seed=23, relief="medium", rows=48, columns=48)
    second = generate_golf_heightfield(seed=23, relief="medium", rows=48, columns=48)
    different = generate_golf_heightfield(seed=24, relief="medium", rows=48, columns=48)

    assert first == second
    assert first != different
    max_neighbor_delta = max(abs(first[index] - first[index - 1]) for index in range(1, len(first)) if index % 48)
    assert max_neighbor_delta < 0.05


@pytest.mark.parametrize("terrain_model", ["flat", "slope", "golf_heightfield"])
def test_stage1_terrain_can_be_created_and_probed(terrain_model: str):
    client_id = p.connect(p.DIRECT)
    try:
        slope_deg = 8.0 if terrain_model == "slope" else 0.0
        scene = create_slope_scene(
            client_id,
            slope_deg=slope_deg,
            time_step=1.0 / 240.0,
            terrain_model=terrain_model,
            golf_seed=17,
            golf_relief="low",
        )
        probe = probe_terrain(client_id, scene.spawn_position[0], scene.spawn_position[1], bounds=scene.bounds)

        assert scene.terrain_type == terrain_model
        assert scene.bounds is not None
        assert probe.terrain_probe_valid is True
        assert probe.out_of_bounds is False
        assert scene.spawn_position[2] == pytest.approx(probe.local_ground_height, abs=0.03)
        if terrain_model == "flat":
            assert probe.local_terrain_normal_z == pytest.approx(1.0, abs=1e-4)
        if terrain_model == "slope":
            assert probe.local_terrain_normal_z == pytest.approx(math.cos(math.radians(8.0)), abs=0.01)
    finally:
        p.disconnect(client_id)


def test_unknown_terrain_is_rejected():
    client_id = p.connect(p.DIRECT)
    try:
        with pytest.raises(ValueError, match="terrain_model"):
            create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="dam_slope")
    finally:
        p.disconnect(client_id)
