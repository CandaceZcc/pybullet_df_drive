# 场景测试：保护简单 box 坡面和 Two-Wheel-Robot-DeepRL 参考坡面的加载与探测。
import math

import pybullet as p
import pytest

import slope_sim.scene as scene_module


def test_twr_slope_scene_places_five_degree_slope_under_robot_start():
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=5.0,
            time_step=1.0 / 240.0,
            ground_lateral_friction=1.3,
            ground_rolling_friction=0.03,
            ground_spinning_friction=0.04,
            terrain_model="twr_slope_5deg",
        )

        assert hasattr(scene_module, "probe_terrain")
        probe = scene_module.probe_terrain(client_id, 0.0, 0.0)

        assert scene.terrain_type == "twr_slope_5deg"
        assert scene.body_id >= 0
        assert probe.local_ground_height == pytest.approx(0.0, abs=0.05)
        assert probe.local_terrain_normal_x == pytest.approx(-math.sin(math.radians(5.0)), abs=0.02)
        assert probe.local_terrain_normal_z == pytest.approx(math.cos(math.radians(5.0)), abs=0.02)
        assert p.getDynamicsInfo(scene.body_id, -1, physicsClientId=client_id)[1] == pytest.approx(1.3)
    finally:
        p.disconnect(client_id)
