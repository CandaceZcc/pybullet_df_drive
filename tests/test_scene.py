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


def test_dam_slope_scene_probes_all_segments_and_bounds():
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=10.0,
            time_step=1.0 / 240.0,
            ground_lateral_friction=0.8,
            terrain_model="dam_slope",
            dam_toe_length=2.0,
            dam_slope_length=8.0,
            dam_crest_length=3.0,
            dam_exit_length=2.0,
            dam_width=4.0,
            dam_wall_height=0.35,
            terrain_guard_enabled=True,
        )

        assert scene.terrain_type == "dam_slope"
        assert scene.body_id == scene.body_ids[0]
        assert len(scene.body_ids) > 5
        assert scene.spawn_position == pytest.approx((1.0, 0.0, 0.0))
        assert scene.bounds is not None

        toe = scene_module.probe_terrain(client_id, 1.0, 0.0, bounds=scene.bounds)
        uphill = scene_module.probe_terrain(client_id, 4.0, 0.0, bounds=scene.bounds)
        crest = scene_module.probe_terrain(client_id, 11.0, 0.0, bounds=scene.bounds)
        downhill = scene_module.probe_terrain(client_id, 16.0, 0.0, bounds=scene.bounds)
        exit_flat = scene_module.probe_terrain(client_id, 22.0, 0.0, bounds=scene.bounds)

        for probe in (toe, uphill, crest, downhill, exit_flat):
            assert probe.terrain_probe_valid is True
            assert probe.out_of_bounds is False

        crest_height = 8.0 * math.tan(math.radians(10.0))
        assert toe.local_ground_height == pytest.approx(0.0, abs=0.04)
        assert uphill.local_ground_height == pytest.approx(2.0 * math.tan(math.radians(10.0)), abs=0.06)
        assert crest.local_ground_height == pytest.approx(crest_height, abs=0.06)
        assert exit_flat.local_ground_height == pytest.approx(0.0, abs=0.04)
        assert uphill.local_terrain_normal_x < -0.15
        assert downhill.local_terrain_normal_x > 0.15
        assert uphill.local_terrain_normal_z == pytest.approx(math.cos(math.radians(10.0)), abs=0.02)
        assert downhill.local_terrain_normal_z == pytest.approx(math.cos(math.radians(10.0)), abs=0.02)
    finally:
        p.disconnect(client_id)


def test_dam_slope_bounds_keep_robot_center_away_from_edge():
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=10.0,
            time_step=1.0 / 240.0,
            terrain_model="dam_slope",
        )

        assert scene.bounds.max_x == pytest.approx(22.5)
        assert scene_module.probe_terrain(client_id, 22.0, 0.0, bounds=scene.bounds).terrain_probe_valid is True
        assert scene_module.probe_terrain(client_id, 22.75, 0.0, bounds=scene.bounds).out_of_bounds is True
    finally:
        p.disconnect(client_id)


def test_dam_slope_guard_rails_leave_forward_exit_open():
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=10.0,
            time_step=1.0 / 240.0,
            terrain_model="dam_slope",
            terrain_guard_enabled=True,
        )

        side_hit = p.rayTest(
            (scene.bounds.max_x / 2.0, 1.8, 0.2),
            (scene.bounds.max_x / 2.0, 2.3, 0.2),
            physicsClientId=client_id,
        )[0]
        forward_exit_hit = p.rayTest(
            (scene.bounds.max_x - 0.5, 0.0, 0.2),
            (scene.bounds.max_x + 0.5, 0.0, 0.2),
            physicsClientId=client_id,
        )[0]

        assert side_hit[0] in scene.body_ids
        assert forward_exit_hit[0] == -1
    finally:
        p.disconnect(client_id)


def test_probe_terrain_marks_bounds_or_misses_as_out_of_bounds():
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=10.0,
            time_step=1.0 / 240.0,
            terrain_model="dam_slope",
        )

        outside = scene_module.probe_terrain(client_id, scene.bounds.max_x + 1.0, 0.0, bounds=scene.bounds)

        assert outside.terrain_probe_valid is False
        assert outside.out_of_bounds is True
    finally:
        p.disconnect(client_id)

    empty_client = p.connect(p.DIRECT)
    try:
        miss = scene_module.probe_terrain(empty_client, 0.0, 0.0)

        assert miss.terrain_probe_valid is False
        assert miss.out_of_bounds is True
    finally:
        p.disconnect(empty_client)


def test_camera_follow_targets_robot_position(monkeypatch):
    calls = {}

    def fake_get_base_position_and_orientation(robot_id, physicsClientId):
        assert robot_id == 42
        assert physicsClientId == 7
        return (1.2, -0.3, 0.5), (0.0, 0.0, 0.0, 1.0)

    def fake_reset_debug_visualizer_camera(**kwargs):
        calls.update(kwargs)

    monkeypatch.setattr(scene_module.p, "getBasePositionAndOrientation", fake_get_base_position_and_orientation)
    monkeypatch.setattr(scene_module.p, "resetDebugVisualizerCamera", fake_reset_debug_visualizer_camera)

    scene_module.update_follow_camera(
        client_id=7,
        robot_id=42,
        camera_distance=6.5,
        camera_pitch=-30.0,
        camera_yaw=45.0,
        camera_follow_view="front",
    )

    assert calls["cameraTargetPosition"] == pytest.approx((1.2, -0.3, 0.5))
    assert calls["cameraYaw"] == -90.0
    assert calls["cameraDistance"] == 6.5
