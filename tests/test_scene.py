# 场景回归测试：保护阶段一场地边界、探测和 GUI 相机跟随。
from __future__ import annotations

import math
import random

import pybullet as p
import pytest

import slope_sim.scene as scene_module


def _probe(client_id: int, scene: scene_module.SceneInfo, x: float):
    """在中线上探测地形，统一接缝与分段断言。"""
    return scene_module.probe_terrain(client_id, x, 0.0, bounds=scene.bounds)


def test_golf_heightfield_covers_the_common_flat_terrain_footprint() -> None:
    """高尔夫地形不得让 flat 中合法的障碍布局在切换时越界。"""
    cell_size = scene_module.GOLF_HEIGHTFIELD_CELL_SIZE
    x_span = (scene_module.GOLF_HEIGHTFIELD_COLUMNS - 1) * cell_size
    y_span = (scene_module.GOLF_HEIGHTFIELD_ROWS - 1) * cell_size

    assert x_span >= scene_module.STAGE1_TERRAIN_LENGTH
    assert x_span - cell_size < scene_module.STAGE1_TERRAIN_LENGTH
    assert y_span >= scene_module.STAGE1_TERRAIN_WIDTH


def test_golf_non_square_heightfield_uses_rows_as_y_and_columns_as_x(monkeypatch):
    """以真实碰撞体检查非方形网格的公共 rows=y、columns=x 语义。"""
    rows, columns = 4, 6
    cell_size = scene_module.GOLF_HEIGHTFIELD_CELL_SIZE
    known_heights = tuple(0.15 * column + 0.03 * row for row in range(rows) for column in range(columns))

    def generate_known_heightfield(seed: int, relief: str):
        assert (seed, relief) == (7, "medium")
        return known_heights

    monkeypatch.setattr(scene_module, "GOLF_HEIGHTFIELD_ROWS", rows)
    monkeypatch.setattr(scene_module, "GOLF_HEIGHTFIELD_COLUMNS", columns)
    monkeypatch.setattr(scene_module, "generate_golf_heightfield", generate_known_heightfield)
    monkeypatch.setattr(scene_module, "_raycast_ground_height", lambda client_id, x, y: 0.0)
    monkeypatch.setattr(scene_module, "_raycast_ground_normal", lambda client_id, x, y: (0.0, 0.0, 1.0))
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=1.0 / 240.0,
            terrain_model="golf_heightfield",
            golf_seed=7,
            golf_relief="medium",
        )
        expected_half_x = (columns - 1) * cell_size / 2.0
        expected_half_y = (rows - 1) * cell_size / 2.0
        aabb_min, aabb_max = p.getAABB(scene.body_id, -1, physicsClientId=client_id)

        assert scene.bounds == scene_module.TerrainBounds(-expected_half_x, expected_half_x, -expected_half_y, expected_half_y)
        assert aabb_max[0] - aabb_min[0] == pytest.approx(2.0 * expected_half_x, abs=1e-6)
        assert aabb_max[1] - aabb_min[1] == pytest.approx(2.0 * expected_half_y, abs=1e-6)

        origin = scene_module.probe_terrain(client_id, -0.5 * cell_size, -0.5 * cell_size)
        x_step = scene_module.probe_terrain(client_id, 0.5 * cell_size, -0.5 * cell_size)
        y_step = scene_module.probe_terrain(client_id, -0.5 * cell_size, 0.5 * cell_size)
        assert all(probe.terrain_probe_valid for probe in (origin, x_step, y_step))
        assert x_step.local_ground_height - origin.local_ground_height == pytest.approx(0.15, abs=1e-5)
        assert y_step.local_ground_height - origin.local_ground_height == pytest.approx(0.03, abs=1e-5)
    finally:
        p.disconnect(client_id)


def _grid(values: tuple[float, ...], rows: int, columns: int) -> tuple[tuple[float, ...], ...]:
    """把 PyBullet 行优先高度数组还原成便于检查邻域的二维网格。"""
    assert len(values) == rows * columns
    return tuple(tuple(values[row * columns : (row + 1) * columns]) for row in range(rows))


def _bilinear_grid_height(
    grid: tuple[tuple[float, ...], ...], normalized_x: float, normalized_y: float
) -> float:
    """在一次构造的最终高度网格上独立做双线性插值。"""
    rows = len(grid)
    columns = len(grid[0])
    grid_x = (normalized_x + 1.0) * (columns - 1) / 2.0
    grid_y = (normalized_y + 1.0) * (rows - 1) / 2.0
    column = min(columns - 2, max(0, math.floor(grid_x)))
    row = min(rows - 2, max(0, math.floor(grid_y)))
    column_fraction = grid_x - column
    row_fraction = grid_y - row
    lower = grid[row][column] * (1.0 - column_fraction) + grid[row][column + 1] * column_fraction
    upper = grid[row + 1][column] * (1.0 - column_fraction) + grid[row + 1][column + 1] * column_fraction
    return lower * (1.0 - row_fraction) + upper * row_fraction


def _second_difference(values: list[float]) -> float:
    """用平均绝对二阶差分衡量一条地形剖面的局部曲折程度。"""
    assert len(values) >= 3
    differences = (abs(values[index - 1] - 2.0 * values[index] + values[index + 1]) for index in range(1, len(values) - 1))
    return sum(differences) / (len(values) - 2)


def _expected_golf_corridor_center(seed: int, normalized_x: float) -> float:
    """独立按规格计算廊道中心，避免测试复用生产实现形成假阳性。"""
    phase = random.Random(int(seed) ^ 0x5F3759DF).uniform(-math.pi, math.pi)
    return 0.24 * math.sin(0.85 * math.pi * normalized_x + phase)


def _linear_detrended_residuals(values: list[float]) -> list[float]:
    """移除最佳拟合直线，保留剖面中不能由整体坡度解释的起伏。"""
    mean_x = (len(values) - 1) / 2.0
    mean_y = sum(values) / len(values)
    denominator = sum((index - mean_x) ** 2 for index in range(len(values)))
    slope = sum((index - mean_x) * (value - mean_y) for index, value in enumerate(values)) / denominator
    return [value - (mean_y + slope * (index - mean_x)) for index, value in enumerate(values)]


def _root_mean_square(values: list[float]) -> float:
    """计算剖面残差的均方根幅值。"""
    return math.sqrt(sum(value * value for value in values) / len(values))


def _moving_average(values: list[float], window: int = 27) -> list[float]:
    """以约半个 x 向细节波长的窗口提取低频丘洼。"""
    radius = window // 2
    return [sum(values[index - radius : index + radius + 1]) / window for index in range(radius, len(values) - radius)]


def _sample_corridor_profiles(
    values: tuple[float, ...], rows: int, columns: int, seed: int
) -> tuple[list[float], list[float], list[float]]:
    """沿廊道和两条固定偏移连续轨迹采样，避免跨中心线切换外侧带。"""
    grid = _grid(values, rows, columns)
    corridor: list[float] = []
    plus_outer: list[float] = []
    minus_outer: list[float] = []
    for column in range(129):
        normalized_x = -1.0 + 2.0 * column / 128
        corridor_y = _expected_golf_corridor_center(seed, normalized_x)
        corridor.append(_bilinear_grid_height(grid, normalized_x, corridor_y))
        plus_outer.append(_bilinear_grid_height(grid, normalized_x, corridor_y + 0.45))
        minus_outer.append(_bilinear_grid_height(grid, normalized_x, corridor_y - 0.45))
    return corridor, plus_outer, minus_outer


def _mean_outer_curvature(plus_outer: list[float], minus_outer: list[float]) -> float:
    """两侧固定外轨迹取平均，减少单侧随机丘洼造成的偶然偏差。"""
    return (_second_difference(plus_outer) + _second_difference(minus_outer)) / 2.0


def _expected_golf_heightfield(
    seed: int,
    rows: int,
    columns: int,
    *,
    feature_floor: float,
    detail_floor: float,
) -> tuple[float, ...]:
    """独立计算含指定廊道衰减合同的完整同 seed 高度网格。"""
    rng = random.Random(int(seed))
    phase_x = rng.uniform(-math.pi, math.pi)
    phase_y = rng.uniform(-math.pi, math.pi)
    corridor_phase = random.Random(int(seed) ^ 0x5F3759DF).uniform(-math.pi, math.pi)
    hills = [
        (
            rng.uniform(-0.78, 0.78),
            rng.uniform(-0.78, 0.78),
            rng.uniform(0.16, 0.40),
            rng.uniform(0.14, 0.34),
            rng.uniform(0.28, 0.72),
        )
        for _ in range(8)
    ]
    basins = [
        (
            rng.uniform(-0.72, 0.72),
            rng.uniform(-0.72, 0.72),
            rng.uniform(0.18, 0.38),
            rng.uniform(0.16, 0.34),
            rng.uniform(0.18, 0.42),
        )
        for _ in range(4)
    ]
    heights: list[float] = []
    for row in range(rows):
        y = -1.0 + 2.0 * row / (rows - 1)
        for column in range(columns):
            x = -1.0 + 2.0 * column / (columns - 1)
            broad = 0.30 * math.sin(0.85 * math.pi * x + phase_x)
            broad += 0.24 * math.cos(0.75 * math.pi * y + phase_y)
            broad += 0.12 * x * y
            features = sum(
                height_scale
                * math.exp(-0.5 * (((x - center_x) / sigma_x) ** 2 + ((y - center_y) / sigma_y) ** 2))
                for center_x, center_y, sigma_x, sigma_y, height_scale in hills
            )
            features -= sum(
                depth_scale
                * math.exp(-0.5 * (((x - center_x) / sigma_x) ** 2 + ((y - center_y) / sigma_y) ** 2))
                for center_x, center_y, sigma_x, sigma_y, depth_scale in basins
            )
            detail = 0.10 * math.sin(2.4 * math.pi * x + 0.5 * phase_y)
            detail += 0.08 * math.cos(2.1 * math.pi * y - 0.5 * phase_x)
            corridor_y = 0.24 * math.sin(0.85 * math.pi * x + corridor_phase)
            corridor_weight = 1.0 - math.exp(-((abs(y - corridor_y) / 0.20) ** 2))
            feature_weight = feature_floor + (1.0 - feature_floor) * corridor_weight
            detail_weight = detail_floor + (1.0 - detail_floor) * corridor_weight
            heights.append(0.20 * (broad + features * feature_weight + detail * detail_weight))
    minimum = min(heights)
    return tuple(height - minimum for height in heights)


def _generate_unattenuated_golf_heightfield(seed: int, rows: int, columns: int) -> tuple[float, ...]:
    """独立构造同 seed 完整地形反事实：保留全部丘洼和 detail，不作廊道衰减。"""
    return _expected_golf_heightfield(seed, rows, columns, feature_floor=1.0, detail_floor=1.0)


def test_segmented_slope_has_three_surfaces_and_expected_geometry():
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=8.0,
            time_step=1.0 / 240.0,
            ground_lateral_friction=1.3,
            ground_rolling_friction=0.03,
            ground_spinning_friction=0.04,
            terrain_model="slope",
        )
        assert (
            scene_module.SLOPE_UPPER_LENGTH,
            scene_module.SLOPE_RAMP_LENGTH,
            scene_module.SLOPE_LOWER_LENGTH,
            scene_module.SLOPE_SEAM_OVERLAP,
            scene_module.TERRAIN_THICKNESS,
        ) == (4.0, 8.0, 6.0, 0.04, 0.08)
        angle = math.radians(8.0)
        drop = scene_module.SLOPE_RAMP_LENGTH * math.sin(angle)
        ramp_half_x = scene_module.SLOPE_RAMP_LENGTH * math.cos(angle) / 2.0
        upper_x = -ramp_half_x - scene_module.SLOPE_UPPER_LENGTH / 2.0 + scene_module.SLOPE_SEAM_OVERLAP / 2.0
        lower_x = ramp_half_x + scene_module.SLOPE_LOWER_LENGTH / 2.0 - scene_module.SLOPE_SEAM_OVERLAP / 2.0

        assert scene.terrain_type == "slope"
        assert len(scene.body_ids) == 3
        upper_id, ramp_id, lower_id = scene.body_ids
        assert scene.body_id == ramp_id
        assert len({upper_id, ramp_id, lower_id}) == 3
        assert p.getNumBodies(physicsClientId=client_id) == 3
        for body_id in scene.body_ids:
            dynamics = p.getDynamicsInfo(body_id, -1, physicsClientId=client_id)
            assert dynamics[1] == pytest.approx(1.3)
            assert dynamics[6] == pytest.approx(0.03)
            assert dynamics[7] == pytest.approx(0.04)
        assert scene.spawn_position == pytest.approx((upper_x, 0.0, drop), abs=1e-6)
        assert p.getEulerFromQuaternion(scene.spawn_orientation) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
        assert scene.bounds == scene_module.TerrainBounds(
            min_x=-ramp_half_x - scene_module.SLOPE_UPPER_LENGTH,
            max_x=ramp_half_x + scene_module.SLOPE_LOWER_LENGTH,
            min_y=-scene_module.STAGE1_TERRAIN_WIDTH / 2.0,
            max_y=scene_module.STAGE1_TERRAIN_WIDTH / 2.0,
        )

        upper = _probe(client_id, scene, upper_x)
        ramp = _probe(client_id, scene, 0.0)
        lower = _probe(client_id, scene, lower_x)
        assert upper.local_ground_height == pytest.approx(drop, abs=1e-6)
        assert lower.local_ground_height == pytest.approx(0.0, abs=1e-6)
        assert upper.local_terrain_normal_x == pytest.approx(0.0, abs=1e-6)
        assert upper.local_terrain_normal_z == pytest.approx(1.0, abs=1e-6)
        assert ramp.local_terrain_normal_x == pytest.approx(math.sin(angle), abs=0.02)
        assert ramp.local_terrain_normal_z == pytest.approx(math.cos(angle), abs=0.02)
        assert lower.local_terrain_normal_x == pytest.approx(0.0, abs=1e-6)
        assert lower.local_terrain_normal_z == pytest.approx(1.0, abs=1e-6)
    finally:
        p.disconnect(client_id)


def test_segmented_slope_flat_segments_include_full_seam_overlap():
    """用物理 AABB 独立检查平台外端、物理长度和 0.04 m 接缝覆盖。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=8.0, time_step=1.0 / 240.0, terrain_model="slope")
        upper_id, _ramp_id, lower_id = scene.body_ids
        ramp_half_x = scene_module.SLOPE_RAMP_LENGTH * math.cos(math.radians(8.0)) / 2.0
        ramp_start_x = -ramp_half_x
        ramp_end_x = ramp_half_x
        upper_aabb = p.getAABB(upper_id, -1, physicsClientId=client_id)
        lower_aabb = p.getAABB(lower_id, -1, physicsClientId=client_id)

        assert upper_aabb[1][0] - upper_aabb[0][0] == pytest.approx(
            scene_module.SLOPE_UPPER_LENGTH + scene_module.SLOPE_SEAM_OVERLAP,
            abs=1e-6,
        )
        assert lower_aabb[1][0] - lower_aabb[0][0] == pytest.approx(
            scene_module.SLOPE_LOWER_LENGTH + scene_module.SLOPE_SEAM_OVERLAP,
            abs=1e-6,
        )
        assert upper_aabb[0][0] == pytest.approx(ramp_start_x - scene_module.SLOPE_UPPER_LENGTH, abs=1e-6)
        assert lower_aabb[1][0] == pytest.approx(ramp_end_x + scene_module.SLOPE_LOWER_LENGTH, abs=1e-6)
        assert upper_aabb[1][0] - ramp_start_x == pytest.approx(scene_module.SLOPE_SEAM_OVERLAP, abs=1e-6)
        assert ramp_end_x - lower_aabb[0][0] == pytest.approx(scene_module.SLOPE_SEAM_OVERLAP, abs=1e-6)
    finally:
        p.disconnect(client_id)


def test_segmented_slope_seams_have_no_raycast_gap_or_height_step():
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=8.0, time_step=1.0 / 240.0, terrain_model="slope")
        ramp_half_x = scene_module.SLOPE_RAMP_LENGTH * math.cos(math.radians(8.0)) / 2.0
        for seam_x in (-ramp_half_x, ramp_half_x):
            probes = [_probe(client_id, scene, seam_x + offset) for offset in (-0.01, 0.01)]
            assert all(probe.terrain_probe_valid for probe in probes)
            heights = [probe.local_ground_height for probe in probes]
            assert max(heights) - min(heights) < 0.01
    finally:
        p.disconnect(client_id)


def test_segmented_slope_cleans_first_body_when_second_body_creation_fails(monkeypatch):
    client_id = p.connect(p.DIRECT)
    original_create_multi_body = scene_module.p.createMultiBody
    calls = 0

    def fail_second_body(**kwargs):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise RuntimeError("injected second segment failure")
        return original_create_multi_body(**kwargs)

    monkeypatch.setattr(scene_module.p, "createMultiBody", fail_second_body)
    try:
        with pytest.raises(RuntimeError, match="second segment failure"):
            scene_module.create_slope_scene(client_id, slope_deg=8.0, time_step=1.0 / 240.0, terrain_model="slope")
        assert p.getNumBodies(physicsClientId=client_id) == 0
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(("slope_deg", "expected_normal_sign"), [(0.0, 0), (-8.0, -1)])
def test_segmented_slope_preserves_zero_and_negative_angle_semantics(slope_deg: float, expected_normal_sign: int):
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=slope_deg, time_step=1.0 / 240.0, terrain_model="slope")
        ramp = _probe(client_id, scene, 0.0)
        if expected_normal_sign == 0:
            assert ramp.local_terrain_normal_x == pytest.approx(0.0, abs=1e-6)
            assert scene.spawn_position[2] == pytest.approx(0.0, abs=1e-6)
        else:
            assert ramp.local_terrain_normal_x < 0.0
            assert scene.spawn_position[2] < 0.0
    finally:
        p.disconnect(client_id)


def test_static_terrain_factory_removes_body_when_friction_setup_fails(monkeypatch):
    client_id = p.connect(p.DIRECT)

    def fail_change_dynamics(*args, **kwargs):
        raise RuntimeError("injected friction failure")

    monkeypatch.setattr(scene_module.p, "changeDynamics", fail_change_dynamics)
    try:
        with pytest.raises(RuntimeError, match="friction failure"):
            scene_module.create_slope_scene(client_id, slope_deg=8.0, time_step=1.0 / 240.0, terrain_model="slope")
        assert p.getNumBodies(physicsClientId=client_id) == 0
    finally:
        p.disconnect(client_id)


def test_probe_terrain_marks_bounds_or_misses_as_out_of_bounds():
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        outside = scene_module.probe_terrain(
            client_id,
            scene.bounds.max_x + 1.0,
            0.0,
            bounds=scene.bounds,
            terrain_body_ids=scene.body_ids,
        )
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


def test_probe_terrain_skips_seven_non_terrain_bodies_above_ground():
    """同一射线上的七个非地形体应逐个跳过，最终返回真实地面。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        collision_shape_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=(0.30, 0.30, 0.12),
            physicsClientId=client_id,
        )
        non_terrain_ids = tuple(
            p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=collision_shape_id,
                basePosition=(0.0, 0.0, center_z),
                physicsClientId=client_id,
            )
            for center_z in (0.22, 0.56, 0.90, 1.24, 1.58, 1.92, 2.26)
        )
        first_hit = p.rayTest((0.0, 0.0, 3.0), (0.0, 0.0, -3.0), physicsClientId=client_id)[0]

        probe = scene_module.probe_terrain(
            client_id,
            0.0,
            0.0,
            ray_height=3.0,
            terrain_body_ids=scene.body_ids,
        )

        assert first_hit[0] == non_terrain_ids[-1]
        assert probe.terrain_probe_valid is True
        assert probe.local_ground_height == pytest.approx(0.0, abs=1e-6)
        assert (
            probe.local_terrain_normal_x,
            probe.local_terrain_normal_y,
            probe.local_terrain_normal_z,
        ) == pytest.approx((0.0, 0.0, 1.0), abs=1e-6)
    finally:
        p.disconnect(client_id)


def test_probe_terrain_skips_single_sphere_touching_ground():
    """球体内部的 fraction=0 重复命中不能耗尽过滤次数。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        collision_shape_id = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=0.30,
            physicsClientId=client_id,
        )
        sphere_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=(0.0, 0.0, 0.30),
            physicsClientId=client_id,
        )
        first_hit = p.rayTest((0.0, 0.0, 2.0), (0.0, 0.0, -2.0), physicsClientId=client_id)[0]

        probe = scene_module.probe_terrain(
            client_id,
            0.0,
            0.0,
            ray_height=2.0,
            terrain_body_ids=scene.body_ids,
        )

        assert first_hit[0] == sphere_id
        assert probe.terrain_probe_valid is True
        assert probe.out_of_bounds is False
        assert probe.local_ground_height == pytest.approx(0.0, abs=2e-4)
        assert probe.local_terrain_normal_z == pytest.approx(1.0, abs=1e-6)
    finally:
        p.disconnect(client_id)


def test_probe_terrain_filtered_ray_uses_terrain_mask_once(monkeypatch):
    """过滤模式应由专用 mask 一次命中地形，不再从物体内部重试。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        collision_shape_id = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=0.30,
            physicsClientId=client_id,
        )
        p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=(0.0, 0.0, 0.30),
            physicsClientId=client_id,
        )
        ray_calls: list[dict[str, object]] = []
        original_ray_test = scene_module.p.rayTest

        def capture_ray_test(*args, **kwargs):
            ray_calls.append(dict(kwargs))
            return original_ray_test(*args, **kwargs)

        monkeypatch.setattr(scene_module.p, "rayTest", capture_ray_test)

        probe = scene_module.probe_terrain(
            client_id,
            0.0,
            0.0,
            ray_height=2.0,
            terrain_body_ids=scene.body_ids,
        )

        assert probe.terrain_probe_valid is True
        assert len(ray_calls) == 1
        assert ray_calls[0]["collisionFilterMask"] == scene_module.TERRAIN_FILTER_GROUP
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("terrain_model", ["flat", "slope", "golf_heightfield"])
def test_scene_assigns_dedicated_collision_filter_to_every_terrain_link(monkeypatch, terrain_model: str):
    """三类场地创建完成时，所有地形 base/link 都必须进入专用碰撞组。"""
    client_id = p.connect(p.DIRECT)
    calls: list[tuple[int, int, int, int, int]] = []
    original_set_filter = scene_module.p.setCollisionFilterGroupMask

    def capture_filter(body_id, link_id, group, mask, *, physicsClientId):
        calls.append((int(body_id), int(link_id), int(group), int(mask), int(physicsClientId)))
        return original_set_filter(body_id, link_id, group, mask, physicsClientId=physicsClientId)

    monkeypatch.setattr(scene_module.p, "setCollisionFilterGroupMask", capture_filter)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=8.0,
            time_step=1.0 / 240.0,
            terrain_model=terrain_model,
        )
        expected_body_links = {
            (body_id, link_id)
            for body_id in scene.body_ids
            for link_id in range(-1, p.getNumJoints(body_id, physicsClientId=client_id))
        }

        assert {(body_id, link_id) for body_id, link_id, _group, _mask, _client in calls} == expected_body_links
        assert all(
            group == scene_module.TERRAIN_COLLISION_GROUP
            and mask == scene_module.TERRAIN_COLLISION_MASK
            and call_client == client_id
            for _, _, group, mask, call_client in calls
        )
    finally:
        p.disconnect(client_id)


def test_scene_reapplies_terrain_collision_filter_after_reset(monkeypatch):
    """reset 会清除碰撞组，重建场地必须再次对新 body 设置。"""
    client_id = p.connect(p.DIRECT)
    calls: list[tuple[int, int]] = []
    original_set_filter = scene_module.p.setCollisionFilterGroupMask

    def capture_filter(body_id, link_id, group, mask, *, physicsClientId):
        assert (group, mask) == (scene_module.TERRAIN_COLLISION_GROUP, scene_module.TERRAIN_COLLISION_MASK)
        calls.append((int(body_id), int(link_id)))
        return original_set_filter(body_id, link_id, group, mask, physicsClientId=physicsClientId)

    monkeypatch.setattr(scene_module.p, "setCollisionFilterGroupMask", capture_filter)
    try:
        scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        assert calls
        p.resetSimulation(physicsClientId=client_id)
        calls.clear()

        rebuilt = scene_module.create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=1.0 / 240.0,
            terrain_model="flat",
        )

        assert calls == [(rebuilt.body_id, -1)]
    finally:
        p.disconnect(client_id)


def test_terrain_collision_group_preserves_static_filter_ray_mask():
    """自定义静态 mask 仍应能命中项目地形，避免破坏外部 PyBullet 调试射线。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")

        hit = p.rayTest(
            (0.0, 0.0, 2.0),
            (0.0, 0.0, -2.0),
            collisionFilterMask=0x2,
            physicsClientId=client_id,
        )[0]

        assert hit[0] == scene.body_id
        assert hit[3][2] == pytest.approx(0.0, abs=1e-6)
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(
    ("terrain_body_ids", "message"),
    [
        ((), "terrain_body_ids.*non-empty"),
        ((True,), "terrain_body_ids.*integers"),
        ((1.0,), "terrain_body_ids.*integers"),
        ((-1,), "terrain_body_ids.*non-negative"),
    ],
)
def test_probe_terrain_rejects_invalid_terrain_body_ids(terrain_body_ids: tuple[object, ...], message: str):
    client_id = p.connect(p.DIRECT)
    try:
        scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        with pytest.raises(ValueError, match=message):
            scene_module.probe_terrain(client_id, 0.0, 0.0, terrain_body_ids=terrain_body_ids)
    finally:
        p.disconnect(client_id)


def test_probe_terrain_rejects_body_id_removed_from_current_client():
    client_id = p.connect(p.DIRECT)
    try:
        scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        stale_body_id = p.createMultiBody(baseMass=0.0, physicsClientId=client_id)
        p.removeBody(stale_body_id, physicsClientId=client_id)

        with pytest.raises(ValueError, match="terrain_body_ids.*current physics client"):
            scene_module.probe_terrain(client_id, 0.0, 0.0, terrain_body_ids=(stale_body_id,))
    finally:
        p.disconnect(client_id)


def test_probe_terrain_rejects_unknown_body_id_in_current_client():
    client_id = p.connect(p.DIRECT)
    try:
        scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")

        with pytest.raises(ValueError, match="terrain_body_ids.*current physics client"):
            scene_module.probe_terrain(client_id, 0.0, 0.0, terrain_body_ids=(999_999,))
    finally:
        p.disconnect(client_id)


def test_probe_terrain_rejects_current_non_terrain_body_id():
    client_id = p.connect(p.DIRECT)
    try:
        scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        collision_shape_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=(0.25, 0.25, 0.30),
            physicsClientId=client_id,
        )
        obstacle_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=(0.0, 0.0, 0.30),
            physicsClientId=client_id,
        )

        with pytest.raises(ValueError, match="terrain_body_ids.*terrain body"):
            scene_module.probe_terrain(client_id, 0.0, 0.0, terrain_body_ids=(obstacle_id,))
    finally:
        p.disconnect(client_id)


def test_probe_terrain_rejects_non_terrain_body_mixed_with_scene_body_ids():
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        collision_shape_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=(0.25, 0.25, 0.30),
            physicsClientId=client_id,
        )
        obstacle_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=(0.0, 0.0, 0.30),
            physicsClientId=client_id,
        )

        with pytest.raises(ValueError, match="terrain_body_ids.*terrain body"):
            scene_module.probe_terrain(
                client_id,
                0.0,
                0.0,
                terrain_body_ids=(*scene.body_ids, obstacle_id),
            )
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(("terrain_model", "extra_body_count"), [("flat", 0), ("flat", 100), ("slope", 0), ("slope", 100)])
def test_probe_terrain_validates_only_requested_body_ids(monkeypatch, terrain_model: str, extra_body_count: int):
    """body ID 校验查询数只随地形集合 K 增长，不得枚举场景中的移动体。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=8.0,
            time_step=1.0 / 240.0,
            terrain_model=terrain_model,
        )
        for _ in range(extra_body_count):
            p.createMultiBody(baseMass=0.0, physicsClientId=client_id)

        queried_body_ids: list[int] = []
        enumeration_calls = 0
        original_get_body_info = scene_module.p.getBodyInfo
        original_get_num_bodies = scene_module.p.getNumBodies
        original_get_body_unique_id = scene_module.p.getBodyUniqueId

        def capture_body_info(body_id, **kwargs):
            queried_body_ids.append(int(body_id))
            return original_get_body_info(body_id, **kwargs)

        def capture_num_bodies(**kwargs):
            nonlocal enumeration_calls
            enumeration_calls += 1
            return original_get_num_bodies(**kwargs)

        def capture_body_unique_id(index, **kwargs):
            nonlocal enumeration_calls
            enumeration_calls += 1
            return original_get_body_unique_id(index, **kwargs)

        monkeypatch.setattr(scene_module.p, "getBodyInfo", capture_body_info)
        monkeypatch.setattr(scene_module.p, "getNumBodies", capture_num_bodies)
        monkeypatch.setattr(scene_module.p, "getBodyUniqueId", capture_body_unique_id)

        probe = scene_module.probe_terrain(
            client_id,
            0.0,
            0.0,
            bounds=scene.bounds,
            terrain_body_ids=scene.body_ids,
        )

        assert probe.terrain_probe_valid is True
        assert enumeration_calls == 0
        assert len(queried_body_ids) == len(scene.body_ids)
        assert set(queried_body_ids) == set(scene.body_ids)
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(
    ("arguments", "message"),
    [
        ({"ray_height": 0.0}, "ray_height.*greater than zero"),
        ({"ray_height": -1.0}, "ray_height.*greater than zero"),
        ({"ray_height": math.nan}, "ray_height.*finite"),
        ({"ray_height": math.inf}, "ray_height.*finite"),
        ({"x": math.nan}, "x.*finite"),
        ({"y": -math.inf}, "y.*finite"),
        ({"ray_start_z": math.nan}, "ray_start_z.*finite"),
        ({"ray_start_z": math.inf}, "ray_start_z.*finite"),
        ({"ray_start_z": -8.0}, "ray_start_z.*above.*ray"),
        ({"ray_start_z": -9.0}, "ray_start_z.*above.*ray"),
    ],
)
def test_probe_terrain_rejects_invalid_ray_geometry(arguments: dict[str, float], message: str):
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        call_arguments = {
            "client_id": client_id,
            "x": 0.0,
            "y": 0.0,
            "terrain_body_ids": scene.body_ids,
        }
        call_arguments.update(arguments)

        with pytest.raises(ValueError, match=message):
            scene_module.probe_terrain(**call_arguments)
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("terrain_model", ["flat", "slope", "golf_heightfield"])
@pytest.mark.parametrize("shape_type", [p.GEOM_BOX, p.GEOM_SPHERE, p.GEOM_CYLINDER])
def test_probe_terrain_filters_common_shapes_across_terrain_models(terrain_model: str, shape_type: int):
    """箱、球和圆柱不应改变三类正式地形的采样高度与法向。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=8.0,
            time_step=1.0 / 240.0,
            terrain_model=terrain_model,
        )
        baseline = scene_module.probe_terrain(
            client_id,
            0.0,
            0.0,
            bounds=scene.bounds,
            terrain_body_ids=scene.body_ids,
        )
        shape_arguments = {"shapeType": shape_type, "physicsClientId": client_id}
        if shape_type == p.GEOM_BOX:
            shape_arguments["halfExtents"] = (0.25, 0.25, 0.30)
        elif shape_type == p.GEOM_SPHERE:
            shape_arguments["radius"] = 0.30
        else:
            shape_arguments.update({"radius": 0.25, "height": 0.60})
        collision_shape_id = p.createCollisionShape(**shape_arguments)
        obstacle_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=(0.0, 0.0, baseline.local_ground_height + 0.30),
            physicsClientId=client_id,
        )
        first_hit = p.rayTest((0.0, 0.0, 2.0), (0.0, 0.0, -2.0), physicsClientId=client_id)[0]

        probe = scene_module.probe_terrain(
            client_id,
            0.0,
            0.0,
            ray_height=2.0,
            bounds=scene.bounds,
            terrain_body_ids=scene.body_ids,
        )

        assert first_hit[0] == obstacle_id
        assert probe.terrain_probe_valid is True
        assert probe.local_ground_height == pytest.approx(baseline.local_ground_height, abs=2e-4)
        assert (
            probe.local_terrain_normal_x,
            probe.local_terrain_normal_y,
            probe.local_terrain_normal_z,
        ) == pytest.approx(
            (
                baseline.local_terrain_normal_x,
                baseline.local_terrain_normal_y,
                baseline.local_terrain_normal_z,
            ),
            abs=1e-5,
        )
    finally:
        p.disconnect(client_id)


def test_golf_relief_presets_change_height_range():
    low = scene_module.generate_golf_heightfield(seed=9, relief="low", rows=32, columns=32)
    high = scene_module.generate_golf_heightfield(seed=9, relief="high", rows=32, columns=32)
    assert max(high) - min(high) > 2.5 * (max(low) - min(low))


@pytest.mark.parametrize("seed", [0, 23])
def test_golf_heightfield_matches_independent_feature_and_detail_attenuation_contract(seed: int):
    """输出必须匹配独立 0.813 feature floor 与 0.18 detail floor 的点间高度差。"""
    rows, columns = 7, 9
    actual = scene_module.generate_golf_heightfield(seed, "medium", rows=rows, columns=columns)
    expected = _expected_golf_heightfield(seed, rows, columns, feature_floor=0.813, detail_floor=0.18)
    detail_floor_mutant = _expected_golf_heightfield(seed, rows, columns, feature_floor=0.813, detail_floor=1.0)
    feature_floor_mutant = _expected_golf_heightfield(seed, rows, columns, feature_floor=0.0, detail_floor=0.18)
    actual_deltas = tuple(height - actual[0] for height in actual)

    assert actual_deltas == pytest.approx(tuple(height - expected[0] for height in expected), abs=1e-12)
    assert max(abs(actual_delta - (height - detail_floor_mutant[0])) for actual_delta, height in zip(actual_deltas, detail_floor_mutant)) > 1e-4
    assert max(abs(actual_delta - (height - feature_floor_mutant[0])) for actual_delta, height in zip(actual_deltas, feature_floor_mutant)) > 1e-4


@pytest.mark.parametrize("normalized_x", [-1.0, -0.25, 0.0, 0.75, 1.0])
def test_golf_corridor_center_uses_independent_seeded_phase(normalized_x: float):
    assert scene_module.golf_corridor_center(23, normalized_x) == pytest.approx(
        _expected_golf_corridor_center(23, normalized_x),
        abs=1e-12,
    )


@pytest.mark.parametrize(
    "arguments",
    [
        {"relief": "rough", "rows": 8, "columns": 8},
        {"relief": "medium", "rows": 3, "columns": 8},
        {"relief": "medium", "rows": 8, "columns": 3},
    ],
)
def test_golf_heightfield_rejects_invalid_relief_or_grid_size(arguments: dict[str, object]):
    with pytest.raises(ValueError):
        scene_module.generate_golf_heightfield(seed=1, **arguments)


def test_golf_corridor_preserves_seeded_low_frequency_relief_after_linear_detrending():
    rows = columns = 64
    residual_profiles: list[list[float]] = []
    linear_profile = [0.02 * index for index in range(129)]
    linear_low_frequency = _moving_average(linear_profile)

    # 同一指标必须排除纯仿射剖面，不能把线性横坡误认为丘洼语义。
    assert _root_mean_square(_linear_detrended_residuals(linear_low_frequency)) < 1e-12
    for seed in (0, 23):
        heights = scene_module.generate_golf_heightfield(seed, "medium", rows=rows, columns=columns)
        corridor, _plus_outer, _minus_outer = _sample_corridor_profiles(heights, rows, columns, seed)
        low_frequency = _moving_average(corridor)
        residuals = _linear_detrended_residuals(low_frequency)

        # medium relief 为 0.20 m；5 mm RMS 要求保留至少 2.5% 的低频丘洼语义。
        assert _root_mean_square(residuals) > 0.005
        residual_profiles.append(residuals)

    seed_difference = [first - second for first, second in zip(*residual_profiles)]
    assert _root_mean_square(seed_difference) > 0.005


def test_golf_continuous_corridor_metric_rejects_unattenuated_control():
    """完整同 seed 无衰减反事实必须被连续双侧外轨迹指标拒绝。"""
    rows = columns = 64
    counterfactual_failures = []
    for seed in (0, 23):
        heights = scene_module.generate_golf_heightfield(seed=seed, relief="medium", rows=rows, columns=columns)
        corridor, plus_outer, minus_outer = _sample_corridor_profiles(heights, rows, columns, seed)
        counterfactual = _generate_unattenuated_golf_heightfield(seed, rows, columns)
        control_corridor, control_plus_outer, control_minus_outer = _sample_corridor_profiles(counterfactual, rows, columns, seed)

        assert _second_difference(corridor) < _mean_outer_curvature(plus_outer, minus_outer)
        counterfactual_failures.append(
            _second_difference(control_corridor) >= _mean_outer_curvature(control_plus_outer, control_minus_outer)
        )

    assert any(counterfactual_failures)


@pytest.mark.parametrize("seed", [0, 23])
def test_complex_golf_is_seeded_continuous_and_has_smoother_drive_corridor(seed: int):
    rows = columns = 64
    first = scene_module.generate_golf_heightfield(seed=seed, relief="medium", rows=rows, columns=columns)
    repeated = scene_module.generate_golf_heightfield(seed=seed, relief="medium", rows=rows, columns=columns)
    different = scene_module.generate_golf_heightfield(seed=seed + 1, relief="medium", rows=rows, columns=columns)
    heights = _grid(first, rows, columns)

    assert first == repeated
    assert first != different

    corridor_profile, plus_outer_profile, minus_outer_profile = _sample_corridor_profiles(first, rows, columns, seed)

    assert _second_difference(corridor_profile) < _mean_outer_curvature(plus_outer_profile, minus_outer_profile)
    horizontal_delta = max(
        abs(heights[row][column] - heights[row][column - 1])
        for row in range(rows)
        for column in range(1, columns)
    )
    vertical_delta = max(
        abs(heights[row][column] - heights[row - 1][column])
        for row in range(1, rows)
        for column in range(columns)
    )
    assert horizontal_delta < 0.12
    assert vertical_delta < 0.12


def test_camera_follow_targets_robot_position(monkeypatch):
    calls = {}

    def fake_get_base_position_and_orientation(robot_id, physicsClientId):
        assert robot_id == 42
        assert physicsClientId == 7
        return (1.2, -0.3, 0.5), p.getQuaternionFromEuler((0.0, 0.0, math.radians(30.0)))

    def fake_reset_debug_visualizer_camera(**kwargs):
        calls.update(kwargs)

    monkeypatch.setattr(scene_module.p, "getBasePositionAndOrientation", fake_get_base_position_and_orientation)
    monkeypatch.setattr(scene_module.p, "resetDebugVisualizerCamera", fake_reset_debug_visualizer_camera)

    scene_module.update_follow_camera(7, 42, 6.5, -30.0, 45.0, "front")

    assert calls["cameraTargetPosition"] == pytest.approx((1.2, -0.3, 0.5))
    assert calls["cameraYaw"] == -60.0
    assert calls["cameraDistance"] == 6.5


@pytest.mark.parametrize(
    ("view", "robot_yaw_deg", "expected_yaw"),
    [
        ("front", 0.0, -90.0),
        ("front", 40.0, -50.0),
        ("side", 40.0, 40.0),
        ("custom", 40.0, 45.0),
    ],
)
def test_camera_follow_yaw_is_relative_to_robot_heading(view: str, robot_yaw_deg: float, expected_yaw: float):
    assert scene_module.camera_follow_yaw(view, 45.0, math.radians(robot_yaw_deg)) == pytest.approx(expected_yaw)
