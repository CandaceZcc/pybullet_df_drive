# 阶段一地形与相机跟随改进 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `slope` 改为可观察平地进入和离开的三段式下坡，增强可复现高尔夫地形，并在 Dashboard 实现真正随车头旋转的相机跟随。

**Architecture:** 保留现有 `flat` / `slope` / `golf_heightfield` 公开入口。`slope` 通过 `SceneInfo.body_ids` 管理三个精确对接静态刚体，`golf_heightfield` 继续使用单个确定性 heightfield；Dashboard 只输出相机持续状态，PyBullet 相机更新仍由手动物理主循环执行。

**Tech Stack:** Python 3.10、PyBullet、PySide6、pytest、YAML、程序化 heightfield。

---

## 工作区与文件边界

当前分支依赖未提交的阶段一基线，不能在新 worktree 中独立复现。执行时保留现有工作区，每个任务用聚焦测试和 `git diff --check` 作检查点；不整文件 `git add`，避免把用户已有未提交改动混入中间提交。

**主要修改文件：**

- `slope_sim/scene.py`：三段式斜面、复杂高尔夫 heightfield、车体相对相机 yaw。
- `slope_sim/config.py`：GUI 跟随默认值和旧视角枚举兼容。
- `slope_sim/dashboard.py`：相机开关/视角控件，以及 `DashboardCommand` 持续状态。
- `slope_sim/manual_demo.py`：命令合并、限速和主循环中的相机状态传播。
- `tests/test_scene.py`：地形几何、可复现性、廊道和相机数学。
- `tests/test_dashboard.py`：相机控件和命令状态。
- `tests/test_manual_demo.py`：相机状态经过键盘合并/场景切换后不丢失。
- `tests/test_config.py`：默认跟随和旧 YAML 兼容。
- `tests/test_simulation_smoke.py`：真实 PyBullet 入坡和四轮驱动回归。
- `README.md`、`ARCHITECTURE.md`、`3d仿真平台需求规格.md`、`docs/阶段一交付报告.md`：用户操作、架构和交付证据。

### Task 1: 三段式下坡几何与失败清理

**Files:**
- Modify: `tests/test_scene.py:12-25`
- Modify: `tests/test_simulation_smoke.py:150-195`
- Modify: `slope_sim/scene.py:13-19,180-277`

- [ ] **Step 1: 用三段法向、高度、出生点和 body 数量失败测试替换旧整体斜面断言**

```python
def test_slope_scene_has_upper_flat_downhill_and_lower_flat():
    client_id = p.connect(p.DIRECT)
    try:
        slope_deg = 8.0
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=slope_deg,
            time_step=1.0 / 240.0,
            terrain_model="slope",
        )
        slope_rad = math.radians(slope_deg)
        horizontal_ramp = scene_module.SLOPE_RAMP_LENGTH * math.cos(slope_rad)
        ramp_start_x = -horizontal_ramp / 2.0
        ramp_end_x = horizontal_ramp / 2.0
        upper_x = ramp_start_x - scene_module.SLOPE_UPPER_LENGTH / 2.0
        lower_x = ramp_end_x + scene_module.SLOPE_LOWER_LENGTH / 2.0

        upper = scene_module.probe_terrain(client_id, upper_x, 0.0, bounds=scene.bounds)
        ramp = scene_module.probe_terrain(client_id, 0.0, 0.0, bounds=scene.bounds)
        lower = scene_module.probe_terrain(client_id, lower_x, 0.0, bounds=scene.bounds)

        assert len(scene.body_ids) == 3
        assert scene.body_id == scene.body_ids[1]
        assert upper.local_terrain_normal_z == pytest.approx(1.0, abs=1e-6)
        assert ramp.local_terrain_normal_x == pytest.approx(math.sin(slope_rad), abs=0.02)
        assert ramp.local_terrain_normal_z == pytest.approx(math.cos(slope_rad), abs=0.02)
        assert lower.local_terrain_normal_z == pytest.approx(1.0, abs=1e-6)
        assert upper.local_ground_height == pytest.approx(
            scene_module.SLOPE_RAMP_LENGTH * math.sin(slope_rad), abs=0.01
        )
        assert lower.local_ground_height == pytest.approx(0.0, abs=0.01)
        assert scene.spawn_position[0] == pytest.approx(upper_x)
        assert p.getEulerFromQuaternion(scene.spawn_orientation)[1] == pytest.approx(0.0, abs=1e-6)
    finally:
        p.disconnect(client_id)
```

- [ ] **Step 2: 写出接缝连续性和构造中途失败清理测试**

```python
def test_slope_segment_seams_have_no_raycast_gap_or_step():
    client_id = p.connect(p.DIRECT)
    try:
        slope_deg = 10.0
        scene = scene_module.create_slope_scene(client_id, slope_deg, 1.0 / 240.0, terrain_model="slope")
        horizontal_ramp = scene_module.SLOPE_RAMP_LENGTH * math.cos(math.radians(slope_deg))
        for seam_x in (-horizontal_ramp / 2.0, horizontal_ramp / 2.0):
            samples = [
                scene_module.probe_terrain(client_id, seam_x + delta, 0.0, bounds=scene.bounds)
                for delta in (-0.01, 0.0, 0.01)
            ]
            assert all(sample.terrain_probe_valid for sample in samples)
            heights = [sample.local_ground_height for sample in samples]
            assert max(heights) - min(heights) < 0.01
    finally:
        p.disconnect(client_id)


def test_segmented_slope_removes_partial_bodies_when_creation_fails(monkeypatch):
    client_id = p.connect(p.DIRECT)
    original = scene_module._create_static_terrain_box
    calls = 0
    try:
        def fail_second_box(*args, **kwargs):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise RuntimeError("injected terrain body failure")
            return original(*args, **kwargs)

        monkeypatch.setattr(scene_module, "_create_static_terrain_box", fail_second_box)
        with pytest.raises(RuntimeError, match="injected terrain body failure"):
            scene_module.create_slope_scene(client_id, 8.0, 1.0 / 240.0, terrain_model="slope")
        assert p.getNumBodies(physicsClientId=client_id) == 0
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(("slope_deg", "normal_x_sign"), [(0.0, 0), (-8.0, -1)])
def test_segmented_slope_preserves_zero_and_negative_angle_semantics(slope_deg, normal_x_sign):
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg, 1.0 / 240.0, terrain_model="slope")
        ramp = scene_module.probe_terrain(client_id, 0.0, 0.0, bounds=scene.bounds)
        if normal_x_sign == 0:
            assert ramp.local_terrain_normal_x == pytest.approx(0.0, abs=1e-6)
            assert scene.spawn_position[2] == pytest.approx(0.0, abs=1e-6)
        else:
            assert ramp.local_terrain_normal_x < 0.0
            assert scene.spawn_position[2] < 0.0
    finally:
        p.disconnect(client_id)


def test_static_terrain_box_removes_body_when_friction_setup_fails(monkeypatch):
    client_id = p.connect(p.DIRECT)
    try:
        def fail_friction(*_args, **_kwargs):
            raise RuntimeError("friction failure")

        monkeypatch.setattr(scene_module, "_apply_terrain_friction", fail_friction)
        with pytest.raises(RuntimeError, match="friction failure"):
            scene_module.create_slope_scene(client_id, 8.0, 1.0 / 240.0, terrain_model="slope")
        assert p.getNumBodies(physicsClientId=client_id) == 0
    finally:
        p.disconnect(client_id)
```

在 `tests/test_simulation_smoke.py` 同步写出四车型完整通过下坡的失败测试，并增加 `get_robot_model`、`create_robot` 和 `scene_module` 导入：

```python
from slope_sim.model_registry import get_robot_model, robot_model_names
from slope_sim.robot import create_robot
import slope_sim.scene as scene_module


@pytest.mark.parametrize("robot_model", robot_model_names())
def test_stage1_robots_cross_downhill_without_tipping(robot_model: str):
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model=robot_model, terrain_model="slope", slope_deg=8.0)
        scene = stage1_verifier.create_slope_scene(
            client_id,
            slope_deg=config.slope_deg,
            time_step=config.time_step,
            terrain_model="slope",
        )
        robot = create_robot(
            client_id,
            robot_model,
            start_x=scene.spawn_position[0],
            start_y=0.0,
            base_height=scene.spawn_position[2] + get_robot_model(robot_model).base_height,
            start_orientation=scene.spawn_orientation,
        )
        robot.apply_drive_friction(1.4, 0.03)

        for settle_step in range(120):
            robot.command_twist(0.0, 0.0, dt=config.time_step)
            p.stepSimulation(physicsClientId=client_id)
            stage1_verifier.validate_robot_pose(
                client_id,
                robot,
                scene,
                require_ground_contact=settle_step >= 30,
            )

        slope_rad = math.radians(config.slope_deg)
        ramp_half_x = scene_module.SLOPE_RAMP_LENGTH * math.cos(slope_rad) / 2.0
        positions: list[float] = []
        pitches: list[float] = []
        for _ in range(7200):
            robot.command_twist(0.7, 0.0, dt=config.time_step)
            p.stepSimulation(physicsClientId=client_id)
            position, orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
            positions.append(float(position[0]))
            pitches.append(float(p.getEulerFromQuaternion(orientation)[1]))
            stage1_verifier.validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
            if positions[-1] > ramp_half_x + 0.8:
                break

        upper_pitches = [pitch for x, pitch in zip(positions, pitches) if x < -ramp_half_x - 0.2]
        ramp_pitches = [pitch for x, pitch in zip(positions, pitches) if -ramp_half_x + 0.5 < x < ramp_half_x - 0.5]
        lower_pitches = [pitch for x, pitch in zip(positions, pitches) if x > ramp_half_x + 0.5]
        assert positions[-1] > ramp_half_x + 0.8
        assert abs(sum(upper_pitches) / len(upper_pitches)) < math.radians(2.0)
        assert max(ramp_pitches) > math.radians(5.0)
        assert abs(sum(lower_pitches) / len(lower_pitches)) < math.radians(2.0)
    finally:
        p.disconnect(client_id)
```

循环在车辆进入低位平地后提前结束，7200 帧只是防卡死上限。如车辆在上限前未进入低位平地，先读取实际最终 `x`、四/两轮速和接触力诊断接触/驱动，不直接放宽 pitch 或距离断言。

- [ ] **Step 3: 运行测试并确认 RED**

Run:

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_scene.py::test_slope_scene_has_upper_flat_downhill_and_lower_flat \
  tests/test_scene.py::test_slope_segment_seams_have_no_raycast_gap_or_step \
  tests/test_scene.py::test_segmented_slope_removes_partial_bodies_when_creation_fails \
  tests/test_scene.py::test_segmented_slope_preserves_zero_and_negative_angle_semantics \
  tests/test_scene.py::test_static_terrain_box_removes_body_when_friction_setup_fails \
  tests/test_simulation_smoke.py::test_stage1_robots_cross_downhill_without_tipping
```

Expected: FAIL，因为 `SLOPE_RAMP_LENGTH` / `_create_static_terrain_box` 未定义，现有斜面也只有一个 body。

- [ ] **Step 4: 增加分段常量和可复用静态地形 box 工厂**

```python
SLOPE_UPPER_LENGTH = 4.0
SLOPE_RAMP_LENGTH = 8.0
SLOPE_LOWER_LENGTH = 6.0
SLOPE_SEAM_OVERLAP = 0.04
TERRAIN_THICKNESS = 0.08


def _create_static_terrain_box(
    client_id: int,
    *,
    length: float,
    width: float,
    thickness: float,
    base_position: tuple[float, float, float],
    orientation: tuple[float, float, float, float],
    rgba_color: tuple[float, float, float, float],
    lateral_friction: float,
    rolling_friction: float,
    spinning_friction: float,
) -> int:
    """创建一个静态地形段，并统一应用视觉与摩擦参数。"""
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=(length / 2.0, width / 2.0, thickness / 2.0),
        physicsClientId=client_id,
    )
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=(length / 2.0, width / 2.0, thickness / 2.0),
        rgbaColor=rgba_color,
        physicsClientId=client_id,
    )
    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=base_position,
        baseOrientation=orientation,
        physicsClientId=client_id,
    )
    try:
        _apply_terrain_friction(
            client_id,
            body_id,
            lateral_friction,
            rolling_friction,
            spinning_friction,
        )
    except Exception:
        # 刚体已创建但摩擦设置失败时，工厂自己负责回收。
        p.removeBody(body_id, physicsClientId=client_id)
        raise
    return body_id
```

- [ ] **Step 5: 实现三段几何，并让 `create_slope_scene()` 按场地类型选择工厂**

```python
def _create_segmented_slope_scene(
    client_id: int,
    slope_deg: float,
    ground_lateral_friction: float,
    ground_rolling_friction: float,
    ground_spinning_friction: float,
) -> SceneInfo:
    """创建高位平地、沿 +X 下坡和低位平地，失败时清理已建地形。"""
    angle = math.radians(slope_deg)
    horizontal_ramp = SLOPE_RAMP_LENGTH * math.cos(angle)
    drop = SLOPE_RAMP_LENGTH * math.sin(angle)
    ramp_start_x = -horizontal_ramp / 2.0
    ramp_end_x = horizontal_ramp / 2.0
    identity = tuple(float(value) for value in p.getQuaternionFromEuler((0.0, 0.0, 0.0)))
    ramp_orientation = tuple(float(value) for value in p.getQuaternionFromEuler((0.0, angle, 0.0)))
    common = {
        "client_id": client_id,
        "width": STAGE1_TERRAIN_WIDTH,
        "thickness": TERRAIN_THICKNESS,
        "lateral_friction": ground_lateral_friction,
        "rolling_friction": ground_rolling_friction,
        "spinning_friction": ground_spinning_friction,
    }
    created: list[int] = []
    try:
        upper_center_x = ramp_start_x - SLOPE_UPPER_LENGTH / 2.0 + SLOPE_SEAM_OVERLAP / 2.0
        upper_id = _create_static_terrain_box(
            **common,
            length=SLOPE_UPPER_LENGTH + SLOPE_SEAM_OVERLAP,
            base_position=(upper_center_x, 0.0, drop - TERRAIN_THICKNESS / 2.0),
            orientation=identity,
            rgba_color=(0.32, 0.56, 0.25, 1.0),
        )
        created.append(upper_id)
        ramp_id = _create_static_terrain_box(
            **common,
            length=SLOPE_RAMP_LENGTH,
            base_position=(
                -math.sin(angle) * TERRAIN_THICKNESS / 2.0,
                0.0,
                drop / 2.0 - math.cos(angle) * TERRAIN_THICKNESS / 2.0,
            ),
            orientation=ramp_orientation,
            rgba_color=(0.48, 0.63, 0.27, 1.0),
        )
        created.append(ramp_id)
        lower_center_x = ramp_end_x + SLOPE_LOWER_LENGTH / 2.0 - SLOPE_SEAM_OVERLAP / 2.0
        lower_id = _create_static_terrain_box(
            **common,
            length=SLOPE_LOWER_LENGTH + SLOPE_SEAM_OVERLAP,
            base_position=(lower_center_x, 0.0, -TERRAIN_THICKNESS / 2.0),
            orientation=identity,
            rgba_color=(0.27, 0.49, 0.22, 1.0),
        )
        created.append(lower_id)
    except Exception:
        for body_id in reversed(created):
            p.removeBody(body_id, physicsClientId=client_id)
        raise

    spawn_x = ramp_start_x - SLOPE_UPPER_LENGTH / 2.0
    return SceneInfo(
        body_id=ramp_id,
        body_ids=(upper_id, ramp_id, lower_id),
        terrain_type="slope",
        slope_deg=float(slope_deg),
        spawn_position=(spawn_x, 0.0, drop),
        spawn_orientation=identity,
        bounds=TerrainBounds(
            ramp_start_x - SLOPE_UPPER_LENGTH,
            ramp_end_x + SLOPE_LOWER_LENGTH,
            -STAGE1_TERRAIN_WIDTH / 2.0,
            STAGE1_TERRAIN_WIDTH / 2.0,
        ),
    )
```

`flat` 继续调用单个 `_create_planar_scene(client_id, 0.0, ground_lateral_friction, ground_rolling_friction, ground_spinning_friction)`；`slope` 改调 `_create_segmented_slope_scene()`；`golf_heightfield` 保持独立分支。

- [ ] **Step 6: 运行斜面聚焦回归并检查差异**

```bash
conda run -n slope-sim python -m pytest -q tests/test_scene.py
git diff --check -- slope_sim/scene.py tests/test_scene.py
```

Expected: `tests/test_scene.py` 全部 PASS，差异检查无输出。

### Task 2: 可复现多尺度高尔夫地形

**Files:**
- Modify: `tests/test_scene.py:42-49`
- Modify: `slope_sim/scene.py:59-99,102-153`

- [ ] **Step 1: 写出不同种子、曲线廊道、局部复杂度和网格连续性测试**

```python
def _grid(values: tuple[float, ...], rows: int, columns: int) -> list[list[float]]:
    return [list(values[row * columns:(row + 1) * columns]) for row in range(rows)]


def _second_difference(values: list[float]) -> float:
    return sum(abs(values[index + 1] - 2.0 * values[index] + values[index - 1]) for index in range(1, len(values) - 1))


def test_complex_golf_is_seeded_continuous_and_has_smoother_drive_corridor():
    rows = columns = 64
    seed = 23
    first = scene_module.generate_golf_heightfield(seed, "high", rows=rows, columns=columns)
    repeated = scene_module.generate_golf_heightfield(seed, "high", rows=rows, columns=columns)
    different = scene_module.generate_golf_heightfield(seed + 1, "high", rows=rows, columns=columns)
    grid = _grid(first, rows, columns)

    corridor: list[float] = []
    outer: list[float] = []
    for column in range(columns):
        x = -1.0 + 2.0 * column / (columns - 1)
        corridor_y = scene_module.golf_corridor_center(seed, x)
        corridor_row = round((corridor_y + 1.0) * (rows - 1) / 2.0)
        corridor.append(grid[corridor_row][column])
        outer.append(grid[4 if corridor_y >= 0.0 else rows - 5][column])

    max_neighbor_delta = max(
        abs(grid[row][column] - grid[next_row][next_column])
        for row in range(rows)
        for column in range(columns)
        for next_row, next_column in ((min(row + 1, rows - 1), column), (row, min(column + 1, columns - 1)))
    )
    assert first == repeated
    assert first != different
    assert _second_difference(corridor) < _second_difference(outer)
    assert max_neighbor_delta < 0.12
```

- [ ] **Step 2: 运行测试并确认 RED**

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_scene.py::test_complex_golf_is_seeded_continuous_and_has_smoother_drive_corridor
```

Expected: FAIL with `AttributeError: golf_corridor_center`。

- [ ] **Step 3: 增加独立可复现的廊道中线函数**

```python
def golf_corridor_center(seed: int, normalized_x: float) -> float:
    """返回归一化坐标下的驾驶廊道中线，与其他随机特征解耦。"""
    rng = random.Random(int(seed) ^ 0x5F3759DF)
    phase = rng.uniform(-math.pi, math.pi)
    return 0.24 * math.sin(0.85 * math.pi * normalized_x + phase)
```

- [ ] **Step 4: 用大尺度波、椭圆丘、浅洼、小尺度波和廊道权重替换高度内核**

```python
rng = random.Random(int(seed))
phase_x = rng.uniform(-math.pi, math.pi)
phase_y = rng.uniform(-math.pi, math.pi)
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
        features = 0.0
        for center_x, center_y, sigma_x, sigma_y, scale in hills:
            radius = ((x - center_x) / sigma_x) ** 2 + ((y - center_y) / sigma_y) ** 2
            features += scale * math.exp(-0.5 * radius)
        for center_x, center_y, sigma_x, sigma_y, scale in basins:
            radius = ((x - center_x) / sigma_x) ** 2 + ((y - center_y) / sigma_y) ** 2
            features -= scale * math.exp(-0.5 * radius)
        detail = 0.10 * math.sin(2.4 * math.pi * x + 0.5 * phase_y)
        detail += 0.08 * math.cos(2.1 * math.pi * y - 0.5 * phase_x)
        corridor_distance = abs(y - golf_corridor_center(seed, x))
        corridor_weight = 1.0 - math.exp(-((corridor_distance / 0.20) ** 2))
        rolling = broad + features + detail * (0.18 + 0.82 * corridor_weight)
        heights.append(amplitude * rolling)
minimum = min(heights)
return tuple(height - minimum for height in heights)
```

如果高差门槛失败，只调整 `detail` 振幅或廊道权重，不放宽连续性断言；一次只改一个参数并重跑该测试。

- [ ] **Step 5: 运行高尔夫和场景回归**

```bash
conda run -n slope-sim python -m pytest -q tests/test_scene.py tests/test_stage1_terrains.py
git diff --check -- slope_sim/scene.py tests/test_scene.py
```

Expected: 两个测试文件全部 PASS，相同种子继续完全可复现。

### Task 3: 车体相对相机数学与默认配置

**Files:**
- Modify: `tests/test_scene.py:50-74`
- Modify: `tests/test_config.py`
- Modify: `slope_sim/scene.py:353-379`
- Modify: `slope_sim/config.py:38-46`
- Modify: `configs/experiment.yaml`
- Modify: `configs/flat_demo.yaml`
- Modify: `configs/gui_step2_demo.yaml`
- Modify: `configs/step3_feedback.yaml`

- [ ] **Step 1: 把相机测试改为验证车辆 yaw 和旧枚举兼容**

```python
@pytest.mark.parametrize(
    ("view", "robot_yaw_deg", "configured_yaw", "expected"),
    [
        ("front", 0.0, 45.0, -90.0),
        ("front", 40.0, 45.0, -50.0),
        ("side", 40.0, 45.0, 40.0),
        ("custom", 40.0, 45.0, 45.0),
    ],
)
def test_camera_follow_yaw_is_robot_relative_except_custom(view, robot_yaw_deg, configured_yaw, expected):
    assert scene_module.camera_follow_yaw(
        view,
        configured_yaw,
        math.radians(robot_yaw_deg),
    ) == pytest.approx(expected)


def test_camera_follow_reads_robot_orientation(monkeypatch):
    calls = {}
    quaternion = p.getQuaternionFromEuler((0.0, 0.0, math.radians(30.0)))
    monkeypatch.setattr(
        scene_module.p,
        "getBasePositionAndOrientation",
        lambda robot_id, physicsClientId: ((1.2, -0.3, 0.5), quaternion),
    )
    monkeypatch.setattr(scene_module.p, "resetDebugVisualizerCamera", lambda **kwargs: calls.update(kwargs))
    scene_module.update_follow_camera(7, 42, 6.5, -30.0, 45.0, "front")
    assert calls["cameraTargetPosition"] == pytest.approx((1.2, -0.3, 0.5))
    assert calls["cameraYaw"] == pytest.approx(-60.0)
```

在 `tests/test_config.py` 增加：

```python
def test_manual_gui_camera_follow_defaults_to_enabled_front_view():
    config = ExperimentConfig(mode="gui")
    assert config.camera_follow_enabled is True
    assert config.camera_follow_view == "front"


@pytest.mark.parametrize("legacy_view", ["front", "side", "custom"])
def test_existing_camera_follow_views_remain_valid(legacy_view: str):
    assert ExperimentConfig(camera_follow_view=legacy_view).camera_follow_view == legacy_view
```

- [ ] **Step 2: 运行测试并确认 RED**

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_scene.py::test_camera_follow_yaw_is_robot_relative_except_custom \
  tests/test_scene.py::test_camera_follow_reads_robot_orientation \
  tests/test_config.py::test_manual_gui_camera_follow_defaults_to_enabled_front_view
```

Expected: FAIL，原因分别是 `camera_follow_yaw()` 缺少车辆 yaw 参数，且默认开关仍为 `False`。

- [ ] **Step 3: 让车后和侧面视角随车头旋转**

```python
def camera_follow_yaw(camera_follow_view: str, camera_yaw: float, robot_yaw: float) -> float:
    """把车后/侧面视角转换为世界 yaw；custom 保留固定角度。"""
    view = camera_follow_view.lower()
    heading_deg = math.degrees(robot_yaw)
    if view == "front":
        return heading_deg - 90.0
    if view == "side":
        return heading_deg
    return camera_yaw


def update_follow_camera(
    client_id: int,
    robot_id: int,
    camera_distance: float,
    camera_pitch: float,
    camera_yaw: float,
    camera_follow_view: str,
) -> None:
    """把 PyBullet debug camera 的目标与方向同步到活动车体。"""
    position, orientation = p.getBasePositionAndOrientation(robot_id, physicsClientId=client_id)
    robot_yaw = float(p.getEulerFromQuaternion(orientation)[2])
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_follow_yaw(camera_follow_view, camera_yaw, robot_yaw),
        cameraPitch=camera_pitch,
        cameraTargetPosition=tuple(float(value) for value in position),
        physicsClientId=client_id,
    )
```

- [ ] **Step 4: 打开 GUI 默认跟随并在可运行 YAML 中显式写出相机参数**

`ExperimentConfig` 改为：

```python
camera_follow_enabled: bool = True
camera_follow_view: str = "front"
```

各 GUI/通用 YAML 统一补充：

```yaml
camera_distance: 6.0
camera_yaw: 45.0
camera_pitch: -35.0
camera_target: [0.8, 0.0, 0.0]
camera_follow_enabled: true
camera_follow_view: front
```

不删除 `front` / `side` / `custom` 验证分支，保证已有配置可直接加载。

- [ ] **Step 5: 运行相机与配置聚焦回归**

```bash
conda run -n slope-sim python -m pytest -q tests/test_scene.py tests/test_config.py tests/test_flat_demo_config.py
git diff --check -- slope_sim/scene.py slope_sim/config.py tests/test_scene.py tests/test_config.py configs
```

Expected: 聚焦回归全部 PASS。

### Task 4: Dashboard 相机控件与手动主循环接线

**Files:**
- Modify: `tests/test_dashboard.py:151-315`
- Modify: `tests/test_manual_demo.py:202-239`
- Modify: `slope_sim/dashboard.py:431-462,566-697,937-973`
- Modify: `slope_sim/manual_demo.py:267-343,351-509`

- [ ] **Step 1: 写出 Dashboard 开关、三视角和持续命令的失败测试**

```python
def test_dashboard_exposes_camera_follow_state(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        camera_follow_enabled=True,
        camera_follow_view="front",
    )
    try:
        assert dashboard.camera_follow_check.isChecked() is True
        assert [dashboard.camera_follow_combo.itemData(index) for index in range(3)] == ["front", "side", "custom"]
        command = dashboard.current_command()
        assert command.camera_follow_enabled is True
        assert command.camera_follow_view == "front"

        dashboard.camera_follow_check.setChecked(False)
        dashboard.camera_follow_combo.setCurrentIndex(dashboard.camera_follow_combo.findData("custom"))
        command = dashboard.current_command()
        assert command.camera_follow_enabled is False
        assert command.camera_follow_view == "custom"
    finally:
        dashboard.close()
```

- [ ] **Step 2: 写出键盘合并和场景命令不丢失相机状态的失败测试**

```python
def test_merge_manual_commands_preserves_camera_state():
    dashboard = DashboardCommand(
        0.0,
        0.0,
        camera_follow_enabled=True,
        camera_follow_view="side",
    )
    merged = merge_manual_commands(dashboard, ManualCommand(0.4, 0.0))
    assert merged.linear_velocity == pytest.approx(0.4)
    assert merged.camera_follow_enabled is True
    assert merged.camera_follow_view == "side"


def test_scene_switch_limit_preserves_camera_state():
    limited = limit_manual_command_step(
        DashboardCommand(0.4, 0.0),
        DashboardCommand(
            0.0,
            0.0,
            requested_terrain=TerrainSelection("slope", slope_deg=8.0),
            camera_follow_enabled=True,
            camera_follow_view="front",
        ),
        dt=0.1,
        linear_acceleration_limit=2.0,
        angular_acceleration_limit=4.0,
    )
    assert limited.camera_follow_enabled is True
    assert limited.camera_follow_view == "front"
```

- [ ] **Step 3: 运行测试并确认 RED**

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_dashboard.py::test_dashboard_exposes_camera_follow_state \
  tests/test_manual_demo.py::test_merge_manual_commands_preserves_camera_state \
  tests/test_manual_demo.py::test_scene_switch_limit_preserves_camera_state
```

Expected: FAIL，因为初始化参数、控件属性和 `DashboardCommand` 字段尚不存在。

- [ ] **Step 4: 扩展 `DashboardCommand` 并增加相机控件**

```python
@dataclass(frozen=True)
class DashboardCommand:
    """Dashboard 输出的驾驶、场景一次性请求和相机持续状态。"""

    linear_velocity: float
    angular_velocity: float
    should_exit: bool = False
    requested_robot_model: str | None = None
    reset_requested: bool = False
    requested_terrain: TerrainSelection | None = None
    camera_follow_enabled: bool = False
    camera_follow_view: str = "front"
```

`TelemetryDashboard.__init__()` 增加：

```python
camera_follow_enabled: bool = True,
camera_follow_view: str = "front",
```

在控制条中的“场地”和“控制”之间增加：

```python
camera_group = QtWidgets.QGroupBox("相机")
camera_layout = QtWidgets.QGridLayout(camera_group)
self.camera_follow_check = QtWidgets.QCheckBox("启用跟随")
self.camera_follow_check.setChecked(camera_follow_enabled)
self.camera_follow_combo = QtWidgets.QComboBox()
for label, value in (("车后", "front"), ("侧面", "side"), ("固定", "custom")):
    self.camera_follow_combo.addItem(label, value)
index = self.camera_follow_combo.findData(camera_follow_view)
if index >= 0:
    self.camera_follow_combo.setCurrentIndex(index)
self.camera_follow_combo.setEnabled(camera_follow_enabled)
self.camera_follow_check.toggled.connect(self.camera_follow_combo.setEnabled)
camera_layout.addWidget(self.camera_follow_check, 0, 0)
camera_layout.addWidget(self.camera_follow_combo, 1, 0)
control_layout.addWidget(camera_group, stretch=0)
```

`current_command()` 的停车和普通返回分支都填入：

```python
camera_follow_enabled=self.camera_follow_check.isChecked(),
camera_follow_view=str(self.camera_follow_combo.currentData()),
```

- [ ] **Step 5: 在所有命令重建分支传播相机状态**

`merge_manual_commands()` 中的退出、场景动作和键盘接管分支都从 `dashboard_command` 复制：

```python
return DashboardCommand(
    linear_velocity,
    angular_velocity,
    should_exit=should_exit,
    requested_robot_model=dashboard_command.requested_robot_model,
    reset_requested=dashboard_command.reset_requested,
    requested_terrain=dashboard_command.requested_terrain,
    camera_follow_enabled=dashboard_command.camera_follow_enabled,
    camera_follow_view=dashboard_command.camera_follow_view,
)
```

`linear_velocity`、`angular_velocity` 和 `should_exit` 继续使用各原分支的实际值，不改变键盘/场景优先级。`limit_manual_command_step()` 的场景快速清零和正常限速分支都从 `target_command` 复制：

```python
return DashboardCommand(
    limited_linear_velocity,
    limited_angular_velocity,
    should_exit=target_command.should_exit,
    requested_robot_model=target_command.requested_robot_model,
    reset_requested=target_command.reset_requested,
    requested_terrain=target_command.requested_terrain,
    camera_follow_enabled=target_command.camera_follow_enabled,
    camera_follow_view=target_command.camera_follow_view,
)
```

`limited_linear_velocity` / `limited_angular_velocity` 在场景分支取 `0.0`；正常分支分别取 `_step_toward(previous_command.linear_velocity, target_command.linear_velocity, linear_acceleration_limit * dt)` 和 `_step_toward(previous_command.angular_velocity, target_command.angular_velocity, angular_acceleration_limit * dt)`。不修改场景请求的一次性消费逻辑。

- [ ] **Step 6: 让手动循环使用 Dashboard 当前相机状态**

Dashboard 初始化补入：

```python
camera_follow_enabled=config.camera_follow_enabled,
camera_follow_view=config.camera_follow_view,
```

在循环前保存兜底状态：

```python
camera_follow_enabled = config.camera_follow_enabled
camera_follow_view = config.camera_follow_view
```

每帧只在 Dashboard 分支的命令合并后更新：

```python
if dashboard is not None:
    camera_follow_enabled = command.camera_follow_enabled
    camera_follow_view = command.camera_follow_view
```

主循环构造 `target_command` 时也保留本帧状态：

```python
target_command = DashboardCommand(
    0.0 if out_of_bounds_latched or scene_action_requested else command.linear_velocity,
    0.0 if out_of_bounds_latched or scene_action_requested else command.angular_velocity,
    should_exit=command.should_exit,
    camera_follow_enabled=camera_follow_enabled,
    camera_follow_view=camera_follow_view,
)
```

相机调用改为：

```python
if camera_follow_enabled:
    update_follow_camera(
        client_id,
        robot.robot_id,
        config.camera_distance,
        config.camera_pitch,
        config.camera_yaw,
        camera_follow_view,
    )
```

Dashboard 不可用时不重建带默认 `False` 的命令覆盖上述兜底状态。

- [ ] **Step 7: 运行 Dashboard 与手动模式回归**

```bash
conda run -n slope-sim python -m pytest -q tests/test_dashboard.py tests/test_manual_demo.py tests/test_scene.py
git diff --check -- slope_sim/dashboard.py slope_sim/manual_demo.py tests/test_dashboard.py tests/test_manual_demo.py
```

Expected: 三个测试文件全部 PASS，Dashboard 无界面测试使用 `QT_QPA_PLATFORM=offscreen`。

### Task 5: 真实物理过渡、四轮驱动证据和文档

**Files:**
- Verify: `tests/test_simulation_smoke.py`
- Modify: `README.md`
- Modify: `ARCHITECTURE.md`
- Modify: `3d仿真平台需求规格.md`
- Modify: `docs/阶段一交付报告.md`
- Modify: `docs/superpowers/specs/2026-07-17-stage1-terrain-camera-improvements-design.md`

- [ ] **Step 1: 重跑四车型完整下坡 DIRECT 回归**

```bash
conda run -n slope-sim python -m pytest -q tests/test_simulation_smoke.py::test_stage1_robots_cross_downhill_without_tipping
```

Expected: 4 个车型参数组全部 PASS。

- [ ] **Step 2: 重跑并记录主动转向四轮实际速度证据**

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_robot_models.py::test_active_steering_twist_drives_all_four_wheels_and_turns_front_wheels \
  tests/test_robot_models.py::test_active_steering_physics_telemetry_contains_four_wheels_and_two_steering_angles \
  tests/test_simulation_smoke.py::test_active_steering_4wd_forward_turn_has_drive_and_yaw_response
```

Expected: `3 passed`。交付报告记录四个关节名、实际轮速接近目标以及扭矩差异属于接触载荷的解释。

- [ ] **Step 3: 更新用户、架构、需求和交付文档**

`README.md` 和 `docs/阶段一交付报告.md` 必须写明：

```text
- slope 正坡度的行驶顺序是高位平地 → 沿 +X 下坡 → 低位平地。
- Dashboard “启用跟随”可即时开关；车后/侧面随车头旋转，固定只跟随位置。
- 高尔夫地形包含多尺度丘陵、浅洼、横坡和驾驶廊道，同一种子/起伏等级仍可复现。
```

`ARCHITECTURE.md` 增加三个 `body_id` 的所有权、坡面姿态方向和相机数据流。`3d仿真平台需求规格.md` 只收紧阶段一验收描述，不将障碍物或传感器提前。设计文档状态改为“已实现并通过自动验证，待 GUI 人工验收”。

- [ ] **Step 4: 运行聚焦回归和文档差异检查**

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_scene.py tests/test_stage1_terrains.py tests/test_config.py \
  tests/test_dashboard.py tests/test_manual_demo.py tests/test_robot_models.py \
  tests/test_simulation_smoke.py
git diff --check
```

Expected: 所有聚焦回归 PASS，差异检查无输出。

### Task 6: 全量验证、视觉检查与独立审查

**Files:**
- Verify only: all changed source, tests, configs and documents

- [ ] **Step 1: 运行全量 pytest、DIRECT 矩阵、编译和差异检查**

```bash
set -e
conda run -n slope-sim python -m pytest -q
conda run -n slope-sim python scripts/verify_stage1_matrix.py
conda run -n slope-sim python -m compileall -q slope_sim tests scripts
git diff --check
```

Expected: pytest 0 failures，4 车型 × 3 场地共 12 个 `PASS`，编译和差异检查退出码为 0。

- [ ] **Step 2: 执行 Qt offscreen 布局检查**

```bash
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest -q tests/test_dashboard.py

QT_QPA_PLATFORM=offscreen /home/cancade/miniforge3/envs/slope-sim/bin/python - <<'PY'
from slope_sim.dashboard import TelemetryDashboard

dashboard = TelemetryDashboard(
    max_linear_speed=0.4,
    max_angular_speed=0.8,
    model_switch_enabled=True,
    terrain_switch_enabled=True,
    camera_follow_enabled=True,
    camera_follow_view="front",
)
dashboard.window.resize(1180, 850)
dashboard.process_events()
assert dashboard.window.grab().save("/tmp/stage1_dashboard_camera.png")
dashboard.close()
PY
```

Expected: Dashboard 测试全部 PASS，用图像查看工具检查 `/tmp/stage1_dashboard_camera.png`，确认新“相机”分组不遮挡车型、场地或方向控制。如当前会话无 `DISPLAY`，不声称已完真实 PyBullet GUI 交互验收。

- [ ] **Step 3: 按项目要求启动独立六维审查线程**

审查线程只读代码和测试，不修改文件；从需求完整性、逻辑正确性、边界情况、代码质量、测试覆盖和实际运行结果六方面按 Critical / Important / Minor 报告。主线程对确认问题先补失败测试，再修复并重跑 Step 1。

- [ ] **Step 4: 更新最终交付数据并报告 GUI 边界**

用最后一次验证的实际 pytest 数量/时间更新 `docs/阶段一交付报告.md`，附上四轮速度证据、三段坡面操作步骤、相机三视角和高尔夫种子复现步骤。若 `DISPLAY` 未设置，明确将下列内容留给用户桌面验收：

```text
1. 三段式斜面的视觉高差、接缝观感和下坡手感。
2. 高尔夫场地的丘陵、浅洼、横坡和廊道层次。
3. 车后/侧面视角的旋转感受，以及关闭跟随后鼠标相机不被抢回。
```
