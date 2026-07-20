# Dashboard Runtime Switching Implementation Plan

> 状态说明：本计划已完成自动验证并进入阶段一补充交付门禁；后续会话只需读取交付报告，不要继续按本计划派生实现线程。

**Goal:** 在 GUI 手动仿真不退出的情况下，通过 Dashboard 的显式应用按钮安全切换四种车型和三种场地。

**Architecture:** Dashboard 只产生一次性 `DashboardCommand` 请求；`manual_demo.py` 在物理主循环中串行执行车型替换或场地事务重建。场地失败时用保存的上一个有效选择重建世界，Dashboard 只显示结果，不直接调用 PyBullet。

**Tech Stack:** Python 3.10、PyBullet、PySide6、pytest、pandas

**Workspace note:** 当前功能分支依赖大量阶段一未提交基线，不能安全拆到新 worktree，也不能在中间提交中混入这些既有改动。每个任务用聚焦测试和 `git diff --check` 作为检查点，最终保留清晰的未提交差异供用户审阅。

---

### Task 1: Dashboard 一次性切换请求与控件

**Files:**
- Modify: `slope_sim/dashboard.py:1-845`
- Test: `tests/test_dashboard.py`

- [x] **Step 1: 写出仅点击应用按钮才发送一次请求的失败测试**

```python
def test_dashboard_model_switch_requires_apply_and_is_one_shot(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        robot_model="df_back",
        model_switch_enabled=True,
    )
    try:
        dashboard.robot_combo.setCurrentIndex(dashboard.robot_combo.findData("df_mid"))
        assert dashboard.current_command().requested_robot_model is None

        dashboard.request_robot_switch()
        assert dashboard.current_command().requested_robot_model == "df_mid"
        assert dashboard.current_command().requested_robot_model is None
    finally:
        dashboard.close()


def test_dashboard_terrain_switch_requires_apply_and_captures_parameters(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(
        max_linear_speed=0.4,
        max_angular_speed=0.8,
        terrain_model="flat",
        terrain_switch_enabled=True,
    )
    try:
        dashboard.terrain_combo.setCurrentIndex(dashboard.terrain_combo.findData("slope"))
        dashboard.slope_spin.setValue(9.5)
        assert dashboard.current_command().requested_terrain is None

        dashboard.request_terrain_switch()
        request = dashboard.current_command().requested_terrain
        assert request == TerrainSelection("slope", slope_deg=9.5)
        assert dashboard.current_command().requested_terrain is None
    finally:
        dashboard.close()
```

- [x] **Step 2: 运行聚焦测试并确认 RED**

Run:

```bash
conda run -n slope-sim python -m pytest tests/test_dashboard.py::test_dashboard_model_switch_requires_apply_and_is_one_shot tests/test_dashboard.py::test_dashboard_terrain_switch_requires_apply_and_captures_parameters -q
```

Expected: 因 `TerrainSelection`、`terrain_switch_enabled` 或应用请求方法不存在而失败。

- [x] **Step 3: 实现不可变场地选择、一次性请求和 Dashboard 控件**

在 `slope_sim/dashboard.py` 中增加：

```python
from slope_sim.scene import terrain_model_names


@dataclass(frozen=True)
class TerrainSelection:
    """Dashboard 提交给手动仿真循环的一组完整场地参数。"""

    terrain_model: str
    slope_deg: float = 0.0
    golf_seed: int = 0
    golf_relief: str = "medium"

    def __post_init__(self) -> None:
        terrain_model = self.terrain_model.lower()
        golf_relief = self.golf_relief.lower()
        if terrain_model not in terrain_model_names():
            raise ValueError(f"terrain_model must be one of: {', '.join(terrain_model_names())}")
        if golf_relief not in {"low", "medium", "high"}:
            raise ValueError("golf_relief must be 'low', 'medium', or 'high'")
        if not math.isfinite(float(self.slope_deg)):
            raise ValueError("slope_deg must be finite")
        object.__setattr__(self, "terrain_model", terrain_model)
        object.__setattr__(self, "slope_deg", float(self.slope_deg))
        object.__setattr__(self, "golf_seed", int(self.golf_seed))
        object.__setattr__(self, "golf_relief", golf_relief)
```

扩展命令：

```python
@dataclass(frozen=True)
class DashboardCommand:
    linear_velocity: float
    angular_velocity: float
    should_exit: bool = False
    requested_robot_model: str | None = None
    reset_requested: bool = False
    requested_terrain: TerrainSelection | None = None
```

在 `TelemetryDashboard` 中保存 `_requested_robot_model` 和 `_requested_terrain`，创建车型、场地、坡度、种子、起伏等级与状态控件。按钮回调只设置一次性请求：

```python
def request_robot_switch(self) -> None:
    if self.robot_combo is not None:
        self._requested_robot_model = str(self.robot_combo.currentData())


def request_terrain_switch(self) -> None:
    if self.terrain_combo is None:
        return
    self._requested_terrain = TerrainSelection(
        terrain_model=str(self.terrain_combo.currentData()),
        slope_deg=self.slope_spin.value(),
        golf_seed=self.golf_seed_spin.value(),
        golf_relief=str(self.golf_relief_combo.currentData()),
    )
```

`current_command()` 读取后立刻清空这两个字段；`sync_active_selection()` 用实际活动状态同步控件；`show_switch_status()` 更新成功或失败文本。

- [x] **Step 4: 运行 Dashboard 测试并确认 GREEN**

Run:

```bash
conda run -n slope-sim python -m pytest tests/test_dashboard.py -q
git diff --check -- slope_sim/dashboard.py tests/test_dashboard.py
```

Expected: `tests/test_dashboard.py` 全部通过，差异检查无输出。

### Task 2: 可测试的 PyBullet 运行时切换事务

**Files:**
- Modify: `slope_sim/manual_demo.py:1-330`
- Test: `tests/test_manual_demo.py`

- [x] **Step 1: 写出真实 PyBullet 车型、场地和回滚失败测试**

```python
def test_apply_manual_switch_replaces_robot_without_rebuilding_terrain():
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        old_terrain_id = world.scene.body_id
        old_robot_id = world.active_robot.robot.robot_id

        result = apply_manual_switch_request(
            client_id,
            config,
            world,
            DashboardCommand(0.0, 0.0, requested_robot_model="active_steering_4wd"),
        )

        assert result.world.scene.body_id == old_terrain_id
        assert result.world.active_robot.robot_model == "active_steering_4wd"
        assert result.world.active_robot.robot.robot_id != old_robot_id
        assert result.state_changed is True
        assert result.world_reset is False
    finally:
        p.disconnect(client_id)


def test_apply_manual_switch_rebuilds_terrain_and_keeps_robot_model():
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_mid", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_mid")
        result = apply_manual_switch_request(
            client_id,
            config,
            world,
            DashboardCommand(0.0, 0.0, requested_terrain=TerrainSelection("slope", slope_deg=8.0)),
        )

        assert result.world.scene.terrain_type == "slope"
        assert result.world.scene.slope_deg == pytest.approx(8.0)
        assert result.world.active_robot.robot_model == "df_mid"
        assert result.world_reset is True
    finally:
        p.disconnect(client_id)


def test_failed_terrain_switch_rolls_back_to_previous_world(monkeypatch):
    client_id = p.connect(p.DIRECT)
    original = manual_demo_module.create_slope_scene
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")

        def fail_target(*args, **kwargs):
            if kwargs.get("terrain_model") == "golf_heightfield":
                raise RuntimeError("target terrain failed")
            return original(*args, **kwargs)

        monkeypatch.setattr(manual_demo_module, "create_slope_scene", fail_target)
        result = apply_manual_switch_request(
            client_id,
            config,
            world,
            DashboardCommand(0.0, 0.0, requested_terrain=TerrainSelection("golf_heightfield", golf_seed=7)),
        )

        assert result.world.terrain == TerrainSelection("flat")
        assert result.world.scene.terrain_type == "flat"
        assert result.world.active_robot.robot_model == "df_back"
        assert "target terrain failed" in result.error_message
        assert result.world_reset is True
    finally:
        p.disconnect(client_id)
```

- [x] **Step 2: 运行聚焦测试并确认 RED**

Run:

```bash
conda run -n slope-sim python -m pytest tests/test_manual_demo.py -k 'apply_manual_switch or failed_terrain_switch' -q
```

Expected: 因 `load_manual_world` 和 `apply_manual_switch_request` 不存在而失败。

- [x] **Step 3: 实现活动世界、车型替换和场地回滚事务**

在 `slope_sim/manual_demo.py` 中增加：

```python
@dataclass(frozen=True)
class ActiveManualWorld:
    """手动模式当前有效的场景、车辆和场地参数。"""

    scene: SceneInfo
    active_robot: ActiveManualRobot
    terrain: TerrainSelection


@dataclass(frozen=True)
class ManualSwitchResult:
    """一次切换后的有效世界和可供 Dashboard 显示的结果。"""

    world: ActiveManualWorld
    state_changed: bool
    world_reset: bool
    status_message: str
    error_message: str | None = None
```

实现 `create_manual_scene()` 复用 `create_slope_scene()`；`load_manual_world()` 创建场地并加载目标车型；`replace_manual_robot()` 先成功加载新车、再删除旧车；`rebuild_manual_world()` 在目标场地失败时用旧 `TerrainSelection` 和旧车型重新创建世界。

切换入口严格执行场地、车型、复位优先级：

```python
def apply_manual_switch_request(
    client_id: int,
    config: ExperimentConfig,
    world: ActiveManualWorld,
    command: DashboardCommand,
) -> ManualSwitchResult:
    """在物理主线程内执行一帧最多一次的场景变更。"""
    if command.requested_terrain is not None:
        if command.requested_terrain == world.terrain:
            return ManualSwitchResult(world, False, False, "当前场地已生效")
        return rebuild_manual_world(client_id, config, world, command.requested_terrain)
    if command.requested_robot_model is not None:
        if command.requested_robot_model == world.active_robot.robot_model:
            return ManualSwitchResult(world, False, False, "当前车型已生效")
        return replace_manual_world_robot(client_id, config, world, command.requested_robot_model)
    if command.reset_requested:
        return replace_manual_world_robot(
            client_id,
            config,
            world,
            world.active_robot.robot_model,
            force=True,
        )
    return ManualSwitchResult(world, False, False, "就绪")
```

所有关键函数写简短中文 docstring；`load_manual_robot()` 在创建后续步骤失败时删除半成品车体。

- [x] **Step 4: 运行手动事务测试并确认 GREEN**

Run:

```bash
conda run -n slope-sim python -m pytest tests/test_manual_demo.py -q
git diff --check -- slope_sim/manual_demo.py tests/test_manual_demo.py
```

Expected: `tests/test_manual_demo.py` 全部通过，差异检查无输出。

### Task 3: 把切换事务接入 GUI 主循环

**Files:**
- Modify: `slope_sim/manual_demo.py:148-304`
- Modify: `slope_sim/dashboard.py:760-810`
- Test: `tests/test_manual_demo.py`
- Test: `tests/test_dashboard.py`

- [x] **Step 1: 写出命令传播、优先级和历史清理的失败测试**

```python
def test_merge_manual_commands_preserves_terrain_switch_request():
    request = TerrainSelection("slope", slope_deg=6.0)
    merged = merge_manual_commands(
        DashboardCommand(0.0, 0.0, requested_terrain=request),
        ManualCommand(0.4, 0.0),
    )
    assert merged.requested_terrain == request
    assert merged.linear_velocity == 0.0


def test_terrain_switch_takes_priority_over_model_switch_and_reset():
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        result = apply_manual_switch_request(
            client_id,
            config,
            world,
            DashboardCommand(
                0.4,
                0.8,
                requested_robot_model="df_mid",
                reset_requested=True,
                requested_terrain=TerrainSelection("slope", slope_deg=5.0),
            ),
        )
        assert result.world.scene.terrain_type == "slope"
        assert result.world.active_robot.robot_model == "df_back"
    finally:
        p.disconnect(client_id)
```

- [x] **Step 2: 运行聚焦测试并确认 RED**

Run:

```bash
conda run -n slope-sim python -m pytest tests/test_manual_demo.py -k 'preserves_terrain or takes_priority' -q
```

Expected: 地形请求未传播或切换接口尚未满足优先级断言。

- [x] **Step 3: 在 `run_manual_demo()` 中使用活动世界**

初始化时创建 `TerrainSelection` 和 `ActiveManualWorld`，并给 Dashboard 开启两类切换控件：

```python
dashboard = TelemetryDashboard(
    max_linear_speed=config.target_linear_velocity,
    max_angular_speed=0.8,
    update_hz=config.dashboard_update_hz,
    smoothing_alpha=config.dashboard_smoothing_alpha,
    robot_model=world.active_robot.robot_model,
    model_switch_enabled=True,
    terrain_model=world.terrain.terrain_model,
    slope_deg=world.terrain.slope_deg,
    golf_seed=world.terrain.golf_seed,
    golf_relief=world.terrain.golf_relief,
    terrain_switch_enabled=True,
    plot_update_hz=config.dashboard_plot_update_hz,
    plot_window_sec=config.dashboard_plot_window_sec,
    plot_snapshot_dir=config.figure_dir,
)
```

每帧读取命令后：退出优先；任何切换或复位请求都先把驾驶命令置零；调用 `apply_manual_switch_request()`；根据返回值同步活动场景、车辆、Dashboard 状态、曲线、越界锁存和加速度限制器。`world_reset=True` 时重新应用 GUI 可视化配置。

更新 `merge_manual_commands()` 和 `limit_manual_command_step()`，保证 `requested_terrain` 不丢失，且切换帧不混入 PyBullet 窗口键盘速度。

- [x] **Step 4: 运行 Dashboard 与手动模式回归测试**

Run:

```bash
conda run -n slope-sim python -m pytest tests/test_dashboard.py tests/test_manual_demo.py -q
git diff --check -- slope_sim/dashboard.py slope_sim/manual_demo.py tests/test_dashboard.py tests/test_manual_demo.py
```

Expected: 两个测试文件全部通过，差异检查无输出。

### Task 4: 文档、全量验证与独立审查

**Files:**
- Modify: `README.md`
- Modify: `ARCHITECTURE.md`
- Modify: `3d仿真平台需求规格.md`
- Modify: `docs/阶段一交付报告.md`
- Modify: `docs/superpowers/specs/2026-07-16-dashboard-runtime-switching-design.md`

- [x] **Step 1: 更新用户操作与架构说明**

文档必须明确：选择后点击应用；场地切换会执行 `resetSimulation()` 并重载当前车型；失败会回滚；CSV 时间连续但曲线清空；障碍物仍不在本次范围。

- [x] **Step 2: 运行全量测试、矩阵和静态检查**

Run:

```bash
conda run -n slope-sim python -m pytest -q
conda run -n slope-sim python scripts/verify_stage1_matrix.py
conda run -n slope-sim python -m compileall -q main.py slope_sim scripts tests
git diff --check
```

Expected: 全量 pytest 通过，4×3 矩阵 12 项全部 `PASS`，编译和差异检查退出码为 0。

- [x] **Step 3: 启动独立审查线程**

审查线程只读代码和测试，不修改文件，从需求完整性、逻辑正确性、边界情况、代码质量、测试覆盖和实际运行结果六个方面报告问题。主线程对确认的问题补失败测试、修复并重新执行 Step 2。

- [x] **Step 4: 报告 GUI 验证边界**

如果当前会话无 `DISPLAY`，明确说明自动测试和 DIRECT 事务已验证，但按钮视觉布局与真实窗口交互仍需要用户在桌面会话按更新后的人工清单验收。
