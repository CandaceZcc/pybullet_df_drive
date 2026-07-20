# Dashboard 固定侧边栏实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将阶段一 Dashboard 改为 420 px 固定宽度、屏幕可用高度 95% 的单列侧边栏，同时保留全部图表、遥测与驾驶控制。

**Architecture:** 保留 `TelemetryDashboard` 的业务状态和事件处理，只替换窗口尺寸约束与 Qt 容器层级。顶部 `QTabWidget` 独立于下方控制 `QScrollArea`；五个控制组在滚动内容中纵向排列，Matplotlib 使用适合 420 px 窄画布的固定边距。

**Tech Stack:** Python 3.10、PySide6、Matplotlib、pytest

---

### Task 1: 固定侧边栏尺寸契约

**Files:**
- Modify: `tests/test_dashboard.py`
- Modify: `slope_sim/dashboard.py`

- [ ] **Step 1: 写尺寸函数失败测试**

将 `test_dashboard_window_size_clamps_to_available_screen` 改为：

```python
def test_dashboard_window_size_uses_fixed_sidebar_width_and_screen_height():
    assert dashboard_window_size(available_width=900, available_height=700) == (420, 665)
    assert dashboard_window_size(available_width=5000, available_height=3000) == (420, 2850)
    assert dashboard_window_size(available_width=320, available_height=320) == (420, 304)
```

- [ ] **Step 2: 运行测试确认因旧宽屏算法失败**

Run: `conda run -n slope-sim python -m pytest tests/test_dashboard.py::test_dashboard_window_size_uses_fixed_sidebar_width_and_screen_height -q`

Expected: FAIL，实际宽度仍为 `820`、`1180` 或 `320`。

- [ ] **Step 3: 实现最小尺寸算法**

在 `slope_sim/dashboard.py` 增加固定侧栏常量并替换函数：

```python
DASHBOARD_FIXED_WIDTH = 420
DASHBOARD_AVAILABLE_HEIGHT_RATIO = 0.95


def dashboard_window_size(available_width: int, available_height: int) -> tuple[int, int]:
    """返回固定宽度及当前屏幕可用高度 95% 的 Dashboard 尺寸。"""
    del available_width
    return DASHBOARD_FIXED_WIDTH, max(1, int(available_height * DASHBOARD_AVAILABLE_HEIGHT_RATIO))
```

- [ ] **Step 4: 运行尺寸测试确认通过**

Run: `conda run -n slope-sim python -m pytest tests/test_dashboard.py::test_dashboard_window_size_uses_fixed_sidebar_width_and_screen_height -q`

Expected: PASS。

### Task 2: 顶部标签页与纵向控制滚动区

**Files:**
- Modify: `tests/test_dashboard.py`
- Modify: `slope_sim/dashboard.py`

- [ ] **Step 1: 写 Qt 布局失败测试**

用一个离屏测试替换旧横向控制条测试，创建启用车型和场地切换的 Dashboard，并断言：

```python
assert dashboard.window.minimumWidth() == 420
assert dashboard.window.maximumWidth() == 420
assert dashboard.window.minimumHeight() == dashboard.window.maximumHeight()
assert dashboard.control_scroll.verticalScrollBar().maximum() > 0
assert [group.title() for group in dashboard.control_groups] == ["参数", "车型", "场地", "相机", "控制"]
assert all(group.parentWidget() is dashboard.control_content for group in dashboard.control_groups)
assert not dashboard.control_scroll.isAncestorOf(dashboard.tabs)
assert all(not dashboard.control_scroll.isAncestorOf(canvas) for canvas in dashboard.plot_canvases.values())
```

同时把每个组映射到窗口坐标系，断言相邻组按 `top()` 从上到下排列，不再比较横向 `left()`。

- [ ] **Step 2: 运行测试确认因缺少控制滚动区或窗口未固定失败**

Run: `conda run -n slope-sim python -m pytest tests/test_dashboard.py -k 'fixed_sidebar or vertical_control' -q`

Expected: FAIL，旧实现没有 `control_scroll`、`control_content`、`control_groups`，且窗口宽度会被最小尺寸提示撑大。

- [ ] **Step 3: 实现单列布局**

在 `TelemetryDashboard.__init__` 中：

```python
self.window.setFixedSize(width, height)
self.tabs = QtWidgets.QTabWidget()
self.tabs.setMinimumHeight(DASHBOARD_TOP_TABS_MIN_HEIGHT)
layout.addWidget(self.tabs, stretch=DASHBOARD_TOP_AREA_STRETCH)

self.control_scroll = QtWidgets.QScrollArea()
self.control_scroll.setWidgetResizable(True)
self.control_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
self.control_content = QtWidgets.QWidget()
self.control_layout = QtWidgets.QVBoxLayout(self.control_content)
self.control_groups = [parameter_group, vehicle_group, terrain_group, camera_group, control_group]
for group in self.control_groups:
    self.control_layout.addWidget(group)
self.control_layout.addStretch(1)
self.control_scroll.setWidget(self.control_content)
layout.addWidget(self.control_scroll, stretch=DASHBOARD_CONTROL_AREA_STRETCH)
```

创建各组时不再立即加入横向布局；场地切换关闭时不把不存在的场地组加入列表。删除 `control_bar`、横向 `QHBoxLayout`、最大高度以及 `setMinimumSize(minimumSizeHint())`。

- [ ] **Step 4: 运行布局测试确认通过**

Run: `conda run -n slope-sim python -m pytest tests/test_dashboard.py -k 'fixed_sidebar or vertical_control' -q`

Expected: PASS。

### Task 3: 窄画布完整性与回归验证

**Files:**
- Modify: `tests/test_dashboard.py`
- Modify: `slope_sim/dashboard.py`

- [ ] **Step 1: 写窄画布失败测试**

新增离屏测试，切换到每个曲线页后断言画布和“清空曲线”“保存当前图”按钮都在顶部标签页中、不在参数滚动区中；同时断言 Matplotlib 轴区域为窄画布保留至少 18% 左边距、18% 下边距以及足够顶部空间：

```python
for label, figure in dashboard.plot_figures.items():
    left, bottom, right, top = figure.subplotpars.left, figure.subplotpars.bottom, figure.subplotpars.right, figure.subplotpars.top
    assert left >= 0.23
    assert bottom >= 0.18
    assert right <= 0.97
    assert top <= 0.90
```

- [ ] **Step 2: 运行测试确认旧边距失败**

Run: `conda run -n slope-sim python -m pytest tests/test_dashboard.py -k 'narrow_plot' -q`

Expected: FAIL，旧边距 `left=0.12`、`bottom=0.14`。

- [ ] **Step 3: 调整画布与上下区域比例**

使用适合固定侧栏的常量：

```python
DASHBOARD_TOP_AREA_STRETCH = 45
DASHBOARD_CONTROL_AREA_STRETCH = 55
DASHBOARD_TOP_TABS_MIN_HEIGHT = 320
DASHBOARD_PLOT_FIGURE_SIZE = (4.0, 3.2)
DASHBOARD_PLOT_MARGINS = {"left": 0.24, "right": 0.96, "bottom": 0.20, "top": 0.86}
DASHBOARD_COMPACT_HEIGHT_PLOT_MARGINS = {"left": 0.24, "right": 0.96, "bottom": 0.35, "top": 0.69}
```

创建图表时使用 `Figure(figsize=DASHBOARD_PLOT_FIGURE_SIZE)` 与对应高度的边距；正常窗口使用 `DASHBOARD_PLOT_MARGINS`，无法保留 320 px tabs 时使用紧凑高度边距。保留原有标题、坐标标签、图例、按钮、更新和保存连接，并用 renderer 像素边界验证两种高度都不裁切。

- [ ] **Step 4: 运行 Dashboard 聚焦回归**

Run: `conda run -n slope-sim python -m pytest tests/test_dashboard.py tests/test_dashboard_manual_verifier.py tests/test_manual_demo.py -q`

Expected: PASS。

- [ ] **Step 5: 运行全量自动测试**

Run: `conda run -n slope-sim python -m pytest -q`

Expected: 全部 PASS。

- [ ] **Step 6: 在真实桌面执行手动驾驶验证**

Run: `conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --duration-sec 4 --hold-sec 2`

Expected: 进程返回 0，输出 `dashboard_manual_motion`，Dashboard 宽度为 420 px，并可与 PyBullet 窗口并排使用。

### Review Follow-up: 真实按钮路径与 XWayland 坐标

**Files:**
- Modify: `scripts/verify_dashboard_manual_drive.py`
- Modify: `tests/test_dashboard_manual_verifier.py`

- [x] **Step 1: 为固定侧栏按钮路径写失败测试**

用 420 px 逻辑宽度覆盖滚动到底、上箭头坐标、完整 xdotool 事件顺序，并加入 1×/2× 坐标测试。

- [x] **Step 2: 复现并定位真实桌面差异**

真实 X11 显示中 Qt 使用 420×653 逻辑像素，XWayland 使用 840×1306 物理像素；`xdotool getwindowgeometry` 还包含重父化偏移，不能作为 Qt 客户区原点。

- [x] **Step 3: 使用客户区绝对几何**

通过 `xwininfo -id` 解析客户区绝对原点和物理宽高；以 `width / 420` 统一缩放 tab、滚动点和按钮偏移。缺少 `xwininfo` 时验收脚本明确返回 2。

- [x] **Step 4: 真实按钮验收**

Run: `conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --duration-sec 4 --hold-sec 2 --input-method button`

Expected: 进程返回 0，日志包含非零前进命令、可见位移且 `out_of_bounds=False`。
