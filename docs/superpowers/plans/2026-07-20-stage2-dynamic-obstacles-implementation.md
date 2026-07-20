# 阶段二 Dashboard 动态场景与障碍物实施计划

> **执行要求：** 实施时使用 `executing-plans`，严格按任务顺序推进。每个功能先写失败测试，再写最小实现；第一项物理可行性门禁未通过时立即停止，不进入 Dashboard 开发。

**目标：** 在阶段一稳定车型、场地与侧边栏基础上，实现可复现、可追加、可删除、可跨场地恢复的静态/移动障碍物，并建立轻量 `SimulationCoordinator` 与 `ObstacleManager` 边界。

**架构：** `SimulationCoordinator` 留在 PyBullet 主线程，统一串行处理车型、场地、复位和障碍物结构操作；`ObstacleManager` 保存稳定逻辑状态、跨帧生成任务和运动学刚体；Dashboard 只发送类型化命令并消费只读快照。场地重建以逻辑快照恢复障碍物，失败时恢复旧世界。

**技术栈：** Python 3.10、PyBullet、PySide6、pytest、现有 CSV/JSON 日志设施。

**范围约束：** 本计划不实现 eCAL、多线雷达、RTK、企业 IMU、自动刹停、路径规划或自动避障。自动导航已记录为阶段四。

---

## Task 1：运动学障碍物碰撞可行性门禁

**文件：**

- 新建：`slope_sim/obstacles.py`
- 新建：`tests/test_obstacles.py`
- 参考：`slope_sim/robot.py`
- 参考：`tests/test_robot_models.py`

### Step 1：写质量为零障碍物的失败测试

在 `tests/test_obstacles.py` 建立最小 DIRECT 场景，覆盖：

- 创建质量为零的箱体障碍物。
- 在 `stepSimulation()` 前按固定小步更新移动体位姿与切向速度。
- 障碍物路径横向误差 `<= 1e-6 m`。
- 接触穿透深度 `<= 0.03 m`。
- 车辆碰撞期间线速度 `<= 3.0 m/s`、角速度 `<= 10.0 rad/s`，全部状态有限。
- 相同车辆初态、控制命令和时长下，静态障碍场景前进位移 `<=` 无障碍基线的 50%。

测试必须复用现有车辆工厂与物理参数，不手写一个与正式车辆无关的替代小车。

### Step 2：运行测试确认红灯

```bash
conda run -n slope-sim python -m pytest tests/test_obstacles.py -q
```

预期：因 `slope_sim.obstacles` 或最小刚体 API 尚不存在而失败。

### Step 3：实现最小刚体与位姿更新 API

在 `slope_sim/obstacles.py` 中加入文件头中文注释，并实现可供测试使用的最小接口：

- 创建箱体碰撞/可视形状和质量为零的 body。
- 在物理步进前更新位姿。
- 设置与路径一致的线速度，保持刚体自身不接受车辆碰撞反推。
- 失败时删除已经创建的半成品 body。

此时不要实现随机规划、Dashboard 或协调器。

### Step 4：运行门禁测试

```bash
conda run -n slope-sim python -m pytest tests/test_obstacles.py -q
```

预期：上述数值判据全部通过。若质量为零并逐帧更新无法稳定满足判据，立即停止阶段二实施，记录实际穿透、速度尖峰和车辆位移，回到设计评审，不继续 Task 2。

### Step 5：提交门禁结果

```bash
git add slope_sim/obstacles.py tests/test_obstacles.py
git commit -m "阶段二: 1. 碰撞门禁"
```

---

## Task 2：限定地形 body 的地表采样

**文件：**

- 修改：`slope_sim/scene.py`
- 修改：`tests/test_scene.py`
- 修改：`tests/test_obstacles.py`

### Step 1：写地形过滤失败测试

新增测试：

- 地面之上放置车辆或箱体，传入 `SceneInfo.body_ids` 后仍返回真实地面高度与法向。
- 射线从移动障碍物自身上方开始时不会命中自身顶面。
- 多个非地形 body 叠在射线上方时能逐个跳过。
- XY 越过 `TerrainBounds` 时仍返回无效/越界。

### Step 2：运行聚焦测试确认红灯

```bash
conda run -n slope-sim python -m pytest tests/test_scene.py tests/test_obstacles.py -q
```

预期：现有 `probe_terrain()` 会返回第一命中，新增过滤断言失败。

### Step 3：扩展 `probe_terrain()`

给 `probe_terrain()` 增加可选的允许 body ID 集合。射线命中非允许 body 时，从命中点下方小偏移继续向下探测，设置明确的最大跳过次数；只接受当前 `SceneInfo.body_ids` 中的 body。保留原调用不传过滤集合时的阶段一行为。

重要段落添加简短中文注释，说明为什么必须跳过车辆与障碍物自身。

### Step 4：运行测试

```bash
conda run -n slope-sim python -m pytest tests/test_scene.py tests/test_obstacles.py tests/test_simulation_smoke.py -q
```

预期：通过，且阶段一地形探测回归不变。

### Step 5：提交

```bash
git add slope_sim/scene.py tests/test_scene.py tests/test_obstacles.py
git commit -m "阶段二: 1. 地形采样"
```

---

## Task 3：确定性障碍物领域模型与规划器

**文件：**

- 修改：`slope_sim/obstacles.py`
- 修改：`tests/test_obstacles.py`

### Step 1：写领域规则失败测试

覆盖以下纯规则：

- `static`、`moving`、`mixed` 参数校验。
- 混合数量采用 `floor(count * ratio + 0.5)`，`count >= 2` 时夹紧为至少一个静态和一个移动。
- 每批范围 `1..50`，场景总数不超过 100。
- 同一场地、已有逻辑集合、请求和种子生成完全相同的模式、尺寸、位置、朝向与路径。
- 使用请求局部随机数，不改变 Python 全局随机状态。
- 外接半径完整位于边界内，并避开出生保护区、当前车辆 AABB、已有对象和本批对象。
- 移动路径整段位于场地内，且扫掠走廊不与其他对象或路径相交。
- 有限候选次数用尽时返回明确错误，不返回半批结果。
- 端点反向能消费剩余位移，较大 `dt` 下不越界。

规划器测试使用可注入地表采样回调，不依赖真实 GUI。

### Step 2：确认红灯

```bash
conda run -n slope-sim python -m pytest tests/test_obstacles.py -q
```

### Step 3：实现不可变领域对象

在 `slope_sim/obstacles.py` 中实现并验证：

- `ObstacleGenerationSettings`
- `ObstacleGenerationRequest`
- `ObstacleGeometry`
- `ObstaclePath`
- `ObstacleSpec`
- `ObstacleSnapshot`
- 混合比例函数、线段/扫掠距离函数和往返进度函数

逻辑 ID 与 PyBullet body ID 分离；逻辑 ID 分配顺序稳定。

### Step 4：实现确定性规划器

规划器逐候选生成尺寸、模式、位置与路径，使用保守外接圆完成第一层快速排斥，再对线段扫掠距离做路径校验。地表高度与法向通过回调取得，姿态组合要同时保留路径 heading 与地表法向。

### Step 5：运行测试并提交

```bash
conda run -n slope-sim python -m pytest tests/test_obstacles.py -q
git add slope_sim/obstacles.py tests/test_obstacles.py
git commit -m "阶段二: 1. 障碍规划"
```

---

## Task 4：ObstacleManager 生命周期与跨帧事务

**文件：**

- 修改：`slope_sim/obstacles.py`
- 新建：`tests/test_obstacle_manager.py`

### Step 1：写管理器失败测试

使用真实 DIRECT 客户端和少量可控 fake clock，覆盖：

- 箱体、圆柱体、球体的质量、碰撞形状、视觉形状和贴地姿态。
- 静态对象不主动移动；移动对象逐步更新并在端点反向。
- Dashboard 快照不暴露可变内部记录或 body ID。
- 删除单个、删除不存在 ID、清空全部。
- 创建中途抛错时只移除本批暂存 body，原逻辑集合不变。
- 添加规划和隐藏暂存按注入时钟的 2 ms 软预算跨帧让出。
- 暂存 body 不可见、不碰撞；整批成功后逻辑列表一次发布。
- 最终提交前重新读取车辆合并 AABB，车辆进入候选区域时撤销整批。
- 快照恢复保持逻辑 ID、XY、路径、进度和方向，仅更新 Z、姿态与 body ID。

### Step 2：确认红灯

```bash
conda run -n slope-sim python -m pytest tests/test_obstacle_manager.py -q
```

### Step 3：实现 `ObstacleManager`

实现：

- `begin_add()`、`advance_pending_operation()`、`delete()`、`begin_clear()`。
- 隐藏且禁用碰撞的暂存 body。
- 原子逻辑提交、失败清理和状态结果。
- `update_moving(dt)`：先限定地形 body 采样，再在物理步进前更新位姿与切向速度。
- `snapshot()` 与 `restore()`。

将 PyBullet 调用集中在管理器，纯规则仍保持无客户端依赖。

### Step 4：运行聚焦与物理测试

```bash
conda run -n slope-sim python -m pytest tests/test_obstacles.py tests/test_obstacle_manager.py tests/test_scene.py -q
```

### Step 5：提交

```bash
git add slope_sim/obstacles.py tests/test_obstacle_manager.py
git commit -m "阶段二: 1. 障碍管理"
```

---

## Task 5：运行期操作协议与 SimulationCoordinator

**文件：**

- 新建：`slope_sim/runtime_actions.py`
- 新建：`slope_sim/coordinator.py`
- 修改：`slope_sim/manual_demo.py`
- 修改：`slope_sim/dashboard.py`
- 新建：`tests/test_coordinator.py`
- 修改：`tests/test_manual_demo.py`
- 修改：`tests/test_dashboard.py`

### Step 1：写协调器事务失败测试

覆盖：

- FIFO 每次只启动一个结构操作。
- 车型切换和车辆复位保留管理器及障碍物 body。
- 场地切换保存逻辑快照，重建后保持 ID、XY、路径进度与方向。
- 目标场地无法容纳快照时恢复旧地形、旧车型和全部障碍物。
- 目标失败且回滚也失败时错误同时包含两段原因。
- 场地/车型/复位是安全停车操作；添加/删除/清空不清零持续驾驶命令。
- 结构任务未完成时协调器继续执行 `update_moving()` 和物理步进。
- 重建后没有重复地形、车辆或障碍物 body。

### Step 2：确认红灯

```bash
conda run -n slope-sim python -m pytest tests/test_coordinator.py tests/test_manual_demo.py -q
```

### Step 3：建立共享操作协议

在 `runtime_actions.py` 放置不依赖 Qt 的：

- `TerrainSelection`
- `SwitchRobotAction`
- `SwitchTerrainAction`
- `ResetRobotAction`
- `AddObstaclesAction`
- `DeleteObstacleAction`
- `ClearObstaclesAction`
- `RuntimeAction` 联合类型

让 Dashboard 和协调器共同导入这些对象，避免协调器依赖 Qt。`DashboardCommand` 保留驾驶、退出、相机状态，并改为最多携带一个 `structural_action`。

### Step 4：实现轻量协调器

将 `ActiveManualRobot`、`ActiveManualWorld` 及车型/场地事务移入 `coordinator.py`，`manual_demo.py` 只保留输入合并、限速和主循环。协调器持有当前 world、`ObstacleManager`、待执行操作和最近结果。

场地切换流程必须是：快照 → 目标世界重建 → 障碍物恢复 → 成功提交；任一目标步骤失败后，用旧选择重建世界并恢复同一快照。

### Step 5：更新阶段一命令测试并运行

```bash
conda run -n slope-sim python -m pytest tests/test_coordinator.py tests/test_manual_demo.py tests/test_dashboard.py -q
```

### Step 6：提交

```bash
git add slope_sim/runtime_actions.py slope_sim/coordinator.py slope_sim/manual_demo.py slope_sim/dashboard.py tests/test_coordinator.py tests/test_manual_demo.py tests/test_dashboard.py
git commit -m "阶段二: 1. 场景协调"
```

---

## Task 6：Dashboard 障碍物页面与控制组

**文件：**

- 修改：`slope_sim/dashboard.py`
- 修改：`tests/test_dashboard.py`
- 修改：`tests/test_dashboard_manual_verifier.py`
- 修改：`scripts/verify_dashboard_manual_drive.py`

### Step 1：写 Dashboard 离屏失败测试

覆盖：

- 顶部存在“障碍物”标签页，表格列为逻辑 ID、模式、形状、位置。
- 表格单选；刷新位置后按逻辑 ID 保留选中行。
- 下方 `control_groups` 顺序包含“障碍物”，且仍位于滚动内容中。
- 静态、移动、混合三种模式正确启用速度和比例控件。
- 默认移动比例为 30%，数量范围 1–50。
- 添加命令完整捕获模式、形状、数量、种子、速度和比例。
- 未选择行时禁用删除；选择后删除命令使用逻辑 ID。
- 清空直接入队，不创建模态对话框。
- 多次结构操作按 FIFO 输出，不丢失、不重复。
- 忙碌期间相关按钮禁用，完成/失败后恢复并显示状态。
- 新标签页不破坏窄侧栏、方向键焦点、绘图节流和按钮脉冲。

### Step 2：确认红灯

```bash
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest tests/test_dashboard.py tests/test_dashboard_manual_verifier.py -q
```

### Step 3：实现障碍物表格与控件

在顶部 tabs 增加障碍物单选表格，在下方滚动区增加障碍物组。实现：

- `_pending_actions` FIFO。
- `update_obstacle_snapshots()` 的低频增量刷新。
- 按逻辑 ID 恢复选择。
- `set_structure_busy()` 与统一状态文本。

保持 Qt 回调不调用 PyBullet。

### Step 4：适配 GUI 验证器坐标/标签定位

更新现有 Dashboard 自动点击辅助逻辑，使新增标签页后仍按标签或稳定几何定位驾驶按钮与曲线页，避免硬编码旧 tab 数量。

### Step 5：运行测试并提交

```bash
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest tests/test_dashboard.py tests/test_dashboard_manual_verifier.py -q
git add slope_sim/dashboard.py tests/test_dashboard.py tests/test_dashboard_manual_verifier.py scripts/verify_dashboard_manual_drive.py
git commit -m "阶段二: 1. 障碍界面"
```

---

## Task 7：手动循环集成与障碍物事件日志

**文件：**

- 修改：`slope_sim/manual_demo.py`
- 修改：`slope_sim/logger.py`
- 修改：`slope_sim/simulation.py`
- 修改：`main.py`
- 修改：`tests/test_manual_demo.py`
- 修改：`tests/test_logger.py`

### Step 1：写主循环和日志失败测试

覆盖：

- 主循环每帧处理 Qt 事件、读取驾驶命令、推进一个结构任务切片、更新移动障碍物、再执行 `stepSimulation()`。
- 添加/删除期间驾驶命令持续生效；车型/场地/复位帧立即停车。
- Dashboard 只收到协调器的只读快照和操作结果。
- JSONL 事件包含仿真时间、事件类型、逻辑 ID、请求参数、种子、车型、场地、成功状态和错误原因。
- 添加失败、删除、清空、场地重建、目标失败和回滚均有事件。
- 日志关闭幂等，中途异常时也关闭文件。
- `SimulationResult` 和 CLI 能报告障碍物事件日志路径。

### Step 2：确认红灯

```bash
conda run -n slope-sim python -m pytest tests/test_manual_demo.py tests/test_logger.py -q
```

### Step 3：实现 JSONL 日志器

在 `logger.py` 添加带中文头注释/关键函数注释的障碍物事件日志器，使用逐行 JSON、稳定字段和及时 flush。不要把不同结构的事件硬塞进车辆遥测 CSV。

### Step 4：接入主循环

`run_manual_demo()` 启动协调器和事件日志器；结构操作结果驱动 Dashboard busy/status；移动障碍物更新必须发生在物理步进前。退出和异常路径统一清理日志、Dashboard、障碍物及 PyBullet 客户端。

### Step 5：运行测试并提交

```bash
conda run -n slope-sim python -m pytest tests/test_manual_demo.py tests/test_logger.py tests/test_simulation_smoke.py -q
git add slope_sim/manual_demo.py slope_sim/logger.py slope_sim/simulation.py main.py tests/test_manual_demo.py tests/test_logger.py
git commit -m "阶段二: 1. 循环日志"
```

---

## Task 8：阶段二 DIRECT、性能与 GUI 验收入口

**文件：**

- 新建：`scripts/verify_stage2_obstacles.py`
- 新建：`configs/stage2_obstacles_gui.yaml`
- 新建：`tests/test_stage2_obstacle_verifier.py`
- 修改：`README.md`
- 修改：`tests/test_entrypoints.py`
- 修改：`tests/test_experiment_entrypoint.py`
- 修改：`tests/test_obstacle_manager.py`

### Step 1：写验证脚本辅助函数测试

先测试报告聚合与失败判据，不让脚本只能靠人工读输出：

- 三类场地的贴地误差。
- 同种子布局摘要哈希。
- 运动端点反向次数和路径误差。
- 删除/清空后的 body 数量。
- 50 个添加与 100 个清空的最大物理步、Qt 事件间隔。

跨帧 2 ms 软预算使用 fake clock 单测；100 ms 真实墙钟只由验收机脚本报告并判定，不作为共享 CI 的脆弱断言。

### Step 2：实现 DIRECT 验证脚本

`verify_stage2_obstacles.py` 至少运行：

- 三种形状 × 三种场地的创建与贴地。
- 相同种子复现。
- 移动往返和碰撞门禁。
- 车型切换、车辆复位、场地重建及失败回滚。
- 删除单个、清空全部、事件日志。
- 批量结构操作性能报告。

输出逐项 `PASS/FAIL` 和最终汇总，任一硬门禁失败时返回非零退出码。

### Step 3：增加 GUI 配置和文档

新增阶段二 GUI 示例配置，README 补充：

- 启动命令。
- 混合 30% 追加生成。
- 表格选择删除与清空。
- 场地切换保持逻辑布局。
- 移动障碍物是运动学刚体、不会被车辆撞偏。
- 自动导航不在阶段二，属于阶段四。

### Step 4：运行阶段二与阶段一回归

```bash
conda run -n slope-sim python scripts/verify_stage2_obstacles.py
conda run -n slope-sim python scripts/verify_stage1_matrix.py
conda run -n slope-sim python -m pytest -q
```

预期：三个命令均返回 0；记录测试数量和性能数据，不能只写“已通过”。

### Step 5：提交

```bash
git add scripts/verify_stage2_obstacles.py configs/stage2_obstacles_gui.yaml tests/test_stage2_obstacle_verifier.py README.md tests/test_entrypoints.py tests/test_experiment_entrypoint.py tests/test_obstacle_manager.py
git commit -m "阶段二: 1. 集成验收"
```

---

## Task 9：独立六维审查、修复与阶段交付

**文件：**

- 新建：`docs/阶段二交付报告.md`
- 按审查发现修改相关源文件和测试

### Step 1：启动独立审查线程

审查线程只检查、不直接修改，从以下六方面给出带文件和行号的问题：

1. 需求完整性。
2. 逻辑正确性。
3. 边界情况。
4. 代码质量与中文注释。
5. 测试覆盖。
6. DIRECT、性能和 GUI 实际运行结果。

### Step 2：逐项验证审查意见

对每一项先复现，再决定是否修改。禁止未经验证直接照单全收；若需修复，先增加或调整失败测试，再改实现，并按项目提交格式单独提交经过验证的修复。

### Step 3：重新运行完整验证

```bash
conda run -n slope-sim python scripts/verify_stage2_obstacles.py
conda run -n slope-sim python scripts/verify_stage1_matrix.py
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest -q
git diff --check
```

在真实 Ubuntu 桌面会话中运行：

```bash
conda run -n slope-sim python main.py --config configs/stage2_obstacles_gui.yaml --gui --manual
```

按设计文档第 7.2 节完成 GUI 清单，记录观察结果；若当前执行环境没有桌面，只能明确标记“待用户 GUI 验收”，不能声称 GUI 已通过。

### Step 4：编写阶段二交付报告

报告必须包含：

- 已完成与明确未完成范围。
- 文件用途和关键 PyBullet 机制说明。
- 实际测试命令、数量、耗时及性能数据。
- GUI 操作步骤。
- 已知问题和残余风险。
- “当前已停止，等待用户人工验收和反馈”。

### Step 5：提交阶段二候选交付

提交前再次检查只包含阶段二相关文件，并遵守最多三个数字分点、每点不超过 10 个字的提交概述：

```bash
git add docs/阶段二交付报告.md
git commit -m "阶段二: 1. 完成验收"
```

提交后停止开发，不进入阶段三或阶段四，等待用户人工验收。
