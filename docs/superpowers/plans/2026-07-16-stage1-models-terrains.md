# 阶段一实施计划：四种车型与三类基础场地

> 状态：Task6自动验证已完成；GUI人工验收待用户完成
> 范围依据：`3d仿真平台需求规格.md` 第 14.1 节
> 阶段门禁：自动验证完成后停止，等待用户 GUI 人工验收；不得提前实施阶段二。

## 1. 本阶段范围

- 删除 `tracked_proxy` 的 URDF、配置入口、命令行入口、Dashboard 车型选项和运行测试。
- 新增 `df_front`、`df_mid`、`df_back`、`active_steering_4wd` 四个模型。
- 建立车型注册表，统一保存 URDF、控制类型、轮距、轮径、出生高度和语义关节名。
- 差速车型使用左右轮差速；主动转向车型使用四轮驱动、前轮转向速度积分与机械限位。
- 新增 `flat`、`slope`、`golf_heightfield` 三类启动时可选场地。
- 保留启动配置、CLI、GUI 手动驾驶、相机跟随、复位、日志和 Dashboard 诊断能力。

本阶段不实现 Dashboard 运行期场地切换、障碍物、eCAL 或企业传感器。

## 2. TDD 执行顺序

### 任务 A：车型注册表和 URDF

1. 修改 `tests/test_config.py`、`tests/test_entrypoints.py`，先断言只接受四个新车型并拒绝 `tracked_proxy`。
2. 新增 `tests/test_robot_models.py`，先断言：
   - 四个模型均在注册表中且 URDF 存在；
   - 三个差速模型各有 2 个驱动轮，支撑轮数量分别为 1/2/1；
   - 驱动轴纵向位置分别在前/中/后；
   - 主动转向模型有 4 个驱动关节和 2 个前轮转向关节；
   - 所有关节均通过名称解析，不依赖硬编码索引。
3. 运行聚焦测试并确认因实现缺失而失败。
4. 新增 `slope_sim/model_registry.py` 和四个 URDF，最小实现使结构测试通过。

### 任务 B：差速与主动转向控制

1. 在 `tests/test_robot_models.py` 中增加失败测试：
   - 三种差速车前进、后退、左右转和原地转向命令正确下发；
   - 主动转向四轮均收到驱动命令；
   - 两个前轮转向速度按步长积分，并限制在机械角范围；
   - 实际轮速和转向角从 PyBullet 关节状态读取。
2. 修改 `slope_sim/robot.py`，保留共享加载/摩擦/遥测逻辑，拆分 `DifferentialDriveRobot` 与 `ActiveSteeringRobot`。
3. 增加统一机器人创建函数，修改 `slope_sim/simulation.py` 和 `slope_sim/manual_demo.py` 使用注册表。
4. 运行机器人聚焦测试和 DIRECT 物理 smoke test。

### 任务 C：三类场地

1. 重写 `tests/test_scene.py`，先断言 `flat`、`slope`、`golf_heightfield` 的类型、边界、出生点和地面探测。
2. 增加同一 `golf_seed` / `golf_relief` 生成完全一致高度数据的失败测试。
3. 修改 `slope_sim/scene.py`：
   - `flat` 使用水平静态碰撞面；
   - `slope` 使用连续可碰撞斜面；
   - `golf_heightfield` 使用低频正弦和圆滑小丘生成连续 heightfield。
4. 运行场地聚焦测试，随后对四车型 × 三场地做短时 DIRECT 加载 smoke test。

### 任务 D：配置、入口、GUI 观察与文档

1. 修改 `slope_sim/config.py`、`main.py` 和 `configs/*.yaml`，只暴露四车型和三场地，并加入 `golf_seed`、`golf_relief`。
2. 修改 `slope_sim/dashboard.py`，删除履带选项并列出四车型；不新增阶段二场地切换和障碍物控件。
3. 更新 `README.md`、`ARCHITECTURE.md`，提供每种车型和场地的 GUI 启动命令及开源参考说明。
4. 删除 `urdf/tracked_proxy.urdf` 和履带调参脚本/测试；把相关旧回归替换为阶段一车型回归。

## 3. 验证命令

聚焦测试：

```bash
python -m pytest -q tests/test_robot_models.py tests/test_scene.py tests/test_config.py tests/test_entrypoints.py
```

完整测试：

```bash
python -m pytest -q
```

DIRECT 矩阵 smoke test：

```bash
python scripts/verify_stage1_matrix.py
```

GUI 人工验收由用户自行决定观察时长；本阶段不会增加定时自动退出要求。

## 4. 完成条件

- 四个 URDF 均可加载，关节和支撑布局符合规格。
- 三个差速模型和主动转向模型的基础控制通过自动测试。
- 三种场地可在 DIRECT 和 GUI 启动，固定种子地形可复现。
- `tracked_proxy` 不再是可运行选项。
- 完整自动测试与 DIRECT 矩阵通过。
- 输出 GUI 命令、人工检查清单和已知限制后停止。
