# PyBullet 3D 移动机器人仿真平台

当前仓库正在按 `3d仿真平台需求规格.md` 分三阶段交付。阶段一实现四种机器人模型、三类基础场地和 GUI 人工观察，并根据用户验收反馈补充 Dashboard 运行期车型/场地切换；障碍物、eCAL 和企业传感器仍属于后续阶段。

Task6自动验证已完成；GUI人工验收待用户完成。完整结果和逐项清单见 [`docs/阶段一交付报告.md`](docs/阶段一交付报告.md)。

## 阶段一已实现范围

机器人模型：

- `df_front`：前置左右差速驱动轮，后部一个球形支撑轮。
- `df_mid`：中置左右差速驱动轮，前后各一个球形支撑轮。
- `df_back`：后置左右差速驱动轮，前部一个球形支撑轮。
- `active_steering_4wd`：四轮独立驱动，两个前轮独立主动转向。

阶段一所有公开入口都使用 PyBullet 关节物理模式；主动转向车的四个实际轮速和两个实际前轮转角会写入 CSV，并显示在 Dashboard 中。

GUI 手动模式可先在 Dashboard 选择车型或场地，再点击“应用车型”或“应用场地”执行切换。只改变下拉框不会修改正在运行的物理世界。

场地模型：

- `flat`：水平连续平面。
- `slope`：三段连续碰撞面。正 `slope_deg` 时，车辆从高位平地沿世界 `+X` 驶过下坡，再进入低位平地。
- `golf_heightfield`：单个可复现 heightfield，包含多尺度丘陵、浅洼、横坡和一条平滑驾驶廊道。廊道只适度减弱局部起伏，保留大部分丘洼，不是一条直线平路。

旧履带模型和旧场地入口已从可运行配置、CLI、Dashboard 和 URDF 中删除。

## 环境

```bash
conda activate slope-sim
python scripts/check_env.py
```

SSH 或无桌面环境请使用 `direct`；只有存在 Ubuntu X11/桌面会话时才能使用 `--gui`。

## DIRECT 快速验证

运行一次默认平面实验：

```bash
python main.py --mode direct --robot-model df_back --terrain-model flat --drive-model physics --duration-sec 1
```

运行阶段一四车型 × 三场地矩阵：

```bash
python scripts/verify_stage1_matrix.py
```

运行自动测试：

```bash
python -m pytest -q
```

## GUI 人工观察

GUI 手动模式不会按配置中的 `duration_sec` 自动退出。使用 `q`、`Esc` 或 Dashboard 的退出按钮结束。

平面示例：

```bash
python main.py --gui --manual --drive-model physics --robot-model df_front --terrain-model flat --slope-deg 0
```

斜面示例：

```bash
python main.py --gui --manual --drive-model physics --robot-model df_mid --terrain-model slope --slope-deg 8
```

高尔夫起伏示例：

```bash
python main.py --gui --manual --drive-model physics --robot-model active_steering_4wd --terrain-model golf_heightfield --golf-seed 41 --golf-relief medium
```

任意一个 GUI 命令都可以作为初始组合；启动后可直接在 Dashboard 完成其余车型和场地的运行期评估。主动转向车需要同时给前进/后退和左/右命令才会形成转弯；原地只按左/右不会像差速车一样自转。

手动控制：

- 上/下方向键：前进 / 后退。
- 左/右方向键：左转 / 右转。
- 空格：停车。
- 选择车型并点击“应用车型”：保留当前场地，从安全出生点加载目标车型。
- 选择场地及对应参数并点击“应用场地”：清零命令、重建场地，并重新投放当前车型。
- “复位车辆”按钮：清零当前命令并把车辆重新放回当前场地出生点。
- `q` 或 `Esc`：退出。
- Dashboard 或 PyBullet 窗口获得焦点时均可使用键盘。
- “启用跟随”是即时开关，不需要点击“应用”。开启时，“车后”和“侧面”视角都会随车头 yaw 旋转；“固定”视角只跟随车辆位置，保持配置的世界 yaw。关闭后可用 PyBullet 鼠标自由调整相机。

场地切换会调用 PyBullet `resetSimulation()`，因此车辆也会随场地一起重建。若目标场地创建失败，程序会自动恢复上一个有效场地和车型，并在 Dashboard 显示错误。CSV 仍记录为同一个手动会话，切换前后的车型和场地由每行字段区分；实时曲线会清空，避免连接两个不同世界的坐标。

## 高尔夫场地复现

```bash
python main.py --mode direct --robot-model df_back --terrain-model golf_heightfield --golf-seed 23 --golf-relief low
python main.py --mode direct --robot-model df_back --terrain-model golf_heightfield --golf-seed 23 --golf-relief high
```

同一 `golf_seed` 和 `golf_relief` 会生成相同高度数据。`low`、`medium`、`high` 只暴露易理解的起伏预设，网格分辨率和碰撞尺度属于内部参数。

## 本轮物理证据

四驱的可复制 DIRECT 诊断、关节顺序、`getJointState()` 字段和本轮输出统一记录在[交付报告的主动转向四驱逐关节诊断](docs/阶段一交付报告.md#主动转向四驱逐关节诊断)。该处是本项目四驱物理证据的唯一复现入口。

## GUI 验收步骤

以下步骤供桌面会话人工验收使用，本次文档更新未打开真实 GUI：

1. 运行上面的 8 度 `slope` 命令，前进驾驶，确认车辆从高位平地沿 `+X` 依次经过下坡和低位平地。
2. 在 Dashboard 勾选和取消“启用跟随”，分别选择“车后”“侧面”“固定”；转弯时前两者随车头旋转，固定视角只移动 target。
3. 运行同一组 `golf_seed` 和 `golf_relief` 两次并重新应用场地，确认丘陵、浅洼和驾驶廊道一致；改变任一参数后确认地形变化。
4. 选择 `active_steering_4wd`，同时按前进和左/右，确认 Dashboard 的四轮实际轮速与两个前轮实际转角均更新。

## 主要目录

```text
slope_sim/model_registry.py       四车型注册表和语义关节元数据
slope_sim/robot.py                差速与主动转向控制、物理状态读取
slope_sim/scene.py                flat/slope/golf_heightfield 场地创建
slope_sim/simulation.py           DIRECT/GUI 自动实验流程
slope_sim/manual_demo.py          GUI 手动驾驶流程
slope_sim/dashboard.py            当前阶段的驾驶与诊断窗口
urdf/df_front.urdf                前置差速模型
urdf/df_mid.urdf                  中置差速模型
urdf/df_back.urdf                 后置差速模型
urdf/active_steering_4wd.urdf     主动转向四驱模型
scripts/verify_stage1_matrix.py   4×3 DIRECT 验证
tests/                            自动测试
```

## 开源设计参考

- Bullet 官方 `racecar.py`：前轮转向和驱动关节控制方式。
- Bullet 官方 `racecar_differential.py`：四轮联动和齿轮约束思路；本项目不使用硬编码关节索引。
- Bullet 官方 `heightfield.py`：`GEOM_HEIGHTFIELD` 的创建方式。
- 仓库内 `references/repos/pybullet_diffdrive`：差速轮和球形支撑轮布局。
- 仓库内 `references/repos/pybullet_sim`：差速控制、射线和仿真封装方式。

阶段一详细范围、自动验证和人工验收门禁见 `3d仿真平台需求规格.md` 第 14.1 节。
