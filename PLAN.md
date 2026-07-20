# PyBullet 斜坡移动底盘仿真

> 历史计划说明：本文保留阶段一需求重构前的开发记录，旧车型、旧场地和旧命令仅用于追溯，不再是当前运行入口。当前阶段一实施范围请以 `3d仿真平台需求规格.md` 第 14.1 节、`README.md` 和 `ARCHITECTURE.md` 为准。

用于实现一个面向斜坡/草坪起伏场景的轻量级 3D 移动机器人仿真器。第一版目标不是做高精度通用物理引擎，而是建立一个可运行、可记录、可分析的实验平台，用于验证移动底盘在不同坡度下的轨迹跟踪效果。

## 当前交付状态

截至 2026-07-16，当前工作树已完成需求规格第 14.1 节的阶段一开发方自动验证，交付四种轮式车型和 `flat`、`slope`、`golf_heightfield` 三类场地，正在等待用户 GUI 人工验收。准确结果见 `docs/阶段一交付报告.md`。

旧 `tracked_proxy`、`diff_drive`、`dam_slope`、大坝 GUI 配置和相关命令已删除。本文后续章节是需求重构前的历史计划，不代表当前可运行能力，也不能作为 GUI/X11 已验收的证据。

## 历史计划正文（以下状态和命令仅适用于旧提交）

## 1. 定位

### 1.1 目标

- 使用 PyBullet 搭建轻量级 3D 仿真环境。
- 支持不同坡度的斜坡场景。
- 建立简化移动机器人底盘模型，优先选择差速轮底盘。
- 接入基础导航或轨迹跟踪算法。
- 支持简单传感器噪声和控制误差。
- 输出真实轨迹、估计轨迹、参考路径、误差指标和实验图表。

## 2. 推荐运行环境

### 2.1 主环境

推荐使用 Ubuntu 实体机作为主运行环境：

```text
系统：Ubuntu 24.04
Python 环境：Miniforge / Conda
Python 版本：3.10
仿真：PyBullet
编辑器：Windows VS Code + Remote SSH
GUI 测试：Ubuntu 笔记本本机 X11 桌面
批量实验：SSH + PyBullet DIRECT 模式
```

不推荐把代码主要放在 Windows 上再远程调用 Ubuntu 运行。更稳的方式是：

```text
代码实际存放：Ubuntu ~/projects/slope-sim
代码编辑入口：Windows VS Code Remote SSH
运行环境：Ubuntu conda 环境 slope-sim
```

### 2.2 GUI 使用原则

PyBullet 有两种常用连接模式：

```python
p.connect(p.GUI)
p.connect(p.DIRECT)
```

建议规则：

| 场景 | 推荐模式 |
|---|---|
| Ubuntu 本机 X11 桌面调试 | `p.GUI` |
| SSH 远程运行 | `p.DIRECT` |
| Windows VS Code Remote SSH | `p.DIRECT` |
| Ubuntu 远程桌面 Wayland 会话 | 不运行 `p.GUI` |
| 批量实验 / 生成数据 | `p.DIRECT` |

## 3. 环境配置检测

### 3.1 检查系统和 Python

```bash
lsb_release -a
python3 --version
conda --version
```

Ubuntu 24.04 通常自带 Python 3.12，不建议直接把 PyBullet 装进系统 Python。建议使用 Conda 创建 Python 3.10 环境。

### 3.2 创建 Conda 环境

```bash
conda create -n slope-sim python=3.10 -y
conda activate slope-sim
conda install -c conda-forge pybullet numpy matplotlib pandas scipy jupyterlab ipykernel -y
```

确认环境：

```bash
which python
python --version
python -c "import pybullet as p; print('pybullet api version:', p.getAPIVersion())"
```

期望：

```text
Python 3.10.x
pybullet api version: <number>
```

### 3.3 检查 PyBullet DIRECT

```bash
conda activate slope-sim
python -c "import pybullet as p; cid=p.connect(p.DIRECT); print('DIRECT connected:', cid); p.disconnect()"
```

期望输出：

```text
DIRECT connected: 0
```

### 3.4 检查桌面会话类型

```bash
echo $XDG_SESSION_TYPE
```

常见结果：

```text
x11
wayland
tty
```

解释：

| 输出 | 含义 |
|---|---|
| `x11` | 本机 X11 桌面，会更适合 PyBullet GUI。 |
| `wayland` | Wayland 桌面或远程桌面，PyBullet GUI 可能不稳定。 |
| `tty` | SSH / 纯终端，没有本地图形显示。 |

### 3.5 检查 OpenGL

```bash
sudo apt update
sudo apt install -y mesa-utils libgl1 libglu1-mesa
glxinfo -B
```

重点看：

```text
OpenGL vendor string
OpenGL renderer string
OpenGL version string
```

如果 `glxinfo -B` 自己就报错或卡死，优先处理显卡驱动/OpenGL，不要继续测试 PyBullet GUI。

### 3.6 检查 GUI

只在 Ubuntu 本机 X11 桌面测试：

```bash
conda activate slope-sim
python -c "import pybullet as p, time; cid=p.connect(p.GUI); print('GUI connected:', cid); time.sleep(3); p.disconnect()"
```

如果在 SSH 或远程桌面下运行，出现如下报错是正常的：

```text
cannot connect to X server
```

这种情况下应使用 `DIRECT` 模式。

## 4. 常用启动命令

### 4.1 进入项目

```bash
cd ~/projects/slope-sim
conda activate slope-sim
```

### 4.2 无 GUI 运行实验

```bash
python main.py
```

默认建议使用：

```python
p.connect(p.DIRECT)
```

### 4.3 本机 GUI 调试

```bash
python main.py --gui
```

代码中建议使用参数控制：

```python
if args.gui:
    physics_client = p.connect(p.GUI)
else:
    physics_client = p.connect(p.DIRECT)
```

### 4.4 运行指定坡度实验

建议后续支持：

```bash
python main.py --slope-deg 5
python main.py --slope-deg 10
python main.py --slope-deg 15
python main.py --slope-deg 20
```

### 4.5 批量实验

建议后续支持：

```bash
python experiments/run_slope_sweep.py --slopes 0 5 10 15 20 --trials 5
```

输出：

```text
results/logs/
results/figures/
```

### 4.6 生成分析图

建议后续支持：

```bash
python analysis.py --log results/logs/latest.csv
```

输出：

```text
真实轨迹 vs 估计轨迹 vs 参考路径
坡度 vs 终点误差
坡度 vs 平均轨迹误差
坡度 vs 航向误差
```

## 5. 常用 Debug 命令

### 5.1 检查 Conda 环境

```bash
conda info --envs
conda activate slope-sim
which python
python --version
pip list | grep -E "pybullet|numpy|matplotlib|pandas|scipy"
```

### 5.2 检查 PyBullet 能否导入

```bash
python -c "import pybullet as p; print(p.getAPIVersion())"
```

### 5.3 检查 DIRECT 模式

```bash
python -c "import pybullet as p; cid=p.connect(p.DIRECT); print(cid); p.disconnect()"
```

### 5.4 检查 GUI 模式

```bash
echo $XDG_SESSION_TYPE
echo $DISPLAY
python -c "import pybullet as p, time; cid=p.connect(p.GUI); print(cid); time.sleep(3); p.disconnect()"
```

### 5.5 SSH 下检查图形显示

SSH 默认是：

```text
XDG_SESSION_TYPE=tty
```

此时不要直接运行 PyBullet GUI。如果确实需要把窗口显示到 Ubuntu 本机屏幕，先在本机 X11 终端执行：

```bash
echo $DISPLAY
xhost +SI:localuser:$(whoami)
```

再在 SSH 终端设置对应 DISPLAY：

```bash
export DISPLAY=:0
```

然后先测试普通 X11 程序：

```bash
sudo apt install -y x11-apps x11-utils
xdpyinfo | head
xeyes
```

只有 `xeyes` 能显示后，才测试 PyBullet GUI。

### 5.6 记录仿真状态

建议在调试阶段每隔固定步数打印：

```text
step
time
position x/y/z
roll/pitch/yaw
linear velocity
angular velocity
current slope angle
tracking error
```

## 6. 参考仓库

### 6.1 主参考：差速底盘最小例子

- 仓库：[thedeepestreality/pybullet_diffdrive](https://github.com/thedeepestreality/pybullet_diffdrive)
- 作用：学习最小差速底盘 URDF、左右轮速度控制、轨迹记录。
- 重点文件：
  - `diff_drive.urdf`
  - `run.py`

适合从这里改出第一版底盘。

### 6.2 主参考：路径跟踪算法

- 仓库：[AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)
- 作用：学习路径规划和路径跟踪算法。
- 重点方向：
  - Pure Pursuit
  - Stanley Control
  - Dynamic Window Approach
  - A*
  - RRT

第一版建议优先移植 Pure Pursuit，不建议一开始使用 MPC。

### 6.3 学习参考：PyBullet 移动机器人算法

- 仓库：[akinami3/PybulletRobotics](https://github.com/akinami3/PybulletRobotics)
- 作用：系统学习 PyBullet 移动机器人基础。
- 重点目录：
  - `PybulletBasic/`
  - `MobileRobot/`

重点 Notebook：

```text
mobile_robot_basic_en.ipynb
mobile_robot_sensor_en.ipynb
mobile_robot_wheel_odometry_en.ipynb
mobile_robot_global_path_planning_a_star_en.ipynb
mobile_robot_local_path_planning_dwa_en.ipynb
mobile_robot_extended_kalman_filter_en.ipynb
```

### 6.4 官方参考：PyBullet 轮式模型

- 仓库：[bulletphysics/bullet3](https://github.com/bulletphysics/bullet3)
- 作用：参考官方 PyBullet 写法、URDF、joint control、racecar 示例。
- 重点文件：
  - `examples/pybullet/examples/racecar_differential.py`
  - `examples/pybullet/gym/pybullet_data/racecar/racecar.urdf`
  - `examples/pybullet/gym/pybullet_data/racecar/racecar_differential.urdf`

注意：racecar 不是差速底盘，不建议一开始照抄其完整结构。

### 6.5 斜坡参考：两轮机器人爬坡

- 仓库：[ngzhili/Two-Wheel-Robot-DeepRL](https://github.com/ngzhili/Two-Wheel-Robot-DeepRL)
- 作用：参考斜坡/terrain 设置。
- 注意：该项目主线是两轮自平衡和深度强化学习，不适合作为本课题主线。

## 7. 推荐项目结构

建议新建项目结构：

```text
slope-sim/
├── README.md
├── main.py
├── scene.py
├── robot.py
├── controller.py
├── logger.py
├── analysis.py
├── configs/
│   └── experiment.yaml
├── urdf/
│   └── diff_drive.urdf
├── experiments/
│   └── run_slope_sweep.py
└── results/
    ├── logs/
    └── figures/
```

模块职责：

| 文件 | 职责 |
|---|---|
| `main.py` | 程序入口，解析参数，启动仿真。 |
| `scene.py` | 创建地面、斜坡、草坪起伏场景。 |
| `robot.py` | 加载底盘 URDF，封装运动控制和状态读取。 |
| `controller.py` | 实现 Pure Pursuit / PID 等轨迹跟踪算法。 |
| `logger.py` | 记录真实轨迹、估计轨迹、控制输入和误差。 |
| `analysis.py` | 读取日志，生成轨迹图和误差图。 |
| `configs/` | 保存实验参数。 |
| `urdf/` | 保存机器人模型。 |
| `experiments/` | 批量实验脚本。 |
| `results/` | 保存实验输出。 |

## 8. 实现步骤

### 阶段 0：环境确认

- [x] 确认 Ubuntu 24.04 + Conda + Python 3.10 环境可用。
- [x] 确认 `p.connect(p.DIRECT)` 可用。
- [x] 确认 Ubuntu 本机 X11 下 `p.connect(p.GUI)` 可用；Dashboard 曲线页控制稳定性仍在修复。
- [x] 确认 SSH / Remote SSH 下默认使用 `DIRECT`，不误用 GUI。

### 阶段 1：最小 PyBullet 世界

- [x] 创建空世界。
- [x] 设置重力。
- [x] 加载平地。
- [x] 放置一个简单刚体。
- [x] 记录刚体位置。
- [x] 使用 Matplotlib 绘制位置变化。

### 阶段 2：差速底盘最小模型

- [x] 参考成熟仓库完成简化差速底盘 URDF。
- [x] 加载底盘。
- [x] 控制左轮和右轮速度。
- [x] 获取底盘位置和 yaw 角。
- [x] 记录底盘轨迹。
- [x] 输出 x-y 轨迹图。

### 阶段 3：斜坡场景

- [x] 创建单一斜坡和参考 5 度坡面。
- [x] 支持通过参数设置坡度，例如 `0/5/10/15/20` 度。
- [x] 验证底盘在不同坡度下是否能稳定运行。
- [x] 记录 z 轴高度、pitch、轨迹误差。

### 阶段 4：水库草坪起伏场景

第一版建议只做“简单坡度起伏”，不要做真实草地。

可选建模方式：

| 方法 | 难度 | 适合阶段 | 说明 |
|---|---:|---|---|
| 多个倾斜平面/长方体拼接 | 低 | 第一版 | 可控、简单，适合模拟几个缓坡和平台。 |
| Heightfield 高度场 | 中 | 第二版 | 能生成连续起伏地形，适合草坪/土坡近似。 |
| 三角网格 mesh 地形 | 中高 | 第二版后期 | 可从外部建模工具导入，但碰撞和尺度要调。 |
| 真实草地/湿滑/软土/轮胎陷入 | 高 | 暂不做 | PyBullet 可以近似摩擦，但不适合高真实度土壤-轮胎相互作用。 |

本课题建议路线：

- [x] 第一版使用单斜坡和分段大坝坡面。
- [ ] 第二版使用 heightfield 生成简单草坪起伏。
- [x] 用摩擦系数近似草地湿滑程度。
- [x] 第一版不模拟草叶、软土形变、轮胎陷入。

### 阶段 5：路径跟踪算法

- [ ] 从 PythonRobotics 学习 Pure Pursuit。
- [ ] 定义参考路径点。
- [ ] 输入当前位姿和参考路径。
- [ ] 输出线速度 `v` 和角速度 `w`。
- [ ] 将 `v/w` 转换为左右轮速度。
- [ ] 在 PyBullet 中执行。

差速轮转换关系：

```text
v_left  = v - w * wheel_base / 2
v_right = v + w * wheel_base / 2
```

### 阶段 6：噪声和误差模型

- [ ] 加入速度控制误差。
- [ ] 加入角速度控制误差。
- [ ] 加入位置观测噪声。
- [ ] 生成真实轨迹和估计轨迹。
- [ ] 计算终点误差、平均轨迹误差、最大横向误差、航向误差。

### 阶段 7：批量实验

- [x] 对不同坡度重复实验。
- [ ] 对不同噪声强度重复实验。
- [x] 保存每次实验的 CSV。
- [x] 输出坡度实验汇总表。
- [x] 输出误差曲线和轨迹对比图。

建议实验维度：

```text
slope_deg: 0, 5, 10, 15, 20
noise_level: none, low, medium
path_shape: straight, curve, s_curve
```

### 阶段 8：结果展示

- [x] GUI 展示机器人在斜坡和大坝场景中运动；曲线页控制稳定性仍在修复。
- [x] Matplotlib 展示真实轨迹、估计轨迹、参考路径和物理反馈。
- [ ] 表格展示不同坡度下的误差指标。
- [ ] 总结坡度和噪声对轨迹跟踪效果的影响。

## 9. 主要难点

### 9.1 轮地接触和摩擦调参

PyBullet 支持摩擦、碰撞和接触，但轮式底盘在斜坡上的真实表现受很多参数影响：

```text
lateralFriction
rollingFriction
spinningFriction
wheel mass
base mass
joint damping
maxForce
time step
solver iterations
```

第一版不要追求真实轮胎动力学，只要保证趋势合理、实验可重复。

### 9.2 斜坡上底盘姿态读取

平地上通常只关心：

```text
x, y, yaw
```

斜坡上还需要记录：

```text
z, roll, pitch
```

否则无法判断机器人是否因坡度产生姿态变化或打滑。

### 9.3 水库草坪起伏场景建模

简单起伏不难，高真实度很难。

建议分级：

1. 单斜坡：最容易，适合验证基础控制。
2. 分段坡面：用多个简单几何体拼出上坡、平台、下坡。
3. Heightfield：用高度矩阵生成连续起伏草坪。
4. 真实草地/软土：不作为本课题第一阶段目标。

需要注意：

- 起伏地形会增加轮子接触不稳定性。
- heightfield 的分辨率过高会降低性能。
- 坡度过陡或摩擦过低会导致滑移明显。
- 草地只能用摩擦和阻尼近似，不能真实模拟草叶和土壤变形。

### 9.4 GUI 和远程环境

PyBullet GUI 对图形环境敏感。已知风险：

```text
远程桌面 Wayland 下可能卡死或断连
SSH 默认没有 X server
OpenGL 驱动异常可能导致 GUI 无法启动
```

解决策略：

```text
本机 X11 用 GUI
SSH / 远程开发用 DIRECT
批量实验全部用 DIRECT
```

### 9.5 参考仓库不能直接拼接

参考仓库的作用是提供模块思路，不是直接合并代码。

推荐组合方式：

```text
pybullet_diffdrive -> 底盘最小实现
PythonRobotics -> 路径跟踪算法
PybulletRobotics -> 移动机器人学习参考
bullet3 -> 官方 PyBullet API 和 URDF 写法
Two-Wheel-Robot-DeepRL -> 斜坡/terrain 参考
```

## 10. 第一版验收标准

第一版完成时应具备：

- [x] 能在 PyBullet 中生成斜坡。
- [x] 能加载简化差速底盘和履带近似底盘。
- [x] 能通过 `v/w` 或左右轮速度控制底盘。
- [ ] 能运行一条预设路径。
- [x] 能记录真实轨迹。
- [ ] 能加入简单控制误差或定位噪声。
- [x] 能输出 CSV 日志。
- [x] 能画出参考路径、真实轨迹和估计轨迹。
- [x] 能计算终点误差、平均轨迹误差和航向误差。
- [x] 能在 `DIRECT` 模式下批量运行实验。
- [x] 能在 Ubuntu 本机 X11 下用 `GUI` 展示结果；Dashboard 曲线页断控问题是当前冻结项。

## 11. 建议推进顺序

最短可行路线：

```text
1. 跑通 PyBullet DIRECT 和 GUI
2. 跑通 pybullet_diffdrive
3. 改出自己的 main.py / robot.py / logger.py
4. 加单斜坡
5. 加轨迹记录和画图
6. 接 Pure Pursuit
7. 加噪声和误差分析
8. 做不同坡度批量实验
9. 再考虑草坪起伏 heightfield
```

优先保证每一步都有可运行结果，不要一开始同时做复杂地形、复杂底盘、复杂算法。
