# PyBullet 差速底盘斜坡仿真

这是一个轻量级 3D 仿真项目，用 PyBullet 模拟差速移动底盘在斜坡上的运动。当前版本重点不是高精度物理建模，而是先跑通一套可运行、可记录、可分析的实验流程。

项目现在可以做到：

- 创建不同坡度的斜坡场景。
- 加载简化差速轮底盘模型和履带近似模型。
- 在 DIRECT 模式下运行仿真。
- 输出 CSV 轨迹日志和 PNG 轨迹图。
- 批量测试多个坡度，并汇总误差指标。
- 在物理模式下记录轮/履带速度、速度传感、加速度、接触力、摩擦力、地形法向和带符号打滑估计。

## 进入环境

如果终端前面显示的是 `(base)`，只表示你在 Conda 的基础环境里，还没有进入本项目环境。

本项目推荐进入 `slope-sim` 环境：

```bash
conda activate slope-sim
```

进入后终端前缀应类似：

```text
(slope-sim)
```

如果需要按项目配置补齐依赖：

```bash
mamba env update -n slope-sim -f environment.yml
```

## 检查环境

```bash
python scripts/check_env.py
```

看到类似下面的信息，说明基础环境可用：

```text
python: 3.10.x
pybullet_api_version: ...
DIRECT connected: 0
```

在 SSH、Remote SSH 或没有桌面的终端中，请使用 DIRECT 模式。只有在 Ubuntu 本机 X11 桌面下，才建议使用 `--gui`。

## 运行一次仿真

```bash
python main.py --slope-deg 5 --duration-sec 5 --mode direct
```

运行后会输出日志和图像路径，例如：

```text
log: results/logs/slope_5_*.csv
figure: results/figures/slope_5_trajectory.png
endpoint_error: ...
mean_tracking_error: ...
```

如果在本机 X11 桌面，可以打开 GUI：

```bash
python main.py --gui --slope-deg 5
```

## 阶段 1：平地演示

运行平地差速车：

```bash
python main.py --config configs/flat_demo.yaml --duration-sec 1 --mode direct
```

如果在 Ubuntu 本机 X11 桌面，可以打开 PyBullet 窗口，用方向键手动控制。手动模式默认不会按配置里的 `duration_sec` 自动退出：

```bash
python main.py --config configs/flat_demo.yaml --gui --manual
```

坡面姿态和履带打滑实验建议使用履带近似模型：

```bash
python main.py --slope-deg 5 --duration-sec 3 --mode direct --drive-model physics --robot-model tracked_proxy --wheel-radius 0.08
```

阶段 3 的完整反馈 build 使用参考 5 度坡面、履带代理和显式摩擦参数：

```bash
python main.py --config configs/step3_feedback.yaml --mode direct
```

如果在 Ubuntu 本机 X11 桌面，可以打开同一套配置手动测试：

```bash
python main.py --config configs/step3_feedback.yaml --gui --manual
```

手动控制说明：

- 上/下方向键：前进 / 后退。
- 左/右方向键：左转 / 右转。
- 空格：停车。
- `q` 或 `Esc`：退出。
- PyBullet 窗口里的滑条可以调整最大线速度和最大角速度。
- 实时 Dashboard 会按位姿、速度、速度传感、接触/打滑、地形/摩擦、传感器/命令分组显示；低速急停时打滑率会标记为“低速”，避免把接近 0 的分母放大成异常尖峰。
- 如果需要固定运行 60 秒后自动退出，可以显式加 `--duration-sec 60`。

打滑和接触指标说明：

- 签名打滑率是带正负号的估计值；负数表示驱动表面速度低于该侧车体局部速度。
- 有效接触点是 PyBullet 求解器里法向力大于阈值的接触点，不等于真实履带接触面积。
- `tracked_proxy` 是多滚轮 + 各向异性摩擦的履带近似模型，适合观察趋势，不是连续柔性履带的真实物理模型。

## 批量测试坡度

```bash
python experiments/run_slope_sweep.py --slopes 0 5 10 15 20 --trials 1
```

默认会生成汇总文件：

```text
results/logs/slope_sweep_summary.csv
```

## 分析日志

选择最新生成的 `slope_5` 日志并分析：

```bash
LOG=$(ls -t results/logs/slope_5_*.csv | head -n 1)
python analysis.py --log "$LOG"
```

分析结果会输出误差指标，并在 `results/figures/` 下生成轨迹图、打滑曲线图和接触/摩擦力曲线图。

## 常用目录

```text
slope_sim/                 核心仿真代码
configs/experiment.yaml    默认实验参数
configs/flat_demo.yaml     阶段 1 平地演示参数
configs/step3_feedback.yaml 阶段 3 摩擦/接触/打滑反馈参数
urdf/diff_drive.urdf       简化差速底盘模型
urdf/terrain/              参考地形 URDF
experiments/               批量实验脚本
results/logs/              CSV 日志
results/figures/           轨迹图
references/                参考仓库清单
tests/                     测试
```

## 参考仓库

同步参考仓库：

```bash
bash scripts/sync_references.sh
```

参考仓库会下载到：

```text
references/repos/
```

这些仓库主要作为学习和对照使用；当前仅把 Two-Wheel-Robot-DeepRL 的 5 度坡面按来源标注后整理成项目内测试地形。

## 下一步学习路线

更详细的由浅入深实现路线见：

```text
LEARNING_PLAN.md
```

每个文件的作用和参数位置见：

```text
ARCHITECTURE.md
```
