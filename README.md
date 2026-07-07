# PyBullet 差速底盘斜坡仿真

这是一个轻量级 3D 仿真项目，用 PyBullet 模拟差速移动底盘在斜坡上的运动。当前版本重点不是高精度物理建模，而是先跑通一套可运行、可记录、可分析的实验流程。

项目现在可以做到：

- 创建不同坡度的斜坡场景。
- 加载简化差速轮底盘模型。
- 在 DIRECT 模式下运行仿真。
- 输出 CSV 轨迹日志和 PNG 轨迹图。
- 批量测试多个坡度，并汇总误差指标。

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

分析结果会输出误差指标，并在 `results/figures/` 下生成轨迹图。

## 常用目录

```text
slope_sim/                 核心仿真代码
configs/experiment.yaml    默认实验参数
urdf/diff_drive.urdf       简化差速底盘模型
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

这些仓库只作为学习和对照使用，不直接混入主项目代码。

## 下一步学习路线

更详细的由浅入深实现路线见：

```text
LEARNING_PLAN.md
```
