# 轻量级斜坡机器人 3D 仿真器学习计划

本计划按“先简单、再复杂”的顺序推进。每一步完成后都要能实际运行，看到日志、图表或指标，再进入下一步。

## 阶段 0：确认环境能运行

目标：确认 Python、Conda、PyBullet 和 DIRECT 模式可用。

要做：

- 进入项目目录。
- 激活 `slope-sim` 环境。
- 检查 PyBullet 是否能以 DIRECT 模式连接。

运行：

```bash
conda activate slope-sim
python scripts/check_env.py
```

成功标志：

```text
python: 3.10.x
DIRECT connected: 0
```

如果终端显示 `(base)`，说明还在 Conda 基础环境，不是项目环境。需要执行 `conda activate slope-sim`。

## 阶段 1：跑通最小平地仿真

目标：先不考虑斜坡，只确认机器人和日志流程能跑通。

要做：

- 使用 `0` 度坡，相当于平地。
- 短时间运行，避免调试时等待太久。
- 检查是否生成 CSV 和轨迹图。

运行：

```bash
python main.py --slope-deg 0 --duration-sec 1 --mode direct
```

成功标志：

```text
log: results/logs/slope_0_*.csv
figure: results/figures/slope_0_trajectory.png
endpoint_error: ...
```

这一阶段完成后，说明项目入口、PyBullet DIRECT、机器人模型、日志和画图流程已经连通。

## 阶段 2：运行单一斜坡

目标：把平地改成一个简单斜坡，观察机器人姿态和轨迹输出。

要做：

- 设置一个温和坡度，例如 `5` 度。
- 确认日志里有 `z`、`roll`、`pitch`、`yaw` 字段。
- 确认轨迹图仍能生成。

运行：

```bash
python main.py --slope-deg 5 --duration-sec 2 --mode direct
```

成功标志：

- 生成 `results/logs/slope_5_*.csv`。
- 生成 `results/figures/slope_5_trajectory.png`。
- CSV 中能看到 `x, y, z, roll, pitch, yaw`。

## 阶段 3：比较不同坡度

目标：确认仿真器支持不同坡度输入，并能批量运行。

要做：

- 运行多个坡度。
- 生成汇总 CSV。
- 对比不同坡度下的误差指标。

运行：

```bash
python experiments/run_slope_sweep.py --slopes 0 5 10 15 20 --trials 1 --duration-sec 1
```

成功标志：

```text
summary: results/logs/slope_sweep_summary.csv
```

查看结果：

```bash
python - <<'PY'
import pandas as pd
print(pd.read_csv("results/logs/slope_sweep_summary.csv"))
PY
```

这一阶段完成后，说明项目已经能做最基础的坡度对比实验。

## 阶段 4：分析单次日志

目标：学会从一次仿真日志生成轨迹图和误差指标。

要做：

- 从 `results/logs/` 中选择一个实际生成的 CSV。
- 用 `analysis.py` 重新分析。

运行示例：

```bash
LOG=$(ls -t results/logs/slope_5_*.csv | head -n 1)
python analysis.py --log "$LOG"
```

成功标志：

```text
figure: results/figures/...
endpoint_error: ...
mean_tracking_error: ...
max_tracking_error: ...
heading_error: ...
```

这一阶段完成后，说明仿真输出已经可以被后处理脚本读取和展示。

## 阶段 5：测试差速转向

目标：不只让机器人直行，还要让它按角速度转弯。

要做：

- 设置线速度和角速度。
- 观察轨迹图是否从直线变为曲线。

运行：

```bash
python main.py \
  --slope-deg 5 \
  --duration-sec 3 \
  --target-linear-velocity 0.4 \
  --target-angular-velocity 0.4 \
  --mode direct
```

成功标志：

- 轨迹图中的 `actual` 不再是直线。
- CSV 中 `angular_velocity` 不为 `0`。

这一阶段完成后，说明差速底盘的 `v/w` 控制输入已经可以影响轨迹形状。

## 阶段 6：接入路径跟踪算法

目标：从“固定速度命令”升级为“跟踪给定路径”。

建议实现：

- 新增路径定义，例如直线、圆弧、S 形路径。
- 接入 Pure Pursuit。
- 输入当前位姿和参考路径。
- 输出线速度 `v` 和角速度 `w`。

建议运行命令：

```bash
python main.py --slope-deg 5 --duration-sec 5 --path-shape s_curve --controller pure_pursuit
```

成功标志：

- 机器人能沿参考路径运动。
- 图中同时显示参考路径、真实轨迹和估计轨迹。
- `mean_tracking_error` 能反映路径跟踪效果。

注意：当前代码还没有 `--path-shape` 和 `--controller` 参数，这一阶段需要先扩展代码后再运行。

## 阶段 7：加入噪声和估计轨迹

目标：让仿真更接近真实实验，区分真实轨迹和估计轨迹。

建议实现：

- 控制噪声：让实际执行速度和命令速度略有偏差。
- 观测噪声：让估计位置带有随机误差。
- 日志中保留真实轨迹和估计轨迹。

建议运行命令：

```bash
python main.py --slope-deg 10 --duration-sec 5 --noise-level low
```

成功标志：

- 轨迹图中 `actual` 和 `estimated` 不完全重合。
- 汇总指标能反映噪声对误差的影响。

注意：当前代码还没有 `--noise-level` 参数，这一阶段需要先扩展配置和日志逻辑。

## 阶段 8：扩展起伏地形

目标：从单一斜坡扩展到更接近草坪起伏的地形。

建议顺序：

1. 单斜坡。
2. 分段坡面。
3. Heightfield 连续起伏地形。

建议运行命令：

```bash
python main.py --terrain heightfield --duration-sec 5
```

成功标志：

- 场景不再只是单一平面。
- 机器人运动时 `z` 和 `pitch` 会随地形变化。
- 仿真仍能生成日志和轨迹图。

注意：当前代码还没有 `--terrain` 参数，这一阶段属于后续扩展。

## 阶段 9：形成完整实验

目标：把坡度、路径、噪声和地形组合成一组可重复实验。

建议实验维度：

```text
slope_deg: 0, 5, 10, 15, 20
path_shape: straight, curve, s_curve
noise_level: none, low, medium
terrain: slope, segmented, heightfield
```

建议运行命令：

```bash
python experiments/run_slope_sweep.py --slopes 0 5 10 15 20 --trials 5
```

成功标志：

- 每次实验都有 CSV 日志。
- 每组实验都有误差指标。
- 能输出轨迹图和汇总表。
- 能比较坡度、噪声、路径形状对轨迹跟踪效果的影响。

## 阶段 10：最终仿真目标

最终目标不是做通用高精度物理引擎，而是完成一个面向斜坡移动机器人的轻量级实验平台。

完成时应具备：

- 能生成斜坡或简单起伏地形。
- 能加载差速移动底盘。
- 能运行导航或轨迹跟踪算法。
- 能记录真实轨迹、估计轨迹和参考路径。
- 能计算终点误差、平均轨迹误差、最大轨迹误差和航向误差。
- 能批量运行实验。
- 能生成图表和汇总表。

推荐始终保持一个原则：

```text
每新增一个能力，都要先保证它能被命令行实际运行和验证。
```
