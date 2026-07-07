# 项目结构与参数说明

这份文档说明当前项目里每个主要文件的作用，以及常用参数在哪里修改。

## 当前进度

当前已经完成：

- 阶段 0：环境检查，PyBullet DIRECT 模式可用。
- 阶段 1：差速车可以在平地中运行，并能生成 CSV 日志和轨迹图。
- 阶段 1 扩展：本机 X11 桌面下可用 PyBullet GUI 手动控制差速车。

还未完成：

- Pure Pursuit 路径跟踪。
- 噪声模型。
- Heightfield 起伏地形。
- 更真实的轮地接触动力学验证。

## 入口文件

### `main.py`

主入口文件。

常用命令：

```bash
python main.py --config configs/flat_demo.yaml --mode direct
python main.py --config configs/flat_demo.yaml --gui --manual
python main.py --slope-deg 5 --duration-sec 5 --mode direct
```

主要作用：

- 读取命令行参数。
- 加载 YAML 配置。
- 根据 `--manual` 决定运行自动仿真还是手动控制。
- 手动控制默认一直运行到按 `q` 或 `Esc`，只有显式传 `--duration-sec` 才会按时长结束。

### `analysis.py`

读取一次仿真生成的 CSV 日志，重新计算误差指标并生成轨迹图。

示例：

```bash
LOG=$(ls -t results/logs/slope_5_*.csv | head -n 1)
python analysis.py --log "$LOG"
```

### `experiments/run_slope_sweep.py`

批量运行多个坡度实验，并生成汇总表。

示例：

```bash
python experiments/run_slope_sweep.py --slopes 0 5 10 15 20 --trials 1
```

## 核心代码

### `slope_sim/config.py`

配置定义和加载逻辑。

主要参数包括：

- `mode`：`direct` 或 `gui`。
- `slope_deg`：坡度角，单位是度。
- `duration_sec`：仿真时长。
- `time_step`：仿真步长。
- `wheel_base`：左右轮间距。
- `wheel_radius`：轮子半径。
- `target_linear_velocity`：目标线速度。
- `target_angular_velocity`：目标角速度。
- `log_dir`：日志输出目录。
- `figure_dir`：图像输出目录。

### `slope_sim/scene.py`

创建 PyBullet 场景。

当前做法：

- 用一个静态长方体表示地面。
- `slope_deg = 0` 时是平地。
- `slope_deg > 0` 时是斜坡。

### `slope_sim/robot.py`

加载差速车 URDF，并封装机器人控制和状态读取。

当前重要逻辑：

- 从 URDF 中找到左右轮关节。
- 把车体线速度和角速度转换成左右轮目标速度。
- 用差速车运动学更新机器人位姿。

注意：当前阶段为了保证实验稳定可重复，轨迹推进主要使用运动学更新；后续可以扩展成更依赖 PyBullet 轮地接触的动力学模式。

### `slope_sim/controller.py`

差速车运动学转换。

核心关系：

```text
v_left  = v - w * wheel_base / 2
v_right = v + w * wheel_base / 2
wheel_speed = wheel_linear_speed / wheel_radius
```

### `slope_sim/simulation.py`

自动仿真主流程。

主要做：

- 连接 PyBullet。
- 创建场景。
- 加载机器人。
- 执行固定速度命令。
- 写 CSV 日志。
- 生成轨迹图和误差指标。

### `slope_sim/manual_control.py`

把键盘输入转换成速度命令。

按键含义：

- 上/下方向键：线速度正负。
- 左/右方向键：角速度正负。
- 空格：停车。
- `q` 或 `Esc`：退出。

### `slope_sim/manual_demo.py`

PyBullet GUI 手动控制演示。

主要做：

- 打开 PyBullet GUI。
- 创建平地或斜坡场景。
- 加载差速车。
- 用方向键控制车体速度。
- 用 PyBullet debug sliders 调整最大线速度和最大角速度。
- 同样输出 CSV 和轨迹图。

### `slope_sim/logger.py`

写 CSV 日志。

记录字段包括：

```text
t, x, y, z, roll, pitch, yaw,
linear_velocity, angular_velocity,
reference_x, reference_y,
estimated_x, estimated_y
```

### `slope_sim/metrics.py`

计算误差指标。

当前包括：

- 终点误差。
- 平均轨迹误差。
- 最大轨迹误差。
- 航向误差。

## 配置文件

### `configs/experiment.yaml`

通用实验配置，默认是 5 度斜坡。

适合用于自动斜坡实验。

### `configs/flat_demo.yaml`

阶段 1 平地演示配置，坡度固定为 0 度。

适合用于：

```bash
python main.py --config configs/flat_demo.yaml --mode direct
python main.py --config configs/flat_demo.yaml --gui --manual
```

## 机器人模型

### `urdf/diff_drive.urdf`

简化差速底盘模型。

包含：

- `base_link`：车体。
- `left_wheel`：左轮。
- `right_wheel`：右轮。
- `caster`：辅助支撑轮。
- `left_wheel_joint`：左轮连续旋转关节。
- `right_wheel_joint`：右轮连续旋转关节。

修改车体尺寸、轮子半径、轮子位置时，主要改这个文件。

## 输出目录

### `results/logs/`

保存 CSV 日志。

### `results/figures/`

保存轨迹图。

## 测试

测试目录是 `tests/`。

运行：

```bash
python -m pytest -q
```

当前测试覆盖：

- 配置加载。
- 差速轮速转换。
- 日志字段。
- 误差指标。
- 自动仿真 smoke test。
- 手动控制按键映射。
- 平地演示配置。
