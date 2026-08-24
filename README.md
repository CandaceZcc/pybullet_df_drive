# PyBullet 3D 移动机器人仿真平台

## 摘要

本项目以 PyBullet 建立轮式移动机器人的可交互物理世界，提供四种底盘、三类地形、运行期场景切换、SBUS 遥控器和企业接口。桌面会话由 PyBullet 主窗口承担物理观察和键盘输入，Qt Dashboard 承担状态显示、配置、采集和会话控制。正式版采用五 topic 的 eCAL v2 实时协议，并以单个 C++ Command 进程作为 `/sim/wheel/command` 的唯一发布者。

阶段一至四的实现范围、历史证据和边界记录在 [阶段四交付报告](docs/阶段四交付报告.md)。协议、命令权和跨语言 golden 检查见 [阶段四协议与命令权说明](docs/阶段四协议与命令权说明.md)。

## 系统结构

```text
                         local authenticated Unix socket
+----------------------+  target lease  +--------------------------+
| PyBullet main window |--------------->| C++ Command              |
| physics / keyboard   |                | sole wheel publisher     |
+----------+-----------+                +------------+-------------+
           |                                         |
           | shared Python process                   | eCAL v2
           v                                         v
+----------------------+       +-----------------------------------+
| Qt Dashboard         |<----->| Python simulator runtime          |
| telemetry / scene /  |       | world, sensors, eCAL subscribers  |
| capture controls     |       +-----------+-------------+---------+
+----------------------+                   |             |
                                             |             |
                  +--------------------------+             +--------------------+
                  v                                                     v
      /sim/wheel/state, /sim/rtk/state,                       C++ Recorder
      /sim/imu/attitude, /sim/lidar/points                    session.mcap
                                                                            |
                                                                            v
                                                                  C++ Export
                                                            PCD / PLY / LVX2
```

| 层次 | 责任 | 主要产物 |
| --- | --- | --- |
| 物理世界 | URDF、关节驱动、地形、障碍物、传感器安装位 | PyBullet body 与逻辑场景 |
| 交互界面 | 驾驶输入、车型/地形切换、遥测与采集控制 | 两个桌面窗口 |
| v2 实时链路 | 命令租约、状态、中心 MID-360、三点 RTK、IMU | 五个 eCAL topic |
| 数据链路 | 完整 topic 会话记录与离线点云导出 | MCAP、PCD、PLY、LVX2 |

正式 v2 topic：`/sim/wheel/command`、`/sim/wheel/state`、`/sim/lidar/points`、`/sim/rtk/state`、`/sim/imu/attitude`。轮命令和轮状态目标频率为 100 Hz；LiDAR、RTK、IMU 为 10 Hz。命令发布权由 C++ Command 独占，Dashboard 与键盘仅续租本机 socket 中的目标值。

更细的进程、数据与退出关系见 [ARCHITECTURE_MAP.md](ARCHITECTURE_MAP.md)。

## 模型与范围

| 类别 | 可选项 |
| --- | --- |
| 车辆 | `df_front`、`df_mid`、`df_back`、`active_steering_4wd` |
| 地形 | `flat`、`slope`、`golf_heightfield` |
| 驱动 | PyBullet 关节物理模式 |
| 会话模式 | `local`、`ecal`、`auto`；正式 `runSim` 默认 `ecal` |

正 `slope_deg` 表示车辆沿世界 `+X` 从高位平地驶入下坡，再进入低位平地。`golf_heightfield` 由 `golf_seed` 和 `golf_relief` 决定，同一组合可重现。障碍物支持静态、移动和混合布局；自动导航、自动刹停和动态避障均不在当前阶段四范围。

## 部署

### 源码开发环境

桌面 GUI 需要 Ubuntu X11 或可用桌面会话；无显示环境使用 `--mode direct`。首次部署执行：

```bash
conda env create -f environment.yml
conda activate slope-sim
python scripts/generate_protos.py
python scripts/check_env.py
```

已有环境更新：

```bash
conda env update -n slope-sim -f environment.yml
conda run -n slope-sim python scripts/generate_protos.py
conda run -n slope-sim python scripts/check_env.py
```

该环境包含仿真运行、eCAL、协议生成和 pytest；Jupyter、SciPy 等离线分析依赖位于独立环境，避免日常 GUI 开发额外安装它们：

```bash
conda env create -f environment-analysis.yml
conda activate slope-sim-analysis
```

分析环境用于读取 CSV/MCAP 和 Notebook，不包含 PyBullet GUI、串口或 eCAL 运行时。正式 `.run` 发行仍使用冻结的 `packaging/python-environment.yml`，不受源码开发环境影响。

协议生成物位于 `slope_sim/interfaces/generated/`，由 `scripts/generate_protos.py` 统一生成。不要手工修改该目录。

### 阶段五源码人工验收

在最终 `.run` 构建前，当前源码可复用本机已验证的 eCAL 运行时进行人工验收。首次由开发者生成小型验收运行根后，在仓库根执行：

```bash
export SLOPE_SIM_V2_RUNTIME_ROOT="$PWD/build/stage5-acceptance-runtime"
./runSim --robot-model df_mid --terrain-model slope --slope-deg 8
```

该变量只指定当前源码会话使用的 `Command`、`ecal.yaml` 和 localtime 插件，不生成或替代发行安装包。未设置时，源码入口保持原有行为；已安装 release 仍直接使用自身运行根。

### `runSim.run` 安装

最终发布包为 Ubuntu 24.04 amd64 单文件安装器 `runSim.run`。安装过程需要联网下载锁定依赖，所有下载和内嵌文件均校验 SHA-256；Livox Viewer 2 使用 Livox 官方 Linux 2.6.0 包，并安装到 release 内的固定路径。Viewer 是 Livox 的专有二进制，本仓库不提交其 ZIP，安装器只保存官方 URL、版本与摘要。

```bash
chmod +x runSim.run
mkdir -p "$HOME/.local/bin"
./runSim.run \
  --install-root "$HOME/.local/share/runSim" \
  --command-dir "$HOME/.local/bin"

# Ubuntu 默认通常已包含此目录；若 command -v runSim 没有输出，再执行：
export PATH="$HOME/.local/bin:$PATH"
grep -qxF 'export PATH="$HOME/.local/bin:$PATH"' "$HOME/.zshrc" 2>/dev/null || \
  printf '%s\n' 'export PATH="$HOME/.local/bin:$PATH"' >> "$HOME/.zshrc"

runSim --version
runSim --help
runSim

# 可选真实 SBUS 遥控器：仅接受稳定 by-id 路径，并在启动前完成 20 帧资格检查
runSim \
  --rc-port /dev/serial/by-id/usb-FTDI_Quad_RS232-HS-if02-port0
```

当前修复版的 `runSim --version` 应输出 `runSim 5.0.1`。release 已内置 `pyserial`，不要再向系统 Python 或安装目录执行 `pip install`；同时启动器会在调用者没有显式覆盖时使用包内 Fontconfig 配置，避免激活后继续访问已删除的安装 staging。`--command-dir` 只会创建它管理的 `runSim` 软链接；如果目标已有普通文件或其他链接，安装会拒绝覆盖。安装器 manifest 记录构建时 Git commit。`--with-ros` 仅在需要 ROS 2 bridge/RViz2 时使用，Livox Viewer 2 本身已包含在普通安装中。

## 运行

### 本地双 GUI 会话

这条命令关闭企业接口，不依赖 release 的 C++ Command，适合先验收物理世界和 Dashboard：

```bash
conda run -n slope-sim python main.py \
  --gui --manual --no-interface --drive-model physics \
  --robot-model df_mid --terrain-model slope --slope-deg 8
```

启动后应出现两个窗口：

```text
+-------------------------------+--------------------------+
| PyBullet                      | Qt Dashboard             |
| 车辆、场地、障碍物、相机       | 遥测、仿真控制、车型地形、|
| 键盘: 上下左右 / 空格 / q Esc | 障碍物、采集等页面        |
+-------------------------------+--------------------------+
```

键盘操作：上/下前进后退，左/右转向，空格停车，`q` 或 `Esc` 退出。`active_steering_4wd` 需要同时按纵向和横向键才会转弯。Dashboard 中选择车型或地形后，必须点击对应“应用”按钮才会重建物理世界。

Dashboard 的折线图默认不创建；需要观察时，从标签栏右上角“图表（按需）”逐项勾选，取消勾选会同时释放该图的绘图对象和历史缓存。使用 SBUS 遥控器时，先将 CH6 拨到低位再拨到高位完成安全解锁，然后在控制源中选择“遥控器”；如果串口、帧或解锁状态未就绪，Dashboard 会自动退回“键盘”，避免两个输入源同时表现为不可用。

“LiDAR 点云”页提供“累计”和“当前帧”两种模式，也可直接点击“查看当前帧”。累计模式会在进入 OpenGL 前按时间顺序等步长采样，渲染输入最多 80,000 点，避免长时间驾驶时反复拼接百万级点阵拖慢 GUI；采集和导出仍使用完整原始帧，不受显示采样影响。

### 正式 release v2 会话

新 `.run` 安装完成后直接调用：

```bash
runSim

# 固定车型、地形和初始速度
runSim \
  --robot-model df_mid --terrain-model golf_heightfield --golf-seed 41 \
  --golf-relief medium --target-linear-velocity 2.0

# 启动后立即记录 90 秒，再导出同一会话的点云文件
runSim \
  --capture-duration-sec 90 --capture-output-dir /absolute/path/to/captures

# 阶段五控制链观测复验
runSim --no-dashboard --developer-diagnostics \
  --robot-model df_mid --terrain-model slope --slope-deg 8 \
  --target-linear-velocity 1.5
```

`runSim` 无参数等价于 GUI、手动、v2 实时和 `ecal` 模式。安装包中的 eCAL 配置与 localtime 插件会自动载入；显式设置 `ECAL_CONFIG_PATH`、`ECAL_DATA`、`ECAL_TIME_PLUGIN_PATH` 或相应 CLI 参数可覆盖默认值。

若需要同时观察控制链、Fontconfig/GLX 和遥控器状态，使用：

```bash
runSim \
  --developer-diagnostics \
  --target-linear-velocity 1.5 \
  --robot-model df_mid --terrain-model slope --slope-deg 8 \
  --rc-port /dev/serial/by-id/usb-FTDI_Quad_RS232-HS-if02-port0
```

只有上述 by-id 路径存在时才加入 `--rc-port`。USB 未接入时删除该参数，不能回退到 `/dev/ttyUSB*`；Dashboard 中先将 CH6 低→高完成解锁，再选择“遥控器”。

采集控制也可从 Dashboard 的“MID-360 采集”页操作。成功导出后，输出目录包含 `session.mcap`、`export/lidar.lvx2`、逐帧 PCD/PLY 和导出回执；默认目录为 `results/manual-mid360/`。

## 手工验收

### A. 现在即可执行：本地双 GUI

1. 执行上节的 `slope` 命令，确认 PyBullet 与 Dashboard 同时出现，窗口互不遮挡。
2. 在 PyBullet 窗口按上键约 3 秒。车辆应沿 `+X` 由高位平地进入下坡；空格后车辆停止。
3. 在 Dashboard 调整线速度、角速度，继续键盘驾驶，确认状态和曲线更新。
4. 选择 `df_front` 并点击“应用车型”；再选 `flat` 并点击“应用场地”。确认车辆和场地切换成功，界面无卡死。
5. 选择 `active_steering_4wd`，同时按上键和左/右键，确认其轮速及前轮转角在 Dashboard 更新。
6. 切换到障碍物页面，添加静态和移动障碍物；观察碰撞与移动轨迹。按 `q` 或 `Esc` 正常退出两个窗口。

该流程验收 GUI、物理驱动和场景切换。它不覆盖 C++ Command、eCAL topic、Recorder 或 Export。

### B. 新 release 构建后：完整 v2 GUI

1. 在已安装的新 release 中执行 `bin/runSim`，确认两个 GUI 同时启动，终端未出现 `BrokenPipeError` 或 Command 退出信息。
2. 用键盘短暂前进、转向和停车；确认 Dashboard 的 v2 状态页持续刷新。
3. 在“MID-360 采集”页选择 1 分钟，点击“启用采集”，观察采集状态变化；提前点击“结束采集”。
4. 确认输出目录存在 `session.mcap`、`export/lidar.lvx2`、PCD/PLY 与成功回执；点击“导入 Livox Viewer”，确认不再报告 launcher missing。
5. 在 LiDAR 页切换累计显示并点击“查看当前帧”，确认 GUI 连续响应且采集文件点数不因显示限流而减少。
6. 使用 `runSim --version` 和 `runSim --help` 复核安装入口及参数合同。

完整 v2 验收以新建、已安装的 release 为准。安装器的 manifest 必须记录包含修复的 commit，不能以未提交工作树生成最终包。

## 自动验证

```bash
# 直接物理回归
conda run -n slope-sim python main.py --mode direct --interface-mode local \
  --robot-model df_back --terrain-model flat --drive-model physics --duration-sec 1

# 启动器与文档合同
conda run -n slope-sim python -m pytest -q \
  tests/unit/test_run_sim_launcher.py \
  tests/stage4/test_delivery_report_contract.py

# 默认非 eCAL 回归
conda run -n slope-sim python -m pytest -q -m "not ecal and not stage4_artifact"
```

真实 eCAL、GUI 和发布制品门禁依赖对应的桌面、eCAL socket 与 release 环境。它们的历史结果和运行条件记录在交付报告。

### 命令行测试编写规则

- 每条验收命令必须能从仓库根目录直接复制执行；明确 Conda 环境、必要环境变量、输入文件和输出目录，不能依赖未说明的当前 shell 状态。
- 自动测试必须有界：使用固定帧数、`--duration-sec`、测试超时或可预测的进程退出条件；日志和采集输出写入一个明确目录，不生成无界或时间戳构建树。
- 在命令旁写清预期退出码和可机器判断的结果，例如帧率下限、100 ms watchdog、文件 SHA、JSON 字段或“不出现周期归零”；不能只写“观察是否正常”。
- GUI、真实 eCAL、USB 串口、ROS 和 Livox Viewer 属于外部门禁，必须单独标注环境条件。串口只使用 `/dev/serial/by-id/...`，不得回退到易漂移的 `/dev/ttyUSB*`。
- 实质性故障修复按聚焦测试 RED→最小实现 GREEN→直接相关回归执行；正式里程碑最后只运行一次完整回归。命令、测试名、结果和跳过原因都应保留在交付记录中。
- 安装包验收必须针对新安装的 release，至少检查 `runSim --version`、`runSim --help`、manifest doctor、正式 v2 启动与正常退出；源码工作树通过不等于安装包通过。

## 目录

```text
main.py                         Python 入口
runSim                          release / 开发启动器
slope_sim/                      物理世界、GUI、接口 runtime
cpp/client/                     C++ Command、Recorder、Export
configs/                        实验配置
scripts/                        协议生成、验证、安装器构建
tests/                          单元、集成、阶段四回归
docs/                           交付报告和操作说明
```
