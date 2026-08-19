# PyBullet 3D 移动机器人仿真平台

当前仓库按 `3d仿真平台需求规格.md` 分四阶段交付。阶段一、阶段二已由用户验收；阶段三已接入真实 eCAL、企业传感器、版本化场景文件和企业 Dashboard。阶段四 A-E 已完成 v2 协议与真实 Phase-0、中心 LiDAR/三点 RTK runtime、C++ Command/Subscriber/Recorder、Replay/Export、可选 ROS/RViz2/Livox Viewer 2 以及联网 `.run` 安装验收。MID-360 Golf 采集与三维回放的 v2 acceptance 已通过；它使用独立离线入口，不改变实时 LiDAR 合同。实时命令、证据路径和残余风险见 [`docs/阶段四交付报告.md`](docs/阶段四交付报告.md)。

各阶段证据见 [`docs/阶段一交付报告.md`](docs/阶段一交付报告.md)、[`docs/阶段二交付报告.md`](docs/阶段二交付报告.md) 和 [`docs/阶段三交付报告.md`](docs/阶段三交付报告.md)。

阶段四 A 的协议边界、命令断线重连规则和跨语言 golden 检查见 [`docs/阶段四协议与命令权说明.md`](docs/阶段四协议与命令权说明.md)。

## MID-360 Golf 映射与回放

固定 `df_mid/golf_heightfield/seed=41/medium` 场景的正式入口为：

```bash
conda run -n slope-sim python scripts/run_mid360_golf_mapping.py \
  --recorder <slope_sim_stage4_recorder> \
  --exporter <slope_sim_stage4_export> \
  --output-dir <new-empty-result-dir> --direct
```

它串行启动正式五 topic 会话、C++ Recorder、路线/地形/运动 acceptance 和独立双 OpenGL
回放。传入 `--exporter` 后，只有 acceptance 成功的同一会话才额外生成
`<output-dir>/export/lidar.lvx2`、逐帧 `.pcd/.ply` 和 `<output-dir>/export.json`；这些是
Livox Viewer 2 与 CloudCompare/MeshLab 的输入，普通 `runSim` 不生成它们。最终同会话证据位于 `results/mid360-golf-mapping-release-qa-v3/`：其 acceptance 与
`gui-qa-retry5/qa.json` 均绑定 simulation session
`47a6ff48245c44d1aad22f7a2a1dce57`，覆盖双视图非空像素、播放/暂停、逐帧、定位、倍率、
回退重建和正常关闭。历史 v1 GUI QA 与失败目录继续保留，不能替代该同会话证据。

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

首次创建或更新环境：

```bash
conda env create -f environment.yml
conda activate slope-sim
python scripts/generate_protos.py
python scripts/check_env.py
```

已有环境可用 `conda env update -n slope-sim -f environment.yml` 同步。项目固定 Protobuf `6.33.6`、`grpcio-tools 1.76.0` 和 Eclipse eCAL Python `6.1.1`；重新生成代码时必须同时生成企业协议和内部日志协议，不能手改 `slope_sim/interfaces/generated/`。

SSH 或无桌面环境请使用 `direct`；只有存在 Ubuntu X11/桌面会话时才能使用 `--gui`。

## 正式 runSim v2（release）

安装后的正式入口为 `bin/runSim`（源码树中也可用根目录的 `./runSim` 做开发验证）。无参数
会启动一个 PyBullet GUI、Dashboard、Python v2 Simulator 和唯一的 C++ `Command`；Dashboard 与
键盘只通过本机认证 socket 续租目标，绝不直接发布 `/sim/wheel/command`。窗口失焦、socket
关闭、目标超时、Command 退出或 runSim 退出都会归零。正式模式只使用五个 v2 topic：
`/sim/wheel/command`、`/sim/wheel/state`、`/sim/lidar/points`、`/sim/rtk/state` 和
`/sim/imu/attitude`。

```bash
# 默认 GUI/Dashboard v2 会话
runSim

# 指定实验配置；命令行参数覆盖 YAML，YAML 覆盖安全默认值
runSim --config /absolute/path/to/experiment.yaml \
  --robot-model df_mid --terrain-model golf_heightfield --golf-seed 41 \
  --target-linear-velocity 2.0

# 启动时采集 1.5 分钟并导出到指定目录；60、90、180 秒分别对应 1、1.5、3 分钟
runSim --capture-duration-sec 90 --capture-output-dir /absolute/path/to/captures

# 启动独立 ROS Bridge/RViz2 实时点云，或指定本机 Livox Viewer 2 后从 Dashboard 导入最新 LVX2
runSim --open-ros-rviz --viewer-root /absolute/path/to/LivoxViewer2

# 启动器与参数合同自检
runSim --version
runSim --help
```

release 自带 `etc/ecal/ecal.yaml` 与 localtime 插件时，启动器会自动设置所需 eCAL 环境；显式
`ECAL_CONFIG_PATH`、`ECAL_DATA`、`ECAL_TIME_PLUGIN_PATH` 或相应 CLI 参数的优先级更高。预检若
发现 eCAL、descriptor 或插件缺失，会在终端和 Dashboard 说明原因，不会回退到 local/v1。

在 Dashboard 的“MID-360 采集”中选择 `1 分钟`、`1.5 分钟`、`3 分钟`或`不限`后点击“启用采集”；也可
通过 `--capture-duration-sec 60|90|180` 在启动时直接开始限时采集。`--capture-output-dir` 覆盖默认输出目录。
“结束采集”会在完整五 topic 边界停止 C++ Recorder，成功后运行同一 release 的 C++ Export，生成
`session.mcap`、逐帧 PCD/PLY、`export/lidar.lvx2` 与导出回执。默认位置为
`results/manual-mid360/capture-YYYYMMDD-HHMMSS/`；同秒冲突会追加稳定序号。Dashboard 同时保存
`results/manual-mid360/last-successful-lvx2.json`，只记录最近一次成功导出的绝对 LVX2 路径；随后
点击“导入 Livox Viewer”使用该已验证路径。若 Viewer 不在常见位置，以 `--viewer-root` 指向其安装根目录。
导出 sidecar 的 `max_observed_range_m` 记录本次 MCAP 实际最远点。旧的 `runSim --lidar` 与
`--lidar-debug-draw` 入口已移除。
实时中心 MID-360 的固定量程为 45 m；离线重建、PCD/PLY/LVX2 回放覆盖 60 m，二者均保持既有
射线数、10 Hz 节拍与点云质量。

安装了 ROS/RViz 组件的 release 会在“v2 eCAL”页提供“打开实时点云”；`--open-ros-rviz` 可在启动时
执行同一操作。它只启动该 release 的 ROS Bridge 和固定 `stage4_live.rviz`，关闭或重启只影响这两个
显示进程，不会中断 Simulator、Command 或 Recorder；若缺少 `rviz2`、Bridge 或 profile，状态栏会给出恢复提示。

## 企业接口与场景文件

`--interface-mode` 有三个严格模式：

- `ecal`：必须创建真实 Eclipse eCAL transport，导入、初始化或资源创建失败时直接报错。
- `local`：显式使用进程内 transport，适合键盘驾驶、DIRECT 调试和不启动 eCAL peer 的测试。
- `auto`：优先创建 eCAL；只有 eCAL 绑定不可用时才降级到 local，并在 Dashboard 显示降级原因。

以下六个固定话题属于旧版兼容接口：`/sim/wheel/command`、`/sim/wheel/state`、`/sim/lidar/front/points`、`/sim/lidar/rear/points`、`/sim/rtk/state` 和 `/sim/imu/attitude`。轮子命令/状态目标频率为 100 Hz，前后点云、RTK 和 IMU 为 10 Hz；正式 `runSim` 使用上节的五 topic v2 合同，绝不同时发布旧前后 LiDAR topic。

导出包含车型、场地、障碍物和传感器安装位的版本化场景：

```bash
python main.py --mode direct --interface-mode local --duration-sec 1 --scene-out results/stage3-scene.yaml
```

再次加载场景并导出运行后的逻辑状态：

```bash
python main.py --mode direct --interface-mode local --duration-sec 1 --scene-in results/stage3-scene.yaml --scene-out results/stage3-scene-after.yaml
```

## DIRECT 快速验证

运行一次默认平面实验：

```bash
python main.py --mode direct --interface-mode local --robot-model df_back --terrain-model flat --drive-model physics --duration-sec 1
```

运行阶段一四车型 × 三场地矩阵：

```bash
python scripts/verify_stage1_matrix.py
```

运行阶段二障碍物 DIRECT 验收：

```bash
python scripts/verify_stage2_obstacles.py
```

运行阶段三 21 项 DIRECT/性能验收和真实 eCAL 六话题门禁：

```bash
python scripts/verify_stage3_interfaces.py
python scripts/verify_ecal_roundtrip.py --runtime simulation --robot-model active_steering_4wd --warmup-sec 1 --duration-sec 5
python scripts/verify_ecal_roundtrip.py --runtime simulation --robot-model df_back --warmup-sec 1 --duration-sec 5
```

后两条门禁分别覆盖主动转向 `4+2` 和代表性差速 `2+0` 命令。每条都在同一个 5 秒生产会话内同时启用 20 个障碍物、真实 eCAL 六话题和接口日志；除频率、断线代际与零丢帧外，还要求日志接受量达到名义消息量的 90%、终态 pending 为 0、没有持续增长或 writer 停滞，且 `sim/wall` 保持在 `0.98..1.02`。2026-07-29 的获授权执行在 discovery 前被当前 Codex 沙箱的 socket 权限阻断，原始证据位于 `/tmp/pybullet-df-postfix-ecal-gate.CBZyMMKJ`，不构成 post-fix 通过或性能失败。再次执行必须由用户重新授权并使用允许 eCAL socket 的环境，两种车型严格串行，仍不得自动重跑或降低 95 Hz 门槛。

生产 session 初始化和周期状态刷新都先调用 `poll_peer_state()`，再读取对应快照。eCAL discovery 使用独立串行门和递增 revision，迟到的旧观察不能覆盖新状态；关闭时先等待在途 discovery 返回，再释放 subscriber/publisher 引用并 finalize participant，避免 count API 访问已销毁资源。

自动实验、GUI 手动入口和独立 eCAL 仿真进程共享 `RuntimeObservationCadence`：native discovery 与接口组合快照按 50 ms 墙钟周期运行，首次进入、暂停恢复和 world 重建后立即观测一次；慢 poll 从完成墙钟重建期限，迟到不突发追赶。100 ms 命令超时判断、控制下发和 PyBullet 物理步仍逐帧执行，并使用该帧的新墙钟。

接口回调统一标记为 `publish/receive/logger` 三类线程局部上下文；`prepare/rebind/commit/abort/fault/close` 在回调或当前 lifecycle owner 同线程重入时立即拒绝，不进入互相等待。wheel-only rebind 在 safe-stop 前失败时恢复原准入；safe-stop 后提交失败只恢复旧 robot/model/mailbox/subscription 引用，并进入 `faulted`、保持 token 失活，禁止已停车旧车被旧命令重新激活。

前后 LiDAR 每帧各使用 2880 条射线，后雷达相对前雷达错相 50 ms。生产 runtime 在每个扫描 deadline 冻结一次雷达安装位姿，并用一次 `rayTestBatch` 生成该发布时刻的原子点云；Dashboard 同批次额外冻结 `base_link` 位姿生成俯视投影。生产后端只回传紧凑 indexed hit，并使用预验证批量逆变换减少 Python 开销；公开扫描 API 不提供跨物理帧的 rolling/incremental 入口，不拼接多个世界状态，也不模拟运动畸变。DIRECT/headless session 不构造 Dashboard 俯视副本。真实 eCAL 循环使用共享绝对期限节拍器，超期帧只调用 `sleep(0)` 让出执行权，并在循环期间暂停 cyclic GC、退出时恢复原状态。

运行自动测试：

```bash
python -m pytest -q -m "not ecal and not stage4_artifact"
```

真实 eCAL 和依赖指定阶段四构建制品的测试不包含在默认回归中，必须按对应独立门禁的环境变量、制品和授权约束执行。

当前默认回归结果以当次命令输出为准；各阶段的历史数值和外部门禁证据仅保留在对应交付报告。

## 自动 GUI 门禁

以下门禁必须严格串行运行，一次只启动一个 PyBullet GUI/Xvfb。默认键盘路径会实际点击页签栏左右滚动按钮、检查 15 个默认页，并在遍历期间持续驾驶：

```bash
DISPLAY=:1 XAUTHORITY=/home/cancade/.Xauthority conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --duration-sec 4
xvfb-run -a -s "-screen 0 1366x768x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --expected-available-size 1366x768 --duration-sec 4
xvfb-run -a -s "-screen 0 1920x1080x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --expected-available-size 1920x1080 --duration-sec 4
xvfb-run -a -s "-screen 0 2560x1440x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --expected-available-size 2560x1440 --duration-sec 4
```

正式路径消费 Dashboard schema v4 报告；除 DPR、viewport、文字和控件矩形外，还以独立 `1:1` oracle 核对上下区真实 `50:50`、根布局 `8px` 边距与 `6px` 间距、两个 pane 共同铺满剩余高度。图表 canvas 至少覆盖 page 宽 85%、高 70%，`axes_rect` 至少覆盖 canvas 宽 60%、高 50%。正式全图表门禁要求 Dashboard client 逻辑高度至少 600px；304/320px 仅验证 compact 布局可滚动、无重叠，不冒充正式门禁。15 页会遍历两轮，第二轮只能读取 JSONL 行游标之后的新 occurrence；图表的 `rendered_data_revision` 必须前进，而 `tabs/controls/page/canvas/axes` 五个矩形必须保持不变。Dashboard 不再提供方向按钮，`--input-method` 只接受 `key`。

## GUI 人工观察

GUI 手动模式不会按配置中的 `duration_sec` 自动退出。使用 `q`、`Esc` 或 Dashboard 的退出按钮结束。

完整中文手动流程、15 页逐项检查表和 LiDAR 读图方法见 [`docs/阶段三GUI手动测试教程.md`](docs/阶段三GUI手动测试教程.md)。

平面示例：

```bash
python main.py --gui --manual --interface-mode local --drive-model physics --robot-model df_front --terrain-model flat --slope-deg 0
```

斜面示例：

```bash
python main.py --gui --manual --interface-mode local --drive-model physics --robot-model df_mid --terrain-model slope --slope-deg 8
```

高尔夫起伏示例：

```bash
python main.py --gui --manual --interface-mode local --drive-model physics --robot-model active_steering_4wd --terrain-model golf_heightfield --golf-seed 41 --golf-relief medium
```

阶段二障碍物 GUI 验收示例：

```bash
python main.py --config configs/stage2_obstacles_gui.yaml --gui --manual --interface-mode local
```

真实 eCAL 人工验收时改用 `--interface-mode ecal`，并由外部 peer 发布 `WheelCommand`；该模式下键盘速度不会替代企业命令。GUI 手动工作流会把 PyBullet 主窗和 Dashboard 按当前主屏可用区域分为名义 `67:33`：Dashboard 宽度精确按 `33/100` 计算，Main 使用余宽。Dashboard 默认提供 15 个企业页面，其中 13 个为图表页；接触力、接触点数和打滑指标只在显式启用的开发者诊断中显示。

Dashboard 的“障碍物”页可以随机追加静态、移动或混合障碍物。混合模式默认按 30% 生成移动障碍物，例如添加 10 个时得到 7 个静态和 3 个移动；相同场地、种子和参数会复现同一组逻辑布局。表格单选逻辑 ID 后可删除选中项，也可以清空全部。切换平面、斜面和高尔夫场地时，障碍物保留逻辑 ID、XY、路径进度和方向，并重新贴合目标地表。移动障碍物是运动学刚体，会沿直线往返并与车辆发生碰撞，但不会被车辆撞偏；自动导航、自动刹停和动态避障均不在当前阶段四范围。

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
python main.py --mode direct --interface-mode local --robot-model df_back --terrain-model golf_heightfield --golf-seed 23 --golf-relief low
python main.py --mode direct --interface-mode local --robot-model df_back --terrain-model golf_heightfield --golf-seed 23 --golf-relief high
```

同一 `golf_seed` 和 `golf_relief` 会生成相同高度数据。`low`、`medium`、`high` 只暴露易理解的起伏预设，网格分辨率和碰撞尺度属于内部参数。

## 本轮物理证据

四驱的可复制 DIRECT 诊断、关节顺序、`getJointState()` 字段和本轮输出统一记录在[交付报告的主动转向四驱逐关节诊断](docs/阶段一交付报告.md#主动转向四驱逐关节诊断)。该处是本项目四驱物理证据的唯一复现入口。

## GUI 验收步骤

以下步骤供桌面会话人工验收使用；开发方自动 GUI 门禁使用独立 `33/100` 水平 oracle、独立 `1:1` 上下区 oracle 和 Dashboard schema v4 两轮布局报告，实际点击页签左右滚动按钮，覆盖 15 个默认页非空、文字与控件完整包含、图表 artist 不重叠、数据变化前后布局稳定、绘图区占比和持续键盘驾驶，最终观感仍由用户确认：

1. 运行上面的 8 度 `slope` 命令，前进驾驶，确认车辆从高位平地沿 `+X` 依次经过下坡和低位平地。
2. 在 Dashboard “仿真控制”中设置线速度和角速度，再用方向键驾驶；这两个值只参数化 local 键盘命令，真实 eCAL 命令不读取它们。
3. 运行同一组 `golf_seed` 和 `golf_relief` 两次并重新应用场地，确认丘陵、浅洼和驾驶廊道一致；改变任一参数后确认地形变化。
4. 选择 `active_steering_4wd`，同时按前进和左/右，确认 Dashboard 的四轮实际轮速与两个前轮实际转角均更新。
5. 打开 `LiDAR点云`：车辆箭头固定在原点并指向 `+X`，30 m 虚线环和固定坐标范围不随点数缩放；结合图例辨认地形、静态和移动障碍命中。

## 主要目录

```text
slope_sim/model_registry.py       四车型注册表和语义关节元数据
slope_sim/robot.py                差速与主动转向控制、物理状态读取
slope_sim/scene.py                flat/slope/golf_heightfield 场地创建
slope_sim/simulation.py           DIRECT/GUI 自动实验流程
slope_sim/manual_demo.py          GUI 手动驾驶流程
slope_sim/dashboard.py            企业 Dashboard、逐话题状态和场景控制
slope_sim/interfaces/             Protobuf、transport、时钟、日志和运行时
slope_sim/lidar_pointcloud.py     前后多线点云与碰撞过滤
slope_sim/truth_sensors.py        双天线 RTK 与 IMU 真值
slope_sim/scene_config.py         版本化场景导入导出
urdf/df_front.urdf                前置差速模型
urdf/df_mid.urdf                  中置差速模型
urdf/df_back.urdf                 后置差速模型
urdf/active_steering_4wd.urdf     主动转向四驱模型
scripts/verify_stage1_matrix.py   4×3 DIRECT 验证
scripts/verify_stage2_obstacles.py 阶段二障碍物 DIRECT 验收
scripts/verify_stage3_interfaces.py 阶段三 DIRECT/性能验收
scripts/verify_ecal_roundtrip.py  真实 eCAL 进程门禁
scripts/verify_dashboard_manual_drive.py GUI 67:33/15 页/驾驶门禁
configs/stage2_obstacles_gui.yaml 阶段二障碍物 GUI 验收配置
tests/                            自动测试
```

## 开源设计参考

- Bullet 官方 `racecar.py`：前轮转向和驱动关节控制方式。
- Bullet 官方 `racecar_differential.py`：四轮联动和齿轮约束思路；本项目不使用硬编码关节索引。
- Bullet 官方 `heightfield.py`：`GEOM_HEIGHTFIELD` 的创建方式。
- 仓库内 `references/repos/pybullet_diffdrive`：差速轮和球形支撑轮布局。
- 仓库内 `references/repos/pybullet_sim`：差速控制、射线和仿真封装方式。

阶段三详细范围、自动验证和人工验收门禁见 `3d仿真平台需求规格.md` 第 14.3 节。
