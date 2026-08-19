# runSim v2 实时 eCAL 与点云执行计划

> 状态：待实施。此计划取代 `runSim` 当前“关闭接口、结束后离线重建”的默认运行语义；
> 旧 v1 前后双 LiDAR 不得作为本计划的正式实时链。

## 1. 已确认的架构事实

- 最终需求规定 **eCAL Protobuf v2 是正式实时数据面**。Simulator 在 240 Hz
  物理循环中发布 `/sim/wheel/state`（100 Hz）、`/sim/lidar/points`、
  `/sim/rtk/state`、`/sim/imu/attitude`（后三者 10 Hz）。
- 阶段四只有一个位于车体中心的 `lidar_link`，采用 MID-360 风格 360 度点云；
  旧的前/后双雷达属于 v1 历史基线，不再是正式 Dashboard 语义。
- C++ Command 必须是 `/sim/wheel/command` 的唯一 publisher。GUI/Dashboard
  的键盘输入不能绕过它直接控制 PyBullet，而要通过本机 Unix socket 传递人工目标。
- 实时点云显示应走 `eCAL v2 -> ROS 2 Bridge -> RViz2`。Livox Viewer 2 只打开
  已完成 MCAP 导出的 synthetic LVX2，不承担 eCAL 实时显示或真实设备发现。
- 当前 `runSim` 默认追加 `--no-interface`，因此 eCAL、LiDAR、RTK、IMU 均未运行；
  这是此前消除旧 v1 同步双雷达卡顿的临时路径，不符合上述正式架构。

## 2. 目标用户流程

1. 执行 `runSim`，启动 PyBullet GUI、统一 Dashboard、正式 eCAL v2 Simulator
   和唯一 C++ Command；启动失败不得回退 local/v1。
2. Dashboard 显示中心 LiDAR、RTK、IMU、WheelCommand、WheelState 的 peer、频率、
   sequence、drop/error 和会话 identity。
3. 用户在主 GUI 或 Dashboard 获得焦点后使用方向键。Dashboard 将人工目标写入
   受权限保护的本机 socket；Command 以 100 ms 租约发布 WheelCommand。
4. 点击“打开实时点云”时，启动 ROS Bridge 和预设 RViz2；RViz 固定坐标系为 `world`，
   通过 RTK/IMU 的同刻 TF 累积显示实时点云。
5. 点击“启用采集”后启动真实 Recorder；点击“结束采集”后在下一个完整 10 Hz 边界
   finalize MCAP，导出 PCD/PLY/LVX2，并允许 Dashboard 打开 Livox Viewer 2。

## 3. 实施顺序

### Task 1：定义并验证 v2 交互会话合同

- 在新模块中定义 runSim 的正式会话状态、进程身份、socket 路径和状态快照。
- Unix socket 目录必须为 `0700`，服务端以 `SO_PEERCRED` 核对同 uid、由编排器启动的
  Command PID；消息只承载人工目标、状态和停止请求，不能承载点云。
- 为断开连接、键盘释放、窗口失焦、超时、重复 Command、未知 PID 和非法目标添加 RED
  测试；实现后进行 GREEN。

### Task 2：扩展 C++ Command 为持续人工控制 peer

- 扩展 `cpp/client/stage4_command.cpp`：保留既有预制 payload/schedule 工具模式，新增
  由受监管 Unix socket 驱动的 interactive mode。
- interactive mode 由 C++ Command 维持唯一 eCAL publisher、100 Hz 发布和 100 ms
  单调墙钟租约；socket 无有效目标时发布零轮速。
- 将线速度/角速度转换为四种车型对应的 `2+0` 或 `4+2` WheelCommand，身份、generation
  与 descriptor 必须由 Command/runtime 协商，不能由 Dashboard 伪造。
- 添加 C++ 单元、Python 跨进程和真实 eCAL exact-one peer 回归。

### Task 3：把 v2 runtime 接入现有手动 GUI 世界

- 从 `scripts/stage4_v2_simulation_runtime.py` 抽取可复用的 v2 运行组件；不要在
  `runSim` 旁再创建一个 DIRECT PyBullet 世界。
- 在 `slope_sim/manual_demo.py` 的现有 `SimulationCoordinator` 物理主线程接入该组件：
  240 Hz 物理、100 Hz WheelState、10 Hz v2 sensor cadence 和 scene transaction 共用
  同一个 world generation。
- 使用既有异步 LiDAR worker 的实时 profile（5,760 条候选射线）；禁止在驾驶期运行
  20,000 条离线 dense profile，也不得恢复 v1 双雷达同步扫描。
- 场地、车型和障碍物切换要原子重建 worker、更新 v2 generation，并使旧 Command token
  安全失效。

### Task 4：用 v2 传感器状态重做正式 Dashboard

- 在 `slope_sim/dashboard.py` 中将正式实时页从 v1 前/后 LiDAR 改为中心 MID-360、RTK、
  IMU、WheelCommand、WheelState。
- 保留 Dashboard 只读取有界不可变 snapshot 的边界；不要在 Dashboard 保存、解码或累积
  完整点云。轻量预览必须限点、限频且可关闭。
- 移除当前“接口已关闭/传感器未启用”的默认文案；正常 `runSim` 应显示 eCAL 已验证和
  实际频率。保留对 eCAL 初始化、peer 和 worker 错误的可操作说明。
- 保持输入框禁用滚轮误调，并加入覆盖所有 `QAbstractSpinBox` 的 GUI 回归。

### Task 5：实现人工 Recorder 开始/停止边界

- 扩展 `cpp/client/stage4_recorder.cpp`，新增受编排器控制的 interactive recording mode。
  现有“预先给定精确数量”模式保持用于验收；新模式从完整 10 Hz 边界开始，在 stop 请求后
  接收最后一个同刻 LiDAR/RTK/IMU 边界并统计实际五 topic 数量。
- 对齐完成后 drain、flush、fsync、原子 manifest 发布；队列溢出、磁盘错误、generation
  变更、Command 丢失或异常退出都必须安全停车，并禁止标记会话为可导出。
- Dashboard 的“启用采集/结束采集”改为驱动该 Recorder；MCAP 成功后复用既有 Export
  生成 PCD/PLY/LVX2，Livox Viewer 自动导入仅保留为离线动作。

### Task 6：接通 ROS Bridge 与 RViz2 实时累积显示

- 扩展 `cpp/client/stage4_ros2_bridge.cpp` 的长驻受监管模式，订阅 live `/sim/*`，发布
  `/slope_sim/lidar/points`、Livox `CustomMsg`、RTK、IMU、clock 和完整 `world -> base_link
  -> lidar_link` TF。
- 新增固定 `world` 坐标系的 RViz2 配置和 Dashboard “打开/关闭实时点云”操作；显示使用
  有界 history/decay，不把累计点永久缓存在 Simulator 或 Dashboard。
- Bridge/RViz2 崩溃或用户关闭时，Simulator、eCAL、Command 和 Recorder 必须继续运行，
  Dashboard 允许单独重启显示链。

### Task 7：启动、性能和端到端验收

- 更新 `runSim`，默认进入 v2 实时编排；把 `--interface-mode ecal` 的 v1 路径明确标为
  legacy，不得与正式 v2 topic 同时运行。
- 增加 eCAL 配置预检，明确处理配置路径、time-sync plugin、descriptor、participant 和
  peer 缺失；失败消息需出现在 Dashboard 与终端。
- 自动回归覆盖四车型、三场地、结构切换、控制安全、v2 topic identity/cadence、Recorder
  finalize、ROS TF、RViz 启动失败隔离和 LVX2 离线导出。
- 在本机真实桌面执行联合负载：Golf heightfield、20 障碍物、5,760 rays、GUI、Dashboard、
  logging、eCAL、Recorder，并分别测试 ROS off/on。验收目标：`sim/wall=0.98..1.02`、
  GUI 事件空窗 `<=100 ms`、Dashboard draw p95 `<100 ms`、五 topic 的 drop/error 为零。

## 4. 非目标与风险

- “实时建模”在本计划中是将同刻点云按 RTK/IMU TF 累积显示，不包括 SLAM、回环或地图优化。
- Livox Viewer 2 不订阅 eCAL；它无法替代 RViz2 的实时显示。
- Iris Xe 共享 GPU 下，RViz2 是可选显示消费者；运行时必须验证 ROS off/on 性能，而不是
  通过降低 eCAL 频率或 LiDAR rays 达到流畅。
- 这是跨 Python/C++、eCAL、GUI、Recorder 与 ROS 的公共架构改动；每个 Task 按 TDD 线性完成，
  Task 3-7 合并后进行一次完整六维只读审查。
