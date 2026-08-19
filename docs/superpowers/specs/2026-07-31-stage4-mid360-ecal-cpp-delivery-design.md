# 阶段四单机运行与联网安装设计

> 初版日期：2026-07-31
>
> 架构修订：2026-08-10
>
> 状态：单机多进程、单文件联网安装和 sudo 边界已按本设计实施；完成状态、
> 真实门禁和最终独立审查结论以 `docs/阶段四交付报告.md` 为准

## 1. 目标与范围

阶段四面向一台 Ubuntu 24.04 amd64 工作站，继续使用 Python/PyBullet
提供物理仿真，并通过 eCAL Protobuf v2 与 C++ 工具通信。

交付功能：

- 一台安装在车体几何中心的 MID-360 风格 360 度点云 LiDAR。
- 固定 `LEFT/CENTER/RIGHT` 三点 RTK 和车体航向。
- 轮态、LiDAR、RTK、IMU 和轮控命令的 eCAL Protobuf v2。
- 独立 C++ Subscriber、Command、Recorder、Replay 和 Export。
- 原始 MCAP 记录、隔离回放、PCD、PLY 和合成 LVX2 导出。
- 可选 ROS 2 Jazzy Bridge 和 RViz2 显示。
- 一个可执行的 `.run` 安装包；安装时联网下载依赖。

不在本阶段实现：

- 自动导航、路径规划、SLAM、定位融合和多车协同。
- ROS 2 作为 Simulator 与 eCAL 之间的必经链路。
- 真实 MID-360 的光学、电气、噪声、雨雾和多回波数字孪生。
- 把双根字节复现、断网构建或跨机器证据事务作为交付功能。

阶段四 A-E 已按本设计完成实现、真实门禁和最终独立六维审查。历史细节从 Git
追溯，本文只描述当前目标架构；实时验收状态以阶段四交付报告为准。

## 2. 总体架构

```text
                    local Unix control socket
                 +---------------------------+
                 |    Python orchestrator    |
                 +---------------------------+
                      starts / status / stop
                                 |
+--------------------------------+----------------------------------+
|                     one Ubuntu workstation                        |
|                                                                   |
| C++ Command  -- /sim/wheel/command --> Python/PyBullet Simulator  |
| C++ Command  <-- /sim/wheel/state   --- Python/PyBullet Simulator |
|                                                                   |
| Simulator -- four output topics --> C++ Subscriber                |
|           |                      -> C++ Recorder -> MCAP           |
|           |                      -> optional ROS 2 Bridge -> RViz2 |
|           +-> bounded snapshot -> Qt Dashboard                    |
|                                                                   |
| session manifest -> Replay -> /replay/sim/*                       |
|                  -> Export -> PCD / PLY / synthetic LVX2          |
+-------------------------------------------------------------------+
```

固定边界：

- Python/PyBullet 是唯一物理世界和传感器真值生产者。
- eCAL Protobuf v2 是正式实时数据面。
- Unix socket 只负责本机编排、人工目标、状态和停止屏障。
- Command 是唯一轮控 publisher；Subscriber 和 Recorder 永远只读。
- Recorder 的磁盘 I/O 不阻塞 PyBullet 主线程。
- ROS 2、RViz2、Replay 和 Export 是下游消费者，不进入核心启动依赖。
- 所有正式组件都运行在同一台电脑，不使用 Docker，也不要求第二台机器。
- 安装完成后的运行流程不主动访问网络。

## 3. 进程与生命周期

### 3.1 Simulator

Simulator 负责 240 Hz 物理步进、命令超时、车型和场地事务、轮态及三类
传感器采样。eCAL callback、日志线程和 Dashboard 都不得直接调用 PyBullet。
LiDAR 可使用持久 worker，但物理世界所有权仍在 Simulator。

正式模式必须初始化真实 eCAL；失败时退出，不能静默降级到 local。
LocalTransport 只用于单元测试和显式开发模式。

### 3.2 C++ 进程

- `slope-sim-sub`：只读订阅、类型验证、频率和连续性诊断。
- `libslope_sim_client`：C++17 公共 SDK、头文件和 CMake package config，
  供 Subscriber、Recorder、Bridge 和外部只读客户端复用。
- `slope-sim-command`：观察 WheelState 后发送车型对应的 `2+0` 或
  `4+2` 命令。
- `slope-sim-record`：无损记录五个正式 topic。
- `slope-sim-replay`：从完整 session manifest 回放到隔离 namespace。
- `slope-sim-export`：导出 PCD、PLY 和合成 LVX2。
- `slope-sim-ros2-bridge`：可选 ROS 2 Jazzy 适配器。

Command 每次启动生成新的 16-byte source session。正式会话只允许一个
Command peer；零个 peer 时等待并停车，多个 peer 时进入冲突状态并停车。
人工控制使用 100 ms 单调墙钟租约；按键释放、窗口失焦、连接关闭、租约
过期或命令权变化都立即归零。

### 3.3 编排与停止

编排器在创建 eCAL participant 前取得单机独占锁。本地 socket 目录权限为
`0700`，连接使用 `SO_PEERCRED` 核对 uid 和已启动子进程 PID。

正常停止顺序：

1. Command 发布零命令并冻结 publisher。
2. Simulator 继续至少一个 10 Hz 周期，冻结四个输出边界。
3. Recorder 接收完边界内消息，排空有界队列并 flush/fsync。
4. Recorder 原子完成 segment 和 session manifest 后报告 finalized。
5. 编排器关闭其余 participant 并释放单实例锁。

Recorder 队列溢出、磁盘不足或持久化失败会使正式会话失败；编排器先安全
停车，再有序关闭。交互调试可以显式不启动 Recorder，但 GUI 和 CLI 必须
持续显示“未记录”。

## 4. eCAL Protobuf v2

v1 schema 和 descriptor 保持冻结，只用于历史数据读取；生产 Simulator
只发布 v2，不长期双发。

| 方向 | Topic | 类型 | 频率 |
|---|---|---|---:|
| Simulator 订阅 | `/sim/wheel/command` | `WheelCommand` | 100 Hz |
| Simulator 发布 | `/sim/wheel/state` | `WheelState` | 100 Hz |
| Simulator 发布 | `/sim/lidar/points` | `LidarPointCloud` | 10 Hz |
| Simulator 发布 | `/sim/rtk/state` | `RtkState` | 10 Hz |
| Simulator 发布 | `/sim/imu/attitude` | `ImuAttitude` | 10 Hz |

所有顶层消息携带：

- 16-byte `simulation_session_id`。
- 32-byte descriptor SHA-256。
- `world_generation` 和逐 topic `sequence`。
- 统一仿真 `timestamp_ns`。

轮控消息额外携带 `command_generation`、`source_id` 和 16-byte
`source_session_id`。旧会话、旧 generation、错误 owner、重复或逆序
sequence、错误 descriptor、NaN/Inf 和错误数组长度都整条拒绝，不能刷新
100 ms 有效命令墙钟。

WheelState 为 100 Hz；LiDAR、RTK、IMU 共用每第十个 WheelState 的整数
采样位点，四个消息的 `timestamp_ns` 必须完全相等。不得用浮点 deadline、
最近邻配对或跨 generation 插值掩盖缺帧。

数组顺序：

- `df_front/df_mid/df_back`：驱动轮 `[left, right]`，转向数组为空。
- `active_steering_4wd`：驱动轮
  `[front_left, front_right, rear_left, rear_right]`，转向轮
  `[front_left, front_right]`。

Python 与 C++ 只从同一 `.proto` 生成 binding 和 descriptor，并以固定
golden bytes 双向验证。

## 5. 传感器与场景

### 5.1 LiDAR

- 四种车型只有一个中心 `lidar_link`，坐标系为 `+X` 前、`+Y` 左、
  `+Z` 上。
- 实时 profile 使用 5,760 条候选射线；离线 dense profile 使用 20,000 条。
- 一帧扫描使用同一冻结世界快照，排除机器人本体，输出可见表面命中。
- 点字段保持 Livox `CustomPoint` 风格：
  `offset_time_ns/x/y/z/reflectivity/tag/line`。
- 第一版 `reflectivity=0`、`tag=0`；`line` 仅表示合成扫描线。
- 不把障碍物中心、类别或真值列表伪装成点云。

### 5.2 RTK 与 IMU

RTK 固定输出世界坐标 `LEFT/CENTER/RIGHT` 三点。CENTER 位于机器人轴心
中心，LEFT/RIGHT 按车型 canonical 几何放置。

`heading_rad` 是 `RIGHT -> LEFT` 基线在世界 XY 平面的方位减
`pi/2`，归一化到 `[-pi, pi)`；它不是非零 roll/pitch 下的 ZYX Euler
yaw。ROS Bridge 和 Export 必须复用同一姿态恢复纯函数。

IMU 输出有限的 roll/pitch 和统一时间、会话、generation、sequence 信息。

### 5.3 场景

schema v2 保留车型、平面/斜面/高尔夫场地、障碍物和传感器 profile。
场景修改在物理主线程事务化执行，失败时回滚。成功重建才推进
`world_generation`；旧命令 token 不恢复。

v1 场景只由冻结读取器打开，显式转换为 v2，并报告“双雷达替换为一个中心
360 度 LiDAR”等语义变化。未知且无法无损转换的字段直接报错。

## 6. 记录、回放与导出

Recorder 保存原始 eCAL Protobuf bytes，不经过 ROS 二次编码。每条业务消息
与记录 metadata 一一配对，metadata 至少保存 topic、类型、会话、descriptor
SHA、payload SHA、仿真时间、接收时间、generation、sequence 和 scene
revision。

每个会话包含：

- 一个最终 session manifest。
- 一个或多个按顺序编号的 MCAP segment。
- 初始场景及已提交 revision 的 canonical YAML attachment。
- segment 大小、SHA-256、首尾 topic identity 和 scene revision 范围。

写入中的文件使用 `.partial`，只有 flush/fsync 和 manifest 校验成功后才
原子完成。Reader 拒绝缺段、错序、hash/CRC 不符、路径逃逸和不完整会话。

Replay 默认不回放 WheelCommand，只发布到 `/replay/sim/*`，不会污染实时
`/sim/*`。显式命令回放只允许隔离的 DIRECT shadow world。

Export：

- PCD/PLY 保留有效点数和明确坐标系，可导出 `lidar_link` 或严格同刻配对
  后的 world 坐标。
- 合成 LVX2 面向 Livox Viewer 2，是有损显示格式；sidecar 标记
  `synthetic=true` 并记录量化、填充和源 MCAP hash。
- 原始 MCAP 始终是权威数据，导出失败可以修复配置后重试。
- Livox Viewer 2 当前只验证了 loopback-only 启动与 `/Game/Maps/Viewer` 地图加载
  smoke；这不证明 LVX2 导入、模拟点云显示或真实 MID-360 发现。标准 LVX2 离线
  回放仍待单独验收。

## 7. 可选 ROS 2 与界面

安装器只有在 `--with-ros` 时安装 ROS 2 Jazzy、Livox 消息依赖和 RViz2。
Bridge 不改变 eCAL payload，关闭 Bridge/RViz2 后核心消息、频率和记录不变。
实时点云显示由 ROS/RViz2 路径承担，不以 Viewer 启动 smoke 代替。

- live：`/sim/* -> /slope_sim/*`。
- replay：`/replay/sim/* -> /replay/slope_sim/*`。
- 输出 Livox `CustomMsg`、`PointCloud2`、轮态、RTK、IMU、TF 和 clock。
- TF/点云只使用同 session、generation 和完全相同 timestamp 的消息。
- Bridge/RViz2 失败可单独重启，不使 Simulator 或 Recorder 退出。

Dashboard 继续显示运行状态、场景、轮态、传感器和逐 topic 健康，但不复制
完整点云。GUI、Dashboard、相机和图表分别限频，物理循环保持 240 Hz。

## 8. 单文件联网安装器

### 8.1 交付物

```text
slope-sim-stage4-<version>-ubuntu24.04-amd64.run
```

`<version>` 必须是 canonical SemVer 2.0.0 单一路径分量；空白、控制字符、
斜杠、`.`、`..`、前导 `v` 和数字段前导零均拒绝。

安装器是唯一交付文件，可用：

```bash
bash slope-sim-stage4-<version>-ubuntu24.04-amd64.run [--with-ros]
```

它包含项目代码、C++ 源码、模型和场景资源、Protobuf、默认配置、依赖锁、
许可证及安装/卸载/doctor 脚本；不包含第三方源码归档、Conda package cache
或开发构建树。

### 8.2 依赖策略

- 目标固定 Ubuntu 24.04 amd64。
- apt 锁定包名和兼容范围，允许同一 Ubuntu release 的安全补丁，安装状态
  记录实际版本。
- eCAL 固定为 `6.1.1`；Protobuf 固定为源码/protoc/C++ `33.6`、Python
  runtime `6.33.6`。MCAP、Zstd 和其他关键 ABI 依赖同样固定版本、URL、
  SHA-256 和许可证。
- Python 使用精确 lock；项目自身 wheel 不参与外部依赖求解。
- ROS 依赖只在 `--with-ros` 时安装。
- 单个进程不得加载第二套 eCAL 或 `libprotobuf.so`。

安装时联网下载。连接中断、超时和服务端临时错误可以有限重试；下载完成后
只校验一次 SHA-256，摘要不符立即删除该文件并失败，不能把相同或备用 URL
的再次下载当成恢复路径。没有校验通过的 fallback 版本。下载和编译在普通
用户的临时目录运行。sudo 只用于 apt、全局安装锁、写入 `/opt` 和切换版本，
不以 root 身份执行网络下载或项目构建。

### 8.3 安装事务

1. 检查 Ubuntu 24.04、amd64、网络、磁盘和 sudo，并以 `mktemp` 创建本轮
   唯一普通用户工作目录。
2. 校验 `.run` 内嵌 payload manifest 和文件摘要；拒绝绝对路径、`..`、
   重复成员、逃逸链接和特殊文件。
3. 通过 sudo 在 `/run/lock/slope-sim-installer.lock` 取得一个 `flock(2)`
   全局排他锁。安装、回退和卸载共用该锁，锁被占用时明确退出，不并发修改
   apt 或 `/opt/slope-sim`。
4. 若 `releases/<version>` 已存在，立即校验 `install-state.json` 中的版本、
   Git SHA、payload manifest SHA 和规范化安装选项（含 `with_ros`），再校验全部
   文件摘要。身份或选项不符、文件损坏都失败；完全一致时不下载、不构建、
   不覆盖，只在 `current` 尚未指向该版本时原子补做切换，然后成功退出。
5. 仅在目标版本不存在时联网下载并验证依赖；创建 Python 环境并构建 C++/
   可选 ROS 组件。
6. 写入唯一 staging：
   `/opt/slope-sim/releases/.<version>.incoming.<nonce>`。
7. 在 staging 中执行 doctor 和核心 smoke。
8. 确认 `releases/<version>` 仍不存在后，原子改名为该目录；再通过同目录
   临时符号链接原子切换 `/opt/slope-sim/current`。

任一步失败都不切换 `current`。临时下载在成功后删除；失败时默认只保留
安装日志，显式 `--keep-work` 才保留工作目录。

同版本重复安装只做身份、安装选项和文件完整性检查；绝不覆盖原目录。完整但
未激活时，只允许原子补做 `current` 切换，使“release 已发布、current 尚未切换”
的中断窗口可在重试时收敛。损坏或同版本 payload/安装选项不一致时失败；想改变
`--with-ros` 选择必须发布、安装一个新版本。新版本通过 smoke 后才切换。
安装器支持列出版本、回退到已通过完整性检查的旧版本，以及卸载非当前版本。
apt 已安装的公共系统包不自动卸载，避免破坏其他软件。

每个版本只保存一份简洁 `install-state.json`：项目版本、Git SHA、payload
manifest SHA、依赖版本、下载摘要、规范化安装选项（至少含 `with_ros`）和
smoke 结果。不要恢复多层 handoff/evidence 状态机。

默认配置位于 release 的 `share/slope-sim`；用户覆盖位于
`~/.config/slope-sim`，记录和导出位于 `~/slope-sim-data`。升级和回退
不得覆盖用户目录。systemd user service 默认禁用，只有显式启用才自启动。

## 9. 异常与恢复

| 异常 | 行为 |
|---|---|
| 安装平台、网络或 sudo 不满足 | 写明检查项并在修改系统前退出 |
| 下载传输失败 | 有限重试后失败，不切换当前版本 |
| 下载 SHA-256 不符 | 立即删除该文件并失败，不重试、不使用 fallback |
| 全局安装锁被占用 | 明确提示已有安装事务并退出，不修改系统 |
| 同版本目录损坏 | 拒绝覆盖，要求使用新版本号重新发布 |
| 同版本 payload 或安装选项不一致 | 拒绝覆盖；按原选项重试或为新选项发布新版本 |
| release 已发布但 current 未切换 | 重试时校验完整后只原子补做 current 切换 |
| 构建、doctor 或 smoke 失败 | 删除本轮唯一 staging；旧版本继续运行 |
| 正式 eCAL 初始化失败 | 会话启动失败，不降级 local |
| Command peer 消失或冲突 | 撤销命令权并在 100 ms 内停车 |
| 非法消息或旧 generation | 整条拒绝、计数，不刷新命令租约 |
| LiDAR 一帧失败 | 该帧不发布，其他 topic 继续并留下 sequence gap |
| Recorder 队列满、磁盘满或 CRC 错误 | 安全停车，会话失败并保留诊断 |
| Bridge/RViz2 失败 | 核心运行和记录继续 |
| 场景重建失败 | 回滚旧世界，旧命令 token 失效 |
| Replay/Export 失败 | 原始 MCAP 保持只读，可重新执行 |

## 10. 验证与完成门槛

### 10.1 自动测试

- v2 Python/C++ golden、descriptor 和非法消息拒绝。
- 四车型乘三场地的传感器真值、数组语义、重建和安全停车。
- Recorder 截断、CRC、磁盘/队列故障、session manifest 和 scene attachment。
- Replay namespace 隔离，以及 PCD/PLY/LVX2 回读。
- ROS 启用时的 Bridge 配对、TF、clock 和 RViz2 实时显示 smoke。
- 标准 LVX2 离线回放的 Viewer 导入与模拟点云显示验收；不得以 loopback-only
  地图加载 smoke 代替。
- 安装器的平台、sudo、传输重试、SHA 篡改立即失败、全局锁竞争、唯一
  staging、重复安装身份/选项/完整性检查、中断后只补激活、同版本损坏或选项
  漂移拒绝、失败不切换、回退和卸载测试。
- 使用本地 HTTP fixture 的完整联网安装集成测试。

普通回归不依赖外部 build 制品或真实 eCAL；外部门禁使用独立 marker 和明确
前置条件。

### 10.2 实际运行

正式会话同时运行 Simulator、Command、Subscriber 和 Recorder，验证：

- 主动转向 `4+2` 和代表性差速 `2+0`。
- 五个 topic 的会话、类型、payload hash、generation 和 sequence 对应。
- WheelCommand/WheelState 为 `95..105 Hz`，LiDAR/RTK/IMU 为
  `9..11 Hz`。
- 五秒正式窗口内运动成立，transport、consumer 和 Recorder drop/error 为 0。
- 正常停止后 Recorder finalized，队列为 0。

联合负载使用 `golf_heightfield + 20 障碍物 + 5,760 射线 + GUI +
Dashboard + logging`，要求 `sim/wall=0.98..1.02`、GUI 事件空窗
`<=100 ms`、Dashboard draw p95 `<100 ms`。ROS off/on 分别验证。
离线 dense profile 不要求墙钟实时，但保持完整性和确定性。

GUI 在实际桌面以及 `1366x768`、`1920x1080`、`2560x1440` 验证布局、
控件和数据更新。真实 eCAL、真实 GUI/RViz2 和性能 invocation 继续按项目
规则逐条取得授权、串行执行；失败保留证据，不自动重跑。

发行验收只要求：

1. 生成一个 `.run` 文件并记录其 SHA-256。
2. 在一台干净 Ubuntu 24.04 amd64 电脑完成联网安装。
3. 运行核心 smoke；使用 `--with-ros` 时再运行 ROS/RViz2 smoke。
4. 验证新版本安装、失败不切换、回退和卸载非当前版本。
5. 大阶段结束后完成独立只读六维审查，且
   `Critical=0, Important=0`。

本文是纯设计变更，不用文档改写伪造 TDD RED。后续重要实现仍按项目 TDD
规则执行。

## 11. 明确取消的复杂度

以下内容不再属于需求或完成门槛：

- 构建和安装全程断网。
- network namespace、无默认路由和 TEST-NET 证明。
- canonical Python package、wheel 和 source archive cache。
- cache producer、materializer 及两轮私有 cache 复制。
- 两个空根构建 byte-identical 归档。
- 第三套 lifecycle probe 和候选/正式 payload equivalence。
- portable SSH transfer、challenge registry、receipt 和跨机 import context。
- accepted-candidate、final-status 和多层 evidence/handoff 事务。
- 自包含 Conda runtime 的离线迁移包。
- 原地覆盖同版本的 `--repair` 路径。

仍保留依赖版本、URL、SHA、许可证、单进程 ABI、路径安全、失败不切换、
真实运行验收和六维审查。这里取消的是发行证明链，不是业务功能、安全停车、
离线 Replay/Export 或 canonical 车型/场景语义。

## 12. 文档与兼容性

- `3d仿真需求.md` 是历史原始输入，不再作为当前阶段四接口定义。
- 根需求规格负责“必须做什么”；本设计负责当前组件和交付边界。
- `ARCHITECTURE.md` 应改为当前单机多进程运行图。
- 旧阶段四 master/A-E 详细计划只保留状态索引；轻量实施计划及其完成结论见
  `docs/superpowers/plans/2026-08-10-stage4-b-e-lightweight-implementation.md`。
- README 只写已经实现和验证的状态，不提前宣称后续功能完成。
- 阶段三 v1 报告保留历史证据，不改写为 v2 PASS。

任何公开 topic、v2 字段号、传感器语义或用户功能变更都必须重新取得用户
确认；单纯删除已取消的发行证明条款不构成 wire 兼容性变化。

## 13. 参考资料

- Eclipse eCAL：<https://eclipse-ecal.github.io/ecal/>
- Livox ROS Driver 2：<https://github.com/Livox-SDK/livox_ros_driver2>
- ROS 2 Jazzy：<https://docs.ros.org/en/jazzy/>
- MCAP：<https://mcap.dev/>
- 仓库 `references/`：只用于阅读成熟实现和许可证，不作为安装缓存。
