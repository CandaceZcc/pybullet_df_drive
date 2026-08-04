# 阶段四 MID-360、eCAL C++ 与完整迁移交付设计

> 设计日期：2026-07-31
>
> 设计状态：用户已逐节确认，独立合同审查修订完成，等待实施计划复核
>
> 阶段三基线：`ce3bee0`（`阶段三测试2: 1. 接口收口 2. GUI反馈 3. 验收证据`）

## 1. 目标、权威关系与范围

阶段四不再实现原需求中的自动导航，而是在阶段三可运行基线之上完成传感器、跨语言接口、数据工具、性能和迁移交付。发生冲突时，用户在本设计会话中确认的决定优先于旧需求、旧阶段计划和历史报告；实施开始后应把相同边界同步到根目录权威需求规格。

阶段四交付以下能力：

- 优化 PyBullet GUI、Dashboard 和高尔夫场地完整负载下的实时性。
- 将前后双 LiDAR 改为一台安装在车体几何中心的 MID-360 风格点云 LiDAR。
- 以固定 `LEFT/CENTER/RIGHT` 角色输出三点 RTK 世界坐标和航向。
- 冻结阶段三 `slope_sim.interfaces.v1`，新增 Python/C++ 共用的 eCAL Protobuf v2。
- 提供 C++ Subscriber SDK、只读 CLI、无损记录器和独立命令测试工具。
- 提供可选 ROS 2 Jazzy Bridge，以 RViz2 实时显示 Livox 风格三维点云。
- 提供 MCAP 无损记录、隔离回放、PCD、PLY 和合成 LVX2 导出。
- 提供可复制到公司 Ubuntu 电脑的完整版本化发行包。

本阶段明确不实现：

- 自动寻路、路径规划、轨迹规划和自动避障决策。
- SLAM、地图构建、定位融合和多车协同。
- 将 ROS 2 放入 Python 仿真到 eCAL 的必经主链路。
- MID-360 光学、电气、噪声、多回波、雨雾或真实非重复扫描规律的硬件级数字孪生。
- 从点云自动聚类出障碍物中心、包围盒或类别；正式 LiDAR 输出是可见表面点，不是障碍物真值列表。

阶段三设计、计划和交付报告是历史证据。它们可以增加“已被阶段四 v2 替代”的说明，但不得把旧双雷达、旧 RTK 或旧测试结果改写成阶段四已经实现或通过。

## 2. 关键方案选择

### 2.1 采用的总体方案

采用“单台 Ubuntu 工作站、原生多进程、eCAL 为主通道”的架构。最终生产运行不要求第二台电脑；开发时可额外启动 RViz2、回放器和诊断工具，但它们仍是同一主链路的可选消费者：

```text
                         local control socket
                    +---------------------------+
                    |     Python orchestrator   |
                    +---------------------------+
                       | starts/status/barriers
                       v
+----------------------+-----------------------------------------------+
|                    one Ubuntu workstation                            |
|                                                                      |
| C++ Command -- WheelCommand /sim/wheel/command --> Python/PyBullet   |
| C++ Command <-- WheelState  /sim/wheel/state   --- Python/PyBullet   |
|                                                                      |
| Python/PyBullet -- four output topics --> C++ Subscriber             |
|                 |                       -> optional ROS 2 Bridge      |
| all four outputs + WheelCommand -------> C++ Recorder -> MCAP        |
|                 +-> bounded snapshot -> Qt Dashboard                 |
|                                                                      |
| final session manifest -> C++ Replay -> /replay/sim/* -> ROS Bridge  |
|                        -> C++ Export -> PCD / PLY / synthetic LVX2    |
| ROS Bridge -> ROS 2 topics + TF + clock -> RViz2                     |
+----------------------------------------------------------------------+
```

- Python 是唯一的物理世界和传感器真值生产者。
- eCAL 是正式实时管道，Protobuf v2 是跨语言线协议。
- C++ Subscriber 和 Recorder 只读，不持有或修改 PyBullet 世界。
- C++ Command Tool 是独立进程，只用于测试和人工控制，不与只读客户端混合。
- `interactive` 下 Dashboard/键盘只把线速度、角速度目标经本地 control socket 交给 C++ Command Tool；真正的 `/sim/wheel/command` publisher 仍只有该 C++ 进程。控制目标使用短租约，GUI、编排器或 control socket 停止刷新后 Command 必须自动归零，不能持续重放最后一个非零目标。
- ROS 2 是 eCAL 下游可选适配层；关闭 ROS 2/RViz2 后，eCAL payload、频率和记录内容必须不变。
- Livox Viewer 2 不直接订阅 eCAL。实时观察使用 RViz2；Livox Viewer 2 使用离线导出的合成 LVX2。

六条边界固定如下：物理面只有 Python/PyBullet；实时数据面只有 eCAL Protobuf v2；进程编排和 barrier 走本机 Unix control socket；持久化面以最终 session manifest 加全部 MCAP segment 为完整单位；ROS 2/RViz2 只做可选显示；PCD/PLY/LVX2 只由已完成记录离线导出。任何组件不得绕过这些边界直接读取另一进程的 PyBullet/Qt 内存，也不得把 ROS、MCAP 或 Livox 格式反向变成物理真值来源。

### 2.2 未采用的方案

| 方案 | 不采用原因 |
|---|---|
| 原地改写 v1 | 会让阶段三日志、Python 类型和外部消费者在相同类型名下出现两种语义 |
| v1/v2 长期双发 | 会增加射线、带宽、状态、日志和测试负担，并延长兼容代码寿命 |
| ROS 2 作为主通道 | 同事明确要求 eCAL；同时会让未安装 ROS 2 的核心运行路径失效 |
| Docker-first | eCAL shared memory、宿主网络、X11、GPU 和 ROS 2 集成更复杂，不适合固定 Ubuntu 工作站的首选交付 |
| 单体 C++ 网关 | Subscriber、记录、ROS 和命令互相拖累，任一故障会扩大到整个消费端 |
| Livox Viewer 实时直连 | Livox Viewer 不原生消费项目 eCAL Protobuf，强行仿冒设备网络协议风险高且无必要 |

## 3. 进程、线程与生命周期边界

### 3.1 Python Simulator

Python Simulator 继续负责：

- 240 Hz 车辆控制、安全超时检查和物理步进。
- 在物理主线程读取关节、link、车体位姿和执行 `rayTestBatch`。
- 生成轮态、单 LiDAR、三点 RTK 和 IMU 不可变消息。
- 对 Protobuf 只执行一次 deterministic serialization，把同一份原始 bytes 写入发布日志并交给 eCAL publisher lane；禁止日志一份、typed publisher 再序列化另一份。
- 使用注册 v2 Protobuf type metadata 的 raw eCAL publisher 异步发送上述原始 bytes。
- 保留阶段三已经验证的“每 topic 一个 ready/native-in-flight owner + lane 外全局有界 latest”语义；同一话题最多 owner+latest 两帧，覆盖 latest 时精确累计 drop，正式负载要求 drop 为 0，不能改成无界 FIFO 阻塞物理线程。
- 向 Dashboard 提供有界、不可变的显示快照。

eCAL 回调不得调用 PyBullet。Dashboard、日志线程、C++ 进程和 ROS 2 Bridge 都不得持有 PyBullet client、body id、link index 或 Qt 对象。

### 3.2 C++ 核心组件

C++17/CMake 组件分为：

- `libslope_sim_client`：v2 类型、订阅生命周期、校验、代际/序列跟踪、逐话题频率和 drop 统计。
- `slope-sim-sub`：只读 CLI，可打印概要、完整 JSON 或周期统计。
- `slope-sim-record`：接收原始 eCAL Protobuf bytes 并无损写入 MCAP。
- `slope-sim-command`：独立轮子命令发送器，按车型发送 `2+0` 或 `4+2` 命令。
- `slope-sim-replay`：从完整 session manifest 指向的 MCAP 段按仿真时间回放到隔离命名空间。
- `slope-sim-export`：从完整 session manifest 导出 PCD、PLY 和合成 LVX2；单段只允许显式诊断。
- `slope-sim-ros2-bridge`：可选 ROS 2 Jazzy 适配器。

Subscriber CLI 和 Recorder 永远不发布轮子命令。Command Tool 必须先观察当前轮态的 simulation session、world generation、command generation 和命令权状态，随后才允许发送命令；正常退出时发送零命令，异常退出仍由 Simulator 的 100 ms 墙钟超时安全停车。每次 Command Tool 启动生成新的 16-byte source session UUID，并携带稳定、非空的 `source_id`，使 Simulator 能区分 Simulator 重启、命令进程重启、迟到包和命令来源。

人工控制目标固定为车体 `linear_velocity_mps/angular_velocity_rad_s`，由车型参数在 Command Tool 内唯一换算成 `2+0` 或 `4+2` 轮命令。Dashboard 保留阶段三已经验收的线速度/角速度设置和键盘方向输入，但不直接创建 eCAL publisher：编排器以 20 ms 周期刷新带递增 request id 的 `ManualTwistTarget`，Command 只接受当前同 UID、同 simulation session 的控制连接，并以 100 ms 单调墙钟租约约束目标。按键释放、窗口失焦、租约过期、连接关闭、命令权变化或 scene transition 任一发生时，目标立即归零；自动验收使用显式固定 motion recipe，不依赖 GUI 按键重复事件。

正式模式只允许一个 wheel command publisher。发现 0 个时保持等待/超时停车；发现超过 1 个时进入“命令权冲突”，立即撤销命令权并安全停车，不能用 latest-wins 猜测控制者。

### 3.3 故障隔离

- C++ Subscriber、ROS 2 Bridge 或 RViz2 退出时，Simulator 和 Recorder 继续运行。
- Recorder 在正式生产 profile 中属于必需组件；队列溢出或磁盘失败会使整个会话失败，编排器先让车辆安全停车，再有序关闭。
- 交互调试 profile 可以不启动 Recorder，但 Dashboard 和 CLI 必须持续显示“未记录”。
- 所有组件以逐话题 world generation、command generation（仅轮控）、sequence、频率、drop 和最后错误报告状态，不能用一个全局 connected 布尔值掩盖局部故障。

## 4. eCAL Protobuf v2 合同

### 4.1 版本策略

- `proto/slope_sim_interfaces.proto` 及 package `slope_sim.interfaces.v1` 冻结，只用于历史日志和兼容读取。
- 新增独立 v2 源文件，package 固定为 `slope_sim.interfaces.v2`。
- `.proto` 是 Python 和 C++ 的唯一消息源；生成物不得手改。
- 生产 Simulator 只发布 v2，不进行 v1/v2 双发。
- 删除字段必须使用 `reserved`，字段号和 enum 数值永不复用；下一次破坏性语义变更升 v3。
- 构建和运行时都验证 v2 `FileDescriptorSet` 的 SHA-256。固定 32-byte digest 同时进入每个顶层 v2 消息，不能只依赖 eCAL discovery metadata。所有 eCAL publisher/subscriber 注册完整 `slope_sim.interfaces.v2.*` type name、`encoding="proto"` 和 descriptor bytes；接收端 raw callback 先复制 payload，再核对远端 type metadata、带内 descriptor SHA-256 和本地 descriptor。peer count 只能表示“已发现”，不能单独标记“协议已验证”。Bundled C++ 进程再通过本地控制 socket 向编排器报告逐话题验证结果。Simulator、C++ SDK、Recorder 和 ROS 2 Bridge 不一致时正式模式直接失败。
- v2 继续使用用户已确认的简洁 topic 名，但 v1/v2 participant 不得在同一 eCAL domain 的这些 topic 上同时运行。Phase 0 必须用 eCAL `6.1.1` 做真实 raw publish/callback spike，证明 Python 和 C++ 都能取得并校验 type name/descriptor；同时启动 v1 peer 的隔离测试必须硬失败。若官方 API 不能可靠识别远端 metadata，则同名 topic 方案停止实施，必须先由用户重新裁决改用 `/sim/v2/...`，不得用猜测式 preflight 冒充隔离。旧 v1 WheelCommand 因缺少 v2 simulation session、双 generation、source session 和 descriptor 字段始终原子拒绝。

### 4.2 正式话题

| 方向 | eCAL topic | v2 类型 | 目标频率 |
|---|---|---|---:|
| Simulator 订阅 | `/sim/wheel/command` | `WheelCommand` | 100 Hz |
| Simulator 发布 | `/sim/wheel/state` | `WheelState` | 100 Hz |
| Simulator 发布 | `/sim/lidar/points` | `LidarPointCloud` | 10 Hz |
| Simulator 发布 | `/sim/rtk/state` | `RtkState` | 10 Hz |
| Simulator 发布 | `/sim/imu/attitude` | `ImuAttitude` | 10 Hz |

阶段三的 `/sim/lidar/front/points` 和 `/sim/lidar/rear/points` 不进入 v2 生产会话。

### 4.3 仿真会话、时间、序列和双 generation

- Simulator 每次进程启动生成新的随机 16-byte `simulation_session_id`。所有顶层输出、WheelCommand、WheelState 命令权回显、MCAP channel/metadata、控制 socket 状态和验收 oracle 都携带同一值；长度不等于 16 bytes 的消息整条拒绝。
- 连续性主键固定为 `(simulation_session_id, topic, world_generation, sequence)`；命令再加入 `command_generation/source_id/source_session_id`。Simulator 重启后即使重新出现 `world_generation=1, sequence=0`，长期 Subscriber、Recorder 和 replay 也不得与旧会话拼接。
- 所有 Simulator 输出时间戳使用统一仿真时间，不使用接收墙钟代替。
- 100 Hz WheelState 与 10 Hz LiDAR/RTK/IMU 共用一个整数采样网格：10 Hz deadline 必须恰好落在每第 10 个 WheelState deadline 上，并共享同一个 `SensorSampleContext.timestamp_ns`。同刻 WheelState、LiDAR、RTK、IMU 分别占用自己的 sequence，但 timestamp 必须逐字相等。调度器在进入该共同 deadline 前已整体超期时，只能跳过整个位点并为四个话题留下各自可审计 gap；已经进入位点后，某个传感器生成、编码或发布失败只留下该话题的 gap，其余同刻话题继续完成。两种情况都不能给三个传感器另算浮点 deadline 或用最近邻 WheelState 补齐。
- 输出话题的 `sequence` 在同一 `world_generation` 内从 0 单调递增。Simulator 在每个计划采样 deadline 前先占用序号；扫描失败、编码失败或 transport 丢帧都留下可观察缺口，禁止失败后复用旧序号。
- `world_generation` 初始为 1，只在车型/场地/world 重建成功提交时递增；递增后四个输出话题 sequence 重新从 0 开始。
- `command_generation` 初始为 1，在 world 重建、Command peer 丢失、命令权冲突或 owner source session 被撤销时递增，但不改变传感器的 world generation 或 sequence。
- Command Tool 在每个 command generation 内从 sequence 0 开始，每次合法发送尝试前占用序号。`WheelCommand.simulation_session_id`、`world_generation` 和 `command_generation` 必须分别等于最近 `WheelState`；ACTIVE 状态下 `source_id/source_session_id` 必须等于当前 owner。旧会话、旧代、错误 owner、重复或逆序 sequence 整条拒绝。
- 进入 world rebuild prepare 时立即撤销 owner、清 mailbox 并推进一次 command generation；无论 commit、abort 或 fault，旧 token 都不恢复。只有成功 commit 才推进 world generation。
- `WheelCommand.timestamp_ns` 用于可观测性和记录，推荐复制最近轮态的仿真时间；安全超时只使用 Simulator 收到命令时的单调墙钟。
- 暂停不推进仿真时钟，也不发布伪造的新物理数据；eCAL discovery 和状态观察仍可运行。

### 4.4 轮子消息

v2 保留 v1 已用字段号，并追加 sequence、双 generation、source owner session 和车型：

```text
CommandAuthorityState:
  0  COMMAND_AUTHORITY_UNSPECIFIED
  1  WAITING
  2  CLAIMABLE
  3  ACTIVE
  4  CONFLICT

WheelCommand:
  1  uint64 timestamp_ns
  2  repeated float drive_wheel_speed_rad_s
  3  repeated float steering_wheel_speed_rad_s
  4  uint64 sequence
  5  uint64 world_generation
  6  uint64 command_generation
  7  string source_id
  8  bytes source_session_id
  9  string robot_model
  10 bytes simulation_session_id
  11 bytes descriptor_sha256

WheelState:
  1  uint64 timestamp_ns
  2  repeated float drive_wheel_speed_rad_s
  3  repeated float steering_wheel_angle_rad
  4  uint64 sequence
  5  uint64 world_generation
  6  uint64 command_generation
  7  string robot_model
  8  bytes simulation_session_id
  9  bytes descriptor_sha256
  10 CommandAuthorityState command_authority_state
  11 string command_owner_source_id
  12 bytes command_owner_source_session_id
  13 uint32 command_peer_count
```

数组顺序继续使用阶段三稳定车型语义：

- 三种差速车型：驱动轮 `[left, right]`，转向数组为空。
- 主动转向四轮车：驱动轮 `[front_left, front_right, rear_left, rear_right]`，转向轮 `[front_left, front_right]`。
- 数组长度、有限值和机械限位任一不合法时原子拒绝，禁止部分应用。
- Command Tool 从最新 `WheelState.robot_model` 复制车型名；命令车型与当前 runtime 不同则整条拒绝。该字段也让独立 C++ Subscriber 不依赖目标机外部配置猜测数组语义。
- `source_session_id` 必须恰好 16 bytes；`source_id` 必须为 1..64 个 ASCII 字节，只允许 `[A-Za-z0-9._-]`。`descriptor_sha256` 必须恰好 32 bytes 并与本地 descriptor 一致。
- 命令权状态机固定为 `WAITING(peer_count=0, no owner) -> CLAIMABLE(peer_count=1, no owner) -> ACTIVE(peer_count=1, owner) -> CONFLICT(peer_count>1, no owner)`。`command_peer_count` 保存原始非负整数，不能压缩成布尔值。
- 只有 CLAIMABLE 状态收到的第一条完整合法、当前代消息可以绑定 owner 并进入 ACTIVE。ACTIVE 中其他 source/session 的消息立即拒绝，撤销 owner、清 mailbox、推进一次 command generation，再回到由最新 peer count 决定的状态；错误消息不得刷新 100 ms 有效命令墙钟。
- peer count 从 1 变成 0 或大于 1 时只对该状态转换推进一次 command generation 并安全停车；持续 WAITING/CONFLICT 不得每次 poll 重复推进。CONFLICT 恢复为 1 后进入 CLAIMABLE，新 owner 必须先读取 WheelState 中的新 generation 再认领。

### 4.5 LiDAR 消息

点字段保持 Livox ROS Driver 2 `CustomPoint` 风格，但明确属于合成数据：

```text
LidarPoint:
  1  uint32 offset_time_ns
  2  float x
  3  float y
  4  float z
  5  uint32 reflectivity
  6  uint32 tag
  7  uint32 line

LidarPointCloud:
  1  uint64 timebase_ns
  2  string frame_id
  3  uint32 point_num
  4  uint32 lidar_id
  5  repeated LidarPoint points
  6  uint64 sequence
  7  uint64 world_generation
  8  bytes simulation_session_id
  9  bytes descriptor_sha256
```

合同约束：

- `frame_id="lidar_link"`，`lidar_id=1`。
- 坐标单位为 m，右手系 `+X` 前、`+Y` 左、`+Z` 上。
- 只把有效表面命中写入 `points`；`point_num == len(points)`，因此点数随场景和遮挡变化。
- `offset_time_ns` 按候选射线时序单调递增且小于 100 ms。它表达扫描顺序，不表示本版本模拟了运动畸变。
- 第一版 `reflectivity=0`、`tag=0`。地形、静态障碍和动态障碍类别不得写入 Livox `tag`。
- `line` 限制为 `0..15`，只表示合成扫描线索引，不声明与 MID-360 硬件通道一一对应。
- `reflectivity/tag/line` 的 wire 类型虽为 `uint32`，业务校验范围固定为 `0..255`，以便无损转换到 Livox ROS 消息。

### 4.6 RTK 消息

RTK 使用固定字段，避免 repeated role 出现缺失、重复或下标猜测：

```text
Point3d:
  1  double x_m
  2  double y_m
  3  double z_m

RtkState:
  1  uint64 timestamp_ns
  2  uint64 sequence
  3  uint64 world_generation
  4  string frame_id
  5  Point3d left
  6  Point3d center
  7  Point3d right
  8  double heading_rad
  9  bytes simulation_session_id
  10 bytes descriptor_sha256
```

- `frame_id="world"`，三点全部是世界坐标，单位 m。
- `heading_rad` 归一化到 `[-pi, pi)`；它是 `RIGHT -> LEFT` 三点 RTK 基线在世界 `XY` 平面的投影方位减 `pi/2`。该量与车体局部左轴使用同一投影定义，在非零 roll/pitch 时一般既不等于车体前向水平投影角，也不等于 ZYX Euler yaw，因此不得命名或直接解释为 yaw。
- Proto3 解码必须逐项检查 `has_left()/has_center()/has_right()`；三点坐标和航向必须有限，`LEFT-RIGHT` 水平基线长度必须大于 `1e-6 m`。任一缺失、非有限或退化时整帧拒绝，不能把缺失子消息的默认零值当作真实 RTK。
- RTK 在本项目中是三个理想空间参考点的仿真真值，不是 LiDAR，也不模拟经纬度、卫星、基站、电离层或 GNSS 噪声。

### 4.7 IMU 消息

阶段四不虚构未要求的加速度、角速度和协方差，继续输出车体姿态：

```text
ImuAttitude:
  1  uint64 timestamp_ns
  2  double roll_rad
  3  double pitch_rad
  4  uint64 sequence
  5  uint64 world_generation
  6  string frame_id
  7  bytes simulation_session_id
  8  bytes descriptor_sha256
```

- `frame_id="base_link"`。
- roll 为车体绕前向 `+X` 的横滚，pitch 为绕左向 `+Y` 的俯仰，单位 rad。
- 值来自同一仿真时刻的 `base_link` 世界姿态。

所有四个 Simulator 输出都要求 `simulation_session_id` 恰好 16 bytes、`descriptor_sha256` 恰好 32 bytes 且与本地冻结 descriptor 一致；这些校验先于 sequence/generation 连续性统计，协议错误不得污染正常 gap 计数。

## 5. 单 MID-360 风格 LiDAR

### 5.1 安装和模型

阶段四为四车型各提供独立的 v2 URDF，它们都只保留一个 `lidar_link`；阶段三 v1 使用的四个 legacy URDF 原样保留双 mount，避免冻结 v1 资源在同一路径下改变语义。固定外参为：

```text
base_link -> lidar_link
translation = (0.0, 0.0, 0.105) m
rotation    = identity
```

这表示雷达位于车辆平面几何中心上方 105 mm，朝向与 `base_link` 一致。射线过滤必须排除本车全部 link，不能把底盘或车轮返回为环境点。

### 5.2 扫描 profile

| profile | 候选射线/帧 | 频率 | 用途 |
|---|---:|---:|---|
| `realtime_mid360` | 5,760 | 10 Hz | 正式实时 eCAL、GUI 和记录 |
| `offline_dense_mid360` | 20,000 | 10 Hz 仿真时标 | 离线高密数据生成，不要求实时墙钟 |

共同参数：

- 水平视场 360°，方位角使用半开区间，禁止 `-180°/+180°` 重复方向。
- 垂直视场近似 `-7°..+52°`。
- 有效距离 `0.1..40 m`。
- 扫描方向和相位按 profile、`scan_seed` 和 sequence 确定性生成；相同输入必须逐射线复现。
- 方向序列使用二维 R2 低差异序列。令 `g` 为 `g^3=g+1` 的正实根，`alpha=(1/g, 1/g^2)`；一帧第 `i` 条候选射线使用全局序号 `k=scan_seed+sequence*N+i`，再由 `frac(0.5+k*alpha)` 分别映射到半开区间 `[0, 360°)` 和 `[-7°, +52°)`。`line=floor(16*v)`，最大钳制为 15；`offset_time_ns=floor(i*100,000,000/N)`。
- 该序列只提供确定性、均匀覆盖和跨帧不完全重复的近似效果，不声称复现 MID-360 私有扫描规律。`scan_seed` 必须写入场景、MCAP metadata 和验收报告。

### 5.3 原子扫描

- 实时 profile 在一个发布 deadline 冻结一次 `lidar_link` 位姿，并以一次 `rayTestBatch` 完成 5,760 条射线。
- 20,000 射线超过单批安全预算时可以分批，但所有批次使用同一冻结位姿，批次之间不得推进物理世界。
- 任一批次失败时整帧失败，不发布部分点云。
- 一帧所有点使用同一位姿，因此第一版不模拟运动畸变；`offset_time_ns` 仅保留真实消息风格和后续扩展空间。
- 阶段三每物理步额外执行的 31 射线摘要必须删除，或严格由最近一次正式单雷达点云派生，禁止第二套世界射线源。

### 5.4 Dashboard 显示边界

RViz2 是正式三维点云观察器。Dashboard 只显示从完整点云派生的有界显示副本：

- 按距离着色，不使用 wire `tag` 作为物体类别。
- 允许体素或屏幕像素降采样，但 eCAL 和 MCAP 中的原始有效点不得被显示策略删减。
- 显示容器具有固定尺寸，不能随点数、坐标范围或文本变化而挤压上半区。
- 图表刷新上限为 2 Hz；接口状态最多 60 Hz；显示路径不得反向阻塞 10 Hz 扫描。

## 6. 三点 RTK 几何

### 6.1 二轮差速车型

- `LEFT`：左驱动轮轴中心的世界坐标。
- `CENTER`：左右驱动轮轴中心的中点。
- `RIGHT`：右驱动轮轴中心的世界坐标。

### 6.2 主动转向四轮车型

- `LEFT`：左前、左后轮轴中心的均值。
- `CENTER`：四个轮轴中心的均值。
- `RIGHT`：右前、右后轮轴中心的均值。

车型注册表必须提供稳定 link 名；传感器不得依赖临时 PyBullet link index。三点每帧从同一物理时刻读取。`heading_rad` 由 `RIGHT -> LEFT` 的水平基线方位减 `pi/2` 得到，并与同帧 `base_link` 局部左轴在世界 `XY` 平面的投影按同一公式所得独立 oracle 交叉验证；RTK 基线或车体左轴水平投影退化、两者误差超限或出现非有限值时整帧失败。

跨语言几何的唯一发行资源为 canonical `models/robot_models.yaml`。它由确定性 generator 从四车型注册表和已验证 URDF 生成，按固定 model id/键顺序记录 base/RTK link、轴心到 `base_link` 以及 `base_link -> lidar_link` 外参；`--check` 必须逐 byte 证明受版本控制文件未漂移并记录 SHA-256。Python runtime、C++ 世界坐标导出、ROS TF 和发行包只消费这一份资源，不能各自硬编码第二套外参。

Dashboard 的 RTK 页面显示 CENTER 的时间曲线、三点最新数值表和俯视几何，不在单张图中同时堆叠九条坐标曲线。

## 7. 场景 schema 与 v1 迁移

- 阶段四场景使用 `schema_version: 2`。`robot`、`terrain` 和 `obstacles` 沿用 v1 的严格结构；`sensors` 固定为以下形状，禁止未知键：

```yaml
schema_version: 2
robot:
  model: df_back
terrain:
  terrain_model: golf_heightfield
  slope_deg: 0.0
  golf_seed: 23
  golf_relief: high
obstacles: []
sensors:
  lidar:
    frame_id: lidar_link
    lidar_id: 1
    parent_link: base_link
    position_m: [0.0, 0.0, 0.105]
    orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
    profile: realtime_mid360
    scan_seed: 0
  rtk:
    frame_id: world
    geometry: wheel_axle_triplet_v1
  imu:
    frame_id: base_link
```

`position_m`、orientation、frame/id、RTK geometry 和 IMU frame 都是固定合同，场景文件保留它们是为了自描述和哈希，不允许用户改成另一套外参。profile 只能从发行清单中的 `realtime_mid360`、`offline_dense_mid360` 选择；`scan_seed` 为有界无符号整数。
- v1 场景仍可由冻结读取器打开，但不能被生产 v2 runtime 静默解释。
- 显式转换工具把 v1 转为 v2，并在结果和控制台报告：“前后两个 180° LiDAR 已替换为一个几何中心 360° LiDAR”。
- 转换保留车型、地形、障碍物和非冲突配置；无法无损转换的未知字段直接报错。
- v2 导出后再导入必须幂等；场景哈希进入 MCAP 和发行验收报告。

## 8. C++、ROS 2 与显示接口

### 8.1 C++ SDK

`libslope_sim_client` 提供：

- 五个 v2 话题的强类型订阅/发布封装。
- 明确的 `start/poll/stop` 生命周期和幂等关闭。
- world/command generation、sequence 连续性、频率、payload hash、解析错误和 reconnect 统计。
- CMake package config、头文件、共享库、v2 `.proto`、`FileDescriptorSet` 和最小示例。
- Python/C++ 端统一使用带 v2 type metadata 的 raw subscriber。eCAL 6.1.1 raw receive callback 固定提供 `(publisher_id, data_type_info, data)`：native callback 直接从本帧 `data_type_info` 复制远端完整 type name/encoding/descriptor，并复制原始 wire bytes、`publisher_id.topic_id` 中的 EntityId、eCAL `send_timestamp/send_clock`、接收单调时钟和 Unix epoch 接收墙钟；worker 再计算 hash、验证并解析为生成的 Protobuf 类型。monitoring snapshot 只按 topic 名和 exact peer count 原子判定 waiting/pending/verified/conflict，不能替代本帧 metadata，也不能用本地 subscriber 声明冒充远端信息；若 callback 与 monitoring 需要诊断关联，比较的是 monitoring `Topic.topic_id` 与 `publisher_id.topic_id.entity_id`。单调时钟只用于本进程 deadline/延迟，epoch 墙钟进入配对 record metadata；禁止在 native callback 调 monitoring、对大点云做 SHA-256，或把 typed callback 得到的对象重新序列化后冒充原始 payload。
- 用户回调只接收拥有明确生命周期的不可变解析结果；用户回调变慢时不得阻塞 eCAL native callback。

只读 CLI 和 ROS 2 Bridge 使用有界 owner+latest 消费并报告 superseded/drop；Recorder 使用完整 FIFO。三者不得共享一个会因慢 RViz2 而拖慢记录器的队列。

### 8.2 ROS 2 Bridge

正式目标为 Ubuntu 24.04 LTS + ROS 2 Jazzy。Bridge 显式接收 `input_prefix/output_namespace`：live 固定 `/sim` -> `/slope_sim`，replay 固定 `/replay/sim` -> `/replay/slope_sim`，禁止把 replay 输出映射回生产 namespace。Bridge 输出：

- `livox_ros_driver2/msg/CustomMsg`：按锁定官方 `.msg` hash 逐字段映射。`header.stamp` 由 `timebase_ns` 精确转换，`header.frame_id="lidar_link"`，`timebase=timebase_ns`，`point_num=points.size()`，单个合成雷达固定 `lidar_id=1`，reserved 三字节为零；每点保留 `offset_time/x/y/z/reflectivity/tag/line`。
- `sensor_msgs/msg/PointCloud2`：包含 `x/y/z/intensity`，并增加派生 `range` 供 RViz2 按距离着色。
- 项目自有的 wheel、三点 RTK 和 IMU ROS 消息，语义与 eCAL v2 一致。
- 静态 TF：`base_link -> lidar_link`。
- 动态 TF 与点云提交：key 固定为 `(simulation_session_id, world_generation, timestamp_ns)`。LiDAR/RTK/IMU 任一先到建立最多 64 个 10 Hz anchor，100 Hz WheelState 使用独立 256 项 exact-time cache；四种消息任意到达顺序都在 anchor 三种传感器与同 key WheelState 齐全后恰好提交一次。anchor 墙钟 TTL 为 2 秒，缺槽、重复、容量淘汰和过期都进入 Bridge health 并使正式门禁失败；没有任何传感器 anchor 的额外 WheelState 到期只计正常 `wheel_unmatched_expired` 诊断，不能让 90 Hz 非采样时刻轮态填满 anchor。session/generation 前进清理旧缓存，绝不最近邻、补零或跨 generation 配对。
- RTK 的 `heading_rad` 是车体左轴水平投影减 `pi/2`，在非零 roll/pitch 时不等于 ZYX Euler yaw。C++ 导出器和 ROS Bridge 必须用同一公开纯函数恢复姿态：令 `lateral_azimuth = heading + pi/2`，`yaw_zyx = wrap(lateral_azimuth - atan2(cos(roll), sin(pitch) * sin(roll)))`，再构造 `Rz(yaw_zyx) * Ry(pitch) * Rx(roll)`；RTK 水平基线退化时前置协议已经拒绝。随后按 `WheelState.robot_model` 的 canonical 轴心到 `base_link` 外参恢复 `world -> base_link`。跨语言测试必须用独立四元数/旋转矩阵 oracle 覆盖非零 roll、pitch、yaw，禁止把 heading 直接塞进 Euler yaw。
- 仿真时钟：live 固定 `/slope_sim/clock`，replay 固定 `/replay/slope_sim/clock`，RViz2 启动时分别把 ROS 特殊 `/clock` remap 到对应 topic。时钟使用最新 Simulator 仿真时间，暂停保持，replay 由回放器推进；禁止 live/replay 共用全局 `/clock` 后仍宣称命名空间隔离。

Bridge 不修改或补造 eCAL payload，raw payload/hash 在转换前后必须保持不变。可选 Jazzy overlay 默认从同时锁定 commit/checksum/license 的完整官方 `livox_ros_driver2 + Livox-SDK2` 源码链构建，并冻结 `CustomMsg/CustomPoint` 的规范化 `ros2 interface show` hash；未联网核验 package 边界或未在 Ubuntu 24.04 + Jazzy 构建通过前，不得假定 message package 能独立抽取，也不得进入 Bridge 实现。目标机不需要预装 Livox package。Bridge/RViz2 失败可以单独重启，不能让 Simulator 或 Recorder 退出。rosbag2 MCAP 可以作为 ROS 侧辅助记录，但不能替代接收端原始 eCAL MCAP。

ROS 输出名固定为 `<namespace>/lidar/custom`（Livox）、`<namespace>/lidar/points`（PointCloud2）、`<namespace>/wheel/state`、`<namespace>/rtk/state`、`<namespace>/imu/attitude`、`<namespace>/rtk/markers`、`<namespace>/trajectory` 和 `<namespace>/clock`；live/replay 只替换 namespace，不复用同名全局输出。

### 8.3 RViz2

发行包提供可直接打开的 RViz2 配置：

- 固定坐标系和 TF 正确。
- 点大小、衰减和按 `range` 着色具有适合室内/场地观察的默认值。
- 用户能直观看出地面、墙面、坡体和障碍物表面。
- 可选实时显示，也能使用 rosbag2 或隔离 eCAL replay 回放。

## 9. MCAP、回放和导出

### 9.1 原始 MCAP

MCAP 是接收端正式记录格式，不新增私有 `.slog2`。MCAP `Message` 不提供任意逐消息键值 metadata，因此每个业务 channel 都有唯一配对的 `/_slope_sim/record-metadata<business-topic>` channel；业务 message data 始终是未经包装的原始 eCAL payload，紧邻的 metadata message 使用 `slope_sim.record.v1.RecordMetadata` 保存动态字段。pair 两条记录共用仿真 `publishTime` 和接收墙钟 `logTime`；MCAP 内建 32-bit sequence 固定为 0，业务的完整 64-bit sequence 只从 payload/metadata 验证，避免长会话截断或回绕被误当成身份。每个文件保存：

- 原始 eCAL Protobuf bytes，不经过 ROS 消息二次编码。
- 完整 `FileDescriptorSet` 和 descriptor SHA-256。
- 由 raw/metadata pair 一一绑定的 topic、完整 Protobuf type name、simulation session、带内 descriptor SHA-256、payload SHA-256、仿真发布时间、eCAL send timestamp/send clock、接收端墙钟、scene revision、对应 canonical YAML attachment SHA-256、sequence、world generation，以及轮控消息的 command generation/source owner session。每个业务 MCAP Schema 固定 `name=<完整 v2 type>`、`encoding="protobuf"`、`data=<完整 FileDescriptorSet bytes>`；业务 Channel 固定 `message_encoding="protobuf"`，静态字符串 metadata 固定为 `ecal_type_name`、`ecal_encoding="proto"`、64 位小写十六进制 `descriptor_sha256_hex` 和 32 位小写十六进制 `simulation_session_id_hex`。Reader 必须把 Schema、Channel、逐消息 metadata、session manifest 和当前 runtime descriptor 交叉验证，并拒绝缺 pair、多 pair、非相邻、错序、静态 metadata 缺失/格式错误或任何身份/hash 不匹配。
- 场景/配置哈希、扫描 profile、runtime manifest SHA-256、release 版本、Git SHA 和主机信息。runtime manifest 是启动当前 Recorder 的安装树内 `share/slope-sim/runtime-manifest.json` 的规范 JSON；开发安装和正式发行使用同一 schema，但 `build_kind` 分别为 `development/release`。
- Zstd chunk、索引和 CRC。

每次会话开始以及 scene revision 生效时，把规范化 schema v2 YAML 作为 MCAP attachment 写入，名称包含 revision、world generation 和生效仿真时间；每条 `RecordMetadata.scene_attachment_sha256` 保存该 attachment 内容的 32-byte SHA-256。初始 scene revision 固定为 1、world generation 为 1、`effective_timestamp_ns=0`；以后每次已提交的障碍物增删、场地/车型重建或传感器 profile 变更都把 revision 精确递增 1。

场景变更使用一个不会留下“已记录但未生效”attachment 的事务：先冻结 Simulator 四个输出，并通过 control socket 让 Command 归零、冻结 publisher、回报最后 identity；随后在物理主线程完成候选 world 操作。物理操作失败时回滚旧世界，不发送新 attachment，旧 command token 仍作废并通过新 command generation 重新认领。物理操作成功后才推进 world/revision，以严格大于五话题既有时间水位的下一共同采样位点作为 effective time，把最终 canonical YAML 交给 Recorder；Recorder 重算 SHA、验证连续 revision/generation/time、持久化并 ACK 后，Simulator 才在该 effective time 恢复四输出，Command 在读到新 WheelState 后重新认领。attachment ACK 失败时新世界保持业务不可见并使正式会话失败，不能无记录继续运行。暂停或同一事务中连续到达的编辑只合并成一个最终 canonical scene；旧 revision 覆盖 `[effective_i, effective_{i+1})`，边界帧属于新 revision。

bundled Command 只复制最近 WheelState 的仿真时间，但五个 eCAL topic 的到达顺序不受 control socket 或跨 topic 顺序保证。Recorder 因而不得在 command 先于对应 WheelState 到达时立即误判“未来时间”：该 command 连同原 reservation 在其原 `record_order` 上标为 `DEFERRED`，直到同 session/world 的 WheelState 水位达到其 timestamp 后再原位选择 scene interval 并转为 `READY`；真正未来、跨代或在 drain 时仍未解析的 command 使会话硬失败。读取端按时间区间选择唯一已提交 attachment，重算其 SHA 并与逐消息 metadata 比较，由此建立消息到 scene revision 的唯一映射，不修改原始 payload。

Recorder 的 native callback 先从跨 raw、ordered-commit disposition 和 rotation holding 共用的 reservation ledger 预约容量，成功后在同一临界区取得从 0 连续递增的 `record_order`，再只复制原始 bytes 和远端 metadata 进入一个全局 raw FIFO；预约或复制失败立即硬失败。单一 validation worker 按 raw FIFO 顺序在同一有界 `OrderedCommitLedger` 为每个已分配 order 原位设置唯一 `READY | REJECTED | DEFERRED` disposition。`READY` 持有不可拆分的 `(record_order, raw payload, deterministic RecordMetadata)` pair；`DEFERRED` 持有等待 WheelState 水位的 command 及原 reservation，辅助 unresolved 索引只能非 owning 地引用 order；`REJECTED` 在连续 frontier 尚被前方 DEFERRED 阻塞时仍以缩小后的审计 marker 占一个 slot，不能提前释放后无界累积。`next_commit_order`/`settled_frontier` 指向尚未写入或审计跳过的最小 order：writer 只在该 order 为 READY 时写 pair、为 REJECTED 时记账并跳过，遇 DEFERRED 或缺 disposition 就停止；后续完成项继续由同一 8192 slot/512 MiB 总账保存，绝不能从数字缺口猜 rejected。WheelState 水位到达后只把原 DEFERRED order 转 READY，所以“order 0 command deferred、1/2 ready、3 WheelState 解锁”的最终 MCAP 顺序仍为 0、1、2、3。owned bytes 包含 payload、远端 metadata、待写 record metadata 和审计 marker；callback 对 record metadata 先保守预留 4096 bytes，形成 pair 后缩为实际大小，metadata 超过上限或任一 owning 队列绕过 ledger 都硬失败。禁止覆盖旧记录或阻塞 eCAL native callback。

MCAP chunk 固定以 Zstd 压缩，未压缩目标大小 16 MiB；单段文件达到 4 GiB 或 30 分钟仿真时间时，Recorder 在 ordered-commit 锁内捕获 `rotation_start_order` 并暂停跨过该 order 的段提交，通过独立 `SegmentCutRequest/SegmentBarrier` 取得五 topic exact fence，以这五个 fence pair 中最大的 `record_order` 作为唯一分段点。只有当前 frontier 到 cut 的全部 order 已为 READY/REJECTED，writer 才依次写/跳过并推进；任一 DEFERRED 阻止轮转或 drain 越过它。`<= cut` 的 READY pair 原序写当前段，`> cut` 原序留给下一段；每段实际 last identity 必须不早于请求 fence。这样不同 topic 异步到达不会破坏全局记录顺序，普通轮转也不进入 DRAINING/FINALIZED。

记录前检查目标卷剩余空间至少为 `10 GiB + 当前队列字节 + 2 * chunk目标大小`，运行中每秒复查；不足即进入 fatal，不自动删除任何旧记录。活动文件使用 `<session>-<segment>.mcap.partial`，正常完成时依次写 summary/index/CRC、flush、`fsync(file)`、close、原子 rename 为 `.mcap`、`fsync(parent directory)`。每段完成后原子更新诊断用 `<session>.manifest.pb.partial` checkpoint，其中按序记录段号、文件 SHA/大小、首尾五话题 fence、session/descriptor，以及每份 scene attachment 的 revision、world generation、生效时间、附件名和 YAML SHA；首尾 fence 集合都必须恰好包含五个正式 topic 各一次，使用固定 topic 顺序编码，且 `size_bytes` 必须等于实际文件大小。会话 manifest 的 record schema version 固定为 1，并保存启动安装树 runtime manifest 原始 bytes 的 32-byte SHA-256；会话最终以同样 durability 顺序把 checkpoint 原子完成为 `<session>.manifest.pb`。

manifest 中 `SegmentEntry.file_name` 和 attachment name 只能是规范 basename，禁止空值、绝对路径、斜杠、`..`、控制字符和重复；Reader 以已 `resolve(strict=True)` 的 manifest 父目录为根，用不跟随符号链接的方式打开普通文件并在读取前后核对 inode/size/hash。正式 replay/export 只接受最终 manifest，并拒绝版本 0/未知、runtime manifest digest 缺失/长度错误、路径逃逸/符号链接/非普通文件、缺段、重复、重叠、gap、fence 缺/重/未知 topic、first/last topic 集不同、文件大小或 hash 错、attachment 缺失/hash 错或任何 `.partial`；单 `.mcap` 只允许显式诊断。崩溃或磁盘错误保留 `.partial` 供诊断，恢复工具只能另存修复结果，不能把未经完整校验的 partial 原地标成正式记录。

### 9.2 回放

- 默认发布到独立 `/replay/sim/...` 命名空间，避免与实时话题混合。
- Reader 只有在完整验证 session manifest、业务 MCAP Schema/Channel、逐消息 `RecordMetadata` 和当前 runtime descriptor 后，才为每个源 topic 产出不可变 Replay publisher contract。Replay 在第一次发送前必须为映射后的每个 `/replay/sim/...` topic 注册原业务消息的完整 `slope_sim.interfaces.v2.*` type name、原 eCAL `encoding="proto"` 和逐字节相同的完整 `FileDescriptorSet`；descriptor bytes 的 SHA-256 必须同时等于 Channel、session manifest、带内消息和 runtime descriptor 的 digest。同一源 topic 在任一 segment 出现不同合同、metadata 缺失或任一值不匹配时，必须在创建 publisher 前拒绝整个回放。
- 回放只改变 topic namespace 和发送 deadline，业务 payload 直接使用已验证的 MCAP raw bytes 调用 raw publisher，禁止 parse 后重新序列化。type name、encoding 或 descriptor 不得从文件名、topic 名、生成类型或本地默认值猜测；ROS Bridge 对 live/replay 使用同一个严格 metadata gate，合法 replay 必须通过，缺失或错误的任一 metadata 必须在 Protobuf parse 前拒绝。
- 默认不回放 `/sim/wheel/command`，防止历史命令驱动车辆。
- 显式危险模式允许把历史 WheelCommand 发送到隔离的 `/replay/sim/wheel/command`，但仍必须携带该消息原始、完整且已验证的 v2 publisher metadata，不能因危险模式放松协议门。
- 支持原速、暂停、单步和倍率回放；顺序以仿真时间和文件记录顺序共同确定。
- 回放前验证 session manifest、全部 segment SHA/首尾 fence、schema/hash/CRC 和 raw/metadata pair；不兼容、缺段或损坏直接拒绝。

### 9.3 PCD 与 PLY

- 每帧文件名包含 world generation、sequence 和仿真时间。
- 文件头明确 `lidar_link` 坐标系、单位 m 和有效点数。
- 在格式允许时保留 `offset_time_ns/reflectivity/tag/line` 点属性；不支持的查看器忽略扩展属性，不能反向改变原始 MCAP。
- PCD/PLY 只包含原始有效命中，不包含 LVX2 对齐产生的填充点。
- 单帧 `lidar_link` 导出不需要姿态配对；世界坐标合并必须对每个 LiDAR 帧找到相同 simulation session、world generation 和完全相同 `timestamp_ns` 的 RTK、IMU、WheelState。位置使用 RTK CENTER，姿态使用 8.2 节冻结的 projected-lateral heading + IMU roll/pitch 恢复公式，再按 `robot_model` 的固定轴心中心到 `base_link` 外参和 `base_link -> lidar_link` 外参变换。任一消息缺失、重复、非有限、车型未知或时间不完全相等时拒绝该次世界合并，禁止最近邻猜测、直接把 heading 当 Euler yaw 或跨 generation 插值。

### 9.4 合成 LVX2

- 使用 Livox LVX2 2.0 布局、MID-360 device type、50 ms frame 和每个 Type 1 数据包 96 点。
- 合成 LVX2 是面向 Livox Viewer 2 的有损显示格式，不是 MCAP 的无损替代：坐标量化到毫米，尾包会重复填充，并且 LVX2 不保留 v2 的 `line`、精确 `offset_time_ns`、simulation session、world generation 和 sequence。
- 10 Hz 点云按 `offset_time_ns` 划入两个 50 ms LVX2 frame。
- 每个源有效点按时序完整写入且不丢弃。最后不足 96 点的数据包重复最后一个有效表面点补齐；这只增加重合点，不制造原点或新表面。sidecar 逐包记录源有效点数、重复填充数、源 MCAP hash 和 `synthetic=true`；没有有效点的区间不生成数据包。PCD/PLY 和 MCAP 不含这些重复点。
- LVX2 Type 1 点记录没有 `offset_time_ns` 和 `line` 字段；数据包 timestamp 使用包内第一源点的 `timebase_ns+offset_time_ns`，原始 offset/line 只在 MCAP 和 sidecar 索引中保留，禁止宣称 LVX2 可无损承载全部 v2 点字段。
- 导出器必须能重新读取自己生成的文件，并用 sidecar 恢复精确有效点计数。
- 正式人工门禁必须在 Livox Viewer 2 中实际打开并观察点云。

官方 `Indoor_sampledata.lvx2` 仅作为内部格式与 Viewer 验证基准。其公开下载页未单独声明可再分发许可证，因此不得进入 Git 或发行包。

## 10. 实时性和高尔夫场地优化

### 10.1 调度上限

| 子系统 | 频率/上限 |
|---|---:|
| PyBullet 物理与命令超时 | 240 Hz |
| 轮态 | 100 Hz |
| LiDAR / RTK / IMU | 10 Hz |
| eCAL discovery 与组合状态 | 20 Hz |
| 跟随相机 | 最多 30 Hz |
| Qt 状态刷新 | 最多 60 Hz |
| Dashboard 图表重绘 | 最多 2 Hz |

全部实时循环使用共享绝对 deadline。超期帧只 `sleep(0)` 让出执行权，不叠加固定正延时，也不突发补跑已经错过的 UI/discovery 周期。100 ms 命令超时仍在每个 240 Hz 物理帧检查。

### 10.2 优化方法

先分别测量 flat/golf、DIRECT/GUI、Dashboard 开关、LiDAR 开关、日志开关的分段 p50/p95/p99，再依据证据优化：

- 相机、Qt 状态和 Matplotlib 分别限频。
- 图表只保存有界历史，对显示数据降采样并保持稳定坐标范围，避免新样本导致容器或坐标轴反复跳动。
- LiDAR 方向、相位、终点和不随位姿变化的数据预计算；生产后端继续使用紧凑 indexed hit 和批量逆变换。
- Dashboard 不复制完整三维云，只持有有界显示样本。
- 高尔夫场地优先降低视觉 mesh 细节或增加视觉 LOD；只有 profiling 证明碰撞 heightfield 是主要瓶颈后，才允许实验碰撞分辨率。

固定轻量化措施完成后必须先跑代表性 `golf_heightfield + 20 障碍物 + 5,760 射线 + GUI + Dashboard + logging` 隔离组合；未达到正式 `sim/wall`、GUI event gap 和 Dashboard draw p95 门槛时，Task B 不得结束。继续按 profiler 最大 p95 段一次只引入一项有界优化，优先级为视觉 mesh/LOD、相机/Qt/Matplotlib 无效重绘、点云显示复制，再到经独立真值保护的碰撞实验；每一步都重跑相同矩阵和真值 fixture，不能把“已测量”写成“已优化”。

任何碰撞分辨率调整都必须重新验证坡度、高程、车轮接触、车辆轨迹、RTK 和 LiDAR 真值。GeForce 可改善 OpenGL/RViz2 绘制，但 PyBullet 物理和 ray test 主要仍是 CPU 工作，不能把显卡升级当作唯一优化方案。

离线 20,000 射线 profile 不要求 `sim/wall` 实时，但必须保持同一冻结世界、完整点数和确定性导出。

## 11. 异常处理

| 异常 | 规定行为 |
|---|---|
| 正式模式无法初始化 eCAL | 启动失败并清理，不能静默降级 local |
| 已有正式 `/sim` 会话占用主机锁 | 新编排器在创建任何 eCAL participant 前失败并报告现有 PID/session，不允许两个随机 session 互相污染 |
| 输出话题没有订阅者 | 继续发布并标记该话题“等待对端”，不污染其他话题状态 |
| Command peer 消失 | 清除命令权；100 ms 内安全停车；重连后必须使用新 command generation |
| 同时出现多个 Command peer | 标记命令权冲突、递增 command generation 并安全停车，直到恢复为唯一 peer |
| 非法 Protobuf/数组/数值/代际 | 整条拒绝并计数，不刷新有效命令墙钟 |
| LiDAR 任一批次失败 | 整帧不发布，下一个 10 Hz deadline 重试 |
| ROS 2 Bridge/RViz2 失败 | Simulator、eCAL 和 Recorder 继续；Bridge 可单独重启 |
| 正式 Recorder 队列满/磁盘失败 | 会话失败、车辆安全停车、保存错误并有序关闭 |
| 交互模式未记录 | 允许继续，但 GUI/CLI 明确显示“未记录” |
| 场景/车型重建失败 | 回滚原世界；旧 command token 不得提交到恢复或新世界 |
| PCD/PLY/LVX2 导出失败 | 原始 MCAP 保持只读完整，可修复配置后重试 |
| 关闭期间晚到回调 | world/command generation 与 token 校验后丢弃，不访问已释放 native 资源 |

eCAL session attach 和周期状态读取继续遵守阶段三已验证的“先 `poll_peer_state()`，再读取快照”；关闭先等待在途 discovery 返回，再释放 publisher/subscriber 和 participant。

## 12. Ubuntu 发行包

### 12.1 正式目标

- Ubuntu 24.04 LTS、amd64。
- Eclipse eCAL 固定为 `6.1.1`：Python Simulator 只使用 PyPI 官方 `eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl`，其 size `6905517`、SHA-256 `57a23af7d83c077c04f01852db13f8cda7686a052d41659fafcbe6b3dbe9f6bc`、PEP 425 tag、RECORD、license/NOTICE 和 ELF inventory 都由独立 canonical wheel manifest 冻结；C++ 组件使用同 tag 源码构建出的 SDK。pip wheel 只提供 Python 扩展和私有 core library，不能冒充 C++ headers/CMake package；升级任一侧 eCAL 都必须作为独立 wire 兼容性变更重新验证。
- Protobuf 版本线固定为 release `v33.6`：Python runtime `6.33.6`、`protoc 33.6`、C++ `libprotobuf 33.6`。当前开发环境的 `grpc_tools.protoc 31.1`、PATH 中 `protoc 35.1` 和历史 v1 pb2 的 `6.31.1` 只用于解释既有基线，均不得生成 v2。v1 生成物保持冻结；v2 使用发行构建器内独立锁定的 `protoc 33.6` 同时生成 Python/C++/descriptor set。
- C++ 构建基线固定为 GCC `13`、C++17、`_GLIBCXX_USE_CXX11_ABI=1`、CMake `3.28.x`。最小 eCAL C++ SDK 关闭 apps/Qt/HDF5/Curl/FTXUI/samples/tests/Python/C#/C binding，优先以 `ECAL_USE_PROTOBUF=OFF` 构建 raw core，使项目消息只链接 `libprotobuf 33.6`；若该选项不能提供所需 monitoring/type metadata，则 eCAL core 必须改为链接同一 `libprotobuf 33.6`。两种路径都禁止同进程出现第二套 `libprotobuf.so`，Phase 0 用 `ldd/readelf` 和跨语言 smoke 裁决；不能成立时先调整并重新冻结整套依赖，不能只替换单个运行库。
- eCAL、Protobuf、MCAP C++ 与 Zstd 先由联网参考门生成独立 `packaging/locks/cpp-dependencies.lock`，其中固定官方 URL、`ref_kind/ref/commit`、源码归档 checksum、许可证和构建选项；PCL 只构建到独立 validation prefix，供格式交叉验证，不进入运行包。总计划可生成供 A/C/开发 D 使用的开发 dependency prefix，但 E 的每次 stage-only 和正式 A/B 构建都必须从同一只读 canonical source artifact 重新创建私有 source/build/dependency/validation tree，正式双根不得读取开发 prefix 或彼此的中间物。`references/manifest.yml` 只管理只读参考 checkout，不能替代发行依赖锁；后续 A/C/E 禁止各自 FetchContent 或临时下载。开发和发行安装都通过同一个 closure install 入口把 eCAL、Protobuf、MCAP、Zstd 的运行库、SONAME 链接及 SDK 所需 header/CMake config 搬入目标 `lib/include/share`，下游只把本轮安装 root 放进 `CMAKE_PREFIX_PATH`。生成的 runtime/release manifest 记录精确版本、commit、lock SHA、源码 SHA-256 和本轮 dependency tree digest。所有安装可执行文件使用 `$ORIGIN/../lib` RUNPATH；清空开发 PATH/LD_LIBRARY_PATH 并把安装树搬到新绝对路径后仍须运行。build tree 可临时引用本轮冻结 dependency prefix，但安装自检拒绝 Conda、仓库或 builder 路径。
- 同一获授权联网 producer 根据 `cpp-dependencies.lock` 与 `ros2-dependencies.lock` 输出只读 canonical C++/ROS source archive artifact 和 `packaging/locks/source-archive-cache.manifest.json`。每条记录固定 normalized HTTPS URL、basename、`ref_kind: tag | commit`、不可变 `ref`、40 位 `commit`、archive format、size、SHA-256、`archives/<sha256>/<basename>` 路径、consumer 集、唯一顶层目录、archive member/regular-byte/symlink census、零链接 materialized member/byte/tree digest 和完整 artifact tree digest；tag ref 必须由 peeled tag 精确落到 commit，commit ref 必须等于完整 commit，Livox-SDK2 使用后者。artifact 层只允许 manifest 内普通 archive 文件，拒绝链接、`st_nlink != 1`、额外成员和同 basename 不同 hash。离线 verifier 逐项 `lstat`/hash 并与两个 lock 一一对应，再用唯一结构化 parser 预检所有成员：拒绝绝对/空/控制字符/`.`/`..` 路径、规范化重复、文件目录冲突、多顶层根、device/FIFO/socket、所有 hardlink、绝对/逃逸/悬空/循环 symlink，以及 member/单文件/总展开大小与 manifest/global ceiling 不符。根内相对 symlink 只有逐跳保持在冻结根内、最终指向已声明普通文件/目录时才允许；file link 复制 bytes，directory link 在本轮私有树按冻结图深拷贝，输出必须只有目录/普通文件且 `st_nlink == 1`。锁定 Zstd 归档中的真实根内 link 是正例，不能一律拒绝。每次 C++、D ROS、E stage/final 构建只能只读消费 canonical root，先 exclusive-copy 精确 consumer 归档到本轮私有 `source-work/archives`，再安全物化到本轮私有 `source-work/trees`；双根不得共享可写副本或解包树，不得在 canonical root 原地解包、调用 shell extractor，缺包/漂移必须在启动 CMake/colcon 前失败且没有联网 fallback。
- Python 锁生产工具链固定为 micromamba `2.8.1-1`、conda-lock `4.0.2` 和 conda-pack `0.9.2`。`packaging/locks/python-toolchain.lock` 必须把 Git tag provenance 与安装制品 checksum 分开记录：mamba `2.8.1` tag commit 为 `0abc611db8b7bc92bfb7841158c713d0d028bedb`，Linux amd64 micromamba binary SHA-256 为 `77b7790ec97f64581118f103585b175df4306f95829b0fa6bfe4a19cc88a1182`；conda-lock `4.0.2` tag commit 为 `e29d5cf7dcb826b07ba1696883426494b4d96d66`；conda-pack `0.9.2` tag commit 为 `3efad58976f33eff3ef21c2882e9cd7458720af5`。conda-lock/conda-pack 的实际 Conda package URL、MD5 和 SHA-256 取自 toolchain unified lock 与 cache manifest，不能拿 tag commit 充当 package hash。
- `packaging/python-environment.yml` 是生产 Python runtime 的唯一人工 spec；`packaging/python-toolchain-environment.yml` 独立声明只在构建机使用的 build/pip/conda-pack tool env；二者共用冻结的 `packaging/locks/virtual-packages.yml`，不得从 producer 主机即时探测。总计划 Task 2 用固定 conda-lock/micromamba argv 分别生成两套 unified/explicit lock，项目自身 wheel 不进入环境求解。所有生产 package manager 必须为 `conda`，unified lock 禁止 `manager: pip`，explicit lock 禁止 `# pip`。conda-lock 的 pip 安装分支没有等价 `--no-index` 硬门，不能进入本离线生产链。其 explicit renderer 只携带 MD5 URL fragment，因此 verifier 必须将其与 unified lock 逐项对应，并对 cache archive 同时复算 MD5 和 SHA-256。缺 spec、virtual-package 漂移、缺 hash、重复 package、unified/explicit render 漂移或 lock 外 package 都是硬失败。
- 联网 producer 输出的 canonical Python cache artifact 是 E 唯一 package 来源。其可审计输入布局固定为规范排序且有独立 SHA-256 的 `pkgs/urls.txt`，加 `pkgs/https/<host>/<channel>/<subdir>/<archive>`；manifest 记录 normalized URL、经 URL parser 验证的 archive basename、canonical relative path、size、MD5、SHA-256、所属 lock 和完整 tree digest。不允许 artifact 扁平 fallback、额外 repodata/package、重复 URL、符号/硬链接、普通文件 `st_nlink != 1` 或开发机 `~/.conda/pkgs`/Mamba cache。该 URL 嵌套 artifact 不是 micromamba 原生 package cache：总计划 Task 2 已测试的 builder 必须在每轮全新空 `mamba-root/pkgs` 按 manifest 重新 `lstat`/hash 后，把 archive bytes 以 exclusive create 物化为 cache 根级 `<archive-basename>` 并写排序后的原 normalized URL `urls.txt`；同 basename 只有 size/MD5/SHA-256 全同才复制一次，否则失败。双根可读同一 canonical artifact，但不得共享可写 native cache、root prefix、tool env、runtime env 或解包结果。lock、artifact、materializer 或工具链要变化时必须返回总计划 Task 2 重新生成并用 pinned micromamba 在硬断网 fake channel 下证明“嵌套 artifact -> native flat cache -> explicit create”；E 禁止 conda-lock、render、solve、repoquery、download 或新增 channel。
- PyPI 官方 eCAL wheel 使用独立 `python-wheel-cache.manifest.json` 与 `wheels/<sha256>/<filename>` canonical artifact；producer 交叉验证 PyPI release JSON 和实际 bytes，并冻结 URL、filename/version、`Requires-Python`、PEP 425 tag、size/SHA-256、ZIP member/tree、`METADATA/WHEEL/RECORD`、逐成员 RECORD、license/NOTICE 和全部 ELF 的 SONAME/NEEDED/RUNPATH/hash。cache 只允许这一个普通文件，拒绝额外成员、链接、错误 ABI/tag、路径逃逸、RECORD/license/ELF 漂移和 bundled `libprotobuf.so`。每轮只读验证后把它 exclusive-copy 到私有 wheel cache 并再次复算；D/E 不得访问 pip index 或用户 pip cache。
- Python 环境创建必须运行在已验证无外网路由的 user+network namespace 或等价断网 VM 中；无法建立隔离时先失败，不能把 micromamba `--offline` 当作完整网络沙箱。wrapper 必须保留可用 loopback，同时移除 IPv4/IPv6 default route 和全部非 loopback interface，并记录仍存活 parent 与 child 的 network namespace inode、PID、argv digest、结构化 link/route、TEST-NET `ENETUNREACH` 和 loopback socket 成功证据；child inode 必须与 parent 不同。每个 Python/C++/ROS builder 在 create/configure 前都从 `/proc` 和内核 link/route 独立复算，拒绝仅靠环境 token、同 netns、默认路由、非 loopback interface、parent 不可读或 evidence 漂移；等价 VM 也必须给出相同字段且没有 skip。runtime 命令固定为 `"$MICROMAMBA" create --no-rc --no-env --root-prefix "$WORK/mamba-root" --prefix "$WORK/python-builder" --file "$SOURCE/packaging/locks/python-linux-64.lock" --offline --always-copy --safety-checks enabled --yes`，tool env 对其独立 explicit lock 使用同样参数。builder 另设空 HOME 并清除 Conda/Mamba rc、channel 和 cache 环境变量。`--offline` 保证使用已缓存 repodata/package，但缺包时实现仍可能构造下载请求，所以内核态断网证据是不可省略的第二道门。
- conda-pack 必须面对完整、未经项目 pip 污染的纯 Conda `python-builder`，且对应 work root 的完整 package cache 要保留到 pack 成功；不得先删除 conda 管理文件或其 `.pyc/__pycache__`。固定命令为 `"$WORK/tool-env/bin/conda-pack" --prefix "$WORK/python-builder" --output "$WORK/python-pack/python-runtime.tar" --format tar --n-threads 1`。成功后才解到 staging `root/runtime/python`，再由同一 Python 3.10 ABI/sysconfig layout 的 tool env 分两次运行 pip，以相同的 `--no-deps --no-index --no-compile --prefix "$WORK/root/runtime/python"` 先安装本轮私有官方 eCAL wheel，再安装本轮唯一项目 wheel；这样两个 wheel 都不会污染 conda-pack 输入。项目命令由根 `bin/` launcher 提供；若 pip 为任一 wheel 写出绝对 `direct_url.json` 则删除，按项目 entry-point metadata 精确移除 console scripts，再分别按规范相对路径确定性重算两个 dist-info `RECORD`。eCAL license/NOTICE 与 ELF inventory 必须保持冻结值，其他 prefix-bearing 文件或陈旧 RECORD 一律失败。
- `conda-meta/history` 会原样携带环境创建绝对路径，因此只在 conda-pack 成功后的 staging 删除它；随后才删除 staging 中全部 `.pyc/__pycache__`、规范 mode/mtime，并扫描 filename、text、binary、wheel RECORD 与 `conda-unpack` records 中的 source/work/builder/cache 路径。除 `history` 外保留 conda-meta package records。原 packed tree 不运行 `conda-unpack`，只有随机 smoke 副本和安装后的最终版本路径可重定位。conda-pack 中间 tar 不保证自身跨根 byte-identical；门禁比较清理、规范化后的 runtime tree 逐成员 digest，以及固定 tar/Zstd 后的最终发行 archive。
- Ubuntu 24.04 系统 ABI 使用独立 lock：只允许动态加载器、glibc、libstdc++、libgcc_s 和 `libcrypto.so.3` 等明确列出的基础 SONAME，记录测试过的 apt 包版本并允许同 SONAME 的安全补丁升级；其他非系统 DSO 必须随包进入根 `lib/`。OpenSSL EVP 只依赖该系统 `libcrypto.so.3`，不再声称来自项目 dependency prefix。
- 核心运行不要求 ROS 2；可选 Bridge 要求目标机预先安装 lock 明确列出的 ROS 2 Jazzy runtime 和 RViz2，而不是笼统的“基础环境”。`ros2-dependencies.lock` 从锁定 `livox_ros_driver2 + Livox-SDK2` 官方 package manifest 生成完整系统构建/运行依赖、测试版本和允许 SONAME；每轮先把 Livox-SDK2 安装到本轮私有 `livox-sdk-install`，再把其 library/include 绝对路径显式传给 driver，禁止 sudo、默认安装或读取 `/usr/local`。builder 在前后对真实 `/usr/local/lib` 与 `/usr/local/include` 做排序 census/hash 并要求不变，再通过 CMakeCache/link command/readelf/ldd 证明没有 fallback。发行包自带用该环境构建并验证接口 hash 的 Bridge/Livox overlay。根 `bin/slope-sim-ros-bridge` launcher 只从自身真实路径推导 release root，依次加载系统 Jazzy 和本包 overlay 后执行精确 binary；整个 release tree 搬到另一绝对路径后仍必须可启动。`doctor` 用 dpkg/ament/ELF 三层检查全部前置，缺任一项时只报告 ROS `optional_unavailable`，不能阻止核心命令运行，也不得开始正式 ROS/RViz2 门禁。
- NVIDIA 驱动和上述 ROS 2 系统包不随包分发，安装器只检查并给出明确的缺失包清单、测试版本和配置教程。

外层交付文件为：

```text
slope-sim-stage4-<version>-ubuntu24.04-amd64.tar.zst
```

`<version>` 只能是完整匹配 SemVer 2.0.0、长度不超过 128 ASCII bytes 的 canonical 单路径分量；`/`、`.`、`..`、空白、控制字符、换行、超长、前导 `v` 和数字段前导零都在创建输出或安装路径前失败，不能通过 trim 或重写变成合法值。builder、verifier、handoff 和安装器必须复用同一解析函数并逐 byte 保留已验证值。

它是迁移和安装容器，不是直接执行文件。包内包含：

- `install.sh`、`uninstall.sh`、安装后自检和 SHA-256 校验。
- 由冻结 explicit lock 和 canonical cache 离线创建的版本化 Python 3.10 运行环境；目标机不要求安装 Conda，也不读取开发机缓存。
- PyBullet、PySide6、Protobuf `6.33.6`、eCAL `6.1.1` 等锁定运行依赖。
- C++ SDK、CLI、Recorder、Command Tool、Replay/Export 和可选 ROS 2 Bridge 包。
- 车型、地形、纹理、默认场景、`ecal.yaml`、RViz2 配置、四车型 canonical `models/robot_models.yaml`、小型完整 MCAP self-test session 和桌面入口。
- 中文部署、SDK、接口、人工测试、回放和故障排查文档。
- manifest、clean Git commit/tree、source snapshot SHA-256、descriptor hash、依赖清单、许可证和 `SHA256SUMS`。

核心安装在无网络环境可完成，不依赖 `.git`、`references/`、开发机 Conda/Mamba 缓存或官方 MID-360 样例；用于构建该 runtime 的 canonical package cache 也不进入最终发行包。

归档中的 packed payload 与安装后的每个版本目录都只有一套 release tree：`bin/` 放编排 launcher 和 C++ ELF，`lib/` 放项目及非系统 C++ runtime closure，`include/` 与 `share/*/cmake` 放可迁移 SDK，`share/slope-sim/` 放全部资源/协议/descriptor/self-test 及 bundled production verifier，`runtime/python/` 放自包含 Python 环境，`ros-overlay/` 放可选 Jazzy Bridge；不得同时出现第二套 `usr/bin` 布局。payload 内 `runtime/python/` 保持 conda-pack packed 状态，构建 smoke 只在随机一次性副本运行 `conda-unpack`。构建 smoke 的状态提交顺序固定为 relocation、CLI/SDK/self-test/loader、无 state 的纯 precommit health probe、单次原子 `install-state.json`、读取最终 state 的普通 doctor；state 保存 probe 的 canonical `health` 而不保存尚未发生的 doctor 输出，doctor 重算并要求 health 相同。构建期 Python/C++ DSO 隔离只允许专用 no-participant loader import/`dlopen` 各自 eCAL core，并用调用审计、前后零 entity census 和 `/proc/<pid>/maps` 证明未调用 Initialize/pub/sub、未新增 participant/topic 且没有交叉/重复 eCAL 或 libprotobuf；它不属于真实通信验收，真实 participant 只在逐条获授权门禁中创建。

最终归档的生产 verifier 在发布前逐项拒绝绝对路径、`..`、重复成员、符号/硬链接、device/FIFO/socket、超出 manifest 的文件和解压膨胀超限。Conda、C++ SONAME 或 ROS staging 中的根内链接只允许存在于受控构建中间树；生成 manifest 前，构建器先冻结原 staging 的 `lstat` 成员图，再在 sibling 临时根构造零链接第二棵树。hardlink 和根内相对 file symlink 复制成独立普通文件；根内相对 directory symlink（包括锁定 Conda runtime 的 `lib/python3.1 -> python3.10`、`lib/terminfo -> ../share/terminfo` 形态）按冻结图排序深拷贝目标子树。绝对/逃逸/悬空/循环链接、特殊目标、路径冲突、展开 member/byte 超限或复制竞态全部失败；临时树通过逐成员 hash、ELF、真实 Conda `conda-unpack`/import 和 ROS smoke 后才替换 staging。非空 root 只能使用经过同文件系统探测的 Linux `renameat2(RENAME_EXCHANGE)`：fsync 新树和父目录，写入并 fsync 外部 `PREPARED` 事务记录，原子交换后记录 `EXCHANGED`，复核成功再保留旧树为诊断副本并记录 `VERIFIED`；不支持、任一 fsync/交换/复核失败或交换后崩溃都必须依据记录与两树 digest fail closed、交换回滚并保留诊断树，不能用普通 rename 覆盖或猜测恢复。最终递归 `lstat` 要求只剩目录和普通文件、每个文件 `st_nlink == 1`，并在 evidence 保存 materialize 前真实 link census 与零链接结果。这会增加少量体积，但使最终 archive 和已解源树保持零链接。

目标机先从 sidecar 所在目录核对归档外部 SHA，再按教程用 `--no-same-owner --no-same-permissions` 解到全新空目录；校验不得依赖调用者 cwd，sidecar 中的精确 archive basename 必须与同目录显式 archive 匹配。包内安装器不能倒置时间声称自己检查了解包前的外层 archive，而是对已解出的源树逐项 `lstat`，拒绝链接、非普通文件/目录、link count 异常、清单外成员和 resolve 逃逸，再以不跟随链接、exclusive create 的方式复制到同文件系统私有 incoming。安装器在写 prefix 前还要只读重算外部 archive、sidecar 和 `release-build-evidence.json`，要求 archive basename/hash 与 evidence 完全匹配；它不枚举或解包 archive，只把来源摘要绑定到本次安装。随后只读验证 packed payload，把版本放入最终 `releases/<version>`，在该最终绝对路径运行 `conda-unpack` 并原子生成绑定当前 root 的 `relocation-state.json`；再依次完成 CLI/SDK/self-test、无 state 的纯 precommit health probe、单次原子 `install-state.json` 和读取最终 state 的普通 doctor，最后才切换 `current`。任何 precommit 或 doctor 失败都隔离失败版本且保持旧 `current` 不变。

正式 `install-state.json` 使用 `provenance.kind="verified_archive"`，保存 archive basename/SHA-256、build-evidence SHA-256、clean Git commit/tree/source snapshot SHA-256、packed manifest SHA、最终路径、解包工具 hash、同根 `relocation-state.json` SHA 和 precommit probe 的 canonical `health`；build smoke 使用互斥的 `provenance.kind="build_smoke"`，不得伪造 archive 或 clean-source 字段。两者都不嵌入未来 doctor 输出；state 提交后的 doctor 以 state SHA 绑定当前安装，重算并要求 `doctor.health == install-state.health`。安装后的 Python 已完成重定位，不再拿它与 packed Python 的逐文件 hash 错误比较，而以不可变 manifest、`install-state.json`、最终 `sys.prefix`、旧前缀清零和关键 import 验证。安装器在 prefix 内创建的版本选择 `current` symlink 不属于 archive 或源树内容，仍由原子切换测试单独约束。

`build_release.sh` 必须接收绝对空 `--work-root/--output-dir`、只读 `--python-package-cache`、只读 `--python-wheel-cache` 和只读 `--source-archive-cache`，Python、CMake、ROS、每轮私有 source-build/cache/source work、安装根和 smoke 全部派生到该 work root。`--stage-only` 不能遍历整个 worktree：它分别消费 Git NUL 结构化 tracked 路径与 `--others --exclude-standard` non-ignored untracked 路径，只读取 `build-source-manifest.yml` allowlist 内的当前 working-tree bytes，allowlist 外 non-ignored untracked 直接失败；helper 以仓库 dirfd/no-follow 及复制前后 device/inode/type/size/hash 把有限成员复制到仓库外只读 `development-snapshot`，忠实记录 dirty tracked 与允许的 dirty untracked，同时排除 `.git`、ignored cache、`references/repos` 和 work/output/evidence。该 snapshot 再 exclusive-copy 到私有可写 source-build，生成明确标记为 `development_worktree` 的 smoke，但不得升级为正式来源。`--final-archive` 在创建任何产物前要求 index、全部 tracked 文件和 untracked 文件集合均为空，且 work/output/evidence 必须在仓库外；正式归档前必须先展示 fresh GREEN 与 diff，取得用户明确 commit 授权并建立 clean-HEAD checkpoint。它从精确 `HEAD` 的 `git archive` 创建递归只读 source-snapshot，计算规范成员/mode/bytes digest，再 exclusive-copy 到本轮独立可写 source-build 并复算相同 digest；setuptools/CMake/ROS/资源安装只消费该副本，构建前后 snapshot digest/权限/成员必须不变。这样 `egg_info` 等 build metadata 只能写 source-build，ignored build/result、staged/unstaged/untracked 内容都不能在仍声称旧 Git SHA 时进入归档。build evidence 固定记录 `source_provenance.kind=clean_git_commit`、`clean=true`、commit/tree id、source snapshot pre/post SHA-256、等值 source-build pre-build SHA-256、source archive manifest/tree digest 和本轮私有 materialization digest；两次构建必须全部一致。

安装器、卸载器、中文教程和纯回归全部完成后，才在同一仓库外私有 parent 下用两个互不相关的空 work/output sibling 各构建一次；final 模式拒绝仓库内或包含仓库的临时根。A 完成后仓库 clean 状态必须仍为空，B 入口再独立执行 clean gate，避免 A 产物使 B 必然失败。两根可消费同一只读 canonical Python package artifact、官方 eCAL wheel artifact 和同一只读 canonical C++/ROS source archive artifact，但必须复制出各自独立的 source-build、可写 native package/wheel cache、tool/runtime env、C++ dependency source/build/install/validation tree、ROS 源码副本和安全解包树；不得读取开发 prefix 或复用第一轮中间物。可复现性不仅规范外层 tar：C/C++/ROS 编译统一使用 `SOURCE_DATE_EPOCH`、`-ffile-prefix-map/-fdebug-prefix-map/-fmacro-prefix-map` 把 source/work root 映射到固定前缀，禁止安装 CMake/pkg-config/generated 文件泄漏 build root；Python wheel 固定时间戳，只在 conda-pack 成功后的 staging 删除 `.pyc/__pycache__` 并清除 `conda-meta/history`，self-test/manifest/SBOM 不记录绝对临时路径。先证明规范化后的两套 C++ dependency、validation 和 Python runtime tree 逐成员一致，再固定 tar 顺序/mtime/uid/gid 与 Zstd 参数；两个精确命名的最终 `.tar.zst` 必须 byte-identical，才能从明确路径发布唯一归档，禁止 glob 猜来源。archive hash 只写在归档外的 `.sha256` 和 build evidence 中，不能自引用写进 archive 内部 manifest；内部 `SHA256SUMS` 只覆盖 packed 内容。发布、验证和验收各自生成带 hash 的 shell-safe handoff；每个后续 shell 都先复核再 source。验收安装完成后必须生成结构化 no-participant smoke，并让 acceptance handoff 以同一 run id 固定 resolved installed root、`install-state.json`、`relocation-state.json`、fresh doctor 和 smoke JSON 五件套的绝对路径与摘要，同时导出本轮仓库外 evidence 目录；后续不能依赖前一 Run 的临时变量或重建这些路径。

真实升级/回退/卸载另从同一 candidate clean HEAD、locks、caches 和 toolchain 在第三个全新根构建一个版本不同的完整 lifecycle probe。它与主 candidate 使用相同生产 builder，但 builder 把 `artifact_purpose=lifecycle_probe`、`publishable=false` 同时写入归档内受 checksum 保护的 release manifest 和外部 build evidence；专用 lifecycle verifier/portable handoff 只用 sibling basename/hash 解析 probe archive/sidecar/build evidence，并保存主 candidate 的不可变 identity。普通 release verifier 与 release handoff 必须要求 `artifact_purpose=release`、`publishable=true`，即使绕过 probe handoff直接提交 probe archive 也失败。primary archive/sidecar/build evidence 三件套和 probe archive/sidecar/build evidence/handoff 四件套都由 verifier 在父目录锁内 exclusive-copy/write、逐文件和目录 fsync、单次 directory rename、parent fsync 后提交，禁止 shell `install + mv` 发布；后续 candidate/probe context 必须验证 committed directory。probe bundle 不进入主发布目录、A/B reproducibility、candidate/final payload equivalence 或 final release。控制机还必须从已验证 primary/probe handoff 生成不含绝对路径的 portable clean-host transfer context，以目录事务提交 canonical release version、精确 artifact/payload/probe basename/hash、shell-safe env 和内部 `SHA256SUMS`；目标机经 pinned SSH 收到完整 context、校验 checksum 和 basename/版本关系后才能 source，教程中不得保留 `<version>` 或用 glob 重新发现 artifact。目标机的 probe preflight JSON/env 必须在全新目录事务中同时提交，并由独立 consumer 结构化复核 committed pair 后才能 `source`；孤立成员、staging 或任一 write/fsync/rename 故障均 fail closed。目标机随后先冻结主版本状态，真实安装 probe 并冻结升级状态，再通过唯一 `install.sh --activate-existing` 原子回退、卸载已经非 current 的 probe，最后结构化证明配置/数据未变和 probe 目录消失。post-install doctor 失败仍由安装器 TDD fixture 注入，生产包和真实机不带 test-only fail hook。

干净机按 bundled production verifier 生成同 schema smoke JSON；初次与 REFACTOR 复验各自使用全新 `mktemp` run root 和 fresh doctor，旧证据只读保留。headless、interactive、live ROS、replay ROS 四类真实结果以 initial handoff 为 acceptance anchor，production freezer 按代码内 schema 递归冻结完整 MCAP session/segments、PCD/PLY/LVX2、GUI/RViz2/Livox Viewer 截图、工具打开结果、安装/运行日志和 host inventory，不接受 caller 成员表。两次 run、chain、lifecycle 和 production handoff 中的 `/opt/...`、`$HOME/...` 只在目标机有效：bundled verifier 必须在路径仍可读取时把实际 evidence bytes、安装树 identity、target/candidate/probe identity、一次性 challenge 和摘要写入成员/大小受限、无链接/特殊/逃逸路径的 deterministic `tar.zst` 与 sidecar。验收控制机只经预先固定的 SSH host key 拉回 bundle/sidecar，并用安全 archive reader 导入。challenge 在 import root 外的持久 0700 registry 中以 `issued -> consuming -> consumed` 原子消费并生成 receipt；相同 challenge/bundle 换新 import root 重放也失败，崩溃停在 consuming 时 fail closed 并重新签发。成功后本地 import context 固定 imported chain/lifecycle/production 和 receipt；控制机不得 source 远端 env 后重新解引用另一台机器的绝对路径。complete evidence/accepted-candidate context 显式冻结这些本地 evidence 与 probe handoff，拒绝 member/bundle 篡改、challenge 重放、换 host/root/archive、旧 role 或直接复制失效 env；它们不从交付报告推断 evidence 或 run 选择。

真实验收使用上述生产格式的双根 acceptance candidate，但候选不能直接升级为最终发布：Task 7/8 只允许在仓库外写 evidence，并只读执行候选内已经冻结的部署教程；教程、代码、lock、测试、packaging、配置或资源若需修改，候选与真实验收全部失效。六维审查清零后，独立审查者的 canonical source 必须先冻结为仓库外不可变 JSON/handoff 目录事务；该事务内嵌 reviewer identity/task id、被审 commit/tree、六个精确维度、findings/disposition 和逐项重算的 evidence index，并要求 `Critical=0, Important=0`。accepted-candidate context 显式绑定该 review transaction，不读取或冻结交付报告/README。随后只有不进入 payload 的 `README.md` 和 `docs/阶段四交付报告.md` 可以形成最终 evidence commit；重新取得用户 commit 授权并从该 clean HEAD 在两组全新根重建正式 archive。accepted-candidate context 冻结候选 commit 的 `functional_source_epoch`，正式 builder 在代码内固定的 source diff 验证通过后继承该 epoch，但 runtime/build evidence 如实记录最终 commit/tree/source snapshot，避免 evidence-only commit 的新时间戳改变功能构建 bytes。

结构化 verifier 安全解包候选与正式包，先证明两个 source commit 的 diff 只含上述两个外部状态路径，再要求安装器/教程、self-test MCAP segment/recipe/models、`bin/lib/include/runtime/python/ros-overlay`、资源/proto/descriptor/lock/许可证逐 byte 相同。唯一允许变化的是代码内固定的受限 provenance 派生闭包：`share/slope-sim/runtime-manifest.json`、`share/slope-sim/selftest/session.manifest.pb`、`share/slope-sim/selftest/selftest-evidence.json`、`share/slope-sim/sbom.spdx.json`、`release-manifest.json`、`SHA256SUMS`，以及由 archive SHA 继续派生的外部 sidecar/build evidence/handoff。verifier 不信任产物或 CLI 自报的 allowlist，必须从稳定叶子按 runtime manifest -> `SessionManifest.runtime_manifest_sha256` -> selftest evidence -> SPDX SBOM -> release manifest -> SHA256SUMS -> archive -> 外部 evidence 的无环顺序自底向上重算；额外路径/字段、断链、循环/自引用、稳定字段变化或修改功能文件后同步重写全部摘要都失败。候选与正式安装状态不要求 raw bytes 相同，而是分别验证各自同一 run 的 installed root、`install-state.json`、`relocation-state.json`、fresh doctor 和 no-participant smoke 五件套，再移除已验证的 prefix/path-bound 表示比较归一化功能语义；不能借安装路径不同放宽任何其他字段。accepted-candidate context 必须冻结候选五件套的路径、摘要与 run id，以及干净机 import context、imported chain/lifecycle/production、consumed receipt、lifecycle-probe handoff 和不可变六维 review handoff；正式包必须先在全新 prefix 重做同 schema smoke，再把 final 五件套作为显式输入交给比较器。probe 不参与候选/正式 payload 比较。全部通过后以一次目录事务同时提交 equivalence JSON 和绑定 accepted context、final handoff、两侧证据及 JSON 摘要的 equivalence handoff；缺失、篡改、跨 run 混用或事后才生成任一 evidence 都禁止继承候选的真实验收。

最终完成状态由独立 `final-release-status.json` 唯一裁决。final-status verifier 必须显式消费 accepted-candidate context、final release handoff、payload equivalence 和 final 五件套，重新验证 archive/sidecar/build evidence、候选真实 eCAL/GUI/RViz2/Livox Viewer/干净机/不可变六维 review transaction、受限闭包及两侧 run 绑定，不能信任前置 preflight。只有全部成立才以一次目录事务同时提交 `status=complete` 的 JSON 和 handoff；缺失、篡改、陈旧 equivalence、错误 run 或跨安装混用时不得创建可消费输出。交付报告和 README 只在 fresh 验证该 handoff 后引用它，不作为 accepted context 或 final status 输入，避免摘要自引用。

小型完整 MCAP self-test 由正式 Recorder/Reader 代码的 build-only generator 产生，并以当前安装树 `runtime-manifest.json` 摘要、canonical `models/robot_models.yaml` 和固定 recipe 为输入；最终 manifest、唯一 segment 和 evidence 一并安装到 `share/slope-sim/selftest/`。它只用于安装后回读/回放/导出 smoke，不替代真实 eCAL 性能与零 drop 门禁。

### 12.2 安装、启动和数据目录

版本化安装目录：

```text
/opt/slope-sim/releases/<version>/
/opt/slope-sim/current -> releases/<version>
```

用户配置默认位于 `~/.config/slope-sim`，记录和导出位于 `~/slope-sim-data`。升级不得覆盖这些目录；新版本启动失败时可以把 `current` 原子切回上一个已安装版本。

安装后同时提供桌面入口和命令：

```text
slope-sim start interactive   # GUI、Dashboard、eCAL、Subscriber、Recorder，可选 ROS/RViz2
slope-sim start headless      # 无界面生产运行
slope-sim status
slope-sim stop
slope-sim-sub                 # 单独只读接收
slope-sim-command             # 单独命令测试
slope-sim-record / replay / export
```

`interactive` 和 `headless` 是编排入口，不把所有组件链接成一个进程。任何正式 `/sim` producer 启动前，编排器先在 `$XDG_RUNTIME_DIR/slope-sim/production.lock` 取得非阻塞独占 `flock` 并持有到全部 participant 关闭；锁记录 PID/session 只供诊断，进程死亡由内核释放。取得锁后才在 `$XDG_RUNTIME_DIR/slope-sim/<session>/` 创建权限为 `0700` 的本地 control socket；子进程连接必须以 `SO_PEERCRED` 同时匹配当前 uid 和编排器实际 spawn 的 PID/唯一 role，不能只凭自报 role/session。两个不同随机 session 的编排器也必须在创建任何 eCAL 资源前互斥。

本地控制面使用独立 `slope_sim.control.v1` Protobuf；Unix stream framing 固定为 network-order `uint32` 长度加最多 1 MiB 的 deterministic payload。Python/C++ 共用同一 `.proto`、字段号和 golden bytes，覆盖 role/state、request/ACK、逐 topic `TopicHealth`（waiting/pending/verified/conflict、exact peer count、远端 type/encoding/descriptor digest、accept/reject/drop）、带租约的 `ManualTwistTarget`、Command scene freeze/resume、scene attachment、segment cut/barrier、begin drain、end barrier、逐 topic fence、队列健康和错误；stream 解码允许分片和一次读取内连续多帧。不得由 C/E 分别实现不兼容 JSON 或私有结构。每个已认证生产 role 的 lifecycle 固定先进入 `STARTING`，只用该 state 承载 READY 前的 WAITING/PENDING/VERIFIED/CONFLICT health，且不打开业务门；全部必需 topic VERIFIED 后只允许 `STARTING -> READY -> ACTIVE`，启动冲突可 `STARTING -> FAILED`，离开 STARTING 后不得回退或跳过 READY。Bridge 与 Replay 使用同一规则，编排器不能代发其 STARTING/READY。

Python 只注册 `slope-sim` 编排入口；`slope-sim-sub/command/record/replay/export` 是 CMake 安装的 C++ ELF。`slope-sim doctor` 与 `service enable|disable|status` 属于正式 parser 合同，systemd 只有显式 `service enable` 才启用。

正常停止顺序固定为：Simulator 主线程先捕获正式 `end_timestamp_ns` 与三类快照；Command Tool 不退出，转为按当前有效 session/world/command generation 以 100 Hz 发送零命令至少 100 ms，在发布第一条越过阈值的零命令后保存其完整 identity、冻结自身 publisher 并保持 participant 存活；Simulator 再取得四输出各一条 `timestamp_ns > end` 的 post-window fence，并在线性化点冻结这四个 publisher。只有五条 fence 已固定且五个 publisher 都不会再产出业务消息，Simulator 才发送 `EndBarrier`。

control socket 与 eCAL 是两个无全局到达顺序的通道，因此 required fence 可能在 `EndBarrier` 之前已经位于 raw、ordered-commit（READY/REJECTED/DEFERRED）、rotation holding 或 written 任一阶段。Recorder 不保留随会话增长的全量 identity 表，而是复用受总账约束的现有 pipeline，并为每 topic 维护 validated/written 高水位和连续性状态。收到 barrier 后在同一个状态锁内进入 DRAINING、锁存五条 required identity：validated 高水位已精确达到 fence 的 topic 立即关闭 ingress，已越过则失败，尚未达到则继续处理已有 raw 和新到在途帧，worker 恰好验证该 fence 后关闭；不能因为 fence 已在 raw 队列中就等待第二次 callback。任何已关闭 ingress 的新帧、identity 越过 required fence、缺 fence 超时、重复 fence 或 drain frontier 前仍有 DEFERRED 都属于协议失败；fence 必须最终按 order 进入 writer，不能因 barrier 前已见就跳过落盘。五个 ingress 都关闭、ordered frontier 已连续跨过全部 required fence 且 written 高水位精确达到五条 fence 后，再排空 raw/ordered-commit/rotation holding，完成全部 segment 和 session manifest 的 flush/fsync/rename 才成为 durable 并进入 FINALIZED；最后才退出 Command、Subscriber、Bridge、Simulator participant 和编排器。Recorder fatal 时编排器先撤销命令并请求 Simulator 安全停止，再以非零结果结束会话；不得按“先停 Command/Recorder、再收 fence”的顺序丢失正常尾帧，也不得让 Command 在已声明 fence 后继续发布。

正式 profile 的 scene/world rebuild、segment rotation 和 normal drain 由同一个编排器 lifecycle mutex 串行化。已 prepare 的 rebuild 必须先冻结五个 publisher，再完成物理 commit/rollback；只有物理 commit 成功才发送 attachment，attachment ACK 后才恢复业务。`ROTATING` 期间最多保留一个 pending stop，轮转回 ACTIVE 后优先进入 drain，新的 rebuild 返回 busy；DRAINING 后不再接受 rebuild/rotation。这样 scene attachment 不会跨越未确定的 segment cut，用户重复点击停止也只产生一次 end barrier。

发行包同时提供默认禁用的 systemd user units，供固定工作站长期运行 headless Simulator、Recorder 和可选 Bridge。安装程序不得未经用户选择就设置开机自启；交互 GUI/RViz2 仍从当前桌面会话启动。

## 13. 测试和最终验收

### 13.1 TDD 与协议门禁

- 重要合同和缺陷先写 RED，再做最小 GREEN。
- v1 descriptor bytes/SHA 保持不变。
- v2 Python 编码、C++ 解码以及 C++ 编码、Python 解码使用固定 golden bytes 双向验证。
- 非法字段、错误 simulation/source session、未知 world/command generation、错误 owner、逆序 sequence、RTK presence、数组基数、NaN/Inf、机械限位和 descriptor 不一致全部有拒绝测试。
- 生成文件由固定命令再生成，并验证源码无直接手改。

### 13.2 四车型三地形矩阵

四种车型分别在正式 `flat`、`slope` 和 `golf_heightfield` 场景验证：

- 仅一个中心 `lidar_link`，外参准确且无本车回波。
- 360° 首尾无重复，地面、墙体和障碍表面点与独立 PyBullet 射线真值一致。
- RTK 每帧恰好三个固定角色，二轮/四轮几何与 wheel link 世界坐标误差不超过 `1e-4 m`。
- IMU、轮态、命令数组语义、场景重建 world generation、命令权 command generation 和安全停车正确。
- 每个 10 Hz 位点都有 timestamp 完全相等的 WheelState/LiDAR/RTK/IMU，浮点漂移、错一纳秒、跳过其中一槽和最近邻补齐都必须被门禁抓住。
- golf 优化前后地形高程、碰撞、轨迹、点云和 RTK 不回归。

### 13.3 真实 eCAL 与 C++

同一正式会话同时运行 Python Simulator、C++ Subscriber、C++ Command Tool 和 C++ Recorder，验证：

- 五个 topic/type 可发现，eCAL metadata 与消息带内 descriptor SHA 一致，所有消息属于同一 simulation session。
- C++ 收到的每条原始 payload hash 与 Simulator 发布日志对应。
- 主动转向 `4+2` 和代表性差速 `2+0` 均有直接关节反馈、RTK 位移、轨迹距离、平均速度和精确有效控制窗口证据；差速车型转向数组必须为空，主动转向左右前轮峰值角均大于 `0.1 rad` 且方向符合命令。
- peer 断开、命令超时、重连、新 command generation、旧命令拒绝和 clean shutdown。
- wheel、LiDAR、RTK、IMU 的 sequence 缺口、重复和额外消息均为零。

正式 oracle 使用同一主线程 start/end barrier 冻结仿真时间边界、raw publish/send 日志、transport 快照和 Recorder 状态。比较窗口固定为 `(start_sim_ns, end_sim_ns]`：四个 Simulator 输出逐条以 `(simulation_session_id, topic, timestamp, world_generation, sequence, descriptor_sha256, payload_sha256)` 在 Simulator、C++ Subscriber 和完整 session manifest 指向的全部 MCAP segment 三份证据间双向匹配；WheelCommand 以 `(simulation_session_id, timestamp, world_generation, command_generation, sequence, source_id, source_session_id, descriptor_sha256, payload_sha256)` 在 Command Tool、Simulator 和同一完整记录之间双向匹配。end 后 Command 保持 100 Hz 零命令 drain，发布并冻结唯一 command post-window fence；Simulator 继续至少一个 10 Hz 周期，发布并冻结四输出 fence。Recorder 在 barrier 后只允许接收不晚于 required fence 的在途 pair，收齐五条后证明每个 topic 零越界、零重复、零缺 fence，完成 manifest 并 flush 后才允许判定零缺失、零额外、零重复。

频率 oracle 同时检查墙钟和消息仿真时间：WheelCommand/WheelState 墙钟 `95..105 Hz`、时间戳 `99..101 Hz`，LiDAR/RTK/IMU 墙钟各 `9..11 Hz`、时间戳各 `9.9..10.1 Hz`。100 Hz 流含首尾边界的最大墙钟间隔不超过 `30 ms`，10 Hz 流不超过 `250 ms`。五秒正式控制窗内 RTK CENTER 位移和车体轨迹距离都大于 `0.5 m`、平均线速度大于 `0.1 m/s`；所有 transport/consumer/Recorder dropped 和 error 为 0，lane ready/latest/pending、Recorder queued messages/bytes 最终为 0，worker 全程存活且 clean shutdown/finalized 为 true。

真实 eCAL 属于受控外部门禁：每个 verifier invocation（不同车型、live/replay、ROS off/on、目标机 headless/interactive）执行前都必须单独重新取得用户明确授权，并即时扫描全机负载后严格串行运行；一次授权不覆盖列表中的下一条命令。失败即保留证据并停止，不自动重试，不降低 95 Hz 下限。阶段三修复前失败、并发污染结果和沙箱 socket 阻断都不能作为阶段四通过证据。

### 13.4 联合性能门禁

正式 workload 固定包含 `golf_heightfield` 场地、20 个障碍物、5,760 候选射线、Dashboard、真实 eCAL、C++ Subscriber 和 Recorder；ROS 2/RViz2 分别关闭和开启测量：

| 指标 | 硬门槛 |
|---|---:|
| `sim/wall` | `0.98..1.02` |
| 命令/轮态墙钟频率 | 各 `95..105 Hz` |
| 命令/轮态仿真时间戳频率 | 各 `99..101 Hz` |
| LiDAR/RTK/IMU 墙钟频率 | 各 `9..11 Hz` |
| LiDAR/RTK/IMU 仿真时间戳频率 | 各 `9.9..10.1 Hz` |
| 100 Hz / 10 Hz 最大墙钟间隔 | `<=30 ms` / `<=250 ms` |
| eCAL transport drop | `0` |
| Recorder drop/CRC error | `0` |
| GUI 事件最长空窗 | `<=100 ms` |
| Dashboard draw p95 | `<100 ms` |

这两条 ROS off/on invocation 都必须由实际进程和 profiler 快照证明 `runtime_mode=interactive`、PyBullet 使用 GUI、Dashboard 已启用、实际障碍物为 20、每个正式 LiDAR 帧候选射线为 5,760，并至少取得 5 个 Dashboard draw 样本；CLI 参数回显不能充当 workload 证据。C 阶段的 `headless/DIRECT/dashboard_enabled=false` 核心链路只证明 13.3，不能替代本节任一结果。

不得为通过门禁而降低业务频率、物理 240 Hz、5,760 射线预算或关闭必需记录。离线 20,000 射线 profile 只豁免墙钟实时性，不豁免完整性和确定性。

### 13.5 记录、回放和显示

- 完整 session manifest 的段号、文件 SHA、首尾 fence 和 scene revision 范围连续；每个 MCAP raw/metadata pair 重新读取后的 topic、type、simulation session、descriptor SHA、timestamp、world/command generation、owner source/session、sequence、payload hash 和消息计数与记录时一致。reader/installer 的路径穿越、链接、文件替换与不安全归档成员反例全部失败。
- 队列满、磁盘满、截断和 CRC 损坏均显式失败。
- 隔离回放不发布 wheel command，不污染实时 namespace。
- PCD/PLY 先由锁定 validation prefix 的 PCL CLI 独立打开，再在人工门禁使用目标机声明的常用点云工具确认，坐标系和点数正确；不得调用 PATH 中来源未知的同名程序。
- ROS 2 Bridge 同时输出 Livox `CustomMsg` 和 `PointCloud2`，TF 正确，RViz2 可按距离着色并看出地面与障碍表面。
- 合成 LVX2 可由导出器回读，并在 Livox Viewer 2 中实际打开；sidecar 明确 `synthetic=true`。

### 13.6 GUI 与迁移

- 真实桌面 `:1` 与独立临时 Xvfb `1366x768`、`1920x1080`、`2560x1440` 严格串行；临时 Xvfb 每次自产自清，不操作长期桌面进程。
- GUI、隔离性能矩阵和任何优化候选 profile 都是受控外部门禁：每个 verifier invocation 单独取得用户明确授权并即时扫描全机负载，一次授权只覆盖紧随其后的单条命令。失败保留证据并停止，不自动重跑或继续列表；任何复测都重新授权。
- 每个分辨率都验证 15 个默认页、页签左右滚动与恢复、所有可交互控件实际点击、Qt 文本/tick/legend/artist 完整包含且互不重叠、数据更新前后布局稳定。Dashboard 占可用宽度精确 `33/100`、Main 取余、DPR 对齐和公共边总覆盖继续保留；Dashboard 上下区保持 `50:50` 内部分区。
- LiDAR/RTK 新页面不得恢复已删除的方向按钮，不得暴露默认隐藏的接触/打滑诊断。
- 在干净 Ubuntu 24.04 amd64 电脑完成复制、校验、离线核心安装、启动、C++ 接收、记录、回放、导出、升级、回退和卸载；四类真实运行的原始 MCAP/导出/截图/日志/host inventory 必须随 portable evidence archive 回传并在控制机导入复核。
- 安装运行不得依赖仓库、开发 Conda、`references/` 或未声明的用户主目录文件。

### 13.7 独立六维审查

大阶段实现和自动验收完成后，启动独立只读审查任务，从以下六方面检查：

1. 需求完整性。
2. 逻辑正确性。
3. 边界情况。
4. 代码质量。
5. 测试覆盖。
6. 实际运行结果。

审查任务不直接修改代码。它在仓库外输出带独立 reviewer identity/task id、被审 commit/tree、六维 verdict、全部 findings/disposition 和逐项 path/size/SHA-256 evidence index 的 canonical source；实现 verifier 重新读取 evidence bytes，并只在 `Critical=0, Important=0` 时以一次目录事务同时提交不可变 review JSON/handoff。发现问题后回到实施任务修复并重新验证，旧审查事务保留但失效，修复后必须重新启动独立审查；没有真实 eCAL、真实 GUI/RViz2、Livox Viewer 和干净电脑证据时，不能仅以单元测试宣告阶段完成。accepted-candidate/final-status 只消费已独立复核的 review handoff，交付报告与 README 在 final status 后展示结果，不得反向成为审查或完成状态的输入。

## 14. 文档和兼容性收口

实施时需要同步：

- 根目录权威需求规格：把旧阶段四自动导航替换为本设计范围，并修正总目标、双雷达、RTK 和 ROS 2 边界。
- README：只描述当前已经实现和通过的状态，不提前宣称外部门禁通过。
- 阶段三设计、计划和交付报告：只增加历史替代说明，保留原始证据和旧结论。
- 阶段四交付报告：区分单元、DIRECT、GUI、真实 eCAL、RViz2、Livox Viewer 和干净机证据。
- C++ SDK、eCAL v2 话题、部署、人工测试、记录回放和导出中文教程。

Stage four 完成前，任何旧 v1 PASS、LocalTransport、并发污染失败或环境阻断都不能替代当前 v2 的真实外部证据。

## 15. 参考资料与使用边界

- Eclipse eCAL 文档：<https://eclipse-ecal.github.io/ecal/>
- Livox ROS Driver 2：<https://github.com/Livox-SDK/livox_ros_driver2>
- Livox 官方下载中心：<https://www.livoxtech.com/downloads>
- Livox LVX2 规范：<https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/LVX2%20Specifications.pdf>
- ROS 2 Jazzy 文档：<https://docs.ros.org/en/jazzy/>
- RViz 2 文档：<https://docs.ros.org/en/jazzy/Tutorials/Intermediate/RViz/RViz-Main.html>
- MCAP 文档：<https://mcap.dev/>
- 仓库 `references/` 中的 PyBullet 项目：参考车辆、相机、地形和仿真组织方式，不复制不兼容协议或许可不清晰资产。

阶段四已在 2026-07-31 用官方 GitHub API、精确 `git ls-remote` branch ref 和同 commit 的全部声明许可证文件三方核验并固定 7 个只读源码 reference：`eclipse-ecal/ecal`、`foxglove/mcap`、`facebook/zstd`、`protocolbuffers/protobuf`、`Livox-SDK/livox_ros_driver2`、`Livox-SDK/Livox-SDK2`、`PointCloudLibrary/pcl`。完整 SHA、许可证作用域、观测时 Star 和 focus 路径见 `references/manifest.yml` 与 `references/README.md`。

`ros2/rosbag2` 和 `ros2/rviz` 也完成了评估但不克隆源码：前者不替代本设计的直接 MCAP C++ 主记录链，但 Livox Driver 2 的系统 `rosbag2` 运行依赖仍进入 ROS lock；后者作为 Ubuntu 24.04/ROS 2 Jazzy 系统应用消费，不进入源码构建。若以后需要源码级诊断，只允许固定各自 `jazzy` 分支，不使用默认 `rolling`。reference 的分支阅读快照不等于发行依赖锁；eCAL、Protobuf、MCAP、Zstd、PCL 和 Livox 构建链仍按总计划 Task 2 的目标发布 tag/commit/checksum 独立冻结。

本地官方 MID-360 样例仅用于内部验证：

```text
/home/cancade/Downloads/Livox-MID360-reference/Indoor_sampledata.lvx2
SHA-256 f892732ff43882b56d1cebc683f6ea9374ab3d3ac688368c9d560f49dcd4d647
```

该文件不属于仓库或发行包内容。

## 16. 阶段停止门禁

用户已逐节确认本设计；详细实施计划位于 `docs/superpowers/plans/2026-07-31-stage4-master-implementation.md` 及其 A-E 子计划，正式实施从用户选定执行方式后开始。实施完成后，只有协议、四车型三地形、真实 eCAL+C++、MCAP/回放/导出、GUI/RViz2、Livox Viewer、性能、干净机迁移和独立六维审查全部形成可核对证据，阶段四才可报告完成；否则必须明确列为未完成或环境阻断。
