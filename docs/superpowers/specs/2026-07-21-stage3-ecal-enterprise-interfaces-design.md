# 阶段三 eCAL、企业传感器与接口基线设计

## 1. 目标、前置条件与范围

阶段三在四种车型、三类场地和动态障碍物已经稳定的基础上，接入企业轮子闭环、前后多线雷达、双天线 RTK、IMU、统一仿真时钟和企业 Dashboard，并完成版本化场景文件和全量接口日志。

用户已于 2026-07-21 确认阶段二 GUI 人工测试未发现异常，并明确要求开始阶段三设计，因此阶段三前置门禁已经满足。

发生冲突时，以仓库根目录需求规格和用户当前明确确认的产品边界为准；阶段设计、实施计划和历史验收证据不得反向覆盖需求基线。依此规则，接触力、接触点数和打滑指标属于默认隐藏的内部诊断。

本阶段交付：

- 版本化 Protobuf 消息与集中式话题配置。
- 真实 eCAL 发布/订阅及不依赖 eCAL 的本地测试模式。
- 100 Hz 轮子命令、实际状态反馈和 100 ms 超时保护。
- 前后独立多线雷达及两个 10 Hz 点云话题。
- 10 Hz 双天线 RTK 和 IMU 真值输出。
- 统一仿真时钟、非整数频率调度和逐话题状态统计。
- 同时保留既有实时曲线并展示阶段三接口数据的企业 Dashboard。
- Main GUI 与 Dashboard 按名义 `67:33` 自动平铺的初始窗口组合：Dashboard 宽度精确按有理数 `33/100` 计算，Main 使用余宽。
- 包含车型、场地、障碍物和传感器安装信息的版本化场景文件。
- 全部企业接口消息、非法输入和通信状态日志。

本阶段不实现自动刹停、地图、路径规划、动态避障、导航控制或安全状态机。这些能力严格保留到阶段四。

## 2. 设计选择

### 2.1 分层接口适配

阶段三采用“仿真核心数据对象 + Protobuf 编解码 + 可替换传输适配器”方案：

```text
PyBullet / 传感器
        |
        v
不可变接口数据对象
        |
        v
Protobuf 编解码
        |
        +------------------+
        |                  |
        v                  v
  EcalTransport      LocalTransport
```

仿真控制、调度、超时、传感器和 Dashboard 不直接依赖 eCAL API。真实 eCAL 与本地传输共享消息语义、校验、时钟、状态统计和日志路径。本地模式只用于开发与自动测试，不能冒充 eCAL 已连接。

### 2.2 依赖边界

- Protobuf 是阶段三必需依赖，本地模式也使用同一套消息编码。正式环境统一固定 `protobuf 6.33.6` 与 `grpcio-tools 1.76.0`；二者满足官方 eCAL 6.1 的 `<7` 约束，不能保留 Protobuf 7 后用 `--no-deps` 绕过兼容元数据。
- 正式接口固定 Eclipse `eclipse-ecal 6.1.1` CPython 3.10 wheel 和 Ubuntu 24.04 的 eCAL 6.1.1 官方 PPA 包。Python 适配器使用 `ecal.nanobind_core` 与 `ecal.msg.proto.core`；同名 PyPI earnings-calendar 包或 Ubuntu Evolution calendar 库不属于 Eclipse eCAL。
- eCAL Python 运行时采用延迟导入，使未安装时 `auto` 可明确降级为本地模式；严格 `ecal` 模式和最终验收必须导入官方 6.1.1 并通过真实独立进程门禁。
- `.proto` 是消息源文件；生成的 Python 文件由固定命令再生成，不手工编辑。

## 3. 架构与职责

### 3.1 核心组件

`InterfaceConfig` 集中保存：

- 六个话题名及目标频率。
- `auto`、`ecal`、`local` 传输模式。
- 发布、日志和状态统计的有界队列容量。

车型机械限位保存在 `RobotModelSpec`；传感器安装外参和 LiDAR 扫描参数保存在版本化场景文档。三类配置各有唯一来源，不能互相覆盖。

`InterfaceModels` 提供不可变消息对象、数组顺序、有限数值及机械限位校验。该层不导入 Protobuf、eCAL、Qt 或 PyBullet。

`ProtoCodec` 负责接口对象与生成的 Protobuf 类型之间双向转换，并拒绝格式错误、字段溢出和不满足消息不变量的数据。

`Transport` 定义统一发布、订阅、状态查询和关闭协议：

- `EcalTransport` 持有真实 eCAL participant、publisher 和 subscriber。
- `LocalTransport` 提供确定性进程内发布/订阅，供 TDD、DIRECT 验收和无 eCAL 环境使用。

`SimulationClock` 与 `PeriodicScheduler` 负责统一仿真时间、100 Hz 和 10 Hz 调度。调度使用 `Fraction(1, rate_hz)` 累加期限，不按固定物理步数取模，避免 240 Hz 物理步长造成漂移；整数纳秒时间戳用精确整数商余数实现与正 `Fraction` 相同的 ties-to-even 舍入，不经过浮点数或普通 `+0.5` 舍入。

正常追赶一次完整返回所有已跨越期限，单次安全上限固定为 10,000 条，覆盖当前最高配置 100 Hz 连续 100 秒的追赶。超过 10,000 条视为异常时间跳变，必须在结果分配和调度器状态修改前原子拒绝，不承诺对任意 uint64 跳变无界返回。正常 240 Hz 物理循环每步最多跨过一个 100 Hz 期限；暂停不推进仿真时钟，因此不会积累待补发期限。

`SensorBackend` 是传感器读取所需的窄协议，只暴露关节状态、link/base 位姿、批量射线和命中类别。PyBullet 实现集中转换现有 `ActiveManualWorld`、机器人和障碍物状态，传感器算法不散布 PyBullet 调用。

`SensorSuite` 生成轮子反馈、前后点云、RTK 和 IMU 接口对象。

`InterfaceRuntime` 连接命令 mailbox、控制器、调度器、传感器、传输、状态统计和日志，对主循环提供启动、步进前、步进后、暂停、重绑定和关闭入口。

`InterfaceDashboardSnapshot` 是运行时向 Qt 暴露的不可变组合快照。它在同一生命周期锁内绑定当前 generation、仿真时间、`InterfaceStatusSnapshot`、最近有效命令、最近成功发布的五路输出以及前后 LiDAR 的 `base_link` 俯视投影；其中只保存冻结对象引用，不把 PyBullet、eCAL、Qt 或可变容器带出运行时边界。

`InterfaceEventLogger` 异步保存企业消息和可读事件，不让磁盘写入阻塞物理线程。

### 3.2 线程与主循环

- PyBullet 状态读取、车辆控制、传感器采样和物理步进只在现有物理主线程执行。
- eCAL 回调线程只解码、校验、计数并更新“最新有效命令”mailbox，不直接调用机器人或 PyBullet。
- 主线程在物理步进前消费最新命令、检查超时并执行控制。
- 主线程在物理步进后按仿真期限读取实际状态并生成企业消息。
- 每个 eCAL publisher lane 固定拥有一个 `ready/in-flight` 槽；物理线程向空闲 lane 原子交接首帧，不调用 native send。公开 `outgoing_queue_size` 只限制所有 lane 共享的 latest 合并缓冲，每个状态话题在该缓冲中至多保留一个最新帧；第三帧及后续帧覆盖尚未发送的旧 latest 时准确累计 dropped。固定五个输出话题时，最大受理量为五个 lane owner 加 `min(outgoing_queue_size, 5)` 个 latest，而不是 FIFO。
- eCAL discovery 分别查询轮子命令 subscriber 和五个输出 publisher 的对端状态；命令安全生命周期只由轮子命令对端驱动，Dashboard 的六个话题状态不得复用一个全局 peer 布尔值。
- 正常验收负载下不得丢帧。异常过载时记录被覆盖帧数并把话题标记为降级，不能阻塞物理循环。

### 3.3 双时钟语义

- 所有输出消息时间戳和 100/10 Hz 发布调度使用统一仿真时钟。
- 仿真暂停时不推进仿真时钟，也不发布新的物理状态。
- eCAL 对端发现、连接状态和 100 ms 命令失效保护使用可注入单调墙钟。
- `WheelCommand.timestamp_ns` 用于接口记录和频率验收，不作为安全超时依据，避免发送方时钟异常阻止停车。
- 暂停期间若命令变旧，恢复仿真时先执行超时归零，不恢复旧控制值。

### 3.4 重建生命周期

车型切换、车辆复位或场地重建前，`InterfaceRuntime` 停止物理消息发布、清除旧命令并让车辆安全停车。事务成功后重新解析当前车型的关节语义和传感器 parent link，再恢复调度。eCAL participant 不随 PyBullet 世界重建。事务失败并回滚后绑定恢复的活动世界。

轮速/传感器读取和 `SensorBackend.bind_scene()` 都属于已登记的 world operation。`prepare_world_rebuild()`、wheel-only `rebind_robot()` 和 `close()` 必须先封锁新操作，再等待旧操作退出，才能停车、换绑或释放 backend；rebind 不等待已经进入 transport 的旧 publish，但旧代结果不得提交到新代。接口回调统一登记为 `publish/receive/logger` 三类线程局部上下文；`prepare/rebind/commit/abort/fault/close` 在任一接口回调或当前 lifecycle owner 同线程重入时必须立即拒绝，不进入 condition 等待。其他线程遇到 prepare、rebind 或 close 已取得 ownership 时，按生命周期串行等待后重新竞争。

wheel-only rebind 在 safe-stop 前失败时恢复旧命令准入和旧 active token；safe-stop 成功后的提交副作用不可回滚，异常路径只恢复旧 robot/model/mailbox/subscription 引用，清空旧 mailbox，并进入 `faulted`，保持 `accepting_commands=False`、`world_ready=False` 和 active token 为空。候选 subscription 必须关闭，旧 token 不得重新激活；`close()` 仍须能幂等释放恢复后的旧 subscription。

## 4. Protobuf 与话题契约

### 4.1 版本与文件

消息采用 `proto3`，包名为 `slope_sim.interfaces.v1`。第一版定义：

- `WheelCommand`
- `WheelState`
- `LidarPoint`
- `LidarPointCloud`
- `RtkState`
- `ImuAttitude`

企业消息不增加需求规格以外的业务字段。内部日志 envelope、运行状态和错误信息使用独立内部类型，不污染企业消息。

### 4.2 默认话题

| 方向 | 话题 | 频率 |
|---|---|---:|
| 订阅 | `/sim/wheel/command` | 100 Hz |
| 发布 | `/sim/wheel/state` | 100 Hz |
| 发布 | `/sim/lidar/front/points` | 10 Hz |
| 发布 | `/sim/lidar/rear/points` | 10 Hz |
| 发布 | `/sim/rtk/state` | 10 Hz |
| 发布 | `/sim/imu/attitude` | 10 Hz |

话题名只从 `InterfaceConfig` 读取。企业更换命名规范时只修改配置，不改变消息语义。

### 4.3 轮子消息

`WheelCommand`：

- `uint64 timestamp_ns`
- `repeated float drive_wheel_speed_rad_s`
- `repeated float steering_wheel_speed_rad_s`

`WheelState`：

- `uint64 timestamp_ns`
- `repeated float drive_wheel_speed_rad_s`
- `repeated float steering_wheel_angle_rad`

数组顺序固定：

- 三种差速车型驱动轮为 `[left, right]`，转向数组为空。
- 主动转向车型驱动轮为 `[front_left, front_right, rear_left, rear_right]`。
- 主动转向车型转向轮为 `[front_left, front_right]`。

## 5. 轮子控制与状态

### 5.1 命令校验

每条消息按当前车型原子校验：

- 差速车型必须有 2 个驱动速度和 0 个转向速度。
- 主动转向车型必须有 4 个驱动速度和 2 个转向速度。
- 所有数值必须有限。
- 任一速度超过车型元数据中的机械限位时整条拒绝：全部车型驱动轮为 `20.0 rad/s`，主动转向轮速度为 `2.0 rad/s`。
- 解析失败、数组长度错误、NaN、无穷值或超限消息都不刷新有效命令接收时刻。

非法消息不部分执行，只更新错误状态并记录原因。每条有效消息都进入接收计数；mailbox 可以覆盖旧控制值，但不能丢失频率计数。

mailbox 维护清空代际。异步接收回调在解码或等待前捕获当前 generation，并在提交命令时一并传入；车型切换、车辆复位、场地重建或断线清空会递增 generation。旧 generation 的迟到回调必须静默拒绝，且不能修改有效/非法计数、错误、命令或墙钟状态；清空后的新回调使用新 generation。

### 5.2 控制与超时

有效命令持续作用到下一条有效命令或 100 ms 超时。主动转向速度按物理 `dt` 积分为角度目标，并受对称机械角限制。

超时后：

- 全部驱动轮目标速度归零。
- 全部转向速度归零。
- 当前有限且在机械限位内的转向角成为保持目标，不强制回正。

启动、车型切换、车辆复位和场地重建后必须收到与当前车型匹配的新命令，旧 mailbox 不复用。命令状态为：`等待命令`、`正常`、`非法命令`、`已超时`、`eCAL 未连接`。

### 5.3 单一控制权

- 真实 eCAL 模式下，`WheelCommand` 独占车辆控制，Dashboard 不直接发送另一套线速度/角速度命令。
- 本地模式下，现有键盘状态以 100 Hz 转换为相同 `WheelCommand`，走同一校验、积分和超时路径。
- 不允许 eCAL、键盘和旧 `command_twist` 路径同时争用车辆。

### 5.4 实际状态

`WheelState` 在物理步进后从 PyBullet 实际关节状态生成，不回显输入。车型对应数组长度、顺序和单位与命令契约一致，主动转向输出实际转向角而不是目标角。

## 6. 企业传感器

### 6.1 安装配置

场景传感器配置对运行时安装外参负责：

- 前雷达绑定 `lidar_front_mount`，传感器局部 `+X` 朝车头。
- 后雷达绑定 `lidar_rear_mount`，附加四元数 `(0, 0, 1, 0)`，即 yaw `π`，传感器局部 `+X` 朝车尾。
- RTK 主天线相对 `base_link` 为 `(-0.20, 0, 0.18) m`，副天线为 `(0.20, 0, 0.18) m`，两者使用单位四元数。
- IMU 相对 `base_link` 为 `(0, 0, 0.08) m`，使用单位四元数。

加载或车型切换时验证 parent link 存在、位置有限、四元数有限且非零。场景文件保存 parent link 和相对外参，不能依赖临时 link index。

### 6.2 前后多线雷达

前后雷达独立以 10 Hz 发布。第一版稳定内部参数为：

- 16 条垂直线。
- 每线 180 个水平采样点。
- 水平视场 180 度。
- 垂直视场 -15 至 +15 度。
- 最小量程 `0.10 m`，最大量程 `30.0 m`。
- 每台雷达每帧 2880 条射线。

PyBullet `rayTestBatch` 在发布时刻一次生成整帧。射线起终点先从传感器坐标转换到世界坐标，命中位置再转换回传感器坐标。未命中射线不进入 `points[]`。Dashboard 路径使用同一批世界命中点，并在同一发布时刻额外读取一次 `base_link` 位姿生成俯视投影。

为稳定跳过机器人自身，增加 `0x10` 雷达可见碰撞位：

- 地形和障碍物在保留原物理 group/mask 的基础上加入 `0x10`。
- 机器人不加入 `0x10`。
- LiDAR 射线使用 `collisionFilterMask=0x10`。
- 必须用接触回归证明新增位不改变车辆、地形和障碍物实体碰撞。

### 6.3 点云字段

`LidarPointCloud`：

- `uint64 timebase_ns`
- `string frame_id`
- `uint32 point_num`
- `uint32 lidar_id`
- `repeated LidarPoint points`

`LidarPoint`：

- `uint32 offset_time_ns`
- `float x`、`float y`、`float z`
- `uint32 reflectivity`
- `uint32 tag`
- `uint32 line`

固定语义：

- `timebase_ns` 是扫描开始时间；没有有效点时仍使用当前扫描开始时间。
- `offset_time_ns` 按原始射线顺序均匀分布在 100 ms 扫描周期内，单调不减。
- 第一版几何在发布时刻一次采样，时间偏移只表达扫描顺序，不模拟运动畸变。
- `point_num == len(points)`。
- 前雷达 `lidar_id=1`、`frame_id=lidar_front`；后雷达 `lidar_id=2`、`frame_id=lidar_rear`。
- `line` 范围为 `0..15`。
- `tag`：未知 `0`、地形 `1`、静态障碍物 `2`、移动障碍物 `3`。
- `reflectivity`：未知 `80`、地形 `100`、静态障碍物 `160`、移动障碍物 `200`。

### 6.4 RTK

每次发布把主、副天线固定点转换到世界坐标。`RtkState` 包含：

- `uint64 timestamp_ns`
- `double main_x`、`main_y`、`main_z`
- `double baseline_yaw_rad`

`baseline_yaw_rad = atan2(y_secondary - y_primary, x_secondary - x_primary)`，归一化为 `[-π, π)`。第一版直接输出仿真真值，不模拟卫星、差分解算或噪声。

### 6.5 IMU

`ImuAttitude` 包含：

- `uint64 timestamp_ns`
- `double roll_rad`
- `double pitch_rad`

roll、pitch 由车体世界姿态四元数转换得到，以 10 Hz 发布，第一版无噪声。

### 6.6 故障隔离

前雷达、后雷达、RTK 和 IMU 分别调度、统计和捕获异常。一个话题失败时不发布半帧，也不阻止其他传感器发布。阶段三前的单值 LiDAR 距离摘要只可保留为内部日志或默认隐藏的开发者诊断，不进入企业消息或主 Dashboard；7.1 定义的企业多线点云俯视页不属于该旧摘要。

## 7. Dashboard 与窗口布局

### 7.1 企业页面与一级页签

保留现有 Qt 窗口、结构操作 FIFO、忙碌状态、障碍物表格和下半区主控制区。窗口标题固定为 `3D仿真Dashboard`。上半区使用一个顶层 `QTabWidget`；以下 15 个默认页面均为同一 `QTabBar` 上的一级页签，不使用筛选器、父级图表页或嵌套诊断页：

1. `接口状态`
2. `障碍物`
3. `轨迹`
4. `速度/命令`
5. `驱动命令`
6. `驱动反馈`
7. `转向命令`
8. `转向反馈`
9. `LiDAR点云`
10. `RTK位置`
11. `RTK航向`
12. `IMU姿态`
13. `轮组频率`
14. `传感频率`
15. `接口异常`

点击页签直接切换上半区内容，不存在额外“筛选”或“应用”操作。33% Dashboard 一次无法显示全部页签时启用 Qt 标签栏左右滚动按钮，所有页面仍保持同级。旧业务图仅保留 `轨迹` 和 `速度/命令` 两页；`打滑`、`接触` 及其接触力、接触点数等字段移入默认隐藏的内部诊断。

只有配置显式启用时才额外创建 `开发者诊断` 页；它只保留含接触/打滑的内部遥测表、相机和调参内容，不复制或包裹上述 13 个图表页。默认界面不存在该诊断页。阶段四导航区域不提前创建。

主控制区只保留：

- 仿真控制：暂停/继续、复位、退出，以及 local 键盘驾驶使用的线速度和角速度。真实 eCAL 模式忽略这两个 GUI 速度值。
- 机器人：车型选择和应用。
- 场地：场地选择、坡度或高尔夫参数和应用。
- 障碍物：现有模式、形状、数量、种子、速度、比例和增删清空。

Dashboard 不提供前进、后退、左转、右转或临时停车方向按钮；人工驾驶统一使用键盘方向键，避免按钮遮挡控制区和形成第二套输入保持状态。

### 7.2 接口状态与 Dashboard 快照

Dashboard 只读取不可变 `InterfaceDashboardSnapshot`，Qt 线程不直接调用 PyBullet、eCAL 或传输工厂。组合快照在 `InterfaceRuntime` 的同一生命周期锁内生成，包含：

- 当前 runtime generation 和统一仿真时间。
- 完整 `InterfaceStatusSnapshot`。
- 最近一次有效接收的 `WheelCommand`，以及 runtime 接收它时的统一仿真时间；两者必须同时出现或同时为空。
- 最近一次被传输层成功接受的 `WheelState`、前后 `LidarPointCloud`、`RtkState` 和 `ImuAttitude`。
- 前后雷达在各自扫描时刻转换到 `base_link` 的不可变俯视点集；每点只保留 `x/y`、tag 和 lidar ID，企业 Protobuf 契约不增加显示字段。

快照只引用冻结消息和冻结元组，不在 240 Hz 主循环中深拷贝点云。车型切换、复位、场地重建、断线清空或回滚改变 generation 时，旧最新值和 Dashboard 业务历史必须原子失效。

`接口状态` 页继续显示：

- 全局传输模式和独立 eCAL 状态。
- 本地模式的“本地测试模式”和明确的“eCAL 未连接”。
- 轮子命令状态、有效接收频率和最新输入时间戳。
- 轮子状态实际轮速/转角数组、发布频率和最新仿真时间戳。
- 四个传感器各自的话题状态、发布频率和最新仿真时间戳。
- `活动`、`等待对端`、`超时`、`降级`、`未连接`、`错误`六类逐话题状态。

真实 eCAL 模式逐话题连接状态来自对应 publisher/subscriber 自身的 discovery；一个输出话题没有 subscriber 时只把该话题标为 `等待对端`，不能污染其他输出，也不能代替轮子命令对端触发车辆控制状态。

实际频率按最近 2 秒单调墙钟事件滚动计算，接收和发布分别统计。GUI 人工验收不以窗口打开时长作为频率判据；自动测试使用消息时间戳和计数。

### 7.3 图表内容与时间语义

所有折线页保留最近 20 秒历史，按新时间戳去重并裁剪旧样本。输出消息使用统一仿真时间戳；命令图使用运行时接收该有效命令时的仿真时间，不能让外部发送方的任意时间戳倒置横轴。暂停期间不追加业务图样本。接口质量页使用快照的单调墙钟时间，因此暂停后实际发布频率会在 2 秒窗口内降到 `0 Hz`。

图表按同量纲低密度拆分：

- `轨迹`、`速度/命令` 保留阶段三前的字段、清空和保存行为；接触/打滑只进入显式开发者诊断，不创建默认图表页。
- `驱动命令`、`驱动反馈` 分别最多绘制四条轮速曲线。
- `转向命令`、`转向反馈` 分别最多绘制两条转向速度或转向角曲线；无转向车型显示“当前车型无转向数据”，不制造零值曲线。
- `LiDAR点云` 把前后最新成功帧的 `base_link` 俯视点合并显示，按地形、静态障碍物、移动障碍物着色，并分别显示前后帧时间戳。视野固定为前后 `x=[-48,48] m`、左右 `y=[-30,30] m`，车辆箭头固定在原点并朝 `+X`，叠加 30 m 量程环和类别图例；数据变化不得改变坐标范围或 active axes 几何。它只保存当前图，不提供“清空曲线”。
- `RTK位置` 绘制 `x/y/z`，`RTK航向` 单独绘制 baseline yaw，`IMU姿态` 绘制 roll/pitch。
- `轮组频率` 绘制命令接收与状态发布的实际 100 Hz；`传感频率` 绘制前后雷达、RTK、IMU 的实际 10 Hz。
- `接口异常` 只绘制全部话题每秒新增错误和每秒新增丢帧两条聚合曲线；逐话题累计数仍由 `接口状态` 页显示。generation 或累计计数回退时重建基线，不产生负值或尖峰。

折线历史是实时监控视图，不替代完整 100/10 Hz 二进制接口日志。Dashboard 只保留它实际观察到的新快照；接口日志仍是消息完整性验收的权威数据源。

### 7.4 绘制、性能与异常状态

- 隐藏图表持续接收轻量标量样本，但不提交 Matplotlib 绘制任务；切换页签时立即使用最新缓存绘制。
- 只有当前可见页参与重绘，折线和 LiDAR 点云均受最高 2 Hz 门禁约束。
- 顶部页签区和下方控制滚动区使用 `50:50` stretch；固定最小高度不得覆盖等权分配。正式门禁在 X11 物理像素中独立核对 `1:1`，图表数据变化不得改变 `tabs/controls/page/canvas/axes`，active axes 至少占画布宽 60%、高 50%。
- 每条 `Line2D` 和 LiDAR `PathCollection` 只创建一次，刷新调用 `set_data` 或 `set_offsets`，不重建画布、坐标轴或图例。
- 每个折线页保留“清空曲线”和“保存当前图”；LiDAR 页只保留“保存当前图”。
- 传感器失败时保留最后成功历史，曲线不向后延伸；LiDAR 显示最后成功时间戳，具体错误由 `接口状态` 页呈现。
- 消息类型、数组长度、有限数值或时间顺序不满足图表契约时拒绝该图表样本，不污染其他话题、运行时控制或接口状态。

### 7.5 暂停

暂停停止物理步进和仿真时钟，因此不生成新的轮子、点云、RTK 或 IMU 状态，业务图和 LiDAR 随之冻结。Qt 事件、Dashboard 接口质量、eCAL 对端发现和连接状态继续刷新。恢复前先检查命令墙钟年龄。

### 7.6 67:33 初始窗口组合

手动 GUI（`--gui --manual`）且 Dashboard 启用时，启动阶段读取主显示器 `availableGeometry`，排除任务栏和系统面板。非手动批量实验的 `--gui` 不创建 Dashboard，保持现有实验窗口语义：

- PyBullet Main GUI 位于左侧，使用分配 Dashboard 后的全部余宽。
- Dashboard 位于右侧，宽度精确按可用宽度的有理数 `33/100` 计算。
- 两个窗口顶部、底部对齐，共同占满可用高度。
- 每次启动重新按当前显示器计算，不读取或要求用户恢复上次手工尺寸。
- Dashboard 使用计算后的固定初始尺寸；Main GUI 在启动时传入目标宽高，并在窗口出现后定位。
- X11/XWayland 验收环境按精确 PyBullet 标题和 XRes client PID 双重确认 Main GUI 所有权；窗口管理器 frame 先解析到 client。缺少 `_NET_WM_PID` 时不得退化为纯标题，XRes 不可用、候选歧义或所有权不符时明确失败。
- Dashboard 构造、显示、frame extents 或矩形应用失败时终止本次手动 GUI 运行并清理资源，不能静默退回单窗口全屏模式。
- Dashboard 禁用时 Main GUI 使用全部可用工作区。
- DIRECT 模式不初始化 Qt 屏幕或调用窗口工具。

布局和控件需在 1366x768、1920x1080、2560x1440 三种可用区域下验证无文本重叠；33% Dashboard 宽度不足时使用页签横向滚动、内容换行和纵向滚动，不破坏 `67:33` 名义比例或挤压下半区控制区域。整数像素下 Dashboard 宽度按 `(available_width * 33 + 50) // 100` 取最近整数，Main 使用余宽；精确半像素按 half-up 向上取整。分数 DPR 下先把 `33/100` 目标宽度除以 DPR 并 half-up 到唯一逻辑宽度，再把该逻辑宽度乘 DPR 并 half-up 到唯一物理边界。两个 pane 的总宽、公共边、顶部和底部必须精确闭合。验证器的默认正式路径使用独立 `33/100` half-up/DPR oracle，不复用生产 layout helper，并拒绝旧 20%、30%、36%、固定 420 px 及错误 DPR 近邻宽度。

## 8. 场景配置与加载

### 8.1 版本化场景文档

场景使用独立 YAML，固定 `schema_version: 1`，保存：

- `robot`：车型。
- `terrain`：类型、坡度、高尔夫种子和起伏等级。
- `obstacles`：逻辑 ID、模式、形状、几何、位置、姿态、路径、速度、进度和方向。
- `sensors`：前后雷达、主副天线和 IMU 的 parent link、外参及 LiDAR 扫描参数。

不保存 PyBullet client、body ID、link index、Qt 对象或 eCAL 句柄。话题与传输配置属于 `InterfaceConfig`，不混入可复现场景文档。

启动时未指定场景文件，则由现有 `ExperimentConfig`、空障碍物集合和本设计确定的传感器默认值构造同一个 `schema_version: 1` 内存文档；运行时和导出路径不维护第二套场景语义。

### 8.2 导出与加载

- 导出从当前世界和不含 body ID 的障碍物快照生成文档。
- 写入临时文件并原子替换目标文件，避免半写入。
- 加载先完成 YAML 结构、版本、枚举、有限数值、四元数、车型/link、障碍物 ID 和边界校验。
- 全量校验通过后才进入协调器事务。
- 目标场景任何创建或恢复失败时回滚到原车型、场地、障碍物和传感器绑定。
- 未知版本明确拒绝，不猜测兼容。
- 第一版通过 Python API 和命令行参数导入/导出，不向企业 Dashboard 添加规格外按钮。

相同场地种子、障碍物快照和传感器配置重新加载后必须得到相同逻辑场景；重建产生新的 PyBullet ID 是预期行为。

## 9. 接口日志

### 9.1 二进制消息流

使用长度前缀 Protobuf 二进制日志完整记录：

- 每条有效 `WheelCommand`。
- 每条成功提交给传输层的 `WheelState`。
- 每条成功提交的前后 `LidarPointCloud`、`RtkState` 和 `ImuAttitude`。

内部 envelope 保存话题、方向、仿真时间、接收墙钟、Protobuf 类型名和原始 payload。日志读取器能按顺序恢复原消息并检查类型。

### 9.2 可读事件日志

JSONL 记录：

- Protobuf 解析失败和非法命令原因。
- 命令超时、车型不匹配和机械限位拒绝。
- eCAL 初始化、断线、重连和关闭。
- 传感器异常、发布失败、日志队列或传输队列丢帧。
- 当前车型、场地和相关话题。

现有车辆 CSV 与障碍物 JSONL 保留。接口二进制日志使用独立有界写入队列，关闭时刷盘。目标验收负载下接口日志丢帧必须为零；异常过载时记录准确丢帧数并显示降级，不能等待磁盘而冻结 Dashboard 或物理循环。

## 10. 传输模式与错误处理

### 10.1 模式

- `auto`：默认。尝试 eCAL；不可用时进入本地模式，记录原因并明确显示“eCAL 未连接”。
- `ecal`：严格正式模式。eCAL 导入、初始化或接口创建失败时安全清理并退出。
- `local`：显式测试模式，不导入 eCAL 运行时。

`LocalTransport` 的用户回调不得在 transport 或 subscription 锁内执行。统一生命周期锁只负责登记 in-flight、计数和关闭状态；外部 `close()` 是等待已启动回调结束的屏障，回调上下文内的关闭只原子禁止新交付并返回，避免回调互相关闭形成循环等待。任一全局关闭开始后，旧发布快照都不能再启动新回调。

### 10.2 断线与重连

eCAL 对端消失只更新接口状态，不重建或销毁 PyBullet 世界。对端恢复后继续发布新物理状态；断线前的轮子命令不能恢复。发布错误按话题计数，并在传输恢复后清除活动错误但保留累计统计。

新建 production session 时，relay 在自身非重入锁外先执行一次 `poll_peer_state()`，再进入锁读取 `snapshot()` 并绑定 runtime；周期刷新同样固定为先 poll、后 snapshot。这样初始化和运行期都不会读取 discovery 之前的旧状态，也不会因 poll 同步触发 callback 而自锁。

自动实验、GUI 手动入口和独立 eCAL 仿真进程共同持有 `RuntimeObservationCadence`。它以 50 ms 绝对墙钟周期调用 `poll_transport()`；第一次循环、暂停恢复、world 重建及协议屏障结束后立即观测一次。慢 poll 的下一期限从 poll 完成墙钟起算，迟到只执行一次且不补跑；非观测帧仍返回新墙钟，使 `before_physics_step(..., wall_time=...)`、100 ms 超时和 240 Hz 物理步保持逐帧执行。Qt 的 `InterfaceDashboardSnapshot` 只在该观测边界构造，不在 240 Hz 循环重复复制组合状态。

discovery 与 publisher lane 使用不同的 gate。每次已受理的 poll 登记 in-flight 和单调 revision；count API 返回后只有比当前已提交 revision 更新的观察才能改变逐话题状态或 mailbox generation。迟到旧观察、并发 poll 和 callback 内重入均不能把新连接状态回退。

### 10.3 关闭顺序

正常退出和异常清理统一执行：

1. 停止接受新命令。
2. 清零驱动并保持安全转向角。
3. 停止传感器调度和物理消息生成。
4. quiesce 传输线程，禁止新交付并取得包含关停 pending frame 的最终逐话题质量快照。
5. 持久化 logger/transport 的聚合 `queue_dropped` 终态事件，刷盘并关闭接口日志。
6. 等待所有在途 discovery/count API 返回，再移除 subscriber callback、释放 subscriber/publisher 引用并 finalize participant。
7. 清理传感器临时资源并断开 PyBullet。

各关闭操作幂等。部分初始化失败也使用同一清理路径，不留下 PyBullet client、后台线程或打开文件。

## 11. 测试与验收

### 11.1 TDD 与任务门禁

每个实施任务先写失败测试，再写最小生产实现。任务完成后先做规格符合性审查，再做代码质量审查。审查发现的问题补回归测试后修复，不降低既定频率、超时、误差或性能门槛。

### 11.2 单元测试

- Proto descriptor 字段、编号、类型和 Python round-trip。
- 集中话题配置和重复话题拒绝。
- 四种车型数组长度、顺序、NaN/Inf 和机械限位拒绝。
- 可注入墙钟下的等待、有效、非法、超时和重建清空状态。
- 240 Hz 步进下长时间 100 Hz 与 10 Hz 累加调度，无固定步取模漂移；暂停不推进仿真时钟。
- 周期调度完整返回 257 条和 10,000 条合法追赶，10,001 条及更大异常跳变在分配前原子拒绝；3 Hz 等非整除频率和半纳秒边界精确匹配 `Fraction` 的 ties-to-even 结果。
- 本地传输的订阅、发布、关闭、计数和 bounded mailbox 语义。
- transport quiesce、关闭回调竞态、logger/transport 两类丢帧的持久化 `queue_dropped` 事件。
- 六话题独立 eCAL discovery，不允许轮子命令 peer 状态覆盖输出话题连接状态。
- 世界/传感器坐标转换、前后朝向、点云字段、线号、时间偏移、tag 和反射率。
- RTK `atan2`、yaw 归一化及 IMU 四元数转换。
- 场景导出 round-trip、未知版本、非法数据和原子写入。
- 接口二进制日志 round-trip 和事件 JSONL。
- `InterfaceDashboardSnapshot` 的冻结映射、成功消息语义、原子 generation 和生命周期清空。
- 前后 LiDAR 到 `base_link` 的俯视投影、tag/lidar ID、空点云及固定 5760 射线上界。
- Dashboard 新时间戳去重、20 秒裁剪、逆序/非有限样本隔离及四种车型数组语义。
- 15 个默认一级页签的精确顺序、13 个默认图表页、接触/打滑诊断边界、图表线数和单位边界。
- 隐藏页只缓存不绘制、当前页最高 2 Hz、保存/清空行为和 LiDAR 单次 artist 更新。
- Dashboard 状态格式、2 秒频率窗口、接口异常增量基线和窗口布局计算。

### 11.3 DIRECT 集成

- 四种车型分别接收合法轮速，验证运动方向、实际关节反馈和数组契约。
- 主动转向速度积分、角度限位、当前角保持和 100 ms 超时。
- 三类地形下前后点云坐标和地形命中。
- 静态和移动障碍物分别进入前后雷达视场并改变对应点云。
- 雷达可见碰撞位不改变车辆与地形/障碍物接触。
- RTK 主天线位置、基线 yaw、IMU roll/pitch 与 PyBullet 真值误差不超过 `1e-4`。
- 暂停不产生新物理消息，恢复不执行旧命令。
- 车型/场地重建、失败回滚、传感器异常和关闭无残留资源。
- 场景文件加载后种子、障碍物逻辑状态和传感器配置复现。
- 有效命令和成功发布输出进入 Dashboard 快照，编码、传输或旧 generation 失败不更新最新值。
- 车型切换、复位、场地重建和回滚后，Dashboard 不混用旧数组、旧点云或旧计数基线。

### 11.4 真实 eCAL 集成

使用独立进程 publisher/subscriber 验证：

- 六个默认话题和 Protobuf 类型可被真实 eCAL Monitor/客户端识别。
- `WheelCommand` 有效接收和 `WheelState` 发布平均 100 Hz。
- 两路点云、RTK、IMU 各自平均 10 Hz。
- 频率根据消息计数和时间戳判定，不用 GUI 打开时长替代。
- 非法命令拒绝、发送停止后超时停车、对端退出和重新启动状态正确。
- 正式双进程 simulation 门禁分别覆盖主动转向 `active_steering_4wd` 的 `4+2` 命令和代表性差速 `df_back` 的 `2+0` 命令；两端必须报告同一车型，命令事件基数与真实关节反馈都由 verifier 独立核对。
- 最终阶段验收不得用 `LocalTransport` 结果替代真实 eCAL 结果。

### 11.5 性能门槛

项目验收机上，一个机器人、前后雷达和 20 个障碍物运行时：

- 真实 eCAL、20 个障碍物和接口日志必须在同一个预热 1 秒、测量 5 秒的生产 session 中运行，不能拼接 local 或空场景结果。
- 每 100 ms 原子记录日志 `pending` 与 `completed=accepted-pending`。队列实际增长连续 1 秒，或正深度下 writer 完成数停滞 1 秒，均视为持续积压；稳定的单项 in-flight 且完成数持续前进不误报。
- 六话题 accepted 消息数至少达到名义总量的 90%，测量结束后日志必须在有界时间内回到 pending=0；传输队列和接口日志队列丢帧数均为零。
- Dashboard 事件处理不出现超过 100 ms 的单次阻塞。
- 所有隐藏图表页均不产生 Matplotlib 绘制，当前图表页重绘不超过 2 Hz；切换页签后只绘制新当前页。
- LiDAR 点云仅在其页签可见时转换为 Matplotlib offsets，运行时快照不按主循环频率深拷贝点云。
- 前后雷达每帧各 2880 条射线、错相 50 ms、消息时间戳共享 100 ms 周期；每个扫描 deadline 冻结一次安装位姿，并用一次 2880-ray `rayTestBatch` 原子生成该发布时刻的整帧。同步双批射线的 80 ms 中位数预算只作为兼容 API 和后端微基准，不能据此把生产帧拆到多个物理时刻。
- PyBullet 后端优先使用紧凑 `ray_test_indexed_hits` 和预验证批量逆变换；headless session 只生成企业点云，只有 GUI Dashboard session 才生成 `base_link` 俯视副本。
- 真实 eCAL 循环以绝对 deadline 追赶；超期帧只用 `sleep(0)` 让出执行权，不再叠加固定正延时。循环期间只暂停 cyclic GC，退出后按调用方原状态恢复，普通引用计数释放不受影响。
- 打开页面、添加障碍物、切换车型或场地时通信调度能够恢复到目标频率。

### 11.6 GUI 与窗口验收

- 在真实 X11/XWayland 桌面通过窗口系统读取 Main GUI 和 Dashboard 实际几何。
- 两个窗口覆盖主屏可用工作区，Dashboard 宽度精确按 `33/100` 计算且 Main 取余，顶部和底部对齐，不要求用户手工拖动。
- Dashboard 标题精确为 `3D仿真Dashboard`；15 个默认页面均为同一顶层标签栏的一级页签，接触/打滑不出现在默认页签或默认可见文字中。
- 默认正式 smoke/verifier 必须消费 Dashboard 的 schema v4 布局报告，实际点击页签栏左右滚动按钮后遍历两轮全部 15 个默认页签；reader 只能返回全局 JSONL 行游标之后的新 occurrence。正式全图表验证要求 client 逻辑高度至少 600px；更矮窗口只做 compact 可达性检查。独立 oracle 核对根布局 `8px` 边距、`6px` 间距、两个 pane 等高且共同铺满剩余高度，canvas 至少覆盖 page 宽 85%/高 70%，axes 至少覆盖 canvas 宽 60%/高 50%。第二轮图表 `rendered_data_revision` 必须增长，`tabs_rect`、`controls_rect`、`page_rect`、`canvas_rect`、`axes_rect` 必须与第一轮一致；同时验证两个数据页正常渲染、13 个图表画布非空，title/xlabel/ylabel/科学计数 offset/tick/legend artist 和 Qt 文字不重叠且完整包含，线速度、角速度、六个障碍物数值控件和关键按钮完全进入 viewport，并实际点击完整控件路径。
- 驾驶门禁只使用键盘路径；Dashboard 方向按钮和 button 输入模式不再属于产品或验收合同。子进程启动预算不得缩短实际持续驾驶窗口，所有按键必须在 `finally` 中释放。
- 验证 1366x768、1920x1080、2560x1440 下无文本重叠、不可达控件或上下区域互相遮挡。
- PyBullet Main GUI 候选必须通过 XRes client PID 所有权验证；并发同标题窗口、缺失 XRes 或 Dashboard 构造失败都必须使门禁非零退出。
- eCAL Monitor 可见轮子、前后点云、RTK、IMU 话题与频率。
- 人工发送差速和主动转向命令，观察运动、转向、超时和实际反馈。
- 在静态/移动障碍物及三类地形中观察点云、RTK 和 IMU 变化。
- 持续按键驾驶时依次切换旧折线、新接口折线和 LiDAR 点云，既有车辆位移门禁仍通过。
- 暂停、复位、重建和断开发送端时，接口状态与安全行为正确。

### 11.7 全量回归与独立审查

阶段三验收前运行阶段一矩阵、阶段二障碍物脚本、阶段三接口脚本和全量 pytest。随后启动独立只读审查线程，从以下六方面复核：

1. 需求完整性。
2. 逻辑正确性。
3. 边界情况。
4. 代码质量。
5. 测试覆盖。
6. 实际运行结果。

审查线程不直接修改代码。主实施线程根据结论补测试、修复并重新执行相关验收。

## 12. 实施协作与并行边界

主线程负责需求、共享契约、集成顺序、最终代码合并和验收证据。可并行任务优先委派给子智能体，以缩短墙钟时间并隔离大量测试输出。

适合并行：

- 只读参考仓库和 eCAL API 调研。
- 相互独立的单元测试审查、测试运行和日志分析。
- 已由共享接口固定边界的 Protobuf、传感器数学、场景序列化或 Dashboard 展示子任务。
- 最终六维独立只读审查。

必须串行或先后执行：

- `.proto`、`InterfaceModels`、`InterfaceConfig` 等共享契约先由主线程确定。
- 同一文件或同一运行时生命周期的写入不并发。
- eCAL、物理主循环、重建事务和关闭顺序由主线程统一集成。
- 所有子智能体结果回到主线程复核，主线程运行聚焦测试和完整回归后才可声明完成。

并行智能体不得自行降低验收阈值、改变企业消息字段或提前实现阶段四功能。

## 13. 实施顺序

1. Protobuf、接口配置、不可变数据对象和编解码。
2. 仿真时钟、周期调度、本地传输和状态统计。
3. 轮子命令校验、控制、实际反馈和超时保护。
4. 真实 eCAL 适配器及独立进程环回。
5. 前后点云、RTK 和 IMU。
6. 场景导入导出、接口日志和故障恢复。
7. 企业 Dashboard 不可变快照、15 个默认一级页签、诊断边界、67:33 窗口布局及运行时集成。
8. 自动验收、文档、独立审查和修复复验。

依赖顺序内的任务串行推进；已经稳定且写集互不重叠的子任务可以并行。

## 14. 开源参考与解释责任

- 参考 Eclipse eCAL Python 和 Protobuf 示例的 participant、publisher/subscriber 与生命周期模式。
- 参考 Livox ROS Driver 2 `CustomMsg/CustomPoint` 的时间基准、点字段和线号语义，不复制 ROS 依赖。
- 参考 UM982 双天线 moving-baseline heading 语义，不模拟真实 GNSS 解算。
- 参考仓库内 `references/repos/pybullet_sim` 的批量射线和传感器组织方式。
- 使用 Bullet 现有碰撞组、link state、`multiplyTransforms`/逆变换和 `rayTestBatch` 时，实施交付报告需用面向 PyBullet 初学者的中文解释关键操作。

复制任何参考代码或资源前检查许可证；优先借鉴接口和算法结构。

## 15. 阶段停止门禁

开发方完成自动验证、真实 eCAL 集成、GUI 窗口验证和六维独立审查后，提交阶段三交付报告、完整测试结果、eCAL/GUI 操作步骤和已知限制，然后停止开发，等待用户人工验收与反馈。

只有用户明确确认阶段三通过并提出开始阶段四，才允许实现自动刹停、路径规划和动态避障。
