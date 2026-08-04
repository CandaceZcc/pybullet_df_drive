# 阶段四 P0 异步 LiDAR Worker 设计

> 设计日期：2026-08-03
>
> 设计状态：用户已批准；独立规范复核问题已纳入，等待实现
>
> 基线分支：`agent/stage3-final-acceptance`
>
> 直接证据：`results/stage4/p0-active-steering-4wd-retest-20260803T161620+0800/`

## 1. 权威关系与适用范围

本文是阶段四总计划 P0 的窄范围补充设计，只处理阶段三真实 eCAL `4+2` 门禁中 LiDAR 阻塞物理主循环的问题。发生冲突时，本文只在以下两点覆盖 `2026-07-31-stage4-mid360-ecal-cpp-delivery-design.md`：

- 实际 transport 为 eCAL 的墙钟节拍生产会话不再由主物理进程同步执行 LiDAR `rayTestBatch`、点云构造和 Protobuf 编码。
- LiDAR 可以在采样时刻冻结世界快照后异步完成并稍后发布，跨话题墙钟发布顺序不再要求与仿真时间戳顺序一致。

阶段四其余协议、MID-360、C++、ROS 2、记录、导出、依赖锁和发行设计仍以原设计与五个子计划为准。本文不把 P0 修复提前解释成阶段四 Task 2 或 A-E 已经开始。

## 2. 问题与证据

2026-08-03 16:16 获授权的唯一一次 `active_steering_4wd 4+2` 真实门禁完整退出，但结果为 FAIL：

- peer 发送和 runtime 接收均为 500 条 WheelCommand，runtime 最大 command 间隙为 `14.367 ms`。
- `/sim/wheel/state` peer 最大接收间隙为 `33.030 ms`，超过既有 `25 ms` oracle。
- 相邻 wheel-state 仿真时间戳连续，因此不是消息缺失。
- 五路输出 publish/receive 数量逐项相等，transport 和 logger 均为零 drop/error，双方 clean shutdown。
- `1152` 个物理步推进 `4.8 s` 仿真时间，`sim/wall=0.9566249594`。

当前调用顺序是：发布本步 wheel-state，随后在 `InterfaceRuntime.after_physics_step()` 内同步扫描到期 LiDAR，扫描返回后才能开始下一物理步。正式世界的附加只读 profiler 已把长帧定位到生产链：

- 单次 `rayTestBatch` 最大约 `23.8 ms`。
- 后雷达完整扫描最大约 `29.5 ms`。
- `after_physics_step()` 最大约 `32.6 ms`。

此前的 `1 ms` Python switch interval 已解决 command callback 的 GIL 饥饿，但不能缩短同步扫描占据的整体墙钟时间。分片 `rayTestBatch` 只能增加片间调度点，不能让下一物理步提前发生，因此不满足本问题的 wheel-state 门禁。

## 3. 目标、冻结合同与非目标

### 3.1 目标

- 从 240 Hz 主物理循环移出 LiDAR raycast、点构造和 Protobuf 编码。
- 保留前后雷达各 10 Hz、错相 50 ms、每帧 2880 条射线和既有 v1 wire schema。
- 每帧仍对应单一冻结世界状态，不拼接跨物理步扫描，不模拟运动畸变。
- 保留 `InterfaceRuntime` 的 scheduler、generation、tracker、logger 和 transport 所有权。
- 保持 wheel、RTK、IMU、命令 watchdog、安全停车和 eCAL verifier oracle 不变。
- 在本地真实 DIRECT 门证明充分余量后，才消耗新的单条真实 eCAL 授权。

### 3.2 冻结合同

- LiDAR 消息继续携带原采样 deadline 的仿真时间戳，不使用完成墙钟或发布时间替代。
- 前后话题、type name、点字段、顺序、分类、反射率和 deterministic Protobuf bytes 语义不变。
- Dashboard 的俯视点必须与同一企业点云来自同一个 worker 扫描结果。
- transport 仍只接受 bytes；不向 `Transport` 协议增加 worker、队列或 lifecycle 方法。
- 日志和 eCAL 发布使用 worker 返回的同一份 payload，禁止父进程再次编码另一份 bytes。
- 旧 generation 或暂停 epoch 的迟到结果不得污染当前 tracker、Dashboard、logger 或 transport。

### 3.3 非目标

- 不降低射线数、扫描频率、碰撞类别或点云精度。
- 不修改 P0 的 `25 ms`、频率、sim/wall、drop、日志、fence 或 clean shutdown oracle。
- 不并发访问主进程的 PyBullet client，也不把同一 client 交给 Python 线程。
- 不引入 C++ 扩展、定制 PyBullet wheel、Embree、CUDA、ROS 2 或新运行依赖。
- 不把无节拍 local/DIRECT 模式改成依赖墙钟吞吐的异步模式。
- 不在本设计内开始阶段四 Task 2，或修改现有 reference admission 和依赖锁范围。

## 4. 方案选择

### 4.1 采用：单个持久化 shadow PyBullet 进程

一个以 `multiprocessing.get_context("spawn")` 创建的非 daemon 子进程维护独立 PyBullet DIRECT 世界。它串行处理前后 LiDAR 请求；主物理进程只捕获不可变快照并非阻塞提交。

选择单进程的理由：

- 前后雷达已错相 50 ms，当前最长完整扫描约 30 ms，单 worker 有明确初始余量。
- `rayTestBatch(numThreads=0)` 已使用 Bullet 内部并行；两个进程重叠运行会争抢 CPU，反而扩大尾延迟。
- 只有一个镜像世界、一个协议状态机和一条关闭链，代际、回压和清理更易证明。
- 继续使用已有 PyBullet native raycast，不新增阶段四 Task 2 之前无法锁定的 ABI。

### 4.2 不采用的方案

| 方案 | 不采用原因 |
|---|---|
| 每个 LiDAR 一个子进程 | 复制世界和生命周期，可能让两个 native 扫描同时占满 CPU；当前没有单 worker 吞吐不足的证据 |
| 同步分片 `rayTestBatch` | 可以让出 GIL，但完整扫描结束前仍不能开始下一物理步，不能解决 wheel-state `33.030 ms` 空档 |
| Python 线程访问主 PyBullet client | 同一 client 与 `stepSimulation` 并发没有线程安全合同；加锁后仍恢复为同步阻塞 |
| 定制 PyBullet/Bullet binding | 释放 GIL不等于缩短扫描墙钟；还会引入 wheel、ABI、构建锁和发行维护 |
| 新原生射线引擎 | 需要复制全部碰撞几何、数值语义和动态同步，范围远大于 P0 |

若单 worker 在正式本地门内无法同时满足吞吐和延迟，必须回到设计评审；不得在实现中静默升级到双进程或降低 LiDAR 合同。

## 5. 激活范围与所有权

### 5.1 激活规则

`create_interface_session()` 在 transport 创建后读取其实际 mode：

- 实际 mode 为 `ecal`：创建并注入 `LidarScanService`。这覆盖严格 eCAL、`auto` 成功取得 eCAL、自动 DIRECT eCAL、手动 GUI eCAL 和 P0 正式 runtime；这些入口已有墙钟 `DeadlinePacer`。
- 实际 mode 为 `local`：不创建 worker，继续使用当前同步 `MultiLineLidar.scan()` 路径。这样保留无节拍 DIRECT、阶段三 verifier、单元测试和本地快速仿真的既有语义。
- worker 的真实 DIRECT 测试和预检脚本显式构造生产 service；不得用同步 local 路径冒充 worker 验收。

不新增用户可调的执行模式开关。worker 是 eCAL 生产会话的内部实现，不形成新的公开配置组合。

`auto` 只允许在现有 `create_transport()` 内因 eCAL binding 不可用而降级。transport 已成功选择 eCAL 后，worker 构建、握手或运行失败必须使 session 初始化或对应 LiDAR service 显式失败，不能二次降级到 local。

### 5.2 所有权

- `InterfaceRuntime` 取得 `LidarScanService` 的唯一生命周期所有权。
- `create_interface_session()` 在 service ready 后、runtime 构造成功前仍由局部初始化事务拥有 service；runtime 构造抛错时入口直接关闭它。runtime 构造成功即完成所有权转移；之后 relay attach 或 peer 初始化失败只能通过 `runtime.close()` 回收 worker，不得双重关闭或泄漏 child。
- 子进程只拥有自己的 PyBullet client、镜像 body id、scanner、codec 和 IPC endpoint。
- 子进程不初始化 eCAL、logger、Dashboard、Qt 或主世界 coordinator。
- 父进程不读取子进程 body id，不把主世界 body id 发送给子进程。
- `InterfaceSession.close()` 继续先关闭 runtime；runtime 必须在 transport quiesce 和 logger 终结前关闭 worker。

## 6. 组件与内部合同

新增 `slope_sim/lidar_worker.py`，文件头和关键函数使用简短中文注释。组件固定为：

### 6.1 `LidarScanService`

父进程私有 facade，职责只有：

- 启动、握手和拥有子进程。
- 校验并提交请求。
- 维护一个 in-flight 槽和一个 pending 槽。
- 非阻塞收取、严格校验并返回结果。
- 处理 pause epoch、world generation、flush 和 close。
- 按顺序暴露只可消费一次的 typed outcome，供 runtime 精确归因。
- 暴露不可变状态快照，供 runtime 状态和验收读取。

它不读取 PyBullet、不编码点云、不直接调用 transport 或 logger。

### 6.2 IPC 值

内部值使用 `frozen=True, slots=True` dataclass，并在两端做精确类型和全字段校验：

```text
LidarWorkerWorldSpec
  protocol_version
  experiment_config
  scene_document
  world_digest

LidarWorkerReady
  protocol_version
  process_id
  world_digest
  prewarmed_topics
  prewarm_payload_sha256_by_topic
  prewarm_max_scan_wall_duration_ns

LidarWorkerStartupFailure
  protocol_version
  process_id
  phase
  stable_error_code
  bounded_detail

LidarScanRequest
  protocol_version
  job_id
  captured_monotonic_ns
  lifecycle_generation
  pause_epoch
  topic
  frame_id
  lidar_id
  timestamp_ns
  world_mount_pose
  optional_base_pose
  complete_obstacle_snapshots_without_body_ids

PreparedLidarFrame
  protocol_version
  job_id
  lifecycle_generation
  pause_epoch
  topic
  timestamp_ns
  message
  optional_top_view
  protobuf_payload
  scan_wall_duration_ns

PreparedLidarPayload
  protocol_version
  job_id
  lifecycle_generation
  pause_epoch
  topic
  timestamp_ns
  protobuf_payload
  scan_wall_duration_ns

LidarScanFailure
  protocol_version
  job_id
  lifecycle_generation
  pause_epoch
  topic
  timestamp_ns
  stable_error_code
  bounded_detail
  scan_wall_duration_ns

LidarServiceEvent
  sequence
  kind
  scope
  optional_topic
  optional_job_identity
  stable_error_code
  bounded_detail
```

IPC `protocol_version` 首版固定为整数 `1`。`stable_error_code` 只允许：`scene_reconcile_failed`、`scene_state_unknown`、`raycast_failed`、`pointcloud_failed`、`codec_failed`、`worker_start_failed`、`worker_preflight_failed`、`worker_protocol_failed`、`worker_exited`、`sensor_overrun` 和 `worker_shutdown_failed`；detail 必须是单行 UTF-8 文本，编码后最多 512 bytes，超长时在 worker 端截断且不能携带 traceback 或任意异常对象。

`LidarServiceEvent.kind` 固定为 `frame_failed`、`capture_rejected`、`job_overrun`、`service_failed`、`retired_cleanup_failed` 五种；`scope` 只允许 `topic` 或 `service`。service 为每个事件分配从 1 开始连续递增的 sequence，`drain_events()` 原子取走当前有序事件并从内部队列删除，重复调用不得重复返回。runtime 只通过这些事件更新 topic error/drop 或 lifecycle event；累计 snapshot 只用于诊断，禁止用累计差值猜测归因。

`LidarWorkerStartupFailure.phase` 只允许 `world_build`、`front_preflight`、`rear_preflight` 和 `startup_cleanup`。`world_build`、`front_preflight`、`rear_preflight` 唯一对应 `worker_preflight_failed`；`startup_cleanup` 唯一对应 `worker_start_failed`。父进程 `Process.start()` 同步抛错也归为 `worker_start_failed`，但它没有 child envelope。child 在 ready 前失败时必须尽力发送一个 exact startup failure、关闭 DIRECT client/endpoint 后以非零状态退出；一次启动只能观察到 Ready 或 StartupFailure 之一。父进程收到 EOF、异常 exit 或超时但没有合法 envelope 时统一归为 `worker_exited`，不得伪造具体 preflight phase。

`LidarWorkerWorldSpec` 只在 spawn 启动时传递。`experiment_config` 必须是父生产世界同一份重新构造并全量验证的 exact `ExperimentConfig`；child 只用它调用现有 world builder，不读取其 interface、GUI、日志或输出路径字段，也不据此创建这些资源。`scene_document` 同样重新构造验证，不能信任绕过 frozen dataclass 的对象。

`PreparedLidarPayload` 是 headless request（`optional_base_pose is None`）唯一允许的成功 response。它不携带 `LidarPointCloud`、`LidarTopViewFrame` 或任何逐点 Python 对象；worker 仍在 child 内从同一次扫描构造企业 message 并只 encode 一次，再把同一 exact payload 连同严格身份返回。parent 只复核 compact response 的版本、identity、非空 exact bytes 和时长，随后以固定 `LidarPointCloud` type name 直接交给既有 transport/logger 提交内核，不 parse、不 decode、不重编码，也不写入 headless Dashboard latest。这样 Pipe 接收成本与 bytes 大小线性相关，不再包含 2880 个点和 top-view 的对象反序列化/重构。

Dashboard request（`optional_base_pose is not None`）继续使用 `PreparedLidarFrame`，保留 message/top-view 的同源原子性与现有 GUI 行为。该完整 response 不作为 headless P0 realtime verifier 的通过依据；若未来 GUI eCAL 也需要同一 heartbeat 门，必须另行设计 UI 线程 payload 解码或有界展示数据面，不能在 parent 物理线程恢复完整点云 decode。

`world_digest` 固定为候选 `SceneDocument` 经 `document_to_mapping()` 后，以 `json.dumps(sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)` 生成 UTF-8 bytes，再计算小写 SHA-256。LiDAR 可见地形和障碍物几何全部来自 `SceneDocument`；friction、time step 和不可见镜像机器人 dynamics 不进入 digest。父进程把同一 world spec 交给 spawn child；两端独立复算并在 ready 握手逐字比较。每帧动态障碍物姿态仍由完整 request 覆盖，不重新定义 startup digest。

父子各使用一条单向 `multiprocessing.Connection`，不使用带 feeder thread 的 `multiprocessing.Queue`。父进程一次只把当前 in-flight 请求写入 OS pipe；pending 保留在父进程内，因此容量不依赖 pipe buffer 或不可靠的 `qsize()`。

子进程是受信任的同包代码，IPC 可使用标准 multiprocessing pickle；父进程仍必须拒绝错误精确类型、协议版本、字段、topic/frame/lidar 对应关系和顺序。`poll(0)` 只保证不会等待 raycast 完成，不把 pipe 传输和反序列化误写为零成本；完整 `recv()`、校验和反序列化都计入主循环 `20 ms` 本地预算，并由真实大点云测试锁定。

### 6.3 冻结位姿扫描入口

`MultiLineLidar` 新增包内窄入口，接受已验证的 `world_mount_pose` 和可选 `base_pose`：

- 复用现有固定射线表、`_indexed_hits()`、逆变换、点分类和 `LidarScanResult` 构造。
- 不调用 backend `world_pose()`，因此不会读取 worker 镜像机器人的瞬时姿态。
- 原有 `scan()` 和 `scan_with_top_view()` 仍先读取当前主世界位姿，再调用同一内部实现。
- 不增加 rolling、incremental 或跨帧公开 API。

### 6.4 预编码发布入口

`InterfaceRuntime` 增加私有 prepared-LiDAR 发布路径：

- 复核 message、top view 的身份字段和 job identity，并要求 payload 是非空 exact `bytes`。
- 使用现有 generation、topic tracker、latest payload、logger 和 transport 提交流程。
- 直接使用 `PreparedLidarFrame.protobuf_payload`，不得再次调用 LiDAR codec encode。
- transport 拒绝、logger 拒绝和 tracker 计数保持现有语义。

对 `PreparedLidarPayload`，运行时只复核 compact identity 和 payload，再以 `LidarPointCloud` 固定 type name调用同一 encoded publish 内核；headless 分支不要求 message/top-view，也不触发 Dashboard latest 更新。

父进程不在实时路径重新 parse 或 encode payload 来证明它和 message 相等；这项同源关系由 worker 在构造 response 前建立，并由真实 worker 的逐字节等价测试锁定。任一 runtime 身份校验失败都按 service protocol failure 处理，不能尝试修补 payload。

## 7. 镜像世界与原子快照

### 7.1 worker 世界

worker ready 前以 `LidarWorkerWorldSpec` 和现有 `build_world_from_scene_document()` 创建独立 DIRECT 世界，包括相同地形、机器人模型和障碍物构造。child 自己固定调用 `p.connect(p.DIRECT)`，不读取 `ExperimentConfig.mode` 来选择连接方式。镜像机器人只用于复用已验证的语义 link 和 backend 构造；扫描始终使用父进程冻结的世界安装位姿，且机器人不在 `LIDAR_VISIBLE_GROUP`，所以镜像机器人姿态和 dynamics 不会进入点云。

ready 不是“进程已启动”的信号。child 必须先完成完整场景构建、全部障碍物逻辑 ID 到自身 body 的映射、碰撞分类 bind、前后 scanner 与 codec 构造，并对前后各执行一次生产 `2880` 射线、点构造和 deterministic encode 全链预热；它校验两份 frame/payload 后才发送 `LidarWorkerReady`。预热帧不返回父进程、不占 job id、不进入 tracker/logger/transport。任一步失败都以 `worker_preflight_failed` 使启动原子失败。

worker 不调用 `stepSimulation()`。地形在同一 world generation 内保持静态；障碍物按每个请求的完整逻辑快照对齐。

### 7.2 每帧捕获

`SimulationCoordinator.step()` 已在物理步前推进移动障碍物，并通过 runtime 的 `SceneDocument` 保存完整无 body-id 逻辑状态。物理步完成后，runtime 在同一主线程捕获：

- 当前 lifecycle generation 和 pause epoch。
- scheduler 占用的原 LiDAR deadline。
- 当前 LiDAR mount 世界位姿。
- Dashboard 模式下当前 `base_link` 世界位姿。
- 当前 `SceneDocument.obstacles` 的完整不可变副本。

主线程在捕获期间不会再次推进物理世界，因此 mount、base 和障碍物属于同一物理状态。worker 只按 `logical_id`、mode、geometry、position 和 orientation 建立自己的 body 映射；父进程 body id 一律为 `None`，不能跨进程泄漏。

每个 job 开始扫描前，worker 原子完成障碍物集合 reconcile：新增缺失逻辑体、删除多余逻辑体、更新全部姿态并重建自己的命中分类映射。任一步失败则整帧失败，不执行或返回部分 raycast。

普通单帧 reconcile 失败只有在 child 能证明候选创建失败且原 body 集、姿态和分类完整回滚时，才返回可恢复的 `scene_reconcile_failed`。删除、绑定或回滚后无法证明镜像状态时必须返回 `scene_state_unknown` 并永久 fault service；不得继续接受下一帧。

### 7.3 P0 启动顺序与 20 障碍预热

正式 P0 不再先创建空场景 worker 再添加 20 个障碍物。`scripts/ecal_simulation_runtime.py` 必须先用尚未绑定 interface runtime 的 bootstrap coordinator 完成 20 障碍事务，生成包含完整障碍物的 `SceneDocument`，再调用 `create_interface_session()`；正式 coordinator 只在 session/worker ready 成功后取得 runtime。这样首次 `LidarWorkerWorldSpec`、world digest、碰撞分类和双雷达全链预热天然覆盖正式 20 障碍，不需要在 240 Hz 运行中增加一个阻塞式 scene-sync fence。

`ready_file` 只能在上述完整 worker ready 成功且正式 coordinator 已绑定后写出。接线测试必须锁定“20 障碍事务完成 -> worker full-batch ready -> ready_file”顺序。GUI/eCAL 运行期新增或删除障碍物仍由每帧完整 snapshot 的原子 reconcile 覆盖；不得让 `refresh_scene_bindings()` 同步等待 worker 预热而重新制造 wheel heartbeat 长帧。

## 8. 正常数据流

```text
main process                                      lidar worker

before_physics_step()
coordinator.step()
publish due WheelState
capture mount/base/obstacles
submit request without waiting  ----------------> reconcile shadow world
publish due RTK/IMU                               rayTestBatch
return to next physics frame                      build LidarPointCloud
                                                  encode protobuf once
poll completed response          <---------------- prepared frame
validate generation/epoch/order
publish same bytes to logger + eCAL lane
update latest message/top view
```

父进程在每个物理帧开始和结束各做一次 `poll(0)`；只有 endpoint 已就绪才 `recv()`。`poll(0)` 只保证不会等待 raycast 完成，pipe 传输、完整 `recv()`、校验和反序列化仍计入主循环本地预算。完成的 LiDAR 可以晚于同时间戳 RTK/IMU 发布，也可以晚于更高仿真时间戳的 wheel-state 发布；每个 LiDAR 话题内部仍按 job/timestamp 严格有序。

`next_physics_step_publish_topics()` 在 process 模式下不把“即将捕获但尚无 payload”的 LiDAR deadline 当作本帧同步发布。prepared result 进入既有非阻塞 eCAL publisher lane；lane 覆盖或 drop 继续由正式 oracle 判失败，不能为了 LiDAR lane 反向阻塞 wheel 物理步。

正式测量开始、正式测量结束、最终协议 fence 和正常 session close 都必须先调用有界 sensor fence：停止提交新 job，等待已捕获的 in-flight/pending job 完成，持续收取并发布合法结果，然后依次执行 logger idle、transport idle、快照和现有 marker ACK。measurement-start 与 measurement-end 都是可恢复 fence，ACK 后恢复进入前为 ready 的 service，以继续执行后测断线、恢复和安全协议；final/close 才保持停止。sensor fence 上限为 `250 ms`；超时使门禁失败，不能丢弃边界帧后继续写成功 ACK。这样 warmup frame 不会在 start snapshot 后插入日志 sequence，测量 frame 也不会在 end snapshot 后迟到。

## 9. 回压、延迟与顺序

### 9.1 容量

service 固定持有：

- 一个已发送给 worker 的 in-flight job。
- 一个尚未写入 pipe 的 parent-side pending job。

新 job 到达时：

1. 没有 in-flight：立即发送并占用 in-flight。
2. 有 in-flight 但没有 pending：保存为 pending。
3. 两者都存在：拒绝新 job，生成对应话题一个 `capture_rejected` event，不覆盖任何旧帧；runtime 消费后精确记录一次 `sensor_overrun`、一次 error 和一次 drop。

worker 返回当前结果后，父进程先消费结果，再把 pending 提升为新的 in-flight。全局 `job_id` 必须逐一递增；worker response 必须精确匹配当前 in-flight，不能跳号、重复或乱序。

pending 只保存冻结 capture，不提前占用 `job_id`。只有 capture 真正写入 request pipe 时才分配下一个连续 ID，并同时保留最初的 `captured_monotonic_ns`；因此 pause 撤销 pending 或第三帧被拒绝不会制造合法 ID 缺口。capture-to-response 延迟从最初捕获时刻计算，包含 parent-side pending 等待，不只计算 child raycast。

### 9.2 时间预算

- worker startup ready：最多 `5 s`。
- 单 job capture 到合法 response：最多 `100 ms`。
- measurement/final sensor flush：最多 `250 ms`。
- 正常 close join：最多 `2 s`。
- 本地正式预检 wheel heartbeat 最大间隙：不超过 `20 ms`，为真实 `25 ms` oracle 留出余量。

capture-to-response 严格大于 `100 ms` 的 job 只在首次越界时生成对应话题一个 `job_overrun` event；runtime 消费后精确记录一次 `sensor_overrun`、一次 error 和一次 drop。即使随后返回，也必须丢弃整个 prepared frame，再处理 pending。禁止重复计错或发布已超过合同的迟到帧。运行时不自动重启 worker，不用重启掩盖延迟证据。service 构造必须注入 `monotonic_ns`，生产默认 `time.monotonic_ns`，测试使用可控时钟而不真实等待 100 ms。

## 10. 生命周期

### 10.1 状态

`LidarScanService` 使用窄状态集：

```text
starting -> ready <-> suspended
             |          |
             +----------+-> draining -> closed
             |          |       |
             +----------+-----> failed -> closed
starting ---------------------> failed
```

- `starting`：进程存在但 ready/world digest 尚未通过。
- `ready`：可接收新 job。
- `suspended`：pause 或 rebuild prepare 已停止新 job；允许 resume/abort 回到 ready，或 commit/close 进入 draining。
- `draining`：拒绝新 job，只完成或终结已有 job。
- `failed`：拒绝全部新 job，保留首个基础设施错误供状态和 close 报告。
- `closed`：资源已终结，close 幂等。

measurement-start 和 measurement-end sensor fence 都是可恢复 barrier，不进入 terminal `draining`。它记录进入前的 `ready/suspended`，只在 ACK 已写且进入前为 `ready` 时恢复；pause/rebuild 已处于 suspended 时不得被 measurement fence 意外唤醒。只有 final protocol fence 或 close 才转入 terminal draining。

### 10.2 pause/resume

- `pause()` 先递增 pause epoch，再把 ready service 置为 suspended 并停止提交新 job。
- 已发送的 native raycast不可取消；结果返回后因 epoch 不匹配而丢弃，不进入 tracker/drop 计数，不在 resume 后发布。
- parent-side pending 在 pause 时撤销。
- `resume()` 把健康的 suspended service 恢复为 ready，不复用旧 job，从恢复后的新 scheduler deadline 重新捕获。

### 10.3 generation 失效

- world rebuild 的唯一 generation 推进点保持为现有 `prepare_world_rebuild()`；commit 不再次推进。
- eCAL disconnect 保持现有 generation 推进语义。每次推进都调用 service 的纯父进程 `invalidate_generation(new_generation)`：撤销旧 pending，保留不可取消的旧 in-flight 供随后按 stale 丢弃，并允许新 generation capture 成为 pending。
- stale result 只增加 service `stale_count`，不生成 topic error/drop event。
- rebuild prepare 在推进 generation 后同时 suspended；abort 在旧 in-flight 收敛后把同一旧 service retag 到已推进的新 generation 再恢复。候选 service 从创建开始绑定该新 generation。
- pause epoch 与 lifecycle generation 独立；pause/resume 不推进 generation，disconnect 不推进 pause epoch。

### 10.4 world rebuild

- prepare 递增 pause epoch，把旧 service 置为 suspended，使其停止接受新 job，撤销 pending，并用 `250 ms` sensor fence 等待当前 in-flight 返回后按旧 epoch 丢弃；随后才按现有 world-operation barrier 等待主世界读操作收敛。旧 service 为空闲后才允许启动候选，避免两个 worker 的 native 扫描重叠。
- prepare 同时保存 suspended old service 的 canonical world digest。commit 若收到不同 digest，才在发布新 runtime world 前启动候选 worker，并在锁外等待最多 5 s ready 和 world digest；若 coordinator 在 target 失败后用同一个 `commit_world_rebuild()` 传回与 old digest 相同的 previous document，则明确走 rollback-reuse 分支，不启动第二个 candidate。
- 候选成功后，runtime 在同一 commit 临界区安装新 robot/backend/sensors/service；沿用 prepare 已推进的 generation，不再次推进。
- commit 发布后才把旧 service 从 suspended 转为 draining 并 close；旧结果因 generation 不匹配不得进入新 tracker。
- 候选启动失败时关闭候选，保留旧 service，由现有 coordinator transaction 决定回滚；不得先销毁旧 service 再尝试候选。
- abort 或上述 digest 相等的 coordinator rollback 关闭尚未提交的候选，确认旧 in-flight 已清空，把健康旧 service retag 到 prepare 后的 generation，再恢复为 ready。digest 比较只用于识别 prepare 保存的 exact previous document，不能把任意碰撞或未经父进程复算的值当 rollback 标志。
- 新 runtime/service 已原子发布后，retired service 的 close/terminate 错误只能生成 `retired_cleanup_failed` lifecycle 诊断并进入 runtime 的待重试清理集合；`commit_world_rebuild()` 不得把该错误抛给 coordinator，不得反转新世界或 fault 新 service。runtime 最终 close 必须再次尽力清理并保留首错。

### 10.5 close

- 正常 close 使用 drain 路径：先停止新 job，保留并完成已捕获的 in-flight/pending，发布所有仍合法的 prepared frame，再发送 stop。
- service 已 failed、pipe 已坏或正常 drain 超时才进入 force 路径：撤销 pending、丢弃不可发布结果，并只 terminate 项目自己创建且仍存活的 child；该路径必须记录 `worker_shutdown_failed`，不能生成成功 fence/ACK。
- child 收到 stop、请求 pipe EOF 或父进程正常关闭时都必须断开自己的 PyBullet client 并回 ACK。
- 正常 join 最多 2 s；只有项目自己创建且仍存活的子进程可以被 `terminate()`，随后必须再次 join 并记录 cleanup error。
- transport 和 logger 只在 worker 关闭或明确失败记录形成后继续终结，确保没有后台发布访问已关闭资源。

## 11. 故障隔离与可观测性

### 11.1 单帧故障

raycast、镜像 reconcile、点构造或 codec 失败返回 `LidarScanFailure`：

- 精确污染请求对应的一个 LiDAR topic。
- 记录 `sensor_failed`、error/drop 和稳定错误码。
- worker 仍可接收下一 job。
- 不返回或发布部分点云。
- 只生成一个 `scope=topic` 的 `frame_failed` event；runtime 消费后精确更新请求话题一次 error/drop。

### 11.2 service 故障

以下情况把 service 置为 `failed`，前后 LiDAR 都停止接收新 job：

- 子进程提前退出、pipe EOF 或 ready 超时。
- IPC exact type、protocol version、world digest 或身份字段错误。
- job id 重复、跳号、错序或 response 不匹配当前 in-flight。
- 无法证明由项目拥有的进程已经终结。
- reconcile 删除、重新绑定或回滚后无法证明镜像状态。

wheel、RTK、IMU、命令 mailbox、watchdog 和安全停车继续运行。禁止因 LiDAR 基础设施失败关闭轮控或阻塞物理主循环。

service 只生成一个 `scope=service` 的 `service_failed` event；runtime 消费后对前后 LiDAR 各精确更新一次 error/drop，并记录同一个稳定基础设施错误。后续 poll/snapshot 不得重复污染 tracker。

### 11.3 状态字段

只读 service snapshot 字段固定为：`state`、`child_pid`、`lifecycle_generation`、`pause_epoch`、`next_job_id`、`in_flight_identity`、`pending_capture_identity`、`completed_count`、`failed_count`、`overrun_count`、`stale_count`、`max_capture_to_response_ns`、`last_error_code` 和 `last_error_detail`。计数属于当前 service 实例的完整生命周期；disconnect retag 或 old-digest rollback 复用时不清零，rebuild 安装新 service 后才从零开始。旧 service 的终态只进入 lifecycle event。P0 runtime result 增加当前 service 诊断字段，但既有 pass/fail oracle 不删除、不放宽。

## 12. TDD 顺序

严格遵守“没有预期失败的测试，就不写生产代码”。每个周期必须保存实际 RED 命令、断言失败摘要、GREEN 命令与结果；RED 必须在测试函数内失败，不能是 collection、ImportError、spawn bootstrap 或环境错误。

### Cycle 1：冻结位姿扫描

Test：`tests/test_lidar_pointcloud.py`

- RED `test_frozen_lidar_scan_matches_live_scan_without_pose_lookup`
- RED `test_frozen_dashboard_scan_keeps_message_and_top_view_atomic`
- GREEN 只提取现有扫描内部路径；原 `scan()` 行为不变。

### Cycle 2：模块与真实 spawn happy path

Test：`tests/test_lidar_worker.py`

- RED 在测试函数内用动态 import 查找 production module/entrypoint，并明确断言其存在且可调用；缺实现时必须是该断言 `FAILED`，不能是 collection/ImportError。
- RED `test_spawned_worker_returns_preencoded_atomic_frame`
- RED `test_spawned_worker_reconciles_complete_obstacle_snapshot_by_logical_id`
- RED `test_worker_ready_follows_full_front_rear_preflight`
- RED `test_worker_preflight_failure_returns_exact_startup_failure_without_ready`
- RED `test_spawned_worker_closes_direct_client_and_process_cleanly`
- GREEN 使用真实 `spawn`、真实 PyBullet DIRECT、真实 scene builder 和真实 codec，不 mock PyBullet。

### Cycle 3：有界 service

Test：`tests/test_lidar_worker.py`

- RED `test_service_keeps_one_pending_without_writing_it_to_pipe`
- RED `test_service_rejects_third_job_without_overwriting_older_jobs`
- RED `test_service_rejects_mismatched_or_out_of_order_response`
- RED `test_service_marks_job_over_hundred_milliseconds_as_overrun_once`
- RED `test_service_events_are_typed_ordered_and_consumed_once`

需要确定性阻塞时使用完整 IPC channel test double，只断言 service 的公开状态和返回值，不断言 mock 调用次数；不得为测试向 production class 增加 release、delay 或 destroy 方法。

### Cycle 4：runtime 非阻塞和预编码发布

Test：`tests/test_interface_runtime.py`、`tests/test_interface_runtime_integration.py`

- RED `test_async_lidar_capture_does_not_wait_before_next_wheel_deadline`
- RED `test_async_lidar_result_uses_worker_payload_without_parent_reencode`
- RED `test_async_lidar_allows_rtk_and_imu_at_same_timestamp_to_publish_immediately`
- RED `test_process_mode_preview_excludes_unprepared_lidar_topic`
- RED `test_measurement_fence_drains_captured_lidar_before_transport_snapshot`
- RED `test_measurement_start_fence_prevents_warmup_lidar_from_crossing_snapshot`
- RED `test_measurement_end_fence_resumes_post_window_protocol_after_snapshot`

测试通过 Event 证明物理调用在未释放扫描结果时已经返回，不使用脆弱的短时间 sleep 作为唯一 oracle。

### Cycle 5：pause、rebuild、failure 和 close

Test：`tests/test_interface_pause_rebuild.py`、`tests/test_lidar_worker.py`

- RED `test_pause_discards_old_epoch_result_before_resume`
- RED `test_disconnect_invalidates_old_generation_without_faulting_service`
- RED `test_rebuild_discards_old_generation_result`
- RED `test_rebuild_candidate_failure_preserves_active_service`
- RED `test_rebuild_rollback_digest_reuses_suspended_old_service`
- RED `test_rebuild_commit_ignores_retired_service_cleanup_failure`
- RED `test_single_scan_failure_degrades_only_requested_topic`
- RED `test_unknown_scene_state_faults_service_instead_of_continuing`
- RED `test_worker_protocol_failure_faults_both_lidar_topics_only`
- RED `test_normal_close_drains_pending_but_force_close_cancels_it`
- RED `test_close_terminates_only_owned_child_after_join_timeout`

### Cycle 6：正式入口接线

Test：`tests/test_interface_runtime_integration.py`、`tests/test_ecal_process_roundtrip.py`

- RED actual eCAL mode 创建 process service，actual local fallback 不创建。
- RED 正式 runtime 在 measurement start/end/final 的 transport/logger fence 前先完成 sensor flush。
- RED P0 在创建 session/worker 前完成 20 障碍 bootstrap，full-batch ready 后才能写 runtime ready_file。
- RED worker 启动失败使 strict eCAL session 原子失败并清理既有资源。
- RED worker ready 后 runtime 构造或 relay attach 失败仍精确关闭 child，不泄漏进程。

### Cycle 7：真实 DIRECT 等价与性能

Test：`tests/test_lidar_pointcloud_direct.py`

- 同一冻结平面、斜面、高尔夫场地和障碍物快照下，worker 与同步基线逐字节比较 wire payload。
- 覆盖 terrain、static obstacle、moving obstacle、unknown/miss、前后 mount、Dashboard top view 和零命中。
- 使用生产 `PyBulletSensorBackend.ray_test_indexed_hits()`，禁止 subclass fallback 冒充生产快路径。

Verifier：新增 `scripts/verify_lidar_worker_realtime.py`，不初始化 eCAL，但复用正式世界、20 个障碍物、240 Hz pacer、双 2880 射线和 production service。预检在静默宿主上连续执行 10 个 5 s 窗口，任何一轮失败即停止。

## 13. 验证门

### 13.1 聚焦 GREEN

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_lidar_pointcloud.py \
  tests/test_lidar_worker.py

conda run -n slope-sim python -m pytest -q \
  tests/test_interface_runtime.py \
  tests/test_interface_runtime_integration.py \
  tests/test_interface_pause_rebuild.py
```

### 13.2 扩大回归

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_sensor_backend.py \
  tests/test_lidar_pointcloud_direct.py \
  tests/test_ecal_transport.py \
  tests/test_ecal_process_roundtrip.py \
  -m "not ecal"

conda run -n slope-sim python scripts/verify_stage3_interfaces.py
conda run -n slope-sim python -m pytest -q -m "not ecal"
```

### 13.3 本地正式预检

连续 10 个窗口必须全部满足：

- wheel heartbeat 最大墙钟间隙 `<= 20 ms`。
- 每个 LiDAR capture-to-response `<= 100 ms`。
- 没有 queue overrun、service failure、protocol error、transport drop 或 logger drop。
- 每个窗口 topic 数量和 timestamp 序列符合既有 100/10 Hz scheduler。
- process、pipe 和 PyBullet DIRECT client 全部 clean shutdown，无残留子进程。
- 正式 `sim/wall` 下限仍使用现有 P0 oracle，预检不得自设更宽松门槛。

### 13.4 独立审查与真实 eCAL

本地 GREEN 和预检通过后，启动独立只读审查线程，从需求完整性、逻辑正确性、边界情况、代码质量、测试覆盖和实际运行结果六方面审查。Critical 和 Important 必须为 0。

随后执行顺序固定：

1. 向用户重新说明并取得仅覆盖下一条 invocation 的 `active_steering_4wd 4+2` 授权。
2. 重新扫描宿主静默状态；存在竞争负载时不消费授权。
3. 严格执行一次既有 P0 命令；FAIL 时保留证据并停止，不自动重跑。
4. 只有 `4+2` PASS 后，才另行取得 `df_back 2+0` 单条授权。
5. 两条均 PASS 后，P0 才解除对阶段四 Task 2 的阻断。

本文不构成任何新的真实 eCAL invocation 授权。

## 14. 参考与依赖影响

本设计复用现有 pinned reading references：

- `bulletphysics/bullet3`：`rayTestBatch`、collision group/mask、DIRECT client 和官方 PyBullet 行为。
- `padawanabhi/pybullet_sim`：同步传感器和仿真结构，用于确认现有成熟参考没有可直接照搬的同-client 并发模式。
- `eclipse-ecal/ecal`：父进程发布 lane、callback 和 clean shutdown；worker 不链接或初始化 eCAL。
- `protocolbuffers/protobuf`：worker 与父进程共享 deterministic wire 合同。

不新增 reference manifest 条目，不修改 Task 2 已冻结的 eCAL/Protobuf/MCAP/Zstd/Livox/PCL admission。实现只使用 Python 标准库 multiprocessing 和现有 PyBullet/Protobuf 依赖，因此没有新的 CMake、C++ ABI、Conda 包或发行许可证输入。

## 15. 完成定义

本设计对应的 P0 修复只有在以下条件全部成立时才完成：

- 全部 TDD cycle 均有真实 RED 和新鲜 GREEN 证据。
- local 与无节拍 DIRECT 保持同步行为，eCAL production 使用单 worker。
- 本地 10 窗口预检全部通过，独立六维审查 Critical/Important 为 0。
- 获授权的真实 `4+2` 与随后独立授权的 `2+0` 均通过未修改 oracle。
- master plan、阶段四交付报告和 README 只引用实际形成的结果，不提前写 PASS。

在此之前，阶段四状态仍为：Task 1 完成，P0 FAIL，Task 2 未开始且继续阻断。
