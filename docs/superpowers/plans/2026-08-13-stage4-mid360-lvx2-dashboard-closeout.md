# 阶段四 MID-360、LVX2、Livox Viewer 2 与 Dashboard 收尾计划

> 状态：执行中；Task 7 真 Viewer 空白已定位，Task 3/4 聚焦重开
> 日期：2026-08-13
> 目标工作树：当前未提交的阶段四工作树，不重开、不回退、不覆盖既有成果
> 最终裁决：PyBullet 中心 LiDAR 的正式 v2 数据经 C++ Recorder 写入完整 MCAP，
> C++ Export 生成符合 Livox LVX2 v1.0 结构的会话级文件，Livox Viewer 2
> 实际加载并显示该模拟点云；Dashboard 同时准确展示 live 状态和离线证据。

## Scope

- In：标准 LVX2 oracle、会话级 writer、Export 边界、真实 PyBullet/Recorder/MCAP
  输入、Livox Viewer 2 文件打开与显示证据、Stage 4 v2 Dashboard 改进、相关文档和
  最终阶段四验证。
- Out：真实 MID-360、真实硬件发现、SDK2/UDP 虚拟设备、修改 v2 wire schema、修改
  PyBullet 扫描算法、修改阶段三 15 页 Dashboard 合同、重新构建发行包、重复 ROS、
  安装器或大型依赖门禁。
- 权威数据：完整 MCAP 始终是唯一无损原始记录；PCD、PLY、LVX2、Dashboard 和
  Viewer 截图都是派生结果或证据。
- Git：未经用户明确授权不 commit、不 push；用任务级文件边界和 `git diff` 保护当前
  脏工作树，不把提交基线作为执行前置条件。

## 已核对的 checkpoint

1. 阶段四中心 LiDAR 已固定在 `lidar_link`，realtime profile 为 5,760 条候选射线，
   v2 `/sim/lidar/points` 以 10 Hz 发布；点云只包含有效命中，不包含 miss 占位点。
2. v2、C++ SDK、Command、Subscriber、Recorder、Replay、ROS Bridge 和原 MCAP
   Reader/Writer 已存在于当前工作树及 CMake/CTest 图中，不重做这些能力。
3. `cpp/client/stage4_export.cpp` 当前 `.lvx2` 是私有
   `SLOPE-SIM-SYNTHETIC-LVX2` 布局，不是 Livox LVX2。
4. 旧 Viewer verifier 只证明 loopback-only 进程启动和
   `/Game/Maps/Viewer` map-load；它不证明 LVX2 导入、点云显示或硬件发现。
5. 本地官方规范和 222,540,611-byte 官方样例可只读复用；不得复制到仓库或重新下载。
6. 阶段四有独立 `V2DashboardWidget`。阶段三企业 Dashboard 的 15 页、顺序、布局和
   GUI 门禁保持不变。
7. 当前相对 HEAD 的大批删除、修改和 untracked 阶段四文件全部视为用户既有成果；
   agent 只能修改任务明确列出的文件。

## 已冻结的 LVX2 合同

### 文件与设备

- 每个完整 MCAP 会话导出一个 `lidar.lvx2`，不再给每个 10 Hz 扫描伪造一个独立
  LVX2 文件；PCD/PLY 仍按扫描 sequence 分文件。
- Public header：16-byte `livox_tech` signature、version `2.0.0.0`、little-endian
  magic `0xAC0EA767`。
- Private header：frame duration 固定 50 ms、device count 固定 1。
- Device info：synthetic SN 固定 `SLOPESIM00000001`，Hub SN 全零，LiDAR ID 取
  v2 cloud `lidar_id`，reserved LiDAR type 为 0，device type 为 Mid-360 `9`，
  extrinsic disabled，六个外参为 0。所有 synthetic 元数据必须写入 sidecar，不能
  暗示来自真实设备。

### 帧、包与时间

- 一个 10 Hz v2 scan 按 `offset_time_ns` 分成 `[0, 50 ms)` 和
  `[50 ms, 100 ms)` 两个 LVX2 frame；输入必须满足 `offset_time_ns < 100 ms`。
- MCAP 中 LiDAR `timebase_ns` 必须严格递增，sequence 必须连续，session、descriptor、
  world、`frame_id=lidar_link` 和 `lidar_id` 必须一致；不一致时 Export fail-closed。
- 每个 frame header 固定 24 bytes；`current_offset` 等于自身绝对位置，
  `next_offset` 指向下一 frame header，最后一个 frame 的 `next_offset` 等于 EOF，
  frame index 从 0 连续递增。
- 点按原始扫描顺序和绝对时间保持稳定顺序。每包最多 96 点；package timestamp 取
  该包第一点的 `timebase_ns + offset_time_ns`，UDP counter 从 0 连续递增，
  frame counter 为 frame index 的低 8 bit。package `LiDAR_Type` 固定为 Livox Viewer
  2.6.0 实际接收的 Mid-360 profile 值 `8`，其余 reserved bytes 为 0；该字段与
  device-info 中按规范保留为 `0` 的 LiDAR type 不得混淆。
- data type 固定 `0x01`，每点固定 14 bytes：int32 millimeter XYZ、uint8
  reflectivity、uint8 tag。坐标采用 `round(m * 1000)`；非有限值或 int32 溢出整次
  Export 失败。
- 不丢点、不复制点、不补零或补假点。尾部不足 96 点时先按规范 `length / 14`
  表达短尾包，并由官方 Viewer 实测裁决兼容性。若 Viewer 拒绝短尾包，则记录为
  “稀疏命中点合同无法在不造点的前提下满足 Mid-360 固定 96 点包”，停止 LVX2
  路径并提交独立替代设计；不得擅自转向 SDK2/UDP 虚拟硬件。
- 每个 v2 scan 即使没有命中也保留对应的两个空 50 ms frame header；空 frame 不写
  package。该行为必须由 oracle 和 Viewer 实测共同验收，不能仅凭 writer 自测判定。

### 损失声明与原子性

- `lidar.lvx2.json` 至少记录：`synthetic=true`、format/version、source MCAP
  SHA-256、simulation session、descriptor SHA-256、world generation、scene、frame id、
  device metadata、1 mm 量化、50 ms 分帧、package timestamp 规则、原 scan/frame/
  point/package 数、短尾包数、空 frame 数、`padding_points=0`、`dropped_points=0`。
- sidecar 明确 `line` 和逐点精确 offset 在 LVX2 type-1 中不可保存；包 timestamp
  只能保留每包第一点时间。PCD/PLY 和权威 MCAP 继续保留原字段。
- Export 继续使用新 staging directory + rename；任一格式失败时不得发布部分输出，
  不得修改输入 MCAP，重复输出路径继续拒绝。

## Dashboard 设计（2026-08-13 用户已确认）

采用“扩展独立 Stage 4 v2 Dashboard”，不新增 Web app，不修改阶段三 15 页面板。
Dashboard 是运行状态与验收证据控制面，不承担完整点云渲染：实时完整三维点云在独立
ROS Bridge + RViz2 窗口显示，离线标准 LVX2 在独立 Livox Viewer 2 窗口显示。

### 显示与启动边界

- Dashboard 提供实时 3D Viewer 启动入口和连接状态，但只有受监管 launcher 明确配置、
  子进程和目标 Viewer 身份均可验证时才允许启用。当前仅有 topic peer 不能证明
  ROS Bridge/RViz2 身份；未配置可靠 launcher 时按钮必须禁用并显示“未配置/连接未验证”。
- Dashboard 不嵌入完整 3D 点云。现有最多 512 点 top-view 改为默认折叠的“采样预览，
  非验收证据”，不得用于证明点数完整、空间完整、Viewer 已连接或阶段验收通过。
- 离线 LVX2 只在 Livox Viewer 2 独立窗口显示。现有启动/map-load smoke 没有证明指定
  LVX2 已打开，因此在真实路径打开合同完成前，离线 Viewer 动作同样 fail-closed。
- evidence JSON 只描述已完成事实，绝不携带或执行命令；任何启动动作只能来自受信任的
  launcher 配置和显式用户操作。

### Live 运行状态区

- 显示当前 simulation session、descriptor 短 hash、world generation、车型和
  `lidar_link`/MID-360 simulation model 标识。
- 固定五话题表以 `slope_sim/interfaces/v2/topics.py` 为唯一名称和目标频率来源，字段至少
  包含 topic、目标/实际 Hz、protocol state、peer count、最后 sequence、点数、数据
  `age`、transport drop 和 sequence gap。LiDAR 显示 `point_num`，其他话题点数显示 `--`。
- actual Hz 使用有界 monotonic 事件窗口；少于两个事件显示 `--`，不得以单次或两帧
  差值伪造稳定频率。
- telemetry observation time 和 transport observation time 独立记录。transport refresh
  不得刷新 telemetry 时刻；`age` 只表示 monotonic 当前时间减 telemetry 观察时间，不能
  标成网络 latency，也不能用墙钟减仿真 timestamp。
- transport dropped count 与 sequence gap 分开显示、分开累计，不得相互推断。
- `/sim/wheel/command` 只在 authority 接受成功后记录；sequence 取 accepted authority，
  timestamp 取 mailbox，接收时刻取 callback `received_at`，拒绝命令不得刷新 live 状态。

### Offline Evidence 区

- 标题固定说明“离线已验证证据，非实时状态”。未提供显式 evidence 时全部显示
  “未提供 verifier evidence”，不扫描 `results/`，不读取 README 的 PASS 文本。
- 只接受显式 `--evidence-json ABS_PATH`，并在 GUI/runtime 启动前完成读取和验证。顶层
  固定 `schema_version=1`、`kind`、offline identity 以及 `recorder`、`replay`、`export`、
  `viewer_startup`、`viewer_display` 五个独立 section。
- artifact 路径必须为绝对、规范化、存在的普通文件；Dashboard 现场复核 SHA-256。
  Recorder/Replay/Export/Viewer 必须绑定同一 offline session、descriptor、world 和 scene
  身份链，但该 offline identity 不要求等于当前 live session。
- Recorder 显示 MCAP hash、五 topic count 和 clean shutdown；Replay 显示隔离的四个
  output topic count；Export 显示 LVX2/PCD/PLY、源 MCAP hash、synthetic/lossiness；
  Viewer 分开显示启动 smoke 与“本轮 LVX2 已加载且非空点云可见”证据。
- `viewer_display` 必须绑定同一 LVX2 绝对路径/hash，并包含 playback progress、非空点云
  可见和截图引用；hash 篡改、链内跨 session 或缺失附件一律 fail-closed。
- 任何离线证据都不得转写为 live peer、真实 MID-360、实时订阅、当前进程健康或 Viewer
  当前连接状态。

## Action items

### Task 1/8：冻结 checkpoint 与纠正证据语义

- [x] 只读核对中心 LiDAR、v2、Recorder、Replay、Export、Viewer 和 Dashboard 调用链。
- [x] 记录当前工作树规模和文件所有权风险，不 reset、stash、checkout 或恢复删除项。
- [x] 将需求、架构、设计和阶段四报告中的 Viewer 旧结论降级为启动/map-load smoke。
- [ ] 最终文档收口时补 README 的 Stage 4 v2 正式入口，并清楚标注 v1 为历史入口。

验收：四份已改文档执行 `git diff --check`；不为纯文档变化运行 pytest。

### Task 2/8：建立独立 LVX2 oracle

- [x] 从官方 PDF 和官方样例冻结字段、offset、字节序及首两帧事实。
- [x] 新增独立只读 parser/verifier，禁止调用待测 C++ writer 的内部实现。
- [x] 用手写 golden bytes 覆盖 header、device、frame、package、point、截断和损坏输入。
- [x] 通过显式环境变量对本机官方样例做 seek-based header/首帧检查，不把 222 MB
  样例复制进测试 fixture，也不整文件读入内存。

文件边界：`scripts/verify_lvx2.py`、`tests/stage4/test_lvx2_oracle.py`。
RED/GREEN：

```bash
conda run -n slope-sim python -m pytest -q tests/stage4/test_lvx2_oracle.py
LIVOX_LVX2_REFERENCE=/home/cancade/Downloads/Livox-MID360-reference/Indoor_sampledata.lvx2 \
  conda run -n slope-sim python -m pytest -q tests/stage4/test_lvx2_oracle.py -m stage4_artifact
```

### Task 3/8：执行会话级 LVX2 Writer TDD

- [x] 先把 C++ Export 测试改为要求官方 signature/version/magic/device/frame/package；
  运行现有 writer 得到与私有 magic 一致的 RED。
- [x] 最小实现会话级 `lidar.lvx2`、sidecar 和扩展后的 export result JSON。
- [x] 用多 scan fixture 覆盖两个 50 ms frame、跨 scan frame、确定性输出、短尾包、
  空 frame、最后 `next_offset=EOF`、毫米量化及 lossiness 声明。
- [x] 保持 PCD/PLY 文件命名、MCAP reader 和 staging/rename 边界不变。

Checkpoint：fresh Export 位于 `/tmp/stage4-lvx2-red.y5tk7R/slope_sim_stage4_export`；
Writer 最终复审为 `Critical=0, Important=0`。canonical CMake cache 的依赖根已删除，
后续不得用它重新 configure/build，也不得用旧 canonical Export 替代 fresh 产物。

2026-08-13 聚焦重开：Task 7 真 Viewer 复验发现上述 writer 把所有 package
`LiDAR_Type` 写成 `0`。Viewer 2.6.0 的
`LvxFileParseHandler::UpdatePacketList(int)` 在 package `+5` 处读取该字节并只在值为
`8` 时入队；项目 1,350 个包全部被跳过。此前结构 GREEN 仍保留，但不再代表 Viewer
兼容 GREEN。

- [x] 先把 C++ writer 测试的 package `LiDAR_Type` 期望从 `0` 改为 `8`，对未改 writer
  运行得到与实际缺陷一致的 RED。
- [x] 最小实现只改 package profile 字节；不得改 device-info 保留字段、补点、padding、
  短尾、空 frame、时间或坐标合同。
- [x] GREEN 后只重跑 Export 聚焦测试和受该字段影响的 cross-read/oracle 回归；不重跑
  Dashboard、完整 C++ 或默认仓库回归。

文件边界：`cpp/client/stage4_export.cpp`、`cpp/client/tests/stage4_export_test.cpp`；只有
确有必要时修改 `cpp/phase0/CMakeLists.txt`。
RED/GREEN：

```bash
STAGE4_BUILD=/home/cancade/pybullet_df_drive/build/stage4-phase0-ecal611-release-mapped-20260809T220738+0800
cmake --build "$STAGE4_BUILD" --target slope_sim_client_export_test
ctest --test-dir "$STAGE4_BUILD" --output-on-failure -R '^slope_sim_client_export$'
```

### Task 4/8：完成格式 GREEN 与 Export 边界回归

- [x] 用独立 oracle 回读 C++ 输出，逐点核对允许的 1 mm 量化误差和稳定顺序。
- [x] 覆盖非有限坐标、int32 溢出、offset 越界、时间/sequence 倒退、身份变化、
  空 MCAP/空点云、96 点边界、短尾、损坏/截断文件和重复输出路径。
- [x] 验证失败不发布 output/result、不修改原 MCAP；相同输入生成 byte-identical LVX2。
- [x] 运行 Export、MCAP Reader/Writer 直接相关 CTest，不运行 ROS 或完整仓库回归。

Checkpoint：C++ Export test 与 Python cross-read 通过，Reader/Writer CTest 为 `2/2`；
最终复审为 `Critical=0, Important=0`。本轮不重跑这些已通过的 oracle/C++ 门禁。

```bash
ctest --test-dir "$STAGE4_BUILD" --output-on-failure \
  -R '^(slope_sim_client_export|slope_sim_client_mcap_session_reader|slope_sim_client_mcap_session_writer)$'
```

### Task 5/8：改进 Stage 4 v2 Dashboard

- [x] 用户确认“独立窗口承担完整 3D 显示，Dashboard 以运行状态与证据为主，top-view
  仅为折叠采样预览”的修订设计；Sol ultra 已完成只读数据源和 launcher 诚信审查。
- [x] TDD 单元 A：先为 live observation 写 RED，再最小实现 identity、固定五话题指标、
  telemetry/transport 双观察时刻、有界 actual Hz、point count、age、transport drop 和
  sequence gap；只记录已接受的 command 元数据。
- [x] TDD 单元 B：先为 UI 写 RED，再实现固定五话题状态表、实时 Viewer 启动入口与
  fail-closed 连接状态，以及默认折叠并明确非证据的 512 点采样 top-view。
- [x] TDD 单元 C：先为 evidence 写 RED，再实现只接受显式绝对 `--evidence-json` 的
  schema/path/hash/offline identity 验证，以及 Recorder/Replay/Export/Viewer 分区。
- [x] 保留 capacity-1 latest snapshot、GUI 线程解码和 simulator/eCAL 子进程隔离；不得把
  artifact I/O 放进物理线程或 native callback，不得从 evidence 执行命令。
- [x] 用 offscreen 测试覆盖窄窗口文字、状态缺失、hash 篡改、链内跨 session evidence、
  MID-360 simulation 文案、live/offline 标签、Viewer 未配置禁用和采样预览默认折叠。

Checkpoint：正式 CLI 显式隔离 runtime；child 采用有界 cancel/terminate/kill 收尾；同步
LiDAR 连续帧会刷新 sequence、点数和采样预览；50 PCD + 50 PLY 使用短摘要和可滚动完整
path/hash 明细。针对性回归为 launcher `10 passed`、adapter `28 passed, 7 deselected`、
Store `2 passed, 8 deselected`、evidence `23 passed, 12 deselected`；最终质量复审为
`Critical=0, Important=0, Minor=2`。遗留 Minor 是大文件整文件 hash 峰值内存和受控 IPC
未拒绝同身份旧 sequence/timestamp，不阻塞本任务。

文件边界：`slope_sim/interfaces/v2/dashboard_snapshot.py`、
`slope_sim/interfaces/v2/simulation_runtime.py`、
`slope_sim/interfaces/v2/dashboard_adapter.py`、`scripts/stage4_v2_dashboard.py`、相应 v2
Dashboard 测试；不改 `slope_sim/interfaces/transport.py`、
`slope_sim/interfaces/v2/runtime_protocol.py`、`slope_sim/dashboard.py`、ROS Bridge、
Viewer verifier、C++ 工具和阶段三 GUI verifier。
线性 RED/GREEN 与直接回归：

```bash
conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_dashboard_snapshot.py -k live_observation
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest -q \
  tests/stage4/test_v2_dashboard_adapter.py -k 'live_status or viewer or sampled_preview'
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest -q \
  tests/stage4/test_v2_dashboard_adapter.py \
  tests/integration/test_v2_dashboard_launcher.py -k evidence
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest -q \
  tests/stage4/test_v2_dashboard_snapshot.py \
  tests/stage4/test_v2_dashboard_adapter.py \
  tests/integration/test_v2_dashboard_launcher.py
```

### Task 6/8：生成真实 PyBullet 到 LVX2 证据链

- [x] 使用固定车型、场地和空障碍物集合的短时会话，串行启动正式 Python v2 Simulator、
  C++ Command/Subscriber/Recorder；不得用手工 fixture 冒充最终输入。
- [x] 从 Recorder 的完整 MCAP 运行正式 Export，生成会话级 LVX2 和 sidecar。
- [x] 生成 canonical evidence index，记录命令、场景、session/world、MCAP/LVX2 SHA-256、
  topic/frame/point/package 数、工具 result JSON 和日志路径。
- [x] 用 oracle 验证本轮 LVX2；fresh Export `rc0` 间接证明正式 MCAP Reader 接受完成态
  MCAP、身份和 raw v2 payload，未另行声称独立 Reader revalidation。

Checkpoint：固定 `df_mid`/`flat`/`obstacles=[]`、5 秒、session
`00112233445566778899aabbccddeeff`、world `1`。Recorder 五话题为
`500/500/50/50/50`，LVX2 为 `100 frame / 1350 package / 126900 point`，50 PCD 和
50 PLY；MCAP SHA-256 为 `32912a53ac11ecafe2cc2f66f30db2b2a86b12f121ecac4d49655a9b276d9b3e`。
canonical v3 为 `results/stage4-lvx2-closeout/canonical-evidence-index-v3.json`，SHA-256
`c92102ba5b3502dd90acba75097167e4f9d7169268dda9eac84ba902dffb7f4b`；独立证据复审为
`Critical=0, Important=0, Minor=0`。首轮 `<stdin>` spawn 失败和第二轮 evidence assertion
错误均原样保留，未覆盖成功制品。

制品只写一个稳定、受控的 `results/stage4-lvx2-closeout/` 根；执行前先检查现有目录，
不得覆盖来源不明的证据。短时 MCAP/LVX2 预计远低于 1 GiB，不重建或复制依赖树。

### Task 7/8：执行 Livox Viewer 2 真显示验收并收口文档

当前 checkpoint：官方样例已真实打开并显示非空 Mid-360 点云，证明文件选择、播放与
观察方法可用；其旧 evidence package 因缺少持久化输入 hash、network namespace 和窗口
PID 绑定而不满足严格证据完整性，但不影响“Viewer 显示链可用”的结论。项目
`results/stage4-lvx2-closeout/export/lvx2/lidar.lvx2` 也已真实打开，识别为 100 帧、5 秒、
`SLOPESIM00000001`、`Mid-360 (9)`，并播放至 `00:00:05/00:00:05`，但主视图区全程空白；
95 次 `Lvx Get Frame` 全部为 `packet size: 0`。失败文件 SHA-256 为
`17d1ddb55751ab8447658df0fd812f3c9bef28c788db52dddad8bbd343781c2d`，失败现场保存在
`results/stage4-lvx2-closeout/viewer/project-lvx2-detached/`，不得覆盖。

Sol ultra 只读根因审查已证实：项目 writer 与旧 C++ 合同把 1,350/1,350 个 package
`LiDAR_Type` 写成 `0`，官方样例 162,293/162,293 个包为 `8`；Viewer 2.6.0 在入队前
硬筛 `LiDAR_Type == 8`，精确解释全部零包。短尾包仍是规范风险，但 Viewer 的当前入队
路径接受本项目 `588`-byte 短尾，因此不是本次全空白根因。

- [x] 先用官方 `Indoor_sampledata.lvx2` 验证文件选择、打开、播放和非空点云观察方式。
- [x] 打开并播放 Task 6 的原始 `lidar.lvx2`，保存完整零包失败证据并完成根因审查。
- [x] 完成 Task 3/4 聚焦 RED/GREEN 后，从同一 canonical MCAP 导出到全新目录；确认
  MCAP SHA-256 前后不变，生成 append-only v4 evidence，绝不覆盖失败 LVX2 或 v3 index。
- [x] 在新的独立 Viewer case 中打开修正版；确认 Viewer 展示的路径/hash 与 v4 evidence
  一致，并真实播放观察。
- [x] PASS 必须同时具备：模拟 Mid-360 设备条目、有效播放进度变化、主视图区非空点云、
  清晰截图、Viewer 日志、窗口/进程证据和输入文件 SHA-256。
- [ ] 启动/map-load smoke 与文件加载/点云显示分别记录。仅进程存活、
  `LoadMap(/Game/Maps/Viewer)` 或 loopback isolation 一律不能判显示 PASS。
- [x] 根据真实结果更新 README、需求、架构、设计和阶段四交付报告；记录准确 RED、
  GREEN、回归命令、证据路径和未运行门禁。

若 `LiDAR_Type=8` 后仍因短尾包或空 frame 导致 Viewer 拒绝，先保存完整失败证据并按
已冻结合同停止 LVX2 路径、提交独立替代设计；不得跳过 oracle、补假点、padding 或改用
真实硬件。

### Task 8/8：最终验证与一次六维只读审查

- [ ] 运行 LVX2 oracle/Export、Viewer verifier、Dashboard 和 Reader/Replay 的直接回归。
- [ ] 运行 `git diff --check` 和 README 定义的一次默认非 eCAL 回归；同一工作树快照不
  重复跑完整回归。
- [ ] 只在实际边界变化时运行真实 eCAL、ROS、GUI 或发行门；否则复用仍有效的阶段四
  历史证据并明确标注 historical/fresh。
- [ ] 启动一次独立只读六维审查，覆盖需求完整性、逻辑、边界、代码质量、测试覆盖和
  实际运行结果；不适用项写 `N/A`。
- [ ] 只有 `Critical=0`、`Important=0` 且 Viewer 真显示证据完整，才能宣布阶段四结束。

默认回归：

```bash
conda run -n slope-sim python -m pytest -q -m 'not ecal and not stage4_artifact'
git diff --check
```

## 并行执行波次

| 波次 | Agent/模型 | 独占文件边界 | 依赖 | 状态 |
|---|---|---|---|---|
| 1 | checkpoint，只读 Terra | 无写入 | 无 | 已完成 |
| 1 | LVX2 规范/样例，只读 Terra | 无写入 | 无 | 已完成 |
| 1 | Dashboard，只读 Terra | 无写入 | 无 | 已完成 |
| 1 | Viewer 文档纠偏 Terra | 四份指定文档 | checkpoint | 已完成 |
| 2 | LVX2 oracle TDD Terra | oracle script/test | 规范调查 | 已完成 |
| 2 | C++ LVX2 writer TDD Terra | Export source/test | 本计划合同 | 已完成 |
| 2 | Viewer 自动化调查 Terra | Viewer verifier/test，只读优先 | 旧 smoke | 已完成 |
| 3 | Dashboard 方案理解 Sol ultra | 无写入 | 用户确认设计 | 已完成 |
| 3 | Dashboard 线性 TDD Terra high | v2 Dashboard 与三个测试文件 | Sol 规格、用户确认 | 已完成 |
| 4 | 主 agent 集成与真实门禁 | 共享结果与文档 | 前述 GREEN | 原 LVX2 真显示失败，根因已证实 |
| 4A | Viewer 根因审查 Sol ultra | 无写入 | 原 LVX2 失败证据 | 已完成 |
| 4A | Writer 聚焦 TDD Terra high | Export source/test | Sol 根因、原 MCAP | 执行中 |
| 4B | 主 agent 新 Export 与独立 Viewer case | append-only evidence | 聚焦 GREEN | 待执行 |
| 5 | 独立审查 agent，只读 | 无写入 | 最终 fresh evidence | 待执行 |

同一时间不得让两个 agent 修改相同文件、相同 CMake build root 或同一功能链。子 agent
必须返回 RED/GREEN 命令和实际输出摘要；主 agent 审查 diff、处理冲突并运行集成回归。

## 最终成功标准

1. 证据能追溯点云来自 PyBullet 中心 `lidar_link`，没有使用真实 MID-360。
2. 正式 v2、C++ Recorder 和完整 MCAP 是 LVX2 的唯一输入链，MCAP SHA-256 前后一致。
3. 独立 oracle 证明导出符合 LVX2 v1.0 的结构、offset、字节序、帧、包和点布局。
4. sidecar 完整披露 synthetic 元数据、1 mm 量化、逐点时间/line 损失，且零补点、
   零丢点。
5. Livox Viewer 2 确实加载本轮 LVX2 并显示非空模拟点云；启动 smoke 不冒充显示。
6. Stage 4 v2 Dashboard 以 session、五话题、频率、点数、age、丢帧和
   Recorder/Export/Viewer 证据为主，能区分 live observation 与离线 evidence；完整
   实时 3D 和离线 LVX2 均在独立窗口显示，折叠 top-view 不作为完整性或验收证据。
7. 既有阶段四成果和未提交改动被保留；没有未经授权的 commit、push、发布或大下载。
8. 最终 fresh 聚焦验证、一次默认回归和一次六维只读审查满足阶段收口门槛。

## 2026-08-14 追加优先级：golf + obstacles + motion 目视验收

本节是 append-only 修订，优先级高于上面的 Task 7/8。阶段四最终收口、Task 8 完整回归和
最终六维审查全部暂停；先生成一份用户可在 Livox Viewer 2 中目视判断的代表性三维场景，
保持 Viewer 独立窗口打开，并等待用户决定后续扫描建模方向。不得为改善视觉效果修改
LiDAR 射线、扫描 profile、点数、padding 或既有 LVX2 合同。

### 已冻结基线与 Dashboard 边界

- Flat/Profile8 LVX2 固定为
  `results/stage4-lvx2-closeout/export/lvx2-v4-profile8/lidar.lvx2`，SHA-256 为
  `ab34f286a4faeae0c837b122237836c2a15591d9b6f6e039544cdbe28add3a27`；其
  `100 frame / 1350 package / 126900 point` 与 Viewer 非空显示只证明协议、Export、加载和
  渲染基线，不作为场景丰富度或最终视觉效果证据。
- Flat Viewer 复审结论冻结为 `DISPLAY PASS WITH SHUTDOWN CONCERN`：播放和非空点云成立，
  但 teardown 在 `RequestExit(143)` 后发生 `SIGSEGV(NULL)`、`wait_rc=139`，不得称为 clean
  shutdown。
- Dashboard 方案保持用户确认的修订版：完整实时三维点云在独立窗口显示，Dashboard 只
  提供启动入口和连接状态；离线 LVX2 由 Livox Viewer 2 独立显示；Dashboard 聚焦 session、
  固定五话题、频率、点数、延迟、丢帧和 Recorder/Export/Viewer 证据；小型 top-view 仅为
  默认折叠的采样辅助，不能作为点云完整性或验收证据。本追加任务不再修改 Dashboard。
- 旧 Task 6 的 `df_mid / flat / obstacles=[]` 会话只有 `10` 个 active command 和 `490` 个
  safe-stop command，不能证明持续车辆运动。旧 canonical 与 `/tmp` provenance 限制只作
  历史参考，不迁入新的 golf evidence 根。

### A1：冻结唯一 SceneDocument

场景必须通过现有 `SceneDocument`、`TerrainDocument`、`ObstacleSpec`、`ObstaclePath` 和
`dump_scene_atomic` 生成 YAML；禁止手写第二套 scene schema 或新增平行配置层。

固定逻辑参数：

```python
SceneDocument(
    schema_version=1,
    robot_model="df_mid",
    terrain=TerrainDocument("golf_heightfield", 0.0, 41, "medium"),
    obstacles=(
        ObstacleSpec(
            1, "static", ObstacleGeometry("box", (0.35, 0.35, 0.60)),
            (-0.8, 1.8, 0.60), (0.0, 0.0, 0.0, 1.0),
        ),
        ObstacleSpec(
            2, "static", ObstacleGeometry("cylinder", (0.32, 0.32, 0.70)),
            (0.7, -1.7, 0.70), (0.0, 0.0, 0.0, 1.0),
        ),
        ObstacleSpec(
            3, "moving", ObstacleGeometry("box", (0.35, 0.35, 0.55)),
            (-0.2, -0.4, 0.55), (0.0, 0.0, 0.0, 1.0),
            ObstaclePath((-0.2, -0.4), (-0.2, 0.8), 0.30, 0.0, 1),
        ),
    ),
    sensors=SensorDocument.default(),
)
```

`ObstacleManager.restore()` 仍负责按 seed 41 的实际 heightfield 重采样障碍物 Z 和贴地姿态；
文件内的 Z/四元数只是合法逻辑种子。三个障碍物必须在真实运行前用中心 `lidar_link` 扫描
证明有命中，不能只凭设计坐标宣称可见。点语义合同固定为 terrain（`tag=1`、
`reflectivity=100`）、static obstacle（`tag=2`、`reflectivity=160`）、moving obstacle
（`tag=3`、`reflectivity=200`）。

### A2：正式 runtime scene 入口与逐帧运动

在 `scripts/stage4_v2_simulation_runtime.py` 增加唯一 `scene: Path | None` / `--scene PATH`
入口：

- `robot_model`、`terrain_model` 的 API/CLI 缺省改为 `None`；无 scene 时才分别落到 `df_mid`
  和 `flat`，保持旧调用方行为。
- scene 与任何显式 robot/terrain selector 同时出现时 fail closed，即使值恰好一致。
- 通过 `ExperimentConfig.scene_in` 调用 `initial_scene_document(config)`，并在
  `p.connect` 前完成加载与校验；随后车型、terrain、slope、seed、relief、物理世界和结果
  元数据只能读取该 document 的权威值。
- 主循环顺序固定为 command decision、`robot.command_wheel_speeds(...)`、
  `obstacle_manager.update_moving(dt)`、`p.stepSimulation(...)`、
  `runtime.after_physics_step(...)`。10 Hz capture 在 parent 物理步后冻结中心 mount 与完整
  无 body-id obstacle snapshot；worker 每次扫描只 reconcile 该快照，不自行推进路径。

线性 TDD：

```bash
conda run -n slope-sim python -m pytest -q \
  tests/stage4/test_v2_simulation_runtime.py -k scene_cli
conda run -n slope-sim python -m pytest -q \
  tests/integration/test_v2_simulation_direct.py \
  -k 'scene_document_is_authoritative or moving_obstacle_advances'
```

每个测试必须先对未修改生产代码运行并得到与缺失行为一致的 RED，再做最小实现并得到 GREEN；
不得重跑已通过的 Oracle、完整 C++、Dashboard 或默认仓库回归。

### A3：唯一 C++ Command 的持续前进、转向和停车

现有 `slope_sim_stage4_command` 默认模式只在 0 ms `Renew` 一次，100 ms 后安全停车。新增
显式 schedule 模式，仍由同一个唯一 C++ Command participant 发布，复用现有
WheelCommand protobuf，不新增 wire 协议：

- `0..2000 ms`：左右驱动轮 `(4.0, 4.0) rad/s`，目标约为 `v=0.40 m/s, w=0`；
- `2000..4500 ms`：左右驱动轮 `(2.875, 4.125) rad/s`，目标约为
  `v=0.35 m/s, w=0.25 rad/s`；
- `4500..5000 ms`：显式零命令 `(0.0, 0.0)`。

Schedule 由直行模板 payload、转向 payload、`turn-at-ms=2000` 和 `stop-at-ms=4500` 显式
启用；每个 10 ms tick 都以当前段续租。全部 500 帧必须保持同一 source/session，匹配
本轮 runtime 的 `simulation_session_id`、`world_generation=1`、`command_generation=1`，
sequence 严格为 `0..499`。默认单 payload 模式及其 100 ms timeout 行为保持不变。

线性 TDD：

```bash
conda run -n slope-sim python -m pytest -q \
  tests/stage4/test_cpp_client_sdk.py \
  -k 'continuous_forward_turn_stop_schedule or command_lease'
conda run -n slope-sim python -m pytest -q \
  tests/stage4/test_c2_supervisor.py -k five_second_scene_motion_window
```

生产文件边界只允许 `scripts/stage4_v2_simulation_runtime.py` 和
`cpp/client/stage4_command.cpp`；测试边界为上述聚焦文件和确有必要的新
`tests/integration/test_v2_simulation_direct.py`。禁止修改 Dashboard、wire proto、LiDAR、
Recorder、Export 或 Viewer verifier。实现代理不得 commit/push。

### A4：全新正式链与语义预检

> 2026-08-16 状态修订：本节及紧随其后的 A5 不再执行。其 Golf 证据链与最终收口已由
> `2026-08-16-mid360-golf-mapping-replay-implementation.md` 和
> `2026-08-16-project-closeout-execution.md` 取代；本计划保留原文、失败目录和历史判据，
> 不改写为成功证据。

唯一证据根固定为
`/home/cancade/pybullet_df_drive/results/stage4-golf-obstacles-motion/`。执行前要求该路径不存在；
任何失败现场都原样保留并在同一根下用编号 attempt 追加，绝不覆盖
`results/stage4-lvx2-closeout/`。本轮固定：

- scene id：`stage4-golf-obstacles-motion`；
- simulation session id：`474f4c462d4d4f54494f4e2d30303031`（ASCII
  `GOLF-MOTION-0001`）；
- world/command generation：`1/1`；
- 正式窗口：5 秒，预期 Command/WheelState 各 500 帧，LiDAR/RTK/IMU 各 50 帧。

从真实 PyBullet `df_mid` session 串行走正式 v2、C++ Command/Recorder、完整 MCAP 和
C++ Export；控制 payload 必须由本轮 descriptor/codec 生成。点云唯一来源是 Recorder 的
完整 MCAP，禁止 fixture、补假点或 padding。记录 MCAP Export 前后 SHA-256 相等，并把
scene YAML、scene digest、participant argv/result/log、descriptor、payload、MCAP、50 组
PCD/PLY、LVX2、sidecar 和独立校验全部绑定到 append-only manifest。

Viewer 前必须先从源 MCAP/protobuf 与 PCD/LVX2 交叉生成结构化语义报告：

- 全局与逐帧 XYZ bbox、`z_span`，并证明 golf 起伏不是 flat；
- `tag/reflectivity` 全类别计数，且 tag 1、2、3 都非零；
- tag 3 的逐帧 bbox/质心/最近距离发生变化，证明移动障碍物实际移动而非只有 moving 标签；
- WheelState 存在等速、左右差速、显式零速三段；RTK 中心位置与 heading 都发生确定性变化；
- scene obstacle snapshot 与点云命中可对应，静态和移动障碍物均在实际可见窗口出现。

任一语义门失败就保留现场并停止，不打开 Viewer，也不得通过修改扫描建模来迁就结果。

### A5：Livox Viewer 2 独立窗口验收与停止点

语义门通过后，在全新的独立 Viewer case 中按绝对路径打开本轮 `lidar.lvx2`，使用透视
视角播放，不以 top-view 代替。证据必须同时绑定：LVX2 绝对路径与 SHA-256、进程/窗口、
`OpenLvxFile` success、非零 packet、非零 `PointsNum`、播放进度和清晰截图。Viewer 窗口
必须保持打开供用户目视判断；不要在用户评价前终止窗口或修改扫描射线。

完成后只向用户报告新文件路径、Viewer 窗口状态、XYZ/Z span、tag/reflectivity、逐帧
移动与车辆运动统计，然后暂停执行。不得继续 Task 8、完整回归、最终文档收口或最终六维
审查，直到用户明确决定后续扫描建模方向。

### 追加并发裁决

| Agent/模型 | 边界 | 状态 |
|---|---|---|
| golf 设计 Sol ultra | 严格只读，冻结 A1-A3 最小设计 | 已完成 |
| golf 实现 Terra high | 唯一写入代理，严格线性 TDD | 待执行 |
| Viewer v4 旧审计 | 只读 flat/profile8 摘要，不启动 Viewer | 已完成 |
| Task 6 旧审计 | 只读旧 runbook/evidence，不读取新根 | 已完成 |
| Dashboard Terra | 会与 runtime 公共路径冲突 | 保持 interrupted |

同一时间只允许一个实现代理修改上述生产/测试路径。Terra 实现完成后先做独立规格符合性
复审，再做代码质量复审；只有前一阶段无未解决问题才进入下一阶段。新 Viewer 展示始终
高于任何旧审计、Dashboard 修复或阶段四最终收口。

## 2026-08-15 追加修订：A2.5 LiDAR 吞吐与 A3.5 Recorder 启动协调

本节是用户批准的 append-only 修订，优先级高于本文件前述 A2/A3 文件边界以及 A4/A5
执行顺序。A1、A2 和 A3 的既有 GREEN 证据继续有效；A4、A5、阶段四最终回归和最终
六维审查继续暂停，直到本节 A2.5 与 A3.5 全部通过。

### 已确认现状与兼容边界

- seed 41 Golf 真实 worker 单帧扫描约为 `134..234 ms`，超过固定的 `100 ms`
  capture-to-response 合同；迟到 payload 被整帧拒绝，因此旧 5 秒尝试实际交付的
  LiDAR/RTK/IMU 均为 `0`。`published_frames=50` 只统计调度 deadline，不能作为交付证据。
- A2.5 与 A3.5 在本修订批准时尚未实施，不能称为“点阵云改善已经执行”。
- 用户批准受控迁移采用毫米级等价：同一锁定环境、world、mount、snapshot、identity、
  sequence 和 capture time 下，5,760 条射线的 hit/miss、全局点序、offset、line、tag、
  reflectivity 和消息字段必须严格一致；每个 XYZ 分量与标量基线的偏差不得超过
  `0.001 m`。这不是跨平台或跨 protobuf 版本的公开 ABI 承诺。
- 首选实现已经在只读原型中达到逐 bit 端点和 deterministic protobuf bytes 一致；允许
  毫米级等价不授权主动降低首选路径精度，也不授权修改射线、profile、点数、padding、
  100 ms 阈值或超时整帧拒绝语义。

### A2.5-a：Stage4-only NumPy 等价快路（首选）

只修改 `slope_sim/lidar_pointcloud.py` 与 `slope_sim/sensor_backend.py`：

1. Stage4 scanner 启动时把既有 local starts/ends 缓存为只读、C-contiguous
   `float64 ndarray`；NumPy `2.2.6` 已由 Stage4 Python lock 冻结，不修改
   `pyproject.toml` 或增加依赖。NumPy 只能在 Stage4 私有构造/扫描路径内延迟加载，不能让
   Stage3 或模块导入路径新增运行时要求。
2. 每帧继续使用 parent 物理步后冻结的中心 `lidar_link` pose 与完整无 body-id obstacle
   snapshot。端点变换按现有标量表达式的左结合运算顺序逐项执行，不使用
   `matmul`/`einsum`。
3. 新增私有 ndarray indexed-hit 入口，仍调用同一个
   `p.rayTestBatch(..., numThreads=0)`；raw hit 后的严格校验、逆变换、字段生成和唯一
   protobuf 编码复用现有路径。
4. Stage3、公开 `SensorBackend` 协议、`LidarScanService`/IPC dataclass、wire proto 和
   Dashboard 均保持不变。

只读基准在 seed 41 Golf 的 24 帧交替测量中为：当前 tuple 路径
median/p95/max=`53.70/90.61/102.98 ms`，NumPy 路径为
`45.51/73.64/85.65 ms`。该基准不含 Pipe、snapshot reconcile 和 parent 调度，不能替代
下述真实 50 帧门。

线性 TDD：

```bash
conda run -n slope-sim python -m pytest -q \
  tests/integration/test_sensor_backend.py \
  tests/integration/test_lidar_worker.py \
  -k 'stage4_numpy_batch_matches_scalar_oracle or stage4_golf_fifty_frame_budget'
```

RED 必须先证明旧路径没有 ndarray 快路或真实 50 帧不满足预算；GREEN 必须同时证明：

- 5,760 个全局 ray index 无缺口、无重复且顺序不变；
- XYZ 每轴偏差不超过 `0.001 m`，其余命中与点字段严格一致；
- 真实 `spawn` service 连续 50 帧全部返回有效 payload；
- 每帧从 parent capture 到 parent 收到并校验 response 的完整时延 `<=100_000_000 ns`；
- service snapshot 为 `completed=50`、`overrun/failed/stale=0`，事件流中没有
  `capture_rejected`；50 帧聚合后的 tag 1/2/3 均非零。

若完整 50 帧门失败，保留 RED/GREEN 现场并停止 Python 微调，不放宽阈值，不进入 A3.5
或 A4，转入已批准的 A2.5-b。

### A2.5-b：双 DIRECT world 连续分片（条件后备）

本路径只在 A2.5-a 的真实 50 帧门失败时实施。协调者以 `spawn` 创建恰好两个 shard；每个
shard 拥有独立 DIRECT client、完整相同 world/body 映射和同一 frozen snapshot，分别处理
连续全局 index `0..2879` 与 `2880..5759`。两侧显式限制 Bullet 内部线程，避免
4C/8T 环境发生进程与内部线程超订阅。

shard 返回带全局 index 的紧凑命中结果；协调者必须验证无缺口、无重复，即使返回乱序也
按 index 合并，并且只在两侧均成功后执行一次 protobuf 编码。任一 shard 启动失败、超时、
异常、缺失或身份不一致都使整帧失败；启动失败、正常 Stop/ACK 和异常退出都必须证明 child
已回收。不得把半帧或上次结果作为 fallback 发布。

后备路径允许修改 `slope_sim/lidar_worker.py`、`slope_sim/sensor_backend.py` 及其现有测试；
`sensor_backend.py` 只能增加供 Stage4 shard 私有调用的 thread-count 参数，并继续复用现有
命中校验逻辑，不能复制第二套 backend 或协议。不得修改 wire proto、公开 Stage3 扫描入口
或发行依赖。定制 PyBullet packed-hit wheel 因扩大依赖/发行边界而排除；heightfield 转
BVH mesh 虽然位置误差小于毫米，但会改变全部 terrain normal，也排除。

### A3.5：Recorder ready/start 协调

A2.5 通过后，只在 `cpp/client/stage4_recorder.cpp` 增加可选且 all-or-nothing 的参数组：

```text
--ready-file /absolute/recorder.ready
--start-file /absolute/start.signal
--expected-publisher-count 1
```

未提供参数组时保持旧 CLI 与行为不变。部分提供、expected 值不是字面值 `1`、路径不是
绝对规范路径、ready/start 不是两个不同 sibling、与 output/result 重名或 marker 预先存在，
均以既有 invalid-argument 退出码 `64` 拒绝。

协调时序冻结为：

1. 构造 RecorderSession、五个 subscriber 并安装全部 callback。
2. 在同一 monotonic deadline 内等待五个 subscriber 同时满足
   `GetPublisherCount()==1`；任一出现 `>1` 立即失败。
3. 以 `O_CREAT|O_EXCL|O_CLOEXEC, 0600` 创建 `recorder.ready`。
4. callback 在 ready 前已经 armed。协调模式新增一个原子 start gate：主循环或首个 callback
   观察到 shared start 是新建的常规文件后只锁存一次，再继续原有复制、验证和入队；若
   callback 在 start 文件尚不存在时收到 payload，则锁存 protocol fault 且该帧不得入队。
   这样不会因 Recorder 主循环比 producer 晚一次 poll 而丢第 0 帧，也不会把 producer 的
   提前发送混入正式窗口。等待期间持续检查五个 peer，任一不再等于 `1` 即失败。
5. 看到 start 后立即再次取得五话题 exact-one 快照，再进入原有 exact-count drain；录制
   窗口内持续拒绝 peer 漂移或竞争者。
6. 协调模式继续以现有 `plan.expected_counts`、sequence 和身份判断完整窗口；只有 Golf 的
   `--duration-ms 5000` 对应 `500/500/50/50/50`。完整时才 finalize，运行期失败不发布
   最终 MCAP；ready 已成功创建后的失败保留排他 result/marker/log 供审计。CLI 校验或
   预存 marker 在 `RunRecorder` 前以 `64` 退出，不承诺生成 result/ready。现有 result JSON
   schema 不变。

线性 TDD：

测试先在 `tests/stage4/test_cpp_client_sdk.py` 增加 `_recorder_tool()`，显式读取并校验
`STAGE4_RECORDER_EXECUTABLE`；不得让该聚焦门隐式调用 `_client_tool()` 现场构建。RED 使用
当前保留的 2026-08-11 旧二进制：

```bash
LD_LIBRARY_PATH=/home/cancade/miniforge3/envs/slope-sim/lib/python3.10/site-packages/ecal \
STAGE4_RECORDER_EXECUTABLE=/home/cancade/pybullet_df_drive/build/stage4-phase0-ecal611-release-mapped-20260809T220738+0800/slope_sim_stage4_recorder \
conda run -n slope-sim python -m pytest -q \
  tests/stage4/test_cpp_client_sdk.py \
  -k 'recorder_coordinated_start_requires_five_single_publishers'
```

RED 必须因旧 Recorder 不认识协调参数而失败且不产生 ready。依赖 preflight 通过后，GREEN
先在同一 canonical root 构建 fresh Command 与 Recorder：

```bash
cmake --build /home/cancade/pybullet_df_drive/build/stage4-phase0-ecal611-release-mapped-20260809T220738+0800 \
  --target slope_sim_stage4_command slope_sim_stage4_recorder
```

再用上面的同一 pytest 命令和同一路径 fresh Recorder 重跑。GREEN 必须覆盖四个 publisher
不 ready、第五个到齐后 ready、start 前无最终 output/result、唯一 publisher 在 start 前发送
合法帧仍 fail closed、start 后完整 MCAP，以及部分 CLI、预存 marker、竞争 publisher、peer
漂移和 timeout。随后运行既有无 marker Recorder 成功/故障测试，证明兼容。

### Golf 联合门与两个 runtime 缺口

允许同步修改 `scripts/stage4_v2_simulation_runtime.py`、
`tests/stage4/test_v2_simulation_runtime.py` 和 `tests/stage4/test_c2_supervisor.py`，只完成：

- runtime CLI 增加显式 `--require-verified-peers` 并传给
  `run_v2_simulation_runtime(require_verified_peers=True)`；带 ready/start 协调但未启用该门时
  继续 fail closed；
- runtime 在 `drain_sensor_outputs()` 完成、关闭 worker 前冻结 `lidar_service.snapshot()`，并在
  result 的 `lidar_worker.service_snapshot` 输出 `completed_count`、`failed_count`、
  `overrun_count`、`stale_count` 和 `max_capture_to_response_ns`；Golf 必须为 `50/0/0/0` 且
  最大时延 `<=100_000_000 ns`；
- runtime result 从 `/sim/wheel/command` 的冻结 topic observation 输出
  `dashboard_snapshot.command_sequence`，Golf 最终值必须为 `499`；不修改 Dashboard 模型；
- 本阶段唯一三进程 Golf orchestrator 是
  `tests/stage4/test_c2_supervisor.py::test_golf_obstacles_motion_completes_five_second_scene_motion_window`；
  它给 Recorder 传协调三元组，等待 runtime、Command、Recorder 三个 ready 全部出现后才以
  排他方式创建唯一 shared start，三方启动后各自复核 peer；不修改
  `scripts/stage4_c2_supervisor.py`。

联合门必须证明 runtime、Command、Recorder 均正常退出，Command/WheelState 各 500 帧，
LiDAR/RTK/IMU 各 50 帧，Recorder 总计 1,150 帧并生成完整 MCAP；LiDAR 数量必须来自
Recorder 和 worker completed 证据，不能使用 runtime `published_frames` 代替。

fresh Command/Recorder 可用后，联合门命令固定为：

```bash
LD_LIBRARY_PATH=/home/cancade/miniforge3/envs/slope-sim/lib/python3.10/site-packages/ecal \
STAGE4_COMMAND_EXECUTABLE=/home/cancade/pybullet_df_drive/build/stage4-phase0-ecal611-release-mapped-20260809T220738+0800/slope_sim_stage4_command \
STAGE4_RECORDER_EXECUTABLE=/home/cancade/pybullet_df_drive/build/stage4-phase0-ecal611-release-mapped-20260809T220738+0800/slope_sim_stage4_recorder \
conda run -n slope-sim python -m pytest -q \
  tests/stage4/test_c2_supervisor.py \
  -k golf_obstacles_motion_completes_five_second_scene_motion_window
```

### 实施、构建与审查裁决

- Sol ultra 只负责架构、书面规格自审和最终只读审查；Terra high 是唯一代码写入代理，按
  A2.5 RED -> 最小 GREEN -> 相关回归 -> A3.5 RED -> 最小 GREEN -> 相关回归 -> Golf 联合门
  严格线性执行。不得让多个 agent 同时修改同一生产或测试路径。
- Recorder 写入前先只读定位可复用的锁定 C++ headers、libraries 和构建命令。当前
  canonical root 中旧 Recorder 可用于 RED，但 build.ninja 引用的 Protobuf 33.6、Abseil、
  MCAP 与 eCAL prefix 已不存在，不能把现有 binary 或当前 Conda 的 libprotobuf 7.35.1
  混用为 fresh GREEN。必须先恢复同一锁定 prefix，之后才允许执行上述 `cmake --build`。
  若恢复依赖或新建构建根预计写入超过 `5 GiB`，必须报告估算、复用失败原因与保留/清理
  方案并取得专项授权；当前一般“授权”不越过该磁盘门。
- A2.5/A3.5 和联合门全部通过后，只启动一次 Sol ultra 六维只读审查，覆盖需求完整性、
  逻辑正确性、边界、代码质量、测试和实际运行证据；局部修复只做针对性复验。
- 审查无未解决问题后才创建全新的
  `results/stage4-golf-obstacles-motion/`，执行 A4 语义证据链；A4 全部通过后执行 A5，打开
  Livox Viewer 2 并保持窗口运行，等待用户目视决定。正式阶段四默认回归只在最终收口时
  运行一次。
- 不 commit、不 push、不发布，不覆盖或清理现有工作树改动。

### 2026-08-15 批准修订：A2.5 交错 Stage4 shard 合同

本修订替换此前 Stage4 连续 range 私有合同，仅用于两个 parent-owned DIRECT shard 的预热和
hot scan 分配。私有 assignment 固定为
`((0, 5760, 2, 2880), (1, 5760, 2, 2880))`，字段依次为
`first/stop/stride/count`：shard 0 只处理全局索引 `0, 2, ..., 5758`，shard 1 只处理
`1, 3, ..., 5759`，每侧恰为 2,880 条。每个 shard 的预热与 hot scan 固定使用
`_stage4_realtime_shard_thread_count == (2, 2)`；ray 输入必须由
`starts[first:stop:stride]` 和对应 ends 建立 C-order、readonly 副本，local hit index 映射为
`first + local_index * stride`。coordinator 必须按 global index 恢复完整顺序。

5,760 条 ray/profile、点语义、wire、100ms 整帧预算和 fail-closed 边界均不改变。正确性以真实
双 DIRECT shard worlds 与另一单独 DIRECT scalar oracle 对照：oracle 仅可使用
`_transform_points` 和公共 `backend.ray_test_batch`，不得复用 `_scan_frozen`、ndarray helper
或 shard world；消息身份、离散字段、全局顺序严格相同，XYZ 每轴误差不超过 1mm。A2.5 先完成
RED/最小 GREEN/相关回归与 oracle GREEN，之后且仅运行一次正式 50 帧门；该门是 A2.5 最终
放行条件，失败即停止。未完成前不得进入 A3.5。
