# 阶段四 B-E 单机轻量实施计划

> 状态索引：B-E 已完成；本文件保留已执行范围、TDD 门和完成条件，不再作为待办
> 执行。实际命令、证据与最终审查结论见 `docs/阶段四交付报告.md`。

> 起点：阶段 A 已完成。复用冻结 v2 schema/descriptor、`ProtocolSession`、
> `CommandAuthority`、raw eCAL transport、Python/C++ golden 与真实 Phase-0 证据。
>
> 目标：在一台 Ubuntu 24.04 amd64 上交付 Python/PyBullet Simulator、C++ Command/
> Subscriber/SDK/Recorder、Replay/Export、可选 ROS 2 与唯一联网 `.run` 安装器。

## 范围

- 保留四车型、三场地、障碍物、GUI 与 Dashboard；正式数据面固定五个 v2 topic。
- 不实现自动导航、路径规划、动态避障、跨机流程、离线 cache/materializer、网络
  namespace、双根复现、SSH/交接证据或原地 `--repair`。
- 每个重要行为按一个线性 TDD 单元执行：RED、最小实现、GREEN、相关回归；每个
  B/C/D/E 阶段结束后启动独立六维只读审查。

## Task 0：审计并精简已废弃资产

- **目标**：建立旧文档、测试、fixture、脚本和 build 输出的引用表；只删除能证明无
  调用方且有唯一替代物的条目，并将测试按 unit/integration/acceptance/stage4 所有权
  归类。
- **涉及文件**：`docs/`、`docs/superpowers/plans/`、`tests/`、`scripts/`、
  `reference/`、CI/锁文件；不直接编辑生成物或 `build/` 产物。
- **验证**：删除前以 CodeGraph、`rg` 和 CI 配置确认无引用；重组后运行受影响测试
  与 `git diff --check`，报告删除/合并依据及净文件/行数。
- **完成条件**：每项移除有引用审计记录，关键故障与真实门禁覆盖不下降。

## Task B1：中心 LiDAR 与三点 RTK 领域合同

- **目标**：为四车型提供唯一中心 `lidar_link`、MID-360 风格候选射线/冻结快照
  采样、LEFT/CENTER/RIGHT RTK 和基线航向恢复；保留现有场地、障碍物和场景事务。
- **涉及文件**：`slope_sim/` 传感器/模型/场景模块、`tests/unit/` 与
  `tests/integration/` 对应传感器测试；复用 A 的 v2 模型与 codec，不复制协议判据。
- **RED/GREEN**：先断言中心安装、点字段、同帧世界快照、非零 roll/pitch 三点航向
  和非法几何拒绝；最小实现后通过同一测试。
- **回归/真实门禁**：传感器、四车型三场地 DIRECT 回归；headless 10 Hz 采样与
  240 Hz 物理预算证据。
- **完成条件**：5,760 实时与 20,000 dense profile、三点 RTK/IMU 合同均可复现，
  且无 PyBullet 所有权越界。

## Task B2：正式 v2 Simulator runtime 与 Dashboard

- **目标**：将 B1 真值接入 A 的五 topic v2 runtime：100 Hz wheel，统一 10 Hz
  LiDAR/RTK/IMU 时间戳；GUI/Dashboard 展示有界快照，不进入物理循环。
- **涉及文件**：`slope_sim/interfaces/v2/`、simulation/runtime 入口、Dashboard
  adapters、`tests/integration/`、`tests/acceptance/`。
- **RED/GREEN**：先断言同刻输出身份/sequence、authority 安全停车、scene rebuild
  generation 与 UI 有界消费；最小接线后通过。
- **回归/真实门禁**：四车型乘三场地 headless 性能；真实 eCAL 五 topic、GUI、
  5 秒 240/100/10 Hz 窗口与零 producer drop。
- **完成条件**：正式 eCAL 初始化失败退出，LocalTransport 仅显式开发/测试可用。

## Task C1：C++17 SDK、Command 与只读 Subscriber

- **目标**：在 `cpp/` 建立 `libslope_sim_client`、唯一 Command publisher 和只读
  Subscriber，复用 A 的 raw descriptor/metadata/authority 边界与生命周期顺序。
- **涉及文件**：`cpp/` CMake、public headers、Command/Subscriber tools、相关
  `tests/stage4/` 与 CTest。
- **RED/GREEN**：先断言五 topic metadata、命令租约/安全停止、单实例和 Python/C++ raw
  bytes；最小实现后通过。
- **回归/真实门禁**：C++ golden/ABI、Simulator+Command+Subscriber 真实 eCAL
  会话，检查单一 command peer 与有序停止。
- **完成条件**：SDK 被所有 C++ consumer 复用，不创建第二套 protobuf/session 判据。

## Task C2：MCAP Recorder 与会话完成事务

- **目标**：独立 Recorder 无损记录五 topic 原始 bytes、身份/场景 manifest 和原子
  segment；队列/磁盘/持久化故障触发安全停止，不阻塞 PyBullet。
- **涉及文件**：`cpp/` recorder/session modules、编排控制面、`tests/integration/`
  与 CTest。
- **RED/GREEN**：先断言队列边界、临时 segment、flush/fsync/finalize、CRC/磁盘
  故障和 manifest 完整性；最小实现后通过。
- **回归/真实门禁**：真实 Simulator+Command+Subscriber+Recorder 5 秒窗口，
  consumer/recorder drop/error 为零，停止后队列为零且 manifest finalized。
- **完成条件**：完成的 MCAP 是后续 Replay/Export 唯一权威输入。

## Task D：安全 Reader、Replay/Export 与可选 ROS

- **目标**：验证完成 session 后隔离回放到 `/replay/sim/*`，默认不回放 command；
  从 MCAP 导出 PCD/PLY/synthetic LVX2。`--with-ros` 才提供 Jazzy Bridge/RViz2。
- **涉及文件**：`cpp/` reader/replay/export、可选 ROS package/bridge、
  `tests/integration/` 与 `tests/acceptance/`。
- **RED/GREEN**：先断言损坏 reader 拒绝、namespace 隔离、导出回读、同 session/
  generation/timestamp 的 ROS 配对和 Bridge 故障隔离；最小实现后通过。
- **回归/真实门禁**：完成会话回放/导出验收；启用 ROS 时运行 RViz2 与 Livox Viewer 2
  smoke，关闭/失败 Bridge 不影响 Simulator 或 Recorder。
- **完成条件**：原始 MCAP 只读且权威，导出可重试而不改变会话。

## Task E：联网 `.run` 安装器与最终联合验收

- **目标**：生成唯一 `slope-sim-stage4-<version>-ubuntu24.04-amd64.run`；普通用户
  下载/构建，sudo 仅用于 apt、全局锁、`/opt` 发布和原子 `current` 激活。
- **涉及文件**：`packaging/` 安装器、依赖锁/许可证/manifest、doctor/smoke 脚本和
  安装器测试；不重新引入离线 cache 或 `.tar.zst`。
- **RED/GREEN**：先断言平台/manifest SHA、锁竞争、唯一 staging、失败不切换、
  同版本身份/`with_ros`/文件校验、未激活只补 current、损坏或选项漂移拒绝覆盖；
  最小实现后通过。
- **回归/真实门禁**：本地 HTTP fixture 的联网安装测试；干净 Ubuntu 24.04 amd64
  联网安装核心 smoke，`--with-ros` 时 ROS/RViz2 smoke；最终真实联合负载。
- **完成条件**：运行时不临时联网下载，安装/回退/卸载非当前版本共享全局锁，最终
  六维只读审查 P0/P1 为零。
