# 当前与阶段四目标架构

> 最近更新：2026-08-12
>
> 目标平台：一台 Ubuntu 24.04 amd64 电脑
>
> 状态：阶段四 A-E 已实现并完成对应门禁；实时完成状态与证据以
> `docs/阶段四交付报告.md` 为准

## 1. 如何阅读本文

本文明确区分两件事：

- **当前已落地**：仓库今天能够运行或已有验证证据的代码。
- **阶段四运行形态**：用户确认并已按 B-E 实施、验收的组件关系；具体完成证据不由
  本文重复声明。

产品需求以 `3d仿真平台需求规格.md` 为准；阶段四协议、安装和验收细节以
`docs/superpowers/specs/2026-07-31-stage4-mid360-ecal-cpp-delivery-design.md`
为准；完成状态以 `docs/阶段四交付报告.md` 为准。本文不把目标图当成已交付
事实。

## 2. 当前已落地

### 2.1 现有 Python 运行链

```text
main.py
  |-- run_experiment()      GUI / DIRECT 自动实验
  `-- run_manual_demo()     GUI 人工驾驶与 Dashboard
          |
          +-- SimulationCoordinator
          |     |-- 当前有效车型与场地
          |     `-- ObstacleManager
          +-- InterfaceRuntime
          |     |-- local 或阶段三 eCAL v1 transport
          |     |-- 轮控安全邮箱与传感器调度
          |     `-- 异步接口日志 / LiDAR worker
          `-- TelemetryDashboard
                `-- 只读取不可变快照
```

PyBullet 世界、车型、场地和障碍物由运行仿真的主线程拥有。Dashboard 按钮只
提交结构化动作；`SimulationCoordinator` 在物理主线程内执行车型/场地事务并
处理回滚。eCAL callback、日志线程、Dashboard 和 LiDAR 子进程不能直接调用
PyBullet。

当前 `create_interface_session()` 仍通过阶段三 `InterfaceConfig` 和
`create_transport()` 建立生产接口，Dashboard 快照仍包含 v1 前/后 LiDAR。
`auto` 仅是现有开发入口；阶段四正式 `ecal` 模式不得静默回退 local。

### 2.2 已有领域能力

- 四种车型：三种差速车与一台主动转向四驱车。
- 三类场地：平面、三段斜面和可复现高尔夫 heightfield。
- 静态、移动和混合障碍物，以及运行期车型/场地/障碍物事务。
- GUI 人工驾驶、相机跟随、Dashboard、CSV 和事件日志。
- 版本化场景导入/导出和失败回滚。
- 阶段三 v1 eCAL、企业传感器基线与异步 LiDAR worker。

### 2.3 阶段四实现状态

阶段四 A-E 已按本文件的目标运行图实现：v2 schema、确定性 codec、
`ProtocolSession`、`CommandAuthority`、raw eCAL transport、Python/C++ golden、
单中心 LiDAR、三点 RTK、正式 runtime、C++ SDK/Command/Subscriber/Recorder、
Replay/Export、可选 ROS 与联网 `.run` 安装器均有对应测试或真实门禁证据。
阶段三 v1 链保留为历史入口与回归基线；阶段四正式 runtime 不静默回退到 local。
`scripts/run_mid360_golf_mapping.py` 是与实时链隔离的固定场景采集/回放入口：它只消费
完整 MCAP 和 Recorder 成功结果，并在独立进程中完成路线、地形、运动及回放验证，不读取
活跃 PyBullet 世界。各项真实命令、结果和残余风险只以交付报告为准，本文不重复叙述历史执行过程。

## 3. 阶段四目标运行图

```text
                         local Unix control socket
                    +-------------------------------+
                    |      Python orchestrator      |
                    | start / status / stop barrier |
                    +-------------------------------+
                                     |
+------------------------------------+------------------------------------+
|                    one Ubuntu workstation                              |
|                                                                         |
| C++ Command  -- /sim/wheel/command --> Python/PyBullet Simulator        |
| C++ Command  <-- /sim/wheel/state   --- Python/PyBullet Simulator       |
|                                                                         |
| Simulator -- wheel/lidar/rtk/imu --> C++ Subscriber / public SDK        |
|           |                         -> C++ Recorder -> MCAP sessions     |
|           |                         -> optional ROS Bridge -> RViz2      |
|           `-> bounded snapshots     -> Qt Dashboard                     |
|                                                                         |
| complete session manifest -> Replay -> /replay/sim/*                    |
|                           `-> Export -> PCD / PLY / synthetic LVX2       |
| complete Golf MCAP       -> mapping replay -> isolated Qt/OpenGL views   |
+-------------------------------------------------------------------------+
```

所有正式组件都在同一台电脑运行。系统不使用 Docker、远程服务、数据库、
第二台构建机或第二个业务消息总线。安装完成后的运行流程不主动联网。

## 4. 组件边界

### 4.1 Simulator

Python/PyBullet Simulator 是唯一物理和传感器真值生产者，负责：

- 240 Hz 物理步进与场景事务。
- 轮控租约、超时停车和 command generation。
- 100 Hz WheelState。
- 10 Hz 单中心 LiDAR、三点 RTK 和 IMU 同刻采样。
- 给 Dashboard 的有界显示快照。

LiDAR 可以使用持久 worker 计算，但 worker 只能处理冻结输入并返回结果，不能
取得世界所有权。日志、绘图和 Recorder I/O 都不进入物理主线程。

### 4.2 Command

`slope-sim-command` 是唯一正式 WheelCommand publisher。每次启动生成新的
source session，并根据 WheelState 取得命令权。零 peer 时等待并停车，多 peer
时进入冲突并停车；按键释放、窗口失焦、连接断开和 100 ms 租约过期都归零。

Command 独立成进程是为了隔离人工输入与物理循环，不再拆分第二套控制服务。

### 4.3 Subscriber 与 SDK

`libslope_sim_client` 封装一次协议身份、descriptor、generation 和 sequence
校验。`slope-sim-sub`、Recorder、ROS Bridge 和外部只读 C++ 客户端复用它，
不得各自复制一套 topic 合法性判据。

### 4.4 Recorder

`slope-sim-record` 保存五个正式 topic 的原始 Protobuf bytes 和一一配对的记录
metadata。它使用有界队列和独立磁盘线程；队列溢出、磁盘不足或持久化失败会
通知编排器安全停车，而不是静默丢帧。

Recorder 保持独立进程是为了让磁盘故障不能卡住 PyBullet，也让 MCAP 会话能在
正常停止时独立 drain、flush、fsync 和 finalize。

### 4.5 Replay、Export 与可选 ROS

Replay 和 Export 只读取已经完成并校验的 session manifest。Replay 默认发布到
`/replay/sim/*` 且不发送 WheelCommand；Export 生成 PCD、PLY 和 synthetic
LVX2，原始 MCAP 始终是权威数据。

ROS 2 Jazzy Bridge/RViz2 是 `--with-ros` 才安装的下游适配层，也是实时点云显示
的验收路径。它失败或关闭时，Simulator、eCAL 和 Recorder 继续运行。LVX2 profile 8
已由 Viewer 读取、播放到结尾并显示非空点云；Viewer 退出仍出现 `SIGSEGV/rc139`，故它是
专有外部 Viewer 的 clean-shutdown concern，不阻塞项目内回放。该结论不等同于真实
MID-360 发现或光学仿真。

## 5. 数据面与控制面

### 5.1 正式数据面

eCAL Protobuf v2 固定承载五个 topic：

| Topic | 方向 | 频率 |
|---|---|---:|
| `/sim/wheel/command` | Command -> Simulator | 100 Hz |
| `/sim/wheel/state` | Simulator -> consumers | 100 Hz |
| `/sim/lidar/points` | Simulator -> consumers | 10 Hz |
| `/sim/rtk/state` | Simulator -> consumers | 10 Hz |
| `/sim/imu/attitude` | Simulator -> consumers | 10 Hz |

v1 只用于历史读取。生产 Simulator 完成 B 阶段切换后只发布 v2，不长期双发，
也不引入 `/sim/v2/...` 第二套业务 topic。

### 5.2 本机控制面

Unix socket 只承载进程启动状态、人工目标、健康状态和停止屏障，不传输点云或
MCAP 数据。socket 目录权限为 `0700`，编排器用 peer uid/PID 核对由自己启动的
进程。控制面不扩展为网络 API。

## 6. 生命周期

编排器在创建首个正式 participant 前取得单实例锁。正常停止按以下顺序：

1. Command 发布最终零命令并冻结 publisher。
2. Simulator 继续至少一个 10 Hz 周期，冻结四个输出边界。
3. Recorder 接收到边界内消息后排空队列并 flush/fsync。
4. Recorder 原子完成 segment 和 session manifest，报告 `FINALIZED`。
5. 编排器关闭其余 participant，释放 socket 和单实例锁。

任何场景重建都会撤销旧命令 token。Recorder fatal、Command peer 冲突或控制面
失联都先停车，再有序关闭。

## 7. 场景与传感器状态

阶段四 schema v2 保留四车型、三场地、障碍物和传感器 profile。场景重建成功
才推进 `world_generation`；失败回滚旧世界，但旧命令 token 不恢复。

每个阶段四车型只有一个中心 `lidar_link` 和固定
`LEFT/CENTER/RIGHT` RTK 几何。WheelState 为 100 Hz；每第十个 WheelState
采样位点同时产生 LiDAR、RTK 和 IMU，四类输出时间戳完全一致。Dashboard、
Recorder、Export 和 ROS Bridge 都消费这套身份，不自行最近邻配对。

## 8. 安装后目录

```text
/opt/slope-sim/
  current -> releases/<version>
  releases/
    <version>/
      bin/ lib/ include/ share/ runtime/

~/.config/slope-sim/       user overrides
~/slope-sim-data/          MCAP, export and logs
```

单一 `.run` 安装器联网获取已锁定依赖。安装、回退和卸载共用全局锁，每轮写唯一
staging；doctor/smoke 通过后才原子发布版本并切换 `current`。同版本只校验
payload、`with_ros` 和文件；完整但未激活时只补做 `current` 切换，损坏或选项
漂移时拒绝原地覆盖。用户配置和数据不放入 release，不随升级或回退重写。

## 9. 轻量化取舍

保留多进程不是为了扩展成分布式系统，而是现有功能本身需要 Python/PyBullet、
C++ Command、C++ Recorder 和可选 ROS 的语言与故障隔离。继续合并这些进程会
扩大重写量，并让磁盘、输入和 ROS 故障进入物理主循环。

本次轻量化删除的是单机交付不需要的基础设施：

- 离线 Conda/package/source cache 和 materializer。
- network namespace、无路由证明和断网构建门。
- `.tar.zst` 离线迁移包、双根 byte-identical 构建和 lifecycle probe。
- SSH challenge、跨机器 evidence、accepted-candidate/final-status 状态机。
- Docker、数据库、远程控制服务和常驻 system service。

仍保留公开 topic、业务功能、依赖版本与 SHA、单进程 ABI、路径安全、失败不
切换、真实运行验收和六维审查。

## 10. 实施状态

| 阶段 | 状态 | 进入下一阶段前的边界 |
|---|---|---|
| A：v2 协议与命令权 | 已完成 | Phase-0、golden 和独立六维审查已通过 |
| B：单 LiDAR、三点 RTK、生产 v2 runtime | 已完成 | 四车型三场地、性能和真实 GUI 门已通过 |
| C：C++ SDK、Command、Subscriber、Recorder | 已完成 | 五 topic 联合会话、记录与停止门已通过 |
| D：Replay、Export、可选 ROS/RViz2 | 已完成 | 回放隔离、导出回读和 ROS/RViz2 实时显示 smoke 已通过；Viewer 标准 LVX2 离线回放仍待验收 |
| E：`.run` 安装与最终验收 | 已完成 | 干净 Ubuntu 安装、联合负载和独立六维审查已通过 |

旧 master/A-E 详细计划已经暂停，只能作为 Git 历史阅读，不能继续执行；当前
轻量实施记录与证据汇总见交付报告。
