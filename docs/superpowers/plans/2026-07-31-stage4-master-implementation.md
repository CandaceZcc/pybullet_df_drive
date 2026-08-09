# 阶段四总路线 Implementation Plan

> **Execution:** Use `subagent-driven-development` only when the user selects delegated execution; otherwise use `executing-plans`. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在阶段三 `ce3bee0` 基线上交付单中心 MID-360 风格点云、三点 RTK、eCAL Protobuf v2、C++ SDK/记录工具、可选 ROS 2/RViz2、MCAP/PCD/PLY/LVX2 和 Ubuntu 24.04 完整迁移包。

**Architecture:** Python/PyBullet 是唯一物理与传感器真值生产者，eCAL Protobuf v2 是正式实时主通道；C++ 组件按 Subscriber、Command、Recorder、Replay/Export 分进程隔离，ROS 2 只作为可选下游。实施拆成五个有硬停止门的子计划，先证明 raw eCAL、descriptor 和 ABI，再推进传感器、记录、显示与发行。

**Tech Stack:** Python 3.10、PyBullet、PySide6、Matplotlib、Protobuf 6.33.6、Eclipse eCAL 6.1.1、C++17/GCC 13/CMake 3.28、MCAP、Zstd、ROS 2 Jazzy、RViz2、Livox ROS Driver 2 消息。

> 2026-08-04 事实状态：P0 的本地异步 LiDAR worker 实现和 DIRECT 10x5 实时门已通过；真实 eCAL `active_steering_4wd 4+2` 与 `df_back 2+0` 均已 PASS。Task 2 已解除阻断，A-E 仍须按总计划顺序执行。

---

## 全局执行协议：严格 RED-GREEN-REFACTOR

本总计划和 A-E 子计划中的所有生产代码任务都执行同一循环；子计划未重复写出的规则仍然强制生效：

1. **RED**：先写一个只描述当前行为缺口的最小测试。缺少 Python 模块时必须把 import 放在测试函数内，使 pytest 正常收集后得到明确 `FAILED`。新增 C++ API 时必须先把测试 target 注册到 CMake/CTest；允许编译只因测试引用的目标 API 尚不存在而失败，但 `unknown target`、configure 失败、缺环境变量、缺构建目录、找不到测试工具、collection error 或 `SKIP` 都不是有效 RED。已有 CLI/库的行为变化必须运行到断言失败，不能退回编译失败。
2. **确认 RED**：运行聚焦命令，记录测试名、失败断言和预期/实际值。失败原因不是目标行为缺失时，先修测试或测试环境，禁止开始生产实现。
3. **GREEN**：只实现让本轮 RED 通过的最小代码，不顺带扩展下一任务合同。
4. **确认 GREEN**：先重跑同一聚焦命令，再运行任务列出的受影响回归；两者都通过才进入下一步。
5. **REFACTOR**：只在 GREEN 后消除本任务引入的重复、命名或所有权问题，然后再次运行同一聚焦测试和受影响回归。即使无需重构，也要在任务证据中记录“REFACTOR：无必要”。

纯文档、固定 reference 元数据、外部设备观察和人工验收不伪造 RED；它们使用 schema/静态校验或明确的外部门禁。真实 eCAL、GUI/RViz2、Livox Viewer 和干净机运行只能发生在相关自动测试 GREEN 之后，也不能反过来替代 TDD。每一条真实外部 invocation 都必须单独说明负载/时长并取得只覆盖紧随命令的授权；执行前即时扫描竞争进程和系统负载，失败保留证据并停止，任何复测都重新授权，禁止批量授权或把一次授权复用于下一条命令。所有真实 eCAL invocation 必须以 `env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD` 启动，verifier/正式 launcher 在创建首个 participant 前还要进程内断言这两个变量 absent，并拒绝任何 test shim、测试 IPC 或测试选择开关出现在安装树/runtime manifest；测试替身证据不得冒充真实 transport 证据。

## 权威输入与非目标

- 设计规格：`docs/superpowers/specs/2026-07-31-stage4-mid360-ecal-cpp-delivery-design.md`。
- 阶段三冻结基线：Git `ce3bee0`；v1 descriptor 和历史证据不得改写。
- 阶段四取消自动寻路、SLAM、自动避障决策和硬件级 MID-360 数字孪生。
- 五个正式 topic 保留用户确认的 `/sim/...` 名称；Phase 0 若不能可靠隔离 v1/v2，必须停止并重新请用户裁决，不能静默改 topic 或放松校验。
- 真实 eCAL、真实桌面/RViz2、Livox Viewer 和干净 Ubuntu 电脑都是外部门禁；单元测试不能替代它们。
- 正式 `/sim` 生产会话必须先取得主机级单实例锁；Dashboard/键盘经带租约的本地 control 消息驱动唯一 C++ Command publisher，Python 不创建第二个命令 publisher。
- 100 Hz WheelState 与 10 Hz LiDAR/RTK/IMU 使用整数共网格；每个 10 Hz 位点四条 timestamp 完全相等。RTK projected-lateral heading 不能直接当 Euler yaw，C++ 导出与 ROS TF 共用设计规格冻结的恢复公式。
- control socket 与 eCAL 没有跨通道到达顺序；EndBarrier 后 Recorder 仍接收到 required fence 为止。场景变更先冻结五个 publisher并完成物理 commit/rollback，成功后才持久化 attachment，ACK 后恢复业务。
- session reader 与离线安装器在读取内容前拒绝路径逃逸、链接和非普通文件；最终发行树把受控 staging 中的根内文件链接确定性物化并要求所有普通文件 `st_nlink == 1`。正式安装状态还必须绑定外部 archive basename/hash 与 build-evidence hash，不能在解包后丢失来源。正式归档只允许从 clean `HEAD` 的只读 Git snapshot 构建并记录 snapshot SHA-256，禁止把 staged/unstaged/untracked 内容打包后仍声称旧 Git SHA；双根可复现构建还必须同时消除 C/C++/ROS debug path、Python wheel/pyc 和生成元数据中的绝对工作路径，不能只规范外层 tar。

## 子计划与依赖

| 顺序 | 子计划 | 可交付结果 | 硬停止门 |
|---:|---|---|---|
| A | `2026-07-31-stage4-a-v2-protocol-session.md` | v2 schema、simulation session、命令权状态机、raw eCAL Python/C++ spike | raw bytes、metadata、ABI 或同 topic 隔离任一失败 |
| B | `2026-07-31-stage4-b-mid360-rtk-performance.md` | 单 LiDAR、三点 RTK、schema v2、Dashboard 与 golf 性能 | 四车型三地形 DIRECT、同刻网格、真值或代表性 golf GUI 性能失败 |
| C | `2026-07-31-stage4-c-cpp-ecal-recorder.md` | C++ SDK/CLI/Command/Recorder 与 MCAP 原始记录 | 手动控制租约、场景事务、任一 topic 双向窗口/fence、零 drop 或原子落盘失败 |
| D | `2026-07-31-stage4-d-ros2-replay-export.md` | ROS 2/RViz2、隔离回放、PCD/PLY、合成 LVX2 | 安全 session 读取、TF 姿态、回放隔离或 Viewer 打开失败 |
| E | `2026-07-31-stage4-e-release-acceptance.md` | `tar.zst`、离线安装、升级回退、全量验收与六维审查 | 单实例/安全安装、双根字节复现、真实联合负载、四分辨率 GUI 或干净机迁移失败 |

A 必须先完成。A 通过后，B 的纯 Python 传感器工作与 C 的 C++ 消费端骨架可以并行，但 C 的真实 eCAL/Recorder 门禁必须消费 B 的最终单 LiDAR/三点 RTK payload。D 依赖 B+C；E 只在 A-D 全部形成证据后开始。

## 全局文件结构

```text
proto/
  slope_sim_interfaces.proto                 # 冻结 v1
  slope_sim_interfaces_v2.proto              # Python/C++ 唯一 v2 schema
  slope_sim_control_v1.proto                 # Python/C++ 本地控制面
  slope_sim_record_v1.proto                  # MCAP 逐消息配对 metadata/session manifest
slope_sim/interfaces/v2/                     # v2 Python 模型、codec、session、authority
slope_sim/mid360_lidar.py                     # 确定性 R2 单雷达扫描
slope_sim/rtk_triplet.py                      # 二轮/四轮三点 RTK 几何
resources/models/robot_models.yaml            # 四车型跨语言 canonical 外参
cpp/                                          # C++ SDK、CLI、Command、Recorder、Replay/Export
ros2/slope_sim_bridge/                        # 可选 Jazzy Bridge
packaging/                                    # 发行编排、安装、自检、systemd user 模板
packaging/python-environment.yml              # 生产 Python 环境唯一声明
packaging/locks/python*.lock                  # Python runtime/toolchain 冻结锁
tests/stage4/                                 # Python/跨语言/场景/性能合同测试
tests/fixtures/stage4/selftest/recipe.json     # 安装包 MCAP self-test 的确定性输入
cpp/tests/                                    # C++ 单元和 golden 测试
scripts/verify_stage4_*.py                    # DIRECT、eCAL、GUI、发行验收入口
docs/阶段四交付报告.md                         # 只记录实际运行证据
```

## Task 1：冻结开始状态与证据目录

**Files:**
- Create: `docs/阶段四交付报告.md`
- Create: `results/stage4/.gitkeep`
- Modify: `.gitignore`
- Test: `tests/stage4/test_delivery_report_contract.py`

- [x] **Step 1: 写交付报告状态 RED**

```python
from pathlib import Path


def test_stage4_report_starts_without_false_pass_claims() -> None:
    report = Path("docs/阶段四交付报告.md")
    assert report.is_file(), "stage four delivery report is not implemented"
    text = report.read_text(encoding="utf-8")
    assert "实现状态：未开始" in text
    assert "| 真实 eCAL | 未执行 | 无 |" in text
    assert "| 真实 RViz2 | 未执行 | 无 |" in text
    assert "| Livox Viewer 2 | 未执行 | 无 |" in text
    assert "| 干净机迁移 | 未执行 | 无 |" in text
```

- [x] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_delivery_report_contract.py`

Expected: pytest 正常收集并 `FAILED`，唯一失败断言为阶段四交付报告尚未实现；不得是文件读取异常或 collection error。

- [x] **Step 3: 创建只陈述当前事实的报告骨架**

```markdown
# 阶段四交付报告

> 基线：ce3bee0
> 实现状态：未开始

| 外部门禁 | 当前状态 | 证据 |
|---|---|---|
| 真实 eCAL | 未执行 | 无 |
| 真实 RViz2 | 未执行 | 无 |
| Livox Viewer 2 | 未执行 | 无 |
| 干净机迁移 | 未执行 | 无 |
```

将 `results/stage4/*` 加入 `.gitignore`，只保留 `.gitkeep`；真实大文件、MCAP、PCD/PLY/LVX2 和临时日志不得进入 Git。

- [x] **Step 4: 运行 GREEN 与基线保护**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_delivery_report_contract.py tests/test_proto_contract.py`

Expected: PASS，且 v1 descriptor 测试继续通过。

- [x] **Step 5: REFACTOR 报告状态模型并原样复验**

只整理已覆盖的状态表、证据引用和否定状态表达；不得提前写 PASS 或更改 v1 基线。无需整理时记录“REFACTOR：无必要”，随后原样重跑 Step 4。

## Prerequisite P0：补齐阶段三 post-fix 真实 eCAL 结论

阶段三代码整改后的真实 eCAL 门禁仍未形成有效 PASS：最近一次获授权执行在当前 Codex 沙箱的 socket 边界前终止，`/tmp/pybullet-df-postfix-ecal-gate.CBZyMMKJ` 只能证明环境阻断，不能证明性能通过或失败。阶段四改变 wire schema 前必须把这项基线单独结案，避免把 v1 transport 未知风险混入 v2 调试。

- [x] **Step 1: 为主动转向 `4+2` 单条 invocation 取得明确授权**

说明车型、5 秒窗口、失败不自动重跑和 95Hz 下限；本次授权只覆盖紧随其后的第一条命令，不覆盖差速或阶段四后续真实 eCAL。

- [x] **Step 2: 扫描全机并确认静默**

确认没有其他 pytest、PyBullet GUI、临时 Xvfb、eCAL participant 或性能任务；保留长期空闲桌面 Xvfb，不终止用户进程。

- [x] **Step 3: 严格执行一次 `4+2`**

```bash
P0_ACTIVE_TIMESTAMP="$(date '+%Y%m%dT%H%M%S%z')"
P0_ACTIVE_EVIDENCE_DIR="results/stage4/p0-active-steering-4wd-retest-${P0_ACTIVE_TIMESTAMP}"
test ! -e "$P0_ACTIVE_EVIDENCE_DIR"
env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD \
  conda run -n slope-sim python scripts/verify_ecal_roundtrip.py \
  --runtime simulation \
  --robot-model active_steering_4wd \
  --warmup-sec 1 \
  --duration-sec 5 \
  --evidence-dir "$P0_ACTIVE_EVIDENCE_DIR"
```

Expected: rc=0，现有阶段三 100/10Hz、sim/wall、运动、日志、transport drop、窗口/fence 和 clean shutdown oracle 全部通过。失败则保留独立证据并停止；不得用阶段四代码掩盖。

执行记录（2026-08-01）：**FAIL，已按硬停止门中止且未重跑。** peer 完成 `new_command.ack` 后未能在 368 秒总预算内退出，verifier 返回 `TimeoutError: simulation peer did not exit`；证据见 `results/stage4/p0-active-steering-4wd-20260801T215112+0800.md`。

修复记录（2026-08-02）：以两轮 TDD 增加最终协议静默屏障；runtime 在写 `new_command.ack` 前有界等待异步发送 lane idle，并从下一轮起停止 physics/output publish、保留 discovery 与安全决策轮询。非真实 eCAL 回归 `140 passed, 4 deselected`，realtime/runtime 集成 `44 passed`；REFACTOR：无必要。真实状态仍为 FAIL，必须取得新的单条授权后复测，不能用本地 GREEN 改写。

复测记录（2026-08-03）：**FAIL，已按硬停止门中止且未重跑。** 完整协议、双方结果文件与 `stop.signal` 均形成，peer/runtime 都为 `clean_shutdown=true`，说明 2026-08-01 的资源关闭超时未复现；但正式窗口二进制日志含 499 个唯一 wheel-state publish，peer 仅收到 493 个且缺 6 个时间戳，逐帧交付 oracle 返回 `AssertionError`。证据见 `results/stage4/p0-active-steering-4wd-retest-20260803T100429+0800.md`；`2+0` 与 Task 2 继续阻断。

二次修复记录（2026-08-03）：证据将 6 帧拆为两个独立边界：1 帧由本地 `owner + latest` 非阻塞队列覆盖，其余 5 帧在 native `send=True` 之后、peer callback 之前消失；它们均出现在 2.1–3.2 ms 追赶 burst，与 eCAL 6.1.1 默认 `memfile_buffer_count=1` 的单 SHM memory file 争用一致。两轮 TDD 先确认 SHM multi-buffer 配置与只读 `is_idle()` physics gate 各自 RED，再以官方 `get_publisher_configuration()` 把 runtime/peer publisher 的 `memfile_buffer_count` 统一绑定到 `outgoing_queue_size`，并在上一帧 native send 未返回时仅跳过该轮 physics step。未改 FIFO、未放松逐帧 verifier，也未在物理线程调用阻塞 `wait_idle()`。新鲜 GREEN：`tests/test_ecal_transport.py` 与排除真实 eCAL 的 `tests/test_ecal_process_roundtrip.py` 合并回归 `224 passed, 4 deselected`；realtime/runtime/clock/report 扩大回归 `173 passed`；变更文件 `py_compile` 和 `git diff --check` 均为 rc=0。REFACTOR：无必要。这些只是本地修复证据，真实状态仍为 FAIL，须在新宿主预检后严格单次复测。

二次修复复测记录（2026-08-03）：**FAIL，已按硬停止门中止且未重跑。** peer 发送 500 条连续 WheelCommand，runtime 收到 499 条，catch-up burst 中的 `4930000000` ns 帧在 native send 之后、subscriber callback 之前消失，命令最大墙钟间隔 29.272 ms 超过 25 ms oracle。输出方向五话题 publish/receive 时间戳序列全部相等且零 drop/error，但 5 s 墙钟窗口只推进 `3.595833333` s 仿真时间，`sim/wall=0.7176079325`，证明全局 `is_idle()` gate 破坏 95 Hz 性能门。双方仍为 `clean_shutdown=true`，且结束后无残留进程。证据见 `results/stage4/p0-active-steering-4wd-retest-20260803T110410+0800.md`；`2+0` 与 Task 2 继续阻断。

附加误执行复测记录（2026-08-03）：**FAIL，且执行纪律违规。** 只读审查子任务越过任务边界，在没有新单条授权时误执行一次 `4+2`；主任务未自动重跑。peer 发送 500 条 command、runtime 收到 499 条，缺 `4940000000` ns；peer 出现 27.206 ms 停顿后以 54 us 间隔 catch-up，runtime 最大 gap 为 27.923 ms。全局 idle 门使 5.0053 s 墙钟只推进 855 步/3.5625 s，`sim/wall=0.7117458924`；五路输出数量逐项相等且 normal-load 零 drop/error，双方 clean shutdown。证据见 `results/stage4/p0-active-steering-4wd-retest-20260803T112140+0800.md`。该结果只能作为失败诊断，不能视为获授权验收；`2+0` 与 Task 2 继续阻断。

后续修复记录（2026-08-03）：三轮 TDD 分别锁定 lane 隔离、ACK 方向和 peer 回调开销。运行时以无副作用预览返回下一物理步的具体到期 topic，并只检查这些 topic 的 publisher lane；全局 `is_idle()` 仅保留给最终关闭屏障。simulation 五路输出继续使用 `ACK=0 + SHM ring`，只有 command publisher 使用由 100 ms 命令 watchdog 派生的 subscriber ACK。peer LiDAR 热回调改用生成 Protobuf 类读取 `timebase_ns`/descriptor，并以 O(1) 检查保留非空 `frame_id` 与 `point_num == len(points)` envelope 合同，不再遍历全部点构造 Python 领域对象；payload SHA-256 与 RTK 证据不变。每轮均先观察目标接口/断言 RED，再完成聚焦 GREEN；独立静态审查提出的 envelope P1 已用追加 RED/GREEN 清零，topic native in-flight 与传感器预览/提交一致性 P2 也已补测试。新鲜非真实 eCAL 主回归 `304 passed, 4 deselected`，runtime/lifecycle/clock/report 扩大回归 `204 passed`。REFACTOR 仅把 ACK role 所有权冻结到 transport 初始化；这些仍是本地证据，真实状态保持 FAIL，下一次 `4+2` 必须重新取得单条授权。

授权复测记录（2026-08-03 15:18）：**FAIL，已按硬停止门中止且未重跑。** 新宿主预检后严格执行用户授权的一次 `4+2`；peer 发送 500 条 command，runtime 收到全部 500 条，时间戳集合无缺失。五路输出 publish/receive 数量逐项相等且 normal-load 零 drop/error；1149 个物理步推进 4.7875 s，5.0056 s 墙钟下 `sim/wall=0.9564234947`，已恢复到 0.95 门槛以上；双方均 `clean_shutdown=true`，结束后无相关残留进程。剩余失败为 runtime command callback 最大间隙 30.616 ms 超过 25 ms，而 peer 最大发送间隙仅 20.729 ms；延迟 command 与 runtime front LiDAR 发布同窗，结合调用路径形成 runtime 侧 LiDAR/GIL 占用假设，但仍须 TDD 证实。证据见 `results/stage4/p0-active-steering-4wd-retest-20260803T151704+0800.md`；本次授权已消耗，`2+0` 与 Task 2 继续阻断。

- [x] **Step 3a: 用 TDD 修复 runtime callback 调度饥饿**

根因已收敛为同进程同步 LiDAR raycast、Python 点构造和 Protobuf 编码占用 GIL，导致 eCAL subscriber callback 在 25 ms 门槛外才获得执行机会；transport 锁、命令缺帧和 peer 发送节拍均不是本轮根因。先观察 `test_lidar_scan_hands_off_to_pending_command_before_encoding` 的 RED：front LiDAR publish 时 command 线程仍未完成；单次 `time.sleep(0)` 的集成 RED 仍失败，拆分 2880-ray native batch 的 DIRECT 对比也没有改善 heartbeat jitter，二者均未保留。GREEN 只在隔离的真实 eCAL simulation runtime 进入 240 Hz 循环前把 Python thread switch interval 收紧到 `min(调用方原值, 1 ms)`，并在外层 `finally` 精确恢复原值；不增加固定正延迟、不改变 PyBullet 单次逻辑扫描、wire、transport 或 verifier oracle。

本地 GREEN：聚焦 callback/接线/恢复测试 `4 passed`；`tests/test_ecal_transport.py` 与排除真实 eCAL 的 `tests/test_ecal_process_roundtrip.py` 为 `236 passed, 4 deselected`；runtime/lifecycle/realtime 扩大回归为 `254 passed`；双 LiDAR DIRECT 预算测试 `1 passed`；相关文件 `py_compile` 与 `git diff --check` 均 rc=0。REFACTOR：无必要。以上只证明候选修复与本地回归，不改变真实 P0 的 FAIL 状态。

- [x] **Step 3b: 为新的主动转向 `4+2` 单条 invocation 重新取得授权并即时预检**

授权只覆盖紧随其后的 Step 3c。执行前重新扫描 pytest、PyBullet、Xvfb、eCAL participant 和系统负载；若存在竞争负载则不消费授权，不得沿用任何历史授权。

- [x] **Step 3c: 严格执行新的唯一一次 `4+2`**

复用本节 Step 3 的正式命令，创建新的时间戳证据目录。Expected：rc=0，callback 最大间隙不超过 25 ms，且完整既有 oracle 全部通过；失败保留证据并停止，不自动重跑，也不进入 Step 4 或 Task 2。

执行记录（2026-08-03 16:16）：**FAIL，已按硬停止门中止且未重跑。** 新授权仅执行了一次 `active_steering_4wd`；预检时系统负载为 `0.12/0.55/0.49`，除长期空闲 `Xvfb :1` 外没有 pytest、PyBullet、verifier 或 eCAL participant。候选调度修复使 command 路径恢复为 peer 发送 500 条、runtime 接收 500 条，peer 最大发送间隙 `12.055 ms`、runtime command 最大接收间隙 `14.367 ms`；但 `/sim/wheel/state` 在 peer 接收端出现 `33.030 ms` 墙钟间隙，超过 `25 ms` oracle。该间隙对应连续的 `2800000000 -> 2810000000` ns wheel-state timestamp，故不是输出序列缺帧。其余正式窗口仍有 480 条 wheel-state、每个传感器 48 条，五路 output 零 transport drop/error，`1152` 步推进 `4.8 s` 仿真时间、`sim/wall=0.9566249594`，接口日志零 dropped、final pending 为 0，双方 clean shutdown；这不能抵消节拍失败。本次证据为 `results/stage4/p0-active-steering-4wd-retest-20260803T161620+0800/{peer-result.json,runtime-result.json}`，授权已消耗；不得重跑，不得执行 `2+0` 或进入 Task 2。

- [x] **Step 3d: 按补充设计用 TDD 实现异步 LiDAR worker**

用户已批准单个持久化 PyBullet DIRECT spawn worker、一个 in-flight 加一个 pending、冻结世界快照和预编码 payload。权威设计为 `docs/superpowers/specs/2026-08-03-stage4-p0-async-lidar-worker-design.md`，逐文件执行计划为 `docs/superpowers/plans/2026-08-03-stage4-p0-async-lidar-worker.md`。Task 0-13 已保存实际 RED/GREEN，通过 10 个本地五秒窗口与独立六维审查；不得将这些本地证据替代真实 eCAL。

- [x] **Step 3e: 本地门通过后重新取得并执行唯一一次 `4+2`**

2026-08-04 已形成主动转向 `4+2` 的有效真实 PASS。此前三次失败分别定位到 discovery 顺序、`log_start` 遗留引用和控制面/数据面就绪差异；data-plane preflight TDD 后，用户手动执行第四次获授权的唯一 `4+2` 得到 rc=0。正式窗口 command/wheel-state 均为 `500`、四传感器各 `50`，`sim/wall=0.998075`，20 障碍日志零丢失、wheel 原始日志/peer `500/500` 匹配且 clean shutdown。证据为 `results/stage4/p0-active-steering-4wd-retest-20260804T152217+0800/`；完整 RED/GREEN 记录见 P0 子计划。现可申请仅覆盖紧随其后差速 `2+0` 的单条授权，仍不得提前进入 Task 2。

- [x] **Step 4: 为差速 `2+0` 重新取得授权并重新预检**

在 `4+2` PASS 后取得用户独立授权；2026-08-04 15:31 预检负载为 `0.12/0.18/0.12`、可用内存约 `10 GiB`，无 pytest、PyBullet 或 eCAL 竞争进程。

- [x] **Step 5: 严格执行一次 `2+0`**

```bash
P0_DF_TIMESTAMP="$(date '+%Y%m%dT%H%M%S%z')"
P0_DF_EVIDENCE_DIR="results/stage4/p0-df-back-2wd-${P0_DF_TIMESTAMP}"
test ! -e "$P0_DF_EVIDENCE_DIR"
env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD \
  conda run -n slope-sim python scripts/verify_ecal_roundtrip.py \
  --runtime simulation \
  --robot-model df_back \
  --warmup-sec 1 \
  --duration-sec 5 \
  --evidence-dir "$P0_DF_EVIDENCE_DIR"
```

Expected: rc=0 且同一完整 oracle 通过；失败保留证据并停止，不自动重跑。

执行记录（2026-08-04 15:31）：**PASS。** `df_back` 真实命令 rc=0；command `500`、wheel `499` 且正式日志/peer `499/499` 匹配，四个传感器各 `50`，五 topic active；20 障碍、`sim/wall=0.998075`、窗口日志零 drop/末尾 pending 为零、双方 clean shutdown。start/end transport snapshot 的 error_count `132` 是数据面就绪前的预热累计，窗口差分为零且逐帧 oracle 通过。证据为 `results/stage4/p0-df-back-2wd-20260804T153125+0800/`；P0 两车型结案，Task 2 可以开始。

## Task 2：完成联网参考、依赖锁定与统一构建入口门

**Files:**
- Modify: `references/manifest.yml`
- Modify: `references/README.md`
- Modify: `scripts/sync_references.sh`
- Modify: `scripts/sync_references.py`
- Create: `packaging/locks/cpp-dependencies.lock`
- Create: `packaging/locks/ros2-dependencies.lock`
- Create: `packaging/locks/ubuntu24-system-dependencies.lock`
- Create: `packaging/python-environment.yml`
- Create: `packaging/python-toolchain-environment.yml`
- Create: `packaging/locks/virtual-packages.yml`
- Create: `packaging/locks/python.conda-lock.yml`
- Create: `packaging/locks/python-linux-64.lock`
- Create: `packaging/locks/python-toolchain.conda-lock.yml`
- Create: `packaging/locks/python-toolchain-linux-64.lock`
- Create: `packaging/locks/python-toolchain.lock`
- Create: `packaging/locks/python-package-cache.manifest.json`
- Create: `packaging/locks/python-wheel-cache.manifest.json`
- Create: `packaging/locks/source-archive-cache.manifest.json`
- Create: `packaging/build_dependencies.sh`
- Create: `packaging/build_ros_overlay.sh`
- Create: `packaging/build_python_runtime.sh`
- Create: `packaging/run_network_isolated.sh`
- Create: `scripts/freeze_python_lock_cache.py`
- Create: `scripts/verify_python_lock_cache.py`
- Create: `scripts/materialize_python_package_cache.py`
- Create: `scripts/materialize_python_package_channel.py`
- Create: `scripts/freeze_python_wheel_cache.py`
- Create: `scripts/verify_python_wheel_cache.py`
- Create: `scripts/materialize_python_wheel_cache.py`
- Create: `scripts/freeze_stage4_source_cache.py`
- Create: `scripts/verify_stage4_source_cache.py`
- Create: `scripts/stage4_source_archive.py`
- Create: `scripts/verify_stage4_dependencies.py`
- Test: `tests/test_sync_references.py`
- Test: `tests/stage4/test_reference_manifest.py`
- Test: `tests/stage4/test_stage4_dependencies.py`
- Test: `tests/stage4/test_python_offline_runtime.py`
- Test: `tests/stage4/test_network_isolation.py`

- [ ] **Step 1: 先复验已有 reference GREEN，再写依赖锁、builder 与探针 RED**

先原样运行现有 `tests/test_sync_references.py` 和真实 `scripts/sync_references.sh --check`；它们是本 Task 的已实现 preflight，必须保持 GREEN，不能删除生产实现来制造 RED。现有回归已经用临时真实 Git 仓库覆盖 branch 前移后仍固定旧 SHA、只读 check、dirty/index flag、checkout/repo-root/祖先和 Git metadata symlink、Git worktree/common-dir/object alternates、继承环境、local config 外部命令、replace metadata、ignored 内容与 partial-clone lazy fetch；不得 mock Git 或 `subprocess.run`。`test_reference_manifest.py` 只补阶段四 admission/真实 manifest 的静态合同，并复用生产 parser，不再复制一套同步器或重复发明 branch-move oracle。两个 Livox 条目还必须逐一验证 `third_party_license_files` 存在。阶段一至三的六个 legacy 条目继续按原 schema 同步，不借本阶段做无来源的元数据补写。

真正的新 RED 从尚未实现的依赖链开始。`test_stage4_dependencies.py` 使用本地 fixture archive 覆盖 SHA 篡改、`ref_kind/ref/commit` 不一致、annotated tag 未 peel、commit ref 不是 40 位 SHA、第二套 libprotobuf、Conda/RUNPATH 泄漏、Livox 任一链缺失和 ROS interface hash 漂移；再用 subprocess 调用尚未实现的 `scripts/verify_stage4_dependencies.py --locks-only`、`scripts/verify_stage4_source_cache.py`、`packaging/build_dependencies.sh` 与 `packaging/build_ros_overlay.sh`。源码 cache RED 还要覆盖 manifest/cache 缺失或多余归档、size/SHA-256 漂移、URL/basename/commit/ref/consumer/规范相对路径漂移、cache 文件链接或 `st_nlink != 1`、同 basename 不同 hash、两个构建共享可写归档副本或解包树，以及缺包时尝试联网 fallback。成员级独立 oracle 逐项覆盖绝对路径、空名/控制字符、`.`/`..` 规范化逃逸、重复路径、文件/目录冲突、多顶层根、device/FIFO/socket、所有 hardlink、绝对/逃逸/悬空/循环 symlink、member count/单文件/总展开大小超限和压缩膨胀；正例含普通文件、目录以及 Zstd 风格的根内相对 file/directory symlink，要求解包结果把链接安全深拷贝为零链接普通树且 digest 固定。正例只使用临时本地 archive，不访问公网。Livox ROS fixture 还要在临时 fake sysroot/default-search path 的 `usr/local/lib` 与 `usr/local/include` 放入错误版本 poison，断言 SDK 只安装到本轮私有 prefix、driver CMake cache 的 `LIVOX_LIDAR_SDK_LIBRARY/LIVOX_LIDAR_SDK_INCLUDE_DIR` 精确指向该 prefix 且链接结果不命中 poison；普通 pytest 不写真实 `/usr/local`。真实 D/E gate 只读比较实际 `/usr/local` 构建前后 census/hash 完全不变；全程禁止 sudo。缺入口时测试必须在测试函数内先用明确断言得到 `FAILED`，不能让 fixture setup、shell 解释器或 import collection 报错。

`test_python_offline_runtime.py` 只用临时本地 `file://` fake channel、最小 Conda 包和合成 wheel，不访问公网，也不读取开发机 Conda/Mamba/pip cache。RED 必须逐项覆盖：生产或 toolchain 人工 spec 缺失/混用、virtual-package spec 漂移、生产 unified lock 出现 `manager: pip`、explicit lock 出现 `# pip`、缺 MD5 或 SHA-256、unified/explicit render 漂移；canonical package artifact 缺包、多包、同 URL/归档重复、hash 篡改、符号/硬链接、普通文件 `st_nlink != 1`、被扁平化或 host/channel/subdir 层级错误；直接把嵌套 artifact 当作 micromamba cache、native cache 缺/多归档、未按 manifest 物化、原 URL `urls.txt` 漂移，以及同 basename 但 size/MD5/SHA-256 不同的碰撞；Python wheel artifact 缺失/多余/篡改、错误 distribution/version、非 `cp310-cp310-manylinux_2_28_x86_64` tag、错误 RECORD、缺完整 license/NOTICE、ELF/DSO inventory 漂移或出现 bundled `libprotobuf.so`；`.condarc`、`MAMBARC`、`CONDA_PKGS_DIRS`、pip index/proxy 和 channel 环境变量污染；只传 micromamba `--offline` 却没有外部断网 namespace/VM；A/B 两轮共用可写 native package cache、wheel 副本或环境；builder/cache/source 绝对路径经 `conda-meta/history`、pip `direct_url.json`、console script、陈旧 wheel `RECORD`、`.pyc` 或 `conda-unpack` prefix records 泄漏；在 `conda-pack` 前删除 Conda 管理的 `.pyc`/`__pycache__` 或安装任一 wheel；以及 Python 进程同时加载 wheel 私有 eCAL 与 release `root/lib` eCAL、C++ 进程反向加载 wheel eCAL、任一进程加载两套 eCAL 或 libprotobuf。缺入口时同样只能得到测试函数内的明确 `FAILED`。

`test_network_isolation.py` 先直接/伪造环境变量调用每个 Python/C++/ROS builder，要求它们在 configure/create 前失败；环境 token 或 JSON 文件不能替代进程内核状态。正例由 `run_network_isolated.sh` 创建新的 user+network namespace，builder 自己读取 `/proc/self/ns/net`、`/proc/<wrapper-parent>/ns/net`、`/proc/net/route`、`/proc/net/ipv6_route` 和结构化 link/route 状态，证明 child netns inode 与 wrapper parent 不同、只有 loopback 可用、没有 IPv4/IPv6 default route 或非 loopback interface，同时用本地 TCP socket 证明 loopback 保留，并断言对 TEST-NET IPv4/IPv6 的 connect 返回 `ENETUNREACH`。wrapper 把上述直接观测值、父子 PID/inode、argv digest 和时间写入 work root 外只读 attestation；builder 在真正动作前重新观测并要求相同，不接受可由调用者单独伪造的“已隔离”标志。测试覆盖 `unshare`/loopback/route 检查失败、同 netns、伪造 parent PID/inode、添加默认路由、非 loopback interface、attestation/argv digest 漂移和 wrapper 内嵌套逃逸；无法建立该合同即 fail closed，不降级到 `--offline`。等价断网 VM 也必须提供同字段的进程内直接观测，不存在 `--skip-network-isolation`。

同一 RED 还要锁定 ROS build context：`verify_stage4_dependencies.py --write-ros-build-context` 只接受成功构建后仍可复核的全新绝对 `run/source-work/livox-sdk/build/install` 路径、枚举 `dependencies|bridge_red|bridge_green` kind 和可选的已验证 dependency parent context；`dependencies` context 还必须以 repeatable `--ros-interface-file <type>=<absolute-path>` 读取本轮断网 `ros2 interface show` 的两个输出，按 lock 的规范化算法复算并精确比较 `livox_ros_driver2/msg/CustomMsg` 与 `CustomPoint` hash。全部通过后才以 paired JSON evidence 加受限 shell serializer 原子替换调用者指定的稳定 context 文件。`--verify-ros-build-context` 必须复算 lock/cache/materialized tree、interface type/hash、only-loopback attestation、`/usr/local` pre/post census、SDK/CMake/link/ELF/install tree digest 和 parent context hash 后才允许 source；调用者提供新的 interface 文件时还要再次按同一结构化 lock 比较。fixture 先让旧的固定输出目录在第二轮因非空失败，再要求两次调用各自使用不同 run root 且第二个 context 原子指向新根；另参数化拒绝路径复用/互相包含、错误 kind、缺/多/重复/未知 interface type、只打印未比较的 SHA、interface bytes/hash 漂移、缺/错 parent、context/evidence/path/tree hash 篡改、换根、shell 注入、失败写入覆盖最后一份有效 context，以及未通过 builder 就伪造 context。RED 的预期行为失败和 GREEN 后的 REFACTOR 复验都必须重新建根，不能把清空旧目录当作修复。

已有 `tests/test_sync_references.py::test_sync_checks_out_pinned_commit_after_branch_moves` 是该行为的唯一生产入口回归；本 Task 只消费它，不另建同义测试 helper。

- [ ] **Step 2: 运行已有 GREEN，再确认新依赖 RED 边界**

Preflight：

```bash
conda run -n slope-sim python -m pytest -q tests/test_sync_references.py
bash scripts/sync_references.sh --check
```

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_reference_manifest.py tests/stage4/test_stage4_dependencies.py tests/stage4/test_python_offline_runtime.py tests/stage4/test_network_isolation.py`

Expected: preflight 全部通过；随后阶段四 pytest 正常收集并 `FAILED`，`test_reference_manifest.py` 应已通过，失败断言只指向依赖锁、canonical Python/source archive cache、断网 builder 或探针尚不存在；不得把已有 manifest 同步器当作缺失功能，也不得是 collection error、真实网络错误、缺编译器/ROS 或 skip。

- [ ] **Step 3: 用官方一手来源复核已固定 reference**

先运行 `node /home/cancade/.agents/skills/web-access/scripts/check-deps.mjs`，要求输出含 `proxy: ready`。阶段四只纳入 `eclipse-ecal/ecal`、`protocolbuffers/protobuf`、`Livox-SDK/livox_ros_driver2`、`Livox-SDK/Livox-SDK2`、`foxglove/mcap`、`facebook/zstd`、`PointCloudLibrary/pcl`；逐项复核官方 URL、精确 branch ref、40 位 commit、同 commit 的全部 `license_files`/`third_party_license_files`、官方 Star 数/观测时间和 focus 路径。`license_scope: first_party` 不得提升为整个源码归档的许可证；Zstd 的双许可证表达必须同时核对 `LICENSE` 与 `COPYING`。`ros2/rosbag2` 与 `ros2/rviz` 保持“已评估、不克隆源码”，但 Livox Driver 2 声明的系统 `rosbag2` 依赖仍必须进入 ROS lock；源码诊断只能固定 `jazzy`，不能把默认 `rolling` 混入 Jazzy 交付。

七个 `stage: 4` 条目的完整元数据已在 manifest 中冻结；同步工具只消费并校验该 manifest，不负责联网解析或写入记录。Step 3 使用下列只读解析逻辑复核 branch 与 commit，不回写 manifest，也不接受人工占位值：

```python
def resolve_reference(url: str, branch: str) -> str:
    expected_ref = f"refs/heads/{branch}"
    output = subprocess.run(
        ["git", "ls-remote", "--exit-code", "--heads", url, expected_ref],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    if len(output) != 1:
        raise RuntimeError(f"cannot resolve exact branch ref {expected_ref} for {url}")
    commit, actual_ref = output[0].split()
    if actual_ref != expected_ref or re.fullmatch(r"[0-9a-f]{40}", commit) is None:
        raise RuntimeError(f"remote returned an invalid branch record for {url}")
    return commit
```

冻结的阶段四 YAML 必须同时含 stage/name/url/branch/具体 commit/SPDX license/license_files/stars/stars_observed_at/purpose/focus，并带第一方 `license_scope` 和非空 `license_files`；只有包含第三方代码的条目才要求非空 `third_party_license_files`，多许可证列出全部证据文件。测试用同一 40 位 SHA、非空 license file list、实际文件存在性和非负 Star 规则阻止任何占位文本进入仓库；legacy 条目不在本阶段被强行改写成未经复核的新 schema。

- [ ] **Step 4: 复核并冻结已有 manifest 同步入口**

确认 `sync_references.sh` 只委托同一个 Python/YAML 入口，真实 manifest 的 13 个 checkout 均固定 SHA 且 `--check` 不 fetch。保留现有 fail-closed 边界：路径与 Git metadata 不跟随 symlink，Git 解析出的 worktree/git-dir/common-dir 精确位于 checkout，禁止 alternates、replace/grafts、未准入 local config、ignored 本地内容和 partial-clone lazy fetch，并在任何 Git 写操作前验证。若阶段四 admission schema 需要改动，必须先在 `tests/test_sync_references.py` 或 `test_reference_manifest.py` 写对应 RED 并观察正确失败；若无需改动，记录“GREEN preflight：无需生产修改”，不得为了匹配 `Create` 清单重写已通过实现。

- [ ] **Step 5: 从目标发布 tag 生成源码依赖锁与 canonical archive cache**

阅读快照不能充当构建锁。`cpp-dependencies.lock` 固定 eCAL `v6.1.1`/`bf0bc5734dd31c6315ebad907c92c2bb1edc1851`、Protobuf `v33.6`/`6e1998413a5bca7c058b85999667893f167434bc`、MCAP C++ `releases/cpp/v1.4.0`/`9e7838c3ea51336d84141a80e2ffb15c589d2f54`、Zstd `v1.5.7`/`f8745da6ff1ad1e7bab384bd1f9d742439278e99`；PCL `pcl-1.14.0`/`f62c018b4fc7df3dc2c096918a8462a190f28bb8` 标为 `validation_only`，只在独立验证前缀生成 `pcl_pcd2ply`。`ros2-dependencies.lock` 固定同一次 Jazzy 构建验证使用的 `livox_ros_driver2` `13eb05e4e6dd7a765b934d0c5fd6236676a57b49` 与 `Livox-SDK2` `68ae1e1dc77f61f03c95d7c2809831e198d0aedd`，并保存两个归档的全部第三方 notice 路径，不能用一个 MIT 字段概括完整归档。

每项仍须写官方 URL、`ref_kind: tag | commit`、不可变 `ref`、40 位 `commit`、现场计算的源码归档 SHA-256、SPDX/license files 和构建选项。`ref_kind: tag` 时 `ref` 必须是完整 release tag，联网 producer 用远端 peeled tag 证明其最终 commit 精确等于 `commit`；`ref_kind: commit` 时 `ref` 必须本身就是与 `commit` 相等的 40 位 SHA，archive URL 也必须以该 SHA 标识，禁止 branch、短 SHA 或伪造 tag。eCAL、Protobuf、MCAP、Zstd、PCL 和 `livox_ros_driver2` 使用各自已冻结 release tag；没有指向该提交的官方 tag 的 Livox-SDK2 使用 `ref_kind: commit`、`ref=commit=68ae1e1dc77f61f03c95d7c2809831e198d0aedd`。`ros2-dependencies.lock` 还保存规范化 `ros2 interface show` hash，以及从固定 package manifest 解出的完整 ROS/系统构建与运行依赖。`ubuntu24-system-dependencies.lock` 冻结 builder image digest、经测试的 Ubuntu 24.04 apt package 名/版本和允许的基础 DSO SONAME。完整官方 Livox 链无法构建即停止 ROS Bridge 子计划。

锁生成器从 `ResolvedReference` 写出具体值，并在落盘前执行：

```python
if re.fullmatch(r"[0-9a-f]{40}", reference.commit) is None:
    raise ValueError(f"unresolved commit for {reference.name}")
if re.fullmatch(r"[0-9a-f]{64}", source_archive_sha256) is None:
    raise ValueError(f"unresolved archive checksum for {reference.name}")
```

测试拒绝锁文件中的尖括号占位、未决标记、默认分支名、未解引用 annotated tag、`ref_kind` 与 `ref` 形态不匹配、commit ref 与 `commit` 不相等或非具体 checksum。

同一获授权联网 producer 随后运行 `freeze_stage4_source_cache.py`，只下载两份源码 lock 精确列出的 eCAL、Protobuf、MCAP、Zstd、PCL、Livox-SDK2 和 `livox_ros_driver2` 归档并输出只读 canonical artifact；D/E 不得从 reference checkout 或 Git 工作树临时重新打包。`source-archive-cache.manifest.json` 对每条记录保存 normalized HTTPS URL、经 URL parser 校验的 basename、`ref_kind`、不可变 `ref`、40 位 commit、archive format、size、SHA-256、`archives/<sha256>/<basename>` 规范相对路径、非空 consumer 集（`cpp_dependency|validation|ros_overlay`）、唯一顶层目录，以及冻结的 archive member count/regular bytes/symlink count、零链接 materialized member count/bytes/tree SHA-256，并保存完整 artifact tree digest。manifest 必须与对应 lock 一一映射；cache 只允许这些普通 archive 文件，拒绝额外成员、cache 层符号/硬链接、普通文件 `st_nlink != 1`、路径逃逸和重复记录。

`stage4_source_archive.py` 是 freeze/verifier/两个 builder 共用的唯一结构化成员 parser，不调用 `tar --extract` 或无过滤 `extractall`。它先完整预检全部 header：member 名必须是唯一顶层目录下的规范相对路径，拒绝绝对路径、空名/NUL/控制字符、`.`/`..`、规范化重复、文件/目录冲突、特殊节点和所有 hardlink；先求和并要求 member count、单文件/总 regular bytes、symlink count 与 manifest 精确相等且不超过固定全局 ceiling，任何一项越界都在创建输出前失败。相对 symlink 只有在冻结 member graph 中逐跳保持在同一顶层根、最终指向已声明普通文件/目录且无悬空/循环时才允许；materializer 用 dirfd/no-follow/exclusive create 把 file link 复制成普通文件、把 directory link 按排序递归深拷贝为普通目录树，限制展开后的 member/byte 数并与 manifest 的 materialized digest 精确比较。所有输出 mode 规范为 `0644/0755`，拒绝 setuid/setgid/sticky，最终递归 `lstat` 要求只有目录/普通文件且每个文件 `st_nlink == 1`。这条正例专门覆盖锁定 Zstd 源码归档中的真实根内相对 symlink，不能通过一律拒绝 symlink 伪造安全。

`verify_stage4_source_cache.py` 使用结构化 JSON/lock parser 逐项 `lstat` 并复算 archive size/SHA-256/artifact tree digest，再调用上述 parser 只读预检每个归档的成员图和 materialized tree digest，不靠 basename 或文本 grep 猜归档。两个 lock 若出现相同 basename 但 bytes/hash 不同必须在任何构建前失败；相同 bytes 只允许以同一 manifest 记录复用。`build_dependencies.sh`、`build_ros_overlay.sh` 和 E 的 `build_release.sh` 都只读消费同一 canonical root，但每轮先按 consumer 精确复制到本轮私有 `$SOURCE_WORK/archives`，再通过该 parser 安全物化到本轮私有 `$SOURCE_WORK/trees`；exclusive create 前后都复算 hash，禁止共享可写副本/解包树、直接在 canonical root 解包、shell extractor 或缺包联网 fallback。联网 freeze 与离线 verify/materialize 的 producer evidence都绑定 archive/member/materialized/artifact digest。

- [ ] **Step 6: 冻结生产 Python 锁、工具链和 canonical package/wheel cache**

`packaging/python-environment.yml` 是纯 Conda 生产 Python runtime 的唯一人工声明；`packaging/python-toolchain-environment.yml` 只声明构建机使用的 `build/pip/conda-pack` 工具环境；二者共用唯一冻结的 `packaging/locks/virtual-packages.yml`，不得从 producer 主机即时探测 virtual packages。项目 wheel 和官方 eCAL wheel 都在 conda-pack 后安装，不以 `pip:` 条目混入环境求解；生产 spec 必须由 Conda 固定满足 eCAL wheel 声明的 `packaging` 与 `protobuf<7,>3.8`，其中 Protobuf 精确为 `6.33.6`。联网 producer 只能使用固定工具链生成以下成对产物：`python.conda-lock.yml` 保存 linux-64 unified lock，`python-linux-64.lock` 保存 explicit lock；`python-toolchain.conda-lock.yml` 与 `python-toolchain-linux-64.lock` 单独冻结 tool env。生产锁中所有包的 manager 必须为 `conda`，explicit 文件不得出现 `# pip`；conda-lock 的 pip 安装分支不提供等价 `--no-index` 硬门，因此不能用于本离线生产链。conda-lock explicit renderer 只写 MD5 URL fragment，所以 verifier 必须把每个 explicit URL 与 unified 记录一一对应，并同时校验 unified 中的 MD5、SHA-256 和实际 archive bytes；任何缺 hash、重复或 render 漂移都失败。

`freeze_python_lock_cache.py` 必须以结构化 argv 执行下面四条唯一 lock/render 命令；`$LOCK_ENV` 是已经由 `python-toolchain.lock` 验证过的 producer tool env，`$PINNED_MICROMAMBA` 是已复算 SHA-256 的绝对 binary。任何命令、平台、virtual spec 或输出名变化都要回到 RED：

```bash
"$LOCK_ENV/bin/conda-lock" lock \
  --conda "$PINNED_MICROMAMBA" --no-mamba --no-micromamba \
  --file packaging/python-environment.yml \
  --platform linux-64 --kind lock \
  --lockfile packaging/locks/python.conda-lock.yml \
  --virtual-package-spec packaging/locks/virtual-packages.yml \
  --no-dev-dependencies
"$LOCK_ENV/bin/conda-lock" render \
  --kind explicit --platform linux-64 --no-dev-dependencies \
  --filename-template "packaging/locks/python-{platform}.lock" \
  packaging/locks/python.conda-lock.yml
"$LOCK_ENV/bin/conda-lock" lock \
  --conda "$PINNED_MICROMAMBA" --no-mamba --no-micromamba \
  --file packaging/python-toolchain-environment.yml \
  --platform linux-64 --kind lock \
  --lockfile packaging/locks/python-toolchain.conda-lock.yml \
  --virtual-package-spec packaging/locks/virtual-packages.yml \
  --no-dev-dependencies
"$LOCK_ENV/bin/conda-lock" render \
  --kind explicit --platform linux-64 --no-dev-dependencies \
  --filename-template "packaging/locks/python-toolchain-{platform}.lock" \
  packaging/locks/python-toolchain.conda-lock.yml
```

`python-toolchain.lock` 分开记录源码 tag provenance 与安装制品 hash，不能把 Git commit 当二进制/package checksum：

- micromamba 固定 `2.8.1-1`，上游 mamba `2.8.1` tag commit 为 `0abc611db8b7bc92bfb7841158c713d0d028bedb`，Linux amd64 二进制 SHA-256 为 `77b7790ec97f64581118f103585b175df4306f95829b0fa6bfe4a19cc88a1182`；
- conda-lock 固定 `4.0.2`，tag commit 为 `e29d5cf7dcb826b07ba1696883426494b4d96d66`；
- conda-pack 固定 `0.9.2`，tag commit 为 `3efad58976f33eff3ef21c2882e9cd7458720af5`。

conda-lock/conda-pack 实际安装包的 URL、filename、MD5 和 SHA-256 必须取自 toolchain unified lock 与 cache manifest 的具体记录，不能从 Git tag 推导，也不能留下占位值。`freeze_python_lock_cache.py` 只在本 Task 的联网 producer 运行；它先校验上述工具链，再生成/渲染锁，并在全新 `$SEED` 下分别执行下面两条联网 `--download-only` 命令，下载两份 explicit lock 的精确并集。两个 download prefix 只能共享本次 seed root，不能读取用户 cache：

```bash
"$PINNED_MICROMAMBA" create \
  --no-rc --no-env --root-prefix "$SEED/mamba-root" \
  --prefix "$SEED/runtime-download-prefix" \
  --file packaging/locks/python-linux-64.lock \
  --download-only --safety-checks enabled --yes
"$PINNED_MICROMAMBA" create \
  --no-rc --no-env --root-prefix "$SEED/mamba-root" \
  --prefix "$SEED/toolchain-download-prefix" \
  --file packaging/locks/python-toolchain-linux-64.lock \
  --download-only --safety-checks enabled --yes
```

脚本随后输出只读 canonical cache artifact 和 `python-package-cache.manifest.json`。artifact 的唯一允许布局为规范排序且自身有 SHA-256 记录的 `pkgs/urls.txt`，以及 `pkgs/https/<host>/<channel>/<subdir>/<archive>`；这是一种可审计、可分发的规范输入布局，不是 micromamba 的原生 package cache。artifact 不接受扁平 fallback、开发机 `~/.conda/pkgs`、额外 repodata/package、重复 URL、链接或 manifest 外文件。manifest 对每个 archive 记录 normalized URL、经过 URL parser 验证的 archive basename、canonical relative path、size、MD5、SHA-256 和所属 lock，目录树 digest 写入阶段证据。

PyPI 官方 6.1.1 发布没有可供这条生产链使用的 Conda 制品，因此同一获授权联网 producer 另以 `freeze_python_wheel_cache.py` 冻结唯一 `eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl`：官方 normalized HTTPS URL、filename、distribution/version、`Requires-Python`、PEP 425 tag、size `6905517` 和 SHA-256 `57a23af7d83c077c04f01852db13f8cda7686a052d41659fafcbe6b3dbe9f6bc` 必须来自 PyPI release JSON 与实际 bytes 的交叉验证。`python-wheel-cache.manifest.json` 还冻结 wheel member count/tree digest、`METADATA/WHEEL/RECORD` digest、RECORD 对每个成员的 hash/size、Apache-2.0 expression、全部 `License-File`/NOTICE 路径及 digest，以及所有 ELF 的 path/class/machine/SONAME/NEEDED/RUNPATH/content SHA-256；已观测 wheel 私有提供 `ecal/libecal_core.so.6` 和 HDF5，禁止出现 bundled `libprotobuf.so`。artifact 只允许 `wheels/<sha256>/<filename>` 这一个普通文件，拒绝额外成员、链接或 `st_nlink != 1`。`verify_python_wheel_cache.py` 用结构化 ZIP/metadata/ELF parser 逐项复算，不执行 wheel 内容；错误 ABI/tag、路径逃逸/重复成员、RECORD/license/ELF 漂移都在 pip 前失败。冻结脚本是唯一联网 producer，D/E 禁止从 pip index 补包。

`verify_python_lock_cache.py` 必须用结构化 YAML/JSON 和 URL parser 校验 environment spec、两组 unified/explicit lock、toolchain provenance、micromamba binary 与 canonical package artifact 一一对应关系；`verify_python_wheel_cache.py` 独立校验官方 eCAL wheel artifact，二者都不得靠文本 grep 猜 package。通过后，`build_python_runtime.sh` 在每轮全新空 `$WORK/mamba-root/pkgs` 中做唯一的 deterministic native materialization：按 manifest 排序逐项重新 `lstat` 并复算 size/MD5/SHA-256，再以 exclusive create 把嵌套 artifact 的 archive bytes 复制成 cache 根级 `$WORK/mamba-root/pkgs/<archive-basename>` 普通文件，最后写入规范排序的原 normalized URL `urls.txt`；eCAL wheel 也必须 exclusive-copy 到本轮私有 `$WORK/wheel-cache/` 并在 pip 前再次复算。同 basename 记录只有在 size、MD5 和 SHA-256 全部相同、归档只复制一次时才允许跨两份 lock 复用；任一不同立即失败。create 前 native cache 只允许 manifest 指定的根级 archive 与 `urls.txt`，物化映射证据写在 cache 外。A/B 不共享可写 native cache、wheel 副本、root prefix、tool env 或 runtime env，也不得把 canonical 嵌套目录原样塞进原生 cache。创建 runtime 的命令固定为：

```bash
"$MICROMAMBA" create \
  --no-rc --no-env \
  --root-prefix "$WORK/mamba-root" \
  --prefix "$WORK/python-builder" \
  --file "$SOURCE/packaging/locks/python-linux-64.lock" \
  --offline --always-copy --safety-checks enabled --yes
```

tool env 以相同参数和 `python-toolchain-linux-64.lock` 创建到 `$WORK/tool-env`。两条命令都必须由 `run_network_isolated.sh` 放进新的 user+network namespace，或由构建 VM 提供完全相同字段的进程内硬断网证明；micromamba `--offline` 只约束 repodata/cache 使用，缺包时仍可能构造下载请求，不能单独充当断网门。wrapper 保留并验证 loopback，移除全部 IPv4/IPv6 default route 和非 loopback interface；记录 wrapper parent/child 的 netns inode、PID、argv digest、结构化 link/route、TEST-NET connect=`ENETUNREACH` 和 loopback socket成功证据。每个 builder 在 create/configure 前独立读取 `/proc` 与 link/route 重算这些字段，必须看到 child 与仍存活 wrapper parent 的 netns inode 不同；单独伪造 env token/attestation、相同 netns、默认路由、非 loopback interface、无法读取 parent 或证据漂移都 fail closed。wrapper 无法建立/验证隔离时必须在任何输出前失败，不能降级；等价 VM 也没有 skip 开关。builder 使用专用空 HOME，并显式清除 Conda/Mamba/pip channel/index/proxy/cache 环境变量；`--no-rc --no-env` 仍为不可删除的双保险。

`build_python_runtime.sh` 在本 Task 就必须完整冻结打包顺序，E 只调用，不能复制或分叉这套流程。它用 tool env 在固定 locale/timezone、`SOURCE_DATE_EPOCH` 和 `--no-isolation` 下构建唯一项目 wheel，但在 conda-pack 前绝不安装官方 eCAL wheel 或项目 wheel，也不删除任何 Conda 管理文件或 `.pyc/__pycache__`。它保留本轮完整 package cache，对纯 Conda runtime 执行：

```bash
"$WORK/tool-env/bin/conda-pack" \
  --prefix "$WORK/python-builder" \
  --output "$WORK/python-pack/python-runtime.tar" \
  --format tar --n-threads 1
```

只有 pack 成功后才解到 `$WORK/root/runtime/python`，由同一 Python 3.10 ABI/sysconfig layout 的 tool env pip 分两次以 `--no-deps --no-index --no-compile --prefix "$WORK/root/runtime/python"` 先安装本轮私有且已复核的官方 eCAL wheel，再安装本轮唯一项目 wheel；argv 中只能出现这两个绝对本地 wheel 路径，环境必须清空 index/find-links/proxy，禁止依赖解析或联网 fallback。安装后复算 eCAL 与项目两个 dist-info `RECORD`，eCAL license/NOTICE 和 ELF inventory 必须仍与冻结 manifest 一致；若生成 `direct_url.json` 则删除，按项目 entry-point metadata 精确移除项目 console scripts，再确定性重算对应 `RECORD`。随后只在 staging 删除 `conda-meta/history` 和全部 `.pyc/__pycache__`，规范 mode/mtime并扫描 source/work/builder/cache prefix。原 staging tree 不运行 `conda-unpack`，只允许随机副本 smoke。除 `history` 外保留 conda-meta package records，且 raw conda-pack tar 只作中间物，不作为双根 byte-identical 判据。

fake channel GREEN 必须使用精确 hash 的 pinned micromamba，在外部断网下从嵌套 canonical package artifact 和 canonical eCAL wheel artifact 分别物化两轮私有 native flat cache/wheel 副本，再完成 tool/runtime explicit create、纯 Conda pack、两 wheel staging 安装、清理和随机副本 relocation smoke；正例必须真实消费根级 Conda archive 和精确 eCAL wheel，反例证明直接复制嵌套 tree、缺包/缺 wheel、hash/RECORD/license/ABI/ELF 篡改和 basename hash 碰撞都会在 create/pip 前失败。证据还要证明 rc/env/index/proxy 注入无效、缺包不会联网、A/B cache/wheel/env 互不引用、两套规范 runtime tree 逐成员相同，以及 pack 发生在任一 wheel 安装和 managed pyc 清理之前。随机 smoke 只启动专用 Python/C++ no-participant loader：Python 只 import/`dlopen` wheel 内 eCAL core，C++ 只 `dlopen(..., RTLD_NOW)` release `root/lib` 内 eCAL core，二者都不得调用 Initialize/monitoring/pub/sub/entity API；用调用审计、前后零 entity census、`/proc/<pid>/maps`、`readelf` 和 `ldd` 证明没有新增 participant/topic，Python 只加载 `runtime/python/.../ecal/libecal_core.so.6`、C++ 只加载 release `root/lib/libecal_core.so.6`，每个进程恰有一套 eCAL 且没有第二套 `libprotobuf.so`。Python launcher 不向进程注入 root `lib`，C++ launcher 不向进程注入 Python site-packages；这项构建 smoke 不算真实 eCAL invocation，也不能写成生产通信通过。E 只消费这套已 GREEN 的 builder、锁和两个 canonical Python artifact，并把真实项目源码/snapshot 作为输入，不得反向修改或内联实现。

- [ ] **Step 7: 实现统一离线构建入口并构建开发 C++ 依赖前缀**

`packaging/build_dependencies.sh` 和 `packaging/build_ros_overlay.sh` 都必须先规范化并验证调用者显式提供的绝对 `--source-archive-cache/--source-work`、build 和 install 路径；ROS 入口还必须接受独立 sibling `--livox-sdk-prefix`。canonical cache 必须只读且匹配 `source-archive-cache.manifest.json`；source work、Livox SDK prefix、build、install、client 与 validation 输出必须全新为空、互不相同、互不包含，并且不得包含 canonical cache 或被其包含。入口只允许在 `run_network_isolated.sh` 已验证的无外网 namespace/VM 内运行，并在任何 configure 前复核隔离证明；按 consumer 从 canonical root exclusive-copy 精确归档到本轮私有 `archives/`，复算 hash 后解到私有 `trees/`，拒绝 lock/source SHA 漂移、共享可写目录和任何运行时联网下载。所有 C/C++ configure/build 固定同一 `SOURCE_DATE_EPOCH`，并用 `-ffile-prefix-map/-fdebug-prefix-map/-fmacro-prefix-map` 把 source/build 根映射到稳定虚拟路径；安装清单、ELF 和 CMake/pkg-config 文件不得泄漏实际根。

ROS builder 成功后由同一探针以 `--ros-context-kind`、五个显式路径、dependency context 的两份 `--ros-interface-file` 或 Bridge 的 `--parent-ros-context` 写 `--write-ros-build-context`；builder 本身不得提前写成功 context。context 固定导出 `STAGE4_ROS_CONTEXT_KIND`、`STAGE4_ROS_RUN_ROOT`、`STAGE4_ROS_SOURCE_WORK`、`STAGE4_ROS_LIVOX_SDK_PREFIX`、`STAGE4_ROS_BUILD_BASE`、`STAGE4_ROS_INSTALL_PREFIX`，Bridge context 还导出已绑定的 `STAGE4_ROS_PARENT_INSTALL_PREFIX`。稳定 context 文件位于 run root 外，paired evidence 位于该轮唯一 run root 内；探针先完整验证并 fsync evidence，再以临时文件+fsync+rename 原子替换 context，因此 interface/lock 或任何后续门失败的轮次保留上一份有效定位。D 的每次 RED 修壳重跑、GREEN 和 REFACTOR 复验都必须用新的 `mktemp` run root，后续 shell 先 `--verify-ros-build-context --expect-ros-context-kind ...` 再 source，不能依赖固定 build/install 路径或上一 shell 变量。

ROS 入口固定先把 Livox-SDK2 构建、安装到调用者显式提供的本轮全新空 `--livox-sdk-prefix`（例如 `$ROS_RUN_ROOT/livox-sdk-install`）；该 prefix 是 `--source-work`、build 和 install 的 sibling，绝不能位于 `$SOURCE_WORK` 内。入口显式传 `-DCMAKE_INSTALL_PREFIX`，禁止 sudo、禁止默认安装或写入 `/usr/local`。再构建完整 `livox_ros_driver2` 时，预置 `LIVOX_LIDAR_SDK_LIBRARY` 和 `LIVOX_LIDAR_SDK_INCLUDE_DIR` 为该私有 prefix 的精确库/头文件，审计 CMakeCache、link command、`readelf` 和 `ldd` 都只命中本轮树；构建前后对 `/usr/local/lib` 与 `/usr/local/include` 做排序 `lstat`/hash census 并要求不变。传 `--project-source` 时才把本项目 ROS packages 加入同一 merge-install overlay，并要求 `--client-prefix` 指向已验证的 C++ 安装树。fixture RED/GREEN 只使用本地最小 CMake/ament archives，并参数化拒绝 SDK prefix 与 source/build/install/client/cache 相同或互相包含、把 SDK prefix 放回 source work，以及跨两轮复用非空 SDK prefix；这些反例必须在复制归档或 configure 前零输出。再以 `/usr/local` 错版本 poison 证明不会 fallback；真实 Jazzy/Livox 构建留到 D 的硬门。不得为测试增加跳过 hash、跳过依赖顺序、跳过私有 SDK 定位或联网 fallback。

```bash
install -d "$PWD/build" "$PWD/results/stage4"
test -d "$STAGE4_SOURCE_ARCHIVE_CACHE"
STAGE4_CPP_SOURCE_WORK="$(mktemp -d "$PWD/build/stage4-sources.XXXXXX")"
STAGE4_CPP_BUILD_ROOT="$(mktemp -d "$PWD/build/stage4-deps-build.XXXXXX")"
bash packaging/run_network_isolated.sh bash packaging/build_dependencies.sh \
  --lock packaging/locks/cpp-dependencies.lock \
  --source-cache-manifest packaging/locks/source-archive-cache.manifest.json \
  --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE" \
  --source-work "$STAGE4_CPP_SOURCE_WORK" \
  --build-root "$STAGE4_CPP_BUILD_ROOT" \
  --prefix "$PWD/build/stage4-deps" \
  --validation-prefix "$PWD/build/stage4-validation-tools"
```

父目录必须由上面的显式 `install -d` 在首次 `mktemp` 或 evidence 写入前创建；不能依赖历史 build/results 残留。脚本先验证每个源码归档 SHA-256，再以 GCC 13/CMake 3.28 构建并安装独立 `protoc/libprotobuf 33.6`、eCAL 6.1.1 raw C++ SDK、MCAP 和 Zstd 到 `build/stage4-deps`，并把锁定 PCL 的验证 CLI 单独装到 `build/stage4-validation-tools`。A/C 和开发 D 只读消费该开发前缀，D 的第三方格式 smoke 只消费 validation prefix 的绝对路径；不得各自 FetchContent、使用 Conda 私有 C++ 库或临时下载另一版本。E 的每个 stage/final work root 必须再次从同一只读 source artifact 调用这个入口，分别创建私有 `cpp-sources`、`cpp-deps-build`、`cpp-deps-install` 和 `validation-prefix`，并只把本轮 `cpp-deps-install` 设为 `STAGE4_DEPENDENCY_PREFIX`；正式 A/B 不得共享这里的开发前缀或彼此的构建/安装树。构建完成后保存 compiler、CMake cache、ELF/RUNPATH、lock/source materialization hash 和逐成员安装树 digest；相同输入在不同根必须得到相同安装文件清单与内容 hash。

- [ ] **Step 8: 写依赖探针**

```python
EXPECTED = {
    "ecal": "6.1.1",
    "python_protobuf": "6.33.6",
    "protoc": "33.6",
    "cpp_libprotobuf": "33.6",
    "gcc_c_major": "13",
    "gcc_cxx_major": "13",
    "cmake_major_minor": "3.28",
    "ctest_major_minor": "3.28",
}
```

探针输出 JSON，包含 Python eCAL wheel cache/manifest/PEP 425 tag/ELF inventory、`build/stage4-deps` 中源码构建的 C++ eCAL SDK、Python Protobuf、独立 `protoc 33.6`、C++ `libprotobuf 33.6`、GCC、CMake、GLIBCXX ABI、MCAP/Zstd commit、锁定 PCL validator、ROS message overlay lock、系统依赖 lock 和每个 ELF 的 RUNPATH/NEEDED/解析路径。v2 构建不得调用当前环境的 `grpc_tools.protoc 31.1` 或 PATH 中 `protoc 35.1`。安装 ELF 的非系统依赖必须解析到当前安装 root 的 `lib/`；系统依赖只能命中 `ubuntu24-system-dependencies.lock` 的 SONAME 白名单，并记录实际 dpkg 版本。Python/C++ 双进程 smoke 必须是 no-participant loader，从 `/proc/<pid>/maps` 证明各自只加载其私有 eCAL core，且没有第二套 eCAL 或 libprotobuf；调用审计与前后 entity census 必须同时证明未调用 Initialize/pub/sub 且 participant/topic 增量为 0。任何 Conda/仓库路径、未知系统 DSO、Python/C++ eCAL 交叉加载、第二套 libprotobuf 或新增 eCAL entity 都非零退出；build tree 的临时 RPATH 可以指向冻结 dependency prefix，但不得进入 install tree。

探针成功时不能只把路径打印到日志；它还要用 exclusive create + fsync 原子生成调用者指定的 sourceable `stage4-build-env.sh`，随后每个 A-E 入口都先由同一 verifier 复核该文件绑定的 JSON evidence/hash，再显式 `source`。文件用受限 shell assignment serializer 逐项单引号转义，拒绝换行、NUL、命令替换和重复变量，只导出绝对路径合同：`STAGE4_CMAKE`、`STAGE4_CTEST`、`STAGE4_CC`、`STAGE4_CXX`、`STAGE4_PROTOC`、`STAGE4_MICROMAMBA`、`STAGE4_PYTHON_PACKAGE_CACHE`、`STAGE4_PYTHON_WHEEL_CACHE`、`STAGE4_SOURCE_ARCHIVE_CACHE`、开发用 `STAGE4_DEPENDENCY_PREFIX`、`STAGE4_CMAKE_PREFIX_PATH`、`STAGE4_PCL_PCD2PLY`、真实样例只读 `STAGE4_MID360_REFERENCE_LVX2` 和 Jazzy `STAGE4_RVIZ2`。每个 executable/file/prefix/cache 必须存在，工具版本分别为 CMake/CTest 3.28.x、GCC/G++ 13、libprotoc 33.6、锁定 micromamba/PCL、官方只读 LVX2 与 `/opt/ros/jazzy` 的 RViz2，package/wheel/source cache manifest/tree digest 必须精确匹配；env evidence 保存每个路径的 realpath、类型、版本或 tree/file hash，source 后立即逐项 preflight。子计划命令不得改用裸 PATH 工具、未定义变量或空 source 目录。E final 只继承工具路径、micromamba 与三个 canonical cache，不得继承开发 `STAGE4_DEPENDENCY_PREFIX` 作为正式依赖输入。

- [ ] **Step 9: 运行 GREEN、真实 checkout、离线 Python 与 REFACTOR 复验**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_reference_manifest.py tests/stage4/test_stage4_dependencies.py tests/stage4/test_python_offline_runtime.py tests/stage4/test_network_isolation.py`

Expected: PASS；fixture 同时证明 commit/checksum 缺失、Star 元数据缺失、Zstd 未锁定、依赖前缀混入第二套 libprotobuf、Livox-SDK2/driver 任一缺锁、ROS message interface hash 漂移，以及 Python lock/cache/toolchain/order、伪造隔离、同 netns、默认路由、非 loopback interface 或断网证据漂移任一负例都会失败。

Run:

```bash
test -x "$STAGE4_MICROMAMBA_INPUT"
test -x "$STAGE4_CMAKE_INPUT"
test -x "$STAGE4_CTEST_INPUT"
test -x "$STAGE4_CC_INPUT"
test -x "$STAGE4_CXX_INPUT"
test -x "$STAGE4_PROTOC_INPUT"
test -x "$STAGE4_PCL_PCD2PLY_INPUT"
test -x "$STAGE4_LDD_INPUT"
test -x "$STAGE4_DPKG_QUERY_INPUT"
test -d "$STAGE4_PYTHON_PACKAGE_CACHE_INPUT"
test -d "$STAGE4_PYTHON_WHEEL_CACHE_INPUT"
test -d "$STAGE4_SOURCE_ARCHIVE_CACHE_INPUT"
test -d "$STAGE4_DEPENDENCY_PREFIX_INPUT"
test -f "$STAGE4_MID360_REFERENCE_LVX2_INPUT"
test -f packaging/locks/ubuntu24-system-dependencies.lock
STAGE4_BUILD_ENV_FILE="$PWD/build/stage4-toolchain.env.sh"
STAGE4_BUILD_ENV_EVIDENCE="$PWD/build/stage4-toolchain.env.json"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --cmake "$STAGE4_CMAKE_INPUT" \
  --ctest "$STAGE4_CTEST_INPUT" \
  --cc "$STAGE4_CC_INPUT" \
  --cxx "$STAGE4_CXX_INPUT" \
  --protoc "$STAGE4_PROTOC_INPUT" \
  --micromamba "$STAGE4_MICROMAMBA_INPUT" \
  --python-package-cache "$STAGE4_PYTHON_PACKAGE_CACHE_INPUT" \
  --python-wheel-cache "$STAGE4_PYTHON_WHEEL_CACHE_INPUT" \
  --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE_INPUT" \
  --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX_INPUT" \
  --pcl-pcd2ply "$STAGE4_PCL_PCD2PLY_INPUT" \
  --system-lock "$PWD/packaging/locks/ubuntu24-system-dependencies.lock" \
  --ldd "$STAGE4_LDD_INPUT" \
  --dpkg-query "$STAGE4_DPKG_QUERY_INPUT" \
  --mid360-reference-lvx2 "$STAGE4_MID360_REFERENCE_LVX2_INPUT" \
  --rviz2 "$STAGE4_RVIZ2_INPUT" \
  --write-env "$STAGE4_BUILD_ENV_FILE" \
  --json "$STAGE4_BUILD_ENV_EVIDENCE"
source "$STAGE4_BUILD_ENV_FILE"
test -x "$STAGE4_MICROMAMBA"
test -d "$STAGE4_PYTHON_PACKAGE_CACHE"
test -d "$STAGE4_PYTHON_WHEEL_CACHE"
test -d "$STAGE4_SOURCE_ARCHIVE_CACHE"
conda run -n slope-sim python scripts/verify_python_lock_cache.py \
  --runtime-spec packaging/python-environment.yml \
  --toolchain-spec packaging/python-toolchain-environment.yml \
  --virtual-packages packaging/locks/virtual-packages.yml \
  --runtime-unified packaging/locks/python.conda-lock.yml \
  --runtime-explicit packaging/locks/python-linux-64.lock \
  --toolchain-unified packaging/locks/python-toolchain.conda-lock.yml \
  --toolchain-explicit packaging/locks/python-toolchain-linux-64.lock \
  --cache-manifest packaging/locks/python-package-cache.manifest.json \
  --cache-root "$STAGE4_PYTHON_PACKAGE_CACHE"
```

Expected: rc=0；两组 lock 与 canonical artifact 一一对应，micromamba binary hash 精确匹配，artifact tree digest 固定，且 producer evidence 证明 fake channel 已完成“嵌套 artifact -> 每轮私有 native flat cache -> 严格断网 explicit create”的双根 GREEN；不存在 basename hash 碰撞。

Run: `conda run -n slope-sim python scripts/verify_python_wheel_cache.py --manifest packaging/locks/python-wheel-cache.manifest.json --cache-root "$STAGE4_PYTHON_WHEEL_CACHE"`

Expected: rc=0；唯一官方 eCAL 6.1.1 wheel 的 URL/filename/tag/size/SHA-256、METADATA/WHEEL/RECORD、全部 license/NOTICE 和 ELF/DSO inventory 精确匹配，未携带 `libprotobuf.so`，artifact 无链接或额外成员。

Run: `conda run -n slope-sim python scripts/verify_stage4_source_cache.py --manifest packaging/locks/source-archive-cache.manifest.json --lock packaging/locks/cpp-dependencies.lock --lock packaging/locks/ros2-dependencies.lock --cache-root "$STAGE4_SOURCE_ARCHIVE_CACHE"`

Expected: rc=0；两个源码 lock 与 canonical archive 一一对应，七个归档的 URL/ref_kind/ref/commit/format/size/SHA-256/consumer、archive member census、零链接 materialized tree digest 和 artifact tree digest 精确匹配；锁定 Zstd 根内相对 link 已安全物化，没有恶意/多余 member、cache 链接、同 basename 异 hash 或可写共享解包树。

Run: `conda run -n slope-sim python scripts/verify_stage4_dependencies.py --verify-env "$STAGE4_BUILD_ENV_FILE" --json results/stage4/dependencies.json`

Expected: 在依赖未安装齐全的开发机可明确列出 `missing` 并非零退出；安装齐全的 release builder 必须全部 `ok`。

Run: `bash scripts/sync_references.sh --check`

Expected: 13 个 checkout 的 HEAD 与 manifest 完全一致，七个阶段四仓库声明的全部 focus 路径均存在。完成必要整理后原样重跑本 Step 的全部命令并保持 GREEN；无需整理时在证据中记录“REFACTOR：无必要”。

**进入 E 的硬门：** 上述生产 environment spec、两组 unified/explicit lock、toolchain provenance、micromamba binary、canonical Python package artifact、canonical Python eCAL wheel artifact、canonical C++/ROS source archive artifact、三份 manifest/tree digest、deterministic native-cache/wheel/source materializer、三个 lock/cache verifier 和断网 builder 必须全部存在且 hash 冻结；`test_python_offline_runtime.py`、`test_stage4_dependencies.py` 及三个实际 cache verifier 必须同时 GREEN。E 只允许读取这些精确产物，并把 lock、三个 canonical tree、每轮私有 materialization evidence 和 toolchain digest 写入 stage/build evidence；不得在 E 内运行 conda-lock、render、solve、repoquery、download、Git fetch、pip index 或源码下载，不得读取开发机 Conda/Mamba/pip/source cache，也不得用新增 channel、pip manager 或联网 fallback 补输入。任一输入需要变化时返回本 Task 重新生成并重过 RED/GREEN，不能在 E 临时修锁。

## Task 3：按硬门顺序执行五个子计划

**Files:**
- Modify: `docs/阶段四交付报告.md`
- Test: 各子计划列出的聚焦与外部门禁

- [ ] **Step 1: 复核并加载可执行环境合同**

每次开始或恢复 A-E 任一子计划都必须独立运行本 Step，不能依赖另一个 shell/线程已经 source：

```bash
test -n "${STAGE4_BUILD_ENV_FILE:-}"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-env "$STAGE4_BUILD_ENV_FILE" \
  --json "$STAGE4_BUILD_ENV_FILE.subplan-preflight.json"
source "$STAGE4_BUILD_ENV_FILE"
test -x "$STAGE4_CMAKE" && test -x "$STAGE4_CTEST"
test -x "$STAGE4_CC" && test -x "$STAGE4_CXX"
test -x "$STAGE4_PROTOC" && test -x "$STAGE4_MICROMAMBA"
test -d "$STAGE4_PYTHON_PACKAGE_CACHE"
test -d "$STAGE4_PYTHON_WHEEL_CACHE"
test -d "$STAGE4_SOURCE_ARCHIVE_CACHE"
test -f "$STAGE4_MID360_REFERENCE_LVX2"
test -x "$STAGE4_RVIZ2"
```

Expected: env/evidence hash、全部 realpath/version/file/tree digest 与 Task 2 冻结值一致；任一缺失、漂移、未定义变量或 shell 注入嫌疑都在子计划创建输出前失败。E final 随后按 E 计划显式 unset 开发 dependency/protoc/PCL 变量并在每轮私有重建。

- [ ] **Step 2: 执行 A 并冻结 v2 descriptor**

Plan: `docs/superpowers/plans/2026-07-31-stage4-a-v2-protocol-session.md`。逐个执行其中的复选步骤；该 Markdown 路径不是 shell 命令。

Expected: Python/C++ golden bytes、raw wire hash、同 topic v1 冲突、authority 状态机和 reconnect lifecycle 全部通过；生成 descriptor SHA 写入报告。

- [ ] **Step 3: 执行 B 并冻结传感器/性能合同**

Plan: `docs/superpowers/plans/2026-07-31-stage4-b-mid360-rtk-performance.md`。只有 A 的完成证据满足进入门槛后开始。

Expected: 四车型三地形 DIRECT 通过，单中心雷达与三点 RTK 误差符合规格，GUI 布局稳定；性能只能记录实测，不提前写 PASS。

- [ ] **Step 4: 执行 C 并冻结 C++/MCAP 主链路**

Plan: `docs/superpowers/plans/2026-07-31-stage4-c-cpp-ecal-recorder.md`。先完成自动 RED/GREEN，再申请逐条真实 eCAL 授权。

Expected: C++ Subscriber/Command/Recorder 与 Python Simulator 在五 topic 上完成三方双向窗口/fence 比对，零 drop；业务 raw bytes 与 `RecordMetadata` 一一配对，segment/session manifest 原子完成且可跨段完整读取；verifier JSON 保存最终 manifest 的绝对路径、SHA-256 和全部 segment 证据。

- [ ] **Step 5: 执行 D 并冻结显示/导出**

Plan: `docs/superpowers/plans/2026-07-31-stage4-d-ros2-replay-export.md`。只消费 C 已生成并验证的完整 golden session。

Expected: ROS 2 TF 精确配对、RViz2 实时/回放、隔离 eCAL replay、PCD/PLY 和合成 LVX2 均形成实际证据；Replay 为每个隔离 topic 注册 Reader 已验证的原始完整 v2 type name、`proto` encoding 和 descriptor bytes，raw payload 不重新序列化且能通过 Bridge 同一严格 metadata gate；真实 replay/export 只消费 C 的 manifest evidence，小型完整 self-test 由正式 Recorder/Reader 确定性生成并可供安装 smoke。

- [ ] **Step 6: 执行 E Tasks 1-8 并完成候选验收**

Plan: `docs/superpowers/plans/2026-07-31-stage4-e-release-acceptance.md`。只在 A-D 的完成证据都可读取后开始；本 Step 精确执行 E Tasks 1-8，在 Task 8 完成候选真实运行与干净机迁移证据后停止。E Task 9 由本总计划 Task 4 承接，不能在这里提前执行后再重复审查。

Expected: 完整非 eCAL、严格串行 GUI、获授权真实 eCAL、Livox Viewer 和干净 Ubuntu 迁移全部完成；安装器、教程和纯回归完成后，两次绝对空 work root 构建出 byte-identical `artifact_purpose=release, publishable=true` acceptance candidate，第三个全新根构建 manifest/build evidence 均固定 `artifact_purpose=lifecycle_probe, publishable=false` 的合法 probe，普通 release verifier 无法为它签发 handoff。primary 三件套和 probe 四件套分别以受锁目录事务提交并由下游 context 复核，不使用 shell `install + mv`；控制机再从已验证 handoff 生成只含 canonical version/basename/hash 的 portable transfer context，经 pinned SSH 完整复制，目标机校验其内部 `SHA256SUMS` 后才 source，不能保留 `<version>` 或 glob。目标机随后用同一目录事务提交 probe preflight JSON/env，并由独立 consumer 复核后真实完成 probe 升级、原子回退和非 current 卸载；初次/repeat smoke、chain、lifecycle 与四类 production 原始 MCAP/导出/截图/日志/inventory 在远端路径仍有效时打入受限 portable archive，经 pinned SSH host key 和一次性 challenge 回传。控制机用持久 registry 原子消费 challenge，导入本地 chain/lifecycle/production 和 receipt。candidate handoff、lifecycle-probe handoff、clean-host import context 与全部真实验收 evidence 可供 E Task 9 消费，但尚不对外宣称最终发布。

- [ ] **Step 7: 汇总各子计划 REFACTOR 与硬门证据**

本 Task 不直接改生产代码；逐一确认 A-D 的最后一个 GREEN 以及 E Tasks 1-8 均记录了 REFACTOR 或“无必要”，并把相同测试命令、外部门禁状态和证据路径写入交付报告。E 的最终 REFACTOR/状态裁决留给 Task 4 执行 E Task 9；缺任一项时返回所属子计划，不能在总计划补写虚假完成状态。

## Task 4：承接 E Task 9 的最终审查与只读复核

**Files:**
- Delegate ownership: E Task 9 唯一负责修改 `docs/阶段四交付报告.md` 与 `README.md`
- Consume read-only: `3d仿真平台需求规格.md`
- Consume read-only: E Task 9 的 accepted-candidate、clean-host import、lifecycle-probe、final handoff 与不可变六维 review JSON/handoff

- [ ] **Step 1: 执行 E Task 9 Steps 1-2 的唯一六维审查**

冻结实现 writer，按 E Task 9 启动唯一独立只读审查。审查者不得修改代码，必须从需求完整性、逻辑正确性、边界情况、代码质量、测试覆盖和实际运行结果六方面逐项给出文件/行号和证据路径，并在仓库外输出包含 reviewer identity/task id、被审 commit/tree、全部 findings/disposition 与逐项 path/size/SHA-256 evidence index 的 canonical source；本总计划不得另起一个会与 E Task 9 竞争所有权的第二审查。

- [ ] **Step 2: 按 E Task 9 清零发现并完成正式重建**

Critical/Important 未清零不得继续；修复由所属 A-E Task 完成原 RED/GREEN/REFACTOR，审查任务只复核，修复后旧审查 source/transaction 失效并重新启动独立复审。清零后继续原样执行 E Task 9 Steps 3-7：先把最新 canonical review source 以一次目录事务冻结成内嵌精确六维 verdict/findings/disposition 和 evidence index 的不可变 JSON/handoff，再冻结候选 installed root/state/relocation marker/doctor/no-participant smoke 五件套、clean-host import context、imported chain/lifecycle/production、consumed challenge receipt、`publishable=false` lifecycle-probe handoff、accepted-candidate context 与候选 `functional_source_epoch`；accepted context 绑定 review handoff，不读取或冻结交付报告/README。控制机从 imported production root 核对原始 MCAP/导出/截图/日志/inventory，不读取目标机 `/opt`/`$HOME` 路径。随后再次取得用户 Git 授权，从最终 evidence commit 只重建两个 `artifact_purpose=release` 根，证明闭包外功能 payload 逐 byte 等价并自底向上重算代码内固定的受限 provenance 派生闭包，重跑同 schema final 安装 smoke，并分别以单次目录事务提交 equivalence JSON/handoff 和 final-status JSON/handoff。最终只以显式消费 accepted context、final handoff、equivalence 和 final 五件套的 `final-release-status.json` 裁决状态。lifecycle probe 不重建、不进入正式 payload、equivalence 或发布目录。需求规格已在 E Task 5 的候选 clean-HEAD 前完成，Task 4 不再修改它。

- [ ] **Step 3: 运行最终静态一致性检查**

Run: `git diff --check`

Expected: 无输出。

Run: `rg -n "[T]BD|[T]ODO|[F]IXME" README.md 3d仿真平台需求规格.md docs/阶段四交付报告.md docs/superpowers/specs/2026-07-31-stage4-mid360-ecal-cpp-delivery-design.md docs/superpowers/plans/2026-07-31-stage4-{master-implementation,a-v2-protocol-session,b-mid360-rtk-performance,c-cpp-ecal-recorder,d-ros2-replay-export,e-release-acceptance}.md`

Expected: 不存在未决占位符；测试代码中的人工占位样例不在扫描范围。

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_delivery_report_contract.py tests/stage4/test_stage4_docs.py`

Expected: PASS；报告和 README 中每个完成/PASS 状态都由结构化 evidence 路径、摘要和相应外部门禁状态支撑，否定句或历史失败记录不会被文本正则误判。

- [ ] **Step 4: 只读核对 E Task 9 的最终状态**

```markdown
> 实现状态：完成

完成依据：A-E 全部硬门通过，Critical/Important 为 0，真实 eCAL、真实 GUI/RViz2、Livox Viewer 2 和干净机迁移证据路径均可读取。
```

上述文字只应由 E Task 9 Step 7 在 fresh 验证 final-status handoff 后写入；报告与 README 只是下游展示，不得作为 accepted context、六维 review transaction 或 final status 的输入，本 Step 只读核对。若任一外部门禁未执行或环境阻断，状态必须保持“部分完成”，逐项列出剩余项，不能用 LocalTransport、Xvfb 或旧阶段三结果替代，也不能由总计划二次改写状态。

- [ ] **Step 5: 复验审查门且不再产生写入**

所有修复和最终 REFACTOR 裁决必须已经由 E Task 9 或所属子计划记录；总计划不再修改报告、README、需求规格或 archive 输入。原样重跑 Step 3，并只读确认 Step 4 的状态、final handoff、payload 等价与 evidence 仍一致。
