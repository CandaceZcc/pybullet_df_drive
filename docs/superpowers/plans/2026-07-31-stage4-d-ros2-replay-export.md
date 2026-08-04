# 阶段四 D：ROS 2、回放与点云导出 Implementation Plan

> **Execution:** Use `subagent-driven-development` only when the user selects delegated execution; otherwise use `executing-plans`. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变 eCAL 主链路的前提下提供隔离 MCAP 回放、PCD/PLY、合成 LVX2，以及可选 ROS 2 Jazzy/RViz2 实时和回放显示。

**Architecture:** 所有正式工具只读取子计划 C 产出的完整 session manifest，并逐段验证 raw business record 与配对 record-metadata；单 `.mcap` 仅供显式诊断。Reader 从已验证的 MCAP Schema/Channel、逐消息 metadata 和 runtime descriptor 生成每 topic 不可变 Replay publisher contract；回放只把 topic 换到 `/replay/sim/...`，注册原始完整 v2 type/encoding/descriptor 后直接发送 raw bytes，默认永不发布轮控。世界坐标导出只做 exact-time join。ROS Bridge 是可独立重启的 eCAL consumer，用与 live 相同的严格 metadata gate 和有界四槽 joiner 处理跨 topic 异步到达，把原始 v2 点云转换为 Livox `CustomMsg`、标准 `PointCloud2`、严格配对 TF 和命名空间隔离的仿真时钟。

**Tech Stack:** C++17、MCAP C++、Protobuf 33.6.0、Zstd、ROS 2 Jazzy、livox_ros_driver2 messages、sensor_msgs、tf2_ros、RViz2、Livox Viewer 2。

---

**TDD gate:** 本计划所有生产代码任务遵守总路线的严格 RED-GREEN-REFACTOR 协议；Python 或已有 C++ API 的 RED 必须是测试已发现后的行为断言失败。Python 测试只在测试函数内 import 尚未创建的模块；CLI 脚本先以明确断言检查文件并把缺失转成 `FAILED`。新增 C++ API 可以在测试 target 已注册、configure 成功且依赖齐全后，首次因测试引用的目标 API 尚不存在而编译失败；缺 package、unknown target、缺工具、configure/collection/fixture error 或 skip 都不是有效 RED。

**前缀所有权硬门：** C 完成后冻结 `build/stage4-dev-install` 及其 `share/slope-sim/runtime-manifest.json`，D 只读消费，任何 D 命令不得向该树 install、stage、生成 manifest 或覆盖文件。D 的 C++ Reader/Replay/Export 安装和依赖 closure 只写 `build/stage4-d-install`；每轮 ROS dependencies/Bridge 则把 source/SDK/build/install 全部写入一个新的 `mktemp` run root，不能复用固定输出目录。总计划 Task 2 的探针只在构建成功且完整复核后，分别原子更新 `build/stage4-d-ros-{deps,red,green}-context.env` 作为稳定定位；后续 shell 必须验证 kind/evidence/tree hash 后 source，context 不能替代实际前缀证据。D 生成的工具 provenance 属于 D 前缀；读取 C 会话时，`SessionManifest.runtime_manifest_sha256` 仍必须交叉验证 C 冻结 runtime manifest，不能替换成 D 工具 manifest。

**D 执行环境硬门：** 总计划 Task 2 原子生成 `STAGE4_BUILD_ENV_FILE` 并绑定 JSON evidence/hash；它是 source 前唯一允许继承的 `STAGE4_*` 输入。开始 D 或进入任何新 shell 后，必须先在同一 shell 原样运行下列入口，再执行本计划任一 `Run:`。若执行器为每条 `Run:` 新建 shell，就逐条前置该入口；不得依赖上一条命令、上一 Task、终端 profile 或人工 `export` 遗留的工具/cache/样例/RViz2 路径。

Run at the start of every new D shell:

```bash
test -n "${STAGE4_BUILD_ENV_FILE:-}"
test -f "$STAGE4_BUILD_ENV_FILE"
install -d "$PWD/build"
D_ENV_PREFLIGHT_ROOT="$(mktemp -d "$PWD/build/stage4-d-env-preflight.XXXXXX")"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-env "$STAGE4_BUILD_ENV_FILE" \
  --json "$D_ENV_PREFLIGHT_ROOT/environment.json"
source "$STAGE4_BUILD_ENV_FILE"
test -x "$STAGE4_CMAKE" && test -x "$STAGE4_CTEST"
test -x "$STAGE4_CC" && test -x "$STAGE4_CXX" && test -x "$STAGE4_PROTOC"
test -d "$STAGE4_DEPENDENCY_PREFIX" && test -d "$STAGE4_CMAKE_PREFIX_PATH"
test -x "$STAGE4_PCL_PCD2PLY"
test -d "$STAGE4_SOURCE_ARCHIVE_CACHE"
test -f "$STAGE4_MID360_REFERENCE_LVX2"
test -x "$STAGE4_RVIZ2"
```

Expected: verifier 先根据总计划 evidence/hash 复核 env 文件，再 source 并逐项验证绝对路径、类型、版本和 digest；任一缺失、漂移或 source 前使用这些变量都停止 D。`STAGE4_MID360_REFERENCE_LVX2` 与 `STAGE4_RVIZ2` 也只能来自该文件，不能在 Task 4/6 临时从当前 shell 或 PATH 补值。

**断网硬门：** D 中所有 ROS configure/build/test、`ros2 interface`、`colcon` 和真实 ROS/RViz2 verifier 都必须作为 `packaging/run_network_isolated.sh` 的子进程；ROS setup 可先在外层 source，但真正命令不得裸跑。wrapper 每次建立并验证新的 network namespace，只有 loopback 可用且可建立本地 socket，没有 IPv4/IPv6 default route 或非 loopback interface；子进程和证据采集器都要复核内核状态。wrapper 或 loopback 合同不可用即 fail closed，不降级为普通 shell、`--offline` 或可伪造环境标志。

## Task 1：只读 MCAP 会话模型与完整性检查

**Files:**
- Create: `cpp/include/slope_sim/record/mcap_reader.hpp`
- Create: `cpp/src/record/mcap_reader.cpp`
- Create: `cpp/apps/selftest_session_main.cpp`
- Create: `cpp/tests/test_mcap_reader.cpp`
- Create: `cpp/tests/test_selftest_session.cpp`
- Create: `tests/stage4/test_selftest_session_process.py`
- Create: `tests/fixtures/stage4/selftest/recipe.json`
- Modify: `cpp/CMakeLists.txt`

- [ ] **Step 1: 写完整会话 RED**

```cpp
TEST(McapReader, RejectsMissingSceneAttachmentAndDescriptor) {
  const auto manifest = WriteFixtureWithoutSceneAttachment();
  EXPECT_THROW(slope_sim::record::ReadSessionManifest(manifest),
               slope_sim::record::InvalidRecording);
}

TEST(McapReader, PreservesRawPayloadAndIdentity) {
  const auto session = slope_sim::record::ReadSessionManifest(GoldenManifestPath());
  ASSERT_EQ(session.messages.size(), 10u);
  EXPECT_EQ(session.messages[0].payload_sha256,
            Sha256(session.messages[0].raw_payload));
  EXPECT_EQ(session.messages[0].simulation_session_id.size(), 16u);
  EXPECT_EQ(session.descriptor_sha256.size(), 32u);
  EXPECT_EQ(session.runtime_manifest_sha256.size(), 32u);
  const auto& replay_type = session.replay_types.at("/sim/lidar/points");
  EXPECT_EQ(replay_type.protobuf_type,
            "slope_sim.interfaces.v2.LidarPointCloud");
  EXPECT_EQ(replay_type.ecal_encoding, "proto");
  EXPECT_EQ(Sha256(replay_type.file_descriptor_set),
            session.descriptor_sha256);
}

TEST(SelfTestSession, IsCompleteAndReproducible) {
  const auto first = GenerateSelfTestSession(TempDir("first"));
  const auto second = GenerateSelfTestSession(TempDir("second"));
  EXPECT_EQ(ReadBytes(first.manifest), ReadBytes(second.manifest));
  EXPECT_EQ(ReadBytes(first.segment), ReadBytes(second.segment));
  EXPECT_EQ(ReadSessionManifest(first.manifest).topics, FormalFiveTopics());
}
```

覆盖 record schema version 0/2、runtime manifest digest 空/短/不匹配、manifest 缺段/重复段/乱序、`size_bytes` 错、segment SHA 错、首尾 fence 缺/重/未知 topic、first/last topic 集不同、fence gap/重叠、CRC 损坏、截断 summary、未知 type、跨 simulation session 混合、scene attachment 晚于业务帧、相同 effective time、精确 revision 边界、同 `(session,topic,generation,sequence)` 重复。路径反例必须包含绝对 `file_name`、`../`、嵌套目录、重复 basename、segment/attachment 符号链接、非普通文件，以及 open 后 hash 前被替换；全部在解析 MCAP 前失败。self-test generator 还覆盖非空输出目录、错误模型 YAML hash、缺 runtime manifest、重复生成差异和残留 `.partial`。

同一步先在 `cpp/CMakeLists.txt` 注册 `test_mcap_reader` 和 `test_selftest_session`；preset configure 必须成功，首次 build 只允许因 Reader/self-test core API 尚未实现而失败。生产 `stage4-selftest-session` target 留到独立 process RED，不能用 CLI wiring 掩盖 core 失败。

- [ ] **Step 2: 证明 CTest 已注册并运行 core RED**

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^(mcap_reader|selftest_session)$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_mcap_reader test_selftest_session`

Expected: configure 成功且 `ctest -N` 恰好列出两个测试；build 只因 Reader/self-test core API 尚不存在而失败。

- [ ] **Step 3: 确认 core RED 的失败原因正确**

诊断必须来自 wished-for Reader/generator 类型或符号；unknown target、0 tests、依赖/fixture 路径、MCAP 动态库或 configure 错误都不算 RED。修正测试壳后原样重跑 Step 2。

- [ ] **Step 4: 最小实现不可变记录视图**

```cpp
struct RecordedMessage final {
  std::string topic;
  std::string protobuf_type;
  std::array<std::byte, 16> simulation_session_id;
  std::array<std::byte, 32> descriptor_sha256;
  std::uint64_t timestamp_ns;
  std::int64_t send_timestamp_us;
  std::int64_t send_clock;
  std::uint64_t received_wall_time_ns;
  std::uint64_t scene_revision;
  std::array<std::byte, 32> scene_attachment_sha256;
  std::uint64_t world_generation;
  std::uint64_t sequence;
  std::optional<std::uint64_t> command_generation;
  std::optional<std::string> source_id;
  std::optional<std::array<std::byte, 16>> source_session_id;
  std::vector<std::byte> raw_payload;
  std::array<std::byte, 32> payload_sha256;
  std::uint32_t segment_index;
  std::uint64_t record_order;
};
```

Reader 在返回第一条消息前要求两个 record schema version 都精确为 1，验证 session manifest 自身、32-byte runtime manifest digest、全部 segment 实际大小/文件 SHA、MCAP magic/summary/index/CRC、descriptor、session metadata 和规范化 scene attachments。manifest 先经 `resolve(strict=True)` 固定父目录；segment/attachment name 只能是唯一规范 basename，Reader 用不跟随符号链接的普通文件句柄读取，并在读取前后复核 device/inode/size，拒绝绝对路径、目录组件、`..`、链接、非普通文件和 TOCTOU 替换。manifest 每个 `SceneAttachmentEntry` 的 revision/world generation/effective timestamp/name/SHA 都必须与对应段内唯一 attachment 相同。每段 `first_by_topic/last_by_topic` 都必须恰好等于五个正式 topic 各一次，两个集合相同且 command optional presence 正确。按 manifest 顺序合并 segment，并用首尾 fence 证明没有缺段、重叠或跨段 sequence 断裂。每个业务 record 后必须紧邻且恰有一个 C 计划 `RecordMetadata` pair；两条 MCAP Message 的 publish/log time 必须分别等于 metadata identity timestamp/received wall time，内建 sequence 必须为 0。reader 重新计算 raw payload SHA，并核对 topic/type/session/descriptor/timestamp/generation/sequence 和 command optional presence；`record_order` 只由 manifest 段顺序和实际 pair 出现顺序派生。scene attachment 从 revision 1/time 0 开始连续、effective time 严格递增，消息按 `[effective_i,effective_{i+1})` 得到唯一 revision，精确边界属于新 revision；reader 再重算 canonical YAML attachment SHA 并与 `scene_attachment_sha256` 精确比较。所有消息必须保留 eCAL send timestamp/clock 和接收墙钟；WheelCommand 必须同时具有 command generation、source id/session，四个输出则必须没有这些命令专属字段。缺字段、pair 错序/多配/少配、scene revision 不唯一、attachment hash 不同或 metadata 与 raw payload 身份不一致均拒绝；不修改输入文件，也不尝试原地修复 `.partial`。开发工具把 digest 和传入的开发安装 runtime manifest 交叉验证；发行 verifier 则必须与 release root 的同名文件交叉验证。

每个业务 topic 还必须得到唯一 `ReplayTypeMetadata`：MCAP Schema 的 `name=<完整 v2 type>`、`encoding="protobuf"` 和 `data=<FileDescriptorSet bytes>`，Channel 的 `message_encoding="protobuf"` 以及静态 `ecal_type_name/ecal_encoding="proto"/descriptor_sha256_hex/simulation_session_id_hex`，都要与配对 `RecordMetadata.protobuf_type`、session manifest、消息带内 descriptor digest 和当前 runtime descriptor 逐项一致；两个 `_hex` 值分别只接受 64/32 位小写十六进制。该值由 Reader 验证后随 session 返回；跨 segment 缺失、重复冲突、type/encoding 错误、descriptor bytes 或 digest 错误都在暴露任何消息或创建 replay publisher 前拒绝。Replay 和 Bridge 禁止从文件名、topic 名、生成类名或本地默认 metadata 重建这些值。

`slope-sim-replay` 和完整 session export 默认只接受 `<session>.manifest.pb`。只有 `slope-sim-export inspect-segment --single-segment file.mcap` 可打开单段做诊断，输出必须标记 `complete_session=false`，不得用于三方 oracle、世界合并或最终交付证据。

- [ ] **Step 5: 最小实现可复现 self-test generator core**

build-only `stage4-selftest-session` 是 self-test 的唯一生产者，复用 C 的正式 Recorder writer、record schema 和 v2 codec，不手写另一种 MCAP。输入固定为受版本控制的 `recipe.json`、B 的 canonical `robot_models.yaml`、A 的 descriptor 和必填 `--runtime-manifest`；输出目录必须为绝对空目录。recipe 使用固定 simulation session、world generation 1、scene revision 1/time 0，包含两个 exact-time group，每组都有 WheelCommand、WheelState、LiDAR、RTK、IMU 的合法 raw/metadata pair；点云同时含有限的地面点和障碍表面点，RTK/IMU/WheelState 足以完成世界坐标导出。输出恰为一个 finalized segment、`session.manifest.pb` 和 `selftest-evidence.json`，没有 `.partial`；evidence 记录 recipe、模型 YAML、runtime manifest、segment 和 session manifest 的 SHA-256。

同一输入在两个不同绝对目录生成的 manifest、MCAP 和 evidence 必须 byte-identical；只改变 runtime manifest bytes 时，业务/metadata MCAP segment 仍必须 byte-identical，只有 `SessionManifest.runtime_manifest_sha256`、最终 manifest hash 和 evidence 中对应字段按输入确定性变化。该 fixture 只证明安装后 Reader/Replay/Export 的小型 smoke，不替代 C/E 真实 eCAL、频率、运动或零 drop 证据。

- [ ] **Step 6: 运行 Reader/generator core GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_mcap_reader test_selftest_session && "$STAGE4_CTEST" --preset stage4-dev -R '^(mcap_reader|selftest_session)$' --output-on-failure --no-tests=error`

Expected: 两个 CTest PASS，Reader 与 generator core 的正反例全部锁定；尚未声称生产 CLI wiring 可用。

- [ ] **Step 7: 写安装后 self-test CLI/process RED**

注册 `stage4-selftest-session` 生产 target，先只实现参数解析、`--version` 和稳定 `UNIMPLEMENTED` 退出。`tests/stage4/test_selftest_session_process.py` 从必填绝对 `STAGE4_SELFTEST_BINARY` 启动进程，用 `STAGE4_C_RUNTIME_MANIFEST` 指向 C 冻结 manifest；正例在两个全新绝对空目录生成并逐文件 byte 比较，反例覆盖相对/非空输出目录、错模型 hash、缺 runtime manifest、残留 `.partial` 和重复生成。stub 可启动但行为断言失败，不能以文件缺失冒充 RED。

- [ ] **Step 8: 运行并确认 self-test process RED**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target stage4-selftest-session && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run: `STAGE4_SELFTEST_BINARY="$PWD/build/stage4-d-install/bin/stage4-selftest-session" STAGE4_C_RUNTIME_MANIFEST="$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json" conda run -n slope-sim python -m pytest -q tests/stage4/test_selftest_session_process.py`

Expected: D 前缀中的生产 target 可启动，pytest 正常收集并因 `UNIMPLEMENTED`/缺预期 finalized session 而 `FAILED`。若失败来自 C 前缀被写入、相对 binary、动态库、临时目录或 collection error，先修基础设施并原样重跑本 Step。

- [ ] **Step 9: 最小实现 self-test CLI wiring**

CLI 只把参数验证、绝对空目录句柄和 Step 5 已通过的 generator core 接起来；运行期间只读 recipe、模型与 C runtime manifest，所有新 ELF/依赖仍来自 D 前缀。成功只在 segment、manifest、evidence 全部 durability 完成且无 `.partial` 后返回 0。

- [ ] **Step 10: 运行安装后 process GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target stage4-selftest-session && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run: `STAGE4_SELFTEST_BINARY="$PWD/build/stage4-d-install/bin/stage4-selftest-session" STAGE4_C_RUNTIME_MANIFEST="$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json" conda run -n slope-sim python -m pytest -q tests/stage4/test_selftest_session_process.py`

Expected: PASS；每个测试运行都自行创建两个全新目录，输出逐 byte 相同，Reader 用 C 冻结 development runtime manifest 交叉验证完整五 topic session；C 前缀内容/hash 不变。

- [ ] **Step 11: REFACTOR 或记录无必要**

只整理已通过的路径验证/Reader/generator 重复；若无必要，记录“REFACTOR：无必要”。

- [ ] **Step 12: 原样复验两个循环**

原样重跑 Step 6 的 CTest GREEN 和 Step 10 的两条安装/process GREEN 命令；process 测试必须再次创建新目录，不复用上次输出。

## Task 2：带完整 v2 metadata 的隔离 eCAL Replay

**Files:**
- Create: `cpp/include/slope_sim/replay/replayer.hpp`
- Create: `cpp/src/replay/replayer.cpp`
- Create: `cpp/apps/replay_main.cpp`
- Create: `cpp/tests/test_replayer.cpp`
- Test: `tests/stage4/test_replay_process.py`
- Consume test-only: C Task 5 的 `stage4_ecal_test_shim` target（不得安装、导出或打包）
- Consume: `proto/slope_sim_control_v1.proto` 与 C 生成的唯一 control codec
- Modify: `cpp/CMakeLists.txt`

- [ ] **Step 1: 写 Replay core 的 namespace、publisher metadata、时序与控制 RED**

```cpp
TEST(Replayer, MapsBusinessTopicsAndDropsWheelCommandByDefault) {
  ReplayOptions options;
  const auto plan = BuildReplayPlan(GoldenSession(), options);
  EXPECT_EQ(plan.MapTopic("/sim/lidar/points"), "/replay/sim/lidar/points");
  EXPECT_FALSE(plan.ShouldPublish("/sim/wheel/command"));
}

TEST(Replayer, PauseStepAndRateUseTimestampBatches) {
  FakeDeadline deadline;
  Replayer replayer(GoldenSessionWithRepeatedTimestamps(), deadline);
  replayer.SetPaused(true);
  EXPECT_EQ(replayer.Advance(), AdvanceResult::kPaused);
  replayer.StepTimestampBatches(1);
  EXPECT_EQ(replayer.Advance(), AdvanceResult::kPublishedOneTimestampBatch);
  EXPECT_TRUE(replayer.IsPaused());
  const auto held_clock = replayer.ReplayClockNs();
  replayer.SetRate(0.5);
  EXPECT_EQ(replayer.ReplayClockNs(), held_clock);
  replayer.SetPaused(false);
  EXPECT_EQ(replayer.Rate(), 0.5);
}

TEST(Replayer, RegistersVerifiedOriginalTypeBeforeSendingRawBytes) {
  const auto plan = BuildReplayPlan(GoldenSession(), ReplayOptions{});
  const auto& publisher = plan.Publishers().at("/replay/sim/lidar/points");
  EXPECT_EQ(publisher.type.name,
            "slope_sim.interfaces.v2.LidarPointCloud");
  EXPECT_EQ(publisher.type.encoding, "proto");
  EXPECT_EQ(Sha256(publisher.type.descriptor), GoldenDescriptorSha256());
  EXPECT_EQ(publisher.first_payload, GoldenRawLidarPayload());
}
```

覆盖原速、0.5x/2x、暂停、单步、跨 scene revision、相同时间按 MCAP record order、rate 切换重置 wall anchor 不补发 burst、结束状态、非法 `0/NaN/Inf/<0.1/>4.0` 倍率，以及实时 `/sim/...` peer 已存在时仍不污染。Fake raw publisher 必须记录“先用 `SDataTypeInformation` 创建，再原样 Send bytes”的事件顺序；逐项删除或篡改 type name、`proto` encoding、descriptor bytes/digest 时必须在第一条 publish 前失败，相同源 topic 跨 segment metadata 冲突同样拒绝。默认计划不存在 WheelCommand publisher；双危险参数开启后只创建 `/replay/sim/wheel/command`，并为它注册已验证的完整 `slope_sim.interfaces.v2.WheelCommand` metadata。控制 fixture 使用 C Task 7 冻结的 `SetReplayPaused/StepReplay/SetReplayRate`；step 只接受 `timestamp_batches=1` 且执行后保持 paused。

同一步先在 `cpp/CMakeLists.txt` 注册 `test_replayer`；preset configure 必须成功，首次 build 只允许因 Replay core API 尚未实现而失败。生产 `slope-sim-replay` target 留到独立 process RED。

- [ ] **Step 2: 证明 CTest 已注册并运行 Replay core RED**

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^replayer$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_replayer`

Expected: configure 成功且 `ctest -N` 恰好列出 `replayer`；build 只因 wished-for Replay core API 尚不存在而失败。

- [ ] **Step 3: 确认 Replay core RED 的失败原因正确**

诊断必须来自 `Replayer`/clock/control state API 缺失；unknown target、0 tests、control binding、MCAP fixture 或依赖错误不算 RED。修正测试壳后原样重跑 Step 2。

- [ ] **Step 4: 最小实现绝对 deadline 回放 core**

```cpp
auto publishers =
    CreateReplayPublishers(session.replay_types, replay_topic_map);

// 以下代码位于调度循环内；这里只查找和校验，不再创建 publisher。
const auto replay_elapsed = std::chrono::duration<double, std::nano>(
    static_cast<double>(message.timestamp_ns - sim_start_ns) / speed);
const auto target = wall_start + std::chrono::duration_cast<
    std::chrono::steady_clock::duration>(replay_elapsed);
deadline.WaitUntil(target);
const auto& type = session.replay_types.at(message.topic);
auto& publisher = publishers.Require(MapReplayTopic(message.topic), type);
publisher.Send(message.raw_payload.data(), message.raw_payload.size());
```

core 用可注入 `steady_clock` 和 publisher interface 输出 publish decision。`CreateReplayPublishers` 在任何 deadline 调度前一次性用每个 `ReplayTypeMetadata` 构造 `SDataTypeInformation`、创建并冻结所需 topic；同一映射 topic 的非完全相同 metadata 必须拒绝，不能静默复用首个 publisher。调度循环中的 `Require` 只能查找并复核既有合同，绝不创建 publisher。`Send` 直接消费 Reader 持有的 raw bytes，禁止 Parse/Serialize。暂停不推进 replay clock，单步只发布下一个相同 timestamp group 并保持暂停。每次 resume/rate change 都以当前 replay clock 和当前 wall time 重建 anchor，禁止补发 burst。默认 plan 排除 WheelCommand；只有 options 同时包含 `include_wheel_command` 与 `unsafe_acknowledge_wheel_command` 才映射到仍隔离的 `/replay/sim/wheel/command`，永远不映射回生产 `/sim/wheel/command`，也不放松其 publisher metadata gate。

- [ ] **Step 5: 运行 Replay core GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_replayer && "$STAGE4_CTEST" --preset stage4-dev -R '^replayer$' --output-on-failure --no-tests=error`

Expected: `replayer` PASS；所有 wall/replay 时间由 fake clock 确定，尚未启动生产 CLI。

- [ ] **Step 6: 写 `slope-sim-replay` CLI/process wiring RED**

注册生产 target `slope-sim-replay`，先只实现参数解析、`--version`、连接 control socket 和稳定 `UNIMPLEMENTED` 状态。`tests/stage4/test_replay_process.py` 用绝对 `STAGE4_REPLAY_BINARY` 启动 D 安装树中的真实进程；测试先用 `STAGE4_SELFTEST_BINARY` 和 C 冻结 runtime manifest 在临时目录生成完整 session，再通过绝对 `STAGE4_ECAL_TEST_SHIM` 复用 C Task 5 的 test-only shim。该变量只由 pytest fixture 消费；pytest/Conda/fixture parent 均不注入，fixture 只在 Replay 生产 child 的 spawn 环境设置绝对 `LD_PRELOAD` 和私有 IPC。安装 ELF 的 `DT_NEEDED` 可以使真实 eCAL DSO 映射进 child，但 C Task 5 冻结的 `readelf --dyn-syms`/loader binding allowlist 必须证明每个相关 eCAL 调用都由 shim 截获；任何 Initialize/Finalize/pub/sub/monitoring/entity symbol 回落到真实 DSO 都立即失败。shim 以 fake raw callback 和调用审计捕获 publisher 的完整 `SDataTypeInformation`、原始 Send bytes 及 topic，并提供确定性 peer/control 输入；测试前后系统 eCAL entity census 增量必须为 0。进程通过 control 身份校验后必须先以 `REPLAY/STARTING` 上报 paused/clock/rate 和 session/publisher 准备健康，在完整验证 session/映射并建立 publishers 之前不得发 READY；随后才以 `REPLAY/READY` 上报初始 paused 状态。fake raw callback 观察到的 type name、encoding、descriptor bytes 和业务 raw payload/hash 必须逐字节等于 Reader contract/MCAP，合法 payload 不得经过解析再序列化。测试发送精确 request-id 的 pause、step、0.5x、2x 控制并核对 ACK、status clock、wall rate、同 timestamp record order、结束状态；调用审计还必须证明生产 `/sim/...` publisher 零新增。该文件保持非 `ecal` marker，生产 ELF 不读取测试变量，shim 不得出现在 D 安装树、runtime manifest 或发布包。

同一 process RED 用受控损坏的完整 session fixture 逐项注入缺失/错误 type name、`proto` encoding、descriptor bytes/digest、runtime descriptor 不一致和跨 segment metadata 冲突；fixture builder 必须重写合法 MCAP CRC、segment hash/size 和 manifest hash，使每个反例除目标 metadata 语义外其余完整性检查都通过，不能用更早的文件哈希失败冒充 metadata gate。进程必须非零退出且 fake raw callback 对 `/replay/sim/...` 记录 0 条 Send，不能由 callback fixture 的本地声明补齐远端 metadata。默认 WheelCommand 为 0 条；显式双危险参数的独立 fixture 只允许 shim 在隔离 topic 记录命令，并核对它同样携带正确的完整 v2 metadata。

- [ ] **Step 7: 运行并确认 Replay process RED**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-replay stage4_ecal_test_shim && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run: `STAGE4_REPLAY_BINARY="$PWD/build/stage4-d-install/bin/slope-sim-replay" STAGE4_SELFTEST_BINARY="$PWD/build/stage4-d-install/bin/stage4-selftest-session" STAGE4_C_RUNTIME_MANIFEST="$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json" STAGE4_ECAL_TEST_SHIM="$PWD/build/stage4-dev/lib/libstage4_ecal_test_shim.so" STAGE4_D_INSTALL_PREFIX="$PWD/build/stage4-d-install" conda run -n slope-sim python -m pytest -q -m "not ecal" tests/stage4/test_replay_process.py`

Expected: 生产 target 与 test-only shim 构建且真实安装进程可启动，pytest 正常收集并因 `UNIMPLEMENTED`/未发布预期 replay batch 或 clock 状态而 `FAILED`。unknown target、PATH 同名程序、动态库、control/shim fixture、调用审计、entity census 或 collection error 均不算 RED；D 安装树/runtime manifest 出现 shim 或测试注入入口也必须作为基础设施失败先修正。

- [ ] **Step 8: 确认 Replay process RED 的失败原因正确**

首个失败必须明确指向真实进程的 READY/ACK/publish/clock 行为尚未接线；若是基础设施问题，先修测试壳并原样重跑 Step 7。

- [ ] **Step 9: 最小实现 Replay CLI、eCAL 与 control wiring**

main 只连接 Task 1 Reader、Step 4 core、raw eCAL publishers 和 C 的唯一 control codec。每个 raw publisher 都只使用 Reader 已验证的 `ReplayTypeMetadata` 构造 `SDataTypeInformation`，不能从文件名、topic、生成类或本地默认值猜 type/encoding/descriptor。生产代码仍针对官方 eCAL ABI；本 Task 的自动 process 测试只在启动 child 时注入 test-only shim，不能增加测试 main、生产选择开关或运行时 fallback。`REPLAY/STARTING` 从 control 身份校验后持续到最终 manifest、全部 segment/runtime manifest、隔离 topic map 和 publisher metadata 注册均成功，此前禁止 READY 和任何业务发布；然后只转一次 `REPLAY/READY`，首次 unpause/step 再转 ACTIVE，完成保持 ACTIVE+终态 fence，任何协议/读取/发布错误转 FAILED。Status 的 replay clock/paused/rate optional 字段在 STARTING/READY/ACTIVE 都必须 present，且只允许 REPLAY role 使用，Bridge/verifier 从该状态核对 ROS clock。CLI 只有显式 `--include-wheel-command --unsafe-acknowledge-wheel-command` 双参数时才允许隔离命令 topic；默认永不创建该 publisher。

- [ ] **Step 10: 运行 Replay process GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-replay stage4_ecal_test_shim && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run: `STAGE4_REPLAY_BINARY="$PWD/build/stage4-d-install/bin/slope-sim-replay" STAGE4_SELFTEST_BINARY="$PWD/build/stage4-d-install/bin/stage4-selftest-session" STAGE4_C_RUNTIME_MANIFEST="$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json" STAGE4_ECAL_TEST_SHIM="$PWD/build/stage4-dev/lib/libstage4_ecal_test_shim.so" STAGE4_D_INSTALL_PREFIX="$PWD/build/stage4-d-install" conda run -n slope-sim python -m pytest -q -m "not ecal" tests/stage4/test_replay_process.py`

Expected: PASS；显式构建并启动生产 `slope-sim-replay` main/adapter，只在该 child 的 eCAL ABI 边界使用 test-only shim；fake raw callback 看到原始完整 type name、`proto` encoding、descriptor bytes 和 raw payload/hash，逐项 metadata 负例及跨 segment 冲突均在发送前因目标语义失败。暂停/单步/0.5x/2x 和 clock 全部由 control socket 实测；调用审计证明生产 topic 收到 0 条 replay 消息，entity census 证明没有真实 participant。默认 WheelCommand 为 0 条，显式危险 fixture 只在隔离 topic 上记录携带正确 metadata 的命令；D 安装树/runtime manifest 不包含 shim 或测试注入入口。

- [ ] **Step 11: REFACTOR 或记录无必要**

只整理 Reader/core/eCAL/control adapter 间已出现的重复；若无必要，记录“REFACTOR：无必要”。不得把 wall clock 或 socket 逻辑塞进纯 core，也不得把 test-only shim 变成生产依赖、安装目标或运行时选择开关。

- [ ] **Step 12: 原样复验两个循环**

原样重跑 Step 5 的 CTest GREEN 和 Step 10 的两条生产 process GREEN 命令，不改为 build-tree/PATH binary，不删除 `-m "not ecal"`、child-only shim、调用审计、entity census、安装树扫描或 pause/step/rate 场景。

## Task 3：PCD/PLY 单帧和世界坐标导出

**Files:**
- Create: `cpp/include/slope_sim/export/point_cloud.hpp`
- Create: `cpp/src/export/point_cloud.cpp`
- Create: `cpp/apps/export_main.cpp`
- Create: `cpp/tests/test_point_cloud_export.cpp`
- Create: `tests/stage4/test_point_cloud_export_process.py`
- Consume: Task 1 的 self-test generator/recipe 与 C 冻结 runtime manifest
- Modify: `cpp/CMakeLists.txt`

- [ ] **Step 1: 写 exact join RED**

```cpp
TEST(PointCloudExport, RejectsNearestNeighborPose) {
  auto session = WorldJoinFixture();
  session.imu.front().timestamp_ns += 1;
  EXPECT_THROW(ExportWorldCloud(session), ExactPoseUnavailable);
}

TEST(PointCloudExport, AppliesRtkImuAndModelExtrinsics) {
  const auto cloud = ExportWorldCloud(WorldJoinFixture());
  EXPECT_NEAR(cloud.points[0].x, 11.0, 1e-6);
  EXPECT_NEAR(cloud.points[0].y, 2.0, 1e-6);
  EXPECT_NEAR(cloud.points[0].z, 0.105, 1e-6);
}

TEST(PointCloudExport, ReconstructsZyxYawFromProjectedLateralHeading) {
  const auto pose = ReconstructPose(/*heading=*/ProjectedLateralHeading(0.2, 0.3, 0.4),
                                    /*roll=*/0.2,
                                    /*pitch=*/0.3);
  EXPECT_ROTATION_NEAR(pose.rotation, QuaternionFromZyx(0.2, 0.3, 0.4), 1e-9);
}
```

覆盖缺失/重复 RTK、IMU、WheelState，session/generation 不同，未知 `robot_model`，NaN/Inf，scene revision 外参不匹配；姿态参数化覆盖非零正负 roll/pitch/yaw、wrap 边界和 RTK 水平基线退化，独立 oracle 从原始 quaternion/旋转矩阵投影左轴，不能调用生产恢复 helper。

同一步先在 `cpp/CMakeLists.txt` 注册 `test_point_cloud_export`；preset configure 必须成功，首次 build 只允许因 Export core API 尚未实现而失败。生产 `slope-sim-export` target 留到独立 process RED。

- [ ] **Step 2: 证明 CTest 已注册并运行 Export core RED**

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^point_cloud_export$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_point_cloud_export`

Expected: configure 成功且 `ctest -N` 恰好列出 `point_cloud_export`；build 只因 wished-for Export core API 尚不存在而失败。

- [ ] **Step 3: 确认 Export core RED 的失败原因正确**

诊断必须来自 `PoseKey`/join/transform/writer API 缺失；unknown target、0 tests、模型/fixture 或依赖错误不算 RED。修正测试壳后原样重跑 Step 2。

- [ ] **Step 4: 最小实现唯一 pose key 与变换**

```cpp
struct PoseKey final {
  SessionId simulation_session_id;
  std::uint64_t world_generation;
  std::uint64_t timestamp_ns;

  bool operator<(const PoseKey& other) const {
    return std::tie(simulation_session_id, world_generation, timestamp_ns) <
           std::tie(other.simulation_session_id,
                    other.world_generation,
                    other.timestamp_ns);
  }
};
```

每个 LiDAR 帧必须在该 key 下各找到恰好一个 RTK、IMU、WheelState。位置取 RTK CENTER，roll/pitch 取 IMU；RTK heading 是车体左轴水平投影减 `pi/2`，不能直接当作 Euler yaw。唯一纯函数先计算 `lateral_azimuth=heading+pi/2`、`yaw_zyx=wrap(lateral_azimuth-atan2(cos(roll), sin(pitch)*sin(roll)))`，再构造 `Rz(yaw_zyx)*Ry(pitch)*Rx(roll)`，最后应用发行包 `robot_models.yaml` 的 axle-center→base 和固定 base→lidar 外参。ROS Bridge 必须复用同一函数和测试向量；禁止最近邻、跨代插值、缺字段默认零或第二套姿态公式。

- [ ] **Step 5: 最小实现 PCD/PLY 属性 writer**

PCD/PLY 单帧保留 `x/y/z/offset_time_ns/reflectivity/tag/line`；世界合并另写 `frame_id=world` metadata。文件名固定为 `<session>-g<generation>-s<sequence>-t<timestamp>.pcd|ply`，临时文件完成并 fsync 后原子 rename。

- [ ] **Step 6: 运行 Export core GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_point_cloud_export && "$STAGE4_CTEST" --preset stage4-dev -R '^point_cloud_export$' --output-on-failure --no-tests=error`

Expected: `point_cloud_export` PASS；精确 join、姿态和 writer core 已锁定，尚未声称 CLI 可用。

- [ ] **Step 7: 写 `slope-sim-export` PCD/PLY process RED**

注册生产 `slope-sim-export` target，先只实现参数解析、`--version` 与稳定 `UNIMPLEMENTED` 退出。`tests/stage4/test_point_cloud_export_process.py` 用绝对 `STAGE4_EXPORT_BINARY` 和 `STAGE4_SELFTEST_BINARY`，在临时目录以 C 冻结 runtime manifest 生成完整 session，再验证安装后 CLI 的 `pcd`/`ply` 单帧、世界坐标、原子输出、已存在目标拒绝、相对/`.partial` manifest 拒绝和稳定退出码；stub 必须可启动但行为断言失败。

- [ ] **Step 8: 运行并确认 Export process RED**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-export && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run: `STAGE4_EXPORT_BINARY="$PWD/build/stage4-d-install/bin/slope-sim-export" STAGE4_SELFTEST_BINARY="$PWD/build/stage4-d-install/bin/stage4-selftest-session" STAGE4_C_RUNTIME_MANIFEST="$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json" conda run -n slope-sim python -m pytest -q tests/stage4/test_point_cloud_export_process.py`

Expected: 生产 target 构建且可启动，pytest 正常收集并因 `UNIMPLEMENTED`/缺期望 PCD/PLY 而 `FAILED`；PATH 同名、动态库、临时目录、fixture 或 collection error 不算 RED。

- [ ] **Step 9: 确认 Export process RED 并最小接线**

先确认首个失败只来自真实 CLI 尚未调用 Step 4–5 core；基础设施错误必须修正并原样重跑 Step 8。随后 main 只接参数、Task 1 Reader、模型文件、core 与原子文件 writer，不复制 pose 公式或另写 MCAP reader。

- [ ] **Step 10: 运行安装后 process GREEN 与第三方读取 smoke**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-export && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run: `STAGE4_EXPORT_BINARY="$PWD/build/stage4-d-install/bin/slope-sim-export" STAGE4_SELFTEST_BINARY="$PWD/build/stage4-d-install/bin/stage4-selftest-session" STAGE4_C_RUNTIME_MANIFEST="$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json" conda run -n slope-sim python -m pytest -q tests/stage4/test_point_cloud_export_process.py`

Run: `install -d "$PWD/build" "$PWD/results/stage4/export" && STAGE4_EXPORT_FIXTURE_ROOT="$(mktemp -d -p "$PWD/build" stage4-d-export.XXXXXX)" && STAGE4_EXPORT_RESULT_ROOT="$(mktemp -d -p "$PWD/results/stage4/export" result.XXXXXX)" && install -d "$STAGE4_EXPORT_FIXTURE_ROOT/session" && "$PWD/build/stage4-d-install/bin/stage4-selftest-session" --recipe "$PWD/tests/fixtures/stage4/selftest/recipe.json" --robot-models "$PWD/resources/models/robot_models.yaml" --runtime-manifest "$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json" --output-dir "$STAGE4_EXPORT_FIXTURE_ROOT/session" && "$PWD/build/stage4-d-install/bin/slope-sim-export" pcd "$STAGE4_EXPORT_FIXTURE_ROOT/session/session.manifest.pb" --runtime-manifest "$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json" --frame-index 0 --output "$STAGE4_EXPORT_RESULT_ROOT/golden.pcd" && STAGE4_PCL_SMOKE_DIR="$(mktemp -d -p "$PWD/results/stage4/export" pcl-smoke.XXXXXX)" && test -x "$STAGE4_PCL_PCD2PLY" && "$STAGE4_PCL_PCD2PLY" "$STAGE4_EXPORT_RESULT_ROOT/golden.pcd" "$STAGE4_PCL_SMOKE_DIR/golden.ply"`

Expected: pytest PASS；`install -d` 明确早于 `golden.pcd` 导出，D 安装树 CLI rc=0，PCL 读取后点数和有限坐标不变；session producer 仍绑定 C 冻结 runtime manifest。

- [ ] **Step 11: REFACTOR 或记录无必要**

只整理 Reader/pose/writer/CLI 间已出现的重复；若无必要，记录“REFACTOR：无必要”。

- [ ] **Step 12: 原样复验两个循环**

原样重跑 Step 6 的 CTest GREEN 和 Step 10 的三条 process/smoke 命令；不得把安装前缀改回 C、把 `install -d` 移到导出后或复用旧临时 session。

## Task 4：合成 LVX2 有损显示导出

**Files:**
- Create: `cpp/include/slope_sim/export/lvx2.hpp`
- Create: `cpp/src/export/lvx2.cpp`
- Create: `cpp/tests/test_lvx2_export.cpp`
- Create: `tests/stage4/test_lvx2_export_process.py`
- Create: `docs/阶段四LVX2格式说明.md`
- Modify: `cpp/apps/export_main.cpp`
- Modify: `cpp/CMakeLists.txt`

- [ ] **Step 1: 写 packet/layout RED**

```cpp
TEST(Lvx2Export, UsesMid360TypeOnePackets) {
  const auto file = ExportSyntheticLvx2(GoldenSession());
  const auto parsed = ReadSyntheticLvx2(file.path);
  EXPECT_EQ(parsed.device_type, DeviceType::kMid360);
  EXPECT_TRUE(std::all_of(
      parsed.packets.begin(), parsed.packets.end(), [](const auto& packet) {
        return packet.data_type == 1 && packet.points.size() == 96;
      }));
}
```

覆盖 50ms 分帧、包 timestamp=首个源点 timebase+offset、毫米量化、空区间不出包、尾包重复最后有效点、sidecar 有效/填充计数、source MCAP hash。

同一步先在 `cpp/CMakeLists.txt` 注册 `test_lvx2_export`；preset configure 必须成功，首次 build 只允许因 LVX2 core API 尚未实现而失败。已有 `slope-sim-export` 的新子命令 wiring 留到独立 process RED。

- [ ] **Step 2: 证明 CTest 已注册并运行 LVX2 core RED**

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^lvx2_export$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_lvx2_export`

Expected: configure 成功且 `ctest -N` 恰好列出 `lvx2_export`；build 只因 wished-for LVX2 core API 尚不存在而失败。

- [ ] **Step 3: 确认 LVX2 core RED 的失败原因正确**

诊断必须来自 packet/layout/quantization/sidecar core API 缺失；unknown target、0 tests、fixture 或依赖错误不算 RED。修正测试壳后原样重跑 Step 2。

- [ ] **Step 4: 最小实现显式有损转换**

```cpp
std::int32_t QuantizeMillimeters(const float meters) {
  if (!std::isfinite(meters)) throw InvalidPoint{};
  return CheckedInt32(std::llround(static_cast<double>(meters) * 1000.0));
}
```

每个 96 点包不足时只重复最后一个真实有效点；sidecar JSON 逐包记录 `source_valid_count`、`padding_count`、source frame identity 和 `synthetic=true`。文档明确 LVX2 丢失 line、精确 offset、session/generation/sequence，MCAP 才是无损源。

sidecar 先构造 `google::protobuf::Struct`，再用项目同一 Protobuf 33.6.0 的 `MessageToJsonString` 输出；字段值来自已验证的 session manifest/record metadata。不得引入第二套 JSON 依赖，也不得从文件名反推 source identity。

- [ ] **Step 5: 运行 LVX2 core GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_lvx2_export && "$STAGE4_CTEST" --preset stage4-dev -R '^lvx2_export$' --output-on-failure --no-tests=error`

Expected: `lvx2_export` PASS；packet/layout/sidecar core 自回读通过，尚未声称 CLI 子命令可用。

- [ ] **Step 6: 写 LVX2/inspect CLI process RED**

`tests/stage4/test_lvx2_export_process.py` 从绝对 `STAGE4_EXPORT_BINARY` 启动已有生产 ELF，用 Task 1 self-test generator 和 C 冻结 runtime manifest 创建临时完整 session；正例期待 `lvx2` 生成文件+sidecar并由 `inspect-lvx2` 自回读，反例覆盖 sidecar 计数/hash/identity 变异、相对或 `.partial` manifest、已有目标、截断/未知 packet、PATH 同名程序。尚未接新子命令时必须以稳定 unknown/unimplemented 行为 `FAILED`，不得 collection error 或 skip。

- [ ] **Step 7: 运行并确认 LVX2 process RED**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-export && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run: `STAGE4_EXPORT_BINARY="$PWD/build/stage4-d-install/bin/slope-sim-export" STAGE4_SELFTEST_BINARY="$PWD/build/stage4-d-install/bin/stage4-selftest-session" STAGE4_C_RUNTIME_MANIFEST="$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json" conda run -n slope-sim python -m pytest -q tests/stage4/test_lvx2_export_process.py`

Expected: pytest 正常收集并因真实 D-prefix CLI 的 LVX2/inspect 行为尚未接线而 `FAILED`。若失败来自 binary/依赖/临时 session/eCAL 或 fixture，先修基础设施并原样重跑本 Step。

- [ ] **Step 8: 最小实现 LVX2/inspect CLI wiring**

只在现有 `export_main.cpp` 接入 Task 4 core 与 Task 1 Reader；`lvx2` 必须同时原子生成 data+sidecar，`inspect-lvx2` 只读且不创建旁路修复文件。格式文档明确 LVX2 是有损显示产物，MCAP 才是无损证据；CLI 不复制 reader/parser/JSON 实现。

- [ ] **Step 9: 运行 process GREEN 与官方样例结构门**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-export && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run: `STAGE4_EXPORT_BINARY="$PWD/build/stage4-d-install/bin/slope-sim-export" STAGE4_SELFTEST_BINARY="$PWD/build/stage4-d-install/bin/stage4-selftest-session" STAGE4_C_RUNTIME_MANIFEST="$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json" conda run -n slope-sim python -m pytest -q tests/stage4/test_lvx2_export_process.py`

Run: `test -n "$STAGE4_MID360_REFERENCE_LVX2" && test -f "$STAGE4_MID360_REFERENCE_LVX2" && test "$(sha256sum "$STAGE4_MID360_REFERENCE_LVX2" | cut -d' ' -f1)" = "f892732ff43882b56d1cebc683f6ea9374ab3d3ac688368c9d560f49dcd4d647" && install -d "$PWD/results/stage4" && STAGE4_LVX2_INSPECT_ROOT="$(mktemp -d -p "$PWD/results/stage4" lvx2-inspect.XXXXXX)" && "$PWD/build/stage4-d-install/bin/slope-sim-export" inspect-lvx2 "$STAGE4_MID360_REFERENCE_LVX2" --json "$STAGE4_LVX2_INSPECT_ROOT/official-lvx2-structure.json"`

Expected: pytest PASS；必填路径指向已核验 SHA 的官方样例，D 安装树的 inspect 只读成功，不复制或修改样例。变量缺失或 SHA 不同则此外部样例门明确失败，不能静默 skip，也不改变纯合成 core/process 已通过的独立结论。

- [ ] **Step 10: REFACTOR 或记录无必要**

只整理 packet reader/writer/CLI 中已出现的重复；若无必要，记录“REFACTOR：无必要”。

- [ ] **Step 11: 原样复验两个循环**

原样重跑 Step 5 的 CTest GREEN 和 Step 9 的三条 process/官方样例命令；不得改用 C 前缀、build-tree binary 或跳过官方样例失败。

## Task 5：ROS 2 消息与 eCAL Bridge

**Files:**
- Create: `ros2/slope_sim_msgs/package.xml`
- Create: `ros2/slope_sim_msgs/CMakeLists.txt`
- Create: `ros2/slope_sim_msgs/msg/WheelState.msg`
- Create: `ros2/slope_sim_msgs/msg/RtkTriplet.msg`
- Create: `ros2/slope_sim_msgs/msg/ImuAttitude.msg`
- Create: `ros2/slope_sim_bridge/package.xml`
- Create: `ros2/slope_sim_bridge/CMakeLists.txt`
- Create: `ros2/slope_sim_bridge/include/slope_sim_bridge/exact_frame_joiner.hpp`
- Create: `ros2/slope_sim_bridge/include/slope_sim_bridge/control_status.hpp`
- Create: `ros2/slope_sim_bridge/src/bridge_node.cpp`
- Create: `ros2/slope_sim_bridge/src/exact_frame_joiner.cpp`
- Create: `ros2/slope_sim_bridge/src/control_status.cpp`
- Create: `ros2/slope_sim_bridge/test/bridge_contract_fallback.hpp`
- Create: `ros2/slope_sim_bridge/test/test_bridge.cpp`
- Create: `ros2/slope_sim_bridge/test/test_exact_frame_joiner.cpp`
- Create: `ros2/slope_sim_bridge/test/test_topic_mapping.cpp`
- Create: `ros2/slope_sim_bridge/test/test_bridge_control_status.cpp`
- Consume: 总计划 Task 2 已测试的 `packaging/build_ros_overlay.sh`
- Consume: 总计划 Task 2 已测试的 `packaging/run_network_isolated.sh`
- Consume read-only: `packaging/locks/source-archive-cache.manifest.json`
- Consume read-only: `scripts/verify_stage4_source_cache.py`
- Create: `resources/rviz/slope_sim_mid360.rviz`
- Create: `resources/rviz/slope_sim_mid360_replay.rviz`

- [ ] **Step 1: 先证明并锁定 Livox ROS 官方源码边界**

这是 Bridge RED 前的依赖硬门，不是生产实现。总计划 Task 2 必须已经读取同一固定源码身份的 `Livox-SDK/livox_ros_driver2` package/CMake/message 文件、`Livox-SDK2` 构建说明和两者 LICENSE，并产出已通过 fixture 测试的统一 overlay 构建入口。默认路径是完整 `livox_ros_driver2 + Livox-SDK2`，不得假定 message 定义可脱离官方 package 单独构建。`packaging/locks/ros2-dependencies.lock` 同时记录两个 40 位 commit、源码 SHA-256、SPDX/license files、构建命令和依赖关系。

source archive lock 与 `source-archive-cache.manifest.json` 的每条记录必须一一保存规范化 HTTPS `url`、枚举 `ref_kind: tag | commit`、字符串 `ref` 和 40 位小写十六进制 `commit`，不得再把所有来源强制解释成 tag。`ref_kind=tag` 时，`ref` 必须是精确 tag 名；验证器分别把 annotated/lightweight tag peel 到 commit，并要求结果与 `commit` 完全相同，annotated tag object id 本身不能冒充 commit。`ref_kind=commit` 时，`ref` 必须是 40 位小写十六进制 SHA 且逐字节等于 `commit`；`Livox-SDK2` 固定为 `ref_kind=commit`、`ref=commit=68ae1e1dc77f61f03c95d7c2809831e198d0aedd`。两种类型都明确拒绝 branch identity，包括 `master`、`main`、`refs/heads/*`、remote-tracking ref、symbolic ref；archive URL 也必须绑定该条 `ref`，不能解析移动分支。

总计划 Task 2 的 source-cache fixture 必须已经先以 RED 证明缺失/未知 `ref_kind`、tag ref 不存在、annotated/lightweight tag peel 后 commit 不符、把 tag object id 写入 `commit`、commit ref 非 40 位或不等于 `commit`、Livox-SDK2 被标为 tag，以及任意 branch identity 都被旧/桩实现错误接受；再以 GREEN 证明 lock、manifest、archive URL 和现场 ref 解析四者一致。D 只消费该 GREEN 产物，任一身份反例没有对应测试证据就不得继续 Bridge 构建。

同一总计划 fixture 还必须以严格 TDD 锁住私有 Livox SDK：RED 在临时 fake sysroot/default-search path 的 `usr/local/lib` 和 `usr/local/include` 放入 ABI/版本错误的 shared library 与 headers，证明桩/旧 builder 会默认命中 poison、尝试默认 `/usr/local` 或没有显式 SDK 路径而失败；GREEN 要求 builder 只安装到调用者给定的全新 `--livox-sdk-prefix`，driver 完全忽略 poison。普通 pytest 只能读写临时 fake sysroot，不能用 sudo、不能写真实 `/usr/local`；真实 D 构建才对 `/usr/local/lib` 与 `/usr/local/include` 做前后只读 census。

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_dependencies.py tests/stage4/test_network_isolation.py`

Expected: PASS；测试证据包含 poison/private-prefix RED -> GREEN、直接或伪造隔离失败、wrapper 只有 loopback 的正例，并证明每个正例 builder subprocess 自己进入断网 wrapper；本 pytest 元测试不在外层再嵌套 wrapper，也不配置真实 ROS 或触碰真实 `/usr/local`。

Run: `conda run -n slope-sim python scripts/verify_stage4_dependencies.py --require-ros-lock-closure --json results/stage4/d-ros-prerequisites.json`

Expected: lock 明确列出固定 builder image、Ubuntu 24.04 + Jazzy、RViz2 前置、Livox-SDK2、完整 `livox_ros_driver2` 的所有构建/运行 apt 包（包括其声明的系统 `rosbag2` 运行依赖）、规范化 interface hash、允许 SONAME 和实际 dpkg 版本；任一缺失或 lock/source hash 漂移均在启动 colcon 前失败。

Run: `conda run -n slope-sim python scripts/verify_stage4_source_cache.py --manifest packaging/locks/source-archive-cache.manifest.json --lock packaging/locks/ros2-dependencies.lock --cache-root "$STAGE4_SOURCE_ARCHIVE_CACHE" --consumer ros_overlay`

Expected: rc=0；Livox-SDK2 与完整 `livox_ros_driver2` 两个 canonical archive 的 URL/ref_kind/ref/commit/size/SHA-256、artifact 普通文件约束、member census、零链接 materialized tree digest 与 lock 精确一致；Livox-SDK2 明确通过 commit ref 验证，tag 类型均完成 annotated/lightweight peel，没有 branch identity、缺失、多余、恶意 member 或同 basename 异 hash。

Run:

```bash
STAGE4_D_ROS_DEPS_CONTEXT_FILE="$PWD/build/stage4-d-ros-deps-context.env"
STAGE4_D_ROS_DEPS_RUN_ROOT="$(mktemp -d \
  "$PWD/build/stage4-d-ros-deps.XXXXXX")"
source /opt/ros/jazzy/setup.bash
bash packaging/run_network_isolated.sh env \
  CC="$STAGE4_CC" CXX="$STAGE4_CXX" \
  bash packaging/build_ros_overlay.sh \
  --lock packaging/locks/ros2-dependencies.lock \
  --source-cache-manifest packaging/locks/source-archive-cache.manifest.json \
  --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE" \
  --source-work "$STAGE4_D_ROS_DEPS_RUN_ROOT/source-work" \
  --livox-sdk-prefix "$STAGE4_D_ROS_DEPS_RUN_ROOT/livox-sdk-install" \
  --build-base "$STAGE4_D_ROS_DEPS_RUN_ROOT/build" \
  --install-base "$STAGE4_D_ROS_DEPS_RUN_ROOT/install"
source "$STAGE4_D_ROS_DEPS_RUN_ROOT/install/setup.bash"
install -d "$STAGE4_D_ROS_DEPS_RUN_ROOT/evidence"
bash packaging/run_network_isolated.sh ros2 interface show \
  livox_ros_driver2/msg/CustomMsg \
  > "$STAGE4_D_ROS_DEPS_RUN_ROOT/evidence/CustomMsg.interface"
bash packaging/run_network_isolated.sh ros2 interface show \
  livox_ros_driver2/msg/CustomPoint \
  > "$STAGE4_D_ROS_DEPS_RUN_ROOT/evidence/CustomPoint.interface"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --ros-context-kind dependencies \
  --ros-run-root "$STAGE4_D_ROS_DEPS_RUN_ROOT" \
  --ros-source-work "$STAGE4_D_ROS_DEPS_RUN_ROOT/source-work" \
  --ros-livox-sdk-prefix "$STAGE4_D_ROS_DEPS_RUN_ROOT/livox-sdk-install" \
  --ros-build-base "$STAGE4_D_ROS_DEPS_RUN_ROOT/build" \
  --ros-install-prefix "$STAGE4_D_ROS_DEPS_RUN_ROOT/install" \
  --ros-interface-file \
    "livox_ros_driver2/msg/CustomMsg=$STAGE4_D_ROS_DEPS_RUN_ROOT/evidence/CustomMsg.interface" \
  --ros-interface-file \
    "livox_ros_driver2/msg/CustomPoint=$STAGE4_D_ROS_DEPS_RUN_ROOT/evidence/CustomPoint.interface" \
  --write-ros-build-context "$STAGE4_D_ROS_DEPS_CONTEXT_FILE" \
  --json "$STAGE4_D_ROS_DEPS_RUN_ROOT/context.json"
```

Run:

```bash
STAGE4_D_ROS_DEPS_CONTEXT_FILE="$PWD/build/stage4-d-ros-deps-context.env"
D_ROS_CONTEXT_PREFLIGHT="$(mktemp -d \
  "$PWD/build/stage4-d-ros-context-preflight.XXXXXX")"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-ros-build-context "$STAGE4_D_ROS_DEPS_CONTEXT_FILE" \
  --expect-ros-context-kind dependencies \
  --json "$D_ROS_CONTEXT_PREFLIGHT/dependencies.json"
source "$STAGE4_D_ROS_DEPS_CONTEXT_FILE"
source /opt/ros/jazzy/setup.bash
source "$STAGE4_ROS_INSTALL_PREFIX/setup.bash"
bash packaging/run_network_isolated.sh ros2 interface show \
  livox_ros_driver2/msg/CustomMsg \
  > "$D_ROS_CONTEXT_PREFLIGHT/CustomMsg.interface"
bash packaging/run_network_isolated.sh ros2 interface show \
  livox_ros_driver2/msg/CustomPoint \
  > "$D_ROS_CONTEXT_PREFLIGHT/CustomPoint.interface"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-ros-build-context "$STAGE4_D_ROS_DEPS_CONTEXT_FILE" \
  --expect-ros-context-kind dependencies \
  --ros-interface-file \
    "livox_ros_driver2/msg/CustomMsg=$D_ROS_CONTEXT_PREFLIGHT/CustomMsg.interface" \
  --ros-interface-file \
    "livox_ros_driver2/msg/CustomPoint=$D_ROS_CONTEXT_PREFLIGHT/CustomPoint.interface" \
  --json "$D_ROS_CONTEXT_PREFLIGHT/interfaces.json"
```

Expected: 完整官方源码链在 Jazzy 构建通过，两个规范接口输出的 SHA-256 与 lock 一致，overlay 可独立发现两种消息；builder evidence 证明 only-loopback attestation 有效、SDK/driver cache/link 只命中本轮唯一 run root，并且真实 `/usr/local` pre/post census 完全相同。context 仅在全部证据通过后原子指向该根；任一步失败都保留上一份有效 context 并停止 D 计划，不进入 Bridge RED。重跑必须新建另一 run root，不能清空或复用失败根。若以后要提取最小消息包，必须先作为单独设计变更完成逐文件许可证审查并重新冻结规范化 `ros2 interface show` hash，当前计划不授权该捷径。

`packaging/build_ros_overlay.sh` 是总计划 Task 2 创建并测试、D/E 共用的唯一 ROS 构建入口：先确认处于 wrapper 建立且只有 loopback 的断网 namespace，再以 manifest/lock 只读验证 canonical archive 的 URL/ref_kind/ref/commit、archive bytes 与冻结 member/materialized tree digest；随后按 `ros_overlay` consumer 以 exclusive create 复制到本轮全新空 `--source-work/archives`，复制前后都复核身份和 hash，并用唯一安全 parser 物化到本轮私有零链接 `trees`。禁止修改 canonical root、共享可写归档/解包树、shell extractor、branch 解析或缺包联网。

入口要求显式且彼此独立的 `--source-cache-manifest`、`--source-archive-cache`、`--source-work`、`--livox-sdk-prefix`、`--build-base`、`--install-base` 和 `CC/CXX`；传项目源码时还要求 `--client-prefix` 指向已安装的 C++ SDK。每轮 `--livox-sdk-prefix` 必须是全新空绝对目录，且与 source/build/install/client/canonical root 互不相同、互不包含。Livox-SDK2 configure 必须固定 `-DCMAKE_INSTALL_PREFIX=<livox-sdk-prefix>`，只把 `liblivox_lidar_sdk_shared.so` 与 headers 安装到该私有前缀；全程禁止 sudo、禁止默认 install、禁止创建或写真实 `/usr/local`。完整 `livox_ros_driver2` configure 必须显式设置 `LIVOX_LIDAR_SDK_LIBRARY=<livox-sdk-prefix>/lib/liblivox_lidar_sdk_shared.so` 和 `LIVOX_LIDAR_SDK_INCLUDE_DIR=<livox-sdk-prefix>/include`，并通过 CMakeCache、link command、`readelf` 与 `ldd` 证明没有命中 fake poison、系统默认或其他轮次 prefix。

每次真实 D builder 在任何输出前，都以不跟随链接的排序 `lstat`/regular-file hash 对真实 `/usr/local/lib` 和 `/usr/local/include` 生成只读 pre-census；构建结束或失败后再生成 post-census，要求成员、类型、mode、link target、size 与 content hash 完全一致。census 过程不得在 `/usr/local` 创建临时文件，任一变化立即失败并保留两份证据。依赖、每次 RED 修壳重跑、GREEN 与 REFACTOR 复验分别创建唯一 `stage4-d-ros-*.XXXXXX` run root，source work、Livox prefix、build 和 install 都是该根下此前不存在的 sibling；稳定 context 只定位最后一轮已验证根。D/E 都必须使用新的 source work、Livox prefix 和 wrapper，不能另写裸 `cmake`、`colcon build` 或联网路径。

- [ ] **Step 2: 注册 ROS packages/CTest 并写 Bridge 行为 RED**

```cpp
TEST(Bridge, PublishesLivoxAndPointCloud2WithoutChangingSource) {
  const RawEnvelope source = GoldenLidarEnvelope();
  const auto original_payload = source.capture.payload;
  const auto original_hash = source.payload_sha256;
  const auto converted = ConvertPointCloud(source);
  EXPECT_EQ(converted.livox.point_num, GoldenLidarPointCount());
  EXPECT_EQ(converted.point_cloud_2.fields,
            Fields({"x", "y", "z", "intensity", "range"}));
  EXPECT_EQ(source.capture.payload, original_payload);
  EXPECT_EQ(source.payload_sha256, original_hash);
  EXPECT_EQ(Sha256(source.capture.payload), original_hash);
}

TEST(Bridge, RequiresExactPoseTriplet) {
  ExactFrameJoiner joiner(ExactFrameJoinerOptions{
      64, 256, std::chrono::seconds{2}});
  EXPECT_FALSE(joiner.AddLidar(GoldenLidar(100)).has_value());
  EXPECT_FALSE(joiner.AddImu(GoldenImu(100)).has_value());
  EXPECT_FALSE(joiner.AddWheelState(GoldenWheelState(100)).has_value());
  const auto batch = joiner.AddRtk(GoldenRtk(100));
  ASSERT_TRUE(batch.has_value());
  EXPECT_EQ(batch->key, PoseKey{Session(), 1, 100});
  EXPECT_THROW(joiner.AddRtk(GoldenRtk(100)), DuplicateJoinComponent);
}

TEST(BridgeTopics, KeepsLiveAndReplayNamespacesDisjoint) {
  EXPECT_EQ(TopicMap(BridgeOptions{"/sim", "/slope_sim"}).LidarInput(),
            "/sim/lidar/points");
  EXPECT_EQ(TopicMap(BridgeOptions{"/replay/sim", "/replay/slope_sim"}).LivoxOutput(),
            "/replay/slope_sim/lidar/custom");
  EXPECT_EQ(TopicMap(BridgeOptions{"/replay/sim", "/replay/slope_sim"}).PointCloud2Output(),
            "/replay/slope_sim/lidar/points");
  EXPECT_EQ(TopicMap(BridgeOptions{"/replay/sim", "/replay/slope_sim"}).ClockOutput(),
            "/replay/slope_sim/clock");
}

TEST(BridgeControlStatus, ReadyRequiresAllFourVerifiedInputs) {
  BridgeControlStatus status(FormalFourInputTopics());
  EXPECT_EQ(status.State(), slope_sim::control::v1::STARTING);
  const auto starting = status.BuildStarting();
  EXPECT_EQ(starting.state(), slope_sim::control::v1::STARTING);
  EXPECT_EQ(starting.topic_health_size(), 4);
  status.ObserveVerified("/sim/lidar/points");
  status.ObserveVerified("/sim/wheel/state");
  status.ObserveVerified("/sim/rtk/state");
  EXPECT_FALSE(status.CanReportReady());
  status.ObserveVerified("/sim/imu/attitude");
  EXPECT_TRUE(status.CanReportReady());
  EXPECT_EQ(status.BuildReady().role(), slope_sim::control::v1::BRIDGE);
  status.ObserveConflict("/sim/imu/attitude", "descriptor mismatch");
  EXPECT_EQ(status.State(), slope_sim::control::v1::FAILED);
}

TEST(Bridge, AcceptsReplayOnlyWithOriginalVerifiedPublisherMetadata) {
  EXPECT_TRUE(ValidateReplayEnvelope(GoldenReplayEnvelope()).ok());
  EXPECT_FALSE(ValidateReplayEnvelope(
                   GoldenReplayEnvelopeWithWrongTypeName()).ok());
  EXPECT_FALSE(ValidateReplayEnvelope(
                   GoldenReplayEnvelopeWithWrongEncoding()).ok());
  EXPECT_FALSE(ValidateReplayEnvelope(
                   GoldenReplayEnvelopeWithWrongDescriptor()).ok());
}
```

同一 RED 参数化 24 种四 topic 到达排列，必须都只产出一次相同 batch；再覆盖缺槽 TTL、64-anchor/256-wheel 容量、重复 component、session/generation 前进后迟到旧帧、时间戳差 1ns、不同 descriptor，以及一秒内 100 个 WheelState 只有 10 个与 10 Hz 传感器时间重合。LiDAR/RTK/IMU 任一到达即可建立 10 Hz anchor；WheelState 先到只进入独立 exact-time cache，不得让其余 90 Hz 合法轮态创建永远缺三槽的 anchor。anchor 缺失计 `incomplete_expired` 并使正式门失败；没有任何传感器 anchor 的额外 WheelState 到期只计 `wheel_unmatched_expired` 诊断，不是丢帧。不得最近邻、补零或跨 generation 配对；重复/倒退属于 Bridge 协议失败，Bridge 退出但 Simulator/Recorder 保持健康。Bridge control RED 还必须证明初始 STARTING 帧已包含四个 WAITING/PENDING health、不会打开业务门，四门 VERIFIED 后才允许 STARTING -> READY，跳过 READY 或状态回退精确失败。Bridge metadata RED 对 live `/sim` 与 replay `/replay/sim` 使用同一组完整 v2 type contracts：合法 Replay publisher metadata 必须通过，缺失或错误 type name、encoding、descriptor bytes/digest 必须在 parse/join 前拒绝；测试 envelope 保存 callback 的真实远端 metadata，不得用 Bridge 本地 subscriber 声明代替。

同一步创建两个 `package.xml`、两份 package `CMakeLists.txt` 和三份 `.msg`，先让 colcon/ament 发现 `slope_sim_msgs`、`slope_sim_bridge` 以及四个 gtest/CTest。每个测试通过 `bridge_contract_fallback.hpp` 选择 wished-for 生产 API；生产 header 尚不存在时，fallback 提供可编译但返回明确 `UNIMPLEMENTED`/空结果的最小同形接口。overlay build、消息生成和 test executable 链接必须成功，首次 CTest 只因上述行为断言不匹配而失败；fallback 只位于 `test/`，不得安装或被生产 node 链接。

process fixture 还必须验证 ROS build context 生命周期：同一个 RED Step 因测试壳原因重跑时产生不同 run root，旧根保持只读诊断，新 build 的 source/SDK/build/install 都不与旧根或 dependency context 重合；只有 overlay build、package discovery 和 `ctest -N` 四项注册成功后才原子更新 `bridge_red` context。GREEN 与后续 REFACTOR 复验同样各用不同根，只有四个 CTest 与 `colcon test-result` 全部通过后才更新 `bridge_green` context，最终 context 必须指向第二轮 fresh 产物并绑定 dependency parent context。注入非空目标、旧根复用、context kind/parent/hash 篡改或失败轮覆盖稳定 context 都必须在运行行为测试前失败。

- [ ] **Step 3: 构建测试壳、证明 CTest 注册并运行 Bridge RED**

Run: `"$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run:

```bash
STAGE4_D_ROS_DEPS_CONTEXT_FILE="$PWD/build/stage4-d-ros-deps-context.env"
STAGE4_D_ROS_RED_CONTEXT_FILE="$PWD/build/stage4-d-ros-red-context.env"
D_ROS_CONTEXT_PREFLIGHT="$(mktemp -d \
  "$PWD/build/stage4-d-ros-context-preflight.XXXXXX")"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-ros-build-context "$STAGE4_D_ROS_DEPS_CONTEXT_FILE" \
  --expect-ros-context-kind dependencies \
  --json "$D_ROS_CONTEXT_PREFLIGHT/dependencies.json"
source "$STAGE4_D_ROS_DEPS_CONTEXT_FILE"
source /opt/ros/jazzy/setup.bash
source "$STAGE4_ROS_INSTALL_PREFIX/setup.bash"
STAGE4_D_ROS_RED_RUN_ROOT="$(mktemp -d \
  "$PWD/build/stage4-d-ros-red.XXXXXX")"
bash packaging/run_network_isolated.sh env \
  CC="$STAGE4_CC" CXX="$STAGE4_CXX" \
  bash packaging/build_ros_overlay.sh \
  --lock packaging/locks/ros2-dependencies.lock \
  --source-cache-manifest packaging/locks/source-archive-cache.manifest.json \
  --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE" \
  --source-work "$STAGE4_D_ROS_RED_RUN_ROOT/source-work" \
  --livox-sdk-prefix "$STAGE4_D_ROS_RED_RUN_ROOT/livox-sdk-install" \
  --build-base "$STAGE4_D_ROS_RED_RUN_ROOT/build" \
  --project-source "$PWD/ros2" \
  --client-prefix "$PWD/build/stage4-d-install" \
  --install-base "$STAGE4_D_ROS_RED_RUN_ROOT/install"
source "$STAGE4_D_ROS_RED_RUN_ROOT/install/setup.bash"
bash packaging/run_network_isolated.sh colcon list \
  --base-paths "$PWD/ros2" --names-only
bash packaging/run_network_isolated.sh "$STAGE4_CTEST" \
  --test-dir "$STAGE4_D_ROS_RED_RUN_ROOT/build/slope_sim_bridge" \
  -N -R '^(test_bridge|test_exact_frame_joiner|test_topic_mapping|test_bridge_control_status)$' \
  --no-tests=error
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --ros-context-kind bridge_red \
  --parent-ros-context "$STAGE4_D_ROS_DEPS_CONTEXT_FILE" \
  --ros-run-root "$STAGE4_D_ROS_RED_RUN_ROOT" \
  --ros-source-work "$STAGE4_D_ROS_RED_RUN_ROOT/source-work" \
  --ros-livox-sdk-prefix "$STAGE4_D_ROS_RED_RUN_ROOT/livox-sdk-install" \
  --ros-build-base "$STAGE4_D_ROS_RED_RUN_ROOT/build" \
  --ros-install-prefix "$STAGE4_D_ROS_RED_RUN_ROOT/install" \
  --write-ros-build-context "$STAGE4_D_ROS_RED_CONTEXT_FILE" \
  --json "$STAGE4_D_ROS_RED_RUN_ROOT/context.json"
```

Run:

```bash
STAGE4_D_ROS_RED_CONTEXT_FILE="$PWD/build/stage4-d-ros-red-context.env"
D_ROS_CONTEXT_PREFLIGHT="$(mktemp -d \
  "$PWD/build/stage4-d-ros-context-preflight.XXXXXX")"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-ros-build-context "$STAGE4_D_ROS_RED_CONTEXT_FILE" \
  --expect-ros-context-kind bridge_red \
  --json "$D_ROS_CONTEXT_PREFLIGHT/bridge-red.json"
source "$STAGE4_D_ROS_RED_CONTEXT_FILE"
source /opt/ros/jazzy/setup.bash
source "$STAGE4_ROS_PARENT_INSTALL_PREFIX/setup.bash"
source "$STAGE4_ROS_INSTALL_PREFIX/setup.bash"
bash packaging/run_network_isolated.sh colcon list \
  --base-paths "$PWD/ros2" --names-only
bash packaging/run_network_isolated.sh "$STAGE4_CTEST" \
  --test-dir "$STAGE4_ROS_BUILD_BASE/slope_sim_bridge" \
  -N -R '^(test_bridge|test_exact_frame_joiner|test_topic_mapping|test_bridge_control_status)$' \
  --no-tests=error
```

Run:

```bash
STAGE4_D_ROS_RED_CONTEXT_FILE="$PWD/build/stage4-d-ros-red-context.env"
D_ROS_CONTEXT_PREFLIGHT="$(mktemp -d \
  "$PWD/build/stage4-d-ros-context-preflight.XXXXXX")"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-ros-build-context "$STAGE4_D_ROS_RED_CONTEXT_FILE" \
  --expect-ros-context-kind bridge_red \
  --json "$D_ROS_CONTEXT_PREFLIGHT/bridge-red.json"
source "$STAGE4_D_ROS_RED_CONTEXT_FILE"
source /opt/ros/jazzy/setup.bash
source "$STAGE4_ROS_PARENT_INSTALL_PREFIX/setup.bash"
source "$STAGE4_ROS_INSTALL_PREFIX/setup.bash"
bash packaging/run_network_isolated.sh "$STAGE4_CTEST" \
  --test-dir "$STAGE4_ROS_BUILD_BASE/slope_sim_bridge" \
  -R '^(test_bridge|test_exact_frame_joiner|test_topic_mapping|test_bridge_control_status)$' \
  --output-on-failure --no-tests=error
```

Expected: overlay build 返回 0，only-loopback attestation 有效，RED 的私有 SDK prefix/cache/link 全在本轮唯一 run root 且真实 `/usr/local` census 前后相同；已验证 `bridge_red` context 原子绑定该根与 dependency parent。`colcon list` 明确列出两个 package，`ctest -N` 恰好列出四个测试；最后一条 CTest 非零，且失败只来自 fallback 的 `UNIMPLEMENTED`/空转换/未 READY 行为断言。package 不存在、0 tests、unknown target、缺依赖、configure/typesupport/链接错误、poison/default `/usr/local` 命中或断网证据缺失都必须先修测试基础设施并原样重跑本 Step；重跑时首条命令必须创建新 run root，不能复用失败轮目录。

- [ ] **Step 4: 确认 Bridge RED 的失败原因正确**

逐个测试确认至少执行到目标断言，并分别覆盖转换、joiner、topic map、control status；不得只有一个 fallback 崩溃导致其余测试未执行。确认后才创建生产 header/source；若失败原因不符，修正测试壳并原样重跑 Step 3。

- [ ] **Step 5: 最小实现严格输入/输出 namespace 与消息转换**

Bridge 参数合同固定为 `input_prefix` 和 `output_namespace`：live 使用 `/sim` -> `/slope_sim`，replay 使用 `/replay/sim` -> `/replay/slope_sim`。参数必须是规范绝对 namespace，不接受 `..`、重复 `/`、尾 `/` 或把 replay 输出映射到 `/slope_sim`；每个 eCAL 输入 topic 和 ROS 输出 topic 由唯一纯函数生成。prefix 只改变 topic，不改变期望的完整 v2 type name、`proto` encoding 或 descriptor bytes。Bridge 的 eCAL callback 先复制本帧真实远端 metadata 和 raw bytes 形成 `RawEnvelope`，worker 完成 hash/metadata gate/parse 后才转换；合法 replay 与 live 通过同一个协议门，转换函数不得修改或重新序列化原始 payload。

每个已验证 worker 只把不可变模型交给 `ExactFrameJoiner`。join key 固定 `(simulation_session_id, world_generation, timestamp_ns)`；LiDAR/RTK/IMU 任一先到就建立最多 64 个 10 Hz anchor，WheelState 使用独立最多 256 项的 exact-time cache，因此 100 Hz 中不对应传感器采样的正常轮态不会制造不完整四槽。四种消息任意顺序到达，anchor 的 LiDAR/RTK/IMU 与同 key WheelState 齐全时原子取出并只发布一次。anchor 自首个 10 Hz 组件接收起 TTL 2 秒，缺槽/容量淘汰计入 Bridge health 并使正式门失败；无 anchor 的额外 WheelState 到期只增加 `wheel_unmatched_expired` 诊断。session 或 generation 前进时清理旧缓存，回退直接拒绝；所有淘汰都不能最近邻、补零或跨 generation 配对。

- [ ] **Step 6: 最小实现消息字段与时钟**

- `livox_ros_driver2/msg/CustomMsg` 按 lock 中规范化 `.msg` hash 逐字段映射：`header.stamp=ns_to_ros_time(timebase_ns)`、`header.frame_id="lidar_link"`、`timebase=timebase_ns`、`point_num=points.size()`、`lidar_id=1`、`rsvd=[0,0,0]`；每个 `CustomPoint.offset_time` 精确保留 uint32 ns，`x/y/z`、`reflectivity/tag/line` 逐项复制并做范围检查。
- `sensor_msgs/PointCloud2` 使用同一 stamp/frame，字段固定 `x:FLOAT32/y:FLOAT32/z:FLOAT32/intensity:FLOAT32/range:FLOAT32`，little-endian、dense=false；`range=sqrt(x*x+y*y+z*z)`，`intensity=float(reflectivity)`，point/row step 由字段布局唯一计算。
- static TF 固定 `base_link -> lidar_link` translation `(0,0,0.105)`。
- dynamic TF 只用 joiner 完成的 exact `PoseKey(session,generation,timestamp)` 的 RTK+IMU+WheelState；调用 Task 3 已测试的唯一 projected-lateral-heading -> ZYX rotation helper，再应用 canonical 车型外参，禁止把 RTK heading 直接当 Euler yaw。同 batch 先提交 TF/RTK marker/trajectory，再提交两种点云。
- 时钟 topic 固定为 `<output_namespace>/clock`：live 是 `/slope_sim/clock`，replay 是 `/replay/slope_sim/clock`。值使用最新仿真时间，暂停保持，replay 由回放器推进；RViz2/Bridge launcher 显式把 ROS 特殊 `/clock` remap 到该 topic，禁止两个模式共享全局 clock 后仍宣称 namespace 隔离。

项目 ROS topic 同样冻结：`<ns>/wheel/state`、`<ns>/rtk/state`、`<ns>/imu/attitude`、`<ns>/rtk/markers`、`<ns>/trajectory`。`RtkTriplet.msg` 保留 LEFT/CENTER/RIGHT 和 heading；`ImuAttitude.msg` 只含 roll/pitch，不虚构角速度、线加速度或协方差。

三种项目消息定义固定为：

```text
# WheelState.msg
std_msgs/Header header
string robot_model
float32[] drive_wheel_speed_rad_s
float32[] steering_wheel_angle_rad
uint64 sequence
uint64 world_generation
uint64 command_generation
uint8 command_authority_state
string command_owner_source_id
uint8[16] command_owner_source_session_id
uint32 command_peer_count
uint8[16] simulation_session_id
uint8[32] descriptor_sha256

# RtkTriplet.msg
std_msgs/Header header
geometry_msgs/Point left
geometry_msgs/Point center
geometry_msgs/Point right
float64 heading_rad
uint64 sequence
uint64 world_generation
uint8[16] simulation_session_id
uint8[32] descriptor_sha256

# ImuAttitude.msg
std_msgs/Header header
float64 roll_rad
float64 pitch_rad
uint64 sequence
uint64 world_generation
uint8[16] simulation_session_id
uint8[32] descriptor_sha256
```

三个 header stamp 都由源 `timestamp_ns` 精确转换；WheelState frame 为 `base_link`，RTK 为 `world`，IMU 为 `base_link`。非 ACTIVE WheelState 的 owner id 为空且 owner session 为 16 个零字节；ACTIVE 必须保留原 owner。转换测试逐字段与原 v2 模型比对，并拒绝数组长度、identity 长度和 authority/owner 不一致。

- [ ] **Step 7: 最小实现 Bridge control status 与进程所有权**

`bridge_node.cpp` 是 `BRIDGE` role 的唯一 owner，通过 C Task 7 的 control codec/socket 上报四个输入 topic 的完整 `TopicHealth`。启动参数显式给出 expected session、descriptor、`input_prefix`、`output_namespace` 和 control socket；control 身份校验后立即发送 `BRIDGE/STARTING`，并在该 state 持续更新四门 WAITING/PENDING/VERIFIED health。节点再建立四个 raw eCAL subscribers，按 C 的 discovery snapshot 规则等待每个规范输入 topic 恰有一个匹配 type/encoding/descriptor 的 endpoint。四门全部 `VERIFIED` 前不得发送 READY，STARTING 也绝不打开 Simulator/Replay 业务门；CONFLICT、回退、重复/缺槽、control socket 断开或 ROS publish 错误立即发送 FAILED 并让 Bridge 非零退出，但不能停止 Simulator/Recorder。

四门同时 VERIFIED 后只允许 `STARTING -> READY` 并发送唯一 `BRIDGE/READY`，正式编排器收到前不得让 Simulator 占 sequence 0 或让 Replay unpause/step。首个 exact-time 四槽 batch 成功提交 ROS 后转 ACTIVE；STARTING/READY/ACTIVE 都携带四个唯一 TopicHealth、累计 accepted/rejected、joiner `incomplete_expired/wheel_unmatched_expired` 和 last clock。live/replay 只改变规范 prefix/namespace，不改变 role 或状态机。`test_bridge_control_status` 覆盖 24 种 gate 到达顺序、STARTING health、READY 只发一次、四门缺一、peer count 0/2、descriptor conflict、READY 后断线/回退、ACTIVE 后 ROS publish 失败，并断言编排器不能代发 Bridge STARTING/READY。

- [ ] **Step 8: 固定 RViz2 配置**

live 配置的 PointCloud2 topic 为 `/slope_sim/lidar/points`，replay 配置只换为 `/replay/slope_sim/lidar/points` 和对应 clock/marker；启动参数分别 remap `/clock:=/slope_sim/clock` 与 `/clock:=/replay/slope_sim/clock`。两者 Fixed Frame 均为 `world`，按 `range` 使用可辨近远色带，点大小适合 MID-360 表面点，同时显示 TF、三点 RTK marker 和轨迹，不把说明文字放进主视图遮挡点云。

- [ ] **Step 9: 运行 ROS GREEN**

Run: `"$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run:

```bash
STAGE4_D_ROS_DEPS_CONTEXT_FILE="$PWD/build/stage4-d-ros-deps-context.env"
STAGE4_D_ROS_GREEN_CONTEXT_FILE="$PWD/build/stage4-d-ros-green-context.env"
D_ROS_CONTEXT_PREFLIGHT="$(mktemp -d \
  "$PWD/build/stage4-d-ros-context-preflight.XXXXXX")"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-ros-build-context "$STAGE4_D_ROS_DEPS_CONTEXT_FILE" \
  --expect-ros-context-kind dependencies \
  --json "$D_ROS_CONTEXT_PREFLIGHT/dependencies.json"
source "$STAGE4_D_ROS_DEPS_CONTEXT_FILE"
source /opt/ros/jazzy/setup.bash
source "$STAGE4_ROS_INSTALL_PREFIX/setup.bash"
STAGE4_D_ROS_GREEN_RUN_ROOT="$(mktemp -d \
  "$PWD/build/stage4-d-ros-green.XXXXXX")"
bash packaging/run_network_isolated.sh env \
  CC="$STAGE4_CC" CXX="$STAGE4_CXX" \
  bash packaging/build_ros_overlay.sh \
  --lock packaging/locks/ros2-dependencies.lock \
  --source-cache-manifest packaging/locks/source-archive-cache.manifest.json \
  --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE" \
  --source-work "$STAGE4_D_ROS_GREEN_RUN_ROOT/source-work" \
  --livox-sdk-prefix "$STAGE4_D_ROS_GREEN_RUN_ROOT/livox-sdk-install" \
  --build-base "$STAGE4_D_ROS_GREEN_RUN_ROOT/build" \
  --project-source "$PWD/ros2" \
  --client-prefix "$PWD/build/stage4-d-install" \
  --install-base "$STAGE4_D_ROS_GREEN_RUN_ROOT/install"
source "$STAGE4_D_ROS_GREEN_RUN_ROOT/install/setup.bash"
bash packaging/run_network_isolated.sh "$STAGE4_CTEST" \
  --test-dir "$STAGE4_D_ROS_GREEN_RUN_ROOT/build/slope_sim_bridge" \
  -N -R '^(test_bridge|test_exact_frame_joiner|test_topic_mapping|test_bridge_control_status)$' \
  --no-tests=error
bash packaging/run_network_isolated.sh "$STAGE4_CTEST" \
  --test-dir "$STAGE4_D_ROS_GREEN_RUN_ROOT/build/slope_sim_bridge" \
  -R '^(test_bridge|test_exact_frame_joiner|test_topic_mapping|test_bridge_control_status)$' \
  --output-on-failure --no-tests=error
bash packaging/run_network_isolated.sh colcon test-result \
  --test-result-base "$STAGE4_D_ROS_GREEN_RUN_ROOT/build" --verbose
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --ros-context-kind bridge_green \
  --parent-ros-context "$STAGE4_D_ROS_DEPS_CONTEXT_FILE" \
  --ros-run-root "$STAGE4_D_ROS_GREEN_RUN_ROOT" \
  --ros-source-work "$STAGE4_D_ROS_GREEN_RUN_ROOT/source-work" \
  --ros-livox-sdk-prefix "$STAGE4_D_ROS_GREEN_RUN_ROOT/livox-sdk-install" \
  --ros-build-base "$STAGE4_D_ROS_GREEN_RUN_ROOT/build" \
  --ros-install-prefix "$STAGE4_D_ROS_GREEN_RUN_ROOT/install" \
  --write-ros-build-context "$STAGE4_D_ROS_GREEN_CONTEXT_FILE" \
  --json "$STAGE4_D_ROS_GREEN_RUN_ROOT/context.json"
```

Run:

```bash
STAGE4_D_ROS_GREEN_CONTEXT_FILE="$PWD/build/stage4-d-ros-green-context.env"
D_ROS_CONTEXT_PREFLIGHT="$(mktemp -d \
  "$PWD/build/stage4-d-ros-context-preflight.XXXXXX")"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-ros-build-context "$STAGE4_D_ROS_GREEN_CONTEXT_FILE" \
  --expect-ros-context-kind bridge_green \
  --json "$D_ROS_CONTEXT_PREFLIGHT/bridge-green.json"
source "$STAGE4_D_ROS_GREEN_CONTEXT_FILE"
source /opt/ros/jazzy/setup.bash
source "$STAGE4_ROS_PARENT_INSTALL_PREFIX/setup.bash"
source "$STAGE4_ROS_INSTALL_PREFIX/setup.bash"
bash packaging/run_network_isolated.sh "$STAGE4_CTEST" \
  --test-dir "$STAGE4_ROS_BUILD_BASE/slope_sim_bridge" \
  -N -R '^(test_bridge|test_exact_frame_joiner|test_topic_mapping|test_bridge_control_status)$' \
  --no-tests=error
bash packaging/run_network_isolated.sh "$STAGE4_CTEST" \
  --test-dir "$STAGE4_ROS_BUILD_BASE/slope_sim_bridge" \
  -R '^(test_bridge|test_exact_frame_joiner|test_topic_mapping|test_bridge_control_status)$' \
  --output-on-failure --no-tests=error
bash packaging/run_network_isolated.sh colcon test-result \
  --test-result-base "$STAGE4_ROS_BUILD_BASE" --verbose
```

Expected: 四个 CTest 全部 PASS；GREEN builder 与所有 ROS test 的 only-loopback attestation 有效，私有 SDK prefix/cache/link 全在本轮唯一 run root，真实 `/usr/local` census 前后相同，`bridge_green` context 原子绑定本轮 tree digest 与 dependency parent；24 种四 topic 到达排列都只形成一次相同 batch，100 Hz WheelState 不耗尽 64 个 10 Hz anchor，anchor TTL/容量/重复/session-generation 淘汰可审计且正式 fixture 中为零；Bridge 先以 STARTING 上报四门 pre-READY health 且不打开业务门，四门全 VERIFIED 后才有唯一 READY，首 batch 后 ACTIVE，冲突/断线/ROS publish 错误转 FAILED；live/replay 输入和输出 namespace 互斥，合法 replay 原始 publisher metadata 通过 Bridge 严格门且 type/encoding/descriptor 任一错误都被拒绝，生产 eCAL/ROS namespace 在 replay 测试中均为零新增，Livox `header/timebase/point_num/lidar_id/rsvd/points` 全字段与锁定依赖一致。

- [ ] **Step 10: REFACTOR 或记录无必要**

只整理转换/joiner/control status/ROS publisher 间已出现的重复；若无必要，记录“REFACTOR：无必要”。不得把 fallback 装入生产包或把四门压成不可审计的单一布尔值。

- [ ] **Step 11: 原样复验**

原样重跑 Step 9 的三条命令；首条命令必须再次 `mktemp` 一个与 GREEN 首轮不同的全新 run root，成功后原子替换 `bridge_green` context，后续 Task 6 只消费这份复验 context。不得改用 C 前缀、清空/复用首轮输出、复用其他轮次 Livox prefix、移除断网 wrapper、使用裸 `colcon build/test` 旁路、缩短四测试正则或移除 `--no-tests=error`。

## Task 6：真实 RViz2、Replay 与 Livox Viewer 门禁

**Files:**
- Create: `scripts/verify_stage4_ros.py`
- Create: `scripts/verify_stage4_exports.py`
- Create: `scripts/stage4_session_evidence.py`
- Create: `docs/阶段四点云显示手动测试教程.md`
- Modify: `docs/阶段四交付报告.md`
- Modify: `scripts/verify_stage4_dependencies.py`
- Test: `tests/stage4/test_stage4_ros_verifier.py`
- Test: `tests/stage4/test_stage4_export_verifier.py`

- [ ] **Step 1: 用 fixture 写 ROS/export verifier RED**

测试通过 subprocess 驱动 wished-for CLI，并在每个测试函数内先断言三个脚本存在；不得顶层 import 尚未创建的模块，也不得在 fixture setup 中因缺文件报 ERROR。正例 fixture 固定要求显式绝对 `--client-prefix`、`--ros-prefix`、`--simulator-entry`、`--subscriber-binary`、`--command-binary`、`--recorder-binary`、`--replay-binary`、`--bridge-binary`、`--rviz2-binary`、全新绝对空 `--evidence-root`，以及该根内尚不存在且 basename 精确为 `result.json` 的 `--output`；并冻结 live/replay topic 集、TF/frame、点数、clock、每个真实 role 自己发送的 STARTING -> READY -> ACTIVE 顺序和 LVX2 sidecar。复用同一 evidence root、输出在根外/已存在、根中预置旧 JSONL/rosbag/control 或用固定 `results/stage4/ros-*.json` 都必须在 spawn 前失败。

路径反例逐一覆盖：任一 prefix/entry/ELF 为相对路径；PATH 前置同名假程序；正确名字或 hash 的 ELF 位于声明 client/ROS root 外；root 内 symlink 逃逸；四个 C++ role 互换；Bridge 不在 ROS overlay；RViz2 不等于 ROS lock/dpkg 冻结绝对路径/hash；binary 在验证后被替换；Conda/build/C 前缀 DSO 泄漏。证据反例再覆盖缺 TF、错 frame、少点、生产 topic 污染、sidecar 数量不一致，以及 C evidence 中相对/`.partial` manifest 路径、64 位 SHA 非法、文件替换或 hash 不符。lifecycle fixture 要求 live 的 Simulator/Subscriber/Command/Recorder/Bridge 与 replay 的 Replay/Bridge 都先各自发送 STARTING；READY 前 WAITING/PENDING/VERIFIED/CONFLICT TopicHealth 只能由该 role 的 STARTING 帧承载且业务计数恒为 0，全部必要门 VERIFIED 后才允许 READY，首条业务后才允许 ACTIVE。逐项反例覆盖缺 STARTING、编排器代发 role 状态、STARTING 时发布、未全 VERIFIED 就 READY、跳过 READY、READY/ACTIVE 后回退。live 命令在五个 role READY 和 RViz2 ROS graph ready 全部到齐前发布计数恒为 0；replay 命令在 Replay、Bridge、RViz2 ready 前不 unpause/step，并真实产生 pause hold、单步一次、0.5x、2x 与对应 clock 证据。

- [ ] **Step 2: 运行 verifier RED**

Run: `bash packaging/run_network_isolated.sh conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_ros_verifier.py tests/stage4/test_stage4_export_verifier.py`

Expected: pytest 正常收集并 `FAILED`，首个失败断言明确指出 verifier/evidence CLI 尚未实现；不得启动 eCAL、ROS、RViz2 或 Livox Viewer，也不得出现 collection error、fixture error 或 skip。

- [ ] **Step 3: 确认 verifier RED 的失败原因正确**

首个失败必须明确指向 wished-for verifier/evidence/path/launch-order 行为尚未实现；collection/fixture error、脚本路径拼错、skip 或意外启动真实 eCAL/ROS/RViz2 都不算 RED。修正测试壳后原样重跑 Step 2。

- [ ] **Step 4: 最小实现 verifier 与 evidence/可执行文件边界**

`stage4_session_evidence.py` 只接受 C 已冻结的 `final_session_manifest_path/final_session_manifest_sha256`，在每次消费前重新 resolve、hash 并用对应 C runtime manifest 完整读取，不从目录或文件名猜 session。每个 prefix/binary 先 `resolve(strict=True)`，拒绝 symlink/非普通文件，并以已打开 fd 的 device/inode/hash 绑定启动前后证据；四个 C++ ELF 必须是 D client prefix runtime manifest 中对应 role 的绝对普通文件，Bridge 必须属于 ROS overlay manifest，RViz2 必须与 ROS lock 中绝对路径、dpkg 版本和 hash 完全一致。进程只用已验证绝对 argv 启动，PATH 不参与选择。

ROS verifier 只在显式绝对空 `--evidence-root` 中创建并解析本次运行的规范 JSONL/rosbag/control 证据并验证 topic/TF/clock/health；`--output` 必须是该根内唯一 `result.json`，所有路径在创建任何 child 前经 dirfd/no-follow 复核。它为每个实际 spawn 的 role 建立独立 lifecycle oracle，要求首帧 STARTING、STARTING 期间零业务、必要 TopicHealth 全部 VERIFIED 后唯一 READY、首条业务后 ACTIVE，并拒绝跳态、回退或编排器代发。export verifier 只调用显式绝对 `--export-binary`，再独立解析 LVX2 sidecar 和源 session identity。三个脚本均原子写唯一 result JSON，拒绝旧根/旧输出。`verify_stage4_dependencies.py --frozen-session-prefix` 只验证 C prefix/runtime manifest/ELF hash，不写该树；D provenance 只写 `build/stage4-d-install`。

真实门禁由 verifier 用结构化 subprocess argv 直接管理，不创建、不 import、不执行 E 才拥有的正式 launcher；D 完成后 E 才能创建 launcher 并消费这些已验证参数合同。verifier 是测试证据采集器，不安装为用户 launcher。ROS verifier 自身及其全部子进程必须继承并复核外层 wrapper 的 only-loopback network namespace，任何 child 逃回宿主 netns、出现 default route/非 loopback interface 或证据漂移都失败。

- [ ] **Step 5: 运行 verifier GREEN**

Run: `bash packaging/run_network_isolated.sh conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_ros_verifier.py tests/stage4/test_stage4_export_verifier.py`

Expected: PASS；相对路径、PATH shadow、root 外/逃逸 ELF、role 互换、RViz2 漂移、STARTING/READY/ACTIVE 顺序、replay 控制及全部证据反例精确失败，正例 fixture 通过。

- [ ] **Step 6: REFACTOR 或记录无必要**

只整理已经通过 fixture 的路径绑定、证据解析和进程编排重复；若无必要，记录“REFACTOR：无必要”。不得引入 E launcher 依赖或 PATH fallback。

- [ ] **Step 7: 原样复验 verifier**

原样重跑 Step 5 命令；不得删除攻击 fixture、放宽 root/hash/READY/control 断言或启动真实组件。

- [ ] **Step 8: 构建并冻结 D 工具前缀**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target stage4-selftest-session slope-sim-sub slope-sim-command slope-sim-record slope-sim-replay slope-sim-export && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-d-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-d-install" --mode sdk`

Run: `install -d "$PWD/results/stage4" && conda run -n slope-sim python scripts/verify_stage4_dependencies.py --install-prefix "$PWD/build/stage4-d-install" --frozen-session-prefix "$PWD/build/stage4-dev-install" --build-kind development --write-runtime-manifest "$PWD/build/stage4-d-install/share/slope-sim/runtime-manifest.json" --json "$PWD/results/stage4/d-runtime.json"`

Expected: D manifest 绑定 self-test/Subscriber/Command/Recorder/Replay/Export 和同前缀依赖的绝对 hash/ABI；C frozen prefix 的 runtime manifest 与其列出的 ELF hash 全部复核一致且未被写入。verifier 拒绝 D ELF 落在 C/build/Conda/PATH、任一 DSO 解析到 D prefix 或系统 allowlist 外，以及 ROS/RViz lock 漂移。

- [ ] **Step 9: 为 live eCAL/RViz2 invocation 单独取得授权并即时预检**

向用户说明将启动真实 eCAL、Simulator、Subscriber、Command、Recorder、Bridge 和 RViz2 10 秒；该授权只覆盖下一条 live 命令。即时扫描 pytest、PyBullet、GUI/Xvfb、eCAL/C++ participant 和系统负载；存在竞争负载时等待并重新确认，不能先消费授权后换时间运行。

- [ ] **Step 10: 启动全部 live 组件并在 READY 后验证实时显示**

Run:

```bash
STAGE4_D_ROS_GREEN_CONTEXT_FILE="$PWD/build/stage4-d-ros-green-context.env"
install -d "$PWD/results/stage4/ros-live"
STAGE4_D_ROS_LIVE_EVIDENCE_ROOT="$(mktemp -d \
  "$PWD/results/stage4/ros-live/run.XXXXXX")"
STAGE4_D_ROS_LIVE_RUNTIME_ROOT="$STAGE4_D_ROS_LIVE_EVIDENCE_ROOT/runtime"
install -d "$STAGE4_D_ROS_LIVE_RUNTIME_ROOT"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-ros-build-context "$STAGE4_D_ROS_GREEN_CONTEXT_FILE" \
  --expect-ros-context-kind bridge_green \
  --json "$STAGE4_D_ROS_LIVE_EVIDENCE_ROOT/context-preflight.json"
source "$STAGE4_D_ROS_GREEN_CONTEXT_FILE"
source /opt/ros/jazzy/setup.bash
source "$STAGE4_ROS_PARENT_INSTALL_PREFIX/setup.bash"
source "$STAGE4_ROS_INSTALL_PREFIX/setup.bash"
env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD \
  bash packaging/run_network_isolated.sh \
  conda run -n slope-sim python scripts/verify_stage4_ros.py \
  --mode live \
  --client-prefix "$PWD/build/stage4-d-install" \
  --ros-prefix "$STAGE4_ROS_INSTALL_PREFIX" \
  --simulator-entry "$PWD/scripts/ecal_simulation_runtime.py" \
  --subscriber-binary "$PWD/build/stage4-d-install/bin/slope-sim-sub" \
  --command-binary "$PWD/build/stage4-d-install/bin/slope-sim-command" \
  --recorder-binary "$PWD/build/stage4-d-install/bin/slope-sim-record" \
  --replay-binary "$PWD/build/stage4-d-install/bin/slope-sim-replay" \
  --bridge-binary \
    "$STAGE4_ROS_INSTALL_PREFIX/lib/slope_sim_bridge/slope_sim_bridge_node" \
  --rviz2-binary "$STAGE4_RVIZ2" \
  --runtime-manifest \
    "$PWD/build/stage4-d-install/share/slope-sim/runtime-manifest.json" \
  --input-prefix /sim --output-namespace /slope_sim \
  --clock-topic /slope_sim/clock \
  --rviz-config "$PWD/resources/rviz/slope_sim_mid360.rviz" \
  --wait-ready-roles simulator,subscriber,command,recorder,bridge \
  --require-rviz-ready --duration-sec 10 \
  --evidence-root "$STAGE4_D_ROS_LIVE_RUNTIME_ROOT" \
  --output "$STAGE4_D_ROS_LIVE_RUNTIME_ROOT/result.json"
```

Expected: verifier 及全部 child 的 only-loopback attestation 有效且没有进程逃回宿主 netns；本次授权只写新建的 `STAGE4_D_ROS_LIVE_EVIDENCE_ROOT`，result/context/JSONL/rosbag/control 证据都不得落到固定旧文件，失败根保留诊断，任何重试重新授权并新建另一根。它实际启动 Simulator、Subscriber、Command、Recorder、Bridge 和锁定 RViz2。五个 control role 各自先发 STARTING 并只在该 state 报告 READY 前 health，STARTING 期间 Simulator publish/sequence count 恒为 0；五个 role READY 且 RViz2 ROS graph ready 后才开门，首条业务后各 role 转 ACTIVE。Livox CustomMsg、PointCloud2、TF、RTK、`/slope_sim/clock` 都存在且 RViz `/clock` remap 生效；地面/坡体/障碍表面可辨。Bridge 注入退出后 Simulator/Recorder 继续健康，编排器记录 Bridge FAILED 并有序结束本门禁；交付报告记录本轮 exact evidence root/hash，不猜 `results/stage4` 下的文件名。

- [ ] **Step 11: 为 replay eCAL/RViz2 invocation 重新取得授权并即时预检**

只有 live 已通过才申请 replay 授权，不得复用 Step 9 的授权；再次说明只启动 Replay、Bridge、RViz2，并且只使用 `/replay/sim` 输入和 `/replay/slope_sim` 输出。失败保留证据并停止，不自动再跑。

- [ ] **Step 12: 启动 Replay/Bridge/RViz2 并实际验证控制与 clock**

Run:

```bash
STAGE4_D_ROS_GREEN_CONTEXT_FILE="$PWD/build/stage4-d-ros-green-context.env"
install -d "$PWD/results/stage4/ros-replay"
STAGE4_D_ROS_REPLAY_EVIDENCE_ROOT="$(mktemp -d \
  "$PWD/results/stage4/ros-replay/run.XXXXXX")"
STAGE4_D_ROS_REPLAY_RUNTIME_ROOT="$STAGE4_D_ROS_REPLAY_EVIDENCE_ROOT/runtime"
install -d "$STAGE4_D_ROS_REPLAY_RUNTIME_ROOT"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-ros-build-context "$STAGE4_D_ROS_GREEN_CONTEXT_FILE" \
  --expect-ros-context-kind bridge_green \
  --json "$STAGE4_D_ROS_REPLAY_EVIDENCE_ROOT/context-preflight.json"
source "$STAGE4_D_ROS_GREEN_CONTEXT_FILE"
source /opt/ros/jazzy/setup.bash
source "$STAGE4_ROS_PARENT_INSTALL_PREFIX/setup.bash"
source "$STAGE4_ROS_INSTALL_PREFIX/setup.bash"
env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD \
  bash packaging/run_network_isolated.sh \
  conda run -n slope-sim python scripts/verify_stage4_ros.py \
  --mode replay \
  --client-prefix "$PWD/build/stage4-d-install" \
  --ros-prefix "$STAGE4_ROS_INSTALL_PREFIX" \
  --simulator-entry "$PWD/scripts/ecal_simulation_runtime.py" \
  --subscriber-binary "$PWD/build/stage4-d-install/bin/slope-sim-sub" \
  --command-binary "$PWD/build/stage4-d-install/bin/slope-sim-command" \
  --recorder-binary "$PWD/build/stage4-d-install/bin/slope-sim-record" \
  --replay-binary "$PWD/build/stage4-d-install/bin/slope-sim-replay" \
  --bridge-binary \
    "$STAGE4_ROS_INSTALL_PREFIX/lib/slope_sim_bridge/slope_sim_bridge_node" \
  --rviz2-binary "$STAGE4_RVIZ2" \
  --input-prefix /replay/sim --output-namespace /replay/slope_sim \
  --clock-topic /replay/slope_sim/clock \
  --rviz-config "$PWD/resources/rviz/slope_sim_mid360_replay.rviz" \
  --session-evidence "$PWD/results/stage4/cpp-gate-4plus2.json" \
  --start-paused \
  --exercise-replay-control pause \
  --exercise-replay-control step=1 \
  --exercise-replay-control rate=0.5 \
  --exercise-replay-control rate=2.0 \
  --wait-ready-roles replay,bridge \
  --require-rviz-ready --duration-sec 10 \
  --evidence-root "$STAGE4_D_ROS_REPLAY_RUNTIME_ROOT" \
  --output "$STAGE4_D_ROS_REPLAY_RUNTIME_ROOT/result.json"
```

Expected: verifier 及全部 child 的 only-loopback attestation 有效且没有进程逃回宿主 netns；本次独立授权只写新建的 `STAGE4_D_ROS_REPLAY_EVIDENCE_ROOT`，失败根保留且任何重试重新授权并新建另一根，交付报告记录 exact root/hash。它只实际启动 Replay、Bridge 和锁定 RViz2。Replay 与 Bridge 各自先发 STARTING，STARTING 只承载 READY 前 health 且保持 paused/零业务；两者 READY、RViz2 graph ready 后才允许控制，首条回放业务后转 ACTIVE。随后 pause 区间 clock 完全保持，`step=1` 只发布下一同 timestamp batch并保持 paused，0.5x/2x 的 wall-to-clock 斜率分别在容差内且切换时不 burst；Replay status clock 与 `/replay/slope_sim/clock` 精确一致。Bridge 逐 topic 报告远端完整 type name、`proto` encoding 和 descriptor digest 已验证，原始 payload/hash 与 MCAP 一致；生产 eCAL `/sim/...` 和 ROS `/slope_sim/...` 均零新增，默认 WheelCommand publisher 不存在。

- [ ] **Step 13: 用 D 导出器导出并人工打开 LVX2**

Run: `install -d "$PWD/results/stage4/livox-viewer" && STAGE4_LIVOX_VIEWER_ROOT="$(mktemp -d -p "$PWD/results/stage4/livox-viewer" run.XXXXXX)" && conda run -n slope-sim python scripts/verify_stage4_exports.py --format lvx2 --client-prefix "$PWD/build/stage4-d-install" --session-evidence "$PWD/results/stage4/cpp-gate-4plus2.json" --export-binary "$PWD/build/stage4-d-install/bin/slope-sim-export" --output-dir "$STAGE4_LIVOX_VIEWER_ROOT" --result-json "$STAGE4_LIVOX_VIEWER_ROOT/livox-viewer-export.json"`

Expected: D 安装树生成 `.lvx2` 与 sidecar；用户在 Livox Viewer 2 实际打开，能辨认地面和障碍表面。教程逐项记录软件版本、截图、点数、颜色观察和异常；未由用户确认前报告保持“人工门禁未完成”。
