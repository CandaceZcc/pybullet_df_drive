# 阶段四 C：C++ eCAL SDK 与 Recorder Implementation Plan

> **Execution:** Use `subagent-driven-development` only when the user selects delegated execution; otherwise use `executing-plans`. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 交付可给同事直接使用的 C++17 eCAL Subscriber SDK、只读 CLI、独立 Command Tool 和无损 MCAP Recorder，并证明五个正式 topic 的原始 bytes、会话身份、序列和窗口消息三方一致。

**Architecture:** eCAL native callback 只复制 payload/完整远端 type metadata、`send_timestamp/send_clock` 和接收墙钟，hash、解析和用户回调都在 worker；只读客户端使用有界 owner+latest，Recorder 使用共享 message+byte reservation ledger 与按 `record_order` 连续推进的 ordered-commit ledger。MCAP 中业务 channel 保持原始 eCAL bytes，另用一一配对的 record-metadata channel 保存逐消息身份；segment manifest、轮转 barrier 与最终 end barrier 共同定义完整会话。所有组件通过本地 control socket 汇报逐话题协议健康和 STARTING/READY/ACTIVE/ROTATING/DRAINING/FAILED/FINALIZED，正常结束先保持 100 Hz 零命令 drain；Command 发布并报告唯一最终零命令 fence 后冻结 publisher，Simulator 再取得其余四话题 post-window fence，最后才发送 end barrier 并落盘。

**Tech Stack:** C++17、GCC 13、CMake 3.28、eCAL 6.1.1 C++ SDK、Protobuf CMake package 33.6.0（release v33.6，`protoc --version` 输出 33.6）、MCAP C++、Zstd、GoogleTest、OpenSSL/libcrypto SHA-256。

---

**TDD gate:** 本计划所有生产代码任务遵守总路线的严格 RED-GREEN-REFACTOR 协议；RED 必须是 pytest/CTest 正常收集后的行为断言失败，不能是 configure/collection error、缺工具、skip 或缺构建目录。Python RED 只在测试函数内 import 尚未创建的模块，或通过 subprocess 调用 wished-for CLI，并把缺模块/脚本转换为明确 `pytest.fail`；C++ RED 必须先注册 target 并成功 configure，新增 API 才允许首次因测试引用的 API 尚不存在而编译失败。

**执行前置：** 总计划 Task 2 必须已经生成 `packaging/locks/cpp-dependencies.lock` 并构建开发 dependency prefix。开始或恢复本计划前必须独立复核并 source 环境合同，不能依赖 A/B 所在 shell：

```bash
test -n "${STAGE4_BUILD_ENV_FILE:-}"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-env "$STAGE4_BUILD_ENV_FILE" \
  --json "$STAGE4_BUILD_ENV_FILE.stage4-c-preflight.json"
source "$STAGE4_BUILD_ENV_FILE"
test -x "$STAGE4_CMAKE" && test -x "$STAGE4_CTEST"
test -x "$STAGE4_CC" && test -x "$STAGE4_CXX" && test -x "$STAGE4_PROTOC"
test -d "$STAGE4_DEPENDENCY_PREFIX"
```

Expected: env/evidence hash、工具版本、lock/source digest 和开发 dependency prefix 一致。本计划只消费该前缀；缺失或 lock hash 不一致时 configure 硬失败，不能把依赖获取推迟到 E，也不能使用 `FetchContent` 联网补齐。

## Task 1：C++ 工程、ABI 与安装接口

**Files:**
- Create: `CMakeLists.txt`
- Create: `CMakePresets.json`
- Create: `cpp/CMakeLists.txt`
- Create: `cpp/cmake/Stage4Dependencies.cmake`
- Create: `cpp/cmake/SlopeSimClientConfig.cmake.in`
- Create: `cpp/include/slope_sim/client/version.hpp.in`
- Create: `cpp/src/client/version.cpp`
- Create: `packaging/stage_cpp_runtime.sh`
- Create: `cpp/tests/test_build_contract.cpp`
- Create: `cpp/examples/version_consumer/CMakeLists.txt`
- Create: `cpp/examples/version_consumer/main.cpp`
- Create: `tests/stage4/test_cpp_install_contract.py`
- Modify: `scripts/verify_stage4_dependencies.py`

- [ ] **Step 1: 建立可配置测试壳并写 ABI 单元 RED**

```cpp
TEST(BuildContract, UsesFrozenVersions) {
  EXPECT_STREQ(SLOPE_SIM_ECAL_VERSION, "6.1.1");
  EXPECT_STREQ(SLOPE_SIM_PROTOBUF_VERSION, "33.6.0");
  EXPECT_EQ(SLOPE_SIM_GLIBCXX_CXX11_ABI, 1);
  EXPECT_EQ(__cplusplus, 201703L);
}
```

先只建立足以让 `stage4-dev` configure/build 成功的顶层 CMake、preset、`cpp/CMakeLists.txt` 和 `test_build_contract` target；测试源在目标 version header 不存在时使用测试内 fallback `UNIMPLEMENTED`/`0`，不得先创建生产 `version.hpp.in`。这一步只有测试基础设施，不接入正式依赖、不创建 SDK API，也不通过测试。CTest 必须已经能发现唯一的 `build_contract`，不能用 unknown target、configure 失败或缺构建目录冒充 RED。

- [ ] **Step 2: 证明 CTest 已注册并观察 ABI 行为 RED**

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^build_contract$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_build_contract && "$STAGE4_CTEST" --preset stage4-dev -R '^build_contract$' --output-on-failure --no-tests=error`

Expected: configure 和 build 均成功，CTest 发现 1 个测试并因冻结版本/ABI 断言不匹配而 `FAILED`；失败不得来自依赖、编译、链接或测试发现。

- [ ] **Step 3: 确认 ABI RED 的失败原因正确**

逐条核对 CTest 日志：失败测试只能是 `build_contract`，至少一条差异必须来自 `6.1.1`、`33.6.0`、ABI=1 或 C++17 占位值；若是 0 tests、target 不存在、依赖查找、编译或动态链接失败，先修测试壳并原样重跑 Step 2，不能进入实现。

- [ ] **Step 4: 最小实现严格依赖与版本入口**

```cmake
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)
find_package(Protobuf 33.6.0 EXACT REQUIRED CONFIG)
find_package(eCAL 6.1.1 EXACT REQUIRED CONFIG)
```

创建最小 `version.cpp`，用实际探测值替换 Step 1 的占位常量；生成头中的 Protobuf 合同必须是完整语义版本 `33.6.0`。CMake configure 同时拒绝 GCC 非 13、C++ eCAL 非 6.1.1、Protobuf 非 33.6.0、缺固定 MCAP/Zstd source lock 和启用网络 FetchContent。只实现本 Task 的构建/安装合同，不提前添加 Client、Recorder 或 CLI 行为。

`CMakePresets.json` 的 `stage4-dev` 和 `stage4-release` 都把以下环境变量作为必填绝对路径，不提供 PATH fallback：`STAGE4_CMAKE`、`STAGE4_CTEST`、`STAGE4_CC`、`STAGE4_CXX`、`STAGE4_PROTOC`、`STAGE4_DEPENDENCY_PREFIX`、`STAGE4_CMAKE_PREFIX_PATH`。preset 把 compiler、prefix 和 `STAGE4_PROTOC_EXECUTABLE` 显式传给 CMake；configure 再校验当前 CMake/CTest 均为 3.28.x、GCC/G++ 均为 13、dependency prefix 的 lock SHA 与总计划证据一致。

两个 Ninja preset 都冻结 artifact 布局：runtime 输出到 `${binaryDir}/bin`，shared/archive library 输出到 `${binaryDir}/lib`，生成的 C++ Protobuf 只在 `${binaryDir}/generated`。Task 1 先提供复用的 `slope_sim_add_proto()` CMake helper 并只接入 A 已存在的 interface v2；Tasks 6/7 在创建 record/control `.proto` 后再分别调用同一 helper，因此早期 configure 不引用尚不存在的文件。helper 统一使用 config-package 的 `protobuf_generate()`，并用 `STAGE4_PROTOC_EXECUTABLE` 复核 imported `protobuf::protoc`；Python 生成脚本只写受版本控制的 `pb2.py` 和 descriptor，不另造 build tree。该布局只供 target/test 定位；C 后续 CLI smoke 必须先 `"$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-dev-install"` 并从该 install tree 运行。C 完成后该前缀冻结为 D 的只读会话输入，D 工具另装到 `build/stage4-d-install`；release 打包同样只消费 install tree，不从生成器私有 build 目录复制 ELF。

MCAP/Zstd 只从 `packaging/locks/cpp-dependencies.lock` 指向的本地、已校验 source tree 构建；所有项目 target 使用 hidden visibility，共享库设置 `VERSION/SOVERSION` 和安装 RUNPATH `$ORIGIN` 或 `$ORIGIN/../lib`。

eCAL `v6.1.1` 最小 SDK 关闭 apps/Qt/HDF5/Curl/FTXUI/samples/tests/Python/C#/C binding，先尝试 `ECAL_USE_PROTOBUF=OFF` 的 raw core；若 monitoring/type metadata 因此不可用，则把 eCAL 与项目统一构建到同一个 Protobuf 33.6.0。`ldd` 发现两套 libprotobuf 时 configure/test 必须失败。

- [ ] **Step 5: 运行 ABI 单元 GREEN**

Run: `"$STAGE4_CMAKE" --preset stage4-dev && "$STAGE4_CTEST" --preset stage4-dev -N -R '^build_contract$' --no-tests=error && "$STAGE4_CMAKE" --build --preset stage4-dev --target test_build_contract && "$STAGE4_CTEST" --preset stage4-dev -R '^build_contract$' --output-on-failure --no-tests=error`

Expected: 只发现并运行 `build_contract`，PASS；生成头报告 eCAL `6.1.1`、Protobuf `33.6.0`、GCC 13、C++17 和 ABI=1。

- [ ] **Step 6: 写安装/package config/process 合同 RED**

`tests/stage4/test_cpp_install_contract.py` 必须完全从 subprocess 驱动 wished-for install/runtime 接口；测试函数内先检查目标脚本或文件，不在 collection/fixture setup 阶段报错。至少包含以下正反例：

- 正例：安装到临时前缀后，仅以该前缀为 `CMAKE_PREFIX_PATH`，独立 `version_consumer` 能 `find_package(SlopeSimClient CONFIG REQUIRED)`、编译并输出 `6.1.1/33.6.0/1`。
- 正例：`stage_cpp_runtime.sh --mode sdk` 合并精确 allowlist 后，整个安装树搬迁到第二个临时目录；清空 `LD_LIBRARY_PATH` 且工作目录离开仓库后，consumer 与已安装库仍可运行。
- 反例：package config 缺 targets/header、版本不匹配、引用仓库或 dependency prefix 时明确失败。
- 反例：runtime stager 遇到不同 hash 覆盖、未知 DSO、第二套 libprotobuf、绝对/逃逸/目录 symlink 时明确拒绝且不留下半成品。

测试先用明确行为断言期待这些合同成立；尚未创建 package config、`version.cpp` install rule 或 runtime stager 时，通过 `pytest.fail("install contract not implemented: ...")` 形成 RED，不能把 `FileNotFoundError`、collection error 或 skip 当作 RED。

- [ ] **Step 7: 运行并确认安装合同 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_cpp_install_contract.py`

Expected: pytest 正常收集全部正反例并 `FAILED`；首个失败明确指向 package config/runtime staging/relocation 行为尚未实现。若失败来自测试未收集、临时目录权限、编译器或依赖缺失，先修测试壳并原样重跑本 Step。

- [ ] **Step 8: 最小实现 package config 与可搬迁安装树**

安装 `libslope_sim_client.so`、`version.hpp`、v2 `.proto`、descriptor set、CMake targets 和独立 `version_consumer`；`SlopeSimClientConfig.cmake` 只导出已安装路径和带 namespace 的 targets。`packaging/stage_cpp_runtime.sh --dependency-prefix ... --project-prefix ... --mode sdk` 再按依赖 lock 的精确 allowlist，把 eCAL/Protobuf/MCAP/Zstd 的运行库、SONAME 别名、下游必需 header 和 CMake config 合并到同一 prefix。脚本先在同级临时目录构造完整结果再原子替换，任何反例都不得部分污染原 project prefix。

阶段 C 的开发安装树允许只指向该 prefix 内普通文件的相对 SONAME symlink，但脚本拒绝覆盖不同 hash 文件、绝对/逃逸/目录 symlink、未知 DSO 和第二套 libprotobuf；阶段 E 生成发行 manifest 前必须把这些别名物化为独立普通文件并证明所有文件 `st_nlink == 1`。下游只把该 prefix 放进 `CMAKE_PREFIX_PATH` 即可 `find_package(SlopeSimClient CONFIG REQUIRED)`，不依赖仓库或原 dependency prefix。

- [ ] **Step 9: 运行安装合同 GREEN 与 install-tree ELF 检查**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_cpp_install_contract.py`

Run: `"$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-dev-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-dev-install" --mode sdk`

Run: `conda run -n slope-sim python scripts/verify_stage4_dependencies.py --install-prefix "$PWD/build/stage4-dev-install" --json results/stage4/cpp-abi.json`

Expected: pytest PASS；verifier 只遍历 install tree 中的 ELF，并对每个已安装 library/executable 执行 `readelf -d`、`ldd` 和真实加载检查。所有依赖只解析到同一安装前缀或系统 allowlist，RUNPATH 只能是 `$ORIGIN`/`$ORIGIN/../lib`，不得出现 Conda、仓库、build tree 或原 dependency prefix 绝对路径。build tree 只负责产物生成，不作为 RPATH/ELF 验收对象。

- [ ] **Step 10: REFACTOR 或记录无必要**

仅整理本 Task 已通过代码中的命名/重复；若无必要，勾选时记录“REFACTOR：无必要”。

- [ ] **Step 11: 原样复验两个循环**

原样重跑 Step 5 的 ABI GREEN 命令，以及 Step 9 的三条安装合同/安装/runtime/verifier 命令，不增删参数、不放宽断言。

## Task 2：原始接收 envelope 与协议校验

**Files:**
- Create: `cpp/include/slope_sim/client/protocol.hpp`
- Create: `cpp/src/client/protocol.cpp`
- Create: `cpp/include/slope_sim/client/raw_message.hpp`
- Create: `cpp/apps/v2_interop_golden.cpp`
- Create: `cpp/tests/test_protocol.cpp`
- Create: `tests/stage4/test_cpp_sdk_v2_interop.py`
- Test: `tests/stage4/test_cpp_v2_interop.py`（仅 Phase-0 回归）
- Modify: `cpp/CMakeLists.txt`

- [ ] **Step 1: 注册测试并写 raw-first 单元 RED**

```cpp
TEST(Protocol, CopiesAndHashesBeforeParsing) {
  const auto wire = GoldenNonCanonicalFieldOrderBytes();
  auto capture = RawCapture::Copy(
      "/sim/wheel/state",
      0,
      TypeMetadata(),
      wire.data(),
      wire.size(),
      1000,
      7,
      std::chrono::steady_clock::time_point{std::chrono::nanoseconds{123}},
      1'725'000'000'000'000'000ULL);
  EXPECT_EQ(capture.payload, wire);
  EXPECT_EQ(capture.record_order, 0u);
  EXPECT_EQ(capture.remote_type_name, TypeMetadata().name);
  EXPECT_EQ(capture.remote_encoding, "proto");
  EXPECT_EQ(capture.remote_descriptor, TypeMetadata().descriptor);
  const auto envelope = ValidateAndHash(std::move(capture), FrozenDescriptor());
  EXPECT_EQ(envelope.payload_sha256, Sha256(wire));
  const auto parsed = DecodeAndValidate<WheelState>(envelope);
  EXPECT_EQ(envelope.payload, wire);
  EXPECT_EQ(parsed.sequence(), 7u);
}
```

覆盖 type name 错误、metadata descriptor 错误、带内 descriptor 错误、session 不是 16 bytes、descriptor 不是 32 bytes、RTK 缺 presence/非有限/退化、数组基数、未知 enum、超长 payload。

同一步先在 `cpp/CMakeLists.txt` 注册 `test_protocol`；preset configure 必须成功，首次 build 只允许因测试引用的 Protocol/RawCapture API 尚未实现而失败。

- [ ] **Step 2: 证明 CTest 已注册并运行单元 RED**

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^protocol$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_protocol`

Expected: configure 成功且 `ctest -N` 恰好列出 `protocol`；首次 build 因测试引用的 Protocol/RawCapture API 尚不存在而失败。

- [ ] **Step 3: 确认单元 RED 的失败原因正确**

编译诊断必须只指向本 Task wished-for API/类型/符号；unknown target、0 tests、缺依赖、生成文件缺失或无关告警当错误都不算 RED。修正测试基础设施后必须原样重跑 Step 2，直到获得目标 API 缺失的确定失败。

- [ ] **Step 4: 最小实现拥有明确生命周期的 envelope**

```cpp
struct RawCapture final {
  std::string topic;
  std::uint64_t record_order;
  std::vector<std::byte> payload;
  std::string remote_type_name;
  std::string remote_encoding;
  std::vector<std::byte> remote_descriptor;
  std::int64_t send_timestamp_us;
  std::int64_t send_clock;
  std::chrono::steady_clock::time_point received_at;
  std::uint64_t received_wall_time_ns;

  static RawCapture Copy(std::string_view topic,
                         std::uint64_t record_order,
                         const TypeMetadata& metadata,
                         const void* payload,
                         std::size_t payload_size,
                         std::int64_t send_timestamp_us,
                         std::int64_t send_clock,
                         std::chrono::steady_clock::time_point received_at,
                         std::uint64_t received_wall_time_ns);
};

struct RawEnvelope final {
  RawCapture capture;
  Sha256Digest payload_sha256;
};
```

callback 返回后 capture 不引用 eCAL buffer。`record_order` 由拥有该 callback 的组件在成功预约有界容量时用同一互斥临界区分配，从 0 连续递增；它只定义该进程观察到的跨 topic 顺序，不冒充发送端全局顺序。`received_at` 只用于同进程 deadline/延迟统计，`received_wall_time_ns` 使用 Unix epoch 纳秒持久化到 MCAP；两者都在 callback 入口一次采样，不能在 worker 中补猜。worker 先对 capture 计算 SHA-256，再做大小/type/metadata、ParseFromArray、带内 session/descriptor/有限值和领域规则；协议错误计 `protocol_rejected`，不得进入 sequence gap 基线。

生产 adapter 固定使用 `eCAL::CPublisher/CSubscriber` 和 `eCAL::SDataTypeInformation{name, encoding, descriptor}`；eCAL 6.1.1 receive callback 的三个参数固定为 `(const STopicId& publisher_id, const SDataTypeInformation& remote_type, const SReceiveCallbackData& data)`。callback 直接复制本帧 `remote_type.name/encoding/descriptor`、`publisher_id.topic_id` 的 EntityId、`data.buffer/buffer_size/send_timestamp/send_clock`，并各采样一次 steady 与 Unix epoch 接收时钟；不能复制本地 Subscriber 构造参数，也不能在 callback 内读取 monitoring。

C++ 侧直接复用阶段 A Phase-0 已证明的双门方法：discovery 线程读取 exact publisher count 与 monitoring 中同 topic 的所有远端 metadata，构造不可变 `RemoteEndpointSnapshot` 并原子发布为 topic protocol gate；0 个为 `waiting`，count 与 monitoring 尚未收敛为 `pending`，任一 metadata 不同或不允许的多 publisher 为 `conflict`。callback 始终只复制本帧真实 metadata，worker 在 hash 后先校验它，再要求当前 topic gate 为 `verified`，之后才 parse；不再按 endpoint id 查表回填类型。若诊断需要关联两份证据，只比较 monitoring `topic_id` 与 `publisher_id.topic_id.entity_id`，并同时记录 host/process。每次状态读取仍固定先 `PollPeerState()` 再 `Snapshot()`；fake 与真实 Phase-0 分别覆盖本地声明正确但远端错误、异步收敛、多 publisher 和断线重连。

Python 侧必须把 `SerializeToString(deterministic=True)` 的结果交 raw publisher，禁止使用内部只调用普通 `SerializeToString()` 的 typed serializer 冒充 wire-exact。

- [ ] **Step 5: 运行协议 core 单元 GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_protocol && "$STAGE4_CTEST" --preset stage4-dev -R '^protocol$' --output-on-failure --no-tests=error`

Expected: `protocol` PASS，raw bytes/hash 在 parse 前后不变，所有拒绝边界都有精确错误码。

- [ ] **Step 6: 写新 C++ SDK golden/process RED**

注册独立 target `slope-sim-v2-golden`，先只实现参数解析和 `UNIMPLEMENTED` JSON 错误，不调用 Phase-0 test binary。`tests/stage4/test_cpp_sdk_v2_interop.py` 通过该 target 的绝对路径启动两个真实子进程方向：

- `encode`：C++ SDK 生成五类 v2 deterministic payload，Python 按冻结 descriptor 解析并核对逐字段、原始 bytes 和 SHA-256。
- `decode`：Python 生成非规范字段顺序及边界 fixture，C++ SDK 从 stdin/raw file 解码并输出规范 JSON 与原 payload hash。
- 反例：错误 type metadata、descriptor、session 长度、未知 enum、超限 payload 必须由 C++ 进程以稳定非零码拒绝。

测试必须从必填 `STAGE4_CPP_GOLDEN_BINARY` 接收并校验绝对路径；不得读取 `STAGE4_PHASE0_BUILD_DIR`。stub 能正常启动但行为断言失败，确保本循环 RED 不是 `FileNotFoundError`、collection error 或 skip。

- [ ] **Step 7: 运行并确认 SDK process RED**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-v2-golden && STAGE4_CPP_GOLDEN_BINARY="$PWD/build/stage4-dev/bin/slope-sim-v2-golden" conda run -n slope-sim python -m pytest -q tests/stage4/test_cpp_sdk_v2_interop.py`

Expected: target 构建成功、pytest 正常收集并 `FAILED`，首个差异来自 stub 的 `UNIMPLEMENTED`/缺少预期 payload；不得失败于 target 不存在、相对路径、旧 Phase-0 目录或动态库加载。

- [ ] **Step 8: 最小实现 SDK golden process wiring**

只把 Step 4 已通过的 Protocol/RawCapture API 接到 `encode`/`decode` 子命令；二进制不得复制另一份 protobuf 校验逻辑。stdin/stdout 使用长度受限的 raw bytes/`google::protobuf::Struct` JSON，诊断写 stderr；每次运行报告实际链接的 SDK、eCAL 和 Protobuf `33.6.0` 版本，测试拒绝 build-tree 外的同名 PATH 程序。

- [ ] **Step 9: 运行新 SDK process GREEN，并单独保留 Phase-0 回归**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-v2-golden && STAGE4_CPP_GOLDEN_BINARY="$PWD/build/stage4-dev/bin/slope-sim-v2-golden" conda run -n slope-sim python -m pytest -q tests/stage4/test_cpp_sdk_v2_interop.py`

Run: `STAGE4_PHASE0_BUILD_DIR="$PWD/build/stage4-phase0" conda run -n slope-sim python -m pytest -q tests/stage4/test_cpp_v2_interop.py`

Expected: 新 SDK golden 的两个方向和反例全部 PASS，raw bytes/hash 不变；随后旧 Phase-0 测试也 PASS，但只作为既有实现回归，不能替代前一条新 SDK 证据。

- [ ] **Step 10: REFACTOR 或记录无必要**

仅消除 Protocol 与 golden wiring 中已经出现的真实重复；若无必要，记录“REFACTOR：无必要”。

- [ ] **Step 11: 原样复验两个循环**

原样重跑 Step 5 的 CTest GREEN 命令和 Step 9 的两条 process/regression 命令，不改变 fixture、binary 路径或断言。

## Task 3：C++ Client 生命周期、latest lane 与统计

**Files:**
- Create: `cpp/include/slope_sim/client/client.hpp`
- Create: `cpp/include/slope_sim/client/statistics.hpp`
- Create: `cpp/include/slope_sim/client/raw_ecal.hpp`
- Create: `cpp/src/client/client.cpp`
- Create: `cpp/src/client/statistics.cpp`
- Create: `cpp/src/client/raw_ecal.cpp`
- Create: `cpp/tests/test_client_lifecycle.cpp`
- Create: `cpp/tests/test_topic_tracker.cpp`
- Create: `cpp/tests/test_latest_lane.cpp`
- Modify: `cpp/CMakeLists.txt`
- Modify: `CMakePresets.json`

- [ ] **Step 1: 写生命周期和慢回调 RED**

```cpp
TEST(ClientLifecycle, IsIdempotentAndIgnoresLateCallbackAfterStop) {
  FakeRawEcal ecal;
  Client client(ecal, ClientOptions{});
  client.Start();
  client.Stop();
  client.Stop();
  ecal.DeliverLate(GoldenWheelStateBytes());
  EXPECT_EQ(client.Snapshot().accepted, 0u);
}

TEST(LatestLane, SlowConsumerDoesNotBlockNativeCallback) {
  LatestLane lane;
  lane.Push(Message(1));
  auto owner = lane.Claim();
  lane.Push(Message(2));
  lane.Push(Message(3));
  EXPECT_EQ(lane.Snapshot().consumer_superseded, 1u);
  EXPECT_EQ(lane.TakeLatest()->sequence, 3u);
}
```

覆盖 start 失败回滚、callback 内 stop 拒绝同步自 join、断线/重连首帧基线、跨 simulation session、generation 前进/回退、重复/逆序、用户异常、每 topic 独立统计。

同一步把三个测试 target 注册进 `cpp/CMakeLists.txt`，并加入可配置的 `stage4-tsan` preset；RED 前 `"$STAGE4_CMAKE" --preset stage4-dev` 必须成功。首次 build 只允许因上述测试引用的 Client API 尚未实现而失败，不能是 unknown target、找不到依赖或 CMake configure 错误。

- [ ] **Step 2: 证明 CTest 已注册并运行 RED**

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^(client_lifecycle|topic_tracker|latest_lane)$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_client_lifecycle test_topic_tracker test_latest_lane`

Expected: configure 成功且 `ctest -N` 恰好列出 3 个测试；build 只因测试引用的 Client/Tracker/LatestLane API 尚不存在而失败。

- [ ] **Step 3: 确认 RED 的失败原因正确**

逐个 target 核对编译诊断都来自本 Task wished-for API；unknown target、0 tests、依赖/生成错误或只构建了部分测试均不算 RED。修正测试壳后原样重跑 Step 2。

- [ ] **Step 4: 最小实现 state/token 边界**

```cpp
enum class ClientState { kCreated, kStarting, kRunning, kStopping, kStopped, kFailed };

struct CallbackToken final {
  std::uint64_t lifecycle_generation;
  std::weak_ptr<CallbackState> state;
};
```

native callback 取得 token 后只复制/入 lane；stop 先使 token 失活、等待在途 callback 返回，再销毁 subscriber/participant。不得在 eCAL callback 或 worker 自身线程阻塞 join。

eCAL 资源顺序固定为 Initialize → 创建 publisher/subscriber → RemoveReceiveCallback → 等待在途 callback → 析构 pub/sub → Finalize；晚到 callback 只能看到失活 token，不能访问已释放 native handle。

- [ ] **Step 5: 完成连续性与 late join 的最小实现**

Tracker 主键为 `(simulation_session_id, topic, world_generation)`。可选 consumer 首个合法帧建立 baseline 并计 `late_join_count=1`；之后缺口、重复、逆序、跨 session 分别计数。Recorder 不使用 late-join 豁免，必须在 sequence 0 前 READY。

- [ ] **Step 6: 运行普通与 TSan GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_client_lifecycle test_topic_tracker test_latest_lane && "$STAGE4_CTEST" --preset stage4-dev -R '^(client_lifecycle|topic_tracker|latest_lane)$' --output-on-failure --no-tests=error`

Run: `"$STAGE4_CMAKE" --preset stage4-tsan && "$STAGE4_CTEST" --preset stage4-tsan -N -R '^(client_lifecycle|topic_tracker|latest_lane)$' --no-tests=error && "$STAGE4_CMAKE" --build --preset stage4-tsan --target test_client_lifecycle test_topic_tracker test_latest_lane && "$STAGE4_CTEST" --preset stage4-tsan -R '^(client_lifecycle|topic_tracker|latest_lane)$' --output-on-failure --no-tests=error`

Expected: 普通与 ThreadSanitizer preset 均 PASS，且没有 data race 报告。

- [ ] **Step 7: REFACTOR 或记录无必要**

只在测试保持 GREEN 的前提下整理生命周期状态转换、token 所有权或统计重复；若没有可证明的简化，勾选时记录“REFACTOR：无必要”。

- [ ] **Step 8: 原样复验**

原样重跑 Step 6 的两条命令；不得删减 TSan、改变正则、关闭 sanitizer 或放宽断言。

## Task 4：只读 Subscriber CLI 与 SDK 示例

**Files:**
- Create: `cpp/apps/subscriber_main.cpp`
- Create: `cpp/examples/minimal_subscriber.cpp`
- Create: `cpp/examples/consumer_project/CMakeLists.txt`
- Create: `cpp/examples/consumer_project/main.cpp`
- Create: `cpp/tests/test_subscriber_cli.cpp`
- Create: `docs/阶段四C++SDK教程.md`
- Modify: `cpp/CMakeLists.txt`

- [ ] **Step 1: 写 CLI 行为 RED**

```cpp
TEST(SubscriberCli, JsonContainsPerTopicHealth) {
  const auto output = RunSubscriber({"--json", "--once"}, GoldenFourOutputFixture());
  EXPECT_EQ(output["simulation_session_id"], SessionHex());
  EXPECT_EQ(output["topics"]["/sim/lidar/points"]["descriptor_valid"], true);
  EXPECT_EQ(output["topics"]["/sim/lidar/points"]["dropped"], 0);
}
```

覆盖概要、完整 JSON、周期统计、SIGINT 正常停止、缺 topic 非零健康、禁止创建 WheelCommand publisher。

同一步先在 `cpp/CMakeLists.txt` 注册 `test_subscriber_cli`；configure 必须成功，首次 build 只允许因 Subscriber 行为入口尚未实现而失败。

- [ ] **Step 2: 证明 CTest 已注册并运行 RED**

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^subscriber_cli$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_subscriber_cli`

Expected: configure 成功且 `ctest -N` 恰好列出 `subscriber_cli`；build 只因测试引用的 Subscriber 行为入口尚不存在而失败。

- [ ] **Step 3: 确认 RED 的失败原因正确**

编译诊断必须指向 wished-for Subscriber CLI/core API；unknown target、0 tests、缺依赖或链接到错误 SDK 不算 RED。修正测试壳后原样重跑 Step 2。

- [ ] **Step 4: 最小实现 CLI 和 SDK 示例**

`slope-sim-sub --summary|--json|--stats-period-ms N` 只构造四个输出 subscribers；JSON 写 stdout，诊断写 stderr。`minimal_subscriber.cpp` 展示 `Start()`、`Poll()`、不可变回调、`Snapshot()` 和幂等 `Stop()`；`consumer_project` 是完全独立的 CMake 工程，只通过 `find_package(SlopeSimClient CONFIG REQUIRED)` 链接安装后的 SDK，不包含或引用主仓库源码。

CLI 结构化输出统一先构造 `google::protobuf::Struct`，再用同一 Protobuf 33.6.0 的 `google::protobuf::util::MessageToJsonString` 和 `preserve_proto_field_names=true` 输出；测试按 JSON 语义解析，不依赖 map 文本顺序。不得为 CLI/manifest/sidecar 再引入第二套 JSON 库，也不得手写通用字符串转义。

- [ ] **Step 5: 运行 CLI GREEN 与安装后 consumer smoke**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_subscriber_cli slope-sim-sub && "$STAGE4_CTEST" --preset stage4-dev -R '^subscriber_cli$' --output-on-failure --no-tests=error`

Run: `STAGE4_SDK_SMOKE_ROOT="$(mktemp -d -t slope-sim-sdk-smoke.XXXXXX)" && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$STAGE4_SDK_SMOKE_ROOT/install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$STAGE4_SDK_SMOKE_ROOT/install" --mode sdk && "$STAGE4_CMAKE" -S cpp/examples/consumer_project -B "$STAGE4_SDK_SMOKE_ROOT/build" -DCMAKE_C_COMPILER="$STAGE4_CC" -DCMAKE_CXX_COMPILER="$STAGE4_CXX" -DCMAKE_PREFIX_PATH="$STAGE4_SDK_SMOKE_ROOT/install" && "$STAGE4_CMAKE" --build "$STAGE4_SDK_SMOKE_ROOT/build" && env -u LD_LIBRARY_PATH PATH=/usr/bin:/bin "$STAGE4_SDK_SMOKE_ROOT/build/slope_sim_consumer_example" --version`

Expected: rc=0，不引用源码树。

- [ ] **Step 6: REFACTOR 或记录无必要**

只整理已经通过测试的 CLI 参数解析/JSON 组装重复；若没有必要，勾选时记录“REFACTOR：无必要”。

- [ ] **Step 7: 原样复验**

原样重跑 Step 5 的两条命令，包括全新 `mktemp` 安装前缀；不得复用上次临时目录或改回 build-tree 运行。

## Task 5：独立 Command Tool 与 authority 握手

**Files:**
- Create: `cpp/include/slope_sim/client/command.hpp`
- Create: `cpp/src/client/command.cpp`
- Create: `cpp/apps/command_main.cpp`
- Create: `cpp/tests/test_command.cpp`
- Create: `cpp/tests/ecal_test_shim.cpp`
- Test: `tests/stage4/test_cpp_command_process.py`
- Modify: `cpp/CMakeLists.txt`

- [ ] **Step 1: 写纯 Command core 的未认领/冲突 RED**

```cpp
TEST(Command, DoesNotPublishBeforeClaimableWheelState) {
  CommandOptions options;
  options.source_id = "qa.tool";
  CommandTool tool(options);
  tool.Observe(WheelStateWithAuthority(CommandAuthorityState::WAITING, 0));
  tool.Tick();
  EXPECT_EQ(tool.PublishedCount(), 0u);
}

TEST(Command, UsesNewGenerationAfterConflict) {
  CommandTool tool = ActiveTool();
  tool.Observe(WheelStateWithAuthority(CommandAuthorityState::CONFLICT, 2));
  EXPECT_TRUE(tool.TargetsZero());
  EXPECT_FALSE(tool.HasAuthority());
}

TEST(Command, DrainFreezesAfterReportingOneFinalZeroFence) {
  CommandTool tool = ActiveTool();
  tool.BeginNormalDrain(/*end_timestamp_ns=*/1'000'000'000,
                        /*minimum_post_window_ns=*/100'000'000);
  tool.Observe(WheelStateAt(1'110'000'000));
  tool.TickAtNextDeadline();
  EXPECT_TRUE(tool.LastCommand().all_wheel_speeds_are_zero());
  ASSERT_TRUE(tool.NormalDrainFence().has_value());
  EXPECT_EQ(tool.NormalDrainFence()->timestamp_ns(), 1'110'000'000);
  EXPECT_FALSE(tool.IsPublishing());
  EXPECT_TRUE(tool.IsParticipantAlive());
  tool.CompleteNormalDrain();
  EXPECT_FALSE(tool.IsParticipantAlive());
}

TEST(Command, ManualTwistLeaseExpiresToZero) {
  CommandTool tool = ActiveToolWithClock();
  tool.AcceptManualTwist(ManualTwistTarget{/*linear_mps=*/0.8,
                                           /*angular_rad_s=*/0.3,
                                           /*lease_ms=*/100,
                                           /*request_id=*/7});
  tool.AdvanceWallClock(std::chrono::milliseconds{99});
  EXPECT_FALSE(tool.TargetsZero());
  tool.AdvanceWallClock(std::chrono::milliseconds{1});
  EXPECT_TRUE(tool.TargetsZero());
}
```

覆盖随机 16-byte source session、source id 字符集、车型 2+0/4+2 数组、旧 simulation/world/command generation、100ms 超时和正常退出零命令。再覆盖线速度/角速度有限值与车型限位、request id 重复/回退、租约 `1..100 ms`、第 99/100ms 边界、按键释放显式零、控制输入断开、窗口失焦、scene freeze 和 authority 变化全部归零；测试只推进 fake 单调钟，不启动 eCAL、不依赖键盘 repeat，也不接共享 control socket。

同一步先在 `cpp/CMakeLists.txt` 注册 `test_command`；configure 必须成功，首次 build 只允许因 Command API 尚未实现而失败。

- [ ] **Step 2: 证明 CTest 已注册并运行 core RED**

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^command$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_command`

Expected: configure 成功且 `ctest -N` 恰好列出 `command`；build 只因 wished-for Command core API 尚不存在而失败。

- [ ] **Step 3: 确认 core RED 的失败原因正确**

诊断必须来自 `CommandTool`/纯输入输出状态 API 缺失；unknown target、0 tests、eCAL 环境、control proto 或依赖错误均不算本循环 RED。修正测试壳后原样重跑 Step 2。

- [ ] **Step 4: 最小实现先观察再认领的纯 core**

```cpp
if (state.command_authority_state() != CLAIMABLE ||
    state.command_peer_count() != 1) {
  return SendDecision::kWait;
}
command.set_simulation_session_id(state.simulation_session_id());
command.set_world_generation(state.world_generation());
command.set_command_generation(state.command_generation());
command.set_source_session_id(source_session_id_);
```

100Hz 决策使用绝对 deadline，超期不补发 burst；每次发送尝试前占 sequence。ACTIVE owner 回显与本 tool 不一致立即归零并停止发送，错误包不尝试抢权。人工模式 core 只接受同 session、严格递增 request id 的 `ManualTwistTarget(linear_velocity_mps, angular_velocity_rad_s, lease_ms)` 值对象；Command 用车型 canonical 参数唯一换算为 `2+0` 或 `4+2`，并在 100ms 租约到期、输入关闭、失焦、scene freeze 或 command generation 改变时原子切零。Dashboard/Python 不创建第二个 eCAL publisher；自动门禁用同一 Command API 注入固定 motion recipe。

`BeginNormalDrain(end_timestamp_ns, minimum_post_window_ns)` 的纯 core 决策不会退出 participant，而是持续用最新合法 WheelState 身份产生全零命令；首次由 adapter 确认成功发布 `timestamp_ns > end_timestamp_ns + minimum_post_window_ns` 的零命令后，原子保存该完整 identity 为唯一 WheelCommand fence 并冻结后续发布决策。共享 control status 上报、等待 Recorder FINALIZED 和最终 participant 退出只在 Task 7 接线。

- [ ] **Step 5: 运行 Command core GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_command && "$STAGE4_CTEST" --preset stage4-dev -R '^command$' --output-on-failure --no-tests=error`

Expected: `command` PASS；全部时序由 fake 单调钟确定，未启动生产进程。

- [ ] **Step 6: 写 `slope-sim-command` CLI/process wiring RED**

先注册并构建生产 target `slope-sim-command`，main 只提供稳定参数解析、`--version` 和明确 `UNIMPLEMENTED` 退出。另以 `EXCLUDE_FROM_ALL` 构建只供测试的 ABI-compatible `stage4_ecal_test_shim`；该 target 由 C Task 5 唯一拥有，只能在 `BUILD_TESTING=ON` 时定义，并使用同一 `STAGE4_CXX`（GCC 13）、C++ ABI 1 和冻结的 eCAL 6.1.1 headers，不得出现在任何 `install()`、export set、runtime manifest 或发布包中。`STAGE4_ECAL_TEST_SHIM` 只允许由 pytest fixture 读取；fixture 为每个用例创建私有 `0700` IPC root，并且只在 `subprocess.Popen` 启动被测生产 child 的环境中设置绝对 `LD_PRELOAD` 与 IPC 路径，pytest/Conda/fixture parent 和其他 child 的环境都必须没有这两个值。生产 ELF、库和 launcher 不得读取测试变量或实现测试 fallback。shim 拦截并记录 Initialize/Finalize/pub/sub/monitoring 调用，以确定性 fake peer/payload 驱动真实 main/adapter，不启动真实 eCAL participant；fixture 用 `readelf --dyn-syms` 加 loader 调用日志对生产 ELF 的 eCAL dynamic symbol binding 做冻结 allowlist 审计，任何未被 shim 接管的 eCAL 调用、任何回落到真实 DSO 的 Initialize/pub/sub/monitoring/entity API 或 allowlist 漂移都失败。`tests/stage4/test_cpp_command_process.py` 以绝对安装前/构建产物路径启动真实进程和该 shim fixture，验证：CLAIMABLE 前零发布、认领后 100 Hz、2+0/4+2 映射、租约到期归零、owner 冲突停止、SIGINT 最终零命令、SIGKILL 后 Simulator 100 ms watchdog 停车，以及 stdout/stderr/退出码合同。fixture 同时断言 shim 只进入指定 child、系统 eCAL entity census 增量为 0，并扫描本次 fresh 临时安装树和 CMake install manifest，证明 shim、测试 IPC、注入变量和测试选择开关均未安装或导出；Task 8 冻结 runtime manifest 时再次执行同一禁入扫描。这里仅接 WheelState subscriber、WheelCommand publisher、信号处理和 Task 5 core；不得提前生成/连接 Task 7 control proto/socket。

- [ ] **Step 7: 运行并确认 CLI/process RED**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-command stage4_ecal_test_shim && STAGE4_COMMAND_TEST_INSTALL_ROOT="$(mktemp -d -t slope-sim-command-install.XXXXXX)" && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$STAGE4_COMMAND_TEST_INSTALL_ROOT" && STAGE4_COMMAND_BINARY="$PWD/build/stage4-dev/bin/slope-sim-command" STAGE4_ECAL_TEST_SHIM="$PWD/build/stage4-dev/lib/libstage4_ecal_test_shim.so" STAGE4_COMMAND_TEST_INSTALL_ROOT="$STAGE4_COMMAND_TEST_INSTALL_ROOT" conda run -n slope-sim python -m pytest -q -m "not ecal" tests/stage4/test_cpp_command_process.py`

Expected: 生产 target 与 test-only shim 构建且进程可启动，pytest 正常收集并因 `UNIMPLEMENTED`/未发布预期命令而 `FAILED`；不得失败于 unknown target、相对/PATH 同名程序、动态库加载、shim fixture 未启动或 collection error，也不得创建真实 eCAL entity。临时安装树和 install manifest 中必须找不到 shim、注入变量或测试选择开关。

- [ ] **Step 8: 最小实现 CLI 与 eCAL process wiring**

main 只把参数、信号、eCAL adapter 和已通过的 Command core 接起来；publisher 成功回执才反馈给 core 占用 sequence/fence。进程持有唯一 WheelCommand publisher；人工/自动输入仍通过可注入 core 入口，Task 7 再把该入口连接到共享 control socket。单独 SIGINT 时先发送一个已确认零命令或等待 Simulator watchdog，所有 native handle 按 Client 生命周期顺序释放。

- [ ] **Step 9: 运行 CLI/process GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-command stage4_ecal_test_shim && STAGE4_COMMAND_TEST_INSTALL_ROOT="$(mktemp -d -t slope-sim-command-install.XXXXXX)" && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$STAGE4_COMMAND_TEST_INSTALL_ROOT" && STAGE4_COMMAND_BINARY="$PWD/build/stage4-dev/bin/slope-sim-command" STAGE4_ECAL_TEST_SHIM="$PWD/build/stage4-dev/lib/libstage4_ecal_test_shim.so" STAGE4_COMMAND_TEST_INSTALL_ROOT="$STAGE4_COMMAND_TEST_INSTALL_ROOT" conda run -n slope-sim python -m pytest -q -m "not ecal" tests/stage4/test_cpp_command_process.py`

Expected: PASS；命令显式构建并执行生产 `slope-sim-command` main/adapter，只在该 child 的 eCAL ABI 边界使用 test-only shim，不使用测试 main、PATH 查找或真实 participant；调用审计与 entity census 均通过，临时安装树和 install manifest 仍不包含 shim 或测试注入入口。

- [ ] **Step 10: REFACTOR 或记录无必要**

只整理 core 与 adapter 间已经出现的重复；若无必要，记录“REFACTOR：无必要”。不得把 eCAL 或 control socket 依赖塞回纯 core。

- [ ] **Step 11: 原样复验两个循环**

原样重跑 Step 5 的 CTest GREEN 和 Step 9 的生产进程命令，不增删参数、不改用测试 binary、不移除 `-m "not ecal"`、test-only shim、安装树扫描或零 entity 断言。

## Task 6：Recorder 双上限总账、ordered commit 与 MCAP writer

**Files:**
- Create: `proto/slope_sim_record_v1.proto`
- Create: `scripts/generate_record_protos.py`
- Generate: `slope_sim/interfaces/generated/slope_sim_record_v1_pb2.py`
- Generate: `slope_sim/interfaces/generated/slope_sim_record_v1.desc`
- Create: `cpp/include/slope_sim/record/recorder.hpp`
- Create: `cpp/include/slope_sim/record/bounded_fifo.hpp`
- Create: `cpp/include/slope_sim/record/session_manifest.hpp`
- Create: `cpp/src/record/recorder.cpp`
- Create: `cpp/src/record/bounded_fifo.cpp`
- Create: `cpp/src/record/session_manifest.cpp`
- Create: `cpp/apps/recorder_main.cpp`
- Create: `cpp/tests/test_recorder_queue.cpp`
- Create: `cpp/tests/test_recorder_writer.cpp`
- Create: `cpp/tests/test_session_manifest.cpp`
- Create: `tests/stage4/test_record_protocol.py`
- Modify: `cpp/CMakeLists.txt`

- [ ] **Step 1: 写容量与硬失败 RED**

```cpp
TEST(RecorderQueue, EnforcesMessageAndByteLimitsWithoutOverwrite) {
  BoundedFifoOptions options;
  options.max_messages = 8192;
  options.max_owned_bytes = 512ULL * 1024ULL * 1024ULL;
  BoundedFifo queue(options);
  FillToByteLimit(queue);
  const auto oldest = queue.Front().payload_sha256;
  EXPECT_EQ(queue.TryPush(OneMoreMessage()), PushResult::kFull);
  EXPECT_EQ(queue.Front().payload_sha256, oldest);
  EXPECT_EQ(queue.Snapshot().dropped, 0u);
  EXPECT_TRUE(queue.Snapshot().failed);
}
```

Recorder 的容量是跨 raw FIFO、ordered-commit ledger（含 deferred command/ready pair/rejected marker）和 rotation holding 共用的一套 reservation ledger：最多 8192 个 slot、512MiB owned bytes。callback 在复制前以 `payload_size + copied remote metadata size + 4096` 预约一个 slot，其中 4096 是 `RecordMetadata` 的硬上限；预约成功才在同一互斥临界区分配从 0 连续递增的 `record_order` 并复制，失败立即 latch fatal。单一 validation worker 按 raw FIFO 顺序为每个已分配 order 原位写入唯一 disposition：`READY | REJECTED | DEFERRED`。先于同 timestamp WheelState 到达的合法 WheelCommand 以 `DEFERRED` 连同原 payload/reservation 留在该 order；辅助 unresolved 索引只保存指向 order 的非 owning key，不得复制消息或另占一套容量。其余合法消息形成 pair 后把 reservation 缩到实际 raw envelope owned bytes + deterministic metadata bytes 并标 `READY`。协议拒绝标 `REJECTED`：若前方有 DEFERRED，只能把 reservation 缩到固定审计 marker 大小并继续占 slot，等连续 frontier 跳过时才释放，避免拒绝风暴绕过 8192 上限。metadata 超过 4096、计数下溢/重复释放、同一 order 重复 disposition 或任一阶段另建未计入 ledger 的 owning 队列都属于内部 fatal。

因此消息数或共享 byte reservation 达到上限都拒绝新消息并 latch fatal；不能覆盖旧消息、阻塞 native callback 或继续假健康。测试同时制造 raw backlog、DEFERRED/READY/REJECTED ordered backlog 和 rotation-holding backlog，证明所有 owning 状态之和而不是每层各自拥有 8192/512MiB。另覆盖 command 先到而 WheelState 后到的合法跨 topic 重排、跨 session/generation、永远未来的 timestamp 和 drain 时仍 unresolved；第一种必须按原 `record_order` 完成，后三种必须硬失败而不是误配 scene revision。

ordered-commit RED 必须精确构造：order 0 是等待 timestamp 100 的 WheelCommand，order 1/2 是已 READY 的普通业务 pair，order 3 是同 session/world、timestamp 100 的 WheelState。处理 order 1/2 后 writer 输出仍为空；处理 order 3 时 worker 用水位在 order 0 原位生成 metadata/pair，把 disposition 从 DEFERRED 改为 READY，再令 writer 最终严格写出 `0,1,2,3`。同组测试还覆盖 order 0 REJECTED 后 order 1 正常推进、DEFERRED 永不解析、rotation/drain 跨越 DEFERRED、以及 READY/REJECTED 在阻塞 frontier 后累积直至共享 slot/byte 上限；不得让 writer 猜“缺口已拒绝”或跳过仍可能解析的 order。

同一 RED 用 Python/C++ golden 锁定 `MessageIdentity/RecordMetadata/SessionManifest` 字段号、optional presence、scene attachment SHA、metadata topic 映射和 deterministic bytes；一个业务 pair 只能作为整体入队，不能只入 raw 或只入 metadata。

- [ ] **Step 2: 写 `.partial` 原子完成 RED**

```cpp
TEST(RecorderWriter, FinalizesInDurabilityOrder) {
  FaultInjectingFilesystem fs;
  Recorder recorder(fs, TestOptions());
  recorder.Finalize();
  EXPECT_EQ(fs.events(), Events({"write_summary", "flush", "fsync_file", "close",
                                 "rename_partial", "fsync_parent"}));
}
```

覆盖磁盘 reserve 不足、write/fsync/rename 失败、CRC 错、崩溃 partial 保留、默认不删旧文件、16MiB chunk、4GiB/30min fence 轮转。

同一步先在 `cpp/CMakeLists.txt` 注册 `test_recorder_queue`、`test_recorder_writer` 和 `test_session_manifest`；preset configure 必须成功，首次 build 只允许因测试引用的 Recorder API 尚未实现而失败。Python record protocol RED 只在测试函数内加载 wished-for generated module。

- [ ] **Step 3: 证明 CTest 已注册并运行 RED**

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^(recorder_queue|recorder_writer|session_manifest)$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_recorder_queue test_recorder_writer test_session_manifest`

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_record_protocol.py`

Expected: configure 成功且 `ctest -N` 恰好列出 3 个 C++ 测试；C++ build 因 wished-for Recorder API 尚不存在而失败，Python pytest 正常收集并因 record binding/行为尚未实现而 `FAILED`。

- [ ] **Step 4: 确认 RED 的失败原因正确**

C++ 诊断只能来自 Recorder/FIFO/manifest API 缺失，Python 首个失败必须是明确的 wished-for binding/合同断言；unknown target、0 tests、collection error、缺 protoc/依赖、skip 或路径错误不算 RED。修正测试基础设施后原样重跑 Step 3。

- [ ] **Step 5: 最小冻结逐消息 metadata 与完整会话 manifest**

MCAP `Message` 没有动态键值 metadata，Channel metadata 又是静态值，因此不能把逐帧 eCAL 时间和 scene revision 写进业务 channel 的“隐含 metadata”。新增独立 record schema：

```proto
// 阶段四原始记录索引：业务 channel 仍保存未经包装的 eCAL payload。
syntax = "proto3";
package slope_sim.record.v1;

message MessageIdentity {
  string topic = 1;
  uint64 timestamp_ns = 2;
  uint64 world_generation = 3;
  uint64 sequence = 4;
  bytes payload_sha256 = 5;
  optional uint64 command_generation = 6;
  optional string source_id = 7;
  optional bytes source_session_id = 8;
}

message RecordMetadata {
  uint32 protocol_version = 1;
  bytes simulation_session_id = 2;
  bytes descriptor_sha256 = 3;
  string protobuf_type = 4;
  uint64 scene_revision = 5;
  int64 send_timestamp_us = 6;
  int64 send_clock = 7;
  uint64 received_wall_time_ns = 8;
  MessageIdentity identity = 9;
  bytes scene_attachment_sha256 = 10;
}

message SceneAttachmentEntry {
  uint64 scene_revision = 1;
  uint64 world_generation = 2;
  uint64 effective_timestamp_ns = 3;
  bytes yaml_sha256 = 4;
  string attachment_name = 5;
}

message SegmentEntry {
  uint32 segment_index = 1;
  string file_name = 2;
  bytes file_sha256 = 3;
  uint64 size_bytes = 4;
  repeated MessageIdentity first_by_topic = 5;
  repeated MessageIdentity last_by_topic = 6;
  repeated SceneAttachmentEntry scene_attachments = 7;
}

message SessionManifest {
  uint32 protocol_version = 1;
  bytes simulation_session_id = 2;
  bytes descriptor_sha256 = 3;
  repeated SegmentEntry segments = 4;
  bytes runtime_manifest_sha256 = 5;
}
```

`RecordMetadata.protocol_version` 和 `SessionManifest.protocol_version` 都表示 record schema 版本，写入值恒为 1；业务接口版本仍由 v2 type/descriptor 表示，禁止把 2 写进这两个字段。`runtime_manifest_sha256` 必须恰好 32 bytes，等于启动当前 Recorder 的安装树 `share/slope-sim/runtime-manifest.json` 原始规范 JSON bytes 的 SHA-256；Recorder 启动时读取一次并锁存，开发安装与 release 安装都不得传空值。该 runtime manifest schema 固定记录 `schema_version=1`、`build_kind=development|release`、Git SHA、v2 descriptor SHA、C++ dependency lock SHA、工具链/ABI 和安装 ELF hash；C 的 verifier 与 E 的发行 verifier 分别和实际安装树交叉验证。

validation worker 为每个已分配 order 在同一有界 `OrderedCommitLedger` 中维护一个 disposition；`READY` 值是不可拆分的 `RecordedPair(record_order, raw_envelope, deterministic_record_metadata_bytes, reservation)`，同一 reservation 从 callback raw FIFO 移交而来，不重复计 slot。`next_commit_order` 是尚未写入或审计跳过的最小 order，`settled_frontier` 恒等于它：writer 只查看该 order，遇 `READY` 固定先写一条业务 record、紧接着写同 key metadata，再释放 reservation 并推进；遇 `REJECTED` 先增加按原因审计计数、释放 marker reservation 并推进；遇 `DEFERRED` 或尚未产生 disposition 则停止，后续 READY/REJECTED 继续由同一共享总账有界保存。任何代码都不得从数值缺口推断 rejected。WheelState 水位到达时，worker 通过 non-owning unresolved 索引定位原 DEFERRED order，在原位验证 scene interval、生成 pair 并转 READY，禁止以当前 WheelState order 重新排队。业务 MCAP channel 的 data 精确等于 `RawEnvelope.capture.payload`；对应 metadata channel 由纯函数 `"/_slope_sim/record-metadata" + business_topic` 得出。两条 MCAP Message 共用 `publishTime=identity.timestamp_ns`、`logTime=received_wall_time_ns`，内建 32-bit `sequence` 固定为 0 且不作为业务身份，完整 64-bit sequence 只从 payload/`MessageIdentity` 校验。reader 以 manifest 段顺序和实际 pair 出现顺序派生只读 `record_order`；被拒绝 order 只留审计计数，不伪造 MCAP pair。禁止把业务 payload 包进另一个 envelope。`RecordMetadata.identity` 的 command optional 字段在 WheelCommand 上必须全部 present，在四个输出上必须全部 absent；`scene_attachment_sha256` 必须恰好 32 bytes，并等于该消息 `scene_revision` 唯一生效的 canonical YAML attachment 内容哈希。测试拒绝两个 record version 的 0/2、空/短/错误 runtime manifest digest 以及 digest 与安装树不匹配。

测试用独立 reader 拒绝缺 pair、多 pair、非相邻、topic/type/session/descriptor/payload/attachment hash 不同、MCAP Schema/Channel name/encoding/data/static metadata 缺失或格式错误、command presence 错误和 metadata payload 非确定性。`generate_record_protos.py` 与 interface/control 生成脚本分离，使用同一必填 `STAGE4_PROTOC`，只生成受版本控制的 record Python binding 和独立 descriptor；root CMake 从同一 `.proto` 在各自 `${binaryDir}/generated` 生成 C++ binding，不得重写 A 已冻结的 interface descriptor。

Run: `STAGE4_PROTOC="$STAGE4_PROTOC" conda run -n slope-sim python scripts/generate_record_protos.py`

Expected: Python binding 和 record descriptor 原子生成；重跑 byte-identical，且 interface v2 descriptor SHA 不变。build-only C++ binding 由下一次 preset build 生成到该 preset 的 `${binaryDir}/generated`。

- [ ] **Step 6: 最小实现 raw MCAP channel/schema/attachment**

每个业务 MCAP Schema 固定 `name=<完整 v2 type>`、`encoding="protobuf"` 和 `data=<完整 FileDescriptorSet bytes>`；业务 Channel 固定 `message_encoding="protobuf"`，静态字符串 metadata 固定为 `ecal_type_name`、`ecal_encoding="proto"`、64 位小写十六进制 `descriptor_sha256_hex` 和 32 位小写十六进制 `simulation_session_id_hex`。每个配对 metadata channel 保存固定 `slope_sim.record.v1.RecordMetadata` schema。Recorder 只允许把 callback 已验证的远端 metadata 规范编码到这些字段，并与本地冻结 descriptor 逐字节交叉检查；同一业务 topic 在任一 segment 出现不同静态合同立即硬失败。会话初始 attachment 固定为 revision 1、world generation 1、effective time 0。以后 scene revision 必须逐次加 1，effective time 严格递增；旧 revision 的合法时间区间为 `[effective_i, effective_{i+1})`，边界帧属于新 revision。会话开始和 scene revision 生效时先写 canonical YAML attachment 及 SHA，完成持久化后才允许对应时间后的业务 pair 入文件；worker 把该 SHA 写入每条 `RecordMetadata`，reader 同时按生效范围选择唯一 attachment并重算其内容 SHA，两项必须相同，不能从邻近业务帧猜测。

Recorder core 接受 adapter 提交的 `(simulation_session_id, scene_revision, world_generation, effective_timestamp_ns, yaml_sha256, canonical_yaml)` 值对象；Task 6 只测试返回 ACK/错误的纯接口，不创建 control socket。该请求只允许在 Simulator 四输出和 Command publisher 都已冻结、物理 world 事务已经成功后出现。core 先重算 canonical YAML SHA 并要求传入 digest 恰好 32 bytes，再验证初始值、revision 连续性、effective time 严格递增、world generation 不回退且每次最多加 1。成功写 attachment、flush 到 MCAP writer 并更新不可变时间区间索引后才返回 ACK；Task 7 再把同一返回值接入控制线。相同 revision、generation、time、hash 和 bytes 的完整重试幂等 ACK；同 revision 任一字段不同、跳号/回退、同 effective time、非法 generation 或跨 session 直接 FAILED。物理 world 事务失败时不得提交 attachment；attachment ACK 失败时会话失败且新世界不得无记录发布。暂停或事务未提交期间的多次编辑由 Simulator 合并，Recorder core 不接收同时间的多个 revision。

WheelCommand 的 timestamp 必须最终不晚于 Recorder 观察到的同 session/world WheelState 水位，但不能依赖跨 topic 到达顺序。command 先到时在 ordered ledger 原 order 标为 DEFERRED 并保留 reservation，直到水位追上再在原位选择唯一 scene interval并转 READY；后续 order 即使先完成也只能停在同一有界 ledger。session/generation 已越过、达到 drain fence仍未解析或总账满时硬失败。禁止把合法重排立即计为协议拒绝，也禁止让真正未来时间无限等待。

- [ ] **Step 7: 最小实现可继续接收的安全轮转与 session manifest**

每秒检查 `available >= 10GiB + queued_owned_bytes + 32MiB`；活动名 `<session>-<segment>.mcap.partial`。达到 4GiB 或 30 分钟仿真时间时 Recorder core 在 ordered-commit 锁内捕获 `rotation_start_order`，停止把该 order 及之后的 disposition提交到当前段，并产生待发送的 `SegmentCutRequest(next_segment_index)` 事件；Task 7 adapter 负责发送并把 Simulator 返回的五 topic `SegmentBarrier` 交回 core。此后所有 order 仍进入同一共享总账，rotation holding 只标记 ledger 中尚不能跨段提交的范围，不能复制 pair。五个 exact fence pair 都出现后，Recorder 取它们各自 `record_order` 的最大值为唯一 `cut_record_order`；只有从当前 `settled_frontier` 到 cut 的每个已分配 order 都变为 READY 或 REJECTED，writer 才依次写/跳过并推进，任一 DEFERRED 都阻止切段。`<= cut` 的 READY pair 原序写当前段，`> cut` 原序留给下一段；随后 finalize 当前段、打开下一段、先写仍有效的 scene attachments，再继续按 order 排空 ledger。轮转期间 native callback 不阻塞、不覆盖；任一 disposition/holding 达到共享上限与主 FIFO 一样 latch fatal。

同 topic fence 匹配键固定：四输出使用完整 `MessageIdentity` 中的 `(world_generation, sequence, timestamp_ns, payload_sha256)`，WheelCommand 再要求 command generation/source id/source session；session/topic 必须先相等，generation/sequence 回退直接 FAILED。barrier 的作用是证明五条指定消息均已进入 holding，不直接拿不同 topic 的 sequence 比较或逐话题切段。即使某 topic 的 post-fence pair 比另一 topic 的 fence 更早到达，它也按 `cut_record_order` 留在当前段；`SegmentEntry.last_by_topic` 保存该段实际最后 identity，必须不早于请求 fence。这样 manifest 顺序始终保留 Recorder 捕获的全局 `record_order`。

每段完成后生成 `SegmentEntry`，其中保存文件 SHA、实际字节大小、首尾五话题 fence，以及该段可见的完整 `SceneAttachmentEntry`；`first_by_topic/last_by_topic` 都按正式五 topic 固定顺序编码，必须恰好各五项且每个 topic 一次，两个 topic 集完全相同，WheelCommand command optional 三字段全 present、其余四话题全 absent。segment `file_name` 和 attachment name 都必须是无斜杠、无 `..`、无控制字符的唯一规范 basename；writer 不生成绝对路径或目录组件。attachment 名必须在对应 MCAP 内唯一存在且内容 SHA 相同。Recorder 每完成一段就把当前有序段列表确定性编码到 `<session>.manifest.pb.partial.next`，按 `flush/fsync/close/rename-to-.partial/fsync(parent)` 原子替换诊断 checkpoint。正常会话结束时先把最后一段加入同一 checkpoint，再把 `<session>.manifest.pb.partial` 原子 rename 为 `<session>.manifest.pb` 并 `fsync(parent)`；正式 reader 永不接受 checkpoint。测试拒绝路径逃逸/绝对路径/符号链接、段号缺失/重复、`size_bytes` 与文件不符、文件 hash 错、首尾缺 topic/重复 topic/未知 topic/topic 集不一致、fence 重叠或 gap、command presence 错、attachment 缺失/重名/hash 错、跨 session/descriptor、manifest 引用 `.partial`；D/E 的“完整会话”入口只接受最终 manifest，单 `.mcap` 只能以显式 `--single-segment` 诊断模式读取。

- [ ] **Step 8: 运行 core/protocol GREEN 并显式构建 Recorder**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_recorder_queue test_recorder_writer test_session_manifest slope-sim-record && "$STAGE4_CTEST" --preset stage4-dev -R '^(recorder_queue|recorder_writer|session_manifest)$' --output-on-failure --no-tests=error`

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_record_protocol.py`

Expected: 三个 CTest 与 Python protocol 测试 PASS，生产 `slope-sim-record` target 明确构建成功；此时只证明 core、文件写入和最小进程入口，不宣称 control socket/end barrier 已接线。

- [ ] **Step 9: REFACTOR 或记录无必要**

只整理已通过的 reservation/ordered-disposition/reader/writer 重复；若无必要，记录“REFACTOR：无必要”。不得新增绕过共享 ledger 的 owning 队列、从 order 缺口猜 rejected 或提前接控制线。

- [ ] **Step 10: 原样复验**

原样重跑 Step 8 的两条命令，仍须显式构建 `slope-sim-record`，不得缩短正则或跳过 Python golden。

## Task 7：Recorder 健康控制面与 end barrier

**Files:**
- Create: `proto/slope_sim_control_v1.proto`
- Create: `scripts/generate_control_protos.py`
- Generate: `slope_sim/interfaces/generated/slope_sim_control_v1_pb2.py`
- Generate: `slope_sim/interfaces/generated/slope_sim_control_v1.desc`
- Build-only: `build/stage4-dev/generated/slope_sim_control_v1.pb.h`
- Build-only: `build/stage4-dev/generated/slope_sim_control_v1.pb.cc`
- Create: `cpp/include/slope_sim/client/control_socket.hpp`
- Create: `cpp/src/client/control_socket.cpp`
- Create: `cpp/apps/control_golden.cpp`
- Create: `cpp/tests/test_control_socket.cpp`
- Create: `tests/stage4/test_control_protocol.py`
- Create: `tests/stage4/test_cpp_control_processes.py`
- Consume test-only: `cpp/tests/ecal_test_shim.cpp`
- Modify: `cpp/src/client/client.cpp`
- Modify: `cpp/apps/subscriber_main.cpp`
- Modify: `cpp/src/client/command.cpp`
- Modify: `cpp/apps/command_main.cpp`
- Modify: `cpp/src/record/recorder.cpp`
- Modify: `cpp/apps/recorder_main.cpp`
- Modify: `scripts/ecal_simulation_runtime.py`
- Modify: `scripts/verify_ecal_roundtrip.py`
- Test: `tests/stage4/test_recorder_end_barrier.py`
- Test: `tests/stage4/test_recorder_segment_rotation.py`
- Modify: `cpp/CMakeLists.txt`

- [ ] **Step 1: 先冻结 Python/C++ 共用控制线协议 RED**

`proto/slope_sim_control_v1.proto` 固定 package `slope_sim.control.v1`，并定义：

```proto
syntax = "proto3";
package slope_sim.control.v1;

enum Role { ROLE_UNSPECIFIED = 0; SIMULATOR = 1; SUBSCRIBER = 2; COMMAND = 3; RECORDER = 4; BRIDGE = 5; REPLAY = 6; }
enum State { STATE_UNSPECIFIED = 0; STARTING = 1; READY = 2; ACTIVE = 3; DRAINING = 4; FINALIZED = 5; FAILED = 6; ROTATING = 7; }
enum ProtocolState { PROTOCOL_UNSPECIFIED = 0; WAITING = 1; PENDING = 2; VERIFIED = 3; CONFLICT = 4; }

message TopicFence {
  string topic = 1;
  uint64 timestamp_ns = 2;
  uint64 world_generation = 3;
  uint64 sequence = 4;
  bytes payload_sha256 = 5;
  optional uint64 command_generation = 6;
  optional string source_id = 7;
  optional bytes source_session_id = 8;
}
message TopicHealth {
  string topic = 1;
  ProtocolState protocol_state = 2;
  uint32 peer_count = 3;
  repeated string remote_type_names = 4;
  repeated string remote_encodings = 5;
  repeated bytes remote_descriptor_sha256 = 6;
  uint64 accepted_count = 7;
  uint64 protocol_rejected = 8;
  uint64 dropped_count = 9;
  string last_error = 10;
}
message Status {
  Role role = 1;
  State state = 2;
  uint64 queued_messages = 3;
  uint64 queued_bytes = 4;
  uint64 written_count = 5;
  repeated TopicFence fences = 6;
  string error_code = 7;
  string error_detail = 8;
  repeated TopicHealth topic_health = 9;
  optional uint64 replay_clock_ns = 10;
  optional bool replay_paused = 11;
  optional double replay_rate = 12;
}
message SceneAttachment {
  uint64 scene_revision = 1;
  uint64 world_generation = 2;
  uint64 effective_timestamp_ns = 3;
  bytes yaml_sha256 = 4;
  bytes canonical_yaml = 5;
}
message EndBarrier {
  uint64 end_timestamp_ns = 1;
  repeated TopicFence required_fences = 2;
}
message BeginNormalDrain {
  uint64 end_timestamp_ns = 1;
  uint64 minimum_post_window_ns = 2;
}
message SegmentCutRequest { uint32 next_segment_index = 1; }
message SegmentBarrier {
  uint32 next_segment_index = 1;
  repeated TopicFence current_segment_fences = 2;
}
message ManualTwistTarget {
  double linear_velocity_mps = 1;
  double angular_velocity_rad_s = 2;
  uint32 lease_ms = 3;
}
message BeginSceneCommandFreeze {
  uint64 transition_id = 1;
  uint64 target_scene_revision = 2;
  uint64 target_world_generation = 3;
}
message SceneCommandFrozen {
  uint64 transition_id = 1;
  TopicFence last_command = 2;
}
message ResumeCommandAfterScene {
  uint64 transition_id = 1;
  uint64 world_generation = 2;
  uint64 command_generation = 3;
}
message SetReplayPaused { bool paused = 1; }
message StepReplay { uint32 timestamp_batches = 1; }
message SetReplayRate { double rate = 1; }
message Ack { bool accepted = 1; string error_code = 2; string error_detail = 3; }
message ControlEnvelope {
  uint32 protocol_version = 1;
  bytes simulation_session_id = 2;
  bytes descriptor_sha256 = 3;
  uint64 request_id = 4;
  oneof body {
    Status status = 10;
    SceneAttachment scene_attachment = 11;
    BeginNormalDrain begin_normal_drain = 12;
    EndBarrier end_barrier = 13;
    SegmentCutRequest segment_cut_request = 14;
    SegmentBarrier segment_barrier = 15;
    Ack ack = 16;
    ManualTwistTarget manual_twist_target = 17;
    BeginSceneCommandFreeze begin_scene_command_freeze = 18;
    SceneCommandFrozen scene_command_frozen = 19;
    ResumeCommandAfterScene resume_command_after_scene = 20;
    SetReplayPaused set_replay_paused = 21;
    StepReplay step_replay = 22;
    SetReplayRate set_replay_rate = 23;
  }
}
```

Python/C++ fixture 必须双向验证 deterministic bytes；拒绝 version 非 1、session 非 16 bytes、descriptor/hash 非 32 bytes、未知 role/state/protocol state、空 oneof、重复/零 request id、ACK request 不匹配、非法状态迁移、0 字节 frame、超过 1 MiB 和截断 header/body。每个生产 role 在 control socket 完成身份校验后必须先发 `STARTING`，它是 READY 前承载 `WAITING/PENDING/VERIFIED/CONFLICT` TopicHealth、队列和诊断的唯一 lifecycle state，绝不能打开业务门；全部必需 topic `VERIFIED` 后只允许 `STARTING -> READY -> ACTIVE`，发现冲突或启动失败允许 `STARTING -> FAILED`，一旦离开 STARTING 不得回退。其余合法边为 `ACTIVE -> ROTATING -> ACTIVE`、`READY|ACTIVE -> DRAINING -> FINALIZED` 以及所有非终态到 `FAILED`；跨 role/session、跳过 READY、READY 前 ACTIVE、FAILED/FINALIZED 后继续上报均失败。`ManualTwistTarget` 只允许有限值、`lease_ms=1..100` 和严格递增 request id；scene freeze/resume 要求非零且精确匹配的 transition id、连续目标 revision/generation，重复完整请求幂等，字段变化或乱序硬失败。Replay 控制只允许发给 `REPLAY`：`timestamp_batches` 必须恰好为 1；rate 只接受有限的 `0.1..4.0`，暂停状态下设置 rate 只更新下一次恢复倍率；每个 pause/step/rate 请求都用严格递增 request id 和精确匹配 ACK。`Status.replay_clock_ns/replay_paused/replay_rate` 只允许 `REPLAY` role 全部 present，其他 role 必须全部 absent；rate 仍受同一有限范围约束。`TopicHealth` 要求 topic 唯一、peer count 精确保留、三列远端 metadata 基数一致、每个 descriptor digest 为 32 bytes；`VERIFIED` 时全部 endpoint 必须匹配本地合同。`TopicFence` 字段 1..8 与 record proto 的 `MessageIdentity` 字段 1..8 在名称、号码、类型和 optional presence 上完全一致；唯一转换函数逐字段复制，Python/C++ descriptor 测试任一侧漂移都失败。两者的 command optional 三字段只能在 WheelCommand 上全部 present，其他话题全部 absent。生产 stream decoder 每次只消费一帧并保留缓冲区中的后续完整/不完整帧；只有声明为“单帧”的 golden fixture 才把该 fixture 内的额外字节判为非法残留。测试必须覆盖 header/body 逐段到达和两个 frame 合并到一次读取，不能把合法下一帧误判成 trailing bytes。

`SceneCommandFrozen.last_command` 在本 session/generation 已发布过命令时必须 present 且等于冻结前最后一条 identity；尚未认领、发布计数为零时允许 absent，但仍必须 ACK freeze。`ResumeCommandAfterScene` 只解除本地 freeze，Command 仍须等待匹配新 world/command generation 的 CLAIMABLE WheelState 后重新认领，不能沿用旧 owner。

同一步先在 `cpp/CMakeLists.txt` 创建 `test_control_socket`/`control_golden` targets，并分别用 `add_test(NAME control_socket ...)`、`add_test(NAME control_golden ...)` 注册两个精确 CTest，让 preset configure 成功；首次 build 只允许因控制 codec API 尚未实现而失败。Python 测试在测试函数内加载 generated binding，缺 binding 时转成明确 `FAILED`。

- [ ] **Step 2: 证明 CTest 已注册并运行控制协议 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_control_protocol.py`

Run: `"$STAGE4_CMAKE" --preset stage4-dev`

Run: `"$STAGE4_CTEST" --preset stage4-dev -N -R '^(control_socket|control_golden)$' --no-tests=error`

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_control_socket control_golden`

Expected: Python pytest 正常收集并 `FAILED`，C++ configure 成功且 `ctest -N` 恰好列出两个测试，随后 build 因控制 codec API 尚未实现而失败。

- [ ] **Step 3: 确认控制协议 RED 的失败原因正确**

Python 首个失败必须是明确的 generated binding/descriptor/codec 行为缺失；C++ 诊断只能来自 wished-for control codec API。collection error、unknown target、0 tests、缺工具、动态库错误或 skip 均不算 RED；修正测试壳后原样重跑 Step 2。

- [ ] **Step 4: 最小实现版本化 framing 与跨语言 golden**

`scripts/generate_control_protos.py` 只允许调用必填 `STAGE4_PROTOC` 的绝对路径，先断言它解析到总路线已验证 dependency prefix 且 `--version` 精确输出 `libprotoc 33.6`，再从同一个 proto 原子生成受版本控制的 Python binding/descriptor；descriptor 使用 `--include_imports`，脚本重跑必须 byte-identical。root CMake 通过同一冻结 protoc 在当前 `${binaryDir}/generated` 生成 build-only C++ binding；不得在 E 计划再次修改生成器或生成另一份协议来源。

Run: `STAGE4_PROTOC="$STAGE4_PROTOC" conda run -n slope-sim python scripts/generate_control_protos.py`

Expected: Python binding 和 descriptor 生成，第二次执行后 SHA-256 不变；下一次 preset build 在该 `${binaryDir}/generated` 生成 C++ binding。若 `protoc --version` 不是精确 `libprotoc 33.6`，则非零退出且不改旧生成物。

Unix stream frame 固定为网络字节序 `uint32 payload_size` 加 deterministic Protobuf payload，`1 <= payload_size <= 1 MiB`；读取必须循环到 header/body 完整，EOF 中断帧为协议错误。解码器返回一条消息和未消费缓冲，调用方循环取出合并到达的第二帧；测试逐字节拆分 header/body，并把两条合法 frame 合并到同一次 `recv`。服务端用 `SO_PEERCRED` 校验同 UID，socket 目录权限 `0700`、socket `0600`。`control_golden` 和 Python 测试以具体非零 session/descriptor/request/fence/queue 字段互相解码并逐 byte 比较；C、E 和 Simulator 只能复用这一份 binding/codec，禁止另造 JSON 或不同长度前缀。

- [ ] **Step 5: 运行 control codec GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_control_socket control_golden && "$STAGE4_CTEST" --preset stage4-dev -R '^(control_socket|control_golden)$' --output-on-failure --no-tests=error`

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_control_protocol.py`

Expected: C++/Python deterministic bytes、framing、`STARTING -> READY -> ACTIVE`、REPLAY pause/step/rate 和全部反例 PASS；尚未修改三个生产进程的 control wiring。

- [ ] **Step 6: 写三个生产进程的 STARTING/READY/FINALIZED wiring RED**

```python
def test_formal_session_waits_for_all_required_roles_ready(orchestrator, roles) -> None:
    orchestrator.start()
    assert orchestrator.simulator_sequence_count == 0
    roles.recorder.report_ready(all_topics_verified=True)
    roles.subscriber.report_ready(all_topics_verified=True)
    roles.command.report_ready(all_topics_verified=True)
    orchestrator.poll()
    assert orchestrator.simulator_can_publish is False
    roles.simulator.report_ready(all_topics_verified=True)
    orchestrator.poll()
    assert orchestrator.simulator_can_publish is True
```

`tests/stage4/test_cpp_control_processes.py` 使用绝对路径启动生产 `slope-sim-sub`、`slope-sim-command`、`slope-sim-record`，并复用 C Task 5 唯一拥有的 child-only `stage4_ecal_test_shim` 与独立 control fixture。pytest/Conda/fixture parent 不注入 shim；fixture 只在三个生产 child 的 spawn 环境设置绝对 `LD_PRELOAD` 和每 child 私有 IPC，shim 以确定性 fake peer/raw payload 驱动三个真实 main/adapter，并审计每个 child 的 Initialize/Finalize/pub/sub/monitoring 调用。fixture 在每个测试前后执行系统 eCAL entity census，要求增量严格为 0，禁止创建真实 domain、participant、publisher 或 subscriber；三个 child 均执行 Task 5 的 dynamic symbol allowlist，任何真实 eCAL DSO 回落都失败。该文件保持非 `ecal` marker，先断言三个进程在 Task 7 之前不会伪报控制状态，再期待它们分别以 `SUBSCRIBER`、`COMMAND`、`RECORDER` 先上报 `STARTING`，在该状态携带 WAITING/PENDING/VERIFIED 的 TopicHealth 和各自队列/fence，全部门通过后才上报 READY/ACTIVE，失败则上报 FAILED。该 RED 同时证明 STARTING 不开门、跳过 READY、状态回退、control socket 断开、错 session/descriptor、重复 request id、协议冲突和子进程异常退出都会使编排器拒绝开门；不得用直接调用 core 的单元替身冒充进程接线。

正式 profile 的必需 role 由启动配置冻结：核心门禁至少含 Simulator、Subscriber、Command、Recorder；ROS-on 门禁必须再加入 Bridge。每个已启用 role 都先以 STARTING 持续报告 pre-READY health；D Task 5 的 Bridge 也只有四个 eCAL 输入协议门全部 `VERIFIED` 后才允许发 READY，不能由编排器代发或降级省略。各 role 的所有必需 topic 都必须 `VERIFIED` 后才能从 STARTING 转 READY，Simulator 在全部必需 READY 前不得占用 sequence 0。调试时显式省略的 role 不伪造 STARTING 或 READY。

正常结束先由 Simulator 在主线程捕获正式窗口 `end_timestamp_ns`、raw publish/accepted-command log 和 transport snapshot；随后编排器发送 `BeginNormalDrain(end_timestamp_ns, minimum_post_window_ns=100_000_000)`。Command 用当前有效 session/world/command generation 以 100 Hz 发送零速度命令，在发布首条越过阈值的零命令后原子冻结 publisher，并报告该完整 identity；编排器若在 deadline 内没有收到唯一 command fence 就失败。Simulator 继续至少一个完整 10 Hz 周期，直到其余四个输出 topic 都出现 `timestamp_ns > end_timestamp_ns` 的 post-window frame，再在线性化点冻结四个 publisher。只有五个 publisher 均已冻结且五条 fence identity 已固定，Simulator 才发送 `EndBarrier`；这只约束 producer 事件，不能推导 Recorder callback 在 barrier 前后何时收到对应 fence。

control socket 的 barrier 不与 eCAL callback 建立跨通道 happens-before。Recorder 不新增随会话长度增长的 identity ledger，而是复用共享 reservation 下的 raw、ordered-commit disposition 与 rotation holding，并保存每 topic validated/written 高水位、连续性状态和全局 `settled_frontier`。收到 `EndBarrier` 时在同一个状态锁内转入 DRAINING、锁存五条 required fence：validated 高水位已精确达到 fence 的 topic 立即关闭 ingress，已越过则 FAILED；尚未达到的继续处理已有 raw 和新到在途帧，validation worker 恰好验证 required identity 后关闭，不能等待同一 fence 重发。已关闭 topic 再到帧、已存在或新到 identity 越过 required fence、重复 required fence、deadline 内缺 fence都立即 FAILED。五条 required identity 都必须先落为 READY pair；writer 仍从 `next_commit_order` 连续写 READY、审计跳过 REJECTED，任何更早 DEFERRED 都阻止 frontier 越过 fence，并在 drain 判定不可再解析时立即 FAILED。只有 settled frontier 已连续跨过五条 fence、五 topic written 高水位精确达到 required identity 且 raw/ordered/rotation 状态全空，才按 durability 顺序 flush/fsync/rename segment 与 session manifest 并发 FINALIZED。正式窗口 oracle 排除 drain 数据，但用这些数据作为封闭排空 fence。

轮转使用独立 `SegmentCutRequest/SegmentBarrier`，状态为 `ACTIVE -> ROTATING -> ACTIVE`；它不进入 DRAINING/FINALIZED。Recorder 的 global cut/holding 和 manifest 规则来自 Task 6，不能把普通 segment cut 当作会话结束。

增加 scene revision 屏障测试：编排器先向 Command 发送 `BeginSceneCommandFreeze`，Command 立即归零并冻结 publisher，返回匹配 transition id 和最后 command identity；与此同时 Simulator 冻结四输出并撤销旧 command token。五个 publisher 全部冻结后才允许协调器修改物理 world。物理事务失败则回滚旧世界、不发送 attachment，并用新 command generation 恢复；成功后才推进 world/revision、选择下一共同采样位点为 effective timestamp并发送最终 canonical attachment。Recorder ACK 到达前不得发布该 timestamp 及之后的消息；ACK 后才解除新 revision 发布门，Command 必须看到新 WheelState 后再认领。测试覆盖冻结超时、物理失败无 attachment、attachment 失败会话 fatal、初始 revision、连续递增、暂停时多编辑合并、相同 effective time、精确边界、错误 YAML hash/长度、generation 回退/跳跃，以及 command 先于对应 WheelState 到达但可在水位追上后合法解析。Recorder 不存在的显式调试 profile 可以继续，但状态必须持续“未记录”。

正式 profile 的 scene/world rebuild transaction、segment rotation 和 normal drain 共用编排器 lifecycle mutex：`ROTATING` 时首次 stop 只锁存一个 `pending_drain`，重复 stop 幂等，所有新 rebuild 都返回 busy；轮转回到 ACTIVE 后立即执行 drain。已 prepare 的 rebuild 必须完成“五 publisher freeze -> 物理 commit/rollback -> 成功时 attachment ACK -> resume”后才允许发 segment cut；DRAINING 后拒绝 rebuild/rotation。不得让新 scene attachment 跨越尚未确定的 cut。

segment 测试固定模拟不同 topic 的 fence/post-fence 消息乱序到达：五条请求 fence 都在当前段，当前段恰好包含全部 `record_order <= cut_record_order` 的 READY pair并审计跳过其中 REJECTED，下一段从严格 `> cut` 开始，两段合并顺序与 callback 捕获顺序一致；在 cut 前插入 DEFERRED 时不得越过，WheelState 解锁后才按原 order 完成轮转。每 topic 当前段实际 last identity 不早于请求 fence。ordered/holding 总账满、重复/缺 fence、next index 错都显式 FAILED。轮转期间 normal stop 只形成一个 pending drain，回到 ACTIVE 后立即执行；第二个 stop 幂等，scene rebuild 请求明确拒绝为 busy，不得静默丢 pair 或直接把 end barrier 注入 ROTATING。

EndBarrier 测试参数化 fence-before-barrier、barrier-before-fence 和五话题混合顺序，并分别把 required fence 放在 raw、ordered DEFERRED、frontier 后的 READY、rotation-held READY、writer 和 written；另在 required fence 之前放入 REJECTED gap 与可解锁/永不解析的 DEFERRED。所有合法排列都从 `next_commit_order` 连续推进并只 FINALIZED 一次；越界、重复、漏 fence、永不解析或“已见但未完成最终 durability”精确 FAILED。

- [ ] **Step 7: 运行生产进程 wiring RED**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-sub slope-sim-command slope-sim-record stage4_ecal_test_shim`

Run: `STAGE4_SUBSCRIBER_BINARY="$PWD/build/stage4-dev/bin/slope-sim-sub" STAGE4_COMMAND_BINARY="$PWD/build/stage4-dev/bin/slope-sim-command" STAGE4_RECORDER_BINARY="$PWD/build/stage4-dev/bin/slope-sim-record" STAGE4_ECAL_TEST_SHIM="$PWD/build/stage4-dev/lib/libstage4_ecal_test_shim.so" conda run -n slope-sim python -m pytest -q -m "not ecal" tests/stage4/test_cpp_control_processes.py tests/stage4/test_recorder_end_barrier.py tests/stage4/test_recorder_segment_rotation.py`

Expected: 三个生产 target 与 test-only shim 构建成功，pytest 正常收集并因真实进程尚未连接 control socket/上报预期状态而 `FAILED`；fixture 只在 child ABI 边界注入 shim，调用审计和 entity census 必须证明没有真实 participant。不得是 binary/shim 不存在、PATH 同名程序、fixture/collection error 或 skip。

- [ ] **Step 8: 确认 wiring RED 的失败原因正确**

首个失败必须明确指出某个真实进程缺少期望 control frame、状态/ACK/fence 不匹配或 barrier 行为未接线。若失败来自 target、动态库、shim child 注入/IPC/符号 allowlist/调用审计、entity census、socket 权限或测试收集，先修基础设施并原样重跑 Step 7。

- [ ] **Step 9: 最小实现三个进程的控制状态接线**

```text
STARTING -> READY -> ACTIVE
STARTING -> FAILED
ACTIVE -> ROTATING -> ACTIVE
ACTIVE -> DRAINING -> FINALIZED
READY|ACTIVE|ROTATING|DRAINING -> FAILED
```

本 Task 才修改 Subscriber、Command、Recorder 的 source/main，把前面已通过的纯 core 连接到唯一 control codec/socket。每个进程在 socket 身份校验后立即发 STARTING，并在 discovery/metadata 校验期间用该 state 更新逐 topic health；READY 前不得报告 ACTIVE 或触发 Simulator 开门。每条状态使用已冻结 `ControlEnvelope` 携带 simulation session、descriptor SHA、request id、queued messages/bytes、written count、逐 topic health、last identity/fence 和错误码。Subscriber 负责四个只读输入的健康，Command 负责 WheelState gate、唯一 publisher 与最终零命令 fence，Recorder 负责五 topic、共享 ledger、轮转和 durability 状态；main 只做进程生命周期与 adapter 组装，不能重写 core 决策。Manual target、Command scene freeze/resume、Scene attachment、begin drain、segment cut/barrier 和 end barrier 必须收到 request id 精确匹配的 ACK；FAILED 由编排器触发安全停车和有序停止，不能靠 output subscriber count 猜 Recorder 是否存活。

- [ ] **Step 10: 运行 codec 与生产 wiring GREEN**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target test_control_socket control_golden slope-sim-sub slope-sim-command slope-sim-record stage4_ecal_test_shim && "$STAGE4_CTEST" --preset stage4-dev -R '^(control_socket|control_golden)$' --output-on-failure --no-tests=error`

Run: `STAGE4_SUBSCRIBER_BINARY="$PWD/build/stage4-dev/bin/slope-sim-sub" STAGE4_COMMAND_BINARY="$PWD/build/stage4-dev/bin/slope-sim-command" STAGE4_RECORDER_BINARY="$PWD/build/stage4-dev/bin/slope-sim-record" STAGE4_ECAL_TEST_SHIM="$PWD/build/stage4-dev/lib/libstage4_ecal_test_shim.so" conda run -n slope-sim python -m pytest -q -m "not ecal" tests/stage4/test_control_protocol.py tests/stage4/test_cpp_control_processes.py tests/stage4/test_recorder_end_barrier.py tests/stage4/test_recorder_segment_rotation.py tests/test_ecal_process_roundtrip.py`

Expected: 非 eCAL 测试全部 PASS；Python/C++ 控制 bytes、ACK/fence 和状态迁移完全一致，三个生产 ELF 均由绝对路径实际启动并只在 child ABI 边界使用 test-only shim，真实 eCAL entity census 增量为 0；STARTING 能携带完整 pre-READY health 但不打开业务门，核心正式门在缺任一 READY 时保持关闭。`tests/test_ecal_process_roundtrip.py` 中四个 `@pytest.mark.ecal` 真实跨进程用例必须被明确 deselect，不能在 Task 8 逐条授权前运行。

- [ ] **Step 11: REFACTOR 或记录无必要**

只整理三个 adapter 与 control codec 间已经出现的共用装配；若无必要，记录“REFACTOR：无必要”。不得把 role-specific health 合并成无法审计的布尔值。

- [ ] **Step 12: 原样复验两个循环**

原样重跑 Step 5 的 codec GREEN 两条命令和 Step 10 的 wiring GREEN 两条命令；不得省略生产 target、绝对 binary/shim 路径、`-m "not ecal"`、零 entity 断言或任一 barrier 测试。

## Task 8：五 topic 三方 oracle 与真实 eCAL+C++ 门禁

**Files:**
- Create: `scripts/verify_stage4_ecal_cpp.py`
- Create: `tests/stage4/test_stage4_ecal_cpp_verifier.py`
- Modify: `docs/阶段四交付报告.md`

- [ ] **Step 1: 用 fixture 写全向集合与健康 oracle RED**

```python
OUTPUT_KEY = (
    "simulation_session_id", "topic", "timestamp_ns", "world_generation",
    "sequence", "descriptor_sha256", "payload_sha256",
)
COMMAND_KEY = (
    "simulation_session_id", "timestamp_ns", "world_generation",
    "command_generation", "sequence", "source_id", "source_session_id",
    "descriptor_sha256", "payload_sha256",
)


def test_result_exposes_final_session_manifest_by_absolute_path(result) -> None:
    manifest = Path(result["final_session_manifest_path"])
    assert manifest.is_absolute()
    assert manifest.name.endswith(".manifest.pb")
    assert not manifest.name.endswith(".partial")
    assert sha256(manifest.read_bytes()).hexdigest() \
        == result["final_session_manifest_sha256"]


def test_result_binds_the_actual_core_workload(result) -> None:
    assert result["runtime_mode"] == "headless"
    assert result["pybullet_connection_mode"] == "DIRECT"
    assert result["dashboard_enabled"] is False
    assert result["terrain_model"] == "golf_heightfield"
    assert result["obstacle_count"] == 20
    assert result["lidar_profile"] == "realtime_mid360"
    assert result["lidar_candidate_ray_count"] == 5760
```

Simulator↔C++ Subscriber↔完整 session manifest 指向的全部 MCAP segment 对四输出做三份双向集合和顺序比较；Command Tool↔Simulator↔同一完整记录对命令做同样比较。MCAP 证据必须先通过业务 raw record 与 `RecordMetadata` 一一配对，再跨 segment 合并，不能只读最后一段或从 Channel 静态 metadata 补猜动态字段。Fixture 分别注入首帧缺失、尾帧缺失、单中间 gap、extra、duplicate、hash 变异、raw/metadata 失配、缺段和错误 fence，全部必须失败。

verifier JSON 顶层固定输出 `final_session_manifest_path` 和 `final_session_manifest_sha256`：路径必须经 `resolve(strict=True)` 得到绝对最终 `.manifest.pb`，摘要是该文件原始 bytes 的 64 位小写十六进制 SHA-256。还输出 `runtime_manifest_path/runtime_manifest_sha256` 和按 manifest 顺序展开的 segment 绝对路径/hash；写结果前重新调用正式 Reader 完整验证，拒绝 `.partial`、相对路径、文件在验证后被替换、摘要不匹配或 segment 不完整。C 的入口要求显式 `--runtime-mode headless`，结果必须由实际连接状态证明 `pybullet_connection_mode=DIRECT` 且 `dashboard_enabled=false`；本计划只证明 13.3 核心链路，不得把它登记成 13.4 GUI 联合性能。正式 workload 的 `terrain_model/obstacle_count/lidar_profile/lidar_candidate_ray_count` 必须来自 Simulator 已提交场景和每帧扫描统计，不能回显 CLI 请求值；fixture 注入请求模式与实际模式不符、“请求 20 但实际 19 个障碍物”、错误 LiDAR profile 和非 5,760 候选射线，oracle 必须失败。D 的真实 replay/export 只消费这个证据合同，不猜 `results/` 下的文件名。

测试函数先断言 verifier 脚本存在，再通过本地 fixture 调用其纯比较入口；不得在模块顶层 import 尚未创建的脚本，也不得让 fixture setup 因缺文件报 ERROR。

- [ ] **Step 2: 运行 verifier RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_ecal_cpp_verifier.py`

Expected: pytest 正常收集并 `FAILED`，失败断言指向三方比较、pair/fence 或健康判定函数尚不存在；不得启动真实 eCAL。

- [ ] **Step 3: 确认 verifier RED 的失败原因正确**

首个失败必须明确指向 wished-for 三方比较、pair/fence、证据路径或健康判定函数；collection/fixture error、脚本路径拼错、skip 或启动真实 eCAL 都不算 RED。修正测试壳后原样重跑 Step 2。

- [ ] **Step 4: 最小实现三方比较、频率、运动和健康 oracle**

同一 fixture 覆盖实际场景为 `golf_heightfield`、障碍物精确 20、LiDAR profile 为 `realtime_mid360` 且每个正式帧候选射线精确 5,760；command/wheel 墙钟 95..105Hz、timestamp 99..101Hz、最大 gap 30ms；三传感器墙钟 9..11Hz、timestamp 9.9..10.1Hz、最大 gap 250ms；RTK/轨迹 >0.5m、平均速度 >0.1m/s、主动转向两轮峰值 >0.1rad、最终所有 raw/pending/ordered READY/REJECTED/DEFERRED/rotation holding 为 0、`settled_frontier` 已越过最后分配 order、worker/clean shutdown/finalized 为 true。正式比较窗口固定为 `(start_sim_ns, end_sim_ns]`，并要求 WheelCommand、WheelState、LiDAR、RTK、IMU 各有一条同 topic、同 generation 且 `timestamp_ns > end_sim_ns` 的 fence；Command fence 必须来自仍在 100 Hz 运行的零命令 drain，不能用退出前旧帧代替。

- [ ] **Step 5: 运行 verifier GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_ecal_cpp_verifier.py`

Expected: PASS；所有缺帧、extra、duplicate、hash 变异、pair 失配、缺段、错误 fence、频率/运动/健康越界 fixture 都精确失败。

- [ ] **Step 6: REFACTOR 或记录无必要**

只整理已经通过 fixture 的证据解析/集合比较重复；若无必要，记录“REFACTOR：无必要”。

- [ ] **Step 7: 原样复验 verifier**

原样重跑 Step 5 命令；不得删 fixture、放宽路径/hash/fence 断言或启动真实 eCAL。

- [ ] **Step 8: 构建并冻结本计划的开发安装来源**

Run: `"$STAGE4_CMAKE" --build --preset stage4-dev --target slope-sim-sub slope-sim-command slope-sim-record && "$STAGE4_CMAKE" --install build/stage4-dev --prefix "$PWD/build/stage4-dev-install" && bash packaging/stage_cpp_runtime.sh --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" --project-prefix "$PWD/build/stage4-dev-install" --mode sdk && conda run -n slope-sim python scripts/verify_stage4_dependencies.py --install-prefix "$PWD/build/stage4-dev-install" --build-kind development --write-runtime-manifest "$PWD/build/stage4-dev-install/share/slope-sim/runtime-manifest.json"`

Expected: 三个 ELF、`libslope_sim_client` 和全部非系统依赖 closure 都来自该 install tree，runtime manifest 是规范 JSON且其 Git/descriptor/lock/ABI/ELF hash 与该树一致；install tree、CMake install manifest 和 runtime manifest 均不含 `stage4_ecal_test_shim`、测试 IPC、注入变量或测试选择开关。把 install tree 搬到新绝对路径、清空 `LD_LIBRARY_PATH` 并收窄 PATH 后，三个 CLI 和已构建 consumer example 仍可运行。verifier 拒绝 PATH 中同名程序、build tree 程序、解析回 dependency prefix 的 DSO、descriptor SHA 不同、任何测试注入痕迹或 runtime manifest 缺失/摘要不符的 prefix。

- [ ] **Step 9: 为 `4+2` 这一条 invocation 单独取得授权并预检**

此步骤必须在用户明确授权后执行；授权只覆盖紧随其后的 `4+2` 命令。先即时扫描全机 pytest、GUI/Xvfb、PyBullet、eCAL 和 C++ participant，确认无竞争负载；发现负载则不消费授权，等待后重新确认。失败后保留证据并停止，不自动重跑，也不继续 `2+0`。

- [ ] **Step 10: 运行唯一一次 `4+2`**

```bash
env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD conda run -n slope-sim python scripts/verify_stage4_ecal_cpp.py --client-prefix "$PWD/build/stage4-dev-install" --runtime-mode headless --robot-model active_steering_4wd --terrain-model golf_heightfield --obstacle-count 20 --warmup-sec 1 --duration-sec 5 --output results/stage4/cpp-gate-4plus2.json
```

Expected: rc=0、结果证明实际运行 `headless/DIRECT/dashboard_enabled=false`、`golf_heightfield`、20 个障碍物和每帧 5,760 候选射线，三方消息完全一致、零 transport/consumer/Recorder drop/error、运动与频率全部达标；JSON 中的最终 session manifest 绝对路径、SHA 和全部 segment 可重新读取且匹配。该结果只属于核心链路，不替代 E 的 interactive Dashboard 联合门禁。失败即停止并登记本次唯一证据。

- [ ] **Step 11: 为 `2+0` 这一条 invocation 重新取得授权并重新预检**

只有 Step 10 的 `4+2` 已通过才可申请；不得沿用 Step 9 的授权。再次说明车型、5 秒窗口和失败不重跑规则，并重新扫描主机静默状态。

- [ ] **Step 12: 运行唯一一次 `2+0`**

```bash
env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD conda run -n slope-sim python scripts/verify_stage4_ecal_cpp.py --client-prefix "$PWD/build/stage4-dev-install" --runtime-mode headless --robot-model df_back --terrain-model golf_heightfield --obstacle-count 20 --warmup-sec 1 --duration-sec 5 --output results/stage4/cpp-gate-2plus0.json
```

Expected: rc=0、结果证明实际运行 `headless/DIRECT/dashboard_enabled=false`、`golf_heightfield`、20 个障碍物和每帧 5,760 候选射线，三方消息完全一致、零 transport/consumer/Recorder drop/error、运动与频率全部达标；JSON 中的最终 session manifest 绝对路径、SHA 和全部 segment 可重新读取且匹配。两条均通过后才把原始 MCAP、三进程 JSONL 和 verifier JSON 一并登记到交付报告，且仍保留 E 的 13.4 联合性能为未执行。
