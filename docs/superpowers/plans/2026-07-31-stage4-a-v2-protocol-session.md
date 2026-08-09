# 阶段四 A：v2 协议、会话与命令权 Implementation Plan

> **Execution:** Use `subagent-driven-development` only when the user selects delegated execution; otherwise use `executing-plans`. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 冻结阶段三 v1，在不伪造兼容性的前提下建立 Python/C++ 共用的 Protobuf v2、仿真会话与双 generation、单命令源状态机，并用真实 eCAL 6.1.1 证明原始字节和远端类型元数据可可靠隔离。

**Architecture:** `proto/slope_sim_interfaces_v2.proto` 是唯一 v2 消息源，Python 先把不可变模型确定性序列化一次，日志与 raw eCAL 共用同一份 bytes；`ProtocolSession` 只管理 simulation session、world generation 和输出序号，`CommandAuthority` 独立管理 command generation、精确 peer count 与 owner。阶段 A 先完成独立 Python/C++ Phase-0 硬门，再把通过验证的 raw transport 能力接入生产边界；单中心 LiDAR、三点 RTK 和五话题生产 runtime 的最终切换由阶段 B 完成，阶段 A 不生成半正确的传感器数据。

**Tech Stack:** Python 3.10、Python Protobuf runtime 6.33.6、独立 protoc/libprotobuf 33.6、Eclipse eCAL 6.1.1 raw pub/sub 与 monitoring API、C++17、GCC 13、CMake 3.28、OpenSSL 3 SHA-256、pytest、CTest。

---

**TDD gate:** 本计划所有生产代码任务遵守总路线的严格 RED-GREEN-REFACTOR 协议；RED 必须是 pytest/CTest 正常收集后的行为断言失败，不能是 collection error、缺工具、skip 或缺构建目录。创建新模块的 RED 测试只在测试函数内通过 `importlib.import_module()` 加载 wished-for API，并把 `ModuleNotFoundError` 转成带明确业务消息的 `pytest.fail(..., pytrace=False)`；不得在测试模块顶层导入尚不存在的包。每个 GREEN 只实现本 Task 的失败行为；REFACTOR 不增加行为，完成后原样复跑该 Task 的 GREEN 命令。

**环境合同前置：** 开始或恢复本计划前必须独立执行，不能依赖总计划所在 shell：

```bash
test -n "${STAGE4_BUILD_ENV_FILE:-}"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-env "$STAGE4_BUILD_ENV_FILE" \
  --json "$STAGE4_BUILD_ENV_FILE.stage4-a-preflight.json"
source "$STAGE4_BUILD_ENV_FILE"
test -x "$STAGE4_CMAKE" && test -x "$STAGE4_CTEST"
test -x "$STAGE4_CC" && test -x "$STAGE4_CXX" && test -x "$STAGE4_PROTOC"
test -d "$STAGE4_DEPENDENCY_PREFIX"
```

Expected: env/evidence hash、工具版本、开发 dependency prefix 与总计划 Task 2 冻结值一致；缺失、未定义或漂移时在 Task 1 创建输出前失败。

下文测试片段为接口主体；凡出现尚未创建的 `slope_sim.interfaces.v2.*` 或生成模块导入，落盘测试时都必须在该 RED 测试文件内定义下面的加载器，随后在测试函数内从返回模块取得类/常量，不新增生产 helper 或跨 Task 隐式 fixture。这个加载器本身不算 RED，RED 必须由同一测试中的明确行为断言或 `pytest.fail` 产生：

```python
from importlib import import_module

import pytest


def require_wished_module(name: str):
    """让缺少待实现模块表现为可读的测试失败，而不是收集错误。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)
```

## 执行边界与停止门

- 基线固定为 Git `ce3bee0`。`proto/slope_sim_interfaces.proto`、package `slope_sim.interfaces.v1` 和阶段三生成 descriptor 不得改写。
- A 只建立协议、会话、命令权、raw transport 证明和跨语言 golden；不修改 LiDAR 射线、RTK 几何、Dashboard、ROS 2、MCAP 或发行包。
- 正式简洁 topic 名只有在 Phase-0 同时证明 raw payload、远端 type/descriptor metadata 和 v1 同话题冲突硬失败后才可保留。
- 任一 Phase-0 条件失败时，立即停止 A 的生产 transport/runtime 接线，在交付报告记录失败证据，并请用户在 `/sim/v2/...` 与其他命名方案之间重新裁决。不得自动改名、只看 peer count、只校验带内 digest，或用 LocalTransport 冒充通过。
- 真实 eCAL 是受控外部门禁。执行该 Task 前必须再次取得用户明确授权、扫描全机负载、严格串行运行一次；失败后不自动重试，也不降低门槛。
- 阶段 A 结束时 v1 回归必须保持通过；阶段 B 接线完成前，阶段三生产入口仍使用 v1，不能声称阶段四五话题 runtime 已交付。

## 文件结构

```text
proto/
  slope_sim_interfaces.proto                       # 冻结 v1
  slope_sim_interfaces_v1.sha256                   # v1 源文件与 descriptor 冻结值
  slope_sim_interfaces_v2.proto                    # 唯一 v2 schema
  slope_sim_interfaces_v2.sha256                   # 首次生成后只读冻结值
slope_sim/interfaces/generated/
  slope_sim_interfaces_v2_pb2.py                   # 脚本生成，不手改
  slope_sim_interfaces_v2.desc                     # FileDescriptorSet 原始 bytes
slope_sim/interfaces/v2/
  __init__.py
  descriptor.py                                    # descriptor bytes/digest 唯一读取入口
  topics.py                                        # 五话题/type/rate 固定合同
  models.py                                        # wheel v2 与会话身份不可变模型
  codec.py                                         # 确定性单次序列化和严格解码
  session.py                                       # session/world/sequence 状态
  authority.py                                     # command generation/owner 状态机
  ecal_raw.py                                      # Python raw eCAL 与 monitoring 边界
  transport.py                                     # 五话题 raw transport factory
  runtime_protocol.py                              # runtime 可组合的协议控制器
cpp/phase0/
  CMakeLists.txt
  ecal_v2_raw_probe.cpp                            # raw bytes/metadata 探针
  v2_golden.cpp                                    # 双向 Protobuf golden 工具
scripts/
  generate_v2_protos.py
  freeze_v2_descriptor.py
  verify_stage4_v2_phase0.py
  generate_stage4_v2_goldens.py
tests/stage4/
  test_v1_descriptor_frozen.py
  test_v2_proto_contract.py
  test_v2_generated_artifacts.py
  test_v2_descriptor.py
  test_v2_codec.py
  test_v2_session.py
  test_command_authority.py
  test_transport_v2_metadata.py
  test_ecal_v2_raw_unit.py
  test_ecal_v2_phase0.py
  test_ecal_v2_transport.py
  test_v2_runtime_protocol.py
  test_cpp_phase0_build.py
  test_cpp_v2_interop.py
```

### Task 1：冻结 v1 源文件与 descriptor

**Files:**
- Create: `proto/slope_sim_interfaces_v1.sha256`
- Create: `tests/stage4/__init__.py`
- Create: `tests/stage4/test_v1_descriptor_frozen.py`
- Test: `tests/test_proto_contract.py`

- [x] **Step 1: 写冻结清单 RED**

```python
# tests/stage4/test_v1_descriptor_frozen.py
from hashlib import sha256
from pathlib import Path

from slope_sim.interfaces.generated import slope_sim_interfaces_pb2 as v1_pb


V1_SOURCE_SHA256 = "9de0e629a6494ea9446893043c7e30ca9d6370868f23def4fcd4f2af5cd102d4"
V1_DESCRIPTOR_SHA256 = "6a524cce7b11ca72f73214394097407c2f8ddc50ea40ca6ffef7be1c248dc2e9"


def test_v1_source_and_descriptor_are_frozen() -> None:
    source = Path("proto/slope_sim_interfaces.proto").read_bytes()
    manifest_path = Path("proto/slope_sim_interfaces_v1.sha256")
    assert manifest_path.is_file(), "v1 SHA-256 manifest is not implemented"
    manifest = manifest_path.read_text(encoding="ascii")
    assert sha256(source).hexdigest() == V1_SOURCE_SHA256
    assert sha256(v1_pb.DESCRIPTOR.serialized_pb).hexdigest() == V1_DESCRIPTOR_SHA256
    assert manifest == (
        f"source {V1_SOURCE_SHA256}\n"
        f"descriptor {V1_DESCRIPTOR_SHA256}\n"
    )
```

- [x] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v1_descriptor_frozen.py`

Expected: pytest 正常收集后 `FAILED`，且唯一失败是 `v1 SHA-256 manifest is not implemented`；不得是文件读取异常、collection error 或 skip。

- [x] **Step 3: 写入已核对的冻结值**

```text
source 9de0e629a6494ea9446893043c7e30ca9d6370868f23def4fcd4f2af5cd102d4
descriptor 6a524cce7b11ca72f73214394097407c2f8ddc50ea40ca6ffef7be1c248dc2e9
```

`tests/stage4/__init__.py` 只写文件头注释：

```python
"""阶段四协议、传感器、跨语言和交付门禁测试。"""
```

- [x] **Step 4: 运行 GREEN 和原 v1 合同**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v1_descriptor_frozen.py tests/test_proto_contract.py`

Expected: `3 passed`，且 `git diff -- proto/slope_sim_interfaces.proto` 无输出。

- [x] **Step 5: REFACTOR 冻结值读取与断言表达**

只合并测试内重复的 SHA-256 读取/格式断言，不改变冻结常量、manifest 字节或 v1 生成物。原样重跑 Step 4 的 GREEN 命令，Expected: `3 passed`，且 v1 proto diff 仍为空。

### Task 2：生成完整 Protobuf v2 与 FileDescriptorSet

**Files:**
- Create: `proto/slope_sim_interfaces_v2.proto`
- Create: `scripts/generate_v2_protos.py`
- Generate: `slope_sim/interfaces/generated/slope_sim_interfaces_v2_pb2.py`
- Generate: `slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc`
- Create: `tests/stage4/test_v2_proto_contract.py`
- Create: `tests/stage4/test_v2_generated_artifacts.py`

- [x] **Step 1: 写 schema 与独立生成产物 oracle RED**

```python
# tests/stage4/test_v2_proto_contract.py
from google.protobuf import descriptor_pb2


TOP_LEVEL = (
    "WheelCommand",
    "WheelState",
    "LidarPointCloud",
    "RtkState",
    "ImuAttitude",
)


def test_v2_package_and_authority_enum_are_exact() -> None:
    pb = require_wished_module(
        "slope_sim.interfaces.generated.slope_sim_interfaces_v2_pb2"
    )
    assert pb.DESCRIPTOR.package == "slope_sim.interfaces.v2"
    enum = pb.DESCRIPTOR.enum_types_by_name["CommandAuthorityState"]
    assert [(v.name, v.number) for v in enum.values] == [
        ("COMMAND_AUTHORITY_UNSPECIFIED", 0),
        ("WAITING", 1),
        ("CLAIMABLE", 2),
        ("ACTIVE", 3),
        ("CONFLICT", 4),
    ]


def test_every_top_level_v2_message_carries_session_and_descriptor() -> None:
    pb = require_wished_module(
        "slope_sim.interfaces.generated.slope_sim_interfaces_v2_pb2"
    )
    for name in TOP_LEVEL:
        fields = pb.DESCRIPTOR.message_types_by_name[name].fields_by_name
        assert fields["simulation_session_id"].type == descriptor_pb2.FieldDescriptorProto.TYPE_BYTES
        assert fields["descriptor_sha256"].type == descriptor_pb2.FieldDescriptorProto.TYPE_BYTES
```

同文件使用以下 oracle 锁定设计稿 4.4-4.7 的字段顺序和号码；每项再从 descriptor 断言 scalar/message type、repeated/optional label 和 `Point3d/LidarPoint/CommandAuthorityState` 的完整 type name：

```python
EXPECTED_FIELDS = {
    "WheelCommand": (
        ("timestamp_ns", 1), ("drive_wheel_speed_rad_s", 2),
        ("steering_wheel_speed_rad_s", 3), ("sequence", 4),
        ("world_generation", 5), ("command_generation", 6),
        ("source_id", 7), ("source_session_id", 8),
        ("robot_model", 9), ("simulation_session_id", 10),
        ("descriptor_sha256", 11),
    ),
    "WheelState": (
        ("timestamp_ns", 1), ("drive_wheel_speed_rad_s", 2),
        ("steering_wheel_angle_rad", 3), ("sequence", 4),
        ("world_generation", 5), ("command_generation", 6),
        ("robot_model", 7), ("simulation_session_id", 8),
        ("descriptor_sha256", 9), ("command_authority_state", 10),
        ("command_owner_source_id", 11),
        ("command_owner_source_session_id", 12), ("command_peer_count", 13),
    ),
    "LidarPoint": (
        ("offset_time_ns", 1), ("x", 2), ("y", 3), ("z", 4),
        ("reflectivity", 5), ("tag", 6), ("line", 7),
    ),
    "LidarPointCloud": (
        ("timebase_ns", 1), ("frame_id", 2), ("point_num", 3),
        ("lidar_id", 4), ("points", 5), ("sequence", 6),
        ("world_generation", 7), ("simulation_session_id", 8),
        ("descriptor_sha256", 9),
    ),
    "Point3d": (("x_m", 1), ("y_m", 2), ("z_m", 3)),
    "RtkState": (
        ("timestamp_ns", 1), ("sequence", 2), ("world_generation", 3),
        ("frame_id", 4), ("left", 5), ("center", 6), ("right", 7),
        ("heading_rad", 8), ("simulation_session_id", 9),
        ("descriptor_sha256", 10),
    ),
    "ImuAttitude": (
        ("timestamp_ns", 1), ("roll_rad", 2), ("pitch_rad", 3),
        ("sequence", 4), ("world_generation", 5), ("frame_id", 6),
        ("simulation_session_id", 7), ("descriptor_sha256", 8),
    ),
}
```

同时先创建 `tests/stage4/test_v2_generated_artifacts.py`。文件顶层只导入标准库和 pytest，不得顶层导入尚未生成的 v2 模块；每个测试在函数内先用 `Path.is_file()` 检查 wished-for schema、生成脚本、Python 模块和 descriptor，任一尚不存在时调用 `pytest.fail("v2 generated artifacts are not implemented", pytrace=False)`，保证首次 RED 是正常收集后的行为失败，而不是 import/subprocess error。

当 wished-for 文件存在时，测试从 `STAGE4_PROTOC` 取得并独立验证 `libprotoc 33.6`，在 pytest 临时目录直接调用该 executable，逐 byte 比较临时 `.desc` 与仓库 `.desc`，并比较临时模块的 `DESCRIPTOR.serialized_pb` 与跟踪模块；测试不得导入生产生成函数、不得调用 `grpc_tools.protoc`，也不得通过读取生产输出常量来自证。

- [x] **Step 2: 运行 RED**

Run: `STAGE4_PROTOC="$STAGE4_PROTOC" conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_proto_contract.py tests/stage4/test_v2_generated_artifacts.py`

Expected: 两个测试文件都正常收集并 `FAILED`；失败消息分别精确指出 v2 generated module 与 v2 generated artifacts 尚未实现，不得是 collection error、fixture error、subprocess 文件错误或 skip。保存该失败输出后才可进入 Step 3。

- [x] **Step 3: 创建唯一 v2 schema**

```proto
// 阶段四企业接口：定义跨 Python/C++ 的会话化 v2 线协议。
syntax = "proto3";

package slope_sim.interfaces.v2;

enum CommandAuthorityState {
  COMMAND_AUTHORITY_UNSPECIFIED = 0;
  WAITING = 1;
  CLAIMABLE = 2;
  ACTIVE = 3;
  CONFLICT = 4;
}

message WheelCommand {
  uint64 timestamp_ns = 1;
  repeated float drive_wheel_speed_rad_s = 2;
  repeated float steering_wheel_speed_rad_s = 3;
  uint64 sequence = 4;
  uint64 world_generation = 5;
  uint64 command_generation = 6;
  string source_id = 7;
  bytes source_session_id = 8;
  string robot_model = 9;
  bytes simulation_session_id = 10;
  bytes descriptor_sha256 = 11;
}

message WheelState {
  uint64 timestamp_ns = 1;
  repeated float drive_wheel_speed_rad_s = 2;
  repeated float steering_wheel_angle_rad = 3;
  uint64 sequence = 4;
  uint64 world_generation = 5;
  uint64 command_generation = 6;
  string robot_model = 7;
  bytes simulation_session_id = 8;
  bytes descriptor_sha256 = 9;
  CommandAuthorityState command_authority_state = 10;
  string command_owner_source_id = 11;
  bytes command_owner_source_session_id = 12;
  uint32 command_peer_count = 13;
}

message LidarPoint {
  uint32 offset_time_ns = 1;
  float x = 2;
  float y = 3;
  float z = 4;
  uint32 reflectivity = 5;
  uint32 tag = 6;
  uint32 line = 7;
}

message LidarPointCloud {
  uint64 timebase_ns = 1;
  string frame_id = 2;
  uint32 point_num = 3;
  uint32 lidar_id = 4;
  repeated LidarPoint points = 5;
  uint64 sequence = 6;
  uint64 world_generation = 7;
  bytes simulation_session_id = 8;
  bytes descriptor_sha256 = 9;
}

message Point3d {
  double x_m = 1;
  double y_m = 2;
  double z_m = 3;
}

message RtkState {
  uint64 timestamp_ns = 1;
  uint64 sequence = 2;
  uint64 world_generation = 3;
  string frame_id = 4;
  Point3d left = 5;
  Point3d center = 6;
  Point3d right = 7;
  double heading_rad = 8;
  bytes simulation_session_id = 9;
  bytes descriptor_sha256 = 10;
}

message ImuAttitude {
  uint64 timestamp_ns = 1;
  double roll_rad = 2;
  double pitch_rad = 3;
  uint64 sequence = 4;
  uint64 world_generation = 5;
  string frame_id = 6;
  bytes simulation_session_id = 7;
  bytes descriptor_sha256 = 8;
}
```

- [x] **Step 4: 新建独立 v2 生成脚本并强制 protoc 33.6**

```python
import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
PROTO_DIR = ROOT / "proto"
OUTPUT_DIR = ROOT / "slope_sim/interfaces/generated"
V2_PROTO = PROTO_DIR / "slope_sim_interfaces_v2.proto"
V2_DESCRIPTOR_SET = OUTPUT_DIR / "slope_sim_interfaces_v2.desc"


def _stage4_protoc() -> Path:
    raw = os.environ.get("STAGE4_PROTOC")
    if not raw:
        raise RuntimeError("STAGE4_PROTOC must point to the frozen protoc 33.6")
    executable = Path(raw).resolve(strict=True)
    version = subprocess.run(
        [str(executable), "--version"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    if version != "libprotoc 33.6":
        raise RuntimeError(f"expected libprotoc 33.6, got {version!r}")
    return executable


def _generate_v2(protoc: Path) -> None:
    subprocess.run([
        str(protoc),
        f"--proto_path={PROTO_DIR}",
        f"--python_out={OUTPUT_DIR}",
        f"--descriptor_set_out={V2_DESCRIPTOR_SET}",
        "--include_imports",
        str(V2_PROTO),
    ], check=True)


def main() -> int:
    """只用冻结的独立 protoc 33.6 生成 v2。"""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    _generate_v2(_stage4_protoc())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

`scripts/generate_protos.py` 与它的历史 v1/internal `grpc_tools.protoc` 流程保持不变；新脚本是 v2 唯一生成入口。测试额外核对独立 compiler 与冻结 libprotobuf 33.6，禁止从 Conda Python wheel、grpcio-tools 或另一套 `libprotobuf.so` 取得 v2 编译器/runtime。

- [x] **Step 5: 生成最小实现并运行可复现 GREEN**

Run: `test -x "$STAGE4_PROTOC" && test "$("$STAGE4_PROTOC" --version)" = "libprotoc 33.6"`

Expected: rc=0；`STAGE4_PROTOC` 必须由总路线依赖门设置为冻结 compiler 的绝对路径，不能用当前 PATH 中碰巧同名的程序。

Run: `STAGE4_PROTOC="$STAGE4_PROTOC" conda run -n slope-sim python scripts/generate_v2_protos.py`

Expected: rc=0，新增 `slope_sim_interfaces_v2_pb2.py` 和非空 `slope_sim_interfaces_v2.desc`，v1 冻结测试继续通过。

Run: `STAGE4_PROTOC="$STAGE4_PROTOC" conda run -n slope-sim python -m pytest -q tests/stage4/test_v1_descriptor_frozen.py tests/stage4/test_v2_proto_contract.py tests/stage4/test_v2_generated_artifacts.py`

Expected: PASS。

- [x] **Step 6: REFACTOR schema oracle 与生成脚本参数构造**

只去除 descriptor 字段 oracle、路径解析和 subprocess 参数构造中的重复代码；不得改变 schema、字段号、protoc 绝对路径门或生成 bytes。原样重跑 Step 5 的三个 Run，Expected 与 Step 5 完全相同。

### Task 3：冻结 v2 descriptor 身份并集中读取

**Files:**
- Create: `scripts/freeze_v2_descriptor.py`
- Create: `proto/slope_sim_interfaces_v2.sha256`
- Create: `slope_sim/interfaces/v2/__init__.py`
- Create: `slope_sim/interfaces/v2/descriptor.py`
- Create: `tests/stage4/test_v2_descriptor.py`

- [x] **Step 1: 写缺少冻结值和篡改拒绝 RED**

```python
from hashlib import sha256

import pytest


def test_descriptor_identity_matches_frozen_manifest() -> None:
    module = require_wished_module("slope_sim.interfaces.v2.descriptor")
    load_v2_descriptor = module.load_v2_descriptor
    identity = load_v2_descriptor()
    assert len(identity.serialized_file_descriptor_set) > 0
    assert len(identity.sha256) == 32
    assert identity.sha256 == sha256(identity.serialized_file_descriptor_set).digest()


def test_descriptor_loader_rejects_manifest_mismatch(tmp_path) -> None:
    module = require_wished_module("slope_sim.interfaces.v2.descriptor")
    load_v2_descriptor = module.load_v2_descriptor
    descriptor = tmp_path / "v2.desc"
    manifest = tmp_path / "v2.sha256"
    descriptor.write_bytes(b"descriptor")
    manifest.write_text("00" * 32 + "\n", encoding="ascii")
    with pytest.raises(RuntimeError, match="descriptor SHA-256 mismatch"):
        load_v2_descriptor(descriptor, manifest)
```

- [x] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_descriptor.py`

Expected: pytest 正常收集后 `FAILED`，失败消息精确指出 descriptor loader 尚未实现；不得是 collection error、fixture error 或 skip。

- [x] **Step 3: 实现一次性冻结脚本**

```python
"""v2 descriptor 冻结工具：首次创建，后续只校验并拒绝覆盖。"""
from argparse import ArgumentParser
from hashlib import sha256
from pathlib import Path


def freeze(descriptor: Path, manifest: Path, *, create: bool) -> str:
    digest = sha256(descriptor.read_bytes()).hexdigest()
    if create:
        with manifest.open("x", encoding="ascii", newline="\n") as stream:
            stream.write(digest + "\n")
        return digest
    expected = manifest.read_text(encoding="ascii").strip()
    if expected != digest:
        raise RuntimeError(f"descriptor SHA-256 mismatch: {digest} != {expected}")
    return digest
```

CLI 固定参数 `--create`；无参数执行校验。`--create` 遇到已存在 manifest 必须由 `open("x")` 非零退出，防止 schema 变化时顺手改写冻结值。

- [x] **Step 4: 首次创建 v2 冻结值**

Run: `conda run -n slope-sim python scripts/freeze_v2_descriptor.py --create`

Expected: rc=0，标准输出是一行 64 位小写十六进制；该值必须与 `sha256sum slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc` 完全一致。以后只运行不带 `--create` 的校验模式。

- [x] **Step 5: 建立运行时唯一读取入口**

```python
# slope_sim/interfaces/v2/descriptor.py
"""阶段四 descriptor 身份：集中校验 FileDescriptorSet 与冻结 SHA-256。"""
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path


@dataclass(frozen=True)
class DescriptorIdentity:
    serialized_file_descriptor_set: bytes
    sha256: bytes


_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_DESCRIPTOR = _ROOT / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
_DEFAULT_MANIFEST = _ROOT / "proto/slope_sim_interfaces_v2.sha256"


def load_v2_descriptor(
    descriptor_path: Path = _DEFAULT_DESCRIPTOR,
    manifest_path: Path = _DEFAULT_MANIFEST,
) -> DescriptorIdentity:
    """加载并校验冻结 descriptor；不一致时禁止启动 v2。"""
    payload = descriptor_path.read_bytes()
    digest = sha256(payload).digest()
    expected = bytes.fromhex(manifest_path.read_text(encoding="ascii").strip())
    if len(expected) != 32 or digest != expected:
        raise RuntimeError("v2 descriptor SHA-256 mismatch")
    return DescriptorIdentity(payload, digest)
```

- [x] **Step 6: 运行 GREEN 与覆盖保护**

Run: `STAGE4_PROTOC="$STAGE4_PROTOC" conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_descriptor.py tests/stage4/test_v2_generated_artifacts.py`

Expected: PASS。

Run: `conda run -n slope-sim python scripts/freeze_v2_descriptor.py --create`

Expected: rc!=0，错误为 manifest 已存在；已有冻结文件内容不变。

- [x] **Step 7: REFACTOR descriptor 路径与 digest 校验**

只共用“读取 bytes -> 计算 digest -> 校验 32-byte identity”的内部逻辑；`--create` exclusive-create 语义和运行时失败类型不变。原样重跑 Step 6 的 GREEN 与覆盖保护命令，Expected 与 Step 6 完全相同。

### Task 4：建立五话题合同、不可变 wheel v2 模型和确定性 codec

**Files:**
- Create: `slope_sim/interfaces/v2/topics.py`
- Create: `slope_sim/interfaces/v2/models.py`
- Create: `slope_sim/interfaces/v2/codec.py`
- Create: `tests/stage4/test_v2_codec.py`
- Test: `tests/test_interface_codec.py`

- [ ] **Step 1: 写话题、严格字段和“同一 bytes”RED**

```python
from hashlib import sha256

import pytest


SESSION = bytes.fromhex("00112233445566778899aabbccddeeff")
SOURCE_SESSION = bytes.fromhex("ffeeddccbbaa99887766554433221100")


def test_v2_topic_contract_is_exact() -> None:
    V2_TOPICS = require_wished_module("slope_sim.interfaces.v2.topics").V2_TOPICS
    assert [(c.topic, c.type_name, c.rate_hz, c.direction) for c in V2_TOPICS] == [
        ("/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand", 100, "subscribe"),
        ("/sim/wheel/state", "slope_sim.interfaces.v2.WheelState", 100, "publish"),
        ("/sim/lidar/points", "slope_sim.interfaces.v2.LidarPointCloud", 10, "publish"),
        ("/sim/rtk/state", "slope_sim.interfaces.v2.RtkState", 10, "publish"),
        ("/sim/imu/attitude", "slope_sim.interfaces.v2.ImuAttitude", 10, "publish"),
    ]


def test_encode_returns_one_deterministic_payload_for_log_and_transport(descriptor) -> None:
    V2ProtoCodec = require_wished_module(
        "slope_sim.interfaces.v2.codec"
    ).V2ProtoCodec
    WheelCommandV2 = require_wished_module(
        "slope_sim.interfaces.v2.models"
    ).WheelCommandV2
    codec = V2ProtoCodec(descriptor)
    command = WheelCommandV2(
        timestamp_ns=20_000_000,
        drive_wheel_speed_rad_s=(1.25, -1.25),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=1,
        command_generation=1,
        source_id="manual.tool-1",
        source_session_id=SOURCE_SESSION,
        robot_model="df_mid",
        simulation_session_id=SESSION,
        descriptor_sha256=descriptor.sha256,
    )
    first = codec.encode(command)
    second = codec.encode(command)
    assert first.payload == second.payload
    assert first.payload_sha256 == sha256(first.payload).digest()
    assert first.type_name == "slope_sim.interfaces.v2.WheelCommand"
    assert codec.decode_wheel_command(first.payload) == command


@pytest.mark.parametrize("field,value", [
    ("simulation_session_id", b"short"),
    ("descriptor_sha256", b"short"),
    ("source_session_id", b"short"),
    ("source_id", "bad source"),
    ("source_id", "x" * 65),
])
def test_wheel_command_rejects_invalid_identity(field: str, value: object, descriptor) -> None:
    WheelCommandV2 = require_wished_module(
        "slope_sim.interfaces.v2.models"
    ).WheelCommandV2
    values = valid_command_values(descriptor)
    values[field] = value
    with pytest.raises(ValueError):
        WheelCommandV2(**values)
```

测试还必须覆盖 uint64/uint32 边界、bool 冒充 int、NaN/Inf、非 ASCII source、空 robot model、ACTIVE 缺 owner、非 ACTIVE 带 owner，以及 decode 时错误带内 descriptor 在 sequence 统计前被拒绝。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_codec.py`

Expected: pytest 正常收集后 `FAILED`，首个失败消息精确指出 topics/models/codec 中第一个尚未实现的行为；不得是 collection error、fixture error 或 skip。

- [ ] **Step 3: 固定五话题数据**

```python
# slope_sim/interfaces/v2/topics.py
"""阶段四五话题合同：集中定义方向、类型、频率和连续性范围。"""
from dataclasses import dataclass


@dataclass(frozen=True)
class V2TopicContract:
    topic: str
    type_name: str
    rate_hz: int
    direction: str


V2_TOPICS = (
    V2TopicContract("/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand", 100, "subscribe"),
    V2TopicContract("/sim/wheel/state", "slope_sim.interfaces.v2.WheelState", 100, "publish"),
    V2TopicContract("/sim/lidar/points", "slope_sim.interfaces.v2.LidarPointCloud", 10, "publish"),
    V2TopicContract("/sim/rtk/state", "slope_sim.interfaces.v2.RtkState", 10, "publish"),
    V2TopicContract("/sim/imu/attitude", "slope_sim.interfaces.v2.ImuAttitude", 10, "publish"),
)
V2_OUTPUT_TOPICS = tuple(contract.topic for contract in V2_TOPICS if contract.direction == "publish")
V2_BY_TOPIC = {contract.topic: contract for contract in V2_TOPICS}
```

构造后断言 `len(V2_BY_TOPIC) == len(V2_TOPICS)`，避免重复 topic 被字典静默覆盖。

- [ ] **Step 4: 实现 wheel v2 模型的完整身份边界**

```python
# slope_sim/interfaces/v2/models.py
"""阶段四 wheel 协议模型：冻结会话、代际、命令来源和命令权回显。"""
from dataclasses import dataclass
from enum import IntEnum
import math
import re
from numbers import Real

from slope_sim.interfaces.models import WheelCommand


_UINT32_MAX = (1 << 32) - 1
_UINT64_MAX = (1 << 64) - 1
_SOURCE_ID = re.compile(r"[A-Za-z0-9._-]{1,64}\Z")


class CommandAuthorityState(IntEnum):
    WAITING = 1
    CLAIMABLE = 2
    ACTIVE = 3
    CONFLICT = 4


def require_fixed_bytes(name: str, value: object, length: int) -> bytes:
    if not isinstance(value, (bytes, bytearray, memoryview)):
        raise ValueError(f"{name} must be bytes-like")
    copied = bytes(value)
    if len(copied) != length:
        raise ValueError(f"{name} must be exactly {length} bytes")
    return copied


def require_uint(name: str, value: object, maximum: int = _UINT64_MAX) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ValueError(f"{name} is outside its unsigned integer range")
    return value


def require_float_tuple(name: str, value: object) -> tuple[float, ...]:
    if not isinstance(value, (tuple, list)):
        raise ValueError(f"{name} must be an ordered sequence")
    if any(
        isinstance(item, bool)
        or not isinstance(item, Real)
        or not math.isfinite(float(item))
        for item in value
    ):
        raise ValueError(f"{name} must contain finite numbers")
    return tuple(float(item) for item in value)


@dataclass(frozen=True)
class WheelCommandV2:
    timestamp_ns: int
    drive_wheel_speed_rad_s: tuple[float, ...]
    steering_wheel_speed_rad_s: tuple[float, ...]
    sequence: int
    world_generation: int
    command_generation: int
    source_id: str
    source_session_id: bytes
    robot_model: str
    simulation_session_id: bytes
    descriptor_sha256: bytes

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", require_uint("timestamp_ns", self.timestamp_ns))
        object.__setattr__(self, "sequence", require_uint("sequence", self.sequence))
        for name in ("world_generation", "command_generation"):
            value = require_uint(name, getattr(self, name))
            if value == 0:
                raise ValueError(f"{name} must be positive")
        if not isinstance(self.source_id, str) or _SOURCE_ID.fullmatch(self.source_id) is None:
            raise ValueError("source_id must match [A-Za-z0-9._-]{1,64}")
        if not isinstance(self.robot_model, str) or not self.robot_model:
            raise ValueError("robot_model must be nonempty")
        object.__setattr__(self, "source_session_id", require_fixed_bytes("source_session_id", self.source_session_id, 16))
        object.__setattr__(self, "simulation_session_id", require_fixed_bytes("simulation_session_id", self.simulation_session_id, 16))
        object.__setattr__(self, "descriptor_sha256", require_fixed_bytes("descriptor_sha256", self.descriptor_sha256, 32))
        object.__setattr__(self, "drive_wheel_speed_rad_s", require_float_tuple("drive_wheel_speed_rad_s", self.drive_wheel_speed_rad_s))
        object.__setattr__(self, "steering_wheel_speed_rad_s", require_float_tuple("steering_wheel_speed_rad_s", self.steering_wheel_speed_rad_s))

    def to_v1_motion(self) -> WheelCommand:
        """只转换轮子运动值供既有 mailbox 复用，不丢弃 v2 权限校验。"""
        return WheelCommand(
            timestamp_ns=self.timestamp_ns,
            drive_wheel_speed_rad_s=self.drive_wheel_speed_rad_s,
            steering_wheel_speed_rad_s=self.steering_wheel_speed_rad_s,
        )


@dataclass(frozen=True)
class WheelStateV2:
    timestamp_ns: int
    drive_wheel_speed_rad_s: tuple[float, ...]
    steering_wheel_angle_rad: tuple[float, ...]
    sequence: int
    world_generation: int
    command_generation: int
    robot_model: str
    simulation_session_id: bytes
    descriptor_sha256: bytes
    command_authority_state: CommandAuthorityState
    command_owner_source_id: str
    command_owner_source_session_id: bytes
    command_peer_count: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp_ns", require_uint("timestamp_ns", self.timestamp_ns))
        object.__setattr__(self, "sequence", require_uint("sequence", self.sequence))
        for name in ("world_generation", "command_generation"):
            if require_uint(name, getattr(self, name)) == 0:
                raise ValueError(f"{name} must be positive")
        object.__setattr__(self, "command_peer_count", require_uint("command_peer_count", self.command_peer_count, _UINT32_MAX))
        object.__setattr__(self, "simulation_session_id", require_fixed_bytes("simulation_session_id", self.simulation_session_id, 16))
        object.__setattr__(self, "descriptor_sha256", require_fixed_bytes("descriptor_sha256", self.descriptor_sha256, 32))
        object.__setattr__(self, "drive_wheel_speed_rad_s", require_float_tuple("drive_wheel_speed_rad_s", self.drive_wheel_speed_rad_s))
        object.__setattr__(self, "steering_wheel_angle_rad", require_float_tuple("steering_wheel_angle_rad", self.steering_wheel_angle_rad))
        if not isinstance(self.robot_model, str) or not self.robot_model:
            raise ValueError("robot_model must be nonempty")
        if type(self.command_authority_state) is not CommandAuthorityState:
            raise ValueError("command_authority_state must be a CommandAuthorityState")
        count_matches_state = {
            CommandAuthorityState.WAITING: self.command_peer_count == 0,
            CommandAuthorityState.CLAIMABLE: self.command_peer_count == 1,
            CommandAuthorityState.ACTIVE: self.command_peer_count == 1,
            CommandAuthorityState.CONFLICT: self.command_peer_count > 1,
        }
        if not count_matches_state[self.command_authority_state]:
            raise ValueError("command authority state does not match exact peer count")
        if not isinstance(self.command_owner_source_id, str):
            raise ValueError("command_owner_source_id must be a string")
        if not isinstance(
            self.command_owner_source_session_id,
            (bytes, bytearray, memoryview),
        ):
            raise ValueError("command_owner_source_session_id must be bytes-like")
        owner_session = bytes(self.command_owner_source_session_id)
        active = self.command_authority_state is CommandAuthorityState.ACTIVE
        if active:
            if _SOURCE_ID.fullmatch(self.command_owner_source_id) is None:
                raise ValueError("ACTIVE requires a valid owner source_id")
            owner_session = require_fixed_bytes(
                "command_owner_source_session_id", owner_session, 16
            )
        elif self.command_owner_source_id or owner_session:
            raise ValueError("non-ACTIVE state must not expose an owner")
        object.__setattr__(self, "command_owner_source_session_id", owner_session)
```

将已有 `validate_wheel_command()` 用于车型数组长度和机械限位，不在 v2 模型中复制车型注册表规则。测试必须逐态锁定 `WAITING=0/CLAIMABLE=1/ACTIVE=1/CONFLICT>1`，拒绝状态与精确 peer count 不一致的 WheelState。

- [ ] **Step 5: 实现确定性编码和 descriptor-first 解码**

```python
# slope_sim/interfaces/v2/codec.py
"""阶段四 Protobuf codec：确定性序列化一次并保留原始 payload hash。"""
from dataclasses import dataclass
from hashlib import sha256

from google.protobuf.message import DecodeError, Message

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.models import WheelCommandV2, WheelStateV2


@dataclass(frozen=True)
class EncodedV2Frame:
    type_name: str
    payload: bytes
    payload_sha256: bytes


class V2ProtoCodec:
    def __init__(self, descriptor: DescriptorIdentity) -> None:
        self._descriptor = descriptor

    def encode(self, model: WheelCommandV2 | WheelStateV2) -> EncodedV2Frame:
        if not isinstance(model, (WheelCommandV2, WheelStateV2)):
            raise TypeError(f"unsupported v2 model: {type(model).__name__}")
        if model.descriptor_sha256 != self._descriptor.sha256:
            raise ValueError("v2 descriptor SHA-256 mismatch")
        if isinstance(model, WheelCommandV2):
            message = pb.WheelCommand(
                timestamp_ns=model.timestamp_ns,
                drive_wheel_speed_rad_s=model.drive_wheel_speed_rad_s,
                steering_wheel_speed_rad_s=model.steering_wheel_speed_rad_s,
                sequence=model.sequence,
                world_generation=model.world_generation,
                command_generation=model.command_generation,
                source_id=model.source_id,
                source_session_id=model.source_session_id,
                robot_model=model.robot_model,
                simulation_session_id=model.simulation_session_id,
                descriptor_sha256=model.descriptor_sha256,
            )
        else:
            message = pb.WheelState(
                timestamp_ns=model.timestamp_ns,
                drive_wheel_speed_rad_s=model.drive_wheel_speed_rad_s,
                steering_wheel_angle_rad=model.steering_wheel_angle_rad,
                sequence=model.sequence,
                world_generation=model.world_generation,
                command_generation=model.command_generation,
                robot_model=model.robot_model,
                simulation_session_id=model.simulation_session_id,
                descriptor_sha256=model.descriptor_sha256,
                command_authority_state=int(model.command_authority_state),
                command_owner_source_id=model.command_owner_source_id,
                command_owner_source_session_id=model.command_owner_source_session_id,
                command_peer_count=model.command_peer_count,
            )
        payload = message.SerializeToString(deterministic=True)
        return EncodedV2Frame(message.DESCRIPTOR.full_name, payload, sha256(payload).digest())

    def _parse(self, payload: object, message: Message) -> Message:
        if not isinstance(payload, (bytes, bytearray, memoryview)):
            raise TypeError("v2 payload must be bytes-like")
        try:
            message.ParseFromString(bytes(payload))
        except DecodeError as error:
            raise ValueError("failed to decode v2 payload") from error
        if bytes(message.descriptor_sha256) != self._descriptor.sha256:
            raise ValueError("v2 descriptor SHA-256 mismatch")
        if len(message.simulation_session_id) != 16:
            raise ValueError("simulation_session_id must be exactly 16 bytes")
        return message
```

`decode_wheel_command()` 和 `decode_wheel_state()` 必须显式逐字段构造上述 dataclass；禁止 `MessageToDict`、反射式字段复制或 typed callback 后再次序列化。测试另用同长度错误 digest 证明 encode/decode 均在产出或连续性统计前拒绝。`WheelState` enum 值 0 或未知值必须在模型构造前拒绝。

- [ ] **Step 6: 运行 GREEN 和 v1 codec 回归**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_codec.py tests/test_interface_codec.py`

Expected: PASS，v1 `ProtoCodec` 源码和行为均不改；确定性单次序列化只由新的 `V2ProtoCodec` 承担，不能借阶段 A 顺手改写仍在生产使用的阶段三 codec。

- [ ] **Step 7: REFACTOR v2 字段校验与 codec 映射**

只合并 uint/fixed-bytes/有限数组校验和 message/model 映射中的重复代码；不得放宽精确类型、改变验证顺序或产生第二次序列化。原样重跑 Step 6 的 GREEN 命令，Expected: PASS，v1 codec 回归仍通过。

### Task 5：实现 simulation session、world generation 与输出 sequence

**Files:**
- Create: `slope_sim/interfaces/v2/session.py`
- Create: `tests/stage4/test_v2_session.py`

- [ ] **Step 1: 写会话重启、预留 gap 和重建事务 RED**

```python
import pytest


def test_process_restart_never_reuses_simulation_session(descriptor) -> None:
    ProtocolSession = require_wished_module(
        "slope_sim.interfaces.v2.session"
    ).ProtocolSession
    first = ProtocolSession(descriptor)
    second = ProtocolSession(descriptor)
    assert len(first.simulation_session_id) == 16
    assert first.simulation_session_id != second.simulation_session_id
    assert first.world_generation == second.world_generation == 1


def test_output_sequence_is_reserved_before_work_and_failure_leaves_gap(descriptor) -> None:
    ProtocolSession = require_wished_module(
        "slope_sim.interfaces.v2.session"
    ).ProtocolSession
    session = ProtocolSession(descriptor, session_id_factory=lambda: b"s" * 16)
    first = session.reserve_output("/sim/lidar/points")
    failed = session.reserve_output("/sim/lidar/points")
    third = session.reserve_output("/sim/lidar/points")
    assert (first.sequence, failed.sequence, third.sequence) == (0, 1, 2)


def test_only_successful_commit_advances_world_generation(descriptor) -> None:
    ProtocolSession = require_wished_module(
        "slope_sim.interfaces.v2.session"
    ).ProtocolSession
    session = ProtocolSession(descriptor, session_id_factory=lambda: b"s" * 16)
    session.prepare_world_rebuild()
    assert session.command_generation == 2
    session.abort_world_rebuild()
    assert session.world_generation == 1
    assert session.command_generation == 2
    session.prepare_world_rebuild()
    session.commit_world_rebuild()
    assert session.world_generation == 2
    assert session.command_generation == 3
    assert session.reserve_output("/sim/wheel/state").sequence == 0
```

再覆盖 fault 不恢复旧 token、prepare 重入、未 prepare 的 commit/abort、未知输出 topic、uint64 overflow、默认 factory 的两个实例不重复、注入 factory 返回错误长度，以及四个输出 topic 各自独立 sequence。注入相同固定 session 只用于单实例确定性测试，不虚构跨进程全局去重能力。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_session.py`

Expected: pytest 正常收集后 `FAILED`，失败消息精确指出 `ProtocolSession` 行为尚未实现；不得是 collection error、fixture error 或 skip。

- [ ] **Step 3: 实现独立会话状态机**

```python
# slope_sim/interfaces/v2/session.py
"""阶段四仿真会话：管理进程身份、world/command 代际和逐话题序号。"""
from dataclasses import dataclass
from threading import Lock
from typing import Callable
from uuid import uuid4

from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.models import require_fixed_bytes
from slope_sim.interfaces.v2.topics import V2_OUTPUT_TOPICS


_UINT64_MAX = (1 << 64) - 1


@dataclass(frozen=True)
class OutputIdentity:
    topic: str
    simulation_session_id: bytes
    descriptor_sha256: bytes
    world_generation: int
    sequence: int


class ProtocolSession:
    def __init__(
        self,
        descriptor: DescriptorIdentity,
        *,
        session_id_factory: Callable[[], bytes] = lambda: uuid4().bytes,
    ) -> None:
        self._descriptor = descriptor
        self._simulation_session_id = require_fixed_bytes(
            "simulation_session_id", session_id_factory(), 16
        )
        self._world_generation = 1
        self._command_generation = 1
        self._next_sequence = {topic: 0 for topic in V2_OUTPUT_TOPICS}
        self._rebuild_prepared = False
        self._lock = Lock()

    def reserve_output(self, topic: str) -> OutputIdentity:
        """在传感器读取前占用序号；后续失败也不回收。"""
        with self._lock:
            if topic not in self._next_sequence:
                raise ValueError(f"unknown v2 output topic: {topic}")
            sequence = self._next_sequence[topic]
            if sequence > _UINT64_MAX:
                raise OverflowError("v2 output sequence exhausted")
            self._next_sequence[topic] = sequence + 1
            return OutputIdentity(
                topic,
                self._simulation_session_id,
                self._descriptor.sha256,
                self._world_generation,
                sequence,
            )

    def advance_command_generation(self) -> int:
        with self._lock:
            if self._command_generation == _UINT64_MAX:
                raise OverflowError("command_generation exhausted")
            self._command_generation += 1
            return self._command_generation

    def prepare_world_rebuild(self) -> int:
        with self._lock:
            if self._rebuild_prepared:
                raise RuntimeError("world rebuild is already prepared")
            if self._command_generation == _UINT64_MAX:
                raise OverflowError("command_generation exhausted")
            self._command_generation += 1
            self._rebuild_prepared = True
            return self._command_generation

    def commit_world_rebuild(self) -> int:
        with self._lock:
            if not self._rebuild_prepared:
                raise RuntimeError("world rebuild is not prepared")
            if self._world_generation == _UINT64_MAX:
                raise OverflowError("world_generation exhausted")
            self._world_generation += 1
            self._next_sequence = {topic: 0 for topic in V2_OUTPUT_TOPICS}
            self._rebuild_prepared = False
            return self._world_generation
```

`abort_world_rebuild()` 与 `fault_world_rebuild()` 只把 `_rebuild_prepared=False`，不改 world/command generation；四个只读 property 必须在同一锁下返回 session id、descriptor digest、world 和 command generation。

- [ ] **Step 4: 运行 GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_session.py`

Expected: PASS。

- [ ] **Step 5: REFACTOR session 锁内状态转换**

只提取锁内重复的 generation/sequence 边界检查；不得合并 world 与 command generation，也不得恢复 abort/fault 前 token。原样重跑 Step 4 的 GREEN 命令，Expected: PASS。

### Task 6：实现精确 peer count 的命令权状态机

**Files:**
- Create: `slope_sim/interfaces/v2/authority.py`
- Create: `tests/stage4/test_command_authority.py`
- Test: `tests/test_wheel_mailbox.py`

- [ ] **Step 1: 写四态转换和唯一 owner RED**

```python
from dataclasses import replace

import pytest


_UINT64_MAX = (1 << 64) - 1


def test_peer_edges_advance_generation_once_and_keep_exact_count(session) -> None:
    CommandAuthority = require_wished_module(
        "slope_sim.interfaces.v2.authority"
    ).CommandAuthority
    CommandAuthorityState = require_wished_module(
        "slope_sim.interfaces.v2.models"
    ).CommandAuthorityState
    authority = CommandAuthority(session)
    assert authority.snapshot().state is CommandAuthorityState.WAITING
    authority.observe_peer_count(1)
    assert authority.snapshot().state is CommandAuthorityState.CLAIMABLE
    assert session.command_generation == 1
    authority.observe_peer_count(2)
    assert authority.snapshot().state is CommandAuthorityState.CONFLICT
    assert authority.snapshot().peer_count == 2
    assert session.command_generation == 2
    authority.observe_peer_count(3)
    authority.observe_peer_count(3)
    assert authority.snapshot().peer_count == 3
    assert session.command_generation == 2
    authority.observe_peer_count(1)
    assert authority.snapshot().state is CommandAuthorityState.CLAIMABLE
    assert session.command_generation == 2


def test_first_complete_valid_command_claims_and_other_owner_revokes(
    session, model, valid_command
) -> None:
    CommandAuthority = require_wished_module(
        "slope_sim.interfaces.v2.authority"
    ).CommandAuthority
    CommandAuthorityState = require_wished_module(
        "slope_sim.interfaces.v2.models"
    ).CommandAuthorityState
    authority = CommandAuthority(session)
    authority.observe_peer_count(1)
    accepted = authority.accept(valid_command, model, commit=lambda: True)
    assert accepted.accepted is True
    assert authority.snapshot().state is CommandAuthorityState.ACTIVE
    assert authority.snapshot().owner_source_id == valid_command.source_id

    intruder = replace(
        valid_command,
        source_id="other",
        source_session_id=b"o" * 16,
        sequence=1,
    )
    rejected = authority.accept(intruder, model, commit=lambda: True)
    assert rejected.accepted is False
    assert rejected.clear_mailbox is True
    assert rejected.safe_stop is True
    assert authority.snapshot().state is CommandAuthorityState.CLAIMABLE
    assert authority.snapshot().owner_source_id is None
    assert authority.snapshot().command_generation == 2


@pytest.mark.parametrize("action_name", ("peer_edge", "suspend", "wrong_owner"))
def test_generation_exhaustion_keeps_active_authority_atomic(
    action_name, session, model, valid_command
) -> None:
    CommandAuthority = require_wished_module(
        "slope_sim.interfaces.v2.authority"
    ).CommandAuthority
    CommandAuthorityState = require_wished_module(
        "slope_sim.interfaces.v2.models"
    ).CommandAuthorityState
    authority = CommandAuthority(session)
    authority.observe_peer_count(1)
    assert authority.accept(valid_command, model, commit=lambda: True).accepted

    # 测试专用边界注入：仍使用真实 ProtocolSession 的锁和推进方法。
    with session._lock:
        session._command_generation = _UINT64_MAX
    before = authority.snapshot()
    assert before.state is CommandAuthorityState.ACTIVE
    assert before.owner_source_id == valid_command.source_id
    intruder = replace(
        valid_command,
        command_generation=_UINT64_MAX,
        source_id="other",
        source_session_id=b"o" * 16,
        sequence=1,
    )
    actions = {
        "peer_edge": lambda: authority.observe_peer_count(2),
        "suspend": lambda: authority.suspend_protocol(0),
        "wrong_owner": lambda: authority.accept(
            intruder, model, commit=lambda: pytest.fail("must not commit")
        ),
    }

    with pytest.raises(OverflowError, match="command_generation exhausted"):
        actions[action_name]()

    after = authority.snapshot()
    assert after == before
    assert after.owner_source_session_id == before.owner_source_session_id
```

参数化拒绝表必须覆盖：peer 0/>1、错误 simulation session、descriptor、world generation、command generation、robot model、source 格式/session 长度、首次 sequence 非 0、重复/逆序 sequence、车型数组基数、NaN/Inf 和机械限位。所有拒绝均不得刷新 mailbox 有效墙钟；只有错误 owner 触发撤权和 generation 递增。代际耗尽用真实 `ProtocolSession` 强制到 `_UINT64_MAX`，逐项证明 peer 边沿、协议 suspend 和错误 owner 三条推进路径都抛 `OverflowError`，且包含 owner bytes 在内的 `AuthoritySnapshot` 逐字段保持不变。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_command_authority.py`

Expected: pytest 正常收集后 `FAILED`，失败消息精确指出 `CommandAuthority` 行为尚未实现；不得是 collection error、fixture error 或 skip。

- [ ] **Step 3: 定义不可变快照与结果**

```python
@dataclass(frozen=True)
class AuthoritySnapshot:
    state: CommandAuthorityState
    peer_count: int
    command_generation: int
    owner_source_id: str | None
    owner_source_session_id: bytes | None
    last_sequence: int | None


@dataclass(frozen=True)
class CommandAcceptance:
    accepted: bool
    reason: str
    clear_mailbox: bool = False
    safe_stop: bool = False
    claimed_owner: bool = False
```

- [ ] **Step 4: 实现 peer 边沿和完整命令认领**

```python
class CommandAuthority:
    def __init__(self, session: ProtocolSession) -> None:
        self._session = session
        self._peer_count = 0
        self._state = CommandAuthorityState.WAITING
        self._owner: tuple[str, bytes] | None = None
        self._last_sequence: int | None = None
        self._rebuild_prepared = False
        self._lock = Lock()

    def observe_peer_count(self, count: int) -> CommandAcceptance:
        normalized = require_uint("command_peer_count", count, (1 << 32) - 1)
        with self._lock:
            previous = self._peer_count
            if previous == 1 and normalized != 1:
                # 先取得新 generation；耗尽异常不能留下半次状态转换。
                self._session.advance_command_generation()
                self._peer_count = normalized
                self._owner = None
                self._last_sequence = None
                self._state = (
                    CommandAuthorityState.WAITING
                    if normalized == 0
                    else CommandAuthorityState.CONFLICT
                )
                return CommandAcceptance(False, "command peer edge", True, True)
            self._peer_count = normalized
            if normalized == 0:
                self._state = CommandAuthorityState.WAITING
            elif normalized == 1:
                self._state = (
                    CommandAuthorityState.ACTIVE
                    if self._owner is not None
                    else CommandAuthorityState.CLAIMABLE
                )
            else:
                self._state = CommandAuthorityState.CONFLICT
                self._owner = None
                self._last_sequence = None
            return CommandAcceptance(False, "peer observation")

    def suspend_protocol(self, count: int) -> CommandAcceptance:
        """在 verified 离开边沿撤销命令 token，精确推进一次 generation。"""
        normalized = require_uint("command_peer_count", count, (1 << 32) - 1)
        with self._lock:
            self._session.advance_command_generation()
            self._peer_count = normalized
            self._owner = None
            self._last_sequence = None
            self._state = self._state_for_unowned_peer_count()
            return CommandAcceptance(False, "command protocol suspended", True, True)
```

`_state_for_unowned_peer_count()` 固定返回 `0->WAITING / 1->CLAIMABLE / >1->CONFLICT`。`suspend_protocol()` 只允许由 runtime 在 `verified -> 非 verified` 边沿调用；它自身不做边沿去重，测试直接证明每次调用只推进一次，runtime 测试负责证明重复 pending/conflict poll 不会重复调用。

`accept(command, model, *, commit)` 的执行顺序固定为：重建/peer 状态 -> session id -> descriptor -> world/command generation -> source/robot/数组机械规则 -> owner -> sequence。CLAIMABLE 只允许 `sequence==0`，但必须在 `commit() is True` 后才绑定 `(source_id, source_session_id)`；ACTIVE 同样先要求同 owner 且 `sequence>last_sequence`，再在 commit 成功后推进 last sequence。这样 mailbox 拒绝或旧 ingress 失效不会留下幽灵 ACTIVE owner。错误 owner 在调用 commit 前必须先成功调用 `advance_command_generation()`，成功后才一次性清 owner/sequence 并回到由当前 peer count 推导的状态；推进抛出 `OverflowError` 时，不得先改 owner、sequence、peer count 或 state，也不得调用 commit。

- [ ] **Step 5: 接入 rebuild prepare/commit/abort/fault 合同**

```python
def prepare_world_rebuild(self) -> CommandAcceptance:
    with self._lock:
        self._session.prepare_world_rebuild()
        self._rebuild_prepared = True
        self._owner = None
        self._last_sequence = None
        self._state = self._state_for_unowned_peer_count()
        return CommandAcceptance(False, "world rebuild prepared", True, True)

def commit_world_rebuild(self) -> int:
    with self._lock:
        generation = self._session.commit_world_rebuild()
        self._rebuild_prepared = False
        self._state = self._state_for_unowned_peer_count()
        return generation
```

abort/fault 调用 session 同名方法并恢复“无 owner 的最新 peer 状态”，绝不恢复 prepare 前 owner、sequence 或 command token。所有方法统一先取 authority lock 再调用 session，其他代码不得反向持 session lock 调 authority，防止锁序死锁。

- [ ] **Step 6: 运行 GREEN 与 mailbox 回归**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_command_authority.py tests/test_wheel_mailbox.py`

Expected: PASS。

- [ ] **Step 7: REFACTOR authority 转换与拒绝路径**

只合并状态快照构造和“撤权后清 owner/sequence”的重复代码；验证顺序、commit 后认领、精确 peer count 与 generation 推进规则不变。原样重跑 Step 6 的 GREEN 命令，Expected: PASS。

### Task 7：把 transport 的布尔发现升级为精确 peer count 与协议状态

**Files:**
- Modify: `slope_sim/interfaces/transport.py`
- Modify: `slope_sim/interfaces/ecal_transport.py`
- Create: `tests/stage4/test_transport_v2_metadata.py`
- Modify: `tests/test_ecal_transport.py`
- Test: `tests/test_local_transport.py`

- [ ] **Step 1: 写“2 不能压成 True”和 metadata conflict RED**

```python
from dataclasses import fields

from slope_sim.interfaces.transport import TransportTopicQuality


def test_topic_quality_preserves_exact_peer_count() -> None:
    field_names = {field.name for field in fields(TransportTopicQuality)}
    assert {
        "peer_count",
        "protocol_state",
        "protocol_detail",
        "remote_type_names",
        "remote_encodings",
        "remote_descriptor_sha256",
    } <= field_names, "exact peer/protocol quality behavior is not implemented"
    quality = TransportTopicQuality(
        topic="/sim/wheel/command",
        peer_connected=True,
        peer_count=2,
        protocol_state="conflict",
        protocol_detail="unexpected slope_sim.interfaces.v1.WheelCommand",
        remote_type_names=("slope_sim.interfaces.v1.WheelCommand", "slope_sim.interfaces.v2.WheelCommand"),
        remote_encodings=("proto", "proto"),
        remote_descriptor_sha256=("11" * 32, "22" * 32),
    )
    assert quality.peer_count == 2
    assert quality.protocol_state == "conflict"


def test_ecal_binding_returns_count_instead_of_bool(fake_raw_subscriber, bindings) -> None:
    assert callable(getattr(bindings, "peer_count", None)), (
        "exact eCAL peer_count behavior is not implemented"
    )
    fake_raw_subscriber.publisher_count = 3
    assert bindings.peer_count(fake_raw_subscriber) == 3
    assert bindings.is_peer_connected(fake_raw_subscriber) is True
```

再覆盖负数、bool、peer_connected 与 count 矛盾、verified 但 count/metadata 数量不符、digest 非 64 位小写 hex、LocalTransport 使用 `peer_count=None/protocol_state="not_checked"`。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_transport_v2_metadata.py tests/test_ecal_transport.py`

Expected: pytest 正常收集后 `FAILED`；断言明确显示 `TransportTopicQuality` 尚无 `peer_count`/protocol metadata，或 binding 仍返回 bool。不得是 collection error、fixture error 或 skip。

- [ ] **Step 3: 扩展不可变质量快照**

```python
_PROTOCOL_STATES = frozenset(
    {"not_checked", "waiting", "pending", "verified", "conflict"}
)
_SHA256_HEX = re.compile(r"[0-9a-f]{64}\Z")


@dataclass(frozen=True)
class TransportTopicQuality:
    # 既有字段原样保留。
    peer_connected: bool | None = None
    peer_count: int | None = None
    protocol_state: str = "not_checked"
    protocol_detail: str = ""
    remote_type_names: tuple[str, ...] = ()
    remote_encodings: tuple[str, ...] = ()
    remote_descriptor_sha256: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        if self.peer_connected is not None and type(self.peer_connected) is not bool:
            raise ValueError("peer_connected must be a bool or None")
        if self.peer_count is not None:
            if isinstance(self.peer_count, bool) or not isinstance(self.peer_count, int) or self.peer_count < 0:
                raise ValueError("peer_count must be a nonnegative integer or None")
            if self.peer_connected is not (self.peer_count > 0):
                raise ValueError("peer_connected must agree with peer_count")
        if self.protocol_state not in _PROTOCOL_STATES:
            raise ValueError("invalid protocol_state")
        if self.protocol_state == "conflict" and not self.protocol_detail:
            raise ValueError("protocol conflict requires detail")
        if not (
            len(self.remote_type_names)
            == len(self.remote_encodings)
            == len(self.remote_descriptor_sha256)
        ):
            raise ValueError("remote metadata columns must have equal length")
        if any(not isinstance(name, str) or not name for name in self.remote_type_names):
            raise ValueError("remote type names must be nonempty strings")
        if any(not isinstance(value, str) or not value for value in self.remote_encodings):
            raise ValueError("remote encodings must be nonempty strings")
        if any(
            not isinstance(digest, str) or _SHA256_HEX.fullmatch(digest) is None
            for digest in self.remote_descriptor_sha256
        ):
            raise ValueError("remote descriptor digests must be lowercase SHA-256 hex")
        if self.protocol_state == "waiting":
            if self.peer_count != 0 or self.remote_type_names:
                raise ValueError("waiting requires zero peers and no remote metadata")
        if self.protocol_state == "pending" and (self.peer_count is None or self.peer_count == 0):
            raise ValueError("pending requires a positive exact peer count")
        if self.protocol_state in {"verified", "conflict"}:
            if self.peer_count is None or self.peer_count == 0:
                raise ValueError("verified/conflict requires a positive peer count")
            if self.peer_count != len(self.remote_type_names):
                raise ValueError("verified/conflict metadata must match exact peer count")
```

文件顶部增加 `import re`。所有构造 `TransportTopicQuality` 的复制路径必须显式保留这些新字段；禁止在错误/drop/recover 更新时把 peer metadata 清回默认值。测试逐项证明 `remote_type_names/remote_encodings/remote_descriptor_sha256` 三列与 endpoint 一一对应，不能丢掉 encoding 后仍声称完整 metadata 已验证。协议 `waiting` 与命令权 `WAITING` 是两个不同层次：前者只表示远端协议尚无 peer，后者是 WheelState 的命令权状态。

- [ ] **Step 4: 保留原始 discovery count**

```python
def _resource_peer_count(resource: _ProtoResource) -> int:
    """读取官方 discovery count，保留 0/1/>1 的精确值。"""
    raw = _resource_raw(resource)
    method_name = (
        "get_publisher_count"
        if resource.direction == "subscriber"
        else "get_subscriber_count"
    )
    count_method = getattr(raw, method_name, None)
    if not callable(count_method):
        raise RuntimeError(f"eCAL resource.{method_name} is unavailable")
    count = count_method()
    if isinstance(count, bool) or not isinstance(count, int) or count < 0:
        raise RuntimeError("eCAL peer count must be a nonnegative integer")
    return count


@staticmethod
def peer_count(resource: _ProtoResource) -> int:
    return _resource_peer_count(resource)


@staticmethod
def is_peer_connected(resource: _ProtoResource) -> bool:
    return _resource_peer_count(resource) > 0
```

`poll_peer_state()` 的 `observed` 改为 `dict[str, int]`，逐 topic 写入 `peer_count` 并由 `count>0` 派生旧 `peer_connected`；全局 eCAL connected 仍只用于旧 Dashboard，不再作为 v2 命令权依据。

- [ ] **Step 5: 运行 GREEN 与阶段三发现回归**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_transport_v2_metadata.py tests/test_ecal_transport.py tests/test_local_transport.py tests/test_interface_runtime.py`

Expected: PASS；旧调用者仍能读取 `peer_connected`，新调用者能区分 0、1、2 和更大 count。

- [ ] **Step 6: REFACTOR discovery 质量快照构造**

只共用 topic quality 的不可变构造和 exact-count 校验；不得把 count 再压成 bool，不得改变 v1 public snapshot 字段。原样重跑 Step 5 的 GREEN 命令，Expected: PASS。

### Task 8：建立 Python raw eCAL 与远端 metadata 验证边界

**Files:**
- Create: `slope_sim/interfaces/v2/ecal_raw.py`
- Create: `tests/stage4/test_ecal_v2_raw_unit.py`
- Test: `tests/test_ecal_installation.py`

- [ ] **Step 1: 写 callback 只复制、worker 先哈希后解析的 RED**

```python
from hashlib import sha256


def test_raw_callback_only_copies_owned_envelope(fake_core, descriptor, remote_metadata) -> None:
    EcalRawBindings = require_wished_module(
        "slope_sim.interfaces.v2.ecal_raw"
    ).EcalRawBindings
    received = []
    bindings = EcalRawBindings(fake_core, monotonic=lambda: 4.5)
    subscriber = bindings.create_subscriber(
        "/sim/wheel/state",
        "slope_sim.interfaces.v2.WheelState",
        descriptor,
        callback=received.append,
    )
    fake_core.subscribers[0].emit(
        b"\x08\x01\x98\x01payload",
        publisher_id=fake_topic_id(
            entity_id=41,
            process_id=7,
            host_name="remote-host",
        ),
        data_type_info=remote_metadata,
        send_timestamp=1234,
        send_clock=7,
    )
    assert received[0].payload == b"\x08\x01\x98\x01payload"
    assert received[0].remote_publisher_entity_id == 41
    assert received[0].remote_publisher_process_id == 7
    assert received[0].remote_publisher_host_name == "remote-host"
    assert received[0].remote_type_name == "slope_sim.interfaces.v2.WheelState"
    assert received[0].remote_descriptor == descriptor.serialized_file_descriptor_set
    assert received[0].send_timestamp_us == 1234
    assert received[0].send_clock == 7
    assert received[0].received_at == 4.5
    assert not hasattr(received[0], "payload_sha256")
    assert fake_core.subscribers[0].callback_argument_count == 3


def test_worker_hashes_before_remote_validation_and_parse(descriptor) -> None:
    module = require_wished_module("slope_sim.interfaces.v2.ecal_raw")
    process_raw_frame = module.process_raw_frame
    raw_frame = module.RawReceivedFrame(
        payload=b"payload",
        remote_publisher_entity_id=41,
        remote_publisher_process_id=7,
        remote_publisher_host_name="remote-host",
        remote_type_name="slope_sim.interfaces.v2.WheelState",
        remote_encoding="proto",
        remote_descriptor=descriptor.serialized_file_descriptor_set,
        send_timestamp_us=1,
        send_clock=1,
        received_at=1.0,
    )
    order = []

    def hash_payload(payload: bytes) -> bytes:
        order.append("hash")
        return sha256(payload).digest()

    def parse_payload(payload: bytes) -> object:
        order.append("parse")
        return object()

    processed = process_raw_frame(
        raw_frame,
        expected_type="slope_sim.interfaces.v2.WheelState",
        descriptor=descriptor,
        parser=parse_payload,
        payload_hasher=hash_payload,
    )
    assert order == ["hash", "parse"]
    assert processed.payload_sha256 == sha256(raw_frame.payload).digest()


def test_wrong_same_topic_endpoint_is_protocol_conflict(fake_core, descriptor) -> None:
    module = require_wished_module("slope_sim.interfaces.v2.ecal_raw")
    EcalRawBindings = module.EcalRawBindings
    ProtocolVerificationState = module.ProtocolVerificationState
    fake_core.monitoring.publishers = [
        fake_topic(
            "/sim/wheel/command",
            "slope_sim.interfaces.v1.WheelCommand",
            b"v1-descriptor",
        )
    ]
    snapshot = EcalRawBindings(fake_core).snapshot_remote_endpoints(
        topic="/sim/wheel/command",
        remote_direction="publisher",
        peer_count=1,
        expected_type="slope_sim.interfaces.v2.WheelCommand",
        descriptor=descriptor,
    )
    assert snapshot.verification.state is ProtocolVerificationState.CONFLICT
    assert "slope_sim.interfaces.v1.WheelCommand" in snapshot.verification.detail
```

fake subscriber 的 `emit()` 必须按 eCAL 6.1.1 nanobind raw API 的真实顺序调用 callback：`(publisher_id, data_type_info, data)`；若 callback 不是三参数，测试必须直接失败。再覆盖：peer_count 0=waiting、count 大于 monitoring 条目数=pending、完全匹配=verified、encoding 错、descriptor bytes 错、同 topic 一对一错 type、同 topic 多 peer 混合 v1/v2、其他 topic 不污染、callback buffer 和 `data_type_info` 在 native 返回后仍由 Python owned bytes/字符串独立持有。

安装探针还要锁定 id 层级：`type(publisher_id) is TopicId`、`type(publisher_id.topic_id) is EntityId`、`type(publisher_id.topic_id.entity_id) is int`，而 monitoring `Topic.topic_id` 直接是 `int`。fake 的 `publisher_id` 必须复刻这三层结构，禁止把 `publisher_id.topic_id` 伪造成整数而掩盖真实 API。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_ecal_v2_raw_unit.py`

Expected: pytest 正常收集后 `FAILED`，失败消息精确指出 raw callback/metadata 行为尚未实现；不得是 collection error、fixture error 或 skip。

- [ ] **Step 3: 封装 eCAL 6.1.1 raw API**

```python
# slope_sim/interfaces/v2/ecal_raw.py
"""阶段四 raw eCAL 边界：原字节收发、SHA-256 与远端类型元数据验证。"""
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
from hashlib import sha256
from importlib import import_module
import time

from slope_sim.interfaces.v2.descriptor import DescriptorIdentity


@dataclass(frozen=True)
class RawReceivedFrame:
    payload: bytes
    remote_publisher_entity_id: int
    remote_publisher_process_id: int
    remote_publisher_host_name: str
    remote_type_name: str
    remote_encoding: str
    remote_descriptor: bytes
    send_timestamp_us: int
    send_clock: int
    received_at: float


@dataclass(frozen=True)
class RemoteTypeMetadata:
    name: str
    encoding: str
    descriptor: bytes

    def __post_init__(self) -> None:
        if not isinstance(self.name, str):
            raise ValueError("remote type name must be a string")
        if not isinstance(self.encoding, str):
            raise ValueError("remote encoding must be a string")
        if not isinstance(self.descriptor, (bytes, bytearray, memoryview)):
            raise ValueError("remote descriptor must be bytes-like")
        object.__setattr__(self, "descriptor", bytes(self.descriptor))


@dataclass(frozen=True)
class ProcessedRawFrame:
    envelope: RawReceivedFrame
    payload_sha256: bytes
    parsed: object


class ProtocolVerificationState(Enum):
    WAITING = "waiting"
    PENDING = "pending"
    VERIFIED = "verified"
    CONFLICT = "conflict"


@dataclass(frozen=True)
class ProtocolVerification:
    state: ProtocolVerificationState
    peer_count: int
    type_names: tuple[str, ...]
    encodings: tuple[str, ...]
    descriptor_sha256: tuple[str, ...]
    detail: str = ""


@dataclass(frozen=True)
class RemoteEndpointSnapshot:
    verification: ProtocolVerification


class EcalRawBindings:
    def __init__(
        self,
        core: object | None = None,
        *,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self._core = import_module("ecal.nanobind_core") if core is None else core
        self._monotonic = monotonic

    def _type_info(self, type_name: str, descriptor: DescriptorIdentity) -> object:
        return self._core.DataTypeInformation(
            name=type_name,
            encoding="proto",
            descriptor=descriptor.serialized_file_descriptor_set,
        )

    def create_publisher(self, topic: str, type_name: str, descriptor: DescriptorIdentity) -> object:
        return self._core.Publisher(topic, self._type_info(type_name, descriptor))

    def create_subscriber(
        self,
        topic: str,
        type_name: str,
        descriptor: DescriptorIdentity,
        callback: Callable[[RawReceivedFrame], None],
    ) -> object:
        subscriber = self._core.Subscriber(topic, self._type_info(type_name, descriptor))

        def receive(
            publisher_id: object,
            data_type_info: object,
            data: object,
        ) -> None:
            # eCAL 6.1.1 raw callback 直接给出该帧远端类型；这里只复制 owned envelope。
            entity_id = publisher_id.topic_id
            metadata = RemoteTypeMetadata(
                str(data_type_info.name),
                str(data_type_info.encoding),
                bytes(data_type_info.descriptor),
            )
            payload = bytes(data.buffer)
            frame = RawReceivedFrame(
                payload=payload,
                remote_publisher_entity_id=int(entity_id.entity_id),
                remote_publisher_process_id=int(entity_id.process_id),
                remote_publisher_host_name=str(entity_id.host_name),
                remote_type_name=metadata.name,
                remote_encoding=metadata.encoding,
                remote_descriptor=bytes(metadata.descriptor),
                send_timestamp_us=int(data.send_timestamp),
                send_clock=int(data.send_clock),
                received_at=float(self._monotonic()),
            )
            callback(frame)

        subscriber.set_receive_callback(receive)
        return subscriber


def _sha256_digest(payload: bytes) -> bytes:
    return sha256(payload).digest()


def process_raw_frame(
    frame: RawReceivedFrame,
    *,
    expected_type: str,
    descriptor: DescriptorIdentity,
    parser: Callable[[bytes], object],
    payload_hasher: Callable[[bytes], bytes] = _sha256_digest,
) -> ProcessedRawFrame:
    """worker 中先哈希、再验远端 metadata，最后解析并验带内身份。"""
    payload_sha256 = bytes(payload_hasher(frame.payload))
    if len(payload_sha256) != 32:
        raise RuntimeError("payload hasher must return exactly 32 bytes")
    if (
        frame.remote_type_name != expected_type
        or frame.remote_encoding != "proto"
        or frame.remote_descriptor != descriptor.serialized_file_descriptor_set
    ):
        raise ValueError("remote type/encoding/descriptor mismatch")
    parsed = parser(frame.payload)
    return ProcessedRawFrame(frame, payload_sha256, parsed)
```

发送只调用 `publisher.send(frame.payload)`，禁止 ParseFromString 后交 typed publisher。eCAL 6.1.1 nanobind raw callback 的固定签名是 `(publisher_id, data_type_info, data)`；callback 参数只能是有界 receive lane 的 `enqueue`，不能是 parser 或业务回调。native 栈直接从本帧 `data_type_info` 复制远端 name/encoding/descriptor，并复制 payload、send timestamp/clock 和 received_at；严禁在 callback 内调用 monitoring、SHA-256、校验或 Protobuf parser。receive worker 取得 envelope 后严格按“payload SHA-256 -> 本帧远端 type/encoding/descriptor -> 当前 topic 的原子 protocol gate -> Protobuf parse -> codec 带内 descriptor/session/业务模型校验”处理。`create_subscriber` 保存 native callback 和 lane callback 的强引用，直到 `remove_receive_callback()` 完成，关闭顺序沿用阶段三 callback gate。

- [ ] **Step 4: 用 monitoring 验证远端而非本地声明**

```python
def _remote_topics(self, remote_direction: str) -> tuple[object, ...]:
    snapshot = self._core.monitoring.get_monitoring()
    if remote_direction == "publisher":
        return tuple(snapshot.publishers)
    if remote_direction == "subscriber":
        return tuple(snapshot.subscribers)
    raise ValueError("remote_direction must be publisher or subscriber")


def snapshot_remote_endpoints(
    self,
    *,
    topic: str,
    remote_direction: str,
    peer_count: int,
    expected_type: str,
    descriptor: DescriptorIdentity,
) -> RemoteEndpointSnapshot:
    endpoints = tuple(
        endpoint
        for endpoint in self._remote_topics(remote_direction)
        if endpoint.topic_name == topic
    )
    if peer_count == 0:
        verification = ProtocolVerification(
            ProtocolVerificationState.WAITING, 0, (), (), ()
        )
        return RemoteEndpointSnapshot(verification)
    names = tuple(item.datatype_information.name for item in endpoints)
    encodings = tuple(item.datatype_information.encoding for item in endpoints)
    descriptors = tuple(bytes(item.datatype_information.descriptor) for item in endpoints)
    descriptor_digests = tuple(sha256(raw).hexdigest() for raw in descriptors)
    if len(endpoints) != peer_count:
        verification = ProtocolVerification(
            ProtocolVerificationState.PENDING,
            peer_count,
            names,
            encodings,
            descriptor_digests,
            "discovery count and monitoring metadata are not yet aligned",
        )
        return RemoteEndpointSnapshot(verification)
    valid = all(
        name == expected_type and encoding == "proto" and raw == descriptor.serialized_file_descriptor_set
        for name, encoding, raw in zip(names, encodings, descriptors, strict=True)
    )
    verification = ProtocolVerification(
        ProtocolVerificationState.VERIFIED if valid else ProtocolVerificationState.CONFLICT,
        peer_count,
        names,
        encodings,
        descriptor_digests,
        "" if valid else "same-topic remote type/encoding/descriptor mismatch",
    )
    return RemoteEndpointSnapshot(verification)
```

monitoring `Topic.topic_id` 在 eCAL 6.1.1 Python binding 中是 `int`；raw callback 第一参是 `TopicId`，对应的整数位于 `publisher_id.topic_id.entity_id`，不是 `publisher_id.topic_id`。本实现不依赖 id 查表取得类型，因为 callback 第二参已经给出该帧真实 `data_type_info`；若诊断日志关联 monitoring 与 callback，只允许比较 `endpoint.topic_id == publisher_id.topic_id.entity_id`，并同时记录 `EntityId.host_name/process_id`，禁止比较错层字段。monitoring 只负责在 discovery gate 中按 topic 名和 exact peer count 原子判定 `WAITING/PENDING/VERIFIED/CONFLICT`；`PENDING` 不允许 payload 进入 parser，`CONFLICT` 在正式模式立即失败并保持详情。fake 与真实 Phase-0 都必须证明三参数 callback、本帧 metadata 与 monitoring gate 同时成立，不能回退成本地 type 声明。

- [ ] **Step 5: 运行 GREEN 与安装探针**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_ecal_v2_raw_unit.py tests/test_ecal_installation.py`

Expected: PASS；测试只用 fake core，不初始化真实 participant。

- [ ] **Step 6: REFACTOR raw envelope 与 monitoring 判定**

只合并 owned metadata 复制、descriptor digest 和 verification snapshot 构造；不得在 callback 中加入 hash/parse/monitoring，不得改变三参数签名或 id 字段层级。原样重跑 Step 5 的 GREEN 命令，Expected: PASS。

### Task 9：构建最小 C++17 raw eCAL/Protobuf 探针

**Files:**
- Create: `cpp/phase0/CMakeLists.txt`
- Create: `cpp/phase0/ecal_v2_raw_probe.cpp`
- Create: `cpp/phase0/v2_golden.cpp`
- Create: `cpp/phase0/sha256.hpp`
- Create: `tests/stage4/test_cpp_phase0_build.py`

- [ ] **Step 1: 先注册可配置 CTest target，再写 ABI 与完整 probe CLI RED**

```python
# tests/stage4/test_cpp_phase0_build.py
from hashlib import sha256
import json
import os
from pathlib import Path
import subprocess

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb


def _phase0_executable(name: str) -> Path:
    raw_build = os.environ.get("STAGE4_PHASE0_BUILD_DIR")
    assert raw_build, "CTest must provide STAGE4_PHASE0_BUILD_DIR"
    stage4_phase0_build = Path(raw_build)
    assert stage4_phase0_build.is_absolute(), "Phase-0 build directory must be absolute"
    assert stage4_phase0_build.is_dir(), "configured Phase-0 build directory is missing"
    executable = stage4_phase0_build / name
    assert executable.is_file(), f"Phase-0 {name} behavior is not implemented"
    assert os.access(executable, os.X_OK), f"Phase-0 {name} is not executable"
    return executable


def _valid_probe_inputs(tmp_path: Path) -> tuple[Path, Path]:
    frozen = (
        Path(__file__).resolve().parents[2]
        / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    )
    assert frozen.is_file(), "frozen v2 descriptor is missing"
    descriptor = tmp_path / "v2.desc"
    descriptor.write_bytes(frozen.read_bytes())
    payload = tmp_path / "command.bin"
    message = pb.WheelCommand(
        timestamp_ns=1,
        drive_wheel_speed_rad_s=(1.0, 1.0),
        sequence=0,
        world_generation=1,
        command_generation=1,
        source_id="phase0",
        source_session_id=b"p" * 16,
        robot_model="df_back",
        simulation_session_id=b"s" * 16,
        descriptor_sha256=sha256(descriptor.read_bytes()).digest(),
    )
    payload.write_bytes(message.SerializeToString(deterministic=True))
    return descriptor, payload


def test_phase0_tools_report_frozen_abi() -> None:
    executable = _phase0_executable("v2_golden")
    result = subprocess.run(
        [str(executable), "--version"],
        check=True,
        capture_output=True,
        text=True,
    )
    assert result.stdout.splitlines() == [
        "cxx=17",
        "compiler=gcc-13",
        "ecal=6.1.1",
        "protobuf=33.6",
        "glibcxx_cxx11_abi=1",
    ]


def test_raw_probe_accepts_complete_publish_and_subscribe_dry_runs(tmp_path) -> None:
    probe = _phase0_executable("ecal_v2_raw_probe")
    descriptor, payload = _valid_probe_inputs(tmp_path)
    publish_result = tmp_path / "publish.json"
    subscribe_payload = tmp_path / "received.bin"
    subscribe_result = tmp_path / "subscribe.json"

    cases = (
        (
            [
                str(probe), "--dry-run", "publish",
                "--topic", "/sim/wheel/command",
                "--type-name", "slope_sim.interfaces.v2.WheelCommand",
                "--descriptor-set", str(descriptor),
                "--payload", str(payload),
                "--result", str(publish_result),
                "--deadline-ms", "10000",
            ],
            {
                "deadline_ms": 10000,
                "descriptor_set": str(descriptor),
                "dry_run": True,
                "mode": "publish",
                "payload": str(payload),
                "result": str(publish_result),
                "topic": "/sim/wheel/command",
                "type_name": "slope_sim.interfaces.v2.WheelCommand",
            },
        ),
        (
            [
                str(probe), "--dry-run", "subscribe",
                "--topic", "/sim/wheel/command",
                "--type-name", "slope_sim.interfaces.v2.WheelCommand",
                "--descriptor-set", str(descriptor),
                "--payload-out", str(subscribe_payload),
                "--result", str(subscribe_result),
                "--expected-peer-count", "1",
                "--deadline-ms", "10000",
            ],
            {
                "deadline_ms": 10000,
                "descriptor_set": str(descriptor),
                "dry_run": True,
                "expected_peer_count": 1,
                "mode": "subscribe",
                "payload_out": str(subscribe_payload),
                "result": str(subscribe_result),
                "topic": "/sim/wheel/command",
                "type_name": "slope_sim.interfaces.v2.WheelCommand",
            },
        ),
    )
    for argv, expected in cases:
        completed = subprocess.run(argv, check=False, capture_output=True, text=True)
        assert completed.returncode == 0, completed.stderr
        assert completed.stderr == ""
        assert completed.stdout == json.dumps(
            expected, sort_keys=True, separators=(",", ":"), ensure_ascii=True
        ) + "\n"
    assert not publish_result.exists()
    assert not subscribe_payload.exists()
    assert not subscribe_result.exists()


def test_raw_probe_rejects_bad_cli_matrix(tmp_path) -> None:
    probe = _phase0_executable("ecal_v2_raw_probe")
    descriptor, payload = _valid_probe_inputs(tmp_path)
    result = tmp_path / "result.json"
    common = [
        "--topic", "/sim/wheel/command",
        "--type-name", "slope_sim.interfaces.v2.WheelCommand",
        "--descriptor-set", str(descriptor),
    ]
    cases = (
        ("missing", ["--dry-run", "publish", *common[2:], "--payload", str(payload), "--result", str(result), "--deadline-ms", "10000"], 64),
        ("duplicate", ["--dry-run", "publish", *common, "--topic", "/duplicate", "--payload", str(payload), "--result", str(result), "--deadline-ms", "10000"], 64),
        ("unknown", ["--dry-run", "publish", *common, "--payload", str(payload), "--result", str(result), "--deadline-ms", "10000", "--unknown"], 64),
        ("relative", ["--dry-run", "publish", *common[:4], "--descriptor-set", "relative.desc", "--payload", str(payload), "--result", str(result), "--deadline-ms", "10000"], 64),
        ("deadline", ["--dry-run", "publish", *common, "--payload", str(payload), "--result", str(result), "--deadline-ms", "0"], 64),
        ("wrong-role", ["--dry-run", "publish", *common, "--payload", str(payload), "--payload-out", str(tmp_path / "out.bin"), "--result", str(result), "--deadline-ms", "10000"], 64),
        ("output-alias", ["--dry-run", "subscribe", *common, "--payload-out", str(result), "--result", str(result), "--expected-peer-count", "1", "--deadline-ms", "10000"], 73),
        ("existing-output", ["--dry-run", "publish", *common, "--payload", str(payload), "--result", str(descriptor), "--deadline-ms", "10000"], 73),
    )
    for label, args, expected_rc in cases:
        completed = subprocess.run(
            [str(probe), *args], check=False, capture_output=True, text=True
        )
        assert completed.returncode == expected_rc, label
        assert completed.stdout == "", label
        assert completed.stderr.startswith("error: "), label
```

`ecal_v2_raw_probe` 的公开 CLI 在任何真实 Phase-0 前冻结为：

```text
ecal_v2_raw_probe --version
ecal_v2_raw_probe [--dry-run] publish \
  --topic <name> --type-name <name> --descriptor-set <absolute.desc> \
  --payload <absolute.bin> --result <absolute.json> --deadline-ms 10000
ecal_v2_raw_probe [--dry-run] subscribe \
  --topic <name> --type-name <name> --descriptor-set <absolute.desc> \
  --payload-out <absolute.bin> --result <absolute.json> \
  --expected-peer-count 1 --deadline-ms 10000
```

除可选且只能位于子命令前的 `--dry-run` 外，上述参数全部显式必填且每项只能出现一次；`deadline-ms` 接受 `1..60000`，Phase-0 subscriber 的 `expected-peer-count` 只接受精确值 1。topic/type-name 必须非空；所有路径必须为已词法规范化的绝对路径，descriptor/payload 必须是已存在的普通文件，output 的父目录必须存在，所有 output 必须彼此不同、不得别名到 input、且执行前均不存在。未知参数、缺参数、重复参数、相对路径、越界数值和角色不匹配参数返回 64；输入缺失/不可读返回 66；输出已存在或路径别名返回 73；运行时/协议失败返回 1；成功返回 0。

`--dry-run` 完成与真实模式完全相同的 parse、输入读取条件和输出排他校验：解析完整 `FileDescriptorSet`、计算 descriptor SHA-256、确认 type-name 存在；publish 还要解析 payload 并验证带内 descriptor/type 业务边界。随后向 stdout 输出 UTF-8、ASCII escaping、`sort_keys`、紧凑分隔符和单个 LF 的上述 canonical plan JSON；不创建 `payload-out/result`，并且必须在任何 `eCAL::Initialize`、publisher/subscriber 构造或 monitoring 调用之前返回。`--version` 只能单独出现。坏参数矩阵刻意放在一个表驱动测试函数内，使 production binary 尚不存在时 CTest 的 RED 恰好是 ABI、合法 dry-run、非法参数三项失败，而不是按 case 数膨胀。

在实现 production target 前，先提交同一 `cpp/phase0/CMakeLists.txt` 的可配置 RED prelude；它能在不查找 eCAL/Protobuf、不编译 production 代码时完成 configure，并把上述 pytest 注册成真实 CTest target：

```cmake
# cpp/phase0/CMakeLists.txt 的 RED prelude；Step 3 在 return() 后扩展生产工程。
cmake_minimum_required(VERSION 3.28)
project(slope_sim_stage4_phase0 VERSION 0.1.0 LANGUAGES NONE)

include(CTest)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
if(NOT Python3_VERSION_MAJOR EQUAL 3 OR NOT Python3_VERSION_MINOR EQUAL 10)
  message(FATAL_ERROR "Phase-0 CTest requires the slope-sim Python 3.10 interpreter")
endif()
if(NOT BUILD_TESTING)
  message(FATAL_ERROR "Phase-0 contract target requires BUILD_TESTING=ON")
endif()
option(STAGE4_PHASE0_RED_ONLY "Configure only the pre-implementation RED target" OFF)
get_filename_component(STAGE4_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

add_test(
  NAME phase0_tools_report_frozen_abi
  COMMAND "${Python3_EXECUTABLE}" -m pytest -q
          "${STAGE4_ROOT}/tests/stage4/test_cpp_phase0_build.py"
)
set_tests_properties(
  phase0_tools_report_frozen_abi
  PROPERTIES
    ENVIRONMENT "STAGE4_PHASE0_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}"
    WORKING_DIRECTORY "${STAGE4_ROOT}"
)

if(STAGE4_PHASE0_RED_ONLY)
  return()
endif()
```

测试不得使用 skip fixture。RED 与 GREEN 都由这一个 CTest target 注入绝对 build 目录；测试函数自己断言目录和 executable，并在 executable 尚不存在时给出明确 `FAILED`。

- [ ] **Step 2: 运行 RED**

Run: `test -x "$STAGE4_CMAKE" && test -x "$STAGE4_CTEST" && test "$("$STAGE4_CTEST" --version | sed -n '1s/^ctest version //p' | cut -d. -f1-2)" = "3.28"`

Expected: rc=0；只能使用总路线依赖门给出的 CMake/CTest 3.28.x 绝对路径。

Run: `test ! -e build/stage4-phase0/v2_golden && test ! -e build/stage4-phase0/ecal_v2_raw_probe && conda run -n slope-sim "$STAGE4_CMAKE" -S cpp/phase0 -B build/stage4-phase0 -G Ninja -DSTAGE4_PHASE0_RED_ONLY=ON`

Expected: configure PASS；此时不查找 eCAL/Protobuf，也没有 production executable。

Run: `"$STAGE4_CTEST" --test-dir build/stage4-phase0 -N -R '^phase0_tools_report_frozen_abi$' --no-tests=error`

Expected: CTest 精确列出 `phase0_tools_report_frozen_abi`，且不存在零测试假通过。

Run: `"$STAGE4_CTEST" --test-dir build/stage4-phase0 --output-on-failure -R '^phase0_tools_report_frozen_abi$' --no-tests=error`

Expected: CTest 正常发现并运行 1 个 target 后非零退出，内部 pytest 精确报告 `3 failed`：ABI 缺 `v2_golden`、合法 dry-run 缺 `ecal_v2_raw_probe`、坏参数矩阵缺 `ecal_v2_raw_probe`。不得是 configure/build error、collection error、fixture error 或 skip。

- [ ] **Step 3: 在已注册 RED target 后扩展独立 Phase-0 CMake 工程**

```cmake
# cpp/phase0/CMakeLists.txt
# 保留 Step 1 的 cmake_minimum/project/CTest/RED_ONLY prelude。
enable_language(CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=1)

find_package(eCAL 6.1.1 EXACT REQUIRED CONFIG)
find_package(Protobuf 33.6.0 EXACT REQUIRED CONFIG)
find_package(OpenSSL 3 REQUIRED COMPONENTS Crypto)

set(V2_PROTO_DIR "${CMAKE_CURRENT_LIST_DIR}/../../proto")
set(V2_PROTO "${V2_PROTO_DIR}/slope_sim_interfaces_v2.proto")
set(V2_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

add_library(stage4_v2_proto STATIC "${V2_PROTO}")
target_link_libraries(stage4_v2_proto PUBLIC protobuf::libprotobuf)
target_include_directories(stage4_v2_proto PUBLIC "${V2_GENERATED_DIR}")
protobuf_generate(
  TARGET stage4_v2_proto
  LANGUAGE cpp
  IMPORT_DIRS "${V2_PROTO_DIR}"
  PROTOC_OUT_DIR "${V2_GENERATED_DIR}"
)

add_executable(ecal_v2_raw_probe ecal_v2_raw_probe.cpp sha256.hpp)
target_link_libraries(ecal_v2_raw_probe PRIVATE eCAL::core stage4_v2_proto OpenSSL::Crypto)

add_executable(v2_golden v2_golden.cpp sha256.hpp)
target_link_libraries(v2_golden PRIVATE stage4_v2_proto OpenSSL::Crypto)
```

Configure 时显式检查 `CMAKE_CXX_COMPILER_ID STREQUAL "GNU"`、major=13、`eCAL_VERSION VERSION_EQUAL 6.1.1` 和 `Protobuf_VERSION VERSION_EQUAL 33.6.0`；再读取 imported target `protobuf::protoc` 的配置相关绝对位置，要求其 `REAL_PATH` 与必填 cache 参数 `STAGE4_PROTOC_EXECUTABLE` 相同，执行 `--version` 并要求 `libprotoc 33.6`。任一不符 `message(FATAL_ERROR ...)`。使用 config package 提供的 `protobuf_generate()`，不依赖 module compatibility 才存在的 `protobuf_generate_cpp()`。Python runtime 的发行版本另行固定为 6.33.6，不能把 Python 包版本误填成 C++ `libprotoc/libprotobuf` 版本；不得用 `FetchContent` 临时下载另一套库。

- [ ] **Step 4: 先实现 probe parser 与无 eCAL dry-run**

`main()` 首先调用无 eCAL 依赖的 `ParseProbeCli(argc, argv)` 得到不可变 `ProbePlan`；解析器按 Step 1 固定顺序完成重复/未知/角色参数校验、整数边界、绝对规范路径、input 普通文件和 output 排他检查。只有 `--version` 或合法 `--dry-run` 在输出固定内容后直接返回；它们的控制流必须在 `eCAL::Initialize` 之前汇合结束。真实 publish/subscribe 只能消费同一个已经校验过的 `ProbePlan`，不得再维护第二套默认值或宽松 parser。

canonical plan JSON 的字段集合精确等于 Step 1 两个 expected dict；字段按 ASCII key 升序、无多余空白、最后一个 LF。dry-run 不打开任何 output，也不创建 participant。参数错误写单行 `error: <stable reason>` 到 stderr 且 stdout 为空。`main()` 中 `if (plan.dry_run) return PrintCanonicalPlan(plan);` 必须在源码控制流上位于唯一的 `InitializeEcal(plan)` 调用之前，且禁止全局/static initializer 间接初始化 eCAL；Task 10 fake 编排再证明每条真实 argv 都经过这一分支。

- [ ] **Step 5: 实现 callback-owned envelope 与 worker hash-before-parse**

```cpp
// cpp/phase0/ecal_v2_raw_probe.cpp
// 阶段四 Phase-0：验证 eCAL 原始字节、远端类型元数据与 C++ Protobuf 解析。
#include <ecal/ecal.h>
#include <ecal/pubsub/publisher.h>
#include <ecal/pubsub/subscriber.h>

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "sha256.hpp"
#include "slope_sim_interfaces_v2.pb.h"

struct RawEnvelope final {
  std::vector<std::byte> payload;
  std::string remote_type_name;
  std::string remote_encoding;
  std::vector<std::byte> remote_descriptor;
  std::int64_t send_timestamp_us;
  std::int64_t send_clock;
  std::chrono::steady_clock::time_point received_at;
};

RawEnvelope CopyEnvelope(const eCAL::SDataTypeInformation& type_info,
                         const eCAL::SReceiveCallbackData& data,
                         std::chrono::steady_clock::time_point received_at) {
  const auto* first = static_cast<const std::byte*>(data.buffer);
  std::vector<std::byte> payload(first, first + data.buffer_size);
  return RawEnvelope{
      payload,
      type_info.name,
      type_info.encoding,
      std::vector<std::byte>(
          reinterpret_cast<const std::byte*>(type_info.descriptor.data()),
          reinterpret_cast<const std::byte*>(type_info.descriptor.data()) +
              type_info.descriptor.size()),
      data.send_timestamp,
      data.send_clock,
      received_at,
  };
}
```

`sha256.hpp` 使用 OpenSSL EVP `EVP_DigestInit_ex/Update/Final_ex`，每一步检查返回值且输出固定 `std::array<std::byte,32>`。callback 内只调用 `CopyEnvelope(..., steady_clock::now())` 并放入有界 receive lane；worker 取出后先 `const auto payload_sha256 = Sha256(envelope.payload)`，再校验远端 metadata，之后才 `ParseFromArray`，最后校验带内 descriptor/session 和业务模型。callback 返回后测试主动覆盖发送缓冲区，worker 持有的 payload/type metadata/timestamps 必须不变。

- [ ] **Step 6: 用完整 type metadata 构造 raw publisher/subscriber**

```cpp
eCAL::SDataTypeInformation TypeInfo(
    const std::string& type_name,
    const std::vector<std::byte>& descriptor) {
  eCAL::SDataTypeInformation info;
  info.name = type_name;
  info.encoding = "proto";
  info.descriptor.assign(
      reinterpret_cast<const char*>(descriptor.data()), descriptor.size());
  return info;
}

eCAL::CPublisher publisher(topic, TypeInfo(type_name, descriptor));
eCAL::CSubscriber subscriber(topic, TypeInfo(type_name, descriptor));
subscriber.SetReceiveCallback(
    [&](const eCAL::STopicId&,
        const eCAL::SDataTypeInformation& remote_type,
        const eCAL::SReceiveCallbackData& data) {
      receive_lane.Push(CopyEnvelope(
          remote_type, data, std::chrono::steady_clock::now()));
    });
```

发布模式从文件读取已经由 Python 确定性编码的 bytes，调用 `publisher.Send(payload.data(), payload.size())`；订阅 worker 先计算原始 payload SHA-256，再比较 callback 复制的远端 `name/encoding/descriptor`，完全一致后才 ParseFromArray。若 eCAL 6.1.1 实际安装头文件的命名空间/回调 typedef 与上述官方 raw API 不同，只允许按该版本头文件做机械签名修正，并把探针编译测试固定下来；不能退回 typed Protobuf publisher/subscriber。

- [ ] **Step 7: 实现无 eCAL 的 golden CLI**

`v2_golden` 固定三个命令：

```text
v2_golden --version
v2_golden decode --descriptor-set <absolute.desc> <message-name> <payload.bin>
v2_golden encode-fixtures --descriptor-set <absolute.desc> --output-dir <directory>
```

descriptor 参数必填；程序读取实际 bytes、解析完整 `FileDescriptorSet` 并自行计算 SHA-256，不能从工作目录猜文件，也不能接受调用方直接注入一个待比较 hash。`decode` 的 `message-name` 只接受 `WheelCommand/WheelState/LidarPointCloud/RtkState/ImuAttitude`，输出该消息全部字段以及 `payload_sha256/descriptor_sha256` 的规范 JSON；未知名称、descriptor 解析失败、payload 解析失败或带内 descriptor 不匹配均非零退出。JSON 只使用同一 `libprotobuf` 的 `google::protobuf::util::MessageToJsonString`，固定 `preserve_proto_field_names=true`，再由受测 helper 包一层只含固定 ASCII type/hash 字段的对象；不得为 Phase-0 临时引入另一套 JSON 依赖或手写通用转义器。`encode-fixtures` 通过 C++ `CodedOutputStream::SetSerializationDeterministic(true)` 生成五个 `.bin` 及 `manifest.json`：

```cpp
std::string SerializeDeterministic(const google::protobuf::MessageLite& message) {
  std::string bytes(message.ByteSizeLong(), '\0');
  google::protobuf::io::ArrayOutputStream array(bytes.data(), bytes.size());
  google::protobuf::io::CodedOutputStream coded(&array);
  coded.SetSerializationDeterministic(true);
  if (!message.SerializeToCodedStream(&coded) || coded.HadError()) {
    throw std::runtime_error("deterministic protobuf serialization failed");
  }
  bytes.resize(coded.ByteCount());
  return bytes;
}
```

测试值固定使用 session `00112233445566778899aabbccddeeff`、source session `ffeeddccbbaa99887766554433221100`、world=7、command=11 和每类非零 sequence，RTK 三个 presence 必须真实 set。

- [ ] **Step 8: 配置、构建并检查动态链接**

Run: `test -x "$STAGE4_CMAKE" && test -x "$STAGE4_CXX" && test "$("$STAGE4_CMAKE" --version | sed -n '1s/^cmake version //p' | cut -d. -f1-2)" = "3.28" && test "$("$STAGE4_CXX" -dumpfullversion -dumpversion | cut -d. -f1)" = "13"`

Expected: rc=0；总路线依赖门必须提供已核验工具的绝对路径，CMake 固定 3.28.x、GCC 固定 major 13；完整 patch/build 身份另写入依赖报告和发行 manifest。

Run: `test -n "$STAGE4_CMAKE_PREFIX_PATH" && "$STAGE4_CMAKE" -S cpp/phase0 -B build/stage4-phase0 -G Ninja -DSTAGE4_PHASE0_RED_ONLY=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_COMPILER="$STAGE4_CXX" -DCMAKE_PREFIX_PATH="$STAGE4_CMAKE_PREFIX_PATH" -DSTAGE4_PROTOC_EXECUTABLE="$STAGE4_PROTOC"`

Expected: configure PASS；`STAGE4_CMAKE_PREFIX_PATH` 只包含总路线依赖门核验过的 eCAL 6.1.1 与 Protobuf 33.6 release prefix。缺 eCAL C++ dev、错误 Protobuf/GCC/ABI 时明确 FAIL 并停止，不通过改 PATH 或链接 Conda 私有库绕过。

Run: `"$STAGE4_CMAKE" --build build/stage4-phase0 --parallel 2`

Expected: 两个 executable 构建成功。

Run: `readelf -d build/stage4-phase0/ecal_v2_raw_probe && ldd build/stage4-phase0/ecal_v2_raw_probe`

Expected: build tree 无 Conda 路径，`libecal_core`、`libprotobuf` 来自冻结 dependency prefix，`libcrypto.so.3` 来自 `ubuntu24-system-dependencies.lock` 明确允许的 Ubuntu 24.04 系统 ABI，且进程内只有一套 `libprotobuf`。build-only RPATH 可以指向该 dependency prefix；后续 C/E 安装树必须改为相对 RUNPATH 并随包带齐非系统闭包。

- [ ] **Step 9: 运行 C++ build contract GREEN**

Run: `"$STAGE4_CTEST" --test-dir build/stage4-phase0 --output-on-failure -R '^phase0_tools_report_frozen_abi$' --no-tests=error`

Expected: 与 Step 2 完全相同的 CTest target 运行 `1/1 PASS`，内部 ABI、完整 dry-run 和坏参数矩阵全部 GREEN 且无 skip；`v2_golden --version` 精确报告冻结 ABI，probe dry-run 未初始化 eCAL。

- [ ] **Step 10: REFACTOR C++ owned envelope 与确定性序列化公共代码**

只在 `sha256.hpp`/内部 helper 中消除 probe 与 golden 的重复校验和错误处理；不得改变 C ABI、CLI/exit code、依赖解析、callback/worker 顺序或 wire bytes。原样重跑 Step 9 的 CTest GREEN，并重跑 Step 8 的 `readelf/ldd` 检查，Expected 均与原步骤相同。

### Task 10：执行真实 eCAL Phase-0 并裁决同名 topic

**Files:**
- Create: `scripts/verify_stage4_v2_phase0.py`
- Create: `tests/stage4/test_ecal_v2_phase0.py`
- Modify: `docs/阶段四交付报告.md`

- [ ] **Step 1: 写编排和结果判定 RED（不启动 eCAL）**

```python
# tests/stage4/test_ecal_v2_phase0.py 中的纯函数测试
def test_phase0_rejects_payload_hash_mismatch() -> None:
    verify_phase0_result = require_wished_module(
        "scripts.verify_stage4_v2_phase0"
    ).verify_phase0_result
    result = valid_phase0_result()
    result["scenarios"]["python_to_cpp_raw"]["subscriber"][
        "payload_sha256"
    ] = "00" * 32
    with pytest.raises(AssertionError, match="payload SHA-256"):
        verify_phase0_result(result)


def test_phase0_requires_v1_same_topic_hard_failure() -> None:
    verify_phase0_result = require_wished_module(
        "scripts.verify_stage4_v2_phase0"
    ).verify_phase0_result
    result = valid_phase0_result()
    result["scenarios"]["v1_v2_same_topic_conflict"]["exit_code"] = 0
    with pytest.raises(AssertionError, match="v1 same-topic peer"):
        verify_phase0_result(result)


def test_phase0_rejects_remote_descriptor_metadata_mismatch() -> None:
    verify_phase0_result = require_wished_module(
        "scripts.verify_stage4_v2_phase0"
    ).verify_phase0_result
    result = valid_phase0_result()
    result["scenarios"]["cpp_to_python_raw"]["subscriber"][
        "remote_descriptor_sha256"
    ] = "11" * 32
    with pytest.raises(AssertionError, match="remote descriptor"):
        verify_phase0_result(result)


def test_cpp_probe_argv_is_exact_and_dry_run_precedes_real() -> None:
    module = require_wished_module("scripts.verify_stage4_v2_phase0")
    build_commands = module.build_cpp_probe_commands
    run_commands = module.run_cpp_probe_commands
    probe = "/opt/stage4/bin/ecal_v2_raw_probe"
    descriptor = "/tmp/stage4/v2.desc"
    type_name = "slope_sim.interfaces.v2.WheelCommand"
    publish = build_commands(
        probe=probe,
        role="publish",
        topic="/sim/wheel/command",
        type_name=type_name,
        descriptor_set=descriptor,
        payload="/tmp/stage4/send.bin",
        result="/tmp/stage4/publish.json",
        deadline_ms=10000,
    )
    subscribe = build_commands(
        probe=probe,
        role="subscribe",
        topic="/sim/wheel/command",
        type_name=type_name,
        descriptor_set=descriptor,
        payload_out="/tmp/stage4/receive.bin",
        result="/tmp/stage4/subscribe.json",
        expected_peer_count=1,
        deadline_ms=10000,
    )
    assert publish == (
        [probe, "--dry-run", "publish", "--topic", "/sim/wheel/command", "--type-name", type_name, "--descriptor-set", descriptor, "--payload", "/tmp/stage4/send.bin", "--result", "/tmp/stage4/publish.json", "--deadline-ms", "10000"],
        [probe, "publish", "--topic", "/sim/wheel/command", "--type-name", type_name, "--descriptor-set", descriptor, "--payload", "/tmp/stage4/send.bin", "--result", "/tmp/stage4/publish.json", "--deadline-ms", "10000"],
    )
    assert subscribe == (
        [probe, "--dry-run", "subscribe", "--topic", "/sim/wheel/command", "--type-name", type_name, "--descriptor-set", descriptor, "--payload-out", "/tmp/stage4/receive.bin", "--result", "/tmp/stage4/subscribe.json", "--expected-peer-count", "1", "--deadline-ms", "10000"],
        [probe, "subscribe", "--topic", "/sim/wheel/command", "--type-name", type_name, "--descriptor-set", descriptor, "--payload-out", "/tmp/stage4/receive.bin", "--result", "/tmp/stage4/subscribe.json", "--expected-peer-count", "1", "--deadline-ms", "10000"],
    )

    calls = []
    run_commands(publish, runner=lambda argv: calls.append(argv) or fake_success(argv))
    assert calls == list(publish)
    calls.clear()
    with pytest.raises(RuntimeError, match="probe dry-run failed"):
        run_commands(
            subscribe,
            runner=lambda argv: calls.append(argv) or fake_failure(argv),
        )
    assert calls == [subscribe[0]]
```

纯函数再拒绝：metadata name/encoding/descriptor 任一不等、peer count 不精确、callback bytes 与发送文件不同、C++ parse 在 metadata verified 前发生、错误进程退出码、缺 clean shutdown/finalized、把 `/sim/v2/...` 偷换成测试 topic。`fake_success/fake_failure` 返回最小 `subprocess.CompletedProcess` 测试替身；fake 测试还要逐角色检查 argv 是由 Task 9 冻结字段顺序生成的全新 list，真实调用只比 dry-run 少一个 `--dry-run`，且 dry-run 非零时绝不调第二条命令。

- [ ] **Step 2: 运行判定器 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_ecal_v2_phase0.py -m "not ecal"`

Expected: pytest 正常收集后 `FAILED`，失败消息精确指出 Phase-0 result verifier 尚未实现；不得是 collection error、fixture error、skip，也不得启动 participant。

- [ ] **Step 3: 实现四场景一次性编排**

```python
SCENARIOS = (
    "python_to_python_raw",
    "python_to_cpp_raw",
    "cpp_to_python_raw",
    "v1_v2_same_topic_conflict",
)
SUCCESS_SCENARIOS = SCENARIOS[:3]


def verify_phase0_result(result: dict[str, object]) -> None:
    assert result["topic"] == "/sim/wheel/command"
    assert result["expected_type"] == "slope_sim.interfaces.v2.WheelCommand"
    assert result["expected_encoding"] == "proto"
    assert result["expected_descriptor_sha256"] == frozen_descriptor_hex()
    scenarios = result["scenarios"]
    assert type(scenarios) is dict
    assert len(scenarios) == len(SCENARIOS)
    assert set(scenarios) == set(SCENARIOS)
    for name in SUCCESS_SCENARIOS:
        scenario = scenarios[name]
        publisher = scenario["publisher"]
        subscriber = scenario["subscriber"]
        assert publisher["payload_sha256"] == subscriber["payload_sha256"], (
            f"payload SHA-256 mismatch in {name}"
        )
        assert publisher["descriptor_sha256"] == subscriber["descriptor_sha256"]
        assert subscriber["remote_type_name"] == result["expected_type"]
        assert subscriber["remote_encoding"] == result["expected_encoding"]
        assert (
            subscriber["remote_descriptor_sha256"]
            == result["expected_descriptor_sha256"]
        ), f"remote descriptor SHA-256 mismatch in {name}"
        assert subscriber["peer_count"] == 1
        assert subscriber["protocol_state"] == "verified"
        assert subscriber["worker_order"] == [
            "payload_sha256",
            "remote_metadata_verified",
            "protobuf_parsed",
            "in_band_identity_validated",
        ]
        assert scenario["clean_shutdown"] is True
        assert scenario["finalized"] is True

    isolation = scenarios["v1_v2_same_topic_conflict"]
    assert isolation["protocol_state"] == "conflict"
    assert isolation["accepted_count"] == 0
    assert isolation["exit_code"] != 0, "v1 same-topic peer must hard fail"
    assert isolation["clean_shutdown"] is True
    assert isolation["finalized"] is True
```

编排固定先对每条 C++ probe argv 执行对应 dry-run，确认 canonical plan 与预期逐字段相等后才允许启动真实进程；随后先起 subscriber，轮询 exact count 和 monitoring metadata 到 verified，再发送一条由 Task 4 codec 从固定 WheelCommand fixture 确定性生成的 payload。每个子进程有 10 秒绝对 deadline，超时发送 SIGTERM、等待 2 秒后才 SIGKILL，并在结果记录。每个 scenario 写独立 JSON，汇总文件只按上面固定 schema 引用四项，不把字段复制成另一套命名；callback payload `.bin`、发送 `.bin` 与两端 JSON hash 必须四向相等。四个场景严格串行；临时结果目录用 `mkdtemp(prefix="stage4-a-phase0-")`，不得复用旧 JSON。Task 10 不依赖 Task 13 才创建的仓库 golden 文件。

- [ ] **Step 4: 运行纯单元 GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_ecal_v2_phase0.py -m "not ecal"`

Expected: 所有判定器/编排 fake 测试 PASS，无 eCAL participant。

- [ ] **Step 5: REFACTOR 结果判定与子进程收口**

只合并 scenario schema 校验、deadline/退出码收集和证据路径规范化；不得启动 eCAL、改变四场景、弱化任一断言或读取历史结果。原样重跑 Step 4 的纯单元 GREEN，Expected: PASS，且仍无 participant。

- [ ] **Step 6: 在用户再次明确授权后做唯一一次真实门禁**

先紧邻授权门重跑 Task 9 的同一 build contract，禁止用较早的 PASS 代替：

Run: `"$STAGE4_CTEST" --test-dir build/stage4-phase0 --output-on-failure -R '^phase0_tools_report_frozen_abi$' --no-tests=error`

Expected: `1/1 PASS`，内部 ABI、合法 dry-run、坏参数矩阵全部 GREEN；若失败则停止，不请求真实 eCAL 授权。

只有该命令刚刚 PASS，才向用户请求本次唯一真实门禁的明确授权并做只读扫描：

Run: `ps -eo pid,stat,pcpu,pmem,args | rg 'pytest|PyBullet|slope_sim|ecal|xvfb-run|Xvfb'`

Expected: 除明确允许的长期空闲 `Xvfb :1` 外无测试、PyBullet、eCAL participant、GUI 或临时 Xvfb；若有其他负载，等待静默窗口，不抢跑。

Run: `STAGE4_PHASE0_BUILD_DIR=$PWD/build/stage4-phase0 env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD conda run -n slope-sim python -m pytest -q -m ecal tests/stage4/test_ecal_v2_phase0.py`

Expected: `4 passed`，且只执行一次。结果目录包含四个独立 JSON、发送/接收原始 `.bin`、stdout/stderr、descriptor 和 SHA-256 清单。

- [ ] **Step 7: 应用不可绕过的裁决**

通过条件必须同时成立：

```text
Python raw -> Python raw: payload bytes/hash 相同，远端 name/encoding/descriptor verified
Python raw -> C++ raw:    callback 复制 bytes/metadata/time，worker hash/metadata verified 后才 parse
C++ raw -> Python raw:    callback 复制 bytes/metadata/time，worker hash/monitoring verified 后才 parse
v1 + v2 同一 topic:       v2 端 protocol_state=conflict，非零退出，payload accepted=0
```

若四项全通过，报告写“同名 topic Phase-0：PASS”并附本次唯一结果路径；若任一失败，写“BLOCKED：需用户裁决 `/sim/v2/...`”，停止后续 Task 11-14。失败本身不能改写成通过，也不能自动进行第二次真实 eCAL。

### Task 11：把通过证明的 raw 能力接入有界 Python transport

**Files:**
- Modify: `slope_sim/interfaces/ecal_transport.py`
- Create: `slope_sim/interfaces/v2/transport.py`
- Create: `tests/stage4/test_ecal_v2_transport.py`
- Modify: `tests/test_ecal_transport.py`
- Test: `tests/test_ecal_process_roundtrip.py`

- [ ] **Step 1: 写 raw send、metadata-before-delivery 和 owner+latest RED**

```python
def test_v2_transport_sends_exact_codec_bytes(
    request, descriptor, fake_v2_bindings, encoded_wheel_state
) -> None:
    module = require_wished_module("slope_sim.interfaces.v2.transport")
    factory = getattr(module, "create_v2_ecal_transport", None)
    assert callable(factory), "v2 raw transport factory is not implemented"
    v2_transport = factory(descriptor=descriptor, bindings=fake_v2_bindings)
    request.addfinalizer(v2_transport.close)
    v2_transport.publish(
        "/sim/wheel/state",
        encoded_wheel_state.payload,
        "slope_sim.interfaces.v2.WheelState",
        10,
    )
    v2_transport.wait_idle(timeout_sec=1.0)
    assert v2_transport.bindings.sent_payloads == [encoded_wheel_state.payload]


def test_payload_is_not_delivered_before_remote_metadata_verified(
    request, descriptor, fake_v2_bindings
) -> None:
    module = require_wished_module("slope_sim.interfaces.v2.transport")
    factory = getattr(module, "create_v2_ecal_transport", None)
    assert callable(factory), "v2 raw transport factory is not implemented"
    v2_transport = factory(descriptor=descriptor, bindings=fake_v2_bindings)
    request.addfinalizer(v2_transport.close)
    accepted = []
    v2_transport.subscribe(
        "/sim/wheel/command",
        "slope_sim.interfaces.v2.WheelCommand",
        lambda payload, _received_at: accepted.append(payload),
    )
    v2_transport.bindings.set_peer_count("/sim/wheel/command", 1)
    v2_transport.bindings.set_metadata_pending("/sim/wheel/command")
    v2_transport.bindings.emit("/sim/wheel/command", b"wire")
    assert accepted == []
    assert v2_transport.snapshot().topic_quality[0].protocol_state == "pending"


def test_v1_same_topic_conflict_latches_protocol_error(
    request, descriptor, fake_v2_bindings
) -> None:
    module = require_wished_module("slope_sim.interfaces.v2.transport")
    factory = getattr(module, "create_v2_ecal_transport", None)
    assert callable(factory), "v2 raw transport factory is not implemented"
    v2_transport = factory(descriptor=descriptor, bindings=fake_v2_bindings)
    request.addfinalizer(v2_transport.close)
    ecal_module = require_wished_module("slope_sim.interfaces.ecal_transport")
    ProtocolConflictError = getattr(ecal_module, "ProtocolConflictError", None)
    assert ProtocolConflictError is not None, "ProtocolConflictError is not implemented"
    v2_transport.bindings.set_protocol_conflict(
        "/sim/wheel/command", "slope_sim.interfaces.v1.WheelCommand"
    )
    with pytest.raises(ProtocolConflictError, match="protocol conflict"):
        v2_transport.poll_peer_state()
    quality = quality_for(v2_transport.snapshot(), "/sim/wheel/command")
    assert quality.protocol_state == "conflict"
    assert quality.error_count == 1
```

慢 send 测试继续要求同一 topic 最多 native owner+latest 两帧、第三帧只覆盖 latest 并精确 drop；五 topic 共享 lane 外全局有界 latest，不新增 FIFO 或物理线程阻塞。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_ecal_v2_transport.py tests/test_ecal_transport.py`

Expected: pytest 正常收集后 `FAILED`，首个失败明确为 v2 raw transport factory/behavior 尚未实现；不得是 collection error、fixture error 或 skip。现有 typed transport 仍会 ParseFromString，且还没有 v2 channel registry。

- [ ] **Step 3: 把 channel binding 与发送 lane 解耦**

```python
@dataclass(frozen=True)
class _ChannelBinding:
    topic: str
    direction: str
    type_name: str
    message_type: type[Message] | None = None
    descriptor: DescriptorIdentity | None = None
    raw_wire: bool = False


class ProtocolConflictError(RuntimeError):
    """表示本轮 discovery 已提交可审计的同话题协议冲突。"""


def _v2_channel_bindings(descriptor: DescriptorIdentity) -> tuple[_ChannelBinding, ...]:
    return tuple(
        _ChannelBinding(
            contract.topic,
            contract.direction,
            contract.type_name,
            descriptor=descriptor,
            raw_wire=True,
        )
        for contract in V2_TOPICS
    )
```

`EcalTransport.__init__` 新增私有依赖注入参数 `channel_bindings`，未传时仍要求/创建阶段三 `InterfaceConfig` 并调用旧 `_channel_bindings(config)`。传入 v2 bindings 时禁止同时传阶段三 config，显式 `queue_size` 提供容量；构造器校验 topic 唯一、方向合法、恰好一个 subscribe channel，并从该通道推导 `_command_topic`。`_on_payload()`、discovery、reconnect 和 delivery gate 全部使用 `_command_topic`，不再在核心 lane 中读取 `config.wheel_command/lidar_front/lidar_rear`。这样 v2 五话题实例不携带隐藏的 v1 六通道配置。

- [ ] **Step 4: 统一 typed v1/raw v2 binding 协议**

```python
class EcalBindings:
    def create_publisher(self, channel: _ChannelBinding) -> _ProtoResource:
        if channel.message_type is None or channel.raw_wire:
            raise RuntimeError("v1 typed publisher requires message_type")
        return _ProtoResource(
            self.proto_core.Publisher(channel.message_type, channel.topic),
            direction="publisher",
        )

    def send(self, publisher: _ProtoResource, payload: bytes, channel: _ChannelBinding) -> None:
        if channel.message_type is None:
            raise RuntimeError("v1 typed publisher requires message_type")
        message = channel.message_type()
        message.ParseFromString(payload)
        result = _resource_raw(publisher).send(message)
        if result is False:
            raise RuntimeError("eCAL ProtoPublisher.send returned False")
```

新增 `_RawV2Bindings` 使用 Task 8 的 `EcalRawBindings`，`send()` 直接调用 raw publisher `send(bytes(payload))`。native receive callback 只把 `RawReceivedFrame` 放入每 topic 有界 receive lane；receive worker 依次计算 payload SHA-256、校验复制的远端 metadata、调用对应 v2 codec parse、校验带内身份/业务模型，然后才把已验证 payload 交给 `EcalTransport._on_payload()`。现有 publisher worker 的 `_bindings.send(publisher, message.payload, channel)` 只调用一次，不按 wire mode 自行 parse。

receive lane 对命令话题使用 owner+latest 两槽；覆盖 latest 精确计入 command ingress drop，并使正式会话失败，不能把命令排成无界 FIFO。关闭先禁止 enqueue、等待 native callback 和 receive worker 收敛，再释放 subscriber。

- [ ] **Step 5: 在 poll 中原子合并 count 与 metadata**

每次 `poll_peer_state()` 在 discovery gate 内按 channel 顺序执行：读取 exact count -> 读取 monitoring endpoint metadata -> 生成 `RemoteEndpointSnapshot`。随后在 delivery gate 内一次提交所有 topic quality；callback 直接复制本帧 `data_type_info`，worker 先校验该 metadata，再读取同一 topic 的 protocol gate。`pending` 不交付，`conflict` 使正式 v2 transport 进入 error，只有“本帧 metadata 匹配且 topic gate 为 verified”才允许解析与交付。关闭仍先等 in-flight discovery，再 remove callback、释放 pub/sub 和 participant。

```python
if channel.raw_wire:
    endpoint_snapshot = self._bindings.snapshot_remote_endpoints(
        channel, resource, peer_count
    )
    verification = endpoint_snapshot.verification
    self._set_topic_protocol_locked(
        channel.topic,
        verification,
    )
    if verification.state is ProtocolVerificationState.CONFLICT:
        raise ProtocolConflictError(
            f"protocol conflict on {channel.topic}: {verification.detail}"
        )
```

- [ ] **Step 6: 提供唯一 v2 factory**

```python
# slope_sim/interfaces/v2/transport.py
"""阶段四 transport factory：只在 Phase-0 通过后创建五话题 raw eCAL。"""
def create_v2_ecal_transport(
    *,
    descriptor: DescriptorIdentity,
    queue_size: int = 32,
    participant_name: str = "slope-sim-v2",
    bindings: object | None = None,
) -> EcalTransport:
    selected = _RawV2Bindings() if bindings is None else bindings
    return EcalTransport(
        bindings=selected,
        queue_size=queue_size,
        participant_name=participant_name,
        role="simulation",
        channel_bindings=_v2_channel_bindings(descriptor),
    )
```

阶段 A 的 factory 只创建 Simulator 方向：唯一订阅是 WheelCommand，四个输出是 raw publisher；不得用 `role="peer"` 反转后绕过尚未实现的 LiDAR/RTK/IMU 业务模型校验。C++ 消费方向由阶段 C 的独立 SDK 实现。`create_transport()` 和 `InterfaceConfig.default()` 仍返回阶段三 v1 行为；阶段 B 补齐三类传感器模型/codec 并能生成单 LiDAR/三点 RTK 后，才把正式 Simulator 入口切换到该 v2 factory。

- [ ] **Step 7: 运行 GREEN 与 v1 transport 回归**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_ecal_v2_transport.py tests/test_ecal_transport.py tests/test_ecal_process_roundtrip.py -m "not ecal"`

Expected: PASS，v2 fake transport 证明 raw bytes 不变、metadata gate 和 exact count；阶段三 typed v1 测试全部不变。

- [ ] **Step 8: REFACTOR typed/raw binding 与 lane 收口**

只共用 channel 遍历、资源关闭和 owner+latest 计数逻辑；不得让 v1/v2 互相 parse、改变 callback metadata 来源或把 `ProtocolConflictError` 降成通用异常。原样重跑 Step 7 的 GREEN 命令，Expected: PASS。

### Task 12：建立 runtime 可组合的会话/命令协议控制器

**Files:**
- Create: `slope_sim/interfaces/v2/runtime_protocol.py`
- Create: `tests/stage4/test_v2_runtime_protocol.py`
- Modify: `tests/test_interface_pause_rebuild.py`
- Modify: `tests/test_interface_runtime.py`

- [ ] **Step 1: 写 poll 顺序、认领、重建和迟到回调 RED**

```python
from dataclasses import replace


def test_refresh_polls_before_reading_command_peer_count(
    model, descriptor, transport
) -> None:
    controller_type = require_wished_module(
        "slope_sim.interfaces.v2.runtime_protocol"
    ).V2RuntimeProtocol
    controller = controller_type(model, transport=transport, descriptor=descriptor)
    transport.calls.clear()
    controller.refresh_transport()
    assert transport.calls[:2] == ["poll_peer_state", "snapshot"]


def test_prepare_invalidates_ingress_and_abort_does_not_restore_token(
    model, descriptor, transport, command
) -> None:
    controller_type = require_wished_module(
        "slope_sim.interfaces.v2.runtime_protocol"
    ).V2RuntimeProtocol
    controller = controller_type(model, transport=transport, descriptor=descriptor)
    transport.set_command_protocol("verified", peer_count=1)
    controller.refresh_transport()
    old = controller.capture_ingress()
    assert controller.accept_decoded_command(command, received_at=1.0, ingress=old)
    controller.prepare_world_rebuild()
    controller.abort_world_rebuild()
    assert controller.accept_decoded_command(command, received_at=1.01, ingress=old) is False
    assert controller.snapshot().world_generation == 1
    assert controller.snapshot().command_generation == 2
    assert controller.mailbox.decision(now=1.01).waiting is True


def test_commit_alone_advances_world_and_resets_output_sequences(
    model, descriptor, transport
) -> None:
    controller_type = require_wished_module(
        "slope_sim.interfaces.v2.runtime_protocol"
    ).V2RuntimeProtocol
    controller = controller_type(model, transport=transport, descriptor=descriptor)
    before = controller.reserve_output("/sim/wheel/state")
    controller.prepare_world_rebuild()
    controller.commit_world_rebuild()
    after = controller.reserve_output("/sim/wheel/state")
    assert before.world_generation == 1 and before.sequence == 0
    assert after.world_generation == 2 and after.sequence == 0


def test_verified_to_pending_revokes_once_and_blocks_claim(
    model, descriptor, transport
) -> None:
    controller_type = require_wished_module(
        "slope_sim.interfaces.v2.runtime_protocol"
    ).V2RuntimeProtocol
    controller = controller_type(model, transport=transport, descriptor=descriptor)
    transport.set_command_protocol("verified", peer_count=1)
    controller.refresh_transport()
    before = controller.snapshot().command_generation
    transport.set_command_protocol("pending", peer_count=1)
    controller.refresh_transport()
    controller.refresh_transport()
    assert controller.snapshot().command_generation == before + 1
    assert controller.mailbox.decision(now=1.0).waiting is True
    assert controller.accept_payload(valid_payload(), received_at=1.0) is False


def test_generation_exhaustion_is_terminal_without_partial_authority(
    model, descriptor, transport, command
) -> None:
    controller_type = require_wished_module(
        "slope_sim.interfaces.v2.runtime_protocol"
    ).V2RuntimeProtocol
    controller = controller_type(model, transport=transport, descriptor=descriptor)
    transport.set_command_protocol("verified", peer_count=1)
    controller.refresh_transport()
    old_ingress = controller.capture_ingress()
    assert controller.accept_decoded_command(
        command, received_at=1.0, ingress=old_ingress
    )

    with controller._session._lock:
        controller._session._command_generation = (1 << 64) - 1
    authority_before = controller.snapshot().authority
    transport.set_command_protocol("conflict", peer_count=2)

    with pytest.raises(OverflowError, match="command_generation exhausted"):
        controller.refresh_transport()

    failed = controller.snapshot()
    assert failed.authority == authority_before
    assert failed.fatal_error == "command_generation exhausted"
    assert controller.mailbox.decision(now=1.0).waiting is True
    boundary_command = replace(
        command, command_generation=(1 << 64) - 1, sequence=1
    )
    assert controller.accept_decoded_command(
        boundary_command, received_at=1.01, ingress=old_ingress
    ) is False
    assert controller.accept_payload(valid_payload(), received_at=1.02) is False
```

再覆盖：慢 `poll_peer_state` 完成后才取 snapshot；poll 期间 callback 到达、断开、重连、0->1->2->1；verified->waiting/pending/conflict->verified；poll 提交 conflict 后抛错仍先 snapshot 并撤权；prepare/commit/abort/fault；decode 期间 rebuild；错误 owner；关闭后的 late callback；100 ms 墙钟超时；所有拒绝不刷新 mailbox 墙钟。对 `observe_peer_count()`、`suspend_protocol()`、错误 owner `accept()` 和 rebuild 路径传播的 `OverflowError` 使用同一参数化 fault oracle，证明 controller 一律进入不可恢复 fatal 状态，而不是只处理上例的 conflict 路径。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_runtime_protocol.py`

Expected: pytest 正常收集后 `FAILED`，失败消息精确指出 runtime protocol 行为尚未实现；不得是 collection error、fixture error 或 skip。

- [ ] **Step 3: 组合既有 mailbox 而不复制控制规则**

```python
# slope_sim/interfaces/v2/runtime_protocol.py
"""阶段四 runtime 协议控制器：串行化 transport、命令权、邮箱和重建事务。"""
@dataclass(frozen=True)
class IngressToken:
    lifecycle_generation: int
    subscription_token: int
    mailbox_generation: int


@dataclass(frozen=True)
class V2RuntimeSnapshot:
    simulation_session_id: bytes
    descriptor_sha256: bytes
    world_generation: int
    command_generation: int
    lifecycle_generation: int
    subscription_token: int
    command_protocol_state: str
    authority: AuthoritySnapshot
    closed: bool
    fatal_error: str | None


class V2RuntimeProtocol:
    def __init__(
        self,
        model: RobotModelSpec,
        *,
        transport: Transport,
        descriptor: DescriptorIdentity,
        monotonic: Callable[[], float] = time.monotonic,
        timeout_sec: float = 0.100,
    ) -> None:
        self._model = model
        self._transport = transport
        self._session = ProtocolSession(descriptor)
        self._authority = CommandAuthority(self._session)
        self._mailbox = WheelCommandMailbox(model, timeout_sec=timeout_sec)
        self._codec = V2ProtoCodec(descriptor)
        self._monotonic = monotonic
        self._lifecycle_generation = 0
        self._subscription_token = 0
        self._command_protocol_state = "not_checked"
        self._closed = False
        self._fatal_error: str | None = None
        self._condition = Condition()

    def refresh_transport(self) -> TransportSnapshot:
        poll = getattr(self._transport, "poll_peer_state", None)
        if not callable(poll):
            raise RuntimeError("v2 transport must expose poll_peer_state")
        conflict_error: ProtocolConflictError | None = None
        try:
            poll()
        except ProtocolConflictError as error:
            conflict_error = error
        snapshot = self._transport.snapshot()
        command_quality = next(
            item for item in snapshot.topic_quality
            if item.topic == "/sim/wheel/command"
        )
        with self._condition:
            if self._closed:
                raise RuntimeError("v2 runtime protocol is closed")
            if command_quality.protocol_state != "verified":
                self._apply_unverified_protocol(command_quality)
            else:
                if command_quality.peer_count is None:
                    raise RuntimeError("verified command topic requires exact peer_count")
                transition = self._authority.observe_peer_count(command_quality.peer_count)
                if transition.clear_mailbox:
                    self._mailbox.clear()
                self._command_protocol_state = "verified"
        if conflict_error is not None:
            raise conflict_error
        return snapshot
```

本类的公开边界必须在本 Task 完整实现，不能让测试依赖未定义的伪 API：

```python
@property
def mailbox(self) -> WheelCommandMailbox: ...

def reserve_output(self, topic: str) -> OutputIdentity: ...
def snapshot(self) -> V2RuntimeSnapshot: ...
def refresh_transport(self) -> TransportSnapshot: ...
def capture_ingress(self) -> IngressToken: ...
def accept_payload(self, payload: bytes, *, received_at: float) -> bool: ...
def accept_decoded_command(
    self,
    command: WheelCommandV2,
    *,
    received_at: float,
    ingress: IngressToken,
) -> bool: ...
def prepare_world_rebuild(self) -> None: ...
def commit_world_rebuild(self) -> int: ...
def abort_world_rebuild(self) -> None: ...
def fault_world_rebuild(self) -> None: ...
def close(self) -> None: ...
```

`mailbox` 只暴露既有线程安全对象供物理主线程调用 `decision/snapshot`；外部不得直接 `accept/clear`。`reserve_output()` 在 controller 条件锁内检查未关闭且无 fatal error 后转发到唯一 `ProtocolSession`。`snapshot()` 在同一锁内组合 session properties、`AuthoritySnapshot`、lifecycle/subscription/protocol/closed/fatal error，返回后不持有可变引用。`close()` 幂等地在 controller 锁内先置 closed、递增 lifecycle/subscription、撤权并清 mailbox，释放该锁后才调用 transport close/等待在途 callback，避免 transport 回调重入 controller 自锁；关闭后的 refresh/reserve/capture/rebuild 抛明确 lifecycle error，accept 返回 False，迟到 callback 不改变任何计数或 owner。`monotonic` 是 100 ms decision/snapshot 的唯一默认墙钟，不能保存后不用。

所有调用 authority 且可能推进 generation 的 controller 边界都必须捕获 `OverflowError` 并进入同一个锁内 fatal 转换：保存稳定 `fatal_error`、递增 lifecycle/subscription token 使所有在途 ingress 失效、清空 mailbox 并唤醒等待者，然后原样重新抛出首次异常。该转换不得修改或重建 authority；后续 `accept_payload()`/`accept_decoded_command()` 固定返回 False，capture/reserve/refresh/rebuild 固定抛含原始原因的 lifecycle error，只有 `snapshot()` 与幂等 `close()` 仍可用。Task 6 保证 authority 自身先推进后提交，因此 controller 观察到的 fatal snapshot 必须与异常前 authority snapshot 完全相等。

`poll_peer_state()` 和紧随其后的 transport `snapshot()` 都在 controller 锁外执行，避免 discovery callback 重入自锁；取得不可变 snapshot 后才进入 controller 条件锁，串行提交 authority/protocol/mailbox 状态。`_apply_unverified_protocol()` 是只允许在该锁内调用的私有 helper，并要求 raw v2 quality 始终提供非空 `peer_count`。若上一状态是 `verified`，它调用一次 `authority.suspend_protocol(peer_count)`；否则调用 `authority.observe_peer_count(peer_count)`，因此未验证期间真实的 1->0/>1 peer 边沿仍只推进一次。随后保存新 protocol state；只有撤权或 peer 边沿返回 `clear_mailbox` 时才清 mailbox/失效 ingress token。重复 poll 同一 waiting/pending/conflict 不得重复推进。它仍把精确 peer count 交给 authority，使 WheelState 的 `WAITING/CLAIMABLE/ACTIVE/CONFLICT` 与 0/1/1/>1 数量关系保持成立，但 `accept_payload()` 另以“`_command_protocol_state == verified`”作为锁内前置门，暂态 `CLAIMABLE` 不会接收未验证命令。`poll_peer_state()` 只有在 conflict quality 已原子提交后才抛 `ProtocolConflictError`，controller 捕获该专用异常、读取本轮 snapshot、在 controller 锁内撤权/清 mailbox，再重新抛出；其他 discovery 异常不伪装成已提交 observation，而是沿既有 transport fault 路径安全停车。

`capture_ingress()` 在锁内把 lifecycle、subscription 和 `mailbox.capture_generation()` 一次装入 `IngressToken`；`accept_payload()` 锁内取得 token、锁外 decode，然后调用 `accept_decoded_command(command, received_at, ingress)` 回锁复验三项，再把提交闭包传给 authority。后者是迟到 decode 并发测试使用的明确边界，不提供给业务调用者：

```python
accepted = self._authority.accept(
    command,
    self._model,
    commit=lambda: self._mailbox.accept(
        command.to_v1_motion(),
        received_at=received_at,
        generation=ingress.mailbox_generation,
    ),
)
```

authority 只有在该闭包返回 True 后才认领或推进 sequence；旧 generation 返回 False 时 owner 状态保持原样。

- [ ] **Step 4: 绑定 rebuild 的单一线性化点**

```python
def prepare_world_rebuild(self) -> None:
    with self._condition:
        self._lifecycle_generation += 1
        self._subscription_token += 1
        transition = self._authority.prepare_world_rebuild()
        if transition.clear_mailbox:
            self._mailbox.clear()

def commit_world_rebuild(self) -> int:
    with self._condition:
        return self._authority.commit_world_rebuild()
```

abort/fault 都只结束 prepared 状态并创建新的 subscription token；commit 才推进 world。`InterfaceRuntime` 在阶段 B 接入该 controller 时，必须在现有 mailbox clear 的同一生命周期临界区调用这些方法，不得保留第二套 world/command generation。

- [ ] **Step 5: 运行 GREEN 和阶段三 reconnect 回归**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_runtime_protocol.py tests/test_interface_pause_rebuild.py tests/test_interface_runtime.py tests/test_ecal_transport.py`

Expected: PASS，且明确存在“先 poll_peer_state，再读取 snapshot/peer count”的回归；迟到旧 token 不能提交到 abort 恢复或新 world。

- [ ] **Step 6: REFACTOR controller 锁内转换与 snapshot 组合**

只合并 lifecycle/subscription token 失效和 snapshot 构造重复代码；不得增加第二套 generation、扩大锁外提交窗口或改变 conflict 先 snapshot 后重抛顺序。原样重跑 Step 5 的 GREEN 命令，Expected: PASS。

### Task 13：冻结 Python/C++ 双向 golden bytes

**Files:**
- Create: `scripts/generate_stage4_v2_goldens.py`
- Create: `tests/fixtures/stage4/v2/WheelCommand.bin`
- Create: `tests/fixtures/stage4/v2/WheelState.bin`
- Create: `tests/fixtures/stage4/v2/LidarPointCloud.bin`
- Create: `tests/fixtures/stage4/v2/RtkState.bin`
- Create: `tests/fixtures/stage4/v2/ImuAttitude.bin`
- Create: `tests/fixtures/stage4/v2/manifest.json`
- Create: `tests/stage4/test_cpp_v2_interop.py`

- [ ] **Step 1: 写五消息双向 RED**

```python
import os
from pathlib import Path


@pytest.mark.parametrize("message_name", [
    "WheelCommand",
    "WheelState",
    "LidarPointCloud",
    "RtkState",
    "ImuAttitude",
])
def test_python_golden_decodes_in_cpp_without_byte_change(
    message_name, golden_dir, v2_descriptor_path
) -> None:
    golden_file = golden_dir / f"{message_name}.bin"
    assert golden_file.is_file(), f"golden fixture is not implemented: {golden_file}"
    raw_build = os.environ.get("STAGE4_PHASE0_BUILD_DIR")
    assert raw_build, "STAGE4_PHASE0_BUILD_DIR must name the Task 9 GREEN build"
    stage4_phase0_build = Path(raw_build)
    assert (stage4_phase0_build / "v2_golden").is_file(), (
        "Task 9 v2_golden build is missing"
    )
    result = run_v2_golden(
        stage4_phase0_build,
        "decode",
        "--descriptor-set",
        v2_descriptor_path,
        message_name,
        golden_file,
    )
    assert result["payload_sha256"] == manifest(message_name)["payload_sha256"]
    assert result["descriptor_sha256"] == frozen_descriptor_hex()


@pytest.mark.parametrize("message_name", [
    "WheelCommand",
    "WheelState",
    "LidarPointCloud",
    "RtkState",
    "ImuAttitude",
])
def test_cpp_encodes_exact_python_golden_bytes(
    message_name, golden_dir, v2_descriptor_path
) -> None:
    golden_file = golden_dir / f"{message_name}.bin"
    assert golden_file.is_file(), f"golden fixture is not implemented: {golden_file}"
    raw_build = os.environ.get("STAGE4_PHASE0_BUILD_DIR")
    assert raw_build, "STAGE4_PHASE0_BUILD_DIR must name the Task 9 GREEN build"
    stage4_phase0_build = Path(raw_build)
    assert (stage4_phase0_build / "v2_golden").is_file(), (
        "Task 9 v2_golden build is missing"
    )
    output = run_cpp_fixture_encoder(
        stage4_phase0_build,
        descriptor_set=v2_descriptor_path,
    )
    assert (output / f"{message_name}.bin").read_bytes() == (
        golden_file
    ).read_bytes()
```

- [ ] **Step 2: 运行 RED**

Run: `STAGE4_PHASE0_BUILD_DIR=$PWD/build/stage4-phase0 conda run -n slope-sim python -m pytest -q tests/stage4/test_cpp_v2_interop.py`

Expected: pytest 正常收集后 `FAILED`，首个失败是明确的 `golden fixture is not implemented` 行为断言；Task 9 build 已存在，不得是 collection error、fixture error、缺 build 或 skip。

- [ ] **Step 3: 用固定字段创建一次性 golden**

生成器用 generated v2 Protobuf 直接构造五个完整消息，所有顶层消息使用相同 session/digest；字段固定如下：

```python
FIXTURE_IDENTITY = {
    "simulation_session_id": bytes.fromhex("00112233445566778899aabbccddeeff"),
    "world_generation": 7,
    "descriptor_sha256": load_v2_descriptor().sha256,
}

MESSAGES = {
    "WheelCommand": pb.WheelCommand(
        timestamp_ns=1_000_000_000,
        drive_wheel_speed_rad_s=[1.5, -2.25],
        steering_wheel_speed_rad_s=[],
        sequence=3,
        command_generation=11,
        source_id="golden.command",
        source_session_id=bytes.fromhex("ffeeddccbbaa99887766554433221100"),
        robot_model="df_mid",
        **FIXTURE_IDENTITY,
    ),
    "RtkState": pb.RtkState(
        timestamp_ns=1_000_000_000,
        sequence=5,
        frame_id="world",
        left=pb.Point3d(x_m=1.0, y_m=0.5, z_m=0.2),
        center=pb.Point3d(x_m=1.0, y_m=0.0, z_m=0.2),
        right=pb.Point3d(x_m=1.0, y_m=-0.5, z_m=0.2),
        heading_rad=0.25,
        **FIXTURE_IDENTITY,
    ),
}
```

WheelState、LidarPointCloud 和 ImuAttitude 使用同一 identity，分别设置非空 4+2 轮数组与 ACTIVE owner、两个不重合的三维点、非零 roll/pitch。脚本 `--create` 对每个 `.bin` 和 manifest 使用 exclusive create；默认模式重新编码并逐 byte 校验，不能自动更新既有 golden。

- [ ] **Step 4: 创建 fixture 并运行双向 GREEN**

Run: `conda run -n slope-sim python scripts/generate_stage4_v2_goldens.py --create`

Expected: 五个非空 `.bin` 和 manifest 原子创建，每项记录 type name、payload SHA-256、descriptor SHA-256、session hex 和 generation。

Run: `STAGE4_PHASE0_BUILD_DIR=$PWD/build/stage4-phase0 conda run -n slope-sim python -m pytest -q tests/stage4/test_cpp_v2_interop.py`

Expected: `10 passed`，五个 Python->C++ 与五个 C++->Python payload 逐 byte 相同。

Run: `conda run -n slope-sim python scripts/generate_stage4_v2_goldens.py`

Expected: rc=0，输出 `5 golden payloads verified`；任何 schema/codec 漂移非零退出。

- [ ] **Step 5: REFACTOR golden 消息构造与 manifest 校验**

只共用五消息 identity 填充、exclusive-create 和 manifest 排序校验；不得重写既有 fixture 或改变任何 byte。重跑 Step 4 的双向 pytest 与不带 `--create` 的 verifier 两条 Run（不重复执行 exclusive create），Expected 分别为 `10 passed` 和 `5 golden payloads verified`。

### Task 14：中文说明、聚焦回归与独立六维审查

**Files:**
- Create: `docs/阶段四协议与命令权说明.md`
- Modify: `README.md`
- Modify: `docs/阶段四交付报告.md`
- Test: A 的全部 Python/C++ 测试

本 Task 是文档与验收收口，不引入生产行为，因此不另造伪 RED。任一验收或独立审查发现实现缺陷时，必须回到对应 Task 补一个可正常收集的 RED，再完成 GREEN-REFACTOR，随后从本 Task Step 2 重新验收。

- [ ] **Step 1: 写面向使用者的协议说明**

说明必须用中文回答以下固定问题：

```text
eCAL 是什么：同机多进程实时消息总线，本项目只把它当传输管道，不让它决定物理真值。
Protobuf v2 是什么：Python/C++ 共用的线格式；descriptor SHA 用来证明双方理解的是同一份 schema。
simulation_session_id：每次 Simulator 进程启动都变化，防止重启前后数据拼接。
world_generation：只有成功换车/换场景提交才加一；四个输出 sequence 从 0 重启。
command_generation：命令源丢失、冲突、撤权或 rebuild prepare 时加一；旧控制 token 永不恢复。
WAITING/CLAIMABLE/ACTIVE/CONFLICT：0 个、1 个未认领、1 个已认领、多个命令发布者。
为什么先 poll 再读状态：poll 完成后 snapshot 才代表本轮最新 peer count/metadata，反过来会读到旧连接状态。
```

提供一张状态转换表和一段“断开 -> 安全停车 -> 重连 -> 读取新 WheelState -> sequence 0 重新认领”的命令工具流程；不得声称 LiDAR/RTK runtime、Recorder 或发行包在 A 已完成。

- [ ] **Step 2: 运行聚焦非 eCAL 套件**

Run: `STAGE4_PROTOC="$STAGE4_PROTOC" STAGE4_PHASE0_BUILD_DIR="$PWD/build/stage4-phase0" conda run -n slope-sim python -m pytest -q tests/stage4/test_v1_descriptor_frozen.py tests/stage4/test_v2_proto_contract.py tests/stage4/test_v2_generated_artifacts.py tests/stage4/test_v2_descriptor.py tests/stage4/test_v2_codec.py tests/stage4/test_v2_session.py tests/stage4/test_command_authority.py tests/stage4/test_transport_v2_metadata.py tests/stage4/test_ecal_v2_raw_unit.py tests/stage4/test_ecal_v2_phase0.py tests/stage4/test_ecal_v2_transport.py tests/stage4/test_v2_runtime_protocol.py tests/stage4/test_cpp_phase0_build.py tests/stage4/test_cpp_v2_interop.py -m "not ecal"`

Expected: PASS，无 skip；两个路径变量都已经过前置版本/ABI 检查，C++ build/interop 与独立 v2 生成测试必须实际执行。

- [ ] **Step 3: 运行受影响阶段三回归**

Run: `conda run -n slope-sim python -m pytest -q tests/test_proto_contract.py tests/test_interface_codec.py tests/test_interface_models.py tests/test_wheel_mailbox.py tests/test_local_transport.py tests/test_ecal_transport.py tests/test_ecal_process_roundtrip.py tests/test_interface_runtime.py tests/test_interface_pause_rebuild.py -m "not ecal"`

Expected: PASS，v1 descriptor/source SHA 不变。

- [ ] **Step 4: 运行完整非 eCAL 回归**

Run: `STAGE4_PROTOC="$STAGE4_PROTOC" STAGE4_PHASE0_BUILD_DIR="$PWD/build/stage4-phase0" conda run -n slope-sim python -m pytest -q -m "not ecal"`

Expected: PASS；不启动 PyBullet GUI、Xvfb 或真实 eCAL。若失败，先定位并以聚焦 RED/GREEN 修复，不能把失败项目移出集合。

- [ ] **Step 5: 运行静态一致性检查**

Run: `conda run -n slope-sim python scripts/freeze_v2_descriptor.py`

Expected: rc=0，descriptor 与冻结 SHA 一致。

Run: `conda run -n slope-sim python scripts/generate_stage4_v2_goldens.py`

Expected: `5 golden payloads verified`。

Run: `git diff --check`

Expected: 无输出。

Run: `git diff --exit-code ce3bee0 -- proto/slope_sim_interfaces.proto && git status --short`

Expected: 第一条无输出；status 只列出本阶段计划内文件和进入阶段 A 前已存在的用户修改。

- [ ] **Step 6: 更新交付报告，严格区分已证明与未接线**

报告记录：v1/v2 descriptor SHA、C++ ABI/`ldd`、非 eCAL 测试、真实 Phase-0 唯一运行路径及四场景结果。状态固定拆成：

```markdown
| 项目 | 状态 |
|---|---|
| v2 schema/session/authority | PASS（附测试证据） |
| Python/C++ raw Phase-0 | PASS 或 BLOCKED（附唯一运行证据） |
| 单 LiDAR/三点 RTK runtime | 未执行，阶段 B |
| C++ SDK/Recorder | 未执行，阶段 C |
| 最终真实联合负载 | 未执行，阶段 E |
```

- [ ] **Step 7: 启动只读六维独立审查**

审查任务不得修改代码，必须分别核对需求完整性、逻辑正确性、边界情况、代码质量、测试覆盖和实际运行结果。重点检查 descriptor 是否真为远端 metadata、peer count 是否保留整数、claim 是否晚于全部校验、prepare/abort 是否可能恢复旧 token、raw bytes 是否被 typed serializer 改写，以及真实 Phase-0 是否只有本次授权运行。

Expected: Critical=0、Important=0。若有发现，审查者只报文件/行号和证据；实现者回到相应 Task 先补 RED 再修复，复核通过后才可把 A 标为完成。

## 阶段 A 完成定义

只有以下条件全部成立才能进入阶段 B/C：

- v1 source/descriptor 冻结测试通过；v2 descriptor/golden 可复现且不能静默覆盖。
- Python wheel codec 确定性编码一次，同一 raw bytes 同时可供日志和 transport。
- session/world/command generation、精确 peer count、四态 owner、rebuild/reconnect/late callback 全部通过聚焦测试。
- Python/C++ 双向五消息 golden bytes 完全一致；Python/C++ callback 只复制拥有所有权的 bytes/type metadata/send timestamp/send clock/received_at，worker 严格按 hash -> 远端 metadata -> Protobuf parse -> 带内身份/业务模型校验的顺序处理。
- 获得单独授权的真实 eCAL Phase-0 四场景一次运行全部通过；同话题 v1 peer 被硬拒绝且 accepted=0。
- 完整非 eCAL 回归通过，独立六维审查 Critical/Important 为 0，报告没有把 B-E 的未完成项写成通过。

若真实 metadata 或同 topic 隔离不能证明，阶段 A 状态必须是 BLOCKED；下一步不是继续写 runtime，而是把证据交给用户裁决是否切换 `/sim/v2/...`。
