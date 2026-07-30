# 阶段三 eCAL、企业传感器与接口基线实施计划

> **Execution:** Use `subagent-driven-development` only when the user selects delegated execution; otherwise use `executing-plans`. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有四车型、三场地和动态障碍物基础上，交付真实 eCAL 轮子闭环、前后点云、RTK、IMU、企业 Dashboard、版本化场景文件和完整阶段三验收。

**Architecture:** 仿真核心只依赖不可变接口对象和窄传输协议；Protobuf 负责编码，`LocalTransport` 与 `EcalTransport` 共享校验、时钟、状态和日志。PyBullet 只在物理主线程中读取或修改，eCAL 回调、日志写入和 Qt 展示通过有界 mailbox 与不可变快照隔离。

**Tech Stack:** Python 3.10、PyBullet、PySide6、pytest、PyYAML、Protobuf 6.33.6、grpcio-tools 1.76.0、Eclipse eCAL 6.1.1、X11/XWayland `xdotool`/XRes。

**Design Spec:** `docs/superpowers/specs/2026-07-21-stage3-ecal-enterprise-interfaces-design.md`

**User-updated acceptance:** 用户最新将阶段三设计期间补充的原 4:1 门禁替换为名义 `67:33` 初始平铺：Dashboard 宽度精确按 `33/100` 计算，Main 使用余宽。该项仍为硬门禁，即使早期原始需求未列出也不得降级为可选项。根需求规格对产品边界的优先级高于本实施计划：默认 Dashboard 固定为 15 页、13 个图表页，接触力、接触点数和打滑指标只进入显式开发者诊断。

**Selected execution:** 用户已选择子智能体并行执行。Task 1-12 已完成；修订后的 Task 13A、13B、13C 按共享契约顺序串行实现，但可与只读审查和 Task 14 环境调查并行。Task 14 由主线程统一集成、验收和审查。并行任务必须有互不重叠的文件写集，子智能体最大深度为 1，主线程复核全部结果后再运行完整回归。未经用户明确要求不执行 Git commit。

---

## 文件结构

- `proto/slope_sim_interfaces.proto`：唯一企业 Protobuf 源文件。
- `slope_sim/interfaces/generated/`：由脚本生成的 Python Protobuf 类型。
- `slope_sim/interfaces/models.py`：不可变企业消息对象与轮子命令校验。
- `slope_sim/interfaces/config.py`：话题、频率、传输和队列配置。
- `slope_sim/interfaces/codec.py`：接口对象与 Protobuf 的转换。
- `slope_sim/interfaces/clock.py`：精确仿真时钟和周期调度。
- `slope_sim/interfaces/status.py`：逐话题频率和不可变状态快照。
- `slope_sim/interfaces/dashboard_snapshot.py`：Dashboard 专用组合快照及 LiDAR 俯视点类型。
- `slope_sim/interfaces/transport.py`：传输协议与本地实现。
- `slope_sim/interfaces/ecal_transport.py`：eCAL 版本兼容绑定与正式适配器。
- `slope_sim/interfaces/wheel.py`：命令 mailbox、超时保护和实际轮子状态。
- `slope_sim/interfaces/runtime.py`：主循环接口编排与重绑定生命周期。
- `slope_sim/interfaces/logging.py`：长度前缀 Protobuf 日志和 JSONL 事件日志。
- `slope_sim/realtime.py`：共享绝对 deadline 和 50 ms runtime 观测节拍。
- `slope_sim/sensor_backend.py`：传感器需要的窄 PyBullet 后端。
- `slope_sim/truth_sensors.py`：RTK 与 IMU 真值生成。
- `slope_sim/lidar_pointcloud.py`：多层射线与点云生成。
- `slope_sim/scene_config.py`：版本化场景文档、原子导出和事务加载。
- `slope_sim/window_layout.py`：主屏 67:33 初始窗口布局。
- `slope_sim/dashboard_charts.py`：纯 Python 图表规格、20 秒缓冲和接口质量增量计算。

现有 `slope_sim/sensors.py` 的二维摘要只保留内部诊断兼容，不承载企业点云。

---

## 当前实施证据（2026-07-30 收口复验）

- Task 1-12、12R、13A-13C、14A 的生产实现和 TDD 回归均已落地；旧 Task 1-12 的未勾选步骤只保留原始配方，从 Task 12R 起的复选框是本轮权威状态。
- `496 passed`、schema v3/button、旧全量和旧真实 eCAL 数字只保留为历史证据，不能证明当前 schema v4、键盘驾驶或 post-fix eCAL 合同。
- 当前 fresh 证据为：cadence/自动/手动/eCAL 进程聚焦组 `272 passed, 4 deselected`；transport/runtime/process 组合 `365 passed, 4 deselected`；阶段一 `12/12`、阶段二 `SUMMARY pass=19 fail=0`、阶段三 DIRECT `SUMMARY pass=21 fail=0`；全量非 eCAL `2209 passed, 4 deselected in 102.53s`。四个 deselect 均为真实 eCAL 标记用例，四组 schema v4 GUI 仍按本计划后续步骤补证。
- DIRECT 首次串行运行曾得到 `accepted=873/1200`；诊断确认是 5 秒墙钟内仿真节拍不足，不是 logger drop。删除未被 runtime 使用的 rolling/incremental LiDAR 容器、恢复直接单批原子路径后，本轮正式复验为 `1200/1200`、`final_pending=0`、transport/logger drop=0、最大 Dashboard 间隔 `54.92 ms`；门槛未下调。
- 用户已授权并完成本次唯一 post-fix 命令执行，但 Codex 沙箱在正式测量前拒绝 eCAL UDP socket（`Operation not permitted`），退出码 `1`，未生成 peer/runtime 结果 JSON。完整环境阻断证据保留于 `/tmp/pybullet-df-postfix-ecal-gate.CBZyMMKJ`；它不能判定 post-fix P0。未自动重跑，后续有效复验仍需用户重新授权并切换到允许 eCAL socket 的环境，不得用修复前失败、并发污染或本次环境阻断替代。
- 本阶段改动尚未按任务提交 Git；这是用户要求“未明确请求不 commit”的结果，不得把缺少提交误报为功能通过，也不得擅自补提交。

---

## Task 1：Protobuf 依赖与消息契约

**Files:**

- Create: `proto/slope_sim_interfaces.proto`
- Create: `scripts/generate_protos.py`
- Create: `slope_sim/interfaces/__init__.py`
- Create: `slope_sim/interfaces/generated/__init__.py`
- Generate: `slope_sim/interfaces/generated/slope_sim_interfaces_pb2.py`
- Modify: `environment.yml`
- Modify: `pyproject.toml`
- Test: `tests/test_proto_contract.py`

- [ ] **Step 1: 写 descriptor 失败测试**

```python
# 阶段三 Protobuf 契约测试：锁定企业字段、编号和类型。
import pytest
from google.protobuf.descriptor import FieldDescriptor
from slope_sim.interfaces.generated import slope_sim_interfaces_pb2 as pb


def test_wheel_command_descriptor_is_versioned_and_stable():
    descriptor = pb.WheelCommand.DESCRIPTOR
    assert descriptor.full_name == "slope_sim.interfaces.v1.WheelCommand"
    assert [(field.name, field.number, field.type, field.label) for field in descriptor.fields] == [
        ("timestamp_ns", 1, FieldDescriptor.TYPE_UINT64, FieldDescriptor.LABEL_OPTIONAL),
        ("drive_wheel_speed_rad_s", 2, FieldDescriptor.TYPE_FLOAT, FieldDescriptor.LABEL_REPEATED),
        ("steering_wheel_speed_rad_s", 3, FieldDescriptor.TYPE_FLOAT, FieldDescriptor.LABEL_REPEATED),
    ]


def test_stage3_proto_declares_all_enterprise_messages():
    assert set(pb.DESCRIPTOR.message_types_by_name) == {
        "WheelCommand", "WheelState", "LidarPoint", "LidarPointCloud", "RtkState", "ImuAttitude"
    }


EXPECTED_FIELDS = {
    "WheelState": (
        ("timestamp_ns", 1, FieldDescriptor.TYPE_UINT64, FieldDescriptor.LABEL_OPTIONAL),
        ("drive_wheel_speed_rad_s", 2, FieldDescriptor.TYPE_FLOAT, FieldDescriptor.LABEL_REPEATED),
        ("steering_wheel_angle_rad", 3, FieldDescriptor.TYPE_FLOAT, FieldDescriptor.LABEL_REPEATED),
    ),
    "LidarPoint": (
        ("offset_time_ns", 1, FieldDescriptor.TYPE_UINT32, FieldDescriptor.LABEL_OPTIONAL),
        ("x", 2, FieldDescriptor.TYPE_FLOAT, FieldDescriptor.LABEL_OPTIONAL),
        ("y", 3, FieldDescriptor.TYPE_FLOAT, FieldDescriptor.LABEL_OPTIONAL),
        ("z", 4, FieldDescriptor.TYPE_FLOAT, FieldDescriptor.LABEL_OPTIONAL),
        ("reflectivity", 5, FieldDescriptor.TYPE_UINT32, FieldDescriptor.LABEL_OPTIONAL),
        ("tag", 6, FieldDescriptor.TYPE_UINT32, FieldDescriptor.LABEL_OPTIONAL),
        ("line", 7, FieldDescriptor.TYPE_UINT32, FieldDescriptor.LABEL_OPTIONAL),
    ),
    "LidarPointCloud": (
        ("timebase_ns", 1, FieldDescriptor.TYPE_UINT64, FieldDescriptor.LABEL_OPTIONAL),
        ("frame_id", 2, FieldDescriptor.TYPE_STRING, FieldDescriptor.LABEL_OPTIONAL),
        ("point_num", 3, FieldDescriptor.TYPE_UINT32, FieldDescriptor.LABEL_OPTIONAL),
        ("lidar_id", 4, FieldDescriptor.TYPE_UINT32, FieldDescriptor.LABEL_OPTIONAL),
        ("points", 5, FieldDescriptor.TYPE_MESSAGE, FieldDescriptor.LABEL_REPEATED),
    ),
    "RtkState": (
        ("timestamp_ns", 1, FieldDescriptor.TYPE_UINT64, FieldDescriptor.LABEL_OPTIONAL),
        ("main_x", 2, FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.LABEL_OPTIONAL),
        ("main_y", 3, FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.LABEL_OPTIONAL),
        ("main_z", 4, FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.LABEL_OPTIONAL),
        ("baseline_yaw_rad", 5, FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.LABEL_OPTIONAL),
    ),
    "ImuAttitude": (
        ("timestamp_ns", 1, FieldDescriptor.TYPE_UINT64, FieldDescriptor.LABEL_OPTIONAL),
        ("roll_rad", 2, FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.LABEL_OPTIONAL),
        ("pitch_rad", 3, FieldDescriptor.TYPE_DOUBLE, FieldDescriptor.LABEL_OPTIONAL),
    ),
}


@pytest.mark.parametrize("message_name,expected", EXPECTED_FIELDS.items())
def test_enterprise_descriptor_fields_are_stable(message_name, expected):
    descriptor = pb.DESCRIPTOR.message_types_by_name[message_name]
    assert tuple((field.name, field.number, field.type, field.label) for field in descriptor.fields) == expected
```

- [ ] **Step 2: 运行测试确认红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_proto_contract.py -q
```

Expected: FAIL，原因是 `google.protobuf` 或生成模块不存在。

- [ ] **Step 3: 增加依赖和唯一 `.proto` 源文件**

在 `environment.yml` 增加已验证的 `protobuf=7.35.1` 与 `grpcio-tools=1.82.1`；在 `pyproject.toml` 的 `[project]` 增加运行时依赖 `dependencies = ["protobuf>=7.35.1,<7.36"]`，并在 `[project.optional-dependencies]` 精确固定生成器 `dev = ["grpcio-tools==1.82.1"]`。生成代码会执行 Protobuf 运行时版本校验，而且生成源码可能随编译器补丁版本变化，因此验收环境精确固定实际验证版本。创建：

```proto
syntax = "proto3";
package slope_sim.interfaces.v1;

message WheelCommand {
  uint64 timestamp_ns = 1;
  repeated float drive_wheel_speed_rad_s = 2;
  repeated float steering_wheel_speed_rad_s = 3;
}
message WheelState {
  uint64 timestamp_ns = 1;
  repeated float drive_wheel_speed_rad_s = 2;
  repeated float steering_wheel_angle_rad = 3;
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
}
message RtkState {
  uint64 timestamp_ns = 1;
  double main_x = 2;
  double main_y = 3;
  double main_z = 4;
  double baseline_yaw_rad = 5;
}
message ImuAttitude {
  uint64 timestamp_ns = 1;
  double roll_rad = 2;
  double pitch_rad = 3;
}
```

- [ ] **Step 4: 创建可复现生成脚本并生成代码**

```python
# Protobuf 生成脚本：只从版本化 proto 源文件生成 Python 类型。
from pathlib import Path
from grpc_tools import protoc

ROOT = Path(__file__).resolve().parents[1]
PROTO_DIR = ROOT / "proto"
OUTPUT_DIR = ROOT / "slope_sim/interfaces/generated"


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    return protoc.main([
        "grpc_tools.protoc",
        f"-I{PROTO_DIR}",
        f"--python_out={OUTPUT_DIR}",
        str(PROTO_DIR / "slope_sim_interfaces.proto"),
    ])


if __name__ == "__main__":
    raise SystemExit(main())
```

```bash
conda env update -n slope-sim -f environment.yml
conda run -n slope-sim python scripts/generate_protos.py
```

Expected: 生成命令退出码 0。若 Conda 环境写权限需要审批，先获得用户批准，不把依赖复制进仓库冒充安装成功。

- [ ] **Step 5: 运行契约测试和生成一致性检查**

```bash
conda run -n slope-sim python -m pytest tests/test_proto_contract.py -q
conda run -n slope-sim python scripts/generate_protos.py
git diff --exit-code -- slope_sim/interfaces/generated/slope_sim_interfaces_pb2.py
```

Expected: 测试 PASS，第二次生成不产生差异。

- [ ] **Step 6: 提交**

```bash
git add environment.yml pyproject.toml proto scripts/generate_protos.py slope_sim/interfaces tests/test_proto_contract.py
git commit -m "阶段三: 1. 消息契约"
```

---

## Task 2：不可变接口模型、集中配置与机械限位

**Files:**

- Create: `slope_sim/interfaces/models.py`
- Create: `slope_sim/interfaces/config.py`
- Modify: `slope_sim/model_registry.py`
- Test: `tests/test_interface_models.py`
- Test: `tests/test_interface_config.py`

- [ ] **Step 1: 写四车型命令规则和配置失败测试**

```python
# 企业接口模型测试：命令必须按当前车型整条校验。
import math
import pytest
from slope_sim.interfaces.models import WheelCommand, validate_wheel_command
from slope_sim.model_registry import get_robot_model


def test_differential_command_requires_two_drive_values_and_no_steering():
    command = WheelCommand(10, (1.0, 2.0), ())
    assert validate_wheel_command(command, get_robot_model("df_back")) == command
    with pytest.raises(ValueError, match="2 drive"):
        validate_wheel_command(WheelCommand(10, (1.0,), ()), get_robot_model("df_back"))


def test_active_steering_command_requires_four_drive_and_two_steering_values():
    model = get_robot_model("active_steering_4wd")
    command = WheelCommand(10, (1.0, 2.0, 3.0, 4.0), (0.5, -0.5))
    assert validate_wheel_command(command, model) == command
    with pytest.raises(ValueError, match="4 drive"):
        validate_wheel_command(WheelCommand(10, (1.0, 2.0), (0.5, -0.5)), model)
    with pytest.raises(ValueError, match="2 steering"):
        validate_wheel_command(WheelCommand(10, (1.0, 2.0, 3.0, 4.0), (0.5,)), model)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, 20.01])
def test_wheel_command_rejects_non_finite_or_out_of_limit_drive(value):
    with pytest.raises(ValueError):
        validate_wheel_command(WheelCommand(10, (value, 0.0), ()), get_robot_model("df_mid"))


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, 2.01, -2.01])
def test_active_steering_rejects_non_finite_or_out_of_limit_rate(value):
    with pytest.raises(ValueError):
        validate_wheel_command(
            WheelCommand(10, (0.0, 0.0, 0.0, 0.0), (value, 0.0)),
            get_robot_model("active_steering_4wd"),
        )
```

```python
from slope_sim.interfaces.config import InterfaceConfig


def test_interface_topics_are_centralized_and_unique():
    config = InterfaceConfig.default()
    assert config.wheel_command.topic == "/sim/wheel/command"
    assert config.wheel_state.rate_hz == 100
    assert len({channel.topic for channel in config.channels}) == 6


@pytest.mark.parametrize(
    "change,match",
    (
        ({"transport_mode": "fake"}, "transport_mode"),
        ({"command_timeout_sec": 0.0}, "command_timeout_sec"),
        ({"status_window_sec": math.nan}, "status_window_sec"),
        ({"outgoing_queue_size": 0}, "outgoing_queue_size"),
        ({"log_queue_size": -1}, "log_queue_size"),
    ),
)
def test_interface_config_rejects_each_invalid_scalar(change, match):
    with pytest.raises(ValueError, match=match):
        dataclasses.replace(InterfaceConfig.default(), **change)


def test_interface_config_rejects_empty_duplicate_and_nonpositive_channel():
    config = InterfaceConfig.default()
    with pytest.raises(ValueError, match="topic"):
        dataclasses.replace(config, imu=dataclasses.replace(config.imu, topic=""))
    with pytest.raises(ValueError, match="duplicate"):
        dataclasses.replace(config, imu=dataclasses.replace(config.imu, topic=config.rtk.topic))
    with pytest.raises(ValueError, match="rate_hz"):
        dataclasses.replace(config, imu=dataclasses.replace(config.imu, rate_hz=0))
```

- [ ] **Step 2: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_models.py tests/test_interface_config.py -q
```

Expected: FAIL，接口模型和配置模块不存在。

- [ ] **Step 3: 实现消息对象与原子校验**

`models.py` 用 frozen dataclass 定义 `WheelCommand`、`WheelState`、`LidarPoint`、`LidarPointCloud`、`RtkState`、`ImuAttitude`。轮子校验入口固定为：

```python
def validate_wheel_command(command: WheelCommand, model: RobotModelSpec) -> WheelCommand:
    expected_drive = 4 if model.controller_kind == "active_steering" else 2
    expected_steering = 2 if model.controller_kind == "active_steering" else 0
    if len(command.drive_wheel_speed_rad_s) != expected_drive:
        raise ValueError(f"{model.name} requires {expected_drive} drive wheel speeds")
    if len(command.steering_wheel_speed_rad_s) != expected_steering:
        raise ValueError(f"{model.name} requires {expected_steering} steering wheel speeds")
    _require_bounded("drive", command.drive_wheel_speed_rad_s, model.max_drive_wheel_speed_rad_s)
    _require_bounded("steering", command.steering_wheel_speed_rad_s, model.max_steering_speed_rad_s)
    return command
```

给 `RobotModelSpec` 增加：

```python
max_drive_wheel_speed_rad_s: float = 20.0
max_steering_speed_rad_s: float = 2.0
```

- [ ] **Step 4: 实现集中配置**

```python
@dataclass(frozen=True)
class ChannelConfig:
    topic: str
    rate_hz: int
    direction: str


@dataclass(frozen=True)
class InterfaceConfig:
    transport_mode: str
    wheel_command: ChannelConfig
    wheel_state: ChannelConfig
    lidar_front: ChannelConfig
    lidar_rear: ChannelConfig
    rtk: ChannelConfig
    imu: ChannelConfig
    command_timeout_sec: float = 0.100
    status_window_sec: float = 2.0
    outgoing_queue_size: int = 32
    log_queue_size: int = 256

    @classmethod
    def default(cls, *, transport_mode: str = "auto") -> "InterfaceConfig":
        return cls(
            transport_mode=transport_mode,
            wheel_command=ChannelConfig("/sim/wheel/command", 100, "subscribe"),
            wheel_state=ChannelConfig("/sim/wheel/state", 100, "publish"),
            lidar_front=ChannelConfig("/sim/lidar/front/points", 10, "publish"),
            lidar_rear=ChannelConfig("/sim/lidar/rear/points", 10, "publish"),
            rtk=ChannelConfig("/sim/rtk/state", 10, "publish"),
            imu=ChannelConfig("/sim/imu/attitude", 10, "publish"),
        )
```

`__post_init__` 拒绝未知模式、空话题、重复话题、非正频率和无效队列容量。`default()` 返回设计文档中的六个话题。

- [ ] **Step 5: 运行测试并提交**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_models.py tests/test_interface_config.py tests/test_robot_models.py -q
git add slope_sim/interfaces/models.py slope_sim/interfaces/config.py slope_sim/model_registry.py tests/test_interface_models.py tests/test_interface_config.py tests/test_robot_models.py
git commit -m "阶段三: 1. 接口模型"
```

---

## Task 3：Protobuf 编解码边界

**Files:**

- Create: `slope_sim/interfaces/codec.py`
- Test: `tests/test_interface_codec.py`

- [ ] **Step 1: 写所有企业消息 round-trip 失败测试**

```python
# Protobuf 编解码测试：企业对象往返后语义不变。
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.models import WheelCommand, LidarPoint, LidarPointCloud


def test_codec_round_trips_wheel_command():
    codec = ProtoCodec()
    source = WheelCommand(123, (1.0, -2.0), ())
    assert codec.decode_wheel_command(codec.encode(source)) == source


def test_codec_round_trips_point_cloud_and_checks_point_count():
    codec = ProtoCodec()
    point = LidarPoint(0, 1.0, 2.0, 3.0, 160, 2, 0)
    source = LidarPointCloud(1000, "lidar_front", 1, 1, (point,))
    assert codec.decode_lidar_point_cloud(codec.encode(source)) == source


@pytest.mark.parametrize(
    "source,decoder",
    (
        (WheelState(100, (1.0, -2.0), ()), "decode_wheel_state"),
        (RtkState(200, 1.5, -2.5, 0.5, -0.25), "decode_rtk_state"),
        (ImuAttitude(300, 0.25, -0.5), "decode_imu_attitude"),
    ),
)
def test_codec_round_trips_each_remaining_enterprise_message(source, decoder):
    codec = ProtoCodec()
    assert getattr(codec, decoder)(codec.encode(source)) == source


def test_codec_rejects_malformed_wire_payload():
    with pytest.raises(ValueError, match="WheelCommand"):
        ProtoCodec().decode_wheel_command(b"\xff")


def test_codec_rejects_point_count_mismatch():
    message = pb.LidarPointCloud(timebase_ns=1, frame_id="lidar_front", point_num=2, lidar_id=1)
    message.points.add(x=1.0, line=0)
    with pytest.raises(ValueError, match="point_num"):
        ProtoCodec().decode_lidar_point_cloud(message.SerializeToString())
```

- [ ] **Step 2: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_codec.py -q
```

- [ ] **Step 3: 实现显式转换表**

`ProtoCodec` 不使用反射猜字段，显式实现 `encode()` 与每种 `decode_*()`：

```python
ENCODERS = {
    WheelCommand: _wheel_command_to_proto,
    WheelState: _wheel_state_to_proto,
    LidarPointCloud: _lidar_cloud_to_proto,
    RtkState: _rtk_to_proto,
    ImuAttitude: _imu_to_proto,
}


def encode(self, message: EnterpriseMessage) -> bytes:
    try:
        proto = self.ENCODERS[type(message)](message)
    except KeyError as exc:
        raise TypeError(f"unsupported enterprise message: {type(message).__name__}") from exc
    return proto.SerializeToString()
```

解码捕获 `DecodeError` 并转成包含消息类型的 `ValueError`；点云验证 `point_num == len(points)`，所有数值重新通过模型不变量。

- [ ] **Step 4: 运行测试并提交**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_codec.py tests/test_proto_contract.py -q
git add slope_sim/interfaces/codec.py tests/test_interface_codec.py
git commit -m "阶段三: 1. 消息编解码"
```

---

## Task 4：精确仿真时钟、周期调度与状态统计

**Files:**

- Create: `slope_sim/interfaces/clock.py`
- Create: `slope_sim/interfaces/status.py`
- Test: `tests/test_interface_clock.py`
- Test: `tests/test_interface_status.py`

- [ ] **Step 1: 写 240 Hz 对 100/10 Hz 的长期测试及追赶安全边界失败测试**

```python
# 仿真时钟测试：非整数步频率不能用固定步取模。
from slope_sim.interfaces.clock import (
    MAX_CATCH_UP_DEADLINES,
    PeriodicScheduler,
    SimulationClock,
)


def test_fractional_scheduler_emits_exact_counts_for_ten_seconds():
    clock = SimulationClock()
    wheel = PeriodicScheduler(100)
    sensor = PeriodicScheduler(10)
    wheel_stamps, sensor_stamps = [], []
    for _ in range(2400):
        now = clock.advance(1.0 / 240.0)
        wheel_stamps.extend(wheel.pop_due(now))
        sensor_stamps.extend(sensor.pop_due(now))
    assert len(wheel_stamps) == 1000
    assert len(sensor_stamps) == 100
    assert wheel_stamps[-1] == 10_000_000_000
    assert sensor_stamps[-1] == 10_000_000_000
```

```python
from slope_sim.interfaces.status import RollingFrequency


def test_frequency_uses_two_second_monotonic_window():
    frequency = RollingFrequency(window_sec=2.0)
    for index in range(201):
        frequency.record(index * 0.01)
    assert frequency.hz(now=2.0) == 100.0


def test_scheduler_catches_up_all_deadlines_without_drift():
    scheduler = PeriodicScheduler(100)
    assert scheduler.pop_due(35_000_000) == (10_000_000, 20_000_000, 30_000_000)
    assert scheduler.pop_due(39_999_999) == ()
    assert scheduler.pop_due(40_000_000) == (40_000_000,)


def test_scheduler_returns_all_normal_catch_up_deadlines_through_limit():
    assert PeriodicScheduler(1_000_000_000).pop_due(257) == tuple(range(1, 258))
    assert MAX_CATCH_UP_DEADLINES == 10_000
    assert PeriodicScheduler(1_000_000_000).pop_due(10_000) == tuple(range(1, 10_001))


def test_scheduler_rejects_abnormal_jump_atomically_before_allocation():
    scheduler = PeriodicScheduler(100)
    assert scheduler.pop_due(10_000_000) == (10_000_000,)
    with pytest.raises(ValueError, match="catch-up limit"):
        scheduler.pop_due(100_020_000_000)
    assert scheduler.pop_due(20_000_000) == (20_000_000,)


@pytest.mark.parametrize("dt", (0.0, -0.01, math.nan, math.inf))
def test_simulation_clock_rejects_nonpositive_or_nonfinite_dt(dt):
    with pytest.raises(ValueError, match="dt"):
        SimulationClock().advance(dt)


def test_frequency_evicts_events_older_than_window_and_all_states_are_validated():
    frequency = RollingFrequency(window_sec=2.0)
    frequency.record(0.0)
    frequency.record(1.0)
    frequency.record(2.5)
    assert frequency.hz(now=2.5) == pytest.approx(1.0 / 1.5)
    with pytest.raises(ValueError, match="state"):
        TopicStatus("/topic", "unknown", 0.0, None, 0, 0, 0)
```

- [ ] **Step 2: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_clock.py tests/test_interface_status.py -q
```

- [ ] **Step 3: 使用有理数累计实现时钟和调度**

```python
class SimulationClock:
    def __init__(self) -> None:
        self._seconds = Fraction(0, 1)

    @property
    def now_ns(self) -> int:
        return round(self._seconds * 1_000_000_000)

    def advance(self, dt: float) -> int:
        step = Fraction(dt).limit_denominator(1_000_000_000)
        if step <= 0:
            raise ValueError("dt must be positive")
        self._seconds += step
        return self.now_ns
```

`PeriodicScheduler` 用 `Fraction(1, rate_hz)` 累加下一期限，`pop_due()` 在正常追赶中返回所有已跨越期限。单次安全上限 `MAX_CATCH_UP_DEADLINES = 10_000`，覆盖当前最高配置 100 Hz 连续 100 秒；超过上限属于异常跳变，必须预计算数量并在结果分配、期限推进和上次调用时间修改前原子拒绝，不支持任意 uint64 跳变的无界返回。

每条纳秒时间戳按 `round(deadline_index * 1_000_000_000 / rate_hz)` 生成，但实现必须使用整数商余数并在恰好一半时按商的奇偶执行 ties-to-even，精确匹配正 `Fraction`，不得转为浮点数或使用普通 `+0.5`。正常 240 Hz 物理循环每步最多返回一个 100 Hz 期限；暂停时调用方不执行 `advance()`，不积累补发期限。测试覆盖 257 条、10,000 条边界、10,001 条原子拒绝、3 Hz 非整除频率和半纳秒 ties-to-even。

- [ ] **Step 4: 实现不可变状态快照**

定义 `TopicStatus`、`WheelCommandStatus`、`InterfaceStatusSnapshot` 和 `RollingFrequency`。`InterfaceStatusSnapshot` 除连接与逐话题统计外必须携带 `wheel_state: WheelState | None`，使 Dashboard 能从同一不可变快照读取最新实际轮速/转角，而不跨线程访问机器人。状态只允许 `active`、`waiting_peer`、`timed_out`、`degraded`、`disconnected`、`error`；滚动窗口按注入的单调墙钟事件统计。

- [ ] **Step 5: 运行测试并提交**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_clock.py tests/test_interface_status.py -q
git add slope_sim/interfaces/clock.py slope_sim/interfaces/status.py tests/test_interface_clock.py tests/test_interface_status.py
git commit -m "阶段三: 1. 仿真时钟"
```

---

## Task 5：本地传输、轮子 mailbox 与 100 ms 超时

**Files:**

- Create: `slope_sim/interfaces/transport.py`
- Create: `slope_sim/interfaces/wheel.py`
- Test: `tests/test_local_transport.py`
- Test: `tests/test_wheel_mailbox.py`

- [ ] **Step 1: 写本地发布订阅和关闭语义失败测试**

```python
# 本地传输测试：使用真实字节负载固定订阅、计数和幂等关闭语义。
import pytest
from slope_sim.interfaces.transport import LocalTransport


def test_local_transport_delivers_bytes_and_counts_each_message():
    transport = LocalTransport()
    received = []
    subscription = transport.subscribe(
        "/sim/wheel/command",
        "slope_sim.interfaces.v1.WheelCommand",
        lambda payload, received_at: received.append((payload, received_at)),
    )
    assert transport.publish(
        "/sim/wheel/command",
        b"command",
        "slope_sim.interfaces.v1.WheelCommand",
        sim_time_ns=10,
        wall_time=2.5,
    )
    assert received == [(b"command", 2.5)]
    assert transport.snapshot().published_count == 1
    assert transport.snapshot().received_count == 1
    subscription.close()
    transport.close()
    transport.close()
    with pytest.raises(RuntimeError, match="closed"):
        transport.publish("/sim/wheel/command", b"late", "type", 20, wall_time=3.0)
```

- [ ] **Step 2: 写有效、非法、清空和超时失败测试**

```python
# 轮子 mailbox 测试：非法消息不能刷新最后有效命令时刻。
from slope_sim.interfaces.models import WheelCommand
from slope_sim.interfaces.wheel import WheelCommandMailbox
from slope_sim.model_registry import get_robot_model


def test_invalid_command_does_not_refresh_timeout_or_replace_valid_command():
    mailbox = WheelCommandMailbox(get_robot_model("df_back"), timeout_sec=0.100)
    mailbox.accept(WheelCommand(1, (2.0, 2.0), ()), received_at=10.0)
    assert not mailbox.accept(WheelCommand(2, (30.0, 30.0), ()), received_at=10.08)
    decision = mailbox.decision(now=10.101)
    assert decision.timed_out
    assert decision.drive_wheel_speed_rad_s == (0.0, 0.0)
    assert decision.steering_wheel_speed_rad_s == ()
    assert mailbox.snapshot().valid_count == 1
    assert mailbox.snapshot().invalid_count == 1


def test_clear_requires_a_new_command_for_the_current_model():
    mailbox = WheelCommandMailbox(get_robot_model("active_steering_4wd"))
    mailbox.accept(WheelCommand(1, (1.0, 1.0, 1.0, 1.0), (0.2, -0.2)), received_at=5.0)
    mailbox.clear()
    decision = mailbox.decision(now=5.01)
    assert decision.waiting
    assert decision.drive_wheel_speed_rad_s == (0.0, 0.0, 0.0, 0.0)
    assert decision.steering_wheel_speed_rad_s == (0.0, 0.0)


def test_latest_valid_command_wins_without_losing_receive_count():
    mailbox = WheelCommandMailbox(get_robot_model("df_back"))
    mailbox.accept(WheelCommand(1, (1.0, 1.0), ()), received_at=1.0)
    mailbox.accept(WheelCommand(2, (2.0, 3.0), ()), received_at=1.01)
    assert mailbox.decision(now=1.02).drive_wheel_speed_rad_s == (2.0, 3.0)
    assert mailbox.snapshot().valid_count == 2


@pytest.mark.parametrize(
    "age,timed_out",
    ((0.099999, False), (0.100000, True), (0.100001, True)),
)
def test_timeout_starts_at_exactly_one_hundred_milliseconds(age, timed_out):
    mailbox = WheelCommandMailbox(get_robot_model("df_back"))
    mailbox.accept(WheelCommand(1, (1.0, 1.0), ()), received_at=10.0)
    assert mailbox.decision(now=10.0 + age).timed_out is timed_out


def test_local_transport_rejects_conflicting_type_for_one_topic():
    transport = LocalTransport()
    transport.subscribe("/topic", "TypeA", lambda *_args: None)
    with pytest.raises(ValueError, match="type"):
        transport.subscribe("/topic", "TypeB", lambda *_args: None)
```

- [ ] **Step 3: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_local_transport.py tests/test_wheel_mailbox.py -q
```

Expected: FAIL，原因是 `transport.py` 和 `wheel.py` 尚不存在。

- [ ] **Step 4: 实现窄传输协议和同步本地实现**

```python
# 企业传输边界：运行时只依赖该协议，不直接依赖 eCAL 回调对象。
class Transport(Protocol):
    def subscribe(self, topic: str, type_name: str, callback: MessageCallback) -> Subscription:
        raise NotImplementedError

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float | None = None,
    ) -> bool:
        raise NotImplementedError

    def snapshot(self) -> TransportSnapshot:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


class LocalTransport:
    """进程内确定性传输；它是测试模式，状态中永远不能声称 eCAL 已连接。"""

    def subscribe(self, topic: str, type_name: str, callback: MessageCallback) -> LocalSubscription:
        self._ensure_open()
        subscription = LocalSubscription(self, topic, type_name, callback)
        self._subscriptions.setdefault(topic, []).append(subscription)
        return subscription

    def publish(self, topic, payload, type_name, sim_time_ns, *, wall_time=None) -> bool:
        self._ensure_open()
        received_at = self._monotonic() if wall_time is None else float(wall_time)
        self._published_count += 1
        for subscription in tuple(self._subscriptions.get(topic, ())):
            if subscription.type_name != type_name:
                continue
            subscription.deliver(bytes(payload), received_at)
            self._received_count += 1
        return True
```

`LocalTransport(monotonic=...)` 注入单调墙钟。实现用一个 `Condition` 统一保护 open/closing/closed、订阅 active、每订阅与全局 in-flight 及累计计数；在锁内检查生命周期并登记 in-flight，在锁外执行用户 callback，`finally` 回锁结束并通知。callback 内关闭订阅或 transport 只原子禁止新交付并返回，外部关闭等待已启动 callback 完成；多个外部 `close()` 等待同一屏障。全局关闭开始后，旧 publish 快照也不能启动新 callback。`LocalSubscription.close()` 和 `LocalTransport.close()` 都幂等；`TransportSnapshot` 是 frozen dataclass，含 `mode`、`ecal_connected`、发布/接收/错误/丢帧累计计数。

- [ ] **Step 5: 实现 mailbox 原子校验和墙钟超时**

```python
@dataclass(frozen=True)
class WheelDecision:
    drive_wheel_speed_rad_s: tuple[float, ...]
    steering_wheel_speed_rad_s: tuple[float, ...]
    waiting: bool = False
    timed_out: bool = False


class WheelCommandMailbox:
    def capture_generation(self) -> int:
        with self._lock:
            return self._generation

    def accept(
        self,
        command: WheelCommand,
        *,
        received_at: float,
        generation: int | None = None,
    ) -> bool:
        received = _require_finite_wall_time(received_at)
        accepted_generation = self.capture_generation() if generation is None else generation
        try:
            validated = validate_wheel_command(command, self._model)
        except ValueError as exc:
            self._invalid_count += 1
            self._last_error = str(exc)
            return False
        self._latest = validated
        self._last_valid_received_at = received
        self._valid_count += 1
        self._last_error = None
        return True

    def decision(self, *, now: float) -> WheelDecision:
        if self._latest is None or self._last_valid_received_at is None:
            return self._zero_decision(waiting=True)
        if _require_finite_wall_time(now) - self._last_valid_received_at >= self._timeout_sec:
            return self._zero_decision(timed_out=True)
        return WheelDecision(
            self._latest.drive_wheel_speed_rad_s,
            self._latest.steering_wheel_speed_rad_s,
        )
```

`__init__(model, timeout_sec=0.100)` 校验正有限超时。回调在解码或等待前调用 `capture_generation()`；`accept(..., generation=token)` 在校验前后都核对 token。`clear()` 原子递增 generation，删除命令和有效接收时刻但保留累计计数；旧 token 返回 `False` 且不修改任何状态。同步本地调用可省略 token，并在进入 `accept()` 时捕获当前 generation。accept 事件时间只约束其他 accept 的顺序，主线程查询只约束其他查询的顺序，非法事件或稍晚提交的有效事件不能让已经捕获的物理查询抛异常。频率查询 horizon 不早于最后有效事件，但安全年龄仍使用调用方 `now`。超时严格按 `now - received_at >= 0.100` 判定，发送方 `timestamp_ns` 不参与安全判断；达到 100 ms 的浮点差值边界已经失效。

- [ ] **Step 6: 运行测试并提交**

```bash
conda run -n slope-sim python -m pytest tests/test_local_transport.py tests/test_wheel_mailbox.py tests/test_interface_models.py -q
git add slope_sim/interfaces/transport.py slope_sim/interfaces/wheel.py tests/test_local_transport.py tests/test_wheel_mailbox.py
git commit -m "阶段三: 1. 本地轮控"
```

---

## Task 6：本地 `InterfaceRuntime` 与 DIRECT 轮子闭环门禁

**Files:**

- Create: `slope_sim/interfaces/runtime.py`
- Modify: `slope_sim/robot.py`
- Test: `tests/test_interface_runtime.py`
- Test: `tests/test_interface_wheel_direct.py`

- [ ] **Step 1: 写实际状态而非命令回显的失败测试**

```python
# 接口运行时测试：轮子状态必须在物理步进后读取真实关节值。
from slope_sim.interfaces.models import WheelCommand
from slope_sim.interfaces.runtime import InterfaceRuntime


def test_runtime_applies_latest_command_and_publishes_actual_state(fake_robot, fake_monotonic):
    runtime = InterfaceRuntime.local_for_robot(fake_robot, monotonic=fake_monotonic)
    try:
        runtime.accept_local_command(WheelCommand(0, (3.0, -2.0), ()), received_at=1.0)
        for step in range(3):
            runtime.before_physics_step(1.0 / 240.0, wall_time=1.001 + step / 240.0)
            fake_robot.actual_drive_speeds = (2.7, -1.8)
            runtime.after_physics_step(1.0 / 240.0)
        assert fake_robot.command_calls[-1] == ((3.0, -2.0), (), 1.0 / 240.0)
        state = runtime.last_wheel_state
        assert state.drive_wheel_speed_rad_s == (2.7, -1.8)
        assert state.timestamp_ns == 10_000_000
    finally:
        runtime.close()


def test_paused_runtime_does_not_control_advance_or_publish(fake_robot, fake_monotonic):
    runtime = InterfaceRuntime.local_for_robot(fake_robot, monotonic=fake_monotonic)
    try:
        runtime.pause()
        before_ns = runtime.clock.now_ns
        assert runtime.before_physics_step(1.0 / 240.0, wall_time=1.0) is None
        assert runtime.after_physics_step(1.0 / 240.0) == ()
        assert runtime.clock.now_ns == before_ns
        assert fake_robot.command_calls == []
    finally:
        runtime.close()


def test_close_rejects_commands_and_concurrent_close_waits_for_one_cleanup(runtime_fixture):
    runtime, _fake_monotonic, fake_robot, fake_transport = runtime_fixture
    first_close, second_close = start_two_closes_while_fake_transport_is_blocked(
        runtime, fake_transport
    )
    assert not first_close.returned.is_set()
    assert not second_close.returned.is_set()
    with pytest.raises(RuntimeError, match="closed"):
        runtime.accept_local_command(valid_command(runtime.robot_model), received_at=2.0)
    fake_transport.release_close.set()
    first_close.join()
    second_close.join()
    assert fake_transport.close_count == 1
    assert fake_robot.safe_stop_count == 1


def test_wheel_topic_status_tracks_publish_failure_and_recovery(runtime_fixture):
    runtime, fake_monotonic, _fake_robot, fake_transport = runtime_fixture
    fake_transport.publish_outcomes.extend((False, True))
    first = run_one_wheel_deadline(runtime, fake_monotonic)
    failed = runtime.status_snapshot(wall_time=fake_monotonic())
    assert failed.wheel_state == first
    assert failed.topics[runtime.config.wheel_state.topic].state == "error"
    assert failed.topics[runtime.config.wheel_state.topic].message_count == 0
    run_one_wheel_deadline(runtime, fake_monotonic)
    recovered = runtime.status_snapshot(wall_time=fake_monotonic())
    topic = recovered.topics[runtime.config.wheel_state.topic]
    assert topic.state == "active"
    assert topic.direction == "publish"
    assert topic.message_count == 1
    assert topic.error_count == 1
    assert topic.dropped_count == 0
    assert topic.detail == ""


def test_same_model_rebind_rejects_delayed_old_mailbox_callback(runtime_fixture, replacement_robot):
    runtime, fake_monotonic, _fake_robot, _fake_transport = runtime_fixture
    old_mailbox, old_generation = runtime.capture_command_ingress()
    runtime.rebind_robot(replacement_robot)
    accepted = old_mailbox.accept(
        valid_command(replacement_robot.model_spec.name),
        received_at=fake_monotonic(),
        generation=old_generation,
    )
    assert not accepted
    status = runtime.status_snapshot(wall_time=fake_monotonic()).command
    assert status.state == "waiting_command"
    assert status.valid_count == status.invalid_count == 0
```

- [ ] **Step 2: 写四车型 DIRECT 运动与主动转向超时门禁**

```python
# DIRECT 轮子闭环：验证数组顺序、实际反馈、转向限位和超时停车。
@pytest.mark.parametrize("robot_model", ("df_front", "df_mid", "df_back"))
def test_differential_interface_command_moves_forward_and_reports_two_wheels(robot_model):
    result = run_direct_wheel_gate(robot_model, drive=(6.0, 6.0), duration_sec=0.8)
    assert result.forward_displacement_m > 0.10
    assert len(result.last_state_while_sending.drive_wheel_speed_rad_s) == 2
    assert result.last_state_while_sending.steering_wheel_angle_rad == ()
    assert result.command_send_count == 80
    assert not result.timed_out_while_sending
    assert 0.100 <= result.first_timeout_time - result.last_send_time <= 0.100 + 1.0 / 240.0


def test_active_steering_integrates_to_limit_and_timeout_holds_angle():
    result = run_direct_active_steering_timeout_gate(
        drive=(5.0, 5.0, 5.0, 5.0),
        steering_rate=(2.0, 2.0),
        command_duration_sec=0.35,
        silence_sec=0.15,
    )
    assert all(abs(angle) <= 0.55 + 1e-6 for angle in result.angle_before_timeout)
    assert max(abs(target - actual) for target, actual in zip(
        result.target_before_timeout, result.angle_before_timeout
    )) > 0.10
    assert result.drive_targets_after_timeout == (0.0, 0.0, 0.0, 0.0)
    assert result.angle_after_timeout == pytest.approx(result.angle_before_timeout, abs=0.03)
    assert max(abs(angle - target) for angle, target in zip(
        result.angle_after_timeout, result.target_before_timeout
    )) > 0.07


def test_active_steering_state_preserves_registered_array_order():
    result = run_direct_wheel_gate(
        "active_steering_4wd",
        drive=(3.0, 4.0, 5.0, 6.0),
        steering=(0.5, -0.5),
        duration_sec=0.08,
    )
    assert result.last_state_while_sending.drive_wheel_speed_rad_s == pytest.approx(
        (3.0, 4.0, 5.0, 6.0), abs=0.3
    )
    assert len(result.last_state_while_sending.steering_wheel_angle_rad) == 2
```

- [ ] **Step 3: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_runtime.py tests/test_interface_wheel_direct.py -q
```

Expected: FAIL，原因是 `InterfaceRuntime` 和 DIRECT gate helper 尚不存在。

- [ ] **Step 4: 给机器人补稳定轮子端口并实现运行时**

`robot.py` 只增加现有 PyBullet 能力的窄封装，不改变 `command_twist()` 兼容行为：

```python
def read_interface_wheel_state(self, timestamp_ns: int) -> WheelState:
    """按车型注册表顺序读取真实关节速度和转向角。"""
    return WheelState(
        timestamp_ns=timestamp_ns,
        drive_wheel_speed_rad_s=self.read_drive_wheel_speeds(),
        steering_wheel_angle_rad=self.read_steering_wheel_angles(),
    )


def hold_current_steering_and_stop_drive(self, dt: float) -> None:
    """等待或超时时把驱动归零；差速车型没有独立转向角。"""
    drive = (0.0,) * len(self.model_spec.drive_joint_names)
    steering = (0.0,) * len(self.model_spec.steering_joint_names)
    self.command_wheel_speeds(drive, steering, dt=dt)
```

`ActiveSteeringRobot` 覆盖安全停车端口：先读取两个真实转向关节角，要求有限并夹到 `[-max_steering_angle, max_steering_angle]`，把结果写入 `_steering_targets`，再以四轮零驱动和两个零转向速度下发。不能只对旧 `_steering_targets` 发送零转向速度，否则超时瞬间实际角尚未追上目标时会继续运动，而不是保持当前角。

`InterfaceRuntime` 第一版固定生命周期：

```python
class InterfaceRuntime:
    @classmethod
    def local_for_robot(cls, robot: WheelRobotPort, *, config=None, monotonic=time.monotonic):
        selected = config or InterfaceConfig.default(transport_mode="local")
        if selected.transport_mode != "local":
            raise ValueError("local_for_robot requires transport_mode='local'")
        return cls(robot, selected, LocalTransport(monotonic=monotonic), monotonic=monotonic)

    def accept_local_command(self, command: WheelCommand, *, received_at: float | None = None) -> bool:
        with self._lifecycle_condition:
            if not self._accepting_commands or self._lifecycle_state != "open":
                raise RuntimeError("interface runtime is closed")
            received = self._monotonic() if received_at is None else received_at
            return self._mailbox.accept(command, received_at=received)

    def capture_command_ingress(self) -> tuple[WheelCommandMailbox, int]:
        with self._lifecycle_condition:
            if not self._accepting_commands or self._lifecycle_state != "open":
                raise RuntimeError("interface runtime is closed")
            mailbox = self._mailbox
            return mailbox, mailbox.capture_generation()

    def before_physics_step(self, dt: float, *, wall_time: float | None = None) -> None:
        if self._paused:
            return None
        now = self._monotonic() if wall_time is None else wall_time
        self._last_decision = self._mailbox.decision(now=now)
        if self._last_decision.waiting or self._last_decision.timed_out:
            self._robot.hold_current_steering_and_stop_drive(dt)
        else:
            self._robot.command_wheel_speeds(
                self._last_decision.drive_wheel_speed_rad_s,
                self._last_decision.steering_wheel_speed_rad_s,
                dt=dt,
            )

    def after_physics_step(self, dt: float) -> tuple[WheelState, ...]:
        if self._paused:
            return ()
        now_ns = self._clock.advance(dt)
        states = tuple(
            self._robot.read_interface_wheel_state(stamp)
            for stamp in self._wheel_scheduler.pop_due(now_ns)
        )
        self._publish_wheel_states(states)
        return states

    def pause(self) -> None:
        self._paused = True

    def resume(self, *, wall_time: float | None = None) -> None:
        decision = self._mailbox.decision(
            now=self._monotonic() if wall_time is None else wall_time
        )
        self._last_decision = decision
        self._paused = False

    def rebind_robot(self, robot: WheelRobotPort) -> None:
        new_mailbox = WheelCommandMailbox(robot.model_spec, self._config.command_timeout_sec)
        new_decision = new_mailbox.decision(now=self._monotonic())
        with self._lifecycle_condition:
            self._require_open()
            self._mailbox.clear()
            self._robot.hold_current_steering_and_stop_drive(1.0 / 240.0)
            self._robot = robot
            self._mailbox = new_mailbox
            self._last_decision = new_decision
            self._last_wheel_state = None

    def status_snapshot(self, *, wall_time: float | None = None) -> InterfaceStatusSnapshot:
        now = self._monotonic() if wall_time is None else wall_time
        transport = self._transport.snapshot()
        return InterfaceStatusSnapshot(
            captured_at=now,
            transport_mode=transport.mode,
            ecal_connected=transport.ecal_connected,
            command=self._mailbox.snapshot(now=now),
            wheel_state=self._last_wheel_state,
            topics=self._topic_status_snapshots(now),
        )

    def close(self) -> None:
        with self._lifecycle_condition:
            if self._lifecycle_state == "closed":
                return
            if self._lifecycle_state == "closing":
                self._lifecycle_condition.wait_for(
                    lambda: self._lifecycle_state == "closed"
                )
                return
            self._lifecycle_state = "closing"
            self._accepting_commands = False
            self._mailbox.clear()
        try:
            self._robot.hold_current_steering_and_stop_drive(1.0 / 240.0)
        finally:
            try:
                self._transport.close()
            finally:
                with self._lifecycle_condition:
                    self._lifecycle_state = "closed"
                    self._lifecycle_condition.notify_all()
```

`WheelRobotPort` 固定暴露 `model_spec`、`command_wheel_speeds(...)`、`read_interface_wheel_state(timestamp_ns)` 和 `hold_current_steering_and_stop_drive(dt)`。`InterfaceRuntime` 用同一个 lifecycle `Condition` 线性化命令接收、重绑和关闭；初始化 `last_wheel_state: WheelState | None = None`、等待态 `last_decision`、`clock` 只读属性、100 Hz wheel scheduler、逐话题频率/计数和 `open/closing/closed` 状态。关闭后命令入口抛 `RuntimeError`。异步回调必须在 lifecycle 锁内通过 `capture_command_ingress()` 原子取得 `(mailbox_ref, generation)`，解码后向同一个 `mailbox_ref` 提交 token；不能只保存整数后再动态读取 `self._mailbox`。重绑先在锁外完整构造新 mailbox 与等待 decision，再在锁内对旧 mailbox 调用 `clear()`、安全停止旧车并一次提交所有新引用；构造失败不产生混合状态，迟到回调仍指向已递增代际的旧 mailbox。同理，关闭在宣布 `closing` 时清空当前 mailbox。

关闭随后释放 lifecycle 锁再执行安全停车和 transport 的 in-flight 屏障，避免回调等待运行时锁时形成反向死锁；并发 `close()` 等待同一清理结果。安全停车抛异常时也必须在 `finally` 中关闭 transport，并最终回锁标记 `closed`、唤醒等待者。Task 6 fixture 显式返回 fake robot 和 fake transport；关闭次数、阻塞和 publish 结果只在 fake 上观察，不给生产 Runtime 增加测试专用字段。

每次读到真实 `WheelState` 都更新 `last_wheel_state`，随后 `_publish_wheel_states()` 用现有 codec 编码并提交 `config.wheel_state` 话题。Task 6 只创建 wheel-state 一个 `TopicStatus`：`direction="publish"`、`target_hz=config.wheel_state.rate_hz`、本地 `dropped_count=0`。只有 transport 返回成功才更新 `message_count`、实际频率和 `latest_timestamp_ns`；读取失败、publish 返回 `False` 或抛异常都累计 `error_count`、置 `state="error"` 并填写活动 `detail`。后续成功恢复 `state="active"` 和空 `detail`，但保留累计错误；发布失败不能丢失已经读取的 `last_wheel_state`。Task 12 接入其余传感器时把映射扩展为六个话题。`status_snapshot()` 直接从 transport、mailbox、话题统计和最后实际轮子状态构造不可变快照，不引用未定义的 `_status` 聚合器。

`before_physics_step()` 只消费 mailbox 并控制车辆；等待或超时时调用安全停车端口。`after_physics_step(dt)` 必须在调用方完成 `p.stepSimulation()` 后执行，才推进 `SimulationClock`、按 100 Hz deadlines 读取并发布 `WheelState`。暂停后两个步进入口都 no-op，既不控制也不推进或发布；`resume()` 先检查墙钟超时。`rebind_robot()` 先安全停止旧车，再替换机器人和新 mailbox，并把 decision 设回等待、清空 `last_wheel_state`，但保留 clock、scheduler 和 transport。`close()` 先停止接受命令，在 PyBullet client 仍连接时安全停车，再幂等关闭订阅和 transport；测试和 DIRECT helper 必须先关 runtime 再断开 client。

DIRECT helper 使用可推进的假单调墙钟，每帧先推进墙钟，再在 100 Hz command scheduler 到期时提交命令，随后固定执行 `runtime.before_physics_step -> p.stepSimulation -> runtime.after_physics_step`。0.8 秒运动门禁必须得到 80 次发送，记录发送期间是否超时、最后发送时刻和静默后的首次超时时刻；发送期间不得超时，首次超时必须位于最后发送后 `[0.100, 0.100 + 1/240]` 秒，不能靠单次命令后的车辆惯性通过位移门槛。`last_state_while_sending` 在最后一次发送后的物理状态处冻结，后续静默超时状态单独记录，不能用超时后的零轮速覆盖数组顺序断言。

主动转向门禁先通过一次较大 `dt` 命令制造内部目标与未步进实际角大于 `0.1 rad` 的差值；必须在产生首次超时的 `before_physics_step()` 调用前读取 `target_before_timeout` 与 `angle_before_timeout`，因为该调用会执行安全停车并覆盖内部目标。停车后实际角应保持后者且不得继续趋近旧目标。每个参数实例独立创建和断开 DIRECT client，异常路径同样关闭 runtime。

- [ ] **Step 5: 运行聚焦测试与既有机器人回归**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_runtime.py tests/test_interface_wheel_direct.py tests/test_robot_models.py -q
```

Expected: PASS，四种车型均按约定数组顺序运行，超时后驱动归零且转向角保持。

- [ ] **Step 6: 提交**

```bash
git add slope_sim/interfaces/runtime.py slope_sim/robot.py tests/test_interface_runtime.py tests/test_interface_wheel_direct.py
git commit -m "阶段三: 1. 轮子闭环"
```

---

## Task 7：真实 eCAL 适配器与独立进程环回

**Files:**

- Create: `slope_sim/interfaces/ecal_transport.py`
- Create: `scripts/ecal_roundtrip_peer.py`
- Create: `scripts/verify_ecal_roundtrip.py`
- Modify: `environment.yml`
- Test: `tests/test_ecal_transport.py`
- Test: `tests/test_ecal_process_roundtrip.py`

- [ ] **Step 1: 写版本绑定、严格模式和自动降级失败测试**

```python
# eCAL 适配器测试：只允许已知现代/旧版 API，并区分严格与 auto 模式。
import pytest
from slope_sim.interfaces.ecal_transport import EcalUnavailableError, create_transport, load_ecal_bindings


def test_strict_ecal_mode_raises_when_bindings_are_missing(monkeypatch):
    monkeypatch.setattr("slope_sim.interfaces.ecal_transport.import_module", lambda _name: (_ for _ in ()).throw(ImportError("missing")))
    with pytest.raises(EcalUnavailableError, match="eCAL"):
        create_transport("ecal")


def test_auto_mode_falls_back_but_never_marks_ecal_connected(monkeypatch):
    monkeypatch.setattr("slope_sim.interfaces.ecal_transport.load_ecal_bindings", lambda: (_ for _ in ()).throw(EcalUnavailableError("missing")))
    transport = create_transport("auto")
    assert transport.snapshot().mode == "local"
    assert not transport.snapshot().ecal_connected
    assert transport.snapshot().detail == "eCAL 未连接"


def test_partial_ecal_initialization_closes_every_created_resource(fake_ecal_bindings):
    fake_ecal_bindings.fail_on_publisher_number = 3
    with pytest.raises(RuntimeError, match="publisher"):
        create_transport("ecal", bindings=fake_ecal_bindings)
    assert fake_ecal_bindings.open_subscribers == 0
    assert fake_ecal_bindings.open_publishers == 0
    assert fake_ecal_bindings.open_participants == 0


def test_outgoing_queue_keeps_latest_per_topic_and_counts_overwrite(fake_ecal_bindings):
    transport = create_transport("ecal", bindings=fake_ecal_bindings, start_worker=False, queue_size=2)
    transport.publish("/sim/wheel/state", b"old", "WheelState", 10)
    transport.publish("/sim/wheel/state", b"new", "WheelState", 20)
    assert transport.pending_payload("/sim/wheel/state") == b"new"
    assert transport.snapshot().dropped_count == 1
```

- [ ] **Step 2: 写真实双进程 Protobuf 环回门禁**

```python
# 该测试不得用 fake module 或 LocalTransport 替代真实 eCAL。
@pytest.mark.ecal
def test_real_ecal_process_roundtrip_exchanges_protobuf_at_target_rates():
    result = run_ecal_process_roundtrip(duration_sec=2.5)
    assert result.transport_name == "ecal"
    assert result.wall_clock_hz["/sim/wheel/command"] == pytest.approx(100.0, rel=0.05)
    assert result.wall_clock_hz["/sim/wheel/state"] == pytest.approx(100.0, rel=0.05)
    for topic in (
        "/sim/lidar/front/points", "/sim/lidar/rear/points", "/sim/rtk/state", "/sim/imu/attitude"
    ):
        assert result.wall_clock_hz[topic] == pytest.approx(10.0, rel=0.10)
        assert result.message_timestamp_hz[topic] == pytest.approx(10.0, rel=0.01)
    assert result.received_topics == {
        "/sim/wheel/state", "/sim/lidar/front/points", "/sim/lidar/rear/points",
        "/sim/rtk/state", "/sim/imu/attitude",
    }


@pytest.mark.ecal
def test_real_ecal_disconnect_reconnect_does_not_restore_stale_command():
    result = run_ecal_reconnect_gate(command=(4.0, 4.0), silence_sec=0.15)
    assert result.states == ("active", "disconnected", "waiting_peer", "active")
    assert result.drive_target_while_disconnected == (0.0, 0.0)
    assert result.drive_target_after_peer_restart_before_new_command == (0.0, 0.0)
```

- [ ] **Step 3: 运行红灯并记录缺失运行时**

```bash
conda run -n slope-sim python -m pytest tests/test_ecal_transport.py -q
conda run -n slope-sim python -m pytest tests/test_ecal_process_roundtrip.py -q -m ecal
```

Expected: 第一条因适配器不存在而 FAIL；第二条因真实 eCAL Python/runtime 不可用或脚本不存在而 FAIL，不能改成 skip 后冒充通过。

- [ ] **Step 4: 安装并固定真实 eCAL 运行时**

```bash
conda install -n slope-sim -c conda-forge ecal
conda run -n slope-sim python -c "import ecal; print(ecal.__file__)"
```

Expected: 两条命令退出码 0。把已验证的 `ecal` 依赖加入 `environment.yml`；若平台只能使用 Eclipse 官方系统包，则在交付报告记录准确包名和版本，但最终门禁仍必须在 `slope-sim` Python 中成功导入。安装写 Conda 环境需要沙箱审批时使用已获用户许可申请审批，不 vendoring 运行时。

- [ ] **Step 5: 实现现代/旧版兼容绑定与有界发布队列**

```python
def load_ecal_bindings() -> EcalBindings:
    """只探测 Eclipse eCAL 已知 API，不吞掉初始化后的真实错误。"""
    try:
        core = import_module("ecal.core")
        publisher = import_module("ecal.msg.protobuf.publisher")
        subscriber = import_module("ecal.msg.protobuf.subscriber")
        return EcalBindings.modern(core, publisher, subscriber)
    except ImportError as modern_error:
        try:
            core = import_module("ecal.core.core")
            publisher = import_module("ecal.core.publisher")
            subscriber = import_module("ecal.core.subscriber")
            return EcalBindings.legacy(core, publisher, subscriber)
        except ImportError as legacy_error:
            raise EcalUnavailableError(
                f"eCAL Python bindings unavailable: {modern_error}; {legacy_error}"
            ) from legacy_error
```

`EcalTransport` 为六个集中配置话题创建真实 Protobuf publisher/subscriber。传输回调只复制 payload、读取单调墙钟并调用上层 callback，不持有或动态查询 mailbox；上层 callback 进入时先从 runtime 原子捕获 `(mailbox_ref, generation)`，再解码，并把命令提交给同一个 mailbox 引用和 token。每个 publisher lane 固定拥有一个 `ready/in-flight` 槽；物理线程向空闲 lane 原子交接首帧，公开 `outgoing_queue_size` 只限制共享 latest 合并缓冲，每个话题在缓冲中至多保留一个最新帧。覆盖 latest 时递增 dropped 并标记 degraded，不引入 FIFO。participant、subscriber 或任一 publisher 创建失败时按已创建资源的逆序清理。对端消失后清空命令 mailbox；discovery 恢复只改为 `waiting_peer`，收到新 generation 的命令后才能回到 `active`。`close()` 先禁止新 publish/delivery/discovery并收敛 worker，把未开始 send 的 ready/latest 计为 terminal drop，再等待在途 discovery/count API，最后移除 subscriber callback、释放 subscriber/publisher 引用并 finalize participant；允许重复调用。

- [ ] **Step 6: 实现两个独立进程的验收脚本**

`scripts/ecal_roundtrip_peer.py` 只负责发送 `WheelCommand` 并订阅五个输出话题，把每条消息时间戳和类型写到临时结果 JSON；`scripts/verify_ecal_roundtrip.py` 在本任务先启动 peer 与不依赖 PyBullet 的 transport harness，等待 discovery，按消息计数和消息时间戳计算频率，任一子进程非零、缺话题、类型不符或频率越界都返回非零。Task 14 再用完成后的真实仿真运行时重复同一门禁。

```bash
conda run -n slope-sim python scripts/verify_ecal_roundtrip.py --duration-sec 2.5
```

Expected: 输出 `transport=ecal`、六话题统计和 `PASS`；不得出现 `local`。

- [ ] **Step 7: 运行测试并提交**

```bash
conda run -n slope-sim python -m pytest tests/test_ecal_transport.py tests/test_ecal_process_roundtrip.py -q -m "ecal or not ecal"
git add environment.yml slope_sim/interfaces/ecal_transport.py scripts/ecal_roundtrip_peer.py scripts/verify_ecal_roundtrip.py tests/test_ecal_transport.py tests/test_ecal_process_roundtrip.py
git commit -m "阶段三: 1. eCAL适配"
```

---

## Task 8：传感器后端、双天线 RTK 与 IMU

**Files:**

- Create: `slope_sim/sensor_backend.py`
- Create: `slope_sim/truth_sensors.py`
- Test: `tests/test_sensor_backend.py`
- Test: `tests/test_truth_sensors.py`
- Test: `tests/test_truth_sensors_direct.py`

- [ ] **Step 1: 写坐标变换和安装点校验失败测试**

```python
# 真值传感器测试：parent link 使用语义名称，四元数在进入 PyBullet 前校验。
from slope_sim.truth_sensors import MountPose, SensorMounts, TruthSensorSuite


def test_default_mounts_match_confirmed_stage3_geometry():
    mounts = SensorMounts.default()
    assert mounts.rtk_primary.position == (-0.20, 0.0, 0.18)
    assert mounts.rtk_secondary.position == (0.20, 0.0, 0.18)
    assert mounts.imu.position == (0.0, 0.0, 0.08)
    assert mounts.lidar_rear.orientation == (0.0, 0.0, 1.0, 0.0)


def test_mount_rejects_unknown_parent_and_zero_quaternion(fake_backend):
    mounts = SensorMounts.default()
    invalid_parent = dataclasses.replace(
        mounts,
        imu=MountPose("missing", mounts.imu.position, mounts.imu.orientation),
    )
    with pytest.raises(ValueError, match="parent link"):
        TruthSensorSuite(fake_backend, invalid_parent)
    with pytest.raises(ValueError, match="quaternion"):
        MountPose("base_link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))
```

- [ ] **Step 2: 写 RTK/IMU 数学和 DIRECT 真值失败测试**

```python
def test_rtk_uses_primary_to_secondary_yaw_and_wraps_to_half_open_interval(fake_backend):
    fake_backend.base_pose = ((3.0, 4.0, 1.0), quaternion_from_yaw(math.pi - 0.1))
    suite = TruthSensorSuite(fake_backend, SensorMounts.default())
    state = suite.read_rtk(timestamp_ns=10)
    assert state.main_x == pytest.approx(3.0 - 0.20 * math.cos(math.pi - 0.1))
    assert -math.pi <= state.baseline_yaw_rad < math.pi
    assert state.baseline_yaw_rad == pytest.approx(math.pi - 0.1)


@pytest.mark.parametrize(
    "yaw,expected",
    ((math.pi, -math.pi), (-math.pi, -math.pi), (math.pi + 1e-9, -math.pi + 1e-9)),
)
def test_rtk_yaw_wraps_exact_pi_boundary(yaw, expected, fake_backend):
    fake_backend.base_pose = ((0.0, 0.0, 0.0), quaternion_from_yaw(yaw))
    assert TruthSensorSuite(fake_backend, SensorMounts.default()).read_rtk(1).baseline_yaw_rad == pytest.approx(expected)


@pytest.mark.parametrize("robot_model", ("df_front", "df_mid", "df_back", "active_steering_4wd"))
def test_default_mount_parent_links_exist_on_every_deliverable_model(robot_model, direct_client):
    robot = create_robot(direct_client, robot_model)
    backend = PyBulletSensorBackend(direct_client, robot.robot_id)
    TruthSensorSuite(backend, SensorMounts.default())


@pytest.mark.parametrize("terrain", ("flat", "slope", "golf_heightfield"))
def test_direct_rtk_and_imu_match_pybullet_truth_within_1e_4(terrain):
    result = run_truth_sensor_gate(terrain)
    assert result.rtk_position_error_m <= 1e-4
    assert result.rtk_yaw_error_rad <= 1e-4
    assert result.imu_roll_error_rad <= 1e-4
    assert result.imu_pitch_error_rad <= 1e-4
```

- [ ] **Step 3: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_sensor_backend.py tests/test_truth_sensors.py tests/test_truth_sensors_direct.py -q
```

Expected: FAIL，传感器后端与真值模块不存在。

- [ ] **Step 4: 实现只读 PyBullet 传感器后端**

```python
class SensorBackend(Protocol):
    def link_names(self) -> tuple[str, ...]:
        raise NotImplementedError

    def world_pose(self, parent_link: str) -> Pose:
        raise NotImplementedError

    def transform_pose(self, parent: Pose, local: Pose) -> Pose:
        raise NotImplementedError

    def inverse_transform_point(self, pose: Pose, point: Vec3) -> Vec3:
        raise NotImplementedError

    def euler_from_quaternion(self, orientation: Quaternion) -> tuple[float, float, float]:
        raise NotImplementedError

    def ray_test_batch(
        self,
        starts: Sequence[Vec3],
        ends: Sequence[Vec3],
        *,
        collision_mask: int,
    ) -> tuple[RayHit, ...]:
        raise NotImplementedError


class PyBulletSensorBackend:
    def world_pose(self, parent_link: str) -> Pose:
        if parent_link == "base_link":
            return _base_pose(self.client_id, self.robot_id)
        link_index = self._link_name_to_index[parent_link]
        state = p.getLinkState(
            self.robot_id,
            link_index,
            computeForwardKinematics=True,
            physicsClientId=self.client_id,
        )
        return Pose(tuple(state[4]), tuple(state[5]))

    def bind_scene(
        self,
        terrain_body_ids: Collection[int],
        obstacles: Sequence[ObstacleSnapshot],
    ) -> None:
        self._hit_categories = {int(body_id): "terrain" for body_id in terrain_body_ids}
        self._hit_categories.update(
            {
                int(item.body_id): "moving_obstacle" if item.mode == "moving" else "static_obstacle"
                for item in obstacles
                if item.body_id is not None
            }
        )
```

所有 `getBasePositionAndOrientation`、`getLinkState`、`multiplyTransforms`、`invertTransform` 和 `rayTestBatch` 调用只存在于该后端，算法测试用 fake backend，不从 eCAL/Qt 线程读取 PyBullet。`RayHit` 保存命中位置、body/link 和稳定类别 `terrain/static_obstacle/moving_obstacle/unknown`；世界创建、障碍物提交和重建后调用 `bind_scene()` 更新 body ID 到逻辑类别的映射，企业消息不暴露临时 ID。

- [ ] **Step 5: 实现 RTK 和 IMU 真值生成**

```python
def read_rtk(self, timestamp_ns: int) -> RtkState:
    primary = self._world_mount(self.mounts.rtk_primary).position
    secondary = self._world_mount(self.mounts.rtk_secondary).position
    yaw = wrap_angle(math.atan2(secondary[1] - primary[1], secondary[0] - primary[0]))
    return RtkState(timestamp_ns, primary[0], primary[1], primary[2], yaw)


def read_imu(self, timestamp_ns: int) -> ImuAttitude:
    pose = self._world_mount(self.mounts.imu)
    roll, pitch, _yaw = self._backend.euler_from_quaternion(pose.orientation)
    return ImuAttitude(timestamp_ns, roll, pitch)
```

`wrap_angle()` 返回 `[-pi, pi)`；每次读取重新由当前车体真值计算，不加入噪声或滤波。

- [ ] **Step 6: 运行测试并提交**

```bash
conda run -n slope-sim python -m pytest tests/test_sensor_backend.py tests/test_truth_sensors.py tests/test_truth_sensors_direct.py -q
git add slope_sim/sensor_backend.py slope_sim/truth_sensors.py tests/test_sensor_backend.py tests/test_truth_sensors.py tests/test_truth_sensors_direct.py
git commit -m "阶段三: 1. 位姿传感器"
```

---

## Task 9：前后 16 线点云与雷达可见碰撞位

**Files:**

- Create: `slope_sim/lidar_pointcloud.py`
- Modify: `slope_sim/scene.py`
- Modify: `slope_sim/obstacles.py`
- Test: `tests/test_lidar_pointcloud.py`
- Test: `tests/test_lidar_pointcloud_direct.py`
- Test: `tests/test_lidar_collision_filters.py`

- [ ] **Step 1: 写固定扫描几何和字段失败测试**

```python
# 多线点云测试：固定 16x180、视场、量程和 Livox 风格字段语义。
from slope_sim.lidar_pointcloud import LidarConfig, MultiLineLidar


def test_default_lidar_scan_geometry_is_stable():
    config = LidarConfig.default()
    assert config.vertical_lines == 16
    assert config.horizontal_samples == 180
    assert config.horizontal_fov_deg == 180.0
    assert config.vertical_fov_deg == (-15.0, 15.0)
    assert config.min_range_m == 0.10
    assert config.max_range_m == 30.0
    assert config.ray_count == 2880


def test_cloud_preserves_ray_order_line_tag_reflectivity_and_offsets(fake_backend):
    lidar = MultiLineLidar.front(fake_backend, LidarConfig.default())
    cloud = lidar.scan(timebase_ns=1_000_000_000)
    assert cloud.frame_id == "lidar_front"
    assert cloud.lidar_id == 1
    assert cloud.point_num == len(cloud.points)
    assert [point.line for point in cloud.points] == sorted(point.line for point in cloud.points)
    assert all(0 <= point.line < 16 for point in cloud.points)
    assert [point.offset_time_ns for point in cloud.points] == sorted(point.offset_time_ns for point in cloud.points)
    assert {(point.tag, point.reflectivity) for point in cloud.points} <= {(1, 100), (2, 160), (3, 200)}


def test_empty_cloud_keeps_scan_time_and_zero_count(fake_backend):
    fake_backend.set_all_rays_to_miss()
    cloud = MultiLineLidar.front(fake_backend, LidarConfig.default()).scan(1_000_000_000)
    assert cloud.timebase_ns == 1_000_000_000
    assert cloud.point_num == 0
    assert cloud.points == ()


def test_range_boundaries_local_coordinates_and_exact_offset(fake_backend):
    fake_backend.hit_ray(0, local_point=(0.10, 0.0, 0.0), category="static_obstacle")
    fake_backend.hit_ray(2879, local_point=(30.0, 0.0, 0.0), category="terrain")
    cloud = MultiLineLidar.front(fake_backend, LidarConfig.default()).scan(5)
    assert (cloud.points[0].x, cloud.points[0].y, cloud.points[0].z) == pytest.approx((0.10, 0.0, 0.0))
    assert cloud.points[0].offset_time_ns == 0
    assert cloud.points[-1].offset_time_ns == 2879 * 100_000_000 // 2880
```

- [ ] **Step 2: 写自身过滤、前后朝向、障碍物与物理接触回归**

```python
@pytest.mark.parametrize("terrain", ("flat", "slope", "golf_heightfield"))
def test_front_and_rear_clouds_hit_terrain_without_hitting_robot(terrain):
    result = run_lidar_direct_gate(terrain=terrain, obstacle_mode=None)
    assert result.front_self_hits == 0
    assert result.rear_self_hits == 0
    assert result.front_terrain_hits > 0
    assert result.rear_terrain_hits > 0


@pytest.mark.parametrize("mode,expected_tag", (("static", 2), ("moving", 3)))
def test_obstacle_in_each_field_of_view_changes_corresponding_cloud(mode, expected_tag):
    result = run_lidar_obstacle_gate(mode)
    assert expected_tag in result.front_tags
    assert expected_tag in result.rear_tags


def test_lidar_visibility_bit_does_not_change_physical_contacts():
    result = run_lidar_collision_filter_gate()
    assert result.visible.vehicle_terrain_contacts == result.baseline.vehicle_terrain_contacts
    assert result.visible.vehicle_obstacle_contacts == result.baseline.vehicle_obstacle_contacts
    assert result.visible.final_pose == pytest.approx(result.baseline.final_pose, abs=1e-6)


def test_moving_obstacle_changes_cloud_continuously_across_scans():
    result = run_lidar_moving_obstacle_sequence(scan_count=5)
    assert len(result.closest_ranges_m) == 5
    assert all(left != right for left, right in itertools.pairwise(result.closest_ranges_m))
    assert max(abs(right - left) for left, right in itertools.pairwise(result.closest_ranges_m)) < 0.10
```

- [ ] **Step 3: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_lidar_pointcloud.py tests/test_lidar_pointcloud_direct.py tests/test_lidar_collision_filters.py -q
```

Expected: FAIL，点云模块和 `0x10` 可见碰撞位尚未实现。

- [ ] **Step 4: 实现固定射线表和坐标变换**

```python
LIDAR_VISIBLE_GROUP = 0x10


def build_unit_rays(config: LidarConfig) -> tuple[Vec3, ...]:
    rays = []
    for line, elevation in enumerate(_inclusive_angles(-15.0, 15.0, 16)):
        for azimuth in _inclusive_angles(-90.0, 90.0, 180):
            rays.append(_direction_from_degrees(azimuth, elevation))
    return tuple(rays)


def scan(self, timebase_ns: int) -> LidarPointCloud:
    parent = self._backend.world_pose(self._mount.parent_link)
    mount = self._backend.transform_pose(parent, Pose(self._mount.position, self._mount.orientation))
    starts, ends = self._world_rays(mount)
    hits = self._backend.ray_test_batch(starts, ends, collision_mask=LIDAR_VISIBLE_GROUP)
    points = tuple(self._point_from_hit(index, hit, mount) for index, hit in enumerate(hits) if hit.hit)
    return LidarPointCloud(timebase_ns, self.frame_id, len(points), self.lidar_id, points)
```

起点使用 `min_range_m`、终点使用 `max_range_m`；命中世界坐标用后端逆变换回雷达局部坐标。`offset_time_ns = floor(ray_index * 100_000_000 / 2880)`，因此即使 miss 被省略，剩余点仍保留原射线时间顺序。

- [ ] **Step 5: 在不改变物理碰撞语义的前提下加入可见位**

`scene.py` 将地形 group 从 `STATIC_COLLISION_GROUP | TERRAIN_FILTER_GROUP` 扩展为再 OR `LIDAR_VISIBLE_GROUP`，保留原 mask；`obstacles.py` 为正式质量零障碍物显式设置 `STATIC_COLLISION_GROUP | LIDAR_VISIBLE_GROUP` 和既有默认物理 mask `0x3`，临时规划可视体仍保持 group/mask 为 0。机器人 group 不增加该位，射线只用 `collisionFilterMask=0x10`。

```python
TERRAIN_COLLISION_GROUP = STATIC_COLLISION_GROUP | TERRAIN_FILTER_GROUP | LIDAR_VISIBLE_GROUP
OBSTACLE_COLLISION_GROUP = STATIC_COLLISION_GROUP | LIDAR_VISIBLE_GROUP
OBSTACLE_COLLISION_MASK = 0x3


p.setCollisionFilterGroupMask(
    body_id,
    -1,
    collisionFilterGroup=OBSTACLE_COLLISION_GROUP,
    collisionFilterMask=OBSTACLE_COLLISION_MASK,
    physicsClientId=client_id,
)
```

- [ ] **Step 6: 运行测试并提交**

```bash
conda run -n slope-sim python -m pytest tests/test_lidar_pointcloud.py tests/test_lidar_pointcloud_direct.py tests/test_lidar_collision_filters.py tests/test_obstacles.py tests/test_scene.py -q
git add slope_sim/lidar_pointcloud.py slope_sim/scene.py slope_sim/obstacles.py tests/test_lidar_pointcloud.py tests/test_lidar_pointcloud_direct.py tests/test_lidar_collision_filters.py
git commit -m "阶段三: 1. 前后点云"
```

---

## Task 10：版本化场景导出、加载与事务输入

**Files:**

- Create: `slope_sim/scene_config.py`
- Test: `tests/test_scene_config.py`
- Test: `tests/test_scene_config_atomic.py`

- [ ] **Step 1: 写 schema round-trip 与临时 ID 排除失败测试**

```python
# 场景文件测试：只保存逻辑语义，不泄漏 PyBullet/Qt/eCAL 临时对象。
from slope_sim.scene_config import SceneDocument, dump_scene_atomic, load_scene


def test_scene_document_round_trips_robot_terrain_obstacles_and_sensor_mounts(tmp_path):
    document = sample_scene_document()
    path = tmp_path / "scene.yaml"
    dump_scene_atomic(document, path)
    loaded = load_scene(path)
    assert loaded == document
    text = path.read_text(encoding="utf-8")
    assert "schema_version: 1" in text
    assert "body_id" not in text
    assert "link_index" not in text
    assert "transport_mode" not in text


def test_same_document_has_stable_yaml_and_logical_digest(tmp_path):
    first = tmp_path / "first.yaml"
    second = tmp_path / "second.yaml"
    dump_scene_atomic(sample_scene_document(), first)
    dump_scene_atomic(sample_scene_document(), second)
    assert first.read_bytes() == second.read_bytes()
```

- [ ] **Step 2: 写未知版本、非法值和原子替换失败测试**

```python
def test_load_rejects_unknown_version_before_runtime_mutation(tmp_path):
    path = tmp_path / "future.yaml"
    path.write_text("schema_version: 2\nrobot:\n  model: df_back\n", encoding="utf-8")
    with pytest.raises(ValueError, match="schema_version 2"):
        load_scene(path)


def test_atomic_export_preserves_previous_file_when_replace_fails(tmp_path, monkeypatch):
    path = tmp_path / "scene.yaml"
    path.write_text("previous", encoding="utf-8")
    monkeypatch.setattr("slope_sim.scene_config.os.replace", lambda *_args: (_ for _ in ()).throw(OSError("disk")))
    with pytest.raises(OSError, match="disk"):
        dump_scene_atomic(sample_scene_document(), path)
    assert path.read_text(encoding="utf-8") == "previous"
    assert list(tmp_path.glob(".scene.yaml.*.tmp")) == []


@pytest.mark.parametrize(
    "mutate,match",
    (
        (lambda raw: raw.update({"unexpected": 1}), "unknown"),
        (lambda raw: raw.pop("robot"), "robot"),
        (lambda raw: raw["obstacles"].append(dict(raw["obstacles"][0])), "duplicate"),
        (lambda raw: raw["obstacles"][0].update({"orientation": [0, 0, 0, 0]}), "quaternion"),
        (lambda raw: raw["obstacles"][0].update({"position": [1e9, 0, 0]}), "bounds"),
    ),
)
def test_load_rejects_each_invalid_scene_before_runtime(mutate, match, tmp_path):
    raw = sample_scene_mapping()
    mutate(raw)
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    with pytest.raises(ValueError, match=match):
        load_scene(path)
```

- [ ] **Step 3: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_scene_config.py tests/test_scene_config_atomic.py -q
```

Expected: FAIL，`scene_config.py` 尚不存在。

- [ ] **Step 4: 实现严格 frozen 场景文档**

```python
SCENE_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class TerrainDocument:
    terrain_model: str
    slope_deg: float
    golf_seed: int
    golf_relief: str


@dataclass(frozen=True)
class SceneDocument:
    schema_version: int
    robot_model: str
    terrain: TerrainDocument
    obstacles: tuple[ObstacleSnapshot, ...]
    sensors: SensorMounts

    @classmethod
    def from_runtime(cls, robot_model, terrain, obstacles, sensors) -> "SceneDocument":
        return cls(
            SCENE_SCHEMA_VERSION,
            get_robot_model(robot_model).name,
            TerrainDocument.from_selection(terrain),
            tuple(_without_body_id(item) for item in obstacles),
            sensors,
        )
```

`__post_init__` 校验：版本必须等于 1；车型、场地、relief、parent link 和障碍物枚举有效；逻辑 ID 唯一；位置/路径/速度/进度均有限且有界；四元数有限、非零并规范化；LiDAR 必须是 16×180、180°、±15°、0.1–30 m。读取器只接受精确键集合，未知键和 YAML 非 mapping 明确拒绝。

- [ ] **Step 5: 实现结构化 YAML 和原子写入**

```python
def dump_scene_atomic(document: SceneDocument, path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{target.name}.", suffix=".tmp", dir=target.parent)
    temp = Path(temp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            yaml.safe_dump(document_to_mapping(document), stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, target)
    except BaseException:
        temp.unlink(missing_ok=True)
        raise
    return target


def load_scene(path: str | Path) -> SceneDocument:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return scene_document_from_mapping(_require_mapping(raw))
```

解析路径使用 dataclass/映射，不用字符串替换；导出的障碍物按 `logical_id` 排序，字段顺序固定以便复现和审查。

- [ ] **Step 6: 运行测试并提交**

```bash
conda run -n slope-sim python -m pytest tests/test_scene_config.py tests/test_scene_config_atomic.py tests/test_obstacles.py -q
git add slope_sim/scene_config.py tests/test_scene_config.py tests/test_scene_config_atomic.py
git commit -m "阶段三: 1. 场景文件"
```

---

## Task 11：二进制接口日志与 JSONL 事件

**Files:**

- Create: `proto/slope_sim_internal.proto`
- Modify: `scripts/generate_protos.py`
- Generate: `slope_sim/interfaces/generated/slope_sim_internal_pb2.py`
- Create: `slope_sim/interfaces/logging.py`
- Test: `tests/test_interface_logging.py`

- [ ] **Step 1: 写有序二进制 round-trip 和事件日志失败测试**

```python
# 接口日志测试：长度前缀 envelope 必须恢复原话题、类型、方向和 payload。
from slope_sim.interfaces.logging import InterfaceEventLogger, InterfaceLogRecord, read_interface_log


def test_binary_log_round_trips_records_in_sequence(tmp_path):
    logger = InterfaceEventLogger(tmp_path, prefix="gate", queue_size=8)
    logger.record_message(InterfaceLogRecord(1, "/sim/wheel/command", "receive", 10, 20, "WheelCommand", b"a"))
    logger.record_message(InterfaceLogRecord(2, "/sim/wheel/state", "publish", 30, 40, "WheelState", b"bb"))
    paths = logger.close()
    assert read_interface_log(paths.binary_path) == (
        InterfaceLogRecord(1, "/sim/wheel/command", "receive", 10, 20, "WheelCommand", b"a"),
        InterfaceLogRecord(2, "/sim/wheel/state", "publish", 30, 40, "WheelState", b"bb"),
    )


def test_event_jsonl_records_invalid_command_and_transport_context(tmp_path):
    logger = InterfaceEventLogger(tmp_path, prefix="gate", queue_size=8)
    logger.record_event(
        "invalid_command",
        wall_time_ns=20,
        sim_time_ns=10,
        robot_model="df_back",
        terrain_model="flat",
        topic="/sim/wheel/command",
        reason="drive[0] exceeds 20.0",
    )
    paths = logger.close()
    event = json.loads(paths.event_path.read_text(encoding="utf-8").splitlines()[0])
    assert event["event"] == "invalid_command"
    assert event["robot_model"] == "df_back"
    assert event["reason"] == "drive[0] exceeds 20.0"
```

- [ ] **Step 2: 写 bounded queue、精确丢帧和刷盘失败测试**

```python
def test_full_queue_never_blocks_and_counts_each_dropped_record(tmp_path, blocked_writer):
    logger = InterfaceEventLogger(tmp_path, prefix="overload", queue_size=2, writer=blocked_writer)
    started = time.monotonic()
    for sequence in range(10):
        logger.record_message(sample_record(sequence))
    assert time.monotonic() - started < 0.050
    assert logger.snapshot().dropped_messages == 8
    blocked_writer.release()
    logger.close()


def test_close_is_idempotent_and_flushes_accepted_records(tmp_path):
    logger = InterfaceEventLogger(tmp_path, prefix="close", queue_size=4)
    logger.record_message(sample_record(1))
    first = logger.close()
    second = logger.close()
    assert second == first
    assert [record.sequence for record in read_interface_log(first.binary_path)] == [1]


@pytest.mark.parametrize("corruption", ("short_prefix", "oversized_length", "short_payload", "bad_protobuf"))
def test_reader_rejects_each_corrupt_binary_frame(tmp_path, corruption):
    path = write_corrupt_interface_log(tmp_path, corruption)
    with pytest.raises(ValueError, match="interface log"):
        read_interface_log(path)


@pytest.mark.parametrize(
    "event",
    (
        "protobuf_parse_failed", "invalid_command", "command_timeout", "model_mismatch",
        "mechanical_limit", "ecal_initialized", "ecal_disconnected", "ecal_reconnected",
        "ecal_closed", "sensor_failed", "publish_failed", "queue_dropped",
    ),
)
def test_all_required_event_categories_are_json_serializable(tmp_path, event):
    logger = InterfaceEventLogger(tmp_path, prefix=event, queue_size=2)
    assert logger.record_event(event, wall_time_ns=1, sim_time_ns=2, robot_model="df_back", terrain_model="flat")
    assert json.loads(logger.close().event_path.read_text(encoding="utf-8"))["event"] == event


def test_writer_failure_is_reported_on_close_and_does_not_deadlock(tmp_path, failing_writer):
    logger = InterfaceEventLogger(tmp_path, prefix="failure", writer=failing_writer)
    logger.record_message(sample_record(1))
    with pytest.raises(RuntimeError, match="writer failed"):
        logger.close()
```

- [ ] **Step 3: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_logging.py -q
```

Expected: FAIL，内部 envelope 与日志模块不存在。

- [ ] **Step 4: 增加独立内部 Protobuf envelope**

```proto
syntax = "proto3";
package slope_sim.internal.v1;

message InterfaceLogEnvelope {
  uint64 sequence = 1;
  string topic = 2;
  string direction = 3;
  uint64 sim_time_ns = 4;
  uint64 wall_time_ns = 5;
  string type_name = 6;
  bytes payload = 7;
}
```

`scripts/generate_protos.py` 对 `proto/` 下两个固定文件分别生成，企业 `.proto` 字段不增加内部日志内容。二进制格式为每帧 `4-byte little-endian uint32 length + serialized InterfaceLogEnvelope`，读取器拒绝截断前缀、超长帧、截断 payload 和非法 Protobuf。

- [ ] **Step 5: 实现异步有界写入和可读事件**

```python
class InterfaceEventLogger:
    def record_message(self, record: InterfaceLogRecord) -> bool:
        return self._enqueue(_MessageItem(record))

    def record_event(self, event: str, **fields: object) -> bool:
        return self._enqueue(_EventItem(_validated_event(event, fields)))

    def close(self) -> InterfaceLogPaths:
        """停止接收、等待已接受项刷盘，并返回稳定路径；重复关闭返回同一结果。"""
        with self._close_lock:
            if self._paths is not None:
                return self._paths
            self._accepting = False
            self._queue.put(_STOP)
        self._worker.join()
        if self._worker_error is not None:
            raise RuntimeError("interface log writer failed") from self._worker_error
        self._paths = InterfaceLogPaths(self._binary_path, self._event_path)
        return self._paths
```

物理线程先通过容量 semaphore 预留一个“排队或写入中”名额，再做 `put_nowait`；后台写完才释放名额，因此 `queue_size=2` 在 writer 阻塞时稳定只接受两条。满队列不覆盖已接受记录，当前记录丢弃并精确累计。后台线程独占文件句柄，事件每行 `json.dumps(event, ensure_ascii=False, sort_keys=True)`；关闭停止接收新记录、插入终止标记、join 后 flush/fsync/close。

- [ ] **Step 6: 运行生成一致性、测试并提交**

```bash
conda run -n slope-sim python scripts/generate_protos.py
conda run -n slope-sim python -m pytest tests/test_interface_logging.py tests/test_proto_contract.py -q
conda run -n slope-sim python scripts/generate_protos.py
git diff --exit-code -- slope_sim/interfaces/generated
git add proto/slope_sim_internal.proto scripts/generate_protos.py slope_sim/interfaces/generated/slope_sim_internal_pb2.py slope_sim/interfaces/logging.py tests/test_interface_logging.py
git commit -m "阶段三: 1. 接口日志"
```

---

## Task 12：主循环集成、暂停、场景事务与关闭顺序

**Files:**

- Modify: `slope_sim/interfaces/runtime.py`
- Modify: `slope_sim/config.py`
- Modify: `slope_sim/coordinator.py`
- Modify: `slope_sim/runtime_actions.py`
- Modify: `slope_sim/manual_demo.py`
- Modify: `slope_sim/dashboard.py`
- Modify: `slope_sim/simulation.py`
- Modify: `main.py`
- Modify: `configs/experiment.yaml`
- Test: `tests/test_interface_runtime_integration.py`
- Test: `tests/test_interface_pause_rebuild.py`
- Test: `tests/test_scene_runtime_transaction.py`
- Test: `tests/test_entrypoints.py`
- Test: `tests/test_manual_demo.py`

- [ ] **Step 1: 写 CLI、auto/local/ecal 和场景文件失败测试**

```python
def test_main_accepts_stage3_interface_and_scene_options():
    args = parse_args([
        "--gui", "--manual", "--interface-mode", "local",
        "--scene-in", "input.yaml", "--scene-out", "output.yaml",
    ])
    assert args.interface_mode == "local"
    assert args.scene_in == Path("input.yaml")
    assert args.scene_out == Path("output.yaml")


def test_experiment_config_rejects_unknown_interface_mode():
    with pytest.raises(ValueError, match="interface_mode"):
        ExperimentConfig(interface_mode="fake")
```

- [ ] **Step 2: 写暂停、故障隔离、重建清空和关闭顺序失败测试**

```python
def test_pause_stops_clock_and_all_physical_topics_but_keeps_connection_polling(runtime_fixture):
    runtime, wall_clock = runtime_fixture
    for _ in range(24):
        runtime.step_frame(1.0 / 240.0, paused=False)
    before = runtime.counters()
    assert before.physical_messages > 0
    runtime.pause()
    wall_clock.advance(0.25)
    for _ in range(60):
        runtime.step_frame(1.0 / 240.0, paused=True)
    assert runtime.counters().physical_messages == before.physical_messages
    assert runtime.clock.now_ns == before.sim_time_ns
    assert runtime.counters().connection_polls > before.connection_polls
    runtime.resume(wall_time=wall_clock())
    assert runtime.last_decision.timed_out


def test_rebuild_requires_new_command_and_rebinds_sensor_parent_links(
    runtime_fixture,
    replacement_robot,
    replacement_sensor_backend,
    replacement_scene_document,
):
    runtime, _clock = runtime_fixture
    runtime.accept_local_command(valid_command(runtime.robot_model), received_at=1.0)
    runtime.prepare_world_rebuild()
    runtime.commit_world_rebuild(replacement_robot, replacement_sensor_backend, replacement_scene_document)
    assert runtime.last_decision.waiting
    assert runtime.bound_robot_id == replacement_robot.robot_id


def test_sensor_failure_isolated_to_one_topic(runtime_fixture):
    runtime, _clock = runtime_fixture
    runtime.sensor_suite.front_lidar.scan = lambda _stamp: (_ for _ in ()).throw(RuntimeError("ray failure"))
    runtime.run_for_sim_seconds(0.2)
    assert runtime.status_snapshot().topics["/sim/lidar/front/points"].state == "error"
    assert runtime.status_snapshot().topics["/sim/lidar/rear/points"].message_count >= 2
    assert runtime.status_snapshot().topics["/sim/rtk/state"].message_count >= 2


def test_close_order_is_safe_and_idempotent(runtime_fixture):
    runtime, _clock = runtime_fixture
    runtime.close()
    runtime.close()
    assert runtime.close_trace == (
        "stop_commands", "safe_stop", "stop_sensors", "close_log", "close_transport", "close_sensors"
    )
```

- [ ] **Step 3: 写场景事务失败回滚测试**

```python
def test_scene_load_failure_rolls_back_robot_terrain_obstacles_and_sensor_binding(coordinator, tmp_path):
    original = coordinator.logical_scene_document()
    original_body_ids = current_body_ids(coordinator.client_id)
    participant = coordinator.interface_runtime.transport_participant
    worker_threads = interface_worker_threads()
    target = sample_scene_document(robot_model="active_steering_4wd", terrain_model="golf_heightfield")
    coordinator.fail_next_obstacle_restore = True
    result = coordinator.apply_scene_document(target)
    assert result.error_message is not None
    assert coordinator.logical_scene_document() == original
    assert coordinator.interface_runtime.scene_document == original
    assert len(current_body_ids(coordinator.client_id)) == len(original_body_ids)
    assert no_duplicate_logical_obstacles(coordinator.obstacle_manager.snapshot())
    assert coordinator.interface_runtime.transport_participant is participant
    assert interface_worker_threads() == worker_threads
```

- [ ] **Step 4: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_runtime_integration.py tests/test_interface_pause_rebuild.py tests/test_scene_runtime_transaction.py tests/test_entrypoints.py tests/test_manual_demo.py -q
```

Expected: FAIL，新 CLI、暂停生命周期和全场景事务尚未接入。

- [ ] **Step 5: 扩展配置、CLI 和运行结果**

`ExperimentConfig` 增加并验证：

```python
interface_mode: str = "auto"
interface_enabled: bool = True
interface_log_enabled: bool = True
scene_in: Path | None = None
scene_out: Path | None = None
developer_diagnostics_enabled: bool = False
```

`main.py` 增加 `--interface-mode {auto,ecal,local}`、`--no-interface`、`--no-interface-log`、`--scene-in`、`--scene-out`、`--developer-diagnostics`，并输出 `interface_binary_log`、`interface_event_log`、`scene_export`。严格 `ecal` 初始化失败返回非零；`auto` 降级继续运行但状态必须是“eCAL 未连接”。

- [ ] **Step 6: 把运行时接入唯一物理主线程**

```python
pacer = _DeadlinePacer(config.time_step)
pacer.start()
while not should_exit:
    dashboard_command = dashboard.current_command() if dashboard else DashboardCommand.idle()
    paused = dashboard_command.paused
    interface_runtime.poll_transport()
    if not paused:
        interface_runtime.before_physics_step(config.time_step)
        coordinator.step(config.time_step)
        interface_runtime.after_physics_step(config.time_step)
    if dashboard:
        dashboard.update_interface_status(interface_runtime.status_snapshot())
    pacer.wait_for_next_deadline()
```

`_DeadlinePacer` 使用上一绝对期限累加下一期限；帧内工作只消耗本帧余量，超期时调用 `sleep(0)` 让出执行权，不能追加固定正延时扩大墙钟欠债。

本地模式把现有键盘/按钮 `linear_velocity`、`angular_velocity` 在 100 Hz deadline 转换为当前车型的 `WheelCommand`，再走同一 codec、mailbox、限位和超时；eCAL 模式忽略 Dashboard 直接运动值。eCAL 命令 callback 的第一步是在 runtime lifecycle 锁内捕获 `(mailbox_ref, generation)`，解码完成后只向该引用提交 token；重建或关闭不能让旧 payload 通过动态 `runtime._mailbox` 进入新 mailbox。主循环不再在接口启用时同时调用 `command_twist()` 和接口轮速控制。

轮子状态、前后点云、RTK、IMU 分别按 scheduler deadline 生成、编码、记录并提交传输；只有传输层成功接受的输出写二进制日志。每个传感器单独 `try/except` 并更新对应状态，不发布半帧。

- [ ] **Step 7: 集成全场景事务和运行时重绑定**

`dashboard.py` 在不改变 Task 13 最终布局前先给 `DashboardCommand` 增加 `paused: bool = False` 和只读暂停状态；Task 13 再添加可见按钮。`runtime_actions.py` 增加纯领域 `LoadSceneAction(document)`；`SimulationCoordinator.apply_scene_document()` 在任何 PyBullet 删除前完成文档校验，随后调用：

```python
self.interface_runtime.prepare_world_rebuild()
try:
    candidate_world, candidate_manager = self._build_scene_document(document)
except Exception:
    restored_world, restored_manager = self._restore_scene_document(previous)
    self.interface_runtime.commit_world_rebuild(restored_world.active_robot.robot, restored_backend, previous)
    raise
else:
    self.world = candidate_world
    self.obstacle_manager = candidate_manager
    self.interface_runtime.commit_world_rebuild(candidate_world.active_robot.robot, candidate_backend, document)
```

车型切换、复位和场地切换也调用相同 prepare/commit hooks；任何重建后旧命令清空，participant 保留。退出时按 Task 12 Step 2 的顺序关闭；部分初始化失败走同一 `finally`，最后断开 PyBullet。

- [ ] **Step 8: 运行聚焦集成和阶段一/二回归**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_runtime_integration.py tests/test_interface_pause_rebuild.py tests/test_scene_runtime_transaction.py tests/test_entrypoints.py tests/test_manual_demo.py tests/test_coordinator.py -q
conda run -n slope-sim python scripts/verify_stage1_matrix.py
conda run -n slope-sim python scripts/verify_stage2_obstacles.py
```

Expected: 聚焦测试 PASS，阶段一矩阵和阶段二脚本均无 FAIL。

- [ ] **Step 9: 提交**

```bash
git add slope_sim/interfaces/runtime.py slope_sim/config.py slope_sim/coordinator.py slope_sim/runtime_actions.py slope_sim/manual_demo.py slope_sim/dashboard.py slope_sim/simulation.py main.py configs/experiment.yaml tests/test_interface_runtime_integration.py tests/test_interface_pause_rebuild.py tests/test_scene_runtime_transaction.py tests/test_entrypoints.py tests/test_manual_demo.py
git commit -m "阶段三: 1. 运行时集成"
```

---

## Task 12R：队列丢帧事件与关闭期收敛审查修复

**Files:**

- Modify: `slope_sim/interfaces/logging.py`
- Modify: `slope_sim/interfaces/transport.py`
- Modify: `slope_sim/interfaces/ecal_transport.py`
- Modify: `slope_sim/interfaces/runtime.py`
- Test: `tests/test_interface_logging.py`
- Test: `tests/test_local_transport.py`
- Test: `tests/test_ecal_transport.py`
- Test: `tests/test_interface_runtime_integration.py`

- [x] **Step 1: 写 logger 队列拒绝最终必须落盘的失败测试**

```python
def test_logger_queue_rejection_is_persisted_after_capacity_recovers(tmp_path):
    gate = BlockingWriter()
    logger = InterfaceEventLogger(tmp_path, queue_size=1, writer=gate)
    runtime = runtime_with_logger(logger)
    runtime.record_test_message(sample_log_record(sequence=0))
    assert not runtime.record_test_message(sample_log_record(sequence=1))
    gate.release()
    wait_until(lambda: logger.snapshot().accepted_messages == 1)
    runtime.flush_pending_quality_events()
    paths = logger.paths
    runtime.close()
    events = read_jsonl(paths.event_path)
    assert one_event(events, "queue_dropped", source="interface_logger", count=1)


def test_terminal_logger_drop_event_waits_for_capacity_only_during_close(tmp_path):
    logger = InterfaceEventLogger(tmp_path, queue_size=1, writer=slow_successful_writer)
    runtime = runtime_with_one_pending_logger_drop(logger)
    started = time.monotonic()
    runtime.close()
    assert time.monotonic() - started < 2.0
    assert one_event(read_jsonl(logger.paths.event_path), "queue_dropped", source="interface_logger", count=1)
```

物理循环中的正常 `record_message`/`record_event` 仍必须无等待；只有关闭路径可使用有界等待提交终态质量事件。

- [x] **Step 2: 写 transport quiesce 在 logger 关闭前暴露最终丢帧的失败测试**

```python
def test_runtime_logs_transport_pending_drop_before_logger_closes(runtime_fixture):
    runtime, transport, logger = runtime_fixture.with_blocked_ecal_worker(queue_size=1)
    transport.publish("/sim/wheel/state", b"old", "WheelState", 10)
    transport.publish("/sim/wheel/state", b"new", "WheelState", 20)
    runtime.close()
    events = read_jsonl(logger.paths.event_path)
    assert one_event(events, "queue_dropped", source="transport", count=1)
    assert runtime.close_trace == (
        "stop_commands", "safe_stop", "stop_sensors", "quiesce_transport",
        "close_log", "close_transport", "close_sensors",
    )
```

同时测试 `LocalTransport.quiesce()`、重复 quiesce/close、回调上下文 close 和 quiesce 后禁止启动新回调。

- [x] **Step 3: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_logging.py tests/test_local_transport.py tests/test_ecal_transport.py tests/test_interface_runtime_integration.py -q
```

Expected: FAIL，缺少 terminal event、transport quiesce 和新的关闭轨迹；失败不得来自已有日志 round-trip。

- [x] **Step 4: 实现 logger 有界终态事件**

```python
def record_terminal_event(
    self,
    event: str,
    *,
    timeout_sec: float = 1.0,
    **fields: object,
) -> bool:
    """只供关闭路径在有界时间内等待容量并提交最终质量事件。"""
```

同时增加只读 `paths: InterfaceLogPaths` property，供关闭前固定最终文件位置。`record_terminal_event()` 先完成与 `record_event()` 相同的 JSON 校验，再以 timeout 获取 capacity；获取后仍在生命周期锁内确认 accepting，最后复用 `_commit_reserved()`。超时、writer 失败或关闭竞态返回 `False` 并增加 dropped event 计数，绝不无限等待。

`InterfaceRuntime` 维护 `_pending_logger_drops`；正常运行中下一次成功事件前尝试非阻塞聚合提交，关闭时调用 `record_terminal_event("queue_dropped", source="interface_logger", count=self._pending_logger_drops)`。不能递归把该终态事件自身拒绝再次加入 pending。

- [x] **Step 5: 增加 transport quiesce 生命周期**

```python
class Transport(Protocol):
    def quiesce(self) -> TransportSnapshot:
        """停止新交付、收敛 worker，并返回关闭资源前的最终质量快照。"""

    def close(self) -> None:
        """在 quiesce 后幂等关闭 participant、publisher 和 subscriber。"""
```

`LocalTransport.quiesce()` 禁止新发布并等待外部 in-flight callback；回调上下文调用时只禁止新交付并返回，保持现有防死锁语义。`EcalTransport.quiesce()` 停止接受新 payload、结束发布 worker、准确统计尚未发送的逐话题 pending frame，再返回最终 `TransportSnapshot`；`close()` 只负责资源终结，重复调用不重复计数。

`InterfaceRuntime.close()` 顺序改为：停止命令、车辆安全停止、停止传感器、transport quiesce、消费最终质量并写聚合事件、关闭 logger、关闭 transport 资源、关闭传感器。该顺序保持日志在所有丢帧计数确定后才终结。

当前生命周期补充合同：轮速/传感器读取和 `bind_scene()` 统一登记为 world operation；prepare、wheel-only rebind 和 close 封锁新操作后等待旧操作退出。rebind 不等待已经进入 transport 的旧 publish。`publish/receive/logger` 回调和 lifecycle owner 的同线程 `prepare/rebind/commit/abort/fault/close` 立即抛错，不进入 condition 等待；其他线程遇到已有 owner 时串行等待后重新竞争。rebind 在 safe-stop 前失败恢复旧准入；safe-stop 后提交异常恢复旧 robot/model/mailbox/subscription 引用但进入 `faulted`，不重新激活旧 token，候选 subscription 关闭且统一 `close()` 仍可释放旧引用。

- [x] **Step 6: 运行聚焦和生命周期回归**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_logging.py tests/test_local_transport.py tests/test_ecal_transport.py tests/test_interface_runtime.py tests/test_interface_runtime_integration.py tests/test_interface_pause_rebuild.py -q
git diff --check
```

Expected: 全部 PASS；物理路径仍无阻塞，close trace 与设计一致，JSONL 包含 logger 和 transport 两类 `queue_dropped`。

- [x] **Step 7: 完成 Task 12R 规格审查和代码质量审查**

规格审查核对两类丢帧都持久化且关闭顺序符合日志完整性；质量审查检查 callback-close 死锁、semaphore 泄漏、终态事件递归、重复 close 计数和 writer 失败路径。发现问题先补红灯测试。

---

## Task 13A：Dashboard 不可变快照与 LiDAR 俯视投影

**Files:**

- Create: `slope_sim/interfaces/dashboard_snapshot.py`
- Modify: `slope_sim/interfaces/runtime.py`
- Modify: `slope_sim/lidar_pointcloud.py`
- Modify: `slope_sim/interfaces/__init__.py`
- Test: `tests/test_interface_dashboard_snapshot.py`
- Test: `tests/test_interface_runtime.py`
- Test: `tests/test_lidar_pointcloud.py`

- [x] **Step 1: 写组合快照冻结和字段校验失败测试**

```python
def test_interface_dashboard_snapshot_copies_lidar_points_and_keeps_payloads_immutable():
    point = LidarTopViewPoint(x=1.0, y=-2.0, tag=2, lidar_id=1)
    source = [point]
    snapshot = InterfaceDashboardSnapshot(
        generation=3,
        robot_model="active_steering_4wd",
        sim_time_ns=100_000_000,
        status=sample_status_snapshot(),
        wheel_command=WheelCommand(90_000_000, (1.0, 2.0, 3.0, 4.0), (0.1, -0.1)),
        wheel_command_received_sim_time_ns=80_000_000,
        wheel_state=WheelState(100_000_000, (0.9, 1.9, 2.9, 3.9), (0.02, -0.02)),
        lidar_front=sample_cloud(lidar_id=1),
        lidar_rear=sample_cloud(lidar_id=2),
        rtk=RtkState(100_000_000, 1.0, 2.0, 3.0, 0.5),
        imu=ImuAttitude(100_000_000, 0.1, -0.2),
        lidar_front_view=LidarTopViewFrame(100_000_000, source),
        lidar_rear_view=None,
    )
    source.append(LidarTopViewPoint(9.0, 9.0, 3, 2))
    assert snapshot.lidar_front_view.points == (point,)
    with pytest.raises(FrozenInstanceError):
        snapshot.generation = 4


@pytest.mark.parametrize(
    "factory,match",
    (
        (lambda: LidarTopViewPoint(float("nan"), 0.0, 1, 1), "finite"),
        (lambda: LidarTopViewPoint(0.0, 0.0, 4, 1), "tag"),
        (lambda: LidarTopViewPoint(0.0, 0.0, 1, 3), "lidar_id"),
    ),
)
def test_lidar_top_view_point_rejects_invalid_values(factory, match):
    with pytest.raises(ValueError, match=match):
        factory()
```

- [x] **Step 2: 写前后雷达到 `base_link` 的数值投影失败测试**

```python
def test_scan_with_top_view_projects_sensor_hits_into_base_link(recording_backend):
    recording_backend.poses["base_link"] = Pose((10.0, 20.0, 0.0), yaw_quaternion(0.5))
    recording_backend.poses["lidar_front_mount"] = Pose((10.4, 20.1, 0.2), yaw_quaternion(0.5))
    recording_backend.hits = one_static_hit_at_world_position((11.0, 21.0, 0.0))
    result = MultiLineLidar.front(recording_backend, LidarConfig.default()).scan_with_top_view(100)
    expected = recording_backend.inverse_transform_point(
        recording_backend.poses["base_link"],
        (11.0, 21.0, 0.0),
    )
    assert result.message.point_num == 1
    assert result.top_view.points == (
        LidarTopViewPoint(expected[0], expected[1], tag=2, lidar_id=1),
    )
```

同时增加后雷达 yaw=pi、空点云、所有 5760 条射线命中和未知命中类别测试。`scan()` 必须继续只返回原 `LidarPointCloud`，现有企业消息调用方不破坏。

- [x] **Step 3: 写 runtime 成功提交、失败隔离和 generation 清理失败测试**

```python
def test_dashboard_snapshot_only_advances_after_valid_command_and_successful_publish(runtime_fixture):
    runtime, transport, clock = runtime_fixture
    assert runtime.dashboard_snapshot().wheel_command is None
    received_sim_time_ns = runtime.dashboard_snapshot().sim_time_ns
    command = valid_command(runtime.robot_model, timestamp_ns=10)
    assert runtime.accept_local_command(command, received_at=clock())
    runtime.before_physics_step(0.01, wall_time=clock())
    runtime.after_physics_step(0.01)
    first = runtime.dashboard_snapshot()
    assert first.wheel_command == command
    assert first.wheel_command_received_sim_time_ns == received_sim_time_ns
    assert first.wheel_command_received_sim_time_ns != command.timestamp_ns
    assert first.wheel_state is not None

    transport.fail_topic(runtime.config.rtk.topic)
    runtime.run_for_sim_seconds(0.1)
    failed = runtime.dashboard_snapshot()
    assert failed.rtk == first.rtk
    assert failed.status.topics[runtime.config.rtk.topic].state == "error"


def test_world_rebuild_changes_dashboard_generation_and_clears_every_latest_payload(runtime_fixture):
    runtime = runtime_fixture.runtime
    populated = publish_one_complete_interface_cycle(runtime)
    runtime.prepare_world_rebuild()
    runtime.commit_world_rebuild(
        runtime_fixture.replacement_robot,
        runtime_fixture.replacement_backend,
        runtime_fixture.replacement_scene,
    )
    cleared = runtime.dashboard_snapshot()
    assert cleared.generation > populated.generation
    assert all(
        value is None
        for value in (
            cleared.wheel_command, cleared.wheel_command_received_sim_time_ns,
            cleared.wheel_state, cleared.lidar_front,
            cleared.lidar_rear, cleared.rtk, cleared.imu,
            cleared.lidar_front_view, cleared.lidar_rear_view,
        )
    )
```

- [x] **Step 4: 运行红灯并确认失败原因**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_dashboard_snapshot.py tests/test_lidar_pointcloud.py tests/test_interface_runtime.py -q
```

Expected: FAIL，缺少 `dashboard_snapshot` 模块、`scan_with_top_view()` 和 runtime 最新消息边界；不得因已有 `InterfaceStatusSnapshot` 测试失败。

- [x] **Step 5: 实现冻结快照与单次扫描投影**

```python
# slope_sim/interfaces/dashboard_snapshot.py
@dataclass(frozen=True, slots=True)
class LidarTopViewPoint:
    x: float
    y: float
    tag: int
    lidar_id: int


@dataclass(frozen=True, slots=True)
class LidarTopViewFrame:
    timestamp_ns: int
    points: tuple[LidarTopViewPoint, ...]


@dataclass(frozen=True, slots=True)
class InterfaceDashboardSnapshot:
    generation: int
    robot_model: str
    sim_time_ns: int
    status: InterfaceStatusSnapshot
    wheel_command: WheelCommand | None = None
    wheel_command_received_sim_time_ns: int | None = None
    wheel_state: WheelState | None = None
    lidar_front: LidarPointCloud | None = None
    lidar_rear: LidarPointCloud | None = None
    rtk: RtkState | None = None
    imu: ImuAttitude | None = None
    lidar_front_view: LidarTopViewFrame | None = None
    lidar_rear_view: LidarTopViewFrame | None = None
```

每个 `__post_init__` 严格拒绝 bool、负 generation、越界 uint64、未知车型、非有限坐标、tag 非 `0..3`、lidar ID 非 `1..2` 和类型不匹配；序列统一复制为 tuple。`wheel_command` 与 `wheel_command_received_sim_time_ns` 必须同时为值或同时为 `None`，命令接收时间使用 runtime 接受该有效命令时的 `_clock.now_ns`，不得复用外部发送方 `WheelCommand.timestamp_ns`。

```python
# slope_sim/lidar_pointcloud.py
@dataclass(frozen=True, slots=True)
class LidarScanResult:
    message: LidarPointCloud
    top_view: LidarTopViewFrame


def scan(self, timebase_ns: int) -> LidarPointCloud:
    message, _ = self._scan_message_at_mount(
        timebase_ns,
        self._world_mount(),
        capture_world_points=False,
    )
    return message


def scan_with_top_view(self, timebase_ns: int) -> LidarScanResult:
    message, accepted_world_hits = self._scan_message_at_mount(
        timebase_ns,
        self._world_mount(),
        capture_world_points=True,
    )
    # 只有 GUI Dashboard 路径再把已接受命中投影到 base_link。
    ...
```

生产 `PyBulletSensorBackend` 暴露 `ray_test_indexed_hits()`，直接省略 miss 并保留原射线索引；`MultiLineLidar` 对该紧凑结果使用 `inverse_transform_points_prevalidated()`，异常扩展类型仍回退完整严格校验。`InterfaceRuntime(capture_lidar_top_view=False)` 的 DIRECT/headless 路径只调用 `scan()`，不读取 `base_link` 或构造俯视副本；GUI Dashboard 才调用 `scan_with_top_view()`。

- [x] **Step 6: 实现 runtime 原子最新值和组合快照**

`InterfaceRuntime` 新增 `_latest_dashboard_payloads`、`_latest_lidar_views`、`_last_wheel_command` 和 `_last_wheel_command_received_sim_time_ns`。有效命令在 mailbox 成功提交且 generation 仍匹配时，将命令与当前 `_clock.now_ns` 原子保存；输出在 `_publish_message(topic, message, timestamp_ns, generation) is True` 后保存。将状态构造提取为仅在持锁时调用的 `_status_snapshot_locked(captured_at, transport_snapshot)`，让 `status_snapshot()` 和 `dashboard_snapshot()` 复用且不嵌套获取 `Condition`。

```python
def dashboard_snapshot(self, wall_time: float | None = None) -> InterfaceDashboardSnapshot:
    with self._condition:
        captured_at = self._monotonic() if wall_time is None else wall_time
        status = self._status_snapshot_locked(captured_at, self._transport.snapshot())
        return InterfaceDashboardSnapshot(
            generation=self._lifecycle_generation,
            robot_model=self._robot_model.name,
            sim_time_ns=self._clock.now_ns,
            status=status,
            wheel_command=self._last_wheel_command,
            wheel_command_received_sim_time_ns=self._last_wheel_command_received_sim_time_ns,
            wheel_state=self._latest_dashboard_payloads.get(self._config.wheel_state.topic),
            lidar_front=self._latest_dashboard_payloads.get(self._config.lidar_front.topic),
            lidar_rear=self._latest_dashboard_payloads.get(self._config.lidar_rear.topic),
            rtk=self._latest_dashboard_payloads.get(self._config.rtk.topic),
            imu=self._latest_dashboard_payloads.get(self._config.imu.topic),
            lidar_front_view=self._latest_lidar_views.get(self._config.lidar_front.topic),
            lidar_rear_view=self._latest_lidar_views.get(self._config.lidar_rear.topic),
        )
```

所有 prepare/commit/rollback/disconnect/close 清理路径通过一个 `_clear_dashboard_payloads_locked()` 统一清空，不增加新的锁。

- [x] **Step 7: 运行聚焦测试和既有接口回归**

```bash
conda run -n slope-sim python -m pytest tests/test_interface_dashboard_snapshot.py tests/test_lidar_pointcloud.py tests/test_lidar_pointcloud_direct.py tests/test_interface_runtime.py tests/test_interface_runtime_integration.py tests/test_interface_pause_rebuild.py -q
git diff --check
```

Expected: 全部 PASS；企业 Protobuf descriptor 与 payload 不增加字段。

- [x] **Step 8: 完成 Task 13A 规格审查和代码质量审查**

先由独立只读线程逐条核对冻结边界、成功发布语义、LiDAR 坐标和 generation 清理；通过后再启动第二个只读线程检查锁顺序、重复变换、点云复制、类型窄化和测试稳定性。任何 Important 结论先补红灯测试再修复。

性能复验后的修订：生产 runtime 仍在每个扫描 deadline 用一次 `rayTestBatch` 生成 2880-ray 原子点云，保持阶段三“单发布时刻、无运动畸变”合同。热路径改用紧凑 indexed hit 和预验证批量逆变换，headless 不构造 Dashboard 俯视副本；前后 50 ms 相位及 100 ms 消息时间戳周期不变，不以跨物理帧拼接世界状态换取墙钟性能。

---

## Task 13B：15 个默认一级页签与低密度图表

**Files:**

- Create: `slope_sim/dashboard_charts.py`
- Modify: `slope_sim/dashboard.py`
- Modify: `slope_sim/manual_demo.py`
- Test: `tests/test_dashboard_charts.py`
- Test: `tests/test_dashboard.py`
- Test: `tests/test_dashboard_enterprise.py`
- Test: `tests/test_manual_demo.py`

- [x] **Step 1: 写精确页签、标题和开发者诊断边界失败测试**

```python
EXPECTED_DEFAULT_TABS = [
    "接口状态", "障碍物", "轨迹", "速度/命令",
    "驱动命令", "驱动反馈", "转向命令", "转向反馈",
    "LiDAR点云", "RTK位置", "RTK航向", "IMU姿态",
    "轮组频率", "传感频率", "接口异常",
]


def test_default_dashboard_exposes_all_fifteen_enterprise_tabs(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(interface_config=InterfaceConfig.default())
    try:
        assert dashboard.window.windowTitle() == "3D仿真Dashboard"
        assert [dashboard.tabs.tabText(i) for i in range(dashboard.tabs.count())] == EXPECTED_DEFAULT_TABS
        assert dashboard.tabs.usesScrollButtons()
        assert dashboard.diagnostic_tabs is None
        assert all(dashboard.tabs.isAncestorOf(canvas) for canvas in dashboard.plot_canvases.values())
    finally:
        dashboard.close()


def test_developer_diagnostics_adds_only_internal_page_without_duplicate_plots(monkeypatch):
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    dashboard = TelemetryDashboard(developer_diagnostics_enabled=True)
    try:
        assert tab_names(dashboard) == [*EXPECTED_DEFAULT_TABS, "开发者诊断"]
        assert not any(
            dashboard.diagnostic_tabs is not None and dashboard.diagnostic_tabs.isAncestorOf(canvas)
            for canvas in dashboard.plot_canvases.values()
        )
    finally:
        dashboard.close()
```

- [x] **Step 2: 写纯图表缓存的时间、代际和低密度失败测试**

```python
def test_interface_chart_buffer_deduplicates_messages_and_clears_on_generation_change():
    buffer = InterfaceChartBuffer(window_sec=20.0, interface_config=InterfaceConfig.default())
    first = full_dashboard_snapshot(generation=1, sim_time_ns=1_000_000_000)
    assert buffer.append(first) == {
        "驱动命令", "驱动反馈", "转向命令", "转向反馈",
        "RTK位置", "RTK航向", "IMU姿态", "轮组频率", "传感频率", "接口异常",
    }
    assert buffer.append(first) == set()
    quality_only = replace(
        first,
        status=replace(first.status, captured_at=first.status.captured_at + 0.2),
    )
    assert buffer.append(quality_only) == {"轮组频率", "传感频率", "接口异常"}
    replacement = full_dashboard_snapshot(generation=2, sim_time_ns=2_000_000_000)
    buffer.append(replacement)
    assert buffer.series("驱动反馈")["t"] == [2.0]


def test_interface_chart_specs_keep_each_tab_within_confirmed_line_density():
    specs = interface_chart_specs(get_robot_model("active_steering_4wd"))
    counts = {spec.tab_label: len(spec.lines) for spec in specs}
    assert counts == {
        "驱动命令": 4, "驱动反馈": 4,
        "转向命令": 2, "转向反馈": 2,
        "RTK位置": 3, "RTK航向": 1, "IMU姿态": 2,
        "轮组频率": 2, "传感频率": 4, "接口异常": 2,
    }


def test_interface_error_rate_resets_baseline_without_negative_spike():
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    buffer.append(status_snapshot_at(10.0, errors=5, drops=3, generation=1))
    buffer.append(status_snapshot_at(11.0, errors=7, drops=4, generation=1))
    assert buffer.series("接口异常")["errors_per_sec"] == [0.0, 2.0]
    buffer.append(status_snapshot_at(12.0, errors=0, drops=0, generation=2))
    assert buffer.series("接口异常")["errors_per_sec"] == [0.0]
```

补充 20 秒边界、逆序时间、NaN、四车型 2/4 驱动轮、0/2 转向轮和暂停业务冻结测试。

- [x] **Step 3: 写仅可见页绘制和 LiDAR artist 复用失败测试**

```python
def test_hidden_tabs_buffer_data_but_only_active_plot_draws(dashboard, complete_dashboard_snapshot):
    dashboard.tabs.setCurrentIndex(tab_index(dashboard, "接口状态"))
    dashboard.update_interface_snapshot(complete_dashboard_snapshot)
    assert dashboard.interface_plot_buffer.series("RTK位置")["t"]
    assert all(canvas.draw_idle.call_count == 0 for canvas in dashboard.plot_canvases.values())

    dashboard.tabs.setCurrentIndex(tab_index(dashboard, "RTK位置"))
    dashboard.process_events()
    assert dashboard.plot_canvases["RTK位置"].draw_idle.call_count == 1


def test_lidar_tab_reuses_one_path_collection_and_updates_latest_offsets(dashboard):
    dashboard.tabs.setCurrentIndex(tab_index(dashboard, "LiDAR点云"))
    collection = dashboard.lidar_collection
    dashboard.update_interface_snapshot(snapshot_with_lidar_points(((1.0, 2.0, 2, 1),)))
    dashboard.update_interface_snapshot(snapshot_with_lidar_points(((3.0, 4.0, 3, 2),)))
    assert dashboard.lidar_collection is collection
    assert collection.get_offsets().tolist() == [[3.0, 4.0]]
```

- [x] **Step 4: 运行红灯**

```bash
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest tests/test_dashboard_charts.py tests/test_dashboard.py tests/test_dashboard_enterprise.py tests/test_manual_demo.py -q
```

Expected: FAIL，当前默认仅有两个页签，旧图仍嵌套在开发者诊断，且没有接口图表缓冲或 LiDAR artist。

- [x] **Step 5: 创建纯 Python 图表模块**

`dashboard_charts.py` 承担 `TelemetryPlotBuffer`、默认保留的轨迹/速度两张旧图规格、`InterfaceChartBuffer`、接口图规格、20 秒裁剪、消息时间戳去重、generation 清理和错误/丢帧增量。`dashboard.py` 从该模块导入并重新导出旧公共名称，避免既有测试和调用方断裂；接触/打滑只保留在显式开发者诊断表中。

```python
@dataclass(frozen=True, slots=True)
class ChartLineSpec:
    key: str
    label: str


@dataclass(frozen=True, slots=True)
class InterfaceChartSpec:
    tab_label: str
    title: str
    x_label: str
    y_label: str
    lines: tuple[ChartLineSpec, ...]


class InterfaceChartBuffer:
    def append(self, snapshot: InterfaceDashboardSnapshot, *, paused: bool = False) -> set[str]:
        """按消息时间戳去重并返回数据发生变化的页签。"""

    def series(self, tab_label: str) -> dict[str, list[float]]:
        """返回指定页可直接交给 Matplotlib 的冻结字段副本。"""

    def clear(self) -> None:
        """清空业务、质量和计数基线。"""
```

- [x] **Step 6: 把 13 个默认图表迁到顶层标签栏**

构造顺序严格使用 `EXPECTED_DEFAULT_TABS`。旧图只默认创建 `轨迹`、`速度/命令`；`开发者诊断` 显式开启时创建含接触/打滑的内部遥测表和调参控件。`_active_plot_label()` 从 `self.tabs` 读取，`currentChanged` 只标记新当前图。折线页共用 `_add_line_plot_tab()`，LiDAR 使用单独 `_add_lidar_plot_tab()`；每个 artist 只创建一次。

```python
def update_interface_snapshot(self, snapshot: InterfaceDashboardSnapshot) -> None:
    """渲染同代不可变快照，并只请求当前可见页绘制。"""
    if snapshot.generation != self._interface_generation:
        self.reset_interface_history(snapshot.generation, snapshot.robot_model)
    self.update_interface_status(snapshot.status)
    changed = self.interface_plot_buffer.append(snapshot, paused=self._paused)
    self._plot_dirty_tabs.update(changed)
    self._latest_lidar_views = (snapshot.lidar_front_view, snapshot.lidar_rear_view)
    self._request_current_plot_draw(time.monotonic())
```

默认标题改为 `3D仿真Dashboard`，`self.tabs.setUsesScrollButtons(True)`。折线页保留 Qt 标准清空/保存图标和 tooltip；LiDAR 只保留保存。无转向车型在画布内显示状态文字并保持空 line data。

- [x] **Step 7: 接入主循环并清理重建历史**

`manual_demo.py` 的暂停和正常路径都只调用一次 `runtime.dashboard_snapshot()`，再传给 `dashboard.update_interface_snapshot()`；不先后获取两个可能跨 generation 的快照。车型、复位、场地或场景事务仍调用 `reset_feedback_history()`，该方法同时清空旧遥测和接口缓存。

```python
if dashboard is not None and interface_session is not None:
    dashboard.update_interface_snapshot(
        interface_session.runtime.dashboard_snapshot()
    )
```

- [x] **Step 8: 运行聚焦 Qt、主循环和性能契约测试**

```bash
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest tests/test_dashboard_charts.py tests/test_dashboard.py tests/test_dashboard_enterprise.py tests/test_manual_demo.py tests/test_interface_runtime_integration.py -q
git diff --check
```

Expected: 全部 PASS；测试断言隐藏页 `draw_idle` 为 0、当前页不超过 2 Hz、点云对象身份不变。

- [x] **Step 9: 完成 Task 13B 规格审查和代码质量审查**

规格审查逐项核对标题、15 个默认页顺序、13 个默认图表、接触/打滑诊断边界、时间语义、暂停和保存/清空；质量审查重点检查 240 Hz 深拷贝、Matplotlib artist 泄漏、图例越界、动态车型标签、重复缓冲和 Qt 全局事件过滤器清理。发现问题先补红灯再修复。

---

## Task 13C：真实窗口、页签遍历与持续驾驶门禁

**Files:**

- Modify: `scripts/verify_dashboard_manual_drive.py`
- Modify: `slope_sim/window_layout.py`
- Modify: `slope_sim/dashboard.py`
- Modify: `slope_sim/manual_demo.py`
- Test: `tests/test_dashboard_manual_verifier.py`
- Test: `tests/test_window_layout.py`
- Test: `tests/test_manual_demo.py`
- Test: `tests/test_dashboard.py`
- Test: `tests/test_dashboard_enterprise.py`

- [x] **Step 1: 写真实页签遍历、标题和像素门禁失败测试**

```python
def test_verifier_cycles_all_fifteen_tabs_and_rejects_blank_capture(fake_x11, fake_capture):
    fake_x11.windows = dashboard_and_pybullet_windows()
    fake_capture.frames = [nonblank_frame()] * 14 + [blank_frame()]
    result = verify_dashboard_tabs(
        display=":99",
        expected_title="3D仿真Dashboard",
        tab_count=15,
        hold_drive_sec=4.0,
    )
    assert not result.passed
    assert result.visited_tabs == 15
    assert "tab 15" in result.detail


def test_verifier_rejects_old_dashboard_title(fake_x11):
    fake_x11.windows = windows_with_dashboard_title("企业仿真 Dashboard")
    with pytest.raises(DashboardVerificationError, match="3D仿真Dashboard"):
        find_dashboard_window(fake_x11)


def test_pybullet_lookup_requires_title_and_xres_process_owner(fake_x11):
    foreign, owned = fake_x11.same_title_windows(pids=(9001, os.getpid()))
    assert find_pybullet_window(fake_x11, expected_pid=os.getpid()) == owned
    fake_x11.xres_pid_by_window.pop(owned)
    with pytest.raises(WindowLayoutError, match="process ownership"):
        find_pybullet_window(fake_x11, expected_pid=os.getpid())


def test_dashboard_construction_failure_is_fatal_when_dashboard_was_requested(monkeypatch):
    monkeypatch.setattr(manual_demo, "TelemetryDashboard", raising_dashboard_constructor)
    with pytest.raises(RuntimeError, match="dashboard construction failed"):
        run_manual_demo(config_with_dashboard_enabled(), duration_limit_sec=0.01)


def test_verifier_forwards_custom_log_directory(tmp_path):
    command = build_child_command(verifier_args(log_dir=tmp_path))
    assert command[-2:] == ["--log-dir", str(tmp_path)]


def test_real_layout_report_rejects_overlapping_top_and_control_areas():
    report = dashboard_layout_report(
        tabs_rect=(0, 40, 384, 410),
        controls_rect=(0, 430, 384, 650),
        critical_controls={"暂停": (8, 440, 80, 24)},
    )
    assert report.passed
    assert not replace(report, controls_rect=(0, 400, 384, 650)).passed
```

像素门禁对每次 Dashboard client-area 截图计算至少两个通道的 `max-min` 与非背景像素比例；阈值通过合成纯色、文字页、折线页和点云页夹具固定，不能只检查文件非空。

- [x] **Step 2: 运行红灯**

```bash
conda run -n slope-sim python -m pytest tests/test_dashboard_manual_verifier.py tests/test_window_layout.py tests/test_dashboard.py tests/test_dashboard_enterprise.py -q
```

Expected: FAIL，当前 verifier 不遍历 15 个默认页，也未锁定新标题和画布像素。

- [x] **Step 3: 扩展真实 verifier**

使用现有 X11 窗口查找和外框读回，并新增基于 `libXRes` 的 client PID 查询：候选必须同时满足精确 PyBullet 标题、属于启动后新增窗口、XRes client PID 等于当前进程；窗口管理器 frame 先解析到 client 再查询。`_NET_WM_PID` 可作为一致性证据，但缺失时不得退化为纯标题；XRes 扩展不可用或所有权不一致时明确失败。找到后立即把 Main client 标题改为本次运行唯一 token，后续 verifier 只跟踪该已认领窗口。

Dashboard 构造、显示、frame extents 或矩形应用在 `dashboard_enabled=True` 时任一失败都清理并终止本次手动仿真，不再扩展 Main GUI 后静默继续。67:33 只适用于 `--gui --manual` 且 Dashboard 启用的工作流；非手动 `--gui` 保持现有批量实验窗口语义，并在设计/README 明确该范围。

激活 Dashboard 后先实际点击页签栏右滚动按钮，再有界重复点击左滚动按钮直到截图恢复；随后用冻结索引的 `Ctrl+Tab` 顺序循环两轮 15 页。每次等待 Qt 事件和 2 Hz 绘制冷却后，用 Pillow `ImageGrab.grab(bbox=client_rect, xdisplay=display)` 读取 client area，记录像素统计。验证进程通过专用环境变量请求 Dashboard 输出只读 JSON schema v4 布局报告；报告包含 DPR、页签顺序、左右按钮、page/canvas/axes/legend/plot buttons/content controls、title/xlabel/ylabel/offset/tick/legend artists、Qt 文字、关键控件的 viewport/scroll value，以及图表 `rendered_data_revision`。父 verifier 用物理 JSONL 行游标拒绝旧 occurrence，独立核对上下区 `1:1`、真实矩形 `contains`、artist-overlap 和 `axes_rect` 最小覆盖；第二轮要求数据修订增长且 `tabs/controls/page/canvas/axes` 五个矩形不变。任一页文字/控件越出 viewport，或轨迹 xlabel、科学计数 offset、tick、legend 和 Qt 文字互相遮挡均失败；完整图表按钮路径还需执行真实点击。该报告不改变正常 GUI，也不把 Qt 对象写入文件。

遍历期间使用键盘持续驾驶，结束后验证 `dx` 位移门禁。Dashboard 不再创建方向按钮，CLI 的 `--input-method` 只接受 `key`；线速度和角速度位于默认“仿真控制”区并必须完整进入 viewport。子进程在目标时长外预留找窗/报告预算，启动耗时不得侵占持续驾驶窗口。所有按键在 `finally` 中释放，关闭等待上限保持 20 秒。父 verifier 必须把 `--log-dir` 原样传给 `main.py`，避免读取其他运行留下的日志。

```python
@dataclass(frozen=True, slots=True)
class DashboardTabVerification:
    passed: bool
    visited_tabs: int
    min_nonbackground_ratio: float
    detail: str
```

- [x] **Step 4: 运行既有窗口回归和新增聚焦测试**

```bash
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest tests/test_window_layout.py tests/test_dashboard_enterprise.py tests/test_dashboard.py tests/test_manual_demo.py tests/test_dashboard_manual_verifier.py -q
git diff --check
```

Expected: 全部 PASS；既有 67:33/33%、DPR、Mutter frame、窗口标题冲突和关闭竞态用例不回归；新增用例证明纯标题候选不能被移动、Dashboard 硬失败不能降级、`--log-dir` 确实传入子进程。

- [ ] **Step 5: 运行真实桌面和三个 Xvfb 门禁**

```bash
DISPLAY=:1 XAUTHORITY=/home/cancade/.Xauthority conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --duration-sec 4
xvfb-run -a -s "-screen 0 1366x768x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --expected-available-size 1366x768 --duration-sec 4
xvfb-run -a -s "-screen 0 1920x1080x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --expected-available-size 1920x1080 --duration-sec 4
xvfb-run -a -s "-screen 0 2560x1440x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --expected-available-size 2560x1440 --duration-sec 4
```

Expected: 每次输出 XRes 所有权、available/main/dashboard 矩形、`tabs=15 nonblank=15` 和满足现有阈值的 `dx`；两窗覆盖工作区且目标宽度为 Main 67%、Dashboard 33%（67:33）。verifier 必须独立按 `33/100` 和 half-up/DPR 对齐规则计算唯一物理边界，不得复用生产布局 helper；总宽、公共边和外框必须精确。不得用 offscreen 单元测试替代。

- [x] **Step 6: 完成 Task 13C 规格审查和代码质量审查**

当前执行状态：schema v4 生产实现、聚焦测试和规格/质量审查已完成；Step 5 保持未勾选，因为当前 managed sandbox 无法连接宿主 X11，也不能创建临时 Xvfb socket。旧 schema v3/17 页数据只作为历史证据，不能替代四组 schema v4 实机门禁。

规格审查核对三分辨率、真实桌面、15 个默认页、诊断边界、非空和持续驾驶；质量审查核对 X11 frame/client 坐标、DPR、截图范围、按键释放、超时、临时文件和无窗口管理器路径。修复后重跑 Step 4-5。

---

## Task 14A：官方 eCAL 6.1 环境、逐话题 discovery 与真实仿真环回

**Files:**

- Modify: `environment.yml`
- Modify: `pyproject.toml`
- Modify: `scripts/generate_protos.py`
- Regenerate: `slope_sim/interfaces/generated/slope_sim_interfaces_pb2.py`
- Regenerate: `slope_sim/interfaces/generated/slope_sim_internal_pb2.py`
- Modify: `slope_sim/interfaces/transport.py`
- Modify: `slope_sim/interfaces/ecal_transport.py`
- Modify: `slope_sim/interfaces/runtime.py`
- Create: `scripts/ecal_simulation_runtime.py`
- Modify: `scripts/ecal_roundtrip_peer.py`
- Modify: `scripts/verify_ecal_roundtrip.py`
- Test: `tests/test_proto_contract.py`
- Test: `tests/test_ecal_installation.py`
- Test: `tests/test_ecal_transport.py`
- Test: `tests/test_ecal_process_roundtrip.py`
- Test: `tests/test_interface_runtime.py`

- [x] **Step 1: 写官方版本锁和真实导入失败测试**

```python
def flatten_conda_and_pip_dependencies(items):
    flattened = []
    for item in items:
        if isinstance(item, str):
            flattened.append(item)
        elif isinstance(item, dict):
            flattened.extend(item.get("pip", ()))
    return flattened


def test_environment_pins_one_ecal_compatible_protobuf_toolchain():
    environment = yaml.safe_load(Path("environment.yml").read_text())
    dependencies = flatten_conda_and_pip_dependencies(environment["dependencies"])
    assert "protobuf=6.33.6" in dependencies
    assert "grpcio-tools=1.76.0" in dependencies
    assert "eclipse-ecal==6.1.1" in dependencies


@pytest.mark.ecal
def test_official_ecal_611_bindings_import_with_project_protobuf_runtime():
    assert importlib.metadata.version("eclipse-ecal") == "6.1.1"
    assert google.protobuf.__version__ == "6.33.6"
    core = importlib.import_module("ecal.nanobind_core")
    proto_core = importlib.import_module("ecal.msg.proto.core")
    assert core.get_version_string().startswith("6.1.1")
    assert callable(proto_core.Publisher)
    assert callable(proto_core.Subscriber)
```

```bash
conda run -n slope-sim python -m pytest tests/test_ecal_installation.py tests/test_proto_contract.py -q
```

Expected: FAIL，当前环境仍为 Protobuf 7.35.1、无 `eclipse-ecal`，不能 skip 或用 PyPI `ecal==1.0.2` 替代。

- [x] **Step 2: 安装官方系统运行时并统一 Python 依赖**

```bash
sudo add-apt-repository -y ppa:ecal/ecal-6.1
sudo apt-get update
sudo apt-get install -y ecal=6.1.1-1ppa1~noble
dpkg-query -W -f='${Package} ${Version}\n' ecal

conda install -n slope-sim -c conda-forge -y \
  protobuf=6.33.6 grpcio-tools=1.76.0 packaging=26.2 pip
conda run -n slope-sim python -m pip install \
  --only-binary=:all: eclipse-ecal==6.1.1
```

`environment.yml` 固定 `protobuf=6.33.6`、`grpcio-tools=1.76.0`、`packaging=26.2`、`pip` 和 pip 子依赖 `eclipse-ecal==6.1.1`；`pyproject.toml` 固定 `protobuf>=6.33.6,<6.34`，dev 固定 `grpcio-tools==1.76.0`，接口可选依赖固定 `eclipse-ecal==6.1.1`。官方 wheel 为 `eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl`，不得使用 `--no-deps` 绕过兼容约束。

- [x] **Step 3: 重新生成 Protobuf 并证明 wire contract 未变**

```bash
conda run -n slope-sim python scripts/generate_protos.py
conda run -n slope-sim python -m pytest tests/test_proto_contract.py tests/test_interface_codec.py tests/test_interface_logging.py -q
conda run -n slope-sim python -c "import google.protobuf; import ecal.nanobind_core as e; print(google.protobuf.__version__, e.get_version_string())"
```

Expected: 输出 `6.33.6 6.1.1`；descriptor 的包名、字段名、编号、类型和序列化 round-trip 与 Task 1 完全一致。生成文件的 runtime version 检查必须由 1.76.0 工具产生，不手改。

- [x] **Step 4: 写官方 v6 resource 生命周期和逐话题 discovery 失败测试**

```python
def test_v6_bindings_use_nanobind_proto_api(fake_ecal_v6_modules):
    bindings = load_ecal_bindings(import_module=fake_ecal_v6_modules.import_module)
    assert bindings.api == "v6.1"
    publisher = bindings.create_publisher("/out", pb.WheelState)
    subscriber = bindings.create_subscriber("/in", pb.WheelCommand, record_payload)
    assert publisher.raw.constructor_args == (pb.WheelState, "/out")
    assert subscriber.raw.constructor_args == (pb.WheelCommand, "/in")
    assert subscriber.raw.receive_callback is not None


def test_simulation_role_reports_each_topic_peer_independently(fake_ecal_v6_bindings):
    transport = create_transport("ecal", bindings=fake_ecal_v6_bindings, role="simulation")
    fake_ecal_v6_bindings.command_subscriber.publisher_count = 1
    fake_ecal_v6_bindings.publishers["/sim/wheel/state"].subscriber_count = 1
    fake_ecal_v6_bindings.publishers["/sim/lidar/front/points"].subscriber_count = 0
    transport.poll_peer_state()
    quality = {item.topic: item for item in transport.snapshot().topic_quality}
    assert quality["/sim/wheel/command"].peer_connected is True
    assert quality["/sim/wheel/state"].peer_connected is True
    assert quality["/sim/lidar/front/points"].peer_connected is False


def test_runtime_does_not_apply_command_peer_state_to_output_topics(runtime_fixture):
    runtime_fixture.transport.set_topic_peers(command=True, wheel_state=True, lidar_front=False)
    runtime_fixture.runtime.poll_transport()
    snapshot = runtime_fixture.runtime.status_snapshot()
    assert snapshot.topics["/sim/wheel/command"].state == "active"
    assert snapshot.topics["/sim/wheel/state"].state == "active"
    assert snapshot.topics["/sim/lidar/front/points"].state == "waiting_peer"
```

- [x] **Step 5: 实现官方 eCAL 6.1 binding**

`load_ecal_bindings()` 首先且只支持已固定的官方 v6 路径：

```python
core = import_module("ecal.nanobind_core")
proto_core = import_module("ecal.msg.proto.core")
common_core = import_module("ecal.msg.common.core")
return EcalBindings.v61(core, proto_core, common_core)
```

初始化调用 `core.initialize(process_name)`，最终调用 `core.finalize()`。publisher 构造为 `proto_core.Publisher(MessageType, topic)` 并调用 `send(message)`；subscriber 构造为 `proto_core.Subscriber(MessageType, topic)` 并调用 `set_receive_callback(callback)`，回调从 `ReceiveCallbackData.message` 复制确定性 Protobuf bytes。关闭 subscriber 时先 `remove_receive_callback()`；v6 publisher/subscriber 没有虚构的 `close()`/`destroy()`，清空 Python 引用后再 finalize participant。部分构造失败仍按 subscriber 回调、资源引用、participant 的逆序清理。

- [x] **Step 6: 扩展传输快照和逐话题轮询**

`TransportTopicQuality` 增加 `peer_connected: bool | None = None`。本地模式保持 `None`；真实 eCAL simulation role 对命令 subscriber 调用 `get_publisher_count()`，对五个输出 publisher 分别调用 `get_subscriber_count()`。peer role 使用相反端点。轮子命令对端仍独立驱动 mailbox generation；runtime 只用每个质量项自身的 peer 值把对应话题标为 `waiting_peer`，不能用全局 `_peer_state` 覆盖五个输出。

新 production session 的 relay attach 和周期刷新都固定为先 `poll_peer_state()`、再读取 `snapshot()`；attach 的 poll 位于 relay 非重入锁外，允许同步 callback。transport 使用独立 discovery gate、in-flight 计数和递增 revision，迟到旧观察不能回退状态或误增 generation；`close()` 在移除 callback、释放 native 资源和 finalize participant 前等待全部在途 count API 返回。

```python
def _resource_peer_connected(resource: _ProtoResource) -> bool:
    count_method = (
        resource.raw.get_publisher_count
        if resource.direction == "subscriber"
        else resource.raw.get_subscriber_count
    )
    count = count_method()
    if isinstance(count, bool) or not isinstance(count, int) or count < 0:
        raise RuntimeError("eCAL peer count must be a nonnegative integer")
    return count > 0
```

- [x] **Step 7: 写真实 PyBullet runtime 环回红灯并实现 simulation 模式**

```python
@pytest.mark.ecal
def test_real_ecal_simulation_runtime_uses_physics_feedback_and_all_six_topics():
    result = run_ecal_process_roundtrip(
        runtime="simulation",
        warmup_sec=1.0,
        duration_sec=5.0,
    )
    assert result.transport_name == "ecal"
    assert result.runtime_name == "simulation"
    assert result.feedback_is_not_command_echo
    assert result.per_topic_peer_states == {topic: "active" for topic in DEFAULT_TOPICS}
    assert result.timeout_stopped_vehicle
    assert result.reconnect_required_new_command
```

`scripts/ecal_simulation_runtime.py` 在独立进程使用 PyBullet DIRECT、正式 `SimulationCoordinator`、`PyBulletSensorBackend` 和 `InterfaceRuntime`，按 240 Hz 推进真实物理，不生成合成输出。前后 2880 射线错相 50 ms，共享 100 ms 消息时间戳周期；每个扫描 deadline 用单次批量射线生成同一发布时刻的完整点云。headless runtime 不构造俯视副本，绝对期限追赶的超期帧只用 `sleep(0)` 让出执行权，并只在实时循环期间暂停 cyclic GC。自动、GUI 手动和独立 eCAL 三条入口复用 `RuntimeObservationCadence`，native discovery 与组合状态快照为 20 Hz；慢 poll 后重建期限且不突发追赶，但每个 240 Hz 帧仍使用新墙钟检查命令超时。`verify_ecal_roundtrip.py` 增加 `--runtime {transport,simulation}`、`--warmup-sec` 和 `--robot-model`；simulation 模式由 peer 按当前车型发送有效/非法/静默/重连命令并订阅五个输出。正式路径分别覆盖主动转向 `4+2` 和代表性差速 `2+0`，要求 peer/runtime 车型、命令事件基数和真实关节反馈一致，同时验证前后点云/RTK/IMU、六话题频率和逐话题 subscriber 退出状态。

- [ ] **Step 8: 运行官方 eCAL 聚焦门禁**

```bash
conda run -n slope-sim python -m pytest \
  tests/test_ecal_installation.py tests/test_ecal_transport.py \
  tests/test_ecal_process_roundtrip.py tests/test_interface_runtime.py \
  -q -m "ecal or not ecal"
conda run -n slope-sim python scripts/verify_ecal_roundtrip.py \
  --runtime simulation --robot-model active_steering_4wd --warmup-sec 1 --duration-sec 5
conda run -n slope-sim python scripts/verify_ecal_roundtrip.py \
  --runtime simulation --robot-model df_back --warmup-sec 1 --duration-sec 5
git diff --check
```

Expected: 测试全部 PASS；脚本输出 `runtime=simulation transport=ecal`、六话题频率、逐话题 peer 状态、超时和重连全部 PASS。任何 local fallback、合成输出或 skip 都是失败。

- [x] **Step 9: 完成 Task 14A 规格审查和代码质量审查**

当前执行状态：官方 binding、逐话题 discovery、active `4+2` 与 differential `2+0` 双进程链路及纯测试已完成并通过独立审查；Step 8 保持未勾选。2026-07-29 获授权的一次 post-fix 执行在 discovery 前被 Codex 沙箱禁止 UDP socket，未形成有效产品结论；下一次真实运行仍需重新授权并在允许 socket 的环境中严格串行执行。

规格审查核对官方版本、wire contract、六话题真实进程、逐话题 discovery 和物理反馈；质量审查检查 Protobuf ABI、v6 回调复制、资源析构、finalize 顺序、peer count 异常、进程超时和临时文件。修复后重跑 Step 3、8。

---

## Task 14B：阶段三验收、全量回归、六维独立审查与交付

**Files:**

- Create: `scripts/verify_stage3_interfaces.py`
- Create: `tests/test_stage3_interface_verifier.py`
- Create: `docs/阶段三交付报告.md`
- Modify: `README.md`
- Modify: `3d仿真平台需求规格.md`

- [x] **Step 1: 写验收脚本聚合和非零退出失败测试**

```python
# 阶段三验收脚本测试：任何门禁失败都必须进入汇总并返回非零。
from scripts.verify_stage3_interfaces import VerificationCheck, exit_code, summarize


def test_stage3_summary_counts_pass_and_fail_and_returns_nonzero():
    checks = (
        VerificationCheck("wheel_rates", True, "100.0 Hz"),
        VerificationCheck("lidar_front", False, "9.1 Hz"),
    )
    assert summarize(checks).final_line == "SUMMARY pass=1 fail=1"
    assert exit_code(checks) == 1
```

- [x] **Step 2: 实现可独立运行的 DIRECT/性能验收脚本**

`scripts/verify_stage3_interfaces.py` 必须逐项输出 `PASS/FAIL name detail` 并覆盖：

```python
def run_stage3_checks() -> tuple[VerificationCheck, ...]:
    return (
        run_proto_and_topic_contract_check(),
        *run_four_model_wheel_checks(),
        run_timeout_and_steering_hold_check(),
        run_100_10_hz_scheduler_check(),
        *run_three_terrain_lidar_checks(),
        run_static_and_moving_obstacle_lidar_check(),
        run_lidar_collision_contact_check(),
        *run_three_terrain_truth_sensor_checks(tolerance=1e-4),
        run_pause_rebuild_and_edge_switch_check(),
        run_scene_roundtrip_check(),
        run_interface_log_roundtrip_check(),
        run_dashboard_snapshot_and_chart_check(),
        run_per_topic_ecal_status_check(),
        run_twenty_obstacle_queue_performance_check(max_dashboard_gap_sec=0.100),
    )
```

DIRECT 调度频率按 10 秒仿真时间的消息计数/时间戳检查：轮子 1000 帧、每个传感器 100 帧。20 个障碍物性能门禁使用 local transport 与生产 runtime/logger wiring，预热 1 秒后连续运行 5 秒墙钟；它验证物理、传感器、Dashboard 快照和接口日志联合负载，但不能替代真实 eCAL。每 100 ms 同时采样日志 `pending` 和 `completed=accepted-pending`：首次实际增长起连续 1 秒不下降，或正深度下完成数停滞 1 秒，判为持续积压；稳定单项 in-flight 但完成数前进不误报。accepted 消息数至少达到六通道名义总量的 90%，终态 pending、传输/日志 dropped 必须为 0，Dashboard 单次事件间隔不得超过 100 ms。后两条真实 eCAL production 命令各自在同一测量窗口绑定 20 个障碍物、六话题、物理反馈和接口日志。

- [ ] **Step 3: 运行阶段三 DIRECT 与真实 eCAL 门禁**

```bash
conda run -n slope-sim python scripts/verify_stage3_interfaces.py
conda run -n slope-sim python scripts/verify_ecal_roundtrip.py --runtime simulation --robot-model active_steering_4wd --warmup-sec 1 --duration-sec 5
conda run -n slope-sim python scripts/verify_ecal_roundtrip.py --runtime simulation --robot-model df_back --warmup-sec 1 --duration-sec 5
```

Expected: 第一条最终输出匹配 `^SUMMARY pass=[1-9][0-9]* fail=0$` 且退出码 0，测试同时断言 pass 数等于实际检查 tuple 长度；后两条必须输出 `runtime=simulation transport=ecal` 并分别证明 `4+2`、`2+0`。真实 eCAL 接收进程在 1 秒 discovery 预热后，用每条到达消息的单调墙钟同时逐话题计算：轮子命令和状态 `95..105 Hz`，前后点云、RTK、IMU 各 `9..11 Hz`；消息自身仿真时间戳也分别为 100/10 Hz，`sim/wall` 必须在 `0.98..1.02`。非法命令、停止发送、逐输出 subscriber 退出、命令对端退出/重启、物理反馈变化和“重连后旧命令不恢复”全部 PASS。真实 eCAL 失败时阶段三不得声明完成。

- [ ] **Step 4: 运行 GUI 窗口门禁和完整回归**

```bash
DISPLAY=:1 XAUTHORITY=/home/cancade/.Xauthority conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --duration-sec 4
xvfb-run -a -s "-screen 0 1366x768x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --expected-available-size 1366x768 --duration-sec 4
xvfb-run -a -s "-screen 0 1920x1080x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --expected-available-size 1920x1080 --duration-sec 4
xvfb-run -a -s "-screen 0 2560x1440x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --verify-window-layout --verify-dashboard-tabs --expected-available-size 2560x1440 --duration-sec 4
conda run -n slope-sim python scripts/verify_stage1_matrix.py
conda run -n slope-sim python scripts/verify_stage2_obstacles.py
conda run -n slope-sim python -m pytest -q -m "not ecal"
git diff --check
```

Expected: 真实桌面和三个 Xvfb 均报告 XRes 所有权、67:33（Dashboard 33%）几何、15 个默认页签非空及驾驶位移 PASS；阶段一 12 项矩阵、阶段二全部门禁、阶段三及全量 pytest 均无失败；`git diff --check` 无输出。

- [x] **Step 5: 做六维独立只读审查**

启动独立审查线程，只读比较两个原始需求、阶段三设计、提交范围、全部测试输出和实际 GUI/eCAL 结果，不直接修改代码。审查报告按以下六节给出文件/行号和严重级别：

1. 需求完整性。
2. 逻辑正确性。
3. 边界情况。
4. 代码质量。
5. 测试覆盖。
6. 实际运行结果。

任何 `Critical`/`Important` 结论由实施线程先补失败测试再修复，随后重跑对应聚焦门禁、全量 pytest 和六维复审；不能通过降低 100/10 Hz、100 ms、`1e-4`、67:33/33% 或性能阈值关闭问题。

当前执行状态：最终六维只读复审结论为 `Critical=0`、`Important=0`；上一轮唯一 Important（文档仍把旧测试数量标为 current/fresh）已通过同步 `272/365/2209` 等本轮证据关闭。复审保留的两个局部 oracle 问题均为 Minor，已列入交付报告残余风险，不影响外层正式门禁拒绝假通过。

- [x] **Step 6: 更新文档和交付报告**

`README.md` 增加环境创建、Protobuf 生成、`auto/ecal/local`、场景导入导出、阶段三验收和 GUI 启动命令。当前 schema v4 GUI 尚未补证、post-fix eCAL 又只形成环境阻断证据时，`3d仿真平台需求规格.md` 和交付文档只允许写“开发方自动复验中”，不得提前标记开发方或用户验收通过。

`docs/阶段三交付报告.md` 必须包含：

- 需求到实现/测试的追踪表。
- 精确变更列表和文件边界；若用户尚未要求提交，记录 base HEAD 和全部未提交文件，不伪造提交列表。
- Protobuf/eCAL 版本及安装命令。
- 阶段一、二、三、全量 pytest、真实 eCAL 和 GUI 几何的实际输出摘要。
- 用户人工验收步骤：eCAL Monitor 六话题、两车型命令、前后障碍点云、三地形 RTK/IMU、暂停/复位/重建/断线、企业 Dashboard 和 67:33 窗口。
- 已知限制：真值无噪声；点云按发布时刻一次采样，`offset_time_ns` 只表达射线顺序，不模拟真实雷达的自运动、运动畸变、回波或光电效应；本地模式不是正式 eCAL；阶段四导航未实现。
- 面向 PyBullet 初学者说明 `getLinkState`/坐标变换、`rayTestBatch`、碰撞 group/mask 和主线程隔离。

当前执行状态：README、权威需求规格、阶段三设计/实施计划和交付报告已同步本轮 fresh 非 eCAL 证据、post-fix eCAL socket 环境阻断、schema v4 GUI X11 环境阻断、用户手动教程及残余风险。Step 3、Step 4 继续保持未勾选；文档同步不能替代外部门禁。

- [ ] **Step 7: 准备提交范围并停止在阶段三人工验收门禁**

```bash
git status --short
git diff --check
```

Expected: 向用户报告完整待提交文件、测试证据和 GUI/eCAL 操作步骤后停止，不开始阶段四。只有用户明确要求 commit/push 后，才按 AGENTS.md 的阶段三提交格式执行 Git 写入。

当前执行状态：仍停在 Step 7 之前。需要先取得两车型 post-fix 真实 eCAL 有效结果、四组 schema v4 GUI 结果和用户人工验收反馈；中间状态按用户要求不提交，不开始阶段四。
