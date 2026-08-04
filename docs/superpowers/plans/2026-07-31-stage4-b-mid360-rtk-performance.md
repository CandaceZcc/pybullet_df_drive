# 阶段四 B：MID-360、三点 RTK 与性能 Implementation Plan

> **Execution:** Use `subagent-driven-development` only when the user selects delegated execution; otherwise use `executing-plans`. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用一个车体几何中心 MID-360 风格 LiDAR 和固定 LEFT/CENTER/RIGHT 三点 RTK 替换阶段三双雷达/双天线语义，并在不降低 240/100/10 Hz 业务预算的前提下收敛 GUI、高尔夫地形和 Dashboard 卡顿。

**Architecture:** 阶段 A 的 v2 descriptor/session/raw transport 是本计划的前置条件；v1 传感器读取器保持显式兼容，但阶段四生产路径先补齐 LiDAR/RTK/IMU 不可变 v2 模型、严格 codec 和独立五通道 `V2RuntimeConfig`，再调用 A 的 `create_v2_ecal_transport()`，不得把阶段三双 LiDAR `InterfaceConfig` 带入 v2 runtime。物理主线程在同一仿真时刻生成原子消息，确定性编码一次后把同一 bytes 交给日志和 transport，Dashboard 只消费有界显示副本；性能优化先用分段计时证明瓶颈，再固定相机、Qt、图表和点云显示预算，碰撞 heightfield 默认不降采样。

**Tech Stack:** Python 3.10、PyBullet、NumPy、PySide6、Matplotlib、阶段四 Protobuf v2、pytest。

---

**TDD gate:** 本计划所有生产代码任务遵守总路线的严格 RED-GREEN-REFACTOR 协议；RED 必须是 pytest 正常收集后的行为断言失败，不能是 collection error、缺工具、skip 或缺构建目录。创建新模块的 Task 在 RED 测试中不得顶层导入尚不存在的模块：测试文件必须先正常收集，再在测试函数内用 `importlib.util.find_spec()`/`importlib.import_module()` 检查 wished-for API，并用带明确消息的 `assert`/`pytest.fail()` 把“API 或行为尚未实现”报告为 `FAILED`。每个 Task 的 GREEN 只写满足本 Task RED 的最小生产代码；REFACTOR 不新增行为，完成后原样复跑该 Task 的 GREEN 命令。

**环境合同前置：** 开始或恢复本计划前必须独立执行，不能依赖总计划或 A 所在 shell：

```bash
test -n "${STAGE4_BUILD_ENV_FILE:-}"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-env "$STAGE4_BUILD_ENV_FILE" \
  --json "$STAGE4_BUILD_ENV_FILE.stage4-b-preflight.json"
source "$STAGE4_BUILD_ENV_FILE"
test -x "$STAGE4_PROTOC"
test -d "$STAGE4_DEPENDENCY_PREFIX"
```

Expected: env/evidence hash、Protobuf 工具和开发 dependency prefix 与总计划 Task 2 冻结值一致；缺失、未定义或漂移时在 Task 1 创建输出前失败。

## 进入门槛

开始 Task 1 前，阶段 A 必须已经满足其完成定义：v1 descriptor 冻结、v2 descriptor/golden、五话题 raw transport、session/authority/rebuild controller、Python/C++ raw Phase-0 和同 topic v1 冲突硬拒绝均有 PASS 证据。若 A 因 metadata 或同 topic 隔离处于 `BLOCKED`，本计划不得先接正式 runtime，也不得擅自改为 `/sim/v2/...`；先把 A 的唯一真实运行证据交给用户裁决。

## Task 1：车型语义 link 与单中心 LiDAR URDF

**Files:**
- Modify: `slope_sim/model_registry.py`
- Create: `scripts/generate_stage4_robot_models.py`
- Generate: `resources/models/robot_models.yaml`
- Create: `urdf/stage4/df_front.urdf`
- Create: `urdf/stage4/df_mid.urdf`
- Create: `urdf/stage4/df_back.urdf`
- Create: `urdf/stage4/active_steering_4wd.urdf`
- Preserve: `urdf/df_front.urdf`
- Preserve: `urdf/df_mid.urdf`
- Preserve: `urdf/df_back.urdf`
- Preserve: `urdf/active_steering_4wd.urdf`
- Test: `tests/stage4/test_stage4_robot_models.py`
- Test: `tests/test_robot_models.py`
- Test: `tests/test_sensor_backend.py`
- Test: `tests/test_truth_sensors.py`

- [ ] **Step 1: 写四车型语义 RED**

```python
import pytest

from slope_sim.model_registry import get_robot_model


@pytest.mark.parametrize("name", ("df_front", "df_mid", "df_back"))
def test_differential_model_exposes_rtk_triplet_links(name: str) -> None:
    model = get_robot_model(name)
    assert model.rtk_left_link_names == ("left_drive_wheel",)
    assert model.rtk_right_link_names == ("right_drive_wheel",)


def test_four_wheel_model_exposes_side_pairs() -> None:
    model = get_robot_model("active_steering_4wd")
    assert model.rtk_left_link_names == (
        "front_left_drive_wheel",
        "rear_left_drive_wheel",
    )
    assert model.rtk_right_link_names == (
        "front_right_drive_wheel",
        "rear_right_drive_wheel",
    )


@pytest.mark.parametrize("name", ("df_front", "df_mid", "df_back", "active_steering_4wd"))
def test_stage4_and_legacy_v1_urdf_are_explicitly_separate(name: str) -> None:
    model = get_robot_model(name)
    assert model.stage4_urdf_path != model.urdf_path
    assert model.stage4_urdf_path.is_file()
    assert model.urdf_path.is_file()
    assert_single_stage4_lidar(model.stage4_urdf_path)
    assert_legacy_v1_lidar_mounts(model.urdf_path)


def test_canonical_robot_models_resource_matches_registry_and_urdf() -> None:
    document = load_stage4_robot_models("resources/models/robot_models.yaml")
    assert document.schema_version == 1
    assert tuple(document.models) == (
        "df_front", "df_mid", "df_back", "active_steering_4wd"
    )
    assert_canonical_models_match_registry_and_urdf(document)
```

同一测试只把 `stage4_urdf_path` 当作 v2 资源：要求恰好一个 `lidar_link`，其 fixed joint 为 `base_link -> lidar_link`、translation `(0,0,0.105)`、rotation identity，并确认其中不存在旧 `lidar_front_mount/lidar_rear_mount`。`urdf_path` 的四个阶段三资源必须继续包含两个旧 mount，`create_robot()` 的 v1 默认路径仍只读 `urdf_path`。测试还运行 generator 的 `--check` 模式，任何注册表、阶段四 URDF 与受版本控制 YAML 的字节差异都失败。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_robot_models.py`

Expected: 测试正常收集并 `FAILED`；断言明确指出当前车型缺少 RTK side link/`stage4_urdf_path`，而不是在 import 或 fixture setup 阶段报错。

- [ ] **Step 3: 扩展注册表而不硬编码 link index**

```python
@dataclass(frozen=True)
class RobotModelSpec:
    # 既有字段保持原顺序。
    stage4_urdf_path: Path | None = None
    rtk_left_link_names: tuple[str, ...] = ()
    rtk_right_link_names: tuple[str, ...] = ()
    axle_center_to_base_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def rtk_wheel_link_names(self) -> tuple[str, ...]:
        """按 LEFT 侧、RIGHT 侧稳定返回 RTK 几何使用的轮轴 link。"""
        return self.rtk_left_link_names + self.rtk_right_link_names
```

四车型实例显式填入上述 link 名、`stage4_urdf_path` 和 `axle_center_to_base_xyz`；加载时验证阶段四路径存在且不等于 legacy `urdf_path`、两侧非空且数量相同、无重复、外参有限，仍由 `PyBulletSensorBackend` 把语义名解析成当前 client 的 link index。`urdf_path` 的含义和所有 v1 调用点保持不变；只有 schema v2 的建车入口显式读取 `stage4_urdf_path`，禁止按文件名字符串猜版本。

`scripts/generate_stage4_robot_models.py` 是 `resources/models/robot_models.yaml` 的唯一生产者。它按上述四个 model id 固定顺序，从已验证注册表和 `stage4_urdf_path` 语义生成 schema v1 文档；每项精确包含 `model_id`、阶段四 `urdf`、`base_link`、左右 RTK link 名、`axle_center_to_base_xyz` 和 `base_to_lidar_xyz/rpy`。canonical 文件不得收录 legacy v1 URDF；后者只由现有 `urdf_path` 合同拥有。输出使用 UTF-8 LF、稳定键顺序、有限十进制表示和末尾单换行；`--check` 在不写文件的情况下逐 byte 比较。Python v2 runtime、C++ Export、ROS TF 和发行包只消费这一个 canonical 文件，不各自维护第二份车型外参。

- [ ] **Step 4: 从 legacy v1 机械资源派生四套独立阶段四 URDF**

```xml
<link name="lidar_link">
  <visual>
    <geometry><cylinder radius="0.04" length="0.055"/></geometry>
  </visual>
</link>
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.105" rpy="0 0 0"/>
</joint>
```

新建 `urdf/stage4/*.urdf` 时保留对应 legacy v1 文件的车体、轮组、质量、惯量、关节限位和 collision，只删除两个旧 LiDAR mount 后加入上述单中心 link。雷达 visual 不增加 collision；射线 self-filter 仍排除整台机器人所有 link。禁止修改四个 legacy v1 URDF；阶段三双雷达测试继续对它们运行。

- [ ] **Step 5: 运行 GREEN 与旧阶段回归**

Run: `conda run -n slope-sim python scripts/generate_stage4_robot_models.py --check --output resources/models/robot_models.yaml`

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_robot_models.py tests/test_robot_models.py tests/test_sensor_backend.py tests/test_truth_sensors.py tests/test_lidar_collision_filters.py`

Expected: generator byte-identical，测试 PASS；v2 四套 URDF 只有一个 `lidar_link`，v1 四套 URDF 仍有两个旧 mount；报告记录 YAML SHA-256，后续 C/D/E 使用同一摘要。

- [ ] **Step 6: REFACTOR 注册表校验和 URDF 解析重复代码**

只抽取 Task 1 已覆盖的路径/语义校验与测试 XML helper，不改变 `RobotModelSpec` 字段、canonical YAML 或两套 URDF 的选择规则。随后原样重跑 Step 5 的 generator `--check` 和完整 pytest 命令，Expected: PASS 且 YAML SHA-256 不变。

## Task 2：严格 schema v2 与 v1 显式转换

**Files:**
- Create: `slope_sim/scene_config_v2.py`
- Create: `scripts/convert_scene_v1_to_v2.py`
- Modify: `main.py`
- Test: `tests/stage4/test_scene_config_v2.py`
- Test: `tests/stage4/test_scene_v1_conversion.py`

- [ ] **Step 1: 写 schema/固定外参 RED**

`tests/stage4/test_scene_config_v2.py` 和 conversion 测试只在测试函数内加载 `slope_sim.scene_config_v2`；先用 `find_spec()` 断言模块存在，再调用 wished-for API。这样当前树会正常收集，并以“schema v2 API 尚未实现”的断言 `FAILED`，不会 collection error。

```python
def test_default_scene_v2_has_one_fixed_lidar_and_triplet_rtk() -> None:
    scene = default_scene_v2("df_back", "golf_heightfield")
    assert scene.schema_version == 2
    assert scene.sensors.lidar.frame_id == "lidar_link"
    assert scene.sensors.lidar.position_m == (0.0, 0.0, 0.105)
    assert scene.sensors.lidar.profile == "realtime_mid360"
    assert scene.sensors.rtk.geometry == "wheel_axle_triplet_v1"
```

再覆盖未知键、可修改外参、非法 profile、负 scan seed、重复 YAML key、过大文件、v1 静默读入 v2 runtime 和 round-trip hash 不稳定。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_scene_config_v2.py tests/stage4/test_scene_v1_conversion.py`

Expected: 正常收集后 `FAILED`；首个失败明确为 schema v2 API/转换行为缺失，不得是顶层 import error、fixture error 或 skip。

- [ ] **Step 3: 建立不可变 v2 sensor 文档**

```python
@dataclass(frozen=True)
class LidarDocumentV2:
    frame_id: str = "lidar_link"
    lidar_id: int = 1
    parent_link: str = "base_link"
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.105)
    orientation_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    profile: str = "realtime_mid360"
    scan_seed: int = 0
```

构造器逐字段拒绝与固定合同不同的 frame/id/parent/外参；reader 只接受设计稿列出的精确键集合，canonical dump 使用稳定键顺序和 UTF-8 LF，再计算 SHA-256。

- [ ] **Step 4: 实现显式 v1→v2 转换**

```python
def convert_scene_v1(document: SceneDocument) -> SceneDocumentV2:
    return SceneDocumentV2(
        robot_model=document.robot_model,
        terrain=document.terrain,
        obstacles=document.obstacles,
        sensors=SensorDocumentV2.default(),
    )
```

CLI 必须打印“前后两个 180° LiDAR 已替换为一个几何中心 360° LiDAR”；遇到 v1 未知字段或无法规范化值直接非零退出，输出使用临时文件、`fsync` 和原子替换。

- [ ] **Step 5: 运行 GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_scene_config_v2.py tests/stage4/test_scene_v1_conversion.py tests/test_scene_config.py tests/test_scene_config_atomic.py`

Expected: PASS，且 v1 reader 旧测试不变。

- [ ] **Step 6: REFACTOR schema 校验与 canonical I/O**

只合并 v2 reader/converter 内已经由 RED 覆盖的重复标量校验、canonical dump 和原子写入代码；不让 v1 reader 隐式接受 v2，也不改变 `SceneDocument.robot_model` 映射。随后原样重跑 Step 5，Expected: PASS 且同一输入的 canonical SHA-256 不变。

## Task 3：确定性 R2 MID-360 原子扫描

**Files:**
- Create: `slope_sim/mid360_lidar.py`
- Modify: `slope_sim/sensor_backend.py`
- Test: `tests/stage4/test_mid360_rays.py`
- Test: `tests/stage4/test_mid360_direct.py`
- Test: `tests/test_sensor_backend.py`

- [ ] **Step 1: 写 profile 和方向 oracle RED**

测试文件顶层只导入 pytest/标准库；在测试函数中通过通用 `_load_mid360_api()` 检查并加载新模块。缺模块或缺符号必须由 helper 调用 `pytest.fail("MID-360 API is not implemented", pytrace=False)`，确保 pytest 正常收集并把 RED 记录为 `FAILED`。

```python
from decimal import Decimal, ROUND_HALF_EVEN, localcontext


def _oracle_r2_q64_steps() -> tuple[int, int]:
    """测试侧用 Decimal 独立求塑性常数，不读取生产常量。"""
    with localcontext() as context:
        context.prec = 100
        plastic = Decimal("1.3")
        for _ in range(32):
            plastic -= (plastic**3 - plastic - 1) / (3 * plastic**2 - 1)
        q64 = Decimal(1 << 64)
        return (
            int((q64 / plastic).to_integral_value(rounding=ROUND_HALF_EVEN)),
            int(
                (q64 / (plastic * plastic)).to_integral_value(
                    rounding=ROUND_HALF_EVEN
                )
            ),
        )


def test_realtime_profile_is_frozen() -> None:
    profile = Mid360Profile.realtime()
    assert profile.ray_count == 5760
    assert profile.scan_period_ns == 100_000_000
    assert profile.min_elevation_deg == -7.0
    assert profile.max_elevation_deg == 52.0
    assert profile.range_m == (0.1, 40.0)


def test_r2_frame_is_deterministic_without_azimuth_seam_duplicate() -> None:
    first = build_mid360_candidates(Mid360Profile.realtime(), scan_seed=7, sequence=3)
    second = build_mid360_candidates(Mid360Profile.realtime(), scan_seed=7, sequence=3)
    assert first == second
    assert len(first) == 5760
    assert len({candidate.direction for candidate in first}) == 5760
    assert [p.offset_time_ns for p in first] == sorted(p.offset_time_ns for p in first)
    assert first[-1].offset_time_ns < 100_000_000


@pytest.mark.parametrize("sequence", (0, (1 << 64) - 1))
def test_r2_frame_keeps_5760_unique_directions_at_uint64_edges(sequence) -> None:
    points = build_mid360_candidates(
        Mid360Profile.realtime(), scan_seed=7, sequence=sequence
    )
    assert len({p.direction for p in points}) == 5760

    step_u, step_v = _oracle_r2_q64_steps()
    assert (step_u, step_v) == (
        13925035116211876495,
        10511698010929265437,
    )
    for index in (0, 1, 5759):
        k = 7 + sequence * 5760 + index
        expected = (
            (((1 << 63) + k * step_u) & ((1 << 64) - 1)) / (1 << 64),
            (((1 << 63) + k * step_v) & ((1 << 64) - 1)) / (1 << 64),
        )
        assert r2_sample(7, sequence, 5760, index) == expected
```

- [ ] **Step 2: 写独立 DIRECT 几何 oracle RED**

先创建 `tests/stage4/test_mid360_direct.py`，不得等扫描器实现后再补。文件顶层只导入标准库、pytest 和已有 PyBullet 依赖；测试函数先通过同一个 `_load_mid360_api()` 检查 wished-for API，缺模块或缺符号时调用 `pytest.fail("MID-360 API is not implemented", pytrace=False)`。API 存在后，在 PyBullet DIRECT 建地面、墙面和三个不同高度障碍；用独立少量 `rayTest` 对抽样方向核对命中点，误差 `<=1e-5 m`。断言 self-hit 为零、点全在 `lidar_link` 坐标、无 miss 占位点、`point_num == len(points)`，不复用生产候选、变换或命中解析 helper。

- [ ] **Step 3: 同时运行单元与 DIRECT RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_mid360_rays.py tests/stage4/test_mid360_direct.py`

Expected: 两个测试文件都正常收集并 `FAILED`，失败消息为 MID-360 profile/R2/扫描行为尚未实现；不得出现 collection error、fixture error、PyBullet 初始化错误或 skip。保存该失败输出后才可进入 Step 4。

- [ ] **Step 4: 实现独立确定性生成器**

```python
_Q64 = 1 << 64
_MASK64 = _Q64 - 1
_HALF_Q64 = 1 << 63
R2_STEP_U_Q64 = 13925035116211876495
R2_STEP_V_Q64 = 10511698010929265437


def r2_sample(scan_seed: int, sequence: int, ray_count: int, index: int) -> tuple[float, float]:
    k = scan_seed + sequence * ray_count + index
    u_phase = (_HALF_Q64 + k * R2_STEP_U_Q64) & _MASK64
    v_phase = (_HALF_Q64 + k * R2_STEP_V_Q64) & _MASK64
    return u_phase / _Q64, v_phase / _Q64
```

两个 Q64 步长由塑性常数的高精度倒数按 `ROUND_HALF_EVEN` 取最近整数得到且必须保持奇数；生产代码不得先把 `k` 或乘积转换为 float。测试 oracle 在测试文件内用 `Decimal` 独立解 `x^3=x+1` 并推导步长，不导入生产常量；除逐样本 phase 外，还锁定 `sequence=0/UINT64_MAX` 均有 5,760 个唯一方向。`u` 映射 `[0, 2*pi)`，`v` 映射 `[-7°,52°)`；`line=min(15, floor(16*v))`，offset 使用 `floor(i*100_000_000/N)`。所有有限值和 uint 边界在构造时校验。

- [ ] **Step 5: 实现一次冻结位姿、一次 batch 的协议无关扫描器**

```python
@dataclass(frozen=True)
class Mid360HitPoint:
    offset_time_ns: int
    x: float
    y: float
    z: float
    reflectivity: int
    tag: int
    line: int


@dataclass(frozen=True)
class Mid360ScanFrame:
    timebase_ns: int
    sequence: int
    points: tuple[Mid360HitPoint, ...]

    @property
    def point_num(self) -> int:
        return len(self.points)


class Mid360Lidar:
    def scan(self, timebase_ns: int, sequence: int) -> Mid360ScanFrame:
        mount = self._world_mount()  # 每帧只读一次
        starts, ends = self._world_rays(mount, sequence)
        hits = self._backend.ray_test_indexed_hits(
            starts,
            ends,
            collision_mask=self._collision_mask,
        )
        points = self._local_points_from_hits(mount, hits, sequence)
        return Mid360ScanFrame(timebase_ns, sequence, tuple(points))
```

`Mid360Lidar.__init__()` 显式接收并保存 `collision_mask: int = LIDAR_VISIBLE_GROUP`，按 `sensor_backend` 已有的 `0..0x7fffffff` 合同校验；fake backend 的签名也必须是 `ray_test_indexed_hits(starts, ends, *, collision_mask)`，并断言收到了该值。`Mid360HitPoint/Mid360ScanFrame` 是物理层内部冻结值，不带 descriptor/session/world generation，也不得导入阶段三 `slope_sim.interfaces.models.LidarPointCloud`。实时 profile 强制单次 5760-ray batch；离线 20000 rays 按每批最多 16383 条分片，但复用同一 `mount` 且批间不得 step。任一批次异常时不返回消息，sequence 由上层已经占用并留下 gap；Task 6 才把该帧显式映射为 Task 5 的 `LidarPointCloudV2`。

- [ ] **Step 6: 运行聚焦 GREEN 回归**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_mid360_rays.py tests/stage4/test_mid360_direct.py tests/test_sensor_backend.py tests/test_lidar_collision_filters.py`

Expected: PASS。

- [ ] **Step 7: REFACTOR R2 表和 batch 分片实现**

只去除 profile 校验、局部/世界坐标变换和实时/离线 batch 分片中的重复代码；不得改变候选顺序、`collision_mask`、offset、sequence gap 或一次冻结 mount 的边界。随后原样重跑 Step 6，Expected: PASS。

## Task 4：三点 RTK 与同刻姿态真值

**Files:**
- Create: `slope_sim/rtk_triplet.py`
- Modify: `slope_sim/truth_sensors.py`
- Modify: `slope_sim/interfaces/runtime.py`
- Test: `tests/stage4/test_rtk_triplet.py`
- Test: `tests/stage4/test_rtk_triplet_direct.py`

- [ ] **Step 1: 写二轮/四轮几何 RED**

测试文件先正常收集，再由测试函数内的 `_load_rtk_triplet_api()` 断言新 API 存在；禁止顶层导入尚不存在的 `slope_sim.rtk_triplet`。除平地样例外，必须增加 `roll=0.2, pitch=0.3, yaw=0.4` 的姿态样例，证明 heading oracle 使用投影后的同一横轴几何，而不是把它错误地与 Euler yaw 直接比较。

```python
def test_four_wheel_triplet_uses_side_means(fake_backend) -> None:
    fake_backend.set_positions({
        "front_left_drive_wheel": (2.0, 1.0, 0.2),
        "rear_left_drive_wheel": (0.0, 1.0, 0.2),
        "front_right_drive_wheel": (2.0, -1.0, 0.2),
        "rear_right_drive_wheel": (0.0, -1.0, 0.2),
    })
    state = TruthSensorSuiteStage4(fake_backend, get_robot_model("active_steering_4wd")).read_rtk(10)
    assert state.left == TruthPoint3d(1.0, 1.0, 0.2)
    assert state.center == TruthPoint3d(1.0, 0.0, 0.2)
    assert state.right == TruthPoint3d(1.0, -1.0, 0.2)
    assert state.heading_rad == pytest.approx(0.0)
```

二轮测试断言 LEFT/RIGHT 为两个驱动轮轴 link 世界中心，CENTER 为中点；缺 link、NaN/Inf、左右基线 `<=1e-6m` 整帧失败。

- [ ] **Step 2: 写四车型 DIRECT oracle RED**

先创建 `tests/stage4/test_rtk_triplet_direct.py`，不得等三点采样器实现后再补。文件顶层只导入标准库、pytest 和已有 PyBullet 依赖；每个测试函数先通过 `_load_rtk_triplet_api()` 检查 wished-for API，缺模块或缺符号时调用 `pytest.fail("RTK triplet API is not implemented", pytrace=False)`。API 存在后，每个模型从 PyBullet `getLinkState` 独立读取 wheel link 位置并计算期望值；再从同帧 base quaternion 的旋转矩阵独立变换局部左轴，并按上述同一投影定义计算 heading oracle。比较消息三点误差 `<=1e-4m`、heading wrapped error `<=1e-4rad`，覆盖非零 roll/pitch 和车辆运动后状态，不复用生产 helper。

- [ ] **Step 3: 同时运行单元与 DIRECT RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_rtk_triplet.py tests/stage4/test_rtk_triplet_direct.py`

Expected: 两个测试文件都正常收集并 `FAILED`；失败原因是三点几何/同刻姿态 API 尚未实现，不得是 import/fixture/PyBullet 初始化错误或 skip。保存该失败输出后才可进入 Step 4。

- [ ] **Step 4: 实现稳定 side mean 与航向**

```python
@dataclass(frozen=True)
class TruthPoint3d:
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class RtkTripletTruth:
    left: TruthPoint3d
    center: TruthPoint3d
    right: TruthPoint3d
    heading_rad: float


@dataclass(frozen=True)
class AttitudeTruth:
    roll_rad: float
    pitch_rad: float


def _mean_position(backend: SensorBackend, names: tuple[str, ...]) -> Vec3:
    positions = tuple(backend.world_pose(name).position for name in names)
    count = float(len(positions))
    return tuple(sum(p[axis] for p in positions) / count for axis in range(3))


left = _mean_position(self._backend, model.rtk_left_link_names)
right = _mean_position(self._backend, model.rtk_right_link_names)
center = tuple((a + b) * 0.5 for a, b in zip(left, right, strict=True))
heading = wrap_angle(math.atan2(left[1] - right[1], left[0] - right[0]) - math.pi / 2.0)
```

同帧只读取一次 base_link quaternion，把 base 局部左轴 `(0, 1, 0)` 用该 quaternion 旋转到世界坐标，取其水平投影并使用与 RTK 基线完全相同的公式计算独立 oracle：

```python
base_left_world = rotate_vector(base_pose.orientation, (0.0, 1.0, 0.0))
if math.hypot(base_left_world[0], base_left_world[1]) <= 1e-6:
    raise RuntimeError("base lateral axis has degenerate horizontal projection")
base_heading_oracle = wrap_angle(
    math.atan2(base_left_world[1], base_left_world[0]) - math.pi / 2.0
)
if abs(wrap_angle(heading - base_heading_oracle)) > 1e-4:
    raise RuntimeError("RTK baseline disagrees with base lateral-axis projection")
```

roll/pitch 非零时，横轴水平投影 heading 一般不等于 quaternion 的 Euler yaw，因此禁止直接比较这两个值。IMU roll/pitch 仍由同一个已冻结 quaternion 计算，保持同一采样时刻。

`TruthSensorSuiteStage4` 返回协议无关的冻结 `RtkTripletTruth(left, center, right, heading_rad)` 与 `AttitudeTruth(roll_rad, pitch_rad)`；它们不导入阶段三 `RtkState/ImuAttitude`，也不自行生成 session/generation/sequence。Task 6 使用同一个 `SensorSampleContext` 和已预留的 A `OutputIdentity` 显式构造 Task 5 的 `RtkStateV2/ImuAttitudeV2`。

- [ ] **Step 5: 运行 GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_rtk_triplet.py tests/stage4/test_rtk_triplet_direct.py tests/test_truth_sensors.py tests/test_truth_sensors_direct.py`

Expected: PASS。

- [ ] **Step 6: REFACTOR 同刻采样和有限值校验**

只抽取已覆盖的 side mean、投影 heading 和冻结 quaternion 读取重复代码；不得重新引入 Euler-yaw 等价假设或额外 backend 采样。随后原样重跑 Step 5，Expected: PASS。

## Task 5：补齐传感器不可变 v2 模型与严格 codec

**Files:**
- Modify: `slope_sim/interfaces/v2/models.py`
- Modify: `slope_sim/interfaces/v2/codec.py`
- Create: `tests/stage4/test_v2_sensor_codec.py`
- Test: `tests/stage4/test_v2_codec.py`
- Test: `tests/stage4/test_cpp_v2_interop.py`

- [ ] **Step 1: 写三种输出模型、确定性编码和严格解码 RED**

`models.py`/`codec.py` 在阶段 A 已存在，但 B 的传感器符号尚不存在；因此测试文件不得在模块顶层 `from ...models import LidarPointV2`。先正常收集，再在测试函数内导入模块、用 `getattr()` 收集缺失符号并 `assert not missing, f"missing v2 sensor API: {missing}"`，然后才执行以下行为断言。当前树必须得到正常 `FAILED`，不能得到 collection error。

```python
from hashlib import sha256
from importlib import import_module

import pytest

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb
from slope_sim.interfaces.v2.codec import V2ProtoCodec


SESSION = bytes.fromhex("00112233445566778899aabbccddeeff")


def _sensor_types():
    models = import_module("slope_sim.interfaces.v2.models")
    names = (
        "ImuAttitudeV2", "LidarPointCloudV2", "LidarPointV2", "Point3dV2", "RtkStateV2"
    )
    missing = tuple(name for name in names if not hasattr(models, name))
    assert not missing, f"missing v2 sensor API: {missing}"
    return tuple(getattr(models, name) for name in names)


def test_lidar_v2_uses_one_deterministic_payload(descriptor) -> None:
    _, LidarPointCloudV2, LidarPointV2, _, _ = _sensor_types()
    model = LidarPointCloudV2(
        timebase_ns=1_000_000_000,
        frame_id="lidar_link",
        point_num=2,
        lidar_id=1,
        points=(
            LidarPointV2(0, 1.0, 0.0, 0.25, 0, 0, 0),
            LidarPointV2(50_000_000, 0.0, 2.0, 0.5, 0, 0, 8),
        ),
        sequence=3,
        world_generation=2,
        simulation_session_id=SESSION,
        descriptor_sha256=descriptor.sha256,
    )
    codec = V2ProtoCodec(descriptor)
    first = codec.encode(model)
    second = codec.encode(model)
    assert first == second
    assert first.type_name == "slope_sim.interfaces.v2.LidarPointCloud"
    assert first.payload_sha256 == sha256(first.payload).digest()
    assert codec.decode_lidar_point_cloud(first.payload) == model


def test_rtk_and_imu_v2_round_trip_exactly(descriptor) -> None:
    ImuAttitudeV2, _, _, Point3dV2, RtkStateV2 = _sensor_types()
    rtk = RtkStateV2(
        timestamp_ns=1_000_000_000,
        sequence=4,
        world_generation=2,
        frame_id="world",
        left=Point3dV2(1.0, 0.5, 0.2),
        center=Point3dV2(1.0, 0.0, 0.2),
        right=Point3dV2(1.0, -0.5, 0.2),
        heading_rad=0.0,
        simulation_session_id=SESSION,
        descriptor_sha256=descriptor.sha256,
    )
    imu = ImuAttitudeV2(
        timestamp_ns=1_000_000_000,
        roll_rad=0.1,
        pitch_rad=-0.2,
        sequence=5,
        world_generation=2,
        frame_id="base_link",
        simulation_session_id=SESSION,
        descriptor_sha256=descriptor.sha256,
    )
    codec = V2ProtoCodec(descriptor)
    assert codec.decode_rtk_state(codec.encode(rtk).payload) == rtk
    assert codec.decode_imu_attitude(codec.encode(imu).payload) == imu


def test_rtk_decode_rejects_missing_center_before_default_zero_is_used(descriptor) -> None:
    message = pb.RtkState(
        timestamp_ns=1,
        sequence=0,
        world_generation=1,
        frame_id="world",
        left=pb.Point3d(x_m=1.0, y_m=0.5, z_m=0.2),
        right=pb.Point3d(x_m=1.0, y_m=-0.5, z_m=0.2),
        heading_rad=0.0,
        simulation_session_id=SESSION,
        descriptor_sha256=descriptor.sha256,
    )
    with pytest.raises(ValueError, match="left, center and right"):
        V2ProtoCodec(descriptor).decode_rtk_state(
            message.SerializeToString(deterministic=True)
        )
```

同文件再用逐项参数化固定以下拒绝表：bool 冒充整数、uint 溢出、NaN/Inf、错误 session/digest 长度、`world_generation=0`；LiDAR 非 `lidar_link`、`lidar_id!=1`、`point_num` 不等、点不是精确 `LidarPointV2`、offset 不严格递增或 `>=100_000_000`、`reflectivity/tag` 超过 255、`line>15`，以及坐标绝对值超过 IEEE-754 binary32 最大有限值（例如 `3.5e38`）时在 model 构造阶段拒绝；RTK 非 `world`、缺任一子消息、非有限三点/航向、航向不在 `[-pi,pi)`、水平基线 `<=1e-6m`；IMU 非 `base_link` 或非有限姿态。另加一个可表示但需舍入的 LiDAR 坐标 round-trip，断言模型在构造时规范化为 binary32，避免模型可构造、codec 才 overflow 或 encode/decode 不相等。三种 decode 都覆盖 malformed bytes 和错误带内 descriptor，且错误必须在 sequence/generation 连续性统计前返回。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_sensor_codec.py tests/stage4/test_v2_codec.py`

Expected: 正常收集后 `FAILED`，失败消息列出缺失的传感器业务模型/decode API；不得出现 collection error 或 skip。

- [ ] **Step 3: 实现冻结且自校验的传感器模型**

在 A 已有 `require_uint()`、`require_fixed_bytes()` 基础上新增有限实数 helper；不修改 v1 `slope_sim/interfaces/models.py`。字段和类型固定如下：

```python
def require_finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{name} must be a finite number")
    return normalized


def require_finite_float32(name: str, value: object) -> float:
    """拒绝 protobuf float 溢出，并把业务值规范化为可精确回读的 binary32。"""
    normalized = require_finite(name, value)
    try:
        packed = struct.pack("!f", normalized)
    except OverflowError as exc:
        raise ValueError(f"{name} must fit finite float32") from exc
    canonical = struct.unpack("!f", packed)[0]
    if not math.isfinite(canonical):
        raise ValueError(f"{name} must fit finite float32")
    return canonical


def _normalize_output_identity(instance: object, *, timestamp_field: str) -> None:
    object.__setattr__(
        instance,
        timestamp_field,
        require_uint(timestamp_field, getattr(instance, timestamp_field), _UINT64_MAX),
    )
    object.__setattr__(
        instance,
        "sequence",
        require_uint("sequence", getattr(instance, "sequence"), _UINT64_MAX),
    )
    generation = require_uint(
        "world_generation", getattr(instance, "world_generation"), _UINT64_MAX
    )
    if generation == 0:
        raise ValueError("world_generation must be positive")
    object.__setattr__(instance, "world_generation", generation)
    object.__setattr__(
        instance,
        "simulation_session_id",
        require_fixed_bytes(
            "simulation_session_id", getattr(instance, "simulation_session_id"), 16
        ),
    )
    object.__setattr__(
        instance,
        "descriptor_sha256",
        require_fixed_bytes(
            "descriptor_sha256", getattr(instance, "descriptor_sha256"), 32
        ),
    )


@dataclass(frozen=True)
class LidarPointV2:
    offset_time_ns: int
    x: float
    y: float
    z: float
    reflectivity: int
    tag: int
    line: int

    def __post_init__(self) -> None:
        offset = require_uint("offset_time_ns", self.offset_time_ns, _UINT32_MAX)
        if offset >= 100_000_000:
            raise ValueError("offset_time_ns must be below 100 ms")
        object.__setattr__(self, "offset_time_ns", offset)
        for name in ("x", "y", "z"):
            object.__setattr__(self, name, require_finite_float32(name, getattr(self, name)))
        for name in ("reflectivity", "tag"):
            value = require_uint(name, getattr(self, name), 255)
            object.__setattr__(self, name, value)
        object.__setattr__(self, "line", require_uint("line", self.line, 15))


@dataclass(frozen=True)
class Point3dV2:
    x_m: float
    y_m: float
    z_m: float

    def __post_init__(self) -> None:
        for name in ("x_m", "y_m", "z_m"):
            object.__setattr__(self, name, require_finite(name, getattr(self, name)))


@dataclass(frozen=True)
class LidarPointCloudV2:
    timebase_ns: int
    frame_id: str
    point_num: int
    lidar_id: int
    points: tuple[LidarPointV2, ...]
    sequence: int
    world_generation: int
    simulation_session_id: bytes
    descriptor_sha256: bytes

    def __post_init__(self) -> None:
        _normalize_output_identity(self, timestamp_field="timebase_ns")
        lidar_id = require_uint("lidar_id", self.lidar_id, _UINT32_MAX)
        if self.frame_id != "lidar_link" or lidar_id != 1:
            raise ValueError("v2 LiDAR requires frame_id=lidar_link and lidar_id=1")
        points = tuple(self.points)
        if any(type(point) is not LidarPointV2 for point in points):
            raise ValueError("points must contain exact LidarPointV2 values")
        point_num = require_uint("point_num", self.point_num, _UINT32_MAX)
        if point_num != len(points):
            raise ValueError("point_num must equal len(points)")
        offsets = tuple(point.offset_time_ns for point in points)
        if any(current >= following for current, following in zip(offsets, offsets[1:])):
            raise ValueError("LiDAR point offsets must be strictly increasing")
        object.__setattr__(self, "point_num", point_num)
        object.__setattr__(self, "lidar_id", lidar_id)
        object.__setattr__(self, "points", points)


@dataclass(frozen=True)
class RtkStateV2:
    timestamp_ns: int
    sequence: int
    world_generation: int
    frame_id: str
    left: Point3dV2
    center: Point3dV2
    right: Point3dV2
    heading_rad: float
    simulation_session_id: bytes
    descriptor_sha256: bytes

    def __post_init__(self) -> None:
        _normalize_output_identity(self, timestamp_field="timestamp_ns")
        if self.frame_id != "world":
            raise ValueError("v2 RTK frame_id must be world")
        if any(type(point) is not Point3dV2 for point in (self.left, self.center, self.right)):
            raise ValueError("RTK left, center and right must be exact Point3dV2 values")
        heading = require_finite("heading_rad", self.heading_rad)
        if not -math.pi <= heading < math.pi:
            raise ValueError("heading_rad must be in [-pi, pi)")
        if math.hypot(self.left.x_m - self.right.x_m, self.left.y_m - self.right.y_m) <= 1e-6:
            raise ValueError("RTK LEFT-RIGHT horizontal baseline is degenerate")
        object.__setattr__(self, "heading_rad", heading)


@dataclass(frozen=True)
class ImuAttitudeV2:
    timestamp_ns: int
    roll_rad: float
    pitch_rad: float
    sequence: int
    world_generation: int
    frame_id: str
    simulation_session_id: bytes
    descriptor_sha256: bytes

    def __post_init__(self) -> None:
        _normalize_output_identity(self, timestamp_field="timestamp_ns")
        if self.frame_id != "base_link":
            raise ValueError("v2 IMU frame_id must be base_link")
        object.__setattr__(self, "roll_rad", require_finite("roll_rad", self.roll_rad))
        object.__setattr__(self, "pitch_rad", require_finite("pitch_rad", self.pitch_rad))
```

上述 helper 校验并写回 timestamp/sequence 为 uint64、`world_generation` 为 `1..UINT64_MAX`、session 为精确 16 bytes、descriptor 为精确 32 bytes。只有 protobuf `float` 类型的 LiDAR `x/y/z` 走 `require_finite_float32()`；`Point3d`、heading 和 IMU 的 protobuf 类型是 `double`，继续走有限 double 校验。`models.py` 同时加入 `from numbers import Real` 和 `import struct`；points 输入列表在构造时复制为 tuple，不能保留调用方可变引用。

- [ ] **Step 4: 扩展同一个 v2 codec，不建立第二套 sensor codec**

保留 A 的 wheel 分支，把 `V2ProtoCodec.encode()` 参数类型精确扩为 `WheelCommandV2 | WheelStateV2 | LidarPointCloudV2 | RtkStateV2 | ImuAttitudeV2`，并给同一 `if/elif` 链追加三类输出：

```python
elif isinstance(model, LidarPointCloudV2):
    message = pb.LidarPointCloud(
        timebase_ns=model.timebase_ns,
        frame_id=model.frame_id,
        point_num=model.point_num,
        lidar_id=model.lidar_id,
        points=tuple(
            pb.LidarPoint(
                offset_time_ns=point.offset_time_ns,
                x=point.x,
                y=point.y,
                z=point.z,
                reflectivity=point.reflectivity,
                tag=point.tag,
                line=point.line,
            )
            for point in model.points
        ),
        sequence=model.sequence,
        world_generation=model.world_generation,
        simulation_session_id=model.simulation_session_id,
        descriptor_sha256=model.descriptor_sha256,
    )
elif isinstance(model, RtkStateV2):
    message = pb.RtkState(
        timestamp_ns=model.timestamp_ns,
        sequence=model.sequence,
        world_generation=model.world_generation,
        frame_id=model.frame_id,
        left=pb.Point3d(x_m=model.left.x_m, y_m=model.left.y_m, z_m=model.left.z_m),
        center=pb.Point3d(x_m=model.center.x_m, y_m=model.center.y_m, z_m=model.center.z_m),
        right=pb.Point3d(x_m=model.right.x_m, y_m=model.right.y_m, z_m=model.right.z_m),
        heading_rad=model.heading_rad,
        simulation_session_id=model.simulation_session_id,
        descriptor_sha256=model.descriptor_sha256,
    )
elif isinstance(model, ImuAttitudeV2):
    message = pb.ImuAttitude(
        timestamp_ns=model.timestamp_ns,
        roll_rad=model.roll_rad,
        pitch_rad=model.pitch_rad,
        sequence=model.sequence,
        world_generation=model.world_generation,
        frame_id=model.frame_id,
        simulation_session_id=model.simulation_session_id,
        descriptor_sha256=model.descriptor_sha256,
    )
```

仍只在分支链末尾执行一次 `SerializeToString(deterministic=True)` 和一次 SHA-256。新增 `decode_lidar_point_cloud()`、`decode_rtk_state()`、`decode_imu_attitude()`，全部先调用 A 的 `_parse()` 做 payload 类型、Protobuf、descriptor 和 session 校验，再显式逐字段构造上述 dataclass；禁止 `MessageToDict` 或反射复制。RTK 在读取坐标前必须执行：

```python
missing = tuple(
    name for name in ("left", "center", "right")
    if not message.HasField(name)
)
if missing:
    raise ValueError("RTK requires left, center and right submessages")
```

三个方法的返回构造固定为：

```python
def decode_lidar_point_cloud(self, payload: object) -> LidarPointCloudV2:
    message = self._parse(payload, pb.LidarPointCloud())
    return LidarPointCloudV2(
        timebase_ns=message.timebase_ns,
        frame_id=message.frame_id,
        point_num=message.point_num,
        lidar_id=message.lidar_id,
        points=tuple(
            LidarPointV2(
                point.offset_time_ns,
                point.x,
                point.y,
                point.z,
                point.reflectivity,
                point.tag,
                point.line,
            )
            for point in message.points
        ),
        sequence=message.sequence,
        world_generation=message.world_generation,
        simulation_session_id=bytes(message.simulation_session_id),
        descriptor_sha256=bytes(message.descriptor_sha256),
    )


def decode_rtk_state(self, payload: object) -> RtkStateV2:
    message = self._parse(payload, pb.RtkState())
    missing = tuple(
        name for name in ("left", "center", "right")
        if not message.HasField(name)
    )
    if missing:
        raise ValueError("RTK requires left, center and right submessages")
    return RtkStateV2(
        timestamp_ns=message.timestamp_ns,
        sequence=message.sequence,
        world_generation=message.world_generation,
        frame_id=message.frame_id,
        left=Point3dV2(message.left.x_m, message.left.y_m, message.left.z_m),
        center=Point3dV2(message.center.x_m, message.center.y_m, message.center.z_m),
        right=Point3dV2(message.right.x_m, message.right.y_m, message.right.z_m),
        heading_rad=message.heading_rad,
        simulation_session_id=bytes(message.simulation_session_id),
        descriptor_sha256=bytes(message.descriptor_sha256),
    )


def decode_imu_attitude(self, payload: object) -> ImuAttitudeV2:
    message = self._parse(payload, pb.ImuAttitude())
    return ImuAttitudeV2(
        timestamp_ns=message.timestamp_ns,
        roll_rad=message.roll_rad,
        pitch_rad=message.pitch_rad,
        sequence=message.sequence,
        world_generation=message.world_generation,
        frame_id=message.frame_id,
        simulation_session_id=bytes(message.simulation_session_id),
        descriptor_sha256=bytes(message.descriptor_sha256),
    )
```

- [ ] **Step 5: 运行 GREEN、wheel 回归和 golden 兼容检查**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_sensor_codec.py tests/stage4/test_v2_codec.py`

Expected: PASS，五类 v2 业务模型都由一个 codec 确定性编码，三种传感器严格 decode。

Run: `STAGE4_PHASE0_BUILD_DIR=$PWD/build/stage4-phase0 conda run -n slope-sim python -m pytest -q tests/stage4/test_cpp_v2_interop.py`

Expected: PASS；A 冻结的五消息 Python/C++ golden bytes 未被 B 改写。若 build 目录尚未由 A 生成，本项必须在 A 完成后执行，不能以 skip 计通过。

- [ ] **Step 6: REFACTOR 模型 identity 与 codec 分支重复代码**

只抽取已被拒绝表覆盖的 identity、finite/binary32 和 protobuf 子消息转换 helper；不改变任何 v2 字段、type name、确定性 bytes 或 A 的 wheel 分支。随后原样重跑 Step 5 的两条命令，Expected: PASS，golden bytes 和 descriptor SHA-256 不变。

## Task 6：正式五通道 runtime 配置、v2 transport 切换与 Dashboard

**Files:**
- Create: `slope_sim/interfaces/v2/runtime_config.py`
- Modify: `slope_sim/interfaces/runtime.py`
- Modify: `slope_sim/simulation.py`
- Modify: `slope_sim/interfaces/dashboard_snapshot.py`
- Modify: `slope_sim/dashboard.py`
- Modify: `slope_sim/dashboard_charts.py`
- Modify: `slope_sim/manual_demo.py`
- Modify: `main.py`
- Create: `tests/stage4/test_v2_runtime_config.py`
- Create: `tests/stage4/test_stage4_runtime_sensors.py`
- Create: `tests/stage4/test_stage4_dashboard.py`
- Test: `tests/stage4/test_v2_codec.py`
- Test: `tests/stage4/test_v2_runtime_protocol.py`
- Test: `tests/test_dashboard_manual_verifier.py`
- Test: `tests/test_interface_runtime.py`
- Test: `tests/test_interface_pause_rebuild.py`

- [ ] **Step 1: 写五通道 config 和禁止阶段三隐式配置 RED**

```python
from importlib import import_module
from importlib.util import find_spec

import pytest

from slope_sim.interfaces.v2.topics import V2_TOPICS


def _runtime_config_type():
    assert find_spec("slope_sim.interfaces.v2.runtime_config") is not None, (
        "V2RuntimeConfig API is not implemented"
    )
    module = import_module("slope_sim.interfaces.v2.runtime_config")
    config_type = getattr(module, "V2RuntimeConfig", None)
    assert config_type is not None, "V2RuntimeConfig API is not implemented"
    return config_type


def test_production_v2_runtime_config_is_exactly_five_channels() -> None:
    V2RuntimeConfig = _runtime_config_type()
    config = V2RuntimeConfig.production()
    assert config.transport_mode == "ecal"
    assert config.channels == V2_TOPICS
    assert tuple(channel.topic for channel in config.channels) == (
        "/sim/wheel/command",
        "/sim/wheel/state",
        "/sim/lidar/points",
        "/sim/rtk/state",
        "/sim/imu/attitude",
    )
    assert config.lidar.topic == "/sim/lidar/points"
    assert not hasattr(config, "lidar_front")
    assert not hasattr(config, "lidar_rear")


def test_v2_production_rejects_auto_fallback() -> None:
    V2RuntimeConfig = _runtime_config_type()
    with pytest.raises(ValueError, match="ecal or local"):
        V2RuntimeConfig(transport_mode="auto")
```

再覆盖：channels 被换序、删减、重复或替换 topic/type/rate/direction；bool/0/负数容量；非有限/非正 timeout/window。`local` 仅是测试依赖注入 profile，测试统一用 `V2RuntimeConfig.local_for_test()` 表达；正式 `production()` 永远是 strict eCAL。

- [ ] **Step 2: 运行 config RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_runtime_config.py`

Expected: 正常收集后 `FAILED`，失败消息为 `V2RuntimeConfig API is not implemented`；不得出现 collection error 或 skip。

- [ ] **Step 3: 实现只引用 A 五话题合同的 config**

```python
# slope_sim/interfaces/v2/runtime_config.py
"""阶段四 runtime 配置：固定五话题和生产 eCAL 资源预算。"""
from dataclasses import dataclass
import math

from slope_sim.interfaces.v2.topics import V2_BY_TOPIC, V2_TOPICS, V2TopicContract


@dataclass(frozen=True)
class V2RuntimeConfig:
    transport_mode: str
    channels: tuple[V2TopicContract, ...] = V2_TOPICS
    command_timeout_sec: float = 0.100
    status_window_sec: float = 2.0
    outgoing_queue_size: int = 32
    log_queue_size: int = 256

    def __post_init__(self) -> None:
        if self.transport_mode not in {"ecal", "local"}:
            raise ValueError("v2 transport_mode must be ecal or local")
        channels = tuple(self.channels)
        if channels != V2_TOPICS:
            raise ValueError("v2 runtime channels must exactly match V2_TOPICS")
        if any(type(channel) is not V2TopicContract for channel in channels):
            raise ValueError("v2 runtime channels must be exact V2TopicContract values")
        for name in ("command_timeout_sec", "status_window_sec"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be positive and finite")
            normalized = float(value)
            if not math.isfinite(normalized) or normalized <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
            object.__setattr__(self, name, normalized)
        for name in ("outgoing_queue_size", "log_queue_size"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        object.__setattr__(self, "channels", channels)

    @property
    def wheel_command(self) -> V2TopicContract:
        return V2_BY_TOPIC["/sim/wheel/command"]

    @property
    def wheel_state(self) -> V2TopicContract:
        return V2_BY_TOPIC["/sim/wheel/state"]

    @property
    def lidar(self) -> V2TopicContract:
        return V2_BY_TOPIC["/sim/lidar/points"]

    @property
    def rtk(self) -> V2TopicContract:
        return V2_BY_TOPIC["/sim/rtk/state"]

    @property
    def imu(self) -> V2TopicContract:
        return V2_BY_TOPIC["/sim/imu/attitude"]

    @classmethod
    def production(cls) -> "V2RuntimeConfig":
        return cls(transport_mode="ecal")

    @classmethod
    def local_for_test(cls) -> "V2RuntimeConfig":
        return cls(transport_mode="local")
```

不得从阶段三 `InterfaceConfig.default()` 复制 channels；`V2_TOPICS` 是唯一五话题源。v1 `InterfaceConfig` 保持六通道并只服务显式 v1 路径。

- [ ] **Step 4: 运行 config GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_runtime_config.py tests/stage4/test_v2_codec.py`

Expected: PASS；五话题只来自 A 的 `V2_TOPICS`，production/local 边界明确。未 GREEN 前不得开始 runtime/Dashboard RED。

- [ ] **Step 5: 写正式 factory、单 LiDAR、Dashboard 和 encode-once RED**

```python
def test_v2_session_calls_only_raw_v2_transport_factory(
    monkeypatch, descriptor, stage4_session_inputs
) -> None:
    calls = []
    raw_transport = FakeRawTransport()

    def raw_factory(**kwargs):
        calls.append(kwargs)
        return raw_transport

    monkeypatch.setattr("slope_sim.simulation.load_v2_descriptor", lambda: descriptor)
    monkeypatch.setattr("slope_sim.simulation.create_v2_ecal_transport", raw_factory)
    monkeypatch.setattr(
        "slope_sim.simulation.create_transport",
        lambda *args, **kwargs: pytest.fail("v2 must not call the v1 transport factory"),
    )
    session = create_v2_interface_session(**stage4_session_inputs)
    assert calls == [{
        "descriptor": descriptor,
        "queue_size": 32,
        "participant_name": "slope-sim-v2",
    }]
    assert type(session.runtime.config) is V2RuntimeConfig
    assert session.transport is raw_transport


def test_runtime_publishes_exact_v2_outputs_and_uses_same_lidar_for_dashboard(
    v2_runtime, transport, logger_spy, codec_spy, codec, wheel_oracle
) -> None:
    v2_runtime.before_physics_step(0.1, wall_time=0.0)
    v2_runtime.after_physics_step(0.1)
    topics = [topic for topic, *_ in transport.published]
    assert topics.count("/sim/wheel/state") == 10
    assert topics.count("/sim/lidar/points") == 1
    assert topics.count("/sim/rtk/state") == 1
    assert topics.count("/sim/imu/attitude") == 1
    assert "/sim/lidar/front/points" not in topics
    assert "/sim/lidar/rear/points" not in topics
    assert codec_spy.encode_calls_by_topic == {
        "/sim/wheel/state": 10,
        "/sim/lidar/points": 1,
        "/sim/rtk/state": 1,
        "/sim/imu/attitude": 1,
    }
    wheel_payloads = transport.payloads_by_topic["/sim/wheel/state"]
    wheel_logs = logger_spy.payloads_by_topic["/sim/wheel/state"]
    assert len(wheel_payloads) == len(wheel_logs) == len(wheel_oracle) == 10
    assert transport.type_names_by_topic["/sim/wheel/state"] == [
        "slope_sim.interfaces.v2.WheelState"
    ] * 10
    for payload, logged_payload, expected in zip(
        wheel_payloads, wheel_logs, wheel_oracle, strict=True
    ):
        wheel_state = codec.decode_wheel_state(payload)
        physical = expected.physical
        identity = expected.identity
        runtime_snapshot = expected.runtime_snapshot
        authority = runtime_snapshot.authority
        assert type(wheel_state) is WheelStateV2
        assert payload is logged_payload
        assert identity.topic == "/sim/wheel/state"
        assert wheel_state.timestamp_ns == physical.timestamp_ns
        assert wheel_state.sequence == identity.sequence
        assert (
            wheel_state.simulation_session_id
            == identity.simulation_session_id
            == runtime_snapshot.simulation_session_id
        )
        assert (
            wheel_state.descriptor_sha256
            == identity.descriptor_sha256
            == runtime_snapshot.descriptor_sha256
        )
        assert (
            wheel_state.world_generation
            == identity.world_generation
            == runtime_snapshot.world_generation
        )
        assert (
            wheel_state.command_generation
            == authority.command_generation
            == runtime_snapshot.command_generation
        )
        assert wheel_state.command_authority_state is authority.state
        assert wheel_state.command_peer_count == authority.peer_count
        assert wheel_state.command_owner_source_id == (
            authority.owner_source_id or ""
        )
        assert wheel_state.command_owner_source_session_id == (
            authority.owner_source_session_id or b""
        )
        assert wheel_state.robot_model == expected.robot_model
        assert wheel_state.drive_wheel_speed_rad_s == (
            physical.drive_wheel_speed_rad_s
        )
        assert wheel_state.steering_wheel_angle_rad == (
            physical.steering_wheel_angle_rad
        )
    for topic in codec_spy.encode_calls_by_topic:
        sent = transport.payloads_by_topic[topic]
        logged = logger_spy.payloads_by_topic[topic]
        assert len(sent) == len(logged)
        assert all(
            sent_payload is logged_payload
            for sent_payload, logged_payload in zip(sent, logged, strict=True)
        )
    snapshot = v2_runtime.dashboard_snapshot()
    assert snapshot.lidar is not None
    assert snapshot.lidar_view is not None
    assert snapshot.lidar is v2_runtime.latest_lidar
    assert snapshot.lidar_view.source_sequence == snapshot.lidar.sequence
    assert len(snapshot.lidar_view.points) <= 2048


def test_wheel_state_reserves_before_read_and_failure_leaves_gap(
    v2_runtime, robot_spy, protocol_spy, transport, codec
) -> None:
    robot_spy.fail_next_wheel_read = RuntimeError("injected wheel read failure")
    v2_runtime.before_physics_step(0.01, wall_time=0.0)
    v2_runtime.after_physics_step(0.01)
    failed = protocol_spy.reserved_by_topic["/sim/wheel/state"][-1]
    reserve_index = protocol_spy.events.index(("reserve", failed))
    read_index = protocol_spy.events.index(("read_wheel_state", failed.sequence))
    assert reserve_index < read_index
    assert all(
        codec.decode_wheel_state(payload).sequence != failed.sequence
        for payload in transport.payloads_by_topic["/sim/wheel/state"]
    )

    v2_runtime.before_physics_step(0.01, wall_time=0.01)
    v2_runtime.after_physics_step(0.01)
    published = codec.decode_wheel_state(
        transport.payloads_by_topic["/sim/wheel/state"][-1]
    )
    assert published.sequence == failed.sequence + 1
```

`wheel_oracle` 由测试的 fake robot 与 protocol spy 在生产调用边界外共同记录，每项只含实际 `WheelState`、预留的 `OutputIdentity`、同次原子 `V2RuntimeSnapshot` 和车型名；不得从待断言 payload 反造 expected。同一 RED 必须断言：每次 output 在传感器工作前调用 `V2RuntimeProtocol.reserve_output(topic)`；WheelState 明确先 reserve 再读取物理轮态，任一读取/构造/编码/发布失败都留下 sequence gap；transport 获得的 `payload` 与 logger 获得的对象为同一 `bytes`；formal factory 没有 `InterfaceConfig` 参数；v2 command callback 只交给 A 的 `V2RuntimeProtocol.accept_payload()`；每轮连接刷新仍是 `poll_peer_state()` 完成后才读 snapshot；prepare/commit/abort/fault 只使用 A controller 的单一 generation，不保留阶段三第二套 epoch。

同一 RED 先通过测试函数内 `getattr()` 检查 `_wheel_state_v2`，再逐项注入错误 topic、session、descriptor、world generation、runtime/authority command generation 不一致，以及四态的 count/owner 组合不一致，全部必须在编码前以明确异常拒绝。`codec_spy` 是对同一个真实 `codec` 的记录装饰器，不能返回不可解码的占位 bytes；`protocol_spy` 与 `robot_spy` 共用一条有序事件表，确保 reserve-before-read 断言来自两个独立 fake 边界。

同一组用 fake integer clock 冻结 100/10 Hz 共网格：每第 10 个 WheelState deadline 与 LiDAR/RTK/IMU 共用一个 `SensorSampleContext.timestamp_ns`，四条 timestamp 逐字相等；注入 1ns 偏移、独立浮点累加漂移和共同 deadline 整体超期时，四个已预留 identity 都留下 gap；注入单个 WheelState、LiDAR、RTK 或 IMU 槽的读取/生成/编码/发布失败时，只留下对应话题的预留 gap，其他三个同刻输出仍完成。所有情况都禁止从前后 WheelState 最近邻补齐。world rebuild/pause resume 后重新以一个整数 epoch 建网格，第一组 10 Hz sample 仍必须落在 WheelState 位点上。

`tests/stage4/test_stage4_dashboard.py` 同一轮先写 Dashboard RED：构造 0、1、2,048、5,760 个命中点的 v2 snapshot，断言只有中心 LiDAR/三点 RTK/IMU 字段、display 点数有界、source sequence 对齐；重复 update 断言 Figure/layout/tab/artist 对象 identity 不变且固定容器尺寸不漂移。测试只用 Task 5 的真实 model 和 Qt offscreen，不用待实现 fixture 名；目标 Dashboard API 在测试函数内经 `getattr()` 检查，缺失时以明确断言 `FAILED`。

- [ ] **Step 6: 运行 runtime 与 Dashboard RED**

Run: `QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_runtime_sensors.py tests/stage4/test_stage4_dashboard.py tests/stage4/test_v2_runtime_protocol.py`

Expected: 正常收集后 `FAILED`；当前入口仍创建 `InterfaceConfig.default()`、调用 `create_transport()`，runtime 仍调度前后两个 LiDAR/编码 v1 模型，也尚无 `_wheel_state_v2` 的完整 identity/authority 绑定和失败 gap，Dashboard 尚无 v2 单中心视图。不得出现 collection error、fixture error 或 skip。

- [ ] **Step 7: 接入 A 的 raw transport/controller 后切换 10 Hz 同刻调度**

`create_v2_interface_session()` 只接受精确 `SceneDocumentV2`，公开 API 固定为：

```python
def create_v2_interface_session(
    config: ExperimentConfig,
    *,
    client_id: int,
    coordinator_world: ActiveManualWorld,
    obstacle_manager: ObstacleManager,
    document: SceneDocumentV2,
    participant_name: str | None = None,
    monotonic: Callable[[], float] = time.monotonic,
) -> InterfaceSession:
    """创建五话题 v2 eCAL/runtime/logger/backend 的唯一所有权链。"""
```

函数先拒绝非精确 `SceneDocumentV2` 和 `config.interface_mode != "ecal"`，加载一次 descriptor，创建 `V2RuntimeConfig.production()`，再直接调用：

```python
descriptor = load_v2_descriptor()
runtime_config = V2RuntimeConfig.production()
backend = PyBulletSensorBackend(
    client_id,
    coordinator_world.active_robot.robot.robot_id,
)
backend.bind_scene(
    coordinator_world.scene.body_ids,
    obstacle_manager.snapshot(include_body_id=True),
)
interface_logger = (
    InterfaceEventLogger(config.log_dir, queue_size=runtime_config.log_queue_size)
    if config.interface_log_enabled
    else None
)
transport = create_v2_ecal_transport(
    descriptor=descriptor,
    queue_size=runtime_config.outgoing_queue_size,
    participant_name="slope-sim-v2" if participant_name is None else participant_name,
)
protocol = V2RuntimeProtocol(
    coordinator_world.active_robot.robot.model_spec,
    transport=transport,
    descriptor=descriptor,
    monotonic=monotonic,
    timeout_sec=runtime_config.command_timeout_sec,
)
runtime = InterfaceRuntime(
    coordinator_world.active_robot.robot,
    config=runtime_config,
    transport=transport,
    monotonic=monotonic,
    sensor_backend=backend,
    scene_document=document,
    logger=interface_logger,
    capture_lidar_top_view=config.mode == "gui" and config.dashboard_enabled,
    v2_protocol=protocol,
    v2_codec=V2ProtoCodec(descriptor),
)
```

`InterfaceRuntime.__init__()` 对应新增精确参数 `config: InterfaceConfig | V2RuntimeConfig`、`scene_document: SceneDocument | SceneDocumentV2`、`v2_protocol: V2RuntimeProtocol | None = None`、`v2_codec: V2ProtoCodec | None = None`；v1 config 要求两个 v2 参数都为 `None`，v2 config 要求二者均为精确实例。初始化任一步失败时仍按 runtime -> logger -> transport -> backend 的既有所有权顺序幂等回滚，不能泄漏 eCAL participant。

`create_interface_session()` 按文档精确类型分派：`SceneDocumentV2` 只能走上述 v2 函数并用 Task 1 的 `stage4_urdf_path` 建车，`SceneDocument` 只能走既有 v1 函数和 `urdf_path`；不得把 v1 文档静默转换，也不得在 v2 eCAL 初始化失败时退回 local。`main.py`/`manual_demo.py` 使用 loader 返回的 schema 类型选择入口，Dashboard 使用同一 runtime config；正式 v2 CLI 遇到 `interface_mode=auto/local` 直接给出非零错误，`local_for_test()` 只允许测试直接注入 fake/local transport。

`InterfaceRuntime` 的 config 边界改为精确 `InterfaceConfig | V2RuntimeConfig` 分支：v1 分支保留全部旧字段；v2 分支只能读取 `wheel_command/wheel_state/lidar/rtk/imu`，不得用 `getattr(..., "lidar_front")` 做兼容猜测。调度器只保存一个整数 100 Hz tick index；每个 100 Hz 位点先 `reserve_output("/sim/wheel/state")`、再取得一份原子 `V2RuntimeSnapshot`，最后调用 `read_interface_wheel_state(timestamp_ns)`，任何后续失败都不回收 identity。`tick_index % 10 == 0` 时，同一主线程冻结一个 `SensorSampleContext(timestamp_ns, world_generation, base_pose)` 并分别预留 LiDAR/RTK/IMU identity；四个 topic 的 timestamp 必须相同，sequence 各自独立，不得为 10 Hz 另维护浮点 deadline。协议无关真值到 Task 5 模型的映射只在 runtime 发生，WheelState builder 必须位于三个 sensor builder 之前：

```python
@dataclass(frozen=True)
class SensorSampleContext:
    timestamp_ns: int
    world_generation: int
    base_pose: Pose


def _wheel_state_v2(
    sample: WheelState,
    identity: OutputIdentity,
    runtime_snapshot: V2RuntimeSnapshot,
    *,
    robot_model: str,
) -> WheelStateV2:
    """把一次物理轮态与同一时刻的输出身份、命令权快照绑定。"""
    authority = runtime_snapshot.authority
    if (
        identity.topic != "/sim/wheel/state"
        or identity.simulation_session_id
        != runtime_snapshot.simulation_session_id
        or identity.descriptor_sha256 != runtime_snapshot.descriptor_sha256
        or identity.world_generation != runtime_snapshot.world_generation
        or authority.command_generation != runtime_snapshot.command_generation
        or runtime_snapshot.closed
        or runtime_snapshot.fatal_error is not None
    ):
        raise RuntimeError("WheelState output identity mismatch")
    state_matches_count = {
        CommandAuthorityState.WAITING: authority.peer_count == 0,
        CommandAuthorityState.CLAIMABLE: authority.peer_count == 1,
        CommandAuthorityState.ACTIVE: authority.peer_count == 1,
        CommandAuthorityState.CONFLICT: authority.peer_count > 1,
    }
    active = authority.state is CommandAuthorityState.ACTIVE
    has_complete_owner = (
        authority.owner_source_id is not None
        and authority.owner_source_session_id is not None
    )
    has_any_owner = (
        authority.owner_source_id is not None
        or authority.owner_source_session_id is not None
    )
    if (
        not state_matches_count[authority.state]
        or (active and not has_complete_owner)
        or (not active and has_any_owner)
    ):
        raise RuntimeError("WheelState authority snapshot is inconsistent")
    return WheelStateV2(
        timestamp_ns=sample.timestamp_ns,
        drive_wheel_speed_rad_s=sample.drive_wheel_speed_rad_s,
        steering_wheel_angle_rad=sample.steering_wheel_angle_rad,
        sequence=identity.sequence,
        world_generation=identity.world_generation,
        command_generation=runtime_snapshot.command_generation,
        robot_model=robot_model,
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=identity.descriptor_sha256,
        command_authority_state=authority.state,
        command_owner_source_id=authority.owner_source_id or "",
        command_owner_source_session_id=(
            authority.owner_source_session_id or b""
        ),
        command_peer_count=authority.peer_count,
    )


def _lidar_v2(
    frame: Mid360ScanFrame,
    context: SensorSampleContext,
    identity: OutputIdentity,
) -> LidarPointCloudV2:
    if (
        identity.topic != "/sim/lidar/points"
        or frame.sequence != identity.sequence
        or frame.timebase_ns != context.timestamp_ns
        or identity.world_generation != context.world_generation
    ):
        raise RuntimeError("LiDAR output identity mismatch")
    return LidarPointCloudV2(
        timebase_ns=frame.timebase_ns,
        frame_id="lidar_link",
        point_num=frame.point_num,
        lidar_id=1,
        points=tuple(
            LidarPointV2(
                point.offset_time_ns,
                point.x,
                point.y,
                point.z,
                point.reflectivity,
                point.tag,
                point.line,
            )
            for point in frame.points
        ),
        sequence=identity.sequence,
        world_generation=identity.world_generation,
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=identity.descriptor_sha256,
    )


def _rtk_v2(
    sample: RtkTripletTruth,
    context: SensorSampleContext,
    identity: OutputIdentity,
) -> RtkStateV2:
    if (
        identity.topic != "/sim/rtk/state"
        or identity.world_generation != context.world_generation
    ):
        raise RuntimeError("RTK output identity mismatch")
    return RtkStateV2(
        timestamp_ns=context.timestamp_ns,
        sequence=identity.sequence,
        world_generation=identity.world_generation,
        frame_id="world",
        left=Point3dV2(sample.left.x, sample.left.y, sample.left.z),
        center=Point3dV2(sample.center.x, sample.center.y, sample.center.z),
        right=Point3dV2(sample.right.x, sample.right.y, sample.right.z),
        heading_rad=sample.heading_rad,
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=identity.descriptor_sha256,
    )


def _imu_v2(
    sample: AttitudeTruth,
    context: SensorSampleContext,
    identity: OutputIdentity,
) -> ImuAttitudeV2:
    if (
        identity.topic != "/sim/imu/attitude"
        or identity.world_generation != context.world_generation
    ):
        raise RuntimeError("IMU output identity mismatch")
    return ImuAttitudeV2(
        timestamp_ns=context.timestamp_ns,
        roll_rad=sample.roll_rad,
        pitch_rad=sample.pitch_rad,
        sequence=identity.sequence,
        world_generation=identity.world_generation,
        frame_id="base_link",
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=identity.descriptor_sha256,
    )
```

删除每物理步额外 31 条摘要 ray，Dashboard 从正式点云做确定性等距抽样：

```python
def bounded_display_indices(point_count: int, limit: int = 2048) -> tuple[int, ...]:
    if point_count <= limit:
        return tuple(range(point_count))
    return tuple((index * point_count) // limit for index in range(limit))
```

该实现必须逐项满足 Step 5 的 `_wheel_state_v2()` identity/authority RED。每条消息调用一次 `encoded = codec.encode(model)`；`encoded.payload` 原对象同时交给 logger 和 `transport.publish(topic, encoded.payload, encoded.type_name, timestamp_ns)`。WheelState/LiDAR/RTK/IMU 任一读取、生成或编码失败时只记录该话题错误和已预留 gap，不发布半帧，也不阻塞其他话题。

本 B 计划只负责在 runtime snapshot 中暴露当前 canonical schema v2 YAML/SHA-256、scene revision 和 A 的 world generation，不实现也不等待 Recorder attachment ACK。物理 world 成功提交后的 attachment 持久化、ACK 后业务发布恢复，以及跨 revision 的 effective-time 事务全部属于后续 C；B 的 factory、构造函数、测试和完成定义不得依赖 C 的任何模块或 ACK。B 仅保持当前内存场景切换的既有原子边界，并在交付报告中把“Recorder attachment barrier”标为 `NOT_IMPLEMENTED (stage C)`，不能据此宣称录制一致性已通过。

- [ ] **Step 8: 固定单 LiDAR/三点 RTK Dashboard 数据与几何**

v2 dashboard snapshot 使用 `lidar`、`rtk`、`imu` 三个字段和 Task 5 的精确模型类型，不暴露 `lidar_front/lidar_rear`：

```python
@dataclass(frozen=True, slots=True)
class V2LidarDisplayFrame:
    timebase_ns: int
    source_sequence: int
    points: tuple[LidarTopViewPoint, ...]


@dataclass(frozen=True, slots=True)
class V2InterfaceDashboardSnapshot:
    generation: int
    robot_model: str
    sim_time_ns: int
    status: InterfaceStatusSnapshot
    wheel_command: WheelCommandV2 | None
    wheel_command_received_sim_time_ns: int | None
    wheel_state: WheelStateV2 | None
    lidar: LidarPointCloudV2 | None
    rtk: RtkStateV2 | None
    imu: ImuAttitudeV2 | None
    lidar_view: V2LidarDisplayFrame | None
```

`__post_init__()` 对所有非空值要求精确类型，并要求 `lidar is None` 当且仅当 `lidar_view is None`；非空时 `timebase_ns/source_sequence` 分别等于点云、显示点不超过 2,048。`InterfaceRuntime.latest_lidar` 返回锁内捕获的 `LidarPointCloudV2 | None` 只读引用，`dashboard_snapshot()` 在同一锁内同时捕获点云和对应 display frame，锁外只构造冻结快照。

`TelemetryDashboard`、`InterfaceChartBuffer` 接受 `InterfaceConfig | V2RuntimeConfig` 的显式分支：v1 仍显示前/后雷达，v2 只显示“中心 LiDAR”；两种类型以外直接拒绝。

LiDAR 页固定 `x/y = [-40,40]m`、等比例、车辆箭头在原点，点色使用 `sqrt(x*x+y*y+z*z)`；RTK 页用 CENTER 时间曲线、三点数值表和俯视三点几何。更新数据只能替换 artist data，不得重建 Figure、layout 或 tab。显示容器从创建起固定尺寸，空点、1 点、2,048 点和 5,760 命中点都不能改变布局。

- [ ] **Step 9: 运行五通道 runtime、Dashboard 和 v1 兼容 GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_v2_runtime_config.py tests/stage4/test_v2_sensor_codec.py tests/stage4/test_stage4_runtime_sensors.py tests/stage4/test_stage4_dashboard.py tests/stage4/test_v2_runtime_protocol.py`

Expected: PASS；v2 runtime 只有五个正式通道，单 LiDAR/RTK/IMU 使用 v2 模型与确定性 bytes，未触碰阶段三 config/factory。

Run: `conda run -n slope-sim python -m pytest -q tests/test_interface_runtime.py tests/test_interface_pause_rebuild.py tests/test_interface_codec.py tests/test_ecal_transport.py tests/test_dashboard.py tests/test_dashboard_charts.py tests/test_dashboard_enterprise.py tests/test_dashboard_manual_verifier.py -m "not ecal"`

Expected: PASS；显式 v1 六通道行为不变，15 个默认页、33% Dashboard、50:50 内部分区和删除方向按钮的合同不回归。

- [ ] **Step 10: REFACTOR v1/v2 runtime 分支和 Dashboard 更新路径**

只抽取已经由 Step 5 RED 覆盖的 encode/publish、snapshot 捕获和 artist data 更新重复代码；保持 config 精确类型分派，禁止引入 `getattr(..., "lidar_front")` 兼容猜测，也不得引入任何 Recorder/C 模块。随后原样重跑 Step 9 的两条命令，Expected: PASS。

## Task 7：性能分解与固定轻量化措施

**Files:**
- Create: `slope_sim/performance.py`
- Create: `scripts/generate_stage4_golf_truth_fixture.py`
- Create: `scripts/profile_stage4_workload.py`
- Modify: `slope_sim/manual_demo.py`
- Modify: `slope_sim/scene.py`
- Modify: `slope_sim/dashboard.py`
- Create: `tests/fixtures/stage4/golf_truth_v1.json`
- Create: `tests/fixtures/stage4/golf_truth_v1.sha256`
- Create: `tests/fixtures/stage4/golf_truth_v1.provenance.json`
- Create: `tests/stage4/test_performance_budget.py`
- Create: `tests/stage4/test_golf_truth_preservation.py`

- [ ] **Step 1: 先写 truth/provenance 分离 RED（不采样物理）**

`tests/stage4/test_golf_truth_preservation.py` 先只写纯 bytes 边界，不启动 PyBullet，也不顶层导入尚不存在的生成器：

```python
from hashlib import sha256

import pytest


def test_truth_bytes_are_independent_of_provenance_hashes() -> None:
    module = require_wished_script_module(
        "scripts.generate_stage4_golf_truth_fixture"
    )
    canonical_truth_bytes = require_callable(module, "canonical_truth_bytes")
    provenance_bytes = require_callable(module, "provenance_bytes")
    verify_truth_bytes = require_callable(module, "verify_truth_bytes")
    inputs, runtime_identity, oracle = fixed_golf_truth_parts()

    truth = canonical_truth_bytes(inputs, runtime_identity, oracle)
    digest = sha256(truth).hexdigest()
    first = provenance_bytes({"slope_sim/scene.py": "11" * 32})
    second = provenance_bytes({"slope_sim/scene.py": "22" * 32})
    assert first != second
    assert canonical_truth_bytes(inputs, runtime_identity, oracle) == truth
    assert sha256(truth).hexdigest() == digest

    changed_oracle = with_one_numeric_oracle_changed(oracle)
    changed_truth = canonical_truth_bytes(
        inputs, runtime_identity, changed_oracle
    )
    with pytest.raises(AssertionError, match="truth bytes"):
        verify_truth_bytes(changed_truth, truth, digest)
```

`fixed_golf_truth_parts()` 与 `with_one_numeric_oracle_changed()` 是测试文件内的固定纯 helper：前者返回最小但完整的 frozen inputs/runtime identity/高度+RTK+LiDAR oracle，后者深复制后只改变一个十六进制 float；它们不得调用生成器或生产模块。`require_wished_script_module/require_callable` 延续其他阶段四 RED 的延迟导入模式。

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_golf_truth_preservation.py -k truth_bytes_are_independent_of_provenance_hashes`

Expected: 正常收集后 `FAILED`，唯一根因是生成器或三个纯 helper 尚未实现；不得采样 PyBullet、创建 baseline、出现 collection error 或 skip。

- [ ] **Step 2: 在任何性能生产代码修改前冻结独立 golf oracle**

`scripts/generate_stage4_golf_truth_fixture.py` 是测试证据生成器，不是生产路径。它固定 `seed=23`、`relief=medium`、96x144 collision heightfield、`df_back`、固定初始位姿和轮速序列；用低层 PyBullet 查询独立采集 256 个固定网格点的高度/法向、固定时间点 base pose/contact、三点 RTK link 世界位置和 64 条固定世界射线命中，禁止调用将被优化的 profiler、Dashboard 抽样或 MID-360 生产 helper。

`golf_truth_v1.json` 是独立物理真值，canonical bytes 只包含冻结输入（seed/relief/碰撞尺寸/车型/初始位姿/轮速时序）、解释数值所必需的 PyBullet/runtime identity，以及实测高度/法向、pose/contact、RTK 和 ray oracle；使用 UTF-8 LF、稳定键顺序和十六进制 float。它严禁包含生成器版本或 `scene.py`、`mid360_lidar.py` 等会被本 Task 修改的源码 hash。`.sha256` 只哈希该 canonical truth JSON。

`golf_truth_v1.provenance.json` 单独记录生成器版本与脚本 hash、`slope_sim/scene.py`、`slope_sim/mid360_lidar.py`、`slope_sim/rtk_triplet.py`、`resources/models/robot_models.yaml` 和所用阶段四 URDF 的逐文件 SHA-256。它在每个实现候选后由 `--check` 原子重生成，用于说明当前代码如何复验冻结真值；它是审计证据，不参与 truth JSON 的 canonical bytes、`.sha256` 或数值 baseline 判定。

Run: `conda run -n slope-sim python scripts/generate_stage4_golf_truth_fixture.py --create --output tests/fixtures/stage4/golf_truth_v1.json --sha256-file tests/fixtures/stage4/golf_truth_v1.sha256 --provenance-file tests/fixtures/stage4/golf_truth_v1.provenance.json`

Expected: 三个文件以 exclusive-create 方式生成，命令打印 truth JSON SHA-256；把打印值逐字写入 `.sha256`。若任一目标已存在必须失败，禁止静默重录 baseline。

Run: `conda run -n slope-sim python scripts/generate_stage4_golf_truth_fixture.py --check --output tests/fixtures/stage4/golf_truth_v1.json --sha256-file tests/fixtures/stage4/golf_truth_v1.sha256 --provenance-file tests/fixtures/stage4/golf_truth_v1.provenance.json`

Expected: PASS，重采样 truth bytes 和 `.sha256` 完全一致，provenance 已原子更新为当前源码 hash。此后才允许修改 Task 7 的生产文件；任何有意更新数值 fixture 都必须独立评审，单纯源码 hash 变化只更新 provenance，不能重录物理 baseline。

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_golf_truth_preservation.py -k truth_bytes_are_independent_of_provenance_hashes`

Expected: PASS；两份不同 provenance 仍导出完全相同的 truth bytes/hash，而数值 oracle 改动被拒绝。至此才允许进入性能生产代码。

- [ ] **Step 3: 写 profiler、deadline 和真值保持 RED**

测试文件不得顶层导入尚不存在的 `slope_sim.performance`；在测试函数内检查模块/API，缺失时用明确断言得到正常 `FAILED`。先写以下聚合行为：

```python
def test_segment_statistics_report_p50_p95_p99_without_unbounded_history() -> None:
    stats = SegmentProfiler(window_size=512)
    for index in range(1000):
        stats.observe("physics_step", index / 1_000_000.0)
    report = stats.snapshot()
    assert report["physics_step"].sample_count == 1000
    assert report["physics_step"].retained_count == 512
    assert report["physics_step"].p50_sec <= report["physics_step"].p95_sec <= report["physics_step"].p99_sec
```

分段固定为 physics、lidar rays、RTK/IMU、transport enqueue、logger、camera、Qt snapshot、Matplotlib draw、wall total。再写 fake clock 测试锁定相机 30 Hz、Qt 60 Hz、Matplotlib 2 Hz 的绝对 deadline 和“超期不补跑”；写 Dashboard 2,048 点/20 秒有界历史、profiler 默认关闭且 240 Hz 热路径不写文件的断言。`test_golf_truth_preservation.py` 必须先验证 `.sha256` 与 fixture bytes，再运行相同固定输入并逐项比较，不允许在测试中自动重录 fixture。

Step 1 的 provenance 分离测试保留在本轮聚合回归中；测试必须通过生成器公开的纯 helper 构造证据，不能先改仓库源文件，也不能从 provenance 反向拼装 truth JSON。

- [ ] **Step 4: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_performance_budget.py tests/stage4/test_golf_truth_preservation.py`

Expected: 正常收集后 `FAILED`；至少 profiler/deadline 行为尚未实现，且 fixture hash 校验已经 PASS。不得出现 collection error、fixture error 或 skip。

- [ ] **Step 5: 实现有界 profiler 和固定轻量化措施**

先实现满足 RED 的最小 `SegmentProfiler`：默认关闭，开启时每段只保存最近 512 个样本、累计 sample count，不在 240 Hz 路径写磁盘。随后只落地以下已测试措施：

- 相机跟随使用独立绝对 deadline，最多 30 Hz。
- Qt 状态最多 60 Hz，Matplotlib 继续最多 2 Hz；超期只 `sleep(0)`，不补跑。
- PyBullet GUI 关闭 RGB/depth/segmentation preview 和 shadows，保留主体渲染。
- MID-360 R2 标量表、offset、line 和局部单位向量用 NumPy 批量生成；每帧只做一次位姿矩阵变换。
- Dashboard 只保留 2048 个显示点和 20 秒有界历史；eCAL/MCAP 完整点云不降采样。
- golf heightfield 的 96x144 碰撞数据、摩擦、出生点和语义高程保持不变；不以减少碰撞网格冒充 GUI 优化。

- [ ] **Step 6: 运行 GREEN 与冻结真值回归**

在固定 seed/relief 的网格采样 256 个 `(x,y)`，比较 Step 2 的冻结 fixture 高度/法向；再比较车辆轨迹、接触、RTK 和 LiDAR 抽样 oracle。高度误差 `<=1e-6m`、RTK/LiDAR `<=1e-4m`，离散 contact/body/link id 必须精确一致。

Run: `conda run -n slope-sim python scripts/generate_stage4_golf_truth_fixture.py --check --output tests/fixtures/stage4/golf_truth_v1.json --sha256-file tests/fixtures/stage4/golf_truth_v1.sha256 --provenance-file tests/fixtures/stage4/golf_truth_v1.provenance.json`

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_performance_budget.py tests/stage4/test_golf_truth_preservation.py tests/test_stage1_terrains.py`

Expected: 两条命令 PASS；truth fixture bytes/hash 未变化，provenance 精确反映当前实现，所有固定优化预算和真值容差满足。

- [ ] **Step 7: REFACTOR 计时上下文与 deadline 更新重复代码**

只抽取已经 GREEN 的 segment context、绝对 deadline 和有界 deque 操作；不调整预算、不改变采样/碰撞/显示数据量。随后原样重跑 Step 6 的两条命令，Expected: PASS 且 fixture SHA-256 不变。

- [ ] **Step 8: 生成隔离性能矩阵**

该脚本会串行运行 `2 terrain x 2 mode x 2 Dashboard x 2 LiDAR x 2 logging = 32` 个真实 PyBullet 高负载组合，每个组合精确 20 个障碍物、10 秒测量窗口（总测量窗口 320 秒，另加启动/清理时间），属于受控外部门禁。执行前必须向用户说明这 32 个组合和时长并取得只覆盖下一条矩阵命令的明确授权，随后即时扫描全机 pytest、GUI/Xvfb、PyBullet、eCAL 和系统负载；有竞争负载就不消费授权。脚本一次只启动一个组合，失败保留当前组合证据并停止，不自动跑后续组合或重跑；任何复测都重新授权和预检。

Run: `conda run -n slope-sim python scripts/profile_stage4_workload.py --terrain-matrix flat,golf_heightfield --mode-matrix direct,gui --dashboard-matrix off,on --lidar-matrix off,on --logging-matrix off,on --obstacle-count 20 --duration-sec 10 --output results/stage4/performance-matrix.json`

Expected: JSON 明确每一组合的 `obstacle_count=20`、sim/wall 和各段 p50/p95/p99；一次只运行一个组合，启动前扫描全机负载，不与 eCAL/GUI 门禁并发。缺失障碍物数量、实际数量不是 20 或脚本静默使用默认值时整份矩阵失败。

- [ ] **Step 9: 对代表性 golf GUI 组合做证据驱动收口**

从矩阵中单独读取 `golf_heightfield + GUI + Dashboard on + LiDAR on + logging on + obstacle_count=20`，先断言结果内实际障碍物数量精确为 20，再要求 `sim/wall=0.98..1.02`、GUI event gap `<=100ms`、Dashboard draw p95 `<100ms`。任一未达标时按最大 p95 段一次只做一项优化：优先减少 golf 视觉 mesh/增加视觉 LOD、删除相机/Qt/Matplotlib 无效重绘、消除点云显示复制；只有 profiler 证明 collision heightfield 主导时才允许实验碰撞分辨率。每次候选都先补行为/真值 RED，再重跑 Step 6 的冻结 fixture；完全相同的单组合 profile 每次都视为新的外部 invocation，必须重新说明候选、负载和时长，取得单条授权并即时预检。失败保留证据并停止，不自动重跑；没有改善则撤销该候选，不能叠加多项后猜原因。

Run once for each separately authorized candidate:

```bash
test -n "${STAGE4_PROFILE_CANDIDATE_ID:-}"
conda run -n slope-sim python scripts/profile_stage4_workload.py \
  --terrain-matrix golf_heightfield \
  --mode-matrix gui \
  --dashboard-matrix on \
  --lidar-matrix on \
  --logging-matrix on \
  --obstacle-count 20 \
  --duration-sec 10 \
  --output "results/stage4/performance-${STAGE4_PROFILE_CANDIDATE_ID}.json"
```

Expected per candidate: JSON 只含一个代表性组合，明确记录候选 ID、`obstacle_count=20`、10 秒窗口、sim/wall、GUI gap 和各段 p50/p95/p99；命令失败即停止，该授权不能用于修订后的再次运行。

Expected: 代表性组合达到三项硬门，且 golf 高度/法向、接触、轨迹、RTK、LiDAR truth fixture bytes/hash 不变、provenance 已更新到最终候选。此处只证明 Python/GUI 本地负载；C/E 仍须在真实 eCAL+C++ Recorder 联合负载中复验相同门槛。

## Task 8：四车型三地形与四分辨率门禁

**Files:**
- Create: `scripts/verify_stage4_sensors.py`
- Modify: `scripts/verify_dashboard_manual_drive.py`
- Modify: `docs/阶段四交付报告.md`
- Test: `tests/stage4/test_stage4_sensor_verifier.py`
- Test: `tests/test_dashboard_manual_verifier.py`

- [ ] **Step 1: 写 DIRECT 结果 schema 与 GUI stage4 行为 RED**

```python
def test_verifier_case_exposes_exact_v2_runtime_contract(case_result) -> None:
    assert case_result["protocol_version"] == 2
    assert case_result["runtime_channels"] == [
        ["/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand", 100, "subscribe"],
        ["/sim/wheel/state", "slope_sim.interfaces.v2.WheelState", 100, "publish"],
        ["/sim/lidar/points", "slope_sim.interfaces.v2.LidarPointCloud", 10, "publish"],
        ["/sim/rtk/state", "slope_sim.interfaces.v2.RtkState", 10, "publish"],
        ["/sim/imu/attitude", "slope_sim.interfaces.v2.ImuAttitude", 10, "publish"],
    ]
    assert case_result["output_model_types"] == {
        "/sim/lidar/points": "LidarPointCloudV2",
        "/sim/rtk/state": "RtkStateV2",
        "/sim/imu/attitude": "ImuAttitudeV2",
    }
    assert case_result["legacy_lidar_topics_observed"] == []
```

结果逐 case 还必须保存模型、地形、simulation session、descriptor SHA、scene hash、scan seed、LiDAR ray/hit/self-hit、RTK 三点误差、IMU、trajectory、分段性能和失败原因；summary 只有全部 case 通过时 rc=0。verifier 用 `V2RuntimeConfig.local_for_test()` 和 fake/local raw transport 跑 DIRECT，不得为方便复用 `InterfaceConfig.default()`。

测试用固定内存 case fixture 调用 verifier 的纯 oracle；测试函数内先断言 `scripts/verify_stage4_sensors.py` 存在，再通过 subprocess/公开解析入口检查正反例。缺脚本时必须得到明确 `FAILED`，不能在 fixture setup、顶层 import 或 subprocess 文件错误中报 ERROR。

在修改 `scripts/verify_dashboard_manual_drive.py` 前，先扩展 `tests/test_dashboard_manual_verifier.py`：无 GUI 地调用现有 `parse_args()` 与 `build_child_command()`，断言 `--stage4` 被解析为显式布尔模式，child command 选择 schema v2 场景与显式 v2 local 测试配置，并继续传递日志、布局报告和窗口 token；再用固定的 v2 布局报告 fixture 断言单中心 LiDAR、三点 RTK 和 IMU 页面 oracle，且 legacy 前/后 LiDAR 字段或页面会失败。若 `--stage4`/v2 布局 API 尚不存在，测试捕获 parser/API 缺失并调用 `pytest.fail("stage4 dashboard verifier behavior is not implemented", pytrace=False)`，不得让 argparse `SystemExit` 或属性错误冒充 RED。

- [ ] **Step 2: 运行 DIRECT 与 GUI verifier RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_sensor_verifier.py tests/test_dashboard_manual_verifier.py`

Expected: 两个测试文件都正常收集并 `FAILED`，失败断言只指向 DIRECT 结果 schema/case oracle/summary 或 stage4 Dashboard parser、child command、v2 页面 oracle 尚未实现；不得是 argparse/属性/fixture error，不得启动 PyBullet GUI、真实 eCAL 或完整 4x3 矩阵。保存失败输出后才可进入 Step 3。

- [ ] **Step 3: 实现最小 DIRECT verifier 与 GUI stage4 参数**

`verify_stage4_sensors.py` 只组合 B 已有 runtime/传感器/profiler API：每个 case 创建全新 simulation session 和临时结果目录，冻结输入模型/地形/seed，运行一个 DIRECT case 后把原始统计交给纯 oracle，最后以原子替换写入规范 JSON。任何 case 失败仍执行清理，但不自动重跑；summary 由 12 个 case 结果派生，不能覆盖失败。`verify_dashboard_manual_drive.py --stage4` 只增加显式 v2 配置、单中心 LiDAR/三点 RTK/IMU 页和布局 oracle，不改变阶段三默认路径。

- [ ] **Step 4: 运行 DIRECT 与 GUI verifier GREEN，再做 REFACTOR 复验**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_sensor_verifier.py tests/test_dashboard_manual_verifier.py`

Expected: PASS；缺字段、旧 LiDAR topic、错误模型类型、RTK 超差、假运动、频率/性能越界、单 case 失败但 summary 伪通过，以及 stage4 参数丢失、错误 scene/config、legacy 双 LiDAR 页面等反例均精确失败。完成必要整理后原样重跑；无需整理则记录“REFACTOR：无必要”。只有本步两份测试都 GREEN 后才可运行 Step 5/6 的真实矩阵和 GUI。

- [ ] **Step 5: 跑 4x3 DIRECT**

Run: `conda run -n slope-sim python scripts/verify_stage4_sensors.py --all-models --all-terrains --output results/stage4/sensor-matrix.json`

Expected: `SUMMARY pass=12 fail=0`；每 case 都报告同一五通道 v2 合同、三个精确 v2 输出模型、零 legacy LiDAR topic、一个中心雷达，RTK 三点误差 `<=1e-4m`。

- [ ] **Step 6: 严格串行跑 GUI**

下面四条命令不得批量授权。每条执行前都向用户说明真实桌面或临时 Xvfb、分辨率和 4 秒时长，取得只覆盖紧随命令的明确授权，并即时扫描全机 pytest、GUI/Xvfb、PyBullet、eCAL 和系统负载；上一条授权不能复用。任一失败都保留证据并停止，不能继续下一条或自动重跑；候选复测同样重新授权。长期 `:1` 只启动本 verifier，不操作其他桌面进程。

Run one at a time:

```bash
DISPLAY=:1 XAUTHORITY=/home/cancade/.Xauthority conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --stage4 --verify-window-layout --verify-dashboard-tabs --duration-sec 4
xvfb-run -a -s "-screen 0 1366x768x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --stage4 --expected-available-size 1366x768 --duration-sec 4
xvfb-run -a -s "-screen 0 1920x1080x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --stage4 --expected-available-size 1920x1080 --duration-sec 4
xvfb-run -a -s "-screen 0 2560x1440x24" conda run -n slope-sim python scripts/verify_dashboard_manual_drive.py --stage4 --expected-available-size 2560x1440 --duration-sec 4
```

Expected: 四条各 rc=0、15/15 页、Dashboard 精确 33%、内部 50:50、DPR/滚动/点击/文字/tick/legend/artist 全部通过，临时 Xvfb 清理完成。

- [ ] **Step 7: 更新报告但不提前声明 eCAL 通过**

记录 DIRECT、GUI、性能矩阵命令、结果、主机配置和证据路径；明确写出“B 的 formal runtime 已接到 A 的 `create_v2_ecal_transport`，但 B 未运行真实 eCAL/C++/Recorder 联合门禁”，后者仍标为未执行并交给子计划 C/E，不能把 fake/local 五通道证明写成真实 transport 通过。

## 阶段 B 完成定义

- Task 5 的 `LidarPointCloudV2/RtkStateV2/ImuAttitudeV2`（及嵌套点类型）冻结、自校验，确定性 encode 与严格 decode 全部通过，A 的 wheel/golden bytes 不变。
- 正式 `V2RuntimeConfig.production()` 精确包含 A 的五个 `V2_TOPICS`，没有 `lidar_front/lidar_rear`，也不允许 `auto`；v1 `InterfaceConfig` 只留在显式历史路径。
- schema v2 入口只调用 A 的 `create_v2_ecal_transport()` 和 `V2RuntimeProtocol`；每个 output 先占 sequence，再生成模型，日志与 transport 共享同一 encoded bytes，连接刷新仍先 poll 后 snapshot。
- 四车型三地形 DIRECT、四个 GUI 分辨率和隔离性能矩阵通过；代表性 golf GUI 组合达到 `sim/wall`、GUI event gap 和 Dashboard draw p95 硬门。阶段三 v1 聚焦回归通过，报告严格保留 C/E 的真实 eCAL、C++ Recorder 与发行联合门禁为未执行。
