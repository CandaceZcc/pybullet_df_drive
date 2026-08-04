# 阶段四 E：发行包与最终验收 Implementation Plan

> **Execution:** Use `subagent-driven-development` only when the user selects delegated execution; otherwise use `executing-plans`. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把已通过协议、传感器、C++、记录和显示门禁的阶段四系统制作成 Ubuntu 24.04 amd64 离线可安装发行包，并完成串行联合负载、升级回退、干净机迁移和独立六维审查。

**Architecture:** `slope-sim` 编排器通过权限 0700 的 Unix control socket 管理 Simulator、Subscriber、Recorder、Command 和可选 ROS Bridge；资源统一从同一 release root 的 `share/slope-sim` 解析。发行包的唯一布局为 `bin/lib/include/share/runtime/python/ros-overlay`：E 只消费总计划 Task 2 已冻结的 Python unified/explicit lock、micromamba、canonical package cache、官方 eCAL wheel cache 和 C++/ROS source archive cache，先对纯 Conda runtime 执行 conda-pack，再在 staging 离线安装 eCAL wheel 与项目 wheel；每轮从只读源码归档独立重建 C++ 依赖，C++ ELF 由 CMake 安装到根 `bin/lib`，可选 Jazzy Bridge 安装进 ROS overlay。不包含开发仓库、Conda/pip 缓存、reference 仓库或 MID-360 官方样例。

**Tech Stack:** Python 3.10、micromamba `2.8.1-1`、conda-pack `0.9.2`、C++17/CMake/CPack、tar+Zstd、systemd user units、XDG、pytest/CTest/colcon、eCAL 6.1.1、ROS 2 Jazzy。

---

**TDD gate:** 本计划所有生产代码任务遵守总路线的严格 RED-GREEN-REFACTOR 协议；RED 必须是测试正常收集后的行为断言失败，不能是缺包、缺工具、collection error、fixture error、缺构建目录或 skip。测试只在测试函数内 import 尚未创建的 Python 模块；CLI/shell 入口通过 subprocess 测试时先用明确断言检查 wished-for 文件，并把缺文件转换为 `FAILED`。每个 GREEN 后原样复跑聚焦命令，REFACTOR 不增加行为。

## Task 1：安装后资源解析与正式 CLI

**Files:**
- Create: `slope_sim/resources.py`
- Create: `slope_sim/cli.py`
- Create: `slope_sim/__main__.py`
- Create: `packaging/bin/slope-sim`
- Modify: `slope_sim/model_registry.py`
- Modify: `slope_sim/interfaces/v2/descriptor.py`
- Modify: `pyproject.toml`
- Test: `tests/stage4/test_resource_resolution.py`
- Test: `tests/stage4/test_stage4_cli.py`

- [ ] **Step 1: 写脱离仓库 RED**

```python
def test_resource_root_comes_from_installed_share(monkeypatch, tmp_path) -> None:
    share = tmp_path / "share" / "slope-sim"
    (share / "urdf").mkdir(parents=True)
    monkeypatch.setenv("SLOPE_SIM_SHARE_DIR", str(share))
    assert resource_path("urdf/df_back.urdf") == share / "urdf/df_back.urdf"


def test_resource_path_rejects_escape(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("SLOPE_SIM_SHARE_DIR", str(tmp_path))
    with pytest.raises(ValueError, match="relative resource path"):
        resource_path("../secret")


def test_resource_path_rejects_symlink_escape(tmp_path, monkeypatch) -> None:
    share = tmp_path / "share"
    outside = tmp_path / "outside"
    share.mkdir()
    outside.mkdir()
    (outside / "secret").write_text("private", encoding="utf-8")
    (share / "escaped").symlink_to(outside, target_is_directory=True)
    monkeypatch.setenv("SLOPE_SIM_SHARE_DIR", str(share))
    with pytest.raises(ValueError, match="outside resource root"):
        resource_path("escaped/secret")
```

同一 RED 把 `packaging/bin/slope-sim` 复制到两个不同的临时 release root，并提供最小 fake `runtime/python/bin/python`；断言 launcher 只由自身 `readlink -f` 后的位置设置 `SLOPE_SIM_ROOT/SLOPE_SIM_SHARE_DIR`，且原路径搬走后从新位置仍能执行。测试函数先断言 launcher 源文件存在，缺失时得到明确 `FAILED`，不能在 fixture setup 报错。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_resource_resolution.py tests/stage4/test_stage4_cli.py`

Expected: pytest 正常收集并 `FAILED`，失败断言指向资源入口、正式 CLI 或 relocatable launcher 尚未实现；不得是顶层 import、fixture setup 或缺文件异常。

- [ ] **Step 3: 实现单一资源入口**

```python
def resource_root() -> Path:
    configured = os.environ.get("SLOPE_SIM_SHARE_DIR")
    if configured:
        return Path(configured).resolve(strict=True)
    release_root = os.environ.get("SLOPE_SIM_ROOT")
    if not release_root:
        raise RuntimeError("SLOPE_SIM_ROOT is required; start through the installed launcher")
    return (Path(release_root) / "share" / "slope-sim").resolve(strict=True)


def resource_path(relative: str) -> Path:
    candidate = PurePosixPath(relative)
    if candidate.is_absolute() or ".." in candidate.parts:
        raise ValueError("resource must be a relative resource path")
    root = resource_root()
    resolved = (root / candidate).resolve(strict=True)
    if not resolved.is_relative_to(root):
        raise ValueError("resource resolves outside resource root")
    return resolved
```

根 `bin/slope-sim` launcher 按自身真实路径设置 `SLOPE_SIM_ROOT` 和 `SLOPE_SIM_SHARE_DIR`，再执行 `runtime/python/bin/python -m slope_sim`；所有 URDF、纹理、默认场景、`ecal.yaml`、RViz2、`robot_models.yaml`、v2/control `.proto`、descriptor set 和 SHA manifest 改经此入口。`descriptor.py` 默认调用 `resource_path("interfaces/slope_sim_interfaces_v2.desc")` 与对应 manifest，不再按 `__file__.parents[3]` 猜源码树。源码开发测试显式设置临时 share，禁止 fallback 到仓库根目录或“Python 包旁边碰巧存在的 share”。

- [ ] **Step 4: 增加 CLI entry points**

```toml
[project.scripts]
slope-sim = "slope_sim.cli:main"
```

Python 只拥有编排入口 `slope-sim`。`slope-sim-sub`、`slope-sim-command`、`slope-sim-record`、`slope-sim-replay`、`slope-sim-export` 由 CMake 安装为根 `bin/` 下的 C++ ELF；编排器以 release root 解析并 `execve` 这些 sibling executable，禁止再注册同名 Python wrapper 或递归调用自己。

`slope-sim` parser 必须完整实现：`start interactive|headless`、`status`、`stop`、只读 `doctor [--json PATH]`、`service enable|disable|status`。`doctor` 检查 manifest/checksum、Python import、C++ ELF/RUNPATH、share、eCAL 和可选 ROS overlay，不修改系统；`service enable/disable` 只在用户显式调用时执行 `systemctl --user`，`status` 只读。未知参数 rc=2，运行/自检失败 rc=1，成功 rc=0。

- [ ] **Step 5: 运行 GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_resource_resolution.py tests/stage4/test_stage4_cli.py tests/test_entrypoints.py`

Expected: PASS；额外断言 wheel 中只有一个 Python project script，五个 C++ 工具从安装 prefix 脱离源码运行 `--version`，且编排器不会解析到 Python 同名 wrapper。

- [ ] **Step 6: REFACTOR 资源解析与 CLI 装配并原样复验**

只整理已经 GREEN 的 release-root 解析、parser 分派和 sibling ELF 定位重复；不得新增 PATH fallback、第二个 launcher 或未测试命令。无需整理时记录“REFACTOR：无必要”，随后原样重跑 Step 5。

## Task 2：本地控制 socket 与多进程编排

**Files:**
- Create: `slope_sim/control_socket.py`
- Create: `slope_sim/orchestrator.py`
- Create: `resources/ecal/ecal.yaml`
- Test: `tests/stage4/test_control_socket.py`
- Test: `tests/stage4/test_orchestrator.py`

- [ ] **Step 1: 写权限、身份和关闭顺序 RED**

```python
import stat


def test_session_socket_directory_is_private(tmp_path) -> None:
    endpoint = create_control_endpoint(tmp_path, simulation_session_id=b"x" * 16)
    assert stat.S_IMODE(endpoint.parent.stat().st_mode) == 0o700


def assert_happens_before(events: list[str], first: str, second: str) -> None:
    assert events.index(first) < events.index(second)


def test_normal_stop_is_ordered(fake_processes) -> None:
    orchestrator = Orchestrator(fake_processes)
    orchestrator.stop()
    events = fake_processes.events
    assert_happens_before(events, "simulator.capture_end_boundary", "command.begin_100hz_zero_drain")
    assert_happens_before(events, "command.begin_100hz_zero_drain", "command.publish_and_freeze_final_zero_fence")
    assert_happens_before(events, "simulator.capture_end_boundary", "simulator.publish_and_freeze_four_output_fences")
    assert_happens_before(events, "command.publish_and_freeze_final_zero_fence", "simulator.send_end_barrier")
    assert_happens_before(events, "simulator.publish_and_freeze_four_output_fences", "simulator.send_end_barrier")
    assert_happens_before(events, "simulator.send_end_barrier", "recorder.flushed_and_finalized")
    assert_happens_before(events, "recorder.flushed_and_finalized", "command.stop")
    assert_happens_before(events, "command.stop", "consumers.stop")
    assert_happens_before(events, "consumers.stop", "simulator.stop")


def test_second_formal_session_fails_before_creating_ecal_resources(runtime_root) -> None:
    first = Orchestrator(runtime_root=runtime_root, session_id=b"a" * 16)
    first.acquire_production_lock()
    second = Orchestrator(runtime_root=runtime_root, session_id=b"b" * 16)
    with pytest.raises(SessionAlreadyRunning):
        second.start()
    assert second.ecal_resource_count == 0


def test_scene_rebuild_commits_physics_before_attachment_ack(fake_processes) -> None:
    orchestrator = Orchestrator(fake_processes)
    orchestrator.rebuild_scene("golf_heightfield")
    assert fake_processes.events == [
        "simulator.freeze_four_outputs",
        "command.zero_and_freeze",
        "simulator.commit_physical_world",
        "recorder.persist_canonical_attachment",
        "recorder.ack_attachment",
        "simulator.resume_new_world_outputs",
        "command.wait_for_new_wheel_state_then_reclaim",
    ]
```

覆盖 stale socket、错误 uid/PID/role/session/descriptor、重复 ready、Recorder fatal、子进程提前退出、超时 kill、幂等 stop、两个 orchestrator 使用相同或不同随机 session。主机级 `production.lock` 用非阻塞 `flock` 持有整个 participant 生命周期，进程死亡后内核自动释放；诊断文件中的旧 PID 不能被误当成仍持锁，也不能在锁未取得前创建任何 eCAL resource。正常停止只断言 producer freeze/barrier/finalize/退出的必要偏序；另复用 C 的 Recorder fixture 参数化五话题 fence 在 barrier 前后混合到达和位于 raw、ordered DEFERRED、frontier 后 READY、rotation-held READY、writer、written，并覆盖更早 REJECTED gap，禁止用总事件列表伪造 control/eCAL 的跨通道顺序或跳过 blocked frontier。

同一 RED 驱动 Dashboard 的线速度/角速度与键盘状态：编排器每 20ms 向唯一 C++ Command 连接刷新 `ManualTwistTarget`，release/失焦/stop 立即发零；fake Command 断言 `ControlEnvelope.request_id` 严格递增、目标租约不超过 100ms，连接断开或 100ms 未刷新后目标为零。测试还要证明 Python/Dashboard 没有创建 `/sim/wheel/command` publisher，自动 motion recipe 和人工控制走同一 Command API。

场景 RED 同时覆盖物理 commit 失败时回滚旧世界且不发送 attachment、attachment ACK 失败时新世界不发布并令正式会话 FAILED、连续 transition id/revision/generation，以及冻结前最后命令的条件存在性：本 session/generation 已发布命令时 `SceneCommandFrozen.last_command` 必须 present 且精确匹配；尚未认领且发布计数为 0 时允许 absent，但 freeze 仍须 ACK。恢复只解除本地 freeze，Command 必须先读取新 world/command generation 的 CLAIMABLE `WheelState` 才能重新认领。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_control_socket.py tests/stage4/test_orchestrator.py`

Expected: pytest 正常收集并 `FAILED`，失败断言指向 socket 权限、身份校验或有序关闭状态机尚未实现；不得启动真实子进程。

- [ ] **Step 3: 消费 C 计划已冻结的 control protobuf 与 framing**

```python
HEADER = struct.Struct("!I")
MAX_CONTROL_MESSAGE_BYTES = 1 << 20


def send_control_message(sock: socket.socket, payload: bytes) -> None:
    if not payload or len(payload) > MAX_CONTROL_MESSAGE_BYTES:
        raise ValueError("invalid control payload length")
    sock.sendall(HEADER.pack(len(payload)) + payload)
```

正式 `/sim` producer 在创建 participant 前先锁定 `$XDG_RUNTIME_DIR/slope-sim/production.lock`；锁文件 mode 0600、父目录 0700，锁内容只作 PID/session 诊断，互斥以仍持有的文件描述符为准。socket path 为 `$XDG_RUNTIME_DIR/slope-sim/<simulation-session-hex>/control.sock`；校验 peer uid 后，还必须把 `SO_PEERCRED.pid` 与编排器实际 spawn 的 PID 和唯一 role 注册表匹配，所有状态携带 simulation session、descriptor SHA、role、state、错误和队列健康。

这里不得重新设计或重新生成消息：直接消费 C 计划由 `scripts/generate_control_protos.py` 冻结的 `slope_sim_control_v1_pb2.py`、control descriptor 和 C++ golden，并复用其 network-order uint32 + 1 MiB 上限。测试逐 byte 验证 STARTING/READY/ACTIVE/ROTATING/DRAINING/FINALIZED/FAILED、逐 topic `TopicHealth`、scene attachment ACK、segment barrier 和 end-barrier fence；每个生产 role 必须先用 STARTING 承载 WAITING/PENDING health，STARTING 不开业务门，全部必需 topic VERIFIED 后才转 READY。stream reader 允许 fragmented frame 和同一次读取中的连续两帧，未知版本、截断、超长、错误 request id/session/descriptor、单帧 fixture 非法残留、跳过 READY 或状态回退均在启动业务进程前失败。

- [ ] **Step 4: 实现 interactive/headless profile**

- interactive：GUI Simulator、Dashboard、C++ Subscriber、正式 Recorder，可选 Command、ROS Bridge/RViz2。
- headless：DIRECT Simulator、C++ Subscriber、正式 Recorder，可选 Command/Bridge，无 Qt/PyBullet GUI。
- 正式 profile 至少启用 Simulator、Subscriber、Command、Recorder；ROS-on 再启用 Bridge。每个已启用 role 通过 control 身份校验后先报告 STARTING 和完整 pre-READY health；全部必需 topic 都达到 `VERIFIED` 后才报告 READY，Simulator 才能占用 sequence 0。调试 profile 显式省略的 role 不伪造 STARTING/READY，`--no-record` 模式持续显示“未记录”。
- interactive 的 Dashboard/键盘只产生本地 `ManualTwistTarget`，编排器以 20ms 周期续租，C++ Command 根据当前 `WheelState.robot_model` 和 canonical 模型参数换算轮命令。按键释放、窗口失焦、租约过期、control 连接断开、authority 变化、scene freeze 和 stop 都必须归零；Python 侧永不创建第二个 wheel command publisher。headless 自动验收以固定 motion recipe 调用同一 target 入口。
- 正常停止先由 Simulator 主线程捕获正式 end boundary，再让 Command 保持当前 session/generation 以 100 Hz 发送零命令至少 100 ms；Command 发布首条越过阈值的零命令后原子保存 fence、冻结自身 publisher 并保持 participant 存活。Simulator 再发布并冻结四个输出 fence；只有五个 publisher 均已冻结才发送 `EndBarrier`。由于 control/eCAL 无跨通道顺序，Recorder 在 barrier 后继续接收不晚于各 topic required fence 的在途 pair，逐 topic 看见 fence 后关 ingress；越界/重复/缺 fence失败。五条均收齐后仍须从 `next_commit_order` 连续写 READY、审计跳过 REJECTED，任何更早 DEFERRED 都阻塞 `settled_frontier` 并在不可解析时令会话失败；只有 frontier 跨过全部 fence且 raw/ordered/rotation 状态全空，才完成全部 segment/session manifest 并 FINALIZED，最后退出 Command 和其余 consumer。

- [ ] **Step 5: 运行 GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_control_socket.py tests/stage4/test_orchestrator.py`

Expected: PASS，无残留 socket/子进程。

- [ ] **Step 6: REFACTOR 控制帧与进程生命周期并原样复验**

只抽取已覆盖的 framing、role registry、超时和幂等关闭重复；不得改变 READY 门、producer fence 偏序或异常停车语义。无需整理时记录“REFACTOR：无必要”，随后原样重跑 Step 5。

## Task 3：可复现离线发行安装树

**Files:**
- Create: `packaging/build_release.sh`
- Create: `packaging/build-source-manifest.yml`
- Create: `packaging/materialize_build_source.py`
- Create: `packaging/materialize_release_tree.py`
- Create: `packaging/relocate_python_runtime.sh`
- Create: `packaging/bin/slope-sim-ros-bridge`
- Create: `packaging/manifest.yml`
- Modify: `pyproject.toml`
- Consume read-only: `packaging/python-environment.yml`
- Consume read-only: `packaging/python-toolchain-environment.yml`
- Consume read-only: `packaging/locks/virtual-packages.yml`
- Consume read-only: `packaging/locks/python.conda-lock.yml`
- Consume read-only: `packaging/locks/python-linux-64.lock`
- Consume read-only: `packaging/locks/python-toolchain.conda-lock.yml`
- Consume read-only: `packaging/locks/python-toolchain-linux-64.lock`
- Consume read-only: `packaging/locks/python-toolchain.lock`
- Consume read-only: `packaging/locks/python-package-cache.manifest.json`
- Consume read-only: `packaging/locks/python-wheel-cache.manifest.json`
- Consume read-only: `packaging/locks/source-archive-cache.manifest.json`
- Consume read-only: `packaging/locks/cpp-dependencies.lock`
- Consume read-only: `packaging/locks/ros2-dependencies.lock`
- Consume read-only: `packaging/build_python_runtime.sh`
- Consume read-only: `packaging/build_dependencies.sh`
- Consume read-only: `packaging/build_ros_overlay.sh`
- Consume read-only: `packaging/run_network_isolated.sh`
- Consume read-only: `scripts/verify_python_lock_cache.py`
- Consume read-only: `scripts/verify_python_wheel_cache.py`
- Consume read-only: `scripts/verify_stage4_source_cache.py`
- Create: `packaging/desktop/slope-sim.desktop`
- Create: `packaging/systemd/slope-sim-headless.service`
- Create: `scripts/verify_stage4_release.py`
- Test: `tests/stage4/test_release_manifest.py`
- Test: `tests/stage4/test_release_build_paths.py`
- Test: `tests/stage4/test_ros_launcher_relocation.py`

- [ ] **Step 1: 写冻结 Python 输入、路径隔离和打包顺序 RED**

```python
FORBIDDEN_RELEASE_PARTS = (
    ".git",
    "references/repos",
    ".pytest_cache",
    "Indoor_sampledata.lvx2",
    "/home/cancade",
    "miniforge3/envs/slope-sim",
)


def test_install_tree_has_no_development_paths(stage4_install_tree) -> None:
    names_and_text = stage4_install_tree.names_and_utf8_text()
    assert not any(
        part in value
        for part in FORBIDDEN_RELEASE_PARTS
        for value in names_and_text
    )


SEMVER_PRERELEASE_IDENTIFIER = (
    r"(?:0|[1-9][0-9]*|[0-9A-Za-z-]*[A-Za-z-][0-9A-Za-z-]*)"
)
RELEASE_VERSION_PATTERN = re.compile(
    rf"(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)"
    rf"(?:-{SEMVER_PRERELEASE_IDENTIFIER}"
    rf"(?:\.{SEMVER_PRERELEASE_IDENTIFIER})*)?"
    r"(?:\+[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?"
)
MAX_RELEASE_VERSION_BYTES = 128


@pytest.mark.parametrize(
    "release_version",
    [
        "",
        ".",
        "..",
        "1.2",
        "v1.2.3",
        "01.2.3",
        "1.02.3",
        "1.2.03",
        "../1.2.3",
        "1/2/3",
        "1.2.3 rc1",
        "1.2.3\n",
        "1.2.3\x1f",
        "1.2.3+" + "a" * 123,
    ],
)
def test_invalid_release_version_fails_before_output(
    release_builder, tmp_path, release_version
) -> None:
    work_root = tmp_path / "work-must-not-exist"
    output_dir = tmp_path / "output-must-not-exist"
    result = release_builder(
        release_version=release_version,
        work_root=work_root,
        output_dir=output_dir,
    )
    assert result.returncode != 0
    assert not work_root.exists()
    assert not output_dir.exists()


def test_build_smoke_commits_state_after_precommit_health_probe(
    release_smoke_fixture,
) -> None:
    result = release_smoke_fixture.run()
    assert result.events == [
        "relocate_runtime",
        "run_cli_sdk_selftest_and_loader_checks",
        "run_precommit_health_probe_without_install_state",
        "atomic_write_final_install_state",
        "run_doctor_from_final_install_state",
    ]
    assert result.install_state_write_count == 1
    assert "doctor" not in result.install_state
    assert result.install_state["health"] == result.precommit_health
    assert result.doctor["health"] == result.install_state["health"]
    assert result.doctor["provenance"]["kind"] == "build_smoke"
```

Manifest 测试要求 release version、Git SHA、descriptor SHA、Python/C++ eCAL、Protobuf、compiler ABI、MCAP/Zstd commit、Python runtime/toolchain lock SHA、canonical Python package/wheel cache tree digest、canonical C++/ROS source archive manifest/tree digest、micromamba binary SHA、由无环控制清单负责的每个不可变安装文件 SHA-256、SPDX license 全部是具体值。release version 的唯一合法表示是上面完整匹配、长度不超过 128 ASCII bytes 的 canonical SemVer 单路径分量；不得 trim、大小写折叠或把非法输入规范化成合法值。正例固定覆盖 `0.0.0`、`1.2.3` 和 `1.2.3-rc.1+build.5` 并要求逐 byte 原样保留；直接 parser 反例另覆盖 NUL，其余 subprocess 反例覆盖 `/`、`.`、`..`、控制字符、换行、空白、超长和非规范版本，全部在创建 work/output/evidence、归档顶层目录或安装路径前失败。控制文件不得自引用：SPDX file inventory 不含自身及下游控制文件，`release-manifest.json` 的 file table 不含自身和 `SHA256SUMS`，`SHA256SUMS` 覆盖归档内除自身外的全部普通文件；三者的唯一性、成员并集和摘要边由 Task 6 的独立结构化 verifier 复核。另用参数测试拒绝相对 `--work-root/--output-dir/--micromamba/--python-package-cache/--python-wheel-cache/--source-archive-cache`、非空工作根、任意输入/输出路径相同或互相包含、位于仓库内或包含仓库的 work/output/evidence 路径、固定仓库 `build/`/`results/` 路径、开发机 Conda/Mamba/pip/source cache 和输出目录中的未知旧归档。两个不同绝对 work root 的 fixture 还要逐 byte 扫描 ELF、CMake/pkg-config、wheel、`.pyc`、self-test、SBOM 和 manifest，注入源/构建/cache 路径泄漏、timestamp 漂移、pyc `co_filename` 漂移和 `conda-meta/history` 绝对创建路径时必须失败。

本 Task 不再测试或实现求解器。fixture 先通过总计划 Task 2 的本地 fake channel/package/wheel/source archive 产物喂给 `--stage-only`，并用 subprocess/event trace 证明 E 从未运行 conda-lock、render、solve、repoquery、download、Git fetch、pip index 或读取用户 cache；只允许 exact explicit lock 加 micromamba `create --offline`、两个本地 wheel 的 pip `--no-index --no-deps --no-compile`，以及从冻结 source artifact 私有 materialize。RED 还要拒绝：仅有 `--offline` 而没有已验证的外部断网 namespace/VM；把 canonical 的 URL 嵌套目录直接复制成 micromamba cache、native 根级归档缺失/多余/hash 漂移、`urls.txt` 漂移或同 basename 不同 hash；官方 eCAL wheel 缺失/篡改/错误 ABI/tag/license/NOTICE/RECORD/ELF inventory、pip 联网 fallback；A/B 共用可写 `mamba-root/pkgs`、wheel 副本、tool env 或 runtime env；source archive manifest/cache 缺失、多余、size/hash/tree digest/consumer 漂移、cache 链接、恶意 archive member、同 basename 不同 hash、直接在 canonical root 解包、A/B 共用可写 source archive 副本/解包树或缺包联网 fallback；把只读 Git `source-snapshot`、stage-only `development-snapshot` 或开发 worktree 直接交给 setuptools/CMake/ROS，`egg_info` 写入 immutable input，`source-build` 在构建前与输入摘要不一致，A/B 共用同一可写 source-build，或构建前后 immutable input digest/权限/成员变化；在 conda-pack 前删除 `conda-meta/files` 管理的任一 `.pyc` 或安装任一 wheel；缺失完整 native package cache；任一 wheel 遗留带绝对 URL 的 `direct_url.json`、prefix-bearing console script 或陈旧 `RECORD`；builder/source/cache 路径出现在 `conda-unpack` prefix records；Python/C++ 进程交叉或重复加载 eCAL/libprotobuf；no-participant loader 调用任一 eCAL Initialize/pub/sub/entity API 或造成 participant/topic entity census 增量；以及对原 packed root 运行 `conda-unpack`。build-smoke event RED 逐个在 relocation、CLI/SDK/self-test/loader、precommit health probe、state 原子写入和 final doctor 处注入失败：最终 `install-state.json` 在 state 提交前任一步失败时必须不存在，普通 doctor 不得在完成 state 前运行，state 只能原子写一次且不能保存未来 doctor 输出；final doctor 失败时 state 可作为诊断保留但 stage evidence 必须不存在，成功时 doctor 必须读取该完成 state、报告 `build_smoke` 并与 state 内由纯 probe 产生的规范 health 字段相同。开发快照 RED 还必须覆盖 dirty tracked 当前 bytes、允许清单内 dirty untracked 当前 bytes都被精确纳入，`.git`、ignored cache、`references/repos`、work/output/evidence 与未声明路径绝不进入，输入位于仓库内造成 self-inclusion 时在创建输出前失败，并用复制前后 stat/hash 检出并发变异。release-tree fixture 还必须接受根内相对 file symlink、directory symlink 和 hardlink 并深拷贝为零链接树，覆盖锁定 Conda runtime 的 `lib/python3.1 -> python3.10`、`lib/terminfo -> ../share/terminfo` 形态；绝对/逃逸/悬空/循环链接、特殊节点、展开超限、复制竞态、`renameat2` 不可用或任一事务中断必须失败并保留可诊断树。fixture 在测试函数内先断言 `build_release.sh`/manifest/launcher 源存在；缺入口必须报告断言 `FAILED`，不能在 fixture setup 报 ERROR。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_release_manifest.py tests/stage4/test_release_build_paths.py tests/stage4/test_ros_launcher_relocation.py`

Expected: pytest 正常收集并 `FAILED`，失败断言只指向只读锁/cache 消费、安装树、顺序、路径隔离、manifest、ROS launcher relocation 或缺 overlay 行为尚未实现；不得是 collection/fixture error、真实网络访问或缺外部构建机。

- [ ] **Step 3: 只消费冻结输入，先 pack 纯 Conda runtime 再依次装两个 wheel**

`pyproject.toml` 增加以下 PEP 517 backend 和 package discovery；`setuptools 80.9.0`、`wheel 0.45.1` 和 `build 1.2.2.post1` 必须已经由总计划 Task 2 的 toolchain unified/explicit lock 固定，E 不得修改或重新 render lock：

```toml
[build-system]
requires = ["setuptools==80.9.0", "wheel==0.45.1"]
build-backend = "setuptools.build_meta"

[tool.setuptools.packages.find]
include = ["slope_sim*"]
```

`packaging/build_release.sh` 的必填参数固定为绝对、已规范化且互不包含的 `--work-root`、`--output-dir`、`--micromamba`、`--python-package-cache`、`--python-wheel-cache` 与只读 `--source-archive-cache`；work/output/evidence 必须在仓库外且不得包含仓库，另要求显式互斥模式 `--stage-only | --final-archive` 和 `--release-version`。`scripts/verify_stage4_release.py` 暴露同一 `validate_release_version()` 与只打印验证后原值的 `--validate-release-version` CLI；builder、verifier、candidate/final context、安装器都调用该函数，严格按 Step 1 的 regex 与 128-byte 上限拒绝非 canonical SemVer，绝不各写一套 shell regex。`build_release.sh` 必须在解析完 argv 后、执行任一 `mkdir/mktemp/touch` 或拼接 work/output/archive/top-level 路径前先验证版本，失败时所有输出根保持不存在；后续消费者从 manifest/context 读到版本时仍重新验证且要求逐 byte 相同。`packaging/build-source-manifest.yml` 以规范相对路径/前缀明确列出 stage-only 可进入构建的源码、配置、资源、协议、ROS、packaging、测试和交付文档，并以 deny 优先排除 `.git`、ignored cache/output、`references/repos` 和任何外部工作根。stage-only 禁止遍历整个 worktree：先分别用 Git NUL 结构化输出取得 tracked 路径集合与 `--others --exclude-standard` 的 non-ignored untracked 集合，只读取 manifest allowlist 内路径的当前 working-tree bytes；allowlist 外的 non-ignored untracked 必须明确失败而非静默漏掉。helper 以仓库 dirfd、no-follow 和复制前后 device/inode/type/size/hash 复核，把这个有限成员集复制到仓库外 `$WORK/development-snapshot`，计算规范 path/mode/bytes digest 后递归只读；dirty tracked 与 allowlist 内 dirty untracked 都被忠实记录，`.git`、ignored/reference checkout 与 work/output/evidence 不可能自包含。final-archive 的 immutable source input 仍是 Task 6 从精确 HEAD 生成的只读 Git snapshot。

两种模式都由 `materialize_build_source.py` 以 no-follow/exclusive create 把只读 immutable snapshot 复制到本轮全新可写 `$WORK/source-build`，逐成员复算并要求 pre-build digest 完全相同。只有 `source-build` 可传给 setuptools/CMake/ROS 和资源安装；构建前后再次验证 immutable snapshot 的 digest、只读权限和成员未变，A/B 不得共享 source-build。`--micromamba` bytes、toolchain provenance、两组 unified/explicit lock、Python package cache manifest/tree 必须先由 `verify_python_lock_cache.py` 精确复核；官方 eCAL wheel cache 必须由 `verify_python_wheel_cache.py` 复核 URL/version/tag/RECORD/license/ELF inventory；C++/ROS 两个 source lock、`source-archive-cache.manifest.json` 和 source artifact tree 必须先由 `verify_stage4_source_cache.py` 精确复核，缺失、多余或漂移都在创建环境/source work 前失败。E 不生成这些输入，也不读取 `~/.conda`、`~/.cache/mamba`、pip cache/index、当前 Conda 环境、reference checkout 或任何未声明 channel。

本 Task 只实现 `--stage-only`：完成安装树、runtime manifest、self-test 和随机副本 smoke，在仓库外 output dir 原子写唯一 `stage-evidence.json`，但不创建 `.tar.zst`。stage evidence 固定写 `source_provenance.kind=development_worktree`、当前 commit、`clean` 布尔值、tracked/untracked 成员清单与 dirty 状态、规范 development-snapshot SHA-256、source-build pre-build SHA-256，以及只读 Python lock/package-cache/wheel-cache/toolchain、C++/ROS source archive manifest/tree、私有 C++ dependency install tree 与本轮 materialization digest；允许本地 dirty 开发 smoke，但此分支不得产生 `verified_archive` 或被 Task 6 复用为正式 source evidence。Task 6 的 RED 后才实现 `--final-archive`。工作根在进入时必须为空，由脚本创建独立 `development-snapshot/`（仅 stage）、`source-build/`、`mamba-root/`、`wheel-cache/`、`tool-env/`、`python-builder/`、`python-pack/`、`wheel/`、`cpp-sources/archives/`、`cpp-sources/trees/`、`cpp-deps-build/`、`cpp-deps-install/`、`validation-prefix/`、`cmake-build/`、`ros-sources/archives/`、`ros-sources/trees/`、`livox-sdk-install/`、`ros-build/`、`root/` 和 `smoke/`；`source-build` 只能由 immutable snapshot 复制并在构建前复核，source work 只能由 builder 从只读 canonical artifact 以 exclusive create 填充，不能把空目录当输入。Python、C++ dependency、项目 CMake、ROS 及最终安装根的所有可变产物都必须位于该工作根。输出目录也必须为空；脚本不得用 glob 猜测或覆盖产物。

`build_release.sh` 只能在 `run_network_isolated.sh` 建立并验证的无外网 namespace，或具有等价证明的断网 VM 中调用总计划 Task 2 已完成 GREEN 的 `build_python_runtime.sh`、`build_dependencies.sh` 和 `build_ros_overlay.sh`；三个 builder 都复核隔离证明，E 不得内联、复制或修改其中任一步。Python builder 只读验证 canonical URL 嵌套 package artifact 和 canonical eCAL wheel artifact 后，在本轮全新空 `$WORK/mamba-root/pkgs` 内按 manifest 逐项复算 hash并物化 micromamba 原生 flat cache，同时把精确 eCAL wheel exclusive-copy 到本轮 `$WORK/wheel-cache` 并复算：Conda archive 位于 cache 根，`urls.txt` 保存排序后的原 normalized URL；不得把 `pkgs/https/...` 嵌套树直接当原生 cache。basename 相同的 Conda 记录只有 size/MD5/SHA-256 全同才复制一次，否则 create 前失败。物化完成并保存 cache 外证据后，先用 `python-toolchain-linux-64.lock` 创建 `$WORK/tool-env`。生产 runtime 创建命令固定为：

```bash
"$MICROMAMBA" create \
  --no-rc --no-env \
  --root-prefix "$WORK/mamba-root" \
  --prefix "$WORK/python-builder" \
  --file "$SOURCE/packaging/locks/python-linux-64.lock" \
  --offline --always-copy --safety-checks enabled --yes
```

builder 使用专用空 HOME 并清除所有 Conda/Mamba/pip rc、channel、index、proxy 和 cache 环境变量；`--offline` 不能代替外部断网。两次构建分别从同一只读 canonical artifact 物化各自的 native cache 和 eCAL wheel 副本，不能共享同一可写 `mamba-root/pkgs`、wheel 副本、tool env 或 runtime env。micromamba create 后必须保留完整 native package cache 和全部 Conda 管理文件，直到 conda-pack 成功；尤其不能先删除环境中由 `conda-meta/files` 管理的 `.pyc/__pycache__`。项目 wheel 在固定 locale/timezone、`SOURCE_DATE_EPOCH` 和禁写随机 pyc 的环境中用 `"$WORK/tool-env/bin/python" -m build --wheel --no-isolation --outdir "$WORK/wheel" "$WORK/source-build"` 构建；setuptools 产生的 `egg_info` 或其他 build metadata 只允许位于这份可写副本/工作输出，immutable snapshot 的构建前后 digest 必须一致，此时不得把任一 wheel 安装进 `python-builder`。以下 pack/install/cleanup 均是只读 builder 的既定合同，E 的 `build_release.sh` 只传入本轮 source-build/work/canonical package+wheel cache/micromamba 并消费结果。

对仍为纯 Conda 内容的 runtime 执行：

```bash
"$WORK/tool-env/bin/conda-pack" \
  --prefix "$WORK/python-builder" \
  --output "$WORK/python-pack/python-runtime.tar" \
  --format tar --n-threads 1
```

只有 conda-pack rc=0 后，才把 tar 解到 `$WORK/root/runtime/python`，再执行：

```bash
"$WORK/tool-env/bin/python" -m pip install \
  --no-deps --no-index --no-compile \
  --prefix "$WORK/root/runtime/python" \
  "$ECAL_WHEEL"
"$WORK/tool-env/bin/python" -m pip install \
  --no-deps --no-index --no-compile \
  --prefix "$WORK/root/runtime/python" \
  "$PROJECT_WHEEL"
```

tool env 与 runtime 必须是同一锁定 Python 3.10 ABI/sysconfig layout；`ECAL_WHEEL` 只能是本轮私有副本，filename/tag/hash/RECORD/license/NOTICE/ELF inventory 与 canonical wheel manifest 精确对应，`PROJECT_WHEEL` 只能是本轮唯一 build output。两个 pip argv 必须保持上述顺序和参数，清空 index/find-links/proxy 环境，任何依赖解析或联网尝试都失败。项目正式命令只由根 `bin/` launcher 提供；pip 若为任一 wheel 写出 `direct_url.json` 则必须删除，项目 wheel 声明的 console scripts 按 entry-point metadata 精确识别并移除，然后按规范相对路径分别重算 eCAL 与项目 dist-info `RECORD`，排序固定、hash/size 与剩余文件一致、`RECORD` 自身 hash/size 留空；eCAL 的 license/NOTICE 与 ELF bytes/NEEDED/RUNPATH 不得改变。其他含 tool/builder/source/cache 前缀的脚本或文件一律拒绝。接着只在 staging 删除 `conda-meta/history`、全部 `.pyc/__pycache__`，规范 mode/mtime，并扫描 filename、UTF-8 text、binary bytes、wheel metadata 和 `conda-unpack` prefix records，证明不含任一绝对 tool/builder/source/cache/work root。除 `history` 外不得删除 conda-meta package records；launcher 固定 `PYTHONDONTWRITEBYTECODE=1`，且不得给 Python 进程注入 release `root/lib`。

`python-runtime.tar` 只是未发布中间物，conda-pack 不承诺它跨根 byte-identical，因此双根门比较清理、规范化后的 `root/runtime/python` tree digest 和最终 archive，而不直接比较这个 tar。原 `root/runtime/python` 仍保持未运行 `conda-unpack` 的 packed 状态；只允许在随机 smoke 副本和安装后的最终版本路径运行 relocation。目标机不要求安装 Conda，也不读取构建 cache。

- [ ] **Step 4: 配置、构建、测试并安装固定 C++ runtime 与资源**

下列所有项目源码参数中的 `STAGE4_SOURCE_BUILD_ROOT` 必须精确等于本轮 `$STAGE4_RELEASE_WORK_ROOT/source-build`；immutable development/Git snapshot 只用于摘要复核，绝不直接传给任一构建命令。每个 stage/final 调用都从同一只读 canonical source artifact 在本轮根内重建 C++ dependency/validation prefix，不得消费总计划 Task 2 的开发 `STAGE4_DEPENDENCY_PREFIX`，更不得消费另一轮 prefix。

```bash
test -x "$STAGE4_CMAKE" && test -x "$STAGE4_CTEST" && \
  test -x "$STAGE4_CC" && test -x "$STAGE4_CXX"
bash "$STAGE4_SOURCE_BUILD_ROOT/packaging/build_dependencies.sh" \
  --lock "$STAGE4_SOURCE_BUILD_ROOT/packaging/locks/cpp-dependencies.lock" \
  --source-cache-manifest \
    "$STAGE4_SOURCE_BUILD_ROOT/packaging/locks/source-archive-cache.manifest.json" \
  --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE" \
  --source-work "$STAGE4_RELEASE_WORK_ROOT/cpp-sources" \
  --build-root "$STAGE4_RELEASE_WORK_ROOT/cpp-deps-build" \
  --prefix "$STAGE4_RELEASE_WORK_ROOT/cpp-deps-install" \
  --validation-prefix "$STAGE4_RELEASE_WORK_ROOT/validation-prefix"
STAGE4_DEPENDENCY_PREFIX="$STAGE4_RELEASE_WORK_ROOT/cpp-deps-install"
STAGE4_PROTOC="$STAGE4_DEPENDENCY_PREFIX/bin/protoc"
STAGE4_CMAKE_PREFIX_PATH="$STAGE4_DEPENDENCY_PREFIX"
export STAGE4_DEPENDENCY_PREFIX STAGE4_PROTOC STAGE4_CMAKE_PREFIX_PATH
test -x "$STAGE4_PROTOC"
"$STAGE4_CMAKE" --preset stage4-release \
  -S "$STAGE4_SOURCE_BUILD_ROOT" -B "$STAGE4_RELEASE_WORK_ROOT/cmake-build"
"$STAGE4_CMAKE" --build "$STAGE4_RELEASE_WORK_ROOT/cmake-build" --parallel 2
"$STAGE4_CTEST" --test-dir "$STAGE4_RELEASE_WORK_ROOT/cmake-build" \
  --output-on-failure --no-tests=error
"$STAGE4_CMAKE" --install "$STAGE4_RELEASE_WORK_ROOT/cmake-build" \
  --prefix "$STAGE4_RELEASE_WORK_ROOT/root"
bash "$STAGE4_SOURCE_BUILD_ROOT/packaging/stage_cpp_runtime.sh" \
  --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" \
  --project-prefix "$STAGE4_RELEASE_WORK_ROOT/root" --mode sdk
install -d "$STAGE4_RELEASE_WORK_ROOT/root/share/slope-sim"
cp -a "$STAGE4_SOURCE_BUILD_ROOT/urdf" "$STAGE4_SOURCE_BUILD_ROOT/configs" \
  "$STAGE4_RELEASE_WORK_ROOT/root/share/slope-sim/"
cp -a "$STAGE4_SOURCE_BUILD_ROOT/resources/." \
  "$STAGE4_RELEASE_WORK_ROOT/root/share/slope-sim/"
install -d "$STAGE4_RELEASE_WORK_ROOT/root/share/slope-sim/interfaces"
install -m 0644 "$STAGE4_SOURCE_BUILD_ROOT/proto/slope_sim_interfaces_v2.proto" \
  "$STAGE4_SOURCE_BUILD_ROOT/proto/slope_sim_interfaces_v2.sha256" \
  "$STAGE4_SOURCE_BUILD_ROOT/proto/slope_sim_control_v1.proto" \
  "$STAGE4_SOURCE_BUILD_ROOT/proto/slope_sim_record_v1.proto" \
  "$STAGE4_SOURCE_BUILD_ROOT/slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc" \
  "$STAGE4_SOURCE_BUILD_ROOT/slope_sim/interfaces/generated/slope_sim_control_v1.desc" \
  "$STAGE4_SOURCE_BUILD_ROOT/slope_sim/interfaces/generated/slope_sim_record_v1.desc" \
  "$STAGE4_RELEASE_WORK_ROOT/root/share/slope-sim/interfaces/"
install -m 0755 "$STAGE4_SOURCE_BUILD_ROOT/packaging/bin/slope-sim" \
  "$STAGE4_RELEASE_WORK_ROOT/root/bin/slope-sim"
install -m 0755 "$STAGE4_SOURCE_BUILD_ROOT/packaging/bin/slope-sim-ros-bridge" \
  "$STAGE4_RELEASE_WORK_ROOT/root/bin/slope-sim-ros-bridge"
```

`stage4-release` preset 与 C 的 `stage4-dev` 使用同一组必填绝对工具和 dependency lock，不允许 PATH fallback；但本轮 `protoc/CMAKE_PREFIX_PATH` 只来自刚生成的私有 `cpp-deps-install`。dependency 与 release preset 共用锁定 `SOURCE_DATE_EPOCH`，并把 source/work root 通过 `-ffile-prefix-map`、`-fdebug-prefix-map`、`-fmacro-prefix-map` 映射到固定 `/usr/src/slope-sim-deps`、`/usr/src/slope-sim` 与 `/usr/src/slope-sim-build`，要求 archive member、linker build-id 和 generated file 顺序确定；测试故意换两个 work root 后用安装树 manifest、`strings/readelf` 证明两个私有 dependency prefix 和项目产物逐成员一致且没有原绝对路径。release root 固定且只能包含 `bin/lib/include/share/runtime/python/ros-overlay` 等清单目录；禁止再产生 `usr/bin` 或依赖 `/opt/slope-sim/current/usr`。构建阶段禁用 CMake `FetchContent` 网络下载，只消费本轮从 canonical source cache 重建且 lock hash 一致的 dependency prefix。`stage_cpp_runtime.sh` 必须把 eCAL、Protobuf、MCAP、Zstd 的运行库、SONAME 别名及 SDK 必需 header/CMake config 一并放入该 root；对每个 ELF 执行 `readelf -d` 和 `ldd`，RUNPATH 必须为 `$ORIGIN/../lib` 或相应相对路径，非系统依赖只能解析到同一 root 的 `lib/`，不得出现仓库、Conda、builder 或私有 dependency prefix 的构建路径。C++ launcher 不注入 Python site-packages；后续 no-participant 双 loader smoke 必须证明 C++ 只加载 root `lib` 的 eCAL，同时没有创建任何 eCAL entity。

可选 ROS payload 使用真实 build 子命令和同一 GCC 13 工具链：

```bash
source /opt/ros/jazzy/setup.bash
CC="$STAGE4_CC" CXX="$STAGE4_CXX" \
  bash "$STAGE4_SOURCE_BUILD_ROOT/packaging/build_ros_overlay.sh" \
  --lock "$STAGE4_SOURCE_BUILD_ROOT/packaging/locks/ros2-dependencies.lock" \
  --source-cache-manifest \
    "$STAGE4_SOURCE_BUILD_ROOT/packaging/locks/source-archive-cache.manifest.json" \
  --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE" \
  --source-work "$STAGE4_RELEASE_WORK_ROOT/ros-sources" \
  --livox-sdk-prefix "$STAGE4_RELEASE_WORK_ROOT/livox-sdk-install" \
  --build-base "$STAGE4_RELEASE_WORK_ROOT/ros-build" \
  --project-source "$STAGE4_SOURCE_BUILD_ROOT/ros2" \
  --client-prefix "$STAGE4_RELEASE_WORK_ROOT/root" \
  --install-base "$STAGE4_RELEASE_WORK_ROOT/root/ros-overlay"
```

该唯一入口先确认仍处于外层已验证的断网 namespace，再只读复核 canonical source manifest/artifact/member/materialized tree digest，按 `ros_overlay` consumer 把两个精确归档 exclusive-copy 到本轮私有 `ros-sources/archives` 并再次 hash，再用总计划唯一安全 parser 预检成员并物化到私有零链接 `trees`。Livox-SDK2 显式以 `CMAKE_INSTALL_PREFIX=$STAGE4_RELEASE_WORK_ROOT/livox-sdk-install` 安装，禁止 sudo 或写 `/usr/local`；driver configure 预置 `LIVOX_LIDAR_SDK_LIBRARY/LIVOX_LIDAR_SDK_INCLUDE_DIR` 精确指向该私有 prefix，并通过 CMakeCache/link command/readelf/ldd 复核。入口在构建前后对真实 `/usr/local/lib` 与 `/usr/local/include` 做只读排序 census/hash 并要求完全不变；测试 fixture 用临时 fake sysroot/default-search path 放错误版本 poison，不要求普通 pytest 写真实 `/usr/local`。随后构建完整官方 `livox_ros_driver2` 和本项目 Bridge 到同一 merge-install overlay，并验证 `CustomMsg/CustomPoint` interface hash、typesupport、GCC 13 和 relocatable linkage。任何缺失、多余、漂移、恶意 member、共享可写树、默认 `/usr/local` 命中或联网 fallback 都在 colcon 前失败；禁止 E 另写 shell extractor 或一套只编 message 的命令。

Python、C++ 和 ROS 内容全部进入 root 后、生成 runtime/release manifest 前，运行 `materialize_release_tree.py --root "$STAGE4_RELEASE_WORK_ROOT/root"`。helper 先只读冻结完整 `lstat` 成员图、inode/size/hash、相对 link target 和总展开预算，再在同一 work root 的全新 sibling `root.materialized.next` 构造第二棵树，绝不边遍历边改原树。目录和普通文件按规范相对路径复制；hardlink 每个名字都复制成独立普通文件；相对 symlink 允许包含 `..`，但逐跳解析必须始终留在冻结 root 内并最终指向普通文件或目录。file link 复制目标 bytes，directory link 按冻结图排序递归深拷贝目标子树；共享目标可以复制，祖先回边/任意循环、绝对/逃逸/悬空 link、特殊节点、展开后的 member/byte 上限超出、同路径冲突或复制期间 inode/size/hash 变化都失败。

非空 `root` 不能靠普通 `rename/os.replace` 覆盖。helper 在复制前用两个同文件系统临时 sibling 探测 Linux `renameat2(RENAME_EXCHANGE)` 并交换回原状；syscall/文件系统不支持时在修改 root 前失败。临时树全部文件和目录写完、逐成员 hash/ELF/Python/ROS smoke 通过后，先 fsync 文件、目录和父目录，写入并 fsync 外部事务记录 `PREPARED`，再以 `RENAME_EXCHANGE` 原子交换 `root` 与 `root.materialized.next`，fsync 父目录并记录 `EXCHANGED`；旧 root 此时保留在 sibling 路径，成功复核新 root 后改名为 `root.materialized.previous` 作为诊断树并记录 `VERIFIED`，不进入发行 manifest。任一 pre-exchange 失败保持原 root；exchange/父目录 fsync/post-exchange 验证失败必须按事务记录交换回滚、再次 fsync，并把失败的新树保留为明确命名的诊断目录。进程在 exchange 后崩溃时，下次调用先依据已 fsync 的记录和两棵树 digest 判定完成验证或交换回滚，禁止猜测、覆盖或自动删除诊断树。

fixture 正例必须同时覆盖 Conda `bin/python`、C++ SONAME、ROS resource file 的 file link，以及锁定 runtime 实际存在的 `lib/python3.1 -> python3.10`、`lib/terminfo -> ../share/terminfo` 这类 directory link；物化后行为/hash 等价，递归 `lstat` 只见目录/普通文件且每个普通文件 `st_nlink == 1`。反例覆盖逃逸/悬空/循环 directory link、展开膨胀、exchange 不支持、exchange syscall 失败、pre/post-exchange fsync 失败、exchange 后崩溃、post-exchange 验证失败、成功/失败回滚以及恢复时 digest 歧义；每个失败点都断言原 root 可用且对应诊断树/事务状态被保留。真实 stage-only GREEN 还要在物化前输出锁定 runtime link census（路径、相对 target、最终类型、target digest），证明至少完整处理实际 census；物化后要求 link count 为 0，再在随机副本运行 `conda-unpack`、关键 import 和 CLI/ROS smoke，不能只用人工最小 fixture 宣称兼容真实 Conda tree。

`slope-sim-ros-bridge` 只从自身 `readlink -f` 后的位置推导 release root，依次加载 `/opt/ros/jazzy/setup.bash` 和该 root 的 `ros-overlay/setup.bash`，再 `exec "$ROOT/ros-overlay/lib/slope_sim_bridge/slope_sim_bridge_node"`；编排器只能调用这个精确 launcher，不从 PATH 猜 overlay binary。测试把整个 release tree 移到另一绝对路径后运行 launcher `--version`，并断言环境和 executable 均来自新 root；缺系统 Jazzy 或 overlay 时 `doctor` 报 `optional_unavailable`，核心 Simulator/Subscriber/Recorder 仍能启动。

- [ ] **Step 5: 实现 runtime manifest、self-test 与随机副本 build smoke**

先用 `verify_stage4_dependencies.py --install-prefix "$STAGE4_RELEASE_WORK_ROOT/root" --build-kind release --write-runtime-manifest "$STAGE4_RELEASE_WORK_ROOT/root/share/slope-sim/runtime-manifest.json"` 生成安装树 runtime manifest，并验证 `$STAGE4_RELEASE_WORK_ROOT/root/share/slope-sim/models/robot_models.yaml` 与 B 记录的 canonical SHA 相同。把生产版 `scripts/verify_stage4_release.py` 安装为只读 `$STAGE4_RELEASE_WORK_ROOT/root/share/slope-sim/tools/verify_stage4_release.py`，由 bundled Python 调用；源码路径和安装后路径必须 byte-identical，目标机 smoke 不依赖 checkout 或 Conda。再运行 `$STAGE4_RELEASE_WORK_ROOT/cmake-build/bin/stage4-selftest-session`，传入该 runtime manifest、已安装模型 YAML 和受版本控制 recipe，把与当前 runtime digest 绑定的一段完整五 topic MCAP session 直接写入新建的 `$STAGE4_RELEASE_WORK_ROOT/root/share/slope-sim/selftest/`；生成器 evidence、最终 manifest 和唯一 segment 都进入发行 manifest，任何 `.partial` 都失败。

原始 `root/runtime/python` 保持 packed；构建 smoke 必须把整个 root 复制到工作根内随机且此前不存在的 `$STAGE4_SMOKE_RELEASE_ROOT`，只对该副本调用 `packaging/relocate_python_runtime.sh --release-root "$STAGE4_SMOKE_RELEASE_ROOT"`。helper 拒绝符号链接/非绝对/未完成 root，只在目标 root 内执行 bundled `runtime/python/bin/conda-unpack`，验证 `sys.prefix` 和旧 builder 前缀清零后，以 canonical JSON、普通文件且 link count=1 原子写入精确 `$RELEASE_ROOT/relocation-state.json`；marker 绑定该绝对 release root、packed runtime digest、unpack tool hash 和完成状态。同一路径重复调用幂等，不同路径复用 marker 或半完成状态失败。Task 4 安装器必须复用此 helper，不得另写第二套重定位顺序。

完成重定位后固定执行无环序列：`relocate_runtime -> run_cli_sdk_selftest_and_loader_checks -> run_precommit_health_probe_without_install_state -> atomic_write_final_install_state -> run_doctor_from_final_install_state`。先清空源码目录的 `sys.path`、`PATH` 和 `LD_LIBRARY_PATH`，完成 Python import、五个 C++ CLI `--version`、SDK consumer、self-test MCAP 回读/PCD/PLY/LVX2 导出和下面的双 loader 检查；这些步骤不得创建 `install-state.json`。随后调用与 doctor 共用检查实现、但明确拒绝读取或要求 `install-state.json` 的纯 `probe_install_health()`，只根据当前 root、runtime manifest 和 `relocation-state.json` 返回 canonical `health` 对象，不执行普通 doctor CLI，也不写 root。harness 再且只再一次以 exclusive temp、fsync 和 rename 原子提交最终 `install-state.json`：共享字段保存 packed manifest SHA、该随机绝对路径、relocation marker SHA、解包工具 hash 和上述 `health`，provenance 判别支固定为 `kind=build_smoke`；state 中禁止嵌入尚未发生的 doctor 输出、archive basename/hash 或 build-evidence hash。state 成功后才运行普通 `bin/slope-sim doctor --json ...`；doctor 必须读取这个最终 state，报告 `build_smoke`，重算并要求 `doctor.health == install-state.health`。relocation、CLI/SDK/self-test/loader 或 precommit probe 任一步失败时 state 不存在；state 原子提交失败或最终 doctor 失败时 smoke 整体失败且不得发布 stage evidence。

双 loader 检查启动两个专用 no-participant 子进程并通过继承 pipe 保持存活：Python 侧只 import/`dlopen` wheel 内 eCAL core，C++ 侧只 `dlopen(..., RTLD_NOW)` release `lib` 内 eCAL core；两者都禁止调用 Python/C++ eCAL Initialize、monitoring、publisher、subscriber 或其他 entity API。读取 `/proc/<pid>/maps` 并结合 `readelf/ldd` 断言 Python 只加载 `runtime/python/.../ecal/libecal_core.so.6`、C++ 只加载当前 release `lib/libecal_core.so.6`，每个进程恰有一套 eCAL 且不出现第二套 `libprotobuf.so`；测试用调用审计桩、真实 smoke 用前后 entity census 共同要求新增 participant/topic 为 0。任何 loader 创建 entity 都是构建失败，不能临时申请真实运行授权或登记成生产 eCAL 证据。若 import 来自 checkout、存在开发 `.pth`、两类 eCAL 交叉加载、非系统 DSO 解析到 root 外、生成 binding/模型 YAML/self-test 缺失或 packed 原树被改写，立即失败。

- [ ] **Step 6: 运行安装树 GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_release_manifest.py tests/stage4/test_release_build_paths.py tests/stage4/test_ros_launcher_relocation.py`

Expected: PASS；本任务只证明可迁移 packed 安装树及一次性解包 smoke，不生成或发布最终 `.tar.zst`。最终归档必须等待 Task 4 安装器、Task 5 教程和 Task 6 纯回归全部完成。

- [ ] **Step 7: 运行 stage-only 集成 GREEN**

只有 Step 6 的三份聚焦合同全部 GREEN 后才执行本 Step；不得用高成本真实构建替代尚未通过的 fixture。

Run:

```bash
test -n "${STAGE4_RELEASE_VERSION:-}"
STAGE4_RELEASE_VERSION="$(
  conda run -n slope-sim python scripts/verify_stage4_release.py \
    --validate-release-version "$STAGE4_RELEASE_VERSION"
)"
readonly STAGE4_RELEASE_VERSION
test -n "${STAGE4_BUILD_ENV_FILE:-}"
test -n "${STAGE4_EXTERNAL_STAGE_PARENT:-}"
test -d "$STAGE4_EXTERNAL_STAGE_PARENT"
STAGE4_STAGE_RUN_PARENT="$(mktemp -d \
  "$STAGE4_EXTERNAL_STAGE_PARENT/slope-sim-stage4-stage.XXXXXX")"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-env "$STAGE4_BUILD_ENV_FILE" \
  --json "$STAGE4_STAGE_RUN_PARENT/toolchain-preflight.json"
source "$STAGE4_BUILD_ENV_FILE"
test -x "$STAGE4_MICROMAMBA"
test -d "$STAGE4_PYTHON_PACKAGE_CACHE"
test -d "$STAGE4_PYTHON_WHEEL_CACHE"
test -d "$STAGE4_SOURCE_ARCHIVE_CACHE"
unset STAGE4_DEPENDENCY_PREFIX STAGE4_CMAKE_PREFIX_PATH
unset STAGE4_PROTOC STAGE4_PCL_PCD2PLY
STAGE4_STAGE_WORK_ROOT="$STAGE4_STAGE_RUN_PARENT/work"
STAGE4_STAGE_OUTPUT_DIR="$STAGE4_STAGE_RUN_PARENT/output"
STAGE4_STAGE_VERIFICATION="$STAGE4_STAGE_RUN_PARENT/release-stage-verification.json"
install -d "$STAGE4_STAGE_WORK_ROOT" "$STAGE4_STAGE_OUTPUT_DIR"
bash packaging/run_network_isolated.sh \
  bash packaging/build_release.sh --stage-only \
    --work-root "$STAGE4_STAGE_WORK_ROOT" \
    --output-dir "$STAGE4_STAGE_OUTPUT_DIR" \
    --release-version "$STAGE4_RELEASE_VERSION" \
    --micromamba "$STAGE4_MICROMAMBA" \
    --python-package-cache "$STAGE4_PYTHON_PACKAGE_CACHE" \
    --python-wheel-cache "$STAGE4_PYTHON_WHEEL_CACHE" \
    --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --stage-evidence "$STAGE4_STAGE_OUTPUT_DIR/stage-evidence.json" \
  --output "$STAGE4_STAGE_VERIFICATION"
```

Expected: rc=0；evidence 只引用仓库外本轮 work root 中的独立 development-snapshot/source-build、Python package/wheel cache/env、C++ dependency source/build/install、ROS archive 副本/解包树、packed root 和随机 smoke 副本，绑定 immutable snapshot/source-build、总计划 Task 2 的 Python lock/package-cache/wheel-cache/toolchain 与 C++/ROS source archive manifest/tree digest；dirty tracked/untracked 当前 bytes 被纳入且 `.git`/ignored/reference/output 未纳入，构建前后 immutable snapshot digest 不变，真实锁定 runtime 的 pre-materialization link census 非空且全部安全物化，最终 release tree link count 为 0，随机副本 `conda-unpack`/import/CLI/ROS 及 no-participant Python/C++ eCAL DSO 隔离 smoke 通过，entity census 增量为 0。原 packed root 未运行 `conda-unpack`，仓库内零输出，不产生任何 `.tar.zst`。

- [ ] **Step 8: REFACTOR builder 阶段边界并原样复验**

只整理已 GREEN 的输入验证、staging 物化和 manifest 扫描重复；不得移动纯 Conda pack、wheel 安装、pyc/history 清理或随机副本 relocation 的顺序，也不得在 E 内生成 lock/cache。无需整理时记录“REFACTOR：无必要”，随后严格按 Step 6 聚焦 GREEN、Step 7 stage-only 集成 GREEN 的顺序原样复验。

## Task 4：安装、升级、回退与卸载

**Files:**
- Create: `packaging/install.sh`
- Create: `packaging/uninstall.sh`
- Modify: `packaging/relocate_python_runtime.sh`
- Test: `tests/stage4/test_installer.py`
- Test: `tests/stage4/test_upgrade_rollback.py`

- [ ] **Step 1: 写临时 prefix 安装 RED**

```python
def test_failed_upgrade_keeps_previous_current_symlink(release_fixture, tmp_path) -> None:
    prefix = tmp_path / "opt" / "slope-sim"
    install(release_fixture("1.0.0"), prefix)
    with pytest.raises(InstallError):
        install(release_fixture("1.1.0", corrupt=True), prefix)
    assert os.readlink(prefix / "current") == "releases/1.0.0"


def test_conda_unpack_runs_only_after_final_version_path_exists(
    release_fixture, tmp_path, install_events
) -> None:
    prefix = tmp_path / "opt" / "slope-sim"
    install(release_fixture("1.0.0"), prefix)
    assert install_events.index("rename_to_releases/1.0.0") \
        < install_events.index("conda_unpack_at_releases/1.0.0") \
        < install_events.index("run_cli_sdk_selftest") \
        < install_events.index("run_precommit_health_probe_without_install_state") \
        < install_events.index("atomic_write_final_install_state") \
        < install_events.index("doctor_at_releases/1.0.0") \
        < install_events.index("switch_current")
```

覆盖已解源树 checksum 失败、目标版本已存在、无权限、磁盘不足、并发安装、`conda-unpack` 失败、precommit health probe 失败、state 原子提交失败、最终路径 doctor 失败、保留用户配置/数据、卸载 current/非 current、systemd 默认禁用。对 relocation、CLI/SDK/self-test、precommit probe、state 提交和 final doctor 逐点注入失败，断言普通 doctor 只能在最终 state 后运行、state 只写一次且不含未来 doctor 输出、任一失败时旧 `current` 不变。另为 `install.sh --activate-existing <version>` 写 RED：只允许同一 prefix 内已有且 `verified_archive` state/marker/fresh doctor 全部有效的非 current 版本，成功才原子切换；缺版本、当前版本、build-smoke/失败隔离目录、state/marker/doctor 漂移或并发切换全部拒绝并保持旧 current。来源 fixture 还要覆盖 archive bytes 与传入 SHA 不同、sidecar basename/hash 不同、build evidence hash 不同、evidence 内 archive 名/hash 不同、缺任一来源参数，以及把 `build_smoke` provenance 冒充正式安装；全部必须在写入 prefix 前失败。`test_installer.py` 的恶意已解源树 fixture 必须覆盖规范根外 resolve、符号链接、硬链接或普通文件 link count 不为 1、device/FIFO/socket、清单外成员、文件/目录类型冲突，以及复制期间 device/inode/size/hash 变化；所有反例都必须在目标 prefix 外零写入。外层 archive 的绝对路径、`../`、重复成员、类型冲突、符号/硬链接、device/FIFO/socket、清单外成员和声明大小/实际膨胀超限由 Task 6 的 `test_release_archive.py` 在发布前独立验证，不能让位于 archive 内的安装器假装检查尚未解出的 archive。失败路径必须证明旧 `current` 未改变、失败版本被移入隔离目录且不会被 launcher 发现。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_installer.py tests/stage4/test_upgrade_rollback.py`

Expected: pytest 正常收集并 `FAILED`，失败断言指向原子安装/升级/回退行为尚未实现；测试只操作临时 prefix，不需要 root 或真实 `/opt`。

- [ ] **Step 3: 实现版本目录与原子 current**

```bash
release_dir="$prefix/releases/$version"
next_link="$prefix/.current.$version"
ln -s "releases/$version" "$next_link"
mv -T "$next_link" "$prefix/current"
```

发行流程分三层，顺序不得倒置：Task 6 的构建端 verifier 在发布前用结构化 archive reader 检查最终 archive；目标机先用外部 sidecar 校验精确 archive SHA，再按 Task 8 教程以安全参数解到此前不存在的 0700 临时目录；archive 内的 `install.sh` 以已解源树作为唯一 payload 输入，不枚举或解包 archive，也不声称验证了解包前成员，但额外接收 archive、sidecar 和 build evidence 的绝对只读路径以重算并保存来源摘要。安装器首先要求三份外部证据的 basename/hash/内部字段完全一致，再以目录 fd 为根逐项 `lstat` 已解源树：名称必须是唯一规范 basename/相对路径，禁止绝对路径、`..`、控制字符和 resolve 逃逸；只允许普通文件/目录，拒绝符号链接、硬链接、普通文件 link count 不为 1、device/FIFO/socket、重复或清单外成员。成员集合、类型、逐文件大小、总大小和 hash 必须与 `release-manifest.json` 及内部 `SHA256SUMS` 精确一致。

源树验证通过后，安装器才以不跟随链接、`openat`/exclusive create 的方式把 packed payload 复制到目标 prefix 同一文件系统、权限 0700 的私有 `.incoming-<version>-<nonce>`；复制每个文件前后重新核对 source device/inode/type/size/hash，目标文件固定清理 setuid/setgid/world-writable 位，任一 TOCTOU 变化立即失败。incoming 中再次只读验证 `SHA256SUMS`、SBOM/manifest 和 packed payload，不能运行 `conda-unpack`。随后把完整版本目录原子 rename 为最终 `releases/<version>`，对这个最终绝对路径调用 Task 3 的 `relocate_python_runtime.sh`，再以 `SLOPE_SIM_ROOT=<final-version-path>` 运行 C++ CLI、SDK 和 self-test。全部通过后调用与 doctor 共用检查实现但不读取 state 的纯 precommit health probe，取得 canonical `health`；安装器只在此时原子写一次最终 `install-state.json`，再运行必须读取该 state 的普通 doctor 并要求重算 health 完全一致。只有 final doctor 成功才原子切换 `current`。

安装器先用 Task 3 的同一 `validate_release_version()` 重新验证 manifest、归档顶层目录和请求版本逐 byte 相同，再允许拼接 `releases/<version>`；非法、非规范或超过 128 ASCII bytes 的版本在创建 prefix/incoming 前失败。`release-manifest.json` 和内部 `SHA256SUMS` 描述归档中尚未重定位的 packed payload；`conda-unpack` 会合法改写 Python 前缀，因此安装器在最终路径另写 `install-state.json`。共享字段记录 packed manifest SHA、最终路径、解包工具 hash、精确 sibling `relocation-state.json` 的 SHA 和 precommit probe 得到的 canonical `health`，明确禁止保存尚未运行的 doctor 输出；正式分支固定 `provenance.kind=verified_archive`，并记录已重算的 archive basename/SHA-256、build-evidence SHA-256，以及 evidence 中与 release manifest 再交叉验证的 clean source 标记、Git commit/tree、source snapshot、lock 和 builder digest。字段缺失、多余、错配、`clean` 非 true 或 `kind=build_smoke` 都不能成为正式 release root。state 提交后的 doctor 对 C++/资源等不可变文件继续逐 hash 校验，对 Python runtime 则检查 `sys.prefix`、项目版本、无旧前缀占位和关键 import，不拿解包后的 Python 文件与 packed hash 做错误的逐文件比较；其 JSON 以 state SHA 绑定本轮安装并要求 `doctor.health == install-state.health`。

若 `conda-unpack` 或 doctor 失败，旧 `current` 保持不变；新目录原子改名为 `releases/.failed-<version>-<nonce>` 并写失败状态，既不伪装成可用版本，也不自动删除诊断证据。目标版本已存在或存在同版本失败目录时默认拒绝，只有显式清理命令才能处理。升级不覆盖 `~/.config/slope-sim`、`~/slope-sim-data`；卸载默认只删指定 release，删除用户数据必须单独显式参数并二次确认。

`install.sh --activate-existing <version> --prefix <absolute-prefix>` 是唯一回退入口，与新安装模式互斥且不接收 source/archive 参数。它复用同一版本 parser 和 prefix 锁，fd-relative 读取目标版本的 verified archive state/marker，运行 fresh doctor 并重算安装树 identity；全部成功后才用同一 `next_link + mv -T` 事务切换 `current`。`uninstall.sh --version <version>` 默认拒绝删除 current，只允许在回退后删除非 current lifecycle-probe；两者都不改用户配置或数据。

- [ ] **Step 4: systemd/desktop 安装保持 opt-in**

安装 desktop entry；systemd user unit 只复制不 enable/start。只有用户显式运行 `slope-sim service enable` 才调用 `systemctl --user enable`。

- [ ] **Step 5: 运行 GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_installer.py tests/stage4/test_upgrade_rollback.py`

Expected: PASS。

- [ ] **Step 6: REFACTOR 安装事务并原样复验**

只整理已覆盖的 fd-relative 校验、incoming rename、失败隔离和 current 切换重复；不得放宽链接/TOCTOU/source provenance 门。无需整理时记录“REFACTOR：无必要”，随后原样重跑 Step 5。

## Task 5：中文部署、SDK 与人工测试教程

**Files:**
- Create: `docs/阶段四部署教程.md`
- Modify: `docs/阶段四C++SDK教程.md`
- Create: `docs/阶段四记录回放导出教程.md`
- Create: `docs/阶段四手动测试教程.md`
- Modify: `README.md`
- Modify: `3d仿真平台需求规格.md`
- Modify: `三阶段反馈解决.md`
- Modify: `docs/阶段三交付报告.md`
- Modify: `docs/superpowers/specs/2026-07-21-stage3-ecal-enterprise-interfaces-design.md`
- Modify: `docs/superpowers/plans/2026-07-21-stage3-ecal-enterprise-interfaces-implementation.md`
- Test: `tests/stage4/test_stage4_docs.py`

- [ ] **Step 1: 写文档合同 RED**

测试要求教程包含：离线安装/校验、interactive/headless、五 topic、三点 RTK、C++ 最小订阅、Command 安全规则、Recorder/Replay/PCD/PLY/LVX2、ROS/RViz2、升级回退、卸载、日志路径和常见错误；还要逐 byte 包含 Task 8 Step 2 通过 bundled Python 与 installed production verifier 生成结构化 no-participant smoke 的命令和输出路径。命令必须能由 CLI parser 或对应 verifier parser 接受，且在无 checkout、无 Conda 的目标机上只依赖安装树。

- [ ] **Step 2: 运行 RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_docs.py`

Expected: pytest 正常收集并 `FAILED`，失败断言列出缺失的教程合同或无法由 parser 接受的命令；不得把尚未实现的功能写成已完成。

- [ ] **Step 3: 写面向非开发者的主教程**

人工教程按“准备→启动→驾驶→15 页 Dashboard→RTK→IMU→三维点云→记录→回放→导出→停止”组织，每一步给出预期画面/数值和失败处理；说明 eCAL 是进程间数据管道、RViz2 是实时三维查看器、Livox Viewer 2 只打开合成 LVX2。

- [ ] **Step 4: 同步权威边界**

需求规格把旧阶段四自动导航替换为已确认交付范围；阶段三设计、实施计划、反馈解决文档和交付报告只追加“由阶段四 v2 替代”的历史注记，不改旧结果。README 只写已经实现且有证据的状态。

- [ ] **Step 5: 运行 GREEN**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_docs.py`

Expected: PASS。

- [ ] **Step 6: REFACTOR 教程命令与术语并原样复验**

只统一已由文档合同覆盖的术语、命令片段和交叉链接；不得把未运行门禁改写为已完成。无需整理时记录“REFACTOR：无必要”，随后原样重跑 Step 5。

## Task 6：纯自动回归、DIRECT/GUI 与最终可复现归档

**Files:**
- Modify: `.gitignore`
- Modify: `packaging/build_release.sh`
- Modify: `scripts/verify_stage4_release.py`
- Modify: `scripts/verify_stage4_ecal_cpp.py`
- Test: `tests/stage4/test_release_archive.py`
- Test: `tests/stage4/test_stage4_ecal_cpp_verifier.py`
- Modify: `docs/阶段四交付报告.md`

- [ ] **Step 1: 写最终 archive 与双构建复现 RED**

`test_release_archive.py` 使用本地最小 Git 仓库 fixture 调用 wished-for final-archive 模式，断言精确 archive 名、外部 SHA 文件、构建 evidence、`source_provenance.kind=clean_git_commit`、`clean=true`、commit/tree/source snapshot/source-build SHA、Python lock/package-cache/wheel-cache/toolchain digest、C++/ROS source archive manifest/tree/materialization digest、每轮私有 C++ dependency install tree digest、规范 runtime tree digest、固定 tar owner/mtime/order、固定 Zstd frame、安装器/教程/SBOM/licenses/self-test 完整性以及 A/B archive byte-identical。sidecar 只写精确 archive basename；测试从既不含 archive 也不含 sidecar 的无关 cwd 运行结构化 verifier 和发布命令同款 `sha256sum -c`，仍必须校验同目录 archive 成功，篡改 basename/hash、把同名 archive 放进 cwd 或从仓库 cwd 偶然命中都必须失败。两轮 work/output/final parent 必须都是仓库外 sibling；集成测试按真实顺序先完成 A，证明仓库 `git status --porcelain=v1 --untracked-files=all` 仍为空，再启动 B 并通过其独立 clean gate。final 模式传入仓库内、仓库子目录或包含仓库的 work/output/evidence parent 必须在创建内容前失败。每轮把 read-only `source-snapshot` 复制为不同可写 `source-build`，pre-build digest 必须等于 snapshot，setuptools `egg_info` 只写 source-build，构建后 snapshot 成员/权限/digest 不变；测试拒绝直接构建 snapshot、A/B 共用 source-build 或副本摘要漂移。另断言 `.gitignore` 只精确忽略 `/build/stage4*/` 与 `/results/stage4/` 等计划内生成物，不得吞掉源码/lock/manifest；在实现已成为 clean HEAD 后，这些 ignored 输出不污染 formal clean gate。

同一测试文件还要用两个时间戳不同的本地 clean commit 覆盖“已验收候选 -> 最终证据 commit”比较：verifier 代码内的 source diff allowlist 固定且只能是 `README.md` 与仓库外发布证据索引 `docs/阶段四交付报告.md`，命令行、context 或产物都不能扩展它，这两个路径也不得进入发行 payload。accepted-candidate context 冻结由候选 commit timestamp 得到的 `functional_source_epoch`；最终 builder 只有在独立验证 context、两个 commit 及固定 source diff 后才可继承这个 epoch，同时 runtime/build evidence 必须如实记录最终 commit/tree/source snapshot。测试故意让最终 commit timestamp 不同，仍要求所有非 provenance 功能输入与输出逐 byte 相同；Git/source 标识不得编译进 ELF、wheel、SDK、资源或教程。

候选与最终包的差异只能形成 verifier 代码内固定的“受限 provenance 派生闭包”。归档内路径精确为 `share/slope-sim/runtime-manifest.json`、`share/slope-sim/selftest/session.manifest.pb`、`share/slope-sim/selftest/selftest-evidence.json`、`share/slope-sim/sbom.spdx.json`、`release-manifest.json` 和 `SHA256SUMS`；归档外只允许精确 archive/sidecar、`release-build-evidence.json`、release verification/handoff/context 的对应摘要变化。安装后的 candidate/final evidence 不做跨 prefix raw bytes 比较：verifier 必须在每一侧分别验证同一 run 的 resolved installed root、`install-state.json`、`relocation-state.json`、fresh doctor JSON 和 no-participant smoke JSON 五件套，证明 final path 等于各自 `current` resolve、marker/state/doctor/smoke 中的路径及摘要均等于实际文件，再只移除这些已验证的 prefix/path-bound 表示比较归一化功能语义；任意不匹配或借“路径不同”放宽非路径字段都失败。candidate acceptance handoff 和 accepted-candidate context 必须逐项冻结五个绝对路径、摘要、安装树 identity digest 和同一 run id，不能只保存 release handoff 或 evidence directory。闭包及字段 allowlist 不能由 archive、manifest、SBOM、context、handoff 或 CLI 自报；比较器必须先证明其余安装器/卸载器、Task 5 随包中文教程、MCAP segment/recipe/models、`bin/lib/include/runtime/python/ros-overlay`、资源/proto/descriptor/lock/许可证逐成员同路径同 bytes，再按 `runtime manifest -> SessionManifest.runtime_manifest_sha256 -> selftest evidence -> SPDX SBOM -> release manifest -> SHA256SUMS -> archive -> sidecar/build evidence -> handoff -> 各自 relocation/install/doctor/smoke evidence` 的固定无环顺序自底向上重算每条摘要边。SPDX 只允许上述上游文件 checksum、package verification code 和由其确定性派生的字段变化；runtime manifest 的 descriptor/lock/ABI/ELF 字段、session manifest 的 segment/topic/fence 字段和所有其他稳定字段必须相同。

正例只改变两个固定外部状态路径并确定性重算上述闭包，并把候选/正式包分别安装到两个不同绝对 prefix；候选先由 Task 7 Step 1 生成并冻结五件套，正式侧再完成全新安装、fresh doctor 和同 schema 的无 participant self-test/PCD/PLY/LVX2 smoke。比较器必须显式接收 accepted-candidate context 和正式五件套，最后以一次目录事务同时发布 `accepted-payload-equivalence.json` 和只引用该 JSON 的 equivalence handoff；handoff 固定 accepted context、final release handoff、candidate/final 五件套与 equivalence output 的绝对路径、SHA-256 和 run id。参数化反例必须覆盖：任一稳定 payload byte 变化后同步重写全部摘要、额外或改名闭包路径、第二份 SPDX、稳定字段漂移、每条摘要边断链、缺失/重复控制文件、闭包循环或自引用、final builder 使用新 commit timestamp 而非冻结 epoch、伪造 allowed diff、candidate/final handoff hash 漂移、缺验收 evidence、把 README/交付报告误装进 payload，以及以路径归一化名义删除非路径字段。对 candidate 与 final 两侧分别逐一省略、篡改或跨 run 混用 installed root/state/marker/doctor/smoke，freeze/compare 都必须失败且不得创建 accepted context、equivalence transaction 或 handoff。

所有由同一 verifier invocation 生成且会被后续 gate 消费的 JSON/handoff 或 archive/sidecar 组合都使用同一目录事务合同；这明确包括 `--publish-release-bundle` 提交 primary archive/sidecar/build evidence，以及 `--create-lifecycle-probe-handoff` 同时复制 probe 三件套并生成 handoff。调用者显式传入此前不存在、与输入/输出根互不包含的 `--transaction-dir`，所有 final paths 必须是该目录的直接普通文件成员；writer 在同父目录独占锁下创建 0700 sibling staging dir，以 exclusive create 写全部成员，逐文件 fsync、fsync staging dir 后只用一次 directory rename 暴露 final dir，再 fsync parent 才返回成功。消费者取得同一锁：只有 final dir 时结构化复核全部成员并补做 parent fsync 后方可接受；只有 staging、孤立 JSON/env/sidecar、成员不全或摘要不符时隔离并失败，绝不补写另一半。故障注入逐点覆盖两种 archive publisher 及全部 paired writer 的每个 copy/write/fsync、staging dir fsync、rename、parent fsync、rename 前后崩溃和并发 reader；rename 后 parent fsync 失败可通过完整重验+补 fsync 恢复，其他半成品只能隔离并在全新 transaction dir 重跑。fixture 还必须拒绝 shell `install + mv` 发布、源成员与 transaction dir 相互包含、目标预存在或 consumer 绕过 committed-dir 检查直接读取 builder output。

真实生命周期验收需要第二个合法版本，但不得加入生产故障注入。fixture 要求调用者提供与主版本不同的 canonical `STAGE4_LIFECYCLE_PROBE_VERSION`，从同一 candidate clean HEAD、lock/cache/toolchain 在第三个全新根构建一份完整 archive。builder 必须把 `artifact_purpose=lifecycle_probe`、`publishable=false` 同时写入归档内受 checksum 保护的 release manifest 和外部 build evidence；专用 lifecycle archive verifier 才能生成 portable JSON handoff，handoff 只用 sibling basename/hash 绑定 archive、sidecar 和 build evidence，并保存主 candidate 的不可变 identity，而不保存构建机绝对路径。普通 release archive verifier 和 `--write-handoff` 必须要求 `artifact_purpose=release`、`publishable=true`，不能靠绕过 probe handoff 把 probe archive 变成 release handoff。正例要求版本不同但 clean commit/tree、source snapshot、lock/cache/toolchain 和构建 schema 与主 candidate 相同，并要求 archive verifier 与 handoff writer 都消费 candidate-context transaction 实际提交的同一个 env 精确路径，不能按 basename 猜到 transaction 父目录。RED 覆盖同版本、非法版本、来源/lock 漂移、manifest/evidence purpose 不同或缺失、缺失/篡改 sibling、绝对或逃逸 basename、主 candidate identity 不同、把 candidate context 改为父目录下不存在的同名文件、直接把 probe archive 交给普通 `--archive ... --write-handoff`、把 probe handoff 当 release/acceptance/final handoff、把 probe 归档发布到主目录或纳入 candidate/final payload equivalence；全部失败。post-install doctor 失败继续由 Task 4 临时 prefix fixture 注入并验证旧 `current`，真实目标机不携带或启用任何 test-only fail hook。

clean-host 复验和真实生命周期都必须使用结构化 oracle。fixture 让 bundled verifier 以显式互斥的 `--clean-host-run-role initial|repeat` 为两次不同 run 各原子生成绑定 root/state/marker/doctor/smoke、安装树 identity digest、run id 和 role 的 shell-safe handoff，再以 `--freeze-clean-host-chain` 显式接收 initial/repeat 两个 handoff 并原子写 chain JSON/env。生命周期正例先让 `--verify-lifecycle-probe-handoff --write-lifecycle-probe-context` 在全新 `--transaction-dir` 内同时提交 preflight JSON/env，再由独立 `--verify-lifecycle-probe-context` 消费同一已提交目录并成功后才允许 `source`；不能顺序写两个文件或直接信任 producer 的退出码。随后用 `--snapshot-lifecycle-state --role before|upgraded` 冻结升级前主版本和升级后 probe 的 root/state/marker/fresh doctor、配置/数据 tree identity，再在 `--activate-existing` 回退并卸载非 current probe 后用 `--freeze-lifecycle-evidence` 证明 `current` 恢复主版本、probe 版本目录已消失且配置/数据未变。lifecycle evidence、doctor、probe 解包和任何临时输出都必须位于本轮 0700 transfer root，且与 `~/.config/slope-sim`、`~/slope-sim-data` 两个受保护树互不包含；在 snapshot 前结构化验证 realpath 边界。参数化 RED 覆盖 probe-context 任一 write/fsync/rename/crash 故障、孤立 JSON/env、未复核即 source、缺任一 run/snapshot/handoff、同一 run、role 颠倒/重复、篡改 doctor/smoke/root/state/marker、跨安装混用、任一输出根等于/位于/包含 config 或 data、probe 与主版本相同、未回退即卸载、probe 仍存在或只改 Markdown 标记；全部不得创建或消费 context、chain 或 lifecycle evidence。Task 8 失败时停止且不自动补跑，所以成功路径精确冻结显式角色和版本，不从历史目录推断“最新”。

真实 production evidence 也必须先冻结。fixture 先用 `--begin-clean-host-production-session` 从 initial run handoff 原子创建唯一 session context，要求 headless、interactive、live ROS、replay ROS 四个不同 role 的结构化结果都绑定该 session id 和同一 candidate install/source identity；`--freeze-clean-host-production-evidence` 显式消费 session context，并按代码内 schema 递归展开完整 session manifest/全部 MCAP segment、PCD/PLY/LVX2、GUI/RViz2/Livox Viewer 截图、工具打开结果、安装/运行日志和 host inventory。RED 逐一覆盖缺 role/member、重复或旧 role、跨 production session/anchor/run/candidate、失败 session 后混入旧成功结果、引用文件篡改、路径逃逸、链接/特殊文件、超 member/byte 上限和 caller 自报 allowlist，任何失败都不得创建 production JSON/handoff。

首次 clean-host 传输不能在目标机命令中保留 `<version>` 或用 glob 猜 archive。fixture 让 `--create-clean-host-transfer-context` 显式消费已经独立复核的 primary release handoff 与 lifecycle-probe handoff，验证 canonical release version、精确 archive/sidecar/build-evidence basename/hash、payload 顶层目录和 probe bundle/handoff basename，再按通用目录事务提交只含 portable basename/value 的 JSON、shell-safe env 与覆盖前两者的 `SHA256SUMS`。控制机经 pinned SSH host key 原样复制整个 committed context；目标机先在 context 目录运行 `sha256sum -c`，再 source env 并逐项要求 basename 无 `/`、版本/归档/顶层目录关系精确，才允许校验和解包。RED 覆盖字面量 `<version>`、任一绝对/逃逸/额外 basename、版本与 archive/payload root 不一致、context/env/checksum 缺失或篡改、换 primary/probe identity、只复制 env、不经 checksum 即 source，以及任一事务故障；全部必须在创建 extract/install root 前失败。

远端 run/chain/lifecycle/production handoff 中的 `/opt/...` 与 `$HOME/...` 只在目标机有效，不能直接交给验收控制机。fixture 要求目标机 bundled verifier 在这些路径仍可重算时用 `--export-clean-host-evidence-bundle` 生成唯一 deterministic portable `tar.zst` 和 sidecar；archive 内只有 canonical manifest JSON、两次 run、chain、lifecycle、production evidence 及其代码内固定的实际 evidence members，禁止 shell、链接、特殊/重复/逃逸成员并固定 member/byte 上限。manifest 绑定安装树、目标机、candidate/probe archive、一次性 challenge 和所有成员摘要。控制机只允许经预先固定的 SSH host key 拉回 bundle/sidecar，并以安全 archive reader 导入，不能先用系统 `tar` 盲解。challenge 在 import root 外的持久 registry 中以 exclusive create 登记为 `issued`；importer 在任何本地输出可见前持锁原子转为 `consuming`，验证和 fsync 全部 imported members/context 后再转为 `consumed` 并写 receipt，崩溃停在 consuming 时 fail closed、必须签发新 challenge。相同 challenge/bundle 即使复制到另一个全新 import root 也因 registry 状态非 issued 而失败。`--import-clean-host-evidence-bundle` 显式接收 bundle/sidecar、原始 challenge、registry、candidate release handoff、lifecycle-probe handoff、期望 target identity 和 pinned host-key fingerprint，成功才生成 `clean-host-import-context.json/env` 及本地 imported chain/lifecycle/production evidence。RED 覆盖 archive/member/sidecar 篡改、challenge 重放/错误或状态跳转、换 host/root/archive、错误 host-key fingerprint、缺 lifecycle/production/raw member、直接复制远端 env 后路径失效，以及 caller 自报 allowlist；全部不得创建 import context。Task 9 的 complete-evidence 和 accepted-candidate freeze 必须显式接收并复核本地 import context、imported lifecycle/production evidence、consumption receipt 及 lifecycle-probe handoff，不能解析远端路径或从报告推断 run 选择。

独立六维审查也必须先冻结为仓库外不可变事务，不能消费仍会在 final status 后更新的 `docs/阶段四交付报告.md`。fixture 让独立审查任务输出一份 canonical review source，精确包含 reviewer identity/task id、被审 commit/tree、candidate/acceptance/clean-host-import/lifecycle-probe identity、需求完整性/逻辑正确性/边界情况/代码质量/测试覆盖/实际运行结果六个且仅六个维度、逐维 verdict、全部发现及 disposition，以及逐项 path/size/SHA-256 evidence index。`--freeze-six-dimension-review` 必须重新读取并校验这些 evidence bytes，要求 `Critical=0, Important=0`，再按通用目录事务同时提交 `six-dimension-review.json` 和 handoff；输出内嵌规范化审查内容与 evidence index，不能只引用可变 source。`--verify-six-dimension-review-handoff` 独立复核成功后才可进入 complete-evidence 和 accepted-candidate freeze。RED 逐一覆盖 reviewer identity 缺失、维度缺失/重复/未知、非零 Critical/Important、未处置发现、被审 commit 或任一 candidate/import/probe identity 错配、evidence 缺失/篡改/跨 run/逃逸，以及 review JSON/handoff 任一事务故障；还要先冻结 accepted context 和 final status，再修改报告/README，证明两者仍可复核且其摘要从未成为上游输入。旧的、把交付报告路径传给 `--complete-evidence` 的形式必须被参数解析拒绝。

同一 RED 还要以 wished-for `--finalize-release-status` 完整 CLI 验证唯一完成门：它必须同时接收 accepted-candidate context、final release handoff、payload equivalence、final installed root/state/marker/doctor/smoke，并只从 accepted context 读取已冻结的真实 eCAL、GUI/RViz2、Livox Viewer、干净机和不可变六维审查事务。正例最后按上面的单次目录事务同时发布 canonical `final-release-status.json` 和 shell-safe handoff，状态精确为 `complete`；逐一缺失、篡改、使用旧 equivalence、错误 run id、跨 final 安装混用、在 equivalence 后改写 evidence、把候选报告摘要冒充正式 smoke，以及 JSON/handoff 任一 write/fsync/rename/crash 故障都必须非零退出且 final transaction 不可被消费。只有 committed transaction 内、经 handoff fresh 验证的状态 JSON 可作为 Task 9 写“完成”的依据；孤立 JSON、archive verifier、candidate complete-evidence、六维 review source 或 equivalence 单独通过都不能替代它。

两根可读取同一只读 canonical Python package artifact、官方 eCAL wheel artifact 和 source archive artifact，但必须各自复制到不同 `mamba-root/pkgs`、`wheel-cache`、`cpp-sources/archives|trees`、`cpp-deps-build|install`、`validation-prefix` 与 `ros-sources/archives|trees`；测试拒绝共享可写 cache/wheel、tool env、runtime env、C++ dependency/source archive 副本/解包/构建/安装树、package extraction 或复用第一轮中间物，也拒绝 E 调用 conda-lock/render/solve/download/Git fetch/pip index。wheel 反例覆盖缺失/篡改/错误 ABI/tag/license/NOTICE/RECORD/ELF inventory、联网 fallback 和 Python/C++ eCAL/libprotobuf 双重或交叉加载；source cache 反例覆盖缺失/多余归档、size/SHA/tree/member/materialized digest/consumer 漂移、cache 链接、恶意 member、同 basename 不同 hash、直接在 canonical root 解包和缺包联网 fallback。C++ 反例额外传入共享预构建 `STAGE4_DEPENDENCY_PREFIX` 并要求 final 在 configure 前拒绝，两个私有安装树的 manifest/hash/ELF closure 必须相同且绝不引用另一根。再分别注入 staged tracked 修改、unstaged tracked 修改、会被 package/resource/docs/packaging 输入选中的 untracked 文件，以及即使两根都使用相同 dirty bytes 的反例；都必须在创建 work/output 内容前失败，不能仍登记 HEAD Git SHA。另注入 archive 自引用 SHA、work-root/source/cache/builder/history/conda-unpack 路径泄漏、ELF debug path、CMake/pkg-config 绝对前缀、wheel/pyc timestamp 或 `co_filename` 漂移、在 conda-pack 前删除 managed pyc 或安装任一 wheel、第二轮复用第一轮产物、glob 选错旧归档、不同 Git/lock/cache/toolchain/source-archive/dependency-builder/source snapshot digest、release tree 遗留任一 link 和输出目录非空。构建端 archive verifier 的恶意成员 oracle 独立覆盖绝对路径、`../`、控制字符、重复成员、文件/目录类型冲突、符号/硬链接、device/FIFO/socket、清单外成员、声明大小不符和总展开膨胀超限；正式产物只能含唯一 payload 根下受支持的普通文件/目录。Task 4 的 `test_installer.py` 只验证已解源树，两个测试不能复用同一个受测 helper 作为彼此 oracle。测试函数先断言 final-archive 入口存在，缺失时得到明确 `FAILED`；不得依赖真实 Conda、ROS、eCAL、网络或完整发行构建机。

- [ ] **Step 2: 运行 archive RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_release_archive.py`

Expected: pytest 正常收集并 `FAILED`，失败断言只指向 final archive/reproducibility 行为尚未实现；不得是 collection/fixture error、skip 或缺外部工具。

- [ ] **Step 3: 运行 Python/C++/ROS 纯测试**

Run serially:

```bash
test -n "${STAGE4_BUILD_ENV_FILE:-}"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-env "$STAGE4_BUILD_ENV_FILE" \
  --json "$STAGE4_BUILD_ENV_FILE.preflight.json"
source "$STAGE4_BUILD_ENV_FILE"
test -n "${STAGE4_EXTERNAL_ACCEPTANCE_PARENT:-}"
test -d "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT"
STAGE4_ACCEPTANCE_RUN_PARENT="$(mktemp -d \
  "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT/slope-sim-stage4-acceptance.XXXXXX")"
STAGE4_ACCEPTANCE_WORK_ROOT="$STAGE4_ACCEPTANCE_RUN_PARENT/work"
STAGE4_ACCEPTANCE_EVIDENCE_DIR="$STAGE4_ACCEPTANCE_RUN_PARENT/evidence"
install -d "$STAGE4_ACCEPTANCE_WORK_ROOT" "$STAGE4_ACCEPTANCE_EVIDENCE_DIR"
export STAGE4_ACCEPTANCE_WORK_ROOT STAGE4_ACCEPTANCE_EVIDENCE_DIR
conda run -n slope-sim python scripts/verify_python_lock_cache.py \
  --environment packaging/python-environment.yml \
  --toolchain-environment packaging/python-toolchain-environment.yml \
  --virtual-packages packaging/locks/virtual-packages.yml \
  --runtime-unified packaging/locks/python.conda-lock.yml \
  --runtime-explicit packaging/locks/python-linux-64.lock \
  --toolchain-unified packaging/locks/python-toolchain.conda-lock.yml \
  --toolchain-explicit packaging/locks/python-toolchain-linux-64.lock \
  --toolchain packaging/locks/python-toolchain.lock \
  --cache-manifest packaging/locks/python-package-cache.manifest.json \
  --cache-root "$STAGE4_PYTHON_PACKAGE_CACHE" \
  --micromamba "$STAGE4_MICROMAMBA"
conda run -n slope-sim python scripts/verify_python_wheel_cache.py \
  --manifest packaging/locks/python-wheel-cache.manifest.json \
  --cache-root "$STAGE4_PYTHON_WHEEL_CACHE" \
  --python-tag cp310 --abi-tag cp310 \
  --platform-tag manylinux_2_28_x86_64
conda run -n slope-sim python scripts/verify_stage4_source_cache.py \
  --manifest packaging/locks/source-archive-cache.manifest.json \
  --lock packaging/locks/cpp-dependencies.lock \
  --lock packaging/locks/ros2-dependencies.lock \
  --cache-root "$STAGE4_SOURCE_ARCHIVE_CACHE"
conda run -n slope-sim python -m pytest -q \
  --ignore=tests/stage4/test_release_archive.py -m "not ecal"
bash packaging/run_network_isolated.sh bash packaging/build_dependencies.sh \
  --lock packaging/locks/cpp-dependencies.lock \
  --source-cache-manifest packaging/locks/source-archive-cache.manifest.json \
  --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE" \
  --source-work "$STAGE4_ACCEPTANCE_WORK_ROOT/cpp-sources" \
  --build-root "$STAGE4_ACCEPTANCE_WORK_ROOT/cpp-deps-build" \
  --prefix "$STAGE4_ACCEPTANCE_WORK_ROOT/cpp-deps-install" \
  --validation-prefix "$STAGE4_ACCEPTANCE_WORK_ROOT/validation-prefix"
STAGE4_DEPENDENCY_PREFIX="$STAGE4_ACCEPTANCE_WORK_ROOT/cpp-deps-install"
STAGE4_CMAKE_PREFIX_PATH="$STAGE4_DEPENDENCY_PREFIX"
STAGE4_PROTOC="$STAGE4_DEPENDENCY_PREFIX/bin/protoc"
STAGE4_PCL_PCD2PLY="$STAGE4_ACCEPTANCE_WORK_ROOT/validation-prefix/bin/pcl_pcd2ply"
export STAGE4_DEPENDENCY_PREFIX STAGE4_CMAKE_PREFIX_PATH STAGE4_PROTOC
export STAGE4_PCL_PCD2PLY
bash packaging/run_network_isolated.sh "$STAGE4_CMAKE" --preset stage4-release \
  -S "$PWD" -B "$STAGE4_ACCEPTANCE_WORK_ROOT/cmake-build"
bash packaging/run_network_isolated.sh "$STAGE4_CMAKE" \
  --build "$STAGE4_ACCEPTANCE_WORK_ROOT/cmake-build" --parallel 2
bash packaging/run_network_isolated.sh "$STAGE4_CTEST" \
  --test-dir "$STAGE4_ACCEPTANCE_WORK_ROOT/cmake-build" \
  --output-on-failure --no-tests=error
bash packaging/run_network_isolated.sh "$STAGE4_CMAKE" \
  --install "$STAGE4_ACCEPTANCE_WORK_ROOT/cmake-build" \
  --prefix "$STAGE4_ACCEPTANCE_WORK_ROOT/client-install"
bash packaging/run_network_isolated.sh bash packaging/stage_cpp_runtime.sh \
  --dependency-prefix "$STAGE4_DEPENDENCY_PREFIX" \
  --project-prefix "$STAGE4_ACCEPTANCE_WORK_ROOT/client-install" --mode sdk
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-env "$STAGE4_BUILD_ENV_FILE" --require-ros-lock-closure \
  --json "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/e-ros-prerequisites.json"
source /opt/ros/jazzy/setup.bash
bash packaging/run_network_isolated.sh env \
  CC="$STAGE4_CC" CXX="$STAGE4_CXX" \
  bash packaging/build_ros_overlay.sh \
  --lock packaging/locks/ros2-dependencies.lock \
  --source-cache-manifest packaging/locks/source-archive-cache.manifest.json \
  --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE" \
  --source-work "$STAGE4_ACCEPTANCE_WORK_ROOT/ros-sources" \
  --livox-sdk-prefix "$STAGE4_ACCEPTANCE_WORK_ROOT/livox-sdk-install" \
  --build-base "$STAGE4_ACCEPTANCE_WORK_ROOT/ros-build" \
  --project-source "$PWD/ros2" \
  --client-prefix "$STAGE4_ACCEPTANCE_WORK_ROOT/client-install" \
  --install-base "$STAGE4_ACCEPTANCE_WORK_ROOT/ros-install"
source "$STAGE4_ACCEPTANCE_WORK_ROOT/ros-install/setup.bash"
bash packaging/run_network_isolated.sh colcon test \
  --packages-select slope_sim_msgs slope_sim_bridge \
  --build-base "$STAGE4_ACCEPTANCE_WORK_ROOT/ros-build" \
  --install-base "$STAGE4_ACCEPTANCE_WORK_ROOT/ros-install" \
  --event-handlers console_direct+ \
  --return-code-on-test-failure
bash packaging/run_network_isolated.sh colcon test-result \
  --test-result-base "$STAGE4_ACCEPTANCE_WORK_ROOT/ros-build" --verbose
```

`STAGE4_BUILD_ENV_FILE` 必须由总计划 Task 2 的探针原子生成并通过 hash 复核，所有工具/cache/样例/RViz2 路径在 source 后立即 preflight；`STAGE4_ACCEPTANCE_WORK_ROOT` 与 evidence dir 由本 Step 在仓库外新建，`client-install` 由同一任务的私有 C++ dependency prefix、CMake install 和 `stage_cpp_runtime.sh` 产生。Python/wheel/source 前置检查只验证总计划 Task 2 的冻结产物，不生成锁、不下载包；全部 configure/build 都在已验证断网 wrapper 内。ROS builder 只读同一 canonical source root，并把 SDK/driver 归档复制/解包到本任务私有 `ros-sources`，Livox SDK 只安装到私有 `livox-sdk-install`，真实 `/usr/local` census 前后不变。ROS lock 前置检查要求 lock 中完整列出 Jazzy、RViz2、Livox-SDK2、完整 `livox_ros_driver2` 构建/运行包和允许 SONAME；不得 source D 阶段遗留的 `build/stage4-ros-install`、source tree、开发 dependency prefix 或其他 overlay。此时 final archive 测试仍处于 RED，所以全量 pytest 明确只临时排除 `test_release_archive.py`；Step 7 GREEN 后必须补跑不排除的完整非 eCAL 套件。Expected: 全部 rc=0；报告记录精确 passed/deselected 和耗时，以及 package/wheel/source manifest/tree/private materialization、C++ dependency install tree 和 `/usr/local` census digest。

- [ ] **Step 4: 跑四车型三地形 DIRECT**

Run: `conda run -n slope-sim python scripts/verify_stage4_sensors.py --all-models --all-terrains --output "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/final-direct.json"`

Expected: `SUMMARY pass=12 fail=0`。

- [ ] **Step 5: 跑真实桌面和三个 Xvfb**

严格使用子计划 B 的四条 GUI 命令，一次一个，但不得复用 B 阶段或本 Step 前一条命令的授权。每条执行前都重新说明真实桌面或临时 Xvfb、分辨率和 4 秒时长，取得只覆盖紧随命令的明确授权，并即时扫描全机 pytest、GUI/Xvfb、PyBullet、eCAL 和系统负载。任一失败都保留证据并停止，不继续下一条、不自动重跑；候选复测重新授权。每次结束确认临时 Xvfb、PyBullet 和 Qt 子进程清理，长期 `:1` 不操作其他桌面进程。

Expected: 四条 rc=0、15/15 页、33% Dashboard、50:50 内部分区、全部点击/滚动/文字/artist 和持续驾驶通过。

- [ ] **Step 6: 实现 final-archive 模式和精确生成物 ignore**

只实现 `test_release_archive.py` 当前 RED 所要求的最小 final 行为和候选/final 功能 payload 比较，不执行真实双根构建。`.gitignore` 增加锚定的 `/build/stage4*/` 与 `/results/stage4/` 等阶段四生成物规则，并由测试证明源码、lock、manifest、evidence 模板和任一非生成路径不会被吞掉。`build-source-manifest.yml` 明确把 `README.md`、`docs/阶段四交付报告.md` 和运行 evidence 排除在发行 payload 外；Task 5 的部署/SDK/接口/人工测试/回放/故障排查教程仍是不可排除的发行输入。`build_release.sh --final-archive` 在创建输出前执行 clean gate，从精确 HEAD 的 `git archive` 生成只读 `source-snapshot`，复制为私有 `source-build`；只读消费 package/wheel/source 三类 canonical artifact，在本轮依次创建私有 Python cache/env、`cpp-sources/cpp-deps-build/cpp-deps-install/validation-prefix`、项目 CMake/ROS/安装树和 smoke。final 模式必须忽略或主动拒绝继承的开发 `STAGE4_DEPENDENCY_PREFIX/CMAKE_PREFIX_PATH/PROTOC/PCL`，只使用本轮重建后设置的值。每次调用还必须显式给出互斥的 `--artifact-purpose release|lifecycle_probe`：前者固定 `publishable=true`，后者固定 `publishable=false`，并把这两个字段写入归档内 release manifest 与外部 build evidence；caller 不能独立覆盖 publishable。普通 release verifier 只接受前者，专用 lifecycle verifier 只接受后者。候选构建从当前 clean commit timestamp 得到并记录 `functional_source_epoch`；带 `--accepted-candidate-context` 的正式重建先结构化验证 context 和固定 source diff，再继承候选 epoch，但仍把最终 commit/tree/source snapshot 写入 provenance。两种模式都固定所有 prefix-map，生成精确命名 archive、sidecar 与 build evidence；不新增求解、下载、pip index、Git fetch、共享可写中间物或仓库内输出。

`verify_stage4_release.py` 同时实现受结构化测试约束的 acceptance/accepted-candidate context、功能 payload 比较和 final-status gate。它使用代码内固定的 source/path/field allowlist，分别通过安全 archive reader、规范 JSON/SPDX parser、生成的 `SessionManifest` Protobuf parser 和严格 `SHA256SUMS` parser 读取两份产物；先比较闭包外全部路径和 bytes，再从稳定叶子按 Step 1 的固定 DAG 自底向上复算闭包，最后交叉验证 archive、sidecar、build evidence、context/handoff。安装 provenance 则先分别绑定各自 resolved prefix、`install-state.json`、`relocation-state.json`、fresh doctor 和 no-participant smoke，再只剥离固定的 path-bound 字段比较归一化语义。

同一脚本的 `--run-installed-no-participant-smoke` 接受已安装 root/state/marker、fresh doctor 和原子 output；Task 7 候选与 Task 9 正式比较还必须传入各自已验证的 release handoff，bundled clean-host 模式则从 `verified_archive` install state 中重算 archive/build-evidence/source identity，不允许 `build_smoke` 或缺来源字段。在清除 checkout/PATH 注入后，它执行 Task 8 Step 2 的 C++ `--version`、Python import、模型、自带 session 五 topic 回读和 PCD/PLY/LVX2 导出，不创建任何 eCAL participant，并在调用者同时要求 run handoff 时按通用 `--transaction-dir` 合同发布 smoke JSON/handoff。clean-host 模式还要求互斥的 `--clean-host-run-role initial|repeat`，handoff 绑定 root/state/marker/doctor/smoke、安装来源、run id 和 role；`--freeze-clean-host-chain` 只接受两个已独立复核且角色分别为 initial/repeat 的 handoff，要求不同 run、来源/归一化结果一致后，以相同目录事务发布 chain JSON/env。`--snapshot-lifecycle-state` 与 `--freeze-lifecycle-evidence` 以相同严格 parser 记录真实 probe 升级、回退和卸载；`--freeze-clean-host-production-evidence` 从四个固定 role 的结果 schema 递归冻结实际 MCAP/导出/截图/日志/inventory members，不接受 caller 成员表。

控制机的 `--create-clean-host-transfer-context` 从已验证 release/probe handoff 派生唯一 portable version/basename/hash 集合，并以目录事务提交 JSON、env 和内部 `SHA256SUMS`；它不输出控制机绝对路径。目标机只在 pinned SSH 复制完整 transaction、`sha256sum -c` 和 basename/版本关系复核成功后 source env，禁止 `<version>`、glob 或重新发现 artifact。后续 `--export-clean-host-evidence-bundle` 必须在所有远端路径仍有效时重算 chain/lifecycle/production 及安装实值，再构造 deterministic `tar.zst` 与 sidecar。控制机的 `--create-clean-host-challenge` 在 import root 外的 0700 registry 以 exclusive create 写 `issued` record；`--import-clean-host-evidence-bundle` 用安全 archive reader 和同一 registry 实现 fail-closed 的 `issued -> consuming -> consumed`，只在 pinned SSH transfer、challenge、host/candidate/probe identity、sidecar 和全部成员摘要匹配后写本地 imported evidence、consumption receipt 与 shell-safe import context。context 精确导出 `STAGE4_IMPORTED_CLEAN_HOST_CHAIN_EVIDENCE`、`STAGE4_IMPORTED_LIFECYCLE_EVIDENCE`、`STAGE4_IMPORTED_CLEAN_HOST_PRODUCTION_EVIDENCE` 和 `STAGE4_CLEAN_HOST_CHALLENGE_RECEIPT` 四个本地绝对路径。`--verify-clean-host-import-context` 重算本地 bundle/imported files 并复核 registry consumed record，不假装能重新读取目标机路径。

`--publish-release-bundle` 从双根比较已通过的明确 A 输出读取 archive/sidecar/build evidence，并以通用 `--transaction-dir` 合同提交唯一 primary bundle；candidate-context freezer 必须显式接收并验证这个 committed directory。`--create-lifecycle-probe-handoff` 同样从明确 lifecycle builder output 读取 probe 三件套，在 sibling staging 内生成 portable handoff 后一次提交整个四文件 bundle，不接受调用者预建 staging 或 `--probe-bundle-root`。`--verify-lifecycle-probe-handoff --write-lifecycle-probe-context` 以相同合同同时提交 preflight JSON/env，`--verify-lifecycle-probe-context` 作为独立 consumer 在 `source` 前重算 pair、probe/primary identity 和目录提交状态。`--freeze-acceptance-install` 只有在 candidate root/state/marker/doctor/smoke 五件套同轮互绑后才写 acceptance transaction；`--freeze-six-dimension-review` 把独立 reviewer source 的身份、精确六维 verdict/findings/disposition 和实际 evidence index 规范化进仓库外 review JSON/handoff；`--freeze-accepted-candidate` 再把候选五件套、显式 clean-host import context、imported chain/lifecycle/production evidence、consumed challenge receipt、lifecycle-probe handoff 和已经独立复核的六维 review handoff 逐一冻结进 accepted context transaction，绝不读取或冻结 README/交付报告。`--compare-accepted-payload` 要求 accepted context 中候选五件套与五个显式 final 参数全部已存在且各自同轮互绑；lifecycle probe 只用于候选目标机的安装事务验收，严禁进入 primary archive、candidate/final payload equivalence 或正式发布目录。比较器不负责安装或运行 smoke，只在全部输入通过后按通用 `--transaction-dir` 合同同时发布 equivalence JSON/handoff。`--finalize-release-status` 显式消费 accepted context、final release handoff、equivalence 与 final 五件套，重新验证所有摘要后才以同合同发布 final-status JSON/handoff。所有 paired writer/consumer 都使用同一父目录锁、sibling staging、逐文件/目录 fsync、单次 directory rename 和 parent fsync；消费者对 rename 后未确认 parent fsync 的完整目录只允许重验并补 fsync，对孤立成员或 staging 只隔离失败。任何额外路径/字段、未知 schema、重复 key/member、第二份 SBOM、摘要断链、循环/自引用、稳定字段变化、任一侧缺失/篡改/跨 run 或事后才生成的 evidence、clean-host role/challenge/registry/host 错配、安装实值错配、陈旧 equivalence、过度归一化或 caller 提供的 allowlist 都失败；不得用文本 diff、受测 manifest 自己、文件扩展名猜测或“所有摘要最终一致”充当 oracle。

- [ ] **Step 7: 运行 archive GREEN 和完整非 eCAL 回归**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_release_archive.py`

Expected: PASS；本地最小 Git/package/wheel/source fixture 完成双根 archive、dependency/runtime tree 与 evidence 比较，不需要真实 Conda、ROS、eCAL 或网络。

Run: `conda run -n slope-sim python -m pytest -q -m "not ecal"`

Expected: PASS；不再排除 `test_release_archive.py`，不得 skip 或 deselect 该文件。

- [ ] **Step 8: REFACTOR final builder 并原样复验**

只整理已 GREEN 的 clean-source、tree digest、tar member 规范化和 evidence invariant 比较重复；不得改动 wheel 安装顺序、每轮私有 dependency rebuild、source snapshot、断网门、root exchange 事务或输出位置。无需整理时记录“REFACTOR：无必要”，随后原样重跑 Step 7 两条命令。

- [ ] **Step 9: 写已安装 release-root 来源与完整 oracle RED**

在 C 的 verifier fixture 上新增 `--release-root` 合同：只接受 `readlink -f` 后的已安装 `current` 目标，要求 `install-state.json` 的 `verified_archive` provenance、runtime manifest、descriptor/lock/ELF hash、archive basename/SHA、build-evidence SHA、clean Git commit/tree/source snapshot、lock/builder digest 和 doctor 全部互相绑定；拒绝 Task 3 builder root、build-smoke state、dirty/development source provenance、未安装 packed payload、相对路径、PATH 同名工具、缺 install state、来源字段错配和 state 指向另一版本。入口把 `--runtime-mode headless|interactive` 设为必填；interactive 还必须显式选择 `--with-dashboard` 和 `--ros-mode off|on`，其中 on 必须同时启动 Bridge 与 RViz2。结果字段来自实际进程/连接/profiler 状态，而不是 CLI 回显：interactive 必须满足 `runtime_mode=interactive`、`pybullet_connection_mode=GUI`、`dashboard_enabled=true`、`gui_event_max_gap_ms<=100`、`dashboard_draw_sample_count>=5` 和 `dashboard_draw_p95_ms<100`；ROS off 要求 Bridge/RViz2 均未启动，ROS on 要求二者 READY。既有反例继续覆盖请求模式与实际模式不同、interactive 未启 Dashboard、GUI/绘图样本缺失或超限、ROS 实际状态错配、CLI 请求 20 但 runtime 实际不是 20 个障碍物、LiDAR 实际不是 `realtime_mid360`/5,760 候选射线、窗口首尾丢帧、单个 20ms wheel gap、command 不足 95Hz、timestamp 错频、RTK 全零、轨迹不足、转向未响应、queue/pending 未排空、worker 退出、MCAP 多/少/重复消息和 fence 缺失。

- [ ] **Step 10: 运行 release-root RED**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_ecal_cpp_verifier.py`

Expected: pytest 正常收集并 `FAILED`，失败断言只指向 `--release-root` 来源或 interactive/ROS 实际 workload 校验尚未实现；C 已有缺帧/假运动反例继续通过，不启动真实 eCAL、GUI 或 RViz2。

- [ ] **Step 11: 实现已安装 release-root 解析**

只扩展 C 的同一 verifier：从显式绝对 `--release-root` 读取 sibling C++ ELF、bundled Python、share/runtime manifest 和 install state，完成不可变来源校验后再复用原三方 oracle；interactive 通过 E 的正式编排器启动 GUI Simulator、Dashboard、Subscriber、Command、Recorder 和按开关选择的 Bridge/RViz2，并把实际 role 状态与 B 的有界 profiler 快照交给同一结果 oracle。禁止复制比较逻辑、支持 PATH fallback、接受 `--client-prefix` 与 `--release-root` 同时出现，或以 headless 默认值掩盖缺失的显式 mode。

- [ ] **Step 12: 运行 release-root GREEN 与 REFACTOR 复验**

Run: `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_ecal_cpp_verifier.py`

Expected: PASS；C 的全部反例以及 E 的安装来源、实际 runtime/GUI/Dashboard/ROS 状态和性能反例均精确失败。完成必要整理后原样重跑；无需整理则记录“REFACTOR：无必要”。

- [ ] **Step 13: 取得验收候选 clean HEAD 的明确 Git 授权检查点**

真实验收候选只能代表已经提交的实现和随包文档。先展示计划内实现/测试/随包文档 diff、Steps 7-12 的 fresh GREEN/REFACTOR 证据和待提交文件清单，确认所有阶段四生成物都由精确 ignore 排除；然后停止并向用户请求明确 commit 授权。未获授权时不得自行 commit，也不得执行候选双根构建，状态保持“acceptance candidate 未执行”。获授权后按项目 `AGENTS.md` 的阶段四提交摘要规则创建 commit，再要求 `git status --porcelain=v1 --untracked-files=all` 无输出、`git diff --cached` 无输出，并记录新的 HEAD/tree。此后代码、lock、测试、packaging、Task 5 随包教程或任何功能 payload 输入变化都会使候选和验收证据失效，必须回到拥有行为的 RED/GREEN、重新获 commit 授权并重做本 Step。Task 7-9 只允许更新不会进入 payload 的 `README.md`、`docs/阶段四交付报告.md` 和仓库外 evidence；它们不会倒改候选 bytes，但最终发布前仍必须在 Task 9 再次获授权提交，并从新 clean HEAD 重建正式双根。

- [ ] **Step 14: 在两个全新空根构建验收候选**

Task 4 的 `install.sh/uninstall.sh`、Task 5 的全部中文教程、本 Task Steps 3-12 的 GREEN/REFACTOR 证据和 Step 13 获授权的 clean HEAD 全部存在后，才执行已经 GREEN 的 final-archive 模式生成真实验收候选。每次入口在创建任何 work/output 内容前要求 `git status --porcelain=v1 --untracked-files=all` 无输出，拒绝 staged、unstaged 和任一 untracked 项；final 模式还拒绝 work/output/evidence 位于仓库内、仓库子目录或包含仓库。随后把精确 `HEAD` 以 `git archive` 解到本轮 work root 内 `source-snapshot/`，计算规范成员/mode/bytes SHA-256 并递归移除写权限；再通过 Task 3 的 materializer exclusive-copy 到本轮独立可写 `source-build/`，逐成员复算相同 digest。setuptools/CMake/ROS/资源安装只消费 source-build，构建前后重算 source-snapshot 并要求完全未变；这仍是 snapshot 的确定性派生，不授权读取当前 checkout。不得把 `stage-only` 的 `development_worktree` evidence 改名复用。此处的 archive 具备完整生产格式，但在 Task 7/8 真实验收和 Task 9 最终 clean-HEAD 重建前只能标为 acceptance candidate，不能对外发布。

`build_release.sh` 必须把 Python package/wheel cache 副本、C++ dependency source/build/install、项目 CMake、ROS、source-build、安装根和 smoke 全部派生到各自 work root，并把唯一结果写为精确文件名 `slope-sim-stage4-${STAGE4_RELEASE_VERSION}-ubuntu24.04-amd64.tar.zst`，不得扫描 `dist/*.tar.zst` 或复用另一根的中间产物。以下命令要求 `STAGE4_EXTERNAL_BUILD_PARENT` 是仓库外有足够空间的绝对目录，在其下现场创建一个私有 run parent，再创建两组互不包含的空 work/output sibling 和一个原子发布 sibling；脚本入口再次验证路径与空目录。A 完成后把仓库 clean 状态写到外部 A work root并断言为空，B 自身仍须重新执行同一 clean gate；这样 A 的产物不可能污染 B。

```bash
test -n "${STAGE4_RELEASE_VERSION:-}"
test -n "${STAGE4_LIFECYCLE_PROBE_VERSION:-}"
test -n "${STAGE4_BUILD_ENV_FILE:-}"
test -n "${STAGE4_EXTERNAL_BUILD_PARENT:-}"
test -d "$STAGE4_EXTERNAL_BUILD_PARENT"
validated_lifecycle_version="$(
  conda run -n slope-sim python scripts/verify_stage4_release.py \
    --validate-release-version "$STAGE4_LIFECYCLE_PROBE_VERSION"
)"
test "$validated_lifecycle_version" = "$STAGE4_LIFECYCLE_PROBE_VERSION"
test "$STAGE4_LIFECYCLE_PROBE_VERSION" != "$STAGE4_RELEASE_VERSION"
STAGE4_RELEASE_RUN_PARENT="$(mktemp -d \
  "$STAGE4_EXTERNAL_BUILD_PARENT/slope-sim-stage4-release.XXXXXX")"
conda run -n slope-sim python scripts/verify_stage4_dependencies.py \
  --verify-env "$STAGE4_BUILD_ENV_FILE" \
  --json "$STAGE4_RELEASE_RUN_PARENT/toolchain-preflight.json"
source "$STAGE4_BUILD_ENV_FILE"
test -x "$STAGE4_MICROMAMBA"
test -d "$STAGE4_PYTHON_PACKAGE_CACHE"
test -d "$STAGE4_PYTHON_WHEEL_CACHE"
test -d "$STAGE4_SOURCE_ARCHIVE_CACHE"
unset STAGE4_DEPENDENCY_PREFIX STAGE4_CMAKE_PREFIX_PATH
unset STAGE4_PROTOC STAGE4_PCL_PCD2PLY
STAGE4_RELEASE_WORK_A="$STAGE4_RELEASE_RUN_PARENT/work-a"
STAGE4_RELEASE_WORK_B="$STAGE4_RELEASE_RUN_PARENT/work-b"
STAGE4_RELEASE_OUTPUT_A="$STAGE4_RELEASE_RUN_PARENT/output-a"
STAGE4_RELEASE_OUTPUT_B="$STAGE4_RELEASE_RUN_PARENT/output-b"
STAGE4_FINAL_OUTPUT_PARENT="$STAGE4_RELEASE_RUN_PARENT/final"
install -d "$STAGE4_RELEASE_WORK_A" "$STAGE4_RELEASE_WORK_B" \
  "$STAGE4_RELEASE_OUTPUT_A" "$STAGE4_RELEASE_OUTPUT_B" \
  "$STAGE4_FINAL_OUTPUT_PARENT"
STAGE4_FINAL_OUTPUT_DIR="$STAGE4_FINAL_OUTPUT_PARENT/published"
test ! -e "$STAGE4_FINAL_OUTPUT_DIR"
STAGE4_ARCHIVE_NAME="slope-sim-stage4-${STAGE4_RELEASE_VERSION}-ubuntu24.04-amd64.tar.zst"
export STAGE4_RELEASE_WORK_A STAGE4_RELEASE_WORK_B
export STAGE4_RELEASE_OUTPUT_A STAGE4_RELEASE_OUTPUT_B
export STAGE4_FINAL_OUTPUT_DIR STAGE4_ARCHIVE_NAME
bash packaging/run_network_isolated.sh \
  bash packaging/build_release.sh \
    --final-archive \
    --work-root "$STAGE4_RELEASE_WORK_A" \
    --output-dir "$STAGE4_RELEASE_OUTPUT_A" \
    --release-version "$STAGE4_RELEASE_VERSION" \
    --artifact-purpose release \
    --micromamba "$STAGE4_MICROMAMBA" \
    --python-package-cache "$STAGE4_PYTHON_PACKAGE_CACHE" \
    --python-wheel-cache "$STAGE4_PYTHON_WHEEL_CACHE" \
    --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE"
git status --porcelain=v1 --untracked-files=all \
  > "$STAGE4_RELEASE_WORK_A/post-a-git-status.txt"
test ! -s "$STAGE4_RELEASE_WORK_A/post-a-git-status.txt"
bash packaging/run_network_isolated.sh \
  bash packaging/build_release.sh \
    --final-archive \
    --work-root "$STAGE4_RELEASE_WORK_B" \
    --output-dir "$STAGE4_RELEASE_OUTPUT_B" \
    --release-version "$STAGE4_RELEASE_VERSION" \
    --artifact-purpose release \
    --micromamba "$STAGE4_MICROMAMBA" \
    --python-package-cache "$STAGE4_PYTHON_PACKAGE_CACHE" \
    --python-wheel-cache "$STAGE4_PYTHON_WHEEL_CACHE" \
    --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --compare-dependency-trees \
  "$STAGE4_RELEASE_WORK_A/cpp-deps-install" \
  "$STAGE4_RELEASE_WORK_B/cpp-deps-install" \
  --compare-validation-trees \
  "$STAGE4_RELEASE_WORK_A/validation-prefix" \
  "$STAGE4_RELEASE_WORK_B/validation-prefix" \
  --output "$STAGE4_RELEASE_RUN_PARENT/cpp-dependency-reproducibility.json"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --compare-runtime-trees \
  "$STAGE4_RELEASE_WORK_A/root/runtime/python" \
  "$STAGE4_RELEASE_WORK_B/root/runtime/python" \
  --output "$STAGE4_RELEASE_RUN_PARENT/python-runtime-reproducibility.json"
cmp \
  "$STAGE4_RELEASE_OUTPUT_A/$STAGE4_ARCHIVE_NAME" \
  "$STAGE4_RELEASE_OUTPUT_B/$STAGE4_ARCHIVE_NAME"
sha256sum \
  "$STAGE4_RELEASE_OUTPUT_A/$STAGE4_ARCHIVE_NAME" \
  "$STAGE4_RELEASE_OUTPUT_B/$STAGE4_ARCHIVE_NAME"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --compare-build-evidence \
  "$STAGE4_RELEASE_OUTPUT_A/release-build-evidence.json" \
  "$STAGE4_RELEASE_OUTPUT_B/release-build-evidence.json" \
  --output "$STAGE4_RELEASE_RUN_PARENT/release-reproducibility.json"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --publish-release-bundle \
  --source-archive "$STAGE4_RELEASE_OUTPUT_A/$STAGE4_ARCHIVE_NAME" \
  --source-archive-sha256 \
    "$STAGE4_RELEASE_OUTPUT_A/$STAGE4_ARCHIVE_NAME.sha256" \
  --source-build-evidence \
    "$STAGE4_RELEASE_OUTPUT_A/release-build-evidence.json" \
  --transaction-dir "$STAGE4_FINAL_OUTPUT_DIR"
test -f "$STAGE4_FINAL_OUTPUT_DIR/$STAGE4_ARCHIVE_NAME"
test -f "$STAGE4_FINAL_OUTPUT_DIR/$STAGE4_ARCHIVE_NAME.sha256"
test -f "$STAGE4_FINAL_OUTPUT_DIR/release-build-evidence.json"
STAGE4_RELEASE_CANDIDATE_CONTEXT_DIR="$STAGE4_RELEASE_RUN_PARENT/candidate-context"
test ! -e "$STAGE4_RELEASE_CANDIDATE_CONTEXT_DIR"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --candidate-archive "$STAGE4_FINAL_OUTPUT_DIR/$STAGE4_ARCHIVE_NAME" \
  --candidate-sha256 "$STAGE4_FINAL_OUTPUT_DIR/$STAGE4_ARCHIVE_NAME.sha256" \
  --candidate-build-evidence \
    "$STAGE4_FINAL_OUTPUT_DIR/release-build-evidence.json" \
  --published-release-dir "$STAGE4_FINAL_OUTPUT_DIR" \
  --candidate-clean-head "$(git rev-parse HEAD)" \
  --transaction-dir "$STAGE4_RELEASE_CANDIDATE_CONTEXT_DIR" \
  --output "$STAGE4_RELEASE_CANDIDATE_CONTEXT_DIR/release-candidate-context.json" \
  --write-candidate-context \
    "$STAGE4_RELEASE_CANDIDATE_CONTEXT_DIR/release-candidate-context.env"
STAGE4_RELEASE_CANDIDATE_CONTEXT_FILE="\
$STAGE4_RELEASE_CANDIDATE_CONTEXT_DIR/release-candidate-context.env"
test -f "$STAGE4_RELEASE_CANDIDATE_CONTEXT_FILE"
STAGE4_LIFECYCLE_WORK="$STAGE4_RELEASE_RUN_PARENT/lifecycle-work"
STAGE4_LIFECYCLE_OUTPUT="$STAGE4_RELEASE_RUN_PARENT/lifecycle-output"
STAGE4_LIFECYCLE_BUNDLE_DIR="$STAGE4_RELEASE_RUN_PARENT/lifecycle-probe"
test ! -e "$STAGE4_LIFECYCLE_BUNDLE_DIR"
install -d "$STAGE4_LIFECYCLE_WORK" "$STAGE4_LIFECYCLE_OUTPUT"
bash packaging/run_network_isolated.sh \
  bash packaging/build_release.sh \
    --final-archive \
    --work-root "$STAGE4_LIFECYCLE_WORK" \
    --output-dir "$STAGE4_LIFECYCLE_OUTPUT" \
    --release-version "$STAGE4_LIFECYCLE_PROBE_VERSION" \
    --artifact-purpose lifecycle_probe \
    --micromamba "$STAGE4_MICROMAMBA" \
    --python-package-cache "$STAGE4_PYTHON_PACKAGE_CACHE" \
    --python-wheel-cache "$STAGE4_PYTHON_WHEEL_CACHE" \
    --source-archive-cache "$STAGE4_SOURCE_ARCHIVE_CACHE"
STAGE4_LIFECYCLE_ARCHIVE_NAME="slope-sim-stage4-${STAGE4_LIFECYCLE_PROBE_VERSION}-ubuntu24.04-amd64.tar.zst"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-lifecycle-probe-archive \
    "$STAGE4_LIFECYCLE_OUTPUT/$STAGE4_LIFECYCLE_ARCHIVE_NAME" \
  --archive-sha256 \
    "$STAGE4_LIFECYCLE_OUTPUT/$STAGE4_LIFECYCLE_ARCHIVE_NAME.sha256" \
  --build-evidence "$STAGE4_LIFECYCLE_OUTPUT/release-build-evidence.json" \
  --primary-candidate-context \
    "$STAGE4_RELEASE_CANDIDATE_CONTEXT_FILE" \
  --output "$STAGE4_RELEASE_RUN_PARENT/lifecycle-probe-verification.json"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --create-lifecycle-probe-handoff \
  --primary-candidate-context \
    "$STAGE4_RELEASE_CANDIDATE_CONTEXT_FILE" \
  --probe-archive \
    "$STAGE4_LIFECYCLE_OUTPUT/$STAGE4_LIFECYCLE_ARCHIVE_NAME" \
  --archive-sha256 \
    "$STAGE4_LIFECYCLE_OUTPUT/$STAGE4_LIFECYCLE_ARCHIVE_NAME.sha256" \
  --build-evidence "$STAGE4_LIFECYCLE_OUTPUT/release-build-evidence.json" \
  --probe-verification \
    "$STAGE4_RELEASE_RUN_PARENT/lifecycle-probe-verification.json" \
  --probe-version "$STAGE4_LIFECYCLE_PROBE_VERSION" \
  --transaction-dir "$STAGE4_LIFECYCLE_BUNDLE_DIR" \
  --output \
    "$STAGE4_LIFECYCLE_BUNDLE_DIR/lifecycle-probe-handoff.json"
STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE="$STAGE4_LIFECYCLE_BUNDLE_DIR/lifecycle-probe-handoff.json"
test -f "$STAGE4_LIFECYCLE_BUNDLE_DIR/$STAGE4_LIFECYCLE_ARCHIVE_NAME"
test -f "$STAGE4_LIFECYCLE_BUNDLE_DIR/$STAGE4_LIFECYCLE_ARCHIVE_NAME.sha256"
test -f "$STAGE4_LIFECYCLE_BUNDLE_DIR/release-build-evidence.json"
test -f "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE"
export STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE
```

外层 archive 包含 Task 4 安装/卸载器、packed payload、Task 5 中文文档、`release-manifest.json`、内部 payload `SHA256SUMS`、唯一 `share/slope-sim/sbom.spdx.json` 和第三方许可证。候选构建以当前 Git commit timestamp 固定并记录 `functional_source_epoch`；Task 9 的正式重建通过已验证 accepted-candidate context 继承同一 epoch，不能改用最终 evidence commit timestamp。tar 固定 sort/mtime/uid/gid，Zstd 固定 level、单线程和 frame 参数。archive 的 SHA 不得自引用写回其内部 manifest；两个输出目录分别生成外部 `<archive>.sha256` 与 `release-build-evidence.json`，后者记录精确 archive 文件名/hash、builder image digest、Git/lock/Python package+wheel cache/toolchain/source archive manifest/tree hash、规范 C++ dependency/validation/Python runtime tree digest、本轮独立 source-build、Python cache/wheel/env、C++ source/build/install、ROS source 副本/解包树和 work root 证据，以及 `source_provenance.kind=clean_git_commit`、`clean=true`、commit/tree id、source snapshot SHA-256、`functional_source_epoch`、等值 source-build pre-build SHA-256、snapshot post-build SHA-256 和 release-tree link census。`--compare-dependency-trees` 与 `--compare-runtime-trees` 先证明两套私有 C++ 安装闭包以及清理、规范化后的两套 Python tree 逐成员相同；不要求中间 `python-runtime.tar` byte-identical。`--compare-build-evidence` 要求除本来就应不同的外部 work/output/source-build/source-work 路径外，archive hash、builder/Git/lock/package+wheel cache/toolchain/source-archive/descriptor/ABI/source snapshot/source-build pre-build/dependency/runtime tree、materialized source bytes、no-participant eCAL DSO 隔离与零 entity census、零链接 census 和文件清单全部相同，并证明这些绝对路径没有泄漏进 archive。只有 repo post-A clean、dependency/runtime tree、archive `cmp` 与 evidence invariant 比较都通过，才把 A 的三个精确文件复制进同文件系统 staging directory，并用一次 directory rename 原子发布；禁止 glob 猜来源或逐文件暴露半发布状态。最后的 candidate context 只绑定这三个已发布文件的绝对路径/hash、release version、clean HEAD、`functional_source_epoch` 和上述比较 evidence，不把“candidate 已定位”冒充“archive 已完成 Step 15 验证”；后续 shell 必须先验证/source 该 context，不能消费 Step 14 的临时变量。

第三个 lifecycle work/output 也必须从同一 clean HEAD 和 canonical 输入完整重建，不能复制或改写 A/B archive。`--create-lifecycle-probe-handoff` 显式读取 probe archive/sidecar/build evidence 与 verifier 结果，在受锁 sibling staging 内复制前三者并生成 handoff，随后按同一目录事务提交四文件 bundle；handoff 的 schema 固定 `purpose=lifecycle_probe`、`publishable=false`，并以规范 sibling basename/hash 解析前三者，同时固定主 candidate archive/source identity。probe bundle 与 primary `published/` 是两个 sibling，probe 绝不复制进 primary 发布目录，也不参与 A/B `cmp`、candidate/final payload equivalence 或 final-status 发布文件集合。

- [ ] **Step 15: 验证唯一验收候选并冻结 handoff**

调用者把 `STAGE4_RELEASE_CANDIDATE_CONTEXT_FILE` 指向 Step 14 明确生成的 `release-candidate-context.env`，然后独立执行：

```bash
test -n "${STAGE4_RELEASE_CANDIDATE_CONTEXT_FILE:-}"
test -n "${STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE:-}"
STAGE4_RELEASE_CANDIDATE_CONTEXT_DIR="$(dirname \
  "$STAGE4_RELEASE_CANDIDATE_CONTEXT_FILE")"
STAGE4_RELEASE_CONTEXT_ROOT="$(dirname \
  "$STAGE4_RELEASE_CANDIDATE_CONTEXT_DIR")"
STAGE4_RELEASE_HANDOFF_TRANSACTION_DIR="$STAGE4_RELEASE_CONTEXT_ROOT/release-handoff"
test ! -e "$STAGE4_RELEASE_HANDOFF_TRANSACTION_DIR"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-candidate-context "$STAGE4_RELEASE_CANDIDATE_CONTEXT_FILE" \
  --output "$STAGE4_RELEASE_CONTEXT_ROOT/candidate-context-preflight.json"
source "$STAGE4_RELEASE_CANDIDATE_CONTEXT_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --archive "$STAGE4_RELEASE_ARCHIVE" \
  --archive-sha256 "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE" \
  --build-evidence "$STAGE4_RELEASE_BUILD_EVIDENCE" \
  --transaction-dir "$STAGE4_RELEASE_HANDOFF_TRANSACTION_DIR" \
  --output "$STAGE4_RELEASE_HANDOFF_TRANSACTION_DIR/release-verification.json" \
  --write-handoff "$STAGE4_RELEASE_HANDOFF_TRANSACTION_DIR/release-handoff.env"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-lifecycle-probe-handoff \
    "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE" \
  --primary-candidate-context "$STAGE4_RELEASE_CANDIDATE_CONTEXT_FILE" \
  --output "$STAGE4_RELEASE_CONTEXT_ROOT/lifecycle-probe-preflight.json"
```

Expected: rc=0，A/B 的 work/output/source-build 全在仓库外；A 完成后 repo 仍 clean，B 独立 clean gate 通过。两个空根从同一 clean Git source snapshot、同一 `functional_source_epoch`、同一只读 canonical Python package/wheel cache 和同一只读 canonical C++/ROS source archive cache，各自创建摘要相同的可写 source-build、独立 package/wheel cache/env、C++ source/build/install/validation、ROS 源码副本和安全解包树，产生相同的 dependency/runtime tree、零链接 release tree 及 byte-identical archive；source provenance、snapshot immutability、Python lock/package-cache/wheel-cache/toolchain、no-participant eCAL DSO 进程隔离与零 entity census、source archive/member/materialization、真实 Conda link census、安装器、教程、packed Python、完整 C++ closure、模型 YAML、完整 self-test session、manifest/checksum/ELF/resource/禁入项全部通过。verifier 只在全部通过后以 shell-safe serializer 原子生成 candidate handoff，固定 archive/sidecar/build-evidence/release-verification 的绝对路径与 SHA-256、version、candidate clean HEAD 和 `functional_source_epoch`；独立 lifecycle preflight 还必须从 handoff 所在目录重算 probe 三件套，确认 `purpose=lifecycle_probe`、`publishable=false`、版本不同且主 candidate identity 相同。Task 7/8 必须先复核再消费各自 handoff，不能依赖上一 shell 的临时 export。Step 15 后不得再重构或修改任何归档输入；审查若要求代码、配置、packaging 或随包教程变化，现有候选及其真实验收立即作废并返回对应 RED/GREEN、Steps 7-12、Step 13 和双根重建。只有 README、外部交付报告与仓库外 evidence 可以继续写入，并在 Task 9 通过固定路径与受限 provenance 派生闭包等价门。

## Task 7：获授权真实 eCAL+C++ 联合负载

**Files:**
- Modify: `docs/阶段四交付报告.md`

**TDD 裁决：** 本 Task 只对 Task 6 已按 RED/GREEN/REFACTOR 冻结的候选包执行获授权真实联合负载，不新增生产行为，也不伪造新的 RED。任一安装、handoff、eCAL/C++、Dashboard、ROS/RViz2 或性能行为失败，都必须回到拥有该行为的 Task 6 或 C/D 子计划先写最小 RED、观察正确失败、完成 GREEN 与 REFACTOR；任何生产或教程修改都会使当前候选和本 Task 已有 evidence 全部失效，必须重新完成 Task 6 的 clean gate、双根可复现构建和候选发布后，才能从本 Task Step 1 使用全新验收根重跑。

- [ ] **Step 1: 为 `4+2` invocation 单独取得授权并预检静默主机**

先复核并 source Task 6 Step 15 的唯一 release handoff，再安全解包、安装到本轮仓库外专用 prefix；不能依赖 Step 14 所在 shell 的临时变量：

```bash
test -n "${STAGE4_RELEASE_HANDOFF_FILE:-}"
test -n "${STAGE4_EXTERNAL_ACCEPTANCE_PARENT:-}"
test -d "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT"
STAGE4_ACCEPTANCE_INSTALL_RUN="$(mktemp -d \
  "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT/slope-sim-stage4-install.XXXXXX")"
STAGE4_ACCEPTANCE_EVIDENCE_DIR="$STAGE4_ACCEPTANCE_INSTALL_RUN/evidence"
install -d -m 0700 "$STAGE4_ACCEPTANCE_EVIDENCE_DIR"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --output "$STAGE4_ACCEPTANCE_INSTALL_RUN/release-handoff-preflight.json"
source "$STAGE4_RELEASE_HANDOFF_FILE"
test -f "$STAGE4_RELEASE_ARCHIVE"
test -f "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE"
test -f "$STAGE4_RELEASE_BUILD_EVIDENCE"
(
  cd -- "$(dirname -- "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE")"
  sha256sum -c -- "$(basename -- "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE")"
)
STAGE4_ACCEPTANCE_EXTRACT_ROOT="$STAGE4_ACCEPTANCE_INSTALL_RUN/unpacked"
STAGE4_ACCEPTANCE_INSTALL_PREFIX="$STAGE4_ACCEPTANCE_INSTALL_RUN/prefix"
install -d -m 0700 "$STAGE4_ACCEPTANCE_EXTRACT_ROOT"
test ! -e "$STAGE4_ACCEPTANCE_INSTALL_PREFIX"
tar --extract --zstd --file "$STAGE4_RELEASE_ARCHIVE" \
  --directory "$STAGE4_ACCEPTANCE_EXTRACT_ROOT" \
  --no-same-owner --no-same-permissions --numeric-owner
STAGE4_ACCEPTANCE_SOURCE_ROOT="$STAGE4_ACCEPTANCE_EXTRACT_ROOT/\
slope-sim-stage4-$STAGE4_RELEASE_VERSION"
STAGE4_RELEASE_ARCHIVE_SHA256="$(sha256sum "$STAGE4_RELEASE_ARCHIVE" | cut -d ' ' -f 1)"
STAGE4_RELEASE_BUILD_EVIDENCE_SHA256="$(sha256sum "$STAGE4_RELEASE_BUILD_EVIDENCE" | cut -d ' ' -f 1)"
bash "$STAGE4_ACCEPTANCE_SOURCE_ROOT/install.sh" --offline \
  --prefix "$STAGE4_ACCEPTANCE_INSTALL_PREFIX" \
  --source-root "$STAGE4_ACCEPTANCE_SOURCE_ROOT" \
  --archive-file "$STAGE4_RELEASE_ARCHIVE" \
  --archive-sha256-file "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE" \
  --archive-sha256 "$STAGE4_RELEASE_ARCHIVE_SHA256" \
  --build-evidence "$STAGE4_RELEASE_BUILD_EVIDENCE" \
  --build-evidence-sha256 "$STAGE4_RELEASE_BUILD_EVIDENCE_SHA256"
STAGE4_ACCEPTANCE_RELEASE_ROOT="$(readlink -f \
  "$STAGE4_ACCEPTANCE_INSTALL_PREFIX/current")"
test -f "$STAGE4_ACCEPTANCE_RELEASE_ROOT/install-state.json"
test -f "$STAGE4_ACCEPTANCE_RELEASE_ROOT/relocation-state.json"
SLOPE_SIM_ROOT="$STAGE4_ACCEPTANCE_RELEASE_ROOT" \
  "$STAGE4_ACCEPTANCE_RELEASE_ROOT/bin/slope-sim" doctor \
  --json "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/candidate-doctor.json"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --run-installed-no-participant-smoke \
  --release-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --installed-release-root "$STAGE4_ACCEPTANCE_RELEASE_ROOT" \
  --install-state "$STAGE4_ACCEPTANCE_RELEASE_ROOT/install-state.json" \
  --relocation-marker \
    "$STAGE4_ACCEPTANCE_RELEASE_ROOT/relocation-state.json" \
  --doctor-evidence \
    "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/candidate-doctor.json" \
  --output \
    "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/candidate-no-participant-smoke.json"
STAGE4_ACCEPTANCE_TRANSACTION_DIR="$STAGE4_ACCEPTANCE_INSTALL_RUN/acceptance-install"
test ! -e "$STAGE4_ACCEPTANCE_TRANSACTION_DIR"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --freeze-acceptance-install \
  --release-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --installed-release-root "$STAGE4_ACCEPTANCE_RELEASE_ROOT" \
  --install-prefix "$STAGE4_ACCEPTANCE_INSTALL_PREFIX" \
  --install-state "$STAGE4_ACCEPTANCE_RELEASE_ROOT/install-state.json" \
  --relocation-marker \
    "$STAGE4_ACCEPTANCE_RELEASE_ROOT/relocation-state.json" \
  --acceptance-evidence-dir "$STAGE4_ACCEPTANCE_EVIDENCE_DIR" \
  --doctor-evidence \
    "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/candidate-doctor.json" \
  --smoke-evidence \
    "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/candidate-no-participant-smoke.json" \
  --transaction-dir "$STAGE4_ACCEPTANCE_TRANSACTION_DIR" \
  --output "$STAGE4_ACCEPTANCE_TRANSACTION_DIR/acceptance-install.json" \
  --write-acceptance-handoff \
    "$STAGE4_ACCEPTANCE_TRANSACTION_DIR/acceptance-install-handoff.env"
```

Expected: 先生成与 Task 8 Step 2 同 schema 的 candidate no-participant smoke；它实际完成 C++ ELF/Python import/SDK/self-test MCAP 回读及 PCD/PLY/LVX2 导出，且不创建真实 eCAL participant。`install-state.json` 的 `verified_archive` provenance、外部 archive/sidecar/build evidence/release handoff、clean HEAD、`relocation-state.json`、fresh doctor 和 smoke 全部匹配后，verifier 才原子生成 shell-safe acceptance handoff。handoff 以唯一 `acceptance_run_id` 固定候选五件套：resolved installed root 的绝对路径和规范安装树 identity digest，以及 `install-state.json`、`relocation-state.json`、candidate doctor JSON、candidate smoke JSON 各自的绝对路径与 SHA-256；同时绑定 install prefix、release handoff hash，并显式导出本轮仓库外、已规范化且无链接的 `STAGE4_ACCEPTANCE_EVIDENCE_DIR`。它固定导出 `STAGE4_ACCEPTANCE_RELEASE_ROOT`、`STAGE4_ACCEPTANCE_INSTALL_STATE`、`STAGE4_ACCEPTANCE_RELOCATION_MARKER`、`STAGE4_ACCEPTANCE_DOCTOR_EVIDENCE` 和 `STAGE4_ACCEPTANCE_SMOKE_EVIDENCE`，不得靠后续 shell 重建路径。调用者把 `STAGE4_ACCEPTANCE_HANDOFF_FILE` 指向这个明确文件；后续每个真实 Run 都独立复核并 source，复核时重算五件套摘要、确认其 run id 相同、evidence dir 仍属于同一 acceptance run、没有被替换且目标输出不存在，不依赖本 Step shell。缺失、篡改、换 root 或跨 acceptance run 混用任一项都必须失败。不得把 Task 3 builder root、build-smoke state 或未安装的 packed payload 作为验收源。完成只读安装预检后再向用户申请真实运行授权；本计划文档不是授权。说明将启动真实 eCAL/PyBullet/C++/Recorder 的 `4+2` 车型、时长和失败不重跑规则；得到明确回复后即时扫描 pytest、GUI、Xvfb、PyBullet、eCAL participant 和系统负载。发现并发负载则等待并重新确认，不并跑；授权只覆盖下一条命令。

- [ ] **Step 2: 运行唯一一次 `4+2`**

```bash
test -n "${STAGE4_ACCEPTANCE_HANDOFF_FILE:-}"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-acceptance-handoff "$STAGE4_ACCEPTANCE_HANDOFF_FILE" \
  --output "$STAGE4_ACCEPTANCE_HANDOFF_FILE.step2-preflight.json"
source "$STAGE4_ACCEPTANCE_HANDOFF_FILE"
env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD conda run -n slope-sim python scripts/verify_stage4_ecal_cpp.py --release-root "$STAGE4_ACCEPTANCE_RELEASE_ROOT" --runtime-mode headless --robot-model active_steering_4wd --terrain-model golf_heightfield --obstacle-count 20 --warmup-sec 1 --duration-sec 5 --output "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/ecal-headless-4plus2.json"
```

Expected: rc=0；结果证明实际 `headless/DIRECT/dashboard_enabled=false`，实际场景为 `golf_heightfield`、障碍物精确 20、LiDAR 为 `realtime_mid360` 且每帧候选射线精确 5,760；sim/wall `0.98..1.02`，command/wheel 墙钟 `95..105Hz`、timestamp `99..101Hz`、最大 gap `<=30ms`，三传感器墙钟 `9..11Hz`、timestamp `9.9..10.1Hz`、最大 gap `<=250ms`，三方 raw payload 双向完全相等，所有 drop/error/pending 为 0，运动和转向阈值通过。失败则保留证据并停止。

- [ ] **Step 3: 为 `2+0` invocation 重新取得授权并重新扫描**

只有 Step 2 通过才申请新的明确授权，不得复用 Step 1 的授权。再次说明车型、时长和失败不重跑规则，并完成即时静默扫描；本授权只覆盖紧随其后的 Step 4 命令。

- [ ] **Step 4: 运行唯一一次 `2+0`**

```bash
test -n "${STAGE4_ACCEPTANCE_HANDOFF_FILE:-}"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-acceptance-handoff "$STAGE4_ACCEPTANCE_HANDOFF_FILE" \
  --output "$STAGE4_ACCEPTANCE_HANDOFF_FILE.step4-preflight.json"
source "$STAGE4_ACCEPTANCE_HANDOFF_FILE"
env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD conda run -n slope-sim python scripts/verify_stage4_ecal_cpp.py --release-root "$STAGE4_ACCEPTANCE_RELEASE_ROOT" --runtime-mode headless --robot-model df_back --terrain-model golf_heightfield --obstacle-count 20 --warmup-sec 1 --duration-sec 5 --output "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/ecal-headless-2plus0.json"
```

Expected: rc=0 且同一完整 oracle 通过，包括实际 20 个障碍物与每帧 5,760 候选射线；两条证据都存在后才可报告核心联合负载 PASS。

若任一条失败，保留唯一原始证据并停止，不自动重跑、不降低门槛；先做只读根因分析，再由用户授权下一次真实运行。两条 headless 结果不能替代下面的 13.4 interactive 联合性能。

- [ ] **Step 5: 为 interactive + Dashboard + ROS off 单独取得授权**

只有两个 headless 车型均通过才申请。向用户明确说明本次会在真实桌面启动 PyBullet GUI、Dashboard、真实 eCAL、C++ Subscriber/Command/Recorder，ROS/RViz2 关闭，车型 `active_steering_4wd`、20 个障碍物、5 秒正式窗口；授权只覆盖 Step 6。随后重新扫描全机 GUI/Xvfb/PyBullet/eCAL/pytest 和负载，发现竞争则等待并重新确认。

- [ ] **Step 6: 运行唯一一次 interactive ROS off 联合门禁**

```bash
test -n "${STAGE4_ACCEPTANCE_HANDOFF_FILE:-}"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-acceptance-handoff "$STAGE4_ACCEPTANCE_HANDOFF_FILE" \
  --output "$STAGE4_ACCEPTANCE_HANDOFF_FILE.step6-preflight.json"
source "$STAGE4_ACCEPTANCE_HANDOFF_FILE"
DISPLAY=:1 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
  env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD \
  conda run -n slope-sim python scripts/verify_stage4_ecal_cpp.py \
  --release-root "$STAGE4_ACCEPTANCE_RELEASE_ROOT" \
  --runtime-mode interactive --with-dashboard --ros-mode off \
  --robot-model active_steering_4wd --terrain-model golf_heightfield \
  --obstacle-count 20 --warmup-sec 1 --duration-sec 5 \
  --output "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/ecal-interactive-ros-off.json"
```

Expected: rc=0；实际状态为 `interactive/GUI/dashboard_enabled=true/ros_mode=off`，Bridge/RViz2 未启动，Dashboard draw 样本不少于 5、p95 `<100ms`、GUI event gap `<=100ms`；实际 20 个障碍物、5,760 候选射线、sim/wall、100/10Hz、三方 raw、运动、零 drop/error/pending 和 final manifest 同样全部通过。失败保留证据并停止。

- [ ] **Step 7: 为完全相同 workload 的 ROS/RViz2 on 重新授权**

只有 Step 6 通过才申请新的单条授权并重新扫描静默主机；明确说明将额外启动 ROS 2 Bridge 与 RViz2，其他车型、场地、障碍物、射线和窗口保持不变。前一次授权不能复用。

- [ ] **Step 8: 运行唯一一次 interactive ROS on 联合门禁**

```bash
test -n "${STAGE4_ACCEPTANCE_HANDOFF_FILE:-}"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-acceptance-handoff "$STAGE4_ACCEPTANCE_HANDOFF_FILE" \
  --output "$STAGE4_ACCEPTANCE_HANDOFF_FILE.step8-preflight.json"
source "$STAGE4_ACCEPTANCE_HANDOFF_FILE"
DISPLAY=:1 XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}" \
  env -u STAGE4_ECAL_TEST_SHIM -u LD_PRELOAD \
  conda run -n slope-sim python scripts/verify_stage4_ecal_cpp.py \
  --release-root "$STAGE4_ACCEPTANCE_RELEASE_ROOT" \
  --runtime-mode interactive --with-dashboard --ros-mode on \
  --robot-model active_steering_4wd --terrain-model golf_heightfield \
  --obstacle-count 20 --warmup-sec 1 --duration-sec 5 \
  --output "$STAGE4_ACCEPTANCE_EVIDENCE_DIR/ecal-interactive-ros-on.json"
```

Expected: rc=0；Bridge 与 RViz2 均实际 READY，除此之外完整复用 Step 6 的 Dashboard/GUI、负载、频率、完整性和性能硬门。ROS off/on 是两份独立证据，报告必须区分核心 headless、interactive ROS off 和 interactive ROS on；任一失败都停止且不自动重跑。

## Task 8：干净 Ubuntu 24.04 迁移验收

**Files:**
- Consume read-only: `docs/阶段四部署教程.md`
- Modify: `docs/阶段四交付报告.md`

**TDD 裁决：** 本 Task 只在已完成实现的全新 Ubuntu 24.04 目标机上按候选包内已经冻结的部署教程执行迁移验收，不新增生产行为，因此不伪造新的 RED/GREEN，也不在验收后原地改教程。任何行为失败都必须回到拥有该行为的 Task 写 RED、完成 GREEN 和 REFACTOR；任何教程命令、顺序或文字缺陷都返回 Task 5 修改并重过文档 GREEN。两类修改都会使当前候选失效，必须回到 Task 6 Steps 7-12 复验、Step 13 重新获 commit 授权并执行 Steps 14-15，再从本 Task Step 1 使用全新安装根执行；本 Task 只向仓库外 evidence 和 `docs/阶段四交付报告.md` 写安装、真实运行与生命周期证据。

- [ ] **Step 1: 在目标机验证归档与离线安装**

复制任何 artifact 前，先在验收控制机从已验证 handoff 取得精确路径，并经预先固定的 SSH host key 在目标机独占创建本轮 0700 transfer root。`STAGE4_CLEAN_HOST_KNOWN_HOSTS` 不能由本轮 `ssh-keyscan` 生成；以下捕获的绝对路径必须保留到 Step 5，换 shell 时从本步明确输出重新设置，不能创建或搜索另一个目录：

```bash
test -n "${STAGE4_RELEASE_HANDOFF_FILE:-}"
test -n "${STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE:-}"
test -n "${STAGE4_CLEAN_HOST_TARGET:-}"
test -n "${STAGE4_CLEAN_HOST_KNOWN_HOSTS:-}"
test -n "${STAGE4_EXTERNAL_ACCEPTANCE_PARENT:-}"
test -f "$STAGE4_CLEAN_HOST_KNOWN_HOSTS"
test -d "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT"
STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_ROOT="$(mktemp -d \
  "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT/slope-sim-stage4-transfer-context.XXXXXX")"
STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_DIR="\
$STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_ROOT/clean-host-transfer-context"
test ! -e "$STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_DIR"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --output "$STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_ROOT/release-preflight.json"
source "$STAGE4_RELEASE_HANDOFF_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-lifecycle-probe-handoff \
    "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE" \
  --primary-release-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --output "$STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_ROOT/probe-preflight.json"
lifecycle_bundle_dir="$(dirname "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE")"
test "$(basename "$lifecycle_bundle_dir")" = lifecycle-probe
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --create-clean-host-transfer-context \
  --release-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --lifecycle-probe-handoff "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE" \
  --transaction-dir "$STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_DIR" \
  --output "$STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_DIR/transfer-context.json" \
  --write-clean-host-transfer-context \
    "$STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_DIR/transfer-context.env"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-clean-host-transfer-context \
    "$STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_DIR/transfer-context.env" \
  --output "$STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_ROOT/context-preflight.json"
STAGE4_CLEAN_HOST_REMOTE_ROOT="$(
  ssh -o BatchMode=yes -o StrictHostKeyChecking=yes \
    -o UserKnownHostsFile="$STAGE4_CLEAN_HOST_KNOWN_HOSTS" \
    "$STAGE4_CLEAN_HOST_TARGET" \
    'umask 077; mktemp -d "$HOME/slope-sim-stage4-transfer.XXXXXX"'
)"
case "$STAGE4_CLEAN_HOST_REMOTE_ROOT" in
  /*) ;;
  *) exit 1 ;;
esac
scp -p -o BatchMode=yes -o StrictHostKeyChecking=yes \
  -o UserKnownHostsFile="$STAGE4_CLEAN_HOST_KNOWN_HOSTS" \
  "$STAGE4_RELEASE_ARCHIVE" \
  "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE" \
  "$STAGE4_RELEASE_BUILD_EVIDENCE" \
  "$STAGE4_CLEAN_HOST_TARGET:$STAGE4_CLEAN_HOST_REMOTE_ROOT/"
scp -p -r -o BatchMode=yes -o StrictHostKeyChecking=yes \
  -o UserKnownHostsFile="$STAGE4_CLEAN_HOST_KNOWN_HOSTS" \
  "$STAGE4_CLEAN_HOST_TRANSFER_CONTEXT_DIR" \
  "$lifecycle_bundle_dir" \
  "$STAGE4_CLEAN_HOST_TARGET:$STAGE4_CLEAN_HOST_REMOTE_ROOT/"
printf 'STAGE4_CLEAN_HOST_REMOTE_ROOT=%s\n' \
  "$STAGE4_CLEAN_HOST_REMOTE_ROOT"
export STAGE4_CLEAN_HOST_REMOTE_ROOT
```

然后在目标机把 `STAGE4_CLEAN_HOST_REMOTE_ROOT` 设置为控制机刚输出的精确值，`cd` 到该目录，并执行：

```bash
umask 077
set -o pipefail
test -n "${STAGE4_CLEAN_HOST_REMOTE_ROOT:-}"
test "$PWD" = "$STAGE4_CLEAN_HOST_REMOTE_ROOT"
test "$(stat -c '%a' "$STAGE4_CLEAN_HOST_REMOTE_ROOT")" = 700
transfer_context_dir="$PWD/clean-host-transfer-context"
test -d "$transfer_context_dir"
(
  cd -- "$transfer_context_dir"
  sha256sum -c -- SHA256SUMS
)
source "$transfer_context_dir/transfer-context.env"
test "$(basename -- "$STAGE4_TRANSFER_ARCHIVE_BASENAME")" = \
  "$STAGE4_TRANSFER_ARCHIVE_BASENAME"
test "$(basename -- "$STAGE4_TRANSFER_ARCHIVE_SHA256_BASENAME")" = \
  "$STAGE4_TRANSFER_ARCHIVE_SHA256_BASENAME"
test "$(basename -- "$STAGE4_TRANSFER_BUILD_EVIDENCE_BASENAME")" = \
  "$STAGE4_TRANSFER_BUILD_EVIDENCE_BASENAME"
test "$(basename -- "$STAGE4_TRANSFER_PAYLOAD_ROOT_BASENAME")" = \
  "$STAGE4_TRANSFER_PAYLOAD_ROOT_BASENAME"
test "$(basename -- "$STAGE4_TRANSFER_LIFECYCLE_BUNDLE_BASENAME")" = \
  "$STAGE4_TRANSFER_LIFECYCLE_BUNDLE_BASENAME"
test "$(basename -- "$STAGE4_TRANSFER_LIFECYCLE_HANDOFF_BASENAME")" = \
  "$STAGE4_TRANSFER_LIFECYCLE_HANDOFF_BASENAME"
test "$STAGE4_TRANSFER_BUILD_EVIDENCE_BASENAME" = \
  release-build-evidence.json
test "$STAGE4_TRANSFER_LIFECYCLE_BUNDLE_BASENAME" = lifecycle-probe
test "$STAGE4_TRANSFER_LIFECYCLE_HANDOFF_BASENAME" = \
  lifecycle-probe-handoff.json
test "$STAGE4_TRANSFER_ARCHIVE_BASENAME" = \
  "slope-sim-stage4-${STAGE4_TRANSFER_RELEASE_VERSION}-ubuntu24.04-amd64.tar.zst"
test "$STAGE4_TRANSFER_ARCHIVE_SHA256_BASENAME" = \
  "${STAGE4_TRANSFER_ARCHIVE_BASENAME}.sha256"
test "$STAGE4_TRANSFER_PAYLOAD_ROOT_BASENAME" = \
  "slope-sim-stage4-${STAGE4_TRANSFER_RELEASE_VERSION}"
archive="$PWD/$STAGE4_TRANSFER_ARCHIVE_BASENAME"
archive_sidecar="$PWD/$STAGE4_TRANSFER_ARCHIVE_SHA256_BASENAME"
build_evidence="$PWD/$STAGE4_TRANSFER_BUILD_EVIDENCE_BASENAME"
target_lifecycle_probe_handoff="\
$PWD/$STAGE4_TRANSFER_LIFECYCLE_BUNDLE_BASENAME/\
$STAGE4_TRANSFER_LIFECYCLE_HANDOFF_BASENAME"
test -f "$target_lifecycle_probe_handoff"
(
  cd -- "$(dirname -- "$archive_sidecar")"
  sha256sum -c -- "$(basename -- "$archive_sidecar")"
)
archive_sha256="$(sha256sum "$archive" | cut -d ' ' -f 1)"
build_evidence_sha256="$(sha256sum "$build_evidence" | cut -d ' ' -f 1)"
extract_dir="$PWD/${STAGE4_TRANSFER_PAYLOAD_ROOT_BASENAME}-unpacked"
test ! -e "$extract_dir"
install -d -m 0700 "$extract_dir"
tar --extract --zstd --file "$archive" --directory "$extract_dir" \
  --no-same-owner --no-same-permissions --numeric-owner
source_root="$extract_dir/$STAGE4_TRANSFER_PAYLOAD_ROOT_BASENAME"
install -d -m 0700 "$HOME/slope-sim-data"
STAGE4_CLEAN_HOST_INSTALL_LOG="$HOME/slope-sim-data/install.log"
test ! -e "$STAGE4_CLEAN_HOST_INSTALL_LOG"
sudo "$source_root/install.sh" --offline \
  --source-root "$source_root" \
  --archive-file "$archive" \
  --archive-sha256-file "$archive_sidecar" \
  --archive-sha256 "$archive_sha256" \
  --build-evidence "$build_evidence" \
  --build-evidence-sha256 "$build_evidence_sha256" \
  2>&1 | tee "$STAGE4_CLEAN_HOST_INSTALL_LOG"
/opt/slope-sim/current/bin/slope-sim doctor \
  --json "$HOME/slope-sim-data/doctor.json"
STAGE4_TARGET_LIFECYCLE_PROBE_HANDOFF="$target_lifecycle_probe_handoff"
export STAGE4_CLEAN_HOST_INSTALL_LOG
export STAGE4_TARGET_LIFECYCLE_PROBE_HANDOFF
```

`release-build-evidence.json` 必须先证明该精确 archive 已通过 Task 6 的构建端成员 verifier，外部 SHA 文件中的 basename 必须与 `archive` 完全一致。主 candidate 三件套与整个 `lifecycle-probe/` bundle 必须位于本轮专用 0700 transfer root；不能只复制 portable handoff 而漏掉其 sibling archive/sidecar/build evidence。教程只允许解到此前不存在的绝对目录，禁止 `sudo tar`、覆盖式解包、`--same-owner` 或信任 archive mode；唯一顶层目录名固定为 `slope-sim-stage4-<version>`。`install.sh` 随后按 Task 4 重算 archive/evidence SHA、交叉核对 sidecar/evidence 字段，对已解 `source_root` 做 `lstat`、link count、清单精确匹配和复制前后 TOCTOU 核验，并把正式来源摘要写入 `install-state.json`。安装器读取 archive 只为哈希，不枚举或再次解包。Expected: 无网络也能安装核心；doctor 不引用 `.git`、开发 Conda、仓库和 reference；可选 ROS 缺失时只明确标为 unavailable，不破坏核心。

- [ ] **Step 2: 先跑不创建真实 eCAL participant 的安装 smoke**

```bash
installed_root="$(readlink -f /opt/slope-sim/current)"
install_state="$installed_root/install-state.json"
relocation_marker="$installed_root/relocation-state.json"
smoke_parent="$HOME/slope-sim-data"
test -d "$smoke_parent"
smoke_run_root="$(mktemp -d \
  "$smoke_parent/stage4-clean-host-smoke.XXXXXX")"
doctor_evidence="$smoke_run_root/doctor.json"
smoke_transaction_dir="$smoke_run_root/smoke"
smoke_evidence="$smoke_transaction_dir/clean-host-no-participant-smoke.json"
run_handoff="$smoke_transaction_dir/clean-host-run.env"
test -d "$installed_root"
test -f "$install_state"
test -f "$relocation_marker"
test ! -e "$doctor_evidence"
test ! -e "$smoke_transaction_dir"
SLOPE_SIM_ROOT="$installed_root" \
  "$installed_root/bin/slope-sim" doctor --json "$doctor_evidence"
"$installed_root/runtime/python/bin/python" \
  "$installed_root/share/slope-sim/tools/verify_stage4_release.py" \
  --run-installed-no-participant-smoke \
  --installed-release-root "$installed_root" \
  --install-state "$install_state" \
  --relocation-marker "$relocation_marker" \
  --doctor-evidence "$doctor_evidence" \
  --transaction-dir "$smoke_transaction_dir" \
  --output "$smoke_evidence" \
  --clean-host-run-role initial \
  --write-clean-host-run-handoff "$run_handoff"
test -f "$smoke_evidence"
test -f "$run_handoff"
STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF="$run_handoff"
export STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF
```

Expected: 初次调用创建全新的 `stage4-clean-host-smoke.XXXXXX` evidence run，重新生成 fresh doctor，再以 canonical JSON 原子生成该 run 内的 `clean-host-no-participant-smoke.json`，最后原子写 `role=initial` 的 `clean-host-run.env`；绝不覆盖或复用旧 evidence。smoke/handoff 固定本轮 clean-host run id、resolved installed root、安装树 identity digest，以及 install state、relocation marker 和 fresh doctor 的绝对路径与 SHA-256。它验证 C++ ELF `--version`、Python import、canonical `models/robot_models.yaml` 读取，以及 `share/slope-sim/selftest/session.manifest.pb` 的 runtime digest/segment hash/五 topic 回读和 PCD/PLY/LVX2 导出；逐项与同目录 `selftest-evidence.json` 交叉验证。缺失、篡改、路径绑定错误或跨安装轮次混用任一输入都必须非零退出且不创建 smoke/handoff。这些检查不得启动 Simulator、Command、Recorder live session，也不得把 self-test 或 LocalTransport 结果写成生产通过。调用者保留精确 `STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF` 供 Step 5 显式消费；换 shell 时必须从本步明确输出重新设置，不能搜索目录猜测。报告可以索引该 run，但 Task 9 不信任报告选择。

- [ ] **Step 3: 对目标机每条真实生产 invocation 单独授权并运行**

先创建本轮唯一 production session；该命令不创建 eCAL participant，不需要真实运行授权：

```bash
test -n "${STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF:-}"
installed_root="$(readlink -f /opt/slope-sim/current)"
production_session_root="$(mktemp -d \
  "$HOME/slope-sim-data/stage4-clean-host-production-session.XXXXXX")"
production_session_transaction_dir="$production_session_root/session"
STAGE4_CLEAN_HOST_PRODUCTION_SESSION_FILE="$production_session_transaction_dir/production-session.env"
test ! -e "$production_session_transaction_dir"
"$installed_root/runtime/python/bin/python" \
  "$installed_root/share/slope-sim/tools/verify_stage4_release.py" \
  --begin-clean-host-production-session \
  --initial-clean-host-run-handoff \
    "$STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF" \
  --transaction-dir "$production_session_transaction_dir" \
  --output "$production_session_transaction_dir/production-session.json" \
  --write-clean-host-production-session \
    "$STAGE4_CLEAN_HOST_PRODUCTION_SESSION_FILE"
test -f "$STAGE4_CLEAN_HOST_PRODUCTION_SESSION_FILE"
export STAGE4_CLEAN_HOST_PRODUCTION_SESSION_FILE
```

headless、interactive、live ROS 和 replay ROS 都是独立真实运行：每一条之前说明 mode/车型/时长，取得用户明确授权并即时扫描目标机负载；失败保留证据并停止，不能沿用一次授权继续下一条。每个 installed verifier invocation 都必须显式接收 `STAGE4_CLEAN_HOST_PRODUCTION_SESSION_FILE`，并分别原子输出由调用者立即保存到 `STAGE4_CLEAN_HOST_HEADLESS_EVIDENCE`、`STAGE4_CLEAN_HOST_INTERACTIVE_EVIDENCE`、`STAGE4_CLEAN_HOST_LIVE_ROS_EVIDENCE` 和 `STAGE4_CLEAN_HOST_REPLAY_ROS_EVIDENCE` 的结构化 JSON；不能扫描历史结果选“最新成功”。通过后验证 C++ Subscriber、Command、Recorder、Replay/Export、ROS/RViz2；replay ROS 必须由 Bridge 的逐 topic `TopicHealth` 证明安装后 Replay 实际注册了原始完整 v2 type name、`proto` encoding 和 descriptor digest，并把 callback raw payload/hash 与源 MCAP 交叉验证。生成 MCAP、PCD、PLY、LVX2 并在相应工具打开。四份 JSON 以代码内固定 schema 引用本轮完整 session manifest/全部 MCAP segment、PCD/PLY/LVX2、四类 invocation 日志、GUI/RViz2/Livox Viewer 截图和工具打开结果；每个引用都有绝对路径、SHA-256、role/run id、共同 production session id 和 candidate install identity。任一 invocation 失败即废弃本 session；复测必须创建新 session、重新逐条授权并运行四类角色，不能拿旧成功结果补齐。

四条真实 invocation 全部通过后，使用 installed verifier 捕获目标机 CPU/RAM/GPU/驱动、Ubuntu、ROS/eCAL 与 SSH host public key identity，并把本轮安装日志和四份结果冻结成唯一 production-evidence handoff：

```bash
test -n "${STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF:-}"
test -n "${STAGE4_CLEAN_HOST_PRODUCTION_SESSION_FILE:-}"
test -n "${STAGE4_CLEAN_HOST_HEADLESS_EVIDENCE:-}"
test -n "${STAGE4_CLEAN_HOST_INTERACTIVE_EVIDENCE:-}"
test -n "${STAGE4_CLEAN_HOST_LIVE_ROS_EVIDENCE:-}"
test -n "${STAGE4_CLEAN_HOST_REPLAY_ROS_EVIDENCE:-}"
test -n "${STAGE4_CLEAN_HOST_INSTALL_LOG:-}"
test -n "${STAGE4_CLEAN_HOST_SSH_HOST_PUBLIC_KEY:-}"
installed_root="$(readlink -f /opt/slope-sim/current)"
production_root="$(mktemp -d \
  "$HOME/slope-sim-data/stage4-clean-host-production.XXXXXX")"
host_inventory="$production_root/host-inventory.json"
production_transaction_dir="$production_root/evidence"
production_json="$production_transaction_dir/production-evidence.json"
production_handoff="$production_transaction_dir/production-evidence.env"
test ! -e "$production_transaction_dir"
"$installed_root/runtime/python/bin/python" \
  "$installed_root/share/slope-sim/tools/verify_stage4_release.py" \
  --capture-clean-host-inventory \
  --initial-clean-host-run-handoff \
    "$STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF" \
  --ssh-host-public-key "$STAGE4_CLEAN_HOST_SSH_HOST_PUBLIC_KEY" \
  --output "$host_inventory"
"$installed_root/runtime/python/bin/python" \
  "$installed_root/share/slope-sim/tools/verify_stage4_release.py" \
  --freeze-clean-host-production-evidence \
  --production-session "$STAGE4_CLEAN_HOST_PRODUCTION_SESSION_FILE" \
  --initial-clean-host-run-handoff \
    "$STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF" \
  --install-log "$STAGE4_CLEAN_HOST_INSTALL_LOG" \
  --host-inventory "$host_inventory" \
  --headless-evidence "$STAGE4_CLEAN_HOST_HEADLESS_EVIDENCE" \
  --interactive-evidence "$STAGE4_CLEAN_HOST_INTERACTIVE_EVIDENCE" \
  --live-ros-evidence "$STAGE4_CLEAN_HOST_LIVE_ROS_EVIDENCE" \
  --replay-ros-evidence "$STAGE4_CLEAN_HOST_REPLAY_ROS_EVIDENCE" \
  --transaction-dir "$production_transaction_dir" \
  --output "$production_json" \
  --write-clean-host-production-evidence-handoff "$production_handoff"
test -f "$production_json"
test -f "$production_handoff"
STAGE4_CLEAN_HOST_PRODUCTION_EVIDENCE_HANDOFF="$production_handoff"
export STAGE4_CLEAN_HOST_PRODUCTION_EVIDENCE_HANDOFF
```

Expected: freezer 结构化重算四份 JSON 和代码内固定的所有递归成员，不接受 caller 提供成员 allowlist。缺任一 role、session segment、导出文件、截图、工具结果、安装/运行日志或 host inventory，任一文件在 handoff 前后变化，跨 candidate/anchor/run 混用，重复 role，旧成功结果，链接/特殊文件或额外未声明成员都失败且不创建 production JSON/handoff。handoff 固定完整成员清单、逐文件 size/hash、总 member/byte 上限与 candidate/target identity，供 Step 5 exporter 在远端路径仍有效时复制实际 bytes。

- [ ] **Step 4: 验证升级、回退和卸载**

调用者把 Step 15 已验证的整个 `lifecycle-probe/` sibling bundle 原样复制到目标机，并把 `STAGE4_TARGET_LIFECYCLE_PROBE_HANDOFF` 指向其中的 portable JSON。以下命令必须在 Step 1 主 candidate 仍是 `current` 时执行；所有 lifecycle evidence、doctor 和 probe 解包输出只写本轮 transfer root，不能写入或包含被比较的 config/data tree：

```bash
test -n "${STAGE4_TARGET_LIFECYCLE_PROBE_HANDOFF:-}"
test -n "${STAGE4_CLEAN_HOST_REMOTE_ROOT:-}"
test -f "$STAGE4_TARGET_LIFECYCLE_PROBE_HANDOFF"
install -d -m 0700 "$HOME/.config/slope-sim" "$HOME/slope-sim-data"
lifecycle_run_root="$(mktemp -d \
  "$STAGE4_CLEAN_HOST_REMOTE_ROOT/stage4-lifecycle.XXXXXX")"
primary_root="$(readlink -f /opt/slope-sim/current)"
primary_state="$primary_root/install-state.json"
primary_marker="$primary_root/relocation-state.json"
primary_doctor="$lifecycle_run_root/primary-doctor.json"
probe_context_transaction_dir="$lifecycle_run_root/probe-context"
probe_preflight="$probe_context_transaction_dir/probe-preflight.json"
probe_context="$probe_context_transaction_dir/lifecycle-probe.env"
test ! -e "$probe_context_transaction_dir"
SLOPE_SIM_ROOT="$primary_root" \
  "$primary_root/bin/slope-sim" doctor --json "$primary_doctor"
"$primary_root/runtime/python/bin/python" \
  "$primary_root/share/slope-sim/tools/verify_stage4_release.py" \
  --verify-lifecycle-probe-handoff \
    "$STAGE4_TARGET_LIFECYCLE_PROBE_HANDOFF" \
  --primary-install-state "$primary_state" \
  --transaction-dir "$probe_context_transaction_dir" \
  --output "$probe_preflight" \
  --write-lifecycle-probe-context "$probe_context"
test -f "$probe_preflight"
test -f "$probe_context"
"$primary_root/runtime/python/bin/python" \
  "$primary_root/share/slope-sim/tools/verify_stage4_release.py" \
  --verify-lifecycle-probe-context "$probe_context" \
  --expected-transaction-dir "$probe_context_transaction_dir"
source "$probe_context"
test "$STAGE4_LIFECYCLE_PROBE_VERSION" != "$STAGE4_PRIMARY_RELEASE_VERSION"
"$primary_root/runtime/python/bin/python" \
  "$primary_root/share/slope-sim/tools/verify_stage4_release.py" \
  --snapshot-lifecycle-state \
  --role before \
  --installed-release-root "$primary_root" \
  --install-state "$primary_state" \
  --relocation-marker "$primary_marker" \
  --doctor-evidence "$primary_doctor" \
  --config-root "$HOME/.config/slope-sim" \
  --data-root "$HOME/slope-sim-data" \
  --output "$lifecycle_run_root/before.json"
probe_extract_root="$(mktemp -d \
  "$STAGE4_CLEAN_HOST_REMOTE_ROOT/stage4-lifecycle-unpacked.XXXXXX")"
(
  cd -- "$(dirname -- "$STAGE4_LIFECYCLE_PROBE_ARCHIVE_SHA256_FILE")"
  sha256sum -c -- \
    "$(basename -- "$STAGE4_LIFECYCLE_PROBE_ARCHIVE_SHA256_FILE")"
)
tar --extract --zstd --file "$STAGE4_LIFECYCLE_PROBE_ARCHIVE" \
  --directory "$probe_extract_root" \
  --no-same-owner --no-same-permissions --numeric-owner
probe_source_root="$probe_extract_root/\
slope-sim-stage4-$STAGE4_LIFECYCLE_PROBE_VERSION"
probe_archive_sha256="$(
  sha256sum "$STAGE4_LIFECYCLE_PROBE_ARCHIVE" | cut -d ' ' -f 1
)"
probe_build_evidence_sha256="$(
  sha256sum "$STAGE4_LIFECYCLE_PROBE_BUILD_EVIDENCE" | cut -d ' ' -f 1
)"
sudo "$probe_source_root/install.sh" --offline \
  --prefix /opt/slope-sim \
  --source-root "$probe_source_root" \
  --archive-file "$STAGE4_LIFECYCLE_PROBE_ARCHIVE" \
  --archive-sha256-file "$STAGE4_LIFECYCLE_PROBE_ARCHIVE_SHA256_FILE" \
  --archive-sha256 "$probe_archive_sha256" \
  --build-evidence "$STAGE4_LIFECYCLE_PROBE_BUILD_EVIDENCE" \
  --build-evidence-sha256 "$probe_build_evidence_sha256"
probe_root="$(readlink -f /opt/slope-sim/current)"
test "$probe_root" = \
  "/opt/slope-sim/releases/$STAGE4_LIFECYCLE_PROBE_VERSION"
probe_doctor="$lifecycle_run_root/probe-doctor.json"
SLOPE_SIM_ROOT="$probe_root" \
  "$probe_root/bin/slope-sim" doctor --json "$probe_doctor"
"$probe_root/runtime/python/bin/python" \
  "$probe_root/share/slope-sim/tools/verify_stage4_release.py" \
  --snapshot-lifecycle-state \
  --role upgraded \
  --installed-release-root "$probe_root" \
  --install-state "$probe_root/install-state.json" \
  --relocation-marker "$probe_root/relocation-state.json" \
  --doctor-evidence "$probe_doctor" \
  --config-root "$HOME/.config/slope-sim" \
  --data-root "$HOME/slope-sim-data" \
  --output "$lifecycle_run_root/upgraded.json"
sudo "$probe_root/install.sh" \
  --activate-existing "$STAGE4_PRIMARY_RELEASE_VERSION" \
  --prefix /opt/slope-sim
restored_root="$(readlink -f /opt/slope-sim/current)"
test "$restored_root" = "$primary_root"
restored_doctor="$lifecycle_run_root/restored-doctor.json"
SLOPE_SIM_ROOT="$restored_root" \
  "$restored_root/bin/slope-sim" doctor --json "$restored_doctor"
sudo "$restored_root/uninstall.sh" \
  --prefix /opt/slope-sim \
  --version "$STAGE4_LIFECYCLE_PROBE_VERSION"
test "$(readlink -f /opt/slope-sim/current)" = "$primary_root"
test ! -e "/opt/slope-sim/releases/$STAGE4_LIFECYCLE_PROBE_VERSION"
lifecycle_transaction_dir="$lifecycle_run_root/evidence"
lifecycle_json="$lifecycle_transaction_dir/lifecycle-evidence.json"
lifecycle_handoff="$lifecycle_transaction_dir/lifecycle-evidence.env"
test ! -e "$lifecycle_transaction_dir"
"$restored_root/runtime/python/bin/python" \
  "$restored_root/share/slope-sim/tools/verify_stage4_release.py" \
  --freeze-lifecycle-evidence \
  --before-snapshot "$lifecycle_run_root/before.json" \
  --upgraded-snapshot "$lifecycle_run_root/upgraded.json" \
  --restored-release-root "$restored_root" \
  --restored-install-state "$restored_root/install-state.json" \
  --restored-relocation-marker "$restored_root/relocation-state.json" \
  --restored-doctor-evidence "$restored_doctor" \
  --removed-probe-release \
    "/opt/slope-sim/releases/$STAGE4_LIFECYCLE_PROBE_VERSION" \
  --config-root "$HOME/.config/slope-sim" \
  --data-root "$HOME/slope-sim-data" \
  --transaction-dir "$lifecycle_transaction_dir" \
  --output "$lifecycle_json" \
  --write-lifecycle-evidence-handoff "$lifecycle_handoff"
test -f "$lifecycle_json"
test -f "$lifecycle_handoff"
STAGE4_LIFECYCLE_EVIDENCE_HANDOFF="$lifecycle_handoff"
export STAGE4_LIFECYCLE_EVIDENCE_HANDOFF
```

Expected: probe 是由 Task 6 第三个全新根从同一 clean HEAD/locks/caches/toolchain 构建的完整合法版本，但其 handoff 固定 `publishable=false`。probe preflight JSON/env 先在全新 transaction dir 的 sibling staging 中完整 fsync，再以一次目录 rename 提交；独立 context consumer 复核 pair 和提交状态后才 `source`。安装 probe 后 `current` 真实切到第二版本；`--activate-existing` 只在原主版本 state/marker/fresh doctor 和安装树 identity 仍有效后原子回退；随后只能卸载已经非 current 的 probe。lifecycle/probe 输出根与 config/data 互不包含，因此 before/upgraded/final 比较不被验收证据自身写入污染。最终 `current`、主版本和用户配置/数据恢复并保持，probe release 目录消失，systemd 仍未默认 enable。post-install doctor 失败和旧 `current` 不变由 Task 4 的 RED/GREEN fixture 覆盖，真实包不加入 test-only fail hook，也不在本 Step 伪造失败。

- [ ] **Step 5: REFACTOR 迁移证据组织并重复无 participant smoke**

只整理仓库外原始证据和交付报告索引，不修改候选内教程或安装/运行行为。无需整理时先记录“REFACTOR：无必要”；无论是否整理，都先在验收控制机创建与 candidate、probe、目标机 identity 和预置 SSH host key 绑定的一次性 challenge。`STAGE4_CLEAN_HOST_KNOWN_HOSTS` 必须是事先通过独立可信渠道固定的专用文件，不能在本轮用 `ssh-keyscan` 临时信任；`STAGE4_CLEAN_HOST_REMOTE_ROOT` 是 Step 1 已在目标机建立的本轮 0700 绝对 transfer root：

```bash
test -n "${STAGE4_RELEASE_HANDOFF_FILE:-}"
test -n "${STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE:-}"
test -n "${STAGE4_CLEAN_HOST_TARGET:-}"
test -n "${STAGE4_CLEAN_HOST_TARGET_ID:-}"
test -n "${STAGE4_CLEAN_HOST_REMOTE_ROOT:-}"
test -n "${STAGE4_CLEAN_HOST_HOST_KEY_SHA256:-}"
test -n "${STAGE4_CLEAN_HOST_KNOWN_HOSTS:-}"
test -n "${STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY:-}"
test -n "${STAGE4_EXTERNAL_ACCEPTANCE_PARENT:-}"
test -f "$STAGE4_CLEAN_HOST_KNOWN_HOSTS"
test -d "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT"
install -d -m 0700 "$STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY"
test "$(stat -c '%a' "$STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY")" = 700
STAGE4_CLEAN_HOST_IMPORT_ROOT="$(mktemp -d \
  "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT/slope-sim-stage4-clean-host-import.XXXXXX")"
STAGE4_CLEAN_HOST_CHALLENGE_FILE="$STAGE4_CLEAN_HOST_IMPORT_ROOT/clean-host-challenge.json"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --create-clean-host-challenge \
  --release-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --lifecycle-probe-handoff "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE" \
  --target-id "$STAGE4_CLEAN_HOST_TARGET_ID" \
  --ssh-target "$STAGE4_CLEAN_HOST_TARGET" \
  --remote-root "$STAGE4_CLEAN_HOST_REMOTE_ROOT" \
  --known-hosts "$STAGE4_CLEAN_HOST_KNOWN_HOSTS" \
  --expected-host-key-sha256 "$STAGE4_CLEAN_HOST_HOST_KEY_SHA256" \
  --challenge-registry "$STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY" \
  --output "$STAGE4_CLEAN_HOST_CHALLENGE_FILE"
scp -p -o BatchMode=yes -o StrictHostKeyChecking=yes \
  -o UserKnownHostsFile="$STAGE4_CLEAN_HOST_KNOWN_HOSTS" \
  "$STAGE4_CLEAN_HOST_CHALLENGE_FILE" \
  "$STAGE4_CLEAN_HOST_TARGET:$STAGE4_CLEAN_HOST_REMOTE_ROOT/clean-host-challenge.json"
```

接着在目标机使用初次 run 的显式 handoff 执行全新 repeat run、结构化 chain freeze，并在所有 `/opt`/`$HOME` 路径仍有效时导出 portable bundle。调用者显式设置与控制机相同的 target id、remote root，以及实际提供 SSH 服务的只读 host public key 文件：

```bash
test -n "${STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF:-}"
test -n "${STAGE4_LIFECYCLE_EVIDENCE_HANDOFF:-}"
test -n "${STAGE4_CLEAN_HOST_PRODUCTION_EVIDENCE_HANDOFF:-}"
test -n "${STAGE4_CLEAN_HOST_TARGET_ID:-}"
test -n "${STAGE4_CLEAN_HOST_REMOTE_ROOT:-}"
test -n "${STAGE4_CLEAN_HOST_SSH_HOST_PUBLIC_KEY:-}"
test -f "$STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF"
test -f "$STAGE4_LIFECYCLE_EVIDENCE_HANDOFF"
test -f "$STAGE4_CLEAN_HOST_PRODUCTION_EVIDENCE_HANDOFF"
test -f "$STAGE4_CLEAN_HOST_SSH_HOST_PUBLIC_KEY"
target_challenge="$STAGE4_CLEAN_HOST_REMOTE_ROOT/clean-host-challenge.json"
test -f "$target_challenge"
installed_root="$(readlink -f /opt/slope-sim/current)"
install_state="$installed_root/install-state.json"
relocation_marker="$installed_root/relocation-state.json"
smoke_parent="$HOME/slope-sim-data"
repeat_run_root="$(mktemp -d \
  "$smoke_parent/stage4-clean-host-smoke.XXXXXX")"
repeat_doctor="$repeat_run_root/doctor.json"
repeat_transaction_dir="$repeat_run_root/smoke"
repeat_smoke="$repeat_transaction_dir/clean-host-no-participant-smoke.json"
repeat_handoff="$repeat_transaction_dir/clean-host-run.env"
test ! -e "$repeat_transaction_dir"
SLOPE_SIM_ROOT="$installed_root" \
  "$installed_root/bin/slope-sim" doctor --json "$repeat_doctor"
"$installed_root/runtime/python/bin/python" \
  "$installed_root/share/slope-sim/tools/verify_stage4_release.py" \
  --run-installed-no-participant-smoke \
  --installed-release-root "$installed_root" \
  --install-state "$install_state" \
  --relocation-marker "$relocation_marker" \
  --doctor-evidence "$repeat_doctor" \
  --transaction-dir "$repeat_transaction_dir" \
  --output "$repeat_smoke" \
  --clean-host-run-role repeat \
  --write-clean-host-run-handoff "$repeat_handoff"
test -f "$repeat_smoke"
test -f "$repeat_handoff"
chain_root="$(mktemp -d \
  "$smoke_parent/stage4-clean-host-chain.XXXXXX")"
chain_transaction_dir="$chain_root/chain"
chain_json="$chain_transaction_dir/clean-host-chain.json"
chain_handoff="$chain_transaction_dir/clean-host-chain.env"
test ! -e "$chain_transaction_dir"
"$installed_root/runtime/python/bin/python" \
  "$installed_root/share/slope-sim/tools/verify_stage4_release.py" \
  --verify-clean-host-run-handoff \
    "$STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF" \
  --output "$chain_root/initial-preflight.json"
"$installed_root/runtime/python/bin/python" \
  "$installed_root/share/slope-sim/tools/verify_stage4_release.py" \
  --verify-clean-host-run-handoff "$repeat_handoff" \
  --output "$chain_root/repeat-preflight.json"
"$installed_root/runtime/python/bin/python" \
  "$installed_root/share/slope-sim/tools/verify_stage4_release.py" \
  --freeze-clean-host-chain \
  --initial-clean-host-run-handoff \
    "$STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF" \
  --repeat-clean-host-run-handoff "$repeat_handoff" \
  --transaction-dir "$chain_transaction_dir" \
  --output "$chain_json" \
  --write-clean-host-chain-handoff "$chain_handoff"
test -f "$chain_json"
test -f "$chain_handoff"
target_bundle_transaction_dir="$STAGE4_CLEAN_HOST_REMOTE_ROOT/bundle"
target_bundle="$target_bundle_transaction_dir/clean-host-evidence-bundle.tar.zst"
target_bundle_sidecar="${target_bundle}.sha256"
test ! -e "$target_bundle_transaction_dir"
"$installed_root/runtime/python/bin/python" \
  "$installed_root/share/slope-sim/tools/verify_stage4_release.py" \
  --export-clean-host-evidence-bundle \
  --initial-clean-host-run-handoff \
    "$STAGE4_CLEAN_HOST_INITIAL_RUN_HANDOFF" \
  --repeat-clean-host-run-handoff "$repeat_handoff" \
  --clean-host-chain-handoff "$chain_handoff" \
  --lifecycle-evidence-handoff "$STAGE4_LIFECYCLE_EVIDENCE_HANDOFF" \
  --production-evidence-handoff \
    "$STAGE4_CLEAN_HOST_PRODUCTION_EVIDENCE_HANDOFF" \
  --challenge "$target_challenge" \
  --target-id "$STAGE4_CLEAN_HOST_TARGET_ID" \
  --remote-root "$STAGE4_CLEAN_HOST_REMOTE_ROOT" \
  --ssh-host-public-key "$STAGE4_CLEAN_HOST_SSH_HOST_PUBLIC_KEY" \
  --transaction-dir "$target_bundle_transaction_dir" \
  --output "$target_bundle" \
  --write-bundle-sha256 "$target_bundle_sidecar"
test -f "$target_bundle"
test -f "$target_bundle_sidecar"
```

最后回到验收控制机，通过同一 pinned SSH host key 拉回唯一 bundle，并在新的本地 import root 生成唯一可供 Task 9 消费的 context：

```bash
test -n "${STAGE4_CLEAN_HOST_IMPORT_ROOT:-}"
test -n "${STAGE4_CLEAN_HOST_CHALLENGE_FILE:-}"
test -n "${STAGE4_RELEASE_HANDOFF_FILE:-}"
test -n "${STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE:-}"
test -n "${STAGE4_CLEAN_HOST_TARGET:-}"
test -n "${STAGE4_CLEAN_HOST_TARGET_ID:-}"
test -n "${STAGE4_CLEAN_HOST_REMOTE_ROOT:-}"
test -n "${STAGE4_CLEAN_HOST_HOST_KEY_SHA256:-}"
test -n "${STAGE4_CLEAN_HOST_KNOWN_HOSTS:-}"
test -n "${STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY:-}"
incoming_staging="$STAGE4_CLEAN_HOST_IMPORT_ROOT/.incoming.tmp"
incoming_dir="$STAGE4_CLEAN_HOST_IMPORT_ROOT/incoming"
test ! -e "$incoming_staging"
test ! -e "$incoming_dir"
install -d -m 0700 "$incoming_staging"
imported_bundle="$incoming_dir/clean-host-evidence-bundle.tar.zst"
imported_bundle_sidecar="${imported_bundle}.sha256"
scp -p -o BatchMode=yes -o StrictHostKeyChecking=yes \
  -o UserKnownHostsFile="$STAGE4_CLEAN_HOST_KNOWN_HOSTS" \
  "$STAGE4_CLEAN_HOST_TARGET:$STAGE4_CLEAN_HOST_REMOTE_ROOT/bundle/clean-host-evidence-bundle.tar.zst" \
  "$STAGE4_CLEAN_HOST_TARGET:$STAGE4_CLEAN_HOST_REMOTE_ROOT/bundle/clean-host-evidence-bundle.tar.zst.sha256" \
  "$incoming_staging/"
mv "$incoming_staging" "$incoming_dir"
import_transaction_dir="$STAGE4_CLEAN_HOST_IMPORT_ROOT/import"
import_context_json="$import_transaction_dir/clean-host-import-context.json"
STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE="$import_transaction_dir/clean-host-import-context.env"
test ! -e "$import_transaction_dir"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --import-clean-host-evidence-bundle "$imported_bundle" \
  --bundle-sha256 "$imported_bundle_sidecar" \
  --challenge "$STAGE4_CLEAN_HOST_CHALLENGE_FILE" \
  --release-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --lifecycle-probe-handoff "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE" \
  --target-id "$STAGE4_CLEAN_HOST_TARGET_ID" \
  --ssh-target "$STAGE4_CLEAN_HOST_TARGET" \
  --remote-root "$STAGE4_CLEAN_HOST_REMOTE_ROOT" \
  --known-hosts "$STAGE4_CLEAN_HOST_KNOWN_HOSTS" \
  --expected-host-key-sha256 "$STAGE4_CLEAN_HOST_HOST_KEY_SHA256" \
  --challenge-registry "$STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY" \
  --transaction-dir "$import_transaction_dir" \
  --output "$import_context_json" \
  --write-clean-host-import-context \
    "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-clean-host-import-context \
    "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE" \
  --challenge-registry "$STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY" \
  --output "$STAGE4_CLEAN_HOST_IMPORT_ROOT/import-context-preflight.json"
source "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE"
test -f "$STAGE4_IMPORTED_CLEAN_HOST_CHAIN_EVIDENCE"
test -f "$STAGE4_IMPORTED_LIFECYCLE_EVIDENCE"
test -f "$STAGE4_IMPORTED_CLEAN_HOST_PRODUCTION_EVIDENCE"
test -f "$STAGE4_CLEAN_HOST_CHALLENGE_RECEIPT"
test -f "$import_context_json"
test -f "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE"
export STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE
```

Expected: repeat handoff 的 role 精确为 `repeat`，run id/目录与 initial 不同，但 resolved install root、archive/source identity 和归一化 smoke 结果相同。chain verifier 在目标机结构化重算两份 handoff 及其 root/state/marker/doctor/smoke；exporter 同轮重算 lifecycle、四类 production evidence 的实际 MCAP/导出/截图/日志/inventory members、目标机 identity、candidate/probe 来源和 challenge 后，生成 deterministic portable archive/sidecar。控制机的 importer 只信任由预置 host key 约束的本轮传输，安全读取 archive，拒绝 challenge/host/root/archive/member 漂移；持久 registry 必须原子记录本次 `issued -> consuming -> consumed` 和 receipt，同一 challenge/bundle 在任何其他 import root 重放都失败。本地 context 固定 imported chain/lifecycle/production evidence 与 receipt。任一失败保留诊断 evidence 并停止，不自动重跑或挑选其他成功 run；真实 invocation 不因外部索引整理自动重跑。交付报告只索引 `STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE` 及摘要，Task 9 从 imported production root 核对原始 JSON/MCAP/截图/日志，不读取远端 env 中的 `/opt`/`$HOME` 路径。若必须修改教程，按本 Task 的 TDD 裁决使候选失效并重建，不能把它当作证据整理。

## Task 9：最终六维审查与状态裁决

**Files:**
- Modify: `docs/阶段四交付报告.md`
- Modify: `README.md`

- [ ] **Step 1: 冻结 writer 并启动独立只读审查**

审查需求完整性、逻辑正确性、边界情况、代码质量、测试覆盖和实际运行结果；必须核对原始 JSON/MCAP/截图/安装日志，不能只读摘要。审查者不得修改仓库或验收产物，只在 `STAGE4_EXTERNAL_ACCEPTANCE_PARENT` 下自己的全新 0700 目录写 canonical review source；source 必须记录独立 reviewer identity/task id、被审 commit/tree、六个精确维度、全部发现/disposition 和逐项 path/size/SHA-256 evidence index，不能把交付报告当 evidence oracle。

- [ ] **Step 2: 清零 Critical/Important**

每个发现回到所属 A-E 子计划补 RED/GREEN 和相应外部门禁；审查者不直接修改代码。若修复触及 README/交付报告之外的任何源码、lock、测试、packaging、配置、资源或随包教程，Task 6 的验收候选与 Task 7/8 证据立即失效，必须重新构建并重跑相应真实门，不能继续做 payload 等价继承。修复后旧 review source/transaction 永久保留但不再可接受，必须启动新的独立只读复审并由其输出新的 source；只有最新 source 的六维集合精确、全部 evidence 可重算且 `Critical=0, Important=0` 才进入 Step 3。

- [ ] **Step 3: 最终一致性检查**

Run: `git diff --check`

Expected: 无输出。

Run:

```bash
test -n "${STAGE4_RELEASE_HANDOFF_FILE:-}"
test -n "${STAGE4_ACCEPTANCE_HANDOFF_FILE:-}"
test -n "${STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE:-}"
test -n "${STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY:-}"
test -n "${STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE:-}"
test -n "${STAGE4_SIX_DIMENSION_REVIEW_SOURCE_FILE:-}"
test -n "${STAGE4_EXTERNAL_ACCEPTANCE_PARENT:-}"
test -d "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT"
test -f "$STAGE4_SIX_DIMENSION_REVIEW_SOURCE_FILE"
STAGE4_FINAL_REVIEW_ROOT="$(mktemp -d \
  "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT/slope-sim-stage4-final-review.XXXXXX")"
STAGE4_SIX_DIMENSION_REVIEW_TRANSACTION_DIR="\
$STAGE4_FINAL_REVIEW_ROOT/six-dimension-review"
STAGE4_SIX_DIMENSION_REVIEW_EVIDENCE_FILE="\
$STAGE4_SIX_DIMENSION_REVIEW_TRANSACTION_DIR/six-dimension-review.json"
STAGE4_SIX_DIMENSION_REVIEW_HANDOFF_FILE="\
$STAGE4_SIX_DIMENSION_REVIEW_TRANSACTION_DIR/six-dimension-review.env"
test ! -e "$STAGE4_SIX_DIMENSION_REVIEW_TRANSACTION_DIR"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --output "$STAGE4_FINAL_REVIEW_ROOT/final-review-handoff-preflight.json"
source "$STAGE4_RELEASE_HANDOFF_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-acceptance-handoff "$STAGE4_ACCEPTANCE_HANDOFF_FILE" \
  --output "$STAGE4_FINAL_REVIEW_ROOT/acceptance-handoff-preflight.json"
source "$STAGE4_ACCEPTANCE_HANDOFF_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-clean-host-import-context \
    "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE" \
  --challenge-registry "$STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY" \
  --output "$STAGE4_FINAL_REVIEW_ROOT/clean-host-import-preflight.json"
source "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE"
test -f "$STAGE4_IMPORTED_CLEAN_HOST_CHAIN_EVIDENCE"
test -f "$STAGE4_IMPORTED_LIFECYCLE_EVIDENCE"
test -f "$STAGE4_IMPORTED_CLEAN_HOST_PRODUCTION_EVIDENCE"
test -f "$STAGE4_CLEAN_HOST_CHALLENGE_RECEIPT"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-lifecycle-probe-handoff \
    "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE" \
  --primary-release-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --output "$STAGE4_FINAL_REVIEW_ROOT/lifecycle-probe-preflight.json"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --freeze-six-dimension-review \
  --review-source "$STAGE4_SIX_DIMENSION_REVIEW_SOURCE_FILE" \
  --release-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --acceptance-handoff "$STAGE4_ACCEPTANCE_HANDOFF_FILE" \
  --clean-host-import-context "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE" \
  --lifecycle-probe-handoff "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE" \
  --transaction-dir "$STAGE4_SIX_DIMENSION_REVIEW_TRANSACTION_DIR" \
  --output "$STAGE4_SIX_DIMENSION_REVIEW_EVIDENCE_FILE" \
  --write-six-dimension-review-handoff \
    "$STAGE4_SIX_DIMENSION_REVIEW_HANDOFF_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-six-dimension-review-handoff \
    "$STAGE4_SIX_DIMENSION_REVIEW_HANDOFF_FILE" \
  --output "$STAGE4_FINAL_REVIEW_ROOT/six-dimension-review-preflight.json"
source "$STAGE4_SIX_DIMENSION_REVIEW_HANDOFF_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --archive "$STAGE4_RELEASE_ARCHIVE" \
  --archive-sha256 "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE" \
  --build-evidence "$STAGE4_RELEASE_BUILD_EVIDENCE" \
  --acceptance-handoff "$STAGE4_ACCEPTANCE_HANDOFF_FILE" \
  --candidate-installed-release-root "$STAGE4_ACCEPTANCE_RELEASE_ROOT" \
  --candidate-install-state "$STAGE4_ACCEPTANCE_INSTALL_STATE" \
  --candidate-relocation-marker "$STAGE4_ACCEPTANCE_RELOCATION_MARKER" \
  --candidate-doctor-evidence "$STAGE4_ACCEPTANCE_DOCTOR_EVIDENCE" \
  --candidate-smoke-evidence "$STAGE4_ACCEPTANCE_SMOKE_EVIDENCE" \
  --clean-host-import-context "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE" \
  --imported-clean-host-chain-evidence \
    "$STAGE4_IMPORTED_CLEAN_HOST_CHAIN_EVIDENCE" \
  --imported-lifecycle-evidence "$STAGE4_IMPORTED_LIFECYCLE_EVIDENCE" \
  --imported-production-evidence \
    "$STAGE4_IMPORTED_CLEAN_HOST_PRODUCTION_EVIDENCE" \
  --clean-host-challenge-receipt "$STAGE4_CLEAN_HOST_CHALLENGE_RECEIPT" \
  --lifecycle-probe-handoff "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE" \
  --six-dimension-review-handoff \
    "$STAGE4_SIX_DIMENSION_REVIEW_HANDOFF_FILE" \
  --require-complete-evidence
export STAGE4_SIX_DIMENSION_REVIEW_HANDOFF_FILE
```

Expected: 每次运行都创建一个全新的仓库外 review root；只有候选 installed root/state/marker/doctor/smoke 五件套同轮互绑，Task 8 初次与 Step 5 复验的两个 fresh clean-host run 来源安装相同、current/历史链选择无歧义，真实 lifecycle probe 的升级/回退/卸载与其 `publishable=false` 构建 handoff 相符，四类 production evidence 的原始 JSON/MCAP/导出/截图/安装与运行日志/host inventory 都已作为本地 imported members 复核，challenge registry/receipt 为 consumed，且真实 eCAL、真实 GUI/RViz2、Livox Viewer 和干净机 evidence 全部存在并匹配 manifest 时 rc=0。六维审查必须以一次目录 rename 提交 JSON/handoff，内嵌 reviewer identity、精确六维 verdict/findings/disposition 和已重算 evidence index，且 `Critical=0, Important=0`；独立 consumer 验证后 complete-evidence 才接受它。控制机只解析 portable bundle 产生的本地 import context/imported evidence，不重新读取目标机绝对路径，也不读取交付报告或 README。

- [ ] **Step 4: 记录 REFACTOR 裁决并原样复验最终门**

每个审查修复都必须回到所属 Task 完成原 RED/GREEN/REFACTOR；本 Task 不另造行为。全部无需进一步整理时记录“REFACTOR：无必要”，在新的仓库外 preflight root 原样重跑 Step 3 的全部只读门，并独立复核已经提交的六维 review transaction，再冻结已验收候选：

```bash
test -n "${STAGE4_RELEASE_HANDOFF_FILE:-}"
test -n "${STAGE4_ACCEPTANCE_HANDOFF_FILE:-}"
test -n "${STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE:-}"
test -n "${STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY:-}"
test -n "${STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE:-}"
test -n "${STAGE4_SIX_DIMENSION_REVIEW_HANDOFF_FILE:-}"
test -n "${STAGE4_EXTERNAL_ACCEPTANCE_PARENT:-}"
test -d "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT"
STAGE4_ACCEPTED_CONTEXT_ROOT="$(mktemp -d \
  "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT/slope-sim-stage4-accepted.XXXXXX")"
STAGE4_ACCEPTED_CANDIDATE_TRANSACTION_DIR="$STAGE4_ACCEPTED_CONTEXT_ROOT/accepted-candidate"
test ! -e "$STAGE4_ACCEPTED_CANDIDATE_TRANSACTION_DIR"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-acceptance-handoff "$STAGE4_ACCEPTANCE_HANDOFF_FILE" \
  --output "$STAGE4_ACCEPTED_CONTEXT_ROOT/acceptance-handoff-preflight.json"
source "$STAGE4_ACCEPTANCE_HANDOFF_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-clean-host-import-context \
    "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE" \
  --challenge-registry "$STAGE4_CLEAN_HOST_CHALLENGE_REGISTRY" \
  --output "$STAGE4_ACCEPTED_CONTEXT_ROOT/clean-host-import-preflight.json"
source "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE"
test -f "$STAGE4_IMPORTED_CLEAN_HOST_CHAIN_EVIDENCE"
test -f "$STAGE4_IMPORTED_LIFECYCLE_EVIDENCE"
test -f "$STAGE4_IMPORTED_CLEAN_HOST_PRODUCTION_EVIDENCE"
test -f "$STAGE4_CLEAN_HOST_CHALLENGE_RECEIPT"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-six-dimension-review-handoff \
    "$STAGE4_SIX_DIMENSION_REVIEW_HANDOFF_FILE" \
  --output "$STAGE4_ACCEPTED_CONTEXT_ROOT/six-dimension-review-preflight.json"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --freeze-accepted-candidate \
  --release-handoff "$STAGE4_RELEASE_HANDOFF_FILE" \
  --acceptance-handoff "$STAGE4_ACCEPTANCE_HANDOFF_FILE" \
  --candidate-installed-release-root "$STAGE4_ACCEPTANCE_RELEASE_ROOT" \
  --candidate-install-state "$STAGE4_ACCEPTANCE_INSTALL_STATE" \
  --candidate-relocation-marker "$STAGE4_ACCEPTANCE_RELOCATION_MARKER" \
  --candidate-doctor-evidence "$STAGE4_ACCEPTANCE_DOCTOR_EVIDENCE" \
  --candidate-smoke-evidence "$STAGE4_ACCEPTANCE_SMOKE_EVIDENCE" \
  --clean-host-import-context "$STAGE4_CLEAN_HOST_IMPORT_CONTEXT_FILE" \
  --imported-clean-host-chain-evidence \
    "$STAGE4_IMPORTED_CLEAN_HOST_CHAIN_EVIDENCE" \
  --imported-lifecycle-evidence "$STAGE4_IMPORTED_LIFECYCLE_EVIDENCE" \
  --imported-production-evidence \
    "$STAGE4_IMPORTED_CLEAN_HOST_PRODUCTION_EVIDENCE" \
  --clean-host-challenge-receipt "$STAGE4_CLEAN_HOST_CHALLENGE_RECEIPT" \
  --lifecycle-probe-handoff "$STAGE4_LIFECYCLE_PROBE_HANDOFF_FILE" \
  --six-dimension-review-handoff \
    "$STAGE4_SIX_DIMENSION_REVIEW_HANDOFF_FILE" \
  --transaction-dir "$STAGE4_ACCEPTED_CANDIDATE_TRANSACTION_DIR" \
  --output "$STAGE4_ACCEPTED_CANDIDATE_TRANSACTION_DIR/accepted-candidate.json" \
  --write-accepted-candidate-context \
    "$STAGE4_ACCEPTED_CANDIDATE_TRANSACTION_DIR/accepted-candidate.env"
```

Expected: verifier 只在候选 archive/release handoff、acceptance handoff、显式 clean-host import context、imported chain/lifecycle/production evidence、consumed challenge receipt、lifecycle-probe handoff、Task 7/8 原始 evidence，以及不可变六维 review JSON/handoff 中的 reviewer identity、精确六维集合、`Critical=0, Important=0` 与 evidence index 全部互相匹配后写 shell-safe context。context 固定候选 archive/handoff/clean HEAD、完整 evidence tree digest、六维 review JSON/handoff 的绝对路径与摘要、候选 `functional_source_epoch`、受限 provenance 派生闭包 schema version 和后续 source diff 的两个精确路径；它不得记录交付报告或 README 的路径/hash。还必须逐项固定 candidate resolved installed root 的绝对路径与 identity digest，以及 install state、relocation marker、fresh doctor、no-participant smoke 的绝对路径、SHA-256 和共同 `acceptance_run_id`。对 Task 8 则固定 portable archive/sidecar、challenge/registry receipt、target/host-key/candidate/probe identity、本地 imported chain/lifecycle/production 的路径与摘要、四类真实 role 及 initial/repeat run id；远端绝对路径仅作为 bundle 内被目标机 verifier 见证的字符串，不在控制机解引用。缺失、篡改、换 host/root/challenge、receipt 未 consumed、role 错配、跨 run/安装混用、probe 可发布、review identity/evidence 断链或仍传入报告时不得创建 JSON/context。verifier 仍以代码内常量复核 source diff 路径与闭包，不能信任 context 自报的 allowlist。调用者把 `STAGE4_ACCEPTED_CANDIDATE_CONTEXT_FILE` 指向该文件，后续不得靠当前 shell 临时变量定位候选。

- [ ] **Step 5: 再次取得 Git 授权并从最终证据 commit 重建双根**

先独立复核 `STAGE4_ACCEPTED_CANDIDATE_CONTEXT_FILE`，展示候选 commit 到当前工作树的结构化 diff；verifier 代码内只允许 `README.md` 与 `docs/阶段四交付报告.md` 变化，其他路径出现即按 Step 2 使候选失效。展示 Step 3-4 fresh GREEN、最终报告/README diff 和待提交清单后停止，向用户请求新的明确 commit 授权；未获授权不得 commit 或构建正式归档。获授权后按 `AGENTS.md` 的阶段四摘要规则提交，再要求 clean HEAD。随后保留 accepted-candidate context，使用这个新 HEAD 和两个全新仓库外空根只执行 Task 6 Steps 14-15 的 primary A/B release 构建、reproducibility、发布和普通 release handoff 部分；两轮 `build_release.sh --final-archive --artifact-purpose release` 都额外传入同一个 `--accepted-candidate-context "$STAGE4_ACCEPTED_CANDIDATE_CONTEXT_FILE"`，先复核固定 source diff，再使用候选冻结的 `functional_source_epoch` 构建功能载荷，同时把新的 clean commit/tree/source snapshot 写入 provenance。不得重跑 Step 14 的第三根 lifecycle-probe block，也不得把 accepted context 中冻结的 probe bundle 复制进 final output；probe 已完成候选安装事务验收且固定不可发布。除此之外 builder、比较和发布命令原样执行，且不得复用候选 work/output/cache。把新生成的 handoff 明确命名为 `STAGE4_FINAL_RELEASE_HANDOFF_FILE`，不能覆盖或改写 accepted-candidate context。

Expected: 正式 A/B 仍 byte-identical、各自 clean gate 通过，且归档内 manifest/外部 build evidence 都固定 `artifact_purpose=release`、`publishable=true`；final handoff 绑定新的 clean commit/tree/source snapshot 和全新 archive/sidecar/build evidence。正式输出目录不存在 lifecycle probe，旧候选、`publishable=false` probe 及其验收 evidence 保持只读可验证。

- [ ] **Step 6: 证明正式包继承已验收功能 payload 并重跑安装 smoke**

Run:

```bash
test -n "${STAGE4_ACCEPTED_CANDIDATE_CONTEXT_FILE:-}"
test -n "${STAGE4_FINAL_RELEASE_HANDOFF_FILE:-}"
test -n "${STAGE4_EXTERNAL_ACCEPTANCE_PARENT:-}"
test -d "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT"
STAGE4_FINAL_EQUIVALENCE_ROOT="$(mktemp -d \
  "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT/slope-sim-stage4-final-equivalence.XXXXXX")"
STAGE4_FINAL_EQUIVALENCE_TRANSACTION_DIR="$STAGE4_FINAL_EQUIVALENCE_ROOT/equivalence"
STAGE4_FINAL_EQUIVALENCE_OUTPUT="$STAGE4_FINAL_EQUIVALENCE_TRANSACTION_DIR/accepted-payload-equivalence.json"
STAGE4_FINAL_EQUIVALENCE_HANDOFF_FILE="$STAGE4_FINAL_EQUIVALENCE_TRANSACTION_DIR/accepted-payload-equivalence.env"
test ! -e "$STAGE4_FINAL_EQUIVALENCE_TRANSACTION_DIR"
test ! -e "$STAGE4_FINAL_EQUIVALENCE_OUTPUT"
test ! -e "$STAGE4_FINAL_EQUIVALENCE_HANDOFF_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-accepted-candidate-context \
    "$STAGE4_ACCEPTED_CANDIDATE_CONTEXT_FILE" \
  --output "$STAGE4_FINAL_EQUIVALENCE_ROOT/accepted-context-preflight.json"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-handoff "$STAGE4_FINAL_RELEASE_HANDOFF_FILE" \
  --output "$STAGE4_FINAL_EQUIVALENCE_ROOT/final-handoff-preflight.json"
source "$STAGE4_FINAL_RELEASE_HANDOFF_FILE"
test -f "$STAGE4_RELEASE_ARCHIVE"
test -f "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE"
test -f "$STAGE4_RELEASE_BUILD_EVIDENCE"
(
  cd -- "$(dirname -- "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE")"
  sha256sum -c -- "$(basename -- "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE")"
)
STAGE4_FINAL_EXTRACT_ROOT="$STAGE4_FINAL_EQUIVALENCE_ROOT/unpacked"
STAGE4_FINAL_INSTALL_PREFIX="$STAGE4_FINAL_EQUIVALENCE_ROOT/prefix"
STAGE4_FINAL_EVIDENCE_DIR="$STAGE4_FINAL_EQUIVALENCE_ROOT/evidence"
install -d -m 0700 \
  "$STAGE4_FINAL_EXTRACT_ROOT" "$STAGE4_FINAL_EVIDENCE_DIR"
test ! -e "$STAGE4_FINAL_INSTALL_PREFIX"
tar --extract --zstd --file "$STAGE4_RELEASE_ARCHIVE" \
  --directory "$STAGE4_FINAL_EXTRACT_ROOT" \
  --no-same-owner --no-same-permissions --numeric-owner
STAGE4_FINAL_SOURCE_ROOT="$STAGE4_FINAL_EXTRACT_ROOT/\
slope-sim-stage4-$STAGE4_RELEASE_VERSION"
STAGE4_FINAL_ARCHIVE_SHA256="$(
  sha256sum "$STAGE4_RELEASE_ARCHIVE" | cut -d ' ' -f 1
)"
STAGE4_FINAL_BUILD_EVIDENCE_SHA256="$(
  sha256sum "$STAGE4_RELEASE_BUILD_EVIDENCE" | cut -d ' ' -f 1
)"
bash "$STAGE4_FINAL_SOURCE_ROOT/install.sh" --offline \
  --prefix "$STAGE4_FINAL_INSTALL_PREFIX" \
  --source-root "$STAGE4_FINAL_SOURCE_ROOT" \
  --archive-file "$STAGE4_RELEASE_ARCHIVE" \
  --archive-sha256-file "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE" \
  --archive-sha256 "$STAGE4_FINAL_ARCHIVE_SHA256" \
  --build-evidence "$STAGE4_RELEASE_BUILD_EVIDENCE" \
  --build-evidence-sha256 "$STAGE4_FINAL_BUILD_EVIDENCE_SHA256"
STAGE4_FINAL_RELEASE_ROOT="$(readlink -f \
  "$STAGE4_FINAL_INSTALL_PREFIX/current")"
test -f "$STAGE4_FINAL_RELEASE_ROOT/install-state.json"
test -f "$STAGE4_FINAL_RELEASE_ROOT/relocation-state.json"
SLOPE_SIM_ROOT="$STAGE4_FINAL_RELEASE_ROOT" \
  "$STAGE4_FINAL_RELEASE_ROOT/bin/slope-sim" doctor \
    --json "$STAGE4_FINAL_EVIDENCE_DIR/doctor.json"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --run-installed-no-participant-smoke \
  --release-handoff "$STAGE4_FINAL_RELEASE_HANDOFF_FILE" \
  --installed-release-root "$STAGE4_FINAL_RELEASE_ROOT" \
  --install-state "$STAGE4_FINAL_RELEASE_ROOT/install-state.json" \
  --relocation-marker "$STAGE4_FINAL_RELEASE_ROOT/relocation-state.json" \
  --doctor-evidence "$STAGE4_FINAL_EVIDENCE_DIR/doctor.json" \
  --output "$STAGE4_FINAL_EVIDENCE_DIR/final-no-participant-smoke.json"
test -f "$STAGE4_FINAL_EVIDENCE_DIR/doctor.json"
test -f "$STAGE4_FINAL_EVIDENCE_DIR/final-no-participant-smoke.json"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --compare-accepted-payload \
  --accepted-candidate-context "$STAGE4_ACCEPTED_CANDIDATE_CONTEXT_FILE" \
  --final-release-handoff "$STAGE4_FINAL_RELEASE_HANDOFF_FILE" \
  --final-installed-release-root "$STAGE4_FINAL_RELEASE_ROOT" \
  --final-install-state "$STAGE4_FINAL_RELEASE_ROOT/install-state.json" \
  --final-relocation-marker \
    "$STAGE4_FINAL_RELEASE_ROOT/relocation-state.json" \
  --final-doctor-evidence "$STAGE4_FINAL_EVIDENCE_DIR/doctor.json" \
  --final-smoke-evidence \
    "$STAGE4_FINAL_EVIDENCE_DIR/final-no-participant-smoke.json" \
  --transaction-dir "$STAGE4_FINAL_EQUIVALENCE_TRANSACTION_DIR" \
  --output "$STAGE4_FINAL_EQUIVALENCE_OUTPUT" \
  --write-equivalence-handoff "$STAGE4_FINAL_EQUIVALENCE_HANDOFF_FILE"
test -f "$STAGE4_FINAL_EQUIVALENCE_OUTPUT"
test -f "$STAGE4_FINAL_EQUIVALENCE_HANDOFF_FILE"
```

Expected: 正式包先在全新仓库外 prefix 完成安全解包、安装、fresh doctor 和 Task 8 Step 2 同合同的 C++/Python/self-test/PCD/PLY/LVX2 无 participant smoke，且五份 final 输入全部存在并互绑后，比较器才允许原子创建 `accepted-payload-equivalence.json`。只有两个 source commit 的 diff 精确等于 verifier 内置的两个路径、两个路径均未进入 payload、候选冻结的 `functional_source_epoch` 被正式 A/B 共用、闭包外全部安装/运行/SDK/资源/随包教程及 self-test MCAP segment bytes 相同，并且六个归档内控制文件及外部 evidence/handoff 只按固定 DAG 确定性变化时 rc=0。比较器结构化解析并自底向上重算每条摘要边；候选 context 中冻结的五件套和显式 final 五件套各自绑定各自 resolved prefix、`relocation-state.json` 和 fresh doctor 实值，路径相关表示允许不同，去除这些已验证表示后的功能结果必须相同。

verifier 在 `STAGE4_FINAL_EQUIVALENCE_TRANSACTION_DIR` 的 sibling staging 中完整写入并 fsync `accepted-payload-equivalence.json` 与 `accepted-payload-equivalence.env`，再以一次目录 rename 同时发布。该 handoff 固定 accepted-candidate context、final release handoff、equivalence JSON 和 final installed root/state/marker/doctor/smoke 的绝对路径、SHA-256、安装树 identity digest 与共同 final run id；同时保留 candidate run id，不能把两个安装轮次误写成同一轮。它精确导出 `STAGE4_ACCEPTED_PAYLOAD_EQUIVALENCE`、`STAGE4_FINAL_INSTALLED_RELEASE_ROOT`、`STAGE4_FINAL_INSTALL_STATE`、`STAGE4_FINAL_RELOCATION_MARKER`、`STAGE4_FINAL_DOCTOR_EVIDENCE` 和 `STAGE4_FINAL_SMOKE_EVIDENCE`，并绑定但不得静默改写调用者指定的 accepted-context/final-handoff 路径。Step 7 只能先独立复核 committed transaction 再 source 这个 handoff，不能从当前 shell 拼回路径。缺失或篡改 candidate/final 任一 evidence、路径绑定错误、跨 run 混用、额外闭包路径/字段、断链、循环/自引用、非路径字段差异、稳定字段或功能 byte/smoke 语义变化，以及修改功能文件后同步重写所有摘要的伪造，都禁止发布 equivalence transaction 或继承候选真实 eCAL/GUI/干净机证据，必须对正式包重新执行 Task 7/8。

- [ ] **Step 7: 用正式 handoff 裁决最终状态**

调用者把 `STAGE4_FINAL_EQUIVALENCE_HANDOFF_FILE` 指向 Step 6 明确生成的 `accepted-payload-equivalence.env`，然后在新 shell 独立执行唯一 final-status gate：

```bash
test -n "${STAGE4_FINAL_EQUIVALENCE_HANDOFF_FILE:-}"
test -n "${STAGE4_ACCEPTED_CANDIDATE_CONTEXT_FILE:-}"
test -n "${STAGE4_FINAL_RELEASE_HANDOFF_FILE:-}"
test -n "${STAGE4_EXTERNAL_ACCEPTANCE_PARENT:-}"
test -d "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT"
git diff --check
STAGE4_FINAL_STATUS_ROOT="$(mktemp -d \
  "$STAGE4_EXTERNAL_ACCEPTANCE_PARENT/slope-sim-stage4-final-status.XXXXXX")"
STAGE4_FINAL_STATUS_TRANSACTION_DIR="$STAGE4_FINAL_STATUS_ROOT/final-status"
STAGE4_FINAL_STATUS_OUTPUT="$STAGE4_FINAL_STATUS_TRANSACTION_DIR/final-release-status.json"
STAGE4_FINAL_STATUS_HANDOFF_FILE="$STAGE4_FINAL_STATUS_TRANSACTION_DIR/final-release-status.env"
test ! -e "$STAGE4_FINAL_STATUS_TRANSACTION_DIR"
test ! -e "$STAGE4_FINAL_STATUS_OUTPUT"
test ! -e "$STAGE4_FINAL_STATUS_HANDOFF_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-equivalence-handoff "$STAGE4_FINAL_EQUIVALENCE_HANDOFF_FILE" \
  --output "$STAGE4_FINAL_STATUS_ROOT/equivalence-handoff-preflight.json"
source "$STAGE4_FINAL_EQUIVALENCE_HANDOFF_FILE"
test -f "$STAGE4_ACCEPTED_PAYLOAD_EQUIVALENCE"
test -d "$STAGE4_FINAL_INSTALLED_RELEASE_ROOT"
test -f "$STAGE4_FINAL_INSTALL_STATE"
test -f "$STAGE4_FINAL_RELOCATION_MARKER"
test -f "$STAGE4_FINAL_DOCTOR_EVIDENCE"
test -f "$STAGE4_FINAL_SMOKE_EVIDENCE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-handoff "$STAGE4_FINAL_RELEASE_HANDOFF_FILE" \
  --output "$STAGE4_FINAL_STATUS_ROOT/final-release-handoff-preflight.json"
source "$STAGE4_FINAL_RELEASE_HANDOFF_FILE"
(
  cd -- "$(dirname -- "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE")"
  sha256sum -c -- "$(basename -- "$STAGE4_RELEASE_ARCHIVE_SHA256_FILE")"
)
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --finalize-release-status \
  --accepted-candidate-context "$STAGE4_ACCEPTED_CANDIDATE_CONTEXT_FILE" \
  --final-release-handoff "$STAGE4_FINAL_RELEASE_HANDOFF_FILE" \
  --payload-equivalence "$STAGE4_ACCEPTED_PAYLOAD_EQUIVALENCE" \
  --final-installed-release-root "$STAGE4_FINAL_INSTALLED_RELEASE_ROOT" \
  --final-install-state "$STAGE4_FINAL_INSTALL_STATE" \
  --final-relocation-marker "$STAGE4_FINAL_RELOCATION_MARKER" \
  --final-doctor-evidence "$STAGE4_FINAL_DOCTOR_EVIDENCE" \
  --final-smoke-evidence "$STAGE4_FINAL_SMOKE_EVIDENCE" \
  --transaction-dir "$STAGE4_FINAL_STATUS_TRANSACTION_DIR" \
  --output "$STAGE4_FINAL_STATUS_OUTPUT" \
  --write-final-status-handoff "$STAGE4_FINAL_STATUS_HANDOFF_FILE"
conda run -n slope-sim python scripts/verify_stage4_release.py \
  --verify-final-status-handoff "$STAGE4_FINAL_STATUS_HANDOFF_FILE" \
  --require-status complete \
  --output "$STAGE4_FINAL_STATUS_ROOT/final-status-preflight.json"
git diff --check
```

Expected: `--finalize-release-status` 不信任前置 preflight 的结论，而是重新结构化验证 final archive/sidecar/build evidence、accepted context 中冻结的候选真实 eCAL/GUI/RViz2/Livox Viewer/干净机/六维审查证据、payload equivalence，以及 final root/state/marker/doctor/smoke 五件套的全部摘要与 run 绑定。只有全部成立时，才在 sibling staging 中写完并 fsync canonical `final-release-status.json` 与 shell-safe handoff，再以一次 directory rename 发布 `STAGE4_FINAL_STATUS_TRANSACTION_DIR`；JSON 的 `status` 精确为 `complete`，并固定所有输入的绝对路径、SHA-256、candidate/final run id 和 verifier schema version。缺失、篡改、陈旧 equivalence、错误 run、跨安装混用或任一 write/fsync/rename 故障时 rc 非零且没有可消费的 final transaction；rename 后 parent fsync 失败只能在锁内完整重验并补 fsync，孤立成员绝不恢复为成功。

只有 `--verify-final-status-handoff --require-status complete` fresh 通过后，才在交付报告与 README 引用该 final-status JSON/handoff 的绝对路径和 SHA-256 并记录“完成”；否则保持“部分完成”并列出缺口。报告和 README 是 final status 的下游展示，不作为其输入，避免摘要自引用。此处只允许更新这两个不进入 payload 的外部状态文档；更新后再次运行 `git diff --check`。如需提交该状态更新，必须再次向用户请求 Git 授权，但无需重建已经证明功能等价的正式 archive。
