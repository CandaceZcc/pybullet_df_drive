# 阶段四恢复检查点

> 写入时间：2026-08-07（设备重启前）
> 分支：`agent/stage3-final-acceptance`
> 当前阶段：阶段四 Task 2；尚未进入 A-E 子计划。

## 重启前状态

所有源码、测试、锁文件和计划修改都已落盘，但尚未提交、推送或发布。工作区还包含用户和此前任务的既有未提交改动；重启后必须保留它们，禁止 `git reset`、`git checkout --`、`git stash` 或删除 `build/`、`results/`。

当前没有需要等待完成的 producer、测试或 eCAL 进程，可以直接重启。

## 已完成：私有 Protobuf Conda producer

- `scripts/build_private_protobuf_conda.py` 已在最终 `conda index` 前接入 `scripts/canonicalize_private_conda_package.py`，消除 Conda 自动 metadata 的可变字段。
- 一个完整 TDD 单元已收口：锁定 Conda-build toolchain 内的 `python -> python3.10` 软链接现被安全接受，但 `conda` 与 `conda-build` 仍要求普通可执行文件。
  - RED：`test_private_protobuf_builder_canonicalizes_output_before_channel_index` 失败于预期断言 `conda-build Python must be an executable regular file`。
  - GREEN：同一聚焦测试 `1 passed`。
  - 回归：
    ```bash
    conda run -n slope-sim python -m pytest -q \
      tests/stage4/test_python_offline_runtime.py -k private_protobuf
    ```
    结果：`26 passed, 39 deselected`。
- 两个全新、网络隔离的真实 producer 已完成。两轮都在最终 `conda index` 后通过，且 network evidence 确认仅 loopback、无默认路由、TEST-NET 连接为 `ENETUNREACH`：

  | 轮次 | work | channel | network evidence |
  | --- | --- | --- | --- |
  | A | `build/stage4-private-protobuf-repro3-a-work-20260807T162547+0800` | `build/stage4-private-protobuf-repro3-a-channel-20260807T162547+0800` | `results/stage4/private-protobuf-repro3-a-network-20260807T162547+0800/network-isolation.json` |
  | B | `build/stage4-private-protobuf-repro3-b-work-20260807T163916+0800` | `build/stage4-private-protobuf-repro3-b-channel-20260807T163916+0800` | `results/stage4/private-protobuf-repro3-b-network-20260807T163916+0800/network-isolation.json` |

  最终 package 完全一致：
  ```text
  protobuf-6.33.6-py310_0.conda
  sha256=4cd4a8a90e5960c38db7e39a453c768a53159140fb3a5e6bac6534a9ec2f8d78
  md5=2d130b9d9f4cb60d654440cca396c267
  size=612190
  ```
  已用锁定 Conda-build Python 解包确认包含 `google/_upb/_message.cpython-310-x86_64-linux-gnu.so`。

## 已完成：锁和新 canonical cache

- 旧 cache `build/stage4-python-package-cache-20260806T153300+0800` 保持不变。
- 新 canonical cache：`build/stage4-python-package-cache-20260807T165000+0800`，只替换 Protobuf archive，已确认它是 `nlink=1` 的普通文件。
- 已更新：`packaging/locks/python-package-cache.manifest.json`、`packaging/locks/python-linux-64.lock`、`packaging/locks/python.conda-lock.yml`。
- 新 cache verifier 已通过：
  ```bash
  conda run -n slope-sim python scripts/verify_python_lock_cache.py \
    --runtime-spec packaging/python-environment.yml \
    --toolchain-spec packaging/python-toolchain-environment.yml \
    --protobuf-build-spec packaging/python-protobuf-build-environment.yml \
    --virtual-packages packaging/locks/virtual-packages.yml \
    --runtime-unified packaging/locks/python.conda-lock.yml \
    --runtime-explicit packaging/locks/python-linux-64.lock \
    --toolchain-unified packaging/locks/python-toolchain.conda-lock.yml \
    --toolchain-explicit packaging/locks/python-toolchain-linux-64.lock \
    --protobuf-build-unified packaging/locks/python-protobuf-build.conda-lock.yml \
    --protobuf-build-explicit packaging/locks/python-protobuf-build-linux-64.lock \
    --cache-manifest packaging/locks/python-package-cache.manifest.json \
    --cache-root build/stage4-python-package-cache-20260807T165000+0800
  ```
  结果：`PASS: stage 4 Python unified and explicit locks verified`。

## 当前 TDD 单元：环境 probe

目标是让 `scripts/verify_stage4_dependencies.py --write-env` 不再依赖硬编码 `build/stage4-deps` / `build/stage4-validation-tools`，而是只接受调用方传入且逐项验证的 `--cmake`、`--ctest`、`--cc`、`--cxx`、`--protoc`、`--dependency-prefix` 和 `--pcl-pcd2ply`。

新测试 `test_dependency_verifier_write_env_exports_complete_explicit_probe_contract` 已完成：

- RED：新 CLI 参数未实现而失败，属于目标行为缺失。
- GREEN：
  ```bash
  conda run -n slope-sim python -m pytest -q \
    tests/stage4/test_stage4_dependencies.py \
    -k dependency_verifier_write_env_exports_complete_explicit_probe_contract
  ```
  结果：`1 passed, 73 deselected`。

生产实现已调用既有的 `build_environment_from_probe_inputs()` 与 `write_build_environment()`，并输出 `PASS: build environment written`。

8 个旧负例已迁移为完整的显式 probe 输入夹具，每个测试只破坏自己要拒绝的输入；两个历史硬编码路径负例已改为显式缺失 `--dependency-prefix` / `--pcl-pcd2ply`。为使错误输出与公开 CLI 一致，生产 verifier 现在统一将输入校验报为对应的 `--option` 名称，版本错误仍使用工具名。

- RED：完整文件为 `8 failed, 66 passed`，其中 6 项提前报 `cmake must be a regular executable`，2 项仍引用已删除 `_STANDARD_DEPENDENCY_PREFIX`。
- GREEN（聚焦）：`-k dependency_verifier_write_env_rejects` 为 `8 passed, 66 deselected`。
- GREEN（回归）：
  ```bash
  conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_dependencies.py
  ```
  结果：`74 passed`。
- Task 2 Python 全量回归：
  ```bash
  conda run -n slope-sim python -m pytest -q \
    tests/stage4/test_reference_manifest.py \
    tests/stage4/test_stage4_dependencies.py \
    tests/stage4/test_python_offline_runtime.py \
    tests/stage4/test_network_isolation.py
  ```
  结果：`163 passed`。

## 已完成：双根离线 Python runtime

- package cache、wheel cache 和 C++/ROS source cache 的实际 verifier 都已通过；新 canonical Protobuf cache 使用 `build/stage4-python-package-cache-20260807T165000+0800`。
- 双根 runtime 执行使用锁定 micromamba、全新 A/B work root、`run_network_isolated.sh` 和 `SOURCE_DATE_EPOCH=0`；每轮均从 canonical 嵌套 artifact 物化自己的 native cache/wheel 副本，不共享可写环境。
- 最终证据：`results/stage4/python-runtime-repro-20260807T171229+0800.json`。
  A/B 均为 `directories=3308`、`files=29140`、`links=1928`、`regular_bytes=1517674659`，并得到相同的
  `tree_sha256=690e7446ffa4c0b24be2c4f1c8ca1d9bbe9cab9361cd9c732120e34f348b9a08`。
- 对应网络隔离 evidence 位于：
  - `build/stage4-python-reproducibility-ax1yz6tx/runtime-a-network-isolation/network-isolation.json`
  - `build/stage4-python-reproducibility-ax1yz6tx/runtime-b-network-isolation/network-isolation.json`

## 重启后续跑顺序

1. 检查工作区：
   ```bash
   cd /home/cancade/pybullet_df_drive
   git status --short
   git branch --show-current
   ```
2. 检查既有 C++ dependency prefix、PCL validator、真实 MID-360 LVX2、Jazzy RViz2 和工具链路径；使用新的显式 `--write-env` 合同创建环境/JSON evidence。旧固定路径不得恢复。
3. Windows Chrome remote debugging 由用户启用后，才可按 `web-access` 核验和更新内部 Tailscale HTTPS channel。runtime lock 的内部 URL 仍为 `https://candace.tail39defd.ts.net:8443/linux-64/protobuf-6.33.6-py310_0.conda`；只允许内部传输，禁止 public repo/package。该步骤尚未执行。
4. 完成 references 检查、环境 probe 及其实际 C++/ROS 依赖验证；若输入缺失，保留失败 evidence 并继续不依赖它的项目。
5. Task 2 全部证据齐全后，启动独立只读六维审查；审查修复和复验通过后才进入 A-E。

## 重启安全边界

- `build/`、`results/`、运行时 cache 和私有 CA/证据一律不提交。
- 不创建公开仓库、公开 Conda channel、公开包或外部发布。
- 本检查点是唯一新增交接文本；它与源码、测试和锁在 Task 2 收口后再统一审查和决定提交边界。

## 本次重启前确认（2026-08-07 17:24 +0800）

- 当前分支仍为 `agent/stage3-final-acceptance`；上述源码、测试、锁文件和计划的未提交修改均已保留在工作区。
- 已检查进程表：没有残留的 `pytest`、Conda build、阶段四 verifier 或 eCAL runtime 进程，重启不会中断正在运行的任务。
- 恢复入口就是本文件的“重启后续跑顺序”。恢复时先执行其中第 1 步，随后继续 Task 2 的真实环境 probe；不要重新生成既有 A/B producer、cache 或 runtime 证据，除非后续验证明确要求。

## 宿主预检复现（2026-08-07，重启前）

- `build/stage4-ubuntu24-private-pcl-daemon-help-20260807T143157+0800/validation-prefix/bin/pcl_pcd2ply` 的 `RUNPATH` 为 `$ORIGIN/../lib`，其 PCL 库确实解析到同一 validation prefix；不是 RPATH 或误用系统 PCL。
- `ldd` 仍显示唯一未解析依赖为 `libboost_filesystem.so.1.83.0 => not found`，实际执行 `pcl_pcd2ply --help` 也以该动态加载错误退出。锁定包 `libboost-filesystem1.83.0:amd64=1.83.0-2.1ubuntu3.2` 未安装；`libboost-iostreams1.83.0` 已安装。不得以 `LD_LIBRARY_PATH`、复制系统库或放松检查绕过。
- `/opt/ros/jazzy/bin/rviz2` 不存在。工作区范围的 `*.lvx2` 搜索无结果，尚未取得 D 计划锁定 SHA-256 为 `f892732ff43882b56d1cebc683f6ea9374ab3d3ac688368c9d560f49dcd4d647` 的官方 MID-360 样例。
- 因此真实 `--write-env` 仍不得伪造成功。下一次推进需要：用户授权安装锁定 Boost 包，并提供/安装 Jazzy RViz2 与合法官方 LVX2 样例；取得后再用所有真实绝对路径生成全新的 environment/evidence 文件。

## 环境 probe：系统 DSO 合同（2026-08-07 17:51 +0800）

- `scripts/verify_stage4_dependencies.py --write-env` 现在额外要求显式 `--system-lock`、`--ldd` 和 `--dpkg-query`。它先运行 PCL validator 的动态依赖检查：私有 prefix 库不进入宿主 package 检查；解析到 `/lib` 或 `/usr/lib` 的每个 SONAME 都必须被 `ubuntu24-system-dependencies.lock` 白名单允许，并由对应 package 的精确已安装版本提供。任何 `ldd` 非零、`not found`、未知 SONAME、未安装 package 或版本漂移均拒绝写入环境。
- 环境 evidence 升为 schema v2。JSON 除受限 shell environment 与其 SHA-256 外，还保存已核验的 `system_dependencies`（SONAME/package/version）；`--verify-env` 会拒绝不含该字段或字段结构无效的证据。
- TDD：
  - RED：`test_system_dependency_verifier_records_exact_locked_soname_package_versions` 因缺少 verifier 失败；GREEN：同一测试通过。
  - RED：`test_pcl_system_dependency_verifier_ignores_private_prefix_but_checks_system_sonames` 因缺少 PCL DSO verifier 失败；GREEN：两项系统 DSO 测试合并为 `2 passed, 74 deselected`。
  - RED：`test_build_environment_from_probe_inputs_rejects_pcl_system_package_version_drift` 因新输入尚未被公开 API 接受而失败；GREEN：同一测试通过。
  - RED：`test_build_environment_evidence_binds_sourceable_assignments` 因 schema v2 `system_dependencies` 参数尚未实现而失败；GREEN：同一测试通过。
  - 回归：`conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_dependencies.py` 为 `77 passed`；四文件 Task 2 回归为 `166 passed`。
- 实际 frozen 输入复核：package lock/cache verifier、wheel cache verifier 与 15 个 source archive verifier 均 PASS；`bash scripts/sync_references.sh --check` 通过全部 13 个 checkout。

## 外部前置项复核（2026-08-07，Boost 已修复）

- 用户已安装 `libboost-filesystem1.83.0:amd64=1.83.0-2.1ubuntu3.2`。PCL validator 的 `ldd` 现将 `libboost_filesystem.so.1.83.0` 解析到 `/lib/x86_64-linux-gnu/`，且 `pcl_pcd2ply --help` 成功；该宿主阻断已解除。
- Windows `Candace` Tailscale 节点在线且 ping 正常，但其 `9222` TCP 端口拒绝连接；本机 `web-access` CDP 检查也没有发现本地 Chrome 调试端口。Windows Chrome 的 remote-debugging 尚未暴露给 Linux 运行端，不能据此访问私有 HTTPS channel，也不能通过受控浏览器获取 ROS 官方安装输入或 MID-360 样例。
- Windows 的 Tailscale SSH 端口 `22` 同样拒绝连接，因此当前 Linux 端没有可用于代为启动 Chrome、Tailscale Serve 或 PowerShell 的远程执行通道。
- `/opt/ros/jazzy/bin/rviz2` 和官方锁定 SHA-256 的 MID-360 `.lvx2` 仍缺失。当前 APT sources 未提供 `ros-jazzy-rviz2` candidate；不得绕过 web-access 直接添加 ROS 源或下载样例。

## Windows channel 与外部来源复核（2026-08-07，CDP 已恢复）

- Windows Chrome remote debugging 已在 `127.0.0.1:9222` 返回有效 DevTools metadata，Windows Tailscale TCP Serve 仅向 tailnet 转发该端口。Linux 侧通过仅绑定 `127.0.0.1` 的临时 `socat` 转发接入；CDP 代理只创建并关闭后台标签，未操作用户原有标签。
- 已只读核验 Windows 交付目录 `C:\Users\Candace\Documents\Codex\2026-08-05\stage4-conda-channel\outputs\stage4-private-protobuf-channel`：它包含 `data/`、`tls/`、`channel_server.py`、`manage-channel.ps1` 和 README。README 规定 HTTPS 服务直接绑定 `100.102.188.81:8443`，重启命令为 `.\manage-channel.ps1 Restart`，不使用 Funnel 或公开仓库。
- `logs/channel-server.out.log` 证明该服务上一轮曾向 Linux Tailscale IP 返回目标 package 和两个 repodata 的 HTTP 200；但本次实时 HTTPS 复测得到连接拒绝，因而旧日志不能替代当前证据。必须在 Windows 执行上述 Restart 后，以私有 CA 重新校验 TLS、两个 repodata、`protobuf-6.33.6-py310_0.conda` 的大小和 SHA-256 `4cd4a8a90e5960c38db7e39a453c768a53159140fb3a5e6bac6534a9ec2f8d78`。
- 已通过官方 ROS 2 Jazzy Ubuntu deb 文档确认 Ubuntu 24.04 的支持状态及安装路径：先安装官方 `ros2-apt-source`，再安装 `ros-jazzy-desktop`（含 RViz）。尚未执行任何 APT 写操作，仍需用户可交互的 sudo 授权与新鲜安装/`rviz2` 运行证据。
- 使用官方 Livox GitHub 入口和公开检索均未把计划冻结的 LVX2 SHA-256 `f892732ff43882b56d1cebc683f6ea9374ab3d3ac688368c9d560f49dcd4d647` 反查到可下载的官方样例；该样例仍是未解决的外部门禁，禁止用空文件、伪造样例或未验证下载替代。

## Private channel 制品不一致（2026-08-07）

- 实时 HTTPS 下载证据 `results/stage4/private-conda-channel-live-20260807T190950+0800/` 表明 Windows channel 当前提供的同名 Protobuf package 是 `size=618376`、SHA-256 `6b9ea864223664c2d842df745cd22561c2e843c3693768dee7257fcd5ebf0a2c`，其 repodata 也声明了相同的旧摘要。该制品与冻结 lock 不一致，必须 fail closed；不得回退 lock 或以旧日志替代。
- 正确来源已从 `build/stage4-private-protobuf-repro3-a-channel-20260807T162547+0800/` 和 canonical cache 交叉验证：`size=612190`、SHA-256 `4cd4a8a90e5960c38db7e39a453c768a53159140fb3a5e6bac6534a9ec2f8d78`，且 `linux-64/repodata.json` 精确绑定该 size/SHA-256。
- 已通过受控 Windows Chrome 将仅含正确 package、两个 repodata 与回滚脚本的内部更新包下载到 `D:\下载\stage4-channel-refresh-20260807.zip`。脚本在切换前重新校验 package/repodata，保留旧 `data` 目录，并在 service 重启失败时回滚。CDP 不具备也不应绕过 Windows PowerShell 的 OS 命令执行权限；待用户执行一次脚本后，立即重新进行 TLS/repodata/package 验收。

## Task 2 外部输入新鲜复核（2026-08-09）

- Windows 私有 Conda channel 已由用户应用刷新包并重启服务；新鲜 TLS/repodata/package 验收通过。`protobuf-6.33.6-py310_0.conda` 为 `size=612190`、SHA-256 `4cd4a8a90e5960c38db7e39a453c768a53159140fb3a5e6bac6534a9ec2f8d78`。旧摘要 `6b9ea864...5ebf0a2c` 不得再使用。
- 本地 `/home/cancade/Downloads/Livox-MID360-reference/Indoor_sampledata.lvx2` 是普通文件且 `nlink=1`，大小 `222540611`，SHA-256 为计划冻结值 `f892732ff43882b56d1cebc683f6ea9374ab3d3ac688368c9d560f49dcd4d647`。同目录 `SOURCE.md` 记录 Livox 官方下载 URL、LVX2 签名与 Mid-360 device type；只作内部验证输入，不纳入 Git 或发行包。
- 已重新通过：reference 同步 13 个 checkout、Python package cache verifier、wheel cache verifier、15 个 source archive verifier，以及 Task 2 pytest 回归 `204 passed`。
- 已确认的正式 probe 输入包括 `/usr/bin/cmake`、`/usr/bin/ctest`、`/usr/bin/x86_64-linux-gnu-gcc-13`、`/usr/bin/x86_64-linux-gnu-g++-13`、`build/stage4-python-producers/micromamba.JFOCPK/extracted/bin/micromamba`、`build/stage4-python-package-cache-20260807T165000+0800`、`build/stage4-python-wheel-cache-20260805T172013+0800`、`build/stage4-source-archive-cache-v9`，及 `build/stage4-ubuntu24-private-pcl-daemon-help-20260807T143157+0800` 下的 C++ prefix/PCL validator/protoc 33.6。PCL validator 和冻结系统 DSO 依赖均通过。
- 完整 `--write-env` preflight 已 fail-closed，只剩 `/opt/ros/jazzy/bin/rviz2` 缺失；两次无效 GCC 符号链接输入和这一次 RViz2 缺失都未留下 environment/evidence 半成品。下一步只需以交互 sudo 安装官方 ROS Jazzy desktop，然后对同一组输入重新运行一次 probe 和 `--verify-env`。

## 阶段四官方 reference 复核（2026-08-09）

- 通过受控浏览器逐个打开官方 GitHub commit 页面，确认 `eclipse-ecal/ecal@e9ca7cf`、`protocolbuffers/protobuf@ea6ec8d`、`foxglove/mcap@58db443`、`facebook/zstd@5c7b7ba`、`Livox-SDK/livox_ros_driver2@13eb05e`、`Livox-SDK/Livox-SDK2@68ae1e1` 和 `PointCloudLibrary/pcl@6a4f535` 均是各官方组织公开可访问的精确提交。
- Livox Driver 2 固定提交的官方变更说明明确包含 Mid-360、Ubuntu 24.04 与 ROS 2 Jazzy；Livox-SDK2 固定提交明确包含 Mid-360 和 Ubuntu 24.04 build 修复。reference 的固定 SHA 仍是阅读快照，构建源码继续只由独立 dependency lock 与 canonical archive cache 消费。
- 本地 `bash scripts/sync_references.sh --check` 已验证 13 个 checkout 的 origin/HEAD，并对阶段四条目的 first-party 与 third-party license 文件及 focus 路径逐项做 Git object 和无 symlink 物化检查；未写回 manifest 或移动任何 checkout。

## Task 2 构建环境合同 GREEN（2026-08-09）

- 用户已安装 `ros-jazzy-desktop=14.1.22-1noble.20260615.174609` 与 `ros-jazzy-rviz2=0.11.0-1noble.20260616.084553`。`/opt/ros/jazzy/bin/rviz2` 是 mode `0755` 的普通文件。
- 裸运行 RViz2 会因未 source ROS 环境缺 `libOgreMain.so.1.12.10`；source `/opt/ros/jazzy/setup.sh` 后该库问题消失，当前无 DISPLAY 则由 Qt xcb 正常 fail。Task 2 只冻结可执行绝对路径，真实 GUI/RViz2 门保留到 D 计划，不能以该 headless 探针冒充 GUI 通过。
- 全部真实绝对路径的环境 probe 已成功写入并立即验证：`results/stage4/task2-build-environment-20260809T190953+0800/stage4-build-env.sh` 与配对 schema v2 `stage4-build-env.json`。`scripts/verify_stage4_dependencies.py --write-env` 与 `--verify-env` 均 PASS；evidence 包含受限 shell payload digest 与实际已锁定的系统 DSO package/version。

## Task 2 环境合同实时复核 GREEN（2026-08-09）

- 独立六维审查发现旧 schema v2 仅绑定 shell/JSON 自身，无法阻止工具、缓存、LVX2 或系统 DSO 在后续 A-E 前发生漂移。该 schema 已被 verifier 拒绝，不能再作为可执行环境合同。
- 已以 TDD 升级 schema v3：`--verify-env` 每次重新执行显式工具/PCL/system package probe，并精确比较工具、样例和三类 cache 的文件或目录树身份；私有 dependency prefix 只允许可解析且留在 prefix 内的 ELF SONAME 链接。删除 micromamba、package cache、LVX2 或改变 dpkg package version 的四项 RED 均会失败，GREEN 为 `4 passed`。
- 新鲜真实合同已写入并即时复验：`results/stage4/task2-build-environment-v3-20260809T192208+0800/stage4-build-env.sh` 与配对 `stage4-build-env.json`。两条命令均 PASS；该目录只作本机证据，保持不提交。
- P1 清零：PCL validator 的所有非系统 DSO 现在必须解析到本轮 `dependency-prefix` 或由固定 `validation-prefix/bin/pcl_pcd2ply` 布局派生的 validation prefix；未知 `/tmp` DSO RED，合法 validation-prefix PCL DSO GREEN。真实 v3 `--verify-env` 在该限制下仍为 PASS。
- P2 覆盖补齐：原地改写 micromamba、package cache 内容、LVX2、PCL validator、RViz2、`ldd` 或 system lock 的七项 identity drift 测试均通过。收口回归命令为 `conda run -n slope-sim python -m pytest -q tests/stage4/test_stage4_dependencies.py tests/stage4/test_python_offline_runtime.py tests/stage4/test_network_isolation.py`；随后运行 `python -m py_compile scripts/verify_stage4_dependencies.py` 与 `git diff --check`，均为零退出。
- 最终复审的两项 P1 已以 TDD 清零：不含 `=>` 的 `/tmp/libunknown.so` 形式 `ldd` 行现在和普通依赖一样 fail-closed；validation prefix 的完整目录树也已纳入 v3 runtime identity，原地改写被 PCL validator 解析的私有库会拒绝合同。两项 RED 后聚焦 GREEN 为 `2 passed`，环境合同模块回归为 `93 passed`。
- 最新真实合同为 `results/stage4/task2-build-environment-v3-validation-20260809T194020+0800/stage4-build-env.sh` 及配对 JSON；`--write-env` 与立即 `--verify-env` 均 PASS。该 evidence 目录只作本机验证，保持不提交。
