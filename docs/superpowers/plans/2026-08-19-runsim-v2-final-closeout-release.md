# runSim v2 最终收口与发行计划

本计划完整接续未完成的 runSim v2 工作，覆盖 eCAL、Command、WheelState、中心 LiDAR、RTK、IMU、Recorder、ROS/RViz、离线导出、Viewer 检测、性能修复、仓库收口和最终 `.run` 交付。

## Scope

- In: 完整 v2 实时数据链、Dashboard 状态与控制、采集和导出、手测问题修复、简化 Shell 启动入口、自定义运行参数、README 使用说明、依赖检测、仓库清理、Git 提交、包含 ROS/RViz 的最终 `.run`。
- Out: 内嵌 Livox Viewer 2、保留或打包 `results/`、真实 MID-360 硬件驱动、SLAM/回环优化、通过降低射线数量或传感器频率解决卡顿。

## 模型管理与委派规则

### 模型职责

- `sol high`：只用于架构设计、执行计划、跨 Python/C++/eCAL/ROS 的边界决策，以及大阶段的架构维度终审。禁止用于常规编码、TDD、测试补充、清理、`.gitignore`、Git 操作和打包执行。
- `terra high`：执行阶段允许使用的最高等级，只用于跨进程控制、eCAL/ROS/Viewer 集成、性能卡顿、安全边界和多模块疑难故障；不得因任务重要但简单而默认使用。
- `terra medium`：默认主力模型，用于普通功能实现、Dashboard、采集命名、量程调整、安装检测、清理、打包脚本和常规代码审查。
- `luna high`：若当前环境可用，用于聚焦 TDD、测试夹具、静态检查、日志归类和简单单线程任务；不可用时使用较低 reasoning effort 的 `terra`，不得为此升级到 `sol`。

### 升级与降级

1. 每个子任务先按最低足够模型分配；测试和机械任务从 `luna high` 或低 effort `terra` 开始，普通实现使用 `terra medium`。
2. 只有出现可复现的跨进程、并发、原生 GUI、性能或安全复杂性时，才把该子任务升级到 `terra high`，并在进度记录中写明原因。
3. 只有缺少架构决策、模块责任边界不清或计划需要重排时，才暂停实现并调用 `sol high`；架构决定完成后，执行必须交回 `terra` 或 `luna`。
4. 同一低等级模型对同一根因连续两次无有效进展时停止盲试，先保存日志和最小复现，再按上述条件升级一级。

### 并发与审查

- 同一时刻最多一个 agent 修改同一文件、同一功能链或同一 C++ 构建根；不同模块的只读调查和独立测试可以并行。
- 实现 agent 不负责最终批准自己的改动；普通只读审查使用 `terra medium/high`，仅架构维度终审允许 `sol high`。
- 每个委派任务必须写明模型、reasoning effort、文件边界、禁止修改范围、RED/GREEN 命令和交付条件；主会话负责集成与最终回归。
- 不因上下文过长自动升级模型；优先记录检查点、缩小任务边界或新开会话，并从本计划继续。

### 本计划任务映射

- runSim/eCAL/Command/Recorder/ROS 编排：`terra high` 实现，聚焦测试使用 `luna high` 或 `terra medium`；仅发生边界争议时调用 `sol high`。
- Viewer 自动导入和 2 m/s 顿挫：`terra high` 负责原生 GUI/性能根因，日志整理与回归测试使用 `terra medium` 或 `luna high`。
- 采集日期命名、量程扩大、Dashboard 文案、依赖检测：默认 `terra medium`，TDD 使用 `luna high` 或低 effort `terra`。
- Shell launcher、CLI 参数合同和 README：默认 `terra medium`；参数转发、安装路径和帮助文本测试使用 `luna high` 或低 effort `terra`。
- 文件清理、`.gitignore`、Git 暂存审计、安装器和打包：`terra medium`；哈希、manifest、安装测试等机械验证使用 `luna high` 或低 effort `terra`。
- 最终六维审查：逻辑、测试和交付审查使用 `terra high`；其中架构维度可单独使用一次 `sol high`，不得让 `sol` 重跑实现或测试。

## Action items

- [ ] 冻结当前检查点并执行模型门禁：记录 Task 1–6 已实现、Task 7 仅完成启动器/eCAL 预检，统一编排和真实桌面验收仍未完成；每次委派前按“模型管理与委派规则”登记模型、理由、文件边界和验收命令。

- [ ] 完成 runSim v2 进程编排：让 `runSim` 默认启动同一个 PyBullet 世界、Dashboard、v2 Simulator、唯一 C++ Command 和受监管本机 socket；禁止回退 local/v1，禁止同时发布旧前后 LiDAR topic。进程退出、窗口失焦、socket 断开和 Command 超时必须立即发布零命令，并在 Dashboard 显示具体状态。

- [ ] 编写并安装简洁 Shell 启动入口：保持无参数 `runSim` 作为完整默认启动命令，脚本只负责定位安装根、加载受控环境并用 `exec` 和 `"$@"` 原样转发参数，不硬编码用户主目录或 Conda 环境。提供 `runSim --help`、`--version`、`--config <yaml>`，以及车型、场地、速度、采集时限、输出目录、ROS/RViz 开关和本机 Viewer 路径等常用自定义参数；统一规定“命令行参数 > 配置文件 > 安全默认值”的优先级。`.run` 安装器把 launcher 安装到所选 prefix 的 `bin/`，检测 PATH，缺失时只提示一条可复制的配置命令。使用 `tests/unit/test_run_sim_launcher.py` 和安装器测试覆盖路径含空格、参数引用、退出码、信号转发、缺失依赖、重复安装和自定义配置，禁止重新引入含义混乱的旧 `runSim --lidar` 特殊入口。

- [ ] 打通并验证 eCAL 五话题：正式运行 `/sim/wheel/command`、`/sim/wheel/state`、`/sim/lidar/points`、`/sim/rtk/state`、`/sim/imu/attitude`，显示 peer 数、协议验证、实际频率、sequence、drop/error、session、descriptor 和 world generation。修复 RTK、IMU、前述传感器长期显示“等待”的问题；只有收到同会话且 identity 正确的数据才能显示“运行中”。

- [ ] 完善 eCAL 环境预检和安装配置：为 release 提供有效 `ecal.yaml`、time-sync plugin 路径、descriptor 和 participant 配置；终端与 Dashboard 同时报告缺失项及恢复操作。安装器先检测本机兼容的 eCAL/插件，版本和摘要满足锁定合同则复用，否则才安装包内版本，禁止重复安装或混用不兼容库。

- [ ] 完成人工采集、日期命名和原子导出：采集目录使用本地日期时间，如 `capture-20260819-143012`，同秒冲突加确定性后缀。Dashboard 的“启用采集/结束采集”驱动真实 C++ Recorder，在完整五 topic 边界开始和结束；成功后自动运行 Export，生成 MCAP、PCD、PLY、LVX2 和 manifest，并保存“最近一次成功导出”的绝对 LVX2 路径。

- [ ] 完成 Livox Viewer 2 本机检测与自动打开：`.run` 不内嵌 Viewer；安装和首次启动时检测常见安装位置及可执行文件版本，允许用户补充路径并持久化。点击“导入 Livox Viewer”必须使用最近一次成功导出的绝对 LVX2，自动启动 Viewer、打开文件、选择模拟 MID-360 设备并开始播放；以 `OpenLvxFile success`、设备选择、播放和非零渲染日志作为成功判据，不能只弹出目录选择框。

- [ ] 扩大中心 MID-360 采集范围 50%：将实时 worker、离线重建、导出元数据和可视化范围统一调整为当前最大量程的 `1.5×`，保持 5,760 条候选射线、10 Hz、角密度和导出质量不变。增加远距离目标、边界命中、PCD/PLY/LVX2 坐标范围和 Viewer 非空显示回归。

- [ ] 定位并修复 2 m/s 驾驶顿挫：持续记录 GUI 事件间隔、socket target cadence、Command 100 ms 租约、WheelCommand sequence、物理步、LiDAR worker、Dashboard 绘制和零命令插入。重点验证持续按键期间是否因焦点、按键重复或续租间隙周期性停车；增加“按住方向键期间命令连续且不插零”的自动回归，不降低传感器频率、射线数量或显示质量。

- [ ] 接通 ROS Bridge/RViz 并实现依赖复用：`.run` 同时携带锁定的 ROS/RViz 安装组件和 `stage4_live.rviz`，安装时先检测本机 ROS 2 Jazzy、RViz2、消息包和依赖版本，满足合同则复用，缺失部分才安装。Dashboard 可独立打开、关闭和重启实时点云；验证 `world -> base_link -> lidar_link` TF、固定 `world`、有界 queue/decay，以及 Bridge/RViz 崩溃不会影响 Simulator、Command 或 Recorder。

- [ ] 重写 README 的安装与运行章节：把 `runSim` 放在最前面的快速开始中，分别给出默认启动、选择车型和场地、2 m/s 测试、采集 1/1.5/3 分钟或不限时、关闭或开启 ROS/RViz、指定 Viewer 路径、指定输出目录和使用 YAML 配置的可复制示例。说明 Dashboard 的 eCAL/RTK/IMU/LiDAR 状态、采集文件按日期时间保存的位置、自动导入 Viewer 的成功判据、依赖检测与复用规则、`results/` 不随包交付，以及 eCAL、Viewer、RViz、X11 和性能故障的排查步骤。README 命令必须由自动测试实际执行 `--help` 或 dry-run 合同，防止文档与 launcher 漂移。

- [ ] 清理、忽略、提交并打包：审计全部 tracked、deleted 和 untracked 文件，保留正式源码、测试、锁文件、配置和必要文档；删除确认无调用方的旧构建、崩溃文件、缓存、临时 CMake 文件和重复证据。将整个 `results/` 视为本地运行输出并加入 `.gitignore`，从最终 payload 完全排除；运行聚焦回归、C++ CTest、Shell/README 命令合同、真实 eCAL、四车型×三场地、Golf+20 障碍物+2 m/s、Recorder/Export、Viewer 和 ROS on/off 验收。审查暂存内容后创建规范 Git commit，再从干净 commit SHA 使用 `scripts/build_stage4_run.py` 生成最终 `.run`，完成全新目录安装、PATH/依赖复用、`runSim` 启动与自定义参数、采集、导出、Viewer、RViz、卸载和 SHA-256 验证。

## Open questions

- 无；Livox Viewer 2 仅检测复用，`results/` 全部排除，ROS/RViz 随 `.run` 提供并优先复用本机兼容安装。
