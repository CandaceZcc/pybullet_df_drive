# Plan

本计划收口 MID-360 Golf 高保真采集与三维回放里程碑，并对齐阶段四剩余的
LVX2、默认回归、发行和文档证据。当前 Golf Task 1-7 的生产实现与聚焦测试已经
落地，`mid360-golf-mapping-acceptance-v2` 正式结果为 `passed=true`；后续只处理
真实欠账，不恢复已暂停的旧架构，也不重复实现第二条 Golf 链。

## Scope

- In: README/需求/架构/阶段四报告和计划状态对账；实时 LiDAR 性能失败复验；最终
  默认回归；同一 v2 会话的真实 GUI/回放 QA；LVX2 显示与退出风险定级；包含 Golf
  的新版本 `.run` 及干净安装；最终文件边界与交付证据核对。
- Out: 旧 7 月 31 日计划中的离线 cache、断网构建、双根复现、`.tar.zst` 和跨机证据；
  8 月 13 日 A4/A5 的重复 Golf 链；放宽实时 LiDAR `100 ms` 门限；自动导航、SLAM、
  真实 GNSS/MID-360 光学数字孪生；未经用户明确要求的 commit、push 或公开发布。
- Delegation: `gpt-5.6-sol`（xhigh）负责需求/架构状态对账、疑难根因和受影响维度只读
  审查；`gpt-5.6-terra`（high）负责边界明确的文档或聚焦验证任务。主 agent 负责集成、
  GUI/eCAL/发行串行门和最终判断；最多四个并发槽，同一文件、GUI、eCAL 会话或构建根
  同时只允许一个执行者。

## Action items

[x] 对账 `README.md`、`3d仿真平台需求规格.md`、`ARCHITECTURE.md`、
`docs/阶段四交付报告.md`、8 月 13 日收尾计划和 8 月 16 日 Golf 计划：补正式 Stage 4
v2/Golf 入口；把已经落地的 `LiDAR_Type=8` 与 Viewer 非空显示从“未执行”改为真实状态；
将旧 A4/A5 明确标记为由 8 月 16 日方案取代；不得改写历史失败证据。

[x] 持久化当前里程碑证据：记录 acceptance v2 的路线 P95 `0.06433634581408047 m`、
终点剩余 `0.07440749727453522 m`、运动/地形覆盖 `1.0`、deskew P95 上界 `0.008 m`、
五 topic 冻结计数、MCAP SHA-256、RED/GREEN 命令、GUI v1 与最终 DIRECT v2 会话分裂，
以及已完成六维审查的两个 Important finding 和针对性复验。

[x] 在宿主空闲、无并行 GUI/eCAL/构建任务时恢复原完整回归失败清单，先只运行其中的
实时 LiDAR `100 ms` 性能 nodeid；保留实际时延和宿主负载证据，不安装/升级依赖、不改
门限。若仍失败，按系统化调试区分代码回归与调度污染，并把无法关闭的环境风险写入报告。

[x] 仅在聚焦性能门完成且工作树进入最终快照后，运行一次 README 默认回归：
`conda run -n slope-sim python -m pytest -q -m 'not ecal and not stage4_artifact'`。完整读取
退出码和失败数；不得用历史通过数、skip 或局部回归冒充全绿，也不得在未改工作树上机械
重复整套测试。

[x] 若要求单一最终端到端证据，使用新的独占结果根串行运行公开 Golf GUI 入口和自动回放
QA，使 acceptance v2、MCAP、双 OpenGL 截图、非空像素、播放/暂停、逐帧、定位、倍率、
回退重建和正常关闭绑定到同一 simulation session。保留现有 v1 QA 与失败目录，不覆盖、
删除或改名历史制品。

[x] 收口 LVX2 状态：把 profile8 已有的非零 packet、`PointsNum=2538`、可见点云和播放到尾
记录为显示 PASS；把 Livox Viewer 2 退出时 `SIGSEGV/rc139` 单独标为 clean-shutdown
concern。默认将其作为专有外部 Viewer 的残余风险，不阻塞项目内回放；只有用户把 clean
shutdown 定为正式阻塞时，才启动新的聚焦调查。不得恢复旧 A4/A5 或通过补假点迁就 Viewer。

[x] Golf 已按同版本不可漂移规则升至 `4.0.1`，以唯一 canonical
构建根和缓存生成包含 Golf 入口、模块、场景及 `mcap/pyqtgraph/PyOpenGL` 的唯一 `.run`，
并在新安装根执行 files doctor、核心命令与 mapping import/启动 smoke。预计新增写入超过
`5 GiB` 时，先报告估算、复用失败原因、保留/清理方案并等待专项授权；未获授权前停在
只读 preflight，不下载、不构建、不创建第二个完整根。

[x] 汇总最终状态并复用本阶段已完成的六维审查；发行 payload 的新增 Golf 入口由聚焦
受影响维度启动一次独立只读复审。最后核对所有新增、修改、删除和未跟踪文件的所有权，
但只有用户明确要求 `git commit` 或 `push` 时才执行 Git 写操作。

## Open questions

- Golf mapping/replay 是否必须进入下一版正式 `.run`？已确认：是；manifest 已升至 `4.0.1`，
  新 `.run` 已在 `/tmp/stage4-golf-4.0.1-clean-install/` 完成联网干净安装、files doctor、
  安装后核心 Command dry-run 与 Golf mapping import/CLI smoke。
- Livox Viewer 2 的退出 `rc139` 是发布阻塞还是已记录残余风险？建议：作为残余风险，不阻塞
  项目内双三维回放。
- 是否要求 acceptance v2 与真实 GUI QA 必须来自同一 simulation session？已确认：必须；
  `mid360-golf-mapping-release-qa-v3` 的 acceptance 与 GUI QA 已绑定同一 session。
