# 阶段四 E 旧计划状态索引

> 状态：未开始（原计划 0/67），旧方案已被替代
>
> 原详细计划不可执行，本文件只记录迁移后的交付范围

## 当前交付范围

- 唯一文件：
  `slope-sim-stage4-<version>-ubuntu24.04-amd64.run`。
- 安装时联网下载已锁定依赖，可请求 sudo；下载和构建使用普通用户。
- 版本化 `/opt/slope-sim/releases/<version>` 与原子 `current` 切换。
- 全局安装锁、每轮唯一 staging、失败不切换；同版本校验 payload、安装选项和
  文件，完整未激活时只补做 `current` 切换，损坏或选项漂移时不覆盖。
- 版本列表、回退、卸载非当前版本和 XDG 用户配置/数据保护。
- Simulator、Command、Subscriber、Recorder 的真实联合负载。
- 一台干净 Ubuntu 24.04 amd64 电脑的联网安装与 smoke。
- 可选 ROS/RViz2 smoke 和最终独立六维只读审查。

## 已取消的旧方案

- `.tar.zst`、自包含 Conda runtime 和离线安装。
- canonical package/wheel/source cache 和断网 namespace。
- 双根 byte-identical、lifecycle probe 和候选/正式 payload equivalence。
- portable SSH evidence、challenge registry、accepted-candidate 和 final-status 链。
- 原地覆盖同版本的 `--repair`。

依赖版本/URL/SHA/许可证、路径安全、doctor/smoke、真实性能和失败不切换仍是
门槛。新的执行步骤必须在用户复核阶段四设计后另行制定。
