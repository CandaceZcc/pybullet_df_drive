# 阶段四旧总计划状态索引

> 状态：暂停执行，等待用户复核轻量化设计
>
> 历史进度：部分完成（原计划 15/36）
>
> 本文件不是可执行实施计划

## 权威替代

- 当前需求：`3d仿真平台需求规格.md`
- 当前架构：`ARCHITECTURE.md`
- 当前设计：
  `docs/superpowers/specs/2026-07-31-stage4-mid360-ecal-cpp-delivery-design.md`
- 实际完成状态：`docs/阶段四交付报告.md`

## 已确认事实

- 总计划 Task 1 和前置 P0 的真实 eCAL `4+2`、`2+0` 已完成。
- 旧 Task 2 只解除阻断，其计划步骤没有开始。
- 阶段 A 为部分完成；阶段 B-E 尚未开始。
- 功能依赖仍是 `A -> B/C -> D -> E`，重要功能继续执行 TDD，真实外部运行
  和大阶段六维审查仍是门禁。

## 不再执行

- canonical package/wheel/source cache、materializer 和断网构建。
- `.tar.zst` 离线包、双根 byte-identical 构建和 lifecycle probe。
- SSH challenge、跨机器 evidence、accepted-candidate/final-status/handoff 链。

这些取消项不影响 v2 wire、单中心 LiDAR、三点 RTK、C++ 工具、MCAP、
Replay/Export、可选 ROS 或真实性能验收。

用户复核当前书面设计后，才能另行制定新的轻量实施计划。历史详细步骤可从 Git
读取，不能从本文件恢复执行。
