# 阶段四 C 旧计划状态索引

> 状态：未开始（原计划 0/82）
>
> 旧详细步骤暂停执行，本文件只记录待交付范围

## 仍有效的范围

- 公共 C++17 `libslope_sim_client` SDK 和 CMake package config。
- 只读 Subscriber、唯一 Command publisher 和命令租约。
- 五 topic 原始 Protobuf bytes 的无损 MCAP Recorder。
- control socket 生命周期、协议健康状态和单实例边界。
- 正常停止的零命令、fence、drain、flush/fsync 和 finalize 顺序。
- Simulator、Subscriber 与 Recorder 的消息身份、频率和零 drop 三方一致性。

阶段 A 的 C++ Phase-0 探针不是公共 SDK、Command 或 Recorder，不能作为 C
完成证据。

## 取消的旧前置

不再要求预构建 dependency prefix、发行树 link materialization 或多层 evidence
事务。依赖版本、SHA、许可证和单进程 ABI 约束仍保留。

新的执行步骤必须在用户复核阶段四设计后另行制定。
