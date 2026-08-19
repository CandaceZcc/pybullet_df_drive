# 阶段四 D 旧计划状态索引

> 状态：未开始（原计划 0/71）
>
> 旧详细步骤暂停执行，本文件只记录待交付范围

## 仍有效的范围

- 完整 session manifest、MCAP segment 和场景 attachment 的安全读取。
- 默认隔离到 `/replay/sim/*` 且不回放 WheelCommand 的 Replay。
- PCD、PLY 与明确标记为 synthetic 的 LVX2 导出。
- 可选 ROS 2 Jazzy Bridge、RViz2 和 Livox Viewer 2 实际验收。
- live/replay namespace、同刻 TF/点云和仿真 clock。

ROS/RViz2 仍是可选下游；关闭或失败不得改变核心 eCAL 和 Recorder。

## 取消的旧前置

不再要求断网 ROS 构建、固定构建前缀、canonical source cache、context/handoff
或跨机器证据传输。安全 Reader、回放隔离、导出回读和真实 Viewer 门仍保留。

新的执行步骤必须在用户复核阶段四设计后另行制定。
