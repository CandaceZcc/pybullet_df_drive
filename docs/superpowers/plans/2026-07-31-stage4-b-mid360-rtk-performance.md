# 阶段四 B 旧计划状态索引

> 状态：未开始（原计划 0/57）
>
> 旧详细步骤暂停执行，本文件只记录待交付范围

## 仍有效的范围

- 四种车型改为一个中心 MID-360 风格 `lidar_link`。
- 固定 `LEFT/CENTER/RIGHT` 三点 RTK 和统一姿态语义。
- 将阶段 A 的 v2 五话题接入正式 Simulator runtime。
- 保留四车型、三场地、障碍物、场景 v2 与显式 v1 转换。
- 处理 Dashboard、高尔夫场地、GUI 和 240/100/10 Hz 联合性能。
- 完成四车型乘三场地、真实 GUI 和实际性能门。

P0 异步 LiDAR worker 是前置修复，不代表 B 已开始或完成。

## 取消的旧前置

不再要求 canonical cache、固定开发 dependency prefix 或多层 evidence context。
canonical 车型/场景语义仍保留，它与依赖缓存无关。

新的执行步骤必须在用户复核阶段四设计后另行制定。
