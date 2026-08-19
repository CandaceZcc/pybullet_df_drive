# 阶段四 A 旧计划状态索引

> 状态：已完成（已发现复审 P1 均已修复；独立六维审查 P0=0、P1=0）
>
> 旧详细步骤暂停执行，本文件只记录边界

## 已完成并有证据

- v1 descriptor 冻结。
- v2 schema、descriptor、确定性 codec、session 和 command authority。
- Python/C++ 五消息 golden。
- raw eCAL Phase-0 与同 topic v1 冲突硬拒绝。

实际测试和真实运行证据见 `docs/阶段四交付报告.md`。

## 收口结论

- 独立六维只读审查已完成，结论为 `Critical=0, Important=0`。

A 不包含单中心 LiDAR/三点 RTK 生产 runtime、C++ SDK/Recorder、ROS 或发行包；
这些边界留待后续 B-E 轻量实施计划。

旧 dependency-prefix、离线 cache 和 evidence 执行细节不再是未来门槛；A 已
形成的 wire、命令权和互操作成果仍然有效。后续边界以阶段四设计为准。
