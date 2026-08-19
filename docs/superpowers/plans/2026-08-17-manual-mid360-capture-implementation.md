# 手动 MID-360 采集实施计划

**目标：** 让普通 `runSim` 的 Dashboard 驱动低开销手动采集；在结束后离线重建 MID-360 v2 MCAP、导出 LVX2 并自动导入 Livox Viewer 2。

**设计依据：** `docs/specs/2026-08-17-manual-mid360-capture-design.md`

## 任务 1：采集领域模型与低开销记录器（TDD）

文件：`slope_sim/manual_capture.py`、`tests/unit/test_manual_capture.py`

1. 先为采集时长选项、状态转换、冻结场景、每物理步轨迹记录、唯一输出目录和 finalize/abort 写 RED 测试。
2. 实现仅保留 JSON 可序列化轨迹与元数据的记录器；它不得导入 LiDAR 扫描器或调用 PyBullet raycast。
3. 运行聚焦单测得到 GREEN。

## 任务 2：Dashboard 采集状态与控件（TDD）

文件：`slope_sim/dashboard.py`、`tests/unit/test_dashboard.py`（必要时新增独立测试）

1. RED：验证默认 1 分钟、四种上限、按钮可用性、状态文本、结构控件锁定和完成前禁用 Viewer 导入。
2. 实现“采集”区域与无 GUI 业务回调边界；删除旧圆形 LiDAR 可见入口。
3. GREEN：运行 Dashboard 聚焦回归。

## 任务 3：手动循环集成与冻结边界（TDD）

文件：`slope_sim/manual_demo.py`、`tests/integration/test_manual_demo.py`

1. RED：采集中每物理步只写轨迹、方向键控制仍提交给本地物理、场景结构动作被拒绝；手动结束与到期共用 finalize。
2. 接入记录器和 Dashboard 事件；退出时写未完成回执并安全终止未完成任务。
3. GREEN：运行手动控制与 Dashboard 集成回归。

## 任务 4：离线重建与导出（TDD）

文件：新增手动回放/重建模块、现有 v2 MCAP/Stage4 exporter 调用点、`tests/stage4/` 与 `tests/integration/` 的聚焦测试。

1. RED：对确定轨迹重建合法 v2 MID-360 点云/MCAP；验证 firing offset、会话身份和 frame id；失败时不得产生可导入标记。
2. 在独立后台进程建立 DIRECT 世界并回放轨迹，用既有 `OfflineMid360Profile` 生成 MCAP，调用既有 exporter 生成并验证 LVX2。
3. GREEN：运行 MCAP、LVX2 解析和导出聚焦回归。测试输入保持极小，禁止产生大制品。

## 任务 5：Viewer 导入与启动器收口（TDD）

文件：`scripts/verify_livox_viewer2_linux.py`、Dashboard 回调、`runSim`、对应测试。

1. RED：仅在完成且验证的 LVX2 上允许导入；启动/自动导入失败应展示路径而非误报成功；`runSim --lidar` 不再启动自动映射。
2. 复用已验证隔离启动和 X11 文件对话框自动化；移除 launcher 的 `--lidar` 特殊自动映射分支。
3. GREEN：运行启动器、Viewer 边界与 Dashboard 聚焦测试。

## 任务 6：性能验证与收口

1. 合并只读性能审计结论，只实施有证据且不改变功能/质量的优化。
2. 运行所有直接受影响的单测和集成测试；必要时做一次短手工 GUI 采集，避免超过 5 GiB 制品。
3. 记录 RED/GREEN 命令、端到端结果和剩余环境风险。
