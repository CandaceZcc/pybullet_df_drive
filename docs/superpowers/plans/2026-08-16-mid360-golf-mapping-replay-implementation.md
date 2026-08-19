# MID-360 Golf 高保真采集与三维回放实施计划

> 日期：2026-08-16
>
> 状态：生产实现与 DIRECT v2 acceptance 已完成；最终同会话 GUI QA、默认回归和发行收口转由 `2026-08-16-project-closeout-execution.md` 管理
>
> 需求基线：`docs/specs/2026-08-16-mid360-golf-mapping-replay-design.md`
>
> 工作树：保留当前全部未提交改动，不 reset、stash、checkout 或恢复已删除文件

## 实施边界

- 新能力使用独立离线 profile、执行器和入口；实时 `Stage4LidarProfile.realtime()`、
  双 shard worker、100 ms deadline 和 5,760-slot 合同保持不变。
- v2 proto、五个正式 topic、C++ Replay/Export 默认行为和 LVX2 原始局部坐标语义保持不变。
- 固定场景使用 `df_mid/golf_heightfield/seed=41/medium`、6 个静态障碍和 3 个往返运动障碍。
- 回放只读取完整 MCAP 和绑定的 Recorder 成功结果；不访问活跃 PyBullet 世界。
- 预计完整约 200 s 会话产生约 40M firing，MCAP 预计低于 2 GiB；复用现有 Stage 4
  构建根和 `slope-sim` 环境，不创建重复源码、环境或构建树。

## Task 1：离线 schedule 与 240 Hz 扫描

文件边界：

- 新增 `slope_sim/mid360_offline.py`
- 新增 `tests/unit/test_mid360_offline.py`
- 新增 `tests/integration/test_mid360_offline_direct.py`
- 仅在确需复用官方 asset 数学时小改 `slope_sim/lidar_pointcloud.py`

线性 TDD：

1. RED 证明独立 profile 为 20,000 slots、5 us、40 帧遍历 800,000 行，24 步为
   833/834 且总数精确；同时证明 realtime 仍为 5,760。
2. GREEN 实现独立 schedule，不让该类型进入 realtime worker。
3. RED/GREEN 证明每步只冻结一次车辆、LiDAR 和障碍状态，执行一次低于 16,383 的
   batch raycast，并把 batch-local hit index 恢复为 global slot。
4. 证明每个命中按所属 240 Hz pose 逆变换为 raw local point，miss 不造点，量程为
   0.1..40 m，tag/reflectivity/line 沿用现有语义。

验收命令：

```bash
conda run -n slope-sim python -m pytest -q tests/unit/test_mid360_offline.py
conda run -n slope-sim python -m pytest -q tests/integration/test_mid360_offline_direct.py
conda run -n slope-sim python -m pytest -q tests/unit/test_mid360_pattern.py tests/unit/test_lidar_pointcloud.py tests/integration/test_sensor_backend.py tests/integration/test_lidar_worker.py
```

## Task 2：姿态恢复与逐点去畸变

文件边界：

- 新增 `slope_sim/mapping_replay.py`
- 新增 `tests/unit/test_mapping_replay.py`

线性 TDD：

1. RED/GREEN 用完整 `LEFT-RIGHT` 三维基线和 IMU roll/pitch 恢复连续 base quaternion，
   禁止把 `heading_rad` 当 ZYX yaw。
2. 冻结并验证 RTK CENTER `(0,0,0.18)` 与 `base_link -> lidar_link`
   `(0,0,0.105)`。
3. RED/GREEN 实现位置线性插值、最短弧 SLERP 和逐点 deskew；已知平移/转弯轨迹的
   raw cloud 必须恢复同一个世界命中，缺 look-ahead 的帧不得入图。

验收命令：

```bash
conda run -n slope-sim python -m pytest -q tests/unit/test_mapping_replay.py
conda run -n slope-sim python -m pytest -q tests/unit/test_truth_sensors.py tests/stage4/test_v2_codec.py tests/stage4/test_v2_sensor_frames.py
```

## Task 3：canonical Golf 路线、场景与安全停车

文件边界：

- 新增 `slope_sim/mid360_golf_drive.py`
- 新增 `configs/mid360_golf_mapping.yaml`
- 新增 `tests/unit/test_mid360_golf_drive.py`
- 新增 `tests/integration/test_mid360_golf_drive_direct.py`

线性 TDD：

1. RED/GREEN 冻结 5 条扫描带、`x=TerrainBounds 内缩 2.75 m`、半径 1.25 m 的四个
   U 形连接和从出生点连续驶入首带的固定 approach。
2. 用实际 `df_mid` AABB 验证路线走廊；固定 6+3 障碍及移动扫掠 AABB 必须避开走廊。
3. RED/GREEN 实现曲率前馈加横向/航向反馈、100 Hz 仿真时间控制、轮速/变化率限幅。
4. RED/GREEN 实现越界、障碍碰撞、持续偏差、卡死和 Recorder fault 的锁存停车；零命令
   持续到 base/驱动轮阈值连续满足 0.2 s。
5. 短 DIRECT 场景证明真实轮关节推进、无 reset/snap、移动障碍先更新再 step。

验收命令：

```bash
conda run -n slope-sim python -m pytest -q tests/unit/test_mid360_golf_drive.py
conda run -n slope-sim python -m pytest -q tests/integration/test_mid360_golf_drive_direct.py
conda run -n slope-sim python -m pytest -q tests/unit/test_controller.py tests/integration/test_scene.py tests/integration/test_obstacle_manager.py tests/integration/test_robot_models.py
```

## Task 4：离线 simulator、唯一 Command peer 与 Recorder 生命周期

文件边界：

- 新增 `scripts/mid360_golf_simulation.py`
- 新增 `scripts/mid360_golf_command_peer.py`
- 小改 `slope_sim/interfaces/v2/transport.py`
- 小改 `cpp/client/stage4_recorder.cpp` 及其聚焦测试
- 新增 `tests/stage4/test_mid360_golf_command_peer.py`
- 新增 `tests/integration/test_mid360_golf_simulation.py`

线性 TDD：

1. Command peer 是唯一 producer，订阅 WheelState/RTK/IMU，以 WheelState 仿真时间推进，
   保留 session/world/command generation、车型顺序和连续 sequence。
2. simulator 按“冻结并扫描、命令、移动障碍、物理 step、发布”执行；机器可慢于实时，
   但 firing 与消息时间只按仿真时钟推进。
3. 正常路径平滑停车并录制 0.5 s 尾段；最后一个 LiDAR 后追加同会话 RTK/IMU
   look-ahead，不外推姿态。
4. Recorder 增加可选的显式逐 topic 精确计数；原 `--duration-ms`、统一
   `--expected-count`、排空和原子 finalize 语义保持不变。
5. fault marker 只传递安全状态；机器人仍只接受 topic WheelCommand，禁止进程外私下控制。

验收命令：

```bash
conda run -n slope-sim python -m pytest -q tests/stage4/test_mid360_golf_command_peer.py tests/integration/test_mid360_golf_simulation.py
cmake --build build/stage4-phase0-ecal611-release-mapped-20260809T220738+0800 --target slope_sim_stage4_recorder
ctest --test-dir build/stage4-phase0-ecal611-release-mapped-20260809T220738+0800 --output-on-failure -R '^(slope_sim_client_recorder_session|slope_sim_client_mcap_session_writer)$'
```

## Task 5：严格 MCAP 读取、地图与播放时钟

文件边界：

- 新增 `slope_sim/mapping_mcap.py`
- 扩展 `slope_sim/mapping_replay.py`
- 新增 `tests/stage4/test_mapping_mcap.py`
- 扩展 `tests/unit/test_mapping_replay.py`

线性 TDD：

1. 使用官方 Python `mcap` 流式读取 footer/summary/schema/channel/manifest 和五 topic，
   不手写 MCAP 二进制解析，不把全会话点对象载入内存。
2. 严格校验 Recorder 成功结果、session/world/descriptor/pattern/scene、MCAP 与 payload
   sequence/time、逐 topic 单调性、RTK/IMU 同刻节点及 LiDAR 5 us offset。
3. RED/GREEN 实现 5 cm 稀疏体素、tag 1/2 永久层、tag 3 的 0.3 s TTL、Golf AABB、
   500,000 静态点上限和确定性显示抽样。
4. RED/GREEN 实现暂停、逐帧、倍率、定位和 generation 重建；单 in-flight 工作未完成时
   时间线等待，不丢逻辑帧。

验收命令：

```bash
conda run -n slope-sim python -m pytest -q tests/stage4/test_mapping_mcap.py
conda run -n slope-sim python -m pytest -q tests/unit/test_mapping_replay.py
```

## Task 6：同步双三维窗口与单一公开编排入口

文件边界：

- 新增 `slope_sim/mapping_replay_gui.py`
- 新增 `scripts/run_mid360_golf_mapping.py`
- 新增 `tests/stage4/test_mapping_replay_gui.py`
- 新增 `tests/integration/test_mid360_golf_mapping_launcher.py`

实施内容：

- PySide6 + `pyqtgraph.opengl` 双 `GLViewWidget`，43/57 布局，左 raw local、右永久/运动
  世界层和轨迹；右视野固定 Golf 全场，提供俯视/透视、视角复位和有界点大小。
- 底部实现回到开头、前后帧、播放/暂停、时间轴和 0.25x..4x；后台使用容量 1 的
  请求/结果队列交付只读 numpy 结果。
- 公开入口完成预检、共享身份、Recorder/simulator/command ready-start 屏障、故障停车、
  result/MCAP 严格验证，并只在成功后自动打开回放。

验收命令：

```bash
QT_QPA_PLATFORM=offscreen conda run -n slope-sim python -m pytest -q tests/stage4/test_mapping_replay_gui.py
conda run -n slope-sim python -m pytest -q tests/integration/test_mid360_golf_mapping_launcher.py
```

## Task 7：正式依赖、lock 与发行 payload

文件边界：

- `environment.yml`
- `pyproject.toml`
- `packaging/python-environment.yml`
- 由 conda-lock 生成 `packaging/locks/python.conda-lock.yml`
- 由 conda-lock 渲染 `packaging/locks/python-linux-64.lock`
- `scripts/stage4_project_payload.py`
- 对应 Stage 4 release/payload 测试

实施内容：

- 正式加入 `mcap`、`pyqtgraph`、`PyOpenGL`，同步开发环境、正式环境、统一 lock 和
  explicit lock；禁止手工编辑 solver 输出。
- release probe 必须导入 `mcap.reader`、`pyqtgraph.opengl` 和 `OpenGL.GL`；payload 必须
  携带公开入口和新增生产模块。

## Task 8：阶段级集成、GUI 验收与独立审查

1. 运行相关 Python 回归、C++ Recorder/Reader/Writer/Replay 回归和 v2 golden/实时
   5,760/LVX2 兼容回归。
2. 只运行一次真实五 topic 短 eCAL 链，再运行一次完整 Golf GUI 采集；核对路线 P95、
   持续运动、覆盖率、障碍命中、去畸变误差、点数上限和 MCAP 完整性。
3. 在真实 `DISPLAY` 打开双 OpenGL 回放，验证非空像素、固定全场构图、同步播放和交互，
   保存有界结果与截图后正常关闭验收进程。
4. 作为本批正式里程碑，只在全部子任务合并后运行一次 README 默认回归。
5. 启动一次独立只读六维审查；实现流程修复发现的问题，只做针对性复验。

最终回归：

```bash
conda run -n slope-sim python -m pytest -q -m 'not ecal and not stage4_artifact'
```
