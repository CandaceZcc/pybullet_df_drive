# 阶段四 P0 异步 LiDAR Worker 实施计划

> 日期：2026-08-03
>
> 执行模型：Sol Ultra 冻结架构与任务；Terra High 可按本文逐项实现
>
> 权威设计：`docs/superpowers/specs/2026-08-03-stage4-p0-async-lidar-worker-design.md`
>
> 当前状态：Task 0-13 的本地实现、DIRECT 实时门和独立审查已通过；2026-08-04 的真实主动转向 `4+2` 与差速 `2+0` 均已 PASS，P0 完成并解除 Task 2 阻断

## 1. 范围与执行纪律

本文只修复阶段四 P0 中同步 LiDAR 阻塞 240 Hz 物理循环的问题。保持前后 LiDAR 各 10 Hz、错相 50 ms、每帧 2880 射线、v1 wire、现有 eCAL oracle 和 local 同步路径不变；不进入阶段四 Task 2，不修改 reference admission，不降低性能门槛。

每个 Task 严格执行：写单一行为 RED -> 运行并确认因缺少目标行为而 FAILED -> 写最小 GREEN -> 运行聚焦测试 -> REFACTOR -> 原样复验。RED 不得是 collection、ImportError、spawn bootstrap 或环境错误。没有观察到目标 RED 前不得修改对应生产代码。

除用户另行给出紧邻单条命令的授权外，本文禁止运行真实 eCAL。所有本地测试必须清理自己创建的 child、pipe 和 PyBullet DIRECT client，不能终止不属于本测试的进程。

## Task 0：清零设计复核并保存实施基线

**Files:**

- Verify: `docs/superpowers/specs/2026-08-03-stage4-p0-async-lidar-worker-design.md`
- Verify: `docs/superpowers/plans/2026-08-03-stage4-p0-async-lidar-worker.md`
- Verify: current worktree only; do not clean or reset it

- [x] 独立复审设计中 measurement-start fence、full-batch ready、generation、retired cleanup、reconcile fault、normal/force close、typed events 和 P0 20 障碍 bootstrap；第三轮结果 `Critical=0`、`Important=0`。
- [x] 原样运行现有 LiDAR/runtime/lifecycle 非 eCAL 基线：`213 passed in 5.17s`。
- [x] 已记录当前分支、相关文件 diff 和基线结果；未 stash、未 reset、未覆盖用户修改。

Run:

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_lidar_pointcloud.py \
  tests/test_lidar_pointcloud_direct.py \
  tests/test_interface_runtime.py \
  tests/test_interface_runtime_integration.py \
  tests/test_interface_pause_rebuild.py
```

Stop：规范复审仍有 Critical/Important，或基线存在未解释失败时不得进入 Task 1。

## Task 1：提取冻结位姿扫描入口

**Files:**

- Modify: `slope_sim/lidar_pointcloud.py`
- Test: `tests/test_lidar_pointcloud.py`

- [x] RED `test_frozen_lidar_scan_matches_live_scan_without_pose_lookup`：因 `_scan_frozen` 不存在而在测试函数内明确 FAILED。
- [x] RED `test_frozen_dashboard_scan_keeps_message_and_top_view_atomic`：同样因冻结入口缺失而明确 FAILED，非 collection/import/environment 错误。
- [x] GREEN 提取接受 exact `Pose` 的包内入口；复用 `_world_rays()`、`_indexed_hits()`、逆变换、分类和现有结果模型。
- [x] 保持 `scan()`、`scan_with_top_view()` 先读取 live mount/base pose，再调用同一入口；冻结入口不调用 `world_pose()`。
- [x] REFACTOR 无额外抽象；主任务新鲜复验为聚焦 `2 passed in 0.58s`、全文件 `67 passed in 1.28s`，独立只读审查无 P0/P1/P2 finding。

RED/GREEN:

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_lidar_pointcloud.py::test_frozen_lidar_scan_matches_live_scan_without_pose_lookup \
  tests/test_lidar_pointcloud.py::test_frozen_dashboard_scan_keeps_message_and_top_view_atomic
conda run -n slope-sim python -m pytest -q tests/test_lidar_pointcloud.py
```

Stop：冻结路径仍查 live pose，或点序、分类、offset、top-view 原子性发生变化。

## Task 2：冻结 IPC 值、digest 与 worker ready 自检

**Files:**

- Create: `slope_sim/lidar_worker.py`
- Create: `tests/test_lidar_worker.py`

- [x] RED `test_lidar_worker_entrypoint_is_importable_and_callable`：使用动态 import，在测试函数内断言缺失入口，不允许 collection error。
- [x] RED `test_worker_contract_values_are_frozen_slotted_and_strict`：覆盖 exact type、bool/int、uint64、topic/frame/lidar 配对、512-byte detail 和协议版本。
- [x] RED `test_worker_world_digest_is_canonical`：覆盖 mapping 字段顺序不影响 digest、内容漂移改变 digest、NaN/非 `SceneDocument` 被拒绝。
- [x] RED `test_worker_ready_follows_full_front_rear_preflight`：真实 spawn、真实 DIRECT、真实 world builder/backend/scanner/codec；ready 晚于前后各一次 2880-ray deterministic encode。
- [x] RED `test_worker_preflight_failure_never_emits_ready` 和 `test_spawned_worker_closes_direct_client_and_process_cleanly`。
- [x] RED `test_spawned_preflight_failure_is_exact_and_leaves_no_child`：覆盖 front/rear phase、稳定 detail、EOF、非零退出和无残留 child；正式 entrypoint 另覆盖 world-build failure。
- [x] 锁定错误码映射：world/front/rear 只允许 `worker_preflight_failed`，startup cleanup 或父 `Process.start()` 只允许 `worker_start_failed`，无合法 envelope 的 EOF/超时只允许 `worker_exited`。
- [x] GREEN 实现顶层可 pickle child entrypoint、`get_context("spawn")`、非 daemon process、两条单向 `Connection`、严格 dataclass 和 canonical SHA-256。
- [x] GREEN child 完整构建/绑定初始场景并预热双雷达后才发 `LidarWorkerReady`；预热帧不占 job id、不返回父进程。主线程新鲜复验为聚焦 `14 passed`、全文件 `21 passed`、点云回归 `67 passed`，`py_compile` 与相关 `git diff --check` 通过；独立复查 `Critical=0`、`Important=0`、`Minor=0`。

Run:

```bash
conda run -n slope-sim python -m pytest -q tests/test_lidar_worker.py \
  -k "entrypoint or contract or digest or ready or preflight or closes_direct"
```

Stop：使用 `fork`、daemon、Queue、mock PyBullet，或把初始 body/scanner/codec 懒创建留到第一条计时 job。

## Task 3：实现真实 worker 帧与逐字节预编码

**Files:**

- Modify: `slope_sim/lidar_worker.py`
- Test: `tests/test_lidar_worker.py`

- [x] RED `test_spawned_worker_returns_preencoded_atomic_frame`：message、可选 top view、payload 和 identity 同源。
- [x] RED `test_spawned_worker_reconciles_complete_obstacle_snapshot_by_logical_id`：覆盖新增、删除、移动和 body id 不跨进程。
- [x] RED `test_spawned_worker_frame_payload_matches_direct_codec_bytes`：worker payload 与父进程同步基线 deterministic bytes 相同。
- [x] RED `test_reconcile_rollback_failure_returns_unknown_scene_state`：无法证明镜像回滚时返回 service-fatal 稳定错误并终止 child。
- [x] GREEN 串行处理 exact `LidarScanRequest`，每 job 先 reconcile 完整 snapshot，再用冻结 mount/base pose 扫描并只 encode 一次。
- [x] 单帧失败不返回部分 frame；detail 不携带 traceback 或异常对象。主线程新鲜复验为原计划聚焦 `4 passed`、帧合同与审查边界 `3 passed`、全 worker `28 passed`、点云回归 `67 passed`，`py_compile` 与相关 `git diff --check` 通过；独立六维复核无 P0/P1/P2。

Run:

```bash
conda run -n slope-sim python -m pytest -q tests/test_lidar_worker.py \
  -k "preencoded or reconciles or codec_bytes or unknown_scene"
```

Stop：父进程 body id 进入 IPC、worker 调用 `stepSimulation()`、一帧多次 encode，或未知镜像状态后继续扫描。

## Task 4：实现 1 in-flight + 1 pending service

**Files:**

- Modify: `slope_sim/lidar_worker.py`
- Test: `tests/test_lidar_worker.py`

- [x] RED `test_service_keeps_one_pending_without_writing_it_to_pipe`。
- [x] RED `test_service_rejects_third_capture_without_overwriting_older_jobs`。
- [x] RED `test_service_assigns_job_id_only_when_capture_enters_pipe` 和 `test_pause_cancels_pending_without_job_gap`。
- [x] RED `test_service_rejects_mismatched_duplicate_or_out_of_order_response`。
- [x] RED `test_service_marks_job_over_hundred_milliseconds_as_overrun_once`：注入 fake `monotonic_ns`，不得真实等待。
- [x] RED `test_service_events_are_typed_ordered_and_consumed_once`：覆盖 topic/service scope 和 drain 后不重复。
- [x] GREEN 实现 production channel seam、连续 job id、`poll(0)`、严格 response matching、固定 snapshot 和一次性 event drain。
- [x] `poll()` 在返回前完成 ready 检查、`recv()`、重构和严格校验；后续本地 heartbeat 将测量整个调用，不把 `poll(0)` 描述为零成本。

实际 RED/GREEN 记录（均为测试函数内行为失败，不是 collection/import/environment 错误）：

1. 合同：`-k 'service_event_and_snapshot_contracts'` 的 RED 为 `LidarServiceEvent must exist`；同命令 GREEN 为 `1 passed, 28 deselected`。
2. 容量：`-k 'service_keeps_one_pending or service_rejects_third_capture'` 的两条 RED 均为 `LidarScanService must exist`；同命令 GREEN 为 `2 passed, 29 deselected`。
3. ID/pause：`-k 'assigns_job_id_only or pause_cancels_pending'` 的 RED 分别为缺少 `poll()` 与 `pause()`；同命令 GREEN 为 `2 passed, 31 deselected`。
4. 响应顺序：`-k 'rejects_mismatched_duplicate_or_out_of_order_response'` 的 RED 证明错身份和未来 job 被错误返回、重复响应抛异常；同命令 GREEN 为 `3 passed, 33 deselected`。随后非法 frame 错误码 RED 证明 `ValueError` 逃逸，参数化 GREEN 为 `4 passed, 37 deselected`。
5. 超时/事件：`-k 'marks_job_over_hundred or service_events_are_typed'` 的 RED 证明 `100 ms + 1 ns` 后没有事件且迟到帧被错误返回；同命令 GREEN 为 `2 passed, 36 deselected`。
6. 发送断管：`-k 'send_failure_is_terminal'` 的 RED 为 `BrokenPipeError` 从 `capture()` 逃逸；同命令 GREEN 为 `1 passed, 38 deselected`。
7. pending 等待：`-k 'overrun_includes_parent_side_pending_wait'` 的 RED 只产生首 job overrun，缺少提升后 job2 的同次事件；同命令 GREEN 为 `1 passed, 39 deselected`。
8. 审查补强：response `poll/recv` 的 EOF/OSError/协议异常新增为既有生产分支的持久化测试，`4 passed, 41 deselected`；这是纯测试覆盖，不伪称事后 RED，生产代码未因此修改。

production `Connection` seam 另用两条真实单向 `multiprocessing.Pipe(False)` 验证 `LidarServiceChannel.send/poll/recv`，命令 rc=0。审查补强后主线程新鲜复验为聚焦 `17 passed`、全 worker `45 passed`、点云回归 `67 passed`、`py_compile` rc=0；三份 untracked 文件的 no-index whitespace check 无输出，禁用模式无匹配且没有遗留 worker/PyBullet child。独立六维定点复审为 `P0/P1/P2/P3=0`、`Critical/Important/Minor=0`。

Run:

```bash
conda run -n slope-sim python -m pytest -q tests/test_lidar_worker.py \
  -k "service or pause_cancels_pending_without_job_gap"
```

Stop：依赖 `sleep()`/`qsize()` 判定、覆盖 pending、给 production 增加测试专用 release/delay，或累计 snapshot 导致重复计错。

## Task 5：接入 runtime 非阻塞捕获与 prepared 发布

**Files:**

- Modify: `slope_sim/interfaces/runtime.py`
- Test: `tests/test_interface_runtime.py`
- Test: `tests/test_interface_runtime_integration.py`

- [x] RED `test_async_lidar_capture_does_not_wait_before_next_wheel_deadline`：用 Event 证明未释放 worker 时物理帧已返回。
- [x] RED `test_async_lidar_capture_freezes_bodyless_scene_atomically`：mount/base/障碍物属于同一 physics generation。
- [x] RED `test_async_lidar_result_uses_worker_payload_without_parent_reencode`：codec encode 在 prepared 路径被设为失败仍能发布原 bytes。
- [x] RED `test_async_lidar_allows_rtk_and_imu_at_same_timestamp_to_publish_immediately`。
- [x] RED `test_process_mode_preview_excludes_unprepared_lidar_topic`。
- [x] GREEN 给 runtime 注入可选 `LidarScanService`；无 service 保持同步路径，actual local/`auto -> local` 明确拒绝 service，避免错误接管所有权。
- [x] 每帧开头和结尾各 poll 一次 service；LiDAR deadline 只冻结 capture 并提交，RTK/IMU 仍在本帧发布。
- [x] 提取 encoded publish 内核，复用 generation、tracker、latest、logger、transport；payload 要求 exact 非空 `bytes`，不 parse、不 encode。

实际 RED/GREEN：

1. 首批 RED 命令以 `-k "async_lidar_capture_does_not_wait or async_lidar_result_uses_worker_payload or async_lidar_allows_rtk"` 运行，三条均因构造器缺少 `lidar_scan_service` 参数在测试函数内失败：`3 failed, 109 deselected`。
2. 第二批 RED 命令以 `-k "async_lidar_capture_freezes or process_mode_preview or async_lidar_polls_once"` 运行，原子快照、preview 和帧首/帧尾 poll 三条同样准确失败：`3 failed, 112 deselected`。
3. 首次 GREEN 为 `5 passed, 1 failed`；唯一失败是 poll 顺序测试漏注入 fake monotonic，显式 `wall_time=0.0` 相对真实构造时钟倒退。只修测试夹具后原样命令为 `6 passed, 109 deselected`，生产行为未为该夹具错误改动。
4. 独立审查发现 actual local 显式注入 service 会错误走异步路径。参数化 `local` 与 `auto -> local` RED 均为 `DID NOT RAISE ValueError`：`2 failed, 74 deselected`；GREEN 只在 service 非空时读取 actual transport snapshot，非 exact `ecal` 拒绝且不使用/关闭 service：`2 passed, 74 deselected`。

最终主线程新鲜复验为 Task 5 聚焦 `8 passed`、runtime/lifecycle `149 passed`、transport/scene 显式非 eCAL 回归 `140 passed`，`py_compile` 与限定 diff check 均 rc=0。独立六维定点复审为 `P0/P1/P2/P3=0`、`Critical/Important/Minor=0`。

Run:

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_interface_runtime.py \
  tests/test_interface_runtime_integration.py \
  -k "async_lidar or process_mode_preview"
```

Stop：主线程等待 raycast、prepared frame 再次 parse/encode、local 变为异步，或 RTK/IMU 被 worker 阻塞。

## Task 6：增加 measurement start/end/final sensor fence

**Files:**

- Modify: `slope_sim/interfaces/runtime.py`
- Modify: `scripts/ecal_simulation_runtime.py`
- Test: `tests/test_interface_runtime_integration.py`
- Test: `tests/test_ecal_process_roundtrip.py`

- [x] RED `test_measurement_start_fence_prevents_warmup_lidar_from_crossing_snapshot`。
- [x] RED `test_measurement_fence_drains_captured_lidar_before_transport_and_logger_snapshot`。
- [x] RED `test_measurement_start_ack_resumes_previously_ready_lidar_service`、`test_measurement_end_ack_resumes_post_window_protocol` 和 `test_fence_does_not_resume_previously_suspended_service`。
- [x] RED `test_sensor_fence_timeout_prevents_success_ack`。
- [x] GREEN 实现 250 ms 可恢复 barrier：停止新 capture、保留并完成 in-flight/pending、发布合法 prepared frame，再依次等 logger idle、transport idle、取快照和写 ACK。
- [x] start/end ACK 后只恢复先前 ready 的 service，以继续后测协议；final 保持冻结。local/no-service 是明确 no-op。

实际 RED/GREEN：

1. runtime fence 六条行为测试在删除先行实现后原样观察到 `6 failed, 41 deselected`，均在测试函数内因缺少 `begin_sensor_fence()` 失败；重写最小实现后为 `6 passed, 41 deselected`。覆盖 captured drain、先前 ready/suspended、final 不恢复、no-service no-op 和 250 ms timeout 后保持 gate 关闭。
2. measurement-start 脚本 RED 为 `_capture_normal_load_start` 不存在：`1 failed`；接入 `sensor -> logger -> transport -> snapshot -> ACK -> resume` 后同命令 `1 passed`。measurement-end RED 为 `_capture_normal_load_end() got an unexpected keyword argument 'runtime'`：`1 failed`；正式调用链接入 runtime fence 后 `1 passed`。
3. 脚本级 timeout 和 suspended-token 用例在对应 runtime/start 行为已 GREEN 后首次运行即通过，分别为 `1 passed` 和主线程组合 `3 passed`；它们是已有生产分支的持久化组合覆盖，不伪称新的孤立 RED，也未因此修改生产代码。
4. final helper RED 为旧两参数接口收到 runtime/logger 后 `TypeError: takes 2 positional arguments but 4 were given`：`1 failed`；GREEN 接入 sensor/logger/transport/snapshot/ACK 且 `resume_capture=False` 后 `1 passed`。inactive snapshot RED 为 `DID NOT RAISE RuntimeError`，补 active/peer gate 后 final 两条 `2 passed`。
5. 独立审查发现 250 ms 总预算在初始 idle 和 poll 后刚变 idle 时可绕过。两条可控时钟 RED 均为 `DID NOT RAISE TimeoutError`：`2 failed`；把 deadline 检查前移到每次 snapshot 后后，含原 timeout 的三条为 `3 passed`。审查另发现 closed logger 会被当作 idle；RED 为 `DID NOT RAISE RuntimeError`，公共 logger gate 拒绝 closed 后 final 三条为 `3 passed`。

最终主线程新鲜复验为 Task 6 聚焦 `18 passed, 186 deselected`；worker/runtime/lifecycle/process-roundtrip 显式 `-m "not ecal"` 回归为 `353 passed, 4 deselected`。`py_compile`、限定 `git diff --check` 均 rc=0，没有遗留 worker/peer 进程。独立六维只读审查修复两项 Important 后，代码与测试复审 `P0/P1/P2/P3=0`、`Critical/Important/Minor=0`；未运行新的真实 eCAL 验收。

Run:

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_interface_runtime_integration.py \
  tests/test_ecal_process_roundtrip.py \
  -k "sensor_fence or measurement_start or measurement_end or measurement_fence or previously_suspended"
```

Stop：warmup frame 穿过 start snapshot、末帧被丢弃后仍 ACK、barrier 误入 terminal draining，或 pause 中 service 被意外恢复。

## Task 7：完成 pause、disconnect 与 generation 失效

**Files:**

- Modify: `slope_sim/lidar_worker.py`
- Modify: `slope_sim/interfaces/runtime.py`
- Test: `tests/test_lidar_worker.py`
- Test: `tests/test_interface_pause_rebuild.py`

- [x] RED `test_pause_discards_old_epoch_result_before_resume`、`test_pause_cancels_pending_without_topic_drop`、`test_resume_uses_new_scheduler_deadline`。
- [x] RED `test_disconnect_invalidates_old_generation_without_faulting_service` 和 `test_disconnect_cancels_old_pending_but_accepts_new_generation`。
- [x] 上述 disconnect RED 同时断言 retag 前后的 service counters 连续，不因 generation 更新清零。
- [x] GREEN pause 先推进 pause epoch、suspend、撤销 pending，不等待 native raycast；迟到结果只计 stale。
- [x] GREEN 保持现有 eCAL disconnect generation 推进点，并调用纯父进程 `invalidate_generation()`；旧 in-flight 迟到丢弃，新 generation capture 可排队。
- [x] 即使 runtime paused，也要非阻塞 poll/回收 stale response，不能让 pipe 和 child 永久挂住。

实际 RED/GREEN：

1. runtime pause 两条 RED 使用真实 `LidarScanService` 形成 `1 in-flight + 1 pending`，均在 `runtime.pause()` 后得到 `ready != suspended`：`2 failed, 32 deselected`；接入 pause epoch/suspend/pending 撤销后 `2 passed, 32 deselected`。
2. resume RED 已先证明旧 `rear/50 ms` 在 paused poll 中只计 stale，随后在 `100 ms` 新 deadline 因 service 仍 suspended 得到 `len(channel.sent) == 1` 而非 2：`1 failed, 34 deselected`；只恢复本次 runtime pause 拥有的同一 service 后，与既有 resume failure 合并为 `4 passed`。
3. worker generation RED 为 `LidarScanService.invalidate_generation must exist`：`1 failed`；纯父端 retag 撤销旧 pending、保留旧 in-flight 且 counters 连续后 `1 passed`。runtime disconnect RED 为 runtime 已到 generation 1 而 service 仍为 0：`1 failed`；在同一断线边沿同步 invalidation 后 `1 passed`。
4. 独立审查确定性复现 capture-vs-pause、pending promotion-vs-pause/invalidate 三个线性化缺口；RED timeline 均显示 transition 在 capture/send 完成前返回：`3 failed, 46 deselected`。service 用同一父端 `RLock` 串行化公开状态机后为 `3 passed, 46 deselected`；锁不等待 child native raycast，`poll(0)` 未 ready 时立即返回。
5. 重复 disconnected、paused disconnect 和 service resume 异常三条是既有 GREEN 分支的补充覆盖，首次组合运行 `3 passed, 36 deselected`，不伪称 RED。它们锁定 generation/epoch 独立、当前 pending 不被重复断线撤销，以及 resume 异常不提前发布 runtime 已恢复状态。

最终主线程新鲜复验为计划聚焦 `44 passed, 44 deselected`、两个目标文件 `88 passed`、worker/runtime/integration/pause-rebuild 四文件显式非 eCAL `213 passed`。`py_compile`、限定 `git diff --check` 均 rc=0，没有遗留 worker/peer 进程。独立六维复审为 `P0/P1/P2/P3=0`、`Critical/Important/Minor=0`；未运行真实 eCAL。

Run:

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_lidar_worker.py tests/test_interface_pause_rebuild.py \
  -k "pause or resume or disconnect or generation"
```

Stop：pause/disconnect 阻塞等 raycast、stale 被算 topic error/drop，或旧结果在恢复后发布。

## Task 8：完成 rebuild 候选事务和 retired cleanup

**Files:**

- Modify: `slope_sim/lidar_worker.py`
- Modify: `slope_sim/interfaces/runtime.py`
- Test: `tests/test_lidar_worker.py`
- Test: `tests/test_interface_pause_rebuild.py`

Task 8 只前移已证明 idle 的 service 所有权与幂等终结原语，供 candidate/retired cleanup 使用；仍有 in-flight/pending 的 active normal drain、超时 force、owned child terminate 和 transport/logger 关闭顺序继续由 Task 9 独占。

- [x] RED `test_rebuild_waits_for_old_service_idle_before_candidate_start`。
- [x] RED `test_rebuild_discards_old_generation_result` 和 `test_rebuild_advances_generation_only_in_prepare`。
- [x] RED `test_rebuild_candidate_failure_preserves_active_service`。
- [x] RED `test_rebuild_rollback_retags_and_reuses_old_service`。
- [x] rollback-reuse RED 同时断言 old service counters 连续；只有安装不同 digest 的新 service 才从零开始。
- [x] RED `test_rebuild_commit_ignores_retired_service_cleanup_failure`。
- [x] GREEN prepare 保存 old canonical digest、suspend 旧 service、撤销 pending、250 ms 收敛 in-flight；不同 digest 的候选只在旧 worker 不扫描后于锁外启动/prewarm，同 digest 的 coordinator rollback 明确复用旧 service。
- [x] commit 在同一临界区交换 robot/backend/sensors/service，沿用 prepare generation；旧文档 rollback/abort retag 并恢复旧 service。
- [x] 原子发布后 retired close 失败只记录诊断和待重试资源，不向 coordinator 抛错、不 fault 新 service；runtime close 再清理。

实际 RED/GREEN 与审查闭环：

1. 核心 candidate/digest/rollback/retired 合同按上述节点完成 RED/GREEN；继续执行前保存的 Task 8 精确节点为 `15 passed`，worker + pause/rebuild 为 `103 passed`，runtime + integration 为 `125 passed`。
2. 首轮六维审查得到 `P1=2, P2=1, P3=1`、`Important=3`。静态 monotonic 的 250 ms 无限等待、factory ownership 后 close 抢跑、prepare generation 非原子和 diagnostic id 长期保留均补确定性覆盖；generation RED 为 `ValueError: lifecycle_generation must not move backwards`，修复后聚焦 GREEN。
3. 多失败首错 RED 中 subscription close 的 `RuntimeError` 错误覆盖先发生的 sensor timeout；GREEN 后保留 `TimeoutError("...250 ms")`。retired identity RED 证明成功重试后 id 仍残留；GREEN 在释放 service 所有权时同步 `discard(identity)`。两条聚焦复验均为 `1 passed`。
4. 修复后首轮独立复审又确定两个前半窗口 `P1 / Important`：close 已完成后 factory 仍被调用一次，以及 candidate prewarm 期间 abort 会恢复旧 worker。新增 close RED 的 `factory_calls` 实际多一项；candidate-active abort 与 abort-active commit 两条 RED 均为 `DID NOT RAISE RuntimeError`。
5. GREEN 在 factory ownership 登记前锁内重验 runtime open；有旧 LiDAR 的 abort 与 commit 双向复用通用 reservation。三条新并发测试和既有无 LiDAR 双 abort 回归均通过，且 factory/spawn、service close 和订阅仍在 runtime condition 外执行。
6. 最终新鲜复验为 Task 8 选择器 `26 passed, 33 deselected`、worker + pause/rebuild `110 passed`、runtime + integration `125 passed`；`py_compile` 和限定 `git diff --check` 均 rc=0。第二轮独立只读六维复审为 `P0/P1/P2/P3=0`、`Critical/Important/Minor=0`；未运行真实 eCAL。

Run:

```bash
conda run -n slope-sim python -m pytest -q tests/test_interface_pause_rebuild.py \
  -k "rebuild and (service or generation or candidate or retired or rollback)"
```

Stop：两个 worker 同时 native scan、持 runtime condition 等 spawn、candidate ready 前销毁旧 service，或 cleanup 错误反转已提交世界。

## Task 9：故障隔离与 normal/force close

**Files:**

- Modify: `slope_sim/lidar_worker.py`
- Modify: `slope_sim/interfaces/runtime.py`
- Test: `tests/test_lidar_worker.py`
- Test: `tests/test_interface_runtime_integration.py`
- Test: `tests/test_interface_pause_rebuild.py`

- [x] RED `test_single_scan_failure_degrades_only_requested_topic_once`。
- [x] RED `test_unknown_scene_state_faults_both_lidar_topics_once`。
- [x] RED `test_worker_protocol_failure_faults_both_lidar_topics_only_once` 和 `test_prepared_identity_failure_never_publishes_payload`。
- [x] RED `test_normal_close_drains_pending_before_transport_and_logger`。
- [x] RED `test_force_close_cancels_pending_and_never_reports_success_fence`。
- [x] RED `test_close_terminates_only_owned_child_after_join_timeout`。
- [x] GREEN runtime 只消费 typed events 更新 tracker；wheel/RTK/IMU/mailbox/watchdog 继续。
- [x] 正常 close 先 drain 合法 in-flight/pending 并发布，再 stop/join；failed/timeout 才 force cancel/terminate。保持既有 close trace 顺序。

实际 RED/GREEN、回归与审查记录（2026-08-04）：

1. normal close、force close、typed Stop/Stopped ACK、owned child force reap、worker failure isolation、prepared identity、P0 result snapshot 均先以对应行为 RED 锁定，再完成最小实现。normal close 会在 closing generation 内排空并发布原始 prepared frame，随后才 Stop/Stopped ACK 与 transport/logger 终结；force 路径撤销 in-flight/pending，且不消费 ACK。
2. 扩大回归发现同步 transport callback 内 `pause()` 会让已经成功交付的当前消息不提交统计。三条既有回归先得到 `3 failed in 0.67s`（`message_count == 0`），新增仅供 transport 返回后使用的 generation/lifecycle 提交门控后，同一命令为 `3 passed in 0.50s`；发布前 gate 继续拒绝 paused。
3. 新鲜验证：Task 9 selector `44 passed, 129 deselected in 9.27s`；`test_lidar_worker.py + test_interface_runtime_integration.py + test_interface_pause_rebuild.py` 为 `173 passed in 19.13s`；`test_interface_runtime.py + test_ecal_process_roundtrip.py -m "not ecal"` 为 `228 passed, 4 deselected in 1.33s`；`test_ecal_transport.py` 为 `91 passed in 1.95s`。相关 `py_compile` 与 tracked-file `git diff --check` 均无错误。
4. 独立六维只读审查覆盖需求完整性、逻辑正确性、边界情况、代码质量、测试覆盖和实际运行结果，结论 `Critical=0`、`Important=0`、`Minor=0`。未运行真实 eCAL；该外部门禁仍留在 Task 13 的新授权门。

Run:

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_lidar_worker.py \
  tests/test_interface_runtime_integration.py \
  tests/test_interface_pause_rebuild.py \
  -k "failure or protocol or unknown_scene or normal_close or force_close or owned_child"
```

Stop：service 故障关闭轮控、事件被重复消费、错误 frame 被修补后发布，或 terminate 非 owned process。

## Task 10：按 actual transport mode 接线并原子回滚初始化

**Files:**

- Modify: `slope_sim/simulation.py`
- Test: `tests/test_interface_runtime_integration.py`

- [x] RED `test_production_session_creates_worker_only_for_actual_ecal_mode`。
- [x] RED `test_auto_local_fallback_does_not_create_worker`。
- [x] RED `test_worker_start_failure_closes_all_session_resources`。
- [x] RED `test_runtime_or_relay_failure_after_worker_ready_closes_child_once`，锁定所有权转移前后两条失败路径。
- [x] 保持现有 initial `poll_peer_state()` before snapshot/attach 测试 GREEN。
- [x] GREEN transport 创建后读取其 actual mode；只为 actual eCAL 构造 service/factory，再把唯一所有权交给 runtime。
- [x] worker/session 构造失败关闭 service、logger、transport、backend；strict eCAL 和已选 eCAL 的 auto 不二次降级 local。

实际 RED/GREEN、回归与审查记录（2026-08-04）：

1. 首批新增 session 合同用例得到 `4 failed, 1 passed`：strict eCAL 未启动 worker、worker start failure 未向上传播、runtime/relay 失败后 ready service 未关闭各自明确失败；`auto -> local` 因既有同步路径首次即 GREEN，仅作为防回归记录，不伪称 RED。
2. GREEN 在 `create_transport()` 后先执行一次 `poll_peer_state() -> snapshot()`，以该实际 mode 决定是否创建 v1、无 body-id 的 `LidarWorkerWorldSpec`、ready service 与 digest-checked rebuild factory。runtime 构造前失败由入口先 close idle/force service，runtime 接管后失败只经 `runtime.close()` 回收。worker start failure 的 logger 创建顺序先被测试指出，已前移至启动 worker 前。
3. 审查补强后 Task 10 selector 为 `12 passed, 54 deselected in 3.51s`，完整 integration 为 `66 passed in 4.23s`；受影响 worker/runtime/pause-rebuild 为 `190 passed in 19.65s`，process-roundtrip/simulation-smoke 非 eCAL 为 `203 passed, 4 deselected in 17.27s`。相关 `py_compile` 与 tracked-file `git diff --check` 无错误。
4. 独立六维只读审查初次结论 `Critical=0`、`Important=0`、`Minor=2`；两个 Minor 均为覆盖不足，已补同一 snapshot identity 与 `auto` 已选 eCAL 的启动失败回滚断言并通过上述 selector/full integration。未运行真实 eCAL。

Run:

```bash
conda run -n slope-sim python -m pytest -q tests/test_interface_runtime_integration.py \
  -k "production_session or actual_mode or worker_start or worker_ready or initial_peer"
```

Stop：依据请求 mode 而非 actual mode、破坏 relay 首次观测顺序，或 worker 失败后静默 local fallback。

## Task 11：把 P0 20 障碍物放入初始 WorkerWorldSpec

**Files:**

- Modify: `scripts/ecal_simulation_runtime.py`
- Test: `tests/test_ecal_process_roundtrip.py`

- [x] RED `test_normal_load_scene_contains_twenty_obstacles_before_interface_session`：缺少 `_bootstrap_normal_load_scene()` 时在测试函数内明确 FAILED。
- [x] RED `test_runtime_ready_file_follows_worker_preflight_for_twenty_obstacles`：入口 source 未包含 bootstrap 调用时明确 FAILED。
- [x] GREEN 先用无 interface runtime 的 bootstrap coordinator 完成 20 障碍事务并取得完整逻辑 document；再创建 session/worker 和正式 coordinator。
- [x] 只有初始 digest/ready 已覆盖 20 障碍且正式 coordinator 绑定完成后才写 `ready_file`。
- [x] 不使用运行中阻塞式 `refresh_scene_bindings()` worker 预热代替启动顺序修复。

实际 TDD 与复验：首批 RED 为 `2 failed`，最小实现后同一选择器为 `2 passed`。六维审查补充的受控入口测试证明 session 接收完整 20 障碍 document，且 `ready_file` 写入前 session 已创建；该测试属于已有 GREEN 分支的组合覆盖，不伪称事后 RED。最终 Task 11 selector 为 `3 passed, 156 deselected`，`tests/test_ecal_process_roundtrip.py -m "not ecal"` 为 `155 passed, 4 deselected`；相关 `py_compile` 与 `git diff --check` 均通过。未运行真实 eCAL。

Run:

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_ecal_process_roundtrip.py \
  -k "twenty_obstacles or worker_preflight or runtime_ready_file"
```

Stop：空障碍 WorldSpec、首次计时 scan 才创建 20 个镜像 body，或 bootstrap coordinator 与正式 runtime document 不一致。

## Task 12：真实 DIRECT 等价与本地实时 verifier

阻断记录（2026-08-04）：已为缺失 verifier 观察到函数内 RED，最小 GREEN 使用 production `start_lidar_worker()`、`LidarScanService.from_worker_handle()`、DIRECT、20 障碍、240 Hz `DeadlinePacer` 和正常 child 回收；契约测试为 `1 passed`。独立 CLI 的短窗曾得到 4 captures/4 completes、最大 heartbeat `8.89 ms`，但计划要求的正式 `--windows 10 --duration-sec 5` 在首个违规帧停止：总 heartbeat `28,466,887 ns`，其中 `poll=26,966,002 ns`、`capture=1,372,407 ns`、`physics=126,838 ns`。这证明完整 Python `PreparedLidarFrame` 经 Pipe 的 parent 反序列化/重构超过 20 ms，不得通过放宽门限、改用同步路径或跳过轮询掩盖。按本 Task Stop 条件，后续必须回到协议/所有权设计评审，任务保持未完成，禁止进入 Task 13 或申请真实 eCAL 授权。

**Files:**

- Modify: `tests/test_lidar_pointcloud_direct.py`
- Create: `scripts/verify_lidar_worker_realtime.py`
- Test: `tests/test_lidar_worker.py`
- Test: `tests/test_ecal_process_roundtrip.py`

- [x] RED 参数化平面、斜面、高尔夫、静态/移动障碍、unknown/miss、前后 mount、top view 和零命中，worker/sync wire bytes 不一致时 FAILED。
- [x] RED `test_realtime_verifier_uses_production_spawn_service_and_contract`：禁止 subclass fallback、同步 local 或测试 worker 冒充生产路径。
- [x] GREEN 新 verifier 使用真实 DIRECT、20 障碍、240 Hz pacer、双 2880 射线和 production service，不初始化 eCAL。
- [x] 连续 10 个 5 秒窗口；任何窗口 heartbeat `>20 ms`、capture `>100 ms`、overrun/drop/failure、sim/wall 失败或残留 child，立即停止并回到设计评审。

实际 TDD、修复与复验（2026-08-04）：

1. 初版正式 10x5 门禁在首个违规帧得到 `heartbeat=28,466,887 ns`，其中 `poll=26,966,002 ns`；根因是 headless worker 经 Pipe 回传完整 `PreparedLidarFrame`，父端反序列化和逐点重构占用了 heartbeat。先以 `test_spawned_headless_worker_returns_compact_payload`、`test_async_headless_lidar_payload_publishes_without_parent_pointcloud` 和 production verifier 合同形成行为约束，再实现仅含身份、预编码 bytes 和 scan 耗时的 `PreparedLidarPayload`。headless runtime 直接发布 bytes，不解码、重编码或写 Dashboard latest；带 top-view 的 GUI 请求仍回传完整原子 frame。
2. DIRECT worker/sync 场景矩阵覆盖 flat/no obstacle/front/headless、slope/static/front/top-view、golf_heightfield/moving/rear/top-view 与 flat/miss/rear/headless zero-hit，结果 `4 passed`；`tests/test_lidar_pointcloud_direct.py tests/test_lidar_worker.py` 新鲜回归为 `69 passed in 22.00s`。
3. 最终正式连续门禁使用持久终端完成原命令：`capture_count=954`、`completed_count=954`、`failure_count=0`、`overrun_count=0`、`max_heartbeat_ns=6,983,880`、最低单窗口 `sim_wall_ratio=0.9999610953035322`、`worker_exitcode=0`。审查补强的逐窗口 oracle 会在每个五秒窗口结束立即校验，且明确拒绝非零或缺失 exitcode。阶段三 verifier 的后续持续 backlog 由其手写超期循环未让出 GIL 引起；以共享 `DeadlinePacer` 的 RED/GREEN 修复后，21 项 verifier 全部通过，性能项为 `accepted=1200/1200`、`backlog=False`。另修复 `_PeerStateRelay.attach()` 在持有普通锁时同步 poll 的自锁，精确 eCAL transport 回归通过。
4. 最终本地回归：`tests/test_stage3_interface_verifier.py` 为 `34 passed`，`tests/test_ecal_transport.py` 为 `91 passed`，session selector 为 `12 passed, 55 deselected`，`python -m pytest -q -m "not ecal"` 为 `2402 passed, 4 deselected in 111.10s`。相关 `py_compile` 和限定 `git diff --check` 无错误。全量 pytest 输出有 eCAL 配置/时间同步库警告，但退出码为 0；没有运行真实 eCAL invocation。

Focused/expanded:

```bash
conda run -n slope-sim python -m pytest -q \
  tests/test_lidar_pointcloud_direct.py tests/test_lidar_worker.py
conda run -n slope-sim python scripts/verify_lidar_worker_realtime.py \
  --windows 10 --duration-sec 5
conda run -n slope-sim python scripts/verify_stage3_interfaces.py
conda run -n slope-sim python -m pytest -q -m "not ecal"
```

## Task 13：独立审查、更新事实状态并停在真实授权门

**Files:**

- Modify: `docs/superpowers/plans/2026-07-31-stage4-master-implementation.md`
- Modify: `docs/阶段四交付报告.md`
- Modify: `README.md` only if current behavior/status text is stale

- [x] 保存每轮实际 RED/GREEN 命令和结果，只写真实形成的证据。
- [x] 运行相关 `py_compile`、`git diff --check`、聚焦回归、阶段三 verifier 和全量非 eCAL 回归。
- [x] 启动独立只读六维审查，从需求完整性、逻辑正确性、边界情况、代码质量、测试覆盖和实际运行结果审查；Critical/Important 必须为 0。
- [x] 本地门全部通过后停止，不沿用历史授权。向用户说明负载和时长，只申请紧邻下一条 `active_steering_4wd 4+2` invocation 的新授权。
- [x] 获授权的 `4+2` 已 PASS：`results/stage4/p0-active-steering-4wd-retest-20260804T152217+0800/`。
- [x] 获授权的 `df_back 2+0` 已 PASS：`results/stage4/p0-df-back-2wd-20260804T153125+0800/`；两车型 P0 均通过，Task 2 阻断已解除。

独立六维审查（2026-08-04）：初审发现逐窗口 sim/wall 被总和掩盖、非零 worker exit 未显式拒绝的 Important；两项均已 RED/GREEN 修复并复审。最终结论为 `Critical=0`、`Important=0`、`Minor=0`。复审确认 compact headless 不 decode/re-encode、GUI frame 原子性、1 in-flight + 1 pending、relay 无自锁、DeadlinePacer 复用及 child 回收。最终全量非 eCAL 回归为 `2404 passed, 4 deselected in 137.49s`，exit code 0；输出的 eCAL 配置/时间同步库提示不构成真实 eCAL invocation。

授权复测（2026-08-04 14:46）：**FAIL，已按硬停止门中止且未重跑。** 预检仅保留桌面 `Xvfb :1`，eCAL 6.1.1 可导入且系统负载为 `0.27/0.09/0.06`。runtime 与 peer 都创建资源后，peer 却在六话题 discovery 完成前写入 `ready`；共同 start 随即让 runtime warmup 发布。eCAL 官方 `Publisher.send()` 在尚无订阅者时按协议返回 `False`，当前 adapter 将其记录为 topic error；peer 随后发现连接并写入 measurement-start marker，runtime 立即 fence，尚未有成功发送清除 warmup error，故报 `normal-load start transport snapshot is not active`。本次证据为 `results/stage4/p0-active-steering-4wd-retest-20260804T144603+0800/interface-logs/interfaces_1785825984655018630_af2ca790.events.jsonl`，其中 179 条 `publish_failed` 均为该 `send=False`。

后续本地修复（2026-08-04）：先加入 `test_simulation_peer_marks_ready_only_after_all_resources_are_discovered`，RED 为进入 `_wait_for_start()` 时 discovery 调用数 `0`；随后把 peer 的全端点 discovery 移至 `ready` 写入前，GREEN 为 `1 passed, 159 deselected`。扩大非真实 eCAL 回归 `tests/test_ecal_process_roundtrip.py tests/test_ecal_transport.py -m "not ecal"` 为 `247 passed, 4 deselected`，相关 `py_compile` 与 `git diff --check` 无错误。修复不把 `send=False` 伪装为成功，且保留正式窗口的全 topic active/connected 断言。必须取得一条新的独立 `4+2` 授权后才能验证真实结果；不得重跑本次失败命令。

授权复测（2026-08-04 14:59）：**FAIL，已按硬停止门中止且未重跑。** 新单次授权的预检仅发现桌面常驻 `Xvfb :1`，负载为 `0.01/0.05/0.07`、可用内存约 `10 GiB`。peer discovery 修复已令 `measurement_start.ack` 写出并显示六话题 active，但 runtime 在正式测量初始化中因 `NameError: name 'log_start' is not defined` 退出，未形成 5 秒正式窗口。本次证据目录为 `results/stage4/p0-active-steering-4wd-retest-20260804T145908+0800/`；其 `measurement_start.ack` 同时保留了此前 warmup 的 `send=False` 错误计数，不能被解释为本轮新根因。

后续本地 TDD 修复（2026-08-04）：新增 `test_measurement_start_log_sample_uses_capture_snapshot`，RED 命令 `conda run -n slope-sim python -m pytest -q tests/test_ecal_process_roundtrip.py -k 'measurement_start_log_sample_uses_capture_snapshot'` 得到 `1 failed, 160 deselected`，明确断言首个 logger 样本未使用 `start_capture.log_snapshot`。最小 GREEN 将遗留的 `log_start` 两处访问替换为同一 start capture 的 logger 快照；同一命令得到 `1 passed, 160 deselected`。相关非真实 eCAL 回归 `conda run -n slope-sim python -m pytest -q -m 'not ecal' tests/test_ecal_process_roundtrip.py tests/test_ecal_transport.py` 为 `248 passed, 4 deselected`，相关 `py_compile` 与 `git diff --check` 无错误。真实 P0 仍为 FAIL；必须重新取得另一条仅覆盖紧随其后的 `4+2` invocation 的授权，`2+0` 与 Task 2 继续阻断。

授权复测（2026-08-04 15:14）：**FAIL，已按硬停止门中止且未重跑。** 预检无 pytest、PyBullet 或 eCAL 竞争进程，负载为 `0.06/0.06/0.07`、可用内存约 `10 GiB`。`log_start` 修复已生效，但 runtime 的全部 warmup output native send 仍返回 `False`，导致 measurement-start transport snapshot 不 active。证据时间线显示 peer 于 `15:14:30.870` 写 ready、共同 start 紧随其后，而 runtime 在仿真时间 `16,666,667..983,333,333 ns` 记录 136 条 output send failed；这证明资源 peer count 只代表控制面发现，尚未证明共享内存数据面可投递。证据目录为 `results/stage4/p0-active-steering-4wd-retest-20260804T151427+0800/`。

后续本地 TDD 修复（2026-08-04）：新增 `test_simulation_peer_waits_for_output_data_plane_before_warmup`，RED 命令 `conda run -n slope-sim python -m pytest -q tests/test_ecal_process_roundtrip.py -k 'simulation_peer_waits_for_output_data_plane_before_warmup'` 得到 `1 failed, 161 deselected`，因为没有 data-plane preflight helper。GREEN 新增 `_wait_for_output_delivery()`：共同 start 后，peer 必须已接收五个 output topic 各至少一帧才发送 warmup command；它使用既有 12 秒协议上限而非任意 sleep。相同命令 GREEN 为 `1 passed, 161 deselected`。相关非真实 eCAL 回归为 `249 passed, 4 deselected`，相关 `py_compile` 与 `git diff --check` 无错误。此修复尚未经过新的真实 `4+2`，P0 仍为 FAIL；必须重新取得另一条仅覆盖紧随其后的 `4+2` invocation 的授权，`2+0` 与 Task 2 继续阻断。

手动授权复测（2026-08-04 15:22）：**PASS。** 用户在空闲主机上执行正式 `active_steering_4wd 4+2` 命令，退出码为 `0`。verifier 输出 command/wheel-state 各 `500` 条、四路传感器各 `50` 条，所有 topic 为 `active`；`1199` 步推进 `4.995833 s` 仿真时间，`sim/wall=0.998075`，20 障碍 normal-load 日志 `1200` 条、`max_pending=1`、零丢失、末尾 pending 为零，wheel 原始日志/peer 为 `500/500` 完整匹配，双方 `clean_shutdown=true`。独立读取 `runtime-result.json` 复核障碍数、逐窗口比率、日志队列与正常关闭；证据目录为 `results/stage4/p0-active-steering-4wd-retest-20260804T152217+0800/`。接口事件文件中预热和后续故障注入阶段的 `publish_failed` 位于正式 start/end snapshot 外，不能与正式窗口的逐帧 PASS 混淆。`4+2` 已解除，下一步仅可申请并执行独立的差速 `2+0`。

授权复测（2026-08-04 15:31）：**PASS。** 新宿主预检无竞争进程，执行正式 `df_back 2+0` 的退出码为 `0`。正式窗口 command 为 `500`、wheel 为 `499` 且原始日志/peer 均为 `499/499`，前后 LiDAR、RTK、IMU 各 `50` 条；全部 topic active。`1199` 步推进 `4.995833 s`，`sim/wall=0.998075`，20 障碍日志窗口为 `1199` 条、`max_pending=1`、零丢失、末尾 pending 为零，双方 `clean_shutdown=true`。独立读取 start/end ACK、runtime/peer JSON 与接口日志确认正式窗口无新增 transport error/drop，证据目录为 `results/stage4/p0-df-back-2wd-20260804T153125+0800/`。两车型真实 P0 均通过，Task 2 现在可开始。

独立六维复审（2026-08-04）：审查任务 `/root/p0_six_dimension_review` 只读检查实现、测试与两份真实 evidence，未启动 eCAL/PyBullet，未修改文件。结论为 `Critical=0`、`Important=0`、`Minor=0`。复审确认 eCAL 实际模式 worker/local 同步边界、20 障碍 preflight、`1 in-flight + 1 pending`、fence/epoch/generation/关闭路径均有实现与测试；peer 的 discovery 与五话题 data-plane preflight 阻止控制面就绪竞态。两轮 start/end snapshot 保留的 error 均为窗口前累计，正式窗口差分为零且 topic active；P0 的残余风险仅为后续阶段未覆盖的 GUI eCAL、RViz2/Livox Viewer 与干净机迁移。

## 完成定义

- Task 0-13 全部有实际 TDD/验证证据。
- local/无节拍 DIRECT 仍同步；actual eCAL 使用单个持久化 spawn worker。
- worker ready 已覆盖完整场景与双雷达 full-batch encode；P0 初始场景包含 20 障碍。
- measurement start/end/final 和 normal close 不存在迟到 LiDAR 穿越 snapshot/ACK。
- 本地 10 窗口门、扩大回归和独立六维审查通过。
- 获单条授权的真实 `4+2`、随后独立授权的 `2+0` 都通过未修改 oracle。

阶段四当前状态：Task 1 与 P0 已完成；Task 2 可以开始，A-E 仍须按总计划顺序执行。
