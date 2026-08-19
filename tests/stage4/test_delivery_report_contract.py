# 阶段四交付报告合同测试：锁定已验证 A/B2 与未执行后续阶段边界。
from pathlib import Path


def test_stage4_report_records_verified_phase0_without_future_pass_claims() -> None:
    report = Path("docs/阶段四交付报告.md")
    assert report.is_file(), "stage four delivery report is not implemented"
    text = report.read_text(encoding="utf-8")
    assert "阶段 A-E 已完成对应实现与真实门禁" in text
    assert "最终全阶段独立六维审查\n> `Critical=0`、`Important=0`、`Minor=0`" in text
    assert "阶段 A 已完成" in text
    assert "最新独立六维审查为 P0=0，P1=0；阶段 A 已完成" in text
    assert "P0 本地异步 LiDAR Worker" in text
    assert "phase0-real-authorized-20260810T152526+0800" in text
    assert "phase0-real-authorized-20260810T152832+0800/phase0-result.json" in text
    assert "phase0-real-reauthorized-20260810T173454+0800/phase0-result.json" in text
    assert "phase0-real-reauthorized-20260810T180741+0800/phase0-result.json" in text
    assert "phase0-real-reauthorized-20260810T183206+0800/phase0-result.json" in text
    assert "a-full-non-ecal-process-evidence-20260810T185222+0800.log" in text
    assert "2722 passed, 8 deselected in 150.24s" in text
    assert "a-full-non-ecal-process-evidence-20260810T191041+0800.log" in text
    assert "a-full-non-ecal-init-trace-verification-20260810T191041+0800.log" in text
    assert "2726 passed, 8 deselected in 147.95s" in text
    assert "396 次 hook 加载（394 个唯一 PID），其中 388 次成功安装、8 次 eCAL 不可用，原生 eCAL initialize 调用为 0" in text
    assert "| Python/C++ raw Phase-0 | PASS：" in text
    assert "| 单 LiDAR/三点 RTK runtime | B2 PASS：" in text
    assert "240/100/10 Hz cadence 均有聚焦 TDD" in text
    assert "真实 eCAL `df_mid` 5 秒窗口已通过" in text
    assert "b2-v2-runtime-real-ecal-async-retest-20260810T210908+0800" in text
    assert "四车型三场地 5 秒 headless 性能门已通过" in text
    assert "12 passed, 15 deselected in 72.02s" in text
    assert "27 passed in 87.60s" in text
    assert "v2 PySide6 adapter 已实现并通过 offscreen GUI 测试" in text
    assert "独立 launcher 已通过 offscreen 集成测试" in text
    assert "Xvfb X11 窗口 smoke 已通过" in text
    assert "launcher 现强制 verified peer、墙钟 pacing 与关闭前 wait_idle" in text
    assert "GUI timer 对同一 LiDAR snapshot 重复解码/重绘" in text
    assert "b2-v2-dashboard-real-ecal-isolated-final-20260810T222608+0800" in text
    assert "native `published=650/error=0/dropped=0`" in text
    assert "B2 已关闭" in text
    assert "| C++ SDK/Recorder | C1 PASS：最新独立六维复验 P0=0、P1=0、P2=0；C2 已完成" in text
    assert "Recorder 的 `safe_stop_required=true` 会终止唯一 Command" in text
    assert "c2-recorder-command-runtime-five-second-20260811T011537+0800" in text
    assert "Recorder 以 `clean_shutdown=true` 原子完成 3,671,694-byte MCAP、`recorded_count=1150`" in text
    assert "第五次 C 阶段独立审查发现" in text
    assert "c2-recorder-command-subscriber-runtime-five-second-20260811T013849+0800" in text
    assert "最终独立六维审查结论为 P0=0、P1=0、P2=0；C 阶段已关闭" in text
    assert "test_cpp_subscriber_rejects_a_window_that_observed_multiple_publishers" in text
    assert "test_cpp_all_output_subscriber_rejects_competing_runtime_publisher_after_verified_peer" in text
    assert "ValidateRawWheelCommand" in text
    assert "ValidateRawV2Payload" in text
    assert "safe_stop_required=true" in text
    assert "runtime 与只读 Recorder 同时消费 command topic" in text
    assert "WheelCommandLease" in text
    assert "CommandInstanceLock" in text
    assert "第六个单元的 RED 为 `1 failed, 32 deselected`" in text
    assert "第七个单元的 RED 为 `1 failed, 33 deselected`" in text
    assert "第八个单元实现了 Supervisor" in text
    assert "第九个单元为正式五秒窗口新增 Recorder `--duration-ms`" in text
    assert "七个 RED 合计为 `10 failed`；最终聚焦 GREEN 为 `16 passed, 11 deselected`" in text
    assert "slope_sim_stage4_command" in text
    assert "| 最终真实联合负载 | PASS：已安装 release 的 C++ Command、Subscriber、Recorder 与 Python/PyBullet runtime 完成五秒 eCAL/MCAP 会话，所有 participant clean shutdown |" in text
    assert "| 真实 RViz2 | PASS：GNOME `DISPLAY=:1` 显示 RViz 窗口，OpenGL 4.6 初始化，SIGINT 后 exit 0 |" in text
    assert "| Livox Viewer 2 | loopback-only 启动 smoke：" in text
    assert "| Livox Viewer 2 | PASS：" not in text
    assert "verify_livox_viewer2_linux.py" in text
    assert "stage4-livox-viewer-no-network-evidence-20260812n.vqAEcI" in text
    assert "network-evidence.json" in text
    assert "launcher_pid=launcher_pgid=3614500" in text
    assert "viewer_pid=3614508" in text
    assert "viewer_pgid=3614500" in text
    assert "LoadMap(/Game/Maps/Viewer)" in text
    assert "launcher_exit_code=241" in text
    assert "仅见 loopback、无外部网络接口" in text
    assert "仅有 `lo`" in text
    assert "多网卡/代理网络环境" in text
    assert "| 干净机迁移（核心） | PASS：Ubuntu 24.04 容器联网安装，`current -> releases/4.0.0`，安装后无网络 Command dry-run 通过 |" in text


def test_stage4_entry_documents_defer_current_completion_to_delivery_report() -> None:
    """入口文档不能保留与交付报告冲突的阶段四未开始状态。"""
    documents = (
        Path("README.md"),
        Path("ARCHITECTURE.md"),
        Path("3d仿真平台需求规格.md"),
    )

    for document in documents:
        text = document.read_text(encoding="utf-8")
        assert "docs/阶段四交付报告.md" in text
        assert "B-E 尚未开始" not in text
        assert "阶段四 E | `.run` 联网安装器和最终联合验收，尚未执行" not in text

    assert "动态避障均不在当前阶段四范围" in Path("README.md").read_text(
        encoding="utf-8"
    )


def test_stage4_report_records_test_reorganization_audit() -> None:
    """精简不能以静默丢失测试覆盖为代价。"""
    text = Path("docs/阶段四交付报告.md").read_text(encoding="utf-8")

    assert "57 个旧平铺测试中，54 个迁移" in text
    assert "3 个只验证已取消的离线缓存/网络隔离门" in text
    assert "迁移后的四个所有权目录有 `86` 个测试文件、无重复 basename" in text
    assert "当前总数为 `88` 个测试文件" in text
    assert "pytest --collect-only -q` 收集 `2759` 项" in text


def test_stage4_handoff_records_auditable_final_candidate_and_review_state() -> None:
    """最终交接必须区分唯一候选、历史候选与尚待复审的状态。"""
    report = Path("docs/阶段四交付报告.md").read_text(encoding="utf-8")
    architecture = Path("ARCHITECTURE.md").read_text(encoding="utf-8")
    plan = Path(
        "docs/superpowers/plans/2026-08-10-stage4-b-e-lightweight-implementation.md"
    ).read_text(encoding="utf-8")

    assert "e-run-candidate-conda-direct-20260812" in report
    assert "04f93566bc97b0568218046bb9f96e6ec8a2add555abd7475488bc2cf3361783" in report
    assert "其余 `.run` 均为保留的历史 TDD/安装证据" in report
    assert "阶段 A-E 已完成对应实现与真实门禁" in report
    assert "第二次独立只读复审确认 `Critical=0`、`Important=0`、`Minor=0`" in report
    assert "仍需在 B-E 实施和验收" not in architecture
    assert "状态索引：B-E 已完成" in plan
