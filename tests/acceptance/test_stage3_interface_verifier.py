# 阶段三接口验收测试：锁定报告值、聚合顺序和失败退出语义。
from __future__ import annotations

from dataclasses import FrozenInstanceError
import inspect

import pytest

from scripts import verify_stage3_interfaces as verifier
from slope_sim.interfaces.backlog import _has_sustained_backlog
from slope_sim.interfaces.logging import InterfaceLogSnapshot


def _check(name: str, passed: bool = True, details: str = "ok") -> verifier.VerificationCheck:
    """构造轻量验收结果，避免单元测试启动 DIRECT、eCAL 或 GUI。"""
    return verifier.VerificationCheck(name, passed, details)


def test_verification_check_keeps_strict_values_and_is_frozen() -> None:
    check = verifier.VerificationCheck("wheel_rates", True, "100.0 Hz")

    assert check.name == "wheel_rates"
    assert check.passed is True
    assert check.details == "100.0 Hz"
    with pytest.raises(FrozenInstanceError):
        check.passed = False


@pytest.mark.parametrize(
    ("name", "passed", "details", "field_name"),
    [
        ("", True, "ok", "name"),
        (1, True, "ok", "name"),
        ("wheel_rates", 1, "ok", "passed"),
        ("wheel_rates", True, None, "details"),
    ],
)
def test_verification_check_rejects_invalid_strict_values(
    name: object,
    passed: object,
    details: object,
    field_name: str,
) -> None:
    with pytest.raises(ValueError, match=field_name):
        verifier.VerificationCheck(name, passed, details)


def test_summarize_counts_checks_and_builds_exact_report_lines() -> None:
    checks = (
        _check("wheel_rates", details="100.0 Hz"),
        _check("lidar_front", False, "9.1 Hz"),
        _check("imu", details="10.0 Hz"),
    )

    summary = verifier.summarize(checks)

    assert summary.lines == (
        "PASS wheel_rates 100.0 Hz",
        "FAIL lidar_front 9.1 Hz",
        "PASS imu 10.0 Hz",
    )
    assert summary.pass_count == 2
    assert summary.fail_count == 1
    assert summary.final_line == "SUMMARY pass=2 fail=1"


@pytest.mark.parametrize("failed_index", [0, 1, 2])
def test_exit_code_is_nonzero_when_any_check_fails(failed_index: int) -> None:
    checks = tuple(_check(f"check_{index}", index != failed_index) for index in range(3))

    assert verifier.exit_code(checks) != 0


def test_exit_code_is_zero_when_every_check_passes() -> None:
    assert verifier.exit_code((_check("first"), _check("second"))) == 0


def test_main_prints_every_check_and_summary_and_propagates_failure(monkeypatch, capsys) -> None:
    checks = (
        _check("proto_contract", details="schema=v1"),
        _check("dashboard", False, "missing snapshot"),
    )
    monkeypatch.setattr(verifier, "run_stage3_checks", lambda: checks)

    result = verifier.main()

    assert capsys.readouterr().out.splitlines() == [
        "PASS proto_contract schema=v1",
        "FAIL dashboard missing snapshot",
        "SUMMARY pass=1 fail=1",
    ]
    assert result != 0


def _stub_stage3_runners(monkeypatch) -> tuple[str, ...]:
    """用完整的轻量子检查替换真实仿真门禁，并返回期望聚合顺序。"""
    single_runner_names = (
        "proto_and_topic_contract",
        "timeout_and_steering_hold",
        "scheduler_100_10_hz",
        "static_and_moving_obstacle_lidar",
        "lidar_collision_contact",
        "pause_rebuild_and_edge_switch",
        "scene_roundtrip",
        "interface_log_roundtrip",
        "dashboard_snapshot_and_chart",
        "per_topic_ecal_status",
    )
    single_runners: dict[str, str] = {
        "run_proto_and_topic_contract_check": single_runner_names[0],
        "run_timeout_and_steering_hold_check": single_runner_names[1],
        "run_100_10_hz_scheduler_check": single_runner_names[2],
        "run_static_and_moving_obstacle_lidar_check": single_runner_names[3],
        "run_lidar_collision_contact_check": single_runner_names[4],
        "run_pause_rebuild_and_edge_switch_check": single_runner_names[5],
        "run_scene_roundtrip_check": single_runner_names[6],
        "run_interface_log_roundtrip_check": single_runner_names[7],
        "run_dashboard_snapshot_and_chart_check": single_runner_names[8],
        "run_per_topic_ecal_status_check": single_runner_names[9],
    }
    for runner_name, check_name in single_runners.items():
        monkeypatch.setattr(verifier, runner_name, lambda name=check_name: _check(name))

    wheel_names = tuple(
        f"wheel_{model}"
        for model in ("df_front", "df_mid", "df_back", "active_steering_4wd")
    )
    lidar_names = tuple(f"lidar_{terrain}" for terrain in ("flat", "slope", "golf_heightfield"))
    truth_names = tuple(f"truth_{terrain}" for terrain in ("flat", "slope", "golf_heightfield"))
    monkeypatch.setattr(verifier, "run_four_model_wheel_checks", lambda: tuple(map(_check, wheel_names)))
    monkeypatch.setattr(verifier, "run_three_terrain_lidar_checks", lambda: tuple(map(_check, lidar_names)))

    def run_truth_checks(*, tolerance: float) -> tuple[verifier.VerificationCheck, ...]:
        assert tolerance == pytest.approx(1e-4)
        return tuple(map(_check, truth_names))

    monkeypatch.setattr(verifier, "run_three_terrain_truth_sensor_checks", run_truth_checks)

    def run_performance_check(*, max_dashboard_gap_sec: float) -> verifier.VerificationCheck:
        assert max_dashboard_gap_sec == pytest.approx(0.100)
        return _check("twenty_obstacle_queue_performance")

    monkeypatch.setattr(verifier, "run_twenty_obstacle_queue_performance_check", run_performance_check)

    return (
        single_runner_names[0],
        *wheel_names,
        single_runner_names[1],
        single_runner_names[2],
        *lidar_names,
        single_runner_names[3],
        single_runner_names[4],
        *truth_names,
        single_runner_names[5],
        single_runner_names[6],
        single_runner_names[7],
        single_runner_names[8],
        single_runner_names[9],
        "twenty_obstacle_queue_performance",
    )


def test_run_stage3_checks_returns_ordered_tuple_with_unique_names(monkeypatch) -> None:
    expected_names = _stub_stage3_runners(monkeypatch)

    checks = verifier.run_stage3_checks()
    names = tuple(check.name for check in checks)

    assert isinstance(checks, tuple)
    assert names == expected_names
    assert len(names) == 21
    assert len(names) == len(set(names))


def test_run_stage3_checks_rejects_duplicate_check_names(monkeypatch) -> None:
    _stub_stage3_runners(monkeypatch)
    duplicate = (_check("wheel_df_front"),)
    monkeypatch.setattr(verifier, "run_three_terrain_lidar_checks", lambda: duplicate)

    with pytest.raises(ValueError, match="duplicate.*wheel_df_front"):
        verifier.run_stage3_checks()


def test_wheel_gate_closes_runtime_before_disconnecting_direct_client() -> None:
    check = verifier._run_wheel_check("df_back")

    assert check.name == "wheel_df_back"
    assert check.passed, check.details


def test_runtime_world_bundle_exposes_owned_transport(tmp_path) -> None:
    """性能门禁通过所有权 bundle 读取 transport，不依赖 runtime 私有字段。"""
    with verifier._runtime_world(
        tmp_path,
        capture_lidar_top_view=False,
        with_logger=False,
    ) as bundle:
        snapshot = bundle.transport.snapshot()

        assert snapshot.mode == "local"
        assert snapshot.error_count == snapshot.dropped_count == 0


def test_performance_gate_reuses_shared_deadline_pacer() -> None:
    """联合负载超期时必须让出执行权，避免饿死异步日志线程。"""
    source = inspect.getsource(verifier.run_twenty_obstacle_queue_performance_check)

    assert "DeadlinePacer(" in source


def test_stage3_verifier_uses_shared_backlog_predicate() -> None:
    assert verifier._has_sustained_backlog is _has_sustained_backlog


def _log_snapshot(
    accepted_messages: int,
    *,
    dropped_messages: int = 0,
    dropped_events: int = 0,
    writer_failed: bool = False,
    pending_count: int = 0,
) -> InterfaceLogSnapshot:
    """构造性能窗口日志快照，集中固定无关字段。"""
    return InterfaceLogSnapshot(
        accepted_messages,
        0,
        dropped_messages,
        dropped_events,
        False,
        writer_failed,
        pending_count,
    )


@pytest.mark.parametrize(
    ("end", "samples", "expected_reason"),
    (
        (_log_snapshot(1_079), ((0.0, 0, 0),), "accepted"),
        (_log_snapshot(1_200, pending_count=1), ((0.0, 1, 1_199),), "pending"),
        (_log_snapshot(1_200, dropped_messages=1), ((0.0, 0, 1_200),), "dropped"),
        (_log_snapshot(1_200, writer_failed=True), ((0.0, 0, 1_200),), "writer"),
        (
            _log_snapshot(1_200),
            ((0.0, 1, 10), (0.5, 1, 10), (1.1, 1, 10)),
            "backlog",
        ),
    ),
)
def test_performance_log_quality_rejects_each_hard_gate_failure(
    end: InterfaceLogSnapshot,
    samples: tuple[tuple[float, int, int], ...],
    expected_reason: str,
) -> None:
    quality = verifier._evaluate_performance_log(
        _log_snapshot(0),
        end,
        samples,
        nominal_message_count=1_200,
    )

    assert quality.passed is False
    assert expected_reason in quality.failure_reasons


def test_performance_log_quality_accepts_ninety_percent_and_idle_end() -> None:
    samples = ((0.0, 1, 10), (0.5, 1, 120), (1.1, 1, 240), (1.2, 0, 1_080))

    quality = verifier._evaluate_performance_log(
        _log_snapshot(20),
        _log_snapshot(1_100),
        samples,
        nominal_message_count=1_200,
    )

    assert quality.passed is True
    assert quality.accepted_messages == 1_080
    assert quality.minimum_accepted_messages == 1_080
    assert quality.final_pending == 0


def test_performance_log_quality_rejects_nonidle_warmup_baseline() -> None:
    quality = verifier._evaluate_performance_log(
        _log_snapshot(20, pending_count=1),
        _log_snapshot(1_100),
        ((0.0, 1, 19), (0.1, 0, 1_100)),
        nominal_message_count=1_200,
    )

    assert quality.passed is False
    assert "baseline_pending" in quality.failure_reasons


@pytest.mark.parametrize(
    "runner",
    (
        verifier.run_timeout_and_steering_hold_check,
        verifier.run_100_10_hz_scheduler_check,
        verifier.run_static_and_moving_obstacle_lidar_check,
        verifier.run_pause_rebuild_and_edge_switch_check,
        verifier.run_per_topic_ecal_status_check,
    ),
)
def test_real_stage3_runner_passes_its_own_hard_gate(runner) -> None:
    check = runner()

    assert check.passed, f"{check.name}: {check.details}"
