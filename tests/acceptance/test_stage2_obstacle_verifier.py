# 阶段二障碍物验收测试：固定报告聚合、哈希、运动和性能判据。
from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from slope_sim.obstacles import ObstacleGeometry, ObstaclePath, ObstacleSnapshot
from slope_sim.runtime_actions import TerrainSelection
from slope_sim.scene import create_slope_scene
from scripts import verify_stage2_obstacles as verifier


def _snapshot(logical_id: int, *, body_id: int | None, x: float, y: float, z: float = 0.3):
    return ObstacleSnapshot(
        logical_id=logical_id,
        body_id=body_id,
        mode="static",
        shape="box",
        position=(x, y, z),
        orientation=(0.0, 0.0, 0.0, 1.0),
        geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
    )


def test_report_summary_formats_pass_fail_and_exit_code():
    checks = (
        verifier.VerificationCheck("ground", True, "max_error=0.010"),
        verifier.VerificationCheck("motion", False, "reversals=0"),
    )

    lines, exit_code = verifier.format_report(checks)

    assert lines == [
        "PASS ground max_error=0.010",
        "FAIL motion reversals=0",
        "SUMMARY pass=1 fail=1",
    ]
    assert exit_code == 1


def test_layout_digest_ignores_pybullet_body_ids_and_preserves_layout_semantics():
    first = (
        _snapshot(2, body_id=90, x=1.0, y=0.5),
        _snapshot(1, body_id=91, x=-1.0, y=0.0),
    )
    same_layout = (
        _snapshot(1, body_id=12, x=-1.0, y=0.0),
        _snapshot(2, body_id=13, x=1.0, y=0.5),
    )
    moved = (
        _snapshot(1, body_id=12, x=-1.0, y=0.0),
        _snapshot(2, body_id=13, x=1.2, y=0.5),
    )

    assert verifier.layout_digest(first) == verifier.layout_digest(same_layout)
    assert verifier.layout_digest(first) != verifier.layout_digest(moved)


def test_motion_metrics_count_endpoint_reversals_and_path_error():
    path = ObstaclePath(start_xy=(0.0, 0.0), end_xy=(1.0, 0.0), speed=0.5)
    samples = (
        SimpleNamespace(position=(0.0, 0.0, 0.3), path=path),
        SimpleNamespace(position=(0.5, 0.02, 0.3), path=path),
        SimpleNamespace(position=(1.0, 0.0, 0.3), path=SimpleNamespace(**{**path.__dict__, "direction": -1})),
        SimpleNamespace(position=(0.5, -0.03, 0.3), path=SimpleNamespace(**{**path.__dict__, "direction": -1})),
        SimpleNamespace(position=(0.0, 0.0, 0.3), path=SimpleNamespace(**{**path.__dict__, "direction": 1})),
    )

    metrics = verifier.motion_metrics(samples)

    assert metrics.reversal_count == 2
    assert metrics.max_path_error == 0.03


def test_ground_attachment_metrics_cover_three_terrain_labels():
    metrics = verifier.ground_attachment_metrics(
        {
            "flat": (0.0, 0.010),
            "slope": (0.020, 0.030),
            "golf_heightfield": (0.005, 0.015),
        }
    )

    assert metrics.max_error == 0.030
    assert metrics.by_terrain == {
        "flat": 0.010,
        "slope": 0.030,
        "golf_heightfield": 0.015,
    }
    assert "golf_heightfield=0.0150m" in metrics.details


def test_body_lifecycle_counts_deleted_and_remaining_ids():
    counts = verifier.body_lifecycle_counts({1, 2, 3, 4}, {2, 4, 5})

    assert counts.deleted_count == 2
    assert counts.remaining_count == 2
    assert counts.created_count == 1


def test_performance_summary_uses_fake_clock_durations_without_wall_clock_assertions():
    stats = verifier.performance_summary(
        operation="add_50_clear_100",
        step_durations=(0.001, 0.0025, 0.0015),
        qt_event_durations=(0.010, 0.040, 0.020),
        soft_budget_seconds=0.002,
        hard_event_seconds=0.100,
    )

    assert stats.operation == "add_50_clear_100"
    assert stats.max_step_seconds == 0.0025
    assert stats.max_qt_event_seconds == 0.040
    assert stats.exceeded_soft_budget is True
    assert stats.exceeded_hard_event_limit is False
    assert "max_step_ms=2.50" in stats.details


def test_batch_performance_gate_fails_on_direct_blocking_or_event_delay():
    direct_at_limit = verifier.performance_summary(
        operation="add_50_clear_100",
        step_durations=(0.100,),
        qt_event_durations=(0.0,),
        soft_budget_seconds=0.002,
        hard_event_seconds=0.100,
    )
    soft_budget_only = verifier.performance_summary(
        operation="add_50_clear_100",
        step_durations=(0.020,),
        qt_event_durations=(0.0,),
        soft_budget_seconds=0.002,
        hard_event_seconds=0.100,
    )
    direct_blocked = verifier.performance_summary(
        operation="add_50_clear_100",
        step_durations=(0.101,),
        qt_event_durations=(0.0,),
        soft_budget_seconds=0.002,
        hard_event_seconds=0.100,
    )
    event_blocked = verifier.performance_summary(
        operation="add_50_clear_100",
        step_durations=(0.001,),
        qt_event_durations=(0.120,),
        soft_budget_seconds=0.002,
        hard_event_seconds=0.100,
    )

    assert verifier.batch_performance_passed(direct_at_limit) is True
    assert verifier.batch_performance_passed(soft_budget_only) is True
    assert verifier.batch_performance_passed(direct_blocked) is False
    assert verifier.batch_performance_passed(event_blocked) is False
    assert "soft_budget_ms=2.00" in direct_blocked.details
    assert "hard_event_ms=100.00" in event_blocked.details


def test_performance_summary_marks_empty_qt_samples_as_unmeasured():
    stats = verifier.performance_summary(
        operation="add_50_clear_100",
        step_durations=(0.035,),
        qt_event_durations=(),
        soft_budget_seconds=0.002,
        hard_event_seconds=0.100,
    )

    assert stats.max_qt_event_seconds is None
    assert stats.exceeded_hard_event_limit is None
    assert "max_qt_event_ms=not_measured" in stats.details
    assert verifier.batch_performance_passed(stats) is True


def test_motion_gate_uses_one_micrometre_path_error_limit():
    at_limit = verifier.MotionMetrics(reversal_count=2, max_path_error=0.000001)
    over_limit = verifier.MotionMetrics(reversal_count=2, max_path_error=0.000002)

    assert verifier.motion_gate_passed(at_limit) is True
    assert verifier.motion_gate_passed(over_limit) is False


def test_collision_gate_requires_complete_static_and_moving_physics_limits():
    baseline = verifier.CollisionMetrics(displacement=1.0)
    static = verifier.CollisionMetrics(
        displacement=0.5,
        contact_frames=3,
        max_penetration=0.03,
        max_robot_linear_speed=3.0,
        max_robot_angular_speed=10.0,
    )
    moving = verifier.CollisionMetrics(
        contact_frames=2,
        max_penetration=0.03,
        max_robot_linear_speed=3.0,
        max_robot_angular_speed=10.0,
        max_obstacle_path_error=0.000001,
    )

    assert verifier.collision_gate_passed(baseline, static, moving) is True
    assert verifier.collision_gate_passed(baseline, replace(static, displacement=0.51), moving) is False
    assert verifier.collision_gate_passed(baseline, replace(static, contact_frames=0), moving) is False
    assert verifier.collision_gate_passed(baseline, replace(static, max_penetration=0.031), moving) is False
    assert verifier.collision_gate_passed(baseline, replace(static, max_robot_linear_speed=3.01), moving) is False
    assert verifier.collision_gate_passed(baseline, static, replace(moving, max_robot_angular_speed=10.01)) is False
    assert verifier.collision_gate_passed(baseline, static, replace(moving, states_finite=False)) is False
    assert verifier.collision_gate_passed(baseline, static, replace(moving, max_obstacle_path_error=0.000002)) is False


def test_slope_scene_gate_checks_metadata_and_physical_surface():
    client_id = verifier.p.connect(verifier.p.DIRECT)
    try:
        target = TerrainSelection("slope", slope_deg=6.0)
        scene = create_slope_scene(client_id, slope_deg=6.0, time_step=1.0 / 240.0, terrain_model="slope")

        assert verifier.slope_scene_matches(client_id, scene, target) is True
        assert verifier.slope_scene_matches(client_id, replace(scene, slope_deg=5.0), target) is False
        assert verifier.slope_scene_matches(client_id, replace(scene, terrain_type="flat"), target) is False
    finally:
        verifier.p.disconnect(client_id)


def test_snapshot_layout_state_ignores_height_orientation_and_body_ids():
    path = ObstaclePath(start_xy=(0.0, 0.0), end_xy=(1.0, 0.0), speed=0.5, progress=0.25, direction=-1)
    before = (
        ObstacleSnapshot(
            logical_id=7,
            body_id=10,
            mode="moving",
            shape="box",
            position=(1.0, 0.5, 0.3),
            orientation=(0.0, 0.0, 0.0, 1.0),
            geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
            path=path,
        ),
    )
    after = (
        ObstacleSnapshot(
            logical_id=7,
            body_id=99,
            mode="moving",
            shape="box",
            position=(1.0, 0.5, 1.2),
            orientation=(0.0, 0.1, 0.0, 0.99),
            geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
            path=path,
        ),
    )

    assert verifier.snapshot_layout_state(before) == verifier.snapshot_layout_state(after)


def test_robot_spawn_pose_matches_scene_spawn_and_model_height(monkeypatch):
    monkeypatch.setattr(verifier, "_robot_base_height", lambda _model: 0.42)

    def fake_pose(_robot_id, *, physicsClientId):
        assert physicsClientId == 11
        return (1.01, 1.98, 0.93), (0.0, 0.0, 0.0, 1.0)

    monkeypatch.setattr(verifier.p, "getBasePositionAndOrientation", fake_pose)

    scene = SimpleNamespace(spawn_position=(1.0, 2.0, 0.5))

    assert verifier.robot_spawn_pose_matches(11, 44, scene, "df_mid") is True


def test_lifecycle_check_closes_event_logger_when_recording_raises(monkeypatch):
    class ExplodingLogger:
        instances = []

        def __init__(self, *_args, **_kwargs):
            self.closed = False
            self.path = Path("/tmp/unused-stage2-log.jsonl")
            self.instances.append(self)

        def record_event(self, **_kwargs):
            raise RuntimeError("record failed")

        def close(self):
            self.closed = True
            return self.path

    monkeypatch.setattr(verifier, "ObstacleEventLogger", ExplodingLogger)

    result = verifier._run_lifecycle_and_event_log_check()

    assert result.passed is False
    assert ExplodingLogger.instances
    assert ExplodingLogger.instances[0].closed is True
