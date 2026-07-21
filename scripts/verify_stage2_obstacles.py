#!/usr/bin/env python3
# 阶段二 DIRECT 验证：检查动态障碍物、事务、事件日志和性能报告，不依赖桌面环境。
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from typing import Iterable, Sequence

import pybullet as p

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import SimulationCoordinator, create_obstacle_manager, load_manual_world
from slope_sim.logger import ObstacleEventLogger
from slope_sim.obstacles import (
    ObstacleGeometry,
    ObstacleGenerationRequest,
    ObstacleGenerationSettings,
    ObstacleManager,
    ObstacleOperationResult,
    ObstacleSnapshot,
    create_box_obstacle,
    update_kinematic_obstacle,
)
from slope_sim.robot import create_robot
from slope_sim.runtime_actions import (
    AddObstaclesAction,
    ResetRobotAction,
    SwitchRobotAction,
    SwitchTerrainAction,
    TerrainSelection,
)
from slope_sim.scene import SceneInfo, TerrainBounds, create_slope_scene, probe_terrain, terrain_model_names
from slope_sim.simulation import _robot_base_height


TIME_STEP = 1.0 / 240.0
GROUND_ATTACHMENT_LIMIT_M = 0.06
MOVING_PATH_ERROR_LIMIT_M = 1e-6
COLLISION_DISPLACEMENT_RATIO = 0.50
MAX_CONTACT_PENETRATION_M = 0.03
MAX_ROBOT_LINEAR_SPEED_MPS = 3.0
MAX_ROBOT_ANGULAR_SPEED_RADPS = 10.0
SOFT_BUDGET_SECONDS = 0.002
HARD_QT_EVENT_SECONDS = 0.100
DIRECT_BLOCKING_SECONDS = 0.100
ROBOT_SPAWN_POSITION_TOLERANCE_M = 0.025


@dataclass(frozen=True)
class VerificationCheck:
    """单项验收结果；脚本统一用它打印 PASS/FAIL。"""

    name: str
    passed: bool
    details: str = ""


@dataclass(frozen=True)
class MotionMetrics:
    """移动障碍物路径质量统计。"""

    reversal_count: int
    max_path_error: float


@dataclass(frozen=True)
class CollisionMetrics:
    """车辆与质量零障碍物碰撞时的阻挡、接触和稳定性统计。"""

    displacement: float = 0.0
    contact_frames: int = 0
    max_penetration: float = 0.0
    max_robot_linear_speed: float = 0.0
    max_robot_angular_speed: float = 0.0
    states_finite: bool = True
    max_obstacle_path_error: float = 0.0


@dataclass(frozen=True)
class BodyLifecycleCounts:
    """删除/清空后的 PyBullet body 集合变化统计。"""

    deleted_count: int
    remaining_count: int
    created_count: int


@dataclass(frozen=True)
class GroundAttachmentMetrics:
    """三类场地贴地误差聚合。"""

    max_error: float
    by_terrain: dict[str, float]
    details: str


@dataclass(frozen=True)
class PerformanceSummary:
    """结构操作期间的帧耗时和事件间隔摘要。"""

    operation: str
    max_step_seconds: float
    max_qt_event_seconds: float | None
    exceeded_soft_budget: bool
    exceeded_hard_event_limit: bool | None
    details: str


def format_report(checks: Sequence[VerificationCheck]) -> tuple[list[str], int]:
    """把检查结果转成稳定文本；任一失败返回非零退出码。"""
    lines: list[str] = []
    pass_count = 0
    fail_count = 0
    for check in checks:
        status = "PASS" if check.passed else "FAIL"
        if check.passed:
            pass_count += 1
        else:
            fail_count += 1
        suffix = f" {check.details}" if check.details else ""
        lines.append(f"{status} {check.name}{suffix}")
    lines.append(f"SUMMARY pass={pass_count} fail={fail_count}")
    return lines, 0 if fail_count == 0 else 1


def layout_digest(snapshots: Sequence[ObstacleSnapshot]) -> str:
    """按逻辑布局生成摘要；忽略 PyBullet body id，便于跨会话比较随机种子复现。"""
    payload = []
    for snapshot in sorted(snapshots, key=lambda item: item.logical_id):
        path = None
        if snapshot.path is not None:
            path = {
                "start_xy": _round_tuple(snapshot.path.start_xy),
                "end_xy": _round_tuple(snapshot.path.end_xy),
                "speed": round(snapshot.path.speed, 6),
                "progress": round(snapshot.path.progress, 6),
                "direction": snapshot.path.direction,
            }
        geometry = None
        if snapshot.geometry is not None:
            geometry = {
                "shape": snapshot.geometry.shape,
                "half_extents": _round_tuple(snapshot.geometry.half_extents),
            }
        payload.append(
            {
                "logical_id": snapshot.logical_id,
                "mode": snapshot.mode,
                "shape": snapshot.shape,
                "position": _round_tuple(snapshot.position),
                "orientation": _round_tuple(snapshot.orientation),
                "path": path,
                "geometry": geometry,
            }
        )
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def snapshot_layout_state(snapshots: Sequence[ObstacleSnapshot]) -> tuple[object, ...]:
    """提取跨场地不变的逻辑布局；忽略 body id、贴地高度和地表姿态。"""
    state: list[object] = []
    for snapshot in sorted(snapshots, key=lambda item: item.logical_id):
        path = snapshot.path
        geometry = snapshot.geometry
        state.append(
            (
                snapshot.logical_id,
                snapshot.mode,
                snapshot.shape,
                _round_tuple(snapshot.position[:2]),
                None
                if path is None
                else (
                    _round_tuple(path.start_xy),
                    _round_tuple(path.end_xy),
                    round(path.speed, 6),
                    round(path.progress, 6),
                    path.direction,
                ),
                None
                if geometry is None
                else (geometry.shape, _round_tuple(geometry.half_extents)),
            )
        )
    return tuple(state)


def robot_spawn_pose_matches(client_id: int, robot_id: int, scene, robot_model: str) -> bool:
    """验证车辆位于场景出生点附近，并容纳 URDF 基座惯性原点偏移。"""
    position, _orientation = p.getBasePositionAndOrientation(robot_id, physicsClientId=client_id)
    expected = (
        scene.spawn_position[0],
        scene.spawn_position[1],
        scene.spawn_position[2] + _robot_base_height(robot_model),
    )
    pose_error = max(abs(float(actual) - target) for actual, target in zip(position, expected))
    return pose_error <= ROBOT_SPAWN_POSITION_TOLERANCE_M


def slope_scene_matches(client_id: int, scene: SceneInfo, target: TerrainSelection) -> bool:
    """验证分段坡元数据、三段 body 以及坡面高度差和法向均应用目标角度。"""
    if (
        target.terrain_model != "slope"
        or scene.terrain_type != "slope"
        or not math.isclose(scene.slope_deg, target.slope_deg, abs_tol=1e-9)
        or len(scene.body_ids) != 3
        or scene.body_id != scene.body_ids[1]
        or scene.bounds is None
    ):
        return False
    left = probe_terrain(
        client_id,
        -1.0,
        0.0,
        bounds=scene.bounds,
        terrain_body_ids=scene.body_ids,
    )
    right = probe_terrain(
        client_id,
        1.0,
        0.0,
        bounds=scene.bounds,
        terrain_body_ids=scene.body_ids,
    )
    angle = math.radians(target.slope_deg)
    return (
        left.terrain_probe_valid
        and right.terrain_probe_valid
        and math.isclose(
            left.local_ground_height - right.local_ground_height,
            2.0 * math.tan(angle),
            abs_tol=1e-4,
        )
        and math.isclose(left.local_terrain_normal_x, math.sin(angle), abs_tol=1e-6)
        and math.isclose(left.local_terrain_normal_y, 0.0, abs_tol=1e-6)
        and math.isclose(left.local_terrain_normal_z, math.cos(angle), abs_tol=1e-6)
    )


def motion_metrics(samples: Sequence[object]) -> MotionMetrics:
    """统计移动障碍物端点反向次数和偏离规划线段的最大距离。"""
    reversal_count = 0
    max_path_error = 0.0
    previous_direction: int | None = None
    for sample in samples:
        path = sample.path
        if path is None:
            continue
        direction = int(path.direction)
        if previous_direction is not None and direction != previous_direction:
            reversal_count += 1
        previous_direction = direction
        position = sample.position
        max_path_error = max(
            max_path_error,
            _point_segment_distance_2d((float(position[0]), float(position[1])), path.start_xy, path.end_xy),
        )
    return MotionMetrics(reversal_count=reversal_count, max_path_error=round(max_path_error, 6))


def motion_gate_passed(metrics: MotionMetrics) -> bool:
    """移动障碍物至少两次反向，且路径横向误差不得超过一微米。"""
    return metrics.reversal_count >= 2 and metrics.max_path_error <= MOVING_PATH_ERROR_LIMIT_M


def collision_gate_passed(
    baseline: CollisionMetrics,
    static: CollisionMetrics,
    moving: CollisionMetrics,
) -> bool:
    """按阶段二物理门禁统一判定静态阻挡和移动障碍物碰撞。"""
    stable_metrics = (static, moving)
    return (
        baseline.displacement > 0.30
        and static.displacement <= baseline.displacement * COLLISION_DISPLACEMENT_RATIO
        and all(metrics.contact_frames > 0 for metrics in stable_metrics)
        and all(metrics.max_penetration <= MAX_CONTACT_PENETRATION_M for metrics in stable_metrics)
        and all(metrics.max_robot_linear_speed <= MAX_ROBOT_LINEAR_SPEED_MPS for metrics in stable_metrics)
        and all(metrics.max_robot_angular_speed <= MAX_ROBOT_ANGULAR_SPEED_RADPS for metrics in stable_metrics)
        and all(metrics.states_finite for metrics in stable_metrics)
        and moving.max_obstacle_path_error <= MOVING_PATH_ERROR_LIMIT_M
    )


def body_lifecycle_counts(before: set[int], after: set[int]) -> BodyLifecycleCounts:
    """比较操作前后的 body 集合，用于删除和清空验收。"""
    return BodyLifecycleCounts(
        deleted_count=len(before - after),
        remaining_count=len(before & after),
        created_count=len(after - before),
    )


def ground_attachment_metrics(errors_by_terrain: dict[str, Sequence[float]]) -> GroundAttachmentMetrics:
    """按场地聚合贴地误差，脚本输出保留每类场地最大值。"""
    by_terrain = {
        terrain: round(max((float(value) for value in errors), default=0.0), 6)
        for terrain, errors in sorted(errors_by_terrain.items())
    }
    max_error = round(max(by_terrain.values(), default=0.0), 6)
    details = " ".join(f"{terrain}={error:.4f}m" for terrain, error in by_terrain.items())
    return GroundAttachmentMetrics(max_error=max_error, by_terrain=by_terrain, details=details)


def performance_summary(
    *,
    operation: str,
    step_durations: Sequence[float],
    qt_event_durations: Sequence[float],
    soft_budget_seconds: float,
    hard_event_seconds: float,
) -> PerformanceSummary:
    """聚合性能采样；真实硬门禁由脚本报告，单测用 fake durations 固定判据。"""
    max_step = max(step_durations, default=0.0)
    max_qt_event = max(qt_event_durations) if qt_event_durations else None
    qt_event_details = "not_measured" if max_qt_event is None else f"{max_qt_event * 1000.0:.2f}"
    return PerformanceSummary(
        operation=operation,
        max_step_seconds=max_step,
        max_qt_event_seconds=max_qt_event,
        exceeded_soft_budget=max_step > soft_budget_seconds,
        exceeded_hard_event_limit=None if max_qt_event is None else max_qt_event > hard_event_seconds,
        details=(
            f"operation={operation} "
            f"max_step_ms={max_step * 1000.0:.2f} "
            f"max_qt_event_ms={qt_event_details} "
            f"soft_budget_ms={soft_budget_seconds * 1000.0:.2f} "
            f"hard_event_ms={hard_event_seconds * 1000.0:.2f}"
        ),
    )


def batch_performance_passed(stats: PerformanceSummary) -> bool:
    """DIRECT 用最大时间片代理硬阻塞；真实 Qt 样本存在时再检查事件间隔。"""
    qt_within_limit = stats.exceeded_hard_event_limit is not True
    return stats.max_step_seconds <= DIRECT_BLOCKING_SECONDS and qt_within_limit


def run_stage2_checks() -> tuple[VerificationCheck, ...]:
    """运行阶段二 DIRECT 验收矩阵并返回结构化报告。"""
    checks: list[VerificationCheck] = []
    checks.extend(_run_ground_attachment_checks())
    checks.append(_run_seed_digest_check())
    checks.append(_run_motion_check())
    checks.append(_run_collision_gate_check())
    checks.extend(_run_coordinator_transaction_checks())
    checks.append(_run_lifecycle_and_event_log_check())
    checks.append(_run_batch_performance_check())
    return tuple(checks)


def _run_ground_attachment_checks() -> tuple[VerificationCheck, ...]:
    checks: list[VerificationCheck] = []
    errors_by_terrain: dict[str, list[float]] = {}
    for terrain_model in terrain_model_names():
        for shape in ("box", "cylinder", "sphere"):
            client_id = p.connect(p.DIRECT)
            try:
                scene = create_slope_scene(
                    client_id,
                    slope_deg=8.0 if terrain_model == "slope" else 0.0,
                    time_step=TIME_STEP,
                    terrain_model=terrain_model,
                    golf_seed=17,
                    golf_relief="medium",
                )
                manager = ObstacleManager(
                    client_id,
                    _settings_for_scene(scene),
                    terrain_body_ids=scene.body_ids,
                )
                manager.begin_add(ObstacleGenerationRequest("static", 1, seed=100 + len(checks), shape=shape))
                result = _finish(manager)
                snapshot = manager.snapshot()[0]
                error = _ground_attachment_error(client_id, snapshot, scene.bounds, scene.body_ids)
                errors_by_terrain.setdefault(terrain_model, []).append(error)
                checks.append(
                    VerificationCheck(
                        f"ground_{terrain_model}_{shape}",
                        result.succeeded and error <= GROUND_ATTACHMENT_LIMIT_M,
                        f"max_error={error:.4f}m",
                    )
                )
            except Exception as exc:
                checks.append(VerificationCheck(f"ground_{terrain_model}_{shape}", False, str(exc)))
            finally:
                p.disconnect(client_id)
    metrics = ground_attachment_metrics(errors_by_terrain)
    checks.append(
        VerificationCheck(
            "ground_attachment_summary",
            metrics.max_error <= GROUND_ATTACHMENT_LIMIT_M,
            metrics.details,
        )
    )
    return tuple(checks)


def _run_seed_digest_check() -> VerificationCheck:
    digests: list[str] = []
    for _ in range(2):
        client_id = p.connect(p.DIRECT)
        try:
            scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
            manager = ObstacleManager(client_id, _settings_for_scene(scene), terrain_body_ids=scene.body_ids)
            manager.begin_add(ObstacleGenerationRequest("mixed", 8, seed=2026, shape="box"))
            result = _finish(manager)
            if not result.succeeded:
                return VerificationCheck("seed_layout_digest", False, result.message)
            digests.append(layout_digest(manager.snapshot(include_body_id=False)))
        finally:
            p.disconnect(client_id)
    return VerificationCheck("seed_layout_digest", digests[0] == digests[1], f"digest={digests[0][:12]}")


def _run_motion_check() -> VerificationCheck:
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        settings = _settings_for_scene(scene)
        settings = ObstacleGenerationSettings(
            bounds=settings.bounds,
            spawn_position=settings.spawn_position,
            spawn_protection_radius=settings.spawn_protection_radius,
            minimum_clearance=settings.minimum_clearance,
            moving_path_length_range=(0.80, 0.80),
            max_candidate_attempts=settings.max_candidate_attempts,
        )
        manager = ObstacleManager(client_id, settings, terrain_body_ids=scene.body_ids)
        manager.begin_add(ObstacleGenerationRequest("moving", 1, seed=3001, shape="box", moving_speed=0.80))
        result = _finish(manager)
        if not result.succeeded:
            return VerificationCheck("moving_ping_pong", False, result.message)
        samples = [manager.snapshot()[0]]
        for _ in range(12):
            manager.update_moving(0.25)
            samples.append(manager.snapshot()[0])
        metrics = motion_metrics(samples)
        passed = motion_gate_passed(metrics)
        return VerificationCheck(
            "moving_ping_pong",
            passed,
            f"reversals={metrics.reversal_count} max_path_error={metrics.max_path_error:.4f}m",
        )
    finally:
        p.disconnect(client_id)


def _run_collision_gate_check() -> VerificationCheck:
    baseline = _static_collision_metrics(with_obstacle=False)
    static = _static_collision_metrics(with_obstacle=True)
    moving = _moving_collision_metrics()
    passed = collision_gate_passed(baseline, static, moving)
    details = (
        f"baseline={baseline.displacement:.3f} blocked={static.displacement:.3f} "
        f"static_contacts={static.contact_frames} moving_contacts={moving.contact_frames} "
        f"max_penetration={max(static.max_penetration, moving.max_penetration):.4f}m "
        f"max_linear={max(static.max_robot_linear_speed, moving.max_robot_linear_speed):.3f}m/s "
        f"max_angular={max(static.max_robot_angular_speed, moving.max_robot_angular_speed):.3f}rad/s "
        f"moving_path_error={moving.max_obstacle_path_error:.6f}m"
    )
    return VerificationCheck("collision_gate", passed, details)


def _run_coordinator_transaction_checks() -> tuple[VerificationCheck, ...]:
    client_id = p.connect(p.DIRECT)
    try:
        config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat", time_step=TIME_STEP)
        world = load_manual_world(client_id, config, TerrainSelection("flat"), "df_back")
        manager = create_obstacle_manager(client_id, world)
        coordinator = SimulationCoordinator(client_id, config, world, manager)
        coordinator.enqueue(AddObstaclesAction(ObstacleGenerationRequest("moving", 1, seed=41)))
        result = _finish_coordinator(coordinator)

        # 车型切换必须替换物理车体，同时完整保留障碍物管理器及其 body。
        before = coordinator.obstacle_manager.snapshot()
        obstacle_body_ids = {item.physics_body_id for item in before}
        original_robot_id = coordinator.world.active_robot.robot.robot_id
        switch_robot = coordinator.apply_action(SwitchRobotAction("df_mid"))
        switched_robot_id = coordinator.world.active_robot.robot.robot_id
        switch_passed = (
            switch_robot.state_changed
            and coordinator.world.active_robot.robot_model == "df_mid"
            and switched_robot_id != original_robot_id
            and original_robot_id not in _body_ids(client_id)
            and {item.physics_body_id for item in coordinator.obstacle_manager.snapshot()} == obstacle_body_ids
            and obstacle_body_ids <= _body_ids(client_id)
        )

        # 先把车辆移离出生点，再验证复位确实新建车体并回到当前场景出生位姿。
        spawn = coordinator.world.scene.spawn_position
        p.resetBasePositionAndOrientation(
            switched_robot_id,
            (spawn[0] + 1.0, spawn[1] + 0.5, spawn[2] + 1.0),
            (0.0, 0.0, 0.0, 1.0),
            physicsClientId=client_id,
        )
        reset_robot = coordinator.apply_action(ResetRobotAction())
        reset_robot_id = coordinator.world.active_robot.robot.robot_id
        reset_position, _reset_orientation = p.getBasePositionAndOrientation(
            reset_robot_id,
            physicsClientId=client_id,
        )
        expected_position = (
            spawn[0],
            spawn[1],
            spawn[2] + _robot_base_height("df_mid"),
        )
        reset_pose_error = max(abs(float(actual) - expected) for actual, expected in zip(reset_position, expected_position))
        reset_passed = (
            reset_robot.state_changed
            and reset_robot_id != switched_robot_id
            and switched_robot_id not in _body_ids(client_id)
            and robot_spawn_pose_matches(client_id, reset_robot_id, coordinator.world.scene, "df_mid")
            and {item.physics_body_id for item in coordinator.obstacle_manager.snapshot()} == obstacle_body_ids
            and obstacle_body_ids <= _body_ids(client_id)
        )

        # 场地重建应保留逻辑布局和路径状态，并为新世界创建有效 body。
        slope_target = TerrainSelection("slope", slope_deg=6.0)
        slope_result = coordinator.apply_action(SwitchTerrainAction(slope_target))
        after = coordinator.obstacle_manager.snapshot()
        after_body_ids = {item.physics_body_id for item in after}
        rebuild_passed = (
            result.obstacle_result is not None
            and result.obstacle_result.succeeded
            and slope_result.state_changed
            and slope_result.world_reset
            and coordinator.world.terrain == slope_target
            and slope_scene_matches(client_id, coordinator.world.scene, slope_target)
            and snapshot_layout_state(before) == snapshot_layout_state(after)
            and None not in after_body_ids
            and after_body_ids <= _body_ids(client_id)
            and _body_ids(client_id)
            == set(coordinator.world.scene.body_ids)
            | {coordinator.world.active_robot.robot.robot_id}
            | after_body_ids
        )

        flat_result = coordinator.apply_action(SwitchTerrainAction(TerrainSelection("flat")))
        if not flat_result.world_reset:
            return (VerificationCheck("coordinator_transactions", False, flat_result.status_message),)
        coordinator.obstacle_manager.restore(
            (
                ObstacleSnapshot(
                    901,
                    None,
                    "static",
                    "box",
                    (8.5, 0.0, 0.25),
                    (0.0, 0.0, 0.0, 1.0),
                    geometry=ObstacleGeometry("box", (0.20, 0.20, 0.25)),
                ),
            )
        )
        rollback_before = coordinator.obstacle_manager.snapshot()
        rollback_digest = layout_digest(rollback_before)
        rollback_result = coordinator.apply_action(SwitchTerrainAction(TerrainSelection("golf_heightfield", golf_seed=3)))
        rollback_after = coordinator.obstacle_manager.snapshot()
        rollback_body_ids = {item.physics_body_id for item in rollback_after}
        rollback_passed = (
            rollback_result.world_reset
            and rollback_result.error_message is not None
            and coordinator.world.terrain == TerrainSelection("flat")
            and layout_digest(rollback_after) == rollback_digest
            and None not in rollback_body_ids
            and rollback_body_ids <= _body_ids(client_id)
            and _body_ids(client_id)
            == set(coordinator.world.scene.body_ids)
            | {coordinator.world.active_robot.robot.robot_id}
            | rollback_body_ids
        )
        return (
            VerificationCheck(
                "coordinator_robot_switch",
                switch_passed,
                f"old_body={original_robot_id} new_body={switched_robot_id}",
            ),
            VerificationCheck(
                "coordinator_robot_reset",
                reset_passed,
                f"old_body={switched_robot_id} new_body={reset_robot_id} pose_error={reset_pose_error:.2e}m",
            ),
            VerificationCheck("coordinator_rebuild", rebuild_passed, f"obstacles={len(after)}"),
            VerificationCheck(
                "coordinator_rollback",
                rollback_passed,
                "restored=flat" if rollback_passed else rollback_result.status_message,
            ),
        )
    except Exception as exc:
        return (VerificationCheck("coordinator_transactions", False, str(exc)),)
    finally:
        p.disconnect(client_id)


def _run_lifecycle_and_event_log_check() -> VerificationCheck:
    client_id = p.connect(p.DIRECT)
    temp_dir = tempfile.TemporaryDirectory()
    logger: ObstacleEventLogger | None = None
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(client_id, _settings_for_scene(scene), terrain_body_ids=scene.body_ids)
        logger = ObstacleEventLogger(temp_dir.name, prefix="stage2")
        manager.begin_add(ObstacleGenerationRequest("static", 2, seed=55, shape="box"))
        add_result = _finish(manager)
        snapshots = manager.snapshot()
        logger.record_event(
            sim_time=0.0,
            event_type="add",
            logical_id=None,
            request_params={"mode": "static", "count": 2, "shape": "box", "seed": 55},
            seed=55,
            robot_model="df_back",
            terrain=TerrainSelection("flat"),
            success=add_result.succeeded,
            error_reason=None if add_result.succeeded else add_result.message,
        )
        before_delete = _body_ids(client_id)
        delete_result = manager.delete(snapshots[0].logical_id)
        logger.record_event(
            sim_time=0.1,
            event_type="delete",
            logical_id=snapshots[0].logical_id,
            request_params={"logical_id": snapshots[0].logical_id},
            seed=None,
            robot_model="df_back",
            terrain=TerrainSelection("flat"),
            success=delete_result.succeeded,
            error_reason=None if delete_result.succeeded else delete_result.message,
        )
        after_delete = _body_ids(client_id)
        manager.begin_clear()
        clear_result = _finish(manager)
        logger.record_event(
            sim_time=0.2,
            event_type="clear",
            logical_id=None,
            request_params={},
            seed=None,
            robot_model="df_back",
            terrain=TerrainSelection("flat"),
            success=clear_result.succeeded,
            error_reason=None if clear_result.succeeded else clear_result.message,
        )
        path = logger.close()
        rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
        lifecycle = body_lifecycle_counts(before_delete, after_delete)
        passed = (
            [row["event_type"] for row in rows] == ["add", "delete", "clear"]
            and lifecycle.deleted_count == 1
            and clear_result.deleted_count == 1
            and manager.snapshot() == ()
        )
        return VerificationCheck(
            "lifecycle_event_log",
            passed,
            f"delete_removed={lifecycle.deleted_count} clear_removed={clear_result.deleted_count} rows={len(rows)}",
        )
    except Exception as exc:
        return VerificationCheck("lifecycle_event_log", False, str(exc))
    finally:
        if logger is not None:
            logger.close()
        temp_dir.cleanup()
        p.disconnect(client_id)


def _run_batch_performance_check() -> VerificationCheck:
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        manager = ObstacleManager(
            client_id,
            _dense_settings_for_scene(scene),
            terrain_body_ids=scene.body_ids,
            soft_budget_seconds=SOFT_BUDGET_SECONDS,
        )
        step_durations: list[float] = []
        scene_body_ids = set(scene.body_ids)
        manager.begin_add(ObstacleGenerationRequest("mixed", 50, seed=77))
        result = ObstacleOperationResult(False, False, "add")
        while not result.done:
            step_start = time.perf_counter()
            result = manager.advance_pending_operation()
            step_durations.append(time.perf_counter() - step_start)
        if not result.succeeded:
            return VerificationCheck("batch_performance", False, result.message)
        first_published = result.published_count
        manager.begin_add(ObstacleGenerationRequest("mixed", 50, seed=78))
        second_result = ObstacleOperationResult(False, False, "add")
        while not second_result.done:
            step_start = time.perf_counter()
            second_result = manager.advance_pending_operation()
            step_durations.append(time.perf_counter() - step_start)
        if not second_result.succeeded:
            return VerificationCheck("batch_performance", False, second_result.message)
        second_published = second_result.published_count
        before_clear_count = len(manager.snapshot())
        manager.begin_clear()
        clear_result = ObstacleOperationResult(False, False, "clear")
        while not clear_result.done:
            step_start = time.perf_counter()
            clear_result = manager.advance_pending_operation()
            step_durations.append(time.perf_counter() - step_start)
        stats = performance_summary(
            operation="add_50_clear_100",
            step_durations=step_durations,
            qt_event_durations=(),
            soft_budget_seconds=SOFT_BUDGET_SECONDS,
            hard_event_seconds=HARD_QT_EVENT_SECONDS,
        )
        passed = (
            first_published == 50
            and second_published == 50
            and before_clear_count == 100
            and clear_result.succeeded
            and clear_result.deleted_count == 100
            and manager.snapshot() == ()
            and _body_ids(client_id) == scene_body_ids
            and batch_performance_passed(stats)
        )
        details = f"{stats.details} direct_limit_ms={DIRECT_BLOCKING_SECONDS * 1000.0:.2f}"
        return VerificationCheck("batch_performance", passed, details)
    except Exception as exc:
        return VerificationCheck("batch_performance", False, str(exc))
    finally:
        p.disconnect(client_id)


def _settings_for_scene(scene) -> ObstacleGenerationSettings:
    return ObstacleGenerationSettings(
        bounds=scene.bounds or TerrainBounds(-8.0, 8.0, -4.0, 4.0),
        spawn_position=scene.spawn_position,
        spawn_protection_radius=0.50,
        max_candidate_attempts=1500,
    )


def _dense_settings_for_scene(scene) -> ObstacleGenerationSettings:
    """批量性能验收使用较小障碍物，确保 100 个对象能在场地内合法排布。"""
    return ObstacleGenerationSettings(
        bounds=scene.bounds or TerrainBounds(-8.0, 8.0, -4.0, 4.0),
        spawn_position=scene.spawn_position,
        spawn_protection_radius=0.20,
        minimum_clearance=0.01,
        half_extent_ranges=((0.08, 0.08), (0.08, 0.08), (0.12, 0.12)),
        max_candidate_attempts=3000,
        max_scene_obstacles=100,
    )


def _finish(manager: ObstacleManager) -> ObstacleOperationResult:
    result = manager.advance_pending_operation()
    guard = 0
    while not result.done:
        guard += 1
        if guard > 300:
            raise RuntimeError("obstacle operation did not finish")
        result = manager.advance_pending_operation()
    return result


def _finish_coordinator(coordinator: SimulationCoordinator):
    result = coordinator.step(TIME_STEP)
    guard = 0
    while result is None or (result.obstacle_result is not None and not result.obstacle_result.done):
        guard += 1
        if guard > 300:
            raise RuntimeError("coordinator operation did not finish")
        result = coordinator.step(TIME_STEP)
    return result


def _ground_attachment_error(client_id: int, snapshot: ObstacleSnapshot, bounds, terrain_body_ids: Sequence[int]) -> float:
    probe = probe_terrain(
        client_id,
        snapshot.position[0],
        snapshot.position[1],
        bounds=bounds,
        terrain_body_ids=terrain_body_ids,
    )
    aabb_min, _aabb_max = p.getAABB(snapshot.physics_body_id, -1, physicsClientId=client_id)
    return abs(float(aabb_min[2]) - probe.local_ground_height)


def _static_collision_metrics(*, with_obstacle: bool) -> CollisionMetrics:
    """用相同车辆初态测量静态障碍物的阻挡和接触稳定性。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        obstacle_id: int | None = None
        if with_obstacle:
            obstacle_id = create_box_obstacle(
                client_id,
                half_extents=(0.25, 0.35, 0.35),
                position=(
                    scene.spawn_position[0] + 0.75,
                    scene.spawn_position[1],
                    scene.spawn_position[2] + 0.35,
                ),
            )
        robot = _create_collision_robot(client_id, scene, scene.spawn_position[0], scene.spawn_position[1])
        start, _ = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
        contact_frames = 0
        max_penetration = 0.0
        max_linear_speed = 0.0
        max_angular_speed = 0.0
        states_finite = True
        for _ in range(240):
            robot.command_twist(0.6, 0.0, dt=TIME_STEP)
            p.stepSimulation(physicsClientId=client_id)
            if obstacle_id is None:
                continue
            contacts = p.getContactPoints(bodyA=robot.robot_id, bodyB=obstacle_id, physicsClientId=client_id)
            if not contacts:
                continue
            contact_frames += 1
            max_penetration = max(max_penetration, max(max(0.0, -float(contact[8])) for contact in contacts))
            finite, linear_speed, angular_speed = _robot_collision_state(client_id, robot.robot_id)
            states_finite = states_finite and finite
            max_linear_speed = max(max_linear_speed, linear_speed)
            max_angular_speed = max(max_angular_speed, angular_speed)
        end, _ = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
        return CollisionMetrics(
            displacement=float(end[0]) - float(start[0]),
            contact_frames=contact_frames,
            max_penetration=max_penetration,
            max_robot_linear_speed=max_linear_speed,
            max_robot_angular_speed=max_angular_speed,
            states_finite=states_finite,
        )
    finally:
        p.disconnect(client_id)


def _moving_collision_metrics() -> CollisionMetrics:
    """让质量零箱体横穿车辆，测量接触稳定性和运动学路径保持。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = create_slope_scene(client_id, slope_deg=0.0, time_step=TIME_STEP, terrain_model="flat")
        robot = _create_collision_robot(client_id, scene, 0.0, 0.0)
        obstacle_id = create_box_obstacle(
            client_id,
            half_extents=(0.25, 0.25, 0.30),
            position=(0.0, -1.0, 0.30),
        )
        obstacle_speed = 0.20
        commanded_y = -1.0
        contact_frames = 0
        max_penetration = 0.0
        max_linear_speed = 0.0
        max_angular_speed = 0.0
        max_path_error = 0.0
        states_finite = True

        for _ in range(960):
            update_kinematic_obstacle(
                client_id,
                obstacle_id,
                position=(0.0, commanded_y, 0.30),
                linear_velocity=(0.0, obstacle_speed, 0.0),
            )
            p.stepSimulation(physicsClientId=client_id)
            commanded_y += obstacle_speed * TIME_STEP
            obstacle_position, _orientation = p.getBasePositionAndOrientation(obstacle_id, physicsClientId=client_id)
            max_path_error = max(max_path_error, abs(float(obstacle_position[0])))
            contacts = p.getContactPoints(bodyA=robot.robot_id, bodyB=obstacle_id, physicsClientId=client_id)
            if not contacts:
                continue
            contact_frames += 1
            max_penetration = max(max_penetration, max(max(0.0, -float(contact[8])) for contact in contacts))
            finite, linear_speed, angular_speed = _robot_collision_state(client_id, robot.robot_id)
            states_finite = states_finite and finite
            max_linear_speed = max(max_linear_speed, linear_speed)
            max_angular_speed = max(max_angular_speed, angular_speed)

        return CollisionMetrics(
            contact_frames=contact_frames,
            max_penetration=max_penetration,
            max_robot_linear_speed=max_linear_speed,
            max_robot_angular_speed=max_angular_speed,
            states_finite=states_finite,
            max_obstacle_path_error=max_path_error,
        )
    finally:
        p.disconnect(client_id)


def _create_collision_robot(client_id: int, scene, start_x: float, start_y: float):
    """按 Task 1 正式摩擦参数创建并稳定碰撞门禁车辆。"""
    robot = create_robot(
        client_id,
        "df_back",
        start_x=start_x,
        start_y=start_y,
        base_height=scene.spawn_position[2] + _robot_base_height("df_back"),
        start_orientation=scene.spawn_orientation,
    )
    robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
    for _ in range(120):
        robot.command_twist(0.0, 0.0, dt=TIME_STEP)
        p.stepSimulation(physicsClientId=client_id)
    return robot


def _robot_collision_state(client_id: int, robot_id: int) -> tuple[bool, float, float]:
    """返回碰撞帧是否全有限，以及车辆线速度和角速度模长。"""
    position, orientation = p.getBasePositionAndOrientation(robot_id, physicsClientId=client_id)
    linear_velocity, angular_velocity = p.getBaseVelocity(robot_id, physicsClientId=client_id)
    values = tuple(float(value) for value in (*position, *orientation, *linear_velocity, *angular_velocity))
    finite = all(math.isfinite(value) for value in values)
    linear_speed = math.sqrt(sum(float(value) ** 2 for value in linear_velocity)) if finite else math.inf
    angular_speed = math.sqrt(sum(float(value) ** 2 for value in angular_velocity)) if finite else math.inf
    return finite, linear_speed, angular_speed


def _body_ids(client_id: int) -> set[int]:
    return {
        p.getBodyUniqueId(index, physicsClientId=client_id)
        for index in range(p.getNumBodies(physicsClientId=client_id))
    }


def _round_tuple(values: Iterable[float]) -> tuple[float, ...]:
    return tuple(round(float(value), 6) for value in values)


def _point_segment_distance_2d(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def main() -> int:
    """运行阶段二验收并打印逐项 PASS/FAIL。"""
    checks = run_stage2_checks()
    lines, exit_code = format_report(checks)
    for line in lines:
        print(line)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
