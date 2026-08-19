"""MID-360 Golf 固定路线、场景、控制和安全状态机单元测试。"""

from __future__ import annotations

from dataclasses import replace
import importlib
import math
from pathlib import Path

import pytest

from slope_sim.scene import TerrainBounds
from slope_sim.scene_config import load_scene


PROJECT_ROOT = Path(__file__).resolve().parents[2]
CANONICAL_BOUNDS = TerrainBounds(-10.01, 10.01, -6.65, 6.65)


def _drive_module():
    """把新生产模块的导入留在测试体内，使缺失模块表现为明确 RED。"""
    return importlib.import_module("slope_sim.mid360_golf_drive")


def test_canonical_route_freezes_five_lanes_four_semicircles_and_approach() -> None:
    drive = _drive_module()

    route = drive.build_canonical_golf_route(
        CANONICAL_BOUNDS,
        spawn_xy=(-3.5, 0.0),
    )

    assert route.lane_y == (-5.0, -2.5, 0.0, 2.5, 5.0)
    assert route.x_min == pytest.approx(-7.26)
    assert route.x_max == pytest.approx(7.26)
    assert len(route.approach_segments) == 2
    assert len(route.scan_segments) == 5
    assert len(route.turn_segments) == 4
    assert route.approach_segments[0].start_xy == pytest.approx((-3.5, 0.0))
    assert route.approach_segments[-1].end_xy == pytest.approx((-7.26, -5.0))
    assert route.approach_segments[-1].end_heading == pytest.approx(0.0)

    for index, segment in enumerate(route.scan_segments):
        expected_start_x, expected_end_x = (
            (route.x_min, route.x_max)
            if index % 2 == 0
            else (route.x_max, route.x_min)
        )
        assert segment.kind == "straight"
        assert segment.start_xy == pytest.approx((expected_start_x, route.lane_y[index]))
        assert segment.end_xy == pytest.approx((expected_end_x, route.lane_y[index]))
        assert segment.target_speed_m_s == pytest.approx(0.6)

    for index, segment in enumerate(route.turn_segments):
        assert segment.kind == "arc"
        assert segment.start_xy == pytest.approx(route.scan_segments[index].end_xy)
        assert segment.end_xy == pytest.approx(route.scan_segments[index + 1].start_xy)
        assert segment.radius == pytest.approx(1.25)
        assert segment.length == pytest.approx(math.pi * 1.25)
        assert segment.curvature == pytest.approx((1.0 if index % 2 == 0 else -1.0) / 1.25)
        assert segment.target_speed_m_s == pytest.approx(0.3)

    assert route.duration_s < 240.0


def test_canonical_scene_has_six_static_three_moving_and_clear_corridors() -> None:
    drive = _drive_module()
    route = drive.build_canonical_golf_route(CANONICAL_BOUNDS, spawn_xy=(-3.5, 0.0))
    obstacles = drive.canonical_golf_obstacles()

    assert tuple(item.logical_id for item in obstacles) == tuple(range(1, 10))
    assert sum(item.mode == "static" for item in obstacles) == 6
    assert sum(item.mode == "moving" for item in obstacles) == 3
    assert {item.geometry.shape for item in obstacles} == {"box", "cylinder", "sphere"}
    assert all(item.path is not None for item in obstacles[6:])
    assert drive.DF_MID_FOOTPRINT_HALF_EXTENTS_M == pytest.approx((0.361, 0.2805))
    assert drive.route_bounds_violations(
        route,
        CANONICAL_BOUNDS,
        vehicle_half_extents=drive.DF_MID_FOOTPRINT_HALF_EXTENTS_M,
        clearance_m=0.5,
    ) == ()
    assert drive.corridor_violations(
        route,
        obstacles,
        vehicle_half_extents=drive.DF_MID_FOOTPRINT_HALF_EXTENTS_M,
        clearance_m=0.5,
    ) == ()


def test_canonical_yaml_matches_the_validated_scene_contract() -> None:
    drive = _drive_module()

    document = load_scene(PROJECT_ROOT / "configs/mid360_golf_mapping.yaml")

    assert document.robot_model == "df_mid"
    assert document.terrain.terrain_model == "golf_heightfield"
    assert document.terrain.golf_seed == 41
    assert document.terrain.golf_relief == "medium"
    assert document.obstacles == drive.canonical_golf_obstacles()


def test_route_controller_combines_curvature_feedforward_with_error_feedback() -> None:
    drive = _drive_module()
    route = drive.build_canonical_golf_route(CANONICAL_BOUNDS)
    fast_limits = {"max_wheel_acceleration_rad_s2": 10_000.0}

    straight = route.scan_segments[0]
    straight_route = replace(route, segments=(straight,))
    straight_controller = drive.GolfRouteController(straight_route, **fast_limits)
    straight_command = straight_controller.update(
        timestamp_ns=0,
        x=0.0,
        y=straight.start_xy[1],
        yaw=0.0,
    )
    assert straight_command is not None
    assert straight_command.drive_wheel_speeds == pytest.approx((6.0, 6.0))

    arc = route.turn_segments[0]
    arc_x, arc_y, arc_heading = arc.point_at(arc.length / 2.0)
    arc_controller = drive.GolfRouteController(
        replace(route, segments=(arc,)),
        **fast_limits,
    )
    arc_command = arc_controller.update(
        timestamp_ns=0,
        x=arc_x,
        y=arc_y,
        yaw=arc_heading,
    )
    assert arc_command is not None
    assert arc_command.linear_velocity_m_s == pytest.approx(0.3)
    assert arc_command.angular_velocity_rad_s == pytest.approx(0.24)
    assert arc_command.drive_wheel_speeds == pytest.approx((2.4, 3.6))

    feedback_controller = drive.GolfRouteController(straight_route, **fast_limits)
    feedback = feedback_controller.update(
        timestamp_ns=0,
        x=0.0,
        y=straight.start_xy[1] + 0.20,
        yaw=0.10,
    )
    assert feedback is not None
    assert feedback.projection.cross_track_error_m < 0.0
    assert feedback.angular_velocity_rad_s < 0.0
    assert feedback.drive_wheel_speeds[0] > feedback.drive_wheel_speeds[1]


def test_route_controller_reserves_only_approach_tracking_headroom() -> None:
    """驶入段补偿真实轮地跟踪损失，但不得改变冻结路线时间或扫描段速度。"""
    drive = _drive_module()
    route = drive.build_canonical_golf_route(CANONICAL_BOUNDS)
    controller = drive.GolfRouteController(
        route,
        max_wheel_acceleration_rad_s2=10_000.0,
    )

    approach = controller.update(
        timestamp_ns=0,
        x=route.segments[0].start_xy[0],
        y=route.segments[0].start_xy[1],
        yaw=route.segments[0].start_heading,
    )

    assert approach is not None
    assert approach.projection.target_speed_m_s == pytest.approx(0.25)
    assert approach.linear_velocity_m_s == pytest.approx(0.275)
    assert route.duration_s == pytest.approx(
        sum(segment.length / segment.target_speed_m_s for segment in route.segments)
    )
    assert route.scan_segments[0].target_speed_m_s == pytest.approx(0.6)
    assert route.turn_segments[0].target_speed_m_s == pytest.approx(0.3)


def test_route_controller_does_not_jump_to_future_overlapping_lane_near_spawn() -> None:
    drive = _drive_module()
    route = drive.build_canonical_golf_route(CANONICAL_BOUNDS)
    controller = drive.GolfRouteController(
        route,
        max_wheel_acceleration_rad_s2=10_000.0,
    )

    initial = controller.update(timestamp_ns=0, x=-3.5, y=0.0, yaw=0.0)
    moved_forward = controller.update(
        timestamp_ns=10_000_000,
        x=-3.49,
        y=0.0,
        yaw=0.0,
    )

    assert initial is not None
    assert moved_forward is not None
    assert route.project(-3.49, 0.0).segment_index == 6
    assert initial.projection.segment_index == 0
    assert moved_forward.projection.segment_index == 0


def test_route_controller_advances_through_adjacent_segments_in_order() -> None:
    drive = _drive_module()
    route = drive.build_canonical_golf_route(CANONICAL_BOUNDS)
    controller = drive.GolfRouteController(
        route,
        max_wheel_acceleration_rad_s2=10_000.0,
    )

    at_spawn = controller.update(timestamp_ns=0, x=-3.5, y=0.0, yaw=0.0)
    first_transition = controller.update(
        timestamp_ns=10_000_000,
        x=route.segments[0].end_xy[0],
        y=route.segments[0].end_xy[1],
        yaw=route.segments[0].end_heading,
    )
    second_transition = controller.update(
        timestamp_ns=20_000_000,
        x=route.segments[1].end_xy[0],
        y=route.segments[1].end_xy[1],
        yaw=route.segments[1].end_heading,
    )

    assert at_spawn is not None
    assert first_transition is not None
    assert second_transition is not None
    assert [
        at_spawn.projection.segment_index,
        first_transition.projection.segment_index,
        second_transition.projection.segment_index,
    ] == [0, 1, 2]


def test_route_controller_invalid_yaw_does_not_advance_segment_progress() -> None:
    drive = _drive_module()
    route = drive.build_canonical_golf_route(CANONICAL_BOUNDS)
    controller = drive.GolfRouteController(route)
    first_endpoint = route.segments[0].end_xy
    second_endpoint = route.segments[1].end_xy

    initial = controller.update(timestamp_ns=0, x=-3.5, y=0.0, yaw=0.0)
    with pytest.raises(ValueError, match="yaw must be finite"):
        controller.update(
            timestamp_ns=10_000_000,
            x=first_endpoint[0],
            y=first_endpoint[1],
            yaw=float("nan"),
        )
    recovered = controller.update(
        timestamp_ns=10_000_000,
        x=second_endpoint[0],
        y=second_endpoint[1],
        yaw=route.segments[1].end_heading,
    )

    assert initial is not None
    assert recovered is not None
    assert recovered.projection.segment_index == 0


def test_route_controller_uses_simulation_time_100hz_and_limits_wheel_changes() -> None:
    drive = _drive_module()
    route = drive.build_canonical_golf_route(CANONICAL_BOUNDS)
    controller = drive.GolfRouteController(
        replace(route, segments=(route.scan_segments[0],)),
        max_wheel_acceleration_rad_s2=20.0,
    )

    first = controller.update(timestamp_ns=0, x=0.0, y=-5.0, yaw=0.0)
    too_early = controller.update(timestamp_ns=5_000_000, x=0.0, y=-5.0, yaw=0.0)
    second = controller.update(timestamp_ns=10_000_000, x=0.0, y=-5.0, yaw=0.0)

    assert first is not None
    assert too_early is None
    assert second is not None
    assert first.drive_wheel_speeds == pytest.approx((0.2, 0.2))
    assert second.drive_wheel_speeds == pytest.approx((0.4, 0.4))
    assert all(abs(speed) <= 20.0 for speed in second.drive_wheel_speeds)

    with pytest.raises(ValueError, match="monotonic"):
        controller.update(timestamp_ns=9_000_000, x=0.0, y=-5.0, yaw=0.0)


def test_route_controller_keeps_extreme_feedback_within_df_mid_mechanical_limit() -> None:
    drive = _drive_module()
    route = drive.build_canonical_golf_route(CANONICAL_BOUNDS)
    controller = drive.GolfRouteController(
        route,
        max_wheel_acceleration_rad_s2=10_000.0,
        heading_gain=100.0,
        cross_track_gain=100.0,
    )

    command = controller.update(timestamp_ns=0, x=0.0, y=-4.0, yaw=math.pi)

    assert command is not None
    assert all(abs(speed) <= 20.0 for speed in command.drive_wheel_speeds)


@pytest.mark.parametrize(
    ("overrides", "expected_reason"),
    [
        ({"x": 9.20}, "out_of_bounds"),
        ({"obstacle_collision": True}, "obstacle_collision"),
        ({"recorder_fault": "queue overflow"}, "recorder_fault: queue overflow"),
    ],
)
def test_safety_monitor_latches_immediate_faults_and_requires_zero_command(
    overrides: dict[str, object],
    expected_reason: str,
) -> None:
    drive = _drive_module()
    monitor = drive.GolfSafetyMonitor(CANONICAL_BOUNDS)
    values: dict[str, object] = {
        "sim_time_s": 0.0,
        "x": 0.0,
        "y": 0.0,
        "base_speed_m_s": 0.3,
        "drive_wheel_speeds": (3.0, 3.0),
        "route_error_m": 0.0,
        "commanded_forward_speed_m_s": 0.6,
        "obstacle_collision": False,
        "recorder_fault": None,
    }
    values.update(overrides)

    decision = monitor.update(**values)

    assert decision.faulted is True
    assert decision.fault_reason == expected_reason
    assert decision.zero_command_required is True
    assert decision.drive_wheel_speeds == (0.0, 0.0)
    assert decision.settled is False


def test_safety_monitor_requires_sustained_route_error_and_stall() -> None:
    drive = _drive_module()
    deviation = drive.GolfSafetyMonitor(CANONICAL_BOUNDS)

    assert deviation.update(
        sim_time_s=0.0,
        x=0.0,
        y=0.0,
        base_speed_m_s=0.3,
        drive_wheel_speeds=(3.0, 3.0),
        route_error_m=0.76,
        commanded_forward_speed_m_s=0.6,
    ).faulted is False
    assert deviation.update(
        sim_time_s=0.99,
        x=0.0,
        y=0.0,
        base_speed_m_s=0.3,
        drive_wheel_speeds=(3.0, 3.0),
        route_error_m=0.76,
        commanded_forward_speed_m_s=0.6,
    ).faulted is False
    assert deviation.update(
        sim_time_s=1.0,
        x=0.0,
        y=0.0,
        base_speed_m_s=0.3,
        drive_wheel_speeds=(3.0, 3.0),
        route_error_m=0.76,
        commanded_forward_speed_m_s=0.6,
    ).fault_reason == "route_deviation"

    stalled = drive.GolfSafetyMonitor(CANONICAL_BOUNDS)
    assert stalled.update(
        sim_time_s=0.0,
        x=0.0,
        y=0.0,
        base_speed_m_s=0.0,
        drive_wheel_speeds=(0.0, 0.0),
        route_error_m=0.0,
        commanded_forward_speed_m_s=0.6,
    ).faulted is False
    assert stalled.update(
        sim_time_s=1.99,
        x=0.0,
        y=0.0,
        base_speed_m_s=0.0,
        drive_wheel_speeds=(0.0, 0.0),
        route_error_m=0.0,
        commanded_forward_speed_m_s=0.6,
    ).faulted is False
    assert stalled.update(
        sim_time_s=2.0,
        x=0.0,
        y=0.0,
        base_speed_m_s=0.0,
        drive_wheel_speeds=(0.0, 0.0),
        route_error_m=0.0,
        commanded_forward_speed_m_s=0.6,
    ).fault_reason == "stalled"


def test_safety_monitor_keeps_first_fault_and_needs_48_consecutive_quiet_steps() -> None:
    drive = _drive_module()
    monitor = drive.GolfSafetyMonitor(CANONICAL_BOUNDS)
    initial = monitor.update(
        sim_time_s=0.0,
        x=0.0,
        y=0.0,
        base_speed_m_s=0.3,
        drive_wheel_speeds=(3.0, 3.0),
        route_error_m=0.0,
        commanded_forward_speed_m_s=0.6,
        recorder_fault="disk write failed",
    )
    assert initial.fault_reason == "recorder_fault: disk write failed"

    decision = initial
    for step in range(1, 48):
        decision = monitor.update(
            sim_time_s=step / 240.0,
            x=0.0,
            y=0.0,
            base_speed_m_s=0.0,
            drive_wheel_speeds=(0.0, 0.0),
            route_error_m=0.0,
            commanded_forward_speed_m_s=0.0,
            obstacle_collision=True,
        )
        assert decision.settled is False
        assert decision.fault_reason == "recorder_fault: disk write failed"

    settled = monitor.update(
        sim_time_s=48 / 240.0,
        x=0.0,
        y=0.0,
        base_speed_m_s=0.0,
        drive_wheel_speeds=(0.0, 0.0),
        route_error_m=0.0,
        commanded_forward_speed_m_s=0.0,
    )
    assert settled.quiet_steps == 48
    assert settled.settled is True


def test_obstacle_contact_filter_uses_only_committed_obstacle_body_ids(monkeypatch) -> None:
    drive = _drive_module()
    contacts = (
        (0, 10, 20, -1, -1),
        (0, 10, 31, -1, -1),
        (0, 10, 32, -1, -1),
    )
    monkeypatch.setattr(drive.p, "getContactPoints", lambda **_kwargs: contacts)

    assert drive.obstacle_contact_body_ids(7, 10, {31, 99}) == (31,)
