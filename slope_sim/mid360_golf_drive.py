# MID-360 Golf 离线行驶：固定路线、canonical 障碍、跟踪控制与安全停车。
from __future__ import annotations

from collections.abc import Callable, Collection
from dataclasses import dataclass
import math
from numbers import Integral, Real

import pybullet as p

from slope_sim.controller import wheel_speeds_from_twist
from slope_sim.model_registry import get_robot_model
from slope_sim.obstacles import ObstacleGeometry, ObstaclePath, ObstacleSpec
from slope_sim.scene import TerrainBounds


LANE_Y_M = (-5.0, -2.5, 0.0, 2.5, 5.0)
ROUTE_BOUND_INSET_M = 2.75
U_TURN_RADIUS_M = 1.25
STRAIGHT_SPEED_M_S = 0.6
TURN_SPEED_M_S = 0.3
APPROACH_SPEED_M_S = 0.25
APPROACH_CONTROL_SPEED_M_S = 0.275
DF_MID_FOOTPRINT_HALF_EXTENTS_M = (0.361, 0.2805)
CONTROL_PERIOD_NS = 10_000_000
PHYSICS_STEP_S = 1.0 / 240.0
SAFE_STOP_QUIET_STEPS = 48

_DF_MID_SPEC = get_robot_model("df_mid")


def _finite(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be finite")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _xy(name: str, value: tuple[float, float]) -> tuple[float, float]:
    if len(value) != 2:
        raise ValueError(f"{name} must contain two values")
    return _finite(f"{name}[0]", value[0]), _finite(f"{name}[1]", value[1])


@dataclass(frozen=True)
class RouteSegment:
    """固定路线中的直线或常曲率圆弧。"""

    kind: str
    start_xy: tuple[float, float]
    end_xy: tuple[float, float]
    start_heading: float
    end_heading: float
    length: float
    target_speed_m_s: float
    curvature: float = 0.0
    radius: float | None = None

    def point_at(self, distance_m: float) -> tuple[float, float, float]:
        """按弧长返回中心线位置和切向航向。"""
        distance = min(self.length, max(0.0, _finite("distance_m", distance_m)))
        if abs(self.curvature) < 1e-12:
            return (
                self.start_xy[0] + distance * math.cos(self.start_heading),
                self.start_xy[1] + distance * math.sin(self.start_heading),
                self.start_heading,
            )
        heading = self.start_heading + self.curvature * distance
        return (
            self.start_xy[0]
            + (math.sin(heading) - math.sin(self.start_heading)) / self.curvature,
            self.start_xy[1]
            - (math.cos(heading) - math.cos(self.start_heading)) / self.curvature,
            heading,
        )


def _line_segment(
    kind: str,
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
    speed_m_s: float,
) -> RouteSegment:
    start = _xy("start_xy", start_xy)
    end = _xy("end_xy", end_xy)
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length <= 0.0:
        raise ValueError("route line endpoints must be distinct")
    heading = math.atan2(dy, dx)
    return RouteSegment(
        kind=kind,
        start_xy=start,
        end_xy=end,
        start_heading=heading,
        end_heading=heading,
        length=length,
        target_speed_m_s=speed_m_s,
    )


def _semicircle_segment(
    start_xy: tuple[float, float],
    start_heading: float,
    curvature: float,
) -> RouteSegment:
    length = math.pi / abs(curvature)
    provisional = RouteSegment(
        kind="arc",
        start_xy=start_xy,
        end_xy=start_xy,
        start_heading=start_heading,
        end_heading=start_heading + curvature * length,
        length=length,
        target_speed_m_s=TURN_SPEED_M_S,
        curvature=curvature,
        radius=1.0 / abs(curvature),
    )
    end_x, end_y, end_heading = provisional.point_at(length)
    return RouteSegment(
        kind=provisional.kind,
        start_xy=provisional.start_xy,
        end_xy=(end_x, end_y),
        start_heading=provisional.start_heading,
        end_heading=end_heading,
        length=provisional.length,
        target_speed_m_s=provisional.target_speed_m_s,
        curvature=provisional.curvature,
        radius=provisional.radius,
    )


@dataclass(frozen=True)
class GolfRoute:
    """包含固定驶入段和五带往复扫描段的完整中心线。"""

    bounds: TerrainBounds
    lane_y: tuple[float, ...]
    x_min: float
    x_max: float
    approach_segments: tuple[RouteSegment, ...]
    scan_segments: tuple[RouteSegment, ...]
    turn_segments: tuple[RouteSegment, ...]
    segments: tuple[RouteSegment, ...]

    @property
    def length(self) -> float:
        return sum(segment.length for segment in self.segments)

    @property
    def duration_s(self) -> float:
        return sum(segment.length / segment.target_speed_m_s for segment in self.segments)

    def iter_centerline(self, spacing_m: float = 0.02):
        """以有界间距采样路线，供启动前走廊和边界门禁使用。"""
        spacing = _finite("spacing_m", spacing_m)
        if spacing <= 0.0:
            raise ValueError("spacing_m must be positive")
        for segment_index, segment in enumerate(self.segments):
            count = max(1, math.ceil(segment.length / spacing))
            first_index = 0 if segment_index == 0 else 1
            for sample_index in range(first_index, count + 1):
                distance = segment.length * sample_index / count
                x, y, _heading = segment.point_at(distance)
                yield x, y

    def project(self, x: float, y: float) -> "RouteProjection":
        """把车辆中心投影到最近路线段，并返回带符号横向误差。"""
        position = (_finite("x", x), _finite("y", y))
        route_offset = 0.0
        candidates: list[RouteProjection] = []
        for segment_index, segment in enumerate(self.segments):
            along, reference_x, reference_y, heading = _project_segment(
                segment,
                position,
            )
            error_x = position[0] - reference_x
            error_y = position[1] - reference_y
            distance = math.hypot(error_x, error_y)
            # 右法向与误差的点积为正时，车辆位于路径右侧，应向左修正。
            cross_track = math.sin(heading) * error_x - math.cos(heading) * error_y
            candidates.append(
                RouteProjection(
                    segment_index=segment_index,
                    route_distance_m=route_offset + along,
                    segment_distance_m=along,
                    reference_xy=(reference_x, reference_y),
                    reference_heading=heading,
                    curvature=segment.curvature,
                    target_speed_m_s=segment.target_speed_m_s,
                    distance_m=distance,
                    cross_track_error_m=cross_track,
                )
            )
            route_offset += segment.length
        return min(candidates, key=lambda item: (item.distance_m, item.segment_index))


@dataclass(frozen=True)
class RouteProjection:
    """车辆相对最近中心线参考点的纯几何结果。"""

    segment_index: int
    route_distance_m: float
    segment_distance_m: float
    reference_xy: tuple[float, float]
    reference_heading: float
    curvature: float
    target_speed_m_s: float
    distance_m: float
    cross_track_error_m: float


def _project_segment(
    segment: RouteSegment,
    position: tuple[float, float],
) -> tuple[float, float, float, float]:
    """解析投影一个直线/半圆段，越过端点时钳制到最近端点。"""
    if abs(segment.curvature) < 1e-12:
        dx = position[0] - segment.start_xy[0]
        dy = position[1] - segment.start_xy[1]
        along = min(
            segment.length,
            max(0.0, dx * math.cos(segment.start_heading) + dy * math.sin(segment.start_heading)),
        )
        x, y, heading = segment.point_at(along)
        return along, x, y, heading

    curvature = segment.curvature
    center_x = segment.start_xy[0] - math.sin(segment.start_heading) / curvature
    center_y = segment.start_xy[1] + math.cos(segment.start_heading) / curvature
    start_angle = math.atan2(
        segment.start_xy[1] - center_y,
        segment.start_xy[0] - center_x,
    )
    point_angle = math.atan2(position[1] - center_y, position[0] - center_x)
    if curvature > 0.0:
        angle_delta = (point_angle - start_angle) % (2.0 * math.pi)
    else:
        angle_delta = (start_angle - point_angle) % (2.0 * math.pi)
    sweep = abs(curvature) * segment.length
    if angle_delta <= sweep:
        along = angle_delta / abs(curvature)
    else:
        start_distance = math.dist(position, segment.start_xy)
        end_distance = math.dist(position, segment.end_xy)
        along = 0.0 if start_distance <= end_distance else segment.length
    x, y, heading = segment.point_at(along)
    return along, x, y, heading


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class GolfDriveCommand:
    """一次 100 Hz 路线控制输出及其可诊断参考量。"""

    timestamp_ns: int
    drive_wheel_speeds: tuple[float, float]
    linear_velocity_m_s: float
    angular_velocity_rad_s: float
    projection: RouteProjection
    heading_error_rad: float


class GolfRouteController:
    """以仿真时间运行的固定路线前馈加反馈差速控制器。"""

    def __init__(
        self,
        route: GolfRoute,
        *,
        heading_gain: float = 1.8,
        cross_track_gain: float = 1.2,
        velocity_floor_m_s: float = 0.15,
        max_angular_velocity_rad_s: float = 1.2,
        max_wheel_acceleration_rad_s2: float = 20.0,
    ) -> None:
        if not isinstance(route, GolfRoute):
            raise ValueError("route must be GolfRoute")
        self.route = route
        self.heading_gain = _finite("heading_gain", heading_gain)
        self.cross_track_gain = _finite("cross_track_gain", cross_track_gain)
        self.velocity_floor_m_s = _finite("velocity_floor_m_s", velocity_floor_m_s)
        self.max_angular_velocity_rad_s = _finite(
            "max_angular_velocity_rad_s",
            max_angular_velocity_rad_s,
        )
        self.max_wheel_acceleration_rad_s2 = _finite(
            "max_wheel_acceleration_rad_s2",
            max_wheel_acceleration_rad_s2,
        )
        if (
            self.heading_gain < 0.0
            or self.cross_track_gain < 0.0
            or self.velocity_floor_m_s <= 0.0
            or self.max_angular_velocity_rad_s <= 0.0
            or self.max_wheel_acceleration_rad_s2 <= 0.0
        ):
            raise ValueError("controller gains and limits must be positive")
        self._last_seen_timestamp_ns: int | None = None
        self._last_command_timestamp_ns: int | None = None
        self._wheel_speeds = (0.0, 0.0)
        self._segment_index = 0

    def _project_control_segment(self, x: float, y: float) -> RouteProjection:
        """只投影当前控制段，避免重叠路线把车辆吸附到未来扫描带。"""
        position = (_finite("x", x), _finite("y", y))
        segment = self.route.segments[self._segment_index]
        along, reference_x, reference_y, heading = _project_segment(segment, position)
        error_x = position[0] - reference_x
        error_y = position[1] - reference_y
        return RouteProjection(
            segment_index=self._segment_index,
            route_distance_m=(
                sum(item.length for item in self.route.segments[: self._segment_index])
                + along
            ),
            segment_distance_m=along,
            reference_xy=(reference_x, reference_y),
            reference_heading=heading,
            curvature=segment.curvature,
            target_speed_m_s=segment.target_speed_m_s,
            distance_m=math.hypot(error_x, error_y),
            cross_track_error_m=(
                math.sin(heading) * error_x - math.cos(heading) * error_y
            ),
        )

    def update(
        self,
        *,
        timestamp_ns: int,
        x: float,
        y: float,
        yaw: float,
    ) -> GolfDriveCommand | None:
        """每 10 ms 仿真时间至多生成一次轮速命令；墙钟速度不参与控制。"""
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, Integral):
            raise ValueError("timestamp_ns must be an integer")
        timestamp = int(timestamp_ns)
        if timestamp < 0:
            raise ValueError("timestamp_ns must be non-negative")
        if self._last_seen_timestamp_ns is not None and timestamp < self._last_seen_timestamp_ns:
            raise ValueError("simulation timestamp must be monotonic")
        self._last_seen_timestamp_ns = timestamp
        if (
            self._last_command_timestamp_ns is not None
            and timestamp - self._last_command_timestamp_ns < CONTROL_PERIOD_NS
        ):
            return None

        normalized_yaw = _finite("yaw", yaw)
        projection = self._project_control_segment(x, y)
        active_segment = self.route.segments[self._segment_index]
        if (
            self._segment_index + 1 < len(self.route.segments)
            and active_segment.length - projection.segment_distance_m <= 1e-9
        ):
            self._segment_index += 1
            projection = self._project_control_segment(x, y)
        heading_error = _wrap_angle(projection.reference_heading - normalized_yaw)
        # 驶入段保留航向对准与轮地跟踪余量，路线时钟仍按冻结的 0.25 m/s 推进。
        linear_velocity = (
            APPROACH_CONTROL_SPEED_M_S
            if self.route.segments[projection.segment_index].kind == "approach"
            else projection.target_speed_m_s
        )
        angular_velocity = (
            linear_velocity * projection.curvature
            + self.heading_gain * heading_error
            + math.atan2(
                self.cross_track_gain * projection.cross_track_error_m,
                max(linear_velocity, self.velocity_floor_m_s),
            )
        )
        angular_velocity = max(
            -self.max_angular_velocity_rad_s,
            min(self.max_angular_velocity_rad_s, angular_velocity),
        )
        desired = wheel_speeds_from_twist(
            linear_velocity,
            angular_velocity,
            _DF_MID_SPEC.wheel_track,
            _DF_MID_SPEC.wheel_radius,
        )
        largest = max(abs(speed) for speed in desired)
        if largest > _DF_MID_SPEC.max_drive_wheel_speed_rad_s:
            scale = _DF_MID_SPEC.max_drive_wheel_speed_rad_s / largest
            desired = desired[0] * scale, desired[1] * scale

        elapsed_s = (
            CONTROL_PERIOD_NS / 1_000_000_000.0
            if self._last_command_timestamp_ns is None
            else (timestamp - self._last_command_timestamp_ns) / 1_000_000_000.0
        )
        maximum_change = self.max_wheel_acceleration_rad_s2 * elapsed_s
        limited = tuple(
            previous + max(-maximum_change, min(maximum_change, target - previous))
            for previous, target in zip(self._wheel_speeds, desired, strict=True)
        )
        self._wheel_speeds = limited  # type: ignore[assignment]
        self._last_command_timestamp_ns = timestamp
        commanded_linear = (limited[0] + limited[1]) * _DF_MID_SPEC.wheel_radius / 2.0
        commanded_angular = (
            (limited[1] - limited[0])
            * _DF_MID_SPEC.wheel_radius
            / _DF_MID_SPEC.wheel_track
        )
        return GolfDriveCommand(
            timestamp_ns=timestamp,
            drive_wheel_speeds=(limited[0], limited[1]),
            linear_velocity_m_s=commanded_linear,
            angular_velocity_rad_s=commanded_angular,
            projection=projection,
            heading_error_rad=heading_error,
        )


@dataclass(frozen=True)
class GolfSafetyDecision:
    """安全监视器的锁存结果；故障后命令端只能持续发送零轮速。"""

    faulted: bool
    fault_reason: str | None
    zero_command_required: bool
    drive_wheel_speeds: tuple[float, float] | None
    quiet_steps: int
    settled: bool


class GolfSafetyMonitor:
    """按仿真时间检测故障，并以 48 个连续物理步确认车辆静止。"""

    def __init__(
        self,
        bounds: TerrainBounds,
        *,
        vehicle_half_extents: tuple[float, float] = DF_MID_FOOTPRINT_HALF_EXTENTS_M,
        boundary_clearance_m: float = 0.5,
        physics_step_s: float = PHYSICS_STEP_S,
    ) -> None:
        if not isinstance(bounds, TerrainBounds):
            raise ValueError("bounds must be TerrainBounds")
        half_x, half_y = _xy("vehicle_half_extents", vehicle_half_extents)
        clearance = _finite("boundary_clearance_m", boundary_clearance_m)
        step = _finite("physics_step_s", physics_step_s)
        if half_x <= 0.0 or half_y <= 0.0 or clearance < 0.0 or step <= 0.0:
            raise ValueError("safety geometry and physics step must be positive")
        self.bounds = bounds
        self._boundary_radius_m = math.hypot(half_x, half_y) + clearance
        self._required_quiet_steps = math.ceil(0.2 / step - 1e-12)
        self._last_time_s: float | None = None
        self._deviation_since_s: float | None = None
        self._stall_since_s: float | None = None
        self._fault_reason: str | None = None
        self._quiet_steps = 0
        self._settled = False

    def _latch(self, reason: str) -> None:
        if self._fault_reason is None:
            self._fault_reason = reason

    def update(
        self,
        *,
        sim_time_s: float,
        x: float,
        y: float,
        base_speed_m_s: float,
        drive_wheel_speeds: tuple[float, ...],
        route_error_m: float,
        commanded_forward_speed_m_s: float,
        obstacle_collision: bool = False,
        recorder_fault: str | bool | None = None,
    ) -> GolfSafetyDecision:
        """处理一个物理步观测；首个故障原因和零命令要求保持锁存。"""
        now = _finite("sim_time_s", sim_time_s)
        position_x = _finite("x", x)
        position_y = _finite("y", y)
        base_speed = _finite("base_speed_m_s", base_speed_m_s)
        route_error = _finite("route_error_m", route_error_m)
        forward_command = _finite(
            "commanded_forward_speed_m_s",
            commanded_forward_speed_m_s,
        )
        if self._last_time_s is not None and now < self._last_time_s:
            raise ValueError("simulation time must be monotonic")
        if base_speed < 0.0 or route_error < 0.0:
            raise ValueError("reported speeds and route error must be non-negative")
        wheel_speeds = tuple(
            _finite(f"drive_wheel_speeds[{index}]", speed)
            for index, speed in enumerate(drive_wheel_speeds)
        )
        if not wheel_speeds:
            raise ValueError("drive_wheel_speeds must be non-empty")
        self._last_time_s = now

        inside = (
            self.bounds.min_x + self._boundary_radius_m
            <= position_x
            <= self.bounds.max_x - self._boundary_radius_m
            and self.bounds.min_y + self._boundary_radius_m
            <= position_y
            <= self.bounds.max_y - self._boundary_radius_m
        )
        if not inside:
            self._latch("out_of_bounds")
        if obstacle_collision:
            self._latch("obstacle_collision")
        if recorder_fault:
            detail = recorder_fault if isinstance(recorder_fault, str) else ""
            self._latch("recorder_fault" + (f": {detail}" if detail else ""))

        if route_error > 0.75:
            if self._deviation_since_s is None:
                self._deviation_since_s = now
            elif now - self._deviation_since_s >= 1.0:
                self._latch("route_deviation")
        else:
            self._deviation_since_s = None

        if forward_command > 0.1 and base_speed < 0.05:
            if self._stall_since_s is None:
                self._stall_since_s = now
            elif now - self._stall_since_s >= 2.0:
                self._latch("stalled")
        else:
            self._stall_since_s = None

        if self._fault_reason is not None and not self._settled:
            quiet = base_speed < 0.02 and all(abs(speed) < 0.1 for speed in wheel_speeds)
            self._quiet_steps = self._quiet_steps + 1 if quiet else 0
            self._settled = self._quiet_steps >= self._required_quiet_steps

        faulted = self._fault_reason is not None
        return GolfSafetyDecision(
            faulted=faulted,
            fault_reason=self._fault_reason,
            zero_command_required=faulted,
            drive_wheel_speeds=(0.0, 0.0) if faulted else None,
            quiet_steps=self._quiet_steps,
            settled=self._settled,
        )


def obstacle_contact_body_ids(
    client_id: int,
    robot_id: int,
    obstacle_body_ids: Collection[int],
) -> tuple[int, ...]:
    """查询 robot 作为 bodyA 的接触，并只保留已提交障碍物 body ID。"""
    if isinstance(client_id, bool) or not isinstance(client_id, Integral) or int(client_id) < 0:
        raise ValueError("client_id must be a non-negative integer")
    if isinstance(robot_id, bool) or not isinstance(robot_id, Integral) or int(robot_id) < 0:
        raise ValueError("robot_id must be a non-negative integer")
    committed = frozenset(int(body_id) for body_id in obstacle_body_ids)
    contacts = p.getContactPoints(
        bodyA=int(robot_id),
        physicsClientId=int(client_id),
    )
    return tuple(
        dict.fromkeys(
            int(contact[2])
            for contact in contacts
            if int(contact[2]) in committed
        )
    )


def advance_golf_physics_step(
    *,
    client_id: int,
    obstacle_manager: object,
    dt: float,
    apply_command: Callable[[], object] | None = None,
) -> None:
    """按离线合同依次下发命令、更新移动障碍，再推进一个 Bullet 物理步。"""
    if isinstance(client_id, bool) or not isinstance(client_id, Integral) or int(client_id) < 0:
        raise ValueError("client_id must be a non-negative integer")
    step = _finite("dt", dt)
    if step <= 0.0:
        raise ValueError("dt must be positive")
    if apply_command is not None:
        apply_command()
    update_moving = getattr(obstacle_manager, "update_moving", None)
    if not callable(update_moving):
        raise ValueError("obstacle_manager must provide update_moving(dt)")
    update_moving(step)
    p.stepSimulation(physicsClientId=int(client_id))


def build_canonical_golf_route(
    bounds: TerrainBounds,
    *,
    spawn_xy: tuple[float, float] = (-3.5, 0.0),
) -> GolfRoute:
    """按 Golf bounds 构造唯一的五带往复路线和固定两段式驶入路径。"""
    if not isinstance(bounds, TerrainBounds):
        raise ValueError("bounds must be TerrainBounds")
    x_min = _finite("bounds.min_x", bounds.min_x) + ROUTE_BOUND_INSET_M
    x_max = _finite("bounds.max_x", bounds.max_x) - ROUTE_BOUND_INSET_M
    if x_min >= x_max:
        raise ValueError("Golf bounds are too narrow for the canonical route")
    if bounds.min_y > LANE_Y_M[0] or bounds.max_y < LANE_Y_M[-1]:
        raise ValueError("Golf bounds do not contain all canonical lanes")

    entry = (x_min, LANE_Y_M[0])
    alignment = (x_min - U_TURN_RADIUS_M, LANE_Y_M[0])
    approach_segments = (
        _line_segment("approach", _xy("spawn_xy", spawn_xy), alignment, APPROACH_SPEED_M_S),
        _line_segment("approach", alignment, entry, APPROACH_SPEED_M_S),
    )

    scan_segments: list[RouteSegment] = []
    turn_segments: list[RouteSegment] = []
    ordered: list[RouteSegment] = [*approach_segments]
    for index, lane_y in enumerate(LANE_Y_M):
        start_x, end_x = (x_min, x_max) if index % 2 == 0 else (x_max, x_min)
        straight = _line_segment(
            "straight",
            (start_x, lane_y),
            (end_x, lane_y),
            STRAIGHT_SPEED_M_S,
        )
        scan_segments.append(straight)
        ordered.append(straight)
        if index == len(LANE_Y_M) - 1:
            continue
        curvature = (1.0 if index % 2 == 0 else -1.0) / U_TURN_RADIUS_M
        turn = _semicircle_segment(straight.end_xy, straight.end_heading, curvature)
        turn_segments.append(turn)
        ordered.append(turn)

    return GolfRoute(
        bounds=bounds,
        lane_y=LANE_Y_M,
        x_min=x_min,
        x_max=x_max,
        approach_segments=approach_segments,
        scan_segments=tuple(scan_segments),
        turn_segments=tuple(turn_segments),
        segments=tuple(ordered),
    )


def canonical_golf_obstacles() -> tuple[ObstacleSpec, ...]:
    """返回固定逻辑 ID、尺寸、位姿和往返路径的 6+3 canonical 场景。"""
    identity = (0.0, 0.0, 0.0, 1.0)
    static_values = (
        (1, "box", (0.18, 0.14, 0.40), (0.0, -3.75, 0.0)),
        (2, "cylinder", (0.16, 0.16, 0.45), (3.0, -1.25, 0.0)),
        (3, "sphere", (0.16, 0.16, 0.16), (-1.0, 1.25, 0.0)),
        (4, "box", (0.20, 0.14, 0.35), (4.0, 3.75, 0.0)),
        (5, "cylinder", (0.16, 0.16, 0.40), (0.0, -6.20, 0.0)),
        (6, "sphere", (0.16, 0.16, 0.16), (0.0, 6.20, 0.0)),
    )
    obstacles = [
        ObstacleSpec(
            logical_id=logical_id,
            mode="static",
            geometry=ObstacleGeometry(shape, half_extents),
            position=position,
            orientation=identity,
        )
        for logical_id, shape, half_extents, position in static_values
    ]
    moving_values = (
        (7, "box", (0.16, 0.14, 0.35), (4.0, -3.75), (5.0, -3.75), 0.25),
        (8, "cylinder", (0.16, 0.16, 0.40), (-0.5, -1.25), (0.7, -1.25), 0.22),
        (9, "sphere", (0.16, 0.16, 0.16), (-4.0, 3.75), (-2.8, 3.75), 0.20),
    )
    for logical_id, shape, half_extents, start_xy, end_xy, speed in moving_values:
        obstacles.append(
            ObstacleSpec(
                logical_id=logical_id,
                mode="moving",
                geometry=ObstacleGeometry(shape, half_extents),
                position=(start_xy[0], start_xy[1], 0.0),
                orientation=identity,
                path=ObstaclePath(start_xy, end_xy, speed),
            )
        )
    return tuple(obstacles)


def route_bounds_violations(
    route: GolfRoute,
    bounds: TerrainBounds,
    *,
    vehicle_half_extents: tuple[float, float],
    clearance_m: float,
) -> tuple[tuple[float, float], ...]:
    """用可旋转整车 AABB 的外接圆保守检查完整路线扫掠边界。"""
    half_x, half_y = _xy("vehicle_half_extents", vehicle_half_extents)
    clearance = _finite("clearance_m", clearance_m)
    if half_x <= 0.0 or half_y <= 0.0 or clearance < 0.0:
        raise ValueError("vehicle footprint must be positive and clearance non-negative")
    radius = math.hypot(half_x, half_y) + clearance
    return tuple(
        (x, y)
        for x, y in route.iter_centerline()
        if not (
            bounds.min_x + radius <= x <= bounds.max_x - radius
            and bounds.min_y + radius <= y <= bounds.max_y - radius
        )
    )


def _obstacle_swept_aabb(obstacle: ObstacleSpec) -> tuple[float, float, float, float]:
    half_x, half_y, _half_z = obstacle.geometry.half_extents
    if obstacle.path is None:
        points = ((obstacle.position[0], obstacle.position[1]),)
    else:
        points = (obstacle.path.start_xy, obstacle.path.end_xy)
    return (
        min(point[0] for point in points) - half_x,
        max(point[0] for point in points) + half_x,
        min(point[1] for point in points) - half_y,
        max(point[1] for point in points) + half_y,
    )


def corridor_violations(
    route: GolfRoute,
    obstacles: tuple[ObstacleSpec, ...],
    *,
    vehicle_half_extents: tuple[float, float],
    clearance_m: float,
) -> tuple[int, ...]:
    """返回与整车 AABB+0.5m 路线走廊相交的障碍逻辑 ID。"""
    half_x, half_y = _xy("vehicle_half_extents", vehicle_half_extents)
    clearance = _finite("clearance_m", clearance_m)
    if half_x <= 0.0 or half_y <= 0.0 or clearance < 0.0:
        raise ValueError("vehicle footprint must be positive and clearance non-negative")
    corridor_radius = math.hypot(half_x, half_y) + clearance
    centerline = tuple(route.iter_centerline())
    violations: list[int] = []
    for obstacle in obstacles:
        min_x, max_x, min_y, max_y = _obstacle_swept_aabb(obstacle)
        expanded = (
            min_x - corridor_radius,
            max_x + corridor_radius,
            min_y - corridor_radius,
            max_y + corridor_radius,
        )
        if any(
            expanded[0] <= x <= expanded[1] and expanded[2] <= y <= expanded[3]
            for x, y in centerline
        ):
            violations.append(obstacle.logical_id)
    return tuple(violations)
