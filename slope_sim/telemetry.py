# 遥测模块：统一整理小车位姿、速度、轮/履带反馈、接触和传感器数据。
from __future__ import annotations

import math
from dataclasses import asdict, dataclass


def body_frame_velocity(world_velocity: tuple[float, float, float], yaw: float) -> tuple[float, float, float]:
    """把世界坐标速度转换到车体坐标系，便于区分前进速度和侧滑速度。"""
    vx, vy, vz = world_velocity
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    forward = cos_yaw * vx + sin_yaw * vy
    lateral = -sin_yaw * vx + cos_yaw * vy
    return forward, lateral, vz


def slip_ratio(drive_surface_speed: float, body_forward_speed: float, small_value: float = 1e-6) -> float:
    """用驱动表面速度和车体前向速度估计打滑率。"""
    if abs(drive_surface_speed) < small_value:
        return 0.0
    return (drive_surface_speed - body_forward_speed) / max(abs(drive_surface_speed), small_value)


def track_body_speeds(body_forward_speed: float, yaw_rate: float, track_width: float) -> tuple[float, float]:
    """把车体中心前向速度换算为左右履带位置的局部前向速度。"""
    half_width = track_width / 2.0
    return body_forward_speed - yaw_rate * half_width, body_forward_speed + yaw_rate * half_width


@dataclass(frozen=True)
class RobotTelemetry:
    """单个仿真时刻的小车完整遥测数据。"""

    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    body_forward_speed: float = 0.0
    body_lateral_speed: float = 0.0
    body_vertical_speed: float = 0.0
    angular_velocity_x: float = 0.0
    angular_velocity_y: float = 0.0
    angular_velocity_z: float = 0.0
    yaw_rate: float = 0.0
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    command_linear_velocity: float = 0.0
    command_angular_velocity: float = 0.0
    left_target_drive_speed: float = 0.0
    right_target_drive_speed: float = 0.0
    left_actual_drive_speed: float = 0.0
    right_actual_drive_speed: float = 0.0
    left_track_surface_speed: float = 0.0
    right_track_surface_speed: float = 0.0
    left_body_track_speed: float = 0.0
    right_body_track_speed: float = 0.0
    left_drive_position: float = 0.0
    right_drive_position: float = 0.0
    left_motor_torque: float = 0.0
    right_motor_torque: float = 0.0
    ground_lateral_friction: float = 1.0
    drive_lateral_friction: float = 1.0
    left_contact_normal_force: float = 0.0
    right_contact_normal_force: float = 0.0
    contact_count: int = 0
    left_slip_ratio: float = 0.0
    right_slip_ratio: float = 0.0
    body_lateral_slip_speed: float = 0.0
    lidar_min_distance: float = math.nan
    lidar_front_distance: float = math.nan
    lidar_left_distance: float = math.nan
    lidar_right_distance: float = math.nan
    nearest_obstacle_distance: float = math.nan
    bumper_contact: bool = False

    def to_row(
        self,
        reference_x: float = 0.0,
        reference_y: float = 0.0,
        estimated_x: float | None = None,
        estimated_y: float | None = None,
    ) -> dict[str, float | int | bool]:
        """转换成 CSV 行，并补齐旧分析脚本需要的参考/估计轨迹字段。"""
        row = asdict(self)
        row["reference_x"] = reference_x
        row["reference_y"] = reference_y
        row["estimated_x"] = self.x if estimated_x is None else estimated_x
        row["estimated_y"] = self.y if estimated_y is None else estimated_y
        return row


TELEMETRY_FIELDNAMES = list(RobotTelemetry().to_row().keys())
