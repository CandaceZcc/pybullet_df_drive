# 控制器工具：把差速车车体速度 v/w 转换成左右轮角速度。
from __future__ import annotations


def wheel_speeds_from_twist(
    linear_velocity: float,
    angular_velocity: float,
    wheel_base: float,
    wheel_radius: float,
) -> tuple[float, float]:
    """把车体线速度和角速度转换为左右轮角速度，单位是 rad/s。"""
    if wheel_base <= 0:
        raise ValueError("wheel_base must be positive")
    if wheel_radius <= 0:
        raise ValueError("wheel_radius must be positive")

    # 差速车左轮和右轮的线速度由车体前进速度和转向角速度共同决定。
    left_linear = linear_velocity - angular_velocity * wheel_base / 2.0
    right_linear = linear_velocity + angular_velocity * wheel_base / 2.0
    return left_linear / wheel_radius, right_linear / wheel_radius
