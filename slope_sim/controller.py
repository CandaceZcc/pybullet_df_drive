from __future__ import annotations


def wheel_speeds_from_twist(
    linear_velocity: float,
    angular_velocity: float,
    wheel_base: float,
    wheel_radius: float,
) -> tuple[float, float]:
    """Convert body twist command to left/right wheel angular speeds in rad/s."""
    if wheel_base <= 0:
        raise ValueError("wheel_base must be positive")
    if wheel_radius <= 0:
        raise ValueError("wheel_radius must be positive")

    left_linear = linear_velocity - angular_velocity * wheel_base / 2.0
    right_linear = linear_velocity + angular_velocity * wheel_base / 2.0
    return left_linear / wheel_radius, right_linear / wheel_radius

