import pytest

from slope_sim.controller import wheel_speeds_from_twist


def test_wheel_speeds_from_twist_converts_linear_and_angular_velocity():
    left, right = wheel_speeds_from_twist(
        linear_velocity=1.0,
        angular_velocity=2.0,
        wheel_base=0.5,
        wheel_radius=0.1,
    )

    assert left == pytest.approx(5.0)
    assert right == pytest.approx(15.0)


def test_wheel_speeds_from_twist_rejects_non_positive_geometry():
    with pytest.raises(ValueError, match="wheel_base"):
        wheel_speeds_from_twist(1.0, 0.0, wheel_base=0.0, wheel_radius=0.1)

    with pytest.raises(ValueError, match="wheel_radius"):
        wheel_speeds_from_twist(1.0, 0.0, wheel_base=0.5, wheel_radius=-0.1)

