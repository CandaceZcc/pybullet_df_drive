# 企业接口模型单元测试：锁定不可变消息、数值边界和车型命令原子校验。
from __future__ import annotations

from dataclasses import FrozenInstanceError, replace
import math

import pytest

from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPoint,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelCommandMechanicalLimitError,
    WheelCommandModelMismatchError,
    WheelState,
    validate_wheel_command,
)
from slope_sim.model_registry import get_robot_model


UINT32_MAX = (1 << 32) - 1
UINT64_MAX = (1 << 64) - 1


def _point() -> LidarPoint:
    return LidarPoint(0, 1.0, 2.0, 3.0, 160, 2, 5)


def test_message_sequences_are_normalized_without_vehicle_length_checks():
    drive_values = [1.0]
    command = WheelCommand(1, drive_values, [0.1, 0.2, 0.3])
    state = WheelState(2, [2.0, 3.0], [0.2])
    points = [_point()]
    cloud = LidarPointCloud(3, "lidar_front", 1, 1, points)

    drive_values.append(2.0)
    points.clear()

    assert command.drive_wheel_speed_rad_s == (1.0,)
    assert command.steering_wheel_speed_rad_s == (0.1, 0.2, 0.3)
    assert state.drive_wheel_speed_rad_s == (2.0, 3.0)
    assert state.steering_wheel_angle_rad == (0.2,)
    assert cloud.points == (_point(),)
    assert WheelCommand(4, (1.0, 2.0), ()).drive_wheel_speed_rad_s == (1.0, 2.0)
    assert WheelCommand(5, range(3), ()).drive_wheel_speed_rad_s == (0.0, 1.0, 2.0)


@pytest.mark.parametrize("field_name", ["drive", "steering"])
@pytest.mark.parametrize(
    "sequence_factory",
    [
        pytest.param(lambda: {1.0, 2.0}, id="set"),
        pytest.param(lambda: {1.0: "left", 2.0: "right"}, id="dict"),
        pytest.param(lambda: frozenset({1.0, 2.0}), id="frozenset"),
        pytest.param(lambda: (value for value in (1.0, 2.0)), id="generator"),
        pytest.param(lambda: "12", id="str"),
        pytest.param(lambda: b"12", id="bytes"),
    ],
)
def test_wheel_command_rejects_unordered_or_one_shot_repeated_fields(field_name: str, sequence_factory):
    values = sequence_factory()
    drive = values if field_name == "drive" else ()
    steering = values if field_name == "steering" else ()

    with pytest.raises(ValueError, match="ordered sequence"):
        WheelCommand(1, drive, steering)


@pytest.mark.parametrize(
    "points_factory",
    [
        pytest.param(lambda point: {point}, id="set"),
        pytest.param(lambda point: {point: "value"}, id="dict"),
        pytest.param(lambda point: frozenset({point}), id="frozenset"),
        pytest.param(lambda point: (item for item in (point,)), id="generator"),
        pytest.param(lambda _point_value: "point", id="str"),
        pytest.param(lambda _point_value: b"point", id="bytes"),
    ],
)
def test_lidar_cloud_rejects_unordered_or_one_shot_points(points_factory):
    point = _point()

    with pytest.raises(ValueError, match="ordered sequence"):
        LidarPointCloud(1, "lidar_front", 1, 1, points_factory(point))


@pytest.mark.parametrize(
    ("instance", "field_name"),
    [
        (WheelCommand(1, (), ()), "timestamp_ns"),
        (WheelState(1, (), ()), "timestamp_ns"),
        (_point(), "x"),
        (LidarPointCloud(1, "lidar_front", 0, 1, ()), "frame_id"),
        (RtkState(1, 0.0, 0.0, 0.0, 0.0), "main_x"),
        (ImuAttitude(1, 0.0, 0.0), "roll_rad"),
    ],
)
def test_all_enterprise_messages_are_frozen(instance: object, field_name: str):
    with pytest.raises(FrozenInstanceError):
        setattr(instance, field_name, None)


@pytest.mark.parametrize("invalid", [True, False, math.nan, math.inf, -math.inf])
@pytest.mark.parametrize(
    "factory",
    [
        pytest.param(lambda value: WheelCommand(1, (value,), ()), id="command-drive"),
        pytest.param(lambda value: WheelCommand(1, (), (value,)), id="command-steering"),
        pytest.param(lambda value: WheelState(1, (value,), ()), id="state-drive"),
        pytest.param(lambda value: WheelState(1, (), (value,)), id="state-steering"),
        pytest.param(lambda value: LidarPoint(0, value, 0.0, 0.0, 0, 0, 0), id="point-x"),
        pytest.param(lambda value: LidarPoint(0, 0.0, value, 0.0, 0, 0, 0), id="point-y"),
        pytest.param(lambda value: LidarPoint(0, 0.0, 0.0, value, 0, 0, 0), id="point-z"),
        pytest.param(lambda value: RtkState(1, value, 0.0, 0.0, 0.0), id="rtk-main-x"),
        pytest.param(lambda value: RtkState(1, 0.0, value, 0.0, 0.0), id="rtk-main-y"),
        pytest.param(lambda value: RtkState(1, 0.0, 0.0, value, 0.0), id="rtk-main-z"),
        pytest.param(lambda value: RtkState(1, 0.0, 0.0, 0.0, value), id="rtk-yaw"),
        pytest.param(lambda value: ImuAttitude(1, value, 0.0), id="imu-roll"),
        pytest.param(lambda value: ImuAttitude(1, 0.0, value), id="imu-pitch"),
    ],
)
def test_every_float_field_rejects_bool_nan_and_infinity(factory, invalid):
    with pytest.raises(ValueError, match="finite"):
        factory(invalid)


@pytest.mark.parametrize("invalid", [True, -1, UINT64_MAX + 1])
@pytest.mark.parametrize(
    "factory",
    [
        pytest.param(lambda value: WheelCommand(value, (), ()), id="command-timestamp"),
        pytest.param(lambda value: WheelState(value, (), ()), id="state-timestamp"),
        pytest.param(
            lambda value: LidarPointCloud(value, "lidar_front", 0, 1, ()),
            id="cloud-timebase",
        ),
        pytest.param(lambda value: RtkState(value, 0.0, 0.0, 0.0, 0.0), id="rtk-timestamp"),
        pytest.param(lambda value: ImuAttitude(value, 0.0, 0.0), id="imu-timestamp"),
    ],
)
def test_uint64_fields_reject_bool_negative_and_overflow(factory, invalid):
    with pytest.raises(ValueError, match="uint64"):
        factory(invalid)


@pytest.mark.parametrize("invalid", [True, -1, UINT32_MAX + 1])
@pytest.mark.parametrize(
    "factory",
    [
        pytest.param(lambda value: LidarPoint(value, 0.0, 0.0, 0.0, 0, 0, 0), id="offset-time"),
        pytest.param(lambda value: LidarPoint(0, 0.0, 0.0, 0.0, value, 0, 0), id="reflectivity"),
        pytest.param(lambda value: LidarPoint(0, 0.0, 0.0, 0.0, 0, value, 0), id="tag"),
        pytest.param(lambda value: LidarPoint(0, 0.0, 0.0, 0.0, 0, 0, value), id="line"),
        pytest.param(
            lambda value: LidarPointCloud(0, "lidar_front", value, 1, ()),
            id="point-num",
        ),
        pytest.param(
            lambda value: LidarPointCloud(0, "lidar_front", 0, value, ()),
            id="lidar-id",
        ),
    ],
)
def test_uint32_fields_reject_bool_negative_and_overflow(factory, invalid):
    with pytest.raises(ValueError, match="uint32"):
        factory(invalid)


def test_unsigned_integer_boundaries_are_accepted():
    point = LidarPoint(UINT32_MAX, 0.0, 0.0, 0.0, UINT32_MAX, 3, 15)
    cloud = LidarPointCloud(UINT64_MAX, "lidar_front", 1, UINT32_MAX, (point,))

    assert WheelCommand(UINT64_MAX, (), ()).timestamp_ns == UINT64_MAX
    assert WheelState(UINT64_MAX, (), ()).timestamp_ns == UINT64_MAX
    assert RtkState(UINT64_MAX, 0.0, 0.0, 0.0, 0.0).timestamp_ns == UINT64_MAX
    assert ImuAttitude(UINT64_MAX, 0.0, 0.0).timestamp_ns == UINT64_MAX
    assert point.offset_time_ns == UINT32_MAX
    assert point.reflectivity == UINT32_MAX
    assert cloud.timebase_ns == UINT64_MAX
    assert cloud.lidar_id == UINT32_MAX


@pytest.mark.parametrize("tag", [-1, 4])
def test_lidar_tag_is_limited_to_known_semantics(tag: int):
    with pytest.raises(ValueError, match="tag"):
        LidarPoint(0, 0.0, 0.0, 0.0, 0, tag, 0)


@pytest.mark.parametrize("line", [-1, 16])
def test_lidar_line_is_limited_to_sixteen_scan_lines(line: int):
    with pytest.raises(ValueError, match="line"):
        LidarPoint(0, 0.0, 0.0, 0.0, 0, 0, line)


def test_lidar_cloud_requires_nonempty_frame_and_matching_point_count():
    point = _point()
    with pytest.raises(ValueError, match="frame_id"):
        LidarPointCloud(1, "", 1, 1, (point,))
    with pytest.raises(ValueError, match="point_num"):
        LidarPointCloud(1, "lidar_front", 2, 1, (point,))


@pytest.mark.parametrize("model_name", ["df_front", "df_mid", "df_back"])
def test_each_differential_model_requires_two_drive_values_and_no_steering(model_name: str):
    model = get_robot_model(model_name)
    command = WheelCommand(10, (1.0, -2.0), ())

    assert validate_wheel_command(command, model) is command
    with pytest.raises(ValueError, match="2 drive"):
        validate_wheel_command(WheelCommand(10, (1.0,), ()), model)
    with pytest.raises(ValueError, match="2 drive"):
        validate_wheel_command(WheelCommand(10, (1.0, 2.0, 3.0), ()), model)
    with pytest.raises(ValueError, match="0 steering"):
        validate_wheel_command(WheelCommand(10, (1.0, 2.0), (0.1,)), model)


def test_active_steering_requires_four_drive_and_two_steering_values():
    model = get_robot_model("active_steering_4wd")
    command = WheelCommand(10, (1.0, 2.0, 3.0, 4.0), (0.5, -0.5))

    assert validate_wheel_command(command, model) is command
    with pytest.raises(ValueError, match="4 drive"):
        validate_wheel_command(WheelCommand(10, (1.0, 2.0, 3.0), (0.5, -0.5)), model)
    with pytest.raises(ValueError, match="4 drive"):
        validate_wheel_command(WheelCommand(10, (1.0, 2.0, 3.0, 4.0, 5.0), (0.5, -0.5)), model)
    with pytest.raises(ValueError, match="2 steering"):
        validate_wheel_command(WheelCommand(10, (1.0, 2.0, 3.0, 4.0), (0.5,)), model)
    with pytest.raises(ValueError, match="2 steering"):
        validate_wheel_command(WheelCommand(10, (1.0, 2.0, 3.0, 4.0), (0.5, -0.5, 0.0)), model)


def test_wheel_command_length_rejection_has_structured_model_mismatch_type():
    with pytest.raises(WheelCommandModelMismatchError):
        validate_wheel_command(
            WheelCommand(10, (1.0, 2.0, 3.0), ()),
            get_robot_model("df_mid"),
        )


def test_wheel_command_limit_rejection_has_structured_mechanical_limit_type():
    with pytest.raises(WheelCommandMechanicalLimitError):
        validate_wheel_command(
            WheelCommand(10, (20.01, 0.0), ()),
            get_robot_model("df_mid"),
        )


@pytest.mark.parametrize("value", [20.01, -20.01])
def test_validate_rejects_drive_values_outside_mechanical_limit(value: float):
    with pytest.raises(ValueError):
        validate_wheel_command(WheelCommand(10, (value, 0.0), ()), get_robot_model("df_mid"))


@pytest.mark.parametrize("value", [2.01, -2.01])
def test_validate_rejects_steering_values_outside_mechanical_limit(value: float):
    with pytest.raises(ValueError):
        validate_wheel_command(
            WheelCommand(10, (0.0, 0.0, 0.0, 0.0), (value, 0.0)),
            get_robot_model("active_steering_4wd"),
        )


@pytest.mark.parametrize("field_name", ["max_drive_wheel_speed_rad_s", "max_steering_speed_rad_s"])
@pytest.mark.parametrize("invalid", [math.nan, math.inf, -1.0, True])
def test_validate_rejects_invalid_model_speed_limits_with_exact_field_name(field_name: str, invalid):
    model = replace(get_robot_model("active_steering_4wd"), **{field_name: invalid})
    command = WheelCommand(10, (1.0, 1.0, 1.0, 1.0), (0.5, -0.5))

    with pytest.raises(ValueError, match=field_name):
        validate_wheel_command(command, model)


def test_wheel_command_accepts_symmetric_mechanical_boundaries_without_mutation():
    differential = WheelCommand(10, (-20.0, 20.0), ())
    active = WheelCommand(11, (-20.0, 20.0, 20.0, -20.0), (-2.0, 2.0))
    active_values = (active.drive_wheel_speed_rad_s, active.steering_wheel_speed_rad_s)

    assert validate_wheel_command(differential, get_robot_model("df_back")) is differential
    assert validate_wheel_command(active, get_robot_model("active_steering_4wd")) is active
    assert (active.drive_wheel_speed_rad_s, active.steering_wheel_speed_rad_s) == active_values
