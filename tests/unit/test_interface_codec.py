# Protobuf 编解码单元测试：锁定五种企业消息的显式映射与输入边界。
from __future__ import annotations

import math

import pytest

from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.generated import slope_sim_interfaces_pb2 as pb
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPoint,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)


def _point_cloud() -> LidarPointCloud:
    """构造仅含 float32 可精确表示数值的双点点云。"""
    return LidarPointCloud(
        timebase_ns=100,
        frame_id="lidar_front",
        point_num=2,
        lidar_id=5,
        points=(
            LidarPoint(1, 0.25, 0.5, 1.0, 64, 1, 2),
            LidarPoint(2, 1.5, -2.0, 0.0, 192, 3, 15),
        ),
    )


@pytest.mark.parametrize(
    ("source", "decoder_name", "expected_type_name"),
    (
        pytest.param(
            WheelCommand(10, (0.25, -2.0), (0.5,)),
            "decode_wheel_command",
            "slope_sim.interfaces.v1.WheelCommand",
            id="wheel-command",
        ),
        pytest.param(
            WheelState(20, (1.0, 1.5), (-2.0,)),
            "decode_wheel_state",
            "slope_sim.interfaces.v1.WheelState",
            id="wheel-state",
        ),
        pytest.param(
            _point_cloud(),
            "decode_lidar_point_cloud",
            "slope_sim.interfaces.v1.LidarPointCloud",
            id="lidar-point-cloud",
        ),
        pytest.param(
            RtkState(30, 1.5, -2.0, 0.5, 0.25),
            "decode_rtk_state",
            "slope_sim.interfaces.v1.RtkState",
            id="rtk-state",
        ),
        pytest.param(
            ImuAttitude(40, 0.25, -2.0),
            "decode_imu_attitude",
            "slope_sim.interfaces.v1.ImuAttitude",
            id="imu-attitude",
        ),
    ),
)
def test_codec_round_trips_each_top_level_message_with_full_type_name(
    source: object,
    decoder_name: str,
    expected_type_name: str,
):
    codec = ProtoCodec()

    payload = codec.encode(source)

    assert isinstance(payload, bytes)
    assert getattr(codec, decoder_name)(payload) == source
    assert codec.type_name(source) == expected_type_name


def test_lidar_point_cloud_preserves_both_points_fields_order_and_count():
    source = _point_cloud()

    decoded = ProtoCodec().decode_lidar_point_cloud(ProtoCodec().encode(source))

    assert decoded.point_num == 2
    assert decoded.points == source.points
    assert tuple(
        (
            point.offset_time_ns,
            point.x,
            point.y,
            point.z,
            point.reflectivity,
            point.tag,
            point.line,
        )
        for point in decoded.points
    ) == (
        (1, 0.25, 0.5, 1.0, 64, 1, 2),
        (2, 1.5, -2.0, 0.0, 192, 3, 15),
    )


def test_encode_wheel_command_maps_each_field_to_protobuf():
    source = WheelCommand(101, (0.25, 0.5), (1.0, 1.5))
    message = pb.WheelCommand()

    message.ParseFromString(ProtoCodec().encode(source))

    assert message.timestamp_ns == 101
    assert tuple(message.drive_wheel_speed_rad_s) == (0.25, 0.5)
    assert tuple(message.steering_wheel_speed_rad_s) == (1.0, 1.5)


def test_encode_wheel_state_maps_each_field_to_protobuf():
    source = WheelState(102, (-2.0, 0.25), (0.5, 1.5))
    message = pb.WheelState()

    message.ParseFromString(ProtoCodec().encode(source))

    assert message.timestamp_ns == 102
    assert tuple(message.drive_wheel_speed_rad_s) == (-2.0, 0.25)
    assert tuple(message.steering_wheel_angle_rad) == (0.5, 1.5)


def test_encode_lidar_point_cloud_maps_each_field_and_point_order_to_protobuf():
    message = pb.LidarPointCloud()

    message.ParseFromString(ProtoCodec().encode(_point_cloud()))

    assert message.timebase_ns == 100
    assert message.frame_id == "lidar_front"
    assert message.point_num == 2
    assert message.lidar_id == 5
    assert tuple(
        (point.offset_time_ns, point.x, point.y, point.z, point.reflectivity, point.tag, point.line)
        for point in message.points
    ) == (
        (1, 0.25, 0.5, 1.0, 64, 1, 2),
        (2, 1.5, -2.0, 0.0, 192, 3, 15),
    )


def test_encode_rtk_state_maps_each_field_to_protobuf():
    source = RtkState(103, 0.25, 0.5, 1.5, -2.0)
    message = pb.RtkState()

    message.ParseFromString(ProtoCodec().encode(source))

    assert message.timestamp_ns == 103
    assert message.main_x == 0.25
    assert message.main_y == 0.5
    assert message.main_z == 1.5
    assert message.baseline_yaw_rad == -2.0


def test_encode_imu_attitude_maps_each_field_to_protobuf():
    source = ImuAttitude(104, 0.25, -2.0)
    message = pb.ImuAttitude()

    message.ParseFromString(ProtoCodec().encode(source))

    assert message.timestamp_ns == 104
    assert message.roll_rad == 0.25
    assert message.pitch_rad == -2.0


def test_decode_wheel_command_maps_each_protobuf_field_to_model():
    message = pb.WheelCommand(
        timestamp_ns=201,
        drive_wheel_speed_rad_s=[0.25, 0.5],
        steering_wheel_speed_rad_s=[1.0, 1.5],
    )

    decoded = ProtoCodec().decode_wheel_command(message.SerializeToString())

    assert decoded.timestamp_ns == 201
    assert decoded.drive_wheel_speed_rad_s == (0.25, 0.5)
    assert decoded.steering_wheel_speed_rad_s == (1.0, 1.5)


def test_decode_wheel_state_maps_each_protobuf_field_to_model():
    message = pb.WheelState(
        timestamp_ns=202,
        drive_wheel_speed_rad_s=[-2.0, 0.25],
        steering_wheel_angle_rad=[0.5, 1.5],
    )

    decoded = ProtoCodec().decode_wheel_state(message.SerializeToString())

    assert decoded.timestamp_ns == 202
    assert decoded.drive_wheel_speed_rad_s == (-2.0, 0.25)
    assert decoded.steering_wheel_angle_rad == (0.5, 1.5)


def test_decode_lidar_point_cloud_maps_each_protobuf_field_and_point_order_to_model():
    message = pb.LidarPointCloud(
        timebase_ns=203,
        frame_id="lidar_rear",
        point_num=2,
        lidar_id=6,
        points=[
            pb.LidarPoint(
                offset_time_ns=3,
                x=0.25,
                y=0.5,
                z=1.0,
                reflectivity=80,
                tag=1,
                line=4,
            ),
            pb.LidarPoint(
                offset_time_ns=4,
                x=1.5,
                y=-2.0,
                z=0.0,
                reflectivity=200,
                tag=3,
                line=14,
            ),
        ],
    )

    decoded = ProtoCodec().decode_lidar_point_cloud(message.SerializeToString())

    assert decoded.timebase_ns == 203
    assert decoded.frame_id == "lidar_rear"
    assert decoded.point_num == 2
    assert decoded.lidar_id == 6
    assert tuple(
        (point.offset_time_ns, point.x, point.y, point.z, point.reflectivity, point.tag, point.line)
        for point in decoded.points
    ) == (
        (3, 0.25, 0.5, 1.0, 80, 1, 4),
        (4, 1.5, -2.0, 0.0, 200, 3, 14),
    )


def test_decode_rtk_state_maps_each_protobuf_field_to_model():
    message = pb.RtkState(
        timestamp_ns=204,
        main_x=0.25,
        main_y=0.5,
        main_z=1.5,
        baseline_yaw_rad=-2.0,
    )

    decoded = ProtoCodec().decode_rtk_state(message.SerializeToString())

    assert decoded.timestamp_ns == 204
    assert decoded.main_x == 0.25
    assert decoded.main_y == 0.5
    assert decoded.main_z == 1.5
    assert decoded.baseline_yaw_rad == -2.0


def test_decode_imu_attitude_maps_each_protobuf_field_to_model():
    message = pb.ImuAttitude(timestamp_ns=205, roll_rad=0.25, pitch_rad=-2.0)

    decoded = ProtoCodec().decode_imu_attitude(message.SerializeToString())

    assert decoded.timestamp_ns == 205
    assert decoded.roll_rad == 0.25
    assert decoded.pitch_rad == -2.0


@pytest.mark.parametrize(
    ("source", "proto_type"),
    (
        (WheelCommand(1, (), ()), pb.WheelCommand),
        (WheelState(1, (), ()), pb.WheelState),
        (LidarPointCloud(1, "lidar_front", 0, 1, ()), pb.LidarPointCloud),
        (RtkState(1, 0.0, 0.0, 0.0, 0.0), pb.RtkState),
        (ImuAttitude(1, 0.0, 0.0), pb.ImuAttitude),
    ),
)
def test_type_name_matches_generated_descriptor(source: object, proto_type):
    assert ProtoCodec().type_name(source) == proto_type.DESCRIPTOR.full_name


def test_float32_round_trip_exposes_protobuf_quantization_without_strict_model_equality():
    codec = ProtoCodec()
    source = WheelCommand(1, (0.1,), ())
    payload = codec.encode(source)
    wire_message = pb.WheelCommand()
    wire_message.ParseFromString(payload)

    decoded_value = codec.decode_wheel_command(payload).drive_wheel_speed_rad_s[0]
    wire_value = wire_message.drive_wheel_speed_rad_s[0]

    assert decoded_value == wire_value
    assert decoded_value == pytest.approx(0.1)
    assert decoded_value != 0.1


NON_FINITE_VALUES = (
    pytest.param(math.nan, id="nan"),
    pytest.param(math.inf, id="positive-infinity"),
    pytest.param(-math.inf, id="negative-infinity"),
)


@pytest.mark.parametrize(
    "field_name",
    ("drive_wheel_speed_rad_s", "steering_wheel_speed_rad_s"),
)
@pytest.mark.parametrize("invalid", NON_FINITE_VALUES)
def test_decode_wheel_command_rejects_each_non_finite_float_field(field_name: str, invalid: float):
    message = pb.WheelCommand(timestamp_ns=1, **{field_name: [invalid]})

    with pytest.raises(ValueError, match="finite"):
        ProtoCodec().decode_wheel_command(message.SerializeToString())


@pytest.mark.parametrize(
    "field_name",
    ("drive_wheel_speed_rad_s", "steering_wheel_angle_rad"),
)
@pytest.mark.parametrize("invalid", NON_FINITE_VALUES)
def test_decode_wheel_state_rejects_each_non_finite_float_field(field_name: str, invalid: float):
    message = pb.WheelState(timestamp_ns=1, **{field_name: [invalid]})

    with pytest.raises(ValueError, match="finite"):
        ProtoCodec().decode_wheel_state(message.SerializeToString())


@pytest.mark.parametrize("field_name", ("x", "y", "z"))
@pytest.mark.parametrize("invalid", NON_FINITE_VALUES)
def test_decode_lidar_cloud_rejects_each_non_finite_coordinate(field_name: str, invalid: float):
    message = pb.LidarPointCloud(
        timebase_ns=1,
        frame_id="lidar_front",
        point_num=1,
        lidar_id=1,
        points=[pb.LidarPoint(**{field_name: invalid})],
    )

    with pytest.raises(ValueError, match="finite"):
        ProtoCodec().decode_lidar_point_cloud(message.SerializeToString())


@pytest.mark.parametrize("field_name", ("main_x", "main_y", "main_z", "baseline_yaw_rad"))
@pytest.mark.parametrize("invalid", NON_FINITE_VALUES)
def test_decode_rtk_state_rejects_each_non_finite_double_field(field_name: str, invalid: float):
    message = pb.RtkState(timestamp_ns=1, **{field_name: invalid})

    with pytest.raises(ValueError, match="finite"):
        ProtoCodec().decode_rtk_state(message.SerializeToString())


@pytest.mark.parametrize("field_name", ("roll_rad", "pitch_rad"))
@pytest.mark.parametrize("invalid", NON_FINITE_VALUES)
def test_decode_imu_attitude_rejects_each_non_finite_double_field(field_name: str, invalid: float):
    message = pb.ImuAttitude(timestamp_ns=1, **{field_name: invalid})

    with pytest.raises(ValueError, match="finite"):
        ProtoCodec().decode_imu_attitude(message.SerializeToString())


@pytest.mark.parametrize(
    ("source", "message_type"),
    (
        (
            WheelCommand(1, (1e39,), ()),
            "slope_sim.interfaces.v1.WheelCommand",
        ),
        (
            LidarPointCloud(
                1,
                "lidar_front",
                1,
                1,
                (LidarPoint(0, 1e39, 0.0, 0.0, 0, 0, 0),),
            ),
            "slope_sim.interfaces.v1.LidarPointCloud",
        ),
    ),
)
def test_encode_wraps_float32_overflow_with_full_message_type(source: object, message_type: str):
    with pytest.raises(ValueError, match=message_type.replace(".", r"\.")):
        ProtoCodec().encode(source)


def test_lidar_cloud_round_trip_preserves_full_2880_point_frame_order():
    point_count = 2880
    source = LidarPointCloud(
        timebase_ns=500,
        frame_id="lidar_front",
        point_num=point_count,
        lidar_id=7,
        points=tuple(
            LidarPoint(
                offset_time_ns=index,
                x=float(index),
                y=-float(index),
                z=(index % 4) * 0.25,
                reflectivity=index,
                tag=index % 4,
                line=index % 16,
            )
            for index in range(point_count)
        ),
    )

    decoded = ProtoCodec().decode_lidar_point_cloud(ProtoCodec().encode(source))

    assert decoded.point_num == point_count
    assert len(decoded.points) == point_count
    assert decoded.points[0] == source.points[0]
    assert decoded.points[-1] == source.points[-1]
    assert tuple(point.offset_time_ns for point in decoded.points) == tuple(range(point_count))


@pytest.mark.parametrize(
    ("decoder_name", "message_type"),
    (
        ("decode_wheel_command", "WheelCommand"),
        ("decode_wheel_state", "WheelState"),
        ("decode_lidar_point_cloud", "LidarPointCloud"),
        ("decode_rtk_state", "RtkState"),
        ("decode_imu_attitude", "ImuAttitude"),
    ),
)
def test_each_decoder_wraps_malformed_wire_error(decoder_name: str, message_type: str):
    with pytest.raises(ValueError, match=message_type):
        getattr(ProtoCodec(), decoder_name)(b"\xff")


@pytest.mark.parametrize(
    ("message", "error_match"),
    (
        pytest.param(
            pb.LidarPointCloud(
                timebase_ns=1,
                frame_id="lidar_front",
                point_num=2,
                lidar_id=1,
                points=[pb.LidarPoint(x=0.25)],
            ),
            "point_num",
            id="point-count-mismatch",
        ),
        pytest.param(
            pb.LidarPointCloud(
                timebase_ns=1,
                frame_id="lidar_front",
                point_num=1,
                lidar_id=1,
                points=[pb.LidarPoint(x=math.nan)],
            ),
            "finite",
            id="nan-coordinate",
        ),
        pytest.param(
            pb.LidarPointCloud(
                timebase_ns=1,
                frame_id="lidar_front",
                point_num=1,
                lidar_id=1,
                points=[pb.LidarPoint(line=16)],
            ),
            "line",
            id="line-16",
        ),
        pytest.param(
            pb.LidarPointCloud(
                timebase_ns=1,
                frame_id="lidar_front",
                point_num=1,
                lidar_id=1,
                points=[pb.LidarPoint(tag=4)],
            ),
            "tag",
            id="tag-4",
        ),
        pytest.param(
            pb.LidarPointCloud(timebase_ns=1, frame_id="", point_num=0, lidar_id=1),
            "frame_id",
            id="empty-frame",
        ),
    ),
)
def test_lidar_decoder_reapplies_model_invariants(message: pb.LidarPointCloud, error_match: str):
    with pytest.raises(ValueError, match=error_match):
        ProtoCodec().decode_lidar_point_cloud(message.SerializeToString())


@pytest.mark.parametrize(
    "unsupported",
    (pytest.param(object(), id="object"), pytest.param(LidarPoint(0, 0.0, 0.0, 0.0, 0, 0, 0), id="nested-point")),
)
def test_encode_and_type_name_reject_non_top_level_messages(unsupported: object):
    codec = ProtoCodec()

    with pytest.raises(TypeError):
        codec.encode(unsupported)
    with pytest.raises(TypeError):
        codec.type_name(unsupported)


@pytest.mark.parametrize(
    "decoder_name",
    (
        "decode_wheel_command",
        "decode_wheel_state",
        "decode_lidar_point_cloud",
        "decode_rtk_state",
        "decode_imu_attitude",
    ),
)
def test_each_decoder_rejects_str_payload(decoder_name: str):
    with pytest.raises(TypeError, match="payload"):
        getattr(ProtoCodec(), decoder_name)("not-bytes")


@pytest.mark.parametrize("payload_factory", (bytearray, memoryview))
def test_decoder_accepts_bytearray_and_memoryview(payload_factory):
    codec = ProtoCodec()
    source = WheelCommand(50, (0.25, 1.5), ())

    assert codec.decode_wheel_command(payload_factory(codec.encode(source))) == source


def test_empty_but_wire_valid_wheel_command_decodes_without_vehicle_length_check():
    assert ProtoCodec().decode_wheel_command(b"") == WheelCommand(0, (), ())
