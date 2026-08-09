"""阶段四 A：锁定 v2 Protobuf schema 的线协议字段合同。"""
from importlib import import_module

import pytest
from google.protobuf import descriptor_pb2


TOP_LEVEL = (
    "WheelCommand",
    "WheelState",
    "LidarPointCloud",
    "RtkState",
    "ImuAttitude",
)

EXPECTED_FIELDS = {
    "WheelCommand": (
        ("timestamp_ns", 1), ("drive_wheel_speed_rad_s", 2),
        ("steering_wheel_speed_rad_s", 3), ("sequence", 4),
        ("world_generation", 5), ("command_generation", 6),
        ("source_id", 7), ("source_session_id", 8), ("robot_model", 9),
        ("simulation_session_id", 10), ("descriptor_sha256", 11),
    ),
    "WheelState": (
        ("timestamp_ns", 1), ("drive_wheel_speed_rad_s", 2),
        ("steering_wheel_angle_rad", 3), ("sequence", 4),
        ("world_generation", 5), ("command_generation", 6), ("robot_model", 7),
        ("simulation_session_id", 8), ("descriptor_sha256", 9),
        ("command_authority_state", 10), ("command_owner_source_id", 11),
        ("command_owner_source_session_id", 12), ("command_peer_count", 13),
    ),
    "LidarPoint": (
        ("offset_time_ns", 1), ("x", 2), ("y", 3), ("z", 4),
        ("reflectivity", 5), ("tag", 6), ("line", 7),
    ),
    "LidarPointCloud": (
        ("timebase_ns", 1), ("frame_id", 2), ("point_num", 3), ("lidar_id", 4),
        ("points", 5), ("sequence", 6), ("world_generation", 7),
        ("simulation_session_id", 8), ("descriptor_sha256", 9),
    ),
    "Point3d": (("x_m", 1), ("y_m", 2), ("z_m", 3)),
    "RtkState": (
        ("timestamp_ns", 1), ("sequence", 2), ("world_generation", 3),
        ("frame_id", 4), ("left", 5), ("center", 6), ("right", 7),
        ("heading_rad", 8), ("simulation_session_id", 9), ("descriptor_sha256", 10),
    ),
    "ImuAttitude": (
        ("timestamp_ns", 1), ("roll_rad", 2), ("pitch_rad", 3), ("sequence", 4),
        ("world_generation", 5), ("frame_id", 6), ("simulation_session_id", 7),
        ("descriptor_sha256", 8),
    ),
}


def require_wished_module(name: str):
    """缺失 v2 模块时给出业务 RED，而不是 pytest collection error。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


def test_v2_package_authority_enum_and_field_numbers_are_exact() -> None:
    """v2 必须固定 package、命令权枚举和全部消息字段号。"""
    pb = require_wished_module("slope_sim.interfaces.generated.slope_sim_interfaces_v2_pb2")
    assert pb.DESCRIPTOR.package == "slope_sim.interfaces.v2"
    enum = pb.DESCRIPTOR.enum_types_by_name["CommandAuthorityState"]
    assert [(value.name, value.number) for value in enum.values] == [
        ("COMMAND_AUTHORITY_UNSPECIFIED", 0), ("WAITING", 1),
        ("CLAIMABLE", 2), ("ACTIVE", 3), ("CONFLICT", 4),
    ]
    for name, expected in EXPECTED_FIELDS.items():
        fields = pb.DESCRIPTOR.message_types_by_name[name].fields
        assert tuple((field.name, field.number) for field in fields) == expected


def test_every_top_level_v2_message_carries_session_and_descriptor() -> None:
    """五个生产话题 payload 都必须携带会话和 descriptor 身份。"""
    pb = require_wished_module("slope_sim.interfaces.generated.slope_sim_interfaces_v2_pb2")
    for name in TOP_LEVEL:
        fields = pb.DESCRIPTOR.message_types_by_name[name].fields_by_name
        assert fields["simulation_session_id"].type == descriptor_pb2.FieldDescriptorProto.TYPE_BYTES
        assert fields["descriptor_sha256"].type == descriptor_pb2.FieldDescriptorProto.TYPE_BYTES
