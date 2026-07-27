# 阶段三 Protobuf 契约测试：锁定企业消息版本及完整字段描述符。
from importlib import import_module


PACKAGE = "slope_sim.interfaces.v1"
EXPECTED_FIELDS = {
    "WheelCommand": (
        ("timestamp_ns", 1, "TYPE_UINT64", "LABEL_OPTIONAL", ""),
        ("drive_wheel_speed_rad_s", 2, "TYPE_FLOAT", "LABEL_REPEATED", ""),
        (
            "steering_wheel_speed_rad_s",
            3,
            "TYPE_FLOAT",
            "LABEL_REPEATED",
            "",
        ),
    ),
    "WheelState": (
        ("timestamp_ns", 1, "TYPE_UINT64", "LABEL_OPTIONAL", ""),
        ("drive_wheel_speed_rad_s", 2, "TYPE_FLOAT", "LABEL_REPEATED", ""),
        (
            "steering_wheel_angle_rad",
            3,
            "TYPE_FLOAT",
            "LABEL_REPEATED",
            "",
        ),
    ),
    "LidarPoint": (
        ("offset_time_ns", 1, "TYPE_UINT32", "LABEL_OPTIONAL", ""),
        ("x", 2, "TYPE_FLOAT", "LABEL_OPTIONAL", ""),
        ("y", 3, "TYPE_FLOAT", "LABEL_OPTIONAL", ""),
        ("z", 4, "TYPE_FLOAT", "LABEL_OPTIONAL", ""),
        ("reflectivity", 5, "TYPE_UINT32", "LABEL_OPTIONAL", ""),
        ("tag", 6, "TYPE_UINT32", "LABEL_OPTIONAL", ""),
        ("line", 7, "TYPE_UINT32", "LABEL_OPTIONAL", ""),
    ),
    "LidarPointCloud": (
        ("timebase_ns", 1, "TYPE_UINT64", "LABEL_OPTIONAL", ""),
        ("frame_id", 2, "TYPE_STRING", "LABEL_OPTIONAL", ""),
        ("point_num", 3, "TYPE_UINT32", "LABEL_OPTIONAL", ""),
        ("lidar_id", 4, "TYPE_UINT32", "LABEL_OPTIONAL", ""),
        (
            "points",
            5,
            "TYPE_MESSAGE",
            "LABEL_REPEATED",
            f".{PACKAGE}.LidarPoint",
        ),
    ),
    "RtkState": (
        ("timestamp_ns", 1, "TYPE_UINT64", "LABEL_OPTIONAL", ""),
        ("main_x", 2, "TYPE_DOUBLE", "LABEL_OPTIONAL", ""),
        ("main_y", 3, "TYPE_DOUBLE", "LABEL_OPTIONAL", ""),
        ("main_z", 4, "TYPE_DOUBLE", "LABEL_OPTIONAL", ""),
        ("baseline_yaw_rad", 5, "TYPE_DOUBLE", "LABEL_OPTIONAL", ""),
    ),
    "ImuAttitude": (
        ("timestamp_ns", 1, "TYPE_UINT64", "LABEL_OPTIONAL", ""),
        ("roll_rad", 2, "TYPE_DOUBLE", "LABEL_OPTIONAL", ""),
        ("pitch_rad", 3, "TYPE_DOUBLE", "LABEL_OPTIONAL", ""),
    ),
}


def _load_generated_descriptor():
    """加载真实生成模块，并还原可稳定断言的文件描述符。"""
    descriptor_pb2 = import_module("google.protobuf.descriptor_pb2")
    generated = import_module(
        "slope_sim.interfaces.generated.slope_sim_interfaces_pb2"
    )
    file_descriptor = descriptor_pb2.FileDescriptorProto()
    file_descriptor.ParseFromString(generated.DESCRIPTOR.serialized_pb)
    return descriptor_pb2, generated, file_descriptor


def test_proto_file_uses_versioned_proto3_package():
    _, _, file_descriptor = _load_generated_descriptor()

    assert file_descriptor.name == "slope_sim_interfaces.proto"
    assert file_descriptor.syntax == "proto3"
    assert file_descriptor.package == PACKAGE


def test_enterprise_message_descriptors_are_exact():
    descriptor_pb2, generated, file_descriptor = _load_generated_descriptor()

    assert tuple(message.name for message in file_descriptor.message_type) == tuple(
        EXPECTED_FIELDS
    )
    assert {
        descriptor.full_name
        for descriptor in generated.DESCRIPTOR.message_types_by_name.values()
    } == {f"{PACKAGE}.{message_name}" for message_name in EXPECTED_FIELDS}

    for message in file_descriptor.message_type:
        actual_fields = tuple(
            (
                field.name,
                field.number,
                descriptor_pb2.FieldDescriptorProto.Type.Name(field.type),
                descriptor_pb2.FieldDescriptorProto.Label.Name(field.label),
                field.type_name,
            )
            for field in message.field
        )
        assert actual_fields == EXPECTED_FIELDS[message.name]
