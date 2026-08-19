"""阶段四 A：v2 五话题、wheel 身份模型和确定性 Protobuf codec。"""
from hashlib import sha256
from importlib import import_module
import math

import pytest


SESSION = bytes.fromhex("00112233445566778899aabbccddeeff")
SOURCE_SESSION = bytes.fromhex("ffeeddccbbaa99887766554433221100")


def require_wished_module(name: str):
    """缺少 v2 模块时输出可读 RED，不让顶层导入破坏收集。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


@pytest.fixture
def descriptor():
    """每个 codec 测试使用同一冻结 descriptor 身份。"""
    return require_wished_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()


def valid_command_values(descriptor):
    """构造符合 v2 identity 边界的基础差速轮命令。"""
    return {
        "timestamp_ns": 20_000_000,
        "drive_wheel_speed_rad_s": (1.25, -1.25),
        "steering_wheel_speed_rad_s": (),
        "sequence": 0,
        "world_generation": 1,
        "command_generation": 1,
        "source_id": "manual.tool-1",
        "source_session_id": SOURCE_SESSION,
        "robot_model": "df_mid",
        "simulation_session_id": SESSION,
        "descriptor_sha256": descriptor.sha256,
    }


def test_v2_topic_contract_is_exact() -> None:
    """v2 只能使用固定五话题、方向、类型名和发布频率。"""
    topics = require_wished_module("slope_sim.interfaces.v2.topics").V2_TOPICS
    assert [(item.topic, item.type_name, item.rate_hz, item.direction) for item in topics] == [
        ("/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand", 100, "subscribe"),
        ("/sim/wheel/state", "slope_sim.interfaces.v2.WheelState", 100, "publish"),
        ("/sim/lidar/points", "slope_sim.interfaces.v2.LidarPointCloud", 10, "publish"),
        ("/sim/rtk/state", "slope_sim.interfaces.v2.RtkState", 10, "publish"),
        ("/sim/imu/attitude", "slope_sim.interfaces.v2.ImuAttitude", 10, "publish"),
    ]


def test_encode_returns_one_deterministic_payload_for_log_and_transport(descriptor) -> None:
    """同一 wheel command 重复编码必须返回同一可同时写日志和发送的 bytes。"""
    codec = require_wished_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    command_type = require_wished_module("slope_sim.interfaces.v2.models").WheelCommandV2
    command = command_type(**valid_command_values(descriptor))
    first = codec.encode(command)
    second = codec.encode(command)
    assert first.payload == second.payload
    assert first.payload_sha256 == sha256(first.payload).digest()
    assert first.type_name == "slope_sim.interfaces.v2.WheelCommand"
    assert codec.decode_wheel_command(first.payload) == command


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("simulation_session_id", b"short"),
        ("descriptor_sha256", b"short"),
        ("source_session_id", b"short"),
        ("source_id", "bad source"),
        ("source_id", "x" * 65),
        ("source_id", "nonascii-中"),
        ("timestamp_ns", True),
        ("timestamp_ns", 1 << 64),
        ("drive_wheel_speed_rad_s", (math.nan,)),
        ("robot_model", ""),
    ],
)
def test_wheel_command_rejects_invalid_identity(field, value, descriptor) -> None:
    """WheelCommand 必须拒绝错误身份、uint 边界和非有限轮速。"""
    command_type = require_wished_module("slope_sim.interfaces.v2.models").WheelCommandV2
    values = valid_command_values(descriptor)
    values[field] = value
    with pytest.raises(ValueError):
        command_type(**values)


def test_wheel_state_requires_exact_peer_count_and_owner(descriptor) -> None:
    """命令权状态必须精确映射 peer count，且只有 ACTIVE 可公开 owner。"""
    models = require_wished_module("slope_sim.interfaces.v2.models")
    values = {
        "timestamp_ns": 1, "drive_wheel_speed_rad_s": (0.0, 0.0),
        "steering_wheel_angle_rad": (), "sequence": 0, "world_generation": 1,
        "command_generation": 1, "robot_model": "df_mid",
        "simulation_session_id": SESSION, "descriptor_sha256": descriptor.sha256,
        "command_authority_state": models.CommandAuthorityState.ACTIVE,
        "command_owner_source_id": "manual.tool-1",
        "command_owner_source_session_id": SOURCE_SESSION, "command_peer_count": 1,
    }
    assert models.WheelStateV2(**values).command_peer_count == 1
    values["command_peer_count"] = 2
    with pytest.raises(ValueError, match="peer count"):
        models.WheelStateV2(**values)
    values["command_authority_state"] = models.CommandAuthorityState.CLAIMABLE
    values["command_peer_count"] = 1
    with pytest.raises(ValueError, match="owner"):
        models.WheelStateV2(**values)


def test_decode_rejects_wrong_descriptor_before_returning_model(descriptor) -> None:
    """错误带内 descriptor 不得进入后续 sequence 或命令权统计。"""
    codec = require_wished_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    command_type = require_wished_module("slope_sim.interfaces.v2.models").WheelCommandV2
    generated = require_wished_module("slope_sim.interfaces.generated.slope_sim_interfaces_v2_pb2")
    command = command_type(**valid_command_values(descriptor))
    message = generated.WheelCommand()
    message.ParseFromString(codec.encode(command).payload)
    message.descriptor_sha256 = b"x" * 32
    with pytest.raises(ValueError, match="descriptor SHA-256 mismatch"):
        codec.decode_wheel_command(message.SerializeToString(deterministic=True))


def test_v2_imu_round_trips_a_deterministic_identity_bound_payload(descriptor) -> None:
    """v2 IMU 必须与 wheel 共用冻结 descriptor、session 和确定性编码边界。"""
    models = require_wished_module("slope_sim.interfaces.v2.models")
    codec = require_wished_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    imu_type = getattr(models, "ImuAttitudeV2", None)
    assert imu_type is not None, "v2 IMU domain model must exist"
    imu = imu_type(
        timestamp_ns=100_000_000,
        roll_rad=0.1,
        pitch_rad=-0.2,
        sequence=1,
        world_generation=1,
        frame_id="base_link",
        simulation_session_id=SESSION,
        descriptor_sha256=descriptor.sha256,
    )

    encoded = codec.encode(imu)

    assert encoded.type_name == "slope_sim.interfaces.v2.ImuAttitude"
    assert codec.decode_imu_attitude(encoded.payload) == imu


def test_v2_rtk_round_trips_three_points_and_identity(descriptor) -> None:
    """v2 RTK 必须原样保留三点、航向和冻结身份字段。"""
    models = require_wished_module("slope_sim.interfaces.v2.models")
    codec = require_wished_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    point_type = getattr(models, "Point3dV2", None)
    rtk_type = getattr(models, "RtkStateV2", None)
    assert point_type is not None, "v2 RTK point model must exist"
    assert rtk_type is not None, "v2 RTK domain model must exist"
    rtk = rtk_type(
        timestamp_ns=100_000_000,
        sequence=1,
        world_generation=1,
        frame_id="world",
        left=point_type(1.2, 2.3, 0.4),
        center=point_type(1.0, 2.0, 0.4),
        right=point_type(0.8, 1.7, 0.4),
        heading_rad=-0.25,
        simulation_session_id=SESSION,
        descriptor_sha256=descriptor.sha256,
    )

    encoded = codec.encode(rtk)

    assert encoded.type_name == "slope_sim.interfaces.v2.RtkState"
    assert codec.decode_rtk_state(encoded.payload) == rtk


def test_v2_lidar_round_trips_center_frame_and_identity(descriptor) -> None:
    """v2 LiDAR 必须保留中心安装帧、逐点字段和冻结身份。"""
    models = require_wished_module("slope_sim.interfaces.v2.models")
    codec = require_wished_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    point_type = getattr(models, "LidarPointV2", None)
    cloud_type = getattr(models, "LidarPointCloudV2", None)
    assert point_type is not None, "v2 LiDAR point model must exist"
    assert cloud_type is not None, "v2 LiDAR cloud model must exist"
    cloud = cloud_type(
        timebase_ns=100_000_000,
        frame_id="lidar_link",
        point_num=2,
        lidar_id=1,
        points=(
            point_type(0, 1.25, 0.0, 0.1, 200, 1, 0),
            point_type(99_982_638, 1.0, -0.2, 0.3, 100, 2, 15),
        ),
        sequence=1,
        world_generation=1,
        simulation_session_id=SESSION,
        descriptor_sha256=descriptor.sha256,
    )

    encoded = codec.encode(cloud)

    assert encoded.type_name == "slope_sim.interfaces.v2.LidarPointCloud"
    assert codec.decode_lidar_point_cloud(encoded.payload) == cloud
