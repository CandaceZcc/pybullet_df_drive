# 企业 Protobuf 编解码：在不可变接口模型与生成消息之间做显式字段转换。
from __future__ import annotations

from struct import pack

from google.protobuf.message import DecodeError, Message

from slope_sim.interfaces.generated import slope_sim_interfaces_pb2 as pb
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPoint,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)


def _require_float32_range(value: float) -> float:
    """确认有限模型值可由 Protobuf float32 表示，避免溢出后静默变成无穷值。"""
    pack("!f", value)
    return value


def _wheel_command_to_proto(message: WheelCommand) -> pb.WheelCommand:
    """逐字段编码轮子速度命令。"""
    return pb.WheelCommand(
        timestamp_ns=message.timestamp_ns,
        drive_wheel_speed_rad_s=[
            _require_float32_range(value)
            for value in message.drive_wheel_speed_rad_s
        ],
        steering_wheel_speed_rad_s=[
            _require_float32_range(value)
            for value in message.steering_wheel_speed_rad_s
        ],
    )


def _wheel_state_to_proto(message: WheelState) -> pb.WheelState:
    """逐字段编码轮子实际状态。"""
    return pb.WheelState(
        timestamp_ns=message.timestamp_ns,
        drive_wheel_speed_rad_s=[
            _require_float32_range(value)
            for value in message.drive_wheel_speed_rad_s
        ],
        steering_wheel_angle_rad=[
            _require_float32_range(value)
            for value in message.steering_wheel_angle_rad
        ],
    )


def _lidar_point_to_proto(point: LidarPoint) -> pb.LidarPoint:
    """编码点云内嵌点，不把单点暴露为顶层消息。"""
    return pb.LidarPoint(
        offset_time_ns=point.offset_time_ns,
        x=_require_float32_range(point.x),
        y=_require_float32_range(point.y),
        z=_require_float32_range(point.z),
        reflectivity=point.reflectivity,
        tag=point.tag,
        line=point.line,
    )


def _lidar_point_cloud_to_proto(message: LidarPointCloud) -> pb.LidarPointCloud:
    """按原始顺序编码整帧点云及其全部点。"""
    return pb.LidarPointCloud(
        timebase_ns=message.timebase_ns,
        frame_id=message.frame_id,
        point_num=message.point_num,
        lidar_id=message.lidar_id,
        points=[_lidar_point_to_proto(point) for point in message.points],
    )


def _rtk_state_to_proto(message: RtkState) -> pb.RtkState:
    """逐字段编码 RTK 状态。"""
    return pb.RtkState(
        timestamp_ns=message.timestamp_ns,
        main_x=message.main_x,
        main_y=message.main_y,
        main_z=message.main_z,
        baseline_yaw_rad=message.baseline_yaw_rad,
    )


def _imu_attitude_to_proto(message: ImuAttitude) -> pb.ImuAttitude:
    """逐字段编码 IMU 姿态。"""
    return pb.ImuAttitude(
        timestamp_ns=message.timestamp_ns,
        roll_rad=message.roll_rad,
        pitch_rad=message.pitch_rad,
    )


def _parse_payload(payload: object, message: Message, message_type: str) -> None:
    """统一检查 bytes-like 输入，并把线协议解析错误转换为业务边界错误。"""
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise TypeError(f"{message_type} payload must be bytes, bytearray, or memoryview")
    try:
        message.ParseFromString(bytes(payload))
    except DecodeError as exc:
        raise ValueError(f"failed to decode {message_type} payload") from exc


_ENCODERS = {
    WheelCommand: _wheel_command_to_proto,
    WheelState: _wheel_state_to_proto,
    LidarPointCloud: _lidar_point_cloud_to_proto,
    RtkState: _rtk_state_to_proto,
    ImuAttitude: _imu_attitude_to_proto,
}

_TYPE_NAMES = {
    WheelCommand: "slope_sim.interfaces.v1.WheelCommand",
    WheelState: "slope_sim.interfaces.v1.WheelState",
    LidarPointCloud: "slope_sim.interfaces.v1.LidarPointCloud",
    RtkState: "slope_sim.interfaces.v1.RtkState",
    ImuAttitude: "slope_sim.interfaces.v1.ImuAttitude",
}


class ProtoCodec:
    """显式编解码五种可独立传输的企业接口消息。"""

    def encode(self, message: object) -> bytes:
        """把受支持的不可变模型序列化为 Protobuf 字节串。"""
        encoder = _ENCODERS.get(type(message))
        if encoder is None:
            raise TypeError(f"unsupported enterprise message: {type(message).__name__}")
        message_type = _TYPE_NAMES[type(message)]
        try:
            proto_message = encoder(message)
        except (ValueError, OverflowError) as exc:
            raise ValueError(f"failed to encode {message_type}: {exc}") from exc
        return proto_message.SerializeToString()

    def type_name(self, message: object) -> str:
        """返回受支持消息的完整版本化 Protobuf 类型名。"""
        name = _TYPE_NAMES.get(type(message))
        if name is None:
            raise TypeError(f"unsupported enterprise message: {type(message).__name__}")
        return name

    def decode_wheel_command(self, payload: object) -> WheelCommand:
        """解析轮子命令；车型数组长度留给独立命令校验器。"""
        message = pb.WheelCommand()
        _parse_payload(payload, message, "WheelCommand")
        return WheelCommand(
            timestamp_ns=message.timestamp_ns,
            drive_wheel_speed_rad_s=tuple(message.drive_wheel_speed_rad_s),
            steering_wheel_speed_rad_s=tuple(message.steering_wheel_speed_rad_s),
        )

    def decode_wheel_state(self, payload: object) -> WheelState:
        """解析轮子状态并通过模型构造器复验数值。"""
        message = pb.WheelState()
        _parse_payload(payload, message, "WheelState")
        return WheelState(
            timestamp_ns=message.timestamp_ns,
            drive_wheel_speed_rad_s=tuple(message.drive_wheel_speed_rad_s),
            steering_wheel_angle_rad=tuple(message.steering_wheel_angle_rad),
        )

    def decode_lidar_point_cloud(self, payload: object) -> LidarPointCloud:
        """解析有序点云，并由模型复验点数、坐标及点语义边界。"""
        message = pb.LidarPointCloud()
        _parse_payload(payload, message, "LidarPointCloud")
        points = tuple(
            LidarPoint(
                offset_time_ns=point.offset_time_ns,
                x=point.x,
                y=point.y,
                z=point.z,
                reflectivity=point.reflectivity,
                tag=point.tag,
                line=point.line,
            )
            for point in message.points
        )
        return LidarPointCloud(
            timebase_ns=message.timebase_ns,
            frame_id=message.frame_id,
            point_num=message.point_num,
            lidar_id=message.lidar_id,
            points=points,
        )

    def decode_rtk_state(self, payload: object) -> RtkState:
        """解析 RTK 状态并通过模型构造器拒绝非有限值。"""
        message = pb.RtkState()
        _parse_payload(payload, message, "RtkState")
        return RtkState(
            timestamp_ns=message.timestamp_ns,
            main_x=message.main_x,
            main_y=message.main_y,
            main_z=message.main_z,
            baseline_yaw_rad=message.baseline_yaw_rad,
        )

    def decode_imu_attitude(self, payload: object) -> ImuAttitude:
        """解析 IMU 姿态并通过模型构造器拒绝非有限值。"""
        message = pb.ImuAttitude()
        _parse_payload(payload, message, "ImuAttitude")
        return ImuAttitude(
            timestamp_ns=message.timestamp_ns,
            roll_rad=message.roll_rad,
            pitch_rad=message.pitch_rad,
        )
