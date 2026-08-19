"""阶段四 Protobuf codec：确定性序列化一次并保留原始 payload hash。"""
from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256

from google.protobuf.message import DecodeError, Message

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.models import (
    CommandAuthorityState,
    ImuAttitudeV2,
    LidarPointCloudV2,
    LidarPointV2,
    Point3dV2,
    RtkStateV2,
    WheelCommandV2,
    WheelStateV2,
)


@dataclass(frozen=True)
class EncodedV2Frame:
    """用于同一日志和 raw transport 的唯一编码 bytes。"""

    type_name: str
    payload: bytes
    payload_sha256: bytes


class V2ProtoCodec:
    """显式映射 v2 wheel 模型，避免反射与二次序列化。"""

    def __init__(self, descriptor: DescriptorIdentity) -> None:
        self._descriptor = descriptor

    def encode(self, model: WheelCommandV2 | WheelStateV2 | ImuAttitudeV2 | RtkStateV2 | LidarPointCloudV2) -> EncodedV2Frame:
        """确定性编码一份 v2 wheel frame，并返回原始 payload 摘要。"""
        if isinstance(model, WheelCommandV2):
            message = pb.WheelCommand(timestamp_ns=model.timestamp_ns, drive_wheel_speed_rad_s=model.drive_wheel_speed_rad_s, steering_wheel_speed_rad_s=model.steering_wheel_speed_rad_s, sequence=model.sequence, world_generation=model.world_generation, command_generation=model.command_generation, source_id=model.source_id, source_session_id=model.source_session_id, robot_model=model.robot_model, simulation_session_id=model.simulation_session_id, descriptor_sha256=model.descriptor_sha256)
        elif isinstance(model, WheelStateV2):
            message = pb.WheelState(timestamp_ns=model.timestamp_ns, drive_wheel_speed_rad_s=model.drive_wheel_speed_rad_s, steering_wheel_angle_rad=model.steering_wheel_angle_rad, sequence=model.sequence, world_generation=model.world_generation, command_generation=model.command_generation, robot_model=model.robot_model, simulation_session_id=model.simulation_session_id, descriptor_sha256=model.descriptor_sha256, command_authority_state=int(model.command_authority_state), command_owner_source_id=model.command_owner_source_id, command_owner_source_session_id=model.command_owner_source_session_id, command_peer_count=model.command_peer_count)
        elif isinstance(model, ImuAttitudeV2):
            message = pb.ImuAttitude(timestamp_ns=model.timestamp_ns, roll_rad=model.roll_rad, pitch_rad=model.pitch_rad, sequence=model.sequence, world_generation=model.world_generation, frame_id=model.frame_id, simulation_session_id=model.simulation_session_id, descriptor_sha256=model.descriptor_sha256)
        elif isinstance(model, RtkStateV2):
            message = pb.RtkState(
                timestamp_ns=model.timestamp_ns,
                sequence=model.sequence,
                world_generation=model.world_generation,
                frame_id=model.frame_id,
                left=pb.Point3d(x_m=model.left.x_m, y_m=model.left.y_m, z_m=model.left.z_m),
                center=pb.Point3d(x_m=model.center.x_m, y_m=model.center.y_m, z_m=model.center.z_m),
                right=pb.Point3d(x_m=model.right.x_m, y_m=model.right.y_m, z_m=model.right.z_m),
                heading_rad=model.heading_rad,
                simulation_session_id=model.simulation_session_id,
                descriptor_sha256=model.descriptor_sha256,
            )
        elif isinstance(model, LidarPointCloudV2):
            message = pb.LidarPointCloud(
                timebase_ns=model.timebase_ns,
                frame_id=model.frame_id,
                point_num=model.point_num,
                lidar_id=model.lidar_id,
                points=[
                    pb.LidarPoint(
                        offset_time_ns=point.offset_time_ns,
                        x=point.x,
                        y=point.y,
                        z=point.z,
                        reflectivity=point.reflectivity,
                        tag=point.tag,
                        line=point.line,
                    )
                    for point in model.points
                ],
                sequence=model.sequence,
                world_generation=model.world_generation,
                simulation_session_id=model.simulation_session_id,
                descriptor_sha256=model.descriptor_sha256,
            )
        else:
            raise TypeError(f"unsupported v2 model: {type(model).__name__}")
        if model.descriptor_sha256 != self._descriptor.sha256:
            raise ValueError("v2 descriptor SHA-256 mismatch")
        payload = message.SerializeToString(deterministic=True)
        return EncodedV2Frame(message.DESCRIPTOR.full_name, payload, sha256(payload).digest())

    def _parse(self, payload: object, message: Message) -> Message:
        """先校验带内 descriptor 和会话长度，才允许构造领域模型。"""
        if not isinstance(payload, (bytes, bytearray, memoryview)):
            raise TypeError("v2 payload must be bytes-like")
        try:
            message.ParseFromString(bytes(payload))
        except DecodeError as error:
            raise ValueError("failed to decode v2 payload") from error
        if bytes(message.descriptor_sha256) != self._descriptor.sha256:
            raise ValueError("v2 descriptor SHA-256 mismatch")
        if len(message.simulation_session_id) != 16:
            raise ValueError("simulation_session_id must be exactly 16 bytes")
        return message

    def decode_wheel_command(self, payload: object) -> WheelCommandV2:
        """显式解析 wheel command，身份失败早于后续命令处理。"""
        message = self._parse(payload, pb.WheelCommand())
        return WheelCommandV2(message.timestamp_ns, tuple(message.drive_wheel_speed_rad_s), tuple(message.steering_wheel_speed_rad_s), message.sequence, message.world_generation, message.command_generation, message.source_id, bytes(message.source_session_id), message.robot_model, bytes(message.simulation_session_id), bytes(message.descriptor_sha256))

    def decode_wheel_state(self, payload: object) -> WheelStateV2:
        """显式解析 wheel state，并拒绝未知或零命令权 enum。"""
        message = self._parse(payload, pb.WheelState())
        try:
            state = CommandAuthorityState(message.command_authority_state)
        except ValueError as error:
            raise ValueError("invalid command_authority_state") from error
        return WheelStateV2(message.timestamp_ns, tuple(message.drive_wheel_speed_rad_s), tuple(message.steering_wheel_angle_rad), message.sequence, message.world_generation, message.command_generation, message.robot_model, bytes(message.simulation_session_id), bytes(message.descriptor_sha256), state, message.command_owner_source_id, bytes(message.command_owner_source_session_id), message.command_peer_count)

    def decode_imu_attitude(self, payload: object) -> ImuAttitudeV2:
        """显式解析 v2 IMU，身份验证仍早于模型构造。"""
        message = self._parse(payload, pb.ImuAttitude())
        return ImuAttitudeV2(message.timestamp_ns, message.roll_rad, message.pitch_rad, message.sequence, message.world_generation, message.frame_id, bytes(message.simulation_session_id), bytes(message.descriptor_sha256))

    def decode_rtk_state(self, payload: object) -> RtkStateV2:
        """显式解析三点 RTK，保留同帧的会话与场景身份边界。"""
        message = self._parse(payload, pb.RtkState())
        return RtkStateV2(
            message.timestamp_ns,
            message.sequence,
            message.world_generation,
            message.frame_id,
            Point3dV2(message.left.x_m, message.left.y_m, message.left.z_m),
            Point3dV2(message.center.x_m, message.center.y_m, message.center.z_m),
            Point3dV2(message.right.x_m, message.right.y_m, message.right.z_m),
            message.heading_rad,
            bytes(message.simulation_session_id),
            bytes(message.descriptor_sha256),
        )

    def decode_lidar_point_cloud(self, payload: object) -> LidarPointCloudV2:
        """显式解析中心 LiDAR，点顺序和帧身份均来自单一 payload。"""
        message = self._parse(payload, pb.LidarPointCloud())
        return LidarPointCloudV2(
            message.timebase_ns,
            message.frame_id,
            message.point_num,
            message.lidar_id,
            tuple(
                LidarPointV2(
                    point.offset_time_ns,
                    point.x,
                    point.y,
                    point.z,
                    point.reflectivity,
                    point.tag,
                    point.line,
                )
                for point in message.points
            ),
            message.sequence,
            message.world_generation,
            bytes(message.simulation_session_id),
            bytes(message.descriptor_sha256),
        )
