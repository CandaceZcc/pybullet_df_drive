# 机器人模块：加载阶段一轮式底盘，并提供差速/主动转向控制和真实状态反馈。
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import pybullet as p

from slope_sim.controller import wheel_speeds_from_twist
from slope_sim.model_registry import RobotModelSpec, get_robot_model
from slope_sim.sensors import LidarSummary
from slope_sim.telemetry import RobotTelemetry, TerrainProbe, body_frame_velocity, slip_is_valid, slip_ratio, track_body_speeds


CONTACT_FORCE_EPSILON = 1e-6


def _require_finite_values(name: str, values: tuple[float, ...]) -> None:
    """在调用 PyBullet 前拒绝非有限命令，避免污染物理状态。"""
    for index, value in enumerate(values):
        try:
            finite = math.isfinite(float(value))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name}[{index}] must be finite") from exc
        if not finite:
            raise ValueError(f"{name}[{index}] must be finite")


def _require_positive_finite(name: str, value: float) -> None:
    """校验物理步长等必须为正的有限标量。"""
    _require_finite_values(name, (value,))
    if value <= 0:
        raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class DriveJoint:
    """单个驱动接触轮的关节索引和有效半径。"""

    joint_index: int
    radius: float


@dataclass(frozen=True)
class ContactFeedback:
    """左右驱动侧接触反馈，包含法向力、摩擦力和有效接触点数量。"""

    left_normal_force: float = 0.0
    right_normal_force: float = 0.0
    left_friction_force: float = 0.0
    right_friction_force: float = 0.0
    left_count: int = 0
    right_count: int = 0

    @property
    def total_count(self) -> int:
        return self.left_count + self.right_count


@dataclass(frozen=True)
class RobotState(RobotTelemetry):
    """兼容旧代码的机器人状态类型，字段来自统一遥测结构。"""


class DifferentialDriveRobot:
    """差速底盘封装，隐藏 URDF 加载、轮关节查找和位姿更新细节。"""

    def __init__(
        self,
        client_id: int,
        urdf_path: str | Path,
        wheel_base: float,
        wheel_radius: float,
        start_x: float = 0.0,
        start_y: float = 0.0,
        base_height: float = 0.18,
        start_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        drive_motor_force: float = 5.0,
        model_spec: RobotModelSpec | None = None,
    ) -> None:
        """加载机器人模型，并初始化运动学状态。"""
        self.client_id = client_id
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.base_height = base_height
        self.drive_motor_force = drive_motor_force
        self.model_spec = model_spec
        self.x = start_x
        self.y = start_y
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0
        self._previous_velocity_time: float | None = None
        self._previous_linear_world: tuple[float, float, float] | None = None
        self._previous_angular_world: tuple[float, float, float] | None = None
        # 当前 URDF 轮轴正方向和“车头朝 +x 前进”的项目约定一致；后续换模型时可集中调整。
        self.physics_drive_sign = 1.0
        # PyBullet 根据 URDF 创建多刚体模型，后续用返回的 id 操作机器人。
        self.robot_id = p.loadURDF(
            str(urdf_path),
            basePosition=[start_x, start_y, base_height],
            baseOrientation=start_orientation,
            physicsClientId=client_id,
        )
        self.joint_name_to_index = self._collect_joint_indices()
        self.left_joint, self.right_joint = self._find_wheel_joints()
        self.drive_center_x = self._drive_center_x()
        self.left_drive_joints, self.right_drive_joints = self._build_drive_joints()
        if self.model_spec is not None:
            self.drive_wheel_joint_indices = tuple(
                self.joint_name_to_index[name] for name in self.model_spec.drive_joint_names
            )
        else:
            self.drive_wheel_joint_indices = tuple(
                drive.joint_index for drive in self.left_drive_joints + self.right_drive_joints
            )
        self.left_contact_links, self.right_contact_links = self._find_drive_side_links()
        self.support_links = self._find_support_links()
        self._disable_passive_drive_joints()

    def _collect_joint_indices(self) -> dict[str, int]:
        """建立关节名到索引的映射，控制器只依赖语义名称而不硬编码索引。"""
        joints: dict[str, int] = {}
        for joint_index in range(p.getNumJoints(self.robot_id, physicsClientId=self.client_id)):
            info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self.client_id)
            joints[info[1].decode("utf-8")] = joint_index
        return joints

    def _find_wheel_joints(self) -> tuple[int, int]:
        """从 URDF 里找到左右轮关节，后续才能分别设置轮速。"""
        if self.model_spec is not None:
            left_name = next(name for name in self.model_spec.drive_joint_names if "left" in name)
            right_name = next(name for name in self.model_spec.drive_joint_names if "right" in name)
            return self.joint_name_to_index[left_name], self.joint_name_to_index[right_name]
        left_name = "left_drive_wheel_joint" if "left_drive_wheel_joint" in self.joint_name_to_index else "left_wheel_joint"
        right_name = "right_drive_wheel_joint" if "right_drive_wheel_joint" in self.joint_name_to_index else "right_wheel_joint"
        return self.joint_name_to_index[left_name], self.joint_name_to_index[right_name]

    def _build_drive_joints(self) -> tuple[list[DriveJoint], list[DriveJoint]]:
        """按注册表语义名称建立左右驱动组。"""
        if self.model_spec is not None:
            left = [
                DriveJoint(self.joint_name_to_index[name], self.wheel_radius)
                for name in self.model_spec.drive_joint_names
                if "left" in name
            ]
            right = [
                DriveJoint(self.joint_name_to_index[name], self.wheel_radius)
                for name in self.model_spec.drive_joint_names
                if "right" in name
            ]
            return left, right
        left = [DriveJoint(self.left_joint, self.wheel_radius)]
        right = [DriveJoint(self.right_joint, self.wheel_radius)]
        return left, right

    def _drive_center_x(self) -> float:
        """读取左右主驱动轮在车体坐标下的 x 位置，侧滑估计用它修正参考点。"""
        if self.model_spec is not None:
            return self.model_spec.drive_center_x
        left_info = p.getJointInfo(self.robot_id, self.left_joint, physicsClientId=self.client_id)
        right_info = p.getJointInfo(self.robot_id, self.right_joint, physicsClientId=self.client_id)
        return (float(left_info[14][0]) + float(right_info[14][0])) / 2.0

    def _find_drive_side_links(self) -> tuple[set[int], set[int]]:
        """按 link 名称归类左右接触件，用于汇总法向接触力。"""
        left_links: set[int] = set()
        right_links: set[int] = set()
        for joint_index in range(p.getNumJoints(self.robot_id, physicsClientId=self.client_id)):
            info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self.client_id)
            link_name = info[12].decode("utf-8").lower()
            if "left" in link_name:
                left_links.add(joint_index)
            if "right" in link_name:
                right_links.add(joint_index)
        left_links.add(self.left_joint)
        right_links.add(self.right_joint)
        return left_links, right_links

    def _find_support_links(self) -> set[int]:
        """查找支撑轮/万向轮 link，便于和驱动轮使用不同摩擦。"""
        support_links: set[int] = set()
        for joint_index in range(p.getNumJoints(self.robot_id, physicsClientId=self.client_id)):
            info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self.client_id)
            link_name = info[12].decode("utf-8").lower()
            if "caster" in link_name or "support" in link_name:
                support_links.add(joint_index)
        return support_links

    def _disable_passive_drive_joints(self) -> None:
        """关闭非驱动转动关节的默认电机，交由具体控制器接管。"""
        drive_joint_indices = {drive.joint_index for drive in self.left_drive_joints + self.right_drive_joints}
        for joint_index in range(p.getNumJoints(self.robot_id, physicsClientId=self.client_id)):
            if joint_index in drive_joint_indices:
                continue
            info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self.client_id)
            continuous_joint = getattr(p, "JOINT_CONTINUOUS", p.JOINT_REVOLUTE)
            if info[2] in {p.JOINT_REVOLUTE, continuous_joint}:
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_index,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=0.0,
                    force=0.0,
                    physicsClientId=self.client_id,
                )

    def apply_drive_friction(self, lateral_friction: float, support_lateral_friction: float = 0.03) -> None:
        """给所有左右驱动接触件设置摩擦，便于做坡面打滑实验。"""
        for link_index in self.left_contact_links | self.right_contact_links:
            kwargs = {
                "lateralFriction": lateral_friction,
                "rollingFriction": 0.02,
                "spinningFriction": 0.02,
                "physicsClientId": self.client_id,
            }
            p.changeDynamics(self.robot_id, link_index, **kwargs)
        for link_index in self.support_links:
            # 支撑轮只负责托住车体，不提供牵引；低摩擦能减少原地转向时的拖拽。
            p.changeDynamics(
                self.robot_id,
                link_index,
                lateralFriction=support_lateral_friction,
                rollingFriction=0.0,
                spinningFriction=0.0,
                physicsClientId=self.client_id,
            )

    def command_wheel_speeds(
        self,
        drive_wheel_speeds: tuple[float, ...],
        steering_wheel_speeds: tuple[float, ...] = (),
        dt: float = 1.0 / 240.0,
    ) -> tuple[float, ...]:
        """直接下发两个差速驱动轮角速度；差速车型不接受转向轮命令。"""
        if len(drive_wheel_speeds) != 2:
            raise ValueError("differential robot requires two drive wheel speeds")
        if steering_wheel_speeds:
            raise ValueError("differential robot does not accept steering wheel speeds")
        _require_finite_values("drive_wheel_speeds", drive_wheel_speeds)
        _require_positive_finite("dt", dt)
        self.left_wheel_speed = float(drive_wheel_speeds[0])
        self.right_wheel_speed = float(drive_wheel_speeds[1])
        for drive, speed in zip(self.left_drive_joints + self.right_drive_joints, drive_wheel_speeds):
            p.setJointMotorControl2(
                self.robot_id,
                drive.joint_index,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=self.physics_drive_sign * float(speed),
                force=self.drive_motor_force,
                physicsClientId=self.client_id,
            )
        return tuple(float(speed) for speed in drive_wheel_speeds)

    def command_twist(
        self,
        linear_velocity: float,
        angular_velocity: float,
        dt: float = 1.0 / 240.0,
    ) -> tuple[float, float]:
        """接收车体速度 v/w，并转换成左右轮的目标角速度。"""
        _require_finite_values("twist", (linear_velocity, angular_velocity))
        _require_positive_finite("dt", dt)
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.left_wheel_speed, self.right_wheel_speed = wheel_speeds_from_twist(
            linear_velocity,
            angular_velocity,
            self.wheel_base,
            self.wheel_radius,
        )
        self.command_wheel_speeds((self.left_wheel_speed, self.right_wheel_speed), dt=dt)
        return self.left_wheel_speed, self.right_wheel_speed

    def read_drive_wheel_speeds(self) -> tuple[float, ...]:
        """按注册表顺序读取 PyBullet 实际驱动轮角速度。"""
        if self.model_spec is not None:
            joint_names = self.model_spec.drive_joint_names
        else:
            joint_names = tuple(
                name
                for name in ("left_drive_wheel_joint", "right_drive_wheel_joint", "left_wheel_joint", "right_wheel_joint")
                if name in self.joint_name_to_index
            )[:2]
        return tuple(
            self.physics_drive_sign
            * float(p.getJointState(self.robot_id, self.joint_name_to_index[name], physicsClientId=self.client_id)[1])
            for name in joint_names
        )

    def read_steering_wheel_angles(self) -> tuple[float, ...]:
        """差速车型没有独立转向轮。"""
        return ()

    def step_kinematic(self, dt: float, slope_deg: float, t: float) -> RobotState:
        """用差速车运动学更新位姿，保证 DIRECT 批量实验稳定可重复。"""
        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt
        self.yaw = _wrap_angle(self.yaw + self.angular_velocity * dt)
        return self.reset_pose_on_slope(slope_deg=slope_deg, t=t)

    def reset_pose_on_slope(self, slope_deg: float, t: float) -> RobotState:
        """把机器人放回斜坡表面，并记录当前姿态状态。"""
        slope_rad = math.radians(slope_deg)
        z = self.base_height + self.x * math.tan(slope_rad)
        roll = 0.0
        pitch = -slope_rad
        quat = p.getQuaternionFromEuler([roll, pitch, self.yaw])
        left_body_track_speed, right_body_track_speed = track_body_speeds(
            self.linear_velocity,
            self.angular_velocity,
            self.wheel_base,
        )
        left_track_surface_speed = self.left_wheel_speed * self.wheel_radius
        right_track_surface_speed = self.right_wheel_speed * self.wheel_radius
        left_slip_speed = left_track_surface_speed - left_body_track_speed
        right_slip_speed = right_track_surface_speed - right_body_track_speed
        left_slip_valid = slip_is_valid(left_track_surface_speed, left_body_track_speed)
        right_slip_valid = slip_is_valid(right_track_surface_speed, right_body_track_speed)
        # 将运动学算出的位姿同步回 PyBullet，可让 GUI 和日志看到同一状态。
        p.resetBasePositionAndOrientation(
            self.robot_id,
            [self.x, self.y, z],
            quat,
            physicsClientId=self.client_id,
        )
        return RobotState(
            t=t,
            x=self.x,
            y=self.y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=self.yaw,
            qx=quat[0],
            qy=quat[1],
            qz=quat[2],
            qw=quat[3],
            vx=self.linear_velocity * math.cos(self.yaw),
            vy=self.linear_velocity * math.sin(self.yaw),
            vz=0.0,
            body_forward_speed=self.linear_velocity,
            body_lateral_speed=0.0,
            body_vertical_speed=0.0,
            angular_velocity_z=self.angular_velocity,
            yaw_rate=self.angular_velocity,
            velocity_sensor_vx=self.linear_velocity * math.cos(self.yaw),
            velocity_sensor_vy=self.linear_velocity * math.sin(self.yaw),
            velocity_sensor_vz=0.0,
            velocity_sensor_body_forward_speed=self.linear_velocity,
            velocity_sensor_yaw_rate=self.angular_velocity,
            linear_velocity=self.linear_velocity,
            angular_velocity=self.angular_velocity,
            command_linear_velocity=self.linear_velocity,
            command_angular_velocity=self.angular_velocity,
            left_target_drive_speed=self.left_wheel_speed,
            right_target_drive_speed=self.right_wheel_speed,
            left_actual_drive_speed=self.left_wheel_speed,
            right_actual_drive_speed=self.right_wheel_speed,
            left_track_surface_speed=left_track_surface_speed,
            right_track_surface_speed=right_track_surface_speed,
            left_body_track_speed=left_body_track_speed,
            right_body_track_speed=right_body_track_speed,
            left_slip_speed=left_slip_speed,
            right_slip_speed=right_slip_speed,
            left_slip_valid=left_slip_valid,
            right_slip_valid=right_slip_valid,
            left_slip_ratio=slip_ratio(left_track_surface_speed, left_body_track_speed),
            right_slip_ratio=slip_ratio(right_track_surface_speed, right_body_track_speed),
        )

    def read_physics_state(
        self,
        t: float,
        command_linear_velocity: float,
        command_angular_velocity: float,
        ground_lateral_friction: float,
        drive_lateral_friction: float,
        ground_rolling_friction: float = 0.02,
        ground_spinning_friction: float = 0.0,
        support_lateral_friction: float = 0.03,
        robot_model: str = "df_back",
        terrain_type: str = "flat",
        terrain_probe: TerrainProbe | None = None,
        lidar_summary: LidarSummary | None = None,
    ) -> RobotState:
        """从 PyBullet 真实物理状态读取完整遥测。"""
        position, orientation = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.client_id)
        roll, pitch, yaw = p.getEulerFromQuaternion(orientation)
        linear_world, angular_world = p.getBaseVelocity(self.robot_id, physicsClientId=self.client_id)
        body_forward, body_lateral, body_vertical = body_frame_velocity(
            (linear_world[0], linear_world[1], linear_world[2]),
            yaw,
        )
        # 驱动轴不在 base 原点时，转弯会让 base 原点产生横向速度；这里修正到驱动轴中心。
        body_lateral += float(angular_world[2]) * self.drive_center_x
        linear_acceleration, angular_acceleration_z = self._acceleration_from_velocity(t, linear_world, angular_world)

        left_joint_state = p.getJointState(self.robot_id, self.left_joint, physicsClientId=self.client_id)
        right_joint_state = p.getJointState(self.robot_id, self.right_joint, physicsClientId=self.client_id)
        actual_drive_speeds = self.read_drive_wheel_speeds()
        actual_steering_angles = self.read_steering_wheel_angles()
        if len(actual_drive_speeds) == 4:
            front_left_speed, front_right_speed, rear_left_speed, rear_right_speed = actual_drive_speeds
        else:
            front_left_speed = front_right_speed = rear_left_speed = rear_right_speed = math.nan
        if len(actual_steering_angles) == 2:
            front_left_steering, front_right_steering = actual_steering_angles
        else:
            front_left_steering = front_right_steering = math.nan
        # 日志里仍按项目约定记录：正轮速表示车体向前。
        left_position = self.physics_drive_sign * float(left_joint_state[0])
        left_actual_speed = self.physics_drive_sign * float(left_joint_state[1])
        right_position = self.physics_drive_sign * float(right_joint_state[0])
        right_actual_speed = self.physics_drive_sign * float(right_joint_state[1])
        left_torque = float(left_joint_state[3])
        right_torque = float(right_joint_state[3])

        contact_feedback = self._drive_contact_feedback()
        left_track_surface_speed = self._average_track_surface_speed(self.left_drive_joints)
        right_track_surface_speed = self._average_track_surface_speed(self.right_drive_joints)
        left_body_track_speed, right_body_track_speed = track_body_speeds(body_forward, float(angular_world[2]), self.wheel_base)
        left_slip_speed = left_track_surface_speed - left_body_track_speed
        right_slip_speed = right_track_surface_speed - right_body_track_speed
        left_slip_valid = slip_is_valid(left_track_surface_speed, left_body_track_speed)
        right_slip_valid = slip_is_valid(right_track_surface_speed, right_body_track_speed)
        lidar = lidar_summary or LidarSummary()
        terrain = terrain_probe or TerrainProbe()
        anisotropic = (0.0, 0.0, 0.0)

        return RobotState(
            t=t,
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            qx=float(orientation[0]),
            qy=float(orientation[1]),
            qz=float(orientation[2]),
            qw=float(orientation[3]),
            vx=float(linear_world[0]),
            vy=float(linear_world[1]),
            vz=float(linear_world[2]),
            body_forward_speed=body_forward,
            body_lateral_speed=body_lateral,
            body_vertical_speed=body_vertical,
            angular_velocity_x=float(angular_world[0]),
            angular_velocity_y=float(angular_world[1]),
            angular_velocity_z=float(angular_world[2]),
            yaw_rate=float(angular_world[2]),
            velocity_sensor_vx=float(linear_world[0]),
            velocity_sensor_vy=float(linear_world[1]),
            velocity_sensor_vz=float(linear_world[2]),
            velocity_sensor_body_forward_speed=body_forward,
            velocity_sensor_yaw_rate=float(angular_world[2]),
            linear_acceleration_x=linear_acceleration[0],
            linear_acceleration_y=linear_acceleration[1],
            linear_acceleration_z=linear_acceleration[2],
            angular_acceleration_z=angular_acceleration_z,
            linear_velocity=body_forward,
            angular_velocity=float(angular_world[2]),
            command_linear_velocity=command_linear_velocity,
            command_angular_velocity=command_angular_velocity,
            left_target_drive_speed=self.left_wheel_speed,
            right_target_drive_speed=self.right_wheel_speed,
            left_actual_drive_speed=left_actual_speed,
            right_actual_drive_speed=right_actual_speed,
            front_left_actual_drive_speed=front_left_speed,
            front_right_actual_drive_speed=front_right_speed,
            rear_left_actual_drive_speed=rear_left_speed,
            rear_right_actual_drive_speed=rear_right_speed,
            front_left_actual_steering_angle=front_left_steering,
            front_right_actual_steering_angle=front_right_steering,
            left_track_surface_speed=left_track_surface_speed,
            right_track_surface_speed=right_track_surface_speed,
            left_body_track_speed=left_body_track_speed,
            right_body_track_speed=right_body_track_speed,
            left_drive_position=left_position,
            right_drive_position=right_position,
            left_motor_torque=left_torque,
            right_motor_torque=right_torque,
            ground_lateral_friction=ground_lateral_friction,
            ground_rolling_friction=ground_rolling_friction,
            ground_spinning_friction=ground_spinning_friction,
            drive_lateral_friction=drive_lateral_friction,
            support_lateral_friction=support_lateral_friction,
            drive_motor_force=self.drive_motor_force,
            track_anisotropic_friction_x=anisotropic[0],
            track_anisotropic_friction_y=anisotropic[1],
            track_anisotropic_friction_z=anisotropic[2],
            left_contact_normal_force=contact_feedback.left_normal_force,
            right_contact_normal_force=contact_feedback.right_normal_force,
            left_contact_friction_force=contact_feedback.left_friction_force,
            right_contact_friction_force=contact_feedback.right_friction_force,
            left_contact_count=contact_feedback.left_count,
            right_contact_count=contact_feedback.right_count,
            contact_count=contact_feedback.total_count,
            left_slip_speed=left_slip_speed,
            right_slip_speed=right_slip_speed,
            left_slip_valid=left_slip_valid,
            right_slip_valid=right_slip_valid,
            left_slip_ratio=slip_ratio(left_track_surface_speed, left_body_track_speed),
            right_slip_ratio=slip_ratio(right_track_surface_speed, right_body_track_speed),
            body_lateral_slip_speed=body_lateral,
            robot_model=robot_model,
            terrain_type=terrain_type,
            terrain_probe_valid=terrain.terrain_probe_valid,
            out_of_bounds=terrain.out_of_bounds,
            local_ground_height=terrain.local_ground_height,
            local_terrain_normal_x=terrain.local_terrain_normal_x,
            local_terrain_normal_y=terrain.local_terrain_normal_y,
            local_terrain_normal_z=terrain.local_terrain_normal_z,
            lidar_min_distance=lidar.min_distance,
            lidar_front_distance=lidar.front_distance,
            lidar_left_distance=lidar.left_distance,
            lidar_right_distance=lidar.right_distance,
            nearest_obstacle_distance=lidar.min_distance,
            bumper_contact=contact_feedback.total_count > 0,
        )

    def _acceleration_from_velocity(
        self,
        t: float,
        linear_world: tuple[float, float, float],
        angular_world: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], float]:
        """用相邻物理帧速度差分估计理想加速度传感读数。"""
        current_linear = (float(linear_world[0]), float(linear_world[1]), float(linear_world[2]))
        current_angular = (float(angular_world[0]), float(angular_world[1]), float(angular_world[2]))
        acceleration = (0.0, 0.0, 0.0)
        angular_acceleration_z = 0.0
        if self._previous_velocity_time is not None and self._previous_linear_world is not None and self._previous_angular_world is not None:
            dt = t - self._previous_velocity_time
            if dt > 0.0:
                acceleration = tuple((current_linear[index] - self._previous_linear_world[index]) / dt for index in range(3))
                angular_acceleration_z = (current_angular[2] - self._previous_angular_world[2]) / dt
        self._previous_velocity_time = t
        self._previous_linear_world = current_linear
        self._previous_angular_world = current_angular
        return acceleration, angular_acceleration_z

    def _average_track_surface_speed(self, drive_joints: list[DriveJoint]) -> float:
        """读取同侧所有驱动接触轮，并估算履带平均接地线速度。"""
        surface_speeds = []
        for drive in drive_joints:
            joint_state = p.getJointState(self.robot_id, drive.joint_index, physicsClientId=self.client_id)
            surface_speeds.append(self.physics_drive_sign * float(joint_state[1]) * drive.radius)
        return sum(surface_speeds) / len(surface_speeds)

    def _drive_contact_feedback(self) -> ContactFeedback:
        """汇总左右驱动接触件的法向力、摩擦力和有效接触点数量。"""
        left_normal_force = 0.0
        right_normal_force = 0.0
        left_friction_force = 0.0
        right_friction_force = 0.0
        left_count = 0
        right_count = 0
        contacts = p.getContactPoints(bodyA=self.robot_id, physicsClientId=self.client_id)
        for contact in contacts:
            link_index = contact[3]
            normal_force = float(contact[9]) if len(contact) > 9 else 0.0
            if normal_force <= CONTACT_FORCE_EPSILON:
                continue
            # PyBullet 接触元组 10/11 和 12/13 分别是两条切向摩擦力及方向。
            friction_force = _contact_friction_magnitude(contact)
            if link_index in self.left_contact_links:
                left_normal_force += normal_force
                left_friction_force += friction_force
                left_count += 1
            if link_index in self.right_contact_links:
                right_normal_force += normal_force
                right_friction_force += friction_force
                right_count += 1
        return ContactFeedback(
            left_normal_force=left_normal_force,
            right_normal_force=right_normal_force,
            left_friction_force=left_friction_force,
            right_friction_force=right_friction_force,
            left_count=left_count,
            right_count=right_count,
        )


def _contact_friction_magnitude(contact: tuple[object, ...]) -> float:
    """把 PyBullet 两个切向摩擦分量合成为一个力的模长。"""
    if len(contact) <= 13:
        return 0.0
    friction_1 = float(contact[10])
    direction_1 = contact[11]
    friction_2 = float(contact[12])
    direction_2 = contact[13]
    force_x = friction_1 * float(direction_1[0]) + friction_2 * float(direction_2[0])
    force_y = friction_1 * float(direction_1[1]) + friction_2 * float(direction_2[1])
    force_z = friction_1 * float(direction_1[2]) + friction_2 * float(direction_2[2])
    return math.sqrt(force_x * force_x + force_y * force_y + force_z * force_z)


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ActiveSteeringRobot(DifferentialDriveRobot):
    """四轮独立驱动、前轮主动转向模型，借鉴 Bullet racecar 的关节层级。"""

    MAX_STEERING_RATE = 2.0

    def __init__(self, *args, model_spec: RobotModelSpec, **kwargs) -> None:
        super().__init__(*args, model_spec=model_spec, **kwargs)
        self.steering_joint_indices = tuple(self.joint_name_to_index[name] for name in model_spec.steering_joint_names)
        self.max_steering_angle = model_spec.max_steering_angle
        self._steering_targets = [0.0, 0.0]
        for joint_index in self.steering_joint_indices:
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.0,
                force=self.drive_motor_force,
                physicsClientId=self.client_id,
            )

    def command_wheel_speeds(
        self,
        drive_wheel_speeds: tuple[float, ...],
        steering_wheel_speeds: tuple[float, ...] = (),
        dt: float = 1.0 / 240.0,
    ) -> tuple[float, ...]:
        """积分两个前轮转向速度，并给四个驱动轮设置独立速度目标。"""
        if len(drive_wheel_speeds) != 4:
            raise ValueError("active steering robot requires four drive wheel speeds")
        if len(steering_wheel_speeds) != 2:
            raise ValueError("active steering robot requires two steering wheel speeds")
        _require_finite_values("drive_wheel_speeds", drive_wheel_speeds)
        _require_finite_values("steering_wheel_speeds", steering_wheel_speeds)
        _require_positive_finite("dt", dt)
        self.left_wheel_speed = float(drive_wheel_speeds[0])
        self.right_wheel_speed = float(drive_wheel_speeds[1])
        self._steering_targets = [
            max(-self.max_steering_angle, min(self.max_steering_angle, target + float(rate) * dt))
            for target, rate in zip(self._steering_targets, steering_wheel_speeds)
        ]
        for joint_index, target in zip(self.steering_joint_indices, self._steering_targets):
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target,
                force=self.drive_motor_force,
                physicsClientId=self.client_id,
            )
        for joint_index, speed in zip(self.drive_wheel_joint_indices, drive_wheel_speeds):
            p.setJointMotorControl2(
                self.robot_id,
                joint_index,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=self.physics_drive_sign * float(speed),
                force=self.drive_motor_force,
                physicsClientId=self.client_id,
            )
        return tuple(float(speed) for speed in drive_wheel_speeds)

    def command_twist(
        self,
        linear_velocity: float,
        angular_velocity: float,
        dt: float = 1.0 / 240.0,
    ) -> tuple[float, float]:
        """把车体 v/w 转换为四轮驱动速度和前轮转向速度。"""
        _require_finite_values("twist", (linear_velocity, angular_velocity))
        _require_positive_finite("dt", dt)
        self.linear_velocity = float(linear_velocity)
        self.angular_velocity = float(angular_velocity)
        drive_speed = self.linear_velocity / self.wheel_radius
        if abs(self.linear_velocity) < 1e-6:
            desired_angle = 0.0
        else:
            desired_angle = math.atan(self.model_spec.axle_distance * self.angular_velocity / self.linear_velocity)
        desired_angle = max(-self.max_steering_angle, min(self.max_steering_angle, desired_angle))
        steering_rates = tuple(
            max(-self.MAX_STEERING_RATE, min(self.MAX_STEERING_RATE, (desired_angle - target) / dt))
            for target in self._steering_targets
        )
        self.command_wheel_speeds((drive_speed, drive_speed, drive_speed, drive_speed), steering_rates, dt=dt)
        self.left_wheel_speed = drive_speed
        self.right_wheel_speed = drive_speed
        return drive_speed, drive_speed

    def read_steering_wheel_angles(self) -> tuple[float, ...]:
        """从两个前轮转向关节读取实际角度，而不是回显积分目标。"""
        return tuple(
            float(p.getJointState(self.robot_id, joint_index, physicsClientId=self.client_id)[0])
            for joint_index in self.steering_joint_indices
        )


def create_robot(
    client_id: int,
    robot_model: str,
    *,
    start_x: float = 0.0,
    start_y: float = 0.0,
    base_height: float | None = None,
    start_orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    drive_motor_force: float = 5.0,
) -> DifferentialDriveRobot:
    """按注册表创建四种阶段一车型，统一处理启动位置和内部参数默认值。"""
    spec = get_robot_model(robot_model)
    existing_body_ids = {
        p.getBodyUniqueId(index, physicsClientId=client_id)
        for index in range(p.getNumBodies(physicsClientId=client_id))
    }
    common = {
        "client_id": client_id,
        "urdf_path": spec.urdf_path,
        "wheel_base": spec.wheel_track,
        "wheel_radius": spec.wheel_radius,
        "start_x": start_x,
        "start_y": start_y,
        "base_height": spec.base_height if base_height is None else base_height,
        "start_orientation": start_orientation,
        "drive_motor_force": drive_motor_force,
    }
    try:
        if spec.controller_kind == "active_steering":
            return ActiveSteeringRobot(model_spec=spec, **common)
        return DifferentialDriveRobot(model_spec=spec, **common)
    except Exception:
        # 构造函数可能在 loadURDF 之后解析关节失败，此时清理本次新增的半成品刚体。
        current_body_ids = {
            p.getBodyUniqueId(index, physicsClientId=client_id)
            for index in range(p.getNumBodies(physicsClientId=client_id))
        }
        for body_id in current_body_ids - existing_body_ids:
            p.removeBody(body_id, physicsClientId=client_id)
        raise
