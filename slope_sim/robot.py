# 机器人模块：加载差速底盘 URDF，并提供车体速度控制和状态记录。
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import pybullet as p

from slope_sim.controller import wheel_speeds_from_twist
from slope_sim.sensors import LidarSummary
from slope_sim.telemetry import RobotTelemetry, body_frame_velocity, slip_ratio


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
    ) -> None:
        """加载机器人模型，并初始化运动学状态。"""
        self.client_id = client_id
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.base_height = base_height
        self.x = start_x
        self.y = start_y
        self.yaw = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.left_wheel_speed = 0.0
        self.right_wheel_speed = 0.0
        # 当前 URDF 轮轴正方向和“车头朝 +x 前进”的项目约定一致；后续换模型时可集中调整。
        self.physics_drive_sign = 1.0
        # PyBullet 根据 URDF 创建多刚体模型，后续用返回的 id 操作机器人。
        self.robot_id = p.loadURDF(
            str(urdf_path),
            basePosition=[start_x, start_y, base_height],
            physicsClientId=client_id,
        )
        self.left_joint, self.right_joint = self._find_wheel_joints()
        self.left_contact_links, self.right_contact_links = self._find_drive_side_links()
        self._disable_passive_drive_joints()

    def _find_wheel_joints(self) -> tuple[int, int]:
        """从 URDF 里找到左右轮关节，后续才能分别设置轮速。"""
        joints: dict[str, int] = {}
        for joint_index in range(p.getNumJoints(self.robot_id, physicsClientId=self.client_id)):
            info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self.client_id)
            joints[info[1].decode("utf-8")] = joint_index
        return joints["left_wheel_joint"], joints["right_wheel_joint"]

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

    def _disable_passive_drive_joints(self) -> None:
        """关闭非驱动连续关节的默认电机，让 tracked_proxy 的滚轮能自由转动。"""
        for joint_index in range(p.getNumJoints(self.robot_id, physicsClientId=self.client_id)):
            if joint_index in {self.left_joint, self.right_joint}:
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

    def apply_drive_friction(self, lateral_friction: float) -> None:
        """给所有左右驱动接触件设置摩擦，便于做坡面打滑实验。"""
        for link_index in self.left_contact_links | self.right_contact_links:
            p.changeDynamics(
                self.robot_id,
                link_index,
                lateralFriction=lateral_friction,
                rollingFriction=0.02,
                spinningFriction=0.02,
                physicsClientId=self.client_id,
            )

    def command_twist(self, linear_velocity: float, angular_velocity: float) -> tuple[float, float]:
        """接收车体速度 v/w，并转换成左右轮的目标角速度。"""
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.left_wheel_speed, self.right_wheel_speed = wheel_speeds_from_twist(
            linear_velocity,
            angular_velocity,
            self.wheel_base,
            self.wheel_radius,
        )
        for joint, speed in ((self.left_joint, self.left_wheel_speed), (self.right_joint, self.right_wheel_speed)):
            p.setJointMotorControl2(
                self.robot_id,
                joint,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=self.physics_drive_sign * speed,
                force=5.0,
                physicsClientId=self.client_id,
            )
        return self.left_wheel_speed, self.right_wheel_speed

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
            linear_velocity=self.linear_velocity,
            angular_velocity=self.angular_velocity,
            command_linear_velocity=self.linear_velocity,
            command_angular_velocity=self.angular_velocity,
            left_target_drive_speed=self.left_wheel_speed,
            right_target_drive_speed=self.right_wheel_speed,
            left_actual_drive_speed=self.left_wheel_speed,
            right_actual_drive_speed=self.right_wheel_speed,
        )

    def read_physics_state(
        self,
        t: float,
        command_linear_velocity: float,
        command_angular_velocity: float,
        ground_lateral_friction: float,
        drive_lateral_friction: float,
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

        left_joint_state = p.getJointState(self.robot_id, self.left_joint, physicsClientId=self.client_id)
        right_joint_state = p.getJointState(self.robot_id, self.right_joint, physicsClientId=self.client_id)
        # 日志里仍按项目约定记录：正轮速表示车体向前。
        left_position = self.physics_drive_sign * float(left_joint_state[0])
        left_actual_speed = self.physics_drive_sign * float(left_joint_state[1])
        right_position = self.physics_drive_sign * float(right_joint_state[0])
        right_actual_speed = self.physics_drive_sign * float(right_joint_state[1])
        left_torque = float(left_joint_state[3])
        right_torque = float(right_joint_state[3])

        left_force, right_force, contact_count = self._drive_contact_forces()
        left_surface_speed = self.wheel_radius * left_actual_speed
        right_surface_speed = self.wheel_radius * right_actual_speed
        lidar = lidar_summary or LidarSummary()

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
            linear_velocity=body_forward,
            angular_velocity=float(angular_world[2]),
            command_linear_velocity=command_linear_velocity,
            command_angular_velocity=command_angular_velocity,
            left_target_drive_speed=self.left_wheel_speed,
            right_target_drive_speed=self.right_wheel_speed,
            left_actual_drive_speed=left_actual_speed,
            right_actual_drive_speed=right_actual_speed,
            left_drive_position=left_position,
            right_drive_position=right_position,
            left_motor_torque=left_torque,
            right_motor_torque=right_torque,
            ground_lateral_friction=ground_lateral_friction,
            drive_lateral_friction=drive_lateral_friction,
            left_contact_normal_force=left_force,
            right_contact_normal_force=right_force,
            contact_count=contact_count,
            left_slip_ratio=slip_ratio(left_surface_speed, body_forward),
            right_slip_ratio=slip_ratio(right_surface_speed, body_forward),
            body_lateral_slip_speed=body_lateral,
            lidar_min_distance=lidar.min_distance,
            lidar_front_distance=lidar.front_distance,
            lidar_left_distance=lidar.left_distance,
            lidar_right_distance=lidar.right_distance,
            nearest_obstacle_distance=lidar.min_distance,
            bumper_contact=contact_count > 0,
        )

    def _drive_contact_forces(self) -> tuple[float, float, int]:
        """汇总左右驱动接触件的法向力。"""
        left_force = 0.0
        right_force = 0.0
        contact_count = 0
        contacts = p.getContactPoints(bodyA=self.robot_id, physicsClientId=self.client_id)
        for contact in contacts:
            link_index = contact[3]
            normal_force = float(contact[9]) if len(contact) > 9 else 0.0
            if link_index in self.left_contact_links:
                left_force += normal_force
            if link_index in self.right_contact_links:
                right_force += normal_force
            if normal_force > 0.0:
                contact_count += 1
        return left_force, right_force, contact_count


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
