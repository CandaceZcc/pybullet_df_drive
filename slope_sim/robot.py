# 机器人模块：加载差速底盘 URDF，并提供车体速度控制和状态记录。
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import pybullet as p

from slope_sim.controller import wheel_speeds_from_twist


@dataclass(frozen=True)
class RobotState:
    """单个仿真时刻的机器人位姿和速度。"""

    t: float
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    linear_velocity: float
    angular_velocity: float


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
        # PyBullet 根据 URDF 创建多刚体模型，后续用返回的 id 操作机器人。
        self.robot_id = p.loadURDF(
            str(urdf_path),
            basePosition=[start_x, start_y, base_height],
            physicsClientId=client_id,
        )
        self.left_joint, self.right_joint = self._find_wheel_joints()

    def _find_wheel_joints(self) -> tuple[int, int]:
        """从 URDF 里找到左右轮关节，后续才能分别设置轮速。"""
        joints: dict[str, int] = {}
        for joint_index in range(p.getNumJoints(self.robot_id, physicsClientId=self.client_id)):
            info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self.client_id)
            joints[info[1].decode("utf-8")] = joint_index
        return joints["left_wheel_joint"], joints["right_wheel_joint"]

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
                targetVelocity=speed,
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
            linear_velocity=self.linear_velocity,
            angular_velocity=self.angular_velocity,
        )


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
