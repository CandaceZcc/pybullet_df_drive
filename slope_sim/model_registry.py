# 车型注册表：集中保存阶段一四种车型的 URDF、控制类型和语义关节名。
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RobotModelSpec:
    """单个机器人模型的稳定元数据，避免在控制代码中硬编码关节索引。"""

    name: str
    urdf_path: Path
    controller_kind: str
    wheel_radius: float
    wheel_track: float
    axle_distance: float
    base_height: float
    drive_center_x: float
    drive_joint_names: tuple[str, ...]
    steering_joint_names: tuple[str, ...] = ()
    support_link_names: tuple[str, ...] = ()
    max_steering_angle: float = 0.55


_DIFFERENTIAL_DRIVE_JOINTS = ("left_drive_wheel_joint", "right_drive_wheel_joint")
PROJECT_ROOT = Path(__file__).resolve().parents[1]

ROBOT_MODELS: dict[str, RobotModelSpec] = {
    "df_front": RobotModelSpec(
        name="df_front",
        urdf_path=PROJECT_ROOT / "urdf/df_front.urdf",
        controller_kind="differential",
        wheel_radius=0.10,
        wheel_track=0.50,
        axle_distance=0.52,
        base_height=0.14,
        drive_center_x=0.22,
        drive_joint_names=_DIFFERENTIAL_DRIVE_JOINTS,
        support_link_names=("rear_support",),
    ),
    "df_mid": RobotModelSpec(
        name="df_mid",
        urdf_path=PROJECT_ROOT / "urdf/df_mid.urdf",
        controller_kind="differential",
        wheel_radius=0.10,
        wheel_track=0.50,
        axle_distance=0.60,
        base_height=0.14,
        drive_center_x=0.0,
        drive_joint_names=_DIFFERENTIAL_DRIVE_JOINTS,
        support_link_names=("front_support", "rear_support"),
    ),
    "df_back": RobotModelSpec(
        name="df_back",
        urdf_path=PROJECT_ROOT / "urdf/df_back.urdf",
        controller_kind="differential",
        wheel_radius=0.10,
        wheel_track=0.50,
        axle_distance=0.52,
        base_height=0.14,
        drive_center_x=-0.22,
        drive_joint_names=_DIFFERENTIAL_DRIVE_JOINTS,
        support_link_names=("front_support",),
    ),
    "active_steering_4wd": RobotModelSpec(
        name="active_steering_4wd",
        urdf_path=PROJECT_ROOT / "urdf/active_steering_4wd.urdf",
        controller_kind="active_steering",
        wheel_radius=0.10,
        wheel_track=0.50,
        axle_distance=0.52,
        base_height=0.14,
        drive_center_x=0.0,
        drive_joint_names=(
            "front_left_drive_wheel_joint",
            "front_right_drive_wheel_joint",
            "rear_left_drive_wheel_joint",
            "rear_right_drive_wheel_joint",
        ),
        steering_joint_names=("front_left_steering_joint", "front_right_steering_joint"),
    ),
}


def robot_model_names() -> tuple[str, ...]:
    """按 Dashboard/CLI 固定顺序返回可交付车型。"""
    return tuple(ROBOT_MODELS)


def get_robot_model(robot_model: str) -> RobotModelSpec:
    """读取车型元数据；旧履带等未知车型在入口处直接拒绝。"""
    model_name = robot_model.lower()
    try:
        return ROBOT_MODELS[model_name]
    except KeyError as exc:
        choices = ", ".join(robot_model_names())
        raise ValueError(f"robot_model must be one of: {choices}") from exc
