# 配置模块：定义一次实验需要的参数，并负责从 YAML 与命令行覆盖项中加载配置。
from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class ExperimentConfig:
    """单次仿真实验的完整参数集合。"""

    mode: str = "direct"
    slope_deg: float = 5.0
    duration_sec: float = 5.0
    time_step: float = 1.0 / 240.0
    wheel_base: float = 0.5
    wheel_radius: float = 0.1
    target_linear_velocity: float = 0.4
    target_angular_velocity: float = 0.0
    robot_model: str = "diff_drive"
    drive_model: str = "kinematic"
    dashboard_enabled: bool = True
    dashboard_update_hz: float = 5.0
    dashboard_smoothing_alpha: float = 0.35
    camera_distance: float = 6.0
    camera_yaw: float = 45.0
    camera_pitch: float = -35.0
    camera_target: tuple[float, float, float] = (0.8, 0.0, 0.0)
    lidar_enabled: bool = False
    lidar_ray_count: int = 31
    lidar_max_distance: float = 4.0
    lidar_fov_deg: float = 180.0
    lidar_debug_draw: bool = False
    ground_lateral_friction: float = 1.0
    drive_lateral_friction: float = 1.0
    log_dir: Path = Path("results/logs")
    figure_dir: Path = Path("results/figures")

    def __post_init__(self) -> None:
        """在配置创建后做基础合法性检查，尽早发现错误参数。"""
        mode = self.mode.lower()
        if mode not in {"direct", "gui"}:
            raise ValueError("mode must be 'direct' or 'gui'")
        robot_model = self.robot_model.lower()
        if robot_model not in {"diff_drive", "tracked_proxy"}:
            raise ValueError("robot_model must be 'diff_drive' or 'tracked_proxy'")
        drive_model = self.drive_model.lower()
        if drive_model not in {"kinematic", "physics"}:
            raise ValueError("drive_model must be 'kinematic' or 'physics'")
        if self.duration_sec <= 0:
            raise ValueError("duration_sec must be positive")
        if self.time_step <= 0:
            raise ValueError("time_step must be positive")
        if self.wheel_base <= 0:
            raise ValueError("wheel_base must be positive")
        if self.wheel_radius <= 0:
            raise ValueError("wheel_radius must be positive")
        if self.dashboard_update_hz <= 0:
            raise ValueError("dashboard_update_hz must be positive")
        if not 0.0 < self.dashboard_smoothing_alpha <= 1.0:
            raise ValueError("dashboard_smoothing_alpha must be in (0, 1]")
        if self.camera_distance <= 0:
            raise ValueError("camera_distance must be positive")
        if len(self.camera_target) != 3:
            raise ValueError("camera_target must contain three numbers")
        if self.lidar_ray_count <= 0:
            raise ValueError("lidar_ray_count must be positive")
        if self.lidar_max_distance <= 0:
            raise ValueError("lidar_max_distance must be positive")
        if self.lidar_fov_deg <= 0:
            raise ValueError("lidar_fov_deg must be positive")
        if self.ground_lateral_friction <= 0:
            raise ValueError("ground_lateral_friction must be positive")
        if self.drive_lateral_friction <= 0:
            raise ValueError("drive_lateral_friction must be positive")

        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "robot_model", robot_model)
        object.__setattr__(self, "drive_model", drive_model)
        object.__setattr__(self, "camera_target", tuple(float(value) for value in self.camera_target))
        object.__setattr__(self, "log_dir", Path(self.log_dir))
        object.__setattr__(self, "figure_dir", Path(self.figure_dir))


def load_config(path: str | Path = "configs/experiment.yaml", overrides: dict[str, Any] | None = None) -> ExperimentConfig:
    """读取 YAML 配置，并用命令行参数覆盖其中的字段。"""
    config_path = Path(path)
    data: dict[str, Any] = {}
    if config_path.exists():
        loaded = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        if not isinstance(loaded, dict):
            raise ValueError(f"Config file must contain a mapping: {config_path}")
        data.update(loaded)

    for key, value in (overrides or {}).items():
        if value is None:
            continue
        # --gui 是命令行快捷开关，对应配置中的 mode: gui。
        if key == "gui":
            if value:
                data["mode"] = "gui"
            continue
        if key == "no_dashboard":
            if value:
                data["dashboard_enabled"] = False
            continue
        if key == "lidar":
            if value:
                data["lidar_enabled"] = True
            continue
        if key == "ground_friction":
            data["ground_lateral_friction"] = value
            continue
        if key == "wheel_friction":
            data["drive_lateral_friction"] = value
            continue
        data[key] = value

    valid_fields = {field.name for field in fields(ExperimentConfig)}
    unknown = sorted(set(data) - valid_fields)
    if unknown:
        raise ValueError(f"Unknown config keys: {', '.join(unknown)}")

    return ExperimentConfig(**data)
