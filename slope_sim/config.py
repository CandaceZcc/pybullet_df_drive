# 配置模块：定义一次实验需要的参数，并负责从 YAML 与命令行覆盖项中加载配置。
from __future__ import annotations

from dataclasses import dataclass, fields
import math
from pathlib import Path
from typing import Any

import yaml

from slope_sim.model_registry import robot_model_names
from slope_sim.scene import terrain_model_names


def _require_finite(name: str, value: object) -> None:
    """拒绝 NaN/Inf，避免非法数值进入步数计算、场景姿态或 PyBullet 参数。"""
    try:
        finite = math.isfinite(float(value))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not finite:
        raise ValueError(f"{name} must be a finite number")


def _require_optional_path(name: str, value: object) -> Path | None:
    """校验可选场景路径，并统一转换为 Path。"""
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (str, Path)):
        raise ValueError(f"{name} must be None, a string, or a Path")
    if isinstance(value, str) and not value:
        raise ValueError(f"{name} must not be empty")
    return Path(value)


@dataclass(frozen=True)
class ExperimentConfig:
    """单次仿真实验的完整参数集合。"""

    mode: str = "direct"
    slope_deg: float = 0.0
    duration_sec: float = 5.0
    time_step: float = 1.0 / 240.0
    target_linear_velocity: float = 0.4
    target_angular_velocity: float = 0.0
    robot_model: str = "df_back"
    drive_model: str = "physics"
    interface_mode: str = "auto"
    interface_enabled: bool = True
    interface_log_enabled: bool = True
    telemetry_log_hz: float = 20.0
    scene_in: Path | None = None
    scene_out: Path | None = None
    developer_diagnostics_enabled: bool = False
    dashboard_enabled: bool = True
    dashboard_update_hz: float = 5.0
    dashboard_smoothing_alpha: float = 0.35
    dashboard_plot_update_hz: float = 5.0
    dashboard_plot_window_sec: float = 20.0
    manual_linear_acceleration_limit: float = 1.5
    manual_angular_acceleration_limit: float = 4.0
    camera_distance: float = 6.0
    camera_yaw: float = 45.0
    camera_pitch: float = -35.0
    camera_target: tuple[float, float, float] = (0.8, 0.0, 0.0)
    camera_follow_enabled: bool = True
    camera_follow_view: str = "front"
    lidar_enabled: bool = False
    lidar_ray_count: int = 31
    lidar_max_distance: float = 4.0
    lidar_fov_deg: float = 180.0
    lidar_debug_draw: bool = False
    terrain_model: str = "flat"
    golf_seed: int = 0
    golf_relief: str = "medium"
    ground_lateral_friction: float = 1.0
    ground_rolling_friction: float = 0.02
    ground_spinning_friction: float = 0.0
    drive_lateral_friction: float = 2.0
    support_lateral_friction: float = 0.03
    drive_motor_force: float = 5.0
    log_dir: Path = Path("results/logs")
    figure_dir: Path = Path("results/figures")

    def __post_init__(self) -> None:
        """在配置创建后做基础合法性检查，尽早发现错误参数。"""
        mode = self.mode.lower()
        if mode not in {"direct", "gui"}:
            raise ValueError("mode must be 'direct' or 'gui'")
        robot_model = self.robot_model.lower()
        if robot_model not in set(robot_model_names()):
            raise ValueError(f"robot_model must be one of: {', '.join(robot_model_names())}")
        drive_model = self.drive_model.lower()
        if drive_model != "physics":
            raise ValueError("drive_model must be 'physics' in stage 1")
        if not isinstance(self.interface_mode, str) or self.interface_mode not in {"auto", "ecal", "local"}:
            raise ValueError("interface_mode must be 'auto', 'ecal', or 'local'")
        for field_name in ("interface_enabled", "interface_log_enabled", "developer_diagnostics_enabled"):
            if not isinstance(getattr(self, field_name), bool):
                raise ValueError(f"{field_name} must be a bool")
        scene_in = _require_optional_path("scene_in", self.scene_in)
        scene_out = _require_optional_path("scene_out", self.scene_out)
        terrain_model = self.terrain_model.lower()
        if terrain_model not in set(terrain_model_names()):
            raise ValueError(f"terrain_model must be one of: {', '.join(terrain_model_names())}")
        golf_relief = self.golf_relief.lower()
        if golf_relief not in {"low", "medium", "high"}:
            raise ValueError("golf_relief must be 'low', 'medium', or 'high'")
        camera_follow_view = self.camera_follow_view.lower()
        if camera_follow_view not in {"front", "side", "custom"}:
            raise ValueError("camera_follow_view must be 'front', 'side', or 'custom'")
        if len(self.camera_target) != 3:
            raise ValueError("camera_target must contain three numbers")
        finite_fields = (
            "slope_deg",
            "duration_sec",
            "time_step",
            "target_linear_velocity",
            "target_angular_velocity",
            "dashboard_update_hz",
            "telemetry_log_hz",
            "dashboard_smoothing_alpha",
            "dashboard_plot_update_hz",
            "dashboard_plot_window_sec",
            "manual_linear_acceleration_limit",
            "manual_angular_acceleration_limit",
            "camera_distance",
            "camera_yaw",
            "camera_pitch",
            "lidar_ray_count",
            "lidar_max_distance",
            "lidar_fov_deg",
            "golf_seed",
            "ground_lateral_friction",
            "ground_rolling_friction",
            "ground_spinning_friction",
            "drive_lateral_friction",
            "support_lateral_friction",
            "drive_motor_force",
        )
        for field_name in finite_fields:
            _require_finite(field_name, getattr(self, field_name))
        for index, value in enumerate(self.camera_target):
            _require_finite(f"camera_target[{index}]", value)
        if self.duration_sec <= 0:
            raise ValueError("duration_sec must be positive")
        if self.time_step <= 0:
            raise ValueError("time_step must be positive")
        if self.dashboard_update_hz <= 0:
            raise ValueError("dashboard_update_hz must be positive")
        if self.telemetry_log_hz <= 0:
            raise ValueError("telemetry_log_hz must be positive")
        if not 0.0 < self.dashboard_smoothing_alpha <= 1.0:
            raise ValueError("dashboard_smoothing_alpha must be in (0, 1]")
        if self.dashboard_plot_update_hz <= 0:
            raise ValueError("dashboard_plot_update_hz must be positive")
        if self.dashboard_plot_window_sec <= 0:
            raise ValueError("dashboard_plot_window_sec must be positive")
        if self.manual_linear_acceleration_limit <= 0:
            raise ValueError("manual_linear_acceleration_limit must be positive")
        if self.manual_angular_acceleration_limit <= 0:
            raise ValueError("manual_angular_acceleration_limit must be positive")
        if self.camera_distance <= 0:
            raise ValueError("camera_distance must be positive")
        if self.lidar_ray_count <= 0:
            raise ValueError("lidar_ray_count must be positive")
        if self.lidar_max_distance <= 0:
            raise ValueError("lidar_max_distance must be positive")
        if self.lidar_fov_deg <= 0:
            raise ValueError("lidar_fov_deg must be positive")
        if self.ground_lateral_friction <= 0:
            raise ValueError("ground_lateral_friction must be positive")
        if self.ground_rolling_friction <= 0:
            raise ValueError("ground_rolling_friction must be positive")
        if self.ground_spinning_friction < 0:
            raise ValueError("ground_spinning_friction must be non-negative")
        if self.drive_lateral_friction <= 0:
            raise ValueError("drive_lateral_friction must be positive")
        if self.support_lateral_friction <= 0:
            raise ValueError("support_lateral_friction must be positive")
        if self.drive_motor_force <= 0:
            raise ValueError("drive_motor_force must be positive")
        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "robot_model", robot_model)
        object.__setattr__(self, "drive_model", drive_model)
        object.__setattr__(self, "scene_in", scene_in)
        object.__setattr__(self, "scene_out", scene_out)
        object.__setattr__(self, "terrain_model", terrain_model)
        object.__setattr__(self, "golf_relief", golf_relief)
        object.__setattr__(self, "camera_follow_view", camera_follow_view)
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
        if key == "no_interface":
            if value:
                data["interface_enabled"] = False
            continue
        if key == "no_interface_log":
            if value:
                data["interface_log_enabled"] = False
            continue
        if key == "developer_diagnostics":
            if value:
                data["developer_diagnostics_enabled"] = True
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
        if key == "support_friction":
            data["support_lateral_friction"] = value
            continue
        data[key] = value

    valid_fields = {field.name for field in fields(ExperimentConfig)}
    unknown = sorted(set(data) - valid_fields)
    if unknown:
        raise ValueError(f"Unknown config keys: {', '.join(unknown)}")

    return ExperimentConfig(**data)
