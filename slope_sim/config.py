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
    dashboard_plot_update_hz: float = 5.0
    dashboard_plot_window_sec: float = 20.0
    manual_linear_acceleration_limit: float = 1.5
    manual_angular_acceleration_limit: float = 4.0
    camera_distance: float = 6.0
    camera_yaw: float = 45.0
    camera_pitch: float = -35.0
    camera_target: tuple[float, float, float] = (0.8, 0.0, 0.0)
    camera_follow_enabled: bool = False
    camera_follow_view: str = "custom"
    lidar_enabled: bool = False
    lidar_ray_count: int = 31
    lidar_max_distance: float = 4.0
    lidar_fov_deg: float = 180.0
    lidar_debug_draw: bool = False
    terrain_model: str = "box_slope"
    dam_toe_length: float = 2.0
    dam_slope_length: float = 8.0
    dam_crest_length: float = 3.0
    dam_exit_length: float = 2.0
    dam_width: float = 4.0
    dam_wall_height: float = 0.35
    terrain_guard_enabled: bool = True
    gui_model_switch_enabled: bool = False
    ground_lateral_friction: float = 1.0
    ground_rolling_friction: float = 0.02
    ground_spinning_friction: float = 0.0
    drive_lateral_friction: float = 2.0
    support_lateral_friction: float = 0.03
    drive_motor_force: float = 5.0
    track_anisotropic_friction: tuple[float, float, float] = (2.0, 0.05, 0.05)
    track_drive_mode: str = "all_rollers"
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
        terrain_model = self.terrain_model.lower()
        if terrain_model not in {"box_slope", "twr_slope_5deg", "dam_slope"}:
            raise ValueError("terrain_model must be 'box_slope', 'twr_slope_5deg', or 'dam_slope'")
        if terrain_model == "twr_slope_5deg" and abs(float(self.slope_deg) - 5.0) > 1e-9:
            raise ValueError("terrain_model 'twr_slope_5deg' requires slope_deg: 5.0")
        camera_follow_view = self.camera_follow_view.lower()
        if camera_follow_view not in {"front", "side", "custom"}:
            raise ValueError("camera_follow_view must be 'front', 'side', or 'custom'")
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
        if len(self.camera_target) != 3:
            raise ValueError("camera_target must contain three numbers")
        if self.dam_toe_length <= 0:
            raise ValueError("dam_toe_length must be positive")
        if self.dam_slope_length <= 0:
            raise ValueError("dam_slope_length must be positive")
        if self.dam_crest_length <= 0:
            raise ValueError("dam_crest_length must be positive")
        if self.dam_exit_length <= 0:
            raise ValueError("dam_exit_length must be positive")
        if self.dam_width <= 0:
            raise ValueError("dam_width must be positive")
        if self.dam_wall_height <= 0:
            raise ValueError("dam_wall_height must be positive")
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
        if len(self.track_anisotropic_friction) != 3:
            raise ValueError("track_anisotropic_friction must contain three numbers")
        track_anisotropic_friction = tuple(float(value) for value in self.track_anisotropic_friction)
        if any(value <= 0 for value in track_anisotropic_friction):
            raise ValueError("track_anisotropic_friction values must be positive")
        track_drive_mode = self.track_drive_mode.lower()
        if track_drive_mode not in {"all_rollers", "center_only"}:
            raise ValueError("track_drive_mode must be 'all_rollers' or 'center_only'")

        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "robot_model", robot_model)
        object.__setattr__(self, "drive_model", drive_model)
        object.__setattr__(self, "terrain_model", terrain_model)
        object.__setattr__(self, "camera_follow_view", camera_follow_view)
        object.__setattr__(self, "track_anisotropic_friction", track_anisotropic_friction)
        object.__setattr__(self, "track_drive_mode", track_drive_mode)
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
        if key == "support_friction":
            data["support_lateral_friction"] = value
            continue
        data[key] = value

    valid_fields = {field.name for field in fields(ExperimentConfig)}
    unknown = sorted(set(data) - valid_fields)
    if unknown:
        raise ValueError(f"Unknown config keys: {', '.join(unknown)}")

    return ExperimentConfig(**data)
