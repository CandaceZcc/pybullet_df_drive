from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class ExperimentConfig:
    mode: str = "direct"
    slope_deg: float = 5.0
    duration_sec: float = 5.0
    time_step: float = 1.0 / 240.0
    wheel_base: float = 0.5
    wheel_radius: float = 0.1
    target_linear_velocity: float = 0.4
    target_angular_velocity: float = 0.0
    log_dir: Path = Path("results/logs")
    figure_dir: Path = Path("results/figures")

    def __post_init__(self) -> None:
        mode = self.mode.lower()
        if mode not in {"direct", "gui"}:
            raise ValueError("mode must be 'direct' or 'gui'")
        if self.duration_sec <= 0:
            raise ValueError("duration_sec must be positive")
        if self.time_step <= 0:
            raise ValueError("time_step must be positive")
        if self.wheel_base <= 0:
            raise ValueError("wheel_base must be positive")
        if self.wheel_radius <= 0:
            raise ValueError("wheel_radius must be positive")

        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "log_dir", Path(self.log_dir))
        object.__setattr__(self, "figure_dir", Path(self.figure_dir))


def load_config(path: str | Path = "configs/experiment.yaml", overrides: dict[str, Any] | None = None) -> ExperimentConfig:
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
        if key == "gui":
            if value:
                data["mode"] = "gui"
            continue
        data[key] = value

    valid_fields = {field.name for field in fields(ExperimentConfig)}
    unknown = sorted(set(data) - valid_fields)
    if unknown:
        raise ValueError(f"Unknown config keys: {', '.join(unknown)}")

    return ExperimentConfig(**data)

