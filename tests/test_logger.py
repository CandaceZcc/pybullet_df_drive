# 日志测试：确保 CSV 输出字段稳定，便于后续分析脚本读取。
from pathlib import Path

import pandas as pd

from slope_sim.logger import CsvSimulationLogger
from slope_sim.robot import RobotState


def test_csv_logger_writes_robot_state_rows(tmp_path: Path):
    logger = CsvSimulationLogger(tmp_path, prefix="unit")
    state = RobotState(
        t=0.5,
        x=1.0,
        y=2.0,
        z=0.3,
        roll=0.1,
        pitch=0.2,
        yaw=0.4,
        linear_velocity=0.6,
        angular_velocity=0.7,
        vx=0.3,
        vy=0.4,
        vz=0.0,
        body_forward_speed=0.3,
        yaw_rate=0.7,
        left_target_drive_speed=6.0,
        right_target_drive_speed=7.0,
        left_actual_drive_speed=5.5,
        right_actual_drive_speed=6.5,
        left_track_surface_speed=0.44,
        right_track_surface_speed=0.52,
        left_body_track_speed=0.40,
        right_body_track_speed=0.50,
        left_slip_ratio=0.1,
        right_slip_ratio=0.2,
        lidar_min_distance=2.5,
    )

    logger.record(state, reference_x=1.5, reference_y=2.5, estimated_x=0.9, estimated_y=1.9)
    log_path = logger.close()

    frame = pd.read_csv(log_path)
    assert {"t", "x", "y", "z", "roll", "pitch", "yaw"}.issubset(frame.columns)
    assert {"vx", "vy", "vz", "body_forward_speed", "yaw_rate"}.issubset(frame.columns)
    assert {"left_actual_drive_speed", "right_actual_drive_speed", "lidar_min_distance"}.issubset(frame.columns)
    assert {"left_track_surface_speed", "right_body_track_speed"}.issubset(frame.columns)
    assert frame.iloc[0]["x"] == 1.0
    assert frame.iloc[0]["reference_y"] == 2.5
    assert frame.iloc[0]["left_track_surface_speed"] == 0.44
    assert frame.iloc[0]["left_slip_ratio"] == 0.1
