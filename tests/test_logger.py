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
    )

    logger.record(state, reference_x=1.5, reference_y=2.5, estimated_x=0.9, estimated_y=1.9)
    log_path = logger.close()

    frame = pd.read_csv(log_path)
    assert list(frame.columns) == [
        "t",
        "x",
        "y",
        "z",
        "roll",
        "pitch",
        "yaw",
        "linear_velocity",
        "angular_velocity",
        "reference_x",
        "reference_y",
        "estimated_x",
        "estimated_y",
    ]
    assert frame.iloc[0]["x"] == 1.0
    assert frame.iloc[0]["reference_y"] == 2.5
