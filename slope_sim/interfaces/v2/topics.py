"""阶段四五话题合同：集中定义方向、类型、频率和连续性范围。"""
from dataclasses import dataclass


@dataclass(frozen=True)
class V2TopicContract:
    """一条 v2 eCAL topic 的固定线协议属性。"""

    topic: str
    type_name: str
    rate_hz: int
    direction: str


V2_TOPICS = (
    V2TopicContract("/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand", 100, "subscribe"),
    V2TopicContract("/sim/wheel/state", "slope_sim.interfaces.v2.WheelState", 100, "publish"),
    V2TopicContract("/sim/lidar/points", "slope_sim.interfaces.v2.LidarPointCloud", 10, "publish"),
    V2TopicContract("/sim/rtk/state", "slope_sim.interfaces.v2.RtkState", 10, "publish"),
    V2TopicContract("/sim/imu/attitude", "slope_sim.interfaces.v2.ImuAttitude", 10, "publish"),
)
V2_OUTPUT_TOPICS = tuple(contract.topic for contract in V2_TOPICS if contract.direction == "publish")
V2_BY_TOPIC = {contract.topic: contract for contract in V2_TOPICS}
if len(V2_BY_TOPIC) != len(V2_TOPICS):
    raise RuntimeError("v2 topic contract contains duplicate topic names")
