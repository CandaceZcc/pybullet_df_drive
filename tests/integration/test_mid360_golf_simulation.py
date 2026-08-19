"""MID-360 Golf 离线 simulator 的步进顺序、仿真时钟与输出边界测试。"""

from __future__ import annotations

from collections import Counter
from importlib import import_module
import inspect
from types import SimpleNamespace

import numpy as np
import pybullet as p
import pytest

from slope_sim.interfaces.models import LidarPointCloud
from slope_sim.interfaces.transport import TransportSnapshot, TransportTopicQuality
from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
from slope_sim.interfaces.v2.models import WheelCommandV2
from slope_sim.interfaces.v2.runtime_protocol import V2RuntimeProtocol
from slope_sim.mid360_offline import OfflineMid360AcceptanceTruth
from slope_sim.model_registry import get_robot_model


SESSION_ID = bytes.fromhex("00112233445566778899aabbccddeeff")
SOURCE_SESSION_ID = bytes.fromhex("ffeeddccbbaa99887766554433221100")


class VerifiedRecordingTransport:
    """提供唯一 command peer，并保留 simulator 的全部 raw 输出。"""

    def __init__(self) -> None:
        self.callback = None
        self.published: list[tuple[str, bytes, str, int, float]] = []
        self.closed = False
        self.idle_waits: list[float] = []

    def subscribe(self, topic: str, type_name: str, callback):
        assert (topic, type_name) == (
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
        )
        self.callback = callback
        return object()

    def poll_peer_state(self) -> None:
        return None

    def snapshot(self) -> TransportSnapshot:
        peer_counts = {
            "/sim/wheel/command": 1,
            "/sim/wheel/state": 2,
            "/sim/lidar/points": 2,
            "/sim/rtk/state": 2,
            "/sim/imu/attitude": 2,
        }
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=True,
            published_count=len(self.published),
            received_count=0,
            error_count=0,
            dropped_count=0,
            topic_quality=tuple(
                TransportTopicQuality(
                    topic=topic,
                    peer_connected=True,
                    peer_count=count,
                    protocol_state="verified",
                    protocol_detail="",
                    remote_type_names=("slope_sim.interfaces.v2.Message",) * count,
                    remote_encodings=("proto",) * count,
                    remote_descriptor_sha256=(load_v2_descriptor().sha256.hex(),)
                    * count,
                )
                for topic, count in peer_counts.items()
            ),
        )

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float,
    ) -> bool:
        self.published.append((topic, bytes(payload), type_name, sim_time_ns, wall_time))
        return True

    def wait_idle(self, *, timeout_sec: float) -> None:
        self.idle_waits.append(timeout_sec)

    def close(self) -> None:
        self.closed = True


class EmptyScanner:
    """保留完整 24 步调用，但不重复离线扫描器自己的射线测试。"""

    instances: list["EmptyScanner"] = []

    def __init__(self, _backend: object, _schedule: object, *, sequence: int) -> None:
        self.sequence = sequence
        self.steps: list[int] = []
        self.truth = None
        type(self).instances.append(self)

    def capture_step(self, step: int, *, body_positions_by_id: dict) -> int:
        assert len(body_positions_by_id) == 9
        self.steps.append(step)
        return 0

    def finalize(self, *, timebase_ns: int) -> LidarPointCloud:
        assert self.steps == list(range(24))
        self.truth = OfflineMid360AcceptanceTruth(
            np.empty((0, 3), dtype=np.float64),
            np.empty((0,), dtype=np.int32),
            np.empty((0, 3), dtype=np.float64),
        )
        return LidarPointCloud(timebase_ns, "lidar_link", 0, 1, ())

    def acceptance_truth(self) -> OfflineMid360AcceptanceTruth:
        assert self.truth is not None
        return self.truth


def test_offline_step_scans_before_topic_command_obstacles_and_bullet(
    monkeypatch,
) -> None:
    module = import_module("scripts.mid360_golf_simulation")
    events: list[str] = []

    class Scanner:
        def capture_step(self, step: int, *, body_positions_by_id: dict) -> None:
            assert step == 7
            assert body_positions_by_id == {21: (1.0, 2.0, 3.0)}
            events.append("scan")

    class Manager:
        def snapshot(self, *, include_body_id: bool):
            assert include_body_id is True
            events.append("snapshot")
            return (
                SimpleNamespace(body_id=21, position=(1.0, 2.0, 3.0)),
            )

        def update_moving(self, dt: float) -> None:
            assert dt == 1.0 / 240.0
            events.append("moving")

    monkeypatch.setattr(
        "slope_sim.mid360_golf_drive.p.stepSimulation",
        lambda *, physicsClientId: events.append(f"step:{physicsClientId}"),
    )

    module.execute_offline_golf_step(
        scanner=Scanner(),
        scanner_step=7,
        client_id=11,
        obstacle_manager=Manager(),
        dt=1.0 / 240.0,
        apply_topic_command=lambda: events.append("command"),
    )

    assert events == ["snapshot", "scan", "command", "moving", "step:11"]


def test_motion_frame_eligibility_excludes_only_startup_arcs_and_tail() -> None:
    module = import_module("scripts.mid360_golf_simulation")
    eligibility = module._MotionFrameEligibility(
        route_end_timestamp_ns=500_000_000,
        startup_target_speed_m_s=0.25,
    )

    assert eligibility.observe(
        timestamp_ns=0,
        segment_kind="approach",
        commanded_forward_speed_m_s=0.10,
    ) is False
    assert eligibility.observe(
        timestamp_ns=100_000_000,
        segment_kind="approach",
        commanded_forward_speed_m_s=0.249,
    ) is False
    assert eligibility.observe(
        timestamp_ns=200_000_000,
        segment_kind="approach",
        commanded_forward_speed_m_s=0.25 - 5e-7,
    ) is True
    assert eligibility.observe(
        timestamp_ns=300_000_000,
        segment_kind="arc",
        commanded_forward_speed_m_s=0.30,
    ) is False
    assert eligibility.observe(
        timestamp_ns=400_000_000,
        segment_kind="straight",
        commanded_forward_speed_m_s=0.0,
    ) is True
    assert eligibility.observe(
        timestamp_ns=500_000_000,
        segment_kind="straight",
        commanded_forward_speed_m_s=0.25,
    ) is False


def test_offline_command_receiver_ages_commands_only_in_simulation_time() -> None:
    module = import_module("scripts.mid360_golf_simulation")
    descriptor = load_v2_descriptor()
    transport = VerifiedRecordingTransport()
    controller = V2RuntimeProtocol(
        get_robot_model("df_mid"),
        transport=transport,
        descriptor=descriptor,
        session_id_factory=lambda: SESSION_ID,
    )
    controller.refresh_transport()
    identity = controller.snapshot()
    command = WheelCommandV2(
        timestamp_ns=10_000_000,
        drive_wheel_speed_rad_s=(2.0, 2.0),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=identity.world_generation,
        command_generation=identity.command_generation,
        source_id="mid360.golf.command-peer",
        source_session_id=SOURCE_SESSION_ID,
        robot_model="df_mid",
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=identity.descriptor_sha256,
    )
    receiver = module.OfflineWheelCommandReceiver(controller, descriptor)

    assert receiver.accept_payload(
        V2ProtoCodec(descriptor).encode(command).payload,
        received_at=9_999.0,
    ) is True
    assert controller.mailbox.decision(now=0.060).timed_out is False
    assert controller.mailbox.decision(now=0.111).timed_out is True


def test_offline_command_receiver_waits_for_the_matching_simulation_timestamp() -> None:
    module = import_module("scripts.mid360_golf_simulation")
    descriptor = load_v2_descriptor()
    transport = VerifiedRecordingTransport()
    controller = V2RuntimeProtocol(
        get_robot_model("df_mid"),
        transport=transport,
        descriptor=descriptor,
        session_id_factory=lambda: SESSION_ID,
    )
    controller.refresh_transport()
    identity = controller.snapshot()
    receiver = module.OfflineWheelCommandReceiver(controller, descriptor)
    command = WheelCommandV2(
        timestamp_ns=10_000_000,
        drive_wheel_speed_rad_s=(0.0, 0.0),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=identity.world_generation,
        command_generation=identity.command_generation,
        source_id="mid360.golf.command-peer",
        source_session_id=SOURCE_SESSION_ID,
        robot_model="df_mid",
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=identity.descriptor_sha256,
    )

    with pytest.raises(TimeoutError, match="matching WheelCommand"):
        receiver.wait_for_command(10_000_000, timeout_sec=0.001)
    assert receiver.accept_payload(
        V2ProtoCodec(descriptor).encode(command).payload,
        received_at=99_999.0,
    ) is True
    receiver.wait_for_command(10_000_000, timeout_sec=0.001)


def test_coordinated_short_session_waits_for_each_wheel_command(
    tmp_path, monkeypatch
) -> None:
    module = import_module("scripts.mid360_golf_simulation")
    waits: list[tuple[int, float]] = []
    monkeypatch.setattr(
        module.OfflineWheelCommandReceiver,
        "wait_for_command",
        lambda _self, timestamp_ns, *, timeout_sec: waits.append(
            (timestamp_ns, timeout_sec)
        ),
        raising=False,
    )
    start_path = tmp_path / "start.json"
    start_path.write_text('{"start":true}\n', encoding="ascii")

    result = module.run_mid360_golf_simulation(
        connection_mode=p.DIRECT,
        collection_duration_sec=0.1,
        transport_factory=lambda _descriptor: VerifiedRecordingTransport(),
        session_id_factory=lambda: SESSION_ID,
        scanner_factory=EmptyScanner,
        ready_path=tmp_path / "simulator.ready.json",
        start_path=start_path,
        fault_path=tmp_path / "fault.json",
    )

    assert result["clean_shutdown"] is True
    assert [timestamp for timestamp, _timeout in waits] == [
        index * 10_000_000 for index in range(1, 11)
    ]
    assert all(timeout > 0.0 for _timestamp, timeout in waits)


def test_direct_short_session_publishes_frame_start_pose_and_lookahead() -> None:
    module = import_module("scripts.mid360_golf_simulation")
    assert module.expected_golf_topic_counts() == {
        "/sim/wheel/command": 20_820,
        "/sim/wheel/state": 20_820,
        "/sim/lidar/points": 2_082,
        "/sim/rtk/state": 2_083,
        "/sim/imu/attitude": 2_083,
    }
    assert (
        inspect.signature(module.run_mid360_golf_simulation)
        .parameters["connection_mode"]
        .default
        == p.GUI
    )
    EmptyScanner.instances.clear()
    transport = VerifiedRecordingTransport()

    result = module.run_mid360_golf_simulation(
        connection_mode=p.DIRECT,
        collection_duration_sec=0.1,
        transport_factory=lambda _descriptor: transport,
        session_id_factory=lambda: SESSION_ID,
        scanner_factory=EmptyScanner,
    )

    topics = Counter(item[0] for item in transport.published)
    timestamps = {
        topic: [item[3] for item in transport.published if item[0] == topic]
        for topic in topics
    }
    assert result["clean_shutdown"] is True
    assert result["role"] == "simulator"
    assert result["scene_id"] == "mid360-golf-mapping-v1"
    assert result["simulation_session_id"] == SESSION_ID.hex()
    assert result["descriptor_sha256"] == load_v2_descriptor().sha256.hex()
    assert result["world_generation"] == 1
    assert result["command_generation"] == 1
    assert result["connection_mode"] == "direct"
    assert result["physics_steps"] == 24
    assert result["sim_duration_ns"] == 100_000_000
    assert result["robot_model"] == "df_mid"
    assert result["terrain_model"] == "golf_heightfield"
    assert result["golf_seed"] == 41
    assert result["golf_relief"] == "medium"
    assert result["static_obstacle_count"] == 6
    assert result["moving_obstacle_count"] == 3
    assert result["truth_acceptance"] == {
        "motion": {
            "eligible_frame_count": 0,
            "speed_above_0_1_m_s_frame_count": 0,
            "speed_above_0_1_m_s_ratio": None,
        },
        "deskew": {
            "point_count": 0,
            "within_0_05_m_count": 0,
            "error_p95_upper_bound_m": None,
        },
        "obstacles": [
            {
                "logical_id": logical_id,
                "mode": "static" if logical_id <= 6 else "moving",
                "hit_frame_count": 0,
                "position_bucket_count": 0,
                "position_span_m": 0.0,
            }
            for logical_id in range(1, 10)
        ],
    }
    assert topics == {
        "/sim/wheel/state": 10,
        "/sim/lidar/points": 1,
        "/sim/rtk/state": 2,
        "/sim/imu/attitude": 2,
    }
    assert result["expected_topic_counts"] == {
        "/sim/wheel/command": 10,
        "/sim/wheel/state": 10,
        "/sim/lidar/points": 1,
        "/sim/rtk/state": 2,
        "/sim/imu/attitude": 2,
    }
    assert timestamps["/sim/wheel/state"] == [index * 10_000_000 for index in range(1, 11)]
    assert timestamps["/sim/lidar/points"] == [0]
    assert timestamps["/sim/rtk/state"] == [0, 100_000_000]
    assert timestamps["/sim/imu/attitude"] == [0, 100_000_000]
    assert [(item[0], item[3]) for item in transport.published[:2]] == [
        ("/sim/rtk/state", 0),
        ("/sim/imu/attitude", 0),
    ]
    assert len(EmptyScanner.instances) == 1
    assert EmptyScanner.instances[0].steps == list(range(24))
    assert transport.idle_waits == [2.0]
    assert transport.closed is True
    assert p.isConnected(result["client_id"]) == 0
