#!/usr/bin/env python3
"""MID-360 Golf 离线 simulator：按 240 Hz 状态分时扫描并发布 v2 原始帧。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
from threading import Condition
import time
from typing import Callable

import pybullet as p


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import build_world_from_scene_document
from slope_sim.interfaces.models import ImuAttitude, LidarPointCloud
from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity, load_v2_descriptor
from slope_sim.interfaces.v2.models import (
    ImuAttitudeV2,
    LidarPointCloudV2,
    LidarPointV2,
    Point3dV2,
    RtkStateV2,
)
from slope_sim.interfaces.v2.runtime_protocol import V2RuntimeProtocol
from slope_sim.interfaces.v2.sensor_frames import (
    V2OutputFramePublisher,
    V2PublishCadence,
    V2WheelStateFactory,
)
from slope_sim.interfaces.v2.session import OutputIdentity
from slope_sim.interfaces.v2.transport import create_v2_ecal_transport
from slope_sim.lidar_pointcloud import MID360_PATTERN_VERSION
from slope_sim.mapping_acceptance import (
    GolfCaptureAcceptance,
    capture_acceptance_document,
)
from slope_sim.mapping_replay import recover_pose_node
from slope_sim.mid360_golf_drive import (
    GolfSafetyMonitor,
    advance_golf_physics_step,
    build_canonical_golf_route,
    obstacle_contact_body_ids,
)
from slope_sim.mid360_offline import (
    OfflineMid360AcceptanceTruth,
    OfflineMid360FrameScanner,
    OfflineMid360Profile,
    OfflineMid360Schedule,
)
from slope_sim.model_registry import get_robot_model
from slope_sim.scene import TerrainBounds
from slope_sim.scene_config import load_scene
from slope_sim.sensor_backend import PyBulletSensorBackend
from slope_sim.truth_sensors import (
    Stage4RtkState,
    Stage4SensorMounts,
    Stage4TruthSensorSuite,
)


_SCENE_PATH = ROOT / "configs/mid360_golf_mapping.yaml"
_SCENE_ID = "mid360-golf-mapping-v1"
_PHYSICS_STEP_SEC = 1.0 / 240.0
_FRAME_PERIOD_NS = 100_000_000
_MAX_SIMULATION_NS = 240_000_000_000
_ZERO_TAIL_SEC = 0.5
_COMMAND_ACCELERATION_RAD_S2 = 20.0
_COMMAND_RESPONSE_TIMEOUT_SEC = 2.0
_TRANSPORT_IDLE_TIMEOUT_SEC = 2.0
_STARTUP_TIMEOUT_SEC = 5.0
_SIMULATOR_PEER_COUNTS = {
    "/sim/wheel/command": 1,
    "/sim/wheel/state": 2,
    "/sim/lidar/points": 2,
    "/sim/rtk/state": 2,
    "/sim/imu/attitude": 2,
}


@dataclass(slots=True)
class _MotionFrameEligibility:
    """锁存启动完成状态，避免中途低命令把慢速帧移出验收分母。"""

    route_end_timestamp_ns: int
    startup_target_speed_m_s: float
    _startup_complete: bool = False

    def observe(
        self,
        *,
        timestamp_ns: int,
        segment_kind: str,
        commanded_forward_speed_m_s: float,
    ) -> bool:
        if (
            not self._startup_complete
            and commanded_forward_speed_m_s
            >= self.startup_target_speed_m_s - 1e-6
        ):
            self._startup_complete = True
        return (
            self._startup_complete
            and timestamp_ns < self.route_end_timestamp_ns
            and segment_kind != "arc"
        )


def _atomic_write_json(path: Path, document: dict[str, object]) -> None:
    """在 marker 同目录写临时文件并原子替换最终路径。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _read_exact_marker(path: Path, *, key: str) -> object | None:
    if not path.exists():
        return None
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"invalid {key} marker") from error
    if not isinstance(document, dict) or set(document) != {key}:
        raise RuntimeError(f"invalid {key} marker")
    return document[key]


def _wait_for_start(path: Path, *, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while True:
        value = _read_exact_marker(path, key="start")
        if value is True:
            return
        if value is not None:
            raise RuntimeError("start marker must contain true")
        if time.monotonic() >= deadline:
            raise TimeoutError("MID-360 Golf start marker did not arrive")
        time.sleep(0.01)


def _fault_from_marker(path: Path | None) -> str | None:
    if path is None:
        return None
    value = _read_exact_marker(path, key="fault")
    if value is None:
        return None
    if not isinstance(value, str) or not value:
        raise RuntimeError("fault marker must contain a nonempty reason")
    return value


@dataclass(slots=True)
class _PendingOfflineFrame:
    """保存帧首姿态身份，并在 24 个状态完成后附加原始点云。"""

    timebase_ns: int
    scanner: object
    lidar_identity: OutputIdentity
    rtk: RtkStateV2
    imu: ImuAttitudeV2


class OfflineWheelCommandReceiver:
    """在 raw 回调中用命令仿真 timestamp 写入既有 authority/mailbox。"""

    def __init__(
        self,
        controller: V2RuntimeProtocol,
        descriptor: DescriptorIdentity,
    ) -> None:
        if not isinstance(controller, V2RuntimeProtocol):
            raise ValueError("controller must be a V2RuntimeProtocol")
        if not isinstance(descriptor, DescriptorIdentity):
            raise ValueError("descriptor must be a DescriptorIdentity")
        self._controller = controller
        self._codec = V2ProtoCodec(descriptor)
        self._condition = Condition()
        self._fault_reason: str | None = None
        self._latest_command_timestamp_ns: int | None = None

    @property
    def fault_reason(self) -> str | None:
        with self._condition:
            return self._fault_reason

    def _latch(self, reason: str) -> None:
        with self._condition:
            if self._fault_reason is None:
                self._fault_reason = reason
            self._condition.notify_all()

    def accept_payload(self, payload: bytes, *, received_at: float) -> bool:
        """忽略 callback 墙钟年龄，只保留其签名并按命令仿真时间提交。"""
        del received_at
        ingress = self._controller.capture_ingress()
        try:
            command = self._codec.decode_wheel_command(payload)
        except (TypeError, ValueError) as error:
            self._latch(f"WheelCommand payload is invalid: {error}")
            return False
        accepted = self._controller.accept_decoded_command(
            command,
            received_at=command.timestamp_ns / 1_000_000_000.0,
            ingress=ingress,
        )
        if not accepted:
            self._latch("WheelCommand identity or sequence was rejected")
        else:
            with self._condition:
                self._latest_command_timestamp_ns = command.timestamp_ns
                self._condition.notify_all()
        return accepted

    def wait_for_command(self, timestamp_ns: int, *, timeout_sec: float) -> None:
        """协调运行逐步等待同仿真时刻命令，避免异步 latest 槽覆盖中间帧。"""
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int) or timestamp_ns < 0:
            raise ValueError("timestamp_ns must be a nonnegative integer")
        if (
            isinstance(timeout_sec, bool)
            or not isinstance(timeout_sec, (int, float))
            or not math.isfinite(float(timeout_sec))
            or timeout_sec <= 0.0
        ):
            raise ValueError("timeout_sec must be positive")
        deadline = time.monotonic() + float(timeout_sec)
        with self._condition:
            while self._latest_command_timestamp_ns != timestamp_ns:
                if self._fault_reason is not None:
                    raise RuntimeError(self._fault_reason)
                if (
                    self._latest_command_timestamp_ns is not None
                    and self._latest_command_timestamp_ns > timestamp_ns
                ):
                    raise RuntimeError("WheelCommand advanced past the requested timestamp")
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("timed out waiting for matching WheelCommand")
                self._condition.wait(timeout=remaining)


def execute_offline_golf_step(
    *,
    scanner: object,
    scanner_step: int,
    client_id: int,
    obstacle_manager: object,
    dt: float,
    apply_topic_command: Callable[[], object],
) -> None:
    """冻结并扫描当前状态，再复用 Task 3 的命令、障碍物、物理顺序。"""
    capture = getattr(scanner, "capture_step", None)
    if not callable(capture):
        raise ValueError("scanner must provide capture_step")
    if not callable(apply_topic_command):
        raise ValueError("apply_topic_command must be callable")
    snapshots = obstacle_manager.snapshot(include_body_id=True)
    body_positions_by_id = {
        int(snapshot.body_id): snapshot.position
        for snapshot in snapshots
        if snapshot.body_id is not None
    }
    if len(body_positions_by_id) != len(snapshots):
        raise RuntimeError("offline obstacle snapshots must have unique body ids")
    capture(scanner_step, body_positions_by_id=body_positions_by_id)
    advance_golf_physics_step(
        client_id=client_id,
        obstacle_manager=obstacle_manager,
        dt=dt,
        apply_command=apply_topic_command,
    )


def _default_collection_duration_sec(route: object) -> float:
    """为最坏机械轮速减速和 0.5 s 零命令尾段预留整帧时长。"""
    route_duration = float(route.duration_s)
    route_end = math.ceil(route_duration * 10.0 - 1e-12) / 10.0
    model = get_robot_model("df_mid")
    stop_duration = math.ceil(
        model.max_drive_wheel_speed_rad_s
        / _COMMAND_ACCELERATION_RAD_S2
        * 10.0
        - 1e-12
    ) / 10.0
    duration = route_end + stop_duration + _ZERO_TAIL_SEC
    if duration > _MAX_SIMULATION_NS / 1_000_000_000.0:
        raise RuntimeError("canonical Golf collection exceeds 240 simulated seconds")
    return duration


def _collection_duration_ns(value: float) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("collection_duration_sec must be a positive 100 ms multiple")
    normalized = float(value)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError("collection_duration_sec must be a positive 100 ms multiple")
    duration_ns = round(normalized * 1_000_000_000.0)
    if (
        not math.isclose(normalized, duration_ns / 1_000_000_000.0, abs_tol=1e-12)
        or duration_ns % _FRAME_PERIOD_NS != 0
        or duration_ns > _MAX_SIMULATION_NS
    ):
        raise ValueError(
            "collection_duration_sec must be a positive 100 ms multiple no greater than 240 s"
        )
    return duration_ns


def expected_golf_topic_counts(
    collection_duration_sec: float | None = None,
) -> dict[str, int]:
    """返回 Recorder 对本次完整离线窗口应看到的五 topic 精确计数。"""
    if collection_duration_sec is None:
        route = build_canonical_golf_route(
            TerrainBounds(-10.01, 10.01, -6.65, 6.65)
        )
        collection_duration_sec = _default_collection_duration_sec(route)
    duration_ns = _collection_duration_ns(collection_duration_sec)
    wheel_count = duration_ns // 10_000_000
    lidar_count = duration_ns // _FRAME_PERIOD_NS
    return {
        "/sim/wheel/command": wheel_count,
        "/sim/wheel/state": wheel_count,
        "/sim/lidar/points": lidar_count,
        "/sim/rtk/state": lidar_count + 1,
        "/sim/imu/attitude": lidar_count + 1,
    }


def _wait_for_verified_peers(transport: object, *, timeout_sec: float) -> object:
    """等待 Command peer 与 Recorder 形成冻结的五话题拓扑。"""
    deadline = time.monotonic() + timeout_sec
    while True:
        poll = getattr(transport, "poll_peer_state", None)
        snapshot_method = getattr(transport, "snapshot", None)
        if not callable(poll) or not callable(snapshot_method):
            raise RuntimeError("v2 transport must provide discovery polling and snapshot")
        poll()
        snapshot = snapshot_method()
        qualities = {item.topic: item for item in snapshot.topic_quality}
        if all(
            topic in qualities
            and qualities[topic].protocol_state == "verified"
            and qualities[topic].peer_count == expected
            for topic, expected in _SIMULATOR_PEER_COUNTS.items()
        ):
            return snapshot
        if time.monotonic() >= deadline:
            raise TimeoutError("MID-360 Golf v2 peers did not become exactly verified")
        time.sleep(0.01)


def _capture_pose_models(
    controller: V2RuntimeProtocol,
    truth: Stage4TruthSensorSuite,
    timestamp_ns: int,
    *,
    identities: tuple[OutputIdentity, OutputIdentity],
) -> tuple[RtkStateV2, ImuAttitudeV2]:
    """把同一物理状态的三点 RTK/IMU 绑定到预留输出身份。"""
    rtk_identity, imu_identity = identities
    rtk = truth.read_rtk(timestamp_ns)
    imu = truth.read_imu(timestamp_ns)
    if not isinstance(rtk, Stage4RtkState) or not isinstance(imu, ImuAttitude):
        raise RuntimeError("truth sensor suite returned invalid Golf pose values")
    if rtk.timestamp_ns != timestamp_ns or imu.timestamp_ns != timestamp_ns:
        raise RuntimeError("Golf RTK/IMU timestamps do not match the frozen state")
    snapshot = controller.snapshot()
    if any(
        identity.simulation_session_id != snapshot.simulation_session_id
        or identity.descriptor_sha256 != snapshot.descriptor_sha256
        or identity.world_generation != snapshot.world_generation
        for identity in identities
    ):
        raise RuntimeError("Golf pose output identity changed during capture")
    return (
        RtkStateV2(
            timestamp_ns=timestamp_ns,
            sequence=rtk_identity.sequence,
            world_generation=rtk_identity.world_generation,
            frame_id="world",
            left=Point3dV2(*rtk.left),
            center=Point3dV2(*rtk.center),
            right=Point3dV2(*rtk.right),
            heading_rad=rtk.heading_rad,
            simulation_session_id=rtk_identity.simulation_session_id,
            descriptor_sha256=rtk_identity.descriptor_sha256,
        ),
        ImuAttitudeV2(
            timestamp_ns=timestamp_ns,
            roll_rad=imu.roll_rad,
            pitch_rad=imu.pitch_rad,
            sequence=imu_identity.sequence,
            world_generation=imu_identity.world_generation,
            frame_id="base_link",
            simulation_session_id=imu_identity.simulation_session_id,
            descriptor_sha256=imu_identity.descriptor_sha256,
        ),
    )


def _start_offline_frame(
    *,
    controller: V2RuntimeProtocol,
    truth: Stage4TruthSensorSuite,
    backend: PyBulletSensorBackend,
    schedule: OfflineMid360Schedule,
    scanner_factory: Callable[..., object],
    timebase_ns: int,
) -> _PendingOfflineFrame:
    identities = controller.reserve_outputs(
        ("/sim/lidar/points", "/sim/rtk/state", "/sim/imu/attitude")
    )
    lidar_identity, rtk_identity, imu_identity = identities
    rtk, imu = _capture_pose_models(
        controller,
        truth,
        timebase_ns,
        identities=(rtk_identity, imu_identity),
    )
    scanner = scanner_factory(
        backend,
        schedule,
        sequence=lidar_identity.sequence,
    )
    if not callable(getattr(scanner, "capture_step", None)) or not callable(
        getattr(scanner, "finalize", None)
    ):
        raise RuntimeError("offline scanner factory returned an invalid scanner")
    return _PendingOfflineFrame(
        timebase_ns,
        scanner,
        lidar_identity,
        rtk,
        imu,
    )


def _finish_offline_frame(
    pending: _PendingOfflineFrame,
) -> tuple[LidarPointCloudV2, OfflineMid360AcceptanceTruth]:
    finalize = getattr(pending.scanner, "finalize")
    cloud = finalize(timebase_ns=pending.timebase_ns)
    if not isinstance(cloud, LidarPointCloud):
        raise RuntimeError("offline MID-360 scanner returned an invalid point cloud")
    if cloud.timebase_ns != pending.timebase_ns:
        raise RuntimeError("offline MID-360 timebase changed during scanning")
    lidar = LidarPointCloudV2(
        timebase_ns=cloud.timebase_ns,
        frame_id=cloud.frame_id,
        point_num=cloud.point_num,
        lidar_id=cloud.lidar_id,
        points=tuple(
            LidarPointV2(
                point.offset_time_ns,
                point.x,
                point.y,
                point.z,
                point.reflectivity,
                point.tag,
                point.line,
            )
            for point in cloud.points
        ),
        sequence=pending.lidar_identity.sequence,
        world_generation=pending.lidar_identity.world_generation,
        simulation_session_id=pending.lidar_identity.simulation_session_id,
        descriptor_sha256=pending.lidar_identity.descriptor_sha256,
    )
    truth_method = getattr(pending.scanner, "acceptance_truth", None)
    if not callable(truth_method):
        raise RuntimeError("offline scanner must provide acceptance_truth()")
    truth = truth_method()
    if type(truth) is not OfflineMid360AcceptanceTruth:
        raise RuntimeError("offline scanner returned invalid acceptance truth")
    return lidar, truth


def _publish_models(
    *,
    transport: object,
    codec: V2ProtoCodec,
    frames: tuple[tuple[str, object, int], ...],
    wall_time: float,
) -> None:
    """发布已经预留身份的离线模型，不引入另一套序号或时间逻辑。"""
    for topic, model, timestamp_ns in frames:
        encoded = codec.encode(model)
        if transport.publish(
            topic,
            encoded.payload,
            encoded.type_name,
            timestamp_ns,
            wall_time=wall_time,
        ) is not True:
            raise RuntimeError(f"v2 transport rejected offline {topic} frame")


def _publish_frame_start_pose(
    *,
    pending: _PendingOfflineFrame,
    transport: object,
    codec: V2ProtoCodec,
    wall_time: float,
) -> None:
    """在扫描开始前公开 pose，使命令端始终能看到当前 10 Hz 节点。"""
    _publish_models(
        transport=transport,
        codec=codec,
        frames=(
            ("/sim/rtk/state", pending.rtk, pending.timebase_ns),
            ("/sim/imu/attitude", pending.imu, pending.timebase_ns),
        ),
        wall_time=wall_time,
    )


def _reserve_pose_lookahead(
    *,
    controller: V2RuntimeProtocol,
    truth: Stage4TruthSensorSuite,
    timestamp_ns: int,
) -> tuple[RtkStateV2, ImuAttitudeV2]:
    identities = controller.reserve_outputs(("/sim/rtk/state", "/sim/imu/attitude"))
    return _capture_pose_models(
        controller,
        truth,
        timestamp_ns,
        identities=identities,
    )


def _transport_factory_call(
    factory: Callable[[DescriptorIdentity], object] | None,
    descriptor: DescriptorIdentity,
) -> object:
    if factory is not None:
        return factory(descriptor)
    return create_v2_ecal_transport(
        descriptor=descriptor,
        participant_name="mid360-golf-simulator",
        role="simulation",
    )


def run_mid360_golf_simulation(
    *,
    connection_mode: int = p.GUI,
    collection_duration_sec: float | None = None,
    transport_factory: Callable[[DescriptorIdentity], object] | None = None,
    session_id_factory: Callable[[], bytes] | None = None,
    scanner_factory: Callable[..., object] = OfflineMid360FrameScanner,
    require_verified_peers: bool = True,
    peer_timeout_sec: float = _STARTUP_TIMEOUT_SEC,
    ready_path: Path | None = None,
    start_path: Path | None = None,
    result_path: Path | None = None,
    fault_path: Path | None = None,
) -> dict[str, object]:
    """运行固定 Golf 世界；机器耗时仅影响 wall_time，不改变任何消息时间。"""
    if connection_mode not in {p.GUI, p.DIRECT}:
        raise ValueError("connection_mode must be pybullet.GUI or pybullet.DIRECT")
    if not callable(scanner_factory):
        raise ValueError("scanner_factory must be callable")
    if not isinstance(require_verified_peers, bool):
        raise ValueError("require_verified_peers must be a bool")
    if (
        isinstance(peer_timeout_sec, bool)
        or not isinstance(peer_timeout_sec, (int, float))
        or not math.isfinite(float(peer_timeout_sec))
        or peer_timeout_sec <= 0.0
    ):
        raise ValueError("peer_timeout_sec must be positive")
    for name, path in (
        ("ready_path", ready_path),
        ("start_path", start_path),
        ("result_path", result_path),
        ("fault_path", fault_path),
    ):
        if path is not None and not isinstance(path, Path):
            raise ValueError(f"{name} must be None or a Path")
    coordination_paths = (ready_path, start_path, fault_path)
    if any(path is not None for path in coordination_paths) and not all(
        path is not None for path in coordination_paths
    ):
        raise ValueError("ready_path, start_path, and fault_path must be provided together")

    document = load_scene(_SCENE_PATH)
    preliminary_config = ExperimentConfig(
        mode="gui" if connection_mode == p.GUI else "direct",
        duration_sec=1.0,
        time_step=_PHYSICS_STEP_SEC,
        robot_model=document.robot_model,
        terrain_model=document.terrain.terrain_model,
        golf_seed=document.terrain.golf_seed,
        golf_relief=document.terrain.golf_relief,
        interface_enabled=False,
        dashboard_enabled=False,
    )
    planned_route = build_canonical_golf_route(
        TerrainBounds(-10.01, 10.01, -6.65, 6.65),
    )
    selected_duration = (
        _default_collection_duration_sec(planned_route)
        if collection_duration_sec is None
        else collection_duration_sec
    )
    duration_ns = _collection_duration_ns(selected_duration)
    physics_steps = duration_ns * 240 // 1_000_000_000
    if physics_steps * 1_000_000_000 != duration_ns * 240:
        raise RuntimeError("Golf duration does not contain an exact number of physics steps")
    config = ExperimentConfig(
        **{
            **preliminary_config.__dict__,
            "duration_sec": duration_ns / 1_000_000_000.0,
        }
    )

    client_id = p.connect(connection_mode)
    if client_id < 0:
        raise RuntimeError("failed to connect PyBullet")
    transport = None
    controller = None
    clean_shutdown = False
    fault_reason: str | None = None
    published = {
        "/sim/wheel/state": 0,
        "/sim/lidar/points": 0,
        "/sim/rtk/state": 0,
        "/sim/imu/attitude": 0,
    }
    active_command_steps = 0
    try:
        world, obstacle_manager = build_world_from_scene_document(
            client_id,
            config,
            document,
        )
        robot = world.active_robot.robot
        if world.active_robot.robot_model != "df_mid" or world.scene.bounds is None:
            raise RuntimeError("canonical Golf scene did not create df_mid with bounds")
        route = build_canonical_golf_route(
            world.scene.bounds,
            spawn_xy=(world.scene.spawn_position[0], world.scene.spawn_position[1]),
        )
        backend = PyBulletSensorBackend(client_id, robot.robot_id)
        snapshots = obstacle_manager.snapshot(include_body_id=True)
        backend.bind_scene(world.scene.body_ids, snapshots)
        acceptance = GolfCaptureAcceptance(snapshots)
        truth = Stage4TruthSensorSuite(backend, Stage4SensorMounts.default())
        descriptor = load_v2_descriptor()
        codec = V2ProtoCodec(descriptor)
        transport = _transport_factory_call(transport_factory, descriptor)
        controller = V2RuntimeProtocol(
            robot.model_spec,
            transport=transport,
            descriptor=descriptor,
            session_id_factory=session_id_factory,
        )
        receiver = OfflineWheelCommandReceiver(controller, descriptor)
        subscribe = getattr(transport, "subscribe", None)
        if not callable(subscribe):
            raise RuntimeError("v2 transport must provide subscribe")
        subscribe(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            lambda payload, received_at: receiver.accept_payload(
                payload,
                received_at=received_at,
            ),
        )
        if require_verified_peers:
            _wait_for_verified_peers(
                transport,
                timeout_sec=float(peer_timeout_sec),
            )
        controller.refresh_transport()
        startup_identity = controller.snapshot()
        if ready_path is not None:
            assert start_path is not None
            _atomic_write_json(
                ready_path,
                {
                    "role": "simulator",
                    "ready": True,
                    "simulation_session_id": startup_identity.simulation_session_id.hex(),
                    "descriptor_sha256": startup_identity.descriptor_sha256.hex(),
                    "world_generation": startup_identity.world_generation,
                },
            )
            _wait_for_start(start_path, timeout_sec=float(peer_timeout_sec))

        output_publisher = V2OutputFramePublisher(transport, descriptor)
        wheel_factory = V2WheelStateFactory(controller, "df_mid")
        cadence = V2PublishCadence()
        schedule = OfflineMid360Schedule(
            OfflineMid360Profile.high_fidelity(),
            MID360_PATTERN_VERSION,
            controller.snapshot().world_generation,
        )
        last_decision = controller.mailbox.decision(now=0.0)
        normal_stop_timestamp_ns = (
            math.ceil(route.duration_s * 10.0 - 1e-12) * _FRAME_PERIOD_NS
        )
        motion_eligibility = _MotionFrameEligibility(
            normal_stop_timestamp_ns,
            route.segments[0].target_speed_m_s,
        )

        def observe_motion_frame(timestamp_ns: int) -> None:
            base_position, _orientation = p.getBasePositionAndOrientation(
                robot.robot_id,
                physicsClientId=client_id,
            )
            linear_velocity, _angular_velocity = p.getBaseVelocity(
                robot.robot_id,
                physicsClientId=client_id,
            )
            projection = route.project(base_position[0], base_position[1])
            commanded_forward_speed = max(
                0.0,
                sum(last_decision.drive_wheel_speed_rad_s)
                * robot.model_spec.wheel_radius
                / 2.0,
            )
            acceptance.observe_motion_frame(
                eligible=motion_eligibility.observe(
                    timestamp_ns=timestamp_ns,
                    segment_kind=route.segments[projection.segment_index].kind,
                    commanded_forward_speed_m_s=commanded_forward_speed,
                ),
                actual_speed_m_s=math.hypot(*linear_velocity),
            )

        pending = _start_offline_frame(
            controller=controller,
            truth=truth,
            backend=backend,
            schedule=schedule,
            scanner_factory=scanner_factory,
            timebase_ns=0,
        )
        observe_motion_frame(0)
        _publish_frame_start_pose(
            pending=pending,
            transport=transport,
            codec=codec,
            wall_time=time.monotonic(),
        )
        published["/sim/rtk/state"] += 1
        published["/sim/imu/attitude"] += 1
        safety = GolfSafetyMonitor(world.scene.bounds)
        obstacle_body_ids = {
            snapshot.body_id for snapshot in snapshots if snapshot.body_id is not None
        }
        completed_steps = 0
        for physics_step in range(physics_steps):
            if p.isConnected(client_id) == 0:
                fault_reason = "PyBullet GUI was closed"
                break
            simulation_time_ns = round(
                physics_step * 1_000_000_000 / 240
            )
            marker_fault = _fault_from_marker(fault_path)
            if marker_fault is not None and fault_reason is None:
                fault_reason = marker_fault
            applied: list[object] = []

            def apply_topic_command() -> None:
                nonlocal active_command_steps, last_decision
                decision = controller.mailbox.decision(
                    now=simulation_time_ns / 1_000_000_000.0
                )
                if fault_reason is not None or receiver.fault_reason is not None:
                    drive = (0.0, 0.0)
                    steering = ()
                else:
                    drive = decision.drive_wheel_speed_rad_s
                    steering = decision.steering_wheel_speed_rad_s
                    if not decision.waiting and not decision.timed_out and any(drive):
                        active_command_steps += 1
                robot.command_wheel_speeds(drive, steering, dt=config.time_step)
                last_decision = decision
                applied.append(decision)

            execute_offline_golf_step(
                scanner=pending.scanner,
                scanner_step=physics_step % 24,
                client_id=client_id,
                obstacle_manager=obstacle_manager,
                dt=config.time_step,
                apply_topic_command=apply_topic_command,
            )
            if len(applied) != 1:
                raise RuntimeError("Golf physics step did not apply exactly one topic decision")
            completed_steps += 1
            batch = cadence.advance(config.time_step)
            wall_time = time.monotonic()
            for timestamp_ns in batch.wheel_timestamps_ns:
                feedback = robot.read_interface_wheel_state(timestamp_ns)
                output_publisher.publish_wheel_state(
                    wheel_factory.build(feedback),
                    wall_time=wall_time,
                )
                published["/sim/wheel/state"] += 1
                if start_path is not None:
                    receiver.wait_for_command(
                        timestamp_ns,
                        timeout_sec=_COMMAND_RESPONSE_TIMEOUT_SEC,
                    )

            for timestamp_ns in batch.sensor_timestamps_ns:
                expected = pending.timebase_ns + _FRAME_PERIOD_NS
                if timestamp_ns != expected:
                    raise RuntimeError("offline MID-360 cadence diverged from its 24 steps")
                completed_pending = pending
                lidar, acceptance_truth = _finish_offline_frame(completed_pending)
                next_pending: _PendingOfflineFrame | None = None
                if timestamp_ns < duration_ns:
                    next_pending = _start_offline_frame(
                        controller=controller,
                        truth=truth,
                        backend=backend,
                        schedule=schedule,
                        scanner_factory=scanner_factory,
                        timebase_ns=timestamp_ns,
                    )
                    lookahead_rtk, lookahead_imu = next_pending.rtk, next_pending.imu
                else:
                    lookahead_rtk, lookahead_imu = _reserve_pose_lookahead(
                        controller=controller,
                        truth=truth,
                        timestamp_ns=timestamp_ns,
                    )
                start_node = recover_pose_node(
                    completed_pending.rtk,
                    completed_pending.imu,
                )
                lookahead_node = recover_pose_node(
                    lookahead_rtk,
                    lookahead_imu,
                    previous_orientation=start_node.base_pose.orientation,
                )
                acceptance.observe_lidar_frame(
                    cloud=lidar,
                    truth=acceptance_truth,
                    start=start_node,
                    lookahead=lookahead_node,
                    obstacle_snapshots=obstacle_manager.snapshot(include_body_id=True),
                )
                _publish_models(
                    transport=transport,
                    codec=codec,
                    frames=(("/sim/lidar/points", lidar, lidar.timebase_ns),),
                    wall_time=wall_time,
                )
                published["/sim/lidar/points"] += 1
                if next_pending is not None:
                    pending = next_pending
                    _publish_frame_start_pose(
                        pending=pending,
                        transport=transport,
                        codec=codec,
                        wall_time=wall_time,
                    )
                    observe_motion_frame(timestamp_ns)
                else:
                    _publish_models(
                        transport=transport,
                        codec=codec,
                        frames=(
                            ("/sim/rtk/state", lookahead_rtk, timestamp_ns),
                            ("/sim/imu/attitude", lookahead_imu, timestamp_ns),
                        ),
                        wall_time=wall_time,
                    )
                published["/sim/rtk/state"] += 1
                published["/sim/imu/attitude"] += 1

            base_position, _orientation = p.getBasePositionAndOrientation(
                robot.robot_id,
                physicsClientId=client_id,
            )
            linear_velocity, _angular_velocity = p.getBaseVelocity(
                robot.robot_id,
                physicsClientId=client_id,
            )
            wheel_speeds = robot.read_drive_wheel_speeds()
            projection = route.project(base_position[0], base_position[1])
            obstacle_collision = bool(
                obstacle_contact_body_ids(
                    client_id,
                    robot.robot_id,
                    obstacle_body_ids,
                )
            )
            safety_decision = safety.update(
                sim_time_s=completed_steps * config.time_step,
                x=base_position[0],
                y=base_position[1],
                base_speed_m_s=math.hypot(*linear_velocity),
                drive_wheel_speeds=wheel_speeds,
                route_error_m=projection.distance_m,
                commanded_forward_speed_m_s=max(
                    0.0,
                    sum(last_decision.drive_wheel_speed_rad_s)
                    * robot.model_spec.wheel_radius
                    / 2.0,
                ),
                obstacle_collision=obstacle_collision,
                recorder_fault=(
                    f"command fault: {receiver.fault_reason}"
                    if receiver.fault_reason is not None
                    else None
                ),
            )
            if safety_decision.faulted and fault_reason is None:
                fault_reason = safety_decision.fault_reason
            if fault_reason is not None and safety_decision.settled:
                break
            if physics_step % 12 == 11:
                transport_snapshot = controller.refresh_transport()
                if transport_snapshot.error_count or transport_snapshot.dropped_count:
                    fault_reason = fault_reason or "v2 transport reported dropped/error frames"

        robot.hold_current_steering_and_stop_drive(config.time_step)
        if fault_reason is None and completed_steps != physics_steps:
            raise RuntimeError("Golf simulation ended before its planned physics window")
        wait_idle = getattr(transport, "wait_idle", None)
        if not callable(wait_idle):
            raise RuntimeError("v2 transport must provide wait_idle")
        wait_idle(timeout_sec=_TRANSPORT_IDLE_TIMEOUT_SEC)
        transport_snapshot = controller.refresh_transport()
        if transport_snapshot.error_count or transport_snapshot.dropped_count:
            fault_reason = fault_reason or "v2 transport reported dropped/error frames"
        clean_shutdown = fault_reason is None
        final_identity = controller.snapshot()
        expected_counts = expected_golf_topic_counts(
            duration_ns / 1_000_000_000.0
        )
        if clean_shutdown and published != {
            topic: expected_counts[topic] for topic in published
        }:
            raise RuntimeError("Golf simulator output counts differ from the frozen plan")
        capture_metrics = acceptance.snapshot()
        result: dict[str, object] = {
            "role": "simulator",
            "clean_shutdown": clean_shutdown,
            "fault_reason": fault_reason,
            "simulation_session_id": final_identity.simulation_session_id.hex(),
            "descriptor_sha256": final_identity.descriptor_sha256.hex(),
            "world_generation": final_identity.world_generation,
            "command_generation": final_identity.command_generation,
            "scene_id": _SCENE_ID,
            "client_id": client_id,
            "connection_mode": "gui" if connection_mode == p.GUI else "direct",
            "physics_steps": completed_steps,
            "sim_duration_ns": round(completed_steps * 1_000_000_000 / 240),
            "robot_model": document.robot_model,
            "terrain_model": document.terrain.terrain_model,
            "golf_seed": document.terrain.golf_seed,
            "golf_relief": document.terrain.golf_relief,
            "static_obstacle_count": sum(
                obstacle.mode == "static" for obstacle in document.obstacles
            ),
            "moving_obstacle_count": sum(
                obstacle.mode == "moving" for obstacle in document.obstacles
            ),
            "published_frames": published,
            "expected_topic_counts": expected_counts,
            "active_command_steps": active_command_steps,
            "truth_acceptance": capture_acceptance_document(capture_metrics),
            "transport_metrics": {
                "published_count": transport_snapshot.published_count,
                "received_count": transport_snapshot.received_count,
                "error_count": transport_snapshot.error_count,
                "dropped_count": transport_snapshot.dropped_count,
            },
        }
        if result_path is not None:
            _atomic_write_json(result_path, result)
        controller.close()
        controller = None
        transport = None
        return result
    finally:
        if controller is not None:
            controller.close()
        elif transport is not None:
            close = getattr(transport, "close", None)
            if callable(close):
                close()
        if p.isConnected(client_id):
            p.disconnect(client_id)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--direct", action="store_true", help="use DIRECT for automation")
    parser.add_argument("--ready-path", type=Path, required=True)
    parser.add_argument("--start-path", type=Path, required=True)
    parser.add_argument("--result-path", type=Path, required=True)
    parser.add_argument("--fault-path", type=Path, required=True)
    parser.add_argument("--simulation-session-id", required=True)
    parser.add_argument("--peer-timeout-sec", type=float, default=_STARTUP_TIMEOUT_SEC)
    arguments = parser.parse_args(argv)
    try:
        simulation_session_id = bytes.fromhex(arguments.simulation_session_id)
    except ValueError as error:
        parser.error(f"--simulation-session-id must be 32 lowercase hex digits: {error}")
    if (
        len(simulation_session_id) != 16
        or simulation_session_id.hex() != arguments.simulation_session_id
    ):
        parser.error("--simulation-session-id must be 32 lowercase hex digits")
    result = run_mid360_golf_simulation(
        connection_mode=p.DIRECT if arguments.direct else p.GUI,
        session_id_factory=lambda: simulation_session_id,
        peer_timeout_sec=arguments.peer_timeout_sec,
        ready_path=arguments.ready_path,
        start_path=arguments.start_path,
        result_path=arguments.result_path,
        fault_path=arguments.fault_path,
    )
    return 0 if result["clean_shutdown"] is True else 1


if __name__ == "__main__":
    raise SystemExit(main())
