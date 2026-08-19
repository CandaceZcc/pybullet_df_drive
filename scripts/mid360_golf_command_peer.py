#!/usr/bin/env python3
"""Golf 离线路线唯一命令端：由 v2 仿真消息生成身份绑定轮速命令。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
from threading import RLock
import time
from uuid import uuid4


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity, load_v2_descriptor
from slope_sim.interfaces.v2.models import (
    CommandAuthorityState,
    ImuAttitudeV2,
    RtkStateV2,
    WheelCommandV2,
    WheelStateV2,
    require_fixed_bytes,
)
from slope_sim.interfaces.v2.transport import create_v2_ecal_transport
from slope_sim.mapping_replay import RecoveredPoseNode, recover_pose_node
from slope_sim.mid360_golf_drive import GolfRoute, GolfRouteController, build_canonical_golf_route
from slope_sim.model_registry import get_robot_model
from slope_sim.scene import TerrainBounds


_WHEEL_PERIOD_NS = 10_000_000
_POSE_PERIOD_NS = 100_000_000
_ZERO_TAIL_NS = 500_000_000
_MAX_WHEEL_ACCELERATION_RAD_S2 = 20.0
_MAX_STOP_DURATION_NS = (
    math.ceil(
        get_robot_model("df_mid").max_drive_wheel_speed_rad_s
        / _MAX_WHEEL_ACCELERATION_RAD_S2
        * 10.0
        - 1e-12
    )
    * _POSE_PERIOD_NS
)
_POSE_CALLBACK_GRACE_SEC = 0.5
_ROUTE_PROGRESS_LAG_LIMIT_M = 0.75
_ROUTE_PROGRESS_LAG_DURATION_NS = 1_000_000_000
_SOURCE_ID = "mid360.golf.command-peer"
_CANONICAL_BOUNDS = TerrainBounds(-10.01, 10.01, -6.65, 6.65)
_STARTUP_TIMEOUT_SEC = 5.0
_PEER_COUNTS = {
    "/sim/wheel/command": 2,
    "/sim/wheel/state": 1,
    "/sim/lidar/points": 1,
    "/sim/rtk/state": 1,
    "/sim/imu/attitude": 1,
}


def _atomic_write_json(path: Path, document: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(document, sort_keys=True, separators=(",", ":")) + "\n",
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


def _fault_from_marker(path: Path) -> str | None:
    value = _read_exact_marker(path, key="fault")
    if value is None:
        return None
    if not isinstance(value, str) or not value:
        raise RuntimeError("fault marker must contain a nonempty reason")
    return value


@dataclass(frozen=True, slots=True)
class GolfCommandPeerSnapshot:
    """供进程编排读取的有界命令端状态。"""

    published_count: int
    last_wheel_timestamp_ns: int | None
    latest_pose_timestamp_ns: int | None
    fault_reason: str | None
    normal_stop_started: bool
    finished: bool


@dataclass(slots=True)
class _StreamProgress:
    """记录一个输入 topic 的严格 sequence 与冻结仿真节拍。"""

    label: str
    period_ns: int
    expected_sequence: int = 0
    last_timestamp_ns: int | None = None

    def accept(self, *, sequence: int, timestamp_ns: int) -> str | None:
        if sequence != self.expected_sequence:
            return f"{self.label} sequence must start at zero and remain continuous"
        if (
            self.last_timestamp_ns is not None
            and timestamp_ns != self.last_timestamp_ns + self.period_ns
        ):
            return f"{self.label} timestamps must follow the frozen cadence"
        self.expected_sequence += 1
        self.last_timestamp_ns = timestamp_ns
        return None


def _yaw_from_pose(node: RecoveredPoseNode) -> float:
    """从已恢复的单位四元数读取 ZYX yaw。"""
    x, y, z, w = node.base_pose.orientation
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _toward_zero(value: float, maximum_change: float) -> float:
    if abs(value) <= maximum_change:
        return 0.0
    return value - math.copysign(maximum_change, value)


def _scheduled_route_distance_m(route: GolfRoute, timestamp_ns: int) -> float:
    """按冻结分段速度把仿真时间映射为应达到的累计里程。"""
    remaining_s = timestamp_ns / 1_000_000_000.0
    distance_m = 0.0
    for segment in route.segments:
        segment_duration_s = segment.length / segment.target_speed_m_s
        if remaining_s < segment_duration_s:
            return distance_m + remaining_s * segment.target_speed_m_s
        distance_m += segment.length
        remaining_s -= segment_duration_s
    return route.length


class GolfCommandPeer:
    """串行校验三条反馈流，并由每条 WheelState 唯一触发一条命令。"""

    def __init__(
        self,
        *,
        transport: object,
        descriptor: DescriptorIdentity,
        route: GolfRoute,
        source_session_id: bytes | None = None,
        source_id: str = _SOURCE_ID,
    ) -> None:
        if not callable(getattr(transport, "subscribe", None)) or not callable(
            getattr(transport, "publish", None)
        ):
            raise ValueError("transport must provide subscribe and publish")
        if not isinstance(descriptor, DescriptorIdentity):
            raise ValueError("descriptor must be a DescriptorIdentity")
        if not isinstance(route, GolfRoute):
            raise ValueError("route must be a GolfRoute")
        if not isinstance(source_id, str) or not source_id:
            raise ValueError("source_id must be nonempty")
        self._transport = transport
        self._descriptor = descriptor
        self._codec = V2ProtoCodec(descriptor)
        self._route = route
        self._controller = GolfRouteController(route)
        self._source_id = source_id
        self._source_session_id = require_fixed_bytes(
            "source_session_id",
            uuid4().bytes if source_session_id is None else source_session_id,
            16,
        )
        self._normal_stop_timestamp_ns = (
            math.ceil(route.duration_s * 10.0 - 1e-12) * _POSE_PERIOD_NS
        )
        self._normal_finish_timestamp_ns = (
            self._normal_stop_timestamp_ns + _MAX_STOP_DURATION_NS + _ZERO_TAIL_NS
        )
        self._lock = RLock()
        self._fault_reason: str | None = None
        self._base_identity: tuple[bytes, int] | None = None
        self._command_identity: tuple[bytes, int, int, str] | None = None
        self._wheel_progress = _StreamProgress("WheelState", _WHEEL_PERIOD_NS)
        self._rtk_progress = _StreamProgress("RTK", _POSE_PERIOD_NS)
        self._imu_progress = _StreamProgress("IMU", _POSE_PERIOD_NS)
        self._pending_rtk: dict[int, RtkStateV2] = {}
        self._pending_imu: dict[int, ImuAttitudeV2] = {}
        self._poses: dict[int, RecoveredPoseNode] = {}
        self._previous_orientation = None
        self._next_command_sequence = 0
        self._published_count = 0
        self._last_wheel_timestamp_ns: int | None = None
        self._latest_pose_timestamp_ns: int | None = None
        self._latest_pose_speed_m_s = 0.0
        self._last_command_speeds = (0.0, 0.0)
        self._route_progress_lag_since_ns: int | None = None
        self._pending_wheel: tuple[WheelStateV2, float] | None = None
        self._normal_zero_since_ns: int | None = None
        self._fault_quiet_wheel_states = 0
        self._finished = False
        self._subscriptions = (
            transport.subscribe(
                "/sim/wheel/state",
                "slope_sim.interfaces.v2.WheelState",
                self._on_wheel_payload,
            ),
            transport.subscribe(
                "/sim/rtk/state",
                "slope_sim.interfaces.v2.RtkState",
                self._on_rtk_payload,
            ),
            transport.subscribe(
                "/sim/imu/attitude",
                "slope_sim.interfaces.v2.ImuAttitude",
                self._on_imu_payload,
            ),
        )

    @property
    def fault_reason(self) -> str | None:
        with self._lock:
            return self._fault_reason

    @property
    def finished(self) -> bool:
        with self._lock:
            return self._finished

    def snapshot(self) -> GolfCommandPeerSnapshot:
        with self._lock:
            return GolfCommandPeerSnapshot(
                published_count=self._published_count,
                last_wheel_timestamp_ns=self._last_wheel_timestamp_ns,
                latest_pose_timestamp_ns=self._latest_pose_timestamp_ns,
                fault_reason=self._fault_reason,
                normal_stop_started=self._last_wheel_timestamp_ns is not None
                and self._last_wheel_timestamp_ns >= self._normal_stop_timestamp_ns,
                finished=self._finished,
            )

    def latch_fault(self, reason: str) -> None:
        """锁存首个外部故障；之后所有可验证 WheelState 只触发零命令。"""
        if not isinstance(reason, str) or not reason:
            raise ValueError("fault reason must be nonempty")
        with self._lock:
            if self._fault_reason is None:
                self._fault_reason = reason

    def _accept_base_identity(self, model: object, label: str) -> bool:
        identity = (model.simulation_session_id, model.world_generation)
        if model.descriptor_sha256 != self._descriptor.sha256:
            self.latch_fault(f"{label} descriptor does not match")
            return False
        if self._base_identity is None:
            self._base_identity = identity
            return True
        if identity != self._base_identity:
            self.latch_fault(f"{label} simulation identity changed")
            return False
        return True

    def _on_rtk_payload(self, payload: bytes, _received_at: float) -> None:
        try:
            message = self._codec.decode_rtk_state(payload)
        except (TypeError, ValueError) as error:
            self.latch_fault(f"RTK payload is invalid: {error}")
            return
        with self._lock:
            self._accept_pose_message(message, is_rtk=True)

    def _on_imu_payload(self, payload: bytes, _received_at: float) -> None:
        try:
            message = self._codec.decode_imu_attitude(payload)
        except (TypeError, ValueError) as error:
            self.latch_fault(f"IMU payload is invalid: {error}")
            return
        with self._lock:
            self._accept_pose_message(message, is_rtk=False)

    def _accept_pose_message(
        self,
        message: RtkStateV2 | ImuAttitudeV2,
        *,
        is_rtk: bool,
    ) -> None:
        label = "RTK" if is_rtk else "IMU"
        progress = self._rtk_progress if is_rtk else self._imu_progress
        reason = progress.accept(
            sequence=message.sequence,
            timestamp_ns=message.timestamp_ns,
        )
        if reason is not None:
            self.latch_fault(reason)
            return
        if not self._accept_base_identity(message, label):
            return
        pending = self._pending_rtk if is_rtk else self._pending_imu
        pending[message.timestamp_ns] = message
        self._complete_pose_pair(message.timestamp_ns)

    def _complete_pose_pair(self, timestamp_ns: int) -> None:
        rtk = self._pending_rtk.get(timestamp_ns)
        imu = self._pending_imu.get(timestamp_ns)
        if rtk is None or imu is None:
            return
        del self._pending_rtk[timestamp_ns]
        del self._pending_imu[timestamp_ns]
        try:
            node = recover_pose_node(
                rtk,
                imu,
                previous_orientation=self._previous_orientation,
            )
        except ValueError as error:
            self.latch_fault(f"RTK/IMU pose is invalid: {error}")
            return
        self._previous_orientation = node.base_pose.orientation
        if self._latest_pose_timestamp_ns is not None:
            previous = self._poses[self._latest_pose_timestamp_ns]
            elapsed_s = (timestamp_ns - previous.timestamp_ns) / 1_000_000_000.0
            if elapsed_s <= 0.0:
                self.latch_fault("RTK/IMU pose time did not advance")
                return
            self._latest_pose_speed_m_s = math.dist(
                previous.base_pose.position,
                node.base_pose.position,
            ) / elapsed_s
        self._poses[timestamp_ns] = node
        self._latest_pose_timestamp_ns = timestamp_ns
        while len(self._poses) > 4:
            del self._poses[min(self._poses)]
        self._dispatch_pending_wheel_if_ready_locked()
        self._maybe_finish_normal_locked()

    def _on_wheel_payload(self, payload: bytes, received_at: float) -> None:
        try:
            state = self._codec.decode_wheel_state(payload)
        except (TypeError, ValueError) as error:
            self.latch_fault(f"WheelState payload is invalid: {error}")
            return
        with self._lock:
            if not self._accept_wheel_state(state):
                return
            if self._pending_wheel is not None:
                self.latch_fault("WheelState advanced while RTK/IMU pose was pending")
                self._dispatch_pending_wheel_if_ready_locked()
                self._publish_command_locked(state, wall_time=received_at)
                return
            if self._wheel_needs_pose_locked(state):
                self._pending_wheel = (state, received_at)
                return
            self._publish_command_locked(state, wall_time=received_at)

    def _wheel_needs_pose_locked(self, state: WheelStateV2) -> bool:
        if self._fault_reason is not None or state.timestamp_ns > self._normal_stop_timestamp_ns:
            return False
        pose = self._latest_pose_at(state.timestamp_ns)
        return pose is None or state.timestamp_ns - pose.timestamp_ns > _POSE_PERIOD_NS

    def _publish_command_locked(self, state: WheelStateV2, *, wall_time: float) -> None:
        speeds = self._command_speeds(state)
        command = WheelCommandV2(
            timestamp_ns=state.timestamp_ns,
            drive_wheel_speed_rad_s=speeds,
            steering_wheel_speed_rad_s=(),
            sequence=self._next_command_sequence,
            world_generation=state.world_generation,
            command_generation=state.command_generation,
            source_id=self._source_id,
            source_session_id=self._source_session_id,
            robot_model=state.robot_model,
            simulation_session_id=state.simulation_session_id,
            descriptor_sha256=state.descriptor_sha256,
        )
        self._next_command_sequence += 1
        encoded = self._codec.encode(command)
        try:
            published = self._transport.publish(
                "/sim/wheel/command",
                encoded.payload,
                encoded.type_name,
                command.timestamp_ns,
                wall_time=wall_time,
            )
        except (RuntimeError, ValueError) as error:
            self.latch_fault(f"WheelCommand publish failed: {error}")
            return
        if published is not True:
            self.latch_fault("WheelCommand transport rejected the frame")
            return
        self._published_count += 1
        self._last_command_speeds = speeds
        self._update_completion(state, speeds)

    def _dispatch_pending_wheel_if_ready_locked(self) -> None:
        if self._pending_wheel is None:
            return
        state, received_at = self._pending_wheel
        if self._wheel_needs_pose_locked(state):
            return
        self._pending_wheel = None
        self._publish_command_locked(state, wall_time=received_at)

    def service_pending_wheel(self, *, now: float) -> None:
        """等待跨 topic 回调配对；超时则以零命令释放 Simulator。"""
        if (
            isinstance(now, bool)
            or not isinstance(now, (int, float))
            or not math.isfinite(float(now))
        ):
            raise ValueError("now must be finite")
        normalized_now = float(now)
        with self._lock:
            if self._pending_wheel is None:
                return
            _state, received_at = self._pending_wheel
            age = normalized_now - received_at
            if age < 0.0:
                # 主循环可能先采样 now，随后 subscriber 才在本锁前安装 pending。
                return
            if (
                self._fault_reason is None
                and age >= _POSE_CALLBACK_GRACE_SEC
                and self._wheel_needs_pose_locked(_state)
            ):
                self.latch_fault("RTK/IMU pose is stale")
            self._dispatch_pending_wheel_if_ready_locked()

    def _accept_wheel_state(self, state: WheelStateV2) -> bool:
        reason = self._wheel_progress.accept(
            sequence=state.sequence,
            timestamp_ns=state.timestamp_ns,
        )
        if reason is not None:
            self.latch_fault(reason)
            return False
        self._last_wheel_timestamp_ns = state.timestamp_ns
        self._accept_base_identity(state, "WheelState")
        if (
            state.robot_model != "df_mid"
            or len(state.drive_wheel_speed_rad_s) != 2
            or state.steering_wheel_angle_rad
        ):
            self.latch_fault("WheelState does not match the canonical df_mid wheel order")
        if (
            state.command_peer_count != 1
            or state.command_authority_state
            not in {CommandAuthorityState.CLAIMABLE, CommandAuthorityState.ACTIVE}
        ):
            self.latch_fault("WheelState does not report one command peer")
        if (
            state.command_authority_state is CommandAuthorityState.ACTIVE
            and (
                state.command_owner_source_id != self._source_id
                or state.command_owner_source_session_id != self._source_session_id
            )
        ):
            self.latch_fault("WheelState command owner differs from this peer")
        identity = (
            state.simulation_session_id,
            state.world_generation,
            state.command_generation,
            state.robot_model,
        )
        if self._command_identity is None:
            self._command_identity = identity
        elif identity != self._command_identity:
            self.latch_fault("WheelState command identity changed")
        return True

    def _latest_pose_at(self, timestamp_ns: int) -> RecoveredPoseNode | None:
        eligible = [value for key, value in self._poses.items() if key <= timestamp_ns]
        if not eligible:
            return None
        return max(eligible, key=lambda item: item.timestamp_ns)

    def _command_speeds(self, state: WheelStateV2) -> tuple[float, float]:
        if self._fault_reason is not None:
            return (0.0, 0.0)
        if state.timestamp_ns > self._normal_stop_timestamp_ns:
            maximum_change = (
                _MAX_WHEEL_ACCELERATION_RAD_S2 * _WHEEL_PERIOD_NS / 1_000_000_000.0
            )
            return tuple(
                _toward_zero(speed, maximum_change)
                for speed in self._last_command_speeds
            )  # type: ignore[return-value]
        pose = self._latest_pose_at(state.timestamp_ns)
        if pose is None:
            if state.timestamp_ns > _POSE_PERIOD_NS:
                self.latch_fault("RTK/IMU pose is missing")
            return (0.0, 0.0)
        if state.timestamp_ns - pose.timestamp_ns > _POSE_PERIOD_NS:
            self.latch_fault("RTK/IMU pose is stale")
            return (0.0, 0.0)
        command = self._controller.update(
            timestamp_ns=state.timestamp_ns,
            x=pose.base_pose.position[0],
            y=pose.base_pose.position[1],
            yaw=_yaw_from_pose(pose),
        )
        if command is None:
            self.latch_fault("route controller skipped a valid WheelState timestamp")
            return (0.0, 0.0)
        self._observe_route_progress(
            timestamp_ns=state.timestamp_ns,
            route_distance_m=command.projection.route_distance_m,
        )
        if self._fault_reason is not None:
            return (0.0, 0.0)
        if state.timestamp_ns == self._normal_stop_timestamp_ns:
            maximum_change = (
                _MAX_WHEEL_ACCELERATION_RAD_S2 * _WHEEL_PERIOD_NS / 1_000_000_000.0
            )
            return tuple(
                _toward_zero(speed, maximum_change)
                for speed in self._last_command_speeds
            )  # type: ignore[return-value]
        return command.drive_wheel_speeds

    def _observe_route_progress(
        self,
        *,
        timestamp_ns: int,
        route_distance_m: float,
    ) -> None:
        """持续落后或到期未完成时锁存首个路线进度故障。"""
        remaining_m = self._route.length - route_distance_m
        if (
            timestamp_ns >= self._normal_stop_timestamp_ns
            and remaining_m > _ROUTE_PROGRESS_LAG_LIMIT_M
        ):
            self.latch_fault("route_progress_deadline")
            return
        lag_m = _scheduled_route_distance_m(self._route, timestamp_ns) - route_distance_m
        if lag_m <= _ROUTE_PROGRESS_LAG_LIMIT_M:
            self._route_progress_lag_since_ns = None
            return
        if self._route_progress_lag_since_ns is None:
            self._route_progress_lag_since_ns = timestamp_ns
            return
        if timestamp_ns - self._route_progress_lag_since_ns >= _ROUTE_PROGRESS_LAG_DURATION_NS:
            self.latch_fault("route_progress_lag")

    def _update_completion(
        self,
        state: WheelStateV2,
        speeds: tuple[float, float],
    ) -> None:
        if self._fault_reason is not None:
            quiet = (
                self._latest_pose_speed_m_s < 0.02
                and all(abs(speed) < 0.1 for speed in state.drive_wheel_speed_rad_s)
            )
            self._fault_quiet_wheel_states = (
                self._fault_quiet_wheel_states + 1 if quiet else 0
            )
            self._finished = self._fault_quiet_wheel_states >= 20
            return
        timestamp_ns = state.timestamp_ns
        if timestamp_ns < self._normal_stop_timestamp_ns:
            return
        if speeds == (0.0, 0.0):
            if self._normal_zero_since_ns is None:
                self._normal_zero_since_ns = timestamp_ns
        else:
            self._normal_zero_since_ns = None
        self._maybe_finish_normal_locked()

    def _maybe_finish_normal_locked(self) -> None:
        if (
            self._fault_reason is not None
            or self._normal_zero_since_ns is None
            or self._last_wheel_timestamp_ns is None
            or self._latest_pose_timestamp_ns is None
        ):
            return
        self._finished = (
            self._last_wheel_timestamp_ns >= self._normal_finish_timestamp_ns
            and self._latest_pose_timestamp_ns >= self._normal_finish_timestamp_ns
            and self._last_wheel_timestamp_ns - self._normal_zero_since_ns
            >= _ZERO_TAIL_NS
        )

    def close(self) -> None:
        """撤销逻辑订阅；transport 生命周期仍由进程入口统一拥有。"""
        for subscription in self._subscriptions:
            close = getattr(subscription, "close", None)
            if callable(close):
                close()


def _wait_for_verified_peers(transport: object, *, timeout_sec: float) -> object:
    deadline = time.monotonic() + timeout_sec
    while True:
        poll = getattr(transport, "poll_peer_state", None)
        snapshot_method = getattr(transport, "snapshot", None)
        if not callable(poll) or not callable(snapshot_method):
            raise RuntimeError("v2 peer transport must provide discovery polling and snapshot")
        poll()
        snapshot = snapshot_method()
        qualities = {item.topic: item for item in snapshot.topic_quality}
        if all(
            topic in qualities
            and qualities[topic].protocol_state == "verified"
            and qualities[topic].peer_count == expected
            for topic, expected in _PEER_COUNTS.items()
        ):
            return snapshot
        if time.monotonic() >= deadline:
            raise TimeoutError("MID-360 Golf peer topology did not become exactly verified")
        time.sleep(0.01)


def run_command_peer(
    *,
    source_session_id: bytes | None = None,
    ready_path: Path,
    start_path: Path,
    result_path: Path,
    fault_path: Path,
    peer_timeout_sec: float = _STARTUP_TIMEOUT_SEC,
) -> dict[str, object]:
    """运行真实 peer，直至固定路线尾段或故障静止条件完成。"""
    if any(
        not isinstance(path, Path)
        for path in (ready_path, start_path, result_path, fault_path)
    ):
        raise ValueError("coordination and result paths must be Path values")
    if (
        isinstance(peer_timeout_sec, bool)
        or not isinstance(peer_timeout_sec, (int, float))
        or not math.isfinite(float(peer_timeout_sec))
        or peer_timeout_sec <= 0.0
    ):
        raise ValueError("peer_timeout_sec must be positive")
    descriptor = load_v2_descriptor()
    transport = create_v2_ecal_transport(
        descriptor=descriptor,
        participant_name="mid360-golf-command-peer",
        role="peer",
    )
    peer = GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(_CANONICAL_BOUNDS),
        source_session_id=source_session_id,
    )
    try:
        _wait_for_verified_peers(transport, timeout_sec=float(peer_timeout_sec))
        _atomic_write_json(
            ready_path,
            {
                "role": "command_peer",
                "ready": True,
                "source_id": peer._source_id,
                "source_session_id": peer._source_session_id.hex(),
                "descriptor_sha256": descriptor.sha256.hex(),
            },
        )
        _wait_for_start(start_path, timeout_sec=float(peer_timeout_sec))
        while not peer.finished:
            marker_fault = _fault_from_marker(fault_path)
            if marker_fault is not None:
                peer.latch_fault(marker_fault)
            poll = getattr(transport, "poll_peer_state", None)
            if not callable(poll):
                raise RuntimeError("v2 peer transport must provide poll_peer_state")
            poll()
            transport_snapshot = transport.snapshot()
            if transport_snapshot.error_count or transport_snapshot.dropped_count:
                peer.latch_fault("v2 transport reported dropped/error frames")
            peer.service_pending_wheel(now=time.monotonic())
            time.sleep(0.01)
        wait_idle = getattr(transport, "wait_idle", None)
        if callable(wait_idle):
            wait_idle(timeout_sec=2.0)
        snapshot = peer.snapshot()
        result: dict[str, object] = {
            "role": "command_peer",
            "clean_shutdown": snapshot.fault_reason is None and snapshot.finished,
            "fault_reason": snapshot.fault_reason,
            "source_id": peer._source_id,
            "source_session_id": peer._source_session_id.hex(),
            "descriptor_sha256": descriptor.sha256.hex(),
            "published_frames": {
                "/sim/wheel/command": snapshot.published_count,
            },
            "last_wheel_timestamp_ns": snapshot.last_wheel_timestamp_ns,
            "latest_pose_timestamp_ns": snapshot.latest_pose_timestamp_ns,
            "normal_stop_started": snapshot.normal_stop_started,
            "finished": snapshot.finished,
        }
        _atomic_write_json(result_path, result)
        return result
    finally:
        peer.close()
        transport.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ready-path", type=Path, required=True)
    parser.add_argument("--start-path", type=Path, required=True)
    parser.add_argument("--result-path", type=Path, required=True)
    parser.add_argument("--fault-path", type=Path, required=True)
    parser.add_argument("--source-session-id", required=True)
    parser.add_argument("--peer-timeout-sec", type=float, default=_STARTUP_TIMEOUT_SEC)
    arguments = parser.parse_args(argv)
    try:
        source_session_id = bytes.fromhex(arguments.source_session_id)
    except ValueError as error:
        parser.error(f"--source-session-id must be 32 lowercase hex digits: {error}")
    if len(source_session_id) != 16 or source_session_id.hex() != arguments.source_session_id:
        parser.error("--source-session-id must be 32 lowercase hex digits")
    result = run_command_peer(
        source_session_id=source_session_id,
        ready_path=arguments.ready_path,
        start_path=arguments.start_path,
        result_path=arguments.result_path,
        fault_path=arguments.fault_path,
        peer_timeout_sec=arguments.peer_timeout_sec,
    )
    if result["fault_reason"] is not None:
        print(result["fault_reason"], file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
