"""阶段四 B2：供 GUI 只读消费的 v2 有界遥测快照。"""
from __future__ import annotations

from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, replace
from functools import wraps
from threading import RLock
import time
from typing import TypeVar

from slope_sim.interfaces.v2.models import ImuAttitudeV2, RtkStateV2, WheelStateV2
from slope_sim.interfaces.v2.sensor_frames import V2PreparedSensorFrames, V2SensorFrames
from slope_sim.interfaces.v2.session import OutputIdentity
from slope_sim.interfaces.v2.topics import V2_BY_TOPIC, V2_TOPICS


_T = TypeVar("_T")


def _writer_locked(method: Callable[..., _T]) -> Callable[..., _T]:
    """让一次 Store writer 更新在 tracker 与 published snapshot 间保持原子。"""
    @wraps(method)
    def synchronized(self: "V2DashboardSnapshotStore", *args: object, **kwargs: object) -> _T:
        with self._lock:
            return method(self, *args, **kwargs)

    return synchronized


@dataclass(frozen=True, slots=True)
class V2TopicObservation:
    """Dashboard 的单话题观测，遥测与 transport 时刻绝不混用。"""

    topic: str
    target_hz: int
    actual_hz: float | None = None
    peer_count: int | None = None
    protocol_state: str = "not_checked"
    error_count: int = 0
    dropped_count: int = 0
    sequence_gap_count: int = 0
    latest_sequence: int | None = None
    latest_timestamp_ns: int | None = None
    point_count: int | None = None
    telemetry_observed_at: float | None = None
    transport_observed_at: float | None = None
    authority_error_count: int = 0
    observer_error_count: int = 0


@dataclass(frozen=True, slots=True)
class V2CommandRejection:
    """一次 command 链拒绝的有界标量诊断，永不保留原始 payload。"""

    topic: str
    source_id: str | None
    source_session_id: bytes | None
    sequence: int | None
    simulation_session_id: bytes | None
    world_generation: int | None
    reason: str
    received_at: float


@dataclass(frozen=True, slots=True)
class V2DashboardSnapshot:
    """GUI 可读取的最后一份 v2 状态，不持有 PyBullet 或 transport。"""

    simulation_session_id: bytes
    descriptor_sha256: bytes
    world_generation: int
    wheel_state: WheelStateV2 | None
    lidar_timestamp_ns: int | None
    lidar_sequence: int | None
    lidar_point_count: int | None
    rtk: RtkStateV2 | None
    imu: ImuAttitudeV2 | None
    topic_observations: tuple[V2TopicObservation, ...] = ()
    robot_model: str = "unknown"
    authority_rejections: tuple[V2CommandRejection, ...] = ()
    observer_rejections: tuple[V2CommandRejection, ...] = ()

    def topic_observation(self, topic: str) -> V2TopicObservation:
        """返回固定五话题中一条观测。"""
        for observation in self.topic_observations:
            if observation.topic == topic:
                return observation
        raise ValueError("topic is not part of the v2 dashboard contract")

    def telemetry_age(self, *, now: float, topic: str) -> float | None:
        """年龄仅由 GUI monotonic 与 telemetry 观察时刻计算。"""
        observed_at = self.topic_observation(topic).telemetry_observed_at
        return None if observed_at is None else float(now) - observed_at


class V2DashboardSnapshotStore:
    """物理线程写入最新不可变值，GUI 线程只读取完整快照。"""

    def __init__(
        self,
        *,
        on_update: Callable[[V2DashboardSnapshot], None] | None = None,
        robot_model: str = "unknown",
    ) -> None:
        if on_update is not None and not callable(on_update):
            raise ValueError("on_update must be callable or None")
        if not isinstance(robot_model, str) or not robot_model:
            raise ValueError("robot_model must be a nonempty string")
        self._identity: tuple[bytes, bytes, int] | None = None
        self._snapshot: V2DashboardSnapshot | None = None
        self._on_update = on_update
        self._robot_model = robot_model
        self._lock = RLock()
        self._events = {contract.topic: deque() for contract in V2_TOPICS}
        self._observed = {
            contract.topic: V2TopicObservation(contract.topic, contract.rate_hz)
            for contract in V2_TOPICS
        }
        self._authority_rejections: deque[V2CommandRejection] = deque(maxlen=64)
        self._observer_rejections: deque[V2CommandRejection] = deque(maxlen=64)

    def _observation_values(self) -> tuple[V2TopicObservation, ...]:
        return tuple(self._observed[contract.topic] for contract in V2_TOPICS)

    @staticmethod
    def _observations_at(
        now: float,
        observations: tuple[V2TopicObservation, ...],
        event_times: dict[str, tuple[float, ...]],
    ) -> tuple[V2TopicObservation, ...]:
        """从锁内复制值派生查询视图，不修改 writer 拥有的 tracker。"""
        derived: list[V2TopicObservation] = []
        for previous in observations:
            events = tuple(
                timestamp
                for timestamp in event_times[previous.topic]
                if timestamp >= now - 2.0
            )
            if events:
                actual_hz = None
                if len(events) >= 3 and events[-1] > events[0]:
                    actual_hz = (len(events) - 1) / (events[-1] - events[0])
            else:
                actual_hz = previous.actual_hz
                if (previous.telemetry_observed_at is None
                        or now - previous.telemetry_observed_at > 2.0):
                    actual_hz = None
            if previous.actual_hz != actual_hz:
                previous = V2TopicObservation(
                    previous.topic, previous.target_hz, actual_hz, previous.peer_count,
                    previous.protocol_state, previous.error_count, previous.dropped_count,
                    previous.sequence_gap_count, previous.latest_sequence,
                    previous.latest_timestamp_ns, previous.point_count,
                    previous.telemetry_observed_at, previous.transport_observed_at,
                    previous.authority_error_count, previous.observer_error_count,
                )
            derived.append(previous)
        return tuple(derived)

    def _record_telemetry(
        self, topic: str, *, observed_at: float, sequence: int, timestamp_ns: int,
        point_count: int | None = None,
    ) -> None:
        """保存有界单调窗口；transport 轮询不会经过这里。"""
        if topic not in V2_BY_TOPIC or observed_at < 0.0:
            raise ValueError("invalid dashboard telemetry observation")
        events = self._events[topic]
        if events and observed_at < events[-1]:
            raise ValueError("telemetry observed_at must not move backwards")
        events.append(observed_at)
        while events and events[0] < observed_at - 2.0:
            events.popleft()
        actual_hz = None
        if len(events) >= 3 and events[-1] > events[0]:
            actual_hz = (len(events) - 1) / (events[-1] - events[0])
        previous = self._observed[topic]
        gaps = previous.sequence_gap_count
        if previous.latest_sequence is not None and sequence > previous.latest_sequence + 1:
            gaps += sequence - previous.latest_sequence - 1
        self._observed[topic] = V2TopicObservation(
            topic, V2_BY_TOPIC[topic].rate_hz, actual_hz, previous.peer_count,
            previous.protocol_state, previous.error_count, previous.dropped_count,
            gaps, sequence, timestamp_ns, point_count, observed_at,
            previous.transport_observed_at,
            previous.authority_error_count, previous.observer_error_count,
        )

    def _make_snapshot(
        self, identity: tuple[bytes, bytes, int], *, wheel_state: WheelStateV2 | None,
        lidar_timestamp_ns: int | None, lidar_sequence: int | None,
        lidar_point_count: int | None,
        rtk: RtkStateV2 | None, imu: ImuAttitudeV2 | None,
        topic_observations: tuple[V2TopicObservation, ...] | None = None,
        robot_model: str | None = None,
    ) -> V2DashboardSnapshot:
        return V2DashboardSnapshot(
            *identity, wheel_state, lidar_timestamp_ns, lidar_sequence,
            lidar_point_count, rtk, imu,
            self._observation_values() if topic_observations is None else topic_observations,
            self._robot_model if robot_model is None else robot_model,
            tuple(self._authority_rejections),
            tuple(self._observer_rejections),
        )

    def _replace_snapshot(self, snapshot: V2DashboardSnapshot) -> None:
        """原子替换 latest 值后才通知 IPC producer，禁止暴露半帧状态。"""
        self._snapshot = snapshot
        if self._on_update is not None:
            self._on_update(snapshot)

    @staticmethod
    def _identity_from(value: object) -> tuple[bytes, bytes, int]:
        """抽取由 v2 模型自身校验过的 session、descriptor 与 world 身份。"""
        return (
            value.simulation_session_id,
            value.descriptor_sha256,
            value.world_generation,
        )

    def _require_identity(self, identity: tuple[bytes, bytes, int]) -> None:
        """首次写入锁定身份，后续写入必须属于同一运行世界。"""
        current = self._identity
        if current is None:
            self._identity = identity
            return
        if identity[0] != current[0]:
            raise ValueError("simulation_session_id does not match dashboard snapshot")
        if identity[1] != current[1]:
            raise ValueError("descriptor_sha256 does not match dashboard snapshot")
        if identity[2] != current[2]:
            raise ValueError("world_generation does not match dashboard snapshot")

    @_writer_locked
    def update_sensor_frames(
        self, frames: V2SensorFrames, *, observed_at: float | None = None,
    ) -> None:
        """原子替换同帧 LiDAR、RTK 与 IMU，拒绝任何混合身份输入。"""
        if type(frames) is not V2SensorFrames:
            raise ValueError("frames must be an exact V2SensorFrames")
        identities = (
            self._identity_from(frames.lidar),
            self._identity_from(frames.rtk),
            self._identity_from(frames.imu),
        )
        if identities[1:] != (identities[0], identities[0]):
            raise ValueError("v2 sensor frames must share one session/world identity")
        identity = identities[0]
        self._require_identity(identity)
        current_time = time.monotonic() if observed_at is None else float(observed_at)
        self._record_telemetry("/sim/lidar/points", observed_at=current_time,
                               sequence=frames.lidar.sequence,
                               timestamp_ns=frames.lidar.timebase_ns,
                               point_count=frames.lidar.point_num)
        self._record_telemetry("/sim/rtk/state", observed_at=current_time,
                               sequence=frames.rtk.sequence, timestamp_ns=frames.rtk.timestamp_ns)
        self._record_telemetry("/sim/imu/attitude", observed_at=current_time,
                               sequence=frames.imu.sequence, timestamp_ns=frames.imu.timestamp_ns)
        previous = self._snapshot
        self._replace_snapshot(self._make_snapshot(
            identity, wheel_state=previous.wheel_state if previous is not None else None,
            lidar_timestamp_ns=frames.lidar.timebase_ns,
            lidar_sequence=frames.lidar.sequence,
            lidar_point_count=frames.lidar.point_num, rtk=frames.rtk, imu=frames.imu,
        ))

    @_writer_locked
    def update_prepared_sensor_frames(
        self, frames: V2PreparedSensorFrames, *, observed_at: float | None = None,
    ) -> None:
        """只保存 child LiDAR 的标量元数据；原始 bytes 仅用于 transport 发布。"""
        if type(frames) is not V2PreparedSensorFrames:
            raise ValueError("frames must be an exact V2PreparedSensorFrames")
        lidar_identity = frames.lidar_identity
        if type(lidar_identity) is not OutputIdentity:
            raise ValueError("prepared lidar must carry an exact OutputIdentity")
        if lidar_identity.topic != "/sim/lidar/points":
            raise ValueError("prepared lidar identity must use the v2 lidar topic")
        if type(frames.lidar_payload) is not bytes or not frames.lidar_payload:
            raise ValueError("prepared lidar payload must be nonempty exact bytes")
        if frames.lidar_timestamp_ns != frames.rtk.timestamp_ns or frames.lidar_timestamp_ns != frames.imu.timestamp_ns:
            raise ValueError("prepared v2 sensor timestamps must match")
        identity = self._identity_from(lidar_identity)
        if (self._identity_from(frames.rtk), self._identity_from(frames.imu)) != (identity, identity):
            raise ValueError("prepared v2 sensor frames must share one session/world identity")
        self._require_identity(identity)
        current_time = time.monotonic() if observed_at is None else float(observed_at)
        self._record_telemetry("/sim/lidar/points", observed_at=current_time,
                               sequence=lidar_identity.sequence,
                               timestamp_ns=frames.lidar_timestamp_ns)
        self._record_telemetry("/sim/rtk/state", observed_at=current_time,
                               sequence=frames.rtk.sequence, timestamp_ns=frames.rtk.timestamp_ns)
        self._record_telemetry("/sim/imu/attitude", observed_at=current_time,
                               sequence=frames.imu.sequence, timestamp_ns=frames.imu.timestamp_ns)
        previous = self._snapshot
        self._replace_snapshot(self._make_snapshot(
            identity, wheel_state=previous.wheel_state if previous is not None else None,
            lidar_timestamp_ns=frames.lidar_timestamp_ns, lidar_sequence=lidar_identity.sequence,
            lidar_point_count=None, rtk=frames.rtk, imu=frames.imu,
        ))

    @_writer_locked
    def update_wheel_state(
        self, wheel_state: WheelStateV2, *, observed_at: float | None = None,
    ) -> None:
        """替换最新 wheel 反馈；拒绝越过 session/world 边界的异步回调。"""
        if type(wheel_state) is not WheelStateV2:
            raise ValueError("wheel_state must be an exact WheelStateV2")
        identity = self._identity_from(wheel_state)
        self._require_identity(identity)
        if self._robot_model == "unknown":
            self._robot_model = wheel_state.robot_model
        elif wheel_state.robot_model != self._robot_model:
            raise ValueError("wheel state robot_model does not match dashboard snapshot")
        self._record_telemetry("/sim/wheel/state",
                               observed_at=time.monotonic() if observed_at is None else float(observed_at),
                               sequence=wheel_state.sequence, timestamp_ns=wheel_state.timestamp_ns)
        previous = self._snapshot
        self._replace_snapshot(self._make_snapshot(
            identity, wheel_state=wheel_state,
            lidar_timestamp_ns=previous.lidar_timestamp_ns if previous is not None else None,
            lidar_sequence=previous.lidar_sequence if previous is not None else None,
            lidar_point_count=previous.lidar_point_count if previous is not None else None,
            rtk=previous.rtk if previous is not None else None,
            imu=previous.imu if previous is not None else None,
        ))

    @_writer_locked
    def record_accepted_command(
        self, *, sequence: int, timestamp_ns: int, received_at: float, accepted: bool,
    ) -> None:
        """只记录 authority 接受后的 command，不把拒绝误呈现为输入遥测。"""
        if not accepted:
            return
        self._record_telemetry("/sim/wheel/command", observed_at=float(received_at),
                               sequence=sequence, timestamp_ns=timestamp_ns)
        if self._snapshot is not None:
                self._replace_snapshot(self._make_snapshot(
                    self._identity, wheel_state=self._snapshot.wheel_state,
                    lidar_timestamp_ns=self._snapshot.lidar_timestamp_ns,
                    lidar_sequence=self._snapshot.lidar_sequence,
                    lidar_point_count=self._snapshot.lidar_point_count, rtk=self._snapshot.rtk,
                    imu=self._snapshot.imu,
                ))

    @_writer_locked
    def record_protocol_error(self, topic: str) -> None:
        """兼容旧观察者入口；新路径应保留具体 observer 原因。"""
        if topic not in V2_BY_TOPIC:
            raise ValueError("topic is not part of the v2 dashboard contract")
        previous = self._observed[topic]
        self._observed[topic] = replace(
            previous,
            error_count=previous.error_count + 1,
            observer_error_count=previous.observer_error_count + 1,
        )
        self._refresh_current_snapshot()

    def _refresh_current_snapshot(self) -> None:
        """仅在已有遥测快照时投影新诊断，避免凭空伪造 topic 数据。"""
        if self._snapshot is not None:
            self._replace_snapshot(self._make_snapshot(
                self._identity, wheel_state=self._snapshot.wheel_state,
                lidar_timestamp_ns=self._snapshot.lidar_timestamp_ns,
                lidar_sequence=self._snapshot.lidar_sequence,
                lidar_point_count=self._snapshot.lidar_point_count, rtk=self._snapshot.rtk,
                imu=self._snapshot.imu,
            ))

    def _record_rejection(
        self,
        domain: str,
        *,
        topic: str,
        source_id: str | None,
        source_session_id: bytes | None,
        sequence: int | None,
        simulation_session_id: bytes | None,
        world_generation: int | None,
        reason: str,
        received_at: float,
    ) -> None:
        if domain not in {"authority", "observer"}:
            raise ValueError("rejection domain is invalid")
        if topic not in V2_BY_TOPIC or not isinstance(reason, str) or not reason:
            raise ValueError("rejection topic and reason must be valid")
        if type(received_at) not in {int, float} or received_at < 0.0:
            raise ValueError("rejection received_at must be nonnegative")
        rejection = V2CommandRejection(
            topic, source_id, source_session_id, sequence, simulation_session_id,
            world_generation, reason, float(received_at),
        )
        previous = self._observed[topic]
        if domain == "authority":
            self._authority_rejections.append(rejection)
            self._observed[topic] = replace(
                previous, authority_error_count=previous.authority_error_count + 1,
            )
        else:
            self._observer_rejections.append(rejection)
            self._observed[topic] = replace(
                previous,
                error_count=previous.error_count + 1,
                observer_error_count=previous.observer_error_count + 1,
            )
        self._refresh_current_snapshot()

    @_writer_locked
    def record_authority_rejection(self, **fields: object) -> None:
        """记录 simulator authority 的 command 拒绝，不混入 Dashboard observer。"""
        self._record_rejection("authority", **fields)

    @_writer_locked
    def record_observer_rejection(self, **fields: object) -> None:
        """记录 Dashboard observer 的拒绝，不影响 authority 计数。"""
        self._record_rejection("observer", **fields)

    @_writer_locked
    def refresh_transport(self, transport_snapshot: object, *, observed_at: float | None = None) -> None:
        """合并逐话题 transport 质量，保持 telemetry 观察时刻不变。"""
        current_time = time.monotonic() if observed_at is None else float(observed_at)
        qualities = {item.topic: item for item in getattr(transport_snapshot, "topic_quality", ())}
        for topic, previous in tuple(self._observed.items()):
            quality = qualities.get(topic)
            self._observed[topic] = V2TopicObservation(
                previous.topic, previous.target_hz, previous.actual_hz,
                previous.peer_count if quality is None else quality.peer_count,
                previous.protocol_state if quality is None else quality.protocol_state,
                previous.error_count if quality is None else quality.error_count,
                previous.dropped_count if quality is None else quality.dropped_count,
                previous.sequence_gap_count, previous.latest_sequence,
                previous.latest_timestamp_ns, previous.point_count,
                previous.telemetry_observed_at, current_time,
                previous.authority_error_count, previous.observer_error_count,
            )
        if self._snapshot is not None:
            self._replace_snapshot(self._make_snapshot(
                self._identity, wheel_state=self._snapshot.wheel_state,
                lidar_timestamp_ns=self._snapshot.lidar_timestamp_ns,
                lidar_sequence=self._snapshot.lidar_sequence,
                lidar_point_count=self._snapshot.lidar_point_count, rtk=self._snapshot.rtk,
                imu=self._snapshot.imu,
            ))

    @_writer_locked
    def update_snapshot(self, snapshot: V2DashboardSnapshot) -> None:
        """接收 IPC 传来的完整只读快照，重新核验每个已解析 v2 身份。"""
        if type(snapshot) is not V2DashboardSnapshot:
            raise ValueError("snapshot must be an exact V2DashboardSnapshot")
        identity = (
            snapshot.simulation_session_id,
            snapshot.descriptor_sha256,
            snapshot.world_generation,
        )
        for value in (snapshot.wheel_state, snapshot.rtk, snapshot.imu):
            if value is not None and self._identity_from(value) != identity:
                raise ValueError("snapshot member world_generation does not match snapshot")
        self._require_identity(identity)
        if snapshot.robot_model != "unknown":
            if self._robot_model != "unknown" and snapshot.robot_model != self._robot_model:
                raise ValueError("snapshot robot_model does not match dashboard store")
            self._robot_model = snapshot.robot_model
        if snapshot.topic_observations:
            self._observed = {item.topic: item for item in snapshot.topic_observations}
        self._replace_snapshot(snapshot)

    def snapshot(self, *, now: float | None = None) -> V2DashboardSnapshot | None:
        """锁内复制最新值；查询频率只派生返回，绝不回写 writer 状态。"""
        with self._lock:
            current = self._snapshot
            if current is None or now is None:
                return current
            identity = self._identity
            observations = self._observation_values()
            event_times = {
                topic: tuple(events) for topic, events in self._events.items()
            }
            robot_model = self._robot_model
        derived = self._observations_at(float(now), observations, event_times)
        if derived == current.topic_observations:
            return current
        return self._make_snapshot(
            identity, wheel_state=current.wheel_state,
            lidar_timestamp_ns=current.lidar_timestamp_ns,
            lidar_sequence=current.lidar_sequence,
            lidar_point_count=current.lidar_point_count, rtk=current.rtk, imu=current.imu,
            topic_observations=derived, robot_model=robot_model,
        )
