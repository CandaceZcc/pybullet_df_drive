"""阶段五 Dashboard 专用 eCAL 接收服务：回调仅入队，解码在后台完成。"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from hashlib import sha256
from threading import Condition, Event, Thread
import time
from typing import Callable

import numpy as np
from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.dashboard_snapshot import V2DashboardSnapshotStore
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.ecal_raw import (
    ProtocolVerificationState,
    RawReceivedFrame,
    classify_raw_frame_metadata,
)
from slope_sim.interfaces.v2.models import (
    ImuAttitudeV2,
    LidarPointCloudV2,
    RtkStateV2,
    WheelCommandV2,
    WheelStateV2,
)
from slope_sim.interfaces.v2.sensor_frames import V2SensorFrames
from slope_sim.interfaces.v2.topics import V2_TOPICS
from slope_sim.mapping_replay import recover_pose_node


_SENSOR_TOPICS = frozenset({
    "/sim/lidar/points", "/sim/rtk/state", "/sim/imu/attitude",
})


@dataclass(frozen=True, slots=True)
class V2DashboardCloudFrame:
    """一帧已变换到 world 坐标的真实 MID-360 点，供 GUI 线程渲染。"""

    timestamp_ns: int
    sequence: int
    positions: np.ndarray
    reflectivity: np.ndarray
    tags: np.ndarray
    vehicle_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    vehicle_forward: tuple[float, float, float] = (1.0, 0.0, 0.0)


@dataclass(frozen=True, slots=True)
class V2RawMetadataSnapshot:
    """单话题最近一帧的 wire metadata 判定，供启动期诊断读取。"""

    expected_type_name: str
    actual_type_name: str
    expected_encoding: str
    actual_encoding: str
    expected_descriptor_sha256: str
    actual_descriptor_sha256: str
    endpoint_state: str


class _RawObserverSubscription:
    """Dashboard raw observer 的单话题资源，关闭时只注销自己的回调。"""

    def __init__(self, observer: "V2DashboardRawObserverTransport", topic: str) -> None:
        self._observer = observer
        self._topic = topic
        self._closed = False

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._observer._close_subscription(self._topic)


class V2DashboardRawObserverTransport:
    """附着既有 eCAL core 的只读 raw observer，不拥有 initialize/finalize。"""

    def __init__(
        self,
        descriptor: DescriptorIdentity,
        *,
        raw_bindings: object | None = None,
        start_worker: bool = True,
    ) -> None:
        if type(descriptor) is not DescriptorIdentity:
            raise ValueError("descriptor must be an exact DescriptorIdentity")
        if not isinstance(start_worker, bool):
            raise ValueError("start_worker must be a bool")
        if raw_bindings is None:
            from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings

            raw_bindings = EcalRawBindings()
        if not callable(getattr(raw_bindings, "create_subscriber", None)):
            raise ValueError("raw_bindings must provide create_subscriber")
        self._descriptor = descriptor
        self._raw_bindings = raw_bindings
        self._condition = Condition()
        self._callbacks: dict[str, tuple[str, Callable[[bytes, float], None]]] = {}
        self._resources: dict[str, object] = {}
        self._retired_resources: list[object] = []
        self._pending: dict[str, deque[tuple[bytes, float, str, str, bytes]]] = {}
        self._metadata: dict[str, V2RawMetadataSnapshot] = {}
        self._diagnostics: list[str] = []
        self._diagnostic_callback: Callable[[str, str], None] | None = None
        self._closed = False
        self._stop = Event()
        self._worker: Thread | None = None
        if start_worker:
            self.start()

    @property
    def diagnostics(self) -> tuple[str, ...]:
        with self._condition:
            return tuple(self._diagnostics)

    def set_diagnostic_callback(
        self, callback: Callable[[str, str], None] | None,
    ) -> None:
        """设置 worker 后台协议错误投影；native callback 永不调用该回调。"""
        if callback is not None and not callable(callback):
            raise ValueError("diagnostic callback must be callable or None")
        with self._condition:
            self._diagnostic_callback = callback

    def metadata_snapshot(self, topic: str) -> V2RawMetadataSnapshot | None:
        """返回 topic 最近的 metadata 状态；尚未收到帧时返回 None。"""
        with self._condition:
            return self._metadata.get(topic)

    def start(self) -> None:
        """在 receiver 已完成错误桥接后启动元数据 worker。"""
        with self._condition:
            if self._closed:
                raise RuntimeError("raw Dashboard observer is closed")
            if self._worker is not None:
                return
            self._worker = Thread(
                target=self._run,
                name="v2-dashboard-raw-observer",
                daemon=True,
            )
            self._worker.start()

    def subscribe(
        self,
        topic: str,
        type_name: str,
        callback: Callable[[bytes, float], None],
    ) -> _RawObserverSubscription:
        """注册一个指定合同 topic；该操作绝不初始化第二个 eCAL participant。"""
        contract = next((item for item in V2_TOPICS if item.topic == topic), None)
        if contract is None or contract.type_name != type_name:
            raise ValueError("topic/type_name must match the v2 Dashboard contract")
        if not callable(callback):
            raise ValueError("callback must be callable")
        with self._condition:
            if self._closed:
                raise RuntimeError("raw Dashboard observer is closed")
            if topic in self._callbacks:
                raise RuntimeError("raw Dashboard topic is already subscribed")
            self._callbacks[topic] = (type_name, callback)
            self._pending[topic] = deque(maxlen=1)

        def receive(frame: object) -> None:
            # native callback 只创建独立副本和有界入队，不能做 descriptor/hash/Protobuf 工作。
            try:
                copied = (
                    bytes(frame.payload),
                    float(frame.received_at),
                    "" if frame.remote_type_name is None else str(frame.remote_type_name),
                    "" if frame.remote_encoding is None else str(frame.remote_encoding),
                    b"" if frame.remote_descriptor is None else bytes(frame.remote_descriptor),
                )
            except (AttributeError, TypeError, ValueError):
                with self._condition:
                    self._record_diagnostic_locked(topic, "raw envelope is invalid")
                return
            with self._condition:
                if self._closed or topic not in self._pending:
                    return
                queue = self._pending[topic]
                if queue:
                    self._record_diagnostic_locked(topic, "replaced stale raw frame")
                queue.clear()
                queue.append(copied)
                self._condition.notify()

        try:
            resource = self._raw_bindings.create_subscriber(
                topic, type_name, self._descriptor, receive,
            )
        except Exception:
            with self._condition:
                self._callbacks.pop(topic, None)
                self._pending.pop(topic, None)
            raise
        with self._condition:
            self._resources[topic] = resource
        return _RawObserverSubscription(self, topic)

    def process_pending(self) -> int:
        """在 worker 中校验 eCAL wire 元数据，之后才交给 receiver 的 bytes callback。"""
        work: list[tuple[str, tuple[bytes, float, str, str, bytes]]] = []
        with self._condition:
            for topic, queue in self._pending.items():
                if queue:
                    work.append((topic, queue.popleft()))
        for topic, frame in work:
            payload, received_at, remote_type, encoding, remote_descriptor = frame
            with self._condition:
                callback_info = self._callbacks.get(topic)
            if callback_info is None:
                continue
            type_name, callback = callback_info
            raw_frame = RawReceivedFrame(
                payload, 0, 0, "", remote_type, encoding, remote_descriptor, 0, 0, received_at,
            )
            state = classify_raw_frame_metadata(
                raw_frame, expected_type=type_name, descriptor=self._descriptor,
            )
            self._record_metadata_snapshot(
                topic, type_name, remote_type, encoding, remote_descriptor, state,
            )
            if state is ProtocolVerificationState.PENDING:
                continue
            if state is ProtocolVerificationState.CONFLICT:
                self._record_diagnostic(topic, self._metadata_conflict_detail(
                    type_name, remote_type, encoding, remote_descriptor,
                ))
                continue
            callback(payload, received_at)
        return len(work)

    def _record_metadata_snapshot(
        self,
        topic: str,
        expected_type: str,
        actual_type: str,
        actual_encoding: str,
        actual_descriptor: bytes,
        state: ProtocolVerificationState,
    ) -> None:
        snapshot = V2RawMetadataSnapshot(
            expected_type,
            actual_type,
            "proto",
            actual_encoding,
            sha256(self._descriptor.serialized_file_descriptor_set).hexdigest(),
            sha256(actual_descriptor).hexdigest(),
            state.value,
        )
        with self._condition:
            self._metadata[topic] = snapshot

    def _metadata_conflict_detail(
        self,
        expected_type: str,
        actual_type: str,
        actual_encoding: str,
        actual_descriptor: bytes,
    ) -> str:
        if actual_type != expected_type:
            return "remote type name does not match v2 contract"
        if actual_encoding != "proto":
            return "remote encoding does not match v2 contract"
        if actual_descriptor != self._descriptor.serialized_file_descriptor_set:
            return "remote descriptor does not match v2 contract"
        raise RuntimeError("metadata conflict requires a mismatching nonempty field")

    def _run(self) -> None:
        while not self._stop.is_set():
            if self.process_pending():
                continue
            with self._condition:
                self._condition.wait(timeout=0.1)

    def _record_diagnostic(self, topic: str, detail: str) -> None:
        with self._condition:
            self._record_diagnostic_locked(topic, detail)
            callback = self._diagnostic_callback
        if callback is not None:
            callback(topic, detail)

    def _record_diagnostic_locked(self, topic: str, detail: str) -> None:
        self._diagnostics.append(f"{topic}: {detail}")
        if len(self._diagnostics) > 2_000:
            del self._diagnostics[:-2_000]

    def _close_subscription(self, topic: str) -> None:
        with self._condition:
            resource = self._resources.pop(topic, None)
            self._callbacks.pop(topic, None)
            self._pending.pop(topic, None)
            self._metadata.pop(topic, None)
            if resource is not None:
                # observer 不拥有进程级 eCAL core；同步注销 native callback 会与
                # 正在等待 Python GIL 的分发线程互等。保留资源至 core 统一 finalize。
                self._retired_resources.append(resource)

    def close(self) -> None:
        """注销 observer 回调并结束 worker，绝不关闭 caller 已初始化的 eCAL core。"""
        with self._condition:
            if self._closed:
                return
            self._closed = True
            topics = tuple(self._resources)
            self._condition.notify_all()
        self._stop.set()
        for topic in topics:
            self._close_subscription(topic)
        if self._worker is not None:
            self._worker.join(timeout=1.0)


class V2DashboardEcalReceiver:
    """独立订阅五路 v2 数据，向 GUI 提供经过校验的不可变 snapshot。"""

    def __init__(
        self,
        descriptor: DescriptorIdentity,
        *,
        transport: object,
        start_worker: bool = True,
    ) -> None:
        if type(descriptor) is not DescriptorIdentity:
            raise ValueError("descriptor must be an exact DescriptorIdentity")
        subscribe = getattr(transport, "subscribe", None)
        if not callable(subscribe):
            raise ValueError("transport must provide subscribe")
        if not isinstance(start_worker, bool):
            raise ValueError("start_worker must be a bool")
        self._descriptor = descriptor
        self._transport = transport
        self._codec = V2ProtoCodec(descriptor)
        self._store = V2DashboardSnapshotStore()
        self._condition = Condition()
        self._pending: dict[str, deque[tuple[bytes, float]]] = {
            contract.topic: deque(maxlen=1) for contract in V2_TOPICS
        }
        self._sensor_samples: dict[int, dict[str, object]] = {}
        self._last_sequences: dict[str, int] = {}
        self._identity: tuple[bytes, bytes, int] | None = None
        self._render_dropped_count = 0
        self._cloud_frame: V2DashboardCloudFrame | None = None
        self._errors: list[str] = []
        self._closed = False
        self._stop = Event()
        self._worker: Thread | None = None
        self._subscriptions = tuple(
            subscribe(contract.topic, contract.type_name, self._callback(contract.topic))
            for contract in V2_TOPICS
        )
        if start_worker:
            self._worker = Thread(
                target=self._run,
                name="v2-dashboard-receiver",
                daemon=True,
            )
            self._worker.start()

    @property
    def snapshot_store(self) -> V2DashboardSnapshotStore:
        """Dashboard GUI 定时读取的接收端专属快照。"""
        return self._store

    @property
    def render_dropped_count(self) -> int:
        """容量为 1 的 LiDAR 队列替换次数，与协议 drop 分开统计。"""
        with self._condition:
            return self._render_dropped_count

    @property
    def diagnostics(self) -> tuple[str, ...]:
        """返回有界接收错误，供 Dashboard 诊断终端显示。"""
        with self._condition:
            return tuple(self._errors)

    def cloud_frame(self) -> V2DashboardCloudFrame | None:
        """返回最新的 worker 结果；数组只读，Qt/OpenGL 不跨线程共享。"""
        with self._condition:
            return self._cloud_frame

    def record_transport_error(self, topic: str, detail: str) -> None:
        """接收 raw observer 在其 worker 中拒绝的 wire 元数据错误。"""
        reason = f"wire metadata: {detail}"
        self._store.record_observer_rejection(
            topic=topic,
            source_id=None,
            source_session_id=None,
            sequence=None,
            simulation_session_id=None,
            world_generation=None,
            reason=reason,
            received_at=time.monotonic(),
        )
        self._record_error(f"{topic}: {reason}")

    def _callback(self, topic: str) -> Callable[[bytes, float], None]:
        def receive(payload: bytes, received_at: float) -> None:
            # native/eCAL 回调只复制 bytes 并替换旧帧；解析从不在此路径发生。
            copied = bytes(payload)
            with self._condition:
                if self._closed:
                    return
                queue = self._pending[topic]
                if queue and topic == "/sim/lidar/points":
                    self._render_dropped_count += 1
                queue.clear()
                queue.append((copied, float(received_at)))
                self._condition.notify()

        return receive

    def process_pending(self) -> int:
        """处理当前每个 topic 的最新帧；供 worker 与确定性测试共用。"""
        work: list[tuple[str, bytes, float]] = []
        with self._condition:
            for contract in V2_TOPICS:
                queue = self._pending[contract.topic]
                if queue:
                    payload, received_at = queue.popleft()
                    work.append((contract.topic, payload, received_at))
        for topic, payload, received_at in work:
            self._process(topic, payload, received_at)
        return len(work)

    def _run(self) -> None:
        while not self._stop.is_set():
            processed = self.process_pending()
            if processed:
                continue
            with self._condition:
                self._condition.wait(timeout=0.1)

    def _process(self, topic: str, payload: bytes, received_at: float) -> None:
        try:
            message = self._decode(topic, payload)
            self._validate_monotonic(topic, message)
            if topic == "/sim/wheel/state":
                self._store.update_wheel_state(message, observed_at=received_at)
            elif topic == "/sim/wheel/command":
                self._record_command(message, received_at)
            else:
                self._record_sensor(topic, message, received_at)
        except (TypeError, ValueError) as error:
            reason = f"{type(error).__name__}: {error}"
            self._store.record_observer_rejection(
                topic=topic,
                source_id=None,
                source_session_id=None,
                sequence=None,
                simulation_session_id=None,
                world_generation=None,
                reason=reason,
                received_at=received_at,
            )
            self._record_error(f"{topic}: {reason}")

    def _decode(self, topic: str, payload: bytes) -> object:
        if topic == "/sim/wheel/command":
            return self._codec.decode_wheel_command(payload)
        if topic == "/sim/wheel/state":
            return self._codec.decode_wheel_state(payload)
        if topic == "/sim/lidar/points":
            return self._codec.decode_lidar_point_cloud(payload)
        if topic == "/sim/rtk/state":
            return self._codec.decode_rtk_state(payload)
        if topic == "/sim/imu/attitude":
            return self._codec.decode_imu_attitude(payload)
        raise ValueError("unexpected v2 dashboard topic")

    def _validate_monotonic(self, topic: str, message: object) -> None:
        identity = (
            message.simulation_session_id,
            message.descriptor_sha256,
            message.world_generation,
        )
        if identity[1] != self._descriptor.sha256:
            raise ValueError("descriptor does not match Dashboard descriptor")
        if self._identity is None:
            self._identity = identity
        elif identity != self._identity:
            previous_session, _previous_descriptor, previous_generation = self._identity
            if identity[0] != previous_session or identity[2] <= previous_generation:
                raise ValueError("message session or world generation does not match Dashboard")
            # 结构动作会让所有 topic 的 sequence 重新开始；同一 session 的较新
            # generation 是一个新的可信世界，而不是协议冲突。
            self._identity = identity
            self._last_sequences.clear()
            self._sensor_samples.clear()
            self._store = V2DashboardSnapshotStore()
            with self._condition:
                self._cloud_frame = None
        previous = self._last_sequences.get(topic)
        if previous is not None and message.sequence <= previous:
            raise ValueError("message sequence did not advance")
        self._last_sequences[topic] = message.sequence

    def _record_command(self, message: WheelCommandV2, received_at: float) -> None:
        # command 有独立 session 字段；仅在已有可显示 snapshot 后写入观测统计。
        snapshot = self._store.snapshot()
        if snapshot is not None:
            self._store.record_accepted_command(
                sequence=message.sequence,
                timestamp_ns=message.timestamp_ns,
                received_at=received_at,
                accepted=True,
            )

    def _record_sensor(self, topic: str, message: object, received_at: float) -> None:
        timestamp_ns = message.timebase_ns if topic == "/sim/lidar/points" else message.timestamp_ns
        sample = self._sensor_samples.setdefault(timestamp_ns, {})
        sample[topic] = (message, received_at)
        while len(self._sensor_samples) > 8:
            del self._sensor_samples[min(self._sensor_samples)]
        if set(sample) != _SENSOR_TOPICS:
            return
        lidar, lidar_at = sample["/sim/lidar/points"]
        rtk, rtk_at = sample["/sim/rtk/state"]
        imu, imu_at = sample["/sim/imu/attitude"]
        if not (
            type(lidar) is LidarPointCloudV2
            and type(rtk) is RtkStateV2
            and type(imu) is ImuAttitudeV2
        ):
            raise ValueError("sensor sample types do not match v2 contract")
        self._store.update_sensor_frames(
            V2SensorFrames(lidar, rtk, imu),
            observed_at=max(lidar_at, rtk_at, imu_at),
        )
        self._publish_cloud_frame(lidar, rtk, imu)
        del self._sensor_samples[timestamp_ns]

    def _publish_cloud_frame(
        self,
        lidar: LidarPointCloudV2,
        rtk: RtkStateV2,
        imu: ImuAttitudeV2,
    ) -> None:
        """复用 RTK+IMU 位姿恢复，把同一采样点云从 lidar_link 放入 world。"""
        recovered = recover_pose_node(rtk, imu)
        pose = recovered.lidar_pose
        local = np.asarray([(point.x, point.y, point.z) for point in lidar.points], dtype=np.float32)
        if local.size == 0:
            local = np.empty((0, 3), dtype=np.float32)
        x, y, z, w = pose.orientation
        rotation = np.asarray((
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
            (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
            (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
        ), dtype=np.float32)
        positions = local @ rotation.T + np.asarray(pose.position, dtype=np.float32)
        positions.setflags(write=False)
        reflectivity = np.asarray([point.reflectivity for point in lidar.points], dtype=np.uint32)
        tags = np.asarray([point.tag for point in lidar.points], dtype=np.uint8)
        reflectivity.setflags(write=False)
        tags.setflags(write=False)
        frame = V2DashboardCloudFrame(
            lidar.timebase_ns,
            lidar.sequence,
            positions,
            reflectivity,
            tags,
            tuple(float(value) for value in recovered.base_pose.position),
            tuple(float(rotation[index, 0]) for index in range(3)),
        )
        with self._condition:
            self._cloud_frame = frame

    def _record_error(self, detail: str) -> None:
        with self._condition:
            self._errors.append(detail)
            if len(self._errors) > 2_000:
                del self._errors[:-2_000]

    def close(self) -> None:
        """先注销回调，再停止 worker；不会关闭外部传入的 transport。"""
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._condition.notify_all()
        self._stop.set()
        for subscription in self._subscriptions:
            close = getattr(subscription, "close", None)
            if callable(close):
                close()
        if self._worker is not None:
            self._worker.join(timeout=1.0)
