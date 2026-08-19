"""阶段四 B2：在物理主线程协调五个 v2 输出 topic 的正式 runtime。"""
from __future__ import annotations

from collections.abc import Callable
import time

from slope_sim.interfaces.models import WheelState
from slope_sim.interfaces.v2.dashboard_snapshot import V2DashboardSnapshotStore
from slope_sim.interfaces.v2.runtime_protocol import V2RuntimeProtocol
from slope_sim.interfaces.v2.sensor_frames import (
    V2OutputFramePublisher,
    V2PreparedSensorFrames,
    V2PublishBatch,
    V2PublishCadence,
    V2SensorFrameFactory,
    V2WheelStateFactory,
)


class V2SimulatorRuntime:
    """复用 v2 authority 和 B1 真值，在每个物理步发布到期的五话题输出。"""

    def __init__(
        self,
        *,
        controller: V2RuntimeProtocol,
        wheel_feedback_reader: Callable[[int], WheelState],
        sensor_frames: V2SensorFrameFactory,
        output_publisher: V2OutputFramePublisher,
        wheel_state_factory: V2WheelStateFactory,
        dashboard_snapshot_store: V2DashboardSnapshotStore | None = None,
    ) -> None:
        if not isinstance(controller, V2RuntimeProtocol):
            raise ValueError("controller must be a V2RuntimeProtocol")
        if not callable(wheel_feedback_reader):
            raise ValueError("wheel_feedback_reader must be callable")
        if not isinstance(sensor_frames, V2SensorFrameFactory):
            raise ValueError("sensor_frames must be a V2SensorFrameFactory")
        if not isinstance(output_publisher, V2OutputFramePublisher):
            raise ValueError("output_publisher must be a V2OutputFramePublisher")
        if not isinstance(wheel_state_factory, V2WheelStateFactory):
            raise ValueError("wheel_state_factory must be a V2WheelStateFactory")
        if dashboard_snapshot_store is not None and type(dashboard_snapshot_store) is not V2DashboardSnapshotStore:
            raise ValueError("dashboard_snapshot_store must be None or an exact V2DashboardSnapshotStore")
        self._controller = controller
        self._wheel_feedback_reader = wheel_feedback_reader
        self._sensor_frames = sensor_frames
        self._output_publisher = output_publisher
        self._wheel_state_factory = wheel_state_factory
        self._dashboard_snapshot_store = dashboard_snapshot_store
        self._cadence = V2PublishCadence()

    def after_physics_step(self, dt: object, *, wall_time: float) -> V2PublishBatch:
        """在 Bullet 步进后只发布已到期输出；传感器三消息始终同一 timestamp。"""
        batch = self._cadence.advance(dt)
        for timestamp_ns in batch.wheel_timestamps_ns:
            feedback = self._wheel_feedback_reader(timestamp_ns)
            if not isinstance(feedback, WheelState):
                raise RuntimeError("wheel feedback reader must return WheelState")
            if feedback.timestamp_ns != timestamp_ns:
                raise RuntimeError("wheel feedback timestamp does not match deadline")
            wheel_state = self._wheel_state_factory.build(feedback)
            self._output_publisher.publish_wheel_state(wheel_state, wall_time=wall_time)
            if self._dashboard_snapshot_store is not None:
                self._dashboard_snapshot_store.update_wheel_state(
                    wheel_state, observed_at=time.monotonic()
                )
        self._publish_completed_sensor_frames(wall_time=wall_time)
        for timestamp_ns in batch.sensor_timestamps_ns:
            frames = self._sensor_frames.capture(timestamp_ns)
            if frames is not None:
                self._output_publisher.publish(frames, wall_time=wall_time)
            if frames is not None and self._dashboard_snapshot_store is not None:
                self._dashboard_snapshot_store.update_sensor_frames(
                    frames, observed_at=time.monotonic()
                )
        self._publish_completed_sensor_frames(wall_time=wall_time)
        return batch

    def _publish_completed_sensor_frames(self, *, wall_time: float) -> int:
        """在物理步边界非阻塞发出已完成的 child LiDAR raw payload。"""
        completed = self._sensor_frames.poll_completed()
        if type(completed) is not tuple:
            raise RuntimeError("sensor frame factory poll_completed must return an exact tuple")
        for frames in completed:
            if not isinstance(frames, V2PreparedSensorFrames):
                raise RuntimeError("sensor frame factory returned an invalid prepared frame")
            self._output_publisher.publish_prepared(frames, wall_time=wall_time)
            if self._dashboard_snapshot_store is not None:
                self._dashboard_snapshot_store.update_prepared_sensor_frames(
                    frames, observed_at=time.monotonic()
                )
        return len(completed)

    def drain_sensor_outputs(self, *, timeout_sec: float) -> int:
        """停止物理步后有界等待已提交 LiDAR job，避免关闭时丢弃最后一帧。"""
        if type(timeout_sec) not in {int, float} or timeout_sec <= 0.0:
            raise ValueError("timeout_sec must be positive")
        deadline = time.monotonic() + float(timeout_sec)
        delivered = 0
        while self._sensor_frames.has_pending():
            delivered += self._publish_completed_sensor_frames(wall_time=time.monotonic())
            if not self._sensor_frames.has_pending():
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("Stage4 lidar worker did not drain before shutdown")
            time.sleep(0.001)
        return delivered

    def refresh_transport(self) -> object:
        """在物理步前刷新 raw discovery，并把协议状态交给唯一 authority。"""
        snapshot = self._controller.refresh_transport()
        if self._dashboard_snapshot_store is not None:
            self._dashboard_snapshot_store.refresh_transport(
                snapshot, observed_at=time.monotonic()
            )
        return snapshot

    def accept_command_payload(self, payload: bytes, *, received_at: float) -> bool:
        """委托既有 controller 解码和认领，拒绝时绝不触碰物理线程状态。"""
        accepted = self._controller.accept_payload(payload, received_at=received_at)
        if accepted and self._dashboard_snapshot_store is not None:
            authority = self._controller.snapshot().authority
            timestamp_ns = self._controller.mailbox.latest_timestamp_ns()
            if authority.last_sequence is None or timestamp_ns is None:
                raise RuntimeError("accepted command metadata is unavailable")
            self._dashboard_snapshot_store.record_accepted_command(
                sequence=authority.last_sequence,
                timestamp_ns=timestamp_ns,
                received_at=received_at,
                accepted=True,
            )
        return accepted

    def command_decision(self, *, now: float) -> object:
        """返回 mailbox 的当前安全决定，供物理主线程在每步应用。"""
        return self._controller.mailbox.decision(now=now)
