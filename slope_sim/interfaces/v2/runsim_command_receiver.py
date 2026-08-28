"""runSim v2：在 GUI 进程外接收命令，并只共享最新一帧。"""
from __future__ import annotations

import multiprocessing
from multiprocessing.connection import Connection
import time

from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.transport import create_v2_command_receive_transport


_MAX_COMMAND_PAYLOAD_BYTES = 65_536


def _run_command_receiver(
    descriptor: DescriptorIdentity,
    payload_buffer: object,
    payload_length: object,
    payload_version: object,
    received_at: object,
    payload_lock: object,
    stop_event: object,
    status_sender: Connection,
) -> None:
    """独立 eCAL participant 只保留最新 command，避免 GUI GIL 阻塞接收。"""
    transport = None
    try:
        transport = create_v2_command_receive_transport(
            descriptor=descriptor,
            participant_name="slope-sim-v2-command-receiver",
        )

        def store_latest(payload: bytes, observed_at: float) -> None:
            wire = bytes(payload)
            if len(wire) > _MAX_COMMAND_PAYLOAD_BYTES:
                raise ValueError("WheelCommand payload exceeds shared buffer")
            with payload_lock:
                payload_buffer[: len(wire)] = wire
                payload_length.value = len(wire)
                received_at.value = float(observed_at)
                payload_version.value += 1

        transport.subscribe(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            store_latest,
        )
        status_sender.send(("ready",))
        while not stop_event.wait(0.02):
            transport.poll_peer_state()
    except BaseException as error:
        try:
            status_sender.send(("error", type(error).__name__, str(error)))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if transport is not None:
            transport.close()
        status_sender.close()


class RunSimCommandReceiver:
    """管理命令接收 sidecar，并向物理主线程暴露 latest-only 快照。"""

    def __init__(
        self,
        process: multiprocessing.Process,
        payload_buffer: object,
        payload_length: object,
        payload_version: object,
        received_at: object,
        payload_lock: object,
        stop_event: object,
        status_receiver: Connection,
    ) -> None:
        self._process = process
        self._payload_buffer = payload_buffer
        self._payload_length = payload_length
        self._payload_version = payload_version
        self._received_at = received_at
        self._payload_lock = payload_lock
        self._stop_event = stop_event
        self._status_receiver = status_receiver
        self._closed = False

    @classmethod
    def launch(
        cls,
        descriptor: DescriptorIdentity,
        *,
        startup_timeout_sec: float = 5.0,
    ) -> "RunSimCommandReceiver":
        if not isinstance(descriptor, DescriptorIdentity):
            raise ValueError("descriptor must be a DescriptorIdentity")
        if type(startup_timeout_sec) not in {int, float} or startup_timeout_sec <= 0.0:
            raise ValueError("startup_timeout_sec must be positive")
        context = multiprocessing.get_context("spawn")
        payload_buffer = context.Array(
            "B", _MAX_COMMAND_PAYLOAD_BYTES, lock=False
        )
        payload_length = context.Value("I", 0, lock=False)
        payload_version = context.Value("Q", 0, lock=False)
        received_at = context.Value("d", 0.0, lock=False)
        payload_lock = context.Lock()
        stop_event = context.Event()
        status_receiver, status_sender = context.Pipe(duplex=False)
        process = context.Process(
            target=_run_command_receiver,
            args=(
                descriptor,
                payload_buffer,
                payload_length,
                payload_version,
                received_at,
                payload_lock,
                stop_event,
                status_sender,
            ),
            name="runsim-command-receiver",
            daemon=True,
        )
        process.start()
        status_sender.close()
        receiver = cls(
            process,
            payload_buffer,
            payload_length,
            payload_version,
            received_at,
            payload_lock,
            stop_event,
            status_receiver,
        )
        if not status_receiver.poll(float(startup_timeout_sec)):
            receiver.close()
            raise RuntimeError("Command receiver did not become ready")
        status = status_receiver.recv()
        if status != ("ready",):
            receiver.close()
            detail = ": ".join(str(item) for item in status[1:])
            raise RuntimeError(f"Command receiver startup failed: {detail}")
        return receiver

    @property
    def process_pid(self) -> int | None:
        """返回 sidecar PID，供资源状态和关闭诊断使用。"""
        return self._process.pid

    def take_latest(self, after_version: int) -> tuple[int, bytes, float] | None:
        """版本未变化时返回 None；变化时只复制当前最新 payload。"""
        if type(after_version) is not int or after_version < 0:
            raise ValueError("after_version must be a nonnegative int")
        self._require_running()
        with self._payload_lock:
            version = int(self._payload_version.value)
            if version <= after_version:
                return None
            length = int(self._payload_length.value)
            payload = bytes(self._payload_buffer[:length])
            observed_at = float(self._received_at.value)
        return version, payload, observed_at

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        self._process.join(timeout=1.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
        self._status_receiver.close()

    def _require_running(self) -> None:
        if self._closed:
            raise RuntimeError("Command receiver is closed")
        if self._status_receiver.poll():
            try:
                status = self._status_receiver.recv()
            except EOFError as error:
                raise RuntimeError(
                    "Command receiver exited unexpectedly"
                ) from error
            detail = ": ".join(str(item) for item in status[1:])
            raise RuntimeError(f"Command receiver failed: {detail}")
        if not self._process.is_alive():
            raise RuntimeError("Command receiver exited unexpectedly")
