"""runSim v2：GUI 向唯一 C++ Command 续租的受认证本机 socket client。"""
from __future__ import annotations

import json
import multiprocessing
from multiprocessing.connection import Connection
import os
from pathlib import Path
import socket
import struct
from threading import RLock
import time

from slope_sim.interfaces.v2.runsim_session import RunSimSession


class RunSimCommandClient:
    """持有单条 Command socket 连接；不发布 eCAL WheelCommand。"""

    def __init__(self, session: RunSimSession) -> None:
        if not isinstance(session, RunSimSession):
            raise ValueError("session must be a RunSimSession")
        self._session = session
        self._socket: socket.socket | None = None
        self._send_lock = RLock()

    def connect(self) -> None:
        """连接并以 SO_PEERCRED 核验预期的 C++ Command 进程。"""
        with self._send_lock:
            if self._socket is not None:
                return
            client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            try:
                client.connect(str(self._session.socket_path))
                pid, uid, _gid = struct.unpack(
                    "3i", client.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12)
                )
                self._session.verify_command_peer(pid=pid, uid=uid)
            except BaseException:
                client.close()
                self._session.connection_closed()
                raise
            self._socket = client

    def send_target(self, linear_velocity_m_s: float, angular_velocity_rad_s: float, *, now: float) -> None:
        """发送一条严格 target；I/O 失败不保留旧 target。"""
        payload = json.dumps(
            {
                "kind": "target",
                "token": self._session.server_authentication["token"],
                "linear_velocity_m_s": linear_velocity_m_s,
                "angular_velocity_rad_s": angular_velocity_rad_s,
            },
            separators=(",", ":"),
        ).encode("utf-8") + b"\n"
        with self._send_lock:
            if self._socket is None:
                raise RuntimeError("Command socket is not connected")
            try:
                self._session.accept_client_message(
                    payload[:-1],
                    client_pid=self._session.snapshot().command_pid,
                    peer_uid=self._session.snapshot().command_uid,
                    now=now,
                )
                self._socket.sendall(payload)
            except OSError:
                self.close()
                raise

    def sync_generation(
        self,
        world_generation: int,
        command_generation: int,
        *,
        robot_model: str,
        now: float,
    ) -> None:
        """以同一认证帧同步新 world 代次和车型，令 C++ 重置轮子形状。"""
        if type(world_generation) is not int or world_generation <= 0:
            raise ValueError("world_generation must be a positive int")
        if type(command_generation) is not int or command_generation <= 0:
            raise ValueError("command_generation must be a positive int")
        payload = json.dumps(
            {
                "kind": "generation",
                "token": self._session.server_authentication["token"],
                "world_generation": world_generation,
                "command_generation": command_generation,
                "robot_model": robot_model,
            },
            separators=(",", ":"),
        ).encode("utf-8") + b"\n"
        with self._send_lock:
            if self._socket is None:
                raise RuntimeError("Command socket is not connected")
            try:
                self._session.accept_client_message(
                    payload[:-1],
                    client_pid=self._session.snapshot().command_pid,
                    peer_uid=self._session.snapshot().command_uid,
                    now=now,
                )
                self._socket.sendall(payload)
            except OSError:
                self.close()
                raise

    def close(self) -> None:
        """关闭持久连接并立即撤销人工目标。"""
        with self._send_lock:
            client, self._socket = self._socket, None
            if client is not None:
                client.close()
            self._session.connection_closed()


def _run_command_relay(
    launch_record_path: str,
    command_pid: int,
    command_uid: int,
    orchestrator_pid: int,
    renewal_hz: float,
    receiver: Connection,
    status_sender: Connection,
    target_state: object,
    target_lock: object,
    wake_event: object,
    relay_metrics: object,
    relay_metrics_lock: object,
) -> None:
    """独立持有 Command socket；主 GUI 的 GIL 停顿不能打断固定续租。"""
    client: RunSimCommandClient | None = None
    try:
        session = RunSimSession.attach(
            Path(launch_record_path),
            command_pid=command_pid,
            command_uid=command_uid,
            orchestrator_pid=orchestrator_pid,
        )
        client = RunSimCommandClient(session)
        client.connect()
        status_sender.send(("ready",))
        period = 1.0 / renewal_hz
        latest_target: tuple[float, float] | None = None
        next_deadline: float | None = None
        observed_target_version = 0
        generation_cutoff = 0

        def renew_target(target: tuple[float, float], observed_at: float) -> None:
            client.send_target(*target, now=observed_at)
            with relay_metrics_lock:
                previous_at = float(relay_metrics[1])
                gap = 0.0 if previous_at == 0.0 else observed_at - previous_at
                relay_metrics[0] += 1.0
                relay_metrics[1] = observed_at
                relay_metrics[2] = max(float(relay_metrics[2]), gap)
                relay_metrics[3] = target[0]
                relay_metrics[4] = target[1]

        while True:
            timeout = (
                None
                if next_deadline is None
                else max(0.0, next_deadline - time.monotonic())
            )
            wake_event.wait(timeout)
            wake_event.clear()
            while receiver.poll():
                message = receiver.recv()
                kind = message[0]
                if kind == "close":
                    return
                if kind == "generation":
                    client.sync_generation(
                        int(message[1]),
                        int(message[2]),
                        robot_model=str(message[3]),
                        now=time.monotonic(),
                    )
                    latest_target = None
                    next_deadline = None
                    generation_cutoff = max(generation_cutoff, int(message[4]))
                    continue
                raise RuntimeError("Command relay received an invalid message")

            with target_lock:
                target_version = int(target_state[2])
                target = (float(target_state[0]), float(target_state[1]))
            if (
                target_version > observed_target_version
                and target_version > generation_cutoff
            ):
                observed_target_version = target_version
                latest_target = target
                if next_deadline is None or latest_target == (0.0, 0.0):
                    now = time.monotonic()
                    renew_target(latest_target, now)
                    next_deadline = now + period

            if latest_target is None or next_deadline is None:
                continue
            now = time.monotonic()
            if now < next_deadline:
                continue
            renew_target(latest_target, now)
            next_deadline += period
            if next_deadline <= now:
                next_deadline = now + period
    except EOFError:
        return
    except BaseException as error:
        try:
            status_sender.send(("error", type(error).__name__, str(error)))
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if client is not None:
            client.close()
        receiver.close()
        status_sender.close()


class RunSimCommandRelayClient:
    """把容量 1 的目标交给独立进程，隔离 GUI/PyBullet 的 GIL 停顿。"""

    def __init__(
        self,
        process: multiprocessing.Process,
        sender: Connection,
        status_receiver: Connection,
        target_state: object,
        target_lock: object,
        wake_event: object,
        relay_metrics: object,
        relay_metrics_lock: object,
    ) -> None:
        self._process = process
        self._sender = sender
        self._status_receiver = status_receiver
        self._target_state = target_state
        self._target_lock = target_lock
        self._wake_event = wake_event
        self._relay_metrics = relay_metrics
        self._relay_metrics_lock = relay_metrics_lock
        self._send_lock = RLock()
        self._closed = False

    @classmethod
    def launch(
        cls,
        session: RunSimSession,
        *,
        renewal_hz: float = 50.0,
        startup_timeout_sec: float = 5.0,
    ) -> "RunSimCommandRelayClient":
        if not isinstance(session, RunSimSession):
            raise ValueError("session must be a RunSimSession")
        if (
            isinstance(renewal_hz, bool)
            or not isinstance(renewal_hz, (int, float))
            or renewal_hz <= 0.0
        ):
            raise ValueError("renewal_hz must be positive")
        if (
            isinstance(startup_timeout_sec, bool)
            or not isinstance(startup_timeout_sec, (int, float))
            or startup_timeout_sec <= 0.0
        ):
            raise ValueError("startup_timeout_sec must be positive")
        snapshot = session.snapshot()
        context = multiprocessing.get_context("spawn")
        receiver, sender = context.Pipe(duplex=False)
        status_receiver, status_sender = context.Pipe(duplex=False)
        target_state = context.Array("d", (0.0, 0.0, 0.0), lock=False)
        target_lock = context.Lock()
        wake_event = context.Event()
        relay_metrics = context.Array("d", (0.0, 0.0, 0.0, 0.0, 0.0), lock=False)
        relay_metrics_lock = context.Lock()
        process = context.Process(
            target=_run_command_relay,
            args=(
                str(session.launch_record_path),
                snapshot.command_pid,
                snapshot.command_uid,
                os.getpid(),
                float(renewal_hz),
                receiver,
                status_sender,
                target_state,
                target_lock,
                wake_event,
                relay_metrics,
                relay_metrics_lock,
            ),
            name="runsim-command-relay",
            daemon=True,
        )
        process.start()
        receiver.close()
        status_sender.close()
        relay = cls(
            process,
            sender,
            status_receiver,
            target_state,
            target_lock,
            wake_event,
            relay_metrics,
            relay_metrics_lock,
        )
        if not status_receiver.poll(float(startup_timeout_sec)):
            relay.close()
            raise RuntimeError("Command relay did not become ready")
        status = status_receiver.recv()
        if status != ("ready",):
            relay.close()
            detail = ": ".join(str(item) for item in status[1:])
            raise RuntimeError(f"Command relay startup failed: {detail}")
        return relay

    def diagnostic_snapshot(self) -> tuple[int, float | None, float, float, float]:
        """返回 relay 实际 socket 续租统计，不把主进程提交次数当作发送次数。"""
        with self._relay_metrics_lock:
            count = int(self._relay_metrics[0])
            last_at = float(self._relay_metrics[1])
            return (
                count,
                None if last_at == 0.0 else last_at,
                float(self._relay_metrics[2]),
                float(self._relay_metrics[3]),
                float(self._relay_metrics[4]),
            )

    def send_target(
        self,
        linear_velocity_m_s: float,
        angular_velocity_rad_s: float,
        *,
        now: float,
    ) -> None:
        del now
        with self._send_lock:
            self._require_running_locked()
            with self._target_lock:
                self._target_state[0] = float(linear_velocity_m_s)
                self._target_state[1] = float(angular_velocity_rad_s)
                self._target_state[2] += 1.0
            self._wake_event.set()

    def sync_generation(
        self,
        world_generation: int,
        command_generation: int,
        *,
        robot_model: str,
        now: float,
    ) -> None:
        del now
        with self._send_lock:
            self._require_running_locked()
            with self._target_lock:
                target_cutoff = int(self._target_state[2])
            self._sender.send(
                (
                    "generation",
                    world_generation,
                    command_generation,
                    robot_model,
                    target_cutoff,
                )
            )
            self._wake_event.set()

    def close(self) -> None:
        with self._send_lock:
            if self._closed:
                return
            self._closed = True
            try:
                if self._process.is_alive():
                    self._sender.send(("close",))
                    self._wake_event.set()
            except (BrokenPipeError, EOFError, OSError):
                pass
            self._sender.close()
        self._process.join(timeout=1.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
        self._status_receiver.close()

    def _require_running_locked(self) -> None:
        """调用方持锁时检查 relay 故障，禁止继续写入失去消费者的 mailbox。"""
        if self._closed:
            raise RuntimeError("Command relay is closed")
        if self._status_receiver.poll():
            try:
                status = self._status_receiver.recv()
            except EOFError as error:
                raise RuntimeError(
                    "Command relay exited unexpectedly"
                ) from error
            detail = ": ".join(str(item) for item in status[1:])
            raise RuntimeError(f"Command relay failed: {detail}")
        if not self._process.is_alive():
            raise RuntimeError("Command relay exited unexpectedly")
