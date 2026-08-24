"""runSim v2：GUI 到唯一 C++ Command 的本机 socket client 合同。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import threading

from slope_sim.interfaces.v2.runsim_session import RunSimSession


def test_command_client_sends_a_bounded_authenticated_target_and_safe_stops_on_close(tmp_path: Path) -> None:
    """GUI 只续租 C++ Command；关闭连接时必须立刻撤销本地 target。"""
    session = RunSimSession.create(
        tmp_path / "socket",
        command_pid=os.getpid(),
        command_uid=os.getuid(),
        orchestrator_pid=os.getpid(),
        session_id_factory=lambda: b"s" * 16,
        token_factory=lambda: b"t" * 32,
    )
    received: list[bytes] = []
    ready = threading.Event()

    def serve() -> None:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(session.socket_path))
            server.listen(1)
            ready.set()
            connection, _ = server.accept()
            with connection:
                payload = bytearray()
                while payload.count(b"\n") < 2:
                    chunk = connection.recv(1024)
                    if not chunk:
                        break
                    payload.extend(chunk)
                received.append(bytes(payload))

    worker = threading.Thread(target=serve)
    worker.start()
    assert ready.wait(timeout=1.0)

    from slope_sim.interfaces.v2.runsim_command_client import RunSimCommandClient

    client = RunSimCommandClient(session)
    client.connect()
    try:
        client.send_target(0.4, -0.2, now=10.0)
        client.sync_generation(2, 2, robot_model="active_steering_4wd", now=10.01)
    finally:
        client.close()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    target, generation = received[0].splitlines()
    assert json.loads(target) == {
        "angular_velocity_rad_s": -0.2,
        "kind": "target",
        "linear_velocity_m_s": 0.4,
        "token": (b"t" * 32).hex(),
    }
    assert json.loads(generation) == {
        "command_generation": 2,
        "kind": "generation",
        "robot_model": "active_steering_4wd",
        "token": (b"t" * 32).hex(),
        "world_generation": 2,
    }
    snapshot = session.snapshot()
    assert snapshot.state.value == "safe_stop"
    assert snapshot.safe_stop_reason == "connection_closed"


def test_command_client_serializes_rc_target_and_world_generation_frames(
    tmp_path: Path,
) -> None:
    """RC worker 与车型切换线程不得并发写坏同一条流式 Command socket。"""
    session = RunSimSession.create(
        tmp_path / "socket",
        command_pid=os.getpid(),
        command_uid=os.getuid(),
        orchestrator_pid=os.getpid(),
        session_id_factory=lambda: b"s" * 16,
        token_factory=lambda: b"t" * 32,
    )
    first_send_entered = threading.Event()
    second_send_entered = threading.Event()
    release_first_send = threading.Event()

    class BlockingSocket:
        def __init__(self) -> None:
            self.calls = 0

        def sendall(self, _payload: bytes) -> None:
            self.calls += 1
            if self.calls == 1:
                first_send_entered.set()
                assert release_first_send.wait(timeout=1.0)
                return
            second_send_entered.set()

    from slope_sim.interfaces.v2.runsim_command_client import RunSimCommandClient

    client = RunSimCommandClient(session)
    client._socket = BlockingSocket()
    target = threading.Thread(
        target=lambda: client.send_target(0.4, 0.0, now=10.0)
    )
    generation = threading.Thread(
        target=lambda: client.sync_generation(
            2, 2, robot_model="df_mid", now=10.01
        )
    )

    target.start()
    assert first_send_entered.wait(timeout=1.0)
    generation.start()
    try:
        assert not second_send_entered.wait(timeout=0.05)
    finally:
        release_first_send.set()
        target.join(timeout=1.0)
        generation.join(timeout=1.0)

    assert not target.is_alive()
    assert not generation.is_alive()
    assert second_send_entered.is_set()
