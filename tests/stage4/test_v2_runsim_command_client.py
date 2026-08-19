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
                received.append(connection.recv(1024))

    worker = threading.Thread(target=serve)
    worker.start()
    assert ready.wait(timeout=1.0)

    from slope_sim.interfaces.v2.runsim_command_client import RunSimCommandClient

    client = RunSimCommandClient(session)
    client.connect()
    client.send_target(0.4, -0.2, now=10.0)
    client.close()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert json.loads(received[0]) == {
        "angular_velocity_rad_s": -0.2,
        "kind": "target",
        "linear_velocity_m_s": 0.4,
        "token": (b"t" * 32).hex(),
    }
    snapshot = session.snapshot()
    assert snapshot.state.value == "safe_stop"
    assert snapshot.safe_stop_reason == "connection_closed"
