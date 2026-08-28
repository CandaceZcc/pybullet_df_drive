"""runSim v2：GUI 到唯一 C++ Command 的本机 socket client 合同。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace

import pytest

from slope_sim.interfaces.v2.runsim_session import RunSimSession


def test_command_relay_reports_status_pipe_eof_as_unexpected_exit() -> None:
    """子进程无状态帧退出时必须稳定 fail-closed，不能泄漏裸 EOFError。"""
    from slope_sim.interfaces.v2.runsim_command_client import (
        RunSimCommandRelayClient,
    )

    class Process:
        @staticmethod
        def is_alive() -> bool:
            return False

    class StatusReceiver:
        @staticmethod
        def poll() -> bool:
            return True

        @staticmethod
        def recv() -> object:
            raise EOFError

    relay = RunSimCommandRelayClient(
        Process(),
        SimpleNamespace(),
        StatusReceiver(),
        [0.0, 0.0, 0.0],
        threading.Lock(),
        SimpleNamespace(set=lambda: None),
        [0.0] * 5,
        threading.Lock(),
    )

    with pytest.raises(RuntimeError, match="exited unexpectedly"):
        relay.send_target(0.4, 0.0, now=1.0)


def test_command_relay_keeps_renewing_while_gui_process_holds_the_gil(
    tmp_path: Path,
) -> None:
    """GUI 被原生调用阻塞超过租约时，独立 relay 仍须维持 50 Hz 续租。"""
    session = RunSimSession.create(
        tmp_path / "socket",
        command_pid=os.getpid(),
        command_uid=os.getuid(),
        orchestrator_pid=os.getpid(),
        session_id_factory=lambda: b"s" * 16,
        token_factory=lambda: b"t" * 32,
    )
    received = bytearray()
    ready = threading.Event()

    def serve() -> None:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(session.socket_path))
            server.listen(1)
            server.settimeout(1.0)
            ready.set()
            try:
                connection, _ = server.accept()
            except TimeoutError:
                return
            with connection:
                while True:
                    chunk = connection.recv(4096)
                    if not chunk:
                        return
                    received.extend(chunk)

    worker = threading.Thread(target=serve, daemon=True)
    worker.start()
    assert ready.wait(timeout=1.0)

    from slope_sim.interfaces.v2.runsim_command_client import (
        RunSimCommandRelayClient,
    )

    relay = RunSimCommandRelayClient.launch(session, renewal_hz=50.0)
    try:
        relay.send_target(0.4, -0.2, now=time.monotonic())
        # PyDLL 调用期间不释放 GIL，复现 PyBullet/eCAL 原生调用阻塞主进程。
        import ctypes

        ctypes.PyDLL(None).usleep(250_000)
        relay_snapshot = relay.diagnostic_snapshot()
    finally:
        relay.close()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    targets = [json.loads(line) for line in received.splitlines()]
    assert len(targets) >= 6
    assert all(item["kind"] == "target" for item in targets)
    assert all(item["linear_velocity_m_s"] == 0.4 for item in targets)
    assert all(item["angular_velocity_rad_s"] == -0.2 for item in targets)
    assert relay_snapshot[0] >= 6
    assert relay_snapshot[2] < 0.1


def test_command_relay_coalesces_gui_burst_to_latest_target(tmp_path: Path) -> None:
    """GUI 高频采样只能覆盖容量 1 mailbox，禁止按帧排队后延迟重放。"""
    session = RunSimSession.create(
        tmp_path / "socket",
        command_pid=os.getpid(),
        command_uid=os.getuid(),
        orchestrator_pid=os.getpid(),
        session_id_factory=lambda: b"s" * 16,
        token_factory=lambda: b"t" * 32,
    )
    received = bytearray()
    ready = threading.Event()

    def serve() -> None:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(session.socket_path))
            server.listen(1)
            server.settimeout(1.0)
            ready.set()
            try:
                connection, _ = server.accept()
            except TimeoutError:
                return
            with connection:
                while True:
                    chunk = connection.recv(4096)
                    if not chunk:
                        return
                    received.extend(chunk)

    worker = threading.Thread(target=serve, daemon=True)
    worker.start()
    assert ready.wait(timeout=1.0)

    from slope_sim.interfaces.v2.runsim_command_client import (
        RunSimCommandRelayClient,
    )

    relay = RunSimCommandRelayClient.launch(session, renewal_hz=50.0)
    try:
        for index in range(2_000):
            relay.send_target(0.4 if index % 2 else -0.4, 0.0, now=time.monotonic())
        relay.send_target(0.0, 0.0, now=time.monotonic())
        time.sleep(0.1)
    finally:
        relay.close()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    targets = [json.loads(line) for line in received.splitlines()]
    assert targets[-1]["linear_velocity_m_s"] == 0.0
    assert len(targets) < 100


def test_command_relay_does_not_renew_pre_generation_target(
    tmp_path: Path,
) -> None:
    """generation 提交后旧目标必须失效，只能由更新版本重新开始续租。"""
    session = RunSimSession.create(
        tmp_path / "socket",
        command_pid=os.getpid(),
        command_uid=os.getuid(),
        orchestrator_pid=os.getpid(),
        session_id_factory=lambda: b"s" * 16,
        token_factory=lambda: b"t" * 32,
    )
    received = bytearray()
    ready = threading.Event()

    def serve() -> None:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(session.socket_path))
            server.listen(1)
            ready.set()
            connection, _ = server.accept()
            with connection:
                while chunk := connection.recv(4096):
                    received.extend(chunk)

    worker = threading.Thread(target=serve)
    worker.start()
    assert ready.wait(timeout=1.0)

    from slope_sim.interfaces.v2.runsim_command_client import (
        RunSimCommandRelayClient,
    )

    relay = RunSimCommandRelayClient.launch(session, renewal_hz=50.0)
    try:
        relay.send_target(0.4, 0.0, now=time.monotonic())
        time.sleep(0.06)
        relay.sync_generation(2, 2, robot_model="df_mid", now=time.monotonic())
        time.sleep(0.06)
        relay.send_target(0.2, 0.0, now=time.monotonic())
        time.sleep(0.06)
    finally:
        relay.close()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    messages = [json.loads(line) for line in received.splitlines()]
    generation_index = next(
        index for index, message in enumerate(messages)
        if message["kind"] == "generation"
    )
    after_generation = messages[generation_index + 1 :]
    assert any(
        message["kind"] == "target"
        and message["linear_velocity_m_s"] == 0.2
        for message in after_generation
    )
    assert all(
        message["kind"] != "target"
        or message["linear_velocity_m_s"] != 0.4
        for message in after_generation
    )


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
