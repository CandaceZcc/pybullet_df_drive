"""runSim v2：唯一 C++ Command 的子进程启动与安全回收合同。"""
from __future__ import annotations

import os
from pathlib import Path
import signal
from types import SimpleNamespace
import subprocess
import sys


def test_supervisor_attaches_child_preexec_record_and_terminates_its_process(tmp_path: Path) -> None:
    """父进程只接管 child PID 的记录，关闭时回收该 child 与认证锁。"""
    from slope_sim.interfaces.v2.runsim_command_supervisor import RunSimCommandSupervisor

    supervisor = RunSimCommandSupervisor.launch(
        [sys.executable, "-c", "import time; time.sleep(30)"],
        socket_parent=tmp_path,
    )
    try:
        snapshot = supervisor.session.snapshot()
        assert snapshot.command_pid == supervisor.process.pid
        assert supervisor.session.launch_record_path.is_file()
    finally:
        supervisor.close()

    assert supervisor.process.poll() is not None
    assert not supervisor.session.launch_record_path.exists()


def test_supervisor_provides_the_future_launch_record_and_session_identity_to_command_builder(
    tmp_path: Path,
) -> None:
    """交互 Command 的参数必须在 spawn 前取得同一会话的记录路径与身份。"""
    from slope_sim.interfaces.v2.runsim_command_supervisor import RunSimCommandSupervisor

    observed = []

    def command_builder(launch):
        observed.append(launch)
        return [sys.executable, "-c", "import time; time.sleep(30)"]

    supervisor = RunSimCommandSupervisor.launch(command_builder, socket_parent=tmp_path)
    try:
        assert len(observed) == 1
        launch = observed[0]
        assert launch.launch_record_path == supervisor.session.launch_record_path
        assert launch.simulation_session_id.hex() == supervisor.session.server_authentication["session_id"]
    finally:
        supervisor.close()


def test_supervisor_removes_private_socket_directory_when_command_builder_fails(
    tmp_path: Path,
) -> None:
    """参数构造失败发生在 spawn 前，也不得遗留认证目录或记录路径。"""
    import pytest

    from slope_sim.interfaces.v2.runsim_command_supervisor import RunSimCommandSupervisor

    def failing_builder(_launch):
        raise RuntimeError("cannot construct command")

    with pytest.raises(RuntimeError, match="cannot construct command"):
        RunSimCommandSupervisor.launch(failing_builder, socket_parent=tmp_path)

    assert tuple(tmp_path.iterdir()) == ()


def test_supervisor_close_kills_a_command_process_that_ignores_sigterm(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """窗口退出不能因 stubborn Command 泄漏异常或留下进程组。"""
    from slope_sim.interfaces.v2.runsim_command_supervisor import RunSimCommandSupervisor

    class StubbornProcess:
        pid = 4816

        def __init__(self) -> None:
            self.wait_timeouts: list[float] = []

        def poll(self):
            return None

        def wait(self, *, timeout: float):
            self.wait_timeouts.append(timeout)
            if len(self.wait_timeouts) < 3:
                raise subprocess.TimeoutExpired("command", timeout)
            return -signal.SIGKILL

    process = StubbornProcess()
    socket_dir = tmp_path / "private"
    socket_dir.mkdir(mode=0o700)
    session_closed: list[bool] = []
    supervisor = RunSimCommandSupervisor(
        process=process,
        session=SimpleNamespace(close=lambda: session_closed.append(True)),
        _socket_dir=socket_dir,
    )
    signals: list[tuple[int, signal.Signals]] = []
    monkeypatch.setattr(
        "slope_sim.interfaces.v2.runsim_command_supervisor.os.killpg",
        lambda pid, sent_signal: signals.append((pid, sent_signal)),
    )

    supervisor.close()

    assert session_closed == [True]
    assert signals == [(process.pid, signal.SIGTERM), (process.pid, signal.SIGKILL)]
    assert process.wait_timeouts == [1.0, 1.0]
    assert not socket_dir.exists()
