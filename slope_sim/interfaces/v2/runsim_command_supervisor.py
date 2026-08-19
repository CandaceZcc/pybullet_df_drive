"""runSim v2：唯一 C++ Command 子进程的认证启动与受监管回收。"""
from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import secrets
import shutil
import signal
import subprocess
import tempfile
import time
from typing import Callable

from slope_sim.interfaces.v2.runsim_session import RunSimSession


def _terminate_process_group(process: subprocess.Popen[object]) -> None:
    """有界回收唯一 Command；SIGTERM 无响应时升级 SIGKILL 且不掩盖主失败。"""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            # SIGKILL 已经发出；退出路径不因 wait 竞态跳过认证目录清理。
            pass


@dataclass(frozen=True)
class RunSimCommandLaunch:
    """spawn 前可安全交给 Command 参数构造器的固定会话信息。"""

    launch_record_path: Path
    simulation_session_id: bytes


@dataclass
class RunSimCommandSupervisor:
    """持有单个 Command 进程及其由 child 发布的认证会话。"""

    process: subprocess.Popen[object]
    session: RunSimSession
    _socket_dir: Path

    @classmethod
    def launch(
        cls,
        command_argv: list[str] | Callable[[RunSimCommandLaunch], list[str]],
        *,
        socket_parent: Path,
        startup_timeout_sec: float = 5.0,
    ) -> "RunSimCommandSupervisor":
        """以 child PID 原子发布认证记录，父进程只在其存在后 attach。"""
        if not isinstance(socket_parent, Path) or not socket_parent.is_absolute() or not socket_parent.is_dir():
            raise ValueError("socket_parent must be an existing absolute directory")
        if type(startup_timeout_sec) not in {int, float} or startup_timeout_sec <= 0:
            raise ValueError("startup_timeout_sec must be positive")
        socket_dir = Path(tempfile.mkdtemp(prefix="runsim-command-", dir=socket_parent))
        os.chmod(socket_dir, 0o700)
        session_id, token = secrets.token_bytes(16), secrets.token_bytes(32)
        record = socket_dir / "command.launch.lock"
        if callable(command_argv):
            try:
                argv = command_argv(RunSimCommandLaunch(record, session_id))
            except BaseException:
                shutil.rmtree(socket_dir, ignore_errors=True)
                raise
        else:
            argv = command_argv
        if not isinstance(argv, list) or not argv or any(
            not isinstance(item, str) or not item for item in argv
        ):
            shutil.rmtree(socket_dir, ignore_errors=True)
            raise ValueError("command_argv must be a nonempty list of strings")

        def publish_child_record() -> None:
            RunSimSession.create(
                socket_dir,
                command_pid=os.getpid(),
                command_uid=os.getuid(),
                orchestrator_pid=os.getppid(),
                session_id_factory=lambda: session_id,
                token_factory=lambda: token,
            )

        process = subprocess.Popen(argv, preexec_fn=publish_child_record, start_new_session=True)
        deadline = time.monotonic() + float(startup_timeout_sec)
        try:
            while not record.exists() and process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.01)
            if not record.exists():
                raise RuntimeError("Command did not publish its launch record")
            session = RunSimSession.attach(
                record,
                command_pid=process.pid,
                command_uid=os.getuid(),
                orchestrator_pid=os.getpid(),
            )
            if session.server_authentication["session_id"] != session_id.hex():
                raise RuntimeError("Command launch record session identity differs from supervisor")
            return cls(process, session, socket_dir)
        except BaseException:
            if process.poll() is None:
                _terminate_process_group(process)
            shutil.rmtree(socket_dir, ignore_errors=True)
            raise

    def close(self) -> None:
        """先撤销会话，再有界终止唯一 child 进程组并删除自建目录。"""
        self.session.close()
        _terminate_process_group(self.process)
        shutil.rmtree(self._socket_dir, ignore_errors=True)
