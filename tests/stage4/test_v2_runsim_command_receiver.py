"""runSim v2 command receiver 的 latest-only 与故障边界。"""
from __future__ import annotations

import threading
from types import SimpleNamespace

import pytest

from slope_sim.interfaces.v2.runsim_command_receiver import RunSimCommandReceiver


def test_command_receiver_reports_status_pipe_eof_as_unexpected_exit() -> None:
    """sidecar 无状态帧退出时不得继续返回共享内存中的旧命令。"""

    class Process:
        pid = 123

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

    receiver = RunSimCommandReceiver(
        Process(),
        bytearray(b"stale"),
        SimpleNamespace(value=5),
        SimpleNamespace(value=1),
        SimpleNamespace(value=10.0),
        threading.Lock(),
        SimpleNamespace(),
        StatusReceiver(),
    )

    with pytest.raises(RuntimeError, match="exited unexpectedly"):
        receiver.take_latest(0)


def test_command_receiver_returns_only_newer_latest_shared_frame() -> None:
    """主进程恢复后只能取得当前版本一次，不能回放中间或旧 payload。"""

    class Process:
        pid = 123

        @staticmethod
        def is_alive() -> bool:
            return True

    class StatusReceiver:
        @staticmethod
        def poll() -> bool:
            return False

    receiver = RunSimCommandReceiver(
        Process(),
        bytearray(b"latest"),
        SimpleNamespace(value=6),
        SimpleNamespace(value=9),
        SimpleNamespace(value=12.5),
        threading.Lock(),
        SimpleNamespace(),
        StatusReceiver(),
    )

    assert receiver.take_latest(8) == (9, b"latest", 12.5)
    assert receiver.take_latest(9) is None
