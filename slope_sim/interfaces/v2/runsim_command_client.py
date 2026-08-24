"""runSim v2：GUI 向唯一 C++ Command 续租的受认证本机 socket client。"""
from __future__ import annotations

import json
import socket
import struct

from slope_sim.interfaces.v2.runsim_session import RunSimSession


class RunSimCommandClient:
    """持有单条 Command socket 连接；不发布 eCAL WheelCommand。"""

    def __init__(self, session: RunSimSession) -> None:
        if not isinstance(session, RunSimSession):
            raise ValueError("session must be a RunSimSession")
        self._session = session
        self._socket: socket.socket | None = None

    def connect(self) -> None:
        """连接并以 SO_PEERCRED 核验预期的 C++ Command 进程。"""
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
        if self._socket is None:
            raise RuntimeError("Command socket is not connected")
        payload = json.dumps(
            {
                "kind": "target",
                "token": self._session.server_authentication["token"],
                "linear_velocity_m_s": linear_velocity_m_s,
                "angular_velocity_rad_s": angular_velocity_rad_s,
            },
            separators=(",", ":"),
        ).encode("utf-8") + b"\n"
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
        if self._socket is None:
            raise RuntimeError("Command socket is not connected")
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
        client, self._socket = self._socket, None
        if client is not None:
            client.close()
        self._session.connection_closed()
