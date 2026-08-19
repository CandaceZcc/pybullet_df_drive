"""阶段四仿真会话：管理进程身份、world/command 代际和逐话题序号。"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Callable
from uuid import uuid4

from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.models import require_fixed_bytes
from slope_sim.interfaces.v2.topics import V2_OUTPUT_TOPICS


_UINT64_MAX = (1 << 64) - 1


@dataclass(frozen=True)
class OutputIdentity:
    """一次输出在固定会话和 world 中的不可变连续性身份。"""

    topic: str
    simulation_session_id: bytes
    descriptor_sha256: bytes
    world_generation: int
    sequence: int


def _new_session_id() -> bytes:
    """为每个进程会话提供不可预测且固定长度的身份。"""
    return uuid4().bytes


class ProtocolSession:
    """串行化 v2 会话身份、world 重建事务和输出序号分配。"""

    def __init__(
        self,
        descriptor: DescriptorIdentity,
        *,
        session_id_factory: Callable[[], bytes] = _new_session_id,
    ) -> None:
        self._descriptor = descriptor
        self._simulation_session_id = require_fixed_bytes(
            "simulation_session_id", session_id_factory(), 16
        )
        self._world_generation = 1
        self._command_generation = 1
        self._next_sequence = {topic: 0 for topic in V2_OUTPUT_TOPICS}
        self._rebuild_prepared = False
        self._lock = Lock()

    @property
    def simulation_session_id(self) -> bytes:
        """返回当前进程固定的会话身份副本。"""
        with self._lock:
            return self._simulation_session_id

    @property
    def descriptor_sha256(self) -> bytes:
        """返回当前冻结 descriptor 的 SHA-256 身份。"""
        with self._lock:
            return self._descriptor.sha256

    @property
    def world_generation(self) -> int:
        """返回最近一次成功 world 提交后的代际。"""
        with self._lock:
            return self._world_generation

    @property
    def command_generation(self) -> int:
        """返回当前命令权代际。"""
        with self._lock:
            return self._command_generation

    def reserve_output(self, topic: str) -> OutputIdentity:
        """在传感器读取前占用序号；后续失败也不回收。"""
        with self._lock:
            if topic not in self._next_sequence:
                raise ValueError(f"unknown v2 output topic: {topic}")
            sequence = self._next_sequence[topic]
            if sequence > _UINT64_MAX:
                raise OverflowError("v2 output sequence exhausted")
            self._next_sequence[topic] = sequence + 1
            return OutputIdentity(
                topic=topic,
                simulation_session_id=self._simulation_session_id,
                descriptor_sha256=self._descriptor.sha256,
                world_generation=self._world_generation,
                sequence=sequence,
            )

    def advance_command_generation(self) -> int:
        """推进命令权代际，供后续 authority 状态机原子调用。"""
        with self._lock:
            self._advance_command_generation_locked()
            return self._command_generation

    def prepare_world_rebuild(self) -> int:
        """开始重建事务并立刻使旧命令代际失效。"""
        with self._lock:
            if self._rebuild_prepared:
                raise RuntimeError("world rebuild is already prepared")
            self._advance_command_generation_locked()
            self._rebuild_prepared = True
            return self._command_generation

    def commit_world_rebuild(self) -> int:
        """仅在已准备事务中提交新 world，并重置全部输出序号。"""
        with self._lock:
            if not self._rebuild_prepared:
                raise RuntimeError("world rebuild is not prepared")
            if self._world_generation == _UINT64_MAX:
                raise OverflowError("world_generation exhausted")
            self._world_generation += 1
            self._next_sequence = {topic: 0 for topic in V2_OUTPUT_TOPICS}
            self._rebuild_prepared = False
            return self._world_generation

    def abort_world_rebuild(self) -> None:
        """取消已准备重建；已推进的命令代际不能恢复。"""
        with self._lock:
            if not self._rebuild_prepared:
                raise RuntimeError("world rebuild is not prepared")
            self._rebuild_prepared = False

    def fault_world_rebuild(self) -> None:
        """故障路径结束重建准备态，但保留失效后的命令代际。"""
        with self._lock:
            if not self._rebuild_prepared:
                raise RuntimeError("world rebuild is not prepared")
            self._rebuild_prepared = False

    def _advance_command_generation_locked(self) -> None:
        """在持锁状态下检查 uint64 边界后推进命令代际。"""
        if self._command_generation == _UINT64_MAX:
            raise OverflowError("command_generation exhausted")
        self._command_generation += 1
