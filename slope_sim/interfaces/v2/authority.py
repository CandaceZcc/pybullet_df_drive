"""阶段四命令权状态机：以精确 peer count 管理唯一 wheel 命令来源。"""
from __future__ import annotations

from dataclasses import dataclass
from threading import Lock
from typing import Callable

from slope_sim.interfaces.models import validate_wheel_command
from slope_sim.interfaces.v2.models import CommandAuthorityState, WheelCommandV2, require_uint
from slope_sim.interfaces.v2.session import ProtocolSession
from slope_sim.model_registry import RobotModelSpec


_UINT32_MAX = (1 << 32) - 1


@dataclass(frozen=True)
class AuthoritySnapshot:
    """当前命令权的不可变观测快照。"""

    state: CommandAuthorityState
    peer_count: int
    command_generation: int
    owner_source_id: str | None
    owner_source_session_id: bytes | None
    last_sequence: int | None


@dataclass(frozen=True)
class CommandAcceptance:
    """一次命令权处理的结果及对 runtime/mailbox 的安全指示。"""

    accepted: bool
    reason: str
    clear_mailbox: bool = False
    safe_stop: bool = False
    claimed_owner: bool = False


class CommandAuthority:
    """在唯一 publisher 条件下认领并维护连续 v2 wheel command。"""

    def __init__(self, session: ProtocolSession) -> None:
        if not isinstance(session, ProtocolSession):
            raise ValueError("session must be a ProtocolSession")
        self._session = session
        self._peer_count = 0
        self._state = CommandAuthorityState.WAITING
        self._owner: tuple[str, bytes] | None = None
        self._last_sequence: int | None = None
        self._rebuild_prepared = False
        self._lock = Lock()

    def snapshot(self) -> AuthoritySnapshot:
        """在同一 authority 锁内复制完整的可观测命令权状态。"""
        with self._lock:
            owner_source_id = self._owner[0] if self._owner is not None else None
            owner_source_session_id = self._owner[1] if self._owner is not None else None
            return AuthoritySnapshot(
                state=self._state,
                peer_count=self._peer_count,
                command_generation=self._session.command_generation,
                owner_source_id=owner_source_id,
                owner_source_session_id=owner_source_session_id,
                last_sequence=self._last_sequence,
            )

    def observe_peer_count(self, count: int) -> CommandAcceptance:
        """记录 discovery 的精确数量，并只在唯一 peer 离开时撤权。"""
        normalized = require_uint("command_peer_count", count, _UINT32_MAX)
        with self._lock:
            previous = self._peer_count
            if previous == 1 and normalized != 1:
                # 先推进 token；耗尽时不可留下半次状态转换。
                self._session.advance_command_generation()
                self._peer_count = normalized
                self._clear_owner_locked()
                self._state = self._state_for_unowned_peer_count()
                return CommandAcceptance(False, "command peer edge", True, True)
            self._peer_count = normalized
            if normalized == 1:
                self._state = (
                    CommandAuthorityState.ACTIVE
                    if self._owner is not None
                    else CommandAuthorityState.CLAIMABLE
                )
            else:
                self._clear_owner_locked()
                self._state = self._state_for_unowned_peer_count()
            return CommandAcceptance(False, "peer observation")

    def suspend_protocol(self, count: int) -> CommandAcceptance:
        """由已验证协议离开边沿调用，精确撤销一次命令 token。"""
        normalized = require_uint("command_peer_count", count, _UINT32_MAX)
        with self._lock:
            self._session.advance_command_generation()
            self._peer_count = normalized
            self._clear_owner_locked()
            self._state = self._state_for_unowned_peer_count()
            return CommandAcceptance(False, "command protocol suspended", True, True)

    def accept(
        self,
        command: WheelCommandV2,
        model: RobotModelSpec,
        *,
        commit: Callable[[], bool],
    ) -> CommandAcceptance:
        """按固定身份和连续性顺序验证后，提交成功才认领或推进序号。"""
        with self._lock:
            if self._rebuild_prepared:
                return CommandAcceptance(False, "world rebuild is prepared")
            if self._peer_count != 1:
                return CommandAcceptance(False, "command peer count is not uniquely claimable")
            rejection = self._command_rejection_reason_locked(command, model)
            if rejection is not None:
                return CommandAcceptance(False, rejection)

            owner = (command.source_id, command.source_session_id)
            if self._owner is None:
                # metadata pending 期间的帧必须 fail-closed 丢弃；首个 verified 帧可
                # 能已经不是发布者的 sequence 0，仍应成为该连续流的基准。
                if commit() is not True:
                    return CommandAcceptance(False, "mailbox commit rejected")
                self._owner = owner
                self._last_sequence = command.sequence
                self._state = CommandAuthorityState.ACTIVE
                return CommandAcceptance(True, "command owner claimed", claimed_owner=True)

            if owner != self._owner:
                # 先推进 token；失败时 owner、序号和状态必须保持原样。
                self._session.advance_command_generation()
                self._clear_owner_locked()
                self._state = self._state_for_unowned_peer_count()
                return CommandAcceptance(False, "command owner changed", True, True)

            assert self._last_sequence is not None
            if command.sequence <= self._last_sequence:
                return CommandAcceptance(False, "command sequence must advance")
            if commit() is not True:
                return CommandAcceptance(False, "mailbox commit rejected")
            self._last_sequence = command.sequence
            return CommandAcceptance(True, "command accepted")

    def prepare_world_rebuild(self) -> CommandAcceptance:
        """准备 world 重建时撤销 owner，并令旧命令代际立即失效。"""
        with self._lock:
            self._session.prepare_world_rebuild()
            self._rebuild_prepared = True
            self._clear_owner_locked()
            self._state = self._state_for_unowned_peer_count()
            return CommandAcceptance(False, "world rebuild prepared", True, True)

    def commit_world_rebuild(self) -> int:
        """提交已准备的 world，保持最新 peer 推导出的无 owner 状态。"""
        with self._lock:
            generation = self._session.commit_world_rebuild()
            self._rebuild_prepared = False
            self._clear_owner_locked()
            self._state = self._state_for_unowned_peer_count()
            return generation

    def abort_world_rebuild(self) -> None:
        """中止重建且不恢复之前 owner、序号或 command token。"""
        with self._lock:
            self._session.abort_world_rebuild()
            self._rebuild_prepared = False
            self._clear_owner_locked()
            self._state = self._state_for_unowned_peer_count()

    def fault_world_rebuild(self) -> None:
        """处理重建故障且不恢复之前 owner、序号或 command token。"""
        with self._lock:
            self._session.fault_world_rebuild()
            self._rebuild_prepared = False
            self._clear_owner_locked()
            self._state = self._state_for_unowned_peer_count()

    def enter_fatal(self) -> None:
        """代际耗尽时无条件撤销 owner，且不再尝试推进已耗尽 token。"""
        with self._lock:
            self._peer_count = 0
            self._clear_owner_locked()
            self._state = CommandAuthorityState.WAITING

    def _command_rejection_reason_locked(
        self,
        command: WheelCommandV2,
        model: RobotModelSpec,
    ) -> str | None:
        """按线协议规定的身份次序拒绝无效命令，绝不触发 commit。"""
        if not isinstance(command, WheelCommandV2):
            return "command must be a WheelCommandV2"
        if not isinstance(model, RobotModelSpec):
            return "model must be a RobotModelSpec"
        if command.simulation_session_id != self._session.simulation_session_id:
            return "simulation session does not match"
        if command.descriptor_sha256 != self._session.descriptor_sha256:
            return "descriptor SHA-256 does not match"
        if command.world_generation != self._session.world_generation:
            return "world generation does not match"
        if command.command_generation != self._session.command_generation:
            return "command generation does not match"
        if command.robot_model != model.name:
            return "robot model does not match"
        try:
            validate_wheel_command(command.to_v1_motion(), model)
        except ValueError as error:
            return str(error)
        return None

    def _state_for_unowned_peer_count(self) -> CommandAuthorityState:
        """把精确 peer count 映射为无 owner 的四态基础状态。"""
        if self._peer_count == 0:
            return CommandAuthorityState.WAITING
        if self._peer_count == 1:
            return CommandAuthorityState.CLAIMABLE
        return CommandAuthorityState.CONFLICT

    def _clear_owner_locked(self) -> None:
        """调用方持 authority 锁时同时丢弃 owner 和其序号历史。"""
        self._owner = None
        self._last_sequence = None
