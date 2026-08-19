"""阶段四 runtime 协议控制器：串行化 transport、命令权、邮箱和重建事务。"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from threading import Condition
import time

from slope_sim.interfaces.ecal_transport import ProtocolConflictError
from slope_sim.interfaces.v2.authority import AuthoritySnapshot, CommandAuthority
from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.session import OutputIdentity, ProtocolSession
from slope_sim.interfaces.wheel import WheelCommandMailbox
from slope_sim.model_registry import RobotModelSpec


@dataclass(frozen=True)
class V2RuntimeSnapshot:
    """控制器在单一生命周期时刻组合出的不可变协议状态。"""

    simulation_session_id: bytes
    descriptor_sha256: bytes
    world_generation: int
    command_generation: int
    lifecycle_generation: int
    subscription_token: int
    command_protocol_state: str
    authority: AuthoritySnapshot
    closed: bool
    fatal_error: str | None


@dataclass(frozen=True)
class IngressToken:
    """将 callback 捕获的生命周期、订阅与邮箱代际一次冻结。"""

    lifecycle_generation: int
    subscription_token: int
    mailbox_generation: int


class V2RuntimeProtocol:
    """组合现有 session、authority 和 mailbox；不复制任一底层规则。"""

    def __init__(
        self,
        model: RobotModelSpec,
        *,
        transport: object,
        descriptor: DescriptorIdentity,
        monotonic: Callable[[], float] = time.monotonic,
        timeout_sec: float = 0.100,
        session_id_factory: Callable[[], bytes] | None = None,
    ) -> None:
        if not isinstance(model, RobotModelSpec):
            raise ValueError("model must be a RobotModelSpec")
        if not isinstance(descriptor, DescriptorIdentity):
            raise ValueError("descriptor must be a DescriptorIdentity")
        if not callable(monotonic):
            raise ValueError("monotonic must be callable")
        if session_id_factory is not None and not callable(session_id_factory):
            raise ValueError("session_id_factory must be callable or None")
        self._model = model
        self._transport = transport
        # 编排层可为 Recorder/Simulator 提供同一会话身份；默认仍由 ProtocolSession 随机生成。
        self._session = (
            ProtocolSession(descriptor)
            if session_id_factory is None
            else ProtocolSession(descriptor, session_id_factory=session_id_factory)
        )
        self._authority = CommandAuthority(self._session)
        self._mailbox = WheelCommandMailbox(model, timeout_sec=timeout_sec)
        self._codec = V2ProtoCodec(descriptor)
        self._monotonic = monotonic
        self._condition = Condition()
        self._lifecycle_generation = 0
        self._subscription_token = 0
        self._command_protocol_state = "not_checked"
        self._closed = False
        self._fatal_error: str | None = None

    @property
    def mailbox(self) -> WheelCommandMailbox:
        """暴露既有线程安全 mailbox，物理主线程只读取 decision/snapshot。"""
        return self._mailbox

    def refresh_transport(self) -> object:
        """先刷新 discovery，随后读取同一轮不可变 transport quality。"""
        with self._condition:
            if self._closed:
                raise RuntimeError("v2 runtime protocol is closed")
            if self._fatal_error is not None:
                raise RuntimeError(f"v2 runtime protocol is fatal: {self._fatal_error}")
        poll = getattr(self._transport, "poll_peer_state", None)
        if not callable(poll):
            raise RuntimeError("v2 transport must expose poll_peer_state")
        try:
            poll()
        except ProtocolConflictError:
            snapshot = self._transport.snapshot()
            self._apply_transport_snapshot(snapshot)
            raise
        snapshot = self._transport.snapshot()
        self._apply_transport_snapshot(snapshot)
        return snapshot

    def _apply_transport_snapshot(self, snapshot: object) -> None:
        """将 poll 已提交的命令质量原子投影为 authority/mailbox 状态。"""
        command_quality = next(
            item
            for item in snapshot.topic_quality
            if item.topic == "/sim/wheel/command"
        )
        with self._condition:
            try:
                if command_quality.protocol_state == "verified":
                    if command_quality.peer_count is None:
                        raise RuntimeError("verified command topic requires exact peer_count")
                    transition = self._authority.observe_peer_count(command_quality.peer_count)
                    if transition.clear_mailbox:
                        self._mailbox.clear()
                    self._command_protocol_state = "verified"
                else:
                    self._apply_unverified_protocol(
                        command_quality.protocol_state,
                        command_quality.peer_count,
                    )
            except OverflowError as error:
                self._enter_fatal_locked(error)
                raise

    def _enter_fatal_locked(self, error: OverflowError) -> None:
        """将 authority/session 的不可恢复代际耗尽原子投影为 controller fatal。"""
        if self._fatal_error is None:
            self._fatal_error = str(error)
            self._lifecycle_generation += 1
            self._subscription_token += 1
            self._mailbox.clear()
            self._authority.enter_fatal()

    def _apply_unverified_protocol(self, state: str, peer_count: int | None) -> None:
        """离开 verified 时只撤权一次；其余未验证轮次只更新精确 count。"""
        if peer_count is None:
            raise RuntimeError("raw v2 command quality requires exact peer_count")
        transition = (
            self._authority.suspend_protocol(peer_count)
            if self._command_protocol_state == "verified"
            else self._authority.observe_peer_count(peer_count)
        )
        if transition.clear_mailbox:
            self._mailbox.clear()
        self._command_protocol_state = state

    def snapshot(self) -> V2RuntimeSnapshot:
        """返回 session、authority 与 lifecycle 的一致不可变观测。"""
        with self._condition:
            return V2RuntimeSnapshot(
                simulation_session_id=self._session.simulation_session_id,
                descriptor_sha256=self._session.descriptor_sha256,
                world_generation=self._session.world_generation,
                command_generation=self._session.command_generation,
                lifecycle_generation=self._lifecycle_generation,
                subscription_token=self._subscription_token,
                command_protocol_state=self._command_protocol_state,
                authority=self._authority.snapshot(),
                closed=self._closed,
                fatal_error=self._fatal_error,
            )

    def capture_ingress(self) -> IngressToken:
        """在 callback 解码前原子捕获全部会使其失效的代际。"""
        with self._condition:
            return IngressToken(
                self._lifecycle_generation,
                self._subscription_token,
                self._mailbox.capture_generation(),
            )

    def accept_decoded_command(
        self,
        command: object,
        *,
        received_at: float,
        ingress: IngressToken,
    ) -> bool:
        """仅让 verified 的当前 ingress 通过 authority 原子写入 mailbox 并认领。"""
        with self._condition:
            if (
                self._closed
                or self._fatal_error is not None
                or self._command_protocol_state != "verified"
                or not isinstance(ingress, IngressToken)
                or ingress.lifecycle_generation != self._lifecycle_generation
                or ingress.subscription_token != self._subscription_token
                or ingress.mailbox_generation != self._mailbox.capture_generation()
            ):
                return False
            try:
                transition = self._authority.accept(
                    command,
                    self._model,
                    commit=lambda: self._mailbox.accept(
                        command.to_v1_motion(),
                        received_at=received_at,
                        generation=ingress.mailbox_generation,
                    ),
                )
            except OverflowError as error:
                self._enter_fatal_locked(error)
                raise
            if transition.clear_mailbox:
                self._mailbox.clear()
            return transition.accepted

    def accept_payload(self, payload: bytes, *, received_at: float) -> bool:
        """在锁外解码 raw command，再用捕获 token 回锁阻止迟到提交。"""
        ingress = self.capture_ingress()
        try:
            command = self._codec.decode_wheel_command(payload)
        except (TypeError, ValueError):
            return False
        return self.accept_decoded_command(
            command,
            received_at=received_at,
            ingress=ingress,
        )

    def prepare_world_rebuild(self) -> None:
        """先失效 callback token，再准备 authority/world 重建事务。"""
        with self._condition:
            self._lifecycle_generation += 1
            self._subscription_token += 1
            try:
                transition = self._authority.prepare_world_rebuild()
            except OverflowError as error:
                self._enter_fatal_locked(error)
                raise
            if transition.clear_mailbox:
                self._mailbox.clear()

    def abort_world_rebuild(self) -> None:
        """结束 prepared 状态但绝不恢复任何旧 ingress 或命令 token。"""
        with self._condition:
            self._authority.abort_world_rebuild()
            self._subscription_token += 1

    def fault_world_rebuild(self) -> None:
        """终结 prepared 重建故障，并保持 prepare 后的命令代际失效。"""
        with self._condition:
            self._authority.fault_world_rebuild()
            self._subscription_token += 1

    def reserve_output(self, topic: str) -> object:
        """仅在有效 lifecycle 中向唯一 session 预留输出序号。"""
        with self._condition:
            if self._closed:
                raise RuntimeError("v2 runtime protocol is closed")
            if self._fatal_error is not None:
                raise RuntimeError("v2 runtime protocol is unavailable")
            return self._session.reserve_output(topic)

    def reserve_outputs(self, topics: tuple[str, ...]) -> tuple[OutputIdentity, ...]:
        """在一个 lifecycle 临界区预留同帧多条输出，防止混合 world generation。"""
        if not isinstance(topics, tuple) or not topics or any(
            not isinstance(topic, str) for topic in topics
        ):
            raise ValueError("topics must be a nonempty tuple of topic strings")
        if len(set(topics)) != len(topics):
            raise ValueError("topics must not contain duplicates")
        with self._condition:
            if self._closed:
                raise RuntimeError("v2 runtime protocol is closed")
            if self._fatal_error is not None:
                raise RuntimeError("v2 runtime protocol is unavailable")
            return tuple(self._session.reserve_output(topic) for topic in topics)

    def commit_world_rebuild(self) -> int:
        """提交已准备的 world；仅此路径推进 world generation 并重置序号。"""
        with self._condition:
            return self._authority.commit_world_rebuild()

    def close(self) -> None:
        """先关闭 controller 入口，再在锁外释放可能回调的 transport。"""
        overflow: OverflowError | None = None
        with self._condition:
            if self._closed:
                return
            self._closed = True
            self._lifecycle_generation += 1
            self._subscription_token += 1
            try:
                transition = self._authority.suspend_protocol(0)
                if transition.clear_mailbox:
                    self._mailbox.clear()
            except OverflowError as error:
                self._enter_fatal_locked(error)
                overflow = error
        close = getattr(self._transport, "close", None)
        if not callable(close):
            raise RuntimeError("v2 transport must expose close")
        close()
        if overflow is not None:
            raise overflow
