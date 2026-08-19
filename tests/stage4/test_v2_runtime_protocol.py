"""阶段四 A Task 12：v2 runtime 协议控制器的生命周期与命令权组合。"""
from importlib import import_module

import pytest

from slope_sim.interfaces.transport import TransportSnapshot, TransportTopicQuality
from slope_sim.model_registry import get_robot_model


def require_wished_module(name: str):
    """让未实现控制器以可读行为失败，而不是破坏 pytest 收集。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


class FakeV2Transport:
    """只提供 Task 12 首个 poll/snapshot 顺序测试所需的 transport 表面。"""

    def __init__(self) -> None:
        self.calls: list[str] = []
        self.closed = False
        self.command_protocol_state = "waiting"
        self.command_peer_count = 0

    def set_command_protocol(self, state: str, *, peer_count: int) -> None:
        """为每轮 fake discovery 设置 command topic 的已提交质量。"""
        self.command_protocol_state = state
        self.command_peer_count = peer_count

    def poll_peer_state(self) -> str:
        self.calls.append("poll_peer_state")
        return "waiting_peer"

    def snapshot(self) -> TransportSnapshot:
        self.calls.append("snapshot")
        metadata = (
            ("slope_sim.interfaces.v2.WheelCommand",),
            ("proto",),
            ("0" * 64,),
        ) if self.command_protocol_state == "verified" else (
            (
                "slope_sim.interfaces.v2.WheelCommand",
                "slope_sim.interfaces.v1.WheelCommand",
            ),
            ("proto", "proto"),
            ("0" * 64, "1" * 64),
        ) if self.command_protocol_state == "conflict" else ((), (), ())
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=False,
            published_count=0,
            received_count=0,
            error_count=0,
            dropped_count=0,
            topic_quality=(
                TransportTopicQuality(
                    topic="/sim/wheel/command",
                    peer_connected=self.command_peer_count > 0,
                    peer_count=self.command_peer_count,
                    protocol_state=self.command_protocol_state,
                    protocol_detail=(
                        "fixture protocol conflict"
                        if self.command_protocol_state == "conflict"
                        else ""
                    ),
                    remote_type_names=metadata[0],
                    remote_encodings=metadata[1],
                    remote_descriptor_sha256=metadata[2],
                ),
            ),
        )

    def close(self) -> None:
        """记录 controller 是否在自身状态冻结后释放 transport。"""
        self.closed = True


@pytest.fixture
def descriptor():
    """读取冻结 v2 descriptor，禁止测试使用伪造协议身份。"""
    return require_wished_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()


def test_refresh_polls_before_reading_command_peer_count(descriptor) -> None:
    """每轮 controller refresh 必须先完成 discovery，再读取同轮 quality snapshot。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    controller_type = getattr(module, "V2RuntimeProtocol", None)
    assert callable(controller_type), "V2RuntimeProtocol is not implemented"
    transport = FakeV2Transport()
    controller = controller_type(get_robot_model("df_mid"), transport=transport, descriptor=descriptor)

    transport.calls.clear()
    controller.refresh_transport()

    assert transport.calls[:2] == ["poll_peer_state", "snapshot"]


def test_verified_refresh_applies_exact_command_peer_count(descriptor) -> None:
    """verified discovery 的精确 count 必须成为 authority 的唯一认领前置状态。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    controller_type = getattr(module, "V2RuntimeProtocol", None)
    assert callable(controller_type), "V2RuntimeProtocol is not implemented"
    transport = FakeV2Transport()
    transport.set_command_protocol("verified", peer_count=1)
    controller = controller_type(get_robot_model("df_mid"), transport=transport, descriptor=descriptor)

    controller.refresh_transport()

    snapshot = controller.snapshot()
    assert snapshot.command_protocol_state == "verified"
    assert snapshot.authority.peer_count == 1
    assert snapshot.authority.state.name == "CLAIMABLE"


def test_verified_to_pending_revokes_once(descriptor) -> None:
    """verified 离开时仅撤权一次；重复 pending poll 不得反复推进 generation。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    transport = FakeV2Transport()
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=transport, descriptor=descriptor
    )
    transport.set_command_protocol("verified", peer_count=1)
    controller.refresh_transport()
    before = controller.snapshot().command_generation

    transport.set_command_protocol("pending", peer_count=1)
    controller.refresh_transport()
    controller.refresh_transport()

    snapshot = controller.snapshot()
    assert snapshot.command_protocol_state == "pending"
    assert snapshot.command_generation == before + 1
    assert snapshot.authority.peer_count == 1
    assert snapshot.authority.state.name == "CLAIMABLE"


def test_conflict_error_applies_committed_snapshot_before_reraising(descriptor) -> None:
    """transport 已提交 conflict 时必须先撤权和停车，再把异常交给调用方。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    models = require_wished_module("slope_sim.interfaces.v2.models")
    from slope_sim.interfaces.ecal_transport import ProtocolConflictError

    transport = FakeV2Transport()
    transport.set_command_protocol("verified", peer_count=1)
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=transport, descriptor=descriptor
    )
    controller.refresh_transport()
    identity = controller.snapshot()
    command = models.WheelCommandV2(
        timestamp_ns=1,
        drive_wheel_speed_rad_s=(1.0, 1.0),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=identity.world_generation,
        command_generation=identity.command_generation,
        source_id="owner",
        source_session_id=b"o" * 16,
        robot_model="df_mid",
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=descriptor.sha256,
    )
    assert controller.accept_decoded_command(
        command, received_at=1.0, ingress=controller.capture_ingress()
    )
    transport.set_command_protocol("conflict", peer_count=2)
    transport.poll_peer_state = lambda: (_ for _ in ()).throw(ProtocolConflictError("conflict"))

    with pytest.raises(ProtocolConflictError, match="conflict"):
        controller.refresh_transport()

    snapshot = controller.snapshot()
    assert snapshot.command_protocol_state == "conflict"
    assert snapshot.authority.state.name == "CONFLICT"
    assert snapshot.authority.owner_source_id is None
    assert controller.mailbox.decision(now=1.0).waiting is True


def test_prepare_and_abort_invalidate_captured_ingress(descriptor) -> None:
    """abort 不能复活 prepare 前捕获的 callback token。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    transport = FakeV2Transport()
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=transport, descriptor=descriptor
    )
    old_ingress = controller.capture_ingress()

    controller.prepare_world_rebuild()
    controller.abort_world_rebuild()

    assert controller.accept_decoded_command(
        object(), received_at=1.0, ingress=old_ingress
    ) is False


def test_verified_current_ingress_claims_owner_and_mailbox(descriptor) -> None:
    """完整有效 command 必须先写邮箱，成功后才成为唯一 owner。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    models = require_wished_module("slope_sim.interfaces.v2.models")
    transport = FakeV2Transport()
    transport.set_command_protocol("verified", peer_count=1)
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=transport, descriptor=descriptor
    )
    controller.refresh_transport()
    identity = controller.snapshot()
    command = models.WheelCommandV2(
        timestamp_ns=20_000_000,
        drive_wheel_speed_rad_s=(1.25, -1.25),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=identity.world_generation,
        command_generation=identity.command_generation,
        source_id="manual.tool-1",
        source_session_id=b"o" * 16,
        robot_model="df_mid",
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=descriptor.sha256,
    )

    assert controller.accept_decoded_command(
        command, received_at=1.0, ingress=controller.capture_ingress()
    ) is True
    assert controller.snapshot().authority.owner_source_id == "manual.tool-1"
    assert controller.mailbox.decision(now=1.0).waiting is False


def test_commit_alone_advances_world_and_resets_output_sequences(descriptor) -> None:
    """只有 commit world rebuild 才递增 world 并使同话题 sequence 从零重启。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=FakeV2Transport(), descriptor=descriptor
    )
    before = controller.reserve_output("/sim/wheel/state")

    controller.prepare_world_rebuild()
    controller.commit_world_rebuild()

    after = controller.reserve_output("/sim/wheel/state")
    assert before.world_generation == 1 and before.sequence == 0
    assert after.world_generation == 2 and after.sequence == 0


def test_close_invalidates_lifecycle_before_releasing_transport(descriptor) -> None:
    """关闭必须先拒绝新入口，再释放底层 transport。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    transport = FakeV2Transport()
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=transport, descriptor=descriptor
    )

    controller.close()

    assert controller.snapshot().closed is True
    assert transport.closed is True
    with pytest.raises(RuntimeError, match="closed"):
        controller.reserve_output("/sim/wheel/state")
    with pytest.raises(RuntimeError, match="closed"):
        controller.refresh_transport()
    assert transport.calls == []


def test_fault_world_rebuild_does_not_restore_command_generation(descriptor) -> None:
    """重建 fault 结束 prepared 状态，但 prepare 已撤销的 command token 永不恢复。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=FakeV2Transport(), descriptor=descriptor
    )
    before = controller.snapshot().command_generation
    controller.prepare_world_rebuild()

    controller.fault_world_rebuild()

    assert controller.snapshot().command_generation == before + 1


def test_accept_payload_rejects_invalid_wire_without_claiming(descriptor) -> None:
    """解码失败的 raw payload 只能被安全拒绝，不能产生 owner。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    transport = FakeV2Transport()
    transport.set_command_protocol("verified", peer_count=1)
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=transport, descriptor=descriptor
    )
    controller.refresh_transport()

    assert controller.accept_payload(b"not-a-v2-command", received_at=1.0) is False
    assert controller.snapshot().authority.owner_source_id is None


def test_generation_exhaustion_enters_terminal_fatal_state(descriptor) -> None:
    """authority 代际耗尽后 controller 必须锁存 fatal 并拒绝后续 ingress。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    transport = FakeV2Transport()
    transport.set_command_protocol("verified", peer_count=1)
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=transport, descriptor=descriptor
    )
    controller.refresh_transport()
    with controller._session._lock:
        controller._session._command_generation = (1 << 64) - 1
    transport.set_command_protocol("pending", peer_count=1)

    with pytest.raises(OverflowError, match="command_generation exhausted"):
        controller.refresh_transport()

    failed = controller.snapshot()
    assert failed.fatal_error == "command_generation exhausted"
    assert controller.accept_payload(b"not-a-v2-command", received_at=1.0) is False


@pytest.mark.parametrize("operation", ("prepare_world_rebuild", "close"))
def test_generation_exhaustion_in_lifecycle_paths_enters_fatal_state(
    descriptor, operation: str
) -> None:
    """重建和关闭中的 token 耗尽也必须 fail-closed，不能留下可用命令权。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    transport = FakeV2Transport()
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=transport, descriptor=descriptor
    )
    with controller._session._lock:
        controller._session._command_generation = (1 << 64) - 1

    with pytest.raises(OverflowError, match="command_generation exhausted"):
        getattr(controller, operation)()

    failed = controller.snapshot()
    assert failed.fatal_error == "command_generation exhausted"
    assert failed.authority.state.name == "WAITING"
    assert controller.mailbox.decision(now=1.0).waiting is True
    if operation == "close":
        assert transport.closed is True


def test_generation_exhaustion_on_owner_change_enters_fatal_state(descriptor) -> None:
    """错误 owner 触发的代际耗尽也必须撤权并停车。"""
    module = require_wished_module("slope_sim.interfaces.v2.runtime_protocol")
    models = require_wished_module("slope_sim.interfaces.v2.models")
    transport = FakeV2Transport()
    transport.set_command_protocol("verified", peer_count=1)
    controller = module.V2RuntimeProtocol(
        get_robot_model("df_mid"), transport=transport, descriptor=descriptor
    )
    controller.refresh_transport()
    identity = controller.snapshot()
    with controller._authority._lock:
        controller._authority._owner = ("owner", b"o" * 16)
        controller._authority._last_sequence = 0
        controller._authority._state = models.CommandAuthorityState.ACTIVE
    with controller._session._lock:
        controller._session._command_generation = (1 << 64) - 1
    command = models.WheelCommandV2(
        timestamp_ns=1,
        drive_wheel_speed_rad_s=(1.0, 1.0),
        steering_wheel_speed_rad_s=(),
        sequence=1,
        world_generation=identity.world_generation,
        command_generation=(1 << 64) - 1,
        source_id="other",
        source_session_id=b"x" * 16,
        robot_model="df_mid",
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=descriptor.sha256,
    )

    with pytest.raises(OverflowError, match="command_generation exhausted"):
        controller.accept_decoded_command(
            command, received_at=1.0, ingress=controller.capture_ingress()
        )

    failed = controller.snapshot()
    assert failed.fatal_error == "command_generation exhausted"
    assert failed.authority.state.name == "WAITING"
    assert controller.mailbox.decision(now=1.0).waiting is True
