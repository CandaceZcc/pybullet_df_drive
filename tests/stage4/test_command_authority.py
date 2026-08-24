"""阶段四 A：验证 v2 精确 peer count 下的唯一轮子命令权。"""
from dataclasses import replace
from importlib import import_module

import pytest

from slope_sim.model_registry import get_robot_model


_UINT64_MAX = (1 << 64) - 1


def require_wished_module(name: str):
    """缺少 authority 时形成可读 RED，避免收集期导入错误。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


@pytest.fixture
def descriptor():
    """读取冻结 descriptor，保持协议身份校验真实可用。"""
    return require_wished_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()


@pytest.fixture
def session(descriptor):
    """每项状态机测试拥有独立且确定的 simulation session。"""
    protocol_session = require_wished_module("slope_sim.interfaces.v2.session").ProtocolSession
    return protocol_session(descriptor, session_id_factory=lambda: b"s" * 16)


@pytest.fixture
def model():
    """使用已有差速车型规则验证 wheel 数量和机械限位。"""
    return get_robot_model("df_mid")


@pytest.fixture
def valid_command(session, descriptor, model):
    """构造恰好匹配当前会话初始代际的完整 v2 命令。"""
    command_type = require_wished_module("slope_sim.interfaces.v2.models").WheelCommandV2
    return command_type(
        timestamp_ns=20_000_000,
        drive_wheel_speed_rad_s=(1.25, -1.25),
        steering_wheel_speed_rad_s=(),
        sequence=0,
        world_generation=session.world_generation,
        command_generation=session.command_generation,
        source_id="manual.tool-1",
        source_session_id=b"o" * 16,
        robot_model=model.name,
        simulation_session_id=session.simulation_session_id,
        descriptor_sha256=descriptor.sha256,
    )


def make_authority(session):
    """按测试延迟加载未实现的 public authority 入口。"""
    command_authority = require_wished_module("slope_sim.interfaces.v2.authority").CommandAuthority
    return command_authority(session)


def test_peer_edges_advance_generation_once_and_keep_exact_count(session) -> None:
    """从唯一 peer 离开时仅推进一次代际，并保留实际数量。"""
    states = require_wished_module("slope_sim.interfaces.v2.models").CommandAuthorityState
    authority = make_authority(session)
    assert authority.snapshot().state is states.WAITING
    authority.observe_peer_count(1)
    assert authority.snapshot().state is states.CLAIMABLE
    assert session.command_generation == 1
    authority.observe_peer_count(2)
    assert authority.snapshot().state is states.CONFLICT
    assert authority.snapshot().peer_count == 2
    assert session.command_generation == 2
    authority.observe_peer_count(3)
    authority.observe_peer_count(3)
    assert authority.snapshot().peer_count == 3
    assert session.command_generation == 2
    authority.observe_peer_count(1)
    assert authority.snapshot().state is states.CLAIMABLE
    assert session.command_generation == 2


def test_first_complete_valid_command_claims_and_other_owner_revokes(session, model, valid_command) -> None:
    """首个有效命令才认领；第二来源必须安全撤权。"""
    states = require_wished_module("slope_sim.interfaces.v2.models").CommandAuthorityState
    authority = make_authority(session)
    authority.observe_peer_count(1)
    accepted = authority.accept(valid_command, model, commit=lambda: True)
    assert accepted.accepted is True
    assert accepted.claimed_owner is True
    assert authority.snapshot().state is states.ACTIVE
    assert authority.snapshot().owner_source_id == valid_command.source_id

    intruder = replace(valid_command, source_id="other", source_session_id=b"i" * 16, sequence=1)
    rejected = authority.accept(intruder, model, commit=lambda: pytest.fail("must not commit"))
    assert rejected.accepted is False
    assert rejected.clear_mailbox is True
    assert rejected.safe_stop is True
    assert authority.snapshot().state is states.CLAIMABLE
    assert authority.snapshot().owner_source_id is None
    assert authority.snapshot().command_generation == 2


def test_first_delivered_command_after_metadata_pending_claims_owner(session, model, valid_command) -> None:
    """metadata fail-closed 丢弃首批帧后，首个已验证帧仍可认领。"""
    authority = make_authority(session)
    authority.observe_peer_count(1)

    delivered_after_verification = replace(valid_command, sequence=1581)
    accepted = authority.accept(delivered_after_verification, model, commit=lambda: True)

    assert accepted.accepted is True
    assert accepted.claimed_owner is True
    assert authority.snapshot().last_sequence == 1581


@pytest.mark.parametrize("action_name", ("peer_edge", "suspend", "wrong_owner"))
def test_generation_exhaustion_keeps_active_authority_atomic(action_name, session, model, valid_command) -> None:
    """所有撤权入口在 generation 耗尽时必须保持完整 authority 快照。"""
    states = require_wished_module("slope_sim.interfaces.v2.models").CommandAuthorityState
    authority = make_authority(session)
    authority.observe_peer_count(1)
    assert authority.accept(valid_command, model, commit=lambda: True).accepted
    with session._lock:
        session._command_generation = _UINT64_MAX
    before = authority.snapshot()
    assert before.state is states.ACTIVE
    intruder = replace(
        valid_command,
        command_generation=_UINT64_MAX,
        source_id="other",
        source_session_id=b"i" * 16,
        sequence=1,
    )
    actions = {
        "peer_edge": lambda: authority.observe_peer_count(2),
        "suspend": lambda: authority.suspend_protocol(0),
        "wrong_owner": lambda: authority.accept(intruder, model, commit=lambda: pytest.fail("must not commit")),
    }
    with pytest.raises(OverflowError, match="command_generation exhausted"):
        actions[action_name]()
    assert authority.snapshot() == before


@pytest.mark.parametrize(
    "change",
    (
        {"simulation_session_id": b"x" * 16},
        {"descriptor_sha256": b"x" * 32},
            {"world_generation": 2},
            {"command_generation": 2},
            {"robot_model": "df_front"},
            {"drive_wheel_speed_rad_s": (1.0,)},
        {"drive_wheel_speed_rad_s": (21.0, 0.0)},
    ),
)
def test_invalid_command_is_rejected_without_committing_or_claiming(change, session, model, valid_command) -> None:
    """身份、顺序、车型和机械规则拒绝不得刷新有效 mailbox 或 owner。"""
    authority = make_authority(session)
    authority.observe_peer_count(1)
    committed = False

    def commit() -> bool:
        nonlocal committed
        committed = True
        return True

    result = authority.accept(replace(valid_command, **change), model, commit=commit)
    snapshot = authority.snapshot()
    assert result.accepted is False
    assert committed is False
    assert snapshot.owner_source_id is None
    assert snapshot.last_sequence is None
    assert snapshot.command_generation == 1


def test_peer_count_zero_or_conflict_rejects_command_without_committing(session, model, valid_command) -> None:
    """没有唯一 peer 或存在冲突时，命令永远不能到达 mailbox。"""
    authority = make_authority(session)
    for peer_count in (0, 2):
        authority.observe_peer_count(peer_count)
        result = authority.accept(valid_command, model, commit=lambda: pytest.fail("must not commit"))
        assert result.accepted is False


def test_commit_failure_does_not_create_owner_or_advance_sequence(session, model, valid_command) -> None:
    """mailbox 失败或旧 ingress 失效不能留下幽灵 ACTIVE owner。"""
    states = require_wished_module("slope_sim.interfaces.v2.models").CommandAuthorityState
    authority = make_authority(session)
    authority.observe_peer_count(1)
    rejected = authority.accept(valid_command, model, commit=lambda: False)
    assert rejected.accepted is False
    assert authority.snapshot().state is states.CLAIMABLE
    assert authority.snapshot().owner_source_id is None
    accepted = authority.accept(valid_command, model, commit=lambda: True)
    assert accepted.accepted is True
    repeated = authority.accept(valid_command, model, commit=lambda: pytest.fail("must not commit"))
    assert repeated.accepted is False
    assert authority.snapshot().last_sequence == 0


def test_protocol_suspend_and_world_rebuild_never_restore_old_owner(session, model, valid_command) -> None:
    """suspend、abort 和 fault 都保留新的 command token 且清除 owner。"""
    states = require_wished_module("slope_sim.interfaces.v2.models").CommandAuthorityState
    authority = make_authority(session)
    authority.observe_peer_count(1)
    assert authority.accept(valid_command, model, commit=lambda: True).accepted
    assert authority.suspend_protocol(1).safe_stop is True
    assert authority.snapshot().state is states.CLAIMABLE
    assert session.command_generation == 2
    assert authority.prepare_world_rebuild().safe_stop is True
    assert session.command_generation == 3
    authority.abort_world_rebuild()
    assert authority.snapshot().owner_source_id is None
    assert (session.world_generation, session.command_generation) == (1, 3)
    authority.prepare_world_rebuild()
    authority.fault_world_rebuild()
    assert authority.snapshot().state is states.CLAIMABLE
    assert (session.world_generation, session.command_generation) == (1, 4)


def test_peer_count_rejects_bool_and_uint32_overflow(session) -> None:
    """peer count 严格对应 Protobuf uint32，不能接受 bool 或越界数。"""
    authority = make_authority(session)
    with pytest.raises(ValueError, match="command_peer_count"):
        authority.observe_peer_count(True)
    with pytest.raises(ValueError, match="command_peer_count"):
        authority.observe_peer_count(1 << 32)
