"""阶段四 A：验证 v2 会话身份、world 事务和逐话题输出序号。"""
from importlib import import_module

import pytest


_UINT64_MAX = (1 << 64) - 1
_OUTPUT_TOPICS = (
    "/sim/wheel/state",
    "/sim/lidar/points",
    "/sim/rtk/state",
    "/sim/imu/attitude",
)


def require_wished_module(name: str):
    """缺少会话实现时保留可读 RED，不让收集阶段中断。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


@pytest.fixture
def descriptor():
    """读取冻结的 v2 descriptor 身份。"""
    return require_wished_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()


def make_session(descriptor):
    """为单实例行为提供稳定会话 ID。"""
    protocol_session = require_wished_module("slope_sim.interfaces.v2.session").ProtocolSession
    return protocol_session(descriptor, session_id_factory=lambda: b"s" * 16)


def test_process_restart_never_reuses_simulation_session(descriptor) -> None:
    """默认 factory 生成的新进程会话不得复用 16-byte 身份。"""
    protocol_session = require_wished_module("slope_sim.interfaces.v2.session").ProtocolSession
    first = protocol_session(descriptor)
    second = protocol_session(descriptor)
    assert len(first.simulation_session_id) == 16
    assert first.simulation_session_id != second.simulation_session_id
    assert first.world_generation == second.world_generation == 1
    assert first.command_generation == second.command_generation == 1
    assert first.descriptor_sha256 == second.descriptor_sha256 == descriptor.sha256


def test_output_sequence_is_reserved_before_work_and_failure_leaves_gap(descriptor) -> None:
    """读取失败后的已保留序号不得回收或重复发送。"""
    session = make_session(descriptor)
    first = session.reserve_output("/sim/lidar/points")
    failed = session.reserve_output("/sim/lidar/points")
    third = session.reserve_output("/sim/lidar/points")
    assert (first.sequence, failed.sequence, third.sequence) == (0, 1, 2)
    assert all(identity.world_generation == 1 for identity in (first, failed, third))


def test_successful_world_rebuild_commits_new_generation_and_resets_output_sequences(descriptor) -> None:
    """只有成功提交可换 world；每个新 world 从各话题序号零开始。"""
    session = make_session(descriptor)
    assert session.reserve_output("/sim/wheel/state").sequence == 0
    assert session.prepare_world_rebuild() == 2
    assert session.command_generation == 2
    assert session.commit_world_rebuild() == 2
    assert session.world_generation == 2
    assert session.command_generation == 2
    identity = session.reserve_output("/sim/wheel/state")
    assert (identity.world_generation, identity.sequence) == (2, 0)


def test_abort_and_fault_leave_world_and_new_command_generation_intact(descriptor) -> None:
    """失败事务撤销准备态，但绝不回收已推进的 command 代际。"""
    session = make_session(descriptor)
    assert session.prepare_world_rebuild() == 2
    assert session.abort_world_rebuild() is None
    assert (session.world_generation, session.command_generation) == (1, 2)
    assert session.prepare_world_rebuild() == 3
    assert session.fault_world_rebuild() is None
    assert (session.world_generation, session.command_generation) == (1, 3)


def test_world_rebuild_requires_exact_transaction_order(descriptor) -> None:
    """prepare 不可重入，未 prepare 的 commit 和 abort 必须拒绝。"""
    session = make_session(descriptor)
    with pytest.raises(RuntimeError, match="not prepared"):
        session.commit_world_rebuild()
    with pytest.raises(RuntimeError, match="not prepared"):
        session.abort_world_rebuild()
    session.prepare_world_rebuild()
    with pytest.raises(RuntimeError, match="already prepared"):
        session.prepare_world_rebuild()


def test_output_topics_have_independent_sequences_and_unknown_topic_is_rejected(descriptor) -> None:
    """四条输出话题分别计数，命令和未知 topic 不能取得输出身份。"""
    session = make_session(descriptor)
    assert tuple(session.reserve_output(topic).sequence for topic in _OUTPUT_TOPICS) == (0, 0, 0, 0)
    assert session.reserve_output("/sim/wheel/state").sequence == 1
    with pytest.raises(ValueError, match="unknown v2 output topic"):
        session.reserve_output("/sim/wheel/command")


def test_session_id_factory_requires_exact_16_bytes(descriptor) -> None:
    """注入 factory 只能用于测试，仍必须满足线上固定长度身份。"""
    protocol_session = require_wished_module("slope_sim.interfaces.v2.session").ProtocolSession
    with pytest.raises(ValueError, match="simulation_session_id must be exactly 16 bytes"):
        protocol_session(descriptor, session_id_factory=lambda: b"short")


def test_generation_and_sequence_overflow_fail_without_advancing_state(descriptor) -> None:
    """uint64 耗尽要 fail closed，且不得部分推进会话状态。"""
    session = make_session(descriptor)
    with session._lock:
        session._next_sequence["/sim/wheel/state"] = _UINT64_MAX + 1
    with pytest.raises(OverflowError, match="output sequence exhausted"):
        session.reserve_output("/sim/wheel/state")
    with session._lock:
        assert session._next_sequence["/sim/wheel/state"] == _UINT64_MAX + 1
        session._command_generation = _UINT64_MAX
    with pytest.raises(OverflowError, match="command_generation exhausted"):
        session.prepare_world_rebuild()
    assert session.command_generation == _UINT64_MAX
    with session._lock:
        session._command_generation = 2
        session._world_generation = _UINT64_MAX
    session.prepare_world_rebuild()
    with pytest.raises(OverflowError, match="world_generation exhausted"):
        session.commit_world_rebuild()
    assert (session.world_generation, session.command_generation) == (_UINT64_MAX, 3)
