"""runSim v2：本机 Command socket 会话合同的安全边界。"""
from __future__ import annotations

from contextlib import contextmanager
import json
import os
import stat
import threading
from importlib import import_module
from pathlib import Path

import pytest


def require_wished_module(name: str):
    """缺少合同实现时保留可读 RED，不让收集阶段中断。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


@pytest.fixture
def contract():
    """按测试延迟加载尚未实现的 runSim 会话合同。"""
    return require_wished_module("slope_sim.interfaces.v2.runsim_session")


@pytest.fixture
def session(contract, tmp_path: Path):
    """为每个测试创建有确定 PID/UID/token 的独立 Command 启动记录。"""
    return contract.RunSimSession.create(
        tmp_path / "runsim-command",
        command_pid=43210,
        command_uid=os.getuid(),
        orchestrator_pid=32109,
        session_id_factory=lambda: b"s" * 16,
        token_factory=lambda: b"t" * 32,
    )


def _target(token: str, *, linear: float = 0.4, angular: float = -0.3) -> bytes:
    """构造规范中唯一允许的人工驾驶目标。"""
    return json.dumps(
        {
            "kind": "target",
            "token": token,
            "linear_velocity_m_s": linear,
            "angular_velocity_rad_s": angular,
        }
    ).encode("utf-8")


def test_launch_record_locks_a_0700_socket_directory_and_exposes_cpp_authentication(
    contract, session
) -> None:
    """启动记录绑定绝对 socket、唯一 Command PID/uid 和不可预测 token。"""
    directory_mode = stat.S_IMODE(session.socket_dir.stat().st_mode)
    assert directory_mode == 0o700
    assert session.socket_path == session.socket_dir / "command.sock"
    assert session.socket_path.is_absolute()
    assert session.launch_record_path.is_file()
    assert stat.S_IMODE(session.launch_record_path.stat().st_mode) == 0o600
    lifecycle_lock = session.socket_dir / "command.lifecycle.lock"
    assert lifecycle_lock.is_file()
    assert stat.S_IMODE(lifecycle_lock.stat().st_mode) == 0o600
    record = json.loads(session.launch_record_path.read_text(encoding="utf-8"))
    assert record == {
        "command_pid": 43210,
        "command_uid": os.getuid(),
        "orchestrator_pid": 32109,
        "protocol": "runsim-command-socket-v1",
        "session_id": (b"s" * 16).hex(),
        "socket_path": str(session.socket_path),
        "token": (b"t" * 32).hex(),
    }
    assert session.server_authentication == {
        "command_pid": 43210,
        "command_uid": os.getuid(),
        "protocol": "runsim-command-socket-v1",
        "session_id": (b"s" * 16).hex(),
        "token": (b"t" * 32).hex(),
    }
    assert session.snapshot().state is contract.RunSimSessionState.LAUNCHING


def test_duplicate_command_cannot_replace_existing_launch_lock(contract, session) -> None:
    """第二个 Command 不能复用或覆盖仍在运行的编排器启动记录。"""
    with pytest.raises(FileExistsError, match="Command launch lock already exists"):
        contract.RunSimSession.create(
            session.socket_dir,
            command_pid=98765,
            command_uid=os.getuid(),
            orchestrator_pid=87654,
        )


def test_orchestrator_attaches_to_the_child_owned_launch_record(contract, session) -> None:
    """父编排器只能校验并接管 child pre-exec 创建的同一认证会话。"""
    attached = contract.RunSimSession.attach(
        session.launch_record_path,
        command_pid=43210,
        command_uid=os.getuid(),
        orchestrator_pid=32109,
    )

    assert attached.socket_dir == session.socket_dir
    assert attached.socket_path == session.socket_path
    assert attached.server_authentication == session.server_authentication
    attached.accept_client_message(
        _target(attached.server_authentication["token"]),
        client_pid=54321,
        peer_uid=os.getuid(),
        now=10.0,
    )
    assert attached.snapshot().state is contract.RunSimSessionState.ACTIVE


def test_close_is_idempotent_releases_only_its_own_lock_and_invalidates_its_token(
    contract, session
) -> None:
    """关闭必须清零、回收本会话记录，并让同目录的新会话使用新 token。"""
    old_token = session.server_authentication["token"]
    session.accept_client_message(
        _target(old_token), client_pid=54321, peer_uid=os.getuid(), now=10.0
    )

    session.close()

    snapshot = session.snapshot()
    assert snapshot.state is contract.RunSimSessionState.CLOSED
    assert (snapshot.linear_velocity_m_s, snapshot.angular_velocity_rad_s) == (0.0, 0.0)
    assert not hasattr(snapshot, "token")
    assert not session.launch_record_path.exists()
    with pytest.raises(ValueError, match="token"):
        session.accept_client_message(
            _target(old_token), client_pid=54321, peer_uid=os.getuid(), now=10.1
        )
    assert session.close() is None

    successor = contract.RunSimSession.create(
        session.socket_dir,
        command_pid=43211,
        command_uid=os.getuid(),
        orchestrator_pid=32110,
        session_id_factory=lambda: b"u" * 16,
        token_factory=lambda: b"v" * 32,
    )
    assert successor.server_authentication["token"] != old_token

    replacement = {"token": "other-session"}
    successor.launch_record_path.write_text(json.dumps(replacement), encoding="utf-8")
    successor.close()
    assert json.loads(successor.launch_record_path.read_text(encoding="utf-8")) == replacement


def test_close_and_successor_create_share_a_lifecycle_lock(contract, session, monkeypatch) -> None:
    """关闭持有生命周期锁时，合规继任创建只能在旧锁清理后发布。"""
    cleanup_entered = threading.Event()
    allow_cleanup = threading.Event()
    creator_attempted = threading.Event()
    creator_acquired = threading.Event()
    close_errors: list[BaseException] = []
    create_errors: list[BaseException] = []
    successors: list[object] = []
    original_remove = contract._remove_launch_lock_if_owned
    original_lifecycle_lock = contract._lifecycle_lock

    def pause_cleanup(path, record):
        cleanup_entered.set()
        assert allow_cleanup.wait(timeout=1.0)
        return original_remove(path, record)

    @contextmanager
    def observe_lifecycle_lock(path):
        is_creator = threading.current_thread().name == "runsim-successor-creator"
        if is_creator:
            creator_attempted.set()
        with original_lifecycle_lock(path):
            if is_creator:
                creator_acquired.set()
            yield

    monkeypatch.setattr(contract, "_remove_launch_lock_if_owned", pause_cleanup)
    monkeypatch.setattr(contract, "_lifecycle_lock", observe_lifecycle_lock)

    def close_session() -> None:
        try:
            session.close()
        except BaseException as error:
            close_errors.append(error)

    def create_successor() -> None:
        try:
            successors.append(
                contract.RunSimSession.create(
                    session.socket_dir,
                    command_pid=43211,
                    command_uid=os.getuid(),
                    orchestrator_pid=32110,
                    session_id_factory=lambda: b"u" * 16,
                    token_factory=lambda: b"v" * 32,
                )
            )
        except BaseException as error:
            create_errors.append(error)

    closer = threading.Thread(target=close_session, name="runsim-closer")
    closer.start()
    assert cleanup_entered.wait(timeout=1.0)
    creator = threading.Thread(target=create_successor, name="runsim-successor-creator")
    creator.start()
    assert creator_attempted.wait(timeout=1.0)
    assert not creator_acquired.wait(timeout=0.1)

    allow_cleanup.set()
    closer.join(timeout=1.0)
    creator.join(timeout=1.0)

    assert not closer.is_alive()
    assert not creator.is_alive()
    assert close_errors == []
    assert create_errors == []
    assert len(successors) == 1
    assert successors[0].launch_record_path.exists()
    assert json.loads(successors[0].launch_record_path.read_text(encoding="utf-8"))["token"] == (
        b"v" * 32
    ).hex()
    successors[0].close()


def test_finalize_freezes_credentials_even_if_launch_lock_cleanup_fails(
    contract, session, monkeypatch
) -> None:
    """清理异常可见，但 finally 路径仍冻结凭据且后续 finalize 幂等。"""
    original_unlink = contract.Path.unlink

    def fail_launch_lock_unlink(path, *args, **kwargs):
        if path == session.launch_record_path:
            raise OSError("injected launch lock cleanup failure")
        return original_unlink(path, *args, **kwargs)

    monkeypatch.setattr(contract.Path, "unlink", fail_launch_lock_unlink)

    with pytest.raises(OSError, match="injected launch lock cleanup failure"):
        session.finalize()

    assert session.snapshot().state is contract.RunSimSessionState.CLOSED
    authentication = session.server_authentication
    assert authentication["session_id"] == ""
    assert authentication["token"] == ""
    assert session.launch_record_path.exists()
    assert session.finalize() is None


def test_safety_edge_cannot_be_overwritten_by_a_target_already_in_progress(
    session, monkeypatch
) -> None:
    """target 已开始时的安全边沿必须赢得竞态，快照只能看到完整安全态。"""
    target_entered = threading.Event()
    release_target = threading.Event()
    received_errors: list[BaseException] = []
    original_accept_target = session._accept_target

    def pause_before_target_commit(*args, **kwargs):
        target_entered.set()
        assert release_target.wait(timeout=1.0)
        return original_accept_target(*args, **kwargs)

    monkeypatch.setattr(session, "_accept_target", pause_before_target_commit)
    token = session.server_authentication["token"]

    def receive_target() -> None:
        try:
            session.accept_client_message(
                _target(token), client_pid=54321, peer_uid=os.getuid(), now=10.0
            )
        except BaseException as error:
            received_errors.append(error)

    worker = threading.Thread(target=receive_target)
    worker.start()
    assert target_entered.wait(timeout=1.0)
    session.keyboard_released()
    release_target.set()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert len(received_errors) == 1
    assert isinstance(received_errors[0], ValueError)
    snapshot = session.snapshot()
    assert snapshot.state.value == "safe_stop"
    assert (snapshot.linear_velocity_m_s, snapshot.angular_velocity_rad_s) == (0.0, 0.0)
    assert snapshot.last_target_monotonic is None


def test_close_releases_its_lock_after_command_already_reported_closed(session) -> None:
    """Command 先报 closed 不得使编排器的最终锁回收被跳过。"""
    token = session.server_authentication["token"]
    session.accept_client_message(
        json.dumps({"kind": "status", "token": token, "state": "closed"}).encode("utf-8"),
        client_pid=54321,
        peer_uid=os.getuid(),
        now=10.0,
    )

    session.close()

    assert not session.launch_record_path.exists()


def test_launch_lock_is_completely_written_before_atomic_publish(contract, tmp_path: Path, monkeypatch) -> None:
    """C++ 看到 launch record 时必须已是 fsync 后的完整 JSON，而非写入中的文件。"""
    original_link = contract.os.link
    observed: list[tuple[Path, dict[str, object]]] = []

    def publish(source, destination, *args, **kwargs):
        source_path = Path(source)
        destination_path = Path(destination)
        assert source_path.parent == destination_path.parent
        assert not destination_path.exists()
        observed.append((source_path, json.loads(source_path.read_text(encoding="utf-8"))))
        return original_link(source, destination, *args, **kwargs)

    monkeypatch.setattr(contract.os, "link", publish)
    created = contract.RunSimSession.create(
        tmp_path / "runsim-command",
        command_pid=43210,
        command_uid=os.getuid(),
        orchestrator_pid=32109,
        session_id_factory=lambda: b"s" * 16,
        token_factory=lambda: b"t" * 32,
    )

    assert len(observed) == 1
    source_path, record = observed[0]
    assert record == json.loads(created.launch_record_path.read_text(encoding="utf-8"))
    assert not source_path.exists()


def test_client_rejects_unknown_command_pid_or_different_uid(session) -> None:
    """Python/Dashboard 连接后必须把 SO_PEERCRED 与启动记录逐项比对。"""
    with pytest.raises(PermissionError, match="unknown Command PID"):
        session.verify_command_peer(pid=43211, uid=os.getuid())
    with pytest.raises(PermissionError, match="Command peer uid"):
        session.verify_command_peer(pid=43210, uid=os.getuid() + 1)
    assert session.verify_command_peer(pid=43210, uid=os.getuid()) is None


@pytest.mark.parametrize(
    "safety_event",
    ("connection_closed", "keyboard_released", "window_focus_lost"),
)
def test_disconnect_release_and_focus_loss_immediately_safe_stop(session, safety_event: str) -> None:
    """所有本机人工输入边沿都将目标清零，不能等待下一条控制消息。"""
    token = session.server_authentication["token"]
    session.accept_client_message(
        _target(token), client_pid=54321, peer_uid=os.getuid(), now=10.0
    )
    getattr(session, safety_event)()
    snapshot = session.snapshot()
    assert snapshot.state.value == "safe_stop"
    assert (snapshot.linear_velocity_m_s, snapshot.angular_velocity_rad_s) == (0.0, 0.0)
    assert snapshot.safe_stop_reason == safety_event


def test_expired_manual_lease_immediately_safe_stops(session) -> None:
    """100 ms 单调租约到期后，旧 target 绝不能继续生效。"""
    token = session.server_authentication["token"]
    session.accept_client_message(
        _target(token), client_pid=54321, peer_uid=os.getuid(), now=10.0
    )
    assert session.expire_manual_target(now=10.099) is False
    assert session.expire_manual_target(now=10.100) is True
    snapshot = session.snapshot()
    assert snapshot.state.value == "safe_stop"
    assert snapshot.safe_stop_reason == "manual_target_timeout"
    assert (snapshot.linear_velocity_m_s, snapshot.angular_velocity_rad_s) == (0.0, 0.0)


@pytest.mark.parametrize("state", ("safe_stop", "stopping", "closed"))
def test_terminal_status_clears_an_active_target_and_lease(session, state: str) -> None:
    """Command 的安全/终止 status 不得留下会继续生效的人工目标。"""
    token = session.server_authentication["token"]
    session.accept_client_message(
        _target(token), client_pid=54321, peer_uid=os.getuid(), now=10.0
    )

    session.accept_client_message(
        json.dumps({"kind": "status", "token": token, "state": state}).encode("utf-8"),
        client_pid=54321,
        peer_uid=os.getuid(),
        now=10.01,
    )

    snapshot = session.snapshot()
    assert snapshot.state.value == state
    assert (snapshot.linear_velocity_m_s, snapshot.angular_velocity_rad_s) == (0.0, 0.0)
    assert snapshot.last_target_monotonic is None


@pytest.mark.parametrize(
    ("terminal_state", "safety_event"),
    tuple(
        (terminal_state, safety_event)
        for terminal_state in ("stopping", "closed")
        for safety_event in (
            "connection_closed",
            "keyboard_released",
            "window_focus_lost",
            "expire_manual_target",
        )
    ),
)
def test_safety_edges_cannot_downgrade_a_terminal_state(session, terminal_state: str, safety_event: str) -> None:
    """安全停车只撤销控制权，不能把 stopping/closed 倒退成 safe_stop。"""
    token = session.server_authentication["token"]
    session.accept_client_message(
        _target(token), client_pid=54321, peer_uid=os.getuid(), now=10.0
    )
    session.accept_client_message(
        json.dumps({"kind": "stop", "token": token, "reason": "user_request"}).encode("utf-8"),
        client_pid=54321,
        peer_uid=os.getuid(),
        now=10.01,
    )
    if terminal_state == "closed":
        session.accept_client_message(
            json.dumps({"kind": "status", "token": token, "state": "closed"}).encode("utf-8"),
            client_pid=54321,
            peer_uid=os.getuid(),
            now=10.02,
        )

    result = (
        session.expire_manual_target(now=10.2)
        if safety_event == "expire_manual_target"
        else getattr(session, safety_event)()
    )

    assert result in {None, False}
    snapshot = session.snapshot()
    assert snapshot.state.value == terminal_state
    assert (snapshot.linear_velocity_m_s, snapshot.angular_velocity_rad_s) == (0.0, 0.0)
    assert snapshot.last_target_monotonic is None


def test_stop_clears_target_lease_and_timeout_cannot_revive_terminal_session(session) -> None:
    """stop 后租约失效、timeout 不重复转换，后续 target 也不得重新 active。"""
    token = session.server_authentication["token"]
    session.accept_client_message(
        _target(token), client_pid=54321, peer_uid=os.getuid(), now=10.0
    )
    session.accept_client_message(
        json.dumps({"kind": "stop", "token": token, "reason": "user_request"}).encode("utf-8"),
        client_pid=54321,
        peer_uid=os.getuid(),
        now=10.01,
    )

    assert session.expire_manual_target(now=10.2) is False
    with pytest.raises(ValueError, match="stopping"):
        session.accept_client_message(
            _target(token), client_pid=54321, peer_uid=os.getuid(), now=10.2
        )
    assert session.snapshot().state.value == "stopping"


def test_expire_manual_target_transitions_only_once(session) -> None:
    """同一 target 的 timeout 是一次性安全边沿，重复轮询不得反复转换。"""
    token = session.server_authentication["token"]
    session.accept_client_message(
        _target(token), client_pid=54321, peer_uid=os.getuid(), now=10.0
    )

    assert session.expire_manual_target(now=10.1) is True
    assert session.expire_manual_target(now=10.2) is False
    assert session.snapshot().state.value == "safe_stop"


@pytest.mark.parametrize(
    "payload",
    (
        {"kind": "target", "token": "t" * 64, "linear_velocity_m_s": float("nan"), "angular_velocity_rad_s": 0.0},
        {"kind": "target", "token": "t" * 64, "linear_velocity_m_s": 1.21, "angular_velocity_rad_s": 0.0},
        {"kind": "target", "token": "wrong", "linear_velocity_m_s": 0.0, "angular_velocity_rad_s": 0.0},
    ),
)
def test_illegal_target_is_rejected_without_refreshing_the_lease(session, payload: dict[str, object]) -> None:
    """目标必须是有界有限数且匹配本会话 token，拒绝不能复活旧驾驶命令。"""
    with pytest.raises(ValueError):
        session.accept_client_message(
            json.dumps(payload).encode("utf-8"),
            client_pid=54321,
            peer_uid=os.getuid(),
            now=10.0,
        )
    snapshot = session.snapshot()
    assert (snapshot.linear_velocity_m_s, snapshot.angular_velocity_rad_s) == (0.0, 0.0)
    assert snapshot.last_target_monotonic is None


def test_pointcloud_fields_and_oversize_payloads_are_hard_rejected(session) -> None:
    """控制 socket 不能成为点云或任意大负载的数据面。"""
    token = session.server_authentication["token"]
    pointcloud = {
        "kind": "target",
        "token": token,
        "linear_velocity_m_s": 0.0,
        "angular_velocity_rad_s": 0.0,
        "pointcloud": [1, 2, 3],
    }
    with pytest.raises(ValueError, match="pointcloud"):
        session.accept_client_message(
            json.dumps(pointcloud).encode("utf-8"),
            client_pid=54321,
            peer_uid=os.getuid(),
            now=10.0,
        )
    with pytest.raises(ValueError, match="maximum"):
        session.accept_client_message(
            b"{" + b" " * 1024 + b"}",
            client_pid=54321,
            peer_uid=os.getuid(),
            now=10.0,
        )


def test_duplicate_json_fields_are_rejected_fail_closed(session) -> None:
    """重复字段不得由 JSON 的“最后一个值胜出”规则悄然绕过认证。"""
    token = session.server_authentication["token"]
    payload = (
        b'{"kind":"target","token":"wrong","token":"'
        + token.encode("ascii")
        + b'","linear_velocity_m_s":0.4,"angular_velocity_rad_s":-0.3}'
    )

    with pytest.raises(ValueError, match="invalid JSON"):
        session.accept_client_message(
            payload, client_pid=54321, peer_uid=os.getuid(), now=10.0
        )
    snapshot = session.snapshot()
    assert snapshot.state.value == "launching"
    assert snapshot.last_target_monotonic is None


def test_only_target_status_and_stop_messages_are_accepted(session) -> None:
    """合同仅定义三种小型 JSON 消息，停止请求立即清零并进入 stopping。"""
    token = session.server_authentication["token"]
    status = session.accept_client_message(
        json.dumps({"kind": "status", "token": token, "state": "ready"}).encode("utf-8"),
        client_pid=54321,
        peer_uid=os.getuid(),
        now=10.0,
    )
    assert status.kind == "status"
    stop = session.accept_client_message(
        json.dumps({"kind": "stop", "token": token, "reason": "user_request"}).encode("utf-8"),
        client_pid=54321,
        peer_uid=os.getuid(),
        now=10.1,
    )
    assert stop.kind == "stop"
    assert session.snapshot().state.value == "stopping"
    with pytest.raises(ValueError, match="kind"):
        session.accept_client_message(
            json.dumps({"kind": "telemetry", "token": token}).encode("utf-8"),
            client_pid=54321,
            peer_uid=os.getuid(),
            now=10.2,
        )


def test_status_projects_legal_lifecycle_without_reviving_terminal_session(session) -> None:
    """status 必须更新快照，但 stopping/closed 不能被后续状态复活。"""
    token = session.server_authentication["token"]

    ready = session.accept_client_message(
        json.dumps({"kind": "status", "token": token, "state": "ready"}).encode("utf-8"),
        client_pid=54321,
        peer_uid=os.getuid(),
        now=10.0,
    )
    assert ready.state == "ready"
    assert session.snapshot().state.value == "ready"

    session.accept_client_message(
        json.dumps({"kind": "stop", "token": token, "reason": "user_request"}).encode("utf-8"),
        client_pid=54321,
        peer_uid=os.getuid(),
        now=10.1,
    )
    with pytest.raises(ValueError, match="status transition"):
        session.accept_client_message(
            json.dumps({"kind": "status", "token": token, "state": "ready"}).encode("utf-8"),
            client_pid=54321,
            peer_uid=os.getuid(),
            now=10.2,
        )
    assert session.snapshot().state.value == "stopping"

    closed = session.accept_client_message(
        json.dumps({"kind": "status", "token": token, "state": "closed"}).encode("utf-8"),
        client_pid=54321,
        peer_uid=os.getuid(),
        now=10.3,
    )
    assert closed.state == "closed"
    assert session.snapshot().state.value == "closed"
    with pytest.raises(ValueError, match="status transition"):
        session.accept_client_message(
            json.dumps({"kind": "status", "token": token, "state": "ready"}).encode("utf-8"),
            client_pid=54321,
            peer_uid=os.getuid(),
            now=10.4,
        )
    assert session.snapshot().state.value == "closed"
