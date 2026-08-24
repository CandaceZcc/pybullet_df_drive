"""runSim v2：C++ Command 的受认证 Unix socket 交互控制边界。"""
from __future__ import annotations

from hashlib import sha256
import json
import os
from pathlib import Path
import socket
import stat
import subprocess
import time

import pytest

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb
from slope_sim.interfaces.v2.runsim_session import RunSimSession


def _command_binary() -> Path:
    """只使用调用方明确给出的现有 C++ 构建根，不创建测试构建。"""
    raw_build = os.environ.get("STAGE4_PHASE0_BUILD_DIR")
    assert raw_build, "STAGE4_PHASE0_BUILD_DIR must name the existing C++ build"
    command = Path(raw_build) / "slope_sim_stage4_command"
    assert command.is_file() and os.access(command, os.X_OK)
    return command


def _require_real_ecal_interactive() -> None:
    """真实 eCAL 发现是外部门禁；默认单测环境不伪造其可用性。"""
    if os.environ.get("STAGE4_ENABLE_REAL_ECAL_INTERACTIVE_TESTS") != "1":
        pytest.skip("requires explicit real eCAL interactive capability")


def _write_identity_payload(path: Path, descriptor: bytes) -> None:
    """socket 不能提供协议身份；交互 Command 只复用此 runtime 协商模板。"""
    path.write_bytes(
        pb.WheelCommand(
            timestamp_ns=1,
            drive_wheel_speed_rad_s=(0.0, 0.0),
            sequence=0,
            world_generation=1,
            command_generation=1,
            source_id="runsim-command",
            source_session_id=b"c" * 16,
            robot_model="df_mid",
            simulation_session_id=b"s" * 16,
            descriptor_sha256=sha256(descriptor).digest(),
        ).SerializeToString(deterministic=True)
    )


def _launch_record_writer(
    socket_dir: Path,
    launch_record: Path,
    token: str,
    *,
    orchestrator_pid: int | None = None,
):
    """返回 exec 前写记录的回调，使 C++ 自验真实子进程 PID。"""

    def write_launch_record() -> None:
        session = RunSimSession.create(
            socket_dir,
            command_pid=os.getpid(),
            command_uid=os.getuid(),
            orchestrator_pid=os.getppid() if orchestrator_pid is None else orchestrator_pid,
            session_id_factory=lambda: bytes.fromhex("73" * 16),
            token_factory=lambda: bytes.fromhex(token),
        )
        assert session.launch_record_path == launch_record

    return write_launch_record


def _interactive_arguments(
    command: Path,
    descriptor_path: Path,
    payload: Path,
    result: Path,
    launch_record: Path,
    *,
    duration_ms: int = 800,
    deadline_ms: int = 1000,
) -> list[str]:
    return [
        str(command),
        "--interactive",
        "--descriptor-set", str(descriptor_path),
        "--payload", str(payload),
        "--duration-ms", str(duration_ms),
        "--deadline-ms", str(deadline_ms),
        "--result", str(result),
        "--launch-record", str(launch_record),
    ]


def _ndjson(document: dict[str, object], *, frame_bytes: int | None = None) -> bytes:
    """NDJSON 一帧的 1024-byte 上限包含末尾 LF；空白仍属于严格 JSON。"""
    encoded = json.dumps(document, separators=(",", ":")).encode("utf-8")
    if frame_bytes is None:
        return encoded + b"\n"
    assert len(encoded) + 1 <= frame_bytes
    return encoded + b" " * (frame_bytes - len(encoded) - 1) + b"\n"


def _close_subscriber(subscriber: object) -> None:
    """兼容 eCAL Python binding 的显式 callback 回收接口。"""
    close = getattr(subscriber, "close", None)
    if callable(close):
        close()
        return
    remove = getattr(subscriber, "remove_receive_callback", None)
    if callable(remove):
        remove()


@pytest.mark.stage4_artifact
def test_interactive_command_ndjson_frames_split_coalesced_and_fail_closed(tmp_path: Path) -> None:
    """C++ 子进程按 LF 分帧：split/coalesced 依序处理，1025 bytes 与坏 token 归零。"""
    _require_real_ecal_interactive()
    from scripts.verify_stage4_v2_phase0 import initialize_phase0_core
    from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
    from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings

    command = _command_binary()
    root = Path(__file__).resolve().parents[2]
    descriptor = (root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc").read_bytes()
    descriptor_path = tmp_path / "v2.desc"
    descriptor_path.write_bytes(descriptor)
    payload = tmp_path / "identity.bin"
    _write_identity_payload(payload, descriptor)
    socket_dir = tmp_path / "socket"
    socket_path = socket_dir / "command.sock"
    launch_record = socket_dir / "command.launch.lock"
    result = tmp_path / "result.json"
    token = "d" * 64
    target = {"kind": "target", "token": token, "linear_velocity_m_s": 0.4, "angular_velocity_rad_s": 0.2}
    status = {"kind": "status", "token": token, "state": "active"}
    exact_limit = _ndjson(status, frame_bytes=1024)
    oversized = _ndjson(status, frame_bytes=1025)
    assert len(exact_limit) == 1024
    assert len(oversized) == 1025
    frames: list[pb.WheelCommand] = []
    bindings = EcalRawBindings()
    core = bindings._core
    subscriber = None
    process = None
    initialize_phase0_core(core, f"runsim-ndjson-command-peer-{os.getpid()}")

    try:
        def receive(frame) -> None:
            message = pb.WheelCommand()
            assert message.ParseFromString(frame.payload)
            frames.append(message)

        subscriber = bindings.create_subscriber(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            load_v2_descriptor(),
            receive,
        )
        process = subprocess.Popen(
            _interactive_arguments(command, descriptor_path, payload, result, launch_record),
            preexec_fn=_launch_record_writer(socket_dir, launch_record, token),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 2.0
        while (not socket_path.exists() or subscriber.get_publisher_count() != 1) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert socket_path.exists(), process.stderr.read()
        assert subscriber.get_publisher_count() == 1
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(1.0)
            client.connect(str(socket_path))
            split_target = _ndjson(target)
            client.sendall(split_target[:11])
            time.sleep(0.02)
            # 一次 send 内 coalesce 余下 target 和恰好 1024-byte status；二者必须按顺序生效。
            client.sendall(split_target[11:] + exact_limit)
            time.sleep(0.04)
            client.sendall(_ndjson({**target, "token": "0" * 64}))
            time.sleep(0.04)
            client.sendall(oversized)
            assert client.recv(1) == b""
        stdout, stderr = process.communicate(timeout=3)
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.communicate()
        if subscriber is not None:
            _close_subscriber(subscriber)
        core.finalize()

    assert process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
    report = json.loads(result.read_text(encoding="utf-8"))
    assert report["active_published_count"] > 0
    assert report["safe_stop_published_count"] > 0
    assert any(any(abs(value) > 0.0 for value in frame.drive_wheel_speed_rad_s) for frame in frames)
    assert any(all(value == 0.0 for value in frame.drive_wheel_speed_rad_s) for frame in frames)


@pytest.mark.stage4_artifact
def test_interactive_command_rejects_untrusted_launch_records_before_ecal(tmp_path: Path) -> None:
    """launch record 必须是本 euid 的 0600 非链接完整文件，错误 uid 也不得启动。"""
    command = _command_binary()
    root = Path(__file__).resolve().parents[2]
    descriptor = (root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc").read_bytes()
    descriptor_path = tmp_path / "v2.desc"
    descriptor_path.write_bytes(descriptor)
    payload = tmp_path / "identity.bin"
    _write_identity_payload(payload, descriptor)
    token = "e" * 64

    def run_with_record(socket_dir: Path, record: Path) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            _interactive_arguments(command, descriptor_path, payload, tmp_path / f"{socket_dir.name}.json", record),
            capture_output=True,
            text=True,
            check=False,
        )

    insecure_dir = tmp_path / "insecure"
    insecure = RunSimSession.create(
        insecure_dir, command_pid=os.getpid(), command_uid=os.getuid(), token_factory=lambda: bytes.fromhex(token)
    )
    os.chmod(insecure.launch_record_path, 0o644)
    insecure_result = run_with_record(insecure_dir, insecure.launch_record_path)
    assert insecure_result.returncode == 66
    assert "launch-record" in insecure_result.stderr

    symlink_dir = tmp_path / "symlink"
    symlink = RunSimSession.create(
        symlink_dir, command_pid=os.getpid(), command_uid=os.getuid(), token_factory=lambda: bytes.fromhex(token)
    )
    target = symlink_dir / "record-target.json"
    symlink.launch_record_path.rename(target)
    symlink.launch_record_path.symlink_to(target)
    symlink_result = run_with_record(symlink_dir, symlink.launch_record_path)
    assert symlink_result.returncode == 66
    assert "launch-record" in symlink_result.stderr

    wrong_uid_dir = tmp_path / "wrong-uid"
    wrong_uid = RunSimSession.create(
        wrong_uid_dir, command_pid=os.getpid(), command_uid=os.getuid() + 1, token_factory=lambda: bytes.fromhex(token)
    )
    wrong_uid_result = run_with_record(wrong_uid_dir, wrong_uid.launch_record_path)
    assert wrong_uid_result.returncode == 66
    assert "command uid" in wrong_uid_result.stderr


@pytest.mark.stage4_artifact
def test_interactive_command_rejects_a_launch_record_from_another_orchestrator(tmp_path: Path) -> None:
    """Command 不得脱离写入 launch record 的启动器独自继续运行。"""
    command = _command_binary()
    root = Path(__file__).resolve().parents[2]
    descriptor = (root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc").read_bytes()
    descriptor_path = tmp_path / "v2.desc"
    descriptor_path.write_bytes(descriptor)
    payload = tmp_path / "identity.bin"
    _write_identity_payload(payload, descriptor)
    socket_dir = tmp_path / "socket"
    launch_record = socket_dir / "command.launch.lock"

    process = subprocess.Popen(
        _interactive_arguments(command, descriptor_path, payload, tmp_path / "result.json", launch_record),
        preexec_fn=_launch_record_writer(socket_dir, launch_record, "f" * 64, orchestrator_pid=1),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        stdout, stderr = process.communicate(timeout=1.0)
    except subprocess.TimeoutExpired:
        process.kill()
        stdout, stderr = process.communicate()
        pytest.fail("Command did not reject the mismatched orchestrator before eCAL startup")

    assert process.returncode == 66, f"stdout={stdout}\nstderr={stderr}"
    assert "orchestrator" in stderr


@pytest.mark.stage4_artifact
def test_interactive_command_accepts_authenticated_target_and_stops_when_peer_disconnects(
    tmp_path: Path,
) -> None:
    """真实子进程仅允许同 uid 的 token target，断开后由 Command 立刻安全停车。"""
    _require_real_ecal_interactive()
    from scripts.verify_stage4_v2_phase0 import initialize_phase0_core
    from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
    from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings

    command = _command_binary()
    root = Path(__file__).resolve().parents[2]
    descriptor = (root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc").read_bytes()
    descriptor_path = tmp_path / "v2.desc"
    descriptor_path.write_bytes(descriptor)
    payload = tmp_path / "identity.bin"
    _write_identity_payload(payload, descriptor)
    socket_dir = tmp_path / "socket"
    socket_path = socket_dir / "command.sock"
    launch_record = socket_dir / "command.launch.lock"
    result = tmp_path / "result.json"
    token = "a" * 64
    bindings = EcalRawBindings()
    core = bindings._core
    subscriber = None
    initialize_phase0_core(core, f"runsim-command-target-peer-{os.getpid()}")

    process = None
    try:
        subscriber = bindings.create_subscriber(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            load_v2_descriptor(),
            lambda _frame: None,
        )
        process = subprocess.Popen(
            _interactive_arguments(command, descriptor_path, payload, result, launch_record, duration_ms=800),
            preexec_fn=_launch_record_writer(socket_dir, launch_record, token),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 2.0
        while (not socket_path.exists() or subscriber.get_publisher_count() != 1) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert socket_path.exists(), process.stderr.read()
        assert subscriber.get_publisher_count() == 1
        assert stat.S_IMODE(socket_dir.stat().st_mode) == 0o700
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(socket_path))
            client.sendall(
                json.dumps(
                    {
                        "kind": "target",
                        "token": token,
                        "linear_velocity_m_s": 0.4,
                        "angular_velocity_rad_s": 0.2,
                    },
                    separators=(",", ":"),
                ).encode("utf-8") + b"\n"
            )
            time.sleep(0.03)
        stdout, stderr = process.communicate(timeout=3)
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.communicate()
        if subscriber is not None:
            _close_subscriber(subscriber)
        core.finalize()

    assert process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
    report = json.loads(result.read_text(encoding="utf-8"))
    assert report["active_published_count"] > 0
    assert report["safe_stop_published_count"] > 0
    assert not socket_path.exists()


@pytest.mark.stage4_artifact
def test_interactive_command_keeps_continuous_target_active_under_240hz_socket_renewal(
    tmp_path: Path,
) -> None:
    """持续 240 Hz socket 续租时，C++ 100 Hz 发布不得在非零命令之间插入安全零速。"""
    _require_real_ecal_interactive()
    from scripts.verify_stage4_v2_phase0 import initialize_phase0_core
    from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
    from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings

    command = _command_binary()
    root = Path(__file__).resolve().parents[2]
    descriptor = (root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc").read_bytes()
    descriptor_path = tmp_path / "v2.desc"
    descriptor_path.write_bytes(descriptor)
    payload = tmp_path / "identity.bin"
    _write_identity_payload(payload, descriptor)
    socket_dir = tmp_path / "socket"
    socket_path = socket_dir / "command.sock"
    launch_record = socket_dir / "command.launch.lock"
    result = tmp_path / "result.json"
    token = "c" * 64
    frames: list[pb.WheelCommand] = []
    bindings = EcalRawBindings()
    core = bindings._core
    subscriber = None
    process = None
    initialize_phase0_core(core, f"runsim-command-240hz-peer-{os.getpid()}")

    try:
        subscriber = bindings.create_subscriber(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            load_v2_descriptor(),
            lambda frame: frames.append(pb.WheelCommand.FromString(frame.payload)),
        )
        process = subprocess.Popen(
            _interactive_arguments(
                command, descriptor_path, payload, result, launch_record,
                duration_ms=1500, deadline_ms=2000,
            ),
            preexec_fn=_launch_record_writer(socket_dir, launch_record, token),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 2.0
        while (not socket_path.exists() or subscriber.get_publisher_count() != 1) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert socket_path.exists(), process.stderr.read()
        assert subscriber.get_publisher_count() == 1
        target = _ndjson(
            {"kind": "target", "token": token, "linear_velocity_m_s": 1.5, "angular_velocity_rad_s": 0.0}
        )
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(socket_path))
            next_send = time.monotonic()
            end = next_send + 1.0
            while next_send < end:
                client.sendall(target)
                next_send += 1.0 / 240.0
                time.sleep(max(0.0, next_send - time.monotonic()))
        stdout, stderr = process.communicate(timeout=3)
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.communicate()
        if subscriber is not None:
            _close_subscriber(subscriber)
        core.finalize()

    assert process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
    nonzero = [any(abs(value) > 0.0 for value in frame.drive_wheel_speed_rad_s) for frame in frames]
    assert any(nonzero), "C++ Command did not publish the sustained target"
    first_active = nonzero.index(True)
    last_active = len(nonzero) - 1 - nonzero[::-1].index(True)
    assert all(nonzero[first_active:last_active + 1]), "C++ inserted a safe stop during continuous socket renewal"


@pytest.mark.stage4_artifact
def test_interactive_command_240hz_renewal_keeps_python_ecal_mailbox_active(tmp_path: Path) -> None:
    """持续 socket 续租经真实 eCAL 到达 Python mailbox 时，100 ms watchdog 不得超时。"""
    _require_real_ecal_interactive()
    from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
    from slope_sim.interfaces.v2.runtime_protocol import V2RuntimeProtocol
    from slope_sim.interfaces.v2.transport import create_v2_ecal_transport
    from slope_sim.model_registry import get_robot_model

    command = _command_binary()
    root = Path(__file__).resolve().parents[2]
    descriptor_bytes = (root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc").read_bytes()
    descriptor_path = tmp_path / "v2.desc"
    descriptor_path.write_bytes(descriptor_bytes)
    payload = tmp_path / "identity.bin"
    _write_identity_payload(payload, descriptor_bytes)
    socket_dir = tmp_path / "socket"
    socket_path = socket_dir / "command.sock"
    launch_record = socket_dir / "command.launch.lock"
    result = tmp_path / "result.json"
    token = "d" * 64
    descriptor = load_v2_descriptor()
    transport = create_v2_ecal_transport(
        descriptor=descriptor,
        participant_name=f"runsim-command-mailbox-peer-{os.getpid()}",
    )
    protocol = V2RuntimeProtocol(
        get_robot_model("df_mid"),
        transport=transport,
        descriptor=descriptor,
        session_id_factory=lambda: b"s" * 16,
    )
    subscription = None
    process = None
    try:
        subscription = transport.subscribe(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            lambda payload, received_at: protocol.accept_payload(payload, received_at=received_at),
        )
        process = subprocess.Popen(
            _interactive_arguments(
                command, descriptor_path, payload, result, launch_record,
                duration_ms=1500, deadline_ms=2000,
            ),
            preexec_fn=_launch_record_writer(socket_dir, launch_record, token),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 2.0
        while not socket_path.exists() and time.monotonic() < deadline:
            protocol.refresh_transport()
            time.sleep(0.01)
        assert socket_path.exists(), process.stderr.read()
        while (
            protocol.refresh_transport().topic_quality[0].protocol_state != "verified"
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        assert protocol.snapshot().command_protocol_state == "verified"
        while transport.snapshot().received_count == 0 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert transport.snapshot().received_count > 0, "Python transport did not receive C++ safe-stop frames"
        target = _ndjson(
            {"kind": "target", "token": token, "linear_velocity_m_s": 1.5, "angular_velocity_rad_s": 0.0}
        )
        timed_out_after_active = False
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(socket_path))
            next_send = time.monotonic()
            end = next_send + 1.0
            active_seen = False
            while next_send < end:
                protocol.refresh_transport()
                client.sendall(target)
                decision = protocol.mailbox.decision(now=time.monotonic())
                active_seen = active_seen or not decision.waiting
                timed_out_after_active = timed_out_after_active or (active_seen and decision.timed_out)
                next_send += 1.0 / 240.0
                time.sleep(max(0.0, next_send - time.monotonic()))
        stdout, stderr = process.communicate(timeout=3)
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.communicate()
        if subscription is not None:
            subscription.close()
        transport.close()

    assert process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
    assert active_seen, "Python mailbox never accepted the C++ command frames"
    assert not timed_out_after_active, "Python mailbox timed out during continuous C++ command frames"


@pytest.mark.stage4_artifact
def test_interactive_command_accepts_subscriber_after_socket_server_starts(tmp_path: Path) -> None:
    """Python world 在 Command socket 就绪后才订阅时，Command 仍保持安全零速并可连接。"""
    _require_real_ecal_interactive()
    from scripts.verify_stage4_v2_phase0 import initialize_phase0_core
    from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
    from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings

    command = _command_binary()
    root = Path(__file__).resolve().parents[2]
    descriptor = (root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc").read_bytes()
    descriptor_path = tmp_path / "v2.desc"
    descriptor_path.write_bytes(descriptor)
    payload = tmp_path / "identity.bin"
    _write_identity_payload(payload, descriptor)
    socket_dir = tmp_path / "socket"
    socket_path = socket_dir / "command.sock"
    launch_record = socket_dir / "command.launch.lock"
    result = tmp_path / "result.json"
    token = "f" * 64
    bindings = EcalRawBindings()
    core = bindings._core
    subscriber = None
    received_frames: list[object] = []
    process = None
    core_initialized = False

    try:
        process = subprocess.Popen(
            _interactive_arguments(
                command,
                descriptor_path,
                payload,
                result,
                launch_record,
                duration_ms=300,
                deadline_ms=8000,
            ),
            preexec_fn=_launch_record_writer(socket_dir, launch_record, token),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 2.0
        while not socket_path.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert socket_path.exists(), process.stderr.read()

        # 原实现会在这里因尚无 eCAL subscriber 等待满五秒后退出。
        time.sleep(5.2)
        assert process.poll() is None, process.stderr.read()

        initialize_phase0_core(core, f"runsim-late-command-peer-{os.getpid()}")
        core_initialized = True
        subscriber = bindings.create_subscriber(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            load_v2_descriptor(),
            received_frames.append,
        )
        deadline = time.monotonic() + 2.0
        while subscriber.get_publisher_count() != 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert subscriber.get_publisher_count() == 1
        deadline = time.monotonic() + 2.0
        while not received_frames and time.monotonic() < deadline:
            time.sleep(0.01)
        assert received_frames, "Command did not publish a safe stop after peer discovery"

        _close_subscriber(subscriber)
        subscriber = None
        stdout, stderr = process.communicate(timeout=3)
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.communicate()
        if subscriber is not None:
            _close_subscriber(subscriber)
        if core_initialized:
            core.finalize()

    assert process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
    report = json.loads(result.read_text(encoding="utf-8"))
    assert report["safe_stop_published_count"] > 0


@pytest.mark.stage4_artifact
def test_interactive_command_is_the_exact_one_real_ecal_wheel_command_publisher(
    tmp_path: Path,
) -> None:
    """真实 eCAL peer 只能发现 Command 一个 command publisher，并收到其协商身份。"""
    _require_real_ecal_interactive()
    from scripts.verify_stage4_v2_phase0 import initialize_phase0_core
    from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
    from slope_sim.interfaces.v2.ecal_raw import EcalRawBindings

    command = _command_binary()
    root = Path(__file__).resolve().parents[2]
    descriptor = (root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc").read_bytes()
    descriptor_path = tmp_path / "v2.desc"
    descriptor_path.write_bytes(descriptor)
    payload = tmp_path / "identity.bin"
    _write_identity_payload(payload, descriptor)
    socket_dir = tmp_path / "socket"
    socket_path = socket_dir / "command.sock"
    launch_record = socket_dir / "command.launch.lock"
    result = tmp_path / "result.json"
    token = "b" * 64
    frames: list[pb.WheelCommand] = []
    bindings = EcalRawBindings()
    core = bindings._core
    subscriber = None
    process = None
    try:
        initialize_phase0_core(core, f"runsim-interactive-command-peer-{os.getpid()}")

        def receive(frame) -> None:
            message = pb.WheelCommand()
            assert message.ParseFromString(frame.payload)
            frames.append(message)

        subscriber = bindings.create_subscriber(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            load_v2_descriptor(),
            receive,
        )

        def write_launch_record() -> None:
            session = RunSimSession.create(
                socket_dir,
                command_pid=os.getpid(),
                command_uid=os.getuid(),
                orchestrator_pid=os.getppid(),
                session_id_factory=lambda: bytes.fromhex("73" * 16),
                token_factory=lambda: bytes.fromhex(token),
            )
            assert session.launch_record_path == launch_record
            assert session.server_authentication["token"] == token

        process = subprocess.Popen(
            [
                str(command),
                "--interactive",
                "--descriptor-set", str(descriptor_path),
                "--payload", str(payload),
                "--duration-ms", "300",
                "--deadline-ms", "1000",
                "--result", str(result),
                "--launch-record", str(launch_record),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=write_launch_record,
        )
        deadline = time.monotonic() + 2.0
        while (not socket_path.exists() or subscriber.get_publisher_count() != 1) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert socket_path.exists()
        assert subscriber.get_publisher_count() == 1
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(socket_path))
            client.sendall(
                json.dumps(
                    {
                        "kind": "target",
                        "token": token,
                        "linear_velocity_m_s": 0.4,
                        "angular_velocity_rad_s": 0.2,
                    },
                    separators=(",", ":"),
                ).encode("utf-8") + b"\n"
            )
            time.sleep(0.03)
        stdout, stderr = process.communicate(timeout=3)
        assert process.returncode == 0, f"stdout={stdout}\nstderr={stderr}"
        assert frames
        assert {frame.robot_model for frame in frames} == {"df_mid"}
        assert {bytes(frame.simulation_session_id) for frame in frames} == {b"s" * 16}
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.communicate()
        if subscriber is not None:
            _close_subscriber(subscriber)
        core.finalize()
