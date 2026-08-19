"""runSim 本机 Command socket：编排器端会话、认证记录和小消息合同。"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum
import fcntl
import hmac
import json
import math
import os
from pathlib import Path
import secrets
import tempfile
import threading
from typing import Callable, Iterator


PROTOCOL = "runsim-command-socket-v1"
MANUAL_TARGET_LEASE_SEC = 0.100
MAX_SOCKET_MESSAGE_BYTES = 1024
MAX_LINEAR_VELOCITY_M_S = 1.2
MAX_ANGULAR_VELOCITY_RAD_S = 1.2
_UNIX_SOCKET_PATH_MAX_BYTES = 107


class RunSimSessionState(str, Enum):
    """正式 runSim 会话向 Dashboard 暴露的最小生命周期。"""

    LAUNCHING = "launching"
    READY = "ready"
    ACTIVE = "active"
    SAFE_STOP = "safe_stop"
    STOPPING = "stopping"
    CLOSED = "closed"


@dataclass(frozen=True)
class ControlMessage:
    """三种允许 JSON 消息中的一条；永不承载传感器或点云数据。"""

    kind: str
    linear_velocity_m_s: float | None = None
    angular_velocity_rad_s: float | None = None
    state: str | None = None
    reason: str | None = None


@dataclass(frozen=True)
class RunSimSessionSnapshot:
    """供编排器/Dashboard 读取的不可变会话状态，不泄漏认证 token。"""

    state: RunSimSessionState
    socket_path: Path
    command_pid: int
    command_uid: int
    linear_velocity_m_s: float
    angular_velocity_rad_s: float
    safe_stop_reason: str | None
    last_target_monotonic: float | None


class RunSimSession:
    """由编排器持有的 Command 启动锁、SO_PEERCRED 约束和人工目标租约。"""

    def __init__(
        self,
        *,
        socket_dir: Path,
        launch_record_path: Path,
        command_pid: int,
        command_uid: int,
        orchestrator_pid: int,
        session_id: bytes,
        token: bytes,
    ) -> None:
        self._lock = threading.RLock()
        self.socket_dir = socket_dir
        self.socket_path = socket_dir / "command.sock"
        self.launch_record_path = launch_record_path
        self._command_pid = command_pid
        self._command_uid = command_uid
        self._orchestrator_pid = orchestrator_pid
        self._session_id = session_id
        self._token = token
        self._state = RunSimSessionState.LAUNCHING
        self._linear_velocity_m_s = 0.0
        self._angular_velocity_rad_s = 0.0
        self._safe_stop_reason: str | None = None
        self._last_target_monotonic: float | None = None
        self._target_epoch = 0
        self._finalized = False

    @classmethod
    def create(
        cls,
        socket_dir: Path,
        *,
        command_pid: int,
        command_uid: int | None = None,
        orchestrator_pid: int | None = None,
        session_id_factory: Callable[[], bytes] = lambda: secrets.token_bytes(16),
        token_factory: Callable[[], bytes] = lambda: secrets.token_bytes(32),
    ) -> "RunSimSession":
        """原子写入唯一 Command 启动锁，供其 server 与客户端各自认证。"""
        path = _prepare_socket_dir(socket_dir)
        _require_pid("command_pid", command_pid)
        if command_uid is None:
            command_uid = os.getuid()
        if orchestrator_pid is None:
            orchestrator_pid = os.getpid()
        _require_uid("command_uid", command_uid)
        _require_pid("orchestrator_pid", orchestrator_pid)
        if not callable(session_id_factory) or not callable(token_factory):
            raise ValueError("session_id_factory and token_factory must be callable")
        session_id = _require_bytes("session_id", session_id_factory(), 16)
        token = _require_bytes("token", token_factory(), 32)
        socket_path = path / "command.sock"
        if len(os.fsencode(socket_path)) > _UNIX_SOCKET_PATH_MAX_BYTES:
            raise ValueError("Unix socket path exceeds 107 bytes")
        lock_path = path / "command.launch.lock"
        record = {
            "command_pid": command_pid,
            "command_uid": command_uid,
            "orchestrator_pid": orchestrator_pid,
            "protocol": PROTOCOL,
            "session_id": session_id.hex(),
            "socket_path": str(socket_path),
            "token": token.hex(),
        }
        with _lifecycle_lock(path / "command.lifecycle.lock"):
            _create_launch_lock(lock_path, record)
        return cls(
            socket_dir=path,
            launch_record_path=lock_path,
            command_pid=command_pid,
            command_uid=command_uid,
            orchestrator_pid=orchestrator_pid,
            session_id=session_id,
            token=token,
        )

    @classmethod
    def attach(
        cls,
        launch_record_path: Path,
        *,
        command_pid: int,
        command_uid: int,
        orchestrator_pid: int,
    ) -> "RunSimSession":
        """校验并接管 child pre-exec 发布的启动记录，不重写认证材料。"""
        _require_pid("command_pid", command_pid)
        _require_uid("command_uid", command_uid)
        _require_pid("orchestrator_pid", orchestrator_pid)
        if not isinstance(launch_record_path, Path):
            raise ValueError("launch_record_path must be a Path")
        path = launch_record_path.resolve(strict=True)
        socket_dir = path.parent
        if (
            path.name != "command.launch.lock"
            or launch_record_path.is_symlink()
            or not path.is_file()
            or path.stat().st_uid != os.getuid()
            or path.stat().st_mode & 0o777 != 0o600
            or socket_dir.is_symlink()
            or not socket_dir.is_dir()
            or socket_dir.stat().st_uid != os.getuid()
            or socket_dir.stat().st_mode & 0o777 != 0o700
        ):
            raise ValueError("Command launch record is invalid")
        try:
            record = json.loads(path.read_text(encoding="utf-8"))
            expected_keys = {
                "command_pid", "command_uid", "orchestrator_pid", "protocol",
                "session_id", "socket_path", "token",
            }
            if not isinstance(record, dict) or set(record) != expected_keys:
                raise ValueError
            if (
                record["command_pid"] != command_pid
                or record["command_uid"] != command_uid
                or record["orchestrator_pid"] != orchestrator_pid
                or record["protocol"] != PROTOCOL
                or record["socket_path"] != str(socket_dir / "command.sock")
            ):
                raise ValueError
            session_id = bytes.fromhex(record["session_id"])
            token = bytes.fromhex(record["token"])
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise ValueError("Command launch record is invalid") from error
        _require_bytes("session_id", session_id, 16)
        _require_bytes("token", token, 32)
        return cls(
            socket_dir=socket_dir,
            launch_record_path=path,
            command_pid=command_pid,
            command_uid=command_uid,
            orchestrator_pid=orchestrator_pid,
            session_id=session_id,
            token=token,
        )

    @property
    def server_authentication(self) -> dict[str, object]:
        """传给刚启动的 C++ server：其需自验 PID，并以 SO_PEERCRED 验证 uid。"""
        with self._lock:
            return {
                "command_pid": self._command_pid,
                "command_uid": self._command_uid,
                "protocol": PROTOCOL,
                "session_id": self._session_id.hex(),
                "token": self._token.hex(),
            }

    def close(self) -> None:
        """幂等关闭会话，仅回收仍归本会话所有的含 token 启动锁。"""
        with self._lock:
            if self._finalized:
                return
            record = {
                "command_pid": self._command_pid,
                "command_uid": self._command_uid,
                "orchestrator_pid": self._orchestrator_pid,
                "protocol": PROTOCOL,
                "session_id": self._session_id.hex(),
                "socket_path": str(self.socket_path),
                "token": self._token.hex(),
            }
            self._safe_stop("session_closed", RunSimSessionState.CLOSED)
            try:
                # create/close 都在同一把跨进程锁内，避免比对后误删继任记录。
                with _lifecycle_lock(self.socket_dir / "command.lifecycle.lock"):
                    _remove_launch_lock_if_owned(self.launch_record_path, record)
            finally:
                # 回收 I/O 出错也不得留下可继续使用的认证凭据或可重试关闭状态。
                self._session_id = b""
                self._token = b""
                self._finalized = True

    def finalize(self) -> None:
        """关闭别名，供编排器的 finally 路径无条件调用。"""
        self.close()

    def verify_command_peer(self, *, pid: int, uid: int) -> None:
        """客户端以已连接 socket 的 SO_PEERCRED 核对刚启动的 Command server。"""
        _require_pid("Command peer pid", pid)
        _require_uid("Command peer uid", uid)
        with self._lock:
            if pid != self._command_pid:
                raise PermissionError("unknown Command PID")
            if uid != self._command_uid:
                raise PermissionError("Command peer uid does not match launch record")

    def accept_client_message(
        self,
        payload: bytes,
        *,
        client_pid: int,
        peer_uid: int,
        now: float,
    ) -> ControlMessage:
        """按 socket 合同验证一条 JSON 消息，并将安全边沿投影到本地快照。"""
        with self._lock:
            target_epoch = self._target_epoch
            expected_token = self._token.hex()
        self.verify_client_peer(pid=client_pid, uid=peer_uid)
        document = _decode_control_document(payload)
        token = document.get("token")
        if not isinstance(token, str) or not hmac.compare_digest(token, expected_token):
            raise ValueError("control message token does not match session")
        kind = document.get("kind")
        if kind == "target":
            _require_exact_keys(
                document,
                {"kind", "token", "linear_velocity_m_s", "angular_velocity_rad_s"},
            )
            linear = _require_target("linear_velocity_m_s", document["linear_velocity_m_s"], MAX_LINEAR_VELOCITY_M_S)
            angular = _require_target("angular_velocity_rad_s", document["angular_velocity_rad_s"], MAX_ANGULAR_VELOCITY_RAD_S)
            self._accept_target(linear, angular, now, target_epoch=target_epoch)
            return ControlMessage(kind="target", linear_velocity_m_s=linear, angular_velocity_rad_s=angular)
        if kind == "status":
            _require_exact_keys(document, {"kind", "token", "state"})
            state = document["state"]
            if state not in {item.value for item in RunSimSessionState}:
                raise ValueError("status state is invalid")
            self._accept_status(RunSimSessionState(state))
            return ControlMessage(kind="status", state=state)
        if kind == "stop":
            _require_exact_keys(document, {"kind", "token", "reason"})
            reason = document["reason"]
            if not isinstance(reason, str) or not reason or len(reason) > 128:
                raise ValueError("stop reason is invalid")
            self._safe_stop("stop_request", RunSimSessionState.STOPPING)
            return ControlMessage(kind="stop", reason=reason)
        raise ValueError("control message kind is invalid")

    def verify_client_peer(self, *, pid: int, uid: int) -> None:
        """供 C++ server 的 SO_PEERCRED 判据：客户端 PID 可变，但 uid 必须相同。"""
        _require_pid("control client pid", pid)
        _require_uid("control client uid", uid)
        with self._lock:
            if uid != self._command_uid:
                raise PermissionError("control client uid does not match launch record")

    def connection_closed(self) -> None:
        """连接关闭即丢弃所有人工驾驶目标。"""
        self._safe_stop("connection_closed", RunSimSessionState.SAFE_STOP)

    def keyboard_released(self) -> None:
        """键盘释放事件立即取消人工驾驶目标。"""
        self._safe_stop("keyboard_released", RunSimSessionState.SAFE_STOP)

    def window_focus_lost(self) -> None:
        """控制窗口失焦立即取消人工驾驶目标。"""
        self._safe_stop("window_focus_lost", RunSimSessionState.SAFE_STOP)

    def expire_manual_target(self, *, now: float) -> bool:
        """由单调墙钟租约到期触发安全停车，返回本次是否发生状态转换。"""
        monotonic_now = _require_now(now)
        with self._lock:
            if self._last_target_monotonic is None:
                return False
            if monotonic_now < self._last_target_monotonic + MANUAL_TARGET_LEASE_SEC:
                return False
            self._safe_stop("manual_target_timeout", RunSimSessionState.SAFE_STOP)
            return True

    def snapshot(self) -> RunSimSessionSnapshot:
        """返回当前状态，而不是直接暴露可变会话字段或认证 token。"""
        with self._lock:
            return RunSimSessionSnapshot(
                state=self._state,
                socket_path=self.socket_path,
                command_pid=self._command_pid,
                command_uid=self._command_uid,
                linear_velocity_m_s=self._linear_velocity_m_s,
                angular_velocity_rad_s=self._angular_velocity_rad_s,
                safe_stop_reason=self._safe_stop_reason,
                last_target_monotonic=self._last_target_monotonic,
            )

    def _accept_target(
        self, linear: float, angular: float, now: float, *, target_epoch: int
    ) -> None:
        """只有认证且有界的目标能刷新 100 ms 租约。"""
        monotonic_now = _require_now(now)
        with self._lock:
            if target_epoch != self._target_epoch:
                raise ValueError("runSim target was revoked before commit")
            if self._state in {RunSimSessionState.STOPPING, RunSimSessionState.CLOSED}:
                raise ValueError("runSim session is stopping")
            self._linear_velocity_m_s = linear
            self._angular_velocity_rad_s = angular
            self._last_target_monotonic = monotonic_now
            self._safe_stop_reason = None
            self._state = RunSimSessionState.ACTIVE

    def _accept_status(self, state: RunSimSessionState) -> None:
        """将合法 Command 生命周期投影给快照，终态只允许保持或继续关闭。"""
        with self._lock:
            if self._state is RunSimSessionState.CLOSED and state is not RunSimSessionState.CLOSED:
                raise ValueError("status transition cannot revive a closed runSim session")
            if self._state is RunSimSessionState.STOPPING and state not in {
                RunSimSessionState.STOPPING,
                RunSimSessionState.CLOSED,
            }:
                raise ValueError("status transition cannot revive a stopping runSim session")
            if state in {
                RunSimSessionState.SAFE_STOP,
                RunSimSessionState.STOPPING,
                RunSimSessionState.CLOSED,
            }:
                self._safe_stop(f"status_{state.value}", state)
                return
            self._state = state

    def _safe_stop(self, reason: str, state: RunSimSessionState) -> None:
        """所有撤权路径共用同一清零操作，避免某个边沿留下旧 target。"""
        with self._lock:
            self._target_epoch += 1
            self._linear_velocity_m_s = 0.0
            self._angular_velocity_rad_s = 0.0
            self._last_target_monotonic = None
            self._safe_stop_reason = reason
            if state is RunSimSessionState.SAFE_STOP and self._state in {
                RunSimSessionState.STOPPING,
                RunSimSessionState.CLOSED,
            }:
                return
            if state is RunSimSessionState.STOPPING and self._state is RunSimSessionState.CLOSED:
                return
            self._state = state


def _prepare_socket_dir(value: Path) -> Path:
    """建立并验证只能由本用户访问的 socket 目录。"""
    if not isinstance(value, Path) or not value.is_absolute():
        raise ValueError("socket_dir must be an absolute Path")
    try:
        value.mkdir(mode=0o700, parents=True, exist_ok=False)
    except FileExistsError:
        metadata = value.lstat()
        if value.is_symlink() or not stat_is_directory(metadata.st_mode):
            raise ValueError("socket_dir must be a real directory")
    os.chmod(value, 0o700)
    return value


def _create_launch_lock(path: Path, record: dict[str, object]) -> None:
    """先落盘私有临时记录，再以不覆盖发布让 C++ 只能读到完整内容。"""
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        encoded = json.dumps(record, sort_keys=True, separators=(",", ":")).encode("utf-8")
        offset = 0
        while offset < len(encoded):
            offset += os.write(descriptor, encoded[offset:])
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = -1
        try:
            os.link(temporary_path, path)
        except FileExistsError as error:
            raise FileExistsError("Command launch lock already exists") from error
    finally:
        if descriptor != -1:
            os.close(descriptor)
        temporary_path.unlink(missing_ok=True)


@contextmanager
def _lifecycle_lock(path: Path) -> Iterator[None]:
    """以持久私有文件串行化本模块的启动记录发布与回收。"""
    descriptor = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
    locked = False
    try:
        os.fchmod(descriptor, 0o600)
        fcntl.flock(descriptor, fcntl.LOCK_EX)
        locked = True
        yield
    finally:
        if locked:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        os.close(descriptor)


def _remove_launch_lock_if_owned(path: Path, record: dict[str, object]) -> None:
    """只有完整启动记录仍匹配本会话时才删除，防止误删继任者的锁。"""
    try:
        existing = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return
    if existing == record:
        path.unlink(missing_ok=True)


def _decode_control_document(payload: bytes) -> dict[str, object]:
    """严格解码小 JSON；任意点云字段、未知字段和超长内容均 fail closed。"""
    if not isinstance(payload, bytes) or len(payload) > MAX_SOCKET_MESSAGE_BYTES:
        raise ValueError("control payload exceeds maximum size")
    try:
        document = json.loads(
            payload.decode("utf-8"),
            parse_constant=lambda _constant: (_ for _ in ()).throw(ValueError("non-finite JSON number")),
            object_pairs_hook=_reject_duplicate_json_fields,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        raise ValueError("control payload is invalid JSON") from error
    if not isinstance(document, dict):
        raise ValueError("control payload must be a JSON object")
    if any("pointcloud" in key.lower() for key in document if isinstance(key, str)):
        raise ValueError("pointcloud fields are forbidden on the control socket")
    return document


def _reject_duplicate_json_fields(pairs: list[tuple[object, object]]) -> dict[object, object]:
    """JSON 重复字段会造成认证值覆盖，解码时必须立即拒绝。"""
    document: dict[object, object] = {}
    for key, value in pairs:
        if key in document:
            raise ValueError("duplicate JSON field")
        document[key] = value
    return document


def _require_exact_keys(document: dict[str, object], expected: set[str]) -> None:
    """消息不允许扩展字段，因而 socket 不会悄悄演化为数据面。"""
    if set(document) != expected:
        raise ValueError("control message fields are invalid")


def _require_target(name: str, value: object, maximum: float) -> float:
    """人工 target 只接受与手动 UI 一致的有限速度范围。"""
    if type(value) not in {int, float} or not math.isfinite(float(value)) or abs(float(value)) > maximum:
        raise ValueError(f"{name} is invalid")
    return float(value)


def _require_now(value: object) -> float:
    """租约时钟必须是有限单调墙钟数值。"""
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise ValueError("now must be a finite monotonic timestamp")
    return float(value)


def _require_pid(name: str, value: object) -> int:
    """PID 需要是正的精确整数，拒绝 Python bool 的整数兼容性。"""
    if type(value) is not int or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def _require_uid(name: str, value: object) -> int:
    """uid 需要是非负精确整数，供 SO_PEERCRED 对照。"""
    if type(value) is not int or value < 0:
        raise ValueError(f"{name} must be a nonnegative integer")
    return value


def _require_bytes(name: str, value: object, size: int) -> bytes:
    """启动记录固定 session/token 长度，避免 C++ 端解析歧义。"""
    if not isinstance(value, bytes) or len(value) != size:
        raise ValueError(f"{name} must be exactly {size} bytes")
    return value


def stat_is_directory(mode: int) -> bool:
    """避免为一个 lstat 分支引入额外项目依赖。"""
    return (mode & 0o170000) == 0o040000
