"""runSim v2：Dashboard 对唯一 C++ Recorder 的受认证编排。"""
from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
import secrets
import shutil
import signal
import socket
import subprocess
import tempfile
import time


_MID360_PATTERN_VERSION = "livox-mid360-800000-v1"
_MID360_PATTERN_SHA256 = "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca"
_RECORDER_DEADLINE_MS = 21_600_000


def _terminate_process_group(process: subprocess.Popen[object]) -> None:
    """有界回收自建 Recorder 进程组；SIGTERM 无效时升级为 SIGKILL。"""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            # 已发出 SIGKILL；清理路径不能因不可等待的子进程掩盖主失败。
            pass


def create_capture_output_dir(output_root: Path, *, now: datetime | None = None) -> Path:
    """以本地日期时间创建不可覆盖的 capture 目录，同秒按稳定序号递增。"""
    if not isinstance(output_root, Path) or not output_root.is_absolute():
        raise ValueError("output_root must be an absolute Path")
    output_root.mkdir(mode=0o755, parents=True, exist_ok=True)
    timestamp = (datetime.now() if now is None else now).strftime("%Y%m%d-%H%M%S")
    base = f"capture-{timestamp}"
    for suffix in range(10_000):
        name = base if suffix == 0 else f"{base}-{suffix}"
        candidate = output_root / name
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError("capture output directory suffix space is exhausted")


def write_latest_successful_lvx2_path(output_root: Path, lvx2_path: Path) -> Path:
    """原子更新最近一次成功 LVX2 的绝对路径，不让 Dashboard 读到半个文件。"""
    if not isinstance(output_root, Path) or not output_root.is_absolute() or not output_root.is_dir():
        raise ValueError("output_root must be an existing absolute directory")
    if not isinstance(lvx2_path, Path) or not lvx2_path.is_absolute() or not lvx2_path.is_file():
        raise ValueError("lvx2_path must be an existing absolute file")
    target = output_root / "last-successful-lvx2.json"
    descriptor, temporary_name = tempfile.mkstemp(prefix=".last-successful-lvx2.", suffix=".tmp", dir=output_root)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump({"lvx2_path": str(lvx2_path)}, stream, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(target)
        return target
    finally:
        temporary.unlink(missing_ok=True)


def load_latest_successful_lvx2_path(output_root: Path) -> Path | None:
    """读取已验证 marker；损坏、相对或已消失路径一律不恢复到 Dashboard。"""
    if not isinstance(output_root, Path) or not output_root.is_absolute():
        raise ValueError("output_root must be an absolute Path")
    marker = output_root / "last-successful-lvx2.json"
    try:
        document = json.loads(marker.read_text(encoding="utf-8"))
        value = document["lvx2_path"]
        if set(document) != {"lvx2_path"} or not isinstance(value, str):
            return None
        path = Path(value)
        return path if path.is_absolute() and path.is_file() else None
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
        return None


class RunSimV2Recorder:
    """管理一项 C++ Recorder 会话；控制面不承载任何点云数据。"""

    def __init__(
        self,
        *,
        process: subprocess.Popen[object],
        control_socket: Path,
        control_token: bytes,
        control_dir: Path,
        output_dir: Path,
    ) -> None:
        self._process = process
        self._control_socket = control_socket
        self._control_token = control_token
        self._control_dir = control_dir
        self._output_dir = output_dir

    @classmethod
    def launch(
        cls,
        *,
        release_root: Path,
        snapshot: object,
        scene_id: str,
        output_dir: Path,
    ) -> "RunSimV2Recorder":
        """从安装 release 启动唯一 Recorder，并等待其认证 socket 就绪。"""
        executable = release_root / "bin" / "slope_sim_stage4_recorder"
        descriptor_set = release_root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
        if not executable.is_file() or not executable.stat().st_mode & 0o111:
            raise RuntimeError(f"runSim v2 Recorder executable is unavailable: {executable}")
        if not descriptor_set.is_file():
            raise RuntimeError(f"runSim v2 descriptor set is unavailable: {descriptor_set}")
        control_dir = Path(tempfile.mkdtemp(prefix="runsim-recorder-", dir=tempfile.gettempdir()))
        os.chmod(control_dir, 0o700)
        token = secrets.token_bytes(32)
        control_socket = control_dir / "recorder.sock"
        process: subprocess.Popen[object] | None = None
        try:
            argv = build_interactive_recorder_argv(
                executable=executable,
                descriptor_set=descriptor_set,
                snapshot=snapshot,
                scene_id=scene_id,
                output_dir=output_dir,
                control_socket=control_socket,
                control_token=token,
            )
            process = subprocess.Popen(argv, start_new_session=True)
            deadline = time.monotonic() + 5.0
            while not control_socket.exists():
                if process.poll() is not None:
                    raise RuntimeError("runSim v2 Recorder exited before opening its control socket")
                if time.monotonic() >= deadline:
                    raise RuntimeError("runSim v2 Recorder did not open its control socket")
                time.sleep(0.01)
            return cls(
                process=process,
                control_socket=control_socket,
                control_token=token,
                control_dir=control_dir,
                output_dir=output_dir,
            )
        except BaseException:
            if process is not None and process.poll() is None:
                _terminate_process_group(process)
            shutil.rmtree(control_dir, ignore_errors=True)
            raise

    def start(self) -> None:
        """请求 Recorder 在下一完整五 topic 边界开启窗口。"""
        self._send("start")

    def stop(self) -> None:
        """请求 Recorder 在下一完整五 topic 边界完成窗口。"""
        self._send("stop")

    def close(self) -> None:
        """异常退出时先请求正常 stop，再有界回收 Recorder 进程组。"""
        if self._process.poll() is None:
            try:
                self.stop()
            except OSError:
                pass
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                _terminate_process_group(self._process)
        shutil.rmtree(self._control_dir, ignore_errors=True)

    def wait_for_success(self, *, timeout_sec: float) -> Path:
        """只接受 C++ Recorder 的 clean_shutdown 回执，并返回已关闭的 MCAP。"""
        if type(timeout_sec) not in {int, float} or timeout_sec <= 0:
            raise ValueError("timeout_sec must be positive")
        try:
            returncode = self._process.wait(timeout=float(timeout_sec))
            if returncode != 0:
                raise RuntimeError(f"runSim v2 Recorder exited with status {returncode}")
            result = self._output_dir / "recorder.result.json"
            document = json.loads(result.read_text(encoding="utf-8"))
            mcap = Path(document["mcap"])
            if document.get("clean_shutdown") is not True or document.get("exportable") is not True:
                raise RuntimeError("runSim v2 Recorder did not produce an exportable capture")
            if not mcap.is_absolute() or not mcap.is_file():
                raise RuntimeError("runSim v2 Recorder result has no valid MCAP")
            return mcap
        finally:
            shutil.rmtree(self._control_dir, ignore_errors=True)

    def export(self, *, release_root: Path, mcap_path: Path) -> tuple[Path, Path]:
        """使用同一 release 的 C++ Export 生成 PCD、PLY、LVX2 和结果回执。"""
        executable = release_root / "bin" / "slope_sim_stage4_export"
        descriptor_set = release_root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
        if not executable.is_file() or not executable.stat().st_mode & 0o111:
            raise RuntimeError(f"runSim v2 Export executable is unavailable: {executable}")
        subprocess.run(
            build_export_argv(
                executable=executable,
                descriptor_set=descriptor_set,
                mcap_path=mcap_path,
                output_dir=self._output_dir,
            ),
            check=True,
        )
        lvx2 = self._output_dir / "export" / "lidar.lvx2"
        result = self._output_dir / "export.result.json"
        if not lvx2.is_file() or not result.is_file():
            raise RuntimeError("runSim v2 Export did not publish LVX2 and result")
        write_latest_successful_lvx2_path(self._output_dir.parent, lvx2)
        return lvx2, result

    def _send(self, kind: str) -> None:
        if kind not in {"start", "stop"}:
            raise ValueError("Recorder control kind is invalid")
        if self._process.poll() is not None:
            raise RuntimeError("runSim v2 Recorder is no longer running")
        document = json.dumps(
            {"kind": kind, "token": self._control_token.hex()},
            separators=(",", ":"),
        ).encode("utf-8")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(self._control_socket))
            client.sendall(document)


def build_interactive_recorder_argv(
    *,
    executable: Path,
    descriptor_set: Path,
    snapshot: object,
    scene_id: str,
    output_dir: Path,
    control_socket: Path,
    control_token: bytes,
) -> list[str]:
    """构造 C++ Recorder argv；identity、路径和 token 均在启动前冻结。"""
    if not isinstance(executable, Path) or not executable.is_absolute():
        raise ValueError("executable must be an absolute Path")
    if not isinstance(descriptor_set, Path) or not descriptor_set.is_absolute():
        raise ValueError("descriptor_set must be an absolute Path")
    if not isinstance(output_dir, Path) or not output_dir.is_absolute() or not output_dir.is_dir():
        raise ValueError("output_dir must be an existing absolute directory")
    if not isinstance(control_socket, Path) or not control_socket.is_absolute():
        raise ValueError("control_socket must be an absolute Path")
    if not isinstance(scene_id, str) or not scene_id:
        raise ValueError("scene_id must be nonempty")
    if not isinstance(control_token, bytes) or len(control_token) != 32:
        raise ValueError("control_token must be exactly 32 bytes")
    session_id = getattr(snapshot, "simulation_session_id", None)
    world_generation = getattr(snapshot, "world_generation", None)
    if not isinstance(session_id, bytes) or len(session_id) != 16:
        raise ValueError("snapshot must expose a 16-byte simulation_session_id")
    if type(world_generation) is not int or world_generation <= 0:
        raise ValueError("snapshot must expose a positive world_generation")
    output = output_dir / "session.mcap"
    result = output_dir / "recorder.result.json"
    if output.exists() or result.exists():
        raise ValueError("Recorder output paths must be new")
    return [
        str(executable), "--interactive",
        "--descriptor-set", str(descriptor_set),
        "--scene-id", scene_id,
        "--simulation-session-id", session_id.hex(),
        "--world-generation", str(world_generation),
        "--lidar-pattern-version", _MID360_PATTERN_VERSION,
        "--lidar-pattern-sha256", _MID360_PATTERN_SHA256,
        "--output", str(output),
        "--deadline-ms", str(_RECORDER_DEADLINE_MS),
        "--result", str(result),
        "--control-socket", str(control_socket),
        "--control-token", control_token.hex(),
    ]


def build_export_argv(
    *,
    executable: Path,
    descriptor_set: Path,
    mcap_path: Path,
    output_dir: Path,
) -> list[str]:
    """构造同 release C++ Export 参数，输出目录和结果文件均要求未存在。"""
    if not isinstance(executable, Path) or not executable.is_absolute():
        raise ValueError("executable must be an absolute Path")
    if not isinstance(descriptor_set, Path) or not descriptor_set.is_absolute() or not descriptor_set.is_file():
        raise ValueError("descriptor_set must be an existing absolute file")
    if not isinstance(mcap_path, Path) or not mcap_path.is_absolute() or not mcap_path.is_file():
        raise ValueError("mcap_path must be an existing absolute file")
    if not isinstance(output_dir, Path) or not output_dir.is_absolute() or not output_dir.is_dir():
        raise ValueError("output_dir must be an existing absolute directory")
    export_dir = output_dir / "export"
    result = output_dir / "export.result.json"
    if export_dir.exists() or result.exists():
        raise ValueError("Export output paths must be new")
    return [
        str(executable),
        "--input", str(mcap_path),
        "--descriptor-set", str(descriptor_set),
        "--output-dir", str(export_dir),
        "--result", str(result),
    ]
