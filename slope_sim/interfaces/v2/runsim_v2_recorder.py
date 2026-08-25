"""runSim v2：Dashboard 对唯一 C++ Recorder 的受认证编排。"""
from __future__ import annotations

from dataclasses import dataclass
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


@dataclass(frozen=True, slots=True)
class CaptureOutputDirectories:
    """同一输出根中的暂存与最终采集目录。"""

    staging_dir: Path
    published_dir: Path


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
        name = base if suffix == 0 else f"{base}-{suffix:02d}"
        candidate = output_root / name
        try:
            candidate.mkdir()
            return candidate
        except FileExistsError:
            continue
    raise RuntimeError("capture output directory suffix space is exhausted")


def prepare_capture_output_dirs(
    output_root: Path, *, now: datetime | None = None,
) -> CaptureOutputDirectories:
    """预留最终目录名，但只创建隐藏暂存目录以隔离未完成采集。"""
    if not isinstance(output_root, Path) or not output_root.is_absolute():
        raise ValueError("output_root must be an absolute Path")
    output_root.mkdir(mode=0o755, parents=True, exist_ok=True)
    timestamp = (datetime.now() if now is None else now).strftime("%Y%m%d-%H%M%S")
    base = f"capture-{timestamp}"
    for suffix in range(10_000):
        name = base if suffix == 0 else f"{base}-{suffix:02d}"
        published = output_root / name
        staging = output_root / f".{name}.tmp"
        if published.exists():
            continue
        try:
            staging.mkdir()
        except FileExistsError:
            continue
        return CaptureOutputDirectories(staging, published)
    raise RuntimeError("capture output directory suffix space is exhausted")


def _capture_artifact_paths(output_dir: Path) -> tuple[str, ...]:
    """确认导出器交付完整的原始、转换和回执文件。"""
    required = (
        "session.mcap",
        "recorder.result.json",
        "export.result.json",
        "export/lidar.lvx2",
    )
    missing = [name for name in required if not (output_dir / name).is_file()]
    pcd = tuple(sorted(output_dir.glob("export/**/*.pcd")))
    ply = tuple(sorted(output_dir.glob("export/**/*.ply")))
    if missing or not pcd or not ply:
        detail = ", ".join([*missing, "PCD" if not pcd else "", "PLY" if not ply else ""])
        raise RuntimeError(f"capture export is incomplete: {detail.strip(', ')}")
    return (
        *(str(path.relative_to(output_dir)) for path in pcd),
        *(str(path.relative_to(output_dir)) for path in ply),
        "export/lidar.lvx2",
        "export.result.json",
        "recorder.result.json",
        "session.mcap",
    )


def _write_capture_manifest(output_dir: Path, artifacts: tuple[str, ...]) -> Path:
    """在暂存目录内原子写入完成清单，改名前不对 Dashboard 可见。"""
    target = output_dir / "capture.manifest.json"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".capture.manifest.", suffix=".tmp", dir=output_dir,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump({"status": "complete", "artifacts": list(artifacts)}, stream, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(target)
        return target
    finally:
        temporary.unlink(missing_ok=True)


def _rebind_published_recorder_result(paths: CaptureOutputDirectories) -> None:
    """暂存目录改名前，将 C++ 回执绑定到即将公开的 MCAP 绝对路径。"""
    result_path = paths.staging_dir / "recorder.result.json"
    try:
        document = json.loads(result_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError("capture Recorder receipt is invalid") from error
    staging_mcap = paths.staging_dir / "session.mcap"
    published_mcap = paths.published_dir / "session.mcap"
    if not isinstance(document, dict) or document.get("mcap") != str(staging_mcap):
        raise RuntimeError("capture Recorder receipt does not bind its staged MCAP")
    document["mcap"] = str(published_mcap)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".recorder.result.", suffix=".tmp", dir=paths.staging_dir,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(document, stream, separators=(",", ":"))
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(result_path)
    finally:
        temporary.unlink(missing_ok=True)


def mark_capture_output_incomplete(output_dir: Path, reason: str) -> Path:
    """为暂存失败保留有界诊断；它绝不更新最近成功导出。"""
    if not isinstance(output_dir, Path) or not output_dir.is_dir():
        raise ValueError("output_dir must be an existing directory")
    if not isinstance(reason, str) or not reason:
        raise ValueError("reason must be a nonempty string")
    target = output_dir / "capture.incomplete.json"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".capture.incomplete.", suffix=".tmp", dir=output_dir,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump({"status": "incomplete", "reason": reason}, stream, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(target)
        return target
    finally:
        temporary.unlink(missing_ok=True)


def publish_capture_output(paths: CaptureOutputDirectories) -> Path:
    """校验暂存采集后原子改名发布，最终目录绝不包含半成品。"""
    if type(paths) is not CaptureOutputDirectories:
        raise ValueError("paths must be CaptureOutputDirectories")
    if (
        not paths.staging_dir.is_dir()
        or paths.published_dir.exists()
        or paths.staging_dir.parent != paths.published_dir.parent
    ):
        raise ValueError("capture output directories are invalid")
    artifacts = _capture_artifact_paths(paths.staging_dir)
    _rebind_published_recorder_result(paths)
    _write_capture_manifest(paths.staging_dir, artifacts)
    paths.staging_dir.replace(paths.published_dir)
    return paths.published_dir


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
        published_output_dir: Path | None = None,
    ) -> None:
        self._process = process
        self._control_socket = control_socket
        self._control_token = control_token
        self._control_dir = control_dir
        self._output_dir = output_dir
        self._published_output_dir = (
            output_dir if published_output_dir is None else published_output_dir
        )

    @classmethod
    def launch(
        cls,
        *,
        release_root: Path,
        snapshot: object,
        scene_id: str,
        output_dir: Path,
        published_output_dir: Path | None = None,
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
                published_output_dir=published_output_dir,
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
        try:
            subprocess.run(
                build_export_argv(
                    executable=executable,
                    descriptor_set=descriptor_set,
                    mcap_path=mcap_path,
                    output_dir=self._output_dir,
                ),
                check=True,
            )
            staging_lvx2 = self._output_dir / "export" / "lidar.lvx2"
            staging_result = self._output_dir / "export.result.json"
            if not staging_lvx2.is_file() or not staging_result.is_file():
                raise RuntimeError("runSim v2 Export did not publish LVX2 and result")
            if self._published_output_dir != self._output_dir:
                publish_capture_output(
                    CaptureOutputDirectories(self._output_dir, self._published_output_dir)
                )
            lvx2 = self._published_output_dir / "export" / "lidar.lvx2"
            result = self._published_output_dir / "export.result.json"
            write_latest_successful_lvx2_path(self._published_output_dir.parent, lvx2)
            return lvx2, result
        except Exception as error:
            if self._published_output_dir != self._output_dir and self._output_dir.is_dir():
                mark_capture_output_incomplete(
                    self._output_dir, str(error) or type(error).__name__,
                )
            raise

    def _send(self, kind: str) -> None:
        if kind not in {"start", "stop"}:
            raise ValueError("Recorder control kind is invalid")
        returncode = self._process.poll()
        if returncode is not None:
            raise self._child_failure(returncode)
        document = json.dumps(
            {"kind": kind, "token": self._control_token.hex()},
            separators=(",", ":"),
        ).encode("utf-8")
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.connect(str(self._control_socket))
            client.sendall(document)

    def _child_failure(self, returncode: object) -> RuntimeError:
        """优先保留 C++ receipt 已锁存的首个故障，避免 stop 覆盖它。"""
        try:
            document = json.loads((self._output_dir / "recorder.result.json").read_text(encoding="utf-8"))
            reason = document.get("fault_reason") if isinstance(document, dict) else None
            if (isinstance(document, dict) and document.get("clean_shutdown") is False
                    and isinstance(reason, str) and reason):
                return RuntimeError(f"runSim v2 Recorder failed: {reason}")
        except (OSError, UnicodeError, json.JSONDecodeError):
            pass
        return RuntimeError(f"runSim v2 Recorder exited with status {returncode}")


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
