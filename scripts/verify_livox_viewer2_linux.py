# Livox Viewer 2 Linux 兼容启动器：隔离多网卡环境而保留 X11 与 GPU 访问。
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping, Sequence


_PROXY_VARIABLES = (
    "HTTP_PROXY",
    "HTTPS_PROXY",
    "ALL_PROXY",
    "http_proxy",
    "https_proxy",
    "all_proxy",
)
_REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
_DEFAULT_VIEWER_ROOT = (
    _REPOSITORY_ROOT / "share/slope-sim/livox-viewer/Viewer2_2.6.0_Linux"
)
_VIEWER_ROOT_ENV = "SLOPE_SIM_LIVOX_VIEWER_ROOT"
_VIEWER_WINDOW_ARGS = ("-windowed", "-ResX=1600", "-ResY=900")
_VIEWER_OPEN_FILE_BUTTON = (230, 18)
_VIEWER_DIALOG_REFERENCE_SIZE = (810, 534)
_VIEWER_FILENAME_FIELD = (577, 461)
_VIEWER_OPEN_CONFIRM_BUTTON = (683, 495)
_RENDERED_POINT_COUNT = re.compile(r"RenderMultiFrame,.*?PointsNum\s*=\s*(\d+)")


@dataclass(frozen=True, slots=True)
class ViewerPlaybackEvidence:
    """本次 LVX2 导入完成状态链的官方 Viewer 日志证据。"""

    opened: bool
    simulated_mid360_selected: bool
    playback_started: bool
    rendered_point_count: int | None

    @property
    def complete(self) -> bool:
        """仅当四项状态全部确认时，Dashboard 才能报告导入成功。"""
        return (
            self.opened
            and self.simulated_mid360_selected
            and self.playback_started
            and self.rendered_point_count is not None
            and self.rendered_point_count > 0
        )


def _require_bwrap() -> None:
    """所有库与 CLI 入口共用同一 bubblewrap 前置门禁。"""
    if shutil.which("bwrap") is None:
        raise RuntimeError(
            "bubblewrap (bwrap) is required for isolated Livox Viewer startup"
        )


def collect_network_evidence(
    *,
    host_pid: int,
    viewer_pid: int,
    proc_root: Path = Path("/proc"),
) -> dict[str, object]:
    """读取 host 与 Viewer 网络命名空间，记录 Viewer 可见网卡。"""
    host_namespace = os.readlink(proc_root / str(host_pid) / "ns" / "net")
    viewer_root = proc_root / str(viewer_pid)
    viewer_namespace = os.readlink(viewer_root / "ns" / "net")
    device_lines = (viewer_root / "net" / "dev").read_text(encoding="utf-8").splitlines()
    interfaces = [
        line.split(":", 1)[0].strip()
        for line in device_lines
        if ":" in line and line.split(":", 1)[0].strip()
    ]
    no_external_interfaces = interfaces == ["lo"]
    return {
        "host_network_namespace": host_namespace,
        "viewer_network_namespace": viewer_namespace,
        "viewer_interfaces": interfaces,
        "network_isolated": host_namespace != viewer_namespace,
        "no_external_interfaces": no_external_interfaces,
    }


def write_network_evidence(evidence_dir: Path, evidence: dict[str, object]) -> Path:
    """排他写入一次真实 Viewer 网络隔离证据。"""
    evidence_dir.mkdir(parents=True, exist_ok=True)
    path = evidence_dir / "network-evidence.json"
    with path.open("x", encoding="utf-8") as handle:
        json.dump(evidence, handle, sort_keys=True)
        handle.write("\n")
    return path


def viewer_map_loaded(log_path: Path) -> bool:
    """只接受官方日志确认的 Viewer 主地图加载完成。"""
    try:
        return "LoadMap(/Game/Maps/Viewer)" in log_path.read_text(
            encoding="utf-8", errors="replace"
        )
    except OSError:
        return False


def viewer_lvx2_opened(log_path: Path, lvx2_path: Path) -> bool:
    """只接受官方日志确认本次绝对路径的 LVX2 已成功打开。"""
    marker = f"ALvxKit::OpenLvxFile({lvx2_path}) success"
    try:
        return marker in log_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return False


def viewer_playback_evidence(log_path: Path, lvx2_path: Path) -> ViewerPlaybackEvidence:
    """解析官方日志的打开、MID-360 选择、播放和非零渲染状态。"""
    try:
        log = log_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        log = ""
    opened = f"ALvxKit::OpenLvxFile({lvx2_path}) success" in log
    simulated_mid360_selected = (
        "device type = Mid-360 (9)" in log
        and "PointCloudSelectionChanged" in log
    )
    playback_started = (
        "AViewerUI::OnDeviceEvent PlayLvxStart" in log
        or "ALvxKit::PlayerPlay" in log
    )
    counts = [int(match.group(1)) for match in _RENDERED_POINT_COUNT.finditer(log)]
    rendered_point_count = next((count for count in reversed(counts) if count > 0), None)
    return ViewerPlaybackEvidence(
        opened=opened,
        simulated_mid360_selected=simulated_mid360_selected,
        playback_started=playback_started,
        rendered_point_count=rendered_point_count,
    )


def _run_xdotool(
    args: Sequence[str],
    *,
    display: str,
    run_command: Callable[..., object],
) -> object:
    """在指定 DISPLAY 上运行一次 xdotool，并保留标准输出用于窗口定位。"""
    environment = dict(os.environ)
    environment["DISPLAY"] = display
    try:
        return run_command(
            ["xdotool", *args],
            env=environment,
            check=False,
            text=True,
            capture_output=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("xdotool is required for Livox Viewer file import") from exc


def _visible_window_id(
    *,
    viewer_pid: int,
    title_pattern: str,
    display: str,
    run_command: Callable[..., object],
) -> str | None:
    result = _run_xdotool(
        (
            "search",
            "--all",
            "--onlyvisible",
            "--pid",
            str(viewer_pid),
            "--name",
            title_pattern,
        ),
        display=display,
        run_command=run_command,
    )
    if getattr(result, "returncode", 1) != 0:
        return None
    window_ids = str(getattr(result, "stdout", "")).split()
    return window_ids[-1] if window_ids else None


def _window_geometry(
    window_id: str,
    *,
    display: str,
    run_command: Callable[..., object],
) -> tuple[int, int]:
    """读取已识别 X11 窗口的当前尺寸，用于 DPI/分辨率无关的控件定位。"""
    result = _run_xdotool(
        ("getwindowgeometry", "--shell", window_id),
        display=display,
        run_command=run_command,
    )
    if getattr(result, "returncode", 1) != 0:
        raise RuntimeError("Livox Viewer window geometry was unavailable")
    fields: dict[str, str] = {}
    for line in str(getattr(result, "stdout", "")).splitlines():
        key, separator, value = line.partition("=")
        if separator:
            fields[key] = value
    try:
        width, height = int(fields["WIDTH"]), int(fields["HEIGHT"])
    except (KeyError, ValueError) as exc:
        raise RuntimeError("Livox Viewer window geometry was malformed") from exc
    if width <= 0 or height <= 0:
        raise RuntimeError("Livox Viewer window geometry was invalid")
    return width, height


def _logical_window_point(
    point: tuple[int, int],
    *,
    width: int,
    height: int,
    logical_width: int = 1600,
    logical_height: int = 900,
) -> tuple[int, int]:
    """将已验证窗口内控件坐标映射到其实际尺寸。"""
    return (
        round(point[0] * width / logical_width),
        round(point[1] * height / logical_height),
    )


def _checked_xdotool(
    args: Sequence[str],
    *,
    display: str,
    run_command: Callable[..., object],
) -> None:
    result = _run_xdotool(args, display=display, run_command=run_command)
    if getattr(result, "returncode", 1) != 0:
        detail = str(getattr(result, "stderr", "")).strip()
        raise RuntimeError(f"Livox Viewer X11 automation failed: {detail or args[0]}")


def automate_lvx2_file_selection(
    *,
    launcher_pid: int,
    lvx2_path: Path,
    log_path: Path,
    display: str,
    timeout_sec: float,
    proc_root: Path = Path("/proc"),
    isolated_lookup: Callable[..., tuple[int, dict[str, object]] | None] | None = None,
    run_command: Callable[..., object] = subprocess.run,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
) -> None:
    """等待本次隔离 Viewer 就绪，再在其文件框中提交精确绝对路径。"""
    if timeout_sec <= 0:
        raise ValueError("timeout_sec must be positive")
    if isolated_lookup is None:
        isolated_lookup = _isolated_viewer_pid
    deadline = monotonic() + timeout_sec
    viewer_pid: int | None = None
    while monotonic() < deadline:
        isolated = isolated_lookup(
            launcher_pid=launcher_pid,
            host_pid=os.getpid(),
            proc_root=proc_root,
        )
        if isolated is not None and viewer_map_loaded(log_path):
            viewer_pid = isolated[0]
            break
        sleep(0.05)
    if viewer_pid is None:
        raise RuntimeError("Livox Viewer did not become ready for LVX2 import")

    main_window: str | None = None
    while monotonic() < deadline and main_window is None:
        main_window = _visible_window_id(
            viewer_pid=viewer_pid,
            title_pattern="^LivoxViewer",
            display=display,
            run_command=run_command,
        )
        if main_window is None:
            sleep(0.05)
    if main_window is None:
        raise RuntimeError("Livox Viewer main X11 window was not found")

    _checked_xdotool(
        ("windowactivate", "--sync", main_window),
        display=display,
        run_command=run_command,
    )
    main_width, main_height = _window_geometry(
        main_window, display=display, run_command=run_command
    )
    open_file_x, open_file_y = _logical_window_point(
        _VIEWER_OPEN_FILE_BUTTON, width=main_width, height=main_height
    )
    _checked_xdotool(
        ("mousemove", "--window", main_window, str(open_file_x), str(open_file_y), "click", "1"),
        display=display,
        run_command=run_command,
    )
    sleep(0.25)
    dialog_window: str | None = None
    while monotonic() < deadline and dialog_window is None:
        dialog_window = _visible_window_id(
            viewer_pid=viewer_pid,
            title_pattern="^Open File$",
            display=display,
            run_command=run_command,
        )
        if dialog_window is None:
            sleep(0.05)
    if dialog_window is None:
        raise RuntimeError("Livox Viewer Open File dialog was not found")
    # 原生文件框拥有独立 X11 身份；验证后只向它输入，不能把按键误送给主窗口。
    dialog_width, dialog_height = _window_geometry(
        dialog_window, display=display, run_command=run_command
    )
    filename_x, filename_y = _logical_window_point(
        _VIEWER_FILENAME_FIELD,
        width=dialog_width,
        height=dialog_height,
        logical_width=_VIEWER_DIALOG_REFERENCE_SIZE[0],
        logical_height=_VIEWER_DIALOG_REFERENCE_SIZE[1],
    )
    open_x, open_y = _logical_window_point(
        _VIEWER_OPEN_CONFIRM_BUTTON,
        width=dialog_width,
        height=dialog_height,
        logical_width=_VIEWER_DIALOG_REFERENCE_SIZE[0],
        logical_height=_VIEWER_DIALOG_REFERENCE_SIZE[1],
    )
    for args in (
        ("windowactivate", "--sync", dialog_window),
        ("mousemove", "--window", dialog_window, str(filename_x), str(filename_y), "click", "1"),
        ("key", "--window", dialog_window, "ctrl+a"),
        (
            "type",
            "--window",
            dialog_window,
            "--clearmodifiers",
            "--delay",
            "5",
            str(lvx2_path),
        ),
        ("mousemove", "--window", dialog_window, str(open_x), str(open_y), "click", "1"),
    ):
        _checked_xdotool(args, display=display, run_command=run_command)


def import_lvx2_in_livox_viewer(
    lvx2_path: Path,
    *,
    viewer_root: Path | None = None,
    display: str | None = None,
    xdg_root: Path | None = None,
    launch_dir: Path | None = None,
    timeout_sec: float = 90.0,
    launch: Callable[..., tuple[int, Path]] | None = None,
    automation: Callable[..., None] | None = None,
    sleep: Callable[[float], None] = time.sleep,
    monotonic: Callable[[], float] = time.monotonic,
    environ: Mapping[str, str] = os.environ,
) -> tuple[int, Path]:
    """隔离启动 Viewer，并仅在其日志确认指定 LVX2 已打开后返回。"""
    if not isinstance(lvx2_path, Path) or not lvx2_path.is_absolute():
        raise ValueError("lvx2_path must be an absolute Path")
    if lvx2_path.suffix.lower() != ".lvx2" or not lvx2_path.is_file():
        raise FileNotFoundError(f"validated LVX2 is unavailable: {lvx2_path}")
    if timeout_sec <= 0:
        raise ValueError("timeout_sec must be positive")
    _require_bwrap()
    if launch is None:
        launch = launch_isolated_viewer
    if automation is None:
        automation = automate_lvx2_file_selection

    resolved_viewer_root = viewer_root
    if resolved_viewer_root is None:
        override = environ.get(_VIEWER_ROOT_ENV, "")
        resolved_viewer_root = Path(override) if override else _DEFAULT_VIEWER_ROOT
    resolved_display = display if display is not None else environ.get("DISPLAY", "")
    if not resolved_display:
        raise RuntimeError("Livox Viewer import requires an X11 DISPLAY")

    if launch_dir is None:
        runtime_root = Path(tempfile.mkdtemp(prefix="slope-sim-livox-viewer-"))
        launch_dir = runtime_root / "launch"
        if xdg_root is None:
            xdg_root = runtime_root / "xdg"
    elif xdg_root is None:
        xdg_root = launch_dir.parent / f"{launch_dir.name}-xdg"

    launcher_pid, log_path = launch(
        viewer_root=resolved_viewer_root,
        display=resolved_display,
        xdg_root=xdg_root,
        launch_dir=launch_dir,
    )
    automation(
        launcher_pid=launcher_pid,
        lvx2_path=lvx2_path,
        log_path=log_path,
        display=resolved_display,
        timeout_sec=timeout_sec,
    )

    deadline = monotonic() + timeout_sec
    while monotonic() < deadline:
        if viewer_playback_evidence(log_path, lvx2_path).complete:
            return launcher_pid, log_path
        sleep(0.05)
    raise RuntimeError(
        f"Livox Viewer did not complete LVX2 playback for {lvx2_path}; inspect {log_path}"
    )


def _process_group(process_root: Path) -> int:
    """读取 Linux proc stat 中的进程组 ID。"""
    fields = (process_root / "stat").read_text(encoding="utf-8").rsplit(")", 1)[1].split()
    return int(fields[2])


def _isolated_viewer_pid(
    *,
    launcher_pid: int,
    host_pid: int,
    proc_root: Path,
) -> tuple[int, dict[str, object]] | None:
    """只在本次 launcher 进程组中寻找已进入 loopback-only namespace 的 Viewer。"""
    try:
        launcher_group = _process_group(proc_root / str(launcher_pid))
    except (OSError, ValueError, IndexError):
        return None
    for path in proc_root.iterdir():
        if not path.name.isdecimal():
            continue
        try:
            if _process_group(path) != launcher_group:
                continue
        except (OSError, ValueError, IndexError):
            continue
        pid = int(path.name)
        try:
            command_name = (proc_root / str(pid) / "comm").read_text(
                encoding="utf-8"
            ).strip()
            evidence = collect_network_evidence(
                host_pid=host_pid,
                viewer_pid=pid,
                proc_root=proc_root,
            )
        except OSError:
            continue
        if (
            command_name == "LivoxViewer2"
            and evidence["network_isolated"]
            and evidence["no_external_interfaces"]
        ):
            return pid, evidence
    return None


def launch_isolated_viewer(
    *,
    viewer_root: Path,
    display: str,
    xdg_root: Path,
    launch_dir: Path,
    viewer_args: Sequence[str] = (),
    popen_factory=subprocess.Popen,
) -> tuple[int, Path]:
    """后台启动隔离 Viewer；只报告进程已启动，不把它误报为文件已导入。"""
    if not isinstance(launch_dir, Path) or not launch_dir.is_absolute():
        raise ValueError("launch_dir must be an absolute Path")
    _require_bwrap()
    command, environment = isolated_viewer_command(
        viewer_root=viewer_root,
        display=display,
        xdg_root=xdg_root,
    )
    command.extend(viewer_args)
    launch_dir.mkdir(parents=True, exist_ok=False)
    log_path = launch_dir / "launcher.log"
    with log_path.open("x", encoding="utf-8") as log:
        process = popen_factory(
            command,
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    return process.pid, log_path


def run_isolated_viewer(
    *,
    viewer_root: Path,
    display: str,
    xdg_root: Path,
    evidence_dir: Path,
    viewer_args: Sequence[str] = (),
    startup_timeout_sec: float = 90.0,
    popen_factory=subprocess.Popen,
    proc_root: Path = Path("/proc"),
    sleep=time.sleep,
) -> int:
    """启动 Viewer，确认其网络隔离后再写入可复核的 smoke 证据。"""
    command, environment = isolated_viewer_command(
        viewer_root=viewer_root,
        display=display,
        xdg_root=xdg_root,
    )
    command.extend(viewer_args)
    evidence_dir.mkdir(parents=True, exist_ok=True)
    log_path = evidence_dir / "launcher.log"
    with log_path.open("x", encoding="utf-8") as log:
        process = popen_factory(
            command,
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        if startup_timeout_sec <= 0:
            raise ValueError("startup_timeout_sec must be positive")
        deadline = time.monotonic() + startup_timeout_sec
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("Livox Viewer exited before its network isolation was verified")
            isolated = _isolated_viewer_pid(
                launcher_pid=process.pid,
                host_pid=os.getpid(),
                proc_root=proc_root,
            )
            log.flush()
            if isolated is None or not viewer_map_loaded(log_path):
                sleep(0.05)
                continue
            viewer_pid, network = isolated
            launcher_pgid = _process_group(proc_root / str(process.pid))
            viewer_pgid = _process_group(proc_root / str(viewer_pid))
            write_network_evidence(
                evidence_dir,
                {
                    "bwrap_command": command,
                    "launcher_pid": process.pid,
                    "launcher_pgid": launcher_pgid,
                    "viewer_pid": viewer_pid,
                    "viewer_pgid": viewer_pgid,
                    "viewer_map_loaded": True,
                    **network,
                },
            )
            return process.wait()
    raise RuntimeError("Livox Viewer did not enter a loopback-only network namespace")


def isolated_viewer_command(
    *,
    viewer_root: Path,
    display: str,
    xdg_root: Path,
) -> tuple[list[str], dict[str, str]]:
    """构造只对 Viewer 断开 IP 网络的 bwrap 启动命令。"""
    launcher = viewer_root / "LivoxViewer2.sh"
    if not launcher.is_file():
        raise FileNotFoundError(f"Livox Viewer launcher is missing: {launcher}")
    if not display:
        raise ValueError("an X11 DISPLAY is required")

    resolved_xdg_root = xdg_root.resolve()
    environment = dict(os.environ)
    for name in _PROXY_VARIABLES:
        environment.pop(name, None)
    environment["DISPLAY"] = display
    environment["XDG_CONFIG_HOME"] = str(resolved_xdg_root / "config")
    environment["XDG_CACHE_HOME"] = str(resolved_xdg_root / "cache")
    environment["XDG_DATA_HOME"] = str(resolved_xdg_root / "data")

    command = ["bwrap", "--unshare-net", "--bind", "/", "/", "--proc", "/proc", "--dev-bind", "/dev", "/dev"]
    for name in _PROXY_VARIABLES:
        command.extend(("--unsetenv", name))
    for name in ("DISPLAY", "XDG_CONFIG_HOME", "XDG_CACHE_HOME", "XDG_DATA_HOME"):
        command.extend(("--setenv", name, environment[name]))
    # 全屏窗口会在本机 Intel/Mesa 的 X11 Vulkan 路径持续重建 swapchain；
    # 固定为小于可用桌面的窗口，保留点云显示所需的完整 GPU 渲染。
    command.extend((str(launcher.resolve()), *_VIEWER_WINDOW_ARGS))
    return command, environment


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Start an official Livox Viewer 2 Linux bundle in an isolated network namespace."
    )
    parser.add_argument("--viewer-root", type=Path, required=True)
    parser.add_argument("--xdg-root", type=Path, required=True)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--display", default=os.environ.get("DISPLAY", ""))
    parser.add_argument("--startup-timeout-sec", type=float, default=90.0)
    parser.add_argument("viewer_args", nargs=argparse.REMAINDER)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """运行官方程序，失败时保留其原始退出码。"""
    args = _parse_args(argv)
    _require_bwrap()
    return run_isolated_viewer(
        viewer_root=args.viewer_root,
        display=args.display,
        xdg_root=args.xdg_root,
        evidence_dir=args.evidence_dir,
        viewer_args=args.viewer_args,
        startup_timeout_sec=args.startup_timeout_sec,
    )


if __name__ == "__main__":
    raise SystemExit(main())
