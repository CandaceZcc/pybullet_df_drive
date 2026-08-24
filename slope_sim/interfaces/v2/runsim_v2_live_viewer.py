"""runSim v2：独立管理 ROS Bridge 与 RViz2 的实时点云显示。"""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import signal
import subprocess
from collections.abc import Callable


_LIVE_DURATION_MS = "21600000"
_JAZZY_RVIZ2 = Path("/opt/ros/jazzy/bin/rviz2")
_JAZZY_SETUP = Path("/opt/ros/jazzy/setup.sh")


def _stop_process_group(process: subprocess.Popen[object]) -> None:
    """只回收本显示链创建的进程组，核心 v2 会话完全不在此范围内。"""
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
            # 已发 SIGKILL；Dashboard 关闭仍须返回，使用户可重试打开。
            pass


class RunSimV2LiveViewer:
    """一份可关闭的 release 内 ROS Bridge/RViz2 显示会话。"""

    def __init__(
        self,
        *,
        bridge_process: subprocess.Popen[object],
        rviz_process: subprocess.Popen[object],
    ) -> None:
        self.bridge_process = bridge_process
        self.rviz_process = rviz_process

    @classmethod
    def launch(cls, *, release_root: Path) -> "RunSimV2LiveViewer":
        """从同一 release 启动有限期 Bridge 与 RViz2，不使用开发树 fallback。"""
        if not isinstance(release_root, Path) or not release_root.is_absolute():
            raise ValueError("release_root must be an absolute Path")
        bridge = release_root / "bin" / "slope_sim_stage4_ros2_bridge"
        descriptor = release_root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
        profile = release_root / "cpp/client/rviz/stage4_live.rviz"
        if not bridge.is_file() or not bridge.stat().st_mode & 0o111:
            raise RuntimeError(f"runSim ROS Bridge is unavailable: {bridge}")
        if not descriptor.is_file():
            raise RuntimeError(f"runSim v2 descriptor is unavailable: {descriptor}")
        if not profile.is_file():
            raise RuntimeError(f"runSim RViz profile is unavailable: {profile}")
        rviz = shutil.which("rviz2")
        if rviz is None and _JAZZY_RVIZ2.is_file() and os.access(_JAZZY_RVIZ2, os.X_OK):
            rviz = str(_JAZZY_RVIZ2)
        if rviz is None:
            raise RuntimeError("RViz2 is unavailable; install the release ROS/RViz component or source ROS 2 Jazzy")
        bridge_process = subprocess.Popen(
            [
                "/bin/sh", "-c",
                '. /opt/ros/jazzy/setup.sh; '
                'export LD_LIBRARY_PATH="$1/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; '
                'exec "$2" --descriptor-set "$3" --duration-ms "$4" --deadline-ms "$4"',
                "bridge", str(release_root), str(bridge), str(descriptor), _LIVE_DURATION_MS,
            ],
            start_new_session=True,
        )
        try:
            rviz_process = subprocess.Popen(
                [
                    "/bin/sh", "-c",
                    '. /opt/ros/jazzy/setup.sh; exec "$1" -d "$2"',
                    "rviz2", rviz, str(profile),
                ],
                start_new_session=True,
            )
        except BaseException:
            _stop_process_group(bridge_process)
            raise
        return cls(bridge_process=bridge_process, rviz_process=rviz_process)

    def close(self) -> None:
        """先关闭 RViz2，再关闭只属于显示链的 Bridge；可重复调用。"""
        _stop_process_group(self.rviz_process)
        _stop_process_group(self.bridge_process)


def build_live_viewer_launcher(release_root: Path) -> Callable[[], Callable[[], None]]:
    """适配 Dashboard 的零参数 launcher：成功后仅暴露对应 close 句柄。"""
    if not isinstance(release_root, Path) or not release_root.is_absolute():
        raise ValueError("release_root must be an absolute Path")

    def launch() -> Callable[[], None]:
        viewer = RunSimV2LiveViewer.launch(release_root=release_root)
        return viewer.close

    return launch
