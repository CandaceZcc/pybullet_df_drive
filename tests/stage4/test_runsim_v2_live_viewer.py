"""runSim v2：ROS Bridge/RViz2 实时显示进程编排合同。"""
from __future__ import annotations

from pathlib import Path
import signal
from types import SimpleNamespace


def test_live_viewer_uses_release_bridge_profile_and_independent_process_groups(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """Dashboard 打开实时点云只能启动 release 内 Bridge 与固定 RViz profile。"""
    from slope_sim.interfaces.v2.runsim_v2_live_viewer import RunSimV2LiveViewer

    release = tmp_path / "release"
    bridge = release / "bin" / "slope_sim_stage4_ros2_bridge"
    descriptor = release / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    profile = release / "cpp/client/rviz/stage4_live.rviz"
    for path in (bridge, descriptor, profile):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"release")
    bridge.chmod(0o755)

    launches: list[tuple[list[str], dict[str, object]]] = []
    processes = [
        SimpleNamespace(pid=8101, poll=lambda: None),
        SimpleNamespace(pid=8102, poll=lambda: None),
    ]

    def popen(argv, **kwargs):
        launches.append((argv, kwargs))
        return processes.pop(0)

    monkeypatch.setattr(
        "slope_sim.interfaces.v2.runsim_v2_live_viewer.shutil.which",
        lambda name: "/usr/bin/rviz2" if name == "rviz2" else None,
    )
    monkeypatch.setattr(
        "slope_sim.interfaces.v2.runsim_v2_live_viewer.subprocess.Popen", popen,
    )

    viewer = RunSimV2LiveViewer.launch(release_root=release)

    assert launches == [
        (
            [
                str(bridge), "--descriptor-set", str(descriptor),
                "--duration-ms", "21600000", "--deadline-ms", "21600000",
            ],
            {"start_new_session": True},
        ),
        (
            ["/usr/bin/rviz2", "-d", str(profile)],
            {"start_new_session": True},
        ),
    ]
    assert viewer.bridge_process.pid == 8101
    assert viewer.rviz_process.pid == 8102


def test_live_viewer_close_terminates_only_its_bridge_and_rviz_groups(monkeypatch) -> None:
    """关闭实时显示不影响 v2 核心进程，必要时以 SIGKILL 有界回收。"""
    from slope_sim.interfaces.v2.runsim_v2_live_viewer import RunSimV2LiveViewer

    class Process:
        def __init__(self, pid: int) -> None:
            self.pid = pid
            self.wait_calls: list[float] = []

        def poll(self):
            return None

        def wait(self, *, timeout: float):
            self.wait_calls.append(timeout)
            return 0

    bridge, rviz = Process(8101), Process(8102)
    signals: list[tuple[int, signal.Signals]] = []
    monkeypatch.setattr(
        "slope_sim.interfaces.v2.runsim_v2_live_viewer.os.killpg",
        lambda pid, sent_signal: signals.append((pid, sent_signal)),
    )
    viewer = RunSimV2LiveViewer(bridge_process=bridge, rviz_process=rviz)

    viewer.close()

    assert signals == [(8102, signal.SIGTERM), (8101, signal.SIGTERM)]
    assert bridge.wait_calls == rviz.wait_calls == [1.0]
