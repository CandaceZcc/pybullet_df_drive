"""runSim v2：ROS Bridge/RViz2 实时显示进程编排合同。"""
from __future__ import annotations

from pathlib import Path
import signal
from types import SimpleNamespace


def test_ros_cmake_install_includes_the_fixed_live_rviz_profile() -> None:
    """ROS release 安装必须携带 Dashboard 固定使用的 RViz profile。"""
    root = Path(__file__).resolve().parents[2]
    cmake = (root / "cpp" / "phase0" / "CMakeLists.txt").read_text(encoding="utf-8")

    assert '"${CMAKE_CURRENT_LIST_DIR}/../client/rviz/stage4_live.rviz"' in cmake
    assert "DESTINATION cpp/client/rviz" in cmake


def test_ros_bridge_accepts_the_live_viewer_six_hour_deadline() -> None:
    """实时查看器的 6 小时上限必须落在 Bridge CLI 的有效范围内。"""
    root = Path(__file__).resolve().parents[2]
    bridge = (root / "cpp" / "client" / "stage4_ros2_bridge.cpp").read_text(encoding="utf-8")

    assert "value > 21600000" in bridge
    assert 'must be in [1, 21600000]' in bridge


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
    monkeypatch.setenv("LD_LIBRARY_PATH", "/system/lib")

    viewer = RunSimV2LiveViewer.launch(release_root=release)

    assert launches[0][0] == [
        "/bin/sh", "-c",
        '. /opt/ros/jazzy/setup.sh; '
        'export LD_LIBRARY_PATH="$1/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; '
        'exec "$2" --descriptor-set "$3" --duration-ms "$4" --deadline-ms "$4"',
        "bridge", str(release), str(bridge), str(descriptor), "21600000",
    ]
    assert launches[0][1]["start_new_session"] is True
    assert "env" not in launches[0][1]
    assert launches[1] == (
        [
            "/bin/sh", "-c",
            '. /opt/ros/jazzy/setup.sh; exec "$1" -d "$2"',
            "rviz2", "/usr/bin/rviz2", str(profile),
        ],
        {"start_new_session": True},
    )
    assert viewer.bridge_process.pid == 8101
    assert viewer.rviz_process.pid == 8102


def test_live_viewer_finds_installed_jazzy_rviz_without_sourced_path(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """CLI 未 source ROS 时仍应使用已安装的 Jazzy RViz2。"""
    from slope_sim.interfaces.v2 import runsim_v2_live_viewer as viewer_module

    release = tmp_path / "release"
    bridge = release / "bin" / "slope_sim_stage4_ros2_bridge"
    descriptor = release / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    profile = release / "cpp/client/rviz/stage4_live.rviz"
    for path in (bridge, descriptor, profile):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"release")
    bridge.chmod(0o755)
    jazzy_rviz = tmp_path / "jazzy" / "bin" / "rviz2"
    jazzy_rviz.parent.mkdir(parents=True)
    jazzy_rviz.write_bytes(b"rviz")
    jazzy_rviz.chmod(0o755)
    launches: list[list[str]] = []

    monkeypatch.setattr(viewer_module.shutil, "which", lambda _name: None)
    monkeypatch.setattr(viewer_module, "_JAZZY_RVIZ2", jazzy_rviz)
    monkeypatch.setattr(
        viewer_module.subprocess,
        "Popen",
        lambda argv, **_kwargs: launches.append(argv)
        or SimpleNamespace(pid=8101 + len(launches), poll=lambda: 0),
    )

    viewer_module.RunSimV2LiveViewer.launch(release_root=release)

    assert launches[1][-2:] == [str(jazzy_rviz), str(profile)]


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
