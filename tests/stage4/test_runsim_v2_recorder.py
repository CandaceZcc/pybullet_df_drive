"""runSim v2：C++ Recorder interactive 编排合同。"""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from datetime import datetime
import json
import signal
import subprocess


def test_recorder_builder_freezes_current_v2_identity_and_private_control_socket(
    tmp_path: Path,
) -> None:
    """Recorder 必须订阅当前 session/world，且控制 socket 不得位于共享目录。"""
    from slope_sim.interfaces.v2.runsim_v2_recorder import build_interactive_recorder_argv

    control_dir = tmp_path / "private"
    control_dir.mkdir(mode=0o700)
    snapshot = SimpleNamespace(simulation_session_id=b"s" * 16, world_generation=4)
    argv = build_interactive_recorder_argv(
        executable=tmp_path / "slope_sim_stage4_recorder",
        descriptor_set=tmp_path / "slope_sim_interfaces_v2.desc",
        snapshot=snapshot,
        scene_id="capture-20260819-143012",
        output_dir=tmp_path,
        control_socket=control_dir / "recorder.sock",
        control_token=b"t" * 32,
    )

    assert argv[:3] == [str(tmp_path / "slope_sim_stage4_recorder"), "--interactive", "--descriptor-set"]
    assert argv[argv.index("--simulation-session-id") + 1] == (b"s" * 16).hex()
    assert argv[argv.index("--world-generation") + 1] == "4"
    assert argv[argv.index("--control-socket") + 1] == str(control_dir / "recorder.sock")
    assert argv[argv.index("--control-token") + 1] == (b"t" * 32).hex()
    assert Path(argv[argv.index("--output") + 1]).name == "session.mcap"


def test_export_builder_uses_recorder_mcap_and_new_artifact_paths(tmp_path: Path) -> None:
    """成功的 C++ Recorder 只能交给同一 release 的 C++ Export。"""
    from slope_sim.interfaces.v2.runsim_v2_recorder import build_export_argv

    mcap = tmp_path / "session.mcap"
    descriptor = tmp_path / "slope_sim_interfaces_v2.desc"
    mcap.write_bytes(b"mcap")
    descriptor.write_bytes(b"descriptor")
    argv = build_export_argv(
        executable=tmp_path / "slope_sim_stage4_export",
        descriptor_set=descriptor,
        mcap_path=mcap,
        output_dir=tmp_path,
    )

    assert argv[:3] == [str(tmp_path / "slope_sim_stage4_export"), "--input", str(mcap)]
    assert Path(argv[argv.index("--output-dir") + 1]).name == "export"
    assert Path(argv[argv.index("--result") + 1]).name == "export.result.json"


def test_capture_directory_uses_local_datetime_and_deterministic_collision_suffix(tmp_path: Path) -> None:
    """同秒 capture 不覆盖既有证据，目录名仍保留本地时间。"""
    from slope_sim.interfaces.v2.runsim_v2_recorder import create_capture_output_dir

    now = datetime(2026, 8, 19, 14, 30, 12)
    first = create_capture_output_dir(tmp_path, now=now)
    second = create_capture_output_dir(tmp_path, now=now)

    assert first.name == "capture-20260819-143012"
    assert second.name == "capture-20260819-143012-1"


def test_latest_successful_lvx2_path_is_persisted_as_an_absolute_path(tmp_path: Path) -> None:
    """Dashboard 重启后仍可定位最近一次成功的 Viewer 输入。"""
    from slope_sim.interfaces.v2.runsim_v2_recorder import write_latest_successful_lvx2_path

    lvx2 = tmp_path / "capture" / "export" / "lidar.lvx2"
    lvx2.parent.mkdir(parents=True)
    lvx2.write_bytes(b"lvx2")

    manifest = write_latest_successful_lvx2_path(tmp_path, lvx2)

    assert manifest == tmp_path / "last-successful-lvx2.json"
    assert json.loads(manifest.read_text(encoding="utf-8"))["lvx2_path"] == str(lvx2)

    from slope_sim.interfaces.v2.runsim_v2_recorder import load_latest_successful_lvx2_path

    assert load_latest_successful_lvx2_path(tmp_path) == lvx2


def test_recorder_close_kills_an_unresponsive_process_group(tmp_path: Path, monkeypatch) -> None:
    """异常退出不能因 Recorder 忽略 SIGTERM 而留下孤儿进程。"""
    from slope_sim.interfaces.v2.runsim_v2_recorder import RunSimV2Recorder

    class StubbornProcess:
        pid = 4815

        def __init__(self) -> None:
            self.wait_timeouts: list[float] = []

        def poll(self):
            return None

        def wait(self, *, timeout: float):
            self.wait_timeouts.append(timeout)
            if len(self.wait_timeouts) < 3:
                raise subprocess.TimeoutExpired("recorder", timeout)
            return -signal.SIGKILL

    process = StubbornProcess()
    control_dir = tmp_path / "private"
    control_dir.mkdir(mode=0o700)
    recorder = RunSimV2Recorder(
        process=process,
        control_socket=control_dir / "recorder.sock",
        control_token=b"t" * 32,
        control_dir=control_dir,
        output_dir=tmp_path,
    )
    signals: list[tuple[int, signal.Signals]] = []
    monkeypatch.setattr(recorder, "stop", lambda: None)
    monkeypatch.setattr(
        "slope_sim.interfaces.v2.runsim_v2_recorder.os.killpg",
        lambda pid, sent_signal: signals.append((pid, sent_signal)),
    )

    recorder.close()

    assert signals == [(process.pid, signal.SIGTERM), (process.pid, signal.SIGKILL)]
    assert process.wait_timeouts == [2.0, 1.0, 1.0]
    assert not control_dir.exists()
