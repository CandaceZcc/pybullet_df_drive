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
    assert second.name == "capture-20260819-143012-01"


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


def test_capture_outputs_are_published_only_after_all_artifacts_are_complete(
    tmp_path: Path,
) -> None:
    """临时采集失败不能伪装成最近成功导出，完整输出才可原子发布。"""
    from slope_sim.interfaces.v2.runsim_v2_recorder import (
        prepare_capture_output_dirs,
        publish_capture_output,
    )

    paths = prepare_capture_output_dirs(tmp_path, now=datetime(2026, 8, 19, 14, 30, 12))
    assert paths.staging_dir.is_dir()
    assert not paths.published_dir.exists()

    staging_mcap = paths.staging_dir / "session.mcap"
    staging_mcap.write_bytes(b"mcap")
    (paths.staging_dir / "recorder.result.json").write_text(
        json.dumps(
            {
                "clean_shutdown": True,
                "exportable": True,
                "mcap": str(staging_mcap),
                "recorded_count": 5,
                "role": "recorder",
                "topics": {},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    export = paths.staging_dir / "export"
    export.mkdir()
    (export / "lidar.lvx2").write_bytes(b"lvx2")
    (export / "frame-0000.pcd").write_text("pcd\n", encoding="utf-8")
    (export / "frame-0000.ply").write_text("ply\n", encoding="utf-8")
    (paths.staging_dir / "export.result.json").write_text("{}\n", encoding="utf-8")

    published = publish_capture_output(paths)

    assert published == paths.published_dir
    assert published.is_dir()
    assert not paths.staging_dir.exists()
    assert json.loads((published / "recorder.result.json").read_text(encoding="utf-8"))["mcap"] == str(
        published / "session.mcap"
    )
    manifest = json.loads((published / "capture.manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "complete"
    assert manifest["artifacts"] == [
        "export/frame-0000.pcd",
        "export/frame-0000.ply",
        "export/lidar.lvx2",
        "export.result.json",
        "recorder.result.json",
        "session.mcap",
    ]


def test_recorder_export_publishes_staged_capture_before_updating_latest_marker(
    tmp_path: Path, monkeypatch
) -> None:
    """正式导出必须先改名最终目录，再把它作为 Viewer 的最近成功输入。"""
    from slope_sim.interfaces.v2.runsim_v2_recorder import (
        RunSimV2Recorder,
        prepare_capture_output_dirs,
        load_latest_successful_lvx2_path,
    )

    paths = prepare_capture_output_dirs(tmp_path, now=datetime(2026, 8, 19, 14, 30, 12))
    mcap = paths.staging_dir / "session.mcap"
    mcap.write_bytes(b"mcap")
    (paths.staging_dir / "recorder.result.json").write_text(
        json.dumps(
            {
                "clean_shutdown": True,
                "exportable": True,
                "mcap": str(mcap),
                "recorded_count": 1,
                "role": "recorder",
                "topics": {},
            }
        )
        + "\n",
        encoding="utf-8",
    )
    release = tmp_path / "release"
    executable = release / "bin" / "slope_sim_stage4_export"
    executable.parent.mkdir(parents=True)
    executable.write_text("#!/bin/sh\n", encoding="utf-8")
    executable.chmod(0o755)
    descriptor = release / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    descriptor.parent.mkdir(parents=True)
    descriptor.write_bytes(b"descriptor")
    control_dir = tmp_path / "control"
    control_dir.mkdir()
    recorder = RunSimV2Recorder(
        process=object(),
        control_socket=control_dir / "recorder.sock",
        control_token=b"t" * 32,
        control_dir=control_dir,
        output_dir=paths.staging_dir,
        published_output_dir=paths.published_dir,
    )

    def fake_run(argv, *, check):
        assert check is True
        output = Path(argv[argv.index("--output-dir") + 1])
        output.mkdir()
        (output / "lidar.lvx2").write_bytes(b"lvx2")
        (output / "frame-0000.pcd").write_text("pcd\n", encoding="utf-8")
        (output / "frame-0000.ply").write_text("ply\n", encoding="utf-8")
        Path(argv[argv.index("--result") + 1]).write_text("{}\n", encoding="utf-8")

    monkeypatch.setattr(
        "slope_sim.interfaces.v2.runsim_v2_recorder.subprocess.run", fake_run
    )

    lvx2, result = recorder.export(release_root=release, mcap_path=mcap)

    assert lvx2 == paths.published_dir / "export/lidar.lvx2"
    assert result == paths.published_dir / "export.result.json"
    assert load_latest_successful_lvx2_path(tmp_path) == lvx2
    assert not paths.staging_dir.exists()


def test_failed_staged_capture_is_marked_incomplete_without_publishing(tmp_path: Path) -> None:
    """失败采集保留诊断，但不占用最终 capture 目录或最近成功标记。"""
    from slope_sim.interfaces.v2.runsim_v2_recorder import (
        mark_capture_output_incomplete,
        prepare_capture_output_dirs,
    )

    paths = prepare_capture_output_dirs(tmp_path, now=datetime(2026, 8, 19, 14, 30, 12))
    marker = mark_capture_output_incomplete(paths.staging_dir, "export exited with status 1")

    assert marker == paths.staging_dir / "capture.incomplete.json"
    assert json.loads(marker.read_text(encoding="utf-8")) == {
        "reason": "export exited with status 1",
        "status": "incomplete",
    }
    assert not paths.published_dir.exists()


def test_recorder_export_marks_staging_incomplete_when_exporter_fails(
    tmp_path: Path, monkeypatch
) -> None:
    """C++ Export 失败后，Dashboard 可追查暂存原因而非误报成功。"""
    import pytest

    from slope_sim.interfaces.v2.runsim_v2_recorder import (
        RunSimV2Recorder,
        prepare_capture_output_dirs,
    )

    paths = prepare_capture_output_dirs(tmp_path, now=datetime(2026, 8, 19, 14, 30, 12))
    mcap = paths.staging_dir / "session.mcap"
    mcap.write_bytes(b"mcap")
    release = tmp_path / "release"
    executable = release / "bin" / "slope_sim_stage4_export"
    executable.parent.mkdir(parents=True)
    executable.write_text("#!/bin/sh\n", encoding="utf-8")
    executable.chmod(0o755)
    descriptor = release / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    descriptor.parent.mkdir(parents=True)
    descriptor.write_bytes(b"descriptor")
    control_dir = tmp_path / "control"
    control_dir.mkdir()
    recorder = RunSimV2Recorder(
        process=object(),
        control_socket=control_dir / "recorder.sock",
        control_token=b"t" * 32,
        control_dir=control_dir,
        output_dir=paths.staging_dir,
        published_output_dir=paths.published_dir,
    )
    failure = subprocess.CalledProcessError(1, ["stage4-export"])
    monkeypatch.setattr(
        "slope_sim.interfaces.v2.runsim_v2_recorder.subprocess.run",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(failure),
    )

    with pytest.raises(subprocess.CalledProcessError):
        recorder.export(release_root=release, mcap_path=mcap)

    marker = paths.staging_dir / "capture.incomplete.json"
    assert json.loads(marker.read_text(encoding="utf-8"))["status"] == "incomplete"
    assert not paths.published_dir.exists()


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


def test_recorder_stop_preserves_the_first_child_failure(tmp_path: Path) -> None:
    """子进程先失败时，stop 不得用泛化的“不再运行”覆盖原始故障。"""
    import pytest

    from slope_sim.interfaces.v2.runsim_v2_recorder import RunSimV2Recorder

    class FailedProcess:
        @staticmethod
        def poll() -> int:
            return 1

    control_dir = tmp_path / "private"
    control_dir.mkdir(mode=0o700)
    (tmp_path / "recorder.result.json").write_text(
        json.dumps({"clean_shutdown": False, "fault_reason": "WheelState sequence is not continuous"}),
        encoding="utf-8",
    )
    recorder = RunSimV2Recorder(
        process=FailedProcess(),
        control_socket=control_dir / "recorder.sock",
        control_token=b"t" * 32,
        control_dir=control_dir,
        output_dir=tmp_path,
    )

    with pytest.raises(RuntimeError, match="WheelState sequence is not continuous"):
        recorder.stop()
