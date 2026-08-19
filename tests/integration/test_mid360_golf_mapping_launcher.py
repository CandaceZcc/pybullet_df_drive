# MID-360 Golf 单入口集成门禁：用受控假进程验证屏障、结果核对和故障回收。
from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace
from uuid import UUID

import pytest

from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
from slope_sim.mapping_acceptance import MappingAcceptanceMetrics
from slope_sim.mapping_mcap import MappingSessionIdentity


SIMULATION_SESSION_HEX = "00112233445566778899aabbccddeeff"
SOURCE_SESSION_HEX = "ffeeddccbbaa99887766554433221100"
SCENE_ID = "mid360-golf-mapping-v1"
SOURCE_ID = "mid360.golf.command-peer"
PATTERN_VERSION = "livox-mid360-800000-v1"
PATTERN_SHA256 = (
    "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca"
)
EXPECTED_COUNTS = {
    "/sim/wheel/command": 20_820,
    "/sim/wheel/state": 20_820,
    "/sim/lidar/points": 2_082,
    "/sim/rtk/state": 2_083,
    "/sim/imu/attitude": 2_083,
}
SIMULATOR_COUNTS = {
    topic: count
    for topic, count in EXPECTED_COUNTS.items()
    if topic != "/sim/wheel/command"
}


def _option(argv: list[str], name: str) -> str:
    index = argv.index(name)
    return argv[index + 1]


def _write_json(path: Path, document: dict[str, object]) -> None:
    path.write_text(json.dumps(document, sort_keys=True) + "\n", encoding="utf-8")


def _ready_documents(descriptor_sha256: str) -> dict[str, dict[str, object]]:
    return {
        "simulator": {
            "role": "simulator",
            "ready": True,
            "simulation_session_id": SIMULATION_SESSION_HEX,
            "descriptor_sha256": descriptor_sha256,
            "world_generation": 1,
        },
        "command_peer": {
            "role": "command_peer",
            "ready": True,
            "source_id": SOURCE_ID,
            "source_session_id": SOURCE_SESSION_HEX,
            "descriptor_sha256": descriptor_sha256,
        },
    }


def _simulator_result(descriptor_sha256: str) -> dict[str, object]:
    return {
        "role": "simulator",
        "clean_shutdown": True,
        "fault_reason": None,
        "simulation_session_id": SIMULATION_SESSION_HEX,
        "descriptor_sha256": descriptor_sha256,
        "world_generation": 1,
        "command_generation": 1,
        "scene_id": SCENE_ID,
        "client_id": 0,
        "connection_mode": "direct",
        "physics_steps": 49_968,
        "sim_duration_ns": 208_200_000_000,
        "robot_model": "df_mid",
        "terrain_model": "golf_heightfield",
        "golf_seed": 41,
        "golf_relief": "medium",
        "static_obstacle_count": 6,
        "moving_obstacle_count": 3,
        "published_frames": SIMULATOR_COUNTS,
        "expected_topic_counts": EXPECTED_COUNTS,
        "active_command_steps": 45_000,
        "truth_acceptance": {
            "motion": {
                "eligible_frame_count": 1_900,
                "speed_above_0_1_m_s_frame_count": 1_820,
                "speed_above_0_1_m_s_ratio": 1_820 / 1_900,
            },
            "deskew": {
                "point_count": 20_000,
                "within_0_05_m_count": 20_000,
                "error_p95_upper_bound_m": 0.01,
            },
            "obstacles": [
                {
                    "logical_id": logical_id,
                    "mode": "static" if logical_id <= 6 else "moving",
                    "hit_frame_count": 1 if logical_id <= 6 else 10,
                    "position_bucket_count": 0 if logical_id <= 6 else 2,
                    "position_span_m": 0.0 if logical_id <= 6 else 0.1,
                }
                for logical_id in range(1, 10)
            ],
        },
        "transport_metrics": {
            "published_count": sum(SIMULATOR_COUNTS.values()),
            "received_count": EXPECTED_COUNTS["/sim/wheel/command"],
            "error_count": 0,
            "dropped_count": 0,
        },
    }


def _command_result(descriptor_sha256: str) -> dict[str, object]:
    return {
        "role": "command_peer",
        "clean_shutdown": True,
        "fault_reason": None,
        "source_id": SOURCE_ID,
        "source_session_id": SOURCE_SESSION_HEX,
        "descriptor_sha256": descriptor_sha256,
        "published_frames": {
            "/sim/wheel/command": EXPECTED_COUNTS["/sim/wheel/command"]
        },
        "last_wheel_timestamp_ns": 208_200_000_000,
        "latest_pose_timestamp_ns": 208_200_000_000,
        "normal_stop_started": True,
        "finished": True,
    }


class _FakeProcess:
    def __init__(
        self,
        *,
        role: str,
        argv: list[str],
        pid: int,
        factory: "_FakePopenFactory",
    ) -> None:
        self.role = role
        self.argv = argv
        self.pid = pid
        self.returncode: int | None = None
        self._factory = factory
        self._completed = False

    def poll(self) -> int | None:
        if self.returncode is not None:
            return self.returncode
        if not self._factory.start_path.is_file():
            return None
        if "start_seen" not in self._factory.events:
            self._factory.events.append("start_seen")
        if self.role == self._factory.fail_role:
            self.returncode = 7
            return self.returncode
        if self.role in self._factory.stall_roles:
            return None
        if not self._completed:
            self._completed = True
            self._factory.complete(self)
            self.returncode = 0
        return self.returncode

    def wait(self, timeout: float | None = None) -> int:
        del timeout
        if self.returncode is None:
            raise subprocess.TimeoutExpired(self.argv, timeout=0.0)
        return self.returncode


class _FakePopenFactory:
    def __init__(
        self,
        output_dir: Path,
        *,
        descriptor_sha256: str,
        fail_role: str | None = None,
        stall_roles: frozenset[str] = frozenset(),
    ) -> None:
        self.output_dir = output_dir
        self.start_path = output_dir / "start.json"
        self.descriptor_sha256 = descriptor_sha256
        self.fail_role = fail_role
        self.stall_roles = stall_roles
        self.events: list[str] = []
        self.processes: list[_FakeProcess] = []
        self.argv_by_role: dict[str, list[str]] = {}

    def __call__(self, argv: list[str], **kwargs: object) -> _FakeProcess:
        assert kwargs == {
            "cwd": str(Path(__file__).resolve().parents[2]),
            "start_new_session": True,
            "text": True,
        }
        command = list(argv)
        if "scripts.mid360_golf_simulation" in command:
            role = "simulator"
        elif "scripts.mid360_golf_command_peer" in command:
            role = "command_peer"
        else:
            role = "recorder"
        self.events.append(f"spawn:{role}")
        self.argv_by_role[role] = command
        process = _FakeProcess(
            role=role,
            argv=command,
            pid=50_000 + len(self.processes),
            factory=self,
        )
        self.processes.append(process)
        if role in {"simulator", "command_peer"}:
            assert not self.start_path.exists()
            ready_path = Path(_option(command, "--ready-path"))
            _write_json(
                ready_path,
                _ready_documents(self.descriptor_sha256)[role],
            )
            self.events.append(f"ready:{role}")
        return process

    def complete(self, process: _FakeProcess) -> None:
        if process.role == "recorder":
            mcap_path = Path(_option(process.argv, "--output"))
            mcap_path.write_bytes(b"strict-loader-fixture")
            _write_json(
                Path(_option(process.argv, "--result")),
                {
                    "clean_shutdown": True,
                    "mcap": str(mcap_path),
                    "recorded_count": sum(EXPECTED_COUNTS.values()),
                    "role": "recorder",
                    "topics": EXPECTED_COUNTS,
                },
            )
        elif process.role == "simulator":
            _write_json(
                Path(_option(process.argv, "--result-path")),
                _simulator_result(self.descriptor_sha256),
            )
        else:
            _write_json(
                Path(_option(process.argv, "--result-path")),
                _command_result(self.descriptor_sha256),
            )
        self.events.append(f"complete:{process.role}")


def _fake_index(descriptor_sha256: bytes) -> SimpleNamespace:
    return SimpleNamespace(
        identity=MappingSessionIdentity(
            bytes.fromhex(SIMULATION_SESSION_HEX),
            descriptor_sha256,
            1,
            SCENE_ID,
            PATTERN_VERSION,
            bytes.fromhex(PATTERN_SHA256),
        ),
        topic_counts=tuple(EXPECTED_COUNTS.items()),
        message_count=sum(EXPECTED_COUNTS.values()),
        lidar_frame_times_ns=tuple(range(0, 208_200_000_000, 100_000_000)),
        pose_nodes=tuple(
            SimpleNamespace(timestamp_ns=timestamp_ns)
            for timestamp_ns in range(0, 208_300_000_000, 100_000_000)
        ),
    )


def _new_fake_recorder(tmp_path: Path) -> Path:
    recorder = tmp_path / "slope_sim_stage4_recorder"
    recorder.write_text("#!/bin/sh\nexit 0\n", encoding="ascii")
    recorder.chmod(0o700)
    return recorder


def test_launcher_starts_recorder_first_and_opens_replay_only_after_strict_load(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from scripts import run_mid360_golf_mapping as launcher

    output_dir = tmp_path / "mapping-session"
    descriptor = load_v2_descriptor()
    processes = _FakePopenFactory(
        output_dir,
        descriptor_sha256=descriptor.sha256.hex(),
    )
    monkeypatch.setattr(launcher.subprocess, "Popen", processes)
    session_ids = iter(
        (UUID(hex=SIMULATION_SESSION_HEX), UUID(hex=SOURCE_SESSION_HEX))
    )
    monkeypatch.setattr(launcher, "uuid4", lambda: next(session_ids))
    opened: list[object] = []

    def strict_load(mcap_path: Path, recorder_result: Path) -> SimpleNamespace:
        assert all(process.poll() == 0 for process in processes.processes)
        assert mcap_path == output_dir / "session.mcap"
        assert recorder_result == output_dir / "recorder.json"
        processes.events.append("strict_load")
        return _fake_index(descriptor.sha256)

    def open_replay(index: object) -> None:
        processes.events.append("replay")
        opened.append(index)

    def evaluate(index: object, **kwargs: object) -> MappingAcceptanceMetrics:
        assert index is not None
        assert set(kwargs) == {"route", "bounds", "obstacles"}
        processes.events.append("acceptance")
        return MappingAcceptanceMetrics(
            route_sample_count=2_082,
            route_error_p95_m=0.20,
            route_final_remaining_m=0.20,
            terrain_eligible_cell_count=3_000,
            terrain_covered_cell_count=2_900,
            terrain_coverage_ratio=2_900 / 3_000,
            permanent_voxel_count=123_456,
            displayed_static_point_count=123_456,
        )

    monkeypatch.setattr(launcher, "load_mapping_session", strict_load)
    monkeypatch.setattr(launcher, "evaluate_mapping_session", evaluate)
    monkeypatch.setattr(launcher, "_open_mapping_replay", open_replay)

    result = launcher.run_mid360_golf_mapping(
        recorder=_new_fake_recorder(tmp_path),
        output_dir=output_dir,
        direct=True,
        open_replay=True,
    )

    assert result["clean_shutdown"] is True
    assert result["topic_counts"] == EXPECTED_COUNTS
    assert result["acceptance"] == str(output_dir / "acceptance.json")
    assert opened
    assert processes.events[:5] == [
        "spawn:recorder",
        "spawn:simulator",
        "ready:simulator",
        "spawn:command_peer",
        "ready:command_peer",
    ]
    assert processes.events.index("start_seen") > processes.events.index(
        "ready:command_peer"
    )
    assert processes.events[-3:] == ["strict_load", "acceptance", "replay"]
    acceptance = json.loads(
        (output_dir / "acceptance.json").read_text(encoding="utf-8")
    )
    assert acceptance["schema"] == "mid360-golf-mapping-acceptance-v2"
    assert acceptance["passed"] is True
    assert acceptance["failures"] == []
    assert acceptance["metrics"]["route"]["error_p95_m"] == pytest.approx(0.20)
    assert acceptance["metrics"]["route"]["final_remaining_m"] == pytest.approx(0.20)
    assert acceptance["metrics"]["terrain"]["coverage_ratio"] == pytest.approx(
        2_900 / 3_000
    )
    assert acceptance["metrics"]["world_map"] == {
        "permanent_voxel_count": 123_456,
        "displayed_static_point_count": 123_456,
    }
    assert json.loads((output_dir / "start.json").read_text(encoding="utf-8")) == {
        "start": True
    }
    assert not (output_dir / "fault.json").exists()

    recorder_argv = processes.argv_by_role["recorder"]
    assert _option(recorder_argv, "--simulation-session-id") == SIMULATION_SESSION_HEX
    assert _option(recorder_argv, "--scene-id") == SCENE_ID
    assert _option(recorder_argv, "--lidar-pattern-version") == PATTERN_VERSION
    assert _option(recorder_argv, "--lidar-pattern-sha256") == PATTERN_SHA256
    assert _option(recorder_argv, "--expected-wheel-command-count") == "20820"
    assert _option(recorder_argv, "--expected-wheel-state-count") == "20820"
    assert _option(recorder_argv, "--expected-lidar-points-count") == "2082"
    assert _option(recorder_argv, "--expected-rtk-state-count") == "2083"
    assert _option(recorder_argv, "--expected-imu-attitude-count") == "2083"
    assert _option(recorder_argv, "--deadline-ms") == "21600000"
    assert "--direct" in processes.argv_by_role["simulator"]
    assert (
        _option(processes.argv_by_role["simulator"], "--peer-timeout-sec")
        == "120.0"
    )
    assert (
        _option(processes.argv_by_role["command_peer"], "--source-session-id")
        == SOURCE_SESSION_HEX
    )
    assert (
        _option(processes.argv_by_role["command_peer"], "--peer-timeout-sec")
        == "120.0"
    )


def test_launcher_exports_the_accepted_mapping_to_lvx2_before_returning(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """新会话只有通过完整 MCAP 与 mapping acceptance 后才产生可导入的 LVX2。"""
    from scripts import run_mid360_golf_mapping as launcher

    output_dir = tmp_path / "mapping-export"
    descriptor = load_v2_descriptor()
    processes = _FakePopenFactory(
        output_dir,
        descriptor_sha256=descriptor.sha256.hex(),
    )
    exporter = tmp_path / "slope_sim_stage4_export"
    exporter.write_text("#!/bin/sh\nexit 0\n", encoding="ascii")
    exporter.chmod(0o700)
    monkeypatch.setattr(launcher.subprocess, "Popen", processes)
    session_ids = iter(
        (UUID(hex=SIMULATION_SESSION_HEX), UUID(hex=SOURCE_SESSION_HEX))
    )
    monkeypatch.setattr(launcher, "uuid4", lambda: next(session_ids))
    monkeypatch.setattr(
        launcher,
        "load_mapping_session",
        lambda *_args: _fake_index(descriptor.sha256),
    )
    monkeypatch.setattr(
        launcher,
        "evaluate_mapping_session",
        lambda *_args, **_kwargs: MappingAcceptanceMetrics(
            route_sample_count=2_082,
            route_error_p95_m=0.20,
            route_final_remaining_m=0.20,
            terrain_eligible_cell_count=3_000,
            terrain_covered_cell_count=2_900,
            terrain_coverage_ratio=2_900 / 3_000,
            permanent_voxel_count=123_456,
            displayed_static_point_count=123_456,
        ),
    )
    received: list[list[str]] = []

    def run_export(command, **kwargs):
        received.append(list(command))
        export_dir = output_dir / "export"
        export_dir.mkdir()
        (export_dir / "lidar.lvx2").write_bytes(b"LVX2")
        (output_dir / "export.json").write_text(
            '{"clean_shutdown":true,"role":"export","lidar_frames":1,'
            '"lvx2":"lidar.lvx2","lvx2_frames":1,"lvx2_packages":1,'
            '"lvx2_points":1}\n',
            encoding="utf-8",
        )
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(launcher.subprocess, "run", run_export)

    result = launcher.run_mid360_golf_mapping(
        recorder=_new_fake_recorder(tmp_path),
        exporter=exporter,
        output_dir=output_dir,
        direct=True,
        open_replay=False,
    )

    assert result["lvx2"] == str(output_dir / "export" / "lidar.lvx2")
    assert result["export"] == str(output_dir / "export.json")
    assert received == [[
        str(exporter),
        "--input", str(output_dir / "session.mcap"),
        "--descriptor-set", str(launcher._DESCRIPTOR_PATH),
        "--output-dir", str(output_dir / "export"),
        "--result", str(output_dir / "export.json"),
    ]]


def test_launcher_persists_failed_acceptance_and_does_not_open_replay(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from scripts import run_mid360_golf_mapping as launcher

    output_dir = tmp_path / "mapping-acceptance-failure"
    descriptor = load_v2_descriptor()
    processes = _FakePopenFactory(
        output_dir,
        descriptor_sha256=descriptor.sha256.hex(),
    )
    monkeypatch.setattr(launcher.subprocess, "Popen", processes)
    session_ids = iter(
        (UUID(hex=SIMULATION_SESSION_HEX), UUID(hex=SOURCE_SESSION_HEX))
    )
    monkeypatch.setattr(launcher, "uuid4", lambda: next(session_ids))
    monkeypatch.setattr(
        launcher,
        "load_mapping_session",
        lambda *_args: _fake_index(descriptor.sha256),
    )
    monkeypatch.setattr(
        launcher,
        "evaluate_mapping_session",
        lambda *_args, **_kwargs: MappingAcceptanceMetrics(
            route_sample_count=2_082,
            route_error_p95_m=0.36,
            route_final_remaining_m=0.20,
            terrain_eligible_cell_count=3_000,
            terrain_covered_cell_count=2_900,
            terrain_coverage_ratio=2_900 / 3_000,
            permanent_voxel_count=123_456,
            displayed_static_point_count=123_456,
        ),
    )
    monkeypatch.setattr(
        launcher,
        "_open_mapping_replay",
        lambda *_args: pytest.fail("failed acceptance must not open replay"),
    )

    with pytest.raises(RuntimeError, match="route_error_p95"):
        launcher.run_mid360_golf_mapping(
            recorder=_new_fake_recorder(tmp_path),
            output_dir=output_dir,
            direct=True,
            open_replay=True,
        )

    acceptance = json.loads(
        (output_dir / "acceptance.json").read_text(encoding="utf-8")
    )
    assert acceptance["passed"] is False
    assert "route_error_p95" in acceptance["failures"]
    assert not any(event == "replay" for event in processes.events)
    assert (output_dir / "fault.json").is_file()


def test_launcher_runs_automated_gui_qa_against_the_accepted_session(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """最终 GUI QA 必须消费刚刚验收的同一 MCAP，而非历史 session。"""
    from scripts import run_mid360_golf_mapping as launcher

    output_dir = tmp_path / "mapping-gui-qa"
    descriptor = load_v2_descriptor()
    processes = _FakePopenFactory(
        output_dir,
        descriptor_sha256=descriptor.sha256.hex(),
    )
    monkeypatch.setattr(launcher.subprocess, "Popen", processes)
    session_ids = iter(
        (UUID(hex=SIMULATION_SESSION_HEX), UUID(hex=SOURCE_SESSION_HEX))
    )
    monkeypatch.setattr(launcher, "uuid4", lambda: next(session_ids))
    index = _fake_index(descriptor.sha256)
    monkeypatch.setattr(launcher, "load_mapping_session", lambda *_args: index)
    monkeypatch.setattr(
        launcher,
        "evaluate_mapping_session",
        lambda *_args, **_kwargs: MappingAcceptanceMetrics(
            route_sample_count=2_082,
            route_error_p95_m=0.20,
            route_final_remaining_m=0.20,
            terrain_eligible_cell_count=3_000,
            terrain_covered_cell_count=2_900,
            terrain_coverage_ratio=2_900 / 3_000,
            permanent_voxel_count=123_456,
            displayed_static_point_count=123_456,
        ),
    )
    received: list[dict[str, object]] = []

    def run_qa(**kwargs: object) -> dict[str, object]:
        received.append(dict(kwargs))
        return {"passed": True, "qa": str(output_dir / "gui-qa" / "qa.json")}

    monkeypatch.setattr(launcher, "_run_mapping_replay_qa", run_qa)
    monkeypatch.setattr(
        launcher,
        "_open_mapping_replay",
        lambda *_args: pytest.fail("automated QA owns the replay window"),
    )

    result = launcher.run_mid360_golf_mapping(
        recorder=_new_fake_recorder(tmp_path),
        output_dir=output_dir,
        direct=True,
        open_replay=True,
        replay_qa=True,
    )

    assert result["replay_qa"] == str(output_dir / "gui-qa" / "qa.json")
    assert received == [
        {
            "index": index,
            "output_dir": output_dir / "gui-qa",
            "simulation_session_hex": SIMULATION_SESSION_HEX,
        }
    ]
    assert json.loads((output_dir / "acceptance.json").read_text(encoding="utf-8"))["simulation_session_id"] == SIMULATION_SESSION_HEX


def test_launcher_faults_and_reaps_process_groups_without_opening_replay(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from scripts import run_mid360_golf_mapping as launcher

    output_dir = tmp_path / "mapping-failure"
    descriptor = load_v2_descriptor()
    processes = _FakePopenFactory(
        output_dir,
        descriptor_sha256=descriptor.sha256.hex(),
        fail_role="simulator",
        stall_roles=frozenset({"recorder", "command_peer"}),
    )
    monkeypatch.setattr(launcher.subprocess, "Popen", processes)
    session_ids = iter(
        (UUID(hex=SIMULATION_SESSION_HEX), UUID(hex=SOURCE_SESSION_HEX))
    )
    monkeypatch.setattr(launcher, "uuid4", lambda: next(session_ids))
    monkeypatch.setattr(launcher, "_FAULT_GRACE_SEC", 0.0)
    killed: list[tuple[int, signal.Signals]] = []

    def kill_process_group(pid: int, sent_signal: signal.Signals) -> None:
        killed.append((pid, sent_signal))
        process = next(item for item in processes.processes if item.pid == pid)
        process.returncode = -int(sent_signal)

    monkeypatch.setattr(launcher.os, "killpg", kill_process_group)
    monkeypatch.setattr(
        launcher,
        "load_mapping_session",
        lambda *_args: pytest.fail("failed sessions must not reach strict MCAP loading"),
    )
    monkeypatch.setattr(
        launcher,
        "_open_mapping_replay",
        lambda *_args: pytest.fail("failed sessions must never open replay"),
    )

    with pytest.raises(RuntimeError, match="simulator exited with code 7"):
        launcher.run_mid360_golf_mapping(
            recorder=_new_fake_recorder(tmp_path),
            output_dir=output_dir,
            direct=True,
            open_replay=True,
        )

    fault = json.loads((output_dir / "fault.json").read_text(encoding="utf-8"))
    assert set(fault) == {"fault"}
    assert isinstance(fault["fault"], str) and fault["fault"]
    killed_pids = {pid for pid, sent_signal in killed if sent_signal == signal.SIGTERM}
    assert killed_pids == {
        process.pid
        for process in processes.processes
        if process.role in {"recorder", "command_peer"}
    }


def test_launcher_rejects_an_existing_output_directory_before_spawning(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from scripts import run_mid360_golf_mapping as launcher

    output_dir = tmp_path / "existing"
    output_dir.mkdir()
    monkeypatch.setattr(
        launcher.subprocess,
        "Popen",
        lambda *_args, **_kwargs: pytest.fail("preflight failure must not spawn"),
    )

    with pytest.raises(ValueError, match="new absolute directory"):
        launcher.run_mid360_golf_mapping(
            recorder=_new_fake_recorder(tmp_path),
            output_dir=output_dir,
            direct=True,
            open_replay=False,
        )


def test_launcher_cli_exposes_recorder_output_direct_and_no_replay(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from scripts import run_mid360_golf_mapping as launcher

    recorder = _new_fake_recorder(tmp_path)
    exporter = tmp_path / "slope_sim_stage4_export"
    exporter.write_text("#!/bin/sh\nexit 0\n", encoding="ascii")
    exporter.chmod(0o700)
    output_dir = tmp_path / "cli-output"
    received: list[dict[str, object]] = []

    def run(**kwargs: object) -> dict[str, object]:
        received.append(dict(kwargs))
        return {"clean_shutdown": True}

    monkeypatch.setattr(launcher, "run_mid360_golf_mapping", run)

    assert launcher.main(
        [
            "--recorder",
            str(recorder),
            "--exporter",
            str(exporter),
            "--output-dir",
            str(output_dir),
            "--direct",
            "--no-replay",
        ]
    ) == 0
    assert received == [
        {
                "recorder": recorder,
                "exporter": exporter,
            "output_dir": output_dir,
            "direct": True,
            "open_replay": False,
            "replay_qa": False,
        }
    ]


def test_completion_wait_is_condition_based_for_slower_than_realtime_capture(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """离线 raycast 可慢于实时，运行阶段不能用固定墙钟窗口误杀。"""
    from scripts import run_mid360_golf_mapping as launcher

    class Process:
        def __init__(self) -> None:
            self.poll_count = 0

        def poll(self) -> int | None:
            self.poll_count += 1
            return 0 if self.poll_count >= 3 else None

    process = Process()
    monkeypatch.setattr(
        launcher.time,
        "monotonic",
        lambda: pytest.fail("completion waiting must not depend on a wall deadline"),
    )
    monkeypatch.setattr(launcher.time, "sleep", lambda _seconds: None)

    launcher._wait_for_completion(
        {"simulator": process},
        tmp_path / "fault.json",
    )

    assert process.poll_count == 3
