# 阶段四 C2：验证 Recorder 故障由单机 Supervisor 停止唯一 Command 进程。
from __future__ import annotations

import json
import multiprocessing
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb


def _required_executable(environment_name: str) -> Path:
    """从显式环境变量取得当前已验证工具，避免测试重配历史构建根。"""
    executable = Path(os.environ[environment_name])
    assert executable.is_absolute() and executable.is_file()
    assert os.access(executable, os.X_OK)
    return executable


def _run_five_second_scene_runtime(
    result_json: str,
    scene: str,
    simulation_session_id_hex: str,
    ready_file: str,
    start_file: str,
) -> None:
    """供 spawn 子进程执行固定会话，避免测试进程先初始化 eCAL。"""
    from scripts.stage4_v2_simulation_runtime import run_v2_simulation_runtime

    run_v2_simulation_runtime(
        result_json=Path(result_json),
        duration_sec=5.0,
        scene=Path(scene),
        require_verified_peers=True,
        peer_timeout_sec=15.0,
        ready_file=Path(ready_file),
        start_file=Path(start_file),
        session_id_factory=lambda: bytes.fromhex(simulation_session_id_hex),
    )


def test_supervisor_stops_command_when_recorder_requires_safe_stop(tmp_path: Path) -> None:
    """Recorder 的排他故障结果必须停止 Command，不能继续向 runtime 发命令。"""
    from scripts.stage4_c2_supervisor import supervise_recorder_and_command

    recorder_result = tmp_path / "recorder.json"
    supervisor_result = tmp_path / "supervisor.json"
    recorder = [
        sys.executable,
        "-c",
        "import json, pathlib, time; time.sleep(0.1); "
        + f"pathlib.Path({str(recorder_result)!r}).write_text("
        + "json.dumps({'clean_shutdown': False, 'safe_stop_required': True}), encoding='utf-8')",
    ]
    command = [sys.executable, "-c", "import time; time.sleep(30)"]

    result = supervise_recorder_and_command(
        command_argv=command,
        recorder_argv=recorder,
        recorder_result=recorder_result,
        supervisor_result=supervisor_result,
        timeout_sec=3.0,
    )

    assert result == {
        "command_stopped": True,
        "command_returncode": -15,
        "clean_shutdown": False,
        "recorder_safe_stop_required": True,
        "role": "supervisor",
    }
    assert json.loads(supervisor_result.read_text(encoding="utf-8")) == result


def test_supervisor_waits_for_recorder_result_to_become_complete_json(tmp_path: Path) -> None:
    """轮询到已创建但尚未写完的 Recorder result 时不能误判为故障。"""
    from scripts.stage4_c2_supervisor import supervise_recorder_and_command

    recorder_result = tmp_path / "recorder.json"
    supervisor_result = tmp_path / "supervisor.json"
    recorder = [
        sys.executable,
        "-c",
        "import pathlib, time; path = pathlib.Path(" + repr(str(recorder_result)) + "); "
        "stream = path.open('w', encoding='utf-8'); stream.write('{'); stream.flush(); "
        "time.sleep(0.1); stream.write(chr(34) + 'safe_stop_required' + chr(34) + ': true}'); "
        "stream.close()",
    ]

    result = supervise_recorder_and_command(
        command_argv=[sys.executable, "-c", "import time; time.sleep(30)"],
        recorder_argv=recorder,
        recorder_result=recorder_result,
        supervisor_result=supervisor_result,
        timeout_sec=3.0,
    )

    assert result["command_stopped"] is True
    assert result["recorder_safe_stop_required"] is True


@pytest.mark.stage4_artifact
def test_supervisor_stops_real_cpp_command_after_real_recorder_fault(tmp_path: Path) -> None:
    """真实 eCAL Recorder 故障后，Supervisor 必须终止仍在运行的 C++ Command。"""
    from scripts.stage4_c2_supervisor import supervise_recorder_and_command

    build_directory = Path(os.environ["STAGE4_PHASE0_BUILD_DIR"])
    for target in ("slope_sim_stage4_command", "slope_sim_stage4_recorder"):
        subprocess.run(["cmake", "--build", str(build_directory), "--target", target], check=True)
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    payload = root / "tests/fixtures/stage4/v2/WheelCommand.bin"
    recorder_result = tmp_path / "recorder.json"
    supervisor_result = tmp_path / "supervisor.json"
    recorder = [
        str(build_directory / "slope_sim_stage4_recorder"),
        "--descriptor-set", str(descriptor),
        "--scene-id", "supervisor-real-ecal",
        "--simulation-session-id", "00112233445566778899aabbccddeeff",
        "--world-generation", "7",
        "--lidar-pattern-version", "livox-mid360-800000-v1",
        "--lidar-pattern-sha256", "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca",
        "--output", str(tmp_path / "partial.mcap"),
        "--expected-count", "1",
        "--deadline-ms", "10000",
        "--result", str(recorder_result),
    ]
    command = [
        str(build_directory / "slope_sim_stage4_command"),
        "--descriptor-set", str(descriptor),
        "--payload", str(payload),
        "--duration-ms", "60000",
        "--deadline-ms", "10000",
        "--result", str(tmp_path / "command.json"),
    ]

    result = supervise_recorder_and_command(
        command_argv=command,
        recorder_argv=recorder,
        recorder_result=recorder_result,
        supervisor_result=supervisor_result,
        timeout_sec=15.0,
    )

    assert result["command_stopped"] is True
    assert result["command_returncode"] < 0
    assert result["recorder_safe_stop_required"] is True
    assert json.loads(recorder_result.read_text(encoding="utf-8"))["safe_stop_required"] is True
    assert not (tmp_path / "partial.mcap").exists()


@pytest.mark.stage4_artifact
def test_real_cpp_recorder_command_and_python_runtime_complete_five_second_window(
    tmp_path: Path,
) -> None:
    """正式五秒窗口必须由 Command、Recorder、Subscriber 和 runtime 一起无损完成。"""
    from scripts.stage4_v2_simulation_runtime import run_v2_simulation_runtime

    simulation_session_id = bytes.fromhex("00112233445566778899aabbccddeeff")
    build_directory = Path(os.environ["STAGE4_PHASE0_BUILD_DIR"])
    for target in (
        "slope_sim_stage4_command",
        "slope_sim_stage4_recorder",
        "slope_sim_stage4_subscriber",
    ):
        subprocess.run(["cmake", "--build", str(build_directory), "--target", target], check=True)
    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    command_payload = tmp_path / "wheel-command-world-1.bin"
    command_frame = pb.WheelCommand()
    command_frame.ParseFromString(
        (root / "tests/fixtures/stage4/v2/WheelCommand.bin").read_bytes()
    )
    command_frame.simulation_session_id = simulation_session_id
    command_frame.world_generation = 1
    command_payload.write_bytes(command_frame.SerializeToString(deterministic=True))
    recording = tmp_path / "five-second.mcap"
    recorder_result = tmp_path / "recorder.json"
    command_result = tmp_path / "command.json"
    subscriber_result = tmp_path / "subscriber.json"
    recorder_process = subprocess.Popen(
        [
            str(build_directory / "slope_sim_stage4_recorder"),
            "--descriptor-set", str(descriptor),
            "--scene-id", "c2-five-second-runtime",
            "--simulation-session-id", "00112233445566778899aabbccddeeff",
            "--world-generation", "1",
            "--lidar-pattern-version", "livox-mid360-800000-v1",
            "--lidar-pattern-sha256", "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca",
            "--output", str(recording),
            "--duration-ms", "5000",
            "--deadline-ms", "20000",
            "--result", str(recorder_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    command_process = subprocess.Popen(
        [
            str(build_directory / "slope_sim_stage4_command"),
            "--descriptor-set", str(descriptor),
            "--payload", str(command_payload),
            "--duration-ms", "5000",
            "--deadline-ms", "10000",
            "--result", str(command_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    subscriber_process = subprocess.Popen(
        [
            str(build_directory / "slope_sim_stage4_subscriber"),
            "--all-outputs", "true",
            "--descriptor-set", str(descriptor),
            "--duration-ms", "5000",
            "--deadline-ms", "20000",
            "--result", str(subscriber_result),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        time.sleep(0.1)
        assert recorder_process.poll() is None, recorder_process.stderr.read()
        assert command_process.poll() is None, command_process.stderr.read()
        assert subscriber_process.poll() is None, subscriber_process.stderr.read()
        runtime = run_v2_simulation_runtime(
            result_json=tmp_path / "runtime.json",
            duration_sec=5.0,
            require_verified_peers=True,
            peer_timeout_sec=10.0,
            session_id_factory=lambda: simulation_session_id,
        )
        recorder_stdout, recorder_stderr = recorder_process.communicate(timeout=25)
        command_stdout, command_stderr = command_process.communicate(timeout=10)
        subscriber_stdout, subscriber_stderr = subscriber_process.communicate(timeout=10)
    finally:
        for process in (recorder_process, command_process, subscriber_process):
            if process.poll() is None:
                process.kill()
                process.communicate()

    assert recorder_process.returncode == 0, (
        f"stdout={recorder_stdout}\nstderr={recorder_stderr}"
    )
    assert command_process.returncode == 0, f"stdout={command_stdout}\nstderr={command_stderr}"
    assert subscriber_process.returncode == 0, (
        f"stdout={subscriber_stdout}\nstderr={subscriber_stderr}"
    )
    assert runtime["published_frames"] == {
        "/sim/wheel/state": 500,
        "/sim/lidar/points": 50,
        "/sim/rtk/state": 50,
        "/sim/imu/attitude": 50,
    }
    assert recording.read_bytes().startswith(b"\x89MCAP0\r\n")
    assert json.loads(recorder_result.read_text(encoding="utf-8")) == {
        "clean_shutdown": True,
        "mcap": str(recording),
        "recorded_count": 1150,
        "role": "recorder",
        "topics": {
            "/sim/wheel/command": 500,
            "/sim/wheel/state": 500,
            "/sim/lidar/points": 50,
            "/sim/rtk/state": 50,
            "/sim/imu/attitude": 50,
        },
    }
    assert json.loads(subscriber_result.read_text(encoding="utf-8")) == {
        "clean_shutdown": True,
        "role": "subscriber",
        "topics": {
            "/sim/wheel/state": 500,
            "/sim/lidar/points": 50,
            "/sim/rtk/state": 50,
            "/sim/imu/attitude": 50,
        },
    }


@pytest.mark.stage4_artifact
def test_golf_obstacles_motion_completes_five_second_scene_motion_window(
    tmp_path: Path,
) -> None:
    """Golf 场景必须由持续 Command、Recorder 和真实 PyBullet 完成五秒窗口。"""
    from slope_sim.obstacles import ObstacleGeometry, ObstaclePath, ObstacleSpec
    from slope_sim.scene_config import (
        SceneDocument,
        SensorDocument,
        TerrainDocument,
        dump_scene_atomic,
    )

    root = Path(__file__).resolve().parents[2]
    descriptor = root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
    session_hex = "474f4c462d4d4f54494f4e2d30303031"
    scene = dump_scene_atomic(
        SceneDocument(
            1,
            "df_mid",
            TerrainDocument("golf_heightfield", 0.0, 41, "medium"),
            (
                ObstacleSpec(
                    1,
                    "static",
                    ObstacleGeometry("box", (0.35, 0.35, 0.60)),
                    (-0.8, 1.8, 0.60),
                    (0.0, 0.0, 0.0, 1.0),
                ),
                ObstacleSpec(
                    2,
                    "static",
                    ObstacleGeometry("cylinder", (0.32, 0.32, 0.70)),
                    (0.7, -1.7, 0.70),
                    (0.0, 0.0, 0.0, 1.0),
                ),
                ObstacleSpec(
                    3,
                    "moving",
                    ObstacleGeometry("box", (0.35, 0.35, 0.55)),
                    (-0.2, -0.4, 0.55),
                    (0.0, 0.0, 0.0, 1.0),
                    ObstaclePath((-0.2, -0.4), (-0.2, 0.8), 0.30, 0.0, 1),
                ),
            ),
            SensorDocument.default(),
        ),
        tmp_path / "golf-obstacles-motion.yaml",
    )

    straight = pb.WheelCommand()
    straight.ParseFromString(
        (root / "tests/fixtures/stage4/v2/WheelCommand.bin").read_bytes()
    )
    straight.sequence = 0
    straight.world_generation = 1
    straight.command_generation = 1
    straight.simulation_session_id = bytes.fromhex(session_hex)
    straight.robot_model = "df_mid"
    del straight.drive_wheel_speed_rad_s[:]
    straight.drive_wheel_speed_rad_s.extend((4.0, 4.0))
    del straight.steering_wheel_speed_rad_s[:]
    turn = pb.WheelCommand()
    turn.CopyFrom(straight)
    del turn.drive_wheel_speed_rad_s[:]
    turn.drive_wheel_speed_rad_s.extend((2.875, 4.125))
    straight_payload = tmp_path / "straight.bin"
    turn_payload = tmp_path / "turn.bin"
    straight_payload.write_bytes(straight.SerializeToString(deterministic=True))
    turn_payload.write_bytes(turn.SerializeToString(deterministic=True))

    runtime_result = tmp_path / "runtime.json"
    recording = tmp_path / "golf-obstacles-motion.mcap"
    recorder_result = tmp_path / "recorder.json"
    command_result = tmp_path / "command.json"
    runtime_ready = tmp_path / "runtime.ready"
    command_ready = tmp_path / "command.ready"
    start_signal = tmp_path / "start.signal"
    runtime_process = multiprocessing.get_context("spawn").Process(
        target=_run_five_second_scene_runtime,
        args=(
            str(runtime_result),
            str(scene),
            session_hex,
            str(runtime_ready),
            str(start_signal),
        ),
    )
    recorder_process: subprocess.Popen[str] | None = None
    command_process: subprocess.Popen[str] | None = None
    runtime_process.start()
    try:
        recorder_process = subprocess.Popen(
            [
                str(_required_executable("STAGE4_RECORDER_EXECUTABLE")),
                "--descriptor-set", str(descriptor),
                "--scene-id", "stage4-golf-obstacles-motion",
                "--simulation-session-id", session_hex,
                "--world-generation", "1",
                "--lidar-pattern-version", "livox-mid360-800000-v1",
                "--lidar-pattern-sha256", "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca",
                "--output", str(recording),
                "--duration-ms", "5000",
                "--deadline-ms", "60000",
                "--result", str(recorder_result),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        command_process = subprocess.Popen(
            [
                str(_required_executable("STAGE4_COMMAND_EXECUTABLE")),
                "--descriptor-set", str(descriptor),
                "--payload", str(straight_payload),
                "--turn-payload", str(turn_payload),
                "--turn-at-ms", "2000",
                "--stop-at-ms", "4500",
                "--duration-ms", "5000",
                "--deadline-ms", "60000",
                "--result", str(command_result),
                "--ready-file", str(command_ready),
                "--start-file", str(start_signal),
                "--expected-subscriber-count", "2",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        startup_deadline = time.monotonic() + 60.0
        while not (runtime_ready.is_file() and command_ready.is_file()):
            if not runtime_process.is_alive():
                runtime_detail = (
                    runtime_result.read_text(encoding="utf-8")
                    if runtime_result.exists()
                    else "runtime result was not written"
                )
                pytest.fail(
                    f"runtime exited during coordinated startup: "
                    f"exitcode={runtime_process.exitcode}\n{runtime_detail}"
                )
            for name, process in (
                ("recorder", recorder_process),
                ("command", command_process),
            ):
                if process.poll() is not None:
                    stdout, stderr = process.communicate(timeout=1)
                    pytest.fail(
                        f"{name} exited during coordinated startup: "
                        f"returncode={process.returncode}\nstdout={stdout}\nstderr={stderr}"
                    )
            if time.monotonic() >= startup_deadline:
                pytest.fail(
                    "coordinated startup timed out: "
                    f"runtime_ready={runtime_ready.exists()} command_ready={command_ready.exists()}"
                )
            time.sleep(0.01)
        start_signal.touch(exist_ok=False)
        runtime_process.join(timeout=30)
        recorder_stdout, recorder_stderr = recorder_process.communicate(timeout=25)
        command_stdout, command_stderr = command_process.communicate(timeout=10)
    finally:
        if runtime_process.is_alive():
            runtime_process.terminate()
            runtime_process.join(timeout=5)
        for process in (command_process, recorder_process):
            if process is not None and process.poll() is None:
                process.kill()
                process.communicate()

    assert runtime_process.exitcode == 0
    assert recorder_process is not None and recorder_process.returncode == 0, (
        f"stdout={recorder_stdout}\nstderr={recorder_stderr}"
    )
    assert command_process is not None and command_process.returncode == 0, (
        f"stdout={command_stdout}\nstderr={command_stderr}"
    )
    runtime = json.loads(runtime_result.read_text(encoding="utf-8"))
    assert runtime["robot_model"] == "df_mid"
    assert runtime["terrain_model"] == "golf_heightfield"
    assert runtime["golf_seed"] == 41
    assert runtime["golf_relief"] == "medium"
    assert runtime["dashboard_snapshot"]["command_sequence"] == 499
    assert runtime["physics_steps"] == 1200
    assert runtime["published_frames"] == {
        "/sim/wheel/state": 500,
        "/sim/lidar/points": 50,
        "/sim/rtk/state": 50,
        "/sim/imu/attitude": 50,
    }
    assert json.loads(command_result.read_text(encoding="utf-8")) == {
        "active_published_count": 500,
        "clean_shutdown": True,
        "published_count": 500,
        "safe_stop_published_count": 0,
        "transport": "ecal",
    }
    assert json.loads(recorder_result.read_text(encoding="utf-8")) == {
        "clean_shutdown": True,
        "mcap": str(recording),
        "recorded_count": 1150,
        "role": "recorder",
        "topics": {
            "/sim/wheel/command": 500,
            "/sim/wheel/state": 500,
            "/sim/lidar/points": 50,
            "/sim/rtk/state": 50,
            "/sim/imu/attitude": 50,
        },
    }
    assert recording.read_bytes().startswith(b"\x89MCAP0\r\n")
