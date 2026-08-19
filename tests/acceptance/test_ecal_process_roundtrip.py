# 真实 eCAL 跨进程验收：只执行真实门禁，不允许任何替代或弱化。
from __future__ import annotations

import hashlib
import inspect
import json
import math
from pathlib import Path
import subprocess
from threading import Lock
from types import SimpleNamespace

import pytest

from scripts import ecal_roundtrip_peer as peer_script
from scripts import ecal_simulation_runtime as runtime_script
from scripts import verify_ecal_roundtrip as verifier
from scripts.verify_ecal_roundtrip import (
    _finish_peer,
    _parse_args,
    run_ecal_process_roundtrip,
    run_ecal_reconnect_gate,
)
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.backlog import _has_sustained_backlog
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.logging import InterfaceLogRecord, InterfaceLogSnapshot
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.lidar_worker import LidarServiceSnapshot


CONFIG = InterfaceConfig.default(transport_mode="ecal")
CODEC = ProtoCodec()
WHEEL_COMMAND = CONFIG.wheel_command.topic
WHEEL_STATE = CONFIG.wheel_state.topic
SENSOR_TOPICS = {
    CONFIG.lidar_front.topic,
    CONFIG.lidar_rear.topic,
    CONFIG.rtk.topic,
    CONFIG.imu.topic,
}
OUTPUT_TOPICS = SENSOR_TOPICS | {WHEEL_STATE}
EXPECTED_TYPES = {
    WHEEL_COMMAND: CODEC.type_name(WheelCommand(0, (), ())),
    WHEEL_STATE: CODEC.type_name(WheelState(0, (), ())),
    CONFIG.lidar_front.topic: CODEC.type_name(
        LidarPointCloud(0, "lidar_front", 0, 1, ())
    ),
    CONFIG.lidar_rear.topic: CODEC.type_name(
        LidarPointCloud(0, "lidar_rear", 0, 2, ())
    ),
    CONFIG.rtk.topic: CODEC.type_name(RtkState(0, 0.0, 0.0, 0.0, 0.0)),
    CONFIG.imu.topic: CODEC.type_name(ImuAttitude(0, 0.0, 0.0)),
}


def test_finish_peer_kills_and_reaps_after_terminate_timeout():
    class StubbornProcess:
        def __init__(self) -> None:
            self.returncode = None
            self.communicate_count = 0
            self.terminate_count = 0
            self.kill_count = 0

        def communicate(self, timeout: float):
            self.communicate_count += 1
            if self.communicate_count <= 2:
                raise subprocess.TimeoutExpired("peer", timeout)
            self.returncode = -9
            return "", ""

        def terminate(self) -> None:
            self.terminate_count += 1

        def kill(self) -> None:
            self.kill_count += 1

    process = StubbornProcess()

    with pytest.raises(TimeoutError, match="peer did not exit"):
        _finish_peer(process, timeout_sec=0.01)

    assert process.terminate_count == 1
    assert process.kill_count == 1
    assert process.communicate_count == 3


@pytest.mark.parametrize(
    ("kwargs", "detail"),
    (
        ({"runtime": "synthetic"}, "runtime"),
        ({"runtime": "simulation", "warmup_sec": -0.1}, "warmup_sec"),
        ({"runtime": "simulation", "warmup_sec": 0.0}, "warmup_sec"),
        ({"runtime": "simulation", "warmup_sec": float("nan")}, "warmup_sec"),
        ({"runtime": "simulation", "process_timeout_sec": 0.0}, "process_timeout_sec"),
    ),
)
def test_roundtrip_rejects_invalid_process_configuration_before_start(
    monkeypatch, kwargs, detail
):
    def unexpected_process(*_args, **_kwargs):
        raise AssertionError("invalid configuration started a child process")

    monkeypatch.setattr(subprocess, "Popen", unexpected_process)

    with pytest.raises(ValueError, match=detail):
        run_ecal_process_roundtrip(duration_sec=1.0, **kwargs)


def test_simulation_discovery_wait_covers_official_registration_timeout(
    monkeypatch,
):
    """官方 eCAL 失联判定上界内才连通时，场景门禁不能提前超时。"""
    now = {"value": 0.0}

    class DelayedBindings:
        @staticmethod
        def is_peer_connected(_resource):
            return now["value"] >= 10.0

    def sleep(duration):
        now["value"] += duration

    monkeypatch.setattr(
        peer_script,
        "time",
        SimpleNamespace(monotonic=lambda: now["value"], sleep=sleep),
    )

    peer_script._wait_for_v61_resource_peer(
        DelayedBindings(),
        object(),
        peer_script._SCENARIO_ACK_TIMEOUT_SEC,
        "delayed official discovery",
    )

    assert now["value"] >= 10.0


def test_simulation_scenario_budget_covers_every_serial_protocol_wait() -> None:
    """总预算必须覆盖慢但逐步成功的完整断连协议，不能复用单次启动上限。"""
    output_count = len(peer_script._official_output_message_types())
    serial_wait_count = 10 + 3 * output_count
    expected = (
        60.0
        + 1.0
        + 5.0
        + serial_wait_count * peer_script._SCENARIO_ACK_TIMEOUT_SEC
        + 2.0
    )

    assert serial_wait_count == 25
    assert verifier._simulation_scenario_budget_sec(  # type: ignore[attr-defined]
        duration_sec=5.0,
        warmup_sec=1.0,
        startup_timeout_sec=60.0,
    ) == pytest.approx(expected)


def test_simulation_peer_process_receives_real_production_warmup(monkeypatch, tmp_path):
    """warmup 必须在 start 门闩后由真实 peer 驱动生产负载。"""
    calls: list[list[str]] = []

    class FakeProcess:
        pass

    def fake_popen(command, **_kwargs):
        calls.append(command)
        return FakeProcess()

    monkeypatch.setattr(verifier.subprocess, "Popen", fake_popen)

    verifier._start_simulation_peer(
        result_json=tmp_path / "result.json",
        scenario_dir=tmp_path / "scenario",
        ready_file=tmp_path / "ready",
        start_file=tmp_path / "start",
        duration_sec=5.0,
        warmup_sec=1.25,
        participant_name="warmup-peer",
        start_timeout_sec=60.0,
    )

    command = calls[0]
    assert command[command.index("--warmup-sec") + 1] == "1.25"


@pytest.mark.parametrize(
    ("robot_model", "drive_count", "steering_count"),
    (
        ("df_back", 2, 0),
        ("active_steering_4wd", 4, 2),
    ),
)
def test_simulation_peer_builds_commands_for_the_selected_robot_model(
    robot_model: str,
    drive_count: int,
    steering_count: int,
) -> None:
    """正式 peer 必须按当前车型发送 2+0 或 4+2 命令。"""
    build_command = getattr(peer_script, "_simulation_command_for_model", None)
    assert callable(build_command), "simulation peer needs a model-aware command helper"

    command = build_command(robot_model, 10_000_000, drive_speed_rad_s=4.0)

    assert command.timestamp_ns == 10_000_000
    assert len(command.drive_wheel_speed_rad_s) == drive_count
    assert len(command.steering_wheel_speed_rad_s) == steering_count
    assert all(value > 0.0 for value in command.drive_wheel_speed_rad_s)


def test_simulation_peer_marks_ready_only_after_all_resources_are_discovered(
    monkeypatch,
    tmp_path,
) -> None:
    """共同 start 门闩打开前，peer ready 必须已经代表完整六话题发现。"""

    class StopAtStart(RuntimeError):
        pass

    class Bindings:
        def __init__(self) -> None:
            self.discovery_calls = 0
            self.closed: list[object] = []

        @staticmethod
        def create_participant(_name: str) -> object:
            return object()

        @staticmethod
        def create_subscriber(
            _topic: str,
            _message_type: type,
            _callback,
        ) -> object:
            return object()

        def is_peer_connected(self, _resource: object) -> bool:
            self.discovery_calls += 1
            return True

        def close(self, resource: object) -> None:
            self.closed.append(resource)

    bindings = Bindings()
    ready_file = tmp_path / "peer.ready"

    monkeypatch.setattr(peer_script, "load_ecal_bindings", lambda: bindings)
    monkeypatch.setattr(
        peer_script,
        "_create_command_publisher",
        lambda _bindings, _config: object(),
    )
    monkeypatch.setattr(
        peer_script,
        "_official_output_message_types",
        lambda: {"/sim/test/output": object},
    )

    def stop_after_ready(_start_file, _timeout_sec: float) -> None:
        assert ready_file.read_text(encoding="utf-8") == "ready\n"
        assert bindings.discovery_calls == 2
        raise StopAtStart("stop after readiness assertion")

    monkeypatch.setattr(peer_script, "_wait_for_start", stop_after_ready)

    with pytest.raises(StopAtStart, match="stop after readiness assertion"):
        peer_script.run_simulation_peer(
            result_json=tmp_path / "peer-result.json",
            scenario_dir=tmp_path / "scenario",
            duration_sec=1.0,
            warmup_sec=1.0,
            participant_name="ready-after-discovery",
            ready_file=ready_file,
            start_file=tmp_path / "start.signal",
            start_timeout_sec=1.0,
        )

    assert len(bindings.closed) == 3


def test_simulation_peer_waits_for_output_data_plane_before_warmup() -> None:
    """控制面发现后，warmup 前仍须看到五话题的真实 output 回调。"""
    wait_for_output_delivery = getattr(
        peer_script,
        "_wait_for_output_delivery",
        None,
    )
    assert callable(
        wait_for_output_delivery
    ), "simulation peer needs an output data-plane preflight"

    first_topic = CONFIG.wheel_state.topic
    second_topic = CONFIG.lidar_front.topic
    received = {first_topic: [], second_topic: []}
    clock = {"now": 0.0}
    deliveries = iter((first_topic, second_topic))

    def sleep(duration: float) -> None:
        clock["now"] += duration
        try:
            received[next(deliveries)].append({"timestamp_ns": 1})
        except StopIteration:
            pass

    wait_for_output_delivery(
        received,
        Lock(),
        (first_topic, second_topic),
        timeout_sec=1.0,
        monotonic=lambda: clock["now"],
        sleep=sleep,
    )

    source = inspect.getsource(peer_script.run_simulation_peer)
    assert source.index("_wait_for_output_delivery(") < source.index(
        "warmup_events, _warmup_start, _warmup_end = _run_v61_command_schedule("
    )


def test_simulation_children_receive_the_same_differential_robot_model(
    monkeypatch,
    tmp_path,
) -> None:
    """orchestrator 必须把同一个差速车型传给 runtime 与 peer 子进程。"""
    calls: list[list[str]] = []

    class FakeProcess:
        pass

    def fake_popen(command, **_kwargs):
        calls.append(command)
        return FakeProcess()

    monkeypatch.setattr(verifier.subprocess, "Popen", fake_popen)
    verifier._start_simulation_runtime(
        result_json=tmp_path / "runtime.json",
        scenario_dir=tmp_path / "scenario",
        ready_file=tmp_path / "runtime.ready",
        start_file=tmp_path / "start",
        stop_file=tmp_path / "stop",
        participant_name="differential-runtime",
        max_runtime_sec=30.0,
        robot_model="df_back",
    )
    verifier._start_simulation_peer(
        result_json=tmp_path / "peer.json",
        scenario_dir=tmp_path / "scenario",
        ready_file=tmp_path / "peer.ready",
        start_file=tmp_path / "start",
        duration_sec=5.0,
        warmup_sec=1.0,
        participant_name="differential-peer",
        start_timeout_sec=30.0,
        robot_model="df_back",
    )

    assert len(calls) == 2
    for command in calls:
        assert command[command.index("--robot-model") + 1] == "df_back"


def test_simulation_schedule_sends_differential_two_plus_zero_commands(
    monkeypatch,
) -> None:
    """生产 100 Hz schedule 必须实际发送差速 2+0，而非只暴露孤立 helper。"""
    now = {"value": 0.0}
    sent: list[WheelCommand] = []

    def sleep(duration: float) -> None:
        now["value"] += duration

    def capture_command(_bindings, _publisher, command, _codec) -> None:
        sent.append(command)

    monkeypatch.setattr(
        peer_script,
        "time",
        SimpleNamespace(monotonic=lambda: now["value"], sleep=sleep),
    )
    monkeypatch.setattr(peer_script, "_send_v61_command", capture_command)

    events, started_at, ended_at = peer_script._run_v61_command_schedule(
        object(),
        object(),
        duration_sec=0.025,
        config=CONFIG,
        codec=CODEC,
        robot_model="df_back",
    )

    assert started_at == pytest.approx(0.0)
    assert ended_at == pytest.approx(0.025)
    assert len(sent) == len(events) == 3
    assert all(len(command.drive_wheel_speed_rad_s) == 2 for command in sent)
    assert all(len(command.steering_wheel_speed_rad_s) == 0 for command in sent)
    assert all(event["drive_wheel_count"] == 2 for event in events)
    assert all(event["steering_wheel_count"] == 0 for event in events)


def test_simulation_command_cardinality_oracle_rejects_wrong_model_evidence() -> None:
    """verifier 必须独立拒绝把 4+2 命令伪装成差速 2+0。"""
    validate = getattr(verifier, "_validate_simulation_command_cardinality", None)
    assert callable(validate), "simulation verifier needs a command cardinality oracle"
    valid_events = [
        {"drive_wheel_count": 2, "steering_wheel_count": 0},
        {"drive_wheel_count": 2, "steering_wheel_count": 0},
    ]

    validate(valid_events, robot_model="df_back")
    invalid_events = [
        {"drive_wheel_count": 4, "steering_wheel_count": 2},
    ]
    with pytest.raises(AssertionError, match="cardinality"):
        validate(invalid_events, robot_model="df_back")


@pytest.mark.parametrize(
    ("robot_model", "reported", "expected"),
    (
        ("df_back", False, False),
        ("active_steering_4wd", True, True),
    ),
)
def test_simulation_steering_direction_evidence_matches_model_contract(
    robot_model: str,
    reported: bool,
    expected: bool,
) -> None:
    """差速无转向关节应报告 False，主动转向则必须报告 True。"""
    parse = getattr(verifier, "_strict_model_steering_same_sign", None)
    assert callable(parse), "simulation verifier needs a model-aware steering parser"

    assert parse(
        {"normal_load_steering_same_sign": reported},
        robot_model=robot_model,
        description="simulation runtime",
    ) is expected
    with pytest.raises(AssertionError, match="normal_load_steering_same_sign"):
        parse(
            {"normal_load_steering_same_sign": not reported},
            robot_model=robot_model,
            description="simulation runtime",
        )


def test_transport_harness_polls_discovery_before_reading_peer_snapshot():
    """纯 transport harness 不得依赖已从 publisher worker 移除的隐式轮询。"""
    calls: list[str] = []

    class ExplicitPollingTransport:
        def poll_peer_state(self):
            calls.append("poll")

        def snapshot(self):
            calls.append("snapshot")
            return SimpleNamespace(
                topic_quality=(
                    SimpleNamespace(peer_connected=True),
                    SimpleNamespace(peer_connected=True),
                )
            )

    assert verifier._all_topic_peers_connected(ExplicitPollingTransport()) is True
    assert calls == ["poll", "snapshot"]


def test_transport_harness_polls_discovery_before_reading_lifecycle_state():
    """断连与重连等待必须主动推进 discovery，不能反复读取旧状态。"""
    calls: list[str] = []

    class ExplicitPollingTransport:
        def poll_peer_state(self):
            calls.append("poll")

        def snapshot(self):
            calls.append("snapshot")
            return SimpleNamespace(state="disconnected")

    assert verifier._polled_transport_state(ExplicitPollingTransport()) == "disconnected"
    assert calls == ["poll", "snapshot"]


def test_physics_step_uses_wall_time_captured_after_transport_poll() -> None:
    """discovery poll 跨过 timeout 时，本物理帧必须立即使用新的墙钟安全停车。"""
    clock = {"value": 10.0}
    calls: list[object] = []

    class Runtime:
        def poll_transport(self) -> None:
            calls.append("poll")
            clock["value"] += 0.11

        def before_physics_step(self, dt: float, *, wall_time: float) -> None:
            calls.append(("before", dt, wall_time))

    prepare = getattr(runtime_script, "_prepare_physics_step", None)
    assert callable(prepare), "runtime needs a fresh post-poll wall-clock boundary"
    cadence = runtime_script.RuntimeObservationCadence(
        monotonic=lambda: clock["value"],
    )

    decision, decision_time, observation_due = prepare(
        Runtime(),
        1.0 / 240.0,
        observation_cadence=cadence,
    )

    assert decision is None
    assert observation_due is True
    assert decision_time == pytest.approx(10.11)
    assert calls == ["poll", ("before", 1.0 / 240.0, pytest.approx(10.11))]


def test_physics_step_checks_command_timeout_when_discovery_is_not_due() -> None:
    """非 discovery 帧仍须用新墙钟执行 240 Hz 命令安全决策。"""
    decision = object()
    calls: list[object] = []

    class Runtime:
        def poll_transport(self) -> None:
            raise AssertionError("non-due frame polled discovery")

        def before_physics_step(self, dt: float, *, wall_time: float) -> object:
            calls.append(("before", dt, wall_time))
            return decision

    clock = [10.0]

    class PrimeRuntime:
        def poll_transport(self) -> None:
            pass

    cadence = runtime_script.RuntimeObservationCadence(
        monotonic=lambda: clock[0],
    )
    cadence.poll_if_due(PrimeRuntime())
    clock[0] = 10.010

    result, decision_time, observation_due = runtime_script._prepare_physics_step(
        Runtime(),
        1.0 / 240.0,
        observation_cadence=cadence,
    )

    assert result is decision
    assert observation_due is False
    assert decision_time == pytest.approx(10.010)
    assert calls == [("before", 1.0 / 240.0, pytest.approx(10.010))]


def test_scheduled_physics_step_rebases_discovery_after_slow_poll() -> None:
    """native poll 超过 50 ms 时，下一期限从 poll 完成时重建而非立即补跑。"""
    clock = {"value": 10.0}
    calls: list[str] = []

    class Runtime:
        def poll_transport(self) -> None:
            calls.append("poll")
            clock["value"] += 0.080

        def before_physics_step(self, _dt: float, *, wall_time: float) -> str:
            calls.append(f"before:{wall_time:.3f}")
            return "decision"

    prepare = getattr(runtime_script, "_prepare_scheduled_physics_step", None)
    assert callable(prepare), "runtime needs a post-poll observation deadline"
    cadence = runtime_script.RuntimeObservationCadence(
        monotonic=lambda: clock["value"],
    )

    decision, decision_time, due, next_at, physics_step_due = prepare(
        Runtime(),
        1.0 / 240.0,
        observation_cadence=cadence,
    )

    assert decision == "decision"
    assert decision_time == pytest.approx(10.080)
    assert due is True
    assert physics_step_due is True
    assert next_at == pytest.approx(10.130)

    clock["value"] = 10.081
    _, _, due, unchanged_next, physics_step_due = prepare(
        Runtime(),
        1.0 / 240.0,
        observation_cadence=cadence,
    )
    assert due is False
    assert physics_step_due is True
    assert unchanged_next == pytest.approx(10.130)
    assert calls.count("poll") == 1


def test_runtime_observation_cadence_is_20hz_without_catch_up_bursts() -> None:
    """专用 eCAL 入口必须复用公共 cadence，迟到时也只执行一次 native 查询。"""
    from slope_sim.realtime import RuntimeObservationCadence

    cadence_type = getattr(runtime_script, "RuntimeObservationCadence", None)
    assert cadence_type is RuntimeObservationCadence
    clock = [0.0]
    polls: list[float] = []

    class Runtime:
        def poll_transport(self) -> None:
            polls.append(clock[0])

    cadence = cadence_type(monotonic=lambda: clock[0])
    runtime = Runtime()
    for frame in range(240):
        clock[0] = frame / 240.0
        cadence.poll_if_due(runtime)
    assert 19 <= len(polls) <= 20

    before_late = len(polls)
    clock[0] = 2.0
    assert cadence.poll_if_due(runtime)[0] is True
    clock[0] = 2.001
    assert cadence.poll_if_due(runtime)[0] is False
    assert len(polls) == before_late + 1


def test_slow_discovery_poll_crossing_deadline_cancels_the_physics_step() -> None:
    """poll 前尚未到期也不够；安全决策后的新墙钟到期时不得再推进物理。"""
    clock = iter((14.999, 15.001))
    calls: list[object] = []

    class Runtime:
        def poll_transport(self) -> None:
            calls.append("poll")

        def before_physics_step(self, time_step_sec: float, *, wall_time: float):
            calls.append((time_step_sec, wall_time))
            return object()

        def after_physics_step(self, _time_step_sec: float):
            raise AssertionError("deadline-crossing frame must not publish")

    class Coordinator:
        def step(self, _time_step_sec: float) -> None:
            raise AssertionError("deadline-crossing frame must not step")

    run_frame = getattr(runtime_script, "_run_scheduled_physics_frame", None)
    assert callable(run_frame), "runtime needs one testable scheduled-frame entry"
    frame = run_frame(
        Runtime(),
        Coordinator(),
        1.0 / 240.0,
        observation_cadence=runtime_script.RuntimeObservationCadence(
            monotonic=lambda: next(clock),
        ),
        allow_physics_step=True,
        normal_load_tracker=SimpleNamespace(start_wall_time=10.0),
        normal_load_duration_sec=5.0,
    )

    assert calls[0] == "poll"
    assert frame.decision_wall_time == pytest.approx(15.001)
    assert frame.observation_due is True
    assert frame.next_observation_at == pytest.approx(15.051)
    assert frame.advanced is False


def test_fence_pause_keeps_discovery_observation_without_physics_step() -> None:
    """等待下一协议 marker 时只停物理，连接观测和命令安全判断仍须继续。"""
    calls: list[str] = []

    class Runtime:
        def poll_transport(self) -> None:
            calls.append("poll")

        def before_physics_step(self, _dt: float, *, wall_time: float):
            calls.append(f"before:{wall_time:.3f}")
            return object()

        def after_physics_step(self, _dt: float):
            raise AssertionError("paused fence must not publish")

    class Coordinator:
        def step(self, _dt: float) -> None:
            raise AssertionError("paused fence must not step")

    run_frame = getattr(runtime_script, "_run_scheduled_physics_frame", None)
    assert callable(run_frame), "runtime needs one testable scheduled-frame entry"
    frame = run_frame(
        Runtime(),
        Coordinator(),
        1.0 / 240.0,
        observation_cadence=runtime_script.RuntimeObservationCadence(
            monotonic=lambda: 20.0,
        ),
        allow_physics_step=False,
    )

    assert calls == ["poll", "before:20.000"]
    assert frame.observation_due is True
    assert frame.advanced is False


def test_transport_busy_iterations_keep_safety_checks_and_consume_deadlines() -> None:
    """发送 lane 忙时逐轮停物理并推进 deadline，空闲后只恢复一帧。"""
    calls: list[str] = []

    class Runtime:
        def poll_transport(self) -> None:
            calls.append("poll")

        def before_physics_step(self, _dt: float, *, wall_time: float):
            calls.append(f"before:{wall_time:.3f}")
            return object()

        def after_physics_step(self, _dt: float):
            calls.append("publish")
            return (object(),)

    class Coordinator:
        def step(self, _dt: float) -> None:
            calls.append("step")

    class Pacer:
        def wait_for_next_deadline(self) -> None:
            calls.append("wait")

    run_frame = getattr(
        runtime_script,
        "_run_scheduled_physics_frame",
        None,
    )
    finish_iteration = getattr(
        runtime_script,
        "_finish_gated_physics_iteration",
        None,
    )
    assert callable(run_frame), "runtime needs one testable scheduled frame"
    assert callable(finish_iteration), "runtime needs one testable gated frame tail"

    cadence = runtime_script.RuntimeObservationCadence(monotonic=lambda: 20.0)
    runtime = Runtime()
    coordinator = Coordinator()
    pacer = Pacer()
    frames = []
    for allow_physics_step in (False, False, True):
        frame = run_frame(
            runtime,
            coordinator,
            1.0 / 240.0,
            observation_cadence=cadence,
            allow_physics_step=allow_physics_step,
        )
        finish_iteration(
            frame,
            allow_physics_step=allow_physics_step,
            pacer=pacer,
        )
        frames.append(frame)

    assert [frame.advanced for frame in frames] == [False, False, True]
    assert calls == [
        "poll",
        "before:20.000",
        "wait",
        "before:20.000",
        "wait",
        "before:20.000",
        "step",
        "publish",
    ]


def test_busy_transport_blocks_only_due_topics_with_busy_lanes() -> None:
    """LiDAR lane 忙不能阻断 wheel-only 帧，只阻断同话题再次到期的帧。"""
    calls: list[str] = []

    class Runtime:
        def __init__(self, topics: tuple[str, ...]) -> None:
            self.topics = topics

        def next_physics_step_publish_topics(
            self,
            time_step_sec: float,
        ) -> tuple[str, ...]:
            assert time_step_sec == pytest.approx(1.0 / 240.0)
            calls.append(f"preview:{','.join(self.topics)}")
            return self.topics

    class Transport:
        def __init__(self, busy_topics: set[str]) -> None:
            self.busy_topics = busy_topics

        def is_topic_idle(self, topic: str) -> bool:
            calls.append(f"idle:{topic}")
            return topic not in self.busy_topics

        def is_idle(self) -> bool:
            raise AssertionError("cadence gate must not consult global transport state")

    allows_step = getattr(
        runtime_script,
        "_transport_allows_physics_step",
        None,
    )
    assert callable(allows_step), "runtime needs a cadence-aware transport gate"

    busy_lidar = Transport({"lidar"})
    assert allows_step(Runtime(()), busy_lidar, 1.0 / 240.0) is True
    assert allows_step(Runtime(("wheel",)), busy_lidar, 1.0 / 240.0) is True
    assert allows_step(Runtime(("lidar",)), busy_lidar, 1.0 / 240.0) is False
    assert calls == [
        "preview:",
        "preview:wheel",
        "idle:wheel",
        "preview:lidar",
        "idle:lidar",
    ]


def test_runtime_observation_reuses_dashboard_status_without_duplicate_snapshot() -> None:
    """headless 门禁只构造一次组合 status，并保留逐话题 peer 原始快照。"""
    calls: list[str] = []
    status = object()
    dashboard = SimpleNamespace(status=status)
    transport_status = object()

    class Runtime:
        def dashboard_snapshot(self, *, wall_time: float) -> object:
            assert wall_time == pytest.approx(12.5)
            calls.append("dashboard")
            return dashboard

        def status_snapshot(self, *, wall_time: float) -> object:
            raise AssertionError("status_snapshot duplicated dashboard status")

    class Transport:
        def snapshot(self) -> object:
            calls.append("transport")
            return transport_status

    capture = getattr(runtime_script, "_capture_runtime_observation", None)
    assert callable(capture), "runtime needs one combined observation boundary"

    observed_status, observed_dashboard, observed_transport = capture(
        Runtime(),
        Transport(),
        wall_time=12.5,
    )

    assert observed_status is status
    assert observed_dashboard is dashboard
    assert observed_transport is transport_status
    assert calls == ["dashboard", "transport"]


def test_runtime_result_serializes_current_lidar_service_snapshot() -> None:
    """P0 result 固定保存当前 worker 的全部生命周期诊断字段。"""
    snapshot = LidarServiceSnapshot(
        "draining",
        42,
        3,
        2,
        8,
        (7, 3, 2, "lidar_front", 900_000_000),
        (3, 2, "lidar_rear", 950_000_000),
        4,
        1,
        2,
        3,
        80_000_000,
        "worker_protocol_failed",
        "response mismatch",
    )

    class Runtime:
        def lidar_service_snapshot(self) -> LidarServiceSnapshot:
            return snapshot

    serialize = getattr(runtime_script, "_lidar_service_snapshot_result", None)
    assert callable(serialize), "runtime result needs LiDAR service diagnostics"
    assert serialize(Runtime()) == {
        "state": "draining",
        "child_pid": 42,
        "lifecycle_generation": 3,
        "pause_epoch": 2,
        "next_job_id": 8,
        "in_flight_identity": (7, 3, 2, "lidar_front", 900_000_000),
        "pending_capture_identity": (3, 2, "lidar_rear", 950_000_000),
        "completed_count": 4,
        "failed_count": 1,
        "overrun_count": 2,
        "stale_count": 3,
        "max_capture_to_response_ns": 80_000_000,
        "last_error_code": "worker_protocol_failed",
        "last_error_detail": "response mismatch",
    }

    class RuntimeWithoutService:
        def lidar_service_snapshot(self) -> None:
            return None

    assert serialize(RuntimeWithoutService()) is None


@pytest.mark.parametrize(
    ("previous_interval", "installed_interval"),
    (
        (0.005, runtime_script._COMMAND_CALLBACK_SWITCH_INTERVAL_SEC),
        (0.0005, 0.0005),
    ),
)
def test_command_callback_scheduling_restores_exact_previous_interval(
    monkeypatch,
    previous_interval: float,
    installed_interval: float,
) -> None:
    """实时窗口只收紧调度，并在退出时精确恢复调用方原值。"""
    set_calls: list[float] = []
    monkeypatch.setattr(
        runtime_script.sys,
        "getswitchinterval",
        lambda: previous_interval,
    )
    monkeypatch.setattr(runtime_script.sys, "setswitchinterval", set_calls.append)

    saved_interval = runtime_script._install_command_callback_scheduling()
    runtime_script._restore_command_callback_scheduling(saved_interval)

    assert saved_interval == previous_interval
    assert set_calls == [installed_interval, previous_interval]


def test_simulation_runtime_main_loop_uses_scheduled_observation_and_boundaries() -> None:
    """真实入口必须接入低频观测和成对仿真时钟屏障，不能只留下孤立 helper。"""
    source = inspect.getsource(runtime_script.run_simulation_runtime)
    frame_source = inspect.getsource(runtime_script._run_scheduled_physics_frame)
    frame_tail_source = inspect.getsource(
        runtime_script._finish_gated_physics_iteration
    )

    assert "_prepare_scheduled_physics_step(" in frame_source
    assert "observation_cadence.poll_if_due(" in inspect.getsource(
        runtime_script._prepare_physics_step
    )
    assert "_run_scheduled_physics_frame(" in source
    scheduling_install = (
        "previous_thread_switch_interval_sec = "
        "_install_command_callback_scheduling()"
    )
    assert scheduling_install in source
    assert source.index(scheduling_install) < source.index(
        "while not stop_file.exists():"
    )
    assert "if previous_thread_switch_interval_sec is not None:" in source
    assert source.rindex("finally:") < source.rindex(
        "_restore_command_callback_scheduling("
    )
    assert "pacer.wait_for_next_deadline()" in frame_tail_source
    assert "RuntimeObservationCadence()" in source
    assert "_capture_runtime_observation(" in source
    assert "_begin_normal_load_motion_window(" in source
    assert "_capture_normal_load_end(" in source
    assert "_WheelDrainFenceGate(" in source
    assert "_wheel_drain_physics_step_due(" in source
    assert "_write_final_protocol_ack(" in source
    assert "_final_protocol_physics_step_due(" in source
    assert "_transport_allows_physics_step(" in source
    assert "and transport.is_idle()" not in source
    assert "allow_physics_step=physics_step_due" in source
    assert "_wait_for_normal_load_completion_marker(" in source
    assert source.index("_wait_for_normal_load_completion_marker(") < source.index(
        "_run_scheduled_physics_frame("
    )
    assert source.index("if observation_due:") < source.index(
        "_finish_gated_physics_iteration("
    )
    assert "frame.published_wheel_states" in source
    assert 'before_ack=begin_motion_window' in source
    assert '(scenario_dir / "invalid.active").exists()' not in source
    assert '"interface_log_files"' in source
    assert '"events"' in source
    assert "runtime.poll_transport()" not in source


def test_normal_load_scene_contains_twenty_obstacles_before_interface_session(
    monkeypatch,
) -> None:
    """P0 worker 输入必须由未绑定 runtime 的 bootstrap coordinator 先完整生成。"""
    bootstrap = getattr(runtime_script, "_bootstrap_normal_load_scene", None)
    assert callable(bootstrap), "runtime needs a normal-load bootstrap helper"
    initial_document = SimpleNamespace(sensors=SimpleNamespace())
    prepared_document = SimpleNamespace(obstacles=tuple(range(20)))
    world = object()
    coordinator_calls: list[dict[str, object]] = []

    class Manager:
        def snapshot(self, *, include_body_id=False):
            if include_body_id:
                return tuple(
                    SimpleNamespace(body_id=index + 1) for index in range(20)
                )
            return tuple(range(20))

    class BootstrapCoordinator:
        def __init__(
            self,
            client_id,
            config,
            selected_world,
            obstacle_manager,
            *,
            interface_runtime=None,
            sensor_document=None,
        ):
            coordinator_calls.append(
                {
                    "client_id": client_id,
                    "config": config,
                    "world": selected_world,
                    "manager": obstacle_manager,
                    "interface_runtime": interface_runtime,
                    "sensor_document": sensor_document,
                }
            )

        def apply_action(self, action):
            assert action.request.count == 20
            return SimpleNamespace(
                obstacle_result=SimpleNamespace(succeeded=True),
            )

        def logical_scene_document(self):
            return prepared_document

    manager = Manager()
    config = SimpleNamespace(time_step=1.0 / 240.0)
    monkeypatch.setattr(
        runtime_script,
        "build_world_from_scene_document",
        lambda *_args: (world, manager),
    )
    monkeypatch.setattr(runtime_script, "SimulationCoordinator", BootstrapCoordinator)

    (
        actual_world,
        actual_manager,
        runtime_document,
        obstacle_count,
        obstacle_body_ids,
    ) = bootstrap(3, config, initial_document)

    assert actual_world is world
    assert actual_manager is manager
    assert runtime_document is prepared_document
    assert obstacle_count == 20
    assert obstacle_body_ids == frozenset(range(1, 21))
    assert coordinator_calls[0]["interface_runtime"] is None
    assert coordinator_calls[0]["sensor_document"] is initial_document.sensors


def test_runtime_ready_file_follows_worker_preflight_for_twenty_obstacles() -> None:
    """ready 文件只能在 20 障碍 bootstrap 与 session worker ready 之后写入。"""
    source = inspect.getsource(runtime_script.run_simulation_runtime)

    bootstrap_index = source.index("_bootstrap_normal_load_scene(")
    session_index = source.index("create_interface_session(")
    ready_index = source.index("ready_file.write_text(")

    assert bootstrap_index < session_index < ready_index


def test_runtime_passes_bootstrap_document_to_session_before_writing_ready_file(
    monkeypatch,
    tmp_path,
) -> None:
    """ready 闩必须可观察到 session 已以完整 20 障碍文档完成 worker preflight。"""
    initial_document = SimpleNamespace(sensors=SimpleNamespace())
    prepared_document = SimpleNamespace(obstacles=tuple(range(20)), sensors=SimpleNamespace())
    robot = object()
    world = SimpleNamespace(active_robot=SimpleNamespace(robot=robot))
    manager = object()
    session_documents: list[object] = []
    session_created = False

    class StopAfterReady(RuntimeError):
        pass

    class FakeTransport:
        pass

    class FakeRuntime:
        config = SimpleNamespace(channels=())
        close_trace = ()

    class FakeSession:
        actual_transport_mode = "ecal"
        transport = FakeTransport()
        logger = object()
        runtime = FakeRuntime()

        def close(self):
            return None

    class FinalCoordinator:
        def __init__(self, _client_id, _config, selected_world, _manager, **kwargs):
            assert kwargs["interface_runtime"] is FakeSession.runtime
            assert kwargs["sensor_document"] is prepared_document.sensors
            self.world = selected_world

    def create_session(_config, *, document, **_kwargs):
        nonlocal session_created
        session_documents.append(document)
        assert len(document.obstacles) == 20
        session_created = True
        return FakeSession()

    def stop_after_ready(_start_file, _stop_file, *, timeout_sec):
        assert timeout_sec > 0.0
        assert session_created
        assert ready_file.exists()
        raise StopAfterReady("stop after ready gate")

    ready_file = tmp_path / "runtime.ready"
    monkeypatch.setattr(runtime_script, "initial_scene_document", lambda _config: initial_document)
    monkeypatch.setattr(
        runtime_script,
        "_bootstrap_normal_load_scene",
        lambda *_args: (world, manager, prepared_document, 20, frozenset(range(1, 21))),
    )
    monkeypatch.setattr(runtime_script, "create_interface_session", create_session)
    monkeypatch.setattr(runtime_script, "SimulationCoordinator", FinalCoordinator)
    monkeypatch.setattr(runtime_script, "EcalTransport", FakeTransport)
    monkeypatch.setattr(runtime_script, "_wait_for_start_signal", stop_after_ready)
    monkeypatch.setattr(runtime_script.p, "connect", lambda _mode: 17)
    monkeypatch.setattr(runtime_script.p, "disconnect", lambda _client_id: None)

    with pytest.raises(StopAfterReady, match="stop after ready gate"):
        runtime_script.run_simulation_runtime(
            result_json=tmp_path / "result.json",
            scenario_dir=tmp_path / "scenario",
            ready_file=ready_file,
            start_file=tmp_path / "start",
            stop_file=tmp_path / "stop",
            participant_name="task11-bootstrap-test",
            max_runtime_sec=1.0,
        )

    assert session_documents == [prepared_document]


def test_final_protocol_ack_pauses_physics_before_peer_shutdown(tmp_path) -> None:
    """最终 ACK 后必须停止发布，让 peer 在静默 transport 上完成资源关闭。"""
    final_ack = tmp_path / "new_command.ack"
    should_step = getattr(
        runtime_script,
        "_final_protocol_physics_step_due",
        None,
    )
    assert callable(should_step), "runtime needs a final protocol shutdown fence"

    assert should_step(final_ack) is True
    final_ack.write_text('{"active": true}', encoding="utf-8")
    assert should_step(final_ack) is False


def test_final_protocol_ack_drains_sensor_and_io_before_staying_frozen(
    tmp_path,
) -> None:
    """最终 ACK 前排空 sensor/logger/transport，ACK 后保持 capture 冻结。"""
    final_ack = tmp_path / "new_command.ack"
    order: list[str] = []
    fence = object()

    class Runtime:
        def begin_sensor_fence(self) -> object:
            order.append("sensor")
            return fence

        def complete_sensor_fence(
            self,
            received_fence: object,
            *,
            resume_capture: bool,
        ) -> None:
            assert received_fence is fence
            assert resume_capture is False
            assert final_ack.exists()
            order.append("frozen")

    class Logger:
        def snapshot(self) -> InterfaceLogSnapshot:
            order.append("logger")
            return InterfaceLogSnapshot(1, 0, 0, 0, False, False, 0)

    class Transport:
        def wait_idle(self, *, timeout_sec: float) -> None:
            assert not final_ack.exists()
            assert timeout_sec == runtime_script._TRANSPORT_IDLE_TIMEOUT_SEC
            order.append("transport_idle")

        def snapshot(self) -> object:
            order.append("snapshot")
            return SimpleNamespace(topic_quality=())

    write_ack = getattr(runtime_script, "_write_final_protocol_ack", None)
    assert callable(write_ack), "runtime needs a final sensor and I/O boundary"

    write_ack(Runtime(), Transport(), Logger(), final_ack)

    assert order == [
        "sensor",
        "logger",
        "transport_idle",
        "snapshot",
        "frozen",
    ]
    assert json.loads(final_ack.read_text(encoding="utf-8")) == {"active": True}


def test_final_protocol_ack_rejects_inactive_snapshot_without_unfreezing(
    tmp_path,
) -> None:
    """final transport 非 active 时不得写成功 ACK 或解除 sensor fence。"""
    final_ack = tmp_path / "new_command.ack"
    completed = False

    class Runtime:
        def begin_sensor_fence(self) -> object:
            return object()

        def complete_sensor_fence(self, *_args, **_kwargs) -> None:
            nonlocal completed
            completed = True

    class Logger:
        def snapshot(self) -> InterfaceLogSnapshot:
            return InterfaceLogSnapshot(1, 0, 0, 0, False, False, 0)

    class Transport:
        def wait_idle(self, *, timeout_sec: float) -> None:
            assert timeout_sec == runtime_script._TRANSPORT_IDLE_TIMEOUT_SEC

        def snapshot(self) -> object:
            return SimpleNamespace(
                topic_quality=(
                    SimpleNamespace(state="waiting_peer", peer_connected=False),
                )
            )

    with pytest.raises(RuntimeError, match="final protocol transport snapshot"):
        runtime_script._write_final_protocol_ack(
            Runtime(),
            Transport(),
            Logger(),
            final_ack,
        )

    assert not final_ack.exists()
    assert completed is False


def test_final_protocol_ack_rejects_closed_logger_before_transport(
    tmp_path,
) -> None:
    """logger 已关闭时不得继续 transport 边界或写 final ACK。"""
    final_ack = tmp_path / "new_command.ack"
    order: list[str] = []

    class Runtime:
        def begin_sensor_fence(self) -> object:
            order.append("sensor")
            return object()

        def complete_sensor_fence(self, *_args, **_kwargs) -> None:
            order.append("complete")

    class Logger:
        def snapshot(self) -> InterfaceLogSnapshot:
            order.append("logger")
            return InterfaceLogSnapshot(1, 0, 0, 0, True, False, 0)

    class Transport:
        def wait_idle(self, *, timeout_sec: float) -> None:
            order.append("transport_idle")

        def snapshot(self) -> object:
            order.append("snapshot")
            return SimpleNamespace(topic_quality=())

    with pytest.raises(RuntimeError, match="logger is closed"):
        runtime_script._write_final_protocol_ack(
            Runtime(),
            Transport(),
            Logger(),
            final_ack,
        )

    assert order == ["sensor", "logger"]
    assert not final_ack.exists()


def test_simulation_peer_recreates_command_publisher_with_same_shm_ring() -> None:
    """初始连接和重连都必须创建独立、同配置的 command publisher。"""
    calls: list[tuple[str, type, int, int]] = []

    class Bindings:
        def create_publisher(
            self,
            topic: str,
            message_type: type,
            *,
            shm_buffer_count: int,
            acknowledge_timeout_ms: int,
        ) -> object:
            calls.append(
                (
                    topic,
                    message_type,
                    shm_buffer_count,
                    acknowledge_timeout_ms,
                )
            )
            return object()

    create_publisher = getattr(
        peer_script,
        "_create_command_publisher",
        None,
    )
    assert callable(create_publisher), "peer needs one command publisher factory"

    config = peer_script.InterfaceConfig.default(transport_mode="ecal")
    bindings = Bindings()
    initial = create_publisher(bindings, config)
    reconnected = create_publisher(bindings, config)

    assert initial is not reconnected
    assert calls == [
        (
            config.wheel_command.topic,
            peer_script.pb.WheelCommand,
            config.outgoing_queue_size,
            100,
        ),
        (
            config.wheel_command.topic,
            peer_script.pb.WheelCommand,
            config.outgoing_queue_size,
            100,
        ),
    ]


def test_simulation_peer_main_flow_uses_sim_window_and_delivery_fence() -> None:
    """peer 入口必须消费完整 ACK，并按 runtime 仿真边界等待、筛选输出。"""
    source = inspect.getsource(peer_script.run_simulation_peer)

    assert source.count("_create_command_publisher(") == 2
    assert "_wait_for_json_object(" in source
    assert "_complete_measurement_window(" in source
    assert "_wait_for_output_fence(" in source
    assert "_events_in_sim_window(" in source
    assert "_payload_evidence(" in source
    assert "_rtk_position_evidence(" in source
    assert '"requested_duration_sec"' in source
    assert '"peer_measurement_duration_sec"' in source
    assert '"wheel_drain_complete"' in source
    assert '"wheel_drain_timestamp_ns"' in source
    assert source.index("_complete_measurement_window(") < source.index(
        "_wait_for_output_fence("
    )
    assert "time.sleep(0.03)" not in source


def test_normal_load_deadline_stops_physics_while_complete_marker_is_late() -> None:
    """正式 deadline 后即使 marker 晚到，也不能多推进一个 4.167 ms 物理帧。"""
    should_step = getattr(runtime_script, "_normal_load_physics_step_due", None)
    assert callable(should_step), "runtime needs a pre-step normal-load deadline gate"
    tracker = SimpleNamespace(start_wall_time=10.0)

    assert should_step(tracker, 5.0, now=14.999) is True
    assert should_step(tracker, 5.0, now=15.000) is False
    assert should_step(tracker, 5.0, now=15.004_167) is False


def test_wheel_drain_fence_pauses_after_one_post_window_data_frame() -> None:
    """结束边界后只排出一条 wheel 帧，下一协议 marker 到达前不能持续推进。"""
    gate_type = getattr(runtime_script, "_WheelDrainFenceGate", None)
    assert gate_type is not None, "runtime needs an explicit one-frame drain gate"
    gate = gate_type(end_sim_time_ns=500)

    assert gate.physics_step_due(next_protocol_ready=False) is True
    gate.observe_wheel_states(())
    gate.observe_wheel_states((SimpleNamespace(timestamp_ns=500),))
    assert gate.physics_step_due(next_protocol_ready=False) is True

    gate.observe_wheel_states((SimpleNamespace(timestamp_ns=510),))
    assert gate.delivered is True
    assert gate.physics_step_due(next_protocol_ready=False) is False
    assert gate.physics_step_due(next_protocol_ready=True) is True
    assert gate.released is True
    assert gate.physics_step_due(next_protocol_ready=False) is True


def test_wheel_drain_release_requires_complete_marker_and_rebases_pacer(
    tmp_path,
) -> None:
    """半写 marker 不能解除暂停；完整 JSON 首次解除时必须重建 deadline。"""
    marker = tmp_path / "invalid.active"
    marker.write_text('{"timestamp_ns":', encoding="utf-8")
    gate = runtime_script._WheelDrainFenceGate(end_sim_time_ns=500)
    gate.observe_wheel_states((SimpleNamespace(timestamp_ns=510),))
    resets: list[str] = []
    pacer = SimpleNamespace(start=lambda: resets.append("reset"))
    release = getattr(runtime_script, "_wheel_drain_physics_step_due", None)

    assert callable(release), "runtime needs one complete-marker fence release helper"
    assert release(gate, marker=marker, pacer=pacer) is False
    assert gate.released is False
    assert resets == []

    marker.write_text('{"timestamp_ns":9000000000000}', encoding="utf-8")
    assert release(gate, marker=marker, pacer=pacer) is True
    assert gate.released is True
    assert resets == ["reset"]
    assert release(gate, marker=marker, pacer=pacer) is True
    assert resets == ["reset"]


def test_simulation_runtime_waits_for_shared_start_signal(monkeypatch, tmp_path):
    """runtime 不能在双进程共同 start 门闩打开前发布或推进物理。"""
    start_file = tmp_path / "start.signal"
    stop_file = tmp_path / "stop.signal"
    now = {"value": 0.0}

    def sleep(duration):
        now["value"] += duration
        if now["value"] >= 0.02:
            start_file.write_text("start\n", encoding="utf-8")

    monkeypatch.setattr(
        runtime_script,
        "time",
        SimpleNamespace(monotonic=lambda: now["value"], sleep=sleep),
    )

    runtime_script._wait_for_start_signal(
        start_file,
        stop_file,
        timeout_sec=0.1,
    )

    assert now["value"] >= 0.02


def test_simulation_runtime_retries_marker_until_json_write_is_complete(tmp_path):
    """marker 文件已创建但 JSON 尚未写完时，本轮不得终止仿真进程。"""
    marker = tmp_path / "drop.active"
    marker.write_text('{"topic":', encoding="utf-8")

    assert runtime_script._read_marker(marker) is None

    marker.write_text('{"topic":"/sim/imu/attitude"}', encoding="utf-8")
    assert runtime_script._read_marker(marker) == {
        "topic": "/sim/imu/attitude"
    }


def test_simulation_runtime_treats_marker_removed_after_exists_as_absent(
    monkeypatch,
    tmp_path,
):
    """peer 收到 ack 后可并发删 marker，runtime 本轮应当安静重试。"""
    marker = tmp_path / "drop.active"
    marker.write_text('{"topic":"/sim/wheel/state"}', encoding="utf-8")
    original_read_text = type(marker).read_text

    def remove_before_read(path, *args, **kwargs):
        if path == marker:
            marker.unlink()
            raise FileNotFoundError(marker)
        return original_read_text(path, *args, **kwargs)

    monkeypatch.setattr(type(marker), "read_text", remove_before_read)

    assert runtime_script._read_marker(marker) is None


def test_simulation_runtime_captures_normal_load_snapshot_before_faults(tmp_path):
    """正常窗口快照必须由显式 marker/ack 锁定，不能混入后续断线故障。"""
    marker = tmp_path / "measurement_complete.active"
    ack = tmp_path / "measurement_complete.ack"
    snapshot = SimpleNamespace(
        published_count=200,
        received_count=100,
        dropped_count=0,
        error_count=0,
        topic_quality=(
            SimpleNamespace(
                topic="/sim/wheel/state",
                dropped_count=0,
                error_count=0,
                state="active",
                detail="",
                peer_connected=True,
            ),
        ),
    )

    class SnapshotTransport:
        def __init__(self):
            self.wait_idle_count = 0
            self.snapshot_count = 0

        def wait_idle(self, *, timeout_sec):
            assert timeout_sec > 0.0
            self.wait_idle_count += 1

        def snapshot(self):
            self.snapshot_count += 1
            return snapshot

    transport = SnapshotTransport()
    assert (
        runtime_script._capture_marked_transport_snapshot(transport, marker, ack)
        is None
    )
    assert transport.snapshot_count == 0
    assert transport.wait_idle_count == 0

    marker.write_text("{}", encoding="utf-8")
    captured = runtime_script._capture_marked_transport_snapshot(
        transport,
        marker,
        ack,
    )

    assert captured is snapshot
    assert transport.wait_idle_count == 1
    assert transport.snapshot_count == 1
    assert json.loads(ack.read_text(encoding="utf-8")) == {
        "published_count": 200,
        "received_count": 100,
        "dropped_count": 0,
        "error_count": 0,
        "topic_quality": {
            "/sim/wheel/state": {
                "detail": "",
                "dropped_count": 0,
                "error_count": 0,
                "peer_connected": True,
                "state": "active",
            },
        },
    }


def test_marked_transport_snapshot_starts_window_before_ack(
    monkeypatch,
    tmp_path,
):
    """正式窗口起点必须先于 ACK，避免 peer 抢先发送窗口内命令。"""
    marker = tmp_path / "measurement_start.active"
    ack = tmp_path / "measurement_start.ack"
    marker.write_text("{}", encoding="utf-8")
    order: list[str] = []
    snapshot = SimpleNamespace(
        published_count=20,
        received_count=10,
        dropped_count=0,
        error_count=0,
        topic_quality=(
            SimpleNamespace(
                topic=WHEEL_STATE,
                dropped_count=0,
                error_count=0,
                state="active",
                detail="",
                peer_connected=True,
            ),
        ),
    )

    class SnapshotTransport:
        def wait_idle(self, *, timeout_sec):
            assert timeout_sec > 0.0
            order.append("idle")

        def snapshot(self):
            order.append("snapshot")
            return snapshot

    def write_ack(path: Path, payload: object = None) -> None:
        order.append("ack")
        path.write_text(json.dumps(payload), encoding="utf-8")

    def capture_start() -> None:
        assert not ack.exists()
        order.append("capture")

    monkeypatch.setattr(runtime_script, "_write_ack", write_ack)

    captured = runtime_script._capture_marked_transport_snapshot(
        SnapshotTransport(),
        marker,
        ack,
        before_ack=capture_start,
        ack_fields={"window_start_sim_time_ns": 1_000_000_000},
    )

    assert captured is snapshot
    assert order == ["idle", "snapshot", "capture", "ack"]
    payload = json.loads(ack.read_text(encoding="utf-8"))
    assert payload["window_start_sim_time_ns"] == 1_000_000_000


def test_measurement_start_fence_prevents_warmup_lidar_from_crossing_snapshot(
    monkeypatch,
    tmp_path,
) -> None:
    """warmup prepared 帧必须先进入日志，再捕获 start 快照并写 ACK。"""
    marker = tmp_path / "measurement_start.active"
    ack = tmp_path / "measurement_start.ack"
    marker.write_text("{}", encoding="utf-8")
    order: list[str] = []
    prepared_visible = False
    log_snapshot = InterfaceLogSnapshot(1, 0, 0, 0, False, False, 0)
    transport_snapshot = SimpleNamespace(
        published_count=1,
        received_count=0,
        dropped_count=0,
        error_count=0,
        topic_quality=(
            SimpleNamespace(
                topic=CONFIG.lidar_front.topic,
                dropped_count=0,
                error_count=0,
                state="active",
                detail="",
                peer_connected=True,
            ),
        ),
    )

    class Runtime:
        def begin_sensor_fence(self) -> object:
            nonlocal prepared_visible
            order.extend(("sensor", "prepared_publish"))
            prepared_visible = True
            return object()

        def complete_sensor_fence(self, _fence: object, *, resume_capture: bool) -> None:
            assert resume_capture is True
            assert ack.exists()
            order.append("resume")

    class Logger:
        def snapshot(self) -> InterfaceLogSnapshot:
            assert prepared_visible
            order.append("logger_idle")
            return log_snapshot

    class Transport:
        def wait_idle(self, *, timeout_sec: float) -> None:
            assert timeout_sec == runtime_script._TRANSPORT_IDLE_TIMEOUT_SEC
            order.append("transport_idle")

        def snapshot(self) -> object:
            assert order[-1] == "transport_idle"
            order.append("snapshot")
            return transport_snapshot

    def write_ack(path: Path, payload: object = None) -> None:
        order.append("ack")
        path.write_text(json.dumps(payload), encoding="utf-8")

    capture_start = getattr(runtime_script, "_capture_normal_load_start", None)
    assert callable(capture_start), "runtime needs one measurement-start sensor fence"
    monkeypatch.setattr(runtime_script, "_write_ack", write_ack)

    captured = capture_start(
        Runtime(),
        Transport(),
        Logger(),
        marker,
        ack,
        before_ack=lambda: order.append("window_start"),
        ack_fields={"window_start_sim_time_ns": 1_000_000_000},
    )

    assert captured.log_snapshot is log_snapshot
    assert captured.transport_snapshot is transport_snapshot
    assert order == [
        "sensor",
        "prepared_publish",
        "logger_idle",
        "transport_idle",
        "snapshot",
        "window_start",
        "ack",
        "resume",
    ]


def test_sensor_fence_timeout_prevents_success_ack(tmp_path) -> None:
    """sensor drain 超时后不得继续 logger/transport 或写成功 ACK。"""
    marker = tmp_path / "measurement_start.active"
    ack = tmp_path / "measurement_start.ack"
    marker.write_text("{}", encoding="utf-8")
    order: list[str] = []

    class Runtime:
        def begin_sensor_fence(self) -> object:
            order.append("sensor")
            raise TimeoutError("sensor fence did not become idle within 250 ms")

        def complete_sensor_fence(self, *_args, **_kwargs) -> None:
            raise AssertionError("timed-out fence must not resume")

    class Logger:
        def snapshot(self) -> object:
            raise AssertionError("logger must not run after sensor timeout")

    class Transport:
        def wait_idle(self, **_kwargs) -> None:
            raise AssertionError("transport must not run after sensor timeout")

        def snapshot(self) -> object:
            raise AssertionError("snapshot must not run after sensor timeout")

    with pytest.raises(TimeoutError, match="250 ms"):
        runtime_script._capture_normal_load_start(
            Runtime(),
            Transport(),
            Logger(),
            marker,
            ack,
        )

    assert order == ["sensor"]
    assert not ack.exists()


def test_fence_does_not_resume_previously_suspended_service(tmp_path) -> None:
    """脚本只回传 opaque token，由 runtime 保留进入前 suspended 状态。"""
    marker = tmp_path / "measurement_start.active"
    ack = tmp_path / "measurement_start.ack"
    marker.write_text("{}", encoding="utf-8")
    token = object()
    log_snapshot = InterfaceLogSnapshot(0, 0, 0, 0, False, False, 0)
    transport_snapshot = SimpleNamespace(
        published_count=0,
        received_count=0,
        dropped_count=0,
        error_count=0,
        topic_quality=(),
    )

    class Runtime:
        capture_enabled = False

        def begin_sensor_fence(self) -> object:
            return token

        def complete_sensor_fence(self, fence: object, *, resume_capture: bool) -> None:
            assert ack.exists()
            assert fence is token
            assert resume_capture is True
            # fake token 表示进入前 suspended，runtime 因此不实际解门。
            self.capture_enabled = False

        def resume(self) -> None:
            raise AssertionError("marker helper must not bypass the fence token")

    class Logger:
        def snapshot(self) -> InterfaceLogSnapshot:
            return log_snapshot

    class Transport:
        def wait_idle(self, *, timeout_sec: float) -> None:
            assert timeout_sec == runtime_script._TRANSPORT_IDLE_TIMEOUT_SEC

        def snapshot(self) -> object:
            return transport_snapshot

    runtime = Runtime()
    runtime_script._capture_normal_load_start(
        runtime,
        Transport(),
        Logger(),
        marker,
        ack,
    )

    assert ack.exists()
    assert runtime.capture_enabled is False


def test_measurement_start_rebases_pacer_before_capturing_wall_time() -> None:
    """正式窗口不能继承 transport/logger 屏障积累的墙钟欠债。"""
    order: list[str] = []

    class RecordingPacer:
        def reset_deadline(self) -> None:
            order.append("reset")

    begin_window = getattr(runtime_script, "_begin_normal_load_motion_window", None)
    assert callable(begin_window), "runtime needs an explicit measurement-start boundary"

    tracker = begin_window(
        pacer=RecordingPacer(),
        start_step_count=10,
        start_sim_time_ns=1_000_000_000,
        physics_time_step_sec=1.0 / 240.0,
        start_position_m=(0.0, 0.0, 0.0),
        monotonic=lambda: order.append("capture") or 20.0,
    )

    assert order == ["reset", "capture"]
    assert tracker.start_wall_time == 20.0


def test_measurement_start_log_sample_uses_capture_snapshot() -> None:
    """正式窗口首个日志样本必须使用同一 start capture 的 logger 快照。"""
    source = inspect.getsource(runtime_script.run_simulation_runtime)

    assert "start_capture.log_snapshot.pending_count" in source
    assert "start_capture.log_snapshot.accepted_messages" in source
    assert "start_capture.log_snapshot.accepted_events" in source
    assert "log_start." not in source


def test_simulation_runtime_does_not_ack_measurement_until_every_peer_is_active(
    tmp_path,
):
    """六话题 discovery 未全部稳定前，不得确认测量基线。"""
    marker = tmp_path / "measurement_start.active"
    ack = tmp_path / "measurement_start.ack"
    marker.write_text("{}", encoding="utf-8")
    topics = tuple(EXPECTED_TYPES)

    class DiscoveryTransport:
        def wait_idle(self, *, timeout_sec):
            assert timeout_sec > 0.0

        def snapshot(self):
            return SimpleNamespace(
                dropped_count=0,
                error_count=0,
                topic_quality=tuple(
                    SimpleNamespace(
                        topic=topic,
                        dropped_count=0,
                        error_count=0,
                        state="active" if index else "degraded",
                        detail="" if index else "discovery pending",
                        peer_connected=bool(index),
                    )
                    for index, topic in enumerate(topics)
                ),
            )

    captured = runtime_script._capture_marked_transport_snapshot(
        DiscoveryTransport(),
        marker,
        ack,
    )

    assert captured is None
    assert not ack.exists()


def test_capture_normal_load_end_freezes_log_transport_and_sim_boundary(tmp_path):
    """结束 ACK、日志快照和 transport 快照必须来自同一个停止步进屏障。"""
    marker = tmp_path / "measurement_complete.active"
    ack = tmp_path / "measurement_complete.ack"
    marker.write_text("{}", encoding="utf-8")
    order: list[str] = []
    window_log_snapshot = InterfaceLogSnapshot(1_200, 0, 0, 0, False, False, 8)
    final_log_snapshot = InterfaceLogSnapshot(1_200, 0, 0, 0, False, False, 0)
    transport_snapshot = SimpleNamespace(
        published_count=200,
        received_count=100,
        dropped_count=0,
        error_count=0,
        topic_quality=(
            SimpleNamespace(
                topic=WHEEL_STATE,
                dropped_count=0,
                error_count=0,
                state="active",
                detail="",
                peer_connected=True,
            ),
        ),
    )

    class IdleLogger:
        def __init__(self) -> None:
            self._snapshots = iter((window_log_snapshot, final_log_snapshot))

        def snapshot(self):
            order.append("logger")
            return next(self._snapshots)

    class IdleTransport:
        def wait_idle(self, *, timeout_sec):
            assert timeout_sec > 0.0
            order.append("idle")

        def snapshot(self):
            order.append("transport")
            return transport_snapshot

    capture_end = getattr(runtime_script, "_capture_normal_load_end", None)
    assert callable(capture_end), "runtime needs one atomic normal-load end boundary"

    captured = capture_end(
        IdleTransport(),
        IdleLogger(),
        marker,
        ack,
        end_step_count=1_200,
        end_sim_time_ns=5_000_000_000,
        end_wall_time=10.0,
        end_position_m=(1.0, 0.0, 0.5),
    )

    assert order == ["logger", "logger", "idle", "transport"]
    assert captured.step_count == 1_200
    assert captured.log_queue_sample == (8, 1_192)
    assert captured.log_snapshot is final_log_snapshot
    assert captured.transport_snapshot is transport_snapshot
    assert json.loads(ack.read_text(encoding="utf-8"))[
        "window_end_sim_time_ns"
    ] == 5_000_000_000


def test_measurement_end_ack_resumes_post_window_protocol(
    monkeypatch,
    tmp_path,
) -> None:
    """end fence 先收敛传感器，且只能在边界 ACK 后恢复 capture。"""
    marker = tmp_path / "measurement_complete.active"
    ack = tmp_path / "measurement_complete.ack"
    marker.write_text("{}", encoding="utf-8")
    order: list[str] = []
    prepared_visible = False
    snapshots = iter(
        (
            InterfaceLogSnapshot(2, 0, 0, 0, False, False, 1),
            InterfaceLogSnapshot(2, 0, 0, 0, False, False, 0),
        )
    )
    transport_snapshot = SimpleNamespace(
        published_count=2,
        received_count=0,
        dropped_count=0,
        error_count=0,
        topic_quality=(
            SimpleNamespace(
                topic=CONFIG.lidar_front.topic,
                dropped_count=0,
                error_count=0,
                state="active",
                detail="",
                peer_connected=True,
            ),
        ),
    )

    class Runtime:
        def begin_sensor_fence(self) -> object:
            nonlocal prepared_visible
            order.extend(("sensor", "prepared_publish"))
            prepared_visible = True
            return object()

        def complete_sensor_fence(self, _fence: object, *, resume_capture: bool) -> None:
            assert resume_capture is True
            assert ack.exists()
            order.append("resume")

    class Logger:
        def snapshot(self) -> InterfaceLogSnapshot:
            assert prepared_visible
            order.append("logger")
            return next(snapshots)

    class Transport:
        def wait_idle(self, *, timeout_sec: float) -> None:
            assert timeout_sec == runtime_script._TRANSPORT_IDLE_TIMEOUT_SEC
            order.append("transport_idle")

        def snapshot(self) -> object:
            order.append("snapshot")
            return transport_snapshot

    def write_ack(path: Path, payload: object = None) -> None:
        order.append("ack")
        path.write_text(json.dumps(payload), encoding="utf-8")

    monkeypatch.setattr(runtime_script, "_write_ack", write_ack)

    runtime_script._capture_normal_load_end(
        Transport(),
        Logger(),
        marker,
        ack,
        runtime=Runtime(),
        end_step_count=2,
        end_sim_time_ns=100_000_000,
        end_wall_time=1.0,
        end_position_m=(0.0, 0.0, 0.0),
    )

    assert order == [
        "sensor",
        "prepared_publish",
        "logger",
        "logger",
        "transport_idle",
        "snapshot",
        "ack",
        "resume",
    ]


def test_measurement_end_rejects_inactive_snapshot_before_committing_boundary(
    tmp_path,
) -> None:
    """结束屏障不稳定时必须硬失败，不能带旧边界继续步进后再重试。"""
    marker = tmp_path / "measurement_complete.active"
    ack = tmp_path / "measurement_complete.ack"
    marker.write_text("{}", encoding="utf-8")
    log_snapshot = InterfaceLogSnapshot(0, 0, 0, 0, False, False, 0)

    class IdleLogger:
        def snapshot(self):
            return log_snapshot

    class InactiveTransport:
        def wait_idle(self, *, timeout_sec):
            assert timeout_sec > 0.0

        def snapshot(self):
            return SimpleNamespace(
                published_count=0,
                received_count=0,
                dropped_count=0,
                error_count=0,
                topic_quality=(
                    SimpleNamespace(
                        topic=WHEEL_STATE,
                        dropped_count=0,
                        error_count=0,
                        state="waiting_peer",
                        detail="peer not active",
                        peer_connected=False,
                    ),
                ),
            )

    capture_end = getattr(runtime_script, "_capture_normal_load_end", None)
    assert callable(capture_end), "runtime needs one atomic normal-load end boundary"

    with pytest.raises(RuntimeError, match="normal-load end.*active"):
        capture_end(
            InactiveTransport(),
            IdleLogger(),
            marker,
            ack,
            end_step_count=1_200,
            end_sim_time_ns=5_000_000_000,
            end_wall_time=10.0,
            end_position_m=(1.0, 0.0, 0.5),
        )

    assert not ack.exists()


def test_simulation_runtime_does_not_snapshot_or_ack_when_idle_wait_times_out(
    tmp_path,
):
    """marker 屏障超时必须显式失败，不能留下错误基线。"""
    marker = tmp_path / "measurement_complete.active"
    ack = tmp_path / "measurement_complete.ack"
    marker.write_text("{}", encoding="utf-8")

    class BlockedTransport:
        def wait_idle(self, *, timeout_sec):
            raise TimeoutError(f"transport did not become idle within {timeout_sec}s")

        def snapshot(self):
            raise AssertionError("snapshot must not run after idle timeout")

    with pytest.raises(TimeoutError, match="did not become idle"):
        runtime_script._capture_marked_transport_snapshot(
            BlockedTransport(),
            marker,
            ack,
        )

    assert not ack.exists()


def test_simulation_runtime_transport_delta_excludes_discovery_warmup_counts():
    """正常门禁只计算 measurement start/end 之间的累计量增量。"""
    topic = "/sim/wheel/state"
    start = SimpleNamespace(
        published_count=101,
        received_count=79,
        dropped_count=7,
        error_count=3,
        topic_quality=(
            SimpleNamespace(
                topic=topic,
                dropped_count=7,
                error_count=3,
                state="degraded",
                detail="warmup",
                peer_connected=False,
            ),
        ),
    )
    end = SimpleNamespace(
        published_count=301,
        received_count=179,
        dropped_count=9,
        error_count=4,
        topic_quality=(
            SimpleNamespace(
                topic=topic,
                dropped_count=9,
                error_count=4,
                state="active",
                detail="",
                peer_connected=True,
            ),
        ),
    )

    assert runtime_script._transport_snapshot_delta(start, end) == {
        "published_count": 200,
        "received_count": 100,
        "dropped_count": 2,
        "error_count": 1,
        "topic_quality": {
            topic: {
                "detail": "",
                "dropped_count": 2,
                "error_count": 1,
                "peer_connected": True,
                "state": "active",
            },
        },
    }


def test_simulation_measurement_uses_runtime_received_commands_not_peer_sends():
    """命令门禁必须读取 runtime 接收日志，peer 自报 100 Hz 不能冒充实收。"""
    collect = getattr(verifier, "_simulation_measurement_events", None)
    assert callable(collect), "simulation verifier needs a runtime-receive event collector"
    peer_result = {
        "commands": [
            {
                "wall_time": 1.0 + index * 0.01,
                "timestamp_ns": index * 10_000_000,
                "type": EXPECTED_TYPES[WHEEL_COMMAND],
            }
            for index in range(100)
        ],
        "received": {
            topic: [
                {"wall_time": 1.0, "timestamp_ns": 1, "type": EXPECTED_TYPES[topic]},
                {"wall_time": 1.9, "timestamp_ns": 2, "type": EXPECTED_TYPES[topic]},
            ]
            for topic in OUTPUT_TOPICS
        },
    }
    runtime_commands = [
        {
            "wall_time": 1.0 + index * 0.01,
            "timestamp_ns": index * 10_000_000,
            "type": EXPECTED_TYPES[WHEEL_COMMAND],
        }
        for index in range(76)
    ]

    events = collect(
        peer_result,
        {"normal_load_received_commands": runtime_commands},
        command_topic=WHEEL_COMMAND,
        output_topics=OUTPUT_TOPICS,
    )

    assert events[WHEEL_COMMAND] == runtime_commands
    assert len(events[WHEEL_COMMAND]) == 76


def test_runtime_log_uses_wall_window_for_commands_and_sim_window_for_outputs() -> None:
    """命令墙钟合同保留，输出则必须与 peer 共享 `(start, end]` 仿真边界。"""
    records = (
        InterfaceLogRecord(
            sequence=1,
            topic=WHEEL_STATE,
            direction="publish",
            sim_time_ns=100,
            wall_time_ns=9_990_000_000,
            type_name=EXPECTED_TYPES[WHEEL_STATE],
            payload=b"start",
        ),
        InterfaceLogRecord(
            sequence=2,
            topic=WHEEL_COMMAND,
            direction="receive",
            sim_time_ns=110,
            wall_time_ns=10_000_000_000,
            type_name=EXPECTED_TYPES[WHEEL_COMMAND],
            payload=b"command",
        ),
        InterfaceLogRecord(
            sequence=3,
            topic=WHEEL_STATE,
            direction="publish",
            sim_time_ns=200,
            wall_time_ns=15_010_000_000,
            type_name=EXPECTED_TYPES[WHEEL_STATE],
            payload=b"end",
        ),
    )

    commands, published, contiguous = runtime_script._measurement_log_events(
        records,
        measurement_start=10.0,
        measurement_end=15.0,
        start_sim_time_ns=100,
        end_sim_time_ns=200,
        command_topic=WHEEL_COMMAND,
        output_topics=(WHEEL_STATE,),
    )

    assert [event["timestamp_ns"] for event in commands] == [110]
    assert [event["timestamp_ns"] for event in published[WHEEL_STATE]] == [200]
    assert contiguous is True


def test_end_to_end_timestamp_match_rejects_loss_duplicates_and_reordering():
    """两端数量相近仍不够，生产与消费 timestamp 必须逐条严格一致。"""
    matches = getattr(verifier, "_timestamp_sequences_match", None)
    assert callable(matches), "roundtrip verifier needs an exact timestamp oracle"

    produced = [10_000_000, 20_000_000, 30_000_000]
    assert matches(produced, produced) is True
    assert matches(produced, produced[:-1]) is False
    assert matches(produced, [10_000_000, 20_000_000, 20_000_000]) is False
    assert matches(produced, [10_000_000, 30_000_000, 20_000_000]) is False


@pytest.mark.parametrize("invalid", (True, 1.5, "10000000"))
def test_end_to_end_timestamp_match_rejects_non_integer_json_values(
    invalid: object,
) -> None:
    """外部 JSON 的 bool、浮点和字符串不能经 int() 冒充纳秒时间戳。"""
    with pytest.raises(AssertionError, match="timestamp"):
        verifier._timestamp_sequences_match([invalid], [invalid])


@pytest.mark.parametrize(
    ("field_name", "invalid"),
    (
        ("wall_time", True),
        ("wall_time", "1.0"),
        ("wall_time", float("nan")),
        ("timestamp_ns", False),
        ("timestamp_ns", 1.5),
        ("timestamp_ns", "100"),
        ("type", 1),
        ("type", ""),
    ),
)
def test_external_event_evidence_rejects_loosely_typed_json(
    field_name: str,
    invalid: object,
) -> None:
    """事件墙钟、仿真时间戳和类型名必须在进入统计前完成严格解析。"""
    event = {"wall_time": 1.0, "timestamp_ns": 100, "type": "example.Type"}
    event[field_name] = invalid
    parser = getattr(verifier, "_strict_event_evidence", None)

    assert callable(parser), "verifier needs one strict external-event parser"
    with pytest.raises(AssertionError):
        parser([event], "peer event")


@pytest.mark.parametrize("invalid", (1, 0, "true", None))
def test_external_boolean_mapping_rejects_truthy_json_substitutes(
    invalid: object,
) -> None:
    """连接和隔离映射只接受 JSON bool，不能依赖 Python 的 1 == True。"""
    parser = getattr(verifier, "_strict_bool_mapping", None)

    assert callable(parser), "verifier needs one strict boolean-mapping parser"
    with pytest.raises(AssertionError, match="boolean"):
        parser({"states": {WHEEL_STATE: invalid}}, "states", "peer states")


def test_maximum_event_gap_includes_window_edges_and_mid_window_stalls():
    """首末均值正常也不能掩盖窗口边缘或中段 500 ms 停顿。"""
    maximum_gap = getattr(verifier, "_maximum_event_gap_sec", None)
    assert callable(maximum_gap), "roundtrip verifier needs an inter-arrival gap oracle"

    assert maximum_gap([1.10, 1.20, 1.70, 1.90], 1.0, 2.0) == pytest.approx(0.5)


def test_simulation_output_gap_uses_each_topics_received_wall_window() -> None:
    """按仿真窗合法筛入的输出可早于命令起点，不能因此被误判为无限 gap。"""
    summarize = getattr(verifier, "_simulation_max_interarrival_gaps", None)
    assert callable(summarize), "simulation verifier needs per-domain gap windows"
    evidence = {
        WHEEL_COMMAND: ([10.00, 10.01, 10.02], [10_000_000, 20_000_000, 30_000_000]),
        WHEEL_STATE: ([9.99, 10.00, 10.01], [10_000_000, 200_000_000, 390_000_000]),
        CONFIG.rtk.topic: ([10.02, 10.12, 10.22], [100_000_000, 200_000_000, 300_000_000]),
    }

    gaps = summarize(
        evidence,
        command_topic=WHEEL_COMMAND,
        command_wall_window=(10.00, 10.02),
        output_sim_window_ns=(0, 400_000_000),
    )

    assert gaps[WHEEL_COMMAND] == pytest.approx(0.01)
    assert gaps[WHEEL_STATE] == pytest.approx(0.01)
    assert gaps[CONFIG.rtk.topic] == pytest.approx(0.10)


def test_simulation_output_gap_keeps_simulation_window_edge_coverage() -> None:
    """输出 callback 墙钟无统一起点时，仍须用仿真时间戳抓住窗口首尾缺帧。"""
    gaps = verifier._simulation_max_interarrival_gaps(
        {
            WHEEL_COMMAND: ([10.0, 10.01], [10_000_000, 20_000_000]),
            WHEEL_STATE: ([10.0, 10.01], [500_000_000, 510_000_000]),
        },
        command_topic=WHEEL_COMMAND,
        command_wall_window=(10.0, 10.01),
        output_sim_window_ns=(0, 1_000_000_000),
    )

    assert gaps[WHEEL_STATE] == pytest.approx(0.50)


def test_roundtrip_gate_rejects_long_gap_despite_full_count_and_average_rate():
    """完整总数与正确首末均值仍必须拒绝超出周期预算的中段停顿。"""
    duration = 5.0
    target_rates = {channel.topic: float(channel.rate_hz) for channel in CONFIG.channels}
    result = SimpleNamespace(
        transport_name="ecal",
        peer_returncode=0,
        received_topics=OUTPUT_TOPICS,
        topic_types=EXPECTED_TYPES,
        dropped_count=0,
        peer_dropped_count=0,
        transport_error_count=0,
        peer_error_count=0,
        duration_sec=duration,
        message_counts={topic: round(duration * rate) for topic, rate in target_rates.items()},
        event_span_sec={topic: duration * 0.98 for topic in target_rates},
        wall_clock_hz=target_rates,
        message_timestamp_hz=target_rates,
        max_interarrival_gap_sec={
            topic: (0.5 if topic == WHEEL_COMMAND else 1.0 / rate)
            for topic, rate in target_rates.items()
        },
        end_to_end_timestamp_match={topic: True for topic in target_rates},
    )

    with pytest.raises(AssertionError, match="gap"):
        verifier._assert_roundtrip_result(result, config=CONFIG, codec=CODEC)


def test_simulation_runtime_detects_isolated_peer_loss_under_visible_topic_fault():
    """故障状态可覆盖等待文案，但 transport peer 位仍须驱动断连握手。"""
    isolated = getattr(runtime_script, "_output_peer_isolated", None)
    assert callable(isolated), "simulation runtime needs a transport-level peer helper"
    snapshot = SimpleNamespace(
        topic_quality=(
            SimpleNamespace(
                topic=topic,
                state="error" if topic == CONFIG.lidar_front.topic else "active",
                peer_connected=topic != CONFIG.lidar_front.topic,
            )
            for topic in OUTPUT_TOPICS
        ),
    )

    assert isolated(
        snapshot,
        CONFIG.lidar_front.topic,
        OUTPUT_TOPICS,
    ) is True


def test_simulation_runtime_reads_command_peer_under_visible_topic_fault():
    """重连等待证据来自 command discovery，不依赖状态页当前显示文案。"""
    connected = getattr(runtime_script, "_topic_peer_connected", None)
    assert callable(connected), "simulation runtime needs a topic peer helper"
    snapshot = SimpleNamespace(
        topic_quality=(
            SimpleNamespace(
                topic=WHEEL_COMMAND,
                state="error",
                peer_connected=True,
            ),
        ),
    )

    assert connected(snapshot, WHEEL_COMMAND) is True


def _valid_simulation_gate_evidence() -> SimpleNamespace:
    """构造包含真实联合负载和 mailbox 代际的完整仿真门禁证据。"""
    expected_topics = set(EXPECTED_TYPES)
    time_step_sec = 1.0 / 240.0
    step_count = 1_200
    control_step_count = 1_198
    controlled_step_count = 1_100
    controlled_path_length_m = 1.5
    base_path_length_m = 1.5
    result = verifier.RoundtripResult(
        transport_name="ecal",
        peer_returncode=0,
        robot_model="active_steering_4wd",
        wall_clock_hz={topic: 100.0 for topic in expected_topics},
        message_timestamp_hz={topic: 100.0 for topic in expected_topics},
        received_topics=expected_topics,
        topic_types=EXPECTED_TYPES,
        message_counts={topic: 10 for topic in expected_topics},
        dropped_count=0,
        runtime_name="simulation",
        duration_sec=5.0,
        feedback_is_not_command_echo=True,
        invalid_command_rejected=True,
        timeout_stopped_vehicle=True,
        timeout_preserved_steering=True,
        reconnect_required_new_command=True,
        reconnect_generation_advanced=True,
        mailbox_generation_before_disconnect=3,
        mailbox_generation_after_disconnect=4,
        clean_shutdown=True,
        output_disconnect_isolated={topic: True for topic in OUTPUT_TOPICS},
        per_topic_peer_states={topic: "active" for topic in expected_topics},
        normal_load_obstacle_count=20,
        normal_load_log_sample_count=50,
        normal_load_log_accepted_messages=3_000,
        normal_load_log_accepted_events=0,
        normal_load_log_max_pending=8,
        normal_load_log_final_pending=0,
        normal_load_log_sustained_backlog=False,
        normal_load_log_dropped_messages=0,
        normal_load_log_dropped_events=0,
        normal_load_log_writer_failed=False,
        normal_load_log_sequence_contiguous=True,
        end_to_end_timestamp_match={topic: True for topic in expected_topics},
        normal_load_requested_duration_sec=5.0,
        peer_measurement_duration_sec=5.0,
        normal_load_physics_time_step_sec=time_step_sec,
        normal_load_step_count=step_count,
        normal_load_sim_duration_sec=step_count * time_step_sec,
        normal_load_wall_duration_sec=5.0,
        normal_load_control_step_count=control_step_count,
        normal_load_control_sim_duration_sec=control_step_count * time_step_sec,
        normal_load_control_wall_duration_sec=4.99,
        normal_load_controlled_motion_step_count=controlled_step_count,
        normal_load_controlled_motion_sim_duration_sec=(
            controlled_step_count * time_step_sec
        ),
        normal_load_obstacle_contact_step_count=0,
        normal_load_controlled_displacement_m=1.4,
        normal_load_controlled_path_length_m=controlled_path_length_m,
        normal_load_controlled_mean_speed_m_s=(
            controlled_path_length_m / (controlled_step_count * time_step_sec)
        ),
        normal_load_controlled_max_speed_m_s=0.5,
        normal_load_warmup_requested_sec=1.0,
        normal_load_warmup_wall_duration_sec=1.0,
        normal_load_warmup_sim_duration_sec=1.0,
        normal_load_warmup_physics_steps=240,
        normal_load_warmup_log_accepted_messages=240,
        normal_load_warmup_topic_counts={topic: 1 for topic in expected_topics},
        normal_load_command_states=("active",),
        normal_load_measurement_wall_duration_sec=5.0,
        normal_load_sim_wall_ratio=1.0,
        normal_load_control_duration_sec=4.99,
        normal_load_rtk_displacement_m=1.4,
        normal_load_base_displacement_m=1.3,
        normal_load_base_path_length_m=base_path_length_m,
        normal_load_base_mean_speed_m_s=(
            base_path_length_m / (step_count * time_step_sec)
        ),
        normal_load_base_max_speed_m_s=0.5,
        normal_load_trajectory_distance_m=1.5,
        normal_load_average_speed_m_s=0.32,
        normal_load_nonzero_drive_feedback_wheels=4,
        normal_load_peak_left_steering_angle_rad=0.55,
        normal_load_peak_right_steering_angle_rad=0.55,
        normal_load_steering_same_sign=True,
        normal_load_peak_steering_angle_rad=0.55,
        peer_rtk_displacement_m=1.4,
        logged_rtk_displacement_m=1.4,
        rtk_log_match_count=50,
        rtk_log_max_position_error_m=0.0,
        normal_load_window_start_sim_time_ns=1_000_000_000,
        normal_load_window_end_sim_time_ns=6_000_000_000,
        wheel_drain_timestamp_ns=6_004_166_667,
        wheel_log_publish_count=500,
        wheel_peer_receive_count=500,
        wheel_log_match_count=500,
        wheel_drain_complete=True,
    )
    return SimpleNamespace(**vars(result))


def test_simulation_gate_accepts_differential_two_plus_zero_joint_evidence() -> None:
    """差速正式门禁必须要求两路驱动反馈，并确认不存在转向关节。"""
    evidence = _valid_simulation_gate_evidence()
    evidence.robot_model = "df_back"
    evidence.normal_load_nonzero_drive_feedback_wheels = 2
    evidence.normal_load_peak_left_steering_angle_rad = 0.0
    evidence.normal_load_peak_right_steering_angle_rad = 0.0
    evidence.normal_load_steering_same_sign = False
    evidence.normal_load_peak_steering_angle_rad = 0.0

    verifier._assert_simulation_result(evidence, config=CONFIG)


@pytest.mark.parametrize(
    ("field_name", "invalid_value"),
    (
        ("normal_load_obstacle_count", 19),
        ("normal_load_log_sample_count", 1),
        ("normal_load_log_accepted_messages", 0),
        ("normal_load_log_final_pending", 1),
        ("normal_load_log_sustained_backlog", True),
        ("normal_load_log_dropped_messages", 1),
        ("normal_load_log_dropped_events", 1),
        ("normal_load_log_writer_failed", True),
        ("normal_load_log_sequence_contiguous", False),
        ("reconnect_generation_advanced", False),
        ("mailbox_generation_after_disconnect", 3),
        ("normal_load_warmup_wall_duration_sec", 0.5),
        ("normal_load_warmup_sim_duration_sec", 0.0),
        ("normal_load_warmup_physics_steps", 0),
        ("normal_load_warmup_log_accepted_messages", 0),
        ("normal_load_warmup_topic_counts", {}),
        ("normal_load_command_states", ("active", "timed_out")),
        ("normal_load_measurement_wall_duration_sec", 4.99),
        ("normal_load_sim_wall_ratio", 0.949),
        ("normal_load_control_duration_sec", 4.0),
        ("normal_load_rtk_displacement_m", 0.0),
        ("normal_load_base_displacement_m", 0.0),
        ("normal_load_trajectory_distance_m", 0.0),
        ("normal_load_average_speed_m_s", 0.0),
        ("normal_load_nonzero_drive_feedback_wheels", 0),
        ("normal_load_peak_left_steering_angle_rad", 0.0),
        ("normal_load_peak_right_steering_angle_rad", 0.0),
        ("normal_load_steering_same_sign", False),
        ("normal_load_peak_steering_angle_rad", 0.0),
    ),
)
def test_simulation_gate_rejects_incomplete_joint_load_or_generation_evidence(
    field_name,
    invalid_value,
):
    """真实门禁不能用空场景、丢日志或自然超时冒充联合验收通过。"""
    evidence = _valid_simulation_gate_evidence()
    setattr(evidence, field_name, invalid_value)

    with pytest.raises(AssertionError):
        verifier._assert_simulation_result(evidence, config=CONFIG)


@pytest.mark.parametrize(
    ("waiting", "timed_out", "drive", "expected"),
    (
        (False, False, (0.11, 0.0), True),
        (False, False, (0.10, -0.10), False),
        (True, False, (2.0, 2.0), False),
        (False, True, (2.0, 2.0), False),
    ),
)
def test_normal_load_active_control_excludes_waiting_and_timeout_decisions(
    waiting: bool,
    timed_out: bool,
    drive: tuple[float, ...],
    expected: bool,
) -> None:
    predicate = getattr(runtime_script, "_is_active_drive_control", None)
    assert callable(predicate), "simulation runtime needs an active-control predicate"
    decision = SimpleNamespace(
        waiting=waiting,
        timed_out=timed_out,
        drive_wheel_speed_rad_s=drive,
    )

    assert predicate(decision) is expected


@pytest.mark.parametrize(
    ("feedback", "expected"),
    (
        ((1.0, 1.5), True),
        ((0.0, 0.0), False),
        ((-1.0, -1.5), False),
    ),
)
def test_controlled_motion_requires_same_direction_joint_feedback(
    feedback: tuple[float, ...],
    expected: bool,
) -> None:
    predicate = getattr(runtime_script, "_is_controlled_motion_step", None)
    assert callable(predicate), "runtime needs a command/feedback causal predicate"
    decision = SimpleNamespace(
        waiting=False,
        timed_out=False,
        drive_wheel_speed_rad_s=(2.0, 2.0),
    )
    wheel_state = SimpleNamespace(drive_wheel_speed_rad_s=feedback)

    assert predicate(decision, wheel_state) is expected


def test_obstacle_contact_detector_only_counts_registered_obstacle_bodies() -> None:
    """地形接触不应污染障碍物推动证据，已注册障碍接触必须命中。"""
    detector = getattr(runtime_script, "_has_obstacle_contact", None)
    assert callable(detector), "runtime needs a pure obstacle-contact predicate"
    robot_id = 10
    contacts = ((0, robot_id, 20), (0, robot_id, 30))

    assert detector(contacts, robot_id=robot_id, obstacle_body_ids={40, 50}) is False
    assert detector(contacts, robot_id=robot_id, obstacle_body_ids={30, 40}) is True


def test_normal_load_motion_tracker_uses_physics_steps_and_horizontal_motion() -> None:
    tracker_type = getattr(runtime_script, "_NormalLoadMotionTracker", None)
    assert callable(tracker_type), "simulation runtime needs a motion tracker"
    tracker = tracker_type(
        start_step_count=10,
        start_sim_time_ns=1_000_000_000,
        start_wall_time=20.0,
        start_position_m=(1.0, 2.0, 3.0),
        physics_time_step_sec=0.25,
    )
    tracker.observe_step(
        position_m=(101.0, 2.0, 99.0),
        linear_velocity_m_s=(100.0, 0.0, 12.0),
        active_control=False,
        controlled_motion=False,
        decision_wall_time=20.00,
    )
    tracker.observe_step(
        position_m=(102.0, 2.0, 99.0),
        linear_velocity_m_s=(4.0, 0.0, 12.0),
        active_control=True,
        controlled_motion=False,
        decision_wall_time=20.25,
    )
    tracker.observe_step(
        position_m=(103.0, 2.0, 99.0),
        linear_velocity_m_s=(4.0, 0.0, 12.0),
        active_control=True,
        controlled_motion=True,
        decision_wall_time=20.50,
    )
    tracker.observe_step(
        position_m=(104.0, 2.0, 99.0),
        linear_velocity_m_s=(5.0, 0.0, 12.0),
        active_control=True,
        controlled_motion=True,
        decision_wall_time=20.75,
    )
    payload = tracker.finish(
        end_step_count=14,
        end_sim_time_ns=2_000_000_000,
        end_wall_time=21.0,
        end_position_m=(104.0, 2.0, -50.0),
    )

    assert payload == {
        "normal_load_physics_time_step_sec": 0.25,
        "normal_load_step_count": 4,
        "normal_load_sim_duration_sec": 1.0,
        "normal_load_wall_duration_sec": 1.0,
        "normal_load_sim_wall_ratio": 1.0,
        "normal_load_control_step_count": 3,
        "normal_load_control_sim_duration_sec": 0.75,
        "normal_load_control_wall_duration_sec": 0.75,
        "normal_load_controlled_motion_step_count": 2,
        "normal_load_controlled_motion_sim_duration_sec": 0.5,
        "normal_load_obstacle_contact_step_count": 0,
        "normal_load_controlled_displacement_m": 2.0,
        "normal_load_controlled_path_length_m": 2.0,
        "normal_load_controlled_mean_speed_m_s": 4.0,
        "normal_load_controlled_max_speed_m_s": 5.0,
        "normal_load_base_displacement_m": 103.0,
        "normal_load_base_path_length_m": 103.0,
        "normal_load_base_mean_speed_m_s": 103.0,
        "normal_load_base_max_speed_m_s": 100.0,
    }


def test_normal_load_logger_delta_proves_messages_were_actually_accepted():
    """零丢弃和零积压不够，联合负载窗口还必须写入真实消息。"""
    start = InterfaceLogSnapshot(10, 2, 0, 0, False, False, 0)
    end = InterfaceLogSnapshot(25, 3, 0, 0, False, False, 0)

    delta = runtime_script._logger_snapshot_delta(start, end)

    assert delta["accepted_messages"] == 15


def test_ecal_runtime_uses_shared_backlog_predicate():
    assert runtime_script._has_sustained_backlog is _has_sustained_backlog


def test_measurement_complete_marker_returns_complete_json_ack_before_fence(
    monkeypatch,
    tmp_path,
) -> None:
    """peer 必须读取完整 end ACK，后续才能按其中仿真边界等待 fence。"""
    events: list[str] = []
    marker_payloads: list[object] = []
    ack_payload = {"window_end_sim_time_ns": 5_000_000_000}

    def write_marker(path: Path, payload: object) -> None:
        events.append("marker")
        marker_payloads.append(payload)
        path.write_text(json.dumps(payload), encoding="utf-8")

    def wait_for_json(path: Path, timeout_sec: float, description: str):
        assert timeout_sec > 0.0
        assert description == "normal-load transport snapshot"
        events.append("ack")
        path.write_text("{}", encoding="utf-8")
        return ack_payload

    monkeypatch.setattr(peer_script, "_write_marker", write_marker)
    monkeypatch.setattr(peer_script, "_wait_for_json_object", wait_for_json, raising=False)

    helper = getattr(peer_script, "_complete_measurement_window", None)
    assert callable(helper), "peer needs an explicit measurement-end barrier"
    observed = helper(
        tmp_path,
        duration_sec=5.0,
        measurement_start=10.0,
        measurement_end=15.0,
    )

    assert observed == ack_payload
    assert marker_payloads == [
        {
            "duration_sec": 5.0,
            "measurement_start": 10.0,
            "measurement_end": 15.0,
        }
    ]
    assert events == ["marker", "ack"]
    assert not (tmp_path / "measurement_complete.active").exists()


def test_peer_json_ack_retries_until_the_object_is_complete(tmp_path) -> None:
    """ACK 文件已创建但 JSON 尚未写完时，peer 必须重试而非读取半包。"""
    ack = tmp_path / "measurement_start.ack"
    ack.write_text('{"window_start_sim_time_ns":', encoding="utf-8")
    now = {"value": 0.0}

    def sleep(duration: float) -> None:
        now["value"] += duration
        ack.write_text('{"window_start_sim_time_ns":100}', encoding="utf-8")

    assert peer_script._wait_for_json_object(
        ack,
        0.1,
        "measurement start ack",
        monotonic=lambda: now["value"],
        sleep=sleep,
    ) == {"window_start_sim_time_ns": 100}


@pytest.mark.parametrize("value", (True, -1, 1.5, "100"))
def test_peer_rejects_invalid_sim_time_in_ack(value: object) -> None:
    """仿真边界必须是严格 uint64，不能接受 bool 或宽松数值转换。"""
    with pytest.raises(ValueError, match="window_start_sim_time_ns"):
        peer_script._ack_sim_time_ns(
            {"window_start_sim_time_ns": value},
            "window_start_sim_time_ns",
        )


def test_peer_waits_for_next_wheel_timestamp_as_delivery_fence() -> None:
    """end ACK 后必须看到同 publisher 的下一帧，不能用固定 sleep 伪装排空。"""
    events = {WHEEL_STATE: [{"timestamp_ns": 200}]}
    now = {"value": 0.0}

    def sleep(duration: float) -> None:
        now["value"] += duration
        if now["value"] >= 0.01:
            events[WHEEL_STATE].append({"timestamp_ns": 301})

    observed = peer_script._wait_for_output_fence(
        events,
        Lock(),
        topic=WHEEL_STATE,
        after_timestamp_ns=300,
        timeout_sec=0.1,
        monotonic=lambda: now["value"],
        sleep=sleep,
    )

    assert observed == 301


def test_peer_delivery_fence_timeout_is_a_hard_failure() -> None:
    now = {"value": 0.0}

    def sleep(duration: float) -> None:
        now["value"] += duration

    with pytest.raises(TimeoutError, match="wheel-state delivery fence"):
        peer_script._wait_for_output_fence(
            {WHEEL_STATE: [{"timestamp_ns": 300}]},
            Lock(),
            topic=WHEEL_STATE,
            after_timestamp_ns=300,
            timeout_sec=0.01,
            monotonic=lambda: now["value"],
            sleep=sleep,
        )


def test_peer_selects_output_events_by_runtime_sim_window() -> None:
    events = [
        {"timestamp_ns": 100},
        {"timestamp_ns": 101},
        {"timestamp_ns": 200},
        {"timestamp_ns": 201},
    ]

    assert peer_script._events_in_sim_window(
        events,
        start_sim_time_ns=100,
        end_sim_time_ns=200,
    ) == events[1:3]


def test_peer_hashes_the_deterministic_output_payload() -> None:
    payload = CODEC.encode(WheelState(123, (1.0, 2.0), ()))

    assert peer_script._payload_evidence(payload) == {
        "payload_sha256": hashlib.sha256(payload).hexdigest()
    }


def test_peer_adds_position_evidence_only_for_decoded_rtk() -> None:
    extractor = getattr(peer_script, "_rtk_position_evidence", None)
    assert callable(extractor), "simulation peer needs RTK position evidence"
    payload = CODEC.encode(RtkState(123, 1.25, -2.5, 0.75, 0.2))

    assert extractor(CONFIG, CODEC, CONFIG.rtk.topic, payload) == {
        "position_m": [1.25, -2.5, 0.75]
    }
    assert extractor(CONFIG, CODEC, CONFIG.imu.topic, b"ignored") == {}


def _rtk_log_record(
    timestamp_ns: int,
    position_m: tuple[float, float, float],
    *,
    direction: str = "publish",
    type_name: str | None = None,
) -> InterfaceLogRecord:
    message = RtkState(timestamp_ns, *position_m, 0.0)
    return InterfaceLogRecord(
        sequence=timestamp_ns,
        topic=CONFIG.rtk.topic,
        direction=direction,
        sim_time_ns=timestamp_ns,
        wall_time_ns=timestamp_ns,
        type_name=CODEC.type_name(message) if type_name is None else type_name,
        payload=CODEC.encode(message),
    )


def _wheel_log_record(
    timestamp_ns: int,
    *,
    drive_speed_rad_s: tuple[float, ...] = (1.0, 2.0),
    direction: str = "publish",
    type_name: str | None = None,
) -> InterfaceLogRecord:
    message = WheelState(timestamp_ns, drive_speed_rad_s, ())
    return InterfaceLogRecord(
        sequence=timestamp_ns,
        topic=WHEEL_STATE,
        direction=direction,
        sim_time_ns=timestamp_ns,
        wall_time_ns=timestamp_ns,
        type_name=CODEC.type_name(message) if type_name is None else type_name,
        payload=CODEC.encode(message),
    )


def _wheel_peer_event(record: InterfaceLogRecord) -> dict[str, object]:
    return {
        "timestamp_ns": record.sim_time_ns,
        "type": record.type_name,
        "payload_sha256": hashlib.sha256(record.payload).hexdigest(),
    }


def test_wheel_log_delivery_matches_every_publish_in_runtime_sim_window() -> None:
    """窗口内每条原始 wheel publish 必须在 peer 有同时间戳、类型和 payload。"""
    before = _wheel_log_record(100)
    first = _wheel_log_record(110)
    last = _wheel_log_record(200, drive_speed_rad_s=(3.0, 4.0))
    after = _wheel_log_record(210)

    evidence = verifier._summarize_wheel_log_delivery(
        (_wheel_peer_event(first), _wheel_peer_event(last)),
        (after, last, before, first),
        start_sim_time_ns=100,
        end_sim_time_ns=200,
        config=CONFIG,
        codec=CODEC,
    )

    assert evidence.logged_count == 2
    assert evidence.peer_count == 2
    assert evidence.match_count == 2


def test_wheel_log_delivery_rejects_a_logged_frame_missing_at_peer() -> None:
    """日志与 peer 的逐帧链必须独立捕获 transport 计数遗漏的缺帧。"""
    first = _wheel_log_record(110)
    missing = _wheel_log_record(120)

    with pytest.raises(AssertionError, match="missing logged wheel-state"):
        verifier._summarize_wheel_log_delivery(
            (_wheel_peer_event(first),),
            (first, missing),
            start_sim_time_ns=100,
            end_sim_time_ns=200,
            config=CONFIG,
            codec=CODEC,
        )


def test_wheel_log_delivery_rejects_peer_extra_frame() -> None:
    """peer 同窗多帧意味着共享 publisher 污染或原始日志链缺口。"""
    logged = _wheel_log_record(110)
    extra = _wheel_log_record(120)

    with pytest.raises(AssertionError, match="unexpected peer wheel-state"):
        verifier._summarize_wheel_log_delivery(
            (_wheel_peer_event(logged), _wheel_peer_event(extra)),
            (logged,),
            start_sim_time_ns=100,
            end_sim_time_ns=200,
            config=CONFIG,
            codec=CODEC,
        )


def test_wheel_log_delivery_rejects_payload_mismatch_at_same_timestamp() -> None:
    """同时间戳但 payload 不同不能被计为成功送达。"""
    logged = _wheel_log_record(110)
    different = _wheel_log_record(110, drive_speed_rad_s=(9.0, 9.0))

    with pytest.raises(AssertionError, match="payload"):
        verifier._summarize_wheel_log_delivery(
            (_wheel_peer_event(different),),
            (logged,),
            start_sim_time_ns=100,
            end_sim_time_ns=200,
            config=CONFIG,
            codec=CODEC,
        )


def test_rtk_log_chain_aligns_shuffled_raw_records_by_sim_timestamp() -> None:
    peer_events = [
        {
            "timestamp_ns": 100,
            "type": EXPECTED_TYPES[CONFIG.rtk.topic],
            "position_m": [0.0, 0.0, 0.5],
        },
        {
            "timestamp_ns": 200,
            "type": EXPECTED_TYPES[CONFIG.rtk.topic],
            "position_m": [0.5, 0.2, 0.5],
        },
        {
            "timestamp_ns": 300,
            "type": EXPECTED_TYPES[CONFIG.rtk.topic],
            "position_m": [1.0, 0.0, 0.5],
        },
    ]
    records = (
        _rtk_log_record(300, (1.0, 0.0, 0.5)),
        _rtk_log_record(100, (0.0, 0.0, 0.5)),
        _rtk_log_record(200, (0.5, 0.2, 0.5)),
    )

    evidence = verifier._summarize_rtk_log_chain(
        peer_events,
        records,
        config=CONFIG,
        codec=CODEC,
    )

    assert evidence.peer_displacement_m == pytest.approx(1.0)
    assert evidence.logged_displacement_m == pytest.approx(1.0)
    assert evidence.match_count == 3
    assert evidence.max_position_error_m == pytest.approx(0.0)


def test_rtk_log_chain_rejects_missing_peer_timestamp() -> None:
    peer_events = [
        {
            "timestamp_ns": 100,
            "type": EXPECTED_TYPES[CONFIG.rtk.topic],
            "position_m": [0.0, 0.0, 0.0],
        },
        {
            "timestamp_ns": 200,
            "type": EXPECTED_TYPES[CONFIG.rtk.topic],
            "position_m": [1.0, 0.0, 0.0],
        },
    ]

    with pytest.raises(AssertionError, match="missing"):
        verifier._summarize_rtk_log_chain(
            peer_events,
            (_rtk_log_record(100, (0.0, 0.0, 0.0)),),
            config=CONFIG,
            codec=CODEC,
        )


@pytest.mark.parametrize(
    "position_m",
    (
        None,
        [1.0, 2.0],
        [True, 2.0, 3.0],
        [math.nan, 2.0, 3.0],
        [math.inf, 2.0, 3.0],
    ),
)
def test_rtk_log_chain_rejects_invalid_peer_position(position_m: object) -> None:
    peer_events = [
        {
            "timestamp_ns": 100,
            "type": EXPECTED_TYPES[CONFIG.rtk.topic],
            "position_m": position_m,
        }
    ]

    with pytest.raises(AssertionError):
        verifier._summarize_rtk_log_chain(
            peer_events,
            (_rtk_log_record(100, (0.0, 0.0, 0.0)),),
            config=CONFIG,
            codec=CODEC,
        )


def test_evidence_path_resolver_rejects_absolute_parent_and_symlink_escape(
    tmp_path,
) -> None:
    """真实门禁只能读取本次保留目录内的原始接口日志。"""
    resolver = getattr(verifier, "_resolve_child_evidence_path", None)
    assert callable(resolver), "verifier needs a confined evidence-path resolver"
    root = tmp_path / "evidence"
    log_dir = root / "interface-logs"
    log_dir.mkdir(parents=True)
    valid = log_dir / "run.interfaces.bin"
    valid.write_bytes(b"")

    assert resolver(
        root,
        "interface-logs/run.interfaces.bin",
        suffix=".interfaces.bin",
    ) == valid
    with pytest.raises(AssertionError):
        resolver(root, str(valid.resolve()), suffix=".interfaces.bin")
    with pytest.raises(AssertionError):
        resolver(root, "../outside.interfaces.bin", suffix=".interfaces.bin")

    outside = tmp_path / "outside.interfaces.bin"
    outside.write_bytes(b"")
    escaped = log_dir / "escaped.interfaces.bin"
    escaped.symlink_to(outside)
    with pytest.raises(AssertionError):
        resolver(
            root,
            "interface-logs/escaped.interfaces.bin",
            suffix=".interfaces.bin",
        )


def test_prepare_evidence_directory_requires_an_empty_dedicated_directory(
    tmp_path,
) -> None:
    """保留证据目录不得覆盖上一轮门禁产物。"""
    prepare = getattr(verifier, "_prepare_evidence_directory", None)
    assert callable(prepare), "verifier needs a retained-evidence directory boundary"
    target = tmp_path / "retained"

    assert prepare(target) == target.resolve()
    (target / "unrelated.txt").write_text("keep", encoding="utf-8")

    with pytest.raises(ValueError, match="empty"):
        prepare(target)


def test_simulation_cli_uses_integrated_safety_gate_without_legacy_reconnect(
    monkeypatch, capsys
):
    expected_topics = set(EXPECTED_TYPES)
    result = _valid_simulation_gate_evidence()
    result.wall_clock_hz = {topic: 10.0 for topic in expected_topics}
    result.message_timestamp_hz = {topic: 10.0 for topic in expected_topics}
    calls: list[dict[str, object]] = []

    def fake_roundtrip(duration_sec: float, **kwargs):
        calls.append({"duration_sec": duration_sec, **kwargs})
        return result

    monkeypatch.setattr(verifier, "run_ecal_process_roundtrip", fake_roundtrip)
    monkeypatch.setattr(
        verifier,
        "run_ecal_reconnect_gate",
        lambda **_kwargs: (_ for _ in ()).throw(
            AssertionError("simulation must not run the legacy reconnect gate")
        ),
    )

    assert (
        verifier.main(
            [
                "--runtime",
                "simulation",
                "--warmup-sec",
                "1.25",
                "--duration-sec",
                "5",
                "--process-timeout-sec",
                "22",
                "--robot-model",
                "df_back",
            ]
        )
        == 0
    )
    output = capsys.readouterr().out.splitlines()

    assert output[0] == "runtime=simulation transport=ecal"
    assert output[-1] == "PASS"
    assert calls == [
        {
            "duration_sec": 5.0,
            "runtime": "simulation",
            "warmup_sec": 1.25,
            "process_timeout_sec": 22.0,
            "evidence_dir": None,
            "robot_model": "df_back",
        }
    ]


def test_roundtrip_cli_supports_real_simulation_runtime_and_bounded_children():
    args = _parse_args(
        [
            "--runtime",
            "simulation",
            "--warmup-sec",
            "1.25",
            "--duration-sec",
            "5",
            "--robot-model",
            "df_back",
        ]
    )

    assert args.runtime == "simulation"
    assert args.warmup_sec == pytest.approx(1.25)
    assert args.duration_sec == pytest.approx(5.0)
    assert args.process_timeout_sec == pytest.approx(60.0)
    assert args.robot_model == "df_back"


def test_simulation_child_clis_accept_differential_robot_model(tmp_path) -> None:
    """两个独立进程入口必须解析同一个代表性差速车型。"""
    runtime_args = runtime_script._parse_args(
        [
            "--result-json",
            str(tmp_path / "runtime.json"),
            "--scenario-dir",
            str(tmp_path / "scenario"),
            "--ready-file",
            str(tmp_path / "runtime.ready"),
            "--start-file",
            str(tmp_path / "start"),
            "--stop-file",
            str(tmp_path / "stop"),
            "--robot-model",
            "df_back",
        ]
    )
    peer_args = peer_script._parse_args(
        [
            "--result-json",
            str(tmp_path / "peer.json"),
            "--scenario-dir",
            str(tmp_path / "scenario"),
            "--ready-file",
            str(tmp_path / "peer.ready"),
            "--start-file",
            str(tmp_path / "start"),
            "--simulation-scenario",
            "--robot-model",
            "df_back",
        ]
    )

    assert runtime_args.robot_model == peer_args.robot_model == "df_back"


@pytest.mark.ecal
def test_real_ecal_process_roundtrip_exchanges_all_protobuf_topics_at_target_rates():
    result = run_ecal_process_roundtrip(duration_sec=2.5)

    assert result.transport_name == "ecal"
    assert result.peer_returncode == 0
    assert result.received_topics == OUTPUT_TOPICS
    assert result.topic_types == EXPECTED_TYPES
    assert result.dropped_count == result.peer_dropped_count == 0
    assert result.transport_error_count == result.peer_error_count == 0
    for channel in (CONFIG.wheel_command, CONFIG.wheel_state):
        assert result.wall_clock_hz[channel.topic] == pytest.approx(
            float(channel.rate_hz), rel=0.05
        )
        assert result.message_timestamp_hz[channel.topic] == pytest.approx(
            float(channel.rate_hz), rel=0.01
        )
    for channel in (CONFIG.lidar_front, CONFIG.lidar_rear, CONFIG.rtk, CONFIG.imu):
        assert result.wall_clock_hz[channel.topic] == pytest.approx(
            float(channel.rate_hz), rel=0.10
        )
        assert result.message_timestamp_hz[channel.topic] == pytest.approx(
            float(channel.rate_hz), rel=0.01
        )
    assert all(result.message_counts[topic] > 0 for topic in EXPECTED_TYPES)


@pytest.mark.ecal
def test_real_ecal_disconnect_reconnect_never_restores_stale_command():
    result = run_ecal_reconnect_gate(command=(4.0, 4.0), silence_sec=0.15)

    assert result.transport_name == "ecal"
    assert result.states == ("active", "disconnected", "waiting_peer", "active")
    assert result.drive_target_before_disconnect == (4.0, 4.0)
    assert (
        result.mailbox_generation_after_disconnect
        > result.mailbox_generation_before_disconnect
    )
    assert result.first_peer_terminated is True
    assert result.first_peer_returncode != 0
    assert result.first_peer_runtime_sec < result.first_peer_planned_duration_sec
    assert result.drive_target_while_disconnected == (0.0, 0.0)
    assert result.drive_target_after_peer_restart_before_new_command == (0.0, 0.0)
    assert result.silence_observed_sec >= 0.15
    assert result.silence_sample_count > 1
    assert result.silence_all_zero is True
    assert result.drive_target_after_new_command == (4.0, 4.0)


@pytest.mark.ecal
def test_real_ecal_simulation_runtime_uses_physics_feedback_and_all_six_topics():
    result = run_ecal_process_roundtrip(
        runtime="simulation",
        warmup_sec=1.0,
        duration_sec=5.0,
    )

    assert result.transport_name == "ecal"
    assert result.runtime_name == "simulation"
    assert result.feedback_is_not_command_echo
    assert result.invalid_command_rejected
    assert result.timeout_stopped_vehicle
    assert result.timeout_preserved_steering
    assert result.output_disconnect_isolated == {
        topic: True for topic in OUTPUT_TOPICS
    }
    assert result.per_topic_peer_states == {
        topic: "active" for topic in EXPECTED_TYPES
    }
    assert result.reconnect_required_new_command
    assert result.reconnect_generation_advanced
    assert (
        result.mailbox_generation_after_disconnect
        > result.mailbox_generation_before_disconnect
    )
    assert result.normal_load_obstacle_count == 20
    assert result.normal_load_log_sample_count >= 45
    assert result.normal_load_log_accepted_messages >= 1_080
    assert result.normal_load_log_final_pending == 0
    assert result.normal_load_log_sustained_backlog is False
    assert result.normal_load_log_dropped_messages == 0
    assert result.normal_load_log_dropped_events == 0
    assert result.normal_load_log_writer_failed is False
    assert result.normal_load_warmup_wall_duration_sec >= 1.0
    assert result.normal_load_warmup_sim_duration_sec >= 1.0
    assert result.normal_load_measurement_wall_duration_sec == pytest.approx(5.0)
    assert result.normal_load_sim_wall_ratio >= 0.95
    assert result.normal_load_control_duration_sec == pytest.approx(4.99, abs=0.02)
    assert result.normal_load_rtk_displacement_m > 0.5
    assert result.normal_load_base_displacement_m > 0.5
    assert result.normal_load_trajectory_distance_m > 0.5
    assert result.normal_load_average_speed_m_s > 0.1
    assert result.normal_load_nonzero_drive_feedback_wheels == 4
    assert result.normal_load_peak_left_steering_angle_rad > 0.1
    assert result.normal_load_peak_right_steering_angle_rad > 0.1
    assert result.clean_shutdown


@pytest.mark.ecal
def test_real_ecal_simulation_runtime_supports_differential_two_plus_zero() -> None:
    """代表性差速车型必须走同一真实双进程、联合负载与安全协议。"""
    result = run_ecal_process_roundtrip(
        runtime="simulation",
        robot_model="df_back",
        warmup_sec=1.0,
        duration_sec=5.0,
    )

    assert result.transport_name == "ecal"
    assert result.runtime_name == "simulation"
    assert result.robot_model == "df_back"
    assert result.normal_load_nonzero_drive_feedback_wheels == 2
    assert result.normal_load_peak_left_steering_angle_rad == pytest.approx(0.0)
    assert result.normal_load_peak_right_steering_angle_rad == pytest.approx(0.0)
    assert result.normal_load_steering_same_sign is False
    assert result.normal_load_base_displacement_m > 0.5
    assert result.normal_load_rtk_displacement_m > 0.5
    assert result.wheel_log_publish_count == result.wheel_peer_receive_count
    assert result.dropped_count == result.peer_dropped_count == 0
    assert result.clean_shutdown
