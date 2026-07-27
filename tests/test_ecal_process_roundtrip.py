# 真实 eCAL 进程门禁：只执行真实跨进程验收，不允许任何替代或弱化。
from __future__ import annotations

import subprocess
from types import SimpleNamespace

import pytest

from scripts import verify_ecal_roundtrip as verifier
from scripts.verify_ecal_roundtrip import (
    _finish_peer,
    _parse_args,
    run_ecal_process_roundtrip,
    run_ecal_reconnect_gate,
)
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)


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


def test_simulation_cli_uses_integrated_safety_gate_without_legacy_reconnect(
    monkeypatch, capsys
):
    expected_topics = set(EXPECTED_TYPES)
    output_topics = OUTPUT_TOPICS
    result = SimpleNamespace(
        transport_name="ecal",
        runtime_name="simulation",
        topic_types=EXPECTED_TYPES,
        message_counts={topic: 10 for topic in expected_topics},
        wall_clock_hz={topic: 10.0 for topic in expected_topics},
        message_timestamp_hz={topic: 10.0 for topic in expected_topics},
        per_topic_peer_states={topic: "active" for topic in expected_topics},
        feedback_is_not_command_echo=True,
        invalid_command_rejected=True,
        timeout_stopped_vehicle=True,
        timeout_preserved_steering=True,
        output_disconnect_isolated={topic: True for topic in output_topics},
        reconnect_required_new_command=True,
        clean_shutdown=True,
    )
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
        ]
    )

    assert args.runtime == "simulation"
    assert args.warmup_sec == pytest.approx(1.25)
    assert args.duration_sec == pytest.approx(5.0)
    assert args.process_timeout_sec == pytest.approx(20.0)


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
    assert result.clean_shutdown
