"""阶段四 B2：持续五话题 raw eCAL peer 的验收合同。"""
from __future__ import annotations

from importlib import import_module
from pathlib import Path
import sys
import time

import pytest

from slope_sim.interfaces.models import WheelState
from slope_sim.model_registry import get_robot_model


def test_runtime_peer_derives_exact_five_topic_counts_from_240hz_window() -> None:
    """5 秒窗口必须要求 500 条 wheel 与各 50 条同步传感器，不能接受近似数量。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")
    counts = module.expected_v2_frame_counts(duration_sec=5.0)

    assert counts == {
        "/sim/wheel/state": 500,
        "/sim/lidar/points": 50,
        "/sim/rtk/state": 50,
        "/sim/imu/attitude": 50,
    }
    assert module.expected_v2_frame_counts(duration_sec=0.1) == {
        "/sim/wheel/state": 10,
        "/sim/lidar/points": 1,
        "/sim/rtk/state": 1,
        "/sim/imu/attitude": 1,
    }
    with pytest.raises(ValueError, match="positive"):
        module.expected_v2_frame_counts(duration_sec=0.0)


def test_runtime_peer_verifier_requires_exact_counts_and_one_output_identity() -> None:
    """peer 证据必须证明所有输出没有漏帧、混代或 callback 错误。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")
    evidence = {
        "command_peer_count": 1,
        "callback_errors": 0,
        "clean_shutdown": True,
        "outputs": {
            "/sim/wheel/state": {
                "count": 10,
                "sequences": list(range(10)),
                "timestamps_ns": [index * 10_000_000 for index in range(1, 11)],
                "identities": [["01" * 16, "ab" * 32, 1]] * 10,
            },
            "/sim/lidar/points": {
                "count": 1,
                "sequences": [0],
                "timestamps_ns": [100_000_000],
                "identities": [["01" * 16, "ab" * 32, 1]],
            },
            "/sim/rtk/state": {
                "count": 1,
                "sequences": [0],
                "timestamps_ns": [100_000_000],
                "identities": [["01" * 16, "ab" * 32, 1]],
            },
            "/sim/imu/attitude": {
                "count": 1,
                "sequences": [0],
                "timestamps_ns": [100_000_000],
                "identities": [["01" * 16, "ab" * 32, 1]],
            },
        },
    }

    module.verify_peer_evidence(evidence, duration_sec=0.1)

    evidence["outputs"]["/sim/imu/attitude"]["count"] = 0
    with pytest.raises(ValueError, match="count"):
        module.verify_peer_evidence(evidence, duration_sec=0.1)


def test_runtime_gate_verifier_rejects_nonzero_producer_drop() -> None:
    """真实 eCAL gate 必须拒绝 runtime 端任意 producer drop/error 或不完整关闭。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")
    verify = getattr(module, "verify_runtime_evidence", None)
    assert callable(verify), "runtime gate needs a fail-closed runtime evidence verifier"
    runtime = {
        "physics_steps": 1200,
        "sim_duration_sec": 5.0,
        "wall_duration_sec": 5.01,
        "published_frames": module.expected_v2_frame_counts(duration_sec=5.0),
        "transport_metrics": {"published_count": 650, "error_count": 0, "dropped_count": 0},
        "lidar_worker": {"clean_shutdown": True},
        "clean_shutdown": True,
    }
    verify(runtime, duration_sec=5.0)
    runtime["transport_metrics"]["dropped_count"] = 1
    with pytest.raises(ValueError, match="dropped"):
        verify(runtime, duration_sec=5.0)


def test_dashboard_ecal_gate_requires_a_real_desktop_backend(monkeypatch: pytest.MonkeyPatch) -> None:
    """真实 GUI 验收不得把 offscreen Qt backend 当作桌面窗口。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")
    require = getattr(module, "require_real_desktop_environment", None)
    assert callable(require), "dashboard gate needs a real desktop environment guard"

    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setenv("DISPLAY", ":1")
    with pytest.raises(RuntimeError, match="offscreen"):
        require()

    monkeypatch.delenv("QT_QPA_PLATFORM")
    monkeypatch.delenv("DISPLAY")
    monkeypatch.delenv("WAYLAND_DISPLAY", raising=False)
    with pytest.raises(RuntimeError, match="DISPLAY"):
        require()

    monkeypatch.setenv("DISPLAY", ":1")
    require()


def test_runtime_peer_collector_preserves_exact_command_peer_seen_during_window() -> None:
    """运行时关闭后 discovery 可归零，证据必须保留窗口内已观测的一条 command peer。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")
    observed_type = getattr(module, "observe_command_peer_count", None)
    assert callable(observed_type), "collector needs a command-peer window observation helper"

    observed = 0
    observed = observed_type(observed, 1)
    observed = observed_type(observed, 0)

    assert observed == 1
    assert observed_type(0, 0) == 0
    with pytest.raises(ValueError, match="zero or one"):
        observed_type(0, 2)


def test_raw_output_collector_verifies_and_records_existing_v2_wheel_bytes() -> None:
    """持续 peer 的 callback 必须复用 raw metadata gate 和 codec，保留输出身份。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    controller_type = import_module("slope_sim.interfaces.v2.runtime_protocol").V2RuntimeProtocol
    wheel_factory = import_module("slope_sim.interfaces.v2.sensor_frames").V2WheelStateFactory
    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    raw_frame_type = import_module("slope_sim.interfaces.v2.ecal_raw").RawReceivedFrame

    class Transport:
        def close(self) -> None:
            """本单元不创建原生资源。"""

    controller = controller_type(
        get_robot_model("df_mid"), transport=Transport(), descriptor=descriptor
    )
    wheel = wheel_factory(controller, "df_mid").build(
        WheelState(10_000_000, (1.5, -1.5), ())
    )
    encoded = codec.encode(wheel)
    frame = raw_frame_type(
        payload=encoded.payload,
        remote_publisher_entity_id=7,
        remote_publisher_process_id=8,
        remote_publisher_host_name="localhost",
        remote_type_name=encoded.type_name,
        remote_encoding="proto",
        remote_descriptor=descriptor.serialized_file_descriptor_set,
        send_timestamp_us=10,
        send_clock=11,
        received_at=12.0,
    )
    collector = module.RawV2OutputCollector(descriptor)

    collector.record("/sim/wheel/state", frame)

    assert collector.output_evidence("/sim/wheel/state") == {
        "count": 1,
        "sequences": [0],
        "timestamps_ns": [10_000_000],
        "received_at_sec": [12.0],
        "identities": [[wheel.simulation_session_id.hex(), descriptor.sha256.hex(), 1]],
        "publishers": [{
            "entity_id": 7,
            "process_id": 8,
            "host_name": "localhost",
        }],
    }


def test_runtime_peer_collector_scans_lidar_audit_fields_without_point_object_decode(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """满负载 LiDAR 审计不得为每个点创建 Python 领域对象。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    models = import_module("slope_sim.interfaces.v2.models")
    raw_frame_type = import_module("slope_sim.interfaces.v2.ecal_raw").RawReceivedFrame
    session = bytes.fromhex("00112233445566778899aabbccddeeff")
    cloud = models.LidarPointCloudV2(
        10_000_000,
        "lidar_link",
        2,
        1,
        (
            models.LidarPointV2(0, 1.0, 2.0, 3.0, 100, 1, 0),
            models.LidarPointV2(10, 4.0, 5.0, 6.0, 120, 2, 1),
        ),
        7,
        1,
        session,
        descriptor.sha256,
    )
    encoded = codec.encode(cloud)
    frame = raw_frame_type(
        payload=encoded.payload,
        remote_publisher_entity_id=7,
        remote_publisher_process_id=8,
        remote_publisher_host_name="localhost",
        remote_type_name=encoded.type_name,
        remote_encoding="proto",
        remote_descriptor=descriptor.serialized_file_descriptor_set,
        send_timestamp_us=10,
        send_clock=11,
        received_at=12.0,
    )
    collector = module.RawV2OutputCollector(descriptor)

    def fail_if_decoded(_topic: str):
        raise AssertionError("LiDAR collector must not create point objects")

    monkeypatch.setattr(collector, "_parser_for_topic", fail_if_decoded)

    collector.record("/sim/lidar/points", frame)

    assert collector.output_evidence("/sim/lidar/points") == {
        "count": 1,
        "sequences": [7],
        "timestamps_ns": [10_000_000],
        "point_counts": [2],
        "received_at_sec": [12.0],
        "identities": [[session.hex(), descriptor.sha256.hex(), 1]],
        "publishers": [{
            "entity_id": 7,
            "process_id": 8,
            "host_name": "localhost",
        }],
    }


def test_runtime_peer_collector_processes_80000_point_lidar_within_callback_budget() -> None:
    """满负载 LiDAR 审计必须小于 30 ms，避免堵塞 native receive callback。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    models = import_module("slope_sim.interfaces.v2.models")
    raw_frame_type = import_module("slope_sim.interfaces.v2.ecal_raw").RawReceivedFrame
    cloud = models.LidarPointCloudV2(
        10_000_000,
        "lidar_link",
        80_000,
        1,
        tuple(
            models.LidarPointV2(index, 1.0, 2.0, 3.0, 100, 1, 0)
            for index in range(80_000)
        ),
        7,
        1,
        bytes(16),
        descriptor.sha256,
    )
    encoded = codec.encode(cloud)
    frame = raw_frame_type(
        payload=encoded.payload,
        remote_publisher_entity_id=7,
        remote_publisher_process_id=8,
        remote_publisher_host_name="localhost",
        remote_type_name=encoded.type_name,
        remote_encoding="proto",
        remote_descriptor=descriptor.serialized_file_descriptor_set,
        send_timestamp_us=10,
        send_clock=11,
        received_at=12.0,
    )
    collector = module.RawV2OutputCollector(descriptor)

    start = time.perf_counter()
    collector.record("/sim/lidar/points", frame)
    duration_ms = (time.perf_counter() - start) * 1_000.0

    assert collector.output_evidence("/sim/lidar/points")["point_counts"] == [80_000]
    assert duration_ms < 30.0


def test_runtime_peer_collector_command_is_explicit_and_module_based(tmp_path: Path) -> None:
    """父进程必须以项目模块启动 collector，且所有证据路径均为绝对路径。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")

    command = module.build_collector_command(
        descriptor_path=tmp_path / "v2.desc",
        ready_path=tmp_path / "ready.json",
        result_path=tmp_path / "peer.json",
        duration_sec=5.0,
        timeout_sec=20.0,
    )

    assert command[:3] == [sys.executable, "-m", "scripts.verify_stage4_v2_runtime_ecal"]
    assert command[3:] == [
        "--participant", "collector",
        "--descriptor-path", str((tmp_path / "v2.desc").resolve()),
        "--ready-path", str((tmp_path / "ready.json").resolve()),
        "--result-path", str((tmp_path / "peer.json").resolve()),
        "--duration-sec", "5.0",
        "--timeout-sec", "20.0",
    ]


def test_runtime_peer_gate_paths_are_unique_absolute_and_not_precreated(tmp_path: Path) -> None:
    """每次真实运行必须使用新证据路径，禁止覆盖任何历史 eCAL 结果。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")

    paths = module.build_runtime_evidence_paths(tmp_path / "run")

    assert set(paths) == {"descriptor", "collector_ready", "collector_result", "runtime_result", "process"}
    assert all(path.is_absolute() for path in paths.values())
    assert len(set(paths.values())) == len(paths)
    assert not any(path.exists() for path in paths.values())


def test_dashboard_ecal_gate_runs_the_dashboard_with_the_verified_transport(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    """桌面验收必须复用同一真实 eCAL collector gate，不能退回 local transport。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")
    dashboard_module = import_module("scripts.stage4_v2_dashboard")
    monkeypatch.delenv("QT_QPA_PLATFORM", raising=False)
    monkeypatch.setenv("DISPLAY", ":1")
    transport_factory = object()
    calls: list[dict[str, object]] = []

    def fake_dashboard_session(**kwargs: object) -> dict[str, object]:
        calls.append(kwargs)
        return {"published_frames": {"/sim/wheel/state": 10}}

    def fake_gate(**kwargs: object) -> dict[str, object]:
        return kwargs["runtime_runner"](
            result_json=tmp_path / "runtime-result.json",
            duration_sec=0.1,
            robot_model="df_mid",
            transport_factory=transport_factory,
            peer_timeout_sec=10.0,
        )

    monkeypatch.setattr(dashboard_module, "run_v2_dashboard_session", fake_dashboard_session)
    monkeypatch.setattr(module, "_run_v2_ecal_gate", fake_gate, raising=False)

    screenshot = tmp_path / "dashboard.png"
    result = module.run_v2_dashboard_ecal_gate(
        evidence_dir=tmp_path / "desktop-ecal",
        duration_sec=0.1,
        robot_model="df_mid",
        peer_timeout_sec=10.0,
        screenshot_png=screenshot,
    )

    assert result == {"published_frames": {"/sim/wheel/state": 10}}
    assert calls == [
        {
                "result_json": tmp_path / "runtime-result.json",
                "duration_sec": 0.1,
                "robot_model": "df_mid",
                "peer_timeout_sec": 10.0,
                "screenshot_png": screenshot,
                "isolate_runtime": True,
            }
        ]


@pytest.mark.ecal
def test_real_v2_runtime_peer_collects_one_verified_100_10hz_window(tmp_path: Path) -> None:
    """真实 eCAL 必须收齐一个 100 ms 五话题窗口，不能使用 local transport 替代。"""
    module = import_module("scripts.verify_stage4_v2_runtime_ecal")

    result = module.run_v2_runtime_ecal_gate(
        evidence_dir=tmp_path / "real-v2-runtime",
        duration_sec=0.1,
        robot_model="df_mid",
        peer_timeout_sec=10.0,
    )

    assert result["runtime"]["published_frames"] == module.expected_v2_frame_counts(
        duration_sec=0.1
    )
    assert result["collector"]["command_peer_count"] == 1
