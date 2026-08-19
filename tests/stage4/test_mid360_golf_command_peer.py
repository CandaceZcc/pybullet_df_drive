"""MID-360 Golf 唯一路线命令端的身份、节拍与故障锁存测试。"""

from __future__ import annotations

from importlib import import_module

import pytest

from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
from slope_sim.interfaces.v2.models import (
    CommandAuthorityState,
    ImuAttitudeV2,
    Point3dV2,
    RtkStateV2,
    WheelStateV2,
)
from slope_sim.mid360_golf_drive import build_canonical_golf_route
from slope_sim.scene import TerrainBounds


SESSION_ID = bytes.fromhex("00112233445566778899aabbccddeeff")
SOURCE_SESSION_ID = bytes.fromhex("ffeeddccbbaa99887766554433221100")
BOUNDS = TerrainBounds(-10.01, 10.01, -6.65, 6.65)


class RecordingTransport:
    """以内存回调模拟 peer transport，并保留命令原始帧。"""

    def __init__(self) -> None:
        self.callbacks: dict[str, object] = {}
        self.published: list[tuple[str, bytes, str, int, float]] = []

    def subscribe(self, topic: str, type_name: str, callback):
        self.callbacks[topic] = callback
        return object()

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float,
    ) -> bool:
        self.published.append((topic, payload, type_name, sim_time_ns, wall_time))
        return True

    def emit(self, topic: str, model: object, *, received_at: float) -> None:
        codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(
            load_v2_descriptor()
        )
        self.callbacks[topic](codec.encode(model).payload, received_at)


def _pose_messages(
    timestamp_ns: int,
    sequence: int = 0,
    *,
    position_xy: tuple[float, float] = (-3.5, 0.0),
):
    descriptor = load_v2_descriptor()
    x, y = position_xy
    return (
        RtkStateV2(
            timestamp_ns=timestamp_ns,
            sequence=sequence,
            world_generation=1,
            frame_id="world",
            left=Point3dV2(x, y + 0.2, 0.32),
            center=Point3dV2(x, y, 0.32),
            right=Point3dV2(x, y - 0.2, 0.32),
            heading_rad=0.0,
            simulation_session_id=SESSION_ID,
            descriptor_sha256=descriptor.sha256,
        ),
        ImuAttitudeV2(
            timestamp_ns=timestamp_ns,
            roll_rad=0.0,
            pitch_rad=0.0,
            sequence=sequence,
            world_generation=1,
            frame_id="base_link",
            simulation_session_id=SESSION_ID,
            descriptor_sha256=descriptor.sha256,
        ),
    )


def _wheel_state(timestamp_ns: int, sequence: int) -> WheelStateV2:
    descriptor = load_v2_descriptor()
    return WheelStateV2(
        timestamp_ns=timestamp_ns,
        drive_wheel_speed_rad_s=(0.0, 0.0),
        steering_wheel_angle_rad=(),
        sequence=sequence,
        world_generation=1,
        command_generation=1,
        robot_model="df_mid",
        simulation_session_id=SESSION_ID,
        descriptor_sha256=descriptor.sha256,
        command_authority_state=CommandAuthorityState.CLAIMABLE,
        command_owner_source_id="",
        command_owner_source_session_id=b"",
        command_peer_count=1,
    )


def test_v2_transport_factory_supports_peer_role_and_all_raw_decoders(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """peer 必须反转方向，且四个输出 subscriber 都有 raw parser。"""
    module = import_module("slope_sim.interfaces.v2.transport")
    captured: dict[str, object] = {}

    class CapturingTransport:
        def __init__(self, **kwargs: object) -> None:
            captured.update(kwargs)

    monkeypatch.setattr(module, "EcalTransport", CapturingTransport)
    module.create_v2_ecal_transport(
        descriptor=load_v2_descriptor(),
        role="peer",
        bindings=object(),
    )

    assert captured["role"] == "peer"
    channels = captured["channel_bindings"]
    assert [channel.direction for channel in channels] == [
        "subscribe",
        "publish",
        "publish",
        "publish",
        "publish",
    ]
    assert all(channel.raw_parser is not None for channel in channels)


def test_command_peer_emits_one_identity_bound_command_per_wheel_timestamp() -> None:
    """墙钟到达速度不参与路线节拍；命令只复制当前 WheelState 身份。"""
    module = import_module("scripts.mid360_golf_command_peer")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    rtk, imu = _pose_messages(0)
    transport.emit("/sim/rtk/state", rtk, received_at=500.0)
    transport.emit("/sim/imu/attitude", imu, received_at=1.0)

    transport.emit("/sim/wheel/state", _wheel_state(10_000_000, 0), received_at=900.0)
    transport.emit("/sim/wheel/state", _wheel_state(20_000_000, 1), received_at=2.0)

    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    commands = [codec.decode_wheel_command(item[1]) for item in transport.published]
    assert len(commands) == 2
    assert [command.timestamp_ns for command in commands] == [10_000_000, 20_000_000]
    assert [command.sequence for command in commands] == [0, 1]
    assert all(command.drive_wheel_speed_rad_s != (0.0, 0.0) for command in commands)
    assert all(command.steering_wheel_speed_rad_s == () for command in commands)
    assert all(command.simulation_session_id == SESSION_ID for command in commands)
    assert all(command.descriptor_sha256 == descriptor.sha256 for command in commands)
    assert all(command.world_generation == 1 for command in commands)
    assert all(command.command_generation == 1 for command in commands)
    assert all(command.robot_model == "df_mid" for command in commands)
    assert all(command.source_session_id == SOURCE_SESSION_ID for command in commands)


def test_command_peer_defers_wheel_until_same_cadence_pose_pair_arrives() -> None:
    """跨 topic 回调乱序时等待完整姿态，不把瞬时缺帧锁存为 stale。"""
    module = import_module("scripts.mid360_golf_command_peer")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    rtk0, imu0 = _pose_messages(0)
    transport.emit("/sim/rtk/state", rtk0, received_at=0.0)
    transport.emit("/sim/imu/attitude", imu0, received_at=0.0)
    for sequence in range(10):
        transport.emit(
            "/sim/wheel/state",
            _wheel_state((sequence + 1) * 10_000_000, sequence),
            received_at=1.0 + sequence * 0.01,
        )

    transport.emit(
        "/sim/wheel/state",
        _wheel_state(110_000_000, 10),
        received_at=2.0,
    )
    assert len(transport.published) == 10
    assert peer.fault_reason is None

    rtk100, imu100 = _pose_messages(100_000_000, sequence=1)
    transport.emit("/sim/rtk/state", rtk100, received_at=2.01)
    assert len(transport.published) == 10
    transport.emit("/sim/imu/attitude", imu100, received_at=2.02)

    command = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(
        descriptor
    ).decode_wheel_command(transport.published[-1][1])
    assert len(transport.published) == 11
    assert command.timestamp_ns == 110_000_000
    assert command.sequence == 10
    assert command.drive_wheel_speed_rad_s != (0.0, 0.0)
    assert peer.fault_reason is None


def test_command_peer_faults_and_releases_wheel_when_pose_pair_never_arrives() -> None:
    """真正缺失的姿态须在 Simulator 命令期限前生成零命令并锁存故障。"""
    module = import_module("scripts.mid360_golf_command_peer")
    simulation_module = import_module("scripts.mid360_golf_simulation")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    rtk0, imu0 = _pose_messages(0)
    transport.emit("/sim/rtk/state", rtk0, received_at=0.0)
    transport.emit("/sim/imu/attitude", imu0, received_at=0.0)
    for sequence in range(10):
        transport.emit(
            "/sim/wheel/state",
            _wheel_state((sequence + 1) * 10_000_000, sequence),
            received_at=1.0 + sequence * 0.01,
        )
    transport.emit(
        "/sim/wheel/state",
        _wheel_state(110_000_000, 10),
        received_at=2.0,
    )

    assert module._POSE_CALLBACK_GRACE_SEC < simulation_module._COMMAND_RESPONSE_TIMEOUT_SEC
    peer.service_pending_wheel(now=2.499_999)
    assert len(transport.published) == 10
    assert peer.fault_reason is None

    peer.service_pending_wheel(now=2.5)

    command = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(
        descriptor
    ).decode_wheel_command(transport.published[-1][1])
    assert len(transport.published) == 11
    assert command.timestamp_ns == 110_000_000
    assert command.sequence == 10
    assert command.drive_wheel_speed_rad_s == (0.0, 0.0)
    assert peer.fault_reason == "RTK/IMU pose is stale"

    peer.service_pending_wheel(now=3.0)
    rtk100, imu100 = _pose_messages(100_000_000, sequence=1)
    transport.emit("/sim/rtk/state", rtk100, received_at=3.01)
    transport.emit("/sim/imu/attitude", imu100, received_at=3.02)
    assert len(transport.published) == 11
    assert peer.fault_reason == "RTK/IMU pose is stale"


def test_command_peer_faults_after_sustained_route_progress_lag() -> None:
    """车辆虽在中心线上但持续落后时间路线，也必须锁存故障并归零。"""
    module = import_module("scripts.mid360_golf_command_peer")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    rtk, imu = _pose_messages(0)
    transport.emit("/sim/rtk/state", rtk, received_at=0.0)
    transport.emit("/sim/imu/attitude", imu, received_at=0.0)

    for sequence in range(500):
        timestamp_ns = (sequence + 1) * 10_000_000
        if timestamp_ns % 100_000_000 == 0:
            pose_sequence = timestamp_ns // 100_000_000
            rtk, imu = _pose_messages(timestamp_ns, sequence=pose_sequence)
            transport.emit("/sim/rtk/state", rtk, received_at=sequence * 0.01)
            transport.emit("/sim/imu/attitude", imu, received_at=sequence * 0.01)
        transport.emit(
            "/sim/wheel/state",
            _wheel_state(timestamp_ns, sequence),
            received_at=sequence * 0.01,
        )
        if peer.fault_reason is not None:
            break

    command = V2ProtoCodec(descriptor).decode_wheel_command(transport.published[-1][1])
    assert peer.fault_reason == "route_progress_lag"
    assert command.drive_wheel_speed_rad_s == (0.0, 0.0)


def test_command_peer_faults_if_route_is_incomplete_at_stop_deadline() -> None:
    """名义路线时间到达时仍停在起点，不能进入正常停车尾段。"""
    module = import_module("scripts.mid360_golf_command_peer")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    peer._normal_stop_timestamp_ns = 100_000_000
    rtk, imu = _pose_messages(0)
    transport.emit("/sim/rtk/state", rtk, received_at=0.0)
    transport.emit("/sim/imu/attitude", imu, received_at=0.0)

    for sequence in range(10):
        timestamp_ns = (sequence + 1) * 10_000_000
        if timestamp_ns == 100_000_000:
            rtk, imu = _pose_messages(timestamp_ns, sequence=1)
            transport.emit("/sim/rtk/state", rtk, received_at=0.1)
            transport.emit("/sim/imu/attitude", imu, received_at=0.1)
        transport.emit(
            "/sim/wheel/state",
            _wheel_state(timestamp_ns, sequence),
            received_at=sequence * 0.01,
        )

    command = V2ProtoCodec(descriptor).decode_wheel_command(transport.published[-1][1])
    assert peer.fault_reason == "route_progress_deadline"
    assert command.drive_wheel_speed_rad_s == (0.0, 0.0)


def test_command_peer_ignores_a_service_sample_taken_before_pending_arrives() -> None:
    """service 先采样墙钟、回调后入锁时，本轮不得把负 age 当成进程故障。"""
    module = import_module("scripts.mid360_golf_command_peer")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    rtk0, imu0 = _pose_messages(0)
    transport.emit("/sim/rtk/state", rtk0, received_at=0.0)
    transport.emit("/sim/imu/attitude", imu0, received_at=0.0)
    for sequence in range(10):
        transport.emit(
            "/sim/wheel/state",
            _wheel_state((sequence + 1) * 10_000_000, sequence),
            received_at=1.0 + sequence * 0.01,
        )
    transport.emit(
        "/sim/wheel/state",
        _wheel_state(110_000_000, 10),
        received_at=2.0,
    )

    peer.service_pending_wheel(now=1.999_999)

    assert len(transport.published) == 10
    assert peer.fault_reason is None
    peer.service_pending_wheel(now=2.5)
    assert len(transport.published) == 11
    assert peer.fault_reason == "RTK/IMU pose is stale"


def test_command_peer_does_not_overwrite_a_pending_wheel_state() -> None:
    """协议异常多来一帧时，先释放 pending，再按连续序列归零新帧。"""
    module = import_module("scripts.mid360_golf_command_peer")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    rtk0, imu0 = _pose_messages(0)
    transport.emit("/sim/rtk/state", rtk0, received_at=0.0)
    transport.emit("/sim/imu/attitude", imu0, received_at=0.0)
    for sequence in range(10):
        transport.emit(
            "/sim/wheel/state",
            _wheel_state((sequence + 1) * 10_000_000, sequence),
            received_at=1.0 + sequence * 0.01,
        )
    transport.emit(
        "/sim/wheel/state",
        _wheel_state(110_000_000, 10),
        received_at=2.0,
    )

    transport.emit(
        "/sim/wheel/state",
        _wheel_state(120_000_000, 11),
        received_at=2.01,
    )

    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    commands = [codec.decode_wheel_command(item[1]) for item in transport.published]
    assert [command.timestamp_ns for command in commands[-2:]] == [
        110_000_000,
        120_000_000,
    ]
    assert [command.sequence for command in commands[-2:]] == [10, 11]
    assert [command.drive_wheel_speed_rad_s for command in commands[-2:]] == [
        (0.0, 0.0),
        (0.0, 0.0),
    ]
    assert peer.fault_reason == "WheelState advanced while RTK/IMU pose was pending"


def test_command_peer_releases_pending_before_a_new_wheel_after_external_fault() -> None:
    """pending 后先锁存外部 fault，也必须按原 timestamp 顺序发布零命令。"""
    module = import_module("scripts.mid360_golf_command_peer")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    rtk0, imu0 = _pose_messages(0)
    transport.emit("/sim/rtk/state", rtk0, received_at=0.0)
    transport.emit("/sim/imu/attitude", imu0, received_at=0.0)
    for sequence in range(10):
        transport.emit(
            "/sim/wheel/state",
            _wheel_state((sequence + 1) * 10_000_000, sequence),
            received_at=1.0 + sequence * 0.01,
        )
    transport.emit(
        "/sim/wheel/state",
        _wheel_state(110_000_000, 10),
        received_at=2.0,
    )
    peer.latch_fault("external fault")

    transport.emit(
        "/sim/wheel/state",
        _wheel_state(120_000_000, 11),
        received_at=2.01,
    )

    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    commands = [codec.decode_wheel_command(item[1]) for item in transport.published]
    assert [command.timestamp_ns for command in commands[-2:]] == [
        110_000_000,
        120_000_000,
    ]
    assert [command.sequence for command in commands[-2:]] == [10, 11]
    assert [command.drive_wheel_speed_rad_s for command in commands[-2:]] == [
        (0.0, 0.0),
        (0.0, 0.0),
    ]
    assert peer.fault_reason == "external fault"


def test_command_peer_releases_pending_before_a_new_normal_stop_wheel() -> None:
    """第二帧恰好进入停车尾段时，也不能绕过单槽而造成 timestamp 倒序。"""
    module = import_module("scripts.mid360_golf_command_peer")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    peer._normal_stop_timestamp_ns = 120_000_000
    rtk0, imu0 = _pose_messages(0)
    transport.emit("/sim/rtk/state", rtk0, received_at=0.0)
    transport.emit("/sim/imu/attitude", imu0, received_at=0.0)
    for sequence in range(10):
        transport.emit(
            "/sim/wheel/state",
            _wheel_state((sequence + 1) * 10_000_000, sequence),
            received_at=1.0 + sequence * 0.01,
        )
    transport.emit(
        "/sim/wheel/state",
        _wheel_state(110_000_000, 10),
        received_at=2.0,
    )

    transport.emit(
        "/sim/wheel/state",
        _wheel_state(120_000_000, 11),
        received_at=2.01,
    )

    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    commands = [codec.decode_wheel_command(item[1]) for item in transport.published]
    assert [command.timestamp_ns for command in commands[-2:]] == [
        110_000_000,
        120_000_000,
    ]
    assert [command.sequence for command in commands[-2:]] == [10, 11]
    assert [command.drive_wheel_speed_rad_s for command in commands[-2:]] == [
        (0.0, 0.0),
        (0.0, 0.0),
    ]
    assert peer.fault_reason == "WheelState advanced while RTK/IMU pose was pending"


def test_command_peer_latches_pose_sequence_fault_and_keeps_publishing_zero() -> None:
    """姿态流不连续后不再恢复路线输出，但后续 WheelState 仍逐条得到零命令。"""
    module = import_module("scripts.mid360_golf_command_peer")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    rtk, imu = _pose_messages(0)
    transport.emit("/sim/rtk/state", rtk, received_at=0.0)
    transport.emit("/sim/imu/attitude", imu, received_at=0.0)
    transport.emit("/sim/wheel/state", _wheel_state(10_000_000, 0), received_at=0.0)

    broken_rtk, _unused = _pose_messages(100_000_000, sequence=2)
    transport.emit("/sim/rtk/state", broken_rtk, received_at=0.0)
    transport.emit("/sim/wheel/state", _wheel_state(20_000_000, 1), received_at=0.0)
    transport.emit("/sim/wheel/state", _wheel_state(30_000_000, 2), received_at=0.0)

    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    commands = [codec.decode_wheel_command(item[1]) for item in transport.published]
    assert commands[0].drive_wheel_speed_rad_s != (0.0, 0.0)
    assert [command.drive_wheel_speed_rad_s for command in commands[1:]] == [
        (0.0, 0.0),
        (0.0, 0.0),
    ]
    assert [command.sequence for command in commands] == [0, 1, 2]
    assert peer.fault_reason == "RTK sequence must start at zero and remain continuous"


def test_command_peer_serves_zero_commands_until_frozen_collection_end() -> None:
    """车辆提前归零也必须服务完整录制窗口，不能让 Simulator 尾段超时。"""
    module = import_module("scripts.mid360_golf_command_peer")
    simulation_module = import_module("scripts.mid360_golf_simulation")
    descriptor = load_v2_descriptor()
    transport = RecordingTransport()
    route = build_canonical_golf_route(BOUNDS)
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=descriptor,
        route=route,
        source_session_id=SOURCE_SESSION_ID,
    )
    normal_stop_timestamp_ns = peer._normal_stop_timestamp_ns
    collection_end_timestamp_ns = round(
        simulation_module._default_collection_duration_sec(route) * 1_000_000_000
    )
    assert peer._normal_finish_timestamp_ns == collection_end_timestamp_ns
    assert collection_end_timestamp_ns == 208_200_000_000
    peer._controller._segment_index = len(route.segments) - 1
    pose_sequence = 0

    def emit_tail_pose(timestamp_ns: int, *, received_at: float) -> None:
        nonlocal pose_sequence
        if (
            timestamp_ns % module._POSE_PERIOD_NS != 0
            or timestamp_ns >= collection_end_timestamp_ns
        ):
            return
        rtk, imu = _pose_messages(
            timestamp_ns,
            sequence=pose_sequence,
            position_xy=route.segments[-1].end_xy,
        )
        transport.emit("/sim/rtk/state", rtk, received_at=received_at)
        transport.emit("/sim/imu/attitude", imu, received_at=received_at)
        pose_sequence += 1

    early_stop_timestamps = range(
        normal_stop_timestamp_ns,
        normal_stop_timestamp_ns + module._ZERO_TAIL_NS + module._WHEEL_PERIOD_NS,
        module._WHEEL_PERIOD_NS,
    )

    for sequence, timestamp_ns in enumerate(early_stop_timestamps):
        emit_tail_pose(timestamp_ns, received_at=sequence * 0.01)
        transport.emit(
            "/sim/wheel/state",
            _wheel_state(timestamp_ns, sequence),
            received_at=sequence * 0.01,
        )

    assert peer.finished is False

    next_sequence = len(early_stop_timestamps)
    for sequence, timestamp_ns in enumerate(
        range(
            normal_stop_timestamp_ns
            + module._ZERO_TAIL_NS
            + module._WHEEL_PERIOD_NS,
            collection_end_timestamp_ns + module._WHEEL_PERIOD_NS,
            module._WHEEL_PERIOD_NS,
        ),
        start=next_sequence,
    ):
        emit_tail_pose(timestamp_ns, received_at=sequence * 0.01)
        transport.emit(
            "/sim/wheel/state",
            _wheel_state(timestamp_ns, sequence),
            received_at=sequence * 0.01,
        )

    snapshot = peer.snapshot()
    assert snapshot.finished is False
    assert snapshot.last_wheel_timestamp_ns == collection_end_timestamp_ns
    assert snapshot.latest_pose_timestamp_ns == (
        collection_end_timestamp_ns - module._POSE_PERIOD_NS
    )

    final_rtk, final_imu = _pose_messages(
        collection_end_timestamp_ns,
        sequence=pose_sequence,
        position_xy=route.segments[-1].end_xy,
    )
    transport.emit("/sim/rtk/state", final_rtk, received_at=next_sequence * 0.01)
    assert peer.finished is False
    transport.emit("/sim/imu/attitude", final_imu, received_at=next_sequence * 0.01)

    snapshot = peer.snapshot()
    assert snapshot.finished is True
    assert snapshot.latest_pose_timestamp_ns == collection_end_timestamp_ns


def test_command_peer_fault_finishes_after_twenty_quiet_wheel_states() -> None:
    """故障停车仍需连续 20 帧静止反馈，正常结束门不得覆盖该判据。"""
    module = import_module("scripts.mid360_golf_command_peer")
    transport = RecordingTransport()
    peer = module.GolfCommandPeer(
        transport=transport,
        descriptor=load_v2_descriptor(),
        route=build_canonical_golf_route(BOUNDS),
        source_session_id=SOURCE_SESSION_ID,
    )
    peer.latch_fault("external fault")

    for sequence in range(19):
        transport.emit(
            "/sim/wheel/state",
            _wheel_state((sequence + 1) * 10_000_000, sequence),
            received_at=sequence * 0.01,
        )

    assert peer.finished is False

    transport.emit(
        "/sim/wheel/state",
        _wheel_state(200_000_000, 19),
        received_at=0.19,
    )

    assert peer.finished is True
    assert peer.fault_reason == "external fault"
