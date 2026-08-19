#!/usr/bin/env python3
# 阶段三接口 DIRECT 验收：聚合轮控、传感器、场景、日志、状态和性能硬门禁。
from __future__ import annotations

from collections import defaultdict
from contextlib import ExitStack, contextmanager
from dataclasses import dataclass, replace
from fractions import Fraction
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Callable, Iterator, Sequence

import pybullet as p
from google.protobuf import descriptor_pb2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import slope_sim.obstacles as obstacle_module
import slope_sim.scene as scene_module
from scripts.verify_stage1_matrix import MAX_CONTACT_PENETRATION_M, validate_robot_pose
from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import SimulationCoordinator, build_world_from_scene_document
from slope_sim.dashboard_charts import (
    INTERFACE_LINE_PLOT_TABS,
    InterfaceChartBuffer,
    interface_chart_specs,
)
from slope_sim.interfaces.backlog import _has_sustained_backlog
from slope_sim.interfaces.clock import PeriodicScheduler, SimulationClock
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.dashboard_snapshot import LidarTopViewFrame, LidarTopViewPoint
from slope_sim.interfaces.generated import slope_sim_interfaces_pb2 as interface_pb
from slope_sim.interfaces.ecal_transport import create_transport
from slope_sim.interfaces.logging import (
    InterfaceEventLogger,
    InterfaceLogRecord,
    InterfaceLogSnapshot,
    read_interface_log,
)
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPoint,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.interfaces.runtime import InterfaceRuntime
from slope_sim.interfaces.transport import Transport, TransportSnapshot, TransportTopicQuality
from slope_sim.lidar_pointcloud import LidarConfig, LidarScanResult, MultiLineLidar
from slope_sim.model_registry import get_robot_model, robot_model_names
from slope_sim.obstacles import (
    ObstacleGenerationRequest,
    ObstacleGenerationSettings,
    ObstacleGeometry,
    ObstacleManager,
    ObstaclePath,
    ObstacleSnapshot,
    ObstacleSpec,
    create_box_obstacle,
    update_kinematic_obstacle,
)
from slope_sim.robot import ActiveSteeringRobot, create_robot
from slope_sim.runtime_actions import (
    AddObstaclesAction,
    SwitchRobotAction,
    SwitchTerrainAction,
    TerrainSelection,
)
from slope_sim.scene import TerrainBounds, create_slope_scene, terrain_model_names
from slope_sim.scene_config import (
    SCENE_SCHEMA_VERSION,
    SceneDocument,
    SensorDocument,
    TerrainDocument,
    dump_scene_atomic,
    load_scene,
)
from slope_sim.sensor_backend import Pose, PyBulletSensorBackend, RayHit
from slope_sim.simulation import initial_scene_document, run_interface_physics_frame
from slope_sim.realtime import DeadlinePacer, RuntimeObservationCadence
from slope_sim.truth_sensors import MountPose, SensorMounts, TruthSensorSuite, wrap_angle


TIME_STEP = 1.0 / 240.0
WHEEL_SEND_FRAMES = 192
WHEEL_EXPECTED_COMMANDS = 80
STOP_SETTLE_FRAMES = 120
SCHEDULER_FRAMES = 2_400
EXPECTED_CHANNELS = (
    ("/sim/wheel/command", 100, "subscribe"),
    ("/sim/wheel/state", 100, "publish"),
    ("/sim/lidar/front/points", 10, "publish"),
    ("/sim/lidar/rear/points", 10, "publish"),
    ("/sim/rtk/state", 10, "publish"),
    ("/sim/imu/attitude", 10, "publish"),
)
EXPECTED_PROTO_FIELDS = {
    "WheelCommand": (("timestamp_ns", 1), ("drive_wheel_speed_rad_s", 2), ("steering_wheel_speed_rad_s", 3)),
    "WheelState": (("timestamp_ns", 1), ("drive_wheel_speed_rad_s", 2), ("steering_wheel_angle_rad", 3)),
    "LidarPoint": (("offset_time_ns", 1), ("x", 2), ("y", 3), ("z", 4), ("reflectivity", 5), ("tag", 6), ("line", 7)),
    "LidarPointCloud": (("timebase_ns", 1), ("frame_id", 2), ("point_num", 3), ("lidar_id", 4), ("points", 5)),
    "RtkState": (("timestamp_ns", 1), ("main_x", 2), ("main_y", 3), ("main_z", 4), ("baseline_yaw_rad", 5)),
    "ImuAttitude": (("timestamp_ns", 1), ("roll_rad", 2), ("pitch_rad", 3)),
}
PERFORMANCE_WARMUP_SEC = 1.0
PERFORMANCE_MEASUREMENT_SEC = 5.0
PERFORMANCE_LOG_SAMPLE_PERIOD_SEC = 0.100
PERFORMANCE_LOG_IDLE_TIMEOUT_SEC = 2.0


@dataclass(frozen=True, slots=True)
class VerificationCheck:
    """单项阶段三验收结果，字段严格且不可变。"""

    name: str
    passed: bool
    details: str = ""

    def __post_init__(self) -> None:
        if type(self.name) is not str or not self.name:
            raise ValueError("name must be a nonempty string")
        if type(self.passed) is not bool:
            raise ValueError("passed must be a bool")
        if type(self.details) is not str:
            raise ValueError("details must be a string")


@dataclass(frozen=True, slots=True)
class VerificationSummary:
    """稳定保存逐项输出及最终通过/失败计数。"""

    lines: tuple[str, ...]
    pass_count: int
    fail_count: int
    final_line: str


@dataclass(frozen=True, slots=True)
class _PerformanceLogQuality:
    """保存联合负载窗口的日志硬门禁证据。"""

    passed: bool
    accepted_messages: int
    minimum_accepted_messages: int
    final_pending: int
    max_pending: int
    failure_reasons: tuple[str, ...]


def summarize(checks: Sequence[VerificationCheck]) -> VerificationSummary:
    """生成稳定 PASS/FAIL 行，拒绝非验收结果混入汇总。"""
    items = tuple(checks)
    if any(type(check) is not VerificationCheck for check in items):
        raise ValueError("checks must contain exact VerificationCheck values")
    lines = tuple(
        f"{'PASS' if check.passed else 'FAIL'} {check.name} {check.details}".rstrip()
        for check in items
    )
    pass_count = sum(check.passed for check in items)
    fail_count = len(items) - pass_count
    return VerificationSummary(
        lines,
        pass_count,
        fail_count,
        f"SUMMARY pass={pass_count} fail={fail_count}",
    )


def exit_code(checks: Sequence[VerificationCheck]) -> int:
    """任一门禁失败时返回非零，供 CI 和人工命令直接判定。"""
    return 1 if any(not check.passed for check in checks) else 0


def _failure(name: str, exc: Exception) -> VerificationCheck:
    return VerificationCheck(name, False, f"{type(exc).__name__}: {exc}")


@contextmanager
def _direct_client() -> Iterator[int]:
    """为单个物理门禁创建隔离的 DIRECT client，并保证退出时断开。"""
    client_id = p.connect(p.DIRECT)
    if client_id < 0:
        raise RuntimeError("failed to connect to PyBullet DIRECT")
    try:
        yield client_id
    finally:
        try:
            p.disconnect(client_id)
        except p.error:
            pass


class _FakeMonotonic:
    """用有理数推进确定性墙钟，避免 240 Hz 浮点累计漂移。"""

    def __init__(self, initial: float = 0.0) -> None:
        self._seconds = Fraction(initial).limit_denominator(1_000_000_000)

    def __call__(self) -> float:
        return float(self._seconds)

    def advance(self, dt: float | Fraction) -> float:
        self._seconds += Fraction(dt).limit_denominator(1_000_000_000)
        return float(self._seconds)


class _VerifierSubscription:
    def __init__(self, callback: Callable[[bytes, float], bool | None]) -> None:
        self.callback = callback
        self.closed = False

    def close(self) -> None:
        self.closed = True


class _RecordingTransport:
    """验收脚本使用的同步窄传输，记录真实 runtime 发布的每个时间戳。"""

    def __init__(self, *, mode: str = "local") -> None:
        self.mode = mode
        self.published: list[tuple[str, bytes, str, int, float | None]] = []
        self.subscriptions: list[_VerifierSubscription] = []
        self.closed = False

    def subscribe(self, _topic: str, _type_name: str, callback) -> _VerifierSubscription:
        subscription = _VerifierSubscription(callback)
        self.subscriptions.append(subscription)
        return subscription

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float | None = None,
    ) -> bool:
        self.published.append((topic, bytes(payload), type_name, sim_time_ns, wall_time))
        return True

    def snapshot(self) -> TransportSnapshot:
        return TransportSnapshot(
            mode=self.mode,
            ecal_connected=self.mode == "ecal",
            published_count=len(self.published),
            received_count=0,
            error_count=0,
            dropped_count=0,
        )

    def quiesce(self) -> TransportSnapshot:
        return self.snapshot()

    def close(self) -> None:
        self.closed = True


class _VerifierRobot:
    """只实现 InterfaceRuntime 轮控端口，用于调度和界面数据契约检查。"""

    def __init__(self, model_name: str = "df_back") -> None:
        self.model_spec = get_robot_model(model_name)
        self.actual_drive = (0.0,) * len(self.model_spec.drive_joint_names)
        self.actual_steering = (0.0,) * len(self.model_spec.steering_joint_names)

    def command_wheel_speeds(
        self,
        drive_wheel_speeds: tuple[float, ...],
        steering_wheel_speeds: tuple[float, ...] = (),
        dt: float = TIME_STEP,
    ) -> tuple[float, ...]:
        self.actual_drive = tuple(drive_wheel_speeds)
        if self.actual_steering:
            limit = self.model_spec.max_steering_angle
            self.actual_steering = tuple(
                max(-limit, min(limit, angle + rate * dt))
                for angle, rate in zip(self.actual_steering, steering_wheel_speeds, strict=True)
            )
        return self.actual_drive

    def hold_current_steering_and_stop_drive(self, _dt: float) -> None:
        self.actual_drive = (0.0,) * len(self.actual_drive)

    def read_interface_wheel_state(self, timestamp_ns: int) -> WheelState:
        return WheelState(timestamp_ns, self.actual_drive, self.actual_steering)


class _RecordingLidar:
    """记录 runtime 触发时刻，并生成一条严格合法的企业点云。"""

    def __init__(self, frame_id: str, lidar_id: int) -> None:
        self.frame_id = frame_id
        self.lidar_id = lidar_id
        self.timestamps: list[int] = []

    def _message(self, timestamp_ns: int) -> LidarPointCloud:
        point = LidarPoint(0, 1.0, 0.0, 0.0, 100, 1, 0)
        return LidarPointCloud(timestamp_ns, self.frame_id, 1, self.lidar_id, (point,))

    def scan(self, timestamp_ns: int) -> LidarPointCloud:
        self.timestamps.append(timestamp_ns)
        return self._message(timestamp_ns)

    def scan_with_top_view(self, timestamp_ns: int) -> LidarScanResult:
        self.timestamps.append(timestamp_ns)
        return LidarScanResult(
            self._message(timestamp_ns),
            LidarTopViewFrame(
                timestamp_ns,
                (LidarTopViewPoint(1.0, 0.0, 1, self.lidar_id),),
            ),
        )


class _RecordingTruthSensors:
    """记录 RTK/IMU 调度时刻，并返回确定性有限真值。"""

    def __init__(self) -> None:
        self.rtk_timestamps: list[int] = []
        self.imu_timestamps: list[int] = []

    def read_rtk(self, timestamp_ns: int) -> RtkState:
        self.rtk_timestamps.append(timestamp_ns)
        return RtkState(timestamp_ns, 1.0, 2.0, 3.0, 0.25)

    def read_imu(self, timestamp_ns: int) -> ImuAttitude:
        self.imu_timestamps.append(timestamp_ns)
        return ImuAttitude(timestamp_ns, 0.1, -0.2)


def run_proto_and_topic_contract_check() -> VerificationCheck:
    """核对生成描述符、六话题顺序，并做编解码冒烟。"""
    name = "proto_and_topic_contract"
    try:
        descriptor = descriptor_pb2.FileDescriptorProto()
        descriptor.ParseFromString(interface_pb.DESCRIPTOR.serialized_pb)
        actual_fields = {
            message.name: tuple((field.name, field.number) for field in message.field)
            for message in descriptor.message_type
        }
        config = InterfaceConfig.default()
        actual_channels = tuple(
            (channel.topic, channel.rate_hz, channel.direction)
            for channel in config.channels
        )
        codec = ProtoCodec()
        command = WheelCommand(17, (1.0, 2.0), ())
        roundtrip = codec.decode_wheel_command(codec.encode(command))
        passed = (
            descriptor.name == "slope_sim_interfaces.proto"
            and descriptor.syntax == "proto3"
            and descriptor.package == "slope_sim.interfaces.v1"
            and actual_fields == EXPECTED_PROTO_FIELDS
            and actual_channels == EXPECTED_CHANNELS
            and roundtrip == command
        )
        return VerificationCheck(
            name,
            passed,
            f"package={descriptor.package} messages={len(actual_fields)} topics={len(actual_channels)}",
        )
    except Exception as exc:
        return _failure(name, exc)


def _create_settled_robot(client_id: int, model_name: str):
    """创建高摩擦平地车型，并静置到稳定接触。"""
    scene = create_slope_scene(
        client_id,
        slope_deg=0.0,
        time_step=TIME_STEP,
        ground_lateral_friction=1.4,
        terrain_model="flat",
    )
    spec = get_robot_model(model_name)
    robot = create_robot(
        client_id,
        model_name,
        start_x=scene.spawn_position[0],
        start_y=scene.spawn_position[1],
        base_height=scene.spawn_position[2] + spec.base_height,
        start_orientation=scene.spawn_orientation,
        drive_motor_force=8.0,
    )
    robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
    zero_drive = (0.0,) * len(spec.drive_joint_names)
    zero_steering = (0.0,) * len(spec.steering_joint_names)
    for settle_step in range(120):
        robot.command_wheel_speeds(zero_drive, zero_steering, dt=TIME_STEP)
        p.stepSimulation(physicsClientId=client_id)
        validate_robot_pose(
            client_id,
            robot,
            scene,
            require_ground_contact=settle_step >= 30,
        )
    return scene, robot


def _send_due_command(
    runtime: InterfaceRuntime,
    wall_clock: _FakeMonotonic,
    send_clock: SimulationClock,
    scheduler: PeriodicScheduler,
    drive: tuple[float, ...],
    steering: tuple[float, ...],
) -> tuple[int, float | None]:
    now_ns = send_clock.advance(TIME_STEP)
    sent = 0
    last_send_at: float | None = None
    for timestamp_ns in scheduler.pop_due(now_ns):
        last_send_at = wall_clock()
        if not runtime.accept_local_command(
            WheelCommand(timestamp_ns, drive, steering),
            received_at=last_send_at,
        ):
            raise AssertionError("valid wheel command was rejected")
        sent += 1
    return sent, last_send_at


def _run_wheel_check(model_name: str) -> VerificationCheck:
    name = f"wheel_{model_name}"
    runtime: InterfaceRuntime | None = None
    try:
        with ExitStack() as stack:
            client_id = stack.enter_context(_direct_client())
            scene, robot = _create_settled_robot(client_id, model_name)
            wall_clock = _FakeMonotonic()
            runtime = InterfaceRuntime.local_for_robot(robot, monotonic=wall_clock)
            stack.callback(runtime.close)
            send_clock = SimulationClock()
            sender = PeriodicScheduler(100)
            spec = get_robot_model(model_name)
            if spec.controller_kind == "active_steering":
                drive = (3.0, 4.0, 5.0, 6.0)
                steering = (0.5, -0.5)
            else:
                drive = (6.0, 6.0)
                steering = ()
            initial, _ = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
            command_count = 0
            for _ in range(WHEEL_SEND_FRAMES):
                wall_clock.advance(TIME_STEP)
                sent, _ = _send_due_command(
                    runtime,
                    wall_clock,
                    send_clock,
                    sender,
                    drive,
                    steering,
                )
                command_count += sent
                decision = runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
                if decision.timed_out:
                    raise AssertionError("wheel command timed out while sender was active")
                p.stepSimulation(physicsClientId=client_id)
                runtime.after_physics_step(TIME_STEP)
                validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
            final, _ = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
            state = runtime.last_wheel_state
            if state is None:
                raise AssertionError("wheel state was not published")
            displacement = math.dist(initial[:2], final[:2])
            drive_error = max(
                (abs(actual - expected) for actual, expected in zip(state.drive_wheel_speed_rad_s, drive, strict=True)),
                default=0.0,
            )
            steering_ok = (
                state.steering_wheel_angle_rad == ()
                if not steering
                else len(state.steering_wheel_angle_rad) == 2
                and state.steering_wheel_angle_rad[0] > 0.30
                and state.steering_wheel_angle_rad[1] < -0.30
            )
            passed = (
                command_count == WHEEL_EXPECTED_COMMANDS
                and runtime.status_snapshot(wall_time=wall_clock()).command.valid_count
                == WHEEL_EXPECTED_COMMANDS
                and displacement > 0.10
                and drive_error <= (0.30 if steering else 0.25)
                and steering_ok
            )
            return VerificationCheck(
                name,
                passed,
                f"commands={command_count} displacement={displacement:.3f}m drive_error={drive_error:.3f}",
            )
    except Exception as exc:
        return _failure(name, exc)


def run_four_model_wheel_checks() -> tuple[VerificationCheck, ...]:
    """在全部四种正式车型上验证真实物理轮速反馈和位移。"""
    return tuple(_run_wheel_check(model_name) for model_name in robot_model_names())


def run_timeout_and_steering_hold_check() -> VerificationCheck:
    """验证主动转向车超时停车时保持实际角，而不是追赶旧目标角。"""
    name = "timeout_and_steering_hold"
    runtime: InterfaceRuntime | None = None
    try:
        with ExitStack() as stack:
            client_id = stack.enter_context(_direct_client())
            scene, robot = _create_settled_robot(client_id, "active_steering_4wd")
            if not isinstance(robot, ActiveSteeringRobot):
                raise AssertionError("active steering model created the wrong robot type")
            wall_clock = _FakeMonotonic()
            runtime = InterfaceRuntime.local_for_robot(robot, monotonic=wall_clock)
            stack.callback(runtime.close)
            send_clock = SimulationClock()
            sender = PeriodicScheduler(100)
            last_send_at: float | None = None
            for _ in range(48):
                wall_clock.advance(TIME_STEP)
                _, frame_send_at = _send_due_command(
                    runtime,
                    wall_clock,
                    send_clock,
                    sender,
                    (2.0, 3.0, 4.0, 5.0),
                    (1.0, -1.0),
                )
                if frame_send_at is not None:
                    last_send_at = frame_send_at
                runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
                p.stepSimulation(physicsClientId=client_id)
                runtime.after_physics_step(TIME_STEP)
            if last_send_at is None:
                raise AssertionError("warmup did not send a command")
            # 保持旧命令仍有效 23 帧，再用大 dt 拉开目标角与实际角。
            for _ in range(23):
                wall_clock.advance(TIME_STEP)
                if wall_clock() - last_send_at >= 0.100:
                    raise AssertionError("timeout warmup crossed the command deadline")
                active = runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
                if active.timed_out:
                    raise AssertionError("command timed out before the 100 ms boundary")
                p.stepSimulation(physicsClientId=client_id)
                runtime.after_physics_step(TIME_STEP)
                validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
            if runtime.before_physics_step(0.5, wall_time=wall_clock()).timed_out:
                raise AssertionError("target-lag setup unexpectedly timed out")
            for _ in range(60):
                wall_clock.advance(TIME_STEP)
                old_target = tuple(robot._steering_targets)
                actual_before = robot.read_steering_wheel_angles()
                drive_before = robot.read_interface_wheel_state(runtime.clock.now_ns).drive_wheel_speed_rad_s
                decision = runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
                if decision.timed_out:
                    timeout_age = wall_clock() - last_send_at
                    break
                p.stepSimulation(physicsClientId=client_id)
                runtime.after_physics_step(TIME_STEP)
            else:
                raise AssertionError("command timeout did not occur")
            held_target = tuple(robot._steering_targets)
            p.stepSimulation(physicsClientId=client_id)
            runtime.after_physics_step(TIME_STEP)
            for _ in range(STOP_SETTLE_FRAMES):
                wall_clock.advance(TIME_STEP)
                if not runtime.before_physics_step(TIME_STEP, wall_time=wall_clock()).timed_out:
                    raise AssertionError("timed-out decision unexpectedly reactivated")
                p.stepSimulation(physicsClientId=client_id)
                runtime.after_physics_step(TIME_STEP)
                validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
            stopped = robot.read_interface_wheel_state(runtime.clock.now_ns)
            passed = (
                0.100 <= timeout_age <= 0.100 + TIME_STEP
                and all(abs(speed) > 1.0 for speed in drive_before)
                and any(abs(target - angle) > 0.10 for target, angle in zip(old_target, actual_before, strict=True))
                and decision.drive_wheel_speed_rad_s == (0.0, 0.0, 0.0, 0.0)
                and max(abs(target - angle) for target, angle in zip(held_target, actual_before, strict=True)) <= 1e-12
                and max(abs(speed) for speed in stopped.drive_wheel_speed_rad_s) <= 0.01
                and max(abs(angle - held) for angle, held in zip(stopped.steering_wheel_angle_rad, actual_before, strict=True)) <= 0.03
            )
            return VerificationCheck(
                name,
                passed,
                f"timeout_age={timeout_age:.6f}s drive_before={min(abs(v) for v in drive_before):.3f} stop_max={max(abs(v) for v in stopped.drive_wheel_speed_rad_s):.4f}",
            )
    except Exception as exc:
        return _failure(name, exc)


def run_100_10_hz_scheduler_check() -> VerificationCheck:
    """按十秒仿真时间检查 100/10 Hz 帧数及双雷达半周期错相。"""
    name = "scheduler_100_10_hz"
    transport = _RecordingTransport()
    wall_clock = _FakeMonotonic()
    runtime = InterfaceRuntime(
        _VerifierRobot(),
        config=InterfaceConfig.default(transport_mode="local"),
        transport=transport,
        monotonic=wall_clock,
        capture_lidar_top_view=False,
    )
    front = _RecordingLidar("lidar_front", 1)
    rear = _RecordingLidar("lidar_rear", 2)
    truth = _RecordingTruthSensors()
    runtime._front_lidar = front
    runtime._rear_lidar = rear
    runtime._truth_sensor_suite = truth
    send_clock = SimulationClock()
    sender = PeriodicScheduler(100)
    try:
        command_count = 0
        for _ in range(SCHEDULER_FRAMES):
            wall_clock.advance(TIME_STEP)
            sent, _ = _send_due_command(
                runtime,
                wall_clock,
                send_clock,
                sender,
                (2.0, 2.0),
                (),
            )
            command_count += sent
            runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
            runtime.after_physics_step(TIME_STEP)
        config = runtime.config
        published_by_topic: dict[str, list[int]] = defaultdict(list)
        for topic, _payload, _type_name, timestamp_ns, _wall_time in transport.published:
            published_by_topic[topic].append(timestamp_ns)
        status = runtime.status_snapshot(wall_time=wall_clock())
        wheel_expected = [index * 10_000_000 for index in range(1, 1_001)]
        front_expected = [index * 100_000_000 for index in range(1, 101)]
        rear_expected = [50_000_000 + index * 100_000_000 for index in range(100)]
        counts_ok = (
            command_count == 1_000
            and status.topics[config.wheel_command.topic].message_count == 1_000
            and published_by_topic[config.wheel_state.topic] == wheel_expected
            and front.timestamps == front_expected
            and rear.timestamps == rear_expected
            and truth.rtk_timestamps == front_expected
            and truth.imu_timestamps == front_expected
        )
        sensor_topic_counts = tuple(
            len(published_by_topic[channel.topic])
            for channel in (config.lidar_front, config.lidar_rear, config.rtk, config.imu)
        )
        counts_ok = counts_ok and sensor_topic_counts == (100, 100, 100, 100)
        return VerificationCheck(
            name,
            counts_ok,
            f"wheel_command={command_count} wheel_state={len(published_by_topic[config.wheel_state.topic])} sensors={sensor_topic_counts} rear_first_ms={rear.timestamps[0] / 1e6:.0f}",
        )
    except Exception as exc:
        return _failure(name, exc)
    finally:
        runtime.close()


class _RecordingPyBulletSensorBackend(PyBulletSensorBackend):
    """保留正式射线结果，供验收同时核对消息语义和原始 body id。"""

    def __init__(self, client_id: int, robot_id: int) -> None:
        super().__init__(client_id, robot_id)
        self.ray_history: list[tuple[RayHit, ...]] = []

    def ray_test_batch(self, starts, ends, *, collision_mask: int) -> tuple[RayHit, ...]:
        hits = super().ray_test_batch(starts, ends, collision_mask=collision_mask)
        self.ray_history.append(hits)
        return hits


def _create_settled_sensor_world(
    client_id: int,
    terrain_model: str,
    robot_model: str = "df_back",
):
    """创建正式地形和车型并稳定接触，随后再读取传感器安装位姿。"""
    scene = create_slope_scene(
        client_id,
        slope_deg=8.0 if terrain_model == "slope" else 0.0,
        time_step=TIME_STEP,
        terrain_model=terrain_model,
        golf_seed=31,
        golf_relief="medium",
    )
    spec = get_robot_model(robot_model)
    robot = create_robot(
        client_id,
        robot_model,
        start_x=scene.spawn_position[0],
        start_y=scene.spawn_position[1],
        base_height=scene.spawn_position[2] + spec.base_height,
        start_orientation=scene.spawn_orientation,
    )
    robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
    zero_drive = (0.0,) * len(spec.drive_joint_names)
    zero_steering = (0.0,) * len(spec.steering_joint_names)
    for settle_step in range(180):
        robot.command_wheel_speeds(zero_drive, zero_steering, dt=TIME_STEP)
        p.stepSimulation(physicsClientId=client_id)
        if settle_step >= 120:
            validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
    return scene, robot


def _world_mount(backend: PyBulletSensorBackend, mount: MountPose) -> Pose:
    return backend.transform_pose(
        backend.world_pose(mount.parent_link),
        Pose(mount.position, mount.orientation),
    )


def _point_from_mount(
    backend: PyBulletSensorBackend,
    mount: Pose,
    local_point: tuple[float, float, float],
) -> tuple[float, float, float]:
    return backend.transform_pose(
        mount,
        Pose(local_point, (0.0, 0.0, 0.0, 1.0)),
    ).position


def _run_terrain_lidar_check(terrain_model: str) -> VerificationCheck:
    name = f"lidar_{terrain_model}"
    try:
        with _direct_client() as client_id:
            scene, robot = _create_settled_sensor_world(client_id, terrain_model)
            backend = _RecordingPyBulletSensorBackend(client_id, robot.robot_id)
            backend.bind_scene(scene.body_ids, ())
            config = LidarConfig.default()
            front = MultiLineLidar.front(backend, config).scan(100_000_000)
            rear = MultiLineLidar.rear(backend, config).scan(100_000_000)
            front_hits, rear_hits = backend.ray_history
            front_terrain_hits = sum(hit.body_id in scene.body_ids for hit in front_hits)
            rear_terrain_hits = sum(hit.body_id in scene.body_ids for hit in rear_hits)
            self_hits = sum(
                hit.body_id == robot.robot_id
                for hits in (front_hits, rear_hits)
                for hit in hits
            )
            finite = all(
                math.isfinite(value)
                for cloud in (front, rear)
                for point in cloud.points
                for value in (point.x, point.y, point.z)
            )
            passed = (
                self_hits == 0
                and front_terrain_hits > 0
                and rear_terrain_hits > 0
                and front.point_num == len(front.points) == front_terrain_hits
                and rear.point_num == len(rear.points) == rear_terrain_hits
                and {point.tag for point in front.points} == {1}
                and {point.tag for point in rear.points} == {1}
                and finite
            )
            return VerificationCheck(
                name,
                passed,
                f"front_points={front.point_num} rear_points={rear.point_num} self_hits={self_hits}",
            )
    except Exception as exc:
        return _failure(name, exc)


def run_three_terrain_lidar_checks() -> tuple[VerificationCheck, ...]:
    """在平地、分段坡和高尔夫高度场分别生成前后完整点云。"""
    return tuple(_run_terrain_lidar_check(terrain) for terrain in terrain_model_names())


def _obstacle_visibility_case(mode: str, expected_tag: int) -> tuple[int, int]:
    """把同语义障碍物分别放到前后局部正向，返回两侧命中点数。"""
    with _direct_client() as client_id:
        scene, robot = _create_settled_sensor_world(client_id, "flat")
        backend = _RecordingPyBulletSensorBackend(client_id, robot.robot_id)
        mounts = SensorMounts.default()
        front_position = _point_from_mount(
            backend,
            _world_mount(backend, mounts.lidar_front),
            (2.0, 0.0, 0.05),
        )
        rear_position = _point_from_mount(
            backend,
            _world_mount(backend, mounts.lidar_rear),
            (2.0, 0.0, 0.05),
        )
        front_body = create_box_obstacle(
            client_id,
            half_extents=(0.25, 0.25, 0.30),
            position=front_position,
        )
        rear_body = create_box_obstacle(
            client_id,
            half_extents=(0.25, 0.25, 0.30),
            position=rear_position,
        )
        snapshots = (
            ObstacleSnapshot(1, front_body, mode, "box", front_position, (0.0, 0.0, 0.0, 1.0)),
            ObstacleSnapshot(2, rear_body, mode, "box", rear_position, (0.0, 0.0, 0.0, 1.0)),
        )
        backend.bind_scene(scene.body_ids, snapshots)
        front = MultiLineLidar.front(backend, LidarConfig.default()).scan(200_000_000)
        rear = MultiLineLidar.rear(backend, LidarConfig.default()).scan(200_000_000)
        front_hits, rear_hits = backend.ray_history
        front_bodies = {hit.body_id for hit in front_hits if hit.hit}
        rear_bodies = {hit.body_id for hit in rear_hits if hit.hit}
        if not (
            expected_tag in {point.tag for point in front.points}
            and expected_tag in {point.tag for point in rear.points}
            and front_body in front_bodies
            and rear_body in rear_bodies
            and rear_body not in front_bodies
            and front_body not in rear_bodies
        ):
            raise AssertionError(f"{mode} obstacle field-of-view semantics failed")
        return (
            sum(point.tag == expected_tag for point in front.points),
            sum(point.tag == expected_tag for point in rear.points),
        )


def _moving_obstacle_ranges() -> tuple[float, ...]:
    """连续移动真实 body 五次，返回每帧最近的前雷达局部量程。"""
    with _direct_client() as client_id:
        scene, robot = _create_settled_sensor_world(client_id, "flat")
        backend = _RecordingPyBulletSensorBackend(client_id, robot.robot_id)
        mount = _world_mount(backend, SensorMounts.default().lidar_front)
        initial = _point_from_mount(backend, mount, (1.70, 0.0, 0.05))
        next_position = _point_from_mount(backend, mount, (1.73, 0.0, 0.05))
        velocity = tuple((after - before) / 0.1 for before, after in zip(initial, next_position, strict=True))
        body_id = create_box_obstacle(
            client_id,
            half_extents=(0.25, 0.25, 0.30),
            position=initial,
        )
        backend.bind_scene(
            scene.body_ids,
            (ObstacleSnapshot(1, body_id, "moving", "box", initial, (0.0, 0.0, 0.0, 1.0)),),
        )
        scanner = MultiLineLidar.front(backend, LidarConfig.default())
        closest: list[float] = []
        for index in range(5):
            position = _point_from_mount(backend, mount, (1.70 + 0.03 * index, 0.0, 0.05))
            update_kinematic_obstacle(
                client_id,
                body_id,
                position=position,
                orientation=(0.0, 0.0, 0.0, 1.0),
                linear_velocity=velocity,
            )
            p.performCollisionDetection(physicsClientId=client_id)
            cloud = scanner.scan(index * 100_000_000)
            obstacle_points = tuple(point for point in cloud.points if point.tag == 3)
            if not obstacle_points:
                raise AssertionError("moving obstacle disappeared from front cloud")
            closest.append(
                min(math.sqrt(point.x**2 + point.y**2 + point.z**2) for point in obstacle_points)
            )
        return tuple(closest)


def run_static_and_moving_obstacle_lidar_check() -> VerificationCheck:
    """验证障碍物语义标签、前后视场隔离和移动点云连续变化。"""
    name = "static_and_moving_obstacle_lidar"
    try:
        static_counts = _obstacle_visibility_case("static", 2)
        moving_counts = _obstacle_visibility_case("moving", 3)
        ranges = _moving_obstacle_ranges()
        deltas = tuple(abs(right - left) for left, right in zip(ranges, ranges[1:]))
        passed = (
            all(count > 0 for count in (*static_counts, *moving_counts))
            and len(ranges) == 5
            and all(delta > 0.0 for delta in deltas)
            and max(deltas) < 0.10
        )
        return VerificationCheck(
            name,
            passed,
            f"static={static_counts} moving={moving_counts} sequence={len(ranges)} max_delta={max(deltas):.4f}m",
        )
    except Exception as exc:
        return _failure(name, exc)


@dataclass(frozen=True, slots=True)
class _CollisionRun:
    terrain_contacts: int
    obstacle_contacts: int
    final_pose: tuple[float, ...]


def _run_lidar_collision_case(*, lidar_visible: bool) -> _CollisionRun:
    """运行唯一变量为 LiDAR 可见位的确定性车辆碰撞场景。"""
    with _direct_client() as client_id:
        scene = create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=TIME_STEP,
            terrain_model="flat",
        )
        spec = get_robot_model("df_back")
        robot = create_robot(
            client_id,
            "df_back",
            start_x=-1.25,
            start_y=0.0,
            base_height=scene.spawn_position[2] + spec.base_height,
            start_orientation=scene.spawn_orientation,
        )
        robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
        for _ in range(120):
            robot.command_twist(0.0, 0.0, dt=TIME_STEP)
            p.stepSimulation(physicsClientId=client_id)
        obstacle_id = create_box_obstacle(
            client_id,
            half_extents=(0.12, 0.65, 0.35),
            position=(0.0, 0.0, 0.35),
        )
        terrain_group = scene_module.TERRAIN_COLLISION_GROUP if lidar_visible else 0x2 | 0x8
        obstacle_group = obstacle_module.OBSTACLE_COLLISION_GROUP if lidar_visible else 0x2
        for terrain_body in scene.body_ids:
            p.setCollisionFilterGroupMask(
                terrain_body,
                -1,
                terrain_group,
                scene_module.TERRAIN_COLLISION_MASK,
                physicsClientId=client_id,
            )
        p.setCollisionFilterGroupMask(
            obstacle_id,
            -1,
            obstacle_group,
            obstacle_module.OBSTACLE_COLLISION_MASK,
            physicsClientId=client_id,
        )
        terrain_contacts = 0
        obstacle_contacts = 0
        for _ in range(720):
            robot.command_twist(0.6, 0.0, dt=TIME_STEP)
            p.stepSimulation(physicsClientId=client_id)
            terrain_contacts += sum(
                len(
                    p.getContactPoints(
                        bodyA=robot.robot_id,
                        bodyB=terrain_body,
                        physicsClientId=client_id,
                    )
                )
                for terrain_body in scene.body_ids
            )
            obstacle_contacts += len(
                p.getContactPoints(
                    bodyA=robot.robot_id,
                    bodyB=obstacle_id,
                    physicsClientId=client_id,
                )
            )
        position, orientation = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )
        return _CollisionRun(
            terrain_contacts,
            obstacle_contacts,
            tuple(float(value) for value in (*position, *orientation)),
        )


def run_lidar_collision_contact_check() -> VerificationCheck:
    """证明新增 0x10 可见位不改变实体接触数和最终位姿。"""
    name = "lidar_collision_contact"
    try:
        baseline = _run_lidar_collision_case(lidar_visible=False)
        visible = _run_lidar_collision_case(lidar_visible=True)
        pose_error = max(
            abs(left - right)
            for left, right in zip(baseline.final_pose, visible.final_pose, strict=True)
        )
        passed = (
            scene_module.TERRAIN_COLLISION_GROUP & scene_module.LIDAR_VISIBLE_GROUP != 0
            and obstacle_module.OBSTACLE_COLLISION_GROUP & scene_module.LIDAR_VISIBLE_GROUP != 0
            and baseline.terrain_contacts > 0
            and baseline.obstacle_contacts > 0
            and visible.terrain_contacts == baseline.terrain_contacts
            and visible.obstacle_contacts == baseline.obstacle_contacts
            and pose_error <= 1e-6
        )
        return VerificationCheck(
            name,
            passed,
            f"terrain_contacts={visible.terrain_contacts} obstacle_contacts={visible.obstacle_contacts} pose_error={pose_error:.2e}",
        )
    except Exception as exc:
        return _failure(name, exc)


def _world_base_link_pose(client_id: int, robot_id: int):
    """按 Bullet 惯性 frame 关系独立恢复 URDF base_link 世界位姿。"""
    inertial_position, inertial_orientation = p.getBasePositionAndOrientation(
        robot_id,
        physicsClientId=client_id,
    )
    dynamics = p.getDynamicsInfo(robot_id, -1, physicsClientId=client_id)
    inverse_position, inverse_orientation = p.invertTransform(dynamics[3], dynamics[4])
    return p.multiplyTransforms(
        inertial_position,
        inertial_orientation,
        inverse_position,
        inverse_orientation,
    )


def _truth_errors(robot_model: str, terrain_model: str) -> tuple[float, float, float, float]:
    with _direct_client() as client_id:
        scene, robot = _create_settled_sensor_world(client_id, terrain_model, robot_model)
        mounts = SensorMounts.default()
        suite = TruthSensorSuite(PyBulletSensorBackend(client_id, robot.robot_id), mounts)
        rtk = suite.read_rtk(123_000_000)
        imu = suite.read_imu(123_000_000)
        base_position, base_orientation = _world_base_link_pose(client_id, robot.robot_id)
        primary, _ = p.multiplyTransforms(
            base_position,
            base_orientation,
            mounts.rtk_primary.position,
            mounts.rtk_primary.orientation,
        )
        secondary, _ = p.multiplyTransforms(
            base_position,
            base_orientation,
            mounts.rtk_secondary.position,
            mounts.rtk_secondary.orientation,
        )
        _, imu_orientation = p.multiplyTransforms(
            base_position,
            base_orientation,
            mounts.imu.position,
            mounts.imu.orientation,
        )
        expected_roll, expected_pitch, _ = p.getEulerFromQuaternion(imu_orientation)
        expected_yaw = wrap_angle(
            math.atan2(secondary[1] - primary[1], secondary[0] - primary[0])
        )
        return (
            math.dist((rtk.main_x, rtk.main_y, rtk.main_z), primary),
            abs(wrap_angle(rtk.baseline_yaw_rad - expected_yaw)),
            abs(wrap_angle(imu.roll_rad - expected_roll)),
            abs(wrap_angle(imu.pitch_rad - expected_pitch)),
        )


def _run_terrain_truth_check(terrain_model: str, tolerance: float) -> VerificationCheck:
    name = f"truth_{terrain_model}"
    try:
        errors = tuple(_truth_errors(model, terrain_model) for model in robot_model_names())
        maxima = tuple(max(values) for values in zip(*errors, strict=True))
        passed = all(error <= tolerance for error in maxima)
        return VerificationCheck(
            name,
            passed,
            "models=4 "
            f"rtk_position={maxima[0]:.2e} rtk_yaw={maxima[1]:.2e} "
            f"imu_roll={maxima[2]:.2e} imu_pitch={maxima[3]:.2e} tolerance={tolerance:.1e}",
        )
    except Exception as exc:
        return _failure(name, exc)


def run_three_terrain_truth_sensor_checks(
    *,
    tolerance: float,
) -> tuple[VerificationCheck, ...]:
    """每个地形聚合四车型真值误差，并保持计划要求的三项输出。"""
    if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)):
        raise ValueError("tolerance must be a positive finite number")
    normalized = float(tolerance)
    if not math.isfinite(normalized) or normalized <= 0.0:
        raise ValueError("tolerance must be a positive finite number")
    return tuple(
        _run_terrain_truth_check(terrain, normalized)
        for terrain in terrain_model_names()
    )


@dataclass(slots=True)
class _RuntimeWorldBundle:
    """三个复杂门禁共享的正式 DIRECT runtime/协调器所有权集合。"""

    client_id: int
    config: ExperimentConfig
    document: SceneDocument
    manager: ObstacleManager
    transport: Transport
    runtime: InterfaceRuntime
    coordinator: SimulationCoordinator
    logger: InterfaceEventLogger | None


@contextmanager
def _runtime_world(
    directory: Path,
    *,
    robot_model: str = "df_back",
    capture_lidar_top_view: bool,
    with_logger: bool,
) -> Iterator[_RuntimeWorldBundle]:
    """构建正式物理世界、传输、后端和 runtime，并按所有权逆序清理。"""
    client_id = p.connect(p.DIRECT)
    if client_id < 0:
        raise RuntimeError("failed to connect to PyBullet DIRECT")
    runtime: InterfaceRuntime | None = None
    transport = None
    backend = None
    logger: InterfaceEventLogger | None = None
    try:
        config = ExperimentConfig(
            mode="direct",
            robot_model=robot_model,
            terrain_model="flat",
            interface_mode="local",
            interface_enabled=True,
            interface_log_enabled=with_logger,
            dashboard_enabled=True,
            log_dir=directory,
            figure_dir=directory,
        )
        document = initial_scene_document(config)
        world, manager = build_world_from_scene_document(client_id, config, document)
        interface_config = InterfaceConfig.default(transport_mode="local")
        transport = create_transport("local", config=interface_config)
        backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
        backend.bind_scene(
            world.scene.body_ids,
            manager.snapshot(include_body_id=True),
        )
        if with_logger:
            logger = InterfaceEventLogger(
                directory,
                prefix="stage3-gate",
                queue_size=interface_config.log_queue_size,
            )
        runtime = InterfaceRuntime(
            world.active_robot.robot,
            config=interface_config,
            transport=transport,
            sensor_backend=backend,
            scene_document=document,
            logger=logger,
            capture_lidar_top_view=capture_lidar_top_view,
        )
        coordinator = SimulationCoordinator(
            client_id,
            config,
            world,
            manager,
            interface_runtime=runtime,
            sensor_document=document.sensors,
        )
        yield _RuntimeWorldBundle(
            client_id=client_id,
            config=config,
            document=document,
            manager=manager,
            transport=transport,
            runtime=runtime,
            coordinator=coordinator,
            logger=logger,
        )
    finally:
        if runtime is not None:
            runtime.close()
        else:
            for resource in (logger, transport, backend):
                close = None if resource is None else getattr(resource, "close", None)
                if callable(close):
                    try:
                        close()
                    except Exception:
                        pass
        try:
            p.disconnect(client_id)
        except p.error:
            pass


def _run_local_frames(bundle: _RuntimeWorldBundle, count: int) -> None:
    """按正式入口顺序推进指定数量的 local 接口物理帧。"""
    observation_cadence = RuntimeObservationCadence()
    for _ in range(count):
        run_interface_physics_frame(
            bundle.runtime,
            bundle.coordinator,
            actual_transport_mode="local",
            linear_velocity=0.25,
            angular_velocity=0.05,
            dt=TIME_STEP,
            observation_cadence=observation_cadence,
        )


def _dashboard_payloads(snapshot) -> tuple[object | None, ...]:
    return (
        snapshot.wheel_command,
        snapshot.wheel_state,
        snapshot.lidar_front,
        snapshot.lidar_rear,
        snapshot.rtk,
        snapshot.imu,
        snapshot.lidar_front_view,
        snapshot.lidar_rear_view,
    )


def run_pause_rebuild_and_edge_switch_check() -> VerificationCheck:
    """验证暂停冻结、超时恢复、车型重建及边界布局场地切换。"""
    name = "pause_rebuild_and_edge_switch"
    try:
        with tempfile.TemporaryDirectory(prefix="stage3-lifecycle-") as temporary:
            with _runtime_world(
                Path(temporary),
                capture_lidar_top_view=False,
                with_logger=False,
            ) as bundle:
                _run_local_frames(bundle, 30)
                runtime = bundle.runtime
                before = runtime.dashboard_snapshot()
                before_clock = runtime.clock.now_ns
                before_counts = tuple(
                    status.message_count for status in before.status.topics.values()
                )
                before_polls = runtime.connection_polls
                runtime.pause()
                time.sleep(runtime.config.command_timeout_sec + 0.02)
                for _ in range(3):
                    runtime.poll_transport()
                    if runtime.before_physics_step(TIME_STEP) is not None:
                        raise AssertionError("paused runtime returned a wheel decision")
                    if runtime.after_physics_step(TIME_STEP) != ():
                        raise AssertionError("paused runtime published physical topics")
                paused = runtime.dashboard_snapshot()
                frozen = (
                    runtime.clock.now_ns == before_clock
                    and tuple(status.message_count for status in paused.status.topics.values())
                    == before_counts
                    and _dashboard_payloads(paused) == _dashboard_payloads(before)
                    and runtime.connection_polls == before_polls + 3
                )
                resumed = runtime.resume()
                switch = bundle.coordinator.apply_action(
                    SwitchRobotAction("active_steering_4wd")
                )
                rebuilt = runtime.dashboard_snapshot()
                rebuild_ok = (
                    switch.state_changed
                    and switch.world_reset
                    and switch.error_message is None
                    and runtime.robot_model.name == "active_steering_4wd"
                    and runtime.bound_robot_id
                    == bundle.coordinator.world.active_robot.robot.robot_id
                    and rebuilt.generation > before.generation
                    and runtime.last_decision.waiting
                    and all(value is None for value in _dashboard_payloads(rebuilt))
                )
                _run_local_frames(bundle, 30)
                rebuilt_live = runtime.dashboard_snapshot()
                new_shape_ok = (
                    rebuilt_live.wheel_state is not None
                    and len(rebuilt_live.wheel_state.drive_wheel_speed_rad_s) == 4
                    and len(rebuilt_live.wheel_state.steering_wheel_angle_rad) == 2
                )

                edge_obstacle = ObstacleSpec(
                    901,
                    "static",
                    ObstacleGeometry("box", (0.20, 0.20, 0.25)),
                    (8.5, 0.0, 0.25),
                    (0.0, 0.0, 0.0, 1.0),
                )
                flat_with_edge_obstacle = replace(
                    bundle.coordinator.logical_scene_document(),
                    terrain=TerrainDocument("flat", 0.0, 0, "medium"),
                    obstacles=(edge_obstacle,),
                )
                installed = bundle.coordinator.apply_scene_document(flat_with_edge_obstacle)
                if installed.error_message is not None:
                    raise AssertionError(
                        f"edge-switch baseline failed: {installed.error_message}"
                    )
                previous = bundle.coordinator.logical_scene_document()
                edge_switch = bundle.coordinator.apply_action(
                    SwitchTerrainAction(
                        TerrainSelection("golf_heightfield", golf_seed=3)
                    )
                )
                edge_document = bundle.coordinator.logical_scene_document()
                edge_snapshot = runtime.dashboard_snapshot()
                edge_switch_ok = (
                    edge_switch.state_changed
                    and edge_switch.world_reset
                    and edge_switch.error_message is None
                    and edge_document.terrain
                    == TerrainDocument("golf_heightfield", 0.0, 3, "medium")
                    and tuple(
                        (
                            obstacle.logical_id,
                            obstacle.mode,
                            obstacle.geometry,
                            obstacle.position[:2],
                            obstacle.path,
                        )
                        for obstacle in edge_document.obstacles
                    )
                    == tuple(
                        (
                            obstacle.logical_id,
                            obstacle.mode,
                            obstacle.geometry,
                            obstacle.position[:2],
                            obstacle.path,
                        )
                        for obstacle in previous.obstacles
                    )
                    and runtime.scene_document == edge_document
                    and runtime.bound_robot_id
                    == bundle.coordinator.world.active_robot.robot.robot_id
                    and runtime.last_decision.waiting
                    and all(value is None for value in _dashboard_payloads(edge_snapshot))
                )
                passed = (
                    frozen
                    and resumed.timed_out
                    and rebuild_ok
                    and new_shape_ok
                    and edge_switch_ok
                )
                return VerificationCheck(
                    name,
                    passed,
                    f"paused_clock_ns={before_clock} polls={runtime.connection_polls - before_polls} generation={edge_snapshot.generation} edge_switch={edge_switch_ok}",
                )
    except Exception as exc:
        return _failure(name, exc)


def _representative_scene_document() -> SceneDocument:
    moving = ObstacleSpec(
        1,
        "moving",
        ObstacleGeometry("cylinder", (0.20, 0.20, 0.40)),
        (1.5, -0.5, 0.40),
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
        ObstaclePath((1.0, -0.5), (3.0, -0.5), 0.35, 0.25, -1),
    )
    static = ObstacleSpec(
        2,
        "static",
        ObstacleGeometry("box", (0.30, 0.20, 0.50)),
        (-1.0, 0.75, 0.50),
        (0.0, 0.0, 0.0, 1.0),
    )
    return SceneDocument(
        SCENE_SCHEMA_VERSION,
        "df_back",
        TerrainDocument("golf_heightfield", 0.0, 23, "high"),
        (static, moving),
        SensorDocument.default(),
    )


def run_scene_roundtrip_check() -> VerificationCheck:
    """核对代表性场景逻辑往返、稳定字节和临时句柄隔离。"""
    name = "scene_roundtrip"
    try:
        with tempfile.TemporaryDirectory(prefix="stage3-scene-") as temporary:
            directory = Path(temporary)
            document = _representative_scene_document()
            first = dump_scene_atomic(document, directory / "first.yaml")
            second = dump_scene_atomic(document, directory / "second.yaml")
            third = dump_scene_atomic(load_scene(first), directory / "third.yaml")
            payload = first.read_bytes()
            forbidden = (
                b"body_id",
                b"client_id",
                b"ecal_handle",
                b"qt_object",
            )
            passed = (
                load_scene(first) == document
                and payload == second.read_bytes() == third.read_bytes()
                and b"\r\n" not in payload
                and all(token not in payload for token in forbidden)
            )
            return VerificationCheck(
                name,
                passed,
                f"obstacles={len(document.obstacles)} bytes={len(payload)} stable={payload == second.read_bytes()}",
            )
    except Exception as exc:
        return _failure(name, exc)


def run_interface_log_roundtrip_check() -> VerificationCheck:
    """提交混合消息/事件，关闭后验证二进制顺序、零丢弃和零 pending。"""
    name = "interface_log_roundtrip"
    logger: InterfaceEventLogger | None = None
    try:
        with tempfile.TemporaryDirectory(prefix="stage3-log-") as temporary:
            codec = ProtoCodec()
            messages = (
                WheelState(10, (1.0, 2.0), ()),
                ImuAttitude(20, 0.1, -0.2),
            )
            records = tuple(
                InterfaceLogRecord(
                    sequence=index,
                    topic=("/sim/wheel/state" if index == 1 else "/sim/imu/attitude"),
                    direction="publish",
                    sim_time_ns=message.timestamp_ns,
                    wall_time_ns=100 + index,
                    type_name=codec.type_name(message),
                    payload=codec.encode(message),
                )
                for index, message in enumerate(messages, start=1)
            )
            logger = InterfaceEventLogger(Path(temporary), prefix="roundtrip", queue_size=8)
            accepted = all(logger.record_message(record) for record in records)
            event_accepted = logger.record_event(
                "invalid_command",
                topic="/sim/wheel/command",
                reason="stage3 verifier sample",
            )
            paths = logger.close()
            snapshot = logger.snapshot()
            loaded = read_interface_log(paths.binary_path)
            events = tuple(
                json.loads(line)
                for line in paths.event_path.read_text(encoding="utf-8").splitlines()
            )
            passed = (
                accepted
                and event_accepted
                and loaded == records
                and len(events) == 1
                and events[0]["event"] == "invalid_command"
                and snapshot.pending_count == 0
                and snapshot.dropped_messages == snapshot.dropped_events == 0
                and snapshot.closed
                and not snapshot.writer_failed
            )
            return VerificationCheck(
                name,
                passed,
                f"records={len(loaded)} events={len(events)} pending={snapshot.pending_count} dropped={snapshot.dropped_messages + snapshot.dropped_events}",
            )
    except Exception as exc:
        return _failure(name, exc)
    finally:
        if logger is not None:
            logger.close()


def _chart_series_are_finite(
    buffer: InterfaceChartBuffer,
    robot_model: str,
    expected_length: int,
) -> bool:
    for spec in interface_chart_specs(get_robot_model(robot_model)):
        series = buffer.series(spec.tab_label)
        if len(series["t"]) != expected_length:
            return False
        for line in spec.lines:
            values = series.get(line.key)
            if values is None or len(values) != expected_length:
                return False
            if not all(math.isfinite(value) for value in values):
                return False
        if expected_length > 1 and any(
            right <= left for left, right in zip(series["t"], series["t"][1:])
        ):
            return False
    return True


def run_dashboard_snapshot_and_chart_check() -> VerificationCheck:
    """贯通真实 runtime 组合快照与十个企业图表页，并验证重复去重。"""
    name = "dashboard_snapshot_and_chart"
    try:
        with tempfile.TemporaryDirectory(prefix="stage3-dashboard-") as temporary:
            with _runtime_world(
                Path(temporary),
                robot_model="active_steering_4wd",
                capture_lidar_top_view=True,
                with_logger=False,
            ) as bundle:
                _run_local_frames(bundle, 30)
                first = bundle.runtime.dashboard_snapshot()
                buffer = InterfaceChartBuffer(
                    bundle.config.dashboard_plot_window_sec,
                    bundle.runtime.config,
                )
                first_changed = buffer.append(first)
                duplicate_changed = buffer.append(first)
                payloads_present = all(value is not None for value in _dashboard_payloads(first))
                paired_lidar = (
                    first.lidar_front.timebase_ns == first.lidar_front_view.timestamp_ns
                    and first.lidar_rear.timebase_ns == first.lidar_rear_view.timestamp_ns
                    and len(first.lidar_front.points) == len(first.lidar_front_view.points)
                    and len(first.lidar_rear.points) == len(first.lidar_rear_view.points)
                )
                status_ok = (
                    len(first.status.topics) == 6
                    and all(topic.message_count > 0 for topic in first.status.topics.values())
                    and all(
                        topic.error_count == topic.dropped_count == 0
                        for topic in first.status.topics.values()
                    )
                )
                first_ok = (
                    payloads_present
                    and paired_lidar
                    and status_ok
                    and first_changed == set(INTERFACE_LINE_PLOT_TABS)
                    and duplicate_changed == set()
                    and _chart_series_are_finite(buffer, first.robot_model, 1)
                )
                _run_local_frames(bundle, 30)
                second = bundle.runtime.dashboard_snapshot()
                second_changed = buffer.append(second)
                second_ok = (
                    second.generation == first.generation
                    and second.sim_time_ns > first.sim_time_ns
                    and second_changed == set(INTERFACE_LINE_PLOT_TABS)
                    and _chart_series_are_finite(buffer, second.robot_model, 2)
                )
                return VerificationCheck(
                    name,
                    first_ok and second_ok,
                    f"generation={second.generation} tabs={len(second_changed)} topics={len(second.status.topics)} lidar_points={second.lidar_front.point_num + second.lidar_rear.point_num}",
                )
    except Exception as exc:
        return _failure(name, exc)


class _PollingPeerTransport(_RecordingTransport):
    """要求 poll 先于首次 snapshot，用于锁定 reconnect 状态读取顺序。"""

    def __init__(self, config: InterfaceConfig) -> None:
        super().__init__(mode="ecal")
        self.polled = False
        self.call_order: list[str] = []
        self.quality = tuple(
            TransportTopicQuality(
                channel.topic,
                error_count=1 if channel is config.imu else 0,
                dropped_count=1 if channel is config.lidar_rear else 0,
                state=(
                    "error"
                    if channel is config.imu
                    else "degraded"
                    if channel is config.lidar_rear
                    else "active"
                ),
                detail=(
                    "injected publish error"
                    if channel is config.imu
                    else "injected queue replacement"
                    if channel is config.lidar_rear
                    else ""
                ),
                revision=1,
                last_error_detail=("injected publish error" if channel is config.imu else None),
                last_drop_detail=("injected queue replacement" if channel is config.lidar_rear else None),
                peer_connected=channel is not config.lidar_front,
            )
            for channel in config.channels
        )

    def poll_peer_state(self) -> str:
        self.call_order.append("poll")
        self.polled = True
        return "active"

    def snapshot(self) -> TransportSnapshot:
        self.call_order.append("snapshot")
        if not self.polled:
            raise RuntimeError("snapshot read before poll_peer_state")
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=True,
            published_count=len(self.published),
            received_count=0,
            error_count=1,
            dropped_count=1,
            topic_quality=self.quality,
        )


class _LinkOnlySensorBackend:
    """逐话题状态门禁只需在构造期证明五个语义 parent link 可用。"""

    def link_names(self) -> tuple[str, ...]:
        return ("base_link", "lidar_front_mount", "lidar_rear_mount")

    def close(self) -> None:
        pass


def run_per_topic_ecal_status_check() -> VerificationCheck:
    """先 poll 后读取逐话题 eCAL 质量，并验证各话题故障互不污染。"""
    name = "per_topic_ecal_status"
    config = InterfaceConfig.default(transport_mode="ecal")
    transport = _PollingPeerTransport(config)
    document = SceneDocument.from_runtime(
        "df_back",
        TerrainSelection("flat"),
        (),
        SensorMounts.default(),
        lidar_config=LidarConfig.default(),
    )
    runtime = InterfaceRuntime(
        _VerifierRobot(),
        config=config,
        transport=transport,
        sensor_backend=_LinkOnlySensorBackend(),
        scene_document=document,
    )
    try:
        runtime.initialize_peer_lifecycle("ecal", "active", ecal_connected=True)
        transport.call_order.clear()
        runtime.poll_transport()
        poll_order = tuple(transport.call_order[:2])
        status = runtime.status_snapshot()
        front = status.topics[config.lidar_front.topic]
        rear = status.topics[config.lidar_rear.topic]
        imu = status.topics[config.imu.topic]
        healthy = status.topics[config.rtk.topic]
        passed = (
            poll_order == ("poll", "snapshot")
            and front.state == "waiting_peer"
            and front.error_count == front.dropped_count == 0
            and rear.state == "degraded"
            and rear.dropped_count == 1
            and imu.state == "error"
            and imu.error_count == 1
            and healthy.state == "active"
            and healthy.error_count == healthy.dropped_count == 0
        )
        return VerificationCheck(
            name,
            passed,
            f"order={','.join(poll_order)} front={front.state} rear={rear.state}/drop{rear.dropped_count} imu={imu.state}/error{imu.error_count}",
        )
    except Exception as exc:
        return _failure(name, exc)
    finally:
        runtime.close()


def _evaluate_performance_log(
    start: InterfaceLogSnapshot,
    end: InterfaceLogSnapshot,
    samples: Sequence[tuple[float, int, int]],
    *,
    nominal_message_count: int,
) -> _PerformanceLogQuality:
    """按窗口增量执行接受量、积压、丢帧和 writer 终态门禁。"""
    if (
        isinstance(nominal_message_count, bool)
        or not isinstance(nominal_message_count, int)
        or nominal_message_count <= 0
    ):
        raise ValueError("nominal_message_count must be a positive integer")
    deltas = {
        "accepted": end.accepted_messages - start.accepted_messages,
        "dropped_messages": end.dropped_messages - start.dropped_messages,
        "dropped_events": end.dropped_events - start.dropped_events,
    }
    if any(value < 0 for value in deltas.values()):
        raise RuntimeError("interface logger counters moved backwards")

    minimum_accepted = math.ceil(nominal_message_count * 0.90)
    sustained_backlog = _has_sustained_backlog(samples)
    reasons: list[str] = []
    if start.pending_count != 0:
        reasons.append("baseline_pending")
    if deltas["accepted"] < minimum_accepted:
        reasons.append("accepted")
    if end.pending_count != 0:
        reasons.append("pending")
    if deltas["dropped_messages"] != 0 or deltas["dropped_events"] != 0:
        reasons.append("dropped")
    if end.writer_failed:
        reasons.append("writer")
    if sustained_backlog:
        reasons.append("backlog")
    return _PerformanceLogQuality(
        passed=not reasons,
        accepted_messages=deltas["accepted"],
        minimum_accepted_messages=minimum_accepted,
        final_pending=end.pending_count,
        max_pending=max((depth for _at, depth, _completed in samples), default=0),
        failure_reasons=tuple(reasons),
    )


def _wait_for_performance_logger_idle(
    logger: InterfaceEventLogger,
) -> InterfaceLogSnapshot:
    """测量停止后有界等待已接受记录落盘，并保留失败快照用于报告。"""
    deadline = time.monotonic() + PERFORMANCE_LOG_IDLE_TIMEOUT_SEC
    while True:
        snapshot = logger.snapshot()
        if snapshot.pending_count == 0 or snapshot.writer_failed:
            return snapshot
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return snapshot
        time.sleep(min(0.005, remaining))


def run_twenty_obstacle_queue_performance_check(
    *,
    max_dashboard_gap_sec: float,
) -> VerificationCheck:
    """预热一秒后实跑五秒，采样异步日志队列、传输质量和 Dashboard 间隔。"""
    name = "twenty_obstacle_queue_performance"
    if (
        isinstance(max_dashboard_gap_sec, bool)
        or not isinstance(max_dashboard_gap_sec, (int, float))
        or not math.isfinite(float(max_dashboard_gap_sec))
        or float(max_dashboard_gap_sec) <= 0.0
    ):
        raise ValueError("max_dashboard_gap_sec must be a positive finite number")
    limit = float(max_dashboard_gap_sec)
    try:
        with tempfile.TemporaryDirectory(prefix="stage3-performance-") as temporary:
            with _runtime_world(
                Path(temporary),
                capture_lidar_top_view=True,
                with_logger=True,
            ) as bundle:
                add = bundle.coordinator.apply_action(
                    AddObstaclesAction(
                        ObstacleGenerationRequest("mixed", 20, seed=7301)
                    )
                )
                if (
                    add.obstacle_result is None
                    or not add.obstacle_result.succeeded
                    or len(bundle.manager.snapshot()) != 20
                ):
                    raise AssertionError("failed to install exactly twenty obstacles")
                chart = InterfaceChartBuffer(
                    bundle.config.dashboard_plot_window_sec,
                    bundle.runtime.config,
                )

                def run_paced(
                    duration_sec: float,
                    *,
                    collect: bool,
                    log_baseline: InterfaceLogSnapshot | None = None,
                ):
                    started_at = time.perf_counter()
                    pacer = DeadlinePacer(
                        TIME_STEP,
                        monotonic=time.perf_counter,
                        sleep=time.sleep,
                    )
                    pacer.start()
                    ended_at = started_at + duration_sec
                    next_sample = started_at
                    last_dashboard = started_at
                    gaps: list[float] = []
                    queue_samples: list[tuple[float, int, int]] = []

                    def capture_log_sample(sampled_at: float) -> None:
                        """从同一个 logger 快照原子派生 pending 与 completed。"""
                        assert bundle.logger is not None
                        log_sample = bundle.logger.snapshot()
                        accepted = (
                            log_sample.accepted_messages
                            + log_sample.accepted_events
                        )
                        queue_samples.append(
                            (
                                sampled_at - started_at,
                                log_sample.pending_count,
                                accepted - log_sample.pending_count,
                            )
                        )

                    if collect:
                        if log_baseline is None:
                            raise RuntimeError("performance log baseline is required")
                        baseline_accepted = (
                            log_baseline.accepted_messages
                            + log_baseline.accepted_events
                        )
                        queue_samples.append(
                            (
                                0.0,
                                log_baseline.pending_count,
                                baseline_accepted - log_baseline.pending_count,
                            )
                        )

                    while time.perf_counter() < ended_at:
                        _run_local_frames(bundle, 1)
                        snapshot = bundle.runtime.dashboard_snapshot()
                        chart.append(snapshot)
                        event_at = time.perf_counter()
                        if collect:
                            gaps.append(event_at - last_dashboard)
                        last_dashboard = event_at
                        if collect and event_at >= next_sample:
                            capture_log_sample(event_at)
                            next_sample += PERFORMANCE_LOG_SAMPLE_PERIOD_SEC
                            if next_sample <= event_at:
                                next_sample = (
                                    event_at + PERFORMANCE_LOG_SAMPLE_PERIOD_SEC
                                )
                        pacer.wait_for_next_deadline()
                    if collect:
                        # 排空前保留窗口终点，避免漏掉最后恰满一秒的停滞。
                        capture_log_sample(time.perf_counter())
                    return gaps, queue_samples

                run_paced(PERFORMANCE_WARMUP_SEC, collect=False)
                assert bundle.logger is not None
                log_start = _wait_for_performance_logger_idle(bundle.logger)
                gaps, queue_samples = run_paced(
                    PERFORMANCE_MEASUREMENT_SEC,
                    collect=True,
                    log_baseline=log_start,
                )
                assert bundle.logger is not None
                log_snapshot = _wait_for_performance_logger_idle(bundle.logger)
                nominal_messages = round(
                    sum(channel.rate_hz for channel in bundle.runtime.config.channels)
                    * PERFORMANCE_MEASUREMENT_SEC
                )
                log_quality = _evaluate_performance_log(
                    log_start,
                    log_snapshot,
                    queue_samples,
                    nominal_message_count=nominal_messages,
                )
                transport_snapshot = bundle.transport.snapshot()
                status = bundle.runtime.status_snapshot()
                max_gap = max(gaps, default=math.inf)
                passed = (
                    len(bundle.manager.snapshot()) == 20
                    and len(queue_samples) >= 45
                    and log_quality.passed
                    and transport_snapshot.error_count == 0
                    and transport_snapshot.dropped_count == 0
                    and all(
                        topic.error_count == topic.dropped_count == 0
                        for topic in status.topics.values()
                    )
                    and max_gap <= limit
                )
                return VerificationCheck(
                    name,
                    passed,
                    f"obstacles=20 samples={len(queue_samples)} "
                    f"accepted={log_quality.accepted_messages}/{nominal_messages} "
                    f"minimum={log_quality.minimum_accepted_messages} "
                    f"max_log_pending={log_quality.max_pending} "
                    f"final_pending={log_quality.final_pending} "
                    f"backlog={'backlog' in log_quality.failure_reasons} "
                    f"max_dashboard_gap_ms={max_gap * 1000.0:.2f} "
                    f"transport_dropped={transport_snapshot.dropped_count} "
                    f"log_dropped={log_snapshot.dropped_messages + log_snapshot.dropped_events}",
                )
    except Exception as exc:
        return _failure(name, exc)


def run_stage3_checks() -> tuple[VerificationCheck, ...]:
    """按阶段三计划固定顺序执行 21 项门禁，并拒绝名称碰撞。"""
    checks = (
        run_proto_and_topic_contract_check(),
        *run_four_model_wheel_checks(),
        run_timeout_and_steering_hold_check(),
        run_100_10_hz_scheduler_check(),
        *run_three_terrain_lidar_checks(),
        run_static_and_moving_obstacle_lidar_check(),
        run_lidar_collision_contact_check(),
        *run_three_terrain_truth_sensor_checks(tolerance=1e-4),
        run_pause_rebuild_and_edge_switch_check(),
        run_scene_roundtrip_check(),
        run_interface_log_roundtrip_check(),
        run_dashboard_snapshot_and_chart_check(),
        run_per_topic_ecal_status_check(),
        run_twenty_obstacle_queue_performance_check(
            max_dashboard_gap_sec=0.100,
        ),
    )
    names = tuple(check.name for check in checks)
    duplicate = next(
        (name for index, name in enumerate(names) if name in names[:index]),
        None,
    )
    if duplicate is not None:
        raise ValueError(f"duplicate verification check name: {duplicate}")
    return checks


def main() -> int:
    """运行完整阶段三 DIRECT 验收并打印可供 CI 解析的稳定报告。"""
    checks = run_stage3_checks()
    report = summarize(checks)
    for line in (*report.lines, report.final_line):
        print(line)
    return exit_code(checks)


if __name__ == "__main__":
    raise SystemExit(main())
