# 轮子接口 DIRECT 集成测试：在四种真实 PyBullet 车型上验证闭环反馈和超时停车。
from __future__ import annotations

from dataclasses import FrozenInstanceError
from fractions import Fraction
import math

import pybullet as p
import pytest

import slope_sim.robot as robot_module
from scripts.verify_stage1_matrix import validate_robot_pose
from slope_sim.interfaces.clock import PeriodicScheduler, SimulationClock
from slope_sim.interfaces.models import WheelCommand
from slope_sim.model_registry import get_robot_model
from slope_sim.robot import ActiveSteeringRobot, create_robot
from slope_sim.scene import create_slope_scene


TIME_STEP = 1.0 / 240.0
SEND_FRAMES = 192
STOP_SETTLE_FRAMES = 120
ACTIVE_TIMEOUT_WARMUP_FRAMES = 48
STOP_SPEED_ABS_TOL = 0.01


class FakeMonotonic:
    """用 Fraction 累积，避免 240 Hz 循环的墙钟漂移。"""

    def __init__(self) -> None:
        self._seconds = Fraction(0)

    def __call__(self) -> float:
        return float(self._seconds)

    def advance(self, dt: float | Fraction) -> float:
        self._seconds += Fraction(dt).limit_denominator(1_000_000_000)
        return float(self._seconds)


def _runtime_type():
    from slope_sim.interfaces.runtime import InterfaceRuntime

    return InterfaceRuntime


def _create_settled_robot(client_id: int, model_name: str):
    """创建高摩擦平地车型，并在零轮速下静置到稳定接触。"""
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
    runtime,
    wall_clock: FakeMonotonic,
    send_clock: SimulationClock,
    scheduler: PeriodicScheduler,
    drive: tuple[float, ...],
    steering: tuple[float, ...],
) -> tuple[int, float | None]:
    """每个物理帧按独立 100 Hz 调度器提交全部到期命令。"""
    now_ns = send_clock.advance(TIME_STEP)
    send_count = 0
    last_send_at = None
    for timestamp_ns in scheduler.pop_due(now_ns):
        last_send_at = wall_clock()
        assert runtime.accept_local_command(
            WheelCommand(timestamp_ns, drive, steering),
            received_at=last_send_at,
        )
        send_count += 1
    return send_count, last_send_at


def _continue_timed_out_stop(
    client_id: int,
    runtime,
    robot,
    scene,
    wall_clock: FakeMonotonic,
):
    """首次超时后继续物理步进，并确认安全决定不会重新激活。"""
    for _ in range(STOP_SETTLE_FRAMES):
        wall_clock.advance(TIME_STEP)
        decision = runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
        assert decision.timed_out
        p.stepSimulation(physicsClientId=client_id)
        runtime.after_physics_step(TIME_STEP)
        validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
    stopped_state = robot.read_interface_wheel_state(runtime.clock.now_ns)
    assert stopped_state.drive_wheel_speed_rad_s == pytest.approx(
        (0.0,) * len(stopped_state.drive_wheel_speed_rad_s),
        abs=STOP_SPEED_ABS_TOL,
    )
    return stopped_state


def test_registered_robot_interface_port_returns_frozen_joint_feedback_not_target_echo():
    client_id = p.connect(p.DIRECT)
    runtime = None
    try:
        _scene, robot = _create_settled_robot(client_id, "df_back")
        robot.command_wheel_speeds((2.7, -1.8), (), dt=TIME_STEP)
        for _ in range(60):
            p.stepSimulation(physicsClientId=client_id)

        state = robot.read_interface_wheel_state(10_000_000)
        runtime = _runtime_type().local_for_robot(robot, monotonic=FakeMonotonic())
        assert state.timestamp_ns == 10_000_000
        assert state.drive_wheel_speed_rad_s == pytest.approx((2.7, -1.8), abs=0.05)
        assert state.steering_wheel_angle_rad == ()
        with pytest.raises(FrozenInstanceError):
            state.drive_wheel_speed_rad_s = ()
    finally:
        try:
            if runtime is not None:
                runtime.close()
        finally:
            p.disconnect(client_id)


def test_active_safe_stop_attempts_all_drive_zeros_before_steering_read_failure(monkeypatch):
    client_id = p.connect(p.DIRECT)
    runtime = None
    try:
        _scene, robot = _create_settled_robot(client_id, "active_steering_4wd")
        original_set = robot_module.p.setJointMotorControl2
        original_get = robot_module.p.getJointState
        drive_attempts: list[int] = []

        def record_set(body_id, joint_index, controlMode, *args, **kwargs):
            if (
                body_id == robot.robot_id
                and controlMode == p.VELOCITY_CONTROL
                and joint_index in robot.drive_wheel_joint_indices
                and kwargs.get("targetVelocity") == 0.0
            ):
                drive_attempts.append(joint_index)
            return original_set(body_id, joint_index, controlMode, *args, **kwargs)

        def fail_steering_read(body_id, joint_index, *args, **kwargs):
            if body_id == robot.robot_id and joint_index == robot.steering_joint_indices[0]:
                raise RuntimeError("injected steering read failure")
            return original_get(body_id, joint_index, *args, **kwargs)

        with monkeypatch.context() as patch:
            patch.setattr(robot_module.p, "setJointMotorControl2", record_set)
            patch.setattr(robot_module.p, "getJointState", fail_steering_read)
            with pytest.raises(RuntimeError, match="steering read failure"):
                robot.hold_current_steering_and_stop_drive(TIME_STEP)

        assert drive_attempts == list(robot.drive_wheel_joint_indices)
        assert robot.left_wheel_speed == robot.right_wheel_speed == 0.0
        runtime = _runtime_type().local_for_robot(robot, monotonic=FakeMonotonic())
    finally:
        try:
            if runtime is not None:
                runtime.close()
        finally:
            p.disconnect(client_id)


def test_active_safe_stop_attempts_all_drives_and_steering_after_first_steering_failure(monkeypatch):
    client_id = p.connect(p.DIRECT)
    runtime = None
    try:
        _scene, robot = _create_settled_robot(client_id, "active_steering_4wd")
        original_set = robot_module.p.setJointMotorControl2
        drive_attempts: list[int] = []
        steering_attempts: list[int] = []
        first_steering_failed = False

        def fail_first_steering(body_id, joint_index, controlMode, *args, **kwargs):
            nonlocal first_steering_failed
            if body_id == robot.robot_id and controlMode == p.VELOCITY_CONTROL:
                if joint_index in robot.drive_wheel_joint_indices and kwargs.get("targetVelocity") == 0.0:
                    drive_attempts.append(joint_index)
            if body_id == robot.robot_id and controlMode == p.POSITION_CONTROL:
                if joint_index in robot.steering_joint_indices:
                    steering_attempts.append(joint_index)
                    if not first_steering_failed:
                        first_steering_failed = True
                        raise RuntimeError("injected first steering failure")
            return original_set(body_id, joint_index, controlMode, *args, **kwargs)

        with monkeypatch.context() as patch:
            patch.setattr(robot_module.p, "setJointMotorControl2", fail_first_steering)
            with pytest.raises(RuntimeError, match="first steering failure"):
                robot.hold_current_steering_and_stop_drive(TIME_STEP)

        assert drive_attempts == list(robot.drive_wheel_joint_indices)
        assert steering_attempts == list(robot.steering_joint_indices)
        assert robot.left_wheel_speed == robot.right_wheel_speed == 0.0
        runtime = _runtime_type().local_for_robot(robot, monotonic=FakeMonotonic())
    finally:
        try:
            if runtime is not None:
                runtime.close()
        finally:
            p.disconnect(client_id)


@pytest.mark.parametrize("model_name", ("df_back", "active_steering_4wd"))
def test_safe_stop_attempts_every_registered_drive_when_first_drive_write_fails(
    model_name: str,
    monkeypatch,
):
    client_id = p.connect(p.DIRECT)
    runtime = None
    try:
        _scene, robot = _create_settled_robot(client_id, model_name)
        drive_count = len(robot.model_spec.drive_joint_names)
        steering_count = len(robot.model_spec.steering_joint_names)
        robot.command_wheel_speeds(
            tuple(float(index + 1) for index in range(drive_count)),
            (0.5,) * steering_count,
            dt=TIME_STEP,
        )
        original_set = robot_module.p.setJointMotorControl2
        drive_attempts: list[int] = []
        steering_attempts: list[int] = []
        first_drive_failed = False

        def fail_first_drive(body_id, joint_index, controlMode, *args, **kwargs):
            nonlocal first_drive_failed
            if (
                body_id == robot.robot_id
                and controlMode == p.VELOCITY_CONTROL
                and joint_index in robot.drive_wheel_joint_indices
                and kwargs.get("targetVelocity") == 0.0
            ):
                drive_attempts.append(joint_index)
                if not first_drive_failed:
                    first_drive_failed = True
                    raise RuntimeError("injected first drive failure")
            if (
                body_id == robot.robot_id
                and controlMode == p.POSITION_CONTROL
                and joint_index in getattr(robot, "steering_joint_indices", ())
            ):
                steering_attempts.append(joint_index)
            return original_set(body_id, joint_index, controlMode, *args, **kwargs)

        with monkeypatch.context() as patch:
            patch.setattr(robot_module.p, "setJointMotorControl2", fail_first_drive)
            with pytest.raises(RuntimeError, match="first drive failure"):
                robot.hold_current_steering_and_stop_drive(TIME_STEP)

        assert drive_attempts == list(robot.drive_wheel_joint_indices)
        if model_name == "active_steering_4wd":
            assert steering_attempts == list(robot.steering_joint_indices)
        else:
            assert steering_attempts == []
        assert robot.left_wheel_speed == robot.right_wheel_speed == 0.0
        runtime = _runtime_type().local_for_robot(robot, monotonic=FakeMonotonic())
    finally:
        try:
            if runtime is not None:
                runtime.close()
        finally:
            p.disconnect(client_id)


@pytest.mark.parametrize("model_name", ("df_front", "df_mid", "df_back"))
def test_differential_runtime_drives_and_times_out_with_actual_two_wheel_feedback(model_name: str):
    client_id = p.connect(p.DIRECT)
    runtime = None
    try:
        scene, robot = _create_settled_robot(client_id, model_name)
        wall_clock = FakeMonotonic()
        runtime = _runtime_type().local_for_robot(robot, monotonic=wall_clock)
        send_clock = SimulationClock()
        sender = PeriodicScheduler(100)
        initial_position = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]
        initial_x = float(initial_position[0])
        command_count = 0
        last_send_at = None

        for _ in range(SEND_FRAMES):
            wall_clock.advance(TIME_STEP)
            sent, frame_last_send = _send_due_command(
                runtime,
                wall_clock,
                send_clock,
                sender,
                (6.0, 6.0),
                (),
            )
            command_count += sent
            if frame_last_send is not None:
                last_send_at = frame_last_send
            decision = runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
            assert not decision.timed_out
            p.stepSimulation(physicsClientId=client_id)
            runtime.after_physics_step(TIME_STEP)
            validate_robot_pose(client_id, robot, scene, require_ground_contact=True)

        last_state_while_sending = runtime.last_wheel_state
        final_position = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]
        final_x = float(final_position[0])
        forward_displacement = final_x - initial_x
        displacement = math.hypot(
            final_x - initial_x,
            float(final_position[1]) - float(initial_position[1]),
        )
        assert command_count == 80
        assert runtime.status_snapshot(wall_time=wall_clock()).command.valid_count == 80
        assert forward_displacement > 0.10
        assert displacement > 0.10
        assert last_state_while_sending is not None
        assert last_state_while_sending.drive_wheel_speed_rad_s == pytest.approx((6.0, 6.0), abs=0.25)
        assert last_state_while_sending.steering_wheel_angle_rad == ()

        assert last_send_at is not None
        while True:
            wall_clock.advance(TIME_STEP)
            decision = runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
            p.stepSimulation(physicsClientId=client_id)
            runtime.after_physics_step(TIME_STEP)
            validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
            if decision.timed_out:
                timeout_age = wall_clock() - last_send_at
                break

        assert 0.100 <= timeout_age <= 0.100 + TIME_STEP
        assert decision.drive_wheel_speed_rad_s == (0.0, 0.0)
        assert runtime.last_wheel_state is not last_state_while_sending
        assert last_state_while_sending.drive_wheel_speed_rad_s == pytest.approx((6.0, 6.0), abs=0.25)
        stopped_state = _continue_timed_out_stop(
            client_id,
            runtime,
            robot,
            scene,
            wall_clock,
        )
        assert all(math.isfinite(speed) for speed in stopped_state.drive_wheel_speed_rad_s)
        print(
            f"DIRECT_METRIC {model_name} forward_displacement_m={forward_displacement:.6f} "
            f"displacement_m={displacement:.6f} "
            f"feedback={last_state_while_sending.drive_wheel_speed_rad_s} "
            f"stop_feedback={stopped_state.drive_wheel_speed_rad_s} "
            f"commands={command_count} timeout_age_s={timeout_age:.9f}"
        )
    finally:
        try:
            if runtime is not None:
                runtime.close()
        finally:
            p.disconnect(client_id)


def test_active_runtime_preserves_registered_drive_order_and_reports_two_actual_angles():
    client_id = p.connect(p.DIRECT)
    runtime = None
    try:
        scene, robot = _create_settled_robot(client_id, "active_steering_4wd")
        wall_clock = FakeMonotonic()
        runtime = _runtime_type().local_for_robot(robot, monotonic=wall_clock)
        send_clock = SimulationClock()
        sender = PeriodicScheduler(100)
        command_count = 0
        initial_position = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]
        initial_x = float(initial_position[0])

        for _ in range(SEND_FRAMES):
            wall_clock.advance(TIME_STEP)
            sent, _ = _send_due_command(
                runtime,
                wall_clock,
                send_clock,
                sender,
                (3.0, 4.0, 5.0, 6.0),
                (0.5, -0.5),
            )
            command_count += sent
            decision = runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
            assert not decision.timed_out
            p.stepSimulation(physicsClientId=client_id)
            runtime.after_physics_step(TIME_STEP)
            validate_robot_pose(client_id, robot, scene, require_ground_contact=True)

        state = runtime.last_wheel_state
        final_position = p.getBasePositionAndOrientation(
            robot.robot_id,
            physicsClientId=client_id,
        )[0]
        final_x = float(final_position[0])
        forward_displacement = final_x - initial_x
        displacement = math.hypot(
            final_x - initial_x,
            float(final_position[1]) - float(initial_position[1]),
        )
        assert command_count == 80
        assert forward_displacement > 0.10
        assert math.isfinite(displacement)
        assert state is not None
        assert state.drive_wheel_speed_rad_s == pytest.approx((3.0, 4.0, 5.0, 6.0), abs=0.30)
        assert len(state.steering_wheel_angle_rad) == 2
        assert state.steering_wheel_angle_rad == pytest.approx(tuple(robot._steering_targets), abs=0.03)
        assert state.steering_wheel_angle_rad[0] > 0.30
        assert state.steering_wheel_angle_rad[1] < -0.30
        with pytest.raises(FrozenInstanceError):
            state.timestamp_ns = 0
        print(
            "DIRECT_METRIC active_steering_4wd "
            f"forward_displacement_m={forward_displacement:.6f} "
            f"displacement_m={displacement:.6f} feedback={state.drive_wheel_speed_rad_s} "
            f"angles={state.steering_wheel_angle_rad} commands={command_count}"
        )
    finally:
        try:
            if runtime is not None:
                runtime.close()
        finally:
            p.disconnect(client_id)


def test_active_timeout_retargets_real_angles_before_stopping_all_drive_wheels():
    client_id = p.connect(p.DIRECT)
    runtime = None
    try:
        scene, robot = _create_settled_robot(client_id, "active_steering_4wd")
        assert isinstance(robot, ActiveSteeringRobot)
        wall_clock = FakeMonotonic()
        runtime = _runtime_type().local_for_robot(robot, monotonic=wall_clock)
        send_clock = SimulationClock()
        sender = PeriodicScheduler(100)
        last_send_at = None

        for _ in range(ACTIVE_TIMEOUT_WARMUP_FRAMES):
            wall_clock.advance(TIME_STEP)
            _sent, frame_last_send = _send_due_command(
                runtime,
                wall_clock,
                send_clock,
                sender,
                (2.0, 3.0, 4.0, 5.0),
                (1.0, -1.0),
            )
            if frame_last_send is not None:
                last_send_at = frame_last_send
            active = runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
            assert not active.timed_out
            p.stepSimulation(physicsClientId=client_id)
            runtime.after_physics_step(TIME_STEP)
            validate_robot_pose(client_id, robot, scene, require_ground_contact=True)

        assert last_send_at is not None
        for _ in range(23):
            wall_clock.advance(TIME_STEP)
            assert wall_clock() - last_send_at < 0.100
            active = runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
            assert not active.timed_out
            p.stepSimulation(physicsClientId=client_id)
            runtime.after_physics_step(TIME_STEP)
            validate_robot_pose(client_id, robot, scene, require_ground_contact=True)

        active = runtime.before_physics_step(0.5, wall_time=wall_clock())
        assert not active.timed_out
        while True:
            wall_clock.advance(TIME_STEP)
            candidate_target = tuple(robot._steering_targets)
            candidate_angle = robot.read_steering_wheel_angles()
            candidate_drive = robot.read_interface_wheel_state(
                runtime.clock.now_ns
            ).drive_wheel_speed_rad_s
            timed_out = runtime.before_physics_step(TIME_STEP, wall_time=wall_clock())
            if timed_out.timed_out:
                target_before_timeout = candidate_target
                angle_before_timeout = candidate_angle
                drive_before_timeout = candidate_drive
                timeout_age = wall_clock() - last_send_at
                break
            p.stepSimulation(physicsClientId=client_id)
            runtime.after_physics_step(TIME_STEP)
            validate_robot_pose(client_id, robot, scene, require_ground_contact=True)

        assert 0.100 <= timeout_age <= 0.100 + TIME_STEP
        assert all(abs(speed) > 1.0 for speed in drive_before_timeout)
        assert all(
            abs(target - actual) > 0.1
            for target, actual in zip(target_before_timeout, angle_before_timeout)
        )
        assert timed_out.drive_wheel_speed_rad_s == (0.0, 0.0, 0.0, 0.0)
        assert tuple(robot._steering_targets) == pytest.approx(angle_before_timeout, abs=1e-12)
        p.stepSimulation(physicsClientId=client_id)
        runtime.after_physics_step(TIME_STEP)
        validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
        angle_after_timeout = robot.read_steering_wheel_angles()
        stopped_state = _continue_timed_out_stop(
            client_id,
            runtime,
            robot,
            scene,
            wall_clock,
        )
        assert all(math.isfinite(speed) for speed in stopped_state.drive_wheel_speed_rad_s)

        for actual_before, actual_after, actual_stopped, old_target in zip(
            angle_before_timeout,
            angle_after_timeout,
            stopped_state.steering_wheel_angle_rad,
            target_before_timeout,
        ):
            assert actual_after == pytest.approx(actual_before, abs=0.03)
            assert actual_stopped == pytest.approx(actual_before, abs=0.03)
            assert abs(actual_stopped - old_target) > 0.07
            assert abs(actual_stopped) <= robot.model_spec.max_steering_angle
        print(
            "DIRECT_METRIC active_timeout "
            f"drive_before={drive_before_timeout} stop_feedback={stopped_state.drive_wheel_speed_rad_s} "
            f"target_before={target_before_timeout} angle_before={angle_before_timeout} "
            f"angle_after={angle_after_timeout} angle_stopped={stopped_state.steering_wheel_angle_rad} "
            f"timeout_age_s={timeout_age:.9f}"
        )
    finally:
        try:
            if runtime is not None:
                runtime.close()
        finally:
            p.disconnect(client_id)
