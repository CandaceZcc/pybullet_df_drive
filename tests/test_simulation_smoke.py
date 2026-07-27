# 仿真 smoke 测试：覆盖阶段一 4×3 组合、日志生成和两类控制器物理响应。
from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pandas as pd
import pybullet as p
import pytest

from scripts.verify_stage1_matrix import verify_combination
import scripts.verify_stage1_matrix as stage1_verifier
import slope_sim.simulation as simulation_module
from slope_sim.config import ExperimentConfig
from slope_sim.interfaces.logging import read_interface_log
from slope_sim.interfaces.transport import LocalTransport, TransportSnapshot
from slope_sim.model_registry import get_robot_model, robot_model_names
from slope_sim.robot import DifferentialDriveRobot, create_robot
from slope_sim.scene_config import load_scene
import slope_sim.scene as scene_module
from slope_sim.scene import terrain_model_names
from slope_sim.simulation import _probe_terrain_for_robot, run_experiment


def test_run_experiment_direct_generates_log_and_figure(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="df_back",
            terrain_model="flat",
            slope_deg=0.0,
            duration_sec=0.2,
            time_step=1.0 / 120.0,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )
    assert result.log_path.exists()
    assert result.figure_path.exists()
    assert result.metrics["endpoint_error"] >= 0.0
    frame = pd.read_csv(result.log_path)
    assert len(frame) > 0
    assert set(frame["robot_model"]) == {"df_back"}
    assert set(frame["terrain_type"]) <= {"", "flat"}


def test_run_experiment_local_interface_uses_runtime_hooks_logs_and_scene_export(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """自动 local 控制必须走企业接口闭环，并从协调器导出最终逻辑场景。"""
    def reject_direct_twist(*_args, **_kwargs):
        raise AssertionError("interface-enabled experiment bypassed runtime with command_twist")

    monkeypatch.setattr(DifferentialDriveRobot, "command_twist", reject_direct_twist)
    scene_out = tmp_path / "scene.yaml"

    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="df_back",
            terrain_model="flat",
            duration_sec=0.04,
            time_step=0.01,
            interface_mode="local",
            interface_enabled=True,
            interface_log_enabled=True,
            scene_out=scene_out,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    assert result.interface_binary_log is not None
    assert result.interface_event_log is not None
    records = read_interface_log(result.interface_binary_log)
    assert any(record.topic == "/sim/wheel/command" and record.direction == "receive" for record in records)
    assert any(record.topic == "/sim/wheel/state" and record.direction == "publish" for record in records)
    assert result.scene_export == scene_out
    exported = load_scene(scene_out)
    assert exported.robot_model == "df_back"
    assert exported.terrain.terrain_model == "flat"
    assert exported.obstacles == ()


def test_run_experiment_disabled_keeps_command_twist_and_skips_interface_factory(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """禁用接口时不得构造任何接口资源，并保留原有 twist 控制路径。"""
    command_count = 0
    original_command_twist = DifferentialDriveRobot.command_twist

    def count_command(self, linear, angular, *, dt):
        nonlocal command_count
        command_count += 1
        return original_command_twist(self, linear, angular, dt=dt)

    def reject_interface_factory(*_args, **_kwargs):
        raise AssertionError("disabled experiment constructed interface resources")

    monkeypatch.setattr(DifferentialDriveRobot, "command_twist", count_command)
    monkeypatch.setattr(simulation_module, "create_interface_session", reject_interface_factory)

    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            duration_sec=0.02,
            time_step=0.01,
            interface_enabled=False,
            interface_log_enabled=True,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    assert command_count == 2
    assert result.interface_binary_log is None
    assert result.interface_event_log is None


def test_run_experiment_interface_log_disabled_returns_no_interface_paths(tmp_path: Path) -> None:
    """接口仍运行时，关闭专用日志只影响路径结果，不改变控制闭环。"""
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            duration_sec=0.02,
            time_step=0.01,
            interface_mode="local",
            interface_enabled=True,
            interface_log_enabled=False,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    assert result.interface_binary_log is None
    assert result.interface_event_log is None
    assert result.log_path.exists()


def test_run_experiment_actual_ecal_uses_drift_free_deadline_pacing(
    tmp_path: Path,
    monkeypatch,
) -> None:
    class EcalLoopbackTransport(LocalTransport):
        def __init__(self, peer_state_callback) -> None:
            super().__init__()
            self._peer_state_callback = peer_state_callback

        def poll_peer_state(self) -> str:
            return "waiting_peer"

        def snapshot(self) -> TransportSnapshot:
            base = super().snapshot()
            return TransportSnapshot(
                "ecal",
                True,
                base.published_count,
                base.received_count,
                base.error_count,
                base.dropped_count,
            )

    def create_ecal_transport(_mode, *, config, peer_state_callback):
        assert config.transport_mode == "ecal"
        return EcalLoopbackTransport(peer_state_callback)

    clock = [0.0]
    sleeps: list[float] = []

    def monotonic() -> float:
        return clock[0]

    def sleep(delay: float) -> None:
        assert delay >= 0.0
        sleeps.append(delay)
        clock[0] += delay

    monkeypatch.setattr(simulation_module, "create_transport", create_ecal_transport)

    run_experiment(
        ExperimentConfig(
            mode="direct",
            duration_sec=0.03,
            time_step=0.01,
            interface_mode="ecal",
            interface_enabled=True,
            interface_log_enabled=False,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        ),
        monotonic=monotonic,
        sleep=sleep,
    )

    assert sleeps == pytest.approx([0.01, 0.01, 0.01])
    assert clock[0] == pytest.approx(0.03)


def test_deadline_pacer_skips_sleep_when_frame_has_overrun() -> None:
    clock = [0.0]
    sleeps: list[float] = []
    pacer = simulation_module._DeadlinePacer(
        0.01,
        monotonic=lambda: clock[0],
        sleep=sleeps.append,
    )
    pacer.start()

    clock[0] = 0.025
    pacer.wait_for_next_deadline()
    clock[0] = 0.026
    pacer.wait_for_next_deadline()

    assert sleeps == []


def test_run_experiment_local_mode_does_not_use_wall_clock_pacer(
    tmp_path: Path,
) -> None:
    run_experiment(
        ExperimentConfig(
            mode="direct",
            duration_sec=0.01,
            time_step=0.01,
            interface_mode="local",
            interface_enabled=True,
            interface_log_enabled=False,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        ),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: pytest.fail("local mode must not pace on wall clock"),
    )


def test_scene_in_is_loaded_before_world_scene_and_robot_factories(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """scene_in 必须先于 load_manual_world 以及其下游 scene/robot body 工厂。"""
    import slope_sim.coordinator as coordinator_module
    from slope_sim.scene import SceneInfo
    from slope_sim.scene_config import SceneDocument, SensorDocument, TerrainDocument

    trace: list[str] = []
    document = SceneDocument(
        1,
        "df_mid",
        TerrainDocument("flat", 0.0, 0, "low"),
        (),
        SensorDocument.default(),
    )
    original_connect = p.connect
    original_disconnect = p.disconnect
    original_load_world = coordinator_module.load_manual_world

    def connect(mode):
        trace.append("connect")
        return original_connect(mode)

    def disconnect(client_id):
        trace.append("disconnect")
        return original_disconnect(client_id)

    def load_world(*args, **kwargs):
        trace.append("load_manual_world")
        return original_load_world(*args, **kwargs)

    def create_scene(*_args, **_kwargs):
        trace.append("create_slope_scene")
        return SceneInfo(999, "flat", 0.0, body_ids=(999,))

    def create_robot_then_fail(*_args, **_kwargs):
        trace.append("create_robot")
        raise RuntimeError("stop after ordering boundary")

    monkeypatch.setattr(simulation_module, "load_scene", lambda _path: (trace.append("load_scene"), document)[1])
    monkeypatch.setattr(simulation_module.p, "connect", connect)
    monkeypatch.setattr(simulation_module.p, "disconnect", disconnect)
    monkeypatch.setattr(coordinator_module, "load_manual_world", load_world)
    monkeypatch.setattr(coordinator_module, "create_slope_scene", create_scene)
    monkeypatch.setattr(coordinator_module, "create_robot", create_robot_then_fail)

    with pytest.raises(RuntimeError, match="ordering boundary"):
        run_experiment(
            ExperimentConfig(
                mode="direct",
                interface_enabled=False,
                scene_in=tmp_path / "input.yaml",
                log_dir=tmp_path / "logs",
                figure_dir=tmp_path / "figures",
            )
        )

    assert trace == [
        "load_scene",
        "connect",
        "load_manual_world",
        "create_slope_scene",
        "create_robot",
        "disconnect",
    ]


def test_scene_in_overrides_initial_world_and_scene_out_tracks_moving_obstacle(
    tmp_path: Path,
) -> None:
    """真实入口应按文档建世界，并从 coordinator 导出已推进的无 body-id 逻辑状态。"""
    from slope_sim.obstacles import ObstacleGeometry, ObstaclePath, ObstacleSpec
    from slope_sim.scene_config import (
        SceneDocument,
        SensorDocument,
        TerrainDocument,
        dump_scene_atomic,
    )

    moving = ObstacleSpec(
        logical_id=9,
        mode="moving",
        geometry=ObstacleGeometry("box", (0.2, 0.2, 0.2)),
        position=(1.5, -1.0, 0.2),
        orientation=(0.0, 0.0, 0.0, 1.0),
        path=ObstaclePath(
            start_xy=(1.0, -1.0),
            end_xy=(3.0, -1.0),
            speed=0.5,
            progress=0.25,
            direction=1,
        ),
    )
    source = SceneDocument(
        1,
        "df_mid",
        TerrainDocument("flat", 0.0, 17, "high"),
        (moving,),
        SensorDocument.default(),
    )
    scene_in = dump_scene_atomic(source, tmp_path / "input.yaml")
    scene_out = tmp_path / "output.yaml"

    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="active_steering_4wd",
            terrain_model="golf_heightfield",
            duration_sec=0.03,
            time_step=0.01,
            interface_mode="local",
            interface_enabled=True,
            interface_log_enabled=False,
            scene_in=scene_in,
            scene_out=scene_out,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )

    frame = pd.read_csv(result.log_path)
    exported = load_scene(scene_out)
    assert set(frame["robot_model"]) == {"df_mid"}
    assert set(frame["terrain_type"]) == {"flat"}
    assert exported.robot_model == "df_mid"
    assert exported.terrain == source.terrain
    assert len(exported.obstacles) == 1
    assert exported.obstacles[0].path is not None
    assert exported.obstacles[0].path.progress > moving.path.progress
    assert not hasattr(exported.obstacles[0], "body_id")


def test_strict_ecal_runtime_initialization_failure_cleans_before_disconnect(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """严格 eCAL 初始化首错必须传播，全部接口资源清理后才能断开 Bullet。"""
    from slope_sim.interfaces.transport import TransportSnapshot
    from slope_sim.scene_config import SceneDocument, SensorDocument, TerrainDocument

    trace: list[str] = []
    robot = SimpleNamespace(robot_id=41)
    world = SimpleNamespace(
        scene=SimpleNamespace(body_ids=(7,)),
        active_robot=SimpleNamespace(robot=robot, robot_model="df_back"),
        terrain=SimpleNamespace(slope_deg=0.0),
    )

    class Manager:
        def snapshot(self, *, include_body_id=False):
            return ()

    document = SceneDocument(
        1,
        "df_back",
        TerrainDocument("flat", 0.0, 0, "medium"),
        (),
        SensorDocument.default(),
    )

    class Backend:
        def __init__(self, client_id, robot_id):
            trace.append(("backend", client_id, robot_id))

        def bind_scene(self, terrain_ids, snapshots):
            trace.append(("bind", tuple(terrain_ids), tuple(snapshots)))

        def close(self):
            trace.append("backend.close")

    class Transport:
        def snapshot(self):
            return TransportSnapshot("ecal", False, 0, 0, 0, 0)

        def close(self):
            trace.append("transport.close")

    class Logger:
        def __init__(self, *_args, **_kwargs):
            trace.append("logger")

        def close(self):
            trace.append("logger.close")
            raise RuntimeError("secondary logger cleanup failure")

    class FailingRuntime:
        def __init__(self, *_args, **_kwargs):
            trace.append("runtime")
            raise RuntimeError("primary strict initialization failure")

    def create_strict_transport(mode, *, config, peer_state_callback):
        assert callable(peer_state_callback)
        trace.append(("transport", mode, config.transport_mode))
        return Transport()

    monkeypatch.setattr(simulation_module, "initial_scene_document", lambda _config: document)
    monkeypatch.setattr(simulation_module.p, "connect", lambda _mode: (trace.append("connect"), 5)[1])
    monkeypatch.setattr(simulation_module.p, "disconnect", lambda _client: trace.append("disconnect"))
    monkeypatch.setattr(simulation_module, "build_world_from_scene_document", lambda *_args, **_kwargs: (world, Manager()))
    monkeypatch.setattr(simulation_module, "PyBulletSensorBackend", Backend)
    monkeypatch.setattr(simulation_module, "create_transport", create_strict_transport)
    monkeypatch.setattr(simulation_module, "InterfaceEventLogger", Logger)
    monkeypatch.setattr(simulation_module, "InterfaceRuntime", FailingRuntime)

    with pytest.raises(RuntimeError, match="primary strict initialization failure"):
        run_experiment(
            ExperimentConfig(
                mode="direct",
                interface_mode="ecal",
                interface_enabled=True,
                interface_log_enabled=True,
                log_dir=tmp_path / "logs",
                figure_dir=tmp_path / "figures",
            )
        )

    assert trace[-4:] == [
        "logger.close",
        "transport.close",
        "backend.close",
        "disconnect",
    ]
    assert ("transport", "ecal", "ecal") in trace


@pytest.mark.parametrize("terrain_model", terrain_model_names())
@pytest.mark.parametrize("robot_model", robot_model_names())
def test_stage1_robot_terrain_matrix_stays_upright_and_moves(robot_model: str, terrain_model: str):
    client_id = p.connect(p.DIRECT)
    try:
        distance, clearance, roll, pitch = verify_combination(client_id, robot_model, terrain_model)
        assert distance >= 0.05
        assert 0.05 <= clearance <= 0.45
        assert roll < 0.7
        assert pitch < 0.7
    finally:
        p.disconnect(client_id)


def test_stage1_pose_validator_rejects_robot_without_ground_contact():
    """矩阵验收不能把尚未接触地面的悬空车辆判为通过。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = stage1_verifier.create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=1.0 / 240.0,
            terrain_model="flat",
        )
        robot = stage1_verifier.create_robot(client_id, "df_back", base_height=2.0)
        with pytest.raises(AssertionError, match="ground contact"):
            stage1_verifier.validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("robot_model", robot_model_names())
def test_all_robot_models_drive_from_upper_flat_across_ramp_to_lower_flat(robot_model: str):
    """四种车型必须真实驶过高位平台、下坡和低位平台。"""
    client_id = p.connect(p.DIRECT)
    try:
        time_step = 1.0 / 240.0
        scene = scene_module.create_slope_scene(
            client_id,
            slope_deg=8.0,
            time_step=time_step,
            terrain_model="slope",
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
        for settle_step in range(120):
            robot.command_twist(0.0, 0.0, dt=time_step)
            p.stepSimulation(physicsClientId=client_id)
            stage1_verifier.validate_robot_pose(
                client_id,
                robot,
                scene,
                require_ground_contact=settle_step >= 30,
            )

        ramp_half_x = scene_module.SLOPE_RAMP_LENGTH * math.cos(math.radians(8.0)) / 2.0
        samples: list[tuple[float, float]] = []
        entered_lower = False
        for _ in range(7200):
            robot.command_twist(0.7, 0.0, dt=time_step)
            p.stepSimulation(physicsClientId=client_id)
            stage1_verifier.validate_robot_pose(client_id, robot, scene, require_ground_contact=True)
            position, orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
            x = float(position[0])
            pitch = float(p.getEulerFromQuaternion(orientation)[1])
            samples.append((x, pitch))
            if x > ramp_half_x + 0.8:
                entered_lower = True
                break

        final_x = samples[-1][0]
        upper_pitch = [abs(pitch) for x, pitch in samples if x < -ramp_half_x - 0.30]
        ramp_pitch = [abs(pitch) for x, pitch in samples if -ramp_half_x + 0.30 < x < ramp_half_x - 0.30]
        lower_pitch = [abs(pitch) for x, pitch in samples if x > ramp_half_x + 0.30]
        assert entered_lower, f"{robot_model} did not enter lower flat: final_x={final_x:.3f}"
        assert upper_pitch, f"{robot_model} had no upper-flat pitch samples"
        assert ramp_pitch, f"{robot_model} had no ramp pitch samples"
        assert lower_pitch, f"{robot_model} had no lower-flat pitch samples"
        assert math.degrees(sum(upper_pitch) / len(upper_pitch)) < 2.0
        assert math.degrees(max(ramp_pitch)) > 5.0
        assert math.degrees(sum(lower_pitch) / len(lower_pitch)) < 2.0
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize(
    ("linear_velocity", "angular_velocity", "expected_x_sign", "expected_yaw_sign"),
    [
        (0.3, 0.0, 1, 0),
        (-0.3, 0.0, -1, 0),
        (0.3, 0.7, 1, 1),
        (0.3, -0.7, 1, -1),
        (0.0, 0.7, 0, 1),
    ],
)
@pytest.mark.parametrize("robot_model", ["df_front", "df_mid", "df_back"])
def test_differential_models_cover_stage1_motion_matrix(
    robot_model: str,
    linear_velocity: float,
    angular_velocity: float,
    expected_x_sign: int,
    expected_yaw_sign: int,
):
    """三种差速布局都自动覆盖前进、后退、左右转和差速转向。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = stage1_verifier.create_slope_scene(
            client_id,
            slope_deg=0.0,
            time_step=1.0 / 240.0,
            terrain_model="flat",
        )
        robot = stage1_verifier.create_robot(
            client_id,
            robot_model,
            start_x=scene.spawn_position[0],
            start_y=scene.spawn_position[1],
            base_height=scene.spawn_position[2] + stage1_verifier.create_robot_base_height(robot_model),
            start_orientation=scene.spawn_orientation,
        )
        robot.apply_drive_friction(lateral_friction=1.4, support_lateral_friction=0.03)
        for _ in range(120):
            robot.command_twist(0.0, 0.0, dt=1.0 / 240.0)
            p.stepSimulation(physicsClientId=client_id)
        start_position, start_orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
        start_drive_axle, _ = p.multiplyTransforms(
            start_position,
            start_orientation,
            (robot.drive_center_x, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        start_yaw = p.getEulerFromQuaternion(start_orientation)[2]

        for _ in range(360):
            robot.command_twist(linear_velocity, angular_velocity, dt=1.0 / 240.0)
            p.stepSimulation(physicsClientId=client_id)
            stage1_verifier.validate_robot_pose(client_id, robot, scene, require_ground_contact=True)

        end_position, end_orientation = p.getBasePositionAndOrientation(robot.robot_id, physicsClientId=client_id)
        end_drive_axle, _ = p.multiplyTransforms(
            end_position,
            end_orientation,
            (robot.drive_center_x, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        yaw_delta = p.getEulerFromQuaternion(end_orientation)[2] - start_yaw
        if expected_x_sign:
            assert expected_x_sign * (float(end_position[0]) - float(start_position[0])) > 0.20
        else:
            drive_axle_displacement = math.hypot(
                float(end_drive_axle[0]) - float(start_drive_axle[0]),
                float(end_drive_axle[1]) - float(start_drive_axle[1]),
            )
            assert drive_axle_displacement < 0.08
        if expected_yaw_sign:
            assert expected_yaw_sign * yaw_delta > 0.50
        else:
            assert abs(yaw_delta) < 0.05
    finally:
        p.disconnect(client_id)


@pytest.mark.parametrize("robot_model", ["df_front", "df_mid", "df_back"])
def test_differential_models_turn_in_place_on_flat(robot_model: str, tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model=robot_model,
            drive_model="physics",
            terrain_model="flat",
            slope_deg=0.0,
            duration_sec=1.2,
            time_step=1.0 / 240.0,
            target_linear_velocity=0.0,
            target_angular_velocity=0.7,
            interface_mode="local",
            log_dir=tmp_path / f"{robot_model}_logs",
            figure_dir=tmp_path / f"{robot_model}_figures",
        )
    )
    frame = pd.read_csv(result.log_path)
    assert frame["yaw"].iloc[-1] > 0.35
    assert frame.tail(100)["yaw_rate"].mean() > 0.35


def test_active_steering_4wd_forward_turn_has_drive_and_yaw_response(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="active_steering_4wd",
            drive_model="physics",
            terrain_model="flat",
            slope_deg=0.0,
            duration_sec=1.8,
            time_step=1.0 / 240.0,
            target_linear_velocity=0.35,
            target_angular_velocity=0.6,
            interface_mode="local",
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )
    frame = pd.read_csv(result.log_path)
    assert frame["x"].iloc[-1] - frame["x"].iloc[0] > 0.25
    assert frame["yaw"].iloc[-1] > 0.15
    assert frame.tail(120)["body_forward_speed"].mean() > 0.15


def test_physics_log_keeps_existing_internal_diagnostics(tmp_path: Path):
    result = run_experiment(
        ExperimentConfig(
            mode="direct",
            robot_model="df_mid",
            drive_model="physics",
            terrain_model="golf_heightfield",
            golf_seed=5,
            golf_relief="low",
            duration_sec=0.4,
            lidar_enabled=True,
            lidar_ray_count=9,
            log_dir=tmp_path / "logs",
            figure_dir=tmp_path / "figures",
        )
    )
    frame = pd.read_csv(result.log_path)
    expected = {
        "left_actual_drive_speed",
        "right_actual_drive_speed",
        "velocity_sensor_body_forward_speed",
        "local_ground_height",
        "local_terrain_normal_z",
        "lidar_min_distance",
    }
    assert expected.issubset(frame.columns)
    assert frame["terrain_probe_valid"].all()
    assert set(frame["terrain_type"]) == {"golf_heightfield"}


def test_robot_terrain_probe_filters_obstacle_on_offset_ray():
    """运行时遥测的侧向探测线被障碍物覆盖时，仍应采到真实地形。"""
    client_id = p.connect(p.DIRECT)
    try:
        scene = scene_module.create_slope_scene(client_id, slope_deg=0.0, time_step=1.0 / 240.0, terrain_model="flat")
        robot_id = p.createMultiBody(
            baseMass=0.0,
            basePosition=(0.0, 0.0, 0.20),
            physicsClientId=client_id,
        )
        collision_shape_id = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=(0.25, 0.25, 0.30),
            physicsClientId=client_id,
        )
        obstacle_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape_id,
            basePosition=(0.0, 0.45, 0.30),
            physicsClientId=client_id,
        )
        first_hit = p.rayTest((0.0, 0.45, 2.0), (0.0, 0.45, -2.0), physicsClientId=client_id)[0]

        probe = _probe_terrain_for_robot(client_id, SimpleNamespace(robot_id=robot_id), scene)

        assert first_hit[0] == obstacle_id
        assert probe.terrain_probe_valid is True
        assert probe.local_ground_height == pytest.approx(0.0, abs=1e-6)
        assert probe.local_terrain_normal_z == pytest.approx(1.0, abs=1e-6)
    finally:
        p.disconnect(client_id)
