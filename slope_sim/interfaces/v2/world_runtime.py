"""v2 runtime 的共享物理世界生命周期；只管理 worker 与协议代际，不创建 Bullet world。"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
import sys

from slope_sim.config import ExperimentConfig
from slope_sim.interfaces.v2.dashboard_snapshot import V2DashboardSnapshotStore
from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity, load_v2_descriptor
from slope_sim.interfaces.v2.runtime_protocol import V2RuntimeProtocol
from slope_sim.interfaces.v2.runsim_command_receiver import RunSimCommandReceiver
from slope_sim.interfaces.v2.sensor_frames import (
    V2AsyncSensorFrameFactory,
    V2OutputFramePublisher,
    V2WheelStateFactory,
)
from slope_sim.interfaces.v2.simulation_runtime import V2SimulatorRuntime
from slope_sim.interfaces.v2.transport import create_v2_ecal_transport
from slope_sim.lidar_worker import _PROTOCOL_VERSION, LidarScanService, LidarWorkerWorldSpec, start_lidar_worker, world_digest_for_document
from slope_sim.obstacles import ObstacleManager
from slope_sim.robot import DifferentialDriveRobot
from slope_sim.sensor_backend import PyBulletSensorBackend
from slope_sim.scene_config import SceneDocument
from slope_sim.truth_sensors import Stage4SensorMounts, Stage4TruthSensorSuite


def start_stage4_lidar_service(
    config: ExperimentConfig,
    document: SceneDocument,
    *,
    world_generation: int,
    worker_starter: Callable[..., object] = start_lidar_worker,
    service_factory: Callable[..., LidarScanService] = LidarScanService.from_worker_handle,
) -> tuple[object, LidarScanService]:
    """启动唯一可复用的 stage4 5,760-ray 异步中心 LiDAR service。"""
    if not callable(worker_starter):
        raise ValueError("worker_starter must be callable")
    if not callable(service_factory):
        raise ValueError("service_factory must be callable")
    handle = worker_starter(
        LidarWorkerWorldSpec(
            _PROTOCOL_VERSION,
            config,
            document,
            world_digest_for_document(document),
            "stage4",
        ),
        startup_timeout_sec=10.0,
    )
    return handle, service_factory(
        handle, lifecycle_generation=world_generation
    )


class V2WorldRuntime:
    """把 worker 重建和 v2 world/command generation 绑定为一笔事务。"""

    def __init__(
        self,
        *,
        controller: V2RuntimeProtocol,
        scene_document: SceneDocument,
        start_worker: Callable[[SceneDocument, int], object],
    ) -> None:
        if not isinstance(controller, V2RuntimeProtocol):
            raise ValueError("controller must be a V2RuntimeProtocol")
        if not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        if not callable(start_worker):
            raise ValueError("start_worker must be callable")
        self._controller = controller
        self._scene_document = scene_document
        self._start_worker = start_worker
        self._worker = self._start_worker(
            scene_document, self._controller.snapshot().world_generation
        )
        self._prepared: tuple[SceneDocument, object] | None = None

    @property
    def scene_document(self) -> SceneDocument:
        """返回当前 worker 已绑定的完整逻辑场景。"""
        return self._scene_document

    def update_moving_scene_document(self, scene_document: SceneDocument) -> None:
        """移动障碍物只推进逻辑快照，worker 仍由同帧 capture 快照驱动。"""
        if not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        if scene_document.sensors != self._scene_document.sensors:
            raise ValueError("scene_document sensors must match the active binding")
        self._scene_document = scene_document

    @property
    def worker(self) -> object:
        """返回当前 worker/service，由同一物理主线程消费。"""
        return self._worker

    def update_scene_document(self, scene_document: SceneDocument) -> None:
        """原子替换 worker；prepare 先让所有旧 command token 失效。"""
        if not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        try:
            self.prepare_world_rebuild()
            self.commit_world_rebuild(scene_document)
        except Exception:
            # close/启动失败也可能发生在 pending capture 尚未退出时；无论恢复是否
            # 成功，都必须撤销 controller 的 prepared 状态，避免旧 token 卡在半事务。
            if self._prepared is not None:
                self.abort_world_rebuild()
            raise

    def prepare_world_rebuild(self) -> None:
        """在删除 GUI world 前停止旧 worker 并失效旧 command ingress。"""
        if self._prepared is not None:
            raise RuntimeError("v2 world rebuild is already prepared")
        self._controller.prepare_world_rebuild()
        self._prepared = (self._scene_document, self._worker)
        self._close_worker(self._worker)

    def commit_world_rebuild(
        self,
        scene_document: SceneDocument,
        *,
        model: object | None = None,
    ) -> None:
        """为已构建的同一逻辑场景启动 worker，再提交新的 world generation。"""
        if not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        if self._prepared is None:
            raise RuntimeError("v2 world rebuild was not prepared")
        next_worker = self._start_worker(
            scene_document, self._controller.snapshot().world_generation + 1
        )
        self._worker = next_worker
        self._scene_document = scene_document
        self._prepared = None
        self._controller.commit_world_rebuild(model=model)

    def abort_world_rebuild(self) -> None:
        """恢复旧场景 worker；旧 command generation 保持失效。"""
        if self._prepared is None:
            raise RuntimeError("v2 world rebuild was not prepared")
        previous_document, _previous_worker = self._prepared
        self._prepared = None
        try:
            self._worker = self._start_worker(
                previous_document, self._controller.snapshot().world_generation
            )
            self._scene_document = previous_document
        finally:
            self._controller.abort_world_rebuild()

    def fault_world_rebuild(self) -> None:
        """无法恢复时终结 controller 事务，禁止旧 ingress 重新写入。"""
        self._prepared = None
        self._controller.fault_world_rebuild()

    def close(self) -> None:
        """先释放 worker，再关闭 command ingress 与 transport。"""
        self._close_worker(self._worker)
        self._controller.close()

    @staticmethod
    def _close_worker(worker: object) -> None:
        """优先走有界正常关闭；测试替身与异常路径可提供 force_close。"""
        begin_draining = getattr(worker, "begin_draining", None)
        close_idle = getattr(worker, "close_idle", None)
        if callable(begin_draining) and callable(close_idle):
            begin_draining()
            close_idle()
            return
        force_close = getattr(worker, "force_close", None)
        if callable(force_close):
            force_close()
            return
        raise RuntimeError("v2 worker must provide close_idle or force_close")


class V2ManualWorldRuntime:
    """把 v2 输出接到 GUI 已有的 coordinator world；不创建第二个主 PyBullet client。"""

    def __init__(
        self,
        *,
        config: ExperimentConfig,
        scene_document: SceneDocument,
        robot: DifferentialDriveRobot,
        sensor_backend: PyBulletSensorBackend,
        obstacle_manager: ObstacleManager,
        transport: object | None = None,
        session_id_factory: Callable[[], bytes] | None = None,
    ) -> None:
        if not isinstance(config, ExperimentConfig):
            raise ValueError("config must be an ExperimentConfig")
        if not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        if not isinstance(robot, DifferentialDriveRobot):
            raise ValueError("robot must be a DifferentialDriveRobot")
        if not isinstance(sensor_backend, PyBulletSensorBackend):
            raise ValueError("sensor_backend must be a PyBulletSensorBackend")
        if not isinstance(obstacle_manager, ObstacleManager):
            raise ValueError("obstacle_manager must be an ObstacleManager")
        self._config = config
        self._robot = robot
        self._sensor_backend = sensor_backend
        self._obstacle_manager = obstacle_manager
        self._descriptor = load_v2_descriptor()
        self._command_receiver = None
        self._command_receiver_version = 0
        self._command_observers: list[Callable[[bytes, float], None]] = []
        self._command_receiver_last_at: float | None = None
        self._command_receiver_last_accepted: bool | None = None
        self._command_receiver_last_sequence: int | None = None
        self._command_receiver_last_drive_max = 0.0
        self._command_receiver_next_diagnostic_at = 0.0
        self._command_codec = V2ProtoCodec(self._descriptor)
        self._transport = (
            create_v2_ecal_transport(descriptor=self._descriptor)
            if transport is None
            else transport
        )
        if transport is None:
            try:
                self._command_receiver = RunSimCommandReceiver.launch(
                    self._descriptor
                )
            except BaseException:
                self._transport.close()
                raise
        self._controller = V2RuntimeProtocol(
            robot.model_spec,
            transport=self._transport,
            descriptor=self._descriptor,
            session_id_factory=session_id_factory,
        )
        self._world_runtime = V2WorldRuntime(
            controller=self._controller,
            scene_document=scene_document,
            start_worker=self._start_lidar_service,
        )
        self._reset_dashboard_snapshot_store()
        self._runtime = self._make_runtime()

        subscribe = getattr(self._transport, "subscribe", None)
        if not callable(subscribe):
            self._world_runtime.close()
            raise RuntimeError("v2 transport must provide subscribe")
        if self._command_receiver is None:
            subscribe(
                "/sim/wheel/command",
                "slope_sim.interfaces.v2.WheelCommand",
                lambda payload, received_at: self._runtime.accept_command_payload(
                    payload, received_at=received_at
                ),
            )

    @property
    def scene_document(self) -> SceneDocument:
        """协调器用该文档验证物理 world 与 v2 worker 的唯一逻辑来源。"""
        return self._world_runtime.scene_document

    @property
    def descriptor(self) -> DescriptorIdentity:
        """返回 GUI 校验快照 descriptor 所需的同一份固定身份。"""
        return self._descriptor

    @property
    def dashboard_snapshot_store(self) -> V2DashboardSnapshotStore:
        """供 GUI 线程读取物理线程原子发布的 v2 不可变快照。"""
        return self._dashboard_snapshot_store

    def subscribe_command_observer(
        self, callback: Callable[[bytes, float], None],
    ) -> object:
        """把 Dashboard 观察回调挂到 Simulator 已有的 command transport。"""
        if not callable(callback):
            raise ValueError("callback must be callable")
        command_receiver = getattr(self, "_command_receiver", None)
        if command_receiver is not None:
            observers = getattr(self, "_command_observers", None)
            if observers is None:
                observers = []
                self._command_observers = observers
            observers.append(callback)
            return _CommandObserverSubscription(observers, callback)
        return self._transport.subscribe(
            "/sim/wheel/command",
            "slope_sim.interfaces.v2.WheelCommand",
            callback,
        )

    def protocol_snapshot(self) -> object:
        """返回同一锁内 session/world snapshot，供 Recorder 启动冻结身份。"""
        return self._controller.snapshot()

    def command_decision(self, *, now: float) -> object:
        """每个物理帧读取一次当前安全 command 决策。"""
        emit_diagnostic = False
        command_receiver = getattr(self, "_command_receiver", None)
        if command_receiver is not None:
            latest = command_receiver.take_latest(
                self._command_receiver_version
            )
            if latest is not None:
                version, payload, received_at = latest
                self._command_receiver_version = version
                accepted = self._runtime.accept_command_payload(
                    payload,
                    received_at=received_at,
                )
                self._command_receiver_last_at = received_at
                self._command_receiver_last_accepted = accepted
                if getattr(
                    getattr(self, "_config", None),
                    "developer_diagnostics_enabled",
                    False,
                ):
                    decoded = self._command_codec.decode_wheel_command(payload)
                    self._command_receiver_last_sequence = decoded.sequence
                    self._command_receiver_last_drive_max = max(
                        (abs(value) for value in decoded.drive_wheel_speed_rad_s),
                        default=0.0,
                    )
                for callback in tuple(getattr(self, "_command_observers", ())):
                    callback(payload, received_at)
            if (
                getattr(
                    getattr(self, "_config", None),
                    "developer_diagnostics_enabled",
                    False,
                )
                and now >= self._command_receiver_next_diagnostic_at
            ):
                emit_diagnostic = True
                age_ms = (
                    None
                    if self._command_receiver_last_at is None
                    else max(0.0, now - self._command_receiver_last_at) * 1000.0
                )
                self._command_receiver_next_diagnostic_at = now + 1.0
        decision = self._runtime.command_decision(now=now)
        if emit_diagnostic:
            protocol = self._controller.snapshot()
            print(
                "v2-command-receiver "
                f"version={self._command_receiver_version} "
                f"age_ms={'none' if age_ms is None else f'{age_ms:.3f}'} "
                f"accepted={self._command_receiver_last_accepted} "
                f"sequence={self._command_receiver_last_sequence} "
                f"max_drive={self._command_receiver_last_drive_max:.6f} "
                f"decision_max={max((abs(value) for value in decision.drive_wheel_speed_rad_s), default=0.0):.6f} "
                f"waiting={decision.waiting} timed_out={decision.timed_out} "
                f"protocol={protocol.command_protocol_state} "
                f"peers={protocol.authority.peer_count} "
                f"authority={protocol.authority.state.name}",
                file=sys.stderr,
                flush=True,
            )
        return decision

    def refresh_transport(self) -> object:
        """在低频观测节拍刷新 raw discovery。"""
        return self._runtime.refresh_transport()

    def poll_transport(self) -> None:
        """兼容共享 RuntimeObservationCadence 的最小 observer 接口。"""
        self.refresh_transport()

    def after_physics_step(self, dt: float, *, wall_time: float) -> object:
        """在 coordinator 完成同一主 world 的 Bullet 步进后发布到期 v2 输出。"""
        return self._runtime.after_physics_step(dt, wall_time=wall_time)

    def prepare_world_rebuild(self) -> None:
        """由 coordinator 在删除旧 body 前调用。"""
        self._world_runtime.prepare_world_rebuild()

    def commit_world_rebuild(
        self,
        robot: DifferentialDriveRobot,
        sensor_backend: PyBulletSensorBackend,
        scene_document: SceneDocument,
    ) -> None:
        """新 body 已完成构建后换 worker，并让输出切到同一新 generation。"""
        self._validate_rebind(robot, sensor_backend, scene_document)
        self._world_runtime.commit_world_rebuild(
            scene_document,
            model=robot.model_spec,
        )
        self._robot = robot
        self._sensor_backend = sensor_backend
        self._reset_dashboard_snapshot_store()
        self._runtime = self._make_runtime()

    def abort_world_rebuild(self) -> None:
        """目标 world 未提交时恢复旧 worker；旧 command token 不恢复。"""
        self._world_runtime.abort_world_rebuild()
        self._reset_dashboard_snapshot_store()
        self._runtime = self._make_runtime()

    def fault_world_rebuild(self) -> None:
        """协调器无法恢复时停止协议重建事务。"""
        self._world_runtime.fault_world_rebuild()

    def update_scene_document(self, scene_document: SceneDocument) -> None:
        """障碍物集合改变时重建 worker，并推进 v2 generation。"""
        if not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        # 移动障碍物每物理帧都会通知 v1 runtime；v2 capture 已携带同刻快照，
        # 只有逻辑场景实际改变时才允许 worker/generation 重建。
        if scene_document == self._world_runtime.scene_document:
            return
        self._world_runtime.update_scene_document(scene_document)
        self._reset_dashboard_snapshot_store()
        self._runtime = self._make_runtime()

    def update_moving_scene_document(self, scene_document: SceneDocument) -> None:
        """移动障碍物只更新同帧逻辑快照，不重建 worker 或 generation。"""
        if not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        self._world_runtime.update_moving_scene_document(scene_document)

    def refresh_scene_bindings(
        self,
        terrain_body_ids: tuple[int, ...],
        obstacle_snapshots: tuple[object, ...],
        scene_document: SceneDocument,
    ) -> None:
        """障碍物事务提交后刷新主 world 分类，并原子切到对应 worker generation。"""
        if type(terrain_body_ids) is not tuple:
            raise ValueError("terrain_body_ids must be an exact tuple")
        if type(obstacle_snapshots) is not tuple:
            raise ValueError("obstacle_snapshots must be an exact tuple")
        self._sensor_backend.bind_scene(terrain_body_ids, obstacle_snapshots)
        self.update_scene_document(scene_document)

    def bind_obstacle_manager(self, obstacle_manager: ObstacleManager) -> None:
        """完整 scene transaction 提交后切换到 coordinator 新安装的障碍物管理器。"""
        if not isinstance(obstacle_manager, ObstacleManager):
            raise ValueError("obstacle_manager must be an ObstacleManager")
        self._obstacle_manager = obstacle_manager

    def close(self) -> None:
        """先排空异步 LiDAR，再关闭唯一 v2 command ingress。"""
        self._runtime.drain_sensor_outputs(timeout_sec=5.0)
        if self._command_receiver is not None:
            self._command_receiver.close()
        self._world_runtime.close()

    def _start_lidar_service(self, document: SceneDocument, generation: int) -> LidarScanService:
        """仅启动既有 stage4 5,760-ray worker profile，禁止驾驶期 dense 离线路径。"""
        _handle, service = start_stage4_lidar_service(
            self._config, document, world_generation=generation
        )
        return service

    def _make_runtime(self) -> V2SimulatorRuntime:
        """将当前主 world 的 robot/backend 绑定到当前 worker，不拥有 Bullet client。"""
        return V2SimulatorRuntime(
            controller=self._controller,
            wheel_feedback_reader=self._robot.read_interface_wheel_state,
            sensor_frames=V2AsyncSensorFrameFactory(
                self._controller,
                self._world_runtime.worker,
                Stage4TruthSensorSuite(self._sensor_backend, Stage4SensorMounts.default()),
                self._capture_context,
            ),
            output_publisher=V2OutputFramePublisher(
                self._transport, self._descriptor
            ),
            wheel_state_factory=V2WheelStateFactory(
                self._controller, self._robot.model_spec.name
            ),
            dashboard_snapshot_store=self._dashboard_snapshot_store,
        )

    def _reset_dashboard_snapshot_store(self) -> None:
        """generation 变化后切换空 store，避免 GUI 混合两个物理世界的帧。"""
        self._dashboard_snapshot_store = V2DashboardSnapshotStore(
            robot_model=self._robot.model_spec.name,
        )

    def _capture_context(self) -> tuple[object, tuple[object, ...]]:
        """在主物理线程冻结 mount 和无 body-id 障碍物，再交给 child worker。"""
        return (
            self._sensor_backend.world_pose("lidar_link"),
            tuple(
                replace(snapshot, body_id=None)
                for snapshot in self._obstacle_manager.snapshot(include_body_id=True)
            ),
        )

    @staticmethod
    def _validate_rebind(
        robot: DifferentialDriveRobot,
        sensor_backend: PyBulletSensorBackend,
        scene_document: SceneDocument,
    ) -> None:
        if not isinstance(robot, DifferentialDriveRobot):
            raise ValueError("robot must be a DifferentialDriveRobot")
        if not isinstance(sensor_backend, PyBulletSensorBackend):
            raise ValueError("sensor_backend must be a PyBulletSensorBackend")
        if not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")


class _CommandObserverSubscription:
    """sidecar latest command 的进程内观察订阅，可由 Dashboard 幂等关闭。"""

    def __init__(
        self,
        callbacks: list[Callable[[bytes, float], None]],
        callback: Callable[[bytes, float], None],
    ) -> None:
        self._callbacks = callbacks
        self._callback = callback
        self._closed = False

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._callbacks.remove(self._callback)
        except ValueError:
            pass
