# LiDAR worker 合同测试：覆盖父端有界调度、真实 spawn、冻结帧和故障收口。
from __future__ import annotations

from dataclasses import fields, is_dataclass
from enum import IntEnum
import importlib
import inspect
import multiprocessing
import os
import signal
from threading import Event, Thread
from types import SimpleNamespace

import pybullet as p
import pytest

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import build_world_from_scene_document
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.dashboard_snapshot import LidarTopViewFrame
from slope_sim.interfaces.models import LidarPointCloud
from slope_sim.lidar_pointcloud import LidarScanResult, MultiLineLidar
from slope_sim.obstacles import (
    ObstacleGeometry,
    ObstaclePath,
    ObstacleSnapshot,
    ObstacleSpec,
    update_kinematic_obstacle,
)
from slope_sim.runtime_actions import TerrainSelection
from slope_sim.scene_config import SceneDocument, SensorDocument
from slope_sim.sensor_backend import Pose, PyBulletSensorBackend


class _IntegerEnum(IntEnum):
    ONE = 1


def _worker_module():
    """延迟导入待实现模块，让 RED 停在测试函数断言而非 collection。"""
    try:
        return importlib.import_module("slope_sim.lidar_worker")
    except ModuleNotFoundError:
        return SimpleNamespace()


def _world_inputs() -> tuple[ExperimentConfig, SceneDocument]:
    """构造最小生产世界输入，不引入任何运行时 body id。"""
    config = ExperimentConfig(mode="direct", robot_model="df_back", terrain_model="flat")
    document = SceneDocument.from_runtime(
        "df_back",
        TerrainSelection("flat"),
        (),
        SensorDocument.default().mounts,
    )
    return config, document


def _worker_world_spec():
    """构造包含逻辑障碍物的 spawn 输入，禁止传递任何 body id。"""
    module = _worker_module()
    config = ExperimentConfig(mode="gui", robot_model="df_back", terrain_model="flat")
    obstacle = ObstacleSpec(
        logical_id=1,
        mode="static",
        geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
        position=(2.0, 0.0, 0.3),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    document = SceneDocument.from_runtime(
        "df_back",
        TerrainSelection("flat"),
        (obstacle,),
        SensorDocument.default().mounts,
    )
    return module.LidarWorkerWorldSpec(
        1,
        config,
        document,
        module.world_digest_for_document(document),
    )


def _bodyless_obstacle_snapshots() -> tuple[ObstacleSnapshot, ObstacleSnapshot]:
    """构造一静一动的完整逻辑快照，不携带任何进程的 body id。"""
    return (
        ObstacleSnapshot(
            logical_id=11,
            body_id=None,
            mode="static",
            shape="box",
            position=(2.0, 0.0, 0.3),
            orientation=(0.0, 0.0, 0.0, 1.0),
            geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
        ),
        ObstacleSnapshot(
            logical_id=12,
            body_id=None,
            mode="moving",
            shape="sphere",
            position=(-2.0, 0.0, 0.2),
            orientation=(0.0, 0.0, 0.0, 1.0),
            path=ObstaclePath((-2.5, 0.0), (-1.5, 0.0), 0.4, 0.5, 1),
            geometry=ObstacleGeometry("sphere", (0.2, 0.2, 0.2)),
        ),
    )


def _initial_worker_snapshots() -> tuple[ObstacleSnapshot, ...]:
    """返回与 `_worker_world_spec` 启动场景一致的无 body-id 快照。"""
    return (
        ObstacleSnapshot(
            logical_id=1,
            body_id=None,
            mode="static",
            shape="box",
            position=(2.0, 0.0, 0.3),
            orientation=(0.0, 0.0, 0.0, 1.0),
            geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
        ),
    )


def _reconcile_snapshots(*, moving_x: float, include_static: bool) -> tuple[ObstacleSnapshot, ...]:
    """构造可观测新增、删除与移动的前雷达完整逻辑集合。"""
    moving = ObstacleSnapshot(
        logical_id=12,
        body_id=None,
        mode="moving",
        shape="box",
        position=(moving_x, -0.35, 0.55),
        orientation=(0.0, 0.0, 0.0, 1.0),
        path=ObstaclePath((1.0, -0.35), (4.0, -0.35), 0.4, 0.5, 1),
        geometry=ObstacleGeometry("box", (0.30, 0.30, 0.55)),
    )
    if not include_static:
        return (moving,)
    static = ObstacleSnapshot(
        logical_id=11,
        body_id=None,
        mode="static",
        shape="box",
        position=(2.0, 0.45, 0.55),
        orientation=(0.0, 0.0, 0.0, 1.0),
        geometry=ObstacleGeometry("box", (0.30, 0.30, 0.55)),
    )
    return (static, moving)


def _scan_request(
    module,
    *,
    job_id: int,
    topic: str = "lidar_front",
    timestamp_ns: int = 900_000_000,
    base_pose: Pose | None = None,
    snapshots: tuple[ObstacleSnapshot, ...] = (),
):
    """构造真实 child 可消费的完整帧请求。"""
    lidar_id = 1 if topic == "lidar_front" else 2
    return module.LidarScanRequest(
        1,
        job_id,
        123_000_000 + job_id,
        3,
        2,
        topic,
        topic,
        lidar_id,
        timestamp_ns,
        Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)),
        base_pose,
        snapshots,
    )


def _receive_job_response(handle, *, timeout_sec: float = 15.0):
    """有界读取 job 结果，并把过早 EOF 转成可读的行为断言。"""
    assert handle.response_receiver.poll(timeout_sec), "worker job response timed out"
    try:
        return handle.response_receiver.recv()
    except EOFError:
        pytest.fail("worker response pipe closed before returning a job result")


class _FakeLidarChannel:
    """完整模拟 Connection 的 send/poll/recv 边界，不依赖队列容量或等待。"""

    def __init__(self) -> None:
        self.sent: list[object] = []
        self.responses: list[object] = []
        self.send_error: BaseException | None = None
        self.poll_error: BaseException | None = None

    def send(self, value: object) -> None:
        if self.send_error is not None:
            raise self.send_error
        self.sent.append(value)

    def poll(self, timeout: float = 0.0) -> bool:
        assert timeout == 0.0
        if self.poll_error is not None:
            raise self.poll_error
        return bool(self.responses)

    def recv(self) -> object:
        if not self.responses:
            raise EOFError("fake response channel is empty")
        response = self.responses.pop(0)
        if isinstance(response, BaseException):
            raise response
        return response


class _BlockingPromotionChannel(_FakeLidarChannel):
    """只阻塞 pending 提升的第二次 send，用于线性化并发边界。"""

    def __init__(self) -> None:
        super().__init__()
        self.send_calls = 0
        self.promotion_entered = Event()
        self.release_promotion = Event()
        self.timeline: list[str] = []

    def send(self, value: object) -> None:
        self.send_calls += 1
        if self.send_calls == 2:
            self.timeline.append("promotion_entered")
            self.promotion_entered.set()
            assert self.release_promotion.wait(timeout=3.0)
            self.timeline.append("promotion_sent")
        super().send(value)


class _FakeMonotonicNs:
    """显式推进的单调时钟，延迟测试不做真实 sleep。"""

    def __init__(self, now_ns: int = 1_000_000_000) -> None:
        self.now_ns = now_ns

    def __call__(self) -> int:
        return self.now_ns

    def advance(self, duration_ns: int) -> None:
        self.now_ns += duration_ns


class _BlockingSecondMonotonicNs(_FakeMonotonicNs):
    """阻塞第二次 capture 的时钟读取，固定 ready-check 后的竞态窗口。"""

    def __init__(self) -> None:
        super().__init__()
        self.calls = 0
        self.capture_entered = Event()
        self.release_capture = Event()
        self.timeline: list[str] = []

    def __call__(self) -> int:
        self.calls += 1
        if self.calls == 2:
            self.timeline.append("capture_entered")
            self.capture_entered.set()
            assert self.release_capture.wait(timeout=3.0)
        return self.now_ns


def _scan_service(module, channel: _FakeLidarChannel, clock: _FakeMonotonicNs):
    """构造绑定固定 generation/epoch 的纯父端 service。"""
    service_type = getattr(module, "LidarScanService", None)
    assert service_type is not None, "LidarScanService must exist"
    return service_type(
        channel,
        child_pid=42,
        lifecycle_generation=3,
        pause_epoch=2,
        monotonic_ns=clock,
    )


def _worker_handle_without_child(module, channel: _FakeLidarChannel):
    """构造 exact worker handle；测试替换 close，绝不启动真实 child。"""
    ready = module.LidarWorkerReady(
        1,
        42,
        "1" * 64,
        ("lidar_front", "lidar_rear"),
        (("lidar_front", "2" * 64), ("lidar_rear", "3" * 64)),
        1,
    )
    return module.LidarWorkerHandle(
        SimpleNamespace(),
        channel,
        channel,
        ready,
    )


def _capture(
    service,
    *,
    topic: str = "lidar_front",
    timestamp_ns: int = 900_000_000,
) -> bool:
    """按 runtime 将使用的公开入口提交一份原子、无 body-id capture。"""
    return service.capture(
        topic=topic,
        timestamp_ns=timestamp_ns,
        world_mount_pose=Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)),
        optional_base_pose=None,
        complete_obstacle_snapshots_without_body_ids=(),
    )


def _prepared_response(module, request, *, payload: bytes = b"prepared"):
    """从 fake channel 已收到的请求构造精确同身份成功响应。"""
    message = LidarPointCloud(
        request.timestamp_ns,
        request.frame_id,
        0,
        request.lidar_id,
        (),
    )
    return module.PreparedLidarFrame(
        request.protocol_version,
        request.job_id,
        request.lifecycle_generation,
        request.pause_epoch,
        request.topic,
        request.timestamp_ns,
        message,
        None,
        payload,
        1,
    )


def _failure_response(
    module,
    request,
    *,
    error_code: str = "codec_failed",
    detail: str = "codec failed",
):
    """构造精确匹配请求身份的 typed 单帧失败。"""
    return module.LidarScanFailure(
        request.protocol_version,
        request.job_id,
        request.lifecycle_generation,
        request.pause_epoch,
        request.topic,
        request.timestamp_ns,
        error_code,
        detail,
        1,
    )


def _forced_preflight_spawn_entrypoint(
    request_receiver,
    response_sender,
    world_spec,
    phase: str,
) -> None:
    """测试专用顶层 wrapper：公共 child 入口保持只使用真实依赖。"""
    module = importlib.import_module("slope_sim.lidar_worker")
    result = module._bootstrap_live_worker(
        world_spec,
        forced_failure_phase=phase,
    )
    envelope = result if type(result) is module.LidarWorkerStartupFailure else result.ready
    try:
        response_sender.send(envelope)
    finally:
        response_sender.close()
        request_receiver.close()
    if type(result) is not module.LidarWorkerStartupFailure:
        module._disconnect_direct_client(result.client_id)
        raise SystemExit(0)
    raise SystemExit(1)


def _rollback_failure_spawn_entrypoint(
    request_receiver,
    response_sender,
    world_spec,
) -> None:
    """只在 child 内注入删除后回滚失败，生产入口和真实 PyBullet 路径不加测试 API。"""
    module = importlib.import_module("slope_sim.lidar_worker")
    original_create = module._create_worker_body
    original_remove = module._remove_worker_body
    create_calls = 0
    remove_calls = 0

    def fail_first_rollback_create(client_id, spec):
        nonlocal create_calls
        create_calls += 1
        if create_calls == 3:
            raise RuntimeError("forced rollback body creation failure")
        return original_create(client_id, spec)

    def fail_second_target_remove(client_id, body_id):
        nonlocal remove_calls
        remove_calls += 1
        if remove_calls == 3:
            raise RuntimeError("forced second target removal failure")
        return original_remove(client_id, body_id)

    module._create_worker_body = fail_first_rollback_create
    module._remove_worker_body = fail_second_target_remove
    module.lidar_worker_entrypoint(request_receiver, response_sender, world_spec)


def _ignore_sigterm_child(ready_sender) -> None:
    """安装 SIGTERM 忽略器，用于证明 owned-process 回收会升级到 kill。"""
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    try:
        ready_sender.send(os.getpid())
    finally:
        ready_sender.close()
    while True:
        signal.pause()


def test_lidar_worker_entrypoint_is_importable_and_callable() -> None:
    """子进程入口必须是模块顶层可 pickle 函数。"""
    module = _worker_module()

    entrypoint = getattr(module, "lidar_worker_entrypoint", None)
    assert callable(entrypoint), "lidar_worker_entrypoint must be importable and callable"


def test_realtime_verifier_uses_production_spawn_service_and_contract() -> None:
    """本地 verifier 必须运行真实 spawn worker，不得改用同步或测试替身。"""
    try:
        verifier = importlib.import_module("scripts.verify_lidar_worker_realtime")
    except ModuleNotFoundError:
        verifier = SimpleNamespace()
    run = getattr(verifier, "run_lidar_worker_realtime_verifier", None)
    assert callable(run), "run_lidar_worker_realtime_verifier must exist"

    source = inspect.getsource(run)
    assert "p.connect(p.DIRECT)" in source
    assert "start_lidar_worker(" in source
    assert "LidarScanService.from_worker_handle(" in source
    assert "DeadlinePacer(" in source
    assert "service.capture(" in source
    assert "optional_base_pose=None" in source
    assert "service.poll()" in source
    assert "service.force_close()" in source


def test_realtime_verifier_rejects_sim_wall_ratio_below_p0_oracle() -> None:
    """本地预检沿用 P0 的 sim/wall >= 0.95 下限，不能自行放宽。"""
    verifier = importlib.import_module("scripts.verify_lidar_worker_realtime")
    require_ratio = getattr(verifier, "_require_sim_wall_ratio", None)
    assert callable(require_ratio), "_require_sim_wall_ratio must exist"

    assert require_ratio(228, 1.0) == pytest.approx(0.95)
    with pytest.raises(RuntimeError, match="sim/wall"):
        require_ratio(227, 1.0)


def test_realtime_verifier_rejects_any_single_window_below_sim_wall_oracle() -> None:
    """十个窗口必须逐个过 P0，下一个健康窗口不能掩盖前一窗口。"""
    verifier = importlib.import_module("scripts.verify_lidar_worker_realtime")
    require_window_ratio = getattr(verifier, "_require_window_sim_wall_ratio", None)
    assert callable(require_window_ratio), "_require_window_sim_wall_ratio must exist"

    assert require_window_ratio(0, 228, 10.0, 11.0) == pytest.approx(0.95)
    with pytest.raises(RuntimeError, match="sim/wall"):
        require_window_ratio(228, 455, 11.0, 12.0)


def test_realtime_verifier_rejects_nonzero_or_missing_worker_exitcode() -> None:
    """正常 Stop/Stopped 后必须确认 owned worker 精确以 0 退出。"""
    verifier = importlib.import_module("scripts.verify_lidar_worker_realtime")
    require_exitcode = getattr(verifier, "_require_clean_worker_exitcode", None)
    assert callable(require_exitcode), "_require_clean_worker_exitcode must exist"

    assert require_exitcode(0) is None
    for exitcode in (None, 1, -15, True):
        with pytest.raises(RuntimeError, match="exitcode"):
            require_exitcode(exitcode)


def test_worker_contract_values_are_frozen_slotted_and_strict() -> None:
    """启动 IPC 值拒绝隐式转换和不完整的预热摘要。"""
    module = _worker_module()
    config, document = _world_inputs()
    digest = module.world_digest_for_document(document)

    expected_fields = {
        module.LidarWorkerWorldSpec: (
            "protocol_version",
            "experiment_config",
            "scene_document",
            "world_digest",
        ),
        module.LidarWorkerReady: (
            "protocol_version",
            "process_id",
            "world_digest",
            "prewarmed_topics",
            "prewarm_payload_sha256_by_topic",
            "prewarm_max_scan_wall_duration_ns",
        ),
        module.LidarWorkerStartupFailure: (
            "protocol_version",
            "process_id",
            "phase",
            "stable_error_code",
            "bounded_detail",
        ),
    }
    for value_type, names in expected_fields.items():
        assert is_dataclass(value_type)
        assert value_type.__dataclass_params__.frozen is True
        assert hasattr(value_type, "__slots__")
        assert tuple(field.name for field in fields(value_type)) == names

    spec = module.LidarWorkerWorldSpec(1, config, document, digest)
    assert spec.experiment_config == config
    assert spec.scene_document == document
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerWorldSpec(True, config, document, digest)
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerWorldSpec(1, object(), document, digest)
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerWorldSpec(1, config, document, "0" * 64)

    ready = module.LidarWorkerReady(
        1,
        42,
        digest,
        ("lidar_front", "lidar_rear"),
        (
            ("lidar_front", "1" * 64),
            ("lidar_rear", "2" * 64),
        ),
        123,
    )
    assert ready.prewarm_payload_sha256_by_topic == (
        ("lidar_front", "1" * 64),
        ("lidar_rear", "2" * 64),
    )
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerReady(1, True, digest, ready.prewarmed_topics, ready.prewarm_payload_sha256_by_topic, 1)
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerReady(1, 42, digest, ("lidar_front", "lidar_front"), ready.prewarm_payload_sha256_by_topic, 1)
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerReady(1, 42, digest, ready.prewarmed_topics, (("lidar_front", "1" * 64),), 1)
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerReady(1, 1 << 64, digest, ready.prewarmed_topics, ready.prewarm_payload_sha256_by_topic, 1)

    failure = module.LidarWorkerStartupFailure(
        1,
        42,
        "front_preflight",
        "worker_preflight_failed",
        "front scan failed",
    )
    assert failure.phase == "front_preflight"
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerStartupFailure(1, 42, "front_preflight", "worker_start_failed", "wrong code")
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerStartupFailure(1, 42, "front_preflight", "worker_preflight_failed", "line one\nline two")
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerStartupFailure(1, 42, "front_preflight", "worker_preflight_failed", "x" * 513)


def test_worker_contract_rejects_intenum_for_uint64_fields() -> None:
    """IntEnum 不得借由整数继承混入协议版本、PID 或时长字段。"""
    module = _worker_module()
    config, document = _world_inputs()
    digest = module.world_digest_for_document(document)

    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerWorldSpec(_IntegerEnum.ONE, config, document, digest)
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerReady(
            _IntegerEnum.ONE,
            42,
            digest,
            ("lidar_front", "lidar_rear"),
            (("lidar_front", "1" * 64), ("lidar_rear", "2" * 64)),
            1,
        )
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerReady(
            1,
            _IntegerEnum.ONE,
            digest,
            ("lidar_front", "lidar_rear"),
            (("lidar_front", "1" * 64), ("lidar_rear", "2" * 64)),
            1,
        )
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerReady(
            1,
            42,
            digest,
            ("lidar_front", "lidar_rear"),
            (("lidar_front", "1" * 64), ("lidar_rear", "2" * 64)),
            _IntegerEnum.ONE,
        )
    with pytest.raises((TypeError, ValueError)):
        module.LidarWorkerStartupFailure(
            1,
            _IntegerEnum.ONE,
            "world_build",
            "worker_preflight_failed",
            "failed",
        )


def test_worker_stop_contract_values_are_frozen_slotted_and_strict() -> None:
    """正常关闭请求与 ACK 使用 exact 版本和 owned child PID。"""
    module = _worker_module()
    stop_type = getattr(module, "LidarWorkerStop", None)
    stopped_type = getattr(module, "LidarWorkerStopped", None)
    assert stop_type is not None, "LidarWorkerStop must exist"
    assert stopped_type is not None, "LidarWorkerStopped must exist"

    for value_type in (stop_type, stopped_type):
        assert value_type.__dataclass_params__.frozen is True
        assert hasattr(value_type, "__slots__")
        assert tuple(field.name for field in fields(value_type)) == (
            "protocol_version",
            "process_id",
        )
        value = value_type(1, 42)
        assert value.protocol_version == 1
        assert value.process_id == 42
        with pytest.raises((TypeError, ValueError)):
            value_type(2, 42)
        with pytest.raises((TypeError, ValueError)):
            value_type(1, True)
        with pytest.raises((TypeError, ValueError)):
            value_type(1, 0)


def test_worker_frame_contract_values_are_frozen_slotted_and_strict() -> None:
    """帧 IPC 只接受精确身份、冻结位姿和完整无 body-id 快照。"""
    module = _worker_module()
    request_type = getattr(module, "LidarScanRequest", None)
    prepared_type = getattr(module, "PreparedLidarFrame", None)
    failure_type = getattr(module, "LidarScanFailure", None)
    assert request_type is not None, "LidarScanRequest must exist"
    assert prepared_type is not None, "PreparedLidarFrame must exist"
    assert failure_type is not None, "LidarScanFailure must exist"

    expected_fields = {
        request_type: (
            "protocol_version",
            "job_id",
            "captured_monotonic_ns",
            "lifecycle_generation",
            "pause_epoch",
            "topic",
            "frame_id",
            "lidar_id",
            "timestamp_ns",
            "world_mount_pose",
            "optional_base_pose",
            "complete_obstacle_snapshots_without_body_ids",
        ),
        prepared_type: (
            "protocol_version",
            "job_id",
            "lifecycle_generation",
            "pause_epoch",
            "topic",
            "timestamp_ns",
            "message",
            "optional_top_view",
            "protobuf_payload",
            "scan_wall_duration_ns",
        ),
        failure_type: (
            "protocol_version",
            "job_id",
            "lifecycle_generation",
            "pause_epoch",
            "topic",
            "timestamp_ns",
            "stable_error_code",
            "bounded_detail",
            "scan_wall_duration_ns",
        ),
    }
    for value_type, names in expected_fields.items():
        assert is_dataclass(value_type)
        assert value_type.__dataclass_params__.frozen is True
        assert hasattr(value_type, "__slots__")
        assert tuple(field.name for field in fields(value_type)) == names

    mount = Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0))
    base = Pose((0.0, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0))
    snapshots = _bodyless_obstacle_snapshots()
    request = request_type(
        1,
        7,
        123,
        3,
        2,
        "lidar_front",
        "lidar_front",
        1,
        900_000_000,
        mount,
        base,
        snapshots,
    )
    assert request.world_mount_pose == mount
    assert request.optional_base_pose == base
    assert request.complete_obstacle_snapshots_without_body_ids == snapshots

    invalid_requests = (
        (True, 7, 123, 3, 2, "lidar_front", "lidar_front", 1, 900_000_000, mount, base, snapshots),
        (1, _IntegerEnum.ONE, 123, 3, 2, "lidar_front", "lidar_front", 1, 900_000_000, mount, base, snapshots),
        (1, 7, 123, 3, 2, "lidar_front", "lidar_rear", 1, 900_000_000, mount, base, snapshots),
        (1, 7, 123, 3, 2, "lidar_front", "lidar_front", 2, 900_000_000, mount, base, snapshots),
        (1, 7, 123, 3, 2, "lidar_front", "lidar_front", 1, 900_000_000, object(), base, snapshots),
        (1, 7, 123, 3, 2, "lidar_front", "lidar_front", 1, 900_000_000, mount, base, list(snapshots)),
        (
            1,
            7,
            123,
            3,
            2,
            "lidar_front",
            "lidar_front",
            1,
            900_000_000,
            mount,
            base,
            (
                ObstacleSnapshot(
                    11,
                    99,
                    "static",
                    "box",
                    (2.0, 0.0, 0.3),
                    (0.0, 0.0, 0.0, 1.0),
                    geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
                ),
            ),
        ),
        (1, 7, 123, 3, 2, "lidar_front", "lidar_front", 1, 900_000_000, mount, base, (snapshots[0], snapshots[0])),
    )
    for arguments in invalid_requests:
        with pytest.raises((TypeError, ValueError)):
            request_type(*arguments)

    message = LidarPointCloud(900_000_000, "lidar_front", 0, 1, ())
    top_view = LidarTopViewFrame(900_000_000, ())
    prepared = prepared_type(1, 7, 3, 2, "lidar_front", 900_000_000, message, top_view, b"payload", 44)
    assert prepared.protobuf_payload == b"payload"
    with pytest.raises((TypeError, ValueError)):
        prepared_type(1, 7, 3, 2, "lidar_front", 900_000_001, message, top_view, b"payload", 44)
    with pytest.raises((TypeError, ValueError)):
        prepared_type(1, 7, 3, 2, "lidar_front", 900_000_000, message, top_view, bytearray(b"payload"), 44)
    with pytest.raises((TypeError, ValueError)):
        prepared_type(1, 7, 3, 2, "lidar_front", 900_000_000, message, top_view, b"", 44)

    failure = failure_type(1, 7, 3, 2, "lidar_front", 900_000_000, "codec_failed", "codec failed", 22)
    assert failure.stable_error_code == "codec_failed"
    with pytest.raises((TypeError, ValueError)):
        failure_type(1, 7, 3, 2, "lidar_front", 900_000_000, "unknown_code", "failed", 22)
    with pytest.raises((TypeError, ValueError)):
        failure_type(1, 7, 3, 2, "lidar_front", 900_000_000, "codec_failed", "line one\nline two", 22)


def test_service_event_and_snapshot_contracts_are_frozen_slotted_and_strict() -> None:
    """父端事件与诊断快照必须固定字段，并拒绝模糊身份和交叉 scope。"""
    module = _worker_module()
    event_type = getattr(module, "LidarServiceEvent", None)
    snapshot_type = getattr(module, "LidarServiceSnapshot", None)
    assert event_type is not None, "LidarServiceEvent must exist"
    assert snapshot_type is not None, "LidarServiceSnapshot must exist"

    expected_fields = {
        event_type: (
            "sequence",
            "kind",
            "scope",
            "optional_topic",
            "optional_job_identity",
            "stable_error_code",
            "bounded_detail",
        ),
        snapshot_type: (
            "state",
            "child_pid",
            "lifecycle_generation",
            "pause_epoch",
            "next_job_id",
            "in_flight_identity",
            "pending_capture_identity",
            "completed_count",
            "failed_count",
            "overrun_count",
            "stale_count",
            "max_capture_to_response_ns",
            "last_error_code",
            "last_error_detail",
        ),
    }
    for value_type, names in expected_fields.items():
        assert is_dataclass(value_type)
        assert value_type.__dataclass_params__.frozen is True
        assert hasattr(value_type, "__slots__")
        assert tuple(field.name for field in fields(value_type)) == names

    job_identity = (7, 3, 2, "lidar_front", 900_000_000)
    event = event_type(
        1,
        "frame_failed",
        "topic",
        "lidar_front",
        job_identity,
        "codec_failed",
        "codec failed",
    )
    assert event.optional_job_identity == job_identity
    service_event = event_type(
        2,
        "service_failed",
        "service",
        None,
        job_identity,
        "worker_protocol_failed",
        "response mismatch",
    )
    assert service_event.optional_topic is None

    invalid_events = (
        (_IntegerEnum.ONE, "frame_failed", "topic", "lidar_front", job_identity, "codec_failed", "failed"),
        (1, "unknown", "topic", "lidar_front", job_identity, "codec_failed", "failed"),
        (1, "frame_failed", "service", None, job_identity, "codec_failed", "failed"),
        (1, "frame_failed", "topic", "lidar_rear", job_identity, "codec_failed", "failed"),
        (1, "capture_rejected", "topic", "lidar_front", job_identity, "sensor_overrun", "full"),
        (1, "service_failed", "service", "lidar_front", None, "worker_protocol_failed", "failed"),
        (1, "service_failed", "service", None, list(job_identity), "worker_protocol_failed", "failed"),
    )
    for arguments in invalid_events:
        with pytest.raises((TypeError, ValueError)):
            event_type(*arguments)

    snapshot = snapshot_type(
        "ready",
        42,
        3,
        2,
        8,
        job_identity,
        (3, 2, "lidar_rear", 950_000_000),
        4,
        1,
        1,
        0,
        80_000_000,
        "codec_failed",
        "codec failed",
    )
    assert snapshot.in_flight_identity == job_identity
    with pytest.raises((TypeError, ValueError)):
        snapshot_type("unknown", 42, 3, 2, 8, None, None, 0, 0, 0, 0, 0, None, "")
    with pytest.raises((TypeError, ValueError)):
        snapshot_type("ready", True, 3, 2, 8, None, None, 0, 0, 0, 0, 0, None, "")
    with pytest.raises((TypeError, ValueError)):
        snapshot_type("ready", 42, 3, 2, 0, None, None, 0, 0, 0, 0, 0, None, "")
    with pytest.raises((TypeError, ValueError)):
        snapshot_type("ready", 42, 3, 2, 8, None, None, 0, 0, 0, 0, 0, None, "orphan detail")


def test_service_close_idle_closes_owned_handle_once_and_is_idempotent(
    monkeypatch,
) -> None:
    """service 接管 exact handle；空闲关闭成功后重复调用不得二次关闭。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    handle = _worker_handle_without_child(module, channel)
    close_calls: list[tuple[object, float]] = []

    def record_close(owned_handle, timeout_sec: float = 5.0) -> None:
        close_calls.append((owned_handle, timeout_sec))

    monkeypatch.setattr(module.LidarWorkerHandle, "close", record_close)
    service = module.LidarScanService.from_worker_handle(
        handle,
        lifecycle_generation=3,
        pause_epoch=2,
    )

    assert service.close_idle(timeout_sec=0.25) is None
    assert service.snapshot().state == "closed"
    assert service.close_idle(timeout_sec=9.0) is None
    assert close_calls == [(handle, 0.25)]


def test_service_close_idle_rejects_busy_and_preserves_owned_close_error(
    monkeypatch,
) -> None:
    """繁忙 service 不触碰 handle；空闲 close 原异常透传并可再次重试。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    handle = _worker_handle_without_child(module, channel)
    close_error = RuntimeError("owned lidar worker close failed")
    close_calls: list[tuple[object, float]] = []

    def fail_once_then_close(owned_handle, timeout_sec: float = 5.0) -> None:
        close_calls.append((owned_handle, timeout_sec))
        if len(close_calls) == 1:
            raise close_error

    monkeypatch.setattr(module.LidarWorkerHandle, "close", fail_once_then_close)
    service = module.LidarScanService.from_worker_handle(
        handle,
        lifecycle_generation=3,
        pause_epoch=2,
    )

    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    with pytest.raises(RuntimeError, match="in-flight|idle"):
        service.close_idle(timeout_sec=0.25)
    assert close_calls == []

    assert _capture(service, topic="lidar_rear", timestamp_ns=950_000_000) is True
    busy = service.snapshot()
    assert busy.in_flight_identity is not None
    assert busy.pending_capture_identity is not None
    with pytest.raises(RuntimeError, match="in-flight|pending|idle"):
        service.close_idle(timeout_sec=0.25)
    assert close_calls == []

    first_request = channel.sent[0]
    channel.responses.append(_prepared_response(module, first_request))
    assert service.poll() is not None
    second_request = channel.sent[1]
    channel.responses.append(_prepared_response(module, second_request))
    assert service.poll() is not None

    with pytest.raises(RuntimeError) as captured:
        service.close_idle(timeout_sec=0.25)
    assert captured.value is close_error
    assert service.snapshot().state != "closed"
    assert service.close_idle(timeout_sec=0.5) is None
    assert service.snapshot().state == "closed"
    assert close_calls == [(handle, 0.25), (handle, 0.5)]


def test_service_keeps_one_pending_without_writing_it_to_pipe() -> None:
    """第二份 capture 只占父端 pending，不提前分配 ID 或写入 pipe。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    clock = _FakeMonotonicNs()
    service = _scan_service(module, channel, clock)

    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    clock.advance(10_000_000)
    assert _capture(service, topic="lidar_rear", timestamp_ns=950_000_000) is True

    assert len(channel.sent) == 1
    request = channel.sent[0]
    assert type(request) is module.LidarScanRequest
    assert request.job_id == 1
    assert request.captured_monotonic_ns == 1_000_000_000
    snapshot = service.snapshot()
    assert snapshot.in_flight_identity == (1, 3, 2, "lidar_front", 900_000_000)
    assert snapshot.pending_capture_identity == (3, 2, "lidar_rear", 950_000_000)
    assert snapshot.next_job_id == 2


def test_service_rejects_third_capture_without_overwriting_older_jobs() -> None:
    """容量满时拒绝最新 capture，并保留已经承诺的两份旧工作。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    service = _scan_service(module, channel, _FakeMonotonicNs())

    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    assert _capture(service, topic="lidar_rear", timestamp_ns=950_000_000) is True
    before = service.snapshot()
    assert _capture(service, topic="lidar_front", timestamp_ns=1_000_000_000) is False

    after = service.snapshot()
    assert len(channel.sent) == 1
    assert after.in_flight_identity == before.in_flight_identity
    assert after.pending_capture_identity == before.pending_capture_identity
    assert after.next_job_id == before.next_job_id
    assert service.drain_events() == (
        module.LidarServiceEvent(
            1,
            "capture_rejected",
            "topic",
            "lidar_front",
            None,
            "sensor_overrun",
            "lidar capture capacity is full",
        ),
    )


def test_service_assigns_job_id_only_when_capture_enters_pipe() -> None:
    """pending 保留原 capture 时钟，只有提升并写 pipe 时才取得下一个 ID。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    clock = _FakeMonotonicNs()
    service = _scan_service(module, channel, clock)

    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    clock.advance(10_000_000)
    assert _capture(service, topic="lidar_rear", timestamp_ns=950_000_000) is True
    first_request = channel.sent[0]
    first_response = _prepared_response(module, first_request)
    channel.responses.append(first_response)

    assert service.poll() == first_response
    assert len(channel.sent) == 2
    second_request = channel.sent[1]
    assert type(second_request) is module.LidarScanRequest
    assert second_request.job_id == 2
    assert second_request.captured_monotonic_ns == 1_010_000_000
    snapshot = service.snapshot()
    assert snapshot.next_job_id == 3
    assert snapshot.in_flight_identity == (2, 3, 2, "lidar_rear", 950_000_000)
    assert snapshot.pending_capture_identity is None


def test_pause_cancels_pending_without_job_gap() -> None:
    """pause 丢弃未发送 capture；旧 in-flight 收敛后下次发送仍连续编号。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    service = _scan_service(module, channel, _FakeMonotonicNs())

    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    assert _capture(service, topic="lidar_rear", timestamp_ns=950_000_000) is True
    first_request = channel.sent[0]
    service.pause()

    paused = service.snapshot()
    assert paused.state == "suspended"
    assert paused.pause_epoch == 3
    assert paused.pending_capture_identity is None
    assert paused.next_job_id == 2
    channel.responses.append(_prepared_response(module, first_request))
    assert service.poll() is None

    service.resume()
    assert _capture(service, topic="lidar_front", timestamp_ns=1_000_000_000) is True
    resumed_request = channel.sent[-1]
    assert resumed_request.job_id == 2
    assert resumed_request.pause_epoch == 3


def test_disconnect_invalidates_old_generation_without_faulting_service() -> None:
    """断线 retag 只失效旧工作，并保持同一 service 的状态与累计计数连续。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    service = _scan_service(module, channel, _FakeMonotonicNs())

    # 先完成一帧，再构造不可取消的旧 in-flight 和可撤销的旧 pending。
    assert _capture(service, topic="lidar_front", timestamp_ns=850_000_000) is True
    completed_request = channel.sent[0]
    completed_response = _prepared_response(module, completed_request, payload=b"completed")
    channel.responses.append(completed_response)
    assert service.poll() == completed_response
    assert _capture(service, topic="lidar_rear", timestamp_ns=900_000_000) is True
    assert _capture(service, topic="lidar_front", timestamp_ns=950_000_000) is True
    old_request = channel.sent[1]
    before = service.snapshot()

    invalidate_generation = getattr(service, "invalidate_generation", None)
    assert callable(
        invalidate_generation
    ), "LidarScanService.invalidate_generation must exist"
    invalidate_generation(4)

    retagged = service.snapshot()
    assert retagged.state == before.state == "ready"
    assert retagged.lifecycle_generation == 4
    assert retagged.pause_epoch == before.pause_epoch == 2
    assert retagged.next_job_id == before.next_job_id == 3
    assert retagged.in_flight_identity == before.in_flight_identity
    assert retagged.pending_capture_identity is None
    assert (
        retagged.completed_count,
        retagged.failed_count,
        retagged.overrun_count,
        retagged.stale_count,
        retagged.max_capture_to_response_ns,
        retagged.last_error_code,
        retagged.last_error_detail,
    ) == (
        before.completed_count,
        before.failed_count,
        before.overrun_count,
        before.stale_count,
        before.max_capture_to_response_ns,
        before.last_error_code,
        before.last_error_detail,
    ) == (1, 0, 0, 0, 0, None, "")
    assert service.drain_events() == ()

    # 新 generation 可先占 pending；旧响应只计 stale，随后提升新工作。
    assert _capture(service, topic="lidar_rear", timestamp_ns=1_000_000_000) is True
    pending = service.snapshot()
    assert pending.pending_capture_identity == (4, 2, "lidar_rear", 1_000_000_000)
    assert pending.next_job_id == 3
    channel.responses.append(_prepared_response(module, old_request, payload=b"stale"))
    assert service.poll() is None

    assert len(channel.sent) == 3
    new_request = channel.sent[2]
    assert (
        new_request.job_id,
        new_request.lifecycle_generation,
        new_request.pause_epoch,
        new_request.topic,
        new_request.timestamp_ns,
    ) == (3, 4, 2, "lidar_rear", 1_000_000_000)
    after_stale = service.snapshot()
    assert after_stale.state == "ready"
    assert after_stale.completed_count == 1
    assert after_stale.stale_count == 1
    assert after_stale.failed_count == 0
    assert after_stale.next_job_id == 4
    assert service.drain_events() == ()

    new_response = _prepared_response(module, new_request, payload=b"new-generation")
    channel.responses.append(new_response)
    assert service.poll() == new_response
    finished = service.snapshot()
    assert finished.state == "ready"
    assert finished.lifecycle_generation == 4
    assert finished.completed_count == 2
    assert finished.stale_count == 1
    assert finished.in_flight_identity is None
    assert finished.pending_capture_identity is None
    assert service.drain_events() == ()


@pytest.mark.parametrize("transition", ("pause", "invalidate"))
def test_lifecycle_transition_linearizes_after_pending_promotion_send(
    transition: str,
) -> None:
    """pause/retag 返回前必须串行化已开始的 pending pipe 提升。"""
    module = _worker_module()
    channel = _BlockingPromotionChannel()
    service = _scan_service(module, channel, _FakeMonotonicNs())
    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    assert _capture(service, topic="lidar_rear", timestamp_ns=950_000_000) is True
    first_request = channel.sent[0]
    channel.responses.append(_prepared_response(module, first_request))

    errors: list[BaseException] = []
    transition_started = Event()
    transition_done = Event()

    def poll_response() -> None:
        try:
            service.poll()
        except BaseException as error:
            errors.append(error)

    def apply_transition() -> None:
        transition_started.set()
        try:
            if transition == "pause":
                service.pause()
            else:
                service.invalidate_generation(4)
            channel.timeline.append("transition_done")
        except BaseException as error:
            errors.append(error)
        finally:
            transition_done.set()

    poll_thread = Thread(target=poll_response)
    transition_thread = Thread(target=apply_transition)
    poll_thread.start()
    assert channel.promotion_entered.wait(timeout=3.0)
    transition_thread.start()
    assert transition_started.wait(timeout=3.0)
    # 无锁实现会在 promotion 仍阻塞时提前返回；有锁实现等待 release。
    transition_done.wait(timeout=1.0)
    channel.release_promotion.set()
    poll_thread.join(timeout=3.0)
    transition_thread.join(timeout=3.0)

    assert not poll_thread.is_alive()
    assert not transition_thread.is_alive()
    assert errors == []
    assert channel.timeline == [
        "promotion_entered",
        "promotion_sent",
        "transition_done",
    ]
    snapshot = service.snapshot()
    if transition == "pause":
        assert snapshot.state == "suspended"
        assert snapshot.pause_epoch == 3
    else:
        assert snapshot.state == "ready"
        assert snapshot.lifecycle_generation == 4


def test_pause_linearizes_with_capture_after_ready_check() -> None:
    """pause 返回后不得让已越过 ready 检查的 capture 重建 pending。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    clock = _BlockingSecondMonotonicNs()
    service = _scan_service(module, channel, clock)
    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    errors: list[BaseException] = []
    pause_started = Event()
    pause_done = Event()

    def capture_pending() -> None:
        try:
            _capture(service, topic="lidar_rear", timestamp_ns=950_000_000)
            clock.timeline.append("capture_finished")
        except BaseException as error:
            errors.append(error)

    def pause_service() -> None:
        pause_started.set()
        try:
            service.pause()
            clock.timeline.append("pause_done")
        except BaseException as error:
            errors.append(error)
        finally:
            pause_done.set()

    capture_thread = Thread(target=capture_pending)
    pause_thread = Thread(target=pause_service)
    capture_thread.start()
    assert clock.capture_entered.wait(timeout=3.0)
    pause_thread.start()
    assert pause_started.wait(timeout=3.0)
    pause_done.wait(timeout=1.0)
    clock.release_capture.set()
    capture_thread.join(timeout=3.0)
    pause_thread.join(timeout=3.0)

    assert not capture_thread.is_alive()
    assert not pause_thread.is_alive()
    assert errors == []
    assert clock.timeline == ["capture_entered", "capture_finished", "pause_done"]
    snapshot = service.snapshot()
    assert snapshot.state == "suspended"
    assert snapshot.pending_capture_identity is None


@pytest.mark.parametrize(
    "response_kind",
    ("mismatched", "duplicate", "out_of_order", "invalid_failure_code"),
)
def test_service_rejects_mismatched_duplicate_or_out_of_order_response(
    response_kind: str,
) -> None:
    """任何无法精确匹配当前 in-flight 的响应都永久 fault 整个 service。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    service = _scan_service(module, channel, _FakeMonotonicNs())
    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    first_request = channel.sent[0]
    first_response = _prepared_response(module, first_request)

    if response_kind == "duplicate":
        channel.responses.append(first_response)
        assert service.poll() == first_response
        channel.responses.append(first_response)
        expected_identity = None
    elif response_kind == "mismatched":
        wrong_request = _scan_request(
            module,
            job_id=1,
            topic="lidar_rear",
            timestamp_ns=950_000_000,
        )
        channel.responses.append(_prepared_response(module, wrong_request))
        expected_identity = (1, 3, 2, "lidar_front", 900_000_000)
    elif response_kind == "out_of_order":
        future_request = _scan_request(
            module,
            job_id=2,
            topic="lidar_front",
            timestamp_ns=900_000_000,
        )
        channel.responses.append(_prepared_response(module, future_request))
        expected_identity = (1, 3, 2, "lidar_front", 900_000_000)
    else:
        channel.responses.append(
            _failure_response(module, first_request, error_code="worker_exited")
        )
        expected_identity = (1, 3, 2, "lidar_front", 900_000_000)

    assert service.poll() is None
    snapshot = service.snapshot()
    assert snapshot.state == "failed"
    assert snapshot.last_error_code == "worker_protocol_failed"
    assert _capture(service, topic="lidar_rear", timestamp_ns=950_000_000) is False
    events = service.drain_events()
    assert len(events) == 1
    assert events[0].kind == "service_failed"
    assert events[0].scope == "service"
    assert events[0].optional_topic is None
    assert events[0].optional_job_identity == expected_identity
    assert events[0].stable_error_code == "worker_protocol_failed"
    assert service.poll() is None
    assert service.drain_events() == ()


def test_service_marks_job_over_hundred_milliseconds_as_overrun_once() -> None:
    """延迟必须严格大于 100 ms 才计错，迟到帧只丢弃且不重复发事件。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    clock = _FakeMonotonicNs()
    service = _scan_service(module, channel, clock)
    assert _capture(service, timestamp_ns=900_000_000) is True
    request = channel.sent[0]

    clock.advance(100_000_000)
    assert service.poll() is None
    assert service.snapshot().overrun_count == 0
    assert service.drain_events() == ()

    clock.advance(1)
    assert service.poll() is None
    first_events = service.drain_events()
    assert first_events == (
        module.LidarServiceEvent(
            1,
            "job_overrun",
            "topic",
            "lidar_front",
            (1, 3, 2, "lidar_front", 900_000_000),
            "sensor_overrun",
            "lidar job exceeded 100 ms capture-to-response budget",
        ),
    )
    assert service.snapshot().overrun_count == 1
    assert service.poll() is None
    assert service.drain_events() == ()

    channel.responses.append(_prepared_response(module, request))
    assert service.poll() is None
    completed = service.snapshot()
    assert completed.in_flight_identity is None
    assert completed.completed_count == 0
    assert completed.overrun_count == 1
    assert completed.max_capture_to_response_ns == 100_000_001
    assert service.drain_events() == ()


def test_service_overrun_includes_parent_side_pending_wait() -> None:
    """pending 提升时若 capture 已超预算，应立即按新分配的 job 身份计错。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    clock = _FakeMonotonicNs()
    service = _scan_service(module, channel, clock)
    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    assert _capture(service, topic="lidar_rear", timestamp_ns=950_000_000) is True
    first_request = channel.sent[0]

    clock.advance(100_000_001)
    channel.responses.append(_prepared_response(module, first_request))
    assert service.poll() is None

    assert len(channel.sent) == 2
    assert channel.sent[1].job_id == 2
    events = service.drain_events()
    assert tuple(event.kind for event in events) == ("job_overrun", "job_overrun")
    assert tuple(event.optional_job_identity for event in events) == (
        (1, 3, 2, "lidar_front", 900_000_000),
        (2, 3, 2, "lidar_rear", 950_000_000),
    )
    assert service.snapshot().overrun_count == 2


def test_service_events_are_typed_ordered_and_consumed_once() -> None:
    """多类 outcome 共享连续序号，drain 后绝不以累计计数重复归因。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    clock = _FakeMonotonicNs()
    service = _scan_service(module, channel, clock)

    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    assert _capture(service, topic="lidar_rear", timestamp_ns=950_000_000) is True
    assert _capture(service, topic="lidar_front", timestamp_ns=1_000_000_000) is False
    first_request = channel.sent[0]
    channel.responses.append(_prepared_response(module, first_request))
    assert service.poll() is not None

    second_request = channel.sent[1]
    channel.responses.append(_failure_response(module, second_request))
    assert service.poll() is None
    assert _capture(service, topic="lidar_front", timestamp_ns=1_050_000_000) is True
    third_request = channel.sent[2]
    clock.advance(100_000_001)
    assert service.poll() is None
    late_response = _prepared_response(module, third_request)
    channel.responses.append(late_response)
    assert service.poll() is None
    channel.responses.append(late_response)
    assert service.poll() is None

    events = service.drain_events()
    assert all(type(event) is module.LidarServiceEvent for event in events)
    assert tuple(event.sequence for event in events) == (1, 2, 3, 4)
    assert tuple(event.kind for event in events) == (
        "capture_rejected",
        "frame_failed",
        "job_overrun",
        "service_failed",
    )
    assert tuple(event.scope for event in events) == (
        "topic",
        "topic",
        "topic",
        "service",
    )
    assert events[1].optional_topic == "lidar_rear"
    assert events[1].stable_error_code == "codec_failed"
    assert events[3].optional_topic is None
    assert events[3].stable_error_code == "worker_protocol_failed"
    assert service.drain_events() == ()


def test_unknown_scene_state_faults_service_once_and_cancels_pending() -> None:
    """镜像状态不可证明时终结整个 service，不能继续提升下一帧。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    clock = _FakeMonotonicNs()
    service = _scan_service(module, channel, clock)
    assert _capture(service, topic="lidar_front", timestamp_ns=900_000_000) is True
    assert _capture(service, topic="lidar_rear", timestamp_ns=950_000_000) is True
    request = channel.sent[0]
    channel.responses.append(
        _failure_response(
            module,
            request,
            error_code="scene_state_unknown",
            detail="scene rollback could not be proven",
        )
    )
    clock.advance(100_000_001)

    assert service.poll() is None

    snapshot = service.snapshot()
    assert snapshot.state == "failed"
    assert snapshot.in_flight_identity == (
        request.job_id,
        request.lifecycle_generation,
        request.pause_epoch,
        request.topic,
        request.timestamp_ns,
    )
    assert snapshot.pending_capture_identity is None
    assert snapshot.failed_count == 1
    assert snapshot.last_error_code == "scene_state_unknown"
    assert len(channel.sent) == 1
    events = service.drain_events()
    assert len(events) == 1
    assert events[0].kind == "service_failed"
    assert events[0].scope == "service"
    assert events[0].optional_topic is None
    assert events[0].optional_job_identity == snapshot.in_flight_identity
    assert events[0].stable_error_code == "scene_state_unknown"
    assert service.poll() is None
    assert service.drain_events() == ()


def test_service_send_failure_is_terminal_without_allocating_job_id() -> None:
    """请求 pipe 断开时锁存基础设施错误，未发送 job 不得消耗连续 ID。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    channel.send_error = BrokenPipeError("worker pipe closed")
    service = _scan_service(module, channel, _FakeMonotonicNs())

    assert _capture(service, timestamp_ns=900_000_000) is False
    snapshot = service.snapshot()
    assert snapshot.state == "failed"
    assert snapshot.next_job_id == 1
    assert snapshot.in_flight_identity is None
    assert snapshot.last_error_code == "worker_exited"
    assert channel.sent == []
    events = service.drain_events()
    assert len(events) == 1
    assert events[0].kind == "service_failed"
    assert events[0].optional_job_identity is None
    assert events[0].stable_error_code == "worker_exited"
    assert _capture(service, timestamp_ns=950_000_000) is False
    assert service.drain_events() == ()


@pytest.mark.parametrize(
    ("boundary", "error", "expected_code"),
    (
        ("poll", EOFError("response pipe closed"), "worker_exited"),
        ("poll", RuntimeError("poll failed"), "worker_protocol_failed"),
        ("recv", OSError("response pipe failed"), "worker_exited"),
        ("recv", ValueError("decode failed"), "worker_protocol_failed"),
    ),
)
def test_service_response_channel_failure_is_terminal_and_emitted_once(
    boundary: str,
    error: BaseException,
    expected_code: str,
) -> None:
    """response poll/recv 断管或协议错误必须永久 fault 且只归因一次。"""
    module = _worker_module()
    channel = _FakeLidarChannel()
    service = _scan_service(module, channel, _FakeMonotonicNs())
    assert _capture(service, timestamp_ns=900_000_000) is True

    if boundary == "poll":
        channel.poll_error = error
    else:
        channel.responses.append(error)

    assert service.poll() is None
    snapshot = service.snapshot()
    assert snapshot.state == "failed"
    assert snapshot.last_error_code == expected_code
    events = service.drain_events()
    assert len(events) == 1
    assert events[0].kind == "service_failed"
    assert events[0].stable_error_code == expected_code
    assert _capture(service, timestamp_ns=950_000_000) is False
    assert service.poll() is None
    assert service.drain_events() == ()


def test_spawned_worker_returns_preencoded_atomic_frame() -> None:
    """Ready 后同一响应管道连续返回同源 message、俯视帧和预编码 bytes。"""
    module = _worker_module()
    handle = module.start_lidar_worker(_worker_world_spec(), startup_timeout_sec=15.0)
    try:
        base_pose = Pose((0.0, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0))
        front_request = _scan_request(
            module,
            job_id=1,
            base_pose=base_pose,
            snapshots=_initial_worker_snapshots(),
        )
        handle.request_sender.send(front_request)
        front = _receive_job_response(handle)

        assert type(front) is module.PreparedLidarFrame
        assert (
            front.job_id,
            front.lifecycle_generation,
            front.pause_epoch,
            front.topic,
            front.timestamp_ns,
        ) == (
            front_request.job_id,
            front_request.lifecycle_generation,
            front_request.pause_epoch,
            front_request.topic,
            front_request.timestamp_ns,
        )
        assert front.message.frame_id == front_request.frame_id
        assert front.message.lidar_id == front_request.lidar_id
        assert front.message.timebase_ns == front_request.timestamp_ns
        assert front.optional_top_view is not None
        assert front.optional_top_view.timestamp_ns == front_request.timestamp_ns
        assert len(front.optional_top_view.points) == len(front.message.points)
        assert tuple(point.tag for point in front.optional_top_view.points) == tuple(
            point.tag for point in front.message.points
        )
        assert front.protobuf_payload == ProtoCodec().encode(front.message)

        rear_request = _scan_request(
            module,
            job_id=2,
            topic="lidar_rear",
            timestamp_ns=1_000_000_000,
            base_pose=base_pose,
            snapshots=_initial_worker_snapshots(),
        )
        handle.request_sender.send(rear_request)
        rear = _receive_job_response(handle)
        assert type(rear) is module.PreparedLidarFrame
        assert rear.job_id == rear_request.job_id
        assert rear.optional_top_view is not None
        assert rear.protobuf_payload == ProtoCodec().encode(rear.message)
    finally:
        handle.close()


def test_spawned_headless_worker_returns_compact_payload() -> None:
    """无 top-view 请求只跨 IPC 返回身份、bytes 和时长，不携带逐点对象。"""
    module = _worker_module()
    compact_type = getattr(module, "PreparedLidarPayload", None)
    assert compact_type is not None, "PreparedLidarPayload must exist"
    handle = module.start_lidar_worker(_worker_world_spec(), startup_timeout_sec=15.0)
    try:
        request = _scan_request(
            module,
            job_id=3,
            timestamp_ns=1_050_000_000,
            snapshots=_initial_worker_snapshots(),
        )
        handle.request_sender.send(request)
        response = _receive_job_response(handle)

        assert type(response) is compact_type
        assert tuple(field.name for field in fields(type(response))) == (
            "protocol_version",
            "job_id",
            "lifecycle_generation",
            "pause_epoch",
            "topic",
            "timestamp_ns",
            "protobuf_payload",
            "scan_wall_duration_ns",
        )
        assert (
            response.job_id,
            response.lifecycle_generation,
            response.pause_epoch,
            response.topic,
            response.timestamp_ns,
        ) == (3, 3, 2, "lidar_front", 1_050_000_000)
        assert type(response.protobuf_payload) is bytes
        assert response.protobuf_payload
        assert not hasattr(response, "message")
        assert not hasattr(response, "optional_top_view")
    finally:
        handle.close()


def test_spawned_worker_reconciles_complete_obstacle_snapshot_by_logical_id() -> None:
    """真实 child 按逻辑 ID 原子新增、删除和移动，不消费父进程 body id。"""
    module = _worker_module()
    handle = module.start_lidar_worker(_worker_world_spec(), startup_timeout_sec=15.0)
    try:
        added_snapshots = _reconcile_snapshots(moving_x=3.2, include_static=True)
        assert all(snapshot.body_id is None for snapshot in added_snapshots)
        added_request = _scan_request(
            module,
            job_id=10,
            base_pose=Pose((0.0, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0)),
            snapshots=added_snapshots,
        )
        handle.request_sender.send(added_request)
        added = _receive_job_response(handle)

        assert type(added) is module.PreparedLidarFrame
        assert {2, 3}.issubset({point.tag for point in added.message.points})
        initial_moving_range = min(
            (point.x * point.x + point.y * point.y + point.z * point.z) ** 0.5
            for point in added.message.points
            if point.tag == 3
        )

        moved_snapshots = _reconcile_snapshots(moving_x=1.45, include_static=False)
        moved_request = _scan_request(
            module,
            job_id=11,
            timestamp_ns=1_000_000_000,
            base_pose=Pose((0.0, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0)),
            snapshots=moved_snapshots,
        )
        handle.request_sender.send(moved_request)
        moved = _receive_job_response(handle)

        assert type(moved) is module.PreparedLidarFrame
        moved_tags = {point.tag for point in moved.message.points}
        assert 2 not in moved_tags
        assert 3 in moved_tags
        moved_range = min(
            (point.x * point.x + point.y * point.y + point.z * point.z) ** 0.5
            for point in moved.message.points
            if point.tag == 3
        )
        assert moved_range < initial_moving_range - 1.0
        assert tuple(field.name for field in fields(type(moved))) == (
            "protocol_version",
            "job_id",
            "lifecycle_generation",
            "pause_epoch",
            "topic",
            "timestamp_ns",
            "message",
            "optional_top_view",
            "protobuf_payload",
            "scan_wall_duration_ns",
        )
    finally:
        handle.close()


def test_spawned_worker_frame_payload_matches_direct_codec_bytes() -> None:
    """相同冻结世界与位姿下，spawn worker 必须逐字节等于同步生产 codec。"""
    module = _worker_module()
    world_spec = _worker_world_spec()
    request = _scan_request(
        module,
        job_id=20,
        timestamp_ns=1_200_000_000,
        base_pose=Pose((0.0, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0)),
        snapshots=_initial_worker_snapshots(),
    )
    client_id = p.connect(p.DIRECT)
    handle = None
    try:
        world, manager = build_world_from_scene_document(
            client_id,
            world_spec.experiment_config,
            world_spec.scene_document,
        )
        backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
        direct_records = manager.snapshot(include_body_id=True)
        assert len(direct_records) == len(request.complete_obstacle_snapshots_without_body_ids) == 1
        direct_record = direct_records[0]
        frozen_snapshot = request.complete_obstacle_snapshots_without_body_ids[0]
        assert direct_record.logical_id == frozen_snapshot.logical_id
        assert direct_record.body_id is not None
        update_kinematic_obstacle(
            client_id,
            direct_record.body_id,
            position=frozen_snapshot.position,
            orientation=frozen_snapshot.orientation,
            linear_velocity=(0.0, 0.0, 0.0),
        )
        backend.bind_scene(world.scene.body_ids, direct_records)
        scanner = MultiLineLidar(
            backend,
            world_spec.scene_document.sensors.lidar,
            world_spec.scene_document.sensors.mounts.lidar_front,
            frame_id="lidar_front",
            lidar_id=1,
        )
        direct_result = scanner._scan_frozen(
            request.timestamp_ns,
            request.world_mount_pose,
            request.optional_base_pose,
        )
        assert type(direct_result) is LidarScanResult
        direct_message = direct_result.message
        assert type(direct_message) is LidarPointCloud
        direct_payload = ProtoCodec().encode(direct_message)

        handle = module.start_lidar_worker(world_spec, startup_timeout_sec=15.0)
        handle.request_sender.send(request)
        worker_frame = _receive_job_response(handle)

        assert type(worker_frame) is module.PreparedLidarFrame
        first_mismatch = next(
            (
                (index, worker_point, direct_point)
                for index, (worker_point, direct_point) in enumerate(
                    zip(worker_frame.message.points, direct_message.points, strict=False)
                )
                if worker_point != direct_point
            ),
            None,
        )
        assert worker_frame.message == direct_message, (
            f"worker/direct counts={len(worker_frame.message.points)}/{len(direct_message.points)} "
            f"first_mismatch={first_mismatch!r}"
        )
        assert worker_frame.protobuf_payload == direct_payload
    finally:
        if handle is not None:
            handle.close()
        p.disconnect(client_id)


def test_reconcile_rollback_failure_returns_unknown_scene_state() -> None:
    """删除后无法证明完整回滚时必须 service-fatal，且绝不继续扫描。"""
    module = _worker_module()
    world_spec = _worker_world_spec()
    context = multiprocessing.get_context("spawn")
    request_receiver, request_sender = context.Pipe(False)
    response_receiver, response_sender = context.Pipe(False)
    process = context.Process(
        target=_rollback_failure_spawn_entrypoint,
        args=(request_receiver, response_sender, world_spec),
        daemon=False,
    )
    try:
        process.start()
        request_receiver.close()
        response_sender.close()
        ready = module.receive_worker_startup_envelope(
            response_receiver,
            timeout_sec=15.0,
            expected_process_id=process.pid,
            expected_world_digest=world_spec.world_digest,
        )
        assert type(ready) is module.LidarWorkerReady

        establish = _scan_request(
            module,
            job_id=30,
            snapshots=_reconcile_snapshots(moving_x=3.2, include_static=True),
        )
        request_sender.send(establish)
        assert response_receiver.poll(15.0)
        established = response_receiver.recv()
        assert type(established) is module.PreparedLidarPayload

        failing = _scan_request(
            module,
            job_id=31,
            timestamp_ns=1_000_000_000,
            snapshots=(),
        )
        request_sender.send(failing)
        assert response_receiver.poll(15.0), "scene-state failure response timed out"
        try:
            failure = response_receiver.recv()
        except EOFError:
            pytest.fail("worker exited without returning scene_state_unknown")

        assert type(failure) is module.LidarScanFailure
        assert failure.job_id == failing.job_id
        assert failure.stable_error_code == "scene_state_unknown"
        assert "traceback" not in failure.bounded_detail.lower()
        assert "\n" not in failure.bounded_detail
        assert len(failure.bounded_detail.encode("utf-8")) <= 512

        rejected = _scan_request(
            module,
            job_id=32,
            timestamp_ns=1_100_000_000,
            snapshots=(),
        )
        try:
            request_sender.send(rejected)
        except (BrokenPipeError, EOFError, OSError):
            pass
        assert response_receiver.poll(5.0), "faulted worker kept response pipe open"
        with pytest.raises(EOFError):
            response_receiver.recv()
        process.join(5.0)
        assert process.is_alive() is False
        assert process.exitcode is not None and process.exitcode != 0
    finally:
        request_sender.close()
        response_receiver.close()
        if process.is_alive():
            process.kill()
            process.join(2.0)


def test_invalid_codec_payload_fails_once_and_allows_next_job() -> None:
    """codec 非 bytes 返回按单帧失败收口，镜像已知时下一 job 仍可扫描。"""
    module = _worker_module()
    live = module._bootstrap_live_worker(_worker_world_spec())
    assert type(live) is module._LiveWorkerBootstrap

    class InvalidPayloadCodec:
        def __init__(self) -> None:
            self.encode_calls = 0

        def encode(self, _message):
            self.encode_calls += 1
            return bytearray(b"not exact bytes")

    invalid_codec = InvalidPayloadCodec()
    live.codec = invalid_codec
    try:
        failed_request = _scan_request(
            module,
            job_id=40,
            snapshots=_initial_worker_snapshots(),
        )
        failure = module._process_scan_request(live, failed_request)

        assert type(failure) is module.LidarScanFailure
        assert failure.stable_error_code == "codec_failed"
        assert invalid_codec.encode_calls == 1
        assert live.scene_state_unknown is False

        live.codec = ProtoCodec()
        next_request = _scan_request(
            module,
            job_id=41,
            timestamp_ns=1_000_000_000,
            snapshots=_initial_worker_snapshots(),
        )
        prepared = module._process_scan_request(live, next_request)
        assert type(prepared) is module.PreparedLidarPayload
        assert prepared.job_id == next_request.job_id
    finally:
        module._disconnect_direct_client(live.client_id)


def test_pointcloud_failure_is_typed_and_allows_next_job() -> None:
    """native raycast 之外的扫描构造错误只降级当前帧，并保留已知镜像。"""
    module = _worker_module()
    live = module._bootstrap_live_worker(_worker_world_spec())
    assert type(live) is module._LiveWorkerBootstrap
    original_front = live.front_scanner

    class PointcloudFailureScanner:
        def _scan_frozen(self, *_args):
            raise RuntimeError("forced pointcloud construction failure")

    live.front_scanner = PointcloudFailureScanner()
    try:
        failed_request = _scan_request(
            module,
            job_id=50,
            snapshots=_initial_worker_snapshots(),
        )
        failure = module._process_scan_request(live, failed_request)

        assert type(failure) is module.LidarScanFailure
        assert failure.stable_error_code == "pointcloud_failed"
        assert live.scene_state_unknown is False

        live.front_scanner = original_front
        next_request = _scan_request(
            module,
            job_id=51,
            timestamp_ns=1_000_000_000,
            snapshots=_initial_worker_snapshots(),
        )
        prepared = module._process_scan_request(live, next_request)
        assert type(prepared) is module.PreparedLidarPayload
        assert prepared.job_id == next_request.job_id
    finally:
        module._disconnect_direct_client(live.client_id)


def test_worker_world_digest_is_canonical(monkeypatch: pytest.MonkeyPatch) -> None:
    """摘要忽略 mapping key 插入顺序，但保留列表顺序与内容语义。"""
    module = _worker_module()
    _config, document = _world_inputs()

    digest = module.world_digest_for_document(document)
    original_document_to_mapping = module.document_to_mapping

    def reverse_mapping_keys(value):
        if type(value) is dict:
            return {
                key: reverse_mapping_keys(value[key])
                for key in reversed(tuple(value))
            }
        if type(value) is list:
            return [reverse_mapping_keys(item) for item in value]
        return value

    def reordered_document_to_mapping(candidate):
        return reverse_mapping_keys(original_document_to_mapping(candidate))

    monkeypatch.setattr(module, "document_to_mapping", reordered_document_to_mapping)
    assert digest == module.world_digest_for_document(document)

    changed = SceneDocument.from_runtime(
        "df_back",
        TerrainSelection("slope", slope_deg=5.0),
        (),
        SensorDocument.default().mounts,
    )
    assert digest != module.world_digest_for_document(changed)
    with pytest.raises((TypeError, ValueError)):
        module.world_digest_for_document(object())

    object.__setattr__(document.terrain, "slope_deg", float("nan"))
    with pytest.raises((TypeError, ValueError)):
        module.world_digest_for_document(document)


def test_worker_ready_follows_full_front_rear_preflight() -> None:
    """真实 spawn 只在双 2880 射线和两次编码完成后才返回 Ready。"""
    module = _worker_module()
    handle = module.start_lidar_worker(_worker_world_spec(), startup_timeout_sec=15.0)
    try:
        ready = handle.ready
        assert type(ready) is module.LidarWorkerReady
        assert ready.prewarmed_topics == ("lidar_front", "lidar_rear")
        assert tuple(topic for topic, _digest in ready.prewarm_payload_sha256_by_topic) == ready.prewarmed_topics
        assert all(len(digest) == 64 for _topic, digest in ready.prewarm_payload_sha256_by_topic)
        assert ready.prewarm_max_scan_wall_duration_ns >= 0
        assert handle.process.daemon is False
    finally:
        handle.close()


@pytest.mark.parametrize("phase", ("front_preflight", "rear_preflight"))
def test_worker_preflight_failure_never_emits_ready(phase: str) -> None:
    """同进程真实 DIRECT bootstrap 的预热失败只能产生精确失败信封。"""
    module = _worker_module()

    envelope = module._bootstrap_worker(_worker_world_spec(), forced_failure_phase=phase)

    assert type(envelope) is module.LidarWorkerStartupFailure
    assert envelope.phase == phase
    assert envelope.stable_error_code == "worker_preflight_failed"
    assert "traceback" not in envelope.bounded_detail.lower()


@pytest.mark.parametrize("phase", ("front_preflight", "rear_preflight"))
def test_spawned_preflight_failure_is_exact_and_leaves_no_child(phase: str) -> None:
    """真实 spawn 的预热失败只发送 Failure，并以非零状态完整退出。"""
    module = _worker_module()
    world_spec = _worker_world_spec()
    context = multiprocessing.get_context("spawn")
    request_receiver, request_sender = context.Pipe(False)
    response_receiver, response_sender = context.Pipe(False)
    process = context.Process(
        target=_forced_preflight_spawn_entrypoint,
        args=(request_receiver, response_sender, world_spec, phase),
        daemon=False,
    )
    try:
        process.start()
        request_receiver.close()
        response_sender.close()
        with pytest.raises(module.LidarWorkerStartupError) as captured:
            module.receive_worker_startup_envelope(
                response_receiver,
                timeout_sec=15.0,
                expected_process_id=process.pid,
                expected_world_digest=world_spec.world_digest,
            )
        assert captured.value.stable_error_code == "worker_preflight_failed"
        assert captured.value.startup_failure is not None
        assert captured.value.startup_failure.phase == phase
        assert captured.value.ready is None
        with pytest.raises(EOFError):
            response_receiver.recv()
        request_sender.close()
        process.join(5.0)
        assert process.is_alive() is False
        assert process.exitcode is not None and process.exitcode != 0
    finally:
        request_sender.close()
        response_receiver.close()
        if process.is_alive():
            process.terminate()
            process.join(2.0)


def test_production_entrypoint_reports_world_build_failure_then_eof() -> None:
    """正式 child 入口必须把篡改 world spec 归为 world_build 并非零退出。"""
    module = _worker_module()
    world_spec = _worker_world_spec()
    expected_digest = world_spec.world_digest
    object.__setattr__(world_spec, "world_digest", "A" * 64)
    context = multiprocessing.get_context("spawn")
    request_receiver, request_sender = context.Pipe(False)
    response_receiver, response_sender = context.Pipe(False)
    process = context.Process(
        target=module.lidar_worker_entrypoint,
        args=(request_receiver, response_sender, world_spec),
        daemon=False,
    )
    try:
        process.start()
        request_receiver.close()
        response_sender.close()
        with pytest.raises(module.LidarWorkerStartupError) as captured:
            module.receive_worker_startup_envelope(
                response_receiver,
                timeout_sec=15.0,
                expected_process_id=process.pid,
                expected_world_digest=expected_digest,
            )
        assert captured.value.stable_error_code == "worker_preflight_failed"
        assert captured.value.startup_failure is not None
        assert captured.value.startup_failure.phase == "world_build"
        assert captured.value.ready is None
        with pytest.raises(EOFError):
            response_receiver.recv()
        request_sender.close()
        process.join(5.0)
        assert process.is_alive() is False
        assert process.exitcode is not None and process.exitcode != 0
    finally:
        request_sender.close()
        response_receiver.close()
        if process.is_alive():
            process.kill()
            process.join(2.0)


def test_spawned_worker_closes_direct_client_and_process_cleanly() -> None:
    """父端关闭自己的请求 pipe 后，worker 必须退出且不遗留 DIRECT client。"""
    module = _worker_module()
    handle = module.start_lidar_worker(_worker_world_spec(), startup_timeout_sec=15.0)

    stopped = handle.close()

    stopped_type = getattr(module, "LidarWorkerStopped", None)
    assert stopped_type is not None, "normal close requires a typed Stopped ACK"
    assert type(stopped) is stopped_type
    assert stopped.process_id == handle.ready.process_id
    assert handle.process.exitcode == 0
    assert handle.process.is_alive() is False


def test_startup_receiver_preserves_failure_and_never_returns_ready() -> None:
    """合法失败信封必须原样归因，不能同时伪造 Ready。"""
    module = _worker_module()
    _receiver, sender = multiprocessing.get_context("spawn").Pipe(False)
    failure = module.LidarWorkerStartupFailure(
        1,
        42,
        "front_preflight",
        "worker_preflight_failed",
        "front preflight failed",
    )
    try:
        sender.send(failure)
        sender.close()
        with pytest.raises(module.LidarWorkerStartupError) as captured:
            module.receive_worker_startup_envelope(
                _receiver,
                timeout_sec=0.1,
                expected_process_id=42,
            )
        assert captured.value.stable_error_code == "worker_preflight_failed"
        assert captured.value.startup_failure == failure
        assert captured.value.ready is None
    finally:
        _receiver.close()


def test_startup_receiver_maps_eof_and_timeout_to_worker_exited() -> None:
    """没有合法信封的 EOF 或超时不能伪造具体 preflight phase。"""
    module = _worker_module()
    receiver, sender = multiprocessing.get_context("spawn").Pipe(False)
    sender.close()
    try:
        with pytest.raises(module.LidarWorkerStartupError) as eof_error:
            module.receive_worker_startup_envelope(
                receiver,
                timeout_sec=0.1,
                expected_process_id=42,
            )
        assert eof_error.value.stable_error_code == "worker_exited"
        assert eof_error.value.startup_failure is None
        assert eof_error.value.ready is None
    finally:
        receiver.close()

    timeout_receiver, timeout_sender = multiprocessing.get_context("spawn").Pipe(False)
    try:
        with pytest.raises(module.LidarWorkerStartupError) as timeout_error:
            module.receive_worker_startup_envelope(
                timeout_receiver,
                timeout_sec=0.001,
                expected_process_id=42,
            )
        assert timeout_error.value.stable_error_code == "worker_exited"
        assert timeout_error.value.startup_failure is None
    finally:
        timeout_sender.close()
        timeout_receiver.close()


def test_parent_process_start_failure_maps_to_worker_start_failed() -> None:
    """父端同步启动错误没有 child envelope，错误码固定为 worker_start_failed。"""
    module = _worker_module()

    error = module.startup_error_from_process_start(RuntimeError("cannot start"))

    assert type(error) is module.LidarWorkerStartupError
    assert error.stable_error_code == "worker_start_failed"
    assert error.startup_failure is None
    assert error.ready is None


def test_startup_receiver_revalidates_pickled_envelope_and_ready_digest() -> None:
    """父端必须复验 pickle 值，且 Ready digest 必须逐字匹配父 world spec。"""
    module = _worker_module()
    _receiver, sender = multiprocessing.get_context("spawn").Pipe(False)
    ready = module.LidarWorkerReady(
        1,
        42,
        "1" * 64,
        ("lidar_front", "lidar_rear"),
        (("lidar_front", "2" * 64), ("lidar_rear", "3" * 64)),
        1,
    )
    object.__setattr__(ready, "world_digest", "A" * 64)
    try:
        sender.send(ready)
        sender.close()
        with pytest.raises(module.LidarWorkerStartupError) as malformed_error:
            module.receive_worker_startup_envelope(
                _receiver,
                timeout_sec=0.1,
                expected_process_id=42,
            )
        assert malformed_error.value.stable_error_code == "worker_exited"
    finally:
        _receiver.close()

    digest_receiver, digest_sender = multiprocessing.get_context("spawn").Pipe(False)
    valid_ready = module.LidarWorkerReady(
        1,
        42,
        "1" * 64,
        ("lidar_front", "lidar_rear"),
        (("lidar_front", "2" * 64), ("lidar_rear", "3" * 64)),
        1,
    )
    try:
        digest_sender.send(valid_ready)
        digest_sender.close()
        with pytest.raises(module.LidarWorkerStartupError) as mismatch_error:
            module.receive_worker_startup_envelope(
                digest_receiver,
                timeout_sec=0.1,
                expected_process_id=42,
                expected_world_digest="4" * 64,
            )
        assert mismatch_error.value.stable_error_code == "worker_exited"
    finally:
        digest_receiver.close()


@pytest.mark.parametrize("envelope_kind", ("ready", "failure"))
def test_startup_receiver_rejects_wrong_process_identity(envelope_kind: str) -> None:
    """Ready 和 StartupFailure 都必须来自本次 spawn 的精确 PID。"""
    module = _worker_module()
    receiver, sender = multiprocessing.get_context("spawn").Pipe(False)
    if envelope_kind == "ready":
        envelope = module.LidarWorkerReady(
            1,
            41,
            "1" * 64,
            ("lidar_front", "lidar_rear"),
            (("lidar_front", "2" * 64), ("lidar_rear", "3" * 64)),
            1,
        )
    else:
        envelope = module.LidarWorkerStartupFailure(
            1,
            41,
            "world_build",
            "worker_preflight_failed",
            "failed",
        )
    try:
        sender.send(envelope)
        sender.close()
        with pytest.raises(module.LidarWorkerStartupError) as mismatch:
            module.receive_worker_startup_envelope(
                receiver,
                timeout_sec=0.1,
                expected_process_id=42,
            )
        assert mismatch.value.stable_error_code == "worker_exited"
        assert mismatch.value.startup_failure is None
    finally:
        receiver.close()


def test_startup_receiver_requires_exact_int_expected_process_identity() -> None:
    """父端期望 PID 本身也必须是 exact built-in int。"""
    module = _worker_module()
    receiver, sender = multiprocessing.get_context("spawn").Pipe(False)
    try:
        with pytest.raises((TypeError, ValueError)):
            module.receive_worker_startup_envelope(
                receiver,
                timeout_sec=0.1,
                expected_process_id=_IntegerEnum.ONE,
            )
    finally:
        sender.close()
        receiver.close()


def test_failed_start_cleanup_kills_owned_child_that_ignores_sigterm() -> None:
    """失败启动回收必须有界升级到 kill，且返回时 child 已经退出。"""
    module = _worker_module()
    context = multiprocessing.get_context("spawn")
    ready_receiver, ready_sender = context.Pipe(False)
    process = context.Process(
        target=_ignore_sigterm_child,
        args=(ready_sender,),
        daemon=False,
    )
    request_receiver, request_sender = context.Pipe(False)
    response_receiver, response_sender = context.Pipe(False)
    try:
        process.start()
        ready_sender.close()
        assert ready_receiver.poll(5.0)
        assert ready_receiver.recv() == process.pid
        request_receiver.close()
        response_sender.close()

        module._close_failed_start(process, request_sender, response_receiver)

        assert process.is_alive() is False
        assert process.exitcode is not None
    finally:
        request_sender.close()
        response_receiver.close()
        ready_receiver.close()
        if process.is_alive():
            process.kill()
            process.join(2.0)


def test_handle_close_reaps_owned_child_before_reporting_timeout() -> None:
    """正常关闭超时也必须先彻底回收自有 child，再向调用方报告异常。"""
    module = _worker_module()
    context = multiprocessing.get_context("spawn")
    ready_receiver, ready_sender = context.Pipe(False)
    process = context.Process(
        target=_ignore_sigterm_child,
        args=(ready_sender,),
        daemon=False,
    )
    request_receiver, request_sender = context.Pipe(False)
    response_receiver, response_sender = context.Pipe(False)
    ready = module.LidarWorkerReady(
        1,
        42,
        "1" * 64,
        ("lidar_front", "lidar_rear"),
        (("lidar_front", "2" * 64), ("lidar_rear", "3" * 64)),
        1,
    )
    try:
        process.start()
        ready_sender.close()
        assert ready_receiver.poll(5.0)
        assert ready_receiver.recv() == process.pid
        request_receiver.close()
        response_sender.close()
        handle = module.LidarWorkerHandle(
            process,
            request_sender,
            response_receiver,
            ready,
        )

        with pytest.raises(RuntimeError, match="did not exit"):
            handle.close(timeout_sec=0.01)

        assert process.is_alive() is False
        assert process.exitcode is not None
    finally:
        request_sender.close()
        response_receiver.close()
        ready_receiver.close()
        if process.is_alive():
            process.kill()
            process.join(2.0)


def test_close_terminates_only_owned_child_after_join_timeout() -> None:
    """Stop 无 ACK 时只终结 handle 的真实 child，不触碰旁观进程。"""
    module = _worker_module()

    class RecordingSender:
        def __init__(self) -> None:
            self.sent: list[object] = []
            self.close_count = 0

        def send(self, value: object) -> None:
            self.sent.append(value)

        def close(self) -> None:
            self.close_count += 1

    class NoAckReceiver:
        def __init__(self) -> None:
            self.poll_timeouts: list[float] = []
            self.recv_count = 0
            self.close_count = 0

        def poll(self, timeout_sec: float) -> bool:
            self.poll_timeouts.append(timeout_sec)
            return False

        def recv(self) -> object:
            self.recv_count += 1
            raise AssertionError("normal close must not recv without a ready ACK")

        def close(self) -> None:
            self.close_count += 1

    context = multiprocessing.get_context("spawn")
    owned_ready_receiver, owned_ready_sender = context.Pipe(False)
    bystander_ready_receiver, bystander_ready_sender = context.Pipe(False)
    owned = context.Process(
        target=_ignore_sigterm_child,
        args=(owned_ready_sender,),
        daemon=False,
    )
    bystander = context.Process(
        target=_ignore_sigterm_child,
        args=(bystander_ready_sender,),
        daemon=False,
    )
    sender = RecordingSender()
    receiver = NoAckReceiver()
    try:
        owned.start()
        bystander.start()
        owned_ready_sender.close()
        bystander_ready_sender.close()
        assert owned_ready_receiver.poll(5.0)
        assert owned_ready_receiver.recv() == owned.pid
        assert bystander_ready_receiver.poll(5.0)
        assert bystander_ready_receiver.recv() == bystander.pid
        ready = module.LidarWorkerReady(
            1,
            owned.pid,
            "1" * 64,
            ("lidar_front", "lidar_rear"),
            (("lidar_front", "2" * 64), ("lidar_rear", "3" * 64)),
            1,
        )
        handle = module.LidarWorkerHandle(owned, sender, receiver, ready)

        with pytest.raises(RuntimeError, match="shutdown|ACK|exit"):
            handle.close(timeout_sec=0.01)

        stop_type = getattr(module, "LidarWorkerStop", None)
        assert stop_type is not None, "normal close requires a typed Stop request"
        assert len(sender.sent) == 1
        assert type(sender.sent[0]) is stop_type
        assert sender.sent[0].process_id == owned.pid
        assert owned.is_alive() is False
        assert bystander.is_alive() is True
        assert receiver.recv_count == 0
    finally:
        owned_ready_receiver.close()
        bystander_ready_receiver.close()
        if owned.is_alive():
            owned.kill()
            owned.join(2.0)
        if bystander.is_alive():
            bystander.kill()
            bystander.join(2.0)


def test_handle_normal_close_rejects_already_exited_worker_without_ack() -> None:
    """child 提前退出不能绕过 Stop/Stopped 合同冒充正常关闭。"""
    module = _worker_module()

    class DeadProcess:
        def join(self, _timeout_sec: float) -> None:
            return None

        def is_alive(self) -> bool:
            return False

        def terminate(self) -> None:  # pragma: no cover - 调用即测试失败
            raise AssertionError("already exited process must not be terminated")

        def kill(self) -> None:  # pragma: no cover - 调用即测试失败
            raise AssertionError("already exited process must not be killed")

    class Sender:
        def __init__(self) -> None:
            self.sent: list[object] = []

        def send(self, value: object) -> None:
            self.sent.append(value)

        def close(self) -> None:
            return None

    class Receiver:
        def poll(self, _timeout_sec: float) -> bool:
            return False

        def recv(self) -> object:  # pragma: no cover - poll False 后不得调用
            raise AssertionError("recv must not run without ACK readiness")

        def close(self) -> None:
            return None

    ready = module.LidarWorkerReady(
        1,
        42,
        "1" * 64,
        ("lidar_front", "lidar_rear"),
        (("lidar_front", "2" * 64), ("lidar_rear", "3" * 64)),
        1,
    )
    sender = Sender()
    handle = module.LidarWorkerHandle(DeadProcess(), sender, Receiver(), ready)

    with pytest.raises(RuntimeError, match="normal shutdown"):
        handle.close(timeout_sec=0.01)

    assert len(sender.sent) == 1
    assert type(sender.sent[0]) is module.LidarWorkerStop


def test_exception_detail_is_single_line_and_bounded_by_utf8_bytes() -> None:
    """多字节异常类名也必须生成至多 512 bytes 的合法单行 detail。"""
    module = _worker_module()
    error_type = type("多字节\n异常" * 100, (RuntimeError,), {})

    detail = module._bounded_exception_detail(
        "front\npreflight",
        error_type("ignored exception payload"),
    )

    assert "\n" not in detail
    assert "\r" not in detail
    assert len(detail.encode("utf-8")) <= 512
    envelope = module.LidarWorkerStartupFailure(
        1,
        42,
        "front_preflight",
        "worker_preflight_failed",
        detail,
    )
    assert envelope.bounded_detail == detail
