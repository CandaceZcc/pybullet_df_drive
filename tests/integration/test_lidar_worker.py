# LiDAR worker 集成测试：覆盖父端有界调度、真实 spawn、冻结帧和故障收口。
from __future__ import annotations

import ast
from dataclasses import fields, is_dataclass, replace
from enum import IntEnum
from functools import lru_cache
import hashlib
import importlib
import inspect
import math
import multiprocessing
import os
from pathlib import Path
from multiprocessing.connection import wait
from multiprocessing.reduction import ForkingPickler
from pprint import pformat
import signal
from statistics import median
import struct
import time
from threading import Event, Thread
from types import SimpleNamespace

import pybullet as p
import pytest

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import build_world_from_scene_document
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.dashboard_snapshot import LidarTopViewFrame
from slope_sim.interfaces.models import LidarPointCloud
from slope_sim.lidar_pointcloud import LidarScanResult, MultiLineLidar, Stage4LidarProfile
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


_STAGE4_ORACLE_PATTERN_VERSION = "livox-mid360-800000-v1"
_STAGE4_ORACLE_PHASE_DOMAIN = b"pybullet-df-drive/mid360-phase/v1\0"
_STAGE4_ORACLE_PATTERN_ROWS = 800_000
_STAGE4_ORACLE_FIRING_SLOTS = 5_760
_STAGE4_ORACLE_SCAN_PERIOD_NS = 100_000_000
_STAGE4_ORACLE_MIN_RANGE_M = 0.10
_STAGE4_ORACLE_MAX_RANGE_M = 30.0
_STAGE4_ORACLE_RANGE_REL_TOLERANCE = 1e-6
_STAGE4_ORACLE_RANGE_ABS_TOLERANCE_M = 1e-5
_STAGE4_ORACLE_PATTERN_PATH = (
    Path(__file__).resolve().parents[2] / "slope_sim" / "assets" / "mid360_pattern.bin"
)
_STAGE4_ORACLE_POINT_SEMANTICS = {
    "unknown": (0, 80),
    "terrain": (1, 100),
    "static_obstacle": (2, 160),
    "moving_obstacle": (3, 200),
}


class _IntegerEnum(IntEnum):
    ONE = 1


def _stage4_shard_entrypoint_with_gc_probe(
    request_receiver: object,
    response_sender: object,
    spec: object,
    probe_sender: object,
) -> None:
    """spawn-safe 测试包装器：仅在正式 freeze 后禁用 child 热路径 GC。"""
    import gc

    events: list[tuple[str, int, int | None, int | None, int | None]] = []
    original_freeze = gc.freeze
    was_enabled = gc.isenabled()
    freeze_calls = 0
    hot_loop_started = False
    thread_count_calls: list[int] = []

    def callback(phase: str, info: dict[str, object]) -> None:
        if not hot_loop_started:
            return
        events.append(
            (
                phase,
                time.monotonic_ns(),
                info.get("generation"),
                info.get("collected"),
                info.get("uncollectable"),
            )
        )

    def freeze_then_disable() -> None:
        nonlocal freeze_calls, hot_loop_started
        original_freeze()
        freeze_calls += 1
        hot_loop_started = True
        gc.disable()

    gc.freeze = freeze_then_disable
    gc.callbacks.append(callback)
    worker_module = None
    original_thread_count = None
    try:
        import slope_sim.lidar_worker as worker_module

        original_thread_count = worker_module._stage4_realtime_shard_thread_count

        def combined_gc_thread_count(shard_id: int) -> int:
            thread_count = (2, 2)[shard_id]
            thread_count_calls.append(thread_count)
            return thread_count

        worker_module._stage4_realtime_shard_thread_count = combined_gc_thread_count

        worker_module.stage4_shard_entrypoint(
            request_receiver,
            response_sender,
            spec,
        )
    finally:
        gc.callbacks.remove(callback)
        gc_enabled_after_freeze = gc.isenabled()
        if original_thread_count is not None:
            worker_module._stage4_realtime_shard_thread_count = original_thread_count
        gc.freeze = original_freeze
        if was_enabled:
            gc.enable()
        else:
            gc.disable()
        probe_sender.send(
            {
                "freeze_calls": freeze_calls,
                "gc_enabled_after_freeze": gc_enabled_after_freeze,
                "hot_loop_gc_events": tuple(events),
                "hot_loop_thread_counts": tuple(thread_count_calls),
            }
        )
        probe_sender.close()


def _worker_module():
    """延迟导入待实现模块，让 RED 停在测试函数断言而非 collection。"""
    try:
        return importlib.import_module("slope_sim.lidar_worker")
    except ModuleNotFoundError:
        return SimpleNamespace()


@lru_cache(maxsize=1)
def _stage4_oracle_pattern_bytes() -> bytes:
    """首次使用时直接读取冻结角度资产，不经过生产 pattern loader。"""
    raw = _STAGE4_ORACLE_PATTERN_PATH.read_bytes()
    assert len(raw) == _STAGE4_ORACLE_PATTERN_ROWS * struct.calcsize("<dd")
    return raw


@lru_cache(maxsize=16)
def _stage4_oracle_phase_base(world_generation: int) -> int:
    """按测试侧冻结 SHA 输入计算一个 world 的 phase base。"""
    digest = hashlib.sha256(
        _STAGE4_ORACLE_PHASE_DOMAIN
        + _STAGE4_ORACLE_PATTERN_VERSION.encode("ascii")
        + b"\0"
        + world_generation.to_bytes(8, "big", signed=False)
    ).digest()
    return int.from_bytes(digest, "big") % _STAGE4_ORACLE_PATTERN_ROWS


def _stage4_oracle_row_index(
    world_generation: int,
    sequence: int,
    global_slot: int,
) -> int:
    """按测试侧冻结 domain 计算 progressive pattern row。"""
    assert type(world_generation) is int and 0 < world_generation < 1 << 64
    assert type(sequence) is int and 0 <= sequence < 1 << 64
    assert type(global_slot) is int and 0 <= global_slot < _STAGE4_ORACLE_FIRING_SLOTS
    return (
        _stage4_oracle_phase_base(world_generation)
        + sequence * _STAGE4_ORACLE_FIRING_SLOTS
        + global_slot
    ) % _STAGE4_ORACLE_PATTERN_ROWS


def _stage4_oracle_firing(
    world_generation: int,
    sequence: int,
    global_slot: int,
) -> tuple[int, int, tuple[float, float, float]]:
    """从冻结 row 读取角度并独立计算 offset、line 与项目坐标方向。"""
    row = _stage4_oracle_row_index(world_generation, sequence, global_slot)
    azimuth_deg, zenith_deg = struct.unpack_from(
        "<dd",
        _stage4_oracle_pattern_bytes(),
        row * struct.calcsize("<dd"),
    )
    azimuth = math.radians(azimuth_deg)
    zenith = math.radians(zenith_deg)
    sin_zenith = math.sin(zenith)
    return (
        global_slot * _STAGE4_ORACLE_SCAN_PERIOD_NS
        // _STAGE4_ORACLE_FIRING_SLOTS,
        row % 4,
        (
            sin_zenith * math.cos(azimuth),
            sin_zenith * math.sin(azimuth),
            math.cos(zenith),
        ),
    )


def _assert_stage4_progressive_point_oracle(
    point_fields: tuple[tuple[object, ...], ...],
    *,
    world_generation: int,
    sequence: int,
) -> None:
    """独立按 progressive firing 身份核对已按全局 slot 排序的点字段。"""
    offset_to_slot = {
        slot * _STAGE4_ORACLE_SCAN_PERIOD_NS // _STAGE4_ORACLE_FIRING_SLOTS: slot
        for slot in range(_STAGE4_ORACLE_FIRING_SLOTS)
    }
    assert len(offset_to_slot) == _STAGE4_ORACLE_FIRING_SLOTS
    previous_slot = -1
    valid_semantics = set(_STAGE4_ORACLE_POINT_SEMANTICS.values())
    for offset_time_ns, x, y, z, reflectivity, tag, line in point_fields:
        assert offset_time_ns in offset_to_slot
        global_slot = offset_to_slot[offset_time_ns]
        assert global_slot > previous_slot
        previous_slot = global_slot
        expected_offset, expected_line, expected_direction = _stage4_oracle_firing(
            world_generation,
            sequence,
            global_slot,
        )
        assert offset_time_ns == expected_offset
        assert line == expected_line
        distance = math.hypot(x, y, z)
        assert distance > 0.0
        observed_direction = (x / distance, y / distance, z / distance)
        assert observed_direction == pytest.approx(
            expected_direction,
            rel=1e-5,
            abs=1e-5,
        )
        assert (tag, reflectivity) in valid_semantics


def test_stage4_progressive_oracle_does_not_call_production_pattern_helpers(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """测试 oracle 必须独立读取冻结资产，不能复用被测 pattern 实现。"""
    lidar_pointcloud = importlib.import_module("slope_sim.lidar_pointcloud")

    def reject_production_helper(*_args: object) -> None:
        raise AssertionError("progressive oracle called a production pattern helper")

    for name in (
        "mid360_offset_time_ns",
        "mid360_line_for_slot",
        "mid360_direction_for_slot",
    ):
        monkeypatch.setattr(lidar_pointcloud, name, reject_production_helper)
    azimuth = math.radians(302.06)
    zenith = math.radians(55.504)
    direction = (
        math.sin(zenith) * math.cos(azimuth),
        math.sin(zenith) * math.sin(azimuth),
        math.cos(zenith),
    )

    _assert_stage4_progressive_point_oracle(
        ((0, *direction, 100, 1, 1),),
        world_generation=1,
        sequence=0,
    )


@pytest.mark.parametrize(
    ("world_generation", "sequence", "global_slot", "expected_row"),
    (
        (1, 0, 0, 17_365),
        (1, 0, 5_759, 23_124),
        (1, 1, 0, 23_125),
        (2, 0, 0, 795_583),
        (2, 0, 5_759, 1_342),
    ),
)
def test_stage4_progressive_oracle_matches_frozen_phase_vectors(
    world_generation: int,
    sequence: int,
    global_slot: int,
    expected_row: int,
) -> None:
    """独立 SHA 公式必须覆盖多 world、sequence 和 800000 row 回绕。"""
    assert (
        _stage4_oracle_row_index(world_generation, sequence, global_slot)
        == expected_row
    )


def test_lidar_worker_request_accepts_only_the_stage4_center_lidar_identity() -> None:
    """阶段四 worker 必须显式接受唯一中心 lidar_link，而非复用前后雷达身份。"""
    module = _worker_module()
    request_type = getattr(module, "LidarScanRequest", None)
    assert request_type is not None, "LidarScanRequest must exist"

    request = request_type(
        1,
        1,
        1,
        0,
        0,
        "lidar_link",
        "lidar_link",
        1,
        100_000_000,
        Pose((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
        None,
        (),
    )

    assert (request.topic, request.frame_id, request.lidar_id) == (
        "lidar_link",
        "lidar_link",
        1,
    )
    with pytest.raises(ValueError, match="frame_id"):
        request_type(
            1,
            2,
            1,
            0,
            0,
            "lidar_link",
            "lidar_front",
            1,
            100_000_000,
            Pose((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
            None,
            (),
        )


def test_stage4_worker_profile_prewarm_is_limited_to_the_center_lidar() -> None:
    """阶段四 profile 必须只预热中心 5,760 射线 LiDAR，不能启动旧双雷达。"""
    module = _worker_module()
    world_spec = _worker_world_spec()
    stage4_spec = module.LidarWorkerWorldSpec(
        world_spec.protocol_version,
        world_spec.experiment_config,
        world_spec.scene_document,
        world_spec.world_digest,
        "stage4",
    )

    result = module._bootstrap_worker(stage4_spec)

    assert type(result) is module.LidarWorkerReady
    assert result.prewarmed_topics == ("lidar_link",)
    assert tuple(topic for topic, _digest in result.prewarm_payload_sha256_by_topic) == (
        "lidar_link",
    )
    assert result.prewarm_max_scan_wall_duration_ns > 0


def test_stage4_coordinator_uses_exact_two_realtime_shards() -> None:
    """Stage4 协调者只能使用冻结的 even/odd 私有 assignment。"""
    module = _worker_module()

    assert module._stage4_realtime_shard_assignments() == (
        (0, 5_760, 2, 2_880),
        (1, 5_760, 2, 2_880),
    )


def test_stage4_realtime_shard_thread_allocation_matches_frozen_ranges() -> None:
    """两个交错 shard 必须各使用两个 Bullet ray 线程。"""
    module = _worker_module()

    allocations = tuple(
        (shard_id, assignment, module._stage4_realtime_shard_thread_count(shard_id))
        for shard_id, assignment in enumerate(module._stage4_realtime_shard_assignments())
    )

    assert allocations == ((0, (0, 5_760, 2, 2_880), 2), (1, (1, 5_760, 2, 2_880), 2))
    assert sum(thread_count for _shard_id, _assignment, thread_count in allocations) == 4
    assert sum(thread_count for _shard_id, _assignment, thread_count in allocations) <= 8


def test_stage4_interleaved_shard_assignment_is_an_exact_even_odd_partition() -> None:
    """两个 Stage4 shard 必须精确、互斥地覆盖全体 even/odd global ray index。"""
    module = _worker_module()

    assignments = module._stage4_realtime_shard_assignments()

    assert assignments == ((0, 5_760, 2, 2_880), (1, 5_760, 2, 2_880))
    covered = tuple(
        index
        for first, stop, stride, count in assignments
        for index in range(first, stop, stride)
    )
    assert len(covered) == 5_760
    assert len(set(covered)) == 5_760
    assert tuple(sorted(covered)) == tuple(range(5_760))
    assert tuple(range(*assignments[0][:3])) == tuple(range(0, 5_760, 2))
    assert tuple(range(*assignments[1][:3])) == tuple(range(1, 5_760, 2))
    assert tuple(len(range(first, stop, stride)) for first, stop, stride, _count in assignments) == (2_880, 2_880)


def test_stage4_shard_prewarm_and_hot_scans_use_two_threads(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """每个 shard 只直接生成自己的 readonly C-order firing slots 并恢复 global index。"""
    import numpy as np

    module = _worker_module()
    assignments = ((0, 5_760, 2, 2_880), (1, 5_760, 2, 2_880))
    assert tuple(module._stage4_realtime_shard_thread_count(shard_id) for shard_id in (0, 1)) == (2, 2)

    calls: list[tuple[tuple[int, ...], int, int, object, object, int]] = []
    point_calls: list[tuple[int, int, int]] = []
    hit = SimpleNamespace(hit_position=(1.0, 2.0, 3.0))

    class Scanner:
        def _stage4_world_rays(self, _mount: object):
            raise AssertionError("a shard must not construct the full 5,760-ray table")

        def _stage4_world_rays_for_slots(
            self,
            _mount: object,
            *,
            pattern_version: str,
            world_generation: int,
            sequence: int,
            global_slots: object,
        ):
            assert pattern_version == "livox-mid360-800000-v1"
            slots = tuple(global_slots)
            starts = np.column_stack(
                (
                    np.asarray(slots, dtype=np.float64),
                    np.zeros(len(slots), dtype=np.float64),
                    np.ones(len(slots), dtype=np.float64),
                )
            )
            ends = starts + 10_000.0
            starts.setflags(write=False)
            ends.setflags(write=False)
            calls.append((slots, world_generation, sequence, starts, ends, -1))
            return starts, ends

        def _stage4_point_values_from_hit(
            self,
            global_slot: int,
            _hit: object,
            _point: object,
            *,
            pattern_version: str,
            world_generation: int,
            sequence: int,
        ):
            assert pattern_version == "livox-mid360-800000-v1"
            point_calls.append((global_slot, world_generation, sequence))
            return (global_slot, 1.0, 2.0, 3.0, 100, 1, global_slot % 4)

    class Backend:
        def _ray_test_indexed_hits_ndarray(self, starts, ends, *, collision_mask: int, num_threads: int):
            assert collision_mask == module.LIDAR_VISIBLE_GROUP
            assert starts.flags.c_contiguous and ends.flags.c_contiguous
            assert not starts.flags.writeable and not ends.flags.writeable
            slots, generation, sequence, original_starts, original_ends, _ = calls[-1]
            assert starts is original_starts and ends is original_ends
            calls[-1] = (slots, generation, sequence, starts, ends, num_threads)
            return ((0, hit), (1, hit))

        def inverse_transform_points_prevalidated(self, _mount: object, points: tuple[object, ...]):
            return ((1.0, 2.0, 3.0),) * len(points)

    descriptor = importlib.import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    live = SimpleNamespace(
        backend=Backend(),
        center_scanner=Scanner(),
        codec=module.V2ProtoCodec(descriptor),
    )
    monkeypatch.setattr(module, "_world_mount_pose", lambda _backend, _scanner: object())
    monkeypatch.setattr(module, "_reconcile_obstacles", lambda _live, _snapshots: None)
    for shard_id, (first, stop, stride, count) in enumerate(assignments):
        spec = SimpleNamespace(
            shard_id=shard_id,
            first=first,
            start=first,
            stop=stop,
            stride=stride,
            count=count,
            world_spec=SimpleNamespace(world_digest="0" * 64),
        )
        prewarm = module._stage4_shard_prewarm(live, spec)
        hot = module._stage4_shard_scan(
            live,
            SimpleNamespace(
                complete_obstacle_snapshots_without_body_ids=(),
                world_mount_pose=object(),
                job_id=1,
                lifecycle_generation=1,
                pause_epoch=0,
                topic="lidar_link",
                timestamp_ns=100_000_000,
                output_identity=SimpleNamespace(world_generation=9, sequence=11),
            ),
            spec,
        )
        assert tuple(index for index, _value in prewarm.values) == (first, first + stride)
        assert tuple(index for index, _value in hot.values) == (first, first + stride)

    assert tuple(thread_count for *_rest, thread_count in calls) == (2, 2, 2, 2)
    assert tuple(starts.shape for _slots, _world, _sequence, starts, _ends, _threads in calls) == (
        (2_880, 3), (2_880, 3), (2_880, 3), (2_880, 3)
    )
    assert tuple((world, sequence) for _slots, world, sequence, *_rest in calls) == (
        (1, 0), (9, 11), (1, 0), (9, 11)
    )
    assert tuple(slots for slots, *_rest in calls) == (
        tuple(range(0, 5_760, 2)),
        tuple(range(0, 5_760, 2)),
        tuple(range(1, 5_760, 2)),
        tuple(range(1, 5_760, 2)),
    )
    assert all(not starts.flags.writeable and not ends.flags.writeable for *_head, starts, ends, _threads in calls)
    assert point_calls == [
        (0, 1, 0), (2, 1, 0), (0, 9, 11), (2, 9, 11),
        (1, 1, 0), (3, 1, 0), (1, 9, 11), (3, 9, 11),
    ]


def test_stage4_interleaved_merge_restores_global_order_and_rejects_bad_coverage(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """反向稀疏流只做线性归并，逐点校验必须已在接收边界完成。"""
    module = _worker_module()
    values = tuple((index, float(index), 0.0, 0.1, 100, 1, 0) for index in range(8))

    with monkeypatch.context() as patch:
        patch.setattr(
            module,
            "_require_stage4_indexed_values",
            lambda *_args: (_ for _ in ()).throw(
                AssertionError("hot merge repeated receive-boundary validation")
            ),
        )
        merged = module._merge_stage4_shard_indexed_values(
            (
                (1, 1, 5_760, 2, 2_880, 2_880, ((1, values[1]), (5, values[5]), (7, values[7]))),
                (0, 0, 5_760, 2, 2_880, 2_880, ((0, values[0]), (6, values[6]))),
            )
        )

    assert merged == (values[0], values[1], values[5], values[6], values[7])
    merge_tree = ast.parse(inspect.getsource(module._merge_stage4_shard_indexed_values))
    assert not any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "sort"
        for node in ast.walk(merge_tree)
    )
    bad_raw_shards = (
        ((1, 1, 5_760, 2, 2_880, 2_880, ()), (1, 1, 5_760, 2, 2_880, 2_880, ())),
        ((1, 1, 5_760, 2, 2_880, 2_880, ()), (0, 2, 5_760, 2, 2_880, 2_880, ())),
        ((1, 1, 5_760, 2, 2_880, 2_879, ()), (0, 0, 5_760, 2, 2_880, 2_880, ())),
    )
    for raw_shards in bad_raw_shards:
        with pytest.raises(RuntimeError):
            module._merge_stage4_shard_indexed_values(raw_shards)


def test_stage4_shard_stop_validates_identity_disconnects_before_ack_and_exits_zero(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """shard 只能接收自己的 Stop，且已断开 DIRECT 后才发送精确 Stopped。"""
    import gc

    module = _worker_module()
    process_id = os.getpid()
    events: list[object] = []
    live = module._LiveWorkerBootstrap(
        7, None, SimpleNamespace(), None, None, object(), SimpleNamespace(), (), {}
    )
    spec = SimpleNamespace(shard_id=0, first=0, start=0, stop=5_760, stride=2, count=2_880, world_spec=SimpleNamespace(world_digest="0" * 64))

    class Receiver:
        def __init__(self, stop: object) -> None:
            self.stop = stop

        def recv(self) -> object:
            return self.stop

        def close(self) -> None:
            events.append("request_close")

    class Sender:
        def send(self, value: object) -> None:
            events.append(("send", value))

        def close(self) -> None:
            events.append("response_close")

    monkeypatch.setattr(module, "_bootstrap_live_worker", lambda _spec, **_kwargs: live)
    monkeypatch.setattr(module, "_stage4_shard_prewarm", lambda _live, _spec: object())
    monkeypatch.setattr(module, "_disconnect_direct_client", lambda client_id: events.append(("disconnect", client_id)))
    monkeypatch.setattr(gc, "freeze", lambda: None)
    monkeypatch.setattr(gc, "disable", lambda: None)

    with pytest.raises(ValueError, match="wrong process id"):
        module.stage4_shard_entrypoint(
            Receiver(module.LidarWorkerStop(1, process_id + 1)), Sender(), spec
        )
    assert not any(
        event == ("send", module._Stage4ShardStopped(0, process_id))
        for event in events
    )

    events.clear()
    module.stage4_shard_entrypoint(Receiver(module.LidarWorkerStop(1, process_id)), Sender(), spec)
    disconnect_index = events.index(("disconnect", 7))
    stopped_index = next(
        index
        for index, event in enumerate(events)
        if event == ("send", module._Stage4ShardStopped(0, process_id))
    )
    assert disconnect_index < stopped_index


def test_stage4_coordinator_stop_uses_shard_pids_and_rejects_wrong_shard_ack(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """coordinator Stop 必须逐 shard 定址并拒绝错误的 shard_id/PID ACK。"""
    module = _worker_module()
    coordinator_pid = os.getpid()
    shard_pids = (coordinator_pid + 10, coordinator_pid + 11)
    world_spec = SimpleNamespace(world_digest="0" * 64)
    assignments = module._stage4_realtime_shard_assignments()
    sent: list[tuple[int, object]] = []
    outer: list[object] = []

    class RequestReceiver:
        def recv(self) -> object:
            return module.LidarWorkerStop(1, coordinator_pid)

        def close(self) -> None:
            return None

    class Sender:
        def __init__(self, shard_id: int) -> None:
            self.shard_id = shard_id

        def send(self, value: object) -> None:
            sent.append((self.shard_id, value))

        def close(self) -> None:
            return None

    class Receiver:
        def __init__(self, shard_id: int) -> None:
            first, stop, stride, count = assignments[shard_id]
            self.values = [
                module._Stage4ShardReady(
                    shard_id, shard_pids[shard_id], first, stop, stride, count,
                    world_spec.world_digest,
                ),
                object(),
                module._Stage4ShardStopped(1 - shard_id, shard_pids[shard_id]),
            ]

        def recv(self) -> object:
            return self.values.pop(0)

        def poll(self, timeout: float) -> bool:
            assert 0.0 <= timeout <= 5.0
            return True

        def close(self) -> None:
            return None

    class OuterSender:
        def send(self, value: object) -> None:
            outer.append(value)

        def close(self) -> None:
            return None

    monkeypatch.setattr(module, "V2ProtoCodec", lambda _descriptor: object())
    monkeypatch.setattr(module, "load_v2_descriptor", lambda: object())
    monkeypatch.setattr(module, "_stage4_prewarm_payload_from_shards", lambda *_args: (b"x", 1))
    monkeypatch.setattr(module.gc, "collect", lambda: None)
    monkeypatch.setattr(module.gc, "freeze", lambda: None)
    monkeypatch.setattr(module.gc, "disable", lambda: None)

    with pytest.raises(RuntimeError, match="Stage4 shard.*Stopped"):
        module.stage4_coordinator_entrypoint(
            RequestReceiver(), OuterSender(), world_spec,
            (Sender(0), Sender(1)), (Receiver(0), Receiver(1)), shard_pids,
        )
    assert sent == [
        (0, module.LidarWorkerStop(1, shard_pids[0])),
        (1, module.LidarWorkerStop(1, shard_pids[1])),
    ]
    assert not any(type(value) is module.LidarWorkerStopped for value in outer)


def test_stage4_coordinator_bounds_shard_timeout_or_exception_to_one_whole_frame_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """超时只失败当前整帧；排空迟到结果后下一帧仍须保持身份同步。"""
    module = _worker_module()
    descriptor = importlib.import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points", b"\x01" * 16, descriptor.sha256, 1, 7
    )
    request = module.LidarScanRequest(
        1, 7, time.monotonic_ns(), 1, 0, "lidar_link", "lidar_link", 1,
        700_000_000, Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)), None, (), identity,
    )
    next_identity = replace(identity, sequence=8)
    next_request = module.LidarScanRequest(
        1, 8, time.monotonic_ns(), 1, 0, "lidar_link", "lidar_link", 1,
        800_000_000, Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)), None, (), next_identity,
    )
    world_spec = SimpleNamespace(world_digest="0" * 64)
    assignments = module._stage4_realtime_shard_assignments()
    shard_pids = (os.getpid() + 20, os.getpid() + 21)
    outer: list[object] = []
    merged: list[object] = []

    class RequestReceiver:
        def __init__(self) -> None:
            self.values = [request, next_request, module.LidarWorkerStop(1, os.getpid())]

        def recv(self) -> object:
            return self.values.pop(0)

        def close(self) -> None:
            return None

    class ShardSender:
        def send(self, _value: object) -> None:
            return None

        def close(self) -> None:
            return None

    class ShardReceiver:
        def __init__(self, shard_id: int) -> None:
            first, stop, stride, count = assignments[shard_id]
            self.values = [
                module._Stage4ShardReady(
                    shard_id, shard_pids[shard_id], first, stop, stride, count, world_spec.world_digest
                ),
                object(),
                module._Stage4ShardResult(
                    shard_id, shard_pids[shard_id], first, stop, stride, count, count,
                    request.job_id, request.lifecycle_generation, request.pause_epoch,
                    request.topic, request.timestamp_ns, (),
                ),
                module._Stage4ShardResult(
                    shard_id, shard_pids[shard_id], first, stop, stride, count, count,
                    next_request.job_id, next_request.lifecycle_generation,
                    next_request.pause_epoch, next_request.topic,
                    next_request.timestamp_ns, (),
                ),
                module._Stage4ShardStopped(shard_id, shard_pids[shard_id]),
            ]
            self.poll_count = 0
            self.shard_id = shard_id

        def poll(self, timeout: float) -> bool:
            assert timeout >= 0.0
            self.poll_count += 1
            # 第一帧严格走 100 ms timeout；迟到结果排空后下一帧与 Stop 均正常。
            return self.poll_count > 1 or self.shard_id == 1

        def recv(self) -> object:
            return self.values.pop(0)

        def close(self) -> None:
            return None

    class OuterSender:
        def send(self, value: object) -> None:
            outer.append(value)

        def close(self) -> None:
            return None

    monkeypatch.setattr(module, "V2ProtoCodec", lambda _descriptor: object())
    monkeypatch.setattr(module, "load_v2_descriptor", lambda: object())
    monkeypatch.setattr(module, "_stage4_prewarm_payload_from_shards", lambda *_args: (b"x", 1))
    monkeypatch.setattr(module, "_stage4_payload_from_shards", lambda *_args: merged.append(_args))
    monkeypatch.setattr(module.gc, "collect", lambda: None)
    monkeypatch.setattr(module.gc, "freeze", lambda: None)
    monkeypatch.setattr(module.gc, "disable", lambda: None)
    module.stage4_coordinator_entrypoint(
        RequestReceiver(), OuterSender(), world_spec,
        (ShardSender(), ShardSender()), (ShardReceiver(0), ShardReceiver(1)), shard_pids,
    )
    failures = [value for value in outer if type(value) is module.LidarScanFailure]
    assert len(failures) == 1
    assert (
        failures[0].job_id, failures[0].lifecycle_generation, failures[0].pause_epoch,
        failures[0].topic, failures[0].timestamp_ns,
    ) == (7, 1, 0, "lidar_link", 700_000_000)
    assert len(merged) == 1
    assert merged[0][0] == next_request
    assert outer[-1] == module.LidarWorkerStopped(1, os.getpid())


def test_stage4_shard_runtime_exception_sends_exact_typed_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """shard scan 异常必须携带 shard/assignment/job 身份的私有 failure 信封。"""
    import gc

    module = _worker_module()
    process_id = os.getpid()
    request = SimpleNamespace(
        complete_obstacle_snapshots_without_body_ids=(),
        world_mount_pose=Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)),
        job_id=9, lifecycle_generation=1, pause_epoch=0, topic="lidar_link", timestamp_ns=900_000_000,
    )
    live = module._LiveWorkerBootstrap(7, None, SimpleNamespace(), None, None, object(), SimpleNamespace(), (), {})
    spec = SimpleNamespace(shard_id=0, first=0, start=0, stop=5_760, stride=2, count=2_880, world_spec=SimpleNamespace(world_digest="0" * 64))
    sent: list[object] = []

    class Receiver:
        def __init__(self) -> None:
            self.values = [object()]

        def recv(self) -> object:
            return self.values.pop(0)

        def close(self) -> None:
            return None

    class Sender:
        def send(self, value: object) -> None:
            sent.append(value)

        def close(self) -> None:
            return None

    monkeypatch.setattr(module, "_bootstrap_live_worker", lambda _spec, **_kwargs: live)
    monkeypatch.setattr(module, "_stage4_shard_prewarm", lambda _live, _spec: object())
    monkeypatch.setattr(module, "_reconstruct_scan_request", lambda _value: request)
    monkeypatch.setattr(module, "_stage4_shard_scan", lambda *_args: (_ for _ in ()).throw(RuntimeError("scan boom")))
    monkeypatch.setattr(module, "_disconnect_direct_client", lambda _client_id: None)
    monkeypatch.setattr(gc, "freeze", lambda: None)
    monkeypatch.setattr(gc, "disable", lambda: None)
    with pytest.raises(SystemExit):
        module.stage4_shard_entrypoint(Receiver(), Sender(), spec)
    failure_type = module._Stage4ShardFailure
    failures = [value for value in sent if type(value) is failure_type]
    assert len(failures) == 1
    failure = failures[0]
    assert (
        failure.shard_id, failure.process_id, failure.first, failure.stop, failure.stride,
        failure.count, failure.job_id, failure.lifecycle_generation, failure.pause_epoch,
        failure.topic, failure.timestamp_ns,
    ) == (0, process_id, 0, 5_760, 2, 2_880, 9, 1, 0, "lidar_link", 900_000_000)
    assert "\n" not in failure.bounded_detail and "traceback" not in failure.bounded_detail.lower()
    assert not any(type(value) in {module._Stage4ShardResult, module._Stage4ShardStopped, module.LidarWorkerStartupFailure} for value in sent)


def test_stage4_coordinator_preserves_shard_failure_detail(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """coordinator 的 whole-frame failure 必须保留 shard 的首个受限错误详情。"""
    module = _worker_module()
    descriptor = importlib.import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points", b"\x01" * 16, descriptor.sha256, 1, 7
    )
    request = module.LidarScanRequest(
        1, 7, time.monotonic_ns(), 1, 0, "lidar_link", "lidar_link", 1,
        700_000_000, Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)), None, (), identity,
    )
    world_spec = SimpleNamespace(world_digest="0" * 64)
    assignments = module._stage4_realtime_shard_assignments()
    shard_pids = (os.getpid() + 20, os.getpid() + 21)
    outer: list[object] = []

    class RequestReceiver:
        def recv(self) -> object:
            return request

        def close(self) -> None:
            return None

    class ShardSender:
        def send(self, _value: object) -> None:
            return None

        def close(self) -> None:
            return None

    class ShardReceiver:
        def __init__(self, shard_id: int) -> None:
            first, stop, stride, count = assignments[shard_id]
            runtime_value = (
                module._Stage4ShardFailure(
                    shard_id, shard_pids[shard_id], first, stop, stride, count,
                    request.job_id, request.lifecycle_generation, request.pause_epoch,
                    request.topic, request.timestamp_ns, "shard_scan_failed",
                    "Stage4 shard scan: RuntimeError: rayTestBatch failed",
                )
                if shard_id == 0 else object()
            )
            self.values = [
                module._Stage4ShardReady(
                    shard_id, shard_pids[shard_id], first, stop, stride, count,
                    world_spec.world_digest,
                ),
                object(),
                runtime_value,
            ]

        def recv(self) -> object:
            return self.values.pop(0)

        def poll(self, _timeout: float) -> bool:
            return True

        def close(self) -> None:
            return None

    class OuterSender:
        def send(self, value: object) -> None:
            outer.append(value)

        def close(self) -> None:
            return None

    monkeypatch.setattr(module, "V2ProtoCodec", lambda _descriptor: object())
    monkeypatch.setattr(module, "load_v2_descriptor", lambda: object())
    monkeypatch.setattr(module, "_stage4_prewarm_payload_from_shards", lambda *_args: (b"x", 1))
    monkeypatch.setattr(module.gc, "collect", lambda: None)
    monkeypatch.setattr(module.gc, "freeze", lambda: None)
    monkeypatch.setattr(module.gc, "disable", lambda: None)

    module.stage4_coordinator_entrypoint(
        RequestReceiver(), OuterSender(), world_spec,
        (ShardSender(), ShardSender()), (ShardReceiver(0), ShardReceiver(1)), shard_pids,
    )

    failure = next(value for value in outer if type(value) is module.LidarScanFailure)
    assert "rayTestBatch failed" in failure.bounded_detail


def test_stage4_coordinator_reconstructs_and_rejects_invalid_shard_result() -> None:
    """pickle 后 shard Result 必须在 coordinator 接收边界重构并校验全部字段。"""
    module = _worker_module()
    descriptor = importlib.import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points", b"\x01" * 16, descriptor.sha256, 1, 9
    )
    request = module.LidarScanRequest(
        1, 9, 1, 1, 0, "lidar_link", "lidar_link", 1, 900_000_000,
        Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)), None, (), identity,
    )
    assignment = (0, 5_760, 2, 2_880)
    reconstruct = module._reconstruct_stage4_shard_result
    lidar_pointcloud = importlib.import_module("slope_sim.lidar_pointcloud")

    def point_value(global_slot: int) -> tuple[int, float, float, float, int, int, int]:
        return (
            lidar_pointcloud.mid360_offset_time_ns(global_slot),
            1.0,
            2.0,
            3.0,
            100,
            1,
            lidar_pointcloud.mid360_line_for_slot(
                "livox-mid360-800000-v1",
                identity.world_generation,
                identity.sequence,
                global_slot,
            ),
        )

    def forge_exact(cls: type, **fields: object) -> object:
        forged = object.__new__(cls)
        for name, value in fields.items():
            object.__setattr__(forged, name, value)
        return forged

    fields_by_name = dict(
        shard_id=0, process_id=os.getpid(), first=0, stop=5_760, stride=2, count=2_880,
        examined_count=2_880, job_id=9, lifecycle_generation=1, pause_epoch=0,
        topic="lidar_link", timestamp_ns=900_000_000,
        values=((0, point_value(0)), (2, point_value(2))),
    )
    valid = forge_exact(module._Stage4ShardResult, **fields_by_name)
    reconstructed = reconstruct(
        valid, shard_id=0, process_id=os.getpid(), assignment=assignment, request=request
    )
    assert type(reconstructed) is module._Stage4ShardResult
    invalid_values = (
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "shard_id": True}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "job_id": True}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "first": 1}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "stride": 1}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "examined_count": 2_879}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((1, fields_by_name["values"][0][1]),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((2, fields_by_name["values"][0][1]), (0, fields_by_name["values"][0][1]))}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((True, fields_by_name["values"][0][1]),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (0, float("nan"), 2.0, 3.0, 100, 1, 0)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (0, float("inf"), 2.0, 3.0, 100, 1, 0)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (0, 1.0, 2.0, 3.0, True, 1, 0)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (0, 1.0)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (-1, 1.0, 2.0, 3.0, 100, 1, 0)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (1 << 32, 1.0, 2.0, 3.0, 100, 1, 0)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (0, 1.0, 2.0, 3.0, -1, 1, 0)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (0, 1.0, 2.0, 3.0, 1 << 32, 1, 0)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (0, 1.0, 2.0, 3.0, 100, -1, 0)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (0, 1.0, 2.0, 3.0, 100, 4, 0)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (0, 1.0, 2.0, 3.0, 100, 1, -1)),)}),
        forge_exact(module._Stage4ShardResult, **{**fields_by_name, "values": ((0, (0, 1.0, 2.0, 3.0, 100, 1, 16)),)}),
    )
    for invalid in (object(), *invalid_values):
        with pytest.raises(ValueError):
            reconstruct(
                invalid, shard_id=0, process_id=os.getpid(), assignment=assignment, request=request
            )


@pytest.mark.parametrize("tampered_field", ("line", "offset_time_ns"))
def test_stage4_coordinator_rejects_forged_shard_point_identity(
    tampered_field: str,
) -> None:
    """shard point 的 line 与 offset 必须由 request identity 和 global slot 唯一决定。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    identity = importlib.import_module(
        "slope_sim.interfaces.v2.session"
    ).OutputIdentity(
        "/sim/lidar/points", b"\x01" * 16, descriptor.sha256, 1, 9
    )
    line_for_slot = importlib.import_module(
        "slope_sim.lidar_pointcloud"
    ).mid360_line_for_slot
    request = module.LidarScanRequest(
        1, 9, 1, 1, 0, "lidar_link", "lidar_link", 1, 900_000_000,
        Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)), None, (), identity,
    )
    point_value = (
        1 if tampered_field == "offset_time_ns" else 0,
        1.0,
        2.0,
        3.0,
        100,
        1,
        4 if tampered_field == "line" else line_for_slot(
            "livox-mid360-800000-v1", identity.world_generation, identity.sequence, 0
        ),
    )
    forged = object.__new__(module._Stage4ShardResult)
    for name, value in {
        "shard_id": 0,
        "process_id": os.getpid(),
        "first": 0,
        "stop": 5_760,
        "stride": 2,
        "count": 2_880,
        "examined_count": 2_880,
        "job_id": 9,
        "lifecycle_generation": 1,
        "pause_epoch": 0,
        "topic": "lidar_link",
        "timestamp_ns": 900_000_000,
        "values": ((0, point_value),),
    }.items():
        object.__setattr__(forged, name, value)

    with pytest.raises(ValueError):
        module._reconstruct_stage4_shard_result(
            forged,
            shard_id=0,
            process_id=os.getpid(),
            assignment=(0, 5_760, 2, 2_880),
            request=request,
        )


def test_stage4_firing_identity_computes_request_phase_once(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """同一 request 的紧凑命中共享一个 phase，不能逐点重复 SHA。"""
    module = _worker_module()
    lidar_pointcloud = importlib.import_module("slope_sim.lidar_pointcloud")
    slots = (1, 3, 5)
    values = tuple(
        (
            slot,
            (
                lidar_pointcloud.mid360_offset_time_ns(slot),
                1.0,
                2.0,
                3.0,
                100,
                1,
                lidar_pointcloud.mid360_line_for_slot(
                    "livox-mid360-800000-v1", 1, 9, slot
                ),
            ),
        )
        for slot in slots
    )
    actual_line_for_slot = module.mid360_line_for_slot
    actual_offset_time_ns = module.mid360_offset_time_ns
    line_calls = []
    offset_calls = []

    def recording_line_for_slot(*args: object) -> int:
        line_calls.append(args)
        return actual_line_for_slot(*args)

    def recording_offset_time_ns(global_slot: object) -> int:
        offset_calls.append(global_slot)
        return actual_offset_time_ns(global_slot)

    monkeypatch.setattr(module, "mid360_line_for_slot", recording_line_for_slot)
    monkeypatch.setattr(module, "mid360_offset_time_ns", recording_offset_time_ns)

    module._require_stage4_firing_identity(
        values,
        world_generation=1,
        sequence=9,
    )

    assert line_calls == [("livox-mid360-800000-v1", 1, 9, 0)]
    assert offset_calls == []


def test_stage4_hot_values_are_validated_once_at_receive_boundary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """shard sender 不重复逐点检查，coordinator pickle 接收边界必须恰好检查一次。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    identity = importlib.import_module(
        "slope_sim.interfaces.v2.session"
    ).OutputIdentity("/sim/lidar/points", b"\x01" * 16, descriptor.sha256, 1, 9)
    request = module.LidarScanRequest(
        1, 9, 1, 1, 0, "lidar_link", "lidar_link", 1, 900_000_000,
        Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)), None, (), identity,
    )
    values = ((
        0,
        (
            0,
            1.0,
            2.0,
            3.0,
            100,
            1,
            module.mid360_line_for_slot(
                module.MID360_PATTERN_VERSION, 1, 9, 0
            ),
        ),
    ),)
    actual_validator = module._require_stage4_indexed_values
    calls = []

    def recording_validator(*args: object):
        calls.append(args)
        return actual_validator(*args)

    monkeypatch.setattr(module, "_require_stage4_indexed_values", recording_validator)
    result = module._Stage4ShardResult(
        0, os.getpid(), 0, 5_760, 2, 2_880, 2_880,
        9, 1, 0, "lidar_link", 900_000_000, values,
    )
    assert calls == []

    reconstructed = module._reconstruct_stage4_shard_result(
        result,
        shard_id=0,
        process_id=os.getpid(),
        assignment=(0, 5_760, 2, 2_880),
        request=request,
    )

    assert reconstructed == result
    assert calls == [(values, 0, 5_760, 2)]


def test_stage4_prewarm_values_are_validated_once_at_coordinator_boundary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """预热 sender 不重复逐点检查，outer Ready 前的 coordinator 边界各检查一次。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    codec = module.V2ProtoCodec(descriptor)
    identity = importlib.import_module(
        "slope_sim.interfaces.v2.session"
    ).OutputIdentity("/sim/lidar/points", b"\x00" * 16, descriptor.sha256, 1, 0)
    assignments = module._stage4_realtime_shard_assignments()
    process_id = os.getpid()
    actual_validator = module._require_stage4_indexed_values
    calls = []

    def recording_validator(*args: object):
        calls.append(args)
        return actual_validator(*args)

    monkeypatch.setattr(module, "_require_stage4_indexed_values", recording_validator)
    partials = tuple(
        module._Stage4ShardPrewarm(
            shard_id,
            process_id + shard_id,
            "0" * 64,
            first,
            stop,
            stride,
            count,
            count,
            0,
            1,
            0,
            "lidar_link",
            0,
            identity,
            ((
                first,
                (
                    module.mid360_offset_time_ns(first),
                    1.0,
                    2.0,
                    3.0,
                    100,
                    1,
                    module.mid360_line_for_slot(
                        module.MID360_PATTERN_VERSION, 1, 0, first
                    ),
                ),
            ),),
            10 + shard_id,
        )
        for shard_id, (first, stop, stride, count) in enumerate(assignments)
    )
    assert calls == []

    payload, duration_ns = module._stage4_prewarm_payload_from_shards(
        codec,
        partials,
        (process_id, process_id + 1),
        "0" * 64,
    )

    assert payload
    assert duration_ns > 11
    assert calls == [
        (partials[0].values, 0, 5_760, 2),
        (partials[1].values, 1, 5_760, 2),
    ]


def test_stage4_reconstructs_private_failure_and_stopped_at_receive_boundary() -> None:
    """私有 Failure/Stopped 也必须 exact reconstruct，伪造字段不能靠 type 或属性通过。"""
    module = _worker_module()
    assignment = (0, 5_760, 2, 2_880)
    request = module.LidarScanRequest(
        1, 9, 1, 1, 0, "lidar_link", "lidar_link", 1, 900_000_000,
        Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)), None, (), None,
    )
    failure_type = getattr(module, "_Stage4ShardFailure", None)
    assert failure_type is not None

    def forge_exact(cls: type, **fields: object) -> object:
        forged = object.__new__(cls)
        for name, value in fields.items():
            object.__setattr__(forged, name, value)
        return forged

    failure_fields = dict(
        shard_id=0, process_id=os.getpid(), first=0, stop=5_760, stride=2, count=2_880,
        job_id=9, lifecycle_generation=1, pause_epoch=0, topic="lidar_link", timestamp_ns=900_000_000,
        stable_error_code="shard_scan_failed", bounded_detail="scan failed: RuntimeError",
    )
    failure = forge_exact(failure_type, **failure_fields)
    reconstructed_failure = module._reconstruct_stage4_shard_failure(
        failure, shard_id=0, process_id=os.getpid(), assignment=assignment, request=request
    )
    assert type(reconstructed_failure) is module._Stage4ShardFailure
    for invalid in (
        object(),
        forge_exact(failure_type, **{**failure_fields, "shard_id": True}),
        forge_exact(failure_type, **{**failure_fields, "process_id": True}),
        forge_exact(failure_type, **{**failure_fields, "stable_error_code": "wrong"}),
        forge_exact(failure_type, **{**failure_fields, "bounded_detail": "line one\nline two"}),
        forge_exact(failure_type, **{**failure_fields, "bounded_detail": "x" * 513}),
    ):
        with pytest.raises(ValueError):
            module._reconstruct_stage4_shard_failure(
                invalid, shard_id=0, process_id=os.getpid(), assignment=assignment, request=request
            )

    stopped = forge_exact(module._Stage4ShardStopped, shard_id=0, process_id=os.getpid())
    reconstructed_stopped = module._reconstruct_stage4_shard_stopped(
        stopped, shard_id=0, process_id=os.getpid()
    )
    assert type(reconstructed_stopped) is module._Stage4ShardStopped
    for invalid in (
        object(),
        forge_exact(module._Stage4ShardStopped, shard_id=True, process_id=os.getpid()),
        forge_exact(module._Stage4ShardStopped, shard_id=0, process_id=True),
    ):
        with pytest.raises(ValueError):
            module._reconstruct_stage4_shard_stopped(
                invalid, shard_id=0, process_id=os.getpid()
            )


def test_stage4_cleanup_attempts_every_owned_child_after_one_cleanup_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """force_close 任一 close/reap 失败后仍必须遍历 coordinator 与两个 owned shard。"""
    module = _worker_module()
    events: list[str] = []

    class Endpoint:
        def close(self) -> None:
            events.append("request_close")
            raise OSError("request close failed")

    class Receiver:
        def close(self) -> None:
            events.append("response_close")

    coordinator = SimpleNamespace(name="coordinator")
    shards = (SimpleNamespace(name="shard0"), SimpleNamespace(name="shard1"))

    def reap(process: object, *, initial_join_timeout_sec: float) -> None:
        assert initial_join_timeout_sec == 0.0
        events.append(getattr(process, "name"))
        if getattr(process, "name") == "coordinator":
            raise OSError("coordinator reap failed")

    monkeypatch.setattr(module, "_reap_owned_process", reap)
    handle = module.LidarWorkerHandle(
        coordinator,
        Endpoint(),
        Receiver(),
        SimpleNamespace(process_id=1),
        shards,
    )

    with pytest.raises(OSError, match="request close failed"):
        handle.force_close()
    assert events == ["request_close", "coordinator", "shard0", "shard1", "response_close"]


def test_stage4_normal_close_reaps_every_child_without_replacing_primary_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """normal close 的次生回收错误不能阻断 sibling 回收或替换协议主错误。"""
    module = _worker_module()
    events: list[str] = []

    class Sender:
        def send(self, _value: object) -> None:
            return None

        def close(self) -> None:
            events.append("request_close")

    class Receiver:
        def poll(self, _timeout_sec: float) -> bool:
            return True

        def recv(self) -> object:
            return object()

        def close(self) -> None:
            events.append("response_close")

    coordinator = SimpleNamespace(name="coordinator")
    shards = (SimpleNamespace(name="shard0"), SimpleNamespace(name="shard1"))

    def reap(process: object, *, initial_join_timeout_sec: float) -> None:
        assert initial_join_timeout_sec == 0.0
        name = getattr(process, "name")
        events.append(name)
        if name == "coordinator":
            raise OSError("coordinator reap failed")

    monkeypatch.setattr(module, "_reap_owned_process", reap)
    handle = module.LidarWorkerHandle(
        coordinator,
        Sender(),
        Receiver(),
        module.LidarWorkerReady(
            1,
            42,
            "1" * 64,
            ("lidar_front", "lidar_rear"),
            (("lidar_front", "2" * 64), ("lidar_rear", "3" * 64)),
            1,
        ),
        shards,
    )

    with pytest.raises(RuntimeError, match="normal shutdown") as captured:
        handle.close(timeout_sec=0.1)
    assert type(captured.value.__cause__) is ValueError
    assert events == ["request_close", "coordinator", "shard0", "shard1", "response_close"]


@pytest.mark.parametrize(
    ("failed_process", "expected_reaped", "cleanup_failed_process"),
    (
        ("shard1", ("shard0",), "shard0"),
        ("coordinator", ("shard0", "shard1"), "shard1"),
    ),
)
def test_stage4_partial_start_cleanup_preserves_original_error(
    monkeypatch: pytest.MonkeyPatch,
    failed_process: str,
    expected_reaped: tuple[str, ...],
    cleanup_failed_process: str,
) -> None:
    """部分 spawn 失败时必须遍历已启动资源，且 cleanup 错误不能覆盖 start 错误。"""
    module = _worker_module()
    endpoints: list[Endpoint] = []
    reaped: list[str] = []
    start_error = OSError("start failed")

    class Endpoint:
        def __init__(self, name: str, *, raises_on_close: bool = False) -> None:
            self.name = name
            self.raises_on_close = raises_on_close
            self.close_calls = 0
            endpoints.append(self)

        def close(self) -> None:
            self.close_calls += 1
            if self.raises_on_close:
                raise OSError("endpoint cleanup failed")

    class Process:
        def __init__(self, name: str, process_id: int) -> None:
            self.name = name
            self.pid = process_id
            self.start_calls = 0
            self.join_calls = 0
            self.started = False

        def start(self) -> None:
            self.start_calls += 1
            if self.name == failed_process:
                raise start_error
            self.started = True

        def join(self, _timeout_sec: float) -> None:
            self.join_calls += 1

    class Context:
        def __init__(self) -> None:
            self.pipe_count = 0
            self.shard_count = 0
            self.processes: dict[str, Process] = {}

        def Pipe(self, _duplex: bool) -> tuple[Endpoint, Endpoint]:
            pipe_id = self.pipe_count
            self.pipe_count += 1
            return (
                Endpoint(f"pipe{pipe_id}:receiver", raises_on_close=pipe_id == 0),
                Endpoint(f"pipe{pipe_id}:sender"),
            )

        def Process(self, *, target: object, args: object, daemon: bool) -> Process:
            if target is module.stage4_shard_entrypoint:
                name = f"shard{self.shard_count}"
                self.shard_count += 1
            else:
                name = "coordinator"
            process = Process(name, 100 + len(self.processes))
            self.processes[name] = process
            return process

    def reap(process: Process, *, initial_join_timeout_sec: float) -> None:
        reaped.append(process.name)
        if process.name == cleanup_failed_process:
            raise OSError("reap cleanup failed")

    context = Context()
    monkeypatch.setattr(module, "_reap_owned_process", reap)

    with pytest.raises(module.LidarWorkerStartupError) as captured:
        module._start_stage4_coordinator_worker(
            context,
            _stage4_golf_worker_world_spec(),
            0.1,
        )

    assert captured.value.stable_error_code == "worker_start_failed"
    assert captured.value.bounded_detail == "process start failed: OSError: start failed"
    assert captured.value.__cause__ is start_error
    assert set(reaped) == set(expected_reaped)
    assert len(reaped) == len(expected_reaped)
    assert all(endpoint.close_calls > 0 for endpoint in endpoints)
    failed = context.processes[failed_process]
    assert failed.name not in reaped
    assert failed.join_calls == 0


def test_stage4_interleaved_coordinator_payload_matches_single_world_scalar_oracle() -> None:
    """真实双 shard coordinator 必须逐点匹配独立 DIRECT scalar oracle。"""
    module = _worker_module()
    assignments = module._stage4_realtime_shard_assignments()
    assert assignments == ((0, 5_760, 2, 2_880), (1, 5_760, 2, 2_880))

    descriptor = importlib.import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    identity_type = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity
    codec = importlib.import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    world_spec = _stage4_golf_worker_world_spec()
    client_id = p.connect(p.DIRECT)
    handle = None
    try:
        world, manager = build_world_from_scene_document(
            client_id, world_spec.experiment_config, world_spec.scene_document
        )
        backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
        backend.bind_scene(
            world.scene.body_ids,
            manager.snapshot(include_body_id=True),
        )
        scanner = MultiLineLidar.stage4(backend, Stage4LidarProfile.realtime())
        mount = scanner._world_mount()
        rotation = p.getMatrixFromQuaternion(mount.orientation)

        def world_endpoint(
            direction: tuple[float, float, float],
            distance: float,
        ) -> tuple[float, float, float]:
            local_x, local_y, local_z = (
                component * distance for component in direction
            )
            return (
                mount.position[0]
                + rotation[0] * local_x
                + rotation[1] * local_y
                + rotation[2] * local_z,
                mount.position[1]
                + rotation[3] * local_x
                + rotation[4] * local_y
                + rotation[5] * local_z,
                mount.position[2]
                + rotation[6] * local_x
                + rotation[7] * local_y
                + rotation[8] * local_z,
            )

        directions = tuple(
            _stage4_oracle_firing(1, 1, global_slot)[2]
            for global_slot in range(_STAGE4_ORACLE_FIRING_SLOTS)
        )
        starts = tuple(
            world_endpoint(direction, _STAGE4_ORACLE_MIN_RANGE_M)
            for direction in directions
        )
        ends = tuple(
            world_endpoint(direction, _STAGE4_ORACLE_MAX_RANGE_M)
            for direction in directions
        )
        scalar_hits = backend.ray_test_batch(
            starts,
            ends,
            collision_mask=module.LIDAR_VISIBLE_GROUP,
        )
        indexed_hits = tuple(
            (index, hit) for index, hit in enumerate(scalar_hits) if hit.hit
        )
        local_points = backend.inverse_transform_points(
            mount, tuple(hit.hit_position for _index, hit in indexed_hits)
        )
        oracle = []
        for (global_slot, hit), local_point in zip(
            indexed_hits, local_points, strict=True
        ):
            distance = math.hypot(*local_point)
            below_minimum = distance < _STAGE4_ORACLE_MIN_RANGE_M and not math.isclose(
                distance,
                _STAGE4_ORACLE_MIN_RANGE_M,
                rel_tol=_STAGE4_ORACLE_RANGE_REL_TOLERANCE,
                abs_tol=_STAGE4_ORACLE_RANGE_ABS_TOLERANCE_M,
            )
            above_maximum = distance > _STAGE4_ORACLE_MAX_RANGE_M and not math.isclose(
                distance,
                _STAGE4_ORACLE_MAX_RANGE_M,
                rel_tol=_STAGE4_ORACLE_RANGE_REL_TOLERANCE,
                abs_tol=_STAGE4_ORACLE_RANGE_ABS_TOLERANCE_M,
            )
            if below_minimum or above_maximum:
                continue
            assert hit.category in _STAGE4_ORACLE_POINT_SEMANTICS
            tag, reflectivity = _STAGE4_ORACLE_POINT_SEMANTICS[hit.category]
            offset_time_ns, line, _direction = _stage4_oracle_firing(
                1, 1, global_slot
            )
            oracle.append(
                (
                    offset_time_ns,
                    *local_point,
                    reflectivity,
                    tag,
                    line,
                )
            )
        oracle = tuple(oracle)
        handle = module.start_lidar_worker(world_spec, startup_timeout_sec=15.0)
        identity = identity_type("/sim/lidar/points", b"\x05" * 16, descriptor.sha256, 1, 1)
        request = module.LidarScanRequest(
            1, 1, time.monotonic_ns(), 1, 0, "lidar_link", "lidar_link", 1,
            100_000_000, mount, None, manager.snapshot(include_body_id=False), identity,
        )
        handle.request_sender.send(request)
        assert handle.response_receiver.poll(15.0)
        response = handle.response_receiver.recv()
        assert type(response) is module.PreparedLidarPayload
        assert (
            response.job_id, response.lifecycle_generation, response.pause_epoch,
            response.topic, response.timestamp_ns,
        ) == (1, 1, 0, "lidar_link", 100_000_000)
        decoded = codec.decode_lidar_point_cloud(response.protobuf_payload)
        assert (
            decoded.timebase_ns, decoded.sequence, decoded.world_generation,
            decoded.simulation_session_id, decoded.descriptor_sha256,
        ) == (100_000_000, 1, 1, b"\x05" * 16, descriptor.sha256)
        assert (decoded.frame_id, decoded.lidar_id) == ("lidar_link", 1)
        observed = tuple(
            (point.offset_time_ns, point.x, point.y, point.z, point.reflectivity, point.tag, point.line)
            for point in decoded.points
        )
        assert decoded.point_num == len(observed) == len(oracle)
        for actual, expected in zip(observed, oracle, strict=True):
            assert actual[0] == expected[0]
            assert actual[4:] == expected[4:]
            assert all(abs(actual[axis] - expected[axis]) <= 0.001 for axis in (1, 2, 3))
    finally:
        if handle is not None:
            try:
                handle.close()
            except RuntimeError:
                handle.force_close()
        p.disconnect(client_id)


def test_stage4_coordinator_merges_reversed_shard_hits_in_global_ray_order() -> None:
    """乱序到达的两个完整 shard 只能按全局 ray index 合并紧凑命中。"""
    module = _worker_module()
    first = (10, 1.0, 0.0, 0.1, 100, 1, 0)
    second = (20, 2.0, 0.0, 0.1, 160, 2, 0)
    third = (30, 3.0, 0.0, 0.1, 200, 3, 8)

    merged = module._merge_stage4_shard_indexed_values(
        (
            (1, 1, 5760, 2, 2880, 2880, ((4001, third),)),
            (0, 0, 5760, 2, 2880, 2880, ((4, first), (12, second))),
        )
    )

    assert merged == (first, second, third)


def test_stage4_start_owns_exactly_two_sibling_shards_and_reaps_them() -> None:
    """Stage4 handle 必须由 parent 精确拥有两个与 coordinator 同级的 shard。"""
    module = _worker_module()
    handle = module.start_lidar_worker(
        _stage4_golf_worker_world_spec(),
        startup_timeout_sec=15.0,
    )
    try:
        shards = handle._stage4_shard_processes
        assert len(shards) == 2
        assert all(shard.daemon is False and shard.is_alive() for shard in shards)
        assert {shard.pid for shard in shards}.isdisjoint({handle.process.pid})
    finally:
        handle.close()
    assert all(shard.is_alive() is False and shard.exitcode == 0 for shard in shards)


def test_stage4_worker_encodes_center_payload_with_reserved_v2_identity() -> None:
    """中心 worker 必须在 child 内编码一次 v2 payload，并保留已预留的输出身份。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points",
        b"\x01" * 16,
        descriptor.sha256,
        3,
        7,
    )
    world_spec = _worker_world_spec()
    stage4_spec = module.LidarWorkerWorldSpec(
        world_spec.protocol_version,
        world_spec.experiment_config,
        world_spec.scene_document,
        world_spec.world_digest,
        "stage4",
    )
    live = module._bootstrap_live_worker(stage4_spec)
    assert type(live) is module._LiveWorkerBootstrap
    request = module.LidarScanRequest(
        1,
        1,
        1,
        0,
        0,
        "lidar_link",
        "lidar_link",
        1,
        100_000_000,
        Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)),
        None,
        (),
        output_identity=identity,
    )
    try:
        prepared = module._process_scan_request(live, request)
    finally:
        module._disconnect_direct_client(live.client_id)

    assert type(prepared) is module.PreparedLidarPayload
    decoded = importlib.import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(
        descriptor
    ).decode_lidar_point_cloud(prepared.protobuf_payload)
    assert (prepared.topic, prepared.timestamp_ns) == ("lidar_link", 100_000_000)
    assert (
        decoded.sequence,
        decoded.world_generation,
        decoded.simulation_session_id,
        decoded.descriptor_sha256,
    ) == (7, 3, b"\x01" * 16, descriptor.sha256)


def test_stage4_worker_encodes_payload_without_duplicate_v2_point_models(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Worker 热路径必须直接生成与通用 v2 codec 完全相同的确定性 bytes。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    v2_codec = importlib.import_module("slope_sim.interfaces.v2.codec")
    v2_models = importlib.import_module("slope_sim.interfaces.v2.models")
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points",
        b"\x03" * 16,
        descriptor.sha256,
        2,
        9,
    )
    point_type = importlib.import_module("slope_sim.interfaces.models").LidarPoint
    message = LidarPointCloud(
        200_000_000,
        "lidar_link",
        2,
        1,
        (
            point_type(0, 1.25, 0.0, 0.1, 200, 3, 0),
            point_type(99_982_638, 1.0, -0.2, 0.3, 100, 1, 15),
        ),
    )
    expected_codec = v2_codec.V2ProtoCodec(descriptor)
    expected = expected_codec.encode(
        v2_models.LidarPointCloudV2(
            message.timebase_ns,
            message.frame_id,
            message.point_num,
            message.lidar_id,
            tuple(
                v2_models.LidarPointV2(
                    point.offset_time_ns,
                    point.x,
                    point.y,
                    point.z,
                    point.reflectivity,
                    point.tag,
                    point.line,
                )
                for point in message.points
            ),
            identity.sequence,
            identity.world_generation,
            identity.simulation_session_id,
            identity.descriptor_sha256,
        )
    ).payload
    worker_codec = v2_codec.V2ProtoCodec(descriptor)
    monkeypatch.setattr(
        worker_codec,
        "encode",
        lambda _model: pytest.fail("worker rebuilt duplicate v2 point models"),
    )

    actual = module._encode_v2_lidar_payload(worker_codec, message, identity)

    assert actual == expected


def test_stage4_center_worker_fuses_indexed_hits_without_legacy_point_objects(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """中心无俯视帧扫描直接由真实紧凑命中写出与正式 V2 codec 相同的 bytes。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    v2_codec = importlib.import_module("slope_sim.interfaces.v2.codec")
    v2_models = importlib.import_module("slope_sim.interfaces.v2.models")
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points",
        b"\x04" * 16,
        descriptor.sha256,
        5,
        11,
    )
    world_spec = _worker_world_spec()
    stage4_spec = module.LidarWorkerWorldSpec(
        world_spec.protocol_version,
        world_spec.experiment_config,
        world_spec.scene_document,
        world_spec.world_digest,
        "stage4",
    )
    live = module._bootstrap_live_worker(stage4_spec)
    assert type(live) is module._LiveWorkerBootstrap
    snapshots = _reconcile_snapshots(moving_x=2.2, include_static=True)
    request = module.LidarScanRequest(
        1,
        44,
        123_000_044,
        3,
        2,
        "lidar_link",
        "lidar_link",
        1,
        1_300_000_000,
        Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)),
        None,
        snapshots,
        output_identity=identity,
    )
    try:
        module._reconcile_obstacles(live, snapshots)
        indexed_local_hits = live.center_scanner._indexed_local_hits_at_mount(
            request.world_mount_pose,
            pattern_version=importlib.import_module(
                "slope_sim.lidar_pointcloud"
            ).MID360_PATTERN_VERSION,
            world_generation=identity.world_generation,
            sequence=identity.sequence,
        )
        baseline_values = tuple(
            value
            for (global_slot, hit), local_point in indexed_local_hits
            if (
                value := live.center_scanner._stage4_point_values_from_hit(
                    global_slot,
                    hit,
                    local_point,
                    pattern_version=importlib.import_module(
                        "slope_sim.lidar_pointcloud"
                    ).MID360_PATTERN_VERSION,
                    world_generation=identity.world_generation,
                    sequence=identity.sequence,
                )
            ) is not None
        )
        assert {value[5] for value in baseline_values} >= {2, 3}
        expected = v2_codec.V2ProtoCodec(descriptor).encode(
            v2_models.LidarPointCloudV2(
                request.timestamp_ns,
                "lidar_link",
                len(baseline_values),
                1,
                tuple(
                    v2_models.LidarPointV2(
                        *value,
                    )
                    for value in baseline_values
                ),
                identity.sequence,
                identity.world_generation,
                identity.simulation_session_id,
                identity.descriptor_sha256,
            )
        ).payload

        def reject_legacy_point(*_args, **_kwargs):
            raise AssertionError("fused center path must not construct LidarPoint")

        def reject_legacy_cloud(*_args, **_kwargs):
            raise AssertionError("fused center path must not construct LidarPointCloud")

        monkeypatch.setattr("slope_sim.lidar_pointcloud.LidarPoint", reject_legacy_point)
        monkeypatch.setattr(
            "slope_sim.lidar_pointcloud.LidarPointCloud", reject_legacy_cloud
        )
        prepared = module._process_scan_request(live, request)
    finally:
        module._disconnect_direct_client(live.client_id)

    assert type(prepared) is module.PreparedLidarPayload
    assert prepared.protobuf_payload == expected


@pytest.mark.parametrize("invalid_coordinate", (float("nan"), True))
def test_stage4_center_fast_path_rejects_invalid_prevalidated_inverse_coordinates(
    monkeypatch: pytest.MonkeyPatch,
    invalid_coordinate: object,
) -> None:
    """紧凑逆变换不得把 NaN 或 bool 坐标静默写进中心 protobuf。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points",
        b"\x06" * 16,
        descriptor.sha256,
        1,
        12,
    )
    world_spec = _worker_world_spec()
    stage4_spec = module.LidarWorkerWorldSpec(
        world_spec.protocol_version,
        world_spec.experiment_config,
        world_spec.scene_document,
        world_spec.world_digest,
        "stage4",
    )
    live = module._bootstrap_live_worker(stage4_spec)
    assert type(live) is module._LiveWorkerBootstrap
    request = module.LidarScanRequest(
        1,
        45,
        123_000_045,
        3,
        2,
        "lidar_link",
        "lidar_link",
        1,
        1_400_000_000,
        Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)),
        None,
        (),
        output_identity=identity,
    )
    try:
        monkeypatch.setattr(
            live.backend,
            "inverse_transform_points_prevalidated",
            lambda _mount, points: tuple(
                (invalid_coordinate, 0.0, 1.0) for _point in points
            ),
        )
        failure = module._process_scan_request(live, request)
    finally:
        module._disconnect_direct_client(live.client_id)

    assert type(failure) is module.LidarScanFailure
    assert failure.stable_error_code == "pointcloud_failed"
    assert not hasattr(failure, "protobuf_payload")


def test_stage4_center_fast_path_reports_protobuf_serialization_failure_as_codec_failed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """中心 protobuf 的确定性序列化失败必须沿用 codec_failed 收口。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points",
        b"\x07" * 16,
        descriptor.sha256,
        1,
        13,
    )
    world_spec = _worker_world_spec()
    stage4_spec = module.LidarWorkerWorldSpec(
        world_spec.protocol_version,
        world_spec.experiment_config,
        world_spec.scene_document,
        world_spec.world_digest,
        "stage4",
    )
    live = module._bootstrap_live_worker(stage4_spec)
    assert type(live) is module._LiveWorkerBootstrap
    request = module.LidarScanRequest(
        1,
        46,
        123_000_046,
        3,
        2,
        "lidar_link",
        "lidar_link",
        1,
        1_500_000_000,
        Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)),
        None,
        (),
        output_identity=identity,
    )
    lidar_module = importlib.import_module("slope_sim.lidar_pointcloud")
    protobuf_type = lidar_module.pb.LidarPointCloud

    class SerializationFailureMessage:
        def __init__(self, **kwargs) -> None:
            object.__setattr__(self, "_delegate", protobuf_type(**kwargs))

        @property
        def points(self):
            return self._delegate.points

        @property
        def point_num(self):
            return self._delegate.point_num

        @point_num.setter
        def point_num(self, value) -> None:
            self._delegate.point_num = value

        def SerializeToString(self, *, deterministic: bool) -> bytes:
            assert deterministic is True
            raise RuntimeError("forced protobuf serialization failure")

    try:
        monkeypatch.setattr(lidar_module.pb, "LidarPointCloud", SerializationFailureMessage)
        failure = module._process_scan_request(live, request)
    finally:
        module._disconnect_direct_client(live.client_id)

    assert type(failure) is module.LidarScanFailure
    assert failure.stable_error_code == "codec_failed"


def test_stage4_worker_freezes_prewarmed_heap_before_ready(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """中心 worker 必须把预热世界移出逐帧 GC 扫描后再宣布 Ready。"""
    import gc

    module = _worker_module()
    process_id = os.getpid()
    ready = module.LidarWorkerReady(
        1,
        process_id,
        "0" * 64,
        ("lidar_link",),
        (("lidar_link", "1" * 64),),
        1,
    )
    live = module._LiveWorkerBootstrap(
        7,
        ready,
        SimpleNamespace(),
        None,
        None,
        object(),
        SimpleNamespace(),
        (),
        {},
    )
    events: list[object] = []

    class RequestReceiver:
        def recv(self):
            return module.LidarWorkerStop(1, process_id)

        def close(self) -> None:
            events.append("request_closed")

    class ResponseSender:
        def send(self, value) -> None:
            events.append(("send", type(value)))

        def close(self) -> None:
            events.append("response_closed")

    monkeypatch.setattr(module, "_bootstrap_live_worker", lambda _spec: live)
    monkeypatch.setattr(
        module,
        "_disconnect_direct_client",
        lambda client_id: events.append(("disconnect", client_id)),
    )
    monkeypatch.setattr(gc, "freeze", lambda: events.append("freeze"))

    module.lidar_worker_entrypoint(RequestReceiver(), ResponseSender(), object())

    assert events[0] == "freeze"
    assert events[1] == ("send", module.LidarWorkerReady)


def test_stage4_shard_freezes_prewarmed_heap_before_ready(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """成功 shard 必须在 Ready 前冻结并禁用 GC；启动失败不得调用二者。"""
    import gc

    module = _worker_module()
    process_id = os.getpid()
    ready = module.LidarWorkerReady(
        1,
        process_id,
        "0" * 64,
        ("lidar_link",),
        (("lidar_link", "1" * 64),),
        1,
    )
    live = module._LiveWorkerBootstrap(
        7,
        ready,
        SimpleNamespace(),
        None,
        None,
        object(),
        SimpleNamespace(),
        (),
        {},
    )
    spec = SimpleNamespace(
        shard_id=0,
        first=0,
        stop=5_760,
        stride=2,
        count=2_880,
        world_spec=SimpleNamespace(world_digest="0" * 64),
    )
    events: list[object] = []

    class RequestReceiver:
        def recv(self):
            return module.LidarWorkerStop(1, process_id)

        def close(self) -> None:
            events.append("request_closed")

    class ResponseSender:
        def send(self, value) -> None:
            events.append(("send", type(value)))

        def close(self) -> None:
            events.append("response_closed")

    monkeypatch.setattr(module, "_bootstrap_live_worker", lambda _spec, **_kwargs: live)
    monkeypatch.setattr(module, "_stage4_shard_prewarm", lambda _live, _spec: object())
    monkeypatch.setattr(
        module,
        "_disconnect_direct_client",
        lambda client_id: events.append(("disconnect", client_id)),
    )
    monkeypatch.setattr(gc, "freeze", lambda: events.append("freeze"))
    monkeypatch.setattr(gc, "disable", lambda: events.append("disable"))

    module.stage4_shard_entrypoint(RequestReceiver(), ResponseSender(), spec)

    assert events.count("freeze") == 1
    assert events.count("disable") == 1
    assert events[:3] == ["freeze", "disable", ("send", module._Stage4ShardReady)]

    failure = module.LidarWorkerStartupFailure(
        1,
        process_id,
        "world_build",
        "worker_preflight_failed",
        "failure",
    )
    events.clear()
    monkeypatch.setattr(module, "_bootstrap_live_worker", lambda _spec, **_kwargs: failure)

    with pytest.raises(SystemExit):
        module.stage4_shard_entrypoint(RequestReceiver(), ResponseSender(), spec)

    assert "freeze" not in events
    assert "disable" not in events
    assert events[0] == ("send", module.LidarWorkerStartupFailure)


def test_stage4_shard_treats_parent_pipe_eof_as_a_normal_stop(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """父 coordinator 退出并关闭请求管道时，shard 不得打印未处理 EOF 栈。"""
    import gc

    module = _worker_module()
    process_id = os.getpid()
    events: list[object] = []
    live = SimpleNamespace(client_id=7)
    spec = SimpleNamespace(
        shard_id=0,
        first=0,
        stop=5_760,
        stride=2,
        count=2_880,
        world_spec=SimpleNamespace(world_digest="0" * 64),
    )

    class RequestReceiver:
        def recv(self) -> object:
            raise EOFError("coordinator closed request pipe")

        def close(self) -> None:
            events.append("request_closed")

    class ResponseSender:
        def send(self, value: object) -> None:
            events.append(("send", type(value)))

        def close(self) -> None:
            events.append("response_closed")

    monkeypatch.setattr(module, "_bootstrap_live_worker", lambda _spec, **_kwargs: live)
    monkeypatch.setattr(module, "_stage4_shard_prewarm", lambda _live, _spec: object())
    monkeypatch.setattr(module, "_disconnect_direct_client", lambda client_id: events.append(("disconnect", client_id)))
    monkeypatch.setattr(gc, "freeze", lambda: None)
    monkeypatch.setattr(gc, "disable", lambda: None)

    module.stage4_shard_entrypoint(RequestReceiver(), ResponseSender(), spec)

    assert ("disconnect", 7) in events
    assert ("send", module._Stage4ShardStopped) in events


def test_stage4_shard_prewarm_uses_its_assignment_and_active_thread_quota(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """每个 shard 的启动预热必须只执行自己的交错 assignment 和线程配额。"""
    import numpy as np
    module = _worker_module()
    calls: list[tuple[tuple[int, ...], tuple[int, ...], int]] = []
    hit = SimpleNamespace(hit_position=(1.0, 2.0, 3.0))

    class Scanner:
        def _stage4_world_rays(self, _mount: object):
            raise AssertionError("shard prewarm must not construct a full ray table")

        def _stage4_world_rays_for_slots(
            self,
            _mount: object,
            *,
            pattern_version: str,
            world_generation: int,
            sequence: int,
            global_slots: object,
        ):
            assert pattern_version == "livox-mid360-800000-v1"
            assert (world_generation, sequence) == (1, 0)
            slots = np.asarray(tuple(global_slots), dtype=np.float64)
            starts = np.column_stack((slots, slots, slots))
            ends = starts + 10_000.0
            starts.setflags(write=False)
            ends.setflags(write=False)
            return starts, ends

        def _stage4_point_values_from_hit(
            self,
            index: int,
            _hit: object,
            _local: object,
            *,
            pattern_version: str,
            world_generation: int,
            sequence: int,
        ):
            assert pattern_version == "livox-mid360-800000-v1"
            assert (world_generation, sequence) == (1, 0)
            return (index, 1.0, 2.0, 3.0, 100, 1, 0)

    class Backend:
        def _ray_test_indexed_hits_ndarray(
            self,
            starts,
            ends,
            *,
            collision_mask: int,
            num_threads: int,
        ):
            assert collision_mask == module.LIDAR_VISIBLE_GROUP
            assert starts.flags.c_contiguous and ends.flags.c_contiguous
            assert not starts.flags.writeable and not ends.flags.writeable
            calls.append((starts.copy(), ends.copy(), num_threads))
            return ((0, hit),)

        def inverse_transform_points_prevalidated(self, _mount: object, _points: object):
            return ((1.0, 2.0, 3.0),)

    descriptor_sha256 = b"\x05" * 32
    live = SimpleNamespace(
        center_scanner=Scanner(),
        backend=Backend(),
        codec=module.V2ProtoCodec(SimpleNamespace(sha256=descriptor_sha256)),
    )
    monkeypatch.setattr(module, "_world_mount_pose", lambda _backend, _scanner: object())

    partials = tuple(
        module._stage4_shard_prewarm(
            live,
            SimpleNamespace(
                shard_id=shard_id,
                first=first,
                stop=stop,
                stride=stride,
                count=count,
                world_spec=SimpleNamespace(world_digest="0" * 64),
            ),
        )
        for shard_id, (first, stop, stride, count) in enumerate(module._stage4_realtime_shard_assignments())
    )

    assert [(call[0][0, 0], call[0][-1, 0], len(call[0]), call[2]) for call in calls] == [
        (0.0, 5_758.0, 2_880, 2),
        (1.0, 5_759.0, 2_880, 2),
    ]
    for shard_id, partial in enumerate(partials):
        first, stop, stride, count = module._stage4_realtime_shard_assignments()[shard_id]
        assert (
            partial.shard_id,
            partial.first,
            partial.stop,
            partial.stride,
            partial.count,
            partial.examined_count,
            partial.job_id,
            partial.lifecycle_generation,
            partial.pause_epoch,
            partial.topic,
            partial.timestamp_ns,
            partial.output_identity.descriptor_sha256,
        ) == (shard_id, first, stop, stride, count, count, 0, 1, 0, "lidar_link", 0, descriptor_sha256)
        assert partial.values[0][0] == first
        assert partial.duration_ns > 0


def test_stage4_shard_prewarm_failure_sends_no_ready_and_disconnects_direct(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """shard prewarm 异常必须 fail closed，断开已建 DIRECT 且不能宣布 Ready。"""
    import gc

    module = _worker_module()
    process_id = os.getpid()
    events: list[object] = []
    live = SimpleNamespace(client_id=7)
    spec = SimpleNamespace(
        shard_id=0,
        first=0,
        stop=5_760,
        stride=2,
        count=2_880,
        world_spec=SimpleNamespace(world_digest="0" * 64),
    )

    class RequestReceiver:
        def close(self) -> None:
            events.append("request_closed")

    class ResponseSender:
        def send(self, value: object) -> None:
            events.append(("send", type(value)))

        def close(self) -> None:
            events.append("response_closed")

    monkeypatch.setattr(module, "_bootstrap_live_worker", lambda _spec, **_kwargs: live)
    monkeypatch.setattr(
        module,
        "_stage4_shard_prewarm",
        lambda _live, _spec: (_ for _ in ()).throw(RuntimeError("prewarm failed")),
    )
    monkeypatch.setattr(
        module,
        "_disconnect_direct_client",
        lambda client_id: events.append(("disconnect", client_id)),
    )
    monkeypatch.setattr(gc, "freeze", lambda: events.append("freeze"))
    monkeypatch.setattr(gc, "disable", lambda: events.append("disable"))

    with pytest.raises(SystemExit):
        module.stage4_shard_entrypoint(RequestReceiver(), ResponseSender(), spec)

    assert events[0] == ("send", module.LidarWorkerStartupFailure)
    assert ("send", module._Stage4ShardReady) not in events
    assert "freeze" not in events and "disable" not in events
    assert ("disconnect", 7) in events


def test_stage4_coordinator_treats_parent_pipe_eof_as_a_normal_stop(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """父 runtime 关闭请求管道时，coordinator 必须收束 sibling 而非抛 EOF。"""
    module = _worker_module()
    process_id = os.getpid()
    events: list[object] = []
    world_spec = SimpleNamespace(world_digest="0" * 64)
    assignments = module._stage4_realtime_shard_assignments()

    class RequestReceiver:
        def recv(self) -> object:
            raise EOFError("runtime closed request pipe")

        def close(self) -> None:
            events.append("request_closed")

    class OuterSender:
        def send(self, value: object) -> None:
            events.append(("outer_send", type(value)))

        def close(self) -> None:
            events.append("outer_closed")

    class ShardSender:
        def send(self, _value: object) -> None:
            raise AssertionError("EOF shutdown must close shard pipes, not send Stop")

        def close(self) -> None:
            events.append("shard_sender_closed")

    class ShardReceiver:
        def __init__(self, shard_id: int) -> None:
            first, stop, stride, count = assignments[shard_id]
            self.values = [
                module._Stage4ShardReady(
                    shard_id, process_id + shard_id, first, stop, stride, count,
                    world_spec.world_digest,
                ),
                object(),
            ]

        def recv(self) -> object:
            return self.values.pop(0)

        def close(self) -> None:
            events.append("shard_receiver_closed")

    monkeypatch.setattr(module, "load_v2_descriptor", lambda: object())
    monkeypatch.setattr(module, "V2ProtoCodec", lambda _descriptor: object())
    monkeypatch.setattr(module, "_stage4_prewarm_payload_from_shards", lambda *_args: (b"prewarm", 1))
    monkeypatch.setattr(module.gc, "collect", lambda: None)
    monkeypatch.setattr(module.gc, "freeze", lambda: None)
    monkeypatch.setattr(module.gc, "disable", lambda: None)

    module.stage4_coordinator_entrypoint(
        RequestReceiver(),
        OuterSender(),
        world_spec,
        (ShardSender(), ShardSender()),
        (ShardReceiver(0), ShardReceiver(1)),
        (process_id, process_id + 1),
    )

    assert events.count("shard_sender_closed") == 2
    assert events.count("shard_receiver_closed") == 2
    assert "outer_closed" in events


def test_stage4_coordinator_ready_uses_merged_shard_prewarm_payload(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """outer Ready 必须公布两个真实 shard partial 合并后的 payload hash 与关键路径时长。"""
    module = _worker_module()
    process_id = os.getpid()
    descriptor_sha256 = b"\x06" * 32
    descriptor = SimpleNamespace(sha256=descriptor_sha256)
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points", b"\x00" * 16, descriptor_sha256, 1, 0
    )
    world_spec = SimpleNamespace(world_digest="0" * 64)
    assignments = module._stage4_realtime_shard_assignments()

    def point_value(global_slot: int) -> tuple[int, float, float, float, int, int, int]:
        return (
            module.mid360_offset_time_ns(global_slot),
            1.0,
            2.0,
            3.0,
            100,
            1,
            module.mid360_line_for_slot(
                "livox-mid360-800000-v1", 1, 0, global_slot
            ),
        )

    partials = tuple(
        module._Stage4ShardPrewarm(
            shard_id,
            process_id + shard_id,
            world_spec.world_digest,
            first,
            stop,
            stride,
            count,
            count,
            0,
            1,
            0,
            "lidar_link",
            0,
            identity,
            ((first, point_value(first)),),
            10 + shard_id,
        )
        for shard_id, (first, stop, stride, count) in enumerate(assignments)
    )
    events: list[object] = []

    class Receiver:
        def __init__(self, shard_id: int) -> None:
            first, stop, stride, count = assignments[shard_id]
            self.values = [
                module._Stage4ShardReady(
                    shard_id, process_id + shard_id, first, stop, stride, count, world_spec.world_digest
                ),
                partials[shard_id],
                module._Stage4ShardStopped(shard_id, process_id + shard_id),
            ]

        def recv(self):
            return self.values.pop(0)

        def poll(self, _timeout: float) -> bool:
            return True

        def close(self) -> None:
            return None

    class Sender:
        def send(self, value: object) -> None:
            events.append(value)

        def close(self) -> None:
            return None

    class RequestReceiver:
        def recv(self):
            return module.LidarWorkerStop(1, process_id)

        def close(self) -> None:
            return None

    monkeypatch.setattr(module, "load_v2_descriptor", lambda: descriptor)
    monkeypatch.setattr(module.gc, "collect", lambda: None)
    monkeypatch.setattr(module.gc, "freeze", lambda: None)
    monkeypatch.setattr(module.gc, "disable", lambda: None)
    module.stage4_coordinator_entrypoint(
        RequestReceiver(),
        Sender(),
        world_spec,
        (Sender(), Sender()),
        (Receiver(0), Receiver(1)),
        (process_id, process_id + 1),
    )

    ready = events[0]
    expected = module.pb.LidarPointCloud(
        timebase_ns=0,
        frame_id="lidar_link",
        lidar_id=1,
        sequence=0,
        world_generation=1,
        simulation_session_id=b"\x00" * 16,
        descriptor_sha256=descriptor_sha256,
    )
    for partial in partials:
        for _index, value in partial.values:
            expected.points.add(
                offset_time_ns=value[0], x=value[1], y=value[2], z=value[3],
                reflectivity=value[4], tag=value[5], line=value[6],
            )
    expected.point_num = 2
    assert type(ready) is module.LidarWorkerReady
    assert ready.prewarm_payload_sha256_by_topic == (
        ("lidar_link", hashlib.sha256(expected.SerializeToString(deterministic=True)).hexdigest()),
    )
    assert ready.prewarm_max_scan_wall_duration_ns > max(partial.duration_ns for partial in partials)


@pytest.mark.parametrize(
    ("case", "mutate"),
    (
        ("exact_type", lambda fields: SimpleNamespace(**fields)),
        ("process_id", lambda fields: {**fields, "process_id": fields["process_id"] + 10}),
        ("world_digest", lambda fields: {**fields, "world_digest": "1" * 64}),
        ("shard_id_bool", lambda fields: {**fields, "shard_id": False}),
        ("first", lambda fields: {**fields, "first": fields["first"] + 1}),
        ("first_bool", lambda fields: {**fields, "first": False}),
        ("stop", lambda fields: {**fields, "stop": fields["stop"] - 1}),
        ("stride", lambda fields: {**fields, "stride": 1}),
        ("count", lambda fields: {**fields, "count": fields["count"] - 1}),
        ("examined_count", lambda fields: {**fields, "examined_count": fields["examined_count"] - 1}),
        ("job_id", lambda fields: {**fields, "job_id": 1}),
        ("job_id_bool", lambda fields: {**fields, "job_id": False}),
        ("lifecycle_generation", lambda fields: {**fields, "lifecycle_generation": 2}),
        ("lifecycle_generation_bool", lambda fields: {**fields, "lifecycle_generation": True}),
        ("pause_epoch", lambda fields: {**fields, "pause_epoch": 1}),
        ("pause_epoch_bool", lambda fields: {**fields, "pause_epoch": False}),
        ("topic", lambda fields: {**fields, "topic": "wrong"}),
        ("timestamp_ns", lambda fields: {**fields, "timestamp_ns": 1}),
        ("timestamp_ns_bool", lambda fields: {**fields, "timestamp_ns": False}),
        ("output_identity", lambda fields: {**fields, "output_identity": SimpleNamespace()}),
        ("output_identity_world_generation_bool", None),
        ("output_identity_sequence_bool", None),
        ("point_offset", None),
        ("point_line", None),
        ("duration_ns", lambda fields: {**fields, "duration_ns": 0}),
    ),
)
def test_stage4_coordinator_rejects_invalid_shard_prewarm_before_outer_ready(
    monkeypatch: pytest.MonkeyPatch,
    case: str,
    mutate: object,
) -> None:
    """任一 shard startup partial 不满足固定身份时，outer Ready 必须 fail closed。"""
    module = _worker_module()
    process_id = os.getpid()
    descriptor = importlib.import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points", b"\x00" * 16, descriptor.sha256, 1, 0
    )
    world_spec = SimpleNamespace(world_digest="0" * 64)
    assignments = module._stage4_realtime_shard_assignments()

    def partial_fields(shard_id: int) -> dict[str, object]:
        first, stop, stride, count = assignments[shard_id]
        point_value = (
            module.mid360_offset_time_ns(first),
            1.0,
            2.0,
            3.0,
            100,
            1,
            module.mid360_line_for_slot(
                "livox-mid360-800000-v1", 1, 0, first
            ),
        )
        return {
            "shard_id": shard_id,
            "process_id": process_id + shard_id,
            "world_digest": world_spec.world_digest,
            "first": first,
            "stop": stop,
            "stride": stride,
            "count": count,
            "examined_count": count,
            "job_id": 0,
            "lifecycle_generation": 1,
            "pause_epoch": 0,
            "topic": "lidar_link",
            "timestamp_ns": 0,
            "output_identity": identity,
            "values": ((first, point_value),),
            "duration_ns": 10 + shard_id,
        }

    first_fields = partial_fields(0)
    if case in {
        "output_identity_world_generation_bool",
        "output_identity_sequence_bool",
    }:
        malformed_identity = object.__new__(type(identity))
        for name in (
            "topic",
            "simulation_session_id",
            "descriptor_sha256",
            "world_generation",
            "sequence",
        ):
            value = getattr(identity, name)
            if case == "output_identity_world_generation_bool" and name == "world_generation":
                value = True
            elif case == "output_identity_sequence_bool" and name == "sequence":
                value = False
            object.__setattr__(malformed_identity, name, value)
        mutated = {**first_fields, "output_identity": malformed_identity}
    elif case in {"point_offset", "point_line"}:
        global_slot, point_value = first_fields["values"][0]
        if case == "point_offset":
            point_value = (point_value[0] + 1, *point_value[1:])
        else:
            point_value = (*point_value[:6], 4)
        mutated = {**first_fields, "values": ((global_slot, point_value),)}
    else:
        assert callable(mutate)
        mutated = mutate(first_fields)
    if isinstance(mutated, SimpleNamespace):
        first_partial = mutated
    else:
        first_partial = object.__new__(module._Stage4ShardPrewarm)
        for name, value in mutated.items():
            object.__setattr__(first_partial, name, value)
    second_partial = module._Stage4ShardPrewarm(**partial_fields(1))
    events: list[object] = []

    class Receiver:
        def __init__(self, shard_id: int, partial: object) -> None:
            first, stop, stride, count = assignments[shard_id]
            self.values = [
                module._Stage4ShardReady(
                    shard_id, process_id + shard_id, first, stop, stride, count, world_spec.world_digest
                ),
                partial,
            ]

        def recv(self) -> object:
            return self.values.pop(0)

        def close(self) -> None:
            return None

    class Sender:
        def send(self, value: object) -> None:
            events.append(value)

        def close(self) -> None:
            return None

    class RequestReceiver:
        def recv(self) -> object:
            raise AssertionError(f"invalid partial {case} reached the request loop")

        def close(self) -> None:
            return None

    monkeypatch.setattr(module.gc, "collect", lambda: None)
    monkeypatch.setattr(module.gc, "freeze", lambda: None)
    monkeypatch.setattr(module.gc, "disable", lambda: None)
    with pytest.raises(RuntimeError, match="Stage4 shard prewarm identity mismatch"):
        module.stage4_coordinator_entrypoint(
            RequestReceiver(),
            Sender(),
            world_spec,
            (Sender(), Sender()),
            (Receiver(0, first_partial), Receiver(1, second_partial)),
            (process_id, process_id + 1),
        )
    assert not events


def test_stage4_coordinator_builds_codec_and_disables_gc_before_outer_ready(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Coordinator 必须在 outer Ready 前完成 codec 构造并关闭热路径 GC。"""
    module = _worker_module()
    process_id = os.getpid()
    world_spec = SimpleNamespace(world_digest="0" * 64)

    class RequestReceiver:
        def recv(self):
            return module.LidarWorkerStop(1, process_id)

        def close(self) -> None:
            return None

    class ResponseSender:
        def __init__(self, events: list[object]) -> None:
            self.events = events

        def send(self, value: object) -> None:
            self.events.append(("outer_send", type(value)))

        def close(self) -> None:
            return None

    class ShardSender:
        def __init__(self, shard_id: int, events: list[object]) -> None:
            self.shard_id = shard_id
            self.events = events

        def send(self, value: object) -> None:
            self.events.append(("shard_send", self.shard_id, type(value)))

        def close(self) -> None:
            return None

    class ShardReceiver:
        def __init__(self, shard_id: int, events: list[object]) -> None:
            self.shard_id = shard_id
            self.events = events
            first, stop, stride, count = module._stage4_realtime_shard_assignments()[shard_id]
            self.values = [
                module._Stage4ShardReady(
                    shard_id,
                    process_id + shard_id,
                    first,
                    stop,
                    stride,
                    count,
                    world_spec.world_digest,
                ),
                object(),
                module._Stage4ShardStopped(shard_id, process_id + shard_id),
            ]

        def recv(self) -> object:
            value = self.values.pop(0)
            self.events.append(("shard_recv", self.shard_id, type(value)))
            return value

        def poll(self, _timeout: float) -> bool:
            return True

        def close(self) -> None:
            return None

    def run(codec_factory: object, events: list[object]) -> None:
        monkeypatch.setattr(module, "load_v2_descriptor", lambda: "descriptor")
        monkeypatch.setattr(module, "V2ProtoCodec", codec_factory)
        monkeypatch.setattr(
            module,
            "_stage4_prewarm_payload_from_shards",
            lambda _codec, _partials, _shard_pids, _world_digest: (b"prewarm", 1),
        )
        monkeypatch.setattr(module.gc, "collect", lambda: events.append("collect"))
        monkeypatch.setattr(module.gc, "freeze", lambda: events.append("freeze"))
        monkeypatch.setattr(module.gc, "disable", lambda: events.append("disable"))
        senders = tuple(ShardSender(shard_id, events) for shard_id in (0, 1))
        receivers = tuple(ShardReceiver(shard_id, events) for shard_id in (0, 1))
        module.stage4_coordinator_entrypoint(
            RequestReceiver(),
            ResponseSender(events),
            world_spec,
            senders,
            receivers,
            (process_id, process_id + 1),
        )

    events: list[object] = []

    def codec_factory(descriptor: object) -> object:
        assert descriptor == "descriptor"
        events.append("codec")
        return object()

    run(codec_factory, events)

    assert events[2] == "codec"
    assert events[5:9] == ["collect", "freeze", "disable", ("outer_send", module.LidarWorkerReady)]

    failed_events: list[object] = []

    def failing_codec_factory(_descriptor: object) -> object:
        failed_events.append("codec")
        raise RuntimeError("codec construction failed")

    with pytest.raises(RuntimeError, match="codec construction failed"):
        run(failing_codec_factory, failed_events)

    assert failed_events[2:] == ["codec"]
    assert not any(
        event in {"collect", "freeze", "disable"}
        or event == ("outer_send", module.LidarWorkerReady)
        for event in failed_events
    )


def test_lidar_scan_service_forwards_reserved_v2_identity_to_the_center_request() -> None:
    """父端有界 service 必须原样转发 controller 已预留的 Stage4 LiDAR identity。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    identity = importlib.import_module("slope_sim.interfaces.v2.session").OutputIdentity(
        "/sim/lidar/points",
        b"\x02" * 16,
        descriptor.sha256,
        1,
        4,
    )
    channel = _FakeLidarChannel()
    service = _scan_service(module, channel, _FakeMonotonicNs())

    assert service.capture(
        topic="lidar_link",
        timestamp_ns=100_000_000,
        world_mount_pose=Pose((0.0, 0.0, 0.8), (0.0, 0.0, 0.0, 1.0)),
        optional_base_pose=None,
        complete_obstacle_snapshots_without_body_ids=(),
        output_identity=identity,
    ) is True

    assert channel.sent[0].output_identity == identity


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


def _stage4_golf_worker_world_spec():
    """构造 seed 41 Golf 下唯一中心 LiDAR 的完整 spawn 世界。"""
    module = _worker_module()
    terrain = TerrainSelection("golf_heightfield", golf_seed=41, golf_relief="medium")
    config = ExperimentConfig(
        mode="direct",
        robot_model="df_back",
        terrain_model="golf_heightfield",
        golf_seed=41,
        golf_relief="medium",
    )
    static = ObstacleSpec(
        logical_id=11,
        mode="static",
        geometry=ObstacleGeometry("box", (0.30, 0.30, 0.55)),
        position=(2.0, 0.45, 0.55),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    moving = ObstacleSpec(
        logical_id=12,
        mode="moving",
        geometry=ObstacleGeometry("box", (0.30, 0.30, 0.55)),
        position=(2.2, -0.35, 0.55),
        orientation=(0.0, 0.0, 0.0, 1.0),
        path=ObstaclePath((1.0, -0.35), (4.0, -0.35), 0.4, 0.4, 1),
    )
    document = SceneDocument.from_runtime(
        "df_back",
        terrain,
        (static, moving),
        SensorDocument.default().mounts,
    )
    return module.LidarWorkerWorldSpec(
        1,
        config,
        document,
        module.world_digest_for_document(document),
        "stage4",
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
            "profile",
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
                "output_identity",
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
    """延迟必须严格大于 100 ms 才计错，但有效迟到帧仍要交付。"""
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
    assert service.poll() is not None
    completed = service.snapshot()
    assert completed.in_flight_identity is None
    assert completed.completed_count == 1
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
    assert service.poll() is not None

    assert len(channel.sent) == 2
    assert channel.sent[1].job_id == 2
    events = service.drain_events()
    assert tuple(event.kind for event in events) == (
        "job_overrun",
        "job_overrun",
    )
    assert tuple(event.optional_job_identity for event in events) == (
        (1, 3, 2, "lidar_front", 900_000_000),
        (2, 3, 2, "lidar_rear", 950_000_000),
    )
    assert events[0].stable_error_code == "sensor_overrun"
    assert service.snapshot().overrun_count == 2
    assert service.snapshot().completed_count == 1


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
    assert service.poll() is not None
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


def test_stage4_golf_fifty_frame_budget() -> None:
    """seed 41 Golf 的 spawn 中心 LiDAR 必须连续 50 帧留在 100ms 预算内。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    identity_type = importlib.import_module(
        "slope_sim.interfaces.v2.session"
    ).OutputIdentity
    codec = importlib.import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(
        descriptor
    )
    world_spec = _stage4_golf_worker_world_spec()
    client_id = p.connect(p.DIRECT)
    handle = None
    service = None
    try:
        world, manager = build_world_from_scene_document(
            client_id,
            world_spec.experiment_config,
            world_spec.scene_document,
        )
        backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
        scanner = MultiLineLidar.stage4(
            backend,
            importlib.import_module("slope_sim.lidar_pointcloud").Stage4LidarProfile.realtime(),
        )
        mount = scanner._world_mount()
        snapshots = manager.snapshot(include_body_id=False)
        assert {snapshot.logical_id for snapshot in snapshots} == {11, 12}
        assert all(snapshot.body_id is None for snapshot in snapshots)

        handle = module.start_lidar_worker(world_spec, startup_timeout_sec=15.0)
        service = module.LidarScanService.from_worker_handle(
            handle,
            lifecycle_generation=1,
        )
        observed_tags: set[int] = set()
        for frame_index in range(50):
            timestamp_ns = (frame_index + 1) * 100_000_000
            identity = identity_type(
                "/sim/lidar/points",
                b"\x03" * 16,
                descriptor.sha256,
                1,
                frame_index + 1,
            )
            capture_start_ns = time.monotonic_ns()
            assert service.capture(
                topic="lidar_link",
                timestamp_ns=timestamp_ns,
                world_mount_pose=mount,
                optional_base_pose=None,
                complete_obstacle_snapshots_without_body_ids=snapshots,
                output_identity=identity,
            )
            response = None
            poll_count = 0
            deadline = time.monotonic() + 1.0
            while response is None and time.monotonic() < deadline:
                poll_count += 1
                response = service.poll()
                if response is None:
                    time.sleep(0.001)
            capture_end_ns = time.monotonic_ns()
            if type(response) is not module.PreparedLidarPayload:
                snapshot = service.snapshot()
                events = service.drain_events()

                def process_state(process: object) -> tuple[object, object, object]:
                    return (
                        process.is_alive(),
                        process.exitcode,
                        process.pid,
                    )

                diagnostic_values = {
                    "frame_index": frame_index,
                    "capture_start_ns": capture_start_ns,
                    "capture_end_ns": capture_end_ns,
                    "poll_count": poll_count,
                    "response_type": type(response),
                    "snapshot": tuple(
                        (field.name, getattr(snapshot, field.name))
                        for field in fields(snapshot)
                    ),
                    "events": tuple(
                        tuple((field.name, getattr(event, field.name)) for field in fields(event))
                        for event in events
                    ),
                    "coordinator_process": process_state(handle.process),
                    "shard_processes": tuple(
                        process_state(process)
                        for process in handle._stage4_shard_processes
                    ),
                }
                diagnostic = pformat(diagnostic_values, sort_dicts=False, width=120)
            else:
                diagnostic = None
            assert type(response) is module.PreparedLidarPayload, diagnostic
            frame_snapshot = service.snapshot()
            assert frame_snapshot.max_capture_to_response_ns <= 100_000_000
            decoded = codec.decode_lidar_point_cloud(response.protobuf_payload)
            observed_tags.update(point.tag for point in decoded.points)
            assert service.drain_events() == ()

        snapshot = service.snapshot()
        assert (
            snapshot.completed_count,
            snapshot.failed_count,
            snapshot.overrun_count,
            snapshot.stale_count,
        ) == (50, 0, 0, 0)
        assert snapshot.in_flight_identity is None
        assert snapshot.pending_capture_identity is None
        assert observed_tags >= {1, 2, 3}
    finally:
        if service is not None:
            try:
                service.close_idle()
            except RuntimeError:
                service.force_close()
        elif handle is not None:
            handle.close()
        p.disconnect(client_id)


def test_stage4_golf_raw_first_request_diagnostic() -> None:
    """同一 raw coordinator handle 连续 20 帧必须逐帧留在 100ms 预算内。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    identity_type = importlib.import_module(
        "slope_sim.interfaces.v2.session"
    ).OutputIdentity
    codec = importlib.import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(
        descriptor
    )
    world_spec = _stage4_golf_worker_world_spec()
    client_id = p.connect(p.DIRECT)
    handle = None
    try:
        world, manager = build_world_from_scene_document(
            client_id,
            world_spec.experiment_config,
            world_spec.scene_document,
        )
        backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
        scanner = MultiLineLidar.stage4(
            backend,
            importlib.import_module("slope_sim.lidar_pointcloud").Stage4LidarProfile.realtime(),
        )
        mount = scanner._world_mount()
        snapshots = manager.snapshot(include_body_id=False)
        handle = module.start_lidar_worker(world_spec, startup_timeout_sec=15.0)

        def process_state(process: object) -> tuple[object, object, object]:
            return process.is_alive(), process.exitcode, process.pid

        diagnostics = []
        for frame_index in range(20):
            job_id = frame_index + 1
            timestamp_ns = job_id * 100_000_000
            identity = identity_type(
                "/sim/lidar/points",
                b"\x03" * 16,
                descriptor.sha256,
                1,
                job_id,
            )
            request = module.LidarScanRequest(
                1,
                job_id,
                time.monotonic_ns(),
                1,
                0,
                "lidar_link",
                "lidar_link",
                1,
                timestamp_ns,
                mount,
                None,
                snapshots,
                identity,
            )
            capture_start_ns = time.monotonic_ns()
            handle.request_sender.send(request)
            response_ready = handle.response_receiver.poll(2.0)
            response = handle.response_receiver.recv() if response_ready else None
            capture_end_ns = time.monotonic_ns()
            decoded = (
                codec.decode_lidar_point_cloud(response.protobuf_payload)
                if type(response) is module.PreparedLidarPayload
                else None
            )
            point_fields = (
                None
                if decoded is None
                else tuple(
                    (
                        point.offset_time_ns,
                        point.x,
                        point.y,
                        point.z,
                        point.reflectivity,
                        point.tag,
                        point.line,
                    )
                    for point in decoded.points
                )
            )
            diagnostic = {
                "frame_index": frame_index,
                "capture_start_ns": capture_start_ns,
                "capture_end_ns": capture_end_ns,
                "outer_latency_ns": capture_end_ns - capture_start_ns,
                "response_ready": response_ready,
                "response_type": type(response),
                "payload_point_count": None if decoded is None else decoded.point_num,
                "payload_bytes": (
                    len(response.protobuf_payload)
                    if type(response) is module.PreparedLidarPayload
                    else None
                ),
                "response_scan_wall_duration_ns": getattr(
                    response,
                    "scan_wall_duration_ns",
                    None,
                ),
                "failure_stable_error_code": getattr(
                    response,
                    "stable_error_code",
                    None,
                ),
                "failure_bounded_detail": getattr(
                    response,
                    "bounded_detail",
                    None,
                ),
                "coordinator_process": process_state(handle.process),
                "shard_processes": tuple(
                    process_state(process) for process in handle._stage4_shard_processes
                ),
            }
            diagnostics.append(diagnostic)
            assert response_ready, diagnostic
            assert type(response) is module.PreparedLidarPayload, diagnostic
            assert diagnostic["outer_latency_ns"] < 100_000_000, diagnostic
            assert response.protobuf_payload
            assert decoded.point_num == len(point_fields)
            assert diagnostic["coordinator_process"][:2] == (True, None), diagnostic
            assert all(
                process[:2] == (True, None)
                for process in diagnostic["shard_processes"]
            ), diagnostic
            assert (
                response.job_id,
                response.lifecycle_generation,
                response.pause_epoch,
                response.topic,
                response.timestamp_ns,
            ) == (job_id, 1, 0, "lidar_link", timestamp_ns)
            assert (
                decoded.timebase_ns,
                decoded.sequence,
                decoded.world_generation,
                decoded.simulation_session_id,
                decoded.descriptor_sha256,
            ) == (
                timestamp_ns,
                job_id,
                1,
                b"\x03" * 16,
                descriptor.sha256,
            )
            _assert_stage4_progressive_point_oracle(
                point_fields,
                world_generation=identity.world_generation,
                sequence=identity.sequence,
            )
        print("stage4 raw twenty-frame diagnostic:\n" + pformat(diagnostics, sort_dicts=False))
    finally:
        if handle is not None:
            try:
                handle.close()
            except RuntimeError:
                handle.force_close()
        p.disconnect(client_id)


def test_stage4_direct_shard_boundary_timing_diagnostic() -> None:
    """分离测量正式 shard、父端 merge/encode 与 shard 结果 pickle 边界。"""
    module = _worker_module()
    descriptor = importlib.import_module(
        "slope_sim.interfaces.v2.descriptor"
    ).load_v2_descriptor()
    identity_type = importlib.import_module(
        "slope_sim.interfaces.v2.session"
    ).OutputIdentity
    codec = importlib.import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(
        descriptor
    )
    world_spec = _stage4_golf_worker_world_spec()
    client_id = p.connect(p.DIRECT)
    context = multiprocessing.get_context("spawn")
    request_senders = []
    response_receivers = []
    probe_receivers = []
    processes = []
    try:
        world, manager = build_world_from_scene_document(
            client_id,
            world_spec.experiment_config,
            world_spec.scene_document,
        )
        backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
        scanner = MultiLineLidar.stage4(
            backend,
            importlib.import_module("slope_sim.lidar_pointcloud").Stage4LidarProfile.realtime(),
        )
        mount = scanner._world_mount()
        snapshots = manager.snapshot(include_body_id=False)
        assert {snapshot.logical_id for snapshot in snapshots} == {11, 12}
        assert all(snapshot.body_id is None for snapshot in snapshots)

        assignments = module._stage4_realtime_shard_assignments()
        for shard_id, (first, stop, stride, count) in enumerate(assignments):
            request_receiver, request_sender = context.Pipe(False)
            response_receiver, response_sender = context.Pipe(False)
            probe_receiver, probe_sender = context.Pipe(False)
            process = context.Process(
                target=_stage4_shard_entrypoint_with_gc_probe,
                args=(
                    request_receiver,
                    response_sender,
                    module._Stage4ShardSpec(
                        shard_id,
                        first,
                        stop,
                        stride,
                        count,
                        world_spec,
                    ),
                    probe_sender,
                ),
                daemon=False,
            )
            process.start()
            request_receiver.close()
            response_sender.close()
            probe_sender.close()
            request_senders.append(request_sender)
            response_receivers.append(response_receiver)
            probe_receivers.append(probe_receiver)
            processes.append(process)

        receiver_to_shard = {
            receiver: shard_id for shard_id, receiver in enumerate(response_receivers)
        }

        def receive_exactly_two(timeout_sec: float) -> dict[int, object]:
            deadline = time.monotonic() + timeout_sec
            received = {}
            while len(received) != 2:
                remaining = deadline - time.monotonic()
                assert remaining > 0.0, "timed out waiting for both Stage4 shard responses"
                ready_receivers = wait(
                    tuple(
                        receiver
                        for receiver, shard_id in receiver_to_shard.items()
                        if shard_id not in received
                    ),
                    timeout=remaining,
                )
                assert ready_receivers, "timed out waiting for a Stage4 shard response"
                for receiver in ready_receivers:
                    shard_id = receiver_to_shard[receiver]
                    assert shard_id not in received
                    received[shard_id] = receiver.recv()
            return received

        ready_by_shard = receive_exactly_two(15.0)
        for shard_id, ready in ready_by_shard.items():
            first, stop, stride, count = assignments[shard_id]
            assert type(ready) is module._Stage4ShardReady
            assert (
                ready.shard_id,
                ready.process_id,
                ready.first,
                ready.stop,
                ready.stride,
                ready.count,
                ready.world_digest,
            ) == (
                shard_id,
                processes[shard_id].pid,
                first,
                stop,
                stride,
                count,
                world_spec.world_digest,
            )

        prewarm_by_shard = receive_exactly_two(15.0)
        for shard_id, partial in prewarm_by_shard.items():
            first, stop, stride, count = assignments[shard_id]
            assert type(partial) is module._Stage4ShardPrewarm
            assert (
                partial.shard_id,
                partial.process_id,
                partial.first,
                partial.stop,
                partial.stride,
                partial.count,
                partial.examined_count,
            ) == (
                shard_id,
                processes[shard_id].pid,
                first,
                stop,
                stride,
                count,
                count,
            )

        def resident_set_size_bytes(process: object) -> int:
            process_id = getattr(process, "pid", None)
            assert isinstance(process_id, int) and process_id > 0
            with open(f"/proc/{process_id}/status", encoding="utf-8") as status_file:
                for line in status_file:
                    if line.startswith("VmRSS:"):
                        return int(line.split()[1]) * 1024
            raise AssertionError(f"VmRSS missing for Stage4 shard pid {process_id}")

        rss_after_ready_bytes = tuple(
            resident_set_size_bytes(process) for process in processes
        )

        def point_fields(decoded: object) -> tuple[tuple[object, ...], ...]:
            return tuple(
                (
                    point.offset_time_ns,
                    point.x,
                    point.y,
                    point.z,
                    point.reflectivity,
                    point.tag,
                    point.line,
                )
                for point in decoded.points
            )

        frame_count = 50
        shard_arrivals_ns = {0: [], 1: []}
        shard_intervals = {0: [], 1: []}
        shard_value_counts = {0: [], 1: []}
        merge_encode_ns = []
        merge_encode_cpu_ns = []
        merge_intervals = []
        first_results = None
        import gc

        parent_gc_events: list[tuple[str, int, int | None, int | None, int | None]] = []

        def parent_gc_callback(phase: str, info: dict[str, object]) -> None:
            parent_gc_events.append(
                (
                    phase,
                    time.monotonic_ns(),
                    info.get("generation"),
                    info.get("collected"),
                    info.get("uncollectable"),
                )
            )

        parent_gc_was_enabled = gc.isenabled()
        # 此处只改变 50 帧父端 hot loop，避免把启动和退出阶段纳入实验变量。
        gc.collect()
        gc.freeze()
        gc.disable()
        gc.callbacks.append(parent_gc_callback)
        try:
            for frame_index in range(frame_count):
                job_id = frame_index + 1
                timestamp_ns = job_id * 100_000_000
                request = module.LidarScanRequest(
                    1,
                    job_id,
                    time.monotonic_ns(),
                    1,
                    0,
                    "lidar_link",
                    "lidar_link",
                    1,
                    timestamp_ns,
                    mount,
                    None,
                    snapshots,
                    identity_type(
                        "/sim/lidar/points",
                        b"\x03" * 16,
                        descriptor.sha256,
                        1,
                        job_id,
                    ),
                )
                sent_at_ns = {}
                for shard_id, sender in enumerate(request_senders):
                    sent_at_ns[shard_id] = time.monotonic_ns()
                    sender.send(request)
                received_at_ns = {}
                deadline = time.monotonic() + 2.0
                results = {}
                while len(results) != 2:
                    remaining = deadline - time.monotonic()
                    assert remaining > 0.0, "timed out waiting for a Stage4 shard result"
                    ready_receivers = wait(
                        tuple(
                            receiver
                            for receiver, shard_id in receiver_to_shard.items()
                            if shard_id not in results
                        ),
                        timeout=remaining,
                    )
                    assert ready_receivers, "timed out waiting for a Stage4 shard result"
                    for receiver in ready_receivers:
                        shard_id = receiver_to_shard[receiver]
                        assert shard_id not in results
                        results[shard_id] = receiver.recv()
                        received_at_ns[shard_id] = time.monotonic_ns()
                ordered_results = tuple(results[shard_id] for shard_id in (0, 1))
                for shard_id, result in enumerate(ordered_results):
                    first, stop, stride, count = assignments[shard_id]
                    assert type(result) is module._Stage4ShardResult
                    assert (
                        result.shard_id,
                        result.first,
                        result.stop,
                        result.stride,
                        result.count,
                        result.examined_count,
                        result.job_id,
                        result.lifecycle_generation,
                        result.pause_epoch,
                        result.topic,
                        result.timestamp_ns,
                    ) == (
                        shard_id,
                        first,
                        stop,
                        stride,
                        count,
                        count,
                        job_id,
                        1,
                        0,
                        "lidar_link",
                        timestamp_ns,
                    )
                for shard_id in (0, 1):
                    shard_arrivals_ns[shard_id].append(
                        received_at_ns[shard_id] - sent_at_ns[shard_id]
                    )
                shard_intervals[0].append((frame_index, sent_at_ns[0], received_at_ns[0]))
                shard_intervals[1].append((frame_index, sent_at_ns[1], received_at_ns[1]))
                for shard_id, result in enumerate(ordered_results):
                    shard_value_counts[shard_id].append(len(result.values))
                merge_started_ns = time.monotonic_ns()
                merge_started_cpu_ns = time.process_time_ns()
                payload = module._stage4_payload_from_shards(request, codec, ordered_results)
                merge_ended_cpu_ns = time.process_time_ns()
                merge_ended_ns = time.monotonic_ns()
                merge_encode_ns.append(merge_ended_ns - merge_started_ns)
                merge_encode_cpu_ns.append(merge_ended_cpu_ns - merge_started_cpu_ns)
                merge_intervals.append((frame_index, merge_started_ns, merge_ended_ns))
                assert type(payload) is module.PreparedLidarPayload
                decoded = codec.decode_lidar_point_cloud(payload.protobuf_payload)
                assert decoded.point_num == len(ordered_results[0].values) + len(ordered_results[1].values)
                current_point_fields = point_fields(decoded)
                globally_sorted_wire_values = tuple(
                    (
                        value[0],
                        *(struct.unpack("<f", struct.pack("<f", coordinate))[0] for coordinate in value[1:4]),
                        *value[4:],
                    )
                    for _global_slot, value in sorted(
                        (*ordered_results[0].values, *ordered_results[1].values),
                        key=lambda item: item[0],
                    )
                )
                assert current_point_fields == globally_sorted_wire_values
                _assert_stage4_progressive_point_oracle(
                    current_point_fields,
                    world_generation=request.output_identity.world_generation,
                    sequence=request.output_identity.sequence,
                )
                if first_results is None:
                    first_results = ordered_results
        finally:
            gc.callbacks.remove(parent_gc_callback)
            if parent_gc_was_enabled:
                gc.enable()
            else:
                gc.disable()

        assert first_results is not None
        rss_after_50_frames_bytes = tuple(
            resident_set_size_bytes(process) for process in processes
        )
        rss_growth_bytes = tuple(
            after - before
            for before, after in zip(
                rss_after_ready_bytes,
                rss_after_50_frames_bytes,
                strict=True,
            )
        )
        # 50 帧诊断不要求零增长，但不得出现无界的 child 常驻内存积累。
        assert all(growth < 64 * 1024 * 1024 for growth in rss_growth_bytes)

        def summarize(durations_ns: list[int]) -> dict[str, int | float | tuple[int, ...]]:
            ordered = tuple(sorted(durations_ns))
            return {
                "samples_ns": tuple(durations_ns),
                "median_ns": median(durations_ns),
                "p95_ns": ordered[(len(ordered) * 95 + 99) // 100 - 1],
                "max_ns": ordered[-1],
            }

        pickle_metrics = {
            shard_id: {"dumps_ns": [], "loads_ns": [], "bytes": []}
            for shard_id in (0, 1)
        }
        for _sample_index in range(20):
            for shard_id, result in enumerate(first_results):
                dumps_started_ns = time.perf_counter_ns()
                encoded_result = ForkingPickler.dumps(result)
                pickle_metrics[shard_id]["dumps_ns"].append(
                    time.perf_counter_ns() - dumps_started_ns
                )
                loads_started_ns = time.perf_counter_ns()
                decoded_result = ForkingPickler.loads(encoded_result)
                pickle_metrics[shard_id]["loads_ns"].append(
                    time.perf_counter_ns() - loads_started_ns
                )
                pickle_metrics[shard_id]["bytes"].append(len(encoded_result))
                assert decoded_result == result

        for shard_id, sender in enumerate(request_senders):
            sender.send(module.LidarWorkerStop(1, processes[shard_id].pid))
        stopped_by_shard = receive_exactly_two(2.0)
        for shard_id, stopped in stopped_by_shard.items():
            assert type(stopped) is module._Stage4ShardStopped
            assert (stopped.shard_id, stopped.process_id) == (
                shard_id,
                processes[shard_id].pid,
            )
        shard_gc_events = {}
        for shard_id, receiver in enumerate(probe_receivers):
            assert receiver.poll(2.0), "timed out waiting for the Stage4 shard GC probe"
            probe = receiver.recv()
            assert type(probe) is dict
            assert probe["freeze_calls"] == 1
            assert probe["gc_enabled_after_freeze"] is False
            events = probe["hot_loop_gc_events"]
            assert type(events) is tuple
            assert not events
            expected_thread_count = (2, 2)[shard_id]
            thread_counts = probe["hot_loop_thread_counts"]
            assert thread_counts[0] == expected_thread_count
            assert thread_counts[1:] == (expected_thread_count,) * frame_count
            assert len(thread_counts) == frame_count + 1
            shard_gc_events[shard_id] = events
        for sender in request_senders:
            sender.close()
        for receiver in response_receivers:
            receiver.close()
        for receiver in probe_receivers:
            receiver.close()
        for process in processes:
            process.join(5.0)
            assert not process.is_alive()
            assert process.exitcode == 0

        def gc_events_in_interval(
            events: tuple[tuple[str, int, int | None, int | None, int | None], ...]
            | list[tuple[str, int, int | None, int | None, int | None]],
            started_ns: int,
            ended_ns: int,
        ) -> tuple[tuple[str, int, int | None, int | None, int | None], ...]:
            return tuple(event for event in events if started_ns <= event[1] <= ended_ns)

        shard_outliers = {
            shard_id: tuple(
                {
                    "frame_index": frame_index,
                    "wall_ns": ended_ns - started_ns,
                    "gc_events": gc_events_in_interval(
                        shard_gc_events[shard_id],
                        started_ns,
                        ended_ns,
                    ),
                }
                for frame_index, started_ns, ended_ns in shard_intervals[shard_id]
                if ended_ns - started_ns > 100_000_000
            )
            for shard_id in (0, 1)
        }
        merge_outliers = tuple(
            {
                "frame_index": frame_index,
                "wall_ns": ended_ns - started_ns,
                "cpu_ns": merge_encode_cpu_ns[frame_index],
                "gc_events": gc_events_in_interval(
                    parent_gc_events,
                    started_ns,
                    ended_ns,
                ),
            }
            for frame_index, started_ns, ended_ns in merge_intervals
            if ended_ns - started_ns > 50_000_000
        )
        critical_path_estimates_ns = [
            max(shard_arrivals_ns[0][frame_index], shard_arrivals_ns[1][frame_index])
            + merge_ns
            for frame_index, merge_ns in enumerate(
                merge_encode_ns,
            )
        ]

        diagnostic = {
            "shard_arrival_ns": {
                shard_id: summarize(shard_arrivals_ns[shard_id])
                for shard_id in (0, 1)
            },
            "shard_value_counts": {
                shard_id: tuple(shard_value_counts[shard_id]) for shard_id in (0, 1)
            },
            "merge_encode_ns": summarize(merge_encode_ns),
            "merge_encode_cpu_ns": summarize(merge_encode_cpu_ns),
            "critical_path_estimate_ns": summarize(critical_path_estimates_ns),
            "shard_outliers_over_100ms": shard_outliers,
            "merge_outliers_over_50ms": merge_outliers,
            "parent_hot_loop_gc_callback_count": len(parent_gc_events),
            "shard_hot_loop_gc_callback_counts": tuple(
                len(shard_gc_events[shard_id]) for shard_id in (0, 1)
            ),
            "rss_bytes": {
                "after_ready": rss_after_ready_bytes,
                "after_50_frames": rss_after_50_frames_bytes,
                "growth": rss_growth_bytes,
                "growth_bound": 64 * 1024 * 1024,
            },
            "forking_pickle": {
                shard_id: {
                    "dumps_ns": summarize(metrics["dumps_ns"]),
                    "loads_ns": summarize(metrics["loads_ns"]),
                    "bytes": tuple(metrics["bytes"]),
                }
                for shard_id, metrics in pickle_metrics.items()
            },
            "process_exitcodes": tuple(process.exitcode for process in processes),
        }
        print("stage4 direct shard boundary timing diagnostic:\n" + pformat(diagnostic, sort_dicts=False))
        assert max(
            max(shard_arrivals_ns[shard_id]) for shard_id in (0, 1)
        ) < 80_000_000, diagnostic
        assert max(merge_encode_ns) < 25_000_000, diagnostic
        assert max(critical_path_estimates_ns) < 95_000_000, diagnostic
    finally:
        for sender in request_senders:
            try:
                sender.close()
            except OSError:
                pass
        for receiver in response_receivers:
            try:
                receiver.close()
            except OSError:
                pass
        for receiver in probe_receivers:
            try:
                receiver.close()
            except OSError:
                pass
        for process in processes:
            if process.is_alive():
                process.terminate()
                process.join(2.0)
        p.disconnect(client_id)


def test_stage4_golf_direct_interleaved_thread_component_diagnostic() -> None:
    """测量交错 Stage4 shard 的双线程组件，并与单线程结果逐值比较。"""
    module = _worker_module()
    world_spec = _stage4_golf_worker_world_spec()
    client_id = p.connect(p.DIRECT)
    try:
        world, manager = build_world_from_scene_document(
            client_id,
            world_spec.experiment_config,
            world_spec.scene_document,
        )
        backend = PyBulletSensorBackend(client_id, world.active_robot.robot.robot_id)
        scanner = MultiLineLidar.stage4(
            backend,
            importlib.import_module("slope_sim.lidar_pointcloud").Stage4LidarProfile.realtime(),
        )
        mount = scanner._world_mount()
        snapshots = manager.snapshot(include_body_id=False)
        assert {snapshot.logical_id for snapshot in snapshots} == {11, 12}
        assert all(snapshot.body_id is None for snapshot in snapshots)
        def scan_component(
            first: int,
            stop: int,
            stride: int,
            count: int,
            num_threads: int,
        ) -> tuple[
            tuple[
                tuple[int, ...],
                tuple[tuple[int, float, float, float, int, int, int], ...],
            ],
            int,
            int,
        ]:
            global_slots = range(first, stop, stride)
            shard_starts, shard_ends = scanner._stage4_world_rays_for_slots(
                mount,
                pattern_version="livox-mid360-800000-v1",
                world_generation=1,
                sequence=0,
                global_slots=global_slots,
            )
            assert len(shard_starts) == len(shard_ends) == count
            assert shard_starts.flags.c_contiguous and shard_ends.flags.c_contiguous
            assert not shard_starts.flags.writeable and not shard_ends.flags.writeable
            ray_start_ns = time.perf_counter_ns()
            indexed_hits = backend._ray_test_indexed_hits_ndarray(
                shard_starts,
                shard_ends,
                collision_mask=0x10,
                num_threads=num_threads,
            )
            ray_end_ns = time.perf_counter_ns()
            world_points = tuple(hit.hit_position for _index, hit in indexed_hits)
            local_points = backend.inverse_transform_points_prevalidated(
                mount,
                world_points,
            )
            point_values = tuple(
                (first + local_index * stride, point_value)
                for (local_index, hit), local_point in zip(
                    indexed_hits,
                    local_points,
                    strict=True,
                )
                if (
                    point_value := scanner._stage4_point_values_from_hit(
                        first + local_index * stride,
                        hit,
                        local_point,
                        pattern_version="livox-mid360-800000-v1",
                        world_generation=1,
                        sequence=0,
                    )
                ) is not None
            )
            component_end_ns = time.perf_counter_ns()
            return (
                (
                    tuple(first + index * stride for index, _hit in indexed_hits),
                    point_values,
                ),
                ray_end_ns - ray_start_ns,
                component_end_ns - ray_end_ns,
            )

        def summarize(durations_ns: list[int]) -> dict[str, int | float]:
            ordered = tuple(sorted(durations_ns))
            return {
                "count": len(durations_ns),
                "median_ns": median(durations_ns),
                "p95_ns": ordered[(len(ordered) * 95 + 99) // 100 - 1],
                "max_ns": ordered[-1],
            }

        load_before = os.getloadavg()
        assignments = module._stage4_realtime_shard_assignments()
        samples = {
            shard_id: {"ray_test_batch_ns": [], "inverse_and_point_ns": []}
            for shard_id in (0, 1)
        }
        oracle_by_shard = {}
        for shard_id, assignment in enumerate(assignments):
            single_thread_oracle = scan_component(*assignment, 1)[0]
            warmed_values, _ray_ns, _component_ns = scan_component(*assignment, 2)
            assert warmed_values == single_thread_oracle
            oracle_by_shard[shard_id] = single_thread_oracle
            for _sample_index in range(12):
                single_thread_values, _ray_ns, _component_ns = scan_component(
                    *assignment,
                    1,
                )
                observed_values, ray_ns, component_ns = scan_component(*assignment, 2)
                assert single_thread_values == single_thread_oracle
                assert observed_values == single_thread_values
                samples[shard_id]["ray_test_batch_ns"].append(ray_ns)
                samples[shard_id]["inverse_and_point_ns"].append(component_ns)
        load_after = os.getloadavg()

        diagnostic = {
            "shard_assignments": assignments,
            "shard_thread_budget": {0: 2, 1: 2, "total": 4},
            "global_hit_count": {
                shard_id: len(oracle_by_shard[shard_id][0]) for shard_id in (0, 1)
            },
            "global_point_count": {
                shard_id: len(oracle_by_shard[shard_id][1]) for shard_id in (0, 1)
            },
            "ray_test_batch_ns": {
                shard_id: summarize(samples[shard_id]["ray_test_batch_ns"])
                for shard_id in (0, 1)
            },
            "inverse_and_point_ns": {
                shard_id: summarize(samples[shard_id]["inverse_and_point_ns"])
                for shard_id in (0, 1)
            },
            "system_load_before": load_before,
            "system_load_after": load_after,
        }
        print(
            "stage4 direct interleaved num_threads=2 diagnostic:\n"
            + pformat(diagnostic, sort_dicts=False)
        )
        assert all(
            len(samples[shard_id][metric_name]) == 12
            for shard_id in (0, 1)
            for metric_name in ("ray_test_batch_ns", "inverse_and_point_ns")
        )
    finally:
        p.disconnect(client_id)


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
    """异常消息需保留诊断，同时生成至多 512 bytes 的合法单行 detail。"""
    module = _worker_module()
    error_type = type("多字节\n异常" * 100, (RuntimeError,), {})

    diagnostic = module._bounded_exception_detail(
        "Stage4 shard scan",
        RuntimeError("ray batch\nrejected"),
    )
    detail = module._bounded_exception_detail(
        "front\npreflight",
        error_type("bounded payload"),
    )

    assert diagnostic == "Stage4 shard scan failed: RuntimeError: ray batch rejected"
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
