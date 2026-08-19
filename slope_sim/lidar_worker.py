# LiDAR 异步服务：负责父端有界调度、子进程握手、冻结扫描和单次预编码。
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, fields
from functools import wraps
import gc
import hashlib
import json
import math
from multiprocessing.connection import Connection
import multiprocessing
import os
from threading import RLock
import time

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import build_world_from_scene_document
from slope_sim.interfaces.codec import ProtoCodec
from slope_sim.interfaces.dashboard_snapshot import LidarTopViewFrame, LidarTopViewPoint
from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb
from slope_sim.interfaces.models import LidarPoint, LidarPointCloud
from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
from slope_sim.interfaces.v2.models import require_fixed_bytes
from slope_sim.interfaces.v2.session import OutputIdentity
from slope_sim.lidar_pointcloud import (
    LIDAR_SCAN_PERIOD_NS,
    MID360_PATTERN_VERSION,
    LidarScanResult,
    MultiLineLidar,
    Stage4LidarProfile,
    _V2PayloadSerializationError,
    mid360_line_for_slot,
    mid360_offset_time_ns,
)
from slope_sim.obstacles import (
    ObstacleGeometry,
    ObstaclePath,
    ObstacleSnapshot,
    ObstacleSpec,
    _create_obstacle_body,
    _remove_committed_body_strict,
    update_kinematic_obstacle,
)
from slope_sim.scene_config import SceneDocument, document_to_mapping, scene_document_from_mapping
from slope_sim.scene import LIDAR_VISIBLE_GROUP
from slope_sim.sensor_backend import Pose, PyBulletSensorBackend, RayHit, Vec3


_PROTOCOL_VERSION = 1
_UINT64_MAX = (1 << 64) - 1


def _stage4_realtime_shard_assignments() -> tuple[
    tuple[int, int, int, int], tuple[int, int, int, int]
]:
    """返回 Stage4 5,760 条实时射线固定的 even/odd 私有分配。"""
    return ((0, 5760, 2, 2880), (1, 5760, 2, 2880))


def _stage4_realtime_shard_thread_count(shard_id: int) -> int:
    """两个均衡的交错 shard 各固定使用两个 Bullet ray 线程。"""
    if type(shard_id) is not int or not 0 <= shard_id < 2:
        raise ValueError("invalid Stage4 shard id")
    return (2, 2)[shard_id]


def _require_stage4_assignment(
    shard_id: object, first: object, stop: object, stride: object, count: object
) -> tuple[int, int, int, int, int]:
    """把私有 shard assignment 收窄为冻结 even/odd 合同。"""
    assignments = _stage4_realtime_shard_assignments()
    if (
        type(shard_id) is not int
        or not 0 <= shard_id < len(assignments)
        or type(first) is not int
        or type(stop) is not int
        or type(stride) is not int
        or type(count) is not int
        or (first, stop, stride, count) != assignments[shard_id]
    ):
        raise ValueError("invalid Stage4 shard assignment")
    return shard_id, first, stop, stride, count


def _require_stage4_indexed_values(
    values: object, first: int, stop: int, stride: int
) -> tuple[tuple[int, tuple[int, float, float, float, int, int, int]], ...]:
    """验证紧凑命中仍严格属于 shard 的交错 global index 集合。"""
    if type(values) is not tuple:
        raise ValueError("Stage4 shard values must be an exact tuple")
    validated = []
    previous_index = first - stride
    for raw_value in values:
        if type(raw_value) is not tuple or len(raw_value) != 2:
            raise ValueError("Stage4 shard indexed hit has an invalid shape")
        ray_index, point_value = raw_value
        if (
            type(ray_index) is not int
            or not first <= ray_index < stop
            or (ray_index - first) % stride != 0
            or ray_index <= previous_index
            or type(point_value) is not tuple
            or len(point_value) != 7
        ):
            raise ValueError("Stage4 shard indexed hit is outside its assigned range")
        offset_time_ns, x, y, z, reflectivity, tag, line = point_value
        if (
            type(offset_time_ns) is not int
            or type(x) is not float
            or not math.isfinite(x)
            or type(y) is not float
            or not math.isfinite(y)
            or type(z) is not float
            or not math.isfinite(z)
            or type(reflectivity) is not int
            or type(tag) is not int
            or type(line) is not int
            or not 0 <= offset_time_ns <= 0xFFFFFFFF
            or not 0 <= reflectivity <= 0xFFFFFFFF
            or not 0 <= tag <= 3
            or not 0 <= line <= 15
        ):
            raise ValueError("Stage4 shard indexed hit is outside its assigned range")
        previous_index = ray_index
        validated.append((ray_index, point_value))
    return tuple(validated)


def _require_stage4_firing_identity(
    values: tuple[tuple[int, tuple[int, float, float, float, int, int, int]], ...],
    *,
    world_generation: int,
    sequence: int,
) -> None:
    """按 request identity 精确核验每个紧凑命中的 firing 时间与四线编号。"""
    if not values:
        return
    first_line = mid360_line_for_slot(
        MID360_PATTERN_VERSION,
        world_generation,
        sequence,
        0,
    )
    firing_slot_count = Stage4LidarProfile.realtime().firing_slot_count
    for global_slot, point_value in values:
        if (
            point_value[0]
            != global_slot * LIDAR_SCAN_PERIOD_NS // firing_slot_count
            or point_value[6] != (first_line + global_slot) % 4
        ):
            raise ValueError("Stage4 shard point firing identity mismatch")


def _merge_stage4_shard_indexed_values(
    raw_shards: object,
) -> tuple[tuple[int, float, float, float, int, int, int], ...]:
    """验证冻结双 shard 覆盖后，线性归并接收边界已核验的紧凑点。"""
    if type(raw_shards) is not tuple or len(raw_shards) != 2:
        raise RuntimeError("Stage4 coordinator requires exactly two shard results")
    assignments = _stage4_realtime_shard_assignments()
    indexed_by_shard: list[
        tuple[tuple[int, tuple[int, float, float, float, int, int, int]], ...] | None
    ] = [None, None]
    seen_shards: set[int] = set()
    for raw_shard in raw_shards:
        if type(raw_shard) is not tuple or len(raw_shard) != 7:
            raise RuntimeError("Stage4 shard result has an invalid shape")
        shard_id, first, stop, stride, count, examined_count, raw_values = raw_shard
        if (
            type(shard_id) is not int
            or not 0 <= shard_id < len(assignments)
            or shard_id in seen_shards
            or (first, stop, stride, count) != assignments[shard_id]
            or type(examined_count) is not int
            or examined_count != count
            or type(raw_values) is not tuple
        ):
            raise RuntimeError("Stage4 shard result does not prove its assigned coverage")
        seen_shards.add(shard_id)
        indexed_by_shard[shard_id] = raw_values
    if seen_shards != {0, 1}:
        raise RuntimeError("Stage4 shard results do not cover both assigned ranges")
    even_values, odd_values = indexed_by_shard
    if even_values is None or odd_values is None:
        raise RuntimeError("Stage4 shard results do not cover both assigned ranges")
    merged = []
    even_index = 0
    odd_index = 0
    while even_index < len(even_values) and odd_index < len(odd_values):
        if even_values[even_index][0] < odd_values[odd_index][0]:
            merged.append(even_values[even_index][1])
            even_index += 1
        else:
            merged.append(odd_values[odd_index][1])
            odd_index += 1
    while even_index < len(even_values):
        merged.append(even_values[even_index][1])
        even_index += 1
    while odd_index < len(odd_values):
        merged.append(odd_values[odd_index][1])
        odd_index += 1
    return tuple(merged)


@dataclass(frozen=True, slots=True)
class _Stage4ShardSpec:
    """仅在父进程创建的 Stage4 sibling shard 间传递的冻结世界分配。"""

    shard_id: int
    first: int
    stop: int
    stride: int
    count: int
    world_spec: LidarWorkerWorldSpec

    def __post_init__(self) -> None:
        _require_stage4_assignment(self.shard_id, self.first, self.stop, self.stride, self.count)
        if type(self.world_spec) is not LidarWorkerWorldSpec or self.world_spec.profile != "stage4":
            raise ValueError("invalid Stage4 shard specification")


@dataclass(frozen=True, slots=True)
class _Stage4ShardReady:
    shard_id: int
    process_id: int
    first: int
    stop: int
    stride: int
    count: int
    world_digest: str

    def __post_init__(self) -> None:
        _require_stage4_assignment(self.shard_id, self.first, self.stop, self.stride, self.count)
        _require_positive_uint64("process_id", self.process_id)
        _require_sha256("world_digest", self.world_digest)


@dataclass(frozen=True, slots=True)
class _Stage4ShardResult:
    shard_id: int
    process_id: int
    first: int
    stop: int
    stride: int
    count: int
    examined_count: int
    job_id: int
    lifecycle_generation: int
    pause_epoch: int
    topic: str
    timestamp_ns: int
    values: tuple[tuple[int, tuple[int, float, float, float, int, int, int]], ...]

    def __post_init__(self) -> None:
        _require_stage4_assignment(self.shard_id, self.first, self.stop, self.stride, self.count)
        _require_positive_uint64("process_id", self.process_id)
        if type(self.examined_count) is not int or self.examined_count != self.count:
            raise ValueError("invalid Stage4 shard examined count")
        _require_job_identity((self.job_id, self.lifecycle_generation, self.pause_epoch, self.topic, self.timestamp_ns))


@dataclass(frozen=True, slots=True)
class _Stage4ShardPrewarm:
    """shard 自己的真实 assignment 预热结果，供 coordinator 在 outer Ready 前验证。"""

    shard_id: int
    process_id: int
    world_digest: str
    first: int
    stop: int
    stride: int
    count: int
    examined_count: int
    job_id: int
    lifecycle_generation: int
    pause_epoch: int
    topic: str
    timestamp_ns: int
    output_identity: OutputIdentity
    values: tuple[tuple[int, tuple[int, float, float, float, int, int, int]], ...]
    duration_ns: int

    def __post_init__(self) -> None:
        _require_stage4_assignment(self.shard_id, self.first, self.stop, self.stride, self.count)
        _require_positive_uint64("process_id", self.process_id)
        _require_sha256("world_digest", self.world_digest)
        if type(self.examined_count) is not int or self.examined_count != self.count:
            raise ValueError("invalid Stage4 shard examined count")
        if (
            type(self.job_id) is not int
            or type(self.lifecycle_generation) is not int
            or type(self.pause_epoch) is not int
            or type(self.topic) is not str
            or type(self.timestamp_ns) is not int
            or (self.job_id, self.lifecycle_generation, self.pause_epoch, self.topic, self.timestamp_ns)
            != (0, 1, 0, "lidar_link", 0)
        ):
            raise ValueError("invalid Stage4 shard prewarm identity")
        if type(self.duration_ns) is not int or self.duration_ns <= 0:
            raise ValueError("invalid Stage4 shard prewarm duration")


@dataclass(frozen=True, slots=True)
class _Stage4ShardStopped:
    shard_id: int
    process_id: int

    def __post_init__(self) -> None:
        if type(self.shard_id) is not int or self.shard_id not in {0, 1}:
            raise ValueError("invalid Stage4 shard id")
        _require_positive_uint64("process_id", self.process_id)


@dataclass(frozen=True, slots=True)
class _Stage4ShardFailure:
    """shard 私有扫描错误，供 coordinator 按整帧收口。"""

    shard_id: int
    process_id: int
    first: int
    stop: int
    stride: int
    count: int
    job_id: int
    lifecycle_generation: int
    pause_epoch: int
    topic: str
    timestamp_ns: int
    stable_error_code: str
    bounded_detail: str

    def __post_init__(self) -> None:
        _require_stage4_assignment(self.shard_id, self.first, self.stop, self.stride, self.count)
        _require_positive_uint64("process_id", self.process_id)
        _require_job_identity((self.job_id, self.lifecycle_generation, self.pause_epoch, self.topic, self.timestamp_ns))
        if self.stable_error_code != "shard_scan_failed":
            raise ValueError("invalid Stage4 shard failure code")
        _require_bounded_detail(self.bounded_detail)
_PROCESS_ESCALATION_TIMEOUT_SEC = 1.0
_LIDAR_JOB_BUDGET_NS = 100_000_000
_TOPICS = ("lidar_front", "lidar_rear")
_STAGE4_TOPICS = ("lidar_link",)
_WORKER_TOPICS_BY_PROFILE = {
    "stage3": _TOPICS,
    "stage4": _STAGE4_TOPICS,
}
_STARTUP_PHASE_CODES = {
    "world_build": "worker_preflight_failed",
    "front_preflight": "worker_preflight_failed",
    "rear_preflight": "worker_preflight_failed",
    "center_preflight": "worker_preflight_failed",
    "startup_cleanup": "worker_start_failed",
}
_STABLE_ERROR_CODES = frozenset(
    {
        "scene_reconcile_failed",
        "scene_state_unknown",
        "raycast_failed",
        "pointcloud_failed",
        "codec_failed",
        "worker_start_failed",
        "worker_preflight_failed",
        "worker_protocol_failed",
        "worker_exited",
        "sensor_overrun",
        "worker_shutdown_failed",
    }
)
_LIDAR_IDENTITIES = {
    "lidar_front": ("lidar_front", 1),
    "lidar_rear": ("lidar_rear", 2),
    # 阶段四独立 profile 使用唯一中心 LiDAR，不与阶段三前后双雷达混用。
    "lidar_link": ("lidar_link", 1),
}
_SERVICE_STATES = frozenset(
    {"starting", "ready", "suspended", "draining", "failed", "closed"}
)
_SERVICE_EVENT_KINDS = frozenset(
    {
        "frame_failed",
        "capture_rejected",
        "job_overrun",
        "service_failed",
        "retired_cleanup_failed",
    }
)
_FRAME_ERROR_CODES = frozenset(
    {
        "sensor_overrun",
        "scene_reconcile_failed",
        "scene_state_unknown",
        "raycast_failed",
        "pointcloud_failed",
        "codec_failed",
    }
)
_SERVICE_ERROR_CODES = frozenset(
    {
        "scene_state_unknown",
        "worker_start_failed",
        "worker_preflight_failed",
        "worker_protocol_failed",
        "worker_exited",
        "worker_shutdown_failed",
    }
)


def _require_uint64(name: str, value: object) -> int:
    """严格校验 IPC 中不允许 bool 混入的无符号 64 位整数。"""
    if type(value) is not int:
        raise ValueError(f"{name} must be a uint64")
    if not 0 <= value <= _UINT64_MAX:
        raise ValueError(f"{name} must be a uint64")
    return value


def _require_positive_uint64(name: str, value: object) -> int:
    validated = _require_uint64(name, value)
    if validated == 0:
        raise ValueError(f"{name} must be positive")
    return validated


def _require_protocol_version(value: object) -> int:
    version = _require_uint64("protocol_version", value)
    if version != _PROTOCOL_VERSION:
        raise ValueError("unsupported lidar worker protocol version")
    return version


def _require_sha256(name: str, value: object) -> str:
    if type(value) is not str or len(value) != 64:
        raise ValueError(f"{name} must be a lowercase SHA-256 hex digest")
    if any(character not in "0123456789abcdef" for character in value):
        raise ValueError(f"{name} must be a lowercase SHA-256 hex digest")
    return value


def _require_bounded_detail(value: object) -> str:
    if type(value) is not str or "\n" in value or "\r" in value:
        raise ValueError("bounded_detail must be a single-line UTF-8 string")
    if len(value.encode("utf-8", errors="strict")) > 512:
        raise ValueError("bounded_detail must be at most 512 UTF-8 bytes")
    return value


def _require_lidar_identity(
    topic: object,
    *,
    frame_id: object | None = None,
    lidar_id: object | None = None,
) -> tuple[str, str, int]:
    """把内部 topic、企业 frame 和 lidar id 约束为已登记的精确组合。"""
    if type(topic) is not str or topic not in _LIDAR_IDENTITIES:
        raise ValueError("topic must identify a configured lidar")
    expected_frame, expected_lidar_id = _LIDAR_IDENTITIES[topic]
    selected_frame = expected_frame if frame_id is None else frame_id
    selected_lidar_id = expected_lidar_id if lidar_id is None else lidar_id
    if type(selected_frame) is not str or selected_frame != expected_frame:
        raise ValueError("frame_id must match topic")
    if type(selected_lidar_id) is not int or selected_lidar_id != expected_lidar_id:
        raise ValueError("lidar_id must match topic")
    return topic, expected_frame, expected_lidar_id


def _reconstruct_v2_output_identity(value: object) -> OutputIdentity | None:
    """跨 IPC 重新校验 Stage4 预留的唯一 LiDAR 输出身份。"""
    if value is None:
        return None
    if type(value) is not OutputIdentity:
        raise ValueError("output_identity must be an exact OutputIdentity or None")
    if value.topic != "/sim/lidar/points":
        raise ValueError("output_identity must reserve the v2 lidar output topic")
    session_id = require_fixed_bytes(
        "output_identity.simulation_session_id",
        value.simulation_session_id,
        16,
    )
    descriptor_sha256 = require_fixed_bytes(
        "output_identity.descriptor_sha256",
        value.descriptor_sha256,
        32,
    )
    world_generation = _require_uint64(
        "output_identity.world_generation",
        value.world_generation,
    )
    sequence = _require_uint64("output_identity.sequence", value.sequence)
    return OutputIdentity(
        "/sim/lidar/points",
        session_id,
        descriptor_sha256,
        world_generation,
        sequence,
    )


def _require_job_identity(value: object) -> tuple[int, int, int, str, int]:
    """校验事件和快照共享的已发送 job 身份。"""
    if type(value) is not tuple or len(value) != 5:
        raise ValueError("job identity must be an exact five-field tuple")
    job_id = _require_positive_uint64("job identity job_id", value[0])
    generation = _require_uint64("job identity lifecycle_generation", value[1])
    pause_epoch = _require_uint64("job identity pause_epoch", value[2])
    topic, _frame_id, _lidar_id = _require_lidar_identity(value[3])
    timestamp_ns = _require_uint64("job identity timestamp_ns", value[4])
    return (job_id, generation, pause_epoch, topic, timestamp_ns)


def _require_capture_identity(value: object) -> tuple[int, int, str, int]:
    """校验尚未分配 job id 的父端 pending capture 身份。"""
    if type(value) is not tuple or len(value) != 4:
        raise ValueError("capture identity must be an exact four-field tuple")
    generation = _require_uint64("capture identity lifecycle_generation", value[0])
    pause_epoch = _require_uint64("capture identity pause_epoch", value[1])
    topic, _frame_id, _lidar_id = _require_lidar_identity(value[2])
    timestamp_ns = _require_uint64("capture identity timestamp_ns", value[3])
    return (generation, pause_epoch, topic, timestamp_ns)


def _reconstruct_pose(name: str, value: object) -> Pose:
    if type(value) is not Pose:
        raise ValueError(f"{name} must be an exact Pose")
    try:
        return Pose(value.position, value.orientation)
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name}: {exc}") from exc


def _reconstruct_obstacle_snapshot(value: object) -> ObstacleSnapshot:
    """重构造无 body-id 障碍物，拒绝被绕过 frozen 校验的 IPC 对象。"""
    if type(value) is not ObstacleSnapshot:
        raise ValueError("obstacle snapshots must contain exact ObstacleSnapshot values")
    if value.body_id is not None:
        raise ValueError("obstacle snapshots must not contain body_id")
    if type(value.geometry) is not ObstacleGeometry:
        raise ValueError("obstacle snapshot geometry must be an exact ObstacleGeometry")
    geometry = ObstacleGeometry(value.geometry.shape, value.geometry.half_extents)
    if value.shape != geometry.shape:
        raise ValueError("obstacle snapshot shape must match geometry")
    path = value.path
    if path is not None:
        if type(path) is not ObstaclePath:
            raise ValueError("obstacle snapshot path must be an exact ObstaclePath")
        path = ObstaclePath(
            path.start_xy,
            path.end_xy,
            path.speed,
            path.progress,
            path.direction,
        )
    spec = ObstacleSpec(
        value.logical_id,
        value.mode,
        geometry,
        value.position,
        value.orientation,
        path,
    )
    return ObstacleSnapshot(
        spec.logical_id,
        None,
        spec.mode,
        spec.geometry.shape,
        spec.position,
        spec.orientation,
        spec.path,
        spec.geometry,
    )


def _reconstruct_lidar_message(value: object) -> LidarPointCloud:
    if type(value) is not LidarPointCloud:
        raise ValueError("message must be an exact LidarPointCloud")
    points: list[LidarPoint] = []
    for point in value.points:
        if type(point) is not LidarPoint:
            raise ValueError("message points must be exact LidarPoint values")
        points.append(
            LidarPoint(
                point.offset_time_ns,
                point.x,
                point.y,
                point.z,
                point.reflectivity,
                point.tag,
                point.line,
            )
        )
    return LidarPointCloud(
        value.timebase_ns,
        value.frame_id,
        value.point_num,
        value.lidar_id,
        tuple(points),
    )


def _reconstruct_top_view(value: object) -> LidarTopViewFrame:
    if type(value) is not LidarTopViewFrame:
        raise ValueError("optional_top_view must be an exact LidarTopViewFrame")
    points: list[LidarTopViewPoint] = []
    for point in value.points:
        if type(point) is not LidarTopViewPoint:
            raise ValueError("top-view points must be exact LidarTopViewPoint values")
        points.append(LidarTopViewPoint(point.x, point.y, point.tag, point.lidar_id))
    return LidarTopViewFrame(value.timestamp_ns, tuple(points))


def _reconstruct_experiment_config(value: object) -> ExperimentConfig:
    """重新构造配置，拒绝绕过 frozen dataclass 的跨进程对象。"""
    if type(value) is not ExperimentConfig:
        raise ValueError("experiment_config must be an exact ExperimentConfig")
    try:
        return ExperimentConfig(**{field.name: getattr(value, field.name) for field in fields(ExperimentConfig)})
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"experiment_config: {exc}") from exc


def _reconstruct_scene_document(value: object) -> SceneDocument:
    """经规范 mapping 往返重建场景，确保 child 重新执行全部领域校验。"""
    if type(value) is not SceneDocument:
        raise ValueError("scene_document must be an exact SceneDocument")
    try:
        return scene_document_from_mapping(document_to_mapping(value))
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"scene_document: {exc}") from exc


def world_digest_for_document(document: object) -> str:
    """计算规范 JSON 的小写 SHA-256，不包含物理运行时参数。"""
    validated = _reconstruct_scene_document(document)
    try:
        canonical = json.dumps(
            document_to_mapping(validated),
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError, UnicodeError) as exc:
        raise ValueError(f"scene_document is not canonical JSON: {exc}") from exc
    return hashlib.sha256(canonical).hexdigest()


@dataclass(frozen=True, slots=True)
class LidarWorkerWorldSpec:
    """spawn 时唯一传入 child 的完整世界输入与显式扫描 profile。"""

    protocol_version: int
    experiment_config: ExperimentConfig
    scene_document: SceneDocument
    world_digest: str
    profile: str = "stage3"

    def __post_init__(self) -> None:
        version = _require_protocol_version(self.protocol_version)
        config = _reconstruct_experiment_config(self.experiment_config)
        document = _reconstruct_scene_document(self.scene_document)
        digest = _require_sha256("world_digest", self.world_digest)
        profile = self.profile
        if type(profile) is not str or profile not in _WORKER_TOPICS_BY_PROFILE:
            raise ValueError("profile must select a supported lidar worker profile")
        expected_digest = world_digest_for_document(document)
        if digest != expected_digest:
            raise ValueError("world_digest must match scene_document")
        object.__setattr__(self, "protocol_version", version)
        object.__setattr__(self, "experiment_config", config)
        object.__setattr__(self, "scene_document", document)
        object.__setattr__(self, "world_digest", digest)
        object.__setattr__(self, "profile", profile)


@dataclass(frozen=True, slots=True)
class LidarWorkerReady:
    """只在所选 worker profile 的全部 LiDAR 完成预热后才发出的成功信封。"""

    protocol_version: int
    process_id: int
    world_digest: str
    prewarmed_topics: tuple[str, ...]
    prewarm_payload_sha256_by_topic: tuple[tuple[str, str], ...]
    prewarm_max_scan_wall_duration_ns: int

    def __post_init__(self) -> None:
        version = _require_protocol_version(self.protocol_version)
        process_id = _require_uint64("process_id", self.process_id)
        digest = _require_sha256("world_digest", self.world_digest)
        topics = self.prewarmed_topics
        hashes = self.prewarm_payload_sha256_by_topic
        if type(topics) is not tuple or topics not in _WORKER_TOPICS_BY_PROFILE.values():
            raise ValueError("prewarmed_topics must match a supported worker profile")
        if type(hashes) is not tuple or len(hashes) != len(topics):
            raise ValueError("prewarm_payload_sha256_by_topic must cover every prewarmed topic")
        normalized_hashes: list[tuple[str, str]] = []
        for topic, pair in zip(topics, hashes, strict=True):
            if type(pair) is not tuple or len(pair) != 2 or pair[0] != topic:
                raise ValueError("prewarm payload hashes must be ordered topic/hash pairs")
            normalized_hashes.append((topic, _require_sha256(f"{topic} prewarm payload", pair[1])))
        duration = _require_uint64(
            "prewarm_max_scan_wall_duration_ns",
            self.prewarm_max_scan_wall_duration_ns,
        )
        object.__setattr__(self, "protocol_version", version)
        object.__setattr__(self, "process_id", process_id)
        object.__setattr__(self, "world_digest", digest)
        object.__setattr__(self, "prewarmed_topics", topics)
        object.__setattr__(self, "prewarm_payload_sha256_by_topic", tuple(normalized_hashes))
        object.__setattr__(self, "prewarm_max_scan_wall_duration_ns", duration)


@dataclass(frozen=True, slots=True)
class LidarWorkerStop:
    """父端只在 service 已排空后发送的正常关闭请求。"""

    protocol_version: int
    process_id: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "protocol_version", _require_protocol_version(self.protocol_version))
        object.__setattr__(
            self,
            "process_id",
            _require_positive_uint64("process_id", self.process_id),
        )


@dataclass(frozen=True, slots=True)
class LidarWorkerStopped:
    """child 断开自有 DIRECT client 后返回的正常关闭确认。"""

    protocol_version: int
    process_id: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "protocol_version", _require_protocol_version(self.protocol_version))
        object.__setattr__(
            self,
            "process_id",
            _require_positive_uint64("process_id", self.process_id),
        )


@dataclass(frozen=True, slots=True)
class LidarWorkerStartupFailure:
    """child ready 前的唯一失败信封，phase 与稳定错误码一一对应。"""

    protocol_version: int
    process_id: int
    phase: str
    stable_error_code: str
    bounded_detail: str

    def __post_init__(self) -> None:
        version = _require_protocol_version(self.protocol_version)
        process_id = _require_uint64("process_id", self.process_id)
        phase = self.phase
        if type(phase) is not str or phase not in _STARTUP_PHASE_CODES:
            raise ValueError("unsupported startup failure phase")
        error_code = self.stable_error_code
        if type(error_code) is not str or error_code != _STARTUP_PHASE_CODES[phase]:
            raise ValueError("startup failure phase has an invalid stable_error_code")
        detail = _require_bounded_detail(self.bounded_detail)
        object.__setattr__(self, "protocol_version", version)
        object.__setattr__(self, "process_id", process_id)
        object.__setattr__(self, "phase", phase)
        object.__setattr__(self, "stable_error_code", error_code)
        object.__setattr__(self, "bounded_detail", detail)


@dataclass(frozen=True, slots=True)
class LidarScanRequest:
    """父进程在单一物理状态冻结的一帧完整扫描输入。"""

    protocol_version: int
    job_id: int
    captured_monotonic_ns: int
    lifecycle_generation: int
    pause_epoch: int
    topic: str
    frame_id: str
    lidar_id: int
    timestamp_ns: int
    world_mount_pose: Pose
    optional_base_pose: Pose | None
    complete_obstacle_snapshots_without_body_ids: tuple[ObstacleSnapshot, ...]
    output_identity: OutputIdentity | None = None

    def __post_init__(self) -> None:
        version = _require_protocol_version(self.protocol_version)
        job_id = _require_uint64("job_id", self.job_id)
        captured_ns = _require_uint64("captured_monotonic_ns", self.captured_monotonic_ns)
        generation = _require_uint64("lifecycle_generation", self.lifecycle_generation)
        pause_epoch = _require_uint64("pause_epoch", self.pause_epoch)
        topic, frame_id, lidar_id = _require_lidar_identity(
            self.topic,
            frame_id=self.frame_id,
            lidar_id=self.lidar_id,
        )
        timestamp_ns = _require_uint64("timestamp_ns", self.timestamp_ns)
        mount = _reconstruct_pose("world_mount_pose", self.world_mount_pose)
        base = (
            None
            if self.optional_base_pose is None
            else _reconstruct_pose("optional_base_pose", self.optional_base_pose)
        )
        raw_snapshots = self.complete_obstacle_snapshots_without_body_ids
        if type(raw_snapshots) is not tuple:
            raise ValueError("complete obstacle snapshots must be an exact tuple")
        snapshots = tuple(_reconstruct_obstacle_snapshot(item) for item in raw_snapshots)
        logical_ids = tuple(snapshot.logical_id for snapshot in snapshots)
        if len(set(logical_ids)) != len(logical_ids):
            raise ValueError("complete obstacle snapshots must use unique logical_id values")
        output_identity = _reconstruct_v2_output_identity(self.output_identity)
        if output_identity is not None and topic != "lidar_link":
            raise ValueError("v2 output identity is only valid for the Stage4 center lidar")
        object.__setattr__(self, "protocol_version", version)
        object.__setattr__(self, "job_id", job_id)
        object.__setattr__(self, "captured_monotonic_ns", captured_ns)
        object.__setattr__(self, "lifecycle_generation", generation)
        object.__setattr__(self, "pause_epoch", pause_epoch)
        object.__setattr__(self, "topic", topic)
        object.__setattr__(self, "frame_id", frame_id)
        object.__setattr__(self, "lidar_id", lidar_id)
        object.__setattr__(self, "timestamp_ns", timestamp_ns)
        object.__setattr__(self, "world_mount_pose", mount)
        object.__setattr__(self, "optional_base_pose", base)
        object.__setattr__(self, "complete_obstacle_snapshots_without_body_ids", snapshots)
        object.__setattr__(self, "output_identity", output_identity)


@dataclass(frozen=True, slots=True)
class PreparedLidarFrame:
    """worker 同源构造并只编码一次的完整发布帧。"""

    protocol_version: int
    job_id: int
    lifecycle_generation: int
    pause_epoch: int
    topic: str
    timestamp_ns: int
    message: LidarPointCloud
    optional_top_view: LidarTopViewFrame | None
    protobuf_payload: bytes
    scan_wall_duration_ns: int

    def __post_init__(self) -> None:
        version = _require_protocol_version(self.protocol_version)
        job_id = _require_uint64("job_id", self.job_id)
        generation = _require_uint64("lifecycle_generation", self.lifecycle_generation)
        pause_epoch = _require_uint64("pause_epoch", self.pause_epoch)
        topic, frame_id, lidar_id = _require_lidar_identity(self.topic)
        timestamp_ns = _require_uint64("timestamp_ns", self.timestamp_ns)
        message = _reconstruct_lidar_message(self.message)
        _require_lidar_identity(topic, frame_id=message.frame_id, lidar_id=message.lidar_id)
        if message.timebase_ns != timestamp_ns:
            raise ValueError("message timestamp must match prepared frame")
        top_view = (
            None
            if self.optional_top_view is None
            else _reconstruct_top_view(self.optional_top_view)
        )
        if top_view is not None:
            LidarScanResult(message, top_view)
        payload = self.protobuf_payload
        if type(payload) is not bytes or not payload:
            raise ValueError("protobuf_payload must be nonempty exact bytes")
        duration = _require_uint64("scan_wall_duration_ns", self.scan_wall_duration_ns)
        object.__setattr__(self, "protocol_version", version)
        object.__setattr__(self, "job_id", job_id)
        object.__setattr__(self, "lifecycle_generation", generation)
        object.__setattr__(self, "pause_epoch", pause_epoch)
        object.__setattr__(self, "topic", topic)
        object.__setattr__(self, "timestamp_ns", timestamp_ns)
        object.__setattr__(self, "message", message)
        object.__setattr__(self, "optional_top_view", top_view)
        object.__setattr__(self, "protobuf_payload", payload)
        object.__setattr__(self, "scan_wall_duration_ns", duration)


@dataclass(frozen=True, slots=True)
class PreparedLidarPayload:
    """headless 成功帧只传递身份和预编码 bytes，避免逐点对象穿过 IPC。"""

    protocol_version: int
    job_id: int
    lifecycle_generation: int
    pause_epoch: int
    topic: str
    timestamp_ns: int
    protobuf_payload: bytes
    scan_wall_duration_ns: int

    def __post_init__(self) -> None:
        version = _require_protocol_version(self.protocol_version)
        job_id = _require_uint64("job_id", self.job_id)
        generation = _require_uint64("lifecycle_generation", self.lifecycle_generation)
        pause_epoch = _require_uint64("pause_epoch", self.pause_epoch)
        topic, _frame_id, _lidar_id = _require_lidar_identity(self.topic)
        timestamp_ns = _require_uint64("timestamp_ns", self.timestamp_ns)
        payload = self.protobuf_payload
        if type(payload) is not bytes or not payload:
            raise ValueError("protobuf_payload must be nonempty exact bytes")
        duration = _require_uint64("scan_wall_duration_ns", self.scan_wall_duration_ns)
        object.__setattr__(self, "protocol_version", version)
        object.__setattr__(self, "job_id", job_id)
        object.__setattr__(self, "lifecycle_generation", generation)
        object.__setattr__(self, "pause_epoch", pause_epoch)
        object.__setattr__(self, "topic", topic)
        object.__setattr__(self, "timestamp_ns", timestamp_ns)
        object.__setattr__(self, "protobuf_payload", payload)
        object.__setattr__(self, "scan_wall_duration_ns", duration)


@dataclass(frozen=True, slots=True)
class LidarScanFailure:
    """单帧失败的稳定信封，不跨 IPC 发送异常或 traceback。"""

    protocol_version: int
    job_id: int
    lifecycle_generation: int
    pause_epoch: int
    topic: str
    timestamp_ns: int
    stable_error_code: str
    bounded_detail: str
    scan_wall_duration_ns: int

    def __post_init__(self) -> None:
        version = _require_protocol_version(self.protocol_version)
        job_id = _require_uint64("job_id", self.job_id)
        generation = _require_uint64("lifecycle_generation", self.lifecycle_generation)
        pause_epoch = _require_uint64("pause_epoch", self.pause_epoch)
        topic, _frame_id, _lidar_id = _require_lidar_identity(self.topic)
        timestamp_ns = _require_uint64("timestamp_ns", self.timestamp_ns)
        error_code = self.stable_error_code
        if type(error_code) is not str or error_code not in _STABLE_ERROR_CODES:
            raise ValueError("stable_error_code is not supported")
        detail = _require_bounded_detail(self.bounded_detail)
        duration = _require_uint64("scan_wall_duration_ns", self.scan_wall_duration_ns)
        object.__setattr__(self, "protocol_version", version)
        object.__setattr__(self, "job_id", job_id)
        object.__setattr__(self, "lifecycle_generation", generation)
        object.__setattr__(self, "pause_epoch", pause_epoch)
        object.__setattr__(self, "topic", topic)
        object.__setattr__(self, "timestamp_ns", timestamp_ns)
        object.__setattr__(self, "stable_error_code", error_code)
        object.__setattr__(self, "bounded_detail", detail)
        object.__setattr__(self, "scan_wall_duration_ns", duration)


@dataclass(frozen=True, slots=True)
class LidarServiceEvent:
    """父端按连续序号暴露的一次性 topic/service 归因事件。"""

    sequence: int
    kind: str
    scope: str
    optional_topic: str | None
    optional_job_identity: tuple[int, int, int, str, int] | None
    stable_error_code: str
    bounded_detail: str

    def __post_init__(self) -> None:
        sequence = _require_positive_uint64("sequence", self.sequence)
        kind = self.kind
        if type(kind) is not str or kind not in _SERVICE_EVENT_KINDS:
            raise ValueError("unsupported lidar service event kind")
        scope = self.scope
        if type(scope) is not str or scope not in {"topic", "service"}:
            raise ValueError("scope must be topic or service")
        raw_identity = self.optional_job_identity
        identity = None if raw_identity is None else _require_job_identity(raw_identity)
        error_code = self.stable_error_code
        if type(error_code) is not str:
            raise ValueError("stable_error_code must be an exact string")
        detail = _require_bounded_detail(self.bounded_detail)

        if kind in {"frame_failed", "capture_rejected", "job_overrun"}:
            if scope != "topic":
                raise ValueError("topic event kind requires topic scope")
            topic, _frame_id, _lidar_id = _require_lidar_identity(self.optional_topic)
            if identity is not None and identity[3] != topic:
                raise ValueError("event topic must match job identity")
            if kind == "frame_failed":
                if identity is None or error_code not in _FRAME_ERROR_CODES:
                    raise ValueError("frame_failed requires a frame job and error code")
            elif kind == "capture_rejected":
                if identity is not None or error_code != "sensor_overrun":
                    raise ValueError("capture_rejected must describe an unsent capture")
            elif identity is None or error_code != "sensor_overrun":
                raise ValueError("job_overrun requires a sent job identity")
        else:
            if scope != "service" or self.optional_topic is not None:
                raise ValueError("service event kind requires service scope without topic")
            topic = None
            if kind == "service_failed":
                if error_code not in _SERVICE_ERROR_CODES:
                    raise ValueError("service_failed requires a service error code")
            elif identity is not None or error_code != "worker_shutdown_failed":
                raise ValueError("retired_cleanup_failed requires shutdown failure")

        object.__setattr__(self, "sequence", sequence)
        object.__setattr__(self, "kind", kind)
        object.__setattr__(self, "scope", scope)
        object.__setattr__(self, "optional_topic", topic)
        object.__setattr__(self, "optional_job_identity", identity)
        object.__setattr__(self, "stable_error_code", error_code)
        object.__setattr__(self, "bounded_detail", detail)


@dataclass(frozen=True, slots=True)
class LidarServiceSnapshot:
    """固定字段的只读 service 生命周期诊断，不承担事件归因。"""

    state: str
    child_pid: int
    lifecycle_generation: int
    pause_epoch: int
    next_job_id: int
    in_flight_identity: tuple[int, int, int, str, int] | None
    pending_capture_identity: tuple[int, int, str, int] | None
    completed_count: int
    failed_count: int
    overrun_count: int
    stale_count: int
    max_capture_to_response_ns: int
    last_error_code: str | None
    last_error_detail: str

    def __post_init__(self) -> None:
        state = self.state
        if type(state) is not str or state not in _SERVICE_STATES:
            raise ValueError("unsupported lidar service state")
        child_pid = _require_positive_uint64("child_pid", self.child_pid)
        generation = _require_uint64("lifecycle_generation", self.lifecycle_generation)
        pause_epoch = _require_uint64("pause_epoch", self.pause_epoch)
        next_job_id = _require_positive_uint64("next_job_id", self.next_job_id)
        in_flight = (
            None
            if self.in_flight_identity is None
            else _require_job_identity(self.in_flight_identity)
        )
        pending = (
            None
            if self.pending_capture_identity is None
            else _require_capture_identity(self.pending_capture_identity)
        )
        if in_flight is not None and in_flight[0] >= next_job_id:
            raise ValueError("next_job_id must follow the in-flight job")
        counts = tuple(
            _require_uint64(name, value)
            for name, value in (
                ("completed_count", self.completed_count),
                ("failed_count", self.failed_count),
                ("overrun_count", self.overrun_count),
                ("stale_count", self.stale_count),
                ("max_capture_to_response_ns", self.max_capture_to_response_ns),
            )
        )
        error_code = self.last_error_code
        if error_code is None:
            if type(self.last_error_detail) is not str or self.last_error_detail != "":
                raise ValueError("last_error_detail requires last_error_code")
            error_detail = ""
        else:
            if type(error_code) is not str or error_code not in _STABLE_ERROR_CODES:
                raise ValueError("last_error_code is not supported")
            error_detail = _require_bounded_detail(self.last_error_detail)

        object.__setattr__(self, "state", state)
        object.__setattr__(self, "child_pid", child_pid)
        object.__setattr__(self, "lifecycle_generation", generation)
        object.__setattr__(self, "pause_epoch", pause_epoch)
        object.__setattr__(self, "next_job_id", next_job_id)
        object.__setattr__(self, "in_flight_identity", in_flight)
        object.__setattr__(self, "pending_capture_identity", pending)
        object.__setattr__(self, "completed_count", counts[0])
        object.__setattr__(self, "failed_count", counts[1])
        object.__setattr__(self, "overrun_count", counts[2])
        object.__setattr__(self, "stale_count", counts[3])
        object.__setattr__(self, "max_capture_to_response_ns", counts[4])
        object.__setattr__(self, "last_error_code", error_code)
        object.__setattr__(self, "last_error_detail", error_detail)


@dataclass(frozen=True, slots=True)
class LidarServiceChannel:
    """把两条单向 Connection 合成 service 使用的 send/poll/recv 窄接口。"""

    request_sender: Connection
    response_receiver: Connection

    def __post_init__(self) -> None:
        if not callable(getattr(self.request_sender, "send", None)):
            raise ValueError("request_sender must provide send()")
        if not callable(getattr(self.response_receiver, "poll", None)):
            raise ValueError("response_receiver must provide poll()")
        if not callable(getattr(self.response_receiver, "recv", None)):
            raise ValueError("response_receiver must provide recv()")

    def send(self, value: object) -> None:
        self.request_sender.send(value)

    def poll(self, timeout: float = 0.0) -> bool:
        return self.response_receiver.poll(timeout)

    def recv(self) -> object:
        return self.response_receiver.recv()


@dataclass(frozen=True, slots=True)
class _PendingLidarCapture:
    """尚未写入 pipe 的完整父端 capture，刻意不包含 job id。"""

    captured_monotonic_ns: int
    lifecycle_generation: int
    pause_epoch: int
    topic: str
    timestamp_ns: int
    world_mount_pose: Pose
    optional_base_pose: Pose | None
    complete_obstacle_snapshots_without_body_ids: tuple[ObstacleSnapshot, ...]
    output_identity: OutputIdentity | None = None

    def __post_init__(self) -> None:
        captured_ns = _require_uint64("captured_monotonic_ns", self.captured_monotonic_ns)
        generation = _require_uint64("lifecycle_generation", self.lifecycle_generation)
        pause_epoch = _require_uint64("pause_epoch", self.pause_epoch)
        topic, _frame_id, _lidar_id = _require_lidar_identity(self.topic)
        timestamp_ns = _require_uint64("timestamp_ns", self.timestamp_ns)
        mount = _reconstruct_pose("world_mount_pose", self.world_mount_pose)
        base = (
            None
            if self.optional_base_pose is None
            else _reconstruct_pose("optional_base_pose", self.optional_base_pose)
        )
        raw_snapshots = self.complete_obstacle_snapshots_without_body_ids
        if type(raw_snapshots) is not tuple:
            raise ValueError("complete obstacle snapshots must be an exact tuple")
        snapshots = tuple(_reconstruct_obstacle_snapshot(item) for item in raw_snapshots)
        logical_ids = tuple(snapshot.logical_id for snapshot in snapshots)
        if len(set(logical_ids)) != len(logical_ids):
            raise ValueError("complete obstacle snapshots must use unique logical_id values")
        output_identity = _reconstruct_v2_output_identity(self.output_identity)
        if output_identity is not None and topic != "lidar_link":
            raise ValueError("v2 output identity is only valid for the Stage4 center lidar")
        object.__setattr__(self, "captured_monotonic_ns", captured_ns)
        object.__setattr__(self, "lifecycle_generation", generation)
        object.__setattr__(self, "pause_epoch", pause_epoch)
        object.__setattr__(self, "topic", topic)
        object.__setattr__(self, "timestamp_ns", timestamp_ns)
        object.__setattr__(self, "world_mount_pose", mount)
        object.__setattr__(self, "optional_base_pose", base)
        object.__setattr__(self, "complete_obstacle_snapshots_without_body_ids", snapshots)
        object.__setattr__(self, "output_identity", output_identity)

    @property
    def identity(self) -> tuple[int, int, str, int]:
        return (
            self.lifecycle_generation,
            self.pause_epoch,
            self.topic,
            self.timestamp_ns,
        )

    def into_request(self, job_id: int) -> LidarScanRequest:
        """只在真正发送前分配连续 ID，并保留最初 capture 时钟。"""
        _topic, frame_id, lidar_id = _require_lidar_identity(self.topic)
        return LidarScanRequest(
            _PROTOCOL_VERSION,
            job_id,
            self.captured_monotonic_ns,
            self.lifecycle_generation,
            self.pause_epoch,
            self.topic,
            frame_id,
            lidar_id,
            self.timestamp_ns,
            self.world_mount_pose,
            self.optional_base_pose,
            self.complete_obstacle_snapshots_without_body_ids,
            self.output_identity,
        )


def _locked_service_method(method: object) -> object:
    """把父端 service 的公开状态迁移线性化到同一临界区。"""

    @wraps(method)
    def locked(self, *args: object, **kwargs: object) -> object:
        with self._state_lock:
            return method(self, *args, **kwargs)

    return locked


class LidarScanService:
    """纯父进程有界 facade；扫描、编码和 transport 均留在其所有权之外。"""

    def __init__(
        self,
        channel: object,
        *,
        child_pid: int,
        lifecycle_generation: int,
        pause_epoch: int = 0,
        monotonic_ns: object = time.monotonic_ns,
    ) -> None:
        for method_name in ("send", "poll", "recv"):
            if not callable(getattr(channel, method_name, None)):
                raise ValueError(f"channel must provide {method_name}()")
        if not callable(monotonic_ns):
            raise ValueError("monotonic_ns must be callable")
        self._state_lock = RLock()
        self._channel = channel
        self._monotonic_ns = monotonic_ns
        self._state = "ready"
        self._child_pid = _require_positive_uint64("child_pid", child_pid)
        self._lifecycle_generation = _require_uint64(
            "lifecycle_generation",
            lifecycle_generation,
        )
        self._pause_epoch = _require_uint64("pause_epoch", pause_epoch)
        self._next_job_id = 1
        self._in_flight: LidarScanRequest | None = None
        self._in_flight_overrun_reported = False
        self._pending: _PendingLidarCapture | None = None
        self._events: list[LidarServiceEvent] = []
        self._next_event_sequence = 1
        self._completed_count = 0
        self._failed_count = 0
        self._overrun_count = 0
        self._stale_count = 0
        self._max_capture_to_response_ns = 0
        self._last_error_code: str | None = None
        self._last_error_detail = ""
        self._owned_worker_handle: LidarWorkerHandle | None = None

    @classmethod
    def from_worker_handle(
        cls,
        handle: object,
        *,
        lifecycle_generation: int,
        pause_epoch: int = 0,
        monotonic_ns: object = time.monotonic_ns,
    ) -> LidarScanService:
        """生产路径把既有 handle 的两条单向 pipe 接入同一窄通道。"""
        if type(handle) is not LidarWorkerHandle:
            raise ValueError("handle must be an exact LidarWorkerHandle")
        channel = LidarServiceChannel(handle.request_sender, handle.response_receiver)
        service = cls(
            channel,
            child_pid=handle.ready.process_id,
            lifecycle_generation=lifecycle_generation,
            pause_epoch=pause_epoch,
            monotonic_ns=monotonic_ns,
        )
        service._owned_worker_handle = handle
        return service

    def _emit_event(
        self,
        kind: str,
        scope: str,
        optional_topic: str | None,
        optional_job_identity: tuple[int, int, int, str, int] | None,
        stable_error_code: str,
        bounded_detail: str,
    ) -> None:
        event = LidarServiceEvent(
            self._next_event_sequence,
            kind,
            scope,
            optional_topic,
            optional_job_identity,
            stable_error_code,
            bounded_detail,
        )
        self._events.append(event)
        self._next_event_sequence += 1

    def _in_flight_identity(self) -> tuple[int, int, int, str, int] | None:
        request = self._in_flight
        if request is None:
            return None
        return (
            request.job_id,
            request.lifecycle_generation,
            request.pause_epoch,
            request.topic,
            request.timestamp_ns,
        )

    def _fail_service(self, stable_error_code: str, bounded_detail: str) -> None:
        """锁存第一个基础设施错误，并只生成一个 service 级事件。"""
        if self._state == "failed":
            return
        identity = self._in_flight_identity()
        self._state = "failed"
        self._pending = None
        self._last_error_code = stable_error_code
        self._last_error_detail = bounded_detail
        self._emit_event(
            "service_failed",
            "service",
            None,
            identity,
            stable_error_code,
            bounded_detail,
        )

    @staticmethod
    def _reconstruct_response(
        value: object,
    ) -> PreparedLidarFrame | PreparedLidarPayload | LidarScanFailure:
        """父端不信任反序列化对象，按 exact type 重构造并触发全字段校验。"""
        if type(value) is PreparedLidarFrame:
            return PreparedLidarFrame(
                value.protocol_version,
                value.job_id,
                value.lifecycle_generation,
                value.pause_epoch,
                value.topic,
                value.timestamp_ns,
                value.message,
                value.optional_top_view,
                value.protobuf_payload,
                value.scan_wall_duration_ns,
            )
        if type(value) is PreparedLidarPayload:
            return PreparedLidarPayload(
                value.protocol_version,
                value.job_id,
                value.lifecycle_generation,
                value.pause_epoch,
                value.topic,
                value.timestamp_ns,
                value.protobuf_payload,
                value.scan_wall_duration_ns,
            )
        if type(value) is LidarScanFailure:
            failure = LidarScanFailure(
                value.protocol_version,
                value.job_id,
                value.lifecycle_generation,
                value.pause_epoch,
                value.topic,
                value.timestamp_ns,
                value.stable_error_code,
                value.bounded_detail,
                value.scan_wall_duration_ns,
            )
            if failure.stable_error_code not in _FRAME_ERROR_CODES:
                raise ValueError("scan failure used a non-frame error code")
            return failure
        raise ValueError("worker returned an unsupported response type")

    def _send_capture(self, capture: _PendingLidarCapture) -> bool:
        request = capture.into_request(self._next_job_id)
        try:
            self._channel.send(request)
        except (EOFError, OSError):
            self._fail_service("worker_exited", "lidar worker request pipe closed")
            return False
        except Exception:
            self._fail_service("worker_protocol_failed", "lidar worker request send failed")
            return False
        self._in_flight = request
        self._in_flight_overrun_reported = False
        self._next_job_id += 1
        return True

    def _check_in_flight_overrun(self) -> None:
        """只在当前 generation/epoch 首次严格越过 100 ms 时生成一次事件。"""
        request = self._in_flight
        if request is None or self._in_flight_overrun_reported:
            return
        if (
            request.lifecycle_generation != self._lifecycle_generation
            or request.pause_epoch != self._pause_epoch
        ):
            return
        now_ns = _require_uint64("monotonic_ns result", self._monotonic_ns())
        if now_ns < request.captured_monotonic_ns:
            raise ValueError("monotonic clock moved backwards")
        if now_ns - request.captured_monotonic_ns <= _LIDAR_JOB_BUDGET_NS:
            return
        self._in_flight_overrun_reported = True
        self._overrun_count += 1
        self._emit_event(
            "job_overrun",
            "topic",
            request.topic,
            self._in_flight_identity(),
            "sensor_overrun",
            "lidar job exceeded 100 ms capture-to-response budget",
        )

    @_locked_service_method
    def capture(
        self,
        *,
        topic: str,
        timestamp_ns: int,
        world_mount_pose: Pose,
        optional_base_pose: Pose | None,
        complete_obstacle_snapshots_without_body_ids: tuple[ObstacleSnapshot, ...],
        output_identity: OutputIdentity | None = None,
    ) -> bool:
        """非阻塞提交 capture；容量满时只拒绝最新值，不覆盖旧帧。"""
        if self._state != "ready":
            return False
        captured_ns = _require_uint64("monotonic_ns result", self._monotonic_ns())
        capture = _PendingLidarCapture(
            captured_ns,
            self._lifecycle_generation,
            self._pause_epoch,
            topic,
            timestamp_ns,
            world_mount_pose,
            optional_base_pose,
            complete_obstacle_snapshots_without_body_ids,
            output_identity,
        )
        if self._in_flight is None:
            return self._send_capture(capture)
        if self._pending is None:
            self._pending = capture
            return True
        self._emit_event(
            "capture_rejected",
            "topic",
            capture.topic,
            None,
            "sensor_overrun",
            "lidar capture capacity is full",
        )
        return False

    @_locked_service_method
    def pause(self) -> None:
        """暂停新 capture、递增 epoch，并撤销仍未写入 pipe 的 pending。"""
        if self._state != "ready":
            return
        if self._pause_epoch == _UINT64_MAX:
            raise OverflowError("pause_epoch cannot advance beyond uint64")
        self._pause_epoch += 1
        self._state = "suspended"
        self._pending = None

    @_locked_service_method
    def resume(self) -> None:
        """仅把健康 suspended service 恢复为可提交状态。"""
        if self._state == "suspended":
            self._state = "ready"

    @_locked_service_method
    def begin_draining(self) -> None:
        """停止接收新 capture，但保留并排空已捕获的两级队列。"""
        if self._state in {"ready", "suspended"}:
            self._state = "draining"

    @_locked_service_method
    def close_idle(self, timeout_sec: float = 5.0) -> None:
        """仅在父端队列为空时终结 owned worker；成功后保持幂等。"""
        if self._state == "closed":
            return
        if self._in_flight is not None or self._pending is not None:
            raise RuntimeError("lidar service must be idle before closing")
        handle = self._owned_worker_handle
        if handle is None:
            raise RuntimeError("lidar service does not own a worker handle")
        # 关闭失败时保留 handle 和原状态，让 retired cleanup 可以重试。
        handle.close(timeout_sec=timeout_sec)
        self._state = "closed"

    @_locked_service_method
    def force_close(self) -> None:
        """撤销不可发布队列并强制回收 exact owned child，不等待正常 ACK。"""
        if self._state == "closed":
            return
        handle = self._owned_worker_handle
        if handle is None:
            raise RuntimeError("lidar service does not own a worker handle")
        self._pending = None
        self._in_flight = None
        self._in_flight_overrun_reported = False
        handle.force_close()
        self._state = "closed"

    @_locked_service_method
    def invalidate_generation(self, lifecycle_generation: int) -> None:
        """推进父端 generation，撤销旧 pending 并保留不可取消的 in-flight。"""
        generation = _require_uint64("lifecycle_generation", lifecycle_generation)
        if generation < self._lifecycle_generation:
            raise ValueError("lifecycle_generation must not move backwards")
        if generation == self._lifecycle_generation:
            return
        self._lifecycle_generation = generation
        self._pending = None

    @_locked_service_method
    def poll(self) -> PreparedLidarFrame | PreparedLidarPayload | None:
        """以 poll(0) 收取至多一个结果，并在完成后提升 parent-side pending。"""
        if self._state in {"failed", "closed"}:
            return None
        try:
            ready = self._channel.poll(0.0)
        except (EOFError, OSError):
            self._fail_service("worker_exited", "lidar worker response channel failed")
            return None
        except Exception:
            self._fail_service("worker_protocol_failed", "lidar worker poll failed")
            return None
        if type(ready) is not bool:
            self._fail_service("worker_protocol_failed", "lidar worker poll returned non-bool")
            return None
        if not ready:
            self._check_in_flight_overrun()
            return None
        try:
            raw_response = self._channel.recv()
        except (EOFError, OSError):
            self._fail_service("worker_exited", "lidar worker response pipe closed")
            return None
        except Exception:
            self._fail_service("worker_protocol_failed", "lidar worker response decode failed")
            return None
        request = self._in_flight
        if request is None:
            self._fail_service(
                "worker_protocol_failed",
                "lidar worker returned a response without an in-flight job",
            )
            return None
        try:
            response = self._reconstruct_response(raw_response)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self._fail_service(
                "worker_protocol_failed",
                "lidar worker returned an invalid response",
            )
            return None
        response_identity = (
            response.job_id,
            response.lifecycle_generation,
            response.pause_epoch,
            response.topic,
            response.timestamp_ns,
        )
        if response_identity != self._in_flight_identity():
            self._fail_service(
                "worker_protocol_failed",
                "lidar worker response did not match the in-flight job",
            )
            return None

        now_ns = _require_uint64("monotonic_ns result", self._monotonic_ns())
        if now_ns < request.captured_monotonic_ns:
            raise ValueError("monotonic clock moved backwards")
        duration_ns = now_ns - request.captured_monotonic_ns
        self._max_capture_to_response_ns = max(
            self._max_capture_to_response_ns,
            duration_ns,
        )
        if (
            type(response) is LidarScanFailure
            and response.stable_error_code == "scene_state_unknown"
        ):
            self._failed_count += 1
            self._fail_service(
                response.stable_error_code,
                response.bounded_detail,
            )
            return None
        stale = (
            request.lifecycle_generation != self._lifecycle_generation
            or request.pause_epoch != self._pause_epoch
        )
        if (
            not stale
            and duration_ns > _LIDAR_JOB_BUDGET_NS
            and not self._in_flight_overrun_reported
        ):
            self._in_flight_overrun_reported = True
            self._overrun_count += 1
            self._emit_event(
                "job_overrun",
                "topic",
                request.topic,
                self._in_flight_identity(),
                "sensor_overrun",
                "lidar job exceeded 100 ms capture-to-response budget",
            )
        self._in_flight = None
        self._in_flight_overrun_reported = False

        result: PreparedLidarFrame | None
        if stale:
            self._stale_count += 1
            result = None
        elif type(response) is LidarScanFailure:
            self._failed_count += 1
            self._emit_event(
                "frame_failed",
                "topic",
                response.topic,
                response_identity,
                response.stable_error_code,
                response.bounded_detail,
            )
            result = None
        else:
            self._completed_count += 1
            result = response

        if self._state in {"ready", "draining"} and self._pending is not None:
            pending = self._pending
            self._pending = None
            if self._send_capture(pending):
                self._check_in_flight_overrun()
        return result

    @_locked_service_method
    def drain_events(self) -> tuple[LidarServiceEvent, ...]:
        """按产生顺序原子取走当前事件；重复 drain 不会重复归因。"""
        events = tuple(self._events)
        self._events.clear()
        return events

    @_locked_service_method
    def snapshot(self) -> LidarServiceSnapshot:
        in_flight_identity = self._in_flight_identity()
        pending_identity = None if self._pending is None else self._pending.identity
        return LidarServiceSnapshot(
            self._state,
            self._child_pid,
            self._lifecycle_generation,
            self._pause_epoch,
            self._next_job_id,
            in_flight_identity,
            pending_identity,
            self._completed_count,
            self._failed_count,
            self._overrun_count,
            self._stale_count,
            self._max_capture_to_response_ns,
            self._last_error_code,
            self._last_error_detail,
        )


class LidarWorkerStartupError(RuntimeError):
    """父端启动失败的稳定错误，不把 child 异常对象暴露到 IPC 边界。"""

    def __init__(
        self,
        stable_error_code: str,
        detail: str,
        *,
        startup_failure: LidarWorkerStartupFailure | None = None,
    ) -> None:
        super().__init__(detail)
        self.stable_error_code = stable_error_code
        self.bounded_detail = detail
        self.startup_failure = startup_failure
        self.ready: None = None


@dataclass(slots=True)
class LidarWorkerHandle:
    """父进程持有的启动期 child 资源；本任务尚未开放扫描请求。"""

    process: multiprocessing.Process
    request_sender: Connection
    response_receiver: Connection
    ready: LidarWorkerReady | None
    _stage4_shard_processes: tuple[multiprocessing.Process, ...] = ()

    def close(self, timeout_sec: float = 5.0) -> LidarWorkerStopped | None:
        """发送 Stop 并只把校验通过的 Stopped 信封视为正常退出。"""
        if type(timeout_sec) not in {int, float} or timeout_sec <= 0:
            raise ValueError("timeout_sec must be positive")
        timeout = float(timeout_sec)
        deadline = time.monotonic() + timeout
        try:
            self.request_sender.send(
                LidarWorkerStop(_PROTOCOL_VERSION, self.ready.process_id)
            )
            remaining = max(0.0, deadline - time.monotonic())
            ready = self.response_receiver.poll(remaining)
            if type(ready) is not bool or not ready:
                raise TimeoutError("lidar worker did not acknowledge Stop")
            raw_ack = self.response_receiver.recv()
            if type(raw_ack) is not LidarWorkerStopped:
                raise ValueError("lidar worker returned an invalid Stopped ACK")
            ack = LidarWorkerStopped(raw_ack.protocol_version, raw_ack.process_id)
            if ack.process_id != self.ready.process_id:
                raise ValueError("lidar worker Stopped ACK used the wrong process id")
            self.request_sender.close()
            self.process.join(max(0.0, deadline - time.monotonic()))
            if self.process.is_alive():
                raise TimeoutError("lidar worker remained alive after Stopped ACK")
            for shard in self._stage4_shard_processes:
                shard.join(max(0.0, deadline - time.monotonic()))
                if shard.is_alive():
                    raise TimeoutError("Stage4 shard remained alive after Stopped ACK")
            self.response_receiver.close()
            return ack
        except BaseException as error:
            _run_cleanup_actions(
                (
                    self.request_sender.close,
                    lambda: _reap_owned_process(
                        self.process,
                        initial_join_timeout_sec=0.0,
                    ),
                    *(
                        lambda shard=shard: _reap_owned_process(
                            shard,
                            initial_join_timeout_sec=0.0,
                        )
                        for shard in self._stage4_shard_processes
                    ),
                    self.response_receiver.close,
                )
            )
            raise RuntimeError(
                "lidar worker did not exit after normal shutdown"
            ) from error

    def force_close(self) -> None:
        """关闭 IPC 后仅终结本 handle 持有的 child；不等待成功信封。"""
        error = _run_cleanup_actions(
            (
                self.request_sender.close,
                lambda: _reap_owned_process(
                    self.process,
                    initial_join_timeout_sec=0.0,
                ),
                *(
                    lambda shard=shard: _reap_owned_process(
                        shard,
                        initial_join_timeout_sec=0.0,
                    )
                    for shard in self._stage4_shard_processes
                ),
                self.response_receiver.close,
            )
        )
        if error is not None:
            raise error


def _run_cleanup_actions(actions: tuple[Callable[[], None], ...]) -> BaseException | None:
    """完整遍历 cleanup 动作，并保留最先观察到的错误。"""
    first_error = None
    for action in actions:
        try:
            action()
        except BaseException as error:
            if first_error is None:
                first_error = error
    return first_error


def _bounded_exception_detail(stage: str, error: BaseException) -> str:
    """把内部错误压缩为稳定单行诊断，绝不发送 traceback。"""
    detail = f"{stage} failed: {type(error).__name__}".replace("\r", " ").replace(
        "\n",
        " ",
    )
    encoded = detail.encode("utf-8", errors="replace")
    if len(encoded) <= 512:
        return encoded.decode("utf-8")
    return encoded[:512].decode("utf-8", errors="ignore")


def _world_mount_pose(backend: PyBulletSensorBackend, scanner: MultiLineLidar) -> Pose:
    """在预热边界冻结安装世界位姿，随后扫描不再读取机器人当前姿态。"""
    mount = scanner._mount
    return backend.transform_pose(
        backend.world_pose(mount.parent_link),
        Pose(mount.position, mount.orientation),
    )


def _prewarm_scanner(
    scanner: MultiLineLidar,
    backend: PyBulletSensorBackend,
    encode_payload: Callable[[LidarPointCloud], bytes],
    *,
    expected_ray_count: int,
) -> tuple[str, int]:
    """执行一整帧生产射线、点构造和一次确定性编码。"""
    if scanner.config.ray_count != expected_ray_count:
        raise RuntimeError("worker preflight scanner has an unexpected ray count")
    started_ns = time.monotonic_ns()
    message = scanner._scan_frozen(0, _world_mount_pose(backend, scanner))
    if type(message) is not LidarPointCloud:
        raise RuntimeError("worker preflight expected a lidar point cloud")
    payload = encode_payload(message)
    if type(payload) is not bytes or not payload:
        raise RuntimeError("worker preflight produced an invalid encoded payload")
    return hashlib.sha256(payload).hexdigest(), time.monotonic_ns() - started_ns


def _encode_v2_lidar_payload(
    codec: V2ProtoCodec,
    message: LidarPointCloud,
    identity: OutputIdentity,
) -> bytes:
    """在 worker 内把冻结点云和既有 session identity 确定性编码为唯一 raw bytes。"""
    validated_identity = _validated_v2_lidar_identity(codec, identity)
    if type(message) is not LidarPointCloud:
        raise ValueError("message must be an exact LidarPointCloud")
    if (message.frame_id, message.lidar_id) != ("lidar_link", 1):
        raise ValueError("v2 worker payload must use the Stage4 center lidar")

    # 扫描器已经生成严格校验过的企业点；直接填充 wire message，避免再造一整帧 v2 点对象。
    encoded = pb.LidarPointCloud(
        timebase_ns=message.timebase_ns,
        frame_id=message.frame_id,
        point_num=message.point_num,
        lidar_id=message.lidar_id,
        sequence=validated_identity.sequence,
        world_generation=validated_identity.world_generation,
        simulation_session_id=validated_identity.simulation_session_id,
        descriptor_sha256=validated_identity.descriptor_sha256,
    )
    add_point = encoded.points.add
    for point in message.points:
        add_point(
            offset_time_ns=point.offset_time_ns,
            x=point.x,
            y=point.y,
            z=point.z,
            reflectivity=point.reflectivity,
            tag=point.tag,
            line=point.line,
        )
    return encoded.SerializeToString(deterministic=True)


def _validated_v2_lidar_identity(
    codec: V2ProtoCodec,
    identity: OutputIdentity,
) -> OutputIdentity:
    """复用中心 LiDAR 的 codec、会话和 descriptor 边界校验。"""
    if not isinstance(codec, V2ProtoCodec):
        raise ValueError("codec must be a V2ProtoCodec")
    validated_identity = _reconstruct_v2_output_identity(identity)
    if validated_identity is None:
        raise ValueError("v2 lidar payload requires an output identity")
    if validated_identity.world_generation == 0:
        raise ValueError("world_generation must be positive")
    if validated_identity.descriptor_sha256 != codec._descriptor.sha256:
        raise ValueError("v2 descriptor SHA-256 mismatch")
    return validated_identity


@dataclass(slots=True)
class _WorkerObstacleRecord:
    """child 内部逻辑障碍物与本进程临时 body id 的对应关系。"""

    spec: ObstacleSpec
    body_id: int


class _SceneReconcileError(RuntimeError):
    """镜像仍可证明等于 job 前状态的可恢复 reconcile 失败。"""


class _SceneStateUnknownError(RuntimeError):
    """镜像集合、位姿或分类无法再被证明一致的终态故障。"""


class _WorkerRaycastError(RuntimeError):
    """标记生产 backend 的 native raycast 边界，供帧错误精确归因。"""


class _WorkerSensorBackend(PyBulletSensorBackend):
    """复用生产后端，只为 worker 标记 native 批量射线异常。"""

    def ray_test_indexed_hits(
        self,
        starts: tuple[Vec3, ...],
        ends: tuple[Vec3, ...],
        *,
        collision_mask: int,
    ) -> tuple[tuple[int, RayHit], ...]:
        try:
            return super().ray_test_indexed_hits(
                starts,
                ends,
                collision_mask=collision_mask,
            )
        except BaseException as error:
            raise _WorkerRaycastError("worker native raycast failed") from error

    def _ray_test_indexed_hits_ndarray(
        self,
        starts: object,
        ends: object,
        *,
        collision_mask: int,
        num_threads: int = 0,
    ) -> tuple[tuple[int, RayHit], ...]:
        try:
            return super()._ray_test_indexed_hits_ndarray(
                starts,
                ends,
                collision_mask=collision_mask,
                num_threads=num_threads,
            )
        except BaseException as error:
            raise _WorkerRaycastError("worker native raycast failed") from error


@dataclass(slots=True)
class _LiveWorkerBootstrap:
    """child 在 Ready 后继续持有的 DIRECT client 与已建场景资源。"""

    client_id: int
    ready: LidarWorkerReady
    backend: PyBulletSensorBackend
    # 保留旧双雷达测试钩子；Stage4 profile 只填充 center_scanner。
    front_scanner: MultiLineLidar | None
    rear_scanner: MultiLineLidar | None
    center_scanner: MultiLineLidar | None
    codec: ProtoCodec | V2ProtoCodec
    terrain_body_ids: tuple[int, ...]
    obstacle_records: dict[int, _WorkerObstacleRecord]
    scene_state_unknown: bool = False


def _snapshot_to_spec(snapshot: ObstacleSnapshot) -> ObstacleSpec:
    if snapshot.geometry is None:
        raise ValueError("obstacle snapshot geometry is required")
    return ObstacleSpec(
        snapshot.logical_id,
        snapshot.mode,
        snapshot.geometry,
        snapshot.position,
        snapshot.orientation,
        snapshot.path,
    )


def _record_snapshot(record: _WorkerObstacleRecord) -> ObstacleSnapshot:
    spec = record.spec
    return ObstacleSnapshot(
        spec.logical_id,
        record.body_id,
        spec.mode,
        spec.geometry.shape,
        spec.position,
        spec.orientation,
        spec.path,
        spec.geometry,
    )


def _create_worker_body(client_id: int, spec: ObstacleSpec) -> int:
    """包内物理 seam：真实路径始终创建正式可碰撞障碍物。"""
    return _create_obstacle_body(client_id, spec, temporary=False)


def _remove_worker_body(client_id: int, body_id: int) -> None:
    """严格删除并复核 child 自己拥有的障碍物 body。"""
    _remove_committed_body_strict(client_id, body_id)


def _worker_body_ids(client_id: int) -> frozenset[int]:
    """枚举 child 当前 body 集，用于事务回滚后的所有权证明。"""
    import pybullet as p

    return frozenset(
        p.getBodyUniqueId(index, physicsClientId=client_id)
        for index in range(p.getNumBodies(client_id))
    )


def _update_worker_body(client_id: int, record: _WorkerObstacleRecord, spec: ObstacleSpec) -> None:
    """用父进程冻结位姿更新保留 body；worker 不自行推进路径。"""
    update_kinematic_obstacle(
        client_id,
        record.body_id,
        position=spec.position,
        orientation=spec.orientation,
        linear_velocity=(0.0, 0.0, 0.0),
    )


def _disconnect_direct_client(client_id: int) -> None:
    """集中关闭 child 自己拥有的 DIRECT client。"""
    import pybullet as p

    p.disconnect(client_id)


def _startup_failure(
    process_id: int,
    phase: str,
    error: BaseException,
) -> LidarWorkerStartupFailure:
    return LidarWorkerStartupFailure(
        _PROTOCOL_VERSION,
        process_id,
        phase,
        _STARTUP_PHASE_CODES[phase],
        _bounded_exception_detail(phase, error),
    )


def _bootstrap_live_worker(
    world_spec: object,
    *,
    forced_failure_phase: str | None = None,
    prewarm_stage4: bool = True,
) -> _LiveWorkerBootstrap | LidarWorkerStartupFailure:
    """建立 child 的真实 DIRECT 世界；成功时不关闭 client，供后续请求复用。"""
    process_id = os.getpid()
    client_id: int | None = None
    phase = "world_build"
    try:
        validated_spec = LidarWorkerWorldSpec(
            world_spec.protocol_version,
            world_spec.experiment_config,
            world_spec.scene_document,
            world_spec.world_digest,
            world_spec.profile,
        )
        import pybullet as p

        # child 固定 DIRECT，完全忽略 ExperimentConfig.mode。
        client_id = p.connect(p.DIRECT)
        if client_id < 0:
            raise RuntimeError("PyBullet DIRECT connection failed")
        world, obstacle_manager = build_world_from_scene_document(
            client_id,
            validated_spec.experiment_config,
            validated_spec.scene_document,
        )
        backend = _WorkerSensorBackend(client_id, world.active_robot.robot.robot_id)
        # manager 产生的是 child 自己创建的 body id，绝不跨进程传递父 body id。
        backend.bind_scene(
            world.scene.body_ids,
            obstacle_manager.snapshot(include_body_id=True),
        )
        initial_records = {
            snapshot.logical_id: _WorkerObstacleRecord(
                _snapshot_to_spec(snapshot),
                snapshot.body_id,
            )
            for snapshot in obstacle_manager.snapshot(include_body_id=True)
            if snapshot.body_id is not None
        }
        codec = ProtoCodec()
        if validated_spec.profile == "stage3":
            front = MultiLineLidar(
                backend,
                validated_spec.scene_document.sensors.lidar,
                validated_spec.scene_document.sensors.mounts.lidar_front,
                frame_id="lidar_front",
                lidar_id=1,
            )
            rear = MultiLineLidar(
                backend,
                validated_spec.scene_document.sensors.lidar,
                validated_spec.scene_document.sensors.mounts.lidar_rear,
                frame_id="lidar_rear",
                lidar_id=2,
            )
            center = None

            phase = "front_preflight"
            if forced_failure_phase == phase:
                raise RuntimeError("forced front preflight failure")
            front_hash, front_duration_ns = _prewarm_scanner(
                front,
                backend,
                codec.encode,
                expected_ray_count=2880,
            )

            phase = "rear_preflight"
            if forced_failure_phase == phase:
                raise RuntimeError("forced rear preflight failure")
            rear_hash, rear_duration_ns = _prewarm_scanner(
                rear,
                backend,
                codec.encode,
                expected_ray_count=2880,
            )
            prewarm_hashes = (
                ("lidar_front", front_hash),
                ("lidar_rear", rear_hash),
            )
            prewarm_duration_ns = max(front_duration_ns, rear_duration_ns)
        else:
            # 阶段四只创建唯一中心扫描器，避免旧前后雷达进入实时物理路径。
            center = MultiLineLidar.stage4(backend, Stage4LidarProfile.realtime())
            front = None
            rear = None
            descriptor = load_v2_descriptor()
            codec = V2ProtoCodec(descriptor)
            if prewarm_stage4:
                prewarm_identity = OutputIdentity(
                    "/sim/lidar/points",
                    b"\x00" * 16,
                    descriptor.sha256,
                    1,
                    0,
                )
                phase = "center_preflight"
                if forced_failure_phase == phase:
                    raise RuntimeError("forced center preflight failure")
                center_hash, prewarm_duration_ns = _prewarm_scanner(
                    center,
                    backend,
                    lambda message: _encode_v2_lidar_payload(
                        codec,
                        message,
                        prewarm_identity,
                    ),
                    expected_ray_count=Stage4LidarProfile.realtime().ray_count,
                )
                prewarm_hashes = (("lidar_link", center_hash),)
            else:
                prewarm_hashes = ()
                prewarm_duration_ns = 0
        ready = None if validated_spec.profile == "stage4" and not prewarm_stage4 else LidarWorkerReady(
            _PROTOCOL_VERSION,
            process_id,
            validated_spec.world_digest,
            _WORKER_TOPICS_BY_PROFILE[validated_spec.profile],
            prewarm_hashes,
            prewarm_duration_ns,
        )
        return _LiveWorkerBootstrap(
            client_id,
            ready,
            backend,
            front,
            rear,
            center,
            codec,
            tuple(world.scene.body_ids),
            initial_records,
        )
    except BaseException as error:
        failure = _startup_failure(process_id, phase, error)
        if client_id is None:
            return failure
        try:
            _disconnect_direct_client(client_id)
        except BaseException as cleanup_error:
            return _startup_failure(process_id, "startup_cleanup", cleanup_error)
        return failure


def _bootstrap_worker(
    world_spec: object,
    *,
    forced_failure_phase: str | None = None,
) -> LidarWorkerReady | LidarWorkerStartupFailure:
    """供同进程测试运行真实 DIRECT bootstrap，并在返回前回收 client。"""
    result = _bootstrap_live_worker(
        world_spec,
        forced_failure_phase=forced_failure_phase,
    )
    if type(result) is LidarWorkerStartupFailure:
        return result
    try:
        _disconnect_direct_client(result.client_id)
    except BaseException as error:
        return _startup_failure(os.getpid(), "startup_cleanup", error)
    return result.ready


def _reconstruct_scan_request(value: object) -> LidarScanRequest:
    """在 child 消费 pickle 后重新执行完整帧合同校验。"""
    if type(value) is not LidarScanRequest:
        raise ValueError("worker accepts only exact LidarScanRequest values")
    return LidarScanRequest(
        value.protocol_version,
        value.job_id,
        value.captured_monotonic_ns,
        value.lifecycle_generation,
        value.pause_epoch,
        value.topic,
        value.frame_id,
        value.lidar_id,
        value.timestamp_ns,
        value.world_mount_pose,
        value.optional_base_pose,
        value.complete_obstacle_snapshots_without_body_ids,
        value.output_identity,
    )


def _reconstruct_stage4_shard_result(
    value: object,
    *,
    shard_id: int,
    process_id: int,
    assignment: tuple[int, int, int, int],
    request: LidarScanRequest,
) -> _Stage4ShardResult:
    """在 coordinator 收到 pickle 后重建并核验私有 shard Result。"""
    if type(value) is not _Stage4ShardResult or type(request) is not LidarScanRequest:
        raise ValueError("invalid Stage4 shard result")
    try:
        result = _Stage4ShardResult(
            value.shard_id, value.process_id, value.first, value.stop, value.stride,
            value.count, value.examined_count, value.job_id, value.lifecycle_generation,
            value.pause_epoch, value.topic, value.timestamp_ns, value.values,
        )
    except (AttributeError, ValueError) as error:
        raise ValueError("invalid Stage4 shard result") from error
    if (
        (result.shard_id, result.process_id, result.first, result.stop, result.stride, result.count)
        != (shard_id, process_id, *assignment)
        or (result.job_id, result.lifecycle_generation, result.pause_epoch, result.topic, result.timestamp_ns)
        != (request.job_id, request.lifecycle_generation, request.pause_epoch, request.topic, request.timestamp_ns)
    ):
        raise ValueError("Stage4 shard result identity mismatch")
    if request.output_identity is None:
        raise ValueError("Stage4 shard result requires output identity")
    _require_stage4_indexed_values(
        result.values,
        result.first,
        result.stop,
        result.stride,
    )
    _require_stage4_firing_identity(
        result.values,
        world_generation=request.output_identity.world_generation,
        sequence=request.output_identity.sequence,
    )
    return result


def _reconstruct_stage4_shard_failure(
    value: object,
    *,
    shard_id: int,
    process_id: int,
    assignment: tuple[int, int, int, int],
    request: LidarScanRequest,
) -> _Stage4ShardFailure:
    """在 coordinator 收到 pickle 后重建并核验私有 shard Failure。"""
    if type(value) is not _Stage4ShardFailure or type(request) is not LidarScanRequest:
        raise ValueError("invalid Stage4 shard failure")
    try:
        failure = _Stage4ShardFailure(
            value.shard_id, value.process_id, value.first, value.stop, value.stride,
            value.count, value.job_id, value.lifecycle_generation, value.pause_epoch,
            value.topic, value.timestamp_ns, value.stable_error_code, value.bounded_detail,
        )
    except (AttributeError, ValueError) as error:
        raise ValueError("invalid Stage4 shard failure") from error
    if (
        (failure.shard_id, failure.process_id, failure.first, failure.stop, failure.stride, failure.count)
        != (shard_id, process_id, *assignment)
        or (failure.job_id, failure.lifecycle_generation, failure.pause_epoch, failure.topic, failure.timestamp_ns)
        != (request.job_id, request.lifecycle_generation, request.pause_epoch, request.topic, request.timestamp_ns)
    ):
        raise ValueError("Stage4 shard failure identity mismatch")
    return failure


def _reconstruct_stage4_shard_stopped(
    value: object,
    *,
    shard_id: int,
    process_id: int,
) -> _Stage4ShardStopped:
    """在 coordinator 收到 pickle 后重建并核验私有 shard Stopped。"""
    if type(value) is not _Stage4ShardStopped:
        raise ValueError("invalid Stage4 shard Stopped")
    try:
        stopped = _Stage4ShardStopped(value.shard_id, value.process_id)
    except (AttributeError, ValueError) as error:
        raise ValueError("invalid Stage4 shard Stopped") from error
    if (stopped.shard_id, stopped.process_id) != (shard_id, process_id):
        raise ValueError("Stage4 shard Stopped identity mismatch")
    return stopped


def _scan_failure(
    request: LidarScanRequest,
    error_code: str,
    stage: str,
    error: BaseException,
    started_ns: int,
) -> LidarScanFailure:
    return LidarScanFailure(
        _PROTOCOL_VERSION,
        request.job_id,
        request.lifecycle_generation,
        request.pause_epoch,
        request.topic,
        request.timestamp_ns,
        error_code,
        _bounded_exception_detail(stage, error),
        time.monotonic_ns() - started_ns,
    )


def _reconcile_obstacles(
    live: _LiveWorkerBootstrap,
    snapshots: tuple[ObstacleSnapshot, ...],
) -> None:
    """按完整逻辑集合对齐 child 世界；先建结构候选，再更新映射与分类。"""
    target_specs = {
        snapshot.logical_id: _snapshot_to_spec(snapshot)
        for snapshot in snapshots
    }
    current = live.obstacle_records
    replacement_ids = {
        logical_id
        for logical_id, target in target_specs.items()
        if logical_id in current
        and (
            current[logical_id].spec.mode != target.mode
            or current[logical_id].spec.geometry != target.geometry
        )
    }
    create_ids = tuple(
        logical_id
        for logical_id in target_specs
        if logical_id not in current or logical_id in replacement_ids
    )
    remove_ids = tuple(
        logical_id
        for logical_id in current
        if logical_id not in target_specs or logical_id in replacement_ids
    )

    try:
        transaction_start_body_ids = _worker_body_ids(live.client_id)
    except BaseException as error:
        raise _SceneStateUnknownError("cannot enumerate worker bodies before reconcile") from error

    candidates: dict[int, _WorkerObstacleRecord] = {}
    try:
        for logical_id in create_ids:
            spec = target_specs[logical_id]
            candidates[logical_id] = _WorkerObstacleRecord(
                spec,
                _create_worker_body(live.client_id, spec),
            )
    except BaseException as creation_error:
        cleanup_failed = False
        for record in candidates.values():
            try:
                _remove_worker_body(live.client_id, record.body_id)
            except BaseException:
                cleanup_failed = True
        try:
            rollback_body_ids = _worker_body_ids(live.client_id)
        except BaseException as diagnosis_error:
            raise _SceneStateUnknownError(
                "candidate creation failed and rollback could not be verified"
            ) from diagnosis_error
        if cleanup_failed or rollback_body_ids != transaction_start_body_ids:
            raise _SceneStateUnknownError(
                "candidate creation failed and did not restore the original body set"
            ) from creation_error
        raise _SceneReconcileError("candidate obstacle creation failed") from creation_error

    try:
        for logical_id in remove_ids:
            _remove_worker_body(live.client_id, current[logical_id].body_id)
    except BaseException as removal_error:
        rollback_failed = False
        for record in candidates.values():
            try:
                _remove_worker_body(live.client_id, record.body_id)
            except BaseException:
                rollback_failed = True
        try:
            remaining_body_ids = _worker_body_ids(live.client_id)
        except BaseException:
            remaining_body_ids = frozenset()
            rollback_failed = True

        restored: dict[int, _WorkerObstacleRecord] = {}
        for logical_id, original in current.items():
            if original.body_id in remaining_body_ids:
                restored[logical_id] = original
                continue
            try:
                restored[logical_id] = _WorkerObstacleRecord(
                    original.spec,
                    _create_worker_body(live.client_id, original.spec),
                )
            except BaseException:
                rollback_failed = True
        try:
            final_body_ids = _worker_body_ids(live.client_id)
        except BaseException:
            final_body_ids = frozenset()
            rollback_failed = True
        restored_ids = tuple(record.body_id for record in restored.values())
        if (
            len(restored) != len(current)
            or len(set(restored_ids)) != len(restored_ids)
            or any(body_id not in final_body_ids for body_id in restored_ids)
        ):
            rollback_failed = True
        if not rollback_failed:
            try:
                live.backend.bind_scene(
                    live.terrain_body_ids,
                    tuple(_record_snapshot(record) for record in restored.values()),
                )
            except BaseException:
                rollback_failed = True
        if rollback_failed:
            raise _SceneStateUnknownError(
                "old body removal failed and rollback could not be proven"
            ) from removal_error
        live.obstacle_records = restored
        raise _SceneReconcileError("old body removal failed but rollback completed") from removal_error

    next_records: dict[int, _WorkerObstacleRecord] = {}
    try:
        for snapshot in snapshots:
            logical_id = snapshot.logical_id
            spec = target_specs[logical_id]
            if logical_id in candidates:
                next_records[logical_id] = candidates[logical_id]
                continue
            record = current[logical_id]
            _update_worker_body(live.client_id, record, spec)
            record.spec = spec
            next_records[logical_id] = record
        live.backend.bind_scene(
            live.terrain_body_ids,
            tuple(_record_snapshot(record) for record in next_records.values()),
        )
    except BaseException as error:
        raise _SceneStateUnknownError(
            "worker scene mutation or category bind failed"
        ) from error
    live.obstacle_records = next_records


def _process_scan_request(
    live: _LiveWorkerBootstrap,
    raw_request: object,
) -> PreparedLidarFrame | LidarScanFailure:
    """在 child 内用冻结位姿生成一次同源点云、俯视帧与编码 bytes。"""
    request = _reconstruct_scan_request(raw_request)
    started_ns = time.monotonic_ns()
    scanner = {
        "lidar_front": live.front_scanner,
        "lidar_rear": live.rear_scanner,
        "lidar_link": live.center_scanner,
    }[request.topic]
    if scanner is None:
        return _scan_failure(
            request,
            "pointcloud_failed",
            "profile",
            RuntimeError("lidar request does not belong to the active worker profile"),
            started_ns,
        )
    try:
        _reconcile_obstacles(
            live,
            request.complete_obstacle_snapshots_without_body_ids,
        )
    except _SceneStateUnknownError as error:
        live.scene_state_unknown = True
        return _scan_failure(
            request,
            "scene_state_unknown",
            "scene reconcile",
            error,
            started_ns,
        )
    except _SceneReconcileError as error:
        return _scan_failure(
            request,
            "scene_reconcile_failed",
            "scene reconcile",
            error,
            started_ns,
        )
    if (
        request.topic == "lidar_link"
        and request.output_identity is not None
        and request.optional_base_pose is None
    ):
        try:
            if type(live.codec) is not V2ProtoCodec:
                raise RuntimeError("Stage3 worker cannot encode a v2 output identity")
            identity = _validated_v2_lidar_identity(live.codec, request.output_identity)
        except BaseException as error:
            return _scan_failure(request, "codec_failed", "codec", error, started_ns)
        try:
            payload = scanner._scan_v2_payload_at_mount(
                request.timestamp_ns,
                request.world_mount_pose,
                sequence=identity.sequence,
                world_generation=identity.world_generation,
                simulation_session_id=identity.simulation_session_id,
                descriptor_sha256=identity.descriptor_sha256,
            )
        except _WorkerRaycastError as error:
            return _scan_failure(request, "raycast_failed", "raycast", error, started_ns)
        except _V2PayloadSerializationError as error:
            return _scan_failure(request, "codec_failed", "codec", error, started_ns)
        except BaseException as error:
            return _scan_failure(request, "pointcloud_failed", "pointcloud", error, started_ns)
        if type(payload) is not bytes or not payload:
            return _scan_failure(
                request,
                "codec_failed",
                "codec",
                ValueError("codec must return nonempty exact bytes"),
                started_ns,
            )
        return PreparedLidarPayload(
            _PROTOCOL_VERSION,
            request.job_id,
            request.lifecycle_generation,
            request.pause_epoch,
            request.topic,
            request.timestamp_ns,
            payload,
            time.monotonic_ns() - started_ns,
        )
    try:
        result = scanner._scan_frozen(
            request.timestamp_ns,
            request.world_mount_pose,
            request.optional_base_pose,
        )
    except _WorkerRaycastError as error:
        return _scan_failure(request, "raycast_failed", "raycast", error, started_ns)
    except BaseException as error:
        return _scan_failure(request, "pointcloud_failed", "pointcloud", error, started_ns)

    if type(result) is LidarScanResult:
        message = result.message
        top_view: LidarTopViewFrame | None = result.top_view
    elif type(result) is LidarPointCloud:
        message = result
        top_view = None
    else:
        return _scan_failure(
            request,
            "pointcloud_failed",
            "pointcloud",
            RuntimeError("frozen scan returned an unexpected type"),
            started_ns,
        )
    try:
        if request.output_identity is None:
            if isinstance(live.codec, V2ProtoCodec):
                raise RuntimeError("Stage4 worker request is missing its v2 output identity")
            encode = getattr(live.codec, "encode", None)
            if not callable(encode):
                raise RuntimeError("worker codec must provide encode")
            payload = encode(message)
        else:
            if type(live.codec) is not V2ProtoCodec:
                raise RuntimeError("Stage3 worker cannot encode a v2 output identity")
            payload = _encode_v2_lidar_payload(
                live.codec,
                message,
                request.output_identity,
            )
        if type(payload) is not bytes or not payload:
            raise ValueError("codec must return nonempty exact bytes")
    except BaseException as error:
        return _scan_failure(request, "codec_failed", "codec", error, started_ns)
    try:
            response_type = (
                PreparedLidarFrame
                if request.optional_base_pose is not None
                else PreparedLidarPayload
            )
            if response_type is PreparedLidarFrame:
                return PreparedLidarFrame(
                    _PROTOCOL_VERSION,
                    request.job_id,
                    request.lifecycle_generation,
                    request.pause_epoch,
                    request.topic,
                    request.timestamp_ns,
                    message,
                    top_view,
                    payload,
                    time.monotonic_ns() - started_ns,
                )
            return PreparedLidarPayload(
                _PROTOCOL_VERSION,
                request.job_id,
                request.lifecycle_generation,
                request.pause_epoch,
                request.topic,
                request.timestamp_ns,
                payload,
                time.monotonic_ns() - started_ns,
            )
    except BaseException as error:
        return _scan_failure(request, "pointcloud_failed", "pointcloud", error, started_ns)


def _stage4_shard_values(
    live: _LiveWorkerBootstrap,
    spec: _Stage4ShardSpec,
    mount: Pose,
    *,
    world_generation: int,
    sequence: int,
) -> tuple[tuple[int, tuple[int, float, float, float, int, int, int]], ...]:
    """在独立 DIRECT 世界中执行所属交错 global ray index 的正式射线批次。"""
    if live.center_scanner is None:
        raise RuntimeError("Stage4 shard has no center scanner")
    scanner = live.center_scanner
    global_slots = range(spec.first, spec.stop, spec.stride)
    shard_starts, shard_ends = scanner._stage4_world_rays_for_slots(
        mount,
        pattern_version=MID360_PATTERN_VERSION,
        world_generation=world_generation,
        sequence=sequence,
        global_slots=global_slots,
    )
    if (
        shard_starts.shape != (spec.count, 3)
        or shard_ends.shape != (spec.count, 3)
        or not shard_starts.flags.c_contiguous
        or not shard_ends.flags.c_contiguous
        or shard_starts.flags.writeable
        or shard_ends.flags.writeable
    ):
        raise RuntimeError("Stage4 shard rays must be readonly C-order (2880, 3) arrays")
    indexed_hits = live.backend._ray_test_indexed_hits_ndarray(
        shard_starts,
        shard_ends,
        collision_mask=LIDAR_VISIBLE_GROUP,
        num_threads=_stage4_realtime_shard_thread_count(spec.shard_id),
    )
    world_points = tuple(hit.hit_position for _index, hit in indexed_hits)
    local_points = live.backend.inverse_transform_points_prevalidated(
        mount,
        world_points,
    )
    return tuple(
        (spec.first + local_index * spec.stride, point_value)
        for (local_index, hit), local_point in zip(indexed_hits, local_points, strict=True)
        if (
            point_value := scanner._stage4_point_values_from_hit(
                spec.first + local_index * spec.stride,
                hit,
                local_point,
                pattern_version=MID360_PATTERN_VERSION,
                world_generation=world_generation,
                sequence=sequence,
            )
        ) is not None
    )


def _stage4_shard_prewarm(
    live: _LiveWorkerBootstrap,
    spec: _Stage4ShardSpec,
) -> _Stage4ShardPrewarm:
    """用 shard 自己的正式 range/线程配额做一次真实启动预热。"""
    if type(live.codec) is not V2ProtoCodec:
        raise RuntimeError("Stage4 shard prewarm requires a v2 codec")
    started_ns = time.monotonic_ns()
    values = _stage4_shard_values(
        live,
        spec,
        _world_mount_pose(live.backend, live.center_scanner),
        world_generation=1,
        sequence=0,
    )
    identity = OutputIdentity(
        "/sim/lidar/points",
        b"\x00" * 16,
        live.codec._descriptor.sha256,
        1,
        0,
    )
    return _Stage4ShardPrewarm(
        spec.shard_id,
        os.getpid(),
        spec.world_spec.world_digest,
        spec.first,
        spec.stop,
        spec.stride,
        spec.count,
        spec.count,
        0,
        1,
        0,
        "lidar_link",
        0,
        identity,
        values,
        time.monotonic_ns() - started_ns,
    )


def _stage4_shard_scan(
    live: _LiveWorkerBootstrap,
    request: LidarScanRequest,
    spec: _Stage4ShardSpec,
) -> _Stage4ShardResult:
    """在独立 DIRECT 世界中只处理所属交错 global ray index。"""
    _reconcile_obstacles(live, request.complete_obstacle_snapshots_without_body_ids)
    if request.output_identity is None:
        raise ValueError("Stage4 shard request requires output_identity")
    values = _stage4_shard_values(
        live,
        spec,
        request.world_mount_pose,
        world_generation=request.output_identity.world_generation,
        sequence=request.output_identity.sequence,
    )
    return _Stage4ShardResult(
        spec.shard_id,
        os.getpid(),
        spec.first,
        spec.stop,
        spec.stride,
        spec.count,
        spec.count,
        request.job_id,
        request.lifecycle_generation,
        request.pause_epoch,
        request.topic,
        request.timestamp_ns,
        values,
    )


def stage4_shard_entrypoint(
    request_receiver: Connection,
    response_sender: Connection,
    spec: _Stage4ShardSpec,
) -> None:
    """parent-owned sibling shard：独立 DIRECT/world，绝不自行再 spawn。"""
    result = _bootstrap_live_worker(spec.world_spec, prewarm_stage4=False)
    if type(result) is LidarWorkerStartupFailure:
        response_sender.send(result)
        response_sender.close()
        request_receiver.close()
        raise SystemExit(1)
    live = result
    normal_stop = False
    try:
        try:
            prewarm = _stage4_shard_prewarm(live, spec)
        except BaseException as error:
            response_sender.send(_startup_failure(os.getpid(), "center_preflight", error))
            raise SystemExit(1)
        gc.freeze()
        gc.disable()
        response_sender.send(
            _Stage4ShardReady(
                spec.shard_id, os.getpid(), spec.first, spec.stop, spec.stride,
                spec.count, spec.world_spec.world_digest,
            )
        )
        response_sender.send(prewarm)
        while True:
            raw_request = request_receiver.recv()
            if type(raw_request) is LidarWorkerStop:
                stop = LidarWorkerStop(raw_request.protocol_version, raw_request.process_id)
                if stop.process_id != os.getpid():
                    raise ValueError("Stage4 shard Stop used the wrong process id")
                normal_stop = True
                break
            request = None
            try:
                request = _reconstruct_scan_request(raw_request)
                response_sender.send(_stage4_shard_scan(live, request, spec))
            except BaseException as error:
                if request is None:
                    raise
                response_sender.send(
                    _Stage4ShardFailure(
                        spec.shard_id,
                        os.getpid(),
                        spec.first,
                        spec.stop,
                        spec.stride,
                        spec.count,
                        request.job_id,
                        request.lifecycle_generation,
                        request.pause_epoch,
                        request.topic,
                        request.timestamp_ns,
                        "shard_scan_failed",
                        _bounded_exception_detail("Stage4 shard scan", error),
                    )
                )
                raise SystemExit(1) from error
    finally:
        request_receiver.close()
        try:
            _disconnect_direct_client(live.client_id)
            if normal_stop:
                response_sender.send(_Stage4ShardStopped(spec.shard_id, os.getpid()))
        finally:
            response_sender.close()


def _stage4_payload_from_shards(
    request: LidarScanRequest,
    codec: V2ProtoCodec,
    responses: tuple[
        _Stage4ShardResult | _Stage4ShardPrewarm,
        _Stage4ShardResult | _Stage4ShardPrewarm,
    ],
) -> PreparedLidarPayload:
    """两 shard identity 全等后唯一一次构造并确定性编码中心 protobuf。"""
    identity = _validated_v2_lidar_identity(codec, request.output_identity)
    raw = tuple(
        (
            response.shard_id, response.first, response.stop, response.stride,
            response.count, response.examined_count, response.values,
        )
        for response in responses
    )
    for response in responses:
        if (
            response.job_id,
            response.lifecycle_generation,
            response.pause_epoch,
            response.topic,
            response.timestamp_ns,
        ) != (
            request.job_id,
            request.lifecycle_generation,
            request.pause_epoch,
            request.topic,
            request.timestamp_ns,
        ):
            raise RuntimeError("Stage4 shard response identity mismatch")
    values = _merge_stage4_shard_indexed_values(raw)
    encoded = pb.LidarPointCloud(
        timebase_ns=request.timestamp_ns,
        frame_id="lidar_link",
        lidar_id=1,
        sequence=identity.sequence,
        world_generation=identity.world_generation,
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=identity.descriptor_sha256,
    )
    for value in values:
        encoded.points.add(offset_time_ns=value[0], x=value[1], y=value[2], z=value[3], reflectivity=value[4], tag=value[5], line=value[6])
    encoded.point_num = len(values)
    return PreparedLidarPayload(
        _PROTOCOL_VERSION, request.job_id, request.lifecycle_generation, request.pause_epoch,
        request.topic, request.timestamp_ns, encoded.SerializeToString(deterministic=True), 0,
    )


def _stage4_prewarm_payload_from_shards(
    codec: V2ProtoCodec,
    partials: tuple[_Stage4ShardPrewarm, _Stage4ShardPrewarm],
    shard_pids: tuple[int, int],
    world_digest: str,
) -> tuple[bytes, int]:
    """验证两个真实 shard 预热 partial，并唯一合并编码 outer Ready 的 payload。"""
    expected_identity = OutputIdentity(
        "/sim/lidar/points", b"\x00" * 16, codec._descriptor.sha256, 1, 0
    )
    assignments = _stage4_realtime_shard_assignments()
    max_duration_ns = 0
    reconstructed_partials = []
    try:
        for shard_id, raw_partial in enumerate(partials):
            if type(raw_partial) is not _Stage4ShardPrewarm:
                raise ValueError("invalid Stage4 shard prewarm type")
            partial = _Stage4ShardPrewarm(
                raw_partial.shard_id,
                raw_partial.process_id,
                raw_partial.world_digest,
                raw_partial.first,
                raw_partial.stop,
                raw_partial.stride,
                raw_partial.count,
                raw_partial.examined_count,
                raw_partial.job_id,
                raw_partial.lifecycle_generation,
                raw_partial.pause_epoch,
                raw_partial.topic,
                raw_partial.timestamp_ns,
                _reconstruct_v2_output_identity(raw_partial.output_identity),
                raw_partial.values,
                raw_partial.duration_ns,
            )
            first, stop, stride, count = assignments[shard_id]
            if (
                partial.shard_id,
                partial.process_id,
                partial.world_digest,
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
                partial.output_identity,
            ) != (
                shard_id,
                shard_pids[shard_id],
                world_digest,
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
                expected_identity,
            ):
                raise ValueError("Stage4 shard prewarm identity mismatch")
            _require_stage4_indexed_values(
                partial.values,
                partial.first,
                partial.stop,
                partial.stride,
            )
            _require_stage4_firing_identity(
                partial.values,
                world_generation=expected_identity.world_generation,
                sequence=expected_identity.sequence,
            )
            reconstructed_partials.append(partial)
            max_duration_ns = max(max_duration_ns, partial.duration_ns)
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        raise RuntimeError("Stage4 shard prewarm identity mismatch") from error
    request = LidarScanRequest(
        _PROTOCOL_VERSION, 0, 0, 1, 0, "lidar_link", "lidar_link", 1, 0,
        Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)), None, (), expected_identity,
    )
    merge_started_ns = time.monotonic_ns()
    payload = _stage4_payload_from_shards(request, codec, tuple(reconstructed_partials))
    return payload.protobuf_payload, max_duration_ns + (time.monotonic_ns() - merge_started_ns)


def stage4_coordinator_entrypoint(
    request_receiver: Connection,
    response_sender: Connection,
    world_spec: LidarWorkerWorldSpec,
    shard_senders: tuple[Connection, Connection],
    shard_receivers: tuple[Connection, Connection],
    shard_pids: tuple[int, int],
) -> None:
    """无 DIRECT 的 Stage4 coordinator；只验证/合并 sibling shard 结果。"""
    try:
        ready_messages = tuple(receiver.recv() for receiver in shard_receivers)
        assignments = _stage4_realtime_shard_assignments()
        for shard_id, message in enumerate(ready_messages):
            if type(message) is not _Stage4ShardReady or (
                message.shard_id, message.process_id, message.first, message.stop,
                message.stride, message.count, message.world_digest
            ) != (shard_id, shard_pids[shard_id], *assignments[shard_id], world_spec.world_digest):
                raise RuntimeError("Stage4 shard startup identity mismatch")
        codec = V2ProtoCodec(load_v2_descriptor())
        prewarm_partials = tuple(receiver.recv() for receiver in shard_receivers)
        prewarm_payload, prewarm_duration_ns = _stage4_prewarm_payload_from_shards(
            codec,
            prewarm_partials,
            shard_pids,
            world_spec.world_digest,
        )
        gc.collect()
        gc.freeze()
        gc.disable()
        digest = hashlib.sha256(prewarm_payload).hexdigest()
        response_sender.send(LidarWorkerReady(_PROTOCOL_VERSION, os.getpid(), world_spec.world_digest, ("lidar_link",), (("lidar_link", digest),), prewarm_duration_ns))
        while True:
            request = request_receiver.recv()
            if type(request) is LidarWorkerStop:
                stop = LidarWorkerStop(request.protocol_version, request.process_id)
                if stop.process_id != os.getpid():
                    raise ValueError("Stage4 coordinator Stop used the wrong process id")
                shutdown_deadline = time.monotonic() + _PROCESS_ESCALATION_TIMEOUT_SEC
                for shard_id, sender in enumerate(shard_senders):
                    sender.send(LidarWorkerStop(_PROTOCOL_VERSION, shard_pids[shard_id]))
                try:
                    for shard_id, receiver in enumerate(shard_receivers):
                        ready = receiver.poll(max(0.0, shutdown_deadline - time.monotonic()))
                        if type(ready) is not bool or not ready:
                            raise TimeoutError("Stage4 shard did not return a Stopped ACK")
                        _reconstruct_stage4_shard_stopped(
                            receiver.recv(),
                            shard_id=shard_id,
                            process_id=shard_pids[shard_id],
                        )
                except BaseException as error:
                    raise RuntimeError("Stage4 shard returned an invalid Stopped ACK") from error
                response_sender.send(LidarWorkerStopped(_PROTOCOL_VERSION, os.getpid()))
                return
            validated = _reconstruct_scan_request(request)
            frame_started_ns = time.monotonic_ns()
            frame_deadline_ns = validated.captured_monotonic_ns + _LIDAR_JOB_BUDGET_NS
            try:
                for sender in shard_senders:
                    sender.send(validated)
                responses = []
                for shard_id, receiver in enumerate(shard_receivers):
                    remaining_sec = max(0.0, (frame_deadline_ns - time.monotonic_ns()) / 1_000_000_000)
                    ready = receiver.poll(remaining_sec)
                    if type(ready) is not bool or not ready:
                        raise TimeoutError("Stage4 shard frame response timed out")
                    raw_response = receiver.recv()
                    if type(raw_response) is _Stage4ShardFailure:
                        _reconstruct_stage4_shard_failure(
                            raw_response,
                            shard_id=shard_id,
                            process_id=shard_pids[shard_id],
                            assignment=assignments[shard_id],
                            request=validated,
                        )
                        raise RuntimeError("Stage4 shard scan failed")
                    responses.append(
                        _reconstruct_stage4_shard_result(
                            raw_response,
                            shard_id=shard_id,
                            process_id=shard_pids[shard_id],
                            assignment=assignments[shard_id],
                            request=validated,
                        )
                    )
                response_sender.send(_stage4_payload_from_shards(validated, codec, tuple(responses)))
            except BaseException as error:
                response_sender.send(
                    _scan_failure(validated, "pointcloud_failed", "Stage4 shard frame", error, frame_started_ns)
                )
                return
    finally:
        request_receiver.close()
        for sender in shard_senders:
            sender.close()
        for receiver in shard_receivers:
            receiver.close()
        response_sender.close()


def lidar_worker_entrypoint(
    request_receiver: Connection,
    response_sender: Connection,
    world_spec: LidarWorkerWorldSpec,
) -> None:
    """spawn child 顶层入口：原子预热后在同一 IPC 管道串行处理扫描。"""
    result = _bootstrap_live_worker(world_spec)
    envelope = result if type(result) is LidarWorkerStartupFailure else result.ready
    if type(result) is _LiveWorkerBootstrap and result.center_scanner is not None:
        # 长寿命 PyBullet 世界已完成预热；移出逐帧代际扫描，避免 10 Hz 热路径出现 full-GC 尾峰。
        gc.freeze()
    response_sender.send(envelope)
    if type(envelope) is LidarWorkerStartupFailure:
        response_sender.close()
        request_receiver.close()
        raise SystemExit(1)
    assert type(result) is _LiveWorkerBootstrap
    normal_stop = False
    try:
        while True:
            try:
                request = request_receiver.recv()
            except EOFError:
                normal_stop = True
                break
            if type(request) is LidarWorkerStop:
                stop = LidarWorkerStop(request.protocol_version, request.process_id)
                if stop.process_id != os.getpid():
                    raise ValueError("lidar worker Stop used the wrong process id")
                normal_stop = True
                break
            response_sender.send(_process_scan_request(result, request))
            if result.scene_state_unknown:
                raise SystemExit(1)
    finally:
        request_receiver.close()
        try:
            # ACK 只能证明 child 已经释放自己的 PyBullet DIRECT client。
            _disconnect_direct_client(result.client_id)
            if normal_stop:
                response_sender.send(
                    LidarWorkerStopped(_PROTOCOL_VERSION, os.getpid())
                )
        finally:
            response_sender.close()


def startup_error_from_process_start(error: BaseException) -> LidarWorkerStartupError:
    """父 Process.start 同步异常没有 child 信封，固定归类为启动失败。"""
    return LidarWorkerStartupError(
        "worker_start_failed",
        _bounded_exception_detail("process start", error),
    )


def receive_worker_startup_envelope(
    response_receiver: Connection,
    *,
    timeout_sec: float,
    expected_process_id: int,
    expected_world_digest: str | None = None,
) -> LidarWorkerReady:
    """只接受一个精确 Ready；失败、EOF、超时均不伪造另一种启动结果。"""
    if type(timeout_sec) not in {int, float} or timeout_sec <= 0:
        raise ValueError("timeout_sec must be positive")
    expected_pid = _require_uint64("expected_process_id", expected_process_id)
    if not response_receiver.poll(float(timeout_sec)):
        raise LidarWorkerStartupError(
            "worker_exited",
            "lidar worker startup timed out without a valid envelope",
        )
    try:
        envelope = response_receiver.recv()
    except EOFError as error:
        raise LidarWorkerStartupError(
            "worker_exited",
            "lidar worker exited without a valid startup envelope",
        ) from error
    try:
        if type(envelope) is LidarWorkerReady:
            ready = LidarWorkerReady(
                envelope.protocol_version,
                envelope.process_id,
                envelope.world_digest,
                envelope.prewarmed_topics,
                envelope.prewarm_payload_sha256_by_topic,
                envelope.prewarm_max_scan_wall_duration_ns,
            )
            if ready.process_id != expected_pid:
                raise ValueError("Ready process_id differs from spawned worker")
            if expected_world_digest is not None:
                expected_digest = _require_sha256(
                    "expected_world_digest",
                    expected_world_digest,
                )
                if ready.world_digest != expected_digest:
                    raise ValueError("Ready world_digest differs from parent world spec")
            return ready
        if type(envelope) is LidarWorkerStartupFailure:
            failure = LidarWorkerStartupFailure(
                envelope.protocol_version,
                envelope.process_id,
                envelope.phase,
                envelope.stable_error_code,
                envelope.bounded_detail,
            )
            if failure.process_id != expected_pid:
                raise ValueError("StartupFailure process_id differs from spawned worker")
            raise LidarWorkerStartupError(
                failure.stable_error_code,
                failure.bounded_detail,
                startup_failure=failure,
            )
    except LidarWorkerStartupError:
        raise
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        raise LidarWorkerStartupError(
            "worker_exited",
            "lidar worker returned an invalid startup envelope",
        ) from error
    raise LidarWorkerStartupError(
        "worker_exited",
        "lidar worker returned an invalid startup envelope",
    )


def _close_failed_start(
    process: multiprocessing.Process,
    request_sender: Connection,
    response_receiver: Connection,
) -> None:
    """启动握手失败时只回收本次创建的 child，避免遗留 DIRECT 世界。"""
    request_sender.close()
    try:
        _reap_owned_process(process, initial_join_timeout_sec=1.0)
    finally:
        response_receiver.close()


def _reap_owned_process(
    process: multiprocessing.Process,
    *,
    initial_join_timeout_sec: float,
) -> bool:
    """有界回收唯一 owned child；返回是否用过 terminate/kill。"""
    process.join(initial_join_timeout_sec)
    if not process.is_alive():
        return False
    process.terminate()
    process.join(_PROCESS_ESCALATION_TIMEOUT_SEC)
    if not process.is_alive():
        return True
    process.kill()
    process.join(_PROCESS_ESCALATION_TIMEOUT_SEC)
    if process.is_alive():
        raise RuntimeError("owned lidar worker remained alive after kill")
    return True


def start_lidar_worker(
    world_spec: LidarWorkerWorldSpec,
    *,
    startup_timeout_sec: float,
) -> LidarWorkerHandle:
    """使用 spawn 启动 child，并只接受完整双雷达预热后的 Ready。"""
    if type(world_spec) is not LidarWorkerWorldSpec:
        raise ValueError("world_spec must be an exact LidarWorkerWorldSpec")
    if type(startup_timeout_sec) not in {int, float} or startup_timeout_sec <= 0:
        raise ValueError("startup_timeout_sec must be positive")
    context = multiprocessing.get_context("spawn")
    if world_spec.profile == "stage4":
        return _start_stage4_coordinator_worker(
            context,
            world_spec,
            float(startup_timeout_sec),
        )
    request_receiver, request_sender = context.Pipe(False)
    response_receiver, response_sender = context.Pipe(False)
    process = context.Process(
        target=lidar_worker_entrypoint,
        args=(request_receiver, response_sender, world_spec),
        daemon=False,
    )
    try:
        process.start()
    except BaseException as error:
        request_receiver.close()
        response_sender.close()
        request_sender.close()
        response_receiver.close()
        raise startup_error_from_process_start(error) from error
    request_receiver.close()
    response_sender.close()
    try:
        ready = receive_worker_startup_envelope(
            response_receiver,
            timeout_sec=float(startup_timeout_sec),
            expected_process_id=process.pid,
            expected_world_digest=world_spec.world_digest,
        )
    except LidarWorkerStartupError:
        _close_failed_start(process, request_sender, response_receiver)
        raise
    return LidarWorkerHandle(process, request_sender, response_receiver, ready)


def _start_stage4_coordinator_worker(
    context: multiprocessing.context.BaseContext,
    world_spec: LidarWorkerWorldSpec,
    startup_timeout_sec: float,
) -> LidarWorkerHandle:
    """由 parent 同一 spawn context 创建 coordinator 与两个 sibling DIRECT shard。"""
    outer_request_receiver, outer_request_sender = context.Pipe(False)
    outer_response_receiver, outer_response_sender = context.Pipe(False)
    shard_processes: list[multiprocessing.Process] = []
    parent_endpoints: list[Connection] = [outer_request_sender, outer_response_receiver]
    coordinator_senders: list[Connection] = []
    coordinator_receivers: list[Connection] = []
    child_endpoints: list[Connection] = [outer_request_receiver, outer_response_sender]
    coordinator: multiprocessing.Process | None = None
    coordinator_started = False
    try:
        for shard_id, (first, stop, stride, count) in enumerate(_stage4_realtime_shard_assignments()):
            shard_request_receiver, coordinator_sender = context.Pipe(False)
            coordinator_receiver, shard_response_sender = context.Pipe(False)
            process = context.Process(
                target=stage4_shard_entrypoint,
                args=(
                    shard_request_receiver,
                    shard_response_sender,
                    _Stage4ShardSpec(shard_id, first, stop, stride, count, world_spec),
                ),
                daemon=False,
            )
            coordinator_senders.append(coordinator_sender)
            coordinator_receivers.append(coordinator_receiver)
            child_endpoints.extend((shard_request_receiver, shard_response_sender))
            process.start()
            shard_processes.append(process)
        coordinator = context.Process(
            target=stage4_coordinator_entrypoint,
            args=(
                outer_request_receiver,
                outer_response_sender,
                world_spec,
                tuple(coordinator_senders),
                tuple(coordinator_receivers),
                tuple(process.pid for process in shard_processes),
            ),
            daemon=False,
        )
        coordinator.start()
        coordinator_started = True
        for endpoint in child_endpoints:
            endpoint.close()
        for endpoint in coordinator_senders + coordinator_receivers:
            endpoint.close()
        ready = receive_worker_startup_envelope(
            outer_response_receiver,
            timeout_sec=startup_timeout_sec,
            expected_process_id=coordinator.pid,
            expected_world_digest=world_spec.world_digest,
        )
        return LidarWorkerHandle(
            coordinator,
            outer_request_sender,
            outer_response_receiver,
            ready,
            tuple(shard_processes),
        )
    except BaseException as error:
        cleanup_actions = [
            endpoint.close
            for endpoint in child_endpoints + parent_endpoints + coordinator_senders + coordinator_receivers
        ]
        cleanup_actions.extend(
            lambda process=process: _reap_owned_process(process, initial_join_timeout_sec=0.0)
            for process in reversed(shard_processes)
        )
        if coordinator_started and coordinator is not None:
            cleanup_actions.append(
                lambda: _reap_owned_process(coordinator, initial_join_timeout_sec=0.0)
            )
        _run_cleanup_actions(tuple(cleanup_actions))
        if isinstance(error, LidarWorkerStartupError):
            raise
        raise startup_error_from_process_start(error) from error
