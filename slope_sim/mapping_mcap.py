"""严格验证 Golf 回放 MCAP，并建立不持有点云 payload 的轻量索引。"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
from pathlib import Path
from typing import Iterator

from mcap.exceptions import McapError
from mcap.reader import make_reader

from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
from slope_sim.interfaces.v2.models import (
    ImuAttitudeV2,
    LidarPointCloudV2,
    RtkStateV2,
)
from slope_sim.interfaces.v2.topics import V2_BY_TOPIC, V2_TOPICS
from slope_sim.mapping_replay import RecoveredPoseNode, recover_pose_node


_MANIFEST_NAME = "slope_sim.session_manifest"
_SCHEMA_NAME = "slope_sim.interfaces.v2"
_LIDAR_TOPIC = "/sim/lidar/points"
_RTK_TOPIC = "/sim/rtk/state"
_IMU_TOPIC = "/sim/imu/attitude"
_RESULT_KEYS = frozenset(
    {"clean_shutdown", "mcap", "recorded_count", "role", "topics"}
)
_MANIFEST_KEYS = frozenset(
    {
        "simulation_session_id",
        "descriptor_sha256",
        "world_generation",
        "scene_id",
        "lidar_pattern_version",
        "lidar_pattern_sha256",
    }
)
_PATTERN_VERSION = "livox-mid360-800000-v1"
_PATTERN_SHA256 = bytes.fromhex(
    "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca"
)
_LIDAR_FRAME_PERIOD_NS = 100_000_000
_LIDAR_FIRING_INTERVAL_NS = 5_000
_LIDAR_LAST_FIRING_NS = 99_995_000
_LIDAR_SLOT_COUNT = 20_000
_UINT64_MAX = (1 << 64) - 1


class MappingMcapError(ValueError):
    """MCAP、Recorder 结果或其冻结会话合同不一致。"""


@dataclass(frozen=True, slots=True)
class MappingSessionIdentity:
    """manifest 中与每条 v2 payload 绑定的会话身份。"""

    simulation_session_id: bytes
    descriptor_sha256: bytes
    world_generation: int
    scene_id: str
    lidar_pattern_version: str
    lidar_pattern_sha256: bytes


@dataclass(frozen=True, slots=True)
class _FileFingerprint:
    device: int
    inode: int
    size: int
    mtime_ns: int


@dataclass(frozen=True, slots=True)
class MappingSessionIndex:
    """仅保留随机访问所需的小型会话信息；LiDAR 点在迭代时重新读取。"""

    mcap_path: Path
    identity: MappingSessionIdentity
    topic_counts: tuple[tuple[str, int], ...]
    message_count: int
    lidar_frame_times_ns: tuple[int, ...]
    pose_nodes: tuple[RecoveredPoseNode, ...]
    _fingerprint: _FileFingerprint = field(repr=False)

    def iter_lidar_frames(self) -> Iterator[LidarPointCloudV2]:
        """重开已索引文件并逐帧解码 LiDAR，不缓存全会话点对象。"""
        descriptor = load_v2_descriptor()
        codec = V2ProtoCodec(descriptor)
        try:
            with self.mcap_path.open("rb") as stream:
                _require_same_file(os.fstat(stream.fileno()), self._fingerprint)
                _require_same_file(self.mcap_path.stat(), self._fingerprint)
                reader = make_reader(stream, validate_crcs=True)
                frame_index = 0
                for schema, channel, message in reader.iter_messages(
                    topics=_LIDAR_TOPIC,
                    log_time_order=False,
                ):
                    if (
                        schema is None
                        or schema.name != _SCHEMA_NAME
                        or channel.topic != _LIDAR_TOPIC
                    ):
                        raise MappingMcapError("streamed LiDAR channel/schema changed")
                    cloud = codec.decode_lidar_point_cloud(message.data)
                    if (
                        frame_index >= len(self.lidar_frame_times_ns)
                        or cloud.timebase_ns != self.lidar_frame_times_ns[frame_index]
                        or message.sequence != cloud.sequence
                        or message.log_time != cloud.timebase_ns
                        or message.publish_time != cloud.timebase_ns
                        or codec.encode(cloud).payload != message.data
                    ):
                        raise MappingMcapError("streamed LiDAR frame differs from its index")
                    _require_payload_identity(cloud, self.identity)
                    _validate_lidar_cloud(cloud)
                    frame_index += 1
                    yield cloud
                if frame_index != len(self.lidar_frame_times_ns):
                    raise MappingMcapError("streamed LiDAR frame count differs from its index")
                _require_same_file(os.fstat(stream.fileno()), self._fingerprint)
                _require_same_file(self.mcap_path.stat(), self._fingerprint)
        except MappingMcapError:
            raise
        except (McapError, OSError, ValueError, TypeError) as error:
            raise MappingMcapError("indexed MCAP changed since indexing") from error


def _fingerprint(stat_result: os.stat_result) -> _FileFingerprint:
    return _FileFingerprint(
        stat_result.st_dev,
        stat_result.st_ino,
        stat_result.st_size,
        stat_result.st_mtime_ns,
    )


def _require_same_file(
    stat_result: os.stat_result,
    expected: _FileFingerprint,
) -> None:
    if _fingerprint(stat_result) != expected:
        raise MappingMcapError("indexed MCAP changed since indexing")


def _require_regular_path(value: Path, *, name: str) -> Path:
    if not isinstance(value, Path):
        raise MappingMcapError(f"{name} must be a Path")
    normalized = Path(os.path.normpath(str(value)))
    if not value.is_absolute() or normalized != value or not value.is_file():
        raise MappingMcapError(f"{name} must be an absolute normalized regular file")
    return value


def _parse_lower_hex(value: object, *, size: int, name: str) -> bytes:
    if not isinstance(value, str) or len(value) != size * 2 or value != value.lower():
        raise MappingMcapError(f"{name} must be canonical lowercase hexadecimal")
    try:
        parsed = bytes.fromhex(value)
    except ValueError as error:
        raise MappingMcapError(f"{name} must be canonical lowercase hexadecimal") from error
    if len(parsed) != size or parsed.hex() != value:
        raise MappingMcapError(f"{name} must be canonical lowercase hexadecimal")
    return parsed


def _manifest_identity(metadata: dict[str, str]) -> MappingSessionIdentity:
    if set(metadata) != _MANIFEST_KEYS:
        raise MappingMcapError("MCAP session manifest fields are not exact")
    try:
        session_id = _parse_lower_hex(
            metadata["simulation_session_id"],
            size=16,
            name="simulation_session_id",
        )
        descriptor_sha256 = _parse_lower_hex(
            metadata["descriptor_sha256"],
            size=32,
            name="descriptor_sha256",
        )
        world_text = metadata["world_generation"]
        world_generation = int(world_text)
        scene_id = metadata["scene_id"]
        pattern_version = metadata["lidar_pattern_version"]
        pattern_sha256 = _parse_lower_hex(
            metadata["lidar_pattern_sha256"],
            size=32,
            name="lidar_pattern_sha256",
        )
    except (KeyError, TypeError, ValueError) as error:
        raise MappingMcapError("MCAP session manifest is incomplete") from error
    if (
        not isinstance(world_text, str)
        or str(world_generation) != world_text
        or not 0 < world_generation <= _UINT64_MAX
        or not isinstance(scene_id, str)
        or not scene_id
        or pattern_version != _PATTERN_VERSION
        or pattern_sha256 != _PATTERN_SHA256
    ):
        raise MappingMcapError("MCAP session manifest is invalid")
    return MappingSessionIdentity(
        session_id,
        descriptor_sha256,
        world_generation,
        scene_id,
        pattern_version,
        pattern_sha256,
    )


def _decode_message(codec: V2ProtoCodec, topic: str, payload: bytes) -> object:
    if topic == "/sim/wheel/command":
        return codec.decode_wheel_command(payload)
    if topic == "/sim/wheel/state":
        return codec.decode_wheel_state(payload)
    if topic == _LIDAR_TOPIC:
        return codec.decode_lidar_point_cloud(payload)
    if topic == _RTK_TOPIC:
        return codec.decode_rtk_state(payload)
    if topic == _IMU_TOPIC:
        return codec.decode_imu_attitude(payload)
    raise MappingMcapError(f"unexpected MCAP topic: {topic}")


def _message_timestamp(model: object) -> int:
    if type(model) is LidarPointCloudV2:
        return model.timebase_ns
    return model.timestamp_ns  # type: ignore[attr-defined, no-any-return]


def _validate_lidar_cloud(cloud: LidarPointCloudV2) -> None:
    if cloud.frame_id != "lidar_link" or cloud.lidar_id != 1:
        raise MappingMcapError("LiDAR frame_id/lidar_id must be lidar_link/1")
    if cloud.point_num > _LIDAR_SLOT_COUNT:
        raise MappingMcapError("LiDAR frame exceeds the frozen 20,000 firing slots")
    previous_offset = -1
    for point in cloud.points:
        offset = point.offset_time_ns
        if (
            offset % _LIDAR_FIRING_INTERVAL_NS != 0
            or offset > _LIDAR_LAST_FIRING_NS
            or offset <= previous_offset
        ):
            raise MappingMcapError(
                "LiDAR offsets must be strictly increasing 5 us firing slots"
            )
        if cloud.timebase_ns > _UINT64_MAX - offset:
            raise MappingMcapError("LiDAR absolute point timestamp exceeds uint64")
        previous_offset = offset


def _require_payload_identity(model: object, identity: MappingSessionIdentity) -> None:
    if (
        model.simulation_session_id != identity.simulation_session_id  # type: ignore[attr-defined]
        or model.descriptor_sha256 != identity.descriptor_sha256  # type: ignore[attr-defined]
        or model.world_generation != identity.world_generation  # type: ignore[attr-defined]
    ):
        raise MappingMcapError("v2 payload identity differs from the MCAP manifest")


def _update_topic_progress(
    topic: str,
    sequence: int,
    timestamp_ns: int,
    progress: dict[str, tuple[int, int | None]],
) -> None:
    expected_sequence, previous_timestamp = progress[topic]
    if sequence != expected_sequence:
        raise MappingMcapError(f"{topic} sequence must start at 0 and remain continuous")
    if previous_timestamp is not None:
        period_ns = 1_000_000_000 // V2_BY_TOPIC[topic].rate_hz
        if timestamp_ns != previous_timestamp + period_ns:
            raise MappingMcapError(f"{topic} timestamps must follow its frozen cadence")
    progress[topic] = (expected_sequence + 1, timestamp_ns)


def _unique_json_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    document: dict[str, object] = {}
    for key, value in pairs:
        if key in document:
            raise MappingMcapError("Recorder result contains a duplicate JSON key")
        document[key] = value
    return document


def _read_recorder_result(path: Path, mcap_path: Path) -> dict[str, object]:
    try:
        document = json.loads(
            path.read_text(encoding="utf-8"),
            object_pairs_hook=_unique_json_object,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise MappingMcapError("Recorder result is not valid UTF-8 JSON") from error
    if not isinstance(document, dict) or set(document) != _RESULT_KEYS:
        raise MappingMcapError("Recorder result must be a JSON object")
    if (
        document.get("clean_shutdown") is not True
        or document.get("role") != "recorder"
        or document.get("mcap") != str(mcap_path)
    ):
        raise MappingMcapError("Recorder result does not bind a clean MCAP session")
    recorded_count = document.get("recorded_count")
    topics = document.get("topics")
    if (
        isinstance(recorded_count, bool)
        or not isinstance(recorded_count, int)
        or recorded_count < 0
        or not isinstance(topics, dict)
        or set(topics) != set(V2_BY_TOPIC)
        or any(
            isinstance(count, bool) or not isinstance(count, int) or count < 0
            for count in topics.values()
        )
    ):
        raise MappingMcapError("Recorder result counts are invalid")
    return document


def _load_mapping_session(
    mcap_path: Path,
    recorder_result_path: Path,
) -> MappingSessionIndex:
    source = _require_regular_path(mcap_path, name="mcap_path")
    result_path = _require_regular_path(
        recorder_result_path,
        name="recorder_result_path",
    )
    recorder_result = _read_recorder_result(result_path, source)
    descriptor = load_v2_descriptor()
    codec = V2ProtoCodec(descriptor)

    with source.open("rb") as stream:
        source_fingerprint = _fingerprint(os.fstat(stream.fileno()))
        reader = make_reader(stream, validate_crcs=True)
        summary = reader.get_summary()
        if summary is None or summary.statistics is None:
            raise MappingMcapError("MCAP session is not finalized with a summary")
        statistics = summary.statistics
        if len(summary.schemas) != 1:
            raise MappingMcapError("MCAP session must contain exactly one schema")
        schema = next(iter(summary.schemas.values()))
        if (
            schema.name != _SCHEMA_NAME
            or schema.encoding != "protobuf"
            or schema.data != descriptor.serialized_file_descriptor_set
        ):
            raise MappingMcapError("MCAP schema differs from the frozen v2 descriptor")
        if len(summary.channels) != len(V2_TOPICS):
            raise MappingMcapError("MCAP session must contain exactly five v2 channels")
        seen_topics: set[str] = set()
        for channel in summary.channels.values():
            contract = V2_BY_TOPIC.get(channel.topic)
            if (
                contract is None
                or channel.topic in seen_topics
                or channel.schema_id != schema.id
                or channel.message_encoding != "protobuf"
                or channel.metadata != {"type": contract.type_name}
            ):
                raise MappingMcapError("MCAP channel differs from the frozen v2 contract")
            seen_topics.add(channel.topic)
        if seen_topics != set(V2_BY_TOPIC):
            raise MappingMcapError("MCAP session is missing a frozen v2 channel")

        metadata_records = tuple(reader.iter_metadata())
        if len(metadata_records) != 1 or metadata_records[0].name != _MANIFEST_NAME:
            raise MappingMcapError("MCAP session must contain exactly one manifest")
        identity = _manifest_identity(metadata_records[0].metadata)
        if identity.descriptor_sha256 != descriptor.sha256:
            raise MappingMcapError("MCAP manifest descriptor digest does not match the schema")

        counts = {contract.topic: 0 for contract in V2_TOPICS}
        channel_counts = {channel_id: 0 for channel_id in summary.channels}
        progress = {contract.topic: (0, None) for contract in V2_TOPICS}
        lidar_times: list[int] = []
        rtk_by_time: dict[int, RtkStateV2] = {}
        imu_by_time: dict[int, ImuAttitudeV2] = {}
        for message_schema, channel, message in reader.iter_messages(
            log_time_order=False
        ):
            contract = V2_BY_TOPIC.get(channel.topic)
            if (
                contract is None
                or message_schema is None
                or message_schema.id != schema.id
                or message.channel_id != channel.id
                or summary.channels.get(channel.id) != channel
            ):
                raise MappingMcapError("MCAP message references an unexpected channel/schema")
            model = _decode_message(codec, channel.topic, message.data)
            encoded = codec.encode(model)  # type: ignore[arg-type]
            if encoded.type_name != contract.type_name or encoded.payload != message.data:
                raise MappingMcapError(
                    "v2 payload contains unknown fields or noncanonical encoding"
                )
            sequence = model.sequence  # type: ignore[attr-defined]
            timestamp_ns = _message_timestamp(model)
            if (
                message.sequence != sequence
                or message.log_time != timestamp_ns
                or message.publish_time != timestamp_ns
            ):
                raise MappingMcapError("MCAP message envelope differs from its v2 payload")
            _require_payload_identity(model, identity)
            _update_topic_progress(
                channel.topic,
                sequence,
                timestamp_ns,
                progress,
            )
            counts[channel.topic] += 1
            channel_counts[channel.id] += 1
            if type(model) is LidarPointCloudV2:
                _validate_lidar_cloud(model)
                lidar_times.append(timestamp_ns)
            elif type(model) is RtkStateV2:
                if timestamp_ns in rtk_by_time:
                    raise MappingMcapError("RTK timestamp is duplicated")
                rtk_by_time[timestamp_ns] = model
            elif type(model) is ImuAttitudeV2:
                if timestamp_ns in imu_by_time:
                    raise MappingMcapError("IMU timestamp is duplicated")
                imu_by_time[timestamp_ns] = model

        if (
            any(count == 0 for count in counts.values())
            or statistics.schema_count != 1
            or statistics.channel_count != len(V2_TOPICS)
            or statistics.metadata_count != 1
            or statistics.message_count != sum(counts.values())
            or statistics.channel_message_counts != channel_counts
        ):
            raise MappingMcapError("MCAP summary counts differ from streamed records")
        _require_same_file(os.fstat(stream.fileno()), source_fingerprint)
        _require_same_file(source.stat(), source_fingerprint)

    if set(rtk_by_time) != set(imu_by_time):
        raise MappingMcapError("RTK and IMU pose timestamps must match")
    pose_times = set(rtk_by_time)
    if any(timestamp_ns not in pose_times for timestamp_ns in lidar_times):
        raise MappingMcapError("every LiDAR frame must have a same-time RTK/IMU pose")
    pose_nodes: list[RecoveredPoseNode] = []
    previous_orientation = None
    for timestamp_ns in sorted(rtk_by_time):
        node = recover_pose_node(
            rtk_by_time[timestamp_ns],
            imu_by_time[timestamp_ns],
            previous_orientation=previous_orientation,
        )
        pose_nodes.append(node)
        previous_orientation = node.base_pose.orientation

    result_topics = recorder_result.get("topics")
    if result_topics != counts or recorder_result.get("recorded_count") != sum(counts.values()):
        raise MappingMcapError("Recorder result counts do not match MCAP messages")
    return MappingSessionIndex(
        source,
        identity,
        tuple((contract.topic, counts[contract.topic]) for contract in V2_TOPICS),
        sum(counts.values()),
        tuple(lidar_times),
        tuple(pose_nodes),
        source_fingerprint,
    )


def load_mapping_session(
    mcap_path: Path,
    recorder_result_path: Path,
) -> MappingSessionIndex:
    """严格读取一次完整会话，返回不含 LiDAR payload 的确定性索引。"""
    try:
        return _load_mapping_session(mcap_path, recorder_result_path)
    except MappingMcapError:
        raise
    except (
        McapError,
        OSError,
        UnicodeError,
        ValueError,
        TypeError,
        ArithmeticError,
        StopIteration,
    ) as error:
        raise MappingMcapError("MCAP session validation failed") from error
