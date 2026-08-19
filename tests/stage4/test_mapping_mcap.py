"""Golf 世界回放 MCAP 的严格会话索引与流式点云合同。"""
from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, replace
import json
import os
from pathlib import Path
from typing import Callable

from mcap.writer import CompressionType, Writer
import pytest

from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
from slope_sim.interfaces.v2.models import (
    CommandAuthorityState,
    ImuAttitudeV2,
    LidarPointCloudV2,
    LidarPointV2,
    Point3dV2,
    RtkStateV2,
    WheelCommandV2,
    WheelStateV2,
)
from slope_sim.interfaces.v2.topics import V2_BY_TOPIC, V2_TOPICS


_SESSION_ID = bytes.fromhex("00112233445566778899aabbccddeeff")
_SOURCE_SESSION_ID = bytes.fromhex("ffeeddccbbaa99887766554433221100")
_WORLD_GENERATION = 7
_SCENE_ID = "mid360-golf-mapping"
_PATTERN_VERSION = "livox-mid360-800000-v1"
_PATTERN_SHA256 = "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca"


@dataclass(slots=True)
class _FixtureFrame:
    topic: str
    model: object
    mcap_sequence: int | None = None
    log_time_ns: int | None = None
    publish_time_ns: int | None = None
    payload_suffix: bytes = b""


def _valid_models() -> tuple[tuple[str, object], ...]:
    descriptor = load_v2_descriptor()
    models: list[tuple[str, object]] = []
    for sequence, timestamp_ns in enumerate(range(0, 200_000_001, 10_000_000)):
        models.extend(
            (
                (
                    "/sim/wheel/command",
                    WheelCommandV2(
                        timestamp_ns,
                        (1.0, 1.0),
                        (0.0, 0.0),
                        sequence,
                        _WORLD_GENERATION,
                        1,
                        "golf-route",
                        _SOURCE_SESSION_ID,
                        "df_mid",
                        _SESSION_ID,
                        descriptor.sha256,
                    ),
                ),
                (
                    "/sim/wheel/state",
                    WheelStateV2(
                        timestamp_ns,
                        (1.0, 1.0),
                        (0.0, 0.0),
                        sequence,
                        _WORLD_GENERATION,
                        1,
                        "df_mid",
                        _SESSION_ID,
                        descriptor.sha256,
                        CommandAuthorityState.ACTIVE,
                        "golf-route",
                        _SOURCE_SESSION_ID,
                        1,
                    ),
                ),
            )
        )
    for sequence, timestamp_ns in enumerate((0, 100_000_000, 200_000_000)):
        rtk = RtkStateV2(
            timestamp_ns,
            sequence,
            _WORLD_GENERATION,
            "world",
            Point3dV2(0.0, 0.20, 0.18),
            Point3dV2(0.0, 0.0, 0.18),
            Point3dV2(0.0, -0.20, 0.18),
            0.0,
            _SESSION_ID,
            descriptor.sha256,
        )
        imu = ImuAttitudeV2(
            timestamp_ns,
            0.0,
            0.0,
            sequence,
            _WORLD_GENERATION,
            "base_link",
            _SESSION_ID,
            descriptor.sha256,
        )
        models.extend((("/sim/rtk/state", rtk), ("/sim/imu/attitude", imu)))
        if sequence < 2:
            cloud = LidarPointCloudV2(
                timestamp_ns,
                "lidar_link",
                2,
                1,
                (
                    LidarPointV2(0, 1.0, 0.0, 0.0, 100, 1, 0),
                    LidarPointV2(5_000, 2.0, 0.0, 0.0, 120, 2, 1),
                ),
                sequence,
                _WORLD_GENERATION,
                _SESSION_ID,
                descriptor.sha256,
            )
            models.append(("/sim/lidar/points", cloud))
    return tuple(models)


def _valid_frames() -> list[_FixtureFrame]:
    return [_FixtureFrame(topic, model) for topic, model in _valid_models()]


def _payload_timestamp(model: object) -> int:
    if isinstance(model, LidarPointCloudV2):
        return model.timebase_ns
    return model.timestamp_ns  # type: ignore[attr-defined, no-any-return]


def _write_fixture(
    tmp_path: Path,
    *,
    frames: list[_FixtureFrame] | None = None,
    schema_name: str = "slope_sim.interfaces.v2",
    schema_encoding: str = "protobuf",
    schema_data: bytes | None = None,
    extra_schema: bool = False,
    omitted_channels: tuple[str, ...] = (),
    extra_channel: bool = False,
    channel_type_overrides: dict[str, str] | None = None,
    manifest_overrides: dict[str, str | None] | None = None,
    manifest_count: int = 1,
    payload_transform: Callable[[str, object, bytes], bytes] | None = None,
    finish: bool = True,
    result_overrides: dict[str, object] | None = None,
) -> tuple[Path, Path, dict[str, int]]:
    descriptor = load_v2_descriptor()
    codec = V2ProtoCodec(descriptor)
    mcap_path = (tmp_path / "golf-session.mcap").absolute()
    counts = {contract.topic: 0 for contract in V2_TOPICS}
    fixture_frames = _valid_frames() if frames is None else frames
    type_overrides = channel_type_overrides or {}
    with mcap_path.open("wb") as output:
        writer = Writer(
            output,
            compression=CompressionType.NONE,
            use_chunking=False,
            enable_crcs=True,
            enable_data_crcs=True,
        )
        writer.start(profile="protobuf", library="slope-sim-test")
        schema_id = writer.register_schema(
            name=schema_name,
            encoding=schema_encoding,
            data=(
                descriptor.serialized_file_descriptor_set
                if schema_data is None
                else schema_data
            ),
        )
        if extra_schema:
            writer.register_schema("unexpected.v2", "protobuf", b"unexpected")
        channel_ids = {
            contract.topic: writer.register_channel(
                topic=contract.topic,
                message_encoding="protobuf",
                schema_id=schema_id,
                metadata={"type": type_overrides.get(contract.topic, contract.type_name)},
            )
            for contract in V2_TOPICS
            if contract.topic not in omitted_channels
        }
        if extra_channel:
            writer.register_channel(
                topic="/sim/unexpected",
                message_encoding="protobuf",
                schema_id=schema_id,
                metadata={"type": "slope_sim.interfaces.v2.ImuAttitude"},
            )
        manifest = {
            "simulation_session_id": _SESSION_ID.hex(),
            "descriptor_sha256": descriptor.sha256.hex(),
            "world_generation": str(_WORLD_GENERATION),
            "scene_id": _SCENE_ID,
            "lidar_pattern_version": _PATTERN_VERSION,
            "lidar_pattern_sha256": _PATTERN_SHA256,
        }
        for key, value in (manifest_overrides or {}).items():
            if value is None:
                manifest.pop(key, None)
            else:
                manifest[key] = value
        for _ in range(manifest_count):
            writer.add_metadata("slope_sim.session_manifest", manifest)
        for frame in sorted(
            fixture_frames,
            key=lambda item: (_payload_timestamp(item.model), item.topic),
        ):
            if frame.topic not in channel_ids:
                continue
            encoded = codec.encode(frame.model)  # type: ignore[arg-type]
            payload = encoded.payload + frame.payload_suffix
            if payload_transform is not None:
                payload = payload_transform(frame.topic, frame.model, payload)
            timestamp_ns = _payload_timestamp(frame.model)
            writer.add_message(
                channel_ids[frame.topic],
                log_time=(timestamp_ns if frame.log_time_ns is None else frame.log_time_ns),
                publish_time=(
                    timestamp_ns
                    if frame.publish_time_ns is None
                    else frame.publish_time_ns
                ),
                sequence=(
                    frame.model.sequence  # type: ignore[attr-defined]
                    if frame.mcap_sequence is None
                    else frame.mcap_sequence
                ),
                data=payload,
            )
            counts[frame.topic] += 1
        if finish:
            writer.finish()

    result_path = (tmp_path / "recorder-result.json").absolute()
    result = {
        "clean_shutdown": True,
        "mcap": str(mcap_path),
        "recorded_count": sum(counts.values()),
        "role": "recorder",
        "topics": counts,
    }
    result.update(result_overrides or {})
    result_path.write_text(
        json.dumps(result, separators=(",", ":")) + "\n",
        encoding="utf-8",
    )
    return mcap_path, result_path, counts


def _fixture_frame(
    frames: list[_FixtureFrame],
    topic: str,
    sequence: int,
) -> _FixtureFrame:
    return next(
        frame
        for frame in frames
        if frame.topic == topic and frame.model.sequence == sequence  # type: ignore[attr-defined]
    )


def test_finalized_session_returns_lightweight_index_and_streams_lidar(
    tmp_path: Path,
) -> None:
    """索引只保留身份、计数、帧时刻和姿态；点云须在重开文件后逐帧产生。"""
    from slope_sim import mapping_mcap

    mcap_path, result_path, counts = _write_fixture(tmp_path)

    index = mapping_mcap.load_mapping_session(mcap_path, result_path)

    assert index.mcap_path == mcap_path
    assert index.identity.simulation_session_id == _SESSION_ID
    assert index.identity.descriptor_sha256 == load_v2_descriptor().sha256
    assert index.identity.world_generation == _WORLD_GENERATION
    assert index.identity.scene_id == _SCENE_ID
    assert index.identity.lidar_pattern_version == _PATTERN_VERSION
    assert index.identity.lidar_pattern_sha256 == bytes.fromhex(_PATTERN_SHA256)
    assert dict(index.topic_counts) == counts
    assert index.message_count == sum(counts.values())
    assert index.lidar_frame_times_ns == (0, 100_000_000)
    assert tuple(node.timestamp_ns for node in index.pose_nodes) == (
        0,
        100_000_000,
        200_000_000,
    )
    assert not hasattr(index, "lidar_frames")
    frames = index.iter_lidar_frames()
    assert isinstance(frames, Iterator)
    assert tuple(
        (cloud.timebase_ns, tuple(point.offset_time_ns for point in cloud.points))
        for cloud in frames
    ) == ((0, (0, 5_000)), (100_000_000, (0, 5_000)))


@pytest.mark.parametrize(
    "mutation",
    ("unclean", "wrong-path", "wrong-total", "wrong-topic-count", "extra-key"),
)
def test_recorder_result_must_exactly_bind_path_and_counts(
    tmp_path: Path,
    mutation: str,
) -> None:
    """只有 Recorder 明确 clean 且精确绑定同一 MCAP 与五 topic 计数时才可回放。"""
    from slope_sim import mapping_mcap

    mcap_path, result_path, _counts = _write_fixture(tmp_path)
    result = json.loads(result_path.read_text(encoding="utf-8"))
    if mutation == "unclean":
        result["clean_shutdown"] = False
    elif mutation == "wrong-path":
        result["mcap"] = str(mcap_path.with_name("other.mcap"))
    elif mutation == "wrong-total":
        result["recorded_count"] += 1
    elif mutation == "wrong-topic-count":
        result["topics"]["/sim/lidar/points"] += 1
    else:
        result["unexpected"] = True
    result_path.write_text(json.dumps(result), encoding="utf-8")

    with pytest.raises(mapping_mcap.MappingMcapError):
        mapping_mcap.load_mapping_session(mcap_path, result_path)


@pytest.mark.parametrize(
    "fixture_options",
    (
        {"finish": False},
        {"extra_schema": True},
        {"omitted_channels": ("/sim/imu/attitude",)},
        {"extra_channel": True},
        {
            "channel_type_overrides": {
                "/sim/lidar/points": "slope_sim.interfaces.v2.ImuAttitude"
            }
        },
        {"manifest_count": 0},
        {"manifest_count": 2},
        {"schema_data": b"not-the-frozen-descriptor"},
        {"manifest_overrides": {"scene_id": ""}},
        {"manifest_overrides": {"lidar_pattern_version": "other-pattern"}},
        {"manifest_overrides": {"lidar_pattern_sha256": "00" * 32}},
    ),
)
def test_mcap_structure_and_manifest_are_exact(
    tmp_path: Path,
    fixture_options: dict[str, object],
) -> None:
    """footer、唯一 schema、五 channel 与唯一完整 manifest 都是硬门禁。"""
    from slope_sim import mapping_mcap

    mcap_path, result_path, _counts = _write_fixture(
        tmp_path,
        **fixture_options,  # type: ignore[arg-type]
    )

    with pytest.raises(mapping_mcap.MappingMcapError):
        mapping_mcap.load_mapping_session(mcap_path, result_path)


@pytest.mark.parametrize(
    "mutation",
    (
        "mcap-sequence",
        "sequence-start",
        "sequence-gap",
        "log-time",
        "publish-time",
        "hundred-hz-cadence",
        "ten-hz-cadence",
        "session",
        "world",
        "unknown-field",
    ),
)
def test_message_envelope_payload_identity_and_cadence_are_exact(
    tmp_path: Path,
    mutation: str,
) -> None:
    """MCAP envelope 必须逐字段绑定 payload，且每 topic 从 sequence 0 按频率连续。"""
    from slope_sim import mapping_mcap

    frames = _valid_frames()
    if mutation == "mcap-sequence":
        _fixture_frame(frames, "/sim/lidar/points", 0).mcap_sequence = 1
    elif mutation == "sequence-start":
        frame = _fixture_frame(frames, "/sim/wheel/command", 0)
        frame.model = replace(frame.model, sequence=1)
        frame.mcap_sequence = 1
    elif mutation == "sequence-gap":
        frame = _fixture_frame(frames, "/sim/wheel/command", 1)
        frame.model = replace(frame.model, sequence=5)
        frame.mcap_sequence = 5
    elif mutation == "log-time":
        _fixture_frame(frames, "/sim/lidar/points", 0).log_time_ns = 1
    elif mutation == "publish-time":
        _fixture_frame(frames, "/sim/lidar/points", 0).publish_time_ns = 1
    elif mutation == "hundred-hz-cadence":
        frame = _fixture_frame(frames, "/sim/wheel/state", 1)
        frame.model = replace(frame.model, timestamp_ns=11_000_000)
    elif mutation == "ten-hz-cadence":
        frame = _fixture_frame(frames, "/sim/lidar/points", 1)
        frame.model = replace(frame.model, timebase_ns=110_000_000)
    elif mutation == "session":
        frame = _fixture_frame(frames, "/sim/wheel/command", 0)
        frame.model = replace(frame.model, simulation_session_id=b"x" * 16)
    elif mutation == "world":
        frame = _fixture_frame(frames, "/sim/wheel/state", 0)
        frame.model = replace(frame.model, world_generation=_WORLD_GENERATION + 1)
    else:
        _fixture_frame(frames, "/sim/wheel/command", 0).payload_suffix = b"\xf8\x07\x01"
    mcap_path, result_path, _counts = _write_fixture(tmp_path, frames=frames)

    with pytest.raises(mapping_mcap.MappingMcapError):
        mapping_mcap.load_mapping_session(mcap_path, result_path)


@pytest.mark.parametrize(
    "mutation",
    ("frame-id", "lidar-id", "off-grid", "late", "duplicate", "descending", "too-many"),
)
def test_lidar_frames_follow_the_offline_mid360_firing_domain(
    tmp_path: Path,
    mutation: str,
) -> None:
    """点云只保留 20k schedule 中真实命中，offset 必须是严格递增的 5 us slot。"""
    from slope_sim import mapping_mcap

    frames = _valid_frames()
    frame = _fixture_frame(frames, "/sim/lidar/points", 0)
    cloud = frame.model
    if mutation == "frame-id":
        frame.model = replace(cloud, frame_id="base_link")
    elif mutation == "lidar-id":
        frame.model = replace(cloud, lidar_id=2)
    else:
        offsets = {
            "off-grid": (1,),
            "late": (100_000_000,),
            "duplicate": (0, 0),
            "descending": (5_000, 0),
            "too-many": tuple(0 for _ in range(20_001)),
        }[mutation]
        points = tuple(
            LidarPointV2(offset, 1.0, 0.0, 0.0, 100, 1, 0)
            for offset in offsets
        )
        frame.model = replace(cloud, point_num=len(points), points=points)
    mcap_path, result_path, _counts = _write_fixture(tmp_path, frames=frames)

    with pytest.raises(mapping_mcap.MappingMcapError):
        mapping_mcap.load_mapping_session(mcap_path, result_path)


@pytest.mark.parametrize(
    "mutation",
    ("rtk-frame", "imu-frame", "unpaired"),
)
def test_pose_nodes_are_canonical_and_paired(
    tmp_path: Path,
    mutation: str,
) -> None:
    """RTK/IMU 必须同刻成对并使用 canonical frame。"""
    from slope_sim import mapping_mcap

    frames = _valid_frames()
    if mutation == "rtk-frame":
        frame = _fixture_frame(frames, "/sim/rtk/state", 0)
        frame.model = replace(frame.model, frame_id="map")
    elif mutation == "imu-frame":
        frame = _fixture_frame(frames, "/sim/imu/attitude", 0)
        frame.model = replace(frame.model, frame_id="imu_link")
    else:
        frames.remove(_fixture_frame(frames, "/sim/imu/attitude", 2))
    mcap_path, result_path, _counts = _write_fixture(tmp_path, frames=frames)

    with pytest.raises(mapping_mcap.MappingMcapError):
        mapping_mcap.load_mapping_session(mcap_path, result_path)


def test_last_lidar_without_lookahead_remains_available_for_raw_replay(
    tmp_path: Path,
) -> None:
    """缺最后姿态包围时仍建立严格索引，由回放层禁用该帧地图累计。"""
    from slope_sim import mapping_mcap

    frames = _valid_frames()
    frames.remove(_fixture_frame(frames, "/sim/rtk/state", 2))
    frames.remove(_fixture_frame(frames, "/sim/imu/attitude", 2))
    mcap_path, result_path, _counts = _write_fixture(tmp_path, frames=frames)

    index = mapping_mcap.load_mapping_session(mcap_path, result_path)

    assert index.lidar_frame_times_ns == (0, 100_000_000)
    assert tuple(node.timestamp_ns for node in index.pose_nodes) == (0, 100_000_000)


@pytest.mark.parametrize("noncanonical", ("relative", "dotdot"))
def test_mapping_paths_must_be_absolute_normalized_regular_files(
    tmp_path: Path,
    noncanonical: str,
) -> None:
    """路径身份必须能稳定绑定 Recorder result 与后续 inode 检查。"""
    from slope_sim import mapping_mcap

    mcap_path, result_path, _counts = _write_fixture(tmp_path)
    invalid = (
        Path(os.path.relpath(mcap_path, Path.cwd()))
        if noncanonical == "relative"
        else mcap_path.parent / ".." / mcap_path.parent.name / mcap_path.name
    )

    with pytest.raises(mapping_mcap.MappingMcapError):
        mapping_mcap.load_mapping_session(invalid, result_path)


@pytest.mark.parametrize("mutation", ("inode", "size", "mtime"))
def test_lidar_stream_reopen_rejects_a_changed_indexed_file(
    tmp_path: Path,
    mutation: str,
) -> None:
    """每次流式重开都须绑定建索引时的 inode、size 与 mtime，禁止路径替换。"""
    from slope_sim import mapping_mcap

    mcap_path, result_path, _counts = _write_fixture(tmp_path)
    index = mapping_mcap.load_mapping_session(mcap_path, result_path)
    original = mcap_path.stat()
    if mutation == "inode":
        replacement = tmp_path / "replacement.mcap"
        replacement.write_bytes(mcap_path.read_bytes())
        replacement.replace(mcap_path)
    elif mutation == "size":
        with mcap_path.open("ab") as output:
            output.write(b"x")
    else:
        os.utime(
            mcap_path,
            ns=(original.st_atime_ns, original.st_mtime_ns + 1_000_000_000),
        )

    with pytest.raises(mapping_mcap.MappingMcapError, match="changed since indexing"):
        tuple(index.iter_lidar_frames())
