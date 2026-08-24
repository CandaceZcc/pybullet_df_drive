"""阶段五离线 MCAP 压缩合同：重写后必须保留会话内容。"""
from __future__ import annotations

from importlib import import_module
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer


def _write_uncompressed_session(path: Path) -> None:
    with path.open("wb") as stream:
        writer = Writer(stream, compression=CompressionType.NONE, use_chunking=False)
        writer.start(profile="protobuf", library="stage5-test")
        schema_id = writer.register_schema("slope-sim.test", "protobuf", b"descriptor")
        channel_id = writer.register_channel(
            "/sim/test", "protobuf", schema_id, {"type": "slope_sim.test.Frame"}
        )
        writer.add_metadata("slope_sim.session_manifest", {"scene_id": "unit-test"})
        writer.add_message(channel_id, 100, b"x" * 4096, 100, sequence=1)
        writer.add_message(channel_id, 200, b"x" * 4096, 200, sequence=2)
        writer.finish()


def _session_records(path: Path) -> tuple[tuple[object, ...], ...]:
    with path.open("rb") as stream:
        reader = make_reader(stream)
        return tuple(
            (channel.topic, channel.message_encoding, schema.name if schema else None,
             message.log_time, message.publish_time, message.sequence, message.data)
            for schema, channel, message in reader.iter_messages()
        )


def test_offline_mcap_compression_preserves_finalized_session_content(tmp_path: Path) -> None:
    """压缩仅生成新副本，所有可重放的原始消息保持逐字节一致。"""
    module = import_module("slope_sim.mcap_compression")
    source = tmp_path / "session.mcap"
    _write_uncompressed_session(source)
    source_bytes = source.read_bytes()

    result = module.compress_mcap_zstd(source)

    assert result.source_path == source
    assert result.output_path == tmp_path / "session.zstd.mcap"
    assert result.source_bytes == len(source_bytes)
    assert result.output_bytes == result.output_path.stat().st_size
    assert result.output_bytes < result.source_bytes
    assert source.read_bytes() == source_bytes
    assert _session_records(result.output_path) == _session_records(source)
