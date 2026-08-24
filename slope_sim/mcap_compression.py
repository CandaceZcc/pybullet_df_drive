"""阶段五会话结束后的 MCAP Zstandard 压缩，不参与实时 Recorder。"""
from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import tempfile

from mcap.reader import make_reader
from mcap.writer import CompressionType, Writer


@dataclass(frozen=True, slots=True)
class McapCompressionResult:
    """离线压缩副本的稳定结果，原始会话始终保留。"""

    source_path: Path
    output_path: Path
    source_bytes: int
    output_bytes: int


def _require_completed_source(path: Path) -> Path:
    if not isinstance(path, Path) or not path.is_absolute() or path != path.resolve():
        raise ValueError("source path must be an absolute normalized Path")
    if path.is_symlink() or not path.is_file() or path.suffix != ".mcap":
        raise ValueError("source path must be a regular .mcap file")
    return path


def _default_output_path(source: Path) -> Path:
    return source.with_name(f"{source.stem}.zstd{source.suffix}")


def compress_mcap_zstd(source_path: Path, *, output_path: Path | None = None) -> McapCompressionResult:
    """以 Zstandard 原子生成已完成 MCAP 的新副本，绝不覆盖原始会话。"""
    source = _require_completed_source(source_path)
    output = _default_output_path(source) if output_path is None else output_path
    if not isinstance(output, Path) or not output.is_absolute() or output != output.resolve():
        raise ValueError("output path must be an absolute normalized Path")
    if output == source or output.suffix != ".mcap" or output.exists():
        raise ValueError("output path must be a new .mcap file distinct from source")

    source_bytes = source.stat().st_size
    temporary_path: Path | None = None
    try:
        with source.open("rb") as source_stream:
            reader = make_reader(source_stream)
            header = reader.get_header()
            summary = reader.get_summary()
            if summary is None:
                raise ValueError("source MCAP is not finalized")
            schemas = summary.schemas
            channels = summary.channels
            metadata = tuple(reader.iter_metadata())

        with tempfile.NamedTemporaryFile(
            mode="wb",
            prefix=f".{output.name}.",
            suffix=".tmp",
            dir=output.parent,
            delete=False,
        ) as temporary_stream:
            temporary_path = Path(temporary_stream.name)
            writer = Writer(temporary_stream, compression=CompressionType.ZSTD)
            writer.start(profile=header.profile, library=header.library)
            schema_ids = {
                source_id: writer.register_schema(schema.name, schema.encoding, schema.data)
                for source_id, schema in schemas.items()
            }
            channel_ids = {
                source_id: writer.register_channel(
                    channel.topic,
                    channel.message_encoding,
                    schema_ids.get(channel.schema_id, 0),
                    dict(channel.metadata),
                )
                for source_id, channel in channels.items()
            }
            for entry in metadata:
                writer.add_metadata(entry.name, dict(entry.metadata))
            with source.open("rb") as source_stream:
                for _schema, channel, message in make_reader(source_stream).iter_messages():
                    writer.add_message(
                        channel_ids[channel.id],
                        message.log_time,
                        message.data,
                        message.publish_time,
                        sequence=message.sequence,
                    )
            writer.finish()
            temporary_stream.flush()
            os.fsync(temporary_stream.fileno())
        os.replace(temporary_path, output)
        temporary_path = None
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass

    return McapCompressionResult(source, output, source_bytes, output.stat().st_size)
