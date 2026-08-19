# 场景原子写入集成测试：覆盖成功替换、各 I/O 失败点、并发与临时文件清理。
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import os

import pytest

from slope_sim.lidar_pointcloud import LidarConfig
from slope_sim.obstacles import ObstacleGeometry, ObstacleSpec
from slope_sim.scene_config import (
    SCENE_SCHEMA_VERSION,
    SceneDocument,
    SensorDocument,
    TerrainDocument,
    dump_scene_atomic,
    load_scene,
)
from slope_sim.truth_sensors import SensorMounts


IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)


def sample_scene_document() -> SceneDocument:
    return SceneDocument(
        schema_version=SCENE_SCHEMA_VERSION,
        robot_model="df_mid",
        terrain=TerrainDocument("flat", 0.0, 0, "medium"),
        obstacles=(
            ObstacleSpec(
                logical_id=1,
                mode="static",
                geometry=ObstacleGeometry("box", (0.2, 0.2, 0.3)),
                position=(0.5, 0.0, 0.3),
                orientation=IDENTITY_QUATERNION,
            ),
        ),
        sensors=SensorDocument(SensorMounts.default(), LidarConfig.default()),
    )


def _temp_files(directory, target_name="scene.yaml"):
    return list(directory.glob(f".{target_name}.*.tmp"))


class _FailingStream:
    """代理真实文本流，只在指定阶段注入一次 I/O 失败。"""

    def __init__(self, wrapped, stage: str) -> None:
        self._wrapped = wrapped
        self._stage = stage
        self._failed = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        try:
            self._wrapped.close()
        except OSError:
            if exc_type is None:
                raise
        return False

    def write(self, text):
        if self._stage == "write" and not self._failed:
            self._failed = True
            raise OSError("write failure")
        return self._wrapped.write(text)

    def flush(self):
        if self._stage == "flush" and not self._failed:
            self._failed = True
            raise OSError("flush failure")
        return self._wrapped.flush()

    def fileno(self):
        return self._wrapped.fileno()


def test_atomic_export_creates_parent_and_replaces_complete_utf8_lf_file(tmp_path):
    target = tmp_path / "nested" / "scene.yaml"

    result = dump_scene_atomic(sample_scene_document(), target)

    assert result == target
    assert load_scene(target) == sample_scene_document()
    assert b"\r\n" not in target.read_bytes()
    assert _temp_files(target.parent) == []


def test_atomic_export_preserves_previous_file_when_replace_fails(tmp_path, monkeypatch):
    import slope_sim.scene_config as scene_config

    target = tmp_path / "scene.yaml"
    target.write_bytes(b"previous\n")

    def fail_replace(*_args):
        raise OSError("replace failure")

    monkeypatch.setattr(scene_config.os, "replace", fail_replace)

    with pytest.raises(OSError, match="replace failure"):
        dump_scene_atomic(sample_scene_document(), target)

    assert target.read_bytes() == b"previous\n"
    assert _temp_files(tmp_path) == []


def test_atomic_export_cleans_temp_when_fdopen_fails(tmp_path, monkeypatch):
    import slope_sim.scene_config as scene_config

    target = tmp_path / "scene.yaml"
    target.write_bytes(b"previous\n")

    def fail_fdopen(*_args, **_kwargs):
        raise OSError("open failure")

    monkeypatch.setattr(scene_config.os, "fdopen", fail_fdopen)

    with pytest.raises(OSError, match="open failure"):
        dump_scene_atomic(sample_scene_document(), target)

    assert target.read_bytes() == b"previous\n"
    assert _temp_files(tmp_path) == []


@pytest.mark.parametrize("stage", ("write", "flush"))
def test_atomic_export_cleans_temp_on_stream_failures(tmp_path, monkeypatch, stage):
    import slope_sim.scene_config as scene_config

    target = tmp_path / "scene.yaml"
    target.write_bytes(b"previous\n")
    real_fdopen = os.fdopen

    def failing_fdopen(fd, *args, **kwargs):
        return _FailingStream(real_fdopen(fd, *args, **kwargs), stage)

    monkeypatch.setattr(scene_config.os, "fdopen", failing_fdopen)

    with pytest.raises(OSError, match=rf"{stage} failure"):
        dump_scene_atomic(sample_scene_document(), target)

    assert target.read_bytes() == b"previous\n"
    assert _temp_files(tmp_path) == []


def test_atomic_export_cleans_temp_when_fsync_fails(tmp_path, monkeypatch):
    import slope_sim.scene_config as scene_config

    target = tmp_path / "scene.yaml"
    target.write_bytes(b"previous\n")

    def fail_fsync(_fd):
        raise OSError("fsync failure")

    monkeypatch.setattr(scene_config.os, "fsync", fail_fsync)

    with pytest.raises(OSError, match="fsync failure"):
        dump_scene_atomic(sample_scene_document(), target)

    assert target.read_bytes() == b"previous\n"
    assert _temp_files(tmp_path) == []


def test_atomic_export_preserves_target_when_temp_creation_fails(tmp_path, monkeypatch):
    import slope_sim.scene_config as scene_config

    target = tmp_path / "scene.yaml"
    target.write_bytes(b"previous\n")

    def fail_mkstemp(*_args, **_kwargs):
        raise OSError("mkstemp failure")

    monkeypatch.setattr(scene_config.tempfile, "mkstemp", fail_mkstemp)

    with pytest.raises(OSError, match="mkstemp failure"):
        dump_scene_atomic(sample_scene_document(), target)

    assert target.read_bytes() == b"previous\n"
    assert _temp_files(tmp_path) == []


def test_invalid_document_is_rejected_before_parent_directory_is_created(tmp_path):
    parent = tmp_path / "must-not-exist"

    with pytest.raises(ValueError, match="document"):
        dump_scene_atomic(object(), parent / "scene.yaml")

    assert not parent.exists()


@pytest.mark.parametrize("path", (None, b"scene.yaml", True, 3, ""))
def test_atomic_export_rejects_invalid_paths(path):
    with pytest.raises(ValueError, match="path"):
        dump_scene_atomic(sample_scene_document(), path)


def test_repeated_exports_leave_no_temporary_files(tmp_path):
    target = tmp_path / "scene.yaml"
    document = sample_scene_document()

    for _ in range(20):
        dump_scene_atomic(document, target)

    assert load_scene(target) == document
    assert _temp_files(tmp_path) == []


def test_concurrent_exports_are_complete_and_leave_no_temporary_files(tmp_path):
    target = tmp_path / "scene.yaml"
    document = sample_scene_document()

    with ThreadPoolExecutor(max_workers=8) as executor:
        results = tuple(executor.map(lambda _index: dump_scene_atomic(document, target), range(32)))

    assert results == (target,) * 32
    assert load_scene(target) == document
    assert _temp_files(tmp_path) == []
