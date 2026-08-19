# 手动 MID-360 采集核心测试：驾驶期仅冻结场景并记录轨迹，不做高密度扫描。
from __future__ import annotations

import importlib.util
import json

from slope_sim.scene_config import SceneDocument, SensorDocument, TerrainDocument


def _api():
    spec = importlib.util.find_spec("slope_sim.manual_capture")
    assert spec is not None, "manual capture domain module must exist"
    module = __import__("slope_sim.manual_capture", fromlist=["ManualCaptureRecorder"])
    return module


def _scene() -> SceneDocument:
    return SceneDocument(
        schema_version=1,
        robot_model="df_back",
        terrain=TerrainDocument("flat", 0.0, 1, "medium"),
        obstacles=(),
        sensors=SensorDocument.default(),
    )


def test_recorder_freezes_scene_and_writes_only_trajectory_samples(tmp_path):
    api = _api()
    recorder = api.ManualCaptureRecorder(tmp_path / "manual-mid360")

    session = recorder.start(
        scene_document=_scene(),
        world_generation=7,
        duration_limit_sec=60,
        started_sim_time_ns=1_000,
    )
    session.record_pose(
        sim_time_ns=1_000,
        position=(1.0, 2.0, 0.3),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    session.record_pose(
        sim_time_ns=1_000 + 4_166_667,
        position=(1.1, 2.0, 0.3),
        orientation=(0.0, 0.0, 0.1, 0.995),
    )
    receipt = session.finish(finished_sim_time_ns=1_000 + 8_333_334)

    assert receipt.status is api.ManualCaptureStatus.FINALIZED
    assert receipt.duration_limit_sec == 60
    assert receipt.output_dir.parent == tmp_path / "manual-mid360"
    assert receipt.scene_path.is_file()
    assert receipt.trajectory_path.is_file()
    assert not any(path.suffix in {".mcap", ".lvx2"} for path in receipt.output_dir.rglob("*"))

    rows = [json.loads(line) for line in receipt.trajectory_path.read_text(encoding="utf-8").splitlines()]
    assert rows == [
        {"sim_time_ns": 1_000, "position": [1.0, 2.0, 0.3], "orientation": [0.0, 0.0, 0.0, 1.0]},
        {"sim_time_ns": 4_167_667, "position": [1.1, 2.0, 0.3], "orientation": [0.0, 0.0, 0.1, 0.995]},
    ]


def test_supported_duration_options_include_unlimited_and_reject_other_values(tmp_path):
    api = _api()
    recorder = api.ManualCaptureRecorder(tmp_path)

    assert recorder.duration_options_sec == (60, 90, 180, None)
    for duration in (60, 90, 180, None):
        session = recorder.start(
            scene_document=_scene(),
            world_generation=1,
            duration_limit_sec=duration,
            started_sim_time_ns=0,
        )
        session.abort(reason="test")

    try:
        recorder.start(
            scene_document=_scene(),
            world_generation=1,
            duration_limit_sec=120,
            started_sim_time_ns=0,
        )
    except ValueError as error:
        assert "duration_limit_sec" in str(error)
    else:
        raise AssertionError("unsupported duration must be rejected")


def test_finish_and_abort_are_terminal_and_output_directories_never_collide(tmp_path):
    api = _api()
    recorder = api.ManualCaptureRecorder(tmp_path)
    first = recorder.start(
        scene_document=_scene(),
        world_generation=1,
        duration_limit_sec=None,
        started_sim_time_ns=0,
    )
    second = recorder.start(
        scene_document=_scene(),
        world_generation=1,
        duration_limit_sec=None,
        started_sim_time_ns=0,
    )

    assert first.output_dir != second.output_dir
    aborted = first.abort(reason="runSim closed")
    assert aborted.status is api.ManualCaptureStatus.ABORTED
    assert "runSim closed" in aborted.receipt_path.read_text(encoding="utf-8")

    second.finish(finished_sim_time_ns=1)
    try:
        second.record_pose(
            sim_time_ns=2,
            position=(0.0, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )
    except RuntimeError as error:
        assert "finalized" in str(error)
    else:
        raise AssertionError("finalized session must reject more samples")
