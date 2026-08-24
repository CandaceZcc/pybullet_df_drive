"""手动采集离线重建的 MCAP 发布契约。"""
from __future__ import annotations

import json
from pathlib import Path

import pytest


def _trajectory_timestamp_ns(step: int) -> int:
    return round(step * 1_000_000_000 / 240)


def _trajectory_receipt(tmp_path: Path, rows: list[dict[str, object]]) -> object:
    from slope_sim.manual_capture import ManualCaptureReceipt, ManualCaptureStatus

    trajectory_path = (tmp_path / "trajectory.jsonl").absolute()
    trajectory_path.write_text(
        "".join(f"{json.dumps(row)}\n" for row in rows),
        encoding="utf-8",
    )
    return ManualCaptureReceipt(
        ManualCaptureStatus.FINALIZED,
        tmp_path.absolute(),
        (tmp_path / "scene.yaml").absolute(),
        trajectory_path,
        (tmp_path / "capture.json").absolute(),
        60,
        1,
        0,
        _trajectory_timestamp_ns(len(rows)),
        len(rows),
    )


def _trajectory_row(step: int) -> dict[str, object]:
    return {
        "sim_time_ns": _trajectory_timestamp_ns(step),
        "position": [float(step), 0.0, 0.25],
        "orientation": [0.0, 0.0, 0.0, 1.0],
    }


def test_trajectory_rows_streams_one_frame_without_read_text_or_read_ahead(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """首个 24 样本帧必须可立即产出，不能全量读取后续轨迹。"""
    from slope_sim.manual_mid360_reconstruction import _trajectory_rows

    receipt = _trajectory_receipt(
        tmp_path,
        [_trajectory_row(step) for step in range(1, 25)],
    )
    with receipt.trajectory_path.open("a", encoding="utf-8") as stream:
        stream.write("not-json\n")

    def reject_read_text(_path: Path, *args: object, **kwargs: object) -> str:
        raise AssertionError("trajectory must be streamed with Path.open")

    monkeypatch.setattr(Path, "read_text", reject_read_text)
    frames = iter(_trajectory_rows(receipt))

    first_frame = next(frames)

    assert len(first_frame) == 24
    assert first_frame[0][0] == _trajectory_timestamp_ns(1)
    assert first_frame[-1][0] == _trajectory_timestamp_ns(24)


@pytest.mark.parametrize(
    "bad_timestamp",
    [
        pytest.param(_trajectory_timestamp_ns(8), id="duplicate"),
        pytest.param(_trajectory_timestamp_ns(7), id="descending"),
        pytest.param(_trajectory_timestamp_ns(10), id="missing-cadence"),
    ],
)
def test_trajectory_rows_rejects_disorder_and_missing_240hz_sample(
    tmp_path: Path,
    bad_timestamp: int,
) -> None:
    """轨迹必须严格递增，且不能漏掉 240 Hz 物理步。"""
    from slope_sim.manual_mid360_reconstruction import _trajectory_rows

    rows = [_trajectory_row(step) for step in range(1, 25)]
    rows[8]["sim_time_ns"] = bad_timestamp
    receipt = _trajectory_receipt(tmp_path, rows)

    with pytest.raises(RuntimeError, match="240 Hz cadence"):
        tuple(_trajectory_rows(receipt))


def test_trajectory_rows_rejects_non_finite_pose(tmp_path: Path) -> None:
    """NaN/Inf 位姿不得进入 PyBullet 回放。"""
    from slope_sim.manual_mid360_reconstruction import _trajectory_rows

    rows = [_trajectory_row(step) for step in range(1, 25)]
    rows[10]["position"] = [float("nan"), 0.0, 0.25]
    receipt = _trajectory_receipt(tmp_path, rows)

    with pytest.raises(RuntimeError, match="invalid pose"):
        tuple(_trajectory_rows(receipt))


def _cloud(identity: object, *, sequence: int, timebase_ns: int) -> object:
    from slope_sim.interfaces.v2.models import LidarPointCloudV2, LidarPointV2

    return LidarPointCloudV2(
        timebase_ns=timebase_ns,
        frame_id="lidar_link",
        point_num=1,
        lidar_id=1,
        points=(LidarPointV2(0, 1.0, 0.0, 0.0, 100, 1, 0),),
        sequence=sequence,
        world_generation=identity.world_generation,
        simulation_session_id=identity.simulation_session_id,
        descriptor_sha256=identity.descriptor_sha256,
    )


def test_write_manual_mcap_registers_v2_contract_and_lidar_frame(tmp_path: Path) -> None:
    """重建器写出的最小帧必须能被既有严格 MCAP 索引读取。"""
    from slope_sim.manual_mid360_reconstruction import (
        ManualMid360Frame,
        ManualMcapIdentity,
        write_manual_mid360_mcap,
    )
    from slope_sim.mapping_mcap import load_mapping_session
    from slope_sim.sensor_backend import Pose

    identity = ManualMcapIdentity.create(world_generation=1, scene_id="manual-capture")
    cloud = _cloud(identity, sequence=0, timebase_ns=100_000_000)
    frame = ManualMid360Frame(
        cloud,
        Pose((1.0, 2.0, 0.25), (0.0, 0.0, 0.0, 1.0)),
    )

    mcap_path, result_path = write_manual_mid360_mcap(
        output_dir=(tmp_path / "output").absolute(),
        identity=identity,
        frames=(frame,),
    )

    session = load_mapping_session(mcap_path, result_path)
    assert session.identity.simulation_session_id == identity.simulation_session_id
    assert session.lidar_frame_times_ns == (100_000_000,)
    assert tuple(session.iter_lidar_frames()) == (cloud,)
    assert session.pose_nodes[0].base_pose.position == pytest.approx((1.0, 2.0, 0.25))


def test_write_manual_mcap_streams_one_pass_and_preserves_distinct_poses(
    tmp_path: Path,
) -> None:
    """高密度帧必须逐个消费，且回放索引能恢复每帧不同的车体位姿。"""
    from slope_sim.manual_mid360_reconstruction import (
        ManualMid360Frame,
        ManualMcapIdentity,
        write_manual_mid360_mcap,
    )
    from slope_sim.mapping_mcap import load_mapping_session
    from slope_sim.sensor_backend import Pose
    import pybullet as p

    identity = ManualMcapIdentity.create(world_generation=3, scene_id="two-poses")
    tilted_orientation = tuple(
        float(value) for value in p.getQuaternionFromEuler((0.2, -0.15, 1.0))
    )
    records = (
        ManualMid360Frame(
            _cloud(identity, sequence=0, timebase_ns=100_000_000),
            Pose((1.0, 2.0, 0.25), (0.0, 0.0, 0.0, 1.0)),
        ),
        ManualMid360Frame(
            _cloud(identity, sequence=1, timebase_ns=200_000_000),
            Pose((-2.0, 4.0, 0.50), tilted_orientation),
        ),
    )

    class OneShotFrames:
        def __init__(self) -> None:
            self.iterations = 0

        def __iter__(self):
            self.iterations += 1
            if self.iterations != 1:
                raise AssertionError("manual frames were buffered or iterated more than once")
            yield from records

    one_shot = OneShotFrames()
    mcap_path, result_path = write_manual_mid360_mcap(
        output_dir=(tmp_path / "streamed").absolute(),
        identity=identity,
        frames=one_shot,
    )

    index = load_mapping_session(mcap_path, result_path)
    assert one_shot.iterations == 1
    assert index.pose_nodes[0].base_pose.position == pytest.approx((1.0, 2.0, 0.25))
    assert index.pose_nodes[1].base_pose.position == pytest.approx((-2.0, 4.0, 0.50))
    assert index.pose_nodes[0].base_pose.orientation == pytest.approx(records[0].base_pose.orientation)
    assert index.pose_nodes[1].base_pose.orientation == pytest.approx(records[1].base_pose.orientation)


def test_export_validation_rejects_non_lvx2_bytes(tmp_path: Path) -> None:
    """导出器即使返回成功，也不能把仅非空的损坏文件交给 Viewer。"""
    from slope_sim.manual_mid360_reconstruction import _validate_lvx2_path

    invalid = (tmp_path / "lidar.lvx2").absolute()
    invalid.write_bytes(b"LVX2")

    with pytest.raises(RuntimeError, match="structurally invalid"):
        _validate_lvx2_path(invalid)


@pytest.mark.stage4_artifact
def test_reconstruct_manual_capture_generates_a_valid_lvx2(tmp_path: Path) -> None:
    """24 个驾驶期姿态必须在离线期生成一帧可读 LVX2。"""
    from slope_sim.config import ExperimentConfig
    from slope_sim.manual_capture import ManualCaptureRecorder
    from slope_sim.manual_mid360_reconstruction import reconstruct_manual_capture
    from slope_sim.simulation import initial_scene_document
    from scripts.verify_lvx2 import parse_lvx2

    config = ExperimentConfig(mode="gui", dashboard_enabled=False, interface_enabled=False)
    session = ManualCaptureRecorder(tmp_path.absolute()).start(
        scene_document=initial_scene_document(config),
        world_generation=1,
        duration_limit_sec=60,
        started_sim_time_ns=0,
    )
    for step in range(24):
        session.record_pose(
            sim_time_ns=_trajectory_timestamp_ns(step + 1),
            position=(0.0, 0.0, 0.25),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )

    result = reconstruct_manual_capture(
        receipt=session.finish(finished_sim_time_ns=100_000_000), config=config
    )

    inspection = parse_lvx2(result.lvx2_path)
    assert result.lidar_frame_count == 1
    assert inspection.frame_count >= 1
    assert inspection.point_count > 0
