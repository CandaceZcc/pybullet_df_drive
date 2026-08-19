"""手动驾驶采集的离线 MID-360 MCAP 发布基础。"""
from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from dataclasses import replace
import json
import math
import os
from pathlib import Path
import subprocess
from uuid import uuid4

from mcap.writer import CompressionType, Writer
import pybullet as p

from slope_sim.config import ExperimentConfig
from slope_sim.coordinator import build_world_from_scene_document
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
from slope_sim.interfaces.v2.topics import V2_TOPICS
from slope_sim.manual_capture import ManualCaptureReceipt, ManualCaptureStatus
from slope_sim.mid360_offline import (
    OfflineMid360FrameScanner,
    OfflineMid360Profile,
    OfflineMid360Schedule,
)
from slope_sim.scene_config import load_scene
from slope_sim.sensor_backend import Pose, PyBulletSensorBackend
from slope_sim.truth_sensors import heading_from_rtk_baseline
from scripts.verify_lvx2 import parse_lvx2


_PATTERN_VERSION = "livox-mid360-800000-v1"
_PATTERN_SHA256 = "4077e0b68a68e40ba8a5da17d4aff5ba86ea4fb557a4f8b594e4de1ebbeb20ca"


@dataclass(frozen=True, slots=True)
class ManualMcapIdentity:
    """冻结进手动采集 MCAP 的唯一会话身份。"""

    simulation_session_id: bytes
    descriptor_sha256: bytes
    world_generation: int
    scene_id: str

    @classmethod
    def create(cls, *, world_generation: int, scene_id: str) -> "ManualMcapIdentity":
        if type(world_generation) is not int or world_generation <= 0:
            raise ValueError("world_generation must be a positive integer")
        if not isinstance(scene_id, str) or not scene_id:
            raise ValueError("scene_id must be nonempty")
        descriptor = load_v2_descriptor()
        return cls(uuid4().bytes, descriptor.sha256, world_generation, scene_id)


@dataclass(frozen=True, slots=True)
class ManualReconstructionResult:
    """离线重建及 C++ 导出的已验证产物。"""

    mcap_path: Path
    recorder_result_path: Path
    lvx2_path: Path
    export_result_path: Path
    lidar_frame_count: int


@dataclass(frozen=True, slots=True)
class ManualMid360Frame:
    """一帧高保真点云及其同刻录制车体位姿。"""

    cloud: LidarPointCloudV2
    base_pose: Pose

    def __post_init__(self) -> None:
        if type(self.cloud) is not LidarPointCloudV2:
            raise ValueError("cloud must be a LidarPointCloudV2")
        if type(self.base_pose) is not Pose:
            raise ValueError("base_pose must be a Pose")


def _next_240hz_timestamp_ns(timestamp_ns: int) -> int:
    """从一个精确物理步时间戳推导下一个 240 Hz 时间戳。"""
    step = (timestamp_ns * 240 + 500_000_000) // 1_000_000_000
    canonical = (step * 1_000_000_000 + 120) // 240
    if canonical != timestamp_ns:
        raise RuntimeError("manual capture trajectory violates exact 240 Hz cadence")
    return ((step + 1) * 1_000_000_000 + 120) // 240


def _trajectory_rows(
    receipt: ManualCaptureReceipt,
) -> Iterable[tuple[tuple[int, tuple[float, ...], tuple[float, ...]], ...]]:
    """逐行校验轨迹并按 24 个物理步产出一帧，内存保持常量级。"""
    frame: list[tuple[int, tuple[float, ...], tuple[float, ...]]] = []
    previous_timestamp: int | None = None
    emitted_frames = 0
    with receipt.trajectory_path.open("r", encoding="utf-8") as stream:
        for line in stream:
            try:
                raw = json.loads(line)
                timestamp = raw["sim_time_ns"]
                position = raw["position"]
                orientation = raw["orientation"]
            except (json.JSONDecodeError, KeyError, TypeError) as error:
                raise RuntimeError("manual capture trajectory contains an invalid pose") from error
            values = (*position, *orientation) if isinstance(position, list) and isinstance(orientation, list) else ()
            if (
                type(timestamp) is not int
                or timestamp < 0
                or not isinstance(position, list)
                or not isinstance(orientation, list)
                or len(position) != 3
                or len(orientation) != 4
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    for value in values
                )
            ):
                raise RuntimeError("manual capture trajectory contains an invalid pose")
            if previous_timestamp is not None and timestamp != _next_240hz_timestamp_ns(previous_timestamp):
                raise RuntimeError("manual capture trajectory violates exact 240 Hz cadence")
            previous_timestamp = timestamp
            frame.append(
                (
                    timestamp,
                    tuple(float(value) for value in position),
                    tuple(float(value) for value in orientation),
                )
            )
            if len(frame) == 24:
                emitted_frames += 1
                yield tuple(frame)
                frame.clear()
    if emitted_frames == 0:
        raise RuntimeError("manual capture is shorter than one MID-360 frame")


def _manual_exporter_path() -> Path:
    root = Path(__file__).resolve().parents[1]
    candidates = tuple(root.glob("build/**/slope_sim_stage4_export"))
    for candidate in sorted(candidates, key=lambda item: item.stat().st_mtime_ns, reverse=True):
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    raise RuntimeError("slope_sim_stage4_export is unavailable; build the Stage4 exporter first")


def _validate_lvx2_path(path: Path) -> Path:
    """用独立 oracle 完整解析导出物，损坏文件不得进入 Viewer。"""
    try:
        inspection = parse_lvx2(path)
    except (OSError, TypeError, ValueError) as error:
        raise RuntimeError("MID-360 export produced a structurally invalid LVX2") from error
    if (
        not inspection.complete
        or inspection.frame_count <= 0
        or inspection.package_count <= 0
        or inspection.point_count <= 0
    ):
        raise RuntimeError("MID-360 export produced a structurally invalid LVX2")
    return path


def reconstruct_manual_capture(
    *,
    receipt: ManualCaptureReceipt,
    config: ExperimentConfig,
) -> ManualReconstructionResult:
    """在独立 DIRECT 世界回放已冻结轨迹，结束后才执行高保真射线。"""
    if type(receipt) is not ManualCaptureReceipt or receipt.status is not ManualCaptureStatus.FINALIZED:
        raise ValueError("receipt must be a finalized ManualCaptureReceipt")
    if type(config) is not ExperimentConfig:
        raise ValueError("config must be an ExperimentConfig")
    sample_frames = _trajectory_rows(receipt)
    document = load_scene(receipt.scene_path)
    direct_config = replace(
        config,
        mode="direct",
        dashboard_enabled=False,
        interface_enabled=False,
        interface_log_enabled=False,
    )
    client_id = p.connect(p.DIRECT)
    try:
        world, obstacle_manager = build_world_from_scene_document(client_id, direct_config, document)
        robot_id = world.active_robot.robot.robot_id
        backend = PyBulletSensorBackend(client_id, robot_id)
        snapshots = obstacle_manager.snapshot(include_body_id=True)
        backend.bind_scene(world.scene.body_ids, snapshots)
        body_positions = {
            body_id: tuple(float(value) for value in p.getBasePositionAndOrientation(body_id, physicsClientId=client_id)[0])
            for body_id in world.scene.body_ids
        }
        body_positions.update({
            snapshot.body_id: tuple(float(value) for value in snapshot.position)
            for snapshot in snapshots if snapshot.body_id is not None
        })
        identity = ManualMcapIdentity.create(
            world_generation=receipt.world_generation,
            scene_id=receipt.output_dir.name,
        )
        schedule = OfflineMid360Schedule(
            OfflineMid360Profile.high_fidelity(),
            _PATTERN_VERSION,
            receipt.world_generation,
        )
        frame_count = 0

        def reconstructed_frames() -> Iterable[ManualMid360Frame]:
            """每完成一帧立即交给 MCAP writer，内存不随录制时长增长。"""
            nonlocal frame_count
            for sequence, frame_samples in enumerate(sample_frames):
                scanner = OfflineMid360FrameScanner(backend, schedule, sequence=sequence)
                base_pose: Pose | None = None
                for step, (_timestamp, position, orientation) in enumerate(frame_samples):
                    p.resetBasePositionAndOrientation(
                        robot_id,
                        position,
                        orientation,
                        physicsClientId=client_id,
                    )
                    if step == 0:
                        base_pose = backend.world_pose("base_link")
                    scanner.capture_step(step, body_positions_by_id=body_positions)
                if base_pose is None:
                    raise RuntimeError("MID-360 frame has no recorded base pose")
                raw = scanner.finalize(timebase_ns=frame_samples[0][0])
                frame_count += 1
                yield ManualMid360Frame(
                    LidarPointCloudV2(
                        raw.timebase_ns,
                        raw.frame_id,
                        raw.point_num,
                        raw.lidar_id,
                        tuple(
                            LidarPointV2(
                                point.offset_time_ns,
                                point.x,
                                point.y,
                                point.z,
                                point.reflectivity,
                                point.tag,
                                point.line,
                            )
                            for point in raw.points
                        ),
                        sequence,
                        identity.world_generation,
                        identity.simulation_session_id,
                        identity.descriptor_sha256,
                    ),
                    base_pose,
                )

        mcap_path, recorder_result_path = write_manual_mid360_mcap(
            output_dir=receipt.output_dir,
            identity=identity,
            frames=reconstructed_frames(),
        )
        export_dir = receipt.output_dir / "export"
        export_result = receipt.output_dir / "export.json"
        command = [
            str(_manual_exporter_path()), "--input", str(mcap_path), "--descriptor-set",
            str(Path(__file__).resolve().parent / "interfaces/generated/slope_sim_interfaces_v2.desc"),
            "--output-dir", str(export_dir), "--result", str(export_result),
        ]
        completed = subprocess.run(command, text=True, capture_output=True, check=False)
        if completed.returncode != 0:
            raise RuntimeError(completed.stderr.strip() or completed.stdout.strip() or "MID-360 export failed")
        lvx2_path = _validate_lvx2_path(export_dir / "lidar.lvx2")
        return ManualReconstructionResult(
            mcap_path,
            recorder_result_path,
            lvx2_path,
            export_result,
            frame_count,
        )
    finally:
        p.disconnect(client_id)


def reconstruction_worker_entrypoint(
    receipt: ManualCaptureReceipt,
    config: ExperimentConfig,
    result_sender: object,
) -> None:
    """子进程入口只回传小型路径结果，避免把点云复制回 GUI 进程。"""
    try:
        result = reconstruct_manual_capture(receipt=receipt, config=config)
        try:
            result_sender.send(
                {
                    "ok": True,
                    "mcap_path": str(result.mcap_path),
                    "lvx2_path": str(result.lvx2_path),
                    "frames": result.lidar_frame_count,
                }
            )
        except (BrokenPipeError, OSError):
            pass
    except Exception as error:
        try:
            result_sender.send({"ok": False, "error": str(error) or type(error).__name__})
        except (BrokenPipeError, OSError):
            pass
    finally:
        try:
            result_sender.close()
        except (BrokenPipeError, OSError):
            pass


def _world_point(base_pose: Pose, local_point: tuple[float, float, float]) -> tuple[float, float, float]:
    """把 canonical RTK 局部安装点投影到冻结的车体世界位姿。"""
    x, y, z, w = base_pose.orientation
    px, py, pz = local_point
    rotated = (
        (1.0 - 2.0 * (y * y + z * z)) * px
        + 2.0 * (x * y - w * z) * py
        + 2.0 * (x * z + w * y) * pz,
        2.0 * (x * y + w * z) * px
        + (1.0 - 2.0 * (x * x + z * z)) * py
        + 2.0 * (y * z - w * x) * pz,
        2.0 * (x * z - w * y) * px
        + 2.0 * (y * z + w * x) * py
        + (1.0 - 2.0 * (x * x + y * y)) * pz,
    )
    return tuple(
        base_pose.position[index] + rotated[index] for index in range(3)
    )  # type: ignore[return-value]


def _pose_models(
    frame: ManualMid360Frame,
    identity: ManualMcapIdentity,
) -> tuple[RtkStateV2, ImuAttitudeV2]:
    """从同刻 base pose 生成可被 mapping_mcap 无损恢复的 RTK/IMU。"""
    left = _world_point(frame.base_pose, (0.0, 0.20, 0.18))
    right = _world_point(frame.base_pose, (0.0, -0.20, 0.18))
    center = tuple(
        (left[index] + right[index]) / 2.0 for index in range(3)
    )
    roll, pitch, _yaw = p.getEulerFromQuaternion(frame.base_pose.orientation)
    timestamp_ns = frame.cloud.timebase_ns
    sequence = frame.cloud.sequence
    return (
        RtkStateV2(
            timestamp_ns,
            sequence,
            identity.world_generation,
            "world",
            Point3dV2(*left),
            Point3dV2(*center),
            Point3dV2(*right),
            heading_from_rtk_baseline(left, right),
            identity.simulation_session_id,
            identity.descriptor_sha256,
        ),
        ImuAttitudeV2(
            timestamp_ns,
            float(roll),
            float(pitch),
            sequence,
            identity.world_generation,
            "base_link",
            identity.simulation_session_id,
            identity.descriptor_sha256,
        ),
    )


def write_manual_mid360_mcap(
    *,
    output_dir: Path,
    identity: ManualMcapIdentity,
    frames: Iterable[ManualMid360Frame],
) -> tuple[Path, Path]:
    """单次遍历并发布高保真帧，保持 Stage4 v2 的严格 MCAP 外形。"""
    if not isinstance(output_dir, Path) or not output_dir.is_absolute():
        raise ValueError("output_dir must be an absolute Path")
    if type(identity) is not ManualMcapIdentity:
        raise ValueError("identity must be a ManualMcapIdentity")
    if isinstance(frames, (str, bytes)) or not isinstance(frames, Iterable):
        raise ValueError("frames must be a finite iterable of ManualMid360Frame")

    output_dir.mkdir(parents=True, exist_ok=True)
    mcap_path = output_dir / "session.mcap"
    result_path = output_dir / "recorder-result.json"
    descriptor = load_v2_descriptor()
    if descriptor.sha256 != identity.descriptor_sha256:
        raise ValueError("identity descriptor does not match current v2 descriptor")
    counts = {contract.topic: 0 for contract in V2_TOPICS}
    with mcap_path.open("wb") as output:
        writer = Writer(
            output,
            compression=CompressionType.NONE,
            use_chunking=False,
            enable_crcs=True,
            enable_data_crcs=True,
        )
        writer.start(profile="protobuf", library="slope-sim-manual-mid360")
        schema_id = writer.register_schema(
            name="slope_sim.interfaces.v2",
            encoding="protobuf",
            data=descriptor.serialized_file_descriptor_set,
        )
        channel_ids = {
            contract.topic: writer.register_channel(
                topic=contract.topic,
                message_encoding="protobuf",
                schema_id=schema_id,
                metadata={"type": contract.type_name},
            )
            for contract in V2_TOPICS
        }
        writer.add_metadata(
            "slope_sim.session_manifest",
            {
                "simulation_session_id": identity.simulation_session_id.hex(),
                "descriptor_sha256": identity.descriptor_sha256.hex(),
                "world_generation": str(identity.world_generation),
                "scene_id": identity.scene_id,
                "lidar_pattern_version": _PATTERN_VERSION,
                "lidar_pattern_sha256": _PATTERN_SHA256,
            },
        )
        codec = V2ProtoCodec(descriptor)
        frame_count = 0
        previous_timebase_ns: int | None = None
        for frame in frames:
            if type(frame) is not ManualMid360Frame:
                raise ValueError("frames must contain only ManualMid360Frame")
            cloud = frame.cloud
            if (
                cloud.sequence != frame_count
                or cloud.world_generation != identity.world_generation
                or cloud.simulation_session_id != identity.simulation_session_id
                or cloud.descriptor_sha256 != identity.descriptor_sha256
            ):
                raise ValueError("all frames must use continuous supplied session identity")
            if (
                previous_timebase_ns is not None
                and cloud.timebase_ns != previous_timebase_ns + 100_000_000
            ):
                raise ValueError("manual MID-360 frames must be exactly 100 ms apart")
            timestamp_ns = cloud.timebase_ns
            source_session = bytes(16)
            models: list[tuple[str, object]] = []
            for wheel_offset in range(10):
                wheel_sequence = frame_count * 10 + wheel_offset
                wheel_timestamp_ns = timestamp_ns + wheel_offset * 10_000_000
                models.extend(
                    (
                        (
                            "/sim/wheel/command",
                            WheelCommandV2(
                                wheel_timestamp_ns,
                                (),
                                (),
                                wheel_sequence,
                                identity.world_generation,
                                1,
                                "manual.capture",
                                source_session,
                                "manual",
                                identity.simulation_session_id,
                                identity.descriptor_sha256,
                            ),
                        ),
                        (
                            "/sim/wheel/state",
                            WheelStateV2(
                                wheel_timestamp_ns,
                                (),
                                (),
                                wheel_sequence,
                                identity.world_generation,
                                1,
                                "manual",
                                identity.simulation_session_id,
                                identity.descriptor_sha256,
                                CommandAuthorityState.WAITING,
                                "",
                                b"",
                                0,
                            ),
                        ),
                    )
                )
            rtk, imu = _pose_models(frame, identity)
            models.extend(
                (
                    ("/sim/rtk/state", rtk),
                    ("/sim/imu/attitude", imu),
                    ("/sim/lidar/points", cloud),
                )
            )
            for topic, model in models:
                writer.add_message(
                    channel_ids[topic],
                    log_time=model.timestamp_ns if topic != "/sim/lidar/points" else model.timebase_ns,
                    publish_time=model.timestamp_ns if topic != "/sim/lidar/points" else model.timebase_ns,
                    sequence=model.sequence,
                    data=codec.encode(model).payload,
                )
                counts[topic] += 1
            frame_count += 1
            previous_timebase_ns = timestamp_ns
        if frame_count == 0:
            raise ValueError("frames must be nonempty")
        writer.finish()
    result_path.write_text(
        json.dumps(
            {
                "clean_shutdown": True,
                "mcap": str(mcap_path),
                "recorded_count": sum(counts.values()),
                "role": "recorder",
                "topics": counts,
            },
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )
    return mcap_path, result_path
