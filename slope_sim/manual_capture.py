# 手动 MID-360 采集核心：驾驶期只冻结场景并缓冲轨迹，绝不执行点云扫描。
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from enum import Enum
import json
import math
from pathlib import Path
import tempfile

from slope_sim.scene_config import SceneDocument, dump_scene_atomic


_DURATION_OPTIONS_SEC = (60, 90, 180, None)
_TRAJECTORY_FLUSH_SIZE = 256


class ManualCaptureStatus(Enum):
    """手动采集会话的终态；重建任务只接受 FINALIZED 会话。"""

    RECORDING = "recording"
    FINALIZED = "finalized"
    ABORTED = "aborted"


class ManualCaptureUiState(Enum):
    """Dashboard 可见的采集状态；与磁盘回执终态刻意分离。"""

    IDLE = "idle"
    RECORDING = "recording"
    GENERATING = "generating"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass(frozen=True, slots=True)
class ManualCaptureAction:
    """Dashboard 到手动主循环的一次性采集请求。"""

    kind: str
    duration_limit_sec: int | None = None
    lvx2_path: Path | None = None
    mcap_path: Path | None = None

    @classmethod
    def start(cls, duration_limit_sec: int | None) -> "ManualCaptureAction":
        if duration_limit_sec not in _DURATION_OPTIONS_SEC:
            raise ValueError("duration_limit_sec must be one of 60, 90, 180, or None")
        return cls("start", duration_limit_sec=duration_limit_sec)

    @classmethod
    def stop(cls) -> "ManualCaptureAction":
        return cls("stop")

    @classmethod
    def open_viewer(cls, lvx2_path: Path) -> "ManualCaptureAction":
        if not isinstance(lvx2_path, Path) or not lvx2_path.is_absolute():
            raise ValueError("lvx2_path must be an absolute Path")
        return cls("open_viewer", lvx2_path=lvx2_path)

    @classmethod
    def compress_mcap(cls, mcap_path: Path) -> "ManualCaptureAction":
        if not isinstance(mcap_path, Path) or not mcap_path.is_absolute():
            raise ValueError("mcap_path must be an absolute Path")
        return cls("compress_mcap", mcap_path=mcap_path)


@dataclass(frozen=True, slots=True)
class ManualCaptureReceipt:
    """供 Dashboard 和后续离线重建读取的轻量会话回执。"""

    status: ManualCaptureStatus
    output_dir: Path
    scene_path: Path
    trajectory_path: Path
    receipt_path: Path
    duration_limit_sec: int | None
    world_generation: int
    started_sim_time_ns: int
    finished_sim_time_ns: int | None
    sample_count: int


@dataclass(frozen=True, slots=True)
class _PoseSample:
    sim_time_ns: int
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]

    def to_mapping(self) -> dict[str, object]:
        return {
            "sim_time_ns": self.sim_time_ns,
            "position": list(self.position),
            "orientation": list(self.orientation),
        }


def _require_uint64(name: str, value: object) -> int:
    if type(value) is not int or not 0 <= value <= (1 << 64) - 1:
        raise ValueError(f"{name} must be a uint64")
    return value


def _require_vector(name: str, value: object, length: int) -> tuple[float, ...]:
    if not isinstance(value, tuple) or len(value) != length:
        raise ValueError(f"{name} must be a tuple of {length} finite values")
    normalized: list[float] = []
    for component in value:
        if isinstance(component, bool) or not isinstance(component, (int, float)):
            raise ValueError(f"{name} must be a tuple of {length} finite values")
        number = float(component)
        if not math.isfinite(number):
            raise ValueError(f"{name} must be a tuple of {length} finite values")
        normalized.append(number)
    return tuple(normalized)


def _write_json_atomic(path: Path, mapping: dict[str, object]) -> Path:
    """发布终态回执时避免 Dashboard 读到半个 JSON。"""
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            json.dump(mapping, stream, ensure_ascii=False, sort_keys=True)
            stream.write("\n")
            stream.flush()
        temporary.replace(path)
        return path
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


class ManualCaptureSession:
    """单次采集的可写轨迹缓冲和最终回执。"""

    def __init__(
        self,
        *,
        output_dir: Path,
        scene_path: Path,
        duration_limit_sec: int | None,
        world_generation: int,
        started_sim_time_ns: int,
    ) -> None:
        self.output_dir = output_dir
        self.scene_path = scene_path
        self.trajectory_path = output_dir / "trajectory.jsonl"
        self.receipt_path = output_dir / "capture.json"
        self.duration_limit_sec = duration_limit_sec
        self.world_generation = world_generation
        self.started_sim_time_ns = started_sim_time_ns
        self._status = ManualCaptureStatus.RECORDING
        self._sample_count = 0
        self._trajectory_buffer: list[_PoseSample] = []

    @property
    def status(self) -> ManualCaptureStatus:
        return self._status

    @property
    def sample_count(self) -> int:
        return self._sample_count

    def record_pose(
        self,
        *,
        sim_time_ns: int,
        position: tuple[float, float, float],
        orientation: tuple[float, float, float, float],
    ) -> None:
        """追加一条物理步位姿；批量落盘避免每帧阻塞 GUI。"""
        if self._status is not ManualCaptureStatus.RECORDING:
            raise RuntimeError(f"cannot record pose after {self._status.value}")
        timestamp = _require_uint64("sim_time_ns", sim_time_ns)
        if timestamp < self.started_sim_time_ns:
            raise ValueError("sim_time_ns must not precede started_sim_time_ns")
        normalized_position = _require_vector("position", position, 3)
        normalized_orientation = _require_vector("orientation", orientation, 4)
        self._trajectory_buffer.append(
            _PoseSample(
                timestamp,
                (normalized_position[0], normalized_position[1], normalized_position[2]),
                (
                    normalized_orientation[0],
                    normalized_orientation[1],
                    normalized_orientation[2],
                    normalized_orientation[3],
                ),
            )
        )
        self._sample_count += 1
        if len(self._trajectory_buffer) >= _TRAJECTORY_FLUSH_SIZE:
            self._flush_trajectory()

    def _flush_trajectory(self) -> None:
        if not self._trajectory_buffer:
            return
        with self.trajectory_path.open("a", encoding="utf-8", newline="\n") as stream:
            for sample in self._trajectory_buffer:
                stream.write(json.dumps(sample.to_mapping(), separators=(",", ":")))
                stream.write("\n")
        self._trajectory_buffer.clear()

    def _receipt(
        self,
        *,
        status: ManualCaptureStatus,
        finished_sim_time_ns: int | None,
        reason: str | None,
    ) -> ManualCaptureReceipt:
        payload: dict[str, object] = {
            "schema": "manual-mid360-capture-v1",
            "status": status.value,
            "duration_limit_sec": self.duration_limit_sec,
            "world_generation": self.world_generation,
            "started_sim_time_ns": self.started_sim_time_ns,
            "finished_sim_time_ns": finished_sim_time_ns,
            "sample_count": self._sample_count,
            "scene": self.scene_path.name,
            "trajectory": self.trajectory_path.name,
        }
        if reason is not None:
            payload["reason"] = reason
        _write_json_atomic(self.receipt_path, payload)
        return ManualCaptureReceipt(
            status,
            self.output_dir,
            self.scene_path,
            self.trajectory_path,
            self.receipt_path,
            self.duration_limit_sec,
            self.world_generation,
            self.started_sim_time_ns,
            finished_sim_time_ns,
            self._sample_count,
        )

    def finish(self, *, finished_sim_time_ns: int) -> ManualCaptureReceipt:
        """完成轨迹文件并发布供后处理消费的 FINALIZED 回执。"""
        if self._status is not ManualCaptureStatus.RECORDING:
            raise RuntimeError(f"cannot finish capture after {self._status.value}")
        finished = _require_uint64("finished_sim_time_ns", finished_sim_time_ns)
        if finished < self.started_sim_time_ns:
            raise ValueError("finished_sim_time_ns must not precede started_sim_time_ns")
        self._flush_trajectory()
        self._status = ManualCaptureStatus.FINALIZED
        return self._receipt(
            status=self._status,
            finished_sim_time_ns=finished,
            reason=None,
        )

    def abort(self, *, reason: str) -> ManualCaptureReceipt:
        """终止未完成会话；此状态永远不能进入 LVX2 重建。"""
        if self._status is not ManualCaptureStatus.RECORDING:
            raise RuntimeError(f"cannot abort capture after {self._status.value}")
        if not isinstance(reason, str) or not reason:
            raise ValueError("reason must be a nonempty string")
        self._flush_trajectory()
        self._status = ManualCaptureStatus.ABORTED
        return self._receipt(
            status=self._status,
            finished_sim_time_ns=None,
            reason=reason,
        )


class ManualCaptureRecorder:
    """创建彼此隔离的手动采集会话。"""

    duration_options_sec = _DURATION_OPTIONS_SEC

    def __init__(self, output_root: Path) -> None:
        if not isinstance(output_root, Path):
            raise ValueError("output_root must be a Path")
        self._output_root = output_root

    def _create_output_dir(self) -> Path:
        """以本地时间创建人工采集目录；同秒冲突按连续后缀避让。"""
        stem = datetime.now().strftime("capture-%Y%m%d-%H%M%S")
        for suffix in range(10_000):
            name = stem if suffix == 0 else f"{stem}-{suffix:02d}"
            candidate = self._output_root / name
            try:
                candidate.mkdir()
            except FileExistsError:
                continue
            return candidate
        raise RuntimeError(f"capture directory suffixes exhausted for {stem}")

    def start(
        self,
        *,
        scene_document: SceneDocument,
        world_generation: int,
        duration_limit_sec: int | None,
        started_sim_time_ns: int,
    ) -> ManualCaptureSession:
        """冻结当前逻辑场景，再返回驾驶期可轻量写入的会话。"""
        if not isinstance(scene_document, SceneDocument):
            raise ValueError("scene_document must be a SceneDocument")
        generation = _require_uint64("world_generation", world_generation)
        if generation == 0:
            raise ValueError("world_generation must be positive")
        if duration_limit_sec not in self.duration_options_sec:
            raise ValueError("duration_limit_sec must be one of 60, 90, 180, or None")
        started = _require_uint64("started_sim_time_ns", started_sim_time_ns)
        self._output_root.mkdir(parents=True, exist_ok=True)
        output_dir = self._create_output_dir()
        scene_path = dump_scene_atomic(scene_document, output_dir / "scene.yaml")
        return ManualCaptureSession(
            output_dir=output_dir,
            scene_path=scene_path,
            duration_limit_sec=duration_limit_sec,
            world_generation=generation,
            started_sim_time_ns=started,
        )
