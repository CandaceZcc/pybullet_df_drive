"""阶段四 B2：独立渲染 v2 有界快照的最小 PySide6 Dashboard。"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time
from typing import Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainterPath, QPen
from PySide6.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QGraphicsScene,
    QGraphicsView,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from slope_sim.interfaces.v2.dashboard_snapshot import (
    V2DashboardSnapshot,
    V2DashboardSnapshotStore,
    V2TopicObservation,
)
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity
from slope_sim.interfaces.v2.topics import V2_TOPICS


_EVIDENCE_SECTIONS = (
    "recorder", "replay", "export", "viewer_startup", "viewer_display",
)
_PREVIEW_MAX_HZ = 5.0
_RECORDER_TOPICS = tuple(contract.topic for contract in V2_TOPICS)
_REPLAY_TOPICS = tuple(
    f"/replay{contract.topic}"
    for contract in V2_TOPICS
    if contract.direction == "publish"
)
_SECTION_FIELDS = {
    "recorder": {"identity", "clean_shutdown", "mcap", "topic_counts"},
    "replay": {"identity", "clean_shutdown", "mcap", "result", "topic_counts"},
    "export": {
        "identity", "source_mcap", "lvx2", "synthetic", "lossiness",
        "pcd_count", "ply_count", "pcd_artifacts", "ply_artifacts",
    },
    "viewer_startup": {"identity", "lvx2", "smoke_passed"},
    "viewer_display": {
        "identity", "lvx2", "nonempty_pointcloud_visible",
        "playback_progress_observed", "screenshot",
    },
}


def _evidence_identity(value: object) -> tuple[str, str, int, str]:
    """验证离线链的固定身份，不允许某个 section 偷换 session 或 world。"""
    if not isinstance(value, dict) or set(value) != {"session", "descriptor", "world", "scene"}:
        raise ValueError("evidence identity must be an object")
    session = value.get("session")
    descriptor = value.get("descriptor")
    world = value.get("world")
    scene = value.get("scene")
    if (not isinstance(session, str) or len(session) != 32 or
            not isinstance(descriptor, str) or len(descriptor) != 64 or
            not isinstance(world, int) or isinstance(world, bool) or world <= 0 or
            not isinstance(scene, str) or not scene):
        raise ValueError("invalid evidence identity")
    try:
        bytes.fromhex(session)
        bytes.fromhex(descriptor)
    except ValueError as error:
        raise ValueError("invalid evidence identity") from error
    return session, descriptor, world, scene


def _evidence_artifact(value: object) -> tuple[str, str]:
    """只接受现场 hash 过的绝对普通文件，阻止 evidence 指向相对或目录路径。"""
    if not isinstance(value, dict) or set(value) != {"path", "sha256"}:
        raise ValueError("evidence artifact must contain path and sha256")
    path_text, digest = value["path"], value["sha256"]
    if not isinstance(path_text, str) or not Path(path_text).is_absolute():
        raise ValueError("evidence artifact path must be absolute")
    path = Path(path_text)
    try:
        resolved = path.resolve(strict=True)
    except OSError as error:
        raise ValueError("evidence artifact must exist") from error
    if path != resolved or not resolved.is_file() or resolved.is_symlink():
        raise ValueError("evidence artifact must be a normalized regular file")
    if not isinstance(digest, str) or len(digest) != 64:
        raise ValueError("evidence artifact SHA-256 is invalid")
    actual = hashlib.sha256(resolved.read_bytes()).hexdigest()
    if actual != digest:
        raise ValueError("evidence artifact SHA-256 does not match")
    return str(resolved), digest


def _evidence_count(name: str, value: object) -> int:
    """证据计数只接受非负 exact int，拒绝 bool 的 Python 整数兼容性。"""
    if type(value) is not int or value < 0:
        raise ValueError(f"evidence {name} must be a nonnegative integer")
    return value


def _evidence_topic_counts(
    section: str, value: object, expected_topics: tuple[str, ...],
) -> dict[str, int]:
    """要求 recorder/replay 给出完整且不增不减的固定话题计数。"""
    if not isinstance(value, dict) or set(value) != set(expected_topics):
        raise ValueError(f"evidence {section} topic counts are incomplete")
    return {
        topic: _evidence_count(f"{section} count for {topic}", value[topic])
        for topic in expected_topics
    }


def _evidence_artifact_list(name: str, value: object, expected_count: int) -> None:
    """逐项现场验证导出 artifact，并绑定声明的确切计数。"""
    if not isinstance(value, list) or len(value) != expected_count:
        raise ValueError(f"evidence {name} artifacts do not match count")
    for artifact in value:
        _evidence_artifact(artifact)


def _artifact_text(name: str, artifact: dict[str, str]) -> str:
    """以可复制审计格式显示规范路径和完整 SHA-256。"""
    return f"{name}.path={artifact['path']} | {name}.sha256={artifact['sha256']}"


def _counts_text(counts: dict[str, int], topics: tuple[str, ...]) -> str:
    """按固定合同顺序显示逐话题计数。"""
    return " | ".join(f"{topic}={counts[topic]}" for topic in topics)


def load_offline_evidence(path: Path) -> dict[str, object]:
    """在 GUI/runtime 启动前验证显式离线证据；绝不执行其任何字符串字段。"""
    if not isinstance(path, Path) or not path.is_absolute():
        raise ValueError("evidence path must be absolute")
    resolved = path.resolve(strict=True)
    if resolved != path or not resolved.is_file():
        raise ValueError("evidence path must be a normalized regular file")
    try:
        document = json.loads(resolved.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError("evidence JSON is invalid") from error
    if (not isinstance(document, dict) or type(document.get("schema_version")) is not int
            or document.get("schema_version") != 1
            or document.get("kind") != "stage4_v2_offline_evidence"):
        raise ValueError("evidence schema is invalid")
    identity = _evidence_identity(document.get("identity"))
    if set(document) != {"schema_version", "kind", "identity", *_EVIDENCE_SECTIONS}:
        raise ValueError("evidence sections are invalid")
    sections: dict[str, dict[str, object]] = {}
    for name in _EVIDENCE_SECTIONS:
        section = document.get(name)
        if (not isinstance(section, dict) or set(section) != _SECTION_FIELDS[name]
                or _evidence_identity(section.get("identity")) != identity):
            raise ValueError("evidence section identity does not match")
        sections[name] = section
    recorder = sections["recorder"]
    if recorder.get("clean_shutdown") is not True:
        raise ValueError("evidence recorder clean shutdown is required")
    _evidence_topic_counts("recorder", recorder.get("topic_counts"), _RECORDER_TOPICS)
    recorder_mcap = _evidence_artifact(recorder.get("mcap"))

    replay = sections["replay"]
    if replay.get("clean_shutdown") is not True:
        raise ValueError("evidence replay clean shutdown is required")
    _evidence_topic_counts("replay", replay.get("topic_counts"), _REPLAY_TOPICS)
    _evidence_artifact(replay.get("mcap"))
    _evidence_artifact(replay.get("result"))
    export = sections["export"]
    lossiness = export.get("lossiness")
    if (type(export.get("synthetic")) is not bool or not isinstance(lossiness, dict)
            or not lossiness or any(
                not isinstance(key, str) or not key or type(value) not in {bool, str}
                or (type(value) is str and not value)
                for key, value in lossiness.items()
            )):
        raise ValueError("export evidence is incomplete")
    pcd_count = _evidence_count("export PCD count", export.get("pcd_count"))
    ply_count = _evidence_count("export PLY count", export.get("ply_count"))
    _evidence_artifact_list("PCD", export.get("pcd_artifacts"), pcd_count)
    _evidence_artifact_list("PLY", export.get("ply_artifacts"), ply_count)
    if _evidence_artifact(export.get("source_mcap"))[1] != recorder_mcap[1]:
        raise ValueError("export source MCAP does not bind recorder evidence")
    lvx2 = _evidence_artifact(export.get("lvx2"))
    startup = sections["viewer_startup"]
    if (startup.get("smoke_passed") is not True
            or _evidence_artifact(startup.get("lvx2")) != lvx2):
        raise ValueError("viewer startup LVX2 does not bind export evidence")
    display = sections["viewer_display"]
    if (_evidence_artifact(display.get("lvx2")) != lvx2 or
            display.get("nonempty_pointcloud_visible") is not True or
            display.get("playback_progress_observed") is not True):
        raise ValueError("viewer display evidence is incomplete")
    _evidence_artifact(display.get("screenshot"))
    return document


class V2DashboardWidget(QWidget):
    """在 Qt GUI 线程显示 v2 wheel、中心 LiDAR、三点 RTK 与 IMU。"""

    def __init__(
        self,
        descriptor: DescriptorIdentity,
        *,
        offline_viewer_launcher: Callable[[Path], None] | None = None,
        live_viewer_launcher: Callable[[], Callable[[], None]] | None = None,
    ) -> None:
        if type(descriptor) is not DescriptorIdentity:
            raise ValueError("descriptor must be an exact DescriptorIdentity")
        if offline_viewer_launcher is not None and not callable(offline_viewer_launcher):
            raise ValueError("offline_viewer_launcher must be callable")
        if live_viewer_launcher is not None and not callable(live_viewer_launcher):
            raise ValueError("live_viewer_launcher must be callable")
        super().__init__()
        self._descriptor = descriptor
        self._offline_viewer_launcher = offline_viewer_launcher
        # 仅保存受信 launcher 交回的关闭句柄；完整点云始终不进入 Dashboard。
        self._live_viewer_launcher = live_viewer_launcher
        self._live_viewer_close: Callable[[], None] | None = None
        self._offline_viewer_lvx2: Path | None = None
        self._last_store_revision: tuple[object, ...] | None = None
        self._last_preview_identity: tuple[object, ...] | None = None
        self._preview_rtk: object | None = None
        self._preview_next_render_at = 0.0
        self.setWindowTitle("Slope Sim Stage 4 Dashboard")
        self.setMinimumWidth(800)

        layout = QVBoxLayout(self)
        title = QLabel("Stage 4 v2 Telemetry Evidence", self)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(title)
        self.identity_value = QLabel("session=-- | descriptor=-- | world=-- | model=-- | lidar_link | MID-360 simulation", self)
        self.identity_value.setWordWrap(True)
        layout.addWidget(self.identity_value)
        status_fields = QFormLayout()
        self.ecal_status_value = QLabel("等待 eCAL v2 初始化", self)
        self.ecal_status_value.setWordWrap(True)
        status_fields.addRow("eCAL 状态", self.ecal_status_value)
        layout.addLayout(status_fields)
        self.topic_table = QTableWidget(len(V2_TOPICS), 11, self)
        self.topic_table.setHorizontalHeaderLabels((
            "Topic", "Target Hz", "Actual Hz", "Peers/State", "Errors",
            "Transport Drops", "Sequence Gaps", "Latest Sequence", "Age", "Points", "Action",
        ))
        self.topic_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.topic_table.setMinimumHeight(180)
        layout.addWidget(self.topic_table)
        viewer_actions = QHBoxLayout()
        self.live_viewer_button = QPushButton("打开实时点云", self)
        self.live_viewer_button.setEnabled(live_viewer_launcher is not None)
        self.live_viewer_close_button = QPushButton("关闭实时点云", self)
        self.live_viewer_close_button.setEnabled(False)
        self.live_viewer_status = QLabel(
            "已配置，点击打开实时点云" if live_viewer_launcher is not None else "未配置/连接未验证",
            self,
        )
        self.offline_viewer_button = QPushButton("Launch Livox Viewer 2", self)
        self.offline_viewer_button.setEnabled(False)
        self.offline_viewer_status = QLabel("未配置/连接未验证", self)
        self.live_viewer_button.clicked.connect(self._launch_live_viewer)
        self.live_viewer_close_button.clicked.connect(self._close_live_viewer)
        self.offline_viewer_button.clicked.connect(self._launch_offline_viewer)
        viewer_actions.addWidget(self.live_viewer_button)
        viewer_actions.addWidget(self.live_viewer_close_button)
        viewer_actions.addWidget(self.live_viewer_status)
        viewer_actions.addWidget(self.offline_viewer_button)
        viewer_actions.addWidget(self.offline_viewer_status)
        layout.addLayout(viewer_actions)
        self.offline_evidence_title = QLabel("离线已验证证据，非实时状态", self)
        layout.addWidget(self.offline_evidence_title)
        evidence_fields = QFormLayout()
        self.offline_evidence_values = {
            name: QLabel("未提供 verifier evidence", self)
            for name in _EVIDENCE_SECTIONS
        }
        for name, label in self.offline_evidence_values.items():
            label.setWordWrap(True)
            evidence_fields.addRow(name, label)
        layout.addLayout(evidence_fields)
        self.offline_evidence_detail = QPlainTextEdit(self)
        self.offline_evidence_detail.setReadOnly(True)
        self.offline_evidence_detail.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self.offline_evidence_detail.setMinimumHeight(96)
        self.offline_evidence_detail.setMaximumHeight(160)
        self.offline_evidence_detail.setPlainText("未提供 verifier evidence")
        layout.addWidget(self.offline_evidence_detail)
        fields = QFormLayout()
        self.wheel_value = QLabel("--", self)
        self.lidar_value = QLabel("--", self)
        self.rtk_value = QLabel("--", self)
        self.imu_value = QLabel("--", self)
        self.lidar_value.setWordWrap(True)
        self.rtk_value.setWordWrap(True)
        fields.addRow("Wheel state", self.wheel_value)
        fields.addRow("Central LiDAR", self.lidar_value)
        fields.addRow("RTK L/C/R", self.rtk_value)
        fields.addRow("IMU", self.imu_value)
        layout.addLayout(fields)
        self.sampled_preview_group = QGroupBox("轻量空间预览，非验收证据", self)
        self.sampled_preview_group.setCheckable(True)
        self.sampled_preview_group.setChecked(False)
        preview_layout = QVBoxLayout(self.sampled_preview_group)
        self.top_view = QGraphicsView(self.sampled_preview_group)
        self.top_view.setMinimumHeight(220)
        self._scene = QGraphicsScene(self)
        self.top_view.setScene(self._scene)
        preview_layout.addWidget(self.top_view)
        self.sampled_preview_group.toggled.connect(self._set_preview_enabled)
        self.top_view.setVisible(False)
        layout.addWidget(self.sampled_preview_group)

    def _launch_live_viewer(self) -> None:
        """只经受信回调启动独立显示链；其失败不能影响 Simulator 或 eCAL。"""
        if self._live_viewer_launcher is None:
            self.live_viewer_status.setText("未配置/连接未验证")
            return
        if self._live_viewer_close is not None:
            self.live_viewer_status.setText("实时点云显示已打开")
            return
        try:
            close = self._live_viewer_launcher()
            if not callable(close):
                raise TypeError("trusted live viewer launcher returned no close handle")
        except Exception as error:
            self.live_viewer_status.setText(f"启动失败：{type(error).__name__}: {error}")
            return
        self._live_viewer_close = close
        self.live_viewer_close_button.setEnabled(True)
        self.live_viewer_status.setText("实时点云显示已打开（关闭仅停止 Bridge/RViz2）")

    def launch_live_viewer(self) -> None:
        """供正式 CLI 自动打开显示链，复用与 Dashboard 按钮完全相同的边界。"""
        self._launch_live_viewer()

    def _close_live_viewer(self) -> None:
        """关闭独立显示链并允许重启；不向核心仿真、Command 或 Recorder 发信号。"""
        close = self._live_viewer_close
        if close is None:
            self.live_viewer_close_button.setEnabled(False)
            return
        self._live_viewer_close = None
        self.live_viewer_close_button.setEnabled(False)
        try:
            close()
        except Exception as error:
            self.live_viewer_status.setText(f"关闭失败：{type(error).__name__}: {error}；可重试打开")
            return
        self.live_viewer_status.setText("实时点云显示已关闭；可重新打开")

    def _launch_offline_viewer(self) -> None:
        """只把已验 hash 的 LVX2 交给受信任启动器，不从界面文本拼接命令。"""
        if self._offline_viewer_launcher is None or self._offline_viewer_lvx2 is None:
            self.offline_viewer_status.setText("未配置/连接未验证")
            return
        try:
            self._offline_viewer_launcher(self._offline_viewer_lvx2)
        except Exception as error:
            self.offline_viewer_status.setText(
                f"启动失败：{type(error).__name__}: {error}"
            )
            return
        self.offline_viewer_status.setText(
            f"已请求启动：{self._offline_viewer_lvx2.name}；等待导入验证"
        )

    @staticmethod
    def _require_identity(value: object, snapshot: V2DashboardSnapshot) -> None:
        """阻止 GUI 误渲染跨 session、descriptor 或 world 的轻量遥测。"""
        if (
            value.simulation_session_id != snapshot.simulation_session_id
            or value.descriptor_sha256 != snapshot.descriptor_sha256
            or value.world_generation != snapshot.world_generation
        ):
            raise ValueError("telemetry identity does not match dashboard snapshot")

    @staticmethod
    def _snapshot_revision(snapshot: V2DashboardSnapshot) -> tuple[object, ...]:
        """仅保留标量版本键，Dashboard 不缓存完整点云或 raw payload。"""
        return (
            snapshot.simulation_session_id,
            snapshot.descriptor_sha256,
            snapshot.world_generation,
            snapshot.lidar_timestamp_ns,
            snapshot.lidar_sequence,
            None if snapshot.wheel_state is None else snapshot.wheel_state.sequence,
            None if snapshot.rtk is None else snapshot.rtk.sequence,
            None if snapshot.imu is None else snapshot.imu.sequence,
            snapshot.topic_observations,
        )

    @classmethod
    def _preview_identity(cls, snapshot: V2DashboardSnapshot) -> tuple[object, ...]:
        """仅当轻量 RTK 预览变化时重建图元。"""
        return (
            None if snapshot.rtk is None else snapshot.rtk.timestamp_ns,
            None if snapshot.rtk is None else snapshot.rtk.sequence,
        )

    def _set_preview_enabled(self, enabled: bool) -> None:
        """预览可随时关闭；关闭时立即释放所有 Qt 图元。"""
        self.top_view.setVisible(enabled)
        if not enabled:
            self._scene.clear()
            return
        self._preview_next_render_at = 0.0
        if self._preview_rtk is not None:
            self._render_top_view()
            self._preview_next_render_at = time.monotonic() + 1.0 / _PREVIEW_MAX_HZ

    def _render_top_view(self) -> None:
        """只绘制 RTK 三点；正式 Dashboard 不解码或保存完整 MID-360 点云。"""
        self._scene.clear()
        # 与扩大后的离线 MID-360 60 m 范围保持可读比例。
        scale = 60.0
        if self._preview_rtk is not None:
            rtk_pen = QPen(QColor("#188038"))
            rtk_pen.setWidth(3)
            rtk_path = QPainterPath()
            for point in (
                self._preview_rtk.left,
                self._preview_rtk.center,
                self._preview_rtk.right,
            ):
                rtk_path.addEllipse(
                    point.x_m * scale - 2.0,
                    -point.y_m * scale - 2.0,
                    4.0,
                    4.0,
                )
            self._scene.addPath(rtk_path, rtk_pen)
        self._scene.setSceneRect(self._scene.itemsBoundingRect().adjusted(-20.0, -20.0, 20.0, 20.0))

    @staticmethod
    def _topic_action(observation: V2TopicObservation | None, topic: str) -> str:
        """将可恢复的 eCAL、peer 和 LiDAR worker 故障投影为下一步操作。"""
        if observation is None or observation.protocol_state == "not_checked":
            return "检查 eCAL 初始化、配置路径和 descriptor"
        if observation.peer_count is None or observation.peer_count == 0:
            if topic == "/sim/wheel/command":
                return "启动唯一 C++ Command peer，并确认其已注册"
            return "检查对应 eCAL consumer peer 是否已启动"
        if observation.error_count:
            if topic == "/sim/lidar/points":
                return "检查 MID-360 worker 与其终端错误后重启 v2 runtime"
            return "检查 eCAL/runtime 终端错误后重启对应 peer"
        if observation.protocol_state != "verified":
            return "检查 peer 的 Protobuf 类型与 descriptor 是否一致"
        if observation.actual_hz is None and observation.telemetry_observed_at is not None:
            if topic == "/sim/lidar/points":
                return "检查 MID-360 worker 是否仍在产生帧"
            return "检查该 topic 的 v2 publisher 是否仍在运行"
        return "运行正常"

    @staticmethod
    def _ecal_status(observations: tuple[V2TopicObservation, ...]) -> str:
        """只有全部五话题经 peer 验证时才报告 eCAL 已验证。"""
        if len(observations) != len(V2_TOPICS):
            return "等待 eCAL v2 初始化"
        if all(item.protocol_state == "verified" and (item.peer_count or 0) > 0 for item in observations):
            return "eCAL 已验证（五话题）"
        if any(item.error_count for item in observations):
            return "eCAL 异常：查看各 topic 的恢复操作"
        if any(item.peer_count is None or item.peer_count == 0 for item in observations):
            return "等待 eCAL peer：查看各 topic 的恢复操作"
        return "eCAL 初始化/协议验证中：查看各 topic 的恢复操作"

    def set_offline_evidence(self, evidence: dict[str, object]) -> None:
        """显示已验证离线链摘要；只在有受信启动器时开放对应 LVX2。"""
        if not isinstance(evidence, dict):
            raise ValueError("offline evidence must be a validated object")
        recorder = evidence["recorder"]
        replay = evidence["replay"]
        export = evidence["export"]
        startup = evidence["viewer_startup"]
        display = evidence["viewer_display"]
        self._offline_viewer_lvx2 = Path(export["lvx2"]["path"])
        if self._offline_viewer_launcher is not None:
            self.offline_viewer_button.setEnabled(True)
            self.offline_viewer_status.setText("已验证 LVX2，点击启动")
        self.offline_evidence_values["recorder"].setText(
            "clean_shutdown=%s | %s"
            % (
                recorder["clean_shutdown"],
                _counts_text(recorder["topic_counts"], _RECORDER_TOPICS),
            )
        )
        self.offline_evidence_values["replay"].setText(
            "clean_shutdown=%s | 4 topics | %s"
            % (
                replay["clean_shutdown"],
                _counts_text(replay["topic_counts"], _REPLAY_TOPICS),
            )
        )
        self.offline_evidence_values["export"].setText(
            "synthetic=%s | lossiness=%s | PCD=%d | PLY=%d"
            % (
                export["synthetic"], export["lossiness"], export["pcd_count"],
                export["ply_count"],
            )
        )
        self.offline_evidence_values["viewer_startup"].setText(
            "smoke_passed=%s" % startup["smoke_passed"]
        )
        self.offline_evidence_values["viewer_display"].setText(
            "nonempty=%s | playback_progress=%s"
            % (
                display["nonempty_pointcloud_visible"],
                display["playback_progress_observed"],
            )
        )
        audit_lines = [
            "[recorder]",
            _artifact_text("mcap", recorder["mcap"]),
            "[replay]",
            _artifact_text("mcap", replay["mcap"]),
            _artifact_text("result", replay["result"]),
            "[export]",
            _artifact_text("source_mcap", export["source_mcap"]),
            _artifact_text("lvx2", export["lvx2"]),
            *(
                _artifact_text(f"pcd[{index}]", artifact)
                for index, artifact in enumerate(export["pcd_artifacts"])
            ),
            *(
                _artifact_text(f"ply[{index}]", artifact)
                for index, artifact in enumerate(export["ply_artifacts"])
            ),
            "[viewer_startup]",
            _artifact_text("lvx2", startup["lvx2"]),
            "[viewer_display]",
            _artifact_text("lvx2", display["lvx2"]),
            _artifact_text("screenshot", display["screenshot"]),
        ]
        self.offline_evidence_detail.setPlainText("\n".join(audit_lines))

    def update_snapshot(
        self, snapshot: V2DashboardSnapshot, *, observed_now: float | None = None,
    ) -> None:
        """由 GUI 定时器调用，消费一份不可变 v2 snapshot 并刷新所有可见字段。"""
        if type(snapshot) is not V2DashboardSnapshot:
            raise ValueError("snapshot must be an exact V2DashboardSnapshot")
        if snapshot.descriptor_sha256 != self._descriptor.sha256:
            raise ValueError("dashboard snapshot descriptor does not match widget")
        self.identity_value.setText(
            "session=%s | descriptor=%s | world=%d | %s | lidar_link | MID-360 simulation"
            % (snapshot.simulation_session_id.hex()[:12], snapshot.descriptor_sha256.hex()[:12],
               snapshot.world_generation, snapshot.robot_model)
        )
        now = time.monotonic() if observed_now is None else observed_now
        observations = {item.topic: item for item in snapshot.topic_observations}
        self.ecal_status_value.setText(self._ecal_status(snapshot.topic_observations))
        for row, contract in enumerate(V2_TOPICS):
            observation = observations.get(contract.topic)
            age = (
                None
                if observation is None or observation.telemetry_observed_at is None
                else now - observation.telemetry_observed_at
            )
            latest_sequence = None if observation is None else observation.latest_sequence
            point_count = None if observation is None else observation.point_count
            if contract.topic == "/sim/lidar/points" and snapshot.lidar_sequence is not None:
                latest_sequence = snapshot.lidar_sequence
                point_count = snapshot.lidar_point_count
            cells = (
                contract.topic,
                str(contract.rate_hz),
                "--" if observation is None or observation.actual_hz is None else f"{observation.actual_hz:.1f}",
                "--/not_checked" if observation is None else "%s/%s" % (
                    "--" if observation.peer_count is None else observation.peer_count,
                    observation.protocol_state,
                ),
                "0" if observation is None else str(observation.error_count),
                "0" if observation is None else str(observation.dropped_count),
                "0" if observation is None else str(observation.sequence_gap_count),
                "--" if latest_sequence is None else str(latest_sequence),
                "--" if age is None else f"{max(age, 0.0):.2f}s",
                "--" if point_count is None else str(point_count),
                self._topic_action(observation, contract.topic),
            )
            for column, value in enumerate(cells):
                self.topic_table.setItem(row, column, QTableWidgetItem(value))
        if snapshot.wheel_state is not None:
            self._require_identity(snapshot.wheel_state, snapshot)
            drive_values = ", ".join(
                f"{value:.3f}" for value in snapshot.wheel_state.drive_wheel_speed_rad_s
            )
            self.wheel_value.setText(
                f"drive=({drive_values}) rad/s"
            )
        else:
            self.wheel_value.setText("--")
        lidar_observation = observations.get("/sim/lidar/points")
        point_count = snapshot.lidar_point_count
        if snapshot.lidar_sequence is None:
            self.lidar_value.setText("--")
        elif point_count is None:
            self.lidar_value.setText(f"MID-360, seq={snapshot.lidar_sequence}")
        else:
            self.lidar_value.setText(
                f"MID-360, {point_count} points, seq={snapshot.lidar_sequence}"
            )
        if snapshot.rtk is not None:
            self._require_identity(snapshot.rtk, snapshot)
            self.rtk_value.setText(
                "L=(%.3f, %.3f, %.3f) C=(%.3f, %.3f, %.3f) R=(%.3f, %.3f, %.3f)"
                % (
                    snapshot.rtk.left.x_m,
                    snapshot.rtk.left.y_m,
                    snapshot.rtk.left.z_m,
                    snapshot.rtk.center.x_m,
                    snapshot.rtk.center.y_m,
                    snapshot.rtk.center.z_m,
                    snapshot.rtk.right.x_m,
                    snapshot.rtk.right.y_m,
                    snapshot.rtk.right.z_m,
                )
            )
        else:
            self.rtk_value.setText("--")
        if snapshot.imu is not None:
            self._require_identity(snapshot.imu, snapshot)
            self.imu_value.setText(
                f"roll={snapshot.imu.roll_rad:.3f}, pitch={snapshot.imu.pitch_rad:.3f} rad"
            )
        else:
            self.imu_value.setText("--")
        preview_identity = self._preview_identity(snapshot)
        if preview_identity != self._last_preview_identity:
            self._preview_rtk = snapshot.rtk
            self._last_preview_identity = preview_identity
            if (
                self.sampled_preview_group.isChecked()
                and now >= self._preview_next_render_at
            ):
                self._render_top_view()
                self._preview_next_render_at = now + 1.0 / _PREVIEW_MAX_HZ

    def refresh_from_store(self, store: V2DashboardSnapshotStore) -> bool:
        """供 GUI 定时器读取一次最新快照；空快照不产生任何渲染工作。"""
        if type(store) is not V2DashboardSnapshotStore:
            raise ValueError("store must be an exact V2DashboardSnapshotStore")
        now = time.monotonic()
        snapshot = store.snapshot(now=now)
        if snapshot is None:
            return False
        revision = self._snapshot_revision(snapshot)
        if revision == self._last_store_revision:
            return False
        self.update_snapshot(snapshot, observed_now=now)
        self._last_store_revision = revision
        return True
