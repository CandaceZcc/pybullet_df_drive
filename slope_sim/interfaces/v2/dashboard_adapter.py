"""阶段四 B2：独立渲染 v2 有界快照的最小 PySide6 Dashboard。"""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import time
from typing import Callable
from collections import deque

import numpy as np
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QColor, QImage, QPainter, QPainterPath, QPen, QPixmap
from PySide6.QtWidgets import (
    QFormLayout,
    QComboBox,
    QGroupBox,
    QGraphicsScene,
    QGraphicsView,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSlider,
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
_CLOUD_RENDER_MAX_HZ = 10.0
_MAX_CLOUD_HISTORY_FRAMES = 512
_MAX_CLOUD_RENDER_POINTS = 80_000
_CLOUD_IMAGE_SIZE = (960, 540)
_CLOUD_CAMERA_PRESETS = {
    "透视": {"distance": 24.0, "elevation": 26.0, "azimuth": 45.0},
    "俯视": {"distance": 24.0, "elevation": 90.0, "azimuth": 0.0},
    "侧视": {"distance": 24.0, "elevation": 0.0, "azimuth": 0.0},
}
_TOPIC_LABELS = {
    "/sim/wheel/command": "轮子命令",
    "/sim/wheel/state": "轮子状态",
    "/sim/lidar/points": "MID-360 实时点云",
    "/sim/rtk/state": "RTK 定位",
    "/sim/imu/attitude": "IMU 姿态",
}
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
        self._cloud_camera = dict(_CLOUD_CAMERA_PRESETS["透视"])
        self._cloud_camera["target"] = np.zeros(3, dtype=np.float32)
        self._cloud_next_render_at = 0.0
        self._cloud_render_failed = False
        self._cloud_render_failure = ""
        self._cloud_render_dropped_count = 0
        self._cloud_last_render_duration_ms = 0.0
        self._cloud_render_generation = 0
        self._cloud_render_pending = False
        self._cloud_render_inflight: Future[tuple[int, QImage, float]] | None = None
        self._cloud_render_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="runsim-cloud-image",
        )
        self._cloud_render_timer = QTimer(self)
        self._cloud_render_timer.setInterval(20)
        self._cloud_render_timer.timeout.connect(self._collect_cloud_render)
        self._live_viewer_close: Callable[[], None] | None = None
        self._offline_viewer_lvx2: Path | None = None
        self._last_store_revision: tuple[object, ...] | None = None
        self._last_preview_identity: tuple[object, ...] | None = None
        self._preview_rtk: object | None = None
        self._preview_next_render_at = 0.0
        self._cloud_frames: deque[object] = deque()
        self._last_cloud_frame: object | None = None
        self._last_cloud_sequence: int | None = None
        self._last_receiver_diagnostics: tuple[str, ...] = ()
        self._last_render_dropped_count = 0
        self.setWindowTitle("Slope Sim Stage 5 Dashboard")
        # 父级 Dashboard 在无显示器/窄侧栏下可能只有几百像素；宽度必须由父级决定。
        self.setMinimumWidth(0)
        self.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)

        layout = QVBoxLayout(self)
        self._dashboard_panels_detached = False
        self.interface_panel = QWidget(self)
        interface_layout = QVBoxLayout(self.interface_panel)
        self.lidar_panel = QWidget(self)
        lidar_layout = QVBoxLayout(self.lidar_panel)

        title = QLabel("Stage 4 v2 Telemetry Evidence", self.interface_panel)
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        interface_layout.addWidget(title)
        self.identity_value = QLabel("session=-- | descriptor=-- | world=-- | model=-- | lidar_link | MID-360 simulation", self.interface_panel)
        self.identity_value.setWordWrap(True)
        interface_layout.addWidget(self.identity_value)
        status_fields = QFormLayout()
        self.ecal_status_value = QLabel("等待 eCAL v2 初始化", self.interface_panel)
        self.ecal_status_value.setWordWrap(True)
        status_fields.addRow("eCAL 状态", self.ecal_status_value)
        interface_layout.addLayout(status_fields)
        rejection_group = QGroupBox("命令拒绝诊断", self.interface_panel)
        rejection_fields = QFormLayout(rejection_group)
        self.authority_rejection_value = QLabel("simulator authority: 累计 0", rejection_group)
        self.observer_rejection_value = QLabel("dashboard observer: 累计 0", rejection_group)
        self.authority_rejection_value.setWordWrap(True)
        self.observer_rejection_value.setWordWrap(True)
        rejection_fields.addRow("Simulator authority", self.authority_rejection_value)
        rejection_fields.addRow("Dashboard observer", self.observer_rejection_value)
        interface_layout.addWidget(rejection_group)
        self.topic_table = QTableWidget(len(V2_TOPICS), 11, self.interface_panel)
        self.topic_table.setHorizontalHeaderLabels((
            "Topic", "Target Hz", "Actual Hz", "Peers/State", "Errors",
            "Transport Drops", "Sequence Gaps", "Latest Sequence", "Age", "Points", "Action",
        ))
        self.topic_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.topic_table.setMinimumHeight(180)
        # 保留表格对象供自动化合同覆盖；人工界面改用可滚动的逐话题状态卡片。
        self.topic_table.hide()
        interface_layout.addWidget(self.topic_table)
        self.topic_cards: dict[str, dict[str, QLabel]] = {}
        cards = QWidget(self.interface_panel)
        cards_layout = QVBoxLayout(cards)
        cards_layout.setContentsMargins(0, 0, 0, 0)
        for contract in V2_TOPICS:
            group = QGroupBox(_TOPIC_LABELS[contract.topic], cards)
            fields = QFormLayout(group)
            labels = {
                "name": QLabel(_TOPIC_LABELS[contract.topic], group),
                "state": QLabel("-- / not_checked", group),
                "target": QLabel(f"{contract.rate_hz} Hz", group),
                "actual": QLabel("--", group),
                "sequence": QLabel("--", group),
                "age": QLabel("--", group),
                "errors": QLabel("0 / 0 / 0", group),
                "action": QLabel("检查 eCAL 初始化、配置路径和 descriptor", group),
            }
            labels["action"].setWordWrap(True)
            fields.addRow("话题", labels["name"])
            fields.addRow("状态", labels["state"])
            fields.addRow("目标频率", labels["target"])
            fields.addRow("实际频率", labels["actual"])
            fields.addRow("最近序列 / 帧年龄", labels["sequence"])
            fields.addRow("错误 / 丢帧 / 缺序", labels["errors"])
            fields.addRow("恢复操作", labels["action"])
            cards_layout.addWidget(group)
            self.topic_cards[contract.topic] = labels
        interface_layout.addWidget(cards)
        viewer_actions = QVBoxLayout()
        self.live_viewer_button = QPushButton("打开实时点云", self.interface_panel)
        self.live_viewer_button.setEnabled(live_viewer_launcher is not None)
        self.live_viewer_close_button = QPushButton("关闭实时点云", self.interface_panel)
        self.live_viewer_close_button.setEnabled(False)
        self.live_viewer_status = QLabel(
            "已配置，点击打开实时点云" if live_viewer_launcher is not None else "未配置/连接未验证",
            self.interface_panel,
        )
        self.live_viewer_status.setWordWrap(True)
        self.offline_viewer_button = QPushButton("Launch Livox Viewer 2", self.interface_panel)
        self.offline_viewer_button.setEnabled(False)
        self.offline_viewer_status = QLabel("未配置/连接未验证", self.interface_panel)
        self.offline_viewer_status.setWordWrap(True)
        self.live_viewer_button.clicked.connect(self._launch_live_viewer)
        self.live_viewer_close_button.clicked.connect(self._close_live_viewer)
        self.offline_viewer_button.clicked.connect(self._launch_offline_viewer)
        viewer_actions.addWidget(self.live_viewer_button)
        viewer_actions.addWidget(self.live_viewer_close_button)
        viewer_actions.addWidget(self.live_viewer_status)
        interface_layout.addLayout(viewer_actions)
        self.offline_evidence_group = QGroupBox("离线采集验收", self.interface_panel)
        offline_evidence_layout = QVBoxLayout(self.offline_evidence_group)
        self.offline_evidence_title = QLabel("离线已验证证据，非实时状态", self.offline_evidence_group)
        offline_evidence_layout.addWidget(self.offline_evidence_title)
        offline_evidence_layout.addWidget(self.offline_viewer_button)
        offline_evidence_layout.addWidget(self.offline_viewer_status)
        evidence_fields = QFormLayout()
        self.offline_evidence_values = {
            name: QLabel("未提供 verifier evidence", self.offline_evidence_group)
            for name in _EVIDENCE_SECTIONS
        }
        for name, label in self.offline_evidence_values.items():
            label.setWordWrap(True)
            evidence_fields.addRow(name, label)
        offline_evidence_layout.addLayout(evidence_fields)
        self.offline_evidence_detail = QPlainTextEdit(self.offline_evidence_group)
        self.offline_evidence_detail.setReadOnly(True)
        self.offline_evidence_detail.setLineWrapMode(QPlainTextEdit.LineWrapMode.WidgetWidth)
        self.offline_evidence_detail.setMinimumHeight(96)
        self.offline_evidence_detail.setMaximumHeight(160)
        self.offline_evidence_detail.setPlainText("未提供 verifier evidence")
        offline_evidence_layout.addWidget(self.offline_evidence_detail)
        self.offline_evidence_group.setVisible(False)
        interface_layout.addWidget(self.offline_evidence_group)
        fields = QFormLayout()
        self.wheel_value = QLabel("--", self.interface_panel)
        self.lidar_value = QLabel("--", self.interface_panel)
        self.rtk_value = QLabel("--", self.interface_panel)
        self.imu_value = QLabel("--", self.interface_panel)
        self.lidar_value.setWordWrap(True)
        self.rtk_value.setWordWrap(True)
        fields.addRow("Wheel state", self.wheel_value)
        fields.addRow("Central LiDAR", self.lidar_value)
        fields.addRow("RTK L/C/R", self.rtk_value)
        fields.addRow("IMU", self.imu_value)
        interface_layout.addLayout(fields)
        self._create_cloud_workbench(lidar_layout, self.lidar_panel)
        self.sampled_preview_group = QGroupBox("轻量空间预览，非验收证据", self.lidar_panel)
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
        lidar_layout.addWidget(self.sampled_preview_group)
        layout.addWidget(self.interface_panel)
        layout.addWidget(self.lidar_panel)

    def take_dashboard_panels(self) -> tuple[QWidget, QWidget]:
        """交给外层 Dashboard 管理状态页与 LiDAR 页，仍共用一个快照控制器。"""
        if self._dashboard_panels_detached:
            raise RuntimeError("v2 dashboard panels are already attached")
        layout = self.layout()
        if layout is None:
            raise RuntimeError("v2 dashboard root layout is unavailable")
        for panel in (self.interface_panel, self.lidar_panel):
            layout.removeWidget(panel)
            panel.setParent(None)
        self._dashboard_panels_detached = True
        return self.interface_panel, self.lidar_panel

    def _create_cloud_workbench(self, layout: QVBoxLayout, parent: QWidget) -> None:
        """创建不依赖 OpenGL 上下文的 QImage 点云工作台。"""
        self.cloud_workbench = QGroupBox("MID-360 实时点云工作台", parent)
        self.cloud_workbench.setCheckable(True)
        self.cloud_workbench.setChecked(False)
        self.cloud_workbench.setMaximumHeight(34)
        workbench_layout = QVBoxLayout(self.cloud_workbench)
        self.cloud_canvas = QLabel(self.cloud_workbench)
        self.cloud_canvas.setMinimumHeight(360)
        self.cloud_canvas.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.cloud_canvas.setStyleSheet("background: #10151c;")
        self.cloud_canvas.setPixmap(QPixmap.fromImage(self._empty_cloud_image()))
        self.cloud_canvas.setVisible(False)
        workbench_layout.addWidget(self.cloud_canvas, stretch=1)

        self.cloud_controls = QWidget(self.cloud_workbench)
        controls = QVBoxLayout(self.cloud_controls)
        controls.setContentsMargins(0, 0, 0, 0)
        self.cloud_window_slider = QSlider(Qt.Orientation.Horizontal, self.cloud_workbench)
        self.cloud_window_slider.setRange(1, 50)
        self.cloud_window_slider.setValue(15)
        self.cloud_point_size_slider = QSlider(Qt.Orientation.Horizontal, self.cloud_workbench)
        self.cloud_point_size_slider.setRange(1, 12)
        self.cloud_point_size_slider.setValue(3)
        self.cloud_color_mode = QComboBox(self.cloud_workbench)
        self.cloud_color_mode.addItems(("语义", "距离", "高度"))
        self.cloud_display_mode = QComboBox(self.cloud_workbench)
        self.cloud_display_mode.addItems(("累计", "当前帧"))
        self.cloud_current_frame_button = QPushButton("查看当前帧", self.cloud_workbench)
        self.cloud_camera_preset = QComboBox(self.cloud_workbench)
        self.cloud_camera_preset.addItems(tuple(_CLOUD_CAMERA_PRESETS))
        self.cloud_reset_view_button = QPushButton("重置视角", self.cloud_workbench)
        self.cloud_fit_view_button = QPushButton("适配点云视角", self.cloud_workbench)
        self.cloud_status = QLabel("等待已验证的 LiDAR/RTK/IMU 同刻数据", self.cloud_workbench)
        self.cloud_status.setWordWrap(True)
        self.cloud_transport_status = QLabel("render_drop=0 | lidar_receiver_errors=0", self.cloud_workbench)
        self.cloud_transport_status.setWordWrap(True)
        self.cloud_command_observer_status = QLabel("wheel_command_observer_errors=0", self.cloud_workbench)
        self.cloud_command_observer_status.setWordWrap(True)
        cloud_controls = QFormLayout()
        cloud_controls.addRow("时间窗", self.cloud_window_slider)
        cloud_controls.addRow("点大小", self.cloud_point_size_slider)
        cloud_controls.addRow("着色", self.cloud_color_mode)
        cloud_controls.addRow("显示", self.cloud_display_mode)
        cloud_controls.addRow("视角", self.cloud_camera_preset)
        controls.addWidget(self.cloud_current_frame_button)
        controls.addLayout(cloud_controls)
        controls.addWidget(self.cloud_reset_view_button)
        controls.addWidget(self.cloud_fit_view_button)
        controls.addWidget(self.cloud_status)
        controls.addWidget(self.cloud_transport_status)
        controls.addWidget(self.cloud_command_observer_status)
        self.cloud_diagnostics = QPlainTextEdit(self.cloud_workbench)
        self.cloud_diagnostics.setReadOnly(True)
        self.cloud_diagnostics.document().setMaximumBlockCount(2_000)
        self.cloud_diagnostics.setPlainText("eCAL 点云工作台已初始化")
        controls.addWidget(self.cloud_diagnostics, stretch=1)
        workbench_layout.addWidget(self.cloud_controls)
        self.cloud_point_size_slider.valueChanged.connect(self._set_cloud_point_size)
        self.cloud_color_mode.currentTextChanged.connect(lambda _value: self._render_cloud())
        self.cloud_display_mode.currentTextChanged.connect(self._change_cloud_display_mode)
        self.cloud_current_frame_button.clicked.connect(self._show_current_cloud_frame)
        self.cloud_camera_preset.currentTextChanged.connect(self._set_cloud_camera_preset)
        self.cloud_reset_view_button.clicked.connect(self._reset_cloud_view)
        self.cloud_fit_view_button.clicked.connect(self._fit_cloud_view)
        self.cloud_workbench.toggled.connect(self._set_cloud_workbench_open)
        layout.addWidget(self.cloud_workbench)

    def _set_cloud_point_size(self, size: int) -> None:
        del size
        self._render_cloud(force=True)

    def _reset_cloud_view(self) -> None:
        self.cloud_camera_preset.setCurrentText("透视")
        self._set_cloud_camera_preset("透视")

    def _set_cloud_camera_preset(self, preset: str) -> None:
        """用固定预设恢复可比较的 CPU 投影视角。"""
        self._cloud_camera = dict(_CLOUD_CAMERA_PRESETS[preset])
        self._cloud_camera["target"] = np.zeros(3, dtype=np.float32)
        self._render_cloud(force=True)

    def _cloud_render_data(self) -> tuple[np.ndarray, np.ndarray]:
        """按显示模式先跨帧等距抽样，再构造有界散点输入。"""
        if self.cloud_display_mode.currentText() == "当前帧":
            frame = self._last_cloud_frame
            if frame is None:
                return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.uint8)
            frames = (frame,)
        else:
            frames = tuple(self._cloud_frames)
        total_points = sum(len(item.positions) for item in frames)
        if total_points == 0:
            return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=np.uint8)
        step = max(
            1,
            (total_points + _MAX_CLOUD_RENDER_POINTS - 1)
            // _MAX_CLOUD_RENDER_POINTS,
        )
        position_chunks: list[np.ndarray] = []
        tag_chunks: list[np.ndarray] = []
        offset = 0
        for item in frames:
            first = (-offset) % step
            if first < len(item.positions):
                position_chunks.append(item.positions[first::step])
                tag_chunks.append(item.tags[first::step])
            offset += len(item.positions)
        return (
            np.concatenate(position_chunks, axis=0),
            np.concatenate(tag_chunks, axis=0),
        )

    @staticmethod
    def _empty_cloud_image() -> QImage:
        image = QImage(*_CLOUD_IMAGE_SIZE, QImage.Format.Format_RGBA8888)
        image.fill(QColor("#10151c"))
        painter = QPainter(image)
        painter.setPen(QPen(QColor("#33404d"), 1))
        width, height = image.width(), image.height()
        for offset in range(0, width + 1, 48):
            painter.drawLine(offset, 0, offset, height)
        for offset in range(0, height + 1, 48):
            painter.drawLine(0, offset, width, offset)
        painter.setPen(QPen(QColor("#697888"), 1))
        painter.drawLine(width // 2, 0, width // 2, height)
        painter.drawLine(0, height // 2, width, height // 2)
        painter.end()
        return image

    @staticmethod
    def _project_cloud_points(
        positions: np.ndarray, camera: dict[str, object] | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """将有界 world 点投影到图像；不触碰任何 GL/驱动上下文。"""
        if not len(positions):
            return np.empty((0, 2), dtype=np.int32), np.empty(0, dtype=bool)
        if camera is None:
            camera = {**_CLOUD_CAMERA_PRESETS["透视"], "target": np.zeros(3, dtype=np.float32)}
        target = np.asarray(camera["target"], dtype=np.float32)
        azimuth = np.deg2rad(float(camera["azimuth"]))
        elevation = np.deg2rad(float(camera["elevation"]))
        relative = positions - target
        cos_azimuth, sin_azimuth = np.cos(azimuth), np.sin(azimuth)
        x = cos_azimuth * relative[:, 0] - sin_azimuth * relative[:, 1]
        depth = sin_azimuth * relative[:, 0] + cos_azimuth * relative[:, 1]
        cos_elevation, sin_elevation = np.cos(elevation), np.sin(elevation)
        vertical = cos_elevation * relative[:, 2] - sin_elevation * depth
        depth = sin_elevation * relative[:, 2] + cos_elevation * depth
        distance = max(1.0, float(camera["distance"]))
        scale = 0.46 * min(_CLOUD_IMAGE_SIZE) / np.maximum(distance + depth, distance * 0.15)
        points = np.column_stack((
            _CLOUD_IMAGE_SIZE[0] / 2.0 + x * scale,
            _CLOUD_IMAGE_SIZE[1] / 2.0 - vertical * scale,
        )).astype(np.int32)
        visible = (
            (distance + depth > distance * 0.05)
            & (points[:, 0] >= 0) & (points[:, 0] < _CLOUD_IMAGE_SIZE[0])
            & (points[:, 1] >= 0) & (points[:, 1] < _CLOUD_IMAGE_SIZE[1])
        )
        return points, visible

    def _render_cloud(self, *, force: bool = False) -> None:
        """把有界绘制输入交给唯一后台任务；GUI 线程只提交完成图像。"""
        if self._cloud_render_failed or not self.cloud_workbench.isChecked():
            return
        now = time.monotonic()
        if not force and now < self._cloud_next_render_at:
            self._cloud_render_dropped_count += 1
            return
        if self._cloud_render_inflight is not None:
            self._cloud_render_dropped_count += 1
            self._cloud_render_pending = True
            return
        self._cloud_next_render_at = now + 1.0 / _CLOUD_RENDER_MAX_HZ
        positions, tags = self._cloud_render_data()
        frame = self._last_cloud_frame
        camera = {
            key: (np.asarray(value, dtype=np.float32).copy() if key == "target" else value)
            for key, value in self._cloud_camera.items()
        }
        self._cloud_render_generation += 1
        self._cloud_render_inflight = self._cloud_render_executor.submit(
            self._build_cloud_image,
            self._cloud_render_generation,
            positions,
            tags,
            frame,
            camera,
            self.cloud_color_mode.currentText(),
            self.cloud_point_size_slider.value(),
        )
        self._cloud_render_timer.start()

    def _collect_cloud_render(self) -> None:
        """在 GUI 线程提交后台完成的 QImage，并对首次异常熔断。"""
        future = self._cloud_render_inflight
        if future is None or not future.done():
            return
        self._cloud_render_inflight = None
        self._cloud_render_timer.stop()
        render_pending = self._cloud_render_pending
        self._cloud_render_pending = False
        try:
            generation, image, duration_ms = future.result()
            if generation == self._cloud_render_generation and self.cloud_workbench.isChecked():
                self.cloud_canvas.setPixmap(QPixmap.fromImage(image))
                self._cloud_last_render_duration_ms = duration_ms
                self._update_cloud_status()
        except Exception as error:
            self._cloud_render_failed = True
            self._cloud_render_failure = f"{type(error).__name__}: {error}"
            self.cloud_status.setText(
                f"点云显示已暂停：{self._cloud_render_failure}。折叠后重新展开可恢复。"
            )
            self.cloud_diagnostics.appendPlainText(
                f"renderer circuit-breaker: {self._cloud_render_failure}"
            )
            return
        if render_pending:
            self._render_cloud(force=True)

    @classmethod
    def _build_cloud_image(
        cls,
        generation: int,
        positions: np.ndarray,
        tags: np.ndarray,
        frame: object | None,
        camera: dict[str, object],
        color_mode: str,
        point_size: int,
    ) -> tuple[int, QImage, float]:
        """后台仅处理不可变 NumPy 帧并返回 QImage，不读取 QWidget 状态。"""
        started_at = time.monotonic()
        image = cls._empty_cloud_image()
        projected, visible = cls._project_cloud_points(positions, camera)
        colors = (cls._cloud_colors_for_mode(positions, tags, color_mode)[:, :3] * 255).astype(np.uint8)
        colors = (colors // 32 * 32).astype(np.uint8)
        visible_points = projected[visible]
        visible_colors = colors[visible]
        if len(visible_points):
            # RGBA8888 与 NumPy byte view 同序，批量写像素避开 80k 次 Qt 调用。
            pixels = np.frombuffer(image.bits(), dtype=np.uint8).reshape(
                image.height(), image.bytesPerLine(),
            )
            radius = max(0, point_size - 1) // 2
            for offset_y in range(-radius, radius + 1):
                y = np.clip(visible_points[:, 1] + offset_y, 0, image.height() - 1)
                for offset_x in range(-radius, radius + 1):
                    x = np.clip(visible_points[:, 0] + offset_x, 0, image.width() - 1)
                    pixels[y, x * 4] = visible_colors[:, 0]
                    pixels[y, x * 4 + 1] = visible_colors[:, 1]
                    pixels[y, x * 4 + 2] = visible_colors[:, 2]
                    pixels[y, x * 4 + 3] = 255
        painter = QPainter(image)
        if frame is not None:
            marker, marker_visible = cls._project_cloud_points(np.asarray((
                frame.vehicle_position,
                np.asarray(frame.vehicle_position) + np.asarray(frame.vehicle_forward) * 1.5,
            ), dtype=np.float32), camera)
            if bool(marker_visible.all()):
                painter.setPen(QPen(QColor("#f4f4f4"), 3))
                painter.drawLine(int(marker[0, 0]), int(marker[0, 1]),
                                 int(marker[1, 0]), int(marker[1, 1]))
        painter.end()
        return generation, image, (time.monotonic() - started_at) * 1_000.0

    def _show_current_cloud_frame(self) -> None:
        """从窄侧栏的一键入口切到持续更新的当前帧视图。"""
        already_current = self.cloud_display_mode.currentText() == "当前帧"
        self.cloud_display_mode.setCurrentText("当前帧")
        if already_current:
            self._change_cloud_display_mode("当前帧")

    def _change_cloud_display_mode(self, _mode: str) -> None:
        """显示模式切换后在同一 GUI 事件内同步点云与状态文字。"""
        self._render_cloud()
        self._update_cloud_status()

    def _update_cloud_status(self) -> None:
        """按当前模式显示最新帧与实际进入显示窗口的点数。"""
        frame = self._last_cloud_frame
        if frame is None:
            self.cloud_status.setText("等待已验证的 LiDAR/RTK/IMU 同刻数据")
            return
        window_ns = self.cloud_window_slider.value() * 100_000_000
        display_count = (
            len(frame.positions)
            if self.cloud_display_mode.currentText() == "当前帧"
            else sum(len(item.positions) for item in self._cloud_frames)
        )
        render_step = max(
            1,
            (display_count + _MAX_CLOUD_RENDER_POINTS - 1)
            // _MAX_CLOUD_RENDER_POINTS,
        )
        rendered_count = (display_count + render_step - 1) // render_step
        self.cloud_status.setText(
            "%s | seq=%d | 本帧=%d | 时间窗=%0.1fs | 显示=%d | render=%0.1fms | drop=%d"
            % (
                self.cloud_display_mode.currentText(),
                frame.sequence,
                len(frame.positions),
                window_ns / 1_000_000_000,
                rendered_count,
                self._cloud_last_render_duration_ms,
                self._cloud_render_dropped_count,
            )
        )

    def _fit_cloud_view(self) -> None:
        """使当前显示点范围居中，空帧时回到可重现的默认透视。"""
        positions, _tags = self._cloud_render_data()
        if not len(positions):
            self._reset_cloud_view()
            return
        lower, upper = positions.min(axis=0), positions.max(axis=0)
        distance = max(6.0, float(np.linalg.norm(upper - lower)) * 2.2)
        self._cloud_camera["target"] = positions.mean(axis=0)
        self._cloud_camera["distance"] = distance
        self._render_cloud(force=True)

    def _set_cloud_workbench_open(self, opened: bool) -> None:
        """默认折叠工作台，避免不查看 3D 时挤压采集与遥测布局。"""
        self.cloud_workbench.setMaximumHeight(900 if opened else 34)
        self.cloud_canvas.setVisible(opened)
        if opened:
            self._cloud_render_failed = False
            self._cloud_render_failure = ""
            self._render_cloud(force=True)

    def update_cloud_frame(self, frame: object) -> bool:
        """在 GUI 线程累积 worker 已转换的 world 点，绝不接收 raw eCAL payload。"""
        from slope_sim.interfaces.v2.dashboard_receiver import V2DashboardCloudFrame

        if type(frame) is not V2DashboardCloudFrame:
            raise ValueError("cloud frame must be an exact V2DashboardCloudFrame")
        if frame.sequence == self._last_cloud_sequence:
            return False
        self._last_cloud_sequence = frame.sequence
        self._cloud_frames.append(frame)
        self._last_cloud_frame = frame
        window_ns = self.cloud_window_slider.value() * 100_000_000
        cutoff = frame.timestamp_ns - window_ns
        while self._cloud_frames and self._cloud_frames[0].timestamp_ns < cutoff:
            self._cloud_frames.popleft()
        while len(self._cloud_frames) > _MAX_CLOUD_HISTORY_FRAMES:
            self._cloud_frames.popleft()
        if self.cloud_workbench.isChecked():
            self._render_cloud()
        self._update_cloud_status()
        return True

    def update_receiver_diagnostics(
        self,
        *,
        render_dropped_count: int,
        diagnostics: tuple[str, ...],
    ) -> bool:
        """把 receiver 的有界拒绝记录和显示队列丢帧投影到 GUI 终端。"""
        if type(render_dropped_count) is not int or render_dropped_count < 0:
            raise ValueError("render_dropped_count must be a nonnegative int")
        if (
            not isinstance(diagnostics, tuple)
            or any(not isinstance(item, str) or not item for item in diagnostics)
        ):
            raise ValueError("diagnostics must be a tuple of nonempty strings")
        if (
            render_dropped_count == self._last_render_dropped_count
            and diagnostics == self._last_receiver_diagnostics
        ):
            return False
        lidar_diagnostics = tuple(
            detail for detail in diagnostics if detail.startswith("/sim/lidar/points:")
        )
        command_diagnostics = tuple(
            detail for detail in diagnostics if detail.startswith("/sim/wheel/command:")
        )
        self.cloud_transport_status.setText(
            "render_drop=%d | lidar_receiver_errors=%d"
            % (render_dropped_count, len(lidar_diagnostics))
        )
        self.cloud_command_observer_status.setText(
            "wheel_command_observer_errors=%d" % len(command_diagnostics)
        )
        previous = self._last_receiver_diagnostics
        new_lines = diagnostics[len(previous):] if diagnostics[:len(previous)] == previous else diagnostics
        if render_dropped_count != self._last_render_dropped_count:
            self.cloud_diagnostics.appendPlainText(
                "receiver render_drop=%d" % render_dropped_count
            )
        for detail in new_lines:
            self.cloud_diagnostics.appendPlainText("receiver: %s" % detail)
        self._last_render_dropped_count = render_dropped_count
        self._last_receiver_diagnostics = diagnostics
        return True

    def _cloud_colors(self, positions: np.ndarray, tags: np.ndarray) -> np.ndarray:
        return self._cloud_colors_for_mode(positions, tags, self.cloud_color_mode.currentText())

    @staticmethod
    def _cloud_colors_for_mode(positions: np.ndarray, tags: np.ndarray, mode: str) -> np.ndarray:
        if mode == "语义":
            palette = np.asarray(((0.35, 0.72, 0.30, 0.85), (0.85, 0.76, 0.22, 0.95),
                                  (0.90, 0.26, 0.18, 0.95), (0.60, 0.28, 0.78, 0.95)), dtype=np.float32)
            return palette[np.minimum(tags, len(palette) - 1)]
        if mode == "距离":
            distance = np.linalg.norm(positions, axis=1)
            scale = np.clip(distance / 30.0, 0.0, 1.0)
            return np.column_stack((scale, 0.85 - 0.55 * scale, 1.0 - scale, np.full(len(scale), 0.9))).astype(np.float32)
        height = positions[:, 2] if len(positions) else np.empty(0, dtype=np.float32)
        lower, upper = (float(height.min()), float(height.max())) if len(height) else (0.0, 1.0)
        scale = np.zeros_like(height) if upper <= lower else (height - lower) / (upper - lower)
        return np.column_stack((scale, 0.25 + 0.65 * scale, 1.0 - scale, np.full(len(scale), 0.9))).astype(np.float32)

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

    def shutdown(self) -> None:
        """主动回收独立显示链，不依赖嵌入 QWidget 是否接收 close event。"""
        self._close_live_viewer()
        self._cloud_render_timer.stop()
        self._cloud_render_executor.shutdown(wait=False, cancel_futures=True)

    def closeEvent(self, event: object) -> None:  # noqa: N802 - Qt 固定回调名
        """父 Dashboard 释放嵌入 widget 时，不能遗留独立 Bridge/RViz2 进程组。"""
        self.shutdown()
        super().closeEvent(event)  # type: ignore[arg-type]

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
            snapshot.authority_rejections,
            snapshot.observer_rejections,
        )

    @staticmethod
    def _rejection_text(domain: str, rejections: tuple[object, ...]) -> str:
        """格式化有界拒绝诊断；仅展示快照中的标量，不读取任何 raw 数据。"""
        if not rejections:
            return f"{domain}: 累计 0"
        latest = rejections[-1]
        source = "--" if latest.source_id is None else latest.source_id
        sequence = "--" if latest.sequence is None else str(latest.sequence)
        session = (
            "--" if latest.simulation_session_id is None
            else latest.simulation_session_id.hex()[:12]
        )
        world = "--" if latest.world_generation is None else str(latest.world_generation)
        return (
            f"{domain}: 累计 {len(rejections)} | source={source} | seq={sequence} | "
            f"session={session} | world={world} | {latest.reason}"
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
        self.offline_evidence_group.setVisible(True)
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
        self.authority_rejection_value.setText(
            self._rejection_text("simulator authority", snapshot.authority_rejections)
        )
        self.observer_rejection_value.setText(
            self._rejection_text("dashboard observer", snapshot.observer_rejections)
        )
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
            target_text = str(contract.rate_hz)
            actual_text = "--" if observation is None or observation.actual_hz is None else f"{observation.actual_hz:.1f}"
            state_text = "-- / not_checked" if observation is None else "%s / %s" % (
                "--" if observation.peer_count is None else observation.peer_count,
                observation.protocol_state,
            )
            errors_text = "0" if observation is None else str(observation.error_count)
            drops_text = "0" if observation is None else str(observation.dropped_count)
            gaps_text = "0" if observation is None else str(observation.sequence_gap_count)
            sequence_text = "--" if latest_sequence is None else str(latest_sequence)
            age_text = "--" if age is None else f"{max(age, 0.0):.2f}s"
            points_text = "--" if point_count is None else str(point_count)
            action_text = self._topic_action(observation, contract.topic)
            card = self.topic_cards[contract.topic]
            card["state"].setText(state_text)
            card["target"].setText(f"{target_text} Hz")
            card["actual"].setText("--" if actual_text == "--" else f"{actual_text} Hz")
            card["sequence"].setText(f"{sequence_text} / {age_text}")
            card["errors"].setText(f"{errors_text} / {drops_text} / {gaps_text}")
            card["action"].setText(action_text)
            cells = (
                contract.topic,
                target_text,
                actual_text,
                "--/not_checked" if observation is None else state_text.replace(" / ", "/"),
                errors_text,
                drops_text,
                gaps_text,
                sequence_text,
                age_text,
                points_text,
                action_text,
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
