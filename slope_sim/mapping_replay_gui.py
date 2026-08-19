"""MID-360 Golf 录制的同步双三维回放窗口。"""
from __future__ import annotations

from dataclasses import dataclass
from queue import Empty, Full, Queue
from threading import Event, Thread
import time
from typing import Iterator, Protocol

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PySide6 import QtCore, QtGui, QtWidgets

from slope_sim.interfaces.v2.models import LidarPointCloudV2
from slope_sim.mapping_replay import (
    MissingPoseLookaheadError,
    PlaybackClock,
    PlaybackFrameRequest,
    RecoveredPoseNode,
    WorldMapAccumulator,
    deskew_lidar_frame,
)


_PLAYBACK_LABELS = ("0.25x", "0.5x", "1x", "2x", "4x")
_PLAYBACK_RATES = (0.25, 0.5, 1.0, 2.0, 4.0)
_GOLF_CENTER = (0.0, 0.0, 0.4)
_GOLF_MAP_MINIMUM = (-10.01, -6.65, -2.0)
_GOLF_MAP_MAXIMUM = (10.01, 6.65, 5.0)
_GOLF_DISTANCE_M = 26.0


def _readonly_positions(name: str, values: object) -> np.ndarray:
    source = np.asarray(values)
    if source.size == 0:
        array = np.empty((0, 3), dtype=np.float32)
    else:
        if source.ndim != 2 or source.shape[1] != 3:
            raise ValueError(f"{name} must have shape (N, 3)")
        array = np.array(source, dtype=np.float32, order="C", copy=True)
        if not np.isfinite(array).all():
            raise ValueError(f"{name} must contain only finite coordinates")
    array.setflags(write=False)
    return array


def _readonly_tags(name: str, values: object, *, length: int) -> np.ndarray:
    source = np.asarray(values)
    if source.size == 0:
        array = np.empty((0,), dtype=np.uint8)
    else:
        if source.ndim != 1 or source.shape[0] != length:
            raise ValueError(f"{name} must have shape (N,)")
        if not np.issubdtype(source.dtype, np.integer):
            raise ValueError(f"{name} must contain integer tags")
        if np.any(source < 0) or np.any(source > 255):
            raise ValueError(f"{name} tags must fit uint8")
        array = np.array(source, dtype=np.uint8, order="C", copy=True)
    if array.shape[0] != length:
        raise ValueError(f"{name} length must match its point array")
    array.setflags(write=False)
    return array


@dataclass(frozen=True, slots=True)
class ReplayRenderFrame:
    """后台交给 Qt 线程的不可变 GPU 数据快照。"""

    generation: int
    frame_index: int
    timebase_ns: int
    raw_positions: np.ndarray
    raw_tags: np.ndarray
    permanent_positions: np.ndarray
    permanent_tags: np.ndarray
    moving_positions: np.ndarray
    moving_tags: np.ndarray
    trajectory_positions: np.ndarray
    map_available: bool
    notice: str

    def __post_init__(self) -> None:
        for name in ("generation", "frame_index", "timebase_ns"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        if type(self.map_available) is not bool or type(self.notice) is not str:
            raise ValueError("map_available/notice have invalid types")

        raw_positions = _readonly_positions("raw_positions", self.raw_positions)
        permanent_positions = _readonly_positions(
            "permanent_positions", self.permanent_positions
        )
        moving_positions = _readonly_positions(
            "moving_positions", self.moving_positions
        )
        object.__setattr__(self, "raw_positions", raw_positions)
        object.__setattr__(
            self,
            "raw_tags",
            _readonly_tags("raw_tags", self.raw_tags, length=len(raw_positions)),
        )
        object.__setattr__(self, "permanent_positions", permanent_positions)
        object.__setattr__(
            self,
            "permanent_tags",
            _readonly_tags(
                "permanent_tags",
                self.permanent_tags,
                length=len(permanent_positions),
            ),
        )
        object.__setattr__(self, "moving_positions", moving_positions)
        object.__setattr__(
            self,
            "moving_tags",
            _readonly_tags(
                "moving_tags", self.moving_tags, length=len(moving_positions)
            ),
        )
        object.__setattr__(
            self,
            "trajectory_positions",
            _readonly_positions("trajectory_positions", self.trajectory_positions),
        )


class ReplayFrameSource(Protocol):
    """经严格 MCAP 校验后供窗口逐帧重建的数据源。"""

    frame_times_ns: tuple[int, ...]

    def render_frame(
        self,
        request: PlaybackFrameRequest,
        cancellation: Event,
    ) -> ReplayRenderFrame: ...


class _MappingSessionIndex(Protocol):
    lidar_frame_times_ns: tuple[int, ...]
    pose_nodes: tuple[RecoveredPoseNode, ...]

    def iter_lidar_frames(self) -> Iterator[LidarPointCloudV2]: ...


class MappingMcapReplaySource:
    """在单个后台线程内流式读取 LiDAR 并确定性重建 Golf 地图。"""

    def __init__(self, index: _MappingSessionIndex) -> None:
        frame_times = tuple(index.lidar_frame_times_ns)
        pose_nodes = tuple(index.pose_nodes)
        if not frame_times:
            raise ValueError("mapping session must contain LiDAR frames")
        self.frame_times_ns = frame_times
        self._index = index
        self._pose_by_time = {node.timestamp_ns: node for node in pose_nodes}
        if len(self._pose_by_time) != len(pose_nodes):
            raise ValueError("mapping pose timestamps must be unique")
        self._map = WorldMapAccumulator(
            minimum=_GOLF_MAP_MINIMUM,
            maximum=_GOLF_MAP_MAXIMUM,
        )
        self._trajectory: list[tuple[float, float, float]] = []
        self._frames: Iterator[LidarPointCloudV2] | None = None
        self._current_index = -1
        self._current_cloud: LidarPointCloudV2 | None = None
        self._current_map_available = False
        self._current_notice = ""

    def _reset(self) -> None:
        if self._frames is not None:
            close = getattr(self._frames, "close", None)
            if callable(close):
                close()
        self._frames = iter(self._index.iter_lidar_frames())
        self._map.clear()
        self._trajectory.clear()
        self._current_index = -1
        self._current_cloud = None
        self._current_map_available = False
        self._current_notice = ""

    def _consume_next(self, cancellation: Event) -> None:
        if cancellation.is_set():
            raise RuntimeError("replay frame rendering was cancelled")
        if self._frames is None:
            raise RuntimeError("LiDAR stream is not open")
        next_index = self._current_index + 1
        try:
            cloud = next(self._frames)
        except StopIteration as error:
            raise ValueError("LiDAR stream ended before the indexed frame") from error
        if (
            next_index >= len(self.frame_times_ns)
            or cloud.timebase_ns != self.frame_times_ns[next_index]
        ):
            raise ValueError("LiDAR stream order differs from the strict session index")
        start = self._pose_by_time.get(cloud.timebase_ns)
        lookahead = self._pose_by_time.get(cloud.timebase_ns + 100_000_000)
        if start is None:
            raise ValueError("LiDAR frame is missing its same-time pose node")
        self._trajectory.append(start.base_pose.position)
        try:
            if lookahead is None:
                raise MissingPoseLookaheadError(
                    "LiDAR frame is missing its next 100 ms pose node"
                )
            deskewed = deskew_lidar_frame(cloud, start, lookahead)
            if cancellation.is_set():
                raise RuntimeError("replay frame rendering was cancelled")
            self._map.add_frame(deskewed, frame_time_ns=cloud.timebase_ns)
            self._current_map_available = True
            self._current_notice = ""
        except MissingPoseLookaheadError as error:
            self._current_map_available = False
            self._current_notice = str(error)
        if next_index == len(self.frame_times_ns) - 1:
            try:
                next(self._frames)
            except StopIteration:
                self._frames = None
            else:
                raise ValueError("LiDAR stream contains frames beyond its strict index")
        self._current_index = next_index
        self._current_cloud = cloud

    def render_frame(
        self,
        request: PlaybackFrameRequest,
        cancellation: Event,
    ) -> ReplayRenderFrame:
        if type(request) is not PlaybackFrameRequest:
            raise ValueError("request must be an exact PlaybackFrameRequest")
        target = request.frame_index
        if not 0 <= target < len(self.frame_times_ns):
            raise ValueError("requested frame is outside the mapping session")
        must_rebuild = (
            request.rebuild_from_start
            or self._frames is None
            or target <= self._current_index
        )
        if must_rebuild:
            self._reset()
        elif target != self._current_index + 1:
            raise ValueError("forward playback must advance exactly one LiDAR frame")
        while self._current_index < target:
            self._consume_next(cancellation)

        cloud = self._current_cloud
        if cloud is None:
            raise RuntimeError("requested LiDAR frame was not decoded")
        snapshot = self._map.snapshot(frame_time_ns=cloud.timebase_ns)
        return ReplayRenderFrame(
            generation=request.generation,
            frame_index=target,
            timebase_ns=cloud.timebase_ns,
            raw_positions=tuple((point.x, point.y, point.z) for point in cloud.points),
            raw_tags=tuple(point.tag for point in cloud.points),
            permanent_positions=snapshot.permanent_positions,
            permanent_tags=snapshot.permanent_tags,
            moving_positions=snapshot.moving_positions,
            moving_tags=snapshot.moving_tags,
            trajectory_positions=tuple(self._trajectory),
            map_available=self._current_map_available,
            notice=self._current_notice,
        )

    def close(self) -> None:
        if self._frames is not None:
            close = getattr(self._frames, "close", None)
            if callable(close):
                close()
            self._frames = None


@dataclass(frozen=True, slots=True)
class _WorkerResult:
    request: PlaybackFrameRequest
    frame: ReplayRenderFrame | None
    error: BaseException | None


class ReplayFrameWorker:
    """串行执行 MCAP 解码和地图累计的容量 1 后台 worker。"""

    def __init__(self, source: ReplayFrameSource) -> None:
        self._source = source
        self._requests: Queue[PlaybackFrameRequest] = Queue(maxsize=1)
        self._results: Queue[_WorkerResult] = Queue(maxsize=1)
        self._cancellation = Event()
        self._thread = Thread(
            target=self._run,
            name="mid360-mapping-replay",
            daemon=False,
        )
        self._thread.start()

    @property
    def request_capacity(self) -> int:
        return self._requests.maxsize

    @property
    def result_capacity(self) -> int:
        return self._results.maxsize

    def submit(self, request: PlaybackFrameRequest) -> bool:
        if self._cancellation.is_set() or type(request) is not PlaybackFrameRequest:
            return False
        try:
            self._requests.put_nowait(request)
        except Full:
            return False
        return True

    def poll(self) -> _WorkerResult | None:
        try:
            return self._results.get_nowait()
        except Empty:
            return None

    def _deliver(self, result: _WorkerResult) -> None:
        while not self._cancellation.is_set():
            try:
                self._results.put(result, timeout=0.05)
                return
            except Full:
                continue

    def _run(self) -> None:
        while not self._cancellation.is_set():
            try:
                request = self._requests.get(timeout=0.05)
            except Empty:
                continue
            try:
                frame = self._source.render_frame(request, self._cancellation)
                if type(frame) is not ReplayRenderFrame:
                    raise TypeError("render_frame must return ReplayRenderFrame")
                if (
                    frame.generation != request.generation
                    or frame.frame_index != request.frame_index
                ):
                    raise ValueError("rendered frame does not match its request")
                result = _WorkerResult(request, frame, None)
            except BaseException as error:
                result = _WorkerResult(request, None, error)
            self._deliver(result)

    def close(self) -> None:
        self._cancellation.set()
        self._thread.join()
        close = getattr(self._source, "close", None)
        if callable(close):
            close()


def _tag_colors(tags: np.ndarray, *, raw: bool = False) -> np.ndarray:
    colors = np.empty((len(tags), 4), dtype=np.float32)
    colors[:] = (0.68, 0.72, 0.78, 0.85)
    colors[tags == 1] = (0.20, 0.78, 0.46, 0.90)
    colors[tags == 2] = (0.98, 0.72, 0.22, 0.95)
    colors[tags == 3] = (0.94, 0.28, 0.25, 1.00)
    if raw:
        colors[:, 3] = 0.88
    return colors


class MappingReplayWindow(QtWidgets.QMainWindow):
    """固定全场构图、按逻辑帧同步推进的双 OpenGL 回放窗口。"""

    def __init__(self, source: ReplayFrameSource) -> None:
        super().__init__()
        frame_times = tuple(source.frame_times_ns)
        self.clock = PlaybackClock(frame_times)
        self.worker = ReplayFrameWorker(source)
        self.displayed_frame_index = -1
        self._initial_request = PlaybackFrameRequest(0, 0, True)
        self._initial_pending = True
        self._updating_timeline = False
        self._next_playback_deadline = time.monotonic()
        self._closed = False

        self.setWindowTitle("MID-360 Golf Mapping Replay")
        self.resize(1280, 760)
        self._build_ui(len(frame_times))
        self._reset_raw_camera()
        self._reset_world_camera()

        self._result_timer = QtCore.QTimer(self)
        self._result_timer.setInterval(5)
        self._result_timer.timeout.connect(self._poll_worker)
        self._result_timer.start()
        self._playback_timer = QtCore.QTimer(self)
        self._playback_timer.setInterval(10)
        self._playback_timer.timeout.connect(self._advance_playback)
        self._playback_timer.start()

        if not self.worker.submit(self._initial_request):
            raise RuntimeError("initial replay frame could not be queued")

    def _build_ui(self, frame_count: int) -> None:
        central = QtWidgets.QWidget(self)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self.splitter = QtWidgets.QSplitter(QtCore.Qt.Orientation.Horizontal)
        self.raw_view = gl.GLViewWidget()
        self.world_view = gl.GLViewWidget()
        self.raw_view.setBackgroundColor((18, 22, 27))
        self.world_view.setBackgroundColor((18, 22, 27))
        self.splitter.addWidget(self._view_panel(self.raw_view, world=False))
        self.splitter.addWidget(self._view_panel(self.world_view, world=True))
        self.splitter.setStretchFactor(0, 43)
        self.splitter.setStretchFactor(1, 57)
        self.splitter.setSizes((430, 570))
        layout.addWidget(self.splitter, 1)

        controls = QtWidgets.QHBoxLayout()
        controls.setSpacing(6)
        self.restart_button = self._icon_button(
            QtWidgets.QStyle.StandardPixmap.SP_MediaSkipBackward,
            "回到开头",
            self._restart,
        )
        self.previous_button = self._icon_button(
            QtWidgets.QStyle.StandardPixmap.SP_MediaSeekBackward,
            "上一帧",
            lambda: self._step(-1),
        )
        self.play_button = self._icon_button(
            QtWidgets.QStyle.StandardPixmap.SP_MediaPlay,
            "播放",
            self._toggle_playback,
        )
        self.next_button = self._icon_button(
            QtWidgets.QStyle.StandardPixmap.SP_MediaSeekForward,
            "下一帧",
            lambda: self._step(1),
        )
        for button in (
            self.restart_button,
            self.previous_button,
            self.play_button,
            self.next_button,
        ):
            controls.addWidget(button)

        self.timeline = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.timeline.setRange(0, frame_count - 1)
        self.timeline.setSingleStep(1)
        self.timeline.setPageStep(1)
        self.timeline.sliderReleased.connect(self._seek_from_timeline)
        controls.addWidget(self.timeline, 1)

        self.frame_label = QtWidgets.QLabel(f"0 / {frame_count - 1}")
        self.frame_label.setMinimumWidth(92)
        controls.addWidget(self.frame_label)
        self.speed_control = QtWidgets.QComboBox()
        self.speed_control.addItems(_PLAYBACK_LABELS)
        self.speed_control.setCurrentIndex(2)
        self.speed_control.currentIndexChanged.connect(self._set_rate)
        controls.addWidget(self.speed_control)
        layout.addLayout(controls)

        self.notice_label = QtWidgets.QLabel("")
        self.notice_label.setWordWrap(True)
        self.notice_label.setVisible(False)
        layout.addWidget(self.notice_label)
        self.setCentralWidget(central)

    def _view_panel(self, view: gl.GLViewWidget, *, world: bool) -> QtWidgets.QWidget:
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        toolbar = QtWidgets.QHBoxLayout()
        toolbar.addStretch(1)
        if world:
            toolbar.addWidget(
                self._icon_button(
                    QtWidgets.QStyle.StandardPixmap.SP_ArrowUp,
                    "俯视",
                    self._set_top_view,
                )
            )
            toolbar.addWidget(
                self._icon_button(
                    QtWidgets.QStyle.StandardPixmap.SP_ArrowForward,
                    "三维透视",
                    self._reset_world_camera,
                )
            )
        toolbar.addWidget(
            self._icon_button(
                QtWidgets.QStyle.StandardPixmap.SP_BrowserReload,
                "复位视角",
                self._reset_world_camera if world else self._reset_raw_camera,
            )
        )
        point_size = QtWidgets.QDoubleSpinBox()
        point_size.setRange(1.0, 8.0)
        point_size.setSingleStep(0.5)
        point_size.setValue(2.5)
        point_size.setSuffix(" px")
        point_size.setToolTip("点大小")
        point_size.valueChanged.connect(self._set_point_size)
        toolbar.addWidget(point_size)
        layout.addLayout(toolbar)
        layout.addWidget(view, 1)
        # Qt offscreen 插件不支持可见 QOpenGLWidget；结构测试不创建原生 GL 上下文。
        application = QtGui.QGuiApplication.instance()
        if application is not None and application.platformName() == "offscreen":
            view.hide()

        if world:
            grid = gl.GLGridItem()
            grid.setSize(
                _GOLF_MAP_MAXIMUM[0] - _GOLF_MAP_MINIMUM[0],
                _GOLF_MAP_MAXIMUM[1] - _GOLF_MAP_MINIMUM[1],
                1.0,
            )
            grid.setSpacing(1.0, 1.0, 1.0)
            grid.setColor((76, 92, 82, 90))
            self.world_grid_item = grid
            view.addItem(grid)
            self.terrain_item = gl.GLScatterPlotItem(
                pos=np.empty((0, 3), dtype=np.float32), size=2.0, pxMode=True
            )
            self.static_item = gl.GLScatterPlotItem(
                pos=np.empty((0, 3), dtype=np.float32), size=2.5, pxMode=True
            )
            self.moving_item = gl.GLScatterPlotItem(
                pos=np.empty((0, 3), dtype=np.float32), size=4.0, pxMode=True
            )
            self.trajectory_item = gl.GLLinePlotItem(
                pos=np.empty((0, 3), dtype=np.float32),
                color=(0.18, 0.72, 1.0, 0.95),
                width=2.0,
                antialias=True,
                mode="line_strip",
            )
            for item in (
                self.terrain_item,
                self.static_item,
                self.moving_item,
                self.trajectory_item,
            ):
                view.addItem(item)
        else:
            axes = gl.GLAxisItem()
            axes.setSize(4.0, 4.0, 4.0)
            view.addItem(axes)
            self.raw_item = gl.GLScatterPlotItem(
                pos=np.empty((0, 3), dtype=np.float32), size=2.5, pxMode=True
            )
            view.addItem(self.raw_item)
        return panel

    def _icon_button(self, icon, tooltip: str, callback) -> QtWidgets.QToolButton:
        button = QtWidgets.QToolButton(self)
        button.setIcon(self.style().standardIcon(icon))
        button.setToolTip(tooltip)
        button.setAutoRaise(True)
        button.setFixedSize(30, 30)
        button.clicked.connect(callback)
        return button

    def _reset_raw_camera(self) -> None:
        self.raw_view.setCameraPosition(
            # 离线 MID-360 已扩展到 60 m，留出 20% 视野避免边缘点被默认相机裁掉。
            pos=pg.Vector(0.0, 0.0, 0.0), distance=72.0, elevation=22.0, azimuth=45.0
        )

    def _reset_world_camera(self) -> None:
        self.world_view.setCameraPosition(
            pos=pg.Vector(*_GOLF_CENTER),
            distance=_GOLF_DISTANCE_M,
            elevation=34.0,
            azimuth=-35.0,
        )

    def _set_top_view(self) -> None:
        self.world_view.setCameraPosition(
            pos=pg.Vector(*_GOLF_CENTER),
            distance=_GOLF_DISTANCE_M,
            elevation=89.9,
            azimuth=0.0,
        )

    def _set_point_size(self, value: float) -> None:
        self.raw_item.setData(size=value)
        self.terrain_item.setData(size=max(1.0, value - 0.5))
        self.static_item.setData(size=value)
        self.moving_item.setData(size=min(8.0, value + 1.5))

    def submit_request(self, request: PlaybackFrameRequest) -> bool:
        return self.worker.submit(request)

    def _restart(self) -> None:
        self._seek(0)

    def _step(self, delta: int) -> None:
        self._set_play_icon(False)
        request = self.clock.step(delta)
        if request is not None:
            self.submit_request(request)

    def _seek_from_timeline(self) -> None:
        self._seek(self.timeline.value())

    def _seek(self, frame_index: int) -> None:
        self._set_play_icon(False)
        request = self.clock.seek(frame_index)
        if request is not None:
            self.submit_request(request)

    def _toggle_playback(self) -> None:
        if self.clock.paused:
            self.clock.play()
            self._set_play_icon(True)
            self._next_playback_deadline = time.monotonic()
            self._advance_playback()
        else:
            self.clock.pause()
            self._set_play_icon(False)

    def _set_play_icon(self, playing: bool) -> None:
        icon = (
            QtWidgets.QStyle.StandardPixmap.SP_MediaPause
            if playing
            else QtWidgets.QStyle.StandardPixmap.SP_MediaPlay
        )
        self.play_button.setIcon(self.style().standardIcon(icon))
        self.play_button.setToolTip("暂停" if playing else "播放")

    def _set_rate(self, index: int) -> None:
        if 0 <= index < len(_PLAYBACK_RATES):
            self.clock.set_rate(_PLAYBACK_RATES[index])

    def _advance_playback(self) -> None:
        if self.clock.paused or time.monotonic() < self._next_playback_deadline:
            return
        request = self.clock.begin_next_frame()
        if request is not None and self.submit_request(request):
            self._next_playback_deadline = float("inf")
        elif self.clock.paused:
            self._set_play_icon(False)

    def _poll_worker(self) -> None:
        result = self.worker.poll()
        if result is None:
            return
        if result.error is not None:
            if not self._closed:
                self.clock.pause()
                self._set_play_icon(False)
                self.notice_label.setText(str(result.error))
                self.notice_label.setVisible(True)
            return

        frame = result.frame
        assert frame is not None
        if self._initial_pending and result.request == self._initial_request:
            self._initial_pending = False
            accepted = True
        else:
            accepted = self.clock.complete(result.request)
        if accepted:
            self._display_frame(frame)
            if not self.clock.paused:
                self._next_playback_deadline = (
                    time.monotonic() + self.clock.frame_interval_ns / 1_000_000_000
                )
            else:
                self._set_play_icon(False)

        pending = self.clock.begin_pending_frame()
        if pending is not None:
            self.submit_request(pending)

    def _display_frame(self, frame: ReplayRenderFrame) -> None:
        self.raw_item.setData(
            pos=frame.raw_positions,
            color=_tag_colors(frame.raw_tags, raw=True),
        )
        terrain_mask = frame.permanent_tags == 1
        static_mask = frame.permanent_tags == 2
        terrain_positions = frame.permanent_positions[terrain_mask]
        static_positions = frame.permanent_positions[static_mask]
        self.terrain_item.setData(
            pos=terrain_positions,
            color=_tag_colors(frame.permanent_tags[terrain_mask]),
        )
        self.static_item.setData(
            pos=static_positions,
            color=_tag_colors(frame.permanent_tags[static_mask]),
        )
        self.moving_item.setData(
            pos=frame.moving_positions,
            color=_tag_colors(frame.moving_tags),
        )
        self.trajectory_item.setData(pos=frame.trajectory_positions)
        self.displayed_frame_index = frame.frame_index
        self._updating_timeline = True
        self.timeline.setValue(frame.frame_index)
        self._updating_timeline = False
        self.frame_label.setText(f"{frame.frame_index} / {self.timeline.maximum()}")
        self.notice_label.setText(frame.notice)
        self.notice_label.setVisible(bool(frame.notice) or not frame.map_available)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if not self._closed:
            self._closed = True
            self._result_timer.stop()
            self._playback_timer.stop()
            self.worker.close()
        super().closeEvent(event)


__all__ = [
    "MappingMcapReplaySource",
    "MappingReplayWindow",
    "ReplayFrameSource",
    "ReplayFrameWorker",
    "ReplayRenderFrame",
]
