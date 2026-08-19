"""MID-360 Golf 同步双三维回放窗口合同。"""
from __future__ import annotations

from importlib import import_module
import os
from threading import Event
import time

import numpy as np
import pytest


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


def _application():
    from PySide6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def _wait_until(application, predicate, *, timeout_s: float = 3.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not predicate():
        application.processEvents()
        if time.monotonic() >= deadline:
            raise AssertionError("Qt condition did not become true")
        time.sleep(0.005)


def _render_frame(gui, request):
    base = float(request.frame_index)
    return gui.ReplayRenderFrame(
        generation=request.generation,
        frame_index=request.frame_index,
        timebase_ns=request.frame_index * 100_000_000,
        raw_positions=np.array(((base, 0.0, 0.0), (base, 1.0, 0.2))),
        raw_tags=np.array((1, 3)),
        permanent_positions=np.array(((0.0, 0.0, 0.0), (1.0, 1.0, 0.1))),
        permanent_tags=np.array((1, 2)),
        moving_positions=np.array(((base, -1.0, 0.2),)),
        moving_tags=np.array((3,)),
        trajectory_positions=np.array(((0.0, 0.0, 0.0), (base, 0.0, 0.0))),
        map_available=True,
        notice="",
    )


class _ImmediateSource:
    frame_times_ns = (0, 100_000_000, 200_000_000)

    def __init__(self, gui) -> None:
        self.gui = gui
        self.calls: list[int] = []

    def render_frame(self, request, cancellation):
        assert cancellation.is_set() is False
        self.calls.append(request.frame_index)
        return _render_frame(self.gui, request)


def test_render_frame_copies_readonly_float32_gpu_arrays() -> None:
    """后台结果必须是连续只读数组，Qt 线程不能观察到生产者后续修改。"""
    gui = import_module("slope_sim.mapping_replay_gui")
    source = np.array(((1.0, 2.0, 3.0),), dtype=np.float64)
    tags = np.array((1,), dtype=np.int64)
    frame = gui.ReplayRenderFrame(
        generation=1,
        frame_index=0,
        timebase_ns=0,
        raw_positions=source,
        raw_tags=tags,
        permanent_positions=source,
        permanent_tags=tags,
        moving_positions=np.empty((0, 3)),
        moving_tags=np.empty((0,)),
        trajectory_positions=np.empty((0, 3)),
        map_available=True,
        notice="",
    )
    source[0, 0] = 99.0

    assert frame.raw_positions.dtype == np.float32
    assert frame.raw_positions.flags.c_contiguous
    assert frame.raw_positions.flags.writeable is False
    assert frame.raw_positions[0, 0] == 1.0
    assert frame.raw_tags.dtype == np.uint8
    assert frame.raw_tags.flags.writeable is False


def test_window_builds_dual_gl_views_controls_and_initial_frame() -> None:
    """第一屏就是 43/57 双三维工具面，不经过 landing page 或二维预览。"""
    gui = import_module("slope_sim.mapping_replay_gui")
    application = _application()
    source = _ImmediateSource(gui)
    window = gui.MappingReplayWindow(source)
    try:
        window.resize(1200, 720)
        window.show()
        _wait_until(application, lambda: window.displayed_frame_index == 0)
        sizes = window.splitter.sizes()

        assert window.splitter.count() == 2
        assert sum(sizes) > 0
        assert sizes[0] / sum(sizes) == pytest.approx(0.43, abs=0.03)
        assert window.worker.request_capacity == 1
        assert window.worker.result_capacity == 1
        assert window.timeline.minimum() == 0
        assert window.timeline.maximum() == 2
        assert tuple(
            window.speed_control.itemText(index)
            for index in range(window.speed_control.count())
        ) == ("0.25x", "0.5x", "1x", "2x", "4x")
        assert window.raw_item.pos.shape == (2, 3)
        assert window.terrain_item.pos.shape[1] == 3
        assert window.static_item.pos.shape[1] == 3
        assert window.moving_item.pos.shape == (1, 3)
        assert source.calls == [0]
    finally:
        window.close()
        application.processEvents()


def test_slow_worker_keeps_timeline_on_current_logical_frame() -> None:
    """容量 1 后台落后时等待结果，不能跳到后续 LiDAR frame。"""
    gui = import_module("slope_sim.mapping_replay_gui")
    application = _application()
    gate = Event()

    class SlowSource(_ImmediateSource):
        def render_frame(self, request, cancellation):
            self.calls.append(request.frame_index)
            if request.frame_index == 1:
                while not gate.wait(0.005):
                    if cancellation.is_set():
                        raise RuntimeError("cancelled")
            return _render_frame(self.gui, request)

    source = SlowSource(gui)
    window = gui.MappingReplayWindow(source)
    try:
        _wait_until(application, lambda: window.displayed_frame_index == 0)
        window.clock.play()
        request = window.clock.begin_next_frame()
        assert request is not None
        assert window.submit_request(request) is True
        _wait_until(application, lambda: source.calls == [0, 1])

        assert window.clock.begin_next_frame() is None
        assert window.displayed_frame_index == 0
        assert source.calls == [0, 1]
        gate.set()
        _wait_until(application, lambda: window.displayed_frame_index == 1)
        assert source.calls == [0, 1]
    finally:
        gate.set()
        window.close()
        application.processEvents()


def test_mcap_source_streams_forward_and_rebuilds_map_when_seeking_back() -> None:
    """顺播复用流，回退必须重开 MCAP 并从空地图确定性累计。"""
    gui = import_module("slope_sim.mapping_replay_gui")
    replay = import_module("slope_sim.mapping_replay")
    from slope_sim.interfaces.v2.models import LidarPointCloudV2, LidarPointV2
    from slope_sim.sensor_backend import Pose

    session_id = bytes.fromhex("00112233445566778899aabbccddeeff")
    descriptor = bytes.fromhex("11" * 32)
    pose_nodes = tuple(
        replay.RecoveredPoseNode(
            frame_index * 100_000_000,
            Pose((float(frame_index), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            Pose((float(frame_index), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        )
        for frame_index in range(3)
    )
    clouds = tuple(
        LidarPointCloudV2(
            frame_index * 100_000_000,
            "lidar_link",
            1,
            1,
            (LidarPointV2(0, 1.0, float(frame_index), 0.0, 100, 1, 0),),
            frame_index,
            1,
            session_id,
            descriptor,
        )
        for frame_index in range(2)
    )

    class FakeIndex:
        lidar_frame_times_ns = (0, 100_000_000)

        def __init__(self) -> None:
            self.pose_nodes = pose_nodes
            self.iterations = 0
            self.completed_iterations = 0

        def iter_lidar_frames(self):
            self.iterations += 1
            yield from clouds
            self.completed_iterations += 1

    index = FakeIndex()
    source = gui.MappingMcapReplaySource(index)
    cancellation = Event()

    first = source.render_frame(replay.PlaybackFrameRequest(1, 0, True), cancellation)
    second = source.render_frame(replay.PlaybackFrameRequest(2, 1, False), cancellation)
    assert index.completed_iterations == 1
    rebuilt = source.render_frame(replay.PlaybackFrameRequest(3, 0, True), cancellation)

    assert index.iterations == 2
    assert first.raw_positions.tolist() == [[1.0, 0.0, 0.0]]
    assert second.raw_positions.tolist() == [[1.0, 1.0, 0.0]]
    assert second.permanent_positions.shape == (2, 3)
    assert second.trajectory_positions.tolist() == [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
    ]
    assert rebuilt.permanent_positions.shape == (1, 3)
    assert rebuilt.trajectory_positions.tolist() == [[0.0, 0.0, 0.0]]
    source.close()


def test_mcap_source_keeps_raw_frame_when_last_lookahead_is_missing() -> None:
    """未包围的最后帧保留 raw，且明确提示该帧没有进入世界地图。"""
    gui = import_module("slope_sim.mapping_replay_gui")
    replay = import_module("slope_sim.mapping_replay")
    from slope_sim.interfaces.v2.models import LidarPointCloudV2, LidarPointV2
    from slope_sim.sensor_backend import Pose

    session_id = bytes.fromhex("00112233445566778899aabbccddeeff")
    descriptor = bytes.fromhex("11" * 32)
    pose_nodes = tuple(
        replay.RecoveredPoseNode(
            frame_index * 100_000_000,
            Pose((float(frame_index), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            Pose((float(frame_index), 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        )
        for frame_index in range(2)
    )
    clouds = tuple(
        LidarPointCloudV2(
            frame_index * 100_000_000,
            "lidar_link",
            1,
            1,
            (LidarPointV2(0, 1.0, float(frame_index), 0.0, 100, 1, 0),),
            frame_index,
            1,
            session_id,
            descriptor,
        )
        for frame_index in range(2)
    )

    class FakeIndex:
        lidar_frame_times_ns = (0, 100_000_000)

        def __init__(self) -> None:
            self.pose_nodes = pose_nodes

        def iter_lidar_frames(self):
            yield from clouds

    source = gui.MappingMcapReplaySource(FakeIndex())
    frame = source.render_frame(
        replay.PlaybackFrameRequest(1, 1, True),
        Event(),
    )

    assert frame.raw_positions.tolist() == [[1.0, 1.0, 0.0]]
    assert frame.permanent_positions.shape == (1, 3)
    assert frame.map_available is False
    assert frame.notice == "LiDAR frame is missing its next 100 ms pose node"
    source.close()
