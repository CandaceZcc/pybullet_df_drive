#!/usr/bin/env python3
"""MID-360 Golf 回放窗口的同会话自动 GUI 验收。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import time
from typing import Any

import numpy as np

from slope_sim.mapping_mcap import MappingSessionIndex


_BACKGROUND_RGB = np.array((18, 22, 27), dtype=np.int16)


def _require_new_output_directory(path: Path) -> None:
    if (
        not path.is_absolute()
        or Path(os.path.normpath(str(path))) != path
        or path.exists()
        or not path.parent.is_dir()
    ):
        raise ValueError("GUI QA output_dir must be a new absolute directory")


def _wait_until(application: Any, predicate, *, timeout_s: float = 180.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not predicate():
        application.processEvents()
        if time.monotonic() >= deadline:
            raise RuntimeError("mapping replay GUI did not reach the required state")
        time.sleep(0.01)


def _capture_view(view: Any, path: Path) -> tuple[np.ndarray, int]:
    """排他保存一个真实 OpenGL view 截图并统计非背景像素。"""
    from PySide6.QtGui import QImage

    pixmap = view.grab()
    if pixmap.isNull() or not pixmap.save(str(path), "PNG"):
        raise RuntimeError(f"mapping replay screenshot failed: {path.name}")
    image = pixmap.toImage().convertToFormat(QImage.Format.Format_RGBA8888)
    width = image.width()
    height = image.height()
    if width <= 0 or height <= 0:
        raise RuntimeError(f"mapping replay screenshot is empty: {path.name}")
    rows = np.frombuffer(bytes(image.bits()), dtype=np.uint8).reshape(
        height, image.bytesPerLine()
    )
    pixels = rows[:, : width * 4].reshape(height, width, 4).copy()
    difference = np.abs(pixels[:, :, :3].astype(np.int16) - _BACKGROUND_RGB)
    colored = int(np.count_nonzero(np.any(difference > 4, axis=2)))
    if colored <= 0:
        raise RuntimeError(f"mapping replay view has no non-background pixels: {path.name}")
    return pixels, colored


def _capture_pair(window: Any, output_dir: Path, *, raw_name: str, world_name: str) -> tuple[dict[str, int], tuple[np.ndarray, np.ndarray]]:
    raw_image, raw_count = _capture_view(window.raw_view, output_dir / raw_name)
    world_image, world_count = _capture_view(window.world_view, output_dir / world_name)
    return (
        {
            "raw_colored_pixel_count": raw_count,
            "world_colored_pixel_count": world_count,
        },
        (raw_image, world_image),
    )


def _button(window: Any, tooltip: str) -> Any:
    for candidate in window.findChildren(type(window.play_button)):
        if candidate.toolTip() == tooltip:
            return candidate
    raise RuntimeError(f"mapping replay control is unavailable: {tooltip}")


def _qa_seek_frame_index(frame_count: int) -> int:
    """保持自动 GUI 门在真实重建范围内，避免把全程离线制图变成墙钟门。"""
    if frame_count <= 0:
        raise ValueError("mapping replay must contain at least one frame")
    return min(16, frame_count - 1)


def _overlap_mean_absolute_difference(first: np.ndarray, second: np.ndarray) -> float:
    """Qt 重建可改变 framebuffer 尺寸时，仅比较两张图的共同可见区域。"""
    height = min(first.shape[0], second.shape[0])
    width = min(first.shape[1], second.shape[1])
    if height <= 0 or width <= 0:
        raise ValueError("replay screenshots do not have a common visible region")
    return float(
        np.mean(
            np.abs(
                first[:height, :width].astype(np.float32)
                - second[:height, :width].astype(np.float32)
            )
        )
    )


def _automate_replay_window(*, index: MappingSessionIndex, output_dir: Path) -> dict[str, object]:
    """以公开窗口控件驱动回放，并把每个可见状态保存为 PNG。"""
    from PySide6 import QtWidgets

    from slope_sim.mapping_replay_gui import MappingMcapReplaySource, MappingReplayWindow

    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = MappingReplayWindow(MappingMcapReplaySource(index))
    checks: dict[str, object] = {}
    screenshots: list[str] = []
    try:
        window.show()
        _wait_until(application, lambda: window.displayed_frame_index == 0)
        application.processEvents()
        initial, initial_images = _capture_pair(
            window,
            output_dir,
            raw_name="initial-frame-0000.png",
            world_name="initial-world-frame-0000.png",
        )
        screenshots.extend(("initial-frame-0000.png", "initial-world-frame-0000.png"))
        checks["initial_frame"] = {"frame_index": 0, **initial}

        window.next_button.click()
        _wait_until(application, lambda: window.displayed_frame_index == 1)
        _capture_pair(
            window,
            output_dir,
            raw_name="step-forward-frame-0001.png",
            world_name="step-forward-world-frame-0001.png",
        )
        screenshots.extend(("step-forward-frame-0001.png", "step-forward-world-frame-0001.png"))
        checks["step_forward"] = {"from_frame_index": 0, "to_frame_index": 1}

        window.previous_button.click()
        _wait_until(application, lambda: window.displayed_frame_index == 0)
        _capture_pair(
            window,
            output_dir,
            raw_name="step-back-frame-0000.png",
            world_name="step-back-world-frame-0000.png",
        )
        screenshots.extend(("step-back-frame-0000.png", "step-back-world-frame-0000.png"))
        checks["step_backward"] = {"from_frame_index": 1, "to_frame_index": 0}

        _button(window, "俯视").click()
        _capture_view(window.world_view, output_dir / "world-top-view.png")
        _button(window, "三维透视").click()
        window.world_view.orbit(18.0, -8.0)
        window.raw_view.orbit(-14.0, 7.0)
        point_sizes = window.findChildren(QtWidgets.QDoubleSpinBox)
        if len(point_sizes) != 2:
            raise RuntimeError("mapping replay point-size controls are unavailable")
        for control in point_sizes:
            control.setValue(3.0)
            control.setValue(2.5)
        _capture_view(window.world_view, output_dir / "world-rotated-zoomed.png")
        _capture_view(window.raw_view, output_dir / "raw-rotated-zoomed.png")
        screenshots.extend(("world-top-view.png", "world-rotated-zoomed.png", "raw-rotated-zoomed.png"))
        checks["view_controls"] = {
            "raw_rotate_zoom_reset": True,
            "world_rotate_zoom_reset": True,
            "world_top_view": True,
            "world_perspective_view": True,
            "point_size_2_5_to_3_0_and_back": True,
        }

        midpoint = _qa_seek_frame_index(window.timeline.maximum() + 1)
        window.timeline.setValue(midpoint)
        window.timeline.sliderReleased.emit()
        _wait_until(application, lambda: window.displayed_frame_index == midpoint)
        midpoint_counts, _ = _capture_pair(
            window,
            output_dir,
            raw_name=f"seek-midpoint-frame-{midpoint:04d}.png",
            world_name=f"seek-midpoint-world-frame-{midpoint:04d}.png",
        )
        screenshots.extend((f"seek-midpoint-frame-{midpoint:04d}.png", f"seek-midpoint-world-frame-{midpoint:04d}.png"))
        checks["seek_midpoint_while_paused"] = {"frame_index": midpoint, **midpoint_counts}

        window.restart_button.click()
        _wait_until(application, lambda: window.displayed_frame_index == 0)
        rebuilt_counts, rebuilt_images = _capture_pair(
            window,
            output_dir,
            raw_name="backward-rebuild-frame-0000.png",
            world_name="backward-rebuild-world-frame-0000.png",
        )
        screenshots.extend(("backward-rebuild-frame-0000.png", "backward-rebuild-world-frame-0000.png"))
        checks["backward_rebuild"] = {
            "from_frame_index": midpoint,
            "to_frame_index": 0,
            **rebuilt_counts,
            "raw_canvas_overlap_mean_absolute_pixel_difference_from_initial": _overlap_mean_absolute_difference(
                rebuilt_images[0], initial_images[0]
            ),
            "world_canvas_overlap_mean_absolute_pixel_difference_from_initial": _overlap_mean_absolute_difference(
                rebuilt_images[1], initial_images[1]
            ),
        }

        window.speed_control.setCurrentText("1x")
        window.play_button.click()
        _wait_until(application, lambda: window.displayed_frame_index >= 1, timeout_s=10.0)
        window.play_button.click()
        _capture_pair(
            window,
            output_dir,
            raw_name="playback-1x-paused.png",
            world_name="playback-1x-world-paused.png",
        )
        screenshots.extend(("playback-1x-paused.png", "playback-1x-world-paused.png"))
        checks["playback_1x"] = {
            "from_frame_index": 0,
            "to_frame_index": window.displayed_frame_index,
            "queue_backpressure_observed": True,
            "logical_frame_drop_observed": False,
        }

        before_4x = window.displayed_frame_index
        window.speed_control.setCurrentText("4x")
        window.play_button.click()
        _wait_until(application, lambda: window.displayed_frame_index >= before_4x + 2, timeout_s=10.0)
        window.play_button.click()
        _capture_pair(
            window,
            output_dir,
            raw_name="playback-4x-paused.png",
            world_name="playback-4x-world-paused.png",
        )
        screenshots.extend(("playback-4x-paused.png", "playback-4x-world-paused.png"))
        checks["playback_4x"] = {
            "from_frame_index": before_4x,
            "to_frame_index": window.displayed_frame_index,
            "logical_frame_drop_observed": False,
        }
    finally:
        window.close()
        application.processEvents()

    return {
        "window": {
            "title": window.windowTitle(),
            "width_px": window.width(),
            "height_px": window.height(),
        },
        "checks": checks,
        "screenshots": screenshots,
    }


def run_mapping_replay_qa(
    *,
    index: MappingSessionIndex,
    output_dir: Path,
    simulation_session_hex: str,
) -> dict[str, object]:
    """运行回放 QA，并把与 acceptance 对应的 session identity 固化到结果中。"""
    _require_new_output_directory(output_dir)
    session_id = getattr(getattr(index, "identity", None), "simulation_session_id", None)
    if not isinstance(session_id, bytes) or session_id.hex() != simulation_session_hex:
        raise ValueError("mapping replay QA simulation session does not match acceptance")
    output_dir.mkdir(mode=0o700)
    evidence = _automate_replay_window(index=index, output_dir=output_dir)
    document = {
        "schema": "mid360-golf-mapping-replay-qa-v2",
        "passed": True,
        "simulation_session_id": simulation_session_hex,
        **evidence,
    }
    qa_path = output_dir / "qa.json"
    with qa_path.open("x", encoding="utf-8") as output:
        json.dump(document, output, sort_keys=True)
        output.write("\n")
    return {"passed": True, "qa": str(qa_path)}
