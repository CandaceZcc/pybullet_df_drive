"""MID-360 Golf 同会话 GUI QA 的持久化合同。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import inspect
from types import SimpleNamespace

import pytest
import numpy as np


def _index(session_hex: str) -> SimpleNamespace:
    return SimpleNamespace(
        identity=SimpleNamespace(simulation_session_id=bytes.fromhex(session_hex))
    )


def test_mapping_replay_qa_persists_the_consumed_simulation_session(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """QA JSON 必须可证明它消费的正是 acceptance 的 simulation session。"""
    from scripts import verify_mid360_golf_mapping_replay as qa

    session_hex = "00112233445566778899aabbccddeeff"
    output_dir = tmp_path / "gui-qa"
    monkeypatch.setattr(
        qa,
        "_automate_replay_window",
        lambda **_kwargs: {
            "checks": {"initial_frame": {"raw_colored_pixel_count": 1}},
            "screenshots": ["initial-frame-0000.png"],
        },
    )

    result = qa.run_mapping_replay_qa(
        index=_index(session_hex),
        output_dir=output_dir,
        simulation_session_hex=session_hex,
    )

    assert result == {"passed": True, "qa": str(output_dir / "qa.json")}
    document = json.loads((output_dir / "qa.json").read_text(encoding="utf-8"))
    assert document["schema"] == "mid360-golf-mapping-replay-qa-v2"
    assert document["simulation_session_id"] == session_hex
    assert document["checks"] == {"initial_frame": {"raw_colored_pixel_count": 1}}
    assert document["screenshots"] == ["initial-frame-0000.png"]


def test_mapping_replay_qa_rejects_a_different_session_before_opening_gui(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """禁止把任何历史 MCAP 的 GUI 结果伪装为当前 acceptance。"""
    from scripts import verify_mid360_golf_mapping_replay as qa

    monkeypatch.setattr(
        qa,
        "_automate_replay_window",
        lambda **_kwargs: pytest.fail("mismatched session must not open GUI"),
    )

    with pytest.raises(ValueError, match="simulation session"):
        qa.run_mapping_replay_qa(
            index=_index("00112233445566778899aabbccddeeff"),
            output_dir=tmp_path / "gui-qa",
            simulation_session_hex="ffeeddccbbaa99887766554433221100",
        )


def test_capture_view_converts_qimage_with_the_qimage_format_enum(tmp_path: Path) -> None:
    """截图转换必须引用 Qt 的类枚举，不能读取尚未赋值的局部 image。"""
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6 import QtGui, QtWidgets
    from scripts import verify_mid360_golf_mapping_replay as qa

    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    image = QtGui.QImage(2, 1, QtGui.QImage.Format.Format_RGBA8888)
    image.fill(QtGui.QColor(255, 0, 0, 255))

    class Pixmap:
        def isNull(self) -> bool:
            return False

        def save(self, path: str, _format: str) -> bool:
            Path(path).write_bytes(b"png-fixture")
            return True

        def toImage(self) -> QtGui.QImage:
            return image

    class View:
        def grab(self) -> Pixmap:
            return Pixmap()

    pixels, count = qa._capture_view(View(), tmp_path / "view.png")

    assert application is not None
    assert pixels.shape == (1, 2, 4)
    assert count == 2


def test_gui_qa_wait_budget_allows_full_mcap_map_rebuild() -> None:
    """2082 帧真实 MCAP 的中点重建不能沿用普通控件的 30 秒等待。"""
    from scripts import verify_mid360_golf_mapping_replay as qa

    assert inspect.signature(qa._wait_until).parameters["timeout_s"].default == 180.0


def test_gui_qa_uses_a_bounded_real_seek_for_replay_smoke() -> None:
    """自动 smoke 验收真实 seek/rebuild，但不把两百万点的中点重建当作时间门。"""
    from scripts import verify_mid360_golf_mapping_replay as qa

    assert qa._qa_seek_frame_index(2082) == 16
    assert qa._qa_seek_frame_index(1) == 0


def test_overlap_pixel_difference_handles_real_viewport_resize() -> None:
    """回退前后 Qt 重新布局时，比较共同像素区域而非要求同一 framebuffer 尺寸。"""
    from scripts import verify_mid360_golf_mapping_replay as qa

    initial = np.zeros((4, 5, 4), dtype=np.uint8)
    rebuilt = np.ones((3, 2, 4), dtype=np.uint8)

    assert qa._overlap_mean_absolute_difference(initial, rebuilt) == 1.0
