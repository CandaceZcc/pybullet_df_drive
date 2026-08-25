"""真实 X11：先创建 PyBullet GLX，再验证 Dashboard CPU 点云图像。"""
from __future__ import annotations

import os
import time
from importlib import import_module

import numpy as np
import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("SLOPE_SIM_X11_ACCEPTANCE") != "1" or not os.environ.get("DISPLAY"),
    reason="requires an explicitly enabled real X11 desktop session",
)


def test_pybullet_then_dashboard_pointcloud_is_visible_and_closes_cleanly() -> None:
    """真实桌面不得出现黑屏、GL 上下文异常或关闭卡死。"""
    import pybullet as pybullet
    from PySide6.QtWidgets import QApplication

    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    adapter = import_module("slope_sim.interfaces.v2.dashboard_adapter")
    receiver = import_module("slope_sim.interfaces.v2.dashboard_receiver")
    client_id = pybullet.connect(pybullet.GUI)
    assert client_id >= 0
    application = QApplication.instance() or QApplication([])
    widget = adapter.V2DashboardWidget(descriptor)
    frame = receiver.V2DashboardCloudFrame(
        1_000_000_000,
        1,
        np.asarray(((1.0, 2.0, 3.0), (3.0, 1.0, 2.0), (2.0, 3.0, 1.0)), dtype=np.float32),
        np.asarray((100, 120, 140), dtype=np.uint32),
        np.asarray((1, 2, 3), dtype=np.uint8),
    )
    try:
        widget.show()
        widget.cloud_workbench.setChecked(True)
        assert widget.update_cloud_frame(frame)
        deadline = time.monotonic() + 5.0
        while widget._cloud_render_inflight is not None and time.monotonic() < deadline:
            application.processEvents()
            time.sleep(0.01)
        application.processEvents()
        assert widget._cloud_render_inflight is None
        image = widget.cloud_canvas.pixmap().toImage()
        background = image.pixelColor(0, 0)
        non_background = sum(
            image.pixelColor(x, y) != background
            for x in range(0, image.width(), 24)
            for y in range(0, image.height(), 24)
        )
        assert non_background > 0
        assert "本帧=3" in widget.cloud_status.text()
    finally:
        widget.close()
        application.processEvents()
        pybullet.disconnect(client_id)
