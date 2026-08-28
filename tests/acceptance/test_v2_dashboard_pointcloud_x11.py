"""真实 X11：先创建 PyBullet GLX，再验证 Dashboard CPU 点云图像。"""
from __future__ import annotations

import os
import multiprocessing
import time
from importlib import import_module

import numpy as np
import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("SLOPE_SIM_X11_ACCEPTANCE") != "1" or not os.environ.get("DISPLAY"),
    reason="requires an explicitly enabled real X11 desktop session",
)


def _publish_real_v2_sensor_triplet() -> None:
    """独立 eCAL participant 只发布一组真实 raw v2 传感器字节。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    models = import_module("slope_sim.interfaces.v2.models")
    raw = import_module("slope_sim.interfaces.v2.ecal_raw").EcalRawBindings()
    core = raw._core
    assert core.initialize(f"runsim-x11-cloud-publisher-{os.getpid()}", 0x3F) is not False
    session = bytes.fromhex("00112233445566778899aabbccddeeff")
    timestamp_ns = 1_000_000_000
    try:
        frames = (
            models.LidarPointCloudV2(
                timestamp_ns, "lidar_link", 3, 1,
                (
                    models.LidarPointV2(0, 1.0, 2.0, 3.0, 100, 1, 0),
                    models.LidarPointV2(0, 3.0, 1.0, 2.0, 120, 2, 1),
                    models.LidarPointV2(0, 2.0, 3.0, 1.0, 140, 3, 2),
                ),
                1, 1, session, descriptor.sha256,
            ),
            models.RtkStateV2(
                timestamp_ns, 1, 1, "world",
                models.Point3dV2(0.0, 0.2, 0.4),
                models.Point3dV2(0.0, 0.0, 0.4),
                models.Point3dV2(0.0, -0.2, 0.4),
                0.0, session, descriptor.sha256,
            ),
            models.ImuAttitudeV2(
                timestamp_ns, 0.0, 0.0, 1, 1, "base_link", session, descriptor.sha256,
            ),
        )
        publishers = {
            contract.type_name: raw.create_publisher(contract.topic, contract.type_name, descriptor)
            for contract in import_module("slope_sim.interfaces.v2.topics").V2_TOPICS
        }
        discovery_deadline = time.monotonic() + 8.0
        while time.monotonic() < discovery_deadline:
            if all(
                publisher.get_subscriber_count() > 0
                for publisher in publishers.values()
            ):
                break
            time.sleep(0.05)
        if not all(publisher.get_subscriber_count() > 0 for publisher in publishers.values()):
            raise RuntimeError("real eCAL raw publisher did not discover Dashboard subscribers")
        for _ in range(60):
            for frame in frames:
                encoded = codec.encode(frame)
                raw.send(publishers[encoded.type_name], encoded.payload)
            time.sleep(0.05)
    finally:
        assert core.finalize() is not False


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
        assert not widget.cloud_canvas.pixmap().isNull()
        # 网格也会产生非背景像素，验收只能接受 CPU renderer 的语义调色板。
        semantic_rgb = {(192, 192, 32), (224, 64, 32), (128, 64, 192)}
        assert any(
            image.pixelColor(x, y).getRgb()[:3] in semantic_rgb
            for x in range(image.width())
            for y in range(image.height())
        )
        assert "本帧=3" in widget.cloud_status.text()
    finally:
        widget.close()
        application.processEvents()
        pybullet.disconnect(client_id)


def test_real_ecal_raw_triplet_reaches_receiver_world_transform_and_qimage() -> None:
    """不得用直接 CloudFrame 注入替代真实 eCAL→raw observer→QImage 路径。"""
    from PySide6.QtWidgets import QApplication

    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    adapter = import_module("slope_sim.interfaces.v2.dashboard_adapter")
    receiver_module = import_module("slope_sim.interfaces.v2.dashboard_receiver")
    transport_module = import_module("slope_sim.interfaces.v2.transport")
    raw_module = import_module("slope_sim.interfaces.v2.ecal_raw")
    application = QApplication.instance() or QApplication([])
    owner = transport_module.create_v2_ecal_transport(
        descriptor=descriptor,
        participant_name=f"runsim-x11-cloud-dashboard-{os.getpid()}",
        role="dashboard",
    )
    observer = receiver_module.V2DashboardRawObserverTransport(
        descriptor,
        raw_bindings=raw_module.EcalRawBindings(core=owner._bindings._raw._core),
    )
    receiver = receiver_module.V2DashboardEcalReceiver(descriptor, transport=observer)
    observer.set_diagnostic_callback(receiver.record_transport_error)
    widget = adapter.V2DashboardWidget(descriptor)
    publisher = multiprocessing.get_context("spawn").Process(
        target=_publish_real_v2_sensor_triplet,
        daemon=False,
    )
    try:
        widget.show()
        widget.cloud_workbench.setChecked(True)
        publisher.start()
        deadline = time.monotonic() + 12.0
        cloud_frame = None
        while time.monotonic() < deadline:
            application.processEvents()
            cloud_frame = receiver.cloud_frame()
            if (
                cloud_frame is not None
                and widget._last_cloud_sequence != cloud_frame.sequence
            ):
                assert widget.update_cloud_frame(cloud_frame)
            if cloud_frame is not None and widget._cloud_render_inflight is None:
                break
            time.sleep(0.01)
        assert cloud_frame is not None, receiver.diagnostics
        assert cloud_frame.positions.shape == (3, 3)
        assert cloud_frame.vehicle_position == pytest.approx((0.0, 0.0, 0.22))
        assert widget._cloud_render_inflight is None
        image = widget.cloud_canvas.pixmap().toImage()
        semantic_rgb = {(192, 192, 32), (224, 64, 32), (128, 64, 192)}
        assert any(
            image.pixelColor(x, y).getRgb()[:3] in semantic_rgb
            for x in range(image.width())
            for y in range(image.height())
        )
    finally:
        if publisher.pid is not None:
            publisher.join(timeout=3.0)
            if publisher.is_alive():
                publisher.terminate()
                publisher.join(timeout=3.0)
            assert publisher.exitcode == 0
        widget.close()
        application.processEvents()
        receiver.close()
        observer.close()
        owner.close()
