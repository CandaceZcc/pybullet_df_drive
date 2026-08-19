"""阶段四 B2：独立 v2 PySide6 Dashboard 的 GUI 线程消费合同。"""
from __future__ import annotations

import os
import hashlib
import json
from dataclasses import replace
from importlib import import_module
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


def _adapter_module():
    """让缺少 adapter 的 RED 指向明确的 v2 GUI 交付物。"""
    try:
        return import_module("slope_sim.interfaces.v2.dashboard_adapter")
    except ModuleNotFoundError as error:
        if error.name != "slope_sim.interfaces.v2.dashboard_adapter":
            raise
        pytest.fail("wished-for behavior is not implemented: v2 dashboard adapter", pytrace=False)


@pytest.fixture(scope="module")
def qapp():
    """使用真实 offscreen QApplication，保证测试覆盖实际 QWidget 渲染路径。"""
    from PySide6.QtWidgets import QApplication

    return QApplication.instance() or QApplication([])


def _snapshot_with_encoded_lidar():
    """构造只含 LiDAR 显示元数据的快照，原始 payload 不进入 GUI 合同。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    models = import_module("slope_sim.interfaces.v2.models")
    snapshot_type = import_module("slope_sim.interfaces.v2.dashboard_snapshot").V2DashboardSnapshot
    session = bytes.fromhex("00112233445566778899aabbccddeeff")
    wheel = models.WheelStateV2(
        100_000_000,
        (1.25, -1.25),
        (),
        10,
        3,
        4,
        "df_mid",
        session,
        descriptor.sha256,
        models.CommandAuthorityState.ACTIVE,
        "command.tool",
        b"c" * 16,
        1,
    )
    rtk = models.RtkStateV2(
        100_000_000,
        7,
        3,
        "base_link",
        models.Point3dV2(1.2, 2.3, 0.4),
        models.Point3dV2(1.0, 2.0, 0.4),
        models.Point3dV2(0.8, 1.7, 0.4),
        -0.25,
        session,
        descriptor.sha256,
    )
    imu = models.ImuAttitudeV2(
        100_000_000,
        0.1,
        -0.2,
        7,
        3,
        "base_link",
        session,
        descriptor.sha256,
    )
    return descriptor, snapshot_type(
        session,
        descriptor.sha256,
        3,
        wheel,
        100_000_000,
        7,
        2,
        rtk,
        imu,
    )


def test_v2_dashboard_widget_uses_bounded_metadata_without_decoding_lidar(qapp) -> None:
    """正式 GUI 只能读取不可变快照元数据，绝不能解码完整 LiDAR payload。"""
    module = _adapter_module()
    widget_type = getattr(module, "V2DashboardWidget", None)
    assert widget_type is not None, "v2 dashboard widget must exist"
    descriptor, snapshot = _snapshot_with_encoded_lidar()

    widget = widget_type(descriptor)
    assert not hasattr(widget, "_codec")
    widget.update_snapshot(snapshot)

    assert "1.250" in widget.wheel_value.text()
    assert widget.lidar_value.text() == "MID-360, 2 points, seq=7"
    assert "L=(1.200, 2.300, 0.400)" in widget.rtk_value.text()
    assert "roll=0.100" in widget.imu_value.text()
    assert not widget.top_view.scene().items()
    widget.close()


def test_v2_dashboard_default_preview_does_not_render_a_full_lidar_frame(qapp) -> None:
    """默认关闭的预览不能为完整 LiDAR 帧创建任何 Qt 图元。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    widget = module.V2DashboardWidget(descriptor)

    widget.update_snapshot(snapshot)

    assert len(widget.top_view.scene().items()) == 0
    widget.close()


def test_v2_dashboard_lightweight_preview_is_rate_limited_and_disableable(qapp, monkeypatch) -> None:
    """RTK 轻量预览最多 5 Hz，关闭后立即清空 Qt 图元。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    widget = module.V2DashboardWidget(descriptor)
    render_calls: list[None] = []
    original_render = widget._render_top_view

    def render_once() -> None:
        render_calls.append(None)
        original_render()

    monkeypatch.setattr(widget, "_render_top_view", render_once)
    widget.sampled_preview_group.setChecked(True)
    widget.update_snapshot(snapshot, observed_now=1.0)
    widget.update_snapshot(
        replace(snapshot, rtk=replace(snapshot.rtk, sequence=8, timestamp_ns=200_000_000)),
        observed_now=1.1,
    )
    widget.update_snapshot(
        replace(snapshot, rtk=replace(snapshot.rtk, sequence=9, timestamp_ns=300_000_000)),
        observed_now=1.21,
    )

    assert len(render_calls) == 2
    assert widget.top_view.scene().items()
    widget.sampled_preview_group.setChecked(False)
    assert not widget.top_view.scene().items()
    widget.close()


def test_v2_dashboard_reserves_width_and_wrapping_for_complete_rtk_text(qapp) -> None:
    """真实窗口必须容纳完整 LiDAR sequence 与三点 RTK，不能在右侧裁切。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    widget = module.V2DashboardWidget(descriptor)
    widget.update_snapshot(snapshot)

    assert widget.minimumWidth() >= 800
    assert widget.lidar_value.wordWrap() is True
    assert widget.rtk_value.wordWrap() is True
    widget.close()


def test_v2_dashboard_does_not_expose_a_full_lidar_preview_helper() -> None:
    """正式 Dashboard 没有完整点云抽样器，避免未来绕过快照边界。"""
    module = _adapter_module()
    widget_type = getattr(module, "V2DashboardWidget", None)
    assert widget_type is not None, "v2 dashboard widget must exist"

    assert not hasattr(widget_type, "_top_view_points")


def test_v2_dashboard_widget_rejects_mismatched_lightweight_telemetry_identity(qapp) -> None:
    """即使不读 LiDAR payload，wheel/RTK/IMU 仍必须匹配 snapshot 身份。"""
    module = _adapter_module()
    widget_type = getattr(module, "V2DashboardWidget", None)
    assert widget_type is not None, "v2 dashboard widget must exist"
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    snapshot_type = type(snapshot)
    mismatched = snapshot_type(
        snapshot.simulation_session_id,
        snapshot.descriptor_sha256,
        snapshot.world_generation + 1,
        snapshot.wheel_state,
        snapshot.lidar_timestamp_ns,
        snapshot.lidar_sequence,
        snapshot.lidar_point_count,
        snapshot.rtk,
        snapshot.imu,
    )

    widget = widget_type(descriptor)
    with pytest.raises(ValueError, match="telemetry identity does not match dashboard snapshot"):
        widget.update_snapshot(mismatched)
    widget.close()


def test_v2_dashboard_widget_refreshes_only_new_store_data_without_decoding_lidar(qapp, monkeypatch) -> None:
    """GUI 定时器跳过重复 snapshot，任何更新都不得解码完整 LiDAR。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    store_type = import_module("slope_sim.interfaces.v2.dashboard_snapshot").V2DashboardSnapshotStore
    prepared_type = import_module("slope_sim.interfaces.v2.sensor_frames").V2PreparedSensorFrames
    identity_type = import_module("slope_sim.interfaces.v2.session").OutputIdentity
    store = store_type()
    widget = module.V2DashboardWidget(descriptor)

    assert widget.refresh_from_store(store) is False
    store.update_wheel_state(snapshot.wheel_state)
    store.update_prepared_sensor_frames(
        prepared_type(
            b"encoded-lidar",
            snapshot.lidar_timestamp_ns,
            snapshot.rtk,
            snapshot.imu,
            identity_type(
                "/sim/lidar/points",
                snapshot.simulation_session_id,
                snapshot.descriptor_sha256,
                snapshot.world_generation,
                snapshot.lidar_sequence,
            ),
        )
    )

    assert widget.refresh_from_store(store) is True
    assert not hasattr(widget, "_codec")
    assert widget.refresh_from_store(store) is False
    store.update_wheel_state(
        replace(snapshot.wheel_state, timestamp_ns=110_000_000, sequence=11)
    )
    assert widget.refresh_from_store(store) is True
    assert not hasattr(widget, "_codec")
    assert "MID-360" in widget.lidar_value.text()
    widget.close()


def test_v2_dashboard_widget_refreshes_each_synchronous_lidar_frame(qapp) -> None:
    """同步帧也必须携带 LiDAR 版本键，避免 GUI 沿用上一帧点云缓存。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    models = import_module("slope_sim.interfaces.v2.models")
    frames_type = import_module("slope_sim.interfaces.v2.sensor_frames").V2SensorFrames
    store_type = import_module("slope_sim.interfaces.v2.dashboard_snapshot").V2DashboardSnapshotStore
    lidar = models.LidarPointCloudV2(
        snapshot.lidar_timestamp_ns,
        "lidar_link",
        2,
        1,
        (
            models.LidarPointV2(0, 1.0, 0.2, 0.1, 100, 1, 0),
            models.LidarPointV2(5, -0.3, 0.4, 0.2, 200, 2, 1),
        ),
        snapshot.lidar_sequence,
        snapshot.world_generation,
        snapshot.simulation_session_id,
        snapshot.descriptor_sha256,
    )
    second_lidar = replace(
        lidar,
        timebase_ns=200_000_000,
        point_num=3,
        points=lidar.points + (
            models.LidarPointV2(10, 20.0, 15.0, 0.5, 250, 3, 2),
        ),
        sequence=8,
    )
    store = store_type()
    widget = module.V2DashboardWidget(descriptor)

    store.update_sensor_frames(
        frames_type(lidar, snapshot.rtk, snapshot.imu), observed_at=0.0,
    )
    assert widget.refresh_from_store(store) is True
    assert not widget.top_view.scene().items()

    store.update_sensor_frames(
        frames_type(
            second_lidar,
            replace(snapshot.rtk, timestamp_ns=200_000_000, sequence=8),
            replace(snapshot.imu, timestamp_ns=200_000_000, sequence=8),
        ),
        observed_at=0.1,
    )
    assert widget.refresh_from_store(store) is True

    lidar_row = next(
        row for row in range(widget.topic_table.rowCount())
        if widget.topic_table.item(row, 0).text() == "/sim/lidar/points"
    )
    assert "3 points" in widget.lidar_value.text()
    assert "seq=8" in widget.lidar_value.text()
    assert widget.topic_table.item(lidar_row, 7).text() == "8"
    assert widget.topic_table.item(lidar_row, 9).text() == "3"
    assert not widget.top_view.scene().items()
    widget.close()


def test_v2_dashboard_live_status_has_fixed_five_topic_rows_and_telemetry_age(qapp) -> None:
    """Dashboard 把 session 和五话题运行证据放在主界面，而非完整点云。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    store_type = import_module("slope_sim.interfaces.v2.dashboard_snapshot").V2DashboardSnapshotStore
    store = store_type()
    store.update_snapshot(snapshot)
    widget = module.V2DashboardWidget(descriptor)

    widget.update_snapshot(store.snapshot())

    assert widget.topic_table.rowCount() == 5
    assert widget.topic_table.columnCount() == 11
    assert "MID-360 simulation" in widget.identity_value.text()
    headers = [widget.topic_table.horizontalHeaderItem(i).text() for i in range(11)]
    assert "Latest Sequence" in headers
    assert "Age" in headers
    assert "Errors" in headers
    assert "Action" in headers
    widget.close()


def test_v2_dashboard_shows_verified_ecal_and_actual_frequency(qapp) -> None:
    """正常 v2 会话必须显示 eCAL 已验证与测得频率，而非“接口未启用”。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    observation_type = import_module(
        "slope_sim.interfaces.v2.dashboard_snapshot"
    ).V2TopicObservation
    observations = tuple(
        observation_type(
            contract.topic,
            contract.rate_hz,
            float(contract.rate_hz),
            1,
            "verified",
            0,
            0,
            0,
            7,
            100_000_000,
            None,
            10.0,
            10.0,
        )
        for contract in import_module("slope_sim.interfaces.v2.topics").V2_TOPICS
    )
    widget = module.V2DashboardWidget(descriptor)

    widget.update_snapshot(replace(snapshot, topic_observations=observations), observed_now=10.1)

    assert widget.ecal_status_value.text() == "eCAL 已验证（五话题）"
    assert [widget.topic_table.item(row, 2).text() for row in range(5)] == [
        "100.0", "100.0", "10.0", "10.0", "10.0",
    ]
    assert [widget.topic_table.item(row, 4).text() for row in range(5)] == ["0"] * 5
    assert {widget.topic_table.item(row, 10).text() for row in range(5)} == {"运行正常"}
    widget.close()


@pytest.mark.parametrize(
    ("topic", "protocol_state", "peer_count", "error_count", "actual_hz", "expected"),
    (
        ("/sim/wheel/command", "not_checked", None, 0, None, "检查 eCAL 初始化"),
        ("/sim/wheel/command", "waiting", 0, 0, None, "启动唯一 C++ Command peer"),
        ("/sim/lidar/points", "verified", 1, 1, None, "检查 MID-360 worker"),
    ),
)
def test_v2_dashboard_explains_init_peer_and_worker_failures(
    qapp, topic, protocol_state, peer_count, error_count, actual_hz, expected,
) -> None:
    """初始化、peer 和 LiDAR worker 异常必须在相应 topic 行给出恢复动作。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    observation_type = import_module(
        "slope_sim.interfaces.v2.dashboard_snapshot"
    ).V2TopicObservation
    observations = tuple(
        observation_type(
            contract.topic,
            contract.rate_hz,
            actual_hz if contract.topic == topic else float(contract.rate_hz),
            peer_count if contract.topic == topic else 1,
            protocol_state if contract.topic == topic else "verified",
            error_count if contract.topic == topic else 0,
            0,
            0,
            None,
            None,
            None,
            None,
            None,
        )
        for contract in import_module("slope_sim.interfaces.v2.topics").V2_TOPICS
    )
    widget = module.V2DashboardWidget(descriptor)

    widget.update_snapshot(replace(snapshot, topic_observations=observations))

    row = next(
        index for index in range(widget.topic_table.rowCount())
        if widget.topic_table.item(index, 0).text() == topic
    )
    assert expected in widget.topic_table.item(row, 10).text()
    widget.close()


def test_v2_dashboard_prepared_lidar_table_never_decodes_raw_payload(qapp) -> None:
    """异步 raw bytes 只能停在运行时，GUI 表格不允许为点数解码它。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    store_type = import_module("slope_sim.interfaces.v2.dashboard_snapshot").V2DashboardSnapshotStore
    store = store_type()
    store.update_snapshot(snapshot)
    widget = module.V2DashboardWidget(descriptor)

    widget.refresh_from_store(store)

    lidar_row = next(
        row for row in range(widget.topic_table.rowCount())
        if widget.topic_table.item(row, 0).text() == "/sim/lidar/points"
    )
    assert widget.topic_table.item(lidar_row, 7).text() == "7"
    assert widget.topic_table.item(lidar_row, 9).text() == "2"
    widget.close()


def test_v2_dashboard_identity_displays_snapshot_robot_model_not_df_mid(qapp) -> None:
    """GUI 必须显示当前运行车型，不能把 df_mid 写死到身份区。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    store_type = import_module("slope_sim.interfaces.v2.dashboard_snapshot").V2DashboardSnapshotStore
    store = store_type(robot_model="df_front")
    store.update_snapshot(replace(snapshot, robot_model="df_front"))
    widget = module.V2DashboardWidget(descriptor)

    widget.update_snapshot(store.snapshot())

    assert "df_front" in widget.identity_value.text()
    assert "df_mid" not in widget.identity_value.text()
    widget.close()


def test_v2_dashboard_refresh_expires_stale_actual_rate_without_new_snapshot(qapp, monkeypatch) -> None:
    """GUI 定时刷新必须让断流频率变为 --，即使没有新 telemetry snapshot。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    store_type = import_module("slope_sim.interfaces.v2.dashboard_snapshot").V2DashboardSnapshotStore
    store = store_type()
    store.update_wheel_state(snapshot.wheel_state, observed_at=1.0)
    store.update_wheel_state(replace(snapshot.wheel_state, sequence=11), observed_at=1.1)
    store.update_wheel_state(replace(snapshot.wheel_state, sequence=12), observed_at=1.2)
    widget = module.V2DashboardWidget(descriptor)
    clock = iter((1.2, 3.3))
    monkeypatch.setattr(module.time, "monotonic", lambda: next(clock))

    assert widget.refresh_from_store(store) is True
    assert widget.topic_table.item(1, 2).text() != "--"
    assert widget.refresh_from_store(store) is True
    assert widget.topic_table.item(1, 2).text() == "--"
    widget.close()


def test_v2_dashboard_viewer_actions_are_fail_closed_until_configured(qapp) -> None:
    """peer_count 不是 Viewer 连接；实时 RViz 与 Livox Viewer 均必须默认禁用。"""
    module = _adapter_module()
    descriptor, _snapshot = _snapshot_with_encoded_lidar()
    widget = module.V2DashboardWidget(descriptor)

    assert widget.live_viewer_button.isEnabled() is False
    assert widget.offline_viewer_button.isEnabled() is False
    assert widget.live_viewer_status.text() == "未配置/连接未验证"
    assert widget.offline_viewer_status.text() == "未配置/连接未验证"
    widget.close()


def test_v2_dashboard_live_viewer_can_close_and_restart_without_touching_runtime(qapp) -> None:
    """受信显示 launcher 只管理 Bridge/RViz2，可单独关闭并再次启动。"""
    module = _adapter_module()
    descriptor, _snapshot = _snapshot_with_encoded_lidar()
    launches: list[int] = []
    closes: list[int] = []

    def launch() -> object:
        launch_id = len(launches) + 1
        launches.append(launch_id)
        return lambda: closes.append(launch_id)

    widget = module.V2DashboardWidget(descriptor, live_viewer_launcher=launch)

    assert widget.live_viewer_button.isEnabled() is True
    assert widget.live_viewer_close_button.isEnabled() is False
    widget.live_viewer_button.click()
    assert launches == [1]
    assert widget.live_viewer_close_button.isEnabled() is True
    assert "已打开" in widget.live_viewer_status.text()

    widget.live_viewer_close_button.click()
    assert closes == [1]
    assert widget.live_viewer_close_button.isEnabled() is False
    assert "已关闭" in widget.live_viewer_status.text()

    widget.live_viewer_button.click()
    assert launches == [1, 2]
    assert widget.live_viewer_close_button.isEnabled() is True
    widget.close()


def test_stage4_rviz_profile_uses_world_and_bounded_pointcloud_history() -> None:
    """实时点云累计只由独立 RViz 配置保留固定时窗。"""
    profile = Path(__file__).resolve().parents[2] / "cpp/client/rviz/stage4_live.rviz"

    text = profile.read_text(encoding="utf-8")

    assert "Fixed Frame: world" in text
    assert "Value: /slope_sim/lidar/points" in text
    assert "Queue Size: 1" in text
    assert "Decay Time: 0.5" in text


def test_v2_dashboard_sampled_preview_is_collapsed_and_not_acceptance_evidence(qapp) -> None:
    """轻量空间预览默认折叠，不能暗示它是点云验收或保存路径。"""
    module = _adapter_module()
    descriptor, _snapshot = _snapshot_with_encoded_lidar()
    widget = module.V2DashboardWidget(descriptor)

    assert widget.sampled_preview_group.isChecked() is False
    assert "轻量空间预览，非验收证据" in widget.sampled_preview_group.title()
    assert widget.top_view.isVisible() is False
    widget.close()


def test_v2_manual_controls_mount_v2_snapshot_widget_without_legacy_inactive_copy(
    qapp,
    monkeypatch,
) -> None:
    """v2 手动 GUI 保留控制输入，只显示 v2 eCAL 状态而非旧版禁用文案。"""
    module = _adapter_module()
    descriptor, snapshot = _snapshot_with_encoded_lidar()
    from slope_sim.dashboard import TelemetryDashboard

    inactive_calls: list[None] = []
    original_inactive = TelemetryDashboard._show_realtime_interface_inactive

    def record_inactive(self) -> None:
        inactive_calls.append(None)
        original_inactive(self)

    monkeypatch.setattr(TelemetryDashboard, "_show_realtime_interface_inactive", record_inactive)
    controls = TelemetryDashboard(show_lidar_tools=False, v2_dashboard_enabled=True)
    widget = module.V2DashboardWidget(descriptor)
    try:
        controls.attach_v2_dashboard_widget(widget)
        observation_type = import_module(
            "slope_sim.interfaces.v2.dashboard_snapshot"
        ).V2TopicObservation
        widget.update_snapshot(replace(
            snapshot,
            topic_observations=tuple(
                observation_type(contract.topic, contract.rate_hz, float(contract.rate_hz), 1, "verified")
                for contract in import_module("slope_sim.interfaces.v2.topics").V2_TOPICS
            ),
        ))

        assert controls.v2_dashboard_widget is widget
        assert inactive_calls == []
        assert controls.ecal_status_label.text() != "未启用"
        assert "关闭实时接口" not in controls.transport_detail_label.text()
        assert widget.ecal_status_value.text() == "eCAL 已验证（五话题）"
        assert "LiDAR点云" not in [
            controls.tabs.tabText(index) for index in range(controls.tabs.count())
        ]
        assert "MID-360" in widget.lidar_value.text()
        assert "C++ Recorder" in controls.capture_start_button.toolTip()
        assert "C++ Export" in controls.capture_stop_button.toolTip()
    finally:
        controls.close()


def _write_offline_evidence(
    tmp_path,
    *,
    relative_artifact=False,
    wrong_hash=False,
    cross_session=False,
    export_count=1,
):
    """构造最小完整离线证据链；内容均是小型临时普通文件。"""
    tmp_path.mkdir(parents=True, exist_ok=True)
    def artifact(name, payload=b"data"):
        path = tmp_path / name
        path.write_bytes(payload)
        return {"path": name if relative_artifact else str(path.resolve()),
                "sha256": "0" * 64 if wrong_hash else hashlib.sha256(payload).hexdigest()}
    identity = {"session": "0" * 32, "descriptor": "1" * 64, "world": 1, "scene": "flat"}
    lvx2 = artifact("cloud.lvx2")
    mcap = artifact("session.mcap")
    document = {
        "schema_version": 1, "kind": "stage4_v2_offline_evidence", "identity": identity,
        "recorder": {
            "identity": identity, "clean_shutdown": True, "mcap": mcap,
            "topic_counts": {
                "/sim/wheel/command": 1, "/sim/wheel/state": 10,
                "/sim/lidar/points": 1, "/sim/rtk/state": 1,
                "/sim/imu/attitude": 1,
            },
        },
        "replay": {
            "identity": identity, "clean_shutdown": True,
            "mcap": mcap,
            "result": artifact("replay-result.json"),
            "topic_counts": {
                "/replay/sim/wheel/state": 10,
                "/replay/sim/lidar/points": 1,
                "/replay/sim/rtk/state": 1,
                "/replay/sim/imu/attitude": 1,
            },
        },
        "export": {"identity": identity, "source_mcap": mcap,
                   "lvx2": lvx2, "synthetic": True, "lossiness": {"line": True},
                   "pcd_count": export_count, "ply_count": export_count,
                   "pcd_artifacts": [
                       artifact(f"cloud-{index:03}.pcd")
                       for index in range(export_count)
                   ],
                   "ply_artifacts": [
                       artifact(f"cloud-{index:03}.ply")
                       for index in range(export_count)
                   ]},
        "viewer_startup": {"identity": identity, "lvx2": lvx2, "smoke_passed": True},
        "viewer_display": {"identity": dict(identity, session="2" * 32) if cross_session else identity,
                           "lvx2": lvx2, "nonempty_pointcloud_visible": True,
                           "playback_progress_observed": True, "screenshot": artifact("viewer.png")},
    }
    evidence_path = tmp_path / "evidence.json"
    evidence_path.write_text(json.dumps(document), encoding="utf-8")
    return evidence_path


def _mutate_evidence(path, mutation) -> None:
    """在单一字段上破坏已生成的完整 evidence 文档。"""
    document = json.loads(path.read_text(encoding="utf-8"))
    mutation(document)
    path.write_text(json.dumps(document), encoding="utf-8")


def test_v2_dashboard_evidence_rejects_relative_or_tampered_artifacts(tmp_path) -> None:
    """离线证据不得通过相对路径或旧 hash 引用未审计文件。"""
    module = _adapter_module()
    with pytest.raises(ValueError, match="absolute"):
        module.load_offline_evidence(_write_offline_evidence(tmp_path, relative_artifact=True))
    with pytest.raises(ValueError, match="SHA-256"):
        module.load_offline_evidence(_write_offline_evidence(tmp_path, wrong_hash=True))


def test_v2_dashboard_evidence_rejects_cross_session_and_keeps_viewer_phases_separate(tmp_path) -> None:
    """display 必须与 startup/导出链同 session，两个 Viewer phases 不能合并。"""
    module = _adapter_module()
    with pytest.raises(ValueError, match="identity"):
        module.load_offline_evidence(_write_offline_evidence(tmp_path, cross_session=True))
    evidence = module.load_offline_evidence(_write_offline_evidence(tmp_path / "valid"))
    assert evidence["viewer_startup"] is not evidence["viewer_display"]


@pytest.mark.parametrize(
    "mutation",
    (
        lambda value: value["recorder"].update(clean_shutdown=False),
        lambda value: value["recorder"]["topic_counts"].pop("/sim/imu/attitude"),
        lambda value: value["replay"]["topic_counts"].update({"/replay/sim/lidar/points": True}),
        lambda value: value["export"].update(pcd_count=-1),
        lambda value: value["export"].update(lossiness={}),
        lambda value: value["viewer_startup"].pop("smoke_passed"),
    ),
)
def test_v2_dashboard_evidence_rejects_incomplete_or_nonfactual_sections(tmp_path, mutation) -> None:
    """每个 PASS section 都必须携带可核验事实，bool 不能冒充计数。"""
    module = _adapter_module()
    evidence_path = _write_offline_evidence(tmp_path)
    _mutate_evidence(evidence_path, mutation)

    with pytest.raises(ValueError, match="evidence"):
        module.load_offline_evidence(evidence_path)


@pytest.mark.parametrize(
    "mutation",
    (
        lambda value: value.update(schema_version=True),
        lambda value: value["recorder"].update(unexpected="drift"),
        lambda value: value["replay"].update(unexpected="drift"),
        lambda value: value["export"].update(unexpected="drift"),
        lambda value: value["viewer_startup"].update(unexpected="drift"),
        lambda value: value["viewer_display"].update(unexpected="drift"),
    ),
)
def test_v2_dashboard_evidence_rejects_bool_schema_and_unknown_section_fields(
    tmp_path, mutation,
) -> None:
    """schema_version 必须是 exact int 1，固定 section 禁止未知字段漂移。"""
    module = _adapter_module()
    evidence_path = _write_offline_evidence(tmp_path)
    _mutate_evidence(evidence_path, mutation)

    with pytest.raises(ValueError, match="evidence"):
        module.load_offline_evidence(evidence_path)


@pytest.mark.parametrize("fault", ("relative", "missing", "tampered"))
def test_v2_dashboard_evidence_validates_replay_mcap_artifact(tmp_path, fault) -> None:
    """Replay MCAP 与其它 artifact 使用相同绝对、存在、现场 hash 门。"""
    module = _adapter_module()
    evidence_path = _write_offline_evidence(tmp_path)
    document = json.loads(evidence_path.read_text(encoding="utf-8"))
    replay_mcap = document["replay"]["mcap"]
    if fault == "relative":
        replay_mcap["path"] = "session.mcap"
    elif fault == "missing":
        replay_mcap["path"] = str((tmp_path / "missing.mcap").resolve())
    else:
        replay_mcap["sha256"] = "f" * 64
    evidence_path.write_text(json.dumps(document), encoding="utf-8")

    with pytest.raises((ValueError, FileNotFoundError)):
        module.load_offline_evidence(evidence_path)


def test_v2_dashboard_evidence_rejects_unknown_identity_field(tmp_path) -> None:
    """严格 schema 的 identity 也只能包含固定四字段。"""
    module = _adapter_module()
    evidence_path = _write_offline_evidence(tmp_path)
    _mutate_evidence(
        evidence_path,
        lambda value: value["identity"].update(unexpected="drift"),
    )

    with pytest.raises(ValueError, match="identity"):
        module.load_offline_evidence(evidence_path)


def test_v2_dashboard_renders_verified_offline_evidence_without_changing_live_viewer_state(qapp, tmp_path) -> None:
    """离线 export/Viewer 证据可见，但绝不把它误呈现为实时 Viewer 连接。"""
    module = _adapter_module()
    descriptor, _snapshot = _snapshot_with_encoded_lidar()
    widget = module.V2DashboardWidget(descriptor)

    widget.set_offline_evidence(module.load_offline_evidence(_write_offline_evidence(tmp_path)))

    assert "synthetic=True" in widget.offline_evidence_values["export"].text()
    assert "PCD=1" in widget.offline_evidence_values["export"].text()
    assert "PLY=1" in widget.offline_evidence_values["export"].text()
    assert widget.live_viewer_button.isEnabled() is False
    assert widget.offline_viewer_button.isEnabled() is False
    widget.close()


def test_v2_dashboard_launches_only_the_verified_lvx2_with_a_configured_launcher(
    qapp, tmp_path,
) -> None:
    """可信启动器只能接收已校验 evidence 绑定的 LVX2，不能读取任意 UI 字符串。"""
    module = _adapter_module()
    descriptor, _snapshot = _snapshot_with_encoded_lidar()
    evidence = module.load_offline_evidence(_write_offline_evidence(tmp_path))
    launched: list[Path] = []
    widget = module.V2DashboardWidget(
        descriptor,
        offline_viewer_launcher=launched.append,
    )

    widget.set_offline_evidence(evidence)
    widget.offline_viewer_button.click()

    assert widget.offline_viewer_button.isEnabled() is True
    assert launched == [Path(evidence["export"]["lvx2"]["path"])]
    assert "已请求启动" in widget.offline_viewer_status.text()
    widget.close()


def test_v2_dashboard_offline_evidence_has_five_independent_nonlive_sections(qapp, tmp_path) -> None:
    """离线证据必须逐段呈现，缺省不能暗示任何 verifier 已运行。"""
    module = _adapter_module()
    descriptor, _snapshot = _snapshot_with_encoded_lidar()
    widget = module.V2DashboardWidget(descriptor)

    assert widget.offline_evidence_title.text() == "离线已验证证据，非实时状态"
    assert tuple(widget.offline_evidence_values) == (
        "recorder", "replay", "export", "viewer_startup", "viewer_display",
    )
    assert all(
        label.text() == "未提供 verifier evidence"
        for label in widget.offline_evidence_values.values()
    )

    widget.set_offline_evidence(module.load_offline_evidence(_write_offline_evidence(tmp_path)))
    assert "clean_shutdown=True" in widget.offline_evidence_values["recorder"].text()
    assert "4 topics" in widget.offline_evidence_values["replay"].text()
    assert "synthetic=True" in widget.offline_evidence_values["export"].text()
    assert "smoke_passed=True" in widget.offline_evidence_values["viewer_startup"].text()
    assert "nonempty=True" in widget.offline_evidence_values["viewer_display"].text()
    widget.close()


def test_v2_dashboard_offline_evidence_renders_auditable_paths_hashes_and_counts(qapp, tmp_path) -> None:
    """五段摘要保留实际计数，完整路径和 hash 进入独立审计明细。"""
    module = _adapter_module()
    descriptor, _snapshot = _snapshot_with_encoded_lidar()
    evidence = module.load_offline_evidence(_write_offline_evidence(tmp_path))
    widget = module.V2DashboardWidget(descriptor)

    widget.set_offline_evidence(evidence)

    recorder = widget.offline_evidence_values["recorder"].text()
    assert "/sim/wheel/command=1" in recorder
    assert "/sim/imu/attitude=1" in recorder

    replay = widget.offline_evidence_values["replay"].text()
    assert "/replay/sim/wheel/state=10" in replay
    assert "/replay/sim/imu/attitude=1" in replay

    export = widget.offline_evidence_values["export"].text()
    assert "PCD=1" in export
    assert "PLY=1" in export
    assert "lossiness={'line': True}" in export

    startup = widget.offline_evidence_values["viewer_startup"].text()
    assert "smoke_passed=True" in startup
    display = widget.offline_evidence_values["viewer_display"].text()
    assert "nonempty=True" in display

    detail = widget.offline_evidence_detail.toPlainText()
    assert evidence["recorder"]["mcap"]["path"] in detail
    assert evidence["recorder"]["mcap"]["sha256"] in detail
    assert evidence["replay"]["mcap"]["sha256"] in detail
    assert evidence["replay"]["result"]["sha256"] in detail
    assert evidence["export"]["source_mcap"]["sha256"] in detail
    assert evidence["export"]["lvx2"]["path"] in detail
    assert evidence["export"]["lvx2"]["sha256"] in detail
    assert evidence["export"]["pcd_artifacts"][0]["path"] in detail
    assert evidence["export"]["pcd_artifacts"][0]["sha256"] in detail
    assert evidence["export"]["ply_artifacts"][0]["path"] in detail
    assert evidence["export"]["ply_artifacts"][0]["sha256"] in detail
    assert evidence["viewer_display"]["screenshot"]["path"] in detail
    assert evidence["viewer_display"]["screenshot"]["sha256"] in detail
    widget.close()


def test_v2_dashboard_large_export_evidence_uses_bounded_scrollable_audit_detail(
    qapp, tmp_path,
) -> None:
    """50+50 导出制品不得撑开主摘要，完整审计内容由有界滚动控件承载。"""
    module = _adapter_module()
    descriptor, _snapshot = _snapshot_with_encoded_lidar()
    evidence = module.load_offline_evidence(
        _write_offline_evidence(tmp_path, export_count=50)
    )
    widget = module.V2DashboardWidget(descriptor)

    widget.set_offline_evidence(evidence)

    detail = widget.offline_evidence_detail
    detail_text = detail.toPlainText()
    assert detail.isReadOnly() is True
    assert detail.maximumHeight() <= 180
    assert all(
        len(label.text()) <= 512
        for label in widget.offline_evidence_values.values()
    )
    for artifacts in (
        evidence["export"]["pcd_artifacts"],
        evidence["export"]["ply_artifacts"],
    ):
        for artifact in (artifacts[0], artifacts[-1]):
            assert artifact["path"] in detail_text
            assert artifact["sha256"] in detail_text

    widget.show()
    for width in (1100, 820):
        widget.resize(width, 680)
        qapp.processEvents()
        evidence_widgets = [
            widget.offline_evidence_title,
            *widget.offline_evidence_values.values(),
            detail,
        ]
        assert all(
            not first.geometry().intersects(second.geometry())
            for index, first in enumerate(evidence_widgets)
            for second in evidence_widgets[index + 1:]
        )
    assert detail.verticalScrollBar().maximum() > 0
    assert detail.horizontalScrollBar().maximum() > 0
    widget.close()
