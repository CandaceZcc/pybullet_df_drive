"""阶段四 B2：真实 PyBullet DIRECT 世界中的五话题 v2 runtime 入口。"""
from pathlib import Path
from types import SimpleNamespace

import pytest

from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb
from slope_sim.interfaces.transport import TransportSnapshot, TransportTopicQuality
from slope_sim.model_registry import robot_model_names
from slope_sim.obstacles import ObstacleGeometry, ObstaclePath, ObstacleSpec
from slope_sim.scene_config import (
    SceneDocument,
    SensorDocument,
    TerrainDocument,
    dump_scene_atomic,
)


def _golf_obstacles_motion_document() -> SceneDocument:
    """返回阶段四目视验收冻结的可复现场景。"""
    return SceneDocument(
        schema_version=1,
        robot_model="df_mid",
        terrain=TerrainDocument("golf_heightfield", 0.0, 41, "medium"),
        obstacles=(
            ObstacleSpec(
                1,
                "static",
                ObstacleGeometry("box", (0.35, 0.35, 0.60)),
                (-0.8, 1.8, 0.60),
                (0.0, 0.0, 0.0, 1.0),
            ),
            ObstacleSpec(
                2,
                "static",
                ObstacleGeometry("cylinder", (0.32, 0.32, 0.70)),
                (0.7, -1.7, 0.70),
                (0.0, 0.0, 0.0, 1.0),
            ),
            ObstacleSpec(
                3,
                "moving",
                ObstacleGeometry("box", (0.35, 0.35, 0.55)),
                (-0.2, -0.4, 0.55),
                (0.0, 0.0, 0.0, 1.0),
                ObstaclePath((-0.2, -0.4), (-0.2, 0.8), 0.30, 0.0, 1),
            ),
        ),
        sensors=SensorDocument.default(),
    )


class RecordingV2Transport:
    """记录 DIRECT 集成所发 raw frame，并提供等待 command peer 的 transport 快照。"""

    def __init__(self) -> None:
        self.published: list[tuple[str, bytes, str, int, float]] = []
        self.closed = False

    def subscribe(self, topic: str, type_name: str, callback) -> None:
        assert (topic, type_name) == (
            "/sim/wheel/command", "slope_sim.interfaces.v2.WheelCommand"
        )
        self.callback = callback

    def poll_peer_state(self) -> None:
        """无 command peer 时 controller 必须维持等待与安全停车。"""

    def snapshot(self) -> TransportSnapshot:
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=False,
            published_count=len(self.published),
            received_count=0,
            error_count=0,
            dropped_count=0,
            topic_quality=(
                TransportTopicQuality(
                    topic="/sim/wheel/command",
                    peer_connected=False,
                    peer_count=0,
                    protocol_state="waiting",
                    protocol_detail="",
                ),
            ),
        )

    def publish(
        self,
        topic: str,
        payload: bytes,
        type_name: str,
        sim_time_ns: int,
        *,
        wall_time: float,
    ) -> bool:
        self.published.append((topic, bytes(payload), type_name, sim_time_ns, wall_time))
        return True

    def close(self) -> None:
        self.closed = True


class VerifiedPeerTransport(RecordingV2Transport):
    """模拟五话题 discovery 已收敛的真实 raw eCAL 连接。"""

    def __init__(self) -> None:
        super().__init__()
        self.idle_waits: list[float] = []

    def wait_idle(self, *, timeout_sec: float) -> None:
        """模拟五条 lane 已经排空，并记录正式 runtime 的关闭门。"""
        self.idle_waits.append(timeout_sec)

    def snapshot(self) -> TransportSnapshot:
        return TransportSnapshot(
            mode="ecal",
            ecal_connected=True,
            published_count=len(self.published),
            received_count=0,
            error_count=0,
            dropped_count=0,
            topic_quality=tuple(
                TransportTopicQuality(
                    topic=topic,
                    peer_connected=True,
                    peer_count=1,
                    protocol_state="verified",
                    protocol_detail="",
                    remote_type_names=("test.v2.Message",),
                    remote_encodings=("proto",),
                    remote_descriptor_sha256=("0" * 64,),
                )
                for topic in (
                    "/sim/wheel/command",
                    "/sim/wheel/state",
                    "/sim/lidar/points",
                    "/sim/rtk/state",
                    "/sim/imu/attitude",
                )
            ),
        )


def test_v2_direct_scene_document_is_authoritative(tmp_path: Path) -> None:
    """正式 DIRECT 入口必须只用 scene 文档选择车型与可复现 golf 参数。"""
    from scripts import stage4_v2_simulation_runtime as entry

    scene_path = dump_scene_atomic(
        _golf_obstacles_motion_document(),
        tmp_path / "golf-obstacles-motion.yaml",
    )

    result = entry.run_v2_simulation_runtime(
        result_json=tmp_path / "scene-result.json",
        duration_sec=0.1,
        scene=scene_path,
        transport_factory=lambda _descriptor: RecordingV2Transport(),
    )

    assert {
        "robot_model": result["robot_model"],
        "terrain_model": result["terrain_model"],
        "slope_deg": result["slope_deg"],
        "golf_seed": result["golf_seed"],
        "golf_relief": result["golf_relief"],
    } == {
        "robot_model": "df_mid",
        "terrain_model": "golf_heightfield",
        "slope_deg": 0.0,
        "golf_seed": 41,
        "golf_relief": "medium",
    }


@pytest.mark.parametrize(
    "selector",
    (
        {"robot_model": "df_mid"},
        {"terrain_model": "golf_heightfield"},
    ),
)
def test_v2_direct_scene_rejects_world_selector_conflicts(
    tmp_path: Path,
    selector: dict[str, str],
) -> None:
    """scene 与任何显式车型或地形 selector 同时出现时都必须拒绝。"""
    from scripts import stage4_v2_simulation_runtime as entry

    scene_path = dump_scene_atomic(
        _golf_obstacles_motion_document(),
        tmp_path / "golf-obstacles-motion.yaml",
    )

    with pytest.raises(
        ValueError,
        match="scene cannot be combined with robot_model or terrain_model",
    ):
        entry.run_v2_simulation_runtime(
            result_json=tmp_path / "conflict-result.json",
            duration_sec=0.1,
            scene=scene_path,
            transport_factory=lambda _descriptor: RecordingV2Transport(),
            **selector,
        )


def test_v2_direct_scene_is_validated_before_connect(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """损坏 scene 必须在创建任何 PyBullet client 之前失败。"""
    from scripts import stage4_v2_simulation_runtime as entry

    scene_path = tmp_path / "invalid.yaml"
    scene_path.write_text("schema_version: [", encoding="utf-8")
    monkeypatch.setattr(
        entry.p,
        "connect",
        lambda _mode: pytest.fail("PyBullet connected before scene validation"),
    )

    with pytest.raises(ValueError, match="scene YAML is malformed"):
        entry.run_v2_simulation_runtime(
            result_json=tmp_path / "invalid-result.json",
            duration_sec=0.1,
            scene=scene_path,
            transport_factory=lambda _descriptor: RecordingV2Transport(),
        )


def test_v2_direct_moving_obstacle_advances_between_lidar_frames(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """移动障碍物必须在真实物理步后进入无 body-id 的下一帧 capture。"""
    from scripts import stage4_v2_simulation_runtime as entry

    class CaptureService:
        """只记录 runtime 交给 worker 的快照，隔离固定 LiDAR 性能预算。"""

        def __init__(self) -> None:
            self.captures: list[tuple[int, tuple[object, ...]]] = []
            self._prepared: list[SimpleNamespace] = []

        def capture(
            self,
            *,
            topic: str,
            timestamp_ns: int,
            complete_obstacle_snapshots_without_body_ids: tuple[object, ...],
            **_kwargs: object,
        ) -> bool:
            self.captures.append((timestamp_ns, complete_obstacle_snapshots_without_body_ids))
            self._prepared.append(
                SimpleNamespace(
                    topic=topic,
                    timestamp_ns=timestamp_ns,
                    protobuf_payload=b"capture",
                )
            )
            return True

        def poll(self) -> SimpleNamespace | None:
            return self._prepared.pop(0) if self._prepared else None

        def drain_events(self) -> tuple[object, ...]:
            return ()

        def begin_draining(self) -> None:
            return None

        def close_idle(self) -> None:
            return None

        def force_close(self) -> None:
            return None

    service = CaptureService()
    worker_handle = SimpleNamespace(ready=SimpleNamespace(prewarmed_topics=("lidar_link",)))
    monkeypatch.setattr(entry, "start_lidar_worker", lambda *_args, **_kwargs: worker_handle)
    monkeypatch.setattr(
        entry,
        "LidarScanService",
        SimpleNamespace(from_worker_handle=lambda _handle, **_kwargs: service),
    )

    scene_path = dump_scene_atomic(
        _golf_obstacles_motion_document(),
        tmp_path / "golf-obstacles-motion.yaml",
    )
    transport = RecordingV2Transport()

    entry.run_v2_simulation_runtime(
        result_json=tmp_path / "moving-result.json",
        duration_sec=0.2,
        scene=scene_path,
        transport_factory=lambda _descriptor: transport,
    )

    assert [
        timestamp_ns for timestamp_ns, _snapshots in service.captures
    ] == [100_000_000, 200_000_000]
    moving_positions = []
    for _timestamp_ns, snapshots in service.captures:
        assert all(getattr(snapshot, "body_id") is None for snapshot in snapshots)
        moving_positions.append(
            next(
                getattr(snapshot, "position")
                for snapshot in snapshots
                if getattr(snapshot, "logical_id") == 3
            )
        )
    displacement = sum(
        (moving_positions[0][index] - moving_positions[1][index]) ** 2
        for index in range(3)
    ) ** 0.5
    assert displacement > 0.01


def test_v2_direct_entry_runs_real_center_sensor_world_and_writes_five_topic_result(tmp_path: Path) -> None:
    """DIRECT 100 ms 必须真实步进并发布 10 个 wheel、各一帧三传感器。"""
    from scripts import stage4_v2_simulation_runtime as entry

    transport = RecordingV2Transport()
    result_path = tmp_path / "v2-result.json"

    result = entry.run_v2_simulation_runtime(
        result_json=result_path,
        duration_sec=0.1,
        robot_model="df_mid",
        transport_factory=lambda _descriptor: transport,
    )

    assert result_path.is_file()
    assert result["clean_shutdown"] is True
    assert result["physics_steps"] == 24
    assert result["published_frames"] == {
        "/sim/wheel/state": 10,
        "/sim/lidar/points": 1,
        "/sim/rtk/state": 1,
        "/sim/imu/attitude": 1,
    }
    assert result["dashboard_snapshot"] == {
        "wheel_timestamp_ns": 100_000_000,
        "lidar_timestamp_ns": 100_000_000,
        "lidar_sequence": 0,
        "rtk_timestamp_ns": 100_000_000,
        "imu_timestamp_ns": 100_000_000,
    }
    assert [topic for topic, *_rest in transport.published].count("/sim/wheel/state") == 10
    assert [topic for topic, *_rest in transport.published].count("/sim/lidar/points") == 1
    assert [topic for topic, *_rest in transport.published].count("/sim/rtk/state") == 1
    assert [topic for topic, *_rest in transport.published].count("/sim/imu/attitude") == 1
    assert transport.closed is True


def test_v2_direct_entry_can_require_all_five_verified_peers_before_first_physics_step(
    tmp_path: Path,
) -> None:
    """真实 eCAL 验收必须先看到每个 v2 topic 的唯一 verified peer。"""
    from scripts import stage4_v2_simulation_runtime as entry

    transport = VerifiedPeerTransport()
    result = entry.run_v2_simulation_runtime(
        result_json=tmp_path / "verified-peer-result.json",
        duration_sec=0.1,
        robot_model="df_mid",
        transport_factory=lambda _descriptor: transport,
        require_verified_peers=True,
        peer_timeout_sec=0.1,
    )

    assert result["clean_shutdown"] is True
    assert len(transport.published) == 13
    assert transport.idle_waits == [0.1]
    assert result["wall_duration_sec"] >= 0.09


def test_v2_direct_entry_uses_and_closes_the_stage4_center_lidar_worker(
    tmp_path: Path,
) -> None:
    """正式 DIRECT runtime 必须异步使用唯一中心 worker，并在结果中留下关闭诊断。"""
    from scripts import stage4_v2_simulation_runtime as entry

    result = entry.run_v2_simulation_runtime(
        result_json=tmp_path / "async-worker-result.json",
        duration_sec=0.1,
        robot_model="df_mid",
        transport_factory=lambda _descriptor: RecordingV2Transport(),
    )

    assert result["lidar_worker"] == {
        "prewarmed_topics": ["lidar_link"],
        "clean_shutdown": True,
    }


def test_v2_direct_entry_can_publish_its_immutable_dashboard_store_to_a_gui_consumer(
    tmp_path: Path,
) -> None:
    """正式 runtime 必须允许 GUI 线程读取其写入的同一份 v2 Dashboard store。"""
    from scripts import stage4_v2_simulation_runtime as entry
    from slope_sim.interfaces.v2.dashboard_snapshot import V2DashboardSnapshotStore

    store = V2DashboardSnapshotStore()
    result = entry.run_v2_simulation_runtime(
        result_json=tmp_path / "dashboard-store-result.json",
        duration_sec=0.1,
        robot_model="df_mid",
        transport_factory=lambda _descriptor: RecordingV2Transport(),
        dashboard_snapshot_store=store,
    )

    snapshot = store.snapshot()
    assert snapshot is not None
    assert snapshot.wheel_state is not None
    assert snapshot.lidar_timestamp_ns is not None
    assert snapshot.lidar_sequence is not None
    assert snapshot.lidar_point_count is None
    assert snapshot.rtk is not None
    assert snapshot.imu is not None
    assert result["wall_duration_sec"] >= 0.09


@pytest.mark.parametrize("terrain_model", ("flat", "slope", "golf_heightfield"))
@pytest.mark.parametrize("robot_model", robot_model_names())
def test_v2_direct_entry_runs_each_supported_terrain_without_changing_five_topic_contract(
    tmp_path: Path,
    terrain_model: str,
    robot_model: str,
) -> None:
    """B2 正式入口必须覆盖四车型三场地，不能悄然退回默认世界。"""
    from scripts import stage4_v2_simulation_runtime as entry

    result = entry.run_v2_simulation_runtime(
        result_json=tmp_path / f"{terrain_model}-result.json",
        duration_sec=0.1,
        robot_model=robot_model,
        terrain_model=terrain_model,
        transport_factory=lambda _descriptor: RecordingV2Transport(),
    )

    assert result["terrain_model"] == terrain_model
    assert result["published_frames"] == {
        "/sim/wheel/state": 10,
        "/sim/lidar/points": 1,
        "/sim/rtk/state": 1,
        "/sim/imu/attitude": 1,
    }


@pytest.mark.parametrize("terrain_model", ("flat", "slope", "golf_heightfield"))
@pytest.mark.parametrize("robot_model", robot_model_names())
def test_v2_direct_entry_sustains_five_second_headless_five_topic_window(
    tmp_path: Path,
    terrain_model: str,
    robot_model: str,
) -> None:
    """四车型三场地必须在实时 5 秒窗口无丢帧地完成 240/100/10 Hz 发布。"""
    from scripts import stage4_v2_simulation_runtime as entry

    transport = VerifiedPeerTransport()
    result = entry.run_v2_simulation_runtime(
        result_json=tmp_path / f"{robot_model}-{terrain_model}-5s-result.json",
        duration_sec=5.0,
        robot_model=robot_model,
        terrain_model=terrain_model,
        transport_factory=lambda _descriptor: transport,
        require_verified_peers=True,
        peer_timeout_sec=2.0,
    )

    expected_counts = {
        "/sim/wheel/state": 500,
        "/sim/lidar/points": 50,
        "/sim/rtk/state": 50,
        "/sim/imu/attitude": 50,
    }
    assert result["physics_steps"] == 1200
    assert result["sim_duration_sec"] == pytest.approx(5.0)
    assert result["sim_duration_sec"] / result["wall_duration_sec"] >= 0.95
    assert result["published_frames"] == expected_counts
    assert {
        topic: sum(1 for published_topic, *_rest in transport.published if published_topic == topic)
        for topic in expected_counts
    } == expected_counts
    assert transport.snapshot().dropped_count == 0
    assert result["transport_metrics"] == {
        "published_count": 650,
        "error_count": 0,
        "dropped_count": 0,
    }
    assert result["lidar_worker"]["clean_shutdown"] is True
    assert transport.idle_waits == [2.0]
    assert transport.closed is True
