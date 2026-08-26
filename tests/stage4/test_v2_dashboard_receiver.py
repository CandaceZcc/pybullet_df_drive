"""阶段五：Dashboard 必须通过独立 eCAL participant 接收真实 v2 数据。"""
from __future__ import annotations

from importlib import import_module
from types import SimpleNamespace


class _FakeSubscription:
    def close(self) -> None:
        return None


class _FakeTransport:
    """仅模拟 receive callback；不解析或重编码原始 payload。"""

    def __init__(self) -> None:
        self.callbacks: dict[str, object] = {}

    def subscribe(self, topic: str, _type_name: str, callback: object) -> _FakeSubscription:
        self.callbacks[topic] = callback
        return _FakeSubscription()

    def emit(self, topic: str, payload: bytes, received_at: float = 1.0) -> None:
        callback = self.callbacks[topic]
        callback(payload, received_at)


def _frames(descriptor, *, world_generation: int = 3, sequence: int = 7):
    """构造同一采样时刻的真实 v2 wire payload。"""
    codec = import_module("slope_sim.interfaces.v2.codec").V2ProtoCodec(descriptor)
    models = import_module("slope_sim.interfaces.v2.models")
    session = bytes.fromhex("00112233445566778899aabbccddeeff")
    timestamp_ns = 100_000_000
    lidar = models.LidarPointCloudV2(
        timestamp_ns, "lidar_link", 1, 1,
        (models.LidarPointV2(0, 1.0, 2.0, 3.0, 100, 1, 0),),
        sequence, world_generation, session, descriptor.sha256,
    )
    rtk = models.RtkStateV2(
        timestamp_ns, sequence, world_generation, "world",
        models.Point3dV2(0.0, 0.2, 0.4),
        models.Point3dV2(0.0, 0.0, 0.4),
        models.Point3dV2(0.0, -0.2, 0.4),
        0.0, session, descriptor.sha256,
    )
    imu = models.ImuAttitudeV2(
        timestamp_ns, 0.0, 0.0, sequence, world_generation, "base_link", session, descriptor.sha256,
    )
    return {
        "/sim/lidar/points": codec.encode(lidar).payload,
        "/sim/rtk/state": codec.encode(rtk).payload,
        "/sim/imu/attitude": codec.encode(imu).payload,
    }


def test_dashboard_receiver_replaces_stale_lidar_before_worker_decodes() -> None:
    """回调只复制最新 LiDAR bytes；worker 才解码并发布同刻三传感器快照。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    module = import_module("slope_sim.interfaces.v2.dashboard_receiver")
    transport = _FakeTransport()
    receiver = module.V2DashboardEcalReceiver(
        descriptor, transport=transport, start_worker=False,
    )
    frames = _frames(descriptor)
    try:
        transport.emit("/sim/lidar/points", frames["/sim/lidar/points"])
        transport.emit("/sim/lidar/points", frames["/sim/lidar/points"])

        assert receiver.snapshot_store.snapshot() is None
        assert receiver.render_dropped_count == 1

        transport.emit("/sim/rtk/state", frames["/sim/rtk/state"])
        transport.emit("/sim/imu/attitude", frames["/sim/imu/attitude"])
        assert receiver.process_pending() == 3

        snapshot = receiver.snapshot_store.snapshot()
        assert snapshot is not None
        assert snapshot.lidar_sequence == 7
        assert snapshot.lidar_point_count == 1
        assert snapshot.rtk is not None and snapshot.rtk.sequence == 7
        assert snapshot.imu is not None and snapshot.imu.sequence == 7
        cloud = receiver.cloud_frame()
        assert cloud is not None
        assert cloud.world_generation == 3
        assert cloud.positions.tolist() == [[1.0, 2.0, 3.325000047683716]]
        assert cloud.positions.flags.writeable is False

        transport.emit("/sim/lidar/points", frames["/sim/lidar/points"], received_at=2.0)
        assert receiver.process_pending() == 1
        snapshot = receiver.snapshot_store.snapshot()
        assert snapshot is not None
        assert snapshot.topic_observation("/sim/lidar/points").error_count == 1
    finally:
        receiver.close()


def test_dashboard_receiver_projects_observer_rejection_separately_from_authority() -> None:
    """Dashboard observer 的解析失败只能计入 observer 域并保留有界原因。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    module = import_module("slope_sim.interfaces.v2.dashboard_receiver")
    transport = _FakeTransport()
    receiver = module.V2DashboardEcalReceiver(
        descriptor, transport=transport, start_worker=False,
    )
    try:
        frames = _frames(descriptor)
        transport.emit("/sim/lidar/points", frames["/sim/lidar/points"])
        transport.emit("/sim/rtk/state", frames["/sim/rtk/state"])
        transport.emit("/sim/imu/attitude", frames["/sim/imu/attitude"])
        assert receiver.process_pending() == 3
        transport.emit("/sim/rtk/state", b"not-a-v2-message", received_at=2.5)
        assert receiver.process_pending() == 1

        snapshot = receiver.snapshot_store.snapshot()
        assert snapshot is not None
        assert snapshot.authority_rejections == ()
        assert len(snapshot.observer_rejections) == 1
        rejection = snapshot.observer_rejections[0]
        assert rejection.topic == "/sim/rtk/state"
        assert rejection.source_id is None
        assert rejection.sequence is None
        assert rejection.simulation_session_id is None
        assert rejection.world_generation is None
        assert "ValueError" in rejection.reason
        assert rejection.received_at == 2.5
        observation = snapshot.topic_observation("/sim/rtk/state")
        assert observation.authority_error_count == 0
        assert observation.observer_error_count == 1
    finally:
        receiver.close()


def test_dashboard_receiver_accepts_newer_world_generation_and_rejects_stale_frames() -> None:
    """结构重建会重置各 topic 序号；同会话的更高 generation 必须重新建快照。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    module = import_module("slope_sim.interfaces.v2.dashboard_receiver")
    transport = _FakeTransport()
    receiver = module.V2DashboardEcalReceiver(
        descriptor, transport=transport, start_worker=False,
    )
    try:
        first = _frames(descriptor, world_generation=3, sequence=7)
        for topic, payload in first.items():
            transport.emit(topic, payload)
        assert receiver.process_pending() == 3

        rebuilt = _frames(descriptor, world_generation=4, sequence=1)
        for topic, payload in rebuilt.items():
            transport.emit(topic, payload, received_at=2.0)
        assert receiver.process_pending() == 3
        snapshot = receiver.snapshot_store.snapshot()
        assert snapshot is not None
        assert snapshot.lidar_sequence == 1
        assert snapshot.rtk is not None and snapshot.rtk.sequence == 1
        assert snapshot.imu is not None and snapshot.imu.sequence == 1

        transport.emit("/sim/lidar/points", first["/sim/lidar/points"], received_at=3.0)
        assert receiver.process_pending() == 1
        snapshot = receiver.snapshot_store.snapshot()
        assert snapshot is not None
        assert snapshot.topic_observation("/sim/lidar/points").error_count == 1
    finally:
        receiver.close()


def test_raw_dashboard_observer_uses_existing_core_and_gates_wire_metadata() -> None:
    """观察者只附着既有 eCAL core，错误 wire 元数据不能进入 receiver。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    module = import_module("slope_sim.interfaces.v2.dashboard_receiver")
    received: list[tuple[bytes, float]] = []

    class FakeSubscriber:
        def __init__(self, callback):
            self.callback = callback
            self.closed = False

        def remove_receive_callback(self):
            self.closed = True

    class FakeRawBindings:
        def __init__(self):
            self.subscribers = []

        def create_subscriber(self, _topic, _type_name, _descriptor, callback):
            subscriber = FakeSubscriber(callback)
            self.subscribers.append(subscriber)
            return subscriber

    bindings = FakeRawBindings()
    observer = module.V2DashboardRawObserverTransport(
        descriptor,
        raw_bindings=bindings,
        start_worker=False,
    )
    subscription = observer.subscribe(
        "/sim/wheel/state",
        "slope_sim.interfaces.v2.WheelState",
        lambda payload, received_at: received.append((payload, received_at)),
    )
    transport_errors: list[tuple[str, str]] = []
    observer.set_diagnostic_callback(
        lambda topic, detail: transport_errors.append((topic, detail))
    )
    bindings.subscribers[0].callback(SimpleNamespace(
        payload=b"valid",
        received_at=2.5,
        remote_type_name="slope_sim.interfaces.v2.WheelState",
        remote_encoding="proto",
        remote_descriptor=descriptor.serialized_file_descriptor_set,
    ))
    assert observer.process_pending() == 1
    assert received == [(b"valid", 2.5)]

    bindings.subscribers[0].callback(SimpleNamespace(
        payload=b"wrong",
        received_at=3.0,
        remote_type_name="wrong.type",
        remote_encoding="proto",
        remote_descriptor=descriptor.serialized_file_descriptor_set,
    ))
    assert observer.process_pending() == 1
    assert received == [(b"valid", 2.5)]
    assert observer.diagnostics == (
        "/sim/wheel/state: remote type name does not match v2 contract",
    )
    assert transport_errors == [
        ("/sim/wheel/state", "remote type name does not match v2 contract"),
    ]

    subscription.close()
    assert not bindings.subscribers[0].closed
    observer.close()


def test_raw_dashboard_observer_defers_empty_metadata_without_protocol_error() -> None:
    """启动暂态 metadata 不交付也不计错，完整不一致仍标记 conflict。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    module = import_module("slope_sim.interfaces.v2.dashboard_receiver")
    received: list[tuple[bytes, float]] = []

    class FakeRawBindings:
        def __init__(self) -> None:
            self.callback = None

        def create_subscriber(self, _topic, _type_name, _descriptor, callback):
            self.callback = callback
            return object()

    bindings = FakeRawBindings()
    observer = module.V2DashboardRawObserverTransport(
        descriptor, raw_bindings=bindings, start_worker=False,
    )
    observer.subscribe(
        "/sim/wheel/state", "slope_sim.interfaces.v2.WheelState",
        lambda payload, received_at: received.append((payload, received_at)),
    )
    transport_errors: list[tuple[str, str]] = []
    observer.set_diagnostic_callback(
        lambda topic, detail: transport_errors.append((topic, detail))
    )
    try:
        bindings.callback(SimpleNamespace(
            payload=b"startup", received_at=1.0,
            remote_type_name="", remote_encoding="", remote_descriptor=b"",
        ))
        assert observer.process_pending() == 1
        metadata = observer.metadata_snapshot("/sim/wheel/state")
        assert metadata.endpoint_state == "pending"
        assert metadata.expected_type_name == "slope_sim.interfaces.v2.WheelState"
        assert metadata.actual_type_name == ""
        assert received == []
        assert observer.diagnostics == ()
        assert transport_errors == []

        bindings.callback(SimpleNamespace(
            payload=b"verified", received_at=2.0,
            remote_type_name="slope_sim.interfaces.v2.WheelState",
            remote_encoding="proto",
            remote_descriptor=descriptor.serialized_file_descriptor_set,
        ))
        assert observer.process_pending() == 1
        assert observer.metadata_snapshot("/sim/wheel/state").endpoint_state == "verified"
        assert received == [(b"verified", 2.0)]

        bindings.callback(SimpleNamespace(
            payload=b"conflict", received_at=3.0,
            remote_type_name="slope_sim.interfaces.v1.WheelState",
            remote_encoding="proto",
            remote_descriptor=descriptor.serialized_file_descriptor_set,
        ))
        assert observer.process_pending() == 1
        assert observer.metadata_snapshot("/sim/wheel/state").endpoint_state == "conflict"
        assert len(observer.diagnostics) == 1
        assert len(transport_errors) == 1
        assert received == [(b"verified", 2.0)]
    finally:
        observer.close()


def test_raw_dashboard_observer_detaches_logically_without_closing_native_subscriber() -> None:
    """observer 不拥有 eCAL core，关闭只能停用回调，不能同步阻塞 native 注销。"""
    descriptor = import_module("slope_sim.interfaces.v2.descriptor").load_v2_descriptor()
    module = import_module("slope_sim.interfaces.v2.dashboard_receiver")

    class FakeSubscriber:
        def __init__(self) -> None:
            self.remove_calls = 0

        def remove_receive_callback(self) -> None:
            self.remove_calls += 1

    class FakeRawBindings:
        def __init__(self) -> None:
            self.subscriber = FakeSubscriber()

        def create_subscriber(self, _topic, _type_name, _descriptor, _callback):
            return self.subscriber

    bindings = FakeRawBindings()
    observer = module.V2DashboardRawObserverTransport(
        descriptor,
        raw_bindings=bindings,
        start_worker=False,
    )
    subscription = observer.subscribe(
        "/sim/wheel/state",
        "slope_sim.interfaces.v2.WheelState",
        lambda _payload, _received_at: None,
    )

    subscription.close()
    observer.close()

    assert bindings.subscriber.remove_calls == 0


def test_manual_dashboard_refresh_uses_ecal_receiver_store_and_cloud() -> None:
    """正式 GUI 刷新必须消费 eCAL receiver，不以进程内 snapshot 冒充连通性。"""
    module = import_module("slope_sim.manual_demo")
    cloud = object()
    receiver_store = object()

    class Receiver:
        snapshot_store = receiver_store
        render_dropped_count = 3
        diagnostics = ("/sim/lidar/points: protocol error",)

        @staticmethod
        def cloud_frame():
            return cloud

    class Widget:
        def __init__(self):
            self.store = None
            self.frames = []

        def refresh_from_store(self, store):
            self.store = store
            return True

        def update_cloud_frame(self, frame):
            self.frames.append(frame)
            return True

        def update_receiver_diagnostics(self, **metrics):
            self.metrics = metrics
            return True

    widget = Widget()
    assert module._refresh_v2_dashboard_from_receiver(widget, Receiver())
    assert widget.store is receiver_store
    assert widget.frames == [cloud]
    assert widget.metrics == {
        "render_dropped_count": 3,
        "diagnostics": ("/sim/lidar/points: protocol error",),
    }


def test_manual_dashboard_refresh_forwards_v2_snapshot_to_chart_sink() -> None:
    """eCAL receiver 的 v2 store 也必须驱动主 Dashboard 的企业图表缓存。"""
    module = import_module("slope_sim.manual_demo")
    snapshot = object()

    class Store:
        def snapshot(self):
            return snapshot

    class Receiver:
        snapshot_store = Store()
        render_dropped_count = 0
        diagnostics = ()

        @staticmethod
        def cloud_frame():
            return None

    class Widget:
        @staticmethod
        def refresh_from_store(_store):
            return True

        @staticmethod
        def update_cloud_frame(_frame):
            return True

    class ChartSink:
        def __init__(self) -> None:
            self.snapshots: list[object] = []

        def update_v2_chart_snapshot(self, value):
            self.snapshots.append(value)

    sink = ChartSink()
    assert module._refresh_v2_dashboard_from_receiver(Widget(), Receiver(), chart_sink=sink)
    assert sink.snapshots == [snapshot]


def test_v2_dashboard_refresh_cadence_limits_receiver_and_chart_updates() -> None:
    """v2 接收快照只能按 Dashboard UI 节拍同步，不能跟随物理循环。"""
    module = import_module("slope_sim.manual_demo")

    class Store:
        @staticmethod
        def snapshot():
            return "snapshot"

    class Receiver:
        snapshot_store = Store()
        render_dropped_count = 0
        diagnostics = ()

        @staticmethod
        def cloud_frame():
            return None

    class Widget:
        def __init__(self) -> None:
            self.refresh_count = 0

        def refresh_from_store(self, _store):
            self.refresh_count += 1
            return True

        @staticmethod
        def update_cloud_frame(_frame):
            return None

    class ChartSink:
        def __init__(self) -> None:
            self.snapshots: list[object] = []

        def update_v2_chart_snapshot(self, snapshot):
            self.snapshots.append(snapshot)

    widget = Widget()
    sink = ChartSink()
    last_refresh_at = None

    refreshed, last_refresh_at = module._refresh_v2_dashboard_if_due(
        widget, Receiver(), chart_sink=sink, last_refresh_at=last_refresh_at,
        now=10.0, update_hz=5.0,
    )
    assert refreshed is True
    refreshed, last_refresh_at = module._refresh_v2_dashboard_if_due(
        widget, Receiver(), chart_sink=sink, last_refresh_at=last_refresh_at,
        now=10.1, update_hz=5.0,
    )
    assert refreshed is False
    refreshed, last_refresh_at = module._refresh_v2_dashboard_if_due(
        widget, Receiver(), chart_sink=sink, last_refresh_at=last_refresh_at,
        now=10.2, update_hz=5.0,
    )

    assert refreshed is True
    assert last_refresh_at == 10.2
    assert widget.refresh_count == 2
    assert sink.snapshots == ["snapshot", "snapshot"]
