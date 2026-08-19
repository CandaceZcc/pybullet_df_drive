"""阶段四 B2：v2 PySide6 Dashboard launcher 的独立 GUI/Simulator 生命周期。"""
from __future__ import annotations

import os
import hashlib
import json
from collections import Counter
from importlib import import_module
from pathlib import Path
from threading import Event, Thread as NativeThread

import pytest

from slope_sim.interfaces.transport import TransportSnapshot, TransportTopicQuality

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


class _FakeClock:
    """由 fake process 的有界 join 推进，不依赖 sleep 或真实墙钟。"""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class _FakeProcess:
    """记录监督信号，并按阶段决定 join 后是否退出。"""

    def __init__(
        self,
        clock: _FakeClock,
        *,
        natural_exit_after: int | None = None,
        terminate_stops: bool = True,
    ) -> None:
        self._clock = clock
        self._natural_exit_after = natural_exit_after
        self._terminate_stops = terminate_stops
        self._phase = "running"
        self._poll_joins = 0
        self._alive = True
        self.calls: list[tuple[str, float | None]] = []
        self.exitcode: int | None = None

    def is_alive(self) -> bool:
        return self._alive

    def join(self, timeout: float | None = None) -> None:
        self.calls.append(("join", timeout))
        if timeout is None:
            raise AssertionError("process supervision must never use an unbounded join")
        self._clock.now += timeout
        if self._phase == "running":
            self._poll_joins += 1
            if self._natural_exit_after == self._poll_joins:
                self._alive = False
                self.exitcode = 0
        elif self._phase == "terminated" and self._terminate_stops:
            self._alive = False
            self.exitcode = -15
        elif self._phase == "killed":
            self._alive = False
            self.exitcode = -9

    def terminate(self) -> None:
        self.calls.append(("terminate", None))
        self._phase = "terminated"

    def kill(self) -> None:
        self.calls.append(("kill", None))
        self._phase = "killed"


class _Signal:
    def __init__(self) -> None:
        self.callbacks: list[object] = []

    def connect(self, callback) -> None:
        self.callbacks.append(callback)

    def disconnect(self, callback) -> None:
        self.callbacks.remove(callback)

    def emit(self) -> None:
        for callback in tuple(self.callbacks):
            callback()


class _LifecycleTimer:
    latest = None

    def __init__(self) -> None:
        type(self).latest = self
        self.timeout = _Signal()

    def start(self, _interval_ms: int) -> None:
        pass

    def stop(self) -> None:
        pass


class _LifecycleApplication:
    mode = "gui_error"
    latest = None

    def __init__(self, _argv: list[object]) -> None:
        type(self).latest = self
        self.aboutToQuit = _Signal()
        self.quit_called = False

    @classmethod
    def instance(cls):
        return None

    def exec(self) -> int:
        if self.mode == "gui_error":
            _LifecycleTimer.latest.timeout.emit()
        else:
            self.aboutToQuit.emit()
        return 0

    def quit(self) -> None:
        self.quit_called = True
        self.aboutToQuit.emit()


class _LifecycleWidget:
    def __init__(self, _descriptor: object) -> None:
        self.closed = False

    def refresh_from_store(self, _store: object) -> None:
        raise RuntimeError("GUI refresh failed")

    def show(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class _StartedProcess:
    exitcode = None

    def __init__(self) -> None:
        self.started = False

    def start(self) -> None:
        self.started = True

    def join(self, timeout: float | None = None) -> None:
        raise AssertionError(f"production path bypassed supervisor with join({timeout!r})")


class _LifecycleContext:
    def __init__(self) -> None:
        self.process = _StartedProcess()

    @staticmethod
    def Queue(*, maxsize: int):
        from queue import Queue

        return Queue(maxsize=maxsize)

    def Process(self, **_kwargs: object) -> _StartedProcess:
        return self.process


class _RecordingThread:
    instances: list["_RecordingThread"] = []

    def __init__(self, *args: object, **kwargs: object) -> None:
        self._thread = NativeThread(*args, **kwargs)
        self.join_timeouts: list[float | None] = []
        type(self).instances.append(self)

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout: float | None = None) -> None:
        self.join_timeouts.append(timeout)
        if timeout is None:
            raise AssertionError("dashboard worker join must be bounded")
        self._thread.join(timeout)

    def is_alive(self) -> bool:
        return self._thread.is_alive()


class _RecordingTransport:
    """让 launcher 集成测试覆盖真实 PyBullet，但不需要启动 native eCAL participant。"""

    def __init__(self) -> None:
        self.published: list[tuple[str, bytes, str, int, float]] = []
        self.idle_waits: list[float] = []

    def subscribe(self, _topic: str, _type_name: str, _callback) -> None:
        """本测试不注入 command，正式 authority 保持安全停车。"""

    def poll_peer_state(self) -> None:
        """测试 transport 没有外部 discovery。"""

    def snapshot(self) -> TransportSnapshot:
        """模拟五话题 exact-one verified peer，覆盖正式 GUI 的 eCAL 启动门。"""
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

    def wait_idle(self, *, timeout_sec: float) -> None:
        """记录正式关闭前的 eCAL lane 排空请求。"""
        self.idle_waits.append(timeout_sec)

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
        """测试不持有外部资源。"""


def test_v2_dashboard_child_supervision_allows_natural_exit_without_signals() -> None:
    """正常子进程只接受有界轮询，绝不误发 terminate/kill。"""
    module = import_module("scripts.stage4_v2_dashboard")
    clock = _FakeClock()
    process = _FakeProcess(clock, natural_exit_after=2)

    reason = module._supervise_child_process(
        process,
        cancellation=Event(),
        deadline=10.0,
        monotonic=clock,
        poll_sec=0.25,
        stop_grace_sec=0.5,
    )

    assert reason == "exited"
    assert process.calls == [("join", 0.25), ("join", 0.25)]


def test_v2_dashboard_child_supervision_terminates_hung_child_at_deadline() -> None:
    """运行时超过硬 deadline 后必须 terminate 并用有界 join 回收。"""
    module = import_module("scripts.stage4_v2_dashboard")
    clock = _FakeClock()
    process = _FakeProcess(clock)

    reason = module._supervise_child_process(
        process,
        cancellation=Event(),
        deadline=0.5,
        monotonic=clock,
        poll_sec=0.25,
        stop_grace_sec=0.5,
    )

    assert reason == "deadline"
    assert process.calls == [
        ("join", 0.25),
        ("join", 0.25),
        ("terminate", None),
        ("join", 0.5),
    ]


def test_v2_dashboard_child_supervision_kills_after_gui_cancellation() -> None:
    """GUI 异常/关闭取消会话；terminate 无效时必须升级 kill 并继续有界等待。"""
    module = import_module("scripts.stage4_v2_dashboard")
    clock = _FakeClock()
    process = _FakeProcess(clock, terminate_stops=False)
    cancellation = Event()
    cancellation.set()

    reason = module._supervise_child_process(
        process,
        cancellation=cancellation,
        deadline=10.0,
        monotonic=clock,
        poll_sec=0.25,
        stop_grace_sec=0.5,
    )

    assert reason == "cancelled"
    assert process.calls == [
        ("terminate", None),
        ("join", 0.5),
        ("kill", None),
        ("join", 0.5),
    ]


@pytest.mark.parametrize(
    ("mode", "expected_error"),
    (("gui_error", "GUI refresh failed"), ("user_close", "cancelled")),
)
def test_v2_dashboard_isolated_runtime_cancels_child_and_bounds_worker_join(
    tmp_path: Path, monkeypatch, mode: str, expected_error: str,
) -> None:
    """Qt 异常或用户关闭都必须取消 child，且主线程不能无限等待 worker。"""
    module = import_module("scripts.stage4_v2_dashboard")
    context = _LifecycleContext()
    supervised: list[Event] = []
    _LifecycleApplication.mode = mode
    _RecordingThread.instances.clear()

    def supervise(process: object, *, cancellation: Event, **_kwargs: object) -> str:
        supervised.append(cancellation)
        assert cancellation.wait(1.0), "Qt lifecycle did not cancel the child"
        process.exitcode = -15
        return "cancelled"

    monkeypatch.setattr(module.multiprocessing, "get_context", lambda _mode: context)
    monkeypatch.setattr(module, "_supervise_child_process", supervise)
    monkeypatch.setattr(module, "V2DashboardWidget", _LifecycleWidget)
    monkeypatch.setattr(module, "Thread", _RecordingThread)
    monkeypatch.setattr("PySide6.QtCore.QTimer", _LifecycleTimer)
    monkeypatch.setattr("PySide6.QtWidgets.QApplication", _LifecycleApplication)

    with pytest.raises(RuntimeError, match=expected_error):
        module.run_v2_dashboard_session(
            result_json=tmp_path / "unused-result.json",
            duration_sec=0.1,
            isolate_runtime=True,
        )

    assert context.process.started is True
    assert len(supervised) == 1 and supervised[0].is_set()
    assert len(_RecordingThread.instances) == 1
    assert _RecordingThread.instances[0].join_timeouts
    assert all(
        timeout is not None and timeout > 0.0
        for timeout in _RecordingThread.instances[0].join_timeouts
    )


def test_v2_dashboard_launcher_runs_gui_and_background_simulator_to_clean_exit(tmp_path: Path) -> None:
    """offscreen GUI 必须在正式 Simulator 完成后自动退出，且实际渲染过共享 snapshot。"""
    module = import_module("scripts.stage4_v2_dashboard")
    transport = _RecordingTransport()
    screenshot = tmp_path / "dashboard.png"

    result = module.run_v2_dashboard_session(
        result_json=tmp_path / "dashboard-launcher-result.json",
        duration_sec=0.2,
        robot_model="df_mid",
        terrain_model="flat",
        transport_factory=lambda _descriptor: transport,
        peer_timeout_sec=10.0,
        screenshot_png=screenshot,
    )

    assert result["clean_shutdown"] is True
    assert result["dashboard_snapshot"]["lidar_sequence"] == 1
    assert result["wall_duration_sec"] >= 0.19
    assert result["published_frames"] == {
        "/sim/wheel/state": 20,
        "/sim/lidar/points": 2,
        "/sim/rtk/state": 2,
        "/sim/imu/attitude": 2,
    }
    actual_counts = Counter(topic for topic, *_rest in transport.published)
    assert actual_counts["/sim/wheel/state"] == 20
    sensor_counts = {
        actual_counts[topic]
        for topic in (
            "/sim/lidar/points",
            "/sim/rtk/state",
            "/sim/imu/attitude",
        )
    }
    assert sensor_counts in ({1}, {2})
    assert transport.idle_waits == [10.0]
    assert screenshot.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")


def test_v2_dashboard_launcher_rejects_missing_requested_complete_screenshot(tmp_path: Path) -> None:
    """请求验收截图时，运行窗口不足以形成完整遥测必须 fail-closed。"""
    module = import_module("scripts.stage4_v2_dashboard")
    transport = _RecordingTransport()
    screenshot = tmp_path / "missing-dashboard.png"

    with pytest.raises(RuntimeError, match="complete dashboard screenshot"):
        module.run_v2_dashboard_session(
            result_json=tmp_path / "short-result.json",
            duration_sec=0.01,
            transport_factory=lambda _descriptor: transport,
            screenshot_png=screenshot,
        )

    assert not screenshot.exists()


def test_v2_dashboard_launcher_accepts_only_explicit_valid_evidence_path(tmp_path: Path) -> None:
    """CLI 不提供 evidence 时正常启动语义，提供时仍只接受显式绝对路径。"""
    module = import_module("scripts.stage4_v2_dashboard")
    evidence = tmp_path / "evidence.json"
    evidence.write_text("{}", encoding="utf-8")

    no_evidence = module._parse_args(["--result-json", str(tmp_path / "result.json")])
    assert no_evidence.evidence_json is None
    with pytest.raises(ValueError, match="absolute"):
        module._parse_args([
            "--result-json", str(tmp_path / "result.json"),
            "--evidence-json", "evidence.json",
        ])
    args = module._parse_args([
        "--result-json", str(tmp_path / "result.json"),
        "--evidence-json", str(evidence.resolve()),
    ])
    assert args.evidence_json == evidence.resolve()


def test_v2_dashboard_launcher_validates_evidence_before_qapplication_or_runtime(
    tmp_path: Path, monkeypatch,
) -> None:
    """无效 evidence 必须在 Qt application 和 Simulator 产生副作用前失败。"""
    module = import_module("scripts.stage4_v2_dashboard")
    evidence = tmp_path / "invalid-evidence.json"
    evidence.write_text("{}", encoding="utf-8")
    events: list[str] = []

    def reject_evidence(_path: Path) -> dict[str, object]:
        events.append("evidence")
        raise ValueError("invalid verifier evidence")

    class _ForbiddenApplication:
        @classmethod
        def instance(cls):
            events.append("qapplication")
            raise AssertionError("QApplication must not be touched")

    monkeypatch.setattr(module, "load_offline_evidence", reject_evidence)
    monkeypatch.setattr("PySide6.QtWidgets.QApplication", _ForbiddenApplication)

    with pytest.raises(ValueError, match="invalid verifier evidence"):
        module.run_v2_dashboard_session(
            result_json=tmp_path / "unused-result.json",
            duration_sec=0.1,
            evidence_json=evidence.resolve(),
        )
    assert events == ["evidence"]


def test_v2_dashboard_cli_isolates_runtime_by_default(tmp_path: Path, monkeypatch) -> None:
    """正式 CLI 必须把 PyBullet/eCAL 放入受监管子进程，不能默认同进程运行。"""
    module = import_module("scripts.stage4_v2_dashboard")
    received: dict[str, object] = {}

    def record_session(**kwargs: object) -> dict[str, object]:
        received.update(kwargs)
        return {"clean_shutdown": True}

    monkeypatch.setattr(module, "run_v2_dashboard_session", record_session)

    assert module.main(["--result-json", str(tmp_path / "result.json")]) == 0
    assert received["isolate_runtime"] is True
