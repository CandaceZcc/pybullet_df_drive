"""阶段五资源状态页的低成本 /proc 采样合同。"""
from __future__ import annotations

from importlib import import_module
from pathlib import Path
from types import SimpleNamespace


def test_resource_monitor_reports_main_and_child_cpu_rss_at_one_hz() -> None:
    """首次采样建立 CPU 基线，下一秒才报告主/子进程的瞬时 CPU。"""
    module = import_module("slope_sim.resource_monitor")
    now = {"value": 10.0}
    usages = {
        101: module.ProcessUsage(cpu_seconds=2.0, rss_bytes=1_024, state="running"),
        202: module.ProcessUsage(cpu_seconds=1.0, rss_bytes=2_048, state="sleeping"),
    }
    monitor = module.ResourceMonitor(
        main_pid=101,
        monotonic=lambda: now["value"],
        read_usage=lambda pid: usages.get(pid),
    )

    first = monitor.sample(children={"Command": 202})
    assert first is not None
    assert [(item.name, item.cpu_percent, item.rss_bytes) for item in first.processes] == [
        ("Python 主进程", None, 1_024),
        ("Command", None, 2_048),
    ]
    assert monitor.sample(children={"Command": 202}) is None

    now["value"] = 11.0
    usages[101] = module.ProcessUsage(cpu_seconds=2.25, rss_bytes=1_536, state="running")
    usages[202] = module.ProcessUsage(cpu_seconds=1.5, rss_bytes=2_560, state="sleeping")
    second = monitor.sample(children={"Command": 202})

    assert second is not None
    assert [(item.name, item.cpu_percent, item.rss_bytes) for item in second.processes] == [
        ("Python 主进程", 25.0, 1_536),
        ("Command", 50.0, 2_560),
    ]


def test_resource_monitor_keeps_low_frequency_scalar_metrics_in_same_snapshot() -> None:
    module = import_module("slope_sim.resource_monitor")
    monitor = module.ResourceMonitor(main_pid=101, monotonic=lambda: 1.0, read_usage=lambda _pid: None)

    snapshot = monitor.sample(metrics={"日志增长": "12 KiB/s", "物理超期": "3"})

    assert snapshot.metrics == (("日志增长", "12 KiB/s"), ("物理超期", "3"))


def test_resource_monitor_evaluates_metric_supplier_only_when_sampling_is_due() -> None:
    module = import_module("slope_sim.resource_monitor")
    now = {"value": 1.0}
    calls = []
    monitor = module.ResourceMonitor(main_pid=101, monotonic=lambda: now["value"], read_usage=lambda _pid: None)

    assert monitor.sample(metrics=lambda: calls.append("sampled") or {"物理超期": "3"}) is not None
    assert monitor.sample(metrics=lambda: calls.append("too_soon") or {}) is None

    assert calls == ["sampled"]


def test_resource_monitor_reports_current_log_growth_and_capture_size() -> None:
    """日志速率和当前采集目录只在 1 Hz 采样，不进入高频遥测文件。"""
    module = import_module("slope_sim.resource_monitor")
    now = {"value": 1.0}
    log_path = Path("/session/telemetry.csv")
    capture_dir = Path("/session/capture")
    sizes = {log_path: 1_024, capture_dir: 4_096}
    monitor = module.ResourceMonitor(
        main_pid=101,
        monotonic=lambda: now["value"],
        read_usage=lambda _pid: None,
        read_path_size=lambda path: sizes.get(path),
    )

    first = monitor.sample(storage_paths={"CSV 日志": log_path, "采集目录": capture_dir})
    assert first is not None
    assert first.metrics == (("CSV 日志", "-- | 1.0 KiB"), ("采集目录", "4.0 KiB"))

    now["value"] = 2.0
    sizes.update({log_path: 3_072, capture_dir: 6_144})
    second = monitor.sample(storage_paths={"CSV 日志": log_path, "采集目录": capture_dir})

    assert second is not None
    assert second.metrics == (("CSV 日志", "2.0 KiB/s | 3.0 KiB"), ("采集目录", "6.0 KiB"))


def test_manual_demo_forwards_due_resource_snapshot_to_dashboard() -> None:
    """手动循环只转交到期快照，采样器拒绝刷新时不触碰 Qt。"""
    module = import_module("slope_sim.manual_demo")
    snapshot = object()

    class Monitor:
        def __init__(self) -> None:
            self.children = None
            self.metrics = None
            self.storage_paths = None

        def sample(self, *, children, metrics, storage_paths):
            self.children = children
            self.metrics = metrics
            self.storage_paths = storage_paths
            return snapshot

    class Dashboard:
        def __init__(self) -> None:
            self.received = []

        def update_resource_status(self, value):
            self.received.append(value)

    monitor, dashboard = Monitor(), Dashboard()

    assert module._update_resource_dashboard(
        dashboard,
        monitor,
        children={"Command": 202},
        metrics={"物理超期": "3"},
        storage_paths={"CSV 日志": Path("/session/telemetry.csv")},
    ) is True
    assert monitor.children == {"Command": 202}
    assert monitor.metrics == {"物理超期": "3"}
    assert monitor.storage_paths == {"CSV 日志": Path("/session/telemetry.csv")}
    assert dashboard.received == [snapshot]


def test_manual_demo_formats_existing_low_frequency_session_metrics() -> None:
    """资源页只转写已有只读统计，缺少 RC 时不伪造串口健康值。"""
    module = import_module("slope_sim.manual_demo")
    metrics = module._resource_scalar_metrics(
        pacer=SimpleNamespace(statistics=SimpleNamespace(overrun_count=3)),
        dashboard=SimpleNamespace(actual_update_hz=5.0),
        rc_snapshot=SimpleNamespace(
            actual_hz=99.5,
            last_frame_age_sec=0.012,
            watchdog_timeout_count=2,
        ),
    )

    assert metrics == {
        "物理超期": "3",
        "Dashboard 刷新": "5.0 Hz",
        "串口帧率": "99.5 Hz",
        "串口帧年龄": "12 ms",
        "串口 watchdog": "2",
    }
