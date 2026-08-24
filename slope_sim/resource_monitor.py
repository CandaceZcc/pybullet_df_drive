"""阶段五资源页的低频 Linux 进程采样，不依赖额外运行时包。"""
from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
import os
import time


@dataclass(frozen=True, slots=True)
class ProcessUsage:
    """单个进程从 /proc 读取的累计 CPU 时间、RSS 和调度状态。"""

    cpu_seconds: float
    rss_bytes: int
    state: str


@dataclass(frozen=True, slots=True)
class ResourceProcessSnapshot:
    """资源页显示的单进程即时采样。"""

    name: str
    pid: int
    cpu_percent: float | None
    rss_bytes: int
    state: str


@dataclass(frozen=True, slots=True)
class ResourceSnapshot:
    """一次 1 Hz 资源页快照。"""

    captured_at: float
    processes: tuple[ResourceProcessSnapshot, ...]
    metrics: tuple[tuple[str, str], ...] = ()


def read_process_usage(pid: int) -> ProcessUsage | None:
    """读取 Linux /proc 指标；退出中的进程或非 Linux 环境返回 None。"""
    if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
        raise ValueError("pid must be a positive int")
    try:
        stat = (Path("/proc") / str(pid) / "stat").read_text(encoding="utf-8")
        status = (Path("/proc") / str(pid) / "status").read_text(encoding="utf-8")
    except OSError:
        return None
    right_parenthesis = stat.rfind(")")
    if right_parenthesis < 0:
        return None
    fields = stat[right_parenthesis + 2 :].split()
    if len(fields) <= 12:
        return None
    try:
        ticks = os.sysconf("SC_CLK_TCK")
        cpu_seconds = (int(fields[11]) + int(fields[12])) / float(ticks)
        rss_kb = next(
            int(line.split()[1])
            for line in status.splitlines()
            if line.startswith("VmRSS:")
        )
    except (IndexError, StopIteration, ValueError, OSError):
        return None
    return ProcessUsage(cpu_seconds, rss_kb * 1024, fields[0])


def read_path_size(path: Path) -> int | None:
    """读取单个日志文件或当前采集目录的字节数；不可读路径不阻塞会话。"""
    if not isinstance(path, Path):
        raise ValueError("path must be a Path")
    try:
        if path.is_file():
            return path.stat().st_size
        if path.is_dir():
            return sum(child.stat().st_size for child in path.rglob("*") if child.is_file())
    except OSError:
        return None
    return None


def _format_bytes(size_bytes: int) -> str:
    return f"{size_bytes / 1024.0:.1f} KiB"


class ResourceMonitor:
    """为 Dashboard 生成 1 Hz 主/子进程资源快照。"""

    def __init__(
        self,
        *,
        main_pid: int | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        read_usage: Callable[[int], ProcessUsage | None] = read_process_usage,
        read_path_size: Callable[[Path], int | None] = read_path_size,
        interval_sec: float = 1.0,
    ) -> None:
        selected_pid = os.getpid() if main_pid is None else main_pid
        if isinstance(selected_pid, bool) or not isinstance(selected_pid, int) or selected_pid <= 0:
            raise ValueError("main_pid must be a positive int")
        if not callable(monotonic) or not callable(read_usage) or not callable(read_path_size):
            raise ValueError("monotonic, read_usage, and read_path_size must be callable")
        if interval_sec <= 0:
            raise ValueError("interval_sec must be positive")
        self._main_pid = selected_pid
        self._monotonic = monotonic
        self._read_usage = read_usage
        self._read_path_size = read_path_size
        self._interval_sec = float(interval_sec)
        self._next_sample_at: float | None = None
        self._previous: dict[int, tuple[float, float]] = {}
        self._previous_sizes: dict[Path, tuple[int, float]] = {}

    def sample(
        self,
        *,
        children: Mapping[str, int] | None = None,
        metrics: Mapping[str, str] | Callable[[], Mapping[str, str]] | None = None,
        storage_paths: Mapping[str, Path] | None = None,
    ) -> ResourceSnapshot | None:
        """到期时读取当前主进程与已知子进程；不到期不访问 /proc。"""
        now = float(self._monotonic())
        if self._next_sample_at is not None and now < self._next_sample_at:
            return None
        selected_children = {} if children is None else children
        selected_metrics = {} if metrics is None else metrics() if callable(metrics) else metrics
        if not isinstance(selected_metrics, Mapping):
            raise ValueError("metrics must be a mapping, callable, or None")
        selected_storage_paths = {} if storage_paths is None else storage_paths
        selected = (("Python 主进程", self._main_pid), *selected_children.items())
        processes: list[ResourceProcessSnapshot] = []
        current: dict[int, tuple[float, float]] = {}
        for name, pid in selected:
            if not isinstance(name, str) or not name:
                raise ValueError("child name must be nonempty")
            usage = self._read_usage(pid)
            if usage is None:
                continue
            previous = self._previous.get(pid)
            cpu_percent = None
            if previous is not None and now > previous[1]:
                cpu_percent = (usage.cpu_seconds - previous[0]) / (now - previous[1]) * 100.0
            processes.append(ResourceProcessSnapshot(name, pid, cpu_percent, usage.rss_bytes, usage.state))
            current[pid] = (usage.cpu_seconds, now)
        self._previous = current
        storage_metrics: list[tuple[str, str]] = []
        current_sizes: dict[Path, tuple[int, float]] = {}
        for name, path in selected_storage_paths.items():
            if not isinstance(name, str) or not name or not isinstance(path, Path):
                raise ValueError("storage path name and path must be valid")
            size = self._read_path_size(path)
            if size is None:
                continue
            if isinstance(size, bool) or not isinstance(size, int) or size < 0:
                raise ValueError("read_path_size must return a nonnegative int or None")
            previous_size = self._previous_sizes.get(path)
            if name == "CSV 日志":
                rate = "--"
                if previous_size is not None and now > previous_size[1]:
                    rate = _format_bytes(round((size - previous_size[0]) / (now - previous_size[1]))) + "/s"
                storage_metrics.append((name, f"{rate} | {_format_bytes(size)}"))
            else:
                storage_metrics.append((name, _format_bytes(size)))
            current_sizes[path] = (size, now)
        self._previous_sizes = current_sizes
        self._next_sample_at = now + self._interval_sec
        return ResourceSnapshot(now, tuple(processes), (*selected_metrics.items(), *storage_metrics))
