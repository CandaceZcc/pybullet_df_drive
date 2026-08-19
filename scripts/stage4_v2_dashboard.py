"""阶段四 B2：以独立 Qt 主线程展示正式 v2 Simulator 的 Dashboard 快照。"""
from __future__ import annotations

import argparse
import json
import multiprocessing
from pathlib import Path
from queue import Empty, Full
import sys
from threading import Event, Thread
import time
from typing import Callable, Sequence


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from slope_sim.interfaces.v2.dashboard_adapter import V2DashboardWidget, load_offline_evidence
from slope_sim.interfaces.v2.dashboard_snapshot import V2DashboardSnapshotStore
from slope_sim.interfaces.v2.descriptor import DescriptorIdentity, load_v2_descriptor
from slope_sim.model_registry import robot_model_names
from slope_sim.scene import terrain_model_names
from scripts.stage4_v2_simulation_runtime import run_v2_simulation_runtime


_CHILD_POLL_SEC = 0.1
_CHILD_STOP_GRACE_SEC = 2.0
_CHILD_STARTUP_SHUTDOWN_BUDGET_SEC = 30.0
_WORKER_JOIN_TIMEOUT_SEC = 5.0


def _offer_latest_snapshot(snapshot_queue: object, snapshot: object) -> None:
    """IPC 队列满时只替换旧显示帧，绝不阻塞 Simulator 的物理/eCAL 数据面。"""
    try:
        snapshot_queue.put_nowait(snapshot)
        return
    except Full:
        pass
    try:
        snapshot_queue.get_nowait()
    except Empty:
        pass
    try:
        snapshot_queue.put_nowait(snapshot)
    except Full:
        pass


def _supervise_child_process(
    process: object,
    *,
    cancellation: Event,
    deadline: float,
    monotonic: Callable[[], float] = time.monotonic,
    poll_sec: float = _CHILD_POLL_SEC,
    stop_grace_sec: float = _CHILD_STOP_GRACE_SEC,
) -> str:
    """有界等待本会话 child；取消或超时后只回收这个受监管进程。"""
    reason = "exited"
    while process.is_alive():
        if cancellation.is_set():
            reason = "cancelled"
            break
        remaining = deadline - monotonic()
        if remaining <= 0.0:
            reason = "deadline"
            break
        process.join(timeout=min(poll_sec, remaining))
    if reason == "exited":
        return reason

    if process.is_alive():
        process.terminate()
        process.join(timeout=stop_grace_sec)
    if process.is_alive():
        process.kill()
        process.join(timeout=stop_grace_sec)
    if process.is_alive():
        raise RuntimeError("isolated dashboard runtime did not stop after kill")
    return reason


def _isolated_runtime_entry(
    snapshot_queue: object,
    result_queue: object,
    *,
    result_json: Path,
    duration_sec: float,
    robot_model: str,
    terrain_model: str,
    peer_timeout_sec: float,
) -> None:
    """子进程独占 PyBullet/eCAL；父进程只接收可丢弃的 GUI latest snapshot。"""
    try:
        from slope_sim.interfaces.v2.transport import create_v2_ecal_transport

        store = V2DashboardSnapshotStore(
            on_update=lambda snapshot: _offer_latest_snapshot(snapshot_queue, snapshot),
            robot_model=robot_model,
        )
        result = run_v2_simulation_runtime(
            result_json=result_json,
            duration_sec=duration_sec,
            robot_model=robot_model,
            terrain_model=terrain_model,
            transport_factory=lambda descriptor: create_v2_ecal_transport(
                descriptor=descriptor,
                participant_name=f"stage4-v2-runtime-{__import__('os').getpid()}",
            ),
            require_verified_peers=True,
            peer_timeout_sec=peer_timeout_sec,
            dashboard_snapshot_store=store,
        )
    except BaseException as error:
        result_queue.put(("error", type(error).__name__, str(error)))
    else:
        result_queue.put(("result", result))


def run_v2_dashboard_session(
    *,
    result_json: Path,
    duration_sec: float,
    robot_model: str = "df_mid",
    terrain_model: str = "flat",
    transport_factory: Callable[[DescriptorIdentity], object] | None = None,
    peer_timeout_sec: float = 20.0,
    screenshot_png: Path | None = None,
    isolate_runtime: bool = False,
    evidence_json: Path | None = None,
) -> dict[str, object]:
    """在后台运行正式 Simulator，并让 Qt 主线程定时消费同一份不可变快照。"""
    offline_evidence = None if evidence_json is None else load_offline_evidence(evidence_json)
    from PySide6.QtCore import QBuffer, QIODevice, QTimer
    from PySide6.QtWidgets import QApplication

    application = QApplication.instance() or QApplication([])
    store = V2DashboardSnapshotStore(robot_model=robot_model)
    widget = V2DashboardWidget(load_v2_descriptor())
    if offline_evidence is not None:
        widget.set_offline_evidence(offline_evidence)
    completed = Event()
    cancellation = Event()
    outcome: dict[str, object] = {}
    errors: list[BaseException] = []
    screenshot_written = False
    snapshot_queue = None
    result_queue = None

    if type(isolate_runtime) is not bool:
        raise ValueError("isolate_runtime must be a bool")
    if isolate_runtime and transport_factory is not None:
        raise ValueError("isolated dashboard runtime creates its own native eCAL transport")

    if screenshot_png is not None:
        if not isinstance(screenshot_png, Path):
            raise ValueError("screenshot_png must be a Path")
        screenshot_png = screenshot_png.resolve()
        if screenshot_png.exists():
            raise FileExistsError("dashboard screenshot already exists")

    def capture_completed_dashboard() -> None:
        """仅在完整五类遥测已渲染后排他落盘 PNG，作为真实桌面验收证据。"""
        nonlocal screenshot_written
        if screenshot_png is None or screenshot_written:
            return
        snapshot = store.snapshot()
        if snapshot is None or any(
            value is None
            for value in (
                snapshot.wheel_state,
                snapshot.lidar_sequence,
                snapshot.rtk,
                snapshot.imu,
            )
        ):
            return
        buffer = QBuffer()
        buffer.open(QIODevice.OpenModeFlag.WriteOnly)
        if not widget.grab().save(buffer, "PNG"):
            raise RuntimeError("dashboard screenshot capture failed")
        screenshot_png.parent.mkdir(parents=True, exist_ok=True)
        with screenshot_png.open("xb") as handle:
            handle.write(bytes(buffer.data()))
        screenshot_written = True

    def run_simulator() -> None:
        """后台线程只执行 PyBullet/transport，不调用任何 Qt 或点云 decoder。"""
        try:
            if isolate_runtime:
                nonlocal snapshot_queue, result_queue
                context = multiprocessing.get_context("spawn")
                snapshot_queue = context.Queue(maxsize=1)
                result_queue = context.Queue(maxsize=1)
                process = context.Process(
                    target=_isolated_runtime_entry,
                    kwargs={
                        "snapshot_queue": snapshot_queue,
                        "result_queue": result_queue,
                        "result_json": result_json,
                        "duration_sec": duration_sec,
                        "robot_model": robot_model,
                        "terrain_model": terrain_model,
                        "peer_timeout_sec": peer_timeout_sec,
                    },
                    daemon=False,
                )
                process.start()
                supervision = _supervise_child_process(
                    process,
                    cancellation=cancellation,
                    deadline=(
                        time.monotonic()
                        + duration_sec
                        + peer_timeout_sec
                        + _CHILD_STARTUP_SHUTDOWN_BUDGET_SEC
                    ),
                )
                if supervision == "cancelled":
                    raise RuntimeError("isolated dashboard runtime cancelled")
                if supervision == "deadline":
                    raise TimeoutError("isolated dashboard runtime exceeded supervision deadline")
                if process.exitcode != 0:
                    raise RuntimeError(f"isolated dashboard runtime exited {process.exitcode}")
                try:
                    message = result_queue.get_nowait()
                except Empty as error:
                    raise RuntimeError("isolated dashboard runtime produced no result") from error
                if message[0] == "error":
                    raise RuntimeError(f"isolated runtime failed: {message[1]}: {message[2]}")
                outcome["result"] = message[1]
            else:
                outcome["result"] = run_v2_simulation_runtime(
                    result_json=result_json,
                    duration_sec=duration_sec,
                    robot_model=robot_model,
                    terrain_model=terrain_model,
                    transport_factory=transport_factory,
                    require_verified_peers=True,
                    peer_timeout_sec=peer_timeout_sec,
                    dashboard_snapshot_store=store,
                )
        except BaseException as error:
            errors.append(error)
        finally:
            completed.set()

    worker = Thread(target=run_simulator, name="stage4-v2-simulator", daemon=False)
    timer = QTimer()
    gui_errors: list[BaseException] = []

    def refresh_dashboard() -> None:
        """Qt 主线程按定时器刷新，Simulator 完成后负责关闭窗口与事件循环。"""
        if not gui_errors:
            try:
                if snapshot_queue is not None:
                    latest = None
                    while True:
                        try:
                            latest = snapshot_queue.get_nowait()
                        except Empty:
                            break
                    if latest is not None:
                        store.update_snapshot(latest)
                widget.refresh_from_store(store)
                capture_completed_dashboard()
            except BaseException as error:
                gui_errors.append(error)
                cancellation.set()
                timer.stop()
                widget.close()
                application.quit()
                return
        if completed.is_set():
            timer.stop()
            widget.close()
            application.quit()

    timer.timeout.connect(refresh_dashboard)
    application.aboutToQuit.connect(cancellation.set)
    widget.show()
    worker.start()
    timer.start(20)
    try:
        application.exec()
    finally:
        cancellation.set()
        worker.join(timeout=_WORKER_JOIN_TIMEOUT_SEC)
        application.aboutToQuit.disconnect(cancellation.set)
    if worker.is_alive():
        raise RuntimeError("dashboard supervisor worker did not stop within its timeout")
    if gui_errors:
        raise gui_errors[0]
    if errors:
        raise errors[0]
    if screenshot_png is not None and not screenshot_written:
        raise RuntimeError("complete dashboard screenshot was not rendered")
    result = outcome.get("result")
    if type(result) is not dict:
        raise RuntimeError("v2 dashboard simulator did not produce a result")
    if offline_evidence is not None:
        result = dict(result)
        result["offline_evidence"] = {"validated": True, "kind": offline_evidence["kind"]}
    return result


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析独立 Dashboard 的单机 DIRECT 运行参数。"""
    parser = argparse.ArgumentParser(description="Run the Stage 4 v2 Dashboard")
    parser.add_argument("--result-json", type=Path, required=True)
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--robot-model", choices=robot_model_names(), default="df_mid")
    parser.add_argument("--terrain-model", choices=terrain_model_names(), default="flat")
    parser.add_argument("--evidence-json", type=Path)
    args = parser.parse_args(argv)
    if args.evidence_json is not None and not args.evidence_json.is_absolute():
        raise ValueError("evidence path must be absolute")
    if args.evidence_json is not None:
        args.evidence_json = args.evidence_json.resolve(strict=True)
    return args


def main(argv: Sequence[str] | None = None) -> int:
    """启动可见 Dashboard；失败只返回非零，不掩盖底层 Simulator 异常。"""
    args = _parse_args(argv)
    try:
        result = run_v2_dashboard_session(
            result_json=args.result_json,
            duration_sec=args.duration_sec,
            robot_model=args.robot_model,
            terrain_model=args.terrain_model,
            evidence_json=args.evidence_json,
            isolate_runtime=True,
        )
    except Exception as error:
        print(f"FAIL: {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
