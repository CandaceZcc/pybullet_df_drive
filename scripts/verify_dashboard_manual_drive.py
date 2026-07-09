# Dashboard 手动验收脚本：打开 GUI，操作 Dashboard 控制区，并用日志确认车辆移动。
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

import pandas as pd


DASHBOARD_WINDOW_TITLE = "Step 2 Telemetry Dashboard"
DEFAULT_MANUAL_PREFIX = "manual_dam_slope_tracked_proxy_10_"


@dataclass(frozen=True)
class WindowGeometry:
    """xdotool 读取到的窗口几何信息。"""

    x: int
    y: int
    width: int
    height: int


@dataclass(frozen=True)
class ManualMotionSummary:
    """一次手动 GUI 日志的移动摘要。"""

    log_path: Path
    dx: float
    max_command_linear_velocity: float
    tail_body_forward_speed: float
    max_body_forward_speed: float
    out_of_bounds: bool


def display_is_available(env: Mapping[str, str] | None = None) -> bool:
    """判断当前 shell 是否显式配置了可访问的图形显示变量。"""
    values = os.environ if env is None else env
    return bool(values.get("DISPLAY") or values.get("WAYLAND_DISPLAY"))


def dashboard_up_button_point(geometry: WindowGeometry) -> tuple[int, int]:
    """按当前 Dashboard 布局估算上箭头按钮中心点。"""
    return geometry.x + geometry.width - 136, geometry.y + geometry.height - 175


def dashboard_window_ids(search_output: str) -> list[str]:
    """从 xdotool search 输出中提取真实窗口 id，忽略调试行。"""
    return [line.strip() for line in search_output.splitlines() if line.strip().isdigit()]


def newest_manual_log(log_dir: Path, *, prefix: str = DEFAULT_MANUAL_PREFIX, after: float = 0.0) -> Path | None:
    """找到脚本启动后生成的最新手动日志。"""
    candidates = [path for path in log_dir.glob(f"{prefix}*.csv") if path.stat().st_mtime >= after]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.name)


def summarize_manual_motion(log_path: Path, tail_samples: int = 60) -> ManualMotionSummary:
    """从手动日志中提取位移、命令和尾段速度，作为验收依据。"""
    frame = pd.read_csv(log_path)
    if frame.empty:
        raise ValueError(f"manual log is empty: {log_path}")
    required = {"x", "command_linear_velocity", "body_forward_speed", "out_of_bounds"}
    missing = required - set(frame.columns)
    if missing:
        raise ValueError(f"manual log missing columns: {', '.join(sorted(missing))}")
    tail = frame.tail(min(tail_samples, len(frame)))
    return ManualMotionSummary(
        log_path=log_path,
        dx=float(frame["x"].iloc[-1] - frame["x"].iloc[0]),
        max_command_linear_velocity=float(frame["command_linear_velocity"].abs().max()),
        tail_body_forward_speed=float(tail["body_forward_speed"].mean()),
        max_body_forward_speed=float(frame["body_forward_speed"].abs().max()),
        out_of_bounds=bool(frame["out_of_bounds"].any()),
    )


def motion_passed(
    summary: ManualMotionSummary,
    *,
    min_dx: float = 0.08,
    min_command: float = 0.20,
    min_peak_speed: float = 0.08,
) -> bool:
    """判断 Dashboard 操作是否已经让车辆产生可见移动。"""
    return (
        summary.dx >= min_dx
        and summary.max_command_linear_velocity >= min_command
        and summary.max_body_forward_speed >= min_peak_speed
        and not summary.out_of_bounds
    )


def parse_geometry(output: str) -> WindowGeometry:
    """解析 xdotool getwindowgeometry --shell 的输出。"""
    values: dict[str, int] = {}
    for line in output.splitlines():
        if "=" not in line:
            continue
        key, raw_value = line.split("=", 1)
        if key in {"X", "Y", "WIDTH", "HEIGHT"}:
            values[key] = int(raw_value)
    return WindowGeometry(
        x=values["X"],
        y=values["Y"],
        width=values["WIDTH"],
        height=values["HEIGHT"],
    )


def _run_xdotool(args: Sequence[str]) -> subprocess.CompletedProcess[str]:
    """调用 xdotool，并把失败信息保留给调用方打印。"""
    return subprocess.run(["xdotool", *args], check=True, text=True, capture_output=True)


def _find_dashboard_window(timeout_sec: float) -> str:
    """等待 Dashboard 窗口出现，并返回第一个窗口 id。"""
    deadline = time.monotonic() + timeout_sec
    last_error = ""
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["xdotool", "search", "--name", DASHBOARD_WINDOW_TITLE],
            text=True,
            capture_output=True,
        )
        ids = dashboard_window_ids(result.stdout)
        if result.returncode == 0 and ids:
            return ids[-1]
        last_error = result.stderr.strip()
        time.sleep(0.2)
    raise RuntimeError(f"Dashboard window not found: {last_error}")


def _click_dashboard_up(window_id: str, hold_sec: float) -> None:
    """把鼠标移到 Dashboard 上箭头按钮位置，并按住一小段时间。"""
    geometry_output = _run_xdotool(["getwindowgeometry", "--shell", window_id]).stdout
    geometry = parse_geometry(geometry_output)
    x, y = dashboard_up_button_point(geometry)
    _run_xdotool(["windowactivate", window_id])
    _run_xdotool(["mousemove", str(x), str(y)])
    _run_xdotool(["mousedown", "1"])
    time.sleep(hold_sec)
    _run_xdotool(["mouseup", "1"])


def _send_dashboard_up_key(window_id: str, hold_sec: float) -> None:
    """激活 Dashboard 窗口并发送方向键，由 Dashboard 的按键过滤器接收。"""
    _run_xdotool(["windowactivate", window_id])
    _run_xdotool(["keydown", "Up"])
    time.sleep(hold_sec)
    _run_xdotool(["keyup", "Up"])


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析 Dashboard 屏幕验收参数。"""
    parser = argparse.ArgumentParser(description="Verify dashboard manual control moves the tracked school-bus proxy.")
    parser.add_argument("--config", default="configs/step4_dam_gui.yaml", help="GUI manual config to run.")
    parser.add_argument("--duration-sec", type=float, default=4.0, help="Manual run duration.")
    parser.add_argument("--hold-sec", type=float, default=2.0, help="How long to hold the dashboard up button.")
    parser.add_argument("--input-method", choices=["key", "button"], default="key", help="Dashboard screen control method.")
    parser.add_argument("--window-timeout-sec", type=float, default=12.0, help="Seconds to wait for dashboard window.")
    parser.add_argument("--log-dir", type=Path, default=Path("results/logs"), help="Manual log directory.")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """启动 GUI 手动模式，点击 Dashboard 上箭头，并按日志结果返回验收状态。"""
    args = parse_args(argv)
    if not display_is_available():
        print("No DISPLAY/WAYLAND_DISPLAY is available; run this from the desktop session.", file=sys.stderr)
        return 2
    if shutil.which("xdotool") is None:
        print("xdotool is required for screen control.", file=sys.stderr)
        return 2

    started_at = time.time()
    command = [
        sys.executable,
        "main.py",
        "--config",
        args.config,
        "--gui",
        "--manual",
        "--duration-sec",
        str(args.duration_sec),
    ]
    process = subprocess.Popen(command)
    try:
        window_id = _find_dashboard_window(args.window_timeout_sec)
        if args.input_method == "button":
            _click_dashboard_up(window_id, args.hold_sec)
        else:
            _send_dashboard_up_key(window_id, args.hold_sec)
        return_code = process.wait(timeout=args.duration_sec + 8.0)
        if return_code != 0:
            print(f"manual GUI process exited with {return_code}", file=sys.stderr)
            return return_code
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5.0)

    log_path = newest_manual_log(args.log_dir, after=started_at)
    if log_path is None:
        print("No new manual log was generated.", file=sys.stderr)
        return 1
    summary = summarize_manual_motion(log_path)
    print(
        "dashboard_manual_motion "
        f"log={summary.log_path} "
        f"dx={summary.dx:.4f} "
        f"max_cmd={summary.max_command_linear_velocity:.4f} "
        f"tail_v={summary.tail_body_forward_speed:.4f} "
        f"max_v={summary.max_body_forward_speed:.4f} "
        f"out_of_bounds={summary.out_of_bounds}"
    )
    return 0 if motion_passed(summary) else 1


if __name__ == "__main__":
    raise SystemExit(main())
