"""阶段四诊断：验证非 eCAL 回归中原生 eCAL 初始化追踪的完整性。"""
from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path
import re


_PID_EVENT = re.compile(
    r"^(sitecustomize|hook_installed|hook_install_failed|ecal_unavailable) pid=(\d+)(?:\s|$)"
)


def verify_trace(trace_path: Path, *, require_zero_initialize: bool) -> dict[str, int]:
    """要求每次钩子加载均成功安装，才接受零原生 initialize 结论。"""
    lines = trace_path.read_text(encoding="utf-8").splitlines()
    loaded: Counter[str] = Counter()
    installed: Counter[str] = Counter()
    unavailable: Counter[str] = Counter()
    failures: list[str] = []
    initialize_calls = 0

    for line in lines:
        match = _PID_EVENT.match(line)
        if match:
            event, pid = match.groups()
            if event == "sitecustomize":
                loaded[pid] += 1
            elif event == "hook_installed":
                installed[pid] += 1
            elif event == "ecal_unavailable":
                unavailable[pid] += 1
            else:
                failures.append(line)
        elif line.startswith("initialize args"):
            initialize_calls += 1

    if failures:
        raise ValueError("eCAL init trace contains hook_install_failed")
    if not loaded:
        raise ValueError("eCAL init trace contains no sitecustomize records")
    if loaded != installed + unavailable:
        raise ValueError("eCAL init trace has missing hook_installed records")
    if require_zero_initialize and initialize_calls:
        raise ValueError("eCAL init trace contains native initialize calls")
    return {
        "hook_loads": sum(loaded.values()),
        "unique_pids": len(loaded),
        "ecal_unavailable": sum(unavailable.values()),
        "initialize_calls": initialize_calls,
    }


def main() -> int:
    """以命令行复核一次已落盘追踪，供阶段四交付证据使用。"""
    parser = argparse.ArgumentParser(description="Verify Stage 4 eCAL init trace.")
    parser.add_argument("--trace", type=Path, required=True)
    parser.add_argument("--require-zero-initialize", action="store_true")
    args = parser.parse_args()
    try:
        summary = verify_trace(
            args.trace.resolve(), require_zero_initialize=args.require_zero_initialize
        )
    except (OSError, ValueError) as error:
        print(f"FAIL: {error}")
        return 1
    print(
        "PASS: "
        f"hook_loads={summary['hook_loads']} "
        f"unique_pids={summary['unique_pids']} "
        f"ecal_unavailable={summary['ecal_unavailable']} "
        f"initialize_calls={summary['initialize_calls']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
