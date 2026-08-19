"""阶段四 C2：由 Recorder 结果驱动唯一 Command 的单机安全停车。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Sequence


def _new_output_path(path: Path) -> Path:
    if not path.is_absolute() or path != path.resolve() or not path.parent.is_dir() or path.exists():
        raise ValueError("supervisor result must be a new file below an existing absolute directory")
    return path


def _write_new_json(path: Path, document: dict[str, object]) -> None:
    """以排他创建和 fsync 发布 Supervisor 的最终判定。"""
    encoded = (json.dumps(document, sort_keys=True) + "\n").encode("utf-8")
    descriptor = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_CLOEXEC, 0o600)
    try:
        offset = 0
        while offset < len(encoded):
            offset += os.write(descriptor, encoded[offset:])
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _stop_process_group(process: subprocess.Popen[str]) -> tuple[bool, int | None]:
    """先发送 TERM，短暂等待后才 KILL，避免 Command 在故障后继续发布。"""
    if process.poll() is not None:
        return False, process.returncode
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=1.0)
    return True, process.returncode


def supervise_recorder_and_command(
    *,
    command_argv: Sequence[str],
    recorder_argv: Sequence[str],
    recorder_result: Path,
    supervisor_result: Path,
    timeout_sec: float,
) -> dict[str, object]:
    """在 Recorder 请求安全停车时停止唯一 Command，并留下可审计结果。"""
    if not command_argv or not recorder_argv or timeout_sec <= 0.0:
        raise ValueError("supervisor requires command, recorder, and a positive timeout")
    if not recorder_result.is_absolute():
        raise ValueError("recorder result must be an absolute path")
    _new_output_path(supervisor_result)
    recorder = subprocess.Popen(list(recorder_argv), text=True, start_new_session=True)
    command = subprocess.Popen(list(command_argv), text=True, start_new_session=True)
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline:
            if recorder_result.is_file():
                try:
                    document = json.loads(recorder_result.read_text(encoding="utf-8"))
                except (OSError, json.JSONDecodeError):
                    # Recorder 以排他文件发布，但写入中的短暂不完整内容不能抢先触发决策。
                    time.sleep(0.01)
                    continue
                if document.get("safe_stop_required") is True:
                    command_stopped, command_returncode = _stop_process_group(command)
                    result = {
                        "command_stopped": command_stopped,
                        "command_returncode": command_returncode,
                        "clean_shutdown": False,
                        "recorder_safe_stop_required": True,
                        "role": "supervisor",
                    }
                    _write_new_json(supervisor_result, result)
                    return result
                raise RuntimeError("recorder completed without a safe-stop request")
            if recorder.poll() is not None:
                raise RuntimeError("recorder exited without a result")
            time.sleep(0.01)
        raise TimeoutError("recorder did not produce a result before supervisor timeout")
    finally:
        _stop_process_group(command)
        _stop_process_group(recorder)
