#!/usr/bin/env python3
# 阶段四 Python eCAL loader 验证：只 dlopen wheel 内 core，并审计进程映射的 DSO 来源。
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import sys


_READY = "stage4-python-no-participant-loader-ready"
_CHILD_SOURCE = """\
import ctypes
import os
import sys

ctypes.CDLL(sys.argv[1], mode=os.RTLD_NOW | os.RTLD_LOCAL)
sys.stdout.write(\"stage4-python-no-participant-loader-ready\\n\")
sys.stdout.flush()
sys.stdin.read(1)
"""


def _regular_in_runtime(runtime_root: Path, path: Path, description: str) -> Path:
    """解析 runtime 内常规文件，拒绝符号链接和根目录逃逸。"""
    resolved_root = runtime_root.resolve(strict=True)
    resolved = path.resolve(strict=True)
    try:
        resolved.relative_to(resolved_root)
    except ValueError as error:
        raise ValueError(f"{description} escapes runtime root") from error
    metadata = path.lstat()
    if path.is_symlink() or not path.is_file() or not resolved.is_file():
        raise ValueError(f"{description} must be a regular file")
    if not os.access(resolved, os.R_OK):
        raise ValueError(f"{description} is not readable")
    return resolved


def _runtime_paths(runtime_root: Path) -> tuple[Path, Path]:
    """定位唯一 Python 解释器和 wheel 内唯一 eCAL core。"""
    python = _regular_in_runtime(runtime_root, runtime_root / "bin" / "python3.10", "runtime Python")
    candidates = {
        _regular_in_runtime(runtime_root, candidate, "wheel eCAL core")
        for candidate in runtime_root.glob("lib/python*/site-packages/ecal/libecal_core.so.6")
    }
    if len(candidates) != 1:
        raise ValueError("runtime must contain exactly one wheel libecal_core.so.6")
    core = next(iter(candidates))
    return python, core


def _maps(pid: int) -> list[Path]:
    """读取仍存活 loader 的 maps，只返回真实且未删除的文件映射。"""
    mapped: set[Path] = set()
    maps_path = Path("/proc") / str(pid) / "maps"
    for line in maps_path.read_text(encoding="utf-8").splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) != 6 or not fields[5].startswith("/"):
            continue
        mapped_name = fields[5]
        if mapped_name.endswith(" (deleted)"):
            raise ValueError(f"loader maps a deleted file: {mapped_name}")
        mapped.add(Path(mapped_name).resolve(strict=True))
    return sorted(mapped, key=lambda path: path.as_posix())


def _readelf(core: Path) -> dict[str, object]:
    """解析 core 的 SONAME、NEEDED 与 RUNPATH，避免文本证据无法审计。"""
    completed = subprocess.run(
        ["readelf", "-d", str(core)],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise ValueError(f"readelf failed: {completed.stderr.strip()}")
    sonames = re.findall(r"Library soname: \[([^]]+)\]", completed.stdout)
    if sonames != ["libecal_core.so.6"]:
        raise ValueError("wheel eCAL core SONAME is invalid")
    needed = re.findall(r"Shared library: \[([^]]+)\]", completed.stdout)
    runpaths = re.findall(r"Library (?:runpath|rpath): \[([^]]+)\]", completed.stdout)
    if any(name.startswith("libprotobuf.so") for name in needed):
        raise ValueError("wheel eCAL core must not need a bundled libprotobuf")
    return {"soname": sonames[0], "needed": needed, "runpaths": runpaths}


def _ldd(core: Path) -> dict[str, object]:
    """记录核心 DSO 的解析结果，并拒绝未解析的动态依赖。"""
    completed = subprocess.run(
        ["ldd", str(core)],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise ValueError(f"ldd failed: {completed.stderr.strip()}")
    resolved: list[str] = []
    unresolved: list[str] = []
    for line in completed.stdout.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("linux-vdso.so"):
            continue
        if "=> not found" in stripped:
            unresolved.append(stripped.split("=>", 1)[0].strip())
            continue
        if "=>" in stripped:
            target = stripped.split("=>", 1)[1].strip().split(" ", 1)[0]
        else:
            target = stripped.split(" ", 1)[0]
        if target.startswith("/"):
            resolved.append(str(Path(target).resolve(strict=True)))
    if unresolved:
        raise ValueError("wheel eCAL core has unresolved dynamic dependencies")
    return {"resolved": sorted(set(resolved)), "unresolved": unresolved}


def verify(runtime_root: Path) -> dict[str, object]:
    """启动只 dlopen core 的短生命周期 child，并验证其 eCAL/Protobuf 映射闭包。"""
    root_metadata = runtime_root.lstat()
    if runtime_root.is_symlink() or not runtime_root.is_dir() or not root_metadata:
        raise ValueError("runtime root must be a real directory")
    runtime_root = runtime_root.resolve(strict=True)
    python, core = _runtime_paths(runtime_root)
    environment = os.environ.copy()
    for key in ("PYTHONPATH", "PYTHONHOME", "LD_LIBRARY_PATH", "LD_PRELOAD"):
        environment.pop(key, None)
    environment.update(
        {
            "PATH": f"{python.parent}:{os.defpath}",
            "PYTHONNOUSERSITE": "1",
            "PYTHONDONTWRITEBYTECODE": "1",
        }
    )
    process = subprocess.Popen(
        [str(python), "-c", _CHILD_SOURCE, str(core)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=environment,
    )
    try:
        assert process.stdout is not None
        ready = process.stdout.readline().strip()
        if ready != _READY:
            stderr = process.stderr.read() if process.stderr is not None else ""
            raise ValueError(f"loader child did not become ready: {stderr.strip()}")
        mapped = _maps(process.pid)
    finally:
        if process.stdin is not None and not process.stdin.closed:
            process.stdin.write("\n")
            process.stdin.close()
        try:
            returncode = process.wait(timeout=10)
        except subprocess.TimeoutExpired as error:
            process.kill()
            process.wait()
            raise ValueError("loader child did not exit") from error
    stderr = process.stderr.read() if process.stderr is not None else ""
    if returncode != 0:
        raise ValueError(f"loader child failed: {stderr.strip()}")
    ecal_cores = [path for path in mapped if path.name.startswith("libecal_core.so")]
    protobuf = [path for path in mapped if path.name.startswith("libprotobuf.so")]
    if ecal_cores != [core]:
        raise ValueError("loader mapped an unexpected eCAL core")
    if protobuf:
        raise ValueError("loader mapped libprotobuf despite wheel ABI contract")
    return {
        "schema_version": 1,
        "runtime_root": str(runtime_root),
        "loaded_ecal_cores": [str(path) for path in ecal_cores],
        "loaded_protobuf_libraries": [str(path) for path in protobuf],
        "readelf": _readelf(core),
        "ldd": _ldd(core),
        # Child source only imports ctypes and calls CDLL; it cannot invoke any eCAL entity API.
        "loader_audit": {"mode": "ctypes-dlopen-only", "ecal_api_calls": 0},
    }


def main() -> int:
    """运行 fail-closed 的 Python wheel loader 验证，并写出结构化证据。"""
    parser = argparse.ArgumentParser(description="Verify Stage 4 Python eCAL no-participant loader.")
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    try:
        evidence = verify(args.runtime_root.absolute())
        args.evidence.parent.mkdir(parents=True, exist_ok=True)
        args.evidence.write_text(json.dumps(evidence, sort_keys=True) + "\n", encoding="utf-8")
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print("PASS: Python eCAL no-participant loader mapped only the wheel core")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
