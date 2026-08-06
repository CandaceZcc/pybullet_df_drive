#!/usr/bin/env python3
# 阶段四 Python runtime 安装标准化：移除 pip work 路径记录并重算两个 wheel 的 RECORD。
from __future__ import annotations

import argparse
import base64
import configparser
import csv
import hashlib
import json
from pathlib import Path, PurePosixPath
import stat
from urllib.parse import unquote, urlsplit


def _wheel_url(path: Path) -> str:
    """生成 pip 写入 direct_url.json 时使用的唯一本地 file URI。"""
    return path.resolve().as_uri()


def _read_direct_url(path: Path) -> str:
    """只接受 pip 标准 JSON 中的本地 file URI，避免宽松路径删除。"""
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(document.get("url"), str):
        raise ValueError("direct_url.json must contain a URL string")
    parsed = urlsplit(document["url"])
    if parsed.scheme != "file" or parsed.netloc not in {"", "localhost"}:
        raise ValueError("direct_url.json must contain a local file URL")
    return Path(unquote(parsed.path)).resolve().as_uri()


def _record_digest(path: Path) -> tuple[str, int]:
    """按 wheel RECORD 规则生成一个普通文件的 base64url SHA-256 与 size。"""
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
            size += len(chunk)
    encoded = base64.urlsafe_b64encode(digest.digest()).rstrip(b"=").decode("ascii")
    return encoded, size


def _safe_record_path(value: str) -> PurePosixPath:
    """限制 RECORD 成员为 site-packages 根下的规范相对普通文件路径。"""
    path = PurePosixPath(value)
    if path.is_absolute() or not path.parts or any(part in {"", ".", ".."} for part in path.parts):
        raise ValueError("RECORD contains an unsafe path")
    return path


def _project_console_scripts(dist_info: Path) -> dict[str, Path]:
    """从项目自身 entry_points 精确解析 pip 应写入 prefix/bin 的脚本。"""
    entry_points = dist_info / "entry_points.txt"
    if not entry_points.exists():
        return {}
    if not entry_points.is_file() or entry_points.is_symlink():
        raise ValueError("entry_points.txt must be a regular file")
    parser = configparser.ConfigParser(interpolation=None)
    parser.optionxform = str
    try:
        parser.read_file(entry_points.open(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, configparser.Error) as error:
        raise ValueError("entry_points.txt is invalid") from error
    if not parser.has_section("console_scripts"):
        return {}
    site_packages = dist_info.parent
    python_lib = site_packages.parent.parent
    if site_packages.name != "site-packages" or python_lib.name != "lib":
        raise ValueError("site-packages layout cannot locate prefix/bin")
    prefix = python_lib.parent
    scripts: dict[str, Path] = {}
    for name, _target in parser.items("console_scripts"):
        candidate = PurePosixPath(name)
        if (
            candidate.name != name
            or name in {"", ".", ".."}
            or any(ord(character) < 32 for character in name)
        ):
            raise ValueError("console script name is unsafe")
        script = prefix / "bin" / name
        script_metadata = script.lstat()
        if not stat.S_ISREG(script_metadata.st_mode) or stat.S_ISLNK(script_metadata.st_mode):
            raise ValueError("console script must be a regular file")
        scripts[f"../../../bin/{name}"] = script
    return scripts


def _refresh_record(dist_info: Path, removed_scripts: dict[str, Path]) -> None:
    """删除本项目脚本与 direct_url 后按路径排序重算同一 dist-info 的 RECORD。"""
    record_path = dist_info / "RECORD"
    site_packages = dist_info.parent
    relative_record = record_path.relative_to(site_packages).as_posix()
    direct_url = dist_info / "direct_url.json"
    relative_direct_url = direct_url.relative_to(site_packages).as_posix()
    try:
        rows = list(csv.reader(record_path.read_text(encoding="utf-8").splitlines()))
    except UnicodeDecodeError as error:
        raise ValueError("RECORD must be UTF-8") from error
    if any(len(row) != 3 for row in rows):
        raise ValueError("RECORD rows must have three columns")
    if direct_url.exists():
        if not direct_url.is_file() or direct_url.is_symlink():
            raise ValueError("direct_url.json must be a regular file")
        direct_url.unlink()
    record_paths = {row[0] for row in rows}
    if len(record_paths) != len(rows):
        raise ValueError("RECORD contains duplicate paths")
    if not set(removed_scripts).issubset(record_paths):
        raise ValueError("RECORD omits declared console script")
    for script in removed_scripts.values():
        script.unlink()
    members: set[str] = set()
    for relative_path, _digest, _size in rows:
        if relative_path in {relative_record, relative_direct_url, *removed_scripts}:
            continue
        safe = _safe_record_path(relative_path)
        target = site_packages.joinpath(*safe.parts)
        if not target.is_file() or target.is_symlink():
            raise ValueError("RECORD member is missing or not a regular file")
        members.add(safe.as_posix())
    refreshed = []
    for relative_path in sorted(members):
        digest, size = _record_digest(site_packages / relative_path)
        refreshed.append((relative_path, f"sha256={digest}", str(size)))
    refreshed.append((relative_record, "", ""))
    with record_path.open("w", encoding="utf-8", newline="") as stream:
        csv.writer(stream, lineterminator="\n").writerows(refreshed)


def normalize(runtime_root: Path, wheels: tuple[Path, ...]) -> None:
    """精确标准化 eCAL/项目 wheel，并只移除项目声明的 console scripts。"""
    if len(wheels) != 2 or len({wheel.resolve() for wheel in wheels}) != 2:
        raise ValueError("normalizer requires exactly two distinct wheel paths")
    expected_urls = {_wheel_url(wheel) for wheel in wheels}
    matches: dict[str, Path] = {}
    for direct_url in runtime_root.rglob("direct_url.json"):
        if not direct_url.is_file() or direct_url.is_symlink() or direct_url.parent.suffix != ".dist-info":
            continue
        actual_url = _read_direct_url(direct_url)
        if actual_url in expected_urls:
            if actual_url in matches:
                raise ValueError("a pip wheel has multiple direct_url.json records")
            matches[actual_url] = direct_url.parent
    if set(matches) != expected_urls:
        raise ValueError("pip wheel direct_url.json record is missing")
    project_url = _wheel_url(wheels[1])
    for wheel_url, dist_info in matches.items():
        scripts = _project_console_scripts(dist_info) if wheel_url == project_url else {}
        _refresh_record(dist_info, scripts)


def main() -> int:
    """标准化此次 pip 安装的 metadata，不接触 Conda 已有 metadata。"""
    parser = argparse.ArgumentParser(description="Normalize Stage 4 pip-installed wheel metadata.")
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--wheel", type=Path, action="append", required=True)
    args = parser.parse_args()
    try:
        normalize(args.runtime_root.resolve(), tuple(path.resolve() for path in args.wheel))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}")
        return 1
    print("PASS: pip wheel metadata normalized")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
