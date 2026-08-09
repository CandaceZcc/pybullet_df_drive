#!/usr/bin/env python3
# 阶段四私有 Conda 包规范化器：消除 conda-build 自动 metadata 的临时路径和时间漂移。
from __future__ import annotations

import argparse
import io
import json
from pathlib import Path
import stat
import tempfile
import tarfile
from collections.abc import Sequence
from zipfile import BadZipFile, ZIP_STORED, ZipFile, ZipInfo

import conda_package_handling.api
import yaml
import zstandard


_ZIP_TIMESTAMP = (1980, 1, 1, 0, 0, 0)
_CANONICAL_SOURCE_PATH = "/stage4/protobuf-work/source/protobuf-33.6"


def _regular_file(path: Path, label: str) -> None:
    """拒绝链接、硬链接和特殊节点，保证重新封装的输入边界可审计。"""
    metadata = path.lstat()
    if (
        not stat.S_ISREG(metadata.st_mode)
        or stat.S_ISLNK(metadata.st_mode)
        or metadata.st_nlink != 1
    ):
        raise ValueError(f"{label} must be a singly linked regular file")


def _verify_conda_container(path: Path) -> None:
    """仅接受 CEP-35 v2 的三个已知 outer ZIP 成员，阻止携带额外内容。"""
    package_id = path.name.removesuffix(".conda")
    expected = {
        "metadata.json",
        f"pkg-{package_id}.tar.zst",
        f"info-{package_id}.tar.zst",
    }
    with ZipFile(path) as archive:
        names = [member.filename for member in archive.infolist()]
        if len(names) != len(set(names)) or set(names) != expected:
            raise ValueError("private Conda package contains unexpected ZIP members")
        metadata = json.loads(archive.read("metadata.json"))
    if metadata != {"conda_pkg_format_version": 2}:
        raise ValueError("private Conda package metadata version is unsupported")


def _regular_tree_files(root: Path) -> list[Path]:
    """枚举解包树的普通文件；目录、链接和硬链接一律不能进入输出。"""
    files: list[Path] = []
    for path in root.rglob("*"):
        metadata = path.lstat()
        if stat.S_ISLNK(metadata.st_mode):
            raise ValueError("private Conda package must not contain links")
        if stat.S_ISDIR(metadata.st_mode):
            continue
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise ValueError("private Conda package must contain only regular files")
        files.append(path)
    return sorted(files, key=lambda path: path.relative_to(root).as_posix())


def _load_mapping(path: Path, label: str) -> dict[str, object]:
    """加载 JSON/YAML mapping，避免把无结构文本当成可安全规范化的 metadata。"""
    if path.suffix == ".json":
        document = json.loads(path.read_text(encoding="utf-8"))
    else:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{label} must be a mapping")
    return document


def _write_json(path: Path, document: dict[str, object]) -> None:
    """统一 JSON 键顺序和换行，消除 Conda 运行时字典插入顺序差异。"""
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_yaml(path: Path, document: dict[str, object]) -> None:
    """以稳定键顺序写回 YAML，保留 recipe 的结构化语义而非路径文本。"""
    path.write_text(
        yaml.safe_dump(document, allow_unicode=False, default_flow_style=False, sort_keys=True),
        encoding="utf-8",
    )


def _normalize_volatile_metadata(root: Path) -> None:
    """只改已验证会随构建根漂移的 metadata，运行 payload 保持原字节。"""
    about_path = root / "info" / "about.json"
    about = _load_mapping(about_path, "info/about.json")
    channels = about.get("channels")
    if not isinstance(channels, list) or not all(isinstance(channel, str) for channel in channels):
        raise ValueError("info/about.json channels must be a string list")
    about["channels"] = []
    _write_json(about_path, about)

    index_path = root / "info" / "index.json"
    index = _load_mapping(index_path, "info/index.json")
    if not isinstance(index.get("timestamp"), int) or isinstance(index["timestamp"], bool):
        raise ValueError("info/index.json timestamp must be an integer")
    index["timestamp"] = 0
    _write_json(index_path, index)

    hash_input_path = root / "info" / "hash_input.json"
    _write_json(hash_input_path, _load_mapping(hash_input_path, "info/hash_input.json"))

    recipe_path = root / "info" / "recipe" / "meta.yaml"
    recipe = _load_mapping(recipe_path, "info/recipe/meta.yaml")
    source = recipe.get("source")
    if not isinstance(source, dict) or not isinstance(source.get("path"), str):
        raise ValueError("info/recipe/meta.yaml source path is missing")
    source["path"] = _CANONICAL_SOURCE_PATH
    _write_yaml(recipe_path, recipe)

    variants_path = root / "info" / "recipe" / "conda_build_config.yaml"
    variants = _load_mapping(variants_path, "info/recipe/conda_build_config.yaml")
    extend_keys = variants.get("extend_keys")
    if not isinstance(extend_keys, list) or not all(isinstance(key, str) for key in extend_keys):
        raise ValueError("info/recipe/conda_build_config.yaml extend_keys is invalid")
    if len(extend_keys) != len(set(extend_keys)):
        raise ValueError("info/recipe/conda_build_config.yaml extend_keys contains duplicates")
    variants["extend_keys"] = sorted(extend_keys)
    _write_yaml(variants_path, variants)


def _tar_component(root: Path, files: Sequence[Path]) -> bytes:
    """以稳定路径、权限、owner 和 mtime 生成一个未压缩 tar component。"""
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode="w", format=tarfile.PAX_FORMAT) as archive:
        for path in files:
            metadata = path.stat()
            member = tarfile.TarInfo(path.relative_to(root).as_posix())
            member.mode = stat.S_IMODE(metadata.st_mode)
            member.uid = 0
            member.gid = 0
            member.uname = ""
            member.gname = ""
            member.mtime = 0
            member.size = metadata.st_size
            with path.open("rb") as source:
                archive.addfile(member, source)
    return stream.getvalue()


def _zstd(data: bytes) -> bytes:
    """固定 zstd 选项，避免线程数、checksum 或 content-size 标记影响最终包。"""
    compressor = zstandard.ZstdCompressor(
        level=19,
        threads=1,
        write_checksum=False,
        write_content_size=True,
        write_dict_id=False,
    )
    return compressor.compress(data)


def _zip_member(name: str, data: bytes) -> ZipInfo:
    """构造不含当前时钟、宿主权限或平台字段的 ZIP member。"""
    member = ZipInfo(name, date_time=_ZIP_TIMESTAMP)
    member.compress_type = ZIP_STORED
    member.create_system = 3
    member.external_attr = 0o100644 << 16
    return member


def canonicalize(input_package: Path, output_package: Path) -> None:
    """解包、规范化已知 metadata，再按 CEP-35 v2 固定顺序重新封装。"""
    _regular_file(input_package, "input package")
    if output_package.exists() or output_package.is_symlink() or not output_package.parent.is_dir():
        raise ValueError("canonical output must be absent under an existing directory")
    _verify_conda_container(input_package)
    package_id = input_package.name.removesuffix(".conda")
    with tempfile.TemporaryDirectory(prefix="stage4-conda-canonical-", dir=output_package.parent) as temporary:
        root = Path(temporary) / "root"
        conda_package_handling.api.extract(str(input_package), dest_dir=str(root))
        _normalize_volatile_metadata(root)
        files = _regular_tree_files(root)
        package_files = [path for path in files if not path.relative_to(root).as_posix().startswith("info/")]
        info_files = [path for path in files if path not in package_files]
        payload = io.BytesIO()
        with ZipFile(payload, "w", compression=ZIP_STORED, allowZip64=False) as archive:
            archive.writestr(
                _zip_member("metadata.json", b'{"conda_pkg_format_version":2}'),
                b'{"conda_pkg_format_version":2}',
            )
            archive.writestr(
                _zip_member(f"pkg-{package_id}.tar.zst", b""),
                _zstd(_tar_component(root, package_files)),
            )
            archive.writestr(
                _zip_member(f"info-{package_id}.tar.zst", b""),
                _zstd(_tar_component(root, info_files)),
            )
        with output_package.open("xb") as output:
            output.write(payload.getvalue())
            output.flush()
            output_package.chmod(0o644)


def main(argv: Sequence[str] | None = None) -> int:
    """解析唯一输入/输出，失败时不生成部分 canonical package。"""
    parser = argparse.ArgumentParser(description="Canonicalize a private Stage 4 Conda package.")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        canonicalize(args.input.resolve(), args.output.resolve())
    except (OSError, ValueError, json.JSONDecodeError, tarfile.TarError, BadZipFile) as error:
        print(f"FAIL: {error}")
        return 1
    print("PASS: private Conda package canonicalized")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
