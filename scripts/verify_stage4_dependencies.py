#!/usr/bin/env python3
# 阶段四依赖验证入口：后续集中校验冻结锁、缓存和构建环境，缺输入时严格失败。
from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


_COMMIT_PATTERN = re.compile(r"[0-9a-f]{40}")
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
_IMAGE_DIGEST_PATTERN = re.compile(r"sha256:[0-9a-f]{64}")
_ENVIRONMENT_NAME_PATTERN = re.compile(r"STAGE4_[A-Z0-9_]+")
_ARCHIVE_FORMATS = frozenset({"tar.gz", "tar.xz", "zip"})
_ARCHIVE_CONSUMERS = frozenset({"cpp_dependency", "validation", "ros_overlay"})
_ROOT = Path(__file__).resolve().parents[1]
_COMMAND_VERSION_PATTERNS = {
    "cmake": r"cmake version 3\.28\.\d+",
    "ctest": r"ctest version 3\.28\.\d+",
    "cc": r".*\b13\.\d+\.\d+",
    "cxx": r".*\b13\.\d+\.\d+",
    "protoc": r"libprotoc 33\.6",
}


@dataclass(frozen=True)
class DependencyLockEntry:
    """单个源码依赖的不可变锁定身份与归档摘要。"""

    name: str
    url: str
    ref_kind: str
    ref: str
    commit: str
    archive_format: str
    archive_size: int
    archive_sha256: str
    consumers: tuple[str, ...]


def _validated_environment(environment: dict[str, str]) -> dict[str, str]:
    """规范化可 source 的环境变量，拒绝 shell 语法与相对路径。"""
    if not environment:
        raise ValueError("build environment must not be empty")
    normalized: dict[str, str] = {}
    for name, value in environment.items():
        if (
            not isinstance(name, str)
            or _ENVIRONMENT_NAME_PATTERN.fullmatch(name) is None
            or not isinstance(value, str)
            or not value.startswith("/")
            or "\0" in value
            or "\n" in value
            or "\r" in value
            or "$(" in value
            or "`" in value
        ):
            raise ValueError("build environment assignment is invalid")
        normalized[name] = value
    return dict(sorted(normalized.items()))


def _validated_system_dependencies(
    system_dependencies: dict[str, dict[str, str]],
) -> dict[str, dict[str, str]]:
    """规范化已核验的系统 SONAME/package/version 证据字段。"""
    if not isinstance(system_dependencies, dict) or not system_dependencies:
        raise ValueError("system dependency evidence must not be empty")
    normalized: dict[str, dict[str, str]] = {}
    for soname, record in system_dependencies.items():
        if (
            not isinstance(soname, str)
            or not soname
            or not isinstance(record, dict)
            or set(record) != {"package", "version"}
            or not all(isinstance(value, str) and value for value in record.values())
        ):
            raise ValueError("system dependency evidence is invalid")
        normalized[soname] = {
            "package": record["package"],
            "version": record["version"],
        }
    return dict(sorted(normalized.items()))


def _environment_payload(environment: dict[str, str]) -> bytes:
    """以受限单引号格式生成 sourceable shell 文件，绝不解释变量值。"""
    lines = []
    for name, value in environment.items():
        escaped = value.replace("'", "'\"'\"'")
        lines.append(f"export {name}='{escaped}'\n")
    return "".join(lines).encode("utf-8")


def _write_new_file(path: Path, payload: bytes) -> None:
    """独占写入并 fsync 证据文件，禁止覆盖上一轮有效环境。"""
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
    with os.fdopen(descriptor, "wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _sha256_file(path: Path) -> tuple[int, str]:
    """流式计算普通文件身份，避免把大样例一次性载入内存。"""
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            size += len(chunk)
            digest.update(chunk)
    return size, digest.hexdigest()


def _directory_tree_sha256(path: Path, *, allow_in_tree_symlinks: bool = False) -> str:
    """计算目录树摘要；仅私有 dependency prefix 可保留根内 SONAME 链接。"""
    digest = hashlib.sha256()
    root = path.resolve(strict=True)
    if path.is_symlink() or not root.is_dir():
        raise ValueError("runtime identity directory is invalid")
    for member in sorted(root.rglob("*"), key=lambda item: item.as_posix()):
        metadata = member.lstat()
        relative = member.relative_to(root).as_posix()
        if member.is_symlink():
            if not allow_in_tree_symlinks:
                raise ValueError("runtime identity directory contains a link")
            target = os.readlink(member)
            if Path(target).is_absolute():
                raise ValueError("runtime identity directory link target is unsafe")
            try:
                member.resolve(strict=True).relative_to(root)
            except ValueError as error:
                raise ValueError("runtime identity directory link target escapes root") from error
            digest.update(f"symlink\\0{relative}\\0{target}\\0".encode("utf-8"))
            continue
        if member.is_dir():
            digest.update(f"directory\\0{relative}\\0".encode("utf-8"))
            continue
        if not member.is_file() or metadata.st_nlink != 1:
            raise ValueError("runtime identity directory contains an unsafe member")
        size, file_digest = _sha256_file(member)
        digest.update(f"file\\0{relative}\\0{size}\\0{file_digest}\\0".encode("utf-8"))
    return digest.hexdigest()


def _path_identity(
    path: Path, option: str, *, allow_in_tree_symlinks: bool = False
) -> dict[str, object]:
    """记录已 probe 输入的实时文件或目录树身份，供每次 source 前复验。"""
    if path.is_symlink():
        raise ValueError(f"{option} runtime identity must not be a link")
    if path.is_file():
        metadata = path.lstat()
        if metadata.st_nlink != 1:
            raise ValueError(f"{option} runtime identity file must have one link")
        size, digest = _sha256_file(path)
        return {"kind": "file", "size": size, "sha256": digest}
    if path.is_dir():
        return {
            "kind": "directory",
            "tree_sha256": _directory_tree_sha256(
                path, allow_in_tree_symlinks=allow_in_tree_symlinks
            ),
        }
    raise ValueError(f"{option} runtime identity path is invalid")


def _validated_runtime_identities(
    identities: object, expected_names: set[str]
) -> dict[str, dict[str, object]]:
    """校验环境证据的完整身份集合，禁止遗漏任一运行时输入。"""
    if not isinstance(identities, dict) or set(identities) != expected_names:
        raise ValueError("build environment runtime identities are invalid")
    normalized: dict[str, dict[str, object]] = {}
    for name, record in identities.items():
        if not isinstance(name, str) or not isinstance(record, dict):
            raise ValueError("build environment runtime identities are invalid")
        if record.get("kind") == "file" and set(record) == {"kind", "size", "sha256"}:
            if (
                isinstance(record["size"], int)
                and not isinstance(record["size"], bool)
                and record["size"] >= 0
                and isinstance(record["sha256"], str)
                and _SHA256_PATTERN.fullmatch(record["sha256"]) is not None
            ):
                normalized[name] = dict(record)
                continue
        if record.get("kind") == "directory" and set(record) == {"kind", "tree_sha256"}:
            if isinstance(record["tree_sha256"], str) and _SHA256_PATTERN.fullmatch(
                record["tree_sha256"]
            ) is not None:
                normalized[name] = dict(record)
                continue
        raise ValueError("build environment runtime identities are invalid")
    return dict(sorted(normalized.items()))


def _runtime_identity_inputs(
    environment: dict[str, str],
    *,
    system_lock: Path,
    ldd: Path,
    dpkg_query: Path,
) -> dict[str, Path]:
    """从环境合同恢复 probe 输入，避免 A-E 使用未记录的默认路径。"""
    pcl_pcd2ply = Path(environment["STAGE4_PCL_PCD2PLY"])
    return {
        **{name: Path(value) for name, value in environment.items()},
        "validation_prefix": pcl_pcd2ply.parent.parent,
        "system_lock": system_lock,
        "ldd": ldd,
        "dpkg_query": dpkg_query,
    }


def _runtime_identities(inputs: dict[str, Path]) -> dict[str, dict[str, object]]:
    """计算全部输入身份；只有私有前缀允许稳定的 SONAME 软链接。"""
    return {
        name: _path_identity(
            path,
            name,
            allow_in_tree_symlinks=name
            in {
                "STAGE4_DEPENDENCY_PREFIX",
                "STAGE4_CMAKE_PREFIX_PATH",
                "validation_prefix",
            },
        )
        for name, path in inputs.items()
    }


def _validated_verification_inputs(inputs: object) -> dict[str, str]:
    """限制重放 probe 所需的非 source 路径，拒绝隐式系统默认值。"""
    if not isinstance(inputs, dict) or set(inputs) != {"system_lock", "ldd", "dpkg_query"}:
        raise ValueError("build environment verification inputs are invalid")
    return _validated_environment(
        {f"STAGE4_{name.upper()}": value for name, value in inputs.items()}
    )


def write_build_environment(
    environment: dict[str, str],
    environment_file: Path,
    evidence_file: Path,
    *,
    system_dependencies: dict[str, dict[str, str]],
    runtime_identities: dict[str, dict[str, object]] | None = None,
    verification_inputs: dict[str, str] | None = None,
) -> None:
    """原子生成环境与 JSON 证据；两个输出都必须是调用方的新路径。"""
    normalized = _validated_environment(environment)
    normalized_system_dependencies = _validated_system_dependencies(system_dependencies)
    if environment_file.exists() or evidence_file.exists() or environment_file == evidence_file:
        raise ValueError("build environment outputs must be distinct new files")
    if not environment_file.parent.is_dir() or not evidence_file.parent.is_dir():
        raise ValueError("build environment output parents must exist")
    payload = _environment_payload(normalized)
    _write_new_file(environment_file, payload)
    document: dict[str, object] = {
        "environment": normalized,
        "environment_file_sha256": hashlib.sha256(payload).hexdigest(),
        "system_dependencies": normalized_system_dependencies,
    }
    if runtime_identities is None and verification_inputs is None:
        document["schema_version"] = 2
    else:
        if runtime_identities is None or verification_inputs is None:
            raise ValueError("build environment runtime evidence is incomplete")
        normalized_verification_inputs = _validated_verification_inputs(verification_inputs)
        document["schema_version"] = 3
        document["runtime_identities"] = _validated_runtime_identities(
            runtime_identities,
            set(normalized) | {"validation_prefix", "system_lock", "ldd", "dpkg_query"},
        )
        document["verification_inputs"] = {
            name.removeprefix("STAGE4_").lower(): value
            for name, value in normalized_verification_inputs.items()
        }
    _write_new_file(
        evidence_file,
        (json.dumps(document, sort_keys=True) + "\n").encode("utf-8"),
    )


def verify_build_environment(environment_file: Path, evidence_file: Path) -> dict[str, str]:
    """复算环境合同，并在 source 前重新 probe 全部实际输入。"""
    document = json.loads(evidence_file.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or set(document) != {
        "environment",
        "environment_file_sha256",
        "runtime_identities",
        "schema_version",
        "system_dependencies",
        "verification_inputs",
    } or document["schema_version"] != 3:
        raise ValueError("build environment evidence is invalid")
    environment = _validated_environment(document["environment"])
    expected_system_dependencies = _validated_system_dependencies(document["system_dependencies"])
    expected_digest = document["environment_file_sha256"]
    if not isinstance(expected_digest, str) or _SHA256_PATTERN.fullmatch(expected_digest) is None:
        raise ValueError("build environment evidence digest is invalid")
    payload = environment_file.read_bytes()
    if hashlib.sha256(payload).hexdigest() != expected_digest or payload != _environment_payload(environment):
        raise ValueError("build environment file differs from evidence")
    verification_inputs = _validated_verification_inputs(document["verification_inputs"])
    actual_environment, actual_system_dependencies = _probe_build_environment_from_inputs(
        cmake=Path(environment["STAGE4_CMAKE"]),
        ctest=Path(environment["STAGE4_CTEST"]),
        cc=Path(environment["STAGE4_CC"]),
        cxx=Path(environment["STAGE4_CXX"]),
        protoc=Path(environment["STAGE4_PROTOC"]),
        micromamba=Path(environment["STAGE4_MICROMAMBA"]),
        python_package_cache=Path(environment["STAGE4_PYTHON_PACKAGE_CACHE"]),
        python_wheel_cache=Path(environment["STAGE4_PYTHON_WHEEL_CACHE"]),
        source_archive_cache=Path(environment["STAGE4_SOURCE_ARCHIVE_CACHE"]),
        dependency_prefix=Path(environment["STAGE4_DEPENDENCY_PREFIX"]),
        pcl_pcd2ply=Path(environment["STAGE4_PCL_PCD2PLY"]),
        system_lock=Path(verification_inputs["STAGE4_SYSTEM_LOCK"]),
        ldd=Path(verification_inputs["STAGE4_LDD"]),
        dpkg_query=Path(verification_inputs["STAGE4_DPKG_QUERY"]),
        mid360_reference_lvx2=Path(environment["STAGE4_MID360_REFERENCE_LVX2"]),
        rviz2=Path(environment["STAGE4_RVIZ2"]),
    )
    if actual_environment != environment or actual_system_dependencies != expected_system_dependencies:
        raise ValueError("build environment runtime probe differs from evidence")
    expected_identities = _validated_runtime_identities(
        document["runtime_identities"],
        set(environment) | {"validation_prefix", "system_lock", "ldd", "dpkg_query"},
    )
    actual_identities = _runtime_identities(
        _runtime_identity_inputs(
            environment,
            system_lock=Path(verification_inputs["STAGE4_SYSTEM_LOCK"]),
            ldd=Path(verification_inputs["STAGE4_LDD"]),
            dpkg_query=Path(verification_inputs["STAGE4_DPKG_QUERY"]),
        )
    )
    if actual_identities != expected_identities:
        raise ValueError("build environment runtime identity differs from evidence")
    return environment


def _require_regular_executable(path: Path | None, option: str) -> Path:
    """验证 probe 的工具输入是绝对、普通且可执行的文件。"""
    if (
        path is None
        or not path.is_absolute()
        or path.is_symlink()
        or not path.is_file()
        or not os.access(path, os.X_OK)
    ):
        raise ValueError(f"{option} must be a regular executable")
    return path.resolve(strict=True)


def _require_directory(path: Path | None, option: str) -> Path:
    """验证 canonical cache 输入是绝对且非链接的已有目录。"""
    if path is None or not path.is_absolute() or path.is_symlink() or not path.is_dir():
        raise ValueError(f"{option} must be an existing directory")
    return path.resolve(strict=True)


def _require_regular_file(path: Path | None, option: str) -> Path:
    """验证 probe 的只读样例输入是绝对、普通且非链接的文件。"""
    if path is None or not path.is_absolute() or path.is_symlink() or not path.is_file():
        raise ValueError(f"{option} must be a regular file")
    return path.resolve(strict=True)


def require_command_version(path: Path, option: str, expected_pattern: str) -> str:
    """运行指定工具的 --version，并以冻结正则验证完整首行。"""
    executable = _require_regular_executable(path, option)
    completed = subprocess.run(
        [str(executable), "--version"],
        check=False,
        capture_output=True,
        text=True,
    )
    first_line = completed.stdout.splitlines()[0] if completed.stdout else ""
    if completed.returncode != 0 or re.fullmatch(expected_pattern, first_line) is None:
        raise ValueError(f"{option} version does not match the frozen contract")
    return first_line


def _probe_build_environment_from_inputs(
    *,
    cmake: Path,
    ctest: Path,
    cc: Path,
    cxx: Path,
    protoc: Path,
    micromamba: Path,
    python_package_cache: Path,
    python_wheel_cache: Path,
    source_archive_cache: Path,
    dependency_prefix: Path,
    pcl_pcd2ply: Path,
    system_lock: Path,
    ldd: Path,
    dpkg_query: Path,
    mid360_reference_lvx2: Path,
    rviz2: Path,
) -> tuple[dict[str, str], dict[str, dict[str, str]]]:
    """同一次 probe 返回 shell 合同与已核验的系统 DSO 证据。"""
    normalized = {
        "STAGE4_CMAKE": _require_regular_executable(cmake, "--cmake"),
        "STAGE4_CTEST": _require_regular_executable(ctest, "--ctest"),
        "STAGE4_CC": _require_regular_executable(cc, "--cc"),
        "STAGE4_CXX": _require_regular_executable(cxx, "--cxx"),
        "STAGE4_PROTOC": _require_regular_executable(protoc, "--protoc"),
        "STAGE4_MICROMAMBA": _require_regular_executable(micromamba, "--micromamba"),
        "STAGE4_PYTHON_PACKAGE_CACHE": _require_directory(
            python_package_cache, "--python-package-cache"
        ),
        "STAGE4_PYTHON_WHEEL_CACHE": _require_directory(
            python_wheel_cache, "--python-wheel-cache"
        ),
        "STAGE4_SOURCE_ARCHIVE_CACHE": _require_directory(
            source_archive_cache, "--source-archive-cache"
        ),
        "STAGE4_DEPENDENCY_PREFIX": _require_directory(
            dependency_prefix, "--dependency-prefix"
        ),
        "STAGE4_CMAKE_PREFIX_PATH": _require_directory(
            dependency_prefix, "--dependency-prefix"
        ),
        "STAGE4_PCL_PCD2PLY": _require_regular_executable(
            pcl_pcd2ply, "--pcl-pcd2ply"
        ),
        "STAGE4_MID360_REFERENCE_LVX2": _require_regular_file(
            mid360_reference_lvx2, "--mid360-reference-lvx2"
        ),
        "STAGE4_RVIZ2": _require_regular_executable(rviz2, "--rviz2"),
    }
    system_dependencies = verify_pcl_system_dependencies(
        normalized["STAGE4_PCL_PCD2PLY"],
        system_lock,
        dependency_prefix=normalized["STAGE4_DEPENDENCY_PREFIX"],
        ldd=ldd,
        dpkg_query=dpkg_query,
    )
    # 统一入口必须固定实际执行的工具版本，不能只依赖文件名或调用机 PATH。
    for name in ("cmake", "ctest", "cc", "cxx", "protoc"):
        require_command_version(normalized[f"STAGE4_{name.upper()}"], name, _COMMAND_VERSION_PATTERNS[name])
    return (
        {name: str(path) for name, path in normalized.items()},
        system_dependencies,
    )


def build_environment_from_probe_inputs(
    *,
    cmake: Path,
    ctest: Path,
    cc: Path,
    cxx: Path,
    protoc: Path,
    micromamba: Path,
    python_package_cache: Path,
    python_wheel_cache: Path,
    source_archive_cache: Path,
    dependency_prefix: Path,
    pcl_pcd2ply: Path,
    system_lock: Path,
    ldd: Path,
    dpkg_query: Path,
    mid360_reference_lvx2: Path,
    rviz2: Path,
) -> dict[str, str]:
    """公开环境映射入口，保留 shell 合同而不暴露内部证据组装。"""
    environment, _ = _probe_build_environment_from_inputs(
        cmake=cmake,
        ctest=ctest,
        cc=cc,
        cxx=cxx,
        protoc=protoc,
        micromamba=micromamba,
        python_package_cache=python_package_cache,
        python_wheel_cache=python_wheel_cache,
        source_archive_cache=source_archive_cache,
        dependency_prefix=dependency_prefix,
        pcl_pcd2ply=pcl_pcd2ply,
        system_lock=system_lock,
        ldd=ldd,
        dpkg_query=dpkg_query,
        mid360_reference_lvx2=mid360_reference_lvx2,
        rviz2=rviz2,
    )
    return environment


def load_system_dependency_lock(path: Path) -> dict[str, object]:
    """加载 Ubuntu 24.04 系统依赖锁，拒绝浮动 image 或 SONAME 白名单。"""
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"system dependency lock is not valid JSON: {path}") from error
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("system dependency lock must use schema_version 1")
    platform = document.get("platform")
    if not isinstance(platform, dict) or platform != {
        "id": "ubuntu",
        "version_id": "24.04",
        "codename": "noble",
        "architecture": "amd64",
    }:
        raise ValueError("system dependency lock must target Ubuntu 24.04 noble amd64")
    builder_image = document.get("builder_image")
    if not isinstance(builder_image, dict) or set(builder_image) != {"reference", "digest"}:
        raise ValueError("system dependency lock builder image is invalid")
    if not isinstance(builder_image["reference"], str) or not builder_image["reference"]:
        raise ValueError("builder image reference must be a nonempty string")
    digest = builder_image["digest"]
    if not isinstance(digest, str) or _IMAGE_DIGEST_PATTERN.fullmatch(digest) is None:
        raise ValueError("builder image digest must be a sha256")
    for field, identity in (("apt_packages", "name"), ("allowed_system_sonames", "soname")):
        records = document.get(field)
        if not isinstance(records, list) or not records:
            raise ValueError(f"system dependency lock {field} must be a nonempty list")
        seen: set[str] = set()
        for record in records:
            expected_fields = (
                {"name", "version", "architecture"}
                if field == "apt_packages"
                else {"soname", "package", "version"}
            )
            if not isinstance(record, dict) or set(record) != expected_fields:
                raise ValueError(f"system dependency lock {field} entry is invalid")
            values = tuple(record.values())
            if not all(isinstance(value, str) and value for value in values):
                raise ValueError(f"system dependency lock {field} entry is invalid")
            if not record["version"][0].isdigit():
                raise ValueError("system dependency lock version must begin with a digit")
            name = record[identity]
            if name in seen:
                raise ValueError(f"system dependency lock {field} contains duplicates")
            seen.add(name)
    return document


def verify_system_soname_packages(
    system_lock: Path,
    sonames: Sequence[str],
    *,
    dpkg_query: Path,
) -> dict[str, dict[str, str]]:
    """将已解析系统 SONAME 逐项绑定到锁定的已安装 Debian package。"""
    document = load_system_dependency_lock(system_lock)
    allowed = {
        record["soname"]: {
            "package": record["package"],
            "version": record["version"],
        }
        for record in document["allowed_system_sonames"]
    }
    query = _require_regular_executable(dpkg_query, "dpkg-query")
    verified: dict[str, dict[str, str]] = {}
    for soname in sorted(set(sonames)):
        expected = allowed.get(soname)
        if expected is None:
            raise ValueError(f"system SONAME is not allowed by the lock: {soname}")
        completed = subprocess.run(
            [
                str(query),
                "--showformat=${db:Status-Status}\\t${Version}\\n",
                "--show",
                expected["package"],
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        fields = completed.stdout.strip().split("\t")
        if completed.returncode != 0 or len(fields) != 2 or fields[0] != "installed":
            raise ValueError(f"system package is not installed: {expected['package']}")
        if fields[1] != expected["version"]:
            raise ValueError(
                f"system package version does not match the lock: {expected['package']}"
            )
        verified[soname] = expected
    return verified


def verify_pcl_system_dependencies(
    pcl_pcd2ply: Path,
    system_lock: Path,
    *,
    dependency_prefix: Path,
    ldd: Path,
    dpkg_query: Path,
) -> dict[str, dict[str, str]]:
    """验证 PCL DSO：系统库锁定，非系统库只能来自本轮私有 prefix。"""
    validator = _require_regular_executable(pcl_pcd2ply, "--pcl-pcd2ply")
    private_prefix = _require_directory(dependency_prefix, "--dependency-prefix")
    if validator.parent.name != "bin":
        raise ValueError("PCL validator must be installed under validation prefix bin")
    validation_prefix = _require_directory(
        validator.parent.parent, "PCL validation prefix"
    )
    ldd_command = _require_regular_executable(ldd, "ldd")
    completed = subprocess.run(
        [str(ldd_command), str(validator)],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise ValueError("PCL validator dynamic dependency inspection failed")
    system_sonames: list[str] = []
    for line in completed.stdout.splitlines():
        line = line.strip()
        if not line or line.startswith("linux-vdso.so"):
            continue
        if "=>" in line:
            soname, resolved = line.split("=>", 1)
            target = resolved.strip().split(maxsplit=1)[0] if resolved.strip() else ""
        elif line.startswith("/"):
            target = line.split(maxsplit=1)[0]
            soname = Path(target).name
            if soname.startswith("ld-linux"):
                continue
        else:
            raise ValueError("PCL validator ldd output contains an unsupported dependency line")
        if target == "not":
            raise ValueError(f"PCL validator has an unresolved dependency: {soname.strip()}")
        if target.startswith(("/lib/", "/usr/lib/", "/lib64/", "/usr/lib64/")):
            system_sonames.append(soname.strip())
            continue
        try:
            resolved_target = Path(target).resolve(strict=True)
            if not any(
                resolved_target.is_relative_to(prefix)
                for prefix in (private_prefix, validation_prefix)
            ):
                raise ValueError("non-system dependency is outside private prefix")
        except (OSError, ValueError) as error:
            raise ValueError("PCL validator non-system dependency is outside private prefix") from error
    return verify_system_soname_packages(
        system_lock,
        system_sonames,
        dpkg_query=dpkg_query,
    )


def _require_text(record: dict[str, object], field: str) -> str:
    """读取结构化 lock 的非空文本字段，拒绝隐式类型转换。"""
    value = record.get(field)
    if not isinstance(value, str) or not value:
        raise ValueError(f"dependency {field} must be a nonempty string")
    return value


def _require_consumers(record: dict[str, object]) -> tuple[str, ...]:
    """读取归档的唯一构建消费者，拒绝无归属或未准入入口。"""
    value = record.get("consumers")
    if not isinstance(value, list) or not value or not all(
        isinstance(item, str) and item for item in value
    ):
        raise ValueError("dependency consumers must be a nonempty list")
    consumers = tuple(value)
    if len(set(consumers)) != len(consumers):
        raise ValueError("dependency consumers must not contain duplicates")
    if not set(consumers).issubset(_ARCHIVE_CONSUMERS):
        raise ValueError("dependency consumers include an unsupported value")
    return consumers


def load_dependency_lock(path: Path) -> tuple[DependencyLockEntry, ...]:
    """加载并校验依赖锁的不可变 ref、commit 与归档摘要关系。"""
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"dependency lock is not valid JSON: {path}") from error
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise ValueError("dependency lock must use schema_version 1")
    records = document.get("dependencies")
    if not isinstance(records, list) or not records:
        raise ValueError("dependency lock must contain a nonempty dependencies list")

    entries: list[DependencyLockEntry] = []
    names: set[str] = set()
    for record in records:
        if not isinstance(record, dict):
            raise ValueError("dependency lock entries must be objects")
        name = _require_text(record, "name")
        if name in names:
            raise ValueError(f"duplicate dependency name: {name}")
        names.add(name)
        ref_kind = _require_text(record, "ref_kind")
        if ref_kind not in {"tag", "commit"}:
            raise ValueError("dependency ref_kind must be tag or commit")
        ref = _require_text(record, "ref")
        commit = _require_text(record, "commit")
        if _COMMIT_PATTERN.fullmatch(commit) is None:
            raise ValueError("dependency commit must be a 40-character lowercase SHA")
        if ref_kind == "commit" and ref != commit:
            raise ValueError("dependency commit ref must equal commit")

        archive = record.get("archive")
        if not isinstance(archive, dict):
            raise ValueError("dependency archive must be an object")
        archive_format = _require_text(archive, "format")
        if archive_format not in _ARCHIVE_FORMATS:
            raise ValueError("dependency archive format is not supported")
        archive_size = archive.get("size")
        if isinstance(archive_size, bool) or not isinstance(archive_size, int) or archive_size <= 0:
            raise ValueError("dependency archive size must be a positive integer")
        archive_sha256 = _require_text(archive, "sha256")
        if _SHA256_PATTERN.fullmatch(archive_sha256) is None:
            raise ValueError("dependency archive sha256 must be a lowercase SHA-256")
        consumers = _require_consumers(record)
        entries.append(
            DependencyLockEntry(
                name=name,
                url=_require_text(record, "url"),
                ref_kind=ref_kind,
                ref=ref,
                commit=commit,
                archive_format=archive_format,
                archive_size=archive_size,
                archive_sha256=archive_sha256,
                consumers=consumers,
            )
        )
    return tuple(entries)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """解析稳定的依赖验证入口，避免 builder 私自承担锁文件检查。"""
    parser = argparse.ArgumentParser(
        description="Verify stage 4 dependency locks and build environment."
    )
    parser.add_argument(
        "--locks-only",
        action="store_true",
        help="verify only frozen dependency locks and canonical caches",
    )
    parser.add_argument(
        "--lock",
        action="append",
        type=Path,
        default=[],
        help="structured dependency lock to validate; may be supplied more than once",
    )
    parser.add_argument(
        "--system-lock",
        type=Path,
        help="structured Ubuntu 24.04 system dependency lock to validate",
    )
    parser.add_argument("--micromamba", type=Path)
    parser.add_argument("--cmake", type=Path)
    parser.add_argument("--ctest", type=Path)
    parser.add_argument("--cc", type=Path)
    parser.add_argument("--cxx", type=Path)
    parser.add_argument("--protoc", type=Path)
    parser.add_argument("--python-package-cache", type=Path)
    parser.add_argument("--python-wheel-cache", type=Path)
    parser.add_argument("--source-archive-cache", type=Path)
    parser.add_argument("--dependency-prefix", type=Path)
    parser.add_argument("--pcl-pcd2ply", type=Path)
    parser.add_argument("--ldd", type=Path)
    parser.add_argument("--dpkg-query", type=Path)
    parser.add_argument("--mid360-reference-lvx2", type=Path)
    parser.add_argument("--rviz2", type=Path)
    parser.add_argument(
        "--write-env",
        type=Path,
        help="new sourceable Stage 4 environment file to create after probing inputs",
    )
    parser.add_argument(
        "--verify-env",
        type=Path,
        help="sourceable Stage 4 environment file bound to --json evidence",
    )
    parser.add_argument(
        "--json",
        type=Path,
        help="JSON evidence paired with --verify-env",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """验证调用方显式提供的冻结依赖锁，缺少输入时严格失败。"""
    args = _parse_args(argv)
    if args.verify_env is not None:
        if (
            args.locks_only
            or args.lock
            or args.system_lock is not None
            or args.write_env is not None
            or args.json is None
        ):
            print("FAIL: --verify-env requires only --json evidence")
            return 1
        try:
            verify_build_environment(args.verify_env, args.json)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            print(f"FAIL: {error}")
            return 1
        print("PASS: build environment evidence verified")
        return 0
    if args.write_env is not None:
        if args.locks_only or args.lock or args.json is None:
            print("FAIL: --write-env requires only probe inputs and --json evidence")
            return 1
        if args.system_lock is None:
            print("FAIL: --write-env requires --system-lock")
            return 1
        try:
            environment, system_dependencies = _probe_build_environment_from_inputs(
                cmake=args.cmake,
                ctest=args.ctest,
                cc=args.cc,
                cxx=args.cxx,
                protoc=args.protoc,
                micromamba=args.micromamba,
                python_package_cache=args.python_package_cache,
                python_wheel_cache=args.python_wheel_cache,
                source_archive_cache=args.source_archive_cache,
                dependency_prefix=args.dependency_prefix,
                pcl_pcd2ply=args.pcl_pcd2ply,
                system_lock=args.system_lock,
                ldd=args.ldd,
                dpkg_query=args.dpkg_query,
                mid360_reference_lvx2=args.mid360_reference_lvx2,
                rviz2=args.rviz2,
            )
            verification_inputs = {
                "system_lock": str(_require_regular_file(args.system_lock, "--system-lock")),
                "ldd": str(_require_regular_executable(args.ldd, "ldd")),
                "dpkg_query": str(_require_regular_executable(args.dpkg_query, "dpkg-query")),
            }
            write_build_environment(
                environment,
                args.write_env,
                args.json,
                system_dependencies=system_dependencies,
                runtime_identities=_runtime_identities(
                    _runtime_identity_inputs(
                        environment,
                        system_lock=Path(verification_inputs["system_lock"]),
                        ldd=Path(verification_inputs["ldd"]),
                        dpkg_query=Path(verification_inputs["dpkg_query"]),
                    )
                ),
                verification_inputs=verification_inputs,
            )
        except (OSError, ValueError) as error:
            print(f"FAIL: {error}")
            return 1
        print("PASS: build environment written")
        return 0
    if args.json is not None:
        print("FAIL: --json requires --verify-env")
        return 1
    if args.locks_only:
        if not args.lock and args.system_lock is None:
            print("FAIL: --locks-only requires at least one --lock or --system-lock")
            return 1
        try:
            # 每份锁均由同一 parser 验证，避免 CLI 与库入口规则漂移。
            entry_count = sum(len(load_dependency_lock(path)) for path in args.lock)
            if args.system_lock is not None:
                load_system_dependency_lock(args.system_lock)
        except (OSError, ValueError) as error:
            print(f"FAIL: {error}")
            return 1
        if args.system_lock is not None:
            print(
                "PASS: "
                f"{entry_count} dependency lock entries and system dependency lock verified"
            )
            return 0
        print(f"PASS: {entry_count} dependency lock entries verified")
        return 0
    print("FAIL: select --locks-only once dependency locks are available")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
