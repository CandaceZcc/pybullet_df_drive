"""阶段四 E：在 release staging 内构建并安装 C++ 工具。"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import zipfile


_HEX = re.compile(r"[0-9a-f]{64}")
_SAFE_COMPONENT = re.compile(r"[A-Za-z0-9][A-Za-z0-9._~-]*")
_ECAL_SUBMODULE_PATH = re.compile(r"^\s*path\s*=\s*([^\s]+)\s*$", re.MULTILINE)
_REQUIRED_ECAL_SUBMODULES = frozenset(
    {
        "thirdparty/asio/asio",
        "thirdparty/ecaludp/ecaludp",
        "thirdparty/protozero/protozero",
        "thirdparty/recycle/recycle",
        "thirdparty/tclap/tclap",
        "thirdparty/tcp_pubsub/tcp_pubsub",
        "thirdparty/yaml-cpp/yaml-cpp",
    }
)
_GIB = 1024**3
_BUILD_RESERVED_BYTES = 2 * _GIB
_BUILD_BYTES_PER_JOB = 1536 * 1024**2
_BUILD_JOBS_CAP = 8
_LIVOX_VIEWER_DEPENDENCY = "livox-viewer2-linux"
_LIVOX_VIEWER_DIRECTORY = "Viewer2_2.6.0_Linux"
_LIVOX_VIEWER_MAX_FILES = 128
_LIVOX_VIEWER_MAX_BYTES = 512 * 1024 * 1024


@dataclass(frozen=True)
class _CMakeBuildContext:
    """保存一次安装内所有 CMake 阶段共用的并行度、launcher 与环境。"""

    jobs: str
    configure_options: tuple[str, ...]
    environment: dict[str, str]


def _resolve_build_jobs(
    *, cpu_count: int | None, mem_available_bytes: int | None, override: str | None
) -> int:
    """按 CPU 与可用内存选择安全并行度，并严格校验显式覆盖。"""
    if override is not None:
        if (
            cpu_count is None
            or cpu_count < 1
            or not override.isascii()
            or not override.isdecimal()
        ):
            raise ValueError("SLOPE_SIM_BUILD_JOBS must be an integer in 1..CPU count")
        jobs = int(override)
        if jobs < 1 or jobs > cpu_count:
            raise ValueError("SLOPE_SIM_BUILD_JOBS must be an integer in 1..CPU count")
        return jobs
    if cpu_count is None or cpu_count < 1 or mem_available_bytes is None:
        return 1
    memory_jobs = max(
        1,
        (mem_available_bytes - _BUILD_RESERVED_BYTES) // _BUILD_BYTES_PER_JOB,
    )
    return min(cpu_count, _BUILD_JOBS_CAP, memory_jobs)


def _mem_available_bytes(path: Path = Path("/proc/meminfo")) -> int | None:
    """读取 Linux MemAvailable；非 Linux 或格式异常时让调用方安全降级。"""
    try:
        for line in path.read_text(encoding="ascii").splitlines():
            match = re.fullmatch(r"MemAvailable:\s+(\d+)\s+kB", line)
            if match:
                return int(match.group(1)) * 1024
    except (OSError, UnicodeError, ValueError):
        pass
    return None


def _cmake_build_context(
    release_root: Path,
    *,
    base_environment: dict[str, str],
    cpu_count: int | None,
    mem_available_bytes: int | None,
    build_jobs_override: str | None,
    ccache: str | None,
) -> _CMakeBuildContext:
    """为整个 release 安装只计算一次 CMake 并行与 ccache 配置。"""
    jobs = _resolve_build_jobs(
        cpu_count=cpu_count,
        mem_available_bytes=mem_available_bytes,
        override=build_jobs_override,
    )
    environment = base_environment.copy()
    options: tuple[str, ...] = ()
    if ccache is not None:
        options = (
            f"-DCMAKE_C_COMPILER_LAUNCHER={ccache}",
            f"-DCMAKE_CXX_COMPILER_LAUNCHER={ccache}",
        )
        environment["CCACHE_BASEDIR"] = str(release_root)
        environment["CCACHE_MAXSIZE"] = "5G"
    return _CMakeBuildContext(str(jobs), options, environment)


def _require_release_directory(path: Path, *, label: str) -> Path:
    """只接受绝对、非链接的 release 目录，避免构建输出逃逸 staging。"""
    if not path.is_absolute() or path.is_symlink() or not path.is_dir():
        raise ValueError(f"{label} must be an absolute non-symlink directory")
    return path


def _require_child_directory(parent: Path, path: Path, *, label: str) -> Path:
    """限制依赖目录属于同一 release，禁止 setup 消费调用方任意路径。"""
    candidate = _require_release_directory(path, label=label)
    if candidate.parent != parent:
        raise ValueError(f"{label} must be a direct child of release-root")
    return candidate


def _with_ros(value: str) -> bool:
    """把安装器传入的布尔身份转换为稳定 CMake 选项。"""
    if value == "true":
        return True
    if value == "false":
        return False
    raise ValueError("with-ros must be true or false")


def _sha256_file(path: Path) -> str:
    """流式计算普通 dependency 文件摘要，避免索引只校验字符串。"""
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_locked_dependencies(directory: Path) -> list[dict[str, str]]:
    """严格核验安装器生成的 dependency 索引及每个已下载文件。"""
    index = directory / "locked-dependencies.json"
    try:
        if index.is_symlink() or not index.is_file():
            raise ValueError
        document = json.loads(index.read_text(encoding="utf-8"))
        if not isinstance(document, dict) or set(document) != {"schema_version", "dependencies"}:
            raise ValueError
        dependencies = document["dependencies"]
        if document["schema_version"] != 1 or not isinstance(dependencies, list):
            raise ValueError
        names: set[str] = set()
        filenames: set[str] = set()
        validated: list[dict[str, str]] = []
        for dependency in dependencies:
            if not isinstance(dependency, dict) or set(dependency) != {
                "name", "license", "filename", "sha256"
            }:
                raise ValueError
            name = dependency["name"]
            license_name = dependency["license"]
            filename = dependency["filename"]
            digest = dependency["sha256"]
            relative = Path(filename) if isinstance(filename, str) else None
            if (
                not isinstance(name, str)
                or not _SAFE_COMPONENT.fullmatch(name)
                or not isinstance(license_name, str)
                or not license_name.strip()
                or relative is None
                or relative.parts != ("dependencies", relative.name)
                or not _SAFE_COMPONENT.fullmatch(relative.name)
                or not isinstance(digest, str)
                or not _HEX.fullmatch(digest)
                or name in names
                or filename in filenames
            ):
                raise ValueError
            target = directory / relative.name
            if target.is_symlink() or not target.is_file() or _sha256_file(target) != digest:
                raise ValueError
            names.add(name)
            filenames.add(filename)
            validated.append({"name": name, "filename": filename, "sha256": digest, "license": license_name})
        return validated
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError, TypeError):
        raise ValueError("locked dependency index is invalid") from None


def _extract_locked_archives(
    directory: Path, dependencies: list[dict[str, str]], destination: Path
) -> None:
    """只把本轮锁定 tar 源码解压到临时 build tree，拒绝链接和路径逃逸。"""
    for dependency in dependencies:
        filename = dependency["filename"].removeprefix("dependencies/")
        if not filename.endswith((".tar.gz", ".tar.xz", ".tgz")):
            continue
        target_root = destination / dependency["name"]
        try:
            with tarfile.open(directory / filename, "r:*") as archive:
                members = archive.getmembers()
                roots = {Path(member.name).parts[0] for member in members if member.name}
                if (
                    not members
                    or len(roots) != 1
                    or any(
                        not member.name
                        or Path(member.name).is_absolute()
                        or any(part in {"", ".", ".."} for part in Path(member.name).parts)
                        or not (member.isdir() or member.isfile())
                        for member in members
                    )
                ):
                    raise ValueError
                target_root.mkdir(mode=0o700, parents=True, exist_ok=False)
                top = next(iter(roots))
                for member in members:
                    relative = Path(member.name).relative_to(top)
                    if relative == Path("."):
                        continue
                    target = target_root / relative
                    if member.isdir():
                        target.mkdir(mode=0o700, parents=True, exist_ok=False)
                        continue
                    source = archive.extractfile(member)
                    if source is None:
                        raise ValueError
                    target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                    with source, target.open("xb") as output:
                        shutil.copyfileobj(source, output)
        except (OSError, tarfile.TarError, ValueError):
            raise ValueError("locked dependency archive is invalid") from None


def _locked_source_root(build_root: Path, name: str) -> Path | None:
    """返回已临时解压的单个锁定源码根；缺失时由调用方决定是否为必需项。"""
    source = build_root / "sources" / name
    if not source.exists():
        return None
    if source.is_symlink() or not source.is_dir():
        raise ValueError("locked dependency source is invalid")
    return source


def _locked_livox_message_source(build_root: Path) -> Path | None:
    """只接受锁定 Livox 归档内的两份官方 ROS 消息定义。"""
    source = _locked_source_root(build_root, "livox_ros_driver2")
    if source is None:
        return None
    for name in ("CustomMsg.msg", "CustomPoint.msg"):
        message = source / "msg" / name
        if message.is_symlink() or not message.is_file():
            raise ValueError("locked Livox message source is invalid")
    return source


def _locked_mcap_include_directory(build_root: Path) -> Path | None:
    """定位锁定 MCAP C++ 归档的唯一公开头文件根。"""
    source = _locked_source_root(build_root, "mcap")
    if source is None:
        return None
    include = source / "cpp" / "mcap" / "include"
    header = include / "mcap" / "mcap.hpp"
    if (
        include.is_symlink()
        or not include.is_dir()
        or header.is_symlink()
        or not header.is_file()
    ):
        raise ValueError("locked MCAP source is invalid")
    return include


def _locked_dependency_path(
    dependencies_dir: Path, dependencies: list[dict[str, str]], name: str
) -> Path | None:
    """从已核验的索引按名称定位唯一下载文件，拒绝 setup 自行扫描目录。"""
    matches = [dependency for dependency in dependencies if dependency["name"] == name]
    if not matches:
        return None
    filename = matches[0]["filename"].removeprefix("dependencies/")
    return dependencies_dir / filename


def _install_locked_livox_viewer(
    release_root: Path,
    dependencies_dir: Path,
    dependencies: list[dict[str, str]],
) -> Path | None:
    """安全解包锁定的官方 Viewer，并固定到 release 内的运行时路径。"""
    bundle = _locked_dependency_path(dependencies_dir, dependencies, _LIVOX_VIEWER_DEPENDENCY)
    if bundle is None:
        return None
    destination = release_root / "share" / "slope-sim" / "livox-viewer"
    viewer_root = destination / _LIVOX_VIEWER_DIRECTORY
    if bundle.is_symlink() or not bundle.is_file() or bundle.suffix != ".zip":
        raise ValueError("locked Livox Viewer bundle is invalid")
    if destination.exists() or destination.is_symlink():
        raise ValueError("release Livox Viewer directory already exists")
    try:
        with zipfile.ZipFile(bundle) as archive:
            members = archive.infolist()
            total_bytes = sum(member.file_size for member in members)
            if (
                not members
                or len(members) > _LIVOX_VIEWER_MAX_FILES
                or total_bytes > _LIVOX_VIEWER_MAX_BYTES
            ):
                raise ValueError
            for member in members:
                relative = PurePosixPath(member.filename)
                mode = (member.external_attr >> 16) & 0xFFFF
                file_type = stat.S_IFMT(mode)
                if (
                    relative.is_absolute()
                    or not relative.parts
                    or relative.parts[0] != _LIVOX_VIEWER_DIRECTORY
                    or any(part in {"", ".", ".."} for part in relative.parts)
                    or (not member.is_dir() and file_type and not stat.S_ISREG(mode))
                ):
                    raise ValueError
            destination.mkdir(mode=0o755, parents=True, exist_ok=False)
            for member in members:
                target = destination.joinpath(*PurePosixPath(member.filename).parts)
                if member.is_dir():
                    target.mkdir(mode=0o755, parents=True, exist_ok=True)
                    continue
                target.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
                with archive.open(member) as source, target.open("xb") as output:
                    shutil.copyfileobj(source, output)
                target.chmod(0o644)
    except (OSError, RuntimeError, ValueError, zipfile.BadZipFile, zipfile.LargeZipFile):
        if destination.exists() and not destination.is_symlink():
            shutil.rmtree(destination)
        raise ValueError("locked Livox Viewer bundle is invalid") from None
    launcher = viewer_root / "LivoxViewer2.sh"
    binary = viewer_root / "LivoxViewer2" / "Binaries" / "Linux" / "LivoxViewer2"
    if any(path.is_symlink() or not path.is_file() for path in (launcher, binary)):
        shutil.rmtree(destination)
        raise ValueError("locked Livox Viewer bundle is invalid")
    launcher.chmod(0o755)
    binary.chmod(0o755)
    return viewer_root


def _build_cmake_target(
    cmake: str,
    build_directory: Path,
    *,
    environment: dict[str, str] | None = None,
    jobs: str = "1",
) -> None:
    """以本次安装统一计算的并行度构建锁定 CMake 目标。"""
    command = [cmake, "--build", str(build_directory), "--parallel", jobs]
    if environment is None:
        subprocess.run(command, check=True)
    else:
        subprocess.run(command, check=True, env=environment)


def _jazzy_environment() -> dict[str, str]:
    """为 ROS CMake 子进程补齐 Jazzy 的 Python ament 模块，不依赖调用者 source。"""
    roots = sorted((Path("/opt/ros/jazzy") / "lib").glob("python*/site-packages"))
    if len(roots) != 1 or roots[0].is_symlink() or not roots[0].is_dir():
        raise RuntimeError("Jazzy Python site-packages is required for ROS release setup")
    environment = os.environ.copy()
    environment["ROS_DISTRO"] = "jazzy"
    ament_existing = environment.get("AMENT_PREFIX_PATH")
    environment["AMENT_PREFIX_PATH"] = "/opt/ros/jazzy" if not ament_existing else f"/opt/ros/jazzy:{ament_existing}"
    existing = environment.get("PYTHONPATH")
    environment["PYTHONPATH"] = str(roots[0]) if not existing else f"{roots[0]}:{existing}"
    return environment


def _locked_prefix_executable(prefix: Path, name: str) -> Path:
    """只接受 prefix 内普通可执行文件或同目录版本链接，禁止逃逸临时依赖根。"""
    executable = prefix / "bin" / name
    try:
        root = prefix.resolve(strict=True)
        target = executable.resolve(strict=True)
    except OSError:
        raise ValueError("locked dependency executable is invalid") from None
    if executable.is_symlink() and Path(os.readlink(executable)).name != os.readlink(executable):
        raise ValueError("locked dependency executable is invalid")
    if (
        prefix.is_symlink()
        or not prefix.is_dir()
        or not target.is_relative_to(root)
        or not target.is_file()
        or not os.access(target, os.X_OK)
    ):
        raise ValueError("locked dependency executable is invalid")
    return executable


def _ecal_config_directory(prefix: Path) -> Path:
    """定位唯一 eCAL CMake config，禁止模糊地回退到系统安装。"""
    candidates = [
        candidate
        for candidate in prefix.rglob("eCALConfig.cmake")
        if candidate.is_file() and not candidate.is_symlink()
    ]
    if len(candidates) != 1:
        raise ValueError("locked eCAL install is invalid")
    return candidates[0].parent


def _install_locked_ecal_deb(
    dependencies_dir: Path, dependencies: list[dict[str, str]], build_root: Path, prefix: Path | None
) -> Path | None:
    """把官方 Noble eCAL deb 仅解包到本轮临时 prefix，不修改系统 package 数据库。"""
    package = _locked_dependency_path(dependencies_dir, dependencies, "ecal")
    if package is None or not package.name.endswith(".deb"):
        return None
    if prefix is None:
        raise ValueError("locked eCAL deb requires the Protobuf prefix")
    dpkg_deb = shutil.which("dpkg-deb")
    if dpkg_deb is None:
        raise RuntimeError("dpkg-deb is required for the locked eCAL package")
    extracted = build_root / "ecal-deb"
    subprocess.run([dpkg_deb, "--extract", str(package), str(extracted)], check=True)
    payload = extracted / "usr"
    if payload.is_symlink() or not payload.is_dir():
        raise ValueError("locked eCAL deb is invalid")
    files = [candidate for candidate in payload.rglob("*") if candidate.is_file() or candidate.is_symlink()]
    for candidate in files:
        relative = candidate.relative_to(payload)
        target = prefix / relative
        if target.exists() or target.is_symlink():
            raise ValueError("locked eCAL deb conflicts with the dependency prefix")
    for candidate in sorted(payload.rglob("*"), key=lambda item: item.as_posix()):
        relative = candidate.relative_to(payload)
        target = prefix / relative
        if candidate.is_dir():
            target.mkdir(mode=0o755, parents=True, exist_ok=True)
        elif candidate.is_symlink():
            target.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
            target.symlink_to(os.readlink(candidate))
        elif candidate.is_file():
            target.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
            shutil.copy2(candidate, target, follow_symlinks=False)
        else:
            raise ValueError("locked eCAL deb is invalid")
    # Debian 把运行配置放在 /etc 而不是 /usr；release 预检必须能随 prefix 找到它。
    configuration = extracted / "etc" / "ecal" / "ecal.yaml"
    if configuration.exists() or configuration.is_symlink():
        if configuration.is_symlink() or not configuration.is_file():
            raise ValueError("locked eCAL configuration is invalid")
        target = prefix / "etc" / "ecal" / "ecal.yaml"
        if target.exists() or target.is_symlink():
            raise ValueError("locked eCAL configuration conflicts with dependency prefix")
        target.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
        shutil.copy2(configuration, target, follow_symlinks=False)
    return _ecal_config_directory(prefix)


def _create_locked_python_runtime(
    release_root: Path, dependencies_dir: Path, dependencies: list[dict[str, str]], build_root: Path
) -> Path | None:
    """只在存在锁定 micromamba 时，以 payload 内 lock 和系统受信 CA 创建 Python runtime。"""
    micromamba = _locked_dependency_path(dependencies_dir, dependencies, "micromamba")
    if micromamba is None:
        return None
    lock = release_root / "packaging" / "locks" / "python-linux-64.lock"
    if (
        micromamba.is_symlink()
        or not micromamba.is_file()
        or not os.access(micromamba, os.X_OK)
        or lock.is_symlink()
        or not lock.is_file()
    ):
        raise ValueError("locked Python runtime inputs are invalid")
    runtime = release_root / "runtime"
    if runtime.exists() or runtime.is_symlink():
        raise ValueError("locked Python runtime already exists")
    build_root.mkdir(mode=0o755, parents=True, exist_ok=True)
    micromamba_runner = build_root / "micromamba"
    if micromamba_runner.exists() or micromamba_runner.is_symlink():
        raise ValueError("micromamba build runner already exists")
    # Conda 的 post-link 脚本只接受 mamba/micromamba 作为 MAMBA_EXE basename。
    shutil.copy2(micromamba, micromamba_runner, follow_symlinks=False)
    micromamba_runner.chmod(0o755)
    # Conda 官方源在目标机直连可用；不继承慢速全局代理，其他安装下载仍保留其原环境。
    environment = os.environ.copy()
    for name in ("HTTP_PROXY", "HTTPS_PROXY", "http_proxy", "https_proxy"):
        environment.pop(name, None)
    no_proxy = [entry for entry in environment.get("NO_PROXY", "").split(",") if entry]
    if "conda.anaconda.org" not in no_proxy:
        no_proxy.append("conda.anaconda.org")
    environment["NO_PROXY"] = ",".join(no_proxy)
    environment["MAMBA_EXE"] = str(micromamba_runner)
    subprocess.run(
        [
            str(micromamba_runner),
            "create",
            "--no-rc",
            "--no-env",
            "--ssl-verify",
            "true",
            "--root-prefix",
            str(build_root / "mamba-root"),
            "--prefix",
            str(runtime),
            "--file",
            str(lock),
            "--safety-checks",
            "enabled",
            "--yes",
        ],
        check=True,
        env=environment,
    )
    python = _locked_runtime_python(runtime)
    _install_locked_python_wheels(python, dependencies_dir, dependencies)
    subprocess.run(
        [str(python), "-c", "import serial, mcap.reader, pyqtgraph.opengl, OpenGL.GL"],
        check=True,
    )
    return runtime


def _locked_runtime_python(runtime: Path) -> Path:
    """只接受 Conda runtime 内部解析到常规可执行文件的 Python 链接。"""
    python = runtime / "bin" / "python"
    try:
        root = runtime.resolve(strict=True)
        target = python.resolve(strict=True)
    except OSError:
        raise ValueError("locked Python runtime is invalid") from None
    if (
        runtime.is_symlink()
        or not runtime.is_dir()
        or not target.is_relative_to(root)
        or not target.is_file()
        or not os.access(target, os.X_OK)
    ):
        raise ValueError("locked Python runtime is invalid")
    return python


def _install_locked_python_wheels(
    python: Path, dependencies_dir: Path, dependencies: list[dict[str, str]]
) -> None:
    """按依赖顺序离线安装 Python wheels，避免 eCAL 在无 Protobuf 时被掩盖。"""
    wheel_contracts = (
        ("protobuf-python", "import google.protobuf"),
        ("ecal-python", "import ecal.nanobind_core, ecal.msg.proto.core, ecal.msg.common.core"),
    )
    for name, import_probe in wheel_contracts:
        wheel = _locked_dependency_path(dependencies_dir, dependencies, name)
        if wheel is None:
            continue
        if wheel.is_symlink() or not wheel.is_file() or wheel.suffix != ".whl":
            raise ValueError(f"locked {name} Python wheel is invalid")
        subprocess.run(
            [str(python), "-m", "pip", "install", "--no-index", "--no-deps", str(wheel)],
            check=True,
        )
        subprocess.run([str(python), "-c", import_probe], check=True)


def _build_locked_abseil(
    cmake: str, build_root: Path, context: _CMakeBuildContext | None = None
) -> Path | None:
    """先构建锁定 Abseil，供同轮 Protobuf 的离线 CMake configure 使用。"""
    source = _locked_source_root(build_root, "abseil-cpp")
    if source is None:
        return None
    if not (source / "CMakeLists.txt").is_file():
        raise ValueError("locked Abseil source is invalid")
    prefix = build_root / "prefix"
    dependency_build = build_root / "dependencies" / "abseil-cpp"
    subprocess.run(
        [
            cmake,
            "-S",
            str(source),
            "-B",
            str(dependency_build),
            "-DCMAKE_BUILD_TYPE=Release",
            f"-DCMAKE_INSTALL_PREFIX={prefix}",
            f"-DCMAKE_PREFIX_PATH={prefix}",
            "-DABSL_BUILD_TESTING=OFF",
            "-DABSL_ENABLE_INSTALL=ON",
            "-DABSL_PROPAGATE_CXX_STD=ON",
            *(context.configure_options if context else ()),
        ],
        check=True,
        **({"env": context.environment} if context else {}),
    )
    _build_cmake_target(
        cmake,
        dependency_build,
        environment=context.environment if context else None,
        jobs=context.jobs if context else "1",
    )
    subprocess.run(
        [cmake, "--install", str(dependency_build)],
        check=True,
        **({"env": context.environment} if context else {}),
    )
    config = prefix / "lib" / "cmake" / "absl" / "abslConfig.cmake"
    if config.is_symlink() or not config.is_file():
        raise ValueError("locked Abseil install is invalid")
    return prefix


def _build_locked_protobuf(
    cmake: str,
    build_root: Path,
    prefix: Path | None,
    context: _CMakeBuildContext | None = None,
) -> Path | None:
    """只在本轮临时前缀构建锁定 Protobuf，供随后同一 CMake configure 使用。"""
    source = _locked_source_root(build_root, "protobuf")
    if source is None:
        return None
    if not (source / "CMakeLists.txt").is_file():
        raise ValueError("locked Protobuf source is invalid")
    if prefix is None:
        prefix = build_root / "prefix"
    dependency_build = build_root / "dependencies" / "protobuf"
    subprocess.run(
        [
            cmake,
            "-S",
            str(source),
            "-B",
            str(dependency_build),
            "-DCMAKE_BUILD_TYPE=Release",
            f"-DCMAKE_INSTALL_PREFIX={prefix}",
            f"-DCMAKE_PREFIX_PATH={prefix}",
            "-Dprotobuf_BUILD_TESTS=OFF",
            "-Dprotobuf_LOCAL_DEPENDENCIES_ONLY=ON",
            *(context.configure_options if context else ()),
        ],
        check=True,
        **({"env": context.environment} if context else {}),
    )
    _build_cmake_target(
        cmake,
        dependency_build,
        environment=context.environment if context else None,
        jobs=context.jobs if context else "1",
    )
    subprocess.run(
        [cmake, "--install", str(dependency_build)],
        check=True,
        **({"env": context.environment} if context else {}),
    )
    try:
        _locked_prefix_executable(prefix, "protoc")
    except ValueError:
        raise ValueError("locked Protobuf install is invalid") from None
    return prefix


def _verify_ecal_source_closure(source: Path) -> None:
    """拒绝缺少任一声明子模块的 eCAL 源码，安装器不再补取或填充源码。"""
    modules = source / ".gitmodules"
    if modules.is_symlink() or not modules.is_file():
        raise ValueError("locked eCAL source closure is invalid")
    paths = set(_ECAL_SUBMODULE_PATH.findall(modules.read_text(encoding="utf-8")))
    if not _REQUIRED_ECAL_SUBMODULES.issubset(paths):
        raise ValueError("locked eCAL source closure is invalid")
    for relative in paths:
        candidate = Path(relative)
        target = source / candidate
        if (
            candidate.is_absolute()
            or not relative.startswith("thirdparty/")
            or ".." in candidate.parts
            or target.is_symlink()
            or not target.is_dir()
            or not any(target.iterdir())
        ):
            raise ValueError("locked eCAL source closure is invalid")


def _build_locked_ecal(
    cmake: str,
    build_root: Path,
    prefix: Path | None,
    context: _CMakeBuildContext | None = None,
) -> Path | None:
    """以本轮 Protobuf prefix 构建完整 eCAL，禁止 FetchContent 或系统包回退。"""
    source = _locked_source_root(build_root, "ecal")
    if source is None:
        return None
    if prefix is None or not (source / "CMakeLists.txt").is_file():
        raise ValueError("locked eCAL source is invalid")
    _verify_ecal_source_closure(source)
    top_level = source / "cmake" / "submodule_dependencies.cmake"
    if top_level.is_symlink() or not top_level.is_file():
        raise ValueError("locked eCAL source is invalid")
    cmakefunctions = source / "thirdparty" / "cmakefunctions" / "cmake_functions"
    if cmakefunctions.is_symlink() or not (cmakefunctions / "CMakeLists.txt").is_file():
        raise ValueError("locked eCAL source closure is invalid")
    functions_build = build_root / "dependencies" / "ecal-cmakefunctions"
    subprocess.run(
        [
            cmake,
            "-S",
            str(cmakefunctions),
            "-B",
            str(functions_build),
            "-DCMAKE_BUILD_TYPE=Release",
            f"-DCMAKE_INSTALL_PREFIX={prefix}",
            f"-DCMAKE_PREFIX_PATH={prefix}",
            "-DFETCHCONTENT_FULLY_DISCONNECTED=ON",
            "-DFETCHCONTENT_UPDATES_DISCONNECTED=ON",
            "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=FALSE",
            "-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=FALSE",
            *(context.configure_options if context else ()),
        ],
        check=True,
        **({"env": context.environment} if context else {}),
    )
    _build_cmake_target(
        cmake,
        functions_build,
        environment=context.environment if context else None,
        jobs=context.jobs if context else "1",
    )
    subprocess.run(
        [cmake, "--install", str(functions_build)],
        check=True,
        **({"env": context.environment} if context else {}),
    )
    dependency_build = build_root / "dependencies" / "ecal"
    subprocess.run(
        [
            cmake,
            "-S",
            str(source),
            "-B",
            str(dependency_build),
            "-DCMAKE_BUILD_TYPE=Release",
            f"-DCMAKE_INSTALL_PREFIX={prefix}",
            f"-DCMAKE_PREFIX_PATH={prefix}",
            f"-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES={top_level}",
            "-DECAL_BUILD_VERSION=6.1.1",
            "-DECAL_USE_HDF5=OFF",
            "-DECAL_USE_QT=OFF",
            "-DECAL_USE_CURL=OFF",
            "-DECAL_USE_FTXUI=OFF",
            "-DECAL_USE_PROTOBUF=OFF",
            "-DECAL_BUILD_APPS=OFF",
            "-DECAL_BUILD_SAMPLES=OFF",
            "-DECAL_BUILD_C_BINDING=OFF",
            "-DECAL_BUILD_CSHARP_BINDING=OFF",
            "-DECAL_BUILD_PY_BINDING=OFF",
            "-DECAL_CORE_CONFIGURATION=OFF",
            "-DECAL_INSTALL_SAMPLE_SOURCES=OFF",
            "-DFETCHCONTENT_FULLY_DISCONNECTED=ON",
            "-DFETCHCONTENT_UPDATES_DISCONNECTED=ON",
            "-DCMAKE_FIND_USE_PACKAGE_REGISTRY=FALSE",
            "-DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=FALSE",
            *(context.configure_options if context else ()),
        ],
        check=True,
        **({"env": context.environment} if context else {}),
    )
    _build_cmake_target(
        cmake,
        dependency_build,
        environment=context.environment if context else None,
        jobs=context.jobs if context else "1",
    )
    subprocess.run(
        [cmake, "--install", str(dependency_build)],
        check=True,
        **({"env": context.environment} if context else {}),
    )
    return _ecal_config_directory(prefix)


def _install_ecal_runtime_config(prefix: Path | None, release_root: Path) -> None:
    """把锁定 eCAL 的唯一配置复制到 release，避免运行期引用临时 build。"""
    if prefix is None:
        return
    source = prefix / "etc" / "ecal" / "ecal.yaml"
    # eCAL 的锁定核心包可以不携带覆盖配置；此时保持其内建默认值而非把空文件写入 release。
    if source.is_symlink():
        raise ValueError("locked eCAL configuration is invalid")
    if not source.is_file():
        return
    destination = release_root / "etc" / "ecal" / "ecal.yaml"
    if destination.exists() or destination.is_symlink():
        raise ValueError("release eCAL configuration conflicts with release")
    destination.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
    shutil.copy2(source, destination, follow_symlinks=False)


def _copy_locked_runtime_libraries(prefix: Path | None, release_root: Path) -> None:
    """把临时 prefix 的共享库带入 release，禁止已安装工具依赖即将删除的 build tree。"""
    if prefix is None:
        return
    library_dir = prefix / "lib"
    if library_dir.is_symlink():
        raise ValueError("locked runtime library directory is invalid")
    if not library_dir.is_dir():
        return
    destination = release_root / "lib"
    if destination.is_symlink():
        raise ValueError("release runtime library directory is invalid")
    destination.mkdir(mode=0o755, exist_ok=True)
    for source in sorted(library_dir.rglob("*"), key=lambda path: path.as_posix()):
        if source.is_dir():
            continue
        if not source.name.startswith("lib") or ".so" not in source.name:
            continue
        target = destination / source.name
        if target.exists() or target.is_symlink():
            raise ValueError("locked runtime library conflicts with release")
        if source.is_symlink():
            link = os.readlink(source)
            if Path(link).name != link:
                raise ValueError("locked runtime library link is invalid")
            target.symlink_to(link)
        elif source.is_file():
            shutil.copy2(source, target, follow_symlinks=False)
        else:
            raise ValueError("locked runtime library is invalid")


def _materialize_internal_release_file_links(release_root: Path) -> None:
    """将运行时产生的内部文件链接物化，保持外层 doctor 的无链接发布契约。"""
    root = release_root.resolve(strict=True)
    links = [
        candidate
        for candidate in sorted(root.rglob("*"), key=lambda path: path.as_posix())
        if candidate.is_symlink()
    ]
    for link in links:
        relative = link.relative_to(root).as_posix()
        try:
            target = link.resolve(strict=True)
        except OSError:
            raise ValueError(f"release runtime link is invalid: {relative}") from None
        if not target.is_relative_to(root):
            raise ValueError(f"release runtime link is invalid: {relative}")
        if target.is_dir():
            if link.is_relative_to(target):
                raise ValueError(f"release runtime link is invalid: {relative}")
            link.unlink()
            shutil.copytree(target, link, symlinks=False)
            continue
        if not target.is_file():
            raise ValueError(f"release runtime link is invalid: {relative}")
        link.unlink()
        shutil.copy2(target, link)


def _install_ros_bridge_launcher(release_root: Path) -> None:
    """以固定 Jazzy setup 包装 ROS bridge，避免调用者手工 source 才能加载 ROS 库。"""
    bridge = release_root / "bin" / "slope_sim_stage4_ros2_bridge"
    binary = release_root / "bin" / "slope_sim_stage4_ros2_bridge.bin"
    if (
        bridge.is_symlink()
        or not bridge.is_file()
        or not os.access(bridge, os.X_OK)
        or binary.exists()
        or binary.is_symlink()
    ):
        raise ValueError("installed ROS bridge is invalid")
    bridge.rename(binary)
    launcher = (
        "#!/bin/sh\n"
        "set -e\n"
        ". /opt/ros/jazzy/setup.sh\n"
        'release_root="$(CDPATH= cd -- "$(dirname "$0")/.." && pwd)"\n'
        'export LD_LIBRARY_PATH="$release_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\n'
        'livox_setup="$release_root/share/livox_ros_driver2/local_setup.sh"\n'
        'if [ -f "$livox_setup" ]; then . "$livox_setup"; fi\n'
        'exec "$(dirname "$0")/slope_sim_stage4_ros2_bridge.bin" "$@"\n'
    )
    with bridge.open("x", encoding="utf-8") as output:
        output.write(launcher)
    bridge.chmod(0o755)


def _install_run_sim_launcher(release_root: Path) -> None:
    """将 payload 中已校验的 runSim 复制到 release 的公共 bin 入口。"""
    source = release_root / "runSim"
    destination = release_root / "bin" / "runSim"
    if (
        source.is_symlink()
        or not source.is_file()
        or not os.access(source, os.X_OK)
        or destination.exists()
        or destination.is_symlink()
    ):
        raise ValueError("release runSim launcher is invalid")
    destination.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
    shutil.copy2(source, destination)
    destination.chmod(0o755)


def _write_setup_result(release_root: Path, *, with_ros: bool) -> None:
    """写入完成标记，供外层 doctor 将真实安装产物纳入 manifest。"""
    result = release_root / "share" / "slope-sim" / "runtime-setup.json"
    result.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
    with result.open("x", encoding="utf-8") as output:
        json.dump({"cpp_tools_built": True, "with_ros": with_ros}, output, sort_keys=True)
        output.write("\n")


def build_cpp_tools(
    release_root: Path, dependencies_dir: Path, locked_dependencies: list[dict[str, str]], *, with_ros: bool
) -> None:
    """从内嵌 cpp/phase0 构建工具并仅把 install 产物保留在 release。"""
    source = release_root / "cpp" / "phase0"
    if source.is_symlink() or not source.is_dir() or not (source / "CMakeLists.txt").is_file():
        raise ValueError("release C++ source is invalid")
    cmake = shutil.which("cmake")
    if cmake is None:
        raise RuntimeError("cmake is required for release setup")
    context = _cmake_build_context(
        release_root,
        base_environment=os.environ.copy(),
        cpu_count=os.cpu_count(),
        mem_available_bytes=_mem_available_bytes(),
        build_jobs_override=os.environ.get("SLOPE_SIM_BUILD_JOBS"),
        ccache=shutil.which("ccache"),
    )
    print(
        "release CMake build: "
        f"jobs={context.jobs}, ccache={'enabled' if context.configure_options else 'disabled'}",
        flush=True,
    )
    build_root = release_root / ".stage4-build"
    if build_root.exists() or build_root.is_symlink():
        raise ValueError("release temporary build directory already exists")
    try:
        runtime = _create_locked_python_runtime(release_root, dependencies_dir, locked_dependencies, build_root)
        _extract_locked_archives(dependencies_dir, locked_dependencies, build_root / "sources")
        livox_message_source = _locked_livox_message_source(build_root) if with_ros else None
        if with_ros and livox_message_source is None:
            raise ValueError("locked Livox message source is missing")
        prefix = _build_locked_abseil(cmake, build_root, context)
        prefix = _build_locked_protobuf(cmake, build_root, prefix, context)
        ecal_dir = _install_locked_ecal_deb(dependencies_dir, locked_dependencies, build_root, prefix)
        if ecal_dir is None:
            ecal_dir = _build_locked_ecal(cmake, build_root, prefix, context)
        command = [
            cmake,
            "-S",
            str(source),
            "-B",
            str(build_root),
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCMAKE_INSTALL_RPATH=$ORIGIN/../lib",
            f"-DSTAGE4_WITH_ROS={'ON' if with_ros else 'OFF'}",
            *context.configure_options,
        ]
        cmake_prefixes = [str(prefix)] if prefix is not None else []
        cmake_environment = context.environment.copy()
        if with_ros:
            cmake_prefixes.append("/opt/ros/jazzy")
            command.append(f"-DSTAGE4_LIVOX_MSG_SOURCE={livox_message_source}")
            command.append("-DSTAGE4_ROSIDL_PYTHON_EXECUTABLE=/usr/bin/python3")
            jazzy_environment = _jazzy_environment()
            for name in ("ROS_DISTRO", "AMENT_PREFIX_PATH", "PYTHONPATH"):
                cmake_environment[name] = jazzy_environment[name]
        if cmake_prefixes:
            command.append(f"-DCMAKE_PREFIX_PATH={';'.join(cmake_prefixes)}")
        if runtime is not None:
            command.append(f"-DPython3_EXECUTABLE={_locked_runtime_python(runtime)}")
        if prefix is not None:
            command.extend(
                [
                    f"-DSTAGE4_PROTOC_EXECUTABLE={prefix / 'bin' / 'protoc'}",
                ]
            )
        if ecal_dir is not None:
            command.append(f"-DeCAL_DIR={ecal_dir}")
        mcap_include = _locked_mcap_include_directory(build_root)
        if mcap_include is not None:
            command.append(f"-DSTAGE4_MCAP_INCLUDE_DIR={mcap_include}")
        subprocess.run(command, check=True, env=cmake_environment)
        _build_cmake_target(
            cmake,
            build_root,
            environment=cmake_environment,
            jobs=context.jobs,
        )
        install_command = [cmake, "--install", str(build_root), "--prefix", str(release_root)]
        subprocess.run(install_command, check=True, env=cmake_environment)
        # 通用 CMake 夹具可只验证 Protobuf/MCAP；只有实际安装 eCAL 后才有配置可发布。
        if ecal_dir is not None:
            _install_ecal_runtime_config(prefix, release_root)
        _copy_locked_runtime_libraries(prefix, release_root)
        if with_ros and (release_root / "bin" / "slope_sim_stage4_ros2_bridge").exists():
            _install_ros_bridge_launcher(release_root)
        # 通用 CMake fixture 不含项目 payload；真实 `.run` 由 payload 合同保证此文件存在。
        if (release_root / "runSim").exists():
            _install_run_sim_launcher(release_root)
        _install_locked_livox_viewer(release_root, dependencies_dir, locked_dependencies)
    finally:
        if build_root.exists() or build_root.is_symlink():
            shutil.rmtree(build_root)
    _materialize_internal_release_file_links(release_root)
    _write_setup_result(release_root, with_ros=with_ros)


def main(argv: list[str] | None = None) -> int:
    """解析固定 installer 协议并运行一次 staging 内的 C++ 安装。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--release-root", type=Path, required=True)
    parser.add_argument("--dependencies-dir", type=Path, required=True)
    parser.add_argument("--with-ros", required=True)
    args = parser.parse_args(argv)
    release_root = _require_release_directory(args.release_root, label="release-root")
    dependencies_dir = _require_child_directory(release_root, args.dependencies_dir, label="dependencies-dir")
    locked_dependencies = _load_locked_dependencies(dependencies_dir)
    build_cpp_tools(release_root, dependencies_dir, locked_dependencies, with_ros=_with_ros(args.with_ros))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
