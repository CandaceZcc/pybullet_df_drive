# 阶段四依赖门合同：先固定公开 verifier 入口，再逐步加入离线锁与归档 fixture。
from __future__ import annotations

import importlib.util
import io
import json
from pathlib import Path
import hashlib
import subprocess
import sys
import tarfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
DEPENDENCY_VERIFIER = ROOT / "scripts" / "verify_stage4_dependencies.py"
SOURCE_ARCHIVE_PARSER = ROOT / "scripts" / "stage4_source_archive.py"
SOURCE_CACHE_VERIFIER = ROOT / "scripts" / "verify_stage4_source_cache.py"
SOURCE_CACHE_FREEZER = ROOT / "scripts" / "freeze_stage4_source_cache.py"
DEPENDENCY_SOURCE_MATERIALIZER = ROOT / "scripts" / "materialize_stage4_dependency_sources.py"
DEPENDENCY_BUILDER = ROOT / "packaging" / "build_dependencies.sh"
UBUNTU24_CONTAINER_BUILDER = ROOT / "packaging" / "build_ubuntu24_dependencies_container.py"
NETWORK_WRAPPER = ROOT / "packaging" / "run_network_isolated.sh"
CPP_DEPENDENCY_LOCK = ROOT / "packaging" / "locks" / "cpp-dependencies.lock"
ROS2_DEPENDENCY_LOCK = ROOT / "packaging" / "locks" / "ros2-dependencies.lock"


def _system_dependency_probe_inputs(tmp_path: Path) -> dict[str, Path]:
    """创建最小锁定系统 DSO 输入，避免环境 probe 测试读取真实宿主包。"""
    system_lock = tmp_path / "ubuntu24-system-dependencies.lock"
    system_lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "platform": {
                    "id": "ubuntu",
                    "version_id": "24.04",
                    "codename": "noble",
                    "architecture": "amd64",
                },
                "builder_image": {
                    "reference": "ubuntu:24.04",
                    "digest": "sha256:" + "a" * 64,
                },
                "apt_packages": [
                    {
                        "name": "libfixture1:amd64",
                        "version": "1.2.3-1",
                        "architecture": "amd64",
                    }
                ],
                "allowed_system_sonames": [
                    {
                        "soname": "libfixture.so.1",
                        "package": "libfixture1:amd64",
                        "version": "1.2.3-1",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    ldd = tmp_path / "ldd"
    ldd.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' 'libfixture.so.1 => /lib/x86_64-linux-gnu/libfixture.so.1 (0x1)'\n",
        encoding="utf-8",
    )
    ldd.chmod(0o755)
    dpkg_query = tmp_path / "dpkg-query"
    dpkg_query.write_text(
        "#!/bin/sh\nprintf 'installed\\t1.2.3-1\\n'\n",
        encoding="utf-8",
    )
    dpkg_query.chmod(0o755)
    return {
        "system_lock": system_lock,
        "ldd": ldd,
        "dpkg_query": dpkg_query,
    }


def _load_verifier() -> object:
    """按 CLI 同一源码加载 verifier，测试不复制生产解析逻辑。"""
    spec = importlib.util.spec_from_file_location(
        "stage4_dependency_verifier",
        DEPENDENCY_VERIFIER,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
        return module
    finally:
        sys.modules.pop(spec.name, None)


def _complete_write_env_probe_inputs(tmp_path: Path) -> dict[str, Path]:
    """创建完整有效的显式 probe 输入，供单项 CLI 负例仅破坏目标值。"""
    version_output = {
        "cmake": "cmake version 3.28.9",
        "ctest": "ctest version 3.28.9",
        "cc": "gcc 13.3.0",
        "cxx": "g++ 13.3.0",
        "protoc": "libprotoc 33.6",
    }
    inputs: dict[str, Path] = {}
    for name, output in version_output.items():
        path = tmp_path / name
        path.write_text(f"#!/bin/sh\nprintf '{output}\\n'\n", encoding="utf-8")
        path.chmod(0o755)
        inputs[name] = path
    for name in ("micromamba", "pcl_pcd2ply", "rviz2"):
        path = (
            tmp_path / "validation-prefix" / "bin" / name
            if name == "pcl_pcd2ply"
            else tmp_path / name
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        path.chmod(0o755)
        inputs[name] = path
    for name in (
        "python_package_cache",
        "python_wheel_cache",
        "source_archive_cache",
        "dependency_prefix",
    ):
        path = tmp_path / name
        path.mkdir()
        inputs[name] = path
    inputs["mid360_reference_lvx2"] = tmp_path / "Indoor_sampledata.lvx2"
    inputs["mid360_reference_lvx2"].write_bytes(b"fixture LVX2")
    inputs.update(_system_dependency_probe_inputs(tmp_path))
    return inputs


def test_dependency_prefix_identity_allows_only_resolved_in_tree_soname_links(tmp_path) -> None:
    """私有依赖 prefix 的 ELF SONAME 链接必须可重放，缓存链接仍由专用 verifier 拒绝。"""
    prefix = tmp_path / "dependency-prefix"
    library = prefix / "lib" / "libfixture.so.1.2.3"
    library.parent.mkdir(parents=True)
    library.write_bytes(b"fixture shared object")
    (library.parent / "libfixture.so").symlink_to("libfixture.so.1.2.3")

    verifier = _load_verifier()
    identity = getattr(verifier, "_path_identity", None)
    assert callable(identity), "dependency verifier needs runtime identity support"

    record = identity(prefix, "STAGE4_DEPENDENCY_PREFIX", allow_in_tree_symlinks=True)

    assert record["kind"] == "directory"
    assert isinstance(record["tree_sha256"], str)


def _write_env_probe_command(inputs: dict[str, Path], tmp_path: Path) -> list[str]:
    """按公开 CLI 组装完整 probe 调用，禁止测试依赖历史默认路径。"""
    return [
        sys.executable,
        str(DEPENDENCY_VERIFIER),
        "--cmake",
        str(inputs["cmake"]),
        "--ctest",
        str(inputs["ctest"]),
        "--cc",
        str(inputs["cc"]),
        "--cxx",
        str(inputs["cxx"]),
        "--protoc",
        str(inputs["protoc"]),
        "--micromamba",
        str(inputs["micromamba"]),
        "--python-package-cache",
        str(inputs["python_package_cache"]),
        "--python-wheel-cache",
        str(inputs["python_wheel_cache"]),
        "--source-archive-cache",
        str(inputs["source_archive_cache"]),
        "--dependency-prefix",
        str(inputs["dependency_prefix"]),
        "--pcl-pcd2ply",
        str(inputs["pcl_pcd2ply"]),
        "--system-lock",
        str(inputs["system_lock"]),
        "--ldd",
        str(inputs["ldd"]),
        "--dpkg-query",
        str(inputs["dpkg_query"]),
        "--mid360-reference-lvx2",
        str(inputs["mid360_reference_lvx2"]),
        "--rviz2",
        str(inputs["rviz2"]),
        "--write-env",
        str(tmp_path / "stage4-build-env.sh"),
        "--json",
        str(tmp_path / "stage4-build-env.json"),
    ]


def _load_source_archive_parser() -> object:
    """按生产源码加载归档 parser，避免安全规则由测试自行实现。"""
    spec = importlib.util.spec_from_file_location(
        "stage4_source_archive_parser",
        SOURCE_ARCHIVE_PARSER,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
        return module
    finally:
        sys.modules.pop(spec.name, None)


def _freeze_fixture_cpp_source(
    tmp_path: Path,
    files: dict[str, bytes] | None = None,
    dependency_name: str = "fixture",
) -> tuple[Path, Path, Path]:
    """生成只供 builder 合同测试使用的本地冻结 C++ 源码 cache。"""
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    archive = source_dir / f"{dependency_name}.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        for relative_path, payload in sorted((files or {"value.txt": b"value"}).items()):
            member = tarfile.TarInfo(f"{dependency_name}-root/{relative_path}")
            member.size = len(payload)
            handle.addfile(member, io.BytesIO(payload))
    payload = archive.read_bytes()
    lock = tmp_path / "dependency.lock"
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": dependency_name,
                        "url": f"https://example.invalid/{dependency_name}.tar.gz",
                        "ref_kind": "commit",
                        "ref": "a" * 40,
                        "commit": "a" * 40,
                        "consumers": ["cpp_dependency"],
                        "archive": {
                            "format": "tar.gz",
                            "size": len(payload),
                            "sha256": hashlib.sha256(payload).hexdigest(),
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    cache = tmp_path / "cache"
    manifest = tmp_path / "manifest.json"
    frozen = subprocess.run(
        [
            sys.executable,
            str(SOURCE_CACHE_FREEZER),
            "--lock",
            str(lock),
            "--source-dir",
            str(source_dir),
            "--cache-root",
            str(cache),
            "--manifest",
            str(manifest),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert frozen.returncode == 0, frozen.stderr
    return cache, manifest, lock


def _freeze_cpp_source_set(
    tmp_path: Path,
    sources: dict[str, dict[str, bytes]],
    consumers: dict[str, list[str]] | None = None,
) -> tuple[Path, Path, Path]:
    """构造包含 eCAL 与其独立 submodule archive 的本地 canonical cache。"""
    source_dir = tmp_path / "source-set"
    source_dir.mkdir()
    records = []
    for name, files in sources.items():
        archive = source_dir / f"{name}.tar.gz"
        with tarfile.open(archive, "w:gz") as handle:
            for relative_path, payload in sorted(files.items()):
                member = tarfile.TarInfo(f"{name}-root/{relative_path}")
                member.size = len(payload)
                handle.addfile(member, io.BytesIO(payload))
        payload = archive.read_bytes()
        records.append(
            {
                "name": name,
                "url": f"https://example.invalid/{name}.tar.gz",
                "ref_kind": "commit",
                "ref": "a" * 40,
                "commit": "a" * 40,
                "consumers": (consumers or {}).get(name, ["cpp_dependency"]),
                "archive": {
                    "format": "tar.gz",
                    "size": len(payload),
                    "sha256": hashlib.sha256(payload).hexdigest(),
                },
            }
        )
    lock = tmp_path / "source-set.lock"
    lock.write_text(json.dumps({"schema_version": 1, "dependencies": records}), encoding="utf-8")
    cache = tmp_path / "source-set-cache"
    manifest = tmp_path / "source-set.manifest.json"
    frozen = subprocess.run(
        [sys.executable, str(SOURCE_CACHE_FREEZER), "--lock", str(lock), "--source-dir", str(source_dir), "--cache-root", str(cache), "--manifest", str(manifest)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert frozen.returncode == 0, frozen.stderr
    return cache, manifest, lock


def test_dependency_verifier_exposes_locks_only_entrypoint() -> None:
    """依赖链必须有独立 CLI，不能把锁验证隐藏在后续 builder 内。"""
    assert DEPENDENCY_VERIFIER.is_file(), (
        "stage 4 needs scripts/verify_stage4_dependencies.py before lock verification"
    )

    result = subprocess.run(
        [sys.executable, str(DEPENDENCY_VERIFIER), "--locks-only", "--help"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


def test_source_cache_verifier_exposes_a_stable_cli() -> None:
    """canonical source cache 必须有独立 verifier，不能仅由 builder 隐式检查。"""
    assert SOURCE_CACHE_VERIFIER.is_file(), "stage 4 source cache verifier is not implemented"
    result = subprocess.run(
        [sys.executable, str(SOURCE_CACHE_VERIFIER), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_source_cache_freezer_exposes_a_stable_cli() -> None:
    """source cache producer 也必须有显式、可审查的本地输入入口。"""
    assert SOURCE_CACHE_FREEZER.is_file(), "stage 4 source cache freezer is not implemented"
    result = subprocess.run(
        [sys.executable, str(SOURCE_CACHE_FREEZER), "--help"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_dependency_builder_rejects_overlapping_output_roots_before_materializing(tmp_path) -> None:
    """离线 builder 必须先拒绝交叠的私有输出根，且不得创建任何输出。"""
    assert DEPENDENCY_BUILDER.is_file(), "offline C++ dependency builder is not implemented"
    source_work = tmp_path / "source-work"
    build_root = source_work / "build"
    install_prefix = tmp_path / "install"

    result = subprocess.run(
        [
            str(DEPENDENCY_BUILDER),
            "--source-archive-cache",
            str(tmp_path / "canonical-cache"),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "output roots must be distinct and non-overlapping" in result.stderr
    assert not source_work.exists()
    assert not install_prefix.exists()


def test_dependency_builder_rejects_missing_canonical_cache_before_creating_outputs(tmp_path) -> None:
    """cache/lock/manifest 复核失败时，builder 不能创建 source、build 或 install 输出。"""
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"

    result = subprocess.run(
        [
            str(DEPENDENCY_BUILDER),
            "--source-archive-cache",
            str(tmp_path / "missing-cache"),
            "--source-archive-manifest",
            str(tmp_path / "missing-manifest.json"),
            "--dependency-lock",
            str(tmp_path / "missing-dependency.lock"),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "canonical source archive cache verification failed" in result.stderr
    assert not source_work.exists()
    assert not build_root.exists()
    assert not install_prefix.exists()


def test_dependency_builder_rejects_symlinked_cmake_before_creating_outputs(
    tmp_path,
) -> None:
    """C++ builder 必须拒绝链接工具，不能经 PATH 或链接隐式替换构建器。"""
    cmake_target = tmp_path / "real-cmake"
    cmake_target.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    cmake_target.chmod(0o755)
    cmake_link = tmp_path / "cmake-link"
    cmake_link.symlink_to(cmake_target)
    cache, manifest, lock = _freeze_fixture_cpp_source(tmp_path)
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    network_evidence = tmp_path / "network-evidence"

    result = subprocess.run(
        [
            str(NETWORK_WRAPPER),
            "--evidence-dir",
            str(network_evidence),
            "--",
            str(DEPENDENCY_BUILDER),
            "--network-evidence",
            str(network_evidence),
            "--cmake",
            str(cmake_link),
            "--cc",
            str(tmp_path / "gcc"),
            "--cxx",
            str(tmp_path / "gxx"),
            "--source-archive-cache",
            str(cache),
            "--source-archive-manifest",
            str(manifest),
            "--dependency-lock",
            str(lock),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "cmake must be an executable regular file" in result.stderr
    assert not source_work.exists()
    assert not build_root.exists()
    assert not install_prefix.exists()


def test_dependency_builder_accepts_fixed_stage4_toolchain_before_cmake_gate(
    tmp_path,
) -> None:
    """合规的固定工具应通过身份检查，并继续进入已物化源码的 CMake 门。"""
    tools = {}
    for name, output in (
        ("cmake", "cmake version 3.28.3"),
        ("cc", "x86_64-linux-gnu-gcc-13 (Ubuntu 13.3.0) 13.3.0"),
        ("cxx", "x86_64-linux-gnu-g++-13 (Ubuntu 13.3.0) 13.3.0"),
    ):
        path = tmp_path / name
        path.write_text(
            f"#!/usr/bin/env bash\nprintf '%s\\n' {output!r}\n",
            encoding="utf-8",
        )
        path.chmod(0o755)
        tools[name] = path
    cache, manifest, lock = _freeze_fixture_cpp_source(tmp_path)
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    network_evidence = tmp_path / "network-evidence"

    result = subprocess.run(
        [
            str(NETWORK_WRAPPER),
            "--evidence-dir",
            str(network_evidence),
            "--",
            str(DEPENDENCY_BUILDER),
            "--network-evidence",
            str(network_evidence),
            "--cmake",
            str(tools["cmake"]),
            "--cc",
            str(tools["cc"]),
            "--cxx",
            str(tools["cxx"]),
            "--source-archive-cache",
            str(cache),
            "--source-archive-manifest",
            str(manifest),
            "--dependency-lock",
            str(lock),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "offline CMake entry is not implemented for fixture" in result.stderr
    assert (source_work / "trees" / "fixture" / "fixture-root").is_dir()
    assert not build_root.exists()
    assert not install_prefix.exists()


def test_dependency_builder_builds_local_cmake_archive_with_reproducible_flags(
    tmp_path,
) -> None:
    """离线 builder 必须在私有根完成本地 CMake 源码的配置、构建和安装。"""
    cache, manifest, lock = _freeze_fixture_cpp_source(
        tmp_path,
        {
            "CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(stage4_fixture LANGUAGES C)
file(WRITE \"${CMAKE_BINARY_DIR}/source-date-epoch.txt\" \"$ENV{SOURCE_DATE_EPOCH}\")
add_library(stage4_fixture STATIC fixture.c)
install(TARGETS stage4_fixture ARCHIVE DESTINATION lib)
""",
            "fixture.c": b"int stage4_fixture(void) { return 0; }\n",
        },
    )
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    network_evidence = tmp_path / "network-evidence"

    result = subprocess.run(
        [
            str(NETWORK_WRAPPER),
            "--evidence-dir",
            str(network_evidence),
            "--",
            str(DEPENDENCY_BUILDER),
            "--network-evidence",
            str(network_evidence),
            "--cmake",
            "/usr/bin/cmake",
            "--cc",
            "/usr/bin/x86_64-linux-gnu-gcc-13",
            "--cxx",
            "/usr/bin/x86_64-linux-gnu-g++-13",
            "--source-archive-cache",
            str(cache),
            "--source-archive-manifest",
            str(manifest),
            "--dependency-lock",
            str(lock),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--source-date-epoch",
            "1700000000",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "PASS: 1 offline C++ dependencies built" in result.stdout
    assert (install_prefix / "lib" / "libstage4_fixture.a").is_file()
    fixture_build = build_root / "fixture"
    assert (fixture_build / "source-date-epoch.txt").read_text(encoding="utf-8") == "1700000000"
    cache_text = (fixture_build / "CMakeCache.txt").read_text(encoding="utf-8")
    assert "CMAKE_INSTALL_PREFIX:PATH=/stage4/dependencies" in cache_text
    commands = json.loads((fixture_build / "compile_commands.json").read_text(encoding="utf-8"))
    command = commands[0]["command"]
    assert f"-ffile-prefix-map={source_work}=/stage4/source" in command
    assert f"-fdebug-prefix-map={build_root}=/stage4/build" in command
    assert f"-fmacro-prefix-map={source_work}=/stage4/source" in command
    assert f"-ffile-prefix-map={install_prefix}=/stage4/dependencies" in command


def test_dependency_builder_install_elf_has_no_absolute_build_rpath(tmp_path) -> None:
    """安装 ELF 只能使用可搬迁路径，不能保留本轮临时构建目录。"""
    prebuilt_dir = tmp_path / "external-prebuilt"
    prebuilt_dir.mkdir()
    support_source = prebuilt_dir / "support.c"
    support_source.write_text(
        "int stage4_support(void) { return 0; }\n",
        encoding="utf-8",
    )
    support_library = prebuilt_dir / "libstage4_support.so"
    support_build = subprocess.run(
        [
            "/usr/bin/x86_64-linux-gnu-gcc-13",
            "-shared",
            "-fPIC",
            "-Wl,-soname,libstage4_support.so",
            str(support_source),
            "-o",
            str(support_library),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert support_build.returncode == 0, support_build.stderr
    cache, manifest, lock = _freeze_fixture_cpp_source(
        tmp_path,
        {
            "CMakeLists.txt": f"""cmake_minimum_required(VERSION 3.28)
project(stage4_rpath_fixture LANGUAGES C)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_library(stage4_support SHARED IMPORTED GLOBAL)
set_target_properties(stage4_support PROPERTIES
  IMPORTED_LOCATION "{support_library}")
add_library(stage4_fixture SHARED fixture.c)
target_link_libraries(stage4_fixture PRIVATE stage4_support)
install(TARGETS stage4_fixture LIBRARY DESTINATION lib)
""".encode(),
            "fixture.c": (
                b"int stage4_support(void); "
                b"int stage4_fixture(void) { return stage4_support(); }\n"
            ),
        },
    )
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    network_evidence = tmp_path / "network-evidence"

    result = subprocess.run(
        [
            str(NETWORK_WRAPPER),
            "--evidence-dir",
            str(network_evidence),
            "--",
            str(DEPENDENCY_BUILDER),
            "--network-evidence",
            str(network_evidence),
            "--cmake",
            "/usr/bin/cmake",
            "--cc",
            "/usr/bin/x86_64-linux-gnu-gcc-13",
            "--cxx",
            "/usr/bin/x86_64-linux-gnu-g++-13",
            "--source-archive-cache",
            str(cache),
            "--source-archive-manifest",
            str(manifest),
            "--dependency-lock",
            str(lock),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--source-date-epoch",
            "1700000000",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    installed_library = install_prefix / "lib" / "libstage4_fixture.so"
    readelf = subprocess.run(
        ["readelf", "-d", str(installed_library)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert readelf.returncode == 0, readelf.stderr
    assert str(tmp_path) not in readelf.stdout


def test_dependency_builder_compiler_wrapper_survives_cmake_arg_override(tmp_path) -> None:
    """项目覆盖 CMake compiler arg 后，安装 ELF 仍不能带临时 RPATH。"""
    toolchain_root = tmp_path / "injected-toolchain"
    compiler_dir = toolchain_root / "bin"
    compiler_dir.mkdir(parents=True)
    injected_rpath = toolchain_root / "lib"
    injected_rpath.mkdir(parents=True)
    real_cc = Path("/usr/bin/x86_64-linux-gnu-gcc-13")
    real_cxx = Path("/usr/bin/x86_64-linux-gnu-g++-13")
    assert real_cc.is_file() and real_cxx.is_file()
    default_specs = subprocess.run(
        [str(real_cc), "-dumpspecs"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert default_specs.returncode == 0, default_specs.stderr
    injected_rule = f"%{{!static:-rpath {injected_rpath}}}"
    specs = compiler_dir / "injected.specs"
    specs.write_text(
        default_specs.stdout.replace("*link:\n", f"*link:\n{injected_rule} ", 1),
        encoding="utf-8",
    )

    def write_compiler(name: str, actual: Path) -> Path:
        """构造 Conda specs 行为的最小编译器替身。"""
        compiler = compiler_dir / name
        compiler.write_text(
            f"""#!/usr/bin/env bash
set -euo pipefail
if [[ \"${{1:-}}\" == --version ]]; then
  printf '%s\\n' 'gcc (fixture) 13.4.0'
  exit 0
fi
if [[ \"${{1:-}}\" == -print-file-name=specs ]]; then
  printf '%s\\n' '{specs}'
  exit 0
fi
for argument in \"$@\"; do
  [[ \"$argument\" == -specs=* ]] && exec '{actual}' \"$@\"
done
exec '{actual}' '-specs={specs}' \"$@\"
""",
            encoding="utf-8",
        )
        compiler.chmod(0o755)
        return compiler

    cc = write_compiler("gcc", real_cc)
    cxx = write_compiler("g++", real_cxx)
    cache, manifest, lock = _freeze_fixture_cpp_source(
        tmp_path,
        {
            "CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(stage4_compiler_wrapper_fixture LANGUAGES C CXX)
set(CMAKE_C_COMPILER_ARG1 "" CACHE STRING "" FORCE)
set(CMAKE_CXX_COMPILER_ARG1 "" CACHE STRING "" FORCE)
set(CMAKE_SKIP_INSTALL_RPATH FALSE CACHE BOOL "" FORCE)
add_library(stage4_fixture SHARED fixture.c)
install(TARGETS stage4_fixture LIBRARY DESTINATION lib)
""",
            "fixture.c": b"int stage4_fixture(void) { return 0; }\n",
        },
    )
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    network_evidence = tmp_path / "network-evidence"
    result = subprocess.run(
        [
            str(NETWORK_WRAPPER), "--evidence-dir", str(network_evidence), "--",
            str(DEPENDENCY_BUILDER), "--network-evidence", str(network_evidence),
            "--cmake", "/usr/bin/cmake", "--cc", str(cc), "--cxx", str(cxx),
            "--source-archive-cache", str(cache), "--source-archive-manifest", str(manifest),
            "--dependency-lock", str(lock), "--source-work", str(source_work),
            "--build-root", str(build_root), "--install-prefix", str(install_prefix),
            "--source-date-epoch", "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    cache_text = (build_root / "fixture" / "CMakeCache.txt").read_text(encoding="utf-8")
    assert f"CMAKE_C_COMPILER:STRING={build_root}/toolchain-compilers/cc" in cache_text
    assert f"CMAKE_CXX_COMPILER:STRING={build_root}/toolchain-compilers/cxx" in cache_text
    installed_library = install_prefix / "lib" / "libstage4_fixture.so"
    readelf = subprocess.run(
        ["readelf", "-d", str(installed_library)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert readelf.returncode == 0, readelf.stderr
    assert str(injected_rpath) not in readelf.stdout


def test_dependency_builder_uses_zstd_build_cmake_entrypoint(tmp_path) -> None:
    """Zstd 的上游 CMake 入口固定在 build/cmake，不能假定归档根可配置。"""
    cache, manifest, lock = _freeze_fixture_cpp_source(
        tmp_path,
        {
            "build/cmake/CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(zstd_fixture LANGUAGES C)
add_library(zstd_fixture STATIC \"${CMAKE_CURRENT_LIST_DIR}/../../fixture.c\")
install(TARGETS zstd_fixture ARCHIVE DESTINATION lib)
""",
            "fixture.c": b"int zstd_fixture(void) { return 0; }\n",
        },
        dependency_name="zstd",
    )
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    network_evidence = tmp_path / "network-evidence"

    result = subprocess.run(
        [
            str(NETWORK_WRAPPER),
            "--evidence-dir",
            str(network_evidence),
            "--",
            str(DEPENDENCY_BUILDER),
            "--network-evidence",
            str(network_evidence),
            "--cmake",
            "/usr/bin/cmake",
            "--cc",
            "/usr/bin/x86_64-linux-gnu-gcc-13",
            "--cxx",
            "/usr/bin/x86_64-linux-gnu-g++-13",
            "--source-archive-cache",
            str(cache),
            "--source-archive-manifest",
            str(manifest),
            "--dependency-lock",
            str(lock),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert (install_prefix / "lib" / "libzstd_fixture.a").is_file()
    assert (build_root / "zstd" / "CMakeCache.txt").is_file()


def test_dependency_builder_installs_pcl_validator_from_the_validation_consumer(
    tmp_path,
) -> None:
    """PCL 必须完整构建并仅安装到独立 validation prefix。"""
    cache, manifest, lock = _freeze_cpp_source_set(
        tmp_path,
        {
            "fixture": {
                "CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(stage4_fixture LANGUAGES C)
add_library(stage4_fixture STATIC fixture.c)
install(TARGETS stage4_fixture ARCHIVE DESTINATION lib)
""",
                "fixture.c": b"int stage4_fixture(void) { return 0; }\n",
            },
            "pcl": {
                "CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(PCLValidator LANGUAGES CXX)
# Mimic PCL overriding caller-provided install RPATH during configuration.
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_library(pcl_fixture_support SHARED pcl_fixture_support.cpp)
set_target_properties(pcl_fixture_support PROPERTIES VERSION 1.14.0 SOVERSION 1.14)
add_executable(pcl_pcd2ply pcl_pcd2ply.cpp)
target_link_libraries(pcl_pcd2ply PRIVATE pcl_fixture_support)
add_custom_target(pcl_full_build_marker ALL
  COMMAND "${CMAKE_COMMAND}" -E touch "${CMAKE_BINARY_DIR}/pcl-full-build-marker")
install(FILES "${CMAKE_BINARY_DIR}/pcl-full-build-marker" DESTINATION share)
install(TARGETS pcl_fixture_support LIBRARY DESTINATION lib)
install(TARGETS pcl_pcd2ply RUNTIME DESTINATION bin)
""",
                "pcl_pcd2ply.cpp": b"""#include <iostream>
#include <string>
int pcl_fixture_support();
int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Syntax is: pcl_pcd2ply input.pcd output.ply\\n";
    return 255;
  }
  std::cout << "PCL fixture validator\\n";
  return pcl_fixture_support() == 7 ? 0 : 1;
}
""",
                "pcl_fixture_support.cpp": b"int pcl_fixture_support() { return 7; }\n",
            },
        },
        consumers={"pcl": ["validation"]},
    )
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    validation_prefix = tmp_path / "validation-prefix"
    network_evidence = tmp_path / "network-evidence"

    result = subprocess.run(
        [
            str(NETWORK_WRAPPER),
            "--evidence-dir",
            str(network_evidence),
            "--",
            str(DEPENDENCY_BUILDER),
            "--network-evidence",
            str(network_evidence),
            "--cmake",
            "/usr/bin/cmake",
            "--cc",
            "/usr/bin/x86_64-linux-gnu-gcc-13",
            "--cxx",
            "/usr/bin/x86_64-linux-gnu-g++-13",
            "--source-archive-cache",
            str(cache),
            "--source-archive-manifest",
            str(manifest),
            "--dependency-lock",
            str(lock),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--validation-prefix",
            str(validation_prefix),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    validator = validation_prefix / "bin" / "pcl_pcd2ply"
    assert validator.is_file()
    assert subprocess.run([str(validator)], check=False).returncode == 0
    readelf = subprocess.run(
        ["readelf", "-d", str(validator)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert readelf.returncode == 0, readelf.stderr
    assert "Library runpath: [$ORIGIN/../lib]" in readelf.stdout
    assert str(validation_prefix) not in readelf.stdout
    support_library = validation_prefix / "lib" / "libpcl_fixture_support.so.1.14.0"
    support_readelf = subprocess.run(
        ["readelf", "-d", str(support_library)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert support_readelf.returncode == 0, support_readelf.stderr
    assert "Library runpath: [$ORIGIN]" in support_readelf.stdout
    assert str(validation_prefix) not in support_readelf.stdout
    assert (build_root / "pcl-validation-runtime.txt").read_text(encoding="utf-8") == (
        "Syntax is: pcl_pcd2ply input.pcd output.ply\n"
    )
    ldd_evidence = (build_root / "pcl-validation-ldd.txt").read_text(encoding="utf-8")
    assert "not found" not in ldd_evidence
    assert (build_root / "pcl-validation" / "pcl-full-build-marker").is_file()
    assert not (install_prefix / "bin" / "pcl_pcd2ply").exists()


def test_dependency_builder_materialize_only_includes_validation_sources(tmp_path) -> None:
    """离线预检也必须私有物化 PCL validation 源码，不能只展开主依赖。"""
    cache, manifest, lock = _freeze_cpp_source_set(
        tmp_path,
        {
            "fixture": {"value.txt": b"cpp"},
            "pcl": {"value.txt": b"validation"},
        },
        consumers={"pcl": ["validation"]},
    )
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    validation_prefix = tmp_path / "validation-prefix"
    network_evidence = tmp_path / "network-evidence"

    result = subprocess.run(
        [
            str(NETWORK_WRAPPER),
            "--evidence-dir",
            str(network_evidence),
            "--",
            str(DEPENDENCY_BUILDER),
            "--network-evidence",
            str(network_evidence),
            "--materialize-only",
            "--source-archive-cache",
            str(cache),
            "--source-archive-manifest",
            str(manifest),
            "--dependency-lock",
            str(lock),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--validation-prefix",
            str(validation_prefix),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert (source_work / "trees" / "fixture" / "fixture-root" / "value.txt").read_bytes() == b"cpp"
    assert (
        source_work / "validation" / "trees" / "pcl" / "pcl-root" / "value.txt"
    ).read_bytes() == b"validation"
    assert not build_root.exists()
    assert not install_prefix.exists()
    assert not validation_prefix.exists()


def test_ubuntu24_container_builder_stages_inputs_before_network_isolation(tmp_path) -> None:
    """Docker 只读挂载输入后，必须从容器私有 staging 路径进入断网 wrapper。"""
    assert UBUNTU24_CONTAINER_BUILDER.is_file(), (
        "stage 4 needs an Ubuntu 24.04 private staging container builder"
    )
    docker = tmp_path / "docker"
    docker.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    docker.chmod(0o755)
    source_cache = tmp_path / "source-cache"
    source_cache.mkdir()
    output_parent = tmp_path / "outputs"
    output_parent.mkdir()
    output_root = output_parent / "fresh-run"

    result = subprocess.run(
        [
            sys.executable,
            str(UBUNTU24_CONTAINER_BUILDER),
            "--print-docker-command",
            "--docker",
            str(docker),
            "--system-lock",
            str(ROOT / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"),
            "--source-archive-cache",
            str(source_cache),
            "--source-archive-manifest",
            str(ROOT / "packaging" / "locks" / "source-archive-cache.manifest.json"),
            "--dependency-lock",
            str(CPP_DEPENDENCY_LOCK),
            "--source-cache-lock",
            str(ROS2_DEPENDENCY_LOCK),
            "--output-root",
            str(output_root),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    document = json.loads(result.stdout)
    command = document["command"]
    script = command[-1]
    assert any(
        f"src={ROOT / 'packaging'},dst=/stage4-input/packaging,readonly" in item
        for item in command
    )
    assert any(
        f"src={ROOT / 'scripts'},dst=/stage4-input/scripts,readonly" in item
        for item in command
    )
    assert any(
        f"src={source_cache},dst=/stage4-input/source-cache,readonly" in item
        for item in command
    )
    assert any(
        f"src={output_parent},dst=/stage4-output-parent" in item for item in command
    )
    assert "cp -a -- /stage4-input/packaging /opt/stage4-private/packaging" in script
    assert "cp -a -- /stage4-input/scripts /opt/stage4-private/scripts" in script
    assert "cp -a -- /stage4-input/source-cache /opt/stage4-private/source-cache" in script
    assert "chown -R root:root -- /opt/stage4-private" in script
    assert "/stage4-input/packaging/run_network_isolated.sh" not in script
    assert "exec /opt/stage4-private/packaging/run_network_isolated.sh" not in script
    assert "/opt/stage4-private/packaging/run_network_isolated.sh" in script
    assert "cmake=3.28.3-1build7" in script


def test_ubuntu24_container_builder_makes_completed_results_host_traversable(
    tmp_path,
) -> None:
    """容器以 root 构建后，宿主仍必须能读取本轮的验证产物。"""
    docker = tmp_path / "docker"
    docker.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    docker.chmod(0o755)
    source_cache = tmp_path / "source-cache"
    source_cache.mkdir()
    output_parent = tmp_path / "outputs"
    output_parent.mkdir()

    result = subprocess.run(
        [
            sys.executable,
            str(UBUNTU24_CONTAINER_BUILDER),
            "--print-docker-command",
            "--docker",
            str(docker),
            "--system-lock",
            str(ROOT / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"),
            "--source-archive-cache",
            str(source_cache),
            "--source-archive-manifest",
            str(ROOT / "packaging" / "locks" / "source-archive-cache.manifest.json"),
            "--dependency-lock",
            str(CPP_DEPENDENCY_LOCK),
            "--output-root",
            str(output_parent / "fresh-run"),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    script = json.loads(result.stdout)["command"][-1]
    assert "mkdir -m 0755 -- /stage4-output-parent/fresh-run" in script


def test_ubuntu24_container_builder_makes_completed_result_tree_host_auditable(
    tmp_path,
) -> None:
    """断网 child 成功返回后，容器入口必须开放本轮全部证据给宿主复核。"""
    docker = tmp_path / "docker"
    docker.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    docker.chmod(0o755)
    source_cache = tmp_path / "source-cache"
    source_cache.mkdir()
    output_parent = tmp_path / "outputs"
    output_parent.mkdir()

    result = subprocess.run(
        [
            sys.executable,
            str(UBUNTU24_CONTAINER_BUILDER),
            "--print-docker-command",
            "--docker",
            str(docker),
            "--system-lock",
            str(ROOT / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"),
            "--source-archive-cache",
            str(source_cache),
            "--source-archive-manifest",
            str(ROOT / "packaging" / "locks" / "source-archive-cache.manifest.json"),
            "--dependency-lock",
            str(CPP_DEPENDENCY_LOCK),
            "--output-root",
            str(output_parent / "fresh-run"),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    script = json.loads(result.stdout)["command"][-1]
    wrapper = "/opt/stage4-private/packaging/run_network_isolated.sh"
    assert f"exec {wrapper}" not in script
    assert wrapper in script
    assert "container-build.log" in script
    assert "container-exit-code.txt" in script
    assert "chmod -R a+rX -- /stage4-output-parent/fresh-run" in script
    assert script.index("trap ") < script.index(wrapper)


def test_ubuntu24_container_builder_can_detach_long_running_builds(tmp_path) -> None:
    """长构建必须交给 Docker daemon，并在结果根保留日志与退出状态。"""
    docker = tmp_path / "docker"
    docker.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    docker.chmod(0o755)
    source_cache = tmp_path / "source-cache"
    source_cache.mkdir()
    output_parent = tmp_path / "outputs"
    output_parent.mkdir()

    result = subprocess.run(
        [
            sys.executable,
            str(UBUNTU24_CONTAINER_BUILDER),
            "--print-docker-command",
            "--detach",
            "--docker",
            str(docker),
            "--system-lock",
            str(ROOT / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"),
            "--source-archive-cache",
            str(source_cache),
            "--source-archive-manifest",
            str(ROOT / "packaging" / "locks" / "source-archive-cache.manifest.json"),
            "--dependency-lock",
            str(CPP_DEPENDENCY_LOCK),
            "--output-root",
            str(output_parent / "fresh-run"),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    command = json.loads(result.stdout)["command"]
    script = command[-1]
    assert "--detach" in command
    assert "--rm" in command
    assert "container-build.log" in script
    assert "container-exit-code.txt" in script


def test_ubuntu24_container_builder_reports_detached_container_id(tmp_path) -> None:
    """启动 detached 容器后，调用方必须得到可轮询的精确 Docker ID。"""
    container_id = "0" * 64
    docker = tmp_path / "docker"
    docker.write_text(f"#!/bin/sh\nprintf '%s\\n' {container_id}\n", encoding="utf-8")
    docker.chmod(0o755)
    source_cache = tmp_path / "source-cache"
    source_cache.mkdir()
    output_parent = tmp_path / "outputs"
    output_parent.mkdir()
    output_root = output_parent / "fresh-run"

    result = subprocess.run(
        [
            sys.executable,
            str(UBUNTU24_CONTAINER_BUILDER),
            "--detach",
            "--docker",
            str(docker),
            "--system-lock",
            str(ROOT / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"),
            "--source-archive-cache",
            str(source_cache),
            "--source-archive-manifest",
            str(ROOT / "packaging" / "locks" / "source-archive-cache.manifest.json"),
            "--dependency-lock",
            str(CPP_DEPENDENCY_LOCK),
            "--output-root",
            str(output_root),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout) == {
        "container_id": container_id,
        "output_root": str(output_root),
    }


def test_ubuntu24_container_builder_rejects_existing_output_root(tmp_path) -> None:
    """容器入口不得复用已有结果根，避免覆盖上一轮构建和断网证据。"""
    docker = tmp_path / "docker"
    docker.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    docker.chmod(0o755)
    source_cache = tmp_path / "source-cache"
    source_cache.mkdir()
    output_parent = tmp_path / "outputs"
    output_parent.mkdir()
    output_root = output_parent / "previous-run"
    output_root.mkdir()

    result = subprocess.run(
        [
            sys.executable,
            str(UBUNTU24_CONTAINER_BUILDER),
            "--print-docker-command",
            "--docker",
            str(docker),
            "--system-lock",
            str(ROOT / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"),
            "--source-archive-cache",
            str(source_cache),
            "--source-archive-manifest",
            str(ROOT / "packaging" / "locks" / "source-archive-cache.manifest.json"),
            "--dependency-lock",
            str(CPP_DEPENDENCY_LOCK),
            "--output-root",
            str(output_root),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    assert "FAIL: output-root must be absent" in result.stdout


def test_ubuntu24_container_builder_allows_the_inner_user_namespace(tmp_path) -> None:
    """Docker 默认 seccomp 会拒绝 unshare，入口必须只放开该内层隔离前置条件。"""
    docker = tmp_path / "docker"
    docker.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    docker.chmod(0o755)
    source_cache = tmp_path / "source-cache"
    source_cache.mkdir()
    output_parent = tmp_path / "outputs"
    output_parent.mkdir()

    result = subprocess.run(
        [
            sys.executable,
            str(UBUNTU24_CONTAINER_BUILDER),
            "--print-docker-command",
            "--docker",
            str(docker),
            "--system-lock",
            str(ROOT / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"),
            "--source-archive-cache",
            str(source_cache),
            "--source-archive-manifest",
            str(ROOT / "packaging" / "locks" / "source-archive-cache.manifest.json"),
            "--dependency-lock",
            str(CPP_DEPENDENCY_LOCK),
            "--output-root",
            str(output_parent / "fresh-run"),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    command = json.loads(result.stdout)["command"]
    assert "--security-opt" in command
    assert "seccomp=unconfined" in command
    assert "--privileged" not in command


def test_ubuntu24_container_builder_uses_real_target_compiler_binaries(tmp_path) -> None:
    """隔离 builder 拒绝工具链接，容器入口必须传入 Ubuntu 的真实 GCC 13 文件。"""
    docker = tmp_path / "docker"
    docker.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    docker.chmod(0o755)
    source_cache = tmp_path / "source-cache"
    source_cache.mkdir()
    output_parent = tmp_path / "outputs"
    output_parent.mkdir()

    result = subprocess.run(
        [
            sys.executable,
            str(UBUNTU24_CONTAINER_BUILDER),
            "--print-docker-command",
            "--docker",
            str(docker),
            "--system-lock",
            str(ROOT / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"),
            "--source-archive-cache",
            str(source_cache),
            "--source-archive-manifest",
            str(ROOT / "packaging" / "locks" / "source-archive-cache.manifest.json"),
            "--dependency-lock",
            str(CPP_DEPENDENCY_LOCK),
            "--output-root",
            str(output_parent / "fresh-run"),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    script = json.loads(result.stdout)["command"][-1]
    assert "--cc /usr/bin/x86_64-linux-gnu-gcc-13" in script
    assert "--cxx /usr/bin/x86_64-linux-gnu-g++-13" in script
    assert "--cc /usr/bin/gcc-13" not in script


def test_dependency_builder_preserves_header_only_mcap_without_cmake_build(tmp_path) -> None:
    """MCAP C++ 是 header-only 源码，必须物化但不得伪造独立 CMake 构建。"""
    cache, manifest, lock = _freeze_cpp_source_set(
        tmp_path,
        {
            "mcap": {"cpp/mcap/include/mcap/mcap.hpp": b"// header-only\n"},
            "fixture": {
                "CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(fixture LANGUAGES C)
add_library(fixture STATIC fixture.c)
install(TARGETS fixture ARCHIVE DESTINATION lib)
""",
                "fixture.c": b"int fixture(void) { return 0; }\n",
            },
        },
    )
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    network_evidence = tmp_path / "network-evidence"

    result = subprocess.run(
        [
            str(NETWORK_WRAPPER), "--evidence-dir", str(network_evidence), "--",
            str(DEPENDENCY_BUILDER), "--network-evidence", str(network_evidence),
            "--cmake", "/usr/bin/cmake", "--cc", "/usr/bin/x86_64-linux-gnu-gcc-13",
            "--cxx", "/usr/bin/x86_64-linux-gnu-g++-13", "--source-archive-cache", str(cache),
            "--source-archive-manifest", str(manifest), "--dependency-lock", str(lock),
            "--source-work", str(source_work), "--build-root", str(build_root),
            "--install-prefix", str(install_prefix), "--source-date-epoch", "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert (source_work / "trees" / "mcap" / "mcap-root" / "cpp" / "mcap" / "include" / "mcap" / "mcap.hpp").is_file()
    assert not (build_root / "mcap").exists()
    assert (install_prefix / "lib" / "libfixture.a").is_file()


def test_dependency_builder_rejects_ecal_archive_without_submodule_sources(
    tmp_path,
) -> None:
    """eCAL release archive 的 submodule 缺失必须在 configure 前被明确拒绝。"""
    cache, manifest, lock = _freeze_fixture_cpp_source(
        tmp_path,
        {
            "CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(ecal_fixture LANGUAGES C)
add_library(ecal_fixture STATIC fixture.c)
install(TARGETS ecal_fixture ARCHIVE DESTINATION lib)
""",
            ".gitmodules": b"""[submodule \"thirdparty/asio\"]
\tpath = thirdparty/asio/asio
\turl = https://example.invalid/asio.git
""",
            "fixture.c": b"int ecal_fixture(void) { return 0; }\n",
        },
        dependency_name="ecal",
    )
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    network_evidence = tmp_path / "network-evidence"

    result = subprocess.run(
        [
            str(NETWORK_WRAPPER),
            "--evidence-dir",
            str(network_evidence),
            "--",
            str(DEPENDENCY_BUILDER),
            "--network-evidence",
            str(network_evidence),
            "--cmake",
            "/usr/bin/cmake",
            "--cc",
            "/usr/bin/x86_64-linux-gnu-gcc-13",
            "--cxx",
            "/usr/bin/x86_64-linux-gnu-g++-13",
            "--source-archive-cache",
            str(cache),
            "--source-archive-manifest",
            str(manifest),
            "--dependency-lock",
            str(lock),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "eCAL source closure is incomplete: thirdparty/asio/asio" in result.stderr
    assert (source_work / "trees" / "ecal" / "ecal-root").is_dir()
    assert not build_root.exists()
    assert not install_prefix.exists()


def test_dependency_builder_allows_disabled_ecal_submodules_to_be_absent(
    tmp_path,
) -> None:
    """固定 eCAL profile 允许与核心无关的禁用 submodule 保持缺失。"""
    cache, manifest, lock = _freeze_fixture_cpp_source(
        tmp_path,
        {
            "CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(ecal_fixture LANGUAGES C)
add_library(ecal_fixture STATIC fixture.c)
install(TARGETS ecal_fixture ARCHIVE DESTINATION lib)
""",
            "cmake/submodule_dependencies.cmake": b"# fixture provider\n",
            ".gitmodules": b"""[submodule \"thirdparty/asio\"]
\tpath = thirdparty/asio/asio
\turl = https://example.invalid/asio.git
[submodule \"thirdparty/ecaludp\"]
\tpath = thirdparty/ecaludp/ecaludp
\turl = https://example.invalid/ecaludp.git
[submodule \"thirdparty/protozero\"]
\tpath = thirdparty/protozero/protozero
\turl = https://example.invalid/protozero.git
[submodule \"thirdparty/recycle\"]
\tpath = thirdparty/recycle/recycle
\turl = https://example.invalid/recycle.git
[submodule \"thirdparty/tclap\"]
\tpath = thirdparty/tclap/tclap
\turl = https://example.invalid/tclap.git
[submodule \"thirdparty/tcp_pubsub\"]
\tpath = thirdparty/tcp_pubsub/tcp_pubsub
\turl = https://example.invalid/tcp_pubsub.git
[submodule \"thirdparty/yaml-cpp\"]
\tpath = thirdparty/yaml-cpp/yaml-cpp
\turl = https://example.invalid/yaml-cpp.git
[submodule \"thirdparty/hdf5\"]
\tpath = thirdparty/hdf5/hdf5
\turl = https://example.invalid/hdf5.git
""",
            "fixture.c": b"int ecal_fixture(void) { return 0; }\n",
            "thirdparty/asio/asio/marker.txt": b"asio",
            "thirdparty/ecaludp/ecaludp/marker.txt": b"ecaludp",
            "thirdparty/protozero/protozero/marker.txt": b"protozero",
            "thirdparty/recycle/recycle/marker.txt": b"recycle",
            "thirdparty/tclap/tclap/marker.txt": b"tclap",
            "thirdparty/tcp_pubsub/tcp_pubsub/marker.txt": b"tcp-pubsub",
            "thirdparty/yaml-cpp/yaml-cpp/marker.txt": b"yaml-cpp",
        },
        dependency_name="ecal",
    )
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    network_evidence = tmp_path / "network-evidence"

    result = subprocess.run(
        [
            str(NETWORK_WRAPPER),
            "--evidence-dir",
            str(network_evidence),
            "--",
            str(DEPENDENCY_BUILDER),
            "--network-evidence",
            str(network_evidence),
            "--cmake",
            "/usr/bin/cmake",
            "--cc",
            "/usr/bin/x86_64-linux-gnu-gcc-13",
            "--cxx",
            "/usr/bin/x86_64-linux-gnu-g++-13",
            "--source-archive-cache",
            str(cache),
            "--source-archive-manifest",
            str(manifest),
            "--dependency-lock",
            str(lock),
            "--source-work",
            str(source_work),
            "--build-root",
            str(build_root),
            "--install-prefix",
            str(install_prefix),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert (install_prefix / "lib" / "libecal_fixture.a").is_file()


def test_dependency_builder_hydrates_locked_ecal_submodules_and_configures_raw_core(
    tmp_path,
) -> None:
    """eCAL 必须从同轮私有 submodule tree 填充源码，不能 FetchContent 或单独构建它们。"""
    cache, manifest, lock = _freeze_cpp_source_set(
        tmp_path,
        {
            "ecal": {
                "CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(ecal_fixture LANGUAGES C)
if(NOT STAGE4_ECAL_PROVIDER_LOADED)
  message(FATAL_ERROR "eCAL submodule dependency provider was not loaded")
endif()
foreach(required_submodule
  "thirdparty/recycle/recycle/recycle.marker"
  "thirdparty/tclap/tclap/include/tclap/CmdLine.h"
  "thirdparty/tcp_pubsub/tcp_pubsub/tcp_pubsub.marker"
  "thirdparty/yaml-cpp/yaml-cpp/yaml-cpp.marker")
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${required_submodule}")
    message(FATAL_ERROR "eCAL locked submodule was not hydrated: ${required_submodule}")
  endif()
endforeach()
if(NOT DEFINED ECAL_USE_PROTOBUF OR ECAL_USE_PROTOBUF)
  message(FATAL_ERROR "eCAL C++ SDK must configure the raw core without Protobuf support")
endif()
if(NOT DEFINED ECAL_BUILD_C_BINDING OR ECAL_BUILD_C_BINDING)
  message(FATAL_ERROR "eCAL C binding must be disabled for the raw core SDK")
endif()
if(NOT DEFINED ECAL_BUILD_CSHARP_BINDING OR ECAL_BUILD_CSHARP_BINDING)
  message(FATAL_ERROR "eCAL C# binding must be disabled for the raw core SDK")
endif()
if(NOT DEFINED ECAL_BUILD_PY_BINDING OR ECAL_BUILD_PY_BINDING)
  message(FATAL_ERROR "eCAL Python binding must be disabled for the raw core SDK")
endif()
if(NOT CMAKE_POSITION_INDEPENDENT_CODE)
  message(FATAL_ERROR "offline C++ dependencies must build position-independent static objects")
endif()
add_library(ecal_fixture STATIC fixture.c)
install(TARGETS ecal_fixture ARCHIVE DESTINATION lib)
""",
                "cmake/submodule_dependencies.cmake": b"set(STAGE4_ECAL_PROVIDER_LOADED TRUE)\n",
                ".gitmodules": b"""[submodule \"thirdparty/asio\"]
\tpath = thirdparty/asio/asio
[submodule \"thirdparty/ecaludp\"]
\tpath = thirdparty/ecaludp/ecaludp
[submodule \"thirdparty/protozero\"]
\tpath = thirdparty/protozero/protozero
[submodule \"thirdparty/recycle\"]
\tpath = thirdparty/recycle/recycle
[submodule \"thirdparty/tclap\"]
\tpath = thirdparty/tclap/tclap
[submodule \"thirdparty/tcp_pubsub\"]
\tpath = thirdparty/tcp_pubsub/tcp_pubsub
[submodule \"thirdparty/yaml-cpp\"]
\tpath = thirdparty/yaml-cpp/yaml-cpp
""",
                "fixture.c": b"int ecal_fixture(void) { return 0; }\n",
                "thirdparty/cmakefunctions/cmake_functions/CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(CMakeFunctions VERSION 0.4.1)
configure_file(CMakeFunctionsConfig.cmake.in CMakeFunctionsConfig.cmake @ONLY)
install(FILES \"${CMAKE_CURRENT_BINARY_DIR}/CMakeFunctionsConfig.cmake\" DESTINATION lib/cmake/CMakeFunctions-0.4.1)
""",
                "thirdparty/cmakefunctions/cmake_functions/CMakeFunctionsConfig.cmake.in": b"set(CMakeFunctions_FOUND TRUE)\n",
            },
            "ecal-asio": {"include/asio.hpp": b"asio"},
            "ecal-ecaludp": {"include/ecaludp.hpp": b"ecaludp"},
            "ecal-protozero": {"include/protozero.hpp": b"protozero"},
            "ecal-recycle": {"recycle.marker": b"recycle"},
            "ecal-tclap": {"include/tclap/CmdLine.h": b"tclap"},
            "ecal-tcp-pubsub": {"tcp_pubsub.marker": b"tcp-pubsub"},
            "ecal-yaml-cpp": {"yaml-cpp.marker": b"yaml-cpp"},
            "protobuf": {
                "CMakeLists.txt": b"""cmake_minimum_required(VERSION 3.28)
project(Protobuf VERSION 33.6.0)
if(NOT DEFINED protobuf_BUILD_TESTS OR protobuf_BUILD_TESTS)
  message(FATAL_ERROR "protobuf tests must be disabled for the offline profile")
endif()
if(NOT protobuf_LOCAL_DEPENDENCIES_ONLY)
  message(FATAL_ERROR "protobuf must not fetch fallback dependencies")
endif()
configure_file(ProtobufConfig.cmake.in ProtobufConfig.cmake @ONLY)
install(FILES \"${CMAKE_CURRENT_BINARY_DIR}/ProtobufConfig.cmake\" DESTINATION lib/cmake/protobuf)
""",
                "ProtobufConfig.cmake.in": b"set(Protobuf_FOUND TRUE)\nset(STAGE4_FIXTURE_PROTOBUF TRUE)\n",
            },
        },
    )
    source_work = tmp_path / "source-work"
    build_root = tmp_path / "build-root"
    install_prefix = tmp_path / "install-prefix"
    network_evidence = tmp_path / "network-evidence"
    result = subprocess.run(
        [str(NETWORK_WRAPPER), "--evidence-dir", str(network_evidence), "--", str(DEPENDENCY_BUILDER), "--network-evidence", str(network_evidence), "--cmake", "/usr/bin/cmake", "--cc", "/usr/bin/x86_64-linux-gnu-gcc-13", "--cxx", "/usr/bin/x86_64-linux-gnu-g++-13", "--source-archive-cache", str(cache), "--source-archive-manifest", str(manifest), "--dependency-lock", str(lock), "--source-work", str(source_work), "--build-root", str(build_root), "--install-prefix", str(install_prefix), "--source-date-epoch", "1"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    ecal_root = source_work / "trees" / "ecal" / "ecal-root"
    assert (ecal_root / "thirdparty" / "asio" / "asio" / "include" / "asio.hpp").read_bytes() == b"asio"
    assert (ecal_root / "thirdparty" / "ecaludp" / "ecaludp" / "include" / "ecaludp.hpp").read_bytes() == b"ecaludp"
    assert (ecal_root / "thirdparty" / "protozero" / "protozero" / "include" / "protozero.hpp").read_bytes() == b"protozero"
    assert (ecal_root / "thirdparty" / "recycle" / "recycle" / "recycle.marker").read_bytes() == b"recycle"
    assert (ecal_root / "thirdparty" / "tclap" / "tclap" / "include" / "tclap" / "CmdLine.h").read_bytes() == b"tclap"
    assert (ecal_root / "thirdparty" / "tcp_pubsub" / "tcp_pubsub" / "tcp_pubsub.marker").read_bytes() == b"tcp-pubsub"
    assert (ecal_root / "thirdparty" / "yaml-cpp" / "yaml-cpp" / "yaml-cpp.marker").read_bytes() == b"yaml-cpp"
    assert (install_prefix / "lib" / "cmake" / "CMakeFunctions-0.4.1" / "CMakeFunctionsConfig.cmake").is_file()
    assert (build_root / "ecal-cmakefunctions" / "CMakeCache.txt").is_file()
    assert (build_root / "protobuf" / "CMakeCache.txt").is_file()
    assert not (build_root / "ecal-asio").exists()
    assert not (build_root / "ecal-ecaludp").exists()
    assert not (build_root / "ecal-protozero").exists()
    assert not (build_root / "ecal-recycle").exists()
    assert not (build_root / "ecal-tcp-pubsub").exists()
    assert not (build_root / "ecal-yaml-cpp").exists()


def test_dependency_source_materializer_copies_and_safely_materializes_cpp_archives(tmp_path) -> None:
    """cpp consumer 的 canonical archive 必须私有复制、复核并物化为零链接源码树。"""
    assert DEPENDENCY_SOURCE_MATERIALIZER.is_file(), "dependency source materializer is not implemented"
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    archive = source_dir / "fixture.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        member = tarfile.TarInfo("fixture-root/README.md")
        member.size = 7
        handle.addfile(member, io.BytesIO(b"payload"))
    archive_bytes = archive.read_bytes()
    sha256 = hashlib.sha256(archive_bytes).hexdigest()
    lock = tmp_path / "dependency.lock"
    lock.write_text(json.dumps({"schema_version": 1, "dependencies": [{
        "name": "fixture", "url": "https://example.invalid/fixture.tar.gz",
        "ref_kind": "commit", "ref": "a" * 40, "commit": "a" * 40,
        "consumers": ["cpp_dependency"],
        "archive": {"format": "tar.gz", "size": len(archive_bytes), "sha256": sha256},
    }]}), encoding="utf-8")
    cache = tmp_path / "canonical-cache"
    manifest = tmp_path / "manifest.json"
    frozen = subprocess.run([sys.executable, str(SOURCE_CACHE_FREEZER), "--lock", str(lock), "--source-dir", str(source_dir), "--cache-root", str(cache), "--manifest", str(manifest)], check=False, capture_output=True, text=True)
    assert frozen.returncode == 0, frozen.stderr
    source_work = tmp_path / "source-work"
    evidence = tmp_path / "materialization.json"

    result = subprocess.run([sys.executable, str(DEPENDENCY_SOURCE_MATERIALIZER), "--manifest", str(manifest), "--lock", str(lock), "--canonical-cache", str(cache), "--source-work", str(source_work), "--evidence", str(evidence)], check=False, capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    copied = source_work / "archives" / sha256 / "fixture.tar.gz"
    tree = source_work / "trees" / "fixture" / "fixture-root"
    assert copied.read_bytes() == archive_bytes
    assert (tree / "README.md").read_bytes() == b"payload"
    assert not (tree / "README.md").is_symlink()
    document = json.loads(evidence.read_text(encoding="utf-8"))
    assert document["archives"][0]["name"] == "fixture"
    assert document["archives"][0]["tree"] == str(tree)


def test_dependency_source_materializer_selects_validation_consumer(tmp_path) -> None:
    """PCL validation 源码必须按显式 consumer 独立物化。"""
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    archive = source_dir / "validation.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        member = tarfile.TarInfo("validation-root/value.txt")
        member.size = 10
        handle.addfile(member, io.BytesIO(b"validation"))
    payload = archive.read_bytes()
    sha256 = hashlib.sha256(payload).hexdigest()
    lock = tmp_path / "validation.lock"
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "validation",
                        "url": "https://example.invalid/validation.tar.gz",
                        "ref_kind": "commit",
                        "ref": "b" * 40,
                        "commit": "b" * 40,
                        "consumers": ["validation"],
                        "archive": {
                            "format": "tar.gz",
                            "size": len(payload),
                            "sha256": sha256,
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    cache = tmp_path / "cache"
    manifest = tmp_path / "manifest.json"
    frozen = subprocess.run(
        [
            sys.executable,
            str(SOURCE_CACHE_FREEZER),
            "--lock",
            str(lock),
            "--source-dir",
            str(source_dir),
            "--cache-root",
            str(cache),
            "--manifest",
            str(manifest),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert frozen.returncode == 0, frozen.stderr
    source_work = tmp_path / "validation-source-work"
    evidence = tmp_path / "validation-materialization.json"

    result = subprocess.run(
        [
            sys.executable,
            str(DEPENDENCY_SOURCE_MATERIALIZER),
            "--manifest",
            str(manifest),
            "--lock",
            str(lock),
            "--consumer",
            "validation",
            "--canonical-cache",
            str(cache),
            "--source-work",
            str(source_work),
            "--evidence",
            str(evidence),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert (
        source_work / "trees" / "validation" / "validation-root" / "value.txt"
    ).read_bytes() == b"validation"


def test_dependency_builder_materialize_only_runs_after_cache_verification(tmp_path) -> None:
    """builder 的物化模式必须消费已验 cache，且不得提前创建 build/install 输出。"""
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    archive = source_dir / "fixture.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        member = tarfile.TarInfo("fixture-root/value.txt")
        member.size = 5
        handle.addfile(member, io.BytesIO(b"value"))
    payload = archive.read_bytes()
    sha256 = hashlib.sha256(payload).hexdigest()
    lock = tmp_path / "dependency.lock"
    lock.write_text(json.dumps({"schema_version": 1, "dependencies": [{"name": "fixture", "url": "https://example.invalid/fixture.tar.gz", "ref_kind": "commit", "ref": "a" * 40, "commit": "a" * 40, "consumers": ["cpp_dependency"], "archive": {"format": "tar.gz", "size": len(payload), "sha256": sha256}}]}), encoding="utf-8")
    cache, manifest = tmp_path / "cache", tmp_path / "manifest.json"
    frozen = subprocess.run([sys.executable, str(SOURCE_CACHE_FREEZER), "--lock", str(lock), "--source-dir", str(source_dir), "--cache-root", str(cache), "--manifest", str(manifest)], check=False, capture_output=True, text=True)
    assert frozen.returncode == 0, frozen.stderr
    source_work, build_root, install_prefix = tmp_path / "source-work", tmp_path / "build", tmp_path / "install"

    network_evidence = tmp_path / "network-evidence"
    result = subprocess.run([str(NETWORK_WRAPPER), "--evidence-dir", str(network_evidence), "--", str(DEPENDENCY_BUILDER), "--network-evidence", str(network_evidence), "--materialize-only", "--source-archive-cache", str(cache), "--source-archive-manifest", str(manifest), "--dependency-lock", str(lock), "--source-work", str(source_work), "--build-root", str(build_root), "--install-prefix", str(install_prefix), "--source-date-epoch", "1"], check=False, capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    assert (source_work / "trees" / "fixture" / "fixture-root" / "value.txt").read_bytes() == b"value"
    assert (source_work / "materialization.json").is_file()
    assert not build_root.exists()
    assert not install_prefix.exists()


def test_dependency_builder_accepts_extra_lock_for_shared_canonical_cache(tmp_path) -> None:
    """共享 manifest 需同时复核 ROS lock，但 C++ builder 只能物化自己的 archive。"""
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    records = []
    for name, consumer in (("cpp", "cpp_dependency"), ("ros", "ros_overlay")):
        archive = source_dir / f"{name}.tar.gz"
        with tarfile.open(archive, "w:gz") as handle:
            member = tarfile.TarInfo(f"{name}-root/value.txt")
            member.size = len(name)
            handle.addfile(member, io.BytesIO(name.encode()))
        payload = archive.read_bytes()
        records.append((name, consumer, payload))
    locks = []
    for name, consumer, payload in records:
        lock = tmp_path / f"{name}.lock"
        lock.write_text(json.dumps({"schema_version": 1, "dependencies": [{"name": name, "url": f"https://example.invalid/{name}.tar.gz", "ref_kind": "commit", "ref": "a" * 40, "commit": "a" * 40, "consumers": [consumer], "archive": {"format": "tar.gz", "size": len(payload), "sha256": hashlib.sha256(payload).hexdigest()}}]}), encoding="utf-8")
        locks.append(lock)
    cache, manifest = tmp_path / "cache", tmp_path / "manifest.json"
    frozen = subprocess.run([sys.executable, str(SOURCE_CACHE_FREEZER), "--lock", str(locks[0]), "--lock", str(locks[1]), "--source-dir", str(source_dir), "--cache-root", str(cache), "--manifest", str(manifest)], check=False, capture_output=True, text=True)
    assert frozen.returncode == 0, frozen.stderr
    source_work = tmp_path / "source-work"
    network_evidence = tmp_path / "network-evidence"
    result = subprocess.run([str(NETWORK_WRAPPER), "--evidence-dir", str(network_evidence), "--", str(DEPENDENCY_BUILDER), "--network-evidence", str(network_evidence), "--materialize-only", "--source-archive-cache", str(cache), "--source-archive-manifest", str(manifest), "--dependency-lock", str(locks[0]), "--source-cache-lock", str(locks[1]), "--source-work", str(source_work), "--build-root", str(tmp_path / "build"), "--install-prefix", str(tmp_path / "install"), "--source-date-epoch", "1"], check=False, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert (source_work / "trees" / "cpp" / "cpp-root" / "value.txt").read_text() == "cpp"
    assert not (source_work / "trees" / "ros").exists()


def test_cpp_dependency_lock_includes_frozen_abseil_lts_source() -> None:
    """私有 Protobuf producer 必须从锁定的 Abseil LTS 源码构建静态 targets。"""
    document = json.loads(CPP_DEPENDENCY_LOCK.read_text(encoding="utf-8"))
    records = document.get("dependencies")
    assert isinstance(records, list)
    abseil = next((record for record in records if record.get("name") == "abseil-cpp"), None)
    assert abseil == {
        "name": "abseil-cpp",
        "url": "https://github.com/abseil/abseil-cpp/archive/76bb24329e8bf5f39704eb10d21b9a80befa7c81.tar.gz",
        "ref_kind": "commit",
        "ref": "76bb24329e8bf5f39704eb10d21b9a80befa7c81",
        "commit": "76bb24329e8bf5f39704eb10d21b9a80befa7c81",
        "license": "Apache-2.0",
        "license_files": ["LICENSE"],
        "consumers": ["cpp_dependency"],
        "archive": {
            "format": "tar.gz",
            "size": 2220566,
            "sha256": "ed8f7d9f39139c449e79fd19765e23c96fdb774172d32d191323d3e3ea06e5ff",
        },
    }


def test_source_cache_freezer_writes_verifiable_canonical_cache(tmp_path) -> None:
    """freezer 必须从本地归档生成可由独立 verifier 消费的 cache。"""
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    archive_name = "fixture.tar.gz"
    archive = source_dir / archive_name
    with tarfile.open(archive, "w:gz") as handle:
        member = tarfile.TarInfo("fixture-root/README.md")
        member.size = 7
        handle.addfile(member, io.BytesIO(b"payload"))
    archive_bytes = archive.read_bytes()
    archive_sha256 = hashlib.sha256(archive_bytes).hexdigest()
    lock = tmp_path / "cpp-dependencies.lock"
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "fixture",
                        "url": "https://github.com/example/fixture/archive/" + archive_name,
                        "ref_kind": "commit",
                        "ref": "a" * 40,
                        "commit": "a" * 40,
                        "consumers": ["cpp_dependency"],
                        "archive": {
                            "format": "tar.gz",
                            "size": len(archive_bytes),
                            "sha256": archive_sha256,
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    cache_root = tmp_path / "cache"
    manifest = tmp_path / "source-archive-cache.manifest.json"
    freeze = subprocess.run(
        [
            sys.executable,
            str(SOURCE_CACHE_FREEZER),
            "--lock",
            str(lock),
            "--source-dir",
            str(source_dir),
            "--cache-root",
            str(cache_root),
            "--manifest",
            str(manifest),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert freeze.returncode == 0, freeze.stderr
    assert "PASS: 1 canonical source archives frozen" in freeze.stdout
    document = json.loads(manifest.read_text(encoding="utf-8"))
    assert document["archives"][0]["top_level_root"] == "fixture-root"
    assert document["archives"][0]["member_count"] == 1
    assert document["archives"][0]["regular_bytes"] == 7
    assert document["archives"][0]["symlink_count"] == 0
    assert document["archives"][0]["materialized_member_count"] == 1
    assert document["archives"][0]["materialized_regular_bytes"] == 7
    assert len(document["archives"][0]["materialized_tree_sha256"]) == 64

    verify = subprocess.run(
        [
            sys.executable,
            str(SOURCE_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--lock",
            str(lock),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert verify.returncode == 0, verify.stderr

    document["archives"][0]["top_level_root"] = "tampered-root"
    manifest.write_text(json.dumps(document), encoding="utf-8")
    tampered = subprocess.run(
        [
            sys.executable,
            str(SOURCE_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--lock",
            str(lock),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert tampered.returncode != 0
    assert "top-level root differs from canonical archive" in tampered.stdout

    document["archives"][0]["top_level_root"] = "fixture-root"
    document["archives"][0]["member_count"] = 2
    manifest.write_text(json.dumps(document), encoding="utf-8")
    census_tampered = subprocess.run(
        [
            sys.executable,
            str(SOURCE_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--lock",
            str(lock),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert census_tampered.returncode != 0
    assert "member count differs from canonical archive" in census_tampered.stdout


def test_source_cache_verifier_accepts_lock_matched_canonical_archive(tmp_path) -> None:
    """canonical cache 只有与 lock/manifest 同时匹配时才能通过。"""
    archive_buffer = io.BytesIO()
    with tarfile.open(fileobj=archive_buffer, mode="w:gz") as handle:
        member = tarfile.TarInfo("fixture-root/README.md")
        member.size = 7
        handle.addfile(member, io.BytesIO(b"payload"))
    archive_bytes = archive_buffer.getvalue()
    archive_sha256 = hashlib.sha256(archive_bytes).hexdigest()
    archive_name = "fixture.tar.gz"
    lock = tmp_path / "cpp-dependencies.lock"
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "fixture",
                        "url": "https://github.com/example/fixture/archive/" + archive_name,
                        "ref_kind": "commit",
                        "ref": "a" * 40,
                        "commit": "a" * 40,
                        "consumers": ["cpp_dependency"],
                        "archive": {
                            "format": "tar.gz",
                            "size": len(archive_bytes),
                            "sha256": archive_sha256,
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    relative_path = f"archives/{archive_sha256}/{archive_name}"
    manifest = tmp_path / "source-archive-cache.manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "archives": [
                    {
                        "name": "fixture",
                        "url": "https://github.com/example/fixture/archive/" + archive_name,
                        "ref_kind": "commit",
                        "ref": "a" * 40,
                        "commit": "a" * 40,
                        "consumers": ["cpp_dependency"],
                        "archive": {
                            "format": "tar.gz",
                            "size": len(archive_bytes),
                            "sha256": archive_sha256,
                        },
                        "relative_path": relative_path,
                        "top_level_root": "fixture-root",
                        "member_count": 1,
                        "regular_bytes": 7,
                        "symlink_count": 0,
                        "materialized_member_count": 1,
                        "materialized_regular_bytes": 7,
                        "materialized_tree_sha256": hashlib.sha256(
                            b"F\0README.md\0"
                            + hashlib.sha256(b"payload").hexdigest().encode()
                            + b"\n"
                        ).hexdigest(),
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    cache_root = tmp_path / "cache"
    artifact = cache_root / relative_path
    artifact.parent.mkdir(parents=True)
    artifact.write_bytes(archive_bytes)

    result = subprocess.run(
        [
            sys.executable,
            str(SOURCE_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--lock",
            str(lock),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "PASS: 1 canonical source archives verified" in result.stdout

    document = json.loads(manifest.read_text(encoding="utf-8"))
    document["archives"][0]["materialized_member_count"] = 2
    manifest.write_text(json.dumps(document), encoding="utf-8")
    materialized_tampered = subprocess.run(
        [
            sys.executable,
            str(SOURCE_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--lock",
            str(lock),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert materialized_tampered.returncode != 0
    assert "materialized member count differs from canonical archive" in materialized_tampered.stdout

    document["archives"][0]["materialized_member_count"] = 1
    manifest.write_text(json.dumps(document), encoding="utf-8")
    (cache_root / "unexpected.tar.gz").write_bytes(b"not in the lock")
    surplus = subprocess.run(
        [
            sys.executable,
            str(SOURCE_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--lock",
            str(lock),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert surplus.returncode != 0
    assert "contains an unexpected entry" in surplus.stdout


def test_dependency_lock_rejects_commit_kind_with_non_commit_ref(tmp_path) -> None:
    """commit lock 必须把 ref 固定为同一个完整 SHA，不能混入浮动 tag。"""
    lock = tmp_path / "cpp-dependencies.lock"
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "fixture",
                        "url": "https://github.com/example/fixture/archive/v1.0.0.tar.gz",
                        "ref_kind": "commit",
                        "ref": "v1.0.0",
                        "commit": "a" * 40,
                        "consumers": ["cpp_dependency"],
                        "archive": {
                            "format": "tar.gz",
                            "size": 1,
                            "sha256": "b" * 64,
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    verifier = _load_verifier()
    load_lock = getattr(verifier, "load_dependency_lock", None)
    assert callable(load_lock), "dependency verifier needs a structured lock parser"

    with pytest.raises(ValueError, match="commit ref must equal commit"):
        load_lock(lock)


def test_locks_only_cli_accepts_a_valid_structured_lock(tmp_path) -> None:
    """公开 CLI 必须实际消费 lock，而不是只提供无行为的帮助选项。"""
    lock = tmp_path / "cpp-dependencies.lock"
    commit = "a" * 40
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "fixture",
                        "url": "https://github.com/example/fixture/archive/" + commit + ".tar.gz",
                        "ref_kind": "commit",
                        "ref": commit,
                        "commit": commit,
                        "consumers": ["cpp_dependency"],
                        "archive": {
                            "format": "tar.gz",
                            "size": 1,
                            "sha256": "b" * 64,
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(DEPENDENCY_VERIFIER),
            "--locks-only",
            "--lock",
            str(lock),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "PASS: 1 dependency lock entries verified" in result.stdout


def test_locks_only_cli_rejects_missing_lock_input() -> None:
    """仅锁校验没有任何显式输入时必须失败，不能伪造成功。"""
    result = subprocess.run(
        [sys.executable, str(DEPENDENCY_VERIFIER), "--locks-only"],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "requires at least one --lock" in result.stdout


def test_system_dependency_lock_rejects_unpinned_builder_image_digest(tmp_path) -> None:
    """系统锁必须绑定不可变 image digest，不能以可移动 tag 充当构建证据。"""
    lock = tmp_path / "ubuntu24-system-dependencies.lock"
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "platform": {
                    "id": "ubuntu",
                    "version_id": "24.04",
                    "codename": "noble",
                    "architecture": "amd64",
                },
                "builder_image": {
                    "reference": "ubuntu:24.04",
                    "digest": "ubuntu:24.04",
                },
                "apt_packages": [
                    {"name": "cmake", "version": "3.28.3-1build7", "architecture": "amd64"}
                ],
                "allowed_system_sonames": [
                    {"soname": "libc.so.6", "package": "libc6", "version": "2.39-0ubuntu8.8"}
                ],
            }
        ),
        encoding="utf-8",
    )

    verifier = _load_verifier()
    load_system_lock = getattr(verifier, "load_system_dependency_lock", None)
    assert callable(load_system_lock), "dependency verifier needs a system dependency lock parser"
    with pytest.raises(ValueError, match="builder image digest must be a sha256"):
        load_system_lock(lock)


def test_system_dependency_lock_accepts_apt_records_without_duplicate_package_name(tmp_path) -> None:
    """apt package 已由 name 标识，锁不得要求重复的 package 字段。"""
    lock = tmp_path / "ubuntu24-system-dependencies.lock"
    document = {
        "schema_version": 1,
        "platform": {
            "id": "ubuntu",
            "version_id": "24.04",
            "codename": "noble",
            "architecture": "amd64",
        },
        "builder_image": {
            "reference": "ubuntu:24.04",
            "digest": "sha256:" + "a" * 64,
        },
        "apt_packages": [
            {"name": "cmake", "version": "3.28.3-1build7", "architecture": "amd64"}
        ],
        "allowed_system_sonames": [
            {"soname": "libc.so.6", "package": "libc6", "version": "2.39-0ubuntu8.8"}
        ],
    }
    lock.write_text(json.dumps(document), encoding="utf-8")

    verifier = _load_verifier()
    load_system_lock = getattr(verifier, "load_system_dependency_lock", None)
    assert callable(load_system_lock), "dependency verifier needs a system dependency lock parser"
    assert load_system_lock(lock) == document


def test_locks_only_cli_verifies_a_system_dependency_lock(tmp_path) -> None:
    """公开 CLI 必须真实消费系统锁，而非把它留给调用方自行解析。"""
    lock = tmp_path / "ubuntu24-system-dependencies.lock"
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "platform": {
                    "id": "ubuntu",
                    "version_id": "24.04",
                    "codename": "noble",
                    "architecture": "amd64",
                },
                "builder_image": {
                    "reference": "ubuntu:24.04",
                    "digest": "sha256:" + "a" * 64,
                },
                "apt_packages": [
                    {"name": "cmake", "version": "3.28.3-1build7", "architecture": "amd64"}
                ],
                "allowed_system_sonames": [
                    {"soname": "libc.so.6", "package": "libc6", "version": "2.39-0ubuntu8.8"}
                ],
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(DEPENDENCY_VERIFIER),
            "--locks-only",
            "--system-lock",
            str(lock),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    assert "PASS: 0 dependency lock entries and system dependency lock verified" in result.stdout


def test_system_dependency_lock_rejects_floating_package_version(tmp_path) -> None:
    """系统锁中的 apt 版本必须是可复核的字面 Debian 版本。"""
    lock = tmp_path / "ubuntu24-system-dependencies.lock"
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "platform": {
                    "id": "ubuntu",
                    "version_id": "24.04",
                    "codename": "noble",
                    "architecture": "amd64",
                },
                "builder_image": {
                    "reference": "ubuntu:24.04",
                    "digest": "sha256:" + "a" * 64,
                },
                "apt_packages": [
                    {"name": "cmake", "version": "latest", "architecture": "amd64"}
                ],
                "allowed_system_sonames": [
                    {"soname": "libc.so.6", "package": "libc6", "version": "2.39-0ubuntu8.8"}
                ],
            }
        ),
        encoding="utf-8",
    )

    verifier = _load_verifier()
    load_system_lock = getattr(verifier, "load_system_dependency_lock", None)
    assert callable(load_system_lock), "dependency verifier needs a system dependency lock parser"
    with pytest.raises(ValueError, match="version must begin with a digit"):
        load_system_lock(lock)


def test_build_environment_evidence_rejects_legacy_unprobed_schema(tmp_path) -> None:
    """旧 schema 仅绑定 shell 文本，不能作为后续阶段的可执行环境合同。"""
    environment = {
        "STAGE4_CC": "/opt/stage4/bin/gcc",
        "STAGE4_CMAKE": "/opt/stage4/bin/cmake",
        "STAGE4_CXX": "/opt/stage4/bin/g++",
    }
    evidence = tmp_path / "stage4-build-env.json"
    environment_file = tmp_path / "stage4-build-env.sh"
    system_dependencies = {
        "libfixture.so.1": {
            "package": "libfixture1:amd64",
            "version": "1.2.3-1",
        }
    }
    verifier = _load_verifier()
    write_environment = getattr(verifier, "write_build_environment", None)
    verify_environment = getattr(verifier, "verify_build_environment", None)
    assert callable(write_environment), "dependency verifier needs a build environment writer"
    assert callable(verify_environment), "dependency verifier needs a build environment verifier"

    write_environment(
        environment,
        environment_file,
        evidence,
        system_dependencies=system_dependencies,
    )

    assert environment_file.read_text(encoding="utf-8") == (
        "export STAGE4_CC='/opt/stage4/bin/gcc'\n"
        "export STAGE4_CMAKE='/opt/stage4/bin/cmake'\n"
        "export STAGE4_CXX='/opt/stage4/bin/g++'\n"
    )
    document = json.loads(evidence.read_text(encoding="utf-8"))
    assert document["schema_version"] == 2
    assert document["environment"] == environment
    assert document["system_dependencies"] == system_dependencies
    with pytest.raises(ValueError, match="build environment evidence is invalid"):
        verify_environment(environment_file, evidence)


def test_build_environment_rejects_command_substitution_value(tmp_path) -> None:
    """环境证据不得接受命令替换语法，即使单引号当前会抑制其执行。"""
    verifier = _load_verifier()
    write_environment = getattr(verifier, "write_build_environment", None)
    assert callable(write_environment), "dependency verifier needs a build environment writer"

    with pytest.raises(ValueError, match="build environment assignment is invalid"):
        write_environment(
            {"STAGE4_CMAKE": "/opt/stage4/$(unexpected-command)"},
            tmp_path / "stage4-build-env.sh",
            tmp_path / "stage4-build-env.json",
            system_dependencies={
                "libfixture.so.1": {
                    "package": "libfixture1:amd64",
                    "version": "1.2.3-1",
                }
            },
        )


def test_build_environment_rejects_backtick_command_substitution(tmp_path) -> None:
    """环境证据也必须拒绝反引号形式的命令替换。"""
    verifier = _load_verifier()
    write_environment = getattr(verifier, "write_build_environment", None)
    assert callable(write_environment), "dependency verifier needs a build environment writer"

    with pytest.raises(ValueError, match="build environment assignment is invalid"):
        write_environment(
            {"STAGE4_CMAKE": "/opt/stage4/`unexpected-command`"},
            tmp_path / "stage4-build-env.sh",
            tmp_path / "stage4-build-env.json",
            system_dependencies={
                "libfixture.so.1": {
                    "package": "libfixture1:amd64",
                    "version": "1.2.3-1",
                }
            },
        )


def test_dependency_verifier_cli_rejects_legacy_unprobed_environment(tmp_path) -> None:
    """CLI 不能让只含 shell 摘要的旧环境证据绕过实时 probe。"""
    environment_file = tmp_path / "stage4-build-env.sh"
    evidence = tmp_path / "stage4-build-env.json"
    verifier = _load_verifier()
    write_environment = getattr(verifier, "write_build_environment", None)
    assert callable(write_environment), "dependency verifier needs a build environment writer"
    write_environment(
        {"STAGE4_CMAKE": "/opt/stage4/bin/cmake"},
        environment_file,
        evidence,
        system_dependencies={
            "libfixture.so.1": {
                "package": "libfixture1:amd64",
                "version": "1.2.3-1",
            }
        },
    )

    result = subprocess.run(
        [
            sys.executable,
            str(DEPENDENCY_VERIFIER),
            "--verify-env",
            str(environment_file),
            "--json",
            str(evidence),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "FAIL: build environment evidence is invalid" in result.stdout


def test_dependency_verifier_write_env_rejects_non_executable_micromamba(tmp_path) -> None:
    """正式 probe 必须把 micromamba 当作显式普通可执行输入。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["micromamba"] = tmp_path / "missing-micromamba"
    result = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "FAIL: --micromamba must be a regular executable" in result.stdout


def test_dependency_verifier_write_env_rejects_missing_package_cache(tmp_path) -> None:
    """micromamba 合法后，probe 必须继续验证 canonical package cache。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["python_package_cache"] = tmp_path / "missing-package-cache"
    result = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "FAIL: --python-package-cache must be an existing directory" in result.stdout


def test_dependency_verifier_write_env_rejects_missing_wheel_cache(tmp_path) -> None:
    """package cache 合法后，probe 必须继续验证 canonical wheel cache。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["python_wheel_cache"] = tmp_path / "missing-wheel-cache"
    result = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "FAIL: --python-wheel-cache must be an existing directory" in result.stdout


def test_dependency_verifier_write_env_rejects_missing_source_archive_cache(tmp_path) -> None:
    """两个 Python cache 合法后，probe 必须验证 canonical source archive cache。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["source_archive_cache"] = tmp_path / "missing-source-cache"
    result = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "FAIL: --source-archive-cache must be an existing directory" in result.stdout


def test_dependency_verifier_write_env_rejects_missing_mid360_reference(tmp_path) -> None:
    """三个 cache 合法后，probe 必须验证官方只读 LVX2 参考文件。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["mid360_reference_lvx2"] = tmp_path / "missing.lvx2"
    result = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "FAIL: --mid360-reference-lvx2 must be a regular file" in result.stdout


def test_dependency_verifier_write_env_rejects_non_executable_rviz2(tmp_path) -> None:
    """LVX2 合法后，probe 必须验证 Jazzy RViz2 是普通可执行文件。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["rviz2"] = tmp_path / "missing-rviz2"
    result = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "FAIL: --rviz2 must be a regular executable" in result.stdout


def test_build_environment_from_probe_inputs_exports_complete_contract(tmp_path) -> None:
    """探针成功时必须导出完整且固定的十四项 Stage 4 环境合同。"""
    executable_names = ("cmake", "ctest", "cc", "cxx", "protoc", "micromamba", "pcl", "rviz2")
    version_output = {
        "cmake": "cmake version 3.28.9",
        "ctest": "ctest version 3.28.9",
        "cc": "gcc 13.3.0",
        "cxx": "g++ 13.3.0",
        "protoc": "libprotoc 33.6",
    }
    executables = {}
    for name in executable_names:
        path = (
            tmp_path / "validation-prefix" / "bin" / name
            if name == "pcl"
            else tmp_path / name
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        output = version_output.get(name)
        script = "#!/bin/sh\nexit 0\n" if output is None else f"#!/bin/sh\nprintf '{output}\\n'\n"
        path.write_text(script, encoding="utf-8")
        path.chmod(0o755)
        executables[name] = path
    package_cache = tmp_path / "package-cache"
    wheel_cache = tmp_path / "wheel-cache"
    source_cache = tmp_path / "source-cache"
    dependency_prefix = tmp_path / "dependency-prefix"
    for directory in (package_cache, wheel_cache, source_cache, dependency_prefix):
        directory.mkdir()
    lvx2 = tmp_path / "Indoor_sampledata.lvx2"
    lvx2.write_bytes(b"fixture LVX2")
    system_probe = _system_dependency_probe_inputs(tmp_path)

    verifier = _load_verifier()
    build_environment = getattr(verifier, "build_environment_from_probe_inputs", None)
    assert callable(build_environment), "dependency verifier needs a complete build environment mapper"
    environment = build_environment(
        cmake=executables["cmake"],
        ctest=executables["ctest"],
        cc=executables["cc"],
        cxx=executables["cxx"],
        protoc=executables["protoc"],
        micromamba=executables["micromamba"],
        python_package_cache=package_cache,
        python_wheel_cache=wheel_cache,
        source_archive_cache=source_cache,
        dependency_prefix=dependency_prefix,
        pcl_pcd2ply=executables["pcl"],
        system_lock=system_probe["system_lock"],
        ldd=system_probe["ldd"],
        dpkg_query=system_probe["dpkg_query"],
        mid360_reference_lvx2=lvx2,
        rviz2=executables["rviz2"],
    )

    assert environment == {
        "STAGE4_CC": str(executables["cc"]),
        "STAGE4_CMAKE": str(executables["cmake"]),
        "STAGE4_CMAKE_PREFIX_PATH": str(dependency_prefix),
        "STAGE4_CTEST": str(executables["ctest"]),
        "STAGE4_CXX": str(executables["cxx"]),
        "STAGE4_DEPENDENCY_PREFIX": str(dependency_prefix),
        "STAGE4_MICROMAMBA": str(executables["micromamba"]),
        "STAGE4_MID360_REFERENCE_LVX2": str(lvx2),
        "STAGE4_PCL_PCD2PLY": str(executables["pcl"]),
        "STAGE4_PROTOC": str(executables["protoc"]),
        "STAGE4_PYTHON_PACKAGE_CACHE": str(package_cache),
        "STAGE4_PYTHON_WHEEL_CACHE": str(wheel_cache),
        "STAGE4_RVIZ2": str(executables["rviz2"]),
        "STAGE4_SOURCE_ARCHIVE_CACHE": str(source_cache),
    }


def test_build_environment_from_probe_inputs_rejects_pcl_system_package_version_drift(
    tmp_path,
) -> None:
    """PCL 解析到的系统库版本与 lock 漂移时，环境 probe 必须拒绝写入。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["dpkg_query"].write_text(
        "#!/bin/sh\nprintf 'installed\\t9.9.9-1\\n'\n",
        encoding="utf-8",
    )

    verifier = _load_verifier()
    build_environment = getattr(verifier, "build_environment_from_probe_inputs", None)
    assert callable(build_environment), "dependency verifier needs a complete build environment mapper"
    with pytest.raises(ValueError, match="system package version does not match the lock"):
        build_environment(
            cmake=inputs["cmake"],
            ctest=inputs["ctest"],
            cc=inputs["cc"],
            cxx=inputs["cxx"],
            protoc=inputs["protoc"],
            micromamba=inputs["micromamba"],
            python_package_cache=inputs["python_package_cache"],
            python_wheel_cache=inputs["python_wheel_cache"],
            source_archive_cache=inputs["source_archive_cache"],
            dependency_prefix=inputs["dependency_prefix"],
            pcl_pcd2ply=inputs["pcl_pcd2ply"],
            system_lock=inputs["system_lock"],
            ldd=inputs["ldd"],
            dpkg_query=inputs["dpkg_query"],
            mid360_reference_lvx2=inputs["mid360_reference_lvx2"],
            rviz2=inputs["rviz2"],
        )


def test_build_environment_probe_rejects_non_system_dso_outside_dependency_prefix(
    tmp_path,
) -> None:
    """PCL validator 的非系统 DSO 必须解析到本轮私有 dependency prefix。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["ldd"].write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' 'libunknown.so.1 => /tmp/libunknown.so.1 (0x1)'\n"
        "printf '%s\\n' 'libfixture.so.1 => /lib/x86_64-linux-gnu/libfixture.so.1 (0x2)'\n",
        encoding="utf-8",
    )

    verifier = _load_verifier()
    build_environment = getattr(verifier, "build_environment_from_probe_inputs", None)
    assert callable(build_environment), "dependency verifier needs a complete build environment mapper"
    with pytest.raises(ValueError, match="non-system dependency is outside private prefix"):
        build_environment(
            cmake=inputs["cmake"],
            ctest=inputs["ctest"],
            cc=inputs["cc"],
            cxx=inputs["cxx"],
            protoc=inputs["protoc"],
            micromamba=inputs["micromamba"],
            python_package_cache=inputs["python_package_cache"],
            python_wheel_cache=inputs["python_wheel_cache"],
            source_archive_cache=inputs["source_archive_cache"],
            dependency_prefix=inputs["dependency_prefix"],
            pcl_pcd2ply=inputs["pcl_pcd2ply"],
            system_lock=inputs["system_lock"],
            ldd=inputs["ldd"],
            dpkg_query=inputs["dpkg_query"],
            mid360_reference_lvx2=inputs["mid360_reference_lvx2"],
            rviz2=inputs["rviz2"],
        )


def test_build_environment_probe_rejects_direct_non_system_ldd_path(tmp_path) -> None:
    """不含箭头的绝对 ldd 路径也必须遵守私有 DSO prefix 门禁。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["ldd"].write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' '/tmp/libunknown.so.1 (0x1)'\n"
        "printf '%s\\n' 'libfixture.so.1 => /lib/x86_64-linux-gnu/libfixture.so.1 (0x2)'\n",
        encoding="utf-8",
    )

    verifier = _load_verifier()
    build_environment = getattr(verifier, "build_environment_from_probe_inputs", None)
    assert callable(build_environment), "dependency verifier needs a complete build environment mapper"
    with pytest.raises(ValueError, match="non-system dependency is outside private prefix"):
        build_environment(
            cmake=inputs["cmake"], ctest=inputs["ctest"], cc=inputs["cc"],
            cxx=inputs["cxx"], protoc=inputs["protoc"], micromamba=inputs["micromamba"],
            python_package_cache=inputs["python_package_cache"],
            python_wheel_cache=inputs["python_wheel_cache"],
            source_archive_cache=inputs["source_archive_cache"],
            dependency_prefix=inputs["dependency_prefix"], pcl_pcd2ply=inputs["pcl_pcd2ply"],
            system_lock=inputs["system_lock"], ldd=inputs["ldd"], dpkg_query=inputs["dpkg_query"],
            mid360_reference_lvx2=inputs["mid360_reference_lvx2"], rviz2=inputs["rviz2"],
        )


def test_build_environment_probe_accepts_private_dso_from_validation_prefix(tmp_path) -> None:
    """PCL validator 可链接同一轮 validation prefix 的私有 PCL 库。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    validation_library = (
        inputs["pcl_pcd2ply"].parent.parent / "lib" / "libpcl_private.so.1"
    )
    validation_library.parent.mkdir(parents=True)
    validation_library.write_bytes(b"fixture validation dso")
    inputs["ldd"].write_text(
        "#!/bin/sh\n"
        f"printf '%s\\n' 'libpcl_private.so.1 => {validation_library} (0x1)'\n"
        "printf '%s\\n' 'libfixture.so.1 => /lib/x86_64-linux-gnu/libfixture.so.1 (0x2)'\n",
        encoding="utf-8",
    )

    verifier = _load_verifier()
    build_environment = getattr(verifier, "build_environment_from_probe_inputs", None)
    assert callable(build_environment), "dependency verifier needs a complete build environment mapper"
    environment = build_environment(
        cmake=inputs["cmake"],
        ctest=inputs["ctest"],
        cc=inputs["cc"],
        cxx=inputs["cxx"],
        protoc=inputs["protoc"],
        micromamba=inputs["micromamba"],
        python_package_cache=inputs["python_package_cache"],
        python_wheel_cache=inputs["python_wheel_cache"],
        source_archive_cache=inputs["source_archive_cache"],
        dependency_prefix=inputs["dependency_prefix"],
        pcl_pcd2ply=inputs["pcl_pcd2ply"],
        system_lock=inputs["system_lock"],
        ldd=inputs["ldd"],
        dpkg_query=inputs["dpkg_query"],
        mid360_reference_lvx2=inputs["mid360_reference_lvx2"],
        rviz2=inputs["rviz2"],
    )

    assert environment["STAGE4_PCL_PCD2PLY"] == str(inputs["pcl_pcd2ply"])


def test_dependency_verifier_verify_env_rejects_validation_prefix_dso_drift(tmp_path) -> None:
    """v3 合同必须固定 validation prefix 内被 PCL validator 解析的私有库树。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    validation_library = (
        inputs["pcl_pcd2ply"].parent.parent / "lib" / "libpcl_private.so.1"
    )
    validation_library.parent.mkdir(parents=True)
    validation_library.write_bytes(b"fixture validation dso")
    inputs["ldd"].write_text(
        "#!/bin/sh\n"
        f"printf '%s\\n' 'libpcl_private.so.1 => {validation_library} (0x1)'\n"
        "printf '%s\\n' 'libfixture.so.1 => /lib/x86_64-linux-gnu/libfixture.so.1 (0x2)'\n",
        encoding="utf-8",
    )
    environment_file = tmp_path / "stage4-build-env.sh"
    evidence = tmp_path / "stage4-build-env.json"
    written = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )
    assert written.returncode == 0, written.stdout
    with validation_library.open("ab") as stream:
        stream.write(b"\nchanged validation dso\n")

    verified = subprocess.run(
        [sys.executable, str(DEPENDENCY_VERIFIER), "--verify-env", str(environment_file), "--json", str(evidence)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert verified.returncode != 0
    assert "FAIL: build environment runtime identity differs from evidence" in verified.stdout


def test_dependency_verifier_write_env_exports_complete_explicit_probe_contract(
    tmp_path,
) -> None:
    """CLI 必须只消费调用方显式给出的本轮前缀和工具，成功生成绑定环境。"""
    version_output = {
        "cmake": "cmake version 3.28.9",
        "ctest": "ctest version 3.28.9",
        "cc": "gcc 13.3.0",
        "cxx": "g++ 13.3.0",
        "protoc": "libprotoc 33.6",
    }
    executables = {}
    for name, output in version_output.items():
        path = tmp_path / name
        path.write_text(f"#!/bin/sh\nprintf '{output}\\n'\n", encoding="utf-8")
        path.chmod(0o755)
        executables[name] = path
    for name in ("micromamba", "pcl_pcd2ply", "rviz2"):
        path = (
            tmp_path / "validation-prefix" / "bin" / name
            if name == "pcl_pcd2ply"
            else tmp_path / name
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        path.chmod(0o755)
        executables[name] = path
    package_cache = tmp_path / "package-cache"
    wheel_cache = tmp_path / "wheel-cache"
    source_cache = tmp_path / "source-cache"
    dependency_prefix = tmp_path / "dependency-prefix"
    for directory in (package_cache, wheel_cache, source_cache, dependency_prefix):
        directory.mkdir()
    lvx2 = tmp_path / "Indoor_sampledata.lvx2"
    lvx2.write_bytes(b"fixture LVX2")
    system_probe = _system_dependency_probe_inputs(tmp_path)
    environment_file = tmp_path / "stage4-build-env.sh"
    evidence = tmp_path / "stage4-build-env.json"

    result = subprocess.run(
        [
            sys.executable,
            str(DEPENDENCY_VERIFIER),
            "--cmake", str(executables["cmake"]),
            "--ctest", str(executables["ctest"]),
            "--cc", str(executables["cc"]),
            "--cxx", str(executables["cxx"]),
            "--protoc", str(executables["protoc"]),
            "--micromamba", str(executables["micromamba"]),
            "--python-package-cache", str(package_cache),
            "--python-wheel-cache", str(wheel_cache),
            "--source-archive-cache", str(source_cache),
            "--dependency-prefix", str(dependency_prefix),
            "--pcl-pcd2ply", str(executables["pcl_pcd2ply"]),
            "--system-lock", str(system_probe["system_lock"]),
            "--ldd", str(system_probe["ldd"]),
            "--dpkg-query", str(system_probe["dpkg_query"]),
            "--mid360-reference-lvx2", str(lvx2),
            "--rviz2", str(executables["rviz2"]),
            "--write-env", str(environment_file),
            "--json", str(evidence),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stdout
    assert "PASS: build environment written" in result.stdout
    assert environment_file.is_file()
    assert evidence.is_file()
    verified = subprocess.run(
        [
            sys.executable,
            str(DEPENDENCY_VERIFIER),
            "--verify-env", str(environment_file),
            "--json", str(evidence),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert verified.returncode == 0, verified.stdout


@pytest.mark.parametrize(
    ("mutated_input", "expected_failure"),
    [
        ("micromamba", "--micromamba must be a regular executable"),
        ("python_package_cache", "--python-package-cache must be an existing directory"),
        ("mid360_reference_lvx2", "--mid360-reference-lvx2 must be a regular file"),
        (
            "system_package",
            "system package version does not match the lock: libfixture1:amd64",
        ),
    ],
)
def test_dependency_verifier_verify_env_reprobes_mutated_runtime_inputs(
    tmp_path, mutated_input, expected_failure
) -> None:
    """已写环境合同必须在每次使用前复核实际输入，不能只验证自身摘要。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    environment_file = tmp_path / "stage4-build-env.sh"
    evidence = tmp_path / "stage4-build-env.json"

    written = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )
    assert written.returncode == 0, written.stdout
    if mutated_input == "system_package":
        inputs["dpkg_query"].write_text(
            "#!/bin/sh\nprintf 'installed\\t9.9.9-1\\n'\n",
            encoding="utf-8",
        )
    elif mutated_input == "python_package_cache":
        inputs[mutated_input].rmdir()
    else:
        inputs[mutated_input].unlink()

    verified = subprocess.run(
        [
            sys.executable,
            str(DEPENDENCY_VERIFIER),
            "--verify-env",
            str(environment_file),
            "--json",
            str(evidence),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert verified.returncode != 0
    assert f"FAIL: {expected_failure}" in verified.stdout


@pytest.mark.parametrize(
    "mutated_input",
    (
        "micromamba",
        "python_package_cache",
        "mid360_reference_lvx2",
        "pcl_pcd2ply",
        "rviz2",
        "ldd",
        "system_lock",
    ),
)
def test_dependency_verifier_verify_env_rejects_in_place_runtime_identity_drift(
    tmp_path, mutated_input
) -> None:
    """v3 合同必须识别仍存在但内容已漂移的每类运行时输入。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    environment_file = tmp_path / "stage4-build-env.sh"
    evidence = tmp_path / "stage4-build-env.json"

    written = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )
    assert written.returncode == 0, written.stdout
    if mutated_input == "python_package_cache":
        (inputs[mutated_input] / "unexpected-package").write_bytes(b"drift")
    elif mutated_input == "system_lock":
        with inputs[mutated_input].open("ab") as stream:
            stream.write(b"\n ")
    else:
        with inputs[mutated_input].open("ab") as stream:
            stream.write(b"\n# identity drift\n")

    verified = subprocess.run(
        [
            sys.executable,
            str(DEPENDENCY_VERIFIER),
            "--verify-env",
            str(environment_file),
            "--json",
            str(evidence),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert verified.returncode != 0
    assert "FAIL: build environment runtime identity differs from evidence" in verified.stdout


def test_build_environment_from_probe_inputs_rejects_wrong_cmake_version(tmp_path) -> None:
    """统一环境拒绝非 3.28.x CMake，不能只验证工具文件存在。"""
    version_output = {
        "cmake": "cmake version 3.27.9",
        "ctest": "ctest version 3.28.9",
        "cc": "gcc 13.3.0",
        "cxx": "g++ 13.3.0",
        "protoc": "libprotoc 33.6",
    }
    executables = {}
    for name, output in version_output.items():
        path = tmp_path / name
        path.write_text(f"#!/bin/sh\nprintf '{output}\\n'\n", encoding="utf-8")
        path.chmod(0o755)
        executables[name] = path
    for name in ("micromamba", "pcl", "rviz2"):
        path = (
            tmp_path / "validation-prefix" / "bin" / name
            if name == "pcl"
            else tmp_path / name
        )
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
        path.chmod(0o755)
        executables[name] = path
    package_cache = tmp_path / "package-cache"
    wheel_cache = tmp_path / "wheel-cache"
    source_cache = tmp_path / "source-cache"
    dependency_prefix = tmp_path / "dependency-prefix"
    for directory in (package_cache, wheel_cache, source_cache, dependency_prefix):
        directory.mkdir()
    lvx2 = tmp_path / "Indoor_sampledata.lvx2"
    lvx2.write_bytes(b"fixture LVX2")
    system_probe = _system_dependency_probe_inputs(tmp_path)

    verifier = _load_verifier()
    build_environment = getattr(verifier, "build_environment_from_probe_inputs", None)
    assert callable(build_environment), "dependency verifier needs a complete build environment mapper"
    with pytest.raises(ValueError, match="cmake version does not match the frozen contract"):
        build_environment(
            cmake=executables["cmake"],
            ctest=executables["ctest"],
            cc=executables["cc"],
            cxx=executables["cxx"],
            protoc=executables["protoc"],
            micromamba=executables["micromamba"],
            python_package_cache=package_cache,
            python_wheel_cache=wheel_cache,
            source_archive_cache=source_cache,
            dependency_prefix=dependency_prefix,
            pcl_pcd2ply=executables["pcl"],
            system_lock=system_probe["system_lock"],
            ldd=system_probe["ldd"],
            dpkg_query=system_probe["dpkg_query"],
            mid360_reference_lvx2=lvx2,
            rviz2=executables["rviz2"],
        )


def test_require_command_version_runs_the_explicit_tool_path(tmp_path) -> None:
    """版本探针必须执行指定绝对路径，而非从 PATH 猜测同名工具。"""
    cmake = tmp_path / "cmake"
    cmake.write_text("#!/bin/sh\nprintf 'cmake version 3.28.9\\n'\n", encoding="utf-8")
    cmake.chmod(0o755)

    verifier = _load_verifier()
    require_version = getattr(verifier, "require_command_version", None)
    assert callable(require_version), "dependency verifier needs an explicit tool version probe"
    assert require_version(cmake, "cmake", r"cmake version 3\.28\.\d+") == "cmake version 3.28.9"


def test_dependency_verifier_write_env_rejects_missing_explicit_dependency_prefix(tmp_path) -> None:
    """公开 probe 不得回退到历史路径，显式前缀缺失时必须失败。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["dependency_prefix"] = tmp_path / "missing-prefix"
    result = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "FAIL: --dependency-prefix must be an existing directory" in result.stdout


def test_dependency_verifier_write_env_rejects_missing_explicit_pcl_validator(tmp_path) -> None:
    """主依赖前缀合法时，显式 PCL validator 缺失仍必须失败。"""
    inputs = _complete_write_env_probe_inputs(tmp_path)
    inputs["pcl_pcd2ply"] = tmp_path / "missing-pcl-pcd2ply"
    result = subprocess.run(
        _write_env_probe_command(inputs, tmp_path),
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "FAIL: --pcl-pcd2ply must be a regular executable" in result.stdout


def test_real_dependency_locks_cover_the_frozen_stage4_source_set() -> None:
    """真实 C++ 与 ROS 锁必须覆盖阶段四构建需要的全部十五份源码。"""
    assert CPP_DEPENDENCY_LOCK.is_file(), "C++ dependency lock is not implemented"
    assert ROS2_DEPENDENCY_LOCK.is_file(), "ROS 2 dependency lock is not implemented"

    verifier = _load_verifier()
    load_lock = getattr(verifier, "load_dependency_lock", None)
    assert callable(load_lock), "dependency verifier needs a structured lock parser"
    entries = (*load_lock(CPP_DEPENDENCY_LOCK), *load_lock(ROS2_DEPENDENCY_LOCK))
    by_name = {entry.name: entry for entry in entries}

    assert set(by_name) == {
        "abseil-cpp",
        "ecal-asio",
        "ecal-ecaludp",
        "ecal-protozero",
        "ecal-recycle",
        "ecal-tclap",
        "ecal-tcp-pubsub",
        "ecal-yaml-cpp",
        "ecal",
        "protobuf",
        "mcap",
        "zstd",
        "pcl",
        "livox_ros_driver2",
        "Livox-SDK2",
    }
    assert by_name["Livox-SDK2"].ref_kind == "commit"
    assert by_name["Livox-SDK2"].ref == by_name["Livox-SDK2"].commit
    assert by_name["ecal-asio"].commit == "ed6aa8a13d51dfc6c00ae453fc9fb7df5d6ea963"
    assert by_name["ecal-ecaludp"].commit == "96e29b8cecfc56a627f7920d972930ebc62d1e79"
    assert by_name["ecal-protozero"].commit == "89a55ad2962cca3adbe8383a4b6d9a8411352ef2"
    assert by_name["ecal-recycle"].commit == "3f3d27ecdee87af9167adf1d2c2345ca2cbe1c94"
    assert by_name["ecal-tclap"].commit == "58c5c8ef24111072fc21fb723f8ab45d23395809"
    assert by_name["ecal-tcp-pubsub"].commit == "352e711b9ef10fec42ba7536bda244f43bf092cc"
    assert by_name["ecal-yaml-cpp"].commit == "3d2888cc8a45da2f420454ad728cdfad01a3d54f"
    commit_sources = {
        "Livox-SDK2",
        "abseil-cpp",
        "ecal-asio",
        "ecal-ecaludp",
        "ecal-protozero",
        "ecal-recycle",
        "ecal-tclap",
        "ecal-tcp-pubsub",
        "ecal-yaml-cpp",
    }
    for name, entry in by_name.items():
        if name not in commit_sources:
            assert entry.ref_kind == "tag"


def test_dependency_lock_rejects_entry_without_consumers(tmp_path) -> None:
    """源码归档没有构建消费者时不得进入锁，避免无归属缓存输入。"""
    lock = tmp_path / "cpp-dependencies.lock"
    commit = "a" * 40
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "fixture",
                        "url": "https://github.com/example/fixture/archive/" + commit + ".tar.gz",
                        "ref_kind": "commit",
                        "ref": commit,
                        "commit": commit,
                        "archive": {
                            "format": "tar.gz",
                            "size": 1,
                            "sha256": "b" * 64,
                        },
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    verifier = _load_verifier()
    load_lock = getattr(verifier, "load_dependency_lock", None)
    assert callable(load_lock), "dependency verifier needs a structured lock parser"
    with pytest.raises(ValueError, match="consumers must be a nonempty list"):
        load_lock(lock)


def test_source_archive_parser_rejects_parent_path_escape() -> None:
    """共享归档 parser 必须先拒绝根外路径，不能交给 tar 的默认行为。"""
    assert SOURCE_ARCHIVE_PARSER.is_file(), "stage 4 source archive parser is not implemented"
    parser = _load_source_archive_parser()
    validate_path = getattr(parser, "validate_member_path", None)
    assert callable(validate_path), "source archive parser needs a member path validator"

    with pytest.raises(ValueError, match="must not contain parent traversal"):
        validate_path("stage4-source/../escape.txt")


def test_source_archive_parser_rejects_absolute_member_path() -> None:
    """归档成员使用绝对路径时必须在解包前 fail closed。"""
    parser = _load_source_archive_parser()
    validate_path = getattr(parser, "validate_member_path", None)
    assert callable(validate_path), "source archive parser needs a member path validator"

    with pytest.raises(ValueError, match="must be relative"):
        validate_path("/var/tmp/escape.txt")


def test_source_archive_parser_rejects_current_directory_segment() -> None:
    """成员路径中的 . 段会造成规范化冲突，必须在预检时拒绝。"""
    parser = _load_source_archive_parser()
    validate_path = getattr(parser, "validate_member_path", None)
    assert callable(validate_path), "source archive parser needs a member path validator"

    with pytest.raises(ValueError, match="must not contain current-directory traversal"):
        validate_path("stage4-source/./README.md")


def test_source_archive_parser_rejects_empty_member_path() -> None:
    """空成员名没有可验证的顶层根，必须在预检时拒绝。"""
    parser = _load_source_archive_parser()
    validate_path = getattr(parser, "validate_member_path", None)
    assert callable(validate_path), "source archive parser needs a member path validator"

    with pytest.raises(ValueError, match="must not be empty"):
        validate_path("")


@pytest.mark.parametrize("path", ["stage4-source/line\nbreak", "stage4-source/zero\x00byte"])
def test_source_archive_parser_rejects_control_characters(path: str) -> None:
    """控制字符会使归档路径审计不可靠，必须在预检时拒绝。"""
    parser = _load_source_archive_parser()
    validate_path = getattr(parser, "validate_member_path", None)
    assert callable(validate_path), "source archive parser needs a member path validator"

    with pytest.raises(ValueError, match="must not contain control characters"):
        validate_path(path)


def test_source_archive_parser_rejects_multiple_top_level_roots() -> None:
    """一个源码归档只能物化到单一顶层目录，混合根必须拒绝。"""
    parser = _load_source_archive_parser()
    validate_paths = getattr(parser, "validate_member_paths", None)
    assert callable(validate_paths), "source archive parser needs a member collection validator"

    with pytest.raises(ValueError, match="must use exactly one top-level root"):
        validate_paths(["source-a/README.md", "source-b/LICENSE"])


def test_source_archive_parser_rejects_duplicate_tar_member_path(tmp_path) -> None:
    """真实 tar 内同名成员会覆盖物化目标，必须在解包前拒绝。"""
    archive = tmp_path / "duplicate.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        for contents in (b"first", b"second"):
            member = tarfile.TarInfo("stage4-source/README.md")
            member.size = len(contents)
            handle.addfile(member, io.BytesIO(contents))

    parser = _load_source_archive_parser()
    inspect_archive = getattr(parser, "inspect_archive", None)
    assert callable(inspect_archive), "source archive parser needs a tar inspection entrypoint"
    with pytest.raises(ValueError, match="duplicate member path"):
        inspect_archive(archive)


def test_source_archive_parser_rejects_file_directory_tar_conflict(tmp_path) -> None:
    """普通文件不能同时成为另一个成员的父目录。"""
    archive = tmp_path / "file-directory-conflict.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        file_member = tarfile.TarInfo("stage4-source/item")
        file_member.size = 1
        handle.addfile(file_member, io.BytesIO(b"x"))
        child_member = tarfile.TarInfo("stage4-source/item/child.txt")
        child_member.size = 1
        handle.addfile(child_member, io.BytesIO(b"y"))

    parser = _load_source_archive_parser()
    inspect_archive = getattr(parser, "inspect_archive", None)
    assert callable(inspect_archive), "source archive parser needs a tar inspection entrypoint"
    with pytest.raises(ValueError, match="file-directory conflict"):
        inspect_archive(archive)


def test_source_archive_parser_rejects_special_tar_member(tmp_path) -> None:
    """设备、FIFO 等特殊 tar 成员不能进入安全物化流程。"""
    archive = tmp_path / "special-member.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        member = tarfile.TarInfo("stage4-source/device")
        member.type = tarfile.CHRTYPE
        handle.addfile(member)

    parser = _load_source_archive_parser()
    inspect_archive = getattr(parser, "inspect_archive", None)
    assert callable(inspect_archive), "source archive parser needs a tar inspection entrypoint"
    with pytest.raises(ValueError, match="unsupported special member"):
        inspect_archive(archive)


def test_source_archive_parser_rejects_symlink_escaping_top_level_root(tmp_path) -> None:
    """相对 symlink 也不能经由 .. 逃出唯一顶层根。"""
    archive = tmp_path / "escaping-symlink.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        target = tarfile.TarInfo("stage4-source/README.md")
        target.size = 1
        handle.addfile(target, io.BytesIO(b"x"))
        link = tarfile.TarInfo("stage4-source/link")
        link.type = tarfile.SYMTYPE
        link.linkname = "../outside"
        handle.addfile(link)

    parser = _load_source_archive_parser()
    inspect_archive = getattr(parser, "inspect_archive", None)
    assert callable(inspect_archive), "source archive parser needs a tar inspection entrypoint"
    with pytest.raises(ValueError, match="symlink target escapes top-level root"):
        inspect_archive(archive)


def test_source_archive_parser_rejects_dangling_symlink(tmp_path) -> None:
    """根内相对 symlink 的终点必须是归档内已声明成员。"""
    archive = tmp_path / "dangling-symlink.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        link = tarfile.TarInfo("stage4-source/link")
        link.type = tarfile.SYMTYPE
        link.linkname = "missing.txt"
        handle.addfile(link)

    parser = _load_source_archive_parser()
    inspect_archive = getattr(parser, "inspect_archive", None)
    assert callable(inspect_archive), "source archive parser needs a tar inspection entrypoint"
    with pytest.raises(ValueError, match="symlink target is not a declared member"):
        inspect_archive(archive)


def test_source_archive_parser_rejects_cyclic_symlink_chain(tmp_path) -> None:
    """多跳 symlink 链若形成环，安全物化必须在写入前拒绝。"""
    archive = tmp_path / "cyclic-symlink.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        for name, target in (("stage4-source/a", "b"), ("stage4-source/b", "a")):
            link = tarfile.TarInfo(name)
            link.type = tarfile.SYMTYPE
            link.linkname = target
            handle.addfile(link)

    parser = _load_source_archive_parser()
    inspect_archive = getattr(parser, "inspect_archive", None)
    assert callable(inspect_archive), "source archive parser needs a tar inspection entrypoint"
    with pytest.raises(ValueError, match="symlink chain contains a cycle"):
        inspect_archive(archive)


def test_source_archive_parser_materializes_file_symlink_as_regular_file(tmp_path) -> None:
    """安全物化必须把 file symlink 深拷贝成零链接普通文件。"""
    archive = tmp_path / "file-symlink.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        target = tarfile.TarInfo("stage4-source/data.txt")
        target.size = 7
        handle.addfile(target, io.BytesIO(b"payload"))
        link = tarfile.TarInfo("stage4-source/alias.txt")
        link.type = tarfile.SYMTYPE
        link.linkname = "data.txt"
        handle.addfile(link)

    parser = _load_source_archive_parser()
    materialize = getattr(parser, "materialize_archive", None)
    assert callable(materialize), "source archive parser needs a safe materialization entrypoint"
    output = tmp_path / "materialized"
    root = materialize(archive, output)
    alias = output / root / "alias.txt"

    assert alias.is_file()
    assert not alias.is_symlink()
    assert alias.read_bytes() == b"payload"


def test_source_archive_parser_materializes_directory_symlink_as_deep_copy(tmp_path) -> None:
    """目录 symlink 必须递归物化完整子树，不能留下链接或空目录。"""
    archive = tmp_path / "directory-symlink.tar.gz"
    with tarfile.open(archive, "w:gz") as handle:
        for directory in ("stage4-source/data", "stage4-source/data/nested"):
            member = tarfile.TarInfo(directory)
            member.type = tarfile.DIRTYPE
            handle.addfile(member)
        target = tarfile.TarInfo("stage4-source/data/nested/value.txt")
        target.size = 7
        handle.addfile(target, io.BytesIO(b"payload"))
        link = tarfile.TarInfo("stage4-source/alias")
        link.type = tarfile.SYMTYPE
        link.linkname = "data"
        handle.addfile(link)

    parser = _load_source_archive_parser()
    materialize = getattr(parser, "materialize_archive", None)
    assert callable(materialize), "source archive parser needs a safe materialization entrypoint"
    output = tmp_path / "materialized"
    root = materialize(archive, output)
    copied = output / root / "alias" / "nested" / "value.txt"

    assert copied.is_file()
    assert not (output / root / "alias").is_symlink()
    assert copied.read_bytes() == b"payload"


def test_source_archive_parser_digests_zero_link_materialized_tree(tmp_path) -> None:
    """物化树 digest 必须记录普通成员数量、字节数和稳定 SHA-256。"""
    tree = tmp_path / "tree"
    tree.mkdir()
    (tree / "payload.txt").write_bytes(b"payload")

    parser = _load_source_archive_parser()
    digest_tree = getattr(parser, "materialized_tree_digest", None)
    assert callable(digest_tree), "source archive parser needs a materialized tree digest"
    digest = digest_tree(tree)

    assert digest["materialized_member_count"] == 1
    assert digest["materialized_regular_bytes"] == 7
    assert isinstance(digest["materialized_tree_sha256"], str)
    assert len(digest["materialized_tree_sha256"]) == 64


def test_repository_system_lock_covers_pcl_validator_build_contract() -> None:
    """仓库锁必须固定已实测的 PCL 验证器构建与运行时边界。"""
    lock = ROOT / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"
    verifier = _load_verifier()
    load_system_lock = getattr(verifier, "load_system_dependency_lock", None)

    assert lock.is_file(), "stage 4 needs a checked-in Ubuntu 24.04 system dependency lock"
    assert callable(load_system_lock), "dependency verifier needs a system dependency lock parser"
    document = load_system_lock(lock)

    assert document["builder_image"] == {
        "reference": "ubuntu:24.04",
        "digest": "sha256:561618e2c15bf2397621dd04f96926663a3b5616c189cf7e38db7e82f5c538ea",
    }
    packages = {record["name"]: record["version"] for record in document["apt_packages"]}
    assert packages["cmake"] == "3.28.3-1build7"
    assert packages["g++-13"] == "13.3.0-6ubuntu2~24.04.1"
    assert packages["libboost-filesystem1.83.0:amd64"] == "1.83.0-2.1ubuntu3.2"
    assert packages["libboost-iostreams1.83.0:amd64"] == "1.83.0-2.1ubuntu3.2"
    assert packages["libboost-system1.83.0:amd64"] == "1.83.0-2.1ubuntu3.2"
    assert packages["libgomp1:amd64"] == "14.2.0-4ubuntu2~24.04.1"
    assert packages["iproute2"] == "6.1.0-1ubuntu6.4"
    assert packages["python3"] == "3.12.3-0ubuntu2.1"
    sonames = {record["soname"] for record in document["allowed_system_sonames"]}
    assert {"libboost_filesystem.so.1.83.0", "libboost_iostreams.so.1.83.0", "libgomp.so.1"} <= sonames


def test_system_dependency_verifier_records_exact_locked_soname_package_versions(
    tmp_path,
) -> None:
    """已解析系统 SONAME 必须绑定锁定 package 的精确已安装版本。"""
    lock = tmp_path / "ubuntu24-system-dependencies.lock"
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "platform": {
                    "id": "ubuntu",
                    "version_id": "24.04",
                    "codename": "noble",
                    "architecture": "amd64",
                },
                "builder_image": {
                    "reference": "ubuntu:24.04",
                    "digest": "sha256:" + "a" * 64,
                },
                "apt_packages": [
                    {
                        "name": "libfixture1:amd64",
                        "version": "1.2.3-1",
                        "architecture": "amd64",
                    }
                ],
                "allowed_system_sonames": [
                    {
                        "soname": "libfixture.so.1",
                        "package": "libfixture1:amd64",
                        "version": "1.2.3-1",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    dpkg_query = tmp_path / "dpkg-query"
    dpkg_query.write_text(
        "#!/bin/sh\nprintf 'installed\\t1.2.3-1\\n'\n",
        encoding="utf-8",
    )
    dpkg_query.chmod(0o755)

    verifier = _load_verifier()
    verify_sonames = getattr(verifier, "verify_system_soname_packages", None)
    assert callable(verify_sonames), (
        "dependency verifier needs a locked system SONAME package verifier"
    )
    assert verify_sonames(
        lock,
        ("libfixture.so.1",),
        dpkg_query=dpkg_query,
    ) == {
        "libfixture.so.1": {
            "package": "libfixture1:amd64",
            "version": "1.2.3-1",
        }
    }


def test_pcl_system_dependency_verifier_ignores_private_prefix_but_checks_system_sonames(
    tmp_path,
) -> None:
    """PCL 的私有库不走宿主包锁，系统库必须逐项锁定并核验。"""
    lock = tmp_path / "ubuntu24-system-dependencies.lock"
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "platform": {
                    "id": "ubuntu",
                    "version_id": "24.04",
                    "codename": "noble",
                    "architecture": "amd64",
                },
                "builder_image": {
                    "reference": "ubuntu:24.04",
                    "digest": "sha256:" + "a" * 64,
                },
                "apt_packages": [
                    {
                        "name": "libfixture1:amd64",
                        "version": "1.2.3-1",
                        "architecture": "amd64",
                    }
                ],
                "allowed_system_sonames": [
                    {
                        "soname": "libfixture.so.1",
                        "package": "libfixture1:amd64",
                        "version": "1.2.3-1",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    pcl = tmp_path / "validation-prefix" / "bin" / "pcl_pcd2ply"
    pcl.parent.mkdir(parents=True)
    pcl.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    pcl.chmod(0o755)
    dependency_prefix = tmp_path / "dependency-prefix"
    private_library = dependency_prefix / "lib" / "libpcl_private.so.1"
    private_library.parent.mkdir(parents=True)
    private_library.write_bytes(b"fixture private dso")
    ldd = tmp_path / "ldd"
    ldd.write_text(
        "#!/bin/sh\n"
        f"printf '%s\\n' 'libpcl_private.so.1 => {private_library} (0x1)'\n"
        "printf '%s\\n' 'libfixture.so.1 => /lib/x86_64-linux-gnu/libfixture.so.1 (0x2)'\n",
        encoding="utf-8",
    )
    ldd.chmod(0o755)
    dpkg_query = tmp_path / "dpkg-query"
    dpkg_query.write_text(
        "#!/bin/sh\nprintf 'installed\\t1.2.3-1\\n'\n",
        encoding="utf-8",
    )
    dpkg_query.chmod(0o755)

    verifier = _load_verifier()
    verify_pcl = getattr(verifier, "verify_pcl_system_dependencies", None)
    assert callable(verify_pcl), (
        "dependency verifier needs a PCL system dependency verifier"
    )
    assert verify_pcl(
        pcl,
        lock,
        dependency_prefix=dependency_prefix,
        ldd=ldd,
        dpkg_query=dpkg_query,
    ) == {
        "libfixture.so.1": {
            "package": "libfixture1:amd64",
            "version": "1.2.3-1",
        }
    }
