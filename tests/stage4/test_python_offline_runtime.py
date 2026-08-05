# 阶段四 Python 离线运行时合同：先固定人工环境声明，再冻结可复核的锁与缓存。
from __future__ import annotations

import base64
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import os
import tomli
import zipfile

import yaml


ROOT = Path(__file__).resolve().parents[2]
RUNTIME_SPEC = ROOT / "packaging" / "python-environment.yml"
TOOLCHAIN_SPEC = ROOT / "packaging" / "python-toolchain-environment.yml"
VIRTUAL_PACKAGES = ROOT / "packaging" / "locks" / "virtual-packages.yml"
LOCK_CACHE_VERIFIER = ROOT / "scripts" / "verify_python_lock_cache.py"
LOCK_CACHE_FREEZER = ROOT / "scripts" / "freeze_python_lock_cache.py"
WHEEL_CACHE_VERIFIER = ROOT / "scripts" / "verify_python_wheel_cache.py"
WHEEL_CACHE_FREEZER = ROOT / "scripts" / "freeze_python_wheel_cache.py"
PRIVATE_PROTOBUF_BUILDER = ROOT / "scripts" / "build_private_protobuf_conda.py"
PRIVATE_PROTOBUF_RECIPE = ROOT / "packaging" / "recipes" / "protobuf-python" / "meta.yaml"
PRIVATE_PROTOBUF_VARIANTS = PRIVATE_PROTOBUF_RECIPE.parent / "conda_build_config.yaml"
PYTHON_CACHE_MATERIALIZER = ROOT / "scripts" / "materialize_python_package_cache.py"
PYTHON_WHEEL_MATERIALIZER = ROOT / "scripts" / "materialize_python_wheel_cache.py"
RUNTIME_ECAL_INSTALL_VERIFIER = ROOT / "scripts" / "verify_python_runtime_ecal_install.py"
RUNTIME_ECAL_WHEEL = (
    ROOT
    / "build"
    / "stage4-python-wheel-cache-20260805T172013+0800"
    / "wheels"
    / "57a23af7d83c077c04f01852db13f8cda7686a052d41659fafcbe6b3dbe9f6bc"
    / "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
)
ABSEIL_SOURCE_ARCHIVE = (
    ROOT / "build" / "stage4-source-archive-input" / "76bb24329e8bf5f39704eb10d21b9a80befa7c81.tar.gz"
)


def _dependencies(document: object) -> tuple[object, ...]:
    """读取环境声明的依赖列表，避免测试静默接受缺失或错误类型。"""
    assert isinstance(document, dict)
    dependencies = document.get("dependencies")
    assert isinstance(dependencies, list) and dependencies
    return tuple(dependencies)


def _wheel_record(record_path: str, members: dict[str, bytes]) -> str:
    """生成符合 wheel 规范的 RECORD，RECORD 自身始终不含摘要。"""
    rows = []
    for path, payload in members.items():
        digest = base64.urlsafe_b64encode(hashlib.sha256(payload).digest())
        encoded_digest = digest.rstrip(b"=").decode("ascii")
        rows.append(f"{path},sha256={encoded_digest},{len(payload)}")
    rows.append(f"{record_path},,")
    return "\n".join(rows) + "\n"


def test_project_wheel_configuration_discovers_only_slope_sim_packages() -> None:
    """独立 source-build 必须只打包项目模块，不能把仓库资源目录误认为 Python 包。"""
    document = tomli.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    assert document["build-system"] == {
        "build-backend": "setuptools.build_meta",
        "requires": ["setuptools>=68", "wheel"],
    }
    assert document["tool"]["setuptools"]["packages"]["find"] == {
        "include": ["slope_sim*"]
    }


def test_runtime_ecal_install_verifier_accepts_frozen_wheel_tree(tmp_path) -> None:
    """pip staging 的 eCAL 文件、license 与 ELF 必须仍完整匹配冻结 wheel manifest。"""
    assert RUNTIME_ECAL_INSTALL_VERIFIER.is_file(), "stage 4 installed eCAL verifier is not implemented"
    assert RUNTIME_ECAL_WHEEL.is_file(), "frozen eCAL wheel fixture is unavailable"
    runtime = tmp_path / "runtime"
    site_packages = runtime / "lib" / "python3.10" / "site-packages"
    site_packages.mkdir(parents=True)
    with zipfile.ZipFile(RUNTIME_ECAL_WHEEL) as archive:
        archive.extractall(site_packages)

    completed = subprocess.run(
        [
            sys.executable,
            str(RUNTIME_ECAL_INSTALL_VERIFIER),
            "--runtime-root",
            str(runtime),
            "--manifest",
            str(ROOT / "packaging" / "locks" / "python-wheel-cache.manifest.json"),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stdout
    assert completed.stdout == "PASS: installed eCAL runtime matches frozen wheel contract\n"


def test_python_package_materializer_creates_sorted_flat_native_cache(tmp_path) -> None:
    """builder 必须将 canonical nested archive 精确复制为本轮原生 flat cache。"""
    assert PYTHON_CACHE_MATERIALIZER.is_file(), "stage 4 Python cache materializer is not implemented"
    canonical = tmp_path / "canonical"
    destination = tmp_path / "native" / "pkgs"
    evidence = tmp_path / "materialization.json"
    first = b"archive-one"
    second = b"archive-two"
    records = []
    for filename, payload, url in (
        ("first.conda", first, "https://packages.example.invalid/main/linux-64/first.conda"),
        ("second.tar.bz2", second, "https://packages.example.invalid/main/noarch/second.tar.bz2"),
    ):
        sha256 = hashlib.sha256(payload).hexdigest()
        relative_path = f"pkgs/https/packages.example.invalid/main/linux-64/{filename}"
        artifact = canonical / relative_path
        artifact.parent.mkdir(parents=True, exist_ok=True)
        artifact.write_bytes(payload)
        records.append(
            {
                "filename": filename,
                "locks": ["runtime"],
                "md5": hashlib.md5(payload).hexdigest(),
                "relative_path": relative_path,
                "sha256": sha256,
                "size": len(payload),
                "url": url,
            }
        )
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps({"schema_version": 1, "archives": records}), encoding="utf-8"
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(PYTHON_CACHE_MATERIALIZER),
            "--manifest",
            str(manifest),
            "--canonical-cache",
            str(canonical),
            "--destination",
            str(destination),
            "--evidence",
            str(evidence),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert (destination / "first.conda").read_bytes() == first
    assert (destination / "second.tar.bz2").read_bytes() == second
    assert (destination / "urls.txt").read_text(encoding="utf-8") == (
        "https://packages.example.invalid/main/linux-64/first.conda\n"
        "https://packages.example.invalid/main/noarch/second.tar.bz2\n"
    )
    document = json.loads(evidence.read_text(encoding="utf-8"))
    assert document["archives"] == ["first.conda", "second.tar.bz2"]


def test_python_package_materializer_rejects_nonobject_manifest_before_output(tmp_path) -> None:
    """非对象 manifest 必须 fail closed，且不能创建任何 native cache 输出。"""
    manifest = tmp_path / "manifest.json"
    manifest.write_text("[]\n", encoding="utf-8")
    destination = tmp_path / "native" / "pkgs"
    evidence = tmp_path / "evidence.json"
    completed = subprocess.run(
        [
            sys.executable,
            str(PYTHON_CACHE_MATERIALIZER),
            "--manifest",
            str(manifest),
            "--canonical-cache",
            str(tmp_path),
            "--destination",
            str(destination),
            "--evidence",
            str(evidence),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert completed.stdout == "FAIL: package cache manifest is invalid\n"
    assert not destination.exists()
    assert not evidence.exists()


def test_python_wheel_materializer_exclusive_copies_canonical_artifact(tmp_path) -> None:
    """eCAL wheel 必须复制到本轮私有路径，不能直接复用 canonical cache 文件。"""
    assert PYTHON_WHEEL_MATERIALIZER.is_file(), "stage 4 Python wheel materializer is not implemented"
    payload = b"canonical ecal wheel"
    sha256 = hashlib.sha256(payload).hexdigest()
    filename = "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    canonical = tmp_path / "canonical"
    source = canonical / "wheels" / sha256 / filename
    source.parent.mkdir(parents=True)
    source.write_bytes(payload)
    manifest = tmp_path / "manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "wheel": {
                    "filename": filename,
                    "relative_path": f"wheels/{sha256}/{filename}",
                    "sha256": sha256,
                    "size": len(payload),
                },
            }
        ),
        encoding="utf-8",
    )
    destination = tmp_path / "work" / "wheel-cache"
    evidence = tmp_path / "evidence.json"
    completed = subprocess.run(
        [
            sys.executable,
            str(PYTHON_WHEEL_MATERIALIZER),
            "--manifest",
            str(manifest),
            "--canonical-cache",
            str(canonical),
            "--destination",
            str(destination),
            "--evidence",
            str(evidence),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    copied = destination / filename
    assert completed.returncode == 0, completed.stderr
    assert copied.read_bytes() == payload
    assert copied.stat().st_nlink == 1
    assert copied.resolve() != source.resolve()
    assert json.loads(evidence.read_text(encoding="utf-8"))["sha256"] == sha256


def test_python_wheel_materializer_rejects_nonobject_manifest_before_output(tmp_path) -> None:
    """wheel manifest 类型错误必须在创建私有副本之前标准化失败。"""
    manifest = tmp_path / "manifest.json"
    manifest.write_text("[]\n", encoding="utf-8")
    destination = tmp_path / "private-wheel"
    evidence = tmp_path / "evidence.json"
    completed = subprocess.run(
        [
            sys.executable,
            str(PYTHON_WHEEL_MATERIALIZER),
            "--manifest",
            str(manifest),
            "--canonical-cache",
            str(tmp_path),
            "--destination",
            str(destination),
            "--evidence",
            str(evidence),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert completed.stdout == "FAIL: wheel cache manifest is invalid\n"
    assert not destination.exists()
    assert not evidence.exists()


def test_python_environment_specs_separate_runtime_from_toolchain_without_pip() -> None:
    """生产 runtime 与构建 toolchain 必须是独立的纯 Conda 人工声明。"""
    assert RUNTIME_SPEC.is_file(), "stage 4 runtime environment spec is not implemented"
    assert TOOLCHAIN_SPEC.is_file(), "stage 4 toolchain environment spec is not implemented"
    assert VIRTUAL_PACKAGES.is_file(), "stage 4 virtual package spec is not implemented"

    runtime = yaml.safe_load(RUNTIME_SPEC.read_text(encoding="utf-8"))
    toolchain = yaml.safe_load(TOOLCHAIN_SPEC.read_text(encoding="utf-8"))
    virtual_packages = yaml.safe_load(VIRTUAL_PACKAGES.read_text(encoding="utf-8"))
    runtime_dependencies = _dependencies(runtime)
    toolchain_dependencies = _dependencies(toolchain)

    assert runtime.get("name") != toolchain.get("name")
    assert runtime.get("channels", [None])[0] == "https://candace.tail39defd.ts.net:8443"
    assert any(dependency == "python=3.10" for dependency in runtime_dependencies)
    assert any(dependency == "protobuf=6.33.6" for dependency in runtime_dependencies)
    assert any(dependency == "packaging" for dependency in runtime_dependencies)
    assert any(dependency == "python-build" for dependency in toolchain_dependencies)
    assert any(dependency == "conda-build" for dependency in toolchain_dependencies)
    assert any(dependency == "pip" for dependency in toolchain_dependencies)
    assert any(dependency == "conda-pack=0.9.2" for dependency in toolchain_dependencies)
    assert all(not isinstance(dependency, dict) for dependency in runtime_dependencies)
    assert all(not isinstance(dependency, dict) for dependency in toolchain_dependencies)
    assert isinstance(virtual_packages, dict) and virtual_packages


def test_python_lock_verifier_rejects_runtime_pip_dependencies(tmp_path) -> None:
    """静态 verifier 必须在锁解析前拒绝 runtime 的 pip 求解分支。"""
    assert LOCK_CACHE_VERIFIER.is_file(), "stage 4 python lock cache verifier is not implemented"
    runtime = tmp_path / "runtime.yml"
    toolchain = tmp_path / "toolchain.yml"
    virtual_packages = tmp_path / "virtual-packages.yml"
    runtime.write_text(
        """name: runtime\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - protobuf=6.33.6\n  - packaging\n""",
        encoding="utf-8",
    )
    toolchain.write_text(
        """name: toolchain\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - python-build\n  - conda-build\n  - pip\n  - conda-lock=4.0.2\n  - conda-pack=0.9.2\n""",
        encoding="utf-8",
    )
    virtual_packages.write_text(
        """subdirs:\n  linux-64:\n    packages:\n      __archspec: 1=x86_64\n      __glibc: \"2.28\"\n      __linux: \"5.15\"\n      __unix: \"0\"\n""",
        encoding="utf-8",
    )
    command = [
        sys.executable,
        str(LOCK_CACHE_VERIFIER),
        "--runtime-spec",
        str(runtime),
        "--toolchain-spec",
        str(toolchain),
        "--virtual-packages",
        str(virtual_packages),
        "--static-only",
    ]
    accepted = subprocess.run(command, check=False, capture_output=True, text=True)
    assert accepted.returncode == 0, accepted.stderr

    runtime.write_text(
        """name: runtime\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - protobuf=6.33.6\n  - packaging\n  - pip:\n      - eclipse-ecal==6.1.1\n""",
        encoding="utf-8",
    )
    rejected = subprocess.run(command, check=False, capture_output=True, text=True)
    assert rejected.returncode != 0
    assert "runtime dependencies must not contain pip entries" in rejected.stdout


def test_python_lock_verifier_requires_pinned_conda_lock_in_toolchain(tmp_path) -> None:
    """锁生成器必须来自已声明的固定 conda-lock，而不是调用机 PATH。"""
    runtime = tmp_path / "runtime.yml"
    toolchain = tmp_path / "toolchain.yml"
    virtual_packages = tmp_path / "virtual-packages.yml"
    runtime.write_text(
        """name: runtime\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - protobuf=6.33.6\n  - packaging\n""",
        encoding="utf-8",
    )
    toolchain.write_text(
        """name: toolchain\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - python-build\n  - pip\n  - conda-pack=0.9.2\n""",
        encoding="utf-8",
    )
    virtual_packages.write_text(
        """subdirs:\n  linux-64:\n    packages:\n      __archspec: 1=x86_64\n      __glibc: \"2.28\"\n      __linux: \"5.15\"\n      __unix: \"0\"\n""",
        encoding="utf-8",
    )
    rejected = subprocess.run(
        [
            sys.executable,
            str(LOCK_CACHE_VERIFIER),
            "--runtime-spec",
            str(runtime),
            "--toolchain-spec",
            str(toolchain),
            "--virtual-packages",
            str(virtual_packages),
            "--static-only",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert rejected.returncode != 0
    assert "toolchain dependencies are missing required Conda packages" in rejected.stdout


def test_python_lock_verifier_requires_conda_build_for_private_protobuf_package(tmp_path) -> None:
    """缺失官方 Conda 制品时，toolchain 必须具备构建冻结私有 Protobuf 包的能力。"""
    runtime = tmp_path / "runtime.yml"
    toolchain = tmp_path / "toolchain.yml"
    virtual_packages = tmp_path / "virtual-packages.yml"
    runtime.write_text(
        """name: runtime\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - protobuf=6.33.6\n  - packaging\n""",
        encoding="utf-8",
    )
    toolchain.write_text(
        """name: toolchain\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - python-build\n  - pip\n  - conda-lock=4.0.2\n  - conda-pack=0.9.2\n""",
        encoding="utf-8",
    )
    virtual_packages.write_text(
        """subdirs:\n  linux-64:\n    packages:\n      __archspec: 1=x86_64\n      __glibc: \"2.28\"\n      __linux: \"5.15\"\n      __unix: \"0\"\n""",
        encoding="utf-8",
    )
    rejected = subprocess.run(
        [sys.executable, str(LOCK_CACHE_VERIFIER), "--runtime-spec", str(runtime),
         "--toolchain-spec", str(toolchain), "--virtual-packages", str(virtual_packages),
         "--static-only"],
        check=False, capture_output=True, text=True,
    )
    assert rejected.returncode != 0
    assert "toolchain dependencies are missing required Conda packages" in rejected.stdout


def test_python_lock_verifier_crosschecks_explicit_urls_with_unified_records(tmp_path) -> None:
    """unified lock 与 explicit render 必须逐包锁定相同 HTTPS URL 和 MD5。"""
    runtime = tmp_path / "runtime.yml"
    toolchain = tmp_path / "toolchain.yml"
    virtual_packages = tmp_path / "virtual-packages.yml"
    runtime_unified = tmp_path / "runtime.conda-lock.yml"
    runtime_explicit = tmp_path / "runtime-linux-64.lock"
    toolchain_unified = tmp_path / "toolchain.conda-lock.yml"
    toolchain_explicit = tmp_path / "toolchain-linux-64.lock"
    runtime.write_text(
        """name: runtime\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - protobuf=6.33.6\n  - packaging\n""",
        encoding="utf-8",
    )
    toolchain.write_text(
        """name: toolchain\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - python-build\n  - conda-build\n  - pip\n  - conda-lock=4.0.2\n  - conda-pack=0.9.2\n""",
        encoding="utf-8",
    )
    virtual_packages.write_text(
        """subdirs:\n  linux-64:\n    packages:\n      __archspec: 1=x86_64\n      __glibc: \"2.28\"\n      __linux: \"5.15\"\n      __unix: \"0\"\n""",
        encoding="utf-8",
    )
    url = "https://conda.anaconda.org/conda-forge/linux-64/example-1.0-0.conda"
    md5 = "0123456789abcdef0123456789abcdef"
    sha256 = "a" * 64
    unified = (
        "metadata:\n  platforms: [linux-64]\npackage:\n"
        "  - manager: conda\n    name: example\n    platform: linux-64\n"
        f"    url: {url}\n    hash:\n      md5: {md5}\n      sha256: {sha256}\n"
    )
    explicit = f"@EXPLICIT\n{url}#{md5}\n"
    for lock in (runtime_unified, toolchain_unified):
        lock.write_text(unified, encoding="utf-8")
    for lock in (runtime_explicit, toolchain_explicit):
        lock.write_text(explicit, encoding="utf-8")
    command = [
        sys.executable,
        str(LOCK_CACHE_VERIFIER),
        "--runtime-spec",
        str(runtime),
        "--toolchain-spec",
        str(toolchain),
        "--virtual-packages",
        str(virtual_packages),
        "--runtime-unified",
        str(runtime_unified),
        "--runtime-explicit",
        str(runtime_explicit),
        "--toolchain-unified",
        str(toolchain_unified),
        "--toolchain-explicit",
        str(toolchain_explicit),
    ]

    accepted = subprocess.run(command, check=False, capture_output=True, text=True)
    assert accepted.returncode == 0, accepted.stdout

    runtime_explicit.write_text(
        "@EXPLICIT\nhttps://example.invalid/example-1.0-0.conda#0123456789abcdef0123456789abcdef\n",
        encoding="utf-8",
    )
    url_drift = subprocess.run(command, check=False, capture_output=True, text=True)
    assert url_drift.returncode != 0
    assert "explicit lock URL differs from unified record" in url_drift.stdout

    runtime_explicit.write_text(
        f"@EXPLICIT\n{url}#fedcba9876543210fedcba9876543210\n",
        encoding="utf-8",
    )
    md5_drift = subprocess.run(command, check=False, capture_output=True, text=True)
    assert md5_drift.returncode != 0
    assert "explicit lock MD5 differs from unified record" in md5_drift.stdout


def test_python_lock_verifier_crosschecks_canonical_cache_archives(tmp_path) -> None:
    """canonical package cache 的清单和 bytes 必须逐项对应两组 unified Conda 锁。"""
    runtime = tmp_path / "runtime.yml"
    toolchain = tmp_path / "toolchain.yml"
    virtual_packages = tmp_path / "virtual-packages.yml"
    runtime_unified = tmp_path / "runtime.conda-lock.yml"
    runtime_explicit = tmp_path / "runtime-linux-64.lock"
    toolchain_unified = tmp_path / "toolchain.conda-lock.yml"
    toolchain_explicit = tmp_path / "toolchain-linux-64.lock"
    cache_root = tmp_path / "cache"
    cache_manifest = tmp_path / "python-package-cache.manifest.json"
    runtime.write_text(
        """name: runtime\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - protobuf=6.33.6\n  - packaging\n""",
        encoding="utf-8",
    )
    toolchain.write_text(
        """name: toolchain\nchannels: [conda-forge]\ndependencies:\n  - python=3.10\n  - python-build\n  - conda-build\n  - pip\n  - conda-lock=4.0.2\n  - conda-pack=0.9.2\n""",
        encoding="utf-8",
    )
    virtual_packages.write_text(
        """subdirs:\n  linux-64:\n    packages:\n      __archspec: 1=x86_64\n      __glibc: \"2.28\"\n      __linux: \"5.15\"\n      __unix: \"0\"\n""",
        encoding="utf-8",
    )

    def write_lock_pair(unified_path: Path, explicit_path: Path, package: str) -> tuple[str, bytes]:
        payload = f"{package} archive bytes".encode("ascii")
        url = f"https://conda.anaconda.org/conda-forge/linux-64/{package}.conda"
        md5 = hashlib.md5(payload).hexdigest()
        sha256 = hashlib.sha256(payload).hexdigest()
        unified_path.write_text(
            "metadata:\n  platforms: [linux-64]\npackage:\n"
            f"  - manager: conda\n    name: {package}\n    platform: linux-64\n"
            f"    url: {url}\n    hash:\n      md5: {md5}\n      sha256: {sha256}\n",
            encoding="utf-8",
        )
        explicit_path.write_text(f"@EXPLICIT\n{url}#{md5}\n", encoding="utf-8")
        return url, payload

    runtime_url, runtime_payload = write_lock_pair(
        runtime_unified, runtime_explicit, "runtime-example-1.0-0"
    )
    toolchain_url, toolchain_payload = write_lock_pair(
        toolchain_unified, toolchain_explicit, "toolchain-example-1.0-0"
    )
    archives = []
    for lock_name, url, payload in (
        ("runtime", runtime_url, runtime_payload),
        ("toolchain", toolchain_url, toolchain_payload),
    ):
        filename = url.rsplit("/", maxsplit=1)[1]
        relative_path = f"pkgs/https/conda.anaconda.org/conda-forge/linux-64/{filename}"
        archive = cache_root / relative_path
        archive.parent.mkdir(parents=True, exist_ok=True)
        archive.write_bytes(payload)
        archives.append(
            {
                "url": url,
                "filename": filename,
                "relative_path": relative_path,
                "size": len(payload),
                "md5": hashlib.md5(payload).hexdigest(),
                "sha256": hashlib.sha256(payload).hexdigest(),
                "locks": [lock_name],
            }
        )
    urls_txt = cache_root / "pkgs" / "urls.txt"
    urls_txt.parent.mkdir(parents=True, exist_ok=True)
    urls_txt.write_text("\n".join(sorted((runtime_url, toolchain_url))) + "\n", encoding="utf-8")
    cache_manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "archives": archives,
                "urls_txt_sha256": hashlib.sha256(urls_txt.read_bytes()).hexdigest(),
            }
        ),
        encoding="utf-8",
    )
    command = [
        sys.executable,
        str(LOCK_CACHE_VERIFIER),
        "--runtime-spec",
        str(runtime),
        "--toolchain-spec",
        str(toolchain),
        "--virtual-packages",
        str(virtual_packages),
        "--runtime-unified",
        str(runtime_unified),
        "--runtime-explicit",
        str(runtime_explicit),
        "--toolchain-unified",
        str(toolchain_unified),
        "--toolchain-explicit",
        str(toolchain_explicit),
        "--cache-manifest",
        str(cache_manifest),
        "--cache-root",
        str(cache_root),
    ]

    accepted = subprocess.run(command, check=False, capture_output=True, text=True)
    assert accepted.returncode == 0, accepted.stdout

    extra_file = cache_root / "pkgs" / "repodata.json"
    extra_file.write_text("not an allowed package cache member", encoding="utf-8")
    unexpected = subprocess.run(command, check=False, capture_output=True, text=True)
    assert unexpected.returncode != 0
    assert "package cache contains files outside the manifest" in unexpected.stdout
    extra_file.unlink()

    tampered = cache_root / "pkgs/https/conda.anaconda.org/conda-forge/linux-64/runtime-example-1.0-0.conda"
    tampered.write_bytes(b"x" * len(runtime_payload))
    rejected = subprocess.run(command, check=False, capture_output=True, text=True)
    assert rejected.returncode != 0
    assert "cache archive MD5 differs from manifest" in rejected.stdout


def test_python_lock_freezer_rejects_unpinned_micromamba_before_producer_work(tmp_path) -> None:
    """联网 producer 必须在任何 lock 或下载动作前校验 micromamba 二进制摘要。"""
    assert LOCK_CACHE_FREEZER.is_file(), "stage 4 Python lock freezer is not implemented"
    unpinned = tmp_path / "micromamba"
    unpinned.write_bytes(b"not the pinned micromamba binary")
    result = subprocess.run(
        [sys.executable, str(LOCK_CACHE_FREEZER), "--micromamba", str(unpinned), "--check-only"],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "micromamba sha256 differs from pinned toolchain" in result.stdout


def test_python_lock_freezer_runs_pinned_lock_and_render_commands(tmp_path) -> None:
    """冻结器必须显式锁定并渲染 runtime/toolchain，不能回退到调用机工具。"""
    pinned_micromamba = (
        ROOT
        / "build"
        / "stage4-python-producers"
        / "micromamba.JFOCPK"
        / "extracted"
        / "bin"
        / "micromamba"
    )
    assert pinned_micromamba.is_file(), "pinned micromamba fixture is unavailable"
    lock_env = tmp_path / "lock-env"
    lock_bin = lock_env / "bin"
    lock_bin.mkdir(parents=True)
    record = tmp_path / "conda-lock-record.txt"
    conda_lock = lock_bin / "conda-lock"
    conda_lock.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$*\" >> \"$STAGE4_TEST_LOCK_RECORD\"\n"
        "previous=\n"
        "for argument in \"$@\"; do\n"
        "  if [ \"$previous\" = --lockfile ]; then : > \"$argument\"; fi\n"
        "  if [ \"$previous\" = --filename-template ]; then\n"
        "    output=$(printf '%s' \"$argument\" | sed 's/{platform}/linux-64/g')\n"
        "    : > \"$output\"\n"
        "  fi\n"
        "  previous=\"$argument\"\n"
        "done\n",
        encoding="utf-8",
    )
    conda_lock.chmod(0o700)
    runtime_unified = tmp_path / "runtime.conda-lock.yml"
    runtime_explicit = tmp_path / "runtime-linux-64.lock"
    toolchain_unified = tmp_path / "toolchain.conda-lock.yml"
    toolchain_explicit = tmp_path / "toolchain-linux-64.lock"

    result = subprocess.run(
        [
            sys.executable,
            str(LOCK_CACHE_FREEZER),
            "--micromamba",
            str(pinned_micromamba),
            "--lock-env",
            str(lock_env),
            "--runtime-spec",
            str(RUNTIME_SPEC),
            "--toolchain-spec",
            str(TOOLCHAIN_SPEC),
            "--virtual-packages",
            str(VIRTUAL_PACKAGES),
            "--runtime-unified",
            str(runtime_unified),
            "--runtime-explicit",
            str(runtime_explicit),
            "--toolchain-unified",
            str(toolchain_unified),
            "--toolchain-explicit",
            str(toolchain_explicit),
        ],
        check=False,
        capture_output=True,
        env={**os.environ, "STAGE4_TEST_LOCK_RECORD": str(record)},
        text=True,
    )

    assert result.returncode == 0, result.stdout
    assert record.read_text(encoding="utf-8").splitlines() == [
        f"lock --conda {pinned_micromamba} --no-mamba --no-micromamba --file {RUNTIME_SPEC} --platform linux-64 --kind lock --lockfile {runtime_unified} --virtual-package-spec {VIRTUAL_PACKAGES} --no-dev-dependencies",
        f"render --kind explicit --platform linux-64 --no-dev-dependencies --filename-template {tmp_path}/runtime-{{platform}}.lock {runtime_unified}",
        f"lock --conda {pinned_micromamba} --no-mamba --no-micromamba --file {TOOLCHAIN_SPEC} --platform linux-64 --kind lock --lockfile {toolchain_unified} --virtual-package-spec {VIRTUAL_PACKAGES} --no-dev-dependencies",
        f"render --kind explicit --platform linux-64 --no-dev-dependencies --filename-template {tmp_path}/toolchain-{{platform}}.lock {toolchain_unified}",
    ]
    assert runtime_unified.is_file() and runtime_explicit.is_file()
    assert toolchain_unified.is_file() and toolchain_explicit.is_file()


def test_python_lock_freezer_constructs_isolated_download_commands(tmp_path) -> None:
    """下载锁包时必须固定本轮 CA、seed root 与两个独立 prefix。"""
    scripts_dir = str(LOCK_CACHE_FREEZER.parent)
    sys.path.insert(0, scripts_dir)
    try:
        module_spec = importlib.util.spec_from_file_location(
            "stage4_freeze_python_lock_cache", LOCK_CACHE_FREEZER
        )
        assert module_spec is not None and module_spec.loader is not None
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
    finally:
        sys.path.remove(scripts_dir)

    micromamba = tmp_path / "micromamba"
    ca_bundle = tmp_path / "private-channel-ca.pem"
    seed = tmp_path / "seed"
    runtime_explicit = tmp_path / "runtime-linux-64.lock"
    toolchain_explicit = tmp_path / "toolchain-linux-64.lock"

    commands = module._download_commands(
        micromamba=micromamba,
        ca_bundle=ca_bundle,
        seed=seed,
        runtime_explicit=runtime_explicit,
        toolchain_explicit=toolchain_explicit,
    )

    assert commands == (
        (
            str(micromamba),
            "create",
            "--no-rc",
            "--no-env",
            "--ssl-verify",
            str(ca_bundle),
            "--root-prefix",
            str(seed / "mamba-root"),
            "--prefix",
            str(seed / "runtime-download-prefix"),
            "--file",
            str(runtime_explicit),
            "--download-only",
            "--safety-checks",
            "enabled",
            "--yes",
        ),
        (
            str(micromamba),
            "create",
            "--no-rc",
            "--no-env",
            "--ssl-verify",
            str(ca_bundle),
            "--root-prefix",
            str(seed / "mamba-root"),
            "--prefix",
            str(seed / "toolchain-download-prefix"),
            "--file",
            str(toolchain_explicit),
            "--download-only",
            "--safety-checks",
            "enabled",
            "--yes",
        ),
    )


def test_python_lock_freezer_writes_canonical_cache_from_verified_seed_archives(tmp_path) -> None:
    """producer 必须从本轮根级下载归档重建唯一的 nested canonical cache。"""
    scripts_dir = str(LOCK_CACHE_FREEZER.parent)
    sys.path.insert(0, scripts_dir)
    try:
        module_spec = importlib.util.spec_from_file_location(
            "stage4_freeze_python_lock_cache", LOCK_CACHE_FREEZER
        )
        assert module_spec is not None and module_spec.loader is not None
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
    finally:
        sys.path.remove(scripts_dir)

    seed_packages = tmp_path / "seed" / "mamba-root" / "pkgs"
    seed_packages.mkdir(parents=True)
    runtime_payload = b"runtime archive bytes"
    toolchain_payload = b"toolchain archive bytes"
    runtime_url = "https://candace.tail39defd.ts.net:8443/linux-64/runtime-1.0-0.conda"
    toolchain_url = "https://conda.anaconda.org/conda-forge/linux-64/toolchain-1.0-0.conda"
    runtime_seed = (
        seed_packages
        / "https"
        / "candace.tail39defd.ts.net_8443"
        / "linux-64"
        / "runtime-1.0-0.conda"
    )
    toolchain_seed = (
        seed_packages
        / "https"
        / "conda.anaconda.org"
        / "conda-forge"
        / "linux-64"
        / "toolchain-1.0-0.conda"
    )
    runtime_seed.parent.mkdir(parents=True)
    toolchain_seed.parent.mkdir(parents=True)
    runtime_seed.write_bytes(runtime_payload)
    toolchain_seed.write_bytes(toolchain_payload)
    records = {
        runtime_url: (
            hashlib.md5(runtime_payload).hexdigest(),
            hashlib.sha256(runtime_payload).hexdigest(),
            frozenset(("runtime",)),
        ),
        toolchain_url: (
            hashlib.md5(toolchain_payload).hexdigest(),
            hashlib.sha256(toolchain_payload).hexdigest(),
            frozenset(("toolchain",)),
        ),
    }
    cache_root = tmp_path / "canonical-cache"
    cache_manifest = tmp_path / "python-package-cache.manifest.json"

    module._write_canonical_cache(
        seed=tmp_path / "seed",
        cache_root=cache_root,
        manifest_path=cache_manifest,
        records=records,
    )

    runtime_cache = (
        cache_root
        / "pkgs/https/candace.tail39defd.ts.net:8443/linux-64/runtime-1.0-0.conda"
    )
    toolchain_cache = (
        cache_root
        / "pkgs/https/conda.anaconda.org/conda-forge/linux-64/toolchain-1.0-0.conda"
    )
    assert runtime_cache.read_bytes() == runtime_payload
    assert toolchain_cache.read_bytes() == toolchain_payload
    assert (cache_root / "pkgs/urls.txt").read_text(encoding="utf-8") == (
        f"{runtime_url}\n{toolchain_url}\n"
    )
    manifest = json.loads(cache_manifest.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == 1
    assert manifest["archives"] == [
        {
            "filename": "runtime-1.0-0.conda",
            "locks": ["runtime"],
            "md5": hashlib.md5(runtime_payload).hexdigest(),
            "relative_path": "pkgs/https/candace.tail39defd.ts.net:8443/linux-64/runtime-1.0-0.conda",
            "sha256": hashlib.sha256(runtime_payload).hexdigest(),
            "size": len(runtime_payload),
            "url": runtime_url,
        },
        {
            "filename": "toolchain-1.0-0.conda",
            "locks": ["toolchain"],
            "md5": hashlib.md5(toolchain_payload).hexdigest(),
            "relative_path": "pkgs/https/conda.anaconda.org/conda-forge/linux-64/toolchain-1.0-0.conda",
            "sha256": hashlib.sha256(toolchain_payload).hexdigest(),
            "size": len(toolchain_payload),
            "url": toolchain_url,
        },
    ]
    assert manifest["urls_txt_sha256"] == hashlib.sha256(
        (cache_root / "pkgs/urls.txt").read_bytes()
    ).hexdigest()


def test_python_lock_freezer_downloads_into_seed_then_writes_canonical_cache(
    tmp_path, monkeypatch
) -> None:
    """完整 producer 必须先下载两份显式锁，再从同一 seed 生成 canonical cache。"""
    scripts_dir = str(LOCK_CACHE_FREEZER.parent)
    sys.path.insert(0, scripts_dir)
    try:
        module_spec = importlib.util.spec_from_file_location(
            "stage4_freeze_python_lock_cache", LOCK_CACHE_FREEZER
        )
        assert module_spec is not None and module_spec.loader is not None
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
    finally:
        sys.path.remove(scripts_dir)

    pinned_micromamba = (
        ROOT
        / "build"
        / "stage4-python-producers"
        / "micromamba.JFOCPK"
        / "extracted"
        / "bin"
        / "micromamba"
    )
    assert pinned_micromamba.is_file(), "pinned micromamba fixture is unavailable"
    lock_env = tmp_path / "lock-env"
    lock_bin = lock_env / "bin"
    lock_bin.mkdir(parents=True)
    conda_lock = lock_bin / "conda-lock"
    conda_lock.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    conda_lock.chmod(0o700)
    runtime_unified = tmp_path / "runtime.conda-lock.yml"
    runtime_explicit = tmp_path / "runtime-linux-64.lock"
    toolchain_unified = tmp_path / "toolchain.conda-lock.yml"
    toolchain_explicit = tmp_path / "toolchain-linux-64.lock"
    seed = tmp_path / "seed"
    cache_root = tmp_path / "canonical-cache"
    cache_manifest = tmp_path / "python-package-cache.manifest.json"
    ca_bundle = tmp_path / "private-channel-ca.pem"
    ca_bundle.write_text("test CA bundle\n", encoding="utf-8")
    runtime_payload = b"runtime package bytes"
    toolchain_payload = b"toolchain package bytes"
    runtime_url = "https://candace.tail39defd.ts.net:8443/linux-64/runtime-1.0-0.conda"
    toolchain_url = "https://conda.anaconda.org/conda-forge/linux-64/toolchain-1.0-0.conda"
    command_log: list[tuple[str, ...]] = []
    command_environments: list[dict[str, str] | None] = []

    def write_lock_pair(
        unified_path: Path, explicit_path: Path, url: str, payload: bytes
    ) -> None:
        md5 = hashlib.md5(payload).hexdigest()
        sha256 = hashlib.sha256(payload).hexdigest()
        unified_path.write_text(
            "metadata:\n  platforms: [linux-64]\npackage:\n"
            "  - manager: conda\n    name: example\n    platform: linux-64\n"
            f"    url: {url}\n    hash:\n      md5: {md5}\n      sha256: {sha256}\n",
            encoding="utf-8",
        )
        explicit_path.write_text(f"@EXPLICIT\n{url}#{md5}\n", encoding="utf-8")

    def run(command, check, env=None):
        command_tuple = tuple(command)
        command_log.append(command_tuple)
        command_environments.append(env)
        if command_tuple[1] == "lock":
            output = Path(command_tuple[command_tuple.index("--lockfile") + 1])
            if output == runtime_unified:
                write_lock_pair(output, runtime_explicit, runtime_url, runtime_payload)
            else:
                write_lock_pair(output, toolchain_explicit, toolchain_url, toolchain_payload)
        elif command_tuple[1] == "render":
            template = command_tuple[command_tuple.index("--filename-template") + 1]
            output = Path(template.replace("{platform}", "linux-64"))
            if output == runtime_explicit:
                output.write_text(
                    f"@EXPLICIT\n{runtime_url}#{hashlib.md5(runtime_payload).hexdigest()}\n",
                    encoding="utf-8",
                )
            else:
                output.write_text(
                    f"@EXPLICIT\n{toolchain_url}#{hashlib.md5(toolchain_payload).hexdigest()}\n",
                    encoding="utf-8",
                )
        elif command_tuple[1] == "create":
            explicit_lock = Path(command_tuple[command_tuple.index("--file") + 1])
            if explicit_lock == runtime_explicit:
                packages = (
                    seed
                    / "mamba-root"
                    / "pkgs"
                    / "https"
                    / "candace.tail39defd.ts.net_8443"
                    / "linux-64"
                )
                packages.mkdir(parents=True, exist_ok=True)
                (packages / "runtime-1.0-0.conda").write_bytes(runtime_payload)
            else:
                packages = (
                    seed
                    / "mamba-root"
                    / "pkgs"
                    / "https"
                    / "conda.anaconda.org"
                    / "conda-forge"
                    / "linux-64"
                )
                packages.mkdir(parents=True, exist_ok=True)
                (packages / "toolchain-1.0-0.conda").write_bytes(toolchain_payload)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, "run", run)
    result = module.main(
        [
            "--micromamba",
            str(pinned_micromamba),
            "--lock-env",
            str(lock_env),
            "--runtime-spec",
            str(RUNTIME_SPEC),
            "--toolchain-spec",
            str(TOOLCHAIN_SPEC),
            "--virtual-packages",
            str(VIRTUAL_PACKAGES),
            "--runtime-unified",
            str(runtime_unified),
            "--runtime-explicit",
            str(runtime_explicit),
            "--toolchain-unified",
            str(toolchain_unified),
            "--toolchain-explicit",
            str(toolchain_explicit),
            "--seed",
            str(seed),
            "--cache-root",
            str(cache_root),
            "--cache-manifest",
            str(cache_manifest),
            "--ca-bundle",
            str(ca_bundle),
        ]
    )

    assert result == 0
    assert [command[1] for command in command_log] == [
        "lock",
        "render",
        "lock",
        "render",
        "create",
        "create",
    ]
    assert all("--download-only" in command for command in command_log[-2:])
    assert all("--ssl-verify" in command for command in command_log[-2:])
    assert all(
        environment is not None
        and environment["MAMBA_SSL_VERIFY"] == str(ca_bundle)
        for environment in command_environments
    )
    assert json.loads(cache_manifest.read_text(encoding="utf-8"))["archives"]


def test_python_wheel_freezer_writes_verified_canonical_artifact(
    tmp_path, monkeypatch
) -> None:
    """wheel producer 必须固定下载 bytes 并物化唯一 canonical 副本。"""
    assert WHEEL_CACHE_FREEZER.is_file(), "stage 4 Python wheel freezer is not implemented"
    scripts_dir = str(WHEEL_CACHE_FREEZER.parent)
    sys.path.insert(0, scripts_dir)
    try:
        module_spec = importlib.util.spec_from_file_location(
            "stage4_freeze_python_wheel_cache", WHEEL_CACHE_FREEZER
        )
        assert module_spec is not None and module_spec.loader is not None
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
    finally:
        sys.path.remove(scripts_dir)

    filename = "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    dist_info = "eclipse_ecal-6.1.1.dist-info"
    metadata_path = f"{dist_info}/METADATA"
    wheel_path = f"{dist_info}/WHEEL"
    record_path = f"{dist_info}/RECORD"
    license_path = f"{dist_info}/licenses/LICENSE.txt"
    license_contents = b"Apache License\n"
    metadata = (
        b"Metadata-Version: 2.1\nName: eclipse-ecal\nVersion: 6.1.1\n"
        b"License-Expression: Apache-2.0\nLicense-File: LICENSE.txt\n"
    )
    wheel_metadata = b"Wheel-Version: 1.0\nTag: cp310-cp310-manylinux_2_28_x86_64\n"
    wheel_bytes = io.BytesIO()
    with zipfile.ZipFile(wheel_bytes, "w") as archive:
        archive.writestr(metadata_path, metadata)
        archive.writestr(wheel_path, wheel_metadata)
        archive.writestr(license_path, license_contents)
        archive.writestr(
            record_path,
            _wheel_record(
                record_path,
                {
                    metadata_path: metadata,
                    wheel_path: wheel_metadata,
                    license_path: license_contents,
                },
            ),
        )
    payload = wheel_bytes.getvalue()
    sha256 = hashlib.sha256(payload).hexdigest()
    record_contents = _wheel_record(
        record_path,
        {
            metadata_path: metadata,
            wheel_path: wheel_metadata,
            license_path: license_contents,
        },
    ).encode("utf-8")
    member_payloads = {
        metadata_path: metadata,
        wheel_path: wheel_metadata,
        license_path: license_contents,
        record_path: record_contents,
    }
    member_tree = hashlib.sha256()
    for member_path, member_payload in sorted(member_payloads.items()):
        member_tree.update(member_path.encode("utf-8"))
        member_tree.update(b"\0")
        member_tree.update(str(len(member_payload)).encode("ascii"))
        member_tree.update(b"\0")
        member_tree.update(hashlib.sha256(member_payload).hexdigest().encode("ascii"))
        member_tree.update(b"\n")
    pinned = {
        "url": f"https://files.example.invalid/packages/{filename}",
        "filename": filename,
        "size": len(payload),
        "sha256": sha256,
        "distribution": "eclipse-ecal",
        "version": "6.1.1",
        "requires_python": ">=3.8",
        "python_tag": "cp310",
        "abi_tag": "cp310",
        "platform_tag": "manylinux_2_28_x86_64",
    }
    monkeypatch.setattr(module, "PINNED_WHEEL", pinned)

    monkeypatch.setattr(
        module,
        "_load_pypi_release",
        lambda: {
            "info": {
                "name": pinned["distribution"],
                "version": pinned["version"],
                "requires_python": pinned["requires_python"],
            },
            "urls": [
                {
                    "filename": pinned["filename"],
                    "url": pinned["url"],
                    "size": pinned["size"],
                    "digests": {"sha256": pinned["sha256"]},
                    "packagetype": "bdist_wheel",
                    "requires_python": pinned["requires_python"],
                    "yanked": False,
                }
            ],
        },
    )

    def fake_download(url: str, destination: Path) -> None:
        assert url == pinned["url"]
        destination.write_bytes(payload)

    monkeypatch.setattr(module, "_download", fake_download)
    download_root = tmp_path / "download"
    cache_root = tmp_path / "wheel-cache"
    manifest = tmp_path / "python-wheel-cache.manifest.json"

    result = module.main(
        [
            "--download-root",
            str(download_root),
            "--cache-root",
            str(cache_root),
            "--manifest",
            str(manifest),
        ]
    )

    artifact = cache_root / "wheels" / sha256 / filename
    assert result == 0
    assert artifact.read_bytes() == payload
    assert artifact.stat().st_nlink == 1
    tree = hashlib.sha256()
    for member_path, member_payload in sorted(
        {
            metadata_path: metadata,
            wheel_path: wheel_metadata,
            license_path: license_contents,
            record_path: _wheel_record(
                record_path,
                {
                    metadata_path: metadata,
                    wheel_path: wheel_metadata,
                    license_path: license_contents,
                },
            ).encode("utf-8"),
        }.items()
    ):
        tree.update(member_path.encode("utf-8"))
        tree.update(b"\0")
        tree.update(str(len(member_payload)).encode("ascii"))
        tree.update(b"\0")
        tree.update(hashlib.sha256(member_payload).hexdigest().encode("ascii"))
        tree.update(b"\n")
    assert json.loads(manifest.read_text(encoding="utf-8"))["wheel"] == {
        **pinned,
        "relative_path": f"wheels/{sha256}/{filename}",
        "dist_info_paths": {
            "metadata": metadata_path,
            "wheel": wheel_path,
            "record": record_path,
        },
        "dist_info_digests": {
            "metadata": hashlib.sha256(metadata).hexdigest(),
            "wheel": hashlib.sha256(wheel_metadata).hexdigest(),
            "record": hashlib.sha256(
                _wheel_record(
                    record_path,
                    {
                        metadata_path: metadata,
                        wheel_path: wheel_metadata,
                        license_path: license_contents,
                    },
                ).encode("utf-8")
            ).hexdigest(),
        },
        "licenses": {
            "expression": "Apache-2.0",
            "files": {license_path: hashlib.sha256(license_contents).hexdigest()},
        },
        "members": {
            "count": 4,
            "record_members": {
                metadata_path: {"sha256": hashlib.sha256(metadata).hexdigest(), "size": len(metadata)},
                wheel_path: {"sha256": hashlib.sha256(wheel_metadata).hexdigest(), "size": len(wheel_metadata)},
                license_path: {"sha256": hashlib.sha256(license_contents).hexdigest(), "size": len(license_contents)},
            },
            "tree_sha256": tree.hexdigest(),
        },
        "elf": {"forbidden_soname": "libprotobuf.so", "members": []},
    }


def test_python_wheel_freezer_rejects_pypi_release_metadata_drift(monkeypatch) -> None:
    """联网 producer 必须先将 PyPI release JSON 与固定 wheel identity 交叉核验。"""
    scripts_dir = str(WHEEL_CACHE_FREEZER.parent)
    sys.path.insert(0, scripts_dir)
    try:
        module_spec = importlib.util.spec_from_file_location(
            "stage4_freeze_python_wheel_cache", WHEEL_CACHE_FREEZER
        )
        assert module_spec is not None and module_spec.loader is not None
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
    finally:
        sys.path.remove(scripts_dir)

    verify_release = getattr(module, "_verify_pypi_release", None)
    assert callable(verify_release), "wheel freezer does not verify PyPI release JSON"
    document = {
        "info": {
            "name": "eclipse-ecal",
            "version": "6.1.1",
            "requires_python": ">=3.8",
        },
        "urls": [
            {
                "filename": module.PINNED_WHEEL["filename"],
                "url": module.PINNED_WHEEL["url"],
                "size": module.PINNED_WHEEL["size"] + 1,
                "digests": {"sha256": module.PINNED_WHEEL["sha256"]},
                "packagetype": "bdist_wheel",
                "requires_python": module.PINNED_WHEEL["requires_python"],
                "yanked": False,
            }
        ],
    }
    try:
        verify_release(document)
    except ValueError as error:
        assert str(error) == "PyPI release metadata differs from pinned wheel"
    else:
        raise AssertionError("drifted PyPI release metadata was accepted")


def test_python_wheel_manifest_freezes_core_zip_member_digests() -> None:
    """正式 manifest 必须绑定 METADATA、WHEEL 与 RECORD 的内容摘要。"""
    manifest = ROOT / "packaging" / "locks" / "python-wheel-cache.manifest.json"
    wheel = json.loads(manifest.read_text(encoding="utf-8"))["wheel"]
    digests = wheel["dist_info_digests"]
    assert set(digests) == {"metadata", "wheel", "record"}
    assert all(len(value) == 64 for value in digests.values())


def test_python_wheel_manifest_freezes_all_declared_license_files() -> None:
    """正式 manifest 必须绑定 METADATA 所声明的全部 license/NOTICE 内容。"""
    manifest = ROOT / "packaging" / "locks" / "python-wheel-cache.manifest.json"
    wheel = json.loads(manifest.read_text(encoding="utf-8"))["wheel"]
    licenses = wheel["licenses"]
    assert licenses["expression"] == "Apache-2.0"
    files = licenses["files"]
    assert len(files) == 25
    assert all(
        path.startswith("eclipse_ecal-6.1.1.dist-info/licenses/")
        and len(digest) == 64
        for path, digest in files.items()
    )
    assert "eclipse_ecal-6.1.1.dist-info/licenses/NOTICE.md" in files


def test_python_wheel_manifest_freezes_complete_record_member_inventory() -> None:
    """正式 manifest 必须绑定整个 wheel 树与 RECORD 的每个可安装成员。"""
    manifest = ROOT / "packaging" / "locks" / "python-wheel-cache.manifest.json"
    wheel = json.loads(manifest.read_text(encoding="utf-8"))["wheel"]
    members = wheel["members"]
    assert members["count"] == 58
    assert len(members["tree_sha256"]) == 64
    record_members = members["record_members"]
    assert len(record_members) == 57
    assert all(
        set(entry) == {"sha256", "size"}
        and len(entry["sha256"]) == 64
        and isinstance(entry["size"], int)
        and entry["size"] >= 0
        for entry in record_members.values()
    )


def test_python_wheel_manifest_freezes_elf_abi_inventory() -> None:
    """正式 manifest 必须冻结所有 wheel ELF，并禁止 bundled libprotobuf。"""
    manifest = ROOT / "packaging" / "locks" / "python-wheel-cache.manifest.json"
    wheel = json.loads(manifest.read_text(encoding="utf-8"))["wheel"]
    elf = wheel["elf"]
    assert elf["forbidden_soname"] == "libprotobuf.so"
    members = elf["members"]
    assert {member["path"] for member in members} == {
        "ecal/_ecal_core_py.cpython-310-x86_64-linux-gnu.so",
        "ecal/_ecal_hdf5_py.cpython-310-x86_64-linux-gnu.so",
        "ecal/libecal_core.so.6",
        "ecal/libecal_core.so.6.1.1",
        "ecal/libhdf5.so.310",
        "ecal/libhdf5.so.310.3.0",
        "ecal/nanobind_core.cpython-310-x86_64-linux-gnu.so",
    }
    assert all(
        set(member) == {"class", "machine", "needed", "path", "runpath", "sha256", "soname"}
        and member["class"] == "ELF64"
        and member["machine"] == "EM_X86_64"
        and len(member["sha256"]) == 64
        and isinstance(member["needed"], list)
        and (member["runpath"] is None or isinstance(member["runpath"], str))
        and (member["soname"] is None or isinstance(member["soname"], str))
        for member in members
    )


def test_python_wheel_verifier_requires_unique_canonical_wheel_artifact(tmp_path) -> None:
    """wheel cache 只能包含 manifest 精确声明的一份单链接嵌套制品。"""
    assert WHEEL_CACHE_VERIFIER.is_file(), "stage 4 python wheel cache verifier is not implemented"
    filename = "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    dist_info = "eclipse_ecal-6.1.1.dist-info"
    metadata = (
        b"Metadata-Version: 2.1\nName: eclipse-ecal\nVersion: 6.1.1\n"
        b"Requires-Python: >=3.10\nLicense-Expression: Apache-2.0\n"
        b"License-File: LICENSE.txt\n"
    )
    wheel_metadata = (
        b"Wheel-Version: 1.0\nRoot-Is-Purelib: false\n"
        b"Tag: cp310-cp310-manylinux_2_28_x86_64\n"
    )
    wheel_bytes = io.BytesIO()
    with zipfile.ZipFile(wheel_bytes, "w") as archive:
        metadata_path = f"{dist_info}/METADATA"
        wheel_path = f"{dist_info}/WHEEL"
        record_path = f"{dist_info}/RECORD"
        license_path = f"{dist_info}/licenses/LICENSE.txt"
        license_contents = b"Apache License\n"
        archive.writestr(metadata_path, metadata)
        archive.writestr(wheel_path, wheel_metadata)
        archive.writestr(license_path, license_contents)
        archive.writestr(
            record_path,
            _wheel_record(
                record_path,
                {
                    metadata_path: metadata,
                    wheel_path: wheel_metadata,
                    license_path: license_contents,
                },
            ),
        )
    payload = wheel_bytes.getvalue()
    sha256 = hashlib.sha256(payload).hexdigest()
    url = f"https://files.example.invalid/packages/{filename}"
    record_contents = _wheel_record(
        record_path,
        {
            metadata_path: metadata,
            wheel_path: wheel_metadata,
            license_path: license_contents,
        },
    ).encode("utf-8")
    member_payloads = {
        metadata_path: metadata,
        wheel_path: wheel_metadata,
        license_path: license_contents,
        record_path: record_contents,
    }
    member_tree = hashlib.sha256()
    for member_path, member_payload in sorted(member_payloads.items()):
        member_tree.update(member_path.encode("utf-8"))
        member_tree.update(b"\0")
        member_tree.update(str(len(member_payload)).encode("ascii"))
        member_tree.update(b"\0")
        member_tree.update(hashlib.sha256(member_payload).hexdigest().encode("ascii"))
        member_tree.update(b"\n")
    cache_root = tmp_path / "wheel-cache"
    wheel = cache_root / "wheels" / sha256 / filename
    wheel.parent.mkdir(parents=True)
    wheel.write_bytes(payload)
    manifest = tmp_path / "python-wheel-cache.manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "wheel": {
                    "url": url,
                    "filename": filename,
                    "relative_path": f"wheels/{sha256}/{filename}",
                    "size": len(payload),
                    "sha256": sha256,
                    "distribution": "eclipse-ecal",
                    "version": "6.1.1",
                    "requires_python": ">=3.10",
                        "python_tag": "cp310",
                        "abi_tag": "cp310",
                        "platform_tag": "manylinux_2_28_x86_64",
                        "dist_info_paths": {
                            "metadata": metadata_path,
                            "wheel": wheel_path,
                            "record": record_path,
                        },
                        "dist_info_digests": {
                            "metadata": hashlib.sha256(metadata).hexdigest(),
                            "wheel": hashlib.sha256(wheel_metadata).hexdigest(),
                            "record": hashlib.sha256(
                                _wheel_record(
                                    record_path,
                                    {
                                        metadata_path: metadata,
                                        wheel_path: wheel_metadata,
                                        license_path: license_contents,
                                    },
                                ).encode("utf-8")
                            ).hexdigest(),
                        },
                        "licenses": {
                            "expression": "Apache-2.0",
                            "files": {
                                license_path: hashlib.sha256(license_contents).hexdigest()
                            },
                        },
                        "members": {
                            "count": len(member_payloads),
                            "record_members": {
                                member_path: {
                                    "sha256": hashlib.sha256(member_payload).hexdigest(),
                                    "size": len(member_payload),
                                }
                                for member_path, member_payload in member_payloads.items()
                                if member_path != record_path
                            },
                            "tree_sha256": member_tree.hexdigest(),
                        },
                        "elf": {"forbidden_soname": "libprotobuf.so", "members": []},
                    },
                }
        ),
        encoding="utf-8",
    )
    command = [
        sys.executable,
        str(WHEEL_CACHE_VERIFIER),
        "--manifest",
        str(manifest),
        "--cache-root",
        str(cache_root),
    ]

    accepted = subprocess.run(command, check=False, capture_output=True, text=True)
    assert accepted.returncode == 0, accepted.stdout

    (cache_root / "wheels" / "extra.whl").write_bytes(b"not declared")
    rejected = subprocess.run(command, check=False, capture_output=True, text=True)
    assert rejected.returncode != 0
    assert "wheel cache contains files outside the manifest" in rejected.stdout


def test_python_wheel_verifier_requires_complete_metadata_contract(tmp_path) -> None:
    """生产 eCAL wheel 的 manifest 不得省略 distribution、版本或 ABI 合同。"""
    payload = b"synthetic wheel bytes"
    sha256 = hashlib.sha256(payload).hexdigest()
    filename = "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    cache_root = tmp_path / "wheel-cache"
    wheel = cache_root / "wheels" / sha256 / filename
    wheel.parent.mkdir(parents=True)
    wheel.write_bytes(payload)
    manifest = tmp_path / "python-wheel-cache.manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "wheel": {
                    "url": f"https://files.example.invalid/packages/{filename}",
                    "filename": filename,
                    "relative_path": f"wheels/{sha256}/{filename}",
                    "size": len(payload),
                    "sha256": sha256,
                },
            }
        ),
        encoding="utf-8",
    )

    rejected = subprocess.run(
        [
            sys.executable,
            str(WHEEL_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert rejected.returncode != 0
    assert "wheel manifest metadata contract is incomplete" in rejected.stdout


def test_python_wheel_verifier_rejects_nonzip_wheel_with_metadata_contract(tmp_path) -> None:
    """声明 runtime ABI 合同时，artifact 必须可作为结构化 ZIP wheel 打开。"""
    payload = b"not a zip wheel"
    sha256 = hashlib.sha256(payload).hexdigest()
    filename = "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    cache_root = tmp_path / "wheel-cache"
    wheel = cache_root / "wheels" / sha256 / filename
    wheel.parent.mkdir(parents=True)
    wheel.write_bytes(payload)
    manifest = tmp_path / "python-wheel-cache.manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "wheel": {
                    "url": f"https://files.example.invalid/packages/{filename}",
                    "filename": filename,
                    "relative_path": f"wheels/{sha256}/{filename}",
                    "size": len(payload),
                    "sha256": sha256,
                    "distribution": "eclipse-ecal",
                    "version": "6.1.1",
                    "requires_python": ">=3.10",
                    "python_tag": "cp310",
                    "abi_tag": "cp310",
                    "platform_tag": "manylinux_2_28_x86_64",
                },
            }
        ),
        encoding="utf-8",
    )

    rejected = subprocess.run(
        [
            sys.executable,
            str(WHEEL_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert rejected.returncode != 0
    assert "wheel artifact is not a valid ZIP file" in rejected.stdout


def test_python_wheel_verifier_rejects_wheel_tag_drift_from_manifest(tmp_path) -> None:
    """有效 ZIP 仍必须声明 manifest 锁定的 CPython 3.10 wheel tag。"""
    filename = "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    dist_info = "eclipse_ecal-6.1.1.dist-info"
    metadata = (
        b"Metadata-Version: 2.1\nName: eclipse-ecal\nVersion: 6.1.1\n"
        b"Requires-Python: >=3.10\n"
    )
    wheel_metadata = (
        b"Wheel-Version: 1.0\nRoot-Is-Purelib: false\n"
        b"Tag: cp39-cp39-manylinux_2_28_x86_64\n"
    )
    wheel_bytes = io.BytesIO()
    with zipfile.ZipFile(wheel_bytes, "w") as archive:
        metadata_path = f"{dist_info}/METADATA"
        wheel_path = f"{dist_info}/WHEEL"
        record_path = f"{dist_info}/RECORD"
        archive.writestr(metadata_path, metadata)
        archive.writestr(wheel_path, wheel_metadata)
        archive.writestr(
            record_path,
            _wheel_record(record_path, {metadata_path: metadata, wheel_path: wheel_metadata}),
        )
    payload = wheel_bytes.getvalue()
    sha256 = hashlib.sha256(payload).hexdigest()
    cache_root = tmp_path / "wheel-cache"
    wheel = cache_root / "wheels" / sha256 / filename
    wheel.parent.mkdir(parents=True)
    wheel.write_bytes(payload)
    manifest = tmp_path / "python-wheel-cache.manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "wheel": {
                    "url": f"https://files.example.invalid/packages/{filename}",
                    "filename": filename,
                    "relative_path": f"wheels/{sha256}/{filename}",
                    "size": len(payload),
                    "sha256": sha256,
                    "distribution": "eclipse-ecal",
                    "version": "6.1.1",
                    "requires_python": ">=3.10",
                    "python_tag": "cp310",
                    "abi_tag": "cp310",
                    "platform_tag": "manylinux_2_28_x86_64",
                },
            }
        ),
        encoding="utf-8",
    )

    rejected = subprocess.run(
        [
            sys.executable,
            str(WHEEL_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert rejected.returncode != 0
    assert "wheel WHEEL tag differs from manifest" in rejected.stdout


def test_python_wheel_verifier_rejects_unpinned_ecal_release(tmp_path) -> None:
    """内部 cache 也只能承载计划锁定的 eCAL 6.1.1 CPython 3.10 wheel。"""
    filename = "eclipse_ecal-9.0.0-cp310-cp310-manylinux_2_28_x86_64.whl"
    dist_info = "eclipse_ecal-9.0.0.dist-info"
    metadata = (
        b"Metadata-Version: 2.1\nName: eclipse-ecal\nVersion: 9.0.0\n"
        b"Requires-Python: >=3.10\n"
    )
    wheel_metadata = (
        b"Wheel-Version: 1.0\nRoot-Is-Purelib: false\n"
        b"Tag: cp310-cp310-manylinux_2_28_x86_64\n"
    )
    wheel_bytes = io.BytesIO()
    with zipfile.ZipFile(wheel_bytes, "w") as archive:
        metadata_path = f"{dist_info}/METADATA"
        wheel_path = f"{dist_info}/WHEEL"
        record_path = f"{dist_info}/RECORD"
        archive.writestr(metadata_path, metadata)
        archive.writestr(wheel_path, wheel_metadata)
        archive.writestr(
            record_path,
            _wheel_record(record_path, {metadata_path: metadata, wheel_path: wheel_metadata}),
        )
    payload = wheel_bytes.getvalue()
    sha256 = hashlib.sha256(payload).hexdigest()
    cache_root = tmp_path / "wheel-cache"
    wheel = cache_root / "wheels" / sha256 / filename
    wheel.parent.mkdir(parents=True)
    wheel.write_bytes(payload)
    manifest = tmp_path / "python-wheel-cache.manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "wheel": {
                    "url": f"https://files.example.invalid/packages/{filename}",
                    "filename": filename,
                    "relative_path": f"wheels/{sha256}/{filename}",
                    "size": len(payload),
                    "sha256": sha256,
                    "distribution": "eclipse-ecal",
                    "version": "9.0.0",
                    "requires_python": ">=3.10",
                    "python_tag": "cp310",
                    "abi_tag": "cp310",
                    "platform_tag": "manylinux_2_28_x86_64",
                },
            }
        ),
        encoding="utf-8",
    )

    rejected = subprocess.run(
        [
            sys.executable,
            str(WHEEL_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert rejected.returncode != 0
    assert "wheel manifest does not match the pinned eCAL release" in rejected.stdout


def test_python_wheel_verifier_requires_dist_info_record(tmp_path) -> None:
    """安装前 verifier 必须拒绝缺少逐成员完整性入口 RECORD 的 wheel。"""
    filename = "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    wheel_bytes = io.BytesIO()
    with zipfile.ZipFile(wheel_bytes, "w") as archive:
        archive.writestr(
            "eclipse_ecal-6.1.1.dist-info/METADATA",
            "Metadata-Version: 2.1\nName: eclipse-ecal\nVersion: 6.1.1\nRequires-Python: >=3.10\n",
        )
        archive.writestr(
            "eclipse_ecal-6.1.1.dist-info/WHEEL",
            "Wheel-Version: 1.0\nRoot-Is-Purelib: false\nTag: cp310-cp310-manylinux_2_28_x86_64\n",
        )
    payload = wheel_bytes.getvalue()
    sha256 = hashlib.sha256(payload).hexdigest()
    cache_root = tmp_path / "wheel-cache"
    wheel = cache_root / "wheels" / sha256 / filename
    wheel.parent.mkdir(parents=True)
    wheel.write_bytes(payload)
    manifest = tmp_path / "python-wheel-cache.manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "wheel": {
                    "url": f"https://files.example.invalid/packages/{filename}",
                    "filename": filename,
                    "relative_path": f"wheels/{sha256}/{filename}",
                    "size": len(payload),
                    "sha256": sha256,
                    "distribution": "eclipse-ecal",
                    "version": "6.1.1",
                    "requires_python": ">=3.10",
                    "python_tag": "cp310",
                    "abi_tag": "cp310",
                    "platform_tag": "manylinux_2_28_x86_64",
                },
            }
        ),
        encoding="utf-8",
    )

    rejected = subprocess.run(
        [
            sys.executable,
            str(WHEEL_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert rejected.returncode != 0
    assert "wheel dist-info RECORD member is missing" in rejected.stdout


def test_python_wheel_verifier_rejects_zip_member_path_escape(tmp_path) -> None:
    """wheel ZIP 不能借 RECORD 自洽地携带会逃出安装根的成员路径。"""
    scripts_dir = str(WHEEL_CACHE_VERIFIER.parent)
    sys.path.insert(0, scripts_dir)
    try:
        module_spec = importlib.util.spec_from_file_location(
            "stage4_verify_python_wheel_cache", WHEEL_CACHE_VERIFIER
        )
        assert module_spec is not None and module_spec.loader is not None
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
    finally:
        sys.path.remove(scripts_dir)

    artifact = tmp_path / "path-escape.whl"
    metadata_path = "eclipse_ecal-6.1.1.dist-info/METADATA"
    wheel_path = "eclipse_ecal-6.1.1.dist-info/WHEEL"
    record_path = "eclipse_ecal-6.1.1.dist-info/RECORD"
    escaped_path = "../outside.py"
    metadata = (
        b"Metadata-Version: 2.1\nName: eclipse-ecal\nVersion: 6.1.1\n"
        b"Requires-Python: >=3.10\n"
    )
    wheel_metadata = b"Wheel-Version: 1.0\nTag: cp310-cp310-manylinux_2_28_x86_64\n"
    with zipfile.ZipFile(artifact, "w") as archive:
        archive.writestr(escaped_path, b"escaped\n")
        archive.writestr(metadata_path, metadata)
        archive.writestr(wheel_path, wheel_metadata)
        archive.writestr(
            record_path,
            _wheel_record(
                record_path,
                {
                    escaped_path: b"escaped\n",
                    metadata_path: metadata,
                    wheel_path: wheel_metadata,
                },
            ),
        )

    contract = {
        "distribution": "eclipse-ecal",
        "version": "6.1.1",
        "requires_python": ">=3.10",
        "python_tag": "cp310",
        "abi_tag": "cp310",
        "platform_tag": "manylinux_2_28_x86_64",
    }
    try:
        module._verify_zip_wheel_contract(artifact, contract)
    except ValueError as error:
        assert str(error) == "wheel ZIP member path is unsafe"
    else:
        raise AssertionError("ZIP member path escape was accepted")


def test_python_wheel_verifier_requires_record_to_enumerate_all_members(tmp_path) -> None:
    """RECORD 不能遗漏 wheel 内可安装的普通文件。"""
    filename = "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    wheel_bytes = io.BytesIO()
    with zipfile.ZipFile(wheel_bytes, "w") as archive:
        archive.writestr("ecal/__init__.py", "__version__ = '6.1.1'\n")
        archive.writestr(
            "eclipse_ecal-6.1.1.dist-info/METADATA",
            "Metadata-Version: 2.1\nName: eclipse-ecal\nVersion: 6.1.1\nRequires-Python: >=3.10\n",
        )
        archive.writestr(
            "eclipse_ecal-6.1.1.dist-info/WHEEL",
            "Wheel-Version: 1.0\nRoot-Is-Purelib: false\nTag: cp310-cp310-manylinux_2_28_x86_64\n",
        )
        archive.writestr("eclipse_ecal-6.1.1.dist-info/RECORD", "")
    payload = wheel_bytes.getvalue()
    sha256 = hashlib.sha256(payload).hexdigest()
    cache_root = tmp_path / "wheel-cache"
    wheel = cache_root / "wheels" / sha256 / filename
    wheel.parent.mkdir(parents=True)
    wheel.write_bytes(payload)
    manifest = tmp_path / "python-wheel-cache.manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "wheel": {
                    "url": f"https://files.example.invalid/packages/{filename}",
                    "filename": filename,
                    "relative_path": f"wheels/{sha256}/{filename}",
                    "size": len(payload),
                    "sha256": sha256,
                    "distribution": "eclipse-ecal",
                    "version": "6.1.1",
                    "requires_python": ">=3.10",
                    "python_tag": "cp310",
                    "abi_tag": "cp310",
                    "platform_tag": "manylinux_2_28_x86_64",
                },
            }
        ),
        encoding="utf-8",
    )

    rejected = subprocess.run(
        [
            sys.executable,
            str(WHEEL_CACHE_VERIFIER),
            "--manifest",
            str(manifest),
            "--cache-root",
            str(cache_root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert rejected.returncode != 0
    assert "wheel RECORD does not enumerate all archive members" in rejected.stdout


def test_python_wheel_verifier_rejects_record_member_sha256_mismatch(tmp_path) -> None:
    """RECORD 即使枚举完整，也必须绑定每个普通成员的真实摘要。"""
    scripts_dir = str(WHEEL_CACHE_VERIFIER.parent)
    sys.path.insert(0, scripts_dir)
    try:
        module_spec = importlib.util.spec_from_file_location(
            "stage4_verify_python_wheel_cache", WHEEL_CACHE_VERIFIER
        )
        assert module_spec is not None and module_spec.loader is not None
        module = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(module)
    finally:
        sys.path.remove(scripts_dir)

    artifact = tmp_path / "eclipse_ecal.whl"
    with zipfile.ZipFile(artifact, "w") as archive:
        archive.writestr("ecal/__init__.py", "__version__ = '6.1.1'\n")
        archive.writestr(
            "eclipse_ecal-6.1.1.dist-info/METADATA",
            "Metadata-Version: 2.1\nName: eclipse-ecal\nVersion: 6.1.1\nRequires-Python: >=3.10\n",
        )
        archive.writestr(
            "eclipse_ecal-6.1.1.dist-info/WHEEL",
            "Wheel-Version: 1.0\nRoot-Is-Purelib: false\nTag: cp310-cp310-manylinux_2_28_x86_64\n",
        )
        archive.writestr(
            "eclipse_ecal-6.1.1.dist-info/RECORD",
            "ecal/__init__.py,sha256=AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA,22\n"
            "eclipse_ecal-6.1.1.dist-info/METADATA,,\n"
            "eclipse_ecal-6.1.1.dist-info/WHEEL,,\n"
            "eclipse_ecal-6.1.1.dist-info/RECORD,,\n",
        )

    contract = {
        "distribution": "eclipse-ecal",
        "version": "6.1.1",
        "requires_python": ">=3.10",
        "python_tag": "cp310",
        "abi_tag": "cp310",
        "platform_tag": "manylinux_2_28_x86_64",
    }
    try:
        module._verify_zip_wheel_contract(artifact, contract)
    except ValueError as error:
        assert str(error) == "wheel RECORD SHA-256 differs from member"
    else:
        raise AssertionError("RECORD member hash mismatch was accepted")


def test_private_protobuf_builder_rejects_noncanonical_source_before_build(tmp_path) -> None:
    """私有 Conda 包只能从已冻结的 Protobuf v33.6 archive 构建。"""
    assert PRIVATE_PROTOBUF_BUILDER.is_file(), "private protobuf Conda builder is not implemented"
    source = tmp_path / "protobuf.tar.gz"
    source.write_bytes(b"not the locked protobuf source archive")
    result = subprocess.run(
        [
            sys.executable,
            str(PRIVATE_PROTOBUF_BUILDER),
            "--source-archive",
            str(source),
            "--abseil-source-archive",
            str(ABSEIL_SOURCE_ARCHIVE),
            "--work-root",
            str(tmp_path / "work"),
            "--channel-root",
            str(tmp_path / "channel"),
            "--conda-build",
            str(tmp_path / "conda-build"),
            "--check-only",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "protobuf source archive sha256 differs from frozen dependency lock" in result.stdout


def test_private_protobuf_builder_rejects_noncanonical_abseil_source_before_build(tmp_path) -> None:
    """私有 Conda 包必须在物化前拒绝非冻结的 Abseil LTS 源码。"""
    assert PRIVATE_PROTOBUF_BUILDER.is_file(), "private protobuf Conda builder is not implemented"
    protobuf_source = ROOT / "build" / "stage4-source-archive-input" / "v33.6.tar.gz"
    assert protobuf_source.is_file(), "locked protobuf archive fixture is unavailable"
    abseil_source = tmp_path / "abseil.tar.gz"
    abseil_source.write_bytes(b"not the locked abseil source archive")
    result = subprocess.run(
        [
            sys.executable,
            str(PRIVATE_PROTOBUF_BUILDER),
            "--source-archive",
            str(protobuf_source),
            "--abseil-source-archive",
            str(abseil_source),
            "--work-root",
            str(tmp_path / "work"),
            "--channel-root",
            str(tmp_path / "channel"),
            "--conda-build",
            str(tmp_path / "conda-build"),
            "--check-only",
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "abseil source archive sha256 differs from frozen dependency lock" in result.stdout


def test_private_protobuf_recipe_uses_only_materialized_frozen_source() -> None:
    """私有 Protobuf recipe 必须固定版本并禁止声明可联网的 source URL。"""
    assert PRIVATE_PROTOBUF_RECIPE.is_file(), "private protobuf Conda recipe is not implemented"
    recipe = PRIVATE_PROTOBUF_RECIPE.read_text(encoding="utf-8")
    assert "version: 6.33.6" in recipe
    assert "STAGE4_PROTOBUF_SOURCE_DIR" in recipe
    assert "script: |" in recipe
    assert "$PYTHON setup.py install" in recipe
    assert "$PYTHON python/dist/setup.py install" not in recipe
    assert "url:" not in recipe


def test_private_protobuf_recipe_stages_release_layout_without_mutating_source() -> None:
    """release setup.py 必须在独立目录取得其期望的源码布局。"""
    recipe = PRIVATE_PROTOBUF_RECIPE.read_text(encoding="utf-8")
    assert "stage4-python-dist" in recipe
    assert "cp -R python/google" in recipe
    assert "cp python/*.c" in recipe
    assert "cp python/*.h" in recipe
    assert "cp upb/reflection/cmake/google/protobuf/descriptor.upb.h" in recipe
    assert "cp upb/reflection/cmake/google/protobuf/descriptor.upb_minitable.h" in recipe
    assert "cp -R upb" in recipe
    assert 'rm -rf "$stage_dir/upb/reflection/stage0"' in recipe
    assert 'rm -rf "$stage_dir/upb/conformance"' in recipe
    assert "upb/reflection/stage0/google/protobuf/descriptor.upb" not in recipe
    assert "cp -R third_party/utf8_range" in recipe
    assert "cp python/dist/setup.py" in recipe
    assert "cd \"$stage_dir\"" in recipe
    assert "$PYTHON setup.py install" in recipe
    assert "ln -s" not in recipe


def test_private_protobuf_recipe_uses_portable_utf8_range_sources() -> None:
    """Linux-64 基线 _upb 只能编译实际使用的可移植 UTF-8 验证器。"""
    recipe = PRIVATE_PROTOBUF_RECIPE.read_text(encoding="utf-8")
    assert 'rm -f "$stage_dir"/utf8_range/lemire-*.c' in recipe
    assert 'rm -f "$stage_dir"/utf8_range/range-*.c' in recipe
    assert 'rm -f "$stage_dir"/utf8_range/range2-*.c' in recipe
    assert 'rm -f "$stage_dir"/utf8_range/lookup.c' in recipe
    assert 'rm -f "$stage_dir"/utf8_range/main.c' in recipe
    assert 'rm -f "$stage_dir"/utf8_range/naive.c' in recipe
    assert "-mssse3" not in recipe
    assert "-march=native" not in recipe


def test_private_protobuf_recipe_generates_upbdefs_without_network_fallback() -> None:
    """原生 _upb 包必须用冻结源码本地生成缺失的 reflection 产物。"""
    recipe = PRIVATE_PROTOBUF_RECIPE.read_text(encoding="utf-8")
    assert "cmake 3.28.*" in recipe
    assert "ninja" in recipe
    assert "STAGE4_ABSEIL_SOURCE_DIR" in recipe
    assert "script_env:\n    - STAGE4_ABSEIL_SOURCE_DIR" in recipe
    assert "protobuf_FORCE_FETCH_DEPENDENCIES=ON" in recipe
    assert 'FETCHCONTENT_SOURCE_DIR_ABSL="$STAGE4_ABSEIL_SOURCE_DIR"' in recipe
    assert "{{ compiler('cxx') }}" in recipe
    assert "protobuf_LOCAL_DEPENDENCIES_ONLY=ON" not in recipe
    assert "protoc-gen-upbdefs" in recipe
    assert "--upbdefs_out" in recipe
    assert "src/google/protobuf/descriptor.proto" in recipe


def test_private_protobuf_recipe_pins_linux_compiler_variants() -> None:
    """私有 Python 扩展必须冻结可解析的 Linux GCC/sysroot 变体。"""
    assert PRIVATE_PROTOBUF_VARIANTS.is_file(), "private protobuf compiler variants are not implemented"
    variants = yaml.safe_load(PRIVATE_PROTOBUF_VARIANTS.read_text(encoding="utf-8"))
    assert variants == {
        "target_platform": ["linux-64"],
        "c_compiler": ["gcc"],
        "c_compiler_version": ["13"],
        "c_stdlib": ["sysroot"],
        "c_stdlib_version": ["2.17"],
    }


def test_private_protobuf_builder_rejects_existing_output_roots(tmp_path) -> None:
    """私有包构建不得复用或污染已有 work/channel 目录。"""
    source = ROOT / "build" / "stage4-source-archive-input" / "v33.6.tar.gz"
    assert source.is_file(), "locked protobuf archive fixture is unavailable"
    work_root = tmp_path / "work"
    work_root.mkdir()
    result = subprocess.run(
        [
            sys.executable,
            str(PRIVATE_PROTOBUF_BUILDER),
            "--source-archive", str(source),
            "--abseil-source-archive", str(ABSEIL_SOURCE_ARCHIVE),
            "--work-root", str(work_root),
            "--channel-root", str(tmp_path / "channel"),
            "--conda-build", str(tmp_path / "conda-build"),
            "--check-only",
        ],
        check=False, capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert "private protobuf work and channel roots must not already exist" in result.stdout


def test_private_protobuf_builder_materializes_locked_source_without_build(tmp_path) -> None:
    """materialize-only 必须从 canonical archive 写出零链接源码树且不创建 channel。"""
    source = ROOT / "build" / "stage4-source-archive-input" / "v33.6.tar.gz"
    assert source.is_file(), "locked protobuf archive fixture is unavailable"
    work_root = tmp_path / "work"
    channel_root = tmp_path / "channel"
    result = subprocess.run(
        [
            sys.executable, str(PRIVATE_PROTOBUF_BUILDER),
            "--source-archive", str(source),
            "--abseil-source-archive", str(ABSEIL_SOURCE_ARCHIVE),
            "--work-root", str(work_root),
            "--channel-root", str(channel_root),
            "--conda-build", str(tmp_path / "conda-build"),
            "--materialize-only",
        ],
        check=False, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout
    assert (work_root / "source" / "protobuf-33.6" / "python" / "dist" / "setup.py").is_file()
    assert not channel_root.exists()


def test_private_protobuf_builder_rejects_nonexecutable_conda_build_before_outputs(tmp_path) -> None:
    """完整私有包构建必须在写输出前拒绝不存在或不可执行的 conda-build。"""
    source = ROOT / "build" / "stage4-source-archive-input" / "v33.6.tar.gz"
    assert source.is_file(), "locked protobuf archive fixture is unavailable"
    work_root = tmp_path / "work"
    channel_root = tmp_path / "channel"
    conda_build = tmp_path / "conda-build"
    conda_build.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    result = subprocess.run(
        [
            sys.executable, str(PRIVATE_PROTOBUF_BUILDER),
            "--source-archive", str(source),
            "--abseil-source-archive", str(ABSEIL_SOURCE_ARCHIVE),
            "--work-root", str(work_root),
            "--channel-root", str(channel_root),
            "--conda-build", str(conda_build),
        ],
        check=False, capture_output=True, text=True,
    )
    assert result.returncode != 0
    assert "conda-build must be an executable regular file" in result.stdout
    assert not work_root.exists()
    assert not channel_root.exists()


def test_private_protobuf_builder_prints_explicit_build_command(tmp_path) -> None:
    """完整构建命令必须绑定 recipe、物化源码、独立 croot 与 local channel 输出。"""
    source = ROOT / "build" / "stage4-source-archive-input" / "v33.6.tar.gz"
    conda_build = ROOT / "build" / "stage4-python-producers" / "micromamba.JFOCPK" / "lock-env-conda-build" / "bin" / "conda-build"
    assert source.is_file() and conda_build.is_file()
    result = subprocess.run(
        [sys.executable, str(PRIVATE_PROTOBUF_BUILDER), "--source-archive", str(source),
         "--abseil-source-archive", str(ABSEIL_SOURCE_ARCHIVE),
         "--work-root", str(tmp_path / "work"), "--channel-root", str(tmp_path / "channel"),
         "--conda-build", str(conda_build), "--print-build-command"],
        check=False, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stdout
    assert str(PRIVATE_PROTOBUF_RECIPE) in result.stdout
    assert "--output-folder" in result.stdout
    assert "--croot" in result.stdout


def test_private_protobuf_builder_materializes_source_then_invokes_isolated_producer(tmp_path) -> None:
    """完整 producer 必须从冻结源码启动，并向独立构建根传递固定调用参数。"""
    source = ROOT / "build" / "stage4-source-archive-input" / "v33.6.tar.gz"
    assert source.is_file(), "locked protobuf archive fixture is unavailable"
    work_root = tmp_path / "work"
    channel_root = tmp_path / "channel"
    relative_work_root = Path(os.path.relpath(work_root, ROOT))
    relative_channel_root = Path(os.path.relpath(channel_root, ROOT))
    record = tmp_path / "conda-build-record.txt"
    conda_build = tmp_path / "conda-build"
    conda_build.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$STAGE4_PROTOBUF_SOURCE_DIR\" > \"$STAGE4_TEST_RECORD\"\n"
        "printf '%s\\n' \"$@\" >> \"$STAGE4_TEST_RECORD\"\n"
        "previous=\n"
        "for argument in \"$@\"; do\n"
        "  if [ \"$previous\" = --output-folder ]; then\n"
        "    mkdir -p \"$argument/linux-64\"\n"
        "    : > \"$argument/linux-64/protobuf-6.33.6-0.conda\"\n"
        "  fi\n"
        "  previous=\"$argument\"\n"
        "done\n",
        encoding="utf-8",
    )
    conda_build.chmod(0o700)
    conda = tmp_path / "conda"
    conda.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    conda.chmod(0o700)

    result = subprocess.run(
        [
            sys.executable,
            str(PRIVATE_PROTOBUF_BUILDER),
            "--source-archive",
            str(source),
            "--abseil-source-archive",
            str(ABSEIL_SOURCE_ARCHIVE),
            "--work-root",
            str(relative_work_root),
            "--channel-root",
            str(relative_channel_root),
            "--conda-build",
            str(conda_build),
        ],
        check=False,
        capture_output=True,
        env={**os.environ, "STAGE4_TEST_RECORD": str(record)},
        text=True,
    )

    assert result.returncode == 0, result.stdout
    source_root = work_root / "source" / "protobuf-33.6"
    assert (source_root / "python" / "dist" / "setup.py").is_file()
    assert record.read_text(encoding="utf-8").splitlines() == [
        str(source_root),
        str(PRIVATE_PROTOBUF_RECIPE.parent),
        "--croot",
        str(work_root / "croot"),
        "--output-folder",
        str(channel_root),
    ]


def test_private_protobuf_builder_indexes_local_channel_with_same_toolchain_prefix(tmp_path) -> None:
    """成功构建后必须用与 conda-build 同前缀的 conda 写 local channel 索引。"""
    source = ROOT / "build" / "stage4-source-archive-input" / "v33.6.tar.gz"
    assert source.is_file(), "locked protobuf archive fixture is unavailable"
    toolchain_bin = tmp_path / "toolchain" / "bin"
    toolchain_bin.mkdir(parents=True)
    conda_build = toolchain_bin / "conda-build"
    conda_build.write_text(
        "#!/bin/sh\n"
        "previous=\n"
        "for argument in \"$@\"; do\n"
        "  if [ \"$previous\" = --output-folder ]; then\n"
        "    mkdir -p \"$argument/linux-64\"\n"
        "    : > \"$argument/linux-64/protobuf-6.33.6-0.conda\"\n"
        "  fi\n"
        "  previous=\"$argument\"\n"
        "done\n",
        encoding="utf-8",
    )
    conda_build.chmod(0o700)
    index_record = tmp_path / "index-record.txt"
    conda = toolchain_bin / "conda"
    conda.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$@\" > \"$STAGE4_TEST_INDEX_RECORD\"\n",
        encoding="utf-8",
    )
    conda.chmod(0o700)
    channel_root = tmp_path / "channel"

    result = subprocess.run(
        [
            sys.executable,
            str(PRIVATE_PROTOBUF_BUILDER),
            "--source-archive",
            str(source),
            "--abseil-source-archive",
            str(ABSEIL_SOURCE_ARCHIVE),
            "--work-root",
            str(tmp_path / "work"),
            "--channel-root",
            str(channel_root),
            "--conda-build",
            str(conda_build),
        ],
        check=False,
        capture_output=True,
        env={**os.environ, "STAGE4_TEST_INDEX_RECORD": str(index_record)},
        text=True,
    )

    assert result.returncode == 0, result.stdout
    assert index_record.is_file(), "conda index was not invoked"
    assert index_record.read_text(encoding="utf-8").splitlines() == [
        "index",
        str(channel_root),
    ]


def test_private_protobuf_builder_rejects_missing_package_before_channel_index(tmp_path) -> None:
    """producer 未输出预期制品时必须在索引前失败，不能把空 channel 伪装为成功。"""
    source = ROOT / "build" / "stage4-source-archive-input" / "v33.6.tar.gz"
    assert source.is_file(), "locked protobuf archive fixture is unavailable"
    toolchain_bin = tmp_path / "toolchain" / "bin"
    toolchain_bin.mkdir(parents=True)
    conda_build = toolchain_bin / "conda-build"
    conda_build.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    conda_build.chmod(0o700)
    index_record = tmp_path / "index-record.txt"
    conda = toolchain_bin / "conda"
    conda.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$@\" > \"$STAGE4_TEST_INDEX_RECORD\"\n",
        encoding="utf-8",
    )
    conda.chmod(0o700)

    result = subprocess.run(
        [
            sys.executable,
            str(PRIVATE_PROTOBUF_BUILDER),
            "--source-archive",
            str(source),
            "--abseil-source-archive",
            str(ABSEIL_SOURCE_ARCHIVE),
            "--work-root",
            str(tmp_path / "work"),
            "--channel-root",
            str(tmp_path / "channel"),
            "--conda-build",
            str(conda_build),
        ],
        check=False,
        capture_output=True,
        env={**os.environ, "STAGE4_TEST_INDEX_RECORD": str(index_record)},
        text=True,
    )

    assert result.returncode != 0
    assert "private Protobuf producer did not create exactly one package" in result.stdout
    assert not index_record.exists()


def test_private_protobuf_builder_persists_failed_producer_output(tmp_path) -> None:
    """外部 producer 失败时必须在 work root 保留 stdout/stderr，供失败证据复核。"""
    source = ROOT / "build" / "stage4-source-archive-input" / "v33.6.tar.gz"
    assert source.is_file(), "locked protobuf archive fixture is unavailable"
    toolchain_bin = tmp_path / "toolchain" / "bin"
    toolchain_bin.mkdir(parents=True)
    conda_build = toolchain_bin / "conda-build"
    conda_build.write_text(
        "#!/bin/sh\n"
        "printf 'producer stdout'\n"
        "printf 'producer stderr' >&2\n"
        "exit 7\n",
        encoding="utf-8",
    )
    conda_build.chmod(0o700)
    conda = toolchain_bin / "conda"
    conda.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    conda.chmod(0o700)
    work_root = tmp_path / "work"

    result = subprocess.run(
        [
            sys.executable,
            str(PRIVATE_PROTOBUF_BUILDER),
            "--source-archive",
            str(source),
            "--abseil-source-archive",
            str(ABSEIL_SOURCE_ARCHIVE),
            "--work-root",
            str(work_root),
            "--channel-root",
            str(tmp_path / "channel"),
            "--conda-build",
            str(conda_build),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert (work_root / "producer.log").read_text(encoding="utf-8") == (
        "stdout:\nproducer stdout\nstderr:\nproducer stderr\n"
    )
