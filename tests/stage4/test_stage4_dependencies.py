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
CPP_DEPENDENCY_LOCK = ROOT / "packaging" / "locks" / "cpp-dependencies.lock"
ROS2_DEPENDENCY_LOCK = ROOT / "packaging" / "locks" / "ros2-dependencies.lock"


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

    result = subprocess.run([str(DEPENDENCY_BUILDER), "--materialize-only", "--source-archive-cache", str(cache), "--source-archive-manifest", str(manifest), "--dependency-lock", str(lock), "--source-work", str(source_work), "--build-root", str(build_root), "--install-prefix", str(install_prefix), "--source-date-epoch", "1"], check=False, capture_output=True, text=True)

    assert result.returncode == 0, result.stderr
    assert (source_work / "trees" / "fixture" / "fixture-root" / "value.txt").read_bytes() == b"value"
    assert (source_work / "materialization.json").is_file()
    assert not build_root.exists()
    assert not install_prefix.exists()


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


def test_real_dependency_locks_cover_the_frozen_stage4_source_set() -> None:
    """真实 C++ 与 ROS 锁必须覆盖阶段四构建需要的全部八份源码。"""
    assert CPP_DEPENDENCY_LOCK.is_file(), "C++ dependency lock is not implemented"
    assert ROS2_DEPENDENCY_LOCK.is_file(), "ROS 2 dependency lock is not implemented"

    verifier = _load_verifier()
    load_lock = getattr(verifier, "load_dependency_lock", None)
    assert callable(load_lock), "dependency verifier needs a structured lock parser"
    entries = (*load_lock(CPP_DEPENDENCY_LOCK), *load_lock(ROS2_DEPENDENCY_LOCK))
    by_name = {entry.name: entry for entry in entries}

    assert set(by_name) == {
        "abseil-cpp",
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
    for name, entry in by_name.items():
        if name not in {"Livox-SDK2", "abseil-cpp"}:
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
