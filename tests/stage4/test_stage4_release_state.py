"""阶段四 E：安装器同版本状态判据必须在写入 releases 前 fail-closed。"""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "scripts" / "stage4_release_state.py"


def _module():
    spec = importlib.util.spec_from_file_location("stage4_release_state", MODULE_PATH)
    assert spec is not None and spec.loader is not None, "stage4 release-state module is missing"
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _manifest(*, payload_sha256: str, with_ros: bool, files: dict[str, str]) -> dict[str, object]:
    return {
        "schema_version": 1,
        "version": "4.0.0",
        "git_sha": "a" * 40,
        "payload_sha256": payload_sha256,
        "with_ros": with_ros,
        "files": files,
        "dependencies": [],
        "doctor": {"files_verified": True},
    }


def test_complete_matching_release_only_needs_current_activation(tmp_path: Path) -> None:
    """完整同版本 release 未被 current 指向时，只能补原子激活而非覆盖文件。"""
    module = _module()
    release = tmp_path / "releases" / "4.0.0"
    release.mkdir(parents=True)
    (release / "runtime.bin").write_bytes(b"runtime")
    files = {"runtime.bin": module.sha256_file(release / "runtime.bin")}
    (release / "manifest.json").write_text(
        json.dumps(_manifest(payload_sha256="a" * 64, with_ros=False, files=files)), encoding="utf-8"
    )

    decision = module.inspect_release(
        release, _manifest(payload_sha256="a" * 64, with_ros=False, files=files)
    )

    assert decision == "activate_only"


def test_damaged_or_option_drift_release_is_rejected_before_overwrite(tmp_path: Path) -> None:
    """同版本文件损坏或 with_ros 漂移都不能被安装器原地覆盖。"""
    module = _module()
    release = tmp_path / "releases" / "4.0.0"
    release.mkdir(parents=True)
    (release / "runtime.bin").write_bytes(b"tampered")
    files = {"runtime.bin": "0" * 64}
    (release / "manifest.json").write_text(
        json.dumps(_manifest(payload_sha256="a" * 64, with_ros=False, files=files)), encoding="utf-8"
    )

    assert module.inspect_release(
        release, _manifest(payload_sha256="a" * 64, with_ros=False, files=files)
    ) == "reject"
    assert module.inspect_release(
        release, _manifest(payload_sha256="a" * 64, with_ros=True, files=files)
    ) == "reject"


def test_activation_atomically_replaces_current_without_touching_release(tmp_path: Path) -> None:
    """完整 release 只允许通过原子替换 current 激活，不能重写版本目录。"""
    module = _module()
    releases = tmp_path / "releases"
    old_release = releases / "3.9.0"
    release = releases / "4.0.0"
    old_release.mkdir(parents=True)
    release.mkdir()
    current = tmp_path / "current"
    current.symlink_to(old_release)

    module.activate_current(tmp_path, release)

    assert current.is_symlink()
    assert current.resolve() == release
    assert old_release.is_dir()
    assert release.is_dir()


def test_complete_installer_manifest_with_locked_dependency_only_needs_activation(tmp_path: Path) -> None:
    """release-state 必须复用安装器的依赖与 doctor 身份，而非拒绝新 manifest。"""
    module = _module()
    release = tmp_path / "releases" / "4.0.0"
    dependency = release / "dependencies" / "ecal.bin"
    dependency.parent.mkdir(parents=True)
    dependency.write_bytes(b"ecal")
    (release / "runtime.bin").write_bytes(b"runtime")
    files = {
        "runtime.bin": module.sha256_file(release / "runtime.bin"),
        "dependencies/ecal.bin": module.sha256_file(dependency),
    }
    manifest = _manifest(payload_sha256="a" * 64, with_ros=False, files=files)
    manifest["dependencies"] = [
        {
            "name": "ecal",
            "license": "Apache-2.0",
            "filename": "dependencies/ecal.bin",
            "sha256": files["dependencies/ecal.bin"],
        }
    ]
    (release / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    assert module.inspect_release(release, manifest) == "activate_only"
