# 阶段四参考清单合同：复用生产解析器检查真实固定 checkout 的准入资料。
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SYNC_SCRIPT = ROOT / "scripts" / "sync_references.py"
MANIFEST = ROOT / "references" / "manifest.yml"
REFERENCE_ROOT = ROOT / "references" / "repos"
RELEASE_MANIFEST = ROOT / "packaging" / "stage4-release-manifest.json"
RUN_BUILDER = ROOT / "scripts" / "build_stage4_run.py"
STAGE4_NAMES = frozenset(
    {
        "bullet3",
        "ecal",
        "protobuf",
        "mcap",
        "zstd",
        "livox_ros_driver2",
        "Livox-SDK2",
        "pcl",
        "livox_laser_simulation",
        "FAST_LIO",
        "elevation_mapping",
    }
)
LIVOX_NAMES = frozenset({"livox_ros_driver2", "Livox-SDK2"})


def _load_production_manifest() -> tuple[object, ...]:
    """以生产 parser 加载清单，避免测试复制同步器的准入语义。"""
    spec = importlib.util.spec_from_file_location("stage4_reference_manifest", SYNC_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
        return module.load_manifest(MANIFEST)
    finally:
        sys.modules.pop(spec.name, None)


def _load_run_builder():
    """加载生产安装器解析器，避免测试自行复制 URL 安全边界。"""
    spec = importlib.util.spec_from_file_location("stage4_run_builder", RUN_BUILDER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
        return module
    finally:
        sys.modules.pop(spec.name, None)


@pytest.mark.stage4_artifact
def test_real_stage4_manifest_has_pinned_admission_and_license_evidence() -> None:
    """十一个 stage 4 阅读快照及其许可证证据必须在固定 checkout 中存在。"""
    repositories = _load_production_manifest()
    stage4 = {repository.name: repository for repository in repositories if repository.stage == 4}

    assert set(stage4) == STAGE4_NAMES
    for name, repository in stage4.items():
        assert repository.url.startswith("https://github.com/")
        assert len(repository.commit) == 40
        assert repository.focus
        assert repository.license_files
        for path in repository.license_files:
            assert (REFERENCE_ROOT / name / path).is_file()


@pytest.mark.stage4_artifact
def test_livox_third_party_license_declarations_exist_in_pinned_checkouts() -> None:
    """Livox 两个归档的第三方 notice 不能只在 YAML 中声明而没有实际文件。"""
    raw = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    records = {record["name"]: record for record in raw["repositories"]}

    for name in LIVOX_NAMES:
        notices = records[name].get("third_party_license_files")
        assert isinstance(notices, list) and notices
        for notice in notices:
            assert (REFERENCE_ROOT / name / notice).is_file()


@pytest.mark.stage4_artifact
def test_release_manifest_pins_the_official_ros_bootstrap_and_livox_message_source() -> None:
    """ROS release 只允许已锁定的官方 bootstrap 与 CustomMsg 源码输入。"""
    document = json.loads(RELEASE_MANIFEST.read_text(encoding="utf-8"))
    dependencies = {record["name"]: record for record in document["ros_dependencies"]}

    assert dependencies["ros2-apt-source"] == {
        "name": "ros2-apt-source",
        "url": "http://packages.ros.org/ros2/ubuntu/pool/main/r/ros-apt-source/ros2-apt-source_1.2.0%7enoble_all.deb",
        "sha256": "0804d9b13db770eb87019be414cd78378835228ad5fa801fc88758596dd8f7e5",
        "filename": "ros2-apt-source_1.2.0~noble_all.deb",
        "license": "Apache-2.0",
    }
    assert dependencies["livox_ros_driver2"] == {
        "name": "livox_ros_driver2",
        "url": "https://github.com/Livox-SDK/livox_ros_driver2/archive/refs/tags/1.2.6.tar.gz",
        "sha256": "185e2fec89c06bec1abf2e2ededc4be23fc6d5347cc88b992f84074bc3795c76",
        "filename": "livox_ros_driver2-1.2.6.tar.gz",
        "license": "MIT",
    }

    normalized = _load_run_builder()._manifest(RELEASE_MANIFEST)
    assert {record["name"] for record in normalized["ros_dependencies"]} == set(dependencies)


@pytest.mark.stage4_artifact
def test_release_manifest_pins_the_official_livox_viewer_linux_bundle() -> None:
    """最终安装器从 Livox 官方 CDN 获取固定 Viewer，不把未知二进制混入 Git。"""
    document = json.loads(RELEASE_MANIFEST.read_text(encoding="utf-8"))
    dependencies = {record["name"]: record for record in document["dependencies"]}

    assert dependencies["livox-viewer2-linux"] == {
        "name": "livox-viewer2-linux",
        "url": (
            "https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/"
            "avia-2/Livox%20Viewer2%20Linux/Viewer2_2.6.0_Linux.zip"
        ),
        "sha256": "3a1e574d3d73ba0b36460c2a358d08f6c722ae0dc376395ba392ec0d533c7e31",
        "filename": "Viewer2_2.6.0_Linux.zip",
        "license": "Livox Viewer 2 proprietary binary",
    }
