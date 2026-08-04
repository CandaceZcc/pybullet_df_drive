# 阶段四参考清单合同：复用生产解析器检查真实固定 checkout 的准入资料。
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
SYNC_SCRIPT = ROOT / "scripts" / "sync_references.py"
MANIFEST = ROOT / "references" / "manifest.yml"
REFERENCE_ROOT = ROOT / "references" / "repos"
STAGE4_NAMES = frozenset(
    {
        "ecal",
        "protobuf",
        "mcap",
        "zstd",
        "livox_ros_driver2",
        "Livox-SDK2",
        "pcl",
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


def test_real_stage4_manifest_has_pinned_admission_and_license_evidence() -> None:
    """七个 stage 4 阅读快照及其许可证证据必须在固定 checkout 中存在。"""
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


def test_livox_third_party_license_declarations_exist_in_pinned_checkouts() -> None:
    """Livox 两个归档的第三方 notice 不能只在 YAML 中声明而没有实际文件。"""
    raw = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    records = {record["name"]: record for record in raw["repositories"]}

    for name in LIVOX_NAMES:
        notices = records[name].get("third_party_license_files")
        assert isinstance(notices, list) and notices
        for notice in notices:
            assert (REFERENCE_ROOT / name / notice).is_file()
