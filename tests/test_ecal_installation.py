# 官方 eCAL 安装契约测试：锁定可复现版本并拒绝同名非 Eclipse 包。
from __future__ import annotations

import importlib
import importlib.metadata
from pathlib import Path
import tomli

import google.protobuf
import yaml


ROOT = Path(__file__).resolve().parents[1]


def _flatten_conda_and_pip_dependencies(items: object) -> list[str]:
    """把 Conda 字符串项和 pip 子清单展平成稳定依赖列表。"""
    if not isinstance(items, list):
        raise AssertionError("environment dependencies must be a list")
    flattened: list[str] = []
    for item in items:
        if isinstance(item, str):
            flattened.append(item)
        elif isinstance(item, dict):
            pip_items = item.get("pip", ())
            if not isinstance(pip_items, list):
                raise AssertionError("environment pip dependencies must be a list")
            flattened.extend(pip_items)
    return flattened


def test_environment_pins_one_ecal_compatible_protobuf_toolchain() -> None:
    environment = yaml.safe_load((ROOT / "environment.yml").read_text(encoding="utf-8"))
    dependencies = _flatten_conda_and_pip_dependencies(environment["dependencies"])

    assert "protobuf=6.33.6" in dependencies
    assert "grpcio-tools=1.76.0" in dependencies
    assert "packaging=26.2" in dependencies
    assert "pip" in dependencies
    assert "eclipse-ecal==6.1.1" in dependencies
    assert not any(item.startswith("ecal=") for item in dependencies)


def test_pyproject_uses_the_same_runtime_generator_and_ecal_versions() -> None:
    project = tomli.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))[
        "project"
    ]

    assert "protobuf>=6.33.6,<6.34" in project["dependencies"]
    assert project["optional-dependencies"]["dev"] == ["grpcio-tools==1.76.0"]
    assert project["optional-dependencies"]["interfaces"] == [
        "eclipse-ecal==6.1.1"
    ]


def test_official_ecal_611_bindings_import_with_project_protobuf_runtime() -> None:
    assert importlib.metadata.version("eclipse-ecal") == "6.1.1"
    assert google.protobuf.__version__ == "6.33.6"

    core = importlib.import_module("ecal.nanobind_core")
    proto_core = importlib.import_module("ecal.msg.proto.core")
    assert core.get_version_string().removeprefix("v").startswith("6.1.1")
    assert callable(proto_core.Publisher)
    assert callable(proto_core.Subscriber)
