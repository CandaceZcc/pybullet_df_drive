"""阶段四 E：正式 `.run` 内嵌项目 payload 的受控组装验收。"""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
BUILDER = ROOT / "scripts" / "stage4_project_payload.py"


def _write(path: Path, content: str = "payload\n") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def test_project_payload_builder_copies_runtime_sources_and_excludes_build_inputs(tmp_path: Path) -> None:
    """正式 payload 只包含运行所需源与锁，不能混入测试、构建或缓存目录。"""
    source = tmp_path / "source"
    _write(source / "main.py")
    _write(source / "runSim", "#!/bin/sh\n")
    (source / "runSim").chmod(0o755)
    _write(source / "pyproject.toml")
    _write(source / "scripts" / "stage4_v2_simulation_runtime.py")
    _write(source / "scripts" / "stage4_v2_dashboard.py")
    _write(source / "scripts" / "stage4_release_setup.py")
    _write(source / "scripts" / "run_mid360_golf_mapping.py")
    _write(source / "scripts" / "verify_mid360_golf_mapping_replay.py")
    _write(source / "scripts" / "mid360_golf_simulation.py")
    _write(source / "scripts" / "mid360_golf_command_peer.py")
    _write(source / "slope_sim" / "__init__.py")
    _write(source / "slope_sim" / "mid360_offline.py")
    _write(source / "slope_sim" / "mid360_golf_drive.py")
    _write(source / "slope_sim" / "mapping_replay.py")
    _write(source / "slope_sim" / "mapping_acceptance.py")
    _write(source / "slope_sim" / "mapping_mcap.py")
    _write(source / "slope_sim" / "mapping_replay_gui.py")
    _write(source / "slope_sim" / "assets" / "mid360_pattern.bin", "pattern")
    _write(source / "slope_sim" / "assets" / "mid360_pattern.provenance.json", "{}\n")
    _write(source / "slope_sim" / "assets" / "livox_laser_simulation.LICENSE", "MIT License\n")
    _write(source / "cpp" / "phase0" / "CMakeLists.txt")
    _write(source / "cpp" / "client" / "tests" / "v2_topics_test.cpp")
    _write(source / "proto" / "slope_sim_interfaces_v2.proto")
    _write(source / "urdf" / "df_back.urdf")
    _write(source / "configs" / "flat_demo.yaml")
    _write(source / "configs" / "mid360_golf_mapping.yaml")
    _write(source / "packaging" / "python-environment.yml")
    for lock in (
        "cpp-dependencies.lock",
        "ubuntu24-system-dependencies.lock",
        "ros2-dependencies.lock",
        "ros2-apt-packages.lock",
        "python.conda-lock.yml",
        "python-linux-64.lock",
    ):
        _write(source / "packaging" / "locks" / lock)
    _write(source / "tests" / "must-not-ship.py")
    _write(source / "build" / "must-not-ship.bin")
    _write(source / "results" / "must-not-ship.json")
    _write(source / "references" / "repos" / "must-not-ship.txt")
    _write(source / "packaging" / "locks" / "python-package-cache.manifest.json")
    output = tmp_path / "project-payload"

    built = subprocess.run(
        [sys.executable, str(BUILDER), "--source", str(source), "--output", str(output)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert built.returncode == 0, built.stderr
    assert (output / "main.py").is_file()
    assert (output / "runSim").is_file()
    assert (output / "runSim").stat().st_mode & 0o111
    assert (output / "scripts" / "stage4_v2_simulation_runtime.py").is_file()
    assert (output / "scripts" / "stage4_v2_dashboard.py").is_file()
    assert (output / "scripts" / "stage4_release_setup.py").is_file()
    assert (output / "scripts" / "run_mid360_golf_mapping.py").is_file()
    assert (output / "scripts" / "verify_mid360_golf_mapping_replay.py").is_file()
    assert (output / "scripts" / "mid360_golf_simulation.py").is_file()
    assert (output / "scripts" / "mid360_golf_command_peer.py").is_file()
    assert (output / "slope_sim" / "__init__.py").is_file()
    assert (output / "slope_sim" / "mid360_offline.py").is_file()
    assert (output / "slope_sim" / "mid360_golf_drive.py").is_file()
    assert (output / "slope_sim" / "mapping_replay.py").is_file()
    assert (output / "slope_sim" / "mapping_acceptance.py").is_file()
    assert (output / "slope_sim" / "mapping_mcap.py").is_file()
    assert (output / "slope_sim" / "mapping_replay_gui.py").is_file()
    assert (output / "slope_sim" / "assets" / "mid360_pattern.bin").is_file()
    assert (output / "slope_sim" / "assets" / "mid360_pattern.provenance.json").is_file()
    assert (output / "slope_sim" / "assets" / "livox_laser_simulation.LICENSE").is_file()
    assert (output / "cpp" / "phase0" / "CMakeLists.txt").is_file()
    assert (output / "cpp" / "client" / "tests" / "v2_topics_test.cpp").is_file()
    assert (output / "proto" / "slope_sim_interfaces_v2.proto").is_file()
    assert (output / "urdf" / "df_back.urdf").is_file()
    assert (output / "configs" / "flat_demo.yaml").is_file()
    assert (output / "configs" / "mid360_golf_mapping.yaml").is_file()
    assert not (output / "packaging" / "stage4-private-ca.pem").exists()
    assert (output / "packaging" / "locks" / "cpp-dependencies.lock").is_file()
    assert (output / "packaging" / "locks" / "ubuntu24-system-dependencies.lock").is_file()
    assert (output / "packaging" / "locks" / "ros2-apt-packages.lock").is_file()
    assert not (output / "tests").exists()
    assert not (output / "build").exists()
    assert not (output / "results").exists()
    assert not (output / "references").exists()
    assert not (output / "packaging" / "locks" / "python-package-cache.manifest.json").exists()
    manifest = json.loads((output / "payload-manifest.json").read_text(encoding="utf-8"))
    assert manifest["schema_version"] == 1
    assert "main.py" in manifest["files"]
    assert "tests/must-not-ship.py" not in manifest["files"]
