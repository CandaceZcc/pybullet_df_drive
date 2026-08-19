# 实验入口验收测试：确保阶段一批量与阶段二障碍物脚本可按路径执行。
import subprocess
import sys
from pathlib import Path


def test_run_slope_sweep_script_works_when_executed_by_path(tmp_path: Path):
    root = Path(__file__).resolve().parents[2]
    template = root / "configs" / "experiment.yaml"
    local_config = tmp_path / "experiment.local.yaml"
    local_config.write_text(
        template.read_text(encoding="utf-8").replace(
            "interface_mode: auto",
            "interface_mode: local",
        ),
        encoding="utf-8",
    )
    result = subprocess.run(
        [
            sys.executable,
            "experiments/run_slope_sweep.py",
            "--config",
            str(local_config),
            "--slopes",
            "0",
            "--trials",
            "1",
            "--duration-sec",
            "0.05",
            "--summary",
            str(tmp_path / "summary.csv"),
        ],
        check=False,
        cwd=root,
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0, result.stderr
    assert (tmp_path / "summary.csv").exists()


def test_stage2_obstacle_verifier_works_when_executed_by_path():
    result = subprocess.run(
        [
            sys.executable,
            "scripts/verify_stage2_obstacles.py",
        ],
        check=False,
        cwd=Path(__file__).resolve().parents[2],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0, result.stderr
    assert "PASS ground_flat_box" in result.stdout
    assert "PASS coordinator_robot_switch" in result.stdout
    assert "PASS coordinator_robot_reset" in result.stdout
    assert "PASS coordinator_edge_terrain_switch" in result.stdout
    assert "operation=add_50_clear_100" in result.stdout
    assert "SUMMARY pass=" in result.stdout
    assert "fail=0" in result.stdout
