import subprocess
import sys
from pathlib import Path


def test_run_slope_sweep_script_works_when_executed_by_path(tmp_path: Path):
    result = subprocess.run(
        [
            sys.executable,
            "experiments/run_slope_sweep.py",
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
        cwd=Path(__file__).resolve().parents[1],
        text=True,
        capture_output=True,
    )

    assert result.returncode == 0, result.stderr
    assert (tmp_path / "summary.csv").exists()

