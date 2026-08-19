"""阶段四 A：非 eCAL 回归的原生初始化追踪证据合同。"""
from importlib import import_module
import os
from pathlib import Path
import subprocess
import sys

import pytest


def require_wished_module(name: str):
    """将尚未实现的追踪验证器转为可读的 RED。"""
    try:
        return import_module(name)
    except ModuleNotFoundError as error:
        if error.name != name and not name.startswith(f"{error.name}."):
            raise
        pytest.fail(f"wished-for behavior is not implemented: {name}", pytrace=False)


def test_trace_verifier_requires_each_loaded_hook_to_install(tmp_path: Path) -> None:
    """加载钩子后未记录成功安装时，零 initialize 不能视为通过。"""
    module = require_wished_module("scripts.verify_ecal_init_trace")
    trace = tmp_path / "init-trace.log"
    trace.write_text(
        "sitecustomize pid=101\n"
        "hook_installed pid=101\n"
        "sitecustomize pid=102\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="missing hook_installed"):
        module.verify_trace(trace, require_zero_initialize=True)


@pytest.mark.parametrize(
    ("core_source", "expect_success"),
    [
        ("def initialize(*args):\n    return 'native'\n", True),
        ("VALUE = 1\n", False),
    ],
)
def test_trace_hook_records_install_outcome(
    tmp_path: Path, core_source: str, expect_success: bool
) -> None:
    """hook 必须记录成功安装，或使缺失官方入口的追踪无效。"""
    hook_directory = Path("scripts/stage4_ecal_init_trace_hook")
    assert hook_directory.is_dir(), "wished-for eCAL init trace hook is not implemented"
    package = tmp_path / "ecal"
    package.mkdir()
    (package / "__init__.py").write_text("", encoding="utf-8")
    (package / "nanobind_core.py").write_text(core_source, encoding="utf-8")
    trace = tmp_path / "init-trace.log"
    environment = {
        **os.environ,
        "PYTHONPATH": os.pathsep.join((str(hook_directory.resolve()), str(tmp_path))),
        "STAGE4_ECAL_INIT_TRACE": str(trace),
    }
    program = "from ecal import nanobind_core; getattr(nanobind_core, 'initialize', lambda *_args: None)('test')"
    completed = subprocess.run(
        [sys.executable, "-c", program],
        env=environment,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    text = trace.read_text(encoding="utf-8")
    verifier = require_wished_module("scripts.verify_ecal_init_trace")
    if expect_success:
        assert "hook_installed pid=" in text
        assert "initialize args" in text
        assert verifier.verify_trace(trace, require_zero_initialize=False) == {
            "hook_loads": 1,
            "unique_pids": 1,
            "ecal_unavailable": 0,
            "initialize_calls": 1,
        }
    else:
        assert "hook_install_failed pid=" in text
        with pytest.raises(ValueError, match="hook_install_failed"):
            verifier.verify_trace(trace, require_zero_initialize=True)


def test_trace_hook_marks_missing_ecal_as_unavailable(tmp_path: Path) -> None:
    """未安装 eCAL 的 conda 辅助进程必须可审计地排除，而非伪报安装失败。"""
    hook_directory = Path("scripts/stage4_ecal_init_trace_hook")
    trace = tmp_path / "init-trace.log"
    environment = {
        **os.environ,
        "PYTHONPATH": str(hook_directory.resolve()),
        "STAGE4_ECAL_INIT_TRACE": str(trace),
    }
    completed = subprocess.run(
        [sys.executable, "-S", "-c", "import sitecustomize; print('no ecal runtime')"],
        env=environment,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stderr
    assert "ecal_unavailable pid=" in trace.read_text(encoding="utf-8")
    verifier = require_wished_module("scripts.verify_ecal_init_trace")
    assert verifier.verify_trace(trace, require_zero_initialize=True) == {
        "hook_loads": 1,
        "unique_pids": 1,
        "ecal_unavailable": 1,
        "initialize_calls": 0,
    }
