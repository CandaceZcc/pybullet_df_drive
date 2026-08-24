"""本地 runSim 快捷入口的命令行合同。"""
from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[2]


def test_run_sim_reports_its_version_without_requiring_conda() -> None:
    """版本查询必须在环境缺失时仍可供安装排障和脚本探测。"""
    completed = subprocess.run(
        [str(ROOT / "runSim"), "--version"],
        cwd=ROOT,
        env={"PATH": os.environ["PATH"]},
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == "runSim 5.0.1\n"
    assert completed.stderr == ""


def test_run_sim_forwards_help_to_the_manual_gui_entrypoint() -> None:
    """runSim 必须可直接执行，并公开 PyBullet GUI 手动模式的帮助。"""
    launcher = ROOT / "runSim"
    assert launcher.is_file(), "runSim launcher is missing"
    completed = subprocess.run(
        [str(launcher), "--help"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    assert "--manual" in completed.stdout
    assert "--gui" in completed.stdout
    assert "--capture-output-dir" in completed.stdout
    assert "--capture-duration-sec" in completed.stdout
    assert "--viewer-root" in completed.stdout
    assert "--open-ros-rviz" in completed.stdout
    assert "--lidar" not in completed.stdout


def test_run_sim_uses_the_repository_root_when_invoked_via_a_symlink(
    tmp_path: Path,
) -> None:
    """放入 PATH 的软链接不能让入口误把链接目录当作仓库根目录。"""
    launcher = ROOT / "runSim"
    linked_launcher = tmp_path / "runSim"
    linked_launcher.symlink_to(launcher)
    completed = subprocess.run(
        [str(linked_launcher), "--help"],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
    )
    assert completed.returncode == 0, completed.stderr
    assert "--manual" in completed.stdout


def test_run_sim_uses_the_embedded_runtime_when_installed_in_release_bin(tmp_path: Path) -> None:
    """安装后的 bin/runSim 必须回到 release 根，并优先使用内嵌 Python。"""
    release = tmp_path / "release with spaces"
    launcher = release / "bin" / "runSim"
    runtime = release / "runtime" / "bin" / "python"
    launcher.parent.mkdir(parents=True)
    runtime.parent.mkdir(parents=True)
    shutil.copy2(ROOT / "runSim", launcher)
    (release / "main.py").write_text("# release entrypoint\n", encoding="utf-8")
    runtime.write_text(
        "#!/usr/bin/env sh\nprintf '%s\\n' \"$@\"\n",
        encoding="utf-8",
    )
    runtime.chmod(0o755)

    completed = subprocess.run(
        [str(launcher), "--help"],
        cwd=tmp_path,
        env={"PATH": os.environ["PATH"]},
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.splitlines()[:4] == [
        str(release / "main.py"),
        "--gui",
        "--manual",
        "--v2-realtime",
    ]


def test_installed_run_sim_supplies_its_bundled_ecal_defaults(tmp_path: Path) -> None:
    """release 有完整 eCAL 配置时，用户无需手动导出两个环境变量。"""
    release = tmp_path / "release"
    launcher = release / "bin" / "runSim"
    runtime = release / "runtime" / "bin" / "python"
    launcher.parent.mkdir(parents=True)
    runtime.parent.mkdir(parents=True)
    (release / "etc" / "ecal").mkdir(parents=True)
    (release / "lib").mkdir()
    shutil.copy2(ROOT / "runSim", launcher)
    (release / "main.py").write_text("# release entrypoint\n", encoding="utf-8")
    (release / "etc" / "ecal" / "ecal.yaml").write_text("# eCAL\n", encoding="utf-8")
    (release / "lib" / "libecaltime-localtime.so").write_bytes(b"plugin")
    runtime.write_text(
        "#!/usr/bin/env sh\nprintf '%s\\n%s\\n' \"$ECAL_DATA\" \"$ECAL_TIME_PLUGIN_PATH\"\n",
        encoding="utf-8",
    )
    runtime.chmod(0o755)

    completed = subprocess.run([str(launcher), "--help"], capture_output=True, text=True)

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.splitlines() == [
        str(release / "etc" / "ecal"),
        str(release / "lib"),
    ]


def test_installed_run_sim_supplies_its_bundled_fontconfig_defaults(tmp_path: Path) -> None:
    """内嵌 Fontconfig 不能继续查找安装 staging 中已经消失的配置。"""
    release = tmp_path / "release"
    launcher = release / "bin" / "runSim"
    runtime = release / "runtime" / "bin" / "python"
    launcher.parent.mkdir(parents=True)
    runtime.parent.mkdir(parents=True)
    (release / "runtime" / "etc" / "fonts").mkdir(parents=True)
    shutil.copy2(ROOT / "runSim", launcher)
    (release / "main.py").write_text("# release entrypoint\n", encoding="utf-8")
    (release / "runtime" / "etc" / "fonts" / "fonts.conf").write_text(
        "<fontconfig/>\n", encoding="utf-8"
    )
    runtime.write_text(
        "#!/usr/bin/env sh\nprintf '%s\\n%s\\n' \"$FONTCONFIG_FILE\" \"$FONTCONFIG_PATH\"\n",
        encoding="utf-8",
    )
    runtime.chmod(0o755)

    completed = subprocess.run([str(launcher), "--help"], capture_output=True, text=True)

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.splitlines() == [
        str(release / "runtime" / "etc" / "fonts" / "fonts.conf"),
        str(release / "runtime" / "etc" / "fonts"),
    ]


def test_installed_run_sim_preserves_explicit_fontconfig_configuration(tmp_path: Path) -> None:
    """调用者显式指定的 Fontconfig 配置必须优先于 release 默认值。"""
    release = tmp_path / "release"
    launcher = release / "bin" / "runSim"
    runtime = release / "runtime" / "bin" / "python"
    launcher.parent.mkdir(parents=True)
    runtime.parent.mkdir(parents=True)
    (release / "runtime" / "etc" / "fonts").mkdir(parents=True)
    shutil.copy2(ROOT / "runSim", launcher)
    (release / "main.py").write_text("# release entrypoint\n", encoding="utf-8")
    (release / "runtime" / "etc" / "fonts" / "fonts.conf").write_text(
        "<fontconfig/>\n", encoding="utf-8"
    )
    runtime.write_text(
        "#!/usr/bin/env sh\nprintf '%s\\n%s\\n' \"$FONTCONFIG_FILE\" \"$FONTCONFIG_PATH\"\n",
        encoding="utf-8",
    )
    runtime.chmod(0o755)
    environment = {
        **os.environ,
        "FONTCONFIG_FILE": "/managed/fonts.conf",
        "FONTCONFIG_PATH": "/managed/fonts",
    }

    completed = subprocess.run(
        [str(launcher), "--help"], capture_output=True, text=True, env=environment
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.splitlines() == ["/managed/fonts.conf", "/managed/fonts"]


def test_installed_run_sim_prevents_release_bytecode_writes(tmp_path: Path) -> None:
    """发行入口不能让 Python 的 __pycache__ 破坏安装器完整性校验。"""
    release = tmp_path / "release"
    launcher = release / "bin" / "runSim"
    runtime = release / "runtime" / "bin" / "python"
    launcher.parent.mkdir(parents=True)
    runtime.parent.mkdir(parents=True)
    shutil.copy2(ROOT / "runSim", launcher)
    (release / "main.py").write_text("# release entrypoint\n", encoding="utf-8")
    runtime.write_text(
        "#!/usr/bin/env sh\nprintf '%s\\n' \"${PYTHONDONTWRITEBYTECODE:-}\"\n",
        encoding="utf-8",
    )
    runtime.chmod(0o755)

    completed = subprocess.run([str(launcher), "--help"], capture_output=True, text=True)

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == "1\n"


def test_run_sim_defaults_to_formal_v2_ecal_keyboard_control(
    tmp_path: Path,
) -> None:
    """普通交互入口不能因企业 LiDAR 同步扫描而拖慢键盘与 GUI。"""
    args_path = tmp_path / "conda-args.txt"
    fake_conda = tmp_path / "conda"
    fake_conda.write_text(
        "#!/usr/bin/env sh\nprintf '%s\\n' \"$@\" > \"$RUN_SIM_ARGS\"\n",
        encoding="utf-8",
    )
    fake_conda.chmod(0o755)
    environment = {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "RUN_SIM_ARGS": str(args_path),
    }

    completed = subprocess.run(
        [str(ROOT / "runSim")],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    assert args_path.read_text(encoding="utf-8").splitlines()[-5:] == [
        "--gui",
        "--manual",
        "--v2-realtime",
        "--interface-mode",
        "ecal",
    ]


def test_run_sim_keeps_an_explicit_interface_mode_for_advanced_runs(
    tmp_path: Path,
) -> None:
    """显式接口模式由调用者承担，不能被交互默认项矛盾地覆盖。"""
    args_path = tmp_path / "conda-args.txt"
    fake_conda = tmp_path / "conda"
    fake_conda.write_text(
        "#!/usr/bin/env sh\nprintf '%s\\n' \"$@\" > \"$RUN_SIM_ARGS\"\n",
        encoding="utf-8",
    )
    fake_conda.chmod(0o755)
    environment = {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "RUN_SIM_ARGS": str(args_path),
    }

    completed = subprocess.run(
        [str(ROOT / "runSim"), "--interface-mode", "ecal"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    forwarded = args_path.read_text(encoding="utf-8").splitlines()
    assert "--no-interface" not in forwarded
    assert forwarded[-4:] == ["--manual", "--v2-realtime", "--interface-mode", "ecal"]


def test_run_sim_forwards_explicit_local_only_as_legacy_mode(
    tmp_path: Path,
) -> None:
    """显式 local 不得与正式 v2 eCAL 参数并存。"""
    args_path = tmp_path / "conda-args.txt"
    fake_conda = tmp_path / "conda"
    fake_conda.write_text(
        "#!/usr/bin/env sh\nprintf '%s\\n' \"$@\" > \"$RUN_SIM_ARGS\"\n",
        encoding="utf-8",
    )
    fake_conda.chmod(0o755)
    environment = {**os.environ, "PATH": f"{tmp_path}:{os.environ['PATH']}", "RUN_SIM_ARGS": str(args_path)}

    completed = subprocess.run(
        [str(ROOT / "runSim"), "--interface-mode", "local"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    forwarded = args_path.read_text(encoding="utf-8").splitlines()
    assert forwarded[-4:] == ["--manual", "--v2-realtime", "--interface-mode", "local"]


def test_run_sim_rejects_legacy_lidar_flags_and_points_to_dashboard_capture(
    tmp_path: Path,
) -> None:
    """旧同步雷达入口会严重卡顿；用户必须使用 Dashboard 的手动采集流程。"""
    args_path = tmp_path / "conda-args.txt"
    fake_conda = tmp_path / "conda"
    fake_conda.write_text(
        "#!/usr/bin/env sh\nprintf '%s\\n' \"$@\" > \"$RUN_SIM_ARGS\"\n",
        encoding="utf-8",
    )
    fake_conda.chmod(0o755)
    environment = {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "RUN_SIM_ARGS": str(args_path),
    }

    completed = subprocess.run(
        [str(ROOT / "runSim"), "--lidar"],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 2
    assert "Dashboard" in completed.stderr
    assert "启用采集" in completed.stderr
    assert not args_path.exists()
