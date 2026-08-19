# Livox Viewer 2 Linux 兼容启动器测试：约束多网卡环境下的无网络显示 smoke。
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from scripts import verify_livox_viewer2_linux as verifier


def test_isolated_viewer_command_preserves_x11_and_removes_proxy_environment(
    tmp_path: Path,
) -> None:
    """隔离模式只断开 Viewer 网络，不改变主机 DISPLAY 或系统代理设置。"""
    viewer_root = tmp_path / "Viewer2_2.6.0_Linux"
    launcher = viewer_root / "LivoxViewer2.sh"
    launcher.parent.mkdir(parents=True)
    launcher.write_text("#!/bin/sh\n", encoding="utf-8")

    command, environment = verifier.isolated_viewer_command(
        viewer_root=viewer_root,
        display=":1",
        xdg_root=tmp_path / "xdg",
    )

    assert command[:5] == ["bwrap", "--unshare-net", "--bind", "/", "/"]
    for name in ("HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "http_proxy", "https_proxy", "all_proxy"):
        assert ("--unsetenv", name) in zip(command, command[1:])
    for name in ("DISPLAY", "XDG_CONFIG_HOME", "XDG_CACHE_HOME", "XDG_DATA_HOME"):
        offset = command.index("--setenv", command.index("--setenv") if name == "DISPLAY" else 0)
        while command[offset + 1] != name:
            offset = command.index("--setenv", offset + 1)
        assert command[offset : offset + 3] == ["--setenv", name, environment[name]]
    assert command[-4:] == [
        str(launcher),
        "-windowed",
        "-ResX=1600",
        "-ResY=900",
    ]
    assert environment["DISPLAY"] == ":1"
    assert environment["XDG_CONFIG_HOME"] == str(tmp_path / "xdg" / "config")
    assert environment["XDG_CACHE_HOME"] == str(tmp_path / "xdg" / "cache")
    assert environment["XDG_DATA_HOME"] == str(tmp_path / "xdg" / "data")
    for name in ("HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "http_proxy", "https_proxy", "all_proxy"):
        assert name not in environment


def test_launch_isolated_viewer_returns_immediately_with_an_exclusive_log(
    tmp_path: Path,
) -> None:
    """Dashboard 点击启动不得阻塞 Qt 线程，且本次 Viewer 日志必须排他保留。"""
    viewer_root = tmp_path / "Viewer2_2.6.0_Linux"
    launcher = viewer_root / "LivoxViewer2.sh"
    launcher.parent.mkdir(parents=True)
    launcher.write_text("#!/bin/sh\n", encoding="utf-8")
    calls = []

    class FakeProcess:
        pid = 123

    def popen(command, env, **kwargs):
        calls.append((command, env, kwargs))
        return FakeProcess()

    pid, log_path = verifier.launch_isolated_viewer(
        viewer_root=viewer_root,
        display=":1",
        xdg_root=tmp_path / "xdg",
        launch_dir=tmp_path / "launch",
        popen_factory=popen,
    )

    assert pid == 123
    assert log_path == tmp_path / "launch" / "launcher.log"
    assert log_path.is_file()
    assert calls[0][2]["start_new_session"] is True
    assert calls[0][2]["stdout"].name == str(log_path)
    assert calls[0][2]["stderr"] is verifier.subprocess.STDOUT


def test_launch_isolated_viewer_rejects_missing_bwrap_before_popen(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """库调用不能绕过 CLI 的 bubblewrap 前置门禁或泄漏裸 FileNotFoundError。"""
    viewer_root = tmp_path / "Viewer2_2.6.0_Linux"
    launcher = viewer_root / "LivoxViewer2.sh"
    launcher.parent.mkdir(parents=True)
    launcher.write_text("#!/bin/sh\n", encoding="utf-8")
    monkeypatch.setattr(verifier.shutil, "which", lambda _name: None)

    with pytest.raises(
        RuntimeError,
        match=r"bubblewrap \(bwrap\) is required for isolated Livox Viewer startup",
    ):
        verifier.launch_isolated_viewer(
            viewer_root=viewer_root,
            display=":1",
            xdg_root=tmp_path / "xdg",
            launch_dir=tmp_path / "launch",
            popen_factory=lambda *_args, **_kwargs: pytest.fail(
                "Popen must not run without bwrap"
            ),
        )


def test_import_lvx2_rejects_missing_bwrap_before_custom_launch(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Dashboard 高层入口自身也必须 fail-fast，而非依赖下层启动器偶然报错。"""
    selected = tmp_path / "lidar.lvx2"
    selected.write_bytes(b"lvx2")
    monkeypatch.setattr(verifier.shutil, "which", lambda _name: None)

    with pytest.raises(
        RuntimeError,
        match=r"bubblewrap \(bwrap\) is required for isolated Livox Viewer startup",
    ):
        verifier.import_lvx2_in_livox_viewer(
            selected,
            viewer_root=tmp_path / "Viewer",
            display=":1",
            xdg_root=tmp_path / "xdg",
            launch_dir=tmp_path / "launch",
            launch=lambda **_kwargs: pytest.fail("launch must not run without bwrap"),
        )


def test_viewer_lvx2_marker_requires_the_exact_absolute_file(tmp_path: Path) -> None:
    """同名历史文件的成功日志不能冒充本次 Dashboard 选择的 LVX2。"""
    selected = tmp_path / "selected" / "lidar.lvx2"
    selected.parent.mkdir()
    selected.write_bytes(b"lvx2")
    other = tmp_path / "old" / "lidar.lvx2"
    log_path = tmp_path / "launcher.log"
    log_path.write_text(
        f"Livox: ALvxKit::OpenLvxFile({other}) success, Frame num = 1\n",
        encoding="utf-8",
    )

    assert verifier.viewer_lvx2_opened(log_path, selected) is False

    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(
            f"Livox: ALvxKit::OpenLvxFile({selected}) success, Frame num = 1\n"
        )
    assert verifier.viewer_lvx2_opened(log_path, selected) is True


def test_import_lvx2_uses_env_viewer_and_waits_for_exact_open_log(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """启动成功不等于导入成功；高层入口必须等到精确文件的官方成功日志。"""
    viewer_root = tmp_path / "Viewer2_2.6.0_Linux"
    launcher = viewer_root / "LivoxViewer2.sh"
    launcher.parent.mkdir(parents=True)
    launcher.write_text("#!/bin/sh\n", encoding="utf-8")
    selected = tmp_path / "capture" / "export" / "lidar.lvx2"
    selected.parent.mkdir(parents=True)
    selected.write_bytes(b"lvx2")
    launch_dir = tmp_path / "launch"
    log_path = launch_dir / "launcher.log"
    calls: list[object] = []

    monkeypatch.setenv("SLOPE_SIM_LIVOX_VIEWER_ROOT", str(viewer_root))

    def launch(**kwargs):
        calls.append(("launch", kwargs))
        launch_dir.mkdir()
        log_path.write_text("Viewer process started\n", encoding="utf-8")
        return 123, log_path

    def automate(**kwargs):
        calls.append(("automate", kwargs))
        with log_path.open("a", encoding="utf-8") as handle:
            handle.write(
                "Livox: ALvxKit::OpenLvxFile(/tmp/old/lidar.lvx2) success, "
                "Frame num = 1\n"
            )

    sleeps: list[float] = []

    def sleep(seconds: float) -> None:
        sleeps.append(seconds)
        with log_path.open("a", encoding="utf-8") as handle:
            handle.write(
                f"Livox: ALvxKit::OpenLvxFile({selected}) success, Frame num = 1\n"
            )

    pid, actual_log = verifier.import_lvx2_in_livox_viewer(
        selected,
        display=":1",
        xdg_root=tmp_path / "xdg",
        launch_dir=launch_dir,
        timeout_sec=2.0,
        launch=launch,
        automation=automate,
        sleep=sleep,
    )

    assert (pid, actual_log) == (123, log_path)
    assert calls[0][0] == "launch"
    assert calls[0][1]["viewer_root"] == viewer_root
    assert calls[1] == (
        "automate",
        {
            "launcher_pid": 123,
            "lvx2_path": selected,
            "log_path": log_path,
            "display": ":1",
            "timeout_sec": 2.0,
        },
    )
    assert sleeps == [0.05]


def test_x11_automation_selects_the_exact_absolute_lvx2_path_in_unreal_modal(tmp_path: Path) -> None:
    """Unreal 内嵌文件框没有独立 X11 窗口，仍须自动填入并提交路径。"""
    selected = tmp_path / "capture" / "lidar.lvx2"
    selected.parent.mkdir()
    selected.write_bytes(b"lvx2")
    log_path = tmp_path / "launcher.log"
    log_path.write_text(
        "LogLoad: Took 0.426 seconds to LoadMap(/Game/Maps/Viewer)\n",
        encoding="utf-8",
    )
    commands: list[list[str]] = []

    def run_command(command, **_kwargs):
        commands.append(command)
        if command[1:7] == [
            "search", "--all", "--onlyvisible", "--pid", "321", "--name",
        ]:
            if command[-1] == "^LivoxViewer":
                return SimpleNamespace(returncode=0, stdout="200\n", stderr="")
        if command[1:4] == ["getwindowgeometry", "--shell", "200"]:
            return SimpleNamespace(
                returncode=0,
                stdout="WINDOW=200\nX=216\nY=196\nWIDTH=1600\nHEIGHT=900\n",
                stderr="",
            )
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    verifier.automate_lvx2_file_selection(
        launcher_pid=123,
        lvx2_path=selected,
        log_path=log_path,
        display=":1",
        timeout_sec=2.0,
        proc_root=tmp_path / "proc",
        isolated_lookup=lambda **_kwargs: (321, {}),
        run_command=run_command,
        sleep=lambda _seconds: None,
    )

    assert ["xdotool", "windowactivate", "--sync", "200"] in commands
    assert ["xdotool", "mousemove", "--window", "200", "230", "18", "click", "1"] in commands
    assert not any(command[-1] == "^Open File$" for command in commands)
    assert [
        "xdotool", "mousemove", "--window", "200", "924", "538", "click", "1",
    ] in commands
    assert ["xdotool", "key", "--window", "200", "ctrl+a"] in commands
    assert [
        "xdotool", "type", "--window", "200", "--clearmodifiers", "--delay", "5",
        str(selected),
    ] in commands
    assert ["xdotool", "mousemove", "--window", "200", "1022", "572", "click", "1"] in commands


def test_x11_automation_reports_a_missing_xdotool_clearly(tmp_path: Path) -> None:
    """桌面自动化依赖缺失时应给 Dashboard 可直接展示的错误，而非裸 OSError。"""
    selected = tmp_path / "lidar.lvx2"
    selected.write_bytes(b"lvx2")
    log_path = tmp_path / "launcher.log"
    log_path.write_text(
        "LogLoad: Took 0.426 seconds to LoadMap(/Game/Maps/Viewer)\n",
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="xdotool is required"):
        verifier.automate_lvx2_file_selection(
            launcher_pid=123,
            lvx2_path=selected,
            log_path=log_path,
            display=":1",
            timeout_sec=2.0,
            proc_root=tmp_path / "proc",
            isolated_lookup=lambda **_kwargs: (321, {}),
            run_command=lambda *_args, **_kwargs: (_ for _ in ()).throw(
                FileNotFoundError("xdotool")
            ),
            sleep=lambda _seconds: None,
        )


def test_network_evidence_records_a_distinct_namespace_with_only_loopback(
    tmp_path: Path,
) -> None:
    """通过的显示 smoke 必须能独立复核 Viewer 没有外部网络接口。"""
    proc_root = tmp_path / "proc"
    for pid, parent_pid, process_group, command, namespace, interfaces in (
        (100, 1, 100, "python", "net:[4026531833]", ("lo", "eth0")),
        (101, 100, 101, "bwrap", "net:[4026532666]", ("lo",)),
        (102, 1, 101, "sh", "net:[4026532666]", ("lo",)),
        (103, 102, 101, "LivoxViewer2", "net:[4026532666]", ("lo",)),
    ):
        process_root = proc_root / str(pid)
        namespace_path = process_root / "ns" / "net"
        namespace_path.parent.mkdir(parents=True)
        namespace_path.symlink_to(namespace)
        net_dir = process_root / "net"
        net_dir.mkdir()
        devices = "".join(f"  {name}: 0 0 0 0\\n" for name in interfaces)
        (net_dir / "dev").write_text(devices, encoding="utf-8")
        (process_root / "stat").write_text(
            f"{pid} ({command}) S {parent_pid} 0 0 0\\n", encoding="utf-8"
        )

    evidence = verifier.collect_network_evidence(
        host_pid=100,
        viewer_pid=103,
        proc_root=proc_root,
    )

    assert evidence == {
        "host_network_namespace": "net:[4026531833]",
        "viewer_network_namespace": "net:[4026532666]",
        "viewer_interfaces": ["lo"],
        "network_isolated": True,
        "no_external_interfaces": True,
    }


def test_write_network_evidence_persists_the_actual_launcher_and_namespace(
    tmp_path: Path,
) -> None:
    """真实 smoke 必须排他记录实际启动 argv 与网络隔离状态。"""
    payload = {
        "bwrap_command": ["bwrap", "--unshare-net", "/tmp/Viewer/LivoxViewer2.sh"],
        "host_network_namespace": "net:[4026531833]",
        "viewer_network_namespace": "net:[4026532666]",
        "viewer_interfaces": ["lo"],
        "viewer_pid": 101,
        "network_isolated": True,
        "no_external_interfaces": True,
    }

    path = verifier.write_network_evidence(tmp_path, payload)

    assert path == tmp_path / "network-evidence.json"
    assert json.loads(path.read_text(encoding="utf-8")) == payload


def test_viewer_map_marker_requires_the_real_viewer_map(tmp_path: Path) -> None:
    """网络隔离本身不能替代 Viewer 主地图已加载的显示 smoke。"""
    log_path = tmp_path / "launcher.log"
    log_path.write_text("LogLoad: LoadMap: /Game/Maps/Start\n", encoding="utf-8")

    assert verifier.viewer_map_loaded(log_path) is False

    log_path.write_text(
        "LogLoad: Took 0.426 seconds to LoadMap(/Game/Maps/Viewer)\n",
        encoding="utf-8",
    )
    assert verifier.viewer_map_loaded(log_path) is True


def test_viewer_cli_uses_a_bounded_ninety_second_startup_timeout() -> None:
    """UE Viewer 初次 GPU 着色器加载可超过一分钟，默认预算仍必须有限。"""
    args = verifier._parse_args(
        [
            "--viewer-root", "/tmp/Viewer",
            "--xdg-root", "/tmp/xdg",
            "--evidence-dir", "/tmp/evidence",
        ]
    )

    assert args.startup_timeout_sec == 90.0


def test_isolated_viewer_lookup_rejects_another_process_group(tmp_path: Path) -> None:
    """不得把并发启动的同名 Viewer 误记为本次 launcher 的证据。"""
    proc_root = tmp_path / "proc"
    for pid, parent_pid, process_group, command, namespace, interfaces in (
        (100, 1, 100, "python", "net:[4026531833]", ("lo", "eth0")),
        (101, 100, 101, "bwrap", "net:[4026532666]", ("lo",)),
        (103, 1, 999, "LivoxViewer2", "net:[4026532666]", ("lo",)),
    ):
        process_root = proc_root / str(pid)
        namespace_path = process_root / "ns" / "net"
        namespace_path.parent.mkdir(parents=True)
        namespace_path.symlink_to(namespace)
        net_dir = process_root / "net"
        net_dir.mkdir()
        (net_dir / "dev").write_text(
            "".join(f"  {name}: 0 0 0 0\n" for name in interfaces),
            encoding="utf-8",
        )
        (process_root / "stat").write_text(
            f"{pid} ({command}) S {parent_pid} {process_group} 0 0\n",
            encoding="utf-8",
        )
        (process_root / "comm").write_text(f"{command}\n", encoding="utf-8")

    assert verifier._isolated_viewer_pid(
        launcher_pid=101,
        host_pid=100,
        proc_root=proc_root,
    ) is None


def test_run_isolated_viewer_writes_verified_network_evidence_before_waiting(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """启动器只能在 bwrap 进程已进入 loopback-only namespace 后记为 smoke。"""
    viewer_root = tmp_path / "Viewer2_2.6.0_Linux"
    launcher = viewer_root / "LivoxViewer2.sh"
    launcher.parent.mkdir(parents=True)
    launcher.write_text("#!/bin/sh\n", encoding="utf-8")
    proc_root = tmp_path / "proc"
    for pid, parent_pid, process_group, command, namespace, interfaces in (
        (100, 1, 100, "python", "net:[4026531833]", ("lo", "eth0")),
        (101, 100, 101, "bwrap", "net:[4026532666]", ("lo",)),
        (102, 1, 101, "sh", "net:[4026532666]", ("lo",)),
        (103, 102, 101, "LivoxViewer2", "net:[4026532666]", ("lo",)),
    ):
        process_root = proc_root / str(pid)
        namespace_path = process_root / "ns" / "net"
        namespace_path.parent.mkdir(parents=True)
        namespace_path.symlink_to(namespace)
        net_dir = process_root / "net"
        net_dir.mkdir()
        (net_dir / "dev").write_text(
            "".join(f"  {name}: 0 0 0 0\n" for name in interfaces),
            encoding="utf-8",
        )
        (process_root / "stat").write_text(
            f"{pid} ({command}) S {parent_pid} {process_group} 0 0\n",
            encoding="utf-8",
        )
        (process_root / "comm").write_text(f"{command}\n", encoding="utf-8")

    calls = []

    class FakeProcess:
        pid = 101

        def wait(self) -> int:
            calls.append("wait")
            return 0

        def poll(self):
            return None

    monkeypatch.setattr(verifier.os, "getpid", lambda: 100)
    evidence_dir = tmp_path / "evidence"
    def popen(command, env, **kwargs):
        calls.append((command, env))
        assert kwargs["start_new_session"] is True
        (evidence_dir / "launcher.log").write_text(
            "LogLoad: Took 0.426 seconds to LoadMap(/Game/Maps/Viewer)\n",
            encoding="utf-8",
        )
        return FakeProcess()

    result = verifier.run_isolated_viewer(
        viewer_root=viewer_root,
        display=":1",
        xdg_root=tmp_path / "xdg",
        evidence_dir=evidence_dir,
        popen_factory=popen,
        proc_root=proc_root,
        sleep=lambda _seconds: None,
    )

    assert result == 0
    assert calls[-1] == "wait"
    evidence = json.loads((evidence_dir / "network-evidence.json").read_text())
    assert evidence["viewer_pid"] == 103
    assert evidence["launcher_pid"] == 101
    assert evidence["launcher_pgid"] == 101
    assert evidence["viewer_pgid"] == 101
    assert evidence["network_isolated"] is True
    assert evidence["no_external_interfaces"] is True
    assert evidence["bwrap_command"][:2] == ["bwrap", "--unshare-net"]


def test_run_isolated_viewer_waits_for_viewer_map_before_writing_evidence(
    tmp_path: Path,
    monkeypatch,
) -> None:
    """隔离成功后仍须等官方日志确认 Viewer 地图，不能提前 PASS。"""
    viewer_root = tmp_path / "Viewer2_2.6.0_Linux"
    launcher = viewer_root / "LivoxViewer2.sh"
    launcher.parent.mkdir(parents=True)
    launcher.write_text("#!/bin/sh\n", encoding="utf-8")
    proc_root = tmp_path / "proc"
    for pid, parent_pid, process_group, command, namespace, interfaces in (
        (100, 1, 100, "python", "net:[4026531833]", ("lo", "eth0")),
        (101, 100, 101, "bwrap", "net:[4026532666]", ("lo",)),
        (102, 1, 101, "sh", "net:[4026532666]", ("lo",)),
        (103, 102, 101, "LivoxViewer2", "net:[4026532666]", ("lo",)),
    ):
        process_root = proc_root / str(pid)
        namespace_path = process_root / "ns" / "net"
        namespace_path.parent.mkdir(parents=True)
        namespace_path.symlink_to(namespace)
        net_dir = process_root / "net"
        net_dir.mkdir()
        (net_dir / "dev").write_text(
            "".join(f"  {name}: 0 0 0 0\n" for name in interfaces),
            encoding="utf-8",
        )
        (process_root / "stat").write_text(
            f"{pid} ({command}) S {parent_pid} {process_group} 0 0\n",
            encoding="utf-8",
        )
        (process_root / "comm").write_text(f"{command}\n", encoding="utf-8")

    evidence_dir = tmp_path / "evidence"
    sleep_calls = []

    class FakeProcess:
        pid = 101

        def wait(self) -> int:
            return 0

        def poll(self):
            return None

    def popen(command, env, **kwargs):
        kwargs["stdout"].write(
            "LogLoad: Took 0.426 seconds to LoadMap(/Game/Maps/Viewer)\n"
        )
        return FakeProcess()

    monkeypatch.setattr(verifier.os, "getpid", lambda: 100)
    assert verifier.run_isolated_viewer(
        viewer_root=viewer_root,
        display=":1",
        xdg_root=tmp_path / "xdg",
        evidence_dir=evidence_dir,
        popen_factory=popen,
        proc_root=proc_root,
        sleep=lambda _seconds: sleep_calls.append(_seconds),
    ) == 0
    assert sleep_calls == []
    evidence = json.loads((evidence_dir / "network-evidence.json").read_text())
    assert evidence["viewer_map_loaded"] is True
