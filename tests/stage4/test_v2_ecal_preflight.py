"""runSim v2：eCAL 启动预检的可测试合同。"""
from __future__ import annotations

from pathlib import Path
import types

import pytest


def _module():
    from slope_sim.interfaces.v2 import ecal_preflight

    return ecal_preflight


def _fixture_files(tmp_path: Path) -> tuple[Path, Path, Path]:
    data_dir = tmp_path / "ecal-data"
    data_dir.mkdir()
    config_path = data_dir / "ecal.yaml"
    config_path.write_text("version: 1\n", encoding="utf-8")
    plugin_dir = tmp_path / "plugins"
    plugin_dir.mkdir()
    (plugin_dir / "libecaltime-localtime.so").write_bytes(b"plugin")
    descriptor_path = tmp_path / "slope_sim_interfaces_v2.desc"
    descriptor_path.write_bytes(b"descriptor")
    return config_path, plugin_dir, descriptor_path


def test_v2_preflight_reports_complete_capability_without_initializing_participant(
    tmp_path: Path,
) -> None:
    """完整文件/API 能力应通过，预检不得调用 eCAL initialize。"""
    module = _module()
    config_path, plugin_dir, descriptor_path = _fixture_files(tmp_path)
    calls: list[str] = []

    class FakeCore:
        Publisher = object
        Subscriber = object
        DataTypeInformation = object
        monitoring = object()

        @staticmethod
        def initialize(*_args: object) -> bool:
            calls.append("initialize")
            return True

        @staticmethod
        def finalize() -> bool:
            calls.append("finalize")
            return True

    report = module.run_v2_ecal_preflight(
        environment={
            "ECAL_CONFIG_PATH": str(config_path),
            "ECAL_TIME_PLUGIN_PATH": str(plugin_dir),
        },
        descriptor_path=descriptor_path,
        manifest_path=None,
        core_loader=lambda: FakeCore,
    )

    assert report.ok is True
    assert report.config_path == config_path.resolve()
    assert report.time_sync_plugin_path == (plugin_dir / "libecaltime-localtime.so").resolve()
    assert report.participant_available is True
    assert report.peer_available is None
    assert calls == []


def test_v2_preflight_has_stable_terminal_and_dashboard_failure_details(
    tmp_path: Path,
) -> None:
    """配置、time-sync、descriptor 和 participant 缺失都必须可操作地呈现。"""
    module = _module()
    report = module.run_v2_ecal_preflight(
        environment={
            "ECAL_CONFIG_PATH": str(tmp_path / "missing.yaml"),
            "ECAL_TIME_PLUGIN_PATH": str(tmp_path / "missing-plugins"),
        },
        descriptor_path=tmp_path / "missing.desc",
        manifest_path=None,
        core_loader=lambda: (_ for _ in ()).throw(ImportError("raw core missing")),
    )

    assert report.ok is False
    codes = {issue.code for issue in report.issues}
    assert codes == {
        "ecal_config_missing",
        "ecal_time_sync_plugin_missing",
        "v2_descriptor_missing",
        "ecal_participant_api_missing",
    }
    terminal = report.format_terminal()
    dashboard = report.format_dashboard()
    assert "eCAL preflight failed" in terminal
    assert "eCAL preflight failed" in dashboard
    for code in sorted(codes):
        assert code in terminal
        assert code in dashboard


def test_v2_peer_preflight_rejects_missing_or_unverified_topics() -> None:
    """运行期 peer 快照缺失不能伪装成 eCAL 已连接。"""
    module = _module()
    report = module.evaluate_v2_peer_snapshot(
        types.SimpleNamespace(
            ecal_connected=True,
            topic_quality=(),
        )
    )

    assert report.ok is False
    assert [issue.code for issue in report.issues] == ["v2_peer_missing"]
    assert "peer" in report.format_dashboard().lower()


@pytest.mark.parametrize("mode", ("local", "auto"))
def test_v2_run_mode_rejects_legacy_transport(mode: str) -> None:
    """正式 runSim v2 不允许 local/auto 静默落入 v1。"""
    module = _module()

    with pytest.raises(module.LegacyInterfaceModeError, match="legacy"):
        module.require_v2_interface_mode(mode)

    assert module.require_v2_interface_mode("ecal") == "ecal"


@pytest.mark.parametrize("mode", ("local", "auto"))
def test_main_blocks_legacy_manual_mode_before_gui(
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
) -> None:
    """正式手动入口不能因缺少隐藏 v2 标记而落入 v1 transport。"""
    import main

    called = False

    def fail_if_called(**_kwargs: object) -> object:
        nonlocal called
        called = True
        raise AssertionError("legacy mode must stop before preflight")

    monkeypatch.setattr(main, "run_v2_ecal_preflight", fail_if_called)
    monkeypatch.setattr(
        main,
        "run_manual_demo",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("legacy mode must stop before GUI")
        ),
    )
    assert main.main(["--gui", "--manual", "--interface-mode", mode]) == 2
    assert called is False


def test_main_surfaces_ecal_preflight_failure_in_dashboard_and_terminal(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """预检失败必须显示 Dashboard 诊断，同时保持终端失败文本。"""
    import main

    module = _module()
    failure = module.EcalPreflightReport(
        None,
        None,
        None,
        False,
        None,
        (module.EcalPreflightIssue("ecal_config_missing", "configure ECAL_CONFIG_PATH"),),
    )
    monkeypatch.setattr(main, "run_v2_ecal_preflight", lambda **_kwargs: failure)
    dashboard_messages: list[str] = []
    monkeypatch.setattr(
        main,
        "show_v2_preflight_failure_dashboard",
        lambda report: dashboard_messages.append(report.format_dashboard()),
    )

    assert main.main(["--manual", "--interface-mode", "ecal"]) == 2
    assert dashboard_messages == [failure.format_dashboard()]
    assert failure.format_terminal() in capsys.readouterr().err


def test_main_applies_explicit_ecal_paths_to_the_preflight_and_v2_runtime(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """CLI eCAL 覆盖不能只让预检通过，实际 participant 必须继承同一环境。"""
    import main

    config_path, plugin_dir, _descriptor_path = _fixture_files(tmp_path)
    module = _module()
    success = module.EcalPreflightReport(
        config_path,
        plugin_dir / "libecaltime-localtime.so",
        "descriptor",
        True,
        None,
        (),
    )
    preflight_environments: list[dict[str, str]] = []
    monkeypatch.setattr(
        main,
        "run_v2_ecal_preflight",
        lambda **kwargs: (
            preflight_environments.append(dict(kwargs["environment"])), success
        )[1],
    )

    class Command:
        session_id_factory = staticmethod(lambda: b"s" * 16)
        client = object()

        def close(self) -> None:
            pass

    monkeypatch.setattr(
        main.RunSimV2Command,
        "launch",
        lambda **_kwargs: Command(),
    )
    observed_runtime_environment: list[tuple[str | None, str | None]] = []
    monkeypatch.setattr(
        main,
        "run_manual_demo",
        lambda *_args, **_kwargs: (
            observed_runtime_environment.append(
                (
                    main.os.environ.get("ECAL_CONFIG_PATH"),
                    main.os.environ.get("ECAL_TIME_PLUGIN_PATH"),
                )
            ),
            types.SimpleNamespace(
                log_path=tmp_path / "log.csv",
                figure_path=tmp_path / "figure.png",
                feedback_figure_paths=(),
                diagnostic_summary_path=None,
                obstacle_event_log_path=None,
                interface_binary_log=None,
                interface_event_log=None,
                scene_export=None,
                metrics={},
                diagnostic_summary=None,
            ),
        )[1],
    )
    monkeypatch.delenv("ECAL_CONFIG_PATH", raising=False)
    monkeypatch.delenv("ECAL_TIME_PLUGIN_PATH", raising=False)

    assert main.main([
        "--gui", "--manual", "--interface-mode", "ecal",
        "--ecal-config", str(config_path),
        "--ecal-time-plugin-path", str(plugin_dir),
    ]) == 0

    expected = (str(config_path.resolve()), str(plugin_dir.resolve()))
    assert observed_runtime_environment == [expected]
    assert (
        preflight_environments[0]["ECAL_CONFIG_PATH"],
        preflight_environments[0]["ECAL_TIME_PLUGIN_PATH"],
    ) == expected


def test_main_uses_explicit_v2_runtime_root_for_development_acceptance(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """源码验收可从受控运行根取得 Command、配置与 localtime 插件。"""
    import main

    runtime_root = tmp_path / "acceptance-runtime"
    (runtime_root / "bin").mkdir(parents=True)
    (runtime_root / "etc" / "ecal").mkdir(parents=True)
    (runtime_root / "lib").mkdir()
    (runtime_root / "etc" / "ecal" / "ecal.yaml").write_text("# eCAL\n", encoding="utf-8")
    (runtime_root / "lib" / "libecaltime-localtime.so").write_bytes(b"plugin")
    monkeypatch.setenv("SLOPE_SIM_V2_RUNTIME_ROOT", str(runtime_root))
    module = _module()
    success = module.EcalPreflightReport(
        runtime_root / "etc" / "ecal" / "ecal.yaml",
        runtime_root / "lib" / "libecaltime-localtime.so",
        "descriptor",
        True,
        None,
        (),
    )
    preflight_environments: list[dict[str, str]] = []
    monkeypatch.setattr(
        main,
        "run_v2_ecal_preflight",
        lambda **kwargs: (preflight_environments.append(dict(kwargs["environment"])), success)[1],
    )
    launch_roots: list[Path] = []

    shutdown_trace: list[str] = []

    class Command:
        session_id_factory = staticmethod(lambda: b"s" * 16)
        client = object()

        def close(self) -> None:
            shutdown_trace.append("command.close")

    monkeypatch.setattr(
        main.RunSimV2Command,
        "launch",
        lambda **kwargs: (launch_roots.append(kwargs["release_root"]), Command())[1],
    )
    captured_roots: list[Path] = []
    def fake_run_manual_demo(*_args, **kwargs):
        captured_roots.append(kwargs["v2_capture_release_root"])
        kwargs["v2_command_shutdown"]()
        shutdown_trace.append("runtime.close")
        return types.SimpleNamespace(
            log_path=tmp_path / "log.csv",
            figure_path=tmp_path / "figure.png",
            feedback_figure_paths=(),
            diagnostic_summary_path=None,
            obstacle_event_log_path=None,
            interface_binary_log=None,
            interface_event_log=None,
            scene_export=None,
            metrics={},
            diagnostic_summary=None,
        )

    monkeypatch.setattr(main, "run_manual_demo", fake_run_manual_demo)

    assert main.main(["--gui", "--manual", "--interface-mode", "ecal"]) == 0

    expected_root = runtime_root.resolve()
    assert launch_roots == [expected_root]
    assert captured_roots == [expected_root]
    assert preflight_environments[0]["ECAL_DATA"] == str(expected_root / "etc" / "ecal")
    assert preflight_environments[0]["ECAL_TIME_PLUGIN_PATH"] == str(expected_root / "lib")
    assert shutdown_trace == ["command.close", "runtime.close"]


def test_main_forwards_v2_capture_and_viewer_paths_to_the_gui_runtime(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """正式 CLI 的采集目录和本地 Viewer 根目录必须进入同一 GUI v2 会话。"""
    import main

    config_path, plugin_dir, _descriptor_path = _fixture_files(tmp_path)
    capture_root = tmp_path / "captures"
    viewer_root = tmp_path / "LivoxViewer2"
    module = _module()
    monkeypatch.setattr(
        main,
        "run_v2_ecal_preflight",
        lambda **_kwargs: module.EcalPreflightReport(
            config_path,
            plugin_dir / "libecaltime-localtime.so",
            "descriptor",
            True,
            None,
            (),
        ),
    )

    class Command:
        session_id_factory = staticmethod(lambda: b"s" * 16)
        client = object()

        def close(self) -> None:
            pass

    monkeypatch.setattr(main.RunSimV2Command, "launch", lambda **_kwargs: Command())
    runtime_kwargs: list[dict[str, object]] = []
    monkeypatch.setattr(
        main,
        "run_manual_demo",
        lambda *_args, **kwargs: (
            runtime_kwargs.append(kwargs),
            types.SimpleNamespace(
                log_path=tmp_path / "log.csv",
                figure_path=tmp_path / "figure.png",
                feedback_figure_paths=(),
                diagnostic_summary_path=None,
                obstacle_event_log_path=None,
                interface_binary_log=None,
                interface_event_log=None,
                scene_export=None,
                metrics={},
                diagnostic_summary=None,
            ),
        )[1],
    )

    assert main.main([
        "--gui", "--manual", "--interface-mode", "ecal",
        "--capture-output-dir", str(capture_root),
        "--viewer-root", str(viewer_root),
        "--capture-duration-sec", "90", "--open-ros-rviz",
    ]) == 0

    assert runtime_kwargs[0]["v2_capture_output_root"] == capture_root.resolve()
    assert runtime_kwargs[0]["v2_viewer_root"] == viewer_root.resolve()
    assert runtime_kwargs[0]["v2_capture_duration_sec"] == 90
    assert runtime_kwargs[0]["v2_open_live_viewer"] is True


def test_main_restores_explicit_ecal_paths_when_command_startup_is_blocked(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Command 未启动时也不能污染调用进程随后创建的 eCAL 会话。"""
    import main

    config_path, plugin_dir, _descriptor_path = _fixture_files(tmp_path)
    module = _module()
    monkeypatch.setattr(
        main,
        "run_v2_ecal_preflight",
        lambda **_kwargs: module.EcalPreflightReport(
            config_path,
            plugin_dir / "libecaltime-localtime.so",
            "descriptor",
            True,
            None,
            (),
        ),
    )
    monkeypatch.setattr(
        main.RunSimV2Command,
        "launch",
        lambda **_kwargs: (_ for _ in ()).throw(RuntimeError("missing command")),
    )
    monkeypatch.delenv("ECAL_CONFIG_PATH", raising=False)
    monkeypatch.delenv("ECAL_TIME_PLUGIN_PATH", raising=False)

    assert main.main([
        "--gui", "--manual", "--interface-mode", "ecal",
        "--ecal-config", str(config_path),
        "--ecal-time-plugin-path", str(plugin_dir),
    ]) == 2
    assert "ECAL_CONFIG_PATH" not in main.os.environ
    assert "ECAL_TIME_PLUGIN_PATH" not in main.os.environ


def test_main_auto_discovers_rc_nonfatally_when_no_port_qualifies(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """无参数 runSim 自动扫描 RC，无合格端口时仍进入键盘模式。"""
    import main

    config_path, plugin_dir, _descriptor_path = _fixture_files(tmp_path)
    module = _module()
    monkeypatch.setattr(
        main,
        "run_v2_ecal_preflight",
        lambda **_kwargs: module.EcalPreflightReport(
            config_path,
            plugin_dir / "libecaltime-localtime.so",
            "descriptor",
            True,
            None,
            (),
        ),
    )

    class Client:
        def send_target(self, _linear, _angular, *, now):
            assert isinstance(now, float)

    class Command:
        session_id_factory = staticmethod(lambda: b"s" * 16)
        client = Client()
        process_pid = 123

        def close(self) -> None:
            pass

    monkeypatch.setattr(main.RunSimV2Command, "launch", lambda **_kwargs: Command())
    attempts: list[Path | None] = []
    monkeypatch.setattr(
        main,
        "start_rc_worker",
        lambda **kwargs: (
            attempts.append(kwargs["explicit_path"]),
            (_ for _ in ()).throw(RuntimeError("no qualified RC SBUS port")),
        )[1],
    )
    runtime_workers: list[object | None] = []
    monkeypatch.setattr(
        main,
        "run_manual_demo",
        lambda *_args, **kwargs: (
            runtime_workers.append(kwargs["rc_worker"]),
            types.SimpleNamespace(
                log_path=tmp_path / "log.csv",
                figure_path=tmp_path / "figure.png",
                feedback_figure_paths=(),
                diagnostic_summary_path=None,
                obstacle_event_log_path=None,
                interface_binary_log=None,
                interface_event_log=None,
                scene_export=None,
                metrics={},
                diagnostic_summary=None,
            ),
        )[1],
    )

    assert main.main(["--gui", "--manual", "--interface-mode", "ecal"]) == 0
    assert attempts == [None]
    assert runtime_workers == [None]
    assert "RC auto-discovery unavailable" in capsys.readouterr().err


def test_main_closes_command_even_when_arbiter_final_zero_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """最终 socket 归零失败也不能跳过受监管 C++ Command 的回收。"""
    import main

    config_path, plugin_dir, _descriptor_path = _fixture_files(tmp_path)
    module = _module()
    monkeypatch.setattr(
        main,
        "run_v2_ecal_preflight",
        lambda **_kwargs: module.EcalPreflightReport(
            config_path,
            plugin_dir / "libecaltime-localtime.so",
            "descriptor",
            True,
            None,
            (),
        ),
    )
    closed: list[str] = []
    arbiter_options: list[dict[str, object]] = []

    class Client:
        @staticmethod
        def send_target(_linear, _angular, *, now):
            assert isinstance(now, float)

    class Command:
        session_id_factory = staticmethod(lambda: b"s" * 16)
        client = Client()
        process_pid = 123

        @staticmethod
        def close() -> None:
            closed.append("command")

    class Arbiter:
        def __init__(self, _client, **kwargs) -> None:
            arbiter_options.append(kwargs)

        @staticmethod
        def close(*, now=None) -> None:
            raise RuntimeError("arbiter close failed")

    monkeypatch.setattr(main.RunSimV2Command, "launch", lambda **_kwargs: Command())
    monkeypatch.setattr(main, "CommandSourceArbiter", Arbiter)
    monkeypatch.setattr(main, "start_rc_worker", lambda **_kwargs: None)
    monkeypatch.setattr(
        main,
        "run_manual_demo",
        lambda *_args, **_kwargs: types.SimpleNamespace(
            log_path=tmp_path / "log.csv",
            figure_path=tmp_path / "figure.png",
            feedback_figure_paths=(),
            diagnostic_summary_path=None,
            obstacle_event_log_path=None,
            interface_binary_log=None,
            interface_event_log=None,
            scene_export=None,
            metrics={},
            diagnostic_summary=None,
        ),
    )

    with pytest.raises(RuntimeError, match="arbiter close failed"):
        main.main(["--gui", "--manual", "--interface-mode", "ecal"])

    assert arbiter_options == [{}]
    assert closed == ["command"]


def test_main_explicit_rc_eio_blocks_startup_with_bounded_diagnostic(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """显式 --rc-port 的 EIO 必须在 GUI 启动前有界失败并保留原因。"""
    import main

    config_path, plugin_dir, _descriptor_path = _fixture_files(tmp_path)
    module = _module()
    monkeypatch.setattr(
        main,
        "run_v2_ecal_preflight",
        lambda **_kwargs: module.EcalPreflightReport(
            config_path,
            plugin_dir / "libecaltime-localtime.so",
            "descriptor",
            True,
            None,
            (),
        ),
    )

    class Client:
        def send_target(self, _linear, _angular, *, now):
            assert isinstance(now, float)

    class Command:
        session_id_factory = staticmethod(lambda: b"s" * 16)
        client = Client()

        def close(self) -> None:
            pass

    monkeypatch.setattr(main.RunSimV2Command, "launch", lambda **_kwargs: Command())
    monkeypatch.setattr(
        main,
        "start_rc_worker",
        lambda **_kwargs: (_ for _ in ()).throw(OSError(5, "Input/output error")),
    )
    manual_started = []
    monkeypatch.setattr(main, "run_manual_demo", lambda *_args, **_kwargs: manual_started.append(True))

    assert main.main([
        "--gui",
        "--manual",
        "--interface-mode",
        "ecal",
        "--rc-port",
        "/dev/serial/by-id/usb-test",
    ]) == 2
    assert manual_started == []
    assert "Input/output error" in capsys.readouterr().err
