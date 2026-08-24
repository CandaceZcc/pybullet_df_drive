"""runSim v2：正式 C++ Command 启动参数与 Python session identity 合同。"""
from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace


def test_command_exposes_only_its_supervised_child_pid_for_resource_monitoring() -> None:
    """资源页可观察 Command，但不需要触碰认证 session 或 socket。"""
    from slope_sim.interfaces.v2.runsim_v2_command import RunSimV2Command

    command = RunSimV2Command(
        SimpleNamespace(process=SimpleNamespace(pid=7123)),
        SimpleNamespace(),
    )

    assert command.process_pid == 7123


def test_command_builder_writes_matching_template_and_bounded_interactive_argv(
    tmp_path: Path,
) -> None:
    """C++ Command 必须取得同一 session 的 wheel 形状和六小时有界运行参数。"""
    from slope_sim.interfaces.generated import slope_sim_interfaces_v2_pb2 as pb
    from slope_sim.interfaces.v2.runsim_command_supervisor import RunSimCommandLaunch
    from slope_sim.interfaces.v2.runsim_v2_command import build_interactive_command_argv

    executable = tmp_path / "slope_sim_stage4_command"
    descriptor = tmp_path / "slope_sim_interfaces_v2.desc"
    launch = RunSimCommandLaunch(tmp_path / "command.launch.lock", b"s" * 16)

    argv = build_interactive_command_argv(
        executable=executable,
        descriptor_set=descriptor,
        robot_model="df_mid",
        launch=launch,
    )

    assert argv[:3] == [str(executable), "--interactive", "--descriptor-set"]
    assert argv[3] == str(descriptor)
    assert argv[argv.index("--duration-ms") + 1] == "21600000"
    assert argv[argv.index("--deadline-ms") + 1] == "21600000"
    assert argv[argv.index("--launch-record") + 1] == str(launch.launch_record_path)
    payload = Path(argv[argv.index("--payload") + 1])
    frame = pb.WheelCommand()
    frame.ParseFromString(payload.read_bytes())
    assert frame.robot_model == "df_mid"
    assert bytes(frame.simulation_session_id) == b"s" * 16
    assert tuple(frame.drive_wheel_speed_rad_s) == (0.0, 0.0)
    assert tuple(frame.steering_wheel_speed_rad_s) == ()
