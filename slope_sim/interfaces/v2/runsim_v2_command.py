"""正式 runSim v2 的唯一 C++ Command 启动参数与身份模板。"""
from __future__ import annotations

from pathlib import Path
import secrets
import tempfile
import time

from slope_sim.interfaces.v2.codec import V2ProtoCodec
from slope_sim.interfaces.v2.descriptor import load_v2_descriptor
from slope_sim.interfaces.v2.models import WheelCommandV2
from slope_sim.interfaces.v2.runsim_command_supervisor import RunSimCommandLaunch
from slope_sim.interfaces.v2.runsim_command_supervisor import RunSimCommandSupervisor
from slope_sim.interfaces.v2.runsim_command_client import RunSimCommandRelayClient
from slope_sim.model_registry import get_robot_model


_INTERACTIVE_DURATION_MS = 21_600_000


class RunSimV2Command:
    """持有正式 C++ Command 与 GUI 使用的唯一认证 socket client。"""

    def __init__(self, supervisor: RunSimCommandSupervisor, client: object) -> None:
        self._supervisor = supervisor
        self.client = client

    @property
    def session_id_factory(self):
        """为 Python v2 protocol 提供与 C++ Command 完全相同的身份。"""
        session_id = bytes.fromhex(self._supervisor.session.server_authentication["session_id"])
        return lambda: session_id

    @property
    def process_pid(self) -> int:
        """返回受 supervisor 管理的唯一 C++ Command 子进程 PID。"""
        pid = self._supervisor.process.pid
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise RuntimeError("runSim v2 Command has an invalid process PID")
        return pid

    @classmethod
    def launch(cls, *, release_root: Path, robot_model: str) -> "RunSimV2Command":
        """仅从安装 release 启动 C++ Command，禁止开发期 fallback。"""
        executable = release_root / "bin" / "slope_sim_stage4_command"
        descriptor_set = release_root / "slope_sim/interfaces/generated/slope_sim_interfaces_v2.desc"
        if not executable.is_file() or not executable.stat().st_mode & 0o111:
            raise RuntimeError(f"runSim v2 Command executable is unavailable: {executable}")
        if not descriptor_set.is_file():
            raise RuntimeError(f"runSim v2 descriptor set is unavailable: {descriptor_set}")
        supervisor = RunSimCommandSupervisor.launch(
            lambda launch: build_interactive_command_argv(
                executable=executable,
                descriptor_set=descriptor_set,
                robot_model=robot_model,
                launch=launch,
            ),
            socket_parent=Path(tempfile.gettempdir()).resolve(),
        )
        try:
            deadline = time.monotonic() + 5.0
            while not supervisor.session.socket_path.exists():
                if supervisor.process.poll() is not None:
                    raise RuntimeError("runSim v2 Command exited before opening its socket")
                if time.monotonic() >= deadline:
                    raise RuntimeError("runSim v2 Command did not open its authenticated socket")
                time.sleep(0.01)
            client = RunSimCommandRelayClient.launch(supervisor.session)
            return cls(supervisor, client)
        except BaseException:
            supervisor.close()
            raise

    def close(self) -> None:
        """先断开 GUI target lease，再终止唯一 C++ Command 进程组。"""
        self.client.close()
        self._supervisor.close()


def build_interactive_command_argv(
    *,
    executable: Path,
    descriptor_set: Path,
    robot_model: str,
    launch: RunSimCommandLaunch,
) -> list[str]:
    """为 C++ Command 创建同 session 的零速模板与六小时受监管 argv。"""
    if not isinstance(executable, Path) or not executable.is_absolute():
        raise ValueError("executable must be an absolute Path")
    if not isinstance(descriptor_set, Path) or not descriptor_set.is_absolute():
        raise ValueError("descriptor_set must be an absolute Path")
    if not isinstance(launch, RunSimCommandLaunch):
        raise ValueError("launch must be a RunSimCommandLaunch")
    model = get_robot_model(robot_model)
    descriptor = load_v2_descriptor()
    template = WheelCommandV2(
        timestamp_ns=0,
        drive_wheel_speed_rad_s=(0.0,) * len(model.drive_joint_names),
        steering_wheel_speed_rad_s=(0.0,) * len(model.steering_joint_names),
        sequence=0,
        world_generation=1,
        command_generation=1,
        source_id="runsim.gui",
        source_session_id=secrets.token_bytes(16),
        robot_model=model.name,
        simulation_session_id=launch.simulation_session_id,
        descriptor_sha256=descriptor.sha256,
    )
    payload_path = launch.launch_record_path.with_name("wheel-command-template.bin")
    payload_path.write_bytes(V2ProtoCodec(descriptor).encode(template).payload)
    result_path = launch.launch_record_path.with_name("command.result.json")
    duration = str(_INTERACTIVE_DURATION_MS)
    return [
        str(executable),
        "--interactive",
        "--descriptor-set",
        str(descriptor_set),
        "--payload",
        str(payload_path),
        "--duration-ms",
        duration,
        "--deadline-ms",
        duration,
        "--result",
        str(result_path),
        "--launch-record",
        str(launch.launch_record_path),
    ]
