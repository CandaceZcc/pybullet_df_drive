"""阶段五：真实遥控器的字节流解析和失联停车门禁。"""
from __future__ import annotations

from importlib import import_module
from pathlib import Path

import pytest


def _frame(channels: tuple[int, ...]) -> bytes:
    packed = 0
    for index, value in enumerate(channels):
        packed |= value << (11 * index)
    return bytes((0x0F,)) + packed.to_bytes(22, "little") + b"\x00\x00"


def test_rc_parser_resynchronizes_and_preserves_partial_reads() -> None:
    """read() 边界不是帧边界；噪声与分段数据后仍只输出完整合法帧。"""
    module = import_module("slope_sim.serial_rc")
    parser = module.SbusFrameParser()
    channels = (1000,) * 16
    wire = _frame(channels)

    assert parser.feed(b"noise\x01" + wire[:9]) == ()
    assert parser.feed(wire[9:]) == (channels,)
    assert parser.feed(_frame((1000,) * 15 + (2000,))) == ()


def test_pyserial_opener_requires_dependency_and_uses_sbus_settings() -> None:
    """真实串口只在启用 RC 时导入；缺依赖与连接参数必须可诊断。"""
    module = import_module("slope_sim.serial_rc")
    path = Path("/dev/serial/by-id/usb-FTDI")

    with pytest.raises(RuntimeError, match="pyserial is required"):
        module.pyserial_opener(serial_import=lambda _name: (_ for _ in ()).throw(ImportError()))

    opened: list[tuple[str, int, float]] = []

    class SerialModule:
        @staticmethod
        def Serial(port, *, baudrate, timeout):
            opened.append((port, baudrate, timeout))
            return object()

    opener = module.pyserial_opener(serial_import=lambda _name: SerialModule)
    assert opener(path) is not None
    assert opened == [(str(path), 115200, 0.02)]


def test_rc_gate_requires_a_fresh_unlock_edge_and_stops_after_timeout() -> None:
    """故障恢复不能自动接管，ch6 必须先低后高，100 ms 无帧必须输出零。"""
    module = import_module("slope_sim.serial_rc")
    gate = module.RcCommandGate()
    locked = (1000, 1000, 1722, 1000, 1000, 448) + (1000,) * 10
    unlocked = (1000, 1000, 1722, 1000, 1000, 1002) + (1000,) * 10

    assert gate.observe(unlocked, now=0.0).active is False
    assert gate.observe(locked, now=0.01).active is False
    command = gate.observe(unlocked, now=0.02)
    assert command.active is True
    assert command.linear_velocity_m_s == 3.0
    assert command.angular_velocity_rad_s == 0.0
    timed_out = gate.decision(now=0.121)
    assert timed_out.active is False
    assert timed_out.linear_velocity_m_s == 0.0
    assert timed_out.reason == "frame_timeout"


def test_rc_gate_accepts_the_observed_two_position_switch_range() -> None:
    """实机 CH6 只覆盖约 448..1002，低到高沿仍必须能安全解锁。"""
    module = import_module("slope_sim.serial_rc")
    gate = module.RcCommandGate()
    locked = (1002, 1002, 1002, 1002, 1002, 448) + (1002,) * 10
    unlocked = (1002, 1002, 1722, 1002, 1002, 1002) + (1002,) * 10

    assert gate.observe(locked, now=0.0).reason == "locked"
    command = gate.observe(unlocked, now=0.01)

    assert command.active is True
    assert command.unlocked is True
    assert command.linear_velocity_m_s == 3.0


@pytest.mark.parametrize(
    ("channel_3", "expected_linear_velocity"),
    ((282, -3.0), (1772, 3.0)),
)
def test_rc_worker_keeps_driving_at_observed_channel_3_endpoints(
    channel_3: int,
    expected_linear_velocity: float,
) -> None:
    """实机 CH3 到达机械端点时仍是有效帧，不能被 parser 故障停车。"""
    module = import_module("slope_sim.serial_rc")
    locked = _frame((1002, 1002, channel_3, 1002, 1002, 448) + (1002,) * 10)
    unlocked = _frame((1002, 1002, channel_3, 1002, 1002, 1002) + (1002,) * 10)

    class Reader:
        def __init__(self):
            self.payloads = iter((locked, unlocked, unlocked))

        def read(self, _size):
            return next(self.payloads, b"")

        def close(self):
            return None

    worker = module.SerialRcWorker(
        Path("/dev/serial/by-id/usb-FTDI"),
        reader=Reader(),
        command_sink=lambda _command, _now: None,
        start_worker=False,
    )

    worker.process_once(now=0.0)
    command = worker.process_once(now=0.01)
    renewed = worker.process_once(now=0.02)
    snapshot = worker.snapshot(now=0.02)
    worker.close()

    assert command.active is True
    assert command.linear_velocity_m_s == expected_linear_velocity
    assert renewed == command
    assert snapshot.failure_reason is None


def test_rc_port_qualification_requires_twenty_valid_frames_and_rejects_ambiguity(
    tmp_path: Path,
) -> None:
    """只从稳定 by-id 链接选择；不足帧数与多个合格端口都不能自动接管。"""
    module = import_module("slope_sim.serial_rc")
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    first = by_id / "usb-FTDI-A"
    second = by_id / "usb-FTDI-B"
    first.touch()
    second.touch()
    valid = _frame((1000,) * 16)

    class Reader:
        def __init__(self, payloads):
            self._payloads = iter(payloads)
            self.closed = False

        def read(self, _size):
            return next(self._payloads, b"")

        def close(self):
            self.closed = True

    paths = module.serial_by_id_candidates(by_id)
    assert paths == (first, second)

    readers = {
        first: Reader([valid] * 19),
        second: Reader([valid] * 20),
    }
    assert module.qualify_rc_ports(
        paths,
        opener=readers.__getitem__,
        duration_sec=2.0,
        min_valid_frames=20,
        monotonic=iter(index * 0.05 for index in range(200)).__next__,
    ) == (second,)

    ambiguous = {
        first: Reader([valid] * 20),
        second: Reader([valid] * 20),
    }
    try:
        module.select_rc_port(
            paths,
            opener=ambiguous.__getitem__,
            duration_sec=2.0,
            min_valid_frames=20,
            monotonic=iter(index * 0.05 for index in range(200)).__next__,
        )
    except RuntimeError as error:
        assert str(error) == "multiple qualified RC ports: usb-FTDI-A, usb-FTDI-B"
    else:
        raise AssertionError("ambiguous RC ports must not be auto-selected")


def test_rc_worker_forwards_only_gated_commands_and_stops_on_parser_fault() -> None:
    """worker 只向注入的本地 IPC sink 发送候选，不创建 eCAL publisher。"""
    module = import_module("slope_sim.serial_rc")
    locked = _frame((1000, 1000, 1722, 1000, 1000, 448) + (1000,) * 10)
    unlocked = _frame((1000, 1000, 1722, 1000, 1000, 1002) + (1000,) * 10)
    sent = []

    class Reader:
        def __init__(self):
            self.payloads = iter((locked, unlocked, b"\x0f" + b"\xff" * 24))

        def read(self, _size):
            return next(self.payloads, b"")

        def close(self):
            return None

    worker = module.SerialRcWorker(
        Path("/dev/serial/by-id/usb-FTDI"),
        reader=Reader(),
        command_sink=lambda command, now: sent.append((command, now)),
        start_worker=False,
    )
    worker.process_once(now=0.0)
    worker.process_once(now=0.01)
    worker.process_once(now=0.02)
    snapshot = worker.snapshot(now=0.02)
    worker.close()

    assert sent[0][0].reason == "locked"
    assert sent[1][0].active is True
    assert sent[1][0].linear_velocity_m_s == 3.0
    assert sent[2][0].reason == "parser_desynchronized"
    assert snapshot.path.name == "usb-FTDI"
    assert snapshot.last_channels is not None and snapshot.last_channels[5] == 1002


def test_rc_worker_preserves_unlock_when_resync_batch_contains_a_valid_frame() -> None:
    """同批坏候选后的有效帧仍须续租，不能把流重同步升级为永久失锁。"""
    module = import_module("slope_sim.serial_rc")
    locked = _frame((1000, 1000, 1722, 1000, 1000, 448) + (1000,) * 10)
    unlocked = _frame((1000, 1000, 1722, 1000, 1000, 1002) + (1000,) * 10)
    malformed = b"\x0f" + b"\xff" * 24
    sent = []

    class Reader:
        def __init__(self):
            self.payloads = iter((locked, unlocked, malformed + unlocked))

        def read(self, _size):
            return next(self.payloads, b"")

        def close(self):
            return None

    worker = module.SerialRcWorker(
        Path("/dev/serial/by-id/usb-FTDI"),
        reader=Reader(),
        command_sink=lambda command, now: sent.append((command, now)),
        start_worker=False,
    )

    worker.process_once(now=0.0)
    worker.process_once(now=0.01)
    command = worker.process_once(now=0.02)
    worker.close()

    assert command.active is True
    assert command.unlocked is True
    assert command.reason == "active"
    assert sent[2][0] == command


def test_rc_worker_uses_a_valid_frame_after_a_scheduler_delay_before_timing_out() -> None:
    """线程延迟后若已读到新鲜有效帧，不得先清除既有解锁授权。"""
    module = import_module("slope_sim.serial_rc")
    locked = _frame((1000, 1000, 1722, 1000, 1000, 448) + (1000,) * 10)
    unlocked = _frame((1000, 1000, 1722, 1000, 1000, 1002) + (1000,) * 10)

    class Reader:
        def __init__(self):
            self.payloads = iter((locked, unlocked, unlocked))

        def read(self, _size):
            return next(self.payloads, b"")

        def close(self):
            return None

    worker = module.SerialRcWorker(
        Path("/dev/serial/by-id/usb-FTDI"),
        reader=Reader(),
        command_sink=lambda _command, _now: None,
        start_worker=False,
    )

    worker.process_once(now=0.0)
    assert worker.process_once(now=0.01).active is True
    command = worker.process_once(now=0.25)
    worker.close()

    assert command.active is True
    assert command.unlocked is True
    assert command.reason == "active"


def test_rc_worker_requires_a_new_unlock_edge_after_a_real_empty_read_timeout() -> None:
    """真实空读超过 100 ms 后必须失锁，恢复时保持高位不能自行复活。"""
    module = import_module("slope_sim.serial_rc")
    locked = _frame((1000, 1000, 1722, 1000, 1000, 448) + (1000,) * 10)
    unlocked = _frame((1000, 1000, 1722, 1000, 1000, 1002) + (1000,) * 10)

    class Reader:
        def __init__(self):
            self.payloads = iter((locked, unlocked, b"", unlocked))

        def read(self, _size):
            return next(self.payloads, b"")

        def close(self):
            return None

    worker = module.SerialRcWorker(
        Path("/dev/serial/by-id/usb-FTDI"),
        reader=Reader(),
        command_sink=lambda _command, _now: None,
        start_worker=False,
    )

    worker.process_once(now=0.0)
    assert worker.process_once(now=0.01).active is True
    timed_out = worker.process_once(now=0.12)
    recovered_high = worker.process_once(now=0.13)
    worker.close()

    assert timed_out.reason == "frame_timeout"
    assert timed_out.active is False
    assert recovered_high.reason == "awaiting_unlock_edge"
    assert recovered_high.active is False


def test_rc_worker_counts_each_watchdog_timeout_transition_once() -> None:
    """资源页的 watchdog 计数代表失联事件数，不随空读循环重复膨胀。"""
    module = import_module("slope_sim.serial_rc")
    locked = _frame((1000, 1000, 1722, 1000, 1000, 448) + (1000,) * 10)
    unlocked = _frame((1000, 1000, 1722, 1000, 1000, 1002) + (1000,) * 10)

    class Reader:
        def __init__(self):
            self.payloads = iter((locked, unlocked))

        def read(self, _size):
            return next(self.payloads, b"")

        def close(self):
            return None

    worker = module.SerialRcWorker(
        Path("/dev/serial/by-id/usb-FTDI"),
        reader=Reader(),
        command_sink=lambda _command, _now: None,
        start_worker=False,
    )
    worker.process_once(now=0.0)
    worker.process_once(now=0.01)
    worker.process_once(now=0.12)
    worker.process_once(now=0.13)

    assert worker.snapshot(now=0.13).watchdog_timeout_count == 1


def test_command_arbiter_requires_explicit_source_and_revokes_rc_on_fault() -> None:
    """控制源切换、RC 失锁和 IPC 故障均先通过唯一 socket 发零目标。"""
    module = import_module("slope_sim.serial_rc")
    sent: list[tuple[float, float, float]] = []

    class Client:
        def send_target(self, linear_velocity, angular_velocity, *, now):
            sent.append((linear_velocity, angular_velocity, now))

    arbiter = module.CommandSourceArbiter(Client())
    active = module.RcCommand(1.5, -0.5, True, True, "active")
    locked = module.RcCommand(0.0, 0.0, False, False, "locked")

    assert arbiter.snapshot().active_source is None

    assert arbiter.submit_rc(active, now=0.0) is False
    assert sent == []

    arbiter.select_source("rc", now=0.01)
    assert sent == [(0.0, 0.0, 0.01)]
    assert arbiter.submit_rc(active, now=0.02) is True
    assert sent[-1] == (1.5, -0.5, 0.02)

    assert arbiter.submit_rc(locked, now=0.03) is False
    assert sent[-1] == (0.0, 0.0, 0.03)
    assert arbiter.snapshot().active_source is None
    assert arbiter.snapshot().failure_reason == "locked"

    arbiter.select_source("keyboard", now=0.04)
    assert arbiter.submit_keyboard(0.2, 0.3, now=0.05) is True
    assert sent[-1] == (0.2, 0.3, 0.05)
    arbiter.submit_fault("ipc_interrupted", now=0.06)
    assert sent[-1] == (0.0, 0.0, 0.06)
    assert arbiter.snapshot().active_source is None


def test_start_rc_worker_qualifies_by_id_port_before_forwarding_to_arbiter(
    tmp_path: Path,
) -> None:
    """端口必须先连续通过 SBUS 资格判定，worker 再经唯一仲裁器送候选命令。"""
    module = import_module("slope_sim.serial_rc")
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    port = by_id / "usb-FTDI"
    port.touch()
    valid = _frame((1000,) * 16)
    locked = _frame((1000, 1000, 1722, 1000, 1000, 448) + (1000,) * 10)
    unlocked = _frame((1000, 1000, 1722, 1000, 1000, 1002) + (1000,) * 10)
    sent: list[tuple[float, float, float]] = []

    class Reader:
        def __init__(self, payloads):
            self._payloads = iter(payloads)

        def read(self, _size):
            return next(self._payloads, b"")

        def close(self):
            return None

    readers = iter((Reader([valid] * 20), Reader([locked, unlocked])))
    arbiter = module.CommandSourceArbiter(
        type("Client", (), {"send_target": lambda _self, v, w, *, now: sent.append((v, w, now))})()
    )
    worker = module.start_rc_worker(
        command_sink=lambda command, now: arbiter.submit_rc(command, now=now),
        opener=lambda _path: next(readers),
        by_id_directory=by_id,
        monotonic=iter(index * 0.05 for index in range(200)).__next__,
        start_worker=False,
    )
    arbiter.select_source("rc", now=1.0)
    worker.process_once(now=1.01)
    arbiter.select_source("rc", now=1.015)
    worker.process_once(now=1.02)
    worker.close()

    assert worker.snapshot(now=1.02).path == port
    assert sent[-2] == (3.0, 0.0, 1.02)
    assert sent[-1] == (0.0, 0.0, 1.05)
