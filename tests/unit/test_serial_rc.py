"""阶段五：真实遥控器的字节流解析和失联停车门禁。"""
from __future__ import annotations

from importlib import import_module
from pathlib import Path
from threading import Event, Lock
import time

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


@pytest.mark.parametrize("endpoint", (172, 1811))
def test_rc_parser_accepts_full_sbus_analog_range_endpoints(endpoint: int) -> None:
    """协议合法端点必须进入控制门禁，不能误报 parser 失步。"""
    module = import_module("slope_sim.serial_rc")
    channels = (1002, 1002, endpoint) + (1002,) * 13

    assert module.SbusFrameParser().feed(_frame(channels)) == (channels,)


@pytest.mark.parametrize("outside", (171, 1812))
def test_rc_parser_rejects_values_outside_full_sbus_analog_range(
    outside: int,
) -> None:
    """超出协议范围的通道仍须 fail-closed，不能被饱和映射掩盖。"""
    module = import_module("slope_sim.serial_rc")
    channels = (1002, 1002, outside) + (1002,) * 13

    assert module.SbusFrameParser().feed(_frame(channels)) == ()


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


def test_rc_gate_ignores_ch6_and_recovers_after_three_fresh_frames() -> None:
    """CH6 不参与控制；短暂失联停车后连续三帧自动恢复。"""
    module = import_module("slope_sim.serial_rc")
    gate = module.RcCommandGate()
    ch6_low = (1002, 1002, 1722, 1002, 1002, 448) + (1002,) * 10
    ch6_high = (1002, 1002, 1722, 1002, 1002, 1002) + (1002,) * 10

    assert gate.observe(ch6_low, now=0.00).active is True
    assert gate.observe(ch6_high, now=0.01).active is True
    assert gate.decision(now=0.16).active is True
    timed_out = gate.decision(now=0.211)
    assert timed_out.active is False
    assert timed_out.linear_velocity_m_s == 0.0
    assert timed_out.reason == "frame_timeout"
    assert gate.observe(ch6_low, now=0.22).active is False
    assert gate.observe(ch6_low, now=0.23).active is False
    recovered = gate.observe(ch6_low, now=0.24)
    assert recovered.active is True
    assert 0.0 < recovered.linear_velocity_m_s < 3.0


def test_rc_gate_holds_last_target_for_short_gap_and_stops_at_200_ms() -> None:
    """20--150 ms 短丢帧续租稳定目标，到达 200 ms 边界立即停车。"""
    module = import_module("slope_sim.serial_rc")
    gate = module.RcCommandGate()
    forward = (1002, 1002, 1722, 1002, 1002, 1002) + (1002,) * 10

    active = gate.observe(forward, now=1.0)

    assert gate.decision(now=1.15) == active
    stopped = gate.decision(now=1.2)
    assert stopped.active is False
    assert stopped.reason == "frame_timeout"
    assert stopped.linear_velocity_m_s == 0.0


def test_rc_gate_maps_observed_ch1_ch3_endpoints_and_center() -> None:
    """实机端点必须按 1002 中位分段映射，1772 不能成为坏帧停车。"""
    module = import_module("slope_sim.serial_rc")
    centered = (1002,) * 16
    minimum = (1772, 1002, 282, 1002, 1002, 448) + (1002,) * 10
    maximum = (282, 1002, 1772, 1002, 1002, 1002) + (1002,) * 10

    center_command = module.RcCommandGate().observe(centered, now=0.00)
    minimum_command = module.RcCommandGate().observe(minimum, now=0.00)
    maximum_command = module.RcCommandGate().observe(maximum, now=0.00)

    assert center_command.linear_velocity_m_s == 0.0
    assert center_command.angular_velocity_rad_s == 0.0
    assert minimum_command.linear_velocity_m_s == -module.MAX_LINEAR_VELOCITY_M_S
    assert minimum_command.angular_velocity_rad_s == -module.MAX_ANGULAR_VELOCITY_RAD_S
    assert maximum_command.linear_velocity_m_s == module.MAX_LINEAR_VELOCITY_M_S
    assert maximum_command.angular_velocity_rad_s == module.MAX_ANGULAR_VELOCITY_RAD_S


def test_rc_gate_filters_small_stick_jitter_and_limits_target_change_rate() -> None:
    """固定杆位的小抖动不改变目标；大幅操作平滑跟随而不跳变。"""
    module = import_module("slope_sim.serial_rc")
    gate = module.RcCommandGate()

    def frame(throttle: int) -> tuple[int, ...]:
        return (1002, 1002, throttle, 1002, 1002, 1002) + (1002,) * 10

    for index in range(5):
        baseline = gate.observe(frame(1350), now=index * 0.01)
    jittered = [
        gate.observe(frame(value), now=0.05 + index * 0.01).linear_velocity_m_s
        for index, value in enumerate((1352, 1348, 1351, 1349, 1350))
    ]
    assert jittered == [baseline.linear_velocity_m_s] * 5

    before_jump = jittered[-1]
    gate.observe(frame(1722), now=0.10)
    gate.observe(frame(1722), now=0.11)
    after_jump = gate.observe(frame(1722), now=0.12)
    assert before_jump < after_jump.linear_velocity_m_s < module.MAX_LINEAR_VELOCITY_M_S


def test_rc_command_contract_has_no_ch6_unlock_state() -> None:
    module = import_module("slope_sim.serial_rc")

    command = module.RcCommand(0.0, 0.0, False, "frame_timeout")

    assert not hasattr(command, "unlocked")


def test_rc_stick_readout_maps_ch3_to_throttle_and_ch1_to_steering() -> None:
    """只读测试显示左杆前后与右杆转向，且不涉及解锁或控制输出。"""
    module = import_module("slope_sim.serial_rc")
    channels = (282, 1002, 1722) + (1002,) * 13

    readout = module.rc_stick_readout(channels)

    assert readout.throttle_channel == 3
    assert readout.steering_channel == 1
    assert readout.throttle_raw == 1722
    assert readout.steering_raw == 282
    assert readout.throttle_normalized == 1.0
    assert readout.steering_normalized == -1.0


def test_rc_stick_readout_rejects_invalid_sbus_channels() -> None:
    module = import_module("slope_sim.serial_rc")

    with pytest.raises(ValueError, match="16 SBUS channel"):
        module.rc_stick_readout((1002,) * 15)


@pytest.mark.parametrize(
    ("channel_3", "expected_linear_velocity"),
    ((172, -3.0), (282, -3.0), (1772, 3.0), (1811, 3.0)),
)
def test_rc_worker_keeps_driving_at_observed_channel_3_endpoints(
    channel_3: int,
    expected_linear_velocity: float,
) -> None:
    """合法 SBUS 全量程超出校准端点后仍饱和匀速，不能被误判停车。"""
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


def test_rc_full_forward_stays_saturated_for_thirty_seconds_at_100_hz() -> None:
    """合法高端固定杆位必须连续匀速，不能在长时窗口中插入安全零。"""
    module = import_module("slope_sim.serial_rc")
    parser = module.SbusFrameParser()
    gate = module.RcCommandGate()
    channels = (1002, 1002, 1811) + (1002,) * 13
    wire = _frame(channels)

    commands = []
    for index in range(3_001):
        (decoded,) = parser.feed(wire)
        commands.append(gate.observe(decoded, now=index / 100.0))

    assert all(command.active for command in commands)
    assert all(command.reason == "active" for command in commands)
    assert all(
        command.linear_velocity_m_s == module.MAX_LINEAR_VELOCITY_M_S
        for command in commands
    )


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


def test_rc_auto_qualification_skips_eio_ports_but_explicit_port_reports_eio(
    tmp_path: Path,
) -> None:
    """自动扫描跳过单口 EIO；显式指定同一坏口时保留原始故障。"""
    module = import_module("slope_sim.serial_rc")
    bad = tmp_path / "usb-bad"
    good = tmp_path / "usb-good"
    valid = _frame((1002,) * 16)

    class Reader:
        def read(self, _size):
            return valid

        def close(self):
            return None

    def opener(path: Path):
        if path == bad:
            raise OSError(5, "Input/output error")
        return Reader()

    assert module.select_rc_port(
        (bad, good), opener=opener, duration_sec=0.1
    ) == good
    with pytest.raises(OSError, match="Input/output error"):
        module.select_rc_port(
            (bad, good),
            opener=opener,
            explicit_path=bad,
            duration_sec=0.1,
        )


def test_rc_worker_forwards_only_gated_commands_and_stops_on_parser_fault() -> None:
    """worker 只向注入的本地 IPC sink 发送候选，不创建 eCAL publisher。"""
    module = import_module("slope_sim.serial_rc")
    ch6_low = _frame((1002, 1002, 1722, 1002, 1002, 448) + (1002,) * 10)
    ch6_high = _frame((1002, 1002, 1722, 1002, 1002, 1002) + (1002,) * 10)
    sent = []

    class Reader:
        def __init__(self):
            self.payloads = iter((ch6_low, ch6_high, b"\x0f" + b"\xff" * 24))

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

    assert sent[0][0].active is True
    assert sent[0][0].linear_velocity_m_s == 3.0
    assert sent[1][0].active is True
    assert sent[1][0].linear_velocity_m_s == 3.0
    assert sent[2][0].reason == "parser_desynchronized"
    assert snapshot.path.name == "usb-FTDI"
    assert snapshot.last_channels is not None and snapshot.last_channels[5] == 1002


def test_rc_worker_uses_valid_frame_when_resync_batch_contains_one() -> None:
    """同批坏候选后的有效帧仍须续租，不能把流重同步升级为硬故障。"""
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
    assert command.reason == "active"
    assert sent[2][0] == command


def test_rc_worker_uses_a_valid_frame_after_a_scheduler_delay_before_timing_out() -> None:
    """线程延迟后若已读到新鲜有效帧，不得先触发超时停车。"""
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
    assert command.reason == "active"


def test_rc_worker_recovers_after_three_frames_following_empty_read_timeout() -> None:
    """真实空读超过 200 ms 停车，恢复连续三帧后自动续行。"""
    module = import_module("slope_sim.serial_rc")
    locked = _frame((1000, 1000, 1722, 1000, 1000, 448) + (1000,) * 10)
    unlocked = _frame((1000, 1000, 1722, 1000, 1000, 1002) + (1000,) * 10)

    class Reader:
        def __init__(self):
            self.payloads = iter((locked, unlocked, b"", unlocked, unlocked, unlocked))

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
    timed_out = worker.process_once(now=0.211)
    recovering_one = worker.process_once(now=0.22)
    recovering_two = worker.process_once(now=0.23)
    recovered = worker.process_once(now=0.24)
    worker.close()

    assert timed_out.reason == "frame_timeout"
    assert timed_out.active is False
    assert recovering_one.reason == "recovering_frames"
    assert recovering_two.reason == "recovering_frames"
    assert recovered.reason == "active"
    assert recovered.active is True


def test_rc_worker_counts_each_watchdog_timeout_transition_once() -> None:
    """资源页的 watchdog 计数代表失联事件数，不随空读循环重复膨胀。"""
    module = import_module("slope_sim.serial_rc")
    first = _frame((1002, 1002, 1722, 1002, 1002, 448) + (1002,) * 10)
    second = _frame((1002, 1002, 1722, 1002, 1002, 1002) + (1002,) * 10)

    class Reader:
        def __init__(self):
            self.payloads = iter((first, second))

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
    worker.process_once(now=0.211)
    worker.process_once(now=0.22)

    assert worker.snapshot(now=0.22).watchdog_timeout_count == 1


def test_rc_worker_stops_immediately_when_serial_read_raises_eio() -> None:
    """EIO/拔线是硬故障，不等待 200 ms 软超时。"""
    module = import_module("slope_sim.serial_rc")
    sent = []

    class Reader:
        def read(self, _size):
            raise OSError(5, "Input/output error")

        def close(self):
            return None

    worker = module.SerialRcWorker(
        Path("/dev/serial/by-id/usb-FTDI"),
        reader=Reader(),
        command_sink=lambda command, now: sent.append((command, now)),
        start_worker=False,
    )

    command = worker.process_once(now=0.01)

    assert command.active is False
    assert command.reason == "serial_read_failed"
    assert sent == [(command, 0.01)]


def test_command_arbiter_keeps_selected_rc_during_soft_stop_and_revokes_hard_fault() -> None:
    """RC 暂时失联只停车并保持选择；硬故障仍撤销控制权。"""
    module = import_module("slope_sim.serial_rc")
    sent: list[tuple[float, float, float]] = []

    class Client:
        def send_target(self, linear_velocity, angular_velocity, *, now):
            sent.append((linear_velocity, angular_velocity, now))

    arbiter = module.CommandSourceArbiter(Client())
    active = module.RcCommand(1.5, -0.5, True, "active")
    timed_out = module.RcCommand(0.0, 0.0, False, "frame_timeout")

    assert arbiter.snapshot().active_source is None

    assert arbiter.submit_rc(active, now=0.0) is False
    assert sent == []

    arbiter.select_source("rc", now=0.01)
    assert sent == [(0.0, 0.0, 0.01)]
    assert arbiter.submit_rc(active, now=0.02) is True
    assert sent[-1] == (1.5, -0.5, 0.02)

    assert arbiter.submit_rc(timed_out, now=0.03) is False
    assert sent[-1] == (0.0, 0.0, 0.03)
    assert arbiter.snapshot().active_source == "rc"
    assert arbiter.snapshot().failure_reason == "frame_timeout"
    assert arbiter.submit_rc(active, now=0.035) is True
    assert sent[-1] == (1.5, -0.5, 0.035)
    assert arbiter.snapshot().failure_reason is None

    parser_fault = module.RcCommand(0.0, 0.0, False, "parser_desynchronized")
    assert arbiter.submit_rc(parser_fault, now=0.037) is False
    assert sent[-1] == (0.0, 0.0, 0.037)
    assert arbiter.snapshot().active_source is None
    assert arbiter.snapshot().failure_reason == "parser_desynchronized"

    arbiter.select_source("keyboard", now=0.04)
    assert arbiter.submit_keyboard(0.2, 0.3, now=0.05) is True
    assert sent[-1] == (0.2, 0.3, 0.05)
    arbiter.submit_fault("ipc_interrupted", now=0.06)
    assert sent[-1] == (0.0, 0.0, 0.06)
    assert arbiter.snapshot().active_source is None


def test_command_arbiter_fixed_rate_renewer_uses_latest_target_and_immediate_stop() -> None:
    """主循环只更新最新目标；续租节拍发最新值，停车不等待节拍。"""
    module = import_module("slope_sim.serial_rc")
    sent: list[tuple[float, float, float]] = []

    class Client:
        def send_target(self, linear_velocity, angular_velocity, *, now):
            sent.append((linear_velocity, angular_velocity, now))

    arbiter = module.CommandSourceArbiter(
        Client(), renewal_hz=50.0, start_renewer=False
    )
    arbiter.select_source("keyboard", now=0.0)
    assert sent == [(0.0, 0.0, 0.0)]

    assert arbiter.submit_keyboard(0.5, 0.1, now=0.001) is True
    assert arbiter.submit_keyboard(0.8, -0.2, now=0.010) is True
    assert sent == [(0.0, 0.0, 0.0)]

    arbiter.renew_once(now=0.020)
    arbiter.renew_once(now=0.040)
    assert sent[-2:] == [(0.8, -0.2, 0.020), (0.8, -0.2, 0.040)]
    snapshot = arbiter.snapshot(now=0.045)
    assert snapshot.latest_target == (0.8, -0.2)
    assert snapshot.mailbox_update_count == 2
    assert snapshot.command_send_count == 3
    assert snapshot.renewal_count == 2
    assert snapshot.last_renewal_age_sec == pytest.approx(0.005)
    assert snapshot.max_renewal_gap_sec == pytest.approx(0.020)
    assert snapshot.renewal_hz == 50.0

    arbiter.select_source("rc", now=0.041)
    assert sent[-1] == (0.0, 0.0, 0.041)
    arbiter.close(now=0.050)
    assert sent[-1] == (0.0, 0.0, 0.050)


def test_command_arbiter_thread_renews_at_50_hz_below_final_lease_margin() -> None:
    """真实线程节拍在无 GUI 负载时必须远低于 C++ 100 ms 最终租约。"""
    module = import_module("slope_sim.serial_rc")
    sent: list[tuple[float, float, float]] = []
    lock = Lock()
    enough = Event()

    class Client:
        def send_target(self, linear_velocity, angular_velocity, *, now):
            with lock:
                sent.append((linear_velocity, angular_velocity, now))
                if sum(value[0] == 1.0 for value in sent) >= 6:
                    enough.set()

    arbiter = module.CommandSourceArbiter(Client(), renewal_hz=50.0)
    arbiter.select_source("keyboard", now=time.monotonic())
    arbiter.submit_keyboard(1.0, 0.0, now=time.monotonic())
    try:
        assert enough.wait(0.5)
    finally:
        arbiter.close()

    renewals = [timestamp for linear, _angular, timestamp in sent if linear == 1.0]
    gaps = [current - previous for previous, current in zip(renewals, renewals[1:])]
    assert len(renewals) >= 6
    assert max(gaps) < 0.08


def test_command_arbiter_switches_keyboard_rc_external_with_one_zero_cycle() -> None:
    """键盘→RC→外部→RC 每次只插入一次零命令，新源下一续租周期接管。"""
    module = import_module("slope_sim.serial_rc")
    sent: list[tuple[float, float]] = []

    class Client:
        def send_target(self, linear, angular, *, now):
            sent.append((linear, angular))

    arbiter = module.CommandSourceArbiter(
        Client(), renewal_hz=50.0, start_renewer=False
    )
    arbiter.select_source("keyboard", now=0.00)
    arbiter.submit_keyboard(0.4, 0.0, now=0.01)
    arbiter.renew_once(now=0.02)
    arbiter.select_source("rc", now=0.03)
    arbiter.submit_rc(module.RcCommand(0.8, -0.1, True, "active"), now=0.04)
    arbiter.renew_once(now=0.05)
    arbiter.select_source("external", now=0.06)
    arbiter.submit_external(-0.2, 0.3, now=0.07)
    arbiter.renew_once(now=0.08)
    arbiter.select_source("rc", now=0.09)
    arbiter.submit_rc(module.RcCommand(0.6, 0.2, True, "active"), now=0.10)
    arbiter.renew_once(now=0.11)

    assert sent == [
        (0.0, 0.0),
        (0.4, 0.0),
        (0.0, 0.0),
        (0.8, -0.1),
        (0.0, 0.0),
        (-0.2, 0.3),
        (0.0, 0.0),
        (0.6, 0.2),
    ]


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
    first = _frame((1002, 1002, 1722, 1002, 1002, 448) + (1002,) * 10)
    second = _frame((1002, 1002, 1722, 1002, 1002, 1002) + (1002,) * 10)
    sent: list[tuple[float, float, float]] = []

    class Reader:
        def __init__(self, payloads):
            self._payloads = iter(payloads)

        def read(self, _size):
            return next(self._payloads, b"")

        def close(self):
            return None

    readers = iter((Reader([valid] * 20), Reader([first, second])))
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
