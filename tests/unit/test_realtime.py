# 墙钟实时节拍单元测试：锁定绝对 deadline、暂停恢复和超期统计。
from __future__ import annotations

import importlib

import pytest


def _deadline_pacer_type():
    """延迟导入待实现模块，让缺失公共节拍器表现为明确测试失败。"""
    return importlib.import_module("slope_sim.realtime").DeadlinePacer


def _runtime_observation_cadence_type():
    """延迟读取共享观测节拍器，让缺失 API 形成明确 RED。"""
    return importlib.import_module("slope_sim.realtime").RuntimeObservationCadence


def test_runtime_observation_cadence_rebases_after_slow_poll() -> None:
    """下一次 discovery 必须从慢 poll 完成时起算，而不是追赶旧期限。"""
    clock = [10.0]
    polls: list[float] = []

    class Runtime:
        def poll_transport(self) -> None:
            polls.append(clock[0])
            clock[0] += 0.080

    cadence = _runtime_observation_cadence_type()(
        period_sec=0.050,
        monotonic=lambda: clock[0],
    )

    due, wall_time = cadence.poll_if_due(Runtime())
    assert due is True
    assert wall_time == pytest.approx(10.080)
    assert cadence.next_observation_at == pytest.approx(10.130)

    clock[0] = 10.081
    due, wall_time = cadence.poll_if_due(Runtime())
    assert due is False
    assert wall_time == pytest.approx(10.081)
    assert cadence.next_observation_at == pytest.approx(10.130)
    assert polls == [10.0]


def test_runtime_observation_cadence_is_20hz_without_catch_up_bursts() -> None:
    """240 Hz 调用只产生 20 Hz poll；大幅迟到也只执行一次。"""
    clock = [0.0]
    polls: list[float] = []

    class Runtime:
        def poll_transport(self) -> None:
            polls.append(clock[0])

    cadence = _runtime_observation_cadence_type()(
        period_sec=0.050,
        monotonic=lambda: clock[0],
    )
    runtime = Runtime()

    for frame in range(240):
        clock[0] = frame / 240.0
        cadence.poll_if_due(runtime)

    assert 19 <= len(polls) <= 20

    before_late_poll = len(polls)
    clock[0] = 2.0
    assert cadence.poll_if_due(runtime)[0] is True
    assert len(polls) == before_late_poll + 1
    assert cadence.next_observation_at == pytest.approx(2.050)

    clock[0] = 2.001
    assert cadence.poll_if_due(runtime)[0] is False
    assert len(polls) == before_late_poll + 1


def test_runtime_observation_reset_polls_once_then_restores_period_boundary() -> None:
    """重建边界后下一帧立即观测，紧随其后的 50 ms 内不得重复 poll。"""
    clock = [5.0]
    polls: list[float] = []

    class Runtime:
        def poll_transport(self) -> None:
            polls.append(clock[0])

    cadence = _runtime_observation_cadence_type()(
        period_sec=0.050,
        monotonic=lambda: clock[0],
    )
    runtime = Runtime()

    assert cadence.poll_if_due(runtime)[0] is True
    clock[0] = 5.010
    assert cadence.poll_if_due(runtime)[0] is False
    cadence.reset()
    assert cadence.poll_if_due(runtime)[0] is True
    clock[0] = 5.020
    assert cadence.poll_if_due(runtime)[0] is False
    assert polls == pytest.approx([5.0, 5.010])


def test_deadline_pacer_subtracts_frame_work_from_each_absolute_period() -> None:
    clock = [0.0]
    sleeps: list[float] = []

    def sleep(delay: float) -> None:
        sleeps.append(delay)
        clock[0] += delay

    pacer = _deadline_pacer_type()(
        0.01,
        monotonic=lambda: clock[0],
        sleep=sleep,
    )
    pacer.start()

    samples = []
    for _index in range(3):
        clock[0] += 0.004
        samples.append(pacer.wait_for_next_deadline())

    assert sleeps == pytest.approx([0.006, 0.006, 0.006])
    assert [sample.deadline_sec for sample in samples] == pytest.approx(
        [0.01, 0.02, 0.03]
    )
    assert [sample.work_sec for sample in samples] == pytest.approx([0.004] * 3)
    assert [sample.sleep_requested_sec for sample in samples] == pytest.approx(
        [0.006] * 3
    )
    assert clock[0] == pytest.approx(0.03)


def test_deadline_pacer_reset_excludes_paused_wall_time_from_next_frame() -> None:
    clock = [0.0]
    sleeps: list[float] = []

    def sleep(delay: float) -> None:
        sleeps.append(delay)
        clock[0] += delay

    pacer = _deadline_pacer_type()(
        0.01,
        monotonic=lambda: clock[0],
        sleep=sleep,
    )
    pacer.start()
    clock[0] += 0.002
    pacer.wait_for_next_deadline()

    clock[0] += 5.0
    pacer.reset_deadline()
    clock[0] += 0.003
    resumed = pacer.wait_for_next_deadline()

    assert sleeps == pytest.approx([0.008, 0.007])
    assert resumed.work_sec == pytest.approx(0.003)
    assert resumed.lateness_sec == 0.0
    assert resumed.overrun is False


def test_deadline_pacer_yields_without_positive_delay_for_overrun_frames() -> None:
    clock = [0.0]
    sleeps: list[float] = []

    def sleep(delay: float) -> None:
        sleeps.append(delay)
        clock[0] += delay

    pacer = _deadline_pacer_type()(
        0.01,
        monotonic=lambda: clock[0],
        sleep=sleep,
    )
    pacer.start()

    clock[0] = 0.025
    first = pacer.wait_for_next_deadline()
    clock[0] += 0.001
    second = pacer.wait_for_next_deadline()

    assert sleeps == pytest.approx([0.0, 0.0])
    assert first.overrun is second.overrun is True
    assert first.lateness_sec == pytest.approx(0.015)
    assert first.yield_requested_sec == 0.0
    assert pacer.statistics.wait_count == 2
    assert pacer.statistics.overrun_count == 2
    assert pacer.statistics.max_lateness_sec == pytest.approx(0.015)
    assert pacer.statistics.total_yield_requested_sec == 0.0
