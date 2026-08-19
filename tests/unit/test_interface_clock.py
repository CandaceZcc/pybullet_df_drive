# 仿真接口时钟单元测试：锁定精确推进、周期期限和输入边界。
from __future__ import annotations

from decimal import Decimal
from fractions import Fraction
import math

import pytest

from slope_sim.interfaces import clock as interface_clock
from slope_sim.interfaces.clock import PeriodicScheduler, SimulationClock


UINT64_MAX = (1 << 64) - 1


def test_simulation_clock_starts_at_zero_and_only_explicit_advance_moves_time():
    clock = SimulationClock()

    assert clock.now_ns == 0
    assert clock.now_ns == 0
    assert clock.advance(Fraction(1, 240)) == 4_166_667
    assert clock.now_ns == 4_166_667


def test_simulation_clock_accepts_positive_real_fraction_and_normalized_float():
    clock = SimulationClock()

    assert clock.advance(1) == 1_000_000_000
    assert clock.advance(Fraction(1, 4)) == 1_250_000_000
    for _ in range(10):
        now_ns = clock.advance(0.1)
    assert now_ns == 2_250_000_000


def test_simulation_clock_accepts_uint64_max_and_overflow_failure_is_atomic():
    clock = SimulationClock()

    assert clock.advance(Fraction(UINT64_MAX, 1_000_000_000)) == UINT64_MAX
    with pytest.raises(ValueError, match="uint64"):
        clock.advance(Fraction(1, 1_000_000_000))
    assert clock.now_ns == UINT64_MAX


def test_simulation_clock_preview_matches_advance_without_mutating_time():
    clock = SimulationClock()
    assert clock.advance(Fraction(1, 4)) == 250_000_000

    candidate = clock.preview_advance(Fraction(1, 240))

    assert candidate == 254_166_667
    assert clock.now_ns == 250_000_000
    assert clock.advance(Fraction(1, 240)) == candidate


def test_simulation_clock_preview_overflow_and_invalid_dt_are_atomic():
    clock = SimulationClock()
    assert clock.advance(Fraction(UINT64_MAX - 1, 1_000_000_000)) == UINT64_MAX - 1

    with pytest.raises(ValueError, match="uint64"):
        clock.preview_advance(Fraction(2, 1_000_000_000))
    with pytest.raises(ValueError, match="dt"):
        clock.preview_advance(0)

    assert clock.now_ns == UINT64_MAX - 1
    assert clock.preview_advance(Fraction(1, 1_000_000_000)) == UINT64_MAX
    assert clock.now_ns == UINT64_MAX - 1


@pytest.mark.parametrize(
    "invalid",
    (True, False, 0, -1, Fraction(0), Fraction(-1, 3), math.nan, math.inf, -math.inf, "0.1", None),
)
def test_simulation_clock_rejects_invalid_dt(invalid):
    with pytest.raises(ValueError, match="dt"):
        SimulationClock().advance(invalid)


def test_scheduler_first_deadline_is_one_period_not_zero_at_rate_boundaries():
    one_hz = PeriodicScheduler(1)
    one_ghz = PeriodicScheduler(1_000_000_000)

    assert one_hz.pop_due(0) == ()
    assert one_hz.pop_due(999_999_999) == ()
    assert one_hz.pop_due(1_000_000_000) == (1_000_000_000,)
    assert one_ghz.pop_due(0) == ()
    assert one_ghz.pop_due(1) == (1,)


@pytest.mark.parametrize(
    "invalid",
    (
        True,
        False,
        0,
        -0.05,
        Fraction(11, 100),
        math.nan,
        math.inf,
        -math.inf,
        "0.05",
        Decimal("0.05"),
    ),
)
def test_scheduler_rejects_first_deadline_outside_one_period(invalid):
    with pytest.raises(ValueError, match="first_deadline_sec"):
        PeriodicScheduler(10, first_deadline_sec=invalid)


@pytest.mark.parametrize(
    ("first_deadline_sec", "now_ns", "expected"),
    (
        (Fraction(5, 2_000_000_000), 3, (2,)),
        (Fraction(7, 2_000_000_000), 4, (4,)),
    ),
)
def test_scheduler_first_deadline_uses_exact_ties_to_even_rounding(
    first_deadline_sec,
    now_ns,
    expected,
):
    scheduler = PeriodicScheduler(
        100_000_000,
        first_deadline_sec=first_deadline_sec,
    )

    assert scheduler.pop_due(now_ns) == expected


def test_scheduler_rounds_each_absolute_phase_deadline_before_emitting() -> None:
    scheduler = PeriodicScheduler(
        1_024,
        first_deadline_sec=Fraction(1, 1_000_000_000),
    )

    assert scheduler.pop_due(2_929_689) == (
        1,
        976_564,
        1_953_126,
        2_929_688,
    )


def test_scheduler_fractional_first_deadline_stays_phase_locked_without_drift():
    scheduler = PeriodicScheduler(10, first_deadline_sec=Fraction(1, 20))

    stamps = scheduler.pop_due(10_000_000_000)

    assert stamps == tuple(
        50_000_000 + index * 100_000_000
        for index in range(100)
    )


@pytest.mark.parametrize(
    "invalid",
    (True, False, 0, -1, 1.0, Fraction(1), 1_000_000_001, "100", None),
)
def test_scheduler_rejects_invalid_rate(invalid):
    with pytest.raises(ValueError, match="rate_hz"):
        PeriodicScheduler(invalid)


def test_scheduler_catches_up_each_crossed_deadline_and_same_now_is_idempotent():
    scheduler = PeriodicScheduler(100)

    assert scheduler.pop_due(35_000_000) == (10_000_000, 20_000_000, 30_000_000)
    assert scheduler.pop_due(35_000_000) == ()
    assert scheduler.pop_due(39_999_999) == ()
    assert scheduler.pop_due(40_000_000) == (40_000_000,)


def test_scheduler_preview_matches_next_pop_and_never_commits_success() -> None:
    scheduler = PeriodicScheduler(100)

    expected = (10_000_000, 20_000_000, 30_000_000)
    assert scheduler.preview_due(35_000_000) == expected
    assert scheduler.preview_due(35_000_000) == expected
    assert scheduler.pop_due(35_000_000) == expected
    assert scheduler.preview_due(40_000_000) == (40_000_000,)
    assert scheduler.pop_due(40_000_000) == (40_000_000,)


def test_scheduler_rejects_time_rollback():
    scheduler = PeriodicScheduler(100)
    scheduler.pop_due(35_000_000)

    with pytest.raises(ValueError, match="now_ns"):
        scheduler.pop_due(34_999_999)


@pytest.mark.parametrize(
    "invalid",
    (True, False, -1, 1.0, Fraction(1), UINT64_MAX + 1, "1", None),
)
def test_scheduler_rejects_non_uint64_now(invalid):
    with pytest.raises(ValueError, match="now_ns"):
        PeriodicScheduler(100).pop_due(invalid)


def test_scheduler_returns_all_257_due_deadlines():
    stamps = PeriodicScheduler(1_000_000_000).pop_due(257)

    assert stamps == tuple(range(1, 258))


def test_scheduler_catch_up_limit_is_10000_and_exact_boundary_is_complete():
    assert interface_clock.MAX_CATCH_UP_DEADLINES == 10_000

    stamps = PeriodicScheduler(1_000_000_000).pop_due(10_000)

    assert stamps == tuple(range(1, 10_001))


def test_scheduler_rejects_oversized_catch_up_before_mutating_state():
    scheduler = PeriodicScheduler(100)
    assert scheduler.pop_due(10_000_000) == (10_000_000,)

    oversized_now_ns = (interface_clock.MAX_CATCH_UP_DEADLINES + 2) * 10_000_000
    with pytest.raises(ValueError, match="catch-up limit"):
        scheduler.pop_due(oversized_now_ns)
    with pytest.raises(ValueError, match="catch-up limit"):
        scheduler.pop_due(UINT64_MAX)

    assert scheduler.pop_due(20_000_000) == (20_000_000,)


def test_scheduler_preview_failure_and_success_both_leave_state_unchanged() -> None:
    scheduler = PeriodicScheduler(100)
    assert scheduler.pop_due(10_000_000) == (10_000_000,)
    oversized_now_ns = (interface_clock.MAX_CATCH_UP_DEADLINES + 2) * 10_000_000

    with pytest.raises(ValueError, match="catch-up limit"):
        scheduler.preview_due(oversized_now_ns)
    assert scheduler.preview_due(20_000_000) == (20_000_000,)
    assert scheduler.pop_due(20_000_000) == (20_000_000,)


def test_fractional_scheduler_accumulates_deadlines_without_drift():
    stamps = PeriodicScheduler(3).pop_due(10_000_000_000)
    expected = tuple(
        round(Fraction(index * 1_000_000_000, 3))
        for index in range(1, 31)
    )

    assert stamps == expected
    assert stamps[0] == 333_333_333
    assert stamps[-1] == 10_000_000_000
    assert set(right - left for left, right in zip(stamps, stamps[1:])) == {
        333_333_333,
        333_333_334,
    }


def test_scheduler_integer_rounding_matches_fraction_ties_to_even():
    scheduler = PeriodicScheduler(1_024)

    assert scheduler.pop_due(2_929_688) == (976_562, 1_953_125, 2_929_688)


def test_240_hz_steps_emit_exact_100_and_10_hz_counts_for_ten_seconds():
    clock = SimulationClock()
    wheel = PeriodicScheduler(100)
    sensor = PeriodicScheduler(10)
    wheel_stamps: list[int] = []
    sensor_stamps: list[int] = []

    for _ in range(2_400):
        now_ns = clock.advance(1.0 / 240.0)
        wheel_stamps.extend(wheel.pop_due(now_ns))
        sensor_stamps.extend(sensor.pop_due(now_ns))

    assert len(wheel_stamps) == 1_000
    assert len(sensor_stamps) == 100
    assert wheel_stamps[-1] == sensor_stamps[-1] == 10_000_000_000
    assert set(right - left for left, right in zip(wheel_stamps, wheel_stamps[1:])) == {10_000_000}
    assert set(right - left for left, right in zip(sensor_stamps, sensor_stamps[1:])) == {100_000_000}
