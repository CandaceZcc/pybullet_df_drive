"""写入积压单元测试：统一 DIRECT 与 eCAL 验收器的队列判据。"""

from __future__ import annotations

import pytest

from slope_sim.interfaces.backlog import _has_sustained_backlog


@pytest.mark.parametrize(
    ("samples", "expected"),
    [
        (((0.0, 1, 10), (0.5, 1, 120), (1.1, 1, 240)), False),
        (((0.0, 1, 10), (0.1, 2, 40), (1.2, 3, 240)), True),
        (((0.0, 0, 0), (0.1, 8, 10), (0.6, 8, 50), (1.2, 8, 100)), True),
        (((0.0, 1, 10), (0.9, 2, 100), (1.0, 2, 110)), False),
        (((0.0, 1, 10), (0.5, 1, 10), (1.1, 1, 10)), True),
        (
            (
                (0.7, 1, 271),
                (1.1, 1, 367),
                (1.5, 1, 463),
                (1.9, 1, 557),
                (2.3, 1, 651),
                (2.4, 4, 670),
                (2.51, 3, 696),
                (2.6, 0, 718),
            ),
            False,
        ),
    ],
    ids=(
        "writer-progress",
        "monotonic-growth",
        "plateau-after-idle",
        "no-backdating",
        "stalled-inflight",
        "short-peak-after-stable-inflight",
    ),
)
def test_sustained_backlog_boundaries(
    samples: tuple[tuple[float, int, int], ...],
    expected: bool,
) -> None:
    assert _has_sustained_backlog(samples) is expected
