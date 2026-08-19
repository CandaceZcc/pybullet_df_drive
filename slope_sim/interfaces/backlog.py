"""异步写入积压判据：共享给 DIRECT 与真实 eCAL 验收器。"""

from __future__ import annotations

from collections.abc import Sequence


def _has_sustained_backlog(
    samples: Sequence[tuple[float, int, int]],
) -> bool:
    """用队列深度和完成数区分真实积压与稳定的单项 in-flight。"""
    if not samples:
        return False

    previous_at, previous_depth, previous_completed = samples[0]
    growth_since: float | None = None
    stalled_since = previous_at if previous_depth > 0 else None
    for sampled_at, depth, completed in samples[1:]:
        if depth <= 0:
            growth_since = None
            stalled_since = None
        else:
            # 完成数前进证明 writer 正在消费，而非同一任务停滞。
            if completed > previous_completed:
                stalled_since = sampled_at
            elif stalled_since is None:
                stalled_since = previous_at if previous_depth > 0 else sampled_at
            if stalled_since is not None and sampled_at - stalled_since >= 1.0:
                return True

            # 多项积压从首次观测到增长时计时；单项 in-flight 不启动该门禁。
            if depth < previous_depth:
                growth_since = None
            elif depth > previous_depth and depth > 1 and growth_since is None:
                growth_since = sampled_at
            if growth_since is not None and sampled_at - growth_since >= 1.0:
                return True

        previous_at = sampled_at
        previous_depth = depth
        previous_completed = completed
    return False
