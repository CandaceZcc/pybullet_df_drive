# 手动演示测试：保护 GUI 手动模式的退出时长策略。
import pytest

from slope_sim.manual_demo import manual_step_limit


def test_manual_step_limit_is_unbounded_without_explicit_duration():
    assert manual_step_limit(duration_limit_sec=None, time_step=1.0 / 240.0) is None


def test_manual_step_limit_uses_explicit_duration_when_given():
    assert manual_step_limit(duration_limit_sec=1.0, time_step=0.25) == 4
    assert manual_step_limit(duration_limit_sec=0.01, time_step=0.25) == 1


def test_manual_step_limit_rejects_invalid_duration_or_time_step():
    with pytest.raises(ValueError):
        manual_step_limit(duration_limit_sec=0.0, time_step=0.25)
    with pytest.raises(ValueError):
        manual_step_limit(duration_limit_sec=1.0, time_step=0.0)
