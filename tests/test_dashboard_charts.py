# Dashboard 纯图表测试：锁定接口缓存的代际、时间、车型和质量统计契约。
from __future__ import annotations

from dataclasses import replace
import math

import pytest

from slope_sim.dashboard_charts import InterfaceChartBuffer, interface_chart_specs
from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.dashboard_snapshot import InterfaceDashboardSnapshot
from slope_sim.interfaces.models import ImuAttitude, RtkState, WheelCommand, WheelState
from slope_sim.interfaces.status import InterfaceStatusSnapshot, TopicStatus, WheelCommandStatus
from slope_sim.model_registry import get_robot_model, robot_model_names


BUSINESS_TABS = {
    "驱动命令",
    "驱动反馈",
    "转向命令",
    "转向反馈",
    "RTK位置",
    "RTK航向",
    "IMU姿态",
}
QUALITY_TABS = {"轮组频率", "传感频率", "接口异常"}


def _status(
    config: InterfaceConfig,
    captured_at: float,
    *,
    errors: int = 0,
    drops: int = 0,
    message_count: int = 1,
) -> InterfaceStatusSnapshot:
    topics = {
        channel.topic: TopicStatus(
            topic=channel.topic,
            direction=channel.direction,
            state="active",
            target_hz=float(channel.rate_hz),
            actual_hz=float(channel.rate_hz),
            latest_timestamp_ns=1,
            message_count=message_count,
            error_count=errors,
            dropped_count=drops,
        )
        for channel in config.channels
    }
    return InterfaceStatusSnapshot(
        captured_at=captured_at,
        transport_mode="local",
        ecal_connected=False,
        command=WheelCommandStatus("active", 100.0, 1, message_count, errors),
        wheel_state=None,
        topics=topics,
    )


def _snapshot(
    *,
    generation: int = 1,
    robot_model: str = "active_steering_4wd",
    timestamp_ns: int = 1_000_000_000,
    captured_at: float = 1.0,
    errors: int = 0,
    drops: int = 0,
    command: WheelCommand | None = None,
    wheel_state: WheelState | None = None,
    rtk: RtkState | None = None,
    imu: ImuAttitude | None = None,
) -> InterfaceDashboardSnapshot:
    model = get_robot_model(robot_model)
    drive_count = len(model.drive_joint_names)
    steering_count = len(model.steering_joint_names)
    command = command or WheelCommand(
        999,
        tuple(float(index + 1) for index in range(drive_count)),
        tuple(float(index + 1) / 10.0 for index in range(steering_count)),
    )
    wheel_state = wheel_state or WheelState(
        timestamp_ns,
        tuple(float(index + 11) for index in range(drive_count)),
        tuple(float(index + 1) / 20.0 for index in range(steering_count)),
    )
    return InterfaceDashboardSnapshot(
        generation=generation,
        robot_model=robot_model,
        sim_time_ns=timestamp_ns,
        status=_status(config=InterfaceConfig.default(), captured_at=captured_at, errors=errors, drops=drops),
        wheel_command=command,
        wheel_command_received_sim_time_ns=timestamp_ns,
        wheel_state=wheel_state,
        lidar_front=None,
        lidar_rear=None,
        rtk=rtk or RtkState(timestamp_ns, 1.0, 2.0, 3.0, 0.4),
        imu=imu or ImuAttitude(timestamp_ns, 0.1, -0.2),
        lidar_front_view=None,
        lidar_rear_view=None,
    )


def test_interface_chart_buffer_validates_constructor_and_exact_snapshot_type() -> None:
    config = InterfaceConfig.default()
    for invalid in (0.0, -1.0, math.nan, math.inf, True, "20"):
        with pytest.raises(ValueError, match="window_sec"):
            InterfaceChartBuffer(invalid, config)
    with pytest.raises(ValueError, match="interface_config"):
        InterfaceChartBuffer(20.0, object())

    buffer = InterfaceChartBuffer(20.0, config)
    with pytest.raises(ValueError, match="InterfaceDashboardSnapshot"):
        buffer.append(object())


def test_interface_chart_buffer_deduplicates_and_clears_everything_on_generation_change() -> None:
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    first = _snapshot(generation=1, timestamp_ns=1_000_000_000, captured_at=10.0)

    assert buffer.append(first) == BUSINESS_TABS | QUALITY_TABS
    assert buffer.append(first) == set()
    quality_only = replace(first, status=replace(first.status, captured_at=10.2))
    assert buffer.append(quality_only) == QUALITY_TABS

    replacement = _snapshot(generation=2, timestamp_ns=2_000_000_000, captured_at=20.0)
    buffer.append(replacement)
    assert buffer.series("驱动反馈")["t"] == [2.0]
    assert buffer.series("轮组频率")["t"] == [20.0]
    assert buffer.series("接口异常")["errors_per_sec"] == [0.0]


def test_interface_chart_buffer_keeps_closed_twenty_second_window() -> None:
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    for timestamp_sec in (1.0, 21.0, 21.000000001):
        buffer.append(
            _snapshot(
                timestamp_ns=round(timestamp_sec * 1_000_000_000),
                captured_at=timestamp_sec,
            )
        )

    assert buffer.series("RTK位置")["t"] == [21.0, 21.000000001]


def test_silent_business_topics_expire_against_snapshot_sim_time_horizon() -> None:
    """即使 latest 消息不变，统一仿真时间推进也必须淘汰静默页旧行。"""
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    first = _snapshot(timestamp_ns=1_000_000_000, captured_at=1.0)
    buffer.append(first)

    silent = replace(
        first,
        sim_time_ns=21_000_000_001,
        status=replace(first.status, captured_at=2.0),
    )
    changed = buffer.append(silent)

    assert BUSINESS_TABS <= changed
    for tab_label in BUSINESS_TABS:
        assert buffer.series(tab_label)["t"] == []


def test_paused_frozen_sim_time_does_not_expire_business_history() -> None:
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    first = _snapshot(timestamp_ns=1_000_000_000, captured_at=1.0)
    buffer.append(first)

    changed = buffer.append(
        replace(first, status=replace(first.status, captured_at=30.0)),
        paused=True,
    )

    assert not (BUSINESS_TABS & changed)
    assert buffer.series("RTK位置")["t"] == [1.0]
    assert buffer.series("驱动命令")["t"] == [1.0]


def test_quality_horizon_prunes_silent_invalid_page_without_appending() -> None:
    """质量字段无效时仍按 captured_at 裁剪该页，且 horizon 不可倒退。"""
    config = InterfaceConfig.default()
    buffer = InterfaceChartBuffer(20.0, config)
    first = _snapshot(timestamp_ns=1_000_000_000, captured_at=1.0)
    buffer.append(first)

    invalid_front = first.status.topics[config.lidar_front.topic]
    object.__setattr__(invalid_front, "actual_hz", math.nan)
    advanced = replace(
        first,
        sim_time_ns=1_000_000_000,
        status=replace(first.status, captured_at=21.000000001),
    )
    changed = buffer.append(advanced, paused=True)

    assert "传感频率" in changed
    assert buffer.series("传感频率")["t"] == []

    backwards = replace(advanced, status=replace(advanced.status, captured_at=20.0))
    assert buffer.append(backwards, paused=True) == set()
    assert buffer.series("传感频率")["t"] == []


def test_reverse_or_duplicate_topic_time_rejects_only_that_business_sample() -> None:
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    first = _snapshot(timestamp_ns=2_000_000_000, captured_at=1.0)
    buffer.append(first)
    reverse_wheel = WheelState(1_000_000_000, (9.0, 8.0, 7.0, 6.0), (0.3, 0.4))
    newer_rtk = RtkState(3_000_000_000, 7.0, 8.0, 9.0, 1.2)

    changed = buffer.append(
        replace(
            first,
            status=replace(first.status, captured_at=2.0),
            wheel_state=reverse_wheel,
            rtk=newer_rtk,
        )
    )

    assert "驱动反馈" not in changed
    assert buffer.series("驱动反馈")["t"] == [2.0]
    assert buffer.series("RTK位置")["t"] == [2.0, 3.0]
    assert buffer.series("RTK位置")["rtk_x"][-1] == 7.0


def test_paused_snapshot_freezes_business_but_quality_continues() -> None:
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    buffer.append(_snapshot(timestamp_ns=1_000_000_000, captured_at=1.0))

    changed = buffer.append(
        _snapshot(timestamp_ns=2_000_000_000, captured_at=2.0),
        paused=True,
    )

    assert changed == QUALITY_TABS
    assert buffer.series("驱动命令")["t"] == [1.0]
    assert buffer.series("RTK位置")["t"] == [1.0]
    assert buffer.series("轮组频率")["t"] == [1.0, 2.0]


@pytest.mark.parametrize("robot_model", robot_model_names())
def test_interface_chart_buffer_accepts_all_four_model_wheel_shapes(robot_model: str) -> None:
    model = get_robot_model(robot_model)
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    buffer.append(_snapshot(robot_model=robot_model))

    drive = buffer.series("驱动命令")
    steering = buffer.series("转向命令")
    assert len([name for name in drive if name.startswith("drive_command_")]) == len(model.drive_joint_names)
    assert len([name for name in steering if name.startswith("steering_command_")]) == len(model.steering_joint_names)
    assert steering["t"] == ([1.0] if model.steering_joint_names else [])


def test_same_generation_model_change_resets_history_and_accepts_new_shape() -> None:
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    buffer.append(
        _snapshot(
            generation=1,
            robot_model="df_back",
            timestamp_ns=1_000_000_000,
            captured_at=1.0,
        )
    )

    changed = buffer.append(
        _snapshot(
            generation=1,
            robot_model="active_steering_4wd",
            timestamp_ns=2_000_000_000,
            captured_at=2.0,
        )
    )

    assert BUSINESS_TABS | QUALITY_TABS <= changed
    drive = buffer.series("驱动命令")
    steering = buffer.series("转向命令")
    assert drive["t"] == [2.0]
    assert [key for key in drive if key.startswith("drive_command_")] == [
        "drive_command_0",
        "drive_command_1",
        "drive_command_2",
        "drive_command_3",
    ]
    assert steering["t"] == [2.0]
    assert [key for key in steering if key.startswith("steering_command_")] == [
        "steering_command_0",
        "steering_command_1",
    ]


def test_wrong_wheel_lengths_do_not_pollute_other_topics() -> None:
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    malformed = _snapshot(
        command=WheelCommand(5, (1.0, 2.0), ()),
        wheel_state=WheelState(1_000_000_000, (1.0, 2.0), ()),
    )

    changed = buffer.append(malformed)

    assert not ({"驱动命令", "转向命令", "驱动反馈", "转向反馈"} & changed)
    assert BUSINESS_TABS - {"驱动命令", "转向命令", "驱动反馈", "转向反馈"} <= changed


def test_non_finite_rtk_sample_does_not_block_other_topic() -> None:
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    first = _snapshot(timestamp_ns=1_000_000_000, captured_at=1.0)
    buffer.append(first)
    invalid_rtk = RtkState(2_000_000_000, 1.0, 2.0, 3.0, 0.4)
    object.__setattr__(invalid_rtk, "main_x", math.nan)

    changed = buffer.append(
        replace(
            first,
            status=replace(first.status, captured_at=2.0),
            rtk=invalid_rtk,
            imu=ImuAttitude(2_000_000_000, 0.3, 0.4),
        )
    )

    assert "RTK位置" not in changed
    assert buffer.series("RTK位置")["t"] == [1.0]
    assert "IMU姿态" in changed
    assert buffer.series("IMU姿态")["t"] == [1.0, 2.0]


def test_interface_chart_specs_keep_confirmed_line_density_for_active_steering() -> None:
    specs = interface_chart_specs(get_robot_model("active_steering_4wd"))
    assert {spec.tab_label: len(spec.lines) for spec in specs} == {
        "驱动命令": 4,
        "驱动反馈": 4,
        "转向命令": 2,
        "转向反馈": 2,
        "RTK位置": 3,
        "RTK航向": 1,
        "IMU姿态": 2,
        "轮组频率": 2,
        "传感频率": 4,
        "接口异常": 2,
    }
    assert all(not hasattr(spec, "__dict__") for spec in specs)
    assert all(not hasattr(line, "__dict__") for spec in specs for line in spec.lines)


@pytest.mark.parametrize("robot_model", ("df_front", "df_mid", "df_back"))
def test_differential_specs_use_two_real_drive_lines_and_no_steering_lines(robot_model: str) -> None:
    specs = {spec.tab_label: spec for spec in interface_chart_specs(get_robot_model(robot_model))}
    assert len(specs["驱动命令"].lines) == len(specs["驱动反馈"].lines) == 2
    assert specs["转向命令"].lines == specs["转向反馈"].lines == ()


def test_quality_rates_use_monotonic_time_and_reset_counter_rollbacks() -> None:
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    buffer.append(_snapshot(generation=1, captured_at=10.0, errors=5, drops=3))
    buffer.append(_snapshot(generation=1, captured_at=11.0, errors=7, drops=4))
    assert buffer.series("接口异常")["errors_per_sec"] == [0.0, 12.0]
    assert buffer.series("接口异常")["drops_per_sec"] == [0.0, 6.0]

    rollback = _snapshot(generation=1, captured_at=12.0, errors=0, drops=0)
    buffer.append(rollback)
    assert buffer.series("接口异常")["errors_per_sec"][-1] == 0.0
    assert buffer.series("接口异常")["drops_per_sec"][-1] == 0.0

    backwards = replace(rollback, status=replace(rollback.status, captured_at=11.5))
    assert buffer.append(backwards) == set()
    assert buffer.series("接口异常")["t"] == [10.0, 11.0, 12.0]


def test_series_returns_new_mutable_copies_and_clear_resets_baselines() -> None:
    buffer = InterfaceChartBuffer(20.0, InterfaceConfig.default())
    buffer.append(_snapshot(captured_at=1.0, errors=1))
    exposed = buffer.series("接口异常")
    exposed["t"].append(99.0)
    exposed["errors_per_sec"][0] = 99.0
    assert buffer.series("接口异常")["t"] == [1.0]
    assert buffer.series("接口异常")["errors_per_sec"] == [0.0]

    buffer.clear()
    assert buffer.series("接口异常")["t"] == []
    buffer.append(_snapshot(captured_at=2.0, errors=20))
    assert buffer.series("接口异常")["errors_per_sec"] == [0.0]
