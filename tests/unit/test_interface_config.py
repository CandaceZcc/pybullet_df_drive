# 企业接口配置单元测试：锁定六个话题、传输模式和有界队列参数。
from __future__ import annotations

import dataclasses
from dataclasses import FrozenInstanceError
import math

import pytest

from slope_sim.interfaces.config import ChannelConfig, InterfaceConfig


EXPECTED_CHANNELS = (
    ("/sim/wheel/command", 100, "subscribe"),
    ("/sim/wheel/state", 100, "publish"),
    ("/sim/lidar/front/points", 10, "publish"),
    ("/sim/lidar/rear/points", 10, "publish"),
    ("/sim/rtk/state", 10, "publish"),
    ("/sim/imu/attitude", 10, "publish"),
)


@pytest.mark.parametrize("transport_mode", ["auto", "ecal", "local"])
def test_default_returns_six_exact_unique_channels(transport_mode: str):
    config = InterfaceConfig.default(transport_mode=transport_mode)

    assert config.transport_mode == transport_mode
    assert tuple((channel.topic, channel.rate_hz, channel.direction) for channel in config.channels) == EXPECTED_CHANNELS
    assert config.channels == (
        config.wheel_command,
        config.wheel_state,
        config.lidar_front,
        config.lidar_rear,
        config.rtk,
        config.imu,
    )
    assert len({channel.topic for channel in config.channels}) == 6


def test_interface_defaults_keep_timeout_window_and_queue_sizes():
    config = InterfaceConfig.default()

    assert config.command_timeout_sec == pytest.approx(0.100)
    assert config.status_window_sec == pytest.approx(2.0)
    assert config.outgoing_queue_size == 32
    assert config.log_queue_size == 256


def test_channel_and_interface_configs_are_frozen():
    config = InterfaceConfig.default()

    with pytest.raises(FrozenInstanceError):
        config.wheel_command.topic = "/changed"
    with pytest.raises(FrozenInstanceError):
        config.transport_mode = "local"


@pytest.mark.parametrize("transport_mode", ["", "fake", "AUTO", None, 1])
def test_interface_config_rejects_each_unknown_transport_mode(transport_mode):
    with pytest.raises(ValueError, match="transport_mode"):
        InterfaceConfig.default(transport_mode=transport_mode)


@pytest.mark.parametrize("field_name", ["command_timeout_sec", "status_window_sec"])
@pytest.mark.parametrize("invalid", [True, 0.0, -0.1, math.nan, math.inf, -math.inf])
def test_interface_config_rejects_each_invalid_time_window(field_name: str, invalid):
    with pytest.raises(ValueError, match=field_name):
        dataclasses.replace(InterfaceConfig.default(), **{field_name: invalid})


@pytest.mark.parametrize("field_name", ["outgoing_queue_size", "log_queue_size"])
@pytest.mark.parametrize("invalid", [True, 0, -1, 1.5])
def test_interface_config_rejects_each_invalid_queue_size(field_name: str, invalid):
    with pytest.raises(ValueError, match=field_name):
        dataclasses.replace(InterfaceConfig.default(), **{field_name: invalid})


def test_channel_config_accepts_nonempty_relative_topic():
    channel = ChannelConfig("sim/relative", 10, "publish")

    assert channel.topic == "sim/relative"


@pytest.mark.parametrize("topic", ["", None, 1])
def test_channel_config_rejects_each_invalid_topic(topic):
    with pytest.raises(ValueError, match="topic"):
        ChannelConfig(topic, 10, "publish")


@pytest.mark.parametrize("rate_hz", [True, False, 0, -1, 1.5])
def test_channel_config_rejects_each_invalid_rate(rate_hz):
    with pytest.raises(ValueError, match="rate_hz"):
        ChannelConfig("/topic", rate_hz, "publish")


@pytest.mark.parametrize("direction", ["", "send", "Publish", None, 1])
def test_channel_config_rejects_each_invalid_direction(direction):
    with pytest.raises(ValueError, match="direction"):
        ChannelConfig("/topic", 10, direction)


def test_interface_config_rejects_duplicate_topics_after_replace():
    config = InterfaceConfig.default()
    duplicate_imu = dataclasses.replace(config.imu, topic=config.rtk.topic)

    with pytest.raises(ValueError, match="duplicate"):
        dataclasses.replace(config, imu=duplicate_imu)


def test_dataclasses_replace_revalidates_channel_and_interface_scalars():
    config = InterfaceConfig.default()

    with pytest.raises(ValueError, match="topic"):
        dataclasses.replace(config.imu, topic="")
    with pytest.raises(ValueError, match="status_window_sec"):
        dataclasses.replace(config, status_window_sec=0.0)
