# Dashboard 只读边界单元测试：锁定快照冻结性、严格类型和前后雷达配对契约。
from __future__ import annotations

from dataclasses import FrozenInstanceError, fields
import math

import pytest

from slope_sim.interfaces.dashboard_snapshot import (
    InterfaceDashboardSnapshot,
    LidarTopViewFrame,
    LidarTopViewPoint,
)
from slope_sim.interfaces.models import (
    ImuAttitude,
    LidarPoint,
    LidarPointCloud,
    RtkState,
    WheelCommand,
    WheelState,
)
from slope_sim.interfaces.status import InterfaceStatusSnapshot, WheelCommandStatus


UINT64_MAX = (1 << 64) - 1


class MutableTopViewPoint(LidarTopViewPoint):
    """模拟继承冻结模型后重新获得 __dict__ 的不安全边界值。"""


class MutableTopViewFrame(LidarTopViewFrame):
    pass


class MutableStatus(InterfaceStatusSnapshot):
    pass


class MutableWheelCommand(WheelCommand):
    pass


class MutableWheelState(WheelState):
    pass


class MutableLidarPointCloud(LidarPointCloud):
    pass


class MutableRtkState(RtkState):
    pass


class MutableImuAttitude(ImuAttitude):
    pass


def _status() -> InterfaceStatusSnapshot:
    return InterfaceStatusSnapshot(
        0.0,
        "local",
        False,
        WheelCommandStatus("waiting_command", 0.0, None, 0, 0, None),
        None,
        {},
    )


def _cloud(side: str, timestamp_ns: int = 7) -> LidarPointCloud:
    lidar_id = 1 if side == "front" else 2
    point = LidarPoint(0, 1.0, 2.0, 3.0, 100, 1, 0)
    return LidarPointCloud(timestamp_ns, f"lidar_{side}", 1, lidar_id, (point,))


def _view(side: str, timestamp_ns: int = 7) -> LidarTopViewFrame:
    lidar_id = 1 if side == "front" else 2
    return LidarTopViewFrame(
        timestamp_ns,
        (LidarTopViewPoint(1.0, 2.0, 1, lidar_id),),
    )


def _snapshot(**changes: object) -> InterfaceDashboardSnapshot:
    values: dict[str, object] = {
        "generation": 0,
        "robot_model": "df_mid",
        "sim_time_ns": 0,
        "status": _status(),
        "wheel_command": None,
        "wheel_command_received_sim_time_ns": None,
        "wheel_state": None,
        "lidar_front": None,
        "lidar_rear": None,
        "rtk": None,
        "imu": None,
        "lidar_front_view": None,
        "lidar_rear_view": None,
    }
    values.update(changes)
    return InterfaceDashboardSnapshot(**values)


def test_top_view_point_is_frozen_slotted_and_normalizes_real_coordinates() -> None:
    point = LidarTopViewPoint(1, 2.5, 3, 2)

    assert (point.x, point.y, point.tag, point.lidar_id) == (1.0, 2.5, 3, 2)
    assert not hasattr(point, "__dict__")
    with pytest.raises(FrozenInstanceError):
        point.x = 9.0


@pytest.mark.parametrize("field_name", ("x", "y"))
@pytest.mark.parametrize("invalid", (True, False, math.nan, math.inf, -math.inf, "1"))
def test_top_view_point_rejects_non_finite_or_bool_coordinates(
    field_name: str,
    invalid: object,
) -> None:
    values = {"x": 1.0, "y": 2.0, "tag": 0, "lidar_id": 1}
    values[field_name] = invalid

    with pytest.raises(ValueError, match="finite"):
        LidarTopViewPoint(**values)


@pytest.mark.parametrize("invalid", (True, 1.0, -1, 4))
def test_top_view_point_rejects_invalid_tag(invalid: object) -> None:
    with pytest.raises(ValueError, match="tag"):
        LidarTopViewPoint(0.0, 0.0, invalid, 1)


@pytest.mark.parametrize("invalid", (True, 1.0, 0, 3))
def test_top_view_point_rejects_invalid_lidar_id(invalid: object) -> None:
    with pytest.raises(ValueError, match="lidar_id"):
        LidarTopViewPoint(0.0, 0.0, 0, invalid)


def test_top_view_frame_copies_ordered_sequence_and_is_frozen_slotted() -> None:
    first = LidarTopViewPoint(1.0, 2.0, 1, 1)
    second = LidarTopViewPoint(3.0, 4.0, 2, 1)
    source = [first]

    frame = LidarTopViewFrame(5, source)
    source.append(second)

    assert frame.points == (first,)
    assert not hasattr(frame, "__dict__")
    with pytest.raises(FrozenInstanceError):
        frame.timestamp_ns = 6


@pytest.mark.parametrize("invalid", (True, -1, UINT64_MAX + 1, 1.0))
def test_top_view_frame_rejects_non_uint64_timestamp(invalid: object) -> None:
    with pytest.raises(ValueError, match="uint64"):
        LidarTopViewFrame(invalid, ())


@pytest.mark.parametrize(
    "factory",
    (
        lambda point: "point",
        lambda point: b"point",
        lambda point: {point},
        lambda point: frozenset({point}),
        lambda point: {"point": point},
        lambda point: (item for item in (point,)),
    ),
)
def test_top_view_frame_rejects_unordered_mapping_or_one_shot_points(factory) -> None:
    point = LidarTopViewPoint(0.0, 0.0, 0, 1)

    with pytest.raises(ValueError, match="ordered sequence"):
        LidarTopViewFrame(0, factory(point))


def test_top_view_frame_rejects_non_point_elements() -> None:
    with pytest.raises(ValueError, match="LidarTopViewPoint"):
        LidarTopViewFrame(0, (object(),))


def test_top_view_frame_rejects_lidar_point_subclass_with_extra_mutable_state() -> None:
    point = MutableTopViewPoint(0.0, 0.0, 0, 1)
    object.__setattr__(point, "extra_state", [])

    with pytest.raises(ValueError, match="LidarTopViewPoint"):
        LidarTopViewFrame(0, (point,))


def test_dashboard_snapshot_has_frozen_field_order_and_keeps_payload_identity() -> None:
    command = WheelCommand(99, (1.0, 2.0), ())
    wheel_state = WheelState(10, (1.0, 2.0), ())
    front_cloud = _cloud("front")
    rear_cloud = _cloud("rear")
    rtk = RtkState(10, 1.0, 2.0, 3.0, 0.1)
    imu = ImuAttitude(10, 0.2, -0.3)
    front_view = _view("front")
    rear_view = _view("rear")

    snapshot = _snapshot(
        generation=4,
        sim_time_ns=10,
        wheel_command=command,
        wheel_command_received_sim_time_ns=10,
        wheel_state=wheel_state,
        lidar_front=front_cloud,
        lidar_rear=rear_cloud,
        rtk=rtk,
        imu=imu,
        lidar_front_view=front_view,
        lidar_rear_view=rear_view,
    )

    assert tuple(field.name for field in fields(snapshot)) == (
        "generation",
        "robot_model",
        "sim_time_ns",
        "status",
        "wheel_command",
        "wheel_command_received_sim_time_ns",
        "wheel_state",
        "lidar_front",
        "lidar_rear",
        "rtk",
        "imu",
        "lidar_front_view",
        "lidar_rear_view",
    )
    assert snapshot.wheel_command is command
    assert snapshot.wheel_state is wheel_state
    assert snapshot.lidar_front is front_cloud
    assert snapshot.lidar_front_view is front_view
    assert not hasattr(snapshot, "__dict__")
    with pytest.raises(FrozenInstanceError):
        snapshot.generation = 5


@pytest.mark.parametrize("invalid", (True, -1, 1.0))
def test_dashboard_snapshot_rejects_invalid_generation(invalid: object) -> None:
    with pytest.raises(ValueError, match="generation"):
        _snapshot(generation=invalid)


@pytest.mark.parametrize("invalid", (True, -1, UINT64_MAX + 1, 1.0))
def test_dashboard_snapshot_rejects_non_uint64_sim_time(invalid: object) -> None:
    with pytest.raises(ValueError, match="sim_time_ns"):
        _snapshot(sim_time_ns=invalid)


@pytest.mark.parametrize("invalid", ("tank", "DF_MID", 1))
def test_dashboard_snapshot_rejects_unknown_or_wrong_robot_model(invalid: object) -> None:
    with pytest.raises(ValueError, match="robot_model"):
        _snapshot(robot_model=invalid)


def test_dashboard_snapshot_rejects_wrong_status_and_payload_types() -> None:
    invalid_fields = (
        "status",
        "wheel_command",
        "wheel_state",
        "lidar_front",
        "lidar_rear",
        "rtk",
        "imu",
        "lidar_front_view",
        "lidar_rear_view",
    )
    for field_name in invalid_fields:
        changes: dict[str, object] = {field_name: object()}
        if field_name == "wheel_command":
            changes["wheel_command_received_sim_time_ns"] = 0
        with pytest.raises(ValueError, match=field_name):
            _snapshot(**changes)


@pytest.mark.parametrize(
    ("field_name", "changes"),
    (
        (
            "status",
            {
                "status": MutableStatus(
                    0.0,
                    "local",
                    False,
                    WheelCommandStatus("waiting_command", 0.0, None, 0, 0, None),
                    None,
                    {},
                )
            },
        ),
        (
            "wheel_command",
            {
                "wheel_command": MutableWheelCommand(1, (1.0, 2.0), ()),
                "wheel_command_received_sim_time_ns": 0,
            },
        ),
        ("wheel_state", {"wheel_state": MutableWheelState(1, (1.0, 2.0), ())}),
        (
            "lidar_front",
            {
                "lidar_front": MutableLidarPointCloud(
                    _cloud("front").timebase_ns,
                    "lidar_front",
                    1,
                    1,
                    _cloud("front").points,
                ),
                "lidar_front_view": _view("front"),
            },
        ),
        ("rtk", {"rtk": MutableRtkState(1, 1.0, 2.0, 3.0, 0.1)}),
        ("imu", {"imu": MutableImuAttitude(1, 0.1, -0.2)}),
        (
            "lidar_front_view",
            {
                "lidar_front": _cloud("front"),
                "lidar_front_view": MutableTopViewFrame(
                    7,
                    (LidarTopViewPoint(1.0, 2.0, 1, 1),),
                ),
            },
        ),
    ),
)
def test_dashboard_snapshot_rejects_payload_subclasses(
    field_name: str,
    changes: dict[str, object],
) -> None:
    with pytest.raises(ValueError, match=field_name):
        _snapshot(**changes)


@pytest.mark.parametrize(
    "changes",
    (
        {"wheel_command": WheelCommand(99, (1.0, 2.0), ())},
        {"wheel_command_received_sim_time_ns": 0},
        {
            "wheel_command": WheelCommand(99, (1.0, 2.0), ()),
            "wheel_command_received_sim_time_ns": True,
        },
        {
            "wheel_command": WheelCommand(99, (1.0, 2.0), ()),
            "wheel_command_received_sim_time_ns": -1,
        },
        {
            "wheel_command": WheelCommand(99, (1.0, 2.0), ()),
            "wheel_command_received_sim_time_ns": UINT64_MAX + 1,
        },
    ),
)
def test_dashboard_snapshot_requires_command_and_received_sim_time_as_valid_pair(
    changes: dict[str, object],
) -> None:
    with pytest.raises(ValueError, match="wheel_command_received_sim_time_ns"):
        _snapshot(**changes)


@pytest.mark.parametrize(
    "changes",
    (
        {"lidar_front": _cloud("rear"), "lidar_front_view": _view("front")},
        {"lidar_front": _cloud("front"), "lidar_front_view": _view("rear")},
        {"lidar_rear": _cloud("front"), "lidar_rear_view": _view("rear")},
        {"lidar_rear": _cloud("rear"), "lidar_rear_view": _view("front")},
        {
            "lidar_front": _cloud("front", 7),
            "lidar_front_view": _view("front", 8),
        },
        {
            "lidar_rear": _cloud("rear", 7),
            "lidar_rear_view": _view("rear", 8),
        },
    ),
)
def test_dashboard_snapshot_rejects_lidar_side_or_timestamp_mismatch(
    changes: dict[str, object],
) -> None:
    with pytest.raises(ValueError, match="lidar"):
        _snapshot(**changes)


@pytest.mark.parametrize(
    "changes",
    (
        {"lidar_front": _cloud("front")},
        {"lidar_front_view": _view("front")},
        {
            "lidar_front": _cloud("front"),
            "lidar_front_view": LidarTopViewFrame(7, ()),
        },
        {"lidar_rear": _cloud("rear")},
        {"lidar_rear_view": _view("rear")},
        {
            "lidar_rear": _cloud("rear"),
            "lidar_rear_view": LidarTopViewFrame(7, ()),
        },
    ),
)
def test_dashboard_snapshot_requires_atomic_cloud_view_pair_with_equal_point_count(
    changes: dict[str, object],
) -> None:
    with pytest.raises(ValueError, match="lidar"):
        _snapshot(**changes)


def test_dashboard_snapshot_rejects_cloud_view_point_tag_mismatch() -> None:
    cloud = _cloud("front")
    mismatched_view = LidarTopViewFrame(
        cloud.timebase_ns,
        (LidarTopViewPoint(1.0, 2.0, 2, 1),),
    )

    with pytest.raises(ValueError, match="tag"):
        _snapshot(lidar_front=cloud, lidar_front_view=mismatched_view)


def test_dashboard_snapshot_accepts_uint64_boundaries() -> None:
    command = WheelCommand(UINT64_MAX, (1.0, 2.0), ())
    snapshot = _snapshot(
        sim_time_ns=UINT64_MAX,
        wheel_command=command,
        wheel_command_received_sim_time_ns=UINT64_MAX,
    )

    assert snapshot.sim_time_ns == UINT64_MAX
    assert snapshot.wheel_command_received_sim_time_ns == UINT64_MAX
