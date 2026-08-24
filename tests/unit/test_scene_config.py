# 场景文件单元测试：锁定纯逻辑文档、严格 YAML 结构与可复现场景语义。
from __future__ import annotations

from copy import deepcopy
from dataclasses import FrozenInstanceError, replace
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from slope_sim.lidar_pointcloud import LidarConfig
from slope_sim.obstacles import (
    ObstacleGeometry,
    ObstaclePath,
    ObstacleSnapshot,
    ObstacleSpec,
)
from slope_sim.runtime_actions import TerrainSelection
from slope_sim.scene_config import (
    MAX_SCENE_FILE_BYTES,
    SCENE_SCHEMA_VERSION,
    SceneDocument,
    SensorDocument,
    TerrainDocument,
    document_to_mapping,
    dump_scene_atomic,
    load_scene,
    scene_document_from_mapping,
)
from slope_sim.truth_sensors import MountPose, SensorMounts


IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)


def test_four_wheel_slope_mixed_obstacle_acceptance_scene_is_loadable() -> None:
    """下一轮人工验收可由一条 --scene-in 命令复现四驱、斜坡和十个混合障碍。"""
    scene = load_scene(
        Path(__file__).resolve().parents[2]
        / "configs/acceptance_4wd_slope_10_mixed.yaml"
    )

    assert scene.robot_model == "active_steering_4wd"
    assert scene.terrain.terrain_model == "slope"
    assert scene.terrain.slope_deg == pytest.approx(8.0)
    assert len(scene.obstacles) == 10
    assert sum(item.mode == "static" for item in scene.obstacles) == 5
    assert sum(item.mode == "moving" for item in scene.obstacles) == 5


def sample_scene_document() -> SceneDocument:
    """构造同时包含静态、移动障碍物和完整传感器配置的代表性文档。"""
    moving = ObstacleSpec(
        logical_id=1,
        mode="moving",
        geometry=ObstacleGeometry("cylinder", (0.2, 0.2, 0.4)),
        position=(1.5, -0.5, 0.4),
        orientation=(0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)),
        path=ObstaclePath(
            start_xy=(1.0, -0.5),
            end_xy=(3.0, -0.5),
            speed=0.35,
            progress=0.25,
            direction=-1,
        ),
    )
    static = ObstacleSpec(
        logical_id=2,
        mode="static",
        geometry=ObstacleGeometry("box", (0.3, 0.2, 0.5)),
        position=(-1.0, 0.75, 0.5),
        orientation=(0.0, 0.0, 0.0, 2.0),
    )
    return SceneDocument(
        schema_version=SCENE_SCHEMA_VERSION,
        robot_model="df_back",
        terrain=TerrainDocument(
            terrain_model="golf_heightfield",
            slope_deg=0.0,
            golf_seed=23,
            golf_relief="high",
        ),
        # 故意逆序输入，文档必须按 logical_id 规范化。
        obstacles=(static, moving),
        sensors=SensorDocument(
            mounts=SensorMounts.default(),
            lidar=LidarConfig.default(),
        ),
    )


def sample_scene_mapping() -> dict[str, object]:
    return deepcopy(document_to_mapping(sample_scene_document()))


def _load_mapping(tmp_path, mapping: object) -> SceneDocument:
    path = tmp_path / "scene.yaml"
    path.write_text(
        yaml.safe_dump(mapping, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return load_scene(path)


def test_scene_document_round_trips_all_logical_state_without_temporary_fields(tmp_path):
    document = sample_scene_document()
    path = dump_scene_atomic(document, tmp_path / "scene.yaml")

    assert load_scene(path) == document
    assert tuple(item.logical_id for item in document.obstacles) == (1, 2)
    text = path.read_text(encoding="utf-8")
    assert "schema_version: 1" in text
    for forbidden in (
        "body_id",
        "client_id",
        "link_index",
        "transport_mode",
        "topic",
        "ecal_handle",
        "qt_object",
    ):
        assert forbidden not in text


def test_same_document_has_stable_bytes_and_load_dump_is_stable(tmp_path):
    document = sample_scene_document()
    first = dump_scene_atomic(document, tmp_path / "first.yaml")
    second = dump_scene_atomic(document, tmp_path / "second.yaml")
    third = dump_scene_atomic(load_scene(first), tmp_path / "third.yaml")

    assert first.read_bytes() == second.read_bytes() == third.read_bytes()
    assert b"\r\n" not in first.read_bytes()


def test_structured_mapping_uses_fixed_field_order_and_sorted_obstacles():
    mapping = document_to_mapping(sample_scene_document())

    assert tuple(mapping) == ("schema_version", "robot", "terrain", "obstacles", "sensors")
    assert tuple(mapping["robot"]) == ("model",)
    assert tuple(mapping["terrain"]) == (
        "terrain_model",
        "slope_deg",
        "golf_seed",
        "golf_relief",
    )
    assert [item["logical_id"] for item in mapping["obstacles"]] == [1, 2]
    assert tuple(mapping["obstacles"][0]) == (
        "logical_id",
        "mode",
        "geometry",
        "position",
        "orientation",
        "path",
    )
    assert tuple(mapping["sensors"]) == (
        "lidar_front",
        "lidar_rear",
        "rtk_primary",
        "rtk_secondary",
        "imu",
        "lidar",
    )


def test_from_runtime_discards_body_id_and_round_trips(tmp_path):
    geometry = ObstacleGeometry("sphere", (0.2, 0.2, 0.2))
    snapshot = ObstacleSnapshot(
        logical_id=7,
        body_id=91,
        mode="static",
        shape="sphere",
        geometry=geometry,
        position=(2.0, 1.0, 0.2),
        orientation=IDENTITY_QUATERNION,
    )

    document = SceneDocument.from_runtime(
        "df_front",
        TerrainSelection("flat"),
        (snapshot,),
        SensorMounts.default(),
        lidar_config=LidarConfig.default(),
    )

    assert document.obstacles == (
        ObstacleSpec(
            logical_id=7,
            mode="static",
            geometry=geometry,
            position=(2.0, 1.0, 0.2),
            orientation=IDENTITY_QUATERNION,
        ),
    )
    path = dump_scene_atomic(document, tmp_path / "runtime.yaml")
    assert load_scene(path) == document
    assert "91" not in path.read_text(encoding="utf-8")


def test_from_runtime_rejects_snapshot_without_recoverable_geometry():
    snapshot = ObstacleSnapshot(
        logical_id=1,
        body_id=9,
        mode="static",
        shape="box",
        geometry=None,
        position=(0.0, 0.0, 0.5),
        orientation=IDENTITY_QUATERNION,
    )

    with pytest.raises(ValueError, match=r"obstacles\[0\].*geometry"):
        SceneDocument.from_runtime(
            "df_back",
            TerrainSelection("flat"),
            (snapshot,),
            SensorMounts.default(),
        )


@pytest.mark.parametrize(
    "robot_model",
    ("df_front", "df_mid", "df_back", "active_steering_4wd"),
)
def test_all_registered_robot_models_accept_the_fixed_sensor_semantics(robot_model):
    mapping = sample_scene_mapping()
    mapping["robot"]["model"] = robot_model

    assert scene_document_from_mapping(mapping).robot_model == robot_model


def test_models_are_frozen():
    terrain = sample_scene_document().terrain
    sensors = sample_scene_document().sensors
    scene = sample_scene_document()

    with pytest.raises(FrozenInstanceError):
        terrain.golf_seed = 3
    with pytest.raises(FrozenInstanceError):
        sensors.lidar = LidarConfig.default()
    with pytest.raises(FrozenInstanceError):
        scene.robot_model = "df_mid"


def test_quaternions_are_normalized_at_document_boundaries():
    mapping = sample_scene_mapping()
    mapping["obstacles"][0]["orientation"] = [0, 0, 0, 4]
    mapping["sensors"]["imu"]["orientation"] = [0, 0, 0, 3]

    document = scene_document_from_mapping(mapping)

    assert document.obstacles[0].orientation == IDENTITY_QUATERNION
    assert document.sensors.mounts.imu.orientation == IDENTITY_QUATERNION


def test_arbitrary_quaternion_round_trip_is_object_and_yaml_stable(tmp_path):
    first_document = sample_scene_document()
    quaternion = (1.0, 2.0, 3.0, 4.0)
    obstacle = replace(first_document.obstacles[0], orientation=quaternion)
    imu = first_document.sensors.mounts.imu
    mounts = replace(
        first_document.sensors.mounts,
        imu=MountPose(imu.parent_link, imu.position, quaternion),
    )
    first_document = replace(
        first_document,
        obstacles=(obstacle, first_document.obstacles[1]),
        sensors=replace(first_document.sensors, mounts=mounts),
    )
    first_mapping = document_to_mapping(first_document)
    second_document = scene_document_from_mapping(first_mapping)
    second_mapping = document_to_mapping(second_document)
    first_path = dump_scene_atomic(first_document, tmp_path / "first.yaml")
    second_path = dump_scene_atomic(second_document, tmp_path / "second.yaml")

    assert second_mapping == first_mapping
    assert second_path.read_bytes() == first_path.read_bytes()


@pytest.mark.parametrize(
    ("field_name", "invalid_value", "message"),
    (
        ("parent_link", "", r"sensors\.imu.*parent[_ ]link"),
        ("position", (0.0, 0.0, math.inf), r"sensors\.imu.*position"),
        ("orientation", (0.0, 0.0, 0.0, 0.0), r"sensors\.imu.*quaternion"),
    ),
)
def test_sensor_document_revalidates_forged_mount_pose(
    field_name,
    invalid_value,
    message,
):
    mounts = SensorMounts.default()
    object.__setattr__(mounts.imu, field_name, invalid_value)

    with pytest.raises(ValueError, match=message):
        SensorDocument(mounts, LidarConfig.default())


@pytest.mark.parametrize(
    ("field_name", "invalid_value"),
    (
        ("vertical_lines", 8),
        ("horizontal_samples", 90),
        ("horizontal_fov_deg", 90.0),
        ("vertical_fov_deg", (-10.0, 10.0)),
        ("min_range_m", 0.2),
        ("max_range_m", 20.0),
    ),
)
def test_sensor_document_revalidates_forged_fixed_lidar_field(
    field_name,
    invalid_value,
):
    lidar = LidarConfig.default()
    object.__setattr__(lidar, field_name, invalid_value)

    with pytest.raises(ValueError, match=r"sensors\.lidar.*unsupported"):
        SensorDocument(SensorMounts.default(), lidar)


@pytest.mark.parametrize(
    ("field_name", "invalid_value", "message"),
    (
        ("terrain_model", "dam_slope", r"terrain.*terrain_model"),
        ("slope_deg", 31.0, r"terrain.*slope_deg.*bounds"),
        ("golf_seed", 1 << 31, r"terrain.*golf_seed.*bounds"),
        ("golf_relief", "extreme", r"terrain.*golf_relief"),
    ),
)
def test_scene_document_revalidates_forged_terrain_document(
    field_name,
    invalid_value,
    message,
):
    document = sample_scene_document()
    object.__setattr__(document.terrain, field_name, invalid_value)

    with pytest.raises(ValueError, match=message):
        replace(document, terrain=document.terrain)


@pytest.mark.parametrize(
    ("nested_name", "field_name", "invalid_value", "message"),
    (
        (
            "mount",
            "orientation",
            (0.0, 0.0, 0.0, 0.0),
            r"sensors.*imu.*quaternion",
        ),
        ("lidar", "vertical_lines", 8, r"sensors.*lidar.*unsupported"),
    ),
)
def test_scene_document_revalidates_forged_sensor_document(
    nested_name,
    field_name,
    invalid_value,
    message,
):
    document = sample_scene_document()
    nested = (
        document.sensors.mounts.imu
        if nested_name == "mount"
        else document.sensors.lidar
    )
    object.__setattr__(nested, field_name, invalid_value)

    with pytest.raises(ValueError, match=message):
        replace(document, sensors=document.sensors)


def test_unknown_schema_is_rejected_before_other_sections_are_parsed(tmp_path):
    mapping = sample_scene_mapping()
    mapping["schema_version"] = 2
    mapping["robot"] = {"model": "not-a-model"}

    with pytest.raises(ValueError, match=r"schema_version 2"):
        _load_mapping(tmp_path, mapping)


def test_unknown_schema_is_rejected_before_required_section_checks():
    mapping = {"schema_version": 2, "robot": {"model": "df_back"}}

    with pytest.raises(ValueError, match=r"unsupported schema_version 2"):
        scene_document_from_mapping(mapping)


@pytest.mark.parametrize(
    ("layer", "mutate", "path_match"),
    (
        ("top", lambda raw: raw.update({"unexpected": 1}), r"scene.*unknown.*unexpected"),
        ("robot", lambda raw: raw["robot"].update({"unexpected": 1}), r"robot.*unknown.*unexpected"),
        ("terrain", lambda raw: raw["terrain"].update({"unexpected": 1}), r"terrain.*unknown.*unexpected"),
        ("obstacle", lambda raw: raw["obstacles"][0].update({"unexpected": 1}), r"obstacles\[0\].*unknown.*unexpected"),
        ("geometry", lambda raw: raw["obstacles"][0]["geometry"].update({"unexpected": 1}), r"geometry.*unknown.*unexpected"),
        ("path", lambda raw: raw["obstacles"][0]["path"].update({"unexpected": 1}), r"path.*unknown.*unexpected"),
        ("sensors", lambda raw: raw["sensors"].update({"unexpected": 1}), r"sensors.*unknown.*unexpected"),
        ("mount", lambda raw: raw["sensors"]["imu"].update({"unexpected": 1}), r"sensors\.imu.*unknown.*unexpected"),
        ("lidar", lambda raw: raw["sensors"]["lidar"].update({"unexpected": 1}), r"sensors\.lidar.*unknown.*unexpected"),
    ),
)
def test_every_mapping_layer_rejects_unknown_keys(layer, mutate, path_match):
    del layer
    mapping = sample_scene_mapping()
    mutate(mapping)

    with pytest.raises(ValueError, match=path_match):
        scene_document_from_mapping(mapping)


@pytest.mark.parametrize(
    ("layer", "mutate", "path_match"),
    (
        ("top", lambda raw: raw.pop("robot"), r"scene.*missing.*robot"),
        ("robot", lambda raw: raw["robot"].pop("model"), r"robot.*missing.*model"),
        ("terrain", lambda raw: raw["terrain"].pop("slope_deg"), r"terrain.*missing.*slope_deg"),
        ("obstacle", lambda raw: raw["obstacles"][0].pop("mode"), r"obstacles\[0\].*missing.*mode"),
        ("geometry", lambda raw: raw["obstacles"][0]["geometry"].pop("shape"), r"geometry.*missing.*shape"),
        ("path", lambda raw: raw["obstacles"][0]["path"].pop("speed"), r"path.*missing.*speed"),
        ("sensors", lambda raw: raw["sensors"].pop("imu"), r"sensors.*missing.*imu"),
        ("mount", lambda raw: raw["sensors"]["imu"].pop("parent_link"), r"sensors\.imu.*missing.*parent_link"),
        ("lidar", lambda raw: raw["sensors"]["lidar"].pop("max_range_m"), r"sensors\.lidar.*missing.*max_range_m"),
    ),
)
def test_every_mapping_layer_rejects_missing_keys(layer, mutate, path_match):
    del layer
    mapping = sample_scene_mapping()
    mutate(mapping)

    with pytest.raises(ValueError, match=path_match):
        scene_document_from_mapping(mapping)


@pytest.mark.parametrize(
    ("mutate", "message"),
    (
        (lambda raw: raw["robot"].update(model="tracked_proxy"), "robot_model"),
        (lambda raw: raw["terrain"].update(terrain_model="dam_slope"), "terrain_model"),
        (lambda raw: raw["terrain"].update(golf_relief="extreme"), "golf_relief"),
        (lambda raw: raw["obstacles"][0].update(mode="flying"), "mode"),
        (lambda raw: raw["obstacles"][0]["geometry"].update(shape="mesh"), "shape"),
    ),
)
def test_illegal_enums_are_rejected(mutate, message):
    mapping = sample_scene_mapping()
    mutate(mapping)

    with pytest.raises(ValueError, match=message):
        scene_document_from_mapping(mapping)


@pytest.mark.parametrize(
    ("mutate", "message"),
    (
        (lambda raw: raw.update(schema_version=True), "schema_version"),
        (lambda raw: raw["terrain"].update(slope_deg=True), "slope_deg"),
        (lambda raw: raw["terrain"].update(golf_seed=False), "golf_seed"),
        (lambda raw: raw["obstacles"][0].update(logical_id=True), "logical_id"),
        (lambda raw: raw["obstacles"][0]["geometry"].update(half_extents=[True, 0.2, 0.4]), "half_extents"),
        (lambda raw: raw["obstacles"][0].update(position=[False, 0, 0]), "position"),
        (lambda raw: raw["obstacles"][0].update(orientation=[0, 0, False, 1]), "orientation"),
        (lambda raw: raw["obstacles"][0]["path"].update(speed=True), "speed"),
        (lambda raw: raw["obstacles"][0]["path"].update(progress=False), "progress"),
        (lambda raw: raw["obstacles"][0]["path"].update(direction=True), "direction"),
        (lambda raw: raw["sensors"]["imu"].update(position=[0, True, 0]), "position"),
        (lambda raw: raw["sensors"]["lidar"].update(vertical_lines=True), "vertical_lines"),
    ),
)
def test_bool_numeric_values_are_rejected(mutate, message):
    mapping = sample_scene_mapping()
    mutate(mapping)

    with pytest.raises(ValueError, match=message):
        scene_document_from_mapping(mapping)


@pytest.mark.parametrize(
    ("mutate", "message"),
    (
        (lambda raw: raw["terrain"].update(slope_deg=math.nan), "slope_deg"),
        (lambda raw: raw["obstacles"][0]["geometry"].update(half_extents=[0.2, math.inf, 0.4]), "half_extents"),
        (lambda raw: raw["obstacles"][0].update(position=[0, -math.inf, 0]), "position"),
        (lambda raw: raw["obstacles"][0].update(orientation=[0, math.nan, 0, 1]), "orientation"),
        (lambda raw: raw["obstacles"][0]["path"].update(start_xy=[math.inf, 0]), "start_xy"),
        (lambda raw: raw["obstacles"][0]["path"].update(speed=math.nan), "speed"),
        (lambda raw: raw["obstacles"][0]["path"].update(progress=math.inf), "progress"),
        (lambda raw: raw["sensors"]["imu"].update(position=[0, 0, math.nan]), "position"),
        (lambda raw: raw["sensors"]["lidar"].update(max_range_m=math.inf), "max_range_m"),
    ),
)
def test_non_finite_numeric_values_are_rejected_by_load(tmp_path, mutate, message):
    mapping = sample_scene_mapping()
    mutate(mapping)

    with pytest.raises(ValueError, match=message):
        _load_mapping(tmp_path, mapping)


@pytest.mark.parametrize(
    ("mutate", "message"),
    (
        (lambda raw: raw["obstacles"][0].update(position=[1e9, 0, 0]), "position.*bounds"),
        (lambda raw: raw["obstacles"][0]["geometry"].update(half_extents=[1e9, 1e9, 1e9]), "half_extents.*bounds"),
        (lambda raw: raw["obstacles"][0]["path"].update(start_xy=[1e9, 0]), "start_xy.*bounds"),
        (lambda raw: raw["obstacles"][0]["path"].update(end_xy=[0, -1e9]), "end_xy.*bounds"),
        (lambda raw: raw["obstacles"][0]["path"].update(speed=1e9), "speed.*bounds"),
        (lambda raw: raw["sensors"]["imu"].update(position=[0, 0, 1e9]), "position.*bounds"),
    ),
)
def test_unreasonable_finite_values_are_rejected(mutate, message):
    mapping = sample_scene_mapping()
    mutate(mapping)

    with pytest.raises(ValueError, match=message):
        scene_document_from_mapping(mapping)


def test_duplicate_logical_ids_are_rejected():
    mapping = sample_scene_mapping()
    duplicate = deepcopy(mapping["obstacles"][0])
    mapping["obstacles"].append(duplicate)

    with pytest.raises(ValueError, match="duplicate logical_id 1"):
        scene_document_from_mapping(mapping)


@pytest.mark.parametrize(
    "target",
    ("obstacle", "mount"),
)
def test_zero_quaternions_are_rejected(target):
    mapping = sample_scene_mapping()
    if target == "obstacle":
        mapping["obstacles"][0]["orientation"] = [0, 0, 0, 0]
    else:
        mapping["sensors"]["imu"]["orientation"] = [0, 0, 0, 0]

    with pytest.raises(ValueError, match="quaternion"):
        scene_document_from_mapping(mapping)


@pytest.mark.parametrize(
    ("mode", "path", "message"),
    (
        ("static", "copy", "static.*path"),
        ("moving", None, "moving.*path"),
    ),
)
def test_static_and_moving_path_rules_are_enforced(mode, path, message):
    mapping = sample_scene_mapping()
    obstacle = mapping["obstacles"][0]
    obstacle["mode"] = mode
    if path == "copy":
        assert obstacle["path"] is not None
    else:
        obstacle["path"] = path

    with pytest.raises(ValueError, match=message):
        scene_document_from_mapping(mapping)


@pytest.mark.parametrize(
    ("mount_name", "parent_link"),
    (
        ("lidar_front", "base_link"),
        ("lidar_rear", "lidar_front_mount"),
        ("rtk_primary", "lidar_front_mount"),
        ("rtk_secondary", "missing_link"),
        ("imu", "lidar_rear_mount"),
    ),
)
def test_each_sensor_role_rejects_the_wrong_parent_link(mount_name, parent_link):
    mapping = sample_scene_mapping()
    mapping["sensors"][mount_name]["parent_link"] = parent_link

    with pytest.raises(ValueError, match=rf"sensors\.{mount_name}\.parent_link"):
        scene_document_from_mapping(mapping)


@pytest.mark.parametrize(
    ("field_name", "value"),
    (
        ("vertical_lines", 8),
        ("horizontal_samples", 90),
        ("horizontal_fov_deg", 90.0),
        ("vertical_fov_deg", [-10.0, 10.0]),
        ("min_range_m", 0.2),
        ("max_range_m", 20.0),
    ),
)
def test_scene_rejects_non_fixed_lidar_geometry(field_name, value):
    mapping = sample_scene_mapping()
    mapping["sensors"]["lidar"][field_name] = value

    with pytest.raises(ValueError, match=r"sensors\.lidar.*unsupported"):
        scene_document_from_mapping(mapping)


@pytest.mark.parametrize("content", ("null\n", "- one\n- two\n", "plain scalar\n"))
def test_load_rejects_yaml_that_is_not_a_mapping(tmp_path, content):
    path = tmp_path / "not-mapping.yaml"
    path.write_text(content, encoding="utf-8")

    with pytest.raises(ValueError, match="scene.*mapping"):
        load_scene(path)


def test_load_rejects_malformed_yaml(tmp_path):
    path = tmp_path / "malformed.yaml"
    path.write_text("schema_version: [1\nrobot: {model: df_back}\n", encoding="utf-8")

    with pytest.raises(ValueError, match="scene YAML"):
        load_scene(path)


def test_load_rejects_duplicate_top_level_schema_version(tmp_path):
    path = tmp_path / "duplicate-schema-version.yaml"
    text = yaml.safe_dump(sample_scene_mapping(), sort_keys=False)
    path.write_text(f"schema_version: 2\n{text}", encoding="utf-8")

    with pytest.raises(ValueError, match="duplicate mapping key"):
        load_scene(path)


def test_load_rejects_duplicate_nested_robot_model(tmp_path):
    path = tmp_path / "duplicate-robot-model.yaml"
    text = yaml.safe_dump(sample_scene_mapping(), sort_keys=False)
    text = text.replace(
        "robot:\n  model: df_back\n",
        "robot:\n  model: df_front\n  model: df_back\n",
        1,
    )
    path.write_text(text, encoding="utf-8")

    with pytest.raises(ValueError, match="duplicate mapping key"):
        load_scene(path)


def test_load_checks_actual_bytes_when_file_grows_before_open(tmp_path, monkeypatch):
    path = tmp_path / "growing.yaml"
    path.write_text(
        yaml.safe_dump(sample_scene_mapping(), sort_keys=False),
        encoding="utf-8",
    )
    real_open = type(path).open
    grew = False

    def grow_then_open(self, *args, **kwargs):
        nonlocal grew
        if self == path and not grew:
            grew = True
            # 稀疏扩展可把内存占用限制在读取上限附近。
            with real_open(self, "r+b") as stream:
                stream.truncate(MAX_SCENE_FILE_BYTES + 1)
        return real_open(self, *args, **kwargs)

    monkeypatch.setattr(type(path), "open", grow_then_open)

    with pytest.raises(ValueError, match="file size"):
        load_scene(path)

    assert grew


def test_load_rejects_extremely_deep_valid_yaml_without_recursion_error(tmp_path):
    path = tmp_path / "deep.yaml"
    path.write_text(f"root: {'[' * 800}0{']' * 800}\n", encoding="utf-8")

    with pytest.raises(ValueError, match="nesting depth"):
        load_scene(path)


def test_load_rejects_recursive_yaml_alias(tmp_path):
    path = tmp_path / "recursive.yaml"
    path.write_text(
        """&scene
schema_version: 1
robot: *scene
terrain: {}
obstacles: []
sensors: {}
""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="recursive YAML alias"):
        load_scene(path)


@pytest.mark.parametrize(
    ("field_path", "value", "message"),
    (
        ("robot", [], "robot.*mapping"),
        ("obstacles", {}, "obstacles.*sequence"),
        ("geometry", [], "geometry.*mapping"),
        ("path", [], "path.*mapping"),
        ("sensors", [], "sensors.*mapping"),
        ("mount", [], r"sensors\.imu.*mapping"),
        ("lidar", [], r"sensors\.lidar.*mapping"),
    ),
)
def test_nested_layers_reject_wrong_container_types(field_path, value, message):
    mapping = sample_scene_mapping()
    if field_path == "robot":
        mapping["robot"] = value
    elif field_path == "obstacles":
        mapping["obstacles"] = value
    elif field_path == "geometry":
        mapping["obstacles"][0]["geometry"] = value
    elif field_path == "path":
        mapping["obstacles"][0]["path"] = value
    elif field_path == "sensors":
        mapping["sensors"] = value
    elif field_path == "mount":
        mapping["sensors"]["imu"] = value
    else:
        mapping["sensors"]["lidar"] = value

    with pytest.raises(ValueError, match=message):
        scene_document_from_mapping(mapping)


def test_document_constructor_revalidates_scene_level_obstacle_bounds():
    oversized = ObstacleSpec(
        logical_id=1,
        mode="static",
        geometry=ObstacleGeometry("box", (0.2, 0.2, 0.2)),
        position=(1e9, 0.0, 0.2),
        orientation=IDENTITY_QUATERNION,
    )

    with pytest.raises(ValueError, match="position.*bounds"):
        replace(sample_scene_document(), obstacles=(oversized,))


@pytest.mark.parametrize(
    ("field_name", "replacement", "message"),
    (
        (
            "geometry",
            SimpleNamespace(shape="mesh", half_extents=(0.2, 0.2, 0.4)),
            "geometry.*ObstacleGeometry",
        ),
        (
            "path",
            SimpleNamespace(
                start_xy=(1.0, -0.5),
                end_xy=(3.0, -0.5),
                speed=0.35,
                progress=2.0,
                direction=0,
            ),
            "path.*ObstaclePath",
        ),
    ),
)
def test_document_constructor_rejects_foreign_nested_obstacle_values(
    field_name,
    replacement,
    message,
):
    obstacle = sample_scene_document().obstacles[0]
    object.__setattr__(obstacle, field_name, replacement)

    with pytest.raises(ValueError, match=message):
        replace(sample_scene_document(), obstacles=(obstacle,))


@pytest.mark.parametrize(
    ("nested_name", "field_name", "invalid_value", "message"),
    (
        ("geometry", "shape", "mesh", "geometry.*shape"),
        ("geometry", "half_extents", (0.0, 0.2, 0.4), "geometry.*half_extents"),
        ("geometry", "half_extents", (True, 0.2, 0.4), "geometry.*half_extents"),
        ("geometry", "half_extents", (101.0, 0.2, 0.4), "half_extents.*bounds"),
        ("path", "start_xy", (10_001.0, 0.0), "start_xy.*bounds"),
        ("path", "end_xy", (0.0, -10_001.0), "end_xy.*bounds"),
        ("path", "end_xy", (1.0, -0.5), "path.*endpoints"),
        ("path", "speed", 0.0, "path.*speed"),
        ("path", "speed", True, "path.*speed"),
        ("path", "speed", 101.0, "speed.*bounds"),
        ("path", "progress", 2.0, "path.*progress"),
        ("path", "progress", False, "path.*progress"),
        ("path", "direction", 0, "path.*direction"),
    ),
)
def test_document_constructor_revalidates_forged_nested_obstacle_semantics(
    nested_name,
    field_name,
    invalid_value,
    message,
):
    obstacle = sample_scene_document().obstacles[0]
    nested = getattr(obstacle, nested_name)
    assert nested is not None
    object.__setattr__(nested, field_name, invalid_value)

    with pytest.raises(ValueError, match=message):
        replace(sample_scene_document(), obstacles=(obstacle,))
