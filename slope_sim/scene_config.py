# 版本化场景文件：严格保存车型、地形、逻辑障碍物和企业传感器配置。
from __future__ import annotations

from collections.abc import Hashable, Iterator, Mapping, Sequence
from dataclasses import dataclass
import math
from numbers import Integral, Real
import os
from pathlib import Path
import tempfile

import yaml
from yaml.constructor import ConstructorError
from yaml.nodes import MappingNode, Node

from slope_sim.lidar_pointcloud import LidarConfig
from slope_sim.model_registry import get_robot_model
from slope_sim.obstacles import (
    ObstacleGeometry,
    ObstaclePath,
    ObstacleSnapshot,
    ObstacleSpec,
)
from slope_sim.runtime_actions import TerrainSelection
from slope_sim.scene import terrain_model_names
from slope_sim.truth_sensors import MountPose, SensorMounts


SCENE_SCHEMA_VERSION = 1
MAX_SCENE_FILE_BYTES = 4 * 1024 * 1024
_MAX_YAML_DEPTH = 64
_MAX_YAML_CONTAINERS = 20_000
_MAX_WORLD_COORDINATE_M = 10_000.0
_MAX_OBSTACLE_HALF_EXTENT_M = 100.0
_MAX_OBSTACLE_SPEED_M_S = 100.0
_MAX_SENSOR_OFFSET_M = 100.0
_MAX_OBSTACLES = 100
_INT32_MIN = -(1 << 31)
_INT32_MAX = (1 << 31) - 1
_QUATERNION_NORM_EPSILON = 1e-12
_QUATERNION_UNIT_ROUNDOFF = 4.0 * math.ulp(1.0)
_RELIEF_LEVELS = frozenset({"low", "medium", "high"})
_EXPECTED_SENSOR_PARENTS = {
    "lidar_front": "lidar_front_mount",
    "lidar_rear": "lidar_rear_mount",
    "rtk_primary": "base_link",
    "rtk_secondary": "base_link",
    "imu": "base_link",
}
_INVALID_SEQUENCES = (str, bytes, bytearray, set, frozenset, Mapping, Iterator)


class _YamlNestingDepthError(yaml.YAMLError):
    """表示 YAML 在构造容器前已超过场景深度上限。"""


class _YamlDuplicateMappingKeyError(ConstructorError):
    """表示任意层级 mapping 中出现重复键。"""


class _SceneSafeLoader(yaml.SafeLoader):
    """场景专用安全加载器，在 compose/construct 阶段提前限制输入。"""

    def __init__(self, stream: str) -> None:
        super().__init__(stream)
        self._compose_depth = 0

    def compose_node(self, parent: Node | None, index: object) -> Node:
        """在 PyYAML 递归构造节点前阻止异常深的 YAML。"""
        if self._compose_depth > _MAX_YAML_DEPTH:
            raise _YamlNestingDepthError
        self._compose_depth += 1
        try:
            return super().compose_node(parent, index)
        finally:
            self._compose_depth -= 1

    def construct_mapping(self, node: Node, deep: bool = False) -> dict[object, object]:
        """构造所有 mapping 时拒绝覆盖已有键，包括嵌套 mapping。"""
        if not isinstance(node, MappingNode):
            raise ConstructorError(
                None,
                None,
                f"expected a mapping node, but found {node.id}",
                node.start_mark,
            )
        self.flatten_mapping(node)
        mapping: dict[object, object] = {}
        for key_node, value_node in node.value:
            key = self.construct_object(key_node, deep=deep)
            if not isinstance(key, Hashable):
                raise ConstructorError(
                    "while constructing a mapping",
                    node.start_mark,
                    "found unhashable key",
                    key_node.start_mark,
                )
            if key in mapping:
                raise _YamlDuplicateMappingKeyError(
                    "while constructing a mapping",
                    node.start_mark,
                    f"found duplicate mapping key {key!r}",
                    key_node.start_mark,
                )
            mapping[key] = self.construct_object(value_node, deep=deep)
        return mapping


def _require_mapping(value: object, path: str) -> Mapping[object, object]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{path} must be a mapping")
    return value


def _require_sequence(value: object, path: str) -> tuple[object, ...]:
    if isinstance(value, _INVALID_SEQUENCES) or not isinstance(value, Sequence):
        raise ValueError(f"{path} must be an ordered sequence")
    return tuple(value)


def _require_exact_keys(
    value: Mapping[object, object],
    expected: tuple[str, ...],
    path: str,
) -> None:
    unknown = tuple(key for key in value if key not in expected)
    missing = tuple(key for key in expected if key not in value)
    if unknown:
        labels = ", ".join(str(key) for key in unknown)
        raise ValueError(f"{path} has unknown keys: {labels}")
    if missing:
        raise ValueError(f"{path} is missing keys: {', '.join(missing)}")


def _require_text(value: object, path: str) -> str:
    if type(value) is not str or not value:
        raise ValueError(f"{path} must be a nonempty string")
    return value


def _require_choice(
    value: object,
    path: str,
    choices: Sequence[str],
) -> str:
    normalized = _require_text(value, path).lower()
    if normalized not in choices:
        raise ValueError(f"{path} must be one of: {', '.join(choices)}")
    return normalized


def _require_int(
    value: object,
    path: str,
    *,
    minimum: int | None = None,
    maximum: int | None = None,
) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError(f"{path} must be an integer")
    normalized = int(value)
    if minimum is not None and normalized < minimum:
        raise ValueError(f"{path} is outside supported bounds")
    if maximum is not None and normalized > maximum:
        raise ValueError(f"{path} is outside supported bounds")
    return normalized


def _require_float(
    value: object,
    path: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
    bounds_label: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{path} must be finite")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError(f"{path} must be finite")
    if (
        (minimum is not None and normalized < minimum)
        or (maximum is not None and normalized > maximum)
    ):
        suffix = " exceeds supported bounds" if bounds_label else " is outside its valid range"
        raise ValueError(f"{path}{suffix}")
    return normalized


def _require_vector(
    value: object,
    path: str,
    *,
    length: int,
    absolute_limit: float | None = None,
) -> tuple[float, ...]:
    items = _require_sequence(value, path)
    if len(items) != length:
        raise ValueError(f"{path} must contain {length} finite values")
    normalized = tuple(_require_float(item, f"{path}[{index}]") for index, item in enumerate(items))
    if absolute_limit is not None and any(abs(item) > absolute_limit for item in normalized):
        raise ValueError(f"{path} exceeds supported bounds")
    return normalized


def _require_quaternion(value: object, path: str) -> tuple[float, float, float, float]:
    quaternion = _require_vector(value, path, length=4)
    norm = math.hypot(*quaternion)
    if not math.isfinite(norm) or norm <= _QUATERNION_NORM_EPSILON:
        raise ValueError(f"{path} quaternion norm must be greater than zero")
    # 已归一化值的 hypot 可能偏离 1 数个 ULP；原样保留才能保证往返幂等。
    if abs(norm - 1.0) <= _QUATERNION_UNIT_ROUNDOFF:
        return quaternion  # type: ignore[return-value]
    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]


def _validate_yaml_tree(value: object) -> None:
    """拒绝递归 alias 和异常深的容器图，解析器不展开共享非递归 alias。"""
    seen: set[int] = set()
    active: set[int] = set()

    def visit(item: object, depth: int) -> None:
        if depth > _MAX_YAML_DEPTH:
            raise ValueError("scene YAML exceeds the supported nesting depth")
        if isinstance(item, Mapping):
            children = tuple(item.items())
        elif isinstance(item, (list, tuple)):
            children = tuple(enumerate(item))
        else:
            return

        identity = id(item)
        if identity in active:
            raise ValueError("scene contains a recursive YAML alias")
        if identity in seen:
            return
        seen.add(identity)
        if len(seen) > _MAX_YAML_CONTAINERS:
            raise ValueError("scene YAML contains too many containers")
        active.add(identity)
        try:
            for key, child in children:
                visit(key, depth + 1)
                visit(child, depth + 1)
        finally:
            active.remove(identity)

    visit(value, 0)


@dataclass(frozen=True)
class TerrainDocument:
    """可复现地形选择，不包含 PyBullet 碰撞体或内部网格参数。"""

    terrain_model: str
    slope_deg: float
    golf_seed: int
    golf_relief: str

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "terrain_model",
            _require_choice(self.terrain_model, "terrain_model", terrain_model_names()),
        )
        object.__setattr__(
            self,
            "slope_deg",
            _require_float(
                self.slope_deg,
                "slope_deg",
                minimum=-30.0,
                maximum=30.0,
                bounds_label=True,
            ),
        )
        object.__setattr__(
            self,
            "golf_seed",
            _require_int(
                self.golf_seed,
                "golf_seed",
                minimum=_INT32_MIN,
                maximum=_INT32_MAX,
            ),
        )
        object.__setattr__(
            self,
            "golf_relief",
            _require_choice(
                self.golf_relief,
                "golf_relief",
                tuple(sorted(_RELIEF_LEVELS)),
            ),
        )

    @classmethod
    def from_selection(cls, terrain: TerrainSelection) -> "TerrainDocument":
        if not isinstance(terrain, TerrainSelection):
            raise ValueError("terrain must be a TerrainSelection")
        return cls(
            terrain.terrain_model,
            terrain.slope_deg,
            terrain.golf_seed,
            terrain.golf_relief,
        )


def _revalidate_mount_pose(value: object, path: str) -> MountPose:
    """重构造安装位姿，防止 frozen 实例被绕过后携带非法字段。"""
    if not isinstance(value, MountPose):
        raise ValueError(f"{path} must be a MountPose")
    try:
        return MountPose(value.parent_link, value.position, value.orientation)
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{path}: {exc}") from exc


def _revalidate_sensor_mounts(value: object) -> SensorMounts:
    """逐个重建五个安装位姿，再生成不共享输入子对象的规范副本。"""
    if not isinstance(value, SensorMounts):
        raise ValueError("mounts must be a SensorMounts value")
    validated: list[MountPose] = []
    for name in _EXPECTED_SENSOR_PARENTS:
        try:
            mount = getattr(value, name)
        except (AttributeError, TypeError) as exc:
            raise ValueError(f"sensors.{name}: {exc}") from exc
        validated.append(_revalidate_mount_pose(mount, f"sensors.{name}"))
    return SensorMounts(*validated)


def _revalidate_lidar_config(value: object) -> LidarConfig:
    """重构造固定雷达参数，复用 LidarConfig 的完整领域校验。"""
    if not isinstance(value, LidarConfig):
        raise ValueError("lidar must be a LidarConfig value")
    try:
        return LidarConfig(
            vertical_lines=value.vertical_lines,
            horizontal_samples=value.horizontal_samples,
            horizontal_fov_deg=value.horizontal_fov_deg,
            vertical_fov_deg=value.vertical_fov_deg,
            min_range_m=value.min_range_m,
            max_range_m=value.max_range_m,
        )
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"sensors.lidar: {exc}") from exc


@dataclass(frozen=True)
class SensorDocument:
    """五个语义安装外参与两路雷达共享的固定扫描参数。"""

    mounts: SensorMounts
    lidar: LidarConfig

    def __post_init__(self) -> None:
        mounts = _revalidate_sensor_mounts(self.mounts)
        lidar = _revalidate_lidar_config(self.lidar)
        for name, mount in mounts.named_mounts():
            expected_parent = _EXPECTED_SENSOR_PARENTS[name]
            if mount.parent_link != expected_parent:
                raise ValueError(
                    f"sensors.{name}.parent_link must be {expected_parent!r}"
                )
            _require_vector(
                mount.position,
                f"sensors.{name}.position",
                length=3,
                absolute_limit=_MAX_SENSOR_OFFSET_M,
            )
        object.__setattr__(self, "mounts", mounts)
        object.__setattr__(self, "lidar", lidar)

    @classmethod
    def default(cls) -> "SensorDocument":
        return cls(SensorMounts.default(), LidarConfig.default())


def _validate_obstacle_spec(obstacle: ObstacleSpec, path: str) -> ObstacleSpec:
    """重建领域对象后再施加场景文件自己的坐标、尺寸与速度上界。"""
    if not isinstance(obstacle, ObstacleSpec):
        raise ValueError(f"{path} must be an ObstacleSpec")
    try:
        validated = ObstacleSpec(
            obstacle.logical_id,
            obstacle.mode,
            obstacle.geometry,
            obstacle.position,
            obstacle.orientation,
            obstacle.path,
        )
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{path}: {exc}") from exc

    _require_int(validated.logical_id, f"{path}.logical_id", minimum=1, maximum=_INT32_MAX)
    _require_vector(
        validated.position,
        f"{path}.position",
        length=3,
        absolute_limit=_MAX_WORLD_COORDINATE_M,
    )
    _require_vector(
        validated.geometry.half_extents,
        f"{path}.geometry.half_extents",
        length=3,
        absolute_limit=_MAX_OBSTACLE_HALF_EXTENT_M,
    )
    if validated.path is not None:
        _require_vector(
            validated.path.start_xy,
            f"{path}.path.start_xy",
            length=2,
            absolute_limit=_MAX_WORLD_COORDINATE_M,
        )
        _require_vector(
            validated.path.end_xy,
            f"{path}.path.end_xy",
            length=2,
            absolute_limit=_MAX_WORLD_COORDINATE_M,
        )
        _require_float(
            validated.path.speed,
            f"{path}.path.speed",
            minimum=0.0,
            maximum=_MAX_OBSTACLE_SPEED_M_S,
            bounds_label=True,
        )
    return validated


@dataclass(frozen=True)
class SceneDocument:
    """schema v1 完整逻辑场景；构造完成即已通过全量领域校验。"""

    schema_version: int
    robot_model: str
    terrain: TerrainDocument
    obstacles: tuple[ObstacleSpec, ...]
    sensors: SensorDocument

    def __post_init__(self) -> None:
        version = _require_int(self.schema_version, "schema_version")
        if version != SCENE_SCHEMA_VERSION:
            raise ValueError(f"unsupported schema_version {version}")
        try:
            model = get_robot_model(_require_text(self.robot_model, "robot_model"))
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError(f"robot_model: {exc}") from exc
        if not isinstance(self.terrain, TerrainDocument):
            raise ValueError("terrain must be a TerrainDocument")
        if not isinstance(self.sensors, SensorDocument):
            raise ValueError("sensors must be a SensorDocument")
        # 外层文档重建嵌套值，确保任何绕过 frozen 的输入都重新经过领域构造器。
        try:
            terrain = TerrainDocument(
                self.terrain.terrain_model,
                self.terrain.slope_deg,
                self.terrain.golf_seed,
                self.terrain.golf_relief,
            )
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"terrain: {exc}") from exc
        try:
            sensors = SensorDocument(self.sensors.mounts, self.sensors.lidar)
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"sensors: {exc}") from exc
        obstacle_items = _require_sequence(self.obstacles, "obstacles")
        if len(obstacle_items) > _MAX_OBSTACLES:
            raise ValueError(f"obstacles exceeds supported bounds of {_MAX_OBSTACLES}")
        logical_ids: set[int] = set()
        normalized_obstacles: list[ObstacleSpec] = []
        for index, obstacle in enumerate(obstacle_items):
            validated = _validate_obstacle_spec(obstacle, f"obstacles[{index}]")
            if validated.logical_id in logical_ids:
                raise ValueError(f"duplicate logical_id {validated.logical_id}")
            logical_ids.add(validated.logical_id)
            normalized_obstacles.append(validated)

        object.__setattr__(self, "schema_version", version)
        object.__setattr__(self, "robot_model", model.name)
        object.__setattr__(self, "terrain", terrain)
        object.__setattr__(self, "sensors", sensors)
        object.__setattr__(
            self,
            "obstacles",
            tuple(sorted(normalized_obstacles, key=lambda item: item.logical_id)),
        )

    def _replace_validated_runtime_obstacles(
        self,
        obstacles: tuple[ObstacleSpec, ...],
    ) -> "SceneDocument":
        """只替换管理器已验证的冻结规格，避免移动帧重复深度校验静态场景。"""
        if type(obstacles) is not tuple or len(obstacles) > _MAX_OBSTACLES:
            raise ValueError("runtime obstacles must be a bounded tuple")
        if any(type(obstacle) is not ObstacleSpec for obstacle in obstacles):
            raise ValueError("runtime obstacles must contain exact ObstacleSpec values")
        ordered = tuple(sorted(obstacles, key=lambda item: item.logical_id))
        logical_ids = tuple(obstacle.logical_id for obstacle in ordered)
        if len(set(logical_ids)) != len(logical_ids):
            raise ValueError("runtime obstacles must have unique logical ids")

        # self 和规格均来自已提交领域对象；公开构造/导入入口仍执行完整重校验。
        document = object.__new__(type(self))
        object.__setattr__(document, "schema_version", self.schema_version)
        object.__setattr__(document, "robot_model", self.robot_model)
        object.__setattr__(document, "terrain", self.terrain)
        object.__setattr__(document, "obstacles", ordered)
        object.__setattr__(document, "sensors", self.sensors)
        return document

    @classmethod
    def from_runtime(
        cls,
        robot_model: str,
        terrain: TerrainSelection | TerrainDocument,
        obstacles: Sequence[ObstacleSnapshot | ObstacleSpec],
        mounts: SensorMounts,
        *,
        lidar_config: LidarConfig | None = None,
    ) -> "SceneDocument":
        """复制运行时逻辑状态，显式丢弃每个障碍物的临时 body id。"""
        terrain_document = (
            terrain
            if isinstance(terrain, TerrainDocument)
            else TerrainDocument.from_selection(terrain)
        )
        obstacle_items = _require_sequence(obstacles, "obstacles")
        logical_obstacles: list[ObstacleSpec] = []
        for index, obstacle in enumerate(obstacle_items):
            if isinstance(obstacle, ObstacleSpec):
                logical_obstacles.append(obstacle)
                continue
            if not isinstance(obstacle, ObstacleSnapshot):
                raise ValueError(
                    f"obstacles[{index}] must be an ObstacleSnapshot or ObstacleSpec"
                )
            if obstacle.geometry is None:
                raise ValueError(
                    f"obstacles[{index}].geometry is required for scene recovery"
                )
            logical_obstacles.append(
                ObstacleSpec(
                    logical_id=obstacle.logical_id,
                    mode=obstacle.mode,
                    geometry=obstacle.geometry,
                    position=obstacle.position,
                    orientation=obstacle.orientation,
                    path=obstacle.path,
                )
            )
        return cls(
            SCENE_SCHEMA_VERSION,
            robot_model,
            terrain_document,
            tuple(logical_obstacles),
            SensorDocument(
                mounts,
                LidarConfig.default() if lidar_config is None else lidar_config,
            ),
        )


def _parse_mount(value: object, path: str) -> MountPose:
    mapping = _require_mapping(value, path)
    _require_exact_keys(mapping, ("parent_link", "position", "orientation"), path)
    parent_link = _require_text(mapping["parent_link"], f"{path}.parent_link")
    position = _require_vector(
        mapping["position"],
        f"{path}.position",
        length=3,
        absolute_limit=_MAX_SENSOR_OFFSET_M,
    )
    orientation = _require_quaternion(mapping["orientation"], f"{path}.orientation")
    try:
        return MountPose(parent_link, position, orientation)
    except ValueError as exc:
        raise ValueError(f"{path}: {exc}") from exc


def _parse_lidar(value: object) -> LidarConfig:
    path = "sensors.lidar"
    mapping = _require_mapping(value, path)
    expected = (
        "vertical_lines",
        "horizontal_samples",
        "horizontal_fov_deg",
        "vertical_fov_deg",
        "min_range_m",
        "max_range_m",
    )
    _require_exact_keys(mapping, expected, path)
    vertical_fov = _require_vector(
        mapping["vertical_fov_deg"],
        f"{path}.vertical_fov_deg",
        length=2,
    )
    try:
        return LidarConfig(
            vertical_lines=_require_int(
                mapping["vertical_lines"], f"{path}.vertical_lines"
            ),
            horizontal_samples=_require_int(
                mapping["horizontal_samples"], f"{path}.horizontal_samples"
            ),
            horizontal_fov_deg=_require_float(
                mapping["horizontal_fov_deg"], f"{path}.horizontal_fov_deg"
            ),
            vertical_fov_deg=(vertical_fov[0], vertical_fov[1]),
            min_range_m=_require_float(
                mapping["min_range_m"], f"{path}.min_range_m"
            ),
            max_range_m=_require_float(
                mapping["max_range_m"], f"{path}.max_range_m"
            ),
        )
    except ValueError as exc:
        raise ValueError(f"{path}: {exc}") from exc


def _parse_sensors(value: object) -> SensorDocument:
    path = "sensors"
    mapping = _require_mapping(value, path)
    expected = (*_EXPECTED_SENSOR_PARENTS, "lidar")
    _require_exact_keys(mapping, expected, path)
    mounts = SensorMounts(
        lidar_front=_parse_mount(mapping["lidar_front"], "sensors.lidar_front"),
        lidar_rear=_parse_mount(mapping["lidar_rear"], "sensors.lidar_rear"),
        rtk_primary=_parse_mount(mapping["rtk_primary"], "sensors.rtk_primary"),
        rtk_secondary=_parse_mount(mapping["rtk_secondary"], "sensors.rtk_secondary"),
        imu=_parse_mount(mapping["imu"], "sensors.imu"),
    )
    return SensorDocument(mounts, _parse_lidar(mapping["lidar"]))


def _parse_geometry(value: object, path: str) -> ObstacleGeometry:
    mapping = _require_mapping(value, path)
    _require_exact_keys(mapping, ("shape", "half_extents"), path)
    shape = _require_choice(
        mapping["shape"],
        f"{path}.shape",
        ("box", "cylinder", "sphere"),
    )
    half_extents = _require_vector(
        mapping["half_extents"],
        f"{path}.half_extents",
        length=3,
        absolute_limit=_MAX_OBSTACLE_HALF_EXTENT_M,
    )
    try:
        return ObstacleGeometry(shape, half_extents)
    except ValueError as exc:
        raise ValueError(f"{path}: {exc}") from exc


def _parse_path(value: object, path: str) -> ObstaclePath:
    mapping = _require_mapping(value, path)
    expected = ("start_xy", "end_xy", "speed", "progress", "direction")
    _require_exact_keys(mapping, expected, path)
    start_xy = _require_vector(
        mapping["start_xy"],
        f"{path}.start_xy",
        length=2,
        absolute_limit=_MAX_WORLD_COORDINATE_M,
    )
    end_xy = _require_vector(
        mapping["end_xy"],
        f"{path}.end_xy",
        length=2,
        absolute_limit=_MAX_WORLD_COORDINATE_M,
    )
    speed = _require_float(
        mapping["speed"],
        f"{path}.speed",
        minimum=0.0,
        maximum=_MAX_OBSTACLE_SPEED_M_S,
        bounds_label=True,
    )
    progress = _require_float(
        mapping["progress"],
        f"{path}.progress",
        minimum=0.0,
        maximum=1.0,
    )
    direction = _require_int(
        mapping["direction"],
        f"{path}.direction",
        minimum=-1,
        maximum=1,
    )
    try:
        return ObstaclePath(start_xy, end_xy, speed, progress, direction)
    except ValueError as exc:
        raise ValueError(f"{path}: {exc}") from exc


def _parse_obstacle(value: object, index: int) -> ObstacleSpec:
    path = f"obstacles[{index}]"
    mapping = _require_mapping(value, path)
    expected = (
        "logical_id",
        "mode",
        "geometry",
        "position",
        "orientation",
        "path",
    )
    _require_exact_keys(mapping, expected, path)
    logical_id = _require_int(
        mapping["logical_id"],
        f"{path}.logical_id",
        minimum=1,
        maximum=_INT32_MAX,
    )
    mode = _require_choice(mapping["mode"], f"{path}.mode", ("static", "moving"))
    geometry = _parse_geometry(mapping["geometry"], f"{path}.geometry")
    position = _require_vector(
        mapping["position"],
        f"{path}.position",
        length=3,
        absolute_limit=_MAX_WORLD_COORDINATE_M,
    )
    orientation = _require_quaternion(mapping["orientation"], f"{path}.orientation")
    raw_path = mapping["path"]
    obstacle_path = None if raw_path is None else _parse_path(raw_path, f"{path}.path")
    try:
        return ObstacleSpec(
            logical_id,
            mode,
            geometry,
            position,
            orientation,
            obstacle_path,
        )
    except ValueError as exc:
        raise ValueError(f"{path}: {exc}") from exc


def scene_document_from_mapping(value: object) -> SceneDocument:
    """从纯 YAML mapping 构造全量校验后的 schema v1 场景。"""
    _validate_yaml_tree(value)
    mapping = _require_mapping(value, "scene")
    expected = ("schema_version", "robot", "terrain", "obstacles", "sensors")
    if "schema_version" not in mapping:
        _require_exact_keys(mapping, expected, "scene")
    version = _require_int(mapping["schema_version"], "schema_version")
    if version != SCENE_SCHEMA_VERSION:
        raise ValueError(f"unsupported schema_version {version}")
    _require_exact_keys(mapping, expected, "scene")

    robot_mapping = _require_mapping(mapping["robot"], "robot")
    _require_exact_keys(robot_mapping, ("model",), "robot")
    robot_model = _require_text(robot_mapping["model"], "robot_model")

    terrain_mapping = _require_mapping(mapping["terrain"], "terrain")
    terrain_keys = ("terrain_model", "slope_deg", "golf_seed", "golf_relief")
    _require_exact_keys(terrain_mapping, terrain_keys, "terrain")
    terrain = TerrainDocument(
        _require_choice(
            terrain_mapping["terrain_model"],
            "terrain_model",
            terrain_model_names(),
        ),
        _require_float(terrain_mapping["slope_deg"], "slope_deg"),
        _require_int(terrain_mapping["golf_seed"], "golf_seed"),
        _require_choice(
            terrain_mapping["golf_relief"],
            "golf_relief",
            tuple(sorted(_RELIEF_LEVELS)),
        ),
    )

    obstacle_values = _require_sequence(mapping["obstacles"], "obstacles")
    obstacles = tuple(
        _parse_obstacle(obstacle, index)
        for index, obstacle in enumerate(obstacle_values)
    )
    return SceneDocument(
        version,
        robot_model,
        terrain,
        obstacles,
        _parse_sensors(mapping["sensors"]),
    )


def _mount_to_mapping(mount: MountPose) -> dict[str, object]:
    return {
        "parent_link": mount.parent_link,
        "position": list(mount.position),
        "orientation": list(mount.orientation),
    }


def _obstacle_to_mapping(obstacle: ObstacleSpec) -> dict[str, object]:
    path = obstacle.path
    path_mapping = None
    if path is not None:
        path_mapping = {
            "start_xy": list(path.start_xy),
            "end_xy": list(path.end_xy),
            "speed": path.speed,
            "progress": path.progress,
            "direction": path.direction,
        }
    return {
        "logical_id": obstacle.logical_id,
        "mode": obstacle.mode,
        "geometry": {
            "shape": obstacle.geometry.shape,
            "half_extents": list(obstacle.geometry.half_extents),
        },
        "position": list(obstacle.position),
        "orientation": list(obstacle.orientation),
        "path": path_mapping,
    }


def document_to_mapping(document: SceneDocument) -> dict[str, object]:
    """按固定字段顺序生成不含任何运行时句柄的纯 Python mapping。"""
    if not isinstance(document, SceneDocument):
        raise ValueError("document must be a SceneDocument")
    mounts = document.sensors.mounts
    lidar = document.sensors.lidar
    return {
        "schema_version": document.schema_version,
        "robot": {"model": document.robot_model},
        "terrain": {
            "terrain_model": document.terrain.terrain_model,
            "slope_deg": document.terrain.slope_deg,
            "golf_seed": document.terrain.golf_seed,
            "golf_relief": document.terrain.golf_relief,
        },
        "obstacles": [
            _obstacle_to_mapping(obstacle) for obstacle in document.obstacles
        ],
        "sensors": {
            "lidar_front": _mount_to_mapping(mounts.lidar_front),
            "lidar_rear": _mount_to_mapping(mounts.lidar_rear),
            "rtk_primary": _mount_to_mapping(mounts.rtk_primary),
            "rtk_secondary": _mount_to_mapping(mounts.rtk_secondary),
            "imu": _mount_to_mapping(mounts.imu),
            "lidar": {
                "vertical_lines": lidar.vertical_lines,
                "horizontal_samples": lidar.horizontal_samples,
                "horizontal_fov_deg": lidar.horizontal_fov_deg,
                "vertical_fov_deg": list(lidar.vertical_fov_deg),
                "min_range_m": lidar.min_range_m,
                "max_range_m": lidar.max_range_m,
            },
        },
    }


def _require_scene_path(value: object) -> Path:
    if isinstance(value, str):
        if not value:
            raise ValueError("scene path must be nonempty")
        return Path(value)
    if isinstance(value, Path):
        return value
    raise ValueError("scene path must be a string or Path")


def dump_scene_atomic(document: SceneDocument, path: str | Path) -> Path:
    """在目标同目录写完并 fsync 临时文件，再以 os.replace 原子发布。"""
    mapping = document_to_mapping(document)
    target = _require_scene_path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    raw_fd: int | None = None
    temporary: Path | None = None
    try:
        raw_fd, temp_name = tempfile.mkstemp(
            prefix=f".{target.name}.",
            suffix=".tmp",
            dir=target.parent,
        )
        temporary = Path(temp_name)
        try:
            stream = os.fdopen(raw_fd, "w", encoding="utf-8", newline="\n")
        except BaseException:
            os.close(raw_fd)
            raw_fd = None
            raise
        raw_fd = None
        with stream:
            yaml.safe_dump(
                mapping,
                stream,
                sort_keys=False,
                allow_unicode=True,
                default_flow_style=False,
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
        temporary = None
        return target
    finally:
        if raw_fd is not None:
            try:
                os.close(raw_fd)
            except OSError:
                pass
        if temporary is not None:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass


def load_scene(path: str | Path) -> SceneDocument:
    """安全读取有限大小 YAML，并在返回前完成递归和领域全量校验。"""
    target = _require_scene_path(path)
    # 按实际读取字节限流，避免 stat 与 read 之间的文件增长竞态。
    with target.open("rb") as stream:
        payload = stream.read(MAX_SCENE_FILE_BYTES + 1)
    if len(payload) > MAX_SCENE_FILE_BYTES:
        raise ValueError("scene YAML exceeds the supported file size")
    try:
        text = payload.decode("utf-8", errors="strict")
    except UnicodeDecodeError as exc:
        raise ValueError("scene YAML must be valid UTF-8") from exc
    try:
        raw = yaml.load(text, Loader=_SceneSafeLoader)
    except (_YamlNestingDepthError, RecursionError) as exc:
        raise ValueError("scene YAML exceeds the supported nesting depth") from exc
    except _YamlDuplicateMappingKeyError as exc:
        raise ValueError("scene YAML contains a duplicate mapping key") from exc
    except yaml.YAMLError as exc:
        raise ValueError("scene YAML is malformed") from exc
    return scene_document_from_mapping(raw)
