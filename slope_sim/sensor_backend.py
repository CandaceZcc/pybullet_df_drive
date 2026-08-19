# 传感器后端：集中封装 PyBullet 位姿、坐标变换与批量射线读取边界。
from __future__ import annotations

from collections.abc import Collection, Iterator, Mapping, Sequence
from dataclasses import dataclass
import math
from numbers import Integral, Real
from typing import Literal, Protocol, TypeAlias

import pybullet as p

from slope_sim.obstacles import ObstacleSnapshot


Vec3: TypeAlias = tuple[float, float, float]
Quaternion: TypeAlias = tuple[float, float, float, float]
HitCategory: TypeAlias = Literal[
    "terrain",
    "static_obstacle",
    "moving_obstacle",
    "unknown",
]

_QUATERNION_NORM_EPSILON = 1e-12
_QUATERNION_UNIT_ROUNDOFF = 4.0 * math.ulp(1.0)
_PYBULLET_SIGNED_C_INT_MAX = 0x7FFFFFFF
PYBULLET_MAX_RAY_BATCH_SIZE = 16383
_KNOWN_HIT_CATEGORIES = frozenset(
    {"terrain", "static_obstacle", "moving_obstacle", "unknown"}
)


def _require_finite_number(
    name: str,
    value: object,
    *,
    error_type: type[Exception] = ValueError,
) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise error_type(f"{name} must be finite")
    normalized = float(value)
    if not math.isfinite(normalized):
        raise error_type(f"{name} must be finite")
    return normalized


def _require_finite_vector(
    name: str,
    values: object,
    *,
    length: int,
    error_type: type[Exception] = ValueError,
) -> tuple[float, ...]:
    invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
    if isinstance(values, invalid_types) or not isinstance(values, Sequence):
        raise error_type(f"{name} must contain {length} finite values")
    items = tuple(values)
    if len(items) != length:
        raise error_type(f"{name} must contain {length} finite values")
    return tuple(
        _require_finite_number(
            f"{name}[{index}]",
            value,
            error_type=error_type,
        )
        for index, value in enumerate(items)
    )


def _normalize_quaternion(
    orientation: object,
    *,
    error_type: type[Exception] = ValueError,
) -> Quaternion:
    quaternion = _require_finite_vector(
        "quaternion",
        orientation,
        length=4,
        error_type=error_type,
    )
    norm = math.hypot(*quaternion)
    if not math.isfinite(norm) or norm <= _QUATERNION_NORM_EPSILON:
        raise error_type("quaternion norm must be finite and greater than zero")
    # 已归一化值的 hypot 可能偏离 1 数个 ULP；保持 Pose 重建严格幂等。
    if abs(norm - 1.0) <= _QUATERNION_UNIT_ROUNDOFF:
        return quaternion  # type: ignore[return-value]
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def _require_integral(
    name: str,
    value: object,
    *,
    minimum: int,
    error_type: type[Exception] = ValueError,
) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise error_type(f"{name} must be an integer greater than or equal to {minimum}")
    normalized = int(value)
    if normalized < minimum:
        raise error_type(f"{name} must be an integer greater than or equal to {minimum}")
    return normalized


def _require_collision_mask(value: object) -> int:
    """把射线 mask 限定到 PyBullet 可接受的非负有符号 C int。"""
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ValueError("collision_mask must be an integer in range 0..0x7fffffff")
    normalized = int(value)
    if not 0 <= normalized <= _PYBULLET_SIGNED_C_INT_MAX:
        raise ValueError("collision_mask must be an integer in range 0..0x7fffffff")
    return normalized


@dataclass(frozen=True)
class Pose:
    """不可变三维位姿；四元数在构造时统一归一化。"""

    position: Vec3
    orientation: Quaternion

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "position",
            _require_finite_vector("position", self.position, length=3),
        )
        object.__setattr__(self, "orientation", _normalize_quaternion(self.orientation))


@dataclass(frozen=True, slots=True)
class RayHit:
    """单条射线的世界命中位置、内部物理索引和稳定逻辑类别。"""

    position: Vec3
    body_id: int
    link_index: int
    category: HitCategory

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "position",
            _require_finite_vector("position", self.position, length=3),
        )
        body_id = _require_integral("body_id", self.body_id, minimum=-1)
        link_index = _require_integral("link_index", self.link_index, minimum=-1)
        if self.category not in _KNOWN_HIT_CATEGORIES:
            raise ValueError("category must be a known ray hit category")
        if body_id < 0 and (link_index != -1 or self.category != "unknown"):
            raise ValueError("a missed ray must use link_index -1 and category unknown")
        object.__setattr__(self, "body_id", body_id)
        object.__setattr__(self, "link_index", link_index)

    @property
    def hit(self) -> bool:
        """body id 非负时表示射线命中。"""
        return self.body_id >= 0

    @property
    def hit_position(self) -> Vec3:
        """提供显式命中位置名称，便于后续点云代码阅读。"""
        return self.position

    @classmethod
    def _from_trusted_pybullet(
        cls,
        position: Vec3,
        body_id: int,
        link_index: int,
        category: HitCategory,
    ) -> "RayHit":
        """仅供已完成整组校验的后端使用，避免重复执行公开构造校验。"""
        instance = object.__new__(cls)
        object.__setattr__(instance, "position", position)
        object.__setattr__(instance, "body_id", body_id)
        object.__setattr__(instance, "link_index", link_index)
        object.__setattr__(instance, "category", category)
        return instance


class SensorBackend(Protocol):
    """传感器算法可用的窄后端协议，不暴露 PyBullet 对象。"""

    def link_names(self) -> tuple[str, ...]: ...

    def world_pose(self, parent_link: str) -> Pose: ...

    def transform_pose(self, parent: Pose, local: Pose) -> Pose: ...

    def inverse_transform_point(self, pose: Pose, point: Vec3) -> Vec3: ...

    def inverse_transform_points(
        self,
        pose: Pose,
        points: Sequence[Vec3],
    ) -> tuple[Vec3, ...]: ...

    def euler_from_quaternion(
        self,
        orientation: Quaternion,
    ) -> tuple[float, float, float]: ...

    def ray_test_batch(
        self,
        starts: Sequence[Vec3],
        ends: Sequence[Vec3],
        *,
        collision_mask: int,
    ) -> tuple[RayHit, ...]: ...


class PyBulletSensorBackend:
    """只读 PyBullet 适配器；所有传感器物理查询集中在当前物理线程调用。"""

    def __init__(self, client_id: int, robot_id: int) -> None:
        self.client_id = _require_integral("client_id", client_id, minimum=0)
        self.robot_id = _require_integral("robot_id", robot_id, minimum=0)
        self._link_name_to_index = self._collect_link_indices()
        self._link_names = tuple(self._link_name_to_index)
        self._hit_categories: dict[int, HitCategory] = {}

    def _collect_link_indices(self) -> dict[str, int]:
        """把 URDF child link 名转换为稳定语义名称，索引只留在后端内部。"""
        links = {"base_link": -1}
        joint_count = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        for link_index in range(joint_count):
            info = p.getJointInfo(
                self.robot_id,
                link_index,
                physicsClientId=self.client_id,
            )
            raw_name = info[12]
            link_name = raw_name.decode("utf-8") if isinstance(raw_name, bytes) else str(raw_name)
            if not link_name or link_name in links:
                raise RuntimeError(f"robot contains an invalid or duplicate link name: {link_name!r}")
            links[link_name] = link_index
        return links

    def link_names(self) -> tuple[str, ...]:
        """返回 base 和全部 child link 的语义名称。"""
        return self._link_names

    def world_pose(self, parent_link: str) -> Pose:
        """读取 base 或 child link 的世界 link frame 位姿。"""
        if not isinstance(parent_link, str) or parent_link not in self._link_name_to_index:
            raise ValueError(f"unknown parent link: {parent_link}")
        if parent_link == "base_link":
            return self._base_link_world_pose()

        state = p.getLinkState(
            self.robot_id,
            self._link_name_to_index[parent_link],
            computeForwardKinematics=True,
            physicsClientId=self.client_id,
        )
        if len(state) < 6:
            raise RuntimeError("PyBullet getLinkState returned fewer than 6 fields")
        # state[4]/[5] 是 URDF world link frame；state[0]/[1] 是惯性 COM frame。
        position, orientation = state[4], state[5]
        return self._pose_from_pybullet(position, orientation, operation="world pose")

    def _base_link_world_pose(self) -> Pose:
        """从根刚体世界惯性 frame 还原 URDF base_link 世界 frame。"""
        world_position, world_orientation = p.getBasePositionAndOrientation(
            self.robot_id,
            physicsClientId=self.client_id,
        )
        world_inertial = self._pose_from_pybullet(
            world_position,
            world_orientation,
            operation="world inertial pose",
        )
        dynamics = p.getDynamicsInfo(
            self.robot_id,
            -1,
            physicsClientId=self.client_id,
        )
        if len(dynamics) < 5:
            raise RuntimeError("PyBullet getDynamicsInfo returned fewer than 5 fields")
        local_inertial = self._pose_from_pybullet(
            dynamics[3],
            dynamics[4],
            operation="local inertial pose",
        )

        # world_inertial = world_base_link * local_inertial。
        inverse_position, inverse_orientation = p.invertTransform(
            local_inertial.position,
            local_inertial.orientation,
        )
        inverse_local_inertial = self._pose_from_pybullet(
            inverse_position,
            inverse_orientation,
            operation="inverse local inertial pose",
        )
        return self.transform_pose(world_inertial, inverse_local_inertial)

    def transform_pose(self, parent: Pose, local: Pose) -> Pose:
        """把局部安装位姿变换到父 frame 所在坐标系。"""
        if not isinstance(parent, Pose) or not isinstance(local, Pose):
            raise ValueError("parent and local must be Pose values")
        position, orientation = p.multiplyTransforms(
            parent.position,
            parent.orientation,
            local.position,
            local.orientation,
        )
        return self._pose_from_pybullet(position, orientation, operation="pose transform")

    def inverse_transform_point(self, pose: Pose, point: Vec3) -> Vec3:
        """把世界点转换到给定位姿的局部坐标系。"""
        if not isinstance(pose, Pose):
            raise ValueError("pose must be a Pose value")
        world_point = _require_finite_vector("point", point, length=3)
        return self._inverse_transform_points_trusted(
            pose,
            (world_point,),
        )[0]

    def inverse_transform_points(
        self,
        pose: Pose,
        points: Sequence[Vec3],
    ) -> tuple[Vec3, ...]:
        """用同一逆旋转批量转换世界点，避免逐点跨入 PyBullet C API。"""
        if not isinstance(pose, Pose):
            raise ValueError("pose must be a Pose value")
        self._require_ray_sequence("points", points)
        normalized_points = self._normalize_ray_points("points", points)
        return self._inverse_transform_points_trusted(pose, normalized_points)

    def inverse_transform_points_prevalidated(
        self,
        pose: Pose,
        points: tuple[Vec3, ...],
    ) -> tuple[Vec3, ...]:
        """转换同一后端刚验证的命中点，避免 LiDAR 热路径再次逐点检查。"""
        if type(pose) is not Pose or type(points) is not tuple:
            raise ValueError("prevalidated inverse transform requires Pose and tuple inputs")
        return self._inverse_transform_points_trusted(pose, points)

    @staticmethod
    def _inverse_transform_points_trusted(
        pose: Pose,
        points: tuple[Vec3, ...],
    ) -> tuple[Vec3, ...]:
        """对已校验点执行 `R^T * (world - origin)`，矩阵每批只计算一次。"""
        x, y, z, w = pose.orientation
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        r00 = 1.0 - 2.0 * (yy + zz)
        r01 = 2.0 * (xy - wz)
        r02 = 2.0 * (xz + wy)
        r10 = 2.0 * (xy + wz)
        r11 = 1.0 - 2.0 * (xx + zz)
        r12 = 2.0 * (yz - wx)
        r20 = 2.0 * (xz - wy)
        r21 = 2.0 * (yz + wx)
        r22 = 1.0 - 2.0 * (xx + yy)
        px, py, pz = pose.position

        return tuple(
            (
                r00 * (point[0] - px)
                + r10 * (point[1] - py)
                + r20 * (point[2] - pz),
                r01 * (point[0] - px)
                + r11 * (point[1] - py)
                + r21 * (point[2] - pz),
                r02 * (point[0] - px)
                + r12 * (point[1] - py)
                + r22 * (point[2] - pz),
            )
            for point in points
        )

    def euler_from_quaternion(
        self,
        orientation: Quaternion,
    ) -> tuple[float, float, float]:
        """把有限非零四元数转换为 PyBullet 约定的 roll/pitch/yaw。"""
        quaternion = _normalize_quaternion(orientation)
        euler = p.getEulerFromQuaternion(quaternion)
        return _require_finite_vector(
            "PyBullet euler",
            euler,
            length=3,
            error_type=RuntimeError,
        )  # type: ignore[return-value]

    def bind_scene(
        self,
        terrain_body_ids: Collection[int],
        obstacles: Sequence[ObstacleSnapshot],
    ) -> None:
        """原子重建临时 body id 到稳定命中类别的内部映射。"""
        invalid_collections = (str, bytes, Mapping, Iterator)
        if (
            isinstance(terrain_body_ids, invalid_collections)
            or not isinstance(terrain_body_ids, Collection)
        ):
            raise ValueError("terrain_body_ids must be a finite Collection, not a Mapping")
        if (
            isinstance(obstacles, invalid_collections)
            or not isinstance(obstacles, Sequence)
        ):
            raise ValueError("obstacles must be a finite Sequence, not a Mapping")
        terrain_items = tuple(terrain_body_ids)
        obstacle_items = tuple(obstacles)

        categories: dict[int, HitCategory] = {}
        for raw_body_id in terrain_items:
            body_id = _require_integral("terrain body id", raw_body_id, minimum=0)
            categories[body_id] = "terrain"

        for index, obstacle in enumerate(obstacle_items):
            try:
                raw_body_id = obstacle.body_id
                mode = obstacle.mode
            except AttributeError as exc:
                raise ValueError(f"obstacles[{index}] must provide body_id and mode") from exc
            if not isinstance(mode, str) or mode not in {"static", "moving"}:
                raise ValueError(
                    f"obstacles[{index}].mode must be a string: static or moving"
                )
            if raw_body_id is None:
                continue
            body_id = _require_integral(
                f"obstacles[{index}].body_id",
                raw_body_id,
                minimum=0,
            )
            category: HitCategory = (
                "moving_obstacle" if mode == "moving" else "static_obstacle"
            )
            existing = categories.get(body_id)
            if existing is not None and existing != category:
                raise ValueError(f"body id {body_id} has conflicting scene categories")
            categories[body_id] = category

        self._hit_categories = categories

    def ray_test_batch(
        self,
        starts: Sequence[Vec3],
        ends: Sequence[Vec3],
        *,
        collision_mask: int,
    ) -> tuple[RayHit, ...]:
        """执行一次严格定长批量射线，并把临时 ID 转换为稳定类别。"""
        mask = _require_collision_mask(collision_mask)
        start_count = self._require_ray_sequence("starts", starts)
        end_count = self._require_ray_sequence("ends", ends)
        if start_count != end_count:
            raise ValueError("starts and ends must have the same length")
        if start_count > PYBULLET_MAX_RAY_BATCH_SIZE:
            raise ValueError(
                f"ray batch must contain at most {PYBULLET_MAX_RAY_BATCH_SIZE} rays"
            )
        if start_count == 0:
            return ()
        normalized_starts = self._normalize_ray_points("starts", starts)
        normalized_ends = self._normalize_ray_points("ends", ends)

        raw_results = p.rayTestBatch(
            normalized_starts,
            normalized_ends,
            collisionFilterMask=mask,
            physicsClientId=self.client_id,
        )
        try:
            results = tuple(raw_results)
        except TypeError as exc:
            raise RuntimeError("PyBullet rayTestBatch returned a non-sequence") from exc
        if len(results) != len(normalized_starts):
            raise RuntimeError(
                f"PyBullet rayTestBatch returned {len(results)} results "
                f"for {len(normalized_starts)} rays"
            )
        return self._ray_hits_from_pybullet(results)

    def ray_test_indexed_hits(
        self,
        starts: tuple[Vec3, ...],
        ends: tuple[Vec3, ...],
        *,
        collision_mask: int,
    ) -> tuple[tuple[int, RayHit], ...]:
        """查询内部已验证射线，只返回命中项及原始索引供实时点云使用。"""
        mask = _require_collision_mask(collision_mask)
        if type(starts) is not tuple or type(ends) is not tuple:
            raise ValueError("indexed ray inputs must be prevalidated tuples")
        if len(starts) != len(ends):
            raise ValueError("starts and ends must have the same length")
        if len(starts) > PYBULLET_MAX_RAY_BATCH_SIZE:
            raise ValueError(
                f"ray batch must contain at most {PYBULLET_MAX_RAY_BATCH_SIZE} rays"
            )
        if not starts:
            return ()

        raw_results = p.rayTestBatch(
            starts,
            ends,
            numThreads=0,
            collisionFilterMask=mask,
            physicsClientId=self.client_id,
        )
        try:
            results = tuple(raw_results)
        except TypeError as exc:
            raise RuntimeError("PyBullet rayTestBatch returned a non-sequence") from exc
        if len(results) != len(starts):
            raise RuntimeError(
                f"PyBullet rayTestBatch returned {len(results)} results "
                f"for {len(starts)} rays"
            )
        return self._indexed_hits_from_pybullet(results)

    def _ray_test_indexed_hits_ndarray(
        self,
        starts: object,
        ends: object,
        *,
        collision_mask: int,
        num_threads: int = 0,
    ) -> tuple[tuple[int, RayHit], ...]:
        """读取 Stage4 scanner 已验证的 readonly float64 ndarray 射线批次。"""
        mask = _require_collision_mask(collision_mask)
        thread_count = _require_integral("num_threads", num_threads, minimum=0)
        start_count = self._require_stage4_ndarray("starts", starts)
        end_count = self._require_stage4_ndarray("ends", ends)
        if start_count != end_count:
            raise ValueError("starts and ends must have the same length")
        if start_count > PYBULLET_MAX_RAY_BATCH_SIZE:
            raise ValueError(
                f"ray batch must contain at most {PYBULLET_MAX_RAY_BATCH_SIZE} rays"
            )
        if start_count == 0:
            return ()

        raw_results = p.rayTestBatch(
            starts,
            ends,
            numThreads=thread_count,
            collisionFilterMask=mask,
            physicsClientId=self.client_id,
        )
        try:
            results = tuple(raw_results)
        except TypeError as exc:
            raise RuntimeError("PyBullet rayTestBatch returned a non-sequence") from exc
        if len(results) != start_count:
            raise RuntimeError(
                f"PyBullet rayTestBatch returned {len(results)} results "
                f"for {start_count} rays"
            )
        return self._indexed_hits_from_pybullet(results)

    @staticmethod
    def _require_stage4_ndarray(name: str, points: object) -> int:
        """避免在 Stage3 导入 NumPy，仅接受 Stage4 已冻结的二维 C 数组。"""
        shape = getattr(points, "shape", None)
        dtype = getattr(points, "dtype", None)
        flags = getattr(points, "flags", None)
        if (
            type(shape) is not tuple
            or len(shape) != 2
            or type(shape[0]) is not int
            or type(shape[1]) is not int
            or shape[0] < 0
            or shape[1] != 3
            or str(dtype) != "float64"
            or flags is None
            or not flags.c_contiguous
            or flags.writeable
        ):
            raise ValueError(
                f"{name} must be a readonly C-contiguous float64 ndarray with shape (N, 3)"
            )
        return shape[0]

    @staticmethod
    def _require_ray_sequence(name: str, points: object) -> int:
        """在取长度或迭代前拒绝无序及可能无限的批量输入。"""
        invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
        if isinstance(points, invalid_types) or not isinstance(points, Sequence):
            raise ValueError(f"{name} must be a sequence of finite 3D points")
        return len(points)

    @staticmethod
    def _normalize_ray_points(name: str, points: Sequence[Vec3]) -> tuple[Vec3, ...]:
        invalid_types = (str, bytes, set, frozenset, Mapping, Iterator)
        items = tuple(points)
        normalized: list[Vec3] | None = None
        isfinite = math.isfinite
        for index, point in enumerate(items):
            if type(point) is tuple and len(point) == 3:
                x, y, z = point
                if type(x) is float and type(y) is float and type(z) is float:
                    if not (isfinite(x) and isfinite(y) and isfinite(z)):
                        raise ValueError(f"{name}[{index}] must contain 3 finite values")
                    if normalized is not None:
                        normalized.append(point)
                    continue
            if isinstance(point, invalid_types) or not isinstance(point, Sequence) or len(point) != 3:
                raise ValueError(f"{name}[{index}] must contain 3 finite values")
            x, y, z = point
            if (
                isinstance(x, bool)
                or isinstance(y, bool)
                or isinstance(z, bool)
                or not isinstance(x, Real)
                or not isinstance(y, Real)
                or not isinstance(z, Real)
            ):
                raise ValueError(f"{name}[{index}] must contain 3 finite values")
            normalized_point = (float(x), float(y), float(z))
            if not (
                isfinite(normalized_point[0])
                and isfinite(normalized_point[1])
                and isfinite(normalized_point[2])
            ):
                raise ValueError(f"{name}[{index}] must contain 3 finite values")
            if normalized is None:
                normalized = list(items[:index])  # type: ignore[list-item]
            normalized.append(normalized_point)
        return items if normalized is None else tuple(normalized)  # type: ignore[return-value]

    def _ray_hits_from_pybullet(self, results: tuple[object, ...]) -> tuple[RayHit, ...]:
        """单循环融合校验完整 PyBullet 帧，成功后才返回 trusted RayHit。"""
        invalid_sequences = (str, bytes, set, frozenset, Mapping, Iterator)
        hits: list[RayHit] = []
        categories = self._hit_categories
        append_hit = hits.append
        trusted_hit = RayHit._from_trusted_pybullet
        isfinite = math.isfinite
        for ray_index, fields in enumerate(results):
            if type(fields) is not tuple:
                if isinstance(fields, invalid_sequences) or not isinstance(fields, Sequence):
                    raise RuntimeError(
                        f"PyBullet ray result {ray_index} must contain exactly 5 fields"
                    )
                fields = tuple(fields)
            if len(fields) != 5:
                raise RuntimeError(f"PyBullet ray result {ray_index} must contain exactly 5 fields")
            body_id, link_index, hit_fraction, position, normal = fields
            if type(body_id) is int:
                normalized_body_id = body_id
            elif isinstance(body_id, bool) or not isinstance(body_id, Integral):
                raise RuntimeError(f"PyBullet ray result {ray_index} body id must be an integer >= -1")
            else:
                normalized_body_id = int(body_id)
            if normalized_body_id < -1:
                raise RuntimeError(f"PyBullet ray result {ray_index} body id must be an integer >= -1")

            if type(link_index) is int:
                normalized_link_index = link_index
            elif isinstance(link_index, bool) or not isinstance(link_index, Integral):
                raise RuntimeError(f"PyBullet ray result {ray_index} link index must be an integer >= -1")
            else:
                normalized_link_index = int(link_index)
            if normalized_link_index < -1:
                raise RuntimeError(f"PyBullet ray result {ray_index} link index must be an integer >= -1")

            if type(hit_fraction) is float:
                normalized_fraction = hit_fraction
            elif isinstance(hit_fraction, bool) or not isinstance(hit_fraction, Real):
                raise RuntimeError(
                    f"PyBullet ray result {ray_index} hit fraction must be finite"
                )
            else:
                normalized_fraction = float(hit_fraction)
            if not isfinite(normalized_fraction):
                raise RuntimeError(
                    f"PyBullet ray result {ray_index} hit fraction must be finite"
                )
            if not 0.0 <= normalized_fraction <= 1.0:
                raise RuntimeError(
                    f"PyBullet ray result {ray_index} hit fraction must be in range 0..1"
                )

            if type(position) is tuple and len(position) == 3:
                px, py, pz = position
                if type(px) is float and type(py) is float and type(pz) is float:
                    if not (isfinite(px) and isfinite(py) and isfinite(pz)):
                        raise RuntimeError(
                            f"PyBullet ray result {ray_index} position must contain 3 finite values"
                        )
                    normalized_position = position
                else:
                    normalized_position = self._normalize_pybullet_result_vector(
                        ray_index,
                        "position",
                        position,
                    )
            else:
                normalized_position = self._normalize_pybullet_result_vector(
                    ray_index,
                    "position",
                    position,
                )

            if type(normal) is tuple and len(normal) == 3:
                nx, ny, nz = normal
                if type(nx) is float and type(ny) is float and type(nz) is float:
                    if not (isfinite(nx) and isfinite(ny) and isfinite(nz)):
                        raise RuntimeError(
                            f"PyBullet ray result {ray_index} normal must contain 3 finite values"
                        )
                else:
                    self._normalize_pybullet_result_vector(ray_index, "normal", normal)
            else:
                self._normalize_pybullet_result_vector(ray_index, "normal", normal)

            if normalized_body_id < 0 and normalized_link_index != -1:
                raise RuntimeError(
                    f"PyBullet ray result {ray_index} miss must use link index -1"
                )
            category: HitCategory = (
                categories.get(normalized_body_id, "unknown")
                if normalized_body_id >= 0
                else "unknown"
            )
            append_hit(
                trusted_hit(
                    normalized_position,
                    normalized_body_id,
                    normalized_link_index,
                    category,
                )
            )
        return tuple(hits)

    def _indexed_hits_from_pybullet(
        self,
        results: tuple[object, ...],
    ) -> tuple[tuple[int, RayHit], ...]:
        """官方 tuple/内建数值走紧凑快路，异常扩展类型回退完整边界校验。"""
        categories = self._hit_categories
        trusted_hit = RayHit._from_trusted_pybullet
        isfinite = math.isfinite
        indexed_hits: list[tuple[int, RayHit]] = []
        append_hit = indexed_hits.append

        for ray_index, fields in enumerate(results):
            if type(fields) is not tuple or len(fields) != 5:
                break
            body_id, link_index, hit_fraction, position, normal = fields
            if (
                type(body_id) is not int
                or type(link_index) is not int
                or type(hit_fraction) is not float
                or type(position) is not tuple
                or len(position) != 3
                or type(normal) is not tuple
                or len(normal) != 3
            ):
                break
            px, py, pz = position
            nx, ny, nz = normal
            if (
                type(px) is not float
                or type(py) is not float
                or type(pz) is not float
                or type(nx) is not float
                or type(ny) is not float
                or type(nz) is not float
                or body_id < -1
                or link_index < -1
                or not isfinite(hit_fraction)
                or not 0.0 <= hit_fraction <= 1.0
                or not isfinite(px)
                or not isfinite(py)
                or not isfinite(pz)
                or not isfinite(nx)
                or not isfinite(ny)
                or not isfinite(nz)
                or (body_id < 0 and link_index != -1)
            ):
                break
            if body_id >= 0:
                append_hit(
                    (
                        ray_index,
                        trusted_hit(
                            position,
                            body_id,
                            link_index,
                            categories.get(body_id, "unknown"),
                        ),
                    )
                )
        else:
            return tuple(indexed_hits)

        # 非官方形状仍由原严格入口给出精确诊断，快路径不吞掉坏帧。
        return tuple(
            (ray_index, hit)
            for ray_index, hit in enumerate(self._ray_hits_from_pybullet(results))
            if hit.hit
        )

    @staticmethod
    def _normalize_pybullet_result_vector(
        ray_index: int,
        field_name: str,
        values: object,
    ) -> Vec3:
        """只处理 PyBullet 扩展数值类型或坏帧，内建 tuple/float 走上方快路径。"""
        invalid_sequences = (str, bytes, set, frozenset, Mapping, Iterator)
        if (
            isinstance(values, invalid_sequences)
            or not isinstance(values, Sequence)
            or len(values) != 3
        ):
            raise RuntimeError(
                f"PyBullet ray result {ray_index} {field_name} must contain 3 finite values"
            )
        x, y, z = values
        if (
            isinstance(x, bool)
            or isinstance(y, bool)
            or isinstance(z, bool)
            or not isinstance(x, Real)
            or not isinstance(y, Real)
            or not isinstance(z, Real)
        ):
            raise RuntimeError(
                f"PyBullet ray result {ray_index} {field_name} must contain 3 finite values"
            )
        normalized = (float(x), float(y), float(z))
        if not (
            math.isfinite(normalized[0])
            and math.isfinite(normalized[1])
            and math.isfinite(normalized[2])
        ):
            raise RuntimeError(
                f"PyBullet ray result {ray_index} {field_name} must contain 3 finite values"
            )
        return normalized

    @staticmethod
    def _pose_from_pybullet(
        position: object,
        orientation: object,
        *,
        operation: str,
    ) -> Pose:
        try:
            return Pose(position, orientation)  # type: ignore[arg-type]
        except ValueError as exc:
            raise RuntimeError(f"PyBullet {operation} returned an invalid pose: {exc}") from exc
