# Dashboard 图表核心：集中定义旧遥测图、企业接口图规格与无 Qt 历史缓存。
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from numbers import Real

from slope_sim.interfaces.config import InterfaceConfig
from slope_sim.interfaces.dashboard_snapshot import InterfaceDashboardSnapshot
from slope_sim.model_registry import RobotModelSpec
from slope_sim.telemetry import RobotTelemetry


LEGACY_PLOT_TABS = ("轨迹", "速度/命令")
INTERFACE_LINE_PLOT_TABS = (
    "驱动命令",
    "驱动反馈",
    "转向命令",
    "转向反馈",
    "RTK位置",
    "RTK航向",
    "IMU姿态",
    "轮组频率",
    "传感频率",
    "接口异常",
)
INTERFACE_BUSINESS_TABS = INTERFACE_LINE_PLOT_TABS[:7]
INTERFACE_QUALITY_TABS = INTERFACE_LINE_PLOT_TABS[7:]
_FIXED_LINE_KEYS = {
    "RTK位置": ("rtk_x", "rtk_y", "rtk_z"),
    "RTK航向": ("rtk_yaw",),
    "IMU姿态": ("imu_roll", "imu_pitch"),
    "轮组频率": ("command_hz", "wheel_state_hz"),
    "传感频率": ("lidar_front_hz", "lidar_rear_hz", "rtk_hz", "imu_hz"),
    "接口异常": ("errors_per_sec", "drops_per_sec"),
}

DASHBOARD_PLOT_LEGEND_STYLE = {
    "fontsize": 7,
    "framealpha": 0.65,
    "borderpad": 0.25,
    "handlelength": 1.2,
    "loc": "upper right",
}


class TelemetryPlotBuffer:
    """保存最近一段时间的旧遥测样本，继续兼容阶段三前公共 API。"""

    def __init__(self, window_sec: float = 20.0) -> None:
        if isinstance(window_sec, bool) or not isinstance(window_sec, Real):
            raise ValueError("window_sec must be positive")
        normalized = float(window_sec)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError("window_sec must be positive")
        self.window_sec = normalized
        self._samples: deque[RobotTelemetry] = deque()

    def append(self, telemetry: RobotTelemetry) -> None:
        """加入新样本，并保留闭区间时间窗。"""
        self._samples.append(telemetry)
        cutoff = telemetry.t - self.window_sec
        while self._samples and self._samples[0].t < cutoff:
            self._samples.popleft()

    def clear(self) -> None:
        self._samples.clear()

    def series(self) -> dict[str, list[float]]:
        """返回 Matplotlib `set_data` 可直接使用的独立列表。"""
        fields_to_plot = (
            "t",
            "x",
            "y",
            "command_linear_velocity",
            "body_forward_speed",
            "command_angular_velocity",
            "yaw_rate",
            "left_slip_ratio",
            "right_slip_ratio",
            "left_slip_speed",
            "right_slip_speed",
            "left_contact_normal_force",
            "right_contact_normal_force",
            "left_contact_friction_force",
            "right_contact_friction_force",
            "left_contact_count",
            "right_contact_count",
        )
        result = {
            field: [float(getattr(sample, field)) for sample in self._samples]
            for field in fields_to_plot
        }
        result["left_abs_slip_ratio"] = [abs(value) for value in result["left_slip_ratio"]]
        result["right_abs_slip_ratio"] = [abs(value) for value in result["right_slip_ratio"]]
        return result


@dataclass(frozen=True, slots=True)
class DashboardPlotLine:
    """旧遥测图中的一条 x/y 字段映射线。"""

    key: str
    x_field: str
    y_field: str
    label: str


@dataclass(frozen=True, slots=True)
class DashboardPlotSpec:
    """旧遥测图规格；一个一级页签对应一张图。"""

    tab_label: str
    title: str
    x_label: str
    y_label: str
    lines: tuple[DashboardPlotLine, ...]
    equal_aspect: bool = False


@dataclass(frozen=True, slots=True)
class ChartLineSpec:
    """企业接口图中一条以 `t` 为横轴的低密度曲线。"""

    key: str
    label: str


@dataclass(frozen=True, slots=True)
class InterfaceChartSpec:
    """企业接口折线页规格。"""

    tab_label: str
    title: str
    x_label: str
    y_label: str
    lines: tuple[ChartLineSpec, ...]


def dashboard_plot_specs() -> list[DashboardPlotSpec]:
    """返回主 Dashboard 保留的两张实时图，诊断指标不进入一级页签。"""
    return [
        DashboardPlotSpec(
            "轨迹",
            "x/y trajectory",
            "x [m]",
            "y [m]",
            (DashboardPlotLine("trajectory", "x", "y", "xy"),),
            equal_aspect=True,
        ),
        DashboardPlotSpec(
            "速度/命令",
            "command vs actual",
            "t [s]",
            "value",
            (
                DashboardPlotLine("command_linear_velocity", "t", "command_linear_velocity", "cmd v"),
                DashboardPlotLine("body_forward_speed", "t", "body_forward_speed", "body v"),
                DashboardPlotLine("command_angular_velocity", "t", "command_angular_velocity", "cmd yaw"),
                DashboardPlotLine("yaw_rate", "t", "yaw_rate", "yaw_rate"),
            ),
        ),
    ]


def interface_chart_specs(model: RobotModelSpec) -> list[InterfaceChartSpec]:
    """按当前车型真实轮组数量生成十个企业折线页规格。"""
    if type(model) is not RobotModelSpec:
        raise ValueError("model must be an exact RobotModelSpec")
    drive_labels = ("FL", "FR", "RL", "RR") if len(model.drive_joint_names) == 4 else ("L", "R")
    steering_labels = ("FL", "FR") if model.steering_joint_names else ()

    def lines(prefix: str, labels: tuple[str, ...]) -> tuple[ChartLineSpec, ...]:
        return tuple(
            ChartLineSpec(f"{prefix}_{index}", label)
            for index, label in enumerate(labels)
        )

    return [
        InterfaceChartSpec("驱动命令", "drive command", "t [s]", "rad/s", lines("drive_command", drive_labels)),
        InterfaceChartSpec("驱动反馈", "drive feedback", "t [s]", "rad/s", lines("drive_feedback", drive_labels)),
        InterfaceChartSpec("转向命令", "steering command", "t [s]", "rad/s", lines("steering_command", steering_labels)),
        InterfaceChartSpec("转向反馈", "steering feedback", "t [s]", "rad", lines("steering_feedback", steering_labels)),
        InterfaceChartSpec(
            "RTK位置",
            "RTK position",
            "t [s]",
            "m",
            (ChartLineSpec("rtk_x", "x"), ChartLineSpec("rtk_y", "y"), ChartLineSpec("rtk_z", "z")),
        ),
        InterfaceChartSpec("RTK航向", "RTK heading", "t [s]", "rad", (ChartLineSpec("rtk_yaw", "yaw"),)),
        InterfaceChartSpec(
            "IMU姿态",
            "IMU attitude",
            "t [s]",
            "rad",
            (ChartLineSpec("imu_roll", "roll"), ChartLineSpec("imu_pitch", "pitch")),
        ),
        InterfaceChartSpec(
            "轮组频率",
            "wheel frequency",
            "wall time [s]",
            "Hz",
            (ChartLineSpec("command_hz", "command rx"), ChartLineSpec("wheel_state_hz", "wheel state tx")),
        ),
        InterfaceChartSpec(
            "传感频率",
            "sensor frequency",
            "wall time [s]",
            "Hz",
            (
                ChartLineSpec("lidar_front_hz", "front LiDAR"),
                ChartLineSpec("lidar_rear_hz", "rear LiDAR"),
                ChartLineSpec("rtk_hz", "RTK"),
                ChartLineSpec("imu_hz", "IMU"),
            ),
        ),
        InterfaceChartSpec(
            "接口异常",
            "interface errors",
            "wall time [s]",
            "events/s",
            (ChartLineSpec("errors_per_sec", "errors/s"), ChartLineSpec("drops_per_sec", "drops/s")),
        ),
    ]


class InterfaceChartBuffer:
    """按话题独立维护接口业务历史和墙钟质量历史。"""

    def __init__(self, window_sec: float = 20.0, interface_config: InterfaceConfig | None = None) -> None:
        if isinstance(window_sec, bool) or not isinstance(window_sec, Real):
            raise ValueError("window_sec must be a positive finite number")
        normalized = float(window_sec)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError("window_sec must be a positive finite number")
        if type(interface_config) is not InterfaceConfig:
            raise ValueError("interface_config must be an exact InterfaceConfig")
        self.window_sec = normalized
        self.interface_config = interface_config
        self.clear()

    def clear(self) -> None:
        """清空业务、质量、代际和聚合计数基线。"""
        self._generation: int | None = None
        self._robot_model: RobotModelSpec | None = None
        self._rows: dict[str, deque[dict[str, float]]] = {
            tab: deque() for tab in INTERFACE_LINE_PLOT_TABS
        }
        self._last_topic_time_ns: dict[str, int] = {}
        self._last_quality_time: float | None = None
        self._counter_baseline: tuple[float, int, int] | None = None
        self._business_horizon: float | None = None
        self._quality_horizon: float | None = None

    @staticmethod
    def _finite_values(values: tuple[float, ...], expected: int) -> bool:
        return len(values) == expected and all(math.isfinite(value) for value in values)

    def _append_row(self, tab_label: str, row: dict[str, float]) -> None:
        self._rows[tab_label].append(row)

    def _advance_horizon(self, current: float | None, candidate: float) -> float | None:
        """接受有限且不倒退的统一时间；逆序值保持现有 horizon。"""
        if not math.isfinite(candidate):
            return current
        if current is None or candidate >= current:
            return candidate
        return current

    def _prune_tabs(self, tab_labels: tuple[str, ...], horizon: float | None) -> set[str]:
        """按统一 horizon 主动裁剪所有页，并报告确实删除过行的页签。"""
        if horizon is None:
            return set()
        cutoff = horizon - self.window_sec
        changed: set[str] = set()
        for tab_label in tab_labels:
            rows = self._rows[tab_label]
            removed = False
            while rows and rows[0]["t"] < cutoff:
                rows.popleft()
                removed = True
            if removed:
                changed.add(tab_label)
        return changed

    def _new_topic_time(self, topic: str, timestamp_ns: object) -> float | None:
        """只接受严格递增 uint64 时间，拒绝值不会推进该话题基线。"""
        if type(timestamp_ns) is not int or not 0 <= timestamp_ns <= (1 << 64) - 1:
            return None
        previous = self._last_topic_time_ns.get(topic)
        if previous is not None and timestamp_ns <= previous:
            return None
        return timestamp_ns / 1_000_000_000.0

    def _append_wheel_messages(self, snapshot: InterfaceDashboardSnapshot) -> set[str]:
        changed: set[str] = set()
        model = self._robot_model
        if model is None:
            return changed
        drive_count = len(model.drive_joint_names)
        steering_count = len(model.steering_joint_names)

        command = snapshot.wheel_command
        command_time_ns = snapshot.wheel_command_received_sim_time_ns
        command_t = self._new_topic_time("wheel_command", command_time_ns)
        if command is not None and command_t is not None:
            drive = command.drive_wheel_speed_rad_s
            steering = command.steering_wheel_speed_rad_s
            if self._finite_values(drive, drive_count) and self._finite_values(steering, steering_count):
                self._append_row(
                    "驱动命令",
                    {"t": command_t, **{f"drive_command_{index}": value for index, value in enumerate(drive)}},
                )
                changed.add("驱动命令")
                if steering_count:
                    self._append_row(
                        "转向命令",
                        {"t": command_t, **{f"steering_command_{index}": value for index, value in enumerate(steering)}},
                    )
                    changed.add("转向命令")
                self._last_topic_time_ns["wheel_command"] = command_time_ns

        state = snapshot.wheel_state
        state_t = self._new_topic_time("wheel_state", None if state is None else state.timestamp_ns)
        if state is not None and state_t is not None:
            drive = state.drive_wheel_speed_rad_s
            steering = state.steering_wheel_angle_rad
            if self._finite_values(drive, drive_count) and self._finite_values(steering, steering_count):
                self._append_row(
                    "驱动反馈",
                    {"t": state_t, **{f"drive_feedback_{index}": value for index, value in enumerate(drive)}},
                )
                changed.add("驱动反馈")
                if steering_count:
                    self._append_row(
                        "转向反馈",
                        {"t": state_t, **{f"steering_feedback_{index}": value for index, value in enumerate(steering)}},
                    )
                    changed.add("转向反馈")
                self._last_topic_time_ns["wheel_state"] = state.timestamp_ns
        return changed

    def _append_pose_messages(self, snapshot: InterfaceDashboardSnapshot) -> set[str]:
        changed: set[str] = set()
        rtk = snapshot.rtk
        rtk_t = self._new_topic_time("rtk", None if rtk is None else rtk.timestamp_ns)
        if rtk is not None and rtk_t is not None:
            values = (rtk.main_x, rtk.main_y, rtk.main_z, rtk.baseline_yaw_rad)
            if all(math.isfinite(value) for value in values):
                self._append_row("RTK位置", {"t": rtk_t, "rtk_x": values[0], "rtk_y": values[1], "rtk_z": values[2]})
                self._append_row("RTK航向", {"t": rtk_t, "rtk_yaw": values[3]})
                self._last_topic_time_ns["rtk"] = rtk.timestamp_ns
                changed.update(("RTK位置", "RTK航向"))

        imu = snapshot.imu
        imu_t = self._new_topic_time("imu", None if imu is None else imu.timestamp_ns)
        if imu is not None and imu_t is not None:
            if math.isfinite(imu.roll_rad) and math.isfinite(imu.pitch_rad):
                self._append_row("IMU姿态", {"t": imu_t, "imu_roll": imu.roll_rad, "imu_pitch": imu.pitch_rad})
                self._last_topic_time_ns["imu"] = imu.timestamp_ns
                changed.add("IMU姿态")
        return changed

    def _append_quality(self, snapshot: InterfaceDashboardSnapshot) -> set[str]:
        status = snapshot.status
        captured_at = status.captured_at
        if not math.isfinite(captured_at):
            return set()
        if self._last_quality_time is not None and captured_at <= self._last_quality_time:
            return set()
        changed: set[str] = set()

        wheel_topic = status.topics.get(self.interface_config.wheel_state.topic)
        wheel_values = (
            status.command.valid_hz,
            None if wheel_topic is None else wheel_topic.actual_hz,
        )
        if all(value is not None and math.isfinite(value) and value >= 0.0 for value in wheel_values):
            self._append_row(
                "轮组频率",
                {"t": captured_at, "command_hz": float(wheel_values[0]), "wheel_state_hz": float(wheel_values[1])},
            )
            changed.add("轮组频率")

        sensor_channels = (
            ("lidar_front_hz", self.interface_config.lidar_front),
            ("lidar_rear_hz", self.interface_config.lidar_rear),
            ("rtk_hz", self.interface_config.rtk),
            ("imu_hz", self.interface_config.imu),
        )
        sensor_row: dict[str, float] = {"t": captured_at}
        for key, channel in sensor_channels:
            topic = status.topics.get(channel.topic)
            if topic is None or not math.isfinite(topic.actual_hz) or topic.actual_hz < 0.0:
                break
            sensor_row[key] = topic.actual_hz
        else:
            self._append_row("传感频率", sensor_row)
            changed.add("传感频率")

        configured_topics = [status.topics.get(channel.topic) for channel in self.interface_config.channels]
        if all(topic is not None for topic in configured_topics):
            total_errors = sum(topic.error_count for topic in configured_topics if topic is not None)
            total_drops = sum(topic.dropped_count for topic in configured_topics if topic is not None)
            errors_per_sec = 0.0
            drops_per_sec = 0.0
            baseline = self._counter_baseline
            if baseline is not None:
                baseline_time, baseline_errors, baseline_drops = baseline
                if total_errors >= baseline_errors and total_drops >= baseline_drops:
                    elapsed = captured_at - baseline_time
                    errors_per_sec = (total_errors - baseline_errors) / elapsed
                    drops_per_sec = (total_drops - baseline_drops) / elapsed
            self._counter_baseline = (captured_at, total_errors, total_drops)
            self._append_row(
                "接口异常",
                {"t": captured_at, "errors_per_sec": errors_per_sec, "drops_per_sec": drops_per_sec},
            )
            changed.add("接口异常")

        self._last_quality_time = captured_at
        return changed

    def append(self, snapshot: InterfaceDashboardSnapshot, *, paused: bool = False) -> set[str]:
        """按消息时间戳去重，并返回数据实际变化的页签。"""
        if type(snapshot) is not InterfaceDashboardSnapshot:
            raise ValueError("snapshot must be an exact InterfaceDashboardSnapshot")
        if type(paused) is not bool:
            raise ValueError("paused must be a bool")
        model_changed = (
            self._robot_model is not None
            and snapshot.robot_model != self._robot_model.name
        )
        if snapshot.generation != self._generation or model_changed:
            self.clear()
            self._generation = snapshot.generation
            from slope_sim.model_registry import get_robot_model

            self._robot_model = get_robot_model(snapshot.robot_model)
        elif self._robot_model is None:
            from slope_sim.model_registry import get_robot_model

            self._robot_model = get_robot_model(snapshot.robot_model)

        changed = self._append_quality(snapshot)
        if not paused:
            changed.update(self._append_wheel_messages(snapshot))
            changed.update(self._append_pose_messages(snapshot))
        # 统一时间必须淘汰静默话题，不能只依赖当前追加行的消息时间。
        business_candidate = snapshot.sim_time_ns / 1_000_000_000.0
        self._business_horizon = self._advance_horizon(
            self._business_horizon,
            business_candidate,
        )
        changed.update(
            self._prune_tabs(INTERFACE_BUSINESS_TABS, self._business_horizon)
        )
        self._quality_horizon = self._advance_horizon(
            self._quality_horizon,
            snapshot.status.captured_at,
        )
        changed.update(
            self._prune_tabs(INTERFACE_QUALITY_TABS, self._quality_horizon)
        )
        return changed

    def series(self, tab_label: str) -> dict[str, list[float]]:
        """返回新的字典和列表，绝不暴露内部 deque 或行字典。"""
        if tab_label not in INTERFACE_LINE_PLOT_TABS:
            raise ValueError(f"unknown interface chart tab: {tab_label}")
        model = self._robot_model
        if model is None:
            keys = tuple(ChartLineSpec(key, key) for key in _FIXED_LINE_KEYS.get(tab_label, ()))
        else:
            specs = interface_chart_specs(model)
            keys = next(spec.lines for spec in specs if spec.tab_label == tab_label)
        result = {"t": []}
        for line in keys:
            result[line.key] = []
        for row in self._rows[tab_label]:
            result["t"].append(float(row["t"]))
            for line in keys:
                result[line.key].append(float(row[line.key]))
        return result
