"""runSim v2 启动预检：只检查 eCAL 能力，不创建 participant 或 GUI。"""
from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256
import importlib
import os
from pathlib import Path
from typing import Callable, Mapping

from slope_sim.interfaces.v2.descriptor import _DEFAULT_DESCRIPTOR, _DEFAULT_MANIFEST
from slope_sim.interfaces.v2.topics import V2_TOPICS


_TIME_SYNC_PLUGIN_NAMES = (
    "libecaltime-localtime.so",
    "libecaltime-localtime.so.6",
    "libecaltime-localtime.so.6.1.1",
)
_REQUIRED_CORE_SYMBOLS = (
    "initialize",
    "finalize",
    "Publisher",
    "Subscriber",
    "DataTypeInformation",
    "monitoring",
)


class LegacyInterfaceModeError(ValueError):
    """正式 v2 入口拒绝静默切换到 v1 local/auto transport。"""


@dataclass(frozen=True)
class EcalPreflightIssue:
    """一条稳定、可在终端和 Dashboard 展示的预检诊断。"""

    code: str
    detail: str
    fatal: bool = True

    def __post_init__(self) -> None:
        if not self.code or any(char in self.code for char in "\r\n"):
            raise ValueError("preflight issue code must be a single nonempty line")
        if not self.detail or "\r" in self.detail or "\n" in self.detail:
            raise ValueError("preflight issue detail must be a single nonempty line")
        if type(self.fatal) is not bool:
            raise ValueError("preflight issue fatal must be a bool")


@dataclass(frozen=True)
class EcalPreflightReport:
    """启动前检查结果；peer 只能在 participant 启动后由运行期快照判定。"""

    config_path: Path | None
    time_sync_plugin_path: Path | None
    descriptor_sha256: str | None
    participant_available: bool
    peer_available: bool | None
    issues: tuple[EcalPreflightIssue, ...]

    @property
    def ok(self) -> bool:
        """只有没有 fatal 诊断时才允许继续启动正式 v2。"""
        return not any(issue.fatal for issue in self.issues)

    def format_terminal(self) -> str:
        """生成稳定的一行一诊断文本，供 runSim stderr 使用。"""
        if self.ok:
            return "eCAL preflight passed: descriptor and participant API ready; peer discovery pending"
        lines = ["eCAL preflight failed:"]
        lines.extend(f"- {issue.code}: {issue.detail}" for issue in self.issues)
        return "\n".join(lines)

    def format_dashboard(self) -> str:
        """生成 Dashboard 状态卡片文本，不泄露任意异常堆栈。"""
        if self.ok:
            return "eCAL preflight passed; waiting for verified v2 peers"
        lines = ["eCAL preflight failed; v2 realtime startup is blocked"]
        lines.extend(f"{issue.code}: {issue.detail}" for issue in self.issues)
        return "\n".join(lines)


def require_v2_interface_mode(mode: str) -> str:
    """正式 runSim v2 只接受 eCAL；local/auto 必须显式作为 legacy 处理。"""
    if mode == "ecal":
        return mode
    if mode in {"local", "auto"}:
        raise LegacyInterfaceModeError(
            f"{mode} interface mode is legacy v1; runSim v2 requires --interface-mode ecal"
        )
    raise ValueError("v2 interface mode must be 'ecal'")


def run_v2_ecal_preflight(
    *,
    environment: Mapping[str, str] | None = None,
    descriptor_path: Path = _DEFAULT_DESCRIPTOR,
    manifest_path: Path | None = _DEFAULT_MANIFEST,
    core_loader: Callable[[], object] | None = None,
) -> EcalPreflightReport:
    """验证配置、time-sync、descriptor 和 raw Python API，不初始化 eCAL。"""
    env = os.environ if environment is None else environment
    issues: list[EcalPreflightIssue] = []
    config_path = _resolve_config_path(env)
    if config_path is None or not config_path.is_file():
        issues.append(
            EcalPreflightIssue(
                "ecal_config_missing",
                "set ECAL_CONFIG_PATH or ECAL_DATA to a directory containing ecal.yaml",
            )
        )

    plugin_path = _resolve_time_sync_plugin(env)
    if plugin_path is None:
        issues.append(
            EcalPreflightIssue(
                "ecal_time_sync_plugin_missing",
                "set ECAL_TIME_PLUGIN_PATH to a directory containing libecaltime-localtime.so",
            )
        )

    digest: str | None = None
    try:
        descriptor = descriptor_path.resolve(strict=True)
        if not descriptor.is_file():
            raise OSError("descriptor is not a regular file")
        payload = descriptor.read_bytes()
        digest = sha256(payload).hexdigest()
        if manifest_path is not None:
            expected = bytes.fromhex(manifest_path.read_text(encoding="ascii").strip()).hex()
            if digest != expected:
                issues.append(
                    EcalPreflightIssue(
                        "v2_descriptor_mismatch",
                        "descriptor SHA-256 differs from proto/slope_sim_interfaces_v2.sha256",
                    )
                )
    except (OSError, ValueError, UnicodeError):
        issues.append(
            EcalPreflightIssue(
                "v2_descriptor_missing",
                f"descriptor file is unavailable: {descriptor_path}",
            )
        )
        descriptor = None

    participant_available = False
    try:
        core = (importlib.import_module("ecal.nanobind_core") if core_loader is None else core_loader())
        participant_available = all(callable(getattr(core, name, None)) for name in ("initialize", "finalize")) and all(
            getattr(core, name, None) is not None
            for name in ("Publisher", "Subscriber", "DataTypeInformation", "monitoring")
        )
    except Exception:
        participant_available = False
    if not participant_available:
        issues.append(
            EcalPreflightIssue(
                "ecal_participant_api_missing",
                "ecal.nanobind_core lacks the required participant/publisher/subscriber API",
            )
        )

    return EcalPreflightReport(
        config_path=None if config_path is None else config_path.resolve(),
        time_sync_plugin_path=None if plugin_path is None else plugin_path.resolve(),
        descriptor_sha256=digest,
        participant_available=participant_available,
        peer_available=None,
        issues=tuple(issues),
    )


def evaluate_v2_peer_snapshot(snapshot: object) -> EcalPreflightReport:
    """将运行期 transport 快照转换为同一份可展示的 peer 诊断。"""
    qualities = getattr(snapshot, "topic_quality", ())
    quality_by_topic = {getattr(item, "topic", None): item for item in qualities}
    missing = []
    unverified = []
    for contract in V2_TOPICS:
        quality = quality_by_topic.get(contract.topic)
        if quality is None or getattr(quality, "peer_count", 0) < 1:
            missing.append(contract.topic)
        elif getattr(quality, "protocol_state", None) != "verified":
            unverified.append(contract.topic)
    issues: list[EcalPreflightIssue] = []
    if missing:
        issues.append(EcalPreflightIssue("v2_peer_missing", f"no discovered peer for {', '.join(missing)}"))
    if unverified:
        issues.append(EcalPreflightIssue("v2_peer_unverified", f"peer metadata is not verified for {', '.join(unverified)}"))
    if not getattr(snapshot, "ecal_connected", False) and not issues:
        issues.append(EcalPreflightIssue("ecal_disconnected", "eCAL participant is not connected"))
    return EcalPreflightReport(None, None, None, True, not issues, tuple(issues))


def _resolve_config_path(environment: Mapping[str, str]) -> Path | None:
    explicit = environment.get("ECAL_CONFIG_PATH")
    if explicit:
        return Path(explicit)
    data_dir = environment.get("ECAL_DATA")
    if data_dir:
        return Path(data_dir) / "ecal.yaml"
    candidate = Path.cwd() / "ecal.yaml"
    return candidate if candidate.exists() else None


def _resolve_time_sync_plugin(environment: Mapping[str, str]) -> Path | None:
    value = environment.get("ECAL_TIME_PLUGIN_PATH")
    if not value:
        return None
    path = Path(value)
    if path.is_file() and path.name in _TIME_SYNC_PLUGIN_NAMES:
        return path
    if not path.is_dir():
        return None
    for name in _TIME_SYNC_PLUGIN_NAMES:
        candidate = path / name
        if candidate.is_file():
            return candidate
    return None
