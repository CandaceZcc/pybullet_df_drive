"""阶段四 E：生成内嵌锁定 manifest 的单文件 Ubuntu `.run` 安装器。"""
from __future__ import annotations

import argparse
import base64
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess
import sys
import tarfile
import textwrap
from typing import cast
from urllib.parse import urlparse


_VERSION = re.compile(r"(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)")
_HEX = re.compile(r"[0-9a-f]{64}")
_GIT_SHA = re.compile(r"[0-9a-f]{40}")
_SAFE_COMPONENT = re.compile(r"[A-Za-z0-9][A-Za-z0-9._~-]*")


def _project_payload(root: Path | None) -> tuple[str, dict[str, str]]:
    """把受控项目目录编码进安装器，并冻结每个常规文件的 SHA-256。"""
    if root is None:
        return "", {}
    if not root.is_absolute() or root.is_symlink() or not root.is_dir():
        raise ValueError("project payload must be an absolute non-symlink directory")

    archive_bytes = io.BytesIO()
    files: dict[str, str] = {}
    with tarfile.open(fileobj=archive_bytes, mode="w:gz") as archive:
        for candidate in sorted(root.rglob("*"), key=lambda path: path.as_posix()):
            if candidate.is_symlink():
                raise ValueError("project payload cannot contain symlinks")
            relative = candidate.relative_to(root)
            if candidate.is_dir():
                continue
            if not candidate.is_file():
                raise ValueError("project payload can contain only regular files")
            name = relative.as_posix()
            if not name or any(part in {"", ".", ".."} for part in relative.parts):
                raise ValueError("project payload path is invalid")
            content = candidate.read_bytes()
            member = tarfile.TarInfo(name)
            member.size = len(content)
            member.mode = candidate.stat().st_mode & 0o777
            member.mtime = 0
            member.uid = 0
            member.gid = 0
            archive.addfile(member, io.BytesIO(content))
            files[name] = hashlib.sha256(content).hexdigest()
    if not files:
        raise ValueError("project payload must contain regular files")
    return base64.b64encode(archive_bytes.getvalue()).decode("ascii"), files


def _locked_download(record: object, *, label: str, require_license: bool) -> dict[str, str]:
    """校验单个锁定下载，防止安装器接收越界路径或未声明许可证。"""
    required = {"url", "sha256", "filename"}
    if require_license:
        required.update({"name", "license"})
    if not isinstance(record, dict) or set(record) != required:
        raise ValueError(f"release {label} shape is invalid")

    url = record["url"]
    filename = record["filename"]
    digest = record["sha256"]
    parsed = urlparse(url) if isinstance(url, str) else None
    official_ros_apt_download = (
        parsed is not None
        and parsed.scheme == "http"
        and parsed.hostname == "packages.ros.org"
        and parsed.path.startswith("/ros2/ubuntu/pool/")
    )
    if (
        parsed is None
        or not parsed.hostname
        or (
            parsed.scheme != "https"
            and not (parsed.scheme == "http" and parsed.hostname in {"127.0.0.1", "localhost"})
            # ROS 2 的官方 Noble bootstrap 仅以 HTTP APT pool 发布；SHA-256 仍是安装内容信任根。
            and not official_ros_apt_download
        )
        or not isinstance(filename, str)
        or not _SAFE_COMPONENT.fullmatch(filename)
        or not isinstance(digest, str)
        or not _HEX.fullmatch(digest)
    ):
        raise ValueError(f"release {label} is invalid")
    normalized = {"url": url, "sha256": digest, "filename": filename}
    if require_license:
        name = record["name"]
        license_name = record["license"]
        if (
            not isinstance(name, str)
            or not _SAFE_COMPONENT.fullmatch(name)
            or not isinstance(license_name, str)
            or not license_name.strip()
        ):
            raise ValueError("release dependency is invalid")
        normalized.update({"name": name, "license": license_name})
    return normalized


def _current_git_sha() -> str:
    """冻结生成安装器时的仓库 commit，避免 release 身份只依赖版本号。"""
    repository = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        check=False,
        capture_output=True,
        text=True,
    )
    value = result.stdout.strip()
    if result.returncode != 0 or not _GIT_SHA.fullmatch(value):
        raise ValueError("repository Git SHA is unavailable")
    return value


def _manifest(path: Path) -> dict[str, object]:
    """读取并严格限制运行时及依赖的锁定下载合同。"""
    document = json.loads(path.read_text(encoding="utf-8"))
    required = {"schema_version", "version"}
    optional = {"payload", "with_ros", "dependencies", "ros_dependencies", "runtime_setup"}
    if (
        not isinstance(document, dict)
        or not required.issubset(document)
        or not set(document).issubset(required | optional)
    ):
        raise ValueError("release manifest shape is invalid")
    if document["schema_version"] != 1 or not isinstance(document["version"], str) or not _VERSION.fullmatch(document["version"]):
        raise ValueError("release version is invalid")
    if "with_ros" in document and not isinstance(document["with_ros"], bool):
        raise ValueError("release with_ros is invalid")
    payload = (
        _locked_download(document["payload"], label="payload", require_license=False)
        if "payload" in document
        else None
    )
    dependencies = document.get("dependencies", [])
    if not isinstance(dependencies, list):
        raise ValueError("release dependencies shape is invalid")
    ros_dependencies = document.get("ros_dependencies", [])
    if not isinstance(ros_dependencies, list):
        raise ValueError("release ROS dependencies shape is invalid")
    normalized_dependencies = [
        _locked_download(dependency, label="dependency", require_license=True)
        for dependency in dependencies
    ]
    normalized_ros_dependencies = [
        _locked_download(dependency, label="ROS dependency", require_license=True)
        for dependency in ros_dependencies
    ]
    all_dependencies = normalized_dependencies + normalized_ros_dependencies
    if len({dependency["name"] for dependency in all_dependencies}) != len(all_dependencies):
        raise ValueError("release dependency names are duplicated")
    if len({dependency["filename"] for dependency in all_dependencies}) != len(all_dependencies):
        raise ValueError("release dependency filenames are duplicated")
    runtime_setup = document.get("runtime_setup")
    if runtime_setup is not None:
        entrypoint = runtime_setup.get("entrypoint") if isinstance(runtime_setup, dict) else None
        path = Path(entrypoint) if isinstance(entrypoint, str) else None
        if (
            path is None
            or path.is_absolute()
            or path.suffix != ".py"
            or len(path.parts) < 2
            or any(not _SAFE_COMPONENT.fullmatch(part) for part in path.parts)
            or set(runtime_setup) != {"entrypoint"}
        ):
            raise ValueError("release runtime setup is invalid")
    normalized = dict(document)
    normalized["git_sha"] = _current_git_sha()
    normalized["payload"] = payload
    normalized["dependencies"] = normalized_dependencies
    normalized["ros_dependencies"] = normalized_ros_dependencies
    normalized["runtime_setup"] = runtime_setup
    return cast(dict[str, object], normalized)


_INSTALLER = r'''
import argparse
import base64
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
from http.client import IncompleteRead
from urllib.parse import urlparse
from urllib.request import urlopen

MANIFEST = json.loads(__embedded_manifest__)
PROJECT_ARCHIVE = base64.b64decode(__embedded_project_archive__)
PROJECT_FILES = json.loads(__embedded_project_files__)
EMBEDDED_PAYLOAD_SHA256 = __embedded_project_archive_sha256__
HEX = re.compile(r"[0-9a-f]{64}")
APT_PACKAGE = re.compile(r"[a-z0-9][a-z0-9+.-]*")
APT_VERSION = re.compile(r"[A-Za-z0-9.+:~_-]+")

def fail(message):
    raise RuntimeError(message)

def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

def download_locked(record, target, label):
    # 临时传输错误最多重试三次；摘要不符绝不能当成可重试的传输错误。
    for attempt in range(3):
        try:
            with urlopen(record["url"], timeout=30) as response, target.open("xb") as output:
                shutil.copyfileobj(response, output)
            break
        except (IncompleteRead, OSError):
            if target.exists() or target.is_symlink():
                target.unlink()
            if attempt == 2:
                raise
    if sha256_file(target) != record["sha256"]:
        target.unlink()
        fail("downloaded " + label + " SHA-256 differs from manifest")
    # 唯一需由 setup 直接执行的锁定下载；必须在摘要核验后才授予执行位。
    if record.get("name") == "micromamba":
        target.chmod(0o755)

def extract_project_payload(staging):
    # 内嵌项目文件只能按构建时的固定文件表解包，防止 tar 路径或类型逃逸。
    if not PROJECT_FILES:
        return
    seen = set()
    with tarfile.open(fileobj=io.BytesIO(PROJECT_ARCHIVE), mode="r:gz") as archive:
        for member in archive.getmembers():
            name = member.name
            relative = Path(name)
            if (
                not name
                or relative.is_absolute()
                or any(part in {"", ".", ".."} for part in relative.parts)
                or not member.isfile()
                or name not in PROJECT_FILES
                or name in seen
            ):
                fail("embedded project payload is invalid")
            source = archive.extractfile(member)
            if source is None:
                fail("embedded project payload is invalid")
            target = staging / relative
            target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            with source, target.open("xb") as output:
                shutil.copyfileobj(source, output)
            target.chmod(member.mode & 0o777)
            if sha256_file(target) != PROJECT_FILES[name]:
                fail("embedded project payload SHA-256 differs from manifest")
            seen.add(name)
    if seen != set(PROJECT_FILES):
        fail("embedded project payload is incomplete")

def verify_expected_files(root, expected_files):
    return all(
        (root / filename).is_file()
        and not (root / filename).is_symlink()
        and sha256_file(root / filename) == digest
        for filename, digest in expected_files.items()
    )

def release_files(root):
    # doctor 覆盖 setup 生成物，拒绝链接和未登记的特殊文件进入最终 release。
    files = {}
    for candidate in sorted(root.rglob("*"), key=lambda path: path.as_posix()):
        if candidate.is_dir():
            if candidate.is_symlink():
                fail("staging doctor found an invalid release file")
            continue
        if candidate.is_symlink() or not candidate.is_file():
            fail("staging doctor found an invalid release file")
        relative = candidate.relative_to(root).as_posix()
        if relative in {"manifest.json", "install-state.json"}:
            continue
        files[relative] = sha256_file(candidate)
    return files

def write_locked_dependency_index(staging, dependencies):
    # setup 只能消费安装器已验 SHA 的名称/路径映射，禁止自行扫描下载目录猜测归属。
    target = staging / "dependencies" / "locked-dependencies.json"
    target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with target.open("x", encoding="utf-8") as output:
        json.dump({"schema_version": 1, "dependencies": dependencies}, output, sort_keys=True)
        output.write("\n")

def bootstrap_locked_system_tools(staging):
    # 只在干净机缺少 C++ 构建工具时申请系统权限；其余安装步骤保持调用者身份。
    required_tools = {
        "python3": "python3",
        "cmake": "cmake",
        "g++": "g++",
        "make": "make",
        "bwrap": "bubblewrap",
        "xdotool": "xdotool",
    }
    required_packages = (
        *required_tools.values(),
        "libxdo3",
        "libssl-dev",
        "libyaml-cpp-dev",
    )
    missing_tools = [tool for tool in required_tools if shutil.which(tool) is None]
    missing_openssl = not Path("/usr/include/openssl/crypto.h").is_file()
    if (
        not missing_tools
        and not missing_openssl
        and os.environ.get("STAGE4_BOOTSTRAPPED_PYTHON") != "1"
    ):
        return
    lock_path = staging / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"
    try:
        document = json.loads(lock_path.read_text(encoding="utf-8"))
        platform = document["platform"]
        packages = document["apt_packages"]
    except (OSError, KeyError, TypeError, json.JSONDecodeError):
        fail("locked system dependency file is invalid")
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != 1
        or not isinstance(platform, dict)
        or platform.get("id") != "ubuntu"
        or platform.get("version_id") != "24.04"
        or platform.get("architecture") != "amd64"
        or not isinstance(packages, list)
    ):
        fail("locked system dependency file is invalid")
    locked_tools = {}
    for package in packages:
        if (
            not isinstance(package, dict)
            or set(package) != {"name", "version", "architecture"}
            or not isinstance(package["name"], str)
            or not isinstance(package["version"], str)
            or not package["version"]
            or package["architecture"] not in {"amd64", "all"}
            or package["name"] in locked_tools
        ):
            fail("locked system dependency file is invalid")
        locked_tools[package["name"]] = package
    if (
        set(required_packages) - set(locked_tools)
        or any(locked_tools[tool]["architecture"] != "amd64" for tool in required_packages)
    ):
        fail("locked system dependency file is invalid")
    install_tools = [required_tools[tool] for tool in missing_tools if tool != "python3"]
    if "xdotool" in install_tools:
        install_tools.append("libxdo3")
    if install_tools or missing_openssl:
        for package in ("libssl-dev", "libyaml-cpp-dev"):
            if package not in install_tools:
                install_tools.append(package)
    commands = []
    if install_tools:
        commands = [
            ["sudo", "apt-get", "update"],
            ["sudo", "apt-get", "install", "--yes", *install_tools],
        ]
    for command in commands:
        try:
            result = subprocess.run(command, check=False)
        except OSError:
            fail("locked system dependency bootstrap failed")
        if result.returncode != 0:
            fail("locked system dependency bootstrap failed")

def bootstrap_locked_ros(staging, dependencies, selected_with_ros):
    # ROS 的 source/key 与 Jazzy 包只在显式选择后安装，核心 release 不申请这组 sudo 权限。
    if not selected_with_ros:
        return
    source = [dependency for dependency in dependencies if dependency["name"] == "ros2-apt-source"]
    if not source:
        return
    if len(source) != 1 or not source[0]["filename"].endswith(".deb"):
        fail("locked ROS apt source is invalid")
    lock_path = staging / "packaging" / "locks" / "ros2-apt-packages.lock"
    try:
        document = json.loads(lock_path.read_text(encoding="utf-8"))
        platform = document["platform"]
        packages = document["packages"]
    except (OSError, KeyError, TypeError, json.JSONDecodeError):
        fail("locked ROS dependency file is invalid")
    if (
        not isinstance(document, dict)
        or set(document) != {"schema_version", "platform", "packages"}
        or document["schema_version"] != 1
        or not isinstance(platform, dict)
        or platform != {"id": "ubuntu", "version_id": "24.04", "architecture": "amd64"}
        or not isinstance(packages, list)
        or not packages
    ):
        fail("locked ROS dependency file is invalid")
    package_specs = []
    names = set()
    for package in packages:
        if (
            not isinstance(package, dict)
            or set(package) != {"name", "version", "architecture"}
            or not isinstance(package["name"], str)
            or not APT_PACKAGE.fullmatch(package["name"])
            or not package["name"].startswith("ros-jazzy-")
            or not isinstance(package["version"], str)
            or not APT_VERSION.fullmatch(package["version"])
            or package["architecture"] != "amd64"
            or package["name"] in names
        ):
            fail("locked ROS dependency file is invalid")
        names.add(package["name"])
        package_specs.append(package["name"] + "=" + package["version"])
    source_path = staging / "dependencies" / source[0]["filename"]
    if source_path.is_symlink() or not source_path.is_file() or sha256_file(source_path) != source[0]["sha256"]:
        fail("locked ROS apt source is invalid")
    query = shutil.which("dpkg-query")
    if query is not None:
        installed = True
        for package in packages:
            try:
                result = subprocess.run(
                    [query, "-W", "-f=${Status} ${Version} ${Architecture}", package["name"]],
                    check=False,
                    capture_output=True,
                    text=True,
                )
            except OSError:
                installed = False
                break
            expected = "install ok installed " + package["version"] + " " + package["architecture"]
            if result.returncode != 0 or result.stdout.strip() != expected:
                installed = False
                break
        if installed:
            return
    for command in (
        ["sudo", "dpkg", "--install", str(source_path)],
        ["sudo", "apt-get", "update"],
        ["sudo", "apt-get", "install", "--yes", *package_specs],
    ):
        try:
            result = subprocess.run(command, check=False)
        except OSError:
            fail("locked ROS dependency bootstrap failed")
        if result.returncode != 0:
            fail("locked ROS dependency bootstrap failed")

def run_runtime_setup(staging, selected_with_ros):
    # 入口只能是已内嵌并校验的 Python 文件；下载完依赖后才允许构建 release。
    setup = MANIFEST["runtime_setup"]
    if setup is None:
        return
    entrypoint = setup["entrypoint"]
    target = staging / entrypoint
    if (
        entrypoint not in PROJECT_FILES
        or target.is_symlink()
        or not target.is_file()
        or sha256_file(target) != PROJECT_FILES[entrypoint]
    ):
        fail("embedded runtime setup is invalid")
    result = subprocess.run(
        [
            sys.executable,
            str(target),
            "--release-root",
            str(staging),
            "--dependencies-dir",
            str(staging / "dependencies"),
            "--with-ros",
            "true" if selected_with_ros else "false",
        ],
        check=False,
    )
    if result.returncode != 0:
        fail("embedded runtime setup failed")

def current_target(root, release):
    temporary = root / (".current-" + str(os.getpid()))
    if os.path.lexists(temporary):
        fail("current activation temporary path already exists")
    try:
        temporary.symlink_to(release.relative_to(root))
        os.replace(temporary, root / "current")
    except BaseException:
        if os.path.lexists(temporary):
            temporary.unlink()
        raise

def command_target(root, command_dir):
    if command_dir is None:
        return None
    directory = Path(command_dir)
    if not directory.is_absolute():
        fail("command-dir must be absolute")
    directory.mkdir(mode=0o755, parents=True, exist_ok=True)
    if directory.is_symlink() or not directory.is_dir():
        fail("command-dir must be a non-symlink directory")
    target = directory / "runSim"
    expected = root / "current" / "bin" / "runSim"
    if os.path.lexists(target):
        if not target.is_symlink() or os.readlink(target) != str(expected):
            fail("runSim command path already exists and is unmanaged")
        return target
    return target

def publish_command_target(root, target):
    if target is None or os.path.lexists(target):
        return
    expected = root / "current" / "bin" / "runSim"
    if expected.is_symlink() or not expected.is_file() or not os.access(expected, os.X_OK):
        fail("installed runSim command is invalid")
    target.symlink_to(expected)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--install-root", required=True)
    parser.add_argument("--command-dir")
    parser.add_argument("--with-ros", action="store_true")
    args = parser.parse_args()
    root = Path(args.install_root)
    if not root.is_absolute():
        fail("install-root must be absolute")
    if "with_ros" in MANIFEST and args.with_ros != MANIFEST["with_ros"]:
        fail("installer with_ros option differs from embedded manifest")
    selected_with_ros = args.with_ros
    payload = MANIFEST["payload"]
    root.mkdir(mode=0o755, parents=True, exist_ok=True)
    command = command_target(root, args.command_dir)
    lock = (root / ".stage4-install.lock").open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        fail("another stage4 installation is active")
    releases = root / "releases"
    staging_root = root / ".staging"
    releases.mkdir(mode=0o755, exist_ok=True)
    staging_root.mkdir(mode=0o700, exist_ok=True)
    release = releases / MANIFEST["version"]
    dependencies = MANIFEST["dependencies"] + (MANIFEST["ros_dependencies"] if selected_with_ros else [])
    dependency_records = [
        {
            "name": dependency["name"],
            "license": dependency["license"],
            "filename": "dependencies/" + dependency["filename"],
            "sha256": dependency["sha256"],
        }
        for dependency in dependencies
    ]
    expected_files = {}
    if payload is not None:
        expected_files[payload["filename"]] = payload["sha256"]
    expected_files.update({dependency["filename"]: dependency["sha256"] for dependency in dependency_records})
    expected_files.update(PROJECT_FILES)
    expected = {
        "schema_version": 1,
        "version": MANIFEST["version"],
        "git_sha": MANIFEST["git_sha"],
        "payload_sha256": payload["sha256"] if payload is not None else EMBEDDED_PAYLOAD_SHA256,
        "with_ros": selected_with_ros,
        "files": expected_files,
        "dependencies": dependency_records,
        "doctor": {"files_verified": True},
    }
    def install_state_for(manifest_content):
        return {
        "schema_version": 1,
        "version": MANIFEST["version"],
        "git_sha": MANIFEST["git_sha"],
        "payload_manifest_sha256": hashlib.sha256(manifest_content.encode("utf-8")).hexdigest(),
        "with_ros": selected_with_ros,
        "dependencies": [
            {"name": dependency["name"], "sha256": dependency["sha256"]}
            for dependency in dependency_records
        ],
        "doctor": {"files_verified": True},
        }
    if release.exists():
        try:
            installed = json.loads((release / "manifest.json").read_text(encoding="utf-8"))
            installed_state = json.loads((release / "install-state.json").read_text(encoding="utf-8"))
            installed_content = (release / "manifest.json").read_text(encoding="utf-8")
            expected_state = install_state_for(installed_content)
            complete = (
                all(installed.get(key) == expected[key] for key in expected if key != "files")
                and all(installed.get("files", {}).get(path) == digest for path, digest in expected_files.items())
                and installed_state == expected_state
                and release_files(release) == installed.get("files")
            )
        except (OSError, ValueError, json.JSONDecodeError):
            complete = False
        if not complete:
            fail("same-version release differs, is damaged, or has option drift")
        current_target(root, release)
        publish_command_target(root, command)
        return
    staging = staging_root / (MANIFEST["version"] + "-" + str(os.getpid()))
    if staging.exists() or staging.is_symlink():
        fail("unique staging path already exists")
    try:
        staging.mkdir(mode=0o700)
        extract_project_payload(staging)
        bootstrap_locked_system_tools(staging)
        downloads = [] if payload is None else [(payload, staging / payload["filename"], "payload")]
        downloads.extend(
            (dependency, staging / "dependencies" / dependency["filename"], "dependency")
            for dependency in dependencies
        )
        for download, target, label in downloads:
            target.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            download_locked(download, target, label)
        bootstrap_locked_ros(staging, dependencies, selected_with_ros)
        write_locked_dependency_index(staging, dependency_records)
        if not verify_expected_files(staging, expected_files):
            fail("staging doctor found an invalid release file")
        run_runtime_setup(staging, selected_with_ros)
        expected["files"] = release_files(staging)
        manifest_content = json.dumps(expected, sort_keys=True) + "\n"
        install_state = install_state_for(manifest_content)
        install_state_content = json.dumps(install_state, sort_keys=True) + "\n"
        (staging / "manifest.json").write_text(manifest_content, encoding="utf-8")
        (staging / "install-state.json").write_text(install_state_content, encoding="utf-8")
        os.replace(staging, release)
        current_target(root, release)
        publish_command_target(root, command)
    except BaseException:
        if staging.exists() or staging.is_symlink():
            shutil.rmtree(staging)
        raise

if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("error: " + str(error), file=sys.stderr)
        raise SystemExit(1)
'''


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--project-payload", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = _manifest(args.manifest)
    project_archive, project_files = _project_payload(args.project_payload)
    if manifest["payload"] is None and not project_files:
        raise ValueError("manifest without network payload requires project payload")
    runtime_setup = manifest["runtime_setup"]
    if runtime_setup is not None and runtime_setup["entrypoint"] not in project_files:
        raise ValueError("runtime setup must be embedded in project payload")
    reserved_project_paths = set()
    if manifest["payload"] is not None:
        reserved_project_paths.add(manifest["payload"]["filename"])
    reserved_project_paths.update(
        f"dependencies/{dependency['filename']}" for dependency in manifest["dependencies"]
    )
    if reserved_project_paths.intersection(project_files):
        raise ValueError("project payload file conflicts with locked download")
    if not args.output.is_absolute() or args.output.exists() or not args.output.parent.is_dir():
        raise ValueError("output must be a new absolute path below an existing directory")
    expected_names = {
        f"slope-sim-stage4-{manifest['version']}-ubuntu24.04-amd64.run",
        "runSim.run",
    }
    if args.output.name not in expected_names:
        raise ValueError("output filename does not match release version")
    body = "#!/bin/sh\n# 阶段四单文件安装器：内嵌项目 payload 与锁定下载 manifest。\n"
    body += "if ! command -v python3 >/dev/null 2>&1; then\n"
    body += "  if ! command -v sudo >/dev/null 2>&1; then\n"
    body += "    printf '%s\\n' 'error: locked system Python bootstrap requires sudo' >&2\n"
    body += "    exit 1\n"
    body += "  fi\n"
    body += "  sudo apt-get update || exit $?\n"
    body += "  sudo apt-get install --yes python3 || exit $?\n"
    body += "  STAGE4_BOOTSTRAPPED_PYTHON=1\n"
    body += "  export STAGE4_BOOTSTRAPPED_PYTHON\n"
    body += "fi\n"
    body += "exec python3 - \"$@\" <<'STAGE4_INSTALLER_PYTHON'\n"
    body += "__embedded_manifest__ = "
    body += repr(json.dumps(manifest, sort_keys=True))
    body += "\n__embedded_project_archive__ = " + repr(project_archive)
    body += "\n__embedded_project_files__ = " + repr(json.dumps(project_files, sort_keys=True))
    body += "\n__embedded_project_archive_sha256__ = " + repr(
        hashlib.sha256(base64.b64decode(project_archive)).hexdigest()
    )
    body += "\n" + textwrap.dedent(_INSTALLER)
    body += "\nSTAGE4_INSTALLER_PYTHON\n"
    args.output.write_text(body, encoding="utf-8")
    args.output.chmod(0o755)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
