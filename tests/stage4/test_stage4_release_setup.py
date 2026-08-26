"""阶段四 E：release 内 C++ 构建 setup 的聚焦验收。"""
from __future__ import annotations

import json
import hashlib
import io
from pathlib import Path
import subprocess
import stat
import sys
import tarfile
import zipfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
SETUP = ROOT / "scripts" / "stage4_release_setup.py"
ROS_APT_LOCK = ROOT / "packaging" / "locks" / "ros2-apt-packages.lock"
_GIB = 1024**3


@pytest.mark.parametrize(
    "cpu_count,mem_available_bytes,expected",
    (
        (8, 9 * _GIB, 4),
        (16, 32 * _GIB, 8),
        (8, 4 * _GIB, 1),
        (None, 16 * _GIB, 1),
        (8, None, 1),
    ),
)
def test_release_setup_selects_memory_aware_parallel_build_jobs(
    cpu_count: int | None,
    mem_available_bytes: int | None,
    expected: int,
) -> None:
    """自动并行必须用 CPU 和可用内存共同限流，探测失败时单任务降级。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_resolve_build_jobs"])

    assert module._resolve_build_jobs(
        cpu_count=cpu_count,
        mem_available_bytes=mem_available_bytes,
        override=None,
    ) == expected


@pytest.mark.parametrize("override", ("0", "9", "1.5", "fast", ""))
def test_release_setup_rejects_invalid_parallel_build_override(override: str) -> None:
    """显式并行度只能是当前 CPU 范围内的正整数。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_resolve_build_jobs"])

    with pytest.raises(ValueError, match="SLOPE_SIM_BUILD_JOBS"):
        module._resolve_build_jobs(
            cpu_count=8,
            mem_available_bytes=16 * _GIB,
            override=override,
        )


def test_release_setup_uses_bounded_ccache_when_available(tmp_path: Path) -> None:
    """可用 ccache 必须跨 staging 复用，同时把单一缓存限制在 5 GiB。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_cmake_build_context"])
    release = tmp_path / "release"
    release.mkdir()

    context = module._cmake_build_context(
        release,
        base_environment={"PATH": "/usr/bin"},
        cpu_count=8,
        mem_available_bytes=9 * _GIB,
        build_jobs_override=None,
        ccache="/usr/bin/ccache",
    )

    assert context.jobs == "4"
    assert context.configure_options == (
        "-DCMAKE_C_COMPILER_LAUNCHER=/usr/bin/ccache",
        "-DCMAKE_CXX_COMPILER_LAUNCHER=/usr/bin/ccache",
    )
    assert context.environment["CCACHE_BASEDIR"] == str(release)
    assert context.environment["CCACHE_MAXSIZE"] == "5G"


def test_release_setup_keeps_parallel_build_without_ccache(tmp_path: Path) -> None:
    """无 ccache 的干净机仍应使用自动并行，且不伪造 launcher 选项。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_cmake_build_context"])
    release = tmp_path / "release"
    release.mkdir()

    context = module._cmake_build_context(
        release,
        base_environment={"PATH": "/usr/bin"},
        cpu_count=8,
        mem_available_bytes=9 * _GIB,
        build_jobs_override=None,
        ccache=None,
    )

    assert context.jobs == "4"
    assert context.configure_options == ()
    assert "CCACHE_BASEDIR" not in context.environment


def test_release_setup_installs_payload_run_sim_into_release_bin(tmp_path: Path) -> None:
    """payload 的正式启动入口必须复制到安装前缀的 bin/。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_install_run_sim_launcher"])
    release = tmp_path / "release"
    release.mkdir()
    source = release / "runSim"
    source.write_text("#!/bin/sh\necho runSim\n", encoding="utf-8")
    source.chmod(0o755)

    module._install_run_sim_launcher(release)

    installed = release / "bin" / "runSim"
    assert installed.read_text(encoding="utf-8") == source.read_text(encoding="utf-8")
    assert installed.stat().st_mode & 0o111


def test_release_setup_installs_the_locked_official_livox_viewer_bundle(
    tmp_path: Path,
) -> None:
    """官方 Viewer ZIP 必须进入固定 release 路径并恢复启动权限。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_install_locked_livox_viewer"])
    release = tmp_path / "release"
    dependencies = release / "dependencies"
    dependencies.mkdir(parents=True)
    archive = dependencies / "Viewer2_2.6.0_Linux.zip"
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr("Viewer2_2.6.0_Linux/LivoxViewer2.sh", "#!/bin/sh\n")
        output.writestr(
            "Viewer2_2.6.0_Linux/LivoxViewer2/Binaries/Linux/LivoxViewer2",
            "viewer binary\n",
        )
    locked = [{
        "name": "livox-viewer2-linux",
        "license": "Livox Viewer 2 proprietary binary",
        "filename": "dependencies/Viewer2_2.6.0_Linux.zip",
        "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
    }]

    installed = module._install_locked_livox_viewer(release, dependencies, locked)

    assert installed == (
        release / "share" / "slope-sim" / "livox-viewer" / "Viewer2_2.6.0_Linux"
    )
    assert (installed / "LivoxViewer2.sh").stat().st_mode & 0o111
    assert (
        installed / "LivoxViewer2" / "Binaries" / "Linux" / "LivoxViewer2"
    ).stat().st_mode & 0o111


@pytest.mark.parametrize("unsafe_name", [
    "../escape",
    "Viewer2_2.6.0_Linux/../escape",
])
def test_release_setup_rejects_livox_viewer_zip_path_escape(
    tmp_path: Path, unsafe_name: str,
) -> None:
    """Viewer ZIP 的绝对根外成员不得写出固定 release 目录。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_install_locked_livox_viewer"])
    release = tmp_path / "release"
    dependencies = release / "dependencies"
    dependencies.mkdir(parents=True)
    archive = dependencies / "Viewer2_2.6.0_Linux.zip"
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr(unsafe_name, "escape")
    locked = [{
        "name": "livox-viewer2-linux",
        "filename": "dependencies/Viewer2_2.6.0_Linux.zip",
    }]

    with pytest.raises(ValueError, match="locked Livox Viewer bundle is invalid"):
        module._install_locked_livox_viewer(release, dependencies, locked)

    assert not (release / "share" / "slope-sim" / "livox-viewer").exists()
    assert not (release / "share" / "slope-sim" / "escape").exists()


def test_release_setup_rejects_livox_viewer_zip_symlink(tmp_path: Path) -> None:
    """Viewer ZIP 不能借 POSIX 符号链接绕过目标路径检查。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_install_locked_livox_viewer"])
    release = tmp_path / "release"
    dependencies = release / "dependencies"
    dependencies.mkdir(parents=True)
    archive = dependencies / "Viewer2_2.6.0_Linux.zip"
    link = zipfile.ZipInfo("Viewer2_2.6.0_Linux/LivoxViewer2.sh")
    link.create_system = 3
    link.external_attr = (stat.S_IFLNK | 0o777) << 16
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr(link, "/tmp/escape")
    locked = [{
        "name": "livox-viewer2-linux",
        "filename": "dependencies/Viewer2_2.6.0_Linux.zip",
    }]

    with pytest.raises(ValueError, match="locked Livox Viewer bundle is invalid"):
        module._install_locked_livox_viewer(release, dependencies, locked)


def test_release_setup_rejects_duplicate_livox_viewer_zip_members(tmp_path: Path) -> None:
    """重复成员不能靠后写入覆盖已验证的 Viewer 文件。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_install_locked_livox_viewer"])
    release = tmp_path / "release"
    dependencies = release / "dependencies"
    dependencies.mkdir(parents=True)
    archive = dependencies / "Viewer2_2.6.0_Linux.zip"
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr("Viewer2_2.6.0_Linux/LivoxViewer2.sh", "first")
        with pytest.warns(UserWarning, match="Duplicate name"):
            output.writestr("Viewer2_2.6.0_Linux/LivoxViewer2.sh", "second")
    locked = [{
        "name": "livox-viewer2-linux",
        "filename": "dependencies/Viewer2_2.6.0_Linux.zip",
    }]

    with pytest.raises(ValueError, match="locked Livox Viewer bundle is invalid"):
        module._install_locked_livox_viewer(release, dependencies, locked)


@pytest.mark.parametrize("limit_name", ["_LIVOX_VIEWER_MAX_FILES", "_LIVOX_VIEWER_MAX_BYTES"])
def test_release_setup_rejects_livox_viewer_zip_over_bounds(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, limit_name: str,
) -> None:
    """Viewer ZIP 的成员数或声明解压体积超过上限时必须 fail closed。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_install_locked_livox_viewer"])
    release = tmp_path / "release"
    dependencies = release / "dependencies"
    dependencies.mkdir(parents=True)
    archive = dependencies / "Viewer2_2.6.0_Linux.zip"
    with zipfile.ZipFile(archive, "w") as output:
        output.writestr("Viewer2_2.6.0_Linux/LivoxViewer2.sh", "#!/bin/sh\n")
        output.writestr(
            "Viewer2_2.6.0_Linux/LivoxViewer2/Binaries/Linux/LivoxViewer2",
            "viewer binary\n",
        )
    monkeypatch.setattr(module, limit_name, 1)
    locked = [{
        "name": "livox-viewer2-linux",
        "filename": "dependencies/Viewer2_2.6.0_Linux.zip",
    }]

    with pytest.raises(ValueError, match="locked Livox Viewer bundle is invalid"):
        module._install_locked_livox_viewer(release, dependencies, locked)


def test_release_setup_installs_locked_ecal_configuration(tmp_path: Path) -> None:
    """安装 release 后 runSim 必须有本地 ecal.yaml，而非依赖构建临时目录。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_install_ecal_runtime_config"])
    prefix = tmp_path / "prefix"
    release = tmp_path / "release"
    source = prefix / "etc" / "ecal" / "ecal.yaml"
    source.parent.mkdir(parents=True)
    release.mkdir()
    source.write_text("ecal:\n", encoding="utf-8")

    module._install_ecal_runtime_config(prefix, release)

    assert (release / "etc" / "ecal" / "ecal.yaml").read_text(encoding="utf-8") == "ecal:\n"


def test_release_setup_ros_bridge_launcher_exposes_release_libraries(tmp_path: Path) -> None:
    """Bridge wrapper 必须让 ROS 动态加载器找到 release 内 Livox 类型支持库。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_install_ros_bridge_launcher"])
    release = tmp_path / "release"
    bridge = release / "bin" / "slope_sim_stage4_ros2_bridge"
    bridge.parent.mkdir(parents=True)
    bridge.write_text("binary", encoding="utf-8")
    bridge.chmod(0o755)

    module._install_ros_bridge_launcher(release)

    launcher = bridge.read_text(encoding="utf-8")
    assert 'export LD_LIBRARY_PATH="$release_root/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"' in launcher


def test_stage4_runtime_lock_covers_import_time_pandas_dependency() -> None:
    """生产 runtime 的唯一环境声明和显式 lock 都必须包含 pandas。"""
    environment = (ROOT / "packaging" / "python-environment.yml").read_text(encoding="utf-8")
    lock = (ROOT / "packaging" / "locks" / "python-linux-64.lock").read_text(encoding="utf-8")

    assert "  - pandas\n" in environment
    assert "/pandas-" in lock


def test_stage4_runtime_lock_covers_rc_serial_dependency() -> None:
    """干净安装的生产 runtime 必须能导入遥控器串口依赖。"""
    environment = (ROOT / "packaging" / "python-environment.yml").read_text(encoding="utf-8")
    conda_lock = (ROOT / "packaging" / "locks" / "python.conda-lock.yml").read_text(
        encoding="utf-8"
    )
    explicit_lock = (ROOT / "packaging" / "locks" / "python-linux-64.lock").read_text(
        encoding="utf-8"
    )

    assert "  - pyserial\n" in environment
    assert "- name: pyserial\n" in conda_lock
    assert "/pyserial-" in explicit_lock


def test_release_setup_bypasses_the_slow_proxy_only_for_locked_conda_runtime(
    tmp_path: Path, monkeypatch
) -> None:
    """Conda runtime 下载只绕过已验证慢代理，其他安装下载路由不受影响。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_create_locked_python_runtime"])
    release = tmp_path / "release"
    micromamba = release / "dependencies" / "micromamba"
    lock = release / "packaging" / "locks" / "python-linux-64.lock"
    micromamba.parent.mkdir(parents=True)
    micromamba.write_text("#!/bin/sh\n", encoding="utf-8")
    micromamba.chmod(0o755)
    lock.parent.mkdir(parents=True)
    lock.write_text("https://conda.anaconda.org/conda-forge/linux-64/example.conda#digest\n", encoding="utf-8")
    captured: dict[str, object] = {}

    def fake_run(command, **kwargs):
        if "env" in kwargs:
            captured["command"] = command
            captured["environment"] = kwargs["env"]
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, "run", fake_run)
    monkeypatch.setattr(module, "_locked_runtime_python", lambda runtime: runtime / "bin" / "python")
    monkeypatch.setattr(module, "_install_locked_python_wheels", lambda *args: None)
    monkeypatch.setenv("HTTP_PROXY", "http://127.0.0.1:7897")
    monkeypatch.setenv("HTTPS_PROXY", "http://127.0.0.1:7897")
    monkeypatch.setenv("http_proxy", "http://127.0.0.1:7897")
    monkeypatch.setenv("https_proxy", "http://127.0.0.1:7897")
    monkeypatch.setenv("NO_PROXY", "localhost,127.0.0.1")

    assert module._create_locked_python_runtime(
        release,
        micromamba.parent,
        [{"name": "micromamba", "filename": "dependencies/micromamba", "license": "BSD-3-Clause", "sha256": "0" * 64}],
        release / ".stage4-build",
    ) == release / "runtime"
    environment = captured["environment"]
    assert isinstance(environment, dict)
    assert "HTTP_PROXY" not in environment
    assert "HTTPS_PROXY" not in environment
    assert "http_proxy" not in environment
    assert "https_proxy" not in environment
    assert environment["NO_PROXY"] == "localhost,127.0.0.1,conda.anaconda.org"


def test_release_setup_runs_locked_micromamba_with_supported_basename(
    tmp_path: Path, monkeypatch
) -> None:
    """Conda post-link 脚本必须看到标准 micromamba 可执行文件名。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_create_locked_python_runtime"])
    release = tmp_path / "release"
    locked_micromamba = release / "dependencies" / "micromamba-linux-64"
    lock = release / "packaging" / "locks" / "python-linux-64.lock"
    build_root = release / ".stage4-build"
    locked_micromamba.parent.mkdir(parents=True)
    locked_micromamba.write_bytes(b"locked micromamba\n")
    locked_micromamba.chmod(0o755)
    lock.parent.mkdir(parents=True)
    lock.write_text("https://conda.anaconda.org/conda-forge/linux-64/example.conda#digest\n", encoding="utf-8")
    captured: dict[str, object] = {}

    def fake_run(command, **kwargs):
        if "env" in kwargs:
            captured["command"] = command
            captured["environment"] = kwargs["env"]
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, "run", fake_run)
    monkeypatch.setattr(module, "_locked_runtime_python", lambda runtime: runtime / "bin" / "python")
    monkeypatch.setattr(module, "_install_locked_python_wheels", lambda *args: None)

    module._create_locked_python_runtime(
        release,
        locked_micromamba.parent,
        [
            {
                "name": "micromamba",
                "filename": "dependencies/micromamba-linux-64",
                "license": "BSD-3-Clause",
                "sha256": "0" * 64,
            }
        ],
        build_root,
    )

    runner = build_root / "micromamba"
    command = captured["command"]
    environment = captured["environment"]
    assert isinstance(command, list)
    assert isinstance(environment, dict)
    assert command[0] == str(runner)
    assert environment["MAMBA_EXE"] == str(runner)
    assert runner.read_bytes() == locked_micromamba.read_bytes()
    assert runner.stat().st_mode & 0o111


def test_release_setup_probes_runtime_imports(tmp_path: Path, monkeypatch) -> None:
    """锁定 Python 创建后必须验证串口、MCAP 与双 OpenGL 依赖可导入。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_create_locked_python_runtime"])
    release = tmp_path / "release"
    micromamba = release / "dependencies" / "micromamba"
    lock = release / "packaging" / "locks" / "python-linux-64.lock"
    micromamba.parent.mkdir(parents=True)
    micromamba.write_text("#!/bin/sh\n", encoding="utf-8")
    micromamba.chmod(0o755)
    lock.parent.mkdir(parents=True)
    lock.write_text("https://conda.anaconda.org/conda-forge/linux-64/example.conda#digest\n", encoding="utf-8")
    commands: list[list[str]] = []

    def fake_run(command, **kwargs):
        commands.append(command)
        return subprocess.CompletedProcess(command, 0)

    runtime_python = release / "runtime" / "bin" / "python"
    monkeypatch.setattr(module.subprocess, "run", fake_run)
    monkeypatch.setattr(module, "_locked_runtime_python", lambda runtime: runtime_python)
    monkeypatch.setattr(module, "_install_locked_python_wheels", lambda *args: None)

    module._create_locked_python_runtime(
        release,
        micromamba.parent,
        [{"name": "micromamba", "filename": "dependencies/micromamba", "license": "BSD-3-Clause", "sha256": "0" * 64}],
        release / ".stage4-build",
    )

    assert commands[-1] == [
        str(runtime_python),
        "-c",
        "import serial, mcap.reader, pyqtgraph.opengl, OpenGL.GL",
    ]


def test_ros_apt_lock_covers_livox_message_generation() -> None:
    """干净机 ROS 构建必须锁定 CustomMsg 的 generator、runtime 与 typesupport。"""
    document = json.loads(ROS_APT_LOCK.read_text(encoding="utf-8"))
    packages = {package["name"]: package["version"] for package in document["packages"]}

    assert packages["ros-jazzy-rosidl-default-generators"] == "1.6.1-1noble.20260612.064900"
    assert packages["ros-jazzy-rosidl-default-runtime"] == "1.6.1-1noble.20260615.090842"
    assert packages["ros-jazzy-rosidl-typesupport-cpp"] == "3.2.3-1noble.20260612.051324"
    assert packages["ros-jazzy-rosidl-typesupport-fastrtps-c"] == "3.6.4-1noble.20260612.051449"
    assert packages["ros-jazzy-rosidl-typesupport-fastrtps-cpp"] == "3.6.4-1noble.20260612.051139"


def test_release_setup_builds_and_installs_cpp_tools_without_shipping_build_tree(tmp_path: Path) -> None:
    """setup 必须在 release 内构建 C++ 工具、安装产物并清理临时构建目录。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(stage4_release_fixture LANGUAGES CXX)\n"
        "add_executable(slope_sim_stage4_fixture main.cpp)\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text(
        "#include <iostream>\n"
        "int main() { std::cout << \"stage4 fixture\\n\"; }\n",
        encoding="utf-8",
    )
    dependencies = release / "dependencies"
    dependencies.mkdir()
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps({"schema_version": 1, "dependencies": []}), encoding="utf-8"
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    installed = release / "bin" / "slope_sim_stage4_fixture"
    assert installed.is_file() and installed.stat().st_mode & 0o111
    assert subprocess.run([str(installed)], check=True, capture_output=True, text=True).stdout == "stage4 fixture\n"
    assert not (release / ".stage4-build").exists()
    assert json.loads((release / "share" / "slope-sim" / "runtime-setup.json").read_text(encoding="utf-8")) == {
        "cpp_tools_built": True,
        "with_ros": False,
    }


def test_release_setup_adds_jazzy_prefix_only_for_ros_build(tmp_path: Path) -> None:
    """ROS release 必须显式传入 Jazzy 与已锁定的 Livox 消息源码。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(stage4_ros_prefix_fixture LANGUAGES CXX)\n"
        "if(NOT STAGE4_WITH_ROS)\n"
        "  message(FATAL_ERROR \"ROS build flag was not enabled\")\n"
        "endif()\n"
        "list(FIND CMAKE_PREFIX_PATH \"/opt/ros/jazzy\" jazzy_index)\n"
        "if(jazzy_index EQUAL -1)\n"
        "  message(FATAL_ERROR \"Jazzy prefix was not supplied\")\n"
        "endif()\n"
        "if(NOT EXISTS \"${STAGE4_LIVOX_MSG_SOURCE}/msg/CustomMsg.msg\")\n"
        "  message(FATAL_ERROR \"Livox CustomMsg source was not supplied\")\n"
        "endif()\n"
        "find_package(ament_cmake REQUIRED)\n"
        "if(NOT \"$ENV{AMENT_PREFIX_PATH}\" MATCHES \"(^|:)/opt/ros/jazzy(:|$)\")\n"
        "  message(FATAL_ERROR \"Jazzy ament prefix was not supplied\")\n"
        "endif()\n"
        "if(NOT \"$ENV{ROS_DISTRO}\" STREQUAL \"jazzy\")\n"
        "  message(FATAL_ERROR \"Jazzy ROS distribution was not supplied\")\n"
        "endif()\n"
        "execute_process(COMMAND /usr/bin/python3 -c \"import ament_package\" "
        "RESULT_VARIABLE ament_package_result)\n"
        "if(NOT ament_package_result EQUAL 0)\n"
        "  message(FATAL_ERROR \"Jazzy Python environment was not supplied\")\n"
        "endif()\n"
        "if(NOT STAGE4_ROSIDL_PYTHON_EXECUTABLE STREQUAL \"/usr/bin/python3\")\n"
        "  message(FATAL_ERROR \"Jazzy ROSIDL Python was not supplied\")\n"
        "endif()\n"
        "add_executable(slope_sim_stage4_fixture main.cpp)\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text("int main() { return 0; }\n", encoding="utf-8")
    dependencies = release / "dependencies"
    dependencies.mkdir()
    apt_source = dependencies / "ros2-apt-source_1.2.0~noble_all.deb"
    apt_source.write_bytes(b"locked apt source\n")
    livox_archive = dependencies / "livox_ros_driver2-1.2.6.tar.gz"
    with tarfile.open(livox_archive, "w:gz") as archive:
        for name, payload in (
            ("livox_ros_driver2-1.2.6/msg/CustomMsg.msg", b"uint64 timebase\n"),
            ("livox_ros_driver2-1.2.6/msg/CustomPoint.msg", b"float32 x\n"),
        ):
            member = tarfile.TarInfo(name)
            member.size = len(payload)
            archive.addfile(member, io.BytesIO(payload))
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "ros2-apt-source",
                        "license": "Apache-2.0",
                        "filename": "dependencies/ros2-apt-source_1.2.0~noble_all.deb",
                        "sha256": hashlib.sha256(apt_source.read_bytes()).hexdigest(),
                    },
                    {
                        "name": "livox_ros_driver2",
                        "license": "MIT",
                        "filename": "dependencies/livox_ros_driver2-1.2.6.tar.gz",
                        "sha256": hashlib.sha256(livox_archive.read_bytes()).hexdigest(),
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "true",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert (release / "bin" / "slope_sim_stage4_fixture").is_file()


def test_ros_release_installs_a_bridge_launcher_that_loads_jazzy(tmp_path: Path) -> None:
    """ROS bridge 的正式入口必须自行加载 Jazzy，普通 shell 不需要手工 source。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(stage4_ros_bridge_launcher_fixture LANGUAGES CXX)\n"
        "if(NOT STAGE4_WITH_ROS)\n"
        "  message(FATAL_ERROR \"ROS build flag was not enabled\")\n"
        "endif()\n"
        "add_executable(slope_sim_stage4_ros2_bridge main.cpp)\n"
        "install(TARGETS slope_sim_stage4_ros2_bridge RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text(
        "#include <iostream>\n"
        "int main(int argc, char** argv) {\n"
        "  std::cout << \"ROS_DISTRO=\" << (std::getenv(\"ROS_DISTRO\") ?: \"\")\n"
        "            << \" livox_runtime=\" << (std::getenv(\"SLOPE_SIM_LIVOX_RUNTIME\") ?: \"\")\n"
        "            << \" arg=\" << (argc > 1 ? argv[1] : \"\") << \"\\n\";\n"
        "}\n",
        encoding="utf-8",
    )
    dependencies = release / "dependencies"
    dependencies.mkdir()
    livox_archive = dependencies / "livox_ros_driver2-1.2.6.tar.gz"
    with tarfile.open(livox_archive, "w:gz") as archive:
        for name, payload in (
            ("livox_ros_driver2-1.2.6/msg/CustomMsg.msg", b"uint64 timebase\n"),
            ("livox_ros_driver2-1.2.6/msg/CustomPoint.msg", b"float32 x\n"),
        ):
            member = tarfile.TarInfo(name)
            member.size = len(payload)
            archive.addfile(member, io.BytesIO(payload))
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "livox_ros_driver2",
                        "license": "MIT",
                        "filename": "dependencies/livox_ros_driver2-1.2.6.tar.gz",
                        "sha256": hashlib.sha256(livox_archive.read_bytes()).hexdigest(),
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    local_setup = release / "share" / "livox_ros_driver2" / "local_setup.sh"
    local_setup.parent.mkdir(parents=True)
    local_setup.write_text("export SLOPE_SIM_LIVOX_RUNTIME=ready\n", encoding="utf-8")

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "true",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    launcher = release / "bin" / "slope_sim_stage4_ros2_bridge"
    private_binary = release / "bin" / "slope_sim_stage4_ros2_bridge.bin"
    assert launcher.is_file() and launcher.stat().st_mode & 0o111
    assert private_binary.is_file() and private_binary.stat().st_mode & 0o111
    environment = {"PATH": f"{release / 'bin'}:/usr/bin:/bin"}
    launched = subprocess.run(
        [launcher.name, "--fixture-argument"],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    assert launched.returncode == 0, launched.stderr
    assert launched.stdout == "ROS_DISTRO=jazzy livox_runtime=ready arg=--fixture-argument\n"


def test_release_setup_materializes_internal_runtime_file_links(tmp_path: Path) -> None:
    """最终 release 必须消除 runtime 内部文件链接，才能通过 fail-closed doctor。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(stage4_release_fixture LANGUAGES CXX)\n"
        "add_executable(slope_sim_stage4_fixture main.cpp)\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text("int main() { return 0; }\n", encoding="utf-8")
    runtime = release / "runtime" / "bin"
    runtime.mkdir(parents=True)
    target = runtime / "python3.10"
    target.write_text("runtime python\n", encoding="utf-8")
    linked = runtime / "python"
    linked.symlink_to(target.name)
    dependencies = release / "dependencies"
    dependencies.mkdir()
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps({"schema_version": 1, "dependencies": []}), encoding="utf-8"
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert linked.is_file() and not linked.is_symlink()
    assert linked.read_text(encoding="utf-8") == "runtime python\n"


def test_release_setup_materializes_internal_runtime_directory_links(tmp_path: Path) -> None:
    """最终 release 也必须物化 runtime 内部目录链接。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(stage4_release_fixture LANGUAGES CXX)\n"
        "add_executable(slope_sim_stage4_fixture main.cpp)\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text("int main() { return 0; }\n", encoding="utf-8")
    icu_root = release / "runtime" / "lib" / "icu"
    versioned = icu_root / "76.1"
    versioned.mkdir(parents=True)
    (versioned / "icudata.dat").write_text("icu data\n", encoding="utf-8")
    linked = icu_root / "current"
    linked.symlink_to(versioned.name, target_is_directory=True)
    dependencies = release / "dependencies"
    dependencies.mkdir()
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps({"schema_version": 1, "dependencies": []}), encoding="utf-8"
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert linked.is_dir() and not linked.is_symlink()
    assert (linked / "icudata.dat").read_text(encoding="utf-8") == "icu data\n"


def test_release_setup_rejects_invalid_locked_dependency_index_before_building(tmp_path: Path) -> None:
    """setup 不得在未校验的 dependency 文件名或 SHA 上开始 CMake 构建。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\nproject(fixture LANGUAGES CXX)\n",
        encoding="utf-8",
    )
    dependencies = release / "dependencies"
    dependencies.mkdir()
    payload = b"locked dependency\n"
    (dependencies / "ecal.tar.gz").write_bytes(payload)
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "ecal",
                        "license": "Apache-2.0",
                        "filename": "dependencies/../ecal.tar.gz",
                        "sha256": hashlib.sha256(payload).hexdigest(),
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 1
    assert completed.stderr == "error: locked dependency index is invalid\n"
    assert not (release / ".stage4-build").exists()


def test_release_setup_extracts_locked_source_archive_only_inside_temporary_build_tree(tmp_path: Path) -> None:
    """锁定源码归档必须在本轮 build tree 解压，供 CMake 使用后不留在 release。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(fixture LANGUAGES CXX)\n"
        "if(NOT EXISTS \"${CMAKE_BINARY_DIR}/sources/fixture-source/source.marker\")\n"
        "  message(FATAL_ERROR \"locked fixture source was not extracted\")\n"
        "endif()\n"
        "add_executable(slope_sim_stage4_fixture main.cpp)\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text("int main() { return 0; }\n", encoding="utf-8")
    dependencies = release / "dependencies"
    dependencies.mkdir()
    archive = dependencies / "fixture-source.tar.gz"
    with tarfile.open(archive, "w:gz") as output:
        member = tarfile.TarInfo("fixture-source-v1/source.marker")
        payload = b"locked source\n"
        member.size = len(payload)
        output.addfile(member, io.BytesIO(payload))
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "fixture-source",
                        "license": "Apache-2.0",
                        "filename": "dependencies/fixture-source.tar.gz",
                        "sha256": digest,
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert not (release / ".stage4-build").exists()
    assert not (release / "sources").exists()


def test_release_setup_builds_locked_protobuf_prefix_and_passes_it_to_phase0(tmp_path: Path) -> None:
    """setup 必须从锁定源码临时构建 Protobuf，并把唯一前缀传给 Phase-0。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(fixture LANGUAGES CXX)\n"
        "if(NOT STAGE4_PROTOC_EXECUTABLE STREQUAL \"${CMAKE_PREFIX_PATH}/bin/protoc\")\n"
        "  message(FATAL_ERROR \"protoc must come from the temporary dependency prefix\")\n"
        "endif()\n"
        "if(NOT EXISTS \"${STAGE4_PROTOC_EXECUTABLE}\")\n"
        "  message(FATAL_ERROR \"locked protoc was not installed\")\n"
        "endif()\n"
        "if(NOT EXISTS \"${STAGE4_MCAP_INCLUDE_DIR}/mcap/mcap.hpp\")\n"
        "  message(FATAL_ERROR \"locked MCAP headers were not passed\")\n"
        "endif()\n"
        "add_executable(slope_sim_stage4_fixture main.cpp)\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text("int main() { return 0; }\n", encoding="utf-8")
    dependencies = release / "dependencies"
    dependencies.mkdir()

    def write_archive(filename: str, members: dict[str, tuple[bytes, int]]) -> dict[str, str]:
        archive = dependencies / filename
        with tarfile.open(archive, "w:gz") as output:
            for name, (payload, mode) in members.items():
                member = tarfile.TarInfo(name)
                member.size = len(payload)
                member.mode = mode
                output.addfile(member, io.BytesIO(payload))
        return {
            "filename": f"dependencies/{filename}",
            "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        }

    protobuf = write_archive(
        "protobuf.tar.gz",
        {
            "protobuf-v33.6/CMakeLists.txt": (
                b"cmake_minimum_required(VERSION 3.28)\n"
                b"project(fake_protobuf LANGUAGES NONE)\n"
                b"install(PROGRAMS protoc DESTINATION bin)\n",
                0o644,
            ),
            "protobuf-v33.6/protoc": (b"#!/bin/sh\necho 'libprotoc 33.6'\n", 0o755),
        },
    )
    mcap = write_archive(
        "mcap.tar.gz",
        {
            "mcap-v1.4.0/cpp/mcap/include/mcap/mcap.hpp": (
                b"// locked MCAP fixture\n",
                0o644,
            )
        },
    )
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {"name": "protobuf", "license": "BSD-3-Clause", **protobuf},
                    {"name": "mcap", "license": "MIT", **mcap},
                ],
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert (release / "bin" / "slope_sim_stage4_fixture").is_file()
    assert not (release / ".stage4-build").exists()


def test_release_setup_keeps_prefix_runtime_libraries_for_installed_cpp_tools(tmp_path: Path) -> None:
    """临时 dependency prefix 清理后，已安装 C++ 工具仍必须加载其锁定共享库。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(fixture LANGUAGES C)\n"
        "find_library(STAGE4_RUNTIME_SUPPORT NAMES stage4_runtime_support "
        "PATHS \"${CMAKE_PREFIX_PATH}/lib\" NO_DEFAULT_PATH REQUIRED)\n"
        "add_executable(slope_sim_stage4_fixture main.c)\n"
        "target_link_libraries(slope_sim_stage4_fixture PRIVATE \"${STAGE4_RUNTIME_SUPPORT}\")\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.c").write_text(
        "int stage4_runtime_support(void);\n"
        "int main(void) { return stage4_runtime_support() == 7 ? 0 : 1; }\n",
        encoding="utf-8",
    )
    dependencies = release / "dependencies"
    dependencies.mkdir()
    protobuf = dependencies / "protobuf.tar.gz"
    with tarfile.open(protobuf, "w:gz") as output:
        for name, payload, mode in (
            (
                "protobuf-v33.6/CMakeLists.txt",
                b"cmake_minimum_required(VERSION 3.28)\n"
                b"project(fake_protobuf LANGUAGES C)\n"
                b"add_library(stage4_runtime_support SHARED runtime_support.c)\n"
                b"set_target_properties(stage4_runtime_support PROPERTIES VERSION 1.0 SOVERSION 1)\n"
                b"install(TARGETS stage4_runtime_support LIBRARY DESTINATION lib)\n"
                b"install(PROGRAMS protoc DESTINATION bin)\n",
                0o644,
            ),
            ("protobuf-v33.6/runtime_support.c", b"int stage4_runtime_support(void) { return 7; }\n", 0o644),
            ("protobuf-v33.6/protoc", b"#!/bin/sh\necho 'libprotoc 33.6'\n", 0o755),
        ):
            member = tarfile.TarInfo(name)
            member.size = len(payload)
            member.mode = mode
            output.addfile(member, io.BytesIO(payload))
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "protobuf",
                        "license": "BSD-3-Clause",
                        "filename": "dependencies/protobuf.tar.gz",
                        "sha256": hashlib.sha256(protobuf.read_bytes()).hexdigest(),
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    installed = release / "bin" / "slope_sim_stage4_fixture"
    assert subprocess.run([str(installed)], check=False).returncode == 0
    assert (release / "lib" / "libstage4_runtime_support.so.1.0").is_file()
    assert not (release / ".stage4-build").exists()


def test_release_setup_builds_locked_abseil_before_protobuf(tmp_path: Path) -> None:
    """Protobuf 源码依赖的 Abseil 必须先进入同一临时 prefix，禁止 CMake 联网补取。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(fixture LANGUAGES CXX)\n"
        "find_package(absl CONFIG REQUIRED)\n"
        "if(NOT TARGET absl::base)\n"
        "  message(FATAL_ERROR \"locked Abseil target was not exported\")\n"
        "endif()\n"
        "add_executable(slope_sim_stage4_fixture main.cpp)\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text("int main() { return 0; }\n", encoding="utf-8")
    dependencies = release / "dependencies"
    dependencies.mkdir()

    def write_archive(filename: str, members: dict[str, tuple[bytes, int]]) -> dict[str, str]:
        archive = dependencies / filename
        with tarfile.open(archive, "w:gz") as output:
            for name, (payload, mode) in members.items():
                member = tarfile.TarInfo(name)
                member.size = len(payload)
                member.mode = mode
                output.addfile(member, io.BytesIO(payload))
        return {
            "filename": f"dependencies/{filename}",
            "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        }

    abseil = write_archive(
        "abseil-cpp.tar.gz",
        {
            "abseil-cpp/CMakeLists.txt": (
                b"cmake_minimum_required(VERSION 3.28)\n"
                b"project(fake_abseil LANGUAGES NONE)\n"
                b"add_library(base INTERFACE)\n"
                b"install(TARGETS base EXPORT abslTargets)\n"
                b"install(EXPORT abslTargets NAMESPACE absl:: DESTINATION lib/cmake/absl)\n"
                b"install(FILES abslConfig.cmake DESTINATION lib/cmake/absl)\n",
                0o644,
            ),
            "abseil-cpp/abslConfig.cmake": (
                b'include("${CMAKE_CURRENT_LIST_DIR}/abslTargets.cmake")\n',
                0o644,
            ),
        },
    )
    protobuf = write_archive(
        "protobuf.tar.gz",
        {
            "protobuf-v33.6/CMakeLists.txt": (
                b"cmake_minimum_required(VERSION 3.28)\n"
                b"project(fake_protobuf LANGUAGES CXX)\n"
                b"find_package(absl CONFIG REQUIRED NO_DEFAULT_PATH "
                b"PATHS \"${CMAKE_PREFIX_PATH}/lib/cmake/absl\")\n"
                b"if(NOT TARGET absl::base)\n"
                b"  message(FATAL_ERROR \"locked Abseil target was not imported\")\n"
                b"endif()\n"
                b"install(PROGRAMS protoc DESTINATION bin)\n",
                0o644,
            ),
            "protobuf-v33.6/protoc": (b"#!/bin/sh\necho 'libprotoc 33.6'\n", 0o755),
        },
    )
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {"name": "abseil-cpp", "license": "Apache-2.0", **abseil},
                    {"name": "protobuf", "license": "BSD-3-Clause", **protobuf},
                ],
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert (release / "bin" / "slope_sim_stage4_fixture").is_file()
    assert not (release / ".stage4-build").exists()


def test_release_setup_builds_locked_ecal_into_the_protobuf_prefix(tmp_path: Path) -> None:
    """eCAL 必须使用同轮 Protobuf 前缀，并且只接受归档内完整子模块闭包。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(fixture LANGUAGES CXX)\n"
        "if(NOT EXISTS \"${CMAKE_PREFIX_PATH}/bin/protoc\")\n"
        "  message(FATAL_ERROR \"locked protoc was not installed\")\n"
        "endif()\n"
        "if(NOT EXISTS \"${CMAKE_PREFIX_PATH}/lib/cmake/eCAL/eCALConfig.cmake\")\n"
        "  message(FATAL_ERROR \"locked eCAL was not installed into the Protobuf prefix\")\n"
        "endif()\n"
        "if(NOT EXISTS \"${STAGE4_MCAP_INCLUDE_DIR}/mcap/mcap.hpp\")\n"
        "  message(FATAL_ERROR \"locked MCAP headers were not passed\")\n"
        "endif()\n"
        "add_executable(slope_sim_stage4_fixture main.cpp)\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text("int main() { return 0; }\n", encoding="utf-8")
    dependencies = release / "dependencies"
    dependencies.mkdir()

    def write_archive(filename: str, members: dict[str, tuple[bytes, int]]) -> dict[str, str]:
        archive = dependencies / filename
        with tarfile.open(archive, "w:gz") as output:
            for name, (payload, mode) in members.items():
                member = tarfile.TarInfo(name)
                member.size = len(payload)
                member.mode = mode
                output.addfile(member, io.BytesIO(payload))
        return {
            "filename": f"dependencies/{filename}",
            "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        }

    protobuf = write_archive(
        "protobuf.tar.gz",
        {
            "protobuf-v33.6/CMakeLists.txt": (
                b"cmake_minimum_required(VERSION 3.28)\n"
                b"project(fake_protobuf LANGUAGES NONE)\n"
                b"install(PROGRAMS protoc DESTINATION bin)\n",
                0o644,
            ),
            "protobuf-v33.6/protoc": (b"#!/bin/sh\necho 'libprotoc 33.6'\n", 0o755),
        },
    )
    ecal_members = {
        "ecal-v6.1.1/CMakeLists.txt": (
            b"cmake_minimum_required(VERSION 3.28)\n"
            b"project(fake_ecal LANGUAGES NONE)\n"
            b"find_package(CMakeFunctions REQUIRED CONFIG)\n"
            b"install(FILES eCALConfig.cmake DESTINATION lib/cmake/eCAL)\n",
            0o644,
        ),
        "ecal-v6.1.1/eCALConfig.cmake": (b"# locked eCAL fixture\n", 0o644),
        "ecal-v6.1.1/cmake/submodule_dependencies.cmake": (b"# locked closure fixture\n", 0o644),
        "ecal-v6.1.1/thirdparty/cmakefunctions/cmake_functions/CMakeLists.txt": (
            b"cmake_minimum_required(VERSION 3.28)\n"
            b"project(fake_cmakefunctions LANGUAGES NONE)\n"
            b"install(FILES CMakeFunctionsConfig.cmake DESTINATION lib/cmake/CMakeFunctions)\n",
            0o644,
        ),
        "ecal-v6.1.1/thirdparty/cmakefunctions/cmake_functions/CMakeFunctionsConfig.cmake": (
            b"# locked CMakeFunctions fixture\n",
            0o644,
        ),
        "ecal-v6.1.1/.gitmodules": (b"", 0o644),
    }
    required_submodules = (
        "thirdparty/asio/asio",
        "thirdparty/ecaludp/ecaludp",
        "thirdparty/protozero/protozero",
        "thirdparty/recycle/recycle",
        "thirdparty/tclap/tclap",
        "thirdparty/tcp_pubsub/tcp_pubsub",
        "thirdparty/yaml-cpp/yaml-cpp",
    )
    ecal_members["ecal-v6.1.1/.gitmodules"] = (
        "".join(f'[submodule "{path}"]\n\tpath = {path}\n' for path in required_submodules).encode(),
        0o644,
    )
    for path in required_submodules:
        ecal_members[f"ecal-v6.1.1/{path}/source.marker"] = (b"locked\n", 0o644)
    ecal = write_archive("ecal.tar.gz", ecal_members)
    mcap = write_archive(
        "mcap.tar.gz",
        {
            "mcap-v1.4.0/cpp/mcap/include/mcap/mcap.hpp": (
                b"// locked MCAP fixture\n",
                0o644,
            )
        },
    )
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {"name": "protobuf", "license": "BSD-3-Clause", **protobuf},
                    {"name": "ecal", "license": "Apache-2.0", **ecal},
                    {"name": "mcap", "license": "MIT", **mcap},
                ],
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert (release / "bin" / "slope_sim_stage4_fixture").is_file()
    assert not (release / ".stage4-build").exists()


def test_release_setup_extracts_locked_noble_ecal_deb_into_the_temporary_prefix(tmp_path: Path) -> None:
    """Ubuntu 24.04 的官方 eCAL deb 必须只在本轮 prefix 解包并供 Phase-0 发现。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(fixture LANGUAGES CXX)\n"
        "if(NOT EXISTS \"${CMAKE_PREFIX_PATH}/bin/protoc\")\n"
        "  message(FATAL_ERROR \"locked protoc was not installed\")\n"
        "endif()\n"
        "if(NOT EXISTS \"${eCAL_DIR}/eCALConfig.cmake\")\n"
        "  message(FATAL_ERROR \"locked eCAL deb config was not discovered\")\n"
        "endif()\n"
        "if(NOT EXISTS \"${STAGE4_MCAP_INCLUDE_DIR}/mcap/mcap.hpp\")\n"
        "  message(FATAL_ERROR \"locked MCAP headers were not passed\")\n"
        "endif()\n"
        "add_executable(slope_sim_stage4_fixture main.cpp)\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text("int main() { return 0; }\n", encoding="utf-8")
    dependencies = release / "dependencies"
    dependencies.mkdir()

    def write_archive(filename: str, members: dict[str, tuple[bytes, int]]) -> dict[str, str]:
        archive = dependencies / filename
        with tarfile.open(archive, "w:gz") as output:
            for name, (payload, mode) in members.items():
                member = tarfile.TarInfo(name)
                member.size = len(payload)
                member.mode = mode
                output.addfile(member, io.BytesIO(payload))
        return {
            "filename": f"dependencies/{filename}",
            "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        }

    protobuf = write_archive(
        "protobuf.tar.gz",
        {
            "protobuf-v33.6/CMakeLists.txt": (
                b"cmake_minimum_required(VERSION 3.28)\n"
                b"project(fake_protobuf LANGUAGES NONE)\n"
                b"install(PROGRAMS protoc DESTINATION bin)\n",
                0o644,
            ),
            "protobuf-v33.6/protoc": (b"#!/bin/sh\necho 'libprotoc 33.6'\n", 0o755),
        },
    )
    mcap = write_archive(
        "mcap.tar.gz",
        {
            "mcap-v1.4.0/cpp/mcap/include/mcap/mcap.hpp": (
                b"// locked MCAP fixture\n",
                0o644,
            )
        },
    )
    package = tmp_path / "ecal-package"
    (package / "DEBIAN").mkdir(parents=True)
    (package / "DEBIAN").chmod(0o755)
    (package / "DEBIAN" / "control").write_text(
        "Package: ecal\nVersion: 6.1.1\nArchitecture: amd64\nMaintainer: fixture\nDescription: fixture\n",
        encoding="utf-8",
    )
    config = package / "usr" / "lib" / "cmake" / "eCAL" / "eCALConfig.cmake"
    config.parent.mkdir(parents=True)
    config.write_text("# official eCAL fixture\n", encoding="utf-8")
    ecal_library = package / "usr" / "lib" / "x86_64-linux-gnu" / "libecal_core.so.6"
    ecal_library.parent.mkdir(parents=True)
    ecal_library.write_text("locked ecal runtime\n", encoding="utf-8")
    package_ecal_config = package / "etc" / "ecal" / "ecal.yaml"
    package_ecal_config.parent.mkdir(parents=True)
    package_ecal_config.write_text("# locked eCAL runtime config\n", encoding="utf-8")
    deb = dependencies / "ecal_6.1.1-noble_amd64.deb"
    built_deb = subprocess.run(
        ["dpkg-deb", "--build", str(package), str(deb)], check=False, capture_output=True, text=True
    )
    assert built_deb.returncode == 0, built_deb.stderr
    ecal = {
        "filename": f"dependencies/{deb.name}",
        "sha256": hashlib.sha256(deb.read_bytes()).hexdigest(),
    }
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {"name": "protobuf", "license": "BSD-3-Clause", **protobuf},
                    {"name": "ecal", "license": "Apache-2.0", **ecal},
                    {"name": "mcap", "license": "MIT", **mcap},
                ],
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert (release / "lib" / "libecal_core.so.6").read_text(encoding="utf-8") == "locked ecal runtime\n"
    assert (release / "etc" / "ecal" / "ecal.yaml").read_text(encoding="utf-8") == (
        "# locked eCAL runtime config\n"
    )
    assert (release / "bin" / "slope_sim_stage4_fixture").is_file()
    assert not (release / ".stage4-build").exists()


def test_release_setup_creates_locked_python_runtime_before_building_cpp_tools(tmp_path: Path) -> None:
    """setup 必须只用锁定 micromamba、环境锁和系统公有 CA 在 staging 创建 runtime。"""
    release = tmp_path / "release"
    source = release / "cpp" / "phase0"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text(
        "cmake_minimum_required(VERSION 3.28)\n"
        "project(fixture LANGUAGES CXX)\n"
        "if(NOT EXISTS \"${CMAKE_CURRENT_LIST_DIR}/../../runtime/bin/python\")\n"
        "  message(FATAL_ERROR \"locked Python runtime was not created\")\n"
        "endif()\n"
        "get_filename_component(expected_python \"${CMAKE_CURRENT_LIST_DIR}/../../runtime/bin/python\" ABSOLUTE)\n"
        "if(NOT Python3_EXECUTABLE STREQUAL \"${expected_python}\")\n"
        "  message(FATAL_ERROR \"Phase-0 must use the locked Python runtime\")\n"
        "endif()\n"
        "add_executable(slope_sim_stage4_fixture main.cpp)\n"
        "install(TARGETS slope_sim_stage4_fixture RUNTIME DESTINATION bin)\n",
        encoding="utf-8",
    )
    (source / "main.cpp").write_text("int main() { return 0; }\n", encoding="utf-8")
    lock = release / "packaging" / "locks" / "python-linux-64.lock"
    lock.parent.mkdir(parents=True)
    lock.write_text("@EXPLICIT\nhttps://example.invalid/runtime.conda#deadbeef\n", encoding="utf-8")
    dependencies = release / "dependencies"
    dependencies.mkdir()
    micromamba = dependencies / "micromamba-linux-64"
    micromamba.write_text(
        "#!/bin/sh\n"
        "set -eu\n"
        "prefix=''\n"
        "lock=''\n"
        "verify=''\n"
        "args=''\n"
        "while [ \"$#\" -gt 0 ]; do\n"
        "  case \"$1\" in\n"
        "    --prefix) prefix=\"$2\"; shift 2 ;;\n"
        "    --file) lock=\"$2\"; shift 2 ;;\n"
        "    --ssl-verify) verify=\"$2\"; shift 2 ;;\n"
        "    *) args=\"${args}$1\\n\"; shift ;;\n"
        "  esac\n"
        "done\n"
        "test -n \"$prefix\"\n"
        "test -f \"$lock\"\n"
        "test \"$verify\" = true\n"
        "mkdir -p \"$prefix/bin\"\n"
        "cat > \"$prefix/bin/python3.10\" <<'PY'\n"
        "#!/bin/sh\n"
        "printf '%s\\n' \"$@\" >> \"$(dirname \"$(dirname \"$0\")\")/python-argv.txt\"\n"
        "exit 0\n"
        "PY\n"
        "chmod 755 \"$prefix/bin/python3.10\"\n"
        "ln -s python3.10 \"$prefix/bin/python\"\n"
        "printf '%bfile=%s\\nssl_verify=%s\\n' \"$args\" \"$lock\" \"$verify\" > \"$(dirname \"$prefix\")/micromamba-argv.txt\"\n",
        encoding="utf-8",
    )
    micromamba.chmod(0o755)
    ecal_wheel = dependencies / "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    ecal_wheel.write_bytes(b"locked eCAL Python wheel\n")
    (dependencies / "locked-dependencies.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "dependencies": [
                    {
                        "name": "micromamba",
                        "license": "BSD-3-Clause",
                        "filename": "dependencies/micromamba-linux-64",
                        "sha256": hashlib.sha256(micromamba.read_bytes()).hexdigest(),
                    },
                    {
                        "name": "ecal-python",
                        "license": "Apache-2.0",
                        "filename": f"dependencies/{ecal_wheel.name}",
                        "sha256": hashlib.sha256(ecal_wheel.read_bytes()).hexdigest(),
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(SETUP),
            "--release-root",
            str(release),
            "--dependencies-dir",
            str(dependencies),
            "--with-ros",
            "false",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    runtime_python = release / "runtime" / "bin" / "python"
    assert runtime_python.is_file() and runtime_python.stat().st_mode & 0o111
    argv = (release / "micromamba-argv.txt").read_text(encoding="utf-8")
    assert "create\n" in argv
    assert "--no-rc\n" in argv
    assert "--no-env\n" in argv
    assert "--safety-checks\n" in argv
    assert "enabled\n" in argv
    assert f"file={lock}\n" in argv
    assert "ssl_verify=true\n" in argv
    python_argv = (release / "runtime" / "python-argv.txt").read_text(encoding="utf-8")
    assert "-m\npip\ninstall\n--no-index\n--no-deps\n" in python_argv
    assert f"{ecal_wheel}\n" in python_argv
    assert "-c\nimport ecal.nanobind_core, ecal.msg.proto.core, ecal.msg.common.core\n" in python_argv
    assert not (release / ".stage4-build").exists()


def test_release_setup_installs_locked_protobuf_python_before_ecal_wheel(tmp_path: Path) -> None:
    """Python runtime 必须先离线安装并验证 Protobuf，再安装 eCAL wheel。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_install_locked_python_wheels"])
    install_wheels = getattr(module, "_install_locked_python_wheels", None)
    assert callable(install_wheels), "setup needs an ordered locked Python wheel installer"

    dependencies = tmp_path / "dependencies"
    dependencies.mkdir()
    protobuf_wheel = dependencies / "protobuf-6.33.6-cp39-abi3-manylinux2014_x86_64.whl"
    protobuf_wheel.write_bytes(b"locked protobuf Python wheel\n")
    ecal_wheel = dependencies / "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    ecal_wheel.write_bytes(b"locked eCAL Python wheel\n")
    python = tmp_path / "python"
    python.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$@\" >> \"$(dirname \"$0\")/python-argv.txt\"\n",
        encoding="utf-8",
    )
    python.chmod(0o755)
    locked = [
        {
            "name": "protobuf-python",
            "license": "BSD-3-Clause",
            "filename": f"dependencies/{protobuf_wheel.name}",
            "sha256": hashlib.sha256(protobuf_wheel.read_bytes()).hexdigest(),
        },
        {
            "name": "ecal-python",
            "license": "Apache-2.0",
            "filename": f"dependencies/{ecal_wheel.name}",
            "sha256": hashlib.sha256(ecal_wheel.read_bytes()).hexdigest(),
        },
    ]

    install_wheels(python, dependencies, locked)

    assert (tmp_path / "python-argv.txt").read_text(encoding="utf-8").splitlines() == [
        "-m",
        "pip",
        "install",
        "--no-index",
        "--no-deps",
        str(protobuf_wheel),
        "-c",
        "import google.protobuf",
        "-m",
        "pip",
        "install",
        "--no-index",
        "--no-deps",
        str(ecal_wheel),
        "-c",
        "import ecal.nanobind_core, ecal.msg.proto.core, ecal.msg.common.core",
    ]


def test_release_setup_bounds_locked_protobuf_build_parallelism(tmp_path: Path, monkeypatch) -> None:
    """Protobuf configure/build 必须复用安装级并行度和 ccache 上下文。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_build_locked_protobuf"])
    build_protobuf = getattr(module, "_build_locked_protobuf", None)
    assert callable(build_protobuf), "setup needs the locked Protobuf build helper"

    build_root = tmp_path / "build"
    source = build_root / "sources" / "protobuf"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.28)\n", encoding="utf-8")
    prefix = build_root / "prefix"
    commands: list[list[str]] = []

    def fake_run(
        command: list[str], *, check: bool, env: dict[str, str]
    ) -> subprocess.CompletedProcess[str]:
        assert env["CCACHE_MAXSIZE"] == "5G"
        commands.append(command)
        if command[1] == "--install":
            protoc = prefix / "bin" / "protoc"
            protoc.parent.mkdir(parents=True)
            protoc.write_text("#!/bin/sh\n", encoding="utf-8")
            protoc.chmod(0o755)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    context = module._cmake_build_context(
        tmp_path / "release",
        base_environment={},
        cpu_count=8,
        mem_available_bytes=9 * _GIB,
        build_jobs_override=None,
        ccache="/usr/bin/ccache",
    )
    assert build_protobuf("cmake", build_root, prefix, context) == prefix
    assert ["cmake", "--build", str(build_root / "dependencies" / "protobuf"), "--parallel", "4"] in commands
    assert any(
        "-DCMAKE_CXX_COMPILER_LAUNCHER=/usr/bin/ccache" in command
        for command in commands
    )


def test_release_setup_accepts_locked_versioned_protoc_link(tmp_path: Path, monkeypatch) -> None:
    """Protobuf 的内部相对版本链接应解析为锁定 prefix 内的可执行文件。"""
    module = __import__("scripts.stage4_release_setup", fromlist=["_build_locked_protobuf"])
    build_protobuf = getattr(module, "_build_locked_protobuf", None)
    assert callable(build_protobuf), "setup needs the locked Protobuf build helper"

    build_root = tmp_path / "build"
    source = build_root / "sources" / "protobuf"
    source.mkdir(parents=True)
    (source / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.28)\n", encoding="utf-8")
    prefix = build_root / "prefix"

    def fake_run(command: list[str], *, check: bool) -> subprocess.CompletedProcess[str]:
        if command[1] == "--install":
            binary = prefix / "bin" / "protoc-33.6.0"
            binary.parent.mkdir(parents=True)
            binary.write_text("#!/bin/sh\n", encoding="utf-8")
            binary.chmod(0o755)
            (prefix / "bin" / "protoc").symlink_to(binary.name)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    assert build_protobuf("cmake", build_root, prefix) == prefix
