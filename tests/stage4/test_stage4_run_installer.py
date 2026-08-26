"""阶段四 E：单一 .run 的联网下载、校验、staging 与原子激活验收。"""
from __future__ import annotations

import hashlib
import http.server
import json
import fcntl
import os
from pathlib import Path
import socketserver
import subprocess
import sys
import threading


ROOT = Path(__file__).resolve().parents[2]
BUILDER = ROOT / "scripts" / "build_stage4_run.py"
RELEASE_MANIFEST = ROOT / "packaging" / "stage4-release-manifest.json"


class _Server(socketserver.TCPServer):
    allow_reuse_address = True


def test_formal_release_manifest_locks_the_eight_delivery_dependencies() -> None:
    """正式 manifest 必须声明单机 `.run` 所需的八项锁定下载与 setup 入口。"""
    module = __import__("scripts.build_stage4_run", fromlist=["_manifest"])

    manifest = module._manifest(RELEASE_MANIFEST)

    assert manifest["version"] == "5.0.3"
    assert "with_ros" not in manifest
    assert manifest["payload"] is None
    assert manifest["runtime_setup"] == {"entrypoint": "scripts/stage4_release_setup.py"}
    dependencies = {dependency["name"]: dependency for dependency in manifest["dependencies"]}
    assert set(dependencies) == {
        "micromamba",
        "abseil-cpp",
        "protobuf",
        "protobuf-python",
        "ecal",
        "ecal-python",
        "mcap",
        "livox-viewer2-linux",
    }
    assert dependencies["micromamba"]["sha256"] == "9689782d863c05a1bf5d2d371ba527104e7a4eb4310c1637d8653b751aed9c82"
    assert dependencies["abseil-cpp"] == {
        "name": "abseil-cpp",
        "url": "https://github.com/abseil/abseil-cpp/archive/76bb24329e8bf5f39704eb10d21b9a80befa7c81.tar.gz",
        "sha256": "ed8f7d9f39139c449e79fd19765e23c96fdb774172d32d191323d3e3ea06e5ff",
        "filename": "abseil-cpp-76bb24329e8bf5f39704eb10d21b9a80befa7c81.tar.gz",
        "license": "Apache-2.0",
    }
    assert dependencies["ecal"]["sha256"] == "fe68188ecf48db98ac28e1917261769b2372418de0e1d92a9c74c79596c30e34"
    assert dependencies["ecal-python"]["sha256"] == "57a23af7d83c077c04f01852db13f8cda7686a052d41659fafcbe6b3dbe9f6bc"
    assert dependencies["protobuf"]["sha256"] == "16498d7dc7967e9b100632138babd4b86b61592beeccdd556f67539d9c231355"
    assert dependencies["protobuf-python"]["sha256"] == "e9db7e292e0ab79dd108d7f1a94fe31601ce1ee3f7b79e0692043423020b0593"
    assert dependencies["mcap"]["sha256"] == "64ff3e51119f37ffcfaf9deecbd987a7cb4d4d9035d74a3fd3773395a470fda1"
    assert dependencies["livox-viewer2-linux"]["sha256"] == "3a1e574d3d73ba0b36460c2a358d08f6c722ae0dc376395ba392ec0d533c7e31"


def test_formal_release_python_runtime_has_no_private_channel_dependency() -> None:
    """正式单机安装只能使用公开、已摘要锁定的 Python Protobuf wheel。"""
    module = __import__("scripts.build_stage4_run", fromlist=["_manifest"])
    manifest = module._manifest(RELEASE_MANIFEST)
    dependencies = {dependency["name"]: dependency for dependency in manifest["dependencies"]}

    protobuf_python = dependencies["protobuf-python"]
    assert protobuf_python == {
        "name": "protobuf-python",
        "url": "https://files.pythonhosted.org/packages/16/92/d1e32e3e0d894fe00b15ce28ad4944ab692713f2e7f0a99787405e43533a/protobuf-6.33.6-cp39-abi3-manylinux2014_x86_64.whl",
        "sha256": "e9db7e292e0ab79dd108d7f1a94fe31601ce1ee3f7b79e0692043423020b0593",
        "filename": "protobuf-6.33.6-cp39-abi3-manylinux2014_x86_64.whl",
        "license": "BSD-3-Clause",
    }
    for relative in (
        "packaging/python-environment.yml",
        "packaging/locks/python.conda-lock.yml",
        "packaging/locks/python-linux-64.lock",
    ):
        assert "tail39defd.ts.net" not in (ROOT / relative).read_text(encoding="utf-8")


def test_formal_release_python_runtime_locks_pip_for_embedded_wheels() -> None:
    """正式 runtime 必须锁定 pip，才能离线安装随 `.run` 下载的 Python wheels。"""
    environment = (ROOT / "packaging" / "python-environment.yml").read_text(encoding="utf-8")
    conda_lock = (ROOT / "packaging" / "locks" / "python.conda-lock.yml").read_text(encoding="utf-8")
    explicit_lock = (ROOT / "packaging" / "locks" / "python-linux-64.lock").read_text(encoding="utf-8")

    assert "  - pip\n" in environment
    assert "- name: pip\n" in conda_lock
    assert "/pip-" in explicit_lock


def test_formal_system_lock_includes_the_viewer_sandbox() -> None:
    """Livox Viewer 的隔离启动和导入依赖必须由正式安装器锁定。"""
    document = json.loads(
        (ROOT / "packaging/locks/ubuntu24-system-dependencies.lock").read_text(
            encoding="utf-8"
        )
    )
    packages = {package["name"]: package for package in document["apt_packages"]}
    assert packages["bubblewrap"] == {
        "name": "bubblewrap",
        "version": "0.9.0-1ubuntu0.1",
        "architecture": "amd64",
    }
    assert packages["xdotool"] == {
        "name": "xdotool",
        "version": "1:3.20160805.1-5build1",
        "architecture": "amd64",
    }
    assert packages["libxdo3"] == {
        "name": "libxdo3",
        "version": "1:3.20160805.1-5build1",
        "architecture": "amd64",
    }


def test_run_installer_uses_embedded_project_payload_without_network_payload(tmp_path: Path) -> None:
    """正式 `.run` 的项目 payload 必须内嵌，不得再要求无关的联网下载。"""
    project_payload = tmp_path / "project-payload"
    entrypoint = project_payload / "bin" / "slope-sim-core"
    entrypoint.parent.mkdir(parents=True)
    entrypoint.write_text("#!/bin/sh\necho stage4\n", encoding="utf-8")
    entrypoint.chmod(0o755)
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(
        json.dumps({"schema_version": 1, "version": "4.1.0"}),
        encoding="utf-8",
    )
    installer = tmp_path / "slope-sim-stage4-4.1.0-ubuntu24.04-amd64.run"

    built = subprocess.run(
        [
            sys.executable,
            str(BUILDER),
            "--manifest",
            str(manifest_path),
            "--project-payload",
            str(project_payload),
            "--output",
            str(installer),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert built.returncode == 0, built.stderr
    installed = subprocess.run(
        [str(installer), "--install-root", str(tmp_path / "install")],
        check=False,
        capture_output=True,
        text=True,
    )

    assert installed.returncode == 0, installed.stderr
    release = tmp_path / "install" / "releases" / "4.1.0"
    assert (release / "bin" / "slope-sim-core").read_text(encoding="utf-8") == entrypoint.read_text(
        encoding="utf-8"
    )
    release_manifest = json.loads((release / "manifest.json").read_text(encoding="utf-8"))
    assert len(release_manifest["payload_sha256"]) == 64
    assert "runtime.bin" not in release_manifest["files"]


def test_run_installer_can_publish_a_bare_runsim_command(tmp_path: Path) -> None:
    """显式命令目录必须得到跟随 current 的稳定 runSim 入口。"""
    project_payload = tmp_path / "project-payload"
    launcher = project_payload / "bin" / "runSim"
    launcher.parent.mkdir(parents=True)
    launcher.write_text("#!/bin/sh\necho installed-runsim\n", encoding="utf-8")
    launcher.chmod(0o755)
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(
        json.dumps({"schema_version": 1, "version": "5.0.0"}),
        encoding="utf-8",
    )
    installer = tmp_path / "runSim.run"
    subprocess.run(
        [
            sys.executable,
            str(BUILDER),
            "--manifest",
            str(manifest_path),
            "--project-payload",
            str(project_payload),
            "--output",
            str(installer),
        ],
        check=True,
    )
    install_root = tmp_path / "install"
    command_dir = tmp_path / "bin"

    installed = subprocess.run(
        [
            str(installer),
            "--install-root",
            str(install_root),
            "--command-dir",
            str(command_dir),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert installed.returncode == 0, installed.stderr
    command = command_dir / "runSim"
    assert command.is_symlink()
    assert command.resolve() == install_root / "releases" / "5.0.0" / "bin" / "runSim"
    invoked = subprocess.run(
        [str(command)], check=False, capture_output=True, text=True
    )
    assert invoked.returncode == 0
    assert invoked.stdout == "installed-runsim\n"
    command.unlink()
    command.write_text("do not overwrite\n", encoding="utf-8")
    rejected = subprocess.run(
        [
            str(installer),
            "--install-root",
            str(install_root),
            "--command-dir",
            str(command_dir),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert rejected.returncode == 1
    assert rejected.stderr == "error: runSim command path already exists and is unmanaged\n"
    assert command.read_text(encoding="utf-8") == "do not overwrite\n"


def test_run_installer_downloads_verified_payload_into_versioned_release(tmp_path: Path) -> None:
    """唯一 .run 必须从锁定 HTTP URL 下载、验 SHA 后才切换 current。"""
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    payload = b"stage4-runtime-payload\n"
    (serve_root / "payload.bin").write_bytes(payload)
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest = {
                "schema_version": 1,
                "version": "4.0.0",
                "payload": {
                    "url": f"http://127.0.0.1:{server.server_address[1]}/payload.bin",
                    "sha256": hashlib.sha256(payload).hexdigest(),
                    "filename": "runtime.bin",
                },
            }
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            installer = tmp_path / "slope-sim-stage4-4.0.0-ubuntu24.04-amd64.run"

            built = subprocess.run(
                [sys.executable, str(BUILDER), "--manifest", str(manifest_path), "--output", str(installer)],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            assert installer.is_file() and installer.stat().st_mode & 0o555 == 0o555

            install_root = tmp_path / "install"
            installed = subprocess.run(
                [str(installer), "--install-root", str(install_root)],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            server.shutdown()
            thread.join()

    assert installed.returncode == 0, installed.stderr
    release = install_root / "releases" / "4.0.0"
    assert (release / "runtime.bin").read_bytes() == payload
    assert (install_root / "current").resolve() == release
    release_manifest = json.loads((release / "manifest.json").read_text(encoding="utf-8"))
    assert release_manifest["payload_sha256"] == manifest["payload"]["sha256"]
    assert release_manifest["with_ros"] is False
    assert release_manifest["doctor"] == {"files_verified": True}
    install_state = json.loads((release / "install-state.json").read_text(encoding="utf-8"))
    assert install_state["version"] == "4.0.0"
    assert len(install_state["git_sha"]) == 40
    assert install_state["payload_manifest_sha256"] == hashlib.sha256(
        (release / "manifest.json").read_bytes()
    ).hexdigest()
    assert install_state["with_ros"] is False
    assert install_state["dependencies"] == []
    assert install_state["doctor"] == {"files_verified": True}
    (install_root / "current").unlink()
    repaired = subprocess.run(
        [str(installer), "--install-root", str(install_root)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert repaired.returncode == 0, repaired.stderr
    assert (install_root / "current").resolve() == release
    assert (release / "runtime.bin").read_bytes() == payload
    (release / "runtime.bin").write_bytes(b"tampered")
    rejected = subprocess.run(
        [str(installer), "--install-root", str(install_root)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert rejected.returncode == 1
    assert rejected.stderr == "error: same-version release differs, is damaged, or has option drift\n"
    assert (release / "runtime.bin").read_bytes() == b"tampered"


def test_run_installer_embeds_project_payload_and_default_resources(tmp_path: Path) -> None:
    """唯一 `.run` 必须内嵌项目入口和默认资源，不能把它们当作联网依赖。"""
    project_payload = tmp_path / "project-payload"
    executable = project_payload / "bin" / "slope-sim-core"
    executable.parent.mkdir(parents=True)
    executable.write_text("#!/bin/sh\necho slope-sim-core\n", encoding="utf-8")
    executable.chmod(0o755)
    default_config = project_payload / "share" / "slope-sim" / "default.toml"
    default_config.parent.mkdir(parents=True)
    default_config.write_text("[runtime]\nmode = 'ecal'\n", encoding="utf-8")

    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    runtime = b"stage4-runtime-payload\n"
    (serve_root / "runtime.bin").write_bytes(runtime)
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.7",
                        "payload": {
                            "url": f"http://127.0.0.1:{server.server_address[1]}/runtime.bin",
                            "sha256": hashlib.sha256(runtime).hexdigest(),
                            "filename": "runtime.bin",
                        },
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.7-ubuntu24.04-amd64.run"
            built = subprocess.run(
                [
                    sys.executable,
                    str(BUILDER),
                    "--manifest",
                    str(manifest_path),
                    "--project-payload",
                    str(project_payload),
                    "--output",
                    str(installer),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            installed = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "install")],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            server.shutdown()
            thread.join()

    assert installed.returncode == 0, installed.stderr
    release = tmp_path / "install" / "releases" / "4.0.7"
    assert (release / "bin" / "slope-sim-core").read_text(encoding="utf-8") == executable.read_text(encoding="utf-8")
    assert (release / "bin" / "slope-sim-core").stat().st_mode & 0o111
    assert (release / "share" / "slope-sim" / "default.toml").read_text(encoding="utf-8") == default_config.read_text(encoding="utf-8")
    installed_manifest = json.loads((release / "manifest.json").read_text(encoding="utf-8"))
    assert installed_manifest["files"]["bin/slope-sim-core"] == hashlib.sha256(executable.read_bytes()).hexdigest()
    assert installed_manifest["files"]["share/slope-sim/default.toml"] == hashlib.sha256(default_config.read_bytes()).hexdigest()


def test_run_installer_runs_embedded_runtime_setup_before_publishing(tmp_path: Path) -> None:
    """受锁定 setup 必须在发布前运行，其产物也必须纳入 release 完整性。"""
    project_payload = tmp_path / "project-payload"
    setup = project_payload / "scripts" / "stage4_release_setup.py"
    setup.parent.mkdir(parents=True)
    setup.write_text(
        "from pathlib import Path\n"
        "import argparse\n"
        "parser = argparse.ArgumentParser()\n"
        "parser.add_argument('--release-root', type=Path, required=True)\n"
        "parser.add_argument('--dependencies-dir', type=Path, required=True)\n"
        "parser.add_argument('--with-ros', required=True)\n"
        "args = parser.parse_args()\n"
        "assert (args.release_root / 'runtime.bin').read_bytes() == b'locked-runtime\\n'\n"
        "marker = args.release_root / 'share' / 'slope-sim' / 'runtime-setup.json'\n"
        "marker.parent.mkdir(parents=True)\n"
        "marker.write_text('{\\\"core_ready\\\":true}\\n', encoding='utf-8')\n",
        encoding="utf-8",
    )
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    runtime = b"locked-runtime\n"
    (serve_root / "runtime.bin").write_bytes(runtime)
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.8",
                        "payload": {
                            "url": f"http://127.0.0.1:{server.server_address[1]}/runtime.bin",
                            "sha256": hashlib.sha256(runtime).hexdigest(),
                            "filename": "runtime.bin",
                        },
                        "runtime_setup": {"entrypoint": "scripts/stage4_release_setup.py"},
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.8-ubuntu24.04-amd64.run"
            built = subprocess.run(
                [
                    sys.executable,
                    str(BUILDER),
                    "--manifest",
                    str(manifest_path),
                    "--project-payload",
                    str(project_payload),
                    "--output",
                    str(installer),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            installed = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "install")],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            server.shutdown()
            thread.join()

    assert installed.returncode == 0, installed.stderr
    release = tmp_path / "install" / "releases" / "4.0.8"
    marker = release / "share" / "slope-sim" / "runtime-setup.json"
    assert marker.read_text(encoding="utf-8") == '{"core_ready":true}\n'
    release_manifest = json.loads((release / "manifest.json").read_text(encoding="utf-8"))
    assert release_manifest["files"]["share/slope-sim/runtime-setup.json"] == hashlib.sha256(
        marker.read_bytes()
    ).hexdigest()
    assert (tmp_path / "install" / "current").resolve() == release


def test_run_installer_bootstraps_locked_system_tools_before_runtime_setup(tmp_path: Path) -> None:
    """缺少构建工具时，只有 sudo apt 可越权，setup 仍以安装调用者身份运行。"""
    project_payload = tmp_path / "project-payload"
    setup = project_payload / "scripts" / "stage4_release_setup.py"
    setup.parent.mkdir(parents=True)
    setup.write_text(
        "import os\n"
        "from pathlib import Path\n"
        "import argparse\n"
        "parser = argparse.ArgumentParser()\n"
        "parser.add_argument('--release-root', type=Path, required=True)\n"
        "parser.add_argument('--dependencies-dir', type=Path, required=True)\n"
        "parser.add_argument('--with-ros', required=True)\n"
        "args = parser.parse_args()\n"
        "assert Path(os.environ['STAGE4_SUDO_LOG']).is_file()\n"
        "marker = args.release_root / 'share' / 'slope-sim' / 'runtime-setup.json'\n"
        "marker.parent.mkdir(parents=True)\n"
        "marker.write_text('{\\\"core_ready\\\":true}\\n', encoding='utf-8')\n",
        encoding="utf-8",
    )
    lock = project_payload / "packaging" / "locks" / "ubuntu24-system-dependencies.lock"
    lock.parent.mkdir(parents=True)
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "platform": {"id": "ubuntu", "version_id": "24.04", "architecture": "amd64"},
                "apt_packages": [
                    {"name": "python3", "version": "3.12.3-0ubuntu2.1", "architecture": "amd64"},
                    {"name": "cmake", "version": "3.28.3-1build7", "architecture": "amd64"},
                    {"name": "g++", "version": "4:13.2.0-7ubuntu1", "architecture": "amd64"},
                    {"name": "make", "version": "4.3-4.1build2", "architecture": "amd64"},
                    {"name": "bubblewrap", "version": "0.9.0-1ubuntu0.1", "architecture": "amd64"},
                    {"name": "xdotool", "version": "1:3.20160805.1-5build1", "architecture": "amd64"},
                    {"name": "libxdo3", "version": "1:3.20160805.1-5build1", "architecture": "amd64"},
                    {"name": "libssl-dev", "version": "3.0.13-0ubuntu3.12", "architecture": "amd64"},
                    {"name": "libyaml-cpp-dev", "version": "0.8.0+dfsg-6build1", "architecture": "amd64"},
                ],
            }
        ),
        encoding="utf-8",
    )
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    sudo = fake_bin / "sudo"
    sudo.write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$*\" >> \"$STAGE4_SUDO_LOG\"\n"
        "if [ \"$*\" = 'apt-get install --yes python3' ]; then\n"
        "  printf '#!/bin/sh\\nexec %s \"$@\"\\n' \"$STAGE4_PYTHON3\" > \"$STAGE4_PYTHON3_TARGET\"\n"
        "  /bin/chmod 755 \"$STAGE4_PYTHON3_TARGET\"\n"
        "fi\n",
        encoding="utf-8",
    )
    sudo.chmod(0o755)
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "version": "4.0.18",
                "runtime_setup": {"entrypoint": "scripts/stage4_release_setup.py"},
            }
        ),
        encoding="utf-8",
    )
    installer = tmp_path / "slope-sim-stage4-4.0.18-ubuntu24.04-amd64.run"
    built = subprocess.run(
        [
            sys.executable,
            str(BUILDER),
            "--manifest",
            str(manifest_path),
            "--project-payload",
            str(project_payload),
            "--output",
            str(installer),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert built.returncode == 0, built.stderr
    sudo_log = tmp_path / "sudo.log"
    installed = subprocess.run(
        [str(installer), "--install-root", str(tmp_path / "install")],
        check=False,
        capture_output=True,
        text=True,
        env={
            "PATH": str(fake_bin),
            "STAGE4_SUDO_LOG": str(sudo_log),
            "STAGE4_PYTHON3": str(Path(sys.executable)),
            "STAGE4_PYTHON3_TARGET": str(fake_bin / "python3"),
        },
    )

    assert installed.returncode == 0, installed.stderr
    assert sudo_log.read_text(encoding="utf-8").splitlines() == [
        "apt-get update",
        "apt-get install --yes python3",
        "apt-get update",
        "apt-get install --yes cmake g++ make bubblewrap xdotool libxdo3 libssl-dev libyaml-cpp-dev",
    ]


def test_run_builder_rejects_nonlocal_http_payload_url(tmp_path: Path) -> None:
    """生产 payload 只能使用 HTTPS；HTTP 只限于本地 fixture 验收。"""
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "version": "4.0.0",
                "payload": {
                    "url": "http://example.invalid/runtime.bin",
                    "sha256": "a" * 64,
                    "filename": "runtime.bin",
                },
            }
        ),
        encoding="utf-8",
    )

    built = subprocess.run(
        [sys.executable, str(BUILDER), "--manifest", str(manifest_path), "--output", str(tmp_path / "bad.run")],
        check=False,
        capture_output=True,
        text=True,
    )

    assert built.returncode == 1
    assert built.stderr == "error: release payload is invalid\n"


def test_run_builder_rejects_noncanonical_release_version(tmp_path: Path) -> None:
    """发布版本必须是无前导零的三段 canonical SemVer。"""
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "version": "01.2.3",
                "payload": {
                    "url": "http://127.0.0.1/runtime.bin",
                    "sha256": "a" * 64,
                    "filename": "runtime.bin",
                },
            }
        ),
        encoding="utf-8",
    )

    built = subprocess.run(
        [sys.executable, str(BUILDER), "--manifest", str(manifest_path), "--output", str(tmp_path / "bad.run")],
        check=False,
        capture_output=True,
        text=True,
    )

    assert built.returncode == 1
    assert built.stderr == "error: release version is invalid\n"


def test_run_builder_requires_delivery_filename_to_match_release_version(tmp_path: Path) -> None:
    """唯一 `.run` 文件名必须绑定其内嵌发布版本。"""
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "version": "4.0.6",
                "payload": {
                    "url": "http://127.0.0.1/runtime.bin",
                    "sha256": "a" * 64,
                    "filename": "runtime.bin",
                },
            }
        ),
        encoding="utf-8",
    )

    built = subprocess.run(
        [sys.executable, str(BUILDER), "--manifest", str(manifest_path), "--output", str(tmp_path / "wrong.run")],
        check=False,
        capture_output=True,
        text=True,
    )

    assert built.returncode == 1
    assert built.stderr == "error: output filename does not match release version\n"


def test_with_ros_run_requires_matching_install_option(tmp_path: Path) -> None:
    """with_ros 是同版本身份的一部分，构建和安装选项必须完全一致。"""
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    payload = b"stage4-ros-runtime-payload\n"
    (serve_root / "payload.bin").write_bytes(payload)
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.1",
                        "with_ros": True,
                        "payload": {
                            "url": f"http://127.0.0.1:{server.server_address[1]}/payload.bin",
                            "sha256": hashlib.sha256(payload).hexdigest(),
                            "filename": "runtime.bin",
                        },
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.1-ubuntu24.04-amd64.run"
            built = subprocess.run(
                [sys.executable, str(BUILDER), "--manifest", str(manifest_path), "--output", str(installer)],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            rejected = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "install")],
                check=False,
                capture_output=True,
                text=True,
            )
            installed = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "install"), "--with-ros"],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            server.shutdown()
            thread.join()

    assert rejected.returncode == 1
    assert rejected.stderr == "error: installer with_ros option differs from embedded manifest\n"
    assert installed.returncode == 0, installed.stderr
    release_manifest = json.loads((tmp_path / "install" / "releases" / "4.0.1" / "manifest.json").read_text())
    assert release_manifest["with_ros"] is True


def test_selectable_ros_run_freezes_first_install_choice_and_passes_it_to_setup(tmp_path: Path) -> None:
    """正式单一 `.run` 首装可选择 ROS，选择必须进入 setup 与同版本身份。"""
    project_payload = tmp_path / "project-payload"
    setup = project_payload / "scripts" / "stage4_release_setup.py"
    setup.parent.mkdir(parents=True)
    setup.write_text(
        "from pathlib import Path\n"
        "import argparse\n"
        "parser = argparse.ArgumentParser()\n"
        "parser.add_argument('--release-root', type=Path, required=True)\n"
        "parser.add_argument('--dependencies-dir', type=Path, required=True)\n"
        "parser.add_argument('--with-ros', required=True)\n"
        "args = parser.parse_args()\n"
        "marker = args.release_root / 'share' / 'slope-sim' / 'runtime-setup.json'\n"
        "marker.parent.mkdir(parents=True)\n"
        "marker.write_text(args.with_ros + '\\n', encoding='ascii')\n",
        encoding="utf-8",
    )
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "version": "4.0.11",
                "runtime_setup": {"entrypoint": "scripts/stage4_release_setup.py"},
            }
        ),
        encoding="utf-8",
    )
    installer = tmp_path / "slope-sim-stage4-4.0.11-ubuntu24.04-amd64.run"
    built = subprocess.run(
        [
            sys.executable,
            str(BUILDER),
            "--manifest",
            str(manifest_path),
            "--project-payload",
            str(project_payload),
            "--output",
            str(installer),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert built.returncode == 0, built.stderr

    install_root = tmp_path / "install"
    installed = subprocess.run(
        [str(installer), "--install-root", str(install_root), "--with-ros"],
        check=False,
        capture_output=True,
        text=True,
    )
    rejected = subprocess.run(
        [str(installer), "--install-root", str(install_root)],
        check=False,
        capture_output=True,
        text=True,
    )

    release = install_root / "releases" / "4.0.11"
    assert installed.returncode == 0, installed.stderr
    assert (release / "share" / "slope-sim" / "runtime-setup.json").read_text(encoding="ascii") == "true\n"
    assert json.loads((release / "manifest.json").read_text(encoding="utf-8"))["with_ros"] is True
    assert json.loads((release / "install-state.json").read_text(encoding="utf-8"))["with_ros"] is True
    assert rejected.returncode == 1
    assert rejected.stderr == "error: same-version release differs, is damaged, or has option drift\n"


def test_selectable_ros_run_downloads_ros_dependencies_only_for_ros_install(tmp_path: Path) -> None:
    """可选 ROS 下载必须随首次选择进入身份，核心安装不得下载它。"""
    project_payload = tmp_path / "project-payload"
    project_payload.mkdir()
    (project_payload / "README.txt").write_text("stage4 fixture\n", encoding="ascii")
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    ros_bootstrap = b"locked-ros-bootstrap\n"
    (serve_root / "ros-bootstrap.deb").write_bytes(ros_bootstrap)
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.12",
                        "ros_dependencies": [
                            {
                                "name": "ros-bootstrap",
                                "url": f"http://127.0.0.1:{server.server_address[1]}/ros-bootstrap.deb",
                                "sha256": hashlib.sha256(ros_bootstrap).hexdigest(),
                                "filename": "ros-bootstrap.deb",
                                "license": "Apache-2.0",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.12-ubuntu24.04-amd64.run"
            built = subprocess.run(
                [
                    sys.executable,
                    str(BUILDER),
                    "--manifest",
                    str(manifest_path),
                    "--project-payload",
                    str(project_payload),
                    "--output",
                    str(installer),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            core = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "core-install")],
                check=False,
                capture_output=True,
                text=True,
            )
            ros = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "ros-install"), "--with-ros"],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            server.shutdown()
            thread.join()

    core_release = tmp_path / "core-install" / "releases" / "4.0.12"
    ros_release = tmp_path / "ros-install" / "releases" / "4.0.12"
    assert core.returncode == 0, core.stderr
    assert not (core_release / "dependencies" / "ros-bootstrap.deb").exists()
    assert json.loads((core_release / "manifest.json").read_text(encoding="utf-8"))["dependencies"] == []
    assert ros.returncode == 0, ros.stderr
    assert (ros_release / "dependencies" / "ros-bootstrap.deb").read_bytes() == ros_bootstrap
    assert json.loads((ros_release / "manifest.json").read_text(encoding="utf-8"))["dependencies"] == [
        {
            "name": "ros-bootstrap",
            "license": "Apache-2.0",
            "filename": "dependencies/ros-bootstrap.deb",
            "sha256": hashlib.sha256(ros_bootstrap).hexdigest(),
        }
    ]


def test_selectable_ros_run_bootstraps_only_locked_ros_apt_packages(tmp_path: Path) -> None:
    """ROS 安装只对已校验 source deb 和严格版本化 Jazzy 包调用 sudo。"""
    project_payload = tmp_path / "project-payload"
    lock = project_payload / "packaging" / "locks" / "ros2-apt-packages.lock"
    lock.parent.mkdir(parents=True)
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "platform": {"id": "ubuntu", "version_id": "24.04", "architecture": "amd64"},
                "packages": [
                    {"name": "ros-jazzy-rclcpp", "version": "28.1.21-1noble.20260615.133124", "architecture": "amd64"}
                ],
            }
        ),
        encoding="utf-8",
    )
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    source_deb = b"locked-ros-apt-source\n"
    (serve_root / "ros2-apt-source.deb").write_bytes(source_deb)
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    (fake_bin / "sudo").write_text(
        "#!/bin/sh\n"
        "printf '%s\\n' \"$*\" >> \"$STAGE4_SUDO_LOG\"\n",
        encoding="ascii",
    )
    (fake_bin / "dpkg-query").write_text("#!/bin/sh\nexit 1\n", encoding="ascii")
    (fake_bin / "sudo").chmod(0o755)
    (fake_bin / "dpkg-query").chmod(0o755)
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.13",
                        "ros_dependencies": [
                            {
                                "name": "ros2-apt-source",
                                "url": f"http://127.0.0.1:{server.server_address[1]}/ros2-apt-source.deb",
                                "sha256": hashlib.sha256(source_deb).hexdigest(),
                                "filename": "ros2-apt-source.deb",
                                "license": "Apache-2.0",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.13-ubuntu24.04-amd64.run"
            built = subprocess.run(
                [
                    sys.executable,
                    str(BUILDER),
                    "--manifest",
                    str(manifest_path),
                    "--project-payload",
                    str(project_payload),
                    "--output",
                    str(installer),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            core_log = tmp_path / "core-sudo.log"
            core = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "core-install")],
                check=False,
                capture_output=True,
                text=True,
                env={**os.environ, "PATH": str(fake_bin) + os.pathsep + os.environ["PATH"], "STAGE4_SUDO_LOG": str(core_log)},
            )
            ros_log = tmp_path / "ros-sudo.log"
            ros = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "ros-install"), "--with-ros"],
                check=False,
                capture_output=True,
                text=True,
                env={**os.environ, "PATH": str(fake_bin) + os.pathsep + os.environ["PATH"], "STAGE4_SUDO_LOG": str(ros_log)},
            )
        finally:
            server.shutdown()
            thread.join()

    assert core.returncode == 0, core.stderr
    assert not core_log.exists()
    assert ros.returncode == 0, ros.stderr
    commands = ros_log.read_text(encoding="utf-8").splitlines()
    assert commands[0].startswith("dpkg --install ")
    assert commands[0].endswith("/dependencies/ros2-apt-source.deb")
    assert commands[1:] == [
        "apt-get update",
        "apt-get install --yes ros-jazzy-rclcpp=28.1.21-1noble.20260615.133124",
    ]


def test_selectable_ros_run_skips_sudo_when_locked_packages_are_already_installed(tmp_path: Path) -> None:
    """精确已安装的 Jazzy 包不得被重复 bootstrap，也不应要求新的 sudo ticket。"""
    project_payload = tmp_path / "project-payload"
    lock = project_payload / "packaging" / "locks" / "ros2-apt-packages.lock"
    lock.parent.mkdir(parents=True)
    lock.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "platform": {"id": "ubuntu", "version_id": "24.04", "architecture": "amd64"},
                "packages": [
                    {"name": "ros-jazzy-rclcpp", "version": "28.1.21-1noble.20260615.133124", "architecture": "amd64"}
                ],
            }
        ),
        encoding="utf-8",
    )
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    source_deb = b"locked-ros-apt-source\n"
    (serve_root / "ros2-apt-source.deb").write_bytes(source_deb)
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    (fake_bin / "dpkg-query").write_text(
        "#!/bin/sh\n"
        "test \"$1\" = '-W' && test \"$3\" = 'ros-jazzy-rclcpp' || exit 64\n"
        "printf '%s\\n' 'install ok installed 28.1.21-1noble.20260615.133124 amd64'\n",
        encoding="ascii",
    )
    (fake_bin / "sudo").write_text("#!/bin/sh\nexit 99\n", encoding="ascii")
    (fake_bin / "dpkg-query").chmod(0o755)
    (fake_bin / "sudo").chmod(0o755)
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.14",
                        "ros_dependencies": [
                            {
                                "name": "ros2-apt-source",
                                "url": f"http://127.0.0.1:{server.server_address[1]}/ros2-apt-source.deb",
                                "sha256": hashlib.sha256(source_deb).hexdigest(),
                                "filename": "ros2-apt-source.deb",
                                "license": "Apache-2.0",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.14-ubuntu24.04-amd64.run"
            built = subprocess.run(
                [
                    sys.executable,
                    str(BUILDER),
                    "--manifest",
                    str(manifest_path),
                    "--project-payload",
                    str(project_payload),
                    "--output",
                    str(installer),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            installed = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "install"), "--with-ros"],
                check=False,
                capture_output=True,
                text=True,
                env={**os.environ, "PATH": str(fake_bin) + os.pathsep + os.environ["PATH"]},
            )
        finally:
            server.shutdown()
            thread.join()

    assert installed.returncode == 0, installed.stderr


def test_run_installer_refuses_when_global_install_lock_is_held(tmp_path: Path) -> None:
    """全局锁被占用时必须在网络和 staging 前失败，不能并发覆盖 current。"""
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    payload = b"stage4-lock-payload\n"
    (serve_root / "payload.bin").write_bytes(payload)
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.2",
                        "payload": {
                            "url": f"http://127.0.0.1:{server.server_address[1]}/payload.bin",
                            "sha256": hashlib.sha256(payload).hexdigest(),
                            "filename": "runtime.bin",
                        },
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.2-ubuntu24.04-amd64.run"
            assert subprocess.run(
                [sys.executable, str(BUILDER), "--manifest", str(manifest_path), "--output", str(installer)],
                check=False,
                capture_output=True,
                text=True,
            ).returncode == 0
            install_root = tmp_path / "install"
            install_root.mkdir()
            with (install_root / ".stage4-install.lock").open("w") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                blocked = subprocess.run(
                    [str(installer), "--install-root", str(install_root)],
                    check=False,
                    capture_output=True,
                    text=True,
                )
        finally:
            server.shutdown()
            thread.join()

    assert blocked.returncode == 1
    assert blocked.stderr == "error: another stage4 installation is active\n"
    assert not (install_root / "releases" / "4.0.2").exists()


def test_failed_download_hash_does_not_switch_existing_current(tmp_path: Path) -> None:
    """SHA 不符必须清理 staging，并保持已经激活的旧版本不变。"""
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    (serve_root / "payload.bin").write_bytes(b"wrong-content")
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.3",
                        "payload": {
                            "url": f"http://127.0.0.1:{server.server_address[1]}/payload.bin",
                            "sha256": "0" * 64,
                            "filename": "runtime.bin",
                        },
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.3-ubuntu24.04-amd64.run"
            assert subprocess.run(
                [sys.executable, str(BUILDER), "--manifest", str(manifest_path), "--output", str(installer)],
                check=False,
                capture_output=True,
                text=True,
            ).returncode == 0
            install_root = tmp_path / "install"
            old_release = install_root / "releases" / "3.9.0"
            old_release.mkdir(parents=True)
            (install_root / "current").symlink_to(old_release)
            failed = subprocess.run(
                [str(installer), "--install-root", str(install_root)],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            server.shutdown()
            thread.join()

    assert failed.returncode == 1
    assert failed.stderr == "error: downloaded payload SHA-256 differs from manifest\n"
    assert (install_root / "current").resolve() == old_release
    assert not (install_root / "releases" / "4.0.3").exists()
    assert not any((install_root / ".staging").iterdir())


def test_run_installer_downloads_each_locked_dependency_with_license(tmp_path: Path) -> None:
    """安装器必须下载并验证 manifest 中的每个锁定依赖及其许可证声明。"""
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    payload = b"stage4-runtime-payload\n"
    dependency = b"stage4-dependency-payload\n"
    (serve_root / "payload.bin").write_bytes(payload)
    (serve_root / "dependency.bin").write_bytes(dependency)
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest_path = tmp_path / "manifest.json"
            base_url = f"http://127.0.0.1:{server.server_address[1]}"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.4",
                        "payload": {
                            "url": f"{base_url}/payload.bin",
                            "sha256": hashlib.sha256(payload).hexdigest(),
                            "filename": "runtime.bin",
                        },
                        "dependencies": [
                            {
                                "name": "ecal",
                                "url": f"{base_url}/dependency.bin",
                                "sha256": hashlib.sha256(dependency).hexdigest(),
                                "filename": "ecal.bin",
                                "license": "Apache-2.0",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.4-ubuntu24.04-amd64.run"
            built = subprocess.run(
                [sys.executable, str(BUILDER), "--manifest", str(manifest_path), "--output", str(installer)],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            installed = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "install")],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            server.shutdown()
            thread.join()

    assert installed.returncode == 0, installed.stderr
    release = tmp_path / "install" / "releases" / "4.0.4"
    assert (release / "runtime.bin").read_bytes() == payload
    assert (release / "dependencies" / "ecal.bin").read_bytes() == dependency
    installed_manifest = json.loads((release / "manifest.json").read_text(encoding="utf-8"))
    assert installed_manifest["dependencies"] == [
        {"name": "ecal", "license": "Apache-2.0", "filename": "dependencies/ecal.bin", "sha256": hashlib.sha256(dependency).hexdigest()}
    ]
    assert json.loads((release / "dependencies" / "locked-dependencies.json").read_text(encoding="utf-8")) == {
        "schema_version": 1,
        "dependencies": installed_manifest["dependencies"],
    }


def test_run_installer_marks_locked_micromamba_executable(tmp_path: Path) -> None:
    """正式 setup 依赖的锁定 micromamba 下载后必须保留可执行权限。"""
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    micromamba = b"#!/bin/sh\nexit 0\n"
    (serve_root / "micromamba-linux-64").write_bytes(micromamba)
    handler = lambda *args, **kwargs: http.server.SimpleHTTPRequestHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            manifest_path = tmp_path / "manifest.json"
            base_url = f"http://127.0.0.1:{server.server_address[1]}"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.10",
                        "dependencies": [
                            {
                                "name": "micromamba",
                                "url": f"{base_url}/micromamba-linux-64",
                                "sha256": hashlib.sha256(micromamba).hexdigest(),
                                "filename": "micromamba-linux-64",
                                "license": "BSD-3-Clause",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.10-ubuntu24.04-amd64.run"
            project_payload = tmp_path / "project-payload"
            project_payload.mkdir()
            (project_payload / "entry.py").write_text("# fixture\n", encoding="utf-8")
            built = subprocess.run(
                [
                    sys.executable,
                    str(BUILDER),
                    "--manifest",
                    str(manifest_path),
                    "--output",
                    str(installer),
                    "--project-payload",
                    str(project_payload),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            installed = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "install")],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            server.shutdown()
            thread.join()

    assert installed.returncode == 0, installed.stderr
    target = tmp_path / "install" / "releases" / "4.0.10" / "dependencies" / "micromamba-linux-64"
    assert target.is_file() and target.stat().st_mode & 0o111


def test_run_installer_retries_a_transient_locked_dependency_download(tmp_path: Path) -> None:
    """临时传输错误可有限重试，成功文件仍必须按锁定 SHA 发布。"""
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    payload = b"stage4-runtime-payload\n"
    dependency = b"stage4-dependency-payload\n"
    (serve_root / "payload.bin").write_bytes(payload)
    (serve_root / "dependency.bin").write_bytes(dependency)
    dependency_requests = {"count": 0}

    class _TransientDependencyHandler(http.server.SimpleHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - HTTP handler API uses this spelling.
            if self.path == "/dependency.bin" and dependency_requests["count"] == 0:
                dependency_requests["count"] += 1
                self.send_response(503)
                self.end_headers()
                return
            if self.path == "/dependency.bin":
                dependency_requests["count"] += 1
            super().do_GET()

    handler = lambda *args, **kwargs: _TransientDependencyHandler(
        *args, directory=str(serve_root), **kwargs
    )
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base_url = f"http://127.0.0.1:{server.server_address[1]}"
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.5",
                        "payload": {
                            "url": f"{base_url}/payload.bin",
                            "sha256": hashlib.sha256(payload).hexdigest(),
                            "filename": "runtime.bin",
                        },
                        "dependencies": [
                            {
                                "name": "ecal",
                                "url": f"{base_url}/dependency.bin",
                                "sha256": hashlib.sha256(dependency).hexdigest(),
                                "filename": "ecal.bin",
                                "license": "Apache-2.0",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.5-ubuntu24.04-amd64.run"
            built = subprocess.run(
                [sys.executable, str(BUILDER), "--manifest", str(manifest_path), "--output", str(installer)],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            installed = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "install")],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            server.shutdown()
            thread.join()

    assert installed.returncode == 0, installed.stderr
    assert dependency_requests["count"] == 2
    assert (tmp_path / "install" / "releases" / "4.0.5" / "dependencies" / "ecal.bin").read_bytes() == dependency


def test_run_installer_retries_a_truncated_locked_dependency_download(tmp_path: Path) -> None:
    """截断的 HTTPS body 也属于可重试传输错误，成功后仍必须校验锁定 SHA。"""
    serve_root = tmp_path / "serve"
    serve_root.mkdir()
    dependency = b"stage4-dependency-payload\n"
    (serve_root / "dependency.bin").write_bytes(dependency)
    dependency_requests = {"count": 0}

    class _TruncatedDependencyHandler(http.server.SimpleHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802 - HTTP handler API uses this spelling.
            if self.path == "/dependency.bin" and dependency_requests["count"] == 0:
                dependency_requests["count"] += 1
                self.send_response(200)
                self.send_header("Transfer-Encoding", "chunked")
                self.end_headers()
                self.wfile.write(f"{len(dependency):X}\\r\\n".encode() + dependency[:1])
                self.wfile.flush()
                self.close_connection = True
                return
            if self.path == "/dependency.bin":
                dependency_requests["count"] += 1
            super().do_GET()

    handler = lambda *args, **kwargs: _TruncatedDependencyHandler(*args, directory=str(serve_root), **kwargs)
    with _Server(("127.0.0.1", 0), handler) as server:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base_url = f"http://127.0.0.1:{server.server_address[1]}"
            manifest_path = tmp_path / "manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": 1,
                        "version": "4.0.11",
                        "dependencies": [
                            {
                                "name": "ecal",
                                "url": f"{base_url}/dependency.bin",
                                "sha256": hashlib.sha256(dependency).hexdigest(),
                                "filename": "ecal.bin",
                                "license": "Apache-2.0",
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            installer = tmp_path / "slope-sim-stage4-4.0.11-ubuntu24.04-amd64.run"
            project_payload = tmp_path / "project-payload"
            project_payload.mkdir()
            (project_payload / "entry.py").write_text("# fixture\n", encoding="utf-8")
            built = subprocess.run(
                [
                    sys.executable,
                    str(BUILDER),
                    "--manifest",
                    str(manifest_path),
                    "--project-payload",
                    str(project_payload),
                    "--output",
                    str(installer),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            assert built.returncode == 0, built.stderr
            installed = subprocess.run(
                [str(installer), "--install-root", str(tmp_path / "install")],
                check=False,
                capture_output=True,
                text=True,
            )
        finally:
            server.shutdown()
            thread.join()

    assert installed.returncode == 0, installed.stderr
    assert dependency_requests["count"] == 2
    assert (tmp_path / "install" / "releases" / "4.0.11" / "dependencies" / "ecal.bin").read_bytes() == dependency
