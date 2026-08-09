# 阶段四网络隔离合同：构建入口必须在可复核的独立 netns 内运行。
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
WRAPPER = ROOT / "packaging" / "run_network_isolated.sh"
VERIFIER = ROOT / "scripts" / "verify_network_isolation.py"
PYTHON_RUNTIME_BUILDER = ROOT / "packaging" / "build_python_runtime.sh"


def _write_python_runtime_builder_fixture(source: Path, micromamba_record: Path) -> Path:
    """构造最小离线 builder 输入，供 shell 编排测试记录真实 argv 顺序。"""
    (source / "packaging" / "locks").mkdir(parents=True)
    (source / "scripts").mkdir()
    for relative_path in (
        "pyproject.toml",
        "packaging/python-environment.yml",
        "packaging/python-toolchain-environment.yml",
        "packaging/python-protobuf-build-environment.yml",
        "packaging/locks/virtual-packages.yml",
        "packaging/locks/python.conda-lock.yml",
        "packaging/locks/python-linux-64.lock",
        "packaging/locks/python-toolchain.conda-lock.yml",
        "packaging/locks/python-toolchain-linux-64.lock",
        "packaging/locks/python-protobuf-build.conda-lock.yml",
        "packaging/locks/python-protobuf-build-linux-64.lock",
        "packaging/locks/python-package-cache.manifest.json",
    ):
        (source / relative_path).write_text("fixture\n", encoding="utf-8")
    (source / "pyproject.toml").write_text(
        "[build-system]\n"
        "requires = ['setuptools>=68', 'wheel']\n"
        "build-backend = 'setuptools.build_meta'\n\n"
        "[project]\nname = 'fixture-project'\nversion = '0.1.0'\n",
        encoding="utf-8",
    )
    (source / "packaging" / "locks" / "python-wheel-cache.manifest.json").write_text(
        (ROOT / "packaging" / "locks" / "python-wheel-cache.manifest.json").read_text(
            encoding="utf-8"
        ),
        encoding="utf-8",
    )
    verifier = "from __future__ import annotations\nprint('PASS: fixture verifier')\n"
    for name in (
        "freeze_python_lock_cache.py",
        "verify_python_lock_cache.py",
        "verify_python_wheel_cache.py",
    ):
        (source / "scripts" / name).write_text(verifier, encoding="utf-8")
    frozen_wheel = (
        ROOT
        / "build"
        / "stage4-python-wheel-cache-20260805T172013+0800"
        / "wheels"
        / "57a23af7d83c077c04f01852db13f8cda7686a052d41659fafcbe6b3dbe9f6bc"
        / "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    )
    assert frozen_wheel.is_file(), "frozen eCAL wheel fixture is unavailable"
    materializer = f"""from __future__ import annotations
import argparse
from pathlib import Path
from shutil import copyfile

parser = argparse.ArgumentParser()
parser.add_argument('--manifest', required=True)
parser.add_argument('--canonical-cache', required=True)
parser.add_argument('--destination', required=True)
parser.add_argument('--evidence', required=True)
args = parser.parse_args()
destination = Path(args.destination)
destination.mkdir(parents=True)
filename = 'eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl'
copyfile(Path({str(frozen_wheel)!r}), destination / filename)
Path(args.evidence).write_text(
    '{{"path": "' + str((destination / filename).resolve()) + '"}}\\n',
    encoding='utf-8',
)
"""
    for name in (
        "materialize_python_package_cache.py",
        "materialize_python_wheel_cache.py",
    ):
        (source / "scripts" / name).write_text(materializer, encoding="utf-8")
    micromamba = source / "pinned-micromamba"
    micromamba.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        f"printf '%s\\n' \"$@\" > {shlex.quote(str(micromamba_record))}\n"
        "printf 'FAKE_MICROMAMBA\n' >&2\n"
        "exit 23\n",
        encoding="utf-8",
    )
    micromamba.chmod(0o755)
    return micromamba


def _write_two_call_micromamba(path: Path, record: Path) -> None:
    """模拟已成功的 toolchain create，并在 runtime create 处记录固定 argv 后停止。"""
    path.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        f"record={shlex.quote(str(record))}\n"
        "count_file=\"${record}.count\"\n"
        "count=0\n"
        "[[ -f \"$count_file\" ]] && count=$(cat \"$count_file\")\n"
        "printf '%s\\n' \"$@\" > \"${record}.${count}\"\n"
        "printf '%s' $((count + 1)) > \"$count_file\"\n"
        "if [[ \"$count\" == 0 ]]; then\n"
        "  exit 0\n"
        "fi\n"
        "printf 'FAKE_RUNTIME_CREATE\\n' >&2\n"
        "exit 24\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def _write_packing_micromamba(path: Path, conda_pack_record: Path) -> None:
    """模拟两个成功的 create，并在 conda-pack 调用处停止以锁定其输入顺序。"""
    conda_pack = (
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        f"printf '%s\\n' \"$@\" > {shlex.quote(str(conda_pack_record))}\n"
        "printf 'FAKE_CONDA_PACK\\n' >&2\n"
        "exit 25\n"
    )
    path.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "prefix=''\n"
        "while [[ \"$#\" -gt 0 ]]; do\n"
        "  if [[ \"$1\" == --prefix ]]; then prefix=$2; break; fi\n"
        "  shift\n"
        "done\n"
        "if [[ \"$prefix\" == */tool-env ]]; then\n"
        "  mkdir -p \"$prefix/bin\"\n"
        f"  cat > \"$prefix/bin/conda-pack\" <<'EOF'\n{conda_pack}EOF\n"
        "  chmod 755 \"$prefix/bin/conda-pack\"\n"
        "fi\n"
        "exit 0\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def _write_staging_micromamba(path: Path) -> None:
    """模拟 conda-pack 成功并生成最小纯 Conda tar，供 staging 边界测试使用。"""
    conda_pack = """#!/usr/bin/env bash
set -euo pipefail
output=''
while [[ "$#" -gt 0 ]]; do
  if [[ "$1" == --output ]]; then output=$2; break; fi
  shift
done
payload=$(mktemp -d)
mkdir -p "$payload/bin" "$payload/conda-meta"
printf 'python' > "$payload/bin/python"
cat > "$payload/bin/conda-unpack" <<'STAGE4_CONDA_UNPACK'
#!/bin/sh
printf '%s\\n' "$0" > "$(dirname "$0")/conda-unpack-ran.txt"
STAGE4_CONDA_UNPACK
chmod 755 "$payload/bin/conda-unpack"
printf 'history' > "$payload/conda-meta/history"
mkdir -p "$(dirname "$output")"
tar -cf "$output" -C "$payload" .
"""
    path.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "prefix=''\n"
        "while [[ \"$#\" -gt 0 ]]; do\n"
        "  if [[ \"$1\" == --prefix ]]; then prefix=$2; break; fi\n"
        "  shift\n"
        "done\n"
        "if [[ \"$prefix\" == */tool-env ]]; then\n"
        "  mkdir -p \"$prefix/bin\"\n"
        f"  cat > \"$prefix/bin/conda-pack\" <<'EOF'\n{conda_pack}EOF\n"
        "  chmod 755 \"$prefix/bin/conda-pack\"\n"
        "fi\n"
        "exit 0\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def _write_wheel_install_micromamba(
    path: Path,
    record: Path,
    *,
    fail_project_pip: bool = True,
    create_project_console_script: bool = False,
) -> None:
    """模拟 tool env 的 build/pip，锁定 pack 后 wheel 构建与两次安装的真实 argv。"""
    conda_pack = """#!/usr/bin/env bash
set -euo pipefail
output=''
while [[ "$#" -gt 0 ]]; do
  if [[ "$1" == --output ]]; then output=$2; break; fi
  shift
done
payload=$(mktemp -d)
mkdir -p "$payload/bin" "$payload/conda-meta"
printf 'python' > "$payload/bin/python"
cat > "$payload/bin/conda-unpack" <<'STAGE4_CONDA_UNPACK'
#!/bin/sh
printf '%s\\n' "$0" > "$(dirname "$0")/conda-unpack-ran.txt"
STAGE4_CONDA_UNPACK
chmod 755 "$payload/bin/conda-unpack"
printf 'history' > "$payload/conda-meta/history"
mkdir -p "$(dirname "$output")"
tar -cf "$output" -C "$payload" .
"""
    fake_python = f"""#!/usr/bin/env bash
set -euo pipefail
record={shlex.quote(str(record))}
count_file="${{record}}.count"
count=0
[[ -f "$count_file" ]] && count=$(cat "$count_file")
printf '%s\\n' "$@" > "${{record}}.${{count}}"
printf '%s' $((count + 1)) > "$count_file"
if [[ "$1" == -m && "$2" == build ]]; then
  outdir=''
  while [[ "$#" -gt 0 ]]; do
    if [[ "$1" == --outdir ]]; then outdir=$2; break; fi
    shift
  done
  mkdir -p "$outdir"
  printf 'fixture project wheel' > "$outdir/fixture_project-0.1.0-py3-none-any.whl"
  exit 0
fi
if [[ "$1" == -m && "$2" == pip && "$3" == install ]]; then
  wheel_path="${{!#}}"
  prefix=''
  while [[ "$#" -gt 0 ]]; do
    if [[ "$1" == --prefix ]]; then prefix=$2; break; fi
    shift
  done
  if [[ "$count" == 1 ]]; then
    dist_info='eclipse_ecal-6.1.1.dist-info'
  else
    dist_info='fixture_project-0.1.0.dist-info'
  fi
  site_packages="$prefix/lib/python3.10/site-packages"
  mkdir -p "$site_packages"
  if [[ "$count" == 1 ]]; then
    python3 - "$wheel_path" "$site_packages" <<'PY'
from pathlib import Path
import sys
import zipfile

with zipfile.ZipFile(Path(sys.argv[1])) as archive:
    archive.extractall(Path(sys.argv[2]))
PY
  else
    mkdir -p "$site_packages/$dist_info"
  fi
  mkdir -p "$prefix/lib/python3.10/site-packages/fixture_cache/__pycache__"
  printf 'bytecode' > "$prefix/lib/python3.10/site-packages/fixture_cache/__pycache__/module.cpython-310.pyc"
  direct_url="$site_packages/$dist_info/direct_url.json"
  printf '{{"url": "file://%s"}}\\n' "$wheel_path" > "$direct_url"
  python3 - "$site_packages/$dist_info/RECORD" "$direct_url" "$dist_info" <<'PY'
import base64
import csv
import hashlib
from pathlib import Path
import sys

record, direct_url, dist_info = map(Path, sys.argv[1:])
rows = list(csv.reader(record.read_text(encoding='utf-8').splitlines())) if record.exists() else []
payload = direct_url.read_bytes()
digest = base64.urlsafe_b64encode(hashlib.sha256(payload).digest()).rstrip(b'=').decode('ascii')
rows.append((f'{{dist_info}}/direct_url.json', f'sha256={{digest}}', str(len(payload))))
with record.open('w', encoding='utf-8', newline='') as stream:
    csv.writer(stream, lineterminator='\\n').writerows(rows)
PY
  if [[ "$count" == 2 && {str(create_project_console_script).lower()} == true ]]; then
    entry_points="$site_packages/$dist_info/entry_points.txt"
    printf '[console_scripts]\nfixture-command = fixture.module:main\n' > "$entry_points"
    mkdir -p "$prefix/bin"
    script_path="$prefix/bin/fixture-command"
    printf '#!/bin/sh\n' > "$script_path"
    chmod 755 "$script_path"
    python3 - "$site_packages/$dist_info/RECORD" "$entry_points" "$script_path" "$dist_info" <<'PY'
import base64
import csv
import hashlib
from pathlib import Path
import sys

record, entry_points, script_path, dist_info = map(Path, sys.argv[1:])
rows = list(csv.reader(record.read_text(encoding='utf-8').splitlines()))
for path, relative in (
    (entry_points, f'{{dist_info}}/entry_points.txt'),
    (script_path, '../../../bin/fixture-command'),
):
    payload = path.read_bytes()
    digest = base64.urlsafe_b64encode(hashlib.sha256(payload).digest()).rstrip(b'=').decode('ascii')
    rows.insert(-1, (relative, f'sha256={{digest}}', str(len(payload))))
with record.open('w', encoding='utf-8', newline='') as stream:
    csv.writer(stream, lineterminator='\\n').writerows(rows)
PY
  fi
  if [[ "$count" == 2 && {str(fail_project_pip).lower()} == true ]]; then
    printf 'FAKE_PROJECT_PIP\\n' >&2
    exit 26
  fi
fi
exit 0
"""
    path.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "prefix=''\n"
        "while [[ \"$#\" -gt 0 ]]; do\n"
        "  if [[ \"$1\" == --prefix ]]; then prefix=$2; break; fi\n"
        "  shift\n"
        "done\n"
        "if [[ \"$prefix\" == */tool-env ]]; then\n"
        "  mkdir -p \"$prefix/bin\"\n"
        f"  cat > \"$prefix/bin/conda-pack\" <<'EOF'\n{conda_pack}EOF\n"
        f"  cat > \"$prefix/bin/python\" <<'EOF'\n{fake_python}EOF\n"
        "  chmod 755 \"$prefix/bin/conda-pack\" \"$prefix/bin/python\"\n"
        "fi\n"
        "exit 0\n",
        encoding="utf-8",
    )
    path.chmod(0o755)


def test_network_wrapper_creates_verified_loopback_only_namespace(tmp_path) -> None:
    """wrapper 必须在执行命令前建立无默认路由的独立 netns 并保存证据。"""
    assert WRAPPER.is_file(), "stage 4 network isolation wrapper is not implemented"
    evidence = tmp_path / "network-evidence"
    command = ["bash", "-c", "printf '%s' \"$STAGE4_NETWORK_ISOLATION_EVIDENCE\""]
    completed = subprocess.run(
        [str(WRAPPER), "--evidence-dir", str(evidence), "--", *command],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == str(evidence.resolve())
    document = json.loads((evidence / "network-isolation.json").read_text(encoding="utf-8"))
    assert document["parent_pid"] > 0
    assert document["child_pid"] > 0
    assert document["parent_netns_inode"] != document["child_netns_inode"]
    assert document["argv_sha256"] == hashlib.sha256(
        b"bash\0-c\0printf '%s' \"$STAGE4_NETWORK_ISOLATION_EVIDENCE\"\0"
    ).hexdigest()
    assert [link["ifname"] for link in document["links"]] == ["lo"]
    assert document["ipv4_default_routes"] == []
    assert document["ipv6_default_routes"] == []
    assert document["test_net_connect_errno"] == "ENETUNREACH"
    assert document["loopback_socket"] is True


def test_network_isolation_verifier_rechecks_live_child_namespace(tmp_path) -> None:
    """构建 child 必须独立复核 evidence、父进程和自己的实时 netns 状态。"""
    assert VERIFIER.is_file(), "stage 4 network isolation verifier is not implemented"
    evidence = tmp_path / "network-evidence"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            "bash",
            "-c",
            'exec "$1" "$2" --evidence "$STAGE4_NETWORK_ISOLATION_EVIDENCE"',
            "bash",
            sys.executable,
            str(VERIFIER),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == "PASS: live network isolation verified\n"


def test_network_isolation_verifier_rechecks_explicit_builder_pid(tmp_path) -> None:
    """builder 可在 helper 子进程中要求 verifier 复核仍存活的 wrapper child PID。"""
    evidence = tmp_path / "network-evidence"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            "bash",
            "-c",
            '"$1" "$2" --evidence "$STAGE4_NETWORK_ISOLATION_EVIDENCE" --process-pid "$$"',
            "bash",
            sys.executable,
            str(VERIFIER),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert completed.stdout == "PASS: live network isolation verified\n"


def test_python_runtime_builder_requires_live_network_before_creating_outputs(tmp_path) -> None:
    """直调 builder 或伪造环境均不得在 work/root 下留下任何输出。"""
    assert PYTHON_RUNTIME_BUILDER.is_file(), "stage 4 Python runtime builder is not implemented"
    work = tmp_path / "work"
    root = tmp_path / "root"
    completed = subprocess.run(
        [str(PYTHON_RUNTIME_BUILDER), "--work", str(work), "--root", str(root)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert "network isolation" in completed.stderr
    assert not work.exists()
    assert not root.exists()


def test_python_runtime_builder_requires_explicit_source_before_creating_outputs(tmp_path) -> None:
    """隔离已通过时，缺少源码根仍必须在任何 builder 输出前失败。"""
    evidence = tmp_path / "network-evidence"
    work = tmp_path / "work"
    root = tmp_path / "root"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            str(PYTHON_RUNTIME_BUILDER),
            "--work",
            str(work),
            "--root",
            str(root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert completed.stderr == "FAIL: --source is required\n"
    assert not work.exists()
    assert not root.exists()


def test_python_runtime_builder_rejects_relative_source_before_creating_outputs(tmp_path) -> None:
    """源码输入必须是调用者明确提供的绝对路径，不能随 cwd 漂移。"""
    evidence = tmp_path / "network-evidence"
    work = tmp_path / "work"
    root = tmp_path / "root"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            str(PYTHON_RUNTIME_BUILDER),
            "--source",
            "relative-source",
            "--work",
            str(work),
            "--root",
            str(root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert completed.stderr == "FAIL: --source must be absolute\n"
    assert not work.exists()
    assert not root.exists()


def test_python_runtime_builder_requires_pinned_micromamba_before_creating_outputs(
    tmp_path,
) -> None:
    """builder 只能接受显式 pinned micromamba，不能回退到调用者 PATH。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    source.mkdir()
    work = tmp_path / "work"
    root = tmp_path / "root"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            str(PYTHON_RUNTIME_BUILDER),
            "--source",
            str(source),
            "--work",
            str(work),
            "--root",
            str(root),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert completed.stderr == "FAIL: --micromamba is required\n"
    assert not work.exists()
    assert not root.exists()


def test_python_runtime_builder_rejects_invalid_micromamba_before_creating_outputs(
    tmp_path,
) -> None:
    """micromamba 必须是显式绝对可执行文件，不能将失败推迟到 create。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    source.mkdir()
    work = tmp_path / "work"
    root = tmp_path / "root"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            str(PYTHON_RUNTIME_BUILDER),
            "--source",
            str(source),
            "--work",
            str(work),
            "--root",
            str(root),
            "--micromamba",
            str(tmp_path / "not-an-executable"),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert completed.stderr == "FAIL: --micromamba must be an executable absolute path\n"
    assert not work.exists()
    assert not root.exists()


def test_python_runtime_builder_requires_explicit_package_cache_before_creating_outputs(
    tmp_path,
) -> None:
    """Conda canonical cache 必须由调用者显式交给 builder，禁止读取用户缓存。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    source.mkdir()
    micromamba = tmp_path / "micromamba"
    micromamba.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    micromamba.chmod(0o755)
    work = tmp_path / "work"
    root = tmp_path / "root"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            str(PYTHON_RUNTIME_BUILDER),
            "--source",
            str(source),
            "--work",
            str(work),
            "--root",
            str(root),
            "--micromamba",
            str(micromamba),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert completed.stderr == "FAIL: --package-cache is required\n"
    assert not work.exists()
    assert not root.exists()


def test_python_runtime_builder_requires_explicit_wheel_cache_before_creating_outputs(
    tmp_path,
) -> None:
    """官方 eCAL wheel canonical cache 也必须显式传入，禁止 pip cache 回退。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    source.mkdir()
    micromamba = tmp_path / "micromamba"
    micromamba.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    micromamba.chmod(0o755)
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    work = tmp_path / "work"
    root = tmp_path / "root"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            str(PYTHON_RUNTIME_BUILDER),
            "--source",
            str(source),
            "--work",
            str(work),
            "--root",
            str(root),
            "--micromamba",
            str(micromamba),
            "--package-cache",
            str(package_cache),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert completed.stderr == "FAIL: --wheel-cache is required\n"
    assert not work.exists()
    assert not root.exists()


def test_python_runtime_builder_rejects_source_without_locked_inputs_before_outputs(
    tmp_path,
) -> None:
    """空源码目录不能绕过锁与 verifier；失败前不得产生任何工作输出。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    source.mkdir()
    micromamba = tmp_path / "micromamba"
    micromamba.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    micromamba.chmod(0o755)
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"
    root = tmp_path / "root"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            str(PYTHON_RUNTIME_BUILDER),
            "--source",
            str(source),
            "--work",
            str(work),
            "--root",
            str(root),
            "--micromamba",
            str(micromamba),
            "--package-cache",
            str(package_cache),
            "--wheel-cache",
            str(wheel_cache),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert completed.stderr == "FAIL: source is missing stage 4 Python lock inputs\n"
    assert not work.exists()
    assert not root.exists()


def test_python_runtime_builder_requires_project_metadata_before_outputs(tmp_path) -> None:
    """wheel 构建源必须含 pyproject.toml，缺失时不得开始任何离线物化。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    micromamba = _write_python_runtime_builder_fixture(source, tmp_path / "unused")
    (source / "pyproject.toml").unlink()
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"
    completed = subprocess.run(
        [str(WRAPPER), "--evidence-dir", str(evidence), "--", str(PYTHON_RUNTIME_BUILDER),
         "--source", str(source), "--work", str(work), "--root", str(work / "root"),
         "--micromamba", str(micromamba), "--package-cache", str(package_cache),
         "--wheel-cache", str(wheel_cache), "--source-date-epoch", "1"],
        check=False, capture_output=True, text=True,
    )
    assert completed.returncode != 0
    assert completed.stderr == "FAIL: source is missing stage 4 Python lock inputs\n"
    assert not work.exists()


def test_python_runtime_builder_materializes_private_caches_before_toolchain_create(
    tmp_path,
) -> None:
    """通过全部输入验证后，先私有物化两个 cache，再以 toolchain lock 启动 create。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    micromamba_record = tmp_path / "micromamba.argv"
    micromamba = _write_python_runtime_builder_fixture(source, micromamba_record)
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"
    root = work / "root"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            str(PYTHON_RUNTIME_BUILDER),
            "--source",
            str(source),
            "--work",
            str(work),
            "--root",
            str(root),
            "--micromamba",
            str(micromamba),
            "--package-cache",
            str(package_cache),
            "--wheel-cache",
            str(wheel_cache),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 23
    assert completed.stderr == "FAKE_MICROMAMBA\n"
    fixture_wheel = "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"
    assert (work / "mamba-root" / "pkgs" / fixture_wheel).is_file()
    assert (work / "wheel-cache" / fixture_wheel).is_file()
    assert (work / "package-cache-materialization.json").is_file()
    assert (work / "wheel-cache-materialization.json").is_file()
    assert micromamba_record.read_text(encoding="utf-8").splitlines() == [
        "create",
        "--no-rc",
        "--no-env",
        "--root-prefix",
        str(work / "mamba-root"),
        "--prefix",
        str(work / "tool-env"),
        "--file",
        str(source / "packaging" / "locks" / "python-toolchain-linux-64.lock"),
        "--offline",
        "--always-copy",
        "--safety-checks",
        "enabled",
        "--yes",
    ]


def test_python_runtime_builder_rejects_micromamba_hash_drift_before_outputs(tmp_path) -> None:
    """可执行但未锁定的 micromamba 必须在 verifier/materializer 前被拒绝。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    micromamba = _write_python_runtime_builder_fixture(source, tmp_path / "argv")
    (source / "scripts" / "freeze_python_lock_cache.py").write_text(
        "from __future__ import annotations\n"
        "import hashlib\n"
        "import sys\n"
        "if hashlib.sha256(open(sys.argv[2], 'rb').read()).hexdigest() != "
        "'77b7790ec97f64581118f103585b175df4306f95829b0fa6bfe4a19cc88a1182':\n"
        "    raise SystemExit(1)\n",
        encoding="utf-8",
    )
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            str(PYTHON_RUNTIME_BUILDER),
            "--source",
            str(source),
            "--work",
            str(work),
            "--root",
            str(work / "root"),
            "--micromamba",
            str(micromamba),
            "--package-cache",
            str(package_cache),
            "--wheel-cache",
            str(wheel_cache),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode != 0
    assert completed.stderr == "FAIL: micromamba sha256 differs from pinned toolchain\n"
    assert not work.exists()


def test_python_runtime_builder_creates_runtime_after_toolchain(tmp_path) -> None:
    """toolchain 成功后，第二条且仅第二条 create 必须使用 runtime explicit lock。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    _write_python_runtime_builder_fixture(source, tmp_path / "unused")
    micromamba_record = tmp_path / "micromamba.argv"
    micromamba = source / "pinned-micromamba"
    _write_two_call_micromamba(micromamba, micromamba_record)
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"
    completed = subprocess.run(
        [
            str(WRAPPER),
            "--evidence-dir",
            str(evidence),
            "--",
            str(PYTHON_RUNTIME_BUILDER),
            "--source",
            str(source),
            "--work",
            str(work),
            "--root",
            str(work / "root"),
            "--micromamba",
            str(micromamba),
            "--package-cache",
            str(package_cache),
            "--wheel-cache",
            str(wheel_cache),
            "--source-date-epoch",
            "1",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 24
    assert completed.stderr == "FAKE_RUNTIME_CREATE\n"
    assert (micromamba_record.with_suffix(".argv.1")).read_text(
        encoding="utf-8"
    ).splitlines() == [
        "create",
        "--no-rc",
        "--no-env",
        "--root-prefix",
        str(work / "mamba-root"),
        "--prefix",
        str(work / "python-builder"),
        "--file",
        str(source / "packaging" / "locks" / "python-linux-64.lock"),
        "--offline",
        "--always-copy",
        "--safety-checks",
        "enabled",
        "--yes",
    ]


def test_python_runtime_builder_packs_pure_conda_runtime_before_wheel_install(tmp_path) -> None:
    """两个 create 成功后，conda-pack 必须直接面对未经 pip 污染的 python-builder。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    _write_python_runtime_builder_fixture(source, tmp_path / "unused")
    conda_pack_record = tmp_path / "conda-pack.argv"
    micromamba = source / "pinned-micromamba"
    _write_packing_micromamba(micromamba, conda_pack_record)
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"
    completed = subprocess.run(
        [
            str(WRAPPER), "--evidence-dir", str(evidence), "--",
            str(PYTHON_RUNTIME_BUILDER), "--source", str(source),
            "--work", str(work), "--root", str(work / "root"),
            "--micromamba", str(micromamba),
            "--package-cache", str(package_cache), "--wheel-cache", str(wheel_cache),
            "--source-date-epoch", "1",
        ],
        check=False, capture_output=True, text=True,
    )

    assert completed.returncode == 25
    assert completed.stderr == "FAKE_CONDA_PACK\n"
    assert conda_pack_record.read_text(encoding="utf-8").splitlines() == [
        "--prefix", str(work / "python-builder"),
        "--output", str(work / "python-pack" / "python-runtime.tar"),
        "--format", "tar", "--n-threads", "1",
    ]


def test_python_runtime_builder_stages_packed_runtime_before_wheel_install(tmp_path) -> None:
    """pack 成功后，runtime 只能解至 work/root/runtime/python，未运行 conda-unpack。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    _write_python_runtime_builder_fixture(source, tmp_path / "unused")
    micromamba = source / "pinned-micromamba"
    _write_staging_micromamba(micromamba)
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"
    completed = subprocess.run(
        [
            str(WRAPPER), "--evidence-dir", str(evidence), "--",
            str(PYTHON_RUNTIME_BUILDER), "--source", str(source),
            "--work", str(work), "--root", str(work / "root"),
            "--micromamba", str(micromamba), "--package-cache", str(package_cache),
            "--wheel-cache", str(wheel_cache), "--source-date-epoch", "1",
        ],
        check=False, capture_output=True, text=True,
    )

    assert completed.returncode != 0
    assert (work / "root" / "runtime" / "python" / "bin" / "python").read_text(
        encoding="utf-8"
    ) == "python"
    assert (work / "root" / "runtime" / "python" / "conda-meta" / "history").is_file()


def test_python_runtime_builder_builds_then_installs_private_and_project_wheels(tmp_path) -> None:
    """pack 后必须先构建唯一项目 wheel，再以无索引 pip 依次安装 eCAL 和项目 wheel。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    _write_python_runtime_builder_fixture(source, tmp_path / "unused")
    pip_record = tmp_path / "tool-python.argv"
    micromamba = source / "pinned-micromamba"
    _write_wheel_install_micromamba(micromamba, pip_record)
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"
    completed = subprocess.run(
        [
            str(WRAPPER), "--evidence-dir", str(evidence), "--",
            str(PYTHON_RUNTIME_BUILDER), "--source", str(source),
            "--work", str(work), "--root", str(work / "root"),
            "--micromamba", str(micromamba), "--package-cache", str(package_cache),
            "--wheel-cache", str(wheel_cache), "--source-date-epoch", "1",
        ],
        check=False, capture_output=True, text=True,
    )

    assert completed.returncode == 26
    assert completed.stderr == "FAKE_PROJECT_PIP\n"
    expected_prefix = str(work / "root" / "runtime" / "python")
    assert (pip_record.with_suffix(".argv.0")).read_text(encoding="utf-8").splitlines() == [
        "-m", "build", "--wheel", "--no-isolation", "--outdir", str(work / "wheel"), str(source),
    ]
    assert (pip_record.with_suffix(".argv.1")).read_text(encoding="utf-8").splitlines() == [
        "-m", "pip", "install", "--no-deps", "--no-index", "--no-compile", "--prefix", expected_prefix,
        str(work / "wheel-cache" / "eclipse_ecal-6.1.1-cp310-cp310-manylinux_2_28_x86_64.whl"),
    ]
    assert (pip_record.with_suffix(".argv.2")).read_text(encoding="utf-8").splitlines() == [
        "-m", "pip", "install", "--no-deps", "--no-index", "--no-compile", "--prefix", expected_prefix,
        str(work / "wheel" / "fixture_project-0.1.0-py3-none-any.whl"),
    ]


def test_python_runtime_builder_removes_pip_direct_urls_and_refreshes_records(tmp_path) -> None:
    """安装后的 eCAL 与项目 RECORD 不能保留指向本轮 work 路径的 direct_url。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    _write_python_runtime_builder_fixture(source, tmp_path / "unused")
    micromamba = source / "pinned-micromamba"
    _write_wheel_install_micromamba(micromamba, tmp_path / "tool-python.argv", fail_project_pip=False)
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"
    completed = subprocess.run(
        [
            str(WRAPPER), "--evidence-dir", str(evidence), "--",
            str(PYTHON_RUNTIME_BUILDER), "--source", str(source),
            "--work", str(work), "--root", str(work / "root"),
            "--micromamba", str(micromamba), "--package-cache", str(package_cache),
            "--wheel-cache", str(wheel_cache), "--source-date-epoch", "1",
        ],
        check=False, capture_output=True, text=True,
    )

    assert completed.returncode == 0, completed.stderr
    site_packages = work / "root" / "runtime" / "python" / "lib" / "python3.10" / "site-packages"
    for dist_info in ("eclipse_ecal-6.1.1.dist-info", "fixture_project-0.1.0.dist-info"):
        assert not (site_packages / dist_info / "direct_url.json").exists()
    ecal_record = (site_packages / "eclipse_ecal-6.1.1.dist-info" / "RECORD").read_text(
        encoding="utf-8"
    )
    assert "direct_url.json" not in ecal_record
    assert ecal_record.endswith("eclipse_ecal-6.1.1.dist-info/RECORD,,\n")
    assert (site_packages / "fixture_project-0.1.0.dist-info" / "RECORD").read_text(
        encoding="utf-8"
    ) == "fixture_project-0.1.0.dist-info/RECORD,,\n"


def test_python_runtime_builder_removes_only_project_console_scripts(tmp_path) -> None:
    """项目 wheel 的 console_scripts 必须按 entry_points 精确删除并从 RECORD 移除。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    _write_python_runtime_builder_fixture(source, tmp_path / "unused")
    micromamba = source / "pinned-micromamba"
    _write_wheel_install_micromamba(
        micromamba,
        tmp_path / "tool-python.argv",
        fail_project_pip=False,
        create_project_console_script=True,
    )
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"

    completed = subprocess.run(
        [
            str(WRAPPER), "--evidence-dir", str(evidence), "--",
            str(PYTHON_RUNTIME_BUILDER), "--source", str(source),
            "--work", str(work), "--root", str(work / "root"),
            "--micromamba", str(micromamba), "--package-cache", str(package_cache),
            "--wheel-cache", str(wheel_cache), "--source-date-epoch", "1",
        ],
        check=False, capture_output=True, text=True,
    )

    assert completed.returncode == 0, completed.stderr
    runtime = work / "root" / "runtime" / "python"
    assert not (runtime / "bin" / "fixture-command").exists()
    record = runtime / "lib" / "python3.10" / "site-packages" / "fixture_project-0.1.0.dist-info" / "RECORD"
    assert "../../../bin/fixture-command" not in record.read_text(encoding="utf-8")


def test_python_runtime_builder_removes_history_and_bytecode_only_after_pip(tmp_path) -> None:
    """两个 wheel 都安装后，staging 才可删除 Conda history 与 Python bytecode。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    _write_python_runtime_builder_fixture(source, tmp_path / "unused")
    micromamba = source / "pinned-micromamba"
    _write_wheel_install_micromamba(micromamba, tmp_path / "tool-python.argv", fail_project_pip=False)
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"
    completed = subprocess.run(
        [
            str(WRAPPER), "--evidence-dir", str(evidence), "--",
            str(PYTHON_RUNTIME_BUILDER), "--source", str(source),
            "--work", str(work), "--root", str(work / "root"),
            "--micromamba", str(micromamba), "--package-cache", str(package_cache),
            "--wheel-cache", str(wheel_cache), "--source-date-epoch", "1",
        ],
        check=False, capture_output=True, text=True,
    )

    assert completed.returncode == 0, completed.stderr
    runtime = work / "root" / "runtime" / "python"
    assert not (runtime / "conda-meta" / "history").exists()
    assert not list(runtime.rglob("*.pyc"))
    assert not list(runtime.rglob("__pycache__"))


def test_python_runtime_builder_runs_relocation_only_in_random_copy(tmp_path) -> None:
    """builder 完成 staging 后只在随机副本执行 conda-unpack，并保留源 tree。"""
    evidence = tmp_path / "network-evidence"
    source = tmp_path / "source"
    _write_python_runtime_builder_fixture(source, tmp_path / "unused")
    micromamba = source / "pinned-micromamba"
    _write_wheel_install_micromamba(micromamba, tmp_path / "tool-python.argv", fail_project_pip=False)
    package_cache = tmp_path / "package-cache"
    package_cache.mkdir()
    wheel_cache = tmp_path / "wheel-cache"
    wheel_cache.mkdir()
    work = tmp_path / "work"

    completed = subprocess.run(
        [
            str(WRAPPER), "--evidence-dir", str(evidence), "--",
            str(PYTHON_RUNTIME_BUILDER), "--source", str(source),
            "--work", str(work), "--root", str(work / "root"),
            "--micromamba", str(micromamba), "--package-cache", str(package_cache),
            "--wheel-cache", str(wheel_cache), "--source-date-epoch", "1",
        ],
        check=False, capture_output=True, text=True,
    )

    assert completed.returncode == 0, completed.stderr
    runtime = work / "root" / "runtime" / "python"
    relocation = json.loads((work / "python-runtime-relocation.json").read_text(encoding="utf-8"))
    assert relocation["source_runtime"] == str(runtime)
    assert relocation["source_tree_before"] == relocation["source_tree_after"]
    relocated_runtime = Path(relocation["relocated_runtime"])
    assert relocated_runtime.parent.name.startswith("stage4-python-relocation-")
    assert (relocated_runtime / "bin" / "conda-unpack-ran.txt").is_file()
    assert not (runtime / "bin" / "conda-unpack-ran.txt").exists()
