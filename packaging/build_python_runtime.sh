#!/usr/bin/env bash
# 阶段四 Python runtime builder：仅在已复核的无外网 namespace 内物化和打包运行时。
set -euo pipefail

fail() {
  printf 'FAIL: %s\n' "$*" >&2
  exit 1
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERIFY_NETWORK="$SCRIPT_DIR/../scripts/verify_network_isolation.py"
NORMALIZE_RUNTIME_INSTALL="$SCRIPT_DIR/../scripts/normalize_python_runtime_install.py"
SANITIZE_RUNTIME_TREE="$SCRIPT_DIR/../scripts/sanitize_python_runtime_tree.py"
VERIFY_RUNTIME_ECAL_INSTALL="$SCRIPT_DIR/../scripts/verify_python_runtime_ecal_install.py"
VERIFY_RUNTIME_ISOLATION="$SCRIPT_DIR/../scripts/verify_python_runtime_isolation.py"
RUNTIME_TREE_DIGEST="$SCRIPT_DIR/../scripts/python_runtime_tree_digest.py"
VERIFY_RUNTIME_RELOCATION="$SCRIPT_DIR/../scripts/verify_python_runtime_relocation.py"

# 此检查必须先于参数解析和任何 mkdir/copy/create；环境 token 单独不能替代内核状态。
EVIDENCE_DIR="${STAGE4_NETWORK_ISOLATION_EVIDENCE:-}"
[[ -n "$EVIDENCE_DIR" ]] || fail "network isolation evidence is required"
command -v python3 >/dev/null 2>&1 || fail "python3 is required for network isolation verification"
[[ -f "$NORMALIZE_RUNTIME_INSTALL" ]] || fail "Python runtime metadata normalizer is required"
[[ -f "$SANITIZE_RUNTIME_TREE" ]] || fail "Python runtime staging sanitizer is required"
[[ -f "$VERIFY_RUNTIME_ECAL_INSTALL" ]] || fail "Python runtime eCAL install verifier is required"
[[ -f "$VERIFY_RUNTIME_ISOLATION" ]] || fail "Python runtime isolation verifier is required"
[[ -f "$RUNTIME_TREE_DIGEST" ]] || fail "Python runtime tree digest helper is required"
[[ -f "$VERIFY_RUNTIME_RELOCATION" ]] || fail "Python runtime relocation helper is required"
python3 "$VERIFY_NETWORK" --evidence "$EVIDENCE_DIR" --process-pid "$$" >/dev/null || fail "network isolation verification failed"

SOURCE=""
WORK=""
ROOT=""
MICROMAMBA=""
PACKAGE_CACHE=""
WHEEL_CACHE=""
SOURCE_DATE_EPOCH_VALUE=""
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --source)
      SOURCE="${2:-}"
      shift 2
      ;;
    --work)
      WORK="${2:-}"
      shift 2
      ;;
    --root)
      ROOT="${2:-}"
      shift 2
      ;;
    --micromamba)
      MICROMAMBA="${2:-}"
      shift 2
      ;;
    --package-cache)
      PACKAGE_CACHE="${2:-}"
      shift 2
      ;;
    --wheel-cache)
      WHEEL_CACHE="${2:-}"
      shift 2
      ;;
    --source-date-epoch)
      SOURCE_DATE_EPOCH_VALUE="${2:-}"
      shift 2
      ;;
    *)
      fail "unknown Python runtime builder argument: $1"
      ;;
  esac
done
[[ -n "$SOURCE" ]] || fail "--source is required"
[[ "$SOURCE" == /* ]] || fail "--source must be absolute"
[[ "$WORK" == /* && "$ROOT" == /* ]] || fail "--work and --root must be absolute"
[[ -n "$MICROMAMBA" ]] || fail "--micromamba is required"
[[ "$MICROMAMBA" == /* && -x "$MICROMAMBA" ]] || fail "--micromamba must be an executable absolute path"
[[ -n "$PACKAGE_CACHE" ]] || fail "--package-cache is required"
[[ -n "$WHEEL_CACHE" ]] || fail "--wheel-cache is required"
[[ -d "$SOURCE" && -d "$PACKAGE_CACHE" && -d "$WHEEL_CACHE" ]] || fail "source and canonical caches must be existing directories"
for required_source_file in \
  "$SOURCE/pyproject.toml" \
  "$SOURCE/packaging/python-environment.yml" \
  "$SOURCE/packaging/python-toolchain-environment.yml" \
  "$SOURCE/packaging/locks/virtual-packages.yml" \
  "$SOURCE/packaging/locks/python.conda-lock.yml" \
  "$SOURCE/packaging/locks/python-linux-64.lock" \
  "$SOURCE/packaging/locks/python-toolchain.conda-lock.yml" \
  "$SOURCE/packaging/locks/python-toolchain-linux-64.lock" \
  "$SOURCE/packaging/locks/python-package-cache.manifest.json" \
  "$SOURCE/packaging/locks/python-wheel-cache.manifest.json" \
  "$SOURCE/scripts/freeze_python_lock_cache.py" \
  "$SOURCE/scripts/verify_python_lock_cache.py" \
  "$SOURCE/scripts/verify_python_wheel_cache.py" \
  "$SOURCE/scripts/materialize_python_package_cache.py" \
  "$SOURCE/scripts/materialize_python_wheel_cache.py"
do
  [[ -f "$required_source_file" ]] || fail "source is missing stage 4 Python lock inputs"
done
[[ "$SOURCE_DATE_EPOCH_VALUE" =~ ^[0-9]+$ ]] || fail "--source-date-epoch must be a nonnegative integer"
[[ "$ROOT" == "$WORK/root" ]] || fail "--root must equal --work/root"
[[ ! -e "$WORK" && -d "$(dirname "$WORK")" ]] || fail "--work must be a new directory under an existing parent"

python3 "$SOURCE/scripts/freeze_python_lock_cache.py" \
  --micromamba "$MICROMAMBA" --check-only >/dev/null || fail "micromamba sha256 differs from pinned toolchain"

# 所有只读输入先由独立结构化 verifier 复核，任何失败均在创建本轮 work 前结束。
python3 "$SOURCE/scripts/verify_python_lock_cache.py" \
  --runtime-spec "$SOURCE/packaging/python-environment.yml" \
  --toolchain-spec "$SOURCE/packaging/python-toolchain-environment.yml" \
  --virtual-packages "$SOURCE/packaging/locks/virtual-packages.yml" \
  --runtime-unified "$SOURCE/packaging/locks/python.conda-lock.yml" \
  --runtime-explicit "$SOURCE/packaging/locks/python-linux-64.lock" \
  --toolchain-unified "$SOURCE/packaging/locks/python-toolchain.conda-lock.yml" \
  --toolchain-explicit "$SOURCE/packaging/locks/python-toolchain-linux-64.lock" \
  --cache-manifest "$SOURCE/packaging/locks/python-package-cache.manifest.json" \
  --cache-root "$PACKAGE_CACHE" >/dev/null || fail "Python lock and package cache verification failed"
python3 "$SOURCE/scripts/verify_python_wheel_cache.py" \
  --manifest "$SOURCE/packaging/locks/python-wheel-cache.manifest.json" \
  --cache-root "$WHEEL_CACHE" >/dev/null || fail "Python wheel cache verification failed"

mkdir -m 0755 "$WORK"
python3 "$SOURCE/scripts/materialize_python_package_cache.py" \
  --manifest "$SOURCE/packaging/locks/python-package-cache.manifest.json" \
  --canonical-cache "$PACKAGE_CACHE" \
  --destination "$WORK/mamba-root/pkgs" \
  --evidence "$WORK/package-cache-materialization.json" >/dev/null
python3 "$SOURCE/scripts/materialize_python_wheel_cache.py" \
  --manifest "$SOURCE/packaging/locks/python-wheel-cache.manifest.json" \
  --canonical-cache "$WHEEL_CACHE" \
  --destination "$WORK/wheel-cache" \
  --evidence "$WORK/wheel-cache-materialization.json" >/dev/null
mkdir -m 0700 "$WORK/empty-home"

# tool env 先于 runtime 创建；显式清空宿主 Conda/Mamba/pip 配置和代理变量。
env -i \
  HOME="$WORK/empty-home" \
  LANG=C.UTF-8 \
  LC_ALL=C.UTF-8 \
  PATH="$PATH" \
  PYTHONDONTWRITEBYTECODE=1 \
  SOURCE_DATE_EPOCH="$SOURCE_DATE_EPOCH_VALUE" \
  TZ=UTC \
  "$MICROMAMBA" create \
    --no-rc --no-env \
    --root-prefix "$WORK/mamba-root" \
    --prefix "$WORK/tool-env" \
    --file "$SOURCE/packaging/locks/python-toolchain-linux-64.lock" \
    --offline --always-copy --safety-checks enabled --yes

env -i \
  HOME="$WORK/empty-home" \
  LANG=C.UTF-8 \
  LC_ALL=C.UTF-8 \
  PATH="$PATH" \
  PYTHONDONTWRITEBYTECODE=1 \
  SOURCE_DATE_EPOCH="$SOURCE_DATE_EPOCH_VALUE" \
  TZ=UTC \
  "$MICROMAMBA" create \
    --no-rc --no-env \
    --root-prefix "$WORK/mamba-root" \
    --prefix "$WORK/python-builder" \
    --file "$SOURCE/packaging/locks/python-linux-64.lock" \
    --offline --always-copy --safety-checks enabled --yes

mkdir -m 0755 "$WORK/python-pack"
"$WORK/tool-env/bin/conda-pack" \
  --prefix "$WORK/python-builder" \
  --output "$WORK/python-pack/python-runtime.tar" \
  --format tar --n-threads 1

mkdir -p "$ROOT/runtime/python"
tar --extract --file "$WORK/python-pack/python-runtime.tar" \
  --directory "$ROOT/runtime/python" --no-same-owner --no-same-permissions

# 项目 wheel 只能由本轮可写 source-build 产出，输出目录必须恰有一个普通 wheel 文件。
mkdir -m 0755 "$WORK/wheel"
env -i \
  HOME="$WORK/empty-home" \
  LANG=C.UTF-8 \
  LC_ALL=C.UTF-8 \
  PATH="$PATH" \
  PYTHONDONTWRITEBYTECODE=1 \
  SOURCE_DATE_EPOCH="$SOURCE_DATE_EPOCH_VALUE" \
  TZ=UTC \
  "$WORK/tool-env/bin/python" -m build --wheel --no-isolation \
    --outdir "$WORK/wheel" "$SOURCE"
mapfile -t PROJECT_WHEEL_FILES < <(
  find "$WORK/wheel" -mindepth 1 -maxdepth 1 -type f -name '*.whl' -printf '%f\n' | LC_ALL=C sort
)
[[ "${#PROJECT_WHEEL_FILES[@]}" -eq 1 ]] || fail "project wheel build must produce exactly one wheel"
PROJECT_WHEEL="$WORK/wheel/${PROJECT_WHEEL_FILES[0]}"
[[ -f "$PROJECT_WHEEL" && ! -L "$PROJECT_WHEEL" ]] || fail "project wheel must be a regular file"

# 解析物化 manifest 并在 pip 前复算私有 eCAL wheel 的身份，禁止路径或缓存回退。
PRIVATE_ECAL_WHEEL="$(python3 - "$SOURCE/packaging/locks/python-wheel-cache.manifest.json" "$WORK/wheel-cache" <<'PY'
import hashlib
import json
from pathlib import Path
import stat
import sys

manifest_path = Path(sys.argv[1])
cache_root = Path(sys.argv[2]).resolve()
document = json.loads(manifest_path.read_text(encoding="utf-8"))
wheel = document.get("wheel") if isinstance(document, dict) else None
if not isinstance(wheel, dict):
    raise SystemExit("wheel manifest is invalid")
filename = wheel.get("filename")
size = wheel.get("size")
sha256 = wheel.get("sha256")
if (
    not isinstance(filename, str)
    or not filename
    or Path(filename).name != filename
    or not isinstance(size, int)
    or isinstance(size, bool)
    or size < 1
    or not isinstance(sha256, str)
    or len(sha256) != 64
    or any(character not in "0123456789abcdef" for character in sha256)
):
    raise SystemExit("wheel manifest identity is invalid")
artifact = cache_root / filename
artifact_stat = artifact.lstat()
if (
    not stat.S_ISREG(artifact_stat.st_mode)
    or stat.S_ISLNK(artifact_stat.st_mode)
    or artifact_stat.st_nlink != 1
):
    raise SystemExit("private eCAL wheel is not a singly linked regular file")
digest = hashlib.sha256()
actual_size = 0
with artifact.open("rb") as stream:
    while chunk := stream.read(1024 * 1024):
        actual_size += len(chunk)
        digest.update(chunk)
if (actual_size, digest.hexdigest()) != (size, sha256):
    raise SystemExit("private eCAL wheel digest differs from manifest")
print(artifact)
PY
)" || fail "private eCAL wheel verification failed"
[[ "$PRIVATE_ECAL_WHEEL" == "$WORK/wheel-cache/"* ]] || fail "private eCAL wheel escaped work cache"

# pip 必须只看到两个绝对本地 wheel；先 eCAL，再安装项目自身，不允许解析依赖或访问 index。
for wheel in "$PRIVATE_ECAL_WHEEL" "$PROJECT_WHEEL"; do
  env -i \
    HOME="$WORK/empty-home" \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    PATH="$PATH" \
    PIP_CONFIG_FILE=/dev/null \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    SOURCE_DATE_EPOCH="$SOURCE_DATE_EPOCH_VALUE" \
    TZ=UTC \
    "$WORK/tool-env/bin/python" -m pip install \
      --no-deps --no-index --no-compile --prefix "$ROOT/runtime/python" "$wheel"
done
python3 "$NORMALIZE_RUNTIME_INSTALL" \
  --runtime-root "$ROOT/runtime/python" \
  --wheel "$PRIVATE_ECAL_WHEEL" \
  --wheel "$PROJECT_WHEEL" >/dev/null
python3 "$VERIFY_RUNTIME_ECAL_INSTALL" \
  --runtime-root "$ROOT/runtime/python" \
  --manifest "$SOURCE/packaging/locks/python-wheel-cache.manifest.json" >/dev/null
python3 "$SANITIZE_RUNTIME_TREE" \
  --runtime-root "$ROOT/runtime/python" \
  --source-date-epoch "$SOURCE_DATE_EPOCH_VALUE" >/dev/null
python3 "$VERIFY_RUNTIME_ISOLATION" \
  --runtime-root "$ROOT/runtime/python" \
  --forbidden-prefix "$SOURCE" \
  --forbidden-prefix "$WORK/python-builder" \
  --forbidden-prefix "$WORK/tool-env" \
  --forbidden-prefix "$WORK/mamba-root" \
  --forbidden-prefix "$WORK/wheel-cache" \
  --forbidden-prefix "$PACKAGE_CACHE" \
  --forbidden-prefix "$WHEEL_CACHE" \
  --evidence "$WORK/python-runtime-isolation.json" >/dev/null
python3 "$RUNTIME_TREE_DIGEST" --runtime-root "$ROOT/runtime/python" \
  > "$WORK/python-runtime-tree-digest.json"

# staging 保持原样；conda-unpack 只能在本轮随机副本执行，证据同时锁定源 tree 前后摘要。
python3 "$VERIFY_RUNTIME_RELOCATION" \
  --runtime-root "$ROOT/runtime/python" \
  --copy-parent "$WORK" \
  --evidence "$WORK/python-runtime-relocation.json" >/dev/null
