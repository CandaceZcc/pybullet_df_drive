#!/usr/bin/env bash
# 阶段四离线 C++ 依赖构建入口：先锁定私有输出根关系，再接入 cache/源码/CMake 门禁。
set -euo pipefail

die() {
  printf 'FAIL: %s\n' "$1" >&2
  exit 1
}

usage() {
  printf '%s\n' 'Usage: build_dependencies.sh --cmake PATH --cc PATH --cxx PATH --source-archive-cache PATH --source-archive-manifest PATH --dependency-lock PATH --source-work PATH --build-root PATH --install-prefix PATH [--validation-prefix PATH] --source-date-epoch EPOCH'
}

require_absolute_path() {
  case "$2" in
    /*) ;;
    *) die "$1 must be an absolute path" ;;
  esac
}

is_same_or_nested() {
  [[ "$2" == "$1" || "$2" == "$1"/* ]]
}

# 仅接受调用方固定的真实工具文件，避免 configure 时经 PATH 或符号链接漂移。
require_tool() {
  local label="$1"
  local path="$2"
  [[ -n "$path" ]] || die "$label is required"
  require_absolute_path "$label" "$path"
  [[ -f "$path" && ! -L "$path" && -x "$path" ]] || die "$label must be an executable regular file"
}

require_version() {
  local label="$1"
  local path="$2"
  local expected="$3"
  local version_output
  version_output="$($path --version 2>&1)" || die "$label version probe failed"
  [[ "$version_output" =~ $expected ]] || die "$label version differs from stage 4 toolchain"
}

# 每个 archive 只允许一个已由安全 materializer 创建的源码根。
source_root_for_dependency() {
  local dependency="$1"
  local candidates=("$source_work/trees/$dependency"/*)
  [[ "${#candidates[@]}" -eq 1 && -d "${candidates[0]}" ]] || die "materialized source root is invalid for $dependency"
  printf '%s\n' "${candidates[0]}"
}

cmake_source_root_for_dependency() {
  local dependency="$1"
  local source_root
  source_root="$(source_root_for_dependency "$dependency")"
  case "$dependency" in
    # Zstd release archive 的根目录没有 CMakeLists，官方入口固定在此子目录。
    zstd) source_root="$source_root/build/cmake" ;;
  esac
  [[ -f "$source_root/CMakeLists.txt" ]] || die "offline CMake entry is not implemented for $dependency"
  printf '%s\n' "$source_root"
}

# 统一传递可重现 CMake 参数；调用方工具和所有可写根均由入口校验。
build_cmake_dependency() {
  local dependency="$1"
  local source_root
  source_root="$(cmake_source_root_for_dependency "$dependency")"
  build_cmake_source "$dependency" "$source_root"
}

# Conda GCC 的 specs 会把本轮 toolchain lib 以绝对 RPATH 注入每个 ELF；仅在私有 build root 生成删除该项的副本。
sanitize_compiler_specs() {
  local label="$1"
  local compiler="$2"
  local specs_path toolchain_root injected_rule sanitized_path
  specs_path="$($compiler -print-file-name=specs)" || die "$label compiler specs probe failed"
  if [[ "$specs_path" == 'specs' && ! -e "$specs_path" ]]; then
    printf '\n'
    return
  fi
  [[ "$specs_path" == /* ]] || die "$label compiler specs location must be absolute"
  [[ -f "$specs_path" && ! -L "$specs_path" ]] || die "$label compiler specs must be a regular file"
  toolchain_root="$(cd "$(dirname "$compiler")/.." && pwd -P)"
  injected_rule="%{!static:-rpath $toolchain_root/lib}"
  mkdir -p "$build_root/toolchain-specs"
  sanitized_path="$build_root/toolchain-specs/$label.specs"
  python3 - "$specs_path" "$sanitized_path" "$injected_rule" <<'PY'
from pathlib import Path
import os
import sys

source = Path(sys.argv[1])
target = Path(sys.argv[2])
rule = sys.argv[3].encode()
payload = source.read_bytes()
if payload.count(rule) != 1:
    raise SystemExit("compiler specs must contain exactly one injected toolchain RPATH rule")
expected = payload.replace(rule, b"")
if target.exists() or target.is_symlink():
    if target.is_symlink() or not target.is_file() or target.read_bytes() != expected:
        raise SystemExit("sanitized compiler specs differ from the verified source")
else:
    with target.open("xb") as handle:
        handle.write(expected)
        handle.flush()
        os.fsync(handle.fileno())
PY
  [[ -f "$sanitized_path" && ! -L "$sanitized_path" ]] || die "$label sanitized compiler specs are invalid"
  printf '%s\n' "-specs=$sanitized_path"
}

# 将清理后的 specs 固化在真实 compiler wrapper 中，项目修改 CMake cache 也不能绕过。
compiler_for_cmake() {
  local label="$1"
  local compiler="$2"
  local specs_argument wrapper expected
  specs_argument="$(sanitize_compiler_specs "$label" "$compiler")"
  if [[ -z "$specs_argument" ]]; then
    printf '%s\n' "$compiler"
    return
  fi
  mkdir -p "$build_root/toolchain-compilers"
  wrapper="$build_root/toolchain-compilers/$label"
  expected="$({
    printf '%s\n' '#!/usr/bin/env bash' 'set -euo pipefail'
    printf 'exec %q %q "$@"\n' "$compiler" "$specs_argument"
  })"
  if [[ -e "$wrapper" ]]; then
    [[ -f "$wrapper" && ! -L "$wrapper" && -x "$wrapper" ]] || die "$label compiler wrapper is invalid"
    [[ "$(<"$wrapper")" == "$expected" ]] || die "$label compiler wrapper differs from verified inputs"
  else
    (set -o noclobber; printf '%s\n' "$expected" > "$wrapper") || die "$label compiler wrapper creation failed"
    chmod 0755 "$wrapper"
  fi
  printf '%s\n' "$wrapper"
}

# 所有 CMake 子项目共享固定 compiler、prefix 与可复现映射参数。
build_cmake_source() {
  local label="$1"
  local source_root="$2"
  local target_prefix="${3:-$install_prefix}"
  local prefix_path="${4:-$install_prefix}"
  [[ -f "$source_root/CMakeLists.txt" ]] || die "offline CMake entry is not implemented for $label"
  local dependency_build="$build_root/$label"
  # 配置期前缀必须稳定；实际文件仍由 install --prefix 写进调用方私有根。
  local configured_prefix='/stage4/dependencies'
  [[ "$target_prefix" == "$install_prefix" ]] || configured_prefix='/stage4/validation-tools'
  local source_flags="-ffile-prefix-map=$source_work=/stage4/source -fdebug-prefix-map=$source_work=/stage4/source -fmacro-prefix-map=$source_work=/stage4/source -ffile-prefix-map=$target_prefix=$configured_prefix -fdebug-prefix-map=$target_prefix=$configured_prefix -fmacro-prefix-map=$target_prefix=$configured_prefix"
  local build_flags="-ffile-prefix-map=$build_root=/stage4/build -fdebug-prefix-map=$build_root=/stage4/build -fmacro-prefix-map=$build_root=/stage4/build"
  local c_compiler cxx_compiler
  c_compiler="$(compiler_for_cmake 'cc' "$cc")"
  cxx_compiler="$(compiler_for_cmake 'cxx' "$cxx")"
  local cmake_options=(
    -DCMAKE_SKIP_INSTALL_RPATH=TRUE
  )
  case "$label" in
    # Protobuf 33.6 否则会加入 GTest，并在缺少包时回退到 FetchContent。
    protobuf)
      cmake_options=(
        -Dprotobuf_BUILD_TESTS=OFF
        -Dprotobuf_LOCAL_DEPENDENCIES_ONLY=ON
      )
      ;;
    # 最小 C++ SDK 只构建 raw core；Python binding 由官方 wheel 独立提供。
    ecal)
      cmake_options=(
        "-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES=$source_root/cmake/submodule_dependencies.cmake"
        -DECAL_USE_HDF5=OFF
        -DECAL_USE_QT=OFF
        -DECAL_USE_CURL=OFF
        -DECAL_USE_FTXUI=OFF
        -DECAL_USE_PROTOBUF=OFF
        -DECAL_BUILD_APPS=OFF
        -DECAL_BUILD_SAMPLES=OFF
        -DECAL_BUILD_C_BINDING=OFF
        -DECAL_BUILD_CSHARP_BINDING=OFF
        -DECAL_BUILD_PY_BINDING=OFF
        -DECAL_CORE_CONFIGURATION=OFF
        -DECAL_INSTALL_SAMPLE_SOURCES=OFF
      )
      ;;
    # PCL 只服务格式验证，明确禁用图形与可选硬件入口以固定最小系统边界。
    pcl-validation)
      cmake_options=(
        # PCL validator 是独立可执行验证工具，必须能从本轮 validation prefix 自举加载。
        -DCMAKE_SKIP_INSTALL_RPATH=FALSE
        -DWITH_OPENGL=OFF
        -DWITH_VTK=OFF
        -DWITH_QT=OFF
        -DWITH_PCAP=OFF
        -DWITH_PNG=OFF
        -DWITH_LIBUSB=OFF
        -DWITH_OPENNI=OFF
        -DWITH_OPENNI2=OFF
        -DWITH_CUDA=OFF
      )
      ;;
  esac

  "$cmake" -S "$source_root" -B "$dependency_build" \
    -DCMAKE_C_COMPILER="$c_compiler" \
    -DCMAKE_CXX_COMPILER="$cxx_compiler" \
    -DCMAKE_INSTALL_PREFIX="$configured_prefix" \
    -DCMAKE_PREFIX_PATH="$prefix_path" \
    -DCMAKE_C_FLAGS="$source_flags $build_flags" \
    -DCMAKE_CXX_FLAGS="$source_flags $build_flags" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
    -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
    -DCMAKE_FIND_USE_PACKAGE_REGISTRY=FALSE \
    -DCMAKE_FIND_USE_SYSTEM_PACKAGE_REGISTRY=FALSE \
    "${cmake_options[@]}"
  "$cmake" --build "$dependency_build"
  "$cmake" --install "$dependency_build" --prefix "$target_prefix"
}

rewrite_elf_rpath() {
  local target="$1"
  local old_rpath="$2"
  local new_rpath="$3"
  local rewriter
  rewriter="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/rewrite_stage4_rpath.cmake"
  [[ -f "$rewriter" && ! -L "$rewriter" ]] || die 'stage4 RPATH rewriter is missing'
  "$cmake" \
    -DSTAGE4_RPATH_TARGET="$target" \
    -DSTAGE4_RPATH_OLD="$old_rpath" \
    -DSTAGE4_RPATH_NEW="$new_rpath" \
    -P "$rewriter"
}

# PCL 的完整 install 依赖完整 build；禁止只编译 pcl_pcd2ply 后执行全量 install。
build_pcl_validator() {
  local candidates=("$source_work/validation/trees/pcl"/*)
  local validator runtime_evidence ldd_evidence pcl_library library_count=0
  [[ "${#candidates[@]}" -eq 1 && -d "${candidates[0]}" ]] || die 'materialized PCL validation source root is invalid'
  [[ -n "$validation_prefix" ]] || die 'validation prefix is required for locked PCL validator'
  build_cmake_source 'pcl-validation' "${candidates[0]}" "$validation_prefix" ''
  validator="$validation_prefix/bin/pcl_pcd2ply"
  [[ -f "$validator" && -x "$validator" ]] || die 'PCL validator install is incomplete'
  # PCL 在配置期固定绝对 RPATH；安装后仅将本轮 validator 树改为 ELF 相对 RPATH。
  rewrite_elf_rpath "$validator" '/stage4/validation-tools/lib' '$ORIGIN/../lib'
  for pcl_library in "$validation_prefix"/lib/libpcl_*.so.*; do
    [[ -f "$pcl_library" && ! -L "$pcl_library" ]] || continue
    rewrite_elf_rpath "$pcl_library" '/stage4/validation-tools/lib' '$ORIGIN'
    library_count=$((library_count + 1))
  done
  [[ "$library_count" -gt 0 ]] || die 'PCL validation libraries are incomplete'
  # 在锁定 Ubuntu 系统和断网 namespace 内保存真实 CLI 与动态链接验收证据。
  runtime_evidence="$build_root/pcl-validation-runtime.txt"
  ldd_evidence="$build_root/pcl-validation-ldd.txt"
  # PCL 1.14 的帮助文本故意以 255 退出；Syntax 输出仍证明 CLI 已可执行。
  if ! "$validator" --help > "$runtime_evidence" 2>&1; then
    grep -Fq 'Syntax is:' "$runtime_evidence" || die 'PCL validator help probe failed'
  fi
  ldd "$validator" > "$ldd_evidence" 2>&1
  ! grep -Fq 'not found' "$ldd_evidence" || die 'PCL validator has unresolved runtime libraries'
}

# eCAL 源码内置 CMakeFunctions，但 eCAL 根项目以 find_package 消费其安装 config。
build_ecal_cmakefunctions() {
  local ecal_root cmakefunctions_root
  ecal_root="$(source_root_for_dependency ecal)"
  cmakefunctions_root="$ecal_root/thirdparty/cmakefunctions/cmake_functions"
  [[ -d "$cmakefunctions_root" ]] || return 0
  build_cmake_source 'ecal-cmakefunctions' "$cmakefunctions_root"
}

# GitHub tag archive 不含 submodule 内容；eCAL 必须在 configure 前拥有完整锁定源码闭包。
verify_ecal_source_closure() {
  local ecal_root
  ecal_root="$(source_root_for_dependency ecal)"
  local modules="$ecal_root/.gitmodules"
  [[ -f "$modules" ]] || die 'eCAL source closure is incomplete: .gitmodules'
  local required_paths=(
    'thirdparty/asio/asio'
    'thirdparty/ecaludp/ecaludp'
    'thirdparty/protozero/protozero'
    'thirdparty/recycle/recycle'
    'thirdparty/tclap/tclap'
    'thirdparty/tcp_pubsub/tcp_pubsub'
    'thirdparty/yaml-cpp/yaml-cpp'
  )
  local declared_paths=()
  local line submodule_path
  while IFS= read -r line || [[ -n "$line" ]]; do
    if [[ "$line" =~ ^[[:space:]]*path[[:space:]]*=[[:space:]]*(.+)$ ]]; then
      submodule_path="${BASH_REMATCH[1]}"
      [[ "$submodule_path" == thirdparty/* && "$submodule_path" != *".."* ]] || die "eCAL submodule path is invalid: $submodule_path"
      declared_paths+=("$submodule_path")
    fi
  done < "$modules"
  for submodule_path in "${required_paths[@]}"; do
    local declared=false
    local candidate
    for candidate in "${declared_paths[@]}"; do
      [[ "$candidate" == "$submodule_path" ]] && declared=true
    done
    [[ "$declared" == true && -d "$ecal_root/$submodule_path" ]] || die "eCAL source closure is incomplete: $submodule_path"
    compgen -G "$ecal_root/$submodule_path/*" >/dev/null || die "eCAL source closure is incomplete: $submodule_path"
  done
}

# eCAL 的 GitHub archive 只保留空 submodule 目录；从同轮私有 tree 填充固定闭包。
hydrate_ecal_submodules() {
  local ecal_root
  ecal_root="$(source_root_for_dependency ecal)"
  local dependency_names=(ecal-asio ecal-ecaludp ecal-protozero ecal-recycle ecal-tclap ecal-tcp-pubsub ecal-yaml-cpp)
  local target_paths=(thirdparty/asio/asio thirdparty/ecaludp/ecaludp thirdparty/protozero/protozero thirdparty/recycle/recycle thirdparty/tclap/tclap thirdparty/tcp_pubsub/tcp_pubsub thirdparty/yaml-cpp/yaml-cpp)
  local index source_root target_root source_tree
  for index in "${!dependency_names[@]}"; do
    source_tree="$source_work/trees/${dependency_names[$index]}"
    [[ -d "$source_tree" ]] || continue
    source_root="$(source_root_for_dependency "${dependency_names[$index]}")"
    target_root="$ecal_root/${target_paths[$index]}"
    if [[ -e "$target_root" ]]; then
      [[ -d "$target_root" ]] || die "eCAL submodule target is invalid: ${target_paths[$index]}"
      compgen -G "$target_root/*" >/dev/null && die "eCAL submodule target is not empty: ${target_paths[$index]}"
    else
      mkdir -p "$target_root"
    fi
    cp -a "$source_root"/. "$target_root"/
  done
}

source_archive_cache=''
source_archive_manifest=''
dependency_lock=''
source_work=''
build_root=''
install_prefix=''
validation_prefix=''
source_date_epoch=''
source_cache_locks=()
network_evidence=''
materialize_only=false
cmake=''
cc=''
cxx=''

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cmake)
      [[ $# -ge 2 ]] || die '--cmake requires a value'
      cmake="$2"
      shift 2
      ;;
    --cc)
      [[ $# -ge 2 ]] || die '--cc requires a value'
      cc="$2"
      shift 2
      ;;
    --cxx)
      [[ $# -ge 2 ]] || die '--cxx requires a value'
      cxx="$2"
      shift 2
      ;;
    --source-archive-cache)
      [[ $# -ge 2 ]] || die '--source-archive-cache requires a value'
      source_archive_cache="$2"
      shift 2
      ;;
    --source-archive-manifest)
      [[ $# -ge 2 ]] || die '--source-archive-manifest requires a value'
      source_archive_manifest="$2"
      shift 2
      ;;
    --dependency-lock)
      [[ $# -ge 2 ]] || die '--dependency-lock requires a value'
      dependency_lock="$2"
      shift 2
      ;;
    --source-cache-lock)
      [[ $# -ge 2 ]] || die '--source-cache-lock requires a value'
      source_cache_locks+=("$2")
      shift 2
      ;;
    --network-evidence)
      [[ $# -ge 2 ]] || die '--network-evidence requires a value'
      network_evidence="$2"
      shift 2
      ;;
    --source-work)
      [[ $# -ge 2 ]] || die '--source-work requires a value'
      source_work="$2"
      shift 2
      ;;
    --build-root)
      [[ $# -ge 2 ]] || die '--build-root requires a value'
      build_root="$2"
      shift 2
      ;;
    --install-prefix)
      [[ $# -ge 2 ]] || die '--install-prefix requires a value'
      install_prefix="$2"
      shift 2
      ;;
    --validation-prefix)
      [[ $# -ge 2 ]] || die '--validation-prefix requires a value'
      validation_prefix="$2"
      shift 2
      ;;
    --source-date-epoch)
      [[ $# -ge 2 ]] || die '--source-date-epoch requires a value'
      source_date_epoch="$2"
      shift 2
      ;;
    --materialize-only)
      materialize_only=true
      shift
      ;;
    --help)
      usage
      exit 0
      ;;
    *) die "unknown argument: $1" ;;
  esac
done

for pair in \
  "source archive cache:$source_archive_cache" \
  "source work:$source_work" \
  "build root:$build_root" \
  "install prefix:$install_prefix"; do
  label="${pair%%:*}"
  value="${pair#*:}"
  [[ -n "$value" ]] || die "$label is required"
  require_absolute_path "$label" "$value"
done

for source_cache_lock in "${source_cache_locks[@]}"; do
  require_absolute_path 'source cache lock' "$source_cache_lock"
done
[[ "$source_date_epoch" =~ ^[0-9]+$ ]] || die 'source date epoch must be a nonnegative integer'

if [[ -n "$validation_prefix" ]]; then
  require_absolute_path 'validation prefix' "$validation_prefix"
fi

# 这三者都必须是本轮私有 sibling，交叠会让源码、构建和安装相互污染。
output_roots=("$source_work" "$build_root" "$install_prefix")
[[ -z "$validation_prefix" ]] || output_roots+=("$validation_prefix")
for left in "${output_roots[@]}"; do
  for right in "${output_roots[@]}"; do
    [[ "$left" == "$right" ]] && continue
    if is_same_or_nested "$left" "$right"; then
      die 'output roots must be distinct and non-overlapping'
    fi
  done
done

for output_root in "${output_roots[@]}"; do
  [[ ! -e "$output_root" ]] || die 'output roots must be new and absent'
done

for pair in \
  "source archive manifest:$source_archive_manifest" \
  "dependency lock:$dependency_lock"; do
  label="${pair%%:*}"
  value="${pair#*:}"
  [[ -n "$value" ]] || die "$label is required"
  require_absolute_path "$label" "$value"
done

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
cache_lock_arguments=(--lock "$dependency_lock")
for source_cache_lock in "${source_cache_locks[@]}"; do
  cache_lock_arguments+=(--lock "$source_cache_lock")
done
if ! python3 "$repository_root/scripts/verify_stage4_source_cache.py" \
  --manifest "$source_archive_manifest" \
  "${cache_lock_arguments[@]}" \
  --cache-root "$source_archive_cache"; then
  die 'canonical source archive cache verification failed'
fi

validation_source_count="$(python3 - "$repository_root" "$dependency_lock" <<'PY'
from pathlib import Path
import sys

sys.path.insert(0, str(Path(sys.argv[1]) / 'scripts'))
from verify_stage4_dependencies import load_dependency_lock

print(sum('validation' in entry.consumers for entry in load_dependency_lock(Path(sys.argv[2]))))
PY
)"
[[ "$validation_source_count" =~ ^[0-9]+$ ]] || die 'validation source count is invalid'
if [[ "$validation_source_count" -gt 0 && -z "$validation_prefix" && "$materialize_only" != true ]]; then
  die 'validation prefix is required for locked validation sources'
fi

[[ -n "$network_evidence" ]] || die 'network evidence is required before materialization'
require_absolute_path 'network evidence' "$network_evidence"
if ! python3 "$repository_root/scripts/verify_network_isolation.py" \
  --evidence "$network_evidence" --process-pid "$$"; then
  die 'live network isolation verification failed'
fi

if [[ "$materialize_only" == true ]]; then
  python3 "$repository_root/scripts/materialize_stage4_dependency_sources.py" \
    --manifest "$source_archive_manifest" \
    --lock "$dependency_lock" \
    --canonical-cache "$source_archive_cache" \
    --source-work "$source_work" \
    --evidence "$source_work/materialization.json"
  if [[ "$validation_source_count" -gt 0 ]]; then
    python3 "$repository_root/scripts/materialize_stage4_dependency_sources.py" \
      --manifest "$source_archive_manifest" \
      --lock "$dependency_lock" \
      --consumer validation \
      --canonical-cache "$source_archive_cache" \
      --source-work "$source_work/validation" \
      --evidence "$source_work/validation/materialization.json"
  fi
  exit $?
fi

require_tool 'cmake' "$cmake"
require_tool 'cc' "$cc"
require_tool 'cxx' "$cxx"
require_version 'cmake' "$cmake" '^cmake version 3\.28\.[0-9]+'
require_version 'cc' "$cc" '^.* 13\.[0-9]+\.[0-9]+'
require_version 'cxx' "$cxx" '^.* 13\.[0-9]+\.[0-9]+'

export SOURCE_DATE_EPOCH="$source_date_epoch"
python3 "$repository_root/scripts/materialize_stage4_dependency_sources.py" \
  --manifest "$source_archive_manifest" \
  --lock "$dependency_lock" \
  --canonical-cache "$source_archive_cache" \
  --source-work "$source_work" \
  --evidence "$source_work/materialization.json"

if [[ "$validation_source_count" -gt 0 ]]; then
  python3 "$repository_root/scripts/materialize_stage4_dependency_sources.py" \
    --manifest "$source_archive_manifest" \
    --lock "$dependency_lock" \
    --consumer validation \
    --canonical-cache "$source_archive_cache" \
    --source-work "$source_work/validation" \
    --evidence "$source_work/validation/materialization.json"
fi

if [[ -d "$source_work/trees/ecal" ]]; then
  hydrate_ecal_submodules
  verify_ecal_source_closure
  build_ecal_cmakefunctions
fi

dependency_count=0
# MCAP C++ 是由消费者编译的 header-only 源码，不单独配置或安装。
build_order=(abseil-cpp protobuf zstd ecal)
for dependency_name in "${build_order[@]}"; do
  [[ -d "$source_work/trees/$dependency_name" ]] || continue
  tree="$(cmake_source_root_for_dependency "$dependency_name")"
  build_cmake_dependency "$dependency_name"
  dependency_count=$((dependency_count + 1))
done
for tree in "$source_work"/trees/*/*; do
  [[ -d "$tree" ]] || continue
  dependency_name="${tree%/*}"
  dependency_name="${dependency_name##*/}"
  case "$dependency_name" in
    abseil-cpp|protobuf|zstd|mcap|ecal|ecal-asio|ecal-ecaludp|ecal-protozero|ecal-recycle|ecal-tclap|ecal-tcp-pubsub|ecal-yaml-cpp) continue ;;
  esac
  [[ -f "$tree/CMakeLists.txt" ]] || die "offline CMake entry is not implemented for $dependency_name"
  build_cmake_dependency "$dependency_name"
  dependency_count=$((dependency_count + 1))
done
[[ "$dependency_count" -gt 0 ]] || die 'no C++ dependency source tree was materialized'
if [[ "$validation_source_count" -gt 0 ]]; then
  build_pcl_validator
fi
printf 'PASS: %s offline C++ dependencies built\n' "$dependency_count"
