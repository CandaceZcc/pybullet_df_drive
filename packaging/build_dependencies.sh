#!/usr/bin/env bash
# 阶段四离线 C++ 依赖构建入口：先锁定私有输出根关系，再接入 cache/源码/CMake 门禁。
set -euo pipefail

die() {
  printf 'FAIL: %s\n' "$1" >&2
  exit 1
}

usage() {
  printf '%s\n' 'Usage: build_dependencies.sh --source-archive-cache PATH --source-archive-manifest PATH --dependency-lock PATH --source-work PATH --build-root PATH --install-prefix PATH --source-date-epoch EPOCH'
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

source_archive_cache=''
source_archive_manifest=''
dependency_lock=''
source_work=''
build_root=''
install_prefix=''
source_date_epoch=''
source_cache_locks=()
network_evidence=''
materialize_only=false

while [[ $# -gt 0 ]]; do
  case "$1" in
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

# 这三者都必须是本轮私有 sibling，交叠会让源码、构建和安装相互污染。
for left in "$source_work" "$build_root" "$install_prefix"; do
  for right in "$source_work" "$build_root" "$install_prefix"; do
    [[ "$left" == "$right" ]] && continue
    if is_same_or_nested "$left" "$right"; then
      die 'output roots must be distinct and non-overlapping'
    fi
  done
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
  exit $?
fi

die 'offline dependency build is not implemented yet'
