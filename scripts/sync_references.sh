#!/usr/bin/env bash
# 参考仓库入口：由 manifest 同步固定 SHA，或用 --check 做离线只读核对。
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "${PYTHON:-python3}" "$ROOT_DIR/scripts/sync_references.py" "$@"
