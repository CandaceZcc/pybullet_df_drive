#!/usr/bin/env bash
# 同步参考仓库脚本：把 PLAN.md 中的外部参考项目浅克隆到 references/repos/。
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_DIR="$ROOT_DIR/references/repos"
mkdir -p "$REPO_DIR"

sync_repo() {
  # 按固定 commit 检出参考仓库，保证不同时间运行得到同一份参考代码。
  local name="$1"
  local url="$2"
  local branch="$3"
  local commit="$4"
  local target="$REPO_DIR/$name"

  if [[ -d "$target/.git" ]]; then
    # 已存在时只抓取目标 commit，避免重复下载完整仓库。
    git -C "$target" fetch --depth 1 origin "$commit"
  else
    git clone --depth 1 --branch "$branch" "$url" "$target"
  fi
  git -C "$target" checkout --detach "$commit"
  echo "$name -> $commit"
}

sync_repo "pybullet_diffdrive" "https://github.com/thedeepestreality/pybullet_diffdrive.git" "main" "f6582502d0e1b0345951f0f825ec2c4e9d25ba4d"
sync_repo "PythonRobotics" "https://github.com/AtsushiSakai/PythonRobotics.git" "master" "b38c510e083d69a5755d98d0680bd50f3d9a91fa"
sync_repo "PybulletRobotics" "https://github.com/akinami3/PybulletRobotics.git" "main" "3dea8e5eaeb89c49015dc0cdda09167d9f9ada5e"
sync_repo "bullet3" "https://github.com/bulletphysics/bullet3.git" "master" "63c4d67e337017f9d8b298c900e9aabdb69296e7"
sync_repo "Two-Wheel-Robot-DeepRL" "https://github.com/ngzhili/Two-Wheel-Robot-DeepRL.git" "main" "4e603a4c2d2d1618b9d8c4d571fdfc4888668d78"
