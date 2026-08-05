#!/usr/bin/env bash
# 阶段四网络隔离入口：在独立 user+network namespace 中执行构建命令并写入证据。
set -euo pipefail

fail() {
  printf 'FAIL: %s\n' "$*" >&2
  exit 1
}

run_child() {
  local parent_pid="$1"
  local parent_inode="$2"
  local evidence_dir="$3"
  local argv_sha256="$4"
  shift 4
  [[ "${1:-}" == "--" ]] || fail "network wrapper child command delimiter is missing"
  shift
  [[ "$#" -gt 0 ]] || fail "network wrapper child command is missing"
  [[ -r "/proc/${parent_pid}/ns/net" ]] || fail "network wrapper parent netns is unreadable"

  # 新 netns 默认仅有关闭的 lo；显式启用后由 Python 复核没有其他链路或默认路由。
  ip link set lo up
  command -v python3 >/dev/null 2>&1 || fail "python3 is required for network evidence"

  python3 - "$evidence_dir" "$parent_pid" "$parent_inode" "$argv_sha256" "$@" <<'PY'
import errno
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys


def command_json(*argv: str) -> object:
    return json.loads(subprocess.check_output(argv, text=True))


evidence = Path(sys.argv[1])
parent_pid = int(sys.argv[2])
parent_inode = int(sys.argv[3])
argv_sha256 = sys.argv[4]
command = sys.argv[5:]
child_pid = os.getpid()
child_inode = os.stat("/proc/self/ns/net").st_ino
observed_parent_inode = os.stat(f"/proc/{parent_pid}/ns/net").st_ino
if observed_parent_inode != parent_inode or child_inode == parent_inode:
    raise SystemExit("FAIL: network wrapper netns inode verification failed")

links = command_json("ip", "-j", "link", "show")
if [link.get("ifname") for link in links] != ["lo"]:
    raise SystemExit("FAIL: network wrapper child has non-loopback interfaces")
ipv4_default_routes = command_json("ip", "-j", "route", "show", "default")
ipv6_default_routes = command_json("ip", "-j", "-6", "route", "show", "default")
if ipv4_default_routes or ipv6_default_routes:
    raise SystemExit("FAIL: network wrapper child has a default route")

test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    test_errno = test_socket.connect_ex(("192.0.2.1", 9))
finally:
    test_socket.close()
if test_errno != errno.ENETUNREACH:
    raise SystemExit("FAIL: network wrapper TEST-NET connect did not return ENETUNREACH")

listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    client.connect(listener.getsockname())
    peer, _ = listener.accept()
    peer.close()
finally:
    client.close()
    listener.close()

if evidence.exists() or not evidence.parent.is_dir():
    raise SystemExit("FAIL: network wrapper evidence path is not a new child directory")
evidence.mkdir(mode=0o700)
document = {
    "argv": command,
    "argv_sha256": argv_sha256,
    "child_netns_inode": child_inode,
    "child_pid": child_pid,
    "ipv4_default_routes": ipv4_default_routes,
    "ipv6_default_routes": ipv6_default_routes,
    "links": links,
    "loopback_socket": True,
    "parent_netns_inode": parent_inode,
    "parent_pid": parent_pid,
    "test_net_connect_errno": "ENETUNREACH",
}
evidence_file = evidence / "network-isolation.json"
with evidence_file.open("x", encoding="utf-8") as stream:
    json.dump(document, stream, ensure_ascii=True, indent=2, sort_keys=True)
    stream.write("\n")
    stream.flush()
    os.fsync(stream.fileno())
directory_fd = os.open(evidence, os.O_RDONLY | os.O_DIRECTORY)
try:
    os.fsync(directory_fd)
finally:
    os.close(directory_fd)

environment = os.environ.copy()
environment.update(
    {
        "STAGE4_NETWORK_ISOLATION_EVIDENCE": str(evidence),
        "STAGE4_NETWORK_ISOLATION_PARENT_PID": str(parent_pid),
        "STAGE4_NETWORK_ISOLATION_PARENT_NETNS_INODE": str(parent_inode),
        "STAGE4_NETWORK_ISOLATION_CHILD_NETNS_INODE": str(child_inode),
        "STAGE4_NETWORK_ISOLATION_ARGV_SHA256": argv_sha256,
    }
)
os.execvpe(command[0], command, environment)
PY
}

run_user_parent() {
  local evidence_dir="$1"
  shift
  [[ "${1:-}" == "--" ]] || fail "network wrapper command delimiter is required"
  shift
  [[ "$#" -gt 0 ]] || fail "network wrapper command is required"
  [[ ! -e "$evidence_dir" && -d "$(dirname "$evidence_dir")" ]] || fail "network wrapper evidence path must be absent"
  command -v unshare >/dev/null 2>&1 || fail "unshare is required for network isolation"
  command -v ip >/dev/null 2>&1 || fail "ip is required for network isolation"
  command -v sha256sum >/dev/null 2>&1 || fail "sha256sum is required for network isolation"

  local parent_inode
  parent_inode="$(stat -Lc '%i' /proc/self/ns/net)"
  local argv_sha256
  argv_sha256="$(printf '%s\0' "$@" | sha256sum | awk '{print $1}')"
  # parent 与 child 共享 userns，child 才能读取仍存活 parent 的 netns inode。
  if unshare --net --fork bash "$0" --child "$$" "$parent_inode" "$evidence_dir" "$argv_sha256" -- "$@"; then
    return 0
  else
    return "$?"
  fi
}

run_parent() {
  local evidence_dir=""
  [[ "${1:-}" == "--evidence-dir" ]] || fail "--evidence-dir is required"
  evidence_dir="${2:-}"
  shift 2
  [[ "$evidence_dir" == /* ]] || fail "network wrapper evidence path must be absolute"
  [[ ! -e "$evidence_dir" && -d "$(dirname "$evidence_dir")" ]] || fail "network wrapper evidence path must be absent"
  [[ "${1:-}" == "--" ]] || fail "network wrapper command delimiter is required"
  shift
  [[ "$#" -gt 0 ]] || fail "network wrapper command is required"
  command -v unshare >/dev/null 2>&1 || fail "unshare is required for network isolation"
  # 先建立 userns，再由其内部父进程建立 netns；不能把普通 Bash 父进程直接跨 userns 传给 child。
  if unshare --user --map-root-user --fork bash "$0" --user-parent "$evidence_dir" -- "$@"; then
    return 0
  else
    return "$?"
  fi
}

if [[ "${1:-}" == "--child" ]]; then
  shift
  run_child "$@"
elif [[ "${1:-}" == "--user-parent" ]]; then
  shift
  run_user_parent "$@"
else
  run_parent "$@"
fi
