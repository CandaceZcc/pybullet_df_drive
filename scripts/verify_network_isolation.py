#!/usr/bin/env python3
# 阶段四网络隔离复核器：构建 child 独立验证 wrapper evidence 与实时 netns。
from __future__ import annotations

import argparse
import errno
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys


_FIELDS = {
    "argv",
    "argv_sha256",
    "child_netns_inode",
    "child_pid",
    "ipv4_default_routes",
    "ipv6_default_routes",
    "links",
    "loopback_socket",
    "parent_netns_inode",
    "parent_pid",
    "test_net_connect_errno",
}


def _is_positive_int(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _command_json(*argv: str) -> object:
    """读取 netns 内核状态的 JSON 表示，禁止从文本输出猜测字段。"""
    return json.loads(subprocess.check_output(argv, text=True))


def _argv_sha256(argv: list[str]) -> str:
    """以 wrapper 使用的 NUL 分隔格式重算实际命令 argv 摘要。"""
    digest = hashlib.sha256()
    for value in argv:
        if not isinstance(value, str) or "\0" in value:
            raise ValueError("network evidence argv is invalid")
        digest.update(value.encode("utf-8"))
        digest.update(b"\0")
    return digest.hexdigest()


def _verify_loopback() -> None:
    """确认 child 仍可使用 loopback，而不是通过完全禁网的假证明。"""
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


def verify(evidence_dir: Path, process_pid: int | None = None) -> None:
    """将 evidence 与实时 parent/child netns、链路、路由和 socket 行为逐项比对。"""
    evidence = evidence_dir.resolve()
    if not evidence.is_absolute() or os.environ.get("STAGE4_NETWORK_ISOLATION_EVIDENCE") != str(evidence):
        raise ValueError("network isolation evidence environment differs from argument")
    document = json.loads((evidence / "network-isolation.json").read_text(encoding="utf-8"))
    if not isinstance(document, dict) or set(document) != _FIELDS:
        raise ValueError("network isolation evidence fields are invalid")
    if not _is_positive_int(document["parent_pid"]) or not _is_positive_int(document["child_pid"]):
        raise ValueError("network isolation evidence PIDs are invalid")
    if not _is_positive_int(document["parent_netns_inode"]) or not _is_positive_int(document["child_netns_inode"]):
        raise ValueError("network isolation evidence netns inodes are invalid")
    observed_pid = os.getpid() if process_pid is None else process_pid
    if not _is_positive_int(observed_pid):
        raise ValueError("network isolation process PID is invalid")
    if document["child_pid"] != observed_pid:
        raise ValueError("network isolation child PID differs from evidence")
    child_inode = os.stat(f"/proc/{observed_pid}/ns/net").st_ino
    parent_path = f"/proc/{document['parent_pid']}/ns/net"
    parent_inode = os.stat(parent_path).st_ino
    if (
        child_inode != document["child_netns_inode"]
        or parent_inode != document["parent_netns_inode"]
        or child_inode == parent_inode
    ):
        raise ValueError("network isolation netns inode differs from evidence")
    if not isinstance(document["argv"], list) or document["argv_sha256"] != _argv_sha256(document["argv"]):
        raise ValueError("network isolation argv digest differs from evidence")
    links = _command_json("ip", "-j", "link", "show")
    ipv4_default_routes = _command_json("ip", "-j", "route", "show", "default")
    ipv6_default_routes = _command_json("ip", "-j", "-6", "route", "show", "default")
    if (
        links != document["links"]
        or [link.get("ifname") for link in links] != ["lo"]
        or ipv4_default_routes != []
        or ipv6_default_routes != []
        or ipv4_default_routes != document["ipv4_default_routes"]
        or ipv6_default_routes != document["ipv6_default_routes"]
    ):
        raise ValueError("network isolation link or route state differs from evidence")
    test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        connect_errno = test_socket.connect_ex(("192.0.2.1", 9))
    finally:
        test_socket.close()
    if connect_errno != errno.ENETUNREACH or document["test_net_connect_errno"] != "ENETUNREACH":
        raise ValueError("network isolation TEST-NET result differs from evidence")
    _verify_loopback()
    if document["loopback_socket"] is not True:
        raise ValueError("network isolation loopback result differs from evidence")


def main(argv: list[str] | None = None) -> int:
    """验证当前进程确实位于 wrapper 创建并已证明的隔离 namespace。"""
    parser = argparse.ArgumentParser(description="Verify live stage 4 network isolation.")
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--process-pid", type=int)
    args = parser.parse_args(argv)
    try:
        verify(args.evidence, args.process_pid)
    except (OSError, ValueError, json.JSONDecodeError, subprocess.CalledProcessError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print("PASS: live network isolation verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
