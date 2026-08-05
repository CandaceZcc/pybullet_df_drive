# 阶段四网络隔离合同：构建入口必须在可复核的独立 netns 内运行。
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
WRAPPER = ROOT / "packaging" / "run_network_isolated.sh"
VERIFIER = ROOT / "scripts" / "verify_network_isolation.py"


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
