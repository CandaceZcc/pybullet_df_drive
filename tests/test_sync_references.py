"""参考仓库同步工具测试：验证固定提交检查不会访问网络或改写 checkout。"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import os
import subprocess
import sys

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SYNC_SCRIPT = ROOT / "scripts" / "sync_references.py"


def _stage4_record(
    *,
    url: str,
    commit: str,
    license_files: tuple[str, ...] = ("LICENSE.txt",),
    focus: tuple[str, ...] = ("src/example.cpp",),
) -> dict[str, object]:
    """构造满足阶段四准入规则的最小参考仓库记录。"""
    return {
        "name": "fixture",
        "stage": 4,
        "url": url,
        "branch": "main",
        "commit": commit,
        "license": "MIT",
        "license_scope": "first_party",
        "license_files": list(license_files),
        "stars": 1,
        "stars_observed_at": "2026-07-31T07:03:51Z",
        "purpose": "Stage four reference fixture.",
        "focus": list(focus),
    }


def _run(
    *args: str,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    """运行测试 Git/同步命令并保留可诊断输出。"""
    return subprocess.run(
        args,
        cwd=cwd,
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )


def _create_reference_checkout(root: Path, name: str, remote_url: str) -> tuple[Path, str]:
    """创建带固定 HEAD、无效远端和阶段四 focus 文件的本地 checkout。"""
    target = root / name
    target.mkdir(parents=True)
    assert _run("git", "init", "-q", str(target)).returncode == 0
    assert _run("git", "config", "user.name", "Reference Test", cwd=target).returncode == 0
    assert _run(
        "git", "config", "user.email", "reference@example.invalid", cwd=target
    ).returncode == 0
    focus = target / "src" / "example.cpp"
    focus.parent.mkdir()
    focus.write_text("int main() { return 0; }\n", encoding="utf-8")
    (target / "LICENSE.txt").write_text("fixture license\n", encoding="utf-8")
    assert _run(
        "git", "add", "src/example.cpp", "LICENSE.txt", cwd=target
    ).returncode == 0
    assert _run("git", "commit", "-q", "-m", "fixture", cwd=target).returncode == 0
    assert _run("git", "remote", "add", "origin", remote_url, cwd=target).returncode == 0
    head = _run("git", "rev-parse", "HEAD", cwd=target)
    assert head.returncode == 0
    return target, head.stdout.strip()


def test_check_is_read_only_and_validates_stage4_focus(tmp_path: Path) -> None:
    """--check 只读取本地 Git 元数据，即使 origin 不可达也能完成核对。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    status_before = _run("git", "status", "--porcelain=v1", cwd=target).stdout

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode == 0, result.stderr
    assert "fixture (main)" in result.stdout
    assert "src/example.cpp" in result.stdout
    assert not (target / ".git" / "FETCH_HEAD").exists()
    assert _run("git", "rev-parse", "HEAD", cwd=target).stdout.strip() == commit
    assert _run("git", "status", "--porcelain=v1", cwd=target).stdout == status_before


def test_shell_check_delegates_without_running_mutating_git(tmp_path: Path) -> None:
    """公开 shell 入口在 --check 下不得落入 fetch/checkout 同步分支。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    _, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_git = fake_bin / "git"
    fake_git.write_text(
        """#!/usr/bin/env bash
for argument in "$@"; do
  case "$argument" in
    fetch|checkout|init) exit 97 ;;
  esac
done
exec /usr/bin/git "$@"
""",
        encoding="utf-8",
    )
    fake_git.chmod(0o755)
    env = os.environ.copy()
    env["PATH"] = f"{fake_bin}:{env['PATH']}"

    result = _run(
        "bash",
        str(ROOT / "scripts" / "sync_references.sh"),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "fixture (main)" in result.stdout


def test_check_rejects_missing_stage4_license_file(tmp_path: Path) -> None:
    """阶段四声明的许可证证据必须在同一固定 checkout 中真实存在。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    _, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "repositories": [
                    _stage4_record(
                        url=remote_url,
                        commit=commit,
                        license_files=("LICENSE.txt", "MISSING-NOTICE.txt"),
                    )
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "MISSING-NOTICE.txt" in result.stderr


def test_sync_checks_out_pinned_commit_after_branch_moves(tmp_path: Path) -> None:
    """同步按 manifest SHA fetch，不把已经前移的 branch HEAD 当成固定快照。"""
    source = tmp_path / "source"
    source.mkdir()
    assert _run("git", "init", "-q", "-b", "main", str(source)).returncode == 0
    assert _run("git", "config", "user.name", "Reference Test", cwd=source).returncode == 0
    assert _run(
        "git", "config", "user.email", "reference@example.invalid", cwd=source
    ).returncode == 0
    (source / "LICENSE.txt").write_text("fixture license\n", encoding="utf-8")
    focus = source / "src" / "example.cpp"
    focus.parent.mkdir()
    focus.write_text("int old_snapshot = 1;\n", encoding="utf-8")
    assert _run("git", "add", ".", cwd=source).returncode == 0
    assert _run("git", "commit", "-q", "-m", "old", cwd=source).returncode == 0
    old_commit = _run("git", "rev-parse", "HEAD", cwd=source).stdout.strip()
    focus.write_text("int moved_branch = 2;\n", encoding="utf-8")
    assert _run("git", "commit", "-q", "-am", "new", cwd=source).returncode == 0
    new_commit = _run("git", "rev-parse", "HEAD", cwd=source).stdout.strip()
    assert old_commit != new_commit

    repo_root = tmp_path / "repos"
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "repositories": [
                    {
                        "name": "fixture",
                        "url": str(source),
                        "branch": "main",
                        "commit": old_commit,
                        "purpose": "Legacy synchronization fixture.",
                        "focus": ["src/example.cpp"],
                    }
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    fake_bin = tmp_path / "guard-bin"
    fake_bin.mkdir()
    fake_git = fake_bin / "git"
    fake_git.write_text(
        """#!/usr/bin/env bash
case " $* " in
  *" $REFERENCE_TEST_REPO_DIR/"*) exec /usr/bin/git "$@" ;;
esac
for argument in "$@"; do
  case "$argument" in
    fetch|checkout|init) exit 97 ;;
  esac
done
exec /usr/bin/git "$@"
""",
        encoding="utf-8",
    )
    fake_git.chmod(0o755)
    env = os.environ.copy()
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    env["REFERENCE_TEST_REPO_DIR"] = str(repo_root)

    result = _run(
        "bash",
        str(ROOT / "scripts" / "sync_references.sh"),
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
        env=env,
    )

    assert result.returncode == 0, result.stderr
    target = repo_root / "fixture"
    assert _run("git", "rev-parse", "HEAD", cwd=target).stdout.strip() == old_commit
    assert (target / "src" / "example.cpp").read_text(encoding="utf-8") == (
        "int old_snapshot = 1;\n"
    )


@pytest.mark.parametrize("dirty_kind", ["modified", "untracked"])
def test_check_rejects_dirty_checkout(tmp_path: Path, dirty_kind: str) -> None:
    """固定 HEAD 仍不足够，modified/untracked 内容都必须使检查失败。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    if dirty_kind == "modified":
        (target / "src" / "example.cpp").write_text(
            "int tampered = 1;\n", encoding="utf-8"
        )
    else:
        (target / "UNTRACKED.txt").write_text("untracked\n", encoding="utf-8")

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "dirty reference checkout" in result.stderr


@pytest.mark.parametrize("index_flag", ["assume-unchanged", "skip-worktree"])
def test_check_rejects_hidden_tracked_changes(
    tmp_path: Path, index_flag: str
) -> None:
    """索引优化位不能隐藏阶段四 checkout 中被篡改的 tracked bytes。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    focus = target / "src" / "example.cpp"
    assert _run(
        "git", "update-index", f"--{index_flag}", "src/example.cpp", cwd=target
    ).returncode == 0
    focus.write_text("int hidden_tamper = 1;\n", encoding="utf-8")
    assert _run("git", "status", "--porcelain=v1", cwd=target).stdout == ""

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "unsafe reference index flag" in result.stderr
    assert "src/example.cpp" in result.stderr


def test_sync_rejects_dirty_checkout_before_fetch(tmp_path: Path) -> None:
    """同步不得用 fetch/checkout 隐式覆盖或保留调用者的本地修改。"""
    repo_root = tmp_path / "repos"
    missing_remote = tmp_path / "missing-remote.git"
    target, commit = _create_reference_checkout(
        repo_root, "fixture", str(missing_remote)
    )
    (target / "src" / "example.cpp").write_text(
        "int local_change = 1;\n", encoding="utf-8"
    )
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "repositories": [
                    {
                        "name": "fixture",
                        "url": str(missing_remote),
                        "branch": "main",
                        "commit": commit,
                    }
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "dirty reference checkout" in result.stderr
    assert "fetch" not in result.stderr.lower()


def test_sync_rejects_checkout_symlink_without_touching_external_repo(
    tmp_path: Path,
) -> None:
    """同步目标不能用 symlink 把 fetch/checkout 重定向到根外仓库。"""
    external_root = tmp_path / "external"
    remote = tmp_path / "remote.git"
    external, commit = _create_reference_checkout(
        external_root, "fixture", str(remote)
    )
    assert (
        _run("git", "clone", "-q", "--bare", str(external), str(remote)).returncode
        == 0
    )
    repo_root = tmp_path / "repos"
    repo_root.mkdir()
    (repo_root / "fixture").symlink_to(external, target_is_directory=True)
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "repositories": [
                    {
                        "name": "fixture",
                        "url": str(remote),
                        "branch": "main",
                        "commit": commit,
                    }
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    status_before = _run("git", "status", "--porcelain=v1", cwd=external).stdout
    head_before = _run("git", "rev-parse", "HEAD", cwd=external).stdout
    assert not (external / ".git" / "FETCH_HEAD").exists()

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "unsafe reference checkout path" in result.stderr
    assert _run("git", "status", "--porcelain=v1", cwd=external).stdout == status_before
    assert _run("git", "rev-parse", "HEAD", cwd=external).stdout == head_before
    assert not (external / ".git" / "FETCH_HEAD").exists()


def test_check_rejects_repo_root_symlink_before_any_git_query(
    tmp_path: Path,
) -> None:
    """checkout 根本身不能用 symlink 把 Git 查询重定向到声明根外。"""
    external_root = tmp_path / "external-repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    _, commit = _create_reference_checkout(external_root, "fixture", remote_url)
    repo_root = tmp_path / "repos"
    repo_root.symlink_to(external_root, target_is_directory=True)
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    git_marker = tmp_path / "git-called"
    fake_bin = tmp_path / "guard-bin"
    fake_bin.mkdir()
    fake_git = fake_bin / "git"
    fake_git.write_text(
        """#!/usr/bin/env bash
: > "$REFERENCE_TEST_GIT_MARKER"
exit 97
""",
        encoding="utf-8",
    )
    fake_git.chmod(0o755)
    env = os.environ.copy()
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    env["REFERENCE_TEST_GIT_MARKER"] = str(git_marker)

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
        env=env,
    )

    assert result.returncode != 0
    assert "unsafe reference checkout path" in result.stderr
    assert not git_marker.exists()


@pytest.mark.parametrize("metadata_name", ["objects", "refs"])
def test_check_rejects_internal_git_metadata_symlink_before_any_git_query(
    tmp_path: Path,
    metadata_name: str,
) -> None:
    """Git 可写元数据目录不能通过 symlink 逃逸 checkout。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    metadata_path = target / ".git" / metadata_name
    external_metadata = tmp_path / f"external-{metadata_name}"
    metadata_path.rename(external_metadata)
    metadata_path.symlink_to(external_metadata, target_is_directory=True)
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    git_marker = tmp_path / "git-called"
    fake_bin = tmp_path / "guard-bin"
    fake_bin.mkdir()
    fake_git = fake_bin / "git"
    fake_git.write_text(
        """#!/usr/bin/env bash
: > "$REFERENCE_TEST_GIT_MARKER"
exit 97
""",
        encoding="utf-8",
    )
    fake_git.chmod(0o755)
    env = os.environ.copy()
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    env["REFERENCE_TEST_GIT_MARKER"] = str(git_marker)

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
        env=env,
    )

    assert result.returncode != 0
    assert "unsafe reference checkout path" in result.stderr
    assert not git_marker.exists()


def test_check_rejects_repo_root_symlink_ancestor_before_any_git_query(
    tmp_path: Path,
) -> None:
    """checkout 根的任一祖先也不能通过 symlink 改写声明位置。"""
    external_parent = tmp_path / "external-parent"
    external_root = external_parent / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    _, commit = _create_reference_checkout(external_root, "fixture", remote_url)
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(external_parent, target_is_directory=True)
    repo_root = linked_parent / "repos"
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    git_marker = tmp_path / "git-called"
    fake_bin = tmp_path / "guard-bin"
    fake_bin.mkdir()
    fake_git = fake_bin / "git"
    fake_git.write_text(
        """#!/usr/bin/env bash
: > "$REFERENCE_TEST_GIT_MARKER"
exit 97
""",
        encoding="utf-8",
    )
    fake_git.chmod(0o755)
    env = os.environ.copy()
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    env["REFERENCE_TEST_GIT_MARKER"] = str(git_marker)

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
        env=env,
    )

    assert result.returncode != 0
    assert "unsafe reference checkout path" in result.stderr
    assert not git_marker.exists()


def test_sync_rejects_repo_root_symlink_ancestor_before_creating_directory(
    tmp_path: Path,
) -> None:
    """同步不能先经 symlink 祖先在声明根外创建 repo_dir。"""
    external_parent = tmp_path / "external-parent"
    external_parent.mkdir()
    linked_parent = tmp_path / "linked-parent"
    linked_parent.symlink_to(external_parent, target_is_directory=True)
    repo_root = linked_parent / "repos"
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "repositories": [
                    {
                        "name": "fixture",
                        "url": str(tmp_path / "missing-remote.git"),
                        "branch": "main",
                        "commit": "0" * 40,
                    }
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "unsafe reference checkout path" in result.stderr
    assert not (external_parent / "repos").exists()


def test_check_rejects_core_worktree_outside_checkout(tmp_path: Path) -> None:
    """普通 Git 配置不能把被核对的工作树重定向到 checkout 外。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    external_worktree = tmp_path / "external-worktree"
    external_focus = external_worktree / "src" / "example.cpp"
    external_focus.parent.mkdir(parents=True)
    external_focus.write_bytes((target / "src" / "example.cpp").read_bytes())
    (external_worktree / "LICENSE.txt").write_bytes(
        (target / "LICENSE.txt").read_bytes()
    )
    assert _run(
        "git",
        "config",
        "core.worktree",
        str(external_worktree),
        cwd=target,
    ).returncode == 0
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "unsafe reference Git layout" in result.stderr


def test_check_rejects_external_git_common_directory(tmp_path: Path) -> None:
    """commondir 不能让 refs、objects 或配置落到 checkout 外。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    git_dir = target / ".git"
    external_common = tmp_path / "external-common"
    external_common.mkdir()
    for name in ("config", "hooks", "info", "logs", "objects", "refs"):
        source = git_dir / name
        if source.exists():
            source.rename(external_common / name)
    (git_dir / "commondir").write_text(f"{external_common}\n", encoding="utf-8")
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "unsafe reference Git layout" in result.stderr


def test_check_rejects_external_object_alternates(tmp_path: Path) -> None:
    """普通 alternates 文件也不能让固定 commit 依赖 checkout 外对象。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    objects = target / ".git" / "objects"
    external_objects = tmp_path / "external-objects"
    objects.rename(external_objects)
    (objects / "info").mkdir(parents=True)
    (objects / "pack").mkdir()
    (objects / "info" / "alternates").write_text(
        f"{external_objects}\n", encoding="utf-8"
    )
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "unsafe reference Git object store" in result.stderr


def test_check_ignores_inherited_git_repository_environment(tmp_path: Path) -> None:
    """继承的 GIT_DIR/GIT_WORK_TREE 不能覆盖 manifest 指定 checkout。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    _, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    external_root = tmp_path / "external-repos"
    external, _ = _create_reference_checkout(external_root, "fixture", remote_url)
    (external / "src" / "example.cpp").write_text(
        "int external_commit = 2;\n", encoding="utf-8"
    )
    assert _run(
        "git", "commit", "-q", "-am", "external", cwd=external
    ).returncode == 0
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    env = os.environ.copy()
    env["GIT_DIR"] = str(external / ".git")
    env["GIT_WORK_TREE"] = str(external)

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
        env=env,
    )

    assert result.returncode == 0, result.stderr
    assert "fixture (main)" in result.stdout


def test_check_rejects_executable_local_filter_before_it_runs(tmp_path: Path) -> None:
    """local Git config 中的 filter 命令必须在 status 前被拒绝。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, _ = _create_reference_checkout(repo_root, "fixture", remote_url)
    (target / ".gitattributes").write_text(
        "*.cpp filter=reference-test\n", encoding="utf-8"
    )
    assert _run("git", "add", ".gitattributes", cwd=target).returncode == 0
    assert _run(
        "git", "commit", "-q", "-m", "attributes", cwd=target
    ).returncode == 0
    commit = _run("git", "rev-parse", "HEAD", cwd=target).stdout.strip()
    filter_marker = tmp_path / "filter-called"
    filter_script = tmp_path / "filter-command"
    filter_script.write_text(
        """#!/usr/bin/env bash
: > "$REFERENCE_TEST_FILTER_MARKER"
cat
""",
        encoding="utf-8",
    )
    filter_script.chmod(0o755)
    assert _run(
        "git",
        "config",
        "filter.reference-test.clean",
        str(filter_script),
        cwd=target,
    ).returncode == 0
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {"repositories": [_stage4_record(url=remote_url, commit=commit)]},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    env = os.environ.copy()
    env["REFERENCE_TEST_FILTER_MARKER"] = str(filter_marker)

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
        env=env,
    )

    assert result.returncode != 0
    assert "unsafe reference Git config" in result.stderr
    assert not filter_marker.exists()


def test_check_rejects_replace_ref_for_manifest_commit(tmp_path: Path) -> None:
    """replace ref 不能让固定 SHA 解析成另一份 commit/tree。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, manifest_commit = _create_reference_checkout(
        repo_root, "fixture", remote_url
    )
    focus = target / "src" / "example.cpp"
    focus.write_text("int replacement_tree = 2;\n", encoding="utf-8")
    assert _run(
        "git", "commit", "-q", "-am", "replacement", cwd=target
    ).returncode == 0
    replacement_commit = _run(
        "git", "rev-parse", "HEAD", cwd=target
    ).stdout.strip()
    assert replacement_commit != manifest_commit
    assert _run(
        "git", "checkout", "-q", "--detach", manifest_commit, cwd=target
    ).returncode == 0
    assert _run(
        "git", "replace", manifest_commit, replacement_commit, cwd=target
    ).returncode == 0
    assert _run("git", "reset", "-q", "--hard", "HEAD", cwd=target).returncode == 0
    assert _run("git", "rev-parse", "HEAD", cwd=target).stdout.strip() == manifest_commit
    assert focus.read_text(encoding="utf-8") == "int replacement_tree = 2;\n"
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "repositories": [
                    _stage4_record(url=remote_url, commit=manifest_commit)
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "unsafe reference Git replacement metadata" in result.stderr


def test_sync_rejects_ignored_sentinel_before_checkout_overwrite(
    tmp_path: Path,
) -> None:
    """同步不得让 checkout 覆盖当前 HEAD 忽略的本地 sentinel。"""
    repo_root = tmp_path / "repos"
    remote = tmp_path / "remote.git"
    target, _ = _create_reference_checkout(repo_root, "fixture", str(remote))
    (target / ".gitignore").write_text("ignored-sentinel.txt\n", encoding="utf-8")
    assert _run("git", "add", ".gitignore", cwd=target).returncode == 0
    assert _run(
        "git", "commit", "-q", "-m", "ignore sentinel", cwd=target
    ).returncode == 0
    old_commit = _run("git", "rev-parse", "HEAD", cwd=target).stdout.strip()
    sentinel = target / "ignored-sentinel.txt"
    sentinel.write_text("remote bytes\n", encoding="utf-8")
    assert _run("git", "add", "-f", sentinel.name, cwd=target).returncode == 0
    assert _run(
        "git", "commit", "-q", "-m", "track sentinel", cwd=target
    ).returncode == 0
    new_commit = _run("git", "rev-parse", "HEAD", cwd=target).stdout.strip()
    assert new_commit != old_commit
    assert _run("git", "clone", "-q", "--bare", str(target), str(remote)).returncode == 0
    assert _run("git", "reset", "-q", "--hard", old_commit, cwd=target).returncode == 0
    sentinel.write_text("local sentinel\n", encoding="utf-8")
    assert _run("git", "status", "--porcelain=v1", cwd=target).stdout == ""
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "repositories": [
                    {
                        "name": "fixture",
                        "url": str(remote),
                        "branch": "main",
                        "commit": new_commit,
                    }
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "ignored reference checkout content" in result.stderr
    assert sentinel.read_text(encoding="utf-8") == "local sentinel\n"


def test_check_rejects_partial_clone_before_lazy_fetch(tmp_path: Path) -> None:
    """--check 必须在缺失对象触发 promisor lazy fetch 前拒绝 partial clone。"""
    repo_root = tmp_path / "repos"
    remote = tmp_path / "remote.git"
    target, commit = _create_reference_checkout(repo_root, "fixture", str(remote))
    assert _run("git", "clone", "-q", "--bare", str(target), str(remote)).returncode == 0
    assert _run(
        "git", "config", "core.repositoryFormatVersion", "1", cwd=target
    ).returncode == 0
    assert _run(
        "git", "config", "extensions.partialClone", "origin", cwd=target
    ).returncode == 0
    assert _run(
        "git", "config", "remote.origin.promisor", "true", cwd=target
    ).returncode == 0
    assert _run(
        "git", "config", "remote.origin.partialCloneFilter", "blob:none", cwd=target
    ).returncode == 0
    blob = _run("git", "hash-object", "src/example.cpp", cwd=target).stdout.strip()
    blob_path = target / ".git" / "objects" / blob[:2] / blob[2:]
    assert blob_path.is_file()
    blob_path.unlink()
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "repositories": [
                    {
                        "name": "fixture",
                        "url": str(remote),
                        "branch": "main",
                        "commit": commit,
                    }
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "unsafe reference Git config" in result.stderr
    assert not blob_path.exists()


def test_check_rejects_ignored_path_before_focus_validation(tmp_path: Path) -> None:
    """工作树中的 ignored 文件不能作为声明 focus 绕过 dirty gate。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, commit = _create_reference_checkout(repo_root, "fixture", remote_url)
    ignored_focus = target / "generated" / "ignored.cpp"
    ignored_focus.parent.mkdir()
    ignored_focus.write_text("int ignored = 1;\n", encoding="utf-8")
    (target / ".git" / "info" / "exclude").write_text(
        "generated/\n", encoding="utf-8"
    )
    assert _run("git", "status", "--porcelain=v1", cwd=target).stdout == ""
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "repositories": [
                    _stage4_record(
                        url=remote_url,
                        commit=commit,
                        focus=("generated/ignored.cpp",),
                    )
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "ignored reference checkout content" in result.stderr
    assert "generated/ignored.cpp" in result.stderr


def test_check_rejects_tracked_focus_symlink_outside_checkout(tmp_path: Path) -> None:
    """固定 commit 中的 symlink 也不能让 focus 校验读取 checkout 根外内容。"""
    repo_root = tmp_path / "repos"
    remote_url = "https://github.com/example/reference-fixture.git"
    target, _ = _create_reference_checkout(repo_root, "fixture", remote_url)
    outside = tmp_path / "outside.cpp"
    outside.write_text("int outside = 1;\n", encoding="utf-8")
    linked_focus = target / "src" / "linked.cpp"
    linked_focus.symlink_to(outside)
    assert _run("git", "add", "src/linked.cpp", cwd=target).returncode == 0
    assert _run(
        "git", "commit", "-q", "-m", "tracked focus symlink", cwd=target
    ).returncode == 0
    commit = _run("git", "rev-parse", "HEAD", cwd=target).stdout.strip()
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "repositories": [
                    _stage4_record(
                        url=remote_url,
                        commit=commit,
                        focus=("src/linked.cpp",),
                    )
                ]
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    outside_before = outside.read_bytes()

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(repo_root),
    )

    assert result.returncode != 0
    assert "unsafe reference path symlink" in result.stderr
    assert outside.read_bytes() == outside_before


@pytest.mark.parametrize(
    ("case", "expected_error"),
    [
        ("license", "reference license must be a nonempty string"),
        ("license_scope", "stage 4 reference license_scope must equal first_party"),
        ("license_files", "must declare nonempty license_files"),
        ("stars", "reference stars must be a nonnegative integer"),
        ("stars_observed_at", "reference stars_observed_at must be a UTC timestamp"),
        ("purpose", "reference purpose must be a nonempty string"),
        ("focus", "stage 4 reference must declare nonempty focus"),
        ("url", "stage 4 reference URL must be an official GitHub HTTPS URL"),
    ],
)
def test_stage4_manifest_rejects_incomplete_admission_schema(
    tmp_path: Path,
    case: str,
    expected_error: str,
) -> None:
    """阶段四记录缺少任一准入证据时必须在访问 checkout 前失败。"""
    record = _stage4_record(
        url="https://github.com/example/reference-fixture.git",
        commit="0" * 40,
    )
    if case == "license_files":
        record.pop("license_files")
        record["third_party_license_files"] = ["NOTICE.txt"]
    elif case == "focus":
        record["focus"] = []
    elif case == "url":
        record["url"] = "https://example.invalid/reference-fixture.git"
    else:
        record.pop(case)
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump({"repositories": [record]}, sort_keys=False),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(tmp_path / "missing-repos"),
    )

    assert result.returncode != 0
    assert expected_error in result.stderr
    assert "missing reference checkout" not in result.stderr


@pytest.mark.parametrize("license_scope", ["whole_archive", "First_Party", ""])
def test_stage4_manifest_rejects_invalid_license_scope(
    tmp_path: Path, license_scope: str
) -> None:
    """阶段四许可证范围只能是精确的 first_party 枚举值。"""
    record = _stage4_record(
        url="https://github.com/example/reference-fixture.git",
        commit="0" * 40,
    )
    record["license_scope"] = license_scope
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump({"repositories": [record]}, sort_keys=False),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(tmp_path / "missing-repos"),
    )

    assert result.returncode != 0
    assert "stage 4 reference license_scope must equal first_party" in result.stderr
    assert "missing reference checkout" not in result.stderr


def test_manifest_rejects_duplicate_repository_name(tmp_path: Path) -> None:
    """同一 checkout 名称只能在 manifest 中声明一次。"""
    record = _stage4_record(
        url="https://github.com/example/reference-fixture.git",
        commit="0" * 40,
    )
    manifest = tmp_path / "manifest.yml"
    manifest.write_text(
        yaml.safe_dump({"repositories": [record, record]}, sort_keys=False),
        encoding="utf-8",
    )

    result = _run(
        sys.executable,
        str(SYNC_SCRIPT),
        "--check",
        "--manifest",
        str(manifest),
        "--repo-dir",
        str(tmp_path / "missing-repos"),
    )

    assert result.returncode != 0
    assert "duplicate reference name: fixture" in result.stderr
    assert "missing reference checkout" not in result.stderr


def test_real_manifest_satisfies_stage4_admission_schema() -> None:
    """真实 manifest 必须持续通过生产解析器，而不只让临时 fixture 通过。"""
    spec = importlib.util.spec_from_file_location("sync_references_test", SYNC_SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
        repositories = module.load_manifest(ROOT / "references" / "manifest.yml")
    finally:
        sys.modules.pop(spec.name, None)

    stage4_names = {repository.name for repository in repositories if repository.stage == 4}
    assert stage4_names == {
        "ecal",
        "protobuf",
        "mcap",
        "zstd",
        "livox_ros_driver2",
        "Livox-SDK2",
        "pcl",
    }
