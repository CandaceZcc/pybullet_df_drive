#!/usr/bin/env python3
"""参考仓库同步入口：按 manifest 只读核对固定 checkout 或执行同步。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import re
import stat
import subprocess
from typing import Any
from urllib.parse import urlsplit

import yaml


COMMIT_PATTERN = re.compile(r"[0-9a-f]{40}")
GITHUB_REPOSITORY_PATH_PATTERN = re.compile(r"/[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+\.git")
GIT_SAFETY_OPTIONS = (
    "--no-replace-objects",
    "-c",
    f"core.hooksPath={os.devnull}",
    "-c",
    "core.fsmonitor=false",
    "-c",
    "gc.auto=0",
    "-c",
    "maintenance.auto=false",
)
SAFE_LOCAL_GIT_CONFIG_KEYS = frozenset(
    {
        "core.bare",
        "core.filemode",
        "core.logallrefupdates",
        "core.repositoryformatversion",
        "remote.origin.fetch",
        "remote.origin.url",
        "user.email",
        "user.name",
    }
)


@dataclass(frozen=True)
class ReferenceRepository:
    """保存只读检查所需的单个参考仓库合同。"""

    name: str
    url: str
    branch: str
    commit: str
    stage: int | None
    focus: tuple[str, ...]
    license_files: tuple[str, ...]


def _require_text(record: dict[str, Any], field: str) -> str:
    """读取非空字符串字段，避免把 YAML 标量隐式转成路径。"""
    value = record.get(field)
    if not isinstance(value, str) or not value:
        raise ValueError(f"reference {field} must be a nonempty string")
    return value


def _string_list(record: dict[str, Any], field: str) -> tuple[str, ...]:
    """读取字符串列表，并拒绝空路径或错误 YAML 类型。"""
    value = record.get(field, [])
    if not isinstance(value, list) or not all(
        isinstance(item, str) and item for item in value
    ):
        raise ValueError(f"reference {field} must be a list of nonempty strings")
    return tuple(value)


def _require_stage4_stars(record: dict[str, Any]) -> None:
    """阶段四 Star 观测值必须是非负整数，布尔值不能冒充整数。"""
    value = record.get("stars")
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("reference stars must be a nonnegative integer")


def _require_stage4_observation_time(record: dict[str, Any]) -> None:
    """阶段四成熟度观测固定为可复核的 UTC 秒级时间。"""
    value = record.get("stars_observed_at")
    if not isinstance(value, str):
        raise ValueError("reference stars_observed_at must be a UTC timestamp")
    try:
        datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ")
    except ValueError as error:
        raise ValueError(
            "reference stars_observed_at must be a UTC timestamp"
        ) from error


def _require_stage4_github_url(url: str) -> None:
    """阶段四只接受规范化的 GitHub HTTPS 仓库地址。"""
    parsed = urlsplit(url)
    if (
        parsed.scheme != "https"
        or parsed.netloc != "github.com"
        or parsed.query
        or parsed.fragment
        or GITHUB_REPOSITORY_PATH_PATTERN.fullmatch(parsed.path) is None
    ):
        raise ValueError(
            "stage 4 reference URL must be an official GitHub HTTPS URL"
        )


def load_manifest(path: Path) -> tuple[ReferenceRepository, ...]:
    """用 YAML parser 加载并校验同步所需的 manifest 字段。"""
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(
        document.get("repositories"), list
    ):
        raise ValueError("reference manifest must contain a repositories list")

    repositories: list[ReferenceRepository] = []
    repository_names: set[str] = set()
    for raw in document["repositories"]:
        if not isinstance(raw, dict):
            raise ValueError("reference repository record must be a mapping")
        name = _require_text(raw, "name")
        if Path(name).name != name or name in {".", ".."}:
            raise ValueError(f"reference name must be one path component: {name}")
        if name in repository_names:
            raise ValueError(f"duplicate reference name: {name}")
        repository_names.add(name)
        commit = _require_text(raw, "commit")
        if COMMIT_PATTERN.fullmatch(commit) is None:
            raise ValueError("reference commit must be a 40-character lowercase SHA")
        url = _require_text(raw, "url")
        focus = _string_list(raw, "focus")
        stage = raw.get("stage")
        if stage is not None and (isinstance(stage, bool) or not isinstance(stage, int)):
            raise ValueError("reference stage must be an integer")
        first_party_license_files = _string_list(raw, "license_files")
        third_party_license_files = _string_list(raw, "third_party_license_files")
        if stage == 4:
            _require_text(raw, "license")
            if raw.get("license_scope") != "first_party":
                raise ValueError(
                    "stage 4 reference license_scope must equal first_party"
                )
            _require_text(raw, "purpose")
            _require_stage4_stars(raw)
            _require_stage4_observation_time(raw)
            _require_stage4_github_url(url)
            if not first_party_license_files:
                raise ValueError(
                    "stage 4 reference must declare nonempty license_files"
                )
            if not focus:
                raise ValueError("stage 4 reference must declare nonempty focus")
        license_files = first_party_license_files + third_party_license_files
        repositories.append(
            ReferenceRepository(
                name=name,
                url=url,
                branch=_require_text(raw, "branch"),
                commit=commit,
                stage=stage,
                focus=focus,
                license_files=license_files,
            )
        )
    return tuple(repositories)


def _git_environment() -> dict[str, str]:
    """清除可覆盖仓库位置的 Git 环境，并禁用外部配置。"""
    environment = {
        key: value for key, value in os.environ.items() if not key.startswith("GIT_")
    }
    environment.update(
        {
            "GIT_ATTR_NOSYSTEM": "1",
            "GIT_CONFIG_GLOBAL": os.devnull,
            "GIT_CONFIG_NOSYSTEM": "1",
            "GIT_NO_LAZY_FETCH": "1",
            "GIT_NO_REPLACE_OBJECTS": "1",
            "GIT_TERMINAL_PROMPT": "0",
        }
    )
    return environment


def _git(target: Path, *args: str) -> str:
    """运行只读 Git 查询并返回去除末尾换行的标准输出。"""
    result = subprocess.run(
        (
            "git",
            *GIT_SAFETY_OPTIONS,
            "--no-optional-locks",
            "-C",
            str(target),
            *args,
        ),
        check=True,
        capture_output=True,
        env=_git_environment(),
        text=True,
    )
    return result.stdout.rstrip("\n")


def _run_git(*args: str) -> None:
    """执行同步所需的结构化 Git argv，并让失败原样中止。"""
    subprocess.run(
        ("git", *GIT_SAFETY_OPTIONS, *args),
        check=True,
        env=_git_environment(),
    )


def _tree_contains_symlink(root: Path) -> bool:
    """用 lstat 遍历目录树，检查时绝不跟随其中的 symlink。"""
    if root.is_symlink():
        return True
    if not root.is_dir():
        return False

    pending = [root]
    while pending:
        current = pending.pop()
        for child in current.iterdir():
            metadata = child.lstat()
            if stat.S_ISLNK(metadata.st_mode):
                return True
            if stat.S_ISDIR(metadata.st_mode):
                pending.append(child)
    return False


def _require_safe_checkout_path(
    repo_dir: Path, target: Path, repository_name: str
) -> None:
    """拒绝 checkout 或 Git 元数据 symlink，避免越出调用者指定根。"""
    lexical_root = Path(os.path.abspath(repo_dir))
    resolved_root = repo_dir.resolve(strict=False)
    if repo_dir.is_symlink() or lexical_root != resolved_root:
        raise RuntimeError(
            f"unsafe reference checkout path for {repository_name}: {target}"
        )

    if (
        target.is_symlink()
        or target.resolve(strict=False).parent != resolved_root
        or (target.exists() and not target.is_dir())
        or _tree_contains_symlink(target / ".git")
    ):
        raise RuntimeError(
            f"unsafe reference checkout path for {repository_name}: {target}"
        )

    # Git 会读取普通 alternates 文件；它不属于 symlink 树检查的覆盖范围。
    for alternate_name in ("alternates", "http-alternates"):
        alternate_path = target / ".git" / "objects" / "info" / alternate_name
        try:
            alternate_path.lstat()
        except (FileNotFoundError, NotADirectoryError):
            continue
        raise RuntimeError(
            "unsafe reference Git object store for "
            f"{repository_name}: {alternate_path}"
        )

    for replacement_path in (
        target / ".git" / "info" / "grafts",
        target / ".git" / "refs" / "replace",
    ):
        try:
            replacement_path.lstat()
        except (FileNotFoundError, NotADirectoryError):
            continue
        raise RuntimeError(
            "unsafe reference Git replacement metadata for "
            f"{repository_name}: {replacement_path}"
        )


def _require_local_git_layout(target: Path, repository_name: str) -> None:
    """确认 Git 解析出的工作树、git dir 和 common dir 均绑定本地 checkout。"""
    expected_worktree = target.resolve(strict=False)
    expected_git_dir = (target / ".git").resolve(strict=False)
    actual_worktree = Path(
        _git(target, "rev-parse", "--show-toplevel")
    ).resolve(strict=False)
    actual_git_dir = Path(
        _git(target, "rev-parse", "--absolute-git-dir")
    ).resolve(strict=False)
    actual_common_dir = Path(
        _git(target, "rev-parse", "--path-format=absolute", "--git-common-dir")
    ).resolve(strict=False)
    if (
        actual_worktree != expected_worktree
        or actual_git_dir != expected_git_dir
        or actual_common_dir != expected_git_dir
    ):
        raise RuntimeError(
            f"unsafe reference Git layout for {repository_name}: {target}"
        )


def _is_safe_local_git_config_key(key: str) -> bool:
    """只接受同步器实际需要且不会执行外部命令的本地 Git 键。"""
    if key in SAFE_LOCAL_GIT_CONFIG_KEYS:
        return True
    lowered = key.lower()
    for suffix in (".merge", ".remote"):
        if lowered.startswith("branch.") and lowered.endswith(suffix):
            return len(key) > len("branch.") + len(suffix)
    return False


def _require_safe_local_git_config(target: Path, repository_name: str) -> None:
    """在 status/fetch/checkout 前拒绝可执行或重定向的本地配置。"""
    worktree_config = target / ".git" / "config.worktree"
    try:
        worktree_config.lstat()
    except (FileNotFoundError, NotADirectoryError):
        pass
    else:
        raise RuntimeError(
            f"unsafe reference Git config for {repository_name}: config.worktree"
        )

    keys = [
        key
        for key in _git(
            target,
            "config",
            "--local",
            "--no-includes",
            "--name-only",
            "--null",
            "--list",
        ).split("\0")
        if key
    ]
    duplicate_keys = {key for key in keys if keys.count(key) > 1}
    unsafe_keys = [key for key in keys if not _is_safe_local_git_config_key(key)]
    if duplicate_keys or unsafe_keys:
        first_key = sorted(duplicate_keys or set(unsafe_keys))[0]
        raise RuntimeError(
            f"unsafe reference Git config for {repository_name}: {first_key}"
        )

    replacement_refs = _git(
        target,
        "for-each-ref",
        "--format=%(refname)",
        "refs/replace",
    )
    if replacement_refs:
        raise RuntimeError(
            "unsafe reference Git replacement metadata for "
            f"{repository_name}: {replacement_refs.splitlines()[0]}"
        )


def _require_safe_index_flags(target: Path, repository_name: str) -> None:
    """拒绝会让 Git status 跳过 tracked 工作树检查的索引优化位。"""
    for record in _git(target, "ls-files", "-v", "-z").split("\0"):
        if not record:
            continue
        tag, separator, relative_path = record.partition(" ")
        if separator and (tag == "S" or tag.islower()):
            raise RuntimeError(
                "unsafe reference index flag for "
                f"{repository_name}: {relative_path}"
            )


def _require_clean_checkout(target: Path, repository_name: str) -> None:
    """拒绝 tracked 或 untracked 本地内容，避免同步覆盖调用者改动。"""
    _require_safe_index_flags(target, repository_name)
    ignored = _git(
        target,
        "ls-files",
        "--others",
        "--ignored",
        "--exclude-standard",
        "-z",
    )
    if ignored:
        first_ignored = ignored.split("\0", maxsplit=1)[0]
        raise RuntimeError(
            "ignored reference checkout content for "
            f"{repository_name}: {first_ignored}"
        )
    status = _git(target, "status", "--porcelain=v1", "--untracked-files=all")
    if status:
        first_change = status.splitlines()[0]
        raise RuntimeError(
            f"dirty reference checkout for {repository_name}: {first_change}"
        )


def _require_path_in_commit(
    target: Path,
    repository: ReferenceRepository,
    relative_text: str,
) -> None:
    """用 Git object 数据确认声明路径属于固定 commit，而非工作树伪造。"""
    result = subprocess.run(
        (
            "git",
            *GIT_SAFETY_OPTIONS,
            "--no-optional-locks",
            "-C",
            str(target),
            "cat-file",
            "-e",
            f"{repository.commit}:{relative_text}",
        ),
        check=False,
        capture_output=True,
        env=_git_environment(),
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "reference path not present in pinned commit for "
            f"{repository.name}: {relative_text}"
        )


def _require_materialized_path_without_symlinks(
    target: Path, repository: ReferenceRepository, relative_text: str
) -> None:
    """逐分量检查物化路径，禁止 checkout symlink 跟随到声明根外。"""
    current = target
    for part in Path(relative_text).parts:
        current /= part
        try:
            metadata = current.lstat()
        except (FileNotFoundError, NotADirectoryError) as error:
            raise RuntimeError(
                f"missing reference path for {repository.name}: {relative_text}"
            ) from error
        if stat.S_ISLNK(metadata.st_mode):
            raise RuntimeError(
                "unsafe reference path symlink for "
                f"{repository.name}: {relative_text}"
            )


def check_repository(repository: ReferenceRepository, repo_dir: Path) -> None:
    """核对本地 HEAD、origin，并验证阶段四声明的真实 focus 路径。"""
    target = repo_dir / repository.name
    _require_safe_checkout_path(repo_dir, target, repository.name)
    if not (target / ".git").is_dir():
        raise RuntimeError(f"missing reference checkout: {repository.name}")
    _require_local_git_layout(target, repository.name)
    _require_safe_local_git_config(target, repository.name)
    _require_clean_checkout(target, repository.name)
    actual_head = _git(target, "rev-parse", "HEAD")
    if actual_head != repository.commit:
        raise RuntimeError(
            f"reference HEAD mismatch for {repository.name}: {actual_head}"
        )
    actual_url = _git(target, "remote", "get-url", "origin")
    if actual_url != repository.url:
        raise RuntimeError(
            f"reference origin mismatch for {repository.name}: {actual_url}"
        )

    checked_focus: list[str] = []
    if repository.stage == 4:
        for relative_text in (*repository.license_files, *repository.focus):
            relative = Path(relative_text)
            if relative.is_absolute() or ".." in relative.parts:
                raise RuntimeError(
                    f"invalid reference path for {repository.name}: {relative_text}"
                )
            _require_path_in_commit(target, repository, relative_text)
            _require_materialized_path_without_symlinks(
                target, repository, relative_text
            )
            if relative_text in repository.focus:
                checked_focus.append(relative_text)
    focus_suffix = "" if not checked_focus else f" focus={','.join(checked_focus)}"
    print(
        f"{repository.name} ({repository.branch}) -> {repository.commit}{focus_suffix}"
    )


def sync_repository(repository: ReferenceRepository, repo_dir: Path) -> None:
    """只 fetch manifest 的完整 SHA，并以 detached HEAD 固定阅读快照。"""
    target = repo_dir / repository.name
    _require_safe_checkout_path(repo_dir, target, repository.name)
    repo_dir.mkdir(parents=True, exist_ok=True)
    if not (target / ".git").is_dir():
        if target.exists() and any(target.iterdir()):
            raise RuntimeError(f"reference target is not empty: {target}")
        _run_git("init", str(target))
        _require_safe_checkout_path(repo_dir, target, repository.name)
        _require_local_git_layout(target, repository.name)
        _run_git("-C", str(target), "remote", "add", "origin", repository.url)
        _require_safe_local_git_config(target, repository.name)
    else:
        _require_local_git_layout(target, repository.name)
        _require_safe_local_git_config(target, repository.name)
        _require_clean_checkout(target, repository.name)
        if _git(target, "remote", "get-url", "origin") != repository.url:
            raise RuntimeError(f"reference origin mismatch for {repository.name}")

    _run_git(
        "-C",
        str(target),
        "fetch",
        "--depth",
        "1",
        "origin",
        repository.commit,
    )
    _run_git(
        "-C",
        str(target),
        "checkout",
        "--detach",
        "--no-overwrite-ignore",
        "FETCH_HEAD",
    )
    check_repository(repository, repo_dir)


def parse_args() -> argparse.Namespace:
    """解析显式 manifest、checkout 根和只读检查模式。"""
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="只读检查，不 fetch")
    parser.add_argument(
        "--manifest",
        type=Path,
        default=project_root / "references" / "manifest.yml",
    )
    parser.add_argument(
        "--repo-dir",
        type=Path,
        default=project_root / "references" / "repos",
    )
    return parser.parse_args()


def main() -> int:
    """按模式执行只读检查或固定 SHA 同步。"""
    args = parse_args()
    for repository in load_manifest(args.manifest):
        if args.check:
            check_repository(repository, args.repo_dir)
        else:
            sync_repository(repository, args.repo_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
