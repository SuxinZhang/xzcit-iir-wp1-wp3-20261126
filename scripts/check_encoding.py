#!/usr/bin/env python3
"""Repository encoding and line-ending gate script for CI and local checks.
用于 CI 门禁与本地自检的仓库字符编码和换行符检查脚本。

Enforces AI_EXECUTION_PLAN Section 1.1 across source, config, build, and doc files:
1. Valid UTF-8 decoding (rejects GBK/GB2312/UTF-16/invalid byte sequences).
2. No UTF-8 BOM (rejects leading 0xEF 0xBB 0xBF).
3. LF-only line endings (rejects CR / CRLF).

在源码、配置、构建脚本与文档中强制执行 AI_EXECUTION_PLAN 第 1.1 节规范：
1. 合法 UTF-8 编码（拒绝 GBK/GB2312/UTF-16 及非法字节序列）。
2. 无 UTF-8 BOM（拒绝文件头 0xEF 0xBB 0xBF）。
3. 仅允许 LF 换行符（拒绝 CR / CRLF）。
"""

from __future__ import annotations

import argparse
import pathlib
import sys
from typing import Iterable

# File extensions subject to UTF-8 (no BOM) and LF enforcement.
# 需要强制执行 UTF-8（无 BOM）与 LF 校验的文件扩展名集合。
TARGET_EXTENSIONS: frozenset[str] = frozenset({
    ".c", ".h", ".cpp", ".hpp",
    ".py", ".rs",
    ".ts", ".tsx", ".js", ".css", ".html",
    ".yaml", ".yml", ".xml", ".json",
    ".cmake", ".md", ".sh",
})

# Exact filenames (without relying on suffix) subject to enforcement.
# 需要强制校验的特定文件名集合（不依赖扩展名）。
TARGET_FILENAMES: frozenset[str] = frozenset({
    "CMakeLists.txt",
    ".editorconfig",
})

# Directory names skipped during recursive repository scans.
# 递归扫描仓库时跳过的目录名称集合。
SKIP_DIR_NAMES: frozenset[str] = frozenset({
    ".git",
    "build",
    "target",
    "node_modules",
    ".codeartsdoer",
    "__pycache__",
    ".pytest_cache",
})

# Read-only third-party vendor SDK prefixes excluded from repo encoding enforcement.
# 排除在编码门禁之外的只读第三方芯片原厂 SDK 路径前缀（保持 ST 官方原样文件）。
SKIP_PATH_PREFIXES: tuple[str, ...] = (
    "Drivers/STM32F4xx_HAL_Driver",
    "Drivers/CMSIS",
    "Middlewares/Third_Party",
)


def is_target_file(path: pathlib.Path) -> bool:
    """Return True if path matches the governed source/config/doc file set.
    当路径属于受控的源码、配置或文档文件集合时返回 True。
    """
    return path.suffix.lower() in TARGET_EXTENSIONS or path.name in TARGET_FILENAMES


def should_skip_path(path: pathlib.Path, root: pathlib.Path) -> bool:
    """Return True if any directory component or vendor SDK prefix should be skipped.
    当路径包含应跳过目录层级或属于只读第三方原厂 SDK 前缀时返回 True。
    """
    if any(part in SKIP_DIR_NAMES for part in path.parts):
        return True
    try:
        rel_posix = path.resolve().relative_to(root.resolve()).as_posix()
    except ValueError:
        rel_posix = path.as_posix()
    return any(
        rel_posix == prefix or rel_posix.startswith(prefix + "/")
        for prefix in SKIP_PATH_PREFIXES
    )


def check_file(path: pathlib.Path) -> list[str]:
    """Validate a single file for UTF-8 without BOM and LF-only line endings.
    校验单个文件是否为无 BOM UTF-8 且仅包含 LF 换行符。

    Args:
        path: File path to inspect.
              待检查的文件路径。

    Returns:
        List of violation messages; empty list means the file passed.
        违规问题描述列表；为空表示校验通过。
    """
    violations: list[str] = []
    try:
        raw = path.read_bytes()
    except OSError as exc:
        return [f"read failed ({exc})"]

    # Reject UTF-8 BOM and UTF-16 BOM markers before UTF-8 decoding.
    # 在 UTF-8 解码之前先拦截 UTF-8 BOM 及 UTF-16 BOM 标记。
    if raw.startswith(b"\xef\xbb\xbf"):
        violations.append("contains UTF-8 BOM (EF BB BF)")
    elif raw.startswith((b"\xff\xfe", b"\xfe\xff")):
        violations.append("contains UTF-16 BOM")

    # Reject any CR character to prevent CRLF or classic-Mac CR line endings.
    # 拦截任意 CR 字符，杜绝 CRLF 或旧式 CR 换行。
    if b"\r" in raw:
        violations.append("contains CR/CRLF line ending (\\r)")

    # Verify the entire byte payload is valid UTF-8.
    # 校验整个字节流是否为合法 UTF-8 编码。
    try:
        raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        violations.append(f"invalid UTF-8 at byte {exc.start}: {exc.reason}")

    return violations


def collect_files(root: pathlib.Path, paths: list[str], scan_all: bool) -> list[pathlib.Path]:
    """Collect candidate files from explicit CLI paths or the repository tree.
    从命令行显式路径或仓库目录树中收集待校验文件列表。
    """
    candidates: list[pathlib.Path] = []
    if paths and not scan_all:
        for item in paths:
            p = pathlib.Path(item)
            if not p.is_absolute():
                p = root / p
            if p.is_dir():
                for child in sorted(p.rglob("*")):
                    if child.is_file() and not should_skip_path(child, root) and is_target_file(child):
                        candidates.append(child)
            elif p.is_file():
                if not should_skip_path(p, root) and is_target_file(p):
                    candidates.append(p)
        return candidates

    for p in sorted(root.rglob("*")):
        if p.is_file() and not should_skip_path(p, root) and is_target_file(p):
            candidates.append(p)
    return candidates


def format_rel(path: pathlib.Path, root: pathlib.Path) -> str:
    """Format path relative to repository root when possible for readable logs.
    尽可能将路径格式化为相对仓库根目录的形式，便于日志阅读。
    """
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def run_check(root: pathlib.Path, paths: list[str], scan_all: bool) -> int:
    """Execute encoding and line-ending checks and return process exit code.
    执行编码与换行符检查并返回进程退出码（0 表示全部通过，1 表示存在违规）。
    """
    files = collect_files(root, paths, scan_all)
    failures: list[tuple[str, list[str]]] = []

    for f in files:
        issues = check_file(f)
        if issues:
            failures.append((format_rel(f, root), issues))

    if failures:
        print(f"ENCODING CHECK FAILED: {len(failures)} / {len(files)} file(s) violated UTF-8 (no BOM) + LF:")
        for rel, issues in failures:
            print(f"  - {rel}: {'; '.join(issues)}")
        return 1

    print(f"ENCODING CHECK PASSED: {len(files)} file(s) verified (UTF-8 without BOM, LF only).")
    return 0


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    """Parse CLI arguments for the encoding checker.
    解析编码检查脚本的命令行参数。
    """
    parser = argparse.ArgumentParser(
        description="Verify source/config/doc files use UTF-8 without BOM and LF line endings."
    )
    parser.add_argument(
        "paths",
        nargs="*",
        help="Optional files or directories to check (defaults to whole repository).",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Explicitly scan the entire repository tree.",
    )
    return parser.parse_args(list(argv) if argv is not None else None)


def main() -> int:
    """Entry point for CLI execution.
    命令行执行的入口函数。
    """
    args = parse_args()
    repo_root = pathlib.Path(__file__).resolve().parent.parent
    return run_check(repo_root, args.paths, args.all)


if __name__ == "__main__":
    sys.exit(main())
