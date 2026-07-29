#!/usr/bin/env python3
"""依存規律の番人 (CLAUDE.md §1.1)。

`core/` は純 Python で、**rclpy / rosbag2_py / shasou_msgs に依存してはならない。**
将来 ROS 以外のミドルウェアからの収録や、MCAP を直接読む変換器へ core/ をそのまま
流用するための規律で、ROS 2 が無い環境で core/ の全モジュールが読み込めることが
そのまま証明になる。`ros/` は ROS に依存してよいので検証対象外。

    python scripts/check_dependencies.py

2 段構えで見る:

1. **静的検査**: core/ の各ソースを AST で読み、import しているモジュールの根を
   集める。関数の中に隠した遅延 import も拾えるうえ、ROS 系だけでなく
   「§1.1 の依存表に無い外部依存を足した」も捕まえられる
2. **実 import 検査**: 禁止モジュールを弾く finder を sys.meta_path に挿して
   全モジュールを実際に import する。間接依存 (core/ が読んだ別モジュールが
   rclpy を読む) や動的 import まで届く

`import shasou_recorder` だけでは不十分な点に注意 (shasou-core の同名スクリプトと
同じ理由): 名前空間パッケージには __init__.py すら無く、素の import では 1 つも
モジュールが読まれない。ここでは pkgutil.walk_packages で全モジュールを走査する。

**禁止依存が環境にあっても検証できるようにしてある。** shasou-core は禁止依存
(pyarrow) が環境にあったらエラーで止める方式だが、recorder で同じにすると、
rclpy が入った Jetson や ROS 開発機 — つまり recorder を実際に触る環境 — で
常に検証不能になり番人として働かない。import を弾く finder で「ROS が無い環境」を
人工的に作れば、CI (ROS 無し) と開発機 (ROS 有り) で判定が食い違わない。
"""

from __future__ import annotations

import ast
import importlib
import pkgutil
import sys
from importlib.machinery import ModuleSpec
from pathlib import Path
from typing import Optional, Sequence

# core/ が依存してはならないサードパーティ (ros/ 専用)
FORBIDDEN = ("rclpy", "rosbag2_py", "shasou_msgs")

# core/ が依存してよい外部ライブラリ = CLAUDE.md §1.1 の依存表。
# **表を更新せずにここへ足さないこと。**
ALLOWED_THIRD_PARTY = frozenset({"pydantic", "yaml", "shasou_core"})

PACKAGE = "shasou_recorder.core"


# --------------------------------------------------------------------------
# 1. 静的検査 (AST)
# --------------------------------------------------------------------------


def _imported_roots(tree: ast.AST) -> set[str]:
    """ソース中の import 文が指すトップレベルモジュール名を集める。

    ast.walk なので関数やメソッドの中の import も拾う (遅延 import で規律を
    迂回できないように)。相対 import (from .layout import ...) は自分の
    パッケージ内なので対象外。
    """
    roots: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module:
                roots.add(node.module.split(".")[0])
    return roots


def check_sources(package_dir: Path) -> list[str]:
    """core/ のソースを読んで、表に無い依存を報告する。"""
    violations: list[str] = []
    for path in sorted(package_dir.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for root in sorted(_imported_roots(tree)):
            if root in FORBIDDEN:
                violations.append(
                    f"{path.name}: {root} を import している "
                    "(ROS 依存は ros/ の責務)"
                )
            elif (
                root not in ALLOWED_THIRD_PARTY
                and root != "shasou_recorder"
                and root not in sys.stdlib_module_names
            ):
                violations.append(
                    f"{path.name}: {root} は §1.1 の依存表に無い "
                    f"(表にあるのは {', '.join(sorted(ALLOWED_THIRD_PARTY))})"
                )
    return violations


# --------------------------------------------------------------------------
# 2. 実 import 検査
# --------------------------------------------------------------------------


class ForbiddenImportBlocker:
    """禁止モジュールの import を ImportError にする finder。

    sys.meta_path の先頭に置く。環境に rclpy が入っていても「入っていない環境」と
    同じ結果になるので、開発機と CI で判定が変わらない。
    """

    def __init__(self, names: Sequence[str]) -> None:
        self._names = frozenset(names)

    def find_spec(self, fullname: str, path=None, target=None) -> Optional[ModuleSpec]:
        if fullname.split(".")[0] in self._names:
            raise ImportError(
                f"{fullname} は core/ から import できない (CLAUDE.md §1.1)"
            )
        return None  # それ以外は通常の finder に任せる


def check_imports() -> tuple[list[str], int]:
    """core/ の全モジュールを、禁止モジュールを塞いだ状態で import する。"""
    package = importlib.import_module(PACKAGE)

    failures: list[str] = []
    checked = 0
    blocker = ForbiddenImportBlocker(FORBIDDEN)
    sys.meta_path.insert(0, blocker)
    try:
        for module in pkgutil.walk_packages(package.__path__, f"{PACKAGE}."):
            try:
                importlib.import_module(module.name)
            except Exception as exc:  # noqa: BLE001 - 原因を問わず失敗として報告する
                failures.append(f"{module.name}: {type(exc).__name__}: {exc}")
            else:
                checked += 1
    finally:
        sys.meta_path.remove(blocker)
    return failures, checked


# --------------------------------------------------------------------------


def main() -> int:
    # 先に読み込まれているとブロッカーを迂回してしまう (sys.modules が優先される)
    preloaded = [name for name in FORBIDDEN if name in sys.modules]
    if preloaded:
        print(
            f"{', '.join(preloaded)} が既に読み込まれているため検証できません。"
            "素の python で実行してください。",
            file=sys.stderr,
        )
        return 1

    try:
        package = importlib.import_module(PACKAGE)
    except ImportError as exc:
        print(
            f"{PACKAGE} を import できません ({exc})。\n"
            '`pip install -e ".[dev]"` するか PYTHONPATH=src を指定してください。',
            file=sys.stderr,
        )
        return 1

    package_dir = Path(next(iter(package.__path__)))
    violations = check_sources(package_dir)
    failures, checked = check_imports()

    if violations or failures:
        print("core/ が §1.1 の依存規律に違反しています:", file=sys.stderr)
        for violation in violations:
            print(f"  - {violation}", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        print(
            "\nROS 依存は ros/ に置き、core/ からは呼ばないでください "
            "(ros/ → core/ の一方向依存のみ)。",
            file=sys.stderr,
        )
        return 1

    print(
        f"OK: {PACKAGE} の {checked} モジュールが "
        f"{' / '.join(FORBIDDEN)} 抜きで読み込めました。"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
