"""パッケージのメタデータと recorder_version() の整合。

manifest の `recorder_version` は「実際に動いたコードの来歴」なので、静かに
フォールバック値 (`0.0.0+unknown`) に落ちていると収録データの追跡ができなくなる。
実行時エラーにならない類の事故なので、ここで固定する。
"""

import re
from importlib.metadata import PackageNotFoundError
from importlib.metadata import version as distribution_version
from pathlib import Path

import pytest

from shasou_recorder.core.config import (
    DISTRIBUTION_NAME,
    UNKNOWN_VERSION,
    recorder_version,
)

PYPROJECT = Path(__file__).resolve().parents[1] / "pyproject.toml"

# tomllib は 3.11 以降。requires-python の下限が 3.10 なので正規表現で読む
# (自分たちが書いたファイルなので、汎用の TOML パーサは要らない)。
_PROJECT_TABLE = re.search(
    r"(?ms)^\[project\]\n(.*?)^\[", PYPROJECT.read_text(encoding="utf-8")
).group(1)


def project_field(name: str) -> str:
    return re.search(rf'(?m)^{name} = "([^"]+)"', _PROJECT_TABLE).group(1)


def declared_dependencies() -> list[str]:
    block = re.search(r"(?ms)^dependencies = \[(.*?)^\]", _PROJECT_TABLE).group(1)
    return re.findall(r'^\s*"([^">=<\[ ]+)', block, flags=re.MULTILINE)


def test_distribution_name_matches_pyproject():
    # ずれると recorder_version() が永久にフォールバックを返し、**全 manifest に
    # 0.0.0+unknown が載る**。実行時は何のエラーも出ないのでここで縛る。
    assert DISTRIBUTION_NAME == project_field("name")


def test_recorder_version_is_not_the_fallback():
    try:
        installed = distribution_version(DISTRIBUTION_NAME)
    except PackageNotFoundError:
        pytest.skip(
            "shasou-recorder が未インストール (PYTHONPATH 実行)。"
            'pip install -e ".[dev]" した環境で検証する'
        )

    assert recorder_version() == installed
    assert recorder_version() != UNKNOWN_VERSION


def test_runtime_dependencies_match_the_dependency_table():
    # CLAUDE.md §1.1 の依存表と一致すること。表を更新せずに依存を足さない
    # (rclpy / rosbag2_py は apt 由来なので pip の依存にはできない)。
    assert set(declared_dependencies()) == {"pydantic", "pyyaml", "shasou-core"}
