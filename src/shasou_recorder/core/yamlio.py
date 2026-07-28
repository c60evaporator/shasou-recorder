"""YAML ファイルの読み書き (core/ 共通)。

recorder が扱う YAML は 3 系統ある — 設定ファイル (config.py)、`definitions/` の
定義 (definitions.py)、`manifest.yaml` (manifest.py)。いずれも「読む →
`yaml.safe_load` → トップレベルがマッピングであることを確認」までが同じなので、
その手順だけをここに集約する。

**スキーマ検証 (`model_validate`) はここに含めない。** config.py は pydantic の
`ValidationError` をそのまま通し、definitions.py はどのファイルが悪いか分かるよう
パスを添えて包む、と方針が違う。共通なのはマッピングを取り出すところまで。

例外の扱い
----------
このモジュールは自分の例外 (`YamlError` 系) を投げ、**呼び出し側が自分の型に
包み直す**。例外クラスを引数で受け取る形にしないのは、呼び出し側の例外階層
(definitions は I/O とスキーマで型を分けている) を yamlio が知る必要が無いから。
メッセージは `what` ラベルから組み立てるので、包み直しても文言は呼び出し側の
語彙のまま保てる。

`yaml.load` は使わない
----------------------
設定も定義も NAS 経由や他人の手で置かれうるので、任意の Python オブジェクトを
構築する loader を使ってはならない (CLAUDE.md §1.1)。読み込みは `safe_load` 固定。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import yaml


class YamlError(ValueError):
    """YAML ファイルを読み込めない。呼び出し側が自分の例外型に包み直す。"""


class YamlReadError(YamlError):
    """ファイル自体にアクセスできない (不在・権限・I/O エラー)。"""


class YamlInvalidError(YamlError):
    """読めたが中身が使えない (YAML 破損・空・トップレベルが非マッピング)。"""


def load_mapping(path: Path, *, what: str) -> dict[str, Any]:
    """YAML を読んでトップレベルのマッピングを返す。

    `what` はメッセージの主語 ("設定ファイル"、"platform 'x' の定義" 等)。
    呼び出し側の語彙でエラーを読ませるためにここで受け取る。
    """
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as error:
        raise YamlReadError(f"{what}を読めない: {path} ({error})") from error

    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise YamlInvalidError(f"{what}の YAML が壊れている: {path}\n{error}") from error

    if data is None:
        raise YamlInvalidError(f"{what}が空: {path}")
    if not isinstance(data, dict):
        raise YamlInvalidError(
            f"{what}のトップレベルはマッピングであること: {path} "
            f"({type(data).__name__} が来た)"
        )
    return data


def dump_mapping(data: Mapping[str, Any]) -> str:
    """マッピングを YAML 文字列にする。

    - `allow_unicode=True`: location や notes に日本語が入りうる。エスケープすると
      人が読めなくなる (manifest は運用者が直接開くファイル)
    - `sort_keys=False`: モデルのフィールド定義順を保つ。アルファベット順に
      並べ替えると manifest の読み順 (識別 → 出自 → メタ) が崩れる
    """
    return yaml.safe_dump(dict(data), allow_unicode=True, sort_keys=False)
