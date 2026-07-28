"""studio が編集元の定義 (vehicle_type / platform / vehicle / calibration) の取得層。

定義の正 (source of truth) は studio で、recorder は同期された `definitions/` を
**読むだけ** (§11)。書き込む経路をこのモジュールに作らないこと — `definitions/` は
消して再同期できるキャッシュであり、recorder が生成する一次データ (`drives/`) とは
バックアップ方針から違う (§10)。`DefinitionsLayout` が作成系メソッドを持たないのと
同じ理由で、Provider も取得系しか持たない。

読む目的は preflight (§9)
-------------------------
収録自体にキャリブは要らない (bag には生のセンサデータが入り、適用は下流の変換時)。
それでも recorder が定義を読むのは、**走り出す前に**「platform の全センサ分の
キャリブが揃っているか」「キャリブが他車両のものになっていないか」を確認して
1 ルート分を無駄にしないため。

例外と Issue の切り分け
-----------------------
- **ファイル層の失敗は例外** (`DefinitionError` 系)。定義が無い・読めない・
  YAML が壊れている・スキーマ違反。一式を組み立てられないので値型を返しようがない
- **定義同士の不整合は Issue** (core の `ValidationResult`)。全部読めているので、
  1 件目で止めずまとめて報告した方が 1 回で直せる

preflight は両方を 1 つの器で受け取れる。`resolve_and_validate()` が解決を
try/except で包み、例外を ERROR 1 件 (`definition_unavailable`) に変換して
`ValidationResult` に載せる。呼び出し側は `result.ok` だけ見れば収録開始の可否を
判断でき、例外処理を各所に散らさずに済む。

整合検証は core が正 (§1.2)
---------------------------
照合規則は shasou-core の `validation.py` が持つ。recorder は再実装せず、
manifest 非依存版 (`*_by_ids`) を呼ぶだけ。manifest 版は同じ関数へ委譲されているので、
preflight (収録前) と studio の取り込み (収録後) で判定が分岐しない。

このモジュールは core/ にあるので ROS に依存しない。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Protocol, TypeVar, runtime_checkable

import yaml
from pydantic import BaseModel, ValidationError

from shasou_core.schemas.calibration import CalibrationSet
from shasou_core.schemas.platform import Platform
from shasou_core.schemas.vehicle import CanSpec, Vehicle, VehicleType
from shasou_core.validation import (
    Severity,
    ValidationResult,
    validate_calibration_against_platform,
    validate_calibration_coverage_by_ids,
    validate_vehicle_consistency_by_ids,
)

from .config import DefinitionsConfig, DefinitionsProvider, RecorderConfig
from .layout import DefinitionsLayout


class DefinitionError(ValueError):
    """定義を取得できない。呼び出し側は preflight の失敗として扱う。

    定義同士の不整合はこの例外ではなく Issue で報告する (モジュール docstring)。
    """


class DefinitionNotFoundError(DefinitionError):
    """定義ファイルが無い。**探したパスを必ずメッセージに含める。**

    同期漏れの一次診断がこのメッセージなので、「どこを見に行ったか」が分からないと
    運用者は `definitions/` のどこを直せばよいか判断できない。
    """


class DefinitionInvalidError(DefinitionError):
    """定義ファイルは在るが使えない (YAML 破損・非マッピング・スキーマ違反)。"""


# --------------------------------------------------------------------------
# YAML 読み込み
# --------------------------------------------------------------------------
#
# 手順は config.py の load_config と同じ (yaml.safe_load 固定、空・非マッピングを
# 弾く)。**まだ共通化しない** — config.py の申し送りどおり、definitions.py は
# 2 例目で、3 例目 (manifest.py) が出た時点で core/yamlio.py へ移す。2 例では
# 「manifest のスキーマ版の扱い」が見えず、早すぎる抽象を固定してしまう。

_M = TypeVar("_M", bound=BaseModel)


def _load_definition(
    path: Path, model: type[_M], *, kind: str, hint: str = ""
) -> _M:
    """定義ファイル 1 つを読んで core のスキーマで検証する。

    config.py はスキーマ違反を pydantic の ValidationError のまま通すが、ここでは
    DefinitionInvalidError に包んで**パスを添える**。設定は 1 ファイルなのでどれが
    悪いか自明だが、定義は 3 つの ID から最大 4 ファイルを引くので「どの定義が古いか」
    こそが診断の主目的だから。pydantic のメッセージは全文を保持し `from` で連鎖
    させるので、情報は減らない。
    """
    if not path.is_file():
        raise DefinitionNotFoundError(
            f"{kind} の定義が見つからない: {path}"
            f"{hint}。sync で定義を同期したか、ID が正しいか確認すること"
        )

    try:
        text = path.read_text(encoding="utf-8")
    except OSError as error:
        raise DefinitionError(f"{kind} の定義を読めない: {path} ({error})") from error

    try:
        # safe_load であること。definitions/ は studio や NAS 経由で置かれるので、
        # 任意オブジェクトを構築する loader を使ってはならない (§1.1)。
        data = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise DefinitionInvalidError(
            f"{kind} の定義の YAML が壊れている: {path}\n{error}"
        ) from error

    if data is None:
        raise DefinitionInvalidError(f"{kind} の定義が空: {path}")
    if not isinstance(data, dict):
        raise DefinitionInvalidError(
            f"{kind} の定義のトップレベルはマッピングであること: {path} "
            f"({type(data).__name__} が来た)"
        )

    try:
        return model.model_validate(data)
    except ValidationError as error:
        raise DefinitionInvalidError(
            f"{kind} の定義が shasou-core のスキーマに適合しない: {path}\n{error}"
        ) from error


# --------------------------------------------------------------------------
# Provider
# --------------------------------------------------------------------------


@runtime_checkable
class DefinitionProvider(Protocol):
    """定義の取得元 (§11)。取得系のみで、書き込みは持たない。

    当面の実装は LocalFileProvider だけ。studio との HTTP 同期を足すときも、
    呼び出し側 (preflight / CLI) はこの Protocol しか見ないので差し替えで済む。
    定義が取得できない場合は DefinitionError 系を送出する契約。
    """

    def get_vehicle_type(self, vehicle_type_id: str) -> VehicleType: ...

    def get_platform(self, platform_id: str) -> Platform: ...

    def get_vehicle(self, vehicle_id: str) -> Vehicle: ...

    def get_calibration(self, vehicle_id: str, calib_id: str) -> CalibrationSet: ...

    def list_calib_ids(self, vehicle_id: str) -> list[str]:
        """ある車両個体の calib_id を昇順で返す (無ければ空)。

        収録には要らないが、「指定した calib_id が無い」ときに候補を出せると
        同期漏れと typo を切り分けられる。`catalog` 系 CLI からも使える。
        """
        ...


@dataclass(frozen=True)
class LocalFileProvider:
    """`definitions/` のローカル YAML を読むだけの Provider (§11)。

    studio を立てないスタンドアロン利用はこれだけで完結する。パスの解決は必ず
    DefinitionsLayout に任せ、自分で組み立てない (§10 のフォルダ構成を表現する
    場所を 1 つに保つため)。
    """

    layout: DefinitionsLayout

    def get_vehicle_type(self, vehicle_type_id: str) -> VehicleType:
        return _load_definition(
            self.layout.vehicle_type(vehicle_type_id),
            VehicleType,
            kind=f"vehicle_type {vehicle_type_id!r}",
        )

    def get_platform(self, platform_id: str) -> Platform:
        return _load_definition(
            self.layout.platform(platform_id),
            Platform,
            kind=f"platform {platform_id!r}",
        )

    def get_vehicle(self, vehicle_id: str) -> Vehicle:
        return _load_definition(
            self.layout.vehicle(vehicle_id),
            Vehicle,
            kind=f"vehicle {vehicle_id!r}",
        )

    def get_calibration(self, vehicle_id: str, calib_id: str) -> CalibrationSet:
        # キャリブ値は個体固有なので vehicle 配下 (§10)。不在時は実在する calib_id を
        # 併記する — 同期漏れ (候補が空) と typo (候補はあるが綴りが違う) を
        # メッセージだけで切り分けられる。
        available = self.list_calib_ids(vehicle_id)
        hint = (
            f" (この車両にあるのは: {', '.join(available)})"
            if available
            else " (この車両のキャリブが 1 つも同期されていない)"
        )
        return _load_definition(
            self.layout.calibration(vehicle_id, calib_id),
            CalibrationSet,
            kind=f"calibration {calib_id!r} (vehicle {vehicle_id!r})",
            hint=hint,
        )

    def list_calib_ids(self, vehicle_id: str) -> list[str]:
        return self.layout.iter_calib_ids(vehicle_id)


class StudioHttpProvider:
    """studio と HTTP 同期する Provider (**未実装**)。

    §11 の「同期のタイミング」(収録前後の双方向 reconcile) と `definitions/` への
    書き込みはここが担う。当面はインターフェースを切っておくだけで、
    スタンドアロン利用者は LocalFileProvider だけで完結する。
    """

    def __init__(self, config: DefinitionsConfig) -> None:
        raise NotImplementedError(
            "studio との HTTP 同期は未実装。definitions.provider=local を使うこと "
            f"(設定された studio_url: {config.studio_url})"
        )


def create_provider(config: RecorderConfig) -> DefinitionProvider:
    """設定から Provider を組み立てる (config.py の DefinitionsProvider と対応)。

    **副作用は無い** — `definitions/` を作りも読みもせず、パス解決の入口を渡すだけ。
    """
    if config.definitions.provider is DefinitionsProvider.LOCAL:
        return LocalFileProvider(layout=config.data_layout().definitions)
    return StudioHttpProvider(config.definitions)


# --------------------------------------------------------------------------
# 定義一式の解決
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ResolvedDefinitions:
    """設定が指す定義一式 (§11 の参照連鎖を解決した結果)。

    requested_* は**設定が要求した ID** で、定義側の自己申告 (platform.platform_id
    等) と食い違いうる。両方を持つのは、core の検証関数が「宣言された ID」と
    「定義の中身」を突き合わせる形になっているため。
    """

    requested_platform_id: str
    requested_vehicle_id: str
    requested_calib_id: str

    platform: Platform
    vehicle: Vehicle
    vehicle_type: VehicleType
    calibration: CalibrationSet

    def effective_can_spec(self) -> CanSpec:
        """この車両個体の実効 CAN 仕様。

        車種デフォルト (VehicleType.can_defaults) を個体上書き
        (Vehicle.can_overrides) がフィールド単位で覆う。合成規則の実体は core の
        Vehicle.effective_can_spec で、ここは解決済みの vehicle_type を渡すだけ。
        """
        return self.vehicle.effective_can_spec(self.vehicle_type)


def resolve_definitions(
    provider: DefinitionProvider,
    *,
    platform_id: str,
    vehicle_id: str,
    calib_id: str,
) -> ResolvedDefinitions:
    """設定が指す定義一式を解決する (取得できなければ DefinitionError)。

    参照の連鎖 (§11)::

        設定 → vehicle_id  → Vehicle
        設定 → platform_id → Platform → vehicle_type → VehicleType
        設定 → vehicle_id + calib_id  → CalibrationSet

    **platform は vehicle.platform_id ではなく設定の platform_id で引く。**
    設定がそのドライブの宣言であり (収録データも設定の platform_id 配下に置かれる)、
    vehicle 側との食い違いは黙って追従せず Issue として報告すべき事実だから。

    整合の検証はしない (validate_definitions の責務)。ここは「読めるか」だけを見る。
    """
    vehicle = provider.get_vehicle(vehicle_id)
    platform = provider.get_platform(platform_id)
    # VehicleType は platform 経由で解決する (Vehicle は車種を直接持たない)
    vehicle_type = provider.get_vehicle_type(platform.vehicle_type)
    calibration = provider.get_calibration(vehicle_id, calib_id)

    return ResolvedDefinitions(
        requested_platform_id=platform_id,
        requested_vehicle_id=vehicle_id,
        requested_calib_id=calib_id,
        platform=platform,
        vehicle=vehicle,
        vehicle_type=vehicle_type,
        calibration=calibration,
    )


# --------------------------------------------------------------------------
# 整合検証
# --------------------------------------------------------------------------


def validate_definitions(resolved: ResolvedDefinitions) -> ValidationResult:
    """解決した定義一式の整合を検証する (§9 の preflight が使う)。

    照合規則の正は core (§1.2) なので、recorder は core の関数を呼ぶだけ。
    例外は下記 2 件で、どちらも「**要求した ID と、返ってきた定義が自己申告する
    ID の一致**」— core には「取得要求」という概念が無いため core 側に置き場が無い
    (core が持つのは manifest/宣言 ID と定義の突き合わせ)。規則の複製ではない。
    将来 studio が独立に手に入れた Platform と VehicleType を突き合わせる必要が
    出たら、そのとき core へ移すのが正しい。

    ERROR が 1 つでもあれば収録を開始しないこと (result.ok が False)。
    """
    result = ValidationResult()

    if resolved.platform.platform_id != resolved.requested_platform_id:
        # ファイル名と中身が食い違っている。収録データは設定の platform_id 配下に
        # 置かれるので、放置すると studio 取り込み時に platform 不明のドライブになる。
        # code は同じ照合をする core の validate_manifest_against_platform と揃える。
        result.add(
            Severity.ERROR, "platform_id_mismatch",
            f"要求した platform ({resolved.requested_platform_id}) != "
            f"platform.platform_id ({resolved.platform.platform_id})",
            requested_platform_id=resolved.requested_platform_id,
            platform_id=resolved.platform.platform_id,
        )

    if resolved.vehicle_type.vehicle_type_id != resolved.platform.vehicle_type:
        # VehicleType は platform.vehicle_type を鍵に引いているので、これも
        # 要求 ID との照合。食い違うと実効 CAN 仕様が別車種のものになる
        # (速度の符号規則が変わり、学習データの行動ラベルが壊れる)。
        result.add(
            Severity.ERROR, "vehicle_type_mismatch",
            f"platform.vehicle_type ({resolved.platform.vehicle_type}) != "
            f"vehicle_type.vehicle_type_id ({resolved.vehicle_type.vehicle_type_id})",
            platform_vehicle_type=resolved.platform.vehicle_type,
            vehicle_type_id=resolved.vehicle_type.vehicle_type_id,
        )

    # vehicle.platform_id / vehicle.vehicle_id と宣言 ID の突き合わせ
    result.merge(
        validate_vehicle_consistency_by_ids(
            resolved.vehicle,
            platform_id=resolved.requested_platform_id,
            vehicle_id=resolved.requested_vehicle_id,
        )
    )
    # calib_id / vehicle の一致と、platform 全センサ分のキャリブ網羅性
    result.merge(
        validate_calibration_coverage_by_ids(
            resolved.platform,
            resolved.calibration,
            calib_id=resolved.requested_calib_id,
            vehicle_id=resolved.requested_vehicle_id,
        )
    )
    # 構成上の宣言 (ChannelSpec) と実測キャリブの整合。元から manifest 非依存で、
    # intrinsics_model の不一致を ERROR で拾える (歪み補正が破綻する組み合わせを
    # 走り出す前に止められる)。
    result.merge(
        validate_calibration_against_platform(resolved.platform, resolved.calibration)
    )
    return result


def resolve_and_validate(
    provider: DefinitionProvider, config: RecorderConfig
) -> tuple[Optional[ResolvedDefinitions], ValidationResult]:
    """設定が指す定義一式を解決して検証する。preflight 向けの入口。

    解決の失敗 (DefinitionError) を ERROR 1 件に変換して同じ ValidationResult に
    載せるので、呼び出し側は例外処理を持たずに `result.ok` だけで収録開始の可否を
    判断できる。**解決に失敗した場合の第 1 要素は None** (一式を組み立てられて
    いないので、値型を半端に埋めて返さない)。
    """
    try:
        resolved = resolve_definitions(
            provider,
            platform_id=config.platform_id,
            vehicle_id=config.vehicle_id,
            calib_id=config.calib_id,
        )
    except DefinitionError as error:
        result = ValidationResult()
        result.add(
            Severity.ERROR, "definition_unavailable",
            f"定義を解決できない: {error}",
            error_type=type(error).__name__,
        )
        return None, result

    return resolved, validate_definitions(resolved)
