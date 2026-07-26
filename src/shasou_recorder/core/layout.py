"""データディレクトリのパス解決と drive_id の採番。

CLAUDE.md §10 のフォルダ構成を表現する唯一の場所。manifest.py / events.py /
checksum.py / catalog.py / ros/writer.py は「どこに何を置くか」を自分で組み立てず、
必ずこのモジュールを経由する。パスの組み立てが散らばると、フォルダ構成を変えた
ときに追随漏れが起きるため。

読み取り専用と書き込みの分離
----------------------------
§10 は 2 系統に分かれる:

- ``definitions/``  studio が編集元。recorder は同期して読むだけ
- ``<platform_id>/drives/<drive_id>/``  recorder が生成する一次データ

この違いを型で表す。DefinitionsLayout はディレクトリを作るメソッドを一つも
持たないので、「definitions/ に書き込んでしまう」事故が API 上起こりえない。
書き込み先を作れるのは DriveLayout.create() だけ。

「パスを返す」ことと「ディレクトリを作る」ことは混同しない。パス解決はすべて
副作用が無く、作成は明示的なメソッド (create_drive / DriveLayout.create) が行う。

drive_id
--------
``日付_時刻_車両_場所`` (例 ``2026-07-16_1030_vehicle01_osaka-umeda``)。
時刻は分まで。同一分内に複数ドライブが始まると衝突するので、ディレクトリ作成を
排他的に行い (os.makedirs(exist_ok=False))、失敗したら連番サフィックスを付けて
採番し直す (``..._osaka-umeda_2``)。

**待たずにサフィックスで解決するのは、採番が StartRecording のハンドラ内で走る
ため。** CARLA ブリッジの応答タイムアウトは 30 秒なので、分が変わるまで待つ
方式だと衝突 1 回で応答がタイムアウトし、クライアントはルートを中断する一方で
recorder は収録を開始する、という不整合が起きる (§8 の「応答が返った時点で
bag は開いていること」が壊れる)。サフィックスなら待ち時間ゼロで確実に解決する。

このモジュールは core/ にあるので ROS に依存しない。shasou-core にも依存しない
(drive_id の採番に DriveManifest は要らない)。
"""

from __future__ import annotations

import os
import re
import unicodedata
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

# --------------------------------------------------------------------------
# §10 のフォルダ構成 (名前はここだけに書く)
# --------------------------------------------------------------------------

DEFINITIONS_DIRNAME = "definitions"
VEHICLE_TYPES_DIRNAME = "vehicle_types"
PLATFORMS_DIRNAME = "platforms"
VEHICLES_DIRNAME = "vehicles"
CALIBRATIONS_DIRNAME = "calibrations"
CALIBRATION_FILENAME = "calibration.yaml"
CALIBRATION_REPORT_FILENAME = "report.pdf"

DRIVES_DIRNAME = "drives"
BAGS_DIRNAME = "bags"
TAGS_DIRNAME = "tags"
HEALTH_DIRNAME = "health"
MANIFEST_FILENAME = "manifest.yaml"
CHECKSUMS_FILENAME = "checksums.sha256"
EVENTS_FILENAME = "events.jsonl"
TOPIC_STATS_FILENAME = "topic_stats.json"
NOTES_FILENAME = "notes.md"

CATALOG_FILENAME = "catalog.sqlite"

SEGMENT_PREFIX = "segment_"
SEGMENT_SUFFIX = ".mcap"

# ドライブ配下に必ず作るサブディレクトリ。ファイルは書き手が作るが、置き場だけは
# 収録開始時に揃えておく (収録中に mkdir が要らないように)。
DRIVE_SUBDIRS: tuple[str, ...] = (BAGS_DIRNAME, TAGS_DIRNAME, HEALTH_DIRNAME)

# 定義ファイルの拡張子 (vehicle_types / platforms / vehicles)
DEFINITION_SUFFIX = ".yaml"


# --------------------------------------------------------------------------
# ID の検証
# --------------------------------------------------------------------------

_UNSAFE_COMPONENTS = frozenset({"", ".", ".."})

# データルート直下で recorder 自身が使う名前。platform_id がこれと衝突すると
# definitions/ や catalog.sqlite と同じパスを指してしまう。
RESERVED_ROOT_NAMES = frozenset({DEFINITIONS_DIRNAME, CATALOG_FILENAME})


def _safe_component(
    value: str, kind: str, *, reserved: frozenset[str] = frozenset()
) -> str:
    """パス要素として使える ID か検証して返す。

    このモジュールの仕事はパスの組み立てなので、外から来た ID (設定ファイルや
    StartRecording のリクエスト由来) がディレクトリ階層を跨がないことをここで
    保証する。core の型は ID の中身を制約していないため、防御は組み立て側に置く。
    """
    if not isinstance(value, str) or value in _UNSAFE_COMPONENTS:
        raise ValueError(f"{kind} が空または不正: {value!r}")
    if "/" in value or "\\" in value or "\0" in value:
        raise ValueError(
            f"{kind} にパス区切り文字を含められない: {value!r}"
        )
    if value.casefold() in reserved:
        # 大文字小文字を無視して比較する。NAS の SMB 共有は区別しないので、
        # "Definitions" を通すと環境によって衝突する
        raise ValueError(
            f"{kind} に予約名は使えない: {value!r} "
            f"(予約: {', '.join(sorted(reserved))})"
        )
    return value


def _safe_platform_id(value: str) -> str:
    """platform_id はデータルート直下のディレクトリ名になるので予約名を弾く。

    platform_id を検証する箇所すべてから呼ぶこと。ある場所では通り別の場所では
    弾かれる、という不整合を避けるため。
    """
    return _safe_component(value, "platform_id", reserved=RESERVED_ROOT_NAMES)


# --------------------------------------------------------------------------
# definitions/ (読み取り専用)
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class DefinitionsLayout:
    """``definitions/`` 配下のパス解決 (読むだけ)。

    定義の正 (source of truth) は studio で、recorder は同期してキャッシュを
    読むだけ (§11)。したがってこのクラスは **作成系のメソッドを持たない** —
    definitions/ を書き換える経路を API から無くしておくため。
    同期そのもの (definitions/ の更新) は DefinitionProvider の責務。
    """

    root: Path

    # ── 定義ファイル ────────────────────────────────────────────────────

    def vehicle_type(self, vehicle_type_id: str) -> Path:
        """vehicle_types/<vehicle_type_id>.yaml"""
        name = _safe_component(vehicle_type_id, "vehicle_type_id")
        return self.root / VEHICLE_TYPES_DIRNAME / f"{name}{DEFINITION_SUFFIX}"

    def platform(self, platform_id: str) -> Path:
        """platforms/<platform_id>.yaml"""
        name = _safe_platform_id(platform_id)
        return self.root / PLATFORMS_DIRNAME / f"{name}{DEFINITION_SUFFIX}"

    def vehicle(self, vehicle_id: str) -> Path:
        """vehicles/<vehicle_id>.yaml"""
        name = _safe_component(vehicle_id, "vehicle_id")
        return self.root / VEHICLES_DIRNAME / f"{name}{DEFINITION_SUFFIX}"

    # ── キャリブ (車両個体固有なので vehicle 配下) ──────────────────────

    def calibration_dir(self, vehicle_id: str, calib_id: str) -> Path:
        """calibrations/<vehicle_id>/<calib_id>/"""
        vehicle = _safe_component(vehicle_id, "vehicle_id")
        calib = _safe_component(calib_id, "calib_id")
        return self.root / CALIBRATIONS_DIRNAME / vehicle / calib

    def calibration(self, vehicle_id: str, calib_id: str) -> Path:
        """calibrations/<vehicle_id>/<calib_id>/calibration.yaml (CalibrationSet の正)"""
        return self.calibration_dir(vehicle_id, calib_id) / CALIBRATION_FILENAME

    def calibration_report(self, vehicle_id: str, calib_id: str) -> Path:
        """calibrations/<vehicle_id>/<calib_id>/report.pdf (品質レポート)"""
        return self.calibration_dir(vehicle_id, calib_id) / CALIBRATION_REPORT_FILENAME

    def iter_calib_ids(self, vehicle_id: str) -> list[str]:
        """ある車両個体のキャリブ ID を昇順で返す (無ければ空)。

        calibration.yaml を持つディレクトリだけを数える。同期途中の空ディレクトリ
        を有効なキャリブとして拾わないため。
        """
        vehicle = _safe_component(vehicle_id, "vehicle_id")
        base = self.root / CALIBRATIONS_DIRNAME / vehicle
        if not base.is_dir():
            return []
        return sorted(
            entry.name
            for entry in base.iterdir()
            if (entry / CALIBRATION_FILENAME).is_file()
        )


# --------------------------------------------------------------------------
# 1 ドライブ分 (recorder が書き込む)
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class DriveLayout:
    """``<platform_id>/drives/<drive_id>/`` 配下のパス解決。

    プロパティはすべて副作用が無い (存在しないドライブのパスも解決できる)。
    ディレクトリを作るのは create() だけ。
    """

    root: Path
    drive_id: str

    # ── 成果物 ──────────────────────────────────────────────────────────

    @property
    def manifest(self) -> Path:
        """manifest.yaml。その存在が「ドライブが完成した」印になる (§4.4)。"""
        return self.root / MANIFEST_FILENAME

    @property
    def bags_dir(self) -> Path:
        return self.root / BAGS_DIRNAME

    @property
    def checksums(self) -> Path:
        return self.bags_dir / CHECKSUMS_FILENAME

    @property
    def tags_dir(self) -> Path:
        return self.root / TAGS_DIRNAME

    @property
    def events(self) -> Path:
        return self.tags_dir / EVENTS_FILENAME

    @property
    def health_dir(self) -> Path:
        return self.root / HEALTH_DIRNAME

    @property
    def topic_stats(self) -> Path:
        return self.health_dir / TOPIC_STATS_FILENAME

    @property
    def notes(self) -> Path:
        return self.root / NOTES_FILENAME

    def segment(self, index: int) -> Path:
        """bags/segment_0000.mcap (分割収録の 1 セグメント)。"""
        if index < 0:
            raise ValueError(f"セグメント番号が負: {index}")
        return self.bags_dir / f"{SEGMENT_PREFIX}{index:04d}{SEGMENT_SUFFIX}"

    def segments(self) -> list[Path]:
        """存在するセグメントを名前順で返す (チェックサム計算・検証の対象)。

        読み取りのみ。まだ収録していなければ空リスト。
        """
        if not self.bags_dir.is_dir():
            return []
        return sorted(self.bags_dir.glob(f"{SEGMENT_PREFIX}*{SEGMENT_SUFFIX}"))

    # ── 作成 ────────────────────────────────────────────────────────────

    @property
    def exists(self) -> bool:
        return self.root.is_dir()

    def create(self, *, exist_ok: bool = False) -> None:
        """ドライブのディレクトリ一式を作る (bags/ tags/ health/ を含む)。

        既定で排他的に作る (exist_ok=False) — drive_id の衝突検出をこの
        makedirs に担わせているため (§10)。既存ディレクトリなら FileExistsError。
        """
        os.makedirs(self.root, exist_ok=exist_ok)
        for name in DRIVE_SUBDIRS:
            # root は今作ったばかりなので、ここは重複を気にしなくてよい
            os.makedirs(self.root / name, exist_ok=True)


# --------------------------------------------------------------------------
# データルート
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class DataLayout:
    """データルート (``data/``) のパス解決。

    definitions/ への入口と、収録データ (platform 配下) の入口を分けて持つ。
    catalog.sqlite はルート直下 (全 platform を横断する索引なので)。
    """

    root: Path

    @classmethod
    def from_path(cls, path: str | os.PathLike[str]) -> "DataLayout":
        """設定ファイルの保存先ルート (文字列) から作る。"""
        return cls(root=Path(path).expanduser())

    # ── definitions/ ────────────────────────────────────────────────────

    @property
    def definitions(self) -> DefinitionsLayout:
        """読み取り専用の定義レイアウト。"""
        return DefinitionsLayout(root=self.root / DEFINITIONS_DIRNAME)

    # ── 収録データ ──────────────────────────────────────────────────────

    @property
    def catalog(self) -> Path:
        """catalog.sqlite (全ドライブの索引)。"""
        return self.root / CATALOG_FILENAME

    def platform_dir(self, platform_id: str) -> Path:
        return self.root / _safe_platform_id(platform_id)

    def drives_dir(self, platform_id: str) -> Path:
        return self.platform_dir(platform_id) / DRIVES_DIRNAME

    def drive(self, platform_id: str, drive_id: str) -> DriveLayout:
        """ドライブのレイアウトを解決する。**副作用は無い** (作成はしない)。"""
        drive = _safe_component(drive_id, "drive_id")
        return DriveLayout(root=self.drives_dir(platform_id) / drive, drive_id=drive)

    def create_drive(self, platform_id: str, drive_id: str) -> DriveLayout:
        """ドライブのディレクトリ一式を作って返す (排他的)。

        既に同じ drive_id のディレクトリがあれば FileExistsError。採番の衝突
        検出はこの排他性に依存している (DriveAllocator を参照)。
        """
        drive = self.drive(platform_id, drive_id)
        drive.create()
        return drive

    def iter_drives(self, platform_id: str) -> list[DriveLayout]:
        """収録済みドライブを drive_id 昇順で返す (読み取りのみ)。

        catalog の再構築や `catalog list` が使う。manifest の有無は見ない —
        「manifest が無い = 不完全なドライブ」の判定は呼び出し側の責務
        (不完全なものこそ一覧に出したい場面がある)。
        """
        base = self.drives_dir(platform_id)
        if not base.is_dir():
            return []
        return [
            DriveLayout(root=entry, drive_id=entry.name)
            for entry in sorted(base.iterdir())
            if entry.is_dir()
        ]


# --------------------------------------------------------------------------
# drive_id の採番
# --------------------------------------------------------------------------

# 正規化後の location の最大長。drive_id 全体をファイル名として扱いやすい長さに
# 収めるための上限で、意味的な根拠がある値ではない。
MAX_LOCATION_LENGTH = 40
LOCATION_FALLBACK = "unknown"

# 同一分内の衝突時に試す連番サフィックスの上限。ここまで埋まっているのは
# 通常運用ではありえず、二重起動等の異常を意味する。
MAX_DRIVE_ID_SUFFIX = 99

_DASH_RUN = re.compile(r"-+")


def normalize_location(location: str) -> str:
    """location を drive_id に埋め込める形に正規化する。

    **location には ASCII の slug を推奨する** (nuScenes の log.location も
    ``singapore-onenorth`` のような ASCII slug)。以下の Unicode 対応は規約から
    外れた入力への保険であり、通常運用では ASCII slug を使うこと。

    最初に NFC 正規化を通すのが重要。macOS は NFD、Linux は NFC を使うため、
    NAS を経由すると同じ名前が別バイト列になり、catalog とファイルシステムの
    不一致という追跡困難なバグを生む。

    規則 (可逆である必要はない。location の正は manifest のフィールド):

    - 英数字は小文字化して残す。CJK 等の文字もそのまま残す
    - それ以外 (空白・スラッシュ・記号・アンダースコア) は ``-`` に潰す。
      アンダースコアを残さないのは、それが drive_id のフィールド区切りだから
    - ``-`` の連続は 1 つに畳み、両端からは取り除く
    - MAX_LOCATION_LENGTH で切り詰める
    - 空になったら ``unknown``

    例: ``"Carla/Maps/Town12"`` → ``"carla-maps-town12"``、
    ``"大阪-梅田"`` → ``"大阪-梅田"``
    """
    text = unicodedata.normalize("NFC", location)
    chars = [ch.casefold() if ch.isalnum() else "-" for ch in text]
    slug = _DASH_RUN.sub("-", "".join(chars)).strip("-")
    # 切り詰めで末尾に "-" が現れうるので、もう一度落とす
    slug = slug[:MAX_LOCATION_LENGTH].rstrip("-")
    return slug or LOCATION_FALLBACK


def format_drive_id(
    started_at: datetime,
    vehicle_id: str,
    location: str,
    *,
    suffix: int | None = None,
) -> str:
    """``日付_時刻_車両_場所`` を組み立てる (§10)。

    日付は date_captured、時刻は収録開始時刻 (分まで)。
    例: ``2026-07-16_1030_vehicle01_osaka-umeda``

    suffix は同一分内で衝突したときだけ付く連番 (``..._osaka-umeda_2``)。
    通常運用の ID 形式は変わらない。

    drive_id を ``_`` で分割して各フィールドを取り出せる前提は持たないこと。
    vehicle_id 自体が ``_`` を含みうるし、衝突時にはサフィックスも付く。
    location / vehicle / 日時の正は manifest のフィールドであり、drive_id は
    人間可読な識別子にすぎない。
    """
    vehicle = _safe_component(vehicle_id, "vehicle_id")
    drive_id = (
        f"{started_at:%Y-%m-%d}_{started_at:%H%M}"
        f"_{vehicle}_{normalize_location(location)}"
    )
    return drive_id if suffix is None else f"{drive_id}_{suffix}"


class DriveIdCollisionError(RuntimeError):
    """サフィックスの上限まで試しても空きが無かった。

    同一分内に上限本ものドライブが始まるのは、同一 vehicle + 同一 platform で
    recorder が二重起動している等の異常なので、黙って別 ID を付けて進まず落とす。
    """


@dataclass(frozen=True)
class DriveAllocation:
    """採番の結果。ディレクトリは作成済み。"""

    drive: DriveLayout
    # 採番に使った収録開始時刻。manifest の date_captured はこれから採ること
    # (日を跨ぐ瞬間に drive_id と date_captured が食い違わないように)。
    started_at: datetime

    @property
    def drive_id(self) -> str:
        return self.drive.drive_id


class DriveAllocator:
    """drive_id を採番してドライブのディレクトリを作る。

    衝突の検出はディレクトリ作成の排他性に委ねる (os.makedirs(exist_ok=False))。
    存在確認してから作る方式だと、確認と作成の間に別プロセスが作りうる。

    時刻は差し替え可能 (テストで固定時刻を注入するため。session.py が clock を
    注入可能にしているのと同じ方針)。
    """

    def __init__(
        self,
        layout: DataLayout,
        *,
        clock: Callable[[], datetime] = datetime.now,
    ) -> None:
        self._layout = layout
        self._clock = clock

    def allocate(
        self,
        *,
        platform_id: str,
        vehicle_id: str,
        location: str,
        max_suffix: int = MAX_DRIVE_ID_SUFFIX,
    ) -> DriveAllocation:
        """drive_id を採番し、ディレクトリ一式を作って返す。

        同一分内に別のドライブが始まっていると衝突するので、空きが見つかるまで
        連番サフィックスを付けて採番し直す (§10)。**待たない** — 採番は
        StartRecording のハンドラ内で走るので、待つと CARLA ブリッジの応答
        タイムアウト (30 秒) を超えてしまう。

        max_suffix まで埋まっていたら DriveIdCollisionError。呼び出し側は
        preflight の失敗として扱い、session.abort() できる。
        """
        if max_suffix < 1:
            raise ValueError(f"max_suffix は 1 以上: {max_suffix}")

        # 待たない以上、時刻を読み直す意味は無い。衝突した組でも収録開始時刻は
        # 同じであるべき (started_at は manifest の date_captured の素になる)。
        started_at = self._clock()
        for suffix in range(1, max_suffix + 1):
            drive_id = format_drive_id(
                started_at, vehicle_id, location,
                suffix=None if suffix == 1 else suffix,
            )
            drive = self._layout.drive(platform_id, drive_id)
            try:
                drive.create()
            except FileExistsError:
                continue
            return DriveAllocation(drive=drive, started_at=started_at)

        base = format_drive_id(started_at, vehicle_id, location)
        raise DriveIdCollisionError(
            f"drive_id {base} がサフィックス {max_suffix} まで埋まっている "
            f"(platform={platform_id})。同じ車両・platform で recorder が"
            "二重起動していないか確認すること"
        )
