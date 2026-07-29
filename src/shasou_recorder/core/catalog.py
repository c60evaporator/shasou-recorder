"""catalog.sqlite の読み書き (§10 の「全ドライブの索引」)。

finalizing の 5 番目 (§4.4。`session.CatalogUpdater`) がドライブを登録し、
CLI の `catalog list` / `catalog show` (§5.1) と transfer の status 更新
(§0 のワークフロー) が読む。

**catalog は manifest からの派生物で、正は manifest 側 (§2)。**
索引にすぎないので、壊れても `drives/` を走査して作り直せる (`rebuild`)。
この位置づけから 3 つの方針が出る:

- **重い migration を持たない。** スキーマ版が古ければ作り直して rebuild する
  (行はすべて manifest から復元できるので、移行コードを書く価値が無い)
- **status の正も manifest。** catalog だけ更新すると rebuild で巻き戻るので、
  transfer は manifest と catalog の両方を更新すること (transfer は未実装)
- **recorder が自力で進めるのは verified まで** (recorded → transferred →
  verified)。`imported` は studio が Raw 層取り込み時に決める値で、recorder は
  `sync` で受け取って書くだけ。`set_status` が imported を拒まないのはそのため

パスを列に持たない
------------------
data_root は環境ごとに違う (車上 SSD と NAS でマウント点が変わる) ので、絶対パスを
保存すると別環境で開いた catalog が嘘をつく。パスは platform / drive_id から
`DataLayout` で導く (§10 のフォルダ構成を知る場所は layout.py だけ: `DriveRecord.drive`)。

このモジュールは core/ にあるので ROS に依存しない。sqlite3 は標準ライブラリなので
§1.1 の依存表に足すものは無い。
"""

from __future__ import annotations

import os
import sqlite3
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import date, datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Iterator, Mapping, Optional, Sequence

from shasou_core.schemas.common import DataSource, EgoPoseBackend
from shasou_core.schemas.health import TopicStats
from shasou_core.schemas.manifest import ArchiveStatus, DriveManifest, DriveStatus

from .layout import DataLayout, DriveLayout
from .manifest import load_manifest
from .stats import load_topic_stats

# スキーマを変えたら上げる。上げた版で開くと古い catalog は作り直される
# (rebuild で復元できるので移行はしない)。
CATALOG_SCHEMA_VERSION = 1

# ロック待ちの上限。finalizing の応答予算は 30 秒 (§4.4 / §8) なので、ロック待ちで
# そこを食い潰さない値にする。既定値と同じだが、制約をコードに残すため明示する。
CONNECT_TIMEOUT_SEC = 5.0


class CatalogError(RuntimeError):
    """catalog を開けない・スキーマを用意できない。

    登録や検索そのものの失敗 (sqlite3.Error) は包まずにそのまま通す —
    メッセージが十分具体的で、包み直すと情報が減るため (config.py がスキーマ違反を
    通すのと同じ方針)。ここで包むのは「どのファイルか」を添えたい経路だけ。
    """


class CatalogSchemaError(CatalogError):
    """catalog のスキーマ版が recorder より新しい。

    新しい recorder が書いた索引を古い recorder が書き換えると、知らない列を
    落としたまま上書きしてしまう。読み書きせずに止める。
    """


# 1 文ずつ持つ。executescript は実行前に暗黙の COMMIT を入れてしまい、
# 明示トランザクション (BEGIN IMMEDIATE) の中で使えないため。
_SCHEMA_STATEMENTS: tuple[str, ...] = (
    """
    CREATE TABLE drives (
        drive_id         TEXT PRIMARY KEY,
        uuid             TEXT NOT NULL,
        schema_version   TEXT NOT NULL,
        platform         TEXT NOT NULL,
        vehicle          TEXT NOT NULL,
        calib_id         TEXT NOT NULL,
        source           TEXT NOT NULL,
        ego_pose_backend TEXT NOT NULL,
        date_captured    TEXT NOT NULL,
        location         TEXT NOT NULL,
        driver           TEXT,
        weather          TEXT,
        recorder_version TEXT NOT NULL,
        status           TEXT NOT NULL,
        archive_status   TEXT NOT NULL,
        duration_ns      INTEGER,
        message_count    INTEGER,
        indexed_at       TEXT NOT NULL
    )
    """,
    "CREATE INDEX drives_platform_date ON drives (platform, date_captured)",
    "CREATE INDEX drives_status ON drives (status)",
    """
    CREATE TABLE drive_tags (
        drive_id TEXT NOT NULL REFERENCES drives (drive_id) ON DELETE CASCADE,
        key      TEXT NOT NULL,
        value    TEXT NOT NULL,
        PRIMARY KEY (drive_id, key)
    )
    """,
    "CREATE INDEX drive_tags_key_value ON drive_tags (key, value)",
)

# drives の列順 (INSERT と行→DriveRecord の変換で共有する)
_COLUMNS: tuple[str, ...] = (
    "drive_id", "uuid", "schema_version", "platform", "vehicle", "calib_id",
    "source", "ego_pose_backend", "date_captured", "location", "driver", "weather",
    "recorder_version", "status", "archive_status", "duration_ns", "message_count",
    "indexed_at",
)


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


# --------------------------------------------------------------------------
# 1 ドライブ分の行
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class DriveRecord:
    """catalog の 1 行 (+ tags)。manifest の主要フィールドの写し。

    `indexed_at` だけが manifest 由来ではない catalog 固有の記帳で、
    「この行がいつ書かれたか」(収録直後か、transfer の status 更新か、rebuild か)
    を示す。
    """

    drive_id: str
    uuid: str
    schema_version: str
    platform: str
    vehicle: str
    calib_id: str
    source: DataSource
    ego_pose_backend: EgoPoseBackend
    date_captured: date
    location: str
    driver: Optional[str]
    weather: Optional[str]
    recorder_version: str
    status: DriveStatus
    archive_status: ArchiveStatus
    # 収録の規模。topic_stats.json 由来なので、読めなければ None のまま登録する
    # (manifest があれば完成したドライブなので、一覧から落としてはならない)。
    duration_ns: Optional[int] = None
    message_count: Optional[int] = None
    indexed_at: Optional[datetime] = None
    tags: Mapping[str, str] = field(default_factory=dict)

    @classmethod
    def from_manifest(
        cls,
        manifest: DriveManifest,
        *,
        stats: Optional[TopicStats] = None,
        indexed_at: Optional[datetime] = None,
    ) -> "DriveRecord":
        """manifest (+ 任意で topic_stats) から 1 行を組み立てる。"""
        return cls(
            drive_id=manifest.drive_id,
            uuid=manifest.uuid,
            schema_version=manifest.schema_version,
            platform=manifest.platform,
            vehicle=manifest.vehicle,
            calib_id=manifest.calib_id,
            source=manifest.source,
            ego_pose_backend=manifest.ego_pose_backend,
            date_captured=manifest.date_captured,
            location=manifest.location,
            driver=manifest.driver,
            weather=manifest.weather,
            recorder_version=manifest.recorder_version,
            status=manifest.status,
            archive_status=manifest.archive_status,
            duration_ns=None if stats is None else stats.duration_ns,
            message_count=(
                None if stats is None else sum(s.message_count for s in stats.stats)
            ),
            indexed_at=indexed_at,
            tags=dict(manifest.tags),
        )

    def drive(self, layout: DataLayout) -> DriveLayout:
        """このドライブのパスを解決する (副作用は無い)。

        パスを列に持たず、開いている環境の data_root から毎回導く
        (モジュール docstring)。
        """
        return layout.drive(self.platform, self.drive_id)

    # ── SQL との変換 ────────────────────────────────────────────────────

    def _row(self, indexed_at: datetime) -> tuple[Any, ...]:
        # 日付・時刻は ISO 文字列で持つ。sqlite3 の date/datetime アダプタは
        # Python 3.12 で非推奨になったうえ、ISO なら辞書順 = 時系列順なので
        # 範囲検索も文字列比較のまま効く。
        return (
            self.drive_id, self.uuid, self.schema_version, self.platform,
            self.vehicle, self.calib_id, self.source.value,
            self.ego_pose_backend.value, self.date_captured.isoformat(),
            self.location, self.driver, self.weather, self.recorder_version,
            self.status.value, self.archive_status.value, self.duration_ns,
            self.message_count, indexed_at.isoformat(),
        )

    @classmethod
    def _from_row(cls, row: sqlite3.Row, tags: Mapping[str, str]) -> "DriveRecord":
        return cls(
            drive_id=row["drive_id"],
            uuid=row["uuid"],
            schema_version=row["schema_version"],
            platform=row["platform"],
            vehicle=row["vehicle"],
            calib_id=row["calib_id"],
            source=DataSource(row["source"]),
            ego_pose_backend=EgoPoseBackend(row["ego_pose_backend"]),
            date_captured=date.fromisoformat(row["date_captured"]),
            location=row["location"],
            driver=row["driver"],
            weather=row["weather"],
            recorder_version=row["recorder_version"],
            status=DriveStatus(row["status"]),
            archive_status=ArchiveStatus(row["archive_status"]),
            duration_ns=row["duration_ns"],
            message_count=row["message_count"],
            indexed_at=datetime.fromisoformat(row["indexed_at"]),
            tags=dict(tags),
        )


@dataclass
class RebuildResult:
    """`rebuild` の結果。CLI が「何件入って何を飛ばしたか」を出せるようにする。"""

    indexed: int = 0
    # manifest が無い = finalizing が 4 番目まで届かなかった不完全ドライブ (§4.4)
    skipped: list[str] = field(default_factory=list)
    # manifest はあるが読めない (drive_id, 理由)
    failed: list[tuple[str, str]] = field(default_factory=list)
    # ディレクトリの platform_id と manifest.platform が食い違う
    mismatched: list[str] = field(default_factory=list)


def _read_stats(drive: DriveLayout) -> Optional[TopicStats]:
    """topic_stats.json を best-effort で読む (読めなければ None)。

    規模の列は索引の付加情報にすぎないので、ここでの失敗で登録を止めない。
    """
    if not drive.topic_stats.is_file():
        return None
    try:
        return load_topic_stats(drive.topic_stats)
    except (OSError, ValueError):
        # ValueError は json の JSONDecodeError と pydantic の ValidationError の両方
        return None


# --------------------------------------------------------------------------
# catalog 本体
# --------------------------------------------------------------------------


class Catalog:
    """catalog.sqlite への接続。

    並行性 (複数プロセスから触られる)
    --------------------------------
    収録中の recorder (finalizing の登録)、transfer (status 更新)、別端末の
    `catalog list` が同じファイルを触りうるので、既定設定のままにはしない:

    - **WAL**: 読み手が書き手をブロックしない。ただし NFS/SMB 上では WAL を
      張れないので、失敗したら既定のロールバックジャーナルに黙って戻す
      (一覧が待たされる方が、catalog を開けないよりましなため)
    - **timeout**: CONNECT_TIMEOUT_SEC (finalizing の応答予算を食い潰さない)
    - **BEGIN IMMEDIATE**: 遅延トランザクションの読み→書き昇格は busy ハンドラが
      効かず、書き手が複数あると取りこぼす
    - `synchronous` はいじらない。catalog は manifest から作り直せるので、
      電源断でのロストが致命的にならない

    **スレッドセーフではない** (sqlite3 の接続をそのまま持つ)。別スレッドから
    使うなら、そのスレッドで別インスタンスを開くこと。
    """

    def __init__(
        self,
        path: str | os.PathLike[str],
        *,
        clock: Callable[[], datetime] = _utc_now,
        timeout_sec: float = CONNECT_TIMEOUT_SEC,
    ) -> None:
        self.path = Path(path)
        self._clock = clock
        # 古い版を作り直したときにその版を残す (CLI が rebuild を促せるように)
        self.recreated_from_version: Optional[int] = None
        self._connection = self._connect(timeout_sec)
        try:
            self._ensure_schema()
        except BaseException:
            self._connection.close()
            raise

    @classmethod
    def open(cls, path: str | os.PathLike[str], **kwargs) -> "Catalog":
        """catalog.sqlite を開く (無ければ作る)。"""
        return cls(path, **kwargs)

    @classmethod
    def for_layout(cls, layout: DataLayout, **kwargs) -> "Catalog":
        """データルートのレイアウトから開く。パス解決は layout の責務 (§10)。"""
        return cls(layout.catalog, **kwargs)

    # ── 接続とスキーマ ──────────────────────────────────────────────────

    def _connect(self, timeout_sec: float) -> sqlite3.Connection:
        try:
            connection = sqlite3.connect(
                self.path,
                timeout=timeout_sec,
                # 明示トランザクション (BEGIN IMMEDIATE) を自分で張るため
                isolation_level=None,
            )
        except sqlite3.Error as error:
            raise CatalogError(
                f"catalog を開けない: {self.path} ({error})。"
                "データ保存先 (data_root) が存在するか確認すること"
            ) from error

        connection.row_factory = sqlite3.Row
        # タグの CASCADE 削除に要る (接続ごとの設定)
        connection.execute("PRAGMA foreign_keys = ON")
        try:
            connection.execute("PRAGMA journal_mode = WAL")
        except sqlite3.Error:
            # NAS (NFS/SMB) 上では WAL を張れない。既定のジャーナルで続行する
            pass
        return connection

    def _user_version(self) -> int:
        return int(self._connection.execute("PRAGMA user_version").fetchone()[0])

    def _has_table(self, name: str) -> bool:
        row = self._connection.execute(
            "SELECT 1 FROM sqlite_master WHERE type = 'table' AND name = ?", (name,)
        ).fetchone()
        return row is not None

    def _ensure_schema(self) -> None:
        """スキーマを用意する。版が古ければ作り直す (モジュール docstring)。"""
        version = self._user_version()
        if version > CATALOG_SCHEMA_VERSION:
            raise CatalogSchemaError(
                f"catalog のスキーマ版が新しすぎる: {self.path} "
                f"(catalog={version} > recorder={CATALOG_SCHEMA_VERSION})。"
                "recorder を更新すること"
            )
        if version == CATALOG_SCHEMA_VERSION and self._has_table("drives"):
            return

        # 版が古い (または版はあるがテーブルが無い) 場合は作り直す。catalog は
        # manifest からの派生物なので、移行せず rebuild で復元する方が安い。
        recreate = version != 0 or self._has_table("drives")
        with self._write() as connection:
            if recreate:
                self.recreated_from_version = version
                connection.execute("DROP TABLE IF EXISTS drive_tags")
                connection.execute("DROP TABLE IF EXISTS drives")
            for statement in _SCHEMA_STATEMENTS:
                connection.execute(statement)
            connection.execute(f"PRAGMA user_version = {CATALOG_SCHEMA_VERSION}")

    @contextmanager
    def _write(self) -> Iterator[sqlite3.Connection]:
        """書き込みトランザクション。"""
        self._connection.execute("BEGIN IMMEDIATE")
        try:
            yield self._connection
        except BaseException:
            self._connection.rollback()
            raise
        self._connection.commit()

    # ── ライフサイクル ──────────────────────────────────────────────────

    def close(self) -> None:
        self._connection.close()

    def __enter__(self) -> "Catalog":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    # ── 登録 ────────────────────────────────────────────────────────────

    def upsert(self, record: DriveRecord) -> None:
        """1 ドライブを登録する。**drive_id が主キーなので冪等** (再実行で重複しない)。

        finalizing の 5 番目から呼ばれるほか、`sync` が studio 由来の値を反映する
        ときにも使う。`indexed_at` を持たない record には登録時刻を入れる
        (rebuild は全行に同じ時刻を入れたいので自分で埋めてくる)。
        """
        with self._write() as connection:
            self._insert(connection, record)

    def _insert(self, connection: sqlite3.Connection, record: DriveRecord) -> None:
        indexed_at = record.indexed_at or self._clock()
        placeholders = ", ".join("?" * len(_COLUMNS))
        connection.execute(
            f"INSERT OR REPLACE INTO drives ({', '.join(_COLUMNS)}) "
            f"VALUES ({placeholders})",
            record._row(indexed_at),
        )
        # タグは入れ直す。差分更新にすると、消えたタグが残って検索に引っかかる。
        connection.execute("DELETE FROM drive_tags WHERE drive_id = ?", (record.drive_id,))
        connection.executemany(
            "INSERT INTO drive_tags (drive_id, key, value) VALUES (?, ?, ?)",
            [(record.drive_id, key, value) for key, value in record.tags.items()],
        )

    def set_status(self, drive_id: str, status: DriveStatus) -> bool:
        """status を更新する (未登録の drive_id なら False)。

        transfer が recorded → transferred → verified を進める。**recorder が
        自力で進めてよいのは verified まで** (§0)。`imported` は studio が Raw 層
        取り込み時に決める値で、`sync` が受け取って書く経路だけが使うこと。

        **manifest 側も更新すること。** 正は manifest なので、catalog だけ書くと
        rebuild で巻き戻る (モジュール docstring)。
        """
        return self._update_column("status", status.value, drive_id)

    def set_archive_status(self, drive_id: str, archive_status: ArchiveStatus) -> bool:
        """archive_status を更新する (未登録の drive_id なら False)。

        status とは独立の軸 (§0)。manifest 側も更新すること (set_status と同じ)。
        """
        return self._update_column("archive_status", archive_status.value, drive_id)

    def _update_column(self, column: str, value: str, drive_id: str) -> bool:
        with self._write() as connection:
            cursor = connection.execute(
                f"UPDATE drives SET {column} = ?, indexed_at = ? WHERE drive_id = ?",
                (value, self._clock().isoformat(), drive_id),
            )
        return cursor.rowcount > 0

    def delete(self, drive_id: str) -> bool:
        """1 ドライブを索引から外す (ディスク上のデータには触らない)。"""
        with self._write() as connection:
            connection.execute("DELETE FROM drive_tags WHERE drive_id = ?", (drive_id,))
            cursor = connection.execute(
                "DELETE FROM drives WHERE drive_id = ?", (drive_id,)
            )
        return cursor.rowcount > 0

    # ── 参照 ────────────────────────────────────────────────────────────

    def get(self, drive_id: str) -> Optional[DriveRecord]:
        """1 ドライブを引く (`catalog show`)。無ければ None。"""
        row = self._connection.execute(
            "SELECT * FROM drives WHERE drive_id = ?", (drive_id,)
        ).fetchone()
        if row is None:
            return None
        return DriveRecord._from_row(row, self._tags_for([drive_id]).get(drive_id, {}))

    def list_drives(
        self,
        *,
        platform: Optional[str] = None,
        vehicle: Optional[str] = None,
        source: Optional[DataSource] = None,
        status: Optional[DriveStatus] = None,
        archive_status: Optional[ArchiveStatus] = None,
        since: Optional[date] = None,
        until: Optional[date] = None,
        tags: Optional[Mapping[str, str]] = None,
        limit: Optional[int] = None,
    ) -> list[DriveRecord]:
        """条件で絞ってドライブを返す (`catalog list`)。

        指定した条件は AND で結合する。`since` / `until` は date_captured の
        **両端を含む**範囲。`tags` は「そのキーと値をすべて持つドライブ」。

        並びは新しい走行が先 (date_captured 降順 → drive_id 降順)。一覧の用途が
        「直近の収録がどうなったか」の確認なので、古い方から並べても読まれない。
        """
        conditions: list[str] = []
        params: list[Any] = []

        for column, value in (
            ("platform", platform),
            ("vehicle", vehicle),
            ("source", None if source is None else source.value),
            ("status", None if status is None else status.value),
            (
                "archive_status",
                None if archive_status is None else archive_status.value,
            ),
        ):
            if value is not None:
                conditions.append(f"{column} = ?")
                params.append(value)

        if since is not None:
            conditions.append("date_captured >= ?")
            params.append(since.isoformat())
        if until is not None:
            conditions.append("date_captured <= ?")
            params.append(until.isoformat())

        for key, value in (tags or {}).items():
            # タグ 1 件ごとに副問い合わせ。(key, value) のインデックスが効く。
            conditions.append(
                "drive_id IN (SELECT drive_id FROM drive_tags WHERE key = ? AND value = ?)"
            )
            params.extend((key, value))

        sql = "SELECT * FROM drives"
        if conditions:
            sql += " WHERE " + " AND ".join(conditions)
        sql += " ORDER BY date_captured DESC, drive_id DESC"
        if limit is not None:
            sql += " LIMIT ?"
            params.append(limit)

        rows = self._connection.execute(sql, params).fetchall()
        tags_by_drive = self._tags_for([row["drive_id"] for row in rows])
        return [
            DriveRecord._from_row(row, tags_by_drive.get(row["drive_id"], {}))
            for row in rows
        ]

    def _tags_for(self, drive_ids: Sequence[str]) -> dict[str, dict[str, str]]:
        """複数ドライブのタグをまとめて引く (1 行ごとに問い合わせない)。"""
        if not drive_ids:
            return {}
        placeholders = ", ".join("?" * len(drive_ids))
        rows = self._connection.execute(
            f"SELECT drive_id, key, value FROM drive_tags "
            f"WHERE drive_id IN ({placeholders})",
            list(drive_ids),
        ).fetchall()
        tags: dict[str, dict[str, str]] = {}
        for row in rows:
            tags.setdefault(row["drive_id"], {})[row["key"]] = row["value"]
        return tags

    # ── 再構築 ──────────────────────────────────────────────────────────

    def rebuild(
        self, layout: DataLayout, *, platforms: Optional[Iterable[str]] = None
    ) -> RebuildResult:
        """`drives/` を走査して索引を作り直す。

        catalog は manifest からの派生物なので、壊れたり消えたりしたらこれで
        復元できる (モジュール docstring)。**中身を全消ししてから入れ直す** —
        upsert だけだと、ディスクから消したドライブの行が残るため。

        manifest が無いドライブは不完全なので載せない (§4.4)。読めない manifest が
        あっても 1 件で止めず RebuildResult に積んで続行する (索引は 1 件欠けても
        使えるので、全体を落とす方が損)。
        """
        platform_ids = (
            list(platforms) if platforms is not None else layout.iter_platforms()
        )
        result = RebuildResult()
        indexed_at = self._clock()
        records: list[DriveRecord] = []

        for platform_id in platform_ids:
            for drive in layout.iter_drives(platform_id):
                if not drive.manifest.is_file():
                    result.skipped.append(drive.drive_id)
                    continue
                try:
                    manifest = load_manifest(drive.manifest)
                except ValueError as error:
                    # YamlError も pydantic の ValidationError も ValueError 系
                    result.failed.append(
                        (drive.drive_id, f"{type(error).__name__}: {error}")
                    )
                    continue
                if manifest.platform != platform_id:
                    # 正は manifest 側なので manifest の値で登録し、事実だけ報告する
                    result.mismatched.append(drive.drive_id)
                records.append(
                    DriveRecord.from_manifest(
                        manifest, stats=_read_stats(drive), indexed_at=indexed_at
                    )
                )

        with self._write() as connection:
            connection.execute("DELETE FROM drive_tags")
            connection.execute("DELETE FROM drives")
            for record in records:
                self._insert(connection, record)

        result.indexed = len(records)
        return result


# --------------------------------------------------------------------------
# finalizing の 5 番目
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class CatalogWriter:
    """収録したドライブを catalog に登録する (`session.CatalogUpdater` を満たす)。

    **この手順の失敗は収録データを損なわない。** manifest は 1 つ前の手順で
    書けており、catalog は最後の手順なので、ここで落ちてもドライブは完成扱いの
    まま (`FinalizeResult.manifest_written` は True)。索引は後から `rebuild` で
    追いつけるので、呼び出し側は StopRecording の応答に載せて報告するだけでよい。
    """

    catalog: Catalog
    manifest: DriveManifest
    drive: DriveLayout

    def update(self) -> None:
        # 規模の列は update() の時点で topic_stats.json を読む (finalizing の
        # 2 番目で書かれた直後なので確実にある)。収録中に増え続ける統計
        # オブジェクトを持ち回らずに済み、rebuild と同じ経路になる。
        self.catalog.upsert(
            DriveRecord.from_manifest(self.manifest, stats=_read_stats(self.drive))
        )
