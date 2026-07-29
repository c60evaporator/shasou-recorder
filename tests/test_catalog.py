import sqlite3
from datetime import date, datetime, timezone
from pathlib import Path

import pytest

from shasou_core.schemas.common import DataSource, EgoPoseBackend
from shasou_core.schemas.health import TopicStat, TopicStats
from shasou_core.schemas.manifest import ArchiveStatus, DriveManifest, DriveStatus

from shasou_recorder.core.catalog import (
    CATALOG_SCHEMA_VERSION,
    Catalog,
    CatalogError,
    CatalogSchemaError,
    CatalogWriter,
    DriveRecord,
)
from shasou_recorder.core.layout import DataLayout, DriveLayout
from shasou_recorder.core.manifest import ManifestWriter
from shasou_recorder.core.session import (
    FinalizeStep,
    Finalizers,
    RecordingSession,
    StopReason,
    StopRequest,
)

PLATFORM = "platform_lincoln_6cam-lidar"
OTHER_PLATFORM = "platform_prius_1cam"
VEHICLE = "vehicle01"
CALIB = "calib_v003_2026-07-01"
DRIVE_ID = "2026-07-16_1030_vehicle01_osaka-umeda"
NOW = datetime(2026, 7, 16, 11, 0, tzinfo=timezone.utc)

SENSOR_CONFIG = {"CAM_FRONT": "/shasou/cam_front/image_raw/compressed"}


def manifest_for(drive_id: str = DRIVE_ID, **overrides) -> DriveManifest:
    fields = {
        "drive_id": drive_id,
        "uuid": "7f3a" * 8,
        "source": DataSource.REAL,
        "platform": PLATFORM,
        "vehicle": VEHICLE,
        "ego_pose_backend": EgoPoseBackend.PPK_INS,
        "calib_id": CALIB,
        "date_captured": date(2026, 7, 16),
        "location": "osaka-umeda",
        "driver": "tanaka",
        "weather": "rain",
        "recorder_version": "1.2.0",
        "sensor_config": dict(SENSOR_CONFIG),
    }
    fields.update(overrides)
    return DriveManifest(**fields)


def stats_for(drive_id: str = DRIVE_ID, *, duration_ns: int = 90_000_000_000) -> TopicStats:
    return TopicStats(
        drive_id=drive_id,
        duration_ns=duration_ns,
        stats=[
            TopicStat(topic_name="/shasou/cam_front/image_raw/compressed",
                      channel="CAM_FRONT", message_count=900),
            TopicStat(topic_name="/shasou/lidar_top/points",
                      channel="LIDAR_TOP", message_count=1800),
        ],
    )


def data_layout(tmp_path: Path) -> DataLayout:
    layout = DataLayout(root=tmp_path / "data")
    layout.root.mkdir(parents=True, exist_ok=True)
    return layout


def catalog_for(tmp_path: Path, **kwargs) -> Catalog:
    kwargs.setdefault("clock", lambda: NOW)
    return Catalog.for_layout(data_layout(tmp_path), **kwargs)


def write_drive(
    layout: DataLayout,
    manifest: DriveManifest,
    *,
    platform_id: str | None = None,
    stats: TopicStats | None = None,
    with_manifest: bool = True,
) -> DriveLayout:
    """ディスク上に 1 ドライブ分の成果物を作る (rebuild のテスト用)。"""
    drive = layout.create_drive(platform_id or manifest.platform, manifest.drive_id)
    if with_manifest:
        ManifestWriter.for_drive(drive, manifest).write()
    if stats is not None:
        drive.topic_stats.write_text(stats.model_dump_json(), encoding="utf-8")
    return drive


def record_for(manifest: DriveManifest, **kwargs) -> DriveRecord:
    return DriveRecord.from_manifest(manifest, **kwargs)


class TestOpen:
    def test_creates_the_catalog_when_missing(self, tmp_path):
        layout = data_layout(tmp_path)
        assert not layout.catalog.exists()

        with Catalog.for_layout(layout) as catalog:
            assert catalog.path == layout.catalog
            assert catalog.list_drives() == []
            assert catalog.recreated_from_version is None

        assert layout.catalog.is_file()

    def test_stamps_the_schema_version(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest_for()))

        with sqlite3.connect(data_layout(tmp_path).catalog) as connection:
            version = connection.execute("PRAGMA user_version").fetchone()[0]
        assert version == CATALOG_SCHEMA_VERSION

    def test_reopening_keeps_the_rows(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest_for()))

        with catalog_for(tmp_path) as catalog:
            assert catalog.recreated_from_version is None
            assert [r.drive_id for r in catalog.list_drives()] == [DRIVE_ID]

    def test_older_schema_is_recreated(self, tmp_path):
        # 索引は manifest から作り直せるので、移行せず作り直して rebuild させる
        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest_for()))
        with sqlite3.connect(data_layout(tmp_path).catalog) as connection:
            connection.execute("PRAGMA user_version = 0")
            connection.execute("CREATE TABLE leftover (x TEXT)")

        with catalog_for(tmp_path) as catalog:
            assert catalog.recreated_from_version == 0
            assert catalog.list_drives() == []

    def test_newer_schema_is_refused(self, tmp_path):
        # 新しい recorder が書いた索引を古い recorder が壊さない
        with catalog_for(tmp_path):
            pass
        with sqlite3.connect(data_layout(tmp_path).catalog) as connection:
            connection.execute(f"PRAGMA user_version = {CATALOG_SCHEMA_VERSION + 1}")

        with pytest.raises(CatalogSchemaError) as excinfo:
            catalog_for(tmp_path)

        assert str(CATALOG_SCHEMA_VERSION) in str(excinfo.value)

    def test_missing_data_root_is_reported_with_the_path(self, tmp_path):
        layout = DataLayout(root=tmp_path / "nonexistent")

        with pytest.raises(CatalogError) as excinfo:
            Catalog.for_layout(layout)

        assert str(layout.catalog) in str(excinfo.value)


class TestUpsert:
    def test_all_manifest_fields_roundtrip(self, tmp_path):
        manifest = manifest_for(tags={"route_id": "town12_route003"})

        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest, stats=stats_for()))
            record = catalog.get(DRIVE_ID)

        assert record == DriveRecord.from_manifest(
            manifest, stats=stats_for(), indexed_at=NOW
        )
        assert record.date_captured == date(2026, 7, 16)
        assert record.source is DataSource.REAL
        assert record.status is DriveStatus.RECORDED
        assert record.archive_status is ArchiveStatus.NONE
        assert record.duration_ns == 90_000_000_000
        assert record.message_count == 2700
        assert record.indexed_at == NOW

    def test_is_idempotent(self, tmp_path):
        # finalizing が二度走っても (復旧の再実行等) 行は 1 つ
        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest_for()))
            catalog.upsert(record_for(manifest_for(location="kyoto")))

            drives = catalog.list_drives()

        assert len(drives) == 1
        assert drives[0].location == "kyoto"

    def test_without_stats_the_size_columns_are_null(self, tmp_path):
        # manifest があれば完成したドライブなので、一覧から落としてはならない
        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest_for()))
            record = catalog.get(DRIVE_ID)

        assert record.duration_ns is None
        assert record.message_count is None

    def test_get_returns_none_for_unknown_drive(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            assert catalog.get("no-such-drive") is None

    def test_tags_are_stored_and_returned(self, tmp_path):
        manifest = manifest_for(tags={"route_id": "town12", "scenario": "cut_in"})

        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest))

            assert catalog.get(DRIVE_ID).tags == {
                "route_id": "town12", "scenario": "cut_in",
            }

    def test_removed_tags_do_not_linger(self, tmp_path):
        # 差分更新にすると、消えたタグが検索に引っかかり続ける
        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest_for(tags={"route_id": "town12"})))
            catalog.upsert(record_for(manifest_for(tags={"scenario": "cut_in"})))

            assert catalog.get(DRIVE_ID).tags == {"scenario": "cut_in"}
            assert catalog.list_drives(tags={"route_id": "town12"}) == []

    def test_delete_removes_the_row_and_its_tags(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest_for(tags={"route_id": "town12"})))

            assert catalog.delete(DRIVE_ID) is True
            assert catalog.delete(DRIVE_ID) is False
            assert catalog.list_drives(tags={"route_id": "town12"}) == []

    def test_path_is_derived_not_stored(self, tmp_path):
        # data_root は環境ごとに違うので、開いている環境から導く
        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest_for()))
            record = catalog.get(DRIVE_ID)

        elsewhere = DataLayout(root=tmp_path / "nas")
        assert record.drive(elsewhere).root == (
            tmp_path / "nas" / PLATFORM / "drives" / DRIVE_ID
        )


class TestStatus:
    def test_updates_status_and_archive_status(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest_for()))

            assert catalog.set_status(DRIVE_ID, DriveStatus.TRANSFERRED) is True
            assert catalog.get(DRIVE_ID).status is DriveStatus.TRANSFERRED

            assert catalog.set_status(DRIVE_ID, DriveStatus.VERIFIED) is True
            assert catalog.set_archive_status(DRIVE_ID, ArchiveStatus.ARCHIVED) is True

            record = catalog.get(DRIVE_ID)

        # status と archive_status は独立の軸
        assert record.status is DriveStatus.VERIFIED
        assert record.archive_status is ArchiveStatus.ARCHIVED

    def test_unknown_drive_returns_false(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            assert catalog.set_status("no-such-drive", DriveStatus.VERIFIED) is False
            assert (
                catalog.set_archive_status("no-such-drive", ArchiveStatus.GLACIER)
                is False
            )

    def test_imported_can_be_written(self, tmp_path):
        # recorder が自力で付ける値ではないが、studio からの書き戻しを sync が入れる
        with catalog_for(tmp_path) as catalog:
            catalog.upsert(record_for(manifest_for()))
            catalog.set_status(DRIVE_ID, DriveStatus.IMPORTED)

            assert catalog.get(DRIVE_ID).status is DriveStatus.IMPORTED

    def test_indexed_at_follows_the_update(self, tmp_path):
        later = datetime(2026, 7, 20, 9, 0, tzinfo=timezone.utc)
        clock = {"now": NOW}

        with catalog_for(tmp_path, clock=lambda: clock["now"]) as catalog:
            catalog.upsert(record_for(manifest_for()))
            clock["now"] = later
            catalog.set_status(DRIVE_ID, DriveStatus.TRANSFERRED)

            assert catalog.get(DRIVE_ID).indexed_at == later


class TestSearch:
    def populate(self, catalog: Catalog) -> None:
        catalog.upsert(record_for(manifest_for(
            "2026-07-14_0900_vehicle01_kyoto",
            date_captured=date(2026, 7, 14), location="kyoto",
            tags={"route_id": "town12", "scenario": "cut_in"},
        )))
        catalog.upsert(record_for(manifest_for(
            "2026-07-16_1030_vehicle01_osaka-umeda",
            date_captured=date(2026, 7, 16),
            status=DriveStatus.TRANSFERRED, tags={"route_id": "town12"},
        )))
        catalog.upsert(record_for(manifest_for(
            "2026-07-18_1200_vehicle02_town12",
            date_captured=date(2026, 7, 18), platform=OTHER_PLATFORM,
            vehicle="vehicle02", source=DataSource.CARLA,
            ego_pose_backend=EgoPoseBackend.CARLA_GT, location="town12",
        )))

    def test_returns_everything_newest_first(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            self.populate(catalog)

            assert [r.date_captured.isoformat() for r in catalog.list_drives()] == [
                "2026-07-18", "2026-07-16", "2026-07-14",
            ]

    def test_filters_by_platform_and_vehicle(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            self.populate(catalog)

            assert len(catalog.list_drives(platform=PLATFORM)) == 2
            assert len(catalog.list_drives(platform=OTHER_PLATFORM)) == 1
            assert len(catalog.list_drives(vehicle="vehicle02")) == 1

    def test_filters_by_status_and_source(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            self.populate(catalog)

            transferred = catalog.list_drives(status=DriveStatus.TRANSFERRED)
            assert [r.drive_id for r in transferred] == [DRIVE_ID]
            assert len(catalog.list_drives(status=DriveStatus.RECORDED)) == 2
            assert len(catalog.list_drives(source=DataSource.CARLA)) == 1
            assert len(catalog.list_drives(archive_status=ArchiveStatus.NONE)) == 3

    def test_filters_by_date_range_inclusively(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            self.populate(catalog)

            window = catalog.list_drives(
                since=date(2026, 7, 14), until=date(2026, 7, 16)
            )

            assert [r.date_captured.isoformat() for r in window] == [
                "2026-07-16", "2026-07-14",
            ]
            assert len(catalog.list_drives(since=date(2026, 7, 17))) == 1

    def test_filters_by_tags(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            self.populate(catalog)

            assert len(catalog.list_drives(tags={"route_id": "town12"})) == 2
            # 複数タグは AND
            both = catalog.list_drives(tags={"route_id": "town12", "scenario": "cut_in"})
            assert [r.drive_id for r in both] == ["2026-07-14_0900_vehicle01_kyoto"]
            assert catalog.list_drives(tags={"route_id": "nope"}) == []

    def test_conditions_combine_with_and(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            self.populate(catalog)

            assert catalog.list_drives(
                platform=PLATFORM, status=DriveStatus.TRANSFERRED
            )[0].drive_id == DRIVE_ID
            assert catalog.list_drives(
                platform=OTHER_PLATFORM, status=DriveStatus.TRANSFERRED
            ) == []

    def test_limit(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            self.populate(catalog)

            latest = catalog.list_drives(limit=1)

            assert [r.drive_id for r in latest] == ["2026-07-18_1200_vehicle02_town12"]

    def test_tags_come_back_with_each_row(self, tmp_path):
        with catalog_for(tmp_path) as catalog:
            self.populate(catalog)

            by_id = {r.drive_id: r.tags for r in catalog.list_drives()}

        assert by_id["2026-07-14_0900_vehicle01_kyoto"] == {
            "route_id": "town12", "scenario": "cut_in",
        }
        assert by_id["2026-07-18_1200_vehicle02_town12"] == {}


class TestRebuild:
    def test_restores_from_manifests(self, tmp_path):
        layout = data_layout(tmp_path)
        write_drive(layout, manifest_for(tags={"route_id": "town12"}),
                    stats=stats_for())
        write_drive(layout, manifest_for("2026-07-18_1200_vehicle02_town12",
                                         platform=OTHER_PLATFORM, vehicle="vehicle02"))

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            result = catalog.rebuild(layout)
            drives = catalog.list_drives()

        assert result.indexed == 2
        assert result.skipped == [] and result.failed == [] and result.mismatched == []
        assert {r.drive_id for r in drives} == {
            DRIVE_ID, "2026-07-18_1200_vehicle02_town12",
        }
        # tags と規模の列も復元される
        restored = {r.drive_id: r for r in drives}
        assert restored[DRIVE_ID].tags == {"route_id": "town12"}
        assert restored[DRIVE_ID].message_count == 2700

    def test_matches_what_finalizing_registered(self, tmp_path):
        layout = data_layout(tmp_path)
        manifest = manifest_for(tags={"route_id": "town12"})
        drive = write_drive(layout, manifest, stats=stats_for())

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            CatalogWriter(catalog=catalog, manifest=manifest, drive=drive).update()
            registered = catalog.list_drives()

            catalog.rebuild(layout)
            rebuilt = catalog.list_drives()

        assert rebuilt == registered

    def test_drives_without_a_manifest_are_skipped(self, tmp_path):
        # manifest が無い = finalizing が届かなかった不完全なドライブ (§4.4)
        layout = data_layout(tmp_path)
        write_drive(layout, manifest_for())
        write_drive(layout, manifest_for("2026-07-17_0800_vehicle01_kobe"),
                    with_manifest=False)

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            result = catalog.rebuild(layout)

            assert result.indexed == 1
            assert result.skipped == ["2026-07-17_0800_vehicle01_kobe"]
            assert [r.drive_id for r in catalog.list_drives()] == [DRIVE_ID]

    def test_broken_manifest_is_reported_and_the_rest_continues(self, tmp_path):
        layout = data_layout(tmp_path)
        broken = write_drive(layout, manifest_for("2026-07-17_0800_vehicle01_kobe"))
        broken.manifest.write_text("drive_id: [壊れている", encoding="utf-8")
        write_drive(layout, manifest_for())

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            result = catalog.rebuild(layout)

            assert result.indexed == 1
            assert [drive_id for drive_id, _ in result.failed] == [
                "2026-07-17_0800_vehicle01_kobe"
            ]
            assert [r.drive_id for r in catalog.list_drives()] == [DRIVE_ID]

    def test_rows_for_drives_removed_from_disk_disappear(self, tmp_path):
        # upsert だけの再構築だと残ってしまう
        layout = data_layout(tmp_path)
        write_drive(layout, manifest_for())

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            catalog.upsert(record_for(manifest_for("2026-07-10_0800_vehicle01_gone")))

            catalog.rebuild(layout)

            assert [r.drive_id for r in catalog.list_drives()] == [DRIVE_ID]

    def test_platform_mismatch_is_reported_but_indexed(self, tmp_path):
        # 正は manifest 側なので manifest の値で載せ、食い違いは事実として報告する
        layout = data_layout(tmp_path)
        write_drive(layout, manifest_for(), platform_id=OTHER_PLATFORM)

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            result = catalog.rebuild(layout)

            assert result.mismatched == [DRIVE_ID]
            assert catalog.get(DRIVE_ID).platform == PLATFORM

    def test_can_be_limited_to_platforms(self, tmp_path):
        layout = data_layout(tmp_path)
        write_drive(layout, manifest_for())
        write_drive(layout, manifest_for("2026-07-18_1200_vehicle02_town12",
                                         platform=OTHER_PLATFORM, vehicle="vehicle02"))

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            catalog.rebuild(layout, platforms=[PLATFORM])

            assert [r.drive_id for r in catalog.list_drives()] == [DRIVE_ID]

    def test_empty_data_root(self, tmp_path):
        layout = data_layout(tmp_path)

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            result = catalog.rebuild(layout)

            assert result.indexed == 0
            assert catalog.list_drives() == []


class TestConcurrency:
    def test_a_second_connection_sees_committed_rows(self, tmp_path):
        # 収録中の recorder と、別端末の catalog list が同時に触りうる
        layout = data_layout(tmp_path)

        with Catalog.for_layout(layout) as writer, Catalog.for_layout(layout) as reader:
            writer.upsert(record_for(manifest_for()))

            assert [r.drive_id for r in reader.list_drives()] == [DRIVE_ID]

            writer.set_status(DRIVE_ID, DriveStatus.VERIFIED)
            assert reader.get(DRIVE_ID).status is DriveStatus.VERIFIED


class TestFinalizeIntegration:
    def test_catalog_step_registers_the_drive(self, tmp_path):
        layout = data_layout(tmp_path)
        manifest = manifest_for()
        drive = write_drive(layout, manifest, stats=stats_for())

        session = RecordingSession(DRIVE_ID)
        session.begin_preflight()
        session.start_recording()
        session.request_stop(StopRequest(StopReason.SERVICE, completed=True))

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            result = session.finalize(
                Finalizers(
                    bag=_Noop(),
                    stats=_Noop(),
                    events=_Noop(),
                    manifest=ManifestWriter.for_drive(drive, manifest),
                    catalog=CatalogWriter(
                        catalog=catalog, manifest=manifest, drive=drive
                    ),
                )
            )
            record = catalog.get(DRIVE_ID)

        assert result.success
        assert FinalizeStep.CATALOG in result.completed_steps
        # catalog はファイルを産まないので成果物には載らない
        assert result.artifacts == [drive.manifest]
        assert record.status is DriveStatus.RECORDED
        # 直前の手順で書かれた topic_stats.json から規模を拾う
        assert record.duration_ns == 90_000_000_000
        assert record.message_count == 2700

    def test_missing_topic_stats_does_not_fail_the_step(self, tmp_path):
        layout = data_layout(tmp_path)
        manifest = manifest_for()
        drive = write_drive(layout, manifest)

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            CatalogWriter(catalog=catalog, manifest=manifest, drive=drive).update()

            assert catalog.get(DRIVE_ID).duration_ns is None

    def test_broken_topic_stats_does_not_fail_the_step(self, tmp_path):
        layout = data_layout(tmp_path)
        manifest = manifest_for()
        drive = write_drive(layout, manifest)
        drive.topic_stats.write_text("{壊れている", encoding="utf-8")

        with Catalog.for_layout(layout, clock=lambda: NOW) as catalog:
            CatalogWriter(catalog=catalog, manifest=manifest, drive=drive).update()

            assert catalog.get(DRIVE_ID).message_count is None


class _Noop:
    """BagWriter / ArtifactWriter / CatalogUpdater のダミー。"""

    def close(self):
        return None

    def write(self):
        return None

    def update(self) -> None:
        return None
