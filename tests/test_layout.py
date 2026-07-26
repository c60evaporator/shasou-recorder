import unicodedata
from datetime import datetime
from pathlib import Path

import pytest

from shasou_recorder.core.layout import (
    MAX_LOCATION_LENGTH,
    DataLayout,
    DriveAllocator,
    DriveIdCollisionError,
    format_drive_id,
    normalize_location,
)

PLATFORM = "platform_lincoln_6cam-lidar"
DRIVE = "2026-07-16_1030_vehicle01_osaka-umeda"
STARTED_AT = datetime(2026, 7, 16, 10, 30, 12)


def layout(tmp_path: Path) -> DataLayout:
    return DataLayout(root=tmp_path / "data")


class TestDefinitionsPaths:
    def test_definition_files(self, tmp_path):
        definitions = layout(tmp_path).definitions
        root = tmp_path / "data" / "definitions"

        assert definitions.root == root
        assert definitions.vehicle_type("lincoln_mkz") == (
            root / "vehicle_types" / "lincoln_mkz.yaml")
        assert definitions.platform(PLATFORM) == root / "platforms" / f"{PLATFORM}.yaml"
        assert definitions.vehicle("vehicle01") == root / "vehicles" / "vehicle01.yaml"

    def test_calibration_paths_are_per_vehicle(self, tmp_path):
        # キャリブ値は個体固有なので vehicle 配下に置く (§10)
        definitions = layout(tmp_path).definitions
        calib_dir = tmp_path / "data" / "definitions" / "calibrations" / "vehicle01" \
            / "calib_v003_2026-07-01"

        assert definitions.calibration_dir("vehicle01", "calib_v003_2026-07-01") \
            == calib_dir
        assert definitions.calibration("vehicle01", "calib_v003_2026-07-01") \
            == calib_dir / "calibration.yaml"
        assert definitions.calibration_report("vehicle01", "calib_v003_2026-07-01") \
            == calib_dir / "report.pdf"

    def test_iter_calib_ids(self, tmp_path):
        definitions = layout(tmp_path).definitions
        for calib_id in ("calib_v002_2026-01-05", "calib_v003_2026-07-01"):
            path = definitions.calibration(calib_id=calib_id, vehicle_id="vehicle01")
            path.parent.mkdir(parents=True)
            path.write_text("calib_id: x\n")
        # calibration.yaml を持たないディレクトリは有効なキャリブとして数えない
        definitions.calibration_dir("vehicle01", "calib_partial_sync").mkdir()

        assert definitions.iter_calib_ids("vehicle01") == [
            "calib_v002_2026-01-05", "calib_v003_2026-07-01"]

    def test_iter_calib_ids_empty_for_unknown_vehicle(self, tmp_path):
        assert layout(tmp_path).definitions.iter_calib_ids("vehicle99") == []

    def test_definitions_layout_has_no_creation_method(self):
        # definitions/ は studio が編集元。recorder から書き込む経路を持たない
        from shasou_recorder.core.layout import DefinitionsLayout
        forbidden = {"create", "mkdir", "write", "makedirs"}
        assert not forbidden & set(dir(DefinitionsLayout))


class TestDrivePaths:
    def test_drive_artifacts(self, tmp_path):
        data = layout(tmp_path)
        drive = data.drive(PLATFORM, DRIVE)
        root = tmp_path / "data" / PLATFORM / "drives" / DRIVE

        assert data.platform_dir(PLATFORM) == tmp_path / "data" / PLATFORM
        assert data.drives_dir(PLATFORM) == tmp_path / "data" / PLATFORM / "drives"
        assert drive.root == root
        assert drive.drive_id == DRIVE
        assert drive.manifest == root / "manifest.yaml"
        assert drive.bags_dir == root / "bags"
        assert drive.checksums == root / "bags" / "checksums.sha256"
        assert drive.tags_dir == root / "tags"
        assert drive.events == root / "tags" / "events.jsonl"
        assert drive.health_dir == root / "health"
        assert drive.topic_stats == root / "health" / "topic_stats.json"
        assert drive.notes == root / "notes.md"

    def test_segment_naming(self, tmp_path):
        drive = layout(tmp_path).drive(PLATFORM, DRIVE)
        assert drive.segment(0).name == "segment_0000.mcap"
        assert drive.segment(1).parent == drive.bags_dir
        assert drive.segment(42).name == "segment_0042.mcap"
        with pytest.raises(ValueError):
            drive.segment(-1)

    def test_catalog_at_data_root(self, tmp_path):
        assert layout(tmp_path).catalog == tmp_path / "data" / "catalog.sqlite"

    def test_from_path(self, tmp_path):
        assert DataLayout.from_path(str(tmp_path / "data")).root == tmp_path / "data"


class TestCreateDrive:
    def test_creates_subdirectories(self, tmp_path):
        data = layout(tmp_path)
        drive = data.create_drive(PLATFORM, DRIVE)

        assert drive.exists
        assert drive.bags_dir.is_dir()
        assert drive.tags_dir.is_dir()
        assert drive.health_dir.is_dir()
        # ファイル自体は各 writer が書く。置き場だけ用意する
        assert not drive.manifest.exists()

    def test_second_create_is_exclusive(self, tmp_path):
        # 採番の衝突検出はこの排他性に依存している
        data = layout(tmp_path)
        data.create_drive(PLATFORM, DRIVE)
        with pytest.raises(FileExistsError):
            data.create_drive(PLATFORM, DRIVE)

    def test_segments_listed_in_order(self, tmp_path):
        drive = layout(tmp_path).create_drive(PLATFORM, DRIVE)
        assert drive.segments() == []

        for index in (1, 0, 2):
            drive.segment(index).write_bytes(b"")
        drive.checksums.write_text("")
        assert drive.segments() == [drive.segment(i) for i in range(3)]

    def test_iter_drives(self, tmp_path):
        data = layout(tmp_path)
        assert data.iter_drives(PLATFORM) == []

        for drive_id in (DRIVE, "2026-07-16_0900_vehicle01_osaka-umeda"):
            data.create_drive(PLATFORM, drive_id)
        assert [d.drive_id for d in data.iter_drives(PLATFORM)] == [
            "2026-07-16_0900_vehicle01_osaka-umeda", DRIVE]


class TestNoSideEffects:
    def test_resolving_paths_creates_nothing(self, tmp_path):
        data = layout(tmp_path)
        definitions = data.definitions
        drive = data.drive(PLATFORM, DRIVE)

        # パス解決は副作用なし。作成は create_drive / create だけが行う
        _ = (
            data.catalog,
            data.platform_dir(PLATFORM),
            data.drives_dir(PLATFORM),
            definitions.vehicle_type("lincoln_mkz"),
            definitions.platform(PLATFORM),
            definitions.vehicle("vehicle01"),
            definitions.calibration("vehicle01", "calib_v003_2026-07-01"),
            definitions.calibration_report("vehicle01", "calib_v003_2026-07-01"),
            drive.manifest,
            drive.bags_dir,
            drive.checksums,
            drive.events,
            drive.topic_stats,
            drive.notes,
            drive.segment(0),
        )

        assert not drive.exists
        assert not (tmp_path / "data").exists()
        assert list(tmp_path.iterdir()) == []

    def test_listing_does_not_create(self, tmp_path):
        data = layout(tmp_path)
        assert data.iter_drives(PLATFORM) == []
        assert data.definitions.iter_calib_ids("vehicle01") == []
        assert data.drive(PLATFORM, DRIVE).segments() == []
        assert list(tmp_path.iterdir()) == []


class TestNormalizeLocation:
    @pytest.mark.parametrize("raw, expected", [
        ("osaka-umeda", "osaka-umeda"),
        ("Carla/Maps/Town12", "carla-maps-town12"),
        ("  Osaka  Umeda ", "osaka-umeda"),
        ("Town12", "town12"),
        (r"C:\maps\town12", "c-maps-town12"),
        ("osaka(梅田)", "osaka-梅田"),
    ])
    def test_normalization(self, raw, expected):
        assert normalize_location(raw) == expected

    def test_japanese_is_kept(self, tmp_path):
        # ASCII slug が推奨だが、規約から外れた入力でも情報を落とさない
        assert normalize_location("大阪-梅田") == "大阪-梅田"

    def test_nfd_and_nfc_agree(self):
        # macOS (NFD) 経由でも Linux (NFC) 経由でも同じバイト列になること。
        # ここが揃わないと catalog とファイルシステムが食い違う
        nfc = unicodedata.normalize("NFC", "がっこう前")
        nfd = unicodedata.normalize("NFD", "がっこう前")
        assert nfd != nfc  # 前提: 2 つは別のバイト列
        assert normalize_location(nfd) == normalize_location(nfc) == nfc

    def test_underscore_never_survives(self):
        # "_" は drive_id のフィールド区切りなので location の中に残さない
        assert "_" not in normalize_location("osaka_umeda_station")
        assert normalize_location("osaka_umeda") == "osaka-umeda"

    def test_empty_and_symbol_only_fall_back(self):
        assert normalize_location("") == "unknown"
        assert normalize_location("///") == "unknown"
        assert normalize_location("   ") == "unknown"

    def test_truncated_without_trailing_dash(self):
        slug = normalize_location("osaka umeda " * 10)
        assert len(slug) <= MAX_LOCATION_LENGTH
        assert not slug.startswith("-") and not slug.endswith("-")


class TestDriveIdFormat:
    def test_format(self):
        started_at = datetime(2026, 7, 16, 10, 30, 45)
        assert format_drive_id(started_at, "vehicle01", "osaka-umeda") == \
            "2026-07-16_1030_vehicle01_osaka-umeda"

    def test_location_normalized_in_id(self):
        started_at = datetime(2026, 7, 16, 9, 5)
        assert format_drive_id(started_at, "vehicle01", "Carla/Maps/Town12") == \
            "2026-07-16_0905_vehicle01_carla-maps-town12"

    def test_suffix_appended(self):
        # サフィックスは衝突時のみ。通常運用の ID 形式は変わらない
        started_at = datetime(2026, 7, 16, 10, 30, 45)
        assert format_drive_id(started_at, "vehicle01", "osaka-umeda", suffix=None) \
            == DRIVE
        assert format_drive_id(started_at, "vehicle01", "osaka-umeda", suffix=2) \
            == "2026-07-16_1030_vehicle01_osaka-umeda_2"


class TestAllocation:
    def _allocator(self, tmp_path, now: datetime = STARTED_AT) -> DriveAllocator:
        # 収録開始時刻を固定して注入する (同一分内の衝突をそのまま再現できる)
        return DriveAllocator(layout(tmp_path), clock=lambda: now)

    def _allocate(self, allocator: DriveAllocator, **kwargs):
        return allocator.allocate(
            platform_id=PLATFORM, vehicle_id="vehicle01", location="osaka-umeda",
            **kwargs)

    def test_allocates_and_creates_directories(self, tmp_path):
        allocation = self._allocate(self._allocator(tmp_path))

        assert allocation.drive_id == DRIVE
        assert allocation.started_at == STARTED_AT
        assert allocation.drive.bags_dir.is_dir()
        assert allocation.drive.tags_dir.is_dir()
        assert allocation.drive.health_dir.is_dir()

    def test_collision_appends_suffix(self, tmp_path):
        # 同一分内に複数本が始まる (CARLA で短いルートを連続実行する場合)。
        # 採番は StartRecording のハンドラ内で走るので、待たずに即決すること
        allocator = self._allocator(tmp_path)
        allocations = [self._allocate(allocator) for _ in range(3)]

        assert [a.drive_id for a in allocations] == [
            DRIVE, f"{DRIVE}_2", f"{DRIVE}_3"]
        for allocation in allocations:
            assert allocation.drive.bags_dir.is_dir()
            assert allocation.drive.tags_dir.is_dir()
            assert allocation.drive.health_dir.is_dir()

    def test_suffix_shares_started_at(self, tmp_path):
        # サフィックス付きでも収録開始時刻は変わらない。manifest の
        # date_captured はここから採るので、drive_id と食い違ってはいけない
        allocator = self._allocator(tmp_path)
        first = self._allocate(allocator)
        second = self._allocate(allocator)

        assert first.started_at == second.started_at == STARTED_AT

    def test_suffix_limit_raises(self, tmp_path):
        allocator = self._allocator(tmp_path)
        for _ in range(3):
            self._allocate(allocator, max_suffix=3)

        with pytest.raises(DriveIdCollisionError) as excinfo:
            self._allocate(allocator, max_suffix=3)

        assert DRIVE in str(excinfo.value)

    def test_max_suffix_must_be_positive(self, tmp_path):
        with pytest.raises(ValueError):
            self._allocate(self._allocator(tmp_path), max_suffix=0)

    def test_existing_drive_dir_is_skipped(self, tmp_path):
        # 前回の収録が残っているディレクトリは (中身が空でも) 再利用しない
        data = layout(tmp_path)
        data.create_drive(PLATFORM, DRIVE)
        allocation = DriveAllocator(data, clock=lambda: STARTED_AT).allocate(
            platform_id=PLATFORM, vehicle_id="vehicle01", location="osaka-umeda")

        assert allocation.drive_id == f"{DRIVE}_2"

    def test_different_platforms_do_not_collide(self, tmp_path):
        # 衝突はディレクトリ単位。platform が違えば同じ分でも共存できる
        allocator = self._allocator(tmp_path)
        first = self._allocate(allocator)
        second = allocator.allocate(
            platform_id="platform_other", vehicle_id="vehicle01",
            location="osaka-umeda")

        assert first.drive_id == second.drive_id == DRIVE


class TestUnsafeComponents:
    @pytest.mark.parametrize("bad", ["", ".", "..", "a/b", "a\\b"])
    def test_rejected_ids(self, tmp_path, bad):
        data = layout(tmp_path)
        with pytest.raises(ValueError):
            data.platform_dir(bad)
        with pytest.raises(ValueError):
            data.drive(PLATFORM, bad)
        with pytest.raises(ValueError):
            data.definitions.vehicle(bad)
        with pytest.raises(ValueError):
            data.definitions.calibration("vehicle01", bad)

    def test_traversal_not_possible_via_drive_id(self, tmp_path):
        with pytest.raises(ValueError):
            layout(tmp_path).drive(PLATFORM, "../../etc")

    def test_traversal_not_possible_via_vehicle_id(self):
        with pytest.raises(ValueError):
            format_drive_id(datetime(2026, 7, 16, 10, 30), "../etc", "osaka")


class TestReservedNames:
    @pytest.mark.parametrize("bad", ["definitions", "Definitions", "catalog.sqlite"])
    def test_platform_id_cannot_shadow_root_entries(self, tmp_path, bad):
        # platform_id はデータルート直下のディレクトリ名になるので、
        # definitions/ や catalog.sqlite と同じパスを指してはならない。
        # 大文字小文字を無視するのは SMB 共有が区別しないため
        data = layout(tmp_path)
        with pytest.raises(ValueError):
            data.platform_dir(bad)
        with pytest.raises(ValueError):
            data.drives_dir(bad)
        with pytest.raises(ValueError):
            data.drive(bad, DRIVE)
        # 同じ ID が別の場所では通る、という不整合を作らない
        with pytest.raises(ValueError):
            data.definitions.platform(bad)

    def test_reserved_names_are_root_only(self, tmp_path):
        # ルート直下と衝突しようがない名前空間には制約を課さない
        data = layout(tmp_path)
        assert data.drive(PLATFORM, "definitions").drive_id == "definitions"
        assert data.definitions.vehicle("definitions").name == "definitions.yaml"
        assert data.definitions.calibration("definitions", "catalog.sqlite").is_absolute()
