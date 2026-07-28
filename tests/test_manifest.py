from datetime import datetime
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from shasou_core.schemas.common import DataSource, EgoPoseBackend
from shasou_core.schemas.manifest import ArchiveStatus, DriveManifest, DriveStatus
from shasou_core.schemas.platform import Platform
from shasou_core.version import SCHEMA_VERSION

from shasou_recorder.core.config import DriveOptions, RecorderConfig
from shasou_recorder.core.layout import DataLayout, DriveAllocation, DriveLayout
from shasou_recorder.core.manifest import (
    ManifestWriter,
    SensorConfigError,
    build_manifest,
    dump_manifest,
    load_manifest,
    resolve_sensor_config,
)
from shasou_recorder.core.session import (
    FinalizeStep,
    Finalizers,
    RecordingSession,
    StopReason,
    StopRequest,
)

PLATFORM = "platform_lincoln_6cam-lidar"
VEHICLE = "vehicle01"
CALIB = "calib_v003_2026-07-01"
DRIVE_ID = "2026-07-16_1030_vehicle01_osaka-umeda"
STARTED_AT = datetime(2026, 7, 16, 10, 30, 12)

PLATFORM_YAML = f"""\
platform_id: {PLATFORM}
vehicle_type: lincoln_mkz
sensor_rig:
  - channel: CAM_FRONT
    modality: camera
    expected_hz: 10.0
  - channel: LIDAR_TOP
    modality: lidar
    expected_hz: 20.0
  - channel: RADAR_FRONT
    modality: radar
    expected_hz: 13.0
"""


def platform(yaml_text: str = PLATFORM_YAML) -> Platform:
    return Platform.model_validate(yaml.safe_load(yaml_text))


def config_for(tmp_path: Path, **overrides) -> RecorderConfig:
    fields = {
        "data_root": tmp_path / "data",
        "platform_id": PLATFORM,
        "vehicle_id": VEHICLE,
        "calib_id": CALIB,
        "source": DataSource.REAL,
        "ego_pose_backend": EgoPoseBackend.PPK_INS,
    }
    fields.update(overrides)
    return RecorderConfig(**fields)


def allocation_for(tmp_path: Path, *, started_at: datetime = STARTED_AT) -> DriveAllocation:
    """ディレクトリを作らずに採番結果だけを組み立てる (書き出し先は別途 create)。"""
    drive = DataLayout(root=tmp_path / "data").drive(PLATFORM, DRIVE_ID)
    return DriveAllocation(drive=drive, started_at=started_at)


def build(tmp_path: Path, **kwargs) -> DriveManifest:
    kwargs.setdefault("config", config_for(tmp_path))
    kwargs.setdefault("platform", platform())
    kwargs.setdefault("allocation", allocation_for(tmp_path))
    return build_manifest(**kwargs)


class TestSensorConfig:
    def test_derives_from_platform_and_namespace(self):
        assert resolve_sensor_config(platform()) == {
            "CAM_FRONT": "/shasou/cam_front/image_raw/compressed",
            "LIDAR_TOP": "/shasou/lidar_top/points",
            "RADAR_FRONT": "/shasou/radar_front/points",
        }

    def test_namespace_is_configurable(self):
        resolved = resolve_sensor_config(platform(), namespace="/fleet/car01")

        assert resolved["LIDAR_TOP"] == "/fleet/car01/lidar_top/points"

    def test_override_replaces_only_the_named_channel(self):
        # 実車のベンダドライバ (RealSense 等) は規約と違う名前で publish する
        resolved = resolve_sensor_config(
            platform(),
            overrides={"CAM_FRONT": "/camera/color/image_raw/compressed"},
        )

        assert resolved["CAM_FRONT"] == "/camera/color/image_raw/compressed"
        assert resolved["LIDAR_TOP"] == "/shasou/lidar_top/points"
        assert resolved["RADAR_FRONT"] == "/shasou/radar_front/points"

    def test_override_for_a_channel_outside_the_rig(self):
        # 黙って無視すると「設定したのに効かない」状態になる
        with pytest.raises(SensorConfigError) as excinfo:
            resolve_sensor_config(platform(), overrides={"CAM_BACK": "/cam_back"})

        message = str(excinfo.value)
        assert "CAM_BACK" in message
        assert PLATFORM in message
        # 綴り間違いを直せるよう、実在するチャネルを併記する
        assert "CAM_FRONT" in message

    def test_channels_are_sorted(self):
        # manifest の diff を安定させる
        resolved = resolve_sensor_config(platform())

        assert list(resolved) == sorted(resolved)

    def test_camera_info_is_not_included(self):
        # sensor_config は 1 チャネル 1 トピック (sample_data の源になる主データ)
        values = resolve_sensor_config(platform()).values()

        assert not any("camera_info" in topic for topic in values)


class TestBuild:
    def test_fields_come_from_their_sources(self, tmp_path):
        manifest = build(
            tmp_path,
            options=DriveOptions(location="osaka-umeda", weather="rain", driver="tanaka"),
        )

        assert manifest.drive_id == DRIVE_ID
        assert manifest.platform == PLATFORM
        assert manifest.vehicle == VEHICLE
        assert manifest.calib_id == CALIB
        assert manifest.source is DataSource.REAL
        assert manifest.ego_pose_backend is EgoPoseBackend.PPK_INS
        assert manifest.location == "osaka-umeda"
        assert manifest.weather == "rain"
        assert manifest.driver == "tanaka"
        assert manifest.recorder_version
        assert manifest.schema_version == SCHEMA_VERSION
        assert len(manifest.uuid) == 32

    def test_date_captured_comes_from_started_at(self, tmp_path):
        # 書き出し時刻ではなく採番時刻から採る (日を跨いでも drive_id とずれない)
        allocation = allocation_for(tmp_path, started_at=datetime(2026, 7, 16, 23, 59, 30))
        manifest = build(tmp_path, allocation=allocation)

        assert str(manifest.date_captured) == "2026-07-16"

    def test_uuid_is_unique_per_drive(self, tmp_path):
        assert build(tmp_path).uuid != build(tmp_path).uuid

    def test_location_falls_back_when_unspecified(self, tmp_path):
        # finalizing で落とさない。drive_id 側の既定 (normalize_location) と同じ値
        manifest = build(tmp_path, options=DriveOptions(driver="tanaka"))

        assert manifest.location == "unknown"
        assert manifest.driver == "tanaka"
        assert manifest.weather is None

    def test_sensor_config_uses_config_namespace_and_overrides(self, tmp_path):
        config = config_for(
            tmp_path,
            topic_namespace="/fleet",
            topic_overrides={"CAM_FRONT": "/camera/color/image_raw/compressed"},
        )
        manifest = build(tmp_path, config=config)

        assert manifest.sensor_config == {
            "CAM_FRONT": "/camera/color/image_raw/compressed",
            "LIDAR_TOP": "/fleet/lidar_top/points",
            "RADAR_FRONT": "/fleet/radar_front/points",
        }

    def test_tags_merge_drive_options_and_stop_reason(self, tmp_path):
        options = DriveOptions(route_id="town12_route003", scenario="cut_in")
        session = RecordingSession(DRIVE_ID)
        session.begin_preflight()
        session.start_recording()
        session.request_stop(
            StopRequest(StopReason.SERVICE, detail="route finished", completed=True)
        )

        manifest = build(tmp_path, options=options, tags=session.stop_tags())

        assert manifest.tags == {
            "route_id": "town12_route003",
            "scenario": "cut_in",
            "stop_reason": "service",
            "completed": "true",
            "stop_detail": "route finished",
        }

    def test_tags_default_to_empty(self, tmp_path):
        assert build(tmp_path).tags == {}

    def test_stop_tags_win_over_drive_options(self, tmp_path):
        # 後に確定した知識が勝つ
        manifest = build(
            tmp_path,
            options=DriveOptions(route_id="from_options"),
            tags={"route_id": "from_session"},
        )

        assert manifest.tags["route_id"] == "from_session"

    def test_initial_status(self, tmp_path):
        manifest = build(tmp_path)

        assert manifest.status is DriveStatus.RECORDED
        assert manifest.archive_status is ArchiveStatus.NONE

    def test_core_schema_validation_applies(self, tmp_path):
        # タグキーの規約 (小文字 snake_case) は core が持つ
        with pytest.raises(ValidationError):
            build(tmp_path, tags={"Route_ID": "town12"})

    def test_carla_source_uses_the_gt_backend(self, tmp_path):
        config = config_for(
            tmp_path, source=DataSource.CARLA, ego_pose_backend=EgoPoseBackend.CARLA_GT
        )
        manifest = build(tmp_path, config=config)

        assert manifest.source is DataSource.CARLA
        assert manifest.ego_pose_backend is EgoPoseBackend.CARLA_GT

    def test_override_outside_the_rig_fails_before_recording(self, tmp_path):
        # preflight で resolve_sensor_config を通していれば収録前に落ちる
        config = config_for(tmp_path, topic_overrides={"CAM_BACK": "/cam_back"})

        with pytest.raises(SensorConfigError):
            build(tmp_path, config=config)


class TestWriter:
    def drive(self, tmp_path: Path) -> DriveLayout:
        drive = DataLayout(root=tmp_path / "data").drive(PLATFORM, DRIVE_ID)
        drive.create()
        return drive

    def test_writes_to_the_layout_path(self, tmp_path):
        drive = self.drive(tmp_path)
        writer = ManifestWriter.for_drive(drive, build(tmp_path))

        path = writer.write()

        assert path == drive.manifest
        assert path.is_file()

    def test_roundtrip_through_core_schema(self, tmp_path):
        drive = self.drive(tmp_path)
        manifest = build(
            tmp_path,
            options=DriveOptions(location="osaka-umeda", weather="rain", driver="tanaka"),
            tags={"stop_reason": "signal"},
        )

        path = ManifestWriter.for_drive(drive, manifest).write()

        assert load_manifest(path) == manifest
        # 素の YAML としても core のスキーマで読めること
        assert DriveManifest.model_validate(yaml.safe_load(path.read_text())) == manifest

    def test_japanese_is_readable(self, tmp_path):
        drive = self.drive(tmp_path)
        manifest = build(tmp_path, options=DriveOptions(location="大阪-梅田"))

        path = ManifestWriter.for_drive(drive, manifest).write()

        # エスケープされると運用者が読めない (manifest は直接開くファイル)
        assert "大阪-梅田" in path.read_text(encoding="utf-8")

    def test_field_order_is_preserved(self, tmp_path):
        # 識別 → 出自 → メタ の読み順を保つ (アルファベット順にしない)
        text = dump_manifest(build(tmp_path))

        assert text.splitlines()[0].startswith("drive_id:")

    def files_in(self, drive: DriveLayout) -> list[str]:
        """ドライブ直下のファイル名 (bags/ tags/ health/ は create() が作る)。"""
        return sorted(p.name for p in drive.root.iterdir() if p.is_file())

    def test_no_temp_file_is_left_behind(self, tmp_path):
        drive = self.drive(tmp_path)
        ManifestWriter.for_drive(drive, build(tmp_path)).write()

        assert self.files_in(drive) == ["manifest.yaml"]

    def test_failed_write_leaves_no_temp_file(self, tmp_path, monkeypatch):
        drive = self.drive(tmp_path)
        writer = ManifestWriter.for_drive(drive, build(tmp_path))
        monkeypatch.setattr(
            "shasou_recorder.core.manifest.dump_manifest",
            lambda manifest: (_ for _ in ()).throw(RuntimeError("boom")),
        )

        with pytest.raises(RuntimeError):
            writer.write()

        assert self.files_in(drive) == []

    def test_interrupted_write_leaves_no_temp_file(self, tmp_path, monkeypatch):
        # 書き込み途中で落ちても、壊れた manifest も一時ファイルも残さない
        drive = self.drive(tmp_path)
        writer = ManifestWriter.for_drive(drive, build(tmp_path))

        def boom(src, dst):
            raise OSError("disk full")

        monkeypatch.setattr("shasou_recorder.core.atomicio.os.replace", boom)

        with pytest.raises(OSError):
            writer.write()

        assert self.files_in(drive) == []
        assert not drive.manifest.exists()

    def test_overwrites_an_existing_manifest(self, tmp_path):
        drive = self.drive(tmp_path)
        ManifestWriter.for_drive(drive, build(tmp_path)).write()
        second = build(tmp_path, options=DriveOptions(location="kyoto"))

        ManifestWriter.for_drive(drive, second).write()

        assert load_manifest(drive.manifest).location == "kyoto"


class TestFinalizeIntegration:
    def test_manifest_step_marks_the_drive_complete(self, tmp_path):
        drive = DataLayout(root=tmp_path / "data").drive(PLATFORM, DRIVE_ID)
        drive.create()

        session = RecordingSession(DRIVE_ID)
        session.begin_preflight()
        session.start_recording()
        session.request_stop(StopRequest(StopReason.SIGNAL, detail="ctrl-c"))

        manifest = build(tmp_path, tags=session.stop_tags())
        result = session.finalize(
            Finalizers(
                bag=_Noop(),
                stats=_Noop(),
                events=_Noop(),
                manifest=ManifestWriter.for_drive(drive, manifest),
                catalog=_Noop(),
            )
        )

        assert result.success
        assert result.manifest_written
        assert FinalizeStep.MANIFEST in result.completed_steps
        assert drive.manifest in result.artifacts

        written = load_manifest(drive.manifest)
        assert written.tags == {"stop_reason": "signal", "completed": "false",
                                "stop_detail": "ctrl-c"}


class _Noop:
    """BagWriter / ArtifactWriter / CatalogUpdater のダミー。"""

    def close(self):
        return None

    def write(self):
        return None

    def update(self) -> None:
        return None
