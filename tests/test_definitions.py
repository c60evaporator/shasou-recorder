from pathlib import Path

import pytest

from shasou_core.schemas.calibration import CalibrationSet
from shasou_core.schemas.common import DataSource, EgoPoseBackend
from shasou_core.schemas.platform import Platform
from shasou_core.schemas.vehicle import (
    BrakeNormalization,
    SpeedSignRule,
    Vehicle,
    VehicleType,
)

from shasou_recorder.core.config import (
    DefinitionsConfig,
    DefinitionsProvider,
    RecorderConfig,
)
from shasou_recorder.core.definitions import (
    DefinitionError,
    DefinitionInvalidError,
    DefinitionNotFoundError,
    DefinitionProvider,
    LocalFileProvider,
    ResolvedDefinitions,
    create_provider,
    resolve_and_validate,
    resolve_definitions,
    validate_definitions,
)
from shasou_recorder.core.layout import DataLayout

PLATFORM = "platform_lincoln_6cam-lidar"
VEHICLE = "vehicle01"
CALIB = "calib_v003_2026-07-01"
VEHICLE_TYPE = "lincoln_mkz"

PLATFORM_YAML = f"""\
platform_id: {PLATFORM}
vehicle_type: {VEHICLE_TYPE}
sensor_rig:
  - channel: CAM_FRONT
    modality: camera
    expected_hz: 10.0
    camera:
      width_px: 1600
      height_px: 900
      intrinsics_model: pinhole_plumb_bob
  - channel: LIDAR_TOP
    modality: lidar
    expected_hz: 20.0
"""

VEHICLE_YAML = f"""\
vehicle_id: {VEHICLE}
platform_id: {PLATFORM}
can_overrides:
  brake_normalization: pressure
"""

VEHICLE_TYPE_YAML = f"""\
vehicle_type_id: {VEHICLE_TYPE}
wheelbase_m: 2.85
can_defaults:
  speed_sign_rule: signed
  brake_normalization: stroke
"""

CALIBRATION_YAML = f"""\
calib_id: {CALIB}
vehicle: {VEHICLE}
captured_at: 2026-07-01
entries:
  - channel: CAM_FRONT
    extrinsics:
      translation: {{x: 1.0, y: 0.0, z: 1.5}}
      rotation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
    intrinsics:
      model: pinhole_plumb_bob
      fx: 1266.0
      fy: 1266.0
      cx: 800.0
      cy: 450.0
      width: 1600
      height: 900
  - channel: LIDAR_TOP
    extrinsics:
      translation: {{x: 0.9, y: 0.0, z: 1.8}}
      rotation: {{x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
"""


def write_definitions(tmp_path: Path, **overrides: str) -> DataLayout:
    """definitions/ 一式を書いて DataLayout を返す。

    overrides で個別の YAML を差し替えられる (値が "" のファイルは書かない =
    同期漏れの再現)。
    """
    layout = DataLayout(root=tmp_path / "data")
    definitions = layout.definitions
    files = {
        definitions.platform(PLATFORM): overrides.get("platform", PLATFORM_YAML),
        definitions.vehicle(VEHICLE): overrides.get("vehicle", VEHICLE_YAML),
        definitions.vehicle_type(VEHICLE_TYPE): overrides.get(
            "vehicle_type", VEHICLE_TYPE_YAML
        ),
        definitions.calibration(VEHICLE, CALIB): overrides.get(
            "calibration", CALIBRATION_YAML
        ),
    }
    for path, body in files.items():
        if not body:
            continue
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(body, encoding="utf-8")
    return layout


def provider_for(tmp_path: Path, **overrides: str) -> LocalFileProvider:
    return LocalFileProvider(layout=write_definitions(tmp_path, **overrides).definitions)


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


def resolve(provider: LocalFileProvider) -> ResolvedDefinitions:
    return resolve_definitions(
        provider, platform_id=PLATFORM, vehicle_id=VEHICLE, calib_id=CALIB
    )


def codes(result) -> list[str]:
    return [issue.code for issue in result.issues]


def snapshot(root: Path) -> set[tuple[str, int, int]]:
    """definitions/ 配下の (相対パス, サイズ, mtime_ns)。書き込みの検出用。"""
    return {
        (str(path.relative_to(root)), path.stat().st_size, path.stat().st_mtime_ns)
        for path in sorted(root.rglob("*"))
    }


class TestLoad:
    def test_reads_each_definition(self, tmp_path):
        provider = provider_for(tmp_path)

        platform = provider.get_platform(PLATFORM)
        vehicle = provider.get_vehicle(VEHICLE)
        vehicle_type = provider.get_vehicle_type(VEHICLE_TYPE)
        calibration = provider.get_calibration(VEHICLE, CALIB)

        assert isinstance(platform, Platform)
        assert platform.channel_names() == {"CAM_FRONT", "LIDAR_TOP"}
        assert isinstance(vehicle, Vehicle)
        assert vehicle.platform_id == PLATFORM
        assert isinstance(vehicle_type, VehicleType)
        assert vehicle_type.wheelbase_m == 2.85
        assert isinstance(calibration, CalibrationSet)
        assert calibration.channel_names() == {"CAM_FRONT", "LIDAR_TOP"}

    def test_list_calib_ids(self, tmp_path):
        provider = provider_for(tmp_path)
        older = provider.layout.calibration(VEHICLE, "calib_v002_2026-01-05")
        older.parent.mkdir(parents=True)
        older.write_text(CALIBRATION_YAML, encoding="utf-8")

        assert provider.list_calib_ids(VEHICLE) == ["calib_v002_2026-01-05", CALIB]
        assert provider.list_calib_ids("vehicle99") == []

    def test_core_schema_validation_applies(self, tmp_path):
        # チャネル名が示す modality と宣言の不一致は core の Platform が弾く
        broken = PLATFORM_YAML.replace("    modality: camera", "    modality: lidar", 1)
        provider = provider_for(tmp_path, platform=broken)

        with pytest.raises(DefinitionInvalidError) as excinfo:
            provider.get_platform(PLATFORM)

        message = str(excinfo.value)
        # パス (どの定義が悪いか) と pydantic のメッセージ全文の両方が残ること
        assert str(provider.layout.platform(PLATFORM)) in message
        assert "CAM_FRONT" in message

    def test_camera_channel_without_intrinsics_is_rejected(self, tmp_path):
        # core の SensorCalibEntry: カメラチャネルには intrinsics が必須
        without = CALIBRATION_YAML.split("    intrinsics:")[0] + """\
  - channel: LIDAR_TOP
    extrinsics:
      translation: {x: 0.9, y: 0.0, z: 1.8}
      rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"""
        provider = provider_for(tmp_path, calibration=without)

        with pytest.raises(DefinitionInvalidError):
            provider.get_calibration(VEHICLE, CALIB)

    def test_broken_yaml(self, tmp_path):
        provider = provider_for(tmp_path, vehicle="vehicle_id: [unclosed\n")

        with pytest.raises(DefinitionInvalidError) as excinfo:
            provider.get_vehicle(VEHICLE)

        assert str(provider.layout.vehicle(VEHICLE)) in str(excinfo.value)

    def test_empty_file(self, tmp_path):
        provider = provider_for(tmp_path, vehicle="# 中身がない\n")

        with pytest.raises(DefinitionInvalidError, match="空"):
            provider.get_vehicle(VEHICLE)

    def test_non_mapping(self, tmp_path):
        provider = provider_for(tmp_path, vehicle="- vehicle01\n- vehicle02\n")

        with pytest.raises(DefinitionInvalidError, match="マッピング"):
            provider.get_vehicle(VEHICLE)


class TestNotFound:
    """同期漏れの一次診断はメッセージなので、探したパスが必ず入ること。"""

    def test_each_kind_reports_searched_path(self, tmp_path):
        layout = write_definitions(
            tmp_path, platform="", vehicle="", vehicle_type="", calibration=""
        )
        provider = LocalFileProvider(layout=layout.definitions)
        definitions = layout.definitions

        cases = [
            (lambda: provider.get_platform(PLATFORM), definitions.platform(PLATFORM)),
            (lambda: provider.get_vehicle(VEHICLE), definitions.vehicle(VEHICLE)),
            (
                lambda: provider.get_vehicle_type(VEHICLE_TYPE),
                definitions.vehicle_type(VEHICLE_TYPE),
            ),
            (
                lambda: provider.get_calibration(VEHICLE, CALIB),
                definitions.calibration(VEHICLE, CALIB),
            ),
        ]
        for call, expected_path in cases:
            with pytest.raises(DefinitionNotFoundError) as excinfo:
                call()
            assert str(expected_path) in str(excinfo.value)

    def test_missing_calibration_lists_available_ids(self, tmp_path):
        provider = provider_for(tmp_path)

        with pytest.raises(DefinitionNotFoundError) as excinfo:
            provider.get_calibration(VEHICLE, "calib_typo")

        # typo (候補あり) と同期漏れ (候補なし) をメッセージだけで切り分けられる
        assert CALIB in str(excinfo.value)

    def test_missing_calibration_without_any_sync(self, tmp_path):
        provider = provider_for(tmp_path, calibration="")

        with pytest.raises(DefinitionNotFoundError, match="1 つも同期されていない"):
            provider.get_calibration(VEHICLE, CALIB)

    def test_not_found_is_a_definition_error(self, tmp_path):
        # 呼び出し側は DefinitionError だけ捕まえれば済む
        assert issubclass(DefinitionNotFoundError, DefinitionError)
        assert issubclass(DefinitionInvalidError, DefinitionError)


class TestResolve:
    def test_resolves_the_whole_chain(self, tmp_path):
        resolved = resolve(provider_for(tmp_path))

        assert resolved.platform.platform_id == PLATFORM
        assert resolved.vehicle.vehicle_id == VEHICLE
        # VehicleType は platform.vehicle_type を鍵に解決される
        assert resolved.vehicle_type.vehicle_type_id == VEHICLE_TYPE
        assert resolved.calibration.calib_id == CALIB
        assert resolved.requested_platform_id == PLATFORM
        assert resolved.requested_vehicle_id == VEHICLE
        assert resolved.requested_calib_id == CALIB

    def test_platform_comes_from_the_request_not_from_vehicle(self, tmp_path):
        # vehicle が別 platform を名乗っていても、設定の platform_id で引く
        # (食い違いは黙って追従せず Issue として報告する)
        other = VEHICLE_YAML.replace(f"platform_id: {PLATFORM}", "platform_id: platform_other")
        resolved = resolve(provider_for(tmp_path, vehicle=other))

        assert resolved.platform.platform_id == PLATFORM
        assert resolved.vehicle.platform_id == "platform_other"

    def test_missing_definition_raises(self, tmp_path):
        with pytest.raises(DefinitionNotFoundError):
            resolve(provider_for(tmp_path, vehicle_type=""))

    def test_local_file_provider_satisfies_protocol(self, tmp_path):
        assert isinstance(provider_for(tmp_path), DefinitionProvider)


class TestCreateProvider:
    def test_local(self, tmp_path):
        provider = create_provider(config_for(tmp_path))

        assert isinstance(provider, LocalFileProvider)
        assert provider.layout.root == tmp_path / "data" / "definitions"

    def test_creates_nothing(self, tmp_path):
        # 定義の取得元を組み立てるだけ。definitions/ を作らない
        create_provider(config_for(tmp_path))

        assert not (tmp_path / "data").exists()

    def test_studio_is_not_implemented(self, tmp_path):
        config = config_for(
            tmp_path,
            definitions=DefinitionsConfig(
                provider=DefinitionsProvider.STUDIO,
                studio_url="https://studio.example.com",
            ),
        )

        with pytest.raises(NotImplementedError, match="local"):
            create_provider(config)


class TestValidate:
    def test_consistent_definitions(self, tmp_path):
        result = validate_definitions(resolve(provider_for(tmp_path)))

        assert result.ok
        assert result.issues == []

    def test_vehicle_belongs_to_another_platform(self, tmp_path):
        other = VEHICLE_YAML.replace(f"platform_id: {PLATFORM}", "platform_id: platform_other")
        result = validate_definitions(resolve(provider_for(tmp_path, vehicle=other)))

        assert not result.ok
        assert "vehicle_platform_mismatch" in codes(result)

    def test_calibration_belongs_to_another_vehicle(self, tmp_path):
        # キャリブ値は個体固有。他車両へ流用してはならない (§11)
        other = CALIBRATION_YAML.replace(f"vehicle: {VEHICLE}", "vehicle: vehicle02")
        result = validate_definitions(resolve(provider_for(tmp_path, calibration=other)))

        assert not result.ok
        assert "calibration_vehicle_mismatch" in codes(result)

    def test_calibration_missing_a_sensor(self, tmp_path):
        # LIDAR_TOP のキャリブが無い = nuScenes 変換で calibrated_sensor を作れない
        partial = CALIBRATION_YAML.split("  - channel: LIDAR_TOP")[0]
        result = validate_definitions(resolve(provider_for(tmp_path, calibration=partial)))

        assert not result.ok
        missing = [i for i in result.issues if i.code == "calibration_missing"]
        assert [i.context["channel"] for i in missing] == ["LIDAR_TOP"]

    def test_calibration_id_mismatch(self, tmp_path):
        other = CALIBRATION_YAML.replace(f"calib_id: {CALIB}", "calib_id: calib_v001")
        result = validate_definitions(resolve(provider_for(tmp_path, calibration=other)))

        assert not result.ok
        assert "calib_id_mismatch" in codes(result)

    def test_platform_file_declares_another_id(self, tmp_path):
        # ファイル名と中身の食い違い。core には「取得要求」の概念が無いので
        # recorder が見る (code は core の同一照合と揃える)
        other = PLATFORM_YAML.replace(f"platform_id: {PLATFORM}", "platform_id: platform_other")
        result = validate_definitions(resolve(provider_for(tmp_path, platform=other)))

        assert not result.ok
        assert "platform_id_mismatch" in codes(result)

    def test_vehicle_type_file_declares_another_id(self, tmp_path):
        other = VEHICLE_TYPE_YAML.replace(
            f"vehicle_type_id: {VEHICLE_TYPE}", "vehicle_type_id: prius"
        )
        result = validate_definitions(resolve(provider_for(tmp_path, vehicle_type=other)))

        assert not result.ok
        assert "vehicle_type_mismatch" in codes(result)

    def test_extra_calibration_channel_is_a_warning(self, tmp_path):
        extra = CALIBRATION_YAML + """\
  - channel: RADAR_FRONT
    extrinsics:
      translation: {x: 2.0, y: 0.0, z: 0.5}
      rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
"""
        result = validate_definitions(resolve(provider_for(tmp_path, calibration=extra)))

        # platform 外のチャネルは収録に支障がないので WARNING (収録は止めない)
        assert result.ok
        assert codes(result) == ["calibration_extra"]

    def test_intrinsics_model_mismatch(self, tmp_path):
        # 宣言と実測の整合 (core の validate_calibration_against_platform)
        other = CALIBRATION_YAML.replace(
            "      model: pinhole_plumb_bob", "      model: fisheye_equidistant"
        )
        result = validate_definitions(resolve(provider_for(tmp_path, calibration=other)))

        assert not result.ok
        assert "intrinsics_model_mismatch" in codes(result)


class TestEffectiveCanSpec:
    def test_overrides_win_over_type_defaults(self, tmp_path):
        can = resolve(provider_for(tmp_path)).effective_can_spec()

        # 個体上書きがある項目は上書きが勝つ
        assert can.brake_normalization is BrakeNormalization.PRESSURE
        # 上書きの無い項目は車種デフォルトが残る
        assert can.speed_sign_rule is SpeedSignRule.SIGNED

    def test_type_defaults_when_no_overrides(self, tmp_path):
        without = VEHICLE_YAML.split("can_overrides:")[0]
        can = resolve(provider_for(tmp_path, vehicle=without)).effective_can_spec()

        assert can.brake_normalization is BrakeNormalization.STROKE
        assert can.speed_sign_rule is SpeedSignRule.SIGNED

    def test_undefined_stays_none(self, tmp_path):
        # 両方 None の項目は未定義のまま (必要とする側がその時点でエラーにする)
        bare = f"vehicle_type_id: {VEHICLE_TYPE}\n"
        without = VEHICLE_YAML.split("can_overrides:")[0]
        can = resolve(
            provider_for(tmp_path, vehicle_type=bare, vehicle=without)
        ).effective_can_spec()

        assert can.speed_sign_rule is None
        assert can.brake_normalization is None


class TestReadOnly:
    """definitions/ は studio が編集元。recorder は絶対に書き込まない (§11)。"""

    def test_resolution_and_validation_do_not_touch_definitions(self, tmp_path):
        provider = provider_for(tmp_path)
        root = provider.layout.root
        before = snapshot(root)

        resolved = resolve(provider)
        validate_definitions(resolved)
        provider.list_calib_ids(VEHICLE)
        resolved.effective_can_spec()

        assert snapshot(root) == before

    def test_missing_definition_does_not_create_anything(self, tmp_path):
        provider = provider_for(tmp_path, calibration="")
        root = provider.layout.root
        before = snapshot(root)

        with pytest.raises(DefinitionNotFoundError):
            provider.get_calibration(VEHICLE, CALIB)

        assert snapshot(root) == before

    def test_provider_has_no_write_methods(self, tmp_path):
        # 書き込み経路を API から無くしておく (DefinitionsLayout と同じ方針)
        forbidden = {"save", "write", "sync", "put", "update", "delete", "create"}
        methods = {name for name in dir(LocalFileProvider) if not name.startswith("_")}

        assert not methods & forbidden


class TestPreflightBridge:
    def test_returns_definitions_and_result(self, tmp_path):
        config = config_for(tmp_path)
        provider = provider_for(tmp_path)

        resolved, result = resolve_and_validate(provider, config)

        assert result.ok
        assert resolved is not None
        assert resolved.platform.platform_id == PLATFORM

    def test_missing_definition_becomes_an_error_issue(self, tmp_path):
        # preflight が例外処理を持たずに result.ok だけで判断できること
        config = config_for(tmp_path)
        provider = provider_for(tmp_path, calibration="")

        resolved, result = resolve_and_validate(provider, config)

        assert resolved is None
        assert not result.ok
        assert codes(result) == ["definition_unavailable"]
        # 探索パスは Issue のメッセージにも残る
        assert str(provider.layout.calibration(VEHICLE, CALIB)) in result.issues[0].message

    def test_uses_the_ids_from_the_config(self, tmp_path):
        config = config_for(tmp_path, vehicle_id="vehicle02")
        provider = provider_for(tmp_path)

        resolved, result = resolve_and_validate(provider, config)

        assert resolved is None
        assert "vehicle02" in result.issues[0].message
