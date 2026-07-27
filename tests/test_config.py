from pathlib import Path

import pytest
from pydantic import ValidationError

from shasou_core.constants import TOPIC_NAMESPACE
from shasou_core.schemas.common import DataSource, EgoPoseBackend

from shasou_recorder.core.config import (
    GIB,
    UNKNOWN_VERSION,
    ConfigError,
    DefinitionsConfig,
    DefinitionsProvider,
    DriveOptions,
    RecorderConfig,
    dump_config,
    load_config,
    recorder_version,
    resolve_drive_options,
    validate_source_backend,
)
from shasou_recorder.core.layout import DataLayout

PLATFORM = "platform_lincoln_6cam-lidar"

MINIMAL_YAML = """\
data_root: {data_root}
platform_id: platform_lincoln_6cam-lidar
vehicle_id: vehicle01
calib_id: calib_v003_2026-07-01
source: real
ego_pose_backend: ppk-ins
"""


def write_config(tmp_path: Path, body: str) -> Path:
    path = tmp_path / "recorder.yaml"
    path.write_text(body, encoding="utf-8")
    return path


def minimal(tmp_path: Path, **overrides) -> RecorderConfig:
    fields = {
        "data_root": tmp_path / "data",
        "platform_id": PLATFORM,
        "vehicle_id": "vehicle01",
        "calib_id": "calib_v003_2026-07-01",
        "source": DataSource.REAL,
        "ego_pose_backend": EgoPoseBackend.PPK_INS,
    }
    fields.update(overrides)
    return RecorderConfig(**fields)


class TestLoad:
    def test_load_minimal(self, tmp_path):
        path = write_config(tmp_path, MINIMAL_YAML.format(data_root=tmp_path / "data"))
        config = load_config(path)

        assert config.data_root == tmp_path / "data"
        assert config.platform_id == PLATFORM
        assert config.source is DataSource.REAL
        assert config.ego_pose_backend is EgoPoseBackend.PPK_INS

    def test_roundtrip(self, tmp_path):
        config = minimal(
            tmp_path,
            ros_domain_id=42,
            drive_defaults=DriveOptions(driver="tanaka"),
        )
        path = write_config(tmp_path, dump_config(config))

        assert load_config(path) == config

    def test_roundtrip_keeps_japanese_readable(self, tmp_path):
        # allow_unicode。日本語が \u30xx に化けると設定ファイルが読めなくなる
        config = minimal(tmp_path, drive_defaults=DriveOptions(location="大阪-梅田"))
        text = dump_config(config)

        assert "大阪-梅田" in text
        assert load_config(write_config(tmp_path, text)) == config

    def test_unknown_field_rejected(self, tmp_path):
        path = write_config(
            tmp_path,
            MINIMAL_YAML.format(data_root=tmp_path / "data") + "storage_root: /mnt/nvme\n",
        )
        with pytest.raises(ValidationError):
            load_config(path)

    def test_unknown_field_rejected_in_section(self, tmp_path):
        # 入れ子のセクションでも extra="forbid" が効くこと
        path = write_config(
            tmp_path,
            MINIMAL_YAML.format(data_root=tmp_path / "data")
            + "bag:\n  max_size_mb: 4096\n",
        )
        with pytest.raises(ValidationError):
            load_config(path)

    def test_missing_file(self, tmp_path):
        with pytest.raises(ConfigError):
            load_config(tmp_path / "no-such.yaml")

    def test_empty_file(self, tmp_path):
        with pytest.raises(ConfigError):
            load_config(write_config(tmp_path, ""))

    def test_top_level_must_be_mapping(self, tmp_path):
        with pytest.raises(ConfigError):
            load_config(write_config(tmp_path, "- data_root: /data\n"))

    def test_broken_yaml(self, tmp_path):
        with pytest.raises(ConfigError):
            load_config(write_config(tmp_path, "data_root: [unclosed\n"))


class TestPathExpansion:
    def test_tilde_expanded(self, tmp_path):
        config = minimal(tmp_path, data_root="~/shasou-data")

        assert config.data_root == Path.home() / "shasou-data"

    def test_env_var_expanded(self, tmp_path, monkeypatch):
        monkeypatch.setenv("SHASOU_DATA", str(tmp_path / "nvme"))
        config = minimal(tmp_path, data_root="$SHASOU_DATA/drives")

        assert config.data_root == tmp_path / "nvme" / "drives"

    def test_undefined_env_var_rejected(self, tmp_path, monkeypatch):
        # 放置すると "$SHASOU_UNDEFINED" という名前のディレクトリが実際に作られる
        monkeypatch.delenv("SHASOU_UNDEFINED", raising=False)
        with pytest.raises(ValidationError):
            minimal(tmp_path, data_root="$SHASOU_UNDEFINED/drives")

    def test_expansion_is_idempotent(self, tmp_path, monkeypatch):
        # 再検証 (with_source) を通しても二重展開されないこと
        monkeypatch.setenv("SHASOU_DATA", str(tmp_path / "nvme"))
        config = minimal(
            tmp_path,
            data_root="$SHASOU_DATA/drives",
            source=DataSource.CARLA,
            ego_pose_backend=EgoPoseBackend.CARLA_GT,
        )

        assert config.with_source(DataSource.CARLA).data_root == config.data_root

    def test_symlink_not_resolved(self, tmp_path):
        # data_root が NVMe マウントへの symlink でも、設定に書いたパスのまま持つ
        target = tmp_path / "mnt" / "nvme"
        target.mkdir(parents=True)
        link = tmp_path / "data"
        link.symlink_to(target)

        assert minimal(tmp_path, data_root=link).data_root == link


class TestDefaults:
    def test_defaults(self, tmp_path):
        config = minimal(tmp_path)

        assert config.topic_namespace == TOPIC_NAMESPACE
        assert config.ros_domain_id is None
        assert config.bag.max_size_bytes == 4 * GIB
        assert config.bag.max_duration_sec == 300
        assert config.disk.min_free_bytes == 10 * GIB
        assert config.disk.poll_interval_sec == 5.0
        assert config.definitions.provider is DefinitionsProvider.LOCAL
        assert config.drive_defaults == DriveOptions()

    def test_preflight_timeout_fits_service_budget(self, tmp_path):
        # StartRecording の応答予算は 30 秒 (§8)。トピック待ちだけで使い切ると、
        # drive_id 採番と bag オープンの分が残らない
        assert minimal(tmp_path).preflight.topic_timeout_sec < 30.0

    def test_config_is_frozen(self, tmp_path):
        config = minimal(tmp_path)
        with pytest.raises(ValidationError):
            config.platform_id = "other"

    @pytest.mark.parametrize("bad", [-1, 233])
    def test_ros_domain_id_range(self, tmp_path, bad):
        with pytest.raises(ValidationError):
            minimal(tmp_path, ros_domain_id=bad)


class TestIdValidation:
    @pytest.mark.parametrize("bad", ["..", "../other", "a/b", "", "definitions"])
    def test_platform_id_rejected(self, tmp_path, bad):
        with pytest.raises(ValidationError):
            minimal(tmp_path, platform_id=bad)

    @pytest.mark.parametrize("field", ["vehicle_id", "calib_id"])
    @pytest.mark.parametrize("bad", ["..", "../etc", "a/b", ""])
    def test_traversal_rejected(self, tmp_path, field, bad):
        with pytest.raises(ValidationError):
            minimal(tmp_path, **{field: bad})

    def test_reserved_names_are_platform_only(self, tmp_path):
        # 予約名はデータルート直下だけの制約。vehicle_id は definitions/vehicles/
        # の下なので衝突しようがない
        assert minimal(tmp_path, vehicle_id="definitions").vehicle_id == "definitions"

    @pytest.mark.parametrize("bad", ["shasou", "/shasou/", "/shasou/1cam", "//shasou"])
    def test_topic_namespace_rejected(self, tmp_path, bad):
        with pytest.raises(ValidationError):
            minimal(tmp_path, topic_namespace=bad)

    def test_root_namespace_allowed(self, tmp_path):
        assert minimal(tmp_path, topic_namespace="/").topic_namespace == "/"


class TestDriveOptions:
    def test_later_source_wins(self):
        # 設定 < CLI < サービス。後の層ほどその走行に近い情報を持つ
        resolved = resolve_drive_options(
            DriveOptions(location="osaka-umeda", driver="tanaka", weather="clear"),
            DriveOptions(location="kyoto"),
            DriveOptions(weather="rain"),
        )

        assert resolved.location == "kyoto"    # CLI が設定を上書き
        assert resolved.weather == "rain"      # サービスが設定を上書き
        assert resolved.driver == "tanaka"     # 誰も上書きしていない

    def test_none_does_not_override(self):
        resolved = resolve_drive_options(
            DriveOptions(driver="tanaka"),
            DriveOptions(driver=None),
        )

        assert resolved.driver == "tanaka"

    def test_empty_string_clears(self):
        # "" は「明示的な空」。設定の既定値を今回だけ消す手段
        resolved = resolve_drive_options(
            DriveOptions(weather="rain"),
            DriveOptions(weather=""),
        )

        assert resolved.weather is None

    def test_blank_is_treated_as_empty(self):
        # "--weather ' '" のような入力でも「明示的な空」の表現を 1 つに保つ
        assert DriveOptions(weather="   ").weather == ""
        assert resolve_drive_options(
            DriveOptions(weather="rain"), DriveOptions(weather="  ")
        ).weather is None

    def test_surrounding_whitespace_stripped(self):
        assert DriveOptions(location=" osaka-umeda\n").location == "osaka-umeda"

    def test_no_sources(self):
        assert resolve_drive_options() == DriveOptions()
        assert resolve_drive_options(None, None) == DriveOptions()

    def test_config_defaults_are_the_weakest_layer(self, tmp_path):
        config = minimal(tmp_path, drive_defaults=DriveOptions(driver="tanaka"))
        resolved = resolve_drive_options(config.drive_defaults, DriveOptions(driver="sato"))

        assert resolved.driver == "sato"

    def test_tags_only_carries_tag_fields(self):
        options = DriveOptions(
            location="osaka-umeda", route_id="town12_route003", scenario="cut_in"
        )

        assert options.tags() == {"route_id": "town12_route003", "scenario": "cut_in"}

    def test_tags_omits_empty_values(self):
        assert DriveOptions(route_id="", scenario=None).tags() == {}


class TestSourceBackend:
    def test_carla_requires_gt_backend(self, tmp_path):
        # manifest 書き出し (finalizing) まで発覚しないと 1 ルートが無駄になる
        with pytest.raises(ValidationError):
            minimal(tmp_path, source=DataSource.CARLA,
                    ego_pose_backend=EgoPoseBackend.PPK_INS)

    def test_real_rejects_gt_backend(self, tmp_path):
        with pytest.raises(ValidationError):
            minimal(tmp_path, source=DataSource.REAL,
                    ego_pose_backend=EgoPoseBackend.CARLA_GT)

    def test_valid_pairs(self, tmp_path):
        assert minimal(tmp_path, source=DataSource.CARLA,
                       ego_pose_backend=EgoPoseBackend.CARLA_GT)
        assert minimal(tmp_path, source=DataSource.REAL,
                       ego_pose_backend=EgoPoseBackend.LIO_GRAPH)

    def test_validate_source_backend_is_reusable(self):
        # ros/ が StartRecording の source を当てるときも同じ関数を通す
        validate_source_backend(DataSource.CARLA, EgoPoseBackend.CARLA_GT)
        with pytest.raises(ValueError):
            validate_source_backend(DataSource.CARLA, EgoPoseBackend.NDT_MAP)

    def test_with_source_revalidates(self, tmp_path):
        config = minimal(tmp_path)  # real / ppk-ins
        with pytest.raises(ValidationError):
            config.with_source(DataSource.CARLA)

    def test_with_source_keeps_other_fields(self, tmp_path):
        config = minimal(tmp_path, source=DataSource.CARLA,
                         ego_pose_backend=EgoPoseBackend.CARLA_GT,
                         drive_defaults=DriveOptions(driver="tanaka"))
        switched = config.with_source(DataSource.CARLA)

        assert switched == config


class TestDefinitionsConfig:
    def test_studio_requires_url(self):
        with pytest.raises(ValidationError):
            DefinitionsConfig(provider=DefinitionsProvider.STUDIO)

    def test_studio_with_url(self):
        config = DefinitionsConfig(
            provider=DefinitionsProvider.STUDIO, studio_url="https://studio.example/api"
        )

        assert config.studio_url == "https://studio.example/api"

    def test_local_rejects_url(self):
        # 同期しているつもりで古いローカル定義を読み続ける事故を防ぐ
        with pytest.raises(ValidationError):
            DefinitionsConfig(studio_url="https://studio.example/api")


class TestIntegration:
    def test_data_layout(self, tmp_path):
        config = minimal(tmp_path)
        layout = config.data_layout()

        assert isinstance(layout, DataLayout)
        assert layout.root == tmp_path / "data"
        assert layout.definitions.root == tmp_path / "data" / "definitions"
        assert layout.drives_dir(config.platform_id) == \
            tmp_path / "data" / PLATFORM / "drives"

    def test_data_layout_creates_nothing(self, tmp_path):
        minimal(tmp_path).data_layout().drive(PLATFORM, "2026-07-16_1030_v01_osaka")

        assert not (tmp_path / "data").exists()

    def test_definitions_config_has_no_path_field(self):
        # definitions/ のパスは data_root から導ける (DataLayout.definitions) ので
        # 設定に持たない。2 箇所から決まると食い違う
        assert not {"path", "root", "dir", "definitions_path"} & set(
            DefinitionsConfig.model_fields
        )

    def test_recorder_version(self):
        version = recorder_version()

        assert isinstance(version, str) and version
        # pyproject.toml がまだ無いので、この環境ではフォールバックになる
        assert version == UNKNOWN_VERSION or version[0].isdigit()
