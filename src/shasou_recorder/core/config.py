"""recorder の実行時設定 (recorder 独自スキーマ)。

データ契約 (トピック名・型・manifest スキーマ) の正は shasou-core だが、保存先
パス・DDS 設定・閾値といった**実行環境の情報は recorder 独自**に持つ (§1.2)。
studio はこれらを知る必要がなく、core は「recorder と studio が共有すべき契約」
の置き場だから。設定が参照する ID と語彙 (DataSource / EgoPoseBackend) だけは
core の型と整合させる。

2 層に分ける (§5.3)
-------------------
- ``RecorderConfig``: セッション間で変わらない設定 (設定ファイル)
- ``DriveOptions``:   走行ごとに変わる値 (location / weather / driver / notes 等)

型を分けるのは、走行ごとの値が固定設定に紛れ込む余地を減らすため。ただし
「ほぼ固定だが正式フィールドではない値」(driver 等) の置き場は要るので、設定
ファイルは ``drive_defaults`` として **既定値だけ**持てる。

値の優先順位
------------
DriveOptions は 3 経路から来る。**後の層ほどその走行に近い情報を持つ**、という
1 本の規則で優先順位を決める::

    設定ファイル (drive_defaults)  <  CLI 引数  <  StartRecording リクエスト

マージ規則は「**None のフィールドは上書きしない。それ以外 (空文字を含む) は
上書きする**」。つまり ``None`` = 未指定、``""`` = 明示的な空。

- CLI: argparse は未指定なら ``default=None`` を渡すので、そのまま「未指定」に
  なる。``--weather ""`` は「今回は空」を意味し、``drive_defaults.weather`` を
  消す。この対応が成り立つよう、CLI 側は ``default=""`` を使ってはならない
- ROS サービス: ROS メッセージは未設定の文字列を ``""`` として運ぶので、``""``
  を「明示的な空」と解釈すると、クライアントが埋めなかった全フィールドが
  CLI/設定の値を消してしまう。``ros/converters.py`` が **``""`` → ``None`` に
  落としてから** DriveOptions を作ること。サービス経路は「明示的な空」を表現
  できないが、リクエストを組み立てるクライアントがその走行の権威なので問題ない
- マージが終われば「未指定」と「明示的な空」はどちらも「この走行にこの値は
  無い」なので、``resolve_drive_options`` の最終段で ``""`` は ``None`` に畳む
  (manifest の ``weather: str | None`` に空文字が載るのを防ぐ)

このモジュールは core/ にあるので ROS に依存しない。``ros_domain_id`` も
**値を保持するだけ**で、環境変数に設定するのは ros/ の責務 (core が
os.environ を書き換えると副作用が読みにくい)。
"""

from __future__ import annotations

import os
import re
from enum import Enum
from importlib.metadata import PackageNotFoundError
from importlib.metadata import version as _distribution_version
from pathlib import Path
from typing import Annotated, Any, Optional

import yaml
from pydantic import (
    BaseModel,
    BeforeValidator,
    ConfigDict,
    Field,
    field_validator,
    model_validator,
)

from shasou_core.constants import TOPIC_NAMESPACE
from shasou_core.schemas.common import DataSource, EgoPoseBackend

from .layout import DataLayout, safe_component, safe_platform_id

# 単位はフィールド名 (_bytes / _sec / _days) で明示する。バイト量だけは桁が
# 読みにくいので定数を経由する。
GIB = 1024**3

# パッケージが入っていない環境 (PYTHONPATH からの開発実行) 用のフォールバック。
DISTRIBUTION_NAME = "shasou-recorder"
UNKNOWN_VERSION = "0.0.0+unknown"

# ROS の名前空間。先頭は "/"、末尾に "/" を持たない。ルート ("/") のみは許す。
_NAMESPACE_PATTERN = re.compile(r"^/([A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*)?$")


class ConfigError(ValueError):
    """設定ファイル自体の問題 (読めない・YAML が壊れている・マッピングでない)。

    スキーマ違反は pydantic の ValidationError をそのまま通す (フィールド位置
    まで含んだメッセージが十分具体的なので、包み直すと情報が減る)。
    """


# --------------------------------------------------------------------------
# 基底とパス展開
# --------------------------------------------------------------------------


class RecorderModel(BaseModel):
    """recorder の実行時設定の基底。未知フィールドを拒否し、ロード後は不変。

    core の ShasouModel を継承しないのは、実行時設定が studio と共有する契約では
    ないため。継承すると core の SCHEMA_VERSION の管轄下に見えてしまう。
    語彙 (DataSource / EgoPoseBackend) は core から借り、器は recorder が持つ。
    """

    model_config = ConfigDict(extra="forbid", frozen=True)


def _expand_path(value: Any) -> Any:
    """``~`` と環境変数を展開する。車載では ``~/data`` や ``$SHASOU_DATA`` と
    書きたくなるため。

    展開後に ``$`` が残っていたらエラーにする。未定義の環境変数を放置すると、
    ``$SHASOU_DATA`` という名前のディレクトリが実際に作られてしまう。

    **resolve() はしない。** Jetson では data_root が NVMe マウントへの symlink
    であることが多く、解決するとログや catalog に出るパスが運用者の書いた設定と
    食い違う。展開は冪等なので、再検証 (with_source) で二重展開にはならない。
    """
    if not isinstance(value, (str, Path)):
        return value  # 型の誤りは pydantic に報告させる
    text = os.path.expandvars(str(value))
    if "$" in text:
        raise ValueError(
            f"環境変数を展開できなかった: {value!r} → {text!r}。"
            "未定義の変数か、$ を含むパス名"
        )
    return Path(text).expanduser()


ExpandedPath = Annotated[Path, BeforeValidator(_expand_path)]


# --------------------------------------------------------------------------
# 走行ごとの値 (CLI 引数 / StartRecording)
# --------------------------------------------------------------------------


class DriveOptions(RecorderModel):
    """走行ごとに変わる値 (§5.3 の CLI 引数、および StartRecording の可変部)。

    全フィールドが optional。``None`` = 未指定 (この層は何も言っていない)、
    ``""`` = 明示的な空 (下位層の値を消す)。詳細はモジュール docstring を参照。
    """

    location: Optional[str] = None
    weather: Optional[str] = None
    driver: Optional[str] = None
    notes: Optional[str] = None
    # route_id / scenario は manifest の正式フィールドではなく tags に入る (§8)。
    # ここに置くのは、走行ごとに変わるという寿命が上の 4 つと同じで、別の入れ物を
    # 用意すると ros/node.py が 2 つ持ち回ることになるため。
    route_id: Optional[str] = None
    scenario: Optional[str] = None

    @field_validator("*", mode="before")
    @classmethod
    def _strip(cls, value: Any) -> Any:
        # 空白のみ ("--weather ' '") は "" に正規化する。「明示的な空」の表現を
        # 1 つに保たないと、マージ規則が値ごとにぶれる。
        return value.strip() if isinstance(value, str) else value

    def tags(self) -> dict[str, str]:
        """manifest の tags に載せる分だけを取り出す (§8)。

        キーは小文字 snake_case (core の TAG_KEY_PATTERN)。値が無いキーは
        載せない — 空文字のタグは studio の検索でノイズにしかならない。
        """
        return {
            key: value
            for key, value in (("route_id", self.route_id), ("scenario", self.scenario))
            if value
        }


def resolve_drive_options(*sources: Optional[DriveOptions]) -> DriveOptions:
    """走行ごとの値を優先順位順にマージする。**後に渡したものが勝つ。**

    呼び出し側は弱い順に渡す::

        resolve_drive_options(config.drive_defaults, cli_options, service_options)

    None のフィールドは上書きしない (= 未指定)。空文字は上書きする
    (= 明示的な空)。マージ後に残った空文字は None に畳む — 消費側にとって
    「未指定」と「明示的な空」はどちらも「この走行にこの値は無い」だから。
    """
    merged: dict[str, str] = {}
    for source in sources:
        if source is None:
            continue
        merged.update(
            {
                field: value
                for field, value in source.model_dump().items()
                if value is not None
            }
        )
    return DriveOptions(**{field: value or None for field, value in merged.items()})


# --------------------------------------------------------------------------
# 設定ファイルの各セクション
# --------------------------------------------------------------------------

# 既定値の根拠は、実車 (6 カメラ + LiDAR) で想定する書き込みレート 50〜70 MB/s。


class BagConfig(RecorderModel):
    """bag の分割設定。ros/writer.py が rosbag2 の StorageOptions に写す。"""

    # 上記レートで 1 セグメント ≒ 1 分強。転送・再転送の単位として扱いやすく、
    # 1 セグメントが壊れたときの被害範囲も限定される。
    # 0 は「分割しない」(rosbag2 の max_bagfile_size と同じ意味)。
    max_size_bytes: int = Field(default=4 * GIB, ge=0)
    # 低レート収録でも 5 分で区切るための上限。実車では通常サイズ側が先に効く。
    max_duration_sec: int = Field(default=300, ge=0)


class DiskConfig(RecorderModel):
    """ディスク監視の設定 (§6.2)。閾値割れは DISK_FULL の停止要求になる。"""

    # §5.3 の「10 GB 程度」。上記レートで約 170 秒分あり、閾値割れを検知してから
    # finalizing を完走するのに十分な余裕がある。
    min_free_bytes: int = Field(default=10 * GIB, gt=0)
    # §6.2 の 5〜10 秒。ポーリング間に消費するのは約 300 MB で、閾値に対して
    # 十分小さい。
    poll_interval_sec: float = Field(default=5.0, gt=0)


class PreflightConfig(RecorderModel):
    """収録前検証の設定 (§9)。"""

    # **30 秒にしてはならない。** --wait-for-service で preflight を
    # StartRecording ハンドラ内に置くと、トピック待ちだけで応答予算
    # (CARLA ブリッジの RECORDER_RESPONSE_TIMEOUT_SEC = 30 秒) を使い切り、
    # drive_id 採番と bag オープンの分が残らない (drive_id の待機をやめたのと
    # 同じ理由)。ros/node.py はブロッキングな待機をサービス広告前 (ノード起動時)
    # に済ませ、ハンドラ内では再確認だけを行うこと。
    topic_timeout_sec: float = Field(default=20.0, ge=0)


class DefinitionsProvider(str, Enum):
    """定義の取得元 (§11)。当面 LocalFileProvider だけを実装する。"""

    LOCAL = "local"    # definitions/ のローカル YAML を読むだけ
    STUDIO = "studio"  # studio と HTTP 同期する (未実装)


class DefinitionsConfig(RecorderModel):
    """DefinitionProvider の設定 (§11)。

    ``definitions/`` のパスは data_root から導ける (DataLayout.definitions) ので
    フィールドに持たない。ここにあるのは provider 固有の設定だけ。
    """

    provider: DefinitionsProvider = DefinitionsProvider.LOCAL
    studio_url: Optional[str] = None
    sync_timeout_sec: float = Field(default=30.0, gt=0)
    # 最後に同期してからこの日数を超えたら起動時に警告する (§11 の鮮度警告)。
    # 判定の閾値を recorder が持つのは §2 の方針どおり。
    staleness_warn_days: int = Field(default=30, ge=0)

    @model_validator(mode="after")
    def _url_matches_provider(self) -> "DefinitionsConfig":
        if self.provider is DefinitionsProvider.STUDIO and not self.studio_url:
            raise ValueError("provider=studio では studio_url が必須")
        if self.provider is DefinitionsProvider.LOCAL and self.studio_url:
            # 設定したのに効かない状態を作らない (同期しているつもりで
            # ローカルの古い定義を読み続ける、という事故を防ぐ)
            raise ValueError(
                "provider=local では studio_url を書けない "
                "(同期するなら provider=studio、しないなら studio_url を消すこと)"
            )
        return self


# --------------------------------------------------------------------------
# 設定ファイル本体
# --------------------------------------------------------------------------


def validate_source_backend(source: DataSource, backend: EgoPoseBackend) -> None:
    """source と ego_pose_backend の整合を検証する (矛盾なら ValueError)。

    core の DriveManifest が同じ制約を持つが、そこに任せると finalizing の
    manifest 書き出し (§4.4 の 4 番目) で初めて落ちる。収録が終わってから
    manifest が書けず、ドライブが不完全扱いになるので、設定ロード時に弾く。

    サービスモードでは source が StartRecording で上書きされうるので、その経路
    (RecorderConfig.with_source) からも同じ関数を通すこと。
    """
    if source is DataSource.CARLA and backend is not EgoPoseBackend.CARLA_GT:
        raise ValueError(
            f"source=carla では ego_pose_backend=carla-gt であること "
            f"(設定: {backend.value})"
        )
    if source is DataSource.REAL and backend is EgoPoseBackend.CARLA_GT:
        raise ValueError("source=real で ego_pose_backend=carla-gt は使用不可")


class RecorderConfig(RecorderModel):
    """セッション間で変わらない設定 (§5.3)。

    ID 系フィールドは layout と同じ規則で検証する (``../`` や予約名を弾く)。
    設定ファイルの platform_id に ``../`` が入るとデータルートを飛び出す。
    """

    # ── 保存先 ──────────────────────────────────────────────────────────
    data_root: ExpandedPath

    # ── このドライブの出自 (manifest に入る) ────────────────────────────
    platform_id: str
    vehicle_id: str
    calib_id: str
    source: DataSource
    ego_pose_backend: EgoPoseBackend

    # ── ROS / DDS ───────────────────────────────────────────────────────
    # 既定は core の定数を参照する (トピック規約の正は core: §1.2)
    topic_namespace: str = TOPIC_NAMESPACE
    # 省略時は環境変数 (ROS_DOMAIN_ID) に従う。実際に環境変数を設定するのは
    # ros/ の責務で、core は値を保持するだけ。
    ros_domain_id: Optional[int] = Field(default=None, ge=0, le=232)

    # ── 収録の挙動 ──────────────────────────────────────────────────────
    bag: BagConfig = Field(default_factory=BagConfig)
    disk: DiskConfig = Field(default_factory=DiskConfig)
    preflight: PreflightConfig = Field(default_factory=PreflightConfig)
    definitions: DefinitionsConfig = Field(default_factory=DefinitionsConfig)

    # ── 走行ごとの値の既定 ──────────────────────────────────────────────
    # CLI 引数と StartRecording が上書きする (優先順位はモジュール docstring)。
    drive_defaults: DriveOptions = Field(default_factory=DriveOptions)

    # ── 検証 ────────────────────────────────────────────────────────────

    @field_validator("platform_id")
    @classmethod
    def _check_platform_id(cls, value: str) -> str:
        return safe_platform_id(value)

    @field_validator("vehicle_id")
    @classmethod
    def _check_vehicle_id(cls, value: str) -> str:
        return safe_component(value, "vehicle_id")

    @field_validator("calib_id")
    @classmethod
    def _check_calib_id(cls, value: str) -> str:
        return safe_component(value, "calib_id")

    @field_validator("topic_namespace")
    @classmethod
    def _check_namespace(cls, value: str) -> str:
        if not _NAMESPACE_PATTERN.match(value):
            raise ValueError(
                f"topic_namespace が ROS の名前空間として不正: {value!r} "
                "(先頭は '/'、末尾に '/' は付けない。例: /shasou)"
            )
        return value

    @model_validator(mode="after")
    def _check_source_backend(self) -> "RecorderConfig":
        validate_source_backend(self.source, self.ego_pose_backend)
        return self

    # ── 導出 ────────────────────────────────────────────────────────────

    def data_layout(self) -> DataLayout:
        """データディレクトリのレイアウトを返す。パス解決はすべて layout の責務。"""
        return DataLayout.from_path(self.data_root)

    def with_source(self, source: DataSource) -> "RecorderConfig":
        """source を差し替えた複製を返す (StartRecording が source を運ぶ: §8)。

        model_copy(update=...) ではなく再検証を通す。pydantic の model_copy は
        バリデータを走らせないので、carla × ppk-ins のような矛盾が素通りする。
        """
        return RecorderConfig.model_validate({**self.model_dump(), "source": source})


# --------------------------------------------------------------------------
# 読み書き
# --------------------------------------------------------------------------
#
# YAML ヘルパをここに置いたまま別モジュールに切っていないのは、共通化すべき形
# (安全な loader の選択、エラー整形、スキーマ版の扱い) が manifest.py /
# definitions.py を書くまで見えないため。3 例目が出た時点で core/yamlio.py へ
# 移すこと。今切ると、まだ 1 例しかない形を早すぎる抽象として固定してしまう。


def load_config(path: str | os.PathLike[str]) -> RecorderConfig:
    """設定ファイル (YAML) を読む。

    ファイル層の失敗は ConfigError、スキーマ違反は pydantic の ValidationError。
    """
    file_path = Path(path).expanduser()
    try:
        text = file_path.read_text(encoding="utf-8")
    except OSError as error:
        raise ConfigError(f"設定ファイルを読めない: {file_path} ({error})") from error

    try:
        # safe_load であること。設定は NAS 経由や他人の手で置かれうるので、
        # 任意オブジェクトを構築する loader を使ってはならない。
        data = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise ConfigError(f"設定ファイルの YAML が壊れている: {file_path}\n{error}") from error

    if data is None:
        raise ConfigError(f"設定ファイルが空: {file_path}")
    if not isinstance(data, dict):
        raise ConfigError(
            f"設定ファイルのトップレベルはマッピングであること: {file_path} "
            f"({type(data).__name__} が来た)"
        )
    return RecorderConfig.model_validate(data)


def dump_config(config: RecorderConfig) -> str:
    """設定を YAML 文字列にする (`config show` 相当と、読み書きの往復確認用)。

    展開済みの値を書き出すので、``~`` や ``$VAR`` は展開後のパスになる。
    """
    return yaml.safe_dump(
        config.model_dump(mode="json"),
        allow_unicode=True,   # location 等に日本語が入りうる
        sort_keys=False,      # フィールドの定義順を保つ (人が読むため)
    )


def recorder_version() -> str:
    """収録に使った recorder の版 (manifest の recorder_version)。

    設定フィールドにしないのは、これが「実際に動いたコード」の来歴だから。
    設定値だと、更新し忘れて実体と静かに乖離しても誰も気づけない。
    """
    try:
        return _distribution_version(DISTRIBUTION_NAME)
    except PackageNotFoundError:
        # まだ pyproject.toml が無く、PYTHONPATH から動かす開発中の経路。
        return UNKNOWN_VERSION
