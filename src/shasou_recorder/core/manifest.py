"""manifest.yaml の生成 (§10) と書き出し (§4.4 の 4 番目)。

**manifest の存在が「ドライブが完成した」印**になる。manifest があれば有効な
ドライブ、無ければ不完全と判定できるので、finalizing では後半に置かれ、
書き出しは原子的でなければならない — 途中で落ちて壊れた manifest が残ると、
完成判定そのものが誤る。

manifest は自己完結する (ディレクトリから自明な情報も冗長に持つ)。スキーマの正は
shasou-core の DriveManifest で、このモジュールは**各フィールドをどこから採るか**
だけを決める:

===================== =======================================================
フィールド            供給元
===================== =======================================================
drive_id              DriveAllocation (layout の採番結果)
date_captured         DriveAllocation.started_at.date()
uuid                  新規採番 (core の new_token)
source / platform /   RecorderConfig (source はサービスモードなら with_source 済み)
vehicle / calib_id /
ego_pose_backend
location / weather /  DriveOptions (設定 < CLI < StartRecording でマージ済み)
driver
recorder_version      config.recorder_version() (実際に動いたコードの来歴)
schema_version        shasou-core の既定 (書き手の版がそのまま入る)
sensor_config         platform の sensor_rig + 名前空間 + 設定の上書き (下記)
tags                  DriveOptions.tags() + session.stop_tags()
status/archive_status core の既定 (recorded / none)
===================== =======================================================

**date_captured を started_at から採るのは、日を跨ぐ瞬間に drive_id と食い違わない
ようにするため** (layout.DriveAllocation の申し送り)。書き出し時刻を使うと、
23:59 に始まって 0:01 に終わったドライブで drive_id の日付と 1 日ずれる。

このモジュールは core/ にあるので ROS に依存しない。
"""

from __future__ import annotations

import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Mapping, Optional

from shasou_core.constants import TOPIC_NAMESPACE
from shasou_core.schemas.common import Modality, new_token
from shasou_core.schemas.manifest import DriveManifest
from shasou_core.schemas.platform import Platform
from shasou_core.schemas.topics import (
    CAMERA_IMAGE,
    LIDAR_POINTS,
    RADAR_POINTS,
    TopicContract,
    resolve_topic_name,
)

from .config import DriveOptions, RecorderConfig, recorder_version
from .layout import LOCATION_FALLBACK, DriveAllocation, DriveLayout
from .yamlio import dump_mapping, load_mapping

# --------------------------------------------------------------------------
# sensor_config の解決
# --------------------------------------------------------------------------

# チャネルの「主データトピック」の契約。sensor_config は 1 チャネル 1 トピックで、
# nuScenes 変換で sample_data の源になるデータ本体を指す — camera_info は同じ
# チャネルの付随情報なので載せない (gt/depth も同様に実センサではない)。
# **命名規約そのものは core が正** (resolve_topic_name)。ここが持つのは
# 「どの契約を主とみなすか」の選択だけ。
PRIMARY_CONTRACTS: dict[Modality, TopicContract] = {
    Modality.CAMERA: CAMERA_IMAGE,
    Modality.LIDAR: LIDAR_POINTS,
    Modality.RADAR: RADAR_POINTS,
}


class SensorConfigError(ValueError):
    """sensor_config を解決できない (設定と platform 定義の食い違い)。

    **preflight で必ず resolve_sensor_config() を通すこと。** ここでの失敗が
    finalizing まで遅れると、収録は済んだのに manifest が書けず、ドライブが
    不完全扱いになる (§4.4)。
    """


def resolve_sensor_config(
    platform: Platform,
    *,
    namespace: str = TOPIC_NAMESPACE,
    overrides: Optional[Mapping[str, str]] = None,
) -> dict[str, str]:
    """「正規チャネル名 → 実トピック名」を解決する。

    既定は platform の sensor_rig と名前空間からの**導出**。CARLA ブリッジは
    規約どおりの名前で publish するのでこれで足りる。実車のベンダドライバは
    規約と違う名前を使うので、`overrides` (設定の topic_overrides) が該当チャネル
    だけを差し替える。

    **sensor_rig に無いチャネルの上書きはエラー。** 1 つの設定は 1 つの platform に
    紐づく (§5.3) ので、rig に無いチャネルの上書きは typo か platform の取り違えで
    しかなく、黙って無視すると「設定したのに効かない」状態を作る。

    購読すべきトピックの一覧としても使える (preflight の存在確認)。ただし
    camera_info・車両状態・gt・tf_static は sensor_config に載らないので、
    購読集合はこれより広い。
    """
    resolved: dict[str, str] = {}
    for spec in platform.sensor_rig:
        contract = PRIMARY_CONTRACTS.get(spec.modality)
        if contract is None:
            # ChannelSpec は CAM_/LIDAR_/RADAR_ しか名乗れず、Platform の
            # validator が名前と modality の一致も見ているので通常は到達しない。
            raise SensorConfigError(
                f"チャネル {spec.channel} の modality ({spec.modality.value}) は "
                "sensor_config に載せられない (camera / lidar / radar のみ)"
            )
        resolved[spec.channel] = resolve_topic_name(namespace, spec.channel, contract)

    for channel, topic in (overrides or {}).items():
        if channel not in resolved:
            raise SensorConfigError(
                f"topic_overrides のチャネル {channel} が platform "
                f"{platform.platform_id} の sensor_rig に無い "
                f"(rig: {', '.join(sorted(resolved))})。"
                "チャネル名の綴りか、設定の platform_id を確認すること"
            )
        resolved[channel] = topic

    # チャネル名昇順にして manifest の diff を安定させる
    return {channel: resolved[channel] for channel in sorted(resolved)}


# --------------------------------------------------------------------------
# manifest の組み立て
# --------------------------------------------------------------------------


def build_manifest(
    *,
    config: RecorderConfig,
    platform: Platform,
    allocation: DriveAllocation,
    options: Optional[DriveOptions] = None,
    tags: Optional[Mapping[str, str]] = None,
    uuid_factory: Callable[[], str] = new_token,
) -> DriveManifest:
    """1 ドライブ分の manifest を組み立てる (core のスキーマ検証を必ず通る)。

    `tags` は session.stop_tags() を渡す想定で、DriveOptions 由来のタグ
    (route_id / scenario) に**後勝ちで**マージする。停止理由の方が後に確定した
    知識だから。タグキーの規約 (小文字 snake_case) は core が検証する。

    値の妥当性は前段で潰してある — source × ego_pose_backend は設定ロード時
    (config.validate_source_backend)、sensor_config は preflight
    (resolve_sensor_config) で見るので、ここで初めて落ちることは通常ない
    (finalizing での失敗は収録済みドライブを不完全にするため避けたい)。
    """
    options = options or DriveOptions()
    merged_tags = {**options.tags(), **(tags or {})}

    return DriveManifest(
        drive_id=allocation.drive_id,
        uuid=uuid_factory(),
        source=config.source,
        platform=config.platform_id,
        vehicle=config.vehicle_id,
        ego_pose_backend=config.ego_pose_backend,
        calib_id=config.calib_id,
        # drive_id と同じ started_at から採る (日跨ぎでずらさない)
        date_captured=allocation.started_at.date(),
        # location 未指定でも finalizing で落とさない。drive_id 側も
        # normalize_location("") が同じ既定に落ちるので、両者の値が一致する。
        location=options.location or LOCATION_FALLBACK,
        driver=options.driver,
        weather=options.weather,
        recorder_version=recorder_version(),
        sensor_config=resolve_sensor_config(
            platform,
            namespace=config.topic_namespace,
            overrides=config.topic_overrides,
        ),
        tags=merged_tags,
        # status / archive_status は core の既定 (recorded / none) に委ねる。
        # 収録直後の状態は契約側が決めることで、recorder と studio がずれない。
    )


# --------------------------------------------------------------------------
# 読み書き
# --------------------------------------------------------------------------


def dump_manifest(manifest: DriveManifest) -> str:
    """manifest を YAML 文字列にする。"""
    return dump_mapping(manifest.model_dump(mode="json"))


def load_manifest(path: str | os.PathLike[str]) -> DriveManifest:
    """manifest.yaml を読む (完成判定・catalog・transfer 用)。

    ファイル層の失敗は yamlio の YamlError、スキーマ違反は pydantic の
    ValidationError がそのまま出る。manifest は recorder 自身が書いたファイルなので、
    定義ファイル (definitions.py) のように同期漏れを案内する必要が無い。
    """
    data = load_mapping(Path(path), what="manifest")
    return DriveManifest.model_validate(data)


def _write_text_atomic(path: Path, text: str) -> None:
    """同じディレクトリの一時ファイルに書いてから rename する。

    manifest の存在が完成の印なので、「中途半端な manifest が見える」瞬間を
    作らない (§4.4)。車載 Jetson は走行中の電源断がありうるため、rename の
    耐久性まで確保する: 本体を fsync してから replace し、親ディレクトリも
    fsync する (でないと rename 自体がディスクに届いていない場合がある)。

    汎用化は 2 例目 (topic_stats.json) が出てから。YAML 固有ではないので
    yamlio には置かない。
    """
    fd, tmp_name = tempfile.mkstemp(
        dir=path.parent, prefix=f".{path.name}.", suffix=".tmp"
    )
    tmp = Path(tmp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
    except BaseException:
        # 失敗しても一時ファイルを残さない (次回の書き出しやディレクトリ走査の
        # ノイズになる)。rename 済みなら tmp はもう無いので missing_ok。
        tmp.unlink(missing_ok=True)
        raise

    dir_fd = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)


@dataclass(frozen=True)
class ManifestWriter:
    """manifest.yaml を書き出す (session.ArtifactWriter を満たす)。

    出力先は自分で知っている — セッションはフォルダ構成から独立させ、順序制御
    だけに専念させる方針 (session.py の docstring)。
    """

    manifest: DriveManifest
    path: Path

    @classmethod
    def for_drive(cls, drive: DriveLayout, manifest: DriveManifest) -> "ManifestWriter":
        """ドライブのレイアウトから出力先を決めて組み立てる。"""
        return cls(manifest=manifest, path=drive.manifest)

    def write(self) -> Path:
        """manifest.yaml を原子的に書き出して、そのパスを返す。"""
        _write_text_atomic(self.path, dump_manifest(self.manifest))
        return self.path
