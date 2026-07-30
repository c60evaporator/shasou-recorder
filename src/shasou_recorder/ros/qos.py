"""core の QoS 契約を rclpy の QoSProfile に変換する (CLAUDE.md §7.3)。

**DDS は publisher と subscriber の QoS が両立しないと接続しない。** トピック名も
型も合っているのにデータが 1 通も来ない、という発見しにくい障害になる。core の
`TopicContract.qos` が publisher 側 (CARLA ブリッジ・実車ドライバ) の設定を契約
として規定しているので、recorder の subscriber はこれに合わせる。

core は ROS に依存できない (§1.1) ので `rclpy.qos.QoSProfile` を持てず、文字列 enum
(`QosReliability` / `QosHistory` / `QosDurability`) で表現している。**変換はこの
境界で一度だけ行う。** 購読側 (ros/node.py) が個別に `QoSProfile` を組み立てると、
契約と実際の購読設定がズレたときに気づけない — 座標変換を CARLA ブリッジの境界に
集約したのと同じ考え方で、変換箇所を散らさない。

現時点で既定と異なる契約は `tf_static` だけ (depth=1 / transient_local)。ここを
volatile で購読すると、**収録開始前に publish された tf_static を取りこぼし、bag に
センサ外部パラメータが入らない。** transient_local なら late joiner (StartRecording
より後に購読を始めた recorder) でも最後の 1 通を受け取れる。

ROS 2 が無い環境での import 失敗について
----------------------------------------
このモジュールは rclpy を素で import する。ROS 2 を source していない環境では
`ImportError` がそのまま上がる — **包まない。** try/except で包むと自分たちの
import のタイポまで捕まえて「ROS 2 を source してください」という的外れな案内を
出すことになるし、rclpy の素のメッセージ自体すでに十分明確だから。

**申し送り: 親切な案内は cli.py の責務。** ユーザーがこの失敗に遭遇するのは
`shasou-recorder record` の実行時なので、cli.py が ros/ を import する箇所を
try/except ImportError で包み「ROS 2 Humble を source してから実行してください」と
案内すること。ライブラリ層は素直にエラーを投げ、アプリケーション層が文脈を足す
分担 (CLAUDE.md §1.1 にも記載)。
"""

from __future__ import annotations

from enum import Enum
from typing import Mapping, TypeVar

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from shasou_core.schemas.topics import (
    QosDurability,
    QosHistory,
    QosReliability,
    TopicContract,
)

# rclpy の `QoSProfile` と S の大小しか違わず取り違えやすいので別名にする。
from shasou_core.schemas.topics import QosProfile as CoreQosProfile

_EnumT = TypeVar("_EnumT", bound=Enum)


class UnsupportedQosError(ValueError):
    """core の QoS 値を rclpy のポリシーへ写せなかった。

    core に enum 値が増えて、下の対応表の更新が漏れたときに出る。**黙って既定値に
    落とさない**: QoS の不一致は「接続しないのでデータが来ない」という発見しにくい
    障害なので、収録を始める前に落ちる方がよい。
    """


# --------------------------------------------------------------------------
# 対応表
# --------------------------------------------------------------------------
# rclpy 側には SYSTEM_DEFAULT / UNKNOWN もあるが、**そこへは写さない。**
# 「RMW の既定に従う」は §7.3 が避けたい曖昧さそのもので、契約は常に明示値で
# あるべきだから (core の enum に対応する値が無いのも同じ理由)。

_RELIABILITY: Mapping[QosReliability, ReliabilityPolicy] = {
    QosReliability.RELIABLE: ReliabilityPolicy.RELIABLE,
    QosReliability.BEST_EFFORT: ReliabilityPolicy.BEST_EFFORT,
}

_HISTORY: Mapping[QosHistory, HistoryPolicy] = {
    QosHistory.KEEP_LAST: HistoryPolicy.KEEP_LAST,
    QosHistory.KEEP_ALL: HistoryPolicy.KEEP_ALL,
}

_DURABILITY: Mapping[QosDurability, DurabilityPolicy] = {
    QosDurability.VOLATILE: DurabilityPolicy.VOLATILE,
    QosDurability.TRANSIENT_LOCAL: DurabilityPolicy.TRANSIENT_LOCAL,
}


def _assert_exhaustive(table: Mapping[_EnumT, object], enum_cls: type[_EnumT]) -> None:
    """対応表が enum の全メンバーを覆っていることを確認する。"""
    missing = [member.value for member in enum_cls if member not in table]
    if missing:
        raise UnsupportedQosError(
            f"{enum_cls.__name__} の値 {', '.join(missing)} が "
            f"{__name__} の対応表に無い (core に値が増えたら表も更新すること)"
        )


# **import 時に検査する。** core に値が増えて表が追いつかなかった場合、その値を使う
# トピックを購読する時点で気づくのでは遅い (実機やシミュレーションを走らせている)。
# ros マーカーのテストは ROS 2 が要るので CI では回らず、開発機での import が実質
# 最初の関門になる。
_assert_exhaustive(_RELIABILITY, QosReliability)
_assert_exhaustive(_HISTORY, QosHistory)
_assert_exhaustive(_DURABILITY, QosDurability)


def _lookup(
    table: Mapping[_EnumT, object], value: _EnumT, enum_cls: type[_EnumT]
) -> object:
    """対応表を引く。**未知の値は明示的にエラーにする** (get で None を返さない)。

    import 時の `_assert_exhaustive` が enum メンバーの網羅を保証するので、ここに
    落ちてくるのは enum メンバーですらない値 (`model_construct` 等の検証を通らない
    生成経路で入った文字列や、将来の別 enum) になる。
    """
    try:
        return table[value]
    except KeyError:
        raise UnsupportedQosError(
            f"{enum_cls.__name__} の値 {value!r} を rclpy のポリシーへ変換できない"
        ) from None


# --------------------------------------------------------------------------
# 変換
# --------------------------------------------------------------------------


def to_rclpy_qos(profile: CoreQosProfile) -> QoSProfile:
    """core の `QosProfile` を rclpy の `QoSProfile` に変換する。

    core が規定するのは 4 ポリシー (reliability / history / depth / durability) だけ。
    lifespan / deadline / liveliness は **rclpy の既定のまま**にする — publisher
    (CARLA ブリッジ・実車ドライバ) も rclpy の既定で publish するので両立する。
    ここで独自の値を入れると、契約に無い軸で接続しなくなる。
    """
    return QoSProfile(
        history=_lookup(_HISTORY, profile.history, QosHistory),
        # depth は history=KEEP_ALL では DDS 上意味を持たないが、rclpy は
        # KEEP_LAST のとき必須で KEEP_ALL でも受け取るので、契約の値をそのまま渡す。
        depth=profile.depth,
        reliability=_lookup(_RELIABILITY, profile.reliability, QosReliability),
        durability=_lookup(_DURABILITY, profile.durability, QosDurability),
    )


def qos_for_contract(contract: TopicContract) -> QoSProfile:
    """トピック契約から、購読に使う `QoSProfile` を得る。

    呼び出し側が `contract.qos` を取り出して変換する 2 段を書かずに済むように。
    **購読は必ずこれを通すこと** — 契約を無視して `QoSProfile` を直に組むと §7.3 の
    一致が崩れ、データが来ない原因を購読側のコードから追う羽目になる。
    """
    return to_rclpy_qos(contract.qos)
