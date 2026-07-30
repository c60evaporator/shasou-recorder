"""ROS メッセージ ⇔ core の型の変換 (CLAUDE.md §8)。

ROS の世界と core の世界は 3 点で表現が違う。その差を**この境界 1 箇所で**吸収し、
node.py が個別に `sec * 10**9 + nanosec` を書いたり `""` の意味を判断したりしない
ようにする (qos.py が QoS の差を集約したのと同じ考え方)。

1. **時刻** — ROS は `header.stamp` (sec + nanosec の 2 整数)、core はエポックからの
   ns 整数 1 本 (§2)。core の `EventTag.timestamp` は `strict=True` で float を
   全面拒否するので、必ず整数で組み立てる
2. **空値** — ROS メッセージは未設定の文字列を `""` として運ぶ。core の
   `DriveOptions` は `""` を「明示的な空」と解釈して下位層 (CLI・設定) の値を
   **消す** (core/config.py の値の優先順位)。そのままでは「クライアントが埋めな
   かった全フィールドが設定値を消す」ので、**`"" → None` に落としてから**
   DriveOptions を作る。これはサービス経路が「明示的な空」を表現できなくなる
   ことを意味するが、リクエストを組み立てるクライアントがその走行の権威なので
   問題にならない
3. **語彙** — core は `EventType` / `DataSource` の語彙と source の命名規約
   (小文字 snake_case) を強制する。ROS 側は素の `string`

不正な値が来たときの方針 (申し送り: node.py の責務)
----------------------------------------------------
**このモジュールは「変換できない」を `ConversionError` で報告するだけ**にして、
純粋関数のままにする (「警告して捨てる」の警告 = ログ出力は副作用で、ROS の
ロガーを持つのは node.py だから)。**捨てるか拒否するかは呼び出し側が決める。
種類ごとに方針が違う**ので、node.py を実装するときは下記のとおりにすること:

- **EventTag は捨てる** (警告ログを出して次へ)。イベントは補助情報なうえ、
  **正は bag 側** (§2) — EventTag は bag に記録済みで jsonl はいつでも再生成
  できるので、1 件落としても失われる情報は無い。逆にここで例外を上げると購読
  コールバックから executor に伝播し、1 走行分のセンサデータを失う
- **StartRecording は拒否する** (`success=false` で応答し、収録を始めない)。
  まだ何も収録していないので拒否のコストが最小で、不正な source のまま走ると
  出自を偽った manifest のドライブが 1 本できてしまう

**値の正規化はしない。** 規約の正は core であって、recorder が勝手に緩めては
ならない (§1.2)。`"" → None` は緩めではなく ROS の**運搬形式の復元**なので、
これだけは converters が行う。

依存について
------------
`shasou_msgs` を素で import する (§1.1 の依存表どおり)。ROS 2 / shasou_msgs が
無い環境では `ImportError` がそのまま上がる — 包まず、案内は cli.py の責務
(qos.py の docstring と §1.1 を参照)。**rclpy には依存しない**: 生成された
メッセージ型と core だけで足りるので、依存は小さいままにしておく。
"""

from __future__ import annotations

from typing import Any, Optional

from builtin_interfaces.msg import Time
from pydantic import ValidationError
from shasou_core.constants import NS_PER_SEC
from shasou_core.schemas.common import DataSource
from shasou_core.schemas.events import EventTag
from std_msgs.msg import Header

from shasou_msgs.msg import EventTag as EventTagMsg  # core の EventTag と同名
from shasou_msgs.srv import StartRecording, StopRecording

from ..core.config import DriveOptions
from ..core.session import FinalizeResult, StopReason, StopRequest, remaining_steps


class ConversionError(ValueError):
    """ROS から届いた値を core の型へ写せなかった (送り手側の不正)。

    捨てるか収録を拒否するかは呼び出し側が決める (モジュール docstring の申し送り)。
    例外の種類は分けていない — 判断が分かれるのは呼び出し箇所であって、値の種類
    ではないため。
    """


# --------------------------------------------------------------------------
# 時刻
# --------------------------------------------------------------------------
# ROS の (sec, nanosec) と core の ns 整数の相互変換。**整数演算だけで組み立てる。**
# core の seconds_to_ns() は float 秒 (CARLA の elapsed_seconds 等) からの変換で
# 桁を間違えないためのヘルパで、ここは両フィールドとも整数なので出番が無い。
# 整数のままの方が丸めの余地が無く厳しい (単位定数は core から取る)。


def stamp_to_ns(stamp: Time) -> int:
    """`builtin_interfaces/Time` をエポックからの ns 整数にする。

    **0 は 0 のまま返す。** 未設定のヘッダ (sec=0, nanosec=0) を「無い」扱いに
    したくなるが、CARLA のシミュレーション時刻は 0 近傍から始まりうる。0 を
    None に落とすと stats が受信時刻 (壁時計) にフォールバックし、同じトピックに
    シム時刻と壁時計が混ざって span が 10^18 ns になる。判定を持ち込まず計測値を
    そのまま渡す (§2「統計は計測値の器に徹する」)。
    """
    return stamp.sec * NS_PER_SEC + stamp.nanosec


def ns_to_stamp(ns: int) -> Time:
    """ns 整数を `builtin_interfaces/Time` にする (stamp_to_ns の逆)。

    `sec` は int32 なので 2038 年に溢れる (ROS 共通の制約)。溢れた場合は
    生成コードの assert が落ちるので、静かに壊れることはない。
    """
    sec, nanosec = divmod(ns, NS_PER_SEC)
    return Time(sec=sec, nanosec=nanosec)


def header_stamp_ns(message: Any) -> Optional[int]:
    """任意の ROS メッセージからヘッダのスタンプ [ns] を取り出す。無ければ None。

    `StatsCollector.update(topic_name, receive_ns, stamp_ns)` の第 3 引数に渡す。
    ヘッダを持たない型 (std_msgs/Bool、rosgraph_msgs/Clock、tf2_msgs/TFMessage)
    では None になり、core 側が受信時刻で代用して max_gap を算出しない (§6.1)。
    """
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    return stamp_to_ns(stamp)


def message_type_has_header(message_type: type) -> bool:
    """メッセージ**型**がヘッダを持つか。購読登録時 (TopicSpec.has_header) に使う。

    **型名の対応表は持たない。** `RosType` → bool の表を書くと core の enum が
    増えるたびに腐るうえ、受信時の header_stamp_ns (実インスタンスを見る) と
    二重の真実になる。生成された型自身に聞けば、登録時 (クラス) と受信時
    (インスタンス) が同じ真実を見る。
    """
    return message_type.get_fields_and_field_types().get("header") == "std_msgs/Header"


# --------------------------------------------------------------------------
# ROS → core
# --------------------------------------------------------------------------


def _optional(value: str) -> Optional[str]:
    """ROS の `""` (未設定) を `None` (未指定) に落とす。**空白のみも同様。**

    空白のみを残すと DriveOptions のバリデータが `""` に正規化し、「明示的な空」
    として下位層の値を消してしまう (core/config.py の値の優先順位)。
    """
    return value.strip() or None


def event_from_msg(msg: EventTagMsg) -> EventTag:
    """`shasou_msgs/msg/EventTag` を core の EventTag にする。

    語彙違反 (EventType にない type、規約外の source、空の label) と、負の
    タイムスタンプは `ConversionError`。**呼び出し側は捨ててよい** (モジュール
    docstring の申し送り)。
    """
    try:
        return EventTag(
            timestamp=stamp_to_ns(msg.header.stamp),
            type=msg.type,
            label=msg.label,
            source=msg.source,
        )
    except ValidationError as error:
        # 受け取った生の値を載せる。送り手の設定ミス (source の表記揺れ等) は
        # 値が見えないと直せない。pydantic の詳細は from で辿れるように残す。
        raise ConversionError(
            f"EventTag を変換できない (type={msg.type!r} label={msg.label!r} "
            f"source={msg.source!r}): {error}"
        ) from error


def drive_options_from_request(request: StartRecording.Request) -> DriveOptions:
    """StartRecording のリクエストから走行ごとの値を取り出す。

    **`"" → None`。** ROS は未設定の文字列を `""` で運ぶので、そのまま渡すと
    クライアントが埋めなかったフィールドが設定・CLI の値を消す。

    `driver` / `notes` は StartRecording に無いので None のまま = 設定と CLI の値が
    残る。source はドライブの出自であって走行ごとの値ではないので、
    data_source_from_request() が別に扱う。
    """
    return DriveOptions(
        location=_optional(request.location),
        weather=_optional(request.weather),
        route_id=_optional(request.route_id),
        scenario=_optional(request.scenario),
    )


def data_source_from_request(
    request: StartRecording.Request,
) -> Optional[DataSource]:
    """StartRecording の source を core の DataSource にする。未指定なら None。

    `""` (未設定) は None を返す — 設定ファイルの source をそのまま使う、という
    意味になる (`""` → None の規則をここにも通す)。**未知の非空値は
    `ConversionError`**: 出自を偽った manifest のドライブを作らないよう、
    呼び出し側は収録を始めずに拒否すること。
    """
    raw = request.source.strip()
    if not raw:
        return None
    try:
        return DataSource(raw)
    except ValueError as error:
        known = " / ".join(source.value for source in DataSource)
        raise ConversionError(
            f"StartRecording.source が未知の値: {request.source!r} (既知: {known})"
        ) from error


def stop_request_from_request(request: StopRecording.Request) -> StopRequest:
    """StopRecording のリクエストを core の StopRequest にする。

    **srv の `reason` は core の `detail` に入る。** core の `StopRequest.reason`
    は StopReason enum (停止要求の出所) で、サービス経路では常に `SERVICE`。
    srv 側の `reason` は「completed=false のときの理由」という自由文字列なので、
    名前が同じでも指すものが違う。
    """
    return StopRequest(
        reason=StopReason.SERVICE,
        detail=request.reason.strip(),
        completed=bool(request.completed),
    )


# --------------------------------------------------------------------------
# core → ROS
# --------------------------------------------------------------------------


def event_to_msg(event: EventTag) -> EventTagMsg:
    """core の EventTag を `shasou_msgs/msg/EventTag` にする (§8 の相互変換)。

    recorder 自身は EventTag を publish しないが、往復をテストで固定できるほか、
    bag から events.jsonl を再生成する処理 (core/events.py が ros/ の責務として
    いるもの) を書くときの入口になる。frame_id は未使用 (空文字)。
    """
    return EventTagMsg(
        header=Header(stamp=ns_to_stamp(event.timestamp)),
        type=event.type.value,
        label=event.label,
        source=event.source,
    )


def start_response_success(drive_id: str, message: str = "") -> StartRecording.Response:
    """収録を開始できたときの応答。**応答を返す時点で bag は開いていること** (§8)。"""
    return StartRecording.Response(success=True, message=message, drive_id=drive_id)


def start_response_failure(message: str) -> StartRecording.Response:
    """収録を開始できなかったときの応答。`drive_id` は空 (srv の規定)。

    成功と失敗で関数を分けているのは、1 つの関数にフラグを持たせると「失敗なのに
    drive_id が載っている」応答を呼び出し側が作れてしまうため。
    """
    return StartRecording.Response(success=False, message=message, drive_id="")


def _finalize_message(result: FinalizeResult) -> str:
    """finalizing の結果を StopRecording.Response.message の文面にする。

    成功時は空にする (srv の第一の意味が「失敗理由」なので、無事なら黙る)。
    失敗時は **manifest が書けたかどうかで文面を変える** — クライアントにとって
    意味が違うから。manifest 前の失敗はドライブが不完全 (走行のやり直しを検討
    すべき) で、catalog だけの失敗はドライブ自体は有効 (索引は再構築できる)。
    """
    if result.success:
        return ""
    step = result.failed_step.value if result.failed_step else "不明な手順"
    if result.manifest_written:
        return (
            f"{step} で失敗: {result.error}。manifest は書けているので"
            "ドライブは有効 (catalog は manifest から再構築できる)"
        )
    pending = ", ".join(s.value for s in remaining_steps(result.completed_steps))
    return (
        f"{step} で失敗: {result.error} (未完了: {pending})。"
        "manifest 未書き出しのためドライブは不完全 (bag は残っている)"
    )


def stop_response(
    result: FinalizeResult, *, drive_id: str, message_count: int
) -> StopRecording.Response:
    """finalizing の結果を StopRecording の応答にする。

    `message_count` は `StatsCollector.total_message_count`。FinalizeResult に
    入っていないのは、統計の集計が session の責務ではないため。

    **catalog だけが失敗した場合も success=false にする。** finalizing は完走して
    いないので事実を曲げず、「ドライブ自体は有効」という差は message で説明する。
    drive_id は失敗時も載せる (bag が残っているディレクトリを指すので、
    クライアントが記録・復旧に使える)。
    """
    return StopRecording.Response(
        success=result.success,
        message=_finalize_message(result),
        drive_id=drive_id,
        message_count=message_count,
        duration_sec=result.duration_sec,
    )
