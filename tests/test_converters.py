"""ROS メッセージ ⇔ core の型の変換 (CLAUDE.md §8)。

固定したいのは 3 点:

- **時刻が ns 整数のまま保たれること** (core は float を全面拒否する: §2)
- **StartRecording の `""` が None に落ちること。** ROS は未設定の文字列を `""`
  で運ぶので、そのまま DriveOptions に渡すとクライアントが埋めなかった全
  フィールドが設定・CLI の値を消す (core/config.py の値の優先順位)
- **ヘッダの有無**。ヘッダを持たない型では stats が受信時刻で代用する (§6.1)

**偽のメッセージ型は作らず、生成された実型を使う。** フィールド名・`uint64` の
範囲・文字列の既定値 `""` といった、実型でしか検出できないずれを通さないため。
"""

import pytest

# **モジュール先頭で skip すること。** pyproject の addopts (`-m 'not ros'`) による
# deselect はモジュールを import した後に効くので、これが無いと shasou_msgs の
# 無い環境 (CI) で下の import が収集エラーになり pytest 全体が赤くなる。
# 見張るのが rclpy でなく shasou_msgs なのは、converters が import するのが
# こちらだから (converters は rclpy に依存しない)。
pytest.importorskip(
    "shasou_msgs",
    reason="shasou_msgs のビルドが要る (ROS 2 を source して colcon build)",
)

# ROS 2 環境で `pytest -m ros` として回す。
pytestmark = pytest.mark.ros

from builtin_interfaces.msg import Time  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
from shasou_core.schemas.common import DataSource  # noqa: E402
from shasou_core.schemas.events import EventTag  # noqa: E402
from shasou_core.schemas.topics import RosType  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

from shasou_msgs.msg import EventTag as EventTagMsg  # noqa: E402
from shasou_msgs.srv import StartRecording, StopRecording  # noqa: E402

from shasou_recorder.core.config import DriveOptions, resolve_drive_options  # noqa: E402
from shasou_recorder.core.session import (  # noqa: E402
    FINALIZE_ORDER,
    FinalizeResult,
    FinalizeStep,
    StopReason,
)
from shasou_recorder.ros.converters import (  # noqa: E402
    ConversionError,
    data_source_from_request,
    drive_options_from_request,
    event_from_msg,
    event_to_msg,
    header_stamp_ns,
    message_type_has_header,
    ns_to_stamp,
    stamp_to_ns,
    start_response_failure,
    start_response_success,
    stop_request_from_request,
    stop_response,
)

# 2026-07-16 10:27:14.512 UTC 相当。実運用のスケール (10^18 ns) で桁落ちが
# 起きないことを見るために、小さな値ではなく実時刻を使う。
EVENT_NS = 1752641234512000000
DRIVE_ID = "2026-07-16_1030_vehicle01_osaka-umeda"


def event_msg(
    *,
    ns: int = EVENT_NS,
    event_type: str = "interesting",
    label: str = "cut-in",
    source: str = "driver_button",
) -> EventTagMsg:
    return EventTagMsg(
        header=Header(stamp=ns_to_stamp(ns)),
        type=event_type,
        label=label,
        source=source,
    )


def start_request(**fields: str) -> StartRecording.Request:
    """StartRecording のリクエスト。**指定しなかったフィールドは `""`** になる
    (ROS の既定値。これがこのモジュールの主題)。"""
    return StartRecording.Request(**fields)


class TestStamp:
    def test_sec_and_nanosec_combine_into_ns(self):
        assert stamp_to_ns(Time(sec=1, nanosec=500_000_000)) == 1_500_000_000

    def test_result_is_an_integer(self):
        # core の EventTag は strict=True で float を全面拒否する (整数値の
        # float も含む) ので、int であること自体が契約。
        assert type(stamp_to_ns(Time(sec=1, nanosec=1))) is int

    def test_round_trip_at_real_world_scale(self):
        stamp = ns_to_stamp(EVENT_NS)

        assert (stamp.sec, stamp.nanosec) == (1752641234, 512000000)
        assert stamp_to_ns(stamp) == EVENT_NS

    def test_zero_stays_zero(self):
        # 未設定のヘッダを「無い」扱いにしない。CARLA のシミュレーション時刻は
        # 0 近傍から始まりうるので、0 を None に落とすと stats が受信時刻
        # (壁時計) にフォールバックし、シム時刻と壁時計が混ざる。
        assert stamp_to_ns(Time()) == 0


class TestEventFromMsg:
    def test_fields_and_timestamp_are_carried_over(self):
        event = event_from_msg(event_msg())

        assert event.timestamp == EVENT_NS
        assert type(event.timestamp) is int
        assert event.type.value == "interesting"
        assert event.label == "cut-in"
        assert event.source == "driver_button"

    def test_round_trip_preserves_the_timestamp(self):
        original = EventTag(
            timestamp=EVENT_NS, type="marker", label="工事区間", source="tablet"
        )

        assert event_from_msg(event_to_msg(original)) == original

    @pytest.mark.parametrize(
        "field,value",
        [
            ("event_type", "cutin"),          # EventType の語彙に無い
            ("source", "Driver Button"),      # 小文字 snake_case 違反
            ("label", ""),                    # min_length=1
            ("label", "   "),                 # 空白のみ
        ],
        ids=["unknown-type", "source-pattern", "empty-label", "blank-label"],
    )
    def test_core_validation_failure_becomes_conversion_error(self, field, value):
        with pytest.raises(ConversionError) as excinfo:
            event_from_msg(event_msg(**{field: value}))

        # 受け取った生の値がメッセージに出ること (送り手の設定ミスを直せるように)。
        assert repr(value) in str(excinfo.value)

    def test_negative_timestamp_is_rejected(self):
        # core の TimestampNs は ge=0。ROS の sec は int32 (符号付き) なので
        # 負の時刻が届きうる。
        msg = event_msg()
        msg.header.stamp = Time(sec=-1, nanosec=0)

        with pytest.raises(ConversionError):
            event_from_msg(msg)


class TestStartRecordingRequest:
    def test_all_unset_fields_become_none(self):
        # **最重要。** ROS は未設定を "" で運ぶので、"" のまま DriveOptions に
        # 渡すと「明示的な空」として下位層 (CLI・設定) の値を消してしまう。
        options = drive_options_from_request(start_request())

        assert options == DriveOptions()
        assert options.location is None
        assert options.weather is None
        assert options.route_id is None
        assert options.scenario is None

    def test_blank_only_also_becomes_none(self):
        # DriveOptions のバリデータは空白のみを "" に正規化するので、
        # ここで落とさないと上と同じ事故になる。
        options = drive_options_from_request(start_request(weather="   "))

        assert options.weather is None

    def test_filled_fields_are_carried_over(self):
        options = drive_options_from_request(
            start_request(
                location="osaka-umeda",
                weather="rain",
                route_id="town12_route003",
                scenario="cut_in",
            )
        )

        assert options.location == "osaka-umeda"
        assert options.weather == "rain"
        assert options.tags() == {"route_id": "town12_route003", "scenario": "cut_in"}

    def test_partially_filled_request_keeps_the_lower_layers(self):
        # 設定 < CLI < サービス。クライアントが location だけ埋めたとき、
        # 埋めなかったフィールドは下位層の値が残らなければならない。
        config_defaults = DriveOptions(
            location="osaka-umeda", driver="tanaka", weather="clear"
        )
        cli = DriveOptions(weather="rain", notes="雨天テスト")
        service = drive_options_from_request(start_request(location="kyoto"))

        resolved = resolve_drive_options(config_defaults, cli, service)

        assert resolved.location == "kyoto"        # サービスが上書き
        assert resolved.weather == "rain"          # サービスは触っていない → CLI
        assert resolved.driver == "tanaka"         # 誰も触っていない → 設定
        assert resolved.notes == "雨天テスト"       # サービスに無いフィールド

    def test_without_the_conversion_the_lower_layers_would_be_cleared(self):
        # 対照実験: "" をそのまま渡すと何が起きるか。この違いのために
        # drive_options_from_request が要る。
        naive = DriveOptions(location="kyoto", weather="", driver="", notes="")

        resolved = resolve_drive_options(
            DriveOptions(location="osaka-umeda", driver="tanaka"),
            DriveOptions(weather="rain"),
            naive,
        )

        assert resolved.weather is None
        assert resolved.driver is None


class TestDataSource:
    def test_known_value(self):
        assert data_source_from_request(start_request(source="carla")) is DataSource.CARLA

    def test_unset_means_unspecified(self):
        # 設定ファイルの source をそのまま使う、という意味 ("" → None の規則)。
        assert data_source_from_request(start_request()) is None
        assert data_source_from_request(start_request(source="  ")) is None

    def test_unknown_value_raises(self):
        with pytest.raises(ConversionError) as excinfo:
            data_source_from_request(start_request(source="sim"))

        message = str(excinfo.value)
        assert "'sim'" in message
        # 既知の語彙は core から引くので、core に値が増えても案内が古びない。
        assert "carla" in message and "real" in message


class TestStopRecordingRequest:
    def test_completed_route(self):
        request = stop_request_from_request(
            StopRecording.Request(completed=True, reason="")
        )

        assert request.completed is True
        assert request.reason is StopReason.SERVICE
        assert request.detail == ""

    def test_srv_reason_becomes_core_detail(self):
        # 名前が同じで意味が違う: core の reason は停止要求の出所 (enum)、
        # srv の reason は completed=false の理由 (自由文字列)。
        request = stop_request_from_request(
            StopRecording.Request(completed=False, reason="collision with pedestrian")
        )

        assert request.reason is StopReason.SERVICE
        assert request.detail == "collision with pedestrian"
        assert request.completed is False

    def test_reaches_the_manifest_tags(self):
        from shasou_recorder.core.session import RecordingSession

        session = RecordingSession(DRIVE_ID)
        session.begin_preflight()
        session.start_recording()
        session.request_stop(
            stop_request_from_request(
                StopRecording.Request(completed=False, reason="timeout")
            )
        )

        assert session.stop_tags() == {
            "stop_reason": "service",
            "completed": "false",
            "stop_detail": "timeout",
        }


class TestStartResponse:
    def test_success_carries_the_drive_id(self):
        response = start_response_success(DRIVE_ID)

        assert response.success is True
        assert response.drive_id == DRIVE_ID
        assert response.message == ""

    def test_failure_leaves_the_drive_id_empty(self):
        # srv の規定 ("drive_id は失敗時は空")。成功と失敗で関数が分かれている
        # ので、失敗なのに drive_id が載る応答は作れない。
        response = start_response_failure("source が未知の値: 'sim'")

        assert response.success is False
        assert response.drive_id == ""
        assert "sim" in response.message


def finalize_result(**overrides) -> FinalizeResult:
    defaults = dict(
        success=True,
        completed_steps=list(FINALIZE_ORDER),
        duration_sec=1234.5,
    )
    return FinalizeResult(**{**defaults, **overrides})


class TestStopResponse:
    def test_success_is_silent(self):
        response = stop_response(
            finalize_result(), drive_id=DRIVE_ID, message_count=482913
        )

        assert response.success is True
        assert response.message == ""       # srv の message の第一の意味は失敗理由
        assert response.drive_id == DRIVE_ID
        assert response.message_count == 482913
        assert response.duration_sec == pytest.approx(1234.5)

    def test_failure_before_manifest_reports_an_incomplete_drive(self):
        result = finalize_result(
            success=False,
            completed_steps=[
                FinalizeStep.BAG_CLOSE,
                FinalizeStep.TOPIC_STATS,
                FinalizeStep.EVENTS,
            ],
            failed_step=FinalizeStep.MANIFEST,
            error="OSError: No space left on device",
        )

        response = stop_response(result, drive_id=DRIVE_ID, message_count=100)

        assert response.success is False
        assert "manifest" in response.message
        assert "No space left on device" in response.message
        assert "不完全" in response.message
        assert "catalog" in response.message      # 未完了の手順が並ぶ
        # 失敗しても drive_id は返す (bag が残っているディレクトリを指す)。
        assert response.drive_id == DRIVE_ID

    def test_catalog_only_failure_reports_a_valid_drive(self):
        result = finalize_result(
            success=False,
            completed_steps=[
                FinalizeStep.BAG_CLOSE,
                FinalizeStep.TOPIC_STATS,
                FinalizeStep.EVENTS,
                FinalizeStep.MANIFEST,
            ],
            failed_step=FinalizeStep.CATALOG,
            error="OperationalError: database is locked",
        )

        response = stop_response(result, drive_id=DRIVE_ID, message_count=100)

        # finalizing は完走していないので success は false のまま (事実を曲げない)。
        # ただし manifest がある = ドライブは有効、という差を message で伝える。
        assert response.success is False
        assert "ドライブは有効" in response.message
        assert "不完全" not in response.message

    def test_generated_type_rejects_a_negative_message_count(self):
        # uint64。実型を使っている効用の確認 (偽の型では素通りする)。
        with pytest.raises(AssertionError):
            stop_response(finalize_result(), drive_id=DRIVE_ID, message_count=-1)


class TestHeaderDetection:
    @pytest.mark.parametrize(
        "ros_type",
        [RosType.BOOL, RosType.CLOCK, RosType.TF_MESSAGE],
        ids=lambda t: t.value,
    )
    def test_types_without_a_header_yield_none(self, ros_type):
        # stats はこれらを受信時刻で代用し、max_gap を算出しない (§6.1)。
        instance = get_message(ros_type.value)()

        assert header_stamp_ns(instance) is None
        assert message_type_has_header(type(instance)) is False

    def test_type_with_a_header_yields_the_stamp(self):
        imu = get_message(RosType.IMU.value)()
        imu.header.stamp = ns_to_stamp(EVENT_NS)

        assert header_stamp_ns(imu) == EVENT_NS
        assert message_type_has_header(type(imu)) is True

    @pytest.mark.parametrize("ros_type", list(RosType), ids=lambda t: t.value)
    def test_type_level_and_instance_level_judgements_agree(self, ros_type):
        # 型名の対応表を持たない判断の担保。登録時 (クラス) と受信時
        # (インスタンス) が同じ真実を見ていることを、core の全 RosType で確認する。
        message_type = get_message(ros_type.value)
        instance = message_type()

        has_header = message_type_has_header(message_type)

        assert has_header == hasattr(instance, "header")
        # 既定のスタンプ (0) でも None にはならない。0 を「無い」扱いにすると
        # シム時刻と壁時計が混ざる (TestStamp.test_zero_stays_zero)。
        assert (header_stamp_ns(instance) is not None) == has_header
