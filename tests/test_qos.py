"""core の QoS 契約 → rclpy QoSProfile の変換 (CLAUDE.md §7.3)。

DDS は publisher と subscriber の QoS が両立しないと接続しないので、ここがズレると
「トピック名も型も合っているのにデータが来ない」という追いにくい障害になる。
特に tf_static (transient_local / depth=1) を取りこぼすと bag にセンサ外部
パラメータが入らないため、既定との差をテストで固定する。
"""

from enum import Enum

import pytest

# **モジュール先頭で skip すること。** pyproject の addopts (`-m 'not ros'`) による
# deselect はモジュールを import した後に効くので、これが無いと ROS 2 の無い環境
# (CI) で下の `from rclpy.qos import ...` が収集エラーになり pytest 全体が赤くなる。
pytest.importorskip(
    "rclpy", reason="ROS 2 環境が要る (source /opt/ros/humble/setup.bash)"
)

# ROS 2 を source した環境で `pytest -m ros` として回す。
pytestmark = pytest.mark.ros

from rclpy.qos import (  # noqa: E402  (importorskip より後に import する必要がある)
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)
from shasou_core.schemas.topics import (  # noqa: E402
    ALL_CONTRACTS,
    DEFAULT_QOS,
    TF_STATIC,
    TRANSIENT_LOCAL_QOS,
    QosDurability,
    QosHistory,
    QosReliability,
)
from shasou_core.schemas.topics import QosProfile as CoreQosProfile  # noqa: E402

from shasou_recorder.ros import qos  # noqa: E402
from shasou_recorder.ros.qos import (  # noqa: E402
    UnsupportedQosError,
    qos_for_contract,
    to_rclpy_qos,
)

# 期待する対応は **このテスト側に literal で持つ**。実装の対応表を import して
# 比べても「表と表が同じ」しか言えないため。
EXPECTED_RELIABILITY = {
    QosReliability.RELIABLE: ReliabilityPolicy.RELIABLE,
    QosReliability.BEST_EFFORT: ReliabilityPolicy.BEST_EFFORT,
}
EXPECTED_HISTORY = {
    QosHistory.KEEP_LAST: HistoryPolicy.KEEP_LAST,
    QosHistory.KEEP_ALL: HistoryPolicy.KEEP_ALL,
}
EXPECTED_DURABILITY = {
    QosDurability.VOLATILE: DurabilityPolicy.VOLATILE,
    QosDurability.TRANSIENT_LOCAL: DurabilityPolicy.TRANSIENT_LOCAL,
}


class TestContractProfiles:
    def test_default_qos_is_reliable_keep_last_10_volatile(self):
        profile = to_rclpy_qos(DEFAULT_QOS)

        assert profile.reliability is ReliabilityPolicy.RELIABLE
        assert profile.history is HistoryPolicy.KEEP_LAST
        assert profile.depth == 10
        assert profile.durability is DurabilityPolicy.VOLATILE

    def test_transient_local_qos_keeps_the_last_message_for_a_late_joiner(self):
        profile = to_rclpy_qos(TRANSIENT_LOCAL_QOS)

        assert profile.durability is DurabilityPolicy.TRANSIENT_LOCAL
        assert profile.depth == 1
        # reliability / history は既定と同じ (差は durability と depth だけ)
        assert profile.reliability is ReliabilityPolicy.RELIABLE
        assert profile.history is HistoryPolicy.KEEP_LAST
        # **既定との差が実際に出ていること。** volatile で購読すると StartRecording
        # より前に publish された tf_static を取りこぼす。
        assert profile != to_rclpy_qos(DEFAULT_QOS)

    def test_tf_static_carries_the_transient_local_profile(self):
        assert qos_for_contract(TF_STATIC) == to_rclpy_qos(TRANSIENT_LOCAL_QOS)

    def test_tf_static_is_the_only_contract_that_deviates_from_the_default(self):
        default = to_rclpy_qos(DEFAULT_QOS)

        deviating = {c.key for c in ALL_CONTRACTS if qos_for_contract(c) != default}

        assert deviating == {"tf_static"}, (
            "core の QoS 契約が変わっている。CLAUDE.md §7.3 は「既定と異なるのは "
            "tf_static だけ」と書いているので、記述と購読側の想定を見直すこと"
        )

    def test_qos_for_contract_matches_converting_the_contract_profile(self):
        # 導線 (qos_for_contract) が本体 (to_rclpy_qos) とズレていないこと。
        for contract in ALL_CONTRACTS:
            assert qos_for_contract(contract) == to_rclpy_qos(contract.qos), contract.key


class TestExhaustiveness:
    """core の 3 つの enum の**全メンバー**が変換できること。"""

    @pytest.mark.parametrize("value", list(QosReliability), ids=lambda v: v.value)
    def test_every_reliability_value_converts(self, value):
        profile = CoreQosProfile(reliability=value)

        assert to_rclpy_qos(profile).reliability is EXPECTED_RELIABILITY[value]

    @pytest.mark.parametrize("value", list(QosHistory), ids=lambda v: v.value)
    def test_every_history_value_converts(self, value):
        converted = to_rclpy_qos(CoreQosProfile(history=value, depth=5))

        assert converted.history is EXPECTED_HISTORY[value]
        # depth は KEEP_ALL では DDS 上意味を持たないが、契約の値をそのまま渡す。
        assert converted.depth == 5

    @pytest.mark.parametrize("value", list(QosDurability), ids=lambda v: v.value)
    def test_every_durability_value_converts(self, value):
        profile = CoreQosProfile(durability=value)

        assert to_rclpy_qos(profile).durability is EXPECTED_DURABILITY[value]

    @pytest.mark.parametrize(
        "enum_cls,expected",
        [
            (QosReliability, EXPECTED_RELIABILITY),
            (QosHistory, EXPECTED_HISTORY),
            (QosDurability, EXPECTED_DURABILITY),
        ],
        ids=["reliability", "history", "durability"],
    )
    def test_this_test_covers_every_enum_value(self, enum_cls, expected):
        # core に値が増えたら、実装の対応表 (import 時に検査される) だけでなく
        # 上の parametrize も追随させるための歯止め。
        assert set(expected) == set(enum_cls)


class _FutureReliability(str, Enum):
    """core に将来値が増えた状況を模す (実装の対応表に無い値)。"""

    EXACTLY_ONCE = "exactly_once"


class _FutureHistory(str, Enum):
    KEEP_FIRST = "keep_first"


class _FutureDurability(str, Enum):
    PERSISTENT = "persistent"


class TestUnknownValues:
    """未知の値は黙って既定に落とさず、明示的にエラーにする。"""

    @pytest.mark.parametrize(
        "field,value",
        [
            ("reliability", _FutureReliability.EXACTLY_ONCE),
            ("history", _FutureHistory.KEEP_FIRST),
            ("durability", _FutureDurability.PERSISTENT),
        ],
        ids=["reliability", "history", "durability"],
    )
    def test_unknown_enum_value_raises(self, field, value):
        # model_construct は pydantic の検証を通さないので、core の enum に無い値を
        # 混ぜられる (= core に値が増えて recorder の対応表が追いつかない状況)。
        profile = CoreQosProfile.model_construct(**{field: value})

        with pytest.raises(UnsupportedQosError) as excinfo:
            to_rclpy_qos(profile)

        # どの値が写せなかったかがメッセージに出ること (表の更新箇所が分かる)。
        assert value.value in str(excinfo.value)

    def test_assert_exhaustive_rejects_a_table_that_misses_a_value(self):
        # import 時の関門が本当に働くことの確認。この検査は公開 API を経由しない
        # (モジュール読み込み時に走る) ので、ここだけ private を直に呼ぶ。
        with pytest.raises(UnsupportedQosError) as excinfo:
            qos._assert_exhaustive({}, _FutureReliability)

        assert "exactly_once" in str(excinfo.value)
