"""収録中のトピック統計とディスク監視。

shasou-core の schemas/health.py (TopicStat / TopicStats / DiskStats) を
組み立てるのがこのモジュールの役目。core/ にあるので ROS には依存せず、
ros/ 側が受信したメッセージから「トピック名・ヘッダのスタンプ・受信時刻」
だけを取り出して渡す。

オンライン累積 (O(1) メモリ)
----------------------------
数十万〜数百万メッセージを扱うので、全メッセージの時刻を保持しない。
1 トピックあたり定数個のスカラーだけを更新する。したがって計算できるのは
件数・最初/最後の時刻・最大間隔・平均レートに限られる (分散やパーセンタイル
は保持データが増えるので対象外)。新しい統計を足したいときは
TopicAccumulator にフィールドと update() の 1 行を加えれば済む。

時刻の扱い
----------
- ヘッダを持つトピック: header.stamp を使う。CARLA ではシミュレーション時刻
  なので、受信時刻では実態と合わない
- ヘッダを持たないトピック (std_msgs/Bool 等): 受信時刻で代用し、
  max_gap_ns は算出しない (スタンプが無いので「取りこぼし」の意味が変わる)

統計の対象
----------
bag に記録するトピックだけ。契約外の想定外トピックは無視する
(それらの検出は shasou-core の validation の責務)。
"""

from __future__ import annotations

import json
import os
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Optional

from shasou_core.schemas.health import DiskStats, TopicStat, TopicStats

NS_PER_SEC = 1_000_000_000


# --------------------------------------------------------------------------
# 1 トピック分の累積
# --------------------------------------------------------------------------


@dataclass
class TopicAccumulator:
    """1 トピックの受信統計をオンラインで累積する。

    保持するのは定数個のスカラーのみ。update() は受信のたびに呼ばれるので、
    ここに重い処理を入れないこと (収録の主処理を止めてしまう)。
    """

    topic_name: str
    channel: Optional[str] = None
    expected_hz: Optional[float] = None
    has_header: bool = True

    message_count: int = 0
    first_timestamp: Optional[int] = None
    last_timestamp: Optional[int] = None
    max_gap_ns: Optional[int] = None

    def update(self, receive_ns: int, stamp_ns: Optional[int] = None) -> None:
        """1 メッセージ分を反映する。

        stamp_ns はヘッダのスタンプ [ns]。ヘッダを持たないトピックでは None を
        渡す (受信時刻で代用する)。
        """
        timestamp = stamp_ns if stamp_ns is not None else receive_ns

        self.message_count += 1
        if self.first_timestamp is None:
            self.first_timestamp = timestamp
        elif self.has_header and self.last_timestamp is not None:
            # 最大間隔はスタンプのあるトピックだけ。時刻が逆行する場合
            # (再生や時刻同期の乱れ) は間隔として数えない。
            gap = timestamp - self.last_timestamp
            if gap > 0 and (self.max_gap_ns is None or gap > self.max_gap_ns):
                self.max_gap_ns = gap
        self.last_timestamp = timestamp

    # ── 導出値 ──────────────────────────────────────────────────────────

    @property
    def span_ns(self) -> Optional[int]:
        """最初と最後のメッセージの時刻差 [ns]。2 件未満なら None。"""
        if (
            self.message_count < 2
            or self.first_timestamp is None
            or self.last_timestamp is None
        ):
            return None
        return self.last_timestamp - self.first_timestamp

    @property
    def measured_hz(self) -> Optional[float]:
        """実測平均レート [Hz]。

        N 件が span の間に届いたとき、間隔の数は N-1 なので (N-1)/span を
        レートとする。収録開始・終了のオーバーヘッドを含まないので、
        データセットとして切り出したときの実態と合う。
        """
        span = self.span_ns
        if span is None or span <= 0:
            return None
        return (self.message_count - 1) * NS_PER_SEC / span

    @property
    def drop_rate(self) -> Optional[float]:
        """期待レートに対する取りこぼし割合 [0,1]。

        実測レートと期待レートの比で求める。期待より速い場合 (計測誤差や
        expected_hz の設定ミス) は 0 に丸める。expected_hz が無ければ None。
        """
        expected = self.expected_hz
        measured = self.measured_hz
        if expected is None or expected <= 0 or measured is None:
            return None
        return min(1.0, max(0.0, 1.0 - measured / expected))

    def to_stat(self) -> TopicStat:
        return TopicStat(
            topic_name=self.topic_name,
            channel=self.channel,
            message_count=self.message_count,
            expected_hz=self.expected_hz,
            measured_hz=self.measured_hz,
            drop_rate=self.drop_rate,
            first_timestamp=self.first_timestamp,
            last_timestamp=self.last_timestamp,
            max_gap_ns=self.max_gap_ns if self.has_header else None,
        )


# --------------------------------------------------------------------------
# ドライブ全体の収集
# --------------------------------------------------------------------------


@dataclass
class TopicSpec:
    """統計対象として登録するトピックの仕様。

    expected_hz は platform 定義 (ChannelSpec.expected_hz) 由来。チャネルに
    紐づかないトピック (車両状態・gt 系) では None になる。
    """

    topic_name: str
    channel: Optional[str] = None
    expected_hz: Optional[float] = None
    has_header: bool = True


class StatsCollector:
    """1 ドライブ分のトピック統計を集める。

    登録済みトピック以外の update() は無視する (契約外の想定外トピックは
    統計の対象外)。呼び出し側が毎メッセージ呼ぶので、ロックは持たない —
    ROS の実行モデル上、単一の executor スレッドから呼ばれる前提。
    別スレッドから呼ぶ場合は呼び出し側で直列化すること。
    """

    def __init__(self, drive_id: str, specs: Iterable[TopicSpec] = ()) -> None:
        self.drive_id = drive_id
        self._accumulators: dict[str, TopicAccumulator] = {}
        for spec in specs:
            self.register(spec)

    def register(self, spec: TopicSpec) -> TopicAccumulator:
        accumulator = TopicAccumulator(
            topic_name=spec.topic_name,
            channel=spec.channel,
            expected_hz=spec.expected_hz,
            has_header=spec.has_header,
        )
        self._accumulators[spec.topic_name] = accumulator
        return accumulator

    def update(
        self, topic_name: str, receive_ns: int, stamp_ns: Optional[int] = None
    ) -> bool:
        """1 メッセージを反映する。未登録トピックなら何もせず False を返す。"""
        accumulator = self._accumulators.get(topic_name)
        if accumulator is None:
            return False
        accumulator.update(receive_ns, stamp_ns)
        return True

    def accumulator(self, topic_name: str) -> Optional[TopicAccumulator]:
        return self._accumulators.get(topic_name)

    @property
    def total_message_count(self) -> int:
        """全トピックの合計件数。StopRecording のレスポンスに載せる。"""
        return sum(a.message_count for a in self._accumulators.values())

    def duration_ns(self) -> int:
        """全トピックを通じた収録の長さ [ns]。

        最も早い first_timestamp から最も遅い last_timestamp まで。
        1 件も受信していなければ 0。
        """
        firsts = [
            a.first_timestamp for a in self._accumulators.values()
            if a.first_timestamp is not None
        ]
        lasts = [
            a.last_timestamp for a in self._accumulators.values()
            if a.last_timestamp is not None
        ]
        if not firsts or not lasts:
            return 0
        return max(0, max(lasts) - min(firsts))

    def build(self, disk: Optional[DiskStats] = None) -> TopicStats:
        """shasou-core の TopicStats を組み立てる (topic_stats.json の中身)。"""
        return TopicStats(
            drive_id=self.drive_id,
            duration_ns=self.duration_ns(),
            stats=[
                self._accumulators[name].to_stat()
                for name in sorted(self._accumulators)
            ],
            disk=disk,
        )


def load_topic_stats(path: str | os.PathLike[str]) -> TopicStats:
    """topic_stats.json を読む (manifest.load_manifest / events.load_events と同形)。

    catalog が収録の規模 (duration / message_count) を索引に載せるために読む。
    topic_stats.json を知るモジュールをここ 1 つに保つための入口で、recorder 自身が
    書いたファイルなので、壊れていれば json / pydantic の例外をそのまま通す。
    """
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return TopicStats.model_validate(data)


# --------------------------------------------------------------------------
# ディスク監視
# --------------------------------------------------------------------------


@dataclass
class DiskSnapshot:
    """ある時点のディスク状況。"""

    free_bytes: int
    total_bytes: int


def read_disk(path: str) -> DiskSnapshot:
    """指定パスが属するファイルシステムの空き容量を読む。"""
    st = os.statvfs(path)
    return DiskSnapshot(
        free_bytes=st.f_bavail * st.f_frsize,
        total_bytes=st.f_blocks * st.f_frsize,
    )


def directory_size(path: str) -> int:
    """ディレクトリ配下の合計バイト数。

    走査コストがあるので収録中には呼ばず、finalizing で 1 回だけ使う
    (total_bytes_written の測定)。
    """
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                # 収録直後の一時ファイル等が消えていても測定は続ける
                continue
    return total


class DiskMonitor:
    """別スレッドでディスク残量を監視する。

    2 つの役目がある:
      1. min_free_bytes の記録 (DiskStats 用)
      2. 閾値割れの検知 → 停止要求 (枯渇でディスクが埋まる前に、正常な
         finalizing 経路で bag を閉じるため)

    メインループ (メッセージ受信) から分離するのは、statvfs の I/O 待ちを
    収録の主処理に持ち込まないため。間隔を秒オーダーにしているのは、実車の
    収録が数十分〜数時間に及ぶので細かい粒度が要らないため。
    """

    def __init__(
        self,
        path: str,
        *,
        interval_sec: float = 5.0,
        min_free_bytes_threshold: Optional[int] = None,
        on_threshold: Optional[Callable[[DiskSnapshot], None]] = None,
        reader: Callable[[str], DiskSnapshot] = read_disk,
    ) -> None:
        self.path = path
        self.interval_sec = interval_sec
        self.min_free_bytes_threshold = min_free_bytes_threshold
        self._on_threshold = on_threshold
        self._reader = reader
        self._min_free_bytes: Optional[int] = None
        self._threshold_fired = False
        self._write_error_count = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    # ── 計測 ────────────────────────────────────────────────────────────

    def poll_once(self) -> Optional[DiskSnapshot]:
        """1 回だけ計測する。スレッドを使わないテストや手動確認用。"""
        try:
            snapshot = self._reader(self.path)
        except OSError:
            # 監視の失敗で収録を止めない (統計が欠けるだけ)
            return None

        with self._lock:
            if (
                self._min_free_bytes is None
                or snapshot.free_bytes < self._min_free_bytes
            ):
                self._min_free_bytes = snapshot.free_bytes
            fire = (
                self.min_free_bytes_threshold is not None
                and snapshot.free_bytes < self.min_free_bytes_threshold
                and not self._threshold_fired
            )
            if fire:
                self._threshold_fired = True

        # コールバックはロックの外で呼ぶ (停止要求が長引いても監視を止めない)
        if fire and self._on_threshold is not None:
            self._on_threshold(snapshot)
        return snapshot

    def record_write_error(self) -> None:
        """書き込みエラーを 1 件数える (bag writer 側から呼ぶ)。"""
        with self._lock:
            self._write_error_count += 1

    # ── スレッド制御 ────────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="shasou-disk-monitor", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            self.poll_once()
            # Event.wait を使うと停止要求に即応できる (sleep だと最大 interval 待つ)
            self._stop.wait(self.interval_sec)

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None

    # ── 結果 ────────────────────────────────────────────────────────────

    def build(self, bag_path: Optional[str] = None) -> DiskStats:
        """DiskStats を組み立てる。

        total_bytes_written は bag_path が与えられたときだけ測る
        (ディレクトリ走査のコストがあるので finalizing で 1 回だけ)。
        """
        with self._lock:
            min_free = self._min_free_bytes
            errors = self._write_error_count
        return DiskStats(
            min_free_bytes=min_free,
            total_bytes_written=directory_size(bag_path) if bag_path else None,
            write_error_count=errors,
        )

    @property
    def threshold_fired(self) -> bool:
        return self._threshold_fired
