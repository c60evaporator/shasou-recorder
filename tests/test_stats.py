import pytest

from shasou_recorder.core.stats import (
    NS_PER_SEC,
    DiskMonitor,
    DiskSnapshot,
    StatsCollector,
    TopicAccumulator,
    TopicSpec,
    directory_size,
)


def feed(acc: TopicAccumulator, start_ns: int, period_ns: int, count: int) -> None:
    """等間隔にメッセージを流し込む。"""
    for i in range(count):
        stamp = start_ns + i * period_ns
        acc.update(receive_ns=stamp, stamp_ns=stamp if acc.has_header else None)


class TestAccumulator:
    def test_empty(self):
        acc = TopicAccumulator("/t")
        assert acc.message_count == 0
        assert acc.measured_hz is None
        assert acc.drop_rate is None
        assert acc.span_ns is None
        stat = acc.to_stat()
        assert stat.first_timestamp is None and stat.last_timestamp is None

    def test_single_message_has_no_rate(self):
        acc = TopicAccumulator("/t", expected_hz=20.0)
        acc.update(receive_ns=1000, stamp_ns=1000)
        assert acc.message_count == 1
        # 1 件では間隔が測れないのでレートは出せない
        assert acc.measured_hz is None
        assert acc.drop_rate is None

    def test_measured_hz_uses_first_last_span(self):
        # 20Hz = 50ms 間隔で 21 件 -> span 1.0s, 間隔 20 個 -> 20Hz
        acc = TopicAccumulator("/t")
        feed(acc, start_ns=0, period_ns=NS_PER_SEC // 20, count=21)
        assert acc.span_ns == NS_PER_SEC
        assert acc.measured_hz == pytest.approx(20.0)

    def test_drop_rate_from_rate_ratio(self):
        # 期待 20Hz に対して実測 10Hz -> 50% ドロップ
        acc = TopicAccumulator("/t", expected_hz=20.0)
        feed(acc, start_ns=0, period_ns=NS_PER_SEC // 10, count=11)
        assert acc.measured_hz == pytest.approx(10.0)
        assert acc.drop_rate == pytest.approx(0.5)

    def test_drop_rate_clamped_when_faster_than_expected(self):
        acc = TopicAccumulator("/t", expected_hz=10.0)
        feed(acc, start_ns=0, period_ns=NS_PER_SEC // 20, count=21)
        assert acc.drop_rate == 0.0

    def test_drop_rate_none_without_expected_hz(self):
        acc = TopicAccumulator("/t")
        feed(acc, start_ns=0, period_ns=NS_PER_SEC // 20, count=21)
        assert acc.drop_rate is None

    def test_max_gap_from_header_stamps(self):
        acc = TopicAccumulator("/t")
        acc.update(receive_ns=0, stamp_ns=0)
        acc.update(receive_ns=10, stamp_ns=100)           # gap 100
        acc.update(receive_ns=20, stamp_ns=100 + 500)     # gap 500 <- 最大
        acc.update(receive_ns=30, stamp_ns=600 + 50)      # gap 50
        assert acc.max_gap_ns == 500

    def test_headerless_topic_uses_receive_time_and_no_gap(self):
        acc = TopicAccumulator("/vehicle/reverse", has_header=False)
        feed(acc, start_ns=1_000, period_ns=NS_PER_SEC, count=3)
        assert acc.message_count == 3
        assert acc.first_timestamp == 1_000
        # ヘッダが無いので最大間隔は算出しない
        assert acc.to_stat().max_gap_ns is None

    def test_backwards_stamp_ignored_for_gap(self):
        acc = TopicAccumulator("/t")
        acc.update(receive_ns=0, stamp_ns=1000)
        acc.update(receive_ns=1, stamp_ns=500)   # 逆行。間隔として数えない
        assert acc.max_gap_ns is None

    def test_to_stat_roundtrip_into_core_schema(self):
        acc = TopicAccumulator("/shasou/cam_front/image_raw/compressed",
                               channel="CAM_FRONT", expected_hz=20.0)
        feed(acc, start_ns=0, period_ns=NS_PER_SEC // 20, count=21)
        stat = acc.to_stat()
        assert stat.channel == "CAM_FRONT"
        assert stat.measured_hz == pytest.approx(20.0)
        assert stat.drop_rate == pytest.approx(0.0)


class TestCollector:
    def _collector(self) -> StatsCollector:
        return StatsCollector("drive01", [
            TopicSpec("/shasou/cam_front/image_raw/compressed",
                      channel="CAM_FRONT", expected_hz=20.0),
            TopicSpec("/shasou/lidar_top/points", channel="LIDAR_TOP", expected_hz=20.0),
            TopicSpec("/shasou/vehicle/reverse", has_header=False),
        ])

    def test_unregistered_topic_ignored(self):
        collector = self._collector()
        # 契約外の想定外トピックは統計に入れない
        assert collector.update("/some/unexpected", receive_ns=1, stamp_ns=1) is False
        assert collector.total_message_count == 0

    def test_registered_topic_counted(self):
        collector = self._collector()
        assert collector.update("/shasou/lidar_top/points", 100, 100) is True
        assert collector.total_message_count == 1

    def test_duration_spans_all_topics(self):
        collector = self._collector()
        collector.update("/shasou/lidar_top/points", 1_000, 1_000)
        collector.update("/shasou/cam_front/image_raw/compressed", 5_000, 5_000)
        collector.update("/shasou/lidar_top/points", 9_000, 9_000)
        assert collector.duration_ns() == 8_000

    def test_duration_zero_when_empty(self):
        assert self._collector().duration_ns() == 0

    def test_build_produces_core_topic_stats(self):
        collector = self._collector()
        for i in range(21):
            stamp = i * (NS_PER_SEC // 20)
            collector.update("/shasou/cam_front/image_raw/compressed", stamp, stamp)
        stats = collector.build()

        assert stats.drive_id == "drive01"
        assert stats.duration_ns == NS_PER_SEC
        # 登録した 3 トピックすべてが出る (0 件でも記録は残す)
        assert len(stats.stats) == 3
        cam = stats.stat_for("/shasou/cam_front/image_raw/compressed")
        assert cam.measured_hz == pytest.approx(20.0)
        assert cam.channel == "CAM_FRONT"
        # トピック名の重複が無いこと (core 側の validator が効く)
        assert stats.stat_for("/shasou/vehicle/reverse").message_count == 0

    def test_build_json_roundtrip(self):
        collector = self._collector()
        collector.update("/shasou/lidar_top/points", 1_000, 1_000)
        stats = collector.build()
        restored = type(stats).model_validate_json(stats.model_dump_json())
        assert restored == stats


class TestDiskMonitor:
    def test_min_free_tracked(self):
        readings = iter([
            DiskSnapshot(free_bytes=100, total_bytes=1000),
            DiskSnapshot(free_bytes=40, total_bytes=1000),
            DiskSnapshot(free_bytes=70, total_bytes=1000),
        ])
        monitor = DiskMonitor("/tmp", reader=lambda _p: next(readings))
        for _ in range(3):
            monitor.poll_once()
        assert monitor.build().min_free_bytes == 40

    def test_threshold_fires_once(self):
        fired: list[int] = []
        readings = iter([
            DiskSnapshot(free_bytes=100, total_bytes=1000),
            DiskSnapshot(free_bytes=10, total_bytes=1000),
            DiskSnapshot(free_bytes=5, total_bytes=1000),
        ])
        monitor = DiskMonitor(
            "/tmp",
            min_free_bytes_threshold=50,
            on_threshold=lambda snap: fired.append(snap.free_bytes),
            reader=lambda _p: next(readings),
        )
        for _ in range(3):
            monitor.poll_once()
        # 閾値割れは 2 回起きるが、停止要求は 1 度だけ
        assert fired == [10]
        assert monitor.threshold_fired

    def test_read_failure_does_not_raise(self):
        def failing(_path):
            raise OSError("no such fs")

        monitor = DiskMonitor("/nonexistent", reader=failing)
        assert monitor.poll_once() is None
        assert monitor.build().min_free_bytes is None

    def test_write_error_count(self):
        monitor = DiskMonitor("/tmp", reader=lambda _p: DiskSnapshot(1, 2))
        monitor.record_write_error()
        monitor.record_write_error()
        assert monitor.build().write_error_count == 2

    def test_thread_start_stop(self):
        calls: list[int] = []

        def reader(_path):
            calls.append(1)
            return DiskSnapshot(free_bytes=100, total_bytes=1000)

        monitor = DiskMonitor("/tmp", interval_sec=0.01, reader=reader)
        monitor.start()
        import time
        time.sleep(0.05)
        monitor.stop()
        assert len(calls) >= 2

    def test_total_bytes_written_measured_from_path(self, tmp_path):
        (tmp_path / "a.mcap").write_bytes(b"x" * 100)
        (tmp_path / "sub").mkdir()
        (tmp_path / "sub" / "b.mcap").write_bytes(b"y" * 50)
        monitor = DiskMonitor(str(tmp_path), reader=lambda _p: DiskSnapshot(1, 2))
        disk = monitor.build(bag_path=str(tmp_path))
        assert disk.total_bytes_written == 150
        assert directory_size(str(tmp_path)) == 150
