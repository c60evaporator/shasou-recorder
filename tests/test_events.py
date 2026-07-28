import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from shasou_core.schemas.events import EventSource, EventTag, EventType

from shasou_recorder.core.events import (
    EventCollector,
    EventsWriter,
    dump_events,
    load_events,
)
from shasou_recorder.core.layout import DataLayout, DriveLayout
from shasou_recorder.core.session import (
    FinalizeStep,
    Finalizers,
    RecordingSession,
    StopReason,
    StopRequest,
)

PLATFORM = "platform_lincoln_6cam-lidar"
DRIVE_ID = "2026-07-16_1030_vehicle01_osaka-umeda"

# §10 の例と同じ 2 件
CUT_IN = EventTag(
    timestamp=1752641234512000000,
    type=EventType.INTERESTING,
    label="cut-in",
    source=EventSource.DRIVER_BUTTON,
)
CONSTRUCTION = EventTag(
    timestamp=1752641301220000000,
    type=EventType.MARKER,
    label="construction zone",
    source=EventSource.TABLET,
)


def event(timestamp: int, label: str = "cut-in", **overrides) -> EventTag:
    fields = {
        "timestamp": timestamp,
        "type": EventType.INTERESTING,
        "label": label,
        "source": EventSource.DRIVER_BUTTON,
    }
    fields.update(overrides)
    return EventTag(**fields)


def drive_for(tmp_path: Path) -> DriveLayout:
    drive = DataLayout(root=tmp_path / "data").drive(PLATFORM, DRIVE_ID)
    drive.create()
    return drive


class TestCollector:
    def test_collects_events(self):
        collector = EventCollector()
        collector.add(CUT_IN)
        collector.add(CONSTRUCTION)

        assert len(collector) == 2
        assert list(collector) == [CUT_IN, CONSTRUCTION]

    def test_starts_empty(self):
        assert len(EventCollector()) == 0
        assert EventCollector().sorted_events() == []

    def test_sorted_by_timestamp(self):
        # 受信順と時刻順が食い違う入力 (まとめて送ってくる入力デバイス等)
        collector = EventCollector()
        collector.add(event(300, "third"))
        collector.add(event(100, "first"))
        collector.add(event(200, "second"))

        assert [e.label for e in collector.sorted_events()] == [
            "first", "second", "third",
        ]

    def test_iteration_keeps_receive_order(self):
        # 時刻順が要るときだけ sorted_events() を使う
        collector = EventCollector()
        collector.add(event(300, "third"))
        collector.add(event(100, "first"))

        assert [e.label for e in collector] == ["third", "first"]

    def test_sort_is_stable_for_equal_timestamps(self):
        # 時刻の分解能を超えて同時のイベントは受信順を保つ
        collector = EventCollector()
        for label in ("a", "b", "c"):
            collector.add(event(1752641234512000000, label))

        assert [e.label for e in collector.sorted_events()] == ["a", "b", "c"]


class TestDump:
    def test_one_object_per_line(self):
        text = dump_events([CUT_IN, CONSTRUCTION])

        lines = text.splitlines()
        assert len(lines) == 2
        assert text.endswith("\n")
        assert [json.loads(line)["label"] for line in lines] == [
            "cut-in", "construction zone",
        ]

    def test_field_order_matches_the_documented_example(self):
        line = dump_events([CUT_IN]).splitlines()[0]

        assert list(json.loads(line)) == ["timestamp", "type", "label", "source"]
        assert line.startswith('{"timestamp": 1752641234512000000, "type": ')

    def test_enums_are_written_as_values(self):
        record = json.loads(dump_events([CONSTRUCTION]).splitlines()[0])

        assert record["type"] == "marker"
        assert record["source"] == "tablet"

    def test_japanese_is_readable(self):
        text = dump_events([event(100, "工事区間")])

        # エスケープされると運用者が読めない (§10 は日本語圏の運用を想定)
        assert "工事区間" in text
        assert "\\u" not in text

    def test_timestamp_stays_an_integer(self):
        text = dump_events([CUT_IN])

        assert "1752641234512000000" in text
        assert ".0" not in text
        assert isinstance(json.loads(text.splitlines()[0])["timestamp"], int)

    def test_core_rejects_float_timestamps(self):
        # このモジュールが「ns 整数しか来ない」と前提できる根拠 (core の strict=True)
        with pytest.raises(ValidationError):
            EventTag(
                timestamp=1752641234.512,
                type=EventType.INTERESTING,
                label="cut-in",
                source=EventSource.DRIVER_BUTTON,
            )
        with pytest.raises(ValidationError):
            # 整数値の float も拒否される (秒を ns 欄に入れる事故を防ぐため)
            EventTag(
                timestamp=1752641234512000000.0,
                type=EventType.INTERESTING,
                label="cut-in",
                source=EventSource.DRIVER_BUTTON,
            )

    def test_empty_is_an_empty_string(self):
        assert dump_events([]) == ""


class TestWriter:
    def files_in(self, drive: DriveLayout) -> list[str]:
        return sorted(p.name for p in drive.tags_dir.iterdir() if p.is_file())

    def test_writes_to_the_layout_path(self, tmp_path):
        drive = drive_for(tmp_path)
        collector = EventCollector([CUT_IN])

        path = EventsWriter.for_drive(drive, collector).write()

        assert path == drive.events
        assert path.parent == drive.tags_dir

    def test_roundtrip(self, tmp_path):
        drive = drive_for(tmp_path)
        collector = EventCollector()
        collector.add(CONSTRUCTION)
        collector.add(CUT_IN)

        path = EventsWriter.for_drive(drive, collector).write()

        # 時刻順で読み戻せること
        assert load_events(path) == [CUT_IN, CONSTRUCTION]
        # 1 行ずつ core の EventTag で読めること
        lines = path.read_text(encoding="utf-8").splitlines()
        assert [EventTag.model_validate(json.loads(line)) for line in lines] == [
            CUT_IN, CONSTRUCTION,
        ]

    def test_empty_creates_an_empty_file(self, tmp_path):
        # 「無い」と「空」の 2 状態を下流に作らない。finalizing の 3 番目まで
        # 到達した証跡にもなる
        drive = drive_for(tmp_path)

        path = EventsWriter.for_drive(drive, EventCollector()).write()

        assert path.is_file()
        assert path.read_text(encoding="utf-8") == ""
        assert load_events(path) == []

    def test_reads_the_collector_at_write_time(self, tmp_path):
        # イベントは停止直前まで増える。構築時のスナップショットではない
        drive = drive_for(tmp_path)
        collector = EventCollector()
        writer = EventsWriter.for_drive(drive, collector)

        collector.add(CUT_IN)
        path = writer.write()

        assert load_events(path) == [CUT_IN]

    def test_no_temp_file_is_left_behind(self, tmp_path):
        drive = drive_for(tmp_path)

        EventsWriter.for_drive(drive, EventCollector([CUT_IN])).write()

        assert self.files_in(drive) == ["events.jsonl"]

    def test_file_is_group_readable(self, tmp_path):
        # NAS 経由で studio (別ユーザー) が読む
        drive = drive_for(tmp_path)

        path = EventsWriter.for_drive(drive, EventCollector([CUT_IN])).write()

        assert path.stat().st_mode & 0o777 == 0o644

    def test_overwrites_a_previous_write(self, tmp_path):
        drive = drive_for(tmp_path)
        collector = EventCollector()
        writer = EventsWriter.for_drive(drive, collector)
        writer.write()

        collector.add(CUT_IN)
        writer.write()

        assert load_events(drive.events) == [CUT_IN]


class TestFinalizeIntegration:
    def test_events_step_writes_the_artifact(self, tmp_path):
        drive = drive_for(tmp_path)
        collector = EventCollector([CONSTRUCTION, CUT_IN])

        session = RecordingSession(DRIVE_ID)
        session.begin_preflight()
        session.start_recording()
        session.request_stop(StopRequest(StopReason.SERVICE, completed=True))
        result = session.finalize(
            Finalizers(
                bag=_Noop(),
                stats=_Noop(),
                events=EventsWriter.for_drive(drive, collector),
                manifest=_Noop(),
                catalog=_Noop(),
            )
        )

        assert result.success
        assert FinalizeStep.EVENTS in result.completed_steps
        assert drive.events in result.artifacts
        assert load_events(drive.events) == [CUT_IN, CONSTRUCTION]


class _Noop:
    """BagWriter / ArtifactWriter / CatalogUpdater のダミー。"""

    def close(self):
        return None

    def write(self):
        return None

    def update(self) -> None:
        return None
