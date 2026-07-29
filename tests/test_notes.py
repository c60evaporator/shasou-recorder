from pathlib import Path

from shasou_recorder.core.config import DriveOptions
from shasou_recorder.core.layout import DataLayout, DriveLayout
from shasou_recorder.core.notes import NotesWriter

PLATFORM = "platform_lincoln_6cam-lidar"
DRIVE_ID = "2026-07-16_1030_vehicle01_osaka-umeda"


def drive_for(tmp_path: Path) -> DriveLayout:
    drive = DataLayout(root=tmp_path / "data").drive(PLATFORM, DRIVE_ID)
    drive.create()
    return drive


def files_in(drive: DriveLayout) -> list[str]:
    """ドライブ直下のファイル名 (bags/ tags/ health/ は create() が作る)。"""
    return sorted(p.name for p in drive.root.iterdir() if p.is_file())


class TestWrite:
    def test_writes_to_the_layout_path(self, tmp_path):
        drive = drive_for(tmp_path)

        path = NotesWriter.for_drive(drive, DriveOptions(notes="雨天テスト")).write()

        assert path == drive.notes
        assert path.parent == drive.root

    def test_text_is_written_verbatim(self, tmp_path):
        # 見出し等を生成すると、人が書いた分と recorder が書いた分の境界が消える
        drive = drive_for(tmp_path)
        text = "梅田交差点で右折レーンの割り込みを狙う。\n2 周目はワイパー ON。"

        path = NotesWriter.for_drive(drive, DriveOptions(notes=text)).write()

        assert path.read_text(encoding="utf-8") == f"{text}\n"

    def test_trailing_newline_is_not_doubled(self, tmp_path):
        drive = drive_for(tmp_path)

        path = NotesWriter.for_drive(drive, DriveOptions(notes="ok\n")).write()

        assert path.read_text(encoding="utf-8") == "ok\n"

    def test_empty_notes_creates_no_file(self, tmp_path):
        # events.jsonl と違い、読み手が人だけなので空ファイルは作らない
        drive = drive_for(tmp_path)

        assert NotesWriter.for_drive(drive, DriveOptions()).write() is None
        assert NotesWriter.for_drive(drive, DriveOptions(notes="")).write() is None
        assert not drive.notes.exists()
        assert files_in(drive) == []

    def test_whitespace_only_notes_creates_no_file(self, tmp_path):
        # DriveOptions が空白のみを "" に正規化する (config.py のマージ規則)
        drive = drive_for(tmp_path)

        assert NotesWriter.for_drive(drive, DriveOptions(notes="   ")).write() is None
        assert not drive.notes.exists()

    def test_no_temp_file_is_left_behind(self, tmp_path):
        drive = drive_for(tmp_path)

        NotesWriter.for_drive(drive, DriveOptions(notes="ok")).write()

        assert files_in(drive) == ["notes.md"]

    def test_file_is_group_readable(self, tmp_path):
        # NAS 経由で studio (別ユーザー) が読む
        drive = drive_for(tmp_path)

        path = NotesWriter.for_drive(drive, DriveOptions(notes="ok")).write()

        assert path.stat().st_mode & 0o777 == 0o644

    def test_overwrites_a_previous_write(self, tmp_path):
        drive = drive_for(tmp_path)
        NotesWriter.for_drive(drive, DriveOptions(notes="first")).write()

        NotesWriter.for_drive(drive, DriveOptions(notes="second")).write()

        assert drive.notes.read_text(encoding="utf-8") == "second\n"
