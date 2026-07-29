"""notes.md の書き出し (§10 の「自由記述 (任意)」)。

`DriveOptions.notes` (CLI の ``--notes`` や設定の ``drive_defaults``) をドライブ
直下に残す。manifest のフィールドではないので manifest.py には置かない —
manifest.py が持つのは DriveManifest の各フィールドをどこから採るかの表であり、
notes はそこに載らない別の成果物だから (events.jsonl と同じ位置づけ)。

**収録開始時に書く (finalizing の手順ではない)**
---------------------------------------------
`FINALIZE_ORDER` には載せず、drive_id を採番してディレクトリを作った直後
(`DriveAllocator.allocate()` の後) に呼ぶこと。呼び出しは ros/node.py と CLI の
責務。理由は 3 つ:

- 内容が開始時点で確定している。DriveOptions は「設定 < CLI < StartRecording」の
  マージ結果で、収録中は変わらない
- **notes.md は人が後から書き足すファイル。** finalizing で書くと、収録中に
  運用者がエディタで書いた内容を上書きして消してしまう
- 収録が異常終了して manifest が書けなかったドライブ (= 不完全) でこそ、
  「何を狙った走行だったか」が手動復旧の手がかりになる。開始時に書けば残る

このモジュールは core/ にあるので ROS に依存しない。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .atomicio import write_text_atomic
from .config import DriveOptions
from .layout import DriveLayout


@dataclass(frozen=True)
class NotesWriter:
    """notes.md を書き出す。

    メソッド名は他の Writer (ManifestWriter / EventsWriter) に合わせて `write()`
    にしてあり、結果として `session.ArtifactWriter` と同じ形になるが、
    **finalizing の手順には載せない** (モジュール docstring)。
    """

    text: str
    path: Path

    @classmethod
    def for_drive(cls, drive: DriveLayout, options: DriveOptions) -> "NotesWriter":
        """ドライブのレイアウトと走行ごとの値から組み立てる。

        `options.notes` の None (未指定) と "" (明示的な空) はどちらも「この走行に
        記述は無い」なので、ここで空文字に畳む (config.py のマージ規則と同じ扱い)。
        """
        return cls(text=options.notes or "", path=drive.notes)

    def write(self) -> Optional[Path]:
        """notes.md を書き出して、そのパスを返す。**空なら作らずに None を返す。**

        events.jsonl は 0 件でも空ファイルを作るが、notes.md は作らない。両者は
        読み手が違う — events.jsonl は studio の取り込みが読むので「ファイルが
        無い」と「中身が空」の 2 状態を作らない方がよいが、notes.md を読むのは
        人だけで、プログラムの分岐は発生しない。全ドライブに空ファイルが増える分
        だけノイズになるので作らない (「ファイルが無い = 記述が無い」で曖昧さも
        生じない)。

        本文は**加工せずそのまま**書く (末尾の改行だけ保証)。見出し等を生成すると、
        人が書いた分と recorder が書いた分の境界が曖昧になる。
        """
        if not self.text:
            return None

        # 原子的に書くのは manifest / events と同じ理由ではなく (完成判定には
        # 使わない)、write_text_atomic が 0644 で置くため。mkstemp 直書きだと
        # 0600 になり、NAS 経由で studio 側から読めない。
        text = self.text if self.text.endswith("\n") else f"{self.text}\n"
        write_text_atomic(self.path, text)
        return self.path
