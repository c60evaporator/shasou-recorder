"""ファイルの原子的な書き出し (core/ 共通)。

finalizing (§4.4) が産む成果物は、いずれも「中途半端な状態のファイルが見える」と
下流の判定を誤らせる:

- `manifest.yaml`: 存在が「ドライブが完成した」印なので、壊れた manifest が
  残ると完成判定そのものが誤る
- `events.jsonl` / `topic_stats.json`: 途中で切れた行やオブジェクトは、読み手が
  パースに失敗する

そこで「一時ファイルに書き切ってから rename する」手順をここに集約する。
YAML 固有ではないので yamlio.py には置かない — JSON / JSONL の書き手が
そこを探しに来ないため。
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path


def write_text_atomic(path: Path, text: str) -> None:
    """同じディレクトリの一時ファイルに書いてから rename する。

    車載 Jetson は走行中の電源断がありうるため、rename の耐久性まで確保する:
    本体を fsync してから replace し、親ディレクトリも fsync する (でないと
    rename 自体がディスクに届いていない場合がある)。

    一時ファイルを同じディレクトリに作るのは、rename が原子的なのは同一
    ファイルシステム内だけだから。
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
        # mkstemp は 0600 で作るので、通常のファイル作成と同じ見え方に直す。
        # NAS 経由で studio (別ユーザー) が読む経路があるため。
        os.chmod(tmp, 0o644)
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
