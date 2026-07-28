"""events.jsonl の生成 (§10) と書き出し (§4.4 の 3 番目)。

**events.jsonl は bag からの派生物で、正は bag 側 (§2)。** イベントは
`shasou_msgs/msg/EventTag` トピックとして bag に記録されている (core の
topics.py の `EVENTS` 契約は `recorded=True`) ので、jsonl はいつでも bag から
再生成できる。両者が食い違ったら bag が正。jsonl はあくまで「bag を開かずに
イベントを一覧できる」ための索引にすぎない。

再生成そのもの (MCAP を読み直して jsonl を作る) は ros/ の責務なので、この
モジュールは持たない。

蓄積の方式
----------
収録中はメモリに溜め、finalizing で一括して書き出す。追記方式にしない理由:

- **時刻順に並べられなくなる。** 追記は受信順にしか書けず、CARLA の
  シミュレーション時刻や、まとめて送ってくる入力デバイスから受信順 ≠ 時刻順の
  並びが来たときに直しようがない
- 途中でプロセスが落ちても情報は失われない (上記のとおり bag が正で、jsonl は
  再生成できる)。追記方式の利点である「落ちてもそこまでは残る」が、この
  構造では利点にならない
- イベントは人がボタンを押す頻度なのでメモリ量は問題にならず、受信経路に
  ファイル I/O を持ち込まずに済む

§10 の「追記のみ」は **jsonl というファイル形式の性質** (行を書き足すだけで
既存行を書き換えない) と読む。収録中に追記し続けよという要求と解釈すると、
上記の時刻順と両立しない。

このモジュールは core/ にあるので ROS に依存しない。ROS の Header.stamp から
ns 整数への変換は ros/converters.py の責務で、ここが受け取るのは変換済みの
core の EventTag。
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Iterator, List

from shasou_core.schemas.events import EventTag

from .atomicio import write_text_atomic
from .layout import DriveLayout


# --------------------------------------------------------------------------
# 収録中の受け口
# --------------------------------------------------------------------------


@dataclass
class EventCollector:
    """収録中に届いた EventTag を溜める (stats.StatsCollector と同じ役回り)。

    呼び出し側が受信のたびに add() を呼ぶので、ロックは持たない — ROS の
    実行モデル上、単一の executor スレッドから呼ばれる前提。別スレッドから
    呼ぶ場合は呼び出し側で直列化すること。

    **収録停止後は購読を止めてから finalize すること。** bag のクローズ
    (finalizing の 1 番目) より後に届いたイベントを jsonl に載せると、
    「bag に無いイベントが jsonl にある」状態になり、§2 の「正は bag 側」が
    崩れる。
    """

    events: List[EventTag] = field(default_factory=list)

    def add(self, event: EventTag) -> None:
        """1 件を記録する。受信のたびに呼ばれるので追加以外は何もしない。

        検証は EventTag を構築した時点で core が済ませている (timestamp は
        strict=True で float を全面拒否する)。ここで再検証はしない。
        """
        self.events.append(event)

    def __len__(self) -> int:
        return len(self.events)

    def __iter__(self) -> Iterator[EventTag]:
        """**受信順**で返す。時刻順が要るときは sorted_events() を使う。"""
        return iter(self.events)

    def sorted_events(self) -> list[EventTag]:
        """timestamp 昇順に並べ替えて返す (安定ソート)。

        下流 (studio) は時刻でシーンを切るので、jsonl も時刻順の方が扱いやすい。
        安定ソートなので、同一 timestamp のイベントは受信順のまま残る
        (時刻の分解能を超えて同時に押されたボタンの前後関係を捨てない)。
        """
        return sorted(self.events, key=lambda event: event.timestamp)


# --------------------------------------------------------------------------
# JSONL の読み書き
# --------------------------------------------------------------------------


def dump_events(events: Iterable[EventTag]) -> str:
    """EventTag の並びを JSONL 文字列にする (1 行 1 オブジェクト)。

    `ensure_ascii=False` は label に日本語が入りうるため (§10 の例も日本語圏の
    運用を想定している)。エスケープすると運用者が読めなくなる。

    フィールド順は EventTag の定義順 (timestamp / type / label / source) で、
    §10 の例と一致する。timestamp は core の TimestampNs (ns 整数) なので、
    `mode="json"` でも整数のまま出る (秒の float が紛れ込む余地は無い)。
    """
    lines = [
        json.dumps(event.model_dump(mode="json"), ensure_ascii=False)
        for event in events
    ]
    # 各行を改行で終える (最終行も含む)。イベントが 0 件なら空文字列。
    return "".join(f"{line}\n" for line in lines)


def load_events(path: str | os.PathLike[str]) -> list[EventTag]:
    """events.jsonl を読む (1 行 1 EventTag)。

    空行は飛ばす。行の不正は pydantic の ValidationError / json の
    JSONDecodeError がそのまま出る — recorder 自身が書いたファイルなので、
    定義ファイル (definitions.py) のように同期漏れを案内する必要が無い。
    """
    text = Path(path).read_text(encoding="utf-8")
    return [
        EventTag.model_validate(json.loads(line))
        for line in text.splitlines()
        if line.strip()
    ]


@dataclass(frozen=True)
class EventsWriter:
    """events.jsonl を書き出す (session.ArtifactWriter を満たす)。

    出力先は自分で知っている — セッションはフォルダ構成から独立させ、順序制御
    だけに専念させる方針 (session.py の docstring)。

    **collector を保持し、write() の時点で中身を読む。** manifest と違って
    イベントは停止直前まで増えるので、構築時にスナップショットを取ってはならない。
    """

    collector: EventCollector
    path: Path

    @classmethod
    def for_drive(cls, drive: DriveLayout, collector: EventCollector) -> "EventsWriter":
        """ドライブのレイアウトから出力先を決めて組み立てる。"""
        return cls(collector=collector, path=drive.events)

    def write(self) -> Path:
        """events.jsonl を原子的に書き出して、そのパスを返す。

        **イベントが 0 件でも空ファイルを作る。** §10 が「実機でイベント入力
        デバイスが無い間は購読だけ実装して空ファイルでよい」としているのに加え、
        消費側 (studio の取り込み) が「ファイルが無い」と「中身が空」の 2 状態を
        扱わずに済む — どちらも「イベント無し」を意味するのに分岐が要るのは無駄。
        さらに、ファイルの存在が「finalizing の 3 番目まで到達した」証跡になる
        (manifest が完成の印であるのと同じ理屈)。
        """
        write_text_atomic(self.path, dump_events(self.collector.sorted_events()))
        return self.path
