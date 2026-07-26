"""収録セッションの状態機械。

1 セッション = 1 drive (= nuScenes の 1 log)。セッションは使い捨てで、次の
ドライブは新しいインスタンスを作る (状態の持ち越しによる汚染を避けるため)。

このモジュールは core/ にあるので ROS に依存しない。bag の書き込みや成果物の
生成といった I/O の実体は Protocol として宣言し、呼び出し側 (ros/node.py) が
注入する。セッションが持つのは「どの手順を、どの順で、失敗したらどうするか」
という順序と方針だけ。

状態遷移
--------
    IDLE → PREFLIGHT → RECORDING → FINALIZING → DONE
                    ↘ (abort)                  ↗

停止要求
--------
停止は複数の経路から来る (Ctrl+C / StopRecording サービス / ディスク枯渇 /
将来のユーザーデバイス)。すべて request_stop() に集約し、最初の要求だけを
採用する (先に来た理由が本当の原因である可能性が高いため)。二度目以降は
無視して成功扱いにする — StopRecording の二重呼び出しや、Ctrl+C 直後の
サービス呼び出しといった競合が実際に起こりうる。

出力先パスの持ち方
------------------
各 Writer が自分の出力先を知る (セッションは渡さない)。セッションをフォルダ
構成 (layout) から独立させ、順序制御だけに専念させるため。Writer は生成時に
出力先を注入され、write() は書き出したパスを返す。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Callable, Optional, Protocol, Sequence


# --------------------------------------------------------------------------
# 状態と停止理由
# --------------------------------------------------------------------------


class SessionState(str, Enum):
    IDLE = "idle"              # 生成直後。まだ何も始まっていない
    PREFLIGHT = "preflight"    # 収録前検証中 (トピック存在確認など)
    RECORDING = "recording"    # 収録中
    FINALIZING = "finalizing"  # 停止要求を受けて成果物を書き出し中
    DONE = "done"              # 完了。このセッションは再利用しない


class StopReason(str, Enum):
    """停止要求の出所。manifest の tags や topic_stats に残して、後から
    「この収録はなぜ終わったか」を追えるようにする。"""

    SERVICE = "service"          # StopRecording サービス (CARLA 等の外部制御)
    SIGNAL = "signal"            # SIGINT (Ctrl+C)
    DISK_FULL = "disk_full"      # ディスク残量が閾値を下回った
    TOPIC_TIMEOUT = "topic_timeout"  # 期待トピックが途絶した
    ERROR = "error"              # 収録処理自体の異常


@dataclass(frozen=True)
class StopRequest:
    """停止要求。複数経路から来るものを 1 つの型で表す。"""

    reason: StopReason
    detail: str = ""
    # ルートが正常完走したか。StopRecording サービス由来のときだけ意味を持ち、
    # それ以外の経路 (Ctrl+C / ディスク枯渇) では False 相当として扱う。
    completed: bool = False


class SessionStateError(RuntimeError):
    """不正な状態遷移。"""


# --------------------------------------------------------------------------
# finalizing の各手順 (実体は注入される)
# --------------------------------------------------------------------------


class BagWriter(Protocol):
    """bag への書き込み担当 (実体は ros/ 側)。"""

    def close(self) -> Optional[Path]:
        """bag をクローズし、bag のパスを返す。"""


class ArtifactWriter(Protocol):
    """成果物を 1 ファイル書き出す担当。出力先は自分で知っている。"""

    def write(self) -> Optional[Path]:
        """書き出して、そのパスを返す。"""


class CatalogUpdater(Protocol):
    """catalog へのドライブ登録・更新担当。"""

    def update(self) -> None: ...


class FinalizeStep(str, Enum):
    """finalizing の手順。この順序で実行する。

    manifest を後半に置くのは、manifest の存在が「ドライブが完成した」印に
    なるため (manifest があれば有効なドライブ、無ければ不完全と判定できる)。
    """

    BAG_CLOSE = "bag_close"
    TOPIC_STATS = "topic_stats"
    EVENTS = "events"
    MANIFEST = "manifest"
    CATALOG = "catalog"


FINALIZE_ORDER: tuple[FinalizeStep, ...] = (
    FinalizeStep.BAG_CLOSE,
    FinalizeStep.TOPIC_STATS,
    FinalizeStep.EVENTS,
    FinalizeStep.MANIFEST,
    FinalizeStep.CATALOG,
)


@dataclass
class Finalizers:
    """finalizing の各手順の実体。すべて optional にはせず、呼び出し側が
    明示的に組み立てる (取り違えを防ぐため)。"""

    bag: BagWriter
    stats: ArtifactWriter
    events: ArtifactWriter
    manifest: ArtifactWriter
    catalog: CatalogUpdater


@dataclass
class FinalizeResult:
    """finalizing の結果。呼び出し側 (ros/node.py) はこれを見て
    StopRecording のレスポンスを組み立てる。"""

    success: bool
    completed_steps: list[FinalizeStep] = field(default_factory=list)
    failed_step: Optional[FinalizeStep] = None
    error: str = ""
    artifacts: list[Path] = field(default_factory=list)
    duration_sec: float = 0.0
    stop_request: Optional[StopRequest] = None

    @property
    def manifest_written(self) -> bool:
        """manifest が書けたか = このドライブが完成しているか。"""
        return FinalizeStep.MANIFEST in self.completed_steps


# --------------------------------------------------------------------------
# セッション
# --------------------------------------------------------------------------


class RecordingSession:
    """1 ドライブ分の収録セッション。使い捨て。

    呼び出し側の典型的な流れ:

        session = RecordingSession(drive_id)
        session.begin_preflight()
        ...                              # トピック存在確認など (実体は呼び出し側)
        session.start_recording()
        ...                              # 収録
        session.request_stop(StopRequest(StopReason.SIGNAL))
        result = session.finalize(finalizers)
    """

    def __init__(
        self,
        drive_id: str,
        *,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.drive_id = drive_id
        self._clock = clock
        self._state = SessionState.IDLE
        self._stop_request: Optional[StopRequest] = None
        self._started_at: Optional[float] = None
        self._stopped_at: Optional[float] = None

    # ── 状態 ────────────────────────────────────────────────────────────

    @property
    def state(self) -> SessionState:
        return self._state

    @property
    def stop_request(self) -> Optional[StopRequest]:
        """採用された停止要求 (最初の 1 件)。"""
        return self._stop_request

    @property
    def duration_sec(self) -> float:
        """収録時間。停止前は現時点までの経過時間を返す。

        収録開始から停止要求までの実時間であり、bag 内の first/last メッセージ
        の差 (stats が measured_hz に使う値) とは別物。
        """
        if self._started_at is None:
            return 0.0
        end = self._stopped_at if self._stopped_at is not None else self._clock()
        return end - self._started_at

    def _require(self, *allowed: SessionState) -> None:
        if self._state not in allowed:
            expected = " / ".join(s.value for s in allowed)
            raise SessionStateError(
                f"drive {self.drive_id}: 状態 {self._state.value} では実行できない "
                f"(期待: {expected})"
            )

    # ── 遷移 ────────────────────────────────────────────────────────────

    def begin_preflight(self) -> None:
        """IDLE → PREFLIGHT。収録前検証に入る。"""
        self._require(SessionState.IDLE)
        self._state = SessionState.PREFLIGHT

    def start_recording(self) -> None:
        """PREFLIGHT → RECORDING。検証が通ったので収録を始める。"""
        self._require(SessionState.PREFLIGHT)
        self._state = SessionState.RECORDING
        self._started_at = self._clock()

    def abort(self, request: Optional[StopRequest] = None) -> None:
        """PREFLIGHT → DONE。収録に入る前に打ち切る。

        検証に失敗した場合や、収録開始前に停止要求が来た場合に使う。bag は
        まだ開いていないので finalizing は行わず、成果物も残らない。
        """
        self._require(SessionState.PREFLIGHT)
        if request is not None and self._stop_request is None:
            self._stop_request = request
        self._state = SessionState.DONE

    def request_stop(self, request: StopRequest) -> bool:
        """停止を要求する。採用されたら True、既に要求済みなら False。

        冪等: 二度目以降は無視して False を返す (エラーにはしない)。
        IDLE での要求だけはエラー — 何も収録していないため。
        """
        self._require(
            SessionState.PREFLIGHT,
            SessionState.RECORDING,
            SessionState.FINALIZING,
            SessionState.DONE,
        )
        if self._stop_request is not None:
            return False
        self._stop_request = request
        if self._stopped_at is None and self._started_at is not None:
            self._stopped_at = self._clock()
        return True

    # ── finalizing ──────────────────────────────────────────────────────

    def finalize(self, finalizers: Finalizers) -> FinalizeResult:
        """RECORDING → FINALIZING → DONE。成果物を FINALIZE_ORDER の順に書き出す。

        失敗しても例外は投げず FinalizeResult に載せて返す (呼び出し側が
        StopRecording のレスポンスに変換するため)。最初の失敗で打ち切り、
        そこまでの成果物はディスクに残す — bag さえ残っていれば後から手動で
        復旧できる。manifest 前に失敗すれば manifest は書かれないので、
        「manifest が無いドライブ = 不完全」の判定が成り立つ。
        """
        self._require(SessionState.RECORDING)
        self._state = SessionState.FINALIZING
        if self._stopped_at is None:
            self._stopped_at = self._clock()

        def update_catalog() -> Optional[Path]:
            # catalog はファイルを産まないので成果物パスは無い
            finalizers.catalog.update()
            return None

        actions: dict[FinalizeStep, Callable[[], Optional[Path]]] = {
            FinalizeStep.BAG_CLOSE: finalizers.bag.close,
            FinalizeStep.TOPIC_STATS: finalizers.stats.write,
            FinalizeStep.EVENTS: finalizers.events.write,
            FinalizeStep.MANIFEST: finalizers.manifest.write,
            FinalizeStep.CATALOG: update_catalog,
        }

        result = FinalizeResult(
            success=True,
            duration_sec=self.duration_sec,
            stop_request=self._stop_request,
        )
        for step in FINALIZE_ORDER:
            try:
                path = actions[step]()
            except Exception as error:  # noqa: BLE001 — 失敗も結果として返す
                result.success = False
                result.failed_step = step
                result.error = f"{type(error).__name__}: {error}"
                break
            result.completed_steps.append(step)
            if path is not None:
                result.artifacts.append(path)

        self._state = SessionState.DONE
        return result

    # ── 補助 ────────────────────────────────────────────────────────────

    def stop_tags(self) -> dict[str, str]:
        """停止理由を manifest の tags 用に文字列化する。

        DriveManifest.tags のキーは小文字 snake_case でなければならない
        (shasou-core の TAG_KEY_PATTERN)。
        """
        if self._stop_request is None:
            return {}
        tags = {
            "stop_reason": self._stop_request.reason.value,
            "completed": "true" if self._stop_request.completed else "false",
        }
        if self._stop_request.detail:
            tags["stop_detail"] = self._stop_request.detail
        return tags


def remaining_steps(completed: Sequence[FinalizeStep]) -> list[FinalizeStep]:
    """未実行の手順。復旧処理が「どこから再開すべきか」を判断するのに使う。"""
    done = set(completed)
    return [step for step in FINALIZE_ORDER if step not in done]
