from pathlib import Path

import pytest

from shasou_recorder.core.session import (
    FINALIZE_ORDER,
    FinalizeStep,
    Finalizers,
    RecordingSession,
    SessionState,
    SessionStateError,
    StopReason,
    StopRequest,
    remaining_steps,
)


class RecordingWriter:
    """呼び出し順を記録するテストダブル。fail_with を渡すと失敗する。"""

    def __init__(self, log: list[str], name: str, path: Path | None = None,
                 fail_with: Exception | None = None) -> None:
        self._log = log
        self._name = name
        self._path = path
        self._fail_with = fail_with

    def _run(self) -> Path | None:
        self._log.append(self._name)
        if self._fail_with is not None:
            raise self._fail_with
        return self._path

    # BagWriter / ArtifactWriter / CatalogUpdater のいずれとしても使える
    close = _run
    write = _run
    update = _run


def make_finalizers(log: list[str], failing: FinalizeStep | None = None) -> Finalizers:
    def writer(step: FinalizeStep, path: Path | None) -> RecordingWriter:
        error = RuntimeError(f"{step.value} boom") if failing is step else None
        return RecordingWriter(log, step.value, path, error)

    return Finalizers(
        bag=writer(FinalizeStep.BAG_CLOSE, Path("/d/bags")),
        stats=writer(FinalizeStep.TOPIC_STATS, Path("/d/health/topic_stats.json")),
        events=writer(FinalizeStep.EVENTS, Path("/d/tags/events.jsonl")),
        manifest=writer(FinalizeStep.MANIFEST, Path("/d/manifest.yaml")),
        catalog=writer(FinalizeStep.CATALOG, None),
    )


def running_session(drive_id: str = "drive01") -> RecordingSession:
    session = RecordingSession(drive_id)
    session.begin_preflight()
    session.start_recording()
    return session


class TestTransitions:
    def test_happy_path(self):
        log: list[str] = []
        session = RecordingSession("drive01")
        assert session.state is SessionState.IDLE

        session.begin_preflight()
        assert session.state is SessionState.PREFLIGHT

        session.start_recording()
        assert session.state is SessionState.RECORDING

        session.request_stop(StopRequest(StopReason.SERVICE, completed=True))
        result = session.finalize(make_finalizers(log))

        assert session.state is SessionState.DONE
        assert result.success

    def test_start_recording_requires_preflight(self):
        session = RecordingSession("drive01")
        with pytest.raises(SessionStateError):
            session.start_recording()

    def test_preflight_twice_is_error(self):
        session = RecordingSession("drive01")
        session.begin_preflight()
        with pytest.raises(SessionStateError):
            session.begin_preflight()

    def test_finalize_requires_recording(self):
        session = RecordingSession("drive01")
        session.begin_preflight()
        with pytest.raises(SessionStateError):
            session.finalize(make_finalizers([]))

    def test_finalize_twice_is_error(self):
        session = running_session()
        session.request_stop(StopRequest(StopReason.SIGNAL))
        session.finalize(make_finalizers([]))
        with pytest.raises(SessionStateError):
            session.finalize(make_finalizers([]))

    def test_abort_from_preflight(self):
        session = RecordingSession("drive01")
        session.begin_preflight()
        session.abort(StopRequest(StopReason.ERROR, "preflight failed"))
        assert session.state is SessionState.DONE
        assert session.stop_request.reason is StopReason.ERROR

    def test_abort_requires_preflight(self):
        session = running_session()
        with pytest.raises(SessionStateError):
            session.abort()


class TestStopRequest:
    def test_idle_stop_is_error(self):
        session = RecordingSession("drive01")
        with pytest.raises(SessionStateError):
            session.request_stop(StopRequest(StopReason.SIGNAL))

    def test_first_request_wins(self):
        session = running_session()
        assert session.request_stop(StopRequest(StopReason.SIGNAL, "ctrl-c")) is True
        # 二度目は無視されるが例外にはしない (冪等)
        assert session.request_stop(
            StopRequest(StopReason.SERVICE, "stop svc", completed=True)) is False
        assert session.stop_request.reason is StopReason.SIGNAL
        assert session.stop_request.detail == "ctrl-c"
        assert session.stop_request.completed is False

    def test_stop_during_finalizing_is_ignored(self):
        # finalizing 中にサービス呼び出しが届く競合を模す
        session = running_session()
        session.request_stop(StopRequest(StopReason.SIGNAL))
        log: list[str] = []

        class LateStopper(RecordingWriter):
            def _run(self):  # type: ignore[override]
                assert session.state is SessionState.FINALIZING
                assert session.request_stop(StopRequest(StopReason.SERVICE)) is False
                return super()._run()
            close = _run

        finalizers = make_finalizers(log)
        finalizers.bag = LateStopper(log, FinalizeStep.BAG_CLOSE.value, Path("/d/bags"))
        result = session.finalize(finalizers)
        assert result.success
        assert session.stop_request.reason is StopReason.SIGNAL

    def test_stop_after_done_is_ignored(self):
        session = running_session()
        session.request_stop(StopRequest(StopReason.SIGNAL))
        session.finalize(make_finalizers([]))
        assert session.request_stop(StopRequest(StopReason.SERVICE)) is False


class TestFinalizeOrder:
    def test_steps_run_in_defined_order(self):
        log: list[str] = []
        session = running_session()
        session.request_stop(StopRequest(StopReason.SERVICE, completed=True))
        result = session.finalize(make_finalizers(log))

        assert log == [step.value for step in FINALIZE_ORDER]
        assert result.completed_steps == list(FINALIZE_ORDER)
        assert result.manifest_written

    def test_artifacts_collected(self):
        session = running_session()
        session.request_stop(StopRequest(StopReason.SIGNAL))
        result = session.finalize(make_finalizers([]))
        # catalog はパスを返さないので成果物は 4 件
        assert result.artifacts == [
            Path("/d/bags"),
            Path("/d/health/topic_stats.json"),
            Path("/d/tags/events.jsonl"),
            Path("/d/manifest.yaml"),
        ]


class TestFinalizeFailure:
    def test_failure_keeps_earlier_artifacts_and_records_step(self):
        log: list[str] = []
        session = running_session()
        session.request_stop(StopRequest(StopReason.SIGNAL))
        result = session.finalize(make_finalizers(log, failing=FinalizeStep.EVENTS))

        assert not result.success
        assert result.failed_step is FinalizeStep.EVENTS
        assert "events boom" in result.error
        # 失敗した手順の手前までは実行済みで、成果物も残る
        assert result.completed_steps == [
            FinalizeStep.BAG_CLOSE, FinalizeStep.TOPIC_STATS]
        assert Path("/d/bags") in result.artifacts
        # 以降の手順は実行されない
        assert log == ["bag_close", "topic_stats", "events"]
        assert session.state is SessionState.DONE

    def test_manifest_not_written_when_earlier_step_fails(self):
        log: list[str] = []
        session = running_session()
        session.request_stop(StopRequest(StopReason.SIGNAL))
        result = session.finalize(make_finalizers(log, failing=FinalizeStep.BAG_CLOSE))

        assert not result.manifest_written
        assert "manifest" not in log
        assert Path("/d/manifest.yaml") not in result.artifacts

    def test_manifest_failure_leaves_bag_and_stats(self):
        session = running_session()
        session.request_stop(StopRequest(StopReason.SIGNAL))
        result = session.finalize(make_finalizers([], failing=FinalizeStep.MANIFEST))

        assert not result.manifest_written
        assert FinalizeStep.TOPIC_STATS in result.completed_steps
        assert remaining_steps(result.completed_steps) == [
            FinalizeStep.MANIFEST, FinalizeStep.CATALOG]


class TestDurationAndTags:
    def test_duration_uses_injected_clock(self):
        ticks = iter([100.0, 130.5])
        session = RecordingSession("drive01", clock=lambda: next(ticks))
        session.begin_preflight()
        session.start_recording()          # 100.0
        session.request_stop(StopRequest(StopReason.SIGNAL))  # 130.5
        assert session.duration_sec == pytest.approx(30.5)

    def test_stop_tags(self):
        session = running_session()
        session.request_stop(
            StopRequest(StopReason.SERVICE, "route finished", completed=True))
        tags = session.stop_tags()
        assert tags == {
            "stop_reason": "service",
            "completed": "true",
            "stop_detail": "route finished",
        }

    def test_stop_tags_empty_without_request(self):
        assert running_session().stop_tags() == {}

    def test_stop_tag_keys_follow_core_convention(self):
        # DriveManifest.tags のキーは小文字 snake_case でなければならない
        import re
        session = running_session()
        session.request_stop(StopRequest(StopReason.DISK_FULL, "below 10GB"))
        pattern = re.compile(r"^[a-z][a-z0-9_]*$")
        assert all(pattern.match(key) for key in session.stop_tags())
