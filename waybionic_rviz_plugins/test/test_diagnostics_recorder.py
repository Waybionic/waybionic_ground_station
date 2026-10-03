import importlib.util
import json
import signal
import subprocess
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "diagnostics_recorder.py"
SPEC = importlib.util.spec_from_file_location("diagnostics_recorder", SCRIPT)
recorder = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(recorder)


def test_default_topic_and_explicit_topic_selection():
    options = recorder.parse_args(
        ["--output-directory", "/tmp/session", "--source-label", "mock"]
    )
    assert options.topics == ["/diagnostics"]

    options = recorder.parse_args(
        [
            "--topic",
            "/diagnostics",
            "--topic",
            "/imu",
            "--output-directory",
            "/tmp/session",
            "--source-label",
            "live",
        ]
    )
    assert options.topics == ["/diagnostics", "/imu"]


def test_invalid_duration_is_rejected():
    with pytest.raises(SystemExit):
        recorder.parse_args(
            [
                "--duration",
                "0",
                "--output-directory",
                "/tmp/session",
                "--source-label",
                "mock",
            ]
        )


def test_existing_output_directory_is_not_overwritten(tmp_path, capsys):
    output = tmp_path / "session"
    output.mkdir()
    marker = output / "keep.txt"
    marker.write_text("keep", encoding="utf-8")

    result = recorder.run(
        ["--output-directory", str(output), "--source-label", "mock"]
    )

    assert result == 2
    assert marker.read_text(encoding="utf-8") == "keep"
    assert "already exists" in capsys.readouterr().err


def test_recorder_failure_is_reported(tmp_path, monkeypatch, capsys):
    class FailedProcess:
        returncode = 7

        def poll(self):
            return self.returncode

        def wait(self, **_kwargs):
            return self.returncode

    monkeypatch.setattr(recorder.subprocess, "Popen", lambda _command: FailedProcess())

    result = recorder.run(
        ["--output-directory", str(tmp_path / "session"), "--source-label", "live"]
    )

    assert result == 1
    assert "failed with exit code 7" in capsys.readouterr().err


def test_timed_shutdown_sends_sigint_and_writes_metadata(tmp_path, monkeypatch):
    class TimedProcess:
        returncode = None
        stopped = False

        def poll(self):
            return self.returncode

        def wait(self, **kwargs):
            if "timeout" in kwargs and not self.stopped:
                raise subprocess.TimeoutExpired("ros2", kwargs["timeout"])
            self.returncode = 0
            return 0

        def send_signal(self, value):
            assert value == signal.SIGINT
            self.stopped = True

        def terminate(self):
            raise AssertionError("terminate should not be needed")

    process = TimedProcess()
    monkeypatch.setattr(recorder.subprocess, "Popen", lambda _command: process)
    monkeypatch.setattr(recorder, "count_recorded_messages", lambda _bag: 3)

    output = tmp_path / "session"
    result = recorder.run(
        [
            "--duration",
            "1",
            "--output-directory",
            str(output),
            "--source-label",
            "mock",
            "--tested-commit",
            "abc123",
        ]
    )

    assert result == 0
    metadata = json.loads((output / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["topic"] == "/diagnostics"
    assert metadata["source_label"] == "mock"
    assert metadata["tested_commit"] == "abc123"
    assert metadata["message_count"] == 3
