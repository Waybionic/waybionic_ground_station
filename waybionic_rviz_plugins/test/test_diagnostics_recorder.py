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


def test_missing_ros2_dependency_is_reported(tmp_path, monkeypatch, capsys):
    def missing_ros2(_command):
        raise FileNotFoundError("ros2")

    monkeypatch.setattr(recorder.subprocess, "Popen", missing_ros2)

    result = recorder.run(
        ["--output-directory", str(tmp_path / "session"), "--source-label", "mock"]
    )

    assert result == 1
    assert "missing dependency" in capsys.readouterr().err


def test_unwritable_output_is_reported(tmp_path, monkeypatch, capsys):
    def mkdir_failure(_path, **_kwargs):
        raise PermissionError("permission denied")

    monkeypatch.setattr(Path, "mkdir", mkdir_failure)

    result = recorder.run(
        ["--output-directory", str(tmp_path / "session"), "--source-label", "mock"]
    )

    assert result == 2
    assert "cannot create output directory" in capsys.readouterr().err


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


def test_missing_rosbag_dependency_is_reported(tmp_path, monkeypatch, capsys):
    class SuccessfulProcess:
        returncode = 0

        def poll(self):
            return self.returncode

        def wait(self, **_kwargs):
            return self.returncode

    monkeypatch.setattr(recorder.subprocess, "Popen", lambda _command: SuccessfulProcess())
    monkeypatch.setattr(
        recorder,
        "count_recorded_messages",
        lambda _bag: (_ for _ in ()).throw(
            RuntimeError("rosbag2_py is required to finalize and validate the recording")
        ),
    )

    result = recorder.run(
        ["--output-directory", str(tmp_path / "session"), "--source-label", "mock"]
    )

    assert result == 1
    assert "could not finalize recording" in capsys.readouterr().err


def test_empty_recording_is_rejected(tmp_path, monkeypatch, capsys):
    class SuccessfulProcess:
        returncode = 0

        def poll(self):
            return self.returncode

        def wait(self, **_kwargs):
            return self.returncode

    monkeypatch.setattr(recorder.subprocess, "Popen", lambda _command: SuccessfulProcess())
    monkeypatch.setattr(recorder, "count_recorded_messages", lambda _bag: 0)

    result = recorder.run(
        ["--output-directory", str(tmp_path / "session"), "--source-label", "mock"]
    )

    assert result == 1
    assert "contained no messages" in capsys.readouterr().err


def test_keyboard_interrupt_stops_child_and_writes_metadata(tmp_path, monkeypatch):
    class InterruptedProcess:
        returncode = None
        stopped = False

        def poll(self):
            return self.returncode

        def wait(self, **_kwargs):
            if not self.stopped:
                raise KeyboardInterrupt
            self.returncode = 130
            return self.returncode

        def send_signal(self, value):
            assert value == signal.SIGINT
            self.stopped = True

    process = InterruptedProcess()
    monkeypatch.setattr(recorder.subprocess, "Popen", lambda _command: process)
    monkeypatch.setattr(recorder, "count_recorded_messages", lambda _bag: 2)

    output = tmp_path / "session"
    result = recorder.run(
        ["--output-directory", str(output), "--source-label", "mock"]
    )

    assert result == 0
    assert json.loads((output / "metadata.json").read_text(encoding="utf-8"))["message_count"] == 2


def test_stop_recorder_escalates_when_child_does_not_stop():
    class StuckProcess:
        returncode = None
        signals = []
        actions = []
        wait_calls = 0

        def poll(self):
            return self.returncode

        def send_signal(self, value):
            self.signals.append(value)

        def wait(self, **kwargs):
            self.actions.append(("wait", kwargs.get("timeout")))
            self.wait_calls += 1
            if self.wait_calls < 3:
                raise subprocess.TimeoutExpired("ros2", kwargs["timeout"])
            self.returncode = -signal.SIGKILL

        def terminate(self):
            self.actions.append(("terminate",))

        def kill(self):
            self.actions.append(("kill",))

    process = StuckProcess()
    recorder.stop_recorder(process)

    assert process.signals == [signal.SIGINT]
    assert process.actions == [
        ("wait", 10),
        ("terminate",),
        ("wait", 5),
        ("kill",),
        ("wait", None),
    ]


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
