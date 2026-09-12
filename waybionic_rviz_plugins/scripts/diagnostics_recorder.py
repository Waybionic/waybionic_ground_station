#!/usr/bin/env python3
"""Record an explicit ROS 2 diagnostics session into a rosbag2 directory."""

import argparse
import json
import signal
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Sequence


DEFAULT_TOPIC = "/diagnostics"


def parse_args(arguments: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record selected ROS 2 topics and write session metadata."
    )
    parser.add_argument(
        "--topic",
        action="append",
        dest="topics",
        metavar="TOPIC",
        help=(
            "Topic to record; repeat for additional topics. Defaults to "
            f"{DEFAULT_TOPIC}. Other topics must be named explicitly."
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        help="Stop after this many seconds; omit to record until Ctrl+C.",
    )
    parser.add_argument(
        "--output-directory",
        required=True,
        type=Path,
        help="New session directory to create; an existing directory is rejected.",
    )
    parser.add_argument(
        "--source-label",
        required=True,
        choices=("mock", "live"),
        help="Label describing whether the session source is mock or live.",
    )
    parser.add_argument(
        "--tested-commit",
        help="Optional commit identifier tested during this session.",
    )
    parsed = parser.parse_args(arguments)
    if parsed.duration is not None and parsed.duration <= 0:
        parser.error("--duration must be greater than zero")
    parsed.topics = parsed.topics or [DEFAULT_TOPIC]
    return parsed


def build_record_command(bag_directory: Path, topics: Sequence[str]) -> list[str]:
    if not topics:
        raise ValueError("at least one topic must be selected")
    return [
        "ros2",
        "bag",
        "record",
        "--disable-keyboard-controls",
        "--output",
        str(bag_directory),
        "--topics",
        *topics,
    ]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_metadata(
    session_directory: Path,
    topics: Sequence[str],
    source_label: str,
    start_time: str,
    end_time: str,
    tested_commit: Optional[str],
    message_count: int,
) -> None:
    metadata = {
        "topic": topics[0] if len(topics) == 1 else list(topics),
        "start_time": start_time,
        "end_time": end_time,
        "source_label": source_label,
        "message_count": message_count,
    }
    if tested_commit:
        metadata["tested_commit"] = tested_commit
    (session_directory / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )


def count_recorded_messages(bag_directory: Path) -> int:
    if not bag_directory.exists():
        return 0
    try:
        import rosbag2_py
    except ImportError as exc:
        raise RuntimeError(
            "rosbag2_py is required to finalize and validate the recording"
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    count = 0
    while reader.has_next():
        reader.read_next()
        count += 1
    return count


def stop_recorder(process: subprocess.Popen[bytes]) -> None:
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def run(arguments: Optional[Sequence[str]] = None) -> int:
    options = parse_args(arguments)
    session_directory = options.output_directory
    bag_directory = session_directory / "bag"

    if session_directory.exists():
        print(f"error: output directory already exists: {session_directory}", file=sys.stderr)
        return 2
    try:
        session_directory.mkdir(parents=True)
    except OSError as exc:
        print(f"error: cannot create output directory: {exc}", file=sys.stderr)
        return 2

    start_time = utc_now()
    command = build_record_command(bag_directory, options.topics)
    try:
        process = subprocess.Popen(command)
    except FileNotFoundError:
        print(
            "error: missing dependency: the 'ros2' command is not available; "
            "source the ROS 2 environment first",
            file=sys.stderr,
        )
        return 1
    except (OSError, subprocess.SubprocessError) as exc:
        print(f"error: could not start ros2 bag record: {exc}", file=sys.stderr)
        return 1

    try:
        if options.duration is None:
            process.wait()
        else:
            process.wait(timeout=options.duration)
    except subprocess.TimeoutExpired:
        print("Recording duration reached; finalizing bag.")
    except KeyboardInterrupt:
        print("Stopping recording; finalizing bag.")
    finally:
        stop_recorder(process)

    end_time = utc_now()
    if process.returncode not in (0, 130, -signal.SIGINT):
        print(
            f"error: ros2 bag record failed with exit code {process.returncode}",
            file=sys.stderr,
        )
        return 1

    try:
        message_count = count_recorded_messages(bag_directory)
        write_metadata(
            session_directory,
            options.topics,
            options.source_label,
            start_time,
            end_time,
            options.tested_commit,
            message_count,
        )
    except Exception as exc:
        print(f"error: could not finalize recording: {exc}", file=sys.stderr)
        return 1

    if message_count == 0:
        print("error: recording completed but contained no messages", file=sys.stderr)
        return 1

    print(f"Recorded {message_count} messages in {session_directory}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())