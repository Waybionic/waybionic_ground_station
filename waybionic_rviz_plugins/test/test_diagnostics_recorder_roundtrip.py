import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
rosbag2_py = pytest.importorskip("rosbag2_py")
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.serialization import deserialize_message


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RECORDER = PACKAGE_ROOT / "scripts" / "diagnostics_recorder.py"


def test_real_rosbag_round_trip_preserves_diagnostics(tmp_path):
    domain_id = str(100 + os.getpid() % 100)
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = domain_id

    previous_domain = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = domain_id
    rclpy.init(args=None)
    node = rclpy.create_node("recorder_roundtrip_publisher")
    publisher = node.create_publisher(DiagnosticArray, "/diagnostics", 10)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    timestamp = node.get_clock().now().to_msg()
    message = DiagnosticArray()
    message.header.stamp = timestamp
    status = DiagnosticStatus()
    status.name = "board.temperature"
    status.level = DiagnosticStatus.ERROR
    status.message = "High temperature detected"
    status.values = [
        KeyValue(key="value", value="82.5"),
        KeyValue(key="unit", value="C"),
    ]
    message.status = [status]
    timer = node.create_timer(0.1, lambda: publisher.publish(message))

    try:
        output = tmp_path / "roundtrip"
        process = subprocess.Popen(
            [
                sys.executable,
                str(RECORDER),
                "--duration",
                "5",
                "--output-directory",
                str(output),
                "--source-label",
                "mock",
            ],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        stdout, stderr = process.communicate(timeout=10)
        assert process.returncode == 0, stderr or stdout
    finally:
        timer.cancel()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2)
        if previous_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_domain

    metadata = json.loads((output / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["topic"] == "/diagnostics"
    assert metadata["source_label"] == "mock"

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(output / "bag"), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    recorded = []
    while reader.has_next():
        _, serialized, recorded_timestamp = reader.read_next()
        recorded.append(
            (deserialize_message(serialized, DiagnosticArray), recorded_timestamp)
        )

    assert recorded
    recorded_message, bag_timestamp = recorded[0]
    recorded_status = recorded_message.status[0]
    assert recorded_status.name == "board.temperature"
    assert recorded_status.level == DiagnosticStatus.ERROR
    assert recorded_status.message == "High temperature detected"
    assert [(item.key, item.value) for item in recorded_status.values] == [
        ("value", "82.5"),
        ("unit", "C"),
    ]
    assert recorded_message.header.stamp == timestamp
    assert bag_timestamp > 0
