import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import diagnostic_msgs.msg
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
def generate_test_description():
    bringup_dir = get_package_share_directory('waybionic_bringup')
    launch_file = os.path.join(bringup_dir, 'launch', 'ground_station.launch.py')

    ground_station_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'launch_rviz': 'false',
            'use_joint_state_publisher_gui': 'false',
            'start_temporary_diagnostics_publisher': 'true'
        }.items()
    )

    return launch.LaunchDescription([
        ground_station_launch,
        launch_testing.actions.ReadyToTest()
    ])


class TestGroundStationLaunch(unittest.TestCase):

    def test_nodes_started(self, proc_info, proc_output):
        proc_info.assertWaitForStartup(process=None, timeout=5)
        assert len(proc_info.processes()) > 0, 'No processes were started!'

    def test_real_diagnostics_contract(self, proc_info):
        rclpy.init()
        node = rclpy.create_node('ground_station_launch_test')
        received = []
        node.create_subscription(
            diagnostic_msgs.msg.DiagnosticArray,
            '/diagnostics',
            received.append,
            10,
        )

        deadline = time.monotonic() + 10.0
        try:
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.2)
        finally:
            node.destroy_node()
            rclpy.shutdown()

        assert received, 'No DiagnosticArray received within 10 seconds'
        statuses = {status.name: status for status in received[-1].status}
        expected_signals = {
            'board.temperature': (diagnostic_msgs.msg.DiagnosticStatus.OK, 'C'),
            'motor.current': (diagnostic_msgs.msg.DiagnosticStatus.OK, 'A'),
            'imu.roll': (diagnostic_msgs.msg.DiagnosticStatus.OK, 'deg'),
            'imu.pitch': (diagnostic_msgs.msg.DiagnosticStatus.OK, 'deg'),
            'imu.yaw': (diagnostic_msgs.msg.DiagnosticStatus.OK, 'deg'),
        }

        assert set(statuses) == set(expected_signals)
        for name, (expected_level, expected_unit) in expected_signals.items():
            status = statuses[name]
            assert status.level == expected_level
            values = {item.key: item.value for item in status.values}
            assert values['value']
            assert values['unit'] == expected_unit


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
