import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.time import Time
import tf2_ros


@pytest.mark.launch_test
def generate_test_description():
    bringup_dir = get_package_share_directory('waybionic_bringup')
    launch_file = os.path.join(bringup_dir, 'launch', 'ground_station.launch.py')

    ground_station_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'launch_rviz': 'false',
            'use_joint_state_publisher_gui': 'false',
            'start_temporary_diagnostics_publisher': 'true',
            'follow_camera': 'false'
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
        time.sleep(3)

    def test_fixed_view_keeps_the_camera_target(self):
        rclpy.init()
        node = rclpy.create_node('fixed_view_test')
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, node)
        try:
            deadline = time.monotonic() + 10.0
            while (not buffer.can_transform('base_link', 'view_focus', Time())
                   and time.monotonic() < deadline):
                rclpy.spin_once(node, timeout_sec=0.1)
            focus = buffer.lookup_transform('base_link', 'view_focus', Time())
            offset = focus.transform.translation
            for actual, expected in zip((offset.x, offset.y, offset.z), (0.0, 0.0, 0.35)):
                self.assertAlmostEqual(actual, expected, places=6)
        finally:
            listener.unregister()
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
