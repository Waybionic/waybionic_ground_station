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
        time.sleep(3)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
