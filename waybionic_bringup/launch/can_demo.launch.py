# Copyright 2026 Waybionic
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_interface_arg = DeclareLaunchArgument(
        'can_interface', default_value='vcan0',
        description='SocketCAN interface; falls back to udp_multicast if unavailable')

    simulate_faults_arg = DeclareLaunchArgument(
        'simulate_faults', default_value='true',
        description='Inject the provisional joint 4 / joint 6 fault scenarios')

    start_mock_drives_arg = DeclareLaunchArgument(
        'start_mock_drives', default_value='true',
        description='Set false when driving real hardware on the bus')

    can_host_node = Node(
        package='waybionic_control',
        executable='can_host',
        name='can_host',
        output='screen',
        parameters=[{'can_interface': LaunchConfiguration('can_interface')}]
    )

    mock_drives_node = Node(
        package='waybionic_control',
        executable='mock_drives',
        name='mock_drives',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_mock_drives')),
        parameters=[
            {'can_interface': LaunchConfiguration('can_interface')},
            {'simulate_faults': LaunchConfiguration('simulate_faults')}
        ]
    )

    return LaunchDescription([
        can_interface_arg,
        simulate_faults_arg,
        start_mock_drives_arg,
        can_host_node,
        mock_drives_node,
    ])
