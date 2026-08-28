"""
Benchtop IMU walkthrough: publisher plus RViz, with demo outputs enabled.

This launch deliberately turns on the synthetic orientation and the rotating TF
so there is something to look at in RViz. Both are visualisation aids and are
off by default in imu_publisher.launch.py. The RViz config loads
``rviz_imu_plugin/Imu`` on ``/waybionic/imu/data_demo``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the launch description for the IMU RViz demo."""
    use_mock = LaunchConfiguration('use_mock')
    topic = LaunchConfiguration('topic')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    mock_stall_after_sec = LaunchConfiguration('mock_stall_after_sec')
    launch_rviz = LaunchConfiguration('launch_rviz')

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('waybionic_sensors'),
                'launch',
                'imu_publisher.launch.py',
            ])
        ),
        launch_arguments={
            'use_mock': use_mock,
            'topic': topic,
            'publish_rate_hz': publish_rate_hz,
            'mock_stall_after_sec': mock_stall_after_sec,
            'publish_demo_orientation': 'true',
            'publish_demo_tf': 'true',
        }.items(),
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('waybionic_sensors'),
        'config',
        'imu_demo.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock',
            default_value='true',
            choices=['true', 'false'],
            description='Generate mock samples instead of reading hardware.',
        ),
        DeclareLaunchArgument(
            'topic',
            default_value='/waybionic/imu/data_raw',
            description='sensor_msgs/msg/Imu topic for raw data.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='50.0',
            description='Sample publish rate in Hz.',
        ),
        DeclareLaunchArgument(
            'mock_stall_after_sec',
            default_value='0.0',
            description='Stop the mock after this many seconds to show the stale heartbeat.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Set false for a headless run in CI or over a plain shell.',
        ),
        imu_launch,
        Node(
            package='rviz2',
            executable='rviz2',
            name='waybionic_imu_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(launch_rviz),
        ),
    ])
