from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='normal',
            choices=['normal', 'fault', 'stale', 'cycle'],
            description=(
                'Diagnostics profile to publish: normal, fault, stale, or cycle '
                '(rotate through normal/fault/stale every 5 seconds).'
            ),
        ),
        DeclareLaunchArgument(
            'topic',
            default_value='/diagnostics',
            description='diagnostic_msgs/msg/DiagnosticArray topic to publish.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='2.0',
            description='Publish rate in Hz.',
        ),
        Node(
            package='waybionic_rviz_plugins',
            executable='temporary_diagnostics_publisher.py',
            name='temporary_diagnostics_publisher',
            output='screen',
            parameters=[{
                'mode': LaunchConfiguration('mode'),
                'topic': LaunchConfiguration('topic'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            }],
        ),
    ])
