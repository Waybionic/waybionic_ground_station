from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_mock = LaunchConfiguration('use_mock')
    topic = LaunchConfiguration('topic')
    frame_id = LaunchConfiguration('frame_id')
    parent_frame_id = LaunchConfiguration('parent_frame_id')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    serial_port = LaunchConfiguration('serial_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock',
            default_value='true',
            choices=['true', 'false'],
            description='Use mock IMU data instead of live hardware input.',
        ),
        DeclareLaunchArgument(
            'topic',
            default_value='/waybionic/imu/data_raw',
            description='sensor_msgs/msg/Imu topic to publish.',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='IMU frame_id in published messages and TF.',
        ),
        DeclareLaunchArgument(
            'parent_frame_id',
            default_value='base_link',
            description='Parent frame for the static IMU TF offset.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='50.0',
            description='Publish rate in Hz.',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='',
            description='Serial device path for live IMU input when use_mock is false.',
        ),
        Node(
            package='waybionic_sensors',
            executable='imu_publisher',
            name='waybionic_imu_publisher',
            output='screen',
            parameters=[{
                'use_mock': ParameterValue(use_mock, value_type=bool),
                'topic': ParameterValue(topic, value_type=str),
                'frame_id': ParameterValue(frame_id, value_type=str),
                'parent_frame_id': ParameterValue(parent_frame_id, value_type=str),
                'publish_rate_hz': ParameterValue(publish_rate_hz, value_type=float),
                'serial_port': ParameterValue(serial_port, value_type=str),
            }],
        ),
    ])
