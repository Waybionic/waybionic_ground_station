from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'bind_address', default_value='127.0.0.1',
            description='Address camera_receiver listens on (0.0.0.0 inside Docker)'),
        DeclareLaunchArgument(
            'port', default_value='47310', description='TCP port for camera_bridge'),
        DeclareLaunchArgument(
            'log_csv', default_value='',
            description='Log each frame: capture time, arrival time and delay (CSV path)'),
        DeclareLaunchArgument(
            'diagnostics_topic', default_value='/diagnostics',
            description='Diagnostics topic for the frame rate and delay'),
        Node(
            package='waybionic_camera', executable='camera_receiver', name='camera_receiver',
            output='screen',
            parameters=[{
                'bind_address': LaunchConfiguration('bind_address'),
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            }]),
        Node(
            package='waybionic_camera', executable='latency_monitor',
            name='camera_latency_monitor',
            parameters=[{
                'log_csv': ParameterValue(LaunchConfiguration('log_csv'), value_type=str),
                'diagnostics_topic': LaunchConfiguration('diagnostics_topic'),
            }]),
    ])
