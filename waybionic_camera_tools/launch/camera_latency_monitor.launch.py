from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Creates a configurable launch description for the monitor node."""

    default_config = get_package_share_directory('waybionic_camera_tools') + '/camera_latency.yaml'

    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value = default_config),

          # these can possibly be split between left and right?
            DeclareLaunchArgument('image_topic', default_value = '/doctor_view/image_raw'),
          
            DeclareLaunchArgument('camera_info_topic', default_value = '/doctor_view/camera_info'),
          
            DeclareLaunchArgument('diagnostics_topic', default_value = '/diagnostics'),
          
            DeclareLaunchArgument('camera_signal_name', default_value = 'camera.primary'),
          
            DeclareLaunchArgument('expected_frame_rate', default_value = '30.0'),
          
            DeclareLaunchArgument('expected_width', default_value = '920'),
          
            DeclareLaunchArgument('expected_height', default_value = '1080'),
          
            DeclareLaunchArgument(
                'optical_frame_id',
                default_value = 'doctor_view_optical_frame',
            ),
            Node(
                package='waybionic_camera_tools',
                executable='image_latency_monitor',
                name='image_latency_monitor',
                output='screen',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {
                        'image_topic': LaunchConfiguration('image_topic'),
                        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                        'diagnostics_topic': LaunchConfiguration('diagnostics_topic'),
                        'camera_signal_name': LaunchConfiguration('camera_signal_name'),
                        'expected_frame_rate': ParameterValue(
                            LaunchConfiguration('expected_frame_rate'),
                            value_type=float,
                        ),
                        'expected_width': ParameterValue(
                            LaunchConfiguration('expected_width'),
                            value_type=int,
                        ),
                        'expected_height': ParameterValue(
                            LaunchConfiguration('expected_height'),
                            value_type=int,
                        ),
                        'optical_frame_id': LaunchConfiguration('optical_frame_id'),
                    },
                ],
            ),
        ]
    )
