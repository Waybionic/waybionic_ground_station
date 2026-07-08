from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock = LaunchConfiguration('use_mock')
    topic = LaunchConfiguration('topic')

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
            description='Use mock IMU data instead of live hardware input.',
        ),
        DeclareLaunchArgument(
            'topic',
            default_value='/waybionic/imu/data_raw',
            description='sensor_msgs/msg/Imu topic to publish.',
        ),
        imu_launch,
        Node(
            package='rviz2',
            executable='rviz2',
            name='waybionic_imu_rviz',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
