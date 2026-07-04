import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Renders ALL meshes in the package (test_all_meshes.urdf), each as its own link
    urdf_path = os.path.join(
        get_package_share_directory('waybionic_description'),
        'urdf', 'test_all_meshes.urdf'
    )
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    rviz_config_path = os.path.join(
        get_package_share_directory('waybionic_bringup'),
        'rviz', 'waybionic.rviz'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])
