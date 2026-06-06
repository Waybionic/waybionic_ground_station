import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your URDF file inside the description package
    urdf_path = os.path.join(
        get_package_share_directory('waybionic_description'),
        'urdf',
        'waybionic.urdf'
    )

    # Read the URDF content
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Node 1: Robot State Publisher (Publishes 3D transforms)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    # Node 2: Joint State Publisher GUI (Creates sliders to rotate the joint)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Node 3: RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
