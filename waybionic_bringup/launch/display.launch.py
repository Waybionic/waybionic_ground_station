import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    waybionic_desc_dir = get_package_share_directory('waybionic_description')
    waybionic_bringup_dir = get_package_share_directory('waybionic_bringup')

    # Define default paths
    default_model_path = os.path.join(waybionic_desc_dir, 'urdf', 'waybionic_placeholder.urdf')
    default_rviz_config_path = os.path.join(waybionic_bringup_dir, 'rviz', 'waybionic.rviz')

    # 1. Declare Launch Arguments (Task 4)
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Absolute path to robot urdf or xacro file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )

    # 2. Nodes
    # Use 'xacro' command substitution to read the URDF/Xacro file dynamically
    robot_description_content = {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_content]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Pass the RViz configuration argument to the RViz node (Task 2)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
