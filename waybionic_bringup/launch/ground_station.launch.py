import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    waybionic_desc_dir = get_package_share_directory('waybionic_description')
    waybionic_bringup_dir = get_package_share_directory('waybionic_bringup')

    default_model_path = os.path.join(waybionic_desc_dir, 'urdf', 'waybionic_placeholder.urdf')
    default_rviz_config_path = os.path.join(waybionic_bringup_dir, 'rviz', 'waybionic_unified.rviz')

    model_arg = DeclareLaunchArgument('model', default_value=default_model_path, description='Absolute path to robot urdf')
    use_mock_diag_arg = DeclareLaunchArgument('use_mock_diagnostics', default_value='true', description='Use mock data for diagnostics')
    diag_topic_arg = DeclareLaunchArgument('diagnostics_topic', default_value='/diagnostics', description='Diagnostics topic name')
    start_temp_pub_arg = DeclareLaunchArgument('start_temporary_diagnostics_publisher', default_value='true', description='Start temporary diagnostics publisher')
    use_jsp_gui_arg = DeclareLaunchArgument('use_joint_state_publisher_gui', default_value='true', description='Launch joint state publisher GUI')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='true', description='Launch RViz (set false for headless validation)')
    rviz_config_arg = DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file')

    robot_description_content = {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_content, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_joint_state_publisher_gui'))
    )

    temp_diag_pub_node = Node(
        package='waybionic_rviz_plugins',
        executable='temporary_diagnostics_publisher',
        name='temp_diag_pub',
        condition=IfCondition(LaunchConfiguration('start_temporary_diagnostics_publisher')),
        parameters=[
            {'use_mock_diagnostics': LaunchConfiguration('use_mock_diagnostics')},
            {'diagnostics_topic': LaunchConfiguration('diagnostics_topic')}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        model_arg, use_mock_diag_arg, diag_topic_arg, start_temp_pub_arg, 
        use_jsp_gui_arg, use_sim_time_arg, launch_rviz_arg, rviz_config_arg,
        rsp_node, jsp_gui_node, temp_diag_pub_node, rviz_node
    ])
