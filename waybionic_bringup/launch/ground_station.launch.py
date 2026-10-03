import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, Command, LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_files_exist(context, *args, **kwargs):
    model_path = LaunchConfiguration('model').perform(context)
    rviz_path = LaunchConfiguration('rvizconfig').perform(context)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f'Model file not found: {model_path}')
    if not os.path.exists(rviz_path):
        raise FileNotFoundError(f'RViz config file not found: {rviz_path}')
    return []


def generate_launch_description():
    waybionic_desc_dir = get_package_share_directory('waybionic_description')
    waybionic_bringup_dir = get_package_share_directory('waybionic_bringup')

    default_model_path = os.path.join(
        waybionic_desc_dir, 'urdf', 'waybionic_arm.urdf')
    default_rviz_config_path = os.path.join(
        waybionic_bringup_dir, 'rviz', 'waybionic_unified.rviz')

    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='Absolute path to robot urdf')

    use_mock_diag_arg = DeclareLaunchArgument(
        'use_mock_diagnostics', default_value='true',
        description='Panel mode: true for internal mock, false for live topics')

    diag_topic_arg = DeclareLaunchArgument(
        'diagnostics_topic', default_value='/diagnostics',
        description='Diagnostics topic name')

    start_temp_pub_arg = DeclareLaunchArgument(
        'start_temporary_diagnostics_publisher', default_value='false',
        description='Start the optional temporary publisher for live demo')

    use_jsp_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui', default_value='true',
        description='Launch joint state publisher GUI')

    demo_mode_arg = DeclareLaunchArgument(
        'demo_mode', default_value='false',
        description='Sweep each joint in turn and check its TF (simulated joint states only)')

    demo_speed_arg = DeclareLaunchArgument(
        'demo_speed', default_value='30.0',
        description='Joint demo sweep speed in degrees per second')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time')

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true',
        description='Launch RViz (set false for headless validation)')

    rviz_config_arg = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')

    file_check = OpaqueFunction(function=check_files_exist)

    robot_description_content = {
        'robot_description': ParameterValue(
            Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    }

    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[
            robot_description_content,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    demo_mode = LaunchConfiguration('demo_mode')
    # Demo mode publishes joint states itself.
    simulated_joints = demo_mode

    jsp_gui_node = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        condition=IfCondition(AndSubstitution(
            LaunchConfiguration('use_joint_state_publisher_gui'),
            NotSubstitution(simulated_joints)))
    )

    demo_speed = ParameterValue(LaunchConfiguration('demo_speed'), value_type=float)
    joint_demo_node = Node(
        package='waybionic_bringup', executable='joint_demo.py', name='joint_demo',
        output='screen',
        condition=IfCondition(demo_mode),
        parameters=[
            {'speed_deg_s': demo_speed},
            {'diagnostics_topic': LaunchConfiguration('diagnostics_topic')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    temp_diag_pub_node = Node(
        package='waybionic_rviz_plugins', executable='temporary_diagnostics_publisher.py',
        name='temp_diag_pub',
        condition=IfCondition(LaunchConfiguration('start_temporary_diagnostics_publisher')),
        parameters=[
            {'mode': 'normal'},
            {'topic': LaunchConfiguration('diagnostics_topic')},
            {'publish_rate_hz': 10.0}
        ]
    )

    # Demo mode reports on /diagnostics, so the panel listens to live diagnostics.
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'use_mock_diagnostics': AndSubstitution(
                LaunchConfiguration('use_mock_diagnostics'), NotSubstitution(simulated_joints))},
            {'diagnostics_topic': LaunchConfiguration('diagnostics_topic')}
        ]
    )

    return LaunchDescription([
        model_arg, use_mock_diag_arg, diag_topic_arg, start_temp_pub_arg,
        use_jsp_gui_arg, demo_mode_arg, demo_speed_arg, use_sim_time_arg, launch_rviz_arg,
        rviz_config_arg, file_check, rsp_node, jsp_gui_node, joint_demo_node,
        temp_diag_pub_node, rviz_node
    ])
