import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
        waybionic_desc_dir, 'urdf', 'waybionic_old_arm.urdf.xacro')
    default_rviz_config_path = os.path.join(
        waybionic_bringup_dir, 'rviz', 'waybionic_old_arm_root.rviz')

    # --- Declare Launch Arguments ---
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
        'start_temporary_diagnostics_publisher', default_value='true',
        description='Start the optional temporary publisher for live demo')

    start_motion_test_arg = DeclareLaunchArgument(
        'start_motion_test', default_value='true',
        description='Start the old-arm four-joint motion-test node')

    hardware_mode_arg = DeclareLaunchArgument(
        'hardware_mode', default_value='simulation',
        description='Arm mode: simulation or arduino')

    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port', default_value='',
        description='Arduino serial port, for example /dev/ttyACM0')

    arduino_baud_arg = DeclareLaunchArgument(
        'arduino_baud', default_value='115200',
        description='Arduino serial baud rate')

    arduino_dry_run_arg = DeclareLaunchArgument(
        'arduino_dry_run', default_value='false',
        description='Run the Arduino bridge without opening a serial port')

    motion_test_duration_arg = DeclareLaunchArgument(
        'motion_test_segment_duration', default_value='3.0',
        description='Seconds for each synchronized motion segment')

    use_jsp_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui', default_value='false',
        description='Launch joint state publisher GUI')

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

    # --- Nodes ---
    robot_description_content = {
        'robot_description': Command(['xacro ', LaunchConfiguration('model')])
    }

    rsp_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[
            robot_description_content,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    jsp_gui_node = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_joint_state_publisher_gui'), "' == 'true' and '",
            LaunchConfiguration('hardware_mode'), "' == 'simulation'"
        ]))
    )

    # Pass the correct arguments to the temporary publisher
    temp_diag_pub_node = Node(
        package='waybionic_rviz_plugins', executable='temporary_diagnostics_publisher.py',
        name='temp_diag_pub',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('start_temporary_diagnostics_publisher'),
            "' == 'true' and '", LaunchConfiguration('hardware_mode'),
            "' == 'simulation'"
        ])),
        parameters=[
            {'mode': 'normal'},
            {'topic': LaunchConfiguration('diagnostics_topic')},
            {'publish_rate_hz': 10.0}
        ]
    )

    motion_test_node = Node(
        package='waybionic_motion_test',
        executable='motion_test',
        name='old_arm_motion_test',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('start_motion_test'), "' == 'true' and '",
            LaunchConfiguration('hardware_mode'), "' == 'simulation'"
        ])),
        parameters=[
            {'segment_duration': LaunchConfiguration('motion_test_segment_duration')}
        ]
    )

    arduino_bridge_node = Node(
        package='waybionic_hardware',
        executable='arduino_bridge',
        name='waybionic_arduino_bridge',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('hardware_mode'), "' == 'arduino'"
        ])),
        parameters=[
            {'port': LaunchConfiguration('arduino_port')},
            {'baud': LaunchConfiguration('arduino_baud')},
            {'segment_duration': LaunchConfiguration('motion_test_segment_duration')},
            {'dry_run': LaunchConfiguration('arduino_dry_run')},
        ]
    )

    # Pass the mock toggles into the RViz node parameters
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'use_mock_diagnostics': LaunchConfiguration('use_mock_diagnostics')},
            {'diagnostics_topic': LaunchConfiguration('diagnostics_topic')}
        ]
    )

    return LaunchDescription([
        model_arg, use_mock_diag_arg, diag_topic_arg, start_temp_pub_arg,
        use_jsp_gui_arg, use_sim_time_arg, launch_rviz_arg, rviz_config_arg,
        start_motion_test_arg, motion_test_duration_arg, hardware_mode_arg,
        arduino_port_arg, arduino_baud_arg, arduino_dry_run_arg, file_check,
        rsp_node, jsp_gui_node, temp_diag_pub_node, motion_test_node,
        arduino_bridge_node, rviz_node
    ])
