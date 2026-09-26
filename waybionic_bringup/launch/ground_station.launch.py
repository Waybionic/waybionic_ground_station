import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def check_files_exist(context, *args, **kwargs):
    model_path = LaunchConfiguration('model').perform(context)
    rviz_path = LaunchConfiguration('rvizconfig').perform(context)
    if not os.path.exists(model_path):
        raise FileNotFoundError(f'Model file not found: {model_path}')
    if not os.path.exists(rviz_path):
        raise FileNotFoundError(f'RViz config file not found: {rviz_path}')
    if (LaunchConfiguration('demo_mode').perform(context) == 'true'
            and LaunchConfiguration('teleop').perform(context) == 'true'):
        raise RuntimeError('demo_mode and teleop both publish joint states; choose one')
    joy_source = LaunchConfiguration('joy_source').perform(context)
    if joy_source not in ('device', 'udp', 'none'):
        raise ValueError(f'joy_source must be device, udp or none, not {joy_source}')
    return []


def generate_launch_description():
    waybionic_desc_dir = get_package_share_directory('waybionic_description')
    waybionic_bringup_dir = get_package_share_directory('waybionic_bringup')
    teleop_config_dir = os.path.join(get_package_share_directory('waybionic_teleop'), 'config')

    default_model_path = os.path.join(
        waybionic_desc_dir, 'urdf', 'waybionic_arm.urdf')
    default_rviz_config_path = os.path.join(
        waybionic_bringup_dir, 'rviz', 'waybionic_unified.rviz')

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

    teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='false',
        description='Drive the arm with an Xbox controller through simulated CAN drives')

    joy_source_arg = DeclareLaunchArgument(
        'joy_source', default_value='device',
        description='Controller input: device (local joystick), udp (host bridge) or none')

    joy_udp_bind_arg = DeclareLaunchArgument(
        'joy_udp_bind', default_value='127.0.0.1',
        description='Address the UDP controller bridge listens on (0.0.0.0 inside Docker)')

    joy_udp_port_arg = DeclareLaunchArgument(
        'joy_udp_port', default_value='47300',
        description='Port the UDP controller bridge listens on')

    follow_camera_arg = DeclareLaunchArgument(
        'follow_camera', default_value='true',
        description='Keep the RViz camera centred near the tool as the arm moves')

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
    teleop = LaunchConfiguration('teleop')
    joy_source = LaunchConfiguration('joy_source')
    diagnostics_topic = {'diagnostics_topic': LaunchConfiguration('diagnostics_topic')}
    # Demo mode and teleop publish joint states themselves.
    simulated_joints = PythonExpression([
        "'", demo_mode, "' == 'true' or '", teleop, "' == 'true'"])

    jsp_gui_node = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('use_joint_state_publisher_gui'), "' == 'true' and not ",
            simulated_joints]))
    )

    joy_node = Node(
        package='joy', executable='game_controller_node', name='joy',
        condition=IfCondition(PythonExpression([
            "'", teleop, "' == 'true' and '", joy_source, "' == 'device'"]))
    )

    joy_udp_node = Node(
        package='waybionic_teleop', executable='joy_udp_receiver', name='joy_udp_receiver',
        condition=IfCondition(PythonExpression([
            "'", teleop, "' == 'true' and '", joy_source, "' == 'udp'"])),
        parameters=[
            {'bind_address': LaunchConfiguration('joy_udp_bind')},
            {'port': ParameterValue(LaunchConfiguration('joy_udp_port'), value_type=int)},
            diagnostics_topic
        ]
    )

    teleop_node = Node(
        package='waybionic_teleop', executable='xbox_teleop', name='xbox_teleop',
        output='screen', condition=IfCondition(teleop),
        parameters=[os.path.join(teleop_config_dir, 'xbox_teleop.yaml'), diagnostics_topic]
    )

    drives_node = Node(
        package='waybionic_teleop', executable='sim_arm_drives', name='sim_arm_drives',
        output='screen', condition=IfCondition(teleop),
        parameters=[os.path.join(teleop_config_dir, 'arm_drives.yaml'), diagnostics_topic]
    )

    camera_follower_node = Node(
        package='waybionic_bringup', executable='camera_follower.py', name='camera_follower',
        condition=IfCondition(LaunchConfiguration('follow_camera'))
    )

    demo_speed = ParameterValue(LaunchConfiguration('demo_speed'), value_type=float)
    joint_demo_node = Node(
        package='waybionic_bringup', executable='joint_demo.py', name='joint_demo',
        output='screen',
        condition=IfCondition(demo_mode),
        parameters=[
            {'speed_deg_s': demo_speed},
            {'diagnostics_topic': LaunchConfiguration('diagnostics_topic')}
        ]
    )

    # Pass the correct arguments to the temporary publisher
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

    # Demo mode and teleop report on /diagnostics, so the panel listens to live diagnostics.
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'use_mock_diagnostics': PythonExpression([
                "'false' if ", simulated_joints, " else '",
                LaunchConfiguration('use_mock_diagnostics'), "'"])},
            {'diagnostics_topic': LaunchConfiguration('diagnostics_topic')}
        ]
    )

    return LaunchDescription([
        model_arg, use_mock_diag_arg, diag_topic_arg, start_temp_pub_arg,
        use_jsp_gui_arg, demo_mode_arg, demo_speed_arg, teleop_arg, joy_source_arg,
        joy_udp_bind_arg, joy_udp_port_arg, follow_camera_arg, use_sim_time_arg,
        launch_rviz_arg, rviz_config_arg, file_check, rsp_node, jsp_gui_node, joint_demo_node,
        joy_node, joy_udp_node, teleop_node, drives_node, camera_follower_node,
        temp_diag_pub_node, rviz_node
    ])
