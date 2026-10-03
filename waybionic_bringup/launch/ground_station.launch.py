import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution, Command, EqualsSubstitution, IfElseSubstitution, LaunchConfiguration,
    NotSubstitution, OrSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rclpy.expand_topic_name import expand_topic_name
from rclpy.validate_full_topic_name import validate_full_topic_name

import xacro
import yaml


BOOLEAN_CHOICES = ['true', 'false', 'True', 'False', '1', '0']
BOOLEAN_OPTIONS = {
    'launch_rviz': 'true',
    'use_joint_state_publisher_gui': 'true',
    'use_diagnostics': 'true',
    'use_mock_diagnostics': 'true',
    'start_temporary_diagnostics_publisher': 'false',
    'demo_mode': 'false',
    'teleop': 'false',
    'autoplay': 'false',
    'follow_camera': 'true',
    'camera': 'false',
    'use_sim_time': 'false',
}


def _read_file(path, argument):
    try:
        with open(path, encoding='utf-8') as source:
            return source.read()
    except (OSError, UnicodeError) as exc:
        raise ValueError(f"Invalid {argument} '{path}': {exc}") from exc


def _without_diagnostics(config):
    if not isinstance(config, dict):
        raise ValueError('RViz config must contain a YAML mapping')
    panels = config.get('Panels', [])
    if not isinstance(panels, list) or any(not isinstance(panel, dict) for panel in panels):
        raise ValueError('RViz config Panels must be a list of mappings')
    return dict(config, Panels=[
        panel for panel in panels
        if panel.get('Class') != 'waybionic_rviz_plugins/DiagnosticsPanel'
    ])


def check_files_exist(context, *args, **kwargs):
    """Validate active resources before any nodes start."""
    model_path = LaunchConfiguration('model').perform(context)
    _read_file(model_path, 'model')
    try:
        xacro.process_file(model_path).toxml()
    except Exception as exc:
        raise ValueError(f"Invalid model '{model_path}': {exc}") from exc

    demo_mode = IfCondition(LaunchConfiguration('demo_mode')).evaluate(context)
    teleop = IfCondition(LaunchConfiguration('teleop')).evaluate(context)
    if demo_mode and teleop:
        raise RuntimeError('demo_mode and teleop both publish joint states; choose one')
    joy_source = LaunchConfiguration('joy_source').perform(context)
    if joy_source not in ('device', 'udp', 'none'):
        raise ValueError(f'joy_source must be device, udp or none, not {joy_source}')
    if IfCondition(LaunchConfiguration('autoplay')).evaluate(context):
        if not teleop:
            raise RuntimeError('autoplay plays the controller demo, so it needs teleop:=true')
        if LaunchConfiguration('drive_interface').perform(context) != 'sim':
            raise RuntimeError('autoplay only runs with simulated drives')

    use_diagnostics = IfCondition(LaunchConfiguration('use_diagnostics')).evaluate(context)
    topic = LaunchConfiguration('diagnostics_topic').perform(context)
    if use_diagnostics:
        try:
            validate_full_topic_name(expand_topic_name(topic, 'rviz2', '/'))
        except (ValueError, RuntimeError) as exc:
            raise ValueError(f"Invalid diagnostics_topic '{topic}': {exc}") from exc

    actions = [
        LogInfo(msg=f'Robot model: {model_path}'),
        LogInfo(msg='Ground station: '
                f'demo_mode={str(demo_mode).lower()}, '
                f'teleop={str(teleop).lower()}, '
                f'use_diagnostics={str(use_diagnostics).lower()}'),
    ]
    rviz_path = LaunchConfiguration('rvizconfig').perform(context)
    context.launch_configurations['effective_rvizconfig'] = rviz_path
    if IfCondition(LaunchConfiguration('launch_rviz')).evaluate(context):
        try:
            config = yaml.safe_load(_read_file(rviz_path, 'rvizconfig'))
            filtered = _without_diagnostics(config)
        except (ValueError, yaml.YAMLError) as exc:
            raise ValueError(f"Invalid rvizconfig '{rviz_path}': {exc}") from exc
        actions.append(LogInfo(msg=f'RViz layout: {rviz_path}'))
        if not use_diagnostics:
            temporary = tempfile.TemporaryDirectory(prefix='waybionic-rviz-')
            effective_path = os.path.join(temporary.name, 'without_diagnostics.rviz')
            with open(effective_path, 'w', encoding='utf-8') as output:
                yaml.safe_dump(filtered, output, sort_keys=False)
            context.launch_configurations['effective_rvizconfig'] = effective_path
            actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
                OpaqueFunction(function=lambda context, temporary=temporary: temporary.cleanup())
            ])))
    if not use_diagnostics:
        actions.append(LogInfo(msg=(
            'Diagnostics disabled: RViz monitoring panel and temporary publisher are disabled.'
        )))
    return actions


def generate_launch_description():
    waybionic_desc_dir = get_package_share_directory('waybionic_description')
    waybionic_bringup_dir = get_package_share_directory('waybionic_bringup')
    teleop_config_dir = os.path.join(get_package_share_directory('waybionic_teleop'), 'config')

    default_model_path = os.path.join(
        waybionic_desc_dir, 'urdf', 'waybionic_arm.urdf')
    default_rviz_config_path = os.path.join(
        waybionic_bringup_dir, 'rviz', 'waybionic_unified.rviz')

    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model_path,
        description='Absolute path to robot urdf')

    use_mock_diag_arg = DeclareLaunchArgument(
        'use_mock_diagnostics', default_value='true', choices=BOOLEAN_CHOICES,
        description='Panel mode: true for internal mock, false for live topics')

    diag_topic_arg = DeclareLaunchArgument(
        'diagnostics_topic', default_value='/diagnostics',
        description='Diagnostics topic name')

    start_temp_pub_arg = DeclareLaunchArgument(
        'start_temporary_diagnostics_publisher', default_value='false', choices=BOOLEAN_CHOICES,
        description='Start the optional temporary publisher for live demo')

    use_jsp_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui', default_value='true', choices=BOOLEAN_CHOICES,
        description='Launch joint state publisher GUI')

    use_diagnostics_arg = DeclareLaunchArgument(
        'use_diagnostics', default_value='true', choices=BOOLEAN_CHOICES,
        description='Enable RViz diagnostics panel and optional demo publisher')

    demo_mode_arg = DeclareLaunchArgument(
        'demo_mode', default_value='false', choices=BOOLEAN_CHOICES,
        description='Sweep each joint in turn and check its TF (simulated joint states only)')

    demo_speed_arg = DeclareLaunchArgument(
        'demo_speed', default_value='30.0',
        description='Joint demo sweep speed in degrees per second')

    teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='false', choices=BOOLEAN_CHOICES,
        description='Drive the arm with an Xbox controller through CAN drives')

    drive_interface_arg = DeclareLaunchArgument(
        'drive_interface', default_value='sim',
        description='Drive bus: sim for simulated drives, or a python-can interface such as '
                    'socketcan or slcan for the real MKS drives')

    drive_channel_arg = DeclareLaunchArgument(
        'drive_channel', default_value='',
        description='CAN channel for a real drive bus, such as can0 or /dev/ttyACM0')

    joy_source_arg = DeclareLaunchArgument(
        'joy_source', default_value='device',
        description='Controller input: device (local joystick), udp (host bridge) or none')

    autoplay_arg = DeclareLaunchArgument(
        'autoplay', default_value='false', choices=BOOLEAN_CHOICES,
        description='Play a scripted demo whenever the controller is left idle '
                    '(teleop with simulated drives only)')

    joy_udp_bind_arg = DeclareLaunchArgument(
        'joy_udp_bind', default_value='127.0.0.1',
        description='Address the UDP controller bridge listens on (0.0.0.0 inside Docker)')

    joy_udp_port_arg = DeclareLaunchArgument(
        'joy_udp_port', default_value='47300',
        description='Port the UDP controller bridge listens on')

    follow_camera_arg = DeclareLaunchArgument(
        'follow_camera', default_value='true', choices=BOOLEAN_CHOICES,
        description='Keep the RViz camera centred near the tool as the arm moves')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', choices=BOOLEAN_CHOICES,
        description='Use simulation time')

    camera_arg = DeclareLaunchArgument(
        'camera', default_value='false', choices=BOOLEAN_CHOICES,
        description='Show the doctor view sent by camera_bridge and report its delay')

    camera_bind_arg = DeclareLaunchArgument(
        'camera_bind', default_value='127.0.0.1',
        description='Address the camera receiver listens on (0.0.0.0 inside Docker)')

    camera_log_arg = DeclareLaunchArgument(
        'camera_log', default_value='',
        description='CSV file for each camera frame: capture time, arrival time and delay')

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz', default_value='true', choices=BOOLEAN_CHOICES,
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
    teleop = LaunchConfiguration('teleop')
    joy_source = LaunchConfiguration('joy_source')
    diagnostics_topic = {'diagnostics_topic': LaunchConfiguration('diagnostics_topic')}
    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    # Demo mode and teleop publish joint states themselves.
    simulated_joints = OrSubstitution(demo_mode, teleop)

    jsp_gui_node = Node(
        package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
        condition=IfCondition(AndSubstitution(
            LaunchConfiguration('use_joint_state_publisher_gui'),
            NotSubstitution(simulated_joints)))
    )

    # With autoplay, the controller goes through the autoplay node before it reaches teleop.
    autoplay = AndSubstitution(teleop, LaunchConfiguration('autoplay'))
    operator_joy = [('joy', IfElseSubstitution(autoplay, 'joy_operator', 'joy'))]

    joy_node = Node(
        package='joy', executable='game_controller_node', name='joy',
        condition=IfCondition(AndSubstitution(teleop, EqualsSubstitution(joy_source, 'device'))),
        parameters=[sim_time], remappings=operator_joy
    )

    joy_udp_node = Node(
        package='waybionic_teleop', executable='joy_udp_receiver', name='joy_udp_receiver',
        condition=IfCondition(AndSubstitution(teleop, EqualsSubstitution(joy_source, 'udp'))),
        parameters=[
            {'bind_address': LaunchConfiguration('joy_udp_bind')},
            {'port': ParameterValue(LaunchConfiguration('joy_udp_port'), value_type=int)},
            diagnostics_topic, sim_time
        ],
        remappings=operator_joy
    )

    autoplay_node = Node(
        package='waybionic_teleop', executable='autoplay', name='autoplay', output='screen',
        condition=IfCondition(autoplay), parameters=[diagnostics_topic, sim_time]
    )

    teleop_node = Node(
        package='waybionic_teleop', executable='xbox_teleop', name='xbox_teleop',
        output='screen', condition=IfCondition(teleop),
        parameters=[os.path.join(teleop_config_dir, 'xbox_teleop.yaml'), diagnostics_topic,
                    sim_time]
    )

    drives_node = Node(
        package='waybionic_teleop', executable='sim_arm_drives', name='sim_arm_drives',
        output='screen', condition=IfCondition(teleop),
        parameters=[os.path.join(teleop_config_dir, 'arm_drives.yaml'), diagnostics_topic,
                    {'interface': ParameterValue(LaunchConfiguration('drive_interface'),
                                                 value_type=str),
                     'channel': ParameterValue(LaunchConfiguration('drive_channel'),
                                               value_type=str)},
                    sim_time]
    )

    # RViz orbits view_focus, so the follower also runs for the fixed view.
    camera_follower_node = Node(
        package='waybionic_bringup', executable='camera_follower.py', name='camera_follower',
        parameters=[
            {'follow': ParameterValue(LaunchConfiguration('follow_camera'), value_type=bool)},
            sim_time
        ]
    )

    demo_speed = ParameterValue(LaunchConfiguration('demo_speed'), value_type=float)
    joint_demo_node = Node(
        package='waybionic_bringup', executable='joint_demo.py', name='joint_demo',
        output='screen',
        condition=IfCondition(demo_mode),
        parameters=[
            {'speed_deg_s': demo_speed},
            {'diagnostics_topic': LaunchConfiguration('diagnostics_topic')},
            sim_time
        ]
    )

    temp_diag_pub_node = Node(
        package='waybionic_rviz_plugins', executable='temporary_diagnostics_publisher.py',
        name='temp_diag_pub',
        condition=IfCondition(AndSubstitution(
            LaunchConfiguration('use_diagnostics'),
            LaunchConfiguration('start_temporary_diagnostics_publisher'))),
        parameters=[
            {'mode': 'normal'},
            {'topic': LaunchConfiguration('diagnostics_topic')},
            {'publish_rate_hz': 10.0},
            sim_time
        ]
    )

    camera = LaunchConfiguration('camera')
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('waybionic_camera'), 'launch', 'camera.launch.py')),
        condition=IfCondition(camera),
        launch_arguments={
            'bind_address': LaunchConfiguration('camera_bind'),
            'log_csv': LaunchConfiguration('camera_log'),
            'diagnostics_topic': LaunchConfiguration('diagnostics_topic'),
        }.items()
    )

    # Demo mode, teleop and the camera report on /diagnostics, so the panel listens to them.
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', LaunchConfiguration('effective_rvizconfig')],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'use_mock_diagnostics': AndSubstitution(
                LaunchConfiguration('use_mock_diagnostics'),
                NotSubstitution(OrSubstitution(simulated_joints, camera)))},
            {'diagnostics_topic': LaunchConfiguration('diagnostics_topic')}
        ]
    )

    return LaunchDescription([
        model_arg, use_mock_diag_arg, diag_topic_arg, start_temp_pub_arg,
        use_jsp_gui_arg, use_diagnostics_arg, demo_mode_arg, demo_speed_arg, teleop_arg,
        drive_interface_arg, drive_channel_arg, joy_source_arg, autoplay_arg,
        joy_udp_bind_arg, joy_udp_port_arg, follow_camera_arg, use_sim_time_arg, camera_arg,
        camera_bind_arg, camera_log_arg,
        launch_rviz_arg, rviz_config_arg, file_check, rsp_node, jsp_gui_node, joint_demo_node,
        joy_node, joy_udp_node, autoplay_node, teleop_node, drives_node, camera_follower_node,
        temp_diag_pub_node, camera_launch, rviz_node
    ])
