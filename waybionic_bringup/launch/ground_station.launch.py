# Start the ground station with explicit visualization and diagnostics options.

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rclpy.expand_topic_name import expand_topic_name
from rclpy.validate_full_topic_name import validate_full_topic_name

import xacro

import yaml


BOOLEAN_OPTIONS = {
    'launch_rviz': ('true', 'Launch the RViz window'),
    'use_joint_state_publisher_gui': ('true', 'Launch the joint-state control window'),
    'use_diagnostics': ('true', 'Enable the RViz diagnostics panel and optional demo publisher'),
    'use_mock_diagnostics': ('true', 'Use internal mock diagnostics instead of live topics'),
    'start_temporary_diagnostics_publisher': ('false', 'Start the demo diagnostics publisher'),
    'use_sim_time': ('false', 'Use the ROS simulation clock'),
}


def _read_file(path, argument):
    try:
        with open(path, encoding='utf-8') as source:
            return source.read()
    except (OSError, UnicodeError) as exc:
        raise ValueError(f"Invalid {argument} '{path}': {exc}") from exc


def _without_diagnostics(config):
    # Preserve the selected layout while removing the monitoring panel.
    if not isinstance(config, dict):
        raise ValueError('RViz config must contain a YAML mapping')
    panels = config.get('Panels', [])
    if not isinstance(panels, list) or any(not isinstance(panel, dict) for panel in panels):
        raise ValueError('RViz config Panels must be a list of mappings')
    return dict(config, Panels=[
        panel for panel in panels
        if panel.get('Class') != 'waybionic_rviz_plugins/DiagnosticsPanel'
    ])


def _launch_nodes(context):
    # Validate all options before returning any processes to launch.
    options = {}
    for name in BOOLEAN_OPTIONS:
        value = LaunchConfiguration(name).perform(context)
        if value not in ('true', 'false'):
            raise ValueError(f"Invalid {name} '{value}': expected true or false")
        options[name] = value == 'true'

    model_path = LaunchConfiguration('model').perform(context)
    _read_file(model_path, 'model')
    try:
        robot_description = xacro.process_file(model_path).toxml()
    except Exception as exc:
        raise ValueError(f"Invalid model '{model_path}': {exc}") from exc

    topic = LaunchConfiguration('diagnostics_topic').perform(context)
    if options['use_diagnostics']:
        try:
            validate_full_topic_name(expand_topic_name(topic, 'rviz2', '/'))
        except (ValueError, RuntimeError) as exc:
            raise ValueError(f"Invalid diagnostics_topic '{topic}': {exc}") from exc

    actions = [LogInfo(msg='Ground station: ' + ', '.join(
        f'{name}={str(value).lower()}' for name, value in options.items()
    )), LogInfo(msg=f'Robot model: {model_path}')]

    rviz_path = LaunchConfiguration('rvizconfig').perform(context)
    if options['launch_rviz']:
        contents = _read_file(rviz_path, 'rvizconfig')
        try:
            config = yaml.safe_load(contents)
            filtered = _without_diagnostics(config)
        except (ValueError, yaml.YAMLError) as exc:
            raise ValueError(f"Invalid rvizconfig '{rviz_path}': {exc}") from exc
        actions.append(LogInfo(msg=f'RViz layout: {rviz_path}'))
        if not options['use_diagnostics']:
            # Never rewrite the user's layout. Remove the temporary copy on shutdown.
            temporary = tempfile.TemporaryDirectory(prefix='waybionic-rviz-')
            rviz_path = os.path.join(temporary.name, 'without_diagnostics.rviz')
            with open(rviz_path, 'w', encoding='utf-8') as output:
                yaml.safe_dump(filtered, output, sort_keys=False)
            actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[
                OpaqueFunction(function=lambda context: temporary.cleanup())
            ])))
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_path],
            parameters=[{
                'use_sim_time': options['use_sim_time'],
                'use_mock_diagnostics': options['use_mock_diagnostics'],
                'diagnostics_topic': topic,
            }],
        ))

    actions.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': options['use_sim_time'],
        }],
    ))
    if options['use_joint_state_publisher_gui']:
        actions.append(Node(
            package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
            output='screen', parameters=[{'use_sim_time': options['use_sim_time']}],
        ))
        if not options['launch_rviz']:
            actions.append(LogInfo(msg=(
                'RViz is disabled, but the joint-state GUI is enabled. For headless '
                'operation also set use_joint_state_publisher_gui:=false.'
            )))
    elif not options['launch_rviz']:
        actions.append(LogInfo(msg='Headless operation: both visualization windows are disabled.'))

    if options['use_diagnostics'] and options['start_temporary_diagnostics_publisher']:
        actions.append(Node(
            package='waybionic_rviz_plugins', executable='temporary_diagnostics_publisher.py',
            name='temp_diag_pub', output='screen',
            parameters=[{'mode': 'normal', 'topic': topic, 'publish_rate_hz': 10.0,
                         'use_sim_time': options['use_sim_time']}],
        ))
    elif not options['use_diagnostics']:
        actions.append(LogInfo(msg=(
            'Diagnostics disabled: RViz monitoring panel and temporary publisher are disabled '
            '(including any request to start the temporary publisher).'
        )))
    return actions


def generate_launch_description():
    # Declare supported options and validate them before starting processes.
    description_dir = get_package_share_directory('waybionic_description')
    bringup_dir = get_package_share_directory('waybionic_bringup')
    arguments = [
        DeclareLaunchArgument(name, default_value=default, choices=['true', 'false'],
                              description=description)
        for name, (default, description) in BOOLEAN_OPTIONS.items()
    ]
    arguments.extend([
        DeclareLaunchArgument(
            'model', default_value=os.path.join(
                description_dir, 'urdf', 'waybionic_placeholder.urdf'),
            description='Path to a readable URDF or xacro robot model'),
        DeclareLaunchArgument(
            'rvizconfig', default_value=os.path.join(
                bringup_dir, 'rviz', 'waybionic_unified.rviz'),
            description='Path to an RViz layout; only needed when launch_rviz=true'),
        DeclareLaunchArgument(
            'diagnostics_topic', default_value='/diagnostics',
            description='ROS topic for live diagnostics and the optional demo publisher'),
    ])
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_nodes)])
