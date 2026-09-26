"""Display the handoff's four-servo arm without commanding physical hardware."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace, SetRemap


def generate_launch_description():
    """Reuse the ground station with an isolated model and joint-state topic."""
    description = get_package_share_directory('waybionic_description')
    bringup = get_package_share_directory('waybionic_bringup')
    arguments = [
        DeclareLaunchArgument(
            'model',
            default_value=os.path.join(description, 'urdf', 'waybionic_old_arm.urdf.xacro'),
            description='Old-arm Xacro or an expanded, calibrated URDF'),
        DeclareLaunchArgument(
            'joint_states_topic', default_value='/old_arm/joint_states',
            description='sensor_msgs/JointState input, with named model joints in radians'),
        DeclareLaunchArgument(
            'use_joint_state_publisher_gui', default_value='false',
            description=(
                'Opt into the manual joint stepper instead of the upright '
                'motion-test source')),
        DeclareLaunchArgument(
            'launch_rviz', default_value='true',
            description='Set false for headless transform validation'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Enable only when an external simulation clock is provided'),
        DeclareLaunchArgument(
            'use_mock_diagnostics', default_value='true',
            description='Diagnostics remain mock unless a real backend is selected'),
        DeclareLaunchArgument(
            'diagnostics_topic', default_value='/diagnostics',
            description='DiagnosticArray input for the Engineering Monitor'),
        DeclareLaunchArgument(
            'start_motion_test', default_value='true',
            description='Start the old-arm motion test node'),
        DeclareLaunchArgument(
            'motion_test_segment_duration', default_value='3.0',
            description='Seconds for each synchronized motion segment'),
        DeclareLaunchArgument(
            'hardware_mode', default_value='simulation',
            description='Arm mode: simulation or arduino'),
        DeclareLaunchArgument(
            'arduino_port', default_value='',
            description='Arduino serial port, for example /dev/ttyACM0'),
        DeclareLaunchArgument(
            'arduino_baud', default_value='115200',
            description='Arduino serial baud rate'),
        DeclareLaunchArgument(
            'arduino_dry_run', default_value='false',
            description='Run the Arduino bridge without opening a serial port'),
    ]
    station = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'ground_station.launch.py')),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'rvizconfig': os.path.join(bringup, 'rviz', 'waybionic_old_arm.rviz'),
            'use_joint_state_publisher_gui': LaunchConfiguration('use_joint_state_publisher_gui'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_mock_diagnostics': LaunchConfiguration('use_mock_diagnostics'),
            'diagnostics_topic': LaunchConfiguration('diagnostics_topic'),
            'start_temporary_diagnostics_publisher': 'false',
        }.items(),
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
            {'segment_duration': LaunchConfiguration(
                'motion_test_segment_duration')}
        ],
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
            {'segment_duration': LaunchConfiguration(
                'motion_test_segment_duration')},
            {'dry_run': LaunchConfiguration('arduino_dry_run')},
        ],
    )

    return LaunchDescription(arguments + [GroupAction([
        PushRosNamespace('old_arm'),
        SetRemap(src='joint_states', dst=LaunchConfiguration('joint_states_topic')),
        station,
        motion_test_node,
        arduino_bridge_node,
    ])])
