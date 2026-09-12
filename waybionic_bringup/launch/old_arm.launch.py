"""Display the handoff's four-servo arm without commanding physical hardware."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


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
            'use_joint_state_publisher_gui', default_value='true',
            description='Set false when a live joint-state source is connected'),
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
    return LaunchDescription(arguments + [GroupAction([
        PushRosNamespace('old_arm'),
        SetRemap(src='joint_states', dst=LaunchConfiguration('joint_states_topic')),
        station,
    ])])
