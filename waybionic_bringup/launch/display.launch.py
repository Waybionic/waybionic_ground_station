import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_nodes(context, *args, **kwargs):
    model_path = LaunchConfiguration('model').perform(context)

    if model_path.endswith('.xacro'):
        robot_description = ParameterValue(
            Command([
                FindExecutable(name='xacro'),
                ' ',
                model_path,
            ]),
            value_type=str,
        )
    else:
        with open(model_path, encoding='utf-8') as model_file:
            robot_description = model_file.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    return [robot_state_publisher_node]


def generate_launch_description():
    waybionic_desc_dir = get_package_share_directory('waybionic_description')
    waybionic_bringup_dir = get_package_share_directory('waybionic_bringup')

    default_model_path = os.path.join(
        waybionic_desc_dir, 'urdf', 'waybionic_placeholder.urdf'
    )
    default_rviz_config_path = os.path.join(
        waybionic_bringup_dir, 'rviz', 'waybionic.rviz'
    )

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf or xacro file',
    )

    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file',
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        OpaqueFunction(function=_launch_nodes),
        joint_state_publisher_gui_node,
        rviz_node,
    ])
