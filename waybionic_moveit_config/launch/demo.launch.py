"""Full MoveIt demo for the WayBionic arm.

Brings up, in one shot:
  robot_state_publisher, ros2_control (mock hardware), the joint_state_broadcaster
  and arm_controller, move_group, and an RViz preloaded with MotionPlanning.

No physical hardware is required.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = "waybionic_moveit_config"


def load_yaml(package_name, file_path):
    absolute_path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(absolute_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Start RViz with the MotionPlanning display",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare(PKG), "rviz", "moveit.rviz"]
            ),
            description="Full path to the RViz configuration file",
        ),
    ]

    # --- robot_description (URDF + ros2_control) ---
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(PKG), "urdf", "waybionic_moveit.urdf.xacro"]
            ),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # --- robot_description_semantic (SRDF) ---
    srdf_path = os.path.join(
        get_package_share_directory(PKG), "srdf", "waybionic.srdf"
    )
    with open(srdf_path, "r", encoding="utf-8") as handle:
        robot_description_semantic = {"robot_description_semantic": handle.read()}

    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(PKG, "config/kinematics.yaml")
    }
    joint_limits = {"robot_description_planning": load_yaml(PKG, "config/joint_limits.yaml")}

    ompl_yaml = load_yaml(PKG, "config/ompl_planning.yaml")
    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": ompl_yaml,
    }

    moveit_controllers = load_yaml(PKG, "config/moveit_controllers.yaml")

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        # Makes move_group publish the SRDF on a topic, so RViz can pick it up
        # even when a MotionPlanning display is added by hand after startup.
        "publish_robot_description_semantic": True,
        "publish_robot_description": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory(PKG), "config", "ros2_controllers.yaml"
    )
    # On Jazzy the controller_manager picks robot_description up from the
    # /robot_description topic published by robot_state_publisher. Passing it as
    # a parameter as well makes it log a spurious "already loaded a urdf" warning.
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
        ],
    )

    # Load arm_controller only once joint_state_broadcaster is up, so the
    # controller_manager is guaranteed ready.
    delayed_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            delayed_arm_controller,
            move_group_node,
            rviz_node,
        ]
    )
