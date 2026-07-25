from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_diagnostics = LaunchConfiguration("use_mock_diagnostics")
    diagnostics_topic = LaunchConfiguration("diagnostics_topic")

    rviz_config = PathJoinSubstitution([
        FindPackageShare("waybionic_rviz_plugins"),
        "config",
        "engineer_monitoring_view.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_diagnostics",
            default_value="true",
            choices=["true", "false"],
            description="Use local mock diagnostics validation states instead of live ROS 2 diagnostics.",
        ),
        DeclareLaunchArgument(
            "diagnostics_topic",
            default_value="/diagnostics",
            description="ROS 2 diagnostic_msgs/msg/DiagnosticArray topic used when use_mock_diagnostics is false.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulation time for passive visualization.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="waybionic_engineer_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "use_mock_diagnostics": ParameterValue(use_mock_diagnostics, value_type=bool),
                "diagnostics_topic": ParameterValue(diagnostics_topic, value_type=str),
            }],
        ),
    ])
