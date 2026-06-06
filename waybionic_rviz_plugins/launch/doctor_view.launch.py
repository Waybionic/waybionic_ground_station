from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    rviz_config = PathJoinSubstitution([
        FindPackageShare("waybionic_rviz_plugins"),
        "config",
        "doctor_camera_view.rviz",
    ])

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="waybionic_doctor_rviz",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
