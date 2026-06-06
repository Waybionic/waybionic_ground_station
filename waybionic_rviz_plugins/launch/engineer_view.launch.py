from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ar_model = LaunchConfiguration("ar_model")
    include_gripper = LaunchConfiguration("include_gripper")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")

    rviz_config = PathJoinSubstitution([
        FindPackageShare("waybionic_rviz_plugins"),
        "config",
        "engineer_monitoring_view.rviz",
    ])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("annin_ar4_description"),
            "urdf",
            "ar.urdf.xacro",
        ]),
        " ",
        "ar_model:=",
        ar_model,
        " ",
        "tf_prefix:=",
        tf_prefix,
        " ",
        "include_gripper:=",
        include_gripper,
    ])
    robot_description = {"robot_description": robot_description_content}

    return LaunchDescription([
        DeclareLaunchArgument(
            "ar_model",
            default_value="mk3",
            choices=["mk1", "mk2", "mk3"],
            description="AR4 model to visualize.",
        ),
        DeclareLaunchArgument(
            "include_gripper",
            default_value="True",
            choices=["True", "False"],
            description="Include the gripper in the visualization.",
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Optional TF prefix for the robot tree.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="False",
            choices=["True", "False"],
            description="Use simulation time for passive visualization.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="waybionic_engineer_robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="waybionic_engineer_joint_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="waybionic_engineer_rviz",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),
    ])
