"""
Launch the WayBionic IMU publisher on its own.

Demo orientation and demo TF default to false here so the raw sensor contract is
what a plain launch produces. Use imu_demo.launch.py for the visual walkthrough.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_ARGUMENTS = [
    ('use_mock', 'true', 'Generate mock samples instead of reading hardware.', bool),
    ('topic', '/waybionic/imu/data_raw',
     'sensor_msgs/msg/Imu topic for raw gyroscope and accelerometer data.', str),
    ('demo_orientation_topic', '/waybionic/imu/data_demo',
     'Topic for the synthetic demo orientation. Never the raw topic.', str),
    ('diagnostics_topic', '/diagnostics',
     'DiagnosticArray topic carrying imu.heartbeat.', str),
    ('frame_id', 'imu_link', 'Frame of the IMU measurements.', str),
    ('parent_frame_id', 'base_link', 'Parent frame used by the optional demo TF.', str),
    ('publish_rate_hz', '50.0', 'Sample publish rate in Hz.', float),
    ('diagnostics_rate_hz', '2.0', 'Diagnostics publish rate in Hz, at least 1 Hz.', float),
    ('stale_timeout_sec', '1.0', 'Sample age after which imu.heartbeat reports STALE.', float),
    ('publish_demo_orientation', 'false',
     'Publish a synthetic orientation on the demo topic. Visualisation aid only.', bool),
    ('publish_demo_tf', 'false',
     'Broadcast a rotating demo TF for the IMU frame. Visualisation aid only.', bool),
    ('angular_velocity_stddev', '0.01',
     'Assumed gyroscope noise standard deviation in rad/s.', float),
    ('linear_acceleration_stddev', '0.05',
     'Assumed accelerometer noise standard deviation in m/s^2.', float),
    ('orientation_stddev', '0.05',
     'Assumed orientation standard deviation in rad, demo topic only.', float),
    ('mock_stall_after_sec', '0.0',
     'Stop the mock after this many seconds to demonstrate the stale heartbeat. '
     '0 disables stalling.', float),
    ('serial_port', '',
     'Reserved for the future hardware reader. No driver is implemented yet.', str),
]


def generate_launch_description():
    """Build the launch description for the IMU publisher node."""
    declarations = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, description, _ in _ARGUMENTS
    ]

    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, _, value_type in _ARGUMENTS
    }

    return LaunchDescription(declarations + [
        Node(
            package='waybionic_sensors',
            executable='imu_publisher',
            name='waybionic_imu_publisher',
            output='screen',
            parameters=[parameters],
        ),
    ])
