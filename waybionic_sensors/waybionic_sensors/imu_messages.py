"""
Builds ``sensor_msgs/msg/Imu`` messages from :class:`ImuReading` values.

Isolated from the node so the message contract, and especially the raw versus
fused orientation rules, can be asserted directly in unit tests.

Covariance layout
-----------------
Each covariance field is a row-major 3x3 matrix.

``sensor_msgs/Imu`` treats an all-zero matrix as *unknown*, not as perfect
certainty. Raw gyroscope and accelerometer covariances therefore stay all-zero
until electrical supplies a datasheet or calibration value (a positive
``*_stddev`` parameter). A positive standard deviation yields ``stddev**2`` on
the diagonal; off-diagonal terms are then zero because the axes are modelled as
uncorrelated, which is a stated assumption rather than a missing value.

Unavailable orientation
-----------------------
``sensor_msgs/msg/Imu`` defines ``orientation_covariance[0] = -1`` as "this
message does not contain orientation". The raw topic always sets that, because
an accelerometer and a gyroscope alone cannot observe absolute heading.

The demo topic may carry a synthetic orientation covariance. That placeholder
is restricted to demo output and is never copied onto ``data_raw``.
"""

from typing import Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

from waybionic_sensors.imu_reading import ImuReading, Quaternion

ORIENTATION_UNAVAILABLE = -1.0
"""Value placed in ``orientation_covariance[0]`` to mark orientation absent."""

UNKNOWN_COVARIANCE = [0.0] * 9
"""All-zero 3x3 matrix: ROS ``sensor_msgs/Imu`` encoding for unknown covariance."""

IDENTITY_QUATERNION: Quaternion = (0.0, 0.0, 0.0, 1.0)


def to_time_msg(stamp_ns: int) -> Time:
    """Convert integer nanoseconds into a ``builtin_interfaces/msg/Time``."""
    return Time(sec=int(stamp_ns // 1_000_000_000), nanosec=int(stamp_ns % 1_000_000_000))


def diagonal_covariance(stddev: float) -> list:
    """Return a row-major 3x3 covariance with ``stddev**2`` on the diagonal."""
    variance = float(stddev) ** 2
    return [
        variance, 0.0, 0.0,
        0.0, variance, 0.0,
        0.0, 0.0, variance,
    ]


def covariance_from_stddev(stddev: float) -> list:
    """
    Return ROS covariance for a 3-vector.

    A non-positive ``stddev`` yields :data:`UNKNOWN_COVARIANCE` (all zeros). A
    positive value yields a diagonal matrix from :func:`diagonal_covariance`.
    """
    if float(stddev) <= 0.0:
        return list(UNKNOWN_COVARIANCE)
    return diagonal_covariance(stddev)


def build_raw_imu_message(
    reading: ImuReading,
    frame_id: str,
    *,
    angular_velocity_stddev: float,
    linear_acceleration_stddev: float,
) -> Imu:
    """
    Build the raw message for ``/waybionic/imu/data_raw``.

    Orientation is always marked unavailable here, even if the reading happens
    to carry one, because this topic is defined as un-fused sensor output.
    Measurement covariances stay unknown unless a positive standard deviation
    has been supplied from a datasheet or calibration.
    """
    message = Imu()
    message.header.stamp = to_time_msg(reading.stamp_ns)
    message.header.frame_id = frame_id

    message.orientation.x = IDENTITY_QUATERNION[0]
    message.orientation.y = IDENTITY_QUATERNION[1]
    message.orientation.z = IDENTITY_QUATERNION[2]
    message.orientation.w = IDENTITY_QUATERNION[3]
    message.orientation_covariance = [0.0] * 9
    message.orientation_covariance[0] = ORIENTATION_UNAVAILABLE

    message.angular_velocity.x = float(reading.angular_velocity[0])
    message.angular_velocity.y = float(reading.angular_velocity[1])
    message.angular_velocity.z = float(reading.angular_velocity[2])
    message.angular_velocity_covariance = covariance_from_stddev(angular_velocity_stddev)

    message.linear_acceleration.x = float(reading.linear_acceleration[0])
    message.linear_acceleration.y = float(reading.linear_acceleration[1])
    message.linear_acceleration.z = float(reading.linear_acceleration[2])
    message.linear_acceleration_covariance = covariance_from_stddev(
        linear_acceleration_stddev
    )

    return message


def build_demo_orientation_message(
    reading: ImuReading,
    frame_id: str,
    orientation: Quaternion,
    *,
    orientation_stddev: float,
    angular_velocity_stddev: float,
    linear_acceleration_stddev: float,
) -> Imu:
    """
    Build the clearly separated demo/fused message.

    The orientation here is generated for visualisation. It is published on its
    own topic so that nothing subscribing to the raw topic can mistake it for a
    measured attitude. Synthetic orientation covariance, when used, stays on
    this topic only.
    """
    message = build_raw_imu_message(
        reading,
        frame_id,
        angular_velocity_stddev=angular_velocity_stddev,
        linear_acceleration_stddev=linear_acceleration_stddev,
    )

    message.orientation.x = float(orientation[0])
    message.orientation.y = float(orientation[1])
    message.orientation.z = float(orientation[2])
    message.orientation.w = float(orientation[3])
    message.orientation_covariance = covariance_from_stddev(orientation_stddev)

    return message


def build_demo_transform(
    stamp_ns: int,
    parent_frame_id: str,
    frame_id: str,
    orientation: Quaternion,
    translation: Tuple[float, float, float] = (0.0, 0.0, 0.1),
) -> TransformStamped:
    """
    Build the optional demo TF that rotates ``frame_id`` for visualisation.

    This is a mock aid. Broadcasting it by default would imply the sensor knows
    its own attitude, so the node only sends it when explicitly enabled.
    """
    transform = TransformStamped()
    transform.header.stamp = to_time_msg(stamp_ns)
    transform.header.frame_id = parent_frame_id
    transform.child_frame_id = frame_id
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    transform.transform.rotation.x = float(orientation[0])
    transform.transform.rotation.y = float(orientation[1])
    transform.transform.rotation.z = float(orientation[2])
    transform.transform.rotation.w = float(orientation[3])
    return transform
