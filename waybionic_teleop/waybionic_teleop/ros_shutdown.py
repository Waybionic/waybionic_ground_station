"""Tell an error from a node shutting down on SIGINT apart from a real failure."""

import time

import rclpy

# How long to wait for rclpy to finish shutting the context down.
SHUTDOWN_GRACE_S = 1.0

# Errors spin can raise once SIGINT has started invalidating the ROS context.
SHUTDOWN_ERRORS = (
    # SIGINT can shut down the context before spin recreates its wait set.
    'the given context is not valid',
    # Jazzy's binding error when a subscription's context is invalidated.
    "Unable to convert call argument '0' to Python object",
    # tf2 can no longer read a lookup time once the context is invalid.
    'time must have sec and nanosec',
)


def is_shutdown_error(error):
    """
    Return whether spin raised this error only because ROS is shutting down.

    On SIGINT, rclpy's signal thread wakes every executor before it shuts the contexts
    down, so spin can fail while rclpy.ok() is still true. Wait briefly for the shutdown
    to land: the same error while ROS keeps running is a real failure.
    """
    if not any(text in str(error) for text in SHUTDOWN_ERRORS):
        return False
    deadline = time.monotonic() + SHUTDOWN_GRACE_S
    while rclpy.ok():
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.01)
    return True
