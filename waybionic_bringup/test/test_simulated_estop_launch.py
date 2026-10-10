"""Exercise the simulated emergency-stop service and published status."""

import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


@pytest.mark.launch_test
def generate_test_description():
    safety_node = Node(
        package='waybionic_teleop',
        executable='sim_safety_status',
        name='sim_safety_status',
        output='screen',
    )
    return launch.LaunchDescription([
        safety_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSimulatedEmergencyStop(unittest.TestCase):

    def test_service_toggles_topic_and_diagnostics(self):
        rclpy.init()
        node = rclpy.create_node('simulated_estop_launch_test')
        emergency_states = []
        diagnostic_messages = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(
            Bool, '/waybionic/safety/emergency_stop', emergency_states.append, latched)
        node.create_subscription(DiagnosticArray, '/diagnostics',
                                 diagnostic_messages.append, 10)
        client = node.create_client(
            SetBool, '/waybionic/safety/emergency_stop/set')

        def spin_until(predicate, timeout=5.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if predicate():
                    return True
            return predicate()

        def set_pressed(pressed):
            self.assertTrue(client.wait_for_service(timeout_sec=5.0))
            future = client.call_async(SetBool.Request(data=pressed))
            self.assertTrue(spin_until(future.done), 'E-stop service call timed out')
            response = future.result()
            self.assertTrue(response.success)
            self.assertEqual(response.message,
                             'Emergency stop pressed' if pressed else 'Emergency stop released')
            self.assertTrue(
                spin_until(lambda: emergency_states and emergency_states[-1].data is pressed),
                f'E-stop topic did not publish {pressed}')

            def diagnostic_matches():
                if not diagnostic_messages:
                    return False
                rows = {status.name: status for status in diagnostic_messages[-1].status}
                status = rows.get('safety.emergency_stop')
                expected_level = DiagnosticStatus.ERROR if pressed else DiagnosticStatus.OK
                expected_value = 'pressed' if pressed else 'released'
                return (status is not None and status.level == expected_level
                        and status.values[0].value == expected_value)

            self.assertTrue(
                spin_until(diagnostic_matches),
                f'E-stop diagnostics did not report {"pressed" if pressed else "released"}')

        try:
            set_pressed(True)
            set_pressed(False)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
