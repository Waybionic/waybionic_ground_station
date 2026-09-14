import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


JOINT_NAMES = [
    'old_arm_base_yaw_joint',
    'old_arm_shoulder_pitch_joint',
    'old_arm_elbow_pitch_joint',
    'old_arm_wrist_roll_joint',
]
JOINT_COUNT = len(JOINT_NAMES)
HOME_PHYSICAL_DEGREES = [33.5, 112.5, 151.5, 27.5]
SERVO_DIRECTIONS = [1.0, -1.0, 1.0, 1.0]


def model_radians_from_physical(degrees):
    """Convert calibrated Arduino servo angles into URDF model radians."""
    return [
        math.radians((physical - zero) / direction)
        for physical, zero, direction in zip(
            degrees, HOME_PHYSICAL_DEGREES, SERVO_DIRECTIONS)
    ]


def smoothstep(progress):
    return 6.0 * progress**5 - 15.0 * progress**4 + 10.0 * progress**3


class MotionTestNode(Node):
    def __init__(self):
        super().__init__('motion_test')

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10,
        )
        self.status_publisher = self.create_publisher(
            String, '/old_arm_motion_test/status', 10)
        self.command_subscription = self.create_subscription(
            String,
            '/old_arm_motion_test/command',
            self.handle_command,
            10,
        )

        self.declare_parameter('segment_duration', 3.0)
        self.segment_duration = float(self.get_parameter('segment_duration').value)
        if self.segment_duration <= 0.0:
            raise ValueError('segment_duration must be positive')
        self.home = model_radians_from_physical(HOME_PHYSICAL_DEGREES)
        self.current = list(self.home)
        self.start = list(self.home)
        self.target = list(self.home)
        self.segment_elapsed = 0.0
        self.segment_active = False
        self.sequence = [
            model_radians_from_physical([60.0, 95.0, 125.0, 60.0]),
            model_radians_from_physical([45.0, 75.0, 200.0, 100.0]),
            model_radians_from_physical([15.0, 100.0, 130.0, 5.0]),
            self.home,
        ]
        self.sequence_index = 0
        self.sequence_playback = False
        self.timer = self.create_timer(0.02, self.update_trajectory)
        self.publish_joint_state()
        self.publish_status('READY')

        self.get_logger().info(
            'Old-arm 4-DOF motion test started; publishing simulated commanded positions.')

    def handle_command(self, message):
        command = message.data.strip().upper()
        if command == 'RUN':
            self.sequence_index = 0
            self.sequence_playback = True
            self.start_segment(self.sequence[self.sequence_index])
            self.publish_status('RUNNING')
        elif command == 'HOME':
            self.sequence_playback = False
            self.start_segment(self.home)
            self.publish_status('HOME')
        elif command == 'STOP':
            self.sequence_playback = False
            self.segment_active = False
            self.target = list(self.current)
            self.publish_joint_state()
            self.publish_status('STOPPED')

    def start_segment(self, target):
        self.start = list(self.current)
        self.target = list(target)
        self.segment_elapsed = 0.0
        self.segment_active = True

    def update_trajectory(self):
        if not self.segment_active:
            return

        self.segment_elapsed += 0.02
        progress = min(self.segment_elapsed / self.segment_duration, 1.0)
        blend = smoothstep(progress)
        self.current = [
            start + (target - start) * blend
            for start, target in zip(self.start, self.target)
        ]
        self.publish_joint_state()

        if progress >= 1.0:
            self.current = list(self.target)
            if self.sequence_playback and self.sequence_index < len(self.sequence) - 1:
                self.sequence_index += 1
                self.start_segment(self.sequence[self.sequence_index])
            else:
                self.segment_active = False
                self.publish_status('COMPLETE')

    def publish_status(self, status):
        message = String()
        message.data = status
        self.status_publisher.publish(message)

    def publish_joint_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = self.current
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)

    node = MotionTestNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()
