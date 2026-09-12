import math
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


JOINT_NAMES = [
    'old_arm_base_yaw_joint',
    'old_arm_shoulder_pitch_joint',
    'old_arm_elbow_pitch_joint',
    'old_arm_wrist_roll_joint',
]
HOME_PHYSICAL_DEGREES = [33.5, 112.5, 151.5, 27.5]
SERVO_DIRECTIONS = [1.0, -1.0, 1.0, 1.0]
PHYSICAL_LIMITS = [(0.0, 270.0), (0.0, 112.5), (90.0, 270.0), (0.0, 270.0)]
SEQUENCE = [
    [60.0, 95.0, 125.0, 60.0],
    [45.0, 75.0, 200.0, 100.0],
    [15.0, 100.0, 130.0, 5.0],
    HOME_PHYSICAL_DEGREES,
]


def model_radians_from_physical(degrees):
    return [
        math.radians((physical - zero) / direction)
        for physical, zero, direction in zip(
            degrees, HOME_PHYSICAL_DEGREES, SERVO_DIRECTIONS)
    ]


def smoothstep(progress):
    return 6.0 * progress**5 - 15.0 * progress**4 + 10.0 * progress**3


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('waybionic_arduino_bridge')

        self.declare_parameter('port', '')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('segment_duration', 3.0)
        self.declare_parameter('dry_run', False)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.segment_duration = float(self.get_parameter('segment_duration').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self.joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.command_subscription = self.create_subscription(
            String, '/old_arm_motion_test/command', self.handle_command, 10)

        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.serial_reader = None
        self.serial_stop = threading.Event()
        self.connected = False
        self.last_status = 'starting'
        self.last_error = ''
        self.current = list(HOME_PHYSICAL_DEGREES)
        self.start = list(self.current)
        self.target = list(self.current)
        self.motion_started = 0.0
        self.motion_duration = 0.0
        self.motion_active = False
        self.sequence_index = 0
        self.sequence_active = False

        self.state_timer = self.create_timer(0.02, self.publish_estimated_state)
        self.diagnostics_timer = self.create_timer(1.0, self.publish_diagnostics)
        self.connect_to_arduino()
        self.publish_estimated_state()

    def connect_to_arduino(self):
        if self.dry_run:
            self.connected = True
            self.last_status = 'dry-run'
            self.get_logger().warn('Running in dry-run mode; no Arduino commands will be sent.')
            return

        if not self.port:
            self.last_status = 'port-not-configured'
            self.last_error = 'Set the port parameter, for example /dev/ttyACM0.'
            self.get_logger().error(self.last_error)
            return

        try:
            import serial

            self.serial_port = serial.Serial(self.port, self.baud, timeout=0.1)
            self.connected = True
            self.last_status = 'connected'
            self.serial_reader = threading.Thread(target=self.read_serial, daemon=True)
            self.serial_reader.start()
            self.send_line('ID')
            self.get_logger().info(f'Connected to Arduino on {self.port} at {self.baud} baud.')
        except Exception as error:
            self.last_error = str(error)
            self.last_status = 'connection-failed'
            self.get_logger().error(f'Could not open Arduino port {self.port}: {error}')

    def read_serial(self):
        while not self.serial_stop.is_set() and self.serial_port is not None:
            try:
                line = self.serial_port.readline().decode('ascii', errors='replace').strip()
                if line:
                    self.process_serial_line(line)
            except Exception as error:
                self.connected = False
                self.last_error = str(error)
                self.last_status = 'serial-read-failed'
                return

    def process_serial_line(self, line):
        self.get_logger().debug(f'Arduino: {line}')
        if line.startswith('READY,IK4,1'):
            self.last_status = 'ready'
            return
        if line.startswith('ERROR,'):
            self.motion_active = False
            self.sequence_active = False
            self.last_error = line[6:]
            self.last_status = 'arduino-error'
            return
        if line == 'OK,ARRIVED':
            self.current = list(self.target)
            self.motion_active = False
            self.last_status = 'arrived'
            if self.sequence_active:
                self.sequence_index += 1
                if self.sequence_index < len(SEQUENCE):
                    self.send_move(SEQUENCE[self.sequence_index])
                else:
                    self.sequence_active = False
            return
        if line in ('OK,MOVE', 'OK,JOG'):
            self.last_status = 'accepted'
            return
        if line in ('OK,HOLD', 'OK,SWITCH HOLD'):
            self.motion_active = False
            self.sequence_active = False
            self.last_status = 'held'

    def send_line(self, line):
        if self.dry_run:
            self.last_status = 'dry-run-accepted'
            return True
        if not self.connected or self.serial_port is None:
            self.last_status = 'not-connected'
            return False
        try:
            with self.serial_lock:
                self.serial_port.write((line + '\n').encode('ascii'))
            return True
        except Exception as error:
            self.connected = False
            self.last_error = str(error)
            self.last_status = 'serial-write-failed'
            return False

    def handle_command(self, message):
        command = message.data.strip().upper()
        if command == 'RUN':
            if not self.connected:
                self.last_status = 'run-rejected-not-connected'
                return
            self.sequence_index = 0
            self.sequence_active = True
            self.send_move(SEQUENCE[0])
        elif command == 'HOME':
            self.sequence_active = False
            self.send_move(HOME_PHYSICAL_DEGREES)
        elif command == 'STOP':
            self.sequence_active = False
            self.motion_active = False
            if self.send_line('HOLD'):
                self.last_status = 'hold-requested'
            else:
                self.last_status = 'hold-request-failed'
        else:
            self.get_logger().warn(f'Ignoring unknown motion command: {command}')

    def send_move(self, target):
        if any(
            value < lower or value > upper
            for value, (lower, upper) in zip(target, PHYSICAL_LIMITS)
        ):
            self.last_error = 'Target exceeds Arduino software limits.'
            self.last_status = 'target-rejected'
            self.motion_active = False
            self.sequence_active = False
            return

        self.start = list(self.current)
        self.target = list(target)
        self.motion_started = time.monotonic()
        self.motion_duration = max(0.3, min(15.0, self.segment_duration))
        command = 'MOVE,' + ','.join(f'{value:.2f}' for value in target)
        command += f',{int(self.motion_duration * 1000)}'
        if self.send_line(command):
            self.motion_active = True
            self.last_status = 'move-requested'

    def publish_estimated_state(self):
        if self.motion_active:
            progress = min(
                (time.monotonic() - self.motion_started) / self.motion_duration,
                1.0,
            )
            blend = smoothstep(progress)
            self.current = [
                start + (target - start) * blend
                for start, target in zip(self.start, self.target)
            ]

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = model_radians_from_physical(self.current)
        self.joint_publisher.publish(message)

        if self.dry_run and self.motion_active and time.monotonic() >= self.motion_started + self.motion_duration:
            self.process_serial_line('OK,ARRIVED')

    def publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = 'waybionic_arduino_bridge'
        status.level = DiagnosticStatus.OK if self.connected else DiagnosticStatus.ERROR
        status.message = self.last_status
        status.values = [
            KeyValue(key='port', value=self.port or 'not configured'),
            KeyValue(key='baud', value=str(self.baud)),
            KeyValue(key='mode', value='dry-run' if self.dry_run else 'physical'),
            KeyValue(key='error', value=self.last_error),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self.diagnostics_publisher.publish(message)

    def destroy_node(self):
        self.serial_stop.set()
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()