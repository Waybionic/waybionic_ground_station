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
HOME_PHYSICAL_DEGREES = [90.0, 35.0, 151.5, 27.5]
SERVO_DIRECTIONS = [1.0, -1.0, 1.0, 1.0]
MODEL_HOME_OFFSETS_DEGREES = [0.0, 90.0, 0.0, 0.0]
PHYSICAL_LIMITS = [(0.0, 270.0), (0.0, 112.5), (90.0, 270.0), (0.0, 270.0)]
SEQUENCE = [
    [60.0, 95.0, 125.0, 60.0],
    [45.0, 75.0, 200.0, 100.0],
    [15.0, 100.0, 130.0, 5.0],
    HOME_PHYSICAL_DEGREES,
]
RESPONSE_TIMEOUT_SECONDS = 8.0
FAULT_STATUSES = {
    'arduino-error',
    'connection-failed',
    'serial-read-failed',
    'serial-write-failed',
    'malformed-response',
    'target-rejected',
}


def model_radians_from_physical(degrees):
    return [
        math.radians((physical - zero) / direction + offset)
        for physical, zero, direction, offset in zip(
            degrees, HOME_PHYSICAL_DEGREES, SERVO_DIRECTIONS,
            MODEL_HOME_OFFSETS_DEGREES)
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
        self.status_publisher = self.create_publisher(
            String, '/old_arm_motion_test/status', 10)
        self.command_subscription = self.create_subscription(
            String, '/old_arm_motion_test/command', self.handle_command, 10)

        self.serial_port = None
        self.serial_lock = threading.Lock()
        self.serial_reader = None
        self.serial_stop = threading.Event()
        self.connected = False
        self.ready = False
        self.faulted = False
        self.last_status = 'starting'
        self.last_error = ''
        self.last_response_monotonic = None
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
            self.ready = True
            self.last_response_monotonic = time.monotonic()
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
            self.ready = False
            self.last_response_monotonic = time.monotonic()
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
                    self.last_response_monotonic = time.monotonic()
                    self.process_serial_line(line)
            except Exception as error:
                self._latch_fault('serial-read-failed', str(error))
                return

    def _write_hold(self):
        if self.dry_run or self.serial_port is None:
            return True
        try:
            with self.serial_lock:
                self.serial_port.write(b'HOLD\n')
            return True
        except Exception as error:
            self.last_error = str(error)
            return False

    def _latch_fault(self, status, error):
        self._write_hold()
        self.motion_active = False
        self.sequence_active = False
        self.faulted = True
        self.connected = False
        self.ready = False
        self.last_error = error
        self.last_status = status

    def process_serial_line(self, line):
        self.get_logger().debug(f'Arduino: {line}')
        if line.startswith('READY,IK4,1'):
            self.connected = True
            self.ready = True
            self.faulted = False
            self.last_error = ''
            self.last_status = 'ready'
            return
        if line.startswith('ERROR,'):
            self._latch_fault('arduino-error', line[6:])
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
            return

        self._latch_fault('malformed-response', f'Unexpected response: {line}')

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
            self._latch_fault('serial-write-failed', str(error))
            return False

    def handle_command(self, message):
        command = message.data.strip().upper()
        if command == 'RUN':
            if not self.connected or not self.ready or self.faulted:
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
            self._latch_fault(
                'target-rejected', 'Target exceeds Arduino software limits.')
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
        if (
            not self.dry_run and self.connected and
            self.motion_active and
            self.last_response_monotonic is not None and
            time.monotonic() - self.last_response_monotonic > (
                self.motion_duration + RESPONSE_TIMEOUT_SECONDS)
        ):
            self._latch_fault(
                'serial-read-failed', 'Arduino response watchdog timed out.')

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
        bridge_status = DiagnosticStatus()
        bridge_status.name = 'waybionic_arduino_bridge'
        bridge_status.level = (
            DiagnosticStatus.ERROR
            if self.faulted or not self.connected or self.last_status in FAULT_STATUSES
            else DiagnosticStatus.OK
        )
        bridge_status.message = self.last_status
        bridge_status.values = [
            KeyValue(key='port', value=self.port or 'not configured'),
            KeyValue(key='baud', value=str(self.baud)),
            KeyValue(key='mode', value='dry-run' if self.dry_run else 'physical'),
            KeyValue(key='error', value=self.last_error),
        ]

        subscriber_count = self.count_subscribers('/joint_states')
        output_status = DiagnosticStatus()
        output_status.name = 'waybionic_joint_state_output'
        output_status.level = (
            DiagnosticStatus.OK if subscriber_count > 0 else DiagnosticStatus.ERROR)
        output_status.message = (
            'robot_state_publisher connected'
            if subscriber_count > 0 else
            'No subscribers on /joint_states; RViz cannot update')
        output_status.values = [
            KeyValue(key='subscriber_count', value=str(subscriber_count)),
            KeyValue(key='topic', value='/joint_states'),
        ]

        feedback_status = DiagnosticStatus()
        feedback_status.name = 'waybionic_servo_feedback'
        feedback_status.level = DiagnosticStatus.WARN
        feedback_status.message = (
            'No measured servo feedback; /joint_states is estimated')
        feedback_status.values = [
            KeyValue(key='verification', value='command timeline only'),
            KeyValue(key='servos', value='individual servo connection unknown'),
        ]

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [bridge_status, output_status, feedback_status]
        self.diagnostics_publisher.publish(message)

        if self.faulted or bridge_status.level == DiagnosticStatus.ERROR:
            motion_status = 'FAULT'
        elif self.motion_active:
            motion_status = 'RUNNING'
        elif self.last_status == 'arrived':
            motion_status = 'COMPLETE'
        elif self.last_status == 'held':
            motion_status = 'STOPPED'
        else:
            motion_status = 'READY'
        status_message = String()
        status_message.data = motion_status
        self.status_publisher.publish(status_message)

    def destroy_node(self):
        if self.motion_active and (self.connected or self.serial_port is not None):
            self._write_hold()
        self.motion_active = False
        self.sequence_active = False
        self.serial_stop.set()
        if self.serial_reader is not None and self.serial_reader.is_alive():
            self.serial_reader.join(timeout=0.5)
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