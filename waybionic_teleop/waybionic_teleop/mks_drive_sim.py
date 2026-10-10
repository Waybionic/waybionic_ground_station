"""
Answer MKS SERVO frames on a CAN interface with simulated drives, in place of the real ones.

This lets the host be tested over a real CAN interface without hardware, for example
``ros2 run waybionic_teleop mks_drive_sim --interface socketcan --channel vcan0``.
"""

import argparse
import threading
import time

from waybionic_teleop.can_bus import CanBus
from waybionic_teleop.sim_drives import SimulatedServo


def serve(bus, servos, stop, period=0.001):
    """Answer commands and move the servos until stop is set."""
    last = time.monotonic()
    while not stop.is_set():
        replies = []
        frame = bus.receive(period)
        while frame is not None:
            can_id, data = frame
            if can_id in servos:
                try:
                    replies += [(can_id, reply) for reply in servos[can_id].receive(data)]
                except ValueError:
                    bus.errors += 1
            frame = bus.receive()
        now = time.monotonic()
        for can_id, servo in servos.items():
            replies += [(can_id, reply) for reply in servo.step(now - last)]
        last = now
        for can_id, data in replies:
            bus.send(can_id, data)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Simulate MKS SERVO42D/57D drives on CAN.')
    parser.add_argument('--interface', default='socketcan', help='python-can interface')
    parser.add_argument('--channel', default='vcan0', help='for example vcan0 or 239.74.163.2')
    parser.add_argument('--bitrate', type=int, default=1000000)
    parser.add_argument('--ids', type=int, nargs='+', default=[1, 2, 3, 4, 5, 6],
                        help='CAN IDs of the simulated drives')
    args = parser.parse_args(argv)
    bus = CanBus(args.interface, args.channel, args.bitrate)
    print(f'Simulating MKS drives {args.ids} on {args.interface} {args.channel}; '
          'press Ctrl+C to stop', flush=True)
    try:
        serve(bus, {can_id: SimulatedServo(can_id) for can_id in args.ids}, threading.Event())
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
