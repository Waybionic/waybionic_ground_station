"""
Set up the arm's MKS drives on the bench: find them, give each its CAN ID, set the bit rate.

The drives ship as CAN ID 1 at 500 kbit/s. Connect one drive at a time to give it the CAN ID
in arm_drives.yaml, then move every drive to the bus bit rate and check they all answer:

    ros2 run waybionic_teleop mks_setup --channel /dev/cu.usbmodem1101 scan
    ros2 run waybionic_teleop mks_setup --channel /dev/cu.usbmodem1101 set-id 1 3
    ros2 run waybionic_teleop mks_setup --channel /dev/cu.usbmodem1101 set-bitrate 3 1000000
    ros2 run waybionic_teleop mks_setup --channel /dev/cu.usbmodem1101 --bitrate 1000000 scan

The drives' screens (CanID and CanRate) do the same. Stop the ground station's drive node
first: two programs on one carrier would answer each other's frames.
"""

import argparse
import time

from waybionic_teleop import carrier, mks_can


def open_bus(interface, channel, bitrate, tty_baudrate):
    """Open the carrier (slcan) or another python-can interface."""
    if interface == 'slcan':
        from waybionic_teleop.slcan_bus import SlcanBus
        return SlcanBus(channel, bitrate, tty_baudrate)
    from waybionic_teleop.can_bus import CanBus
    return CanBus(interface, channel, bitrate)


def request(bus, can_id, data, code, answer_ids=None, timeout=0.2):
    """Send a frame; return the arguments of the reply with this code, or None."""
    answer_ids = answer_ids or {can_id}
    bus.send(can_id, data)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        bus.step(0.0)
        frame = bus.receive()
        if frame is None:
            time.sleep(0.002)
            continue
        reply_id, reply = frame
        try:
            reply_code, arguments = mks_can.parse(reply_id, reply)
        except ValueError:
            continue
        if reply_id in answer_ids and reply_code == code:
            return arguments
    return None


def scan(bus, ids):
    """Return {CAN ID: encoder count} for the drives that answer."""
    found = {}
    for can_id in ids:
        arguments = request(bus, can_id, mks_can.read_encoder(can_id), mks_can.READ_ENCODER)
        if arguments is not None:
            found[can_id] = mks_can.encoder_value(arguments)
    return found


def set_id(bus, old, new):
    """Give the drive at old the CAN ID new; return a message, or raise SystemExit."""
    if old == new:
        raise SystemExit(f'the drive already has CAN ID {new}')
    if scan(bus, [new]):
        raise SystemExit(f'CAN ID {new} is taken; connect only the drive to change')
    if not scan(bus, [old]):
        raise SystemExit(f'no drive answers on CAN ID {old}')
    status = request(bus, old, mks_can.set_can_id(old, new), mks_can.SET_CAN_ID, {old, new})
    if status != b'\x01' or not scan(bus, [new]):
        raise SystemExit(f'the drive did not take CAN ID {new}')
    return f'The drive on CAN ID {old} now answers on {new}'


def set_bitrate(bus, can_id, bitrate):
    """Switch one drive's CAN bit rate; return a message, or raise SystemExit."""
    status = request(bus, can_id, mks_can.set_bitrate(can_id, bitrate), mks_can.SET_BITRATE)
    if status != b'\x01':
        raise SystemExit(f'CAN ID {can_id} did not confirm the new bit rate')
    return (f'CAN ID {can_id} now runs at {bitrate} bit/s. Once every drive is switched, '
            f'unplug and replug the carrier (it keeps its first bit rate until reset), then '
            f'scan with --bitrate {bitrate}')


def wait_for_carrier(bus, seconds=3.0):
    """Give a carrier that resets when its port opens time to boot; True once it reports."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        bus.step(0.0)
        frame = bus.receive()
        if frame is not None and frame[0] == carrier.STATUS_ID:
            return True
        if frame is None:
            time.sleep(0.01)
    return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--interface', default='slcan',
                        help='slcan for the WayBionic carrier, or a python-can interface')
    parser.add_argument('--channel', required=True,
                        help='for example /dev/cu.usbmodem1101, COM5 or can0')
    parser.add_argument('--bitrate', type=int, default=500000, choices=sorted(mks_can.BITRATES),
                        help="the drives' present CAN bit rate (they ship at 500000)")
    parser.add_argument('--tty-baudrate', type=int, default=1000000,
                        help="the carrier's USB serial speed")
    commands = parser.add_subparsers(dest='command', required=True)
    scan_command = commands.add_parser('scan', help='list the drives that answer')
    scan_command.add_argument('--ids', type=int, nargs='+', default=list(range(1, 17)))
    id_command = commands.add_parser('set-id', help='give one drive a new CAN ID')
    id_command.add_argument('old', type=int)
    id_command.add_argument('new', type=int)
    rate_command = commands.add_parser('set-bitrate', help="change one drive's CAN bit rate")
    rate_command.add_argument('can_id', type=int)
    rate_command.add_argument('rate', type=int, choices=sorted(mks_can.BITRATES))
    commands.add_parser('stop', help='stop every drive at once (F7h to broadcast ID 0)')
    args = parser.parse_args(argv)

    bus = open_bus(args.interface, args.channel, args.bitrate, args.tty_baudrate)
    try:
        if args.interface == 'slcan' and not wait_for_carrier(bus):
            print('No status from the carrier; is carrier_bridge flashed? Trying anyway.')
        if args.command == 'scan':
            found = scan(bus, args.ids)
            for can_id, count in found.items():
                print(f'CAN ID {can_id}: encoder {count / mks_can.COUNTS_PER_REV:+.3f} turns')
            if not found:
                raise SystemExit(f'No drive answered at {args.bitrate} bit/s')
        elif args.command == 'set-id':
            print(set_id(bus, args.old, args.new))
        elif args.command == 'set-bitrate':
            print(set_bitrate(bus, args.can_id, args.rate))
        else:
            bus.send(0, mks_can.emergency_stop(0))
            bus.receive()
            print('Sent the emergency stop to every drive')
    finally:
        bus.shutdown()


if __name__ == '__main__':
    main()
