"""
Status that the WayBionic carrier sends its host ten times a second.

It arrives through the SLCAN link as a frame from STATUS_ID but never goes on the CAN bus,
whose drives use low IDs. See carrier_bridge.ino for the layout.
"""

STATUS_ID = 0x7F0
ESTOP_WIRED, ESTOP_PRESSED, SUPPLY_WIRED = 0x01, 0x02, 0x04


def parse_status(data):
    """Return the carrier's status; e-stop and supply are None until Electrical wires them."""
    data = bytes(data)
    if len(data) != 8:
        raise ValueError('a carrier status frame carries 8 bytes')
    flags = data[1]
    return {
        'sequence': data[0],
        'estop': (('pressed' if flags & ESTOP_PRESSED else 'released')
                  if flags & ESTOP_WIRED else None),
        'supply_v': (int.from_bytes(data[2:4], 'big') / 1000.0
                     if flags & SUPPLY_WIRED else None),
        'can_errors': int.from_bytes(data[4:6], 'big'),
        'failed_writes': data[6],
        'refused_lines': data[7],
    }
