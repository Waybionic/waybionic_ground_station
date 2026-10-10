#!/usr/bin/env python3
"""
Compare the C++/Arduino MKS framing with waybionic_teleop/mks_can.py (PR #24) byte for byte.

mks_can.py is the reference: it was written against the MKS SERVO42D/57D CAN manual V1.0.9.
The script feeds the same edge and random cases to mks_can.py and to the mks_frame_tool
binary built from the shared Arduino library, and fails on the first difference.

    python3 cross_check_mks.py --mks-can path/to/mks_can.py --tool path/to/mks_frame_tool

mks_can.py is not on main yet; one way to get it:
    git show origin/feature/xbox-teleop:waybionic_teleop/waybionic_teleop/mks_can.py > /tmp/mks_can.py
"""

import argparse
import importlib.util
import random
import subprocess
import sys


def load(path):
    spec = importlib.util.spec_from_file_location('mks_can', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def reference(mks, request):
    verb, *args = request.split()
    try:
        if verb == 'encoder_value':
            return str(mks.encoder_value(bytes.fromhex(args[0])))
        can_id = int(args[0])
        if verb == 'parse':
            code, arguments = mks.parse(can_id, bytes.fromhex(args[1]))
            return f'{code:02X} {bytes(arguments).hex().upper()}'
        numbers = [int(value) for value in args[1:]]
        if verb == 'absolute_axis':
            data = mks.absolute_axis(can_id, *numbers)
        elif verb == 'read_encoder':
            data = mks.read_encoder(can_id)
        elif verb == 'enable':
            data = mks.enable(can_id, bool(numbers[0]))
        elif verb == 'set_mode':
            data = mks.set_mode(can_id, numbers[0])
        elif verb == 'set_response':
            data = mks.set_response(can_id, bool(numbers[0]), bool(numbers[1]))
        elif verb == 'set_heartbeat':
            data = mks.set_heartbeat(can_id, numbers[0])
        else:
            raise ValueError(verb)
        return data.hex().upper()
    except (ValueError, OverflowError):
        return 'ERR'


def requests(mks, count, rng):
    edge_axes = [0, 1, -1, 0x4000, -0x4000, mks.MAX_AXIS, -mks.MAX_AXIS, mks.MAX_AXIS + 1,
                 -mks.MAX_AXIS - 1]
    edge_speeds = [0, 1, mks.MAX_SPEED_RPM, mks.MAX_SPEED_RPM + 1, -1]
    edge_accs = [0, 1, 255, 256, -1]
    edge_ids = [0, 1, 0xFF, 0x100, 0x7FF, 0x800]
    for can_id in edge_ids:
        yield f'read_encoder {can_id}'
        yield f'enable {can_id} 1'
        yield f'enable {can_id} 0'
        yield f'set_mode {can_id} {mks.MODE_SR_VFOC}'
        yield f'set_heartbeat {can_id} 500'
        for respond in (0, 1):
            for active in (0, 1):
                yield f'set_response {can_id} {respond} {active}'
        for axis in edge_axes:
            for speed in edge_speeds:
                for acc in edge_accs:
                    yield f'absolute_axis {can_id} {axis} {speed} {acc}'
    for value in (0, 1, -1, -16, 2**47 - 1, -2**47):
        yield f'encoder_value {value.to_bytes(6, "big", signed=True).hex()}'
    for heartbeat in (0, 1, 2**32 - 1, 2**32, -1):
        yield f'set_heartbeat 1 {heartbeat}'
    for _ in range(count):
        can_id = rng.randint(0, 0x800)
        yield (f'absolute_axis {can_id} {rng.randint(-0x800000, 0x800000)} '
               f'{rng.randint(-1, 3001)} {rng.randint(-1, 256)}')
        yield f'set_heartbeat {can_id} {rng.randint(0, 2**32 - 1)}'
        yield f'encoder_value {rng.randbytes(6).hex()}'
        parse_id = rng.randint(0, 0x7FF)
        length = rng.randint(1, 9)
        data = bytearray(rng.randbytes(length))
        if length >= 2 and rng.random() < 0.5:
            data[-1] = mks.checksum(parse_id, data[:-1])
        yield f'parse {parse_id} {data.hex()}'


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--mks-can', required=True)
    parser.add_argument('--tool', required=True)
    parser.add_argument('--random', type=int, default=20000)
    parser.add_argument('--seed', type=int, default=24)
    options = parser.parse_args()

    mks = load(options.mks_can)
    cases = list(requests(mks, options.random, random.Random(options.seed)))
    result = subprocess.run([options.tool], input='\n'.join(cases) + '\n', capture_output=True,
                            text=True, check=True)
    answers = result.stdout.splitlines()
    if len(answers) != len(cases):
        sys.exit(f'tool answered {len(answers)} of {len(cases)} requests')
    errors = 0
    for case, answer in zip(cases, answers):
        expected = reference(mks, case)
        if answer != expected:
            errors += 1
            if errors <= 10:
                print(f'MISMATCH {case!r}: C++ {answer!r}, mks_can.py {expected!r}')
    accepted = sum(answer != 'ERR' for answer in answers)
    print(f'{len(cases)} cases ({accepted} valid frames/values, {len(cases) - accepted} rejected '
          f'by both): {errors} mismatches')
    sys.exit(1 if errors else 0)


if __name__ == '__main__':
    main()
