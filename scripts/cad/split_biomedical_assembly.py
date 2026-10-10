"""Split Julie's assembled clamp STL into fixed geometry and two moving claws.

Usage: python3 scripts/cad/split_biomedical_assembly.py ASSEMBLY.stl OUTPUT_DIR
The output keeps the assembly's coordinates in millimetres. No guessed part
transforms are introduced. Julie's current export has eight closed components.
"""

import argparse
from collections import defaultdict
from pathlib import Path
import struct


def triangles(path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError('STL is too short')
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError('expected a binary STL with complete triangles')
    result = []
    for index in range(count):
        record = data[84 + index * 50:84 + (index + 1) * 50]
        values = struct.unpack('<12fH', record)
        vertices = tuple(tuple(round(value, 5) for value in values[offset:offset + 3])
                         for offset in (3, 6, 9))
        result.append((vertices, record))
    return result


def components(faces):
    vertex_ids = {}
    parents = []

    def vertex_id(point):
        if point not in vertex_ids:
            vertex_ids[point] = len(parents)
            parents.append(len(parents))
        return vertex_ids[point]

    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for vertices, _ in faces:
        ids = [vertex_id(vertex) for vertex in vertices]
        for index in ids[1:]:
            parents[root(index)] = root(ids[0])

    groups = defaultdict(list)
    for vertices, record in faces:
        groups[root(vertex_ids[vertices[0]])].append((vertices, record))
    return list(groups.values())


def bounds(group):
    points = [point for vertices, _ in group for point in vertices]
    lower = tuple(min(point[axis] for point in points) for axis in range(3))
    upper = tuple(max(point[axis] for point in points) for axis in range(3))
    return lower, upper


def write_stl(path, groups):
    records = [record for group in groups for _, record in group]
    header = b'WayBionic Biomedical assembly component'.ljust(80, b' ')
    path.write_bytes(header + struct.pack('<I', len(records)) + b''.join(records))


def split(source, output_dir):
    groups = components(triangles(source))
    if len(groups) != 8:
        raise ValueError(f'expected eight assembly components, found {len(groups)}')
    claws = []
    fixed = []
    for group in groups:
        low, high = bounds(group)
        extent = tuple(high[axis] - low[axis] for axis in range(3))
        if (10 < extent[0] < 16 and 12 < extent[1] < 15
                and 25 < extent[2] < 28 and low[2] > 110):
            claws.append((0.5 * (low[0] + high[0]), group))
        else:
            fixed.append(group)
    if len(claws) != 2 or len(fixed) != 6:
        raise ValueError('could not identify exactly two separate claw components')
    claws.sort(key=lambda item: item[0])
    output_dir.mkdir(parents=True, exist_ok=True)
    write_stl(output_dir / 'assembly_static.stl', fixed)
    write_stl(output_dir / 'assembly_left_claw.stl', [claws[0][1]])
    write_stl(output_dir / 'assembly_right_claw.stl', [claws[1][1]])
    return [len(group) for group in fixed], [len(item[1]) for item in claws]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly', type=Path)
    parser.add_argument('output_dir', type=Path)
    args = parser.parse_args()
    fixed, claws = split(args.assembly, args.output_dir)
    print(f'Wrote six fixed parts ({sum(fixed)} faces) and two claws ({claws} faces)')


if __name__ == '__main__':
    main()
