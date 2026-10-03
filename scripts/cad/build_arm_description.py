#!/usr/bin/env python3
"""Generate the WayBionic arm URDF and COLLADA meshes from a SolidWorks export.

1. On Windows with SolidWorks, build export_solidworks_assembly.cs (see its
   header) and export the assembly. The Sep 12 stepper casing is missing from
   the CAD upload; the Sep 8 part fits all of its mates, so substitute it:
     export_solidworks_assembly.exe full-arm-smaller.SLDASM build/cad/arm_export.json
       --substitute "diff_assem_sep18-1/StepperHolderCasing-Sep12 (1)-1=j4/StepperHolderCasing-Sep8.SLDPRT"
2. With any Python 3.8+ (standard library only):
     python3 scripts/cad/build_arm_description.py build/cad/arm_export.json
   This rewrites waybionic_description/urdf/waybionic_arm.urdf and
   waybionic_description/meshes/arm/*.dae.
3. Optional SolidWorks motion check: run this script with --write-moves
   moves.json, export again with --moves moves.json --no-geometry, then run this
   script on the full export with --check-moves <new export>. SolidWorks
   only carries the differential when the casing resolves, so until the Sep 12
   file exists, copy the Sep 8 part to that name in a scratch CAD copy and add
   --unsuppress "diff_assem_sep18-1/StepperHolderCasing-Sep12 (1)-1".

Joint axes come from the named mates below. Joint zero is the upright pose used
by the mechanical team's fk.py (links along +Z, pitch about +Y); the saved CAD
pose is reported as joint values. Limits are provisional until hardware sets them.
"""

import argparse
import array
import json
import math
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]
URDF_PATH = REPOSITORY / 'waybionic_description' / 'urdf' / 'waybionic_arm.urdf'
MESH_DIRECTORY = REPOSITORY / 'waybionic_description' / 'meshes' / 'arm'
MESH_URI = 'package://waybionic_description/meshes/arm/'

# Top-level components, including everything below them, for each rigid link.
LINKS = {
    'base_link': (
        'bottom-base-assembly-1', 'stepper-base-1', 'sweep_gearbox_redone-1',
        'heatSetInsertm3-1', 'heatSetInsertm3-2', 'heatSetInsertm3-3', 'heatSetInsertm3-4',
        'm3-20long-1'),
    'shoulder_link': ('outer_ring_sweep_loose-1', 'nema23_w_shoulder-1', 'j2_nema23_gearbox-2'),
    'upper_arm_link': (
        'outer-ring-Nema23-1', 'bottom-j2-nema23-v2_wHole-1', 'j2shouldersplit-1',
        'stepper_j2_sweep-3', 'Stepper-2', 'sweep_gearbox_redone-3'),
    'forearm_link': (
        'outer_ring_sweep_loose-2', '3rd_joint_bend-1', 'heatSetInsertm3-7', 'm3-12long-6',
        'diff_assem_sep18-1'),
    'wrist_pitch_link': ('DifferentialHousing-1',),
    'wrist_left_gear_link': ('straight bevel pinion_iso-4', 'BevelGearShaftV2-3', 'GT2 60T b5 (6mm)-2'),
    'wrist_right_gear_link': ('straight bevel pinion_iso-2', 'BevelGearShaftV2-4', 'GT2 60T b5 (6mm)-1'),
    'wrist_roll_link': ('straight bevel pinion_iso-3', 'Biomed Lock Mechanism_V3_HT_20260815-1'),
}

# name, parent, child, axis mate, origin plane mate, SolidWorks rotor, limits in degrees.
JOINTS = (
    ('joint_1', 'base_link', 'shoulder_link', 'Concentric124', 'Coincident70',
     'outer_ring_sweep_loose-1', (-180.0, 180.0)),
    ('joint_2', 'shoulder_link', 'upper_arm_link', 'Concentric112', 'Coincident57',
     'outer-ring-Nema23-1', (-90.0, 90.0)),
    ('joint_3', 'upper_arm_link', 'forearm_link', 'Concentric127', 'Coincident71',
     'outer_ring_sweep_loose-2', (-90.0, 90.0)),
    ('joint_4', 'forearm_link', 'wrist_pitch_link', 'Concentric139', None,
     'DifferentialHousing-1', (-90.0, 90.0)),
    ('joint_5', 'wrist_pitch_link', 'wrist_roll_link', 'Concentric143', None,
     'straight bevel pinion_iso-3', (-90.0, 90.0)),
)
DESCRIPTIONS = {
    'joint_1': 'base yaw', 'joint_2': 'shoulder pitch', 'joint_3': 'elbow pitch',
    'joint_4': 'wrist pitch (differential housing)', 'joint_5': 'wrist roll (output bevel)',
}
GEAR_LINKS = ('wrist_left_gear_link', 'wrist_right_gear_link')
BEVELS = ('straight bevel pinion_iso-2', 'straight bevel pinion_iso-3', 'straight bevel pinion_iso-4')
# Threaded screws and the enclosed cycloidal discs dominate the triangle count.
COARSE_PARTS = ('m3-', 'discs-sweep', 'loose_disc')
COARSE_GRID = 0.0012

# SolidWorks is Y-up. base_link: X = SolidWorks -Z (arm forward), Y = -X, Z = +Y.
SOLIDWORKS_TO_BASE = ((0.0, 0.0, -1.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0))


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(a, k):
    return tuple(x * k for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    length = norm(a)
    if length < 1e-12:
        raise ValueError('Zero-length vector')
    return scale(a, 1.0 / length)


def reject(a, axis):
    return sub(a, scale(axis, dot(a, axis)))


def degrees_between(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, abs(dot(unit(a), unit(b)))))))


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def apply(m, v):
    return tuple(dot(row, v) for row in m)


def columns(x, y, z):
    return tuple((x[i], y[i], z[i]) for i in range(3))


class Pose:
    """Rigid transform p' = rotation * p + origin."""

    def __init__(self, rotation, origin):
        self.rotation = tuple(tuple(row) for row in rotation)
        self.origin = tuple(origin)

    def __mul__(self, other):
        return Pose(matmul(self.rotation, other.rotation),
                    add(apply(self.rotation, other.origin), self.origin))

    def inverse(self):
        rotation = transpose(self.rotation)
        return Pose(rotation, scale(apply(rotation, self.origin), -1.0))

    def point(self, p):
        return add(apply(self.rotation, p), self.origin)

    def vector(self, v):
        return apply(self.rotation, v)


def rotation_about(axis, angle):
    x, y, z = unit(axis)
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return ((t * x * x + c, t * x * y - s * z, t * x * z + s * y),
            (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
            (t * x * z - s * y, t * y * z + s * x, t * z * z + c))


def rotation_angle(rotation):
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    return math.degrees(math.acos(max(-1.0, min(1.0, 0.5 * (trace - 1.0)))))


def rotation_angle_about(rotation, axis):
    """Twist angle about axis (radians) and the remaining swing (degrees)."""
    m = rotation
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w, v = 0.25 * s, ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2])
        w, v = (m[2][1] - m[1][2]) / s, (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    elif m[1][1] > m[2][2]:
        s = 2.0 * math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2])
        w, v = (m[0][2] - m[2][0]) / s, ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1])
        w, v = (m[1][0] - m[0][1]) / s, ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)
    angle = math.remainder(2.0 * math.atan2(dot(v, axis), w), 2.0 * math.pi)
    return angle, rotation_angle(matmul(transpose(rotation_about(axis, angle)), rotation))


def roll_pitch_yaw(rotation):
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    roll = math.atan2(rotation[2][1], rotation[2][2])
    yaw = math.atan2(rotation[1][0], rotation[0][0])
    return roll, pitch, yaw


def solidworks_pose(data):
    """Component transform from SolidWorks row-vector MathTransform data."""
    if abs(data[12] - 1.0) > 1e-9:
        raise ValueError('Scaled component transforms are not supported')
    rotation = tuple(tuple(data[3 * j + i] for j in range(3)) for i in range(3))
    return Pose(rotation, data[9:12])


def fmt(value, digits=6):
    text = f'{value:.{digits}f}'.rstrip('0').rstrip('.')
    return '0' if text in ('-0', '') else text


def vector_text(values, digits=6):
    return ' '.join(fmt(value, digits) for value in values)


class Export:
    """SolidWorks export JSON plus its float32/int32 geometry buffer."""

    def __init__(self, path):
        self.path = Path(path)
        self.data = json.loads(self.path.read_text(encoding='utf-8'))
        self.blob = self.path.with_suffix('.bin').read_bytes() if self.path.with_suffix('.bin').exists() else b''
        self.components = {item['name']: item for item in self.data['components']}
        self.mates = {(item['owner'], item['name']): item for item in self.data['mates']}
        self.palette = self.data.get('palette', [])

    def face_arrays(self, face):
        vertices = array.array('f')
        vertices.frombytes(self.blob[face['v']:face['v'] + 12 * face['nv']])
        indices = array.array('i')
        indices.frombytes(self.blob[face['t']:face['t'] + 12 * face['nt']])
        return vertices, indices

    def mate_line(self, name, owner=''):
        mate = self.mates[(owner, name)]
        first, second = (entity['parameters'] for entity in mate['entities'][:2])
        point, direction = tuple(first[:3]), unit(first[3:6])
        other_point, other_direction = tuple(second[:3]), unit(second[3:6])
        offset = norm(reject(sub(other_point, point), direction))
        if degrees_between(direction, other_direction) > 0.05 or offset > 5e-5:
            raise ValueError(f'{name} is not satisfied in the saved pose')
        return point, direction

    def mate_plane(self, name, owner=''):
        first = self.mates[(owner, name)]['entities'][0]['parameters']
        return tuple(first[:3]), unit(first[3:6])


def owner_link(name):
    top = name.split('/')[0]
    matches = [link for link, members in LINKS.items() if top in members]
    return matches[0] if len(matches) == 1 else None


def resolved_color(export, component, face):
    for index in (component.get('componentColor', -1), face.get('faceColor', -1),
                  face.get('featureColor', -1), face.get('bodyColor', -1),
                  component.get('partColor', -1)):
        if index is not None and index >= 0:
            return tuple(round(value, 4) for value in export.palette[index][:3])
    return (0.792, 0.82, 0.933)


def box_corners(box):
    return [(box[3 * (i & 1)], box[1 + 3 * ((i >> 1) & 1)], box[2 + 3 * (i >> 2)]) for i in range(8)]


def check_components(export):
    """Every visible part is assigned to exactly one link; placements are consistent."""
    assigned = {link: [] for link in LINKS}
    worst_part = 0.0
    placement = []
    for name, component in export.components.items():
        if not component.get('faces'):
            continue
        link = owner_link(name)
        if link is None:
            raise ValueError(f'Component is not assigned to one link: {name}')
        assigned[link].append(name)
        pose = solidworks_pose(component['transform'])
        points = [(v[i], v[i + 1], v[i + 2]) for face in component['faces']
                  for v in [export.face_arrays(face)[0]] for i in range(0, len(v), 3)]
        low = [min(p[axis] for p in points) for axis in range(3)]
        high = [max(p[axis] for p in points) for axis in range(3)]
        part_box = component['partBox']
        worst_part = max(worst_part, max(abs(a - b) for a, b in zip(low + high, part_box)))
        if component.get('box') and not component.get('substitute'):
            placed = [pose.point(corner) for corner in box_corners(part_box)]
            placed_box = [min(p[a] for p in placed) for a in range(3)] + [max(p[a] for p in placed) for a in range(3)]
            placement.append(max(abs(a - b) for a, b in zip(placed_box, component['box'])))
    placement.sort()
    # SolidWorks boxes are transformed part boxes; a few include non-solid bodies.
    if worst_part > 1e-3 or placement[len(placement) // 2] > 1e-4 or placement[-1] > 1e-2:
        raise ValueError(f'Geometry is not in part coordinates or transforms differ: '
                         f'{worst_part}, {placement[len(placement) // 2]}, {placement[-1]}')
    for top in {member for members in LINKS.values() for member in members}:
        if not any(name == top or name.startswith(top + '/') for name in export.components):
            raise ValueError(f'Link member missing from export: {top}')
    return assigned


def check_substitutes(export):
    """A substituted part must fit every cached mate on the component it replaces."""
    report = []
    for name, component in export.components.items():
        if not component.get('substitute'):
            continue
        pose = solidworks_pose(component['transform'])
        cylinders = [(pose.point(face['cylinder'][:3]), unit(pose.vector(face['cylinder'][3:6])),
                      face['cylinder'][6]) for face in component['faces'] if face.get('cylinder')]
        for (owner, mate_name), mate in export.mates.items():
            owner_pose = solidworks_pose(export.components[owner]['transform']) if owner else None
            prefix = owner + '/' if owner else ''
            for entity in mate['entities']:
                if entity['component'] is None or prefix + entity['component'] != name:
                    continue
                values = entity['parameters']
                if mate['type'] != 1 or len(values) < 7:
                    continue
                point, direction = tuple(values[:3]), unit(values[3:6])
                if owner_pose:
                    point, direction = owner_pose.point(point), owner_pose.vector(direction)
                errors = [(norm(reject(sub(center, point), direction)), degrees_between(axis, direction),
                           abs(radius - values[6])) for center, axis, radius in cylinders]
                best = min(errors, key=lambda e: e[0] * 1e3 + e[1] + e[2] * 1e3)
                if best[0] > 1e-4 or best[1] > 0.1 or best[2] > 2e-4:
                    raise ValueError(f'{Path(component["substitute"]).name} does not fit {mate_name}: {best}')
                report.append((mate_name, owner or 'top', best))
    return report


class Model:
    """Link frames at the saved CAD pose, all expressed in base_link."""

    def __init__(self, export, assigned):
        self.export = export
        self.assigned = assigned
        self.to_base = self._base_transform()
        lines = {}
        origins = {}
        for name, _, _, axis_mate, plane_mate, _, _ in JOINTS:
            point, direction = export.mate_line(axis_mate)
            point, direction = self.to_base.point(point), self.to_base.vector(direction)
            if plane_mate:
                plane_point, normal = export.mate_plane(plane_mate)
                plane_point, normal = self.to_base.point(plane_point), self.to_base.vector(normal)
                point = add(point, scale(direction, dot(sub(plane_point, point), normal) / dot(direction, normal)))
            lines[name] = (point, direction)
            origins[name] = point
        (p4, a4), (p5, a5) = lines['joint_4'], lines['joint_5']
        normal = cross(a4, a5)
        s = dot(cross(sub(p5, p4), a5), normal) / dot(normal, normal)
        t = dot(cross(sub(p5, p4), a4), normal) / dot(normal, normal)
        closest4, closest5 = add(p4, scale(a4, s)), add(p5, scale(a5, t))
        self.wrist_gap = norm(sub(closest4, closest5))
        self.wrist_angle = 90.0 - degrees_between(a4, a5)
        if self.wrist_gap > 1e-4 or abs(self.wrist_angle) > 0.1:
            raise ValueError('Wrist pitch and roll axes must intersect at a right angle')
        center = scale(add(closest4, closest5), 0.5)
        origins['joint_4'] = origins['joint_5'] = center

        a1 = unit(lines['joint_1'][1])
        a1 = a1 if a1[2] > 0 else scale(a1, -1.0)
        forward = unit(cross(lines['joint_2'][1], a1))
        forward = forward if forward[0] > 0 else scale(forward, -1.0)
        lateral = cross(a1, forward)
        signed = {name: (lambda d: d if dot(d, lateral) > 0 else scale(d, -1.0))(unit(lines[name][1]))
                  for name in ('joint_2', 'joint_3', 'joint_4')}
        roll_center = self.component_center('straight bevel pinion_iso-3')
        roll = unit(lines['joint_5'][1])
        roll = roll if dot(roll, sub(roll_center, center)) > 0 else scale(roll, -1.0)

        def frame(origin, y_axis, z_hint):
            z_axis = unit(reject(z_hint, y_axis))
            return Pose(columns(cross(y_axis, z_axis), y_axis, z_axis), origin)

        self.frames = {'base_link': Pose(((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0, 0, 0))}
        self.frames['shoulder_link'] = Pose(columns(forward, lateral, a1), origins['joint_1'])
        self.frames['upper_arm_link'] = frame(origins['joint_2'], signed['joint_2'],
                                              sub(origins['joint_3'], origins['joint_2']))
        self.frames['forearm_link'] = frame(origins['joint_3'], signed['joint_3'],
                                            sub(center, origins['joint_3']))
        self.frames['wrist_pitch_link'] = frame(center, signed['joint_4'], roll)
        for link in GEAR_LINKS + ('wrist_roll_link',):
            self.frames[link] = self.frames['wrist_pitch_link']
        self.axis_errors = {
            'joint_1': degrees_between(a1, (0, 0, 1)),
            'joint_2': degrees_between(signed['joint_2'], lateral),
            'joint_3': degrees_between(signed['joint_3'], signed['joint_2']),
            'joint_4': degrees_between(signed['joint_4'], signed['joint_2']),
            'joint_5': abs(self.wrist_angle),
        }
        self.cad_angles = {}
        self.joints = []
        for name, parent, child, _, _, rotor, limits in JOINTS:
            axis = (0.0, 0.0, 1.0) if name in ('joint_1', 'joint_5') else (0.0, 1.0, 0.0)
            self._add_joint(name, parent, child, axis, limits, rotor)
        self.gear_multipliers = {}
        for link in GEAR_LINKS:
            side = dot(sub(self.link_center(link), center), signed['joint_4'])
            expected = 'left' if side > 0 else 'right'
            if expected not in link:
                raise ValueError(f'{link} is on the {expected} side of the wrist')
            self.gear_multipliers[link] = self._gear_multiplier(link, center, roll, signed['joint_4'])
            self._add_joint(link.replace('_link', '_joint'), 'wrist_pitch_link', link,
                            (0.0, 1.0, 0.0), None, None, mimic=('joint_5', self.gear_multipliers[link]))
        self.tool_length = max(dot(sub(p, center), roll) for p in self.link_points('wrist_roll_link'))

    def _base_transform(self):
        rotation = SOLIDWORKS_TO_BASE
        base = Pose(rotation, (0.0, 0.0, 0.0))
        axis_point, _ = self.export.mate_line(JOINTS[0][3])
        low = min(p[2] for p in self.link_points('base_link', base))
        start = base.point(axis_point)
        return Pose(rotation, (-start[0], -start[1], -low))

    def component_points(self, name, to_base=None):
        component = self.export.components[name]
        pose = (to_base or self.to_base) * solidworks_pose(component['transform'])
        for face in component['faces']:
            vertices, _ = self.export.face_arrays(face)
            for i in range(0, len(vertices), 3):
                yield pose.point((vertices[i], vertices[i + 1], vertices[i + 2]))

    def component_center(self, name):
        points = list(self.component_points(name))
        return tuple((min(p[a] for p in points) + max(p[a] for p in points)) / 2 for a in range(3))

    def link_points(self, link, to_base=None):
        for name in self.assigned[link]:
            yield from self.component_points(name, to_base)

    def link_center(self, link):
        points = list(self.link_points(link))
        return tuple((min(p[a] for p in points) + max(p[a] for p in points)) / 2 for a in range(3))

    def _add_joint(self, name, parent, child, axis, limits, rotor, mimic=None):
        parent_frame, child_frame = self.frames[parent], self.frames[child]
        if name == 'joint_1':
            x_axis = child_frame.rotation[0][0], child_frame.rotation[1][0], child_frame.rotation[2][0]
            angle = math.atan2(x_axis[1], x_axis[0])
        elif name in ('joint_2', 'joint_3', 'joint_4'):
            z_parent = tuple(parent_frame.rotation[i][2] for i in range(3))
            z_child = tuple(child_frame.rotation[i][2] for i in range(3))
            y_child = tuple(child_frame.rotation[i][1] for i in range(3))
            angle = math.atan2(dot(cross(z_parent, z_child), y_child), dot(z_parent, z_child))
        else:
            angle = 0.0
        origin = parent_frame.inverse() * child_frame * Pose(rotation_about(axis, -angle), (0, 0, 0))
        rpy = roll_pitch_yaw(origin.rotation)
        if max(abs(value) for value in rpy) < 1e-3:
            rpy = (0.0, 0.0, 0.0)
        self.cad_angles[name] = angle
        self.joints.append({'name': name, 'parent': parent, 'child': child, 'axis': axis,
                            'xyz': origin.origin, 'rpy': rpy, 'limits': limits, 'rotor': rotor,
                            'mimic': mimic})

    def _gear_multiplier(self, link, center, roll, pitch):
        """Rolling contact of equal bevels: side gear turn per output-bevel turn."""
        configurations = {self.export.components[name]['configuration'] for name in BEVELS}
        if len(configurations) != 1:
            raise ValueError('Differential bevels differ; update the gear ratio')
        side = unit(scale(pitch, 1.0 if dot(sub(self.link_center(link), center), pitch) > 0 else -1.0))
        contact = unit(add(roll, side))
        gear_speed = cross(pitch, contact)
        return round(dot(cross(roll, contact), gear_speed) / dot(gear_speed, gear_speed), 6)

    def world_frame(self, link):
        return self.to_base.inverse() * self.frames[link]


def quantized_link_mesh(model, link, grid):
    """Triangles per color in link coordinates, snapped to a grid with per-face normals."""
    export = model.export
    groups = {}
    to_link = model.frames[link].inverse() * model.to_base
    stats = {'input': 0, 'output': 0, 'bodies': 0, 'flipped': 0, 'conflicts': 0}
    for name in sorted(model.assigned[link]):
        component = export.components[name]
        pose = to_link * solidworks_pose(component['transform'])
        cell = COARSE_GRID if name.split('/')[-1].startswith(COARSE_PARTS) else grid
        faces_by_body = {}
        for face in component['faces']:
            faces_by_body.setdefault(face['body'], []).append(face)
        for faces in faces_by_body.values():
            arrays = [export.face_arrays(face) for face in faces]
            volume = 0.0
            directed = {}
            for vertices, indices in arrays:
                for i in range(0, len(indices), 3):
                    a, b, c = (tuple(vertices[3 * k:3 * k + 3]) for k in indices[i:i + 3])
                    volume += dot(a, cross(b, c))
                    for edge in ((a, b), (b, c), (c, a)):
                        directed[edge] = directed.get(edge, 0) + 1
            flip = volume < 0
            stats['bodies'] += 1
            stats['flipped'] += flip
            stats['conflicts'] += sum(1 for count in directed.values() if count > 1)
            for face, (vertices, indices) in zip(faces, arrays):
                stats['input'] += len(indices) // 3
                color = resolved_color(export, component, face)
                group = groups.setdefault(color, {'positions': [], 'normals': [], 'triangles': []})
                local = {}
                corners = []
                for k in range(0, len(vertices), 3):
                    p = pose.point((vertices[k], vertices[k + 1], vertices[k + 2]))
                    key = tuple(round(value / cell) for value in p)
                    corners.append(local.setdefault(key, len(local)))
                keys = [None] * len(local)
                for key, index in local.items():
                    keys[index] = scale(key, cell)
                normals = [(0.0, 0.0, 0.0)] * len(keys)
                triangles = []
                for i in range(0, len(indices), 3):
                    a, b, c = (corners[k] for k in indices[i:i + 3])
                    if flip:
                        b, c = c, b
                    if a == b or b == c or a == c:
                        continue
                    weighted = cross(sub(keys[b], keys[a]), sub(keys[c], keys[a]))
                    if norm(weighted) < 1e-14:
                        continue
                    for k in (a, b, c):
                        normals[k] = add(normals[k], weighted)
                    triangles.append((a, b, c))
                used = sorted({k for triangle in triangles for k in triangle})
                remap = {old: len(group['positions']) + new for new, old in enumerate(used)}
                for old in used:
                    length = norm(normals[old])
                    group['positions'].append(keys[old])
                    group['normals'].append(scale(normals[old], 1.0 / length) if length > 1e-15 else (0.0, 0.0, 1.0))
                group['triangles'].extend(tuple(remap[k] for k in triangle) for triangle in triangles)
                stats['output'] += len(triangles)
    return groups, stats


def write_collada(path, link, groups):
    materials = sorted(groups)
    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">',
        '  <asset><contributor><authoring_tool>scripts/cad/build_arm_description.py</authoring_tool>'
        '</contributor><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>',
        '  <library_effects>']
    for index, color in enumerate(materials):
        lines.append(
            f'    <effect id="m{index}-fx"><profile_COMMON><technique sid="common"><phong>'
            f'<diffuse><color>{vector_text(color, 4)} 1</color></diffuse>'
            '<specular><color>0.25 0.25 0.25 1</color></specular><shininess><float>30</float></shininess>'
            '</phong></technique></profile_COMMON></effect>')
    lines.append('  </library_effects>')
    lines.append('  <library_materials>')
    for index, _ in enumerate(materials):
        lines.append(f'    <material id="m{index}"><instance_effect url="#m{index}-fx"/></material>')
    lines.append('  </library_materials>')
    positions, normals, blocks, offset = [], [], [], 0
    for index, color in enumerate(materials):
        group = groups[color]
        positions.extend(group['positions'])
        normals.extend(group['normals'])
        indices = ' '.join(str(offset + k) for triangle in group['triangles'] for k in triangle)
        blocks.append(f'        <triangles material="m{index}" count="{len(group["triangles"])}">'
                      f'<input semantic="VERTEX" source="#v" offset="0"/><p>{indices}</p></triangles>')
        offset += len(group['positions'])
    position_text = ' '.join(vector_text(p, 4) for p in positions)
    normal_text = ' '.join(vector_text(n, 2) for n in normals)
    count = len(positions)
    lines += [
        '  <library_geometries>',
        f'    <geometry id="g" name="{link}"><mesh>',
        f'        <source id="p"><float_array id="pa" count="{3 * count}">{position_text}</float_array>'
        f'<technique_common><accessor source="#pa" count="{count}" stride="3"><param name="X" type="float"/>'
        '<param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>',
        f'        <source id="n"><float_array id="na" count="{3 * count}">{normal_text}</float_array>'
        f'<technique_common><accessor source="#na" count="{count}" stride="3"><param name="X" type="float"/>'
        '<param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>',
        '        <vertices id="v"><input semantic="POSITION" source="#p"/><input semantic="NORMAL" source="#n"/></vertices>',
        *blocks,
        '    </mesh></geometry>',
        '  </library_geometries>',
        '  <library_visual_scenes><visual_scene id="s">',
        f'    <node id="{link}" name="{link}"><instance_geometry url="#g"><bind_material><technique_common>',
        *[f'      <instance_material symbol="m{index}" target="#m{index}"/>' for index, _ in enumerate(materials)],
        '    </technique_common></bind_material></instance_geometry></node>',
        '  </visual_scene></library_visual_scenes>',
        '  <scene><instance_visual_scene url="#s"/></scene>',
        '</COLLADA>',
        '']
    path.write_text('\n'.join(lines), encoding='utf-8')


def write_urdf(model, source):
    angles = ', '.join(f'{name}={math.degrees(model.cad_angles[name]):.2f}' for name, *_ in JOINTS)
    lines = [
        '<?xml version="1.0"?>',
        f'<!-- Generated by scripts/cad/build_arm_description.py from {source}.',
        '     Do not edit by hand; re-export the SolidWorks assembly and re-run the script.',
        '     Joint zero is the upright pose from the mechanical fk.py (links along +Z,',
        f'     pitch about +Y). Saved CAD pose in degrees {angles}.',
        '     Limits are provisional; effort and velocity are placeholders for display. -->',
        '<robot name="waybionic_arm">']
    for link in LINKS:
        lines += [
            f'  <link name="{link}">',
            '    <visual>',
            f'      <geometry><mesh filename="{MESH_URI}{link}.dae"/></geometry>',
            '    </visual>',
            '  </link>']
    lines += ['  <link name="tool_link"/>']
    for joint in model.joints:
        kind = 'continuous' if joint['mimic'] else 'revolute'
        note = DESCRIPTIONS.get(joint['name'], 'differential side gear, follows the output bevel')
        lines += [
            f'  <!-- {note} -->',
            f'  <joint name="{joint["name"]}" type="{kind}">',
            f'    <parent link="{joint["parent"]}"/>',
            f'    <child link="{joint["child"]}"/>',
            f'    <origin xyz="{vector_text(joint["xyz"])}" rpy="{vector_text(joint["rpy"])}"/>',
            f'    <axis xyz="{vector_text(joint["axis"])}"/>']
        if joint['limits']:
            lower, upper = (math.radians(value) for value in joint['limits'])
            lines.append(f'    <limit lower="{fmt(lower)}" upper="{fmt(upper)}" effort="0" velocity="0"/>')
        if joint['mimic']:
            lines.append(f'    <mimic joint="{joint["mimic"][0]}" multiplier="{fmt(joint["mimic"][1])}" offset="0"/>')
        lines.append('  </joint>')
    lines += [
        '  <joint name="tool_joint" type="fixed">',
        '    <parent link="wrist_roll_link"/>',
        '    <child link="tool_link"/>',
        f'    <origin xyz="0 0 {fmt(model.tool_length)}" rpy="0 0 0"/>',
        '  </joint>',
        '</robot>',
        '']
    URDF_PATH.write_text('\n'.join(lines), encoding='utf-8')


def forward_kinematics(model, angles):
    """Link poses in base_link for joint values, using the generated joint table."""
    poses = {'base_link': model.frames['base_link']}
    for joint in model.joints:
        value = angles.get(joint['name'], 0.0)
        if joint['mimic']:
            value = joint['mimic'][1] * angles.get(joint['mimic'][0], 0.0)
        r, p, y = joint['rpy']
        rotation = matmul(rotation_about((0, 0, 1), y), matmul(rotation_about((0, 1, 0), p), rotation_about((1, 0, 0), r)))
        poses[joint['child']] = (poses[joint['parent']] * Pose(rotation, joint['xyz'])
                                 * Pose(rotation_about(joint['axis'], value), (0, 0, 0)))
    return poses


def check_cad_pose(model):
    """The joint table at the saved CAD angles must reproduce every link frame."""
    poses = forward_kinematics(model, model.cad_angles)
    worst = 0.0
    for link, frame in model.frames.items():
        pose = poses[link]
        for corner in ((0.2, 0, 0), (0, 0.2, 0), (0, 0, 0.2), (0, 0, 0)):
            worst = max(worst, norm(sub(pose.point(corner), frame.point(corner))))
    return worst


def write_moves(model, path, degrees=30.0):
    # Minimum-movement drags; SetTransformAndSolve leaves base yaw in place.
    moves = []
    for joint in model.joints:
        if not joint['rotor']:
            continue
        frame = model.world_frame(joint['child'])
        moves.append({'name': joint['name'], 'component': joint['rotor'], 'point': list(frame.origin),
                      'axis': list(frame.vector(joint['axis'])), 'angle': math.radians(degrees),
                      'method': 'drag:0:0', 'steps': 30})
    Path(path).write_text(json.dumps(moves, indent=2), encoding='utf-8')
    print(f'Wrote {len(moves)} SolidWorks moves to {path}')


def check_moves(model, path):
    """Compare SolidWorks-solved poses with rigid links and URDF joint axes."""
    moved = json.loads(Path(path).read_text(encoding='utf-8'))
    original = {name: solidworks_pose(item['transform']) for name, item in model.export.components.items()
                if item.get('faces') and not item.get('substitute')}
    failures = []
    for move in moved['moves']:
        transforms = move['transforms']
        delta = {name: solidworks_pose(transforms[name]) * pose.inverse() for name, pose in original.items()}
        link_delta = {}
        notes = []
        rotors = {joint['child']: joint['rotor'] for joint in model.joints if joint['rotor']}
        rotors.update({'wrist_left_gear_link': BEVELS[2], 'wrist_right_gear_link': BEVELS[0]})
        for link, members in model.assigned.items():
            names = sorted(name for name in members if name in delta)
            reference = rotors.get(link, names[0])
            link_delta[link] = delta[reference]
            for name in names:
                worst = max(norm(sub(delta[name].point(p), delta[reference].point(p)))
                            for p in model.corner_cache[name])
                if worst > 1e-4:
                    notes.append(f'{name} moves {worst * 1000:.2f} mm off {link}')
        values = {}
        for joint in model.joints:
            parent, child = joint['parent'], joint['child']
            frame = model.world_frame(child)
            relative = frame.inverse() * link_delta[parent].inverse() * link_delta[child] * frame
            angle, residual = rotation_angle_about(relative.rotation, joint['axis'])
            shift = norm(relative.origin)
            values[joint['name']] = angle
            if residual > 0.05 or shift > 1e-4:
                failures.append(f'{move["name"]}: {child} is not a pure rotation about {joint["name"]} '
                                f'({residual:.3f} deg, {shift * 1000:.3f} mm)')
        expected = math.degrees(move['angle'])
        measured = math.degrees(values[move['name']])
        status = 'solved' if move['solved'] else 'NOT SOLVED'
        others = ', '.join(f'{name} {math.degrees(value):+.2f}' for name, value in values.items()
                           if name != move['name'] and abs(value) > 1e-4)
        print(f'  {move["name"]}: SolidWorks {status}; rotor {measured:+.2f} deg (asked {expected:+.2f})'
              + (f'; also {others}' if others else ''))
        for note in notes:
            print(f'      {note}')
        if abs(measured - expected) > 0.05:
            failures.append(f'{move["name"]}: rotor turned {measured:.3f} deg, expected {expected:.3f}')
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('export', help='JSON from export_solidworks_assembly.exe')
    parser.add_argument('--grid', type=float, default=0.0005, help='mesh vertex grid in metres')
    parser.add_argument('--write-moves', help='write SolidWorks joint moves and exit')
    parser.add_argument('--check-moves', help='export made with --moves, compared with the full export')
    arguments = parser.parse_args()

    export = Export(arguments.export)
    assigned = check_components(export)
    fits = check_substitutes(export)
    model = Model(export, assigned)
    source = Path(export.data['assembly'].replace('\\', '/')).name
    print(f'Source: {source} ({export.data["configuration"]}, SolidWorks {export.data["solidworks"]})')
    for mate_name, owner, (offset, angle, radius) in fits:
        print(f'  substitute fits {owner}:{mate_name} (axis {offset * 1000:.3f} mm, {angle:.3f} deg, '
              f'radius {radius * 1000:.3f} mm)')
    print(f'Wrist axes: gap {model.wrist_gap * 1000:.3f} mm, {90 - model.wrist_angle:.3f} deg apart')
    for joint in model.joints:
        cad = math.degrees(model.cad_angles[joint['name']])
        error = model.axis_errors.get(joint['name'])
        print(f'  {joint["name"]:<24} xyz=({vector_text(joint["xyz"], 4)}) rpy=({vector_text(joint["rpy"], 4)}) '
              f'cad={cad:+.2f} deg' + (f' axis err={error:.4f} deg' if error is not None else '')
              + (f' mimic {joint["mimic"]}' if joint['mimic'] else ''))
    print(f'  tool_link at {model.tool_length * 1000:.1f} mm along the roll axis')
    print(f'CAD-pose reproduction error: {check_cad_pose(model) * 1e6:.3f} um')
    wrist = model.frames['wrist_pitch_link']
    print('CAD pose (rad): ' + ', '.join(f'{name}={model.cad_angles[name]:.6f}' for name, *_ in JOINTS))
    print(f'SolidWorks reference in base_link: forearm_link origin '
          f'({vector_text(model.frames["forearm_link"].origin)}), wrist center ({vector_text(wrist.origin)}), '
          f'roll axis ({vector_text(wrist.vector((0, 0, 1)))})')

    if arguments.write_moves:
        write_moves(model, arguments.write_moves)
        return 0
    if arguments.check_moves:
        model.corner_cache = {
            name: [solidworks_pose(component['transform']).point(corner)
                   for corner in box_corners(component['partBox'])]
            for name, component in export.components.items()
            if component.get('faces') and not component.get('substitute')}
        failures = check_moves(model, arguments.check_moves)
        for failure in failures:
            print('FAIL ' + failure)
        return 1 if failures else 0

    MESH_DIRECTORY.mkdir(parents=True, exist_ok=True)
    total_in = total_out = 0
    for link in LINKS:
        groups, stats = quantized_link_mesh(model, link, arguments.grid)
        path = MESH_DIRECTORY / f'{link}.dae'
        write_collada(path, link, groups)
        total_in += stats['input']
        total_out += stats['output']
        print(f'  {path.name}: {stats["input"]} -> {stats["output"]} triangles, {len(groups)} colors, '
              f'{path.stat().st_size / 1e6:.2f} MB, {stats["flipped"]}/{stats["bodies"]} bodies flipped, '
              f'{stats["conflicts"]} winding conflicts')
    write_urdf(model, source)
    print(f'Wrote {URDF_PATH.relative_to(REPOSITORY)} ({total_in} -> {total_out} triangles)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
