"""
Keep the arm's links off the table and out of its own base (pure logic, no ROS).

Each link's collision boxes come from the URDF, and every check happens in the base frame.
The moving links must stay above the table, and the forearm, wrist and tool must stay out of
the base and shoulder, the parts they can fold into. Adjacent links overlap at their joints,
so those pairs are never compared.
"""

import math
import xml.etree.ElementTree as ET

# The fixed base and the yaw stage, and what may never touch them.
BODY = ('base_link', 'shoulder_link')
FOLDING = ('forearm_link', 'wrist_pitch_link', 'wrist_left_gear_link', 'wrist_right_gear_link',
           'wrist_roll_link')
REQUIRED = BODY + ('upper_arm_link',) + FOLDING


def _vector(element, name, default=(0.0, 0.0, 0.0)):
    text = element.get(name) if element is not None else None
    return tuple(float(value) for value in text.split()) if text else default


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _multiply(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def _apply(matrix, vector):
    return tuple(_dot(row, vector) for row in matrix)


def _rotation(axis, angle):
    x, y, z = axis
    c, s, t = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return ((t * x * x + c, t * x * y - s * z, t * x * z + s * y),
            (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
            (t * x * z - s * y, t * y * z + s * x, t * z * z + c))


def _rpy(roll, pitch, yaw):
    return _multiply(_rotation((0.0, 0.0, 1.0), yaw), _multiply(
        _rotation((0.0, 1.0, 0.0), pitch), _rotation((1.0, 0.0, 0.0), roll)))


class ArmCollision:
    """Collision boxes and joint tree read from a URDF."""

    def __init__(self, boxes, joints, root, table_z=0.0, clearance=0.01):
        self.boxes, self.joints, self.root = boxes, joints, root
        self.table_z, self.clearance = table_z, clearance

    @classmethod
    def from_urdf(cls, urdf, table_z=0.0, clearance=0.01):
        """Read boxes and joints; raise ValueError without boxes for the base and folding links."""
        try:
            robot = ET.fromstring(urdf)
        except ET.ParseError as error:
            raise ValueError(f'robot_description is not valid XML: {error}') from None
        boxes = {}
        for link in robot.findall('link'):
            for collision in link.findall('collision'):
                box = collision.find('geometry/box')
                if box is None:
                    continue
                origin = collision.find('origin')
                boxes.setdefault(link.get('name'), []).append((
                    _vector(origin, 'xyz'), _rpy(*_vector(origin, 'rpy')),
                    tuple(size / 2.0 for size in _vector(box, 'size'))))
        missing = [name for name in REQUIRED if name not in boxes]
        if missing:
            raise ValueError('robot_description has no collision box for ' + ', '.join(missing))
        joints, children = {}, set()
        for joint in robot.findall('joint'):
            mimic = joint.find('mimic')
            origin = joint.find('origin')
            joints[joint.find('child').get('link')] = {
                'name': joint.get('name'), 'type': joint.get('type'),
                'parent': joint.find('parent').get('link'),
                'xyz': _vector(origin, 'xyz'), 'rpy': _rpy(*_vector(origin, 'rpy')),
                'axis': _vector(joint.find('axis'), 'xyz', (1.0, 0.0, 0.0)),
                'mimic': None if mimic is None else (
                    mimic.get('joint'), float(mimic.get('multiplier', 1.0)),
                    float(mimic.get('offset', 0.0)))}
            children.add(joint.find('child').get('link'))
        roots = [link.get('name') for link in robot.findall('link')
                 if link.get('name') not in children]
        checker = cls(boxes, joints, roots[0], table_z, clearance)
        checker.poses({})
        return checker

    def poses(self, positions):
        """Return {link: (rotation, origin)} in the root frame for the joint positions."""
        poses = {self.root: (_rpy(0.0, 0.0, 0.0), (0.0, 0.0, 0.0))}
        pending = list(self.joints)
        while pending:
            ready = [child for child in pending if self.joints[child]['parent'] in poses]
            if not ready:
                raise ValueError('the URDF joints do not form a tree from ' + self.root)
            for child in ready:
                joint = self.joints[child]
                rotation, origin = poses[joint['parent']]
                angle = 0.0
                if joint['mimic']:
                    source, multiplier, offset = joint['mimic']
                    angle = multiplier * positions.get(source, 0.0) + offset
                elif joint['type'] in ('revolute', 'continuous'):
                    angle = positions.get(joint['name'], 0.0)
                local = _multiply(joint['rpy'], _rotation(joint['axis'], angle))
                offset = _apply(rotation, joint['xyz'])
                poses[child] = (_multiply(rotation, local),
                                tuple(o + d for o, d in zip(origin, offset)))
                pending.remove(child)
        return poses

    def world_boxes(self, poses, link):
        rotation, origin = poses[link]
        for center, box_rotation, half in self.boxes.get(link, []):
            world = _multiply(rotation, box_rotation)
            axes = tuple(tuple(world[row][column] for row in range(3)) for column in range(3))
            yield tuple(o + d for o, d in zip(origin, _apply(rotation, center))), axes, half

    def hits(self, positions):
        """Return what would collide at these joint positions, such as 'forearm_link: table'."""
        return list(self.check(positions))

    def check(self, positions):
        """
        Return {collision: intrusion}: each contact, and how far into it the arm reaches.

        Against the table that is the depth in m. Inside the base or shoulder it is the volume
        in m^3 of the link's box within the obstacle's box grown by the clearance, which, unlike
        a depth, grows whichever way the link pushes further in.
        """
        poses = self.poses(positions)
        found = {}
        for link in self.boxes:
            if link in BODY:
                continue
            lowest = min(center[2] - sum(h * abs(axis[2]) for h, axis in zip(half, axes))
                         for center, axes, half in self.world_boxes(poses, link))
            if lowest < self.table_z + self.clearance:
                found[f'{link}: table'] = self.table_z + self.clearance - lowest
        for link in FOLDING:
            for body in BODY:
                volume = sum(_intrusion(box, obstacle, self.clearance)
                             for box in self.world_boxes(poses, link)
                             for obstacle in self.world_boxes(poses, body)
                             if _overlap(box, obstacle, self.clearance) > 0.0)
                if volume > 0.0:
                    found[f'{link}: {body}'] = volume
        return found


def _intrusion(a, b, margin):
    """Return the volume of box a inside box b grown by margin on every side."""
    (center_a, axes_a, half_a), (center_b, axes_b, half_b) = a, b
    # Work relative to b's centre, which keeps the volume sums well conditioned.
    offset = tuple(ca - cb for ca, cb in zip(center_a, center_b))
    faces = []
    for i in range(3):
        j, k = (i + 1) % 3, (i + 2) % 3
        for side in (-1.0, 1.0):
            loop = []
            for along_j, along_k in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)):
                signs = [0.0] * 3
                signs[i], signs[j], signs[k] = side, along_j, along_k
                loop.append(tuple(o + sum(s * h * u[n] for s, h, u in zip(signs, half_a, axes_a))
                                  for n, o in enumerate(offset)))
            # Each face runs counterclockwise seen from outside the box.
            faces.append(loop if side > 0.0 else loop[::-1])
    for axis, half in zip(axes_b, half_b):
        for normal in (axis, tuple(-value for value in axis)):
            faces = _clip(faces, normal, half + margin)
    return sum(_dot(face[0], _cross(p, q)) for face in faces
               for p, q in zip(face[1:], face[2:])) / 6.0


def _clip(faces, normal, limit):
    """Cut a convex polyhedron, given as outward faces, down to normal . point <= limit."""
    distances = [_dot(normal, point) - limit for face in faces for point in face]
    if not distances or max(distances) <= 0.0:
        return faces
    if min(distances) >= 0.0:
        return []
    kept, cut = [], []
    for face in faces:
        loop = []
        for start, end in zip(face, face[1:] + face[:1]):
            a, b = _dot(normal, start) - limit, _dot(normal, end) - limit
            if a <= 0.0:
                loop.append(start)
            if a == 0.0:
                cut.append(start)
            if a < 0.0 < b or b < 0.0 < a:
                point = tuple(s + a / (a - b) * (e - s) for s, e in zip(start, end))
                loop.append(point)
                cut.append(point)
        if len(loop) >= 3:
            kept.append(loop)
    # Close the cut with a face on the plane, counterclockwise about the normal.
    middle = tuple(sum(values) / len(cut) for values in zip(*cut))
    first = _cross(normal, (1.0, 0.0, 0.0) if abs(normal[0]) < 0.9 else (0.0, 1.0, 0.0))
    second = _cross(normal, first)
    kept.append(sorted(cut, key=lambda point: math.atan2(
        _dot(second, point) - _dot(second, middle), _dot(first, point) - _dot(first, middle))))
    return kept


def _overlap(a, b, margin):
    """Return how far two oriented boxes overlap within margin (separating axes), or 0."""
    (center_a, axes_a, half_a), (center_b, axes_b, half_b) = a, b
    gap = tuple(cb - ca for ca, cb in zip(center_a, center_b))
    candidates = list(axes_a) + list(axes_b) + [
        _cross(u, v) for u in axes_a for v in axes_b]
    overlap = math.inf
    for axis in candidates:
        length = math.sqrt(_dot(axis, axis))
        if length < 1e-9:
            continue
        axis = tuple(value / length for value in axis)
        reach = (sum(h * abs(_dot(u, axis)) for h, u in zip(half_a, axes_a))
                 + sum(h * abs(_dot(u, axis)) for h, u in zip(half_b, axes_b)) + margin)
        overlap = min(overlap, reach - abs(_dot(gap, axis)))
        if overlap <= 0.0:
            return 0.0
    return overlap
