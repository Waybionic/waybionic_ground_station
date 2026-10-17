"""
Keep the arm's links off the table and out of its own base (pure logic, no ROS).

Each link's collision shapes come from the URDF, made from its CAD parts: a cylinder for a part
round about a joint axis, such as the base's lid or the tool mount, a cylinder and a box for a
part round on one side, such as the wrist's differential housing, and otherwise a box. Every
check happens in the base frame. The moving links must stay above the table, and the forearm,
wrist and tool must stay out of the base and shoulder, the parts they can fold into. Adjacent
links overlap at their joints, so those pairs are never compared; the joint limits keep them
apart. Where the URDF has a fold table for a pair (waybionic_fold, measured from the CAD), the
table replaces the shapes for it: the elbow may fold until the forearm is about to touch the
shoulder or the base.
"""

import math
import xml.etree.ElementTree as ET

# The fixed base and the yaw stage, and what may never touch them.
BODY = ('base_link', 'shoulder_link')
FOLDING = ('forearm_link', 'wrist_pitch_link', 'wrist_left_gear_link', 'wrist_right_gear_link',
           'wrist_roll_link', 'tool_link')
REQUIRED = BODY + ('upper_arm_link',) + FOLDING
# A cylinder counts as the prism with this many sides around it, at most 0.5 % wider.
SIDES = 32
_WIDER = 1.0 / math.cos(math.pi / SIDES)
_SIDES = tuple((math.cos(2.0 * math.pi * side / SIDES), math.sin(2.0 * math.pi * side / SIDES))
               for side in range(SIDES))
_CORNERS = tuple((math.cos(math.pi * (2 * side + 1) / SIDES),
                  math.sin(math.pi * (2 * side + 1) / SIDES)) for side in range(SIDES))
# A fold table may stop this short of its joint's limits, which the URDF rounds.
SPAN_TOLERANCE_DEG = 0.01


def _vector(element, name, default=(0.0, 0.0, 0.0)):
    text = element.get(name) if element is not None else None
    if not text:
        return default
    try:
        values = tuple(float(value) for value in text.split())
    except ValueError:
        values = ()
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f'robot_description has {name}="{text}", not three finite numbers')
    return values


def _number(element, name, default=math.nan):
    """Return the attribute as a float, the default if it is missing, or NaN if it is junk."""
    text = element.get(name)
    if text is None:
        return default
    try:
        return float(text)
    except ValueError:
        return math.nan


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


class Fold:
    """Limits of one joint against another joint's angle, linearly interpolated."""

    def __init__(self, element, moved_by, limits):
        """
        Read a table; raise ValueError unless it is whole and fits the joint tree.

        moved_by maps each link to the limited revolute joints between it and the root, and
        limits maps those joints to their (lower, upper) in radians. The table replaces the
        shapes for its pair, so both its joints must move the link against the obstacle, and
        it must cover the other joint's whole range.
        """
        self.link, self.obstacle = element.get('link'), element.get('obstacle')
        self.joint, self.across = element.get('joint'), element.get('across')
        try:
            self.start, self.step = float(element.get('start')), float(element.get('step'))
            self.upper = [math.radians(float(v)) for v in element.get('upper').split()]
            self.lower = [math.radians(float(v)) for v in element.get('lower').split()]
        except (AttributeError, TypeError, ValueError):
            raise ValueError('waybionic_fold needs numeric start, step, upper and lower') from None
        if not (math.isfinite(self.start) and math.isfinite(self.step) and self.step > 0.0
                and len(self.upper) == len(self.lower) >= 2
                and all(math.isfinite(value) for value in self.upper + self.lower)):
            raise ValueError('waybionic_fold needs finite values, a positive step and matching '
                             'tables')
        if self.link not in FOLDING or self.obstacle not in BODY:
            raise ValueError('waybionic_fold must pair a folding link with the base or shoulder')
        between = moved_by.get(self.link, set()) - moved_by.get(self.obstacle, set())
        if self.joint == self.across or not {self.joint, self.across} <= between:
            raise ValueError(f'waybionic_fold for {self.link} and {self.obstacle} needs two '
                             'different limited joints that move one against the other')
        lower, upper = (math.degrees(limit) for limit in limits[self.across])
        end = self.start + self.step * (len(self.upper) - 1)
        if (self.start > lower + SPAN_TOLERANCE_DEG
                or end < upper - SPAN_TOLERANCE_DEG):
            raise ValueError(f'waybionic_fold for {self.link} and {self.obstacle} covers '
                             f'{self.across} from {self.start:g} to {end:g} degrees, not its '
                             f'whole range {lower:.2f} to {upper:.2f}')
        if any(low > high for low, high in zip(self.lower, self.upper)):
            raise ValueError(f'waybionic_fold for {self.link} and {self.obstacle} has a lower '
                             'value above its upper value')

    def limits(self, across):
        """Return (lower, upper) in radians at this angle of the other joint."""
        x = (math.degrees(across) - self.start) / self.step
        index = min(max(int(math.floor(x)), 0), len(self.upper) - 2)
        t = min(max(x - index, 0.0), 1.0)
        return tuple(table[index] + t * (table[index + 1] - table[index])
                     for table in (self.lower, self.upper))

    def excess(self, positions):
        """How far past the table the joint is, in radians, or 0."""
        lower, upper = self.limits(positions.get(self.across, 0.0))
        angle = positions.get(self.joint, 0.0)
        return max(angle - upper, lower - angle, 0.0)


class ArmCollision:
    """Collision shapes and joint tree read from a URDF."""

    def __init__(self, shapes, joints, root, table_z=0.0, clearance=0.01, self_clearance=0.002,
                 folds=()):
        self.shapes, self.joints, self.root = shapes, joints, root
        self.table_z, self.clearance, self.self_clearance = table_z, clearance, self_clearance
        self.folds = {(fold.link, fold.obstacle): fold for fold in folds}

    @classmethod
    def from_urdf(cls, urdf, table_z=0.0, clearance=0.01, self_clearance=0.002):
        """Read shapes and joints; raise ValueError if a required link has no shapes."""
        try:
            robot = ET.fromstring(urdf)
        except ET.ParseError as error:
            raise ValueError(f'robot_description is not valid XML: {error}') from None
        shapes = {}
        for link in robot.findall('link'):
            for collision in link.findall('collision'):
                box = collision.find('geometry/box')
                cylinder = collision.find('geometry/cylinder')
                if box is None and cylinder is None:
                    continue
                if box is not None:
                    half, radius = tuple(size / 2.0 for size in _vector(box, 'size')), None
                else:
                    radius = _number(cylinder, 'radius')
                    half = (radius, radius, _number(cylinder, 'length') / 2.0)
                if not all(math.isfinite(size) and size > 0.0 for size in half):
                    # A shape without a size would silently drop out of the checks.
                    raise ValueError('robot_description has a collision shape without a '
                                     f'positive finite size on {link.get("name")}')
                origin = collision.find('origin')
                shapes.setdefault(link.get('name'), []).append((
                    _vector(origin, 'xyz'), _rpy(*_vector(origin, 'rpy')), half, radius))
        missing = [name for name in REQUIRED if name not in shapes]
        if missing:
            raise ValueError('robot_description has no collision box for ' + ', '.join(missing))
        joints, children = {}, set()
        for joint in robot.findall('joint'):
            mimic = joint.find('mimic')
            origin = joint.find('origin')
            axis = _vector(joint.find('axis'), 'xyz', (1.0, 0.0, 0.0))
            length = math.sqrt(_dot(axis, axis))
            if mimic is not None:
                mimic = (mimic.get('joint'), _number(mimic, 'multiplier', 1.0),
                         _number(mimic, 'offset', 0.0))
            if length < 1e-9 or (mimic and not all(map(math.isfinite, mimic[1:]))):
                raise ValueError(f'robot_description has joint {joint.get("name")} without an '
                                 'axis, or with a mimic that is not two finite numbers')
            joints[joint.find('child').get('link')] = {
                'name': joint.get('name'), 'type': joint.get('type'),
                'parent': joint.find('parent').get('link'),
                'xyz': _vector(origin, 'xyz'), 'rpy': _rpy(*_vector(origin, 'rpy')),
                'axis': tuple(value / length for value in axis), 'mimic': mimic}
            children.add(joint.find('child').get('link'))
        roots = [link.get('name') for link in robot.findall('link')
                 if link.get('name') not in children]
        if len(roots) != 1:
            raise ValueError(f'robot_description needs one root link, not {len(roots)}')
        checker = cls(shapes, joints, roots[0], table_z, clearance, self_clearance)
        checker.poses({})
        limits = {}
        for joint in robot.findall('joint'):
            limit = joint.find('limit')
            if (joint.get('type') == 'revolute' and joint.find('mimic') is None
                    and limit is not None):
                bounds = (_number(limit, 'lower'), _number(limit, 'upper'))
                if all(math.isfinite(bound) for bound in bounds):
                    limits[joint.get('name')] = bounds
        moved_by = {}
        for link in shapes:
            moved_by[link], child = set(), link
            # poses() has checked that the joints form a tree, so this reaches the root.
            while child in joints:
                if joints[child]['name'] in limits:
                    moved_by[link].add(joints[child]['name'])
                child = joints[child]['parent']
        folds = [Fold(element, moved_by, limits) for element in robot.findall('waybionic_fold')]
        pairs = {(fold.link, fold.obstacle) for fold in folds}
        if len(pairs) != len(folds):
            raise ValueError('robot_description has two waybionic_fold tables for one pair')
        checker.folds = {(fold.link, fold.obstacle): fold for fold in folds}
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

    def world_shapes(self, poses, link):
        """
        Yield (centre, axes, half sizes, radius) for the link's shapes in the root frame.

        A box has radius None. A cylinder runs along axes[2], with half sizes (radius, radius,
        half length).
        """
        rotation, origin = poses[link]
        for center, local, half, radius in self.shapes.get(link, []):
            world = _multiply(rotation, local)
            axes = tuple(tuple(world[row][column] for row in range(3)) for column in range(3))
            yield (tuple(o + d for o, d in zip(origin, _apply(rotation, center))), axes, half,
                   radius)

    def hits(self, positions):
        """Return what would collide at these joint positions, such as 'forearm_link: table'."""
        return list(self.check(positions))

    def check(self, positions):
        """
        Return {collision: intrusion}: each contact, and how far into it the arm reaches.

        Against the table that is the depth in m. Inside the base or shoulder it is the volume
        in m^3 of the link's shapes within the obstacle's shapes grown by the self-collision
        clearance, which, unlike a depth, grows whichever way the link pushes further in. Past
        a fold table it is the angle in radians beyond the table.
        """
        poses = self.poses(positions)
        world = {link: [(shape, _reach(shape)) for shape in self.world_shapes(poses, link)]
                 for link in self.shapes}
        found = {}
        for link, placed in world.items():
            if link in BODY:
                continue
            lowest = min(_lowest(shape) for shape, _ in placed)
            if lowest < self.table_z + self.clearance:
                found[f'{link}: table'] = self.table_z + self.clearance - lowest
        margin = self.self_clearance
        for link in FOLDING:
            for body in BODY:
                fold = self.folds.get((link, body))
                if fold is not None:
                    excess = fold.excess(positions)
                    if excess > 0.0:
                        found[f'{link}: {body}'] = excess
                    continue
                # Shapes whose bounding spheres are apart cannot overlap, nor can two boxes
                # that a separating axis keeps apart.
                volume = sum(_intrusion(shape, obstacle, margin)
                             for shape, reach in world[link] for obstacle, extent in world[body]
                             if _squared_distance(shape[0], obstacle[0])
                             < (reach + extent + margin) ** 2
                             and (shape[3] is not None or obstacle[3] is not None
                                  or _overlap(shape, obstacle, margin) > 0.0))
                if volume > 0.0:
                    found[f'{link}: {body}'] = volume
        return found


def _squared_distance(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b))


def _reach(shape):
    """Return the radius of a sphere about the shape's centre that holds all of it."""
    _, _, half, radius = shape
    if radius is None:
        return math.sqrt(_dot(half, half))
    return math.hypot(radius * _WIDER, half[2])


def _lowest(shape):
    """Return the height of the shape's lowest point."""
    center, axes, half, radius = shape
    if radius is None:
        return center[2] - sum(h * abs(axis[2]) for h, axis in zip(half, axes))
    tilt = abs(axes[2][2])
    return center[2] - half[2] * tilt - radius * math.sqrt(max(0.0, 1.0 - tilt * tilt))


def _spread(shape, normal):
    """Return how far the shape reaches from its centre along a unit normal."""
    _, axes, half, radius = shape
    if radius is None:
        return sum(h * abs(_dot(normal, axis)) for h, axis in zip(half, axes))
    along = abs(_dot(normal, axes[2]))
    return half[2] * along + radius * _WIDER * math.sqrt(max(0.0, 1.0 - along * along))


def _facing(cylinder, toward):
    """Return the cylinder's cross axes, turned so that the first points at toward."""
    center, (first, second, axis), _, _ = cylinder
    offset = tuple(t - c for t, c in zip(toward, center))
    along = _dot(offset, axis)
    outward = tuple(o - along * a for o, a in zip(offset, axis))
    length = math.sqrt(_dot(outward, outward))
    if length < 1e-9:
        return first, second
    first = tuple(value / length for value in outward)
    return first, _cross(axis, first)


def _planes(shape, toward, margin):
    """Yield the shape's face planes as (outward normal, distance from its centre), grown."""
    _, axes, half, radius = shape
    if radius is None:
        for axis, h in zip(axes, half):
            yield axis, h + margin
            yield tuple(-value for value in axis), h + margin
        return
    first, second = _facing(shape, toward)
    yield first, radius + margin
    yield axes[2], half[2] + margin
    yield tuple(-value for value in axes[2]), half[2] + margin
    for c, s in _SIDES[1:]:
        yield tuple(c * u + s * v for u, v in zip(first, second)), radius + margin


def _faces(shape, origin):
    """
    Return the shape's faces relative to origin, each counterclockwise seen from outside.

    A cylinder's prism turns to face origin, so turning about its axis changes nothing.
    """
    center, axes, half, radius = shape
    offset = tuple(c - o for c, o in zip(center, origin))
    if radius is not None:
        first, second = _facing(shape, origin)
        top, bottom = ([tuple(o + side * half[2] * a + radius * _WIDER * (c * u + s * v)
                              for o, a, u, v in zip(offset, axes[2], first, second))
                        for c, s in _CORNERS] for side in (1.0, -1.0))
        return [top, bottom[::-1]] + [
            [bottom[k], bottom[(k + 1) % SIDES], top[(k + 1) % SIDES], top[k]]
            for k in range(SIDES)]
    faces = []
    for i in range(3):
        j, k = (i + 1) % 3, (i + 2) % 3
        for side in (-1.0, 1.0):
            loop = []
            for along_j, along_k in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)):
                signs = [0.0] * 3
                signs[i], signs[j], signs[k] = side, along_j, along_k
                loop.append(tuple(o + sum(s * h * u[n] for s, h, u in zip(signs, half, axes))
                                  for n, o in enumerate(offset)))
            faces.append(loop if side > 0.0 else loop[::-1])
    return faces


def _volume(faces):
    return sum(_dot(face[0], _cross(p, q)) for face in faces
               for p, q in zip(face[1:], face[2:])) / 6.0


def _intrusion(a, b, margin):
    """
    Return the volume of shape a inside shape b grown by margin on every side.

    A cylinder counts as the SIDES-sided prism around it, turned to face the other shape.
    """
    offset = tuple(x - y for x, y in zip(a[0], b[0]))
    cutting = []
    for normal, limit in _planes(b, a[0], margin):
        middle, spread = _dot(normal, offset), _spread(a, normal)
        if middle - spread >= limit:
            return 0.0
        if middle + spread > limit:
            cutting.append((normal, limit))
    # Work relative to b's centre, which keeps the volume sums well conditioned.
    faces = _faces(a, b[0])
    for normal, limit in cutting:
        faces = _clip(faces, normal, limit)
    return _volume(faces)


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
    (center_a, axes_a, half_a, _), (center_b, axes_b, half_b, _) = a, b
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
