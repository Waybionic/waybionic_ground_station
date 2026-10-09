"""
Exact kinematics for straight tool-tip moves (pure logic, no ROS).

The arm is a base yaw joint, three pitch joints with parallel axes and a tool roll joint whose
axis runs through the tool tip. The pitch joints keep the arm in one vertical plane, offset
sideways from the yaw axis, so every tip position and tool pitch has a closed-form solution.
Pitch is the tool axis angle from straight up, in the arm plane: 0 up, pi straight down.
"""

import math
import xml.etree.ElementTree as ET

TOLERANCE = 1e-9
LAYOUT = 'Cartesian moves need a yaw joint, three parallel pitch joints and a tool roll joint'


def _vector(element, name):
    text = element.get(name) if element is not None else None
    if text is None:
        return (0.0, 0.0, 0.0)
    try:
        vector = tuple(float(value) for value in text.split())
    except ValueError:
        raise ValueError(f'robot_description has an invalid {name} vector: {text}') from None
    if len(vector) != 3 or not all(math.isfinite(value) for value in vector):
        raise ValueError(f'robot_description has an invalid {name} vector: {text}')
    return vector


def _is(vector, expected):
    return all(abs(a - b) <= TOLERANCE for a, b in zip(vector, expected))


def _within(value, bounds):
    lower, upper = bounds or (-math.inf, math.inf)
    return lower - TOLERANCE <= value <= upper + TOLERANCE


def nearest_within(angle, reference, bounds):
    """Return the turn of angle nearest reference that lies within bounds, or None."""
    base = reference + math.remainder(angle - reference, 2 * math.pi)
    options = [base + turn * 2 * math.pi for turn in (0, -1, 1)]
    options = [value for value in options if _within(value, bounds)]
    return min(options, key=lambda value: abs(value - reference)) if options else None


def joint_limits(urdf, joints, required=True):
    """Return validated {joint: (lower, upper)} for the named joints of a URDF string."""
    try:
        robot = ET.fromstring(urdf)
    except ET.ParseError as error:
        raise ValueError(f'robot_description is not valid XML: {error}') from None
    if robot.tag != 'robot':
        raise ValueError('robot_description is not a robot URDF')
    limits, seen, requested = {}, set(), set(joints)
    for element in robot.findall('joint'):
        name = element.get('name')
        if name not in requested:
            continue
        if name in seen:
            raise ValueError(f'robot_description has duplicate joint {name}')
        seen.add(name)
        if element.get('type') == 'continuous':
            limits[name] = (-math.inf, math.inf)
        elif element.get('type') in ('revolute', 'prismatic'):
            limit = element.find('limit')
            try:
                lower = float(limit.get('lower'))
                upper = float(limit.get('upper'))
            except (AttributeError, TypeError, ValueError):
                raise ValueError(
                    f'robot_description has missing or invalid limits for {name}') from None
            if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
                raise ValueError(f'robot_description has missing or invalid limits for {name}')
            limits[name] = (lower, upper)
    missing = [joint for joint in joints if joint not in limits]
    if missing and required:
        raise ValueError('robot_description has no movable joint ' + ', '.join(missing))
    return limits


class ArmKinematics:
    """Tool-tip position and pitch from joint angles, and joint angles from a tip pose."""

    def __init__(self, joints, height, offset, upper, fore, tool):
        self.joints = tuple(joints)
        self.height, self.offset = height, offset
        self.upper, self.fore, self.tool = upper, fore, tool

    @classmethod
    def from_urdf(cls, urdf, base_frame='base_link', tool_frame='tool_link'):
        """Read the geometry from a URDF string; raise ValueError for any other layout."""
        try:
            robot = ET.fromstring(urdf)
        except ET.ParseError as error:
            raise ValueError(f'robot_description is not valid XML: {error}') from None
        if robot.tag != 'robot' or base_frame == tool_frame:
            raise ValueError(LAYOUT)
        by_child = {}
        for joint in robot.findall('joint'):
            parent, child = joint.find('parent'), joint.find('child')
            if (parent is None or child is None or not parent.get('link')
                    or not child.get('link') or child.get('link') in by_child):
                raise ValueError(f'{LAYOUT}: missing or duplicate link')
            by_child[child.get('link')] = joint
        chain, link, visited = [], tool_frame, set()
        while link != base_frame:
            if link in visited:
                raise ValueError(f'{LAYOUT}: joint chain contains a cycle')
            visited.add(link)
            if link not in by_child:
                raise ValueError(f'no joint chain from {base_frame} to {tool_frame}')
            chain.insert(0, by_child[link])
            link = chain[0].find('parent').get('link')
        axes = [_vector(joint.find('axis'), 'xyz') for joint in chain[:5]]
        origins = [_vector(joint.find('origin'), 'xyz') for joint in chain]
        up, side = (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)
        if (len(chain) < 5
                or not all(joint.get('type') in ('revolute', 'continuous') for joint in chain[:5])
                or not all(joint.get('type') == 'fixed' for joint in chain[5:])
                or not all(_is(axis, expected)
                           for axis, expected in zip(axes, (up, side, side, side, up)))
                or not all(_is(_vector(joint.find('origin'), 'rpy'), (0.0, 0.0, 0.0))
                           for joint in chain)
                or not all(abs(origin[0]) <= TOLERANCE for origin in origins)
                or abs(origins[0][1]) > TOLERANCE
                or not all(abs(origin[1]) <= TOLERANCE for origin in origins[4:])):
            raise ValueError(LAYOUT)
        height = origins[0][2] + origins[1][2]
        offset = origins[1][1] + origins[2][1] + origins[3][1]
        upper, fore = origins[2][2], origins[3][2]
        tool = sum(origin[2] for origin in origins[4:])
        if min(upper, fore, tool) <= TOLERANCE:
            raise ValueError(f'{LAYOUT}: link lengths must be positive')
        return cls([joint.get('name') for joint in chain[:5]],
                   height=height, offset=offset, upper=upper, fore=fore, tool=tool)

    def forward(self, joints):
        """Return the tool-tip position (x, y, z) in the base frame and the tool pitch."""
        yaw, a, b, c = (joints[name] for name in self.joints[:4])
        pitch = a + b + c
        reach = (self.upper * math.sin(a) + self.fore * math.sin(a + b)
                 + self.tool * math.sin(pitch))
        height = (self.height + self.upper * math.cos(a) + self.fore * math.cos(a + b)
                  + self.tool * math.cos(pitch))
        cos, sin = math.cos(yaw), math.sin(yaw)
        return (cos * reach - sin * self.offset, sin * reach + cos * self.offset, height), pitch

    def inverse(self, position, pitch, roll, reference, limits=None):
        """Return the joints nearest reference that put the tip at position, or None."""
        limits = limits or {}
        x, y, z = position
        planar = x * x + y * y - self.offset * self.offset
        if planar < -TOLERANCE:
            return None
        best, best_cost = None, math.inf
        for reach in {math.sqrt(max(planar, 0.0)), -math.sqrt(max(planar, 0.0))}:
            yaw = math.atan2(y, x) - math.atan2(self.offset, reach)
            wrist_x = reach - self.tool * math.sin(pitch)
            wrist_z = z - self.height - self.tool * math.cos(pitch)
            cos_b = ((wrist_x * wrist_x + wrist_z * wrist_z - self.upper ** 2 - self.fore ** 2)
                     / (2.0 * self.upper * self.fore))
            if abs(cos_b) > 1.0 + TOLERANCE:
                continue
            elbow = math.acos(min(max(cos_b, -1.0), 1.0))
            for b in {elbow, -elbow}:
                a = math.atan2(wrist_x, wrist_z) - math.atan2(
                    self.fore * math.sin(b), self.upper + self.fore * math.cos(b))
                candidate = {}
                for name, value in zip(self.joints, (yaw, a, b, pitch - a - b, roll)):
                    value = nearest_within(value, reference[name], limits.get(name))
                    if value is None:
                        break
                    candidate[name] = value
                else:
                    cost = sum((candidate[name] - reference[name]) ** 2 for name in self.joints)
                    if cost < best_cost:
                        best, best_cost = candidate, cost
        return best

    def jog(self, joints, velocity, pitch_rate, roll_rate, dt, limits, max_rate):
        """
        Move the tip along a straight line for dt; return (joints, fraction, blocked).

        pitch_rate tilts the tool about its tip. The whole step shrinks, never one joint, so
        the tip stays on the line when a joint would pass max_rate or a limit. Roll turns
        against the yaw, so the tool doesn't spin about its own axis as the base turns; with
        the tool pointing straight down, that keeps a blade's heading.
        """
        if not (math.isfinite(dt) and dt > 0 and math.isfinite(max_rate) and max_rate > 0):
            raise ValueError('Cartesian step duration and joint speed must be positive')
        # Permit recovery from an already-outside joint, without moving farther beyond its limit.
        bounds = {}
        for name, (lower, upper) in limits.items():
            if name in joints:
                current = joints[name]
                # Normal solver tolerance must not widen the limit a little on every tick.
                bounds[name] = (current if current < lower - 10 * TOLERANCE else lower,
                                current if current > upper + 10 * TOLERANCE else upper)
        start, pitch = self.forward(joints)
        yaw, roll = self.joints[0], self.joints[4]

        def solve(fraction, bounds):
            new_pitch = pitch + pitch_rate * dt * fraction
            target = [p + v * dt * fraction for p, v in zip(start, velocity)]
            arm = {name: value for name, value in bounds.items() if name != roll}
            result = self.inverse(target, new_pitch, joints[roll], joints, arm)
            if result is None:
                return None
            # Turning the yaw also turns the tool about its own axis by cos(pitch) of that turn.
            result[roll] = (joints[roll] + roll_rate * dt * fraction
                            - math.cos((pitch + new_pitch) / 2.0) * (result[yaw] - joints[yaw]))
            if not _within(result[roll], bounds.get(roll)):
                return None
            if bounds:
                for name, (lower, upper) in limits.items():
                    if name not in joints:
                        continue
                    current = joints[name]
                    if ((current < lower - 10 * TOLERANCE and result[name] < current)
                            or (current > upper + 10 * TOLERANCE
                                and result[name] > current)):
                        return None
            return result

        def fastest(result):
            return max(abs(result[name] - joints[name]) for name in self.joints) / dt

        fraction, result = 1.0, None
        for _ in range(4):
            result = solve(fraction, bounds)
            if result is None:
                break
            if fastest(result) <= max_rate * (1.0 + 1e-6):
                return result, fraction, []
            fraction *= max_rate / fastest(result)
        blocked = [] if result is not None else self._blockers(solve(fraction, {}), limits)
        # Go as far along the line as the limits allow; the rate check also rejects any
        # jump to another solution branch.
        low, high, best = 0.0, fraction, None
        for _ in range(40):
            middle = (low + high) / 2.0
            found = solve(middle, bounds)
            if found is not None and fastest(found) <= max_rate * (1.0 + 1e-6):
                low, best = middle, found
            else:
                high = middle
        return (best, low, blocked) if best is not None else (dict(joints), 0.0, blocked)

    def _blockers(self, free, limits):
        names = [] if free is None else [
            name for name in self.joints if not _within(free[name], limits.get(name))]
        return names or ['reach']
