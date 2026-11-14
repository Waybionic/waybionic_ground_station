#!/usr/bin/env python3
"""Find exactly where the arm's CAD parts touch, and where the arm meets the ground.

    python3 scripts/cad/exact_limits.py build/cad/arm_export.json [--hulls scripts/cad/link_hulls.json]

This needs numpy, scipy and python-fcl (pip install python-fcl); the build script does not.
After a CAD change, run this, copy the joint limits it prints into JOINTS in
build_arm_description.py, run joint_clearance.py for the ELBOW_FOLDS, rebuild the URDF, and
check it with check_collision_model.py.

Every CAD part, gearbox internals excepted (they are enclosed, and static CAD cannot model their
eccentric motion), becomes an FCL triangle mesh in its link frame. A joint turns from the upright
pose, every other joint at zero, and the links it carries move against the rest of the arm. Each
step turns no point of the moving parts further than the smallest gap, so no contact is stepped
over, and the contact is then bisected to under 0.001 degree on exact triangle distances. Parts
that already touch upright, such as bearing rings on their seats and meshing gears, turn on the
joint's own axis and are left out. The ground is the plane under the base (z = 0 in base_link),
and the lowest vertex of the moving parts is exact.

A limit that holds in every pose comes from the parts that only that joint moves against each
other, over a full turn of any joint that turns them too (the wrist's roll turns the tool mount
against the forearm's cap); where a contact also depends on another joint, the teleop collision
check stops the arm instead (ELBOW_FOLDS for the forearm, and the ground). --hulls writes each
moving link's convex hull in its link frame, which the URDF carries so the teleop check finds
the lowest point of the real parts rather than of their collision shapes.
"""

import argparse
import importlib.util
import json
import math
import multiprocessing
from pathlib import Path

import fcl
import numpy as np
from scipy.spatial import ConvexHull

spec = importlib.util.spec_from_file_location(
    'build_arm_description', Path(__file__).resolve().with_name('build_arm_description.py'))
arm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arm)

INTERNAL = ('discs-sweep', 'disc-bearing', 'loose_disc', 'inner_bearing', 'shaft-sweep',
            'NEMA23SHAFT', 'outer_ball_bearing', 'outer-ring-bearing', 'bearing_brace',
            'updatednema23bearingbrace', 'top-sweep')
TOUCH = 1e-6    # m: a gap this small is contact
MATED = 5e-5    # m: parts this close in the upright pose are mated
# Links the teleop check keeps off the ground: all but the base and the yaw stage, which turns
# only about the vertical.
HULL_LINKS = ('upper_arm_link', 'forearm_link', 'wrist_pitch_link', 'wrist_left_gear_link',
              'wrist_right_gear_link', 'wrist_roll_link')


def rotation(axis, angle):
    return np.array(arm.rotation_about(axis, angle))


def rpy(roll, pitch, yaw):
    return rotation((0, 0, 1), yaw) @ rotation((0, 1, 0), pitch) @ rotation((1, 0, 0), roll)


class Robot:
    """CAD parts as FCL meshes in their link frames, and the joint tree."""

    def __init__(self, export_path):
        export = arm.Export(export_path)
        self.model = arm.Model(export, arm.check_components(export))
        self.parts = []  # (link, name, hull vertices, FCL mesh)
        self.points = {}  # link -> every vertex in its link frame, internals included
        for link, names in self.model.assigned.items():
            for name in sorted(names):
                component = export.components[name]
                pose = (self.model.frames[link].inverse() * self.model.to_base
                        * arm.solidworks_pose(component['transform']))
                turn, origin = np.array(pose.rotation), np.array(pose.origin)
                vertices, triangles, offset = [], [], 0
                for face in component['faces']:
                    v, t = export.face_arrays(face)
                    v = np.frombuffer(v.tobytes(), dtype=np.float32).reshape(-1, 3).astype(float)
                    vertices.append(v)
                    triangles.append(np.frombuffer(t.tobytes(), dtype=np.int32).reshape(-1, 3)
                                     + offset)
                    offset += len(v)
                if not vertices:
                    continue
                v = np.vstack(vertices) @ turn.T + origin
                self.points.setdefault(link, []).append(v)
                if name.split('/')[-1].startswith(INTERNAL):
                    continue
                t = np.vstack(triangles)
                mesh = fcl.BVHModel()
                mesh.beginModel(len(v), len(t))
                mesh.addSubModel(v, t)
                mesh.endModel()
                self.parts.append((link, name, v[ConvexHull(v).vertices], mesh))
        self.joints = {joint['name']: joint for joint in self.model.joints}

    def poses(self, q):
        poses = {'base_link': (np.eye(3), np.zeros(3))}
        for joint in self.model.joints:
            angle = q.get(joint['name'], 0.0)
            if joint['mimic']:
                angle = joint['mimic'][1] * q.get(joint['mimic'][0], 0.0)
            r, o = poses[joint['parent']]
            poses[joint['child']] = (r @ rpy(*joint['rpy']) @ rotation(joint['axis'], angle),
                                     o + r @ np.array(joint['xyz']))
        return poses

    def subtree(self, link):
        found = {link}
        for joint in self.model.joints:
            if joint['parent'] in found:
                found.add(joint['child'])
        return found

    def hull(self, link):
        points = np.vstack(self.points[link])
        return points[ConvexHull(points).vertices]


class Sweep:
    """One joint turning from the upright pose, some links against others."""

    def __init__(self, robot, joint, moving=None, fixed=None, held=None):
        self.robot, self.name = robot, joint
        # Other joints held at these angles (radians) while this one turns.
        self.held = held or {}
        moving = moving or robot.subtree(robot.joints[joint]['child'])
        self.moving = [part for part in robot.parts if part[0] in moving]
        self.fixed = [part for part in robot.parts
                      if part[0] not in moving and (fixed is None or part[0] in fixed)]
        self.objects = {id(part): fcl.CollisionObject(part[3], fcl.Transform())
                        for part in robot.parts}
        self.pairs = [(m, f) for m in self.moving for f in self.fixed]
        self.place(0.0)
        self.mated = [(m, f) for bound, m, f in self._candidates()
                      if bound <= MATED and self._distance(m, f) <= MATED]
        mated = {(id(m), id(f)) for m, f in self.mated}
        self.pairs = [(m, f) for m, f in self.pairs if (id(m), id(f)) not in mated]
        info = robot.joints[joint]
        r, o = robot.poses(self.held)[info['parent']]
        self.origin = o + r @ np.array(info['xyz'])
        self.axis = r @ rpy(*info['rpy']) @ np.array(info['axis'], dtype=float)
        # No moving point is further than this from the joint axis.
        self.reach = max(float(np.linalg.norm(
            (self.world[id(part)] - self.origin)
            - np.outer((self.world[id(part)] - self.origin) @ self.axis, self.axis),
            axis=1).max()) for part in self.moving)

    def place(self, angle):
        poses = self.robot.poses({**self.held, self.name: angle})
        self.world, self.boxes = {}, {}
        for part in self.robot.parts:
            r, o = poses[part[0]]
            self.objects[id(part)].setTransform(fcl.Transform(r, o))
            points = part[2] @ r.T + o
            self.world[id(part)] = points
            self.boxes[id(part)] = (points.min(axis=0), points.max(axis=0))

    def _candidates(self):
        """Pairs with a lower bound on their gap from their boxes, nearest first."""
        found = []
        for m, f in self.pairs:
            (a0, a1), (b0, b1) = self.boxes[id(m)], self.boxes[id(f)]
            found.append((float(np.linalg.norm(np.maximum(0.0, np.maximum(a0 - b1, b0 - a1)))),
                          m, f))
        found.sort(key=lambda item: item[0])
        return found

    def _distance(self, m, f):
        return fcl.distance(self.objects[id(m)], self.objects[id(f)], fcl.DistanceRequest(),
                            fcl.DistanceResult())

    def gap(self):
        best, pair = math.inf, None
        for bound, m, f in self._candidates():
            if bound >= best:
                break
            distance = self._distance(m, f)
            if distance < best:
                best, pair = distance, (m, f)
        return best, pair

    def contact(self, sign, limit=360.0):
        """Degrees turned before two parts touch, with the parts, or (None, None)."""
        angle = clear = 0.0
        while angle < limit:
            self.place(sign * math.radians(angle))
            gap, pair = self.gap()
            if gap <= TOUCH:
                break
            clear = angle
            angle += max(math.degrees(gap / self.reach), 0.002)
        else:
            return None, None
        low, high = clear, angle
        while high - low > 0.0005:
            middle = (low + high) / 2
            self.place(sign * math.radians(middle))
            if self.gap()[0] <= TOUCH:
                high = middle
            else:
                low = middle
        self.place(sign * math.radians(high))
        m, f = self.gap()[1]
        return low, f'{m[0]}: {m[1]}  ->  {f[0]}: {f[1]}'

    def lowest(self):
        return min(float(self.world[id(part)][:, 2].min()) for part in self.moving)

    def ground(self, sign, limit=360.0, step=0.25):
        """Degrees turned before the moving parts reach z = 0, or None."""
        angle = 0.0
        while angle < limit:
            angle += step
            self.place(sign * math.radians(angle))
            if self.lowest() <= 0.0:
                low, high = angle - step, angle
                while high - low > 0.0005:
                    middle = (low + high) / 2
                    self.place(sign * math.radians(middle))
                    if self.lowest() <= 0.0:
                        high = middle
                    else:
                        low = middle
                return low
        return None


# The robot for the roll scan's worker processes, which fork with it.
_ROBOT = None


def _wrist_contacts(roll):
    sweep = Sweep(_ROBOT, 'joint_4', fixed={'forearm_link'},
                  held={'joint_5': math.radians(roll)})
    return [sweep.contact(sign, 120.0)[0] for sign in (1, -1)]


def wrist_stops(robot, step=0.1):
    """
    Return the wrist pitch (degrees) either way where the wrist first touches the forearm.

    Rolling the wrist turns the tool mount's facets against the forearm's cap, so the contact
    is found every step degrees over a full turn of roll and the lowest kept. It changes by
    under 0.1 degree of pitch per degree of roll, so between two steps it is at most 0.005
    degree lower than the lowest found.
    """
    global _ROBOT
    _ROBOT = robot
    with multiprocessing.get_context('fork').Pool() as pool:
        found = pool.map(_wrist_contacts, np.arange(0.0, 360.0, step))
    return tuple(min(contacts[side] for contacts in found) for side in (0, 1))


def report(sweep, title):
    print(title)
    if sweep.mated:
        print('  left out, mated upright: ' + '; '.join(
            f'{m[1].split("/")[-1]} on {f[1].split("/")[-1]}' for m, f in sweep.mated))
    found = []
    for sign in (1, -1):
        angle, parts = sweep.contact(sign)
        floor = sweep.ground(sign)
        side = '+' if sign > 0 else '-'
        touch = 'nothing touches in a full turn' if angle is None else f'{sign * angle:+.3f}'
        meets = '' if floor is None else f', the ground at {sign * floor:+.3f}'
        print(f'  {side}: {touch}{meets}')
        if parts:
            print(f'      {parts}')
        found.append((angle, floor))
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export', type=Path, help='JSON written by export_solidworks_assembly')
    parser.add_argument('--hulls', type=Path, help='write the moving links\' hulls here')
    args = parser.parse_args()
    robot = Robot(args.export)

    print('Each joint from the upright pose, the links it carries against the rest of the arm:')
    for name in ('joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'):
        report(Sweep(robot, name), f'{name}:')
    print('Contacts that no other joint changes:')
    report(Sweep(robot, 'joint_2', moving={'upper_arm_link'},
                 fixed={'base_link', 'shoulder_link'}),
           'joint_2, the upper arm against the base and the yaw stage:')
    report(Sweep(robot, 'joint_3', fixed={'upper_arm_link'}),
           'joint_3, the forearm and wrist against the upper arm:')
    report(Sweep(robot, 'joint_4', fixed={'forearm_link'}),
           'joint_4, the wrist against the forearm, roll at zero:')
    upper, lower = (math.floor((angle - 0.005) * 100) / 100 for angle in wrist_stops(robot))
    print(f'  over a full turn of roll, wrist pitch limits ({-lower:.2f}, {upper:.2f})')

    if args.hulls:
        hulls = {link: [[round(float(value), 5) for value in point]
                        for point in robot.hull(link)] for link in HULL_LINKS}
        args.hulls.write_text(json.dumps(hulls, separators=(',', ':')) + '\n', encoding='utf-8')
        print(f'Wrote {sum(map(len, hulls.values()))} hull points to {args.hulls}')


if __name__ == '__main__':
    main()
