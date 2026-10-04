#!/usr/bin/env python3
"""Find how far the arm's pitch joints turn before two CAD parts touch.

    python3 scripts/cad/joint_clearance.py build/cad/arm_export.json

After a CAD change, copy the printed limits into JOINTS and ELBOW_FOLD in
build_arm_description.py and rebuild the URDF. This script needs numpy and
scipy; the build script does not.

Each part's SolidWorks tessellation is sampled densely in its link frame, with
outward normals. For a joint, the links it carries turn as one body against
the rest of the arm. A sample that ends up more than PENETRATION inside a part
of the other body is a clash once a winding number and the exact distance to
that part's triangles confirm it. Samples already inside in the upright pose
are press fits and are ignored. Gearbox internals are skipped: they are
enclosed, and static CAD cannot model their eccentric motion. Base yaw and
wrist roll turn round parts in round housings, so cabling sets their range.

How far the elbow folds before the forearm meets the shoulder also depends on
the shoulder angle, so that is tabulated against it. The collision boxes are
too coarse for that pair, and the teleop collision check uses the table.
"""

import argparse
import importlib.util
import math
from pathlib import Path
import textwrap

import numpy as np
from scipy.spatial import cKDTree

DENSITY = 2.0e6        # samples per square metre of surface
PENETRATION = 0.0005   # m inside another part that counts as a clash
GROWTH = 0.00025       # m deeper than in the upright pose
SEARCH = 0.004         # m
NEIGHBOURS = 6
CONTACT = 0.001        # m between forearm and shoulder samples that counts as touching
MARGIN = 1.0           # degrees kept clear of contact
INTERNAL = ('discs-sweep', 'disc-bearing', 'loose_disc', 'inner_bearing', 'shaft-sweep',
            'NEMA23SHAFT', 'outer_ball_bearing', 'outer-ring-bearing', 'bearing_brace',
            'updatednema23bearingbrace', 'top-sweep')
PITCH_JOINTS = ('joint_2', 'joint_3', 'joint_4')
FOLD = ('forearm_link', 'shoulder_link', 'joint_3', 'joint_2')

spec = importlib.util.spec_from_file_location(
    'build_arm_description', Path(__file__).resolve().with_name('build_arm_description.py'))
arm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(arm)


def rotation(axis, angle):
    return np.array(arm.rotation_about(axis, angle))


def rpy(roll, pitch, yaw):
    return rotation((0, 0, 1), yaw) @ rotation((0, 1, 0), pitch) @ rotation((1, 0, 0), roll)


class Arm:
    """Sampled CAD parts in their link frames, and the joint tree."""

    def __init__(self, export_path, seed=7):
        export = arm.Export(export_path)
        self.model = arm.Model(export, arm.check_components(export))
        self.rng = np.random.default_rng(seed)
        self.parts = {}  # link -> [(part, points, normals, triangles per body)]
        for link, names in self.model.assigned.items():
            self.parts[link] = [(name, *self._sample(export, name, link)) for name in sorted(names)
                                if not name.split('/')[-1].startswith(INTERNAL)]

    def _sample(self, export, name, link):
        component = export.components[name]
        pose = (self.model.frames[link].inverse() * self.model.to_base
                * arm.solidworks_pose(component['transform']))
        turn, origin = np.array(pose.rotation), np.array(pose.origin)
        bodies = {}
        for face in component['faces']:
            vertices, indices = export.face_arrays(face)
            v = np.frombuffer(vertices.tobytes(), dtype=np.float32).reshape(-1, 3).astype(float)
            t = np.frombuffer(indices.tobytes(), dtype=np.int32).reshape(-1, 3)
            bodies.setdefault(face['body'], []).append(v[t])
        points, normals, solids = [], [], []
        for triangles in bodies.values():
            tri = np.vstack(triangles)
            solids.append(tri @ turn.T + origin)
            a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
            cross = np.cross(b - a, c - a)
            if np.einsum('ij,ij->', a, np.cross(b, c)) < 0:
                cross = -cross  # make the normals point out of the body
            area = 0.5 * np.linalg.norm(cross, axis=1)
            good = area > 1e-14
            a, b, c, cross, area = a[good], b[good], c[good], cross[good], area[good]
            count = np.maximum(1, np.floor(area * DENSITY + self.rng.random(len(area))).astype(int))
            which = np.repeat(np.arange(len(a)), count)
            r1, r2 = np.sqrt(self.rng.random(len(which))), self.rng.random(len(which))
            local = ((1 - r1)[:, None] * a[which] + (r1 * (1 - r2))[:, None] * b[which]
                     + (r1 * r2)[:, None] * c[which])
            points.append(local @ turn.T + origin)
            normals.append((cross[which] / (2.0 * area[which])[:, None]) @ turn.T)
        return np.vstack(points), np.vstack(normals), solids

    def joint(self, name):
        return next(joint for joint in self.model.joints if joint['name'] == name)

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

    def cloud(self, link):
        return np.vstack([points for _, points, _, _ in self.parts[link]])


def inside(point, solids):
    """True if point is inside a closed body (generalized winding number above one half)."""
    for tri in solids:
        a, b, c = tri[:, 0] - point, tri[:, 1] - point, tri[:, 2] - point
        la, lb, lc = (np.linalg.norm(x, axis=1) for x in (a, b, c))
        numerator = np.einsum('ij,ij->i', a, np.cross(b, c))
        denominator = (la * lb * lc + np.einsum('ij,ij->i', a, b) * lc
                       + np.einsum('ij,ij->i', a, c) * lb + np.einsum('ij,ij->i', b, c) * la)
        if abs(np.arctan2(numerator, denominator).sum() / (2.0 * math.pi)) > 0.5:
            return True
    return False


def _segment(p, a, b):
    ab = b - a
    t = np.clip(np.einsum('ij,ij->i', p - a, ab)
                / np.maximum(np.einsum('ij,ij->i', ab, ab), 1e-30), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def depth(point, solids):
    """How far point is inside the bodies (distance to the nearest triangle), or 0 outside."""
    if not inside(point, solids):
        return 0.0
    best = math.inf
    for tri in solids:
        a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
        n = np.cross(b - a, c - a)
        length = np.linalg.norm(n, axis=1)
        ok = length > 1e-15
        n = n / np.where(ok, length, 1.0)[:, None]
        height = np.einsum('ij,ij->i', point - a, n)
        v0, v1, v2 = c - a, b - a, point - height[:, None] * n - a
        d00, d01, d11 = (np.einsum('ij,ij->i', x, y) for x, y in ((v0, v0), (v0, v1), (v1, v1)))
        d02, d12 = np.einsum('ij,ij->i', v0, v2), np.einsum('ij,ij->i', v1, v2)
        det = d00 * d11 - d01 * d01
        with np.errstate(divide='ignore', invalid='ignore'):
            u = (d11 * d02 - d01 * d12) / det
            v = (d00 * d12 - d01 * d02) / det
        within = ok & (u >= 0) & (v >= 0) & (u + v <= 1)
        p = np.broadcast_to(point, a.shape)
        edges = np.minimum(np.minimum(_segment(p, a, b), _segment(p, b, c)), _segment(p, c, a))
        best = min(best, float(np.where(within, np.abs(height), edges).min()))
    return best


def signed(tree, normals, points):
    """Signed distance to the nearest samples (negative inside), and the nearest sample."""
    distance, index = tree.query(points, k=NEIGHBOURS, distance_upper_bound=SEARCH, workers=-1)
    near = np.flatnonzero(np.isfinite(distance[:, NEIGHBOURS // 2]))
    value = np.full(len(points), np.inf)
    if len(near):
        ok = np.isfinite(distance[near])
        safe = np.where(ok, index[near], 0)
        s = np.einsum('ijk,ijk->ij', points[near][:, None, :] - tree.data[safe], normals[safe])
        inner = np.nanmedian(np.where(ok, s, np.nan), axis=1) < 0.0
        value[near] = np.where(inner, -distance[near, 0], distance[near, 0])
    return value, index[:, 0]


class Sweep:
    """One joint turning from the upright pose, its links against the rest of the arm."""

    def __init__(self, robot, name):
        self.robot, self.joint = robot, robot.joint(name)
        self.axis = np.array(self.joint['axis'])
        moving = robot.subtree(self.joint['child'])
        poses = robot.poses({})
        self.child0 = poses[self.joint['child']]
        self.fixed = self._gather([link for link in robot.parts if link not in moving], poses, None)
        # The moving links are kept in the child frame, where they never move.
        self.moving = self._gather([link for link in robot.parts if link in moving], poses,
                                   self.child0)
        self.start = self._depths(0.0)

    def _gather(self, links, poses, frame):
        """Points, normals, labels and bodies in the base frame, or in frame = (rotation, origin)."""
        points, normals, labels, solids = [], [], [], {}
        for link in links:
            r, o = poses[link]
            if frame is not None:
                r, o = frame[0].T @ r, frame[0].T @ (o - frame[1])
            for name, local, n, bodies in self.robot.parts[link]:
                points.append(local @ r.T + o)
                normals.append(n @ r.T)
                labels += [f'{link}: {name}'] * len(local)
                solids[f'{link}: {name}'] = [tri @ r.T + o for tri in bodies]
        points = np.vstack(points)
        return {'points': points, 'normals': np.vstack(normals), 'labels': np.array(labels),
                'solids': solids, 'tree': cKDTree(points)}

    def child_pose(self, angle):
        return self.robot.poses({self.joint['name']: angle})[self.joint['child']]

    def _depths(self, angle):
        r, o = self.child_pose(angle)
        moving_world = self.moving['points'] @ r.T + o
        fixed_local = (self.fixed['points'] - o) @ r
        into_fixed = signed(self.fixed['tree'], self.fixed['normals'], moving_world)
        into_moving = signed(self.moving['tree'], self.moving['normals'], fixed_local)
        return into_fixed, into_moving, moving_world, fixed_local

    def _radius(self, local):
        """Distance of a point in the child frame from the joint axis."""
        return float(np.linalg.norm(local - self.axis * (local @ self.axis)))

    def clashes(self, angle):
        """Return {(moving part, fixed part): (depth, radius)} at this angle, and the lowest
        moving point. Radius is the clashing point's distance from the joint axis."""
        (into_fixed, fixed_index), (into_moving, moving_index), world, local = self._depths(angle)
        r0, o0 = self.child0
        pairs = {}

        def add(pair, now, point):
            if now > pairs.get(pair, (0.0, 0.0))[0]:
                pairs[pair] = (now, self._radius(point))

        for i in np.flatnonzero((into_fixed < -PENETRATION) & (self.start[0][0] >= -PENETRATION)):
            pair = (self.moving['labels'][i], self.fixed['labels'][fixed_index[i]])
            solids = self.fixed['solids'][pair[1]]
            now = depth(world[i], solids)
            if now > PENETRATION and now - depth(
                    self.moving['points'][i] @ r0.T + o0, solids) > GROWTH:
                add(pair, now, self.moving['points'][i])
        for i in np.flatnonzero((into_moving < -PENETRATION) & (self.start[1][0] >= -PENETRATION)):
            pair = (self.moving['labels'][moving_index[i]], self.fixed['labels'][i])
            solids = self.moving['solids'][pair[0]]
            now = depth(local[i], solids)
            if now > PENETRATION and now - depth(
                    (self.fixed['points'][i] - o0) @ r0, solids) > GROWTH:
                add(pair, now, local[i])
        return pairs, float(world[:, 2].min())

    def first_contact(self, sign, step=1.0):
        """Degrees turned in one direction when two parts first touch, the parts, and where
        the arm meets the table (the plane under the base)."""
        angle, table = 0.0, None
        while angle < 181.0:
            angle += step
            pairs, lowest = self.clashes(sign * math.radians(angle))
            if table is None and lowest < 0.0:
                table = angle
            if pairs:
                fine = angle - step
                while fine < angle:
                    fine += 0.05
                    pairs, _ = self.clashes(sign * math.radians(fine))
                    if pairs:
                        break
                # Back off by the angle that took the deepest point that far in.
                touch = fine - max(math.degrees(inside_by / max(radius, 1e-3))
                                   for inside_by, radius in pairs.values())
                return touch, sorted(pairs), table
        return None, [], table


class Fold:
    """How far one link folds against another, for each angle of the joint between them."""

    def __init__(self, robot, link, obstacle, joint, across):
        self.robot, self.link, self.obstacle = robot, link, obstacle
        self.joint, self.across = joint, across
        self.points = robot.cloud(link)
        self.tree = cKDTree(robot.cloud(obstacle))
        axis = np.array(robot.joint(joint)['axis'])
        # No point of the link moves faster than this per radian of the joint.
        self.reach = float(np.linalg.norm(
            self.points - np.outer(self.points @ axis, axis), axis=1).max())

    def distance(self, across, angle):
        poses = self.robot.poses({self.across: across, self.joint: angle})
        ro, oo = poses[self.obstacle]
        rl, ol = poses[self.link]
        moved = self.points @ (ro.T @ rl).T + ro.T @ (ol - oo)
        return float(self.tree.query(moved, distance_upper_bound=0.25, workers=-1)[0].min())

    def contact(self, across, sign):
        """Degrees the joint turns before contact, or None within half a turn."""
        angle = 0.0
        while angle < math.pi:
            gap = self.distance(across, sign * angle)
            if gap < CONTACT:
                return math.degrees(angle)
            # Conservative advancement: no point can close more than the gap in one step.
            angle += max((gap - CONTACT / 2) / self.reach, math.radians(0.02))
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export', type=Path, help='JSON written by export_solidworks_assembly')
    parser.add_argument('--fold-step', type=float, default=5.0,
                        help='shoulder angle step of the elbow fold table in degrees')
    args = parser.parse_args()
    robot = Arm(args.export)

    print('Pitch joints from the upright pose (base yaw and wrist roll do not touch):')
    for name in PITCH_JOINTS:
        sweep = Sweep(robot, name)
        found = []
        for sign in (1, -1):
            angle, pairs, table = sweep.first_contact(sign)
            found.append(angle)
            where = f'touches at {sign * angle:+.2f}' if angle is not None else 'clear to 181'
            meets = f', meets the table at {sign * table:+.0f}' if table is not None else ''
            print(f'  {name} {"+-"[sign < 0]}: {where}{meets}', flush=True)
            for moving, fixed in pairs[:3]:
                print(f'      {moving}  ->  {fixed}')
        upper, lower = (math.floor(angle - MARGIN) if angle is not None else 180 for angle in found)
        print(f'    limits ({-lower:.1f}, {upper:.1f})')

    link, obstacle, joint, across = FOLD
    fold = Fold(robot, link, obstacle, joint, across)
    low, high = robot.joint(across)['limits']
    print(f'Elbow fold against the shoulder, {MARGIN:.0f} degree short of contact:')
    upper, lower = [], []
    for index in range(int(round((high - low) / args.fold_step)) + 1):
        shoulder = low + index * args.fold_step
        values = []
        for sign in (1, -1):
            angle = fold.contact(math.radians(shoulder), sign)
            values.append(sign * (math.floor((angle - MARGIN) * 10) / 10 if angle is not None
                                  else 180.0))
        upper.append(values[0])
        lower.append(values[1])
        print(f'  {across} {shoulder:+7.1f}: {joint} from {values[1]:+7.1f} to {values[0]:+7.1f}',
              flush=True)
    print('ELBOW_FOLD = (')
    print(f'    {low:.1f}, {args.fold_step:.1f},')
    for values in (upper, lower):
        print(textwrap.fill(', '.join(f'{value:.1f}' for value in values) + '),', width=99,
                            initial_indent='    (', subsequent_indent='     '))
    print(')')


if __name__ == '__main__':
    main()
