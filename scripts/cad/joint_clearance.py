#!/usr/bin/env python3
"""Find how far the elbow folds before the forearm meets the shoulder or the base.

    python3 scripts/cad/joint_clearance.py build/cad/arm_export.json

After a CAD change, run scripts/cad/exact_limits.py for the joint limits first, then copy the
ELBOW_FOLDS this prints into build_arm_description.py and rebuild the URDF. This script needs
numpy and scipy; the build script does not.

Each part's SolidWorks tessellation is sampled densely in its link frame. Gearbox internals are
skipped: they are enclosed, and static CAD cannot model their eccentric motion. How far the
elbow folds before the forearm meets the shoulder or the base depends on the shoulder angle, so
that is tabulated against it, up to the samples' first contact (CONTACT). The base is round, so
its gap is measured in distance from the yaw axis and height, which is the gap at the nearest
yaw. The collision shapes are too coarse for these pairs, and the teleop collision check uses
the tables.
"""

import argparse
import importlib.util
import math
from pathlib import Path
import textwrap

import numpy as np
from scipy.spatial import cKDTree

DENSITY = 2.0e6        # samples per square metre of surface
CONTACT = 0.0005       # m between forearm and shoulder or base samples that counts as touching
FAR = 0.25             # m beyond which a fold's gap is only bounded
SPARSE = 0.004         # m cells of the samples that find a fold's closest region
MARGIN = 0.0           # degrees kept clear of contact: the tables stop where the parts touch
INTERNAL = ('discs-sweep', 'disc-bearing', 'loose_disc', 'inner_bearing', 'shaft-sweep',
            'NEMA23SHAFT', 'outer_ball_bearing', 'outer-ring-bearing', 'bearing_brace',
            'updatednema23bearingbrace', 'top-sweep')
# The link that folds, what it folds against, the joint tabulated, the joint across and its step
# in degrees. The base's limit bends sharply near some shoulder angles, so it needs finer steps.
FOLDS = (('forearm_link', 'shoulder_link', 'joint_3', 'joint_2', 5.0),
         ('forearm_link', 'base_link', 'joint_3', 'joint_2', 2.5))

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


def radial(points):
    """Distance from the z axis and height, which base yaw leaves unchanged."""
    return np.column_stack((np.hypot(points[:, 0], points[:, 1]), points[:, 2]))


class Fold:
    """How far one link folds against another, for each angle of the joint between them."""

    def __init__(self, robot, link, obstacle, joint, across):
        self.robot, self.link, self.obstacle = robot, link, obstacle
        self.joint, self.across = joint, across
        self.points = robot.cloud(link)
        # Base yaw turns the link about the base's z axis: compared in radial(), the gap is the
        # one at the nearest yaw. There the base's samples crowd onto its profile, so keep one
        # per 0.1 mm cell.
        self.any_yaw = obstacle == 'base_link'
        cloud = robot.cloud(obstacle)
        if self.any_yaw:
            cloud = np.unique(np.round(radial(cloud), 4), axis=0)
        self.tree = cKDTree(cloud)
        axis = np.array(robot.joint(joint)['axis'])
        # No point of the link moves faster than this per radian of the joint.
        self.reach = float(np.linalg.norm(
            self.points - np.outer(self.points @ axis, axis), axis=1).max())
        # One sample per SPARSE cell finds the closest region fast. Every sample is within
        # self.cover of its sparse representative, so only those near the closest need checking.
        cells = np.floor(self.points / SPARSE).astype(np.int64)
        self.sparse = self.points[np.unique(cells, axis=0, return_index=True)[1]]
        cover, self.representative = cKDTree(self.sparse).query(self.points, workers=-1)
        self.cover = float(cover.max())

    def distance(self, across, angle):
        """The gap, or a lower bound on it when it is beyond FAR."""
        poses = self.robot.poses({self.across: across, self.joint: angle})
        ro, oo = poses[self.obstacle]
        rl, ol = poses[self.link]
        turn, shift = ro.T @ rl, ro.T @ (ol - oo)

        def gaps(points):
            moved = points @ turn.T + shift
            if self.any_yaw:
                moved = radial(moved)
            return self.tree.query(moved, distance_upper_bound=FAR, workers=-1)[0]

        sparse = gaps(self.sparse)
        best = float(sparse.min())
        if best >= FAR:
            return FAR - self.cover
        near = sparse[self.representative] < best + self.cover
        return min(best, float(gaps(self.points[near]).min()), FAR - self.cover)

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
    args = parser.parse_args()
    robot = Arm(args.export)

    tables = {}
    for link, obstacle, joint, across, step in FOLDS:
        fold = Fold(robot, link, obstacle, joint, across)
        low, high = robot.joint(across)['limits']
        # The table ends exactly at the other joint's limits, in steps no longer than step.
        count = math.ceil((high - low) / step - 1e-9)
        step = (high - low) / count
        print(f'Elbow fold against {obstacle}, until contact:')
        upper, lower = [], []
        for index in range(count + 1):
            shoulder = low + index * step
            values = []
            for sign in (1, -1):
                angle = fold.contact(math.radians(shoulder), sign)
                values.append(sign * (math.floor((angle - MARGIN) * 10) / 10 if angle is not None
                                      else 180.0))
            upper.append(values[0])
            lower.append(values[1])
            print(f'  {across} {shoulder:+7.1f}: {joint} from {values[1]:+7.1f} to '
                  f'{values[0]:+7.1f}', flush=True)
        tables[obstacle] = (low, step, upper, lower)
    print('ELBOW_FOLDS = {')
    for obstacle, (low, step, upper, lower) in tables.items():
        print(f"    '{obstacle}': (")
        print(f'        {low!r}, {step!r},')
        for values in (upper, lower):
            print(textwrap.fill(', '.join(f'{value:.1f}' for value in values) + '),', width=99,
                                initial_indent='        (', subsequent_indent='         '))
        print('    ),')
    print('}')


if __name__ == '__main__':
    main()
