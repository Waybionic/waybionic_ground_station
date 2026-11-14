#!/usr/bin/env python3
"""Check the teleop collision model against the exact CAD at random poses.

    python3 scripts/cad/check_collision_model.py build/cad/arm_export.json [--poses 320000]

This needs numpy, scipy and python-fcl, like exact_limits.py. The model is what teleop and the
joint sliders stop at: the URDF's joint limits, collision shapes, fold tables and hulls, read by
waybionic_teleop.collision. It must never call a pose clear where two CAD parts on different
links, or a part and the tool's placeholder box, intersect, or where a part reaches below the
ground under the base. Poses are drawn uniformly within the joint limits, then with one joint at
a limit, then at the ends of straight moves from clear poses, where the model or a joint limit
first stops them. FCL checks each pose the model calls clear on the triangle meshes. Parts that
already touch upright, such as bearings on their seats and meshing gears, are left out, as in
exact_limits.py. The script exits with status 1 if any pose clashes.
"""

import argparse
import collections
import importlib.util
import math
import multiprocessing
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as ET

import fcl
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'waybionic_teleop'))
from waybionic_teleop.collision import ArmCollision  # noqa: E402
from waybionic_teleop.kinematics import joint_limits  # noqa: E402

spec = importlib.util.spec_from_file_location(
    'exact_limits', Path(__file__).resolve().with_name('exact_limits.py'))
exact = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exact)

URDF = (ROOT / 'waybionic_description' / 'urdf' / 'waybionic_arm.urdf').read_text(
    encoding='utf-8')
JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
BODY = {'base_link', 'shoulder_link'}
STEP = math.radians(0.25)


class Truth:
    """Every CAD part as a triangle mesh, and the tool's box, posed by the joint angles."""

    def __init__(self, export):
        self.robot = exact.Robot(export)
        # (link, name, hull points in the link frame, FCL object, its centre in the link frame)
        self.items = [(link, name.split('/')[-1], hull,
                       fcl.CollisionObject(mesh, fcl.Transform()), np.zeros(3))
                      for link, name, hull, mesh in self.robot.parts]
        box = ET.fromstring(URDF).find("link[@name='tool_link']/collision")
        centre = np.array([float(v) for v in box.find('origin').get('xyz').split()])
        size = np.array([float(v) for v in box.find('geometry/box').get('size').split()])
        corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        self.items.append(('tool_link', 'tool box', corners * size / 2 + centre,
                           fcl.CollisionObject(fcl.Box(*size), fcl.Transform()), centre))
        tool = ET.fromstring(URDF).find("joint[@name='tool_joint']/origin")
        self.tool = np.array([float(v) for v in tool.get('xyz').split()])
        self.place({})
        self.pairs = [(a, b) for i, a in enumerate(self.items) for b in self.items[i + 1:]
                      if a[0] != b[0] and fcl.distance(a[3], b[3], fcl.DistanceRequest(),
                                                       fcl.DistanceResult()) > exact.MATED]

    def place(self, q):
        poses = self.robot.poses(q)
        r, o = poses['wrist_roll_link']
        poses['tool_link'] = (r, o + r @ self.tool)
        self.boxes, self.lowest = {}, (math.inf, None)
        for link, name, points, obj, centre in self.items:
            r, o = poses[link]
            obj.setTransform(fcl.Transform(r, o + r @ centre))
            world = points @ r.T + o
            self.boxes[id(obj)] = (world.min(axis=0), world.max(axis=0))
            if link not in BODY and world[:, 2].min() < self.lowest[0]:
                self.lowest = (float(world[:, 2].min()), link)

    def clashes(self, q):
        """Return what clashes at these joint angles: ('ground', link) or (link, link) pairs."""
        self.place(q)
        found = []
        if self.lowest[0] < -1e-6:
            found.append(('ground', self.lowest[1]))
        for a, b in self.pairs:
            (a0, a1), (b0, b1) = self.boxes[id(a[3])], self.boxes[id(b[3])]
            if np.any(a0 > b1 + 1e-9) or np.any(b0 > a1 + 1e-9):
                continue
            if fcl.collide(a[3], b[3], fcl.CollisionRequest(), fcl.CollisionResult()):
                found.append((f'{a[0]}: {a[1]}', f'{b[0]}: {b[1]}'))
        return found


_TRUTH = None


def check(task):
    seed, count = task
    rng = random.Random(seed)
    model = ArmCollision.from_urdf(URDF)
    limits = joint_limits(URDF, JOINTS)
    clear, kinds, examples = 0, collections.Counter(), []

    def pose():
        return {joint: rng.uniform(*limits[joint]) for joint in JOINTS}

    def judge(q):
        nonlocal clear
        clear += 1
        found = _TRUTH.clashes(q)
        if found:
            at = [joint for joint in JOINTS
                  if min(abs(q[joint] - bound) for bound in limits[joint]) < 1e-9]
            links = sorted({' and '.join(sorted(item.split(':')[0] for item in pair))
                            for pair in found})
            kinds['; '.join(links) + (' at the limit of ' + ', '.join(at) if at else '')] += 1
            if len(examples) < 3:
                examples.append((', '.join(f'{joint} {math.degrees(q[joint]):.2f}'
                                           for joint in JOINTS), found[:2]))

    for _ in range(count):
        q = pose()
        if not model.hits(q):
            judge(q)
    for _ in range(count // 4):
        q = pose()
        joint = rng.choice(JOINTS)
        q[joint] = limits[joint][rng.randrange(2)]
        if not model.hits(q):
            judge(q)
    for _ in range(count // 20):
        q = pose()
        if model.hits(q):
            continue
        direction = [rng.gauss(0.0, 1.0) for _ in JOINTS]
        norm = math.sqrt(sum(value * value for value in direction))
        while True:
            step = {joint: q[joint] + STEP * value / norm
                    for joint, value in zip(JOINTS, direction)}
            if (any(not limits[j][0] <= step[j] <= limits[j][1] for j in JOINTS)
                    or model.hits(step)):
                judge(q)
                break
            q = step
    return clear, kinds, examples


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('export', type=Path, help='JSON written by export_solidworks_assembly')
    parser.add_argument('--poses', type=int, default=320000, help='random poses to draw')
    args = parser.parse_args()
    global _TRUTH
    _TRUTH = Truth(args.export)
    workers = multiprocessing.cpu_count()
    with multiprocessing.get_context('fork').Pool(workers) as pool:
        results = pool.map(check, [(seed, args.poses // workers) for seed in range(workers)])
    clear = sum(result[0] for result in results)
    kinds = sum((result[1] for result in results), collections.Counter())
    print(f'{clear} poses the model calls clear; {sum(kinds.values())} of them clash in the CAD')
    for kind, count in kinds.most_common():
        print(f'  {count}: {kind}')
    for _, _, examples in results:
        for q, found in examples:
            print(f'  for example {q}: {found}')
    return 1 if kinds else 0


if __name__ == '__main__':
    sys.exit(main())
