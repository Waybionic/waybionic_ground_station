"""
Regression checks for the as-delivered full-arm model.

The workspace runs the 2026-09-05 mechanical drop unmodified:
urdf/full-arm-smaller.urdf is that export byte-for-byte. These checks exist to
keep it that way, and to keep the plumbing that lets it resolve its meshes from
silently going stale.
"""

import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parent.parent

# The articulated model MoveIt plans over.
URDF_PATH = PACKAGE_ROOT / 'urdf' / 'full_arm_smaller.urdf'
# The assumption-free model: the export's parts, fixed, zero DOF.
AS_EXPORTED_PATH = PACKAGE_ROOT / 'urdf' / 'full_arm_smaller_as_exported.urdf'
# The mechanical drop itself, carried byte-for-byte.
EXPORT_PATH = PACKAGE_ROOT / 'urdf' / 'full-arm-smaller.urdf'
DROP_PATH = (
    PACKAGE_ROOT.parent.parent / 'sep_05_latest_info' / 'full-arm-smaller.urdf'
)

# Structural links of the articulated model, and the mesh each renders.
STRUCTURAL_MESHES = {
    'base_link': 'bottom_base_assembly.STL',
    'sweep': 'sweep.STL',
    'shoulder': 'shouldersplit.STL',
    'elbow': 'third_joint_bend.STL',
    'forearm': 'diff_assembly_pulley.STL',
    'tool': 'biomed_lock_mech.STL',
}

# The serial chain the SRDF planning group depends on, base to tip.
EXPECTED_CHAIN = ['base_link', 'sweep', 'shoulder', 'elbow', 'forearm', 'tool']

# The export names its meshes with spaces; ours are snake_case. CMakeLists
# installs a renamed copy of each so package://full-arm-smaller/ resolves.
DELIVERED_TO_OURS = {
    'bottom base assembly.STL': 'bottom_base_assembly.STL',
    'Sweep.STL': 'sweep.STL',
    'shouldersplit.STL': 'shouldersplit.STL',
    '3rd joint bend.STL': 'third_joint_bend.STL',
    'diff-assembly-pulley.STL': 'diff_assembly_pulley.STL',
    'biomed lock mech.STL': 'biomed_lock_mech.STL',
    'Stepper.STL': 'stepper.STL',
    'nema23.STL': 'nema23.STL',
    'outerring.STL': 'outerring.STL',
    'bevel gear.STL': 'bevel_gear.STL',
    'heatSetInsert.STL': 'heat_set_insert.STL',
    'm3.STL': 'm3.STL',
    'Full Arm Smaller.STL': 'Full Arm Smaller.STL',
}

# The six joints the export declares movable. srdf/waybionic.srdf lists these
# as its planning group and the ros2_control block drives them.
EXPORT_MOVABLE_JOINTS = {
    '3rd joint bend',
    'm3',
    'diff-assembly-pulley',
    'nema23',
    'bevel gear',
    'biomed lock mech',
}


def test_shipped_export_is_byte_identical_to_the_drop():
    """
    Keep urdf/full-arm-smaller.urdf exactly as mechanical delivered it.

    Its whole value is that nobody edited it. A byte difference against
    sep_05_latest_info/ means someone has, and the fix is to change whatever
    consumes it rather than the export.
    """
    assert EXPORT_PATH.exists(), 'the as-delivered export is missing'
    if not DROP_PATH.exists():
        import pytest
        pytest.skip('source drop not present (installed tree)')

    assert EXPORT_PATH.read_bytes() == DROP_PATH.read_bytes(), (
        f'{EXPORT_PATH} has diverged from the 2026-09-05 drop; it must stay '
        f'byte-identical, CRLF included'
    )


def test_every_referenced_mesh_can_be_installed():
    """
    Each mesh the export names must have a source file to rename from.

    If the export references a mesh we do not hold, the install step silently
    ships a robot with missing geometry.
    """
    root = ET.parse(EXPORT_PATH).getroot()
    wanted = {
        mesh.get('filename').rsplit('/', 1)[-1]
        for mesh in root.iter('mesh')
    }

    unmapped = wanted - set(DELIVERED_TO_OURS)
    assert not unmapped, f'export references meshes with no mapping: {unmapped}'

    for delivered in wanted:
        ours = PACKAGE_ROOT / 'meshes' / DELIVERED_TO_OURS[delivered]
        assert ours.exists(), f'{ours.name} missing (needed for "{delivered}")'


def test_export_still_declares_the_joints_the_moveit_config_drives():
    """
    The SRDF group and ros2_control block name these six joints.

    They are hardcoded in waybionic_moveit_config, so a re-export that renames
    or drops one would break move_group with a less obvious error than this.
    """
    root = ET.parse(EXPORT_PATH).getroot()
    movable = {
        joint.get('name')
        for joint in root.findall('joint')
        if joint.get('type') != 'fixed'
    }
    assert movable == EXPORT_MOVABLE_JOINTS, (
        f'movable joints changed: {movable ^ EXPORT_MOVABLE_JOINTS}. '
        f'Update waybionic_moveit_config to match.'
    )


def test_geometry_is_still_duplicated_by_the_export():
    """
    Document the known duplication rather than pretend it is fixed.

    The export's root link carries the whole assembly as one mesh, and the
    twelve part meshes sum to exactly the same triangle count - so every
    surface is drawn twice. We run the export as delivered, so this is
    expected; the test exists so the number is recorded and a future drop that
    fixes it fails loudly and gets this file updated.
    """
    root = ET.parse(EXPORT_PATH).getroot()
    referenced = {
        mesh.get('filename').rsplit('/', 1)[-1]
        for mesh in root.iter('mesh')
    }

    def triangles(delivered):
        path = PACKAGE_ROOT / 'meshes' / DELIVERED_TO_OURS[delivered]
        return struct.unpack_from('<I', path.read_bytes(), 80)[0]

    aggregate = triangles('Full Arm Smaller.STL')
    parts = sum(
        triangles(name)
        for name in referenced
        if name != 'Full Arm Smaller.STL'
    )
    assert aggregate == parts == 218740, (
        f'aggregate={aggregate}, parts={parts}. If these no longer match, '
        f'mechanical has fixed the duplicated-geometry defect - update this '
        f'test and the disable_collisions block in srdf/waybionic.srdf.'
    )


def _binary_stl_bounds_cached(path, _cache={}):
    """Return the axis-aligned bounds of a binary STL, memoised."""
    if path not in _cache:
        data = path.read_bytes()
        count = struct.unpack_from('<I', data, 80)[0]
        assert len(data) == 84 + count * 50
        lo = [float('inf')] * 3
        hi = [float('-inf')] * 3
        for i in range(count):
            for k, v in enumerate(struct.unpack_from('<9f', data, 84 + i * 50 + 12)):
                lo[k % 3] = min(lo[k % 3], v)
                hi[k % 3] = max(hi[k % 3], v)
        _cache[path] = (lo, hi)
    return _cache[path]


def _pose(element):
    """Return (3x3 rotation as nested lists, translation) for an <origin>."""
    if element is None:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0.0, 0.0, 0.0]
    xyz = [float(v) for v in element.get('xyz', '0 0 0').split()]
    r, p, y = (float(v) for v in element.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = (
        math.cos(r), math.sin(r), math.cos(p),
        math.sin(p), math.cos(y), math.sin(y),
    )
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], xyz


def _mesh_bounds_in_link_frame(link, mesh_name):
    """Return the link-frame AABB of a link's visual mesh."""
    rot, tr = _pose(link.find('visual/origin'))
    lo, hi = _binary_stl_bounds_cached(PACKAGE_ROOT / 'meshes' / mesh_name)
    corners = [
        [sum(rot[a][k] * pt[k] for k in range(3)) + tr[a] for a in range(3)]
        for pt in [
            [x, y, z]
            for x in (lo[0], hi[0])
            for y in (lo[1], hi[1])
            for z in (lo[2], hi[2])
        ]
    ]
    return (
        [min(c[a] for c in corners) for a in range(3)],
        [max(c[a] for c in corners) for a in range(3)],
    )


def test_link_frames_sit_on_the_parts_they_belong_to():
    """
    Each structural link's frame must be near its own geometry.

    This is the defect that made the raw export unusable: four of its six link
    frames sat at the assembly origin, 1.5-1.8 m from their part. The model
    still renders correctly at the zero pose, so nothing looks wrong until a
    joint moves and swings its child through a metre-wide arc. Guard the fix.
    """
    root = ET.parse(URDF_PATH).getroot()
    for link_name, mesh_name in STRUCTURAL_MESHES.items():
        link = root.find(f'link[@name="{link_name}"]')
        assert link is not None, f'{link_name} is missing'
        lo, hi = _mesh_bounds_in_link_frame(link, mesh_name)
        centroid = [(a + b) / 2 for a, b in zip(lo, hi)]
        distance = math.sqrt(sum(v * v for v in centroid))
        assert distance < 0.5, (
            f'{link_name} frame is {distance:.3f} m from its geometry; the '
            f'joint would rotate the part about a distant axis'
        )


def test_planning_group_chain_is_intact():
    """Plan over this chain; a break makes srdf/waybionic.srdf unloadable."""
    root = ET.parse(URDF_PATH).getroot()
    parent_of = {
        j.find('child').get('link'): j.find('parent').get('link')
        for j in root.findall('joint')
    }
    for child, parent in zip(EXPECTED_CHAIN[1:], EXPECTED_CHAIN[:-1]):
        assert parent_of.get(child) == parent, (
            f'expected {child} to parent to {parent}, got {parent_of.get(child)}'
        )
    assert parent_of.get('base_link') == 'world'


def test_actuated_joints_are_revolute_and_bounded():
    """
    The export delivered zero usable DOF; make sure five survive here.

    Its five prismatic joints were pinned at lower=0 upper=0 and its one
    continuous joint was unbounded, so nothing could move.
    """
    root = ET.parse(URDF_PATH).getroot()
    for i in range(1, 6):
        joint = root.find(f'joint[@name="joint{i}"]')
        assert joint is not None, f'joint{i} is missing'
        assert joint.get('type') == 'revolute'
        limit = joint.find('limit')
        assert limit is not None, f'joint{i} has no <limit>'
        assert float(limit.get('upper')) > float(limit.get('lower'))
        assert float(limit.get('effort')) > 0
        assert float(limit.get('velocity')) > 0


def test_derived_models_do_not_reference_the_aggregate_mesh():
    """
    Keep the duplicated aggregate mesh out of our own models.

    "Full Arm Smaller.STL" is the whole assembly as one 218740-triangle body,
    and the twelve part meshes sum to exactly that. It lives in meshes/ because
    the as-delivered export names it - but either derived model picking it up
    alongside the parts would draw every surface twice.
    """
    for path in (URDF_PATH, AS_EXPORTED_PATH):
        root = ET.parse(path).getroot()
        referenced = {
            m.get('filename').rsplit('/', 1)[-1] for m in root.iter('mesh')
        }
        assert 'Full Arm Smaller.STL' not in referenced, path.name
        assert len(referenced) == 12, f'{path.name} refs {len(referenced)}'
        total = sum(
            struct.unpack_from(
                '<I', (PACKAGE_ROOT / 'meshes' / n).read_bytes(), 80,
            )[0]
            for n in referenced
        )
        assert total == 218740, f'{path.name} totals {total}, expected 218740'


def test_as_exported_model_asserts_no_kinematics():
    """
    The as-exported model must stay free of every inferred value.

    Its whole purpose is that it guesses nothing, so a movable joint appearing
    in it means someone has added kinematics the export does not contain.
    """
    root = ET.parse(AS_EXPORTED_PATH).getroot()
    movable = [
        j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed'
    ]
    assert not movable, (
        f'{movable} are movable; articulated values belong in '
        f'full_arm_smaller.urdf'
    )
    assert not root.findall('.//axis')
    assert not root.findall('.//limit')


def test_derived_model_names_contain_no_spaces():
    """Keep spaces out of names, since they become tf frame ids."""
    for path in (URDF_PATH, AS_EXPORTED_PATH):
        root = ET.parse(path).getroot()
        offenders = [
            e.get('name')
            for e in root.findall('link') + root.findall('joint')
            if ' ' in e.get('name')
        ]
        assert not offenders, f'{path.name}: {offenders}'
