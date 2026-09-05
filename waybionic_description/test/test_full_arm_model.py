"""
Regression checks for the as-delivered full-arm model.

The workspace runs the 2026-09-05 mechanical drop unmodified:
urdf/full-arm-smaller.urdf is that export byte-for-byte. These checks exist to
keep it that way, and to keep the plumbing that lets it resolve its meshes from
silently going stale.
"""

from pathlib import Path
import struct
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
URDF_PATH = PACKAGE_ROOT / 'urdf' / 'full-arm-smaller.urdf'
DROP_PATH = (
    PACKAGE_ROOT.parent.parent / 'sep_05_latest_info' / 'full-arm-smaller.urdf'
)

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
    assert URDF_PATH.exists(), 'the as-delivered export is missing'
    if not DROP_PATH.exists():
        import pytest
        pytest.skip('source drop not present (installed tree)')

    assert URDF_PATH.read_bytes() == DROP_PATH.read_bytes(), (
        f'{URDF_PATH} has diverged from the 2026-09-05 drop; it must stay '
        f'byte-identical, CRLF included'
    )


def test_every_referenced_mesh_can_be_installed():
    """
    Each mesh the export names must have a source file to rename from.

    If the export references a mesh we do not hold, the install step silently
    ships a robot with missing geometry.
    """
    root = ET.parse(URDF_PATH).getroot()
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
    root = ET.parse(URDF_PATH).getroot()
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
    root = ET.parse(URDF_PATH).getroot()
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
