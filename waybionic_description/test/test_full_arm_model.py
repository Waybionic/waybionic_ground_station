"""Regression checks for the imported full-arm model."""

from pathlib import Path
import struct
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parent.parent


def _binary_stl_bounds(path):
    """Return the axis-aligned bounds of a binary STL."""
    data = path.read_bytes()
    triangle_count = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + triangle_count * 50

    bounds_min = [float('inf')] * 3
    bounds_max = [float('-inf')] * 3
    for triangle_index in range(triangle_count):
        offset = 84 + triangle_index * 50 + 12
        coordinates = struct.unpack_from('<9f', data, offset)
        for coordinate_index, value in enumerate(coordinates):
            axis = coordinate_index % 3
            bounds_min[axis] = min(bounds_min[axis], value)
            bounds_max[axis] = max(bounds_max[axis], value)
    return bounds_min, bounds_max


def test_base_center_of_mass_is_inside_its_mesh_bounds():
    """Keep the base inertia origin in the mesh/link coordinate frame."""
    urdf_path = PACKAGE_ROOT / 'urdf' / 'full_arm_mar24.urdf'
    root = ET.parse(urdf_path).getroot()
    base_link = root.find('link[@name="base_link"]')
    assert base_link is not None

    inertial_origin = base_link.find('inertial/origin')
    assert inertial_origin is not None
    center_of_mass = [
        float(value)
        for value in inertial_origin.get('xyz').split()
    ]

    mesh_path = PACKAGE_ROOT / 'meshes' / 'base_link.STL'
    bounds_min, bounds_max = _binary_stl_bounds(mesh_path)
    assert all(
        lower <= coordinate <= upper
        for coordinate, lower, upper in zip(
            center_of_mass,
            bounds_min,
            bounds_max,
        )
    ), (
        f'base COM {center_of_mass} is outside its mesh bounds '
        f'{bounds_min} .. {bounds_max}'
    )
