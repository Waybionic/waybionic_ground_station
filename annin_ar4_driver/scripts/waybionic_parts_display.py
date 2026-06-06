#!/usr/bin/env python3
"""
AI GENERATED

Publishes waybionic arm parts from annin_ar4_description/meshes/waybionic/
as a MarkerArray so they appear in RViz alongside the robot model.

Parts are color-coded by sub-assembly group and centered at the world origin.
Topic: /waybionic_parts  (MarkerArray, latched/transient-local)
"""

import os
import re
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from ament_index_python.packages import get_package_share_directory

# RGB colors per sub-assembly (r, g, b, a)
# For 7 subgroups - allows you to distinguish
_GROUPS = [
    ('base',         (0.25, 0.45, 0.85, 0.85), ['base_updated', 'base_funnel', 'armcart', 'dome', 'new_dome', 'lid']),
    ('shoulder',     (0.20, 0.80, 0.40, 0.85), ['shoulder', 'elbow', 'base_to_shoulder', 'basetoshoulder', 'pipe', 'pvc']),
    ('differential', (0.85, 0.20, 0.20, 0.85), ['differential', 'diff', 'keyed_shaft', 'key_to_diff', 'bevel', 'capfor']),
    ('drive',        (0.90, 0.55, 0.10, 0.85), ['stepper', 'carrier', 'slip_ring', 'slipring', '2wireslipring']),
    ('gears_belts',  (0.60, 0.20, 0.85, 0.85), ['spur_gear', 'gt2', 'pulley', 'timing_belt', 'planet', 'outer_ring']),
    ('joints',       (0.15, 0.75, 0.85, 0.85), ['3rd_joint', 'joint', 'revised', 'bearing', 'shaft']),
    ('structure',    (0.65, 0.65, 0.65, 0.85), []),  # catch-all
]

# World-frame offset to center the GLB assembly at the RViz world origin.
# Full arm bounding box center from the GLB export (meters):
_CENTER_OFFSET = (-1.539, -1.134, -0.903)


def _color_for(filename: str) -> ColorRGBA:
    key = filename.lower().replace('-', '_').replace(' ', '_')
    for _, rgba, keywords in _GROUPS:
        if any(kw in key for kw in keywords):
            return ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    r, g, b, a = _GROUPS[-1][1]
    return ColorRGBA(r=r, g=g, b=b, a=a)


class WayBionicPartsDisplay(Node):

    def __init__(self):
        super().__init__('waybionic_parts_display')

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._pub = self.create_publisher(MarkerArray, '/waybionic_parts', latched_qos)

        meshes_dir = os.path.join(
            get_package_share_directory('annin_ar4_description'),
            'meshes', 'waybionic',
        )

        if not os.path.isdir(meshes_dir):
            self.get_logger().error(f'Meshes directory not found: {meshes_dir}')
            return

        stl_files = sorted(f for f in os.listdir(meshes_dir) if f.lower().endswith('.stl'))
        if not stl_files:
            self.get_logger().warn('No STL files found in waybionic meshes directory.')
            return

        array = MarkerArray()
        for idx, filename in enumerate(stl_files):
            m = Marker()
            m.header.frame_id = 'world'
            m.ns = 'waybionic_parts'
            m.id = idx
            m.type = Marker.MESH_RESOURCE
            m.action = Marker.ADD
            m.mesh_resource = f'package://annin_ar4_description/meshes/waybionic/{filename}'
            m.mesh_use_embedded_materials = False

            # Center the GLB assembly at the RViz world origin
            m.pose = Pose(
                position=Point(
                    x=_CENTER_OFFSET[0],
                    y=_CENTER_OFFSET[1],
                    z=_CENTER_OFFSET[2],
                ),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
            m.scale = Vector3(x=1.0, y=1.0, z=1.0)
            m.color = _color_for(filename)
            array.markers.append(m)

        self._pub.publish(array)
        self.get_logger().info(f'Published {len(array.markers)} waybionic part markers.')


def main(args=None):
    rclpy.init(args=args)
    # Displays arm
    node = WayBionicPartsDisplay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
