# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Building-like 3D model of the 2D map: occupied cells extruded to a fixed height.

    /rtabmap/map --> extrude --> /exploration/extruded_map (Marker, RViz)
                                 ~/export service        --> ~/warehouse_3d.obj

The height is a parameter (default 2.4 m, the rack height), not a measurement.
The marker is republished every few seconds so an RViz that starts later shows it.
"""

import os

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from wsl_exploration import extrusion_utils as eu

WALL_COLOUR = (0.35, 0.55, 0.85)
FLOOR_COLOUR = (0.22, 0.22, 0.24)


class MapExtruder(Node):

    def __init__(self):
        super().__init__('map_extruder')
        self.declare_parameter('map_topic', '/rtabmap/map')
        self.declare_parameter('height', 2.4)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('min_component_cells', 8)
        self.declare_parameter('smooth_cells', 1)
        self.declare_parameter('with_floor', True)
        self.declare_parameter('publish_period', 3.0)
        self.declare_parameter('export_path', '~/warehouse_3d.obj')

        self.map = None
        self.version = 0
        self.built_version = -1
        self.vertices = np.zeros((0, 3))
        self.marker = None

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                 self.on_map, map_qos)
        self.pub = self.create_publisher(Marker, '/exploration/extruded_map', 1)
        self.create_service(Trigger, '~/export', self.on_export)
        self.create_timer(self.get_parameter('publish_period').value, self.tick)
        self.get_logger().info(
            'Map extruder started. Save the model with: '
            'ros2 service call /map_extruder/export std_srvs/srv/Trigger')

    def on_map(self, msg):
        self.map = msg
        self.version += 1

    def build(self):
        info = self.map.info
        grid = np.array(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        ox, oy = info.origin.position.x, info.origin.position.y
        vertices, shades = eu.extrude(
            grid, info.resolution, ox, oy,
            height=self.get_parameter('height').value,
            occupied_min=self.get_parameter('occupied_threshold').value,
            min_cells=self.get_parameter('min_component_cells').value,
            smooth_cells=self.get_parameter('smooth_cells').value)
        self.vertices = vertices

        points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in vertices]
        colours = [ColorRGBA(r=WALL_COLOUR[0] * s, g=WALL_COLOUR[1] * s,
                             b=WALL_COLOUR[2] * s, a=1.0) for s in shades]
        if self.get_parameter('with_floor').value:
            for x, y, z in eu.floor_quad(grid, info.resolution, ox, oy):
                points.append(Point(x=float(x), y=float(y), z=float(z) - 0.01))
                colours.append(ColorRGBA(r=FLOOR_COLOUR[0], g=FLOOR_COLOUR[1],
                                         b=FLOOR_COLOUR[2], a=1.0))
        marker = Marker()
        marker.header.frame_id = self.map.header.frame_id
        marker.ns, marker.id = 'extruded_map', 0
        marker.type, marker.action = Marker.TRIANGLE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        marker.points, marker.colors = points, colours
        self.marker = marker
        self.get_logger().info(
            f'Extruded map: {len(vertices) // 3} triangles, height '
            f'{self.get_parameter("height").value:.1f} m.')

    def tick(self):
        if self.map is None:
            self.get_logger().info('Waiting for the map...', throttle_duration_sec=10.0)
            return
        if self.version != self.built_version:
            self.build()
            self.built_version = self.version
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)

    def on_export(self, _request, response):
        if len(self.vertices) == 0:
            response.success = False
            response.message = 'Nothing to export yet: no occupied cells in the map.'
            return response
        path = os.path.expanduser(self.get_parameter('export_path').value)
        eu.write_obj(path, self.vertices)
        response.success = True
        response.message = f'Wrote {len(self.vertices) // 3} triangles to {path}'
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapExtruder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
