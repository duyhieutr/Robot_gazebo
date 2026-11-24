# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Save RTAB-Map's 3D map cloud to a coloured PLY file.

    ros2 run wsl_exploration save_cloud [path.ply]     (default ~/warehouse_cloud.ply)

Reads one /rtabmap/cloud_map message and exits. If RTAB-Map has not published
the cloud yet, it asks for it through /rtabmap/publish_map. Open the file in
MeshLab, CloudCompare or Blender.
"""

import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from wsl_exploration import extrusion_utils as eu


class CloudSaver(Node):

    def __init__(self, path):
        super().__init__('save_cloud')
        self.path = path
        self.done = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, '/rtabmap/cloud_map', self.on_cloud, qos)

    def on_cloud(self, msg):
        count = msg.width * msg.height
        fields = {f.name: f.offset for f in msg.fields}
        if count == 0 or not {'x', 'y', 'z'} <= fields.keys():
            self.get_logger().warn('Received an empty cloud; waiting for the next one.')
            return
        xyz, rgb = eu.decode_xyz_rgb(msg.data, msg.point_step, fields, count)
        eu.write_ply(self.path, xyz, rgb)
        coloured = 'rgb' in fields
        self.get_logger().info(
            f'Saved {len(xyz)} points ({"coloured" if coloured else "grey: no rgb field"}) '
            f'to {self.path}')
        self.done = True


def main(args=None):
    path = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 and
                              not sys.argv[1].startswith('--') else '~/warehouse_cloud.ply')
    rclpy.init(args=args)
    node = CloudSaver(path)
    node.get_logger().info('Waiting for /rtabmap/cloud_map (10 s)...')
    deadline = node.get_clock().now().nanoseconds + 10_000_000_000
    while rclpy.ok() and not node.done and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.done:
        node.get_logger().error(
            'No cloud received. Drive the robot a little, or run: ros2 service call '
            '/rtabmap/publish_map rtabmap_msgs/srv/PublishMap '
            '"{global_map: true, optimized: true, graph_only: false}"')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0 if node.done else 1)


if __name__ == '__main__':
    main()
