# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Autonomous exploration: drive Nav2 to frontiers of the RTAB-Map occupancy grid
until no reachable unknown space is left, then (optionally) return to the start.

    /rtabmap/map  --> frontier detection --> NavigateToPose goal --> Nav2
         ^                                                             |
         +------------ robot drives, RTAB-Map extends the map <--------+

One goal at a time. A goal is dropped when it is reached, rejected by the
planner, times out, or its frontier has been explored on the way (the map
changed under it). Reached and failed goals are blacklisted so the robot never
loops on a frontier it cannot clear (for example a gap narrower than the robot,
or a window the LiDAR sees through).

RViz: /exploration/frontiers shows frontier cells (green), the current goal
(red arrow) and blacklisted goals (grey).
"""

import math

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from wsl_exploration import frontier_utils as fu

IDLE, NAVIGATING, RETURNING, DONE = 'idle', 'navigating', 'returning', 'done'


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        self.declare_parameter('map_topic', '/rtabmap/map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('free_threshold', 25)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('clearance', 0.4)
        self.declare_parameter('min_frontier_length', 0.5)
        self.declare_parameter('min_goal_distance', 0.6)
        self.declare_parameter('blacklist_radius', 0.7)
        self.declare_parameter('length_weight', 0.5)
        self.declare_parameter('tick_period', 1.0)
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('no_frontier_patience', 5)
        self.declare_parameter('return_to_start', True)

        self.map = None
        self.state = IDLE
        self.goal_handle = None
        self.goal_xy = None
        self.goal_sent_at = None
        self.start_xy = None
        self.start_yaw = 0.0
        self.blacklist = []
        self.no_frontier_count = 0
        self.retry_after = None
        self.goals_sent = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # RTAB-Map publishes the grid reliable + transient local.
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                 self.on_map, map_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 5)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(self.get_parameter('tick_period').value, self.tick)
        self.get_logger().info('Frontier explorer started: waiting for map and Nav2.')

    # ------------------------------------------------------------------ inputs

    def on_map(self, msg):
        self.map = msg

    def robot_pose(self):
        """(x, y, yaw) of the robot in the map frame, or None until TF is up."""
        if self.map is None:
            return None
        try:
            t = self.tf_buffer.lookup_transform(
                self.map.header.frame_id, self.get_parameter('robot_frame').value,
                rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def current_frontiers(self):
        info = self.map.info
        grid = np.array(self.map.data, dtype=np.int16).reshape(info.height, info.width)
        return fu.find_frontiers(
            grid, info.resolution, info.origin.position.x, info.origin.position.y,
            free_max=self.get_parameter('free_threshold').value,
            occupied_min=self.get_parameter('occupied_threshold').value,
            clearance_m=self.get_parameter('clearance').value,
            min_length_m=self.get_parameter('min_frontier_length').value)

    # -------------------------------------------------------------- main loop

    def tick(self):
        if self.state == DONE:
            return
        pose = self.robot_pose()
        if pose is None:
            self.get_logger().info('Waiting for the map and the map -> robot transform...',
                                   throttle_duration_sec=10.0)
            return
        x, y, yaw = pose
        if self.start_xy is None:
            self.start_xy, self.start_yaw = (x, y), yaw
            self.get_logger().info(f'Start pose recorded: ({x:.2f}, {y:.2f}).')

        frontiers = self.current_frontiers()
        self.publish_markers(frontiers)

        if self.state == NAVIGATING:
            self.supervise_goal(frontiers)
        elif self.state == IDLE:
            self.choose_next(frontiers, x, y)

    def supervise_goal(self, frontiers):
        """Cancel the active goal if it ran out of time or its frontier is gone."""
        if self.goal_handle is None:
            return  # goal request still in flight
        elapsed = (self.get_clock().now() - self.goal_sent_at).nanoseconds * 1e-9
        info = self.map.info
        if elapsed > self.get_parameter('goal_timeout').value:
            self.get_logger().warn('Goal timed out: giving up on this frontier.')
            self.blacklist.append(self.goal_xy)
            self.cancel_goal()
        elif not fu.still_frontier(frontiers, self.goal_xy[0], self.goal_xy[1],
                                   self.get_parameter('blacklist_radius').value,
                                   info.resolution, info.origin.position.x,
                                   info.origin.position.y):
            self.get_logger().info('Frontier explored on the way: choosing a new one.')
            self.cancel_goal()

    def choose_next(self, frontiers, x, y):
        if self.retry_after is not None and self.get_clock().now() < self.retry_after:
            return
        target = fu.pick_frontier(
            frontiers, x, y, self.blacklist,
            self.get_parameter('blacklist_radius').value,
            self.get_parameter('min_goal_distance').value,
            self.get_parameter('length_weight').value)
        if target is None:
            self.no_frontier_count += 1
            patience = self.get_parameter('no_frontier_patience').value
            self.get_logger().info(
                f'No reachable frontier ({self.no_frontier_count}/{patience}).')
            if self.no_frontier_count >= patience:
                self.finish()
            return
        self.no_frontier_count = 0
        heading = math.atan2(target.y - y, target.x - x)
        self.send_goal(target.x, target.y, heading, False)

    # ------------------------------------------------------------ goal plumbing

    def send_goal(self, gx, gy, heading, returning):
        if not self.client.wait_for_server(timeout_sec=0.0):
            self.get_logger().info('Waiting for Nav2 navigate_to_pose...',
                                   throttle_duration_sec=10.0)
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map.header.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_to_quaternion(heading)

        self.goal_xy = (gx, gy)
        self.goal_handle = None
        self.goal_sent_at = self.get_clock().now()
        self.state = RETURNING if returning else NAVIGATING
        self.goals_sent += 1
        label = 'start pose' if returning else f'frontier #{self.goals_sent}'
        self.get_logger().info(f'Going to {label} at ({gx:.2f}, {gy:.2f}).')
        self.client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            # Nav2 rejects goals only while its servers are not active yet, so
            # this is a start-up condition, not a verdict on the frontier.
            self.get_logger().warn('Nav2 rejected the goal (not active yet?): retrying shortly.')
            self.state = IDLE
            self.retry_after = self.get_clock().now() + Duration(seconds=3.0)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        status = future.result().status
        was_returning = self.state == RETURNING
        self.goal_handle = None
        if was_returning:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Back at the start pose. Exploration finished.')
            else:
                self.get_logger().warn('Could not drive back to the start. Exploration finished.')
            self.state = DONE
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Frontier reached.')
            # Whatever frontier remains here cannot be cleared by standing on it.
            self.blacklist.append(self.goal_xy)
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Nav2 could not reach this frontier: blacklisted.')
            self.blacklist.append(self.goal_xy)
        # CANCELED: the supervisor cancelled and already handled the blacklist.
        self.state = IDLE

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def finish(self):
        pose = self.robot_pose()
        if self.get_parameter('return_to_start').value and self.start_xy is not None and pose:
            dist = math.hypot(self.start_xy[0] - pose[0], self.start_xy[1] - pose[1])
            if dist > self.get_parameter('min_goal_distance').value:
                self.get_logger().info(
                    f'Exploration complete after {self.goals_sent} goals. Returning to start.')
                self.send_goal(self.start_xy[0], self.start_xy[1], self.start_yaw, True)
                return
        self.get_logger().info(f'Exploration complete after {self.goals_sent} goals.')
        self.state = DONE

    # ----------------------------------------------------------------- RViz

    def publish_markers(self, frontiers):
        stamp = self.get_clock().now().to_msg()
        frame = self.map.header.frame_id
        info = self.map.info

        def base(ns, marker_id, kind):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = ns, marker_id, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        cells = base('frontier_cells', 0, Marker.POINTS)
        cells.scale.x = cells.scale.y = 0.08
        cells.color.g, cells.color.a = 1.0, 0.9
        for f in frontiers:
            # Subsample long frontiers: RViz does not need every cell.
            for row, col in f.cells[::3]:
                x, y = fu.cell_to_world(row, col, info.resolution,
                                        info.origin.position.x, info.origin.position.y)
                cells.points.append(Point(x=x, y=y, z=0.05))

        banned = base('blacklist', 0, Marker.SPHERE_LIST)
        banned.scale.x = banned.scale.y = banned.scale.z = 0.3
        banned.color.r = banned.color.g = banned.color.b = 0.5
        banned.color.a = 0.8
        banned.points = [Point(x=bx, y=by, z=0.15) for bx, by in self.blacklist]

        goal = base('goal', 0, Marker.ARROW)
        if self.goal_xy is not None and self.state in (NAVIGATING, RETURNING):
            goal.scale.x, goal.scale.y, goal.scale.z = 0.12, 0.12, 0.12
            goal.color.r, goal.color.a = 1.0, 1.0
            goal.points = [Point(x=self.goal_xy[0], y=self.goal_xy[1], z=1.2),
                           Point(x=self.goal_xy[0], y=self.goal_xy[1], z=0.1)]
        else:
            goal.action = Marker.DELETE

        self.marker_pub.publish(MarkerArray(markers=[cells, banned, goal]))


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_goal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
