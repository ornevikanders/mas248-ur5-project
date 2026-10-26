"""Show the drawing targets from /tmp/mas248_targets.json in RViz2.

Replacement for the missing show_targets_rviz.py in the Lecture 9 starter.
Publishes a MarkerArray on /mas248/drawing_targets in the 'world' frame.
"""
import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def tcp_z_axis(qx, qy, qz, qw):
    """Direction of the TCP +Z axis in world, from the pose quaternion."""
    return (2.0 * (qx * qz + qw * qy),
            2.0 * (qy * qz - qw * qx),
            1.0 - 2.0 * (qx * qx + qy * qy))


def pose_to_list(p):
    """Return [x, y, z, qx, qy, qz, qw] whether the pose is a list or a dict."""
    if isinstance(p, (list, tuple)):
        return list(p)
    if 'position' in p:  # {"position": {...}, "orientation": {...}}
        pos, ori = p['position'], p['orientation']
        if isinstance(pos, (list, tuple)):
            return list(pos) + list(ori)
        return [pos['x'], pos['y'], pos['z'], ori['x'], ori['y'], ori['z'], ori['w']]
    return [p['x'], p['y'], p['z'], p['qx'], p['qy'], p['qz'], p['qw']]


class TargetMarkers(Node):
    def __init__(self, json_path, axis_every, axis_len):
        super().__init__('show_targets_rviz')
        with open(json_path) as f:
            self.data = json.load(f)
        self.axis_every = max(1, axis_every)
        self.axis_len = axis_len
        self.pub = self.create_publisher(MarkerArray, '/mas248/drawing_targets', 10)
        self.timer = self.create_timer(1.0, self.publish)
        n = len(self.data['strokes'])
        self.get_logger().info(
            f'Showing {n} strokes from {json_path} on /mas248/drawing_targets; Ctrl+C to stop.')

    def base_marker(self, ns, mid, mtype):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish(self):
        markers = MarkerArray()
        mid = 0
        for s_idx, stroke in enumerate(self.data['strokes']):
            poses = [pose_to_list(p) for p in stroke['poses']]  # world, metres
            if not poses:
                continue
            pts = [Point(x=p[0], y=p[1], z=p[2]) for p in poses]

            line = self.base_marker('MAS248_contour_paths', mid, Marker.LINE_STRIP)
            line.scale.x = 0.002
            line.color.r, line.color.g, line.color.b, line.color.a = 0.1, 0.9, 0.2, 1.0
            line.points = pts
            markers.markers.append(line)
            mid += 1

            dots = self.base_marker('MAS248_TCP_waypoints', mid, Marker.SPHERE_LIST)
            dots.scale.x = dots.scale.y = dots.scale.z = 0.004
            dots.color.r, dots.color.g, dots.color.b, dots.color.a = 1.0, 0.5, 0.0, 1.0
            dots.points = pts
            markers.markers.append(dots)
            mid += 1

            start = self.base_marker('MAS248_stroke_start', mid, Marker.SPHERE)
            start.pose.position = pts[0]
            start.scale.x = start.scale.y = start.scale.z = 0.010
            start.color.r, start.color.g, start.color.b, start.color.a = 1.0, 0.0, 0.0, 1.0
            markers.markers.append(start)
            mid += 1

            for i in range(0, len(poses), self.axis_every):
                x, y, z, qx, qy, qz, qw = poses[i]
                dx, dy, dz = tcp_z_axis(qx, qy, qz, qw)
                arrow = self.base_marker('MAS248_TCP_orientation_Z', mid, Marker.ARROW)
                arrow.points = [Point(x=x, y=y, z=z),
                                Point(x=x + self.axis_len * dx,
                                      y=y + self.axis_len * dy,
                                      z=z + self.axis_len * dz)]
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.0015, 0.003, 0.004
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.1, 0.3, 1.0, 1.0
                markers.markers.append(arrow)
                mid += 1
        self.pub.publish(markers)


def main(args=None):
    parser = argparse.ArgumentParser(description='Show MAS248 drawing targets in RViz2')
    parser.add_argument('--json', default='/tmp/mas248_targets.json')
    parser.add_argument('--axis-every', type=int, default=4)
    parser.add_argument('--axis-len', type=float, default=0.03)
    known, _ = parser.parse_known_args(sys.argv[1:])  # ignore unknown/ROS args

    rclpy.init(args=args)
    node = TargetMarkers(known.json, known.axis_every, known.axis_len)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
    