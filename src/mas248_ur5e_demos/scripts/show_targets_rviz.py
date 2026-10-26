"""Publish /tmp/mas248_targets.json as RViz markers on /mas248/drawing_targets until Ctrl+C.

Also broadcasts the static transform world -> image_center from the JSON.
Namespaces: MAS248_image_bounds, MAS248_contour_paths, MAS248_TCP_waypoints,
MAS248_TCP_orientation_X/Y/Z, MAS248_stroke_start, MAS248_image_frame_axes.
"""
import argparse
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

TOPIC = '/mas248/drawing_targets'


def rotate_vector_xyzw(q, v):
    """Rotate vector v by quaternion q=(x,y,z,w); q is normalised first."""
    x, y, z, w = (float(c) for c in q)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12: raise ValueError('Zero quaternion')
    x, y, z, w = x/n, y/n, z/n, w/n
    vx, vy, vz = (float(c) for c in v)
    # v' = v + 2w(q x v) + 2 q x (q x v)
    cx, cy, cz = y*vz - z*vy, z*vx - x*vz, x*vy - y*vx
    ccx, ccy, ccz = y*cz - z*cy, z*cx - x*cz, x*cy - y*cx
    return (vx + 2*(w*cx + ccx), vy + 2*(w*cy + ccy), vz + 2*(w*cz + ccz))


def target_axes(q, length):
    """TCP X, Y, Z axes in world for orientation q, each scaled to length."""
    return tuple(rotate_vector_xyzw(q, a) for a in ((length, 0, 0), (0, length, 0), (0, 0, length)))


def _color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _pt(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


def build_markers(data, stamp, axis_every=10, axis_length_mm=18., axis_lift_mm=0., orientations=True):
    arr = MarkerArray(); mid = [0]

    def marker(ns, mtype, frame='world', scale=(0.002, 0.002, 0.002), color=_color(0, 0, 0)):
        m = Marker(); m.header.frame_id = frame; m.header.stamp = stamp
        m.ns = ns; m.id = mid[0]; mid[0] += 1
        m.type = mtype; m.action = Marker.ADD; m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = (float(s) for s in scale); m.color = color
        arr.markers.append(m); return m

    clear = Marker(); clear.action = Marker.DELETEALL; arr.markers.append(clear)

    # Image rectangle and frame axes in image_center frame.
    hw, hh = data['image']['width_m']/2., data['image']['height_m']/2.
    b = marker('MAS248_image_bounds', Marker.LINE_STRIP, 'image_center', (0.002, 0, 0), _color(0.1, 0.1, 0.1))
    b.points = [_pt(-hw, -hh, 0), _pt(hw, -hh, 0), _pt(hw, hh, 0), _pt(-hw, hh, 0), _pt(-hw, -hh, 0)]
    for axis, col in (((1, 0, 0), _color(1, 0, 0)), ((0, 1, 0), _color(0, 0.8, 0))):
        a = marker('MAS248_image_frame_axes', Marker.ARROW, 'image_center', (0.003, 0.006, 0.008), col)
        a.points = [_pt(0, 0, 0), _pt(0.04*axis[0], 0.04*axis[1], 0)]

    palette = [(0.9, 0.4, 0.15), (0.1, 0.3, 0.7), (0.2, 0.6, 0.2), (0.6, 0.15, 0.6), (0.75, 0.6, 0.3)]
    axis_cols = (_color(1, 0, 0), _color(0, 0.8, 0), _color(0, 0, 1))
    length, lift = axis_length_mm/1000., axis_lift_mm/1000.
    for i, stroke in enumerate(data['strokes']):
        poses = stroke['poses']; col = _color(*palette[i % len(palette)])
        path = marker('MAS248_contour_paths', Marker.LINE_STRIP, scale=(0.0015, 0, 0), color=col)
        path.points = [_pt(p['x'], p['y'], p['z']) for p in poses]
        dots = marker('MAS248_TCP_waypoints', Marker.SPHERE_LIST, scale=(0.003, 0.003, 0.003), color=col)
        dots.points = list(path.points)
        s = marker('MAS248_stroke_start', Marker.SPHERE, scale=(0.008, 0.008, 0.008), color=_color(0, 0.75, 0))
        s.pose.position = _pt(poses[0]['x'], poses[0]['y'], poses[0]['z'])
        if not orientations: continue
        for p in poses[::max(1, axis_every)]:
            q = (p['qx'], p['qy'], p['qz'], p['qw'])
            o = _pt(p['x'], p['y'], p['z'] + lift)
            for name, vec, c in zip('XYZ', target_axes(q, length), axis_cols):
                a = marker('MAS248_TCP_orientation_' + name, Marker.ARROW, scale=(0.0015, 0.003, 0.004), color=c)
                a.points = [o, _pt(o.x + vec[0], o.y + vec[1], o.z + vec[2])]
    return arr


class TargetPublisher(Node):
    def __init__(self, args):
        super().__init__('mas248_show_targets_rviz')
        with open(args.json, encoding='utf-8') as f:
            self.data = json.load(f)
        self.args = args
        self.pub = self.create_publisher(MarkerArray, TOPIC, 10)
        ft = self.data['frame_to_world']
        t = TransformStamped(); t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = ft['parent']; t.child_frame_id = ft['child']
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in ft['xyz'])
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = (float(v) for v in ft['quaternion_xyzw'])
        self.tf = StaticTransformBroadcaster(self); self.tf.sendTransform(t)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'{len(self.data["strokes"])} strokes on {TOPIC} (MarkerArray, Fixed Frame world)')

    def publish(self):
        a = self.args
        self.pub.publish(build_markers(self.data, self.get_clock().now().to_msg(), a.axis_every,
                                       a.axis_length_mm, a.axis_lift_mm, not a.no_orientations))


def main():
    p = argparse.ArgumentParser(description='Show MAS248 image targets in RViz (Ctrl+C to stop)')
    p.add_argument('--json', default='/tmp/mas248_targets.json')
    p.add_argument('--axis-every', type=int, default=10)
    p.add_argument('--axis-length-mm', type=float, default=18.)
    p.add_argument('--axis-lift-mm', type=float, default=0.)
    p.add_argument('--no-orientations', action='store_true')
    args = p.parse_args()
    rclpy.init(); node = TargetPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
