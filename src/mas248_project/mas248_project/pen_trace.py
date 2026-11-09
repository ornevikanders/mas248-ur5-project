"""Viser i RViz det pennen faktisk har tegnet.

Noden folger TF world -> tcp. Nar TCP-en er nede pa tegneflaten, legges posisjonen til
i en strek. Nar pennen loftes, avsluttes streken. Alle strekene publiseres som en
MarkerArray pa /mas248/pen_trace (ramme world). Legg til en MarkerArray-visning med
det topicet i RViz for a se tegningen.

Kjor:  ros2 run mas248_project pen_trace
       ros2 run mas248_project pen_trace --ros-args -p flate_z:=0.31
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

TOPIC = '/mas248/pen_trace'
VERDEN = 'world'
SPISS = 'tcp'


def avstand(a, b):
    return math.dist((a.x, a.y), (b.x, b.y))


class PennSpor(Node):
    def __init__(self):
        super().__init__('mas248_pen_trace')
        # Tegneflaten i world (m): toppen av bordet, samme som --center-z til image_to_targets
        self.flate_z = self.declare_parameter('flate_z', 0.31).value
        # Pennen regnes som nede nar TCP-en er hoyst sa mye over flaten (m)
        self.toleranse = self.declare_parameter('toleranse', 0.003).value
        # Strekbredde i RViz (m), omtrent som en tusj
        self.bredde = self.declare_parameter('linjebredde', 0.002).value
        # Streken vises litt over flaten, ellers skjules den av arket (1 mm tykt)
        self.vis_z = self.flate_z + 0.002

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, TOPIC, 10)

        self.ferdige = []      # ferdige streker, hver er en liste av Point
        self.aktiv = []        # streken som tegnes akkurat na
        self.endret = True

        self.slett_alt()
        self.create_timer(1.0 / 30.0, self.les_tcp)    # folg TCP-en 30 ganger i sekundet
        self.create_timer(0.1, self.publiser)          # oppdater RViz 10 ganger i sekundet
        self.get_logger().info(
            f'Folger {VERDEN} -> {SPISS}. Penn nede nar z <= {self.flate_z + self.toleranse:.3f} m. '
            f'Vis {TOPIC} som MarkerArray i RViz.')

    def slett_alt(self):
        """Fjern gamle spor i RViz (fra en tidligere kjoring)."""
        m = Marker()
        m.action = Marker.DELETEALL
        self.pub.publish(MarkerArray(markers=[m]))

    def les_tcp(self):
        try:
            t = self.tf_buffer.lookup_transform(VERDEN, SPISS, Time())
        except TransformException:
            return                                      # TF er ikke klar enna
        pos = t.transform.translation
        if pos.z <= self.flate_z + self.toleranse:      # penn nede: legg til punkt
            punkt = Point(x=pos.x, y=pos.y, z=self.vis_z)
            if not self.aktiv or avstand(self.aktiv[-1], punkt) > 0.0005:
                self.aktiv.append(punkt)
                self.endret = True
        elif self.aktiv:                                # penn loftet: avslutt streken
            if len(self.aktiv) > 1:
                self.ferdige.append(self.aktiv)
                self.get_logger().info(f'Strek {len(self.ferdige)} ferdig ({len(self.aktiv)} punkter)')
            self.aktiv = []
            self.endret = True

    def publiser(self):
        if not self.endret:
            return
        markorer = MarkerArray()
        for i, strek in enumerate(self.ferdige + [self.aktiv]):
            if len(strek) < 2:                          # LINE_STRIP trenger minst to punkter
                continue
            m = Marker()
            m.header.frame_id = VERDEN
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'MAS248_pen_trace'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = float(self.bredde)
            m.color.r, m.color.g, m.color.b, m.color.a = 0.05, 0.05, 0.05, 1.0
            m.points = list(strek)
            markorer.markers.append(m)
        self.pub.publish(markorer)
        self.endret = False


def main():
    rclpy.init()
    node = PennSpor()
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
