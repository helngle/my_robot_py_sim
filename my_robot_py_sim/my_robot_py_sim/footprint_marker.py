from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker


class FootprintMarker(Node):
    def __init__(self):
        super().__init__('footprint_marker')
        self.declare_parameter('topic', '/base_footprint_marker')
        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('length', 0.84)
        self.declare_parameter('width', 0.84)
        self.declare_parameter('z_offset', 0.025)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            Marker,
            self.get_parameter('topic').value,
            qos,
        )
        self.timer = self.create_timer(0.2, self.publish_marker)

    def publish_marker(self):
        length = float(self.get_parameter('length').value)
        width = float(self.get_parameter('width').value)
        z_offset = float(self.get_parameter('z_offset').value)
        half_length = length / 2.0
        half_width = width / 2.0

        marker = Marker()
        marker.header.frame_id = self.get_parameter('frame_id').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mobile_manipulator_footprint'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.10
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=0, nanosec=0)
        marker.frame_locked = True

        corners = [
            (half_length, half_width, z_offset),
            (half_length, -half_width, z_offset),
            (-half_length, -half_width, z_offset),
            (-half_length, half_width, z_offset),
            (half_length, half_width, z_offset),
        ]
        marker.points = [
            Point(x=float(x), y=float(y), z=float(z))
            for x, y, z in corners
        ]
        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FootprintMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
