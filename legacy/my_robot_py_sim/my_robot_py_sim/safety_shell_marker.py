from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
import yaml


class SafetyShellMarker(Node):
    def __init__(self):
        super().__init__('safety_shell_marker')
        self.declare_parameter('topic', '/safety_shell_array')
        self.declare_parameter('config_file', '')
        self.declare_parameter('padding', 0.06)
        self.declare_parameter('alpha', 0.22)

        self.config = self.load_config()
        shell_config = self.config.get('safety_shell', {})
        self.padding = float(shell_config.get('padding', self.get_parameter('padding').value))
        self.alpha = float(shell_config.get('alpha', self.get_parameter('alpha').value))
        self.color = tuple(shell_config.get('color', [0.10, 0.75, 0.95]))
        self.shells = shell_config.get('shells', [])
        topic = shell_config.get('marker_topic', self.get_parameter('topic').value)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray,
            topic,
            qos,
        )
        self.timer = self.create_timer(0.2, self.publish_markers)

    def load_config(self):
        config_file = self.get_parameter('config_file').value
        if not config_file:
            config_file = (
                get_package_share_directory('my_robot_py_sim')
                + '/config/safety_shell.yaml'
            )

        try:
            with open(config_file, 'r') as config_stream:
                return yaml.safe_load(config_stream) or {}
        except OSError as exc:
            self.get_logger().error(f'Failed to read safety shell config {config_file}: {exc}')
            return {'safety_shell': {'shells': []}}

    def make_marker(self, marker_id, frame_id, marker_type, pose, scale):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mobile_manipulator_safety'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.position.z = float(pose[2])
        marker.pose.orientation.x = float(pose[3])
        marker.pose.orientation.y = float(pose[4])
        marker.pose.orientation.z = float(pose[5])
        marker.pose.orientation.w = float(pose[6])
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.r = float(self.color[0])
        marker.color.g = float(self.color[1])
        marker.color.b = float(self.color[2])
        marker.color.a = self.alpha
        marker.lifetime = Duration(sec=0, nanosec=0)
        marker.frame_locked = True
        return marker

    def box_marker(self, marker_id, frame_id, size, pose):
        scale = tuple(value + 2.0 * self.padding for value in size)
        return self.make_marker(marker_id, frame_id, Marker.CUBE, pose, scale)

    def cylinder_marker(self, marker_id, frame_id, radius, length, pose):
        scale = (
            2.0 * (radius + self.padding),
            2.0 * (radius + self.padding),
            length + 2.0 * self.padding,
        )
        return self.make_marker(marker_id, frame_id, Marker.CYLINDER, pose, scale)

    def publish_markers(self):
        markers = MarkerArray()
        for marker_id, shell in enumerate(self.shells):
            marker = self.marker_from_shell(marker_id, shell)
            if marker is not None:
                markers.markers.append(marker)
        self.publisher.publish(markers)

    def marker_from_shell(self, marker_id, shell):
        shape = shell.get('shape')
        frame_id = shell.get('frame_id')
        pose = shell.get('pose', [0, 0, 0, 0, 0, 0, 1])

        if shape == 'box':
            return self.box_marker(marker_id, frame_id, shell['size'], pose)
        if shape == 'cylinder':
            return self.cylinder_marker(
                marker_id,
                frame_id,
                float(shell['radius']),
                float(shell['length']),
                pose,
            )

        self.get_logger().warn(f'Ignoring unknown safety shell shape: {shape}')
        return None


def main(args=None):
    rclpy.init(args=args)
    node = SafetyShellMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
