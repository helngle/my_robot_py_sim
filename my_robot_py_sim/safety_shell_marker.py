from builtin_interfaces.msg import Duration
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


class SafetyShellMarker(Node):
    def __init__(self):
        super().__init__('safety_shell_marker')
        self.declare_parameter('topic', '/safety_shell_array')
        self.declare_parameter('padding', 0.06)
        self.declare_parameter('alpha', 0.22)

        self.padding = float(self.get_parameter('padding').value)
        self.alpha = float(self.get_parameter('alpha').value)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray,
            self.get_parameter('topic').value,
            qos,
        )
        self.timer = self.create_timer(0.2, self.publish_markers)

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
        marker.color.r = 0.10
        marker.color.g = 0.75
        marker.color.b = 0.95
        marker.color.a = self.alpha
        marker.lifetime = Duration(sec=0, nanosec=0)
        marker.frame_locked = True
        return marker

    def box_marker(self, marker_id, frame_id, size):
        scale = tuple(value + 2.0 * self.padding for value in size)
        return self.make_marker(marker_id, frame_id, Marker.CUBE, (0, 0, 0, 0, 0, 0, 1), scale)

    def cylinder_marker(self, marker_id, frame_id, radius, length, pose):
        scale = (
            2.0 * (radius + self.padding),
            2.0 * (radius + self.padding),
            length + 2.0 * self.padding,
        )
        return self.make_marker(marker_id, frame_id, Marker.CYLINDER, pose, scale)

    def publish_markers(self):
        wheel_pose = (0, 0, 0, 0.70710678, 0, 0, 0.70710678)
        arm_pose_positive_y = (0, 0.21, 0, 0.70710678, 0, 0, 0.70710678)
        forearm_pose_positive_y = (0, 0.18, 0, 0.70710678, 0, 0, 0.70710678)
        arm_pose_negative_y = (0, -0.21, 0, 0.70710678, 0, 0, 0.70710678)
        forearm_pose_negative_y = (0, -0.18, 0, 0.70710678, 0, 0, 0.70710678)

        markers = MarkerArray()
        markers.markers.extend([
            self.box_marker(0, 'base_link', (0.72, 0.56, 0.24)),
            self.cylinder_marker(1, 'front_left_wheel_link', 0.105, 0.11, wheel_pose),
            self.cylinder_marker(2, 'front_right_wheel_link', 0.105, 0.11, wheel_pose),
            self.cylinder_marker(3, 'rear_left_wheel_link', 0.105, 0.11, wheel_pose),
            self.cylinder_marker(4, 'rear_right_wheel_link', 0.105, 0.11, wheel_pose),
            self.box_marker(5, 'torso_link', (0.44, 0.36, 0.68)),
            self.box_marker(6, 'head_sensor_link', (0.26, 0.24, 0.18)),
            self.cylinder_marker(7, 'left_upper_arm_link', 0.045, 0.42, arm_pose_positive_y),
            self.cylinder_marker(8, 'left_forearm_link', 0.04, 0.36, forearm_pose_positive_y),
            self.box_marker(9, 'left_hand_link', (0.10, 0.07, 0.12)),
            self.cylinder_marker(10, 'right_upper_arm_link', 0.045, 0.42, arm_pose_negative_y),
            self.cylinder_marker(11, 'right_forearm_link', 0.04, 0.36, forearm_pose_negative_y),
            self.box_marker(12, 'right_hand_link', (0.10, 0.07, 0.12)),
        ])
        self.publisher.publish(markers)


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
