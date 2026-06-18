import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboPoseOdom(Node):
    def __init__(self):
        super().__init__('gazebo_pose_odom')
        self.declare_parameter('pose_topic', '/gazebo_pose_info')
        self.declare_parameter('model_name', 'mobile_manipulator')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        self.model_name = self.get_parameter('model_name').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.previous_stamp = None
        self.previous_x = None
        self.previous_y = None
        self.previous_yaw = None

        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').value,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            TFMessage,
            self.get_parameter('pose_topic').value,
            self.handle_pose_info,
            10,
        )

    def handle_pose_info(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == self.model_name:
                self.publish_odom(transform)
                return

    def publish_odom(self, transform):
        stamp = self.get_clock().now().to_msg()
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        yaw = yaw_from_quaternion(transform.transform.rotation)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        if self.previous_stamp is not None:
            current_time = stamp.sec + stamp.nanosec * 1e-9
            previous_time = self.previous_stamp.sec + self.previous_stamp.nanosec * 1e-9
            dt = current_time - previous_time
            if dt > 1e-6:
                dx = x - self.previous_x
                dy = y - self.previous_y
                heading_dx = math.cos(yaw) * dx + math.sin(yaw) * dy
                odom.twist.twist.linear.x = heading_dx / dt
                odom.twist.twist.angular.z = normalize_angle(yaw - self.previous_yaw) / dt

        self.previous_stamp = stamp
        self.previous_x = x
        self.previous_y = y
        self.previous_yaw = yaw

        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseOdom()
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
