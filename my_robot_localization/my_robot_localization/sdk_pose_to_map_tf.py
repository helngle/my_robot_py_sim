import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class SdkPoseToMapTf(Node):
    def __init__(self):
        super().__init__('sdk_pose_to_map_tf')
        self.declare_parameter('pose_topic', '/vmr_base_bridge/pose')
        self.declare_parameter('estimated_pose_topic', '/estimated_pose')
        self.declare_parameter('estimated_odom_topic', '/estimated_odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('max_pose_age', 1.0)
        self.declare_parameter('stamp_with_current_time', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_pose_age = float(self.get_parameter('max_pose_age').value)
        self.stamp_with_current_time = bool(
            self.get_parameter('stamp_with_current_time').value
        )

        self.latest_pose = None
        self.latest_pose_time = None
        self.last_pose_xy_yaw = None
        self.last_pose_time_sec = None
        self.velocity = (0.0, 0.0, 0.0)

        self.broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(
            PoseStamped,
            self.get_parameter('estimated_pose_topic').value,
            10,
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('estimated_odom_topic').value,
            10,
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            self.handle_pose,
            10,
        )

        publish_rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish)

    def handle_pose(self, msg):
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        yaw = yaw_from_quaternion(msg.pose.orientation)
        pose_xy_yaw = (msg.pose.position.x, msg.pose.position.y, yaw)

        if self.last_pose_xy_yaw is not None:
            dt = now_sec - self.last_pose_time_sec
            if dt > 1e-3:
                dx = pose_xy_yaw[0] - self.last_pose_xy_yaw[0]
                dy = pose_xy_yaw[1] - self.last_pose_xy_yaw[1]
                dyaw = math.atan2(
                    math.sin(pose_xy_yaw[2] - self.last_pose_xy_yaw[2]),
                    math.cos(pose_xy_yaw[2] - self.last_pose_xy_yaw[2]),
                )
                self.velocity = (dx / dt, dy / dt, dyaw / dt)

        self.latest_pose = msg
        self.latest_pose_time = now
        self.last_pose_xy_yaw = pose_xy_yaw
        self.last_pose_time_sec = now_sec

    def publish(self):
        if self.latest_pose is None:
            return

        now = self.get_clock().now()
        pose_age = (now - self.latest_pose_time).nanoseconds / 1e9
        if pose_age > self.max_pose_age:
            return

        yaw = yaw_from_quaternion(self.latest_pose.pose.orientation)
        pose = PoseStamped()
        pose.header.stamp = (
            now.to_msg()
            if self.stamp_with_current_time
            else self.latest_pose.header.stamp
        )
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = self.latest_pose.pose.position.x
        pose.pose.position.y = self.latest_pose.pose.position.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = quaternion_from_yaw(yaw)

        self.pose_pub.publish(pose)
        self.publish_odom(pose)
        self.publish_tf(pose)

    def publish_odom(self, pose):
        odom = Odometry()
        odom.header = pose.header
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        odom.twist.twist.angular.z = self.velocity[2]
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        self.odom_pub.publish(odom)

    def publish_tf(self, pose):
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = SdkPoseToMapTf()
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
