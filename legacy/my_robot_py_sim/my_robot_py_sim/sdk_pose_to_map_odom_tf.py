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


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SdkPoseToMapOdomTf(Node):
    def __init__(self):
        super().__init__('sdk_pose_to_map_odom_tf')
        self.declare_parameter('pose_topic', '/vmr_base_bridge/pose')
        self.declare_parameter('odom_topic', '/vmr_base_bridge/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('max_pose_age', 1.0)
        self.declare_parameter('max_odom_age', 1.0)
        self.declare_parameter('stamp_with_current_time', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_pose_age = float(self.get_parameter('max_pose_age').value)
        self.max_odom_age = float(self.get_parameter('max_odom_age').value)
        self.stamp_with_current_time = bool(
            self.get_parameter('stamp_with_current_time').value
        )

        self.latest_pose = None
        self.latest_pose_time = None
        self.latest_odom = None
        self.latest_odom_time = None

        self.broadcaster = TransformBroadcaster(self)
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            self.handle_pose,
            10,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.handle_odom,
            10,
        )
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_tf)

    def handle_pose(self, msg):
        self.latest_pose = msg
        self.latest_pose_time = self.get_clock().now()

    def handle_odom(self, msg):
        self.latest_odom = msg
        self.latest_odom_time = self.get_clock().now()

    def publish_tf(self):
        if self.latest_pose is None or self.latest_odom is None:
            return

        now = self.get_clock().now()
        pose_age = (now - self.latest_pose_time).nanoseconds / 1e9
        odom_age = (now - self.latest_odom_time).nanoseconds / 1e9
        if pose_age > self.max_pose_age or odom_age > self.max_odom_age:
            return

        map_base = self.latest_pose.pose
        odom_base = self.latest_odom.pose.pose

        map_x = map_base.position.x
        map_y = map_base.position.y
        map_yaw = yaw_from_quaternion(map_base.orientation)

        odom_x = odom_base.position.x
        odom_y = odom_base.position.y
        odom_yaw = yaw_from_quaternion(odom_base.orientation)

        map_odom_yaw = normalize_angle(map_yaw - odom_yaw)
        cos_yaw = math.cos(map_odom_yaw)
        sin_yaw = math.sin(map_odom_yaw)

        map_odom_x = map_x - (cos_yaw * odom_x - sin_yaw * odom_y)
        map_odom_y = map_y - (sin_yaw * odom_x + cos_yaw * odom_y)

        transform = TransformStamped()
        if self.stamp_with_current_time:
            transform.header.stamp = now.to_msg()
        else:
            transform.header.stamp = self.latest_pose.header.stamp
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = map_odom_x
        transform.transform.translation.y = map_odom_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion_from_yaw(map_odom_yaw)
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = SdkPoseToMapOdomTf()
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
