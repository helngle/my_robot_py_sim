import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def stamp_nanoseconds(msg):
    stamp = msg.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def pose_key(msg):
    position = msg.pose.position
    orientation = msg.pose.orientation
    return (
        stamp_nanoseconds(msg),
        msg.header.frame_id,
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )


def odom_key(msg):
    pose = msg.pose.pose
    twist = msg.twist.twist
    return (
        stamp_nanoseconds(msg),
        msg.header.frame_id,
        msg.child_frame_id,
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )


class LatestSample:
    def __init__(self, key_function, drop_regressed):
        self._key_function = key_function
        self._drop_regressed = drop_regressed
        self._lock = threading.Lock()
        self._latest = None
        self._latest_key = None
        self._latest_stamp_ns = None
        self._generation = 0
        self._published_generation = 0
        self.received = 0
        self.published = 0
        self.duplicates = 0
        self.regressed = 0
        self.overwritten = 0

    def update(self, msg):
        key = self._key_function(msg)
        stamp_ns = key[0]

        with self._lock:
            self.received += 1
            if (
                self._latest_stamp_ns is not None
                and stamp_ns != 0
                and stamp_ns < self._latest_stamp_ns
            ):
                self.regressed += 1
                if self._drop_regressed:
                    return

            if key == self._latest_key:
                self.duplicates += 1
                return

            if self._generation != self._published_generation:
                self.overwritten += 1

            self._latest = msg
            self._latest_key = key
            self._latest_stamp_ns = stamp_ns
            self._generation += 1

    def take_new(self):
        with self._lock:
            if (
                self._latest is None
                or self._generation == self._published_generation
            ):
                return None
            self._published_generation = self._generation
            self.published += 1
            return self._latest

    def snapshot_counts(self):
        with self._lock:
            return (
                self.received,
                self.published,
                self.duplicates,
                self.regressed,
                self.overwritten,
            )


class LatestSdkSampleRelay(Node):
    def __init__(self):
        super().__init__('latest_sdk_sample_relay')
        self.declare_parameter('input_pose_topic', '/vmr_base_bridge/pose')
        self.declare_parameter(
            'output_pose_topic', '/vmr_base_bridge/latest_pose'
        )
        self.declare_parameter('relay_odom', False)
        self.declare_parameter('input_odom_topic', '/vmr_base_bridge/odom')
        self.declare_parameter(
            'output_odom_topic', '/vmr_base_bridge/latest_odom'
        )
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('stats_period', 5.0)
        self.declare_parameter('drop_regressed', True)

        input_pose_topic = self.get_parameter('input_pose_topic').value
        output_pose_topic = self.get_parameter('output_pose_topic').value
        input_odom_topic = self.get_parameter('input_odom_topic').value
        output_odom_topic = self.get_parameter('output_odom_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        stats_period = float(self.get_parameter('stats_period').value)
        drop_regressed = bool(self.get_parameter('drop_regressed').value)
        self.relay_odom = bool(self.get_parameter('relay_odom').value)

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        if stats_period <= 0.0:
            raise ValueError('stats_period must be greater than zero')
        if input_pose_topic == output_pose_topic:
            raise ValueError(
                'input_pose_topic and output_pose_topic must differ'
            )
        if self.relay_odom and input_odom_topic == output_odom_topic:
            raise ValueError(
                'input_odom_topic and output_odom_topic must differ'
            )

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pose_sample = LatestSample(pose_key, drop_regressed)
        self.pose_publisher = self.create_publisher(
            PoseStamped, output_pose_topic, output_qos
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped, input_pose_topic, self.pose_sample.update, input_qos
        )

        self.odom_sample = None
        self.odom_publisher = None
        self.odom_subscription = None
        if self.relay_odom:
            self.odom_sample = LatestSample(odom_key, drop_regressed)
            self.odom_publisher = self.create_publisher(
                Odometry, output_odom_topic, output_qos
            )
            self.odom_subscription = self.create_subscription(
                Odometry, input_odom_topic, self.odom_sample.update, input_qos
            )

        self.publish_timer = self.create_timer(
            1.0 / publish_rate, self.publish_latest
        )
        self.stats_timer = self.create_timer(stats_period, self.log_statistics)

        self.get_logger().info(
            'Experimental latest-sample relay: '
            f'{input_pose_topic} -> {output_pose_topic} at '
            f'{publish_rate:.1f} Hz; '
            f'odom={self.relay_odom}'
        )

    def publish_latest(self):
        pose = self.pose_sample.take_new()
        if pose is not None:
            self.pose_publisher.publish(pose)

        if self.odom_sample is not None:
            odom = self.odom_sample.take_new()
            if odom is not None:
                self.odom_publisher.publish(odom)

    def log_statistics(self):
        self._log_stream_statistics('pose', self.pose_sample)
        if self.odom_sample is not None:
            self._log_stream_statistics('odom', self.odom_sample)

    def _log_stream_statistics(self, name, sample):
        received, published, duplicates, regressed, overwritten = (
            sample.snapshot_counts()
        )
        self.get_logger().info(
            f'{name} totals: received={received} published={published} '
            f'duplicates={duplicates} regressed={regressed} '
            f'overwritten_before_publish={overwritten}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LatestSdkSampleRelay()
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
