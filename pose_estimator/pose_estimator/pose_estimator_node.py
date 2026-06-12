import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math
import numpy as np

class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__('pose_estimator_node')

        # Declare parameters
        self.declare_parameter('source_topic', '/seer_pose')
        self.declare_parameter('target_topic', '/estimated_pose')
        self.declare_parameter('odom_topic', '/estimated_odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('target_frequency', 100.0)
        self.declare_parameter('interpolation_interval_sec', 1.0) # How long to predict ahead
        self.declare_parameter('publish_tf', False)

        source_topic = self.get_parameter('source_topic').value
        target_topic = self.get_parameter('target_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.target_freq = self.get_parameter('target_frequency').value
        self.interp_max_dt = self.get_parameter('interpolation_interval_sec').value
        self.publish_tf_enabled = bool(self.get_parameter('publish_tf').value)

        # Subscriber
        self.sub_pose = self.create_subscription(
            PoseStamped,
            source_topic,
            self.pose_callback,
            10
        )

        # Publisher
        self.pub_pose = self.create_publisher(PoseStamped, target_topic, 10)
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 10)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf_enabled else None

        # Timer for high frequency publishing
        self.timer = self.create_timer(1.0 / self.target_freq, self.timer_callback)

        # State
        self.last_pose_msg = None
        self.last_pose_time = 0.0
        self.current_velocity = np.array([0.0, 0.0, 0.0]) # vx, vy, v_theta
        self.prev_pose_data = None # [x, y, theta]
        self.valid_pose = False

    def get_yaw_from_quat(self, q):
        # q: x, y, z, w
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_quat_from_yaw(self, yaw):
        # return x, y, z, w
        return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def pose_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Extract x, y, theta
        x = msg.pose.position.x
        y = msg.pose.position.y
        theta = self.get_yaw_from_quat(msg.pose.orientation)
        current_pose_data = np.array([x, y, theta])

        if self.prev_pose_data is not None:
            dt = current_time - self.last_pose_time
            if dt > 0.001: # Avoid division by zero
                # Calculate velocity
                deltas = current_pose_data - self.prev_pose_data

                # Handle angle wrap for theta delta
                # delta_theta should be in [-pi, pi]
                deltas[2] = math.atan2(math.sin(deltas[2]), math.cos(deltas[2]))

                self.current_velocity = deltas / dt
            else:
                self.current_velocity = np.array([0.0, 0.0, 0.0])

        self.last_pose_msg = msg
        self.prev_pose_data = current_pose_data
        self.last_pose_time = current_time
        self.valid_pose = True

    def timer_callback(self):
        if not self.valid_pose:
            return

        now = self.get_clock().now()
        stamp_msg = now.to_msg()
        current_time = now.nanoseconds / 1e9
        dt_since_last = current_time - self.last_pose_time

        # If the last pose is too old (e.g. robot stopped sending), stop predicting or stick to last known?
        # For this simple estimator, we'll continue predicting for a short while, then stop or hold.
        # But user requirement implies "interpolation/higher frequency", so dead reckoning is best.

        if dt_since_last > self.interp_max_dt:
            # Too old, maybe publish last known with updated stamp or just nothing?
            # Publishing last known static pose for safety
            out_msg = PoseStamped()
            out_msg.header.stamp = stamp_msg
            out_msg.header.frame_id = self.parent_frame
            out_msg.pose = self.last_pose_msg.pose
            self.pub_pose.publish(out_msg)
            self.publish_odom(out_msg)

            self.publish_tf(out_msg)
            return

        # Simple Linear Prediction (Dead Reckoning)
        # new_pose = old_pose + v * dt

        pred_data = self.prev_pose_data + self.current_velocity * dt_since_last

        # Create new message
        out_msg = PoseStamped()
        out_msg.header.stamp = stamp_msg
        out_msg.header.frame_id = self.parent_frame

        out_msg.pose.position.x = pred_data[0]
        out_msg.pose.position.y = pred_data[1]
        out_msg.pose.position.z = self.last_pose_msg.pose.position.z # Keep z same

        qx, qy, qz, qw = self.get_quat_from_yaw(pred_data[2])
        out_msg.pose.orientation.x = qx
        out_msg.pose.orientation.y = qy
        out_msg.pose.orientation.z = qz
        out_msg.pose.orientation.w = qw

        self.pub_pose.publish(out_msg)
        self.publish_odom(out_msg)

        self.publish_tf(out_msg)

    def publish_tf(self, pose_msg: PoseStamped):
        if not self.tf_broadcaster:
            return
        t = TransformStamped()
        t.header.stamp = pose_msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.target_frame
        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        t.transform.rotation = pose_msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_odom(self, pose_msg: PoseStamped):
        odom = Odometry()
        odom.header = pose_msg.header
        odom.child_frame_id = self.target_frame
        odom.pose.pose = pose_msg.pose
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        self.pub_odom.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
