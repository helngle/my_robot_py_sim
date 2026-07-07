import math
from copy import deepcopy

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def set_pose_yaw(pose, yaw):
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(yaw * 0.5)
    pose.orientation.w = math.cos(yaw * 0.5)


class HybridViewpointOrchestrator(Node):
    """Route table goals to long, short, or low-speed refinement control."""

    def __init__(self):
        super().__init__('hybrid_viewpoint_orchestrator')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_frame', 'map'),
                ('base_frame', 'base_footprint'),
                ('viewpoint_goal_topic', '/table_viewpoint_goal'),
                ('goal_topic_transient_local', True),
                ('bbox_topic', '/target_bbox_3d'),
                ('navigate_action', '/navigate_to_pose'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('sam3_detection_service', '/detect_sam3_table'),
                ('distance_split_m', 1.5),
                ('long_bt_xml', ''),
                ('short_bt_xml', ''),
                ('fine_bt_xml', ''),
                ('spin_action', '/spin'),
                ('enable_refinement', True),
                ('enable_final_yaw', True),
                ('refinement_mode', 'detection'),
                ('ignore_short_goal_yaw', False),
                ('auto_rearm', False),
                ('settle_time_s', 1.0),
                ('detection_timeout_s', 8.0),
                ('max_refinement_attempts', 3),
                ('max_refinement_step_m', 0.20),
                ('max_refinement_yaw_rad', 0.20),
                ('fine_position_tolerance_m', 0.025),
                ('fine_yaw_tolerance_rad', 0.035),
                ('final_yaw_tolerance_rad', 0.10),
                ('final_yaw_position_tolerance_m', 0.15),
                ('final_yaw_time_allowance_s', 10.0),
            ],
        )

        self.map_frame = self.parameter('map_frame')
        self.base_frame = self.parameter('base_frame')
        self.distance_split = float(self.parameter('distance_split_m'))
        self.long_bt_xml = self.parameter('long_bt_xml')
        self.short_bt_xml = self.parameter('short_bt_xml')
        self.fine_bt_xml = self.parameter('fine_bt_xml')
        self.enable_refinement = bool(self.parameter('enable_refinement'))
        self.enable_final_yaw = bool(self.parameter('enable_final_yaw'))
        self.refinement_mode = str(self.parameter('refinement_mode')).lower()
        self.ignore_short_goal_yaw = bool(
            self.parameter('ignore_short_goal_yaw')
        )
        self.auto_rearm = bool(self.parameter('auto_rearm'))
        self.settle_time = float(self.parameter('settle_time_s'))
        self.detection_timeout = float(
            self.parameter('detection_timeout_s')
        )
        self.max_refinement_attempts = int(
            self.parameter('max_refinement_attempts')
        )
        self.max_refinement_step = float(
            self.parameter('max_refinement_step_m')
        )
        self.max_refinement_yaw = float(
            self.parameter('max_refinement_yaw_rad')
        )
        self.fine_position_tolerance = float(
            self.parameter('fine_position_tolerance_m')
        )
        self.fine_yaw_tolerance = float(
            self.parameter('fine_yaw_tolerance_rad')
        )
        self.final_yaw_tolerance = float(
            self.parameter('final_yaw_tolerance_rad')
        )
        self.final_yaw_position_tolerance = float(
            self.parameter('final_yaw_position_tolerance_m')
        )
        self.final_yaw_time_allowance = float(
            self.parameter('final_yaw_time_allowance_s')
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.parameter('navigate_action'),
        )
        self.spin_client = ActionClient(
            self,
            Spin,
            self.parameter('spin_action'),
        )
        self.detect_client = self.create_client(
            Trigger,
            self.parameter('sam3_detection_service'),
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.parameter('cmd_vel_topic'),
            10,
        )

        goal_qos = QoSProfile(depth=1)
        goal_qos.reliability = ReliabilityPolicy.RELIABLE
        goal_qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
            if bool(self.parameter('goal_topic_transient_local'))
            else DurabilityPolicy.VOLATILE
        )
        self.create_subscription(
            PoseStamped,
            self.parameter('viewpoint_goal_topic'),
            self.goal_callback,
            goal_qos,
        )
        self.create_subscription(
            Detection3D,
            self.parameter('bbox_topic'),
            self.bbox_callback,
            10,
        )
        self.create_service(
            Trigger,
            'reset_hybrid_table_viewpoint',
            self.reset_callback,
        )
        self.create_timer(0.10, self.tick)

        self.state = 'WAIT_INITIAL_GOAL'
        self.mode = None
        self.goal_handle = None
        self.spin_goal_handle = None
        self.nav_goal_token = 0
        self.spin_goal_token = 0
        self.bbox_version = 0
        self.refinement_bbox_version = None
        self.refinement_bbox_received = False
        self.pending_refinement_goal = None
        self.refinement_target_goal = None
        self.refinement_ignores_yaw = False
        self.fine_attempts = 0
        self.settle_deadline = None
        self.detection_deadline = None
        self.get_logger().info(
            f'Hybrid viewpoint ready: long distance >= '
            f'{self.distance_split:.2f} m; shorter goals use ShortPath.'
        )

    def parameter(self, name):
        return self.get_parameter(name).value

    def current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.15),
            )
        except TransformException as exc:
            self.get_logger().warn(f'Cannot read robot pose: {exc}')
            return None
        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return float(translation.x), float(translation.y), yaw

    def goal_callback(self, msg):
        if self.state == 'WAIT_INITIAL_GOAL':
            self.start_goal(msg, refinement=False)
            return
        if self.state in ('REQUESTING_DETECTION', 'WAIT_REFINED_GOAL'):
            if not self.refinement_bbox_received:
                return
            if self.state == 'REQUESTING_DETECTION':
                self.pending_refinement_goal = deepcopy(msg)
            else:
                self.start_goal(msg, refinement=True)

    def bbox_callback(self, _msg):
        self.bbox_version += 1
        if (
            self.state in ('REQUESTING_DETECTION', 'WAIT_REFINED_GOAL') and
            self.refinement_bbox_version is not None and
            self.bbox_version > self.refinement_bbox_version
        ):
            self.refinement_bbox_received = True

    def start_goal(self, msg, refinement):
        robot = self.current_pose()
        if robot is None:
            return
        dx = msg.pose.position.x - robot[0]
        dy = msg.pose.position.y - robot[1]
        distance = math.hypot(dx, dy)
        goal_yaw = yaw_from_quaternion(msg.pose.orientation)
        yaw_error = normalize_angle(goal_yaw - robot[2])

        original_goal = deepcopy(msg)
        original_goal.header.frame_id = self.map_frame
        original_goal.header.stamp = self.get_clock().now().to_msg()
        goal = deepcopy(original_goal)
        initial_short_goal = (
            not refinement and distance < self.distance_split
        )
        ignore_goal_yaw = (
            (initial_short_goal and self.ignore_short_goal_yaw) or
            (refinement and self.refinement_ignores_yaw)
        )
        if ignore_goal_yaw:
            set_pose_yaw(goal.pose, robot[2])
            goal_yaw = robot[2]
            yaw_error = 0.0
        if not refinement:
            self.refinement_target_goal = deepcopy(original_goal)
            self.refinement_ignores_yaw = False
            self.fine_attempts = 0
        if refinement:
            if (
                distance <= self.fine_position_tolerance and
                abs(yaw_error) <= self.fine_yaw_tolerance
            ):
                self.complete(
                    'Refinement check passed without another movement: '
                    f'position error={distance:.3f} m, '
                    f'yaw error={abs(yaw_error):.3f} rad.'
                )
                return
            if self.fine_attempts >= self.max_refinement_attempts:
                self.complete(
                    'Refinement limit reached with remaining error: '
                    f'position={distance:.3f} m, '
                    f'yaw={abs(yaw_error):.3f} rad.'
                )
                return
            goal = self.clamp_refinement_goal(goal, robot, dx, dy, yaw_error)
            mode = 'fine'
            behavior_tree = self.fine_bt_xml
            self.fine_attempts += 1
        elif distance >= self.distance_split:
            mode = 'long'
            behavior_tree = self.long_bt_xml
        else:
            mode = 'short'
            behavior_tree = self.short_bt_xml

        if not behavior_tree:
            self.get_logger().error(
                f'{mode} behavior-tree path is empty; refusing the goal.'
            )
            self.fail_task('Missing behavior-tree path.')
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('/navigate_to_pose is not ready yet.')
            return

        action_goal = NavigateToPose.Goal()
        action_goal.pose = goal
        action_goal.behavior_tree = behavior_tree
        self.mode = mode
        self.state = 'GOAL_PENDING'
        self.nav_goal_token += 1
        token = self.nav_goal_token
        future = self.nav_client.send_goal_async(action_goal)
        future.add_done_callback(
            lambda future, token=token: self.goal_response_callback(
                future,
                token,
            )
        )
        self.get_logger().info(
            f'Submitted {mode} viewpoint goal: distance={distance:.3f} m, '
            f'yaw_error={yaw_error:.3f} rad.'
        )

    def clamp_refinement_goal(self, goal, robot, dx, dy, yaw_error):
        distance = math.hypot(dx, dy)
        if distance > self.max_refinement_step > 0.0:
            scale = self.max_refinement_step / distance
            goal.pose.position.x = robot[0] + dx * scale
            goal.pose.position.y = robot[1] + dy * scale
        limited_yaw_error = max(
            -self.max_refinement_yaw,
            min(yaw_error, self.max_refinement_yaw),
        )
        set_pose_yaw(goal.pose, robot[2] + limited_yaw_error)
        return goal

    def goal_response_callback(self, future, token):
        try:
            goal_handle = future.result()
        except Exception as exc:
            if token != self.nav_goal_token:
                return
            self.get_logger().error(f'Goal request failed: {exc}')
            self.fail_task('Nav2 goal request failed.')
            return
        if token != self.nav_goal_token:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().error(f'Nav2 rejected the {self.mode} goal.')
            self.fail_task('Nav2 rejected the goal.')
            return
        self.goal_handle = goal_handle
        self.state = 'NAVIGATING'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, token=token: self.navigation_result_callback(
                future,
                token,
            )
        )

    def navigation_result_callback(self, future, token):
        if token != self.nav_goal_token:
            return
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f'Cannot read navigation result: {exc}')
            self.fail_task('Could not read the Nav2 result.')
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'{self.mode} navigation ended with status {status}; '
                'refinement will not start.'
            )
            self.fail_task(f'Navigation ended with status {status}.')
            return
        self.get_logger().info(f'{self.mode} navigation succeeded.')
        if not self.enable_refinement:
            self.complete('Refinement is disabled.')
            return
        self.state = 'SETTLING'
        self.settle_deadline = self.now_seconds() + self.settle_time

    def tick(self):
        now = self.now_seconds()
        if (
            self.state == 'SETTLING' and
            self.settle_deadline is not None and
            now >= self.settle_deadline
        ):
            if self.refinement_mode == 'original_goal':
                self.start_original_goal_refinement()
            else:
                self.request_refinement_detection()
        elif (
            self.state == 'WAIT_REFINED_GOAL' and
            self.detection_deadline is not None and
            now >= self.detection_deadline
        ):
            self.complete('Timed out waiting for a refined SAM3 viewpoint.')

    def start_original_goal_refinement(self):
        self.settle_deadline = None
        if self.refinement_target_goal is None:
            self.complete('No original goal is available for refinement.')
            return
        self.start_goal(self.refinement_target_goal, refinement=True)

    def request_refinement_detection(self):
        self.settle_deadline = None
        if not self.detect_client.service_is_ready():
            self.complete(
                'SAM3 detection service is unavailable; navigation is done '
                'without visual refinement.'
            )
            return
        self.refinement_bbox_version = self.bbox_version
        self.refinement_bbox_received = False
        self.pending_refinement_goal = None
        self.state = 'REQUESTING_DETECTION'
        future = self.detect_client.call_async(Trigger.Request())
        future.add_done_callback(self.detection_response_callback)
        self.get_logger().info('Requested SAM3 post-arrival refinement check.')

    def detection_response_callback(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.complete(f'SAM3 refinement request failed: {exc}')
            return
        if not response.success:
            self.complete(
                f'SAM3 refinement was unavailable: {response.message}'
            )
            return
        self.state = 'WAIT_REFINED_GOAL'
        self.detection_deadline = self.now_seconds() + self.detection_timeout
        self.get_logger().info(
            'SAM3 refinement detection succeeded; waiting for the new '
            'safe viewpoint goal.'
        )
        if self.pending_refinement_goal is not None:
            goal = self.pending_refinement_goal
            self.pending_refinement_goal = None
            self.start_goal(goal, refinement=True)

    def complete(self, message):
        if self.start_final_yaw_if_needed(message):
            return
        self.finish_task(message)

    def start_final_yaw_if_needed(self, completion_message):
        if not self.enable_final_yaw:
            return False
        if self.refinement_target_goal is None:
            return False
        robot = self.current_pose()
        if robot is None:
            return False

        target = self.refinement_target_goal.pose
        dx = target.position.x - robot[0]
        dy = target.position.y - robot[1]
        position_error = math.hypot(dx, dy)
        target_yaw = yaw_from_quaternion(target.orientation)
        yaw_error = normalize_angle(target_yaw - robot[2])
        if position_error > self.final_yaw_position_tolerance:
            self.get_logger().info(
                'Skipping final yaw correction because position error is '
                f'{position_error:.3f} m.'
            )
            return False
        if abs(yaw_error) <= self.final_yaw_tolerance:
            self.get_logger().info(
                'Skipping final yaw correction because yaw error is '
                f'{math.degrees(abs(yaw_error)):.1f} deg.'
            )
            return False
        if not self.spin_client.server_is_ready():
            self.get_logger().warn(
                'Spin action is not ready; finishing without final yaw '
                'correction.'
            )
            return False

        spin_goal = Spin.Goal()
        spin_goal.target_yaw = float(yaw_error)
        allowance = max(self.final_yaw_time_allowance, 0.0)
        spin_goal.time_allowance = DurationMsg(
            sec=int(allowance),
            nanosec=int((allowance % 1.0) * 1e9),
        )
        self.state = 'FINAL_YAW_PENDING'
        self.mode = 'final_yaw'
        self.spin_goal_token += 1
        token = self.spin_goal_token
        future = self.spin_client.send_goal_async(spin_goal)
        future.add_done_callback(
            lambda future, token=token: self.final_yaw_response_callback(
                future,
                token,
            )
        )
        self.get_logger().info(
            'Starting final yaw correction with Nav2 Spin: '
            f'yaw_error={math.degrees(yaw_error):.1f} deg. '
            f'Previous step: {completion_message}'
        )
        return True

    def final_yaw_response_callback(self, future, token):
        try:
            goal_handle = future.result()
        except Exception as exc:
            if token != self.spin_goal_token:
                return
            self.get_logger().error(f'Final yaw request failed: {exc}')
            self.finish_task('Final yaw request failed.')
            return
        if token != self.spin_goal_token:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the final yaw Spin goal.')
            self.finish_task('Final yaw was rejected.')
            return
        self.spin_goal_handle = goal_handle
        self.state = 'FINAL_YAW_NAVIGATING'
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, token=token: self.final_yaw_result_callback(
                future,
                token,
            )
        )

    def final_yaw_result_callback(self, future, token):
        if token != self.spin_goal_token:
            return
        self.spin_goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().error(f'Cannot read final yaw result: {exc}')
            self.finish_task('Could not read final yaw result.')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.finish_task('Final yaw correction succeeded.')
            return
        self.get_logger().warn(
            f'Final yaw correction ended with status {status}.'
        )
        self.finish_task(f'Final yaw ended with status {status}.')

    def finish_task(self, message):
        self.state = (
            'WAIT_INITIAL_GOAL' if self.auto_rearm else 'COMPLETE'
        )
        self.mode = None
        self.goal_handle = None
        self.spin_goal_handle = None
        self.detection_deadline = None
        self.settle_deadline = None
        self.refinement_target_goal = None
        self.refinement_ignores_yaw = False
        self.get_logger().info(f'Hybrid viewpoint task complete: {message}')

    def fail_task(self, message):
        self.state = (
            'WAIT_INITIAL_GOAL' if self.auto_rearm else 'COMPLETE'
        )
        self.mode = None
        self.goal_handle = None
        self.spin_goal_handle = None
        self.detection_deadline = None
        self.settle_deadline = None
        self.refinement_bbox_version = None
        self.refinement_bbox_received = False
        self.pending_refinement_goal = None
        self.refinement_target_goal = None
        self.refinement_ignores_yaw = False
        self.get_logger().warn(
            f'Hybrid viewpoint task stopped: {message} '
            f'Next state is {self.state}.'
        )

    def reset_callback(self, _request, response):
        had_active_goal = self.state in (
            'GOAL_PENDING',
            'NAVIGATING',
            'FINAL_YAW_PENDING',
            'FINAL_YAW_NAVIGATING',
        )
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            had_active_goal = True
        if self.spin_goal_handle is not None:
            self.spin_goal_handle.cancel_goal_async()
            had_active_goal = True
        self.nav_goal_token += 1
        self.spin_goal_token += 1
        self.publish_stop()
        self.state = 'WAIT_INITIAL_GOAL'
        self.mode = None
        self.goal_handle = None
        self.spin_goal_handle = None
        self.fine_attempts = 0
        self.refinement_bbox_version = None
        self.refinement_bbox_received = False
        self.pending_refinement_goal = None
        self.refinement_target_goal = None
        self.refinement_ignores_yaw = False
        self.settle_deadline = None
        self.detection_deadline = None
        response.success = True
        response.message = (
            'Canceled the active hybrid goal and stopped the robot.'
            if had_active_goal else
            'Hybrid viewpoint orchestrator is ready for a new task.'
        )
        return response

    def publish_stop(self):
        stop = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(stop)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = HybridViewpointOrchestrator()
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
