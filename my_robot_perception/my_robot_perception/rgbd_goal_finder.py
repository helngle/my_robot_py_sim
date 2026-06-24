import math
from copy import deepcopy

import cv2
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

import tf2_geometry_msgs  # noqa: F401

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return {
        'z': math.sin(half_yaw),
        'w': math.cos(half_yaw),
    }


class RgbdGoalFinder(Node):
    def __init__(self):
        super().__init__('rgbd_goal_finder')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/color/camera_info',
        )
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('target_detector', 'yolo')
        self.declare_parameter('target_color', 'white')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('yolo_model', '/home/jensen/ros2_ws/yolo11n.pt')
        self.declare_parameter('yolo_device', 'cuda:0')
        self.declare_parameter('yolo_confidence', 0.10)
        self.declare_parameter('yolo_imgsz', 640)
        self.declare_parameter('use_yolo_tracking', True)
        self.declare_parameter('yolo_tracker', 'bytetrack.yaml')
        self.declare_parameter('yolo_observation_classes', ['*'])
        self.declare_parameter('yolo_observation_confidence', 0.25)
        self.declare_parameter('lock_yolo_target', True)
        self.declare_parameter('lock_lost_frames', 15)
        self.declare_parameter('min_depth_m', 0.25)
        self.declare_parameter('max_depth_m', 5.0)
        self.declare_parameter('min_area_px', 300.0)
        self.declare_parameter('depth_window_px', 9)
        self.declare_parameter('white_min_value', 150)
        self.declare_parameter('white_max_saturation', 90)
        self.declare_parameter('approach_distance_m', 0.8)
        self.declare_parameter('goal_z', 0.0)
        self.declare_parameter('process_rate_hz', 8.0)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop_s', 0.08)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('enable_target_localization', True)
        self.declare_parameter('auto_send_goal', False)
        self.declare_parameter('auto_send_cooldown_s', 10.0)
        self.declare_parameter('follow_enabled', False)
        self.declare_parameter('follow_rate_hz', 0.3)
        self.declare_parameter('follow_min_goal_delta_m', 0.55)
        self.declare_parameter('follow_target_timeout_s', 3.0)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.target_detector = self.get_parameter('target_detector').value
        self.target_color = self.get_parameter('target_color').value
        self.target_class = self.get_parameter('target_class').value
        self.yolo_model_path = self.get_parameter('yolo_model').value
        self.yolo_device = self.get_parameter('yolo_device').value
        self.yolo_confidence = float(
            self.get_parameter('yolo_confidence').value
        )
        self.yolo_imgsz = int(self.get_parameter('yolo_imgsz').value)
        self.use_yolo_tracking = bool(
            self.get_parameter('use_yolo_tracking').value
        )
        self.yolo_tracker = self.get_parameter('yolo_tracker').value
        self.yolo_observation_classes = set(
            self.get_parameter('yolo_observation_classes').value
        )
        self.observe_all_yolo_classes = (
            '*' in self.yolo_observation_classes
        )
        self.yolo_observation_confidence = float(
            self.get_parameter('yolo_observation_confidence').value
        )
        self.lock_yolo_target = bool(
            self.get_parameter('lock_yolo_target').value
        )
        self.lock_lost_frames = int(
            self.get_parameter('lock_lost_frames').value
        )
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.min_area_px = float(self.get_parameter('min_area_px').value)
        self.depth_window_px = int(self.get_parameter('depth_window_px').value)
        self.white_min_value = int(self.get_parameter('white_min_value').value)
        self.white_max_saturation = int(
            self.get_parameter('white_max_saturation').value
        )
        self.approach_distance_m = float(
            self.get_parameter('approach_distance_m').value
        )
        self.goal_z = float(self.get_parameter('goal_z').value)
        self.process_period = 1.0 / float(
            self.get_parameter('process_rate_hz').value
        )
        self.sync_queue_size = int(
            self.get_parameter('sync_queue_size').value
        )
        self.sync_slop_s = float(self.get_parameter('sync_slop_s').value)
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value
        )
        self.enable_target_localization = bool(
            self.get_parameter('enable_target_localization').value
        )
        self.auto_send_goal = bool(self.get_parameter('auto_send_goal').value)
        self.auto_send_cooldown_s = float(
            self.get_parameter('auto_send_cooldown_s').value
        )
        self.follow_enabled = bool(
            self.get_parameter('follow_enabled').value
        )
        self.follow_rate_hz = float(self.get_parameter('follow_rate_hz').value)
        self.follow_min_goal_delta_m = float(
            self.get_parameter('follow_min_goal_delta_m').value
        )
        self.follow_target_timeout_s = float(
            self.get_parameter('follow_target_timeout_s').value
        )
        self.yolo_model = None
        if self.target_detector == 'yolo':
            self.yolo_model = self.load_yolo_model()

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )

        self.latest_target = None
        self.latest_goal = None
        self.latest_goal_time = None
        self.latest_track_id = None
        self.last_process_time = self.get_clock().now()
        self.last_goal_time = None
        self.last_follow_goal = None
        self.last_follow_track_id = None
        self.active_goal_handle = None
        self.locked_track_id = None
        self.lock_lost_count = 0
        self.yolo_observations = []

        self.target_pub = self.create_publisher(
            PointStamped,
            'rgbd_target',
            10,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            'rgbd_goal',
            10,
        )
        self.marker_pub = self.create_publisher(
            Marker,
            'rgbd_goal_marker',
            10,
        )
        self.debug_image_pub = self.create_publisher(
            Image,
            'rgbd_debug_image',
            qos_profile_sensor_data,
        )

        self.color_sub = Subscriber(
            self,
            Image,
            self.color_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.camera_info_sub = Subscriber(
            self,
            CameraInfo,
            self.camera_info_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.rgbd_sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.camera_info_sub],
            queue_size=max(self.sync_queue_size, 1),
            slop=max(self.sync_slop_s, 0.0),
        )
        self.rgbd_sync.registerCallback(self.rgbd_callback)
        self.create_service(
            Trigger,
            'send_rgbd_goal',
            self.send_goal_service,
        )
        self.create_service(
            Trigger,
            'unlock_rgbd_target',
            self.unlock_target_service,
        )
        self.create_service(
            Trigger,
            'start_rgbd_follow',
            self.start_follow_service,
        )
        self.create_service(
            Trigger,
            'stop_rgbd_follow',
            self.stop_follow_service,
        )
        self.create_timer(
            1.0 / max(self.follow_rate_hz, 0.1),
            self.follow_timer_callback,
        )

        self.get_logger().info(
            f'RGBD {self.target_label} goal finder listening on '
            f'{self.color_topic}, {self.depth_topic}, {self.camera_info_topic}'
        )

    @property
    def target_label(self):
        if self.target_detector == 'yolo':
            return f'yolo:{self.target_class}'
        return f'color:{self.target_color}'

    def load_yolo_model(self):
        if YOLO is None:
            self.get_logger().error(
                'ultralytics is not installed; RGBD YOLO detection is disabled'
            )
            return None

        model = YOLO(self.yolo_model_path)
        self.get_logger().info(
            'Loaded YOLO model '
            f'{self.yolo_model_path} for class "{self.target_class}" '
            f'on {self.yolo_device}'
        )
        return model

    def rgbd_callback(self, msg, depth_msg, camera_info):
        now = self.get_clock().now()
        elapsed = (now - self.last_process_time).nanoseconds * 1e-9
        if elapsed < self.process_period:
            return
        self.last_process_time = now

        try:
            color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'Failed to convert RGB image: {exc}')
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough',
            )
        except CvBridgeError as exc:
            self.get_logger().warn(f'Failed to convert depth image: {exc}')
            return

        try:
            detection = self.find_target(color)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'RGBD detector failed: {exc}')
            if self.publish_debug_image:
                self.publish_debug_image_msg(
                    color,
                    msg.header,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    f'detector error: {exc}',
                )
            return
        if detection is None:
            if self.publish_debug_image:
                self.publish_debug_image_msg(
                    color,
                    msg.header,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                )
            return

        u, v, score, box, track_id = detection
        depth_m = self.sample_depth_m(depth, u, v, depth_msg.encoding)
        if self.publish_debug_image:
            self.publish_debug_image_msg(
                color,
                msg.header,
                box,
                u,
                v,
                score,
                depth_m,
                track_id,
                None,
            )
        if not self.enable_target_localization:
            return
        if depth_m is None:
            self.get_logger().debug(
                f'{self.target_label} detected but no valid depth nearby'
            )
            return

        camera_point = self.pixel_to_camera_point(
            u,
            v,
            depth_m,
            depth_msg.header.frame_id or msg.header.frame_id,
            depth_msg.header.stamp,
            camera_info,
        )
        try:
            target = self.tf_buffer.transform(
                camera_point,
                self.target_frame,
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Cannot transform RGBD target to map: {exc}'
            )
            return

        self.latest_target = target
        self.latest_goal = self.make_goal_pose(target)
        self.latest_goal_time = now
        self.latest_track_id = track_id
        self.target_pub.publish(target)
        self.goal_pub.publish(self.latest_goal)
        self.marker_pub.publish(self.make_marker(target, score))

        if self.auto_send_goal and self.can_auto_send(now):
            self.send_latest_goal()
            self.last_goal_time = now

    def find_target(self, color):
        if self.target_detector == 'yolo':
            return self.find_yolo_target(color)

        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        if self.target_color == 'white':
            return self.find_white_target(hsv)
        return self.find_red_target(hsv)

    def find_yolo_target(self, color):
        self.yolo_observations = []
        if self.yolo_model is None:
            return None

        if self.use_yolo_tracking:
            results = self.yolo_model.track(
                source=color,
                imgsz=self.yolo_imgsz,
                conf=self.yolo_confidence,
                device=self.yolo_device,
                tracker=self.yolo_tracker,
                persist=True,
                verbose=False,
            )
        else:
            results = self.yolo_model.predict(
                source=color,
                imgsz=self.yolo_imgsz,
                conf=self.yolo_confidence,
                device=self.yolo_device,
                verbose=False,
            )
        if not results:
            return None

        names = results[0].names
        candidates = []
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            class_name = names.get(class_id, str(class_id))
            confidence = float(box.conf[0])
            track_id = None
            if getattr(box, 'id', None) is not None:
                track_id = int(box.id[0])
            if (
                confidence >= self.yolo_observation_confidence and
                (
                    self.observe_all_yolo_classes or
                    class_name in self.yolo_observation_classes
                )
            ):
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                self.yolo_observations.append({
                    'class_name': class_name,
                    'confidence': confidence,
                    'track_id': track_id,
                    'box': (x1, y1, x2, y2),
                })
            if class_name != self.target_class:
                continue
            candidates.append((confidence, track_id, box))

        if not candidates:
            if self.locked_track_id is not None:
                self.lock_lost_count += 1
                if self.lock_lost_count >= self.lock_lost_frames:
                    self.clear_locked_target()
            return None

        selected_confidence, selected_track_id, selected_box = (
            self.select_yolo_candidate(candidates)
        )
        if selected_box is None:
            return None
        x1, y1, x2, y2 = selected_box.xyxy[0].tolist()
        u = int((x1 + x2) * 0.5)
        v = int((y1 + y2) * 0.5)
        return u, v, selected_confidence, (x1, y1, x2, y2), selected_track_id

    def select_yolo_candidate(self, candidates):
        if self.lock_yolo_target and self.locked_track_id is not None:
            for confidence, track_id, box in candidates:
                if track_id == self.locked_track_id:
                    self.lock_lost_count = 0
                    return confidence, track_id, box

            self.lock_lost_count += 1
            if self.lock_lost_count < self.lock_lost_frames:
                return None, None, None
            self.clear_locked_target()

        selected = max(candidates, key=lambda candidate: candidate[0])
        if self.lock_yolo_target and selected[1] is not None:
            self.locked_track_id = selected[1]
            self.lock_lost_count = 0
            self.get_logger().info(
                f'Locked RGBD YOLO target id={self.locked_track_id}'
            )
        return selected

    def clear_locked_target(self):
        if self.locked_track_id is not None:
            self.get_logger().info(
                f'Unlocked RGBD YOLO target id={self.locked_track_id}'
            )
        self.locked_track_id = None
        self.lock_lost_count = 0

    def find_white_target(self, hsv):
        lower_white = np.array(
            [0, 0, self.white_min_value],
            dtype=np.uint8,
        )
        upper_white = np.array(
            [180, self.white_max_saturation, 255],
            dtype=np.uint8,
        )
        mask = cv2.inRange(hsv, lower_white, upper_white)
        return self.find_largest_mask_target(mask)

    def find_red_target(self, hsv):
        lower_red_1 = np.array([0, 80, 60], dtype=np.uint8)
        upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)
        lower_red_2 = np.array([170, 80, 60], dtype=np.uint8)
        upper_red_2 = np.array([180, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_red_1, upper_red_1)
        mask |= cv2.inRange(hsv, lower_red_2, upper_red_2)
        return self.find_largest_mask_target(mask)

    def find_largest_mask_target(self, mask):
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < self.min_area_px:
            return None

        moments = cv2.moments(contour)
        if moments['m00'] == 0.0:
            return None
        u = int(moments['m10'] / moments['m00'])
        v = int(moments['m01'] / moments['m00'])
        x, y, width, height = cv2.boundingRect(contour)
        return u, v, area, (x, y, x + width, y + height), None

    def publish_debug_image_msg(
        self,
        color,
        header,
        box,
        u,
        v,
        score,
        depth_m,
        track_id,
        status_label=None,
    ):
        debug = color.copy()
        for observation in self.yolo_observations:
            obs_box = observation['box']
            ox1, oy1, ox2, oy2 = [int(value) for value in obs_box]
            cv2.rectangle(
                debug,
                (ox1, oy1),
                (ox2, oy2),
                (80, 210, 80),
                2,
            )
            obs_id = ''
            if observation['track_id'] is not None:
                obs_id = f' id:{observation["track_id"]}'
            obs_label = (
                f'{observation["class_name"]}{obs_id} '
                f'{observation["confidence"]:.2f}'
            )
            cv2.putText(
                debug,
                obs_label,
                (ox1, max(18, oy1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (80, 210, 80),
                2,
                cv2.LINE_AA,
            )
        if box is not None:
            x1, y1, x2, y2 = [int(value) for value in box]
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 255), 2)

        if u is not None and v is not None:
            cv2.drawMarker(
                debug,
                (int(u), int(v)),
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=18,
                thickness=2,
            )

        if status_label is not None:
            label = status_label
        elif score is None:
            label = f'no {self.target_label}'
        else:
            depth_text = 'depth: --'
            if depth_m is not None:
                depth_text = f'depth: {depth_m:.2f}m'
            id_text = ''
            if track_id is not None:
                id_text = f' id:{track_id}'
            label = f'{self.target_label}{id_text} {score:.2f} {depth_text}'
        cv2.putText(
            debug,
            label,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        try:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'Failed to publish debug image: {exc}')
            return

        msg.header = header
        self.debug_image_pub.publish(msg)

    def sample_depth_m(self, depth, u, v, encoding):
        half_window = max(1, self.depth_window_px // 2)
        y_min = max(0, v - half_window)
        y_max = min(depth.shape[0], v + half_window + 1)
        x_min = max(0, u - half_window)
        x_max = min(depth.shape[1], u + half_window + 1)
        roi = np.asarray(depth[y_min:y_max, x_min:x_max]).astype(np.float32)

        if encoding in ('16UC1', 'mono16'):
            roi *= 0.001

        valid = roi[np.isfinite(roi)]
        valid = valid[
            (valid >= self.min_depth_m) & (valid <= self.max_depth_m)
        ]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def pixel_to_camera_point(
        self,
        u,
        v,
        depth_m,
        frame_id,
        stamp,
        camera_info,
    ):
        projection = camera_info.p
        fx = projection[0] if projection[0] else camera_info.k[0]
        fy = projection[5] if projection[5] else camera_info.k[4]
        cx = projection[2] if projection[2] else camera_info.k[2]
        cy = projection[6] if projection[6] else camera_info.k[5]

        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = frame_id
        point.point.x = (float(u) - cx) * depth_m / fx
        point.point.y = (float(v) - cy) * depth_m / fy
        point.point.z = depth_m
        return point

    def make_goal_pose(self, target):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.target_frame

        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            robot_x = robot_tf.transform.translation.x
            robot_y = robot_tf.transform.translation.y
        except TransformException:
            robot_x = target.point.x
            robot_y = target.point.y - 1.0

        dx = target.point.x - robot_x
        dy = target.point.y - robot_y
        distance = math.hypot(dx, dy)
        if distance > self.approach_distance_m:
            scale = (distance - self.approach_distance_m) / distance
            goal_x = robot_x + dx * scale
            goal_y = robot_y + dy * scale
        else:
            goal_x = robot_x
            goal_y = robot_y

        yaw = math.atan2(target.point.y - goal_y, target.point.x - goal_x)
        quat = yaw_to_quaternion(yaw)
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = self.goal_z
        goal.pose.orientation.z = quat['z']
        goal.pose.orientation.w = quat['w']
        return goal

    def make_marker(self, target, area):
        marker = Marker()
        marker.header = target.header
        marker.ns = 'rgbd_goal'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target.point.x
        marker.pose.position.y = target.point.y
        marker.pose.position.z = max(target.point.z, 0.2)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        if self.target_detector == 'yolo':
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 1.0
        elif self.target_color == 'white':
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.05
            marker.color.b = 0.05
        marker.color.a = 0.85
        marker.lifetime = Duration(seconds=1.0).to_msg()
        return marker

    def can_auto_send(self, now):
        if self.last_goal_time is None:
            return True
        elapsed = (now - self.last_goal_time).nanoseconds * 1e-9
        return elapsed >= self.auto_send_cooldown_s

    def has_fresh_goal(self):
        if self.latest_goal is None or self.latest_goal_time is None:
            return False
        age = (
            self.get_clock().now() - self.latest_goal_time
        ).nanoseconds * 1e-9
        return age <= self.follow_target_timeout_s

    def should_update_follow_goal(self):
        if self.last_follow_goal is None:
            return True
        if self.latest_track_id != self.last_follow_track_id:
            return True

        dx = (
            self.latest_goal.pose.position.x -
            self.last_follow_goal.pose.position.x
        )
        dy = (
            self.latest_goal.pose.position.y -
            self.last_follow_goal.pose.position.y
        )
        return math.hypot(dx, dy) >= self.follow_min_goal_delta_m

    def copy_goal_pose(self, goal):
        return deepcopy(goal)

    def follow_timer_callback(self):
        if not self.follow_enabled:
            return
        if not self.has_fresh_goal():
            return
        if not self.should_update_follow_goal():
            return

        if self.send_latest_goal('follow'):
            self.last_follow_goal = self.copy_goal_pose(self.latest_goal)
            self.last_follow_track_id = self.latest_track_id

    def send_goal_service(self, request, response):
        del request
        if self.latest_goal is None:
            response.success = False
            response.message = (
                f'No valid {self.target_label} RGBD goal has been '
                'detected yet.'
            )
            return response

        sent = self.send_latest_goal()
        response.success = sent
        response.message = (
            f'Sent latest {self.target_label} RGBD goal to Nav2.'
            if sent else
            'Nav2 navigate_to_pose action server is not available.'
        )
        return response

    def start_follow_service(self, request, response):
        del request
        self.follow_enabled = True
        self.last_follow_goal = None
        self.last_follow_track_id = None
        response.success = True
        response.message = (
            'RGBD follow mode started.'
            if self.has_fresh_goal() else
            'RGBD follow mode started; waiting for a fresh target.'
        )
        return response

    def stop_follow_service(self, request, response):
        del request
        self.follow_enabled = False
        self.last_follow_goal = None
        self.last_follow_track_id = None
        if self.active_goal_handle is not None:
            self.active_goal_handle.cancel_goal_async()
        response.success = True
        response.message = 'RGBD follow mode stopped.'
        return response

    def unlock_target_service(self, request, response):
        del request
        self.clear_locked_target()
        response.success = True
        response.message = 'Unlocked RGBD YOLO target.'
        return response

    def send_latest_goal(self, mode='single'):
        if self.latest_goal is None:
            return False
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.latest_goal
        goal_future = self.nav_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(
            f'Sent RGBD {self.target_label} {mode} goal: '
            f'x={self.latest_goal.pose.position.x:.2f}, '
            f'y={self.latest_goal.pose.position.y:.2f}'
        )
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('RGBD goal was rejected by Nav2.')
            return
        self.active_goal_handle = goal_handle


def main(args=None):
    rclpy.init(args=args)
    node = RgbdGoalFinder()
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
