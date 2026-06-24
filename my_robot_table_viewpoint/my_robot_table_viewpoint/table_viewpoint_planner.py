import math
import os
from copy import deepcopy

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from action_msgs.msg import GoalStatus
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_matrix(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return np.array([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])


def transform_to_matrix(transform):
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_matrix(transform.rotation)
    matrix[0, 3] = transform.translation.x
    matrix[1, 3] = transform.translation.y
    matrix[2, 3] = transform.translation.z
    return matrix


def pose_matrix(x, y, z, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.array([
        [cosine, -sine, 0.0, x],
        [sine, cosine, 0.0, y],
        [0.0, 0.0, 1.0, z],
        [0.0, 0.0, 0.0, 1.0],
    ])


def yaw_quaternion(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def normalize_yaw(yaw):
    return math.atan2(math.sin(yaw), math.cos(yaw))


class TableViewpointPlanner(Node):
    def __init__(self):
        super().__init__('table_viewpoint_planner')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_frame', 'map'),
                ('base_frame', 'base_footprint'),
                ('depth_topic', '/camera/depth/image_raw'),
                ('camera_info_topic', '/camera/color/camera_info'),
                ('clicked_point_topic', '/clicked_point'),
                ('global_costmap_topic', '/global_costmap/costmap'),
                ('local_costmap_topic', '/local_costmap/costmap'),
                ('database_file', ''),
                ('table_name', 'office_desk_1'),
                ('table_length_m', 1.40),
                ('table_width_m', 0.60),
                ('table_height_m', 0.73),
                ('observe_tabletop_only', True),
                ('tabletop_thickness_m', 0.05),
                ('search_radius_m', 1.0),
                ('tabletop_height_tolerance_m', 0.10),
                ('plane_tolerance_m', 0.035),
                ('depth_min_m', 0.30),
                ('depth_max_m', 5.0),
                ('depth_pixel_stride', 4),
                ('min_plane_points', 80),
                ('robot_length_m', 0.80),
                ('robot_width_m', 0.70),
                ('footprint_margin_m', 0.05),
                ('occupied_cost_threshold', 50),
                ('reject_unknown_cost', True),
                ('min_standoff_m', 0.60),
                ('max_standoff_m', 3.00),
                ('standoff_step_m', 0.10),
                ('image_margin_ratio', 0.05),
                ('center_error_max', 0.06),
                ('auto_detect_after_click', True),
                ('auto_send_goal', True),
                ('publish_table_marker', True),
                ('publish_goal_marker', True),
                ('publish_candidate_markers', False),
            ],
        )

        self.map_frame = self.parameter('map_frame')
        self.base_frame = self.parameter('base_frame')
        self.database_file = os.path.expanduser(
            self.parameter('database_file')
        )
        self.table_name = self.parameter('table_name')
        self.table_length = float(self.parameter('table_length_m'))
        self.table_width = float(self.parameter('table_width_m'))
        self.table_height = float(self.parameter('table_height_m'))
        self.observe_tabletop_only = bool(
            self.parameter('observe_tabletop_only')
        )
        self.tabletop_thickness = float(
            self.parameter('tabletop_thickness_m')
        )
        self.search_radius = float(self.parameter('search_radius_m'))
        self.tabletop_height_tolerance = float(
            self.parameter('tabletop_height_tolerance_m')
        )
        self.plane_tolerance = float(self.parameter('plane_tolerance_m'))
        self.depth_min = float(self.parameter('depth_min_m'))
        self.depth_max = float(self.parameter('depth_max_m'))
        self.depth_stride = max(1, int(self.parameter('depth_pixel_stride')))
        self.min_plane_points = int(self.parameter('min_plane_points'))
        self.robot_length = float(self.parameter('robot_length_m'))
        self.robot_width = float(self.parameter('robot_width_m'))
        self.footprint_margin = float(self.parameter('footprint_margin_m'))
        self.occupied_threshold = int(
            self.parameter('occupied_cost_threshold')
        )
        self.reject_unknown = bool(self.parameter('reject_unknown_cost'))
        self.min_standoff = float(self.parameter('min_standoff_m'))
        self.max_standoff = float(self.parameter('max_standoff_m'))
        self.standoff_step = float(self.parameter('standoff_step_m'))
        self.image_margin_ratio = float(
            self.parameter('image_margin_ratio')
        )
        self.center_error_max = float(
            self.parameter('center_error_max')
        )
        self.auto_detect = bool(self.parameter('auto_detect_after_click'))
        self.auto_send_goal = bool(self.parameter('auto_send_goal'))
        self.publish_table_marker = bool(
            self.parameter('publish_table_marker')
        )
        self.publish_goal_marker = bool(
            self.parameter('publish_goal_marker')
        )
        self.publish_candidate_markers = bool(
            self.parameter('publish_candidate_markers')
        )

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_state_client = self.create_client(
            GetState,
            '/bt_navigator/get_state',
        )

        self.camera_info = None
        self.latest_depth = None
        self.global_costmap = None
        self.local_costmap = None
        self.search_seed = None
        self.pending_detection = False
        self.table = None
        self.best_goal = None
        self.candidates = []
        self.last_plan_status = None
        self.latest_tf_fallback_reported = False
        self.auto_goal_sent = False
        self.goal_send_pending = False
        self.nav_state_pending = False
        self.nav_is_active = False
        self.nav_wait_reported = False
        self.goal_handle = None
        self.marker_clear_published = False

        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'table_viewpoint_markers',
            marker_qos,
        )
        self.goal_pub = self.create_publisher(
            PoseStamped,
            'table_viewpoint_goal',
            marker_qos,
        )

        self.create_subscription(
            CameraInfo,
            self.parameter('camera_info_topic'),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.parameter('depth_topic'),
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointStamped,
            self.parameter('clicked_point_topic'),
            self.clicked_point_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.parameter('global_costmap_topic'),
            self.global_costmap_callback,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            self.parameter('local_costmap_topic'),
            self.local_costmap_callback,
            10,
        )
        self.create_service(
            Trigger,
            'detect_clicked_table',
            self.detect_service,
        )
        self.create_service(
            Trigger,
            'save_table_calibration',
            self.save_service,
        )
        self.create_service(
            Trigger,
            'send_table_viewpoint',
            self.send_goal_service,
        )
        self.create_service(
            Trigger,
            'clear_table_calibration',
            self.clear_service,
        )
        self.create_timer(1.0, self.refresh_callback)

        self.load_table_database()
        self.get_logger().info(
            'Table viewpoint planner is isolated from the existing RGBD goal '
            'pipeline. Use RViz Publish Point near the tabletop to calibrate.'
        )

    def parameter(self, name):
        return self.get_parameter(name).value

    def camera_info_callback(self, msg):
        self.camera_info = msg
        if self.table is not None and self.global_costmap is not None:
            self.plan_viewpoint()

    def global_costmap_callback(self, msg):
        self.global_costmap = msg
        if self.table is not None and self.camera_info is not None:
            self.plan_viewpoint()

    def local_costmap_callback(self, msg):
        self.local_costmap = msg
        if self.table is not None and self.camera_info is not None:
            self.plan_viewpoint()

    def depth_callback(self, msg):
        self.latest_depth = msg
        if self.pending_detection and self.camera_info is not None:
            self.pending_detection = False
            self.detect_table(msg)

    def clicked_point_callback(self, msg):
        point = self.point_in_map(msg)
        if point is None:
            return
        self.search_seed = point
        self.pending_detection = self.auto_detect
        self.auto_goal_sent = False
        self.get_logger().info(
            f'Table search seed: x={point[0]:.2f}, y={point[1]:.2f}; '
            'waiting for the next depth frame.'
        )
        self.publish_markers()

    def point_in_map(self, msg):
        if not msg.header.frame_id or msg.header.frame_id == self.map_frame:
            return np.array([msg.point.x, msg.point.y, msg.point.z])
        try:
            stamp = Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == 0:
                stamp = Time()
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.3),
            )
        except TransformException as exc:
            self.get_logger().warn(f'Cannot transform clicked point: {exc}')
            return None
        matrix = transform_to_matrix(transform.transform)
        point = matrix @ np.array([
            msg.point.x,
            msg.point.y,
            msg.point.z,
            1.0,
        ])
        return point[:3]

    def detect_service(self, request, response):
        del request
        if self.search_seed is None:
            response.success = False
            response.message = 'Use RViz Publish Point near the table first.'
            return response
        if self.latest_depth is None or self.camera_info is None:
            response.success = False
            response.message = 'No synchronized camera depth/intrinsics received.'
            return response
        response.success = self.detect_table(self.latest_depth)
        response.message = (
            'Table geometry detected.'
            if response.success else
            'No matching tabletop plane found in the clicked region.'
        )
        return response

    def detect_table(self, depth_msg):
        if self.search_seed is None or self.camera_info is None:
            return False
        points = self.depth_points_in_map(depth_msg, self.camera_info)
        if points is None:
            return False
        dx = points[:, 0] - self.search_seed[0]
        dy = points[:, 1] - self.search_seed[1]
        radial_mask = dx * dx + dy * dy <= self.search_radius ** 2
        expected_top = self.table_height
        height_mask = (
            np.abs(points[:, 2] - expected_top) <=
            self.tabletop_height_tolerance
        )
        points = points[radial_mask & height_mask]
        if points.shape[0] < self.min_plane_points:
            self.get_logger().warn(
                f'Only {points.shape[0]} possible tabletop points found; '
                f'need at least {self.min_plane_points}.'
            )
            return False

        tabletop_z = self.dominant_tabletop_height(points[:, 2])
        plane = points[
            np.abs(points[:, 2] - tabletop_z) <= self.plane_tolerance
        ]
        if plane.shape[0] < self.min_plane_points:
            self.get_logger().warn(
                f'Tabletop plane has only {plane.shape[0]} inliers.'
            )
            return False

        center_xy, yaw, observed_size = self.fit_tabletop(plane[:, :2])
        if center_xy is None:
            return False
        self.table = {
            'name': self.table_name,
            'frame_id': self.map_frame,
            'center': [
                float(center_xy[0]),
                float(center_xy[1]),
                float(tabletop_z - self.table_height * 0.5),
            ],
            'yaw': float(yaw),
            'size': [
                self.table_length,
                self.table_width,
                self.table_height,
            ],
        }
        self.get_logger().info(
            'Detected tabletop: '
            f'center=({center_xy[0]:.2f}, {center_xy[1]:.2f}), '
            f'yaw={yaw:.2f}, z={tabletop_z:.2f}, '
            f'observed={observed_size[0]:.2f}x{observed_size[1]:.2f} m.'
        )
        self.plan_viewpoint()
        self.publish_markers()
        return True

    def depth_points_in_map(self, depth_msg, camera_info):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough',
            )
        except CvBridgeError as exc:
            self.get_logger().warn(f'Cannot convert depth image: {exc}')
            return None
        depth = np.asarray(depth, dtype=np.float32)
        if depth_msg.encoding in ('16UC1', 'mono16'):
            depth *= 0.001
        rows = np.arange(0, depth.shape[0], self.depth_stride)
        columns = np.arange(0, depth.shape[1], self.depth_stride)
        uu, vv = np.meshgrid(columns, rows)
        zz = depth[vv, uu]
        valid = (
            np.isfinite(zz) &
            (zz >= self.depth_min) &
            (zz <= self.depth_max)
        )
        if not np.any(valid):
            self.get_logger().warn('Depth image contains no valid points.')
            return None
        projection = camera_info.p
        fx = projection[0] if projection[0] else camera_info.k[0]
        fy = projection[5] if projection[5] else camera_info.k[4]
        cx = projection[2] if projection[2] else camera_info.k[2]
        cy = projection[6] if projection[6] else camera_info.k[5]
        z = zz[valid]
        x = (uu[valid].astype(np.float32) - cx) * z / fx
        y = (vv[valid].astype(np.float32) - cy) * z / fy
        points_camera = np.column_stack((x, y, z, np.ones_like(z)))
        frame_id = depth_msg.header.frame_id or camera_info.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                frame_id,
                Time.from_msg(depth_msg.header.stamp),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exact_error:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    frame_id,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as latest_error:
                self.get_logger().warn(
                    'Cannot transform depth points at capture time or with '
                    f'the latest TF: exact={exact_error}; latest={latest_error}'
                )
                return None
            if not self.latest_tf_fallback_reported:
                self.latest_tf_fallback_reported = True
                self.get_logger().warn(
                    'Depth capture time is slightly newer than the buffered '
                    'map TF; using the latest TF for this calibration. Keep '
                    'the robot stationary while clicking the tabletop.'
                )
        matrix = transform_to_matrix(transform.transform)
        return (matrix @ points_camera.T).T[:, :3]

    def dominant_tabletop_height(self, heights):
        bin_size = max(self.plane_tolerance, 0.01)
        minimum = float(np.min(heights))
        maximum = float(np.max(heights)) + bin_size
        bins = np.arange(minimum, maximum + bin_size, bin_size)
        if bins.size < 2:
            return float(np.median(heights))
        histogram, edges = np.histogram(heights, bins=bins)
        index = int(np.argmax(histogram))
        in_bin = (
            (heights >= edges[index]) &
            (heights < edges[index + 1])
        )
        return float(np.median(heights[in_bin]))

    def fit_tabletop(self, xy_points):
        center = np.mean(xy_points, axis=0)
        centered = xy_points - center
        covariance = np.cov(centered, rowvar=False)
        values, vectors = np.linalg.eigh(covariance)
        long_axis = vectors[:, int(np.argmax(values))]
        long_axis /= np.linalg.norm(long_axis)
        short_axis = np.array([-long_axis[1], long_axis[0]])
        long_values = xy_points @ long_axis
        short_values = xy_points @ short_axis
        long_min, long_max = np.percentile(long_values, [2.0, 98.0])
        short_min, short_max = np.percentile(short_values, [2.0, 98.0])
        observed_length = float(long_max - long_min)
        observed_width = float(short_max - short_min)
        minimum_observed_length = min(self.table_length * 0.30, 0.40)
        minimum_observed_width = min(self.table_width * 0.30, 0.20)
        if (
            observed_length < minimum_observed_length or
            observed_width < minimum_observed_width
        ):
            self.get_logger().warn(
                'Observed plane is too small to determine table orientation: '
                f'{observed_length:.2f}x{observed_width:.2f} m.'
            )
            return None, None, None
        local_center = np.array([
            (long_min + long_max) * 0.5,
            (short_min + short_max) * 0.5,
        ])
        center_xy = (
            long_axis * local_center[0] +
            short_axis * local_center[1]
        )
        yaw = normalize_yaw(math.atan2(long_axis[1], long_axis[0]))
        return center_xy, yaw, (observed_length, observed_width)

    def plan_viewpoint(self):
        if self.table is None or self.camera_info is None:
            return False
        if self.global_costmap is None:
            self.set_plan_status('waiting for /global_costmap/costmap')
            self.publish_markers()
            return False
        camera_frame = self.camera_info.header.frame_id
        if not camera_frame:
            self.set_plan_status('camera_info has no frame_id')
            return False
        try:
            base_to_camera = self.tf_buffer.lookup_transform(
                self.base_frame,
                camera_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.set_plan_status(f'waiting for camera TF: {exc}')
            return False
        robot_xy = self.robot_position()
        table_center = np.array(self.table['center'], dtype=float)
        table_yaw = float(self.table['yaw'])
        long_axis = np.array([
            math.cos(table_yaw),
            math.sin(table_yaw),
        ])
        normal = np.array([-long_axis[1], long_axis[0]])
        transform_base_camera = transform_to_matrix(
            base_to_camera.transform
        )
        corners = self.tabletop_corners()
        candidates = []
        distance_values = np.arange(
            self.min_standoff,
            self.max_standoff + self.standoff_step * 0.5,
            self.standoff_step,
        )
        for side in (-1.0, 1.0):
            outward = normal * side
            for standoff in distance_values:
                goal_xy = (
                    table_center[:2] +
                    outward * (self.table_width * 0.5 + standoff)
                )
                yaw = math.atan2(
                    table_center[1] - goal_xy[1],
                    table_center[0] - goal_xy[0],
                )
                safe = self.goal_footprint_is_free(
                    goal_xy[0],
                    goal_xy[1],
                    yaw,
                )
                projection = self.evaluate_projection(
                    goal_xy,
                    yaw,
                    transform_base_camera,
                    corners,
                )
                valid = safe and projection['valid']
                path_distance = 0.0
                if robot_xy is not None:
                    path_distance = float(np.linalg.norm(goal_xy - robot_xy))
                candidates.append({
                    'x': float(goal_xy[0]),
                    'y': float(goal_xy[1]),
                    'yaw': float(yaw),
                    'standoff': float(standoff),
                    'safe': safe,
                    'valid': valid,
                    'inside': projection['inside'],
                    'centered': projection['centered'],
                    'area_ratio': projection['area_ratio'],
                    'center_error': projection['center_error'],
                    'path_distance': path_distance,
                })
        self.candidates = candidates
        valid_candidates = [item for item in candidates if item['valid']]
        if not valid_candidates:
            self.best_goal = None
            safe_count = sum(item['safe'] for item in candidates)
            inside_count = sum(
                item['safe'] and item['inside']
                for item in candidates
            )
            centered_count = sum(
                item['safe'] and item['inside'] and item['centered']
                for item in candidates
            )
            self.set_plan_status(
                'no valid viewpoint: '
                f'total={len(candidates)}, safe={safe_count}, '
                f'full_frame={inside_count}, centered={centered_count}'
            )
            self.publish_markers()
            return False
        best = min(
            valid_candidates,
            key=lambda item: (
                -item['area_ratio'],
                item['center_error'],
                item['path_distance'],
            ),
        )
        self.best_goal = self.make_goal(best)
        self.goal_pub.publish(self.best_goal)
        self.set_plan_status(
            f'goal ready: standoff={best["standoff"]:.2f} m, '
            f'tabletop_area={best["area_ratio"] * 100.0:.1f}%'
        )
        self.publish_markers()
        self.try_auto_send_goal()
        return True

    def evaluate_projection(
        self,
        goal_xy,
        yaw,
        transform_base_camera,
        corners,
    ):
        map_to_base = pose_matrix(goal_xy[0], goal_xy[1], 0.0, yaw)
        map_to_camera = map_to_base @ transform_base_camera
        camera_to_map = np.linalg.inv(map_to_camera)
        homogeneous = np.column_stack((corners, np.ones(corners.shape[0])))
        camera_points = (camera_to_map @ homogeneous.T).T[:, :3]
        if np.any(camera_points[:, 2] <= 0.05):
            return {
                'valid': False,
                'inside': False,
                'centered': False,
                'area_ratio': 0.0,
                'center_error': 1.0,
            }
        projection = self.camera_info.p
        fx = projection[0] if projection[0] else self.camera_info.k[0]
        fy = projection[5] if projection[5] else self.camera_info.k[4]
        cx = projection[2] if projection[2] else self.camera_info.k[2]
        cy = projection[6] if projection[6] else self.camera_info.k[5]
        u = fx * camera_points[:, 0] / camera_points[:, 2] + cx
        v = fy * camera_points[:, 1] / camera_points[:, 2] + cy
        width = float(self.camera_info.width)
        height = float(self.camera_info.height)
        margin_u = width * self.image_margin_ratio
        margin_v = height * self.image_margin_ratio
        inside = (
            float(np.min(u)) >= margin_u and
            float(np.max(u)) <= width - margin_u and
            float(np.min(v)) >= margin_v and
            float(np.max(v)) <= height - margin_v
        )
        polygon_area = 0.5 * abs(
            float(np.dot(u, np.roll(v, -1))) -
            float(np.dot(v, np.roll(u, -1)))
        )
        area_ratio = polygon_area / (width * height)
        center_u = float(np.mean(u))
        center_error = abs(center_u - cx) / width
        centered = center_error <= self.center_error_max
        valid = inside and centered
        return {
            'valid': valid,
            'inside': inside,
            'centered': centered,
            'area_ratio': area_ratio,
            'center_error': center_error,
        }

    def tabletop_corners(self):
        center, box_height = self.observation_box_geometry()
        yaw = float(self.table['yaw'])
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        rotation = np.array([
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ])
        corners = []
        table_top_z = box_height * 0.5
        for local_x, local_y in (
            (-self.table_length * 0.5, -self.table_width * 0.5),
            (self.table_length * 0.5, -self.table_width * 0.5),
            (self.table_length * 0.5, self.table_width * 0.5),
            (-self.table_length * 0.5, self.table_width * 0.5),
        ):
            local = np.array([local_x, local_y, table_top_z])
            corners.append(center + rotation @ local)
        return np.asarray(corners)

    def observation_box_geometry(self):
        center = np.array(self.table['center'], dtype=float)
        if not self.observe_tabletop_only:
            return center, self.table_height
        thickness = min(
            max(self.tabletop_thickness, 0.01),
            self.table_height,
        )
        table_top_z = center[2] + self.table_height * 0.5
        center[2] = table_top_z - thickness * 0.5
        return center, thickness

    def robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None
        return np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
        ])

    def goal_footprint_is_free(self, x, y, yaw):
        if self.global_costmap is None:
            return False
        if not self.footprint_is_free_in_costmap(
            self.global_costmap,
            x,
            y,
            yaw,
            reject_outside=True,
        ):
            return False
        if (
            self.local_costmap is not None and
            self.world_to_costmap(x, y, self.local_costmap) is not None
        ):
            return self.footprint_is_free_in_costmap(
                self.local_costmap,
                x,
                y,
                yaw,
                reject_outside=False,
            )
        return True

    def footprint_is_free_in_costmap(
        self,
        costmap,
        x,
        y,
        yaw,
        reject_outside,
    ):
        resolution = costmap.info.resolution
        half_length = self.robot_length * 0.5 + self.footprint_margin
        half_width = self.robot_width * 0.5 + self.footprint_margin
        sample_step = max(resolution, 0.03)
        local_x = np.arange(
            -half_length,
            half_length + sample_step * 0.5,
            sample_step,
        )
        local_y = np.arange(
            -half_width,
            half_width + sample_step * 0.5,
            sample_step,
        )
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for offset_x in local_x:
            for offset_y in local_y:
                world_x = x + cosine * offset_x - sine * offset_y
                world_y = y + sine * offset_x + cosine * offset_y
                cell = self.world_to_costmap(world_x, world_y, costmap)
                if cell is None:
                    if reject_outside:
                        return False
                    continue
                grid_x, grid_y = cell
                index = grid_y * costmap.info.width + grid_x
                cost = costmap.data[index]
                if cost < 0 and self.reject_unknown:
                    return False
                if cost >= self.occupied_threshold:
                    return False
        return True

    def world_to_costmap(self, x, y, costmap):
        origin = costmap.info.origin
        origin_yaw = self.yaw_from_quaternion(origin.orientation)
        dx = x - origin.position.x
        dy = y - origin.position.y
        cosine = math.cos(origin_yaw)
        sine = math.sin(origin_yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        grid_x = int(math.floor(local_x / costmap.info.resolution))
        grid_y = int(math.floor(local_y / costmap.info.resolution))
        if (
            grid_x < 0 or grid_y < 0 or
            grid_x >= costmap.info.width or
            grid_y >= costmap.info.height
        ):
            return None
        return grid_x, grid_y

    @staticmethod
    def yaw_from_quaternion(quaternion):
        siny = 2.0 * (
            quaternion.w * quaternion.z +
            quaternion.x * quaternion.y
        )
        cosy = 1.0 - 2.0 * (
            quaternion.y * quaternion.y +
            quaternion.z * quaternion.z
        )
        return math.atan2(siny, cosy)

    def make_goal(self, candidate):
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = candidate['x']
        goal.pose.position.y = candidate['y']
        quaternion = yaw_quaternion(candidate['yaw'])
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        return goal

    def save_service(self, request, response):
        del request
        if self.table is None:
            response.success = False
            response.message = 'No detected table calibration to save.'
            return response
        if not self.database_file:
            response.success = False
            response.message = 'database_file parameter is empty.'
            return response
        try:
            database = self.read_database()
            database.setdefault('tables', {})[self.table_name] = deepcopy(
                self.table
            )
            directory = os.path.dirname(self.database_file)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self.database_file, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(
                    database,
                    stream,
                    allow_unicode=True,
                    sort_keys=False,
                )
        except (OSError, yaml.YAMLError) as exc:
            response.success = False
            response.message = f'Failed to save table calibration: {exc}'
            return response
        response.success = True
        response.message = f'Saved table to {self.database_file}'
        self.get_logger().info(response.message)
        return response

    def load_table_database(self):
        database = self.read_database()
        entry = database.get('tables', {}).get(self.table_name)
        if not entry:
            return
        try:
            center = [float(value) for value in entry['center']]
            size = [float(value) for value in entry['size']]
            yaw = float(entry['yaw'])
            if len(center) != 3 or len(size) != 3:
                raise ValueError('center and size must have three values')
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(f'Invalid saved table calibration: {exc}')
            return
        self.table = {
            'name': self.table_name,
            'frame_id': self.map_frame,
            'center': center,
            'yaw': yaw,
            'size': size,
        }
        self.table_length, self.table_width, self.table_height = size
        self.get_logger().info(
            f'Loaded {self.table_name} from {self.database_file}'
        )

    def read_database(self):
        if not self.database_file or not os.path.exists(self.database_file):
            return {
                'map_name': 'Test052601_table_viewpoint',
                'tables': {},
            }
        try:
            with open(self.database_file, encoding='utf-8') as stream:
                database = yaml.safe_load(stream) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warn(f'Cannot read table database: {exc}')
            return {'tables': {}}
        if not isinstance(database, dict):
            return {'tables': {}}
        return database

    def clear_service(self, request, response):
        del request
        self.table = None
        self.best_goal = None
        self.candidates = []
        self.search_seed = None
        self.auto_goal_sent = False
        self.goal_send_pending = False
        self.nav_wait_reported = False
        self.publish_markers(clear=True)
        response.success = True
        response.message = (
            'Cleared runtime table calibration; saved database is unchanged.'
        )
        return response

    def send_goal_service(self, request, response):
        del request
        if self.best_goal is None:
            response.success = False
            response.message = (
                'No valid table viewpoint exists. Check the latest '
                '"Viewpoint status" log and the green candidate markers.'
            )
            return response
        if not self.nav_is_active:
            response.success = False
            response.message = (
                'Nav2 is not active yet; automatic sending will retry.'
            )
            return response
        response.success = self.send_current_goal()
        response.message = (
            'Submitted table viewpoint goal; check the node log for '
            'acceptance.'
            if response.success else
            'A table viewpoint goal request is already pending.'
        )
        return response

    def send_current_goal(self):
        if self.best_goal is None or self.goal_send_pending:
            return False
        if not self.nav_client.server_is_ready():
            return False
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(self.best_goal)
        try:
            future = self.nav_client.send_goal_async(goal)
        except Exception as exc:  # rclpy may reject a disappearing server
            self.get_logger().warn(f'Cannot submit Nav2 goal: {exc}')
            return False
        self.goal_send_pending = True
        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(
            'Submitted table viewpoint goal; waiting for Nav2 acceptance.'
        )
        return True

    def goal_response_callback(self, future):
        self.goal_send_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.auto_goal_sent = False
            self.get_logger().warn(
                f'Nav2 goal request failed; will retry automatically: {exc}'
            )
            return
        if not goal_handle.accepted:
            self.auto_goal_sent = False
            self.get_logger().warn(
                'Nav2 rejected the table viewpoint goal; will retry when '
                'bt_navigator is active.'
            )
            return
        self.goal_handle = goal_handle
        self.auto_goal_sent = True
        self.get_logger().info('Nav2 accepted the table viewpoint goal.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        self.goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().warn(f'Cannot read Nav2 goal result: {exc}')
            return
        status_names = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }
        status_name = status_names.get(status, f'finished with status {status}')
        log = self.get_logger().info
        if status != GoalStatus.STATUS_SUCCEEDED:
            log = self.get_logger().warn
        log(f'Table viewpoint navigation {status_name}.')

    def try_auto_send_goal(self):
        if (
            not self.auto_send_goal or
            self.best_goal is None or
            self.auto_goal_sent or
            self.goal_send_pending or
            self.nav_state_pending
        ):
            return
        if not self.nav_state_client.service_is_ready():
            if not self.nav_wait_reported:
                self.get_logger().info(
                    'Waiting for bt_navigator lifecycle service before '
                    'sending the table viewpoint goal.'
                )
                self.nav_wait_reported = True
            return
        self.nav_state_pending = True
        future = self.nav_state_client.call_async(GetState.Request())
        future.add_done_callback(self.nav_state_response_callback)

    def nav_state_response_callback(self, future):
        self.nav_state_pending = False
        try:
            state = future.result().current_state
        except Exception as exc:
            self.nav_is_active = False
            self.get_logger().warn(f'Cannot read bt_navigator state: {exc}')
            return
        self.nav_is_active = state.id == State.PRIMARY_STATE_ACTIVE
        if not self.nav_is_active:
            if not self.nav_wait_reported:
                self.get_logger().info(
                    f'Waiting for bt_navigator to become active '
                    f'(currently {state.label}).'
                )
                self.nav_wait_reported = True
            return
        self.nav_wait_reported = False
        if self.send_current_goal():
            self.get_logger().info(
                'bt_navigator is active; automatic goal submission started.'
            )

    def refresh_callback(self):
        if self.table is not None and self.best_goal is None:
            self.plan_viewpoint()
        self.try_auto_send_goal()
        self.publish_markers()

    def set_plan_status(self, status):
        if status == self.last_plan_status:
            return
        self.last_plan_status = status
        self.get_logger().info(f'Viewpoint status: {status}')

    def publish_markers(self, clear=False):
        markers_enabled = (
            self.publish_table_marker or
            self.publish_goal_marker or
            self.publish_candidate_markers
        )
        if not markers_enabled and not clear:
            if self.marker_clear_published:
                return
            clear = True
        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.frame_id = self.map_frame
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)
        if clear:
            self.marker_pub.publish(markers)
            self.marker_clear_published = True
            return
        marker_id = 0
        if self.publish_candidate_markers and self.search_seed is not None:
            marker = Marker()
            marker.header = delete_marker.header
            marker.ns = 'table_search'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(self.search_seed[0])
            marker.pose.position.y = float(self.search_seed[1])
            marker.pose.position.z = 0.01
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.search_radius * 2.0
            marker.scale.y = self.search_radius * 2.0
            marker.scale.z = 0.02
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.0
            marker.color.a = 0.18
            markers.markers.append(marker)
        if self.publish_table_marker and self.table is not None:
            marker_id = self.add_table_markers(markers, marker_id)
        if self.publish_candidate_markers:
            for candidate in self.candidates:
                marker = Marker()
                marker.header = delete_marker.header
                marker.ns = 'table_candidates'
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = candidate['x']
                marker.pose.position.y = candidate['y']
                marker.pose.position.z = 0.08
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.10
                marker.scale.y = 0.10
                marker.scale.z = 0.10
                if candidate['valid']:
                    marker.color.g = 1.0
                elif candidate['safe']:
                    marker.color.r = 1.0
                    marker.color.g = 0.75
                else:
                    marker.color.r = 1.0
                marker.color.a = 0.75
                markers.markers.append(marker)
        if self.publish_goal_marker and self.best_goal is not None:
            marker = Marker()
            marker.header = delete_marker.header
            marker.ns = 'table_best_goal'
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = self.best_goal.pose
            marker.pose.position.z = 0.12
            marker.scale.x = 0.55
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 0.0
            marker.color.g = 0.45
            marker.color.b = 1.0
            marker.color.a = 1.0
            markers.markers.append(marker)
        self.marker_pub.publish(markers)
        self.marker_clear_published = False

    def add_table_markers(self, markers, marker_id):
        center, box_height = self.observation_box_geometry()
        yaw = float(self.table['yaw'])
        quaternion = yaw_quaternion(yaw)
        box = Marker()
        box.header.frame_id = self.map_frame
        box.header.stamp = self.get_clock().now().to_msg()
        box.ns = 'table_box'
        box.id = marker_id
        marker_id += 1
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = center[0]
        box.pose.position.y = center[1]
        box.pose.position.z = center[2]
        box.pose.orientation.x = quaternion[0]
        box.pose.orientation.y = quaternion[1]
        box.pose.orientation.z = quaternion[2]
        box.pose.orientation.w = quaternion[3]
        box.scale.x = self.table_length
        box.scale.y = self.table_width
        box.scale.z = box_height
        box.color.r = 0.1
        box.color.g = 0.65
        box.color.b = 1.0
        box.color.a = 0.30
        markers.markers.append(box)

        footprint = Marker()
        footprint.header = box.header
        footprint.ns = 'table_footprint'
        footprint.id = marker_id
        marker_id += 1
        footprint.type = Marker.LINE_STRIP
        footprint.action = Marker.ADD
        footprint.scale.x = 0.05
        footprint.color.r = 0.0
        footprint.color.g = 0.8
        footprint.color.b = 1.0
        footprint.color.a = 1.0
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_corners = [
            (self.table_length * 0.5, self.table_width * 0.5),
            (self.table_length * 0.5, -self.table_width * 0.5),
            (-self.table_length * 0.5, -self.table_width * 0.5),
            (-self.table_length * 0.5, self.table_width * 0.5),
            (self.table_length * 0.5, self.table_width * 0.5),
        ]
        for local_x, local_y in local_corners:
            point = Point()
            point.x = center[0] + cosine * local_x - sine * local_y
            point.y = center[1] + sine * local_x + cosine * local_y
            point.z = center[2] + box_height * 0.5 + 0.01
            footprint.points.append(point)
        markers.markers.append(footprint)
        return marker_id


def main(args=None):
    rclpy.init(args=args)
    node = TableViewpointPlanner()
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
