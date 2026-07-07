import math
import os
from copy import deepcopy

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from action_msgs.msg import GoalStatus
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D
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


def normalize_axis_yaw(yaw):
    return 0.5 * math.atan2(math.sin(2.0 * yaw), math.cos(2.0 * yaw))


def pose_to_matrix(pose):
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_matrix(pose.orientation)
    matrix[0, 3] = pose.position.x
    matrix[1, 3] = pose.position.y
    matrix[2, 3] = pose.position.z
    return matrix


class TableViewpointPlanner(Node):
    def __init__(self):
        super().__init__('table_viewpoint_planner')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_frame', 'map'),
                ('base_frame', 'base_footprint'),
                ('camera_info_topic', '/camera/color/camera_info'),
                ('input_mode', 'topic'),
                ('bbox_topic', '/target_bbox_3d'),
                ('global_costmap_topic', '/global_costmap/costmap'),
                ('local_costmap_topic', '/local_costmap/costmap'),
                ('database_file', ''),
                ('table_name', 'office_desk_1'),
                ('observe_tabletop_only', True),
                ('tabletop_thickness_m', 0.05),
                ('robot_length_m', 0.80),
                ('robot_width_m', 0.70),
                ('footprint_margin_m', 0.05),
                ('occupied_cost_threshold', 50),
                ('reject_unknown_cost', True),
                ('min_standoff_m', 0.60),
                ('max_standoff_m', 3.00),
                ('standoff_step_m', 0.10),
                ('image_margin_ratio', 0.05),
                ('area_weight', 0.70),
                ('horizontal_center_weight', 0.10),
                ('vertical_center_weight', 0.08),
                ('clearance_weight', 0.10),
                ('travel_weight', 0.02),
                ('target_area_ratio', 0.60),
                ('horizontal_error_scale', 0.10),
                ('vertical_error_scale', 0.20),
                ('travel_distance_scale_m', 8.0),
                ('max_bbox_tilt_deg', 10.0),
                ('bbox_position_tolerance_m', 0.03),
                ('bbox_size_tolerance_m', 0.03),
                ('bbox_yaw_tolerance_rad', 0.03),
                ('repeat_bbox_retriggers_goal', True),
                ('repeat_bbox_retrigger_distance_m', 0.35),
                ('approach_slowdown_distance_m', 1.50),
                ('approach_speed_limit_mps', 0.15),
                ('speed_limit_topic', '/speed_limit'),
                ('auto_send_goal', True),
                ('publish_table_marker', True),
                ('publish_goal_marker', True),
                ('publish_candidate_markers', False),
            ],
        )

        self.map_frame = self.parameter('map_frame')
        self.base_frame = self.parameter('base_frame')
        self.input_mode = str(self.parameter('input_mode')).strip().lower()
        if self.input_mode not in ('topic', 'yaml'):
            self.get_logger().warn(
                f'Unknown input_mode={self.input_mode!r}; using topic.'
            )
            self.input_mode = 'topic'
        self.database_file = os.path.expanduser(
            self.parameter('database_file')
        )
        self.table_name = self.parameter('table_name')
        self.table_length = 0.0
        self.table_width = 0.0
        self.table_height = 0.0
        self.observe_tabletop_only = bool(
            self.parameter('observe_tabletop_only')
        )
        self.tabletop_thickness = float(
            self.parameter('tabletop_thickness_m')
        )
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
        weights = np.array([
            self.parameter('area_weight'),
            self.parameter('horizontal_center_weight'),
            self.parameter('vertical_center_weight'),
            self.parameter('clearance_weight'),
            self.parameter('travel_weight'),
        ], dtype=float)
        if not np.all(np.isfinite(weights)) or np.any(weights < 0.0):
            raise ValueError('Viewpoint score weights must be finite and >= 0')
        weight_sum = float(np.sum(weights))
        if weight_sum <= 1e-9:
            raise ValueError('At least one viewpoint score weight must be > 0')
        (
            self.area_weight,
            self.horizontal_center_weight,
            self.vertical_center_weight,
            self.clearance_weight,
            self.travel_weight,
        ) = weights / weight_sum
        self.target_area_ratio = max(
            float(self.parameter('target_area_ratio')),
            1e-6,
        )
        self.horizontal_error_scale = max(
            float(self.parameter('horizontal_error_scale')),
            1e-6,
        )
        self.vertical_error_scale = max(
            float(self.parameter('vertical_error_scale')),
            1e-6,
        )
        self.travel_distance_scale = max(
            float(self.parameter('travel_distance_scale_m')),
            1e-6,
        )
        self.max_bbox_tilt = math.radians(
            float(self.parameter('max_bbox_tilt_deg'))
        )
        self.bbox_position_tolerance = float(
            self.parameter('bbox_position_tolerance_m')
        )
        self.bbox_size_tolerance = float(
            self.parameter('bbox_size_tolerance_m')
        )
        self.bbox_yaw_tolerance = float(
            self.parameter('bbox_yaw_tolerance_rad')
        )
        self.repeat_bbox_retriggers_goal = bool(
            self.parameter('repeat_bbox_retriggers_goal')
        )
        self.repeat_bbox_retrigger_distance = max(
            float(self.parameter('repeat_bbox_retrigger_distance_m')),
            0.0,
        )
        self.approach_slowdown_distance = max(
            float(self.parameter('approach_slowdown_distance_m')),
            0.0,
        )
        self.approach_speed_limit = max(
            float(self.parameter('approach_speed_limit_mps')),
            0.0,
        )
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )
        self.nav_state_client = self.create_client(
            GetState,
            '/bt_navigator/get_state',
        )

        self.camera_info = None
        self.global_costmap = None
        self.local_costmap = None
        self.table = None
        self.best_goal = None
        self.candidates = []
        self.last_plan_status = None
        self.auto_goal_sent = False
        self.goal_send_pending = False
        self.nav_state_pending = False
        self.nav_is_active = False
        self.nav_wait_reported = False
        self.goal_handle = None
        self.active_goal_pose = None
        self.approach_speed_limited = False
        self.marker_clear_published = False
        self.table_version = 0
        self.goal_table_version = None

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
        self.speed_limit_pub = self.create_publisher(
            SpeedLimit,
            self.parameter('speed_limit_topic'),
            10,
        )

        self.create_subscription(
            CameraInfo,
            self.parameter('camera_info_topic'),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Detection3D,
            self.parameter('bbox_topic'),
            self.bbox_callback,
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
        self.create_timer(0.10, self.update_approach_speed_limit)

        if self.input_mode == 'yaml':
            self.load_table_database()
        self.get_logger().info(
            f'Table viewpoint input mode: {self.input_mode}. '
            'topic uses Detection3D and yaml loads the saved table.'
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

    def bbox_callback(self, msg):
        if self.input_mode != 'topic':
            return
        size = np.array([
            msg.bbox.size.x,
            msg.bbox.size.y,
            msg.bbox.size.z,
        ], dtype=float)
        if not np.all(np.isfinite(size)) or np.any(size <= 0.01):
            self.get_logger().warn(
                'Ignoring Detection3D with non-positive or invalid bbox size.'
            )
            return
        frame_id = msg.header.frame_id
        if not frame_id:
            self.get_logger().warn(
                'Ignoring Detection3D because header.frame_id is empty.'
            )
            return
        source_to_box = pose_to_matrix(msg.bbox.center)
        if frame_id == self.map_frame:
            map_to_box = source_to_box
        else:
            stamp = Time.from_msg(msg.header.stamp)
            if stamp.nanoseconds == 0:
                stamp = Time()
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    frame_id,
                    stamp,
                    timeout=Duration(seconds=0.2),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f'Cannot transform Detection3D bbox to map: {exc}'
                )
                return
            map_to_box = (
                transform_to_matrix(transform.transform) @ source_to_box
            )
        rotation = map_to_box[:3, :3]
        upright_alignment = abs(float(rotation[2, 2]))
        minimum_alignment = math.cos(self.max_bbox_tilt)
        if upright_alignment < minimum_alignment:
            tilt = math.degrees(math.acos(min(upright_alignment, 1.0)))
            self.get_logger().warn(
                f'Ignoring tilted bbox ({tilt:.1f} deg); expected a '
                'near-horizontal tabletop.'
            )
            return
        center = map_to_box[:3, 3]
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        self.update_runtime_table(
            center=center,
            yaw=yaw,
            size=size,
            source=f'Detection3D {msg.id or "without id"}',
        )

    def update_runtime_table(self, center, yaw, size, source):
        center = np.asarray(center, dtype=float)
        size = np.asarray(size, dtype=float)
        if (
            center.shape != (3,) or size.shape != (3,) or
            not np.all(np.isfinite(center)) or
            not np.all(np.isfinite(size)) or
            np.any(size <= 0.01)
        ):
            self.get_logger().warn(f'Ignoring invalid table from {source}.')
            return False
        length, width, height = size
        if width > length:
            length, width = width, length
            yaw += math.pi * 0.5
        yaw = normalize_axis_yaw(yaw)
        normalized_size = np.array([length, width, height], dtype=float)
        if self.table is not None:
            old_center = np.asarray(self.table['center'], dtype=float)
            old_size = np.asarray(self.table['size'], dtype=float)
            yaw_error = abs(normalize_axis_yaw(yaw - self.table['yaw']))
            if (
                np.linalg.norm(center - old_center) <=
                self.bbox_position_tolerance and
                np.max(np.abs(normalized_size - old_size)) <=
                self.bbox_size_tolerance and
                yaw_error <= self.bbox_yaw_tolerance
            ):
                self.retrigger_repeated_bbox(source)
                return False
        self.table_length = float(length)
        self.table_width = float(width)
        self.table_height = float(height)
        self.table = {
            'name': self.table_name,
            'frame_id': self.map_frame,
            'center': [float(value) for value in center],
            'yaw': float(yaw),
            'size': [float(value) for value in normalized_size],
        }
        self.table_version += 1
        self.best_goal = None
        self.candidates = []
        self.last_plan_status = None
        if self.goal_handle is None:
            self.auto_goal_sent = False
        self.get_logger().info(
            f'Accepted {source}: center=({center[0]:.2f}, '
            f'{center[1]:.2f}, {center[2]:.2f}), '
            f'size={length:.2f}x{width:.2f}x{height:.2f} m, '
            f'yaw={yaw:.2f}.'
        )
        if self.camera_info is not None and self.global_costmap is not None:
            self.plan_viewpoint()
        else:
            self.publish_markers()
        return True

    def retrigger_repeated_bbox(self, source):
        if (
            not self.repeat_bbox_retriggers_goal or
            not self.auto_send_goal or
            self.goal_send_pending or
            self.goal_handle is not None
        ):
            return False
        if self.best_goal is None:
            self.auto_goal_sent = False
            return self.plan_viewpoint()
        robot_xy = self.robot_position()
        if robot_xy is None:
            self.get_logger().warn(
                f'Cannot retrigger repeated {source}: robot TF unavailable.'
            )
            return False
        goal_xy = np.array([
            self.best_goal.pose.position.x,
            self.best_goal.pose.position.y,
        ])
        distance = float(np.linalg.norm(robot_xy - goal_xy))
        if distance <= self.repeat_bbox_retrigger_distance:
            return False
        self.get_logger().info(
            f'Received repeated {source} while robot is {distance:.2f} m '
            'from the previous viewpoint; starting a new task.'
        )
        self.auto_goal_sent = False
        return self.plan_viewpoint()

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
                safe, clearance_score, max_cost = (
                    self.goal_footprint_evaluation(
                        goal_xy[0],
                        goal_xy[1],
                        yaw,
                    )
                )
                projection = self.evaluate_projection(
                    goal_xy,
                    yaw,
                    transform_base_camera,
                    corners,
                )
                valid = safe and projection['inside']
                path_distance = None
                if robot_xy is not None:
                    path_distance = float(np.linalg.norm(goal_xy - robot_xy))
                candidate = {
                    'x': float(goal_xy[0]),
                    'y': float(goal_xy[1]),
                    'yaw': float(yaw),
                    'standoff': float(standoff),
                    'safe': safe,
                    'valid': valid,
                    'inside': projection['inside'],
                    'area_ratio': projection['area_ratio'],
                    'horizontal_error': projection['horizontal_error'],
                    'vertical_error': projection['vertical_error'],
                    'clearance_score': clearance_score,
                    'max_cost': max_cost,
                    'path_distance': path_distance,
                }
                candidates.append(candidate)
        self.candidates = candidates
        valid_candidates = [item for item in candidates if item['valid']]
        if not valid_candidates:
            self.best_goal = None
            safe_count = sum(item['safe'] for item in candidates)
            inside_count = sum(
                item['safe'] and item['inside']
                for item in candidates
            )
            self.set_plan_status(
                'no valid viewpoint: '
                f'total={len(candidates)}, safe={safe_count}, '
                f'full_frame={inside_count}'
            )
            self.publish_markers()
            return False
        maximum_area_ratio = max(
            item['area_ratio'] for item in valid_candidates
        )
        area_reference = max(
            min(self.target_area_ratio, maximum_area_ratio),
            1e-6,
        )
        for candidate in valid_candidates:
            candidate.update(
                self.score_candidate(candidate, area_reference)
            )
        best = max(
            valid_candidates,
            key=lambda item: (
                item['score'],
                item['area_score'],
                item['clearance_score'],
            ),
        )
        self.best_goal = self.make_goal(best)
        self.goal_pub.publish(self.best_goal)
        self.set_plan_status(
            f'goal ready: score={best["score"]:.1f}/100, '
            f'fill={best["area_ratio"] * 100.0:.2f}%, '
            f'area_score={best["area_score"]:.2f}, '
            f'h_center={best["horizontal_center_score"]:.2f}, '
            f'v_center={best["vertical_center_score"]:.2f}, '
            f'clearance={best["clearance_score"]:.2f}, '
            f'travel={best["travel_score"]:.2f}, '
            f'standoff={best["standoff"]:.2f} m'
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
                'inside': False,
                'area_ratio': 0.0,
                'horizontal_error': 1.0,
                'vertical_error': 1.0,
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
        center_v = float(np.mean(v))
        horizontal_error = abs(center_u - cx) / width
        vertical_error = abs(center_v - cy) / height
        return {
            'inside': inside,
            'area_ratio': area_ratio,
            'horizontal_error': horizontal_error,
            'vertical_error': vertical_error,
        }

    def score_candidate(self, candidate, area_reference=None):
        if area_reference is None:
            area_reference = self.target_area_ratio
        area_score = min(
            max(candidate['area_ratio'] / area_reference, 0.0),
            1.0,
        )
        horizontal_center_score = 1.0 - min(
            candidate['horizontal_error'] / self.horizontal_error_scale,
            1.0,
        )
        vertical_center_score = 1.0 - min(
            candidate['vertical_error'] / self.vertical_error_scale,
            1.0,
        )
        if candidate['path_distance'] is None:
            travel_score = 0.0
        else:
            travel_score = 1.0 - min(
                candidate['path_distance'] / self.travel_distance_scale,
                1.0,
            )
        score = 100.0 * (
            self.area_weight * area_score +
            self.horizontal_center_weight * horizontal_center_score +
            self.vertical_center_weight * vertical_center_score +
            self.clearance_weight * candidate['clearance_score'] +
            self.travel_weight * travel_score
        )
        return {
            'score': float(score),
            'area_score': float(area_score),
            'horizontal_center_score': float(horizontal_center_score),
            'vertical_center_score': float(vertical_center_score),
            'travel_score': float(travel_score),
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

    def goal_footprint_evaluation(self, x, y, yaw):
        if self.global_costmap is None:
            return False, 0.0, self.occupied_threshold
        global_safe, global_max_cost = self.footprint_cost_in_costmap(
            self.global_costmap,
            x,
            y,
            yaw,
            reject_outside=True,
        )
        if not global_safe:
            return False, 0.0, global_max_cost
        max_cost = global_max_cost
        if (
            self.local_costmap is not None and
            self.world_to_costmap(x, y, self.local_costmap) is not None
        ):
            local_safe, local_max_cost = self.footprint_cost_in_costmap(
                self.local_costmap,
                x,
                y,
                yaw,
                reject_outside=False,
            )
            if not local_safe:
                return False, 0.0, local_max_cost
            max_cost = max(max_cost, local_max_cost)
        clearance_score = 1.0 - min(
            max(max_cost, 0) / max(self.occupied_threshold, 1),
            1.0,
        )
        return True, float(clearance_score), int(max_cost)

    def footprint_cost_in_costmap(
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
        max_cost = 0
        for offset_x in local_x:
            for offset_y in local_y:
                world_x = x + cosine * offset_x - sine * offset_y
                world_y = y + sine * offset_x + cosine * offset_y
                cell = self.world_to_costmap(world_x, world_y, costmap)
                if cell is None:
                    if reject_outside:
                        return False, self.occupied_threshold
                    continue
                grid_x, grid_y = cell
                index = grid_y * costmap.info.width + grid_x
                cost = costmap.data[index]
                if cost < 0 and self.reject_unknown:
                    return False, self.occupied_threshold
                if cost >= self.occupied_threshold:
                    return False, cost
                if cost >= 0:
                    max_cost = max(max_cost, cost)
        return True, max_cost

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
        self.update_runtime_table(
            center=center,
            yaw=yaw,
            size=size,
            source=f'saved table {self.table_name}',
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
        self.table_version += 1
        self.table = None
        self.best_goal = None
        self.candidates = []
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
        response.success = self.send_current_goal()
        response.message = (
            'Submitted table viewpoint goal; check the node log for '
            'acceptance.'
            if response.success else
            'Nav2 action server is unavailable, or a goal is already active.'
        )
        return response

    def send_current_goal(self):
        if (
            self.best_goal is None or
            self.goal_send_pending or
            self.goal_handle is not None
        ):
            return False
        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                'Cannot send viewpoint: /navigate_to_pose is not ready.'
            )
            return False
        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(self.best_goal)
        try:
            future = self.nav_client.send_goal_async(goal)
        except Exception as exc:  # rclpy may reject a disappearing server
            self.get_logger().warn(f'Cannot submit Nav2 goal: {exc}')
            return False
        self.goal_send_pending = True
        self.active_goal_pose = deepcopy(goal.pose)
        self.goal_table_version = self.table_version
        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(
            'Submitted table viewpoint goal '
            f'x={goal.pose.pose.position.x:.2f}, '
            f'y={goal.pose.pose.position.y:.2f}; '
            'waiting for Nav2 acceptance.'
        )
        return True

    def goal_response_callback(self, future):
        self.goal_send_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.auto_goal_sent = False
            self.active_goal_pose = None
            self.goal_table_version = None
            self.get_logger().warn(
                f'Nav2 goal request failed; will retry automatically: {exc}'
            )
            return
        if not goal_handle.accepted:
            self.auto_goal_sent = False
            self.active_goal_pose = None
            self.goal_table_version = None
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
        self.active_goal_pose = None
        self.clear_approach_speed_limit()
        completed_table_version = self.goal_table_version
        self.goal_table_version = None
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().warn(f'Cannot read Nav2 goal result: {exc}')
            if completed_table_version != self.table_version:
                self.auto_goal_sent = False
                self.plan_viewpoint()
            return
        status_names = {
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }
        status_name = status_names.get(
            status,
            f'finished with status {status}',
        )
        log = self.get_logger().info
        if status != GoalStatus.STATUS_SUCCEEDED:
            log = self.get_logger().warn
        log(f'Table viewpoint navigation {status_name}.')
        if completed_table_version != self.table_version:
            self.auto_goal_sent = False
            self.get_logger().info(
                'A newer 3D bbox arrived during navigation; planning its '
                'viewpoint now.'
            )
            self.plan_viewpoint()

    def publish_speed_limit(self, speed_limit):
        message = SpeedLimit()
        message.header.stamp = self.get_clock().now().to_msg()
        message.percentage = False
        message.speed_limit = float(speed_limit)
        self.speed_limit_pub.publish(message)

    def clear_approach_speed_limit(self):
        if not self.approach_speed_limited:
            return
        self.publish_speed_limit(0.0)
        self.approach_speed_limited = False
        self.get_logger().info('Cleared table-approach speed limit.')

    def update_approach_speed_limit(self):
        if self.goal_handle is None or self.active_goal_pose is None:
            self.clear_approach_speed_limit()
            return
        if self.approach_speed_limited:
            return
        robot_xy = self.robot_position()
        if robot_xy is None:
            return
        goal_xy = np.array([
            self.active_goal_pose.pose.position.x,
            self.active_goal_pose.pose.position.y,
        ])
        distance = float(np.linalg.norm(robot_xy - goal_xy))
        if distance > self.approach_slowdown_distance:
            return
        self.publish_speed_limit(self.approach_speed_limit)
        self.approach_speed_limited = True
        self.get_logger().info(
            f'Within {distance:.2f} m of table viewpoint; limiting '
            f'linear speed to {self.approach_speed_limit:.2f} m/s.'
        )

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
