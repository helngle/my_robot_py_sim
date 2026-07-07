import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D
from visualization_msgs.msg import Marker


try:
    from segment_anything import SamPredictor, sam_model_registry
except ImportError:
    SamPredictor = None
    sam_model_registry = None


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_matrix(q):
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    norm = x * x + y * y + z * z + w * w
    if norm <= 1e-12:
        return np.eye(3)
    scale = 2.0 / norm
    xx = x * x * scale
    yy = y * y * scale
    zz = z * z * scale
    xy = x * y * scale
    xz = x * z * scale
    yz = y * z * scale
    wx = w * x * scale
    wy = w * y * scale
    wz = w * z * scale
    return np.array([
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ])


def transform_to_matrix(transform):
    rotation = quaternion_to_matrix(transform.transform.rotation)
    translation = transform.transform.translation
    return rotation, np.array([translation.x, translation.y, translation.z])


class SamTableBboxNode(Node):
    """Generate a Detection3D table box from a SAM mask and aligned depth.

    First experimental workflow:
      1. Keep the table in the RGB-D camera view.
      2. Click the table roughly in RViz with Publish Point.
      3. The clicked map point is projected into the camera image as a SAM
         positive point prompt.
      4. The resulting mask + aligned depth is converted to a 3D bbox and
         published on /target_bbox_3d.
    """

    def __init__(self):
        super().__init__('sam_table_bbox_node')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/color/camera_info',
        )
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('bbox_topic', '/target_bbox_3d')
        self.declare_parameter('debug_image_topic', '/sam_table_mask/debug')
        self.declare_parameter('marker_topic', '/sam_table_bbox_marker')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('sam_checkpoint', '')
        self.declare_parameter('sam_model_type', 'vit_b')
        self.declare_parameter('sam_device', 'cuda')
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop_s', 0.08)
        self.declare_parameter('min_depth_m', 0.25)
        self.declare_parameter('max_depth_m', 6.0)
        self.declare_parameter('min_mask_area_px', 500)
        self.declare_parameter('max_points_for_fit', 3000)
        self.declare_parameter('depth_percentile_low', 5.0)
        self.declare_parameter('depth_percentile_high', 95.0)
        self.declare_parameter('xy_percentile_low', 2.0)
        self.declare_parameter('xy_percentile_high', 98.0)
        self.declare_parameter('min_box_length_m', 0.20)
        self.declare_parameter('min_box_width_m', 0.20)
        self.declare_parameter('min_box_height_m', 0.05)
        self.declare_parameter('bbox_id', 'sam_table')
        self.declare_parameter('publish_debug_image', True)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.clicked_point_topic = self.get_parameter(
            'clicked_point_topic'
        ).value
        self.bbox_topic = self.get_parameter('bbox_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.sam_checkpoint = self.get_parameter('sam_checkpoint').value
        self.sam_model_type = self.get_parameter('sam_model_type').value
        self.sam_device = self.get_parameter('sam_device').value
        self.sync_queue_size = int(
            self.get_parameter('sync_queue_size').value
        )
        self.sync_slop_s = float(self.get_parameter('sync_slop_s').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.min_mask_area_px = int(
            self.get_parameter('min_mask_area_px').value
        )
        self.max_points_for_fit = int(
            self.get_parameter('max_points_for_fit').value
        )
        self.depth_percentile_low = float(
            self.get_parameter('depth_percentile_low').value
        )
        self.depth_percentile_high = float(
            self.get_parameter('depth_percentile_high').value
        )
        self.xy_percentile_low = float(
            self.get_parameter('xy_percentile_low').value
        )
        self.xy_percentile_high = float(
            self.get_parameter('xy_percentile_high').value
        )
        self.min_box_length_m = float(
            self.get_parameter('min_box_length_m').value
        )
        self.min_box_width_m = float(
            self.get_parameter('min_box_width_m').value
        )
        self.min_box_height_m = float(
            self.get_parameter('min_box_height_m').value
        )
        self.bbox_id = self.get_parameter('bbox_id').value
        self.publish_debug_image = bool(
            self.get_parameter('publish_debug_image').value
        )

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_color = None
        self.latest_depth = None
        self.latest_camera_info = None

        self.predictor = self.load_sam_predictor()

        self.bbox_pub = self.create_publisher(Detection3D, self.bbox_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.debug_image_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            qos_profile_sensor_data,
        )
        self.clicked_sub = self.create_subscription(
            PointStamped,
            self.clicked_point_topic,
            self.clicked_point_callback,
            10,
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

        self.get_logger().info(
            'SAM table bbox node listening on '
            f'{self.color_topic}, {self.depth_topic}, '
            f'{self.camera_info_topic}; click table on '
            f'{self.clicked_point_topic} to publish {self.bbox_topic}.'
        )

    def load_sam_predictor(self):
        if SamPredictor is None or sam_model_registry is None:
            self.get_logger().error(
                'segment_anything is not installed. Install SAM first, then '
                'run this node again. Existing table navigation is unaffected.'
            )
            return None
        if not self.sam_checkpoint:
            self.get_logger().error(
                'sam_checkpoint is empty. Set it to a SAM .pth checkpoint.'
            )
            return None
        if self.sam_model_type not in sam_model_registry:
            self.get_logger().error(
                f'Unknown SAM model type "{self.sam_model_type}". Available: '
                f'{sorted(sam_model_registry.keys())}'
            )
            return None

        model = sam_model_registry[self.sam_model_type](
            checkpoint=self.sam_checkpoint
        )
        model.to(device=self.sam_device)
        self.get_logger().info(
            f'Loaded SAM {self.sam_model_type} from {self.sam_checkpoint} '
            f'on {self.sam_device}.'
        )
        return SamPredictor(model)

    def rgbd_callback(self, color_msg, depth_msg, camera_info_msg):
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(
                color_msg,
                desired_encoding='bgr8',
            )
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough',
            )
        except CvBridgeError as exc:
            self.get_logger().warning(f'Failed to convert RGB-D image: {exc}')
            return

        self.latest_color = color_bgr
        self.latest_depth = depth
        self.latest_camera_info = camera_info_msg

    def clicked_point_callback(self, msg):
        if self.predictor is None:
            self.get_logger().warning(
                'SAM predictor is not ready; cannot segment table.'
            )
            return
        if (
            self.latest_color is None
            or self.latest_depth is None
            or self.latest_camera_info is None
        ):
            self.get_logger().warning(
                'No synchronized RGB-D frame yet; wait for camera frames.'
            )
            return

        pixel = self.project_clicked_point(msg, self.latest_camera_info)
        if pixel is None:
            return
        u, v = pixel

        color_rgb = cv2.cvtColor(self.latest_color, cv2.COLOR_BGR2RGB)
        self.predictor.set_image(color_rgb)
        masks, scores, _ = self.predictor.predict(
            point_coords=np.array([[u, v]], dtype=np.float32),
            point_labels=np.array([1], dtype=np.int32),
            multimask_output=True,
        )
        if masks is None or len(masks) == 0:
            self.get_logger().warning('SAM returned no mask.')
            return

        best_index = int(np.argmax(scores))
        mask = masks[best_index].astype(bool)
        mask_area = int(np.count_nonzero(mask))
        if mask_area < self.min_mask_area_px:
            self.get_logger().warning(
                f'SAM mask too small: {mask_area} px '
                f'< {self.min_mask_area_px} px.'
            )
            return

        bbox = self.fit_bbox_from_mask(mask, self.latest_camera_info)
        if bbox is None:
            return

        self.publish_bbox(bbox, self.latest_camera_info.header.stamp)
        self.publish_marker(bbox, self.latest_camera_info.header.stamp)
        if self.publish_debug_image:
            self.publish_mask_debug(mask, (u, v), scores[best_index])

    def project_clicked_point(self, point_msg, camera_info):
        camera_frame = camera_info.header.frame_id
        if not camera_frame:
            self.get_logger().warning('CameraInfo header.frame_id is empty.')
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                point_msg.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Cannot transform clicked point to {camera_frame}: {exc}'
            )
            return None

        rotation, translation = transform_to_matrix(transform)
        point = np.array([
            point_msg.point.x,
            point_msg.point.y,
            point_msg.point.z,
        ])
        camera_point = rotation @ point + translation
        z = camera_point[2]
        if z <= 0.0:
            self.get_logger().warning(
                'Clicked point projects behind the camera; click a visible '
                'point on the table.'
            )
            return None

        k = camera_info.k
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]
        u = int(round(fx * camera_point[0] / z + cx))
        v = int(round(fy * camera_point[1] / z + cy))
        height, width = self.latest_color.shape[:2]
        if u < 0 or u >= width or v < 0 or v >= height:
            self.get_logger().warning(
                f'Clicked point projects outside image: ({u}, {v}) not in '
                f'{width}x{height}.'
            )
            return None
        return u, v

    def fit_bbox_from_mask(self, mask, camera_info):
        points_camera = self.mask_to_camera_points(mask, camera_info)
        if points_camera is None or len(points_camera) < 30:
            self.get_logger().warning('Not enough valid depth points in mask.')
            return None

        camera_frame = camera_info.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Cannot transform mask points to {self.target_frame}: {exc}'
            )
            return None

        rotation, translation = transform_to_matrix(transform)
        points = (rotation @ points_camera.T).T + translation
        return self.fit_oriented_bbox(points)

    def mask_to_camera_points(self, mask, camera_info):
        depth = self.latest_depth
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        ys, xs = np.nonzero(mask)
        if len(xs) == 0:
            return None

        depth_values = depth[ys, xs].astype(np.float32)
        if self.latest_depth.dtype == np.uint16:
            depth_values *= 0.001

        valid = np.isfinite(depth_values)
        valid &= depth_values >= self.min_depth_m
        valid &= depth_values <= self.max_depth_m
        if np.count_nonzero(valid) < 30:
            return None

        xs = xs[valid]
        ys = ys[valid]
        depth_values = depth_values[valid]

        low = np.percentile(depth_values, self.depth_percentile_low)
        high = np.percentile(depth_values, self.depth_percentile_high)
        keep = (depth_values >= low) & (depth_values <= high)
        xs = xs[keep]
        ys = ys[keep]
        depth_values = depth_values[keep]

        if len(xs) > self.max_points_for_fit:
            indices = np.linspace(
                0,
                len(xs) - 1,
                self.max_points_for_fit,
                dtype=np.int32,
            )
            xs = xs[indices]
            ys = ys[indices]
            depth_values = depth_values[indices]

        k = camera_info.k
        fx = k[0]
        fy = k[4]
        cx = k[2]
        cy = k[5]
        x = (xs.astype(np.float32) - cx) * depth_values / fx
        y = (ys.astype(np.float32) - cy) * depth_values / fy
        z = depth_values
        return np.column_stack((x, y, z))

    def fit_oriented_bbox(self, points):
        z_low = np.percentile(points[:, 2], self.xy_percentile_low)
        z_high = np.percentile(points[:, 2], self.xy_percentile_high)
        keep_z = (points[:, 2] >= z_low) & (points[:, 2] <= z_high)
        points = points[keep_z]
        if len(points) < 30:
            return None

        xy = points[:, :2]
        xy_mean = np.mean(xy, axis=0)
        xy_centered = xy - xy_mean
        covariance = np.cov(xy_centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        major = eigenvectors[:, int(np.argmax(eigenvalues))]
        yaw = math.atan2(major[1], major[0])
        direction = np.array([math.cos(yaw), math.sin(yaw)])
        side = np.array([-math.sin(yaw), math.cos(yaw)])

        local_x = xy_centered @ direction
        local_y = xy_centered @ side
        x_min, x_max = np.percentile(
            local_x,
            [self.xy_percentile_low, self.xy_percentile_high],
        )
        y_min, y_max = np.percentile(
            local_y,
            [self.xy_percentile_low, self.xy_percentile_high],
        )
        length = max(float(x_max - x_min), self.min_box_length_m)
        width = max(float(y_max - y_min), self.min_box_width_m)
        if width > length:
            length, width = width, length
            yaw += math.pi * 0.5
            direction = np.array([math.cos(yaw), math.sin(yaw)])
            side = np.array([-math.sin(yaw), math.cos(yaw)])
            local_x, local_y = local_y, local_x
            x_min, x_max = y_min, y_max
            y_min, y_max = np.percentile(
                local_y,
                [self.xy_percentile_low, self.xy_percentile_high],
            )

        center_local_x = 0.5 * (x_min + x_max)
        center_local_y = 0.5 * (y_min + y_max)
        center_xy = (
            xy_mean
            + center_local_x * direction
            + center_local_y * side
        )
        z_min = float(np.percentile(points[:, 2], self.xy_percentile_low))
        z_max = float(np.percentile(points[:, 2], self.xy_percentile_high))
        height = max(z_max - z_min, self.min_box_height_m)
        center_z = z_min + 0.5 * height
        return {
            'center': (float(center_xy[0]), float(center_xy[1]), center_z),
            'size': (length, width, height),
            'yaw': float(math.atan2(math.sin(yaw), math.cos(yaw))),
            'points': len(points),
        }

    def publish_bbox(self, bbox, stamp):
        msg = Detection3D()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame
        msg.id = self.bbox_id
        msg.bbox.center.position.x = bbox['center'][0]
        msg.bbox.center.position.y = bbox['center'][1]
        msg.bbox.center.position.z = bbox['center'][2]
        qx, qy, qz, qw = yaw_to_quaternion(bbox['yaw'])
        msg.bbox.center.orientation.x = qx
        msg.bbox.center.orientation.y = qy
        msg.bbox.center.orientation.z = qz
        msg.bbox.center.orientation.w = qw
        msg.bbox.size.x = bbox['size'][0]
        msg.bbox.size.y = bbox['size'][1]
        msg.bbox.size.z = bbox['size'][2]
        self.bbox_pub.publish(msg)
        self.get_logger().info(
            'Published SAM table bbox: '
            f'center=({bbox["center"][0]:.3f}, {bbox["center"][1]:.3f}, '
            f'{bbox["center"][2]:.3f}), '
            f'size=({bbox["size"][0]:.3f}, {bbox["size"][1]:.3f}, '
            f'{bbox["size"][2]:.3f}), yaw={bbox["yaw"]:.3f}, '
            f'points={bbox["points"]}.'
        )

    def publish_marker(self, bbox, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.target_frame
        marker.ns = 'sam_table_bbox'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = bbox['center'][0]
        marker.pose.position.y = bbox['center'][1]
        marker.pose.position.z = bbox['center'][2]
        qx, qy, qz, qw = yaw_to_quaternion(bbox['yaw'])
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = bbox['size'][0]
        marker.scale.y = bbox['size'][1]
        marker.scale.z = bbox['size'][2]
        marker.color.r = 0.1
        marker.color.g = 0.55
        marker.color.b = 1.0
        marker.color.a = 0.35
        self.marker_pub.publish(marker)

    def publish_mask_debug(self, mask, pixel, score):
        overlay = self.latest_color.copy()
        overlay[mask] = (
            0.45 * overlay[mask]
            + 0.55 * np.array([255, 80, 20])
        ).astype(np.uint8)
        cv2.circle(overlay, pixel, 7, (0, 255, 255), -1)
        cv2.putText(
            overlay,
            f'SAM score {float(score):.3f}',
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        try:
            msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warning(
                f'Failed to publish SAM debug image: {exc}'
            )
            return
        msg.header = self.latest_camera_info.header
        self.debug_image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SamTableBboxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
