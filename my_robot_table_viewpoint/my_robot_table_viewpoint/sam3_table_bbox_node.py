import cv2
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D
from visualization_msgs.msg import Marker

from .sam_table_bbox_node import SamTableBboxNode


try:
    from ultralytics.models.sam.predict import SAM3SemanticPredictor
except ImportError:
    SAM3SemanticPredictor = None


class Sam3TableBboxNode(SamTableBboxNode):
    """Generate /target_bbox_3d from a SAM3 text prompt and aligned depth.

    This experimental node deliberately reuses the same depth-to-3D-box fitting
    code as the SAM1 click node, but replaces the click prompt with SAM3 open
    vocabulary text segmentation such as "office desk" or "table".
    """

    def __init__(self):
        Node.__init__(self, 'sam3_table_bbox_node')
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/color/camera_info',
        )
        self.declare_parameter('bbox_topic', '/target_bbox_3d')
        self.declare_parameter('debug_image_topic', '/sam3_table_mask/debug')
        self.declare_parameter('marker_topic', '/sam3_table_bbox_marker')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('sam3_model', '')
        self.declare_parameter('sam3_prompt', 'office desk')
        self.declare_parameter('sam3_device', 'cuda')
        self.declare_parameter('sam3_confidence', 0.25)
        self.declare_parameter('sam3_imgsz', 1024)
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
        self.declare_parameter('bbox_id', 'sam3_table')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_republish_rate_hz', 2.0)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.bbox_topic = self.get_parameter('bbox_topic').value
        self.debug_image_topic = self.get_parameter('debug_image_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.sam3_model_path = self.get_parameter('sam3_model').value
        self.sam3_prompt = self.get_parameter('sam3_prompt').value
        self.sam3_device = self.get_parameter('sam3_device').value
        self.sam3_confidence = float(
            self.get_parameter('sam3_confidence').value
        )
        self.sam3_imgsz = int(self.get_parameter('sam3_imgsz').value)
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
        self.debug_republish_rate_hz = float(
            self.get_parameter('debug_republish_rate_hz').value
        )

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_color = None
        self.latest_depth = None
        self.latest_camera_info = None
        self.latest_debug_image_msg = None
        self.predictor = self.load_sam3_predictor()

        self.bbox_pub = self.create_publisher(Detection3D, self.bbox_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.debug_image_pub = self.create_publisher(
            Image,
            self.debug_image_topic,
            qos_profile_sensor_data,
        )
        self.create_service(Trigger, 'detect_sam3_table', self.detect_service)
        if self.debug_republish_rate_hz > 0.0:
            self.create_timer(
                1.0 / self.debug_republish_rate_hz,
                self.republish_debug_image,
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
            'SAM3 table bbox node listening on '
            f'{self.color_topic}, {self.depth_topic}, '
            f'{self.camera_info_topic}; call /detect_sam3_table to segment '
            f'"{self.sam3_prompt}" and publish {self.bbox_topic}.'
        )

    def load_sam3_predictor(self):
        if SAM3SemanticPredictor is None:
            self.get_logger().error(
                'Ultralytics SAM3SemanticPredictor is not available. '
                'Upgrade ultralytics to a SAM3-capable version.'
            )
            return None
        if not self.sam3_model_path:
            self.get_logger().error(
                'sam3_model is empty. Set it to your SAM3 .pt model path.'
            )
            return None
        overrides = {
            'model': self.sam3_model_path,
            'task': 'segment',
            'mode': 'predict',
            'conf': self.sam3_confidence,
            'imgsz': self.sam3_imgsz,
            'device': self.sam3_device,
            'verbose': False,
        }
        try:
            predictor = SAM3SemanticPredictor(overrides=overrides)
            predictor.setup_model(verbose=False)
            self.patch_sam3_tokenizer(predictor.model)
        except Exception as exc:
            self.get_logger().error(f'Failed to load SAM3 model: {exc}')
            return None
        self.get_logger().info(
            f'Loaded SAM3 model {self.sam3_model_path} on '
            f'{self.sam3_device}; prompt="{self.sam3_prompt}".'
        )
        return predictor

    def patch_sam3_tokenizer(self, model):
        """Make Ultralytics SAM3 use the callable CLIP tokenize function.

        Some CLIP packages expose ``clip.simple_tokenizer.SimpleTokenizer`` as
        a non-callable object, while this Ultralytics SAM3 version calls the
        tokenizer like a function. Replacing that field with ``clip.tokenize``
        keeps the fix local to this experimental node.
        """
        try:
            import clip
        except ImportError:
            return

        patched = 0
        for module in model.modules():
            tokenizer = getattr(module, 'tokenizer', None)
            if tokenizer is not None and not callable(tokenizer):
                setattr(module, 'tokenizer', clip.tokenize)
                patched += 1
        if patched:
            self.get_logger().info(
                f'Patched {patched} SAM3 tokenizer reference(s) to '
                'use clip.tokenize.'
            )

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

    def detect_service(self, request, response):
        del request
        if self.predictor is None:
            response.success = False
            response.message = 'SAM3 predictor is not ready.'
            return response
        if (
            self.latest_color is None
            or self.latest_depth is None
            or self.latest_camera_info is None
        ):
            response.success = False
            response.message = 'No synchronized RGB-D frame yet.'
            return response

        mask = self.segment_latest_frame()
        if mask is None:
            response.success = False
            response.message = (
                f'No usable SAM3 mask for prompt "{self.sam3_prompt}".'
            )
            return response

        bbox = self.fit_bbox_from_mask(mask, self.latest_camera_info)
        if bbox is None:
            response.success = False
            response.message = 'SAM3 mask had insufficient valid depth.'
            return response

        stamp = self.latest_camera_info.header.stamp
        self.publish_bbox(bbox, stamp)
        self.publish_marker(bbox, stamp)
        if self.publish_debug_image:
            self.publish_mask_debug(mask, self.sam3_prompt)
        response.success = True
        response.message = 'Published SAM3 table Detection3D.'
        return response

    def segment_latest_frame(self):
        color = self.latest_color.copy()
        try:
            results = self.predictor(
                source=color,
                stream=False,
                text=[self.sam3_prompt],
            )
        except Exception as exc:
            self.get_logger().error(f'SAM3 inference failed: {exc}')
            return None
        if not results:
            return None
        result = results[0]
        if result.masks is None or len(result.masks) == 0:
            self.get_logger().warning(
                f'SAM3 found no masks for "{self.sam3_prompt}".'
            )
            return None

        masks = result.masks.data
        masks_np = masks.detach().cpu().numpy().astype(bool)
        if masks_np.ndim != 3:
            return None

        scores = None
        if result.boxes is not None and len(result.boxes) == len(masks_np):
            scores = result.boxes.conf.detach().cpu().numpy()
        areas = np.count_nonzero(masks_np, axis=(1, 2))
        valid = areas >= self.min_mask_area_px
        if not np.any(valid):
            self.get_logger().warning(
                f'SAM3 masks are too small; largest area={int(areas.max())} px.'
            )
            return None
        candidate_indices = np.flatnonzero(valid)
        if scores is not None:
            best = candidate_indices[
                int(np.argmax(scores[candidate_indices]))
            ]
        else:
            best = candidate_indices[int(np.argmax(areas[candidate_indices]))]
        self.get_logger().info(
            f'SAM3 selected mask area={int(areas[best])} px'
            + (
                f', score={float(scores[best]):.3f}'
                if scores is not None
                else ''
            )
            + f' for prompt "{self.sam3_prompt}".'
        )
        return masks_np[best]

    def publish_mask_debug(self, mask, prompt):
        overlay = self.latest_color.copy()
        overlay[mask] = (
            0.45 * overlay[mask]
            + 0.55 * np.array([60, 180, 255])
        ).astype(np.uint8)
        cv2.putText(
            overlay,
            f'SAM3: {prompt}',
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
                f'Failed to publish SAM3 debug image: {exc}'
            )
            return
        msg.header = self.latest_camera_info.header
        self.latest_debug_image_msg = msg
        self.debug_image_pub.publish(msg)

    def republish_debug_image(self):
        if self.latest_debug_image_msg is None:
            return
        self.latest_debug_image_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_image_pub.publish(self.latest_debug_image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Sam3TableBboxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
