import copy
import math

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class PlanningMapFusion(Node):
    def __init__(self):
        super().__init__('planning_map_fusion')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('safety_grid_topic', '/safety_forbidden_grid')
        self.declare_parameter('planning_map_topic', '/planning_map')
        self.declare_parameter('occupied_threshold', 50)

        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.static_map = None
        self.safety_grid = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        volatile_qos = QoSProfile(depth=1)
        volatile_qos.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('planning_map_topic').value,
            transient_qos,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self.handle_map,
            transient_qos,
        )
        self.safety_subscription = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('safety_grid_topic').value,
            self.handle_safety_grid,
            volatile_qos,
        )

        self.timer = self.create_timer(0.5, self.publish_planning_map)

    def handle_map(self, msg):
        self.static_map = msg
        self.publish_planning_map()

    def handle_safety_grid(self, msg):
        self.safety_grid = msg
        self.publish_planning_map()

    def publish_planning_map(self):
        if self.static_map is None:
            return

        planning_map = copy.deepcopy(self.static_map)
        planning_map.header.stamp = self.get_clock().now().to_msg()

        if self.safety_grid is not None:
            try:
                self.overlay_safety_grid(planning_map, self.safety_grid)
            except TransformException as exc:
                self.get_logger().debug(f'Waiting to overlay safety grid: {exc}')

        self.publisher.publish(planning_map)

    def overlay_safety_grid(self, planning_map, safety_grid):
        transform = self.tf_buffer.lookup_transform(
            planning_map.header.frame_id,
            safety_grid.header.frame_id,
            Time(),
            timeout=RclpyDuration(seconds=0.05),
        )

        map_origin = planning_map.info.origin.position
        map_yaw = yaw_from_quaternion(planning_map.info.origin.orientation)
        safety_origin = safety_grid.info.origin.position
        safety_yaw = yaw_from_quaternion(safety_grid.info.origin.orientation)

        cos_map = math.cos(-map_yaw)
        sin_map = math.sin(-map_yaw)
        cos_safety = math.cos(safety_yaw)
        sin_safety = math.sin(safety_yaw)
        transform_yaw = yaw_from_quaternion(transform.transform.rotation)
        cos_tf = math.cos(transform_yaw)
        sin_tf = math.sin(transform_yaw)
        tf_translation = transform.transform.translation

        for safety_y in range(safety_grid.info.height):
            for safety_x in range(safety_grid.info.width):
                safety_index = safety_y * safety_grid.info.width + safety_x
                if safety_grid.data[safety_index] < self.occupied_threshold:
                    continue

                local_x = (safety_x + 0.5) * safety_grid.info.resolution
                local_y = (safety_y + 0.5) * safety_grid.info.resolution
                safety_frame_x = safety_origin.x + cos_safety * local_x - sin_safety * local_y
                safety_frame_y = safety_origin.y + sin_safety * local_x + cos_safety * local_y

                map_frame_x = (
                    tf_translation.x
                    + cos_tf * safety_frame_x
                    - sin_tf * safety_frame_y
                )
                map_frame_y = (
                    tf_translation.y
                    + sin_tf * safety_frame_x
                    + cos_tf * safety_frame_y
                )

                relative_x = map_frame_x - map_origin.x
                relative_y = map_frame_y - map_origin.y
                map_local_x = cos_map * relative_x - sin_map * relative_y
                map_local_y = sin_map * relative_x + cos_map * relative_y
                map_x = int(map_local_x / planning_map.info.resolution)
                map_y = int(map_local_y / planning_map.info.resolution)

                if 0 <= map_x < planning_map.info.width and 0 <= map_y < planning_map.info.height:
                    map_index = map_y * planning_map.info.width + map_x
                    planning_map.data[map_index] = 100


def main(args=None):
    rclpy.init(args=args)
    node = PlanningMapFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
