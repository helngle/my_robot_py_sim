import math
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField

FLOAT32 = PointField.FLOAT32


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class GridToPointCloud(Node):
    """Convert OccupancyGrid cells to PointCloud2 for costmap obstacle layers.

    Subscribes to an OccupancyGrid (e.g. /safety_forbidden_grid) and
    publishes occupied cells as PointCloud2 points at a configurable Z
    height.  Designed to feed Nav2 local costmap obstacle_layer so the
    DWB controller can see the full 3D safety shell projections.
    """

    def __init__(self):
        super().__init__('grid_to_pointcloud')
        self.declare_parameter('grid_topic', '/safety_forbidden_grid')
        self.declare_parameter('cloud_topic', '/safety_forbidden_cloud')
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('point_z', 0.5)
        self.declare_parameter('publish_rate', 5.0)

        self.threshold = int(self.get_parameter('occupied_threshold').value)
        self.point_z = float(self.get_parameter('point_z').value)
        self.latest_grid = None

        self.subscription = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('grid_topic').value,
            self.handle_grid,
            10,
        )
        self.publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter('cloud_topic').value,
            10,
        )
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter('publish_rate').value),
            self.publish_cloud,
        )

    def handle_grid(self, msg):
        self.latest_grid = msg
        self.publish_cloud()

    def publish_cloud(self):
        if self.latest_grid is None:
            return
        cloud = self.grid_to_cloud(self.latest_grid)
        if cloud is not None:
            self.publisher.publish(cloud)

    def grid_to_cloud(self, grid):
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        yaw = yaw_from_quaternion(grid.info.origin.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        points = []
        for i, value in enumerate(grid.data):
            if value < self.threshold or value < 0:
                continue
            gx = (i % width + 0.5) * resolution
            gy = (i // width + 0.5) * resolution
            wx = origin_x + cos_yaw * gx - sin_yaw * gy
            wy = origin_y + sin_yaw * gx + cos_yaw * gy
            points.append((wx, wy, self.point_z))

        if not points:
            return None

        cloud = PointCloud2()
        cloud.header = grid.header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_bigendian = False
        cloud.is_dense = True

        buf = bytearray(cloud.row_step)
        pack_into = struct.Struct('<fff').pack_into
        for i, (x, y, z) in enumerate(points):
            pack_into(buf, i * 12, x, y, z)
        cloud.data = bytes(buf)
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = GridToPointCloud()
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
