import math
import struct

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


FLOAT32 = PointField.FLOAT32


class Shell:
    def __init__(self, name, center, size, color):
        self.name = name
        self.center = center
        self.size = size
        self.color = color
        self.half = tuple(value * 0.5 for value in size)

    def contains(self, point):
        return (
            abs(point[0] - self.center[0]) <= self.half[0]
            and abs(point[1] - self.center[1]) <= self.half[1]
            and abs(point[2] - self.center[2]) <= self.half[2]
        )

    def forbidden_xy_for_point(self, point):
        return (
            point[0] - self.center[0],
            point[1] - self.center[1],
        )


def rotate_vector(q, v):
    x, y, z = v
    qx = q.x
    qy = q.y
    qz = q.z
    qw = q.w

    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def transform_point(transform, point):
    rotated = rotate_vector(transform.transform.rotation, point)
    translation = transform.transform.translation
    return (
        rotated[0] + translation.x,
        rotated[1] + translation.y,
        rotated[2] + translation.z,
    )


def find_float_field_offsets(cloud):
    offsets = {}
    for field in cloud.fields:
        if field.name in ('x', 'y', 'z') and field.datatype == FLOAT32:
            offsets[field.name] = field.offset
    if {'x', 'y', 'z'} <= set(offsets):
        return offsets
    return None


def iter_xyz_points(cloud, max_points):
    offsets = find_float_field_offsets(cloud)
    if offsets is None or cloud.point_step <= 0:
        return

    data = cloud.data
    total = cloud.width * cloud.height
    step = max(1, total // max_points) if max_points > 0 else 1
    endian = '>' if cloud.is_bigendian else '<'

    for index in range(0, total, step):
        base = index * cloud.point_step
        try:
            x = struct.unpack_from(endian + 'f', data, base + offsets['x'])[0]
            y = struct.unpack_from(endian + 'f', data, base + offsets['y'])[0]
            z = struct.unpack_from(endian + 'f', data, base + offsets['z'])[0]
        except struct.error:
            return

        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            yield (x, y, z)


class FullBodyClearanceChecker(Node):
    def __init__(self):
        super().__init__('full_body_clearance_checker')
        self.declare_parameter('cloud_topic', '/lidar/points')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('collision_topic', '/full_body_collision_markers')
        self.declare_parameter('forbidden_topic', '/full_body_forbidden_zones')
        self.declare_parameter('voxel_size', 0.12)
        self.declare_parameter('max_points', 12000)
        self.declare_parameter('min_z', 0.02)
        self.declare_parameter('max_z', 1.6)
        self.declare_parameter('max_range_xy', 5.0)

        self.target_frame = self.get_parameter('target_frame').value
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.min_z = float(self.get_parameter('min_z').value)
        self.max_z = float(self.get_parameter('max_z').value)
        self.max_range_xy = float(self.get_parameter('max_range_xy').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.collision_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('collision_topic').value,
            10,
        )
        self.forbidden_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('forbidden_topic').value,
            10,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter('cloud_topic').value,
            self.handle_cloud,
            10,
        )

        self.shells = self.create_shells()

    def create_shells(self):
        return [
            Shell('base', (0.0, 0.0, 0.18), (0.84, 0.84, 0.36), (0.95, 0.25, 0.15, 0.85)),
            Shell('torso', (0.0, 0.0, 0.65), (0.56, 0.48, 0.82), (0.95, 0.55, 0.12, 0.78)),
            Shell('head', (0.04, 0.0, 1.08), (0.38, 0.36, 0.30), (0.85, 0.15, 0.75, 0.78)),
            Shell('sensor_mount', (0.20, 0.0, 1.18), (0.28, 0.24, 0.20), (0.35, 0.55, 1.0, 0.75)),
            Shell('lidar', (0.22, 0.0, 1.26), (0.22, 0.22, 0.22), (0.15, 0.75, 1.0, 0.75)),
            Shell('left_upper_arm', (0.0, 0.39, 0.895), (0.20, 0.46, 0.20), (1.0, 0.1, 0.1, 0.78)),
            Shell('left_forearm', (0.0, 0.71, 0.895), (0.18, 0.42, 0.18), (1.0, 0.45, 0.05, 0.78)),
            Shell('left_hand', (0.0, 0.88, 0.895), (0.20, 0.18, 0.22), (1.0, 0.75, 0.05, 0.78)),
            Shell('right_upper_arm', (0.0, -0.39, 0.895), (0.20, 0.46, 0.20), (1.0, 0.1, 0.1, 0.78)),
            Shell('right_forearm', (0.0, -0.71, 0.895), (0.18, 0.42, 0.18), (1.0, 0.45, 0.05, 0.78)),
            Shell('right_hand', (0.0, -0.88, 0.895), (0.20, 0.18, 0.22), (1.0, 0.75, 0.05, 0.78)),
        ]

    def handle_cloud(self, cloud):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                cloud.header.frame_id,
                Time.from_msg(cloud.header.stamp),
                timeout=RclpyDuration(seconds=0.05),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    cloud.header.frame_id,
                    Time(),
                    timeout=RclpyDuration(seconds=0.05),
                )
            except TransformException as exc:
                self.get_logger().debug(f'No transform for cloud: {exc}')
                return

        shell_hits = {shell.name: set() for shell in self.shells}
        forbidden_cells = {shell.name: set() for shell in self.shells}

        for raw_point in iter_xyz_points(cloud, self.max_points):
            point = transform_point(transform, raw_point)
            if not self.point_in_bounds(point):
                continue

            for shell in self.shells:
                if shell.contains(point):
                    shell_hits[shell.name].add(self.voxel_key(point))
                    forbidden_cells[shell.name].add(self.xy_key(shell.forbidden_xy_for_point(point)))

        stamp = self.get_clock().now().to_msg()
        self.collision_pub.publish(self.make_collision_markers(shell_hits, stamp))
        self.forbidden_pub.publish(self.make_forbidden_markers(forbidden_cells, stamp))

    def point_in_bounds(self, point):
        if point[2] < self.min_z or point[2] > self.max_z:
            return False
        return math.hypot(point[0], point[1]) <= self.max_range_xy

    def voxel_key(self, point):
        return tuple(round(value / self.voxel_size) for value in point)

    def xy_key(self, xy):
        return (
            round(xy[0] / self.voxel_size),
            round(xy[1] / self.voxel_size),
        )

    def point_from_voxel(self, key):
        return Point(
            x=float(key[0] * self.voxel_size),
            y=float(key[1] * self.voxel_size),
            z=float(key[2] * self.voxel_size),
        )

    def point_from_xy(self, key, z):
        return Point(
            x=float(key[0] * self.voxel_size),
            y=float(key[1] * self.voxel_size),
            z=float(z),
        )

    def make_cube_list(self, marker_id, namespace, points, color, scale_z, stamp):
        marker = Marker()
        marker.header.frame_id = self.target_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = scale_z
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        marker.lifetime = Duration(sec=1, nanosec=0)
        marker.points = points
        return marker

    def make_collision_markers(self, shell_hits, stamp):
        markers = MarkerArray()
        for marker_id, shell in enumerate(self.shells):
            points = [self.point_from_voxel(key) for key in sorted(shell_hits[shell.name])]
            markers.markers.append(
                self.make_cube_list(marker_id, shell.name, points, shell.color, self.voxel_size, stamp)
            )
        return markers

    def make_forbidden_markers(self, forbidden_cells, stamp):
        markers = MarkerArray()
        for marker_id, shell in enumerate(self.shells):
            points = [self.point_from_xy(key, 0.04) for key in sorted(forbidden_cells[shell.name])]
            color = (1.0, 0.0, 0.0, 0.55) if shell.name != 'base' else (1.0, 0.65, 0.0, 0.60)
            markers.markers.append(
                self.make_cube_list(marker_id, shell.name, points, color, 0.04, stamp)
            )
        return markers


def main(args=None):
    rclpy.init(args=args)
    node = FullBodyClearanceChecker()
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
