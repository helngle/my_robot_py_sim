import math
import os
import struct

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


FLOAT32 = PointField.FLOAT32


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


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q, v):
    x, y, z = v
    qx, qy, qz, qw = q

    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def inverse_rotate_vector(q, v):
    qx, qy, qz, qw = q
    return rotate_vector((-qx, -qy, -qz, qw), v)


def transform_point(transform, point):
    rotation = transform.transform.rotation
    q = (rotation.x, rotation.y, rotation.z, rotation.w)
    rotated = rotate_vector(q, point)
    translation = transform.transform.translation
    return (
        rotated[0] + translation.x,
        rotated[1] + translation.y,
        rotated[2] + translation.z,
    )


def inverse_transform_point(transform, point):
    translation = transform.transform.translation
    shifted = (
        point[0] - translation.x,
        point[1] - translation.y,
        point[2] - translation.z,
    )
    rotation = transform.transform.rotation
    return inverse_rotate_vector(
        (rotation.x, rotation.y, rotation.z, rotation.w),
        shifted,
    )


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class SafetyForbiddenGrid(Node):
    def __init__(self):
        super().__init__('safety_forbidden_grid')
        self.declare_parameter('cloud_topic', '/lidar/points')
        self.declare_parameter('grid_topic', '/safety_forbidden_grid')
        self.declare_parameter('config_file', '')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('grid_size', 10.0)
        self.declare_parameter('forward_offset', 0.0)
        self.declare_parameter('max_points', 5000)
        self.declare_parameter('planning_padding', -1.0)
        self.declare_parameter('self_filter_padding', 0.03)
        self.declare_parameter('obstacle_keep_time', 0.0)
        self.declare_parameter('min_obstacle_z', 0.25)
        self.declare_parameter('max_obstacle_z', 1.8)

        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.grid_size = float(self.get_parameter('grid_size').value)
        self.forward_offset = float(self.get_parameter('forward_offset').value)
        self.width = int(round(self.grid_size / self.resolution))
        self.height = self.width
        self.max_points = int(self.get_parameter('max_points').value)
        self.planning_padding = float(self.get_parameter('planning_padding').value)
        self.self_filter_padding = float(self.get_parameter('self_filter_padding').value)
        self.obstacle_keep_time = float(self.get_parameter('obstacle_keep_time').value)
        self.min_obstacle_z = float(self.get_parameter('min_obstacle_z').value)
        self.max_obstacle_z = float(self.get_parameter('max_obstacle_z').value)
        self.occupied_cells = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.shells = self.load_shells(planning_only=True)
        self.self_filter_shells = self.load_shells(
            planning_only=False,
            padding_override=self.self_filter_padding,
        )

        self.publisher = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('grid_topic').value,
            10,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter('cloud_topic').value,
            self.handle_cloud,
            10,
        )

    def load_shells(self, planning_only=True, padding_override=None):
        config_file = self.get_parameter('config_file').value
        if not config_file:
            config_file = os.path.join(
                get_package_share_directory('my_robot_py_sim'),
                'config',
                'safety_shell.yaml',
            )

        with open(config_file, 'r') as stream:
            config = yaml.safe_load(stream) or {}
        safety_config = config.get('safety_shell', {})
        padding = self.planning_padding if padding_override is None else padding_override
        if padding < 0.0 and padding_override is None:
            padding = float(safety_config.get('planning_padding', safety_config.get('padding', 0.0)))
        shells = []
        for shell in safety_config.get('shells', []):
            if planning_only and not shell.get('planning_enabled', True):
                continue
            shells.append(self.make_shell(shell, padding))
        purpose = 'planning' if planning_only else 'self-filter'
        self.get_logger().info(
            f'Loaded {len(shells)} {purpose} shells from {config_file} '
            f'with planning padding {padding:.3f} m'
        )
        return shells

    def make_shell(self, shell, padding):
        shape = shell.get('shape')
        if shape == 'box':
            sx, sy, sz = shell['size']
            size = (sx + 2.0 * padding, sy + 2.0 * padding, sz + 2.0 * padding)
            radius = None
            length = None
        elif shape == 'cylinder':
            radius = float(shell['radius']) + padding
            length = float(shell['length']) + 2.0 * padding
            diameter = 2.0 * radius
            size = (diameter, diameter, length)
        else:
            raise ValueError(f'Unknown safety shell shape: {shape}')

        return {
            'name': shell['name'],
            'frame_id': shell['frame_id'],
            'shape': shape,
            'pose': tuple(float(value) for value in shell.get('pose', [0, 0, 0, 0, 0, 0, 1])),
            'size': size,
            'radius': radius,
            'length': length,
        }

    def handle_cloud(self, cloud):
        try:
            cloud_to_fixed = self.lookup_transform(
                self.fixed_frame,
                cloud.header.frame_id,
                Time.from_msg(cloud.header.stamp),
            )
            cloud_to_base = self.lookup_transform(
                self.base_frame,
                cloud.header.frame_id,
                Time.from_msg(cloud.header.stamp),
            )
            base_to_fixed = self.lookup_transform(
                self.fixed_frame,
                self.base_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(f'Waiting for transforms: {exc}')
            return

        shell_profiles = self.make_shell_profiles(base_to_fixed)
        if not shell_profiles:
            return
        self_filter_volumes = self.make_self_filter_volumes()

        yaw = yaw_from_quaternion(base_to_fixed.transform.rotation)
        center_x = base_to_fixed.transform.translation.x + self.forward_offset * math.cos(yaw)
        center_y = base_to_fixed.transform.translation.y + self.forward_offset * math.sin(yaw)
        origin_x = center_x - 0.5 * self.grid_size
        origin_y = center_y - 0.5 * self.grid_size
        now = self.get_clock().now().nanoseconds * 1e-9

        for raw_point in iter_xyz_points(cloud, self.max_points):
            point_in_base = transform_point(cloud_to_base, raw_point)
            if self.point_inside_robot(point_in_base, self_filter_volumes):
                continue

            point = transform_point(cloud_to_fixed, raw_point)
            if point[2] < self.min_obstacle_z or point[2] > self.max_obstacle_z:
                continue

            for profile in shell_profiles:
                if point[2] < profile['min_z'] or point[2] > profile['max_z']:
                    continue
                self.mark_forbidden_cells(point, profile, now)

        data = self.make_grid_data(origin_x, origin_y, now)
        self.publisher.publish(self.make_grid(data, origin_x, origin_y))

    def lookup_transform(self, target, source, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                stamp,
                timeout=RclpyDuration(seconds=0.05),
            )
        except TransformException:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                Time(),
                timeout=RclpyDuration(seconds=0.05),
            )

    def make_shell_profiles(self, base_to_fixed):
        yaw = yaw_from_quaternion(base_to_fixed.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        base_z = base_to_fixed.transform.translation.z
        profiles = []

        for shell in self.shells:
            try:
                shell_to_base = self.lookup_transform(
                    self.base_frame,
                    shell['frame_id'],
                    Time(),
                )
            except TransformException:
                continue

            offsets, min_z, max_z = self.shell_profile_in_base(
                shell,
                shell_to_base,
            )
            if not offsets:
                continue
            profiles.append({
                'name': shell['name'],
                'min_z': base_z + min_z,
                'max_z': base_z + max_z,
                'offsets': offsets,
                'cos_yaw': cos_yaw,
                'sin_yaw': sin_yaw,
            })
        return profiles

    def make_self_filter_volumes(self):
        volumes = []
        for shell in self.self_filter_shells:
            try:
                shell_to_base = self.lookup_transform(
                    self.base_frame,
                    shell['frame_id'],
                    Time(),
                )
            except TransformException:
                continue
            volumes.append((shell, shell_to_base))
        return volumes

    def point_inside_robot(self, point_in_base, volumes):
        for shell, shell_to_base in volumes:
            point = self.base_point_to_shell_local(shell, shell_to_base, point_in_base)
            if self.point_inside_shell(shell, point):
                return True
        return False

    def base_point_to_shell_local(self, shell, shell_to_base, point_in_base):
        point_in_shell_frame = inverse_transform_point(shell_to_base, point_in_base)
        pose = shell['pose']
        pose_translation = pose[:3]
        pose_rotation = pose[3:]
        centered = (
            point_in_shell_frame[0] - pose_translation[0],
            point_in_shell_frame[1] - pose_translation[1],
            point_in_shell_frame[2] - pose_translation[2],
        )
        return inverse_rotate_vector(pose_rotation, centered)

    def point_inside_shell(self, shell, point):
        if shell['shape'] == 'box':
            sx, sy, sz = shell['size']
            return (
                abs(point[0]) <= 0.5 * sx
                and abs(point[1]) <= 0.5 * sy
                and abs(point[2]) <= 0.5 * sz
            )
        if shell['shape'] == 'cylinder':
            radius = shell['radius']
            length = shell['length']
            return math.hypot(point[0], point[1]) <= radius and abs(point[2]) <= 0.5 * length
        return False

    def shell_profile_in_base(self, shell, shell_to_base):
        points = self.sample_shell_points(shell)
        points_in_base = [self.shell_point_to_base(shell, shell_to_base, point) for point in points]
        if not points_in_base:
            return [], 0.0, 0.0

        offsets = sorted({
            (
                round(point[0] / self.resolution) * self.resolution,
                round(point[1] / self.resolution) * self.resolution,
            )
            for point in points_in_base
        })
        z_values = [point[2] for point in points_in_base]
        return offsets, min(z_values), max(z_values)

    def sample_shell_points(self, shell):
        if shell['shape'] == 'box':
            return self.sample_box_points(shell['size'])
        if shell['shape'] == 'cylinder':
            return self.sample_cylinder_points(shell['radius'], shell['length'])
        return []

    def sample_box_points(self, size):
        sx, sy, sz = size
        points = []
        for x in self.sample_range(-0.5 * sx, 0.5 * sx):
            for y in self.sample_range(-0.5 * sy, 0.5 * sy):
                points.append((x, y, 0.0))

        # Z samples only affect height overlap, not the XY footprint.
        points.extend([
            (0.0, 0.0, -0.5 * sz),
            (0.0, 0.0, 0.5 * sz),
        ])
        return points

    def sample_cylinder_points(self, radius, length):
        points = []
        angular_samples = 12
        radial_samples = (0.0, radius)
        for z in self.sample_range(-0.5 * length, 0.5 * length):
            for radial in radial_samples:
                if radial == 0.0:
                    points.append((0.0, 0.0, z))
                    continue
                for index in range(angular_samples):
                    angle = 2.0 * math.pi * index / angular_samples
                    points.append((radial * math.cos(angle), radial * math.sin(angle), z))
        return points

    def sample_range(self, start, end):
        if end < start:
            return []
        step = max(self.resolution, 0.05)
        values = []
        value = start
        while value < end:
            values.append(value)
            value += step
        values.append(end)
        return values

    def shell_point_to_base(self, shell, shell_to_base, point):
        pose = shell['pose']
        pose_translation = pose[:3]
        pose_rotation = pose[3:]
        local = rotate_vector(pose_rotation, point)
        local = (
            local[0] + pose_translation[0],
            local[1] + pose_translation[1],
            local[2] + pose_translation[2],
        )
        return transform_point(shell_to_base, local)

    def mark_forbidden_cells(self, point, profile, stamp):
        cos_yaw = profile['cos_yaw']
        sin_yaw = profile['sin_yaw']
        for offset_x, offset_y in profile['offsets']:
            rotated_x = cos_yaw * offset_x - sin_yaw * offset_y
            rotated_y = sin_yaw * offset_x + cos_yaw * offset_y
            base_x = point[0] - rotated_x
            base_y = point[1] - rotated_y
            world_x = int(math.floor(base_x / self.resolution))
            world_y = int(math.floor(base_y / self.resolution))
            self.occupied_cells[(world_x, world_y)] = stamp

    def make_grid_data(self, origin_x, origin_y, stamp):
        data = [0] * (self.width * self.height)
        if self.obstacle_keep_time > 0.0:
            cutoff = stamp - self.obstacle_keep_time
            self.occupied_cells = {
                cell: last_seen
                for cell, last_seen in self.occupied_cells.items()
                if last_seen >= cutoff
            }
        else:
            self.occupied_cells = {
                cell: last_seen
                for cell, last_seen in self.occupied_cells.items()
                if last_seen == stamp
            }

        for world_x, world_y in self.occupied_cells:
            cell_x = world_x * self.resolution
            cell_y = world_y * self.resolution
            grid_x = int((cell_x - origin_x) / self.resolution)
            grid_y = int((cell_y - origin_y) / self.resolution)
            if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                data[grid_y * self.width + grid_x] = 100
        return data

    def make_grid(self, data, origin_x, origin_y):
        grid = OccupancyGrid()
        grid.header.frame_id = self.fixed_frame
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = origin_x
        grid.info.origin.position.y = origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        return grid


def main(args=None):
    rclpy.init(args=args)
    node = SafetyForbiddenGrid()
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
