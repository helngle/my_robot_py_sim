import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml


def quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_vector(q, point):
    x, y, z = point
    qx, qy, qz, qw = q
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def compose_transform(parent, child):
    p_xyz, p_q = parent
    c_xyz, c_q = child
    rotated = rotate_vector(p_q, c_xyz)
    return (
        (
            p_xyz[0] + rotated[0],
            p_xyz[1] + rotated[1],
            p_xyz[2] + rotated[2],
        ),
        quat_multiply(p_q, c_q),
    )


def transform_point(transform, point):
    xyz, q = transform
    rotated = rotate_vector(q, point)
    return (
        xyz[0] + rotated[0],
        xyz[1] + rotated[1],
        xyz[2] + rotated[2],
    )


def yaw_from_quaternion(q):
    qx, qy, qz, qw = q
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def parse_pose(text):
    values = [float(value) for value in (text or '').split()]
    values += [0.0] * (6 - len(values))
    xyz = (values[0], values[1], values[2])
    q = quat_from_rpy(values[3], values[4], values[5])
    return xyz, q


def get_child_text(element, path, default=''):
    child = element.find(path)
    if child is None or child.text is None:
        return default
    return child.text.strip()


class Static25DForbiddenGrid(Node):
    def __init__(self):
        super().__init__('static_25d_forbidden_grid')
        share_dir = get_package_share_directory('my_robot_py_sim')
        self.declare_parameter(
            'world_file', os.path.join(share_dir, 'worlds', 'door_world.sdf'))
        self.declare_parameter(
            'urdf_file', os.path.join(share_dir, 'urdf', 'mobile_manipulator.urdf'))
        self.declare_parameter(
            'config_file', os.path.join(share_dir, 'config', 'safety_shell.yaml'))
        self.declare_parameter('grid_topic', '/static_25d_forbidden_grid')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('origin_x', -7.0)
        self.declare_parameter('origin_y', -5.0)
        self.declare_parameter('size_x', 14.0)
        self.declare_parameter('size_y', 10.0)
        self.declare_parameter('yaw_samples', 16)
        self.declare_parameter('planning_padding', 0.0)

        self.resolution = float(self.get_parameter('resolution').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.width = int(round(float(self.get_parameter('size_x').value) / self.resolution))
        self.height = int(round(float(self.get_parameter('size_y').value) / self.resolution))
        self.yaw_samples = max(1, int(self.get_parameter('yaw_samples').value))

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('grid_topic').value,
            qos,
        )

        self.grid = self.build_grid()
        self.create_timer(1.0, self.publish_grid)
        self.publish_grid()

    def build_grid(self):
        frame_transforms = self.load_urdf_fixed_transforms()
        shell_profiles = self.make_shell_profiles(frame_transforms)
        obstacle_points = self.load_world_obstacle_points()
        data = [0] * (self.width * self.height)

        for x, y, min_z, max_z in obstacle_points:
            for profile in shell_profiles:
                if max_z < profile['min_z'] or min_z > profile['max_z']:
                    continue
                for yaw_index in range(self.yaw_samples):
                    yaw = 2.0 * math.pi * yaw_index / self.yaw_samples
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    for offset_x, offset_y in profile['offsets']:
                        base_x = x - (cos_yaw * offset_x - sin_yaw * offset_y)
                        base_y = y - (sin_yaw * offset_x + cos_yaw * offset_y)
                        self.mark_cell(data, base_x, base_y)

        occupied = sum(1 for value in data if value > 0)
        self.get_logger().info(
            f'Static 2.5D forbidden grid ready: '
            f'{occupied} occupied cells from {len(obstacle_points)} obstacle samples'
        )
        return data

    def load_urdf_fixed_transforms(self):
        tree = ET.parse(self.get_parameter('urdf_file').value)
        root = tree.getroot()
        children = {}
        for joint in root.findall('joint'):
            if joint.get('type') != 'fixed':
                continue
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            transform = parse_pose(get_child_text(joint, 'origin'))
            children.setdefault(parent, []).append((child, transform))

        base_frame = self.get_parameter('base_frame').value
        transforms = {base_frame: ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))}
        queue = [base_frame]
        while queue:
            parent = queue.pop(0)
            for child, transform in children.get(parent, []):
                transforms[child] = compose_transform(transforms[parent], transform)
                queue.append(child)
        return transforms

    def make_shell_profiles(self, frame_transforms):
        with open(self.get_parameter('config_file').value, 'r') as stream:
            config = yaml.safe_load(stream) or {}
        safety_config = config.get('safety_shell', {})
        padding = float(self.get_parameter('planning_padding').value)
        profiles = []
        for shell in safety_config.get('shells', []):
            if not shell.get('planning_enabled', True):
                continue
            frame_id = shell['frame_id']
            if frame_id not in frame_transforms:
                self.get_logger().warn(f'Skip shell without URDF transform: {frame_id}')
                continue
            points = self.sample_shell_points(shell, padding)
            transformed = [
                transform_point(
                    compose_transform(
                        frame_transforms[frame_id],
                        (
                            tuple(float(value) for value in shell.get('pose', [0, 0, 0, 0, 0, 0, 1])[:3]),
                            tuple(float(value) for value in shell.get('pose', [0, 0, 0, 0, 0, 0, 1])[3:]),
                        ),
                    ),
                    point,
                )
                for point in points
            ]
            offsets = sorted({
                (
                    round(point[0] / self.resolution) * self.resolution,
                    round(point[1] / self.resolution) * self.resolution,
                )
                for point in transformed
            })
            z_values = [point[2] for point in transformed]
            profiles.append({
                'name': shell['name'],
                'offsets': offsets,
                'min_z': min(z_values),
                'max_z': max(z_values),
            })
        self.get_logger().info(f'Loaded {len(profiles)} static 2.5D shell profiles')
        return profiles

    def sample_shell_points(self, shell, padding):
        if shell['shape'] == 'box':
            sx, sy, sz = [float(value) + 2.0 * padding for value in shell['size']]
            points = []
            for x in self.sample_range(-0.5 * sx, 0.5 * sx):
                for y in self.sample_range(-0.5 * sy, 0.5 * sy):
                    points.append((x, y, 0.0))
            points.append((0.0, 0.0, -0.5 * sz))
            points.append((0.0, 0.0, 0.5 * sz))
            return points
        if shell['shape'] == 'cylinder':
            radius = float(shell['radius']) + padding
            length = float(shell['length']) + 2.0 * padding
            points = []
            for z in self.sample_range(-0.5 * length, 0.5 * length):
                points.append((0.0, 0.0, z))
                for index in range(12):
                    angle = 2.0 * math.pi * index / 12
                    points.append((radius * math.cos(angle), radius * math.sin(angle), z))
            return points
        return []

    def load_world_obstacle_points(self):
        tree = ET.parse(self.get_parameter('world_file').value)
        root = tree.getroot()
        points = []
        for model in root.findall('.//model'):
            if model.get('name') == 'ground_plane':
                continue
            model_tf = parse_pose(get_child_text(model, 'pose'))
            for link in model.findall('link'):
                link_tf = compose_transform(model_tf, parse_pose(get_child_text(link, 'pose')))
                for collision in link.findall('collision'):
                    collision_tf = compose_transform(
                        link_tf,
                        parse_pose(get_child_text(collision, 'pose')),
                    )
                    points.extend(self.sample_collision(collision, collision_tf))
        return points

    def sample_collision(self, collision, transform):
        geometry = collision.find('geometry')
        if geometry is None:
            return []
        if geometry.find('box') is not None:
            size = [
                float(value)
                for value in get_child_text(geometry.find('box'), 'size').split()
            ]
            return self.sample_box_obstacle(transform, size)
        if geometry.find('cylinder') is not None:
            cylinder = geometry.find('cylinder')
            radius = float(get_child_text(cylinder, 'radius'))
            length = float(get_child_text(cylinder, 'length'))
            return self.sample_cylinder_obstacle(transform, radius, length)
        if geometry.find('sphere') is not None:
            radius = float(get_child_text(geometry.find('sphere'), 'radius'))
            return self.sample_sphere_obstacle(transform, radius)
        return []

    def sample_box_obstacle(self, transform, size):
        sx, sy, sz = size
        corners = [
            transform_point(transform, (x, y, z))
            for x in (-0.5 * sx, 0.5 * sx)
            for y in (-0.5 * sy, 0.5 * sy)
            for z in (-0.5 * sz, 0.5 * sz)
        ]
        min_z = min(point[2] for point in corners)
        max_z = max(point[2] for point in corners)
        samples = []
        for x in self.sample_range(-0.5 * sx, 0.5 * sx):
            for y in self.sample_range(-0.5 * sy, 0.5 * sy):
                point = transform_point(transform, (x, y, 0.0))
                samples.append((point[0], point[1], min_z, max_z))
        return samples

    def sample_cylinder_obstacle(self, transform, radius, length):
        center = transform_point(transform, (0.0, 0.0, 0.0))
        min_z = center[2] - 0.5 * length
        max_z = center[2] + 0.5 * length
        samples = []
        for x in self.sample_range(-radius, radius):
            for y in self.sample_range(-radius, radius):
                if math.hypot(x, y) > radius:
                    continue
                point = transform_point(transform, (x, y, 0.0))
                samples.append((point[0], point[1], min_z, max_z))
        return samples

    def sample_sphere_obstacle(self, transform, radius):
        center = transform_point(transform, (0.0, 0.0, 0.0))
        samples = []
        for x in self.sample_range(-radius, radius):
            for y in self.sample_range(-radius, radius):
                if math.hypot(x, y) > radius:
                    continue
                point = transform_point(transform, (x, y, 0.0))
                samples.append((point[0], point[1], center[2] - radius, center[2] + radius))
        return samples

    def sample_range(self, start, end):
        values = []
        value = start
        while value < end:
            values.append(value)
            value += self.resolution
        values.append(end)
        return values

    def mark_cell(self, data, x, y):
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            data[grid_y * self.width + grid_x] = 100

    def publish_grid(self):
        grid = OccupancyGrid()
        grid.header.frame_id = self.get_parameter('frame_id').value
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = self.grid
        self.publisher.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = Static25DForbiddenGrid()
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
