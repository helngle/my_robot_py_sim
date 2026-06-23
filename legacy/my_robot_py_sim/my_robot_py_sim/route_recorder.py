import math
import os

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from my_robot_py_sim.route_utils import (
    default_routes_file,
    load_routes,
    quaternion_to_yaw,
    save_routes,
)


class RouteRecorder(Node):
    def __init__(self):
        super().__init__('route_recorder')

        self.declare_parameter('output_file', default_routes_file())
        self.declare_parameter('route_name', 'recorded_route')
        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('min_point_spacing', 0.35)
        self.declare_parameter('record_rate', 5.0)

        self.output_file = os.path.expanduser(
            self.get_parameter('output_file').value)
        self.route_name = self.get_parameter('route_name').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_point_spacing = self.get_parameter(
            'min_point_spacing').value
        self.record_rate = self.get_parameter('record_rate').value

        self.waypoints = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.warned_tf = False
        timer_period = 1.0 / max(self.record_rate, 0.1)
        self.create_timer(timer_period, self.record_pose)

        self.get_logger().info(
            f'Recording route from {self.fixed_frame} -> {self.base_frame}; '
            f'output={self.output_file}'
        )
        self.get_logger().info('Press Ctrl+C to save the recorded route.')

    def record_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            if not self.warned_tf:
                self.get_logger().warn(
                    f'Waiting for TF {self.fixed_frame} -> '
                    f'{self.base_frame}: {ex}'
                )
                self.warned_tf = True
            return

        translation = transform.transform.translation
        yaw = quaternion_to_yaw(transform.transform.rotation)
        waypoint = {
            'name': f'wp_{len(self.waypoints)}',
            'x': float(translation.x),
            'y': float(translation.y),
            'yaw': float(yaw),
        }

        if not self.waypoints:
            self.waypoints.append(waypoint)
            self.get_logger().info(
                f'Recorded wp_0 at x={waypoint["x"]:.2f}, '
                f'y={waypoint["y"]:.2f}'
            )
            return

        if self.distance(waypoint, self.waypoints[-1]) < \
                self.min_point_spacing:
            return

        self.waypoints.append(waypoint)
        self.get_logger().info(
            f'Recorded {waypoint["name"]} at '
            f'x={waypoint["x"]:.2f}, y={waypoint["y"]:.2f}'
        )

    def save(self):
        if len(self.waypoints) < 2:
            self.get_logger().error(
                'Route was not saved: at least 2 recorded points are needed.')
            return False

        data = load_routes(self.output_file)
        data.setdefault('routes', {})
        data['routes'][self.route_name] = {
            'frame_id': self.fixed_frame,
            'waypoints': self.waypoints,
        }
        save_routes(self.output_file, data)

        self.get_logger().info(
            f'Saved {len(self.waypoints)} waypoints to {self.output_file}')
        return True

    @staticmethod
    def distance(first, second):
        return math.hypot(first['x'] - second['x'],
                          first['y'] - second['y'])


def main(args=None):
    rclpy.init(args=args)
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        ok = node.save()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
