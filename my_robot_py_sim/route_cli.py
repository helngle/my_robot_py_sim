import argparse
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener

from my_robot_py_sim.route_utils import (
    default_routes_file,
    load_routes,
    nearest_waypoint_name,
    route_segment_waypoints,
    route_to_poses,
    waypoint_to_pose,
)


class RouteCli(Node):
    def __init__(self, route_file):
        super().__init__('route_cli')
        self.route_file = route_file
        self.go_to_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self.follow_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def list_routes(self):
        data = load_routes(self.route_file)
        routes = data.get('routes', {})
        if not routes:
            self.get_logger().info(f'No routes in {self.route_file}')
            return True

        for route_name, route in routes.items():
            names = [
                waypoint.get('name', str(index))
                for index, waypoint in enumerate(route.get('waypoints', []))
            ]
            self.get_logger().info(f'{route_name}: {", ".join(names)}')
        return True

    def go_to(self, route_name, waypoint_name):
        route = self.get_route(route_name)
        for waypoint in route.get('waypoints', []):
            if waypoint.get('name') == waypoint_name:
                pose = waypoint_to_pose(waypoint, route.get('frame_id', 'map'))
                pose.header.stamp = self.get_clock().now().to_msg()
                return self.send_go_to(pose, route_name, waypoint_name)
        raise ValueError(f'Waypoint "{waypoint_name}" was not found.')

    def follow(self, route_name, start_name=None, goal_name=None):
        route = self.get_route(route_name)
        poses = route_to_poses(route, start_name, goal_name)
        stamp = self.get_clock().now().to_msg()
        for pose in poses:
            pose.header.stamp = stamp
        return self.send_follow(poses, route_name)

    def go_to_nearest(self, route_name):
        route = self.get_route(route_name)
        waypoint_name = self.find_nearest_name(route)
        self.get_logger().info(
            f'Nearest waypoint on {route_name}: {waypoint_name}')
        return self.go_to(route_name, waypoint_name)

    def follow_nearest(self, route_name, goal_name=None):
        route = self.get_route(route_name)
        start_name = self.find_nearest_name(route)
        self.get_logger().info(
            f'Nearest route entry on {route_name}: {start_name}')
        return self.follow(route_name, start_name, goal_name)

    def preview_nearest(self, route_name, goal_name=None):
        route = self.get_route(route_name)
        fixed_frame = route.get('frame_id', 'map')
        transform = self.lookup_robot_pose(fixed_frame)
        translation = transform.transform.translation
        start_name = nearest_waypoint_name(
            route, translation.x, translation.y)
        segment = route_segment_waypoints(route, start_name, goal_name)
        names = [
            waypoint.get('name', str(index))
            for index, waypoint in enumerate(segment)
        ]

        self.get_logger().info(
            f'Current pose in {fixed_frame}: '
            f'x={translation.x:.2f}, y={translation.y:.2f}')
        self.get_logger().info(f'Nearest route entry: {start_name}')
        self.get_logger().info(f'Target waypoint: {names[-1]}')
        self.get_logger().info('Waypoint sequence: ' + ' -> '.join(names))
        return True

    def find_nearest_name(self, route):
        fixed_frame = route.get('frame_id', 'map')
        transform = self.lookup_robot_pose(fixed_frame)
        translation = transform.transform.translation
        return nearest_waypoint_name(route, translation.x, translation.y)

    def lookup_robot_pose(self, fixed_frame):
        deadline = time.monotonic() + 5.0
        last_error = None
        while time.monotonic() < deadline:
            try:
                return self.tf_buffer.lookup_transform(
                    fixed_frame,
                    'base_footprint',
                    rclpy.time.Time(),
                )
            except TransformException as ex:
                last_error = ex
                rclpy.spin_once(self, timeout_sec=0.1)

        raise ValueError(
            f'Could not find robot pose in {fixed_frame}: {last_error}')

    def get_route(self, route_name):
        data = load_routes(self.route_file)
        routes = data.get('routes', {})
        if route_name not in routes:
            raise ValueError(f'Route "{route_name}" was not found.')
        return routes[route_name]

    def send_go_to(self, pose, route_name, waypoint_name):
        if not self.go_to_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose is not available.')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f'Going to {route_name}/{waypoint_name}...')
        return self.wait_for_action(self.go_to_client, goal)

    def send_follow(self, poses, route_name):
        if not self.follow_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_through_poses is not available.')
            return False

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        self.get_logger().info(
            f'Following {route_name} with {len(poses)} waypoints...')
        return self.wait_for_action(self.follow_client, goal)

    def wait_for_action(self, client, goal):
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Navigation failed with status {result.status}.')
            return False

        self.get_logger().info('Navigation finished.')
        return True


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--route-file',
        default=default_routes_file(),
        help='Path to routes.yaml',
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    subparsers.add_parser('list')

    go_to = subparsers.add_parser('go_to')
    go_to.add_argument('route')
    go_to.add_argument('waypoint')

    follow = subparsers.add_parser('follow')
    follow.add_argument('route')
    follow.add_argument('--start')
    follow.add_argument('--goal')

    go_to_nearest = subparsers.add_parser('go_to_nearest')
    go_to_nearest.add_argument('route')

    follow_nearest = subparsers.add_parser('follow_nearest')
    follow_nearest.add_argument('route')
    follow_nearest.add_argument('--goal')

    preview_nearest = subparsers.add_parser('preview_nearest')
    preview_nearest.add_argument('route')
    preview_nearest.add_argument('--goal')

    return parser.parse_args(argv)


def main(args=None):
    argv = remove_ros_args(args=sys.argv)[1:] if args is None else args
    parsed = parse_args(argv)

    rclpy.init(args=[])
    node = RouteCli(parsed.route_file)
    ok = False
    try:
        if parsed.command == 'list':
            ok = node.list_routes()
        elif parsed.command == 'go_to':
            ok = node.go_to(parsed.route, parsed.waypoint)
        elif parsed.command == 'follow':
            ok = node.follow(parsed.route, parsed.start, parsed.goal)
        elif parsed.command == 'go_to_nearest':
            ok = node.go_to_nearest(parsed.route)
        elif parsed.command == 'follow_nearest':
            ok = node.follow_nearest(parsed.route, parsed.goal)
        elif parsed.command == 'preview_nearest':
            ok = node.preview_nearest(parsed.route, parsed.goal)
    except ValueError as ex:
        node.get_logger().error(str(ex))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
