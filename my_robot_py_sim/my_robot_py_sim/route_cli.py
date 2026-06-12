import argparse
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from my_robot_py_sim.route_utils import (
    default_routes_file,
    load_routes,
    nearest_waypoint_name,
    route_segment_waypoints,
    route_to_poses,
    save_routes,
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

    def route_info(self, route_name):
        route = self.get_route(route_name)
        waypoints = route.get('waypoints', [])
        frame_id = route.get('frame_id', 'map')
        distance = self.route_distance(waypoints)

        self.get_logger().info(f'Route: {route_name}')
        self.get_logger().info(f'Frame: {frame_id}')
        self.get_logger().info(f'Waypoints: {len(waypoints)}')
        self.get_logger().info(f'Length: {distance:.2f} m')

        if not waypoints:
            return True

        start = waypoints[0]
        goal = waypoints[-1]
        self.get_logger().info(
            'Start: '
            f'{start.get("name", 0)} '
            f'x={float(start["x"]):.2f}, y={float(start["y"]):.2f}'
        )
        self.get_logger().info(
            'Goal: '
            f'{goal.get("name", len(waypoints) - 1)} '
            f'x={float(goal["x"]):.2f}, y={float(goal["y"]):.2f}'
        )
        self.get_logger().info(
            'Waypoint sequence: ' + ' -> '.join(
                waypoint.get('name', str(index))
                for index, waypoint in enumerate(waypoints)
            )
        )
        return True

    def rename_route(self, old_name, new_name, overwrite=False):
        if old_name == new_name:
            raise ValueError('Old and new route names are the same.')

        data = load_routes(self.route_file)
        routes = data.get('routes', {})
        if old_name not in routes:
            raise ValueError(f'Route "{old_name}" was not found.')
        if new_name in routes and not overwrite:
            raise ValueError(
                f'Route "{new_name}" already exists. Use --overwrite '
                'if you want to replace it.'
            )

        routes[new_name] = routes.pop(old_name)
        save_routes(self.route_file, data)
        self.get_logger().info(
            f'Renamed route "{old_name}" to "{new_name}" in '
            f'{self.route_file}'
        )
        return True

    def delete_route(self, route_name, yes=False):
        if not yes:
            raise ValueError(
                f'Delete route "{route_name}" requires --yes. '
                'This prevents accidental route loss.'
            )

        data = load_routes(self.route_file)
        routes = data.get('routes', {})
        if route_name not in routes:
            raise ValueError(f'Route "{route_name}" was not found.')

        del routes[route_name]
        save_routes(self.route_file, data)
        self.get_logger().info(
            f'Deleted route "{route_name}" from {self.route_file}')
        return True

    def display_route(self, route_names):
        for route_name in route_names:
            self.get_route(route_name)
        return self.set_route_manager_active_routes(route_names)

    def display_all_routes(self):
        return self.set_route_manager_active_routes([])

    def display_no_routes(self):
        return self.set_route_manager_active_routes(None)

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

    def check_nav(self):
        ok = True
        self.warm_tf_buffer()
        checks = [
            self.check_topic('/map', OccupancyGrid, self.map_qos()),
            self.check_topic('/odom', Odometry),
            self.check_topic('/scan', LaserScan, qos_profile_sensor_data),
            self.check_transform('odom', 'base_footprint'),
            self.check_transform('map', 'base_footprint'),
            self.check_action(self.go_to_client, 'navigate_to_pose'),
            self.check_action(self.follow_client, 'navigate_through_poses'),
        ]
        for check_ok in checks:
            ok = ok and check_ok

        if ok:
            self.get_logger().info('Navigation route checks passed.')
        else:
            self.get_logger().error(
                'Navigation route checks failed. Start navigation first, set '
                'the RViz initial pose, then check /map, /odom, /scan, and TF.'
            )
        return ok

    def find_nearest_name(self, route):
        fixed_frame = route.get('frame_id', 'map')
        transform = self.lookup_robot_pose(fixed_frame)
        translation = transform.transform.translation
        return nearest_waypoint_name(route, translation.x, translation.y)

    def lookup_robot_pose(self, fixed_frame):
        self.warm_tf_buffer()
        deadline = time.monotonic() + 10.0
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

    def warm_tf_buffer(self):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def check_topic(self, topic_name, msg_type, qos_profile=10):
        result = {'received': False}

        def callback(_msg):
            result['received'] = True

        subscription = self.create_subscription(
            msg_type, topic_name, callback, qos_profile)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not result['received']:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(subscription)
        if result['received']:
            self.get_logger().info(f'{topic_name}: OK')
            return True

        self.get_logger().error(f'{topic_name}: no message received')
        return False

    @staticmethod
    def map_qos():
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    def check_transform(self, target_frame, source_frame):
        deadline = time.monotonic() + 10.0
        last_error = None
        while time.monotonic() < deadline:
            try:
                self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
                self.get_logger().info(
                    f'TF {target_frame} -> {source_frame}: OK')
                return True
            except TransformException as ex:
                last_error = ex
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            f'TF {target_frame} -> {source_frame}: {last_error}')
        return False

    def check_action(self, client, action_name):
        if client.wait_for_server(timeout_sec=3.0):
            self.get_logger().info(f'{action_name}: OK')
            return True

        self.get_logger().error(f'{action_name}: action server unavailable')
        return False

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

    def set_route_manager_active_routes(self, route_names):
        client = self.create_client(SetParameters, '/route_manager/set_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                '/route_manager/set_parameters is not available. '
                'Start navigation_with_rviz.launch.py first.'
            )
            return False

        parameter = Parameter()
        parameter.name = 'active_routes'
        parameter.value.type = ParameterType.PARAMETER_STRING
        if route_names is None:
            parameter.value.string_value = '__none__'
        else:
            parameter.value.string_value = ','.join(route_names)

        request = SetParameters.Request()
        request.parameters = [parameter]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.results:
            self.get_logger().error('No response from route_manager.')
            return False

        result = response.results[0]
        if not result.successful:
            self.get_logger().error(
                f'route_manager rejected active_routes: {result.reason}')
            return False

        if route_names is None:
            self.get_logger().info('RViz route display: none')
        elif route_names:
            self.get_logger().info(
                'RViz route display: ' + ', '.join(route_names))
        else:
            self.get_logger().info('RViz route display: all routes')
        return True

    @staticmethod
    def route_distance(waypoints):
        distance = 0.0
        for first, second in zip(waypoints, waypoints[1:]):
            distance += (
                (float(first['x']) - float(second['x'])) ** 2 +
                (float(first['y']) - float(second['y'])) ** 2
            ) ** 0.5
        return distance


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--route-file',
        default=default_routes_file(),
        help='Path to routes.yaml',
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    subparsers.add_parser('list')

    info = subparsers.add_parser('info')
    info.add_argument('route')

    rename = subparsers.add_parser('rename')
    rename.add_argument('old_name')
    rename.add_argument('new_name')
    rename.add_argument('--overwrite', action='store_true')

    delete = subparsers.add_parser('delete')
    delete.add_argument('route')
    delete.add_argument('--yes', action='store_true')

    display = subparsers.add_parser('display')
    display.add_argument('routes', nargs='+')

    subparsers.add_parser('display_all')
    subparsers.add_parser('display_none')

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

    subparsers.add_parser('check_nav')

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
        elif parsed.command == 'info':
            ok = node.route_info(parsed.route)
        elif parsed.command == 'rename':
            ok = node.rename_route(
                parsed.old_name, parsed.new_name, parsed.overwrite)
        elif parsed.command == 'delete':
            ok = node.delete_route(parsed.route, parsed.yes)
        elif parsed.command == 'display':
            ok = node.display_route(parsed.routes)
        elif parsed.command == 'display_all':
            ok = node.display_all_routes()
        elif parsed.command == 'display_none':
            ok = node.display_no_routes()
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
        elif parsed.command == 'check_nav':
            ok = node.check_nav()
    except ValueError as ex:
        node.get_logger().error(str(ex))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
