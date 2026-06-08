import math
import os
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import yaml


def quaternion_from_yaw(yaw):
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class RoutePlayer(Node):
    def __init__(self):
        super().__init__('route_player')
        self.declare_parameter('route_file', '')
        self.declare_parameter('route_name', 'door_test_route')
        self.declare_parameter('retry_count', -1)
        self.declare_parameter('retry_delay', -1.0)
        self.declare_parameter('clear_costmaps_on_failure', True)

        self.route_name = self.get_parameter('route_name').value
        self.clear_on_failure = bool(self.get_parameter('clear_costmaps_on_failure').value)
        self.route = self.load_route()
        self.retry_count = self.get_retry_count()
        self.retry_delay = self.get_retry_delay()

        self.nav_client = ActionClient(
            self,
            NavigateThroughPoses,
            'navigate_through_poses',
        )
        self.clear_global_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
        )
        self.clear_local_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
        )

    def load_route(self):
        route_file = self.get_parameter('route_file').value
        if not route_file:
            route_file = os.path.join(
                get_package_share_directory('my_robot_py_sim'),
                'config',
                'routes.yaml',
            )

        with open(route_file, 'r') as stream:
            config = yaml.safe_load(stream) or {}

        routes = config.get('routes', {})
        if self.route_name not in routes:
            names = ', '.join(sorted(routes.keys())) or '<none>'
            raise RuntimeError(
                f'Route "{self.route_name}" not found in {route_file}. '
                f'Available routes: {names}'
            )

        route = routes[self.route_name]
        waypoints = route.get('waypoints', [])
        if not waypoints:
            raise RuntimeError(f'Route "{self.route_name}" has no waypoints')
        return route

    def get_retry_count(self):
        override = int(self.get_parameter('retry_count').value)
        if override >= 0:
            return override
        return int(self.route.get('retry_count', 3))

    def get_retry_delay(self):
        override = float(self.get_parameter('retry_delay').value)
        if override >= 0.0:
            return override
        return float(self.route.get('retry_delay', 1.0))

    def make_goal(self):
        frame_id = self.route.get('frame_id', 'map')
        goal = NavigateThroughPoses.Goal()
        goal.poses = []
        for waypoint in self.route['waypoints']:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(waypoint['x'])
            pose.pose.position.y = float(waypoint['y'])
            pose.pose.position.z = float(waypoint.get('z', 0.0))
            qx, qy, qz, qw = quaternion_from_yaw(float(waypoint.get('yaw', 0.0)))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            goal.poses.append(pose)
        return goal

    def run(self):
        self.get_logger().info(
            f'Waiting for Nav2 action server with route "{self.route_name}"'
        )
        self.nav_client.wait_for_server()

        for attempt in range(1, self.retry_count + 2):
            status = self.send_route(attempt)
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f'Route "{self.route_name}" completed')
                return True

            self.get_logger().warn(
                f'Route "{self.route_name}" failed with status {status} '
                f'on attempt {attempt}/{self.retry_count + 1}'
            )
            if attempt > self.retry_count:
                break
            if self.clear_on_failure:
                self.clear_costmaps()
            time.sleep(self.retry_delay)

        self.get_logger().error(f'Route "{self.route_name}" could not be completed')
        return False

    def send_route(self, attempt):
        goal = self.make_goal()
        self.get_logger().info(
            f'Sending {len(goal.poses)} waypoints for "{self.route_name}" '
            f'(attempt {attempt}/{self.retry_count + 1})'
        )
        send_future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.handle_feedback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the route goal')
            return GoalStatus.STATUS_ABORTED

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            return GoalStatus.STATUS_UNKNOWN
        return result.status

    def handle_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'Distance remaining: {feedback.distance_remaining:.2f} m, '
            f'poses remaining: {feedback.number_of_poses_remaining}'
        )

    def clear_costmaps(self):
        for name, client in (
            ('global', self.clear_global_client),
            ('local', self.clear_local_client),
        ):
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'{name} costmap clear service is not ready')
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result() is not None:
                self.get_logger().info(f'Cleared {name} costmap')
            else:
                self.get_logger().warn(f'Failed to clear {name} costmap')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoutePlayer()
        ok = node.run()
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(exc)
        ok = False
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
