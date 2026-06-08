import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputeRoute, NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node


class RouteCommander(Node):
    def __init__(self):
        super().__init__('route_commander')

        self.declare_parameter('start_id', 0)
        self.declare_parameter('goal_id', 4)
        self.declare_parameter('compute_route_action', 'compute_route')
        self.declare_parameter('navigate_action', 'navigate_through_poses')
        self.declare_parameter('follow_route', True)
        self.declare_parameter('route_frame', 'map')
        self.declare_parameter('waypoint_spacing', 0.4)

        self.start_id = self.get_parameter('start_id').value
        self.goal_id = self.get_parameter('goal_id').value
        compute_action = self.get_parameter('compute_route_action').value
        navigate_action = self.get_parameter('navigate_action').value
        self.follow_route = self.get_parameter('follow_route').value
        self.route_frame = self.get_parameter('route_frame').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value

        self.compute_client = ActionClient(self, ComputeRoute, compute_action)
        self.navigate_client = ActionClient(
            self, NavigateThroughPoses, navigate_action)

    def run(self):
        self.get_logger().info(
            f'Requesting saved nav2_route path: start_id={self.start_id}, '
            f'goal_id={self.goal_id}'
        )
        if not self.compute_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'compute_route action server is not available.')
            return False

        goal = ComputeRoute.Goal()
        goal.start_id = int(self.start_id)
        goal.goal_id = int(self.goal_id)
        goal.use_start = False
        goal.use_poses = False

        send_future = self.compute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('compute_route goal was rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_response = result_future.result()
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'compute_route failed with status {result_response.status}.')
            return False

        route_result = result_response.result
        poses = self.prepare_route_poses(
            route_result.path.poses,
            route_result.path.header.frame_id,
        )
        self.get_logger().info(
            f'Route ready: {len(route_result.route.nodes)} nodes, '
            f'{len(route_result.route.edges)} edges, {len(poses)} path poses.'
        )

        if not self.follow_route:
            self.get_logger().info(
                'follow_route is false, only computed the saved route.')
            return True

        if len(poses) < 2:
            self.get_logger().error(
                'The computed route has too few poses to execute.')
            return False

        return self.navigate_through_poses(poses)

    def prepare_route_poses(self, route_poses, path_frame):
        frame_id = path_frame or self.route_frame or 'map'
        stamped_poses = []
        stamp = self.get_clock().now().to_msg()

        for pose in route_poses:
            pose.header.frame_id = pose.header.frame_id or frame_id
            pose.header.stamp = stamp
            stamped_poses.append(pose)

        sparse_poses = self.downsample_poses(stamped_poses)
        if stamped_poses and sparse_poses[-1] is not stamped_poses[-1]:
            sparse_poses.append(stamped_poses[-1])

        self.get_logger().info(
            f'Prepared {len(sparse_poses)} Nav2 waypoints in {frame_id}.')
        return sparse_poses

    def downsample_poses(self, poses):
        if len(poses) <= 2 or self.waypoint_spacing <= 0.0:
            return poses

        sparse_poses = [poses[0]]
        last = poses[0].pose.position
        for pose in poses[1:]:
            current = pose.pose.position
            distance = (
                (current.x - last.x) ** 2 +
                (current.y - last.y) ** 2
            ) ** 0.5
            if distance >= self.waypoint_spacing:
                sparse_poses.append(pose)
                last = current

        return sparse_poses

    def navigate_through_poses(self, poses):
        if not self.navigate_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'navigate_through_poses action server is not available.')
            return False

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses

        self.get_logger().info(f'Sending {len(poses)} route poses to Nav2.')
        send_future = self.navigate_client.send_goal_async(
            goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                'navigate_through_poses goal was rejected.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_response = result_future.result()
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            status = result_response.status
            self.get_logger().error(
                f'navigate_through_poses failed with status {status}.'
            )
            return False

        self.get_logger().info('Saved route navigation finished.')
        return True

    def feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Route remaining: poses={feedback.number_of_poses_remaining}, '
            f'distance={feedback.distance_remaining:.2f} m'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RouteCommander()
    ok = False
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
