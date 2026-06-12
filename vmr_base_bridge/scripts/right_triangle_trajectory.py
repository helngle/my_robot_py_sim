#!/usr/bin/env python3

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node

from vmr_base_bridge.srv import QueryTaskStatus
from vmr_base_bridge.srv import StepMove
from vmr_base_bridge.srv import VectorMove


class RightTriangleTrajectory(Node):
    def __init__(self, args):
        super().__init__("right_triangle_trajectory")
        self._step_move_client = self.create_client(StepMove, args.step_move_service)
        self._vector_move_client = self.create_client(VectorMove, args.vector_move_service)
        self._query_client = self.create_client(QueryTaskStatus, args.query_status_service)
        self._poll_interval = args.poll_interval
        self._timeout = args.timeout

    def wait_for_services(self):
        for client, service_name in (
            (self._step_move_client, self._step_move_client.srv_name),
            (self._vector_move_client, self._vector_move_client.srv_name),
            (self._query_client, self._query_client.srv_name),
        ):
            self.get_logger().info(f"Waiting for service: {service_name}")
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"Service unavailable: {service_name}")
                return False
        return True

    def step_move(self, direction, value, label):
        request = StepMove.Request()
        request.direction = direction
        request.value = value

        self.get_logger().info(f"Sending {label}: direction={direction}, value={value:.3f}")
        future = self._step_move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f"{label} service call failed")
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(f"{label} rejected: {response.message}")
            return False

        self.get_logger().info(f"{label} accepted: task_id={response.task_id}")
        return self.wait_for_task(response.task_id, label)

    def vector_move(self, distance, angle, label):
        request = VectorMove.Request()
        request.distance = distance
        request.angle = angle

        self.get_logger().info(f"Sending {label}: distance={distance:.3f}, angle={angle:.3f}")
        future = self._vector_move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f"{label} service call failed")
            return False

        response = future.result()
        if not response.success:
            self.get_logger().error(f"{label} rejected: {response.message}")
            return False

        self.get_logger().info(f"{label} accepted: task_id={response.task_id}")
        return self.wait_for_task(response.task_id, label)

    def wait_for_task(self, task_id, label):
        deadline = time.monotonic() + self._timeout
        while rclpy.ok() and time.monotonic() < deadline:
            request = QueryTaskStatus.Request()
            request.task_id = task_id
            future = self._query_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            response = future.result()
            if response is None or not response.success:
                message = response.message if response is not None else "query service call failed"
                self.get_logger().error(f"{label} status query failed: {message}")
                return False

            self.get_logger().info(
                f"{label} status: task_id={response.task_id_echo}, "
                f"task_result={response.task_result}, message={response.message}"
            )
            if response.task_result == 0:
                return True
            if response.task_result != -1:
                return False

            time.sleep(self._poll_interval)

        self.get_logger().error(f"{label} timeout: task_id={task_id}")
        return False

    def run_triangle(self, leg_length):
        hypotenuse = leg_length * math.sqrt(2.0)

        return (
            self.step_move(StepMove.Request.DIR_FORWARD, leg_length, "leg_1") and
            self.vector_move(hypotenuse, -3.0 * math.pi / 4.0, "diagonal") and
            self.step_move(StepMove.Request.DIR_LEFT, leg_length, "leg_2")
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Drive an isosceles right triangle trajectory from the current right-angle vertex."
    )
    parser.add_argument("--leg-length", type=float, default=1.3)
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--poll-interval", type=float, default=0.5)
    parser.add_argument("--step-move-service", default="/vmr_base_bridge/step_move")
    parser.add_argument("--vector-move-service", default="/vmr_base_bridge/vector_move")
    parser.add_argument("--query-status-service", default="/vmr_base_bridge/query_task_status")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    rclpy.init()
    node = RightTriangleTrajectory(args)
    try:
        if not node.wait_for_services():
            return 1
        if not node.run_triangle(args.leg_length):
            return 1
        node.get_logger().info("Right triangle trajectory completed")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
