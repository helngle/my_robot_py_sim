#!/usr/bin/python3

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from vmr_base_bridge.msg import VmrLocation
from vmr_base_bridge.srv import CancelTask
from vmr_base_bridge.srv import QueryTaskStatus
from vmr_base_bridge.srv import StepMove
from vmr_base_bridge.srv import VectorMove


class TaskModeProbe(Node):
    def __init__(self, args):
        super().__init__("task_mode_probe")
        self._args = args
        self._step_move_client = self.create_client(StepMove, args.step_move_service)
        self._vector_move_client = self.create_client(VectorMove, args.vector_move_service)
        self._query_client = self.create_client(QueryTaskStatus, args.query_status_service)
        self._cancel_client = self.create_client(CancelTask, args.cancel_task_service)
        self._latest_pose = None
        self._latest_location = None
        self.create_subscription(PoseStamped, args.pose_topic, self._pose_cb, 10)
        self.create_subscription(VmrLocation, args.location_topic, self._location_cb, 10)

    def _pose_cb(self, msg):
        self._latest_pose = msg

    def _location_cb(self, msg):
        self._latest_location = msg

    def wait_for_services(self):
        for client in (
            self._step_move_client,
            self._vector_move_client,
            self._query_client,
            self._cancel_client,
        ):
            self.get_logger().info(f"Waiting for service: {client.srv_name}")
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"Service unavailable: {client.srv_name}")
                return False
        return True

    def wait_for_pose_sample(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_pose is not None or self._latest_location is not None:
                return

    def pose_summary(self):
        if self._latest_pose is not None:
            p = self._latest_pose.pose.position
            q = self._latest_pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return f"pose x={p.x:.3f}, y={p.y:.3f}, yaw={yaw:.3f}"
        if self._latest_location is not None:
            loc = self._latest_location
            return f"location x={loc.x:.3f}, y={loc.y:.3f}, theta={loc.theta:.3f}, status={loc.status}"
        return "pose unavailable"

    def send_step(self):
        request = StepMove.Request()
        request.direction = self._args.direction
        request.value = self._args.value
        self.get_logger().info(f"Sending StepMove: direction={request.direction}, value={request.value:.3f}")
        return self._call_task_service(self._step_move_client, request)

    def send_vector(self):
        request = VectorMove.Request()
        request.distance = self._args.distance
        request.angle = self._args.angle
        self.get_logger().info(f"Sending VectorMove: distance={request.distance:.3f}, angle={request.angle:.3f}")
        return self._call_task_service(self._vector_move_client, request)

    def _call_task_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("Task service call failed")
            return None
        if not response.success:
            self.get_logger().error(f"Task rejected: {response.message}")
            return None
        self.get_logger().info(f"Task accepted: task_id={response.task_id}")
        return response.task_id

    def wait_for_task(self, task_id):
        deadline = time.monotonic() + self._args.timeout
        last_result = None
        while rclpy.ok() and time.monotonic() < deadline:
            request = QueryTaskStatus.Request()
            request.task_id = task_id
            future = self._query_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            self.wait_for_pose_sample(timeout=0.1)
            if response is None:
                self.get_logger().error("QueryTaskStatus service call failed")
                return False
            if not response.success:
                self.get_logger().error(f"QueryTaskStatus failed: {response.message}")
                return False

            last_result = response.task_result
            self.get_logger().info(
                f"status task_flag={response.task_flag}, task_result={response.task_result}, "
                f"{self.pose_summary()}"
            )
            if response.task_result == self._args.success_result:
                return True
            if response.task_result not in self._args.running_results:
                return False
            time.sleep(self._args.poll_interval)

        self.get_logger().error(f"Task timeout: last_result={last_result}, task_id={task_id}")
        if self._args.cancel_on_timeout:
            self.cancel_task(task_id)
        return False

    def cancel_task(self, task_id):
        request = CancelTask.Request()
        request.task_id = task_id
        self.get_logger().warn(f"Canceling timed out task: task_id={task_id}")
        future = self._cancel_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("CancelTask service call failed")
            return False
        if not response.success:
            self.get_logger().error(f"CancelTask rejected: {response.message}")
            return False
        self.get_logger().info(response.message)
        return True

    def run(self):
        if not self.wait_for_services():
            return 1
        self.wait_for_pose_sample()
        self.get_logger().info(f"Before task: {self.pose_summary()}")
        task_id = self.send_vector() if self._args.mode == "vector" else self.send_step()
        if not task_id:
            return 1
        ok = self.wait_for_task(task_id)
        self.wait_for_pose_sample()
        self.get_logger().info(f"After task: {self.pose_summary()}")
        return 0 if ok else 1


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Probe VMR task-mode motion without using /cmd_vel.")
    parser.add_argument("--mode", choices=("step", "vector"), default="step")
    parser.add_argument("--direction", type=int, default=0, help="StepMove direction, 0 forward, 1 backward, 2 left, 3 right, 4 cw, 5 ccw")
    parser.add_argument("--value", type=float, default=0.2, help="StepMove distance in m or rotation in rad")
    parser.add_argument("--distance", type=float, default=0.2, help="VectorMove distance in m")
    parser.add_argument("--angle", type=float, default=0.0, help="VectorMove angle in rad, relative to robot forward")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--poll-interval", type=float, default=0.5)
    parser.add_argument("--success-result", type=int, default=0)
    parser.add_argument("--running-results", type=int, nargs="*", default=[-1])
    parser.add_argument("--cancel-on-timeout", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--step-move-service", default="/vmr_base_bridge/step_move")
    parser.add_argument("--vector-move-service", default="/vmr_base_bridge/vector_move")
    parser.add_argument("--query-status-service", default="/vmr_base_bridge/query_task_status")
    parser.add_argument("--cancel-task-service", default="/vmr_base_bridge/cancel_task")
    parser.add_argument("--pose-topic", default="/vmr_base_bridge/pose")
    parser.add_argument("--location-topic", default="/vmr_base_bridge/location")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = TaskModeProbe(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
