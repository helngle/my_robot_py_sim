import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudRestamper(Node):
    def __init__(self):
        super().__init__('pointcloud_restamper')
        self.declare_parameter(
            'input_topic',
            '/vmr_base_bridge/laser/points',
        )
        self.declare_parameter(
            'output_topic',
            '/vmr_base_bridge/laser/points_stamped',
        )

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.handle_pointcloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Restamping point clouds from {input_topic} to {output_topic}'
        )

    def handle_pointcloud(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRestamper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
