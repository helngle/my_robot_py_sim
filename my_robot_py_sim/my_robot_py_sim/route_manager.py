import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_py_sim.route_utils import default_routes_file, load_routes


class RouteManager(Node):
    def __init__(self):
        super().__init__('route_manager')

        self.declare_parameter('route_file', default_routes_file())
        self.declare_parameter('marker_topic', '/route_markers')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('active_route', '__none__')
        self.declare_parameter('active_routes', '__none__')
        self.declare_parameter('show_points', False)
        self.declare_parameter('show_labels', False)

        self.route_file = self.get_parameter('route_file').value
        marker_topic = self.get_parameter('marker_topic').value
        publish_rate = self.get_parameter('publish_rate').value

        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_timer(1.0 / max(publish_rate, 0.1), self.publish_markers)
        self.get_logger().info(f'Route markers from {self.route_file}')

    def publish_markers(self):
        data = load_routes(self.route_file)
        active_routes = self.selected_routes()
        show_points = self.get_parameter('show_points').value
        show_labels = self.get_parameter('show_labels').value
        marker_array = MarkerArray()
        marker_id = 0

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for route_name, route in data.get('routes', {}).items():
            if active_routes is not None and route_name not in active_routes:
                continue

            frame_id = route.get('frame_id', 'map')
            waypoints = route.get('waypoints', [])
            if not waypoints:
                continue

            marker_id = self.add_line_marker(
                marker_array, marker_id, route_name, frame_id, waypoints)
            if show_points:
                for waypoint in waypoints:
                    marker_id = self.add_point_marker(
                        marker_array, marker_id, route_name, frame_id, waypoint)
            if show_labels:
                for index, waypoint in enumerate(waypoints):
                    marker_id = self.add_text_marker(
                        marker_array, marker_id, route_name, frame_id,
                        waypoint, index)

        self.marker_pub.publish(marker_array)

    def selected_routes(self):
        active_routes = self.get_parameter('active_routes').value
        active_route = self.get_parameter('active_route').value

        selection = active_routes if active_routes != '__none__' else active_route
        if selection == '':
            return None
        if selection == '__none__':
            return set()

        return {
            route_name.strip()
            for route_name in str(selection).split(',')
            if route_name.strip()
        }

    def add_line_marker(self, marker_array, marker_id, route_name,
                        frame_id, waypoints):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f'{route_name}_line'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r = 0.95
        marker.color.g = 0.15
        marker.color.b = 0.10
        marker.color.a = 0.95
        for waypoint in waypoints:
            point = Point()
            point.x = float(waypoint['x'])
            point.y = float(waypoint['y'])
            point.z = float(waypoint.get('z', 0.03))
            marker.points.append(point)
        marker_array.markers.append(marker)
        return marker_id + 1

    def add_point_marker(self, marker_array, marker_id, route_name,
                         frame_id, waypoint):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f'{route_name}_points'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(waypoint['x'])
        marker.pose.position.y = float(waypoint['y'])
        marker.pose.position.z = float(waypoint.get('z', 0.05))
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 0.05
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker_array.markers.append(marker)
        return marker_id + 1

    def add_text_marker(self, marker_array, marker_id, route_name,
                        frame_id, waypoint, index):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f'{route_name}_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(waypoint['x'])
        marker.pose.position.y = float(waypoint['y'])
        marker.pose.position.z = float(waypoint.get('z', 0.25))
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.text = f'{index}: {waypoint.get("name", index)}'
        marker_array.markers.append(marker)
        return marker_id + 1


def main(args=None):
    rclpy.init(args=args)
    node = RouteManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
