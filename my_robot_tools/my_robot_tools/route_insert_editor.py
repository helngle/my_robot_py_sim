import math
import os

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from my_robot_tools.route_utils import (
    default_routes_file,
    load_routes,
    save_routes,
)


class RouteInsertEditor(Node):
    def __init__(self):
        super().__init__('route_insert_editor')

        self.declare_parameter('route_file', default_routes_file())
        self.declare_parameter('source_route', 'recorded_route')
        self.declare_parameter('output_route', '')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('marker_topic', '/route_edit_markers')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('insert_prefix', 'manual')
        self.declare_parameter('overwrite_source', False)
        self.declare_parameter('show_source_points', False)
        self.declare_parameter('show_labels', False)

        self.route_file = os.path.expanduser(
            self.get_parameter('route_file').value)
        self.source_route = self.get_parameter('source_route').value
        output_route = self.get_parameter('output_route').value
        self.overwrite_source = self.get_parameter('overwrite_source').value
        self.output_route = output_route or f'{self.source_route}_edit'
        if self.overwrite_source:
            self.output_route = self.source_route

        self.insert_prefix = self.get_parameter('insert_prefix').value
        self.show_source_points = self.get_parameter('show_source_points').value
        self.show_labels = self.get_parameter('show_labels').value
        marker_topic = self.get_parameter('marker_topic').value
        clicked_point_topic = self.get_parameter('clicked_point_topic').value
        publish_rate = self.get_parameter('publish_rate').value

        data = load_routes(self.route_file)
        routes = data.get('routes', {})
        if self.source_route not in routes:
            raise RuntimeError(
                f'Route "{self.source_route}" was not found in '
                f'{self.route_file}')

        source = routes[self.source_route]
        self.frame_id = source.get('frame_id', 'map')
        self.waypoints = [
            dict(waypoint) for waypoint in source.get('waypoints', [])
        ]
        if len(self.waypoints) < 2:
            raise RuntimeError(
                f'Route "{self.source_route}" needs at least 2 waypoints.')

        self.insert_count = 0
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_subscription(
            PointStamped, clicked_point_topic, self.handle_clicked_point, 10)
        self.create_timer(1.0 / max(publish_rate, 0.1), self.publish_markers)

        self.get_logger().info(
            f'Editing route "{self.source_route}" from {self.route_file}')
        self.get_logger().info(
            f'RViz Publish Point -> insert waypoint; save target is '
            f'"{self.output_route}". Press Ctrl+C to save.')

    def handle_clicked_point(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.get_logger().warn(
                f'Clicked point frame is "{msg.header.frame_id}", route frame '
                f'is "{self.frame_id}". Inserting using raw coordinates.')

        insert_after = self.nearest_segment_index(
            msg.point.x,
            msg.point.y,
        )
        yaw = self.segment_yaw(insert_after)
        waypoint = {
            'name': self.next_manual_name(),
            'x': float(msg.point.x),
            'y': float(msg.point.y),
            'yaw': float(yaw),
        }
        self.waypoints.insert(insert_after + 1, waypoint)
        self.get_logger().info(
            f'Inserted {waypoint["name"]} after index {insert_after}: '
            f'x={waypoint["x"]:.2f}, y={waypoint["y"]:.2f}')
        self.publish_markers()

    def next_manual_name(self):
        existing = {
            str(waypoint.get('name', '')) for waypoint in self.waypoints
        }
        while True:
            name = f'{self.insert_prefix}_{self.insert_count}'
            self.insert_count += 1
            if name not in existing:
                return name

    def nearest_segment_index(self, x, y):
        best_index = 0
        best_distance = float('inf')
        for index in range(len(self.waypoints) - 1):
            first = self.waypoints[index]
            second = self.waypoints[index + 1]
            distance = self.point_to_segment_distance(
                x, y,
                float(first['x']), float(first['y']),
                float(second['x']), float(second['y']),
            )
            if distance < best_distance:
                best_distance = distance
                best_index = index
        return best_index

    @staticmethod
    def point_to_segment_distance(px, py, ax, ay, bx, by):
        dx = bx - ax
        dy = by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-9:
            return math.hypot(px - ax, py - ay)

        t = ((px - ax) * dx + (py - ay) * dy) / length_sq
        t = min(1.0, max(0.0, t))
        closest_x = ax + t * dx
        closest_y = ay + t * dy
        return math.hypot(px - closest_x, py - closest_y)

    def segment_yaw(self, index):
        first = self.waypoints[index]
        second = self.waypoints[index + 1]
        return math.atan2(
            float(second['y']) - float(first['y']),
            float(second['x']) - float(first['x']),
        )

    def save(self):
        data = load_routes(self.route_file)
        data.setdefault('routes', {})
        data['routes'][self.output_route] = {
            'frame_id': self.frame_id,
            'waypoints': self.renumber_waypoints(self.waypoints),
        }
        save_routes(self.route_file, data)
        self.get_logger().info(
            f'Saved route "{self.output_route}" with '
            f'{len(self.waypoints)} waypoints to {self.route_file}')
        return True

    def renumber_waypoints(self, waypoints):
        renamed = []
        for index, waypoint in enumerate(waypoints):
            item = dict(waypoint)
            name = str(item.get('name', ''))
            if name.startswith(self.insert_prefix):
                item['name'] = f'{self.insert_prefix}_{index}'
            elif name.startswith('wp_'):
                item['name'] = f'wp_{index}'
            renamed.append(item)
        return renamed

    def publish_markers(self):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_id = 0
        marker_id = self.add_line_marker(marker_array, marker_id)
        for index, waypoint in enumerate(self.waypoints):
            is_inserted = str(waypoint.get('name', '')).startswith(
                self.insert_prefix)
            if self.show_source_points or is_inserted:
                marker_id = self.add_point_marker(
                    marker_array, marker_id, waypoint)
            if self.show_labels:
                marker_id = self.add_text_marker(
                    marker_array, marker_id, waypoint, index)

        self.marker_pub.publish(marker_array)

    def add_line_marker(self, marker_array, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'route_edit_line'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.06
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.25
        marker.color.a = 0.95
        for waypoint in self.waypoints:
            point = Point()
            point.x = float(waypoint['x'])
            point.y = float(waypoint['y'])
            point.z = float(waypoint.get('z', 0.07))
            marker.points.append(point)
        marker_array.markers.append(marker)
        return marker_id + 1

    def add_point_marker(self, marker_array, marker_id, waypoint):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'route_edit_points'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(waypoint['x'])
        marker.pose.position.y = float(waypoint['y'])
        marker.pose.position.z = float(waypoint.get('z', 0.10))
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        if str(waypoint.get('name', '')).startswith(self.insert_prefix):
            marker.color.r = 1.0
            marker.color.g = 0.82
            marker.color.b = 0.05
        else:
            marker.color.r = 0.05
            marker.color.g = 0.35
            marker.color.b = 1.0
        marker.color.a = 0.95
        marker_array.markers.append(marker)
        return marker_id + 1

    def add_text_marker(self, marker_array, marker_id, waypoint, index):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'route_edit_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(waypoint['x'])
        marker.pose.position.y = float(waypoint['y'])
        marker.pose.position.z = float(waypoint.get('z', 0.35))
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
    node = RouteInsertEditor()
    ok = False
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
