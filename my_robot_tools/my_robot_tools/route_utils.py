import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped


def default_routes_file():
    env_file = os.environ.get('MY_ROBOT_ROUTES_FILE')
    if env_file:
        return os.path.expanduser(env_file)

    source_file = os.path.expanduser(
        '~/ros2_ws/src/my_robot_navigation/config/routes.yaml')
    if os.path.exists(source_file):
        return source_file

    share_dir = get_package_share_directory('my_robot_navigation')
    return os.path.join(share_dir, 'config', 'routes.yaml')


def load_routes(path):
    if not os.path.exists(path):
        return {'routes': {}}

    with open(path, 'r', encoding='utf-8') as route_file:
        data = yaml.safe_load(route_file) or {}

    if 'routes' not in data or data['routes'] is None:
        data['routes'] = {}
    return data


def save_routes(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as route_file:
        yaml.safe_dump(data, route_file, sort_keys=False)


def yaw_to_quaternion(yaw):
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw * 0.5),
        'w': math.cos(yaw * 0.5),
    }


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def waypoint_to_pose(waypoint, frame_id):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(waypoint['x'])
    pose.pose.position.y = float(waypoint['y'])
    pose.pose.position.z = float(waypoint.get('z', 0.0))
    quat = yaw_to_quaternion(float(waypoint.get('yaw', 0.0)))
    pose.pose.orientation.x = quat['x']
    pose.pose.orientation.y = quat['y']
    pose.pose.orientation.z = quat['z']
    pose.pose.orientation.w = quat['w']
    return pose


def find_waypoint_index(route, waypoint_name):
    for index, waypoint in enumerate(route.get('waypoints', [])):
        if waypoint.get('name') == waypoint_name:
            return index
    raise ValueError(f'Waypoint "{waypoint_name}" was not found.')


def nearest_waypoint_name(route, x, y):
    waypoints = route.get('waypoints', [])
    if not waypoints:
        raise ValueError('Route has no waypoints.')

    nearest = min(
        waypoints,
        key=lambda waypoint: math.hypot(
            float(waypoint['x']) - x,
            float(waypoint['y']) - y,
        ),
    )
    return nearest.get('name')


def route_segment_waypoints(route, start_name=None, goal_name=None):
    waypoints = route.get('waypoints', [])
    if not waypoints:
        raise ValueError('Route has no waypoints.')

    start_index = 0
    goal_index = len(waypoints) - 1
    if start_name:
        start_index = find_waypoint_index(route, start_name)
    if goal_name:
        goal_index = find_waypoint_index(route, goal_name)

    if start_index > goal_index:
        return list(reversed(waypoints[goal_index:start_index + 1]))

    return waypoints[start_index:goal_index + 1]


def route_to_poses(route, start_name=None, goal_name=None):
    selected = route_segment_waypoints(route, start_name, goal_name)

    frame_id = route.get('frame_id', 'map')
    return [waypoint_to_pose(waypoint, frame_id) for waypoint in selected]
