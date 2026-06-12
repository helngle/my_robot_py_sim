import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'pose_estimator'
    pkg_share = get_package_share_directory(pkg_name)

    config_file = os.path.join(pkg_share, 'config', 'pose_estimator.yaml')

    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='pose_estimator_node',
            name='pose_estimetor', # As requested by user
            output='screen',
            parameters=[config_file]
        )
    ])
