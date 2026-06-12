import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot_py_sim')
    base_launch = os.path.join(
        pkg_my_robot,
        'launch',
        'navigation_with_rviz.launch.py',
    )
    nav2_params = os.path.join(
        pkg_my_robot,
        'config',
        'nav2_navigation_25d.yaml',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'use_25d_avoidance': 'true',
                'nav2_params': nav2_params,
            }.items(),
        ),
    ])
