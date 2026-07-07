import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('my_robot_bringup')
    default_rviz_config = os.path.join(
        package_share,
        'rviz',
        'local_server_navigation.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='RViz configuration used by the local operator PC.',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='local_server_navigation_rviz',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
