import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        'my_robot_table_viewpoint'
    )
    maps_share = get_package_share_directory('my_robot_maps')
    default_params = os.path.join(
        package_share,
        'config',
        'table_viewpoint.yaml',
    )
    default_database = os.path.join(
        maps_share,
        'maps',
        'Test052601_table_viewpoint',
        'tables.yaml',
    )
    default_rviz_config = os.path.join(
        package_share,
        'rviz',
        'table_viewpoint.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
        ),
        DeclareLaunchArgument(
            'database_file',
            default_value=default_database,
        ),
        DeclareLaunchArgument('use_table_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
        ),
        Node(
            package='my_robot_table_viewpoint',
            executable='table_viewpoint_planner',
            name='table_viewpoint_planner',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'database_file': LaunchConfiguration('database_file'),
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='table_viewpoint_rviz',
            condition=IfCondition(LaunchConfiguration('use_table_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
        ),
    ])
