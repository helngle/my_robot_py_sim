import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('my_robot_bringup')
    navigation_share = get_package_share_directory('my_robot_navigation')
    table_share = get_package_share_directory('my_robot_table_viewpoint')
    maps_share = get_package_share_directory('my_robot_maps')

    navigation_launch = os.path.join(
        bringup_share,
        'launch',
        'real_navigation_mppi_hybrid_distance.launch.py',
    )
    default_params = os.path.join(
        table_share,
        'config',
        'hybrid_table_viewpoint.yaml',
    )
    default_map = os.path.join(
        maps_share,
        'maps',
        'Test052601_table_viewpoint',
        'Test052601_table_viewpoint.yaml',
    )
    long_bt = os.path.join(
        navigation_share,
        'behavior_trees',
        'navigate_long_forward_replanning_if_path_invalid.xml',
    )
    short_bt = os.path.join(
        navigation_share,
        'behavior_trees',
        'navigate_short_omni_replanning_if_path_invalid.xml',
    )
    fine_bt = os.path.join(
        navigation_share,
        'behavior_trees',
        'navigate_fine_omni_replanning_if_path_invalid.xml',
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'lidar_source': LaunchConfiguration('lidar_source'),
            'use_base_driver': LaunchConfiguration('use_base_driver'),
            'use_scan_conversion': LaunchConfiguration('use_scan_conversion'),
            'use_orbbec_camera': LaunchConfiguration('use_orbbec_camera'),
            'use_rviz': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('lidar_source', default_value='livox'),
        DeclareLaunchArgument('use_base_driver', default_value='true'),
        DeclareLaunchArgument('use_scan_conversion', default_value='true'),
        DeclareLaunchArgument('use_orbbec_camera', default_value='true'),
        navigation,
        Node(
            package='my_robot_table_viewpoint',
            executable='hybrid_viewpoint_orchestrator',
            name='hybrid_viewpoint_orchestrator',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'long_bt_xml': long_bt,
                    'short_bt_xml': short_bt,
                    'fine_bt_xml': fine_bt,
                },
            ],
        ),
    ])
