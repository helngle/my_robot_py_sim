import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
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
    hybrid_params = os.path.join(
        table_share,
        'config',
        'hybrid_table_viewpoint.yaml',
    )
    rviz_config = os.path.join(
        bringup_share,
        'rviz',
        'hybrid_obstacle_test.rviz',
    )
    default_map = os.path.join(
        maps_share,
        'maps',
        'Test052601',
        'Test052601.yaml',
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
            'use_orbbec_pointcloud': 'false',
            'use_rgbd_goal': 'false',
            'use_rviz': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument(
            'lidar_source',
            default_value='livox',
            description='Use livox by default; set vmr for bridge pointcloud.',
        ),
        DeclareLaunchArgument('use_base_driver', default_value='true'),
        DeclareLaunchArgument('use_scan_conversion', default_value='true'),
        DeclareLaunchArgument('use_orbbec_camera', default_value='true'),
        DeclareLaunchArgument(
            'use_local_rviz',
            default_value='true',
            description='Start the RViz view for this local obstacle test.',
        ),
        navigation,
        Node(
            package='my_robot_table_viewpoint',
            executable='hybrid_viewpoint_orchestrator',
            name='hybrid_goal_router',
            output='screen',
            parameters=[
                hybrid_params,
                {
                    'viewpoint_goal_topic': '/hybrid_goal_pose',
                    'goal_topic_transient_local': False,
                    'distance_split_m': 1.5,
                    'enable_refinement': True,
                    'enable_final_yaw': True,
                    'refinement_mode': 'original_goal',
                    'ignore_short_goal_yaw': True,
                    'fine_position_tolerance_m': 0.05,
                    'fine_yaw_tolerance_rad': 3.14,
                    'max_refinement_yaw_rad': 3.14,
                    'final_yaw_tolerance_rad': 0.10,
                    'final_yaw_position_tolerance_m': 0.15,
                    'final_yaw_time_allowance_s': 10.0,
                    'auto_rearm': True,
                    'long_bt_xml': long_bt,
                    'short_bt_xml': short_bt,
                    'fine_bt_xml': fine_bt,
                },
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='hybrid_obstacle_test_rviz',
            condition=IfCondition(LaunchConfiguration('use_local_rviz')),
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
